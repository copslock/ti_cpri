/******************************************************************************
 *
 * Copyright (C) 2012-2019 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 *
 * dp_afe_sd0801.c
 *
 ******************************************************************************
 */

#include "dp_sd0801_if.h"
#include "dp_sd0801_priv.h"
#include "dp_sd0801_sanity.h"
#include "dp_regs.h"

#include "cdn_log.h"

#include "dp_sd0801_internal.h"

bool isPhySupported(const DP_SD0801_PrivateData* pD);

/**
 * Adapt address to bus and write to PHY APB.
 */
void afeWrite(const DP_SD0801_PrivateData* pD, uint32_t offset, uint16_t val)
{
    CPS_REG_WRITE(&(pD->regBase[offset]), val);
}

/**
 * Adapt address to bus and read from PHY APB.
 */
uint16_t afeRead(const DP_SD0801_PrivateData* pD, uint32_t offset)
{
    return (uint16_t)(CPS_REG_READ(&(pD->regBase[offset])));
}

/**
 * Write value to PHY register, but only if that's required (when value
 * provided to function is different from one already in register)
 */
void afeWriteChanged(const DP_SD0801_PrivateData* pD, uint32_t offset, uint16_t val)
{
    uint16_t reg_val = afeRead(pD, offset);
    if (val != reg_val)
    {
        afeWrite(pD, offset, val);
    }
}

/**
 * Fill table of voltage-related coefficients with default values.
 */
static uint32_t fillDefaultCoefficients(DP_SD0801_PrivateData* pD)
{
    uint8_t i;
    uint8_t j;
    uint32_t retVal = CDN_EOK;

    for (i = 0U; i < DP_SD0801_SWING_LEVEL_COUNT; i++) {
        for (j = 0U; j < DP_SD0801_EMPHASIS_LEVEL_COUNT; j++) {
            if (CDN_EOK == retVal) {
                /* Put values in driver's Private Data. */
                retVal = DP_SD0801_GetDefaultCoeffs(pD, i, j, &(pD->vCoeffs[i][j]));
            }
        }
    }

    return retVal;
}

/**
 * Get memory requirements of the driver
 */
uint32_t DP_SD0801_Probe(const DP_SD0801_Config* config, uint32_t* memReq)
{
    uint32_t retVal = CDN_EOK;
    if (CDN_EOK != DP_SD0801_ProbeSF(config, memReq)) {
        retVal = CDN_EINVAL;
    } else {
        /* No more memory, than for driver's Private Data, is required. */
        *memReq = (uint32_t)(sizeof(DP_SD0801_PrivateData));
    }
    return retVal;
}

uint32_t DP_SD0801_Init(DP_SD0801_PrivateData* pD, const DP_SD0801_Config* config)
{
    uint32_t retVal;
    uint8_t i;

    retVal = DP_SD0801_InitSF(pD, config);

    if (CDN_EOK == retVal) {
        /* Assign addresses of register bases from configuration. */
        pD->regBase = config->regBase;
        pD->regBaseDp = config->regBaseDp;
        pD->linkState.laneCount = 0; /* Indicates uninitialized PHY driver. */
        pD->linkState.linkRate = DP_SD0801_LINK_RATE_1_62;
        for (i = 0; i < 4U; i++)
        {
            pD->linkState.voltageSwing[i] = 0;
            pD->linkState.preEmphasis[i] = 0;
        }
        /* put default voltage coefficients to Private Data (may be modified later). */
        retVal = fillDefaultCoefficients(pD);
    }
    if (CDN_EOK == retVal)
    {
        if (!isPhySupported(pD))
        {
            retVal = CDN_ENOTSUP;
        }
    }

    return retVal;
}

/**
 * Assert or release PHY reset signal.
 */
uint32_t DP_SD0801_PhySetReset(const DP_SD0801_PrivateData* pD, bool reset)
{
    uint32_t regTmp;
    uint32_t retVal = CDN_EOK;

    retVal = DP_SD0801_PhySetResetSF(pD);

    if (CDN_EOK == retVal)
    {
        /* Set or clear PHY_RESET bit in register. */
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PHY_RESET_p);
        regTmp = CPS_FLD_WRITE(DP__DP_REGS__PHY_RESET_P, PHY_RESET, regTmp, (reset ? 0 : 1));
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, regTmp);

        CPS_ExtPhyReset(reset);
    }
    return retVal;
}

/**
 * Get current values of voltage-related PHY registers, for given voltage
 * swing and pre-emphasis.
 */
uint32_t DP_SD0801_GetCoefficients(const DP_SD0801_PrivateData*   pD,
                                   uint8_t                        voltageSwing,
                                   uint8_t                        preEmphasis,
                                   DP_SD0801_VoltageCoefficients* coefficients)
{
    uint32_t retVal;
    retVal = DP_SD0801_GetCoefficientsSF(pD, voltageSwing, preEmphasis, coefficients);

    if (CDN_EOK == retVal)
    {
        /* Fill the structure. */
        coefficients->DiagTxDrv = pD->vCoeffs[voltageSwing][preEmphasis].DiagTxDrv;
        coefficients->MgnfsMult = pD->vCoeffs[voltageSwing][preEmphasis].MgnfsMult;
        coefficients->CpostMult = pD->vCoeffs[voltageSwing][preEmphasis].CpostMult;
    }

    return retVal;
}

/**
 * Set values of voltage-related PHY registers, to be used for given
 * voltage swing and pre-emphasis.
 */
uint32_t DP_SD0801_SetCoefficients(DP_SD0801_PrivateData*               pD,
                                   uint8_t                              voltageSwing,
                                   uint8_t                              preEmphasis,
                                   const DP_SD0801_VoltageCoefficients* coefficients)
{
    uint32_t retVal;
    retVal = DP_SD0801_SetCoefficientsSF(pD, voltageSwing, preEmphasis, coefficients);

    if (CDN_EOK == retVal)
    {
        /* Set values from provided structure. */
        pD->vCoeffs[voltageSwing][preEmphasis].DiagTxDrv = coefficients->DiagTxDrv;
        pD->vCoeffs[voltageSwing][preEmphasis].MgnfsMult = coefficients->MgnfsMult;
        pD->vCoeffs[voltageSwing][preEmphasis].CpostMult = coefficients->CpostMult;
    }

    return retVal;
}

/**
 * Get current link status.
 */
uint32_t DP_SD0801_ReadLinkStat(const DP_SD0801_PrivateData* pD, DP_SD0801_LinkState* linkState)
{
    uint32_t retVal = DP_SD0801_ReadLinkStatSF(pD, linkState);
    if (CDN_EOK == retVal)
    {
        /* Copy structure from driver's private data to one provided by user. */
        CPS_BufferCopy((uint8_t*)linkState,
                       (const uint8_t*)(&(pD->linkState)),
                       (uint32_t)sizeof(DP_SD0801_LinkState));
    }

    return retVal;
}

/**
 * Automatically initialize and configure DP Main Link on PHY.
 */
uint32_t DP_SD0801_PhyStartUp(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate)
{
    uint32_t retVal;

    retVal = DP_SD0801_PhyStartUpSF(pD, linkRate);

    if (CDN_EOK == retVal) {
        /* Perform operations to be done before releasing reset. */
        retVal = DP_SD0801_PhyInit(pD, laneCount, linkRate);
    }

    if (CDN_EOK == retVal) {
        /* release PHY reset */
        retVal = DP_SD0801_PhySetReset(pD, false);
    }

    if (CDN_EOK == retVal) {
        retVal = DP_SD0801_WaitPmaCmnReady(pD);
    }

    if (CDN_EOK == retVal) {
        /* Perform operations to be done after releasing reset. */
        retVal = DP_SD0801_PhyRun(pD, laneCount);
    }

    return retVal;
}
