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
 * dp_dummy_spec.c
 *
 ******************************************************************************
 */

#include "dp_sd0801_if.h"
#include "dp_sd0801_priv.h"
#include "dp_sd0801_sanity.h"
#include "dp_regs.h"

#include "cdn_log.h"

#include "dp_sd0801_internal.h"

bool isPhySupported(const DP_SD0801_PrivateData* pD)
{
    return true;
}

uint32_t DP_SD0801_PhyInit(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate)
{
    /* Internal reg Addr 2 controls line for link rate. */
    /* B16 (0x8----) enables control from regs - otherwise dummy phy is */
    /* controlled by state on the lines. */
    uint32_t retVal;

    retVal = DP_SD0801_PhyInitSF(pD);

    if (CDN_EOK == retVal)
    {
        switch (linkRate)
        {
        case DP_SD0801_LINK_RATE_1_62:
            afeWrite(pD, 2, 0x8000);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 162\n");
            break;

        case DP_SD0801_LINK_RATE_2_70:
            afeWrite(pD, 2, 0x8001);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 270\n");
            break;

        case DP_SD0801_LINK_RATE_5_40:
            afeWrite(pD, 2, 0x8002);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 540\n");
            break;

        case DP_SD0801_LINK_RATE_8_10:
            afeWrite(pD, 2, 0x8004);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 810\n");
            break;

        case DP_SD0801_LINK_RATE_2_16:
            afeWrite(pD, 2, 0x8008);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 216\n");
            break;

        case DP_SD0801_LINK_RATE_2_43:
            afeWrite(pD, 2, 0x8010);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 243\n");
            break;

        case DP_SD0801_LINK_RATE_3_24:
            afeWrite(pD, 2, 0x8020);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 324\n");
            break;

        case DP_SD0801_LINK_RATE_4_32:
            afeWrite(pD, 2, 0x8040);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Set dummy rate to 432\n");
            break;

        default:
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "ERROR: Wrong rate code: 0x%x\n",linkRate);
            break;
        }
        /* Release PHY reset */
        DP_SD0801_PhySetReset(pD, false);

        pD->linkState.linkRate  = linkRate;
        pD->linkState.laneCount = laneCount;

    }

    return retVal;
}

/**
 * Enable DP Main Link lanes, after releasing PHY reset and waiting for PHY to
 * get ready.
 */
uint32_t DP_SD0801_PhyRun(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    return DP_SD0801_PhyRunSF(pD);
}

/**
 * Initialize part of PHY responsible for AUX channel.
 */
uint32_t DP_SD0801_ConfigurePhyAuxCtrl(const DP_SD0801_PrivateData* pD)
{
    return DP_SD0801_ConfigurePhyAuxCtrSF(pD);
}

/**
 * Wait, until PHY gets ready after releasing PHY reset signal.
 */
uint32_t DP_SD0801_WaitPmaCmnReady(const DP_SD0801_PrivateData* pD)
{
    return DP_SD0801_WaitPmaCmnReadySF(pD);
}

uint32_t DP_SD0801_ConfigLane(DP_SD0801_PrivateData* pD, uint8_t lane, const DP_SD0801_LinkState* linkState)
{
    uint32_t retVal = DP_SD0801_ConfigLaneSF(pD, linkState);
    if (CDN_EOK == retVal) {
        /* Store new settings in pD. */
        pD->linkState.voltageSwing[lane] = linkState->voltageSwing[lane];
        pD->linkState.preEmphasis[lane] = linkState->preEmphasis[lane];
    }

    return retVal;
}

uint32_t DP_SD0801_SetLinkRate(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState)
{
    uint32_t retVal;

    retVal = DP_SD0801_SetLinkRateSF(pD, linkState);
    if (CDN_EOK == retVal) {
        /* Store new settings in pD. */
        pD->linkState.linkRate = linkState->linkRate;
        pD->linkState.laneCount = linkState->laneCount;
    }

    return retVal;
}

uint32_t DP_SD0801_EnableLanes(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState)
{
    /* set active lane count (1, 2 or 4) as in linkState->laneCount. */

    uint32_t retVal;

    retVal = DP_SD0801_EnableLanesSF(pD, linkState);
    if (CDN_EOK == retVal) {
        /* Store new setting in pD. */
        pD->linkState.laneCount = linkState->laneCount;
    }

    return retVal;
}

uint32_t DP_SD0801_GetDefaultCoeffs(const DP_SD0801_PrivateData*   pD,
                                    uint8_t                        voltageSwing,
                                    uint8_t                        preEmphasis,
                                    DP_SD0801_VoltageCoefficients* coefficients)
{
    uint32_t retVal;
    retVal = DP_SD0801_GetDefaultCoeffsSF(pD, voltageSwing, preEmphasis, coefficients);

    if (CDN_EOK == retVal)
    {
        coefficients->DiagTxDrv = 0xFFFF;
        coefficients->MgnfsMult = 0xFFFF;
        coefficients->CpostMult = 0xFFFF;
    }

    return retVal;
}
