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
 * dp_sd0801_spec.c
 *
 ******************************************************************************
 */

#include "dp_sd0801_if.h"
#include "dp_sd0801_priv.h"
#include "dp_sd0801_sanity.h"
#include "dp_regs.h"

#include "cdn_log.h"

#include "dp_sd0801_internal.h"
#include "dp_sd0801_spec.h"

typedef struct phyRegValue_t
{
    uint32_t addr;
    uint16_t val;
} phyRegValue;

typedef enum
{
    POWERSTATE_A0 = 0U,
    /* Powerstate A1 is unused */
    POWERSTATE_A2 = 2U,
    POWERSTATE_A3 = 3U,
} phyPowerstate;

#define PID_TYPE_SD (uint16_t)0x7364 /* ASCII for "SD", SerDes */
#define PID_NUM_0801 (uint16_t)0x0801

bool isPhySupported(const DP_SD0801_PrivateData* pD);

/**
 * Return true for supported device, false for unsupported one)
 */
bool isPhySupported(const DP_SD0801_PrivateData* pD)
{
    bool retVal = true;
    const uint16_t pidType = afeRead(pD, CMN_PID_TYPE);
    const uint16_t pidNum = afeRead(pD, CMN_PID_NUM);

    if ((pidType != PID_TYPE_SD) || (pidNum != PID_NUM_0801))
    {
        retVal = false;
    }

    return retVal;
}

#ifdef REF_CLK_19_2MHz

#define TxRcvdetStTmrVal 0x0780 /* Avoids MISRA C violation. */

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 19.2 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg19[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x0014},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0027},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00A1},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0027},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00A1},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x0060},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x0060},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x0014},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x0018},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x0018},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x0240},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x000B},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x0137},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x00C0},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x0260},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
#ifdef HAVE_CMN_PLL1
        {.addr = CMN_PDIAG_PLL1_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL1_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL1_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL1_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x00C0},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL1_VCOCAL_REFTIM_START, .val = 0x0260},
        {.addr = CMN_PLL1_VCOCAL_TCTRL, .val = 0x0003},
#endif
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg19) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg19[i].addr, PhyPmaCmnCfg19[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#elif defined REF_CLK_20MHz

#define TxRcvdetStTmrVal 0x07D0

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 20 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg20[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x0014},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0028},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00A8},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0028},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00A8},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x0064},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x0064},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x0014},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x0018},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x0018},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x0258},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x000B},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x012B},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x00C8},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x00C8},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x0279},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg20) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg20[i].addr, PhyPmaCmnCfg20[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#elif defined REF_CLK_24MHz

#define TxRcvdetStTmrVal 0x0960

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 24 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg24[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x0018},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0030},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00C9},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0030},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00C9},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x0078},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x0078},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x0018},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x001D},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x001D},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x02D0},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x000E},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x0137},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x00F0},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x00F0},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x02F8},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg24) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg24[i].addr, PhyPmaCmnCfg24[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#elif defined REF_CLK_26MHz

#define TxRcvdetStTmrVal 0x0A28

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 26 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg26[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x001A},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0034},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00DA},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0034},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00DA},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x0082},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x0082},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x001A},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x0020},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x0020},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x030C},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0003},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0003},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x000F},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x0132},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x0104},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x0104},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x0337},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg26) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg26[i].addr, PhyPmaCmnCfg26[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#elif defined REF_CLK_27MHz

#define TxRcvdetStTmrVal 0x0A8C

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 27 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg27[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x001B},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0036},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00E2},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0036},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00E2},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x0087},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x0087},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x001B},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x0021},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x0021},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x032A},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0007},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0003},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0003},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x0010},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x0139},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x010E},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x010E},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0005},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x0357},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg27) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg27[i].addr, PhyPmaCmnCfg27[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#else /* 25 MHz - default */

#define TxRcvdetStTmrVal 0x09C4

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
static void configurePhyPmaCmnCfg(const DP_SD0801_PrivateData* pD)
{
    /* Configuring for 25 MHz reference clock */
    /* Values of registers to write are taken from sd0801 PHY programming guide. */
    static const phyRegValue PhyPmaCmnCfg25[] = {
        /* Refclock Registers */
        {.addr = CMN_SSM_BIAS_TMR, .val = 0x0019},
        {.addr = CMN_PLLSM0_PLLPRE_TMR, .val = 0x0032},
        {.addr = CMN_PLLSM0_PLLLOCK_TMR, .val = 0x00D1},
        {.addr = CMN_PLLSM1_PLLPRE_TMR, .val = 0x0032},
        {.addr = CMN_PLLSM1_PLLLOCK_TMR, .val = 0x00D1},
        {.addr = CMN_BGCAL_INIT_TMR, .val = 0x007D},
        {.addr = CMN_BGCAL_ITER_TMR, .val = 0x007D},
        {.addr = CMN_IBCAL_INIT_TMR, .val = 0x0019},
        {.addr = CMN_TXPUCAL_INIT_TMR, .val = 0x001E},
        {.addr = CMN_TXPUCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_TXPDCAL_INIT_TMR, .val = 0x001E},
        {.addr = CMN_TXPDCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_RXCAL_INIT_TMR, .val = 0x02EE},
        {.addr = CMN_RXCAL_ITER_TMR, .val = 0x0006},
        {.addr = CMN_SD_CAL_INIT_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_ITER_TMR, .val = 0x0002},
        {.addr = CMN_SD_CAL_REFTIM_START, .val = 0x000E},
        {.addr = CMN_SD_CAL_PLLCNT_START, .val = 0x012B},
        /* PLL Registers */
        {.addr = CMN_PDIAG_PLL0_CP_PADJ_M0, .val = 0x0509},
        {.addr = CMN_PDIAG_PLL0_CP_IADJ_M0, .val = 0x0F00},
        {.addr = CMN_PDIAG_PLL0_FILT_PADJ_M0, .val = 0x0F08},
        {.addr = CMN_PLL0_DSM_DIAG_M0, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_INIT_TMR, .val = 0x00FA},
        {.addr = CMN_PLL0_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL1_VCOCAL_INIT_TMR, .val = 0x00FA},
        {.addr = CMN_PLL1_VCOCAL_ITER_TMR, .val = 0x0004},
        {.addr = CMN_PLL0_VCOCAL_REFTIM_START, .val = 0x0317},
        {.addr = CMN_PLL0_VCOCAL_TCTRL, .val = 0x0003},
    };
    uint32_t i, regCount;
    regCount = (uint32_t)sizeof(PhyPmaCmnCfg25) / (uint32_t)sizeof(phyRegValue);
    for (i = 0; i < regCount; i++)
    {
        afeWrite(pD, PhyPmaCmnCfg25[i].addr, PhyPmaCmnCfg25[i].val);
    }
}
/* parasoft-end-suppress METRICS-39-3 */

#endif /* refclock */

/**
 * This driver supports only DPTX.
 */
static void disableRx(const DP_SD0801_PrivateData* pD, uint32_t laneOffset)
{
    uint32_t regsToClear[] = {
        /* Powerstate-related */
        RX_PSC_A0,
        RX_PSC_A2,
        RX_PSC_A3,
        RX_PSC_CAL,

        RX_REE_GCSM1_CTRL,
        RX_REE_GCSM2_CTRL,
        RX_REE_PERGCSM_CTRL
    };
    uint8_t i;

    for (i = 0; i < (sizeof(regsToClear) / sizeof(uint32_t)); i++)
    {
        afeWrite(pD, (uint32_t)(regsToClear[i] | laneOffset), 0x0000U);
    }
}

static void configurePhyPmaLnDpCfg(const DP_SD0801_PrivateData* pD, uint8_t lane)
{
    /* DP supports lanes 0-3 - wrap. */
    /* uint32_t used for consistency with bitwise operations. */
    const uint32_t i = 0x0003U & (uint32_t)lane;

    /* Bits 9 and 10 of address indicate lane number. */
    const uint32_t laneOffset = (i << 9);

#ifdef TxRcvdetStTmrVal
    const uint16_t rcvdetVal = (uint16_t)TxRcvdetStTmrVal;
    /* Per lane, refclock-dependent receiver detection setting. */
    afeWrite(pD, (uint32_t)(TX_RCVDET_ST_TMR | laneOffset), rcvdetVal);
#endif /* for reflock of 100 MHz, use reset value */

    /* Writing Tx/Rx Power State Controllers Registers */

    /* 2.8.3 Display Port / Embedded Display Port */
    afeWrite(pD, (uint32_t)(TX_PSC_A0 | laneOffset), 0x00FBU);
    afeWrite(pD, (uint32_t)(TX_PSC_A2 | laneOffset), 0x04AAU);
    afeWrite(pD, (uint32_t)(TX_PSC_A3 | laneOffset), 0x04AAU);

    disableRx(pD, laneOffset);

    afeWrite(pD, (uint32_t)(XCVR_DIAG_BIDI_CTRL | laneOffset), 0x000F);
    afeWrite(pD, (uint32_t)(XCVR_DIAG_PLLDRC_CTRL | laneOffset), 0x0001U);
    afeWrite(pD, (uint32_t)(XCVR_DIAG_HSCLK_SEL | laneOffset), 0x0000U);
}

static void configurePhyPmaDpCfg(const DP_SD0801_PrivateData* pD, uint8_t linkCfg)
{
    uint8_t i;
    /* PMA cmn configuration */
    configurePhyPmaCmnCfg(pD);

    /* PMA lane configuration to deal with multi-link operation */
    for (i = 0; i < 4U; i++) {
        /* Configure lane for DP operation */
        if (0U != (linkCfg & (1U << i))) {
            configurePhyPmaLnDpCfg(pD, i);
        }
    }
}

static ENUM_VCO_FREQ getVcoFreq(DP_SD0801_LinkRate linkRate)
{
    ENUM_VCO_FREQ retVal;

    switch (linkRate)
    {
    case DP_SD0801_LINK_RATE_1_62:
        retVal = VCO_9GHz72_refclk; /* Setting VCO for 9.72GHz */
        break;
    case DP_SD0801_LINK_RATE_2_16:
        retVal = VCO_8GHz64_refclk; /* Setting VCO for 8.64GHz */
        break;
    case DP_SD0801_LINK_RATE_2_43:
        retVal = VCO_9GHz72_refclk; /* Setting VCO for 9.72GHz */
        break;
    case DP_SD0801_LINK_RATE_2_70:
        retVal = VCO_10GHz8_refclk; /* Setting VCO for 10.8GHz */
        break;
    case DP_SD0801_LINK_RATE_3_24:
        retVal = VCO_9GHz72_refclk; /* Setting VCO for 9.72GHz */
        break;
    case DP_SD0801_LINK_RATE_4_32:
        retVal = VCO_8GHz64_refclk; /* Setting VCO for 8.64GHz */
        break;
    case DP_SD0801_LINK_RATE_5_40:
        retVal = VCO_10GHz8_refclk; /* Setting VCO for 10.8GHz */
        break;
    case DP_SD0801_LINK_RATE_8_10:
        retVal = VCO_8GHz1_refclk;  /* Setting VCO for 8.1GHz */
        break;
    default:
        retVal = VCO_9GHz72_refclk;
        break;
    }
    return retVal;
}

#ifdef REF_CLK_19_2MHz

/**
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val, uint16_t ctrl3Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0003);

#ifdef HAVE_CMN_PLL1
    afeWrite(pD, CMN_PLL1_SS_CTRL1_M0, 0x0001); /*  Enable SSC */
    afeWrite(pD, CMN_PLL1_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL1_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL1_SS_CTRL4_M0, 0x0003);
#endif
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 19.2MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0119);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x4000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00BC);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0012);

#ifdef HAVE_CMN_PLL1
    afeWrite(pD, CMN_PLL1_INTDIV_M0, 0x0119);
    afeWrite(pD, CMN_PLL1_FRACDIVL_M0, 0x4000);
    afeWrite(pD, CMN_PLL1_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL1_HIGH_THR_M0, 0x00BC);
    afeWrite(pD, CMN_PDIAG_PLL1_CTRL_M0, 0x0012);
#endif

    if (ssc)
    {
        enableSsc(pD, 0x033A, 0x006A); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 19.2MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01FA);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x4000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0152);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

#ifdef HAVE_CMN_PLL1
    afeWrite(pD, CMN_PLL1_INTDIV_M0, 0x01FA);
    afeWrite(pD, CMN_PLL1_FRACDIVL_M0, 0x4000);
    afeWrite(pD, CMN_PLL1_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL1_HIGH_THR_M0, 0x0152);
    afeWrite(pD, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
#endif

    if (ssc)
    {
        enableSsc(pD, 0x05DD, 0x0069); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 19.2MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01C2);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x012C);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

#ifdef HAVE_CMN_PLL1
    afeWrite(pD, CMN_PLL1_INTDIV_M0, 0x01C2);
    afeWrite(pD, CMN_PLL1_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL1_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL1_HIGH_THR_M0, 0x012C);
    afeWrite(pD, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
#endif

    if (ssc)
    {
        enableSsc(pD, 0x0536, 0x0069); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 19.2MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01A5);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0xE000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x011A);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

#ifdef HAVE_CMN_PLL1
    /*  Setting VCO for 8.1GHz -- 19.2MHz */
    afeWrite(pD, CMN_PLL1_INTDIV_M0, 0x01A5);
    afeWrite(pD, CMN_PLL1_FRACDIVL_M0, 0xE000);
    afeWrite(pD, CMN_PLL1_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL1_HIGH_THR_M0, 0x011A);
    afeWrite(pD, CMN_PDIAG_PLL1_CTRL_M0, 0x0002);
#endif

    if (ssc)
    {
        enableSsc(pD, 0x04D7, 0x006A); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 19.2 MHz */
    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x025E);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0260);
        /* Set reset register values to disable SSC. */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x0099);
    afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x0099);

#ifdef HAVE_CMN_PLL1
    /*  Settings common for all VCOs - 19.2 MHz */
    if (ssc)
    {
        /*  Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL1_VCOCAL_PLLCNT_START, 0x025E);
        afeWrite(pD, CMN_PLL1_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL1_VCOCAL_PLLCNT_START, 0x0260);
        /*  Set reset register values to disable SSC. */
        afeWrite(pD, CMN_PLL1_SS_CTRL1_M0, 0x0002); /*  Disable SSC */
        afeWrite(pD, CMN_PLL1_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL1_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL1_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL1_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL1_LOCK_REFCNT_START, 0x0099);
    afeWrite(pD, CMN_PLL1_LOCK_PLLCNT_START, 0x0099);
#endif
}

#elif defined REF_CLK_20MHz

/**
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val, uint16_t ctrl3Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0003);
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 20MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x010E);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00B4);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0012);
    if (ssc)
    {
        enableSsc(pD, 0x05F8, 0x006E); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 20MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01E6);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0144);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
    if (ssc)
    {
        enableSsc(pD, 0x0553, 0x006F); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 20MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01B0);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0120);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
    if (ssc)
    {
        enableSsc(pD, 0x04DD, 0x006C); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 20MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0195);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x010E);
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);
    if (ssc)
    {
        enableSsc(pD, 0x047A, 0x006E); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 20 MHz */
    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0277);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0279);
        /* Set reset register values to disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x009F);
    afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x009F);
}

#elif defined REF_CLK_24MHz

/**
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val, uint16_t ctrl3Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0003);
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 24MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01C2);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x012C);
    if (ssc)
    {
        enableSsc(pD, 0x044F, 0x007F); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 24MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0195);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x010E);
    if (ssc)
    {
        enableSsc(pD, 0x0401, 0x007B); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 24MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0168);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00F0);

    if (ssc)
    {
        enableSsc(pD, 0x038F, 0x007B); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 24MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0151);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x8000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00E2);
    if (ssc)
    {
        enableSsc(pD, 0x0342, 0x007E); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 24 MHz */
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x02F6);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x02F8);
        /* Set reset register values to disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x00BF);
    afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00BF);

}

#elif defined REF_CLK_26MHz

/**
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val, uint16_t ctrl3Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0004);
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 26MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x019F);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x6276);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0115);
    if (ssc)
    {
        enableSsc(pD, 0x04C4, 0x006A); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 26MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0175);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0xD89E);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00FA);
    if (ssc)
    {
        enableSsc(pD, 0x044A, 0x006A); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 26MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x014C);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x4EC5);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00DE);
    if (ssc)
    {
        enableSsc(pD, 0x03D0, 0x006A); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 26MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0137);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x89D9);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00D0);
    if (ssc)
    {
        enableSsc(pD, 0x0382, 0x006C); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 26 MHz */
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0335);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00CE);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0337);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00CF);
        /* Set reset register values to disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x00CF);
}

#elif defined REF_CLK_27MHz

/**
 * Set registers responsible for enabling and configuring SSC, with second and
 * third register values provided by parameters.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val, uint16_t ctrl3Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, ctrl3Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0004);
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 27MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0190);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x010C);
    if (ssc)
    {
        enableSsc(pD, 0x046C, 0x006E); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 27MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0168);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00F0);
    if (ssc)
    {
        enableSsc(pD, 0x0404, 0x006D); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 27MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0140);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00D6);
    if (ssc)
    {
        enableSsc(pD, 0x03A3, 0x006B); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 27MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x012C);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00C8);
    if (ssc)
    {
        enableSsc(pD, 0x0351, 0x006E); /* values specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 27 MHz */
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0355);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00D6);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0357);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00D7);
        /* Set reset register values to disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x00D7);
}

#else /* 25 MHz - default */

/**
 * Set registers responsible for enabling and configuring SSC, with second
 * register value provided by a parameter.
 */
static void enableSsc(const DP_SD0801_PrivateData* pD, uint16_t ctrl2Val)
{
    afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0001); /* Enable SSC */
    afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, ctrl2Val);
    afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x007F);
    afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0003);
}

static void configurePhyPmaCmnVcoCfg10_8(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 10.8GHz -- 25MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x01B0);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0120);
    if (ssc)
    {
        enableSsc(pD, 0x0423); /* value specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg9_72(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 9.72GHz -- 25MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0184);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0xCCCD);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x0104);
    if (ssc)
    {
        enableSsc(pD, 0x03B9); /* value specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_64(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.64GHz -- 25MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0159);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x999A);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00E8);
    if (ssc)
    {
        enableSsc(pD, 0x034F); /* value specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCfg8_1(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Setting VCO for 8.1GHz -- 25MHz */
    afeWrite(pD, CMN_PLL0_INTDIV_M0, 0x0144);
    afeWrite(pD, CMN_PLL0_FRACDIVL_M0, 0x0000);
    afeWrite(pD, CMN_PLL0_FRACDIVH_M0, 0x0002);
    afeWrite(pD, CMN_PLL0_HIGH_THR_M0, 0x00D8);
    if (ssc)
    {
        enableSsc(pD, 0x031A); /* value specific for VCO. */
    }
}

static void configurePhyPmaCmnVcoCommon(const DP_SD0801_PrivateData* pD, bool ssc)
{
    /* Settings common for all VCOs - 25 MHz */
    afeWrite(pD, CMN_PDIAG_PLL0_CTRL_M0, 0x0002);

    if (ssc)
    {
        /* Settings for SSC enabled. */
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0315);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0005);
    } else {
        afeWrite(pD, CMN_PLL0_VCOCAL_PLLCNT_START, 0x0317);
        /* Set reset register values to disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL1_M0, 0x0002); /* Disable SSC */
        afeWrite(pD, CMN_PLL0_SS_CTRL2_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL3_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_SS_CTRL4_M0, 0x0000);
        afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_THR, 0x0003);
    }

    afeWrite(pD, CMN_PLL0_LOCK_REFCNT_START, 0x00C7);
    afeWrite(pD, CMN_PLL0_LOCK_PLLCNT_START, 0x00C7);
}

#endif /* refclock */

/* Configure PLL for requested VCO frequency. */
static void configurePhyPmaCmnVcoCfg(const DP_SD0801_PrivateData* pD, ENUM_VCO_FREQ vco_freq, bool ssc)
{
    /* Perform register writes specific to VCO frequency. */
    switch (vco_freq)
    {
    case VCO_10GHz8_refclk:
        configurePhyPmaCmnVcoCfg10_8(pD, ssc);
        break;
    case VCO_9GHz72_refclk:
        configurePhyPmaCmnVcoCfg9_72(pD, ssc);
        break;
    case VCO_8GHz64_refclk:
        configurePhyPmaCmnVcoCfg8_64(pD, ssc);
        break;
    default:
        configurePhyPmaCmnVcoCfg8_1(pD, ssc);
        break;
    }
    /* Write register values common for all VCO frequencies. */
    configurePhyPmaCmnVcoCommon(pD, ssc);
}

static bool isPllSet(uint8_t pllBits, uint8_t pllIdx)
{
    bool result = false;
    if (pllIdx < 2U) {
        /* Check, if bit for particular PLL (0 or 1) is set. */
        if (0U != ((pllBits >> pllIdx) & 1U)) {
            result = true;
        }
    }

    return result;
}

static uint16_t getClkSelM0Val(DP_SD0801_LinkRate linkRate)
{
    uint16_t clkSelM0Val;
    switch (linkRate)
    {
    /* Rate: 1.62G */
    case (DP_SD0801_LINK_RATE_1_62):
        clkSelM0Val = 0x0F01;
        break;
    /* Rate: 2.16G */
    case (DP_SD0801_LINK_RATE_2_16):
        clkSelM0Val = 0x0701;
        break;
    /* Rate: 2.43G */
    case (DP_SD0801_LINK_RATE_2_43):
        clkSelM0Val =  0x0701;
        break;
    /* Rate: 2.7G */
    case (DP_SD0801_LINK_RATE_2_70):
        clkSelM0Val =  0x0701;
        break;
    /* Rate: 3.24G */
    case (DP_SD0801_LINK_RATE_3_24):
        clkSelM0Val =  0x0B00;
        break;
    /* Rate: 4.32G */
    case (DP_SD0801_LINK_RATE_4_32):
        clkSelM0Val =  0x0301;
        break;
    /* Rate: 5.4G */
    case (DP_SD0801_LINK_RATE_5_40):
        clkSelM0Val =  0x0301;
        break;
    /* Rate: 8.1G */
    default:
        clkSelM0Val =  0x0200;
        break;
    }

    return clkSelM0Val;
}

static uint16_t getHsclkDivVal(DP_SD0801_LinkRate linkRate)
{
    uint16_t hsclkDivVal;

    switch (linkRate)
    {
    /* ******* Writing XCVR_DIAG_HSCLK_DIV Register for Lane %d ******* */
    /* Rate: 1.62G */
    case (DP_SD0801_LINK_RATE_1_62):
        hsclkDivVal = 0x0002U;
        break;
    /* Rate: 2.16G */
    case (DP_SD0801_LINK_RATE_2_16):
        hsclkDivVal = 0x0001U;
        break;
    /* Rate: 2.43G */
    case (DP_SD0801_LINK_RATE_2_43):
        hsclkDivVal = 0x0001U;
        break;
    /* Rate: 2.7G */
    case (DP_SD0801_LINK_RATE_2_70):
        hsclkDivVal = 0x0001U;
        break;
    /* Rate: 3.24G */
    case (DP_SD0801_LINK_RATE_3_24):
        hsclkDivVal = 0x0002U;
        break;
    /* Rate: 4.32G */
    case (DP_SD0801_LINK_RATE_4_32):
        hsclkDivVal = 0x0000U;
        break;
    /* Rate: 5.4G */
    case (DP_SD0801_LINK_RATE_5_40):
        hsclkDivVal = 0x0000U;
        break;
    /* Rate: 8.1G */
    default:
        hsclkDivVal = 0x0000U;
        break;
    }

    return hsclkDivVal;
}

static void configurePhyPmaCmnDpRate(const DP_SD0801_PrivateData* pD, uint8_t linkCfg, DP_SD0801_LinkRate dp_rate, uint8_t dp_pll)
{
    uint16_t hsclkDivVal = getHsclkDivVal(dp_rate);
    /* uint32_t used for consistency with bitwise operations. */
    uint32_t i;

    /* 16’h0000 for single DP link configuration */
    /* 16’h0002 for 2 DP link configuration */
    afeWrite(pD, PHY_PLL_CFG, 0x0000);

    /* Configure appropriate PLL (0 / 1) */
    if (isPllSet(dp_pll, 0)) {
        afeWrite(pD, CMN_PDIAG_PLL0_CLK_SEL_M0, getClkSelM0Val(dp_rate));
    }
    if (isPllSet(dp_pll, 1)) {
        afeWrite(pD, CMN_PDIAG_PLL1_CLK_SEL_M0, getClkSelM0Val(dp_rate));
    }

    /* PMA lane configuration to deal with multi-link operation */
    for (i = 0; i < 4U; i++) /* depends of active lane. If not it results in timeout */
    {
        if (0U != (linkCfg & (1U << i)))
        {
            /* ******* Writing XCVR_DIAG_HSCLK_DIV Register for Lane 'i' ******* */
            afeWrite(pD, (XCVR_DIAG_HSCLK_DIV | (i << 9U)), hsclkDivVal);
        }
    }
}

static void setA0PowerRegPwrState(uint32_t* pwrState, uint8_t laneCount)
{
    /* lane 0 */
    *pwrState = CPS_FLD_WRITE(DP__DP_REGS__PMA_POWER_STATE_REQ_P, PMA_XCVR_POWER_STATE_REQ_LN_0, *pwrState, 0x00);

    if (laneCount > 1U)
    {
        /* lane 1 */
        *pwrState = CPS_FLD_WRITE(DP__DP_REGS__PMA_POWER_STATE_REQ_P, PMA_XCVR_POWER_STATE_REQ_LN_1, *pwrState, 0x00);
    }

    if (laneCount > 2U)
    {
        /* lanes 2 and 3 */
        *pwrState = CPS_FLD_WRITE(DP__DP_REGS__PMA_POWER_STATE_REQ_P, PMA_XCVR_POWER_STATE_REQ_LN_2, *pwrState, 0x00);
        *pwrState = CPS_FLD_WRITE(DP__DP_REGS__PMA_POWER_STATE_REQ_P, PMA_XCVR_POWER_STATE_REQ_LN_3, *pwrState, 0x00);
    }
}

static void setA0PowerRegPllclkEn(uint32_t* pllclkEn, uint8_t laneCount)
{
    /* lane 0 */
    *pllclkEn = CPS_FLD_WRITE(DP__DP_REGS__PMA_PLLCLK_EN_P, PMA_XCVR_PLLCLK_EN_LN_0, *pllclkEn, 0x00);

    if (laneCount > 1U)
    {
        /* lane 1 */
        *pllclkEn = CPS_FLD_WRITE(DP__DP_REGS__PMA_PLLCLK_EN_P, PMA_XCVR_PLLCLK_EN_LN_1, *pllclkEn, 0x00);
    }

    if (laneCount > 2U)
    {
        /* lanes 2 and 3 */
        *pllclkEn = CPS_FLD_WRITE(DP__DP_REGS__PMA_PLLCLK_EN_P, PMA_XCVR_PLLCLK_EN_LN_2, *pllclkEn, 0x00);
        *pllclkEn = CPS_FLD_WRITE(DP__DP_REGS__PMA_PLLCLK_EN_P, PMA_XCVR_PLLCLK_EN_LN_3, *pllclkEn, 0x00);
    }
}

/**
 * Set lines power state to A0
 * Set lines pll clk enable to 0
 */
static void setPowerA0(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    uint32_t pwrState = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_POWER_STATE_REQ_p);
    uint32_t pllclkEn = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_p);

    setA0PowerRegPwrState(&pwrState, laneCount);
    setA0PowerRegPllclkEn(&pllclkEn, laneCount);

    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_POWER_STATE_REQ_p, pwrState);
    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_p, pllclkEn);
}

uint32_t DP_SD0801_PhyInit(DP_SD0801_PrivateData* pD, uint8_t laneCount, DP_SD0801_LinkRate linkRate)
{
    uint32_t regTmp;
    uint8_t lane_cfg = 0U;
#ifdef HAVE_CMN_PLL1
    uint8_t dp_pll = 3;
#else
    uint8_t dp_pll = 1;
#endif
    uint32_t retVal = CDN_EOK;

    retVal = DP_SD0801_PhyInitSF(pD);

    if (CDN_EOK == retVal)
    {
        if ((laneCount <= 4U) && (laneCount >  0U))
        {
            lane_cfg = (uint8_t)((1U << (laneCount)) - 1U);
        }

        /* PHY PMA registers configuration function */
        configurePhyPmaDpCfg(pD, lane_cfg);

        setPowerA0(pD, laneCount);

        /* release phy_l0*_reset_n and pma_tx_elec_idle_ln_* based on used laneCount */
        regTmp = ((0x000FU & ~(uint32_t)lane_cfg) << 4U) | (0x000FU & (uint32_t)lane_cfg);
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, regTmp);

        /* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_p, 0x0001);

        /* PHY PMA registers configuration functions */
        /* Set SSC disabled on init, can be enabled on link rate change. */
        configurePhyPmaCmnVcoCfg(pD, getVcoFreq(linkRate), false);
        configurePhyPmaCmnDpRate(pD, lane_cfg,linkRate, dp_pll);

        pD->linkState.linkRate  = linkRate;
        pD->linkState.laneCount = laneCount;
    }

    return retVal;
}

static void setPowerState(const DP_SD0801_PrivateData* pD, phyPowerstate pwrState, uint8_t laneCount)
{
    uint32_t regTmp;

    /* Register value for power state for a single byte. */
    uint32_t pmaPowerStateValPart;

    uint32_t pmaPowerStateVal;
    uint32_t pmaPowerStateMask;

    switch (pwrState)
    {
    case (POWERSTATE_A0):
        pmaPowerStateValPart = 0x01U;
        break;
    case (POWERSTATE_A2):
        pmaPowerStateValPart = 0x04U;
        break;
    default:
        pmaPowerStateValPart = 0x08U;
        break;
    }

    /* Select values of registers and mask, depending on enabled lane count. */
    switch (laneCount)
    {
    /* lane 0 */
    case (0x0001):
        pmaPowerStateVal = pmaPowerStateValPart;
        pmaPowerStateMask = 0x0000003FU;
        break;
    /* lanes 0-1 */
    case (0x0002):
        pmaPowerStateVal = (pmaPowerStateValPart
                            | (pmaPowerStateValPart << 8));
        pmaPowerStateMask = 0x00003F3FU;
        break;
    /* lanes 0-3, all */
    default:
        pmaPowerStateVal = (pmaPowerStateValPart
                            | (pmaPowerStateValPart << 8)
                            | (pmaPowerStateValPart << 16)
                            | (pmaPowerStateValPart << 24));
        pmaPowerStateMask = 0x3F3F3F3FU;
        break;
    }

    /* Set power state A<n> */
    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_POWER_STATE_REQ_p, pmaPowerStateVal);
    /* Wait, until PHY acknowledges power state completion */
    do {
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_POWER_STATE_ACK_p);
    } while (((regTmp) & pmaPowerStateMask) != pmaPowerStateVal);
    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_POWER_STATE_REQ_p, 0x00000000U);
    CPS_DelayNs(100);
}

/**
 * Version of function DP_SD0801_PhyRun internal to driver.
 * To be used, if parameters (sanity) are ensured to be correct by caller.
 */
static void phyRun(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    uint32_t regTmp;

    /* waiting for ACK of pma_xcvr_pllclk_en_ln_*, only for the master lane */
    do {
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_ACK_p);
    } while (((regTmp) & 0x0001U) == 0x0000U);

    CPS_DelayNs(100);

    setPowerState(pD, POWERSTATE_A2, laneCount);
    setPowerState(pD, POWERSTATE_A0, laneCount);
}

/**
 * Enable DP Main Link lanes, after releasing PHY reset and waiting for PHY to
 * get ready.
 */
uint32_t DP_SD0801_PhyRun(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SD0801_PhyRunSF(pD);

    if (CDN_EOK == retVal)
    {
        phyRun(pD, laneCount);
    }
    return retVal;
}

/**
 * Initialize part of PHY responsible for AUX channel.
 */
uint32_t DP_SD0801_ConfigurePhyAuxCtrl(const DP_SD0801_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SD0801_ConfigurePhyAuxCtrSF(pD);

    if (CDN_EOK == retVal)
    {
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.AUX_CTRL_p, 0x0003);
    }
    return retVal;
}

/**
 * Version of function DP_SD0801_waitPmaCmnReady internal to driver.
 * To be used, if parameter (sanity) is ensured to be correct by caller.
 */
static void waitPmaCmnReady(const DP_SD0801_PrivateData* pD)
{
    uint32_t regTmp;

    do {
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_CMN_READY_p);
    } while (((regTmp) & 1U) == 0U);
}

/**
 * Wait, until PHY gets ready after releasing PHY reset signal.
 */
uint32_t DP_SD0801_WaitPmaCmnReady(const DP_SD0801_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SD0801_WaitPmaCmnReadySF(pD);
    if (CDN_EOK == retVal)
    {
        waitPmaCmnReady(pD);
    }

    return retVal;
}

/* ------------------------------------------------------------------------------------------------------------------- */

uint32_t DP_SD0801_ConfigLane(DP_SD0801_PrivateData* pD, uint8_t lane, const DP_SD0801_LinkState* linkState)
{
    /* set voltage swing level (0 - 3) for lane (passed as parameter "lane", */
    /* ranging 0-3) as in linkState->voltageSwing[lane] */
    /* set pre-emphasis level (0 - 3) for lane (passed as parameter "lane", */
    /* ranging 0-3) as in linkState->preEmphasis[lane] */

    /* 3.5 Procedure: Change PMA TX Emphasis */
    /* 2.8.3 Display Port / Embedded Display Port */
    /* Torrent16FFC_Programmers_Guide_v0.6.pdf */

    uint32_t retVal;
    uint8_t voltageSwing;
    uint8_t preEmphasis;
    uint16_t regTmp;
    DP_SD0801_VoltageCoefficients* coeffs;
    /* Bits 9 and 10 of address indicate lane number. */
    const uint32_t laneOffset = ((uint32_t)lane << 9);
    uint32_t DiagAcyaAddr = (TX_DIAG_ACYA | laneOffset);

    retVal = DP_SD0801_ConfigLaneSF(pD, linkState);
    if (CDN_EOK == retVal) {

        voltageSwing = linkState->voltageSwing[lane];
        preEmphasis = linkState->preEmphasis[lane];

        /* Store new settings in pD. */
        pD->linkState.voltageSwing[lane] = voltageSwing;
        pD->linkState.preEmphasis[lane] = preEmphasis;

        /* Write register bit TX_DIAG_ACYA[0] to 1'b1 to freeze the current state of the analog TX driver. */
        regTmp = afeRead(pD, DiagAcyaAddr);
        regTmp |= TX_DIAG_ACYA_HBDC_MASK;
        afeWrite(pD, DiagAcyaAddr, regTmp);

        if ((voltageSwing + preEmphasis) <= 3U)
        {
            coeffs = &(pD->vCoeffs[voltageSwing][preEmphasis]);
            afeWriteChanged(pD, (TX_TXCC_CTRL | laneOffset), 0x08A4);
            afeWriteChanged(pD, (DRV_DIAG_TX_DRV | laneOffset), coeffs->DiagTxDrv);
            afeWriteChanged(pD, (TX_TXCC_MGNFS_MULT_000 | laneOffset), coeffs->MgnfsMult);
            afeWriteChanged(pD, (TX_TXCC_CPOST_MULT_00 | laneOffset), coeffs->CpostMult);
        }

        /* Write register bit TX_DIAG_ACYA[0] to 1'b0 */
        /* to allow the state of the analog TX driver to reflect the new programmed. */
        regTmp = afeRead(pD, DiagAcyaAddr);
        regTmp &= ~(TX_DIAG_ACYA_HBDC_MASK);
        afeWrite(pD, DiagAcyaAddr, regTmp);
    }

    return retVal;
}

/**
 * Enable or disable PLL for selected lanes)
 */
static void setPllEnable(const DP_SD0801_PrivateData* pD, uint8_t laneCount, bool enable)
{
    uint32_t regTmp;
    /* used to determine, which bits to check for or enable in PMA_PLLCLK_EN register */
    uint32_t pllRegBits;
    /* used to enable or disable lanes */
    uint32_t pllRegWriteVal;

    /* Select values of registers and mask, depending on enabled lane count. */
    switch (laneCount)
    {
    /* lane 0 */
    case (0x0001):
        pllRegBits = 0x00000001U;
        break;
    /* lanes 0-1 */
    case (0x0002):
        pllRegBits = 0x00000003U;
        break;
    /* lanes 0-3, all */
    default:
        pllRegBits = 0x0000000FU;
        break;
    }

    if (enable) {
        pllRegWriteVal = pllRegBits;
    } else {
        pllRegWriteVal = 0x00000000U;
    }

    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_p, pllRegWriteVal); /* Enable / disable PLL */
    do {
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_ACK_p);
    } while (((regTmp) & pllRegBits) != pllRegWriteVal);
    CPS_DelayNs(100);
}

/**
 * Reconfigure VCO to required one and configure Link Rate, while PLLs are
 * disabled
 */
static void reconfigureLinkRate(const DP_SD0801_PrivateData* pD, DP_SD0801_LinkRate linkRate, uint8_t dpPll, uint8_t linkCfg, bool ssc)
{
    uint32_t regTmp;
    /* Disable the cmn_pll0_en before re-programming the new data rate */
    afeWrite(pD, PHY_PMA_PLL_RAW_CTRL, 0x0000U);

    /* Wait for PLL ready de-assertion */
    /* For PLL0 - PHY_PMA_CMN_CTRL2[2] == 1 */
    /* For PLL1 - PHY_PMA_CMN_CTRL2[3] == 1 */
    if (isPllSet(dpPll, 0)) {
        do {
            regTmp = afeRead(pD, PHY_PMA_CMN_CTRL2);
        } while (((regTmp >> 2U) & 1U) == 0U);
    }
    if (isPllSet(dpPll, 1)) {
        do {
            regTmp = afeRead(pD, PHY_PMA_CMN_CTRL2);
        } while (((regTmp >> 3U) & 1U) == 0U);
    }
    CPS_DelayNs(200);
    /* DP Rate Change - VCO Output setting */
    configurePhyPmaCmnVcoCfg(pD, getVcoFreq(linkRate), ssc);
    configurePhyPmaCmnDpRate(pD, linkCfg, linkRate, dpPll);

    /* Enable the cmn_pll0_en */
    afeWrite(pD, PHY_PMA_PLL_RAW_CTRL, 0x0003U);

    /* Wait for PLL ready assertion */
    /* For PLL0 - PHY_PMA_CMN_CTRL2[0] == 1 */
    /* For PLL1 - PHY_PMA_CMN_CTRL2[1] == 1 */
    if (isPllSet(dpPll, 0)) {
        do {
            regTmp = afeRead(pD, PHY_PMA_CMN_CTRL2);
        } while (((regTmp) & 1U) == 0U);
    }
    if (isPllSet(dpPll, 1)) {
        do {
            regTmp = afeRead(pD, PHY_PMA_CMN_CTRL2);
        } while (((regTmp >> 1U) & 1U) == 0U);
    }
}

uint32_t DP_SD0801_SetLinkRate(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState)
{
    /* set link rate as in linkState->linkRate */

    /* Both pma_xcvr_standard_mode_ln_* and pma_xcvr_data_width_ln_* are IPS fixed */
    /* According to above there's no need to place link in A3 power state */

    uint32_t retVal;
    uint8_t linkCfg = 0U;
    uint8_t dpPll = 1; /* TBD how to set */

    retVal = DP_SD0801_SetLinkRateSF(pD, linkState);
    if (CDN_EOK == retVal) {

        const uint8_t laneCount = linkState->laneCount;
        const DP_SD0801_LinkRate linkRate = linkState->linkRate;
        const bool ssc = linkState->ssc;

        /* Store new settings in pD. */
        pD->linkState.linkRate = linkRate;
        pD->linkState.laneCount = laneCount;
        pD->linkState.ssc = ssc;

        if ((laneCount <= 4U) && (laneCount >  0U))
        {
            linkCfg = (uint8_t)((1U << (laneCount)) - 1U);
        }

        setPowerState(pD, POWERSTATE_A3, laneCount);

        /* Disable PLLs */
        setPllEnable(pD, laneCount, false);
        CPS_DelayNs(100);

        reconfigureLinkRate(pD, linkRate, dpPll, linkCfg, ssc);
        /* No need as far as pma_xcvr_standard_mode_ln_* and pma_xcvr_data_width_ln_* are IPS fixed */
        CPS_DelayNs(200);

        /* Enable PLLs */
        setPllEnable(pD, laneCount, true);

        setPowerState(pD, POWERSTATE_A2, laneCount);
        setPowerState(pD, POWERSTATE_A0, laneCount);
        CPS_DelayNs(900); /* 100ns in total with delay in setPowerState */
    }

    return retVal;
}

static void setPhyIdleBits(uint32_t *regTmp, uint8_t laneCount)
{
    /* always enable lane 0 */
    *regTmp = CPS_FLD_WRITE(DP__DP_REGS__PHY_RESET_P, PMA_TX_ELEC_IDLE_LN_0, *regTmp, 0);

    /* Enable lane 1 for > 1 lanes */
    *regTmp = CPS_FLD_WRITE(DP__DP_REGS__PHY_RESET_P, PMA_TX_ELEC_IDLE_LN_1, *regTmp, ((laneCount > 1U) ? 0U : 1U));

    /* Enable lanes 2 and 3 for > 2 lanes */
    *regTmp = CPS_FLD_WRITE(DP__DP_REGS__PHY_RESET_P, PMA_TX_ELEC_IDLE_LN_2, *regTmp, ((laneCount > 2U) ? 0U : 1U));
    *regTmp = CPS_FLD_WRITE(DP__DP_REGS__PHY_RESET_P, PMA_TX_ELEC_IDLE_LN_3, *regTmp, ((laneCount > 2U) ? 0U : 1U));
}

/**
 * Assert lane reset (Active low) on lane 0, among disabled lanes.
 */
static void resetLane0(const DP_SD0801_PrivateData* pD, uint8_t linkCfg)
{
    uint32_t regTmp;

    /* Assert lane reset low so that unused lanes remain in reset and powered down when re-enable the link */
    regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PHY_RESET_p);
    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, ((regTmp & 0x0000FFF0U) | (0x0000000EU & (uint32_t)linkCfg)));
}

static void startupLanes(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    uint32_t regTmp;
    uint8_t linkCfg = 0U;

    if ((laneCount <= 4U) && (laneCount >  0U))
    {
        linkCfg = (uint8_t)((1U << (laneCount)) - 1U);
    }

    /* Assert lane reset (Active low) on lane 0, among disabled lanes. */
    resetLane0(pD, linkCfg);

    /* Set lanes into power state A0 */
    setPowerA0(pD, laneCount);

    /* release phy_l0*_reset_n based on used laneCount */
    regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PHY_RESET_p);
    CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, ((regTmp & 0x0000FFF0U) | (0x0000000FU & (uint32_t)linkCfg)));
}

/**
 * Start-up PHY lanes and wait for pma_cmn_ready, then delay for 100 ns
 */
static void startupLanesAndWait(const DP_SD0801_PrivateData* pD, uint8_t laneCount)
{
    startupLanes(pD, laneCount);

    /* Checking pma_cmn_ready */
    waitPmaCmnReady(pD);

    CPS_DelayNs(100);
}

uint32_t DP_SD0801_EnableLanes(DP_SD0801_PrivateData* pD, const DP_SD0801_LinkState* linkState)
{
    /* set active lane count (1, 2 or 4) as in linkState->laneCount. */

    uint32_t retVal;
    uint32_t regTmp;

    retVal = DP_SD0801_EnableLanesSF(pD, linkState);
    if (CDN_EOK == retVal) {

        const uint8_t laneCount = linkState->laneCount;

        /* Store new setting in pD. */
        pD->linkState.laneCount = laneCount;

        /* Assert pma_tx_elec_idle_ln_* for disabled lanes */
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PHY_RESET_p);
        setPhyIdleBits(&regTmp, laneCount);
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, regTmp);

        /* reset the link by asserting phy_l00_reset_n low */
        regTmp = CPS_REG_READ(&pD->regBaseDp->dp_regs.PHY_RESET_p);
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PHY_RESET_p, (regTmp & 0x0000FFFEU));

        startupLanesAndWait(pD, laneCount);

        /* release pma_xcvr_pllclk_en_ln_*, only for the master lane */
        CPS_REG_WRITE(&pD->regBaseDp->dp_regs.PMA_PLLCLK_EN_p, 0x0001);

        phyRun(pD, laneCount);
    }

    return retVal;
}

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4, DRV-3852" */
/**
 * Get default values of voltage-related PHY registers, for given
 * voltage swing and pre-emphasis.
 */
uint32_t DP_SD0801_GetDefaultCoeffs(const DP_SD0801_PrivateData*   pD,
                                    uint8_t                        voltageSwing,
                                    uint8_t                        preEmphasis,
                                    DP_SD0801_VoltageCoefficients* coefficients)
{
    /* Array consist of default values of voltage-related registers for sd0801 PHY. */
    static const DP_SD0801_VoltageCoefficients defaults[DP_SD0801_SWING_LEVEL_COUNT][DP_SD0801_EMPHASIS_LEVEL_COUNT] = \
        /* voltage swing 0, pre-emphasis 0->3 */
    {{{.DiagTxDrv = 0x0003, .MgnfsMult = 0x002A, .CpostMult = 0x0000},
      {.DiagTxDrv = 0x0003, .MgnfsMult = 0x001F, .CpostMult = 0x0014},
      {.DiagTxDrv = 0x0003, .MgnfsMult = 0x0012, .CpostMult = 0x0020},
      {.DiagTxDrv = 0x0003, .MgnfsMult = 0x0000, .CpostMult = 0x002A}},

     /* voltage swing 1, pre-emphasis 0->3 */
     {{.DiagTxDrv = 0x0003, .MgnfsMult = 0x001F, .CpostMult = 0x0000},
        {.DiagTxDrv = 0x0003, .MgnfsMult = 0x0013, .CpostMult = 0x0012},
        {.DiagTxDrv = 0x0003, .MgnfsMult = 0x0000, .CpostMult = 0x001F},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF}},

     /* voltage swing 2, pre-emphasis 0->3 */
     {{.DiagTxDrv = 0x0003, .MgnfsMult = 0x0013, .CpostMult = 0x0000},
        {.DiagTxDrv = 0x0003, .MgnfsMult = 0x0000, .CpostMult = 0x0013},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF}},

     /* voltage swing 3, pre-emphasis 0->3 */
     {{.DiagTxDrv = 0x0003, .MgnfsMult = 0x0000, .CpostMult = 0x0000},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF},
        {.DiagTxDrv = 0xFFFF, .MgnfsMult = 0xFFFF, .CpostMult = 0xFFFF}}};

    uint32_t retVal;
    retVal = DP_SD0801_GetDefaultCoeffsSF(pD, voltageSwing, preEmphasis, coefficients);

    if (CDN_EOK == retVal)
    {
        /* Fill structure with default values for voltage swing and pre-emphasis */
        /* taken from the table. */
        coefficients->DiagTxDrv = defaults[voltageSwing][preEmphasis].DiagTxDrv;
        coefficients->MgnfsMult = defaults[voltageSwing][preEmphasis].MgnfsMult;
        coefficients->CpostMult = defaults[voltageSwing][preEmphasis].CpostMult;
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-39-3 */
