/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  \file     sbl_lib_am57xx_prcm_dpll.c
 *
 *  \brief    This file contains the structure for all DPLL Divider elements for
 *            am57xx SOC family. This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/soc.h>
#include <sbl_lib.h>
#include <sbl_lib_board.h>
#include <sbl_lib_config.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Arrays given below are defined for 20 MHz */
static pmhalPrcmPllPostDivValue_t dpllMpuPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
};
#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmPllPostDivValue_t dpllMpuPostDivCfgOppOd_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
};
#endif
static pmhalPrcmPllPostDivValue_t dpllCorePostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2,  2 },  /* Div_m2_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_H12, 4 },  /* Div_h12_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H13, 62},  /* Div_h13_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H14, 5 },  /* Div_h14_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H22, 5 },  /* Div_h22_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H23, 4 },  /* Div_h23_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H24, 1 }   /* Div_h24_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllPerPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2,  4},  /* Div_m2_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_M3,  1},  /* Div_m3_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_H11, 3},  /* Div_h11_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H12, 4},  /* Div_h12_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H13, 4},  /* Div_h13_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H14, 2}   /* Div_h14_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllDspPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_M3, 3}  /* Div_m3_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllEvePostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2} /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllIvaPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 3} /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllGpuPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2} /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllDdrPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2,  2},  /* Div_m2_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_H11, 8}   /* Div_h11_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllGmacPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2,  4 },  /* Div_m2_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_M3,  10},  /* Div_m3_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_H11, 40},  /* Div_h11_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H12, 8 },  /* Div_h12_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H13, 10}   /* Div_h13_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllAbePostDivCfgAllOpp_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_M3, 1}  /* Div_m3_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllUsbPostDivCfgAllOpp_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2} /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllPcieRefPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 15}, /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllDspPostDivCfgOppOd_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_M3, 3}  /* Div_m3_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllIvaPostDivCfgOppOd_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2}, /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllGpuPostDivCfgOppOd_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2} /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllEvePostDivCfgOppHigh_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2}, /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllIvaPostDivCfgOppHigh_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 2}, /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllVideo1PostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_H11, 13} /* Div_h11_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllHdmiPostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1} /* Div_m2_clkcfg */
};

#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmPllPostDivValue_t dpllVideo2PostDivCfgOppNom_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2,  5 },  /* Div_m2_clkcfg  */
    {PMHAL_PRCM_DPLL_POST_DIV_H11, 10},  /* Div_h11_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H12, 10},  /* Div_h12_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H13, 10},  /* Div_h13_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_H14, 10}   /* Div_h14_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllMpuPostDivCfgOppLow_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
};

static pmhalPrcmPllPostDivValue_t dpllDspPostDivCfgOppHigh_20[] =
{
    {PMHAL_PRCM_DPLL_POST_DIV_M2, 1}, /* Div_m2_clkcfg */
    {PMHAL_PRCM_DPLL_POST_DIV_M3, 3}  /* Div_m3_clkcfg */
};
#endif

static pmhalPrcmDpllConfig_t      dpllCoreCfgOppNom_20 =
{
    266,
    4,
    0,
    dpllCorePostDivCfgOppNom_20,
    (sizeof (dpllCorePostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllPerCfgOppNom_20 =
{
    96,
    4,
    0,
    dpllPerPostDivCfgOppNom_20,
    (sizeof (dpllPerPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllDspCfgOppNom_20 =
{
    150,
    4,
    0,
    dpllDspPostDivCfgOppNom_20,
    (sizeof (dpllDspPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllEveCfgOppNom_20 =
{
    214,
    3,
    0,
    dpllEvePostDivCfgOppNom_20,
    (sizeof (dpllEvePostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllIvaCfgOppNom_20 =
{
    233,
    3,
    0,
    dpllIvaPostDivCfgOppNom_20,
    (sizeof (dpllIvaPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllGpuCfgOppNom_20 =
{
    170,
    3,
    0,
    dpllGpuPostDivCfgOppNom_20,
    (sizeof (dpllGpuPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllGmacCfgOppNom_20 =
{
    250,
    4,
    0,
    dpllGmacPostDivCfgOppNom_20,
    (sizeof (dpllGmacPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllAbeCfgAllOpp_20 =
{
    200,
    9,
    0,
    dpllAbePostDivCfgAllOpp_20,
    (sizeof (dpllAbePostDivCfgAllOpp_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllUsbCfgAllOpp_20 =
{
    27,
    0,
    0,
    dpllUsbPostDivCfgAllOpp_20,
    (sizeof (dpllUsbPostDivCfgAllOpp_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllPcieRefCfgOppNom_20 =
{
    750,                                  /* Multiplier */
    9,                                    /* Divider */
    0,                                    /* DutyCycleCorrector */
    dpllPcieRefPostDivCfgOppNom_20,
    (sizeof (dpllPcieRefPostDivCfgOppNom_20) /
     sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllDspCfgOppOd_20 =
{
    175,
    4,
    0,
    dpllDspPostDivCfgOppOd_20,
    (sizeof (dpllDspPostDivCfgOppOd_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllIvaCfgOppOd_20 =
{
    172,
    3,
    0,
    dpllIvaPostDivCfgOppOd_20,
    (sizeof (dpllIvaPostDivCfgOppOd_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllGpuCfgOppOd_20 =
{
    200,
    3,
    0,
    dpllGpuPostDivCfgOppOd_20,
    (sizeof (dpllGpuPostDivCfgOppOd_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllEveCfgOppHigh_20 =
{
    260,
    3,
    0,
    dpllEvePostDivCfgOppHigh_20,
    (sizeof (dpllEvePostDivCfgOppHigh_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllIvaCfgOppHigh_20 =
{
    266,
    4,
    0,
    dpllIvaPostDivCfgOppHigh_20,
    (sizeof (dpllIvaPostDivCfgOppHigh_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllVideo1CfgOppNom_20 =
{
    1637,                                 /* Multiplier */
    39,                                   /* Divider */
    0,                                    /* DutyCycleCorrector */
    dpllVideo1PostDivCfgOppNom_20,
    (sizeof (dpllVideo1PostDivCfgOppNom_20) /
     sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllHdmiCfgOppNom_20 =
{
    1188,
    15,
    0,
    dpllHdmiPostDivCfgOppNom_20,
    (sizeof (dpllHdmiPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmDpllConfig_t      dpllVideo2CfgOppNom_20 =
{
    297,                                  /* Multiplier */
    7,                                    /* Divider */
    0,                                    /* DutyCycleCorrector */
    dpllVideo2PostDivCfgOppNom_20,
    (sizeof (dpllVideo2PostDivCfgOppNom_20) /
     sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllDspCfgOppHigh_20 =
{
    187,
    4,
    0,
    dpllDspPostDivCfgOppHigh_20,
    (sizeof (dpllDspPostDivCfgOppHigh_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllMpuCfgOppLow_20 =
{
    250,
    9,
    0,
    dpllMpuPostDivCfgOppLow_20,
    (sizeof (dpllMpuPostDivCfgOppLow_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};
#endif

#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmDpllConfig_t      dpllDdrCfgOppNom_20 =
{
    266,
    4,
    0,
    dpllDdrPostDivCfgOppNom_20,
    (sizeof (dpllDdrPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllMpuCfgOppNom_20 =
{
    375,
    9,
    0,
    dpllMpuPostDivCfgOppNom_20,
    (sizeof (dpllMpuPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllMpuCfgOppOd_20 =
{
    294,
    4,
    0,
    dpllMpuPostDivCfgOppOd_20,
    (sizeof (dpllMpuPostDivCfgOppOd_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};
#else
static pmhalPrcmDpllConfig_t      dpllDdrCfgOppNom_20 =
{
    333,
    4,
    0,
    dpllDdrPostDivCfgOppNom_20,
    (sizeof (dpllDdrPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};

static pmhalPrcmDpllConfig_t      dpllMpuCfgOppNom_20 =
{
    400,
    9,
    0,
    dpllMpuPostDivCfgOppNom_20,
    (sizeof (dpllMpuPostDivCfgOppNom_20) / sizeof (pmhalPrcmPllPostDivValue_t)),
    0
};
#endif

static pmhalPrcmDpllConfig_t     *pDpllAbeCfg_20[] =
{
    &dpllAbeCfgAllOpp_20,
    &dpllAbeCfgAllOpp_20,
    &dpllAbeCfgAllOpp_20,
    &dpllAbeCfgAllOpp_20
};

static pmhalPrcmDpllConfig_t     *pDpllCoreCfg_20[] =
{
    &dpllCoreCfgOppNom_20,
    &dpllCoreCfgOppNom_20,
    &dpllCoreCfgOppNom_20,
    &dpllCoreCfgOppNom_20
};

static pmhalPrcmDpllConfig_t     *pDpllDdrCfg_20[] =
{
    &dpllDdrCfgOppNom_20,
    &dpllDdrCfgOppNom_20,
    &dpllDdrCfgOppNom_20,
    &dpllDdrCfgOppNom_20
};

static pmhalPrcmDpllConfig_t     *pDpllGmacCfg_20[] =
{
    &dpllGmacCfgOppNom_20,
    &dpllGmacCfgOppNom_20,
    &dpllGmacCfgOppNom_20,
    &dpllGmacCfgOppNom_20
};

static pmhalPrcmDpllConfig_t     *pDpllIvaCfg_20[] =
{
    &dpllIvaCfgOppNom_20,
    &dpllIvaCfgOppNom_20,
    &dpllIvaCfgOppOd_20,
    &dpllIvaCfgOppHigh_20
};

static pmhalPrcmDpllConfig_t     *pDpllPcieRefCfg_20[] =
{
    &dpllPcieRefCfgOppNom_20,
    &dpllPcieRefCfgOppNom_20,
    &dpllPcieRefCfgOppNom_20,
    &dpllPcieRefCfgOppNom_20
};

static pmhalPrcmDpllConfig_t     *pDpllPerCfg_20[] =
{
    &dpllPerCfgOppNom_20,
    &dpllPerCfgOppNom_20,
    &dpllPerCfgOppNom_20,
    &dpllPerCfgOppNom_20
};

static pmhalPrcmDpllConfig_t     *pDpllUsbCfg_20[] =
{
    &dpllUsbCfgAllOpp_20,
    &dpllUsbCfgAllOpp_20,
    &dpllUsbCfgAllOpp_20,
    &dpllUsbCfgAllOpp_20
};

static pmhalPrcmDpllConfig_t     *pDpllEveCfg_20[] =
{
    &dpllEveCfgOppNom_20,
    &dpllEveCfgOppNom_20,
    &dpllEveCfgOppHigh_20,
    &dpllEveCfgOppHigh_20
};

static pmhalPrcmDpllConfig_t     *pDpllVideo1Cfg_20[] =
{
    &dpllVideo1CfgOppNom_20,
    &dpllVideo1CfgOppNom_20,
    &dpllVideo1CfgOppNom_20,
    &dpllVideo1CfgOppNom_20
};

#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmDpllConfig_t     *pDpllVideo2Cfg_20[] =
{
    &dpllVideo2CfgOppNom_20,
    &dpllVideo2CfgOppNom_20,
    &dpllVideo2CfgOppNom_20,
    &dpllVideo2CfgOppNom_20
};
#endif

static pmhalPrcmDpllConfig_t     *pDpllHdmiCfg_20[] =
{
    &dpllHdmiCfgOppNom_20,
    &dpllHdmiCfgOppNom_20,
    &dpllHdmiCfgOppNom_20,
    &dpllHdmiCfgOppNom_20
};

#if defined (SOC_AM572x) || defined (SOC_AM574x)
static pmhalPrcmDpllConfig_t     *pDpllDspCfg_20[] =
{
    &dpllDspCfgOppNom_20,
    &dpllDspCfgOppNom_20,
    &dpllDspCfgOppOd_20,
    &dpllDspCfgOppHigh_20
};

static pmhalPrcmDpllConfig_t     *pDpllGpuCfg_20[] =
{
    &dpllGpuCfgOppNom_20,
    &dpllGpuCfgOppNom_20,
    &dpllGpuCfgOppOd_20,
    &dpllGpuCfgOppOd_20
};

static pmhalPrcmDpllConfig_t     *pDpllMpuCfg_23x23Package_20[] =
{
    &dpllMpuCfgOppLow_20,
    &dpllMpuCfgOppNom_20,
    &dpllMpuCfgOppOd_20,
    &dpllMpuCfgOppOd_20
};
static pmhalPrcmDpllConfig_t     *pDpllMpuCfg_17x17Package_20[] =
{
    &dpllMpuCfgOppLow_20,
    &dpllMpuCfgOppNom_20,
    &dpllMpuCfgOppNom_20,
    &dpllMpuCfgOppNom_20
};
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

int32_t SBLLibGetDpllStructure(uint32_t                dpllId,
                               uint32_t                sysClk,
                               uint32_t                opp,
                               pmhalPrcmDpllConfig_t **dpllCfg)
{
#if defined (SOC_AM572x) || defined (SOC_AM574x)
    uint32_t siliconPackageType;
#endif
    int32_t  retVal = STW_SOK;

    if (PMHAL_PRCM_SYSCLK_20_MHZ == sysClk)
    {
        switch (dpllId)
        {
            case PMHAL_PRCM_DPLL_ABE:
                *dpllCfg = pDpllAbeCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_CORE:
                *dpllCfg = pDpllCoreCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_DDR:
                *dpllCfg = pDpllDdrCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_GMAC:
                *dpllCfg = pDpllGmacCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_GPU:
                *dpllCfg = pDpllGpuCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_IVA:
                *dpllCfg = pDpllIvaCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_PCIE_REF:
                *dpllCfg = pDpllPcieRefCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_PER:
                *dpllCfg = pDpllPerCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_USB:
                *dpllCfg = pDpllUsbCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_DSP:
                *dpllCfg = pDpllDspCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_EVE:
                *dpllCfg = pDpllEveCfg_20[opp];
                break;
            case PMHAL_PRCM_DPLL_MPU:
#if defined (SOC_AM572x) || defined (SOC_AM574x)
                siliconPackageType = SBLLibGetSiliconPackageType();
                if (siliconPackageType == SBLLIB_SILICON_PACKAGE_TYPE_17X17)
                {
                    *dpllCfg = pDpllMpuCfg_17x17Package_20[opp];
                }
                else
                {
                    *dpllCfg = pDpllMpuCfg_23x23Package_20[opp];
                }
#else
                *dpllCfg = pDpllMpuCfg_23x23Package_20[opp];
#endif
                break;
            case PMHAL_PRCM_VIDEOPLL_VIDEO1:
                *dpllCfg = pDpllVideo1Cfg_20[opp];
                break;
#if defined (SOC_AM572x) || defined (SOC_AM574x)
            case PMHAL_PRCM_VIDEOPLL_VIDEO2:
                *dpllCfg = pDpllVideo2Cfg_20[opp];
                break;
#endif
            case PMHAL_PRCM_VIDEOPLL_HDMI:
                *dpllCfg = pDpllHdmiCfg_20[opp];
                break;
            default:
                retVal = STW_EFAIL;
                break;
        }
    }
    else
    {
        retVal = STW_EFAIL;
    }

    return retVal;
}

#ifdef __cplusplus
}
#endif

