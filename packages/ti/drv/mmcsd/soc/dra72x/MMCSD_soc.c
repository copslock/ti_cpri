/**
 *  \file   MMCSD_soc.c
 *
 *  \brief  DRA72x SoC specific MMCSD hardware attributes.
 *
 *   This file contains the hardware attributes of MMCSD peripheral like
 *   base address, interrupt number etc.
 */

/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>
#include <ti/csl/soc.h>
#include <ti/drv/mmcsd/MMCSD.h>
#include <ti/drv/mmcsd/soc/MMCSD_soc.h>

#define CSL_EDMA3_CHA_MMC1_RX    61
#define CSL_EDMA3_CHA_MMC1_TX    60
#define CSL_EDMA3_CHA_MMC2_RX    47
#define CSL_EDMA3_CHA_MMC2_TX    46
#define CSL_EDMA3_CHA_MMC3_RX    52
#define CSL_EDMA3_CHA_MMC3_TX    53
#define CSL_EDMA3_CHA_MMC4_RX    57
#define CSL_EDMA3_CHA_MMC4_TX    56

extern MMCSD_Error MMCSD_iodelayFxn (uint32_t instanceNum,
                              MMCSD_v1_IodelayParams *iodelayParams);

/* MMCSD configuration structure */
MMCSD_v1_HwAttrs MMCSDInitCfg[MMCSD_CNT] =
{
    {
        1,
#ifdef _TMS320C6X
        SOC_MMC1_BASE,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
        75,/* DSP1_IRQ_75 is available to use, hence using it */
#elif defined(__ARM_ARCH_7A__)
        SOC_MMC1_BASE,
        115, /* Corresponds to MPU_IRQ_83 (32 + MPU_IRQ_83) */
        0,  /* Event ID is not used by A15 */
#else
        SOC_MMC1_BASE,
        66, /* Corresponds to IPU1_IRQ_66 */
        0,
#endif
        192000000U,
        400000U,
        MMCSD_CARD_SD,
        (MMCSD_BUS_WIDTH_1BIT | MMCSD_BUS_WIDTH_4BIT),
        (MMCSD_BUS_VOLTAGE_1_8V | MMCSD_BUS_VOLTAGE_3_0V),
        1,
        0,
        NULL,//&MMCSD_iodelayFxn,
        NULL,
        NULL,
        1,  /*Enable DMA by default */
        CSL_EDMA3_CHA_MMC1_RX,
        CSL_EDMA3_CHA_MMC1_TX,
        0U,
        0U,
        0U,
        0U,
        0U,
        NULL,
#ifdef _TMS320C6X
        MUXINTCP_CROSSBAR_MUXNUM_DSP1, /* Crossbar default DSP1 */
        CSL_XBAR_MMC1_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_DSP1_IRQ_75 /* Crossbar Instance number for DSP_IRQ75 */
#elif defined(__TI_ARM_V7M4__)
        MUXINTCP_CROSSBAR_MUXNUM_IPU1, /* Crossbar default for IPU1 */
        CSL_XBAR_MMC1_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_IPU1_IRQ_66  /* Cross bar instance number for IRQ66 used in eventNum */ 
#else
        MUXINTCP_CROSSBAR_MUXNUM_MPU, /* Crossbar default for MPU */
        CSL_XBAR_MMC1_IRQ,/* Crossbar mux in */
        CSL_XBAR_INST_MPU_IRQ_83 /* Cross bar instance number for IRQ83 used in eventNum */ 
#endif
    },
    {
        2,
#ifdef _TMS320C6X
        SOC_MMC2_BASE,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
        76,/* DSP1_IRQ_76 is available to use, hence using it */
#elif defined(__ARM_ARCH_7A__)
        SOC_MMC2_BASE,
        118, /* Corresponds to MPU_IRQ_86 (32 + MPU_IRQ_86) */
        0,
#else
        SOC_MMC2_BASE,
        67,/* Corresponds to IPU1_IRQ_67 */
        0,
#endif
        192000000U,
        400000U,
        MMCSD_CARD_EMMC,
        MMCSD_BUS_WIDTH_8BIT,
        (MMCSD_BUS_VOLTAGE_1_8V | MMCSD_BUS_VOLTAGE_3_0V),
        1,
        0,
        NULL,//&MMCSD_iodelayFxn,
        NULL,
        NULL,
        1,  /*Enable DMA by default */
        CSL_EDMA3_CHA_MMC2_RX,
        CSL_EDMA3_CHA_MMC2_TX,
        0U,
        0U,
        0U,
        0U,
        0U,
        NULL,
#ifdef _TMS320C6X
        MUXINTCP_CROSSBAR_MUXNUM_DSP1, /* Crossbar default DSP1 */
        CSL_XBAR_MMC2_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_DSP1_IRQ_76 /* Crossbar Instance number for DSP_IRQ76 used in eventNum  */
#elif defined(__TI_ARM_V7M4__)
        MUXINTCP_CROSSBAR_MUXNUM_IPU1, /* Crossbar default for IPU1 */
        CSL_XBAR_MMC2_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_IPU1_IRQ_67  /* Cross bar instance number for IRQ67 used in eventNum */ 
#else
        MUXINTCP_CROSSBAR_MUXNUM_MPU, /* Crossbar default for MPU */
        CSL_XBAR_MMC2_IRQ,/* Crossbar mux in */
        CSL_XBAR_INST_MPU_IRQ_84 /* Cross bar instance number for IRQ84 used in eventNum */ 
#endif
    },
    {
        3,
#ifdef _TMS320C6X
        SOC_MMC3_BASE,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
        77,/* DSP1_IRQ_77 is available to use, hence using it */
#elif defined(__ARM_ARCH_7A__)
        SOC_MMC3_BASE,
        126, /* Corresponds to MPU_IRQ_94 (32 + MPU_IRQ_94) */
        0,
#else
        SOC_MMC3_BASE,
        68,/* Corresponds to IPU1_IRQ_68 */
        0,
#endif
        192000000U,
        400000U,
        MMCSD_CARD_SD,
        (MMCSD_BUS_WIDTH_1BIT | MMCSD_BUS_WIDTH_4BIT),
        (MMCSD_BUS_VOLTAGE_1_8V | MMCSD_BUS_VOLTAGE_3_0V),
        1,
        0,
        NULL,//&MMCSD_iodelayFxn,
        NULL,
        NULL,
        1,  /*Enable DMA by default */
        CSL_EDMA3_CHA_MMC3_RX,
        CSL_EDMA3_CHA_MMC3_TX,
        0U,
        0U,
        0U,
        0U,
        0U,
        NULL,
#ifdef _TMS320C6X
        MUXINTCP_CROSSBAR_MUXNUM_DSP1, /* Crossbar default DSP1 */
        CSL_XBAR_MMC3_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_DSP1_IRQ_77 /* Crossbar Instance number for DSP_IRQ77 used in eventNum */
#elif defined(__TI_ARM_V7M4__)
        MUXINTCP_CROSSBAR_MUXNUM_IPU1, /* Crossbar default for IPU1 */
        CSL_XBAR_MMC3_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_IPU1_IRQ_68  /* Cross bar instance number for IRQ68 used in eventNum */ 
#else
        MUXINTCP_CROSSBAR_MUXNUM_MPU, /* Crossbar default for MPU */
        CSL_XBAR_MMC3_IRQ,/* Crossbar mux in */
        CSL_XBAR_INST_MPU_IRQ_94 /* Cross bar instance number for IRQ94 used in eventNum */ 
#endif
    },
    {
        4,
#ifdef _TMS320C6X
        SOC_MMC4_BASE,
        OSAL_REGINT_INTVEC_EVENT_COMBINER,
        78,/* DSP1_IRQ_78 is available to use, hence using it */
#elif defined(__ARM_ARCH_7A__)
        SOC_MMC4_BASE,
        128,/* Corresponds to MPU_IRQ_96 (32 + MPU_IRQ_96) */
        0,
#else
        SOC_MMC4_BASE,
        69,/* Corresponds to IPU1_IRQ_69 */
        0,
#endif
        192000000U,
        400000U,
        MMCSD_CARD_SD,
        (MMCSD_BUS_WIDTH_1BIT | MMCSD_BUS_WIDTH_4BIT),
        (MMCSD_BUS_VOLTAGE_1_8V | MMCSD_BUS_VOLTAGE_3_0V),
        1,
        0,
        NULL,//&MMCSD_iodelayFxn,
        NULL,
        NULL,
        1,  /*Enable DMA by default */
        CSL_EDMA3_CHA_MMC4_RX,
        CSL_EDMA3_CHA_MMC4_TX,
        0U,
        0U,
        0U,
        0U,
        0U,
        NULL,
#ifdef _TMS320C6X
        MUXINTCP_CROSSBAR_MUXNUM_DSP1, /* Crossbar default DSP1 */
        CSL_XBAR_MMC4_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_DSP1_IRQ_78 /* Crossbar Instance number for DSP_IRQ78 used in eventNum */
#elif defined(__TI_ARM_V7M4__)
        MUXINTCP_CROSSBAR_MUXNUM_IPU1, /* Crossbar default for IPU1 */
        CSL_XBAR_MMC4_IRQ, /* Crossbar mux in */
        CSL_XBAR_INST_IPU1_IRQ_69  /* Cross bar instance number for IRQ69 used in eventNum */ 
#else
        MUXINTCP_CROSSBAR_MUXNUM_MPU, /* Crossbar default for MPU */
        CSL_XBAR_MMC4_IRQ,/* Crossbar mux in */
        CSL_XBAR_INST_MPU_IRQ_96 /* Cross bar instance number for IRQ96 used in eventNum */ 
#endif
    }
};


/* MMCSD objects */
MMCSD_v1_Object MMCSDObjects[MMCSD_CNT];


/* MMC configuration structure */
const MMCSD_Config_list MMCSD_config = {
    {
        &MMCSD_v1_FxnTable,
        &MMCSDObjects[0],
        &MMCSDInitCfg[0]
    },

    {
        &MMCSD_v1_FxnTable,
        &MMCSDObjects[1],
        &MMCSDInitCfg[1]
    },

    {
        &MMCSD_v1_FxnTable,
        &MMCSDObjects[2],
        &MMCSDInitCfg[2]
    },

    {
        &MMCSD_v1_FxnTable,
        &MMCSDObjects[3],
        &MMCSDInitCfg[3]
    },

    {NULL, NULL, NULL}
};

/* This function converts the local L2 address to a global address and
*  will be used for DMA transactions which need a global addresses.
*/
uint32_t MMCSD_soc_l2_global_addr (uint32_t addr)
{
    if ((addr >= 0x800000) && (addr < 0x1000000))
    {
    #ifdef _TMS320C6X
        uint32_t coreNum;

        /* Get the core number. */
        coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

        /* Compute the global address. */
        return ((1 << 30) | (coreNum << 24) | (addr & 0x00ffffff));
    #else
        return addr;
    #endif
    }
    else
    {
       /* non-L2 address range */
       return addr;
    }
}
