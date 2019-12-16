/**
 *  \file   sbl_main.c
 *
 *  \brief  This file contain main function, call the Board Initialization
 *          functions & slave core boot-up functions in sequence.
 *
 */

/*
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com/
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

 /* TI RTOS header files */
#include "sbl_main.h"

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
extern sblProfileInfo_t sblProfileLog[MAX_PROFILE_LOG_ENTRIES];
extern uint32_t sblProfileLogIndx;
extern uint32_t sblProfileLogOvrFlw;

#pragma DATA_SECTION(sblProfileLogAddr, ".sbl_profile_info")
volatile sblProfileInfo_t * sblProfileLogAddr;

#pragma DATA_SECTION(sblProfileLogIndxAddr, ".sbl_profile_info")
volatile uint32_t *sblProfileLogIndxAddr;

#pragma DATA_SECTION(sblProfileLogOvrFlwAddr, ".sbl_profile_info")
volatile uint32_t *sblProfileLogOvrFlwAddr;

sblEntryPoint_t k3xx_evmEntry;

const CSL_ArmR5MpuRegionCfg gCslR5MpuCfg[CSL_ARM_R5F_MPU_REGIONS_MAX] =
{
    {
        /* Region 0 configuration: complete 32 bit address space = 4Gbits */
        .regionId         = 0U,
        .enable           = 1U,
        .baseAddr         = 0x0U,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_4GB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 1U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)FALSE,
        .cachePolicy      = 0U,
        .memAttr          = 0U,
    },
    {
        /* Region 1 configuration: 128 bytes memory for exception vector execution */
        .regionId         = 1U,
        .enable           = 1U,
        .baseAddr         = 0x0U,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_32KB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_CACHE_POLICY_NON_CACHEABLE,
        .memAttr          = 0U,
    },
    {
        /* Region 2 configuration: 512 KB OCMS RAM */
        .regionId         = 2U,
        .enable           = 1U,
        .baseAddr         = 0x41C00000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_512KB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_MEM_ATTR_CACHED_WT_NO_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 3 configuration: 2 MB MCMS3 RAM */
        .regionId         = 3U,
        .enable           = 1U,
        .baseAddr         = 0x70000000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_8MB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_MEM_ATTR_CACHED_WT_NO_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 4 configuration: 2 GB DDR RAM */
        .regionId         = 4U,
        .enable           = 1U,
        .baseAddr         = 0x80000000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_2GB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_MEM_ATTR_CACHED_WT_NO_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 5 configuration: 64 KB BTCM */
        .regionId         = 5U,
        .enable           = 1U,
        .baseAddr         = 0x41010000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_32KB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_CACHE_POLICY_NON_CACHEABLE,
        .memAttr          = 0U,
    },
    {
        /* Region 6 configuration: 128 MB FSS DAT0 */
        .regionId         = 6U,
        .enable           = 1U,
        .baseAddr         = 0x50000000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_128MB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_MEM_ATTR_CACHED_WT_NO_WA,
        .memAttr          = 0U,
    },
    {
        /* Region 7 configuration: 128 MB FSS DAT1 */
        .regionId         = 7U,
        .enable           = 1U,
        .baseAddr         = 0x58000000,
        .size             = CSL_ARM_R5_MPU_REGION_SIZE_128MB,
        .subRegionEnable  = CSL_ARM_R5_MPU_SUB_REGION_ENABLE_ALL,
        .exeNeverControl  = 0U,
        .accessPermission = CSL_ARM_R5_ACC_PERM_PRIV_USR_RD_WR,
        .shareable        = 0U,
        .cacheable        = (uint32_t)TRUE,
        .cachePolicy      = CSL_ARM_R5_MEM_ATTR_CACHED_WT_NO_WA,
        .memAttr          = 0U,
    },
};

int main()
{
    cpu_core_id_t core_id;

    SBL_ADD_PROFILE_POINT;

    /* Any SoC specific Init. */
    SBL_SocEarlyInit();

    if (SBL_LOG_LEVEL > SBL_LOG_ERR)
    {
        /* Configure UART Tx pinmux. */
        Board_uartTxPinmuxConfig();
    }

    SBL_ADD_PROFILE_POINT;

    if (SBL_LOG_LEVEL > SBL_LOG_NONE)
    {
        UART_HwAttrs uart_cfg;

        UART_socGetInitCfg(BOARD_UART_INSTANCE, &uart_cfg);
        /* Use UART fclk freq setup by ROM */
        uart_cfg.frequency = SBL_ROM_UART_MODULE_INPUT_CLK;
        /* Disable the UART interrupt */
        uart_cfg.enableInterrupt = FALSE;
        UART_socSetInitCfg(BOARD_UART_INSTANCE, &uart_cfg);
        /* Init UART for logging. */
        UART_stdioInit(BOARD_UART_INSTANCE);
    }

    SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MIN, "%s (%s - %s)\n", SBL_VERSION_STR, __DATE__, __TIME__);

    SBL_ADD_PROFILE_POINT;

   /* Initialize the ATCM */
    memset((void *)SBL_MCU_ATCM_BASE, 0xFF, 0x8000);

    /* Relocate CSL Vectors to ATCM*/
    memcpy((void *)SBL_MCU_ATCM_BASE, (void *)_resetvectors, 0x100);

    SBL_ADD_PROFILE_POINT;

    /* Setup RAT */
    SBL_RAT_Config(sblRatCfgList);

    SBL_ADD_PROFILE_POINT;

    /* Load SYSFW. */
    SBL_SciClientInit();

    SBL_ADD_PROFILE_POINT;

    /* Board pinmux. */
    Board_init(BOARD_INIT_PINMUX_CONFIG);

    SBL_ADD_PROFILE_POINT;

    /* Any SoC specific Init. */
    SBL_SocLateInit();

#if defined(SBL_ENABLE_PLL) && !defined(SBL_SKIP_SYSFW_INIT)
    SBL_log(SBL_LOG_MAX, "Initlialzing PLLs ...");
    SBL_ADD_PROFILE_POINT;
    Board_init(SBL_PLL_INIT);
    SBL_log(SBL_LOG_MAX, "done.\n");
#endif

#if defined(SBL_ENABLE_CLOCKS) && !defined(SBL_SKIP_SYSFW_INIT)
    SBL_log(SBL_LOG_MAX, "InitlialzingClocks ...");
    SBL_ADD_PROFILE_POINT;
    Board_init(SBL_CLOCK_INIT);
    SBL_log(SBL_LOG_MAX, "done.\n");
#endif

#if defined(SBL_ENABLE_DDR) && defined(SBL_ENABLE_PLL) && defined(SBL_ENABLE_CLOCKS)  && !defined(SBL_SKIP_SYSFW_INIT)
    SBL_log(SBL_LOG_MAX, "Initlialzing DDR ...");
    SBL_ADD_PROFILE_POINT;
    Board_init(BOARD_INIT_DDR);
    SBL_log(SBL_LOG_MAX, "done.\n");
#endif

    SBL_log(SBL_LOG_MAX, "Begin parsing user application\n");

    /* Image Copy */
    SBL_ImageCopy(&k3xx_evmEntry);

    /* Export SBL logs */
    sblProfileLogAddr = sblProfileLog;
    sblProfileLogIndxAddr = &sblProfileLogIndx;
    sblProfileLogOvrFlwAddr = &sblProfileLogOvrFlw;

    for (core_id = MPU1_CPU0_ID; core_id <= DSP2_C7X_ID; core_id ++)
    {
        /* Try booting all cores other than the cluster running the SBL */
        if ((k3xx_evmEntry.CpuEntryPoint[core_id] != SBL_INVALID_ENTRY_ADDR) &&
            (core_id != MCU1_CPU1_ID))
            SBL_SlaveCoreBoot(core_id, NULL, &k3xx_evmEntry);
    }

    /* Boot the core running SBL in the end */
    if ((k3xx_evmEntry.CpuEntryPoint[MCU1_CPU1_ID] != SBL_INVALID_ENTRY_ADDR) ||
        (k3xx_evmEntry.CpuEntryPoint[MCU1_CPU0_ID] < SBL_INVALID_ENTRY_ADDR))
        SBL_SlaveCoreBoot(MCU1_CPU1_ID, NULL, &k3xx_evmEntry);

    /* Execute a WFI */
    asm volatile (" wfi");

    return 0;
}
