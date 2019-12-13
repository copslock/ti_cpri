/*
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ti/board/board.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/arch/r5/csl_arm_r5.h>

#include "sbl_boot_perf_benchmark.h"

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
#pragma DATA_SECTION(sblPerfTestBoardCfg, ".sysfw_data_cfg_board")
struct tisci_boardcfg sblPerfTestBoardCfg;
#pragma DATA_SECTION(sblPerfTestBoardCfg_rm, ".sysfw_data_cfg_board_rm")
struct sblTest_local_rm_boardcfg sblPerfTestBoardCfg_rm;
#pragma DATA_SECTION(sblPerfTestBoardCfg_sec, ".sysfw_data_cfg_board_sec")
struct tisci_boardcfg_sec sblPerfTestBoardCfg_sec;

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/
static void  BOOT_PERF_TEST_CacheCleanInvalidateDcacheSetWay (void)
{
    uint32_t set = 0, way = 0;

    for (set = 0; set < 64; set ++)
    {
        for (way = 0; way < 4; way++)
        {
            CSL_armR5CacheCleanInvalidateDcacheSetWay(set, way);
        }
    }
}

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_PM)
static void BOOT_PERF_TEST_UartLogDisable(void)
{
    uint32_t McuUart0TxDPinCfg =
        (0 << SBL_MCU_UART_PADCONFIG_PULLUDEN_SHIFT)    | \
        (1 << SBL_MCU_UART_PADCONFIG_PULLTYPESEL_SHIFT) | \
        (0 << SBL_MCU_UART_PADCONFIG_RXACTIVE_SHIFT)    | \
        (1 << SBL_MCU_UART_PADCONFIG_TX_DIS_SHIFT)      | \
        (4 << SBL_MCU_UART_PADCONFIG_MUXMODE_SHIFT);

    Board_init(BOARD_INIT_UNLOCK_MMR);
    HW_WR_REG32(SBL_MCU_UART_PADCONFIG_ADDR, (McuUart0TxDPinCfg));
}

static void BOOT_PERF_TEST_UartLogEnable(void)
{
    uint32_t McuUart0TxDPinCfg =
        (0 << SBL_MCU_UART_PADCONFIG_PULLUDEN_SHIFT)    | \
        (1 << SBL_MCU_UART_PADCONFIG_PULLTYPESEL_SHIFT) | \
        (0 << SBL_MCU_UART_PADCONFIG_RXACTIVE_SHIFT)    | \
        (0 << SBL_MCU_UART_PADCONFIG_TX_DIS_SHIFT)      | \
        (4 << SBL_MCU_UART_PADCONFIG_MUXMODE_SHIFT);

    HW_WR_REG32(SBL_MCU_UART_PADCONFIG_ADDR, (McuUart0TxDPinCfg));
}
#endif

static void BOOT_PERF_TEST_SYSFW_UartLogEnable(void)
{
    uint32_t WkupUart0TxDPinCfg =
        (0 << SBL_SYSFW_UART_PADCONFIG_PULLUDEN_SHIFT)    | \
        (1 << SBL_SYSFW_UART_PADCONFIG_PULLTYPESEL_SHIFT) | \
        (0 << SBL_SYSFW_UART_PADCONFIG_RXACTIVE_SHIFT)    | \
        (0 << SBL_SYSFW_UART_PADCONFIG_TX_DIS_SHIFT)      | \
        (0 << SBL_SYSFW_UART_PADCONFIG_MUXMODE_SHIFT);

    HW_WR_REG32(SBL_SYSFW_UART_PADCONFIG_ADDR, (WkupUart0TxDPinCfg));
}

static void BOOT_PERF_TEST_printSblProfileLog(sblProfileInfo_t *sblProfileLog, uint32_t sblProfileLogIndx, uint32_t sblProfileLogOvrFlw)
{
    uint64_t mcu_clk_freq = SBL_MCU1_CPU0_FREQ_HZ;
    uint32_t i = 0, prev_cycle_cnt = 0, cycles_per_usec;
    uint32_t lastlogIndx;
    char sbl_test_str[256];

    Sciclient_pmGetModuleClkFreq(SBL_DEV_ID_MCU1_CPU0, SBL_CLK_ID_MCU1_CPU0, &mcu_clk_freq, SCICLIENT_SERVICE_WAIT_FOREVER);
    cycles_per_usec = (mcu_clk_freq / 1000000);

    sbl_puts("\r\nProfiling info ....\r\n");
    sprintf(sbl_test_str,"MCU @ %uHz.\r\n", ((uint32_t)mcu_clk_freq));sbl_puts(sbl_test_str);
    sprintf(sbl_test_str,"cycles per usec  = %u\r\n", cycles_per_usec);sbl_puts(sbl_test_str);

    lastlogIndx = sblProfileLogIndx;

    if (sblProfileLogOvrFlw)
    {
        i = sblProfileLogIndx;
        prev_cycle_cnt = sblProfileLog[i].cycle_cnt;
        lastlogIndx = MAX_PROFILE_LOG_ENTRIES;
        sbl_puts("Detected overflow, some profile entries might be lost.\r\n");
        sbl_puts("Rebuild with a larger vlaue of MAX_PROFILE_LOG_ENTRIES ??\r\n");
    }

    while((i % MAX_PROFILE_LOG_ENTRIES) < lastlogIndx)
    {
        uint32_t cycles_to_us;

        if (sblProfileLog[i].cycle_cnt < prev_cycle_cnt)
        {
            sbl_puts("**");
        }
        else
        {
            sbl_puts("  ");
        }
        cycles_to_us = sblProfileLog[i].cycle_cnt/cycles_per_usec;
        sprintf(sbl_test_str,"fxn:%32s\t", sblProfileLog[i].fxn);sbl_puts(sbl_test_str);
        sprintf(sbl_test_str,"line:%4u\t", sblProfileLog[i].line);sbl_puts(sbl_test_str);
        sprintf(sbl_test_str,"cycle:%10u\t", sblProfileLog[i].cycle_cnt);sbl_puts(sbl_test_str);
        sprintf(sbl_test_str,"timestamp:%10uus\r\n", cycles_to_us);sbl_puts(sbl_test_str);
        prev_cycle_cnt = sblProfileLog[i].cycle_cnt;
        i++;
    }
}

static int32_t BOOT_PERF_TEST_sysfwInit(void)
{
    int32_t status = CSL_PASS;

#if defined(SBL_SKIP_SYSFW_INIT)
    void *sysfw_ptr = (void *)&syfw_image;

    Sciclient_ConfigPrms_t        config =
    {
        SCICLIENT_SERVICE_OPERATION_MODE_POLLED,
    };
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_BOARD)
    Sciclient_BoardCfgPrms_t sblPerfTestBoardCfgPrms={(uint32_t)&sblPerfTestBoardCfg,0, sizeof(sblPerfTestBoardCfg), DEVGRP_ALL};
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_RM)
    Sciclient_BoardCfgPrms_t sblPerfTestBoardCfgRmPrms={(uint32_t)&sblPerfTestBoardCfg_rm,0, sizeof(sblPerfTestBoardCfg_rm), DEVGRP_ALL};
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_SEC)
    Sciclient_BoardCfgPrms_t sblPerfTestBoardCfgSecPrms={(uint32_t)&sblPerfTestBoardCfg_sec,0, sizeof(sblPerfTestBoardCfg_sec), DEVGRP_ALL};
#endif
    BOOT_PERF_TEST_CacheCleanInvalidateDcacheSetWay();

    BOOT_PERF_TEST_SYSFW_UartLogEnable();

#if defined(SBL_SKIP_SYSFW_INIT)
    /* Skipped by SBL for fast boot times, so we do it here */
    /* refer sbl_smp_r5.asm to see how to skip mpu_init     */
    status = Sciclient_loadFirmware(sysfw_ptr);
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }

    status = Sciclient_init(&config);
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_BOARD)

    memcpy((void *)&sblPerfTestBoardCfg, (void *)&gBoardConfigLow, sizeof(sblPerfTestBoardCfg));
    /* Redirect DMSC logs to UART 0 */
    sblPerfTestBoardCfg.debug_cfg.trace_dst_enables = TISCI_BOARDCFG_TRACE_DST_UART0;
    /* Enable full logs */
    sblPerfTestBoardCfg.debug_cfg.trace_src_enables = TISCI_BOARDCFG_TRACE_SRC_PM   |
                                                      TISCI_BOARDCFG_TRACE_SRC_RM   |
                                                      TISCI_BOARDCFG_TRACE_SRC_SEC  |
                                                      TISCI_BOARDCFG_TRACE_SRC_BASE |
                                                      TISCI_BOARDCFG_TRACE_SRC_USER |
                                                      TISCI_BOARDCFG_TRACE_SRC_SUPR ;
    BOOT_PERF_TEST_CacheCleanInvalidateDcacheSetWay();

    status = Sciclient_boardCfg(&sblPerfTestBoardCfgPrms);
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_PM)
    BOOT_PERF_TEST_UartLogDisable();
    status = Sciclient_boardCfgPm((Sciclient_BoardCfgPrms_t *)NULL);
    BOOT_PERF_TEST_UartLogEnable();
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }
#endif

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_RM)
    memcpy((void *)&sblPerfTestBoardCfg_rm, (void *)&gBoardConfigLow_rm, sizeof(sblPerfTestBoardCfg_rm));
    BOOT_PERF_TEST_CacheCleanInvalidateDcacheSetWay();
    status = Sciclient_boardCfgRm(&sblPerfTestBoardCfgRmPrms);
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }
#endif

    if (SBL_LOG_LEVEL == SBL_LOG_NONE)
    {
        Board_init(BOARD_INIT_UART_STDIO);
    }

#if defined(SBL_SKIP_SYSFW_INIT) || defined(SBL_SKIP_BRD_CFG_SEC)
    memcpy((void *)&sblPerfTestBoardCfg_sec, (void *)&gBoardConfigLow_security, sizeof(sblPerfTestBoardCfg_sec));
    BOOT_PERF_TEST_CacheCleanInvalidateDcacheSetWay();
    status = Sciclient_boardCfgSec(&sblPerfTestBoardCfgSecPrms);
    if (status != CSL_PASS)
    {
        return CSL_EFAIL;
    }
#endif

    return status;
}

int32_t main()
{
    volatile uint32_t pmuCntrVal = CSL_armR5PmuReadCntr(0x1F);
    char sbl_test_str[256];
    uint64_t mcu_clk_freq = SBL_MCU1_CPU0_FREQ_HZ;
    uint32_t cycles_per_usec;
    char *comp_mk = "sbl_component.mk";

    /* Perform the sysfw init here that was skipped */
    /* by the SBL to speed up boot times            */
    BOOT_PERF_TEST_sysfwInit();

    sbl_puts("\r\n");
    sprintf(sbl_test_str, "Time elapsed since start of SBL:");sbl_puts(sbl_test_str);
    cycles_per_usec = ((uint32_t)mcu_clk_freq) / 1000000;
    sprintf(sbl_test_str, "%10uus\r\n", pmuCntrVal/cycles_per_usec);sbl_puts(sbl_test_str);

    sprintf(sbl_test_str, "fxn:%16s\t", "boot_perf_test_main");sbl_puts(sbl_test_str);
    sprintf(sbl_test_str, "cycles:%10u\t\r\n", pmuCntrVal);sbl_puts(sbl_test_str);
    sbl_puts("\r\n");

    sbl_puts("Attempting board config ...");

#if !defined(SBL_ENABLE_PLL) || defined(SBL_ENABLE_CUST_PLLS) || defined(SBL_SKIP_SYSFW_INIT)
    sbl_puts("BOARD_INIT_PLL ...");
    Board_init(BOARD_INIT_PLL);
    sbl_puts("passed\r\n");
#endif

#if !defined(SBL_ENABLE_CLOCKS) || defined(SBL_ENABLE_CUST_CLOCKS) || defined(SBL_SKIP_SYSFW_INIT)
    sbl_puts("BOARD_INIT_MODULE_CLOCK...");
    Board_init(BOARD_INIT_MODULE_CLOCK);
    sbl_puts("passed\r\n");
#endif

#if !defined(SBL_ENABLE_DDR) || defined(SBL_SKIP_SYSFW_INIT)
    sbl_puts("BOARD_INIT_DDR...");
    Board_init(BOARD_INIT_DDR);
    sbl_puts("passed\r\n");
#endif

    sbl_puts("\r\nAnalyzing run results .... \r\n");

    if (pmuCntrVal == 0)
    {
        sprintf(sbl_test_str,"Do this first: Enable SBL_SKIP_MCU_RESET and set SBL_LOG_LEVEL=1 in %s.\r\n", comp_mk);sbl_puts(sbl_test_str);
        sbl_puts("If the regular mpu_init is good for you, then save some time by skipping/configuring only delta in mpu_init in the app.\r\n");
        sbl_puts("Refer sbl_smp_r5.asm to see how to overrride the default mpu_init with a custom one.\r\n");
        return 0;
    }
    else
    {
        if (pmuCntrVal > 50000000)
        {
            sprintf(sbl_test_str,"Do this next: Disable SBL_DISPLAY_PROFILE_INFO in %s to drastically reduce boot time.\r\n", comp_mk);sbl_puts(sbl_test_str);
            sbl_puts("Also recheck SBL_LOG_LEVEL=1 and no log messages are displayed form the SBL\r\n");
        }
        else
        {
            if (pmuCntrVal < 20000000)
            {
                sbl_puts("Boot time is now optimized....\r\n");
                sbl_puts("All tests have passed\r\n");
            }
            else
            {
                sprintf(sbl_test_str,"Now Try disabling the following one by one in %s to reduce a little more boot time.\r\n", comp_mk);sbl_puts(sbl_test_str);
                sbl_puts("SBL_ENABLE_PLL (big impact to boot time), SBL_ENABLE_DDR (must be disabled if SBL_ENABLE_PLL is disable), SBL_ENABLE_CLOCKS(least imapct).\r\n");
                sbl_puts("Please remember to assess impact of removing PLL init, DDR init (does your app need DDR?) and clock init (does your app use PHYs??) on your app. \r\n");
                sbl_puts("As a last resort, enable (uncomment) SBL_SKIP_BRD_CFG_PM\r\n");
                sbl_puts("Once enabled, all SBL UART logs will be garbled. Remember to call Sciclient_boardCfgPm from the app to get Uart_printf to work.\r\n");
            }
        }
    }

    BOOT_PERF_TEST_printSblProfileLog(sblProfileLogAddr, *sblProfileLogIndxAddr, *sblProfileLogOvrFlwAddr);
    return 0;
}
