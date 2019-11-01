/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 */

/**
 *  \file main_tirtos.c
 *
 *  \brief Main file for TI-RTOS build
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/board/board.h>
#include <ti/drv/dss/examples/utils/app_utils.h>
#include "dss_display_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Test application stack size */
#define DISP_APP_TSK_STACK_MAIN         (10U * 1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static Void taskFxn(UArg a0, UArg a1);
extern int32_t Dss_displayTest(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Test application stack */
static uint8_t gDispAppTskStackMain[DISP_APP_TSK_STACK_MAIN];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int main(void)
{
    Task_Handle task;
    Error_Block eb;
    Task_Params taskParams;

    Error_init(&eb);

    /* Initialize the task params */
    Task_Params_init(&taskParams);
    /* Set the task priority higher than the default priority (1) */
    taskParams.priority = 2;
    taskParams.stack = gDispAppTskStackMain;
    taskParams.stackSize = sizeof(gDispAppTskStackMain);

    task = Task_create(taskFxn, &taskParams, &eb);
    if(NULL == task)
    {
        BIOS_exit(0);
    }
    BIOS_start();    /* does not return */

    return(0);
}

static Void taskFxn(UArg a0, UArg a1)
{
#if(1U == DISP_APP_TEST_MULTISYNC)
    uint32_t regVal;
#endif
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
                BOARD_INIT_UNLOCK_MMR |
                BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);

#if defined (SOC_AM65XX)
    App_configureLCD(APP_OUTPUT_OLDI);
#else
#if (1U == DISP_APP_TEST_EDP)
    App_configureLCD(APP_OUTPUT_EDP);
#else
    App_configureLCD(APP_OUTPUT_HDMI);
#if (1U == DISP_APP_TEST_MULTISYNC)
    /* HSYNC mode 14 <- VP0 HSYNC */
    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG38);
    CSL_FINS(regVal,
            MAIN_CTRL_MMR_CFG0_PADCONFIG38_MUXMODE,
            0xEU);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG38, regVal);

    /* DE MODE 14 <- VP0 DE */
    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG39);
    CSL_FINS(regVal,
            MAIN_CTRL_MMR_CFG0_PADCONFIG39_MUXMODE,
            0xEU);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG39, regVal);

    /* VSYNC MODE 14 <- VP0 VSYNC */
    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG40);
    CSL_FINS(regVal,
            MAIN_CTRL_MMR_CFG0_PADCONFIG40_MUXMODE,
            0xEU);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG40, regVal);

    /* DPI_0_PCLK <- DPI_1_PCLK */
    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3);
    CSL_FINS(regVal,
            MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3_DPI3_PCLK,
            0x5U);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
            CSL_MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3, regVal);
#endif
#endif
#endif
    App_configureSoC();

    Dss_displayTest();

    return;
}

#if defined(BUILD_MPU) || defined (__C7100__)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif
