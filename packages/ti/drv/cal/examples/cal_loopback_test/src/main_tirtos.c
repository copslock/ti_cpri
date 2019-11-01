/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**< Demo application stack size */
#define CAPT_APP_TSK_STACK_MAIN         (10U * 1024U)

/**< Demo application task priority */
#define CAPT_APP_TSK_PRI_MAIN           (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
static Void taskFxn(UArg a0, UArg a1);
extern void Cal_loopbackTest(void);
static int32_t appBoardInit(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/**< Align stack memory to integer boundary. */
static uint8_t gCaptAppTskStackMain[CAPT_APP_TSK_STACK_MAIN]
                    __attribute__ ((section (".bss:taskStackSection")))
                    __attribute__((aligned(32)));
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
    taskParams.priority      = CAPT_APP_TSK_PRI_MAIN;
    taskParams.stack         = gCaptAppTskStackMain;
    taskParams.stackSize     = sizeof (gCaptAppTskStackMain);

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
    appBoardInit();
    Cal_loopbackTest();
    return;
}

static int32_t appBoardInit(void)
{
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
                BOARD_INIT_UNLOCK_MMR |
                BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);

#if defined (SOC_AM65XX)
    App_configureLCD(APP_OUTPUT_OLDI);
#else
    App_configureLCD(APP_OUTPUT_HDMI);
#endif
    App_configureSoC();

    return 0;
}

#if defined(BUILD_MPU) || defined (__C7100__)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif
