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
#include <stdint.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "ipc_utils.h"
#if defined (__C7100__)
#include <ti/sysbios/family/c7x/Mmu.h>
#endif

#ifdef IPC_SUPPORT_SCICLIENT
#include <ti/drv/sciclient/sciclient.h>
#endif

#include <ti/board/board.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static Void taskFxn(UArg a0, UArg a1);
extern int32_t Ipc_echo_test(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


#ifdef IPC_SUPPORT_SCICLIENT
void ipc_initSciclient()
{
    Sciclient_ConfigPrms_t        config;

    /* Now reinitialize it as default parameter */
    Sciclient_configPrmsInit(&config);

#if defined(BUILD_MPU1_0) || defined(BUILD_C7X_1)
    config.opModeFlag  = SCICLIENT_SERVICE_OPERATION_MODE_POLLED;
#endif

    Sciclient_init(&config);

}

void ipc_boardInit()
{
    Board_initCfg           boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_UART_STDIO;
    Board_init(boardCfg);

}
#endif

int main(void)
{
    Task_Handle task;
    Error_Block eb;
    Task_Params taskParams;

#ifdef IPC_SUPPORT_SCICLIENT
    ipc_boardInit();
#endif


#if defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
/* To set C66 timer interrupts on J7ES VLAB */
    //C66xTimerInterruptInit();
#endif

#ifdef BUILD_C7X_1
    Ipc_appC7xPreInit();
    //C7x_ConfigureTimerOutput();
#endif

    Error_init(&eb);

    /* Initialize the task params */
    Task_Params_init(&taskParams);
    /* Set the task priority higher than the default priority (1) */
    taskParams.priority = 2;

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
#ifdef IPC_SUPPORT_SCICLIENT
    ipc_initSciclient();
#endif
    Ipc_echo_test();
    return;
}

#if defined (__C7100__)
void InitMmu(void)
{
    Bool            retVal;
    Mmu_MapAttrs    attrs;

    Mmu_initMapAttrs(&attrs);

    attrs.attrIndx = Mmu_AttrIndx_MAIR0;

    retVal = Mmu_map(0x00000000, 0x00000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x20000000, 0x20000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x030800000, 0x030800000, 0xC000000, &attrs); /* navss        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x40000000, 0x40000000, 0x20000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0x60000000, 0x60000000, 0x10000000, &attrs);
    if(retVal==FALSE)
    {
        goto mmu_exit;
    }


    attrs.attrIndx = Mmu_AttrIndx_MAIR7;
    retVal = Mmu_map(0x80000000, 0x80000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    retVal = Mmu_map(0xA0000000, 0xA0000000, 0x20000000, &attrs); /* ddr            */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    /*
     * DDR range 0xA0000000 - 0xAA000000 : Used as RAM by multiple
     * remote cores, no need to mmp_map this range.
     * IPC VRing Buffer - uncached
     * */
    attrs.attrIndx =  Mmu_AttrIndx_MAIR4;
    retVal = Mmu_map(0xAA000000, 0xAA000000, 0x02000000, &attrs);
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

    attrs.attrIndx = Mmu_AttrIndx_MAIR0;
    retVal = Mmu_map(0x70000000, 0x70000000, 0x10000000, &attrs); /* msmc        */
    if(retVal == FALSE)
    {
        goto mmu_exit;
    }

mmu_exit:
    if(retVal == FALSE)
    {
         System_printf(" ERROR: MMU init failed (status = %d) !!!", retVal);
    }

    return;
}
#endif

