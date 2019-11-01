/**
 *  \file   sbl_main.c
 *
 *  \brief  This file contain main function, call the Board Initialization
 *          functions & slave core boot-up functions in sequence.
 *
 */

/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
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
#include <ti/csl/cslr_device.h>
#include <ti/board/board.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/src/UART_osal.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/tistdtypes.h>
#include <ti/csl/csl_a15.h>

#include "sbl_slave_core_boot.h"
#include "sbl_avs_config.h"
#include "sbl_ver.h"

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
sblEntryPoint_t evmAM572xEntry;

typedef void (*EntryFunPtr_t)(void);

int main()
{
    void (*func_ptr)(void);
    Board_initCfg boardCfg;
    uint32_t oppMode = OPP_MODE_NOM;

    #if defined(OPP_HIGH)
        boardCfg = BOARD_INIT_PLL_OPP_HIGH;
        oppMode = OPP_MODE_HIGH;
    #elif defined(OPP_OD)
        boardCfg = BOARD_INIT_PLL_OPP_OD;
        oppMode = OPP_MODE_OD;
    #elif defined(OPP_NOM)
        boardCfg = BOARD_INIT_PLL_OPP_NOM;
        oppMode = OPP_MODE_NOM;
    #endif

    boardCfg |= BOARD_INIT_UNLOCK_MMR |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_DDR |
        BOARD_INIT_UART_STDIO |
        BOARD_INIT_WATCHDOG_DISABLE;

    /* Configure AVS voltage for the selected OPP to the voltage rails. */
    SBL_Configure_AVS(oppMode);

    /* Board Library Init. */
    Board_init(boardCfg);

    /* enable clocks for slave core modules. */
    SBL_SlaveCorePrcmEnable();

    UART_printf("**** PDK SBL ****\n");
    UART_printf("%s (%s - %s)\n", SBL_VERSION_STR, __DATE__, __TIME__);

    UART_printf("Begin parsing user application\n");

    /* Image Copy */
    SBL_ImageCopy(&evmAM572xEntry);

    /* Cache Write back after image copy to ensure the slave cores are brought
    ** out of reset correctly.
    */
    CSL_a15WbAllDataCache();

    UART_printf("Jumping to user application...\n");

    if (evmAM572xEntry.entryPoint_MPU_CPU1 != 0)
    {
        /* Bring the A15 CPU1 core out of reset. */
        SBL_MPU_CPU1_BringUp(evmAM572xEntry.entryPoint_MPU_CPU1);
    }

    if (evmAM572xEntry.entryPoint_DSP1 != 0)
    {
        /* Release the DSP1 core out of reset */
        SBL_DSP1_BringUp(evmAM572xEntry.entryPoint_DSP1);
    }

    if (evmAM572xEntry.entryPoint_DSP2 != 0)
    {
        /* Release the DSP2 core out of reset */
        SBL_DSP2_BringUp(evmAM572xEntry.entryPoint_DSP2);
    }

    if (evmAM572xEntry.entryPoint_IPU1_CPU0 != 0)
    {
        /* Release the IPU1 core out of reset and set the Entry point */
        SBL_IPU1_CPU0_BringUp(evmAM572xEntry.entryPoint_IPU1_CPU0);
    }

    if (evmAM572xEntry.entryPoint_IPU1_CPU1 != 0)
    {
        SBL_IPU1_CPU1_BringUp(evmAM572xEntry.entryPoint_IPU1_CPU1);
    }

    if (evmAM572xEntry.entryPoint_IPU2_CPU0 != 0)
    {
        /* Release the IPU2 CPU0 core out of reset and set the Entry point */
        SBL_IPU2_CPU0_BringUp(evmAM572xEntry.entryPoint_IPU2_CPU0);
    }

    if (evmAM572xEntry.entryPoint_IPU2_CPU1 != 0)
    {
         /* Release the IPU2 CPU1 core out of reset and set the Entry point */
        SBL_IPU2_CPU1_BringUp(evmAM572xEntry.entryPoint_IPU2_CPU1);
    }

    /*Jump to MPU CPU0 APP*/
    if (evmAM572xEntry.entryPoint_MPU_CPU0 != 0)
    {
        func_ptr = (EntryFunPtr_t) evmAM572xEntry.entryPoint_MPU_CPU0;
        CSL_a15WbAllDataCache();
        CSL_a15InvAllInstrCache();
        __sync_synchronize();
        func_ptr();
    }
    else
    {
        while(1);
    }

    return 0;
}
