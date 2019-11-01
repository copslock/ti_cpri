/**
 *  \file    sbl_uart.c
 *
 *  \brief   This file contains functions for UART read/write operations for SBL
 *
 */

/*
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "string.h"

/* SBL Header files. */
#include "sbl_soc.h"
#include "sbl_soc_cfg.h"
#include "sbl_rprc_parse.h"
#include "sbl_err_trap.h"
#include "sbl_sci_client.h"

/* TI-RTOS Header files */
#include <ti/drv/udma/udma.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/board/board_cfg.h>
#include "sbl_uart.h"

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/
#define XMODEM_TEMP_BUFF_SIZE    ((SBL_SCRATCH_MEM_SIZE) / 2)
#define SBL_XMODEM_BUFF_ADDR     ((SBL_SCRATCH_MEM_START) + (XMODEM_TEMP_BUFF_SIZE))

static int32_t SBL_UART_ReadXmodemBuffer(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length)
{
    int32_t ret = 0;
    uint32_t *xModemPktBuffIndx = (uint32_t *)srcOffsetAddr;
    void *srcAddr = (void *)(SBL_XMODEM_BUFF_ADDR + *xModemPktBuffIndx);

    SBL_log(SBL_LOG_MAX, "\nRead: dstAddr=0x%x, srcOffsetAddr=0x%x, srcAddr=0x%x, length=%d\n", dstAddr, srcOffsetAddr, srcAddr, length);

    memcpy(dstAddr, srcAddr, length);
    *xModemPktBuffIndx += length;

    return ret;
}

static void SBL_UART_seek(void *srcAddr, uint32_t location)
{
    uint32_t *xModemPktBuffIndx = (uint32_t *)srcAddr;

    SBL_log(SBL_LOG_MAX, "\nSeek: srcAddr=0x%x, location=%d\n", srcAddr, location);
    *xModemPktBuffIndx = location;
}

int32_t SBL_ReadSysfwImage(void **pBuffer, uint32_t num_bytes)
{
    SBL_ADD_PROFILE_POINT;

    SBL_uartInit(SBL_ROM_UART_MODULE_INPUT_CLK);

    SBL_log(SBL_LOG_MIN, "Waiting for sysfw.bin ...\n");

    SBL_uartXmodemRead((uint8_t *)(*pBuffer), SBL_SYSFW_MAX_SIZE);

    SBL_ADD_PROFILE_POINT;

    return CSL_PASS;
}

int32_t SBL_uartInit(uint32_t inClkFreqHz)
{
    /* Init UART for xmodem boot */
    UART_HwAttrs uart_cfg;

    SBL_ADD_PROFILE_POINT;

    UART_socGetInitCfg(BOARD_UART_INSTANCE, &uart_cfg);
    uart_cfg.frequency = inClkFreqHz;
    /* Disable the UART interrupt */
    uart_cfg.enableInterrupt = FALSE;
    UART_socSetInitCfg(BOARD_UART_INSTANCE, &uart_cfg);

    UART_stdioInit(BOARD_UART_INSTANCE);

    SBL_ADD_PROFILE_POINT;

    return 0;
}

int32_t SBL_uartClose(void)
{
    SBL_ADD_PROFILE_POINT;

    UART_stdioDeInit();

    SBL_ADD_PROFILE_POINT;

    return 0;
}

int32_t SBL_UARTBootImage(sblEntryPoint_t *pEntry)
{
    int32_t retVal = 0;
    uint32_t imgOffset = 0;

    /* Re-initialize the uart to a different freq */
    /* depending on whether SysFwConfigPm has run */
    /* or not                                     */
#if defined(DSBL_SKIP_BRD_CFG_PM) || defined(SBL_SKIP_SYSFW_INIT)
    SBL_uartInit(SBL_ROM_UART_MODULE_INPUT_CLK);
#else
    SBL_uartInit(SBL_SYSFW_UART_MODULE_INPUT_CLK);
#endif

    /* Initialize the function pointers to parse through the RPRC format. */
    fp_readData = &SBL_UART_ReadXmodemBuffer;
    fp_seek     = &SBL_UART_seek;

    SBL_log(SBL_LOG_MIN, "Waiting for multicore app ...\n");

    SBL_uartXmodemRead((uint8_t *)SBL_XMODEM_BUFF_ADDR, XMODEM_TEMP_BUFF_SIZE);

    retVal =  SBL_MulticoreImageParse((void *)&imgOffset, imgOffset, pEntry);

    return retVal;
}
