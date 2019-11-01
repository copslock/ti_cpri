/**
 *  \file    sbl_ospi.h
 *
 *  \brief   This file contains functions prototypes for UART Boot functionality
 *           of SBL.
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

#ifndef SBL_UART_H
#define SBL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/board/board.h>
#include "sbl_log.h"
#include "sbl_typecast.h"
#include "sbl_profile.h"
#include "sbl_slave_core_boot.h"
#include <ti/drv/uart/soc/UART_soc.h>

/*
 *  \brief    SBL_UARTBootImage function initializes the UART driver and copies
 *            the application image from the UART device to the DDR memory and
 *            gives control to the processor core.
 *
 *  \param    pointer to the structure holding the entry pointers for different
 *            cores.
 *
 *  \return   error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 *
 */
int32_t SBL_UARTBootImage(sblEntryPoint_t *pEntry);

/**
 * @brief - SBL_uartInit() - function to do initialize QSPI
 *
 *
 * @param
 *     handle = pointer to return QSPI handle
 *
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 *
 */
int32_t SBL_uartInit(uint32_t inClkFreqHz);

/**
 *
 * @brief - SBL_uartClose() - function to do close QSPI handle
 *
 *
 * @param
 *
 *     handle = pointer to QSPI handle
 *
 *
 * @return - int32t
 *      0 = Init completed successfully
 *
 *     -1 = Error occurred
 *
 *
 */
int32_t SBL_uartClose(void);

/**
 *  @brief        This function receives the image, using xmodem protocol
 *                from uniflash and copies to the specified location of DDR.
 *
 *  @param        dest    [IN]   DDR address to store image.
 *                destsz  [IN]   Maximum size for storing image to DDR.
 *
 *  @return       uint32_t
 *                size of the image    - in case of success
 *                0                    - in case of failure.
 *
 */
int32_t SBL_uartXmodemRead(uint8_t *dest, uint32_t destsz);
#ifdef __cplusplus
}
#endif

#endif
