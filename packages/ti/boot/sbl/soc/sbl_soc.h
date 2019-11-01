/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef SBL_SOC_H
#define SBL_SOC_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/board/board.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/arch/csl_arch.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SBL_INVALID_ENTRY_ADDR ((uint32_t)0xFFFFFFFEU)

/**
 * @brief - SBL_socInit() - function to do initialize settings based on soc
 *
 * @param   none
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 */
int32_t SBL_socInit(void);

#if defined(BOOT_QSPI)
/**
 * @brief - SBL_qspiInit() - function to do initialize QSPI
 *
 * @param
 *     handle = pointer to return QSPI handle
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 */
int32_t SBL_qspiInit(void *handle);

/**
 * @brief - SBL_qspiFlashWrite() - function to do flash QSPI
 *
 * @param
 *     handle = pointer to QSPI handle
 *     src = byte pointer to source
 *     length = size of source to copy
 *     offset = QSPI offset to flash into
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     <0 = Negative value indicate error occurred
 *
 */
int32_t SBL_qspiFlashWrite(void *handle, uint8_t *src, uint32_t length,
    uint32_t offset);

/**
 * @brief - SBL_qspiFlashRead() - function to do flash QSPI
 *
 * @param
 *     handle = pointer to QSPI handle
 *     dst = byte pointer to destination
 *     length = size of source to copy
 *     offset = QSPI offset to flash into
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     <0 = Negative value indicate error occurred
 *
 */
int32_t SBL_qspiFlashRead(void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset);

/**
 * @brief - SBL_qspiClose() - function to do close QSPI handle
 *
 * @param
 *     handle = pointer to QSPI handle
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 */
int32_t SBL_qspiClose(void *handle);

#endif /* end of BOOT_QSPI definitions */

#if defined(BOOT_SPI)
/**
 * @brief - SBL_spiInit() - function to do initialize SPI
 *
 * @param
 *     handle = pointer to return SPI handle
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 */
int32_t SBL_spiInit(void *handle);

/**
 * @brief - SBL_spiFlashWrite() - function to do flash SPI
 *
 * @param
 *     handle = pointer to SPI handle
 *     src = byte pointer to source
 *     length = size of source to copy
 *     offset = SPI offset to flash into
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     <0 = Negative value indicate error occurred
 *
 */
int32_t SBL_spiFlashWrite(void *handle, uint8_t *src, uint32_t length,
    uint32_t offset);

/**
 * @brief - SBL_spiFlashRead() - function to do flash SPI
 *
 * @param
 *     handle = pointer to SPI handle
 *     dst = byte pointer to destination
 *     length = size of source to copy
 *     offset = SPI offset to flash into
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     <0 = Negative value indicate error occurred
 *
 */
int32_t SBL_spiFlashRead(void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset);

/**
 * @brief - SBL_spiClose() - function to do close SPI handle
 *
 * @param
 *     handle = pointer to SPI handle
 *
 * @return - int32t
 *      0 = Init completed successfully
 *     -1 = Error occurred
 *
 */
int32_t SBL_spiClose(void *handle);

#endif /* end of BOOT_SPI definitions */

#endif
