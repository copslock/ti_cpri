/**
 *  \file    sbl_ospi.h
 *
 *  \brief   This file contains functions prototypes for OSPI Boot functionality
 *           of SBL.
 *
 */

/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef SBL_OSPI_H
#define SBL_OSPI_H

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

/*
 *  \brief    SBL_OSPIBootImage function initializes the OSPI driver and copies
 *            the application image from the OSPI device to the DDR memory and
 *            gives control to the processor core.
 *
 *  \param    pointer to the structure holding the entry pointers for different
 *            cores.
 *
 *  \return   error status.If error has occured it returns a non zero value.
 *            If no error has occured then return status will be zero.
 *
 */
int32_t SBL_OSPIBootImage(sblEntryPoint_t *pEntry);

/**
 * @brief - SBL_ospiInit() - function to do initialize QSPI
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
int32_t SBL_ospiInit(void *handle);

/**
 * @brief - SBL_ospiFlashRead() - function to do flash QSPI
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
int32_t SBL_ospiFlashRead(const void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset);

/**
 *
 * @brief - SBL_ospiClose() - function to do close QSPI handle
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
int32_t SBL_ospiClose(const void *handle);

#ifdef __cplusplus
}
#endif

#endif
