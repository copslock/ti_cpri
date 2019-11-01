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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>

/* TI-RTOS Header files */
#include <ti/drv/spi/SPI.h>
#include <ti/drv/spi/src/SPI_osal.h>

/* SBL Header files. */
#include "sbl_soc.h"
#include "sbl_rprc_parse.h"

/* Macro representing the offset where the App Image has to be written/Read from 
   the SPI Flash.
*/
#define SPI_OFFSET_SI              (0x80000)

/* SPI Flash Read Sector API. */
int32_t SBL_SPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length);

/* Initialize the SPI driver and the controller. */
void SBL_SPI_Initialize();

/* Sets the src address to the given offset address. */
void SBL_SPI_seek(void *srcAddr, uint32_t location);

int32_t SBL_SPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length);

void *boardHandle;

int32_t SBL_SPIBootImage(sblEntryPoint_t *pEntry)
{
    int32_t retVal;
    uint32_t offset = SPI_OFFSET_SI;

    /* Initialization of the driver. */
    SBL_SPI_Initialize();

    retVal =  SBL_MulticoreImageParse((void *) &offset, SPI_OFFSET_SI, pEntry);

    /* Close the SPI driver */
    SBL_spiClose(&boardHandle);

    return retVal;
}

void SBL_SPI_Initialize()
{
    SBL_spiInit(&boardHandle);

    /* Initialize the function pointers to parse through the RPRC format. */
    fp_readData = &SBL_SPI_ReadSectors;
    fp_seek     = &SBL_SPI_seek;
}

int32_t SBL_SPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length)
{
    int32_t ret;
    ret = SBL_spiFlashRead(&boardHandle, (uint8_t *) dstAddr, length, 
        *((uint32_t *) srcOffsetAddr));
    *((uint32_t *) srcOffsetAddr) += length;
    return ret;
}

void SBL_SPI_seek(void *srcAddr, uint32_t location)
{
    *((uint32_t *) srcAddr) = location;
}
