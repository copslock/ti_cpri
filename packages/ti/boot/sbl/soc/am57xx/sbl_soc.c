/*
 * Copyright (C) 2016-2017 Texas Instruments Incorporated - http://www.ti.com/
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

#include "sbl_soc.h"
#include <ti/drv/spi/soc/QSPI_v1.h>

/* Flash header file */
#if defined(AM572x_BUILD)
#include <ti/board/src/idkAM572x/device/qspi_flash.h>
#elif defined(AM571x_BUILD)
#include <ti/board/src/idkAM571x/device/qspi_flash.h>
#elif defined(AM574x_BUILD)
#include <ti/board/src/idkAM574x/device/qspi_flash.h>
#endif

/** Memory size 1 byte */
#define MEM_SIZE_1B                (1U)
/** Memory size 1 kilo-byte */
#define MEM_SIZE_KB                (1024U * MEM_SIZE_1B)
/** Memory size 1 gega-byte */
#define MEM_SIZE_MB                (1024U * MEM_SIZE_KB)

/** QSPI device specific definitions */
#define QSPI_INSTANCE                       (1U)
#define QSPI_OFFSET                         (4U)
#define QSPI_DEVICE_SIZE                    (32 * MEM_SIZE_MB)
#define QSPI_DEVICE_BLOCK_SIZE              (64 * MEM_SIZE_KB)

extern QSPI_HwAttrs qspiInitCfg[1];

int32_t SBL_socInit()
{
    Board_initCfg boardCfg;
    boardCfg = BOARD_INIT_DEFAULT & ~(BOARD_INIT_DDR_ECC);
    
    /* Board Library Init. */
    if (Board_init(boardCfg))
    {
        return -1;
    }
    return 0;
}

#if defined (BOOT_QSPI)
int32_t SBL_qspiInit(void *handle)
{
    SPI_Params spiParams;

     /* Initialize SPI driver */
    SPI_init();

    /* Default SPI configuration parameters */
    SPI_Params_init(&spiParams);

    /* Enabling QSPI in polling mode. */
    qspiInitCfg[0].intrEnable = false;

    /* Open QSPI driver */
    *(S25FL_Handle *) handle = SF25FL_open(((QSPI_INSTANCE - 1)+(QSPI_OFFSET)), &spiParams);

    if (!handle)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_qspiFlashWrite(void *handle, uint8_t *src, uint32_t length,
    uint32_t offset)
{
    S25FL_Handle flashHandle = *(S25FL_Handle *) handle;
    uint32_t startBlockNumber, endBlockNumber;
    uint32_t i;
    S25FL_Transaction flashTransaction;

    /* Computing the block numbers to be erased  */
    startBlockNumber = (offset / QSPI_DEVICE_BLOCK_SIZE);
    endBlockNumber = (offset + length) /
        QSPI_DEVICE_BLOCK_SIZE;

    for (i = startBlockNumber; i <= endBlockNumber; i++)
    {
        S25FLFlash_BlockErase(flashHandle, i);
    }

    flashTransaction.data = src;
    flashTransaction.address = (uint32_t) offset;
    flashTransaction.dataSize = length;

    /* Write image to QSPI flash */
    SF25FL_bufferWrite(flashHandle, &flashTransaction);
    return 0;
}

int32_t SBL_qspiFlashRead(void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset)
{
    S25FL_Handle flashHandle = *(S25FL_Handle *) handle;
    S25FL_Transaction flashTransaction;
    int32_t retVal;

    S25FLFlash_QuadModeEnable(flashHandle);

    /* Update transaction parameters for the Read operation. */
    flashTransaction.data       = dst;
    flashTransaction.address    = offset;
    flashTransaction.dataSize   = length;

    /* Read data from flash */
    retVal = SF25FL_bufferRead(flashHandle, &flashTransaction);

    /* SPI_transfer() returns TRUE if successful
       SBL_qspiFlashRead() needs to return 0 if there's no errors */
    return !retVal;
}

int32_t SBL_qspiClose(void *handle)
{
    S25FL_Handle flashHandle = *(S25FL_Handle *) handle;
    S25FLFlash_QuadModeEnable(flashHandle);
    SF25FL_close(flashHandle);
    return 0;
}
#endif /* end of BOOT_QSPI definitions */
