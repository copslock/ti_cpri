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

#include "sbl_soc.h"
#if defined(A15_CORE)
#include "sbl_a15.h"
#endif

#include <ti/board/src/evmK2E/include/board_cfg.h>
#include <ti/board/src/flash/include/board_flash.h>
#include <ti/drv/spi/soc/SPI_soc.h>

int32_t SBL_socInit()
{
    Board_initCfg boardCfg;
    boardCfg = BOARD_INIT_PLL | 
        BOARD_INIT_MODULE_CLOCK | 
        BOARD_INIT_DDR |
        BOARD_INIT_UART_STDIO;

#if defined(A15_CORE)
    void (*monitorFunction) (void (*)(void), ...);

    /* A15 startup calls */
    monitorFunction = (void (*)) 0x1000;
    (*monitorFunction)(SBL_setNSMode);
    (*monitorFunction)(SBL_a15EnableNeon);
    (*monitorFunction)(SBL_a15EnableSMP);
    SBL_a15EnableVFP11co();
#endif

    /* Board Library Init. */
    if (Board_init(boardCfg))
    {
        return -1;
    }
    return 0;
}

#if defined (BOOT_SPI)
int32_t SBL_spiInit(void *handle)
{
    SPI_v0_HWAttrs spi_cfg;
    SPI_Params       spiParams;

    /* Get the default SPI init configurations */
    SPI_socGetInitCfg(BOARD_SPI_NOR_INSTANCE, &spi_cfg);

    /* Modify the default SPI configurations if necessary */
    /* Turning off interrupts for baremetal mode. May be re-enabled by app */
    spi_cfg.enableIntr = false;

    /* Set the default SPI init configurations */
    SPI_socSetInitCfg(BOARD_SPI_NOR_INSTANCE, &spi_cfg);

    /* Default SPI configuration parameters */
    SPI_Params_init(&spiParams);
    spiParams.frameFormat  = SPI_POL0_PHA1;

    *(Board_flashHandle *) handle = Board_flashOpen(BOARD_FLASH_ID_NORN25Q128,
                            BOARD_SPI_NOR_INSTANCE, &spiParams);

    if (*(uint32_t *) handle == 0)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_spiFlashWrite(void *handle, uint8_t *src, uint32_t length,
    uint32_t offset)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;
    int32_t ret;
    uint32_t blockNum, pageNum, len, lenOffset;
    uint8_t dst[1];

    /* Iterate through block size = 0x10000 */
    len = length;
    lenOffset = 0;
    while (len > 0x10000)
    {
        if ((ret = Board_flashOffsetToBlkPage(h, offset+lenOffset, &blockNum, &pageNum)))
        {
            return ret;
        }

        /* Erase block, to which data has to be written */
        if ((ret = Board_flashEraseBlk(h, blockNum)))
        {
            return ret;
        }
        /* Write buffer to flash */
        if ((ret = Board_flashWrite(h, offset+lenOffset, src+lenOffset, 0x10000, NULL)))
        {
            return ret;
        }

        /* dummy read */
        Board_flashRead(h, offset+lenOffset, dst, 1, NULL);
        len -= 0x10000;
        lenOffset += 0x10000;
    }

    /* Flash remainder as necessary */
    if ((ret = Board_flashOffsetToBlkPage(h, offset+lenOffset, &blockNum, &pageNum)))
    {
        return ret;
    }

    /* Erase block, to which data has to be written */
    if ((ret = Board_flashEraseBlk(h, blockNum)))
    {
        return ret;
    }

    /* Write buffer to flash */
    if ((ret = Board_flashWrite(h, offset+lenOffset, src+lenOffset, len, NULL)))
    {
        return ret;
    }

    return 0;
}

int32_t SBL_spiFlashRead(void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;

    if (Board_flashRead(h, offset, dst, length, NULL))
    {
        return -1;
    }

    return 0;
}

int32_t SBL_spiClose(void *handle)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;
    Board_flashClose(h);
    return 0;
}
#endif
