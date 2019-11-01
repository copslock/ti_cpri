/*
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
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
#include "sbl_a15.h"

#include <ti/board/board_cfg.h>
#include <ti/board/src/flash/include/board_flash.h>
#include <ti/board/src/flash/nor/qspi/nor_qspi.h>
#include <ti/drv/spi/soc/SPI_soc.h>

#ifdef iceK2G
#define SBL_QSPI_FLASH_ID   BOARD_FLASH_ID_QSPIFLASH_S25FL256S
#else
#define SBL_QSPI_FLASH_ID   BOARD_FLASH_ID_QSPIFLASH_S25FL512S
#endif

int32_t SBL_socInit()
{
    Board_initCfg boardCfg;
    boardCfg = BOARD_INIT_PLL | 
        BOARD_INIT_MODULE_CLOCK | 
        BOARD_INIT_DDR |
        BOARD_INIT_PINMUX_CONFIG | 
        BOARD_INIT_UART_STDIO;

#ifndef SECURE_BOOT
    void (*monitorFunction) (void (*)(void), ...);

    /* A15 startup calls */
    monitorFunction = (void (*)) 0x1000;

    (*monitorFunction)(SBL_setNSMode);
    (*monitorFunction)(SBL_a15EnableNeon);
    SBL_a15EnableVFP11co();
#endif

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
    QSPI_v0_HwAttrs qspi_cfg;

    /* Get the default QSPI init configurations */
    QSPI_socGetInitCfg(BOARD_QSPI_NOR_INSTANCE, &qspi_cfg);

    /* Modify the default QSPI configurations if necessary */
    /* Turning off interrupts for baremetal mode. May be re-enabled by app */
    qspi_cfg.intrEnable = false;

    /* Set the default QSPI init configurations */
    QSPI_socSetInitCfg(BOARD_QSPI_NOR_INSTANCE, &qspi_cfg);

    /* Open the Board QSPI NOR device with QSPI port 0
       and use default QSPI configurations */
    *(Board_flashHandle *) handle = Board_flashOpen(SBL_QSPI_FLASH_ID,
                            BOARD_QSPI_NOR_INSTANCE, NULL);

    if (!handle)
    {
        return -1;
    }

    return 0;
}

int32_t SBL_qspiFlashWrite(void *handle, uint8_t *src, uint32_t length,
    uint32_t offset)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;
    uint32_t startBlockNum, endBlockNum, pageNum;
    uint32_t ioMode, i;
    ioMode = BOARD_FLASH_QSPI_IO_MODE_QUAD;

    /* Get starting block number */
    if (Board_flashOffsetToBlkPage(h, offset, &startBlockNum, &pageNum))
    {
        return -1;
    }

    /* Get ending block number */
    endBlockNum = (offset+length)/(NOR_SECTOR_SIZE);

    /* Erase blocks, to which data has to be written */
    for(i = startBlockNum; i <= endBlockNum; i++)
    {
        if (Board_flashEraseBlk(h, i))
        {
            return -2;
        }
    }

    /* Write buffer to flash */
    if (Board_flashWrite(h, offset, src, length, (void *)(&ioMode)))
    {
        return -3;
    }

    return 0;
}

int32_t SBL_qspiFlashRead(void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;
    uint32_t ioMode;
    ioMode = BOARD_FLASH_QSPI_IO_MODE_QUAD;

    if (Board_flashRead(h, offset, dst, length, (void *)(&ioMode)))
    {
        return -1;
    }

    return 0;
}

int32_t SBL_qspiClose(void *handle)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;
    Board_flashClose(h);
    return 0;
}
#endif /* end of BOOT_QSPI definitions */
