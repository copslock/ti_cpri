/**
 *  \file    sbl_hyperflash.c
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
#include "sbl_rprc_parse.h"
#include "sbl_err_trap.h"
#include "sbl_sci_client.h"
#include "sbl_soc_cfg.h"

/* TI-RTOS Header files */
#include <ti/drv/gpio/soc/GPIO_soc.h>
#include <ti/drv/spi/SPI.h>
#include <ti/drv/udma/udma.h>
#include <ti/drv/spi/src/SPI_osal.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/drv/spi/src/v0/OSPI_v0.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/arch/csl_arch.h>
#include <ti/csl/arch/r5/csl_arm_r5.h>
#include <ti/csl/arch/r5/interrupt.h>
#include <ti/board/board_cfg.h>
#include <ti/board/src/flash/include/board_flash.h>
#include <ti/csl/csl_hyperbus.h>
#include <ti/csl/cslr_hyperbus.h>
#include "sbl_hyperflash.h"

/* Macro representing the offset where the App Image has to be written/Read from
 * the HyperFlash.
 */
#define HYPERFLASH_OFFSET_SI              (0xC0000U)
#define HYPERFLASH_OFFSET_SYSFW           (0x40000U)

uint32_t gBaseAddress = SBL_HYPERFLASH_BASE_ADDRESS;

CSL_hyperbus_coreRegs *hpbCoreRegs = (CSL_hyperbus_coreRegs *) (SBL_HYPERFLASH_CTLR_BASE_ADDRESS);

int8_t Hyperflash_mdllLocked(void);
static void SBL_HYPERFLASH_seek(void *srcAddr, uint32_t location);
static int32_t SBL_HYPERFLASH_ReadSectors(void *dstAddr, void *srcOffsetAddr,
                                          uint32_t length);
int32_t SBL_hyperflashClose(const void *handle);

static void *boardHandle = NULL;

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

int32_t SBL_ReadSysfwImage(void **pBuffer, uint32_t num_bytes)
{
    Board_flashHandle h;

    SBL_ADD_PROFILE_POINT;

    SBL_hyperflashInit();

    h = Board_flashOpen(BOARD_FLASH_ID_S71KS512S,
                                  BOARD_HPF_INSTANCE, NULL);

    if(h)
    {
        SBL_ADD_PROFILE_POINT;

        SBL_log(SBL_LOG_MAX, "Waiting for sysfw.bin ...\n");

        *pBuffer = (void *)(gBaseAddress + HYPERFLASH_OFFSET_SYSFW);

        /* Update handle for later use*/
        boardHandle = (void *)h;
    }
    else
    {
        SBL_log(SBL_LOG_ERR, "Board_flashOpen failed!\n");
        SblErrLoop(__FILE__, __LINE__);
    }

    SBL_ADD_PROFILE_POINT;

    return CSL_PASS;
}

/*
 * Initiallise the hyperflash
 */

void SBL_hyperflashInit()
{
    int8_t ret;

    /* Configure the pinmux for hyperbus as the default
       pinmux configuration is set for OSPI */
    Board_PinmuxConfig_t pixmuxCfg;

    Board_pinmuxGetCfg(&pixmuxCfg);
    pixmuxCfg.fssCfg = BOARD_PINMUX_FSS_HPB;
    Board_pinmuxSetCfg(&pixmuxCfg);

    Board_init(BOARD_INIT_PINMUX_CONFIG);

    ret = Hyperflash_mdllLocked();
    if(!ret)
    {
        SBL_log(SBL_LOG_ERR, "HyperFlash MDLL locking failed!\n");
        SblErrLoop(__FILE__, __LINE__);
    }
}

/**
 *  \brief    The function validates the locking of the MDLL
 *
 *  \return   int8_t
 *		 1 - in case of success
 *		 0 - in case of failure
 */
int8_t Hyperflash_mdllLocked(void)
{
    uint8_t num_times = NUM_READ_FOR_MDLL_LOCK;
    CSL_REG16_WR(gBaseAddress + HPF_CMD_ADDR1, HPF_WRITE_UNLOCK_CMD1);
    CSL_REG16_WR(gBaseAddress + HPF_CMD_ADDR2, HPF_WRITE_UNLOCK_CMD2);
    CSL_REG16_WR(gBaseAddress + HPF_CMD_ADDR1, HPF_ID_ENTRY);
    CSL_REG16_WR(gBaseAddress + HPF_CMD_ADDR1, HPF_CFI_ENTRY_CMD);

    SBL_log(SBL_LOG_MAX, "Waiting for locking of HyperFlash MDLL...\n");

    /* Issue a few (here 20) reads to the HyperFlash CFI region to read the
     * CFI Query Identification String "QRY"
     */
    while(num_times--)
    {
        if( (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_Q_OFFSET) == 0x51U) && \
            (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_R_OFFSET) == 0x52U) && \
            (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_Y_OFFSET) == 0x59U) )
        {
            SBL_log(SBL_LOG_MAX, "MATCH\n");
        }
        else
        {
            SBL_log(SBL_LOG_MAX, "NOT MATCHED\n");
        }
    }

    /* MDLL should get locked after 16-20 reads, if it doesn't then return 0 */
    if( (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_Q_OFFSET) == 0x51U) && \
        (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_R_OFFSET) == 0x52U) && \
        (CSL_REG16_RD(gBaseAddress + CFI_QUERY_STR_Y_OFFSET) == 0x59U) )
    {
        /* Exit from CFI */
        CSL_REG32_WR(gBaseAddress, HPF_RESET_CMD);
        return 1;
    }
    else
    {
        /* Exit from CFI */
        CSL_REG32_WR(gBaseAddress, HPF_RESET_CMD);
        return 0;
    }
}

int32_t SBL_HYPERFLASHBootImage(sblEntryPoint_t *pEntry)
{
    int32_t retVal = 0;

    /* SBL_hyperflashInit() has already been done in sciclientinit */

    uint32_t imgOffset = HYPERFLASH_OFFSET_SI;

    /* Initialize the function pointers to parse through the RPRC format. */
    fp_readData = &SBL_HYPERFLASH_ReadSectors;
    fp_seek     = &SBL_HYPERFLASH_seek;

    SBL_log(SBL_LOG_MAX, "Waiting for multicore app ...\n");

    retVal =  SBL_MulticoreImageParse((void *)&imgOffset, HYPERFLASH_OFFSET_SI, pEntry);

    SBL_hyperflashClose(&boardHandle);

    return retVal;
}

static int32_t SBL_HYPERFLASH_ReadSectors(void *dstAddr, void *srcOffsetAddr,
                                          uint32_t length)
{
    int32_t ret;

    ret = SBL_hyperflashRead(&boardHandle, *((uint32_t *)srcOffsetAddr),
                             dstAddr, length);

    *((uint32_t *) srcOffsetAddr) += length;

    return ret;
}

static void SBL_HYPERFLASH_seek(void *srcAddr, uint32_t location)
{
    SBL_log(SBL_LOG_MAX, "\nSeek: srcAddr=0x%x, location=%d\n", srcAddr, location);
    *((uint32_t *) srcAddr) = location;
}


/**
 *  \brief    Reads data from hyperflash and check for the device status.
 *
 *  \param    offset            [IN]        Offset to read data from hyperflash
 *  \param    dataBuff          [OUT]       Buffer to store read data
 *  \param    rdCount           [IN]        Number of bytes to read.
 *
 */
static int32_t SBL_hyperflashRead(const void *handle, uint32_t offset,
                                  void *dataBuff,
                                  uint32_t rdCount)
{
    SBL_ADD_PROFILE_POINT;

    /*
     * Curently all the memories have been configured as write through,
     * no write allocate. In that case this works fine.
     * But in case the R5 is booting an A53 image and if the memory is cached,
     * then the memcopy data would be in cache and not updated in the actual
     * memory. Later when the A53 starts running, it would access the memory,
     * read the stale data and crash.
     */
    memcpy(dataBuff, (uint16_t *)(gBaseAddress + offset), rdCount);

    SBL_ADD_PROFILE_POINT;

    return 0;
}

int32_t SBL_hyperflashClose(const void *handle)
{
    Board_flashHandle h = *(const Board_flashHandle *) handle;

    SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MAX, "SBL_hyperflashClose called!\n");
    Board_flashClose(h);
    SBL_ADD_PROFILE_POINT;

    return 0;
}
