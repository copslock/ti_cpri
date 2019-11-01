/**
 *  \file    sbl_qspi.c
 *
 *  \brief   This file contains functions for QSPI read/write operations for SBL
 *
 */

/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

/* TI-RTOS Header files */
#include <ti/drv/spi/SPI.h>
#include <ti/drv/spi/src/SPI_osal.h>
#include <ti/drv/uart/UART_stdio.h>

/* SBL Header files. */
#include "sbl_rprc_parse.h"
#include "sbl_soc.h"

#ifdef SECURE_BOOT
#include "sbl_sec.h"
#endif

/* Macro representing the offset where the App Image has to be written/Read from 
   the QSPI Flash.
*/
#define QSPI_OFFSET_SI              (0x80000)

/* QSPI Flash Read Sector API. */
int32_t SBL_QSPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length);

/* Initialize the QSPI driver and the controller. */
void SBL_QSPI_Initialize();

/* Sets the src address to the given offset address. */
void SBL_QSPI_seek(void *srcAddr, uint32_t location);

int32_t SBL_QSPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length);

void *boardHandle;

#ifdef SECURE_BOOT
extern SBL_incomingBootData_S sblInBootData;

int32_t SBL_loadQSPIBootData(void);
#endif

int32_t SBL_QSPIBootImage(sblEntryPoint_t *pEntry)
{
    int32_t retVal;

#ifdef SECURE_BOOT
    uint32_t authenticated = 0;
    uint32_t srcAddr = 0;
    uint32_t imgOffset = 0;
#else
    uint32_t offset = QSPI_OFFSET_SI;
#endif

    /* Initialization of the driver. */
    SBL_QSPI_Initialize();

#ifndef SECURE_BOOT
    retVal =  SBL_MulticoreImageParse((void *) &offset, QSPI_OFFSET_SI, pEntry);
#else
    retVal = SBL_loadQSPIBootData();

    if (retVal == E_PASS)
    { 
        /* authentiate it */
        authenticated = SBL_authentication(sblInBootData.sbl_boot_buff);
        if (authenticated == 0)
        {
            /* fails authentiation */
            UART_printf("\n QSPI Boot - fail authentication\n");

            retVal = E_FAIL;
        }
        else
        {
            /* need to skip the TOC headers */
            imgOffset = ((uint32_t*)sblInBootData.sbl_boot_buff)[0];
            srcAddr = (uint32_t)(sblInBootData.sbl_boot_buff) + imgOffset; 
            retVal = SBL_MulticoreImageParse((void *)srcAddr, 0, pEntry);
        }
    }
    else
    {
        retVal = E_FAIL;
        UART_printf("\n QSPI Boot - problem processing image \n");
    }

    /* install RAM Secure Kernel and overwrite DSP secure server*/
    UART_printf("\n Starting Secure Kernel on DSP...\n");
    SBL_startSK();

#endif

    /* Close the QSPI driver */
    SBL_qspiClose(&boardHandle);

    return retVal;
}

void SBL_QSPI_Initialize()
{
    SBL_qspiInit(&boardHandle);

    /* Initialize the function pointers to parse through the RPRC format. */

#ifndef SECURE_BOOT
    fp_readData = &SBL_QSPI_ReadSectors;
    fp_seek     = &SBL_QSPI_seek;
#else
    fp_readData = &SBL_MemRead;
    fp_seek     = &SBL_MemSeek;
#endif

}


#ifndef SECURE_BOOT

int32_t SBL_QSPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length)
{
    int32_t ret;
    ret = SBL_qspiFlashRead(&boardHandle, (uint8_t *) dstAddr, length, 
        *((uint32_t *) srcOffsetAddr));
    *((uint32_t *) srcOffsetAddr) += length;
    return ret;
}

void SBL_QSPI_seek(void *srcAddr, uint32_t location)
{
    *((uint32_t *) srcAddr) = location;
}

#else

/* load signed boot data from QSPI */
int32_t SBL_loadQSPIBootData()
{
    int32_t  retVal = E_PASS;
    uint32_t key_size = 0;
    uint32_t load_size = 0;
    uint32_t total_size = 0;
    uint8_t  *u8Ptr = 0;

    sblInBootData.sbl_boot_size = 0;
    sblInBootData.sbl_boot_buff_idx = 0;    /* reset the read pointer */

    u8Ptr = sblInBootData.sbl_boot_buff;

    /* first read a block to figure out the max size */
    retVal = SBL_qspiFlashRead(&boardHandle, sblInBootData.sbl_boot_buff, 
                               READ_BUFF_SIZE, QSPI_OFFSET_SI);

    if (retVal == E_PASS)
    {
        if (strcmp((char *)&u8Ptr[TOC_HDR_SIZE + TOC_FILE_NAME_OFFSET], "KEYS")==0)
        {
            /* first TOC HDR is IMG. Second TOC HDR is image */
            key_size = *(uint32_t*)(&u8Ptr[TOC_HDR_SIZE + TOC_DAT_SIZE_OFFSET]);

            if (strcmp((char *)&u8Ptr[TOC_FILE_NAME_OFFSET], "2ND")==0)
            {
                load_size = *(uint32_t*)(&u8Ptr[TOC_DAT_SIZE_OFFSET]);

                total_size = key_size + load_size + 0x60;
                sblInBootData.sbl_boot_size = total_size;

                if (total_size < SBL_MAX_BOOT_BUFF_SIZE)
                {
                    /* read the entire boot data in */
                    retVal = SBL_qspiFlashRead(&boardHandle,
                                            sblInBootData.sbl_boot_buff,
                                            total_size,
                                            QSPI_OFFSET_SI);
                }
                else
                {
                    /* not enough buffer to read entire boot data */
                    retVal = E_FAIL;
                }
	        }
            else
            {
                /* bad or unsupported image type */
                retVal = E_FAIL;
            }
        }
        else
        {
	        /* Keys not found */
            retVal = E_FAIL;
	    }
    }
    else
    {
        /* Fail to read the QSPI flash */
        retVal = E_FAIL;
    }


    return retVal;
}
#endif


