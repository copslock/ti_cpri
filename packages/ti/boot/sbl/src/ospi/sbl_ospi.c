/**
 *  \file    sbl_ospi.c
 *
 *  \brief   This file contains functions for OSPI read/write operations for SBL
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
#include "sbl_ospi.h"

#ifdef SECURE_BOOT
#include "sbl_sec.h"
#endif

extern OSPI_v0_HwAttrs ospiInitCfg[];

/* Macro representing the offset where the App Image has to be written/Read from
   the OSPI Flash.
*/
#define OSPI_FLASH_BASE_ADDR        ((uint32_t)(ospiInitCfg[BOARD_OSPI_NOR_INSTANCE].dataAddr))
#define OSPI_CTLR_BASE_ADDR         (ospiInitCfg[BOARD_OSPI_NOR_INSTANCE].baseAddr)
#define OSPI_OFFSET_SI              (0xA0000U)
#define OSPI_OFFSET_SYSFW           (0x40000U)
#define OSPI_MPU_REGION_NUM         (0x6)
#define OSPI_MPU_ENABLE_REGION      (0x1)

/* OSPI Flash Read Sector API. */
static int32_t SBL_OSPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length);

/* Initialize the OSPI driver and the controller. */
static void SBL_OSPI_Initialize(void);

/* Sets the src address to the given offset address. */
static void SBL_OSPI_seek(void *srcAddr, uint32_t location);

void SBL_DCacheClean(void *addr, uint32_t size);

void SBL_SysFwLoad(void *dst, void *src, uint32_t size);

static void *boardHandle = NULL;

static OSPI_v0_HwAttrs ospi_cfg;

#ifdef SECURE_BOOT
extern SBL_incomingBootData_S sblInBootData;

int32_t SBL_loadOSPIBootData(void);
#endif

#if SBL_USE_DMA

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much memcpy operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * \
                                         UDMA_TEST_APP_RING_ENTRY_SIZE)
/**
 *  \brief UDMA TR packet descriptor memory.
 *  This contains the CSL_UdmapCppi5TRPD + Padding to sizeof(CSL_UdmapTR15) +
 *  one Type_15 TR (CSL_UdmapTR15) + one TR response of 4 bytes.
 *  Since CSL_UdmapCppi5TRPD is less than CSL_UdmapTR15, size is just two times
 *  CSL_UdmapTR15 for alignment.
 */
#define UDMA_TEST_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaChObj;
struct Udma_EventObj    gUdmaCqEventObj;

static Udma_DrvHandle          gDrvHandle = NULL;
/*
 * UDMA Memories
 */
uint8_t gTxRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gTxCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gTxTdCompRingMem[UDMA_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
uint8_t gUdmaTprdMem[UDMA_TEST_APP_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
OSPI_dmaInfo gUdmaInfo;

static int32_t Ospi_udma_deinit(void);
static int32_t Ospi_udma_init(OSPI_v0_HwAttrs *cfg);

static int32_t Ospi_udma_init(OSPI_v0_HwAttrs *cfg)
{
    int32_t         retVal = UDMA_SOK;
    Udma_InitPrms   initPrms;
    uint32_t        instId;

    SBL_ADD_PROFILE_POINT;

    if (gDrvHandle == (Udma_DrvHandle)uint32_to_void_ptr(0U))
    {
        /* UDMA driver init */
        instId = UDMA_INST_ID_MCU_0;

        UdmaInitPrms_init(instId, &initPrms);
        retVal = Udma_init(&gUdmaDrvObj, &initPrms);
        if(UDMA_SOK == retVal)
        {
            gDrvHandle = &gUdmaDrvObj;
        }
    }

    if(gDrvHandle)
    {
        gUdmaInfo.drvHandle      = (void *)gDrvHandle;
        gUdmaInfo.chHandle       = (void *)&gUdmaChObj;
        gUdmaInfo.ringMem        = (void *)&gTxRingMem[0];
        gUdmaInfo.cqRingMem      = (void *)&gTxCompRingMem[0];
        gUdmaInfo.tdCqRingMem    = (void *)&gTxTdCompRingMem[0];
        gUdmaInfo.tprdMem        = (void *)&gUdmaTprdMem[0];
        gUdmaInfo.eventHandle    = (void *)&gUdmaCqEventObj;
        cfg->dmaInfo             = &gUdmaInfo;
    }

    SBL_ADD_PROFILE_POINT;

    return (retVal);
}

static int32_t Ospi_udma_deinit(void)
{
    int32_t         retVal = UDMA_SOK;

    SBL_ADD_PROFILE_POINT;

    if (gDrvHandle)
    {
        retVal = Udma_deinit(gDrvHandle);
        if(UDMA_SOK == retVal)
        {
            gDrvHandle = NULL;
        }
    }

    SBL_ADD_PROFILE_POINT;

    return (retVal);
}
#endif

int32_t SBL_ReadSysfwImage(void **pBuffer, uint32_t num_bytes)
{
    Board_flashHandle h;

    SBL_ADD_PROFILE_POINT;

    /* Init SPI driver */
    SPI_init();

    /* Get default OSPI cfg */
    OSPI_socGetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    ospi_cfg.funcClk = OSPI_MODULE_CLK_133M;

    /* false = SDR mode, sysfw read by rom using DMA */
    /* true  = DDR mode, sysfw read by SBL using CPU */
    ospi_cfg.dtrEnable = true;

    /* Set the default SPI init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    h = Board_flashOpen(BOARD_FLASH_ID_MT35XU512ABA1G12,
                            BOARD_OSPI_NOR_INSTANCE, NULL);

    if (h)
    {
        SBL_ADD_PROFILE_POINT;

        if (ospi_cfg.dtrEnable == true)
        {   
            /* Enable PHY pipeline mode  */
            CSL_ospiPipelinePhyEnable((const CSL_ospi_flash_cfgRegs *)(OSPI_CTLR_BASE_ADDR), TRUE);

            /* Optimized CPU copy loop SDR */
            SBL_SysFwLoad((void *)(*pBuffer), (void *)(OSPI_FLASH_BASE_ADDR + OSPI_OFFSET_SYSFW), num_bytes);
        }
        else
        {
            /* Point ROM to system firmware in OSPI flash, if SDR*/
            *pBuffer = (void *)(OSPI_FLASH_BASE_ADDR + OSPI_OFFSET_SYSFW);
        }

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

int32_t SBL_ospiInit(void *handle)
{
    Board_flashHandle h = *(Board_flashHandle *) handle;

    SBL_ADD_PROFILE_POINT;

    if (h)
    {
        Board_flashClose(h);
        SBL_ADD_PROFILE_POINT;

    }

#if !defined(SBL_SKIP_BRD_CFG_PM) && !defined(SBL_SKIP_SYSFW_INIT)
    {
        struct ospiClkParams
        {
            uint32_t moduleId;
            uint32_t clockId;
        };
        struct ospiClkParams ospiClkInfo[] = {
                                                {SBL_DEV_ID_OSPI0, SBL_CLK_ID_OSPI0},
                                                {SBL_DEV_ID_OSPI1, SBL_CLK_ID_OSPI1},
                                             };
        uint64_t ospiFunClk;

        /* System Firmware reconfigures the OSPI clock to
         * unsupported values. This is a workaround until
         * that issue is fixed
         */
        ospiFunClk = (uint64_t)(OSPI_MODULE_CLK_133M);
        Sciclient_pmSetModuleClkFreq(ospiClkInfo[BOARD_OSPI_NOR_INSTANCE].moduleId, ospiClkInfo[BOARD_OSPI_NOR_INSTANCE].clockId, ospiFunClk, TISCI_MSG_FLAG_AOP, SCICLIENT_SERVICE_WAIT_FOREVER);
        ospi_cfg.funcClk = (uint32_t)ospiFunClk;
        SBL_log(SBL_LOG_MAX, "ospiFunClk = %d Hz \n", ospi_cfg.funcClk);
    }
#endif

    ospi_cfg.dtrEnable = true;

#if SBL_USE_DMA
    ospi_cfg.dmaEnable = SBL_USE_DMA;
    Ospi_udma_init(&ospi_cfg);
#endif
    /* We set xipEnable = true only at the last open() */
    ospi_cfg.xipEnable = true;

    /* Set the default SPI init configurations */
    OSPI_socSetInitCfg(BOARD_OSPI_NOR_INSTANCE, &ospi_cfg);

    h = Board_flashOpen(BOARD_FLASH_ID_MT35XU512ABA1G12,
                            BOARD_OSPI_NOR_INSTANCE, NULL);

    if (h)
    {
        *(Board_flashHandle *) handle = h;
    }
    else
    {
        SBL_log(SBL_LOG_ERR, "Board_flashOpen failed!\n");
        SblErrLoop(__FILE__, __LINE__);
    }

    SBL_ADD_PROFILE_POINT;

    return 0;
}

int32_t SBL_ospiFlashRead(const void *handle, uint8_t *dst, uint32_t length,
    uint32_t offset)
{
    uint32_t start_time = SBL_ADD_PROFILE_POINT;
    uint32_t end_time = 0;

#if SBL_USE_DMA
    if (length > 4 * 1024)
    {
        Board_flashHandle h = *(const Board_flashHandle *) handle;
        uint32_t ioMode  = OSPI_FLASH_OCTAL_READ;
        /* split transfer if not reading from 16 byte aligned flash offset */
        uint32_t dma_offset  = (offset + 0xF) & (~0xF);
        uint32_t non_aligned_bytes = dma_offset - offset;
        uint8_t *dma_dst = (dst + non_aligned_bytes);
        uint32_t dma_len = length - non_aligned_bytes;

        SBL_DCacheClean((void *)dst, length);

        if ((non_aligned_bytes) && (Board_flashRead(h, offset, dst, non_aligned_bytes, (void *)(&ioMode))))
        {
            SBL_log(SBL_LOG_ERR, "Board_flashRead failed!\n");
            SblErrLoop(__FILE__, __LINE__);
        }

        if (Board_flashRead(h, dma_offset, dma_dst, dma_len, (void *)(&ioMode)))
        {
            SBL_log(SBL_LOG_ERR, "Board_flashRead failed!\n");
            SblErrLoop(__FILE__, __LINE__);
        }
    }
    else
    {
        memcpy((void *)dst, (void *)(OSPI_FLASH_BASE_ADDR + offset), length);
    }
#else
    memcpy((void *)dst, (void *)(OSPI_FLASH_BASE_ADDR + offset), length);
#endif

    end_time = SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MAX, "Ospi Read speed for 0x%x bytes from offset 0x%x = %d Mbytes per sec\n", length, offset, ((400000000 / (end_time-start_time)) * length)/0x100000);

    return 0;
}

int32_t SBL_ospiClose(const void *handle)
{
    Board_flashHandle h = *(const Board_flashHandle *) handle;

    SBL_ADD_PROFILE_POINT;

    SBL_log(SBL_LOG_MAX, "SBL_ospiClose called\n");
    Board_flashClose(h);
#if SBL_USE_DMA
    Ospi_udma_deinit();
#endif
    SBL_ADD_PROFILE_POINT;

    return 0;
}

int32_t SBL_OSPIBootImage(sblEntryPoint_t *pEntry)
{
    int32_t retVal;

#ifdef SECURE_BOOT
    uint32_t authenticated = 0;
    uint32_t srcAddr = 0;
    uint32_t imgOffset = 0;
#else
    uint32_t offset = OSPI_OFFSET_SI;
#endif

    /* Initialization of the driver. */
    SBL_OSPI_Initialize();

#ifndef SECURE_BOOT
    retVal =  SBL_MulticoreImageParse((void *) &offset, OSPI_OFFSET_SI, pEntry);
#else
    retVal = SBL_loadOSPIBootData();

    if (retVal == E_PASS)
    {
        /* authentiate it */
        authenticated = SBL_authentication(sblInBootData.sbl_boot_buff);
        if (authenticated == 0)
        {
            /* fails authentiation */
            SBL_log(SBL_LOG_ERR, "\n OSPI Boot - fail authentication\n");

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
        SBL_log(SBL_LOG_ERR, "\n OSPI Boot - problem processing image \n");
    }

#endif

    SBL_ospiClose(&boardHandle);

    return retVal;
}

static void SBL_OSPI_Initialize(void)
{
    SBL_ospiInit(&boardHandle);

    /* Initialize the function pointers to parse through the RPRC format. */

#ifndef SECURE_BOOT
    fp_readData = &SBL_OSPI_ReadSectors;
    fp_seek     = &SBL_OSPI_seek;
#else
    fp_readData = &SBL_MemRead;
    fp_seek     = &SBL_MemSeek;
#endif

}


#ifndef SECURE_BOOT

static int32_t SBL_OSPI_ReadSectors(void *dstAddr,
                             void *srcOffsetAddr,
                             uint32_t length)
{
    int32_t ret;
    ret = SBL_ospiFlashRead(&boardHandle, (uint8_t *) dstAddr, length,
        *((uint32_t *) srcOffsetAddr));
    *((uint32_t *) srcOffsetAddr) += length;
    return ret;
}

static void SBL_OSPI_seek(void *srcAddr, uint32_t location)
{
    *((uint32_t *) srcAddr) = location;
}

#else

/* load signed boot data from OSPI */
int32_t SBL_loadOSPIBootData()
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
    retVal = SBL_ospiFlashRead(&boardHandle, sblInBootData.sbl_boot_buff,
                               READ_BUFF_SIZE, OSPI_OFFSET_SI);

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
                    retVal = SBL_ospiFlashRead(&boardHandle,
                                            sblInBootData.sbl_boot_buff,
                                            total_size,
                                            OSPI_OFFSET_SI);
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
        /* Fail to read the OSPI flash */
        retVal = E_FAIL;
    }


    return retVal;
}
#endif


