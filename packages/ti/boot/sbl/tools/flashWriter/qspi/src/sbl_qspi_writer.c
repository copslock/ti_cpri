/**
 *  \file    sbl_qspi.c
 *
 *  \brief   This file contains functions for QSPI read/write operations for SBL
 *
 */

/*
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/src/UART_osal.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/tistdtypes.h>

#include <ti/drv/mmcsd/MMCSD.h>
#include <ti/drv/mmcsd/soc/MMCSD_v1.h>
#include <ti/drv/mmcsd/src/MMCSD_osal.h>

#include <ti/fs/fatfs/ff.h>
#include <ti/fs/fatfs/FATFS.h>

/* TI-RTOS Header files */
#include <ti/drv/spi/SPI.h>
#include <ti/drv/spi/src/SPI_osal.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/drv/spi/test/qspi_flash/src/QSPI_board.h>
#include <ti/drv/spi/test/qspi_flash/src/SPI_log.h>

#include <ti/board/src/flash/include/board_flash.h>

#include "sbl_soc.h"

/* The iceK2G board has SD card on instance 1 instead of 0 */
#ifdef iceK2G
#define MMCSD_INSTANCE_SDCARD 1U
#else
#define MMCSD_INSTANCE_SDCARD 0U
#endif

#define S_PASS                     (0U)

#define E_FAIL                     (-1U)

/** \brief Maximum buffer size. */
#define MMCSD_DATA_BUF_SIZE                 (512U)


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

FATFS_Handle fatfsHandle = NULL;

/* MMCSD function table for MMCSD implementation */
FATFS_DrvFxnTable FATFS_drvFxnTable = {
    MMCSD_close,
    MMCSD_control,
    MMCSD_init,
    MMCSD_open,
    MMCSD_write,
    MMCSD_read
};

/* FATFS configuration structure */
FATFS_HwAttrs FATFS_initCfg[_VOLUMES] =
{
    {
        MMCSD_INSTANCE_SDCARD
    },
    {
        1U
    },
    {
        2U
    },
    {
        3U
    }
};

/* FATFS objects */
FATFS_Object FATFS_objects[_VOLUMES];

/* FATFS configuration structure */
const FATFS_Config FATFS_config[_VOLUMES + 1] = {
    {
        &FATFS_drvFxnTable,
        &FATFS_objects[0],
        &FATFS_initCfg[0]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[1],
         &FATFS_initCfg[1]
    },

    {
         &FATFS_drvFxnTable,
         &FATFS_objects[2],
         &FATFS_initCfg[2]
    },

    {NULL, NULL, NULL},

    {NULL, NULL, NULL}
};

/**
 * \brief Temporary data buffer to be used while copying data
 *        from SD card to DDR.
 */
static char gTmpBuf[MMCSD_DATA_BUF_SIZE];

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/**
 * \brief This API copies the flash image from MMCSD to DDR
 *
 * \param fileName Name of the file to be copied.
 * \param destAddr Address where the file is to be copied
 * \param fileSize Total size of the file read
 *
 * \retval status S_PASS Image copy passed
 * \retval status E_FAIL Image copy failed
 */
static int32_t MmcsdImageCopy(char * fileName,
                               uint32_t destAddr,
                               uint32_t * fileSize);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

uint32_t checkFlash(uint32_t x, uint32_t y, uint32_t size)
{
    uint32_t a = 0, b = 0;
    uint32_t i = 0;
    uint32_t ret = 0;
    for (i = 0; i<size; i+=4)
    {
        a = *(uint32_t *) (x+i);
        b = *(uint32_t *) (y+i);
        if (a != b)
        {
            ret++;
            UART_printf("Data mismatch at 0x%08x, wrote 0x%08x, read 0x%08x\n", (x+i), a, b);
        }
    }
    return ret;
}

int main(void)
{
    FRESULT  fresult;
    FIL  fileObject;

    /* File name which need to be copied from mmcsd to QSPI */
    char fileName[9U];
    /* Offset address where the flash image is to be written */
    uint32_t offsetAddr = 0U;
    /* DDR address where image has to be copied from mmcsd card.
     * The image is then copied from this address to QSPI flash
     */
    uint32_t destAddr = 0x82000000U;
    uint32_t checkAddr = 0x83000000U;
    uint32_t length = 0U;
    int32_t status = E_FAIL;
    void *boardHandle;
    int32_t ret;

    if (SBL_socInit())
    {
        UART_printf("Error initialzing soc!\n");
        return -1;
    }
    else
    {
        UART_printf("\n*** PDK QSPI Flash Writer ***\n");
    }

    if (SBL_qspiInit(&boardHandle))
    {
        UART_printf("BOARD_flashOpen failed. \n");
        return -1;
    }

#ifdef iceK2G
    MMCSD_v1_HwAttrs hwAttrsConfig;
    MMCSD_socGetInitCfg(MMCSD_INSTANCE_SDCARD, &hwAttrsConfig);
    hwAttrsConfig.cardType=MMCSD_CARD_SD;
    hwAttrsConfig.supportedBusWidth= (MMCSD_BUS_WIDTH_1BIT | MMCSD_BUS_WIDTH_4BIT);
    hwAttrsConfig.enableInterrupt = 0;
    MMCSD_socSetInitCfg(MMCSD_INSTANCE_SDCARD, &hwAttrsConfig);
#endif

    /* Initialization of the driver. */
    fresult = FATFS_init();

    /* MMCSD FATFS initialization */
    fresult = FATFS_open(0U, NULL, fatfsHandle);

    if (FR_OK == fresult)
    {
        /* Open the config file and read the file name and parameters */
        fresult = f_open(&fileObject, "config", FA_READ);

        if(FR_OK != fresult)
        {
            /* If there was some problem opening the file, then return an error. */
            UART_printf("\r\n Unable to open config file \r\n");
        }
        else
        {
            while(0U != f_gets(gTmpBuf, sizeof(gTmpBuf), &fileObject))
            {
                if ( -1 != sscanf(gTmpBuf,"%8s %x", fileName, &offsetAddr))
                {
                    status = S_PASS;
                }
                else
                {
                    status = E_FAIL;
                    UART_printf("\nSorry! Something wrong in config file\n");
                    UART_printf("\nPlease check if file is in format shown below\n");
                    UART_printf("\nboot 0\napp 80000\n");
                }
                if (S_PASS == status)
                {
                    length = 0U;
                    status = MmcsdImageCopy(fileName,
                                            destAddr,
                                            &length);
                }
                if (S_PASS == status)
                {
                    UART_printf("Begin flashing '%s' into QSPI\n", fileName);
                    if ((ret = SBL_qspiFlashWrite(&boardHandle, (uint8_t *) destAddr, length, offsetAddr)))
                    {
                        UART_printf("SBL_qspiFlash: received ERROR %d\n", ret);
                        return -1;
                    }
                    else
                    {
                        UART_printf("Finished flashing '%s' with size %x at offset %x\n", fileName, length, offsetAddr);
                    }

                    UART_printf("Reading and verifying flashed image\n");
                    if ((ret = SBL_qspiFlashRead(&boardHandle, (uint8_t *) checkAddr, length, offsetAddr)))
                    {
                        UART_printf("SBL_qspiFlash: received ERROR %d\n", ret);
                    }
                    else
                    {
                        UART_printf("Finished checking flashed image! \n");
                    }
                    ret = checkFlash(destAddr, checkAddr, length);
                }
                else
                {
                    UART_printf("ERROR: Unable to load '%s' into local memory!\n", fileName);
                    return -1;
                }
            }
             /* Close the file here */
             f_close(&fileObject);
        }

        UART_printf("Flashing completed! \n");
    }
    SBL_qspiClose(&boardHandle);

    return (S_PASS);
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

static int32_t MmcsdImageCopy(char * fileName,
                              uint32_t destAddr,
                              uint32_t * fileSize)
{
    FRESULT fResult;
    static FIL fileObject;
    uint32_t bytesRead = 0U;
    int32_t status = E_FAIL;
    /* Open the file for reading */
    fResult = f_open(&fileObject, fileName, FA_READ);

    if(FR_OK == fResult)
    {
        status = S_PASS;
    }
    /* If there was some problem opening the file, then return an error. */
    else
    {
        UART_printf("\r\n Unable to open file %s \r\n", fileName);
    }

    if(S_PASS == status)
    {
        /*
         * Enter a loop to repeatedly read data from the file and display it, until
         * the end of the file is reached.
         */
        UART_printf("Copying '%s' to local memory\n", fileName);
        do
        {
            /*
             * Read a block of data from the file.  Read as much as can fit in the
             * temporary buffer, including a space for the trailing null.
             */
            fResult = f_read(&fileObject, gTmpBuf, sizeof(gTmpBuf) - 1,
                             (uint32_t *) &bytesRead);

            /*
             * If there was an error reading, then print a newline and return the
             * error to the user.
             */
            if(fResult != FR_OK)
            {
                UART_printf("\r\n Error reading application file\r\n");
                status = E_FAIL;
            }

            if(bytesRead >= sizeof(gTmpBuf))
            {
                status = E_FAIL;
            }

            /* Read the last chunk of the file that was received. */
            memcpy((uint32_t *)destAddr, gTmpBuf, (sizeof(gTmpBuf) - 1));
            destAddr += (sizeof(gTmpBuf) - 1);
            /*
             * Continue reading until less than the full number of bytes are read.
             * That means the end of the buffer was reached.
             */
            *(fileSize) += bytesRead;
        }
        while((bytesRead == sizeof(gTmpBuf) - 1) && (S_PASS == status));
        /* Close the file. */
        fResult = f_close(&fileObject);
        if (E_FAIL == status)
        {
            UART_printf(" Read failed for %s\n",fileName);
        }
    }
    return status;
}
