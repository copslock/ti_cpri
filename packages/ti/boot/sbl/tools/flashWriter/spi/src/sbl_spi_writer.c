/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* TI-RTOS Header files */
#include <ti/drv/uart/UART_stdio.h>
#include <ti/csl/tistdtypes.h>

/* TI-RTOS Header files */
#include <ti/drv/spi/SPI.h>
/* SPI v0 for Keystone devices */
#include <ti/drv/spi/src/v0/SPI_v0.h>
#include <ti/drv/spi/soc/SPI_soc.h>
#include <ti/board/src/flash/include/board_flash.h>

#include "board_cfg.h"
#include "sbl_soc.h"

/* Name of config file to parse */
#define SPI_CONFIG_FILE_NAME "config"

/* Maximum length of each config line */
#define MAX_LINE_LENGTH 128

#ifdef SOC_OMAPL137
/* Temporary memory to hold binary to flash. Default is base of DDR */
#define LOAD_BUFFER_ADDR 0xC0000000

/* Temporary memory to read binary from flash. */
#define READ_BUFFER_ADDR 0xC1000000

#else
/* Temporary memory to hold binary to flash. Default is base of DDR */
#define LOAD_BUFFER_ADDR 0x80000000

/* Temporary memory to read binary from flash. */
#define READ_BUFFER_ADDR 0x90000000

#endif

/* Buffer pointer to load temporary memory */
uint32_t *buf_ptr;

int32_t FW_parseInputFile(const char *s, void *spiHandle)
{
    FILE *fp;
    FILE *binPtr;
    char line[MAX_LINE_LENGTH];
    char fileName[20];
    int32_t offsetAddr = 0U;
    uint32_t len, i;
    int32_t ret;

    buf_ptr = (uint32_t *) LOAD_BUFFER_ADDR;

    fp = fopen(s, "r");
    if (fp == NULL)
    {
        UART_printf("Error opening file %s\n", s);
        return -1;
    }

    memset(line, 0, MAX_LINE_LENGTH);

    while (fgets(line, MAX_LINE_LENGTH, fp) != 0)
    {
        if (sscanf(line,"%s %x", fileName, &offsetAddr) < 0)
        {
            UART_printf("Error parsing config line -\n");
            UART_printf("\t Make sure each line is in the format: [filename] [address]\n");
            fclose(fp);
            return -2;
        }
        else
        {
            UART_printf("Parsed config line, received parameters: filename = %s, address = 0x%x\n", fileName, offsetAddr);
            binPtr = fopen(fileName, "rb");
            if (binPtr == NULL)
            {
                UART_printf("\tUnable to open file to load: %s\n", fileName);
                fclose(fp);
                return -3;
            }
            fseek(binPtr, 0, SEEK_END);
            len = ftell(binPtr);
            fseek(binPtr, 0, SEEK_SET);
            if (len == 0)
            {
                UART_printf("\tUnable to read size of file %s\n", fileName);
                fclose(binPtr);
                fclose(fp);
                return -4;
            }
            else
            {
                UART_printf("\tSize of %s is 0x%x\n", fileName, len);
            }
            UART_printf("\tLoading binary to memory ...\n");
            fread(buf_ptr, len, 1, binPtr);
            UART_printf("\tFinished loading binary to memory!\n");
            if (ret = SBL_spiFlashWrite(spiHandle, (uint8_t *) LOAD_BUFFER_ADDR, len, offsetAddr))
            {
                UART_printf("\tError flashing memory! Error code %d\n", ret);
                return -5;
            }
            else
            {
                UART_printf("\tFlashed %s to offset 0x%x!\n", fileName, offsetAddr);
            }
            if (ret = SBL_spiFlashRead(spiHandle, (uint8_t *) READ_BUFFER_ADDR, len, offsetAddr))
            {
                UART_printf("\tError reading memory at addr 0x%x\n", offsetAddr);
                return -6;
            }
            else
            {
                UART_printf("\tRead flash memory at 0x%x, checking flashed content...\n", offsetAddr);
            }
            for (i = 0; i<len; i+=4)
            {
                if ( (*(uint32_t *) (LOAD_BUFFER_ADDR + i)) != (*(uint32_t *) (READ_BUFFER_ADDR + i)) )
                {
                    UART_printf("\t\tMismatched data at offset 0x%x, expected = 0x%08x, read = 0x%08x\n",
                        i, (*(uint32_t *) (LOAD_BUFFER_ADDR + i)), (*(uint32_t *) (READ_BUFFER_ADDR + i)));
                    ret = -7;
                }
            }
            if (ret == -7)
            {
                UART_printf("\tVerifying flashed data failed!\n");
                //return ret;
            }
            else
            {
                UART_printf("\tVerified flash data equal expected data!\n");
            }
            fclose(binPtr);

        }
    }
    fclose(fp);

    return 0;
}

int main(void)
{
    void *boardHandle;

    if (SBL_socInit())
    {
        return -1;
    }
    else
    {
        UART_printf("\n*** PDK SPI Flash Writer ***\n");
    }

    UART_printf("Opening SPI handle...\n");
    if (SBL_spiInit(&boardHandle) < 0)
    {
        UART_printf("\tSPI init failed! \n");
        return -1;
    }
    UART_printf("SPI handle opened!\n");

    UART_printf("Parsing config file and flashing content to SPI NOR...\n");
    if (FW_parseInputFile(SPI_CONFIG_FILE_NAME, &boardHandle) < 0)
    {
        UART_printf("Error parsing config file!\n");
        return -1;
    }
    else
    {
        UART_printf("Successfully flashed memory content!\n");
    }

    SBL_spiClose(&boardHandle);

    return 0;
}
