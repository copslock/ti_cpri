/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*============================================================================*/
/*============================================================================*/

/**
 *  \file dmautils_autoincrement_test.c
 *
 *  \brief Simple application demonstrating 2D auto increment feature of dmautils along with handling 
 *        of last block.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2643:PDK-2649:PDK-2646)
 */
 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#if defined(HOST_EMULATION)
#include <malloc.h>
#endif

#include "dmautils_autoincrement_example.h"
#include "ti/drv/sciclient/sciclient.h"

#define TEST_malloc(heap, size)   malloc(size)
#define TEST_free(ptr)            free(ptr)

#define L2SRAM_SIZE (64*1024)

#ifdef _MSC_VER
#ifndef __attribute__
#define __attribute__()
#endif
#endif
uint8_t L2SRAM[L2SRAM_SIZE] __attribute__((aligned(128)));


typedef struct
{
  uint32_t testcaseId;
  uint32_t requirementId;
  uint32_t imageWidth;
  uint32_t imageHeight;
  uint32_t blockWidth;
  uint32_t blockHeight;
}dmautilsAutoIncTest_config;


dmautilsAutoIncTest_config gTestConfig[] =
{
    {
        0,
        1,
        40,/*Image Width */
        16,/*Image Height */
        8,/*Image blockWidth */
        8/*Image blockHeight */
    },

    {
          1,
          1,
          44,/*Image Width */
          16,/*Image Height */
          8,/*Image blockWidth */
          8/*Image blockHeight */
     },
     {
          2,
          1,
          44,/*Image Width */
          35,/*Image Height */
          8,/*Image blockWidth */
          8/*Image blockHeight */
     },

     {
          3,
          1,
          127,/*Image Width */
          16,/*Image Height */
          16,/*Image blockWidth */
          8/*Image blockHeight */
     }

};


int32_t test_sciclientDmscGetVersion(char *version_str, uint32_t version_str_size)
{
    int32_t retVal = 0;

    const Sciclient_ReqPrm_t      reqPrm =
    {
        TISCI_MSG_VERSION,
        TISCI_MSG_FLAG_AOP,
        NULL,
        0,
        SCICLIENT_SERVICE_WAIT_FOREVER
    };
    struct tisci_msg_version_resp response;
    Sciclient_RespPrm_t           respPrm =
    {
        0,
        (uint8_t *) &response,
        sizeof (response)
    };

    retVal = Sciclient_service(&reqPrm, &respPrm);
    if (0 == retVal)
    {
        if (respPrm.flags == TISCI_MSG_FLAG_ACK)
        {
            if(version_str == NULL)
            {
                printf("SCICLIENT: DMSC FW version [%s]\n", (char *) response.str);
                printf("SCICLIENT: DMSC FW revision 0x%x  \n", response.version);
                printf("SCICLIENT: DMSC FW ABI revision %d.%d\n",
                    response.abi_major, response.abi_minor);
            }
            else
            {
                snprintf(version_str, version_str_size, "version %s, revision 0x%x, ABI %d.%d",
                    (char *) response.str,
                    response.version,
                    response.abi_major, response.abi_minor
                    );
            }
        }
        else
        {
            retVal = -1;
        }
    }
    if(retVal!=0)
    {
        printf("SCICLIENT: ERROR: DMSC Firmware Get Version failed !!!\n");
    }

    return (retVal);
}

int32_t main()
{
  uint16_t   width;
  uint16_t   height;
  uint16_t   inPitch;
  uint16_t   outPitch;
  uint16_t   blockWidth;
  uint16_t   blockHeight;

  int32_t i, j;
  uint8_t *input     = NULL;
  uint8_t  *output    = NULL;
  uint8_t  *refOut    = NULL;

  uint8_t * pInputBlock;
  uint8_t * pOutputBlock;

  uint8_t*    pIntMmeBase  = L2SRAM;
  uint32_t   intMemSize   = L2SRAM_SIZE;
  uint8_t    useDMA      ;
  uint8_t status = 1;
  uint32_t testcaseIdx;

#ifdef HOST_EMULATION
#if defined(_MSC_VER)
    pIntMmeBase = (uint8_t*)_aligned_malloc(L2SRAM_SIZE, L2SRAM_SIZE);
#else
    pIntMmeBase = (uint8_t*)memalign(L2SRAM_SIZE, L2SRAM_SIZE);
#endif
#else
    int32_t retVal = 0;
    Sciclient_ConfigPrms_t  sciClientCfg;
    Sciclient_configPrmsInit(&sciClientCfg);
    sciClientCfg.isSecureMode = 1;
    retVal = Sciclient_init(&sciClientCfg);
    if(retVal!=0)
    {
      printf("Sciclient Init Failed \n");
      goto Exit;
    }

    test_sciclientDmscGetVersion(NULL, 0 ); 
#endif

  for (testcaseIdx = 0; testcaseIdx < sizeof(gTestConfig)/ sizeof(dmautilsAutoIncTest_config); testcaseIdx++)
  {
      width    = gTestConfig[testcaseIdx].imageWidth;
      height   = gTestConfig[testcaseIdx].imageHeight;
printf("wi %d \n", width);
      printf("height %d \n", height);

      inPitch  = gTestConfig[testcaseIdx].imageWidth;
      outPitch = gTestConfig[testcaseIdx].imageWidth;
      blockWidth = gTestConfig[testcaseIdx].blockWidth;
      blockHeight  = gTestConfig[testcaseIdx].blockHeight;

      /* Buffer allocations for input, output and reference output  */

      input = (uint8_t *)malloc(width * height);
      output = (uint8_t *)malloc(width * height);
      refOut = (uint8_t *)malloc(width * height);

      pInputBlock = (uint8_t *)malloc(blockWidth * blockHeight * 2);
      pOutputBlock = (uint8_t *)malloc(blockWidth * blockHeight * 2);

      memset(output, 0, width * height);
      memset(refOut, 0, width * height);

      /* Random pattern generator for input  */
      for ( j = 0 ; j < height; j++)
      {
        for (i = 0; i < width; i++)
        {
          input[i + j * inPitch] = i + j* 56;
        }
      }

      //DMA based function call
      useDMA = 1;

#if (!HOST_EMULATION) && (CORE_DSP)
      tscStart = _TSC_read();
#endif

      blockCopy(
        input,
        output,
        pInputBlock,
        pOutputBlock,
        width,
        height,
        blockWidth,
        blockHeight,
        inPitch,
        outPitch,
        pIntMmeBase,
        intMemSize,
        useDMA );

#if (!HOST_EMULATION) && (CORE_DSP)
      tscEnd = _TSC_read();
      printf("Cycles - Using DMA = %llu\n",(tscEnd-tscStart));
#endif

      useDMA = 0;
#if (!HOST_EMULATION) && (CORE_DSP)
      tscStart = _TSC_read();
#endif
      blockCopy(
        input,
        refOut,
        pInputBlock,
        pOutputBlock,
        width,
        height,
        blockWidth,
        blockHeight,
        inPitch,
        outPitch,
        pIntMmeBase,
        intMemSize,
        useDMA );
#if (!HOST_EMULATION) && (CORE_DSP)
      tscEnd = _TSC_read();
      printf("Cycles - Without using DMA = %llu\n",(tscEnd-tscStart));
#endif

      /*Compare output with reference output */
      for(j = 0; j < height; j++)
      {
        for(i = 0; i < width; i++)
        {
          if(output[j * outPitch + i] != refOut[j * outPitch + i])
          {
            status = 0;
            printf("[%d][%d] - output = %d\trefOutput = %d\n",j,i,output[j*outPitch + i], refOut[j*outPitch + i]);
            break;
          }
          if ( status == 0 )
          {
             break;
          }
        }
      }
      if(status == 1)
      {
        printf("DMAUtils TestCase %d,        PASSED \n", gTestConfig[testcaseIdx].testcaseId);
      }
      else
      {
        printf("\nDMAUtils TestCase %d,        FAILED!!!!!! \n", gTestConfig[testcaseIdx].testcaseId);
      }

      free(input);
      free(output);
      free(refOut);
      free(pInputBlock);
      free(pOutputBlock);
  }

#ifdef HOST_EMULATION
#if defined(_MSC_VER)
      _aligned_free(pIntMmeBase);
#else
      free(pIntMmeBase);
#endif
#endif
Exit:
  return 0;
}

