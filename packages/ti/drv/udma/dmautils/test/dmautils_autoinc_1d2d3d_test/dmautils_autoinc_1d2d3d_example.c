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
----------------------------------------------------------------------------
@file    dma_example.c
@brief   Demostrates a simple example of auto increment DMA to allow DSP to
operate a function on internal memory and transfer back the result.
@version 0.0 (Jan 2017) : First version
----------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdint.h>

#include "dmautils_autoinc_1d2d3d_example.h"

#include "ti/drv/udma/dmautils/dmautils.h"
#include "ti/drv/udma/udma.h"


int32_t blockCopyKernel(
  uint8_t *inputData,
  uint8_t  *outputData,
  uint16_t width,
  uint16_t height,
  uint16_t inPitch,
  uint16_t outPitch)
{
  int32_t i, j;

  for(j = 0; j < height; j++)
  {
    for(i = 0; i < width; i++)
    {
      outputData[i + (j * outPitch)] =
           inputData[i + (j * inPitch)] ;
    }
  }

  return 0;
}

static void testDmaAutoIncPrintf(const char *str)
{
#ifdef ENABLE_PRINT
  print(str);
#endif
}

#define TEST_DMAUTILS_ALIGN_CEIL(VAL, ALIGN) ((((VAL) + (ALIGN) - 1)/(ALIGN)) * (ALIGN) )


typedef enum{
  DMAUTILSTESTAUTOINC_CHANNEL_IN,
  DMAUTILSTESTAUTOINC_CHANNEL_OUT,
  DMAUTILSTESTAUTOINC_CHANNEL_MAX
}dmautilsTestAutoInc_Channel;

#define TEST_ALIGN_SIZE (128U)


static void testDmaAutoIncSetupXferProp
(
  int16_t   width,
  int16_t   height,
  int16_t   blockWidth,
  int16_t   blockHeight,
  int16_t   inPitch,
  int16_t   outPitch,
  DmaUtilsAutoInc3d_TransferDim *transferDimIn
)
{
  transferDimIn->sicnt0 = blockWidth;
  transferDimIn->sicnt1 = blockHeight;
  transferDimIn->sicnt2 = width/blockWidth;
  transferDimIn->sicnt3 = height/blockHeight;
  transferDimIn->sdim1 = inPitch;
  transferDimIn->sdim2 = blockWidth;
  transferDimIn->sdim3 = blockHeight * inPitch;

  transferDimIn->dicnt0 = blockWidth;
  transferDimIn->dicnt1 = blockHeight;
  transferDimIn->dicnt2 = width/blockWidth;
  transferDimIn->dicnt3 = height/blockHeight;
  transferDimIn->ddim1 = outPitch;
  transferDimIn->ddim2 = blockWidth;
  transferDimIn->ddim3 =  outPitch * blockHeight;

}

/* This function is main function exposed to user*/
int32_t blockCopy(
  uint8_t*   pInput,
  uint8_t*   pOutput,
  uint8_t*   pInputBlock,
  uint8_t*   pOutputBlock,
  uint16_t   width,
  uint16_t   height,
  uint16_t   blockWidth,
  uint16_t   blockHeight,
  uint16_t   inPitch,
  uint16_t   outPitch,
  uint32_t   transferSize,
  uint8_t*   pIntMmeBase,
  uint32_t   intMemSize,
  uint8_t    useDMA
  )
{
  int32_t retVal = UDMA_SOK ;

  if(useDMA == 0)
  {
    //call the kernel directly on data in DDR
    blockCopyKernel(pInput,
      pOutput,
      width,
      height,
      inPitch,
      outPitch);
  }
  else
  {
    uint32_t intMemUsedSize = 0;
    uint8_t *dmautilsContext;
    uint8_t *inTrMem;
    uint32_t blockIdx = 0;
    uint32_t inTrSize;
    uint32_t dmaChannels;

    DmaUtilsAutoInc3d_InitParam initParams;
    DmaUtilsAutoInc3d_ChannelInitParam chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_MAX];
    DmaUtilsAutoInc3d_TrPrepareParam trPrepParamIn;
    DmaUtilsAutoInc3d_TransferProp *transferPropIn;


    Udma_InitPrms   initPrms;
    struct Udma_DrvObj      udmaDrvObj;
    uint32_t        instId;

    Udma_DrvHandle  drvHandle = &udmaDrvObj;

    instId = UDMA_INST_ID_MAIN_0;
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = &testDmaAutoIncPrintf;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        testDmaAutoIncPrintf("[Error] UDMA init failed!!\n");
    }


    dmaChannels = 1U; /* One for input and other for output */

    //Allocation/Assignment of buffers in internal memory
    dmautilsContext     =  pIntMmeBase + intMemUsedSize ;
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL(DmaUtilsAutoInc3d_getContextSize(dmaChannels), TEST_ALIGN_SIZE);

    transferPropIn = (DmaUtilsAutoInc3d_TransferProp * ) (pIntMmeBase + intMemUsedSize );
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL((sizeof(DmaUtilsAutoInc3d_TransferProp)), TEST_ALIGN_SIZE);

    initParams.contextSize = DmaUtilsAutoInc3d_getContextSize(dmaChannels);
    initParams.numChannels = dmaChannels;
    initParams.traceLogLevel    = 1;
    initParams.udmaDrvHandle = drvHandle;
    initParams.DmaUtilsVprintf = vprintf;

    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_IN].dmaQueNo  = 0;
    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_IN].druOwner    = DMAUTILSAUTOINC3D_DRUOWNER_DIRECT_TR;

    retVal = DmaUtilsAutoInc3d_init(dmautilsContext, &initParams, chInitParams);
    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }
    inTrSize = DmaUtilsAutoInc3d_getTrMemReq(1);
    inTrMem = pIntMmeBase + intMemUsedSize ;
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL(inTrSize, TEST_ALIGN_SIZE);

    if(intMemUsedSize > intMemSize)
    {
      printf("insufficient memory, required is %d vs provided %d\n",intMemUsedSize, intMemSize);
      return -1 ;
    }

    testDmaAutoIncSetupXferProp(width,
                                                       height,
                                                       blockWidth,
                                                       blockHeight,
                                                       inPitch,
                                                       outPitch,
                                                       &transferPropIn->transferDim);

    trPrepParamIn.channelId = DMAUTILSTESTAUTOINC_CHANNEL_IN;
    trPrepParamIn.numTRs  = 1;
    trPrepParamIn.trMem     = inTrMem;
    trPrepParamIn.trMemSize = inTrSize;

    transferPropIn->circProp.circDir = DMAUTILSAUTOINC3D_CIRCDIR_SRC;
    transferPropIn->circProp.circSize1 = 0;
    transferPropIn->circProp.circSize2 = 0;
    transferPropIn->circProp.addrModeIcnt0 = DMAUTILSAUTOINC3D_ADDR_LINEAR;
    transferPropIn->circProp.addrModeIcnt0 = DMAUTILSAUTOINC3D_ADDR_LINEAR;
    transferPropIn->circProp.addrModeIcnt0 = DMAUTILSAUTOINC3D_ADDR_LINEAR;
    transferPropIn->circProp.addrModeIcnt0 = DMAUTILSAUTOINC3D_ADDR_LINEAR;

    transferPropIn->syncType = transferSize;
    transferPropIn->ioPointers.srcPtr = pInput;
    transferPropIn->ioPointers.dstPtr = pOutput;

    retVal = DmaUtilsAutoInc3d_prepareTr(&trPrepParamIn, &transferPropIn[0]);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }


    retVal = DmaUtilsAutoInc3d_configure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN, inTrMem, 1);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }

    while ( 1 )
    {
      //DMA trigger for pipe-up, out transfer is dummy and handled inside DMA utility
      blockIdx = DmaUtilsAutoInc3d_trigger(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);
      //Wait for previous transfer of in
      DmaUtilsAutoInc3d_wait(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);

      if ( blockIdx == 0 )
      {
        break;
      }
    }


    retVal = DmaUtilsAutoInc3d_deconfigure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN, inTrMem, 1);

     if ( retVal != UDMA_SOK )
     {
       goto Exit;
     }


    retVal = DmaUtilsAutoInc3d_deinit(dmautilsContext);

     if ( retVal != UDMA_SOK )
     {
       goto Exit;
     }

    retVal = Udma_deinit(drvHandle);
    if ( retVal != UDMA_SOK )
     {
       goto Exit;
     }
  }


Exit:
  return retVal ;
}
