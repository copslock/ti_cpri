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

#include "dmautils_autoinc_circular_example.h"

#include "ti/drv/udma/dmautils/dmautils.h"
#include "ti/drv/udma/udma.h"


int32_t flipHorizontalKernel(
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
      outputData[i] =  inputData[width - i - 1] ;
    }
    inputData  += inPitch ;
    outputData += outPitch ;
  }
  return 0;
}


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

static int32_t testDmaAutoIncNumTrRequired(  int16_t   width,
  int16_t   height,
  int16_t   blockWidth,
  int16_t   blockHeight,
  uint32_t  *numHorzTrsRequired,
  uint32_t  *numVertTrRowsRequired
)
{
    uint32_t numHorzTr;
    uint32_t numVertTr;

    numVertTr = TEST_DMAUTILS_ALIGN_CEIL(height, blockHeight) / blockHeight;
    numHorzTr = 1;

    if ( width % blockWidth != 0 )
    {
      numHorzTr = 2;
    }

    if (numHorzTr == 1)
    {
      if ( height % blockHeight == 0 )
      {
        numVertTr = 1;
      }
      else
      {
        /* One TR to handle upper part and one TR to handle last block row */
        numVertTr = 2;
      }
    }

    if ( numHorzTrsRequired != NULL )
    {
      *numHorzTrsRequired = numHorzTr;
    }

    if ( numVertTrRowsRequired != NULL )
    {
      *numVertTrRowsRequired = numVertTr;
    }

    return ( (numHorzTr) * (numVertTr));
}

static int32_t testDmaAutoIncSetupTr(  int16_t   width,
  int16_t   height,
  int16_t   blockWidth,
  int16_t   blockHeight,
  int16_t   inPitch,
  int16_t   outPitch,
  uint8_t * pInput,
  uint8_t * pInputBlock,
  uint8_t * pOutput,
  uint8_t * pOutputBlock,
  uint32_t circularPitch,
  DmaUtilsAutoInc3d_TransferProp transferPropIn[],
  DmaUtilsAutoInc3d_TransferProp transferPropOut[]
)
{
    uint32_t numHorzTrsRequired, numVertTrRowsRequired;

    testDmaAutoIncNumTrRequired(width, height, blockWidth, blockHeight, &numHorzTrsRequired, &numVertTrRowsRequired);

    transferPropIn[0].syncType = DMAUTILSAUTOINC3D_SYNC_2D;

    transferPropIn[0].circProp.circDir = DMAUTILSAUTOINC3D_CIRCDIR_DST;
    transferPropIn[0].circProp.circSize1 = circularPitch;
    transferPropIn[0].circProp.circSize2 = 0;
    transferPropIn[0].circProp.addrModeIcnt0 = DMAUTILSAUTOINC3D_ADDR_CIRC1;
    transferPropIn[0].circProp.addrModeIcnt1 = DMAUTILSAUTOINC3D_ADDR_LINEAR;
    transferPropIn[0].circProp.addrModeIcnt2 = DMAUTILSAUTOINC3D_ADDR_CIRC1;
    transferPropIn[0].circProp.addrModeIcnt3 = DMAUTILSAUTOINC3D_ADDR_LINEAR;


    transferPropIn[0].ioPointers.srcPtr = pInput;
    transferPropIn[0].ioPointers.dstPtr = pInputBlock;

    transferPropIn[0].transferDim.sicnt0 = blockWidth;
    transferPropIn[0].transferDim.sicnt1 = blockHeight;
    transferPropIn[0].transferDim.sicnt2 = (width/blockWidth) ;
    transferPropIn[0].transferDim.sicnt3 = height / blockHeight;

    transferPropIn[0].transferDim.sdim1 = inPitch;
    transferPropIn[0].transferDim.sdim2 = blockWidth;
    transferPropIn[0].transferDim.sdim3 = blockHeight * inPitch;

    transferPropIn[0].transferDim.dicnt0 = blockWidth;
    transferPropIn[0].transferDim.dicnt1 = blockHeight;
    transferPropIn[0].transferDim.dicnt2 = (width/blockWidth) * (height / blockHeight);
    transferPropIn[0].transferDim.dicnt3 = 1;
    transferPropIn[0].transferDim.ddim1 = circularPitch;
    transferPropIn[0].transferDim.ddim2 = blockWidth;
    transferPropIn[0].transferDim.ddim3 = 0;

    transferPropOut[0] = transferPropIn[0];

    transferPropOut[0].circProp.circDir = DMAUTILSAUTOINC3D_CIRCDIR_SRC;

    transferPropOut[0].ioPointers.srcPtr = (uint8_t *)(pOutputBlock);
    transferPropOut[0].ioPointers.dstPtr = (uint8_t *)pOutput;

    transferPropOut[0].transferDim.sicnt0 = transferPropIn[0].transferDim.dicnt0;
    transferPropOut[0].transferDim.sicnt1 = transferPropIn[0].transferDim.dicnt1;
    transferPropOut[0].transferDim.sicnt2 = transferPropIn[0].transferDim.dicnt2;
    transferPropOut[0].transferDim.sicnt3 = transferPropIn[0].transferDim.dicnt3 ;
    transferPropOut[0].transferDim.sdim1 = circularPitch;
    transferPropOut[0].transferDim.sdim2 = blockWidth;
    transferPropOut[0].transferDim.sdim3 = 0;

    transferPropOut[0].transferDim.dicnt0 = transferPropIn[0].transferDim.sicnt0;
    transferPropOut[0].transferDim.dicnt1 = transferPropIn[0].transferDim.sicnt1;
    transferPropOut[0].transferDim.dicnt2 = transferPropIn[0].transferDim.sicnt2;
    transferPropOut[0].transferDim.dicnt3 = transferPropIn[0].transferDim.sicnt3 ;
    transferPropOut[0].transferDim.ddim1 = outPitch;
    transferPropOut[0].transferDim.ddim2 = blockWidth;
    transferPropOut[0].transferDim.ddim3 = blockHeight * outPitch;

    return numHorzTrsRequired * numVertTrRowsRequired;

}

typedef enum{
  DMAUTILSTESTAUTOINC_CHANNEL_IN,
  DMAUTILSTESTAUTOINC_CHANNEL_OUT,
  DMAUTILSTESTAUTOINC_CHANNEL_MAX
}dmautilsTestAutoInc_Channel;


#define DMAUTILSTESTAUTOINC_MAX_NUM_TR  (32)
#define TEST_ALIGN_SIZE (128U)


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
    uint8_t *outTrMem;
    uint32_t pingPongFlag = 0;
    uint32_t blockIdx = 0;
    uint32_t firstTrigger = 0;
    uint32_t inTrSize;
    uint32_t outTrSize;
    uint32_t numTrReq;
    uint32_t dmaChannels;

    DmaUtilsAutoInc3d_InitParam initParams;
    DmaUtilsAutoInc3d_ChannelInitParam chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_MAX];
    DmaUtilsAutoInc3d_TrPrepareParam trPrepParamIn;
    DmaUtilsAutoInc3d_TrPrepareParam trPrepParamOut;
    DmaUtilsAutoInc3d_TransferProp *transferPropIn;
    DmaUtilsAutoInc3d_TransferProp *transferPropOut;
    uint32_t circularPitch;
    uint32_t blockOffset = 0;
    uint32_t linearOffset = 0;
    Udma_InitPrms   initPrms;
    struct Udma_DrvObj      udmaDrvObj;
    uint32_t        instId;

    Udma_DrvHandle  drvHandle = &udmaDrvObj;

    /* Circularity testcase is only for exact multiples */
    if ( ( (width % blockWidth) != 0  ) || ( (height % blockHeight) != 0  ) )
    {
      return -1;
    }
    circularPitch = blockWidth *  2;
    circularPitch = (circularPitch < 512)? 512 : circularPitch;
    while ((circularPitch & (circularPitch - 1)) != 0)
    {
      circularPitch += 1;
    }

    instId = UDMA_INST_ID_MAIN_0;
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = &testDmaAutoIncPrintf;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        testDmaAutoIncPrintf("[Error] UDMA init failed!!\n");
    }


    dmaChannels = DMAUTILSTESTAUTOINC_CHANNEL_MAX; /* One for input and other for output */

    //Allocation/Assignment of buffers in internal memory
    dmautilsContext     =  pIntMmeBase + intMemUsedSize ;
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL(DmaUtilsAutoInc3d_getContextSize(dmaChannels), TEST_ALIGN_SIZE);

    transferPropIn = (DmaUtilsAutoInc3d_TransferProp * ) (pIntMmeBase + intMemUsedSize );
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL((DMAUTILSTESTAUTOINC_MAX_NUM_TR * sizeof(DmaUtilsAutoInc3d_TransferProp)), TEST_ALIGN_SIZE);

    transferPropOut = (DmaUtilsAutoInc3d_TransferProp * ) (pIntMmeBase + intMemUsedSize );
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL((DMAUTILSTESTAUTOINC_MAX_NUM_TR * sizeof(DmaUtilsAutoInc3d_TransferProp)), TEST_ALIGN_SIZE);

    initParams.contextSize = DmaUtilsAutoInc3d_getContextSize(dmaChannels);
    initParams.numChannels = dmaChannels;
    initParams.traceLogLevel    = 1;
    initParams.udmaDrvHandle = drvHandle;
    initParams.DmaUtilsVprintf = vprintf;

    numTrReq = testDmaAutoIncNumTrRequired(width, height, blockWidth, blockHeight, NULL, NULL);

    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_IN].dmaQueNo  = 0;
    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_OUT].dmaQueNo  = 0;

    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_IN].druOwner    = DMAUTILSAUTOINC3D_DRUOWNER_DIRECT_TR;
    chInitParams[DMAUTILSTESTAUTOINC_CHANNEL_OUT].druOwner    = DMAUTILSAUTOINC3D_DRUOWNER_DIRECT_TR;


    retVal = DmaUtilsAutoInc3d_init(dmautilsContext, &initParams, chInitParams);
    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }

    numTrReq =     testDmaAutoIncSetupTr(width,
                                                height,
                                                blockWidth,
                                                blockHeight,
                                                inPitch,
                                                outPitch,
                                                pInput,
                                                pInputBlock,
                                                pOutput,
                                                pOutputBlock,
                                                circularPitch,
                                                &transferPropIn[0],
                                                &transferPropOut[0]);

    inTrSize = DmaUtilsAutoInc3d_getTrMemReq(numTrReq);
    outTrSize = DmaUtilsAutoInc3d_getTrMemReq(numTrReq);

    inTrMem = pIntMmeBase + intMemUsedSize ;

    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL(inTrSize, TEST_ALIGN_SIZE);

    outTrMem = pIntMmeBase + intMemUsedSize ;
    intMemUsedSize += TEST_DMAUTILS_ALIGN_CEIL(outTrSize, TEST_ALIGN_SIZE);

    trPrepParamIn.channelId = DMAUTILSTESTAUTOINC_CHANNEL_IN;
    trPrepParamIn.numTRs  = numTrReq;
    trPrepParamIn.trMem     = inTrMem;
    trPrepParamIn.trMemSize = inTrSize;

    retVal = DmaUtilsAutoInc3d_prepareTr(&trPrepParamIn, &transferPropIn[0]);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }


    trPrepParamOut.channelId = DMAUTILSTESTAUTOINC_CHANNEL_OUT;
    trPrepParamOut.numTRs  = numTrReq;
    trPrepParamOut.trMem     = outTrMem;
    trPrepParamOut.trMemSize = outTrSize;

    retVal = DmaUtilsAutoInc3d_prepareTr(&trPrepParamOut, &transferPropOut[0]);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }

    if(intMemUsedSize > intMemSize)
    {
      printf("insufficient memory, required is %d vs provided %d\n",intMemUsedSize, intMemSize);
      return -1 ;
    }

    retVal = DmaUtilsAutoInc3d_configure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN, inTrMem, numTrReq);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }

    retVal = DmaUtilsAutoInc3d_configure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT, outTrMem, numTrReq);

    if ( retVal != UDMA_SOK )
    {
      goto Exit;
    }


    //DMA trigger for pipe-up, out transfer is dummy and handled inside DMA utility
    DmaUtilsAutoInc3d_trigger(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);
    //Wait for previous transfer of in
    DmaUtilsAutoInc3d_wait(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);

    pingPongFlag^=1;
    blockIdx = 0;
    linearOffset  = 0;

    while (1)
    {

      pingPongFlag^=1;

      if (firstTrigger != 0 )
      {
         blockIdx = DmaUtilsAutoInc3d_trigger(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT) ;
      }

      //DMA trigger for next in buffer
      if ( blockIdx != 1)
      {
        DmaUtilsAutoInc3d_trigger(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);
      }

      blockCopyKernel(
        pInputBlock   + blockOffset,
        pOutputBlock + blockOffset,
        blockWidth,
        blockHeight,
        circularPitch,
        circularPitch);

      linearOffset += (blockWidth );
      blockOffset = (linearOffset & (circularPitch - 1));


      if ( blockIdx != 1 )
      {
        DmaUtilsAutoInc3d_wait(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN);
      }
      //Wait for previous transfer out
       if (firstTrigger != 0 )
      {
        DmaUtilsAutoInc3d_wait(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT)  ;
      }
      else
      {
        firstTrigger = 1;
      }


      if ( blockIdx == 1 )
        break;

    }

    DmaUtilsAutoInc3d_trigger(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT) ;

    //Need to wait for last out transfer
    DmaUtilsAutoInc3d_wait(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT)  ;


    retVal = DmaUtilsAutoInc3d_deconfigure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_IN, inTrMem, 1);

     if ( retVal != UDMA_SOK )
     {
       goto Exit;
     }

     retVal = DmaUtilsAutoInc3d_deconfigure(dmautilsContext, DMAUTILSTESTAUTOINC_CHANNEL_OUT, outTrMem, 1);

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

