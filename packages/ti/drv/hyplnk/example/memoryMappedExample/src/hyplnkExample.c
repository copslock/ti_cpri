/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/
 * 
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
 *
 */

/*  
 *  This is a simple example which brings up HyperLink.  It can work
 *  between two devices or in loopback on one device.
 *
 *  It is suggested to use the platform GEL file, since as evmc6678l.gel,
 *  that comes with CCS 5.x or newer.  This sets up the system PLL and
 *  DDR.  This example configures the PLL associated with hyperlink.
 *
 *  Once HyperLink is configured, data is exchanged.  The buffer is 
 *  arranged such that the logic is agnostic to whether there are two
 *  devices connected, or it is a single device in loopback.
 *
 *  There are two types of data exchanges.  The first is a token exchange,
 *  where a token (sequence number) is passed symmetrically between the
 *  two device.  This exchange uses the dataBuffer.cpuTokens and 
 *  dataBuffer.dmaTokens.  This is a synchronous transfer, because
 *  each side waits until it sees the appropriate response from the
 *  other side before proceeding.
 *
 *  Since the transaction is symmetric it also works with loopback.  
 *  The token is actually passed each direction twice, such that each device
 *  issues remote reads and remote writes.
 *
 *  The second type of data exchange is asynchronous block transfer.  
 *  This transfer can be run on either or both sides independently.
 *  A block of data is remotely written then read back via
 *  dataBuffer.cpuBlock and dataBuffer.dmaBlock.
 *
 *  The source of the data is currently in the local L2SRAM and
 *  transfers to the remote L2SRAM. In loopback mode the remote and local L2SRAM
 *  are the same exact locations. The remote address is currently based off of the
 *  local address (for flexibility) and can be easily changed within the configuration file.
 *  It is important that the starting address must be aligned to 64KB
 *  (e.g. 0x0001_0000, 0x0080_0000, 0x0081_0000, 0x8007_0000).
 *
 *  Both the CPU and DMA transfers are currently implemented.
 *  EDMA, QDMA, and infrastructure DMA are currently implemented
 *  as DMA transfers.
 *
 *  Infrastructure DMA uses the QMSS and Packet DMA to transfer data. The way it works with HyperLink
 *  is by setting up a queue manager on the local and remote side. The local queue manager will then send
 *  the packets (which contain the HyperLink addresses of the memory to be transfered) through HyperLink
 *  and the remote queue manager then requests data from the local side using the HyperLink addresses
 *  that were given in the packets.
 *
 *  In this example infrastructure DMA uses monolithic descriptors and the user can select between
 *  polling mode or accumulated mode which determines how the data is received.
 *
 *  In non-loopback mode two boards must be connected to each other via a HyperLink cable.
 *  Both boards must run this code at the same time in order for this example code to work.
 *  The maximum sustainable HyperLink serial rate depends on the length of the cable.
 *
 *  ALL user configuration for HyperLink, EDMA, interrupt, and infraDMA can be found in the
 *  hyplnkCfg.h file.
 *
 *  The defines below are configured for the C6678 and C6670 EVM
 *  coupled with a SAS cable.
 *
 *  For other configurations, change the definitions starting 
 *  at "Beginning of user configuration" as required.
 *
 */

#include "hyplnkResource.h"
#include <ti/drv/hyplnk/hyplnk.h>
#include <ti/csl/csl_bootcfgAux.h>

#include "hyplnkLLDIFace.h"
#include "hyplnkPlatCfg.h"

#ifndef __LINUX_USER_SPACE
#include "../../InfraDMA/hyplnkInfraDMA.h"
#include "../../EDMA/hyplnkEDMA.h"
#endif

/* Adjust timings based on loopback */
#ifdef hyplnk_EXAMPLE_LOOPBACK
  #define hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT   hyplnk_EXAMPLE_uS_TO_CYCLES(   100)
  #define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT    hyplnk_EXAMPLE_uS_TO_CYCLES(   100)
#else
  #define hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT   hyplnk_EXAMPLE_uS_TO_CYCLES(30000000)
  #ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
    #define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT  hyplnk_EXAMPLE_uS_TO_CYCLES(    1000)
  #else
    #define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT  hyplnk_EXAMPLE_uS_TO_CYCLES(      10)
  #endif
#endif

/* Number of times to pass tokens between remote and local */
#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
  #define hyplnk_EXAMPLE_NTOKENS    0x10000 /* token pass round trip cycles */
#else
  #define hyplnk_EXAMPLE_NTOKENS    0x1000000 /* token pass round trip cycles */
#endif

/* Convert speed to string for printing */
#if defined(hyplnk_EXAMPLE_SERRATE_01p250)
  #define hyplnk_EXAMPLE_LINK_SPEED "1.25G"
#elif defined(hyplnk_EXAMPLE_SERRATE_03p125)
  #define hyplnk_EXAMPLE_LINK_SPEED "3.125"
#elif defined(hyplnk_EXAMPLE_SERRATE_06p250)
  #define hyplnk_EXAMPLE_LINK_SPEED "6.25"
#elif defined(hyplnk_EXAMPLE_SERRATE_07p500)
  #define hyplnk_EXAMPLE_LINK_SPEED "7.5"
#elif defined(hyplnk_EXAMPLE_SERRATE_10p000)
  #define hyplnk_EXAMPLE_LINK_SPEED "10.0"
#elif defined(hyplnk_EXAMPLE_SERRATE_12p500)
  #define hyplnk_EXAMPLE_LINK_SPEED "12.5"
#else
  #define hyplnk_EXAMPLE_LINK_SPEED "Unknown - add #define"
#endif

#ifdef hyplnk_EXAMPLE_ALLOW_4_LANES
  #define hyplnk_EXAMPLE_MAX_LANES 4
#else
  #define hyplnk_EXAMPLE_MAX_LANES 1
#endif

typedef struct {
  uint32_t value;
  char     padding[hyplnk_EXAMPLE_LINE_SIZE - 4];
} hyplnkExampleOneToken_t;

/* bidirectional token buffers */
typedef struct {
  hyplnkExampleOneToken_t srcLoc2Rem;
  hyplnkExampleOneToken_t srcRem2Loc;
  hyplnkExampleOneToken_t dstLoc2Rem;
  hyplnkExampleOneToken_t dstRem2Loc;
} hyplnkExampleTokenBuffer_t;

/* Memory block transfer buffer */
typedef struct {
  uint32_t buffer[hyplnk_EXAMPLE_BLOCK_BUF_SIZE];
} hyplnkExampleBlockBuffer_t;

/* Combined shared data structure containing tokens and blocks */
typedef struct {
#ifdef hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE
  hyplnkExampleTokenBuffer_t cpuTokens;
#endif

#ifdef hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE
#define EDMA
  hyplnkExampleTokenBuffer_t dmaTokens;
#endif

#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
  hyplnkExampleBlockBuffer_t cpuBlock;
#endif

#ifdef hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER
#ifndef EDMA
#define EDMA
#endif
  hyplnkExampleBlockBuffer_t dmaBlock;
#endif

#ifdef hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    int dummyBlock[1]; //This is to keep the program running when using only InfraDMA
#endif
  /* If user didn't define any tests, this blank struct will generate error */
} hyplnkExampleDataBuffer_t;

/* Global variable timers for throughput */
uint64_t totalCPUTime = 0;
uint64_t totalINFRATime = 0;
uint64_t totalDMATime = 0;

#ifdef enableEDMA
/* This is the data that will be used as a temporary space holder
 * for the data being transfered using DMA.
 *
 * This is done since EDMA cannot send a specific value or token
 * but instead it can send blocks of data.
 * */
#pragma DATA_SECTION(dataContainer, ".bss:testData")
#pragma DATA_ALIGN(dataContainer, hyplnk_EXAMPLE_LINE_SIZE)
UInt32 dataContainer[hyplnk_EXAMPLE_BLOCK_BUF_SIZE];

#endif
#ifdef infraDMA  
#pragma DATA_SECTION(RXPacketBuffer, ".bss:packetData")
#pragma DATA_ALIGN (RXPacketBuffer, 128);
hyplnkQMSSExamplePacketBlock_t RXPacketBuffer;
#endif
#if  defined(hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE) || defined(hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE)
/*****************************************************************************
 * Write val into ptr, writeback cache, and wait for result to land.
 * This function is agnostic of physical location of ptr (L2, MSMC, DDR, etc)
 ****************************************************************************/
void hyplnkExampleSet (volatile uint32_t *ptr, uint32_t val)
{
  *ptr = val;
#ifdef hyplnk_CACHE_ENABLE
  CACHE_wbL1d ((void *)ptr, sizeof(*ptr), CACHE_FENCE_WAIT);
  CACHE_wbL2  ((void *)ptr, sizeof(*ptr), CACHE_FENCE_WAIT);
#endif
}

/*****************************************************************************
 * Invalidate cache, then read ptr
 * This function is agnostic of physical location of ptr (L2, MSMC, DDR, etc)
 ****************************************************************************/
uint32_t hyplnkExampleGet (volatile uint32_t *ptr)
{
#ifdef hyplnk_CACHE_ENABLE
  CACHE_invL1d ((void *)ptr, sizeof(*ptr), CACHE_WAIT);
  CACHE_invL2  ((void *)ptr, sizeof(*ptr), CACHE_WAIT);
#endif
  return *ptr;
}
#endif

#ifdef hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE
/*****************************************************************************
 * Pass one "token" by writing to the "write" address then attempting
 * to read the same value back via the "read" address.  
 *
 * The CPU generates the transactions
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int hyplnkExampleCPUTokenExchange (hyplnkExampleTokenBuffer_t *local, 
                                   hyplnkExampleTokenBuffer_t *remote, 
                                   uint32_t token,
                                   uint64_t timeLimit)
{
  uint64_t startTime;
  int done = 0;
  uint32_t val1, val2;
  int other_side_copied = 0, i_copied = 0; 

  /* Write my srcLoc2Rem, and the other side's srcRem2Loc */
  hyplnkExampleSet(&local->srcLoc2Rem.value, token);
  hyplnkExampleSet(&remote->srcRem2Loc.value, token);

  /* Wait for the remote side to copy:
   * local->dstLoc2Rem.value = local->srcLoc2Rem.value 
   * and
   * remote->dstRem2Loc.value = remote->srcRem2Loc.value
   */
  startTime = hyplnkExampleReadTime();
#ifdef __ARMv7
   //while (((hyplnkExampleReadTime() - startTime) < 1) && (!done)) {
  while(!done) {
#else
  while (((hyplnkExampleReadTime() - startTime) < timeLimit) && (!done)) {
#endif
    /* see if the other side did his copy */
    if (! other_side_copied) {
      val1 = hyplnkExampleGet(&local->dstLoc2Rem.value);
      val2 = hyplnkExampleGet(&remote->dstRem2Loc.value);
      other_side_copied = ((val1 == token) && (val2 == token));
    }
    /* Do my copy */
    if (! i_copied) {
      val1 = hyplnkExampleGet(&remote->srcLoc2Rem.value);
      val2 = hyplnkExampleGet(&local->srcRem2Loc.value);
      i_copied = ((val1 == token) && (val2 == token));
      if (i_copied) {
        hyplnkExampleSet(&remote->dstLoc2Rem.value, val1);
        hyplnkExampleSet(&local->dstRem2Loc.value, val2);
      }
    }
    done = i_copied && other_side_copied;
  }
  if (! done) {
    printf("fail %d %d\n", other_side_copied, i_copied);
  }
  return ! done;
}
#endif /* hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE */

#ifdef hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE
/*****************************************************************************
 * Pass one "token" by writing to the "write" address then attempting
 * to read the same value back via the "read" address.  
 *
 * The EDMA generates the transactions
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int hyplnkExampleDMATokenExchange (hyplnkExampleTokenBuffer_t *local, 
                                   hyplnkExampleTokenBuffer_t *remote, 
                                   uint32_t token,
                                   uint64_t timeLimit,
                                   EDMA3_DRV_Handle hEdma)
{
    int ACount, BCount, CCount;
    uint64_t startTime;
    int done = 0;
    uint32_t val1, val2;
    int other_side_copied = 0, i_copied = 0;

    dataContainer[0] = token;

    /*
     * Setting up EDMA parameters
     *
     * The following expressions take care of overflow
     *
     */
    if (sizeof(hyplnkExampleOneToken_t) > 16384) {
        ACount = 16384;
        BCount = sizeof(hyplnkExampleOneToken_t) / 16384;
    } else {
        ACount = sizeof(hyplnkExampleOneToken_t);
        BCount = 1;
    }
    CCount = 1;

    /* Write my srcLoc2Rem, and the other side's srcRem2Loc */
    edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) dataContainer,(unsigned int*) &local->srcLoc2Rem.value,
            ACount, BCount, CCount,EDMA3_DRV_SYNC_A,NULL);
    edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) dataContainer,(unsigned int*) &remote->srcRem2Loc.value,
            ACount, BCount, CCount,EDMA3_DRV_SYNC_A,NULL);

    /* Wait for the remote side to copy:
     * local->dstLoc2Rem.value = local->srcLoc2Rem.value
     * and
     * remote->dstRem2Loc.value = remote->srcRem2Loc.value
     */
    startTime = hyplnkExampleReadTime();
    while (((hyplnkExampleReadTime() - startTime) < timeLimit) && (!done)) {
        /* see if the other side did his copy */
        if (!other_side_copied) {
            val1 = hyplnkExampleGet(&local->dstLoc2Rem.value);
            val2 = hyplnkExampleGet(&remote->dstRem2Loc.value);
            other_side_copied = ((val1 == token) && (val2 == token));
        }
        /* Do my copy */
        if (!i_copied) {
            val1 = hyplnkExampleGet(&remote->srcLoc2Rem.value);
            val2 = hyplnkExampleGet(&local->srcRem2Loc.value);
            i_copied = ((val1 == token) && (val2 == token));
            if (i_copied) {
                dataContainer[0] = val1;
                edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) dataContainer,(unsigned int*) &local->dstLoc2Rem.value,
                        ACount,BCount, CCount, EDMA3_DRV_SYNC_A,NULL);
                dataContainer[0] = val2;
                edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) dataContainer,(unsigned int*) &remote->dstRem2Loc.value,
                        ACount,BCount, CCount, EDMA3_DRV_SYNC_A,NULL);
            }
        }
        done = i_copied && other_side_copied;
    }
    if (!done) {
        System_printf("fail %d %d\n", other_side_copied, i_copied);
    }
    return !done;
}
#endif /* hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE */

#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
/*****************************************************************************
 * Write a block of data to remote side then verify it
 *
 * The CPU generates the transactions via memcpy and memset
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int hyplnkExampleCPUBlockXfer (hyplnkExampleBlockBuffer_t *remote, 
                               uint32_t value,
                               uint64_t timeLimit)
{
  int i;
  uint32_t *remoteBuf;
  int fail = 0;
  uint64_t startTime, totalTime;

  hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "before block IO", 1);

  /* Write the data to the remote buffer */
  startTime = hyplnkExampleReadTime();
  for (i = 0, remoteBuf = remote->buffer; 
       i < hyplnk_EXAMPLE_BLOCK_BUF_SIZE; 
       i++) {
    *remoteBuf++ = value;
  }
  
#ifdef hyplnk_CACHE_ENABLE
  CACHE_wbInvL1d ((void *)remote, sizeof(*remote), CACHE_FENCE_WAIT);
  CACHE_wbInvL2  ((void *)remote, sizeof(*remote), CACHE_FENCE_WAIT);
#endif

  totalTime = hyplnkExampleReadTime()-startTime;
  totalCPUTime += totalTime;
  /* Verify the data in the remote buffer */
  for (i = 0, remoteBuf = remote->buffer; 
       i < hyplnk_EXAMPLE_BLOCK_BUF_SIZE; 
       i++) {
    if (*remoteBuf++ != value) fail++;
  }

  return fail;
}
#endif /* hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER */

#ifdef hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER
/*****************************************************************************
 * Write a block of data to remote side then verify it
 *
 * The EDMA generates the transactions
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int hyplnkExampleDMABlockXfer (hyplnkExampleBlockBuffer_t *remote, 
                               uint32_t value,
                               uint64_t timeLimit,
                               EDMA3_DRV_Handle hEdma) 
{
    int ACount, BCount, CCount;
    int i;
    int fail = 0;
    int sizeCheck=0;
    uint32_t *remoteBuf;
    uint32_t *source;
    unsigned long totalTime=0;
    unsigned long*totalTimePointer;

    for (i = 0; i < hyplnk_EXAMPLE_BLOCK_BUF_SIZE; i++) {
        dataContainer[i] = value;
    }

    hyplnkExampleCheckOneStat(hyplnk_LOCATION_LOCAL, "before block IO", 1);

    /*
     * Setting up EDMA parameters
     *
     * The following takes care of large array sizes by spreading the data into
     * BCount arrays
     *
     */

    sizeCheck =sizeof(dataContainer);
    if (sizeCheck > 16384) {
        ACount = 16384;
        BCount =  sizeCheck / 16384;
    } else {
        ACount = sizeCheck ;
        BCount = 1;
    }
    CCount = 1;

    remoteBuf = remote->buffer;
    source = dataContainer;

    totalTimePointer=&totalTime;
    *totalTimePointer=0;

    edmaTransfer(hEdma,(EDMA3_Type) EDMA_TYPE, (unsigned int*) source, (unsigned int*) remoteBuf,
            ACount, BCount, CCount, EDMA3_DRV_SYNC_A,totalTimePointer);

    totalDMATime += *totalTimePointer;
    
#ifdef hyplnk_CACHE_ENABLE
    CACHE_invL1d ((void *)remote, sizeof(*remote), CACHE_WAIT);
    CACHE_invL2  ((void *)remote, sizeof(*remote), CACHE_WAIT);
#endif

    /* Verify the data in the remote buffer */
    for (i = 0, remoteBuf = remote->buffer; i < hyplnk_EXAMPLE_BLOCK_BUF_SIZE; i++) {
        if (*remoteBuf++ != value)
            fail++;
    }

    return fail;
}
#endif /* hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER */

#ifdef hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
/*****************************************************************************
 * Sets up queue manage on local and remote sides. Then initializes monolithic descriptor with
 * HyperLink addresses from local side and sends that descriptor to remote side. From there remote
 * side reads the packet and transfers the data from local side to remote side.
 *
 * The QMSS generates the transactions
 *
 *              0 : pass
 * greater than 0 : fail
 ****************************************************************************/
int hyplnkExampleINFRABlockXfer(hyplnkQMSSExamplePacketBlock_t *RXpacket,
        hyplnkQMSSExamplePacketBlock_t *HyperLinkPacket,
        uint32_t value, uint64_t timeLimit, Cppi_Handle cppiHnd,Qmss_Result localRegion, Qmss_Result remoteRegion) {

    int result;
    int i,j;
    int fail = 1;
    uint32_t RxPktAdr;


    /* The following function sets up the infraDMA, sends the data to RXpacket using HyperLink,
     * and then returns the total time it took to send the data.
     * */
    result = InfraDMATransfer(cppiHnd, value,localRegion,remoteRegion,HyperLinkPacket);

    if(result!=-1) {
        totalINFRATime += result;
        fail=0;
    }
    RxPktAdr=(uint32_t)RXpacket;

    if ((RxPktAdr >= 0x800000) && (RxPktAdr < 0xa00000)) {
        /* Address is in L2 */
        RxPktAdr |= 0x10000000 | (DNUM << 24);
    }
    
#ifdef hyplnk_CACHE_ENABLE
    CACHE_invL1d ((void *)RxPktAdr, sizeof(*RXpacket), CACHE_WAIT);
    CACHE_invL2  ((void *)RxPktAdr, sizeof(*RXpacket), CACHE_WAIT);
#endif

    /* Verify the data in the remote packet */
    for(i=0;i<NUM_PACKETS;i++){
            for(j=0;j<(SIZE_DATA_BUFFER/NUM_PACKETS);j++){
                if(RXpacket->txQmssBlock[i].dataBuffer[j]!=value){
                    fail++;
                }
            }
        }

    return fail;
}
#endif /* hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER */

/*****************************************************************************
 * Perform an IO cycle (token pass or read/write)
 *
 * 0: pass
 * 1: fail
 ****************************************************************************/
int hyplnkExampleIOCycle (hyplnkExampleDataBuffer_t *local, 
                          hyplnkExampleDataBuffer_t *remote, 
#ifdef infraDMA  
                          hyplnkQMSSExamplePacketBlock_t *RXpacket,
                          hyplnkQMSSExamplePacketBlock_t *HyperLinkPacket,
#endif
                          uint32_t value,
                          uint64_t timeLimit
#ifdef enableEDMA
                         ,EDMA3_DRV_Handle hEdma
#endif
#ifdef infraDMA  
                         ,Cppi_Handle cppiHnd,
                         Qmss_Result localRegion,
                         Qmss_Result remoteRegion
#endif
)
{
  int oneRetVal;
  int retval = 0;
#ifdef hyplnk_EXAMPLE_TEST_CPU_TOKEN_EXCHANGE
  oneRetVal = hyplnkExampleCPUTokenExchange (
                &local->cpuTokens, &remote->cpuTokens, value, timeLimit);
  if (oneRetVal) {
    printf ("hyplnkExampleCPUTokenExchange failed: %d\n", oneRetVal);
    retval = 1;
  }
#endif
#ifdef hyplnk_EXAMPLE_TEST_DMA_TOKEN_EXCHANGE
  oneRetVal = hyplnkExampleDMATokenExchange (
                &local->dmaTokens, &remote->dmaTokens, value, timeLimit, hEdma);
  if (oneRetVal) {
    printf ("hyplnkExampleDMATokenExchange failed: %d\n", oneRetVal);
    retval = 1;
  }
#endif
#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
  oneRetVal = hyplnkExampleCPUBlockXfer (&remote->cpuBlock, value, timeLimit);
  if (oneRetVal) {
    printf ("hyplnkExampleCPUBlockXfer failed: %d\n", oneRetVal);
    retval = 1;
  }
#endif
#ifdef hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER
  oneRetVal = hyplnkExampleDMABlockXfer (&remote->dmaBlock, value, timeLimit, hEdma);
  if (oneRetVal) {
    printf ("hyplnkExampleDMABlockXfer failed: %d\n", oneRetVal);
    retval = 1;
  }
#endif
#ifdef hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    oneRetVal = hyplnkExampleINFRABlockXfer(RXpacket, HyperLinkPacket, value, timeLimit, cppiHnd,localRegion,remoteRegion);
    if (oneRetVal) {
        System_printf("hyplnkExampleINFRABlockXfer failed: %d\n", oneRetVal);
        retval = 1;
    }
#endif

  return retval;
}



/* This is the actual data buffer that will be accessed.  
 *
 * When accessing this location directly, the program is playing
 * the role of the remote device (remember we are in loopback).
 *
 * If this is placed in an L2 RAM there are no other dependancies.
 *
 * If this is placed in MSMC or DDR RAM, then the SMS
 * or SES MPAX must be configured to allow the access.
 */

#pragma DATA_SECTION (dataBuffer, ".bss:hyplnkData");
#pragma DATA_ALIGN (dataBuffer, hyplnk_EXAMPLE_LINE_SIZE);
hyplnkExampleDataBuffer_t dataBuffer;

/* 
 * This pointer is the local address within the Hyperlink's address
 * space which will access dataBuffer via HyperLink.
 *
 */
hyplnkExampleDataBuffer_t *bufferThroughHypLnk;
#ifdef infraDMA  
/*
 * These pointers are used for infraDMA. The QMSSBuffer pointer is used for
 * the remote queue manager and the packetBuffer is used by the remote QMSS to
 * access HyperLink addresses that point to local memory.
 *
 */
hyplnkQMSSExamplePacketBlock_t *QMSSBufferThroughHypLnk;
hyplnkQMSSExamplePacketBlock_t *packetBufferThroughHypLnk;
#endif

int main(void)
{
    hyplnkRet_e retVal;
    uint32_t token;
    uint32_t i;
    int iteration = 0;
    hyplnkExampleDataBuffer_t *dataBufPtr;
#ifndef __ARMv7
    uint8_t mar;
    TSCL = 1;
#endif

#ifdef infraDMA  
    Qmss_Result localRegion,remoteRegion;
#endif

#if defined (SOC_C6678)
    CSL_BootCfgUnlockKicker();
#endif

    /*Initialize EDMA and InfraDMA handles*/
#ifdef EDMA
    EDMA3_DRV_Handle hEdma;
    hEdma = NULL;
#endif

#ifdef infraDMA  
#ifdef     hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    Cppi_Handle cppiHnd;
    cppiHnd = NULL;
#else
    int cppiHnd = NULL;
#endif
#endif
    if (hyplnk_memAllocInit() != 0)
    {
        printf("Memory Alloc Init failed!");
        exit(1);
    }

#ifdef _VIRTUAL_ADDR_SUPPORT
    dataBufPtr = (hyplnkExampleDataBuffer_t *) hyplnk_mmap(CSL_MSMC_SRAM_REGS, 0x1000);
#else
    dataBufPtr = &dataBuffer;
    /* Hyperlink is not up yet, so we don't have to worry about concurrence */
    memset (&dataBuffer, 0, sizeof(dataBuffer));
#endif

    printf ("Version #: 0x%08x; string %s\n", Hyplnk_getVersion(), Hyplnk_getVersionStr());


    /* Pass device config to LLD */
    if ((retVal = Hyplnk_init (&hyplnkInitCfg)) != hyplnk_RET_OK)
    {
        printf ("LLD device configuration failed\n");
        exit(1);
    }

    /* Set up the system PLL, PSC, and DDR as required for this HW */
    printf ("About to do system setup (PLL, PSC, and DDR)\n");
    if ((retVal = hyplnkExampleSysSetup()) != hyplnk_RET_OK) {
        printf ("system setup failed (%d)\n", (int)retVal);
        exit(1);
    }
    printf ("system setup worked\n");

  /* Install the ISR */
#ifdef hyplnk_EXAMPLE_ERROR_INTERRUPT
  hyplnkExampleInstallIsr();
#endif

    /* Enable the peripheral */
    printf ("About to set up HyperLink Peripheral\n");
    if ((retVal = hyplnkExamplePeriphSetup()) != hyplnk_RET_OK) {
        printf ("Peripheral setup failed (%d)\n", (int)retVal);
        exit(1);
    }
    printf ("Peripheral setup worked\n");

    /* Set up address mapsrc */
#ifdef _VIRTUAL_ADDR_SUPPORT
      if ((retVal = hyplnkExampleAddrMap ((void *)CSL_MSMC_SRAM_REGS, (void **)&bufferThroughHypLnk, 0)) != hyplnk_RET_OK) {
#else
    if ((retVal = hyplnkExampleAddrMap (dataBufPtr, (void **)&bufferThroughHypLnk, 0)) != hyplnk_RET_OK) {
#endif
        printf ("Address map setup failed (%d)\n", (int)retVal);
        exit(1);
    }

#ifndef _VIRTUAL_ADDR_SUPPORT
    /* Enable caching of hlink area */
    mar = ((uint32_t)bufferThroughHypLnk >> 24);
    CACHE_setL2Size (CACHE_32KCACHE);
    CACHE_enableCaching (mar);
#endif

#ifdef hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
#ifndef _VIRTUAL_ADDR_SUPPORT
    /* Set up QMSSBufferThroughHypLnk to be a hyperlink address that will point to QMSS_DATA_ADDR address on the remote side  */
    if ((retVal = hyplnkExampleAddrMap((void*) QMSS_DATA_ADDR, (void **) &QMSSBufferThroughHypLnk, 5)) != hyplnk_RET_OK) {
        printf("Queue Manager address map setup failed (%d)\n", (int) retVal);
        exit(1);
    }

    /* Enable caching of hlink area */
    mar = ((uint32_t)QMSSBufferThroughHypLnk >> 24);
    CACHE_setL2Size (CACHE_32KCACHE);
    CACHE_enableCaching (mar);

    /* Set up packetBufferThroughHypLnk to be a hyperlink address that will point to RXPacketBuffer address on the remote side  */
    if ((retVal = hyplnkExampleAddrMap(&RXPacketBuffer, (void **) &packetBufferThroughHypLnk, 7)) != hyplnk_RET_OK) {
        printf("InfraDMA data address map setup failed (%d)\n", (int) retVal);
        exit(1);
    }

    /* Enable caching of hlink area */
    mar = ((uint32_t)packetBufferThroughHypLnk >> 24);
    CACHE_enableCaching (mar);

    /* Enable caching of packet area */
    mar = ((uint32_t)&RXPacketBuffer >> 24);
    CACHE_enableCaching (mar);
#endif
#endif

    /*Initialize EDMA handle*/
#ifdef EDMA
    hEdma = edmaInit(hEdma);
    if (hEdma==NULL) System_printf("ERROR: EDMA handle not initialized!\n");
#endif

    /*Initialize CPPI handle*/
#ifdef     hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    localRegion=0;
    remoteRegion=0;
    cppiHnd = InfraDMAInit((int) QMSSBufferThroughHypLnk,(int) packetBufferThroughHypLnk, &localRegion, &remoteRegion,cppiHnd);
    if (cppiHnd==NULL) System_printf("ERROR: CPPI handle not initialized!\n");
#endif

    /* Perform one read/write cycle */
    printf ("About to read/write once\n");
    if (hyplnkExampleIOCycle (dataBufPtr, bufferThroughHypLnk,
#ifdef infraDMA  
                              &RXPacketBuffer, packetBufferThroughHypLnk,
#endif
                              0xfee1dead,hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT
#ifdef enableEDMA
                              ,(EDMA3_DRV_Handle)hEdma
#endif
#ifdef infraDMA  
                              ,(Cppi_Handle)cppiHnd,localRegion,remoteRegion
#endif
         )) {
        printf ("Single write failed\n");
        exit(1);
    }
    printf ("Single write test passed\n");

  /* Bulk token pass test */
  for (iteration = 0; 
       (hyplnk_EXAMPLE_NITERS == 0) || (iteration < hyplnk_EXAMPLE_NITERS); 
       iteration++) {
    printf ("About to pass %d tokens; iteration = %d\n", 
                   hyplnk_EXAMPLE_NTOKENS, iteration);

    /* Run one more read/write as a barrier (to compensate for above printf) */
    /* Do not printf unless test fails (else printf will CAUSE fail!) */
    if (hyplnkExampleIOCycle (dataBufPtr, bufferThroughHypLnk,
#ifdef infraDMA  
                              &RXPacketBuffer, packetBufferThroughHypLnk,
#endif
                              0xbabeface, hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT
#ifdef enableEDMA
                              ,(EDMA3_DRV_Handle)hEdma
#endif
#ifdef infraDMA  

                              ,(Cppi_Handle)cppiHnd,localRegion, remoteRegion
#endif
        )) {
        printf ("Single write failed (when synchronizing bulk test)\n");
        exit(1);
    }

    /* Initialize time measurements for bulk test*/
    totalCPUTime   = 0;
    totalDMATime   = 0;
    totalINFRATime = 0;
    
    /* Bulk test */
    for (i = 0, token = 0; i < hyplnk_EXAMPLE_NTOKENS; i++) {
      if (hyplnkExampleIOCycle (dataBufPtr, bufferThroughHypLnk,
#ifdef infraDMA  
                                &RXPacketBuffer,packetBufferThroughHypLnk,
#endif
                                ++token, hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT
#ifdef enableEDMA
                                ,(EDMA3_DRV_Handle)hEdma
#endif
#ifdef infraDMA  
                                ,(Cppi_Handle)cppiHnd,localRegion, remoteRegion
#endif
            )) {
        printf ("Failed to pass token %08x from remote to local\n", token);
        exit(1);
      }
    }

    printf ("Link Speed is %d * %s Gbps\n", 
                   hyplnk_EXAMPLE_MAX_LANES, hyplnk_EXAMPLE_LINK_SPEED);
#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
    printf("\n=== HyperLink results using CPU transfers ===\n");
    printf ("Passed %d tokens round trip (read+write through hyplnk) in %d Mcycles\n", 
                   hyplnk_EXAMPLE_NTOKENS, (unsigned int)(totalCPUTime/1000000));
    printf ("Approximately %d cycles per round-trip\n", (unsigned int)(totalCPUTime/hyplnk_EXAMPLE_NTOKENS));
    printf ("=== this is not an optimized example ===\n");
#endif

#ifdef hyplnk_EXAMPLE_TEST_DMA_BLOCK_XFER
    printf("\n=== HyperLink results using DMA transfers ===\n");
    printf ("Passed %d tokens round trip (read+write through hyplnk) in %d Mcycles\n", 
                   hyplnk_EXAMPLE_NTOKENS, (unsigned int)(totalDMATime/1000000));
    printf ("Approximately %d cycles per round-trip\n", (unsigned int)(totalDMATime/hyplnk_EXAMPLE_NTOKENS));
    printf ("=== this is not an optimized example ===\n");
#endif
#ifdef hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    printf("\n=== HyperLink results using INFRA transfers ===\n");
    printf ("Passed %d tokens round trip (read+write through hyplnk) in %d Mcycles\n", 
                   hyplnk_EXAMPLE_NTOKENS, (unsigned int)(totalINFRATime/1000000));
    printf ("Approximately %d cycles per round-trip\n", (unsigned int)(totalINFRATime/hyplnk_EXAMPLE_NTOKENS));
    printf ("=== this is not an optimized example ===\n");
#endif
    printf ("Checking statistics\n");
    hyplnkExampleCheckOneStat (hyplnk_LOCATION_LOCAL, "after passing tokens", 0);
    hyplnkExampleCheckOneStat (hyplnk_LOCATION_REMOTE, "after passing tokens", 0);
  }

    /*Deinitialize EDMA and InfraDMA handles*/
#ifdef EDMA
    edmaDeinit(hEdma);
#endif
#ifdef     hyplnk_EXAMPLE_TEST_INFRA_BLOCK_XFER
    InfraDMADeinit(cppiHnd);
#endif
    hyplnkReset(hyplnk_EXAMPLE_PORT);
    hyplnk_memRelease((void *)dataBufPtr, sizeof(dataBuffer));

    /* #if just suppresses compiler warning when set to run forever */
#if hyplnk_EXAMPLE_NITERS > 0
    printf("Hyperlink LLD Example Completed Successfully!");
#endif
    return 0;
}
