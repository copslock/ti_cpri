/* --COPYRIGHT--,BSD
 * Copyright (c) 2011 - 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include <xdc/std.h>

/* XDC.RUNTIME module Headers */
#include <xdc/runtime/Types.h>

/* IPC module Headers */
#include <ti/ipc/MessageQ.h>

/* CSL modules */
#include <ti/csl/csl_chip.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VERBOSE_MODE 0

/* Message allocation/free Defines */
#define MSG_FREE_FAILED (-1)
#define MSG_FREE_GOOD (0)
#define MSG_PREALLOC_GOOD    (0)

/* Core Synchronization Defines */
#define CORE_SYNC_NULL_CORE (-1)

#define CORE_SYNC_TX_SIDE_READY (2)
#define CORE_SYNC_RX_SIDE_READY (3)

#define SEND_MESSAGE_LAST_MESSAGE (4)

/* Convert cycles to microseconds */
#define CYCLES_TO_US(CYCLES, CPUFREQ) \
    ((UInt32)((((Double)CYCLES * 1000000))/(Double)CPUFREQ))

#define CYCLES_TO_US_DB(CYCLES, CPUFREQ) \
    ((Double)((((Double)CYCLES * 1000000))/(Double)CPUFREQ))
    
/* Convert cycles to nanoseconds */
#define CYCLES_TO_NS(CYCLES, CPUFREQ) \
    ((UInt32)((((Double)CYCLES * 1000000000))/(Double)CPUFREQ))

/* Convert Timestamp64 to UInt64 */
#define TIMESTAMP64_TO_UINT64(ts64hi, ts64lo) \
    ((((UInt64) ts64hi) << 32) | ((UInt64) ts64lo))

typedef struct Statistics 
{
    UInt numVals;
    Double stddev;
    UInt32 max;
    UInt32 min;
    UInt maxIndex;
    UInt minIndex;
    Double mean;
} Statistics;

#define MESSAGE_SIZE_IN_BYTES (48)
typedef struct TstMsg 
{
  MessageQ_MsgHeader header;      /* 32 bytes */
  Int src;
  Int flags;
  Int numMsgs; 
  Int seqNum;
} TstMsg;

inline UInt32 l2_global_address (UInt32 addr)
{
  UInt32 coreNum;

  if (addr & 0xF0000000)
  {
    /* Address is already global, return */
    return (addr);
  }

  /* Get the core number. */
  coreNum = CSL_chipReadReg (CSL_CHIP_DNUM); 

  /* Compute the global address. */
  return (addr + (0x10000000 + (coreNum * 0x1000000)));
}

int32_t Osal_dataBufferInitMemory(uint32_t dataBufferSize);
 
Void getStats(UInt32 *values, UInt numVals, Statistics *stats);
Void attachAll(UInt numCores);
Void detachAll(UInt numCores);

/* Common benchmark functions */
Void setLocalId (Void);
Int allocateMessages (Uint32 numMsgs, UInt16 heapId, TstMsg **msgPtrArray);
Int freeMessages (Uint32 numMsgs, TstMsg **msgPtrArray);
void syncSendCore (Int16 *syncCoresArrayPtr, 
                                     MessageQ_Handle coreRcvMsgQ, 
                                     MessageQ_QueueId *remoteCoreQIdArrayPtr, 
                                     TstMsg **syncMsgPtrArray,
                                     Bool freeRxMsgs);
void syncReceiveCore (Int16 *syncCoresArrayPtr, 
                                     MessageQ_Handle coreRcvMsgQ, 
                                     MessageQ_QueueId *remoteCoreQIdArrayPtr);
UInt32 sendMessages (UInt16 numberOfReceivingCores,
                                         UInt32 *numMsgsToSendPerCoreArrayPtr,
                                         MessageQ_QueueId *remoteCoreQIdArrayPtr, 
                                         TstMsg **msgPtrArray);
UInt32 receiveMessages(Int16 *sendCoresArrayPtr, MessageQ_Handle coreRcvMsgQ,
                                            TstMsg **rxMsgPtrArray, UInt32 uSecDelayPerMsg);
UInt64 getStartTime64();
UInt64 getExecutionTime64(UInt64 startTime,  UInt64 tsReadAdjust);
void calculateThroughput (UInt32 numMsgsRx, UInt64 executionTime, Types_FreqHz cpuFrequency);
 
#ifdef __cplusplus
}
#endif

