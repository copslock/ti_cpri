/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
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

/* XDC module Headers */
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/System.h>
#include <xdc/std.h>

/* IPC modules */
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/Notify.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MultiProc.h>

/* BIOS modules */
#include <ti/sysbios/family/c66/Cache.h>

/* PDK module Headers */
#include <ti/platform/platform.h>

#include <math.h>
#include "bench_common.h"

/*
 *  ======== getStats ========
 */
Void getStats(UInt32 *values, UInt numVals, Statistics *stats ) 
{
    UInt i;
    Double sum = 0;
    stats->numVals = numVals;
    stats->min = values[0];
    stats->minIndex = 0;
    stats->max = values[0];
    stats->maxIndex = 0;
    for (i = 0; i < numVals; i++) {
        sum += (Double)values[i];
        if (values[i] < stats->min) {
            stats->min = values[i];
            stats->minIndex = i;
        }
        else if (values[i] > stats->max) {
            stats->max = values[i];
            stats->maxIndex = i;
        }
    }
    stats->mean = sum / (Double)numVals;
    sum = 0;
    for (i = 0; i < numVals; i++) {
        sum += pow((Double)values[i] - stats->mean, 2);
    }
    stats->stddev = sqrt(sum / (Double)numVals);
}


/*
 *  ======== attachAll ========
 */
Void attachAll(UInt numCores) 
{
    SharedRegion_Entry entry;
    Int status;
    UInt i;
    UInt16 clusterBaseId = MultiProc_getBaseIdOfCluster();
    
    /* Call Ipc_start() */
    status = Ipc_start();
    if (status < 0) {
        System_abort("Ipc_start failed!\n");
    }
    
    /* get region 0 information */
    SharedRegion_getEntry(0, &entry);

    /* if entry is not valid then return */
    if (entry.isValid == FALSE) {
        return;
    }
    
    /* Must attach to owner first to get default GateMP */
    if (MultiProc_self() != entry.ownerProcId) {
        do {
            status = Ipc_attach(entry.ownerProcId);
        } while (status < 0);
    }

    /* Loop to attach to all other processors */
    for (i = clusterBaseId; i < (clusterBaseId+numCores); i++)
    {
        if ((i == MultiProc_self()) ||
             (i == entry.ownerProcId)) {
            
            continue;
        }

        if (Notify_numIntLines(i) == 0) {
            continue;
        }

        /* call Ipc_attach for every remote processor */
        do {
            status = Ipc_attach(i);
        } while (status < 0);
    }
}

/*
 *  ======== attachAll ========
 */

Void detachAll(UInt numCores)
{
    SharedRegion_Entry entry;
    Int status;
    UInt i;
    UInt16 clusterBaseId = MultiProc_getBaseIdOfCluster();

    /* get region 0 information */
    SharedRegion_getEntry(0, &entry);

    /* if entry is not valid then return */
    if (entry.isValid == FALSE) {
        return;
    }

    /* Loop to attach to all other processors */
    for (i = clusterBaseId; i < (clusterBaseId + numCores); i++)
    {
        if ((i == MultiProc_self()) || 
             (i == entry.ownerProcId)) {
            continue;
        }

        if (Notify_numIntLines(i) == 0) {
            continue;
        }

        /* call Ipc_attach for every remote processor */
        do {
            status = Ipc_detach(i);
        } while (status < 0);
    }

    /* Must detach from shared region owner last */
    if (MultiProc_self() != entry.ownerProcId) {
        do {
            status = Ipc_detach(entry.ownerProcId);
        } while (status < 0);
    }
}

Int allocateMessages (Uint32 numMsgs, UInt16 heapId, TstMsg **msgPtrArray)
{
  Int i;
  TstMsg *allocatedMsg;

#if VERBOSE_MODE
  System_printf ("Core %d: Preallocating %d MessageQ messages.\n", 
                        CSL_chipReadReg (CSL_CHIP_DNUM), numMsgs);
#endif
  
  /* Allocate the number of messages specified */
  for (i = 0; i < numMsgs; i++)
  {
    allocatedMsg = (TstMsg *)MessageQ_alloc(heapId, sizeof(TstMsg));
    if (allocatedMsg)
    {
      /* Initialize and store the allocated message as long as it's not NULL */
      allocatedMsg->src = CSL_chipReadReg (CSL_CHIP_DNUM);
      msgPtrArray[i] = allocatedMsg;
    }
    else
    {
      /* Return the number of messages allocated prior to error */
      return i;
    }
  }

#if VERBOSE_MODE
  System_printf ("Core %d: Allocated %d MessageQ messages.\n", 
                        CSL_chipReadReg (CSL_CHIP_DNUM), numMsgs);
#endif

  return MSG_PREALLOC_GOOD;
}

Int freeMessages (Uint32 numMsgs, TstMsg **msgPtrArray)
{
  Int i;
  Int status = 0;

#if VERBOSE_MODE
  System_printf ("Core %d: Freeing %d MessageQ messages.\n", 
                         CSL_chipReadReg (CSL_CHIP_DNUM), numMsgs);
#endif
  
  /* Free the number of messages specified */
  for (i = 0; i < numMsgs; i++)
  {
    status = MessageQ_free((MessageQ_Msg) msgPtrArray[i]);
    if (status < 0)
    {
#if VERBOSE_MODE
      System_printf("Core %d: Freed %d messages prior to failing with error %d\n", 
                           CSL_chipReadReg (CSL_CHIP_DNUM), (i+1), status);
#endif
      return MSG_FREE_FAILED;
    }
  }

#if VERBOSE_MODE
  System_printf ("Core %d: Freed %d MessageQ messages.\n", 
                        CSL_chipReadReg (CSL_CHIP_DNUM), numMsgs);
#endif

  return MSG_FREE_GOOD;
}

void syncSendCore (Int16 *syncCoresArrayPtr, 
                                     MessageQ_Handle coreRcvMsgQ, 
                                     MessageQ_QueueId *remoteCoreQIdArrayPtr, 
                                     TstMsg **syncMsgPtrArray,
                                     Bool freeRxMsgs)
{
  Int status;
  Int i = 0;
  TstMsg *txSyncMsg = NULL;
  TstMsg *rxSyncMsg = NULL;
  UInt32 syncReceivedMask = 0; /* Support for up to 32 cores */
  
  /* Send a sync message to each core */
  while (syncCoresArrayPtr[i] != CORE_SYNC_NULL_CORE)
  {
    /* Get a new sync message */
    txSyncMsg = syncMsgPtrArray[i];
    txSyncMsg->src = CSL_chipReadReg (CSL_CHIP_DNUM);
    txSyncMsg->flags = CORE_SYNC_TX_SIDE_READY;

    /* Send the TX side sync message */
#if VERBOSE_MODE
    System_printf ("Core %d: Sending sync message to Core %d\n", 
                           CSL_chipReadReg (CSL_CHIP_DNUM), syncCoresArrayPtr[i]);
#endif
    status = MessageQ_put(remoteCoreQIdArrayPtr[i], (MessageQ_Msg) txSyncMsg);
    if (status < 0) 
    {
      System_abort("MessageQ_put failed\n");
    }

    /* Add to mask the core that sync was sent to */
    syncReceivedMask |= (0x1 << syncCoresArrayPtr[i]);
    i++;
  }
  
  /* Wait for response from remote cores */
  do
  {
    /* Get the sync message */
    status = MessageQ_get(coreRcvMsgQ, (MessageQ_Msg*)&rxSyncMsg, MessageQ_FOREVER);
    if (status < 0) 
    {
      System_abort("MessageQ_get failed\n");
    }

    /* Check if it is remote core informing it is sync'd */
    if (rxSyncMsg->flags == CORE_SYNC_RX_SIDE_READY)
    {
      /* Clear the core from the received mask */
      if (syncReceivedMask & (0x1 << rxSyncMsg->src))
      {
#if VERBOSE_MODE        
        System_printf ("Core %d: Received sync message back from Core %d\n", 
                              CSL_chipReadReg (CSL_CHIP_DNUM), rxSyncMsg->src);
#endif
        syncReceivedMask &= ~(0x1 << rxSyncMsg->src);
      }
    }

    if (freeRxMsgs)
    {
      /* Free message if requested.  This is used for certain transport implementations
        * that allocate new messages for each newly received packet */
      MessageQ_free ((MessageQ_Msg) rxSyncMsg);
    }
  } while(syncReceivedMask != 0);
}

void syncReceiveCore (Int16 *syncCoresArrayPtr, 
                                           MessageQ_Handle coreRcvMsgQ, 
                                           MessageQ_QueueId *remoteCoreQIdArrayPtr)
{
  Int status;
  Int i = 0;
  Int16 coreIndex = 0;
  TstMsg *txSyncMsg = NULL;
  TstMsg *rxSyncMsg = NULL;
  UInt32 syncReceivedMask = 0; /* Support for up to 32 cores */

  /* Initialize the receive core bitfield for verifying all sync messages were received */
  while (syncCoresArrayPtr[i] != CORE_SYNC_NULL_CORE)
  {
    /* Add to mask the core that sync should be received from */
    syncReceivedMask |= (0x1 << syncCoresArrayPtr[i]);
    i++;
  }

  /* Receive sync messages as long as sync's have not been received from all specified cores */
  while (syncReceivedMask != 0)
  {
    /* Wait for sync message from TX core */
    do
    {
      /* Get the sync message */
      status = MessageQ_get(coreRcvMsgQ, (MessageQ_Msg*)&rxSyncMsg, MessageQ_FOREVER);
      if (status < 0)
      {
        System_abort("MessageQ_get failed\n");
      }
      
      /* Check if it is sending core wanting to sync */
      if (rxSyncMsg->flags == CORE_SYNC_TX_SIDE_READY)
      {
        /* Clear the core from the received mask */
        if (syncReceivedMask & (0x1 << rxSyncMsg->src))
        {
#if VERBOSE_MODE        
          System_printf ("Core %d: received sync message from Core %d\n", 
                                 CSL_chipReadReg (CSL_CHIP_DNUM), rxSyncMsg->src);
#endif
          syncReceivedMask &= ~(0x1 << rxSyncMsg->src);

          /* Store index of core for send back of sync */
          i = 0;
          while (syncCoresArrayPtr[i] != CORE_SYNC_NULL_CORE)
          {
            if (syncCoresArrayPtr[i] == rxSyncMsg->src)
            {
              coreIndex = i;
              break;
            }
            i++;
          } 
        }
      }    
    } while(rxSyncMsg->flags != CORE_SYNC_TX_SIDE_READY);
    
    txSyncMsg = rxSyncMsg;
    txSyncMsg->src = CSL_chipReadReg (CSL_CHIP_DNUM);
    txSyncMsg->flags = CORE_SYNC_RX_SIDE_READY;
    
    /* Send the RX side sync message */
#if VERBOSE_MODE        
    System_printf ("Core %d: Sending sync message back to Core %d\n", 
                           CSL_chipReadReg (CSL_CHIP_DNUM), syncCoresArrayPtr[i]);
#endif    
    status = MessageQ_put(remoteCoreQIdArrayPtr[coreIndex], (MessageQ_Msg)txSyncMsg);
    if (status < 0)
    {
      System_abort("MessageQ_put failed\n");
    }
  }
}

UInt32 sendMessages (UInt16 numberOfReceivingCores,
                                         UInt32 *numMsgsToSendPerCoreArrayPtr,
                                         MessageQ_QueueId *remoteCoreQIdArrayPtr, 
                                         TstMsg **msgPtrArray)
{
  UInt32 maxMsgs = 0;
  Int i, j;
  Int status;
  UInt32 msgIndex = 0;
  TstMsg *sndMsg;

  /* Find maximum number of messages to be sent */
  for (i = 0; i < numberOfReceivingCores; i++)
  {
    if (numMsgsToSendPerCoreArrayPtr[i] > maxMsgs)
    {
      maxMsgs = numMsgsToSendPerCoreArrayPtr[i];
    }
  }
  
  /* Send the specified number of messages to the specified cores.  
    * The last message sent will have a flag signifying to receiving core that
    * all messages have been sent. */
  for (i = 0; i < maxMsgs; i++)
  {
    for (j = 0; j < numberOfReceivingCores; j++)
    {
      /* Send a message if haven't gone over max to send to this destination core */
      if (i < numMsgsToSendPerCoreArrayPtr[j])
      {
        /* Get a message */
        sndMsg = msgPtrArray[msgIndex];
        sndMsg->src = CSL_chipReadReg (CSL_CHIP_DNUM);
    
        if (i == (numMsgsToSendPerCoreArrayPtr[j] - 1))
        {
          /* If it's the last message for this core inform the remote core */
          sndMsg->flags = SEND_MESSAGE_LAST_MESSAGE;
#if VERBOSE_MODE
          System_printf("Core %d: Sent %d messages to Queue ID: %d\n", 
                                CSL_chipReadReg (CSL_CHIP_DNUM), (i+1), remoteCoreQIdArrayPtr[j]);
#endif
        }
        else
        {
          sndMsg->flags = 0;
        }

        /* Send message to remote core */
        status = MessageQ_put(remoteCoreQIdArrayPtr[j], (MessageQ_Msg)sndMsg);
        if (status < 0)
        {
          System_abort("MessageQ_put failed\n");
        }
        
        msgIndex++;
      }
    }
  }

  return msgIndex;
}

UInt32 receiveMessages(Int16 *sendCoresArrayPtr, MessageQ_Handle coreRcvMsgQ,
                                             TstMsg **rxMsgPtrArray, UInt32 uSecDelayPerMsg)
{
  UInt32 numberReceived = 0;
  Int status;
  Int i = 0;
  TstMsg *rxMsg;
  UInt32 senderCoresMask = 0; /* Support for up to 32 cores */

  /* Initialize the sender core bitfield for to track whether all messages have been received from all
    * senders */
  while (sendCoresArrayPtr[i] != CORE_SYNC_NULL_CORE)
  {
    /* Add to mask the core that sync should be received from */
    senderCoresMask |= (0x1 << sendCoresArrayPtr[i]);
    i++;
  }
  
  /* Receive messages until all senders have completed */
  while (senderCoresMask) 
  {
    /* Get a message */
    status = MessageQ_get(coreRcvMsgQ, (MessageQ_Msg*)&rxMsg, MessageQ_FOREVER);
    if (status < 0) 
    {
        System_abort("MessageQ_get failed\n");
    }

    /* Make sure the message received was from an expected sender */
    if (senderCoresMask & (0x1 << rxMsg->src))
    {
      if (rxMsg->flags == SEND_MESSAGE_LAST_MESSAGE)
      {
#if VERBOSE_MODE        
        System_printf ("Core %d: Received all messages from Core %d\n", 
                               CSL_chipReadReg (CSL_CHIP_DNUM), rxMsg->src);
#endif
        senderCoresMask &= ~(0x1 << rxMsg->src);
      }

      /* Time system should wait in while loop per message.  This can be used to simulate application processing
        * for each message received */
      if (uSecDelayPerMsg != 0)
      {
        platform_delay(uSecDelayPerMsg);
      }

      /* Store the received messageQ msg for use by the application */
      if (rxMsgPtrArray != NULL)
      {
        rxMsgPtrArray[numberReceived] = rxMsg;
      }

#if VERBOSE_MODE
      System_printf("Core %d: Received message from Core %d\n", 
                            CSL_chipReadReg (CSL_CHIP_DNUM), rxMsg->src);
#endif      
      
      numberReceived++;
    }
    else
    {
#if VERBOSE_MODE
      System_printf("Core %d: Received unexpected message from Core %d\n", 
                            CSL_chipReadReg (CSL_CHIP_DNUM), rxMsg->src);
#endif
    }
  }

#if VERBOSE_MODE
  System_printf("Core %d: Received a total of %d messages\n", 
                        CSL_chipReadReg (CSL_CHIP_DNUM), numberReceived);
#endif

  return numberReceived;
}

UInt64 getStartTime64()
{
  Types_Timestamp64 time64;
  
  Timestamp_get64(&time64);
  return TIMESTAMP64_TO_UINT64(time64.hi,time64.lo);
}

UInt64 getExecutionTime64(UInt64 startTime, UInt64 tsReadAdjust)
{
  Types_Timestamp64 time64;
  
  Timestamp_get64(&time64);
  return (TIMESTAMP64_TO_UINT64(time64.hi,time64.lo) - startTime - tsReadAdjust);
}

void calculateThroughput (UInt32 numMsgsRx, UInt64 executionTime, 
                                                 Types_FreqHz cpuFrequency)
{
  UInt32 throughPut = 0;

  throughPut = ((UInt32)(1000000)*(double)numMsgsRx /
                          (double)CYCLES_TO_US(executionTime, cpuFrequency.lo));

  /* If timeLength starts going over 4.2 trillion cycles will have to figure out how to print 64 bit values.
    * for now just downcast timeLength to 32 bits before printing */
  System_printf("Core %d. msgs Received= %d time=%d (cycles - %d us). thrput=%d [msgs/s]\n",
                       CSL_chipReadReg (CSL_CHIP_DNUM),
                       numMsgsRx,
                       (UInt32) executionTime, 
                       (UInt32) CYCLES_TO_US(executionTime, cpuFrequency.lo),
                       throughPut);
  System_printf("cycles/msg = %d\n", (executionTime/numMsgsRx));
}
