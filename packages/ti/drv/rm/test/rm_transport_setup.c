/*
 *   rm_transport_setup.c
 *
 *   RM test application IPC transport setup code
 *
 *  ============================================================================
 *
 * Copyright (c) 2013, Texas Instruments Incorporated
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
 *
 */

/* XDC includes */ 
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>

/* IPC includes */ 
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/HeapBufMP.h>

/* BIOS Includes */
#include <ti/sysbios/knl/Task.h>

/* RM Includes */
#include <ti/drv/rm/rm_transport.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>

#define MAX_TEST_CORES              4

/* IPC MessageQ heap name */
#define MSGQ_HEAP_NAME              "msgQHeapBuf"
/* IPC MessageQ heap ID */
#define MSGQ_HEAP_ID                0

/* RM packet heap name */
#define RM_PKT_HEAP_NAME            "rmHeapBuf"

/* IPC MessageQ RM packet encapsulation structure */
typedef struct {
    /* IPC MessageQ header (must be first element in structure) */
    MessageQ_MsgHeader  msgQHeader;
    /* Pointer to RM packet */
    Rm_Packet          *rmPkt;
} MsgQ_RmPacket;

/* RM registered transport map.  Maps a remote RM instance's transport
 * handle to the receive MsgQ that packets from this RM instance will be
 * received on. */
typedef struct {
    /* Registered destintation RM transport handle */
    Rm_TransportHandle transportHandle;
    /* MessageQ receive queue tied to the transport handle */
    MessageQ_Handle    receiveMsgQ;
} Transport_MapEntry;

/************************ GLOBAL VARIABLES ********************/
/* Numer of cores used by the test */
uint32_t            testCores = 0;
/* Test application's initialization core */
uint32_t            initCore = 0;
/* Application test task pointer */
Task_FuncPtr        appTestTask = NULL;
/* Receive task handle */
Task_Handle         rcvTask = NULL;

/* Handle for heap that RM packets will be allocated from */
HeapBufMP_Handle    rmPktHeapHandle = NULL;

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
Transport_MapEntry  rmTransportMap[MAX_TEST_CORES];

/************************ EXTERN VARIABLES ********************/
extern Rm_Handle rmHandle;

extern void Osal_rmBeginMemAccess(void *ptr, uint32_t size);
extern void Osal_rmEndMemAccess(void *ptr, uint32_t size);

/*************************** FUNCTIONS ************************/

Rm_Packet *rmTransPktAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet     *rmPkt = NULL;
    MsgQ_RmPacket *rmMsg = NULL;
    uint32_t       corenum = CSL_chipReadReg(CSL_CHIP_DNUM);      

    /* Allocate a messageQ message for containing the RM packet */
    rmMsg = (MsgQ_RmPacket *)MessageQ_alloc(MSGQ_HEAP_ID, sizeof(MsgQ_RmPacket));
    if (rmMsg == NULL) {
        System_printf("Error Core %d : MessageQ_alloc failed to allocate RM packet\n", corenum);
        *pktHandle = NULL;
        return(NULL);
    }
    else {
        /* Create and attach RM packet to MessageQ message.  All transports will allocate from the same heap */
        rmPkt = HeapBufMP_alloc(rmPktHeapHandle, pktSize, 0);
        rmPkt->pktLenBytes = pktSize;
        Osal_rmEndMemAccess((void *)rmPkt, rmPkt->pktLenBytes);
        rmMsg->rmPkt = rmPkt;
        *pktHandle = (Rm_PacketHandle)rmMsg;
    }
    return (rmPkt);
}

void rmTransPktFree (MessageQ_Msg rmMsgQMsg, Rm_Packet *pkt)
{
    uint32_t pktSize = pkt->pktLenBytes;
    uint32_t corenum = CSL_chipReadReg(CSL_CHIP_DNUM);      
    int32_t  status;

    /* All transports will free rmPkts to the same heap */
    HeapBufMP_free(rmPktHeapHandle, pkt, pktSize);

    status = MessageQ_free(rmMsgQMsg);
    if (status < 0) { 
        System_printf("Error Core %d : MessageQ_free of RM packet\n", corenum);
    }     
}

int32_t rmTransPktSend (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    MessageQ_QueueId  remoteQueueId = (MessageQ_QueueId)appTransport;
    MsgQ_RmPacket    *rmMsg = pktHandle;
    uint32_t          corenum = CSL_chipReadReg(CSL_CHIP_DNUM);      
    int32_t           status;    

    /* Write back data that was written by RM after alloc */
    Osal_rmEndMemAccess((void *)rmMsg->rmPkt, rmMsg->rmPkt->pktLenBytes);

    /* Send the message to the remote side */
    status = MessageQ_put(remoteQueueId, (MessageQ_Msg)rmMsg);
    if (status < 0) {
        rmTransPktFree((MessageQ_Msg)rmMsg, rmMsg->rmPkt);
        System_printf("Error Core %d : MessageQ_put sending RM packet : %d\n", corenum, status);
    }
    return (status);
}

void rmTransPktRcv (uint32_t transportMapEntry)
{
    MessageQ_Handle  receiveQ;
    int32_t          numPkts;
    MessageQ_Msg     rmMsg = NULL;
    Rm_Packet       *rmPkt = NULL;
    uint32_t         corenum = CSL_chipReadReg(CSL_CHIP_DNUM);  
    int32_t          status;
    uint32_t         i;  

    /* Check if any packets available */
    receiveQ = rmTransportMap[transportMapEntry].receiveMsgQ;
    numPkts = (int32_t) MessageQ_count(receiveQ);

    /* Process all available packets */
    for (i = 0; i < numPkts; i++) {
        status = (int32_t) MessageQ_get(receiveQ, &rmMsg, MessageQ_FOREVER);
        if (rmMsg == NULL) {
            System_printf("Error Core %d : NULL msg returned by MessageQ\n", corenum);
        }

        /* Extract the Rm_Packet from the RM msg */
        rmPkt = ((MsgQ_RmPacket *)rmMsg)->rmPkt;
        Osal_rmBeginMemAccess((void *) rmPkt, rmPkt->pktLenBytes);

        /* Provide packet to RM for processing */
        if (status = Rm_receivePacket(rmTransportMap[transportMapEntry].transportHandle, rmPkt)) {
            System_printf("Error Core %d : Receiving RM packet : %d\n", corenum, status);
        }

        /* Free RM packet buffer and messageQ message */
        rmTransPktFree(rmMsg, rmPkt);
    }
}

void rmReceiveTsk(UArg arg0, UArg arg1)
{
    uint32_t corenum = CSL_chipReadReg(CSL_CHIP_DNUM);
    int      i;

    while(1) {
        /* Check the receive MessageQs for received RM packets */
        for (i = 0; i < testCores; i++) {
            if (((corenum == initCore) && (i != initCore)) ||
                ((corenum != initCore) && (i == initCore))) {
                rmTransPktRcv(i);
            }
        }
        /* Sleep for 1ms so that usageTsk can run */
        Task_sleep(1);
    }
}

void configRmTransportTsk(UArg arg0, UArg arg1)
{
    int32_t           result;
    uint32_t          corenum = 0;
    uint32_t          i;
    HeapBufMP_Params  heapBufParams;
    HeapBufMP_Handle  msgQHeapHandle;
    int               status;
    char              name[RM_NAME_MAX_CHARS];
    MessageQ_QueueId  remoteRcvQId;
    Rm_TransportCfg   rmTransportCfg;
    Task_Params       taskParams;    
    
    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Initialize the transport map */
    for (i = 0; i < testCores; i++) {
        rmTransportMap[i].transportHandle = NULL;
    } 

    /* Configure IPC as the application transport used by RM to communicate between instances
     * on different cores. */
    if (corenum == initCore) {
        /* Create the heap that will be used to allocate RM messages. This
         * heap is a multi-processor heap.  It will be shared amongst
         * all RM instances. */     
        HeapBufMP_Params_init(&heapBufParams);
        heapBufParams.regionId       = 0;
        heapBufParams.name           = RM_PKT_HEAP_NAME;
        heapBufParams.numBlocks      = 16;
        heapBufParams.blockSize      = sizeof(Rm_Packet);
        rmPktHeapHandle = HeapBufMP_create(&heapBufParams);
        if (rmPktHeapHandle == NULL) {
            System_printf("Error Core %d : RM packet HeapBufMP_create failed \n", corenum);
        }
        else {
            System_printf("Core %d : Created RM packet heap\n", corenum);
        }

        /* Create the heap that will be used to allocate messageQ messages. */     
        HeapBufMP_Params_init(&heapBufParams);
        heapBufParams.regionId       = 0;
        heapBufParams.name           = MSGQ_HEAP_NAME;
        heapBufParams.numBlocks      = 16;
        heapBufParams.blockSize      = sizeof(MsgQ_RmPacket);
        msgQHeapHandle = HeapBufMP_create(&heapBufParams);
        if (msgQHeapHandle == NULL) {
            System_printf("Error Core %d : IPC MessageQ HeapBufMP_create failed \n", corenum);
        }
        else {
            System_printf("Core %d : Created IPC MessageQ heap\n", corenum);
        }
    }
    else {
        /* Open the heaps created by initCore core. Loop until opened. */
        do {
            status = HeapBufMP_open(RM_PKT_HEAP_NAME, &rmPktHeapHandle);
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : Opened RM packet heap\n", corenum);
        
        do {
            status = HeapBufMP_open(MSGQ_HEAP_NAME, &msgQHeapHandle);
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : Opened IPC MessageQ heap\n", corenum);
    }

    /* Register the MessageQ heap with MessageQ */
    MessageQ_registerHeap(msgQHeapHandle, MSGQ_HEAP_ID); 

    /* Setup the MessageQs required for RM instances to communicate */
    for (i = 0; i < testCores; i++) {
        if ((corenum == initCore) && (i != initCore)) {
            /* RM Server core transport registration (Should run testCores-1 times) */
            
            /* Create a queue to receive messages from a RM client core */
            System_sprintf(name, "serverRcvQForClient%d", i);
            rmTransportMap[i].receiveMsgQ = MessageQ_create(name, NULL);
            if (rmTransportMap[i].receiveMsgQ == NULL) {
                System_printf("Error Core %d : Failed to create receive Q for Client%d\n", corenum, i);
            }
            else {
                System_printf("Core %d : Created receive Q for Client%d\n", corenum, i);
            }

            /* Open the RM Client core's receive MessageQ */
            System_sprintf(name, "client%dRcvQForServer", i);
            do {
                status = MessageQ_open(name, &remoteRcvQId); 
                /* 
                 *  Sleep for 1 clock tick to avoid inundating remote processor
                 *  with interrupts if open failed
                 */
                if (status < 0) { 
                    Task_sleep(1);
                }
            } while (status < 0);
            System_printf("Core %d : Opened Client%d's receive Q for Server\n", corenum, i);

            /* Register Client receive Q with Server */
            rmTransportCfg.rmHandle = rmHandle;
            rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) remoteRcvQId;
            rmTransportCfg.remoteInstType = Rm_instType_CLIENT;
            rmTransportCfg.transportCallouts.rmAllocPkt = rmTransPktAlloc;
            rmTransportCfg.transportCallouts.rmSendPkt = rmTransPktSend;
            rmTransportMap[i].transportHandle = Rm_transportRegister(&rmTransportCfg, &result);  
        }
        else if ((corenum != initCore) && (i == initCore)) {
            /* RM Client core transport registration (Should only run once) */
            
            /* Create queue to receive messages from Server */
            System_sprintf(name, "client%dRcvQForServer", corenum);
            rmTransportMap[i].receiveMsgQ = MessageQ_create(name, NULL);
            if (rmTransportMap[i].receiveMsgQ == NULL) {
                System_printf("Error Core %d : Failed to create receive Q for Server\n", corenum);
            }
            else {
                System_printf("Core %d : Created receive Q for Server\n", corenum);
            }
            
            /* Open the RM Server core's receive MessageQ for this core */
            System_sprintf(name, "serverRcvQForClient%d", corenum);
            do {
                status = MessageQ_open(name, &remoteRcvQId); 
                /* 
                 *  Sleep for 1 clock tick to avoid inundating remote processor
                 *  with interrupts if open failed
                 */
                if (status < 0) { 
                    Task_sleep(1);
                }
            } while (status < 0);
            System_printf("Core %d : Opened Server's receive Q\n", corenum);
            
            /* Register the Server receive Q */
            rmTransportCfg.rmHandle = rmHandle;
            rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) remoteRcvQId;
            rmTransportCfg.remoteInstType = Rm_instType_SERVER;
            rmTransportCfg.transportCallouts.rmAllocPkt = rmTransPktAlloc;
            rmTransportCfg.transportCallouts.rmSendPkt = rmTransPktSend;
            rmTransportMap[i].transportHandle = Rm_transportRegister(&rmTransportCfg, &result);   
        }
    }

    /* Create the RM receive task.  Receive task has priority of 2 so that it pre-empts the appTestTask */
    Task_Params_init (&taskParams);
    taskParams.priority = 2;
    rcvTask = Task_create (rmReceiveTsk, &taskParams, NULL);   

    /* Create the application test task */
    Task_Params_init (&taskParams);
    taskParams.priority = 1;
    Task_create (appTestTask, &taskParams, NULL);     
}

int setupRmTransConfig(uint32_t numTestCores, uint32_t systemInitCore, Task_FuncPtr testTask)
{
    Task_Params taskParams;

    if (numTestCores > MAX_TEST_CORES) {
        return (-1);
    }

    /* Store application core parameters */
    testCores = numTestCores;
    initCore = systemInitCore;

    /* Store the applications testTask for creation after the configRmTransportTsk
     * runs */
    appTestTask = testTask;

    /* Create the RM transport configuration task */
    Task_Params_init (&taskParams);
    taskParams.priority = 1;
    Task_create (configRmTransportTsk, &taskParams, NULL);

    return(0);
}

int deleteRmTrans(void)
{
    uint32_t corenum = 0;
    uint32_t i;
    int32_t  rmResult = RM_OK;
    
    /* Get the core number. */
    corenum = CSL_chipReadReg(CSL_CHIP_DNUM);

    /* Delete the RM receive task */
    System_printf("Core %d: Deleting RM receive task...\n", corenum);
    if (rcvTask) {
        Task_delete(&rcvTask);
        rcvTask = NULL;
    }

    /* Cleanup transport objects */
    for (i = 0; i < testCores; i++) {
         if ((corenum == initCore) && (i != initCore)) {
             if (rmResult = Rm_transportUnregister(rmTransportMap[i].transportHandle) < 0) {
                break;
             }
         }
         else if ((corenum != initCore) && (i == initCore)) {
             if (rmResult = Rm_transportUnregister(rmTransportMap[i].transportHandle) < 0) {
                break;
             }
         }
    }

    return (rmResult);
}

