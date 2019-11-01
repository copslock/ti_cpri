/*
 *   rm_test.c
 *
 *   Multicore Resource Manager test that uses IPC to an application
 *   requesting RM services from a RM Server, Client Delegate, and Client.
 *
 *  ============================================================================
 *
 * Copyright (c) 2012-2013, Texas Instruments Incorporated
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
 
/* Standard Includes */
#include <string.h>

/* XDC Includes */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* IPC Includes */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/GateMP.h>

/* BIOS Includes */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* CSL Includes */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>

/* RM Includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>
#include <ti/drv/rm/rm_osal.h>

/**********************************************************************
 ************************** RM Test Symbols ***************************
 **********************************************************************/

#define PRINT_USED_RESOURCES         0  /* Make 1 and rebuild project to print resources allocated in example */

#define SYSINIT                      0
#define NUM_CORES                    2

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* IPC MessageQ heap name */
#define MSGQ_HEAP_NAME               "msgQHeapBuf"
/* IPC MessageQ heap ID */
#define MSGQ_HEAP_ID                 0

/* RM packet heap name */
#define RM_PKT_HEAP_NAME             "rmHeapBuf"
/* GateMP names used to synchronize the RM test tasks */
#define RM_SERVER_GATE_NAME          "rmServerGate"
#define RM_CLIENT_GATE_NAME          "rmClientGate"

/* Application's core 0 registered RM transport indices */
#define SERVER_TO_CD_MAP_ENTRY       0
/* Application's core 1 registered RM transport indices */
#define CD_TO_SERVER_MAP_ENTRY       0
#define CD_TO_CLIENT_MAP_ENTRY       1
#define CLIENT_TO_CD_MAP_ENTRY       2
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES          3

/* Size of RM static allocation response queue.  Must be greater than number of APPROVED
 * static allocations */
#define MAX_STATIC_ALLOCATION_RESPS  5 

/* Size of RM service response queue */
#define MAX_QUEUED_SERVICE_RESPONSES 10

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)                   \
    if (resultVal != checkVal) {                                                 \
        char errorMsgToPrint[] = printMsg;                                       \
        System_printf("Error Core %d : %s : ", coreNum, rmInstName);             \
        System_printf("%s with error code : %d\n", errorMsgToPrint, resultVal);  \
        testErrors++;                                                            \
        System_abort("Test Failure\n");                                          \
    }

#define POSITIVE_PASS_CHECK(title, core, instName, resName, resStart, resLen, align, state, check)  \
    do {                                                                                            \
        int32_t start = resStart;                                                                   \
        int32_t alignment = align;                                                                  \
        char    titleMsg[] = title;                                                                 \
        System_printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        System_printf ("Core %d : %s\n", core, titleMsg);                                           \
        System_printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);               \
        System_printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                \
        if (start == RM_RESOURCE_BASE_UNSPECIFIED) {                                                \
            System_printf ("Core %d : - Start:         UNSPECIFIED                            -\n", \
                           core);                                                                   \
            System_printf ("Core %d : - Length:        %-16d                       -\n",            \
                           core, resLen);                                                           \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - Start:         %-16d                       -\n",            \
                           core, resStart);                                                         \
            System_printf ("Core %d : - End:           %-16d                       -\n", core,      \
                           (start + resLen - 1));                                                   \
        }                                                                                           \
        if (alignment == RM_RESOURCE_ALIGNMENT_UNSPECIFIED) {                                       \
            System_printf ("Core %d : - Alignment:     UNSPECIFIED                            -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - Alignment:     %-16d                       -\n",            \
                           core, alignment);                                                        \
        }                                                                                           \
        System_printf ("Core %d : -                                                       -\n",     \
                       core);                                                                       \
        if (state == check) {                                                                       \
            System_printf ("Core %d : - PASSED                                                -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - FAILED - Denial: %-6d                               -\n",   \
                           core, state);                                                            \
            testErrors++;                                                                           \
        }                                                                                           \
        System_printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        System_printf ("\n");                                                                       \
    } while(0);

#define NEGATIVE_PASS_CHECK(title, core, instName, resName, resStart, resLen, align, state, check)  \
    do {                                                                                            \
        int32_t start = resStart;                                                                   \
        int32_t alignment = align;                                                                  \
        char    titleMsg[] = title;                                                                 \
        System_printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        System_printf ("Core %d : %s\n", core, titleMsg);                                           \
        System_printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);               \
        System_printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                \
        if (start == RM_RESOURCE_BASE_UNSPECIFIED) {                                                \
            System_printf ("Core %d : - Start:         UNSPECIFIED                            -\n", \
                           core);                                                                   \
            System_printf ("Core %d : - Length:        %-16d                       -\n",            \
                           core, resLen);                                                           \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - Start:         %-16d                       -\n",            \
                           core, resStart);                                                         \
            System_printf ("Core %d : - End:           %-16d                       -\n", core,      \
                           (start + resLen - 1));                                                   \
        }                                                                                           \
        if (alignment == RM_RESOURCE_ALIGNMENT_UNSPECIFIED) {                                       \
            System_printf ("Core %d : - Alignment:     UNSPECIFIED                            -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - Alignment:     %-16d                       -\n",            \
                           core, alignment);                                                        \
        }                                                                                           \
        System_printf ("Core %d : -                                                       -\n",     \
                       core);                                                                       \
        if (state != check) {                                                                       \
            System_printf ("Core %d : - PASSED - Denial: %-6d                               -\n",   \
                           core, state);                                                            \
        }                                                                                           \
        else {                                                                                      \
            System_printf ("Core %d : - FAILED - Expected Denial                              -\n", \
                           core);                                                                   \
            testErrors++;                                                                           \
        }                                                                                           \
        System_printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        System_printf ("\n");                                                                       \
    } while(0);    

#define STATUS_PASS_CHECK(title, core, instName, resName, resStart, resLen, refCnt, allocCnt, state, check, expectRefCnt, expectAllocCnt) \
    do {                                                                                                        \
        int32_t start = resStart;                                                                               \
        char    titleMsg[] = title;                                                                             \
        System_printf ("Core %d : ---------------------------------------------------------\n",                 \
                       core);                                                                                   \
        System_printf ("Core %d : %s\n", core, titleMsg);                                                       \
        System_printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);                           \
        System_printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                            \
        System_printf ("Core %d : - Start:         %-16d                       -\n",                            \
                           core, resStart);                                                                     \
        System_printf ("Core %d : - End:           %-16d                       -\n", core,                      \
                           (start + resLen - 1));                                                               \
        System_printf ("Core %d : - Expected Owner Count: %-16d                -\n",                            \
                       core, expectRefCnt);                                                                     \
        System_printf ("Core %d : - Returned Owner Count: %-16d                -\n",                            \
                       core, refCnt);                                                                           \
        System_printf ("Core %d : - Expected Inst Allocation Count: %-16d      -\n",                            \
                       core, expectAllocCnt);                                                                   \
        System_printf ("Core %d : - Returned Inst Allocation Count: %-16d      -\n",                            \
                       core, allocCnt);                                                                         \
        System_printf ("Core %d : -                                                       -\n", core);          \
        if ((state == check) && (refCnt == expectRefCnt) && (allocCnt == expectAllocCnt)) {                     \
            System_printf ("Core %d : - PASSED                                                -\n", core);      \
        }                                                                                                       \
        else {                                                                                                  \
            if (refCnt != expectRefCnt) {                                                                       \
                System_printf ("Core %d : - FAILED - Owner Count Mismatch                         -\n",         \
                               core);                                                                           \
            }                                                                                                   \
            else if (allocCnt != expectAllocCnt) {                                                              \
                System_printf ("Core %d : - FAILED - Instance Allocation Count Mismatch           -\n",         \
                               core);                                                                           \
            }                                                                                                   \
            else {                                                                                              \
                System_printf ("Core %d : - FAILED - Denial: %-6d                               -\n",           \
                               core, state);                                                                    \
            }                                                                                                   \
            testErrors++;                                                                                       \
        }                                                                                                       \
        System_printf ("Core %d : ---------------------------------------------------------\n",                 \
                       core);                                                                                   \
        System_printf ("\n");                                                                                   \
    } while(0);

/**********************************************************************
 ********************** RM Test Data Structures ***********************
 **********************************************************************/

/* IPC MessageQ RM packet encapsulation structure */
typedef struct {
    /* IPC MessageQ header (must be first element in structure) */
    MessageQ_MsgHeader  msgQHeader;
    /* Pointer to RM packet */
    Rm_Packet          *rmPkt;
} MsgQ_RmPacket;

/* RM registered transport mapping structure */
typedef struct {
    /* Registered RM transport handle */
    Rm_TransportHandle transportHandle;
    /* MessageQ receive queue tied to the transport handle */
    MessageQ_Handle    receiveMsgQ;
} Transport_MapEntry;

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter;
extern uint32_t rmFreeCounter;

/* RM test Global Resource List (GRL) */
extern const char rmGRL[];
/* Example Linux DTB file provided to RM Server for automatic Linux Kernel resource extraction */
extern const char rmLinuxDtb[];
/* RM test Global Policy provided to RM Server */
extern const char rmGlobalPolicy[];
/* RM test Static Policy provided to RM Client Delegate and Client */
extern const char rmStaticPolicy[];

/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* DSP core number according to DNUM */
uint16_t            coreNum;
/* Number of errors that occurred during the test */
uint16_t            testErrors;

/* Task to configure application transport code for RM */
Task_Handle         rmStartupTskHandle;
/* High priority task for receiving RM packets */
Task_Handle         rmReceiveTskHandle;
/* RM server test task */
Task_Handle         rmServerTskHandle;
/* RM client delegate and client test task */
Task_Handle         rmClientTskHandle;

/* GateMPs used to synchronize tests between the two RM test tasks */
GateMP_Handle       serverGate = NULL;
GateMP_Handle       clientGate = NULL;
/* GateMP keys */
IArg                serverKey;
IArg                clientKey;

/* Handle for heap that RM packets will be allocated from */
HeapBufMP_Handle    rmPktHeapHandle = NULL;

/* MessageQ used by RM CD to send packets to RM Server */
char                serverFromCdQueueName[30] = "RM_Server_From_CD_Queue";
/* MessageQ used by RM Server to send packets to RM CD */
char                cdFromServerQueueName[30] = "RM_CD_From_Server_Queue";
/* MessageQ used by RM Client to send packets to RM CD */
char                cdFromClientQueueName[30] = "RM_CD_From_Client_Queue";
/* MessageQ used by RM CD to send packets to RM Client */
char                clientFromCdQueueName[30] = "RM_Client_From_CD_Queue";

/* RM Server instance name (must match with RM Global Resource List (GRL) and policies */
char                rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
/* RM CD instance name (must match with RM Global Resource List (GRL) and policies */
char                rmCdName[RM_NAME_MAX_CHARS]     = "RM_Client_Delegate";
/* RM Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client";

/* RM instance handles */
Rm_Handle           rmServerHandle = NULL;
Rm_Handle           rmCdHandle     = NULL;
Rm_Handle           rmClientHandle = NULL;

/* RM instance service handles */
Rm_ServiceHandle   *rmServerServiceHandle = NULL;
Rm_ServiceHandle   *rmCdServiceHandle     = NULL;
Rm_ServiceHandle   *rmClientServiceHandle = NULL;

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
Transport_MapEntry  rmTransportMap[MAX_MAPPING_ENTRIES];

/* Static allocation response queue */
Rm_ServiceRespInfo  staticResponseQueue[MAX_STATIC_ALLOCATION_RESPS];
/* Static allocation response queue index */
uint32_t            numStaticResponses;

/* RM response info queue used to store service responses received via the callback function */
Rm_ServiceRespInfo  responseInfoQueue[MAX_QUEUED_SERVICE_RESPONSES];

/* RM resource names (must match resource node names in GRL and policies */
char                resourceNameMemRegion[RM_NAME_MAX_CHARS]  = "memory-regions";
char                resourceNameAccumCh[RM_NAME_MAX_CHARS]    = "accumulator-ch";
char                resourceNameGpQ[RM_NAME_MAX_CHARS]        = "gp-queue";
char                resourceNameAifQ[RM_NAME_MAX_CHARS]       = "aif-queue";
char                resourceNameQosCluster[RM_NAME_MAX_CHARS] = "qos-cluster";
char                resourceNameAifRxCh[RM_NAME_MAX_CHARS]    = "aif-rx-ch";
char                resourceNameInfraQ[RM_NAME_MAX_CHARS]     = "infra-queue";
char                resourceNameLowPrioQ[RM_NAME_MAX_CHARS]   = "low-prio-queue";
char                resourceNamePassQ[RM_NAME_MAX_CHARS]      = "pass-queue";
char                resourceNameBcpQ[RM_NAME_MAX_CHARS]       = "bcp-queue";

/* Test RM NameServer name */
char                nameServerNameFavQ[RM_NAME_MAX_CHARS]     = "My_Favorite_Queue";

/**********************************************************************
 *************************** Test Functions ***************************
 **********************************************************************/

Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet     *rmPkt = NULL;
    MsgQ_RmPacket *rmMsg = NULL;

    /* Allocate a messageQ message for containing the RM packet */
    rmMsg = (MsgQ_RmPacket *)MessageQ_alloc(MSGQ_HEAP_ID, sizeof(MsgQ_RmPacket));
    if (rmMsg == NULL) {
        System_printf("Error Core %d : MessageQ_alloc failed to allocate message: %d\n", coreNum);
        testErrors++;
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

void transportFree (MessageQ_Msg rmMsgQMsg, Rm_Packet *pkt)
{
    uint32_t pktSize = pkt->pktLenBytes;
    int32_t  status;

    /* All transports will free rmPkts to the same heap */
    HeapBufMP_free(rmPktHeapHandle, pkt, pktSize);

    status = MessageQ_free(rmMsgQMsg);
    if (status < 0) { 
        System_printf("Error Core %d : MessageQ_free failed to free message: %d\n", coreNum, status);
        testErrors++;
    }     
}

int32_t transportSend (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    MessageQ_QueueId  remoteQueueId = (MessageQ_QueueId)appTransport;
    MsgQ_RmPacket    *rmMsg = pktHandle;
    int32_t           status;    

    /* Write back data that was written by RM after alloc */
    Osal_rmEndMemAccess((void *)rmMsg->rmPkt, rmMsg->rmPkt->pktLenBytes);

    /* Send the message to the remote side */
    status = MessageQ_put(remoteQueueId, (MessageQ_Msg)rmMsg);
    if (status < 0) {
        transportFree((MessageQ_Msg)rmMsg, rmMsg->rmPkt);
        System_printf("Error Core %d : MessageQ_put failed to send message: %d\n", coreNum, status);
        testErrors++;
    }
    return (status);
}

void transportReceive (uint32_t transportMapEntry)
{
    MessageQ_Handle  receiveQ;
    int32_t          numPkts;
    MessageQ_Msg     rmMsg = NULL;
    Rm_Packet       *rmPkt = NULL;
    int32_t          status;
    uint32_t         i;  

    /* Check if any packets available */
    receiveQ = rmTransportMap[transportMapEntry].receiveMsgQ;
    numPkts = (int32_t) MessageQ_count(receiveQ);

    /* Process all available packets */
    for (i = 0; i < numPkts; i++) {
        status = (int32_t) MessageQ_get(receiveQ, &rmMsg, MessageQ_FOREVER);
        if (rmMsg == NULL) {
            System_printf("Error Core %d : MessageQ_get failed, returning a NULL packet\n", coreNum);
            testErrors++;
        }

        /* Extract the Rm_Packet from the RM msg */
        rmPkt = ((MsgQ_RmPacket *)rmMsg)->rmPkt;
        Osal_rmBeginMemAccess((void *) rmPkt, rmPkt->pktLenBytes);

        /* Provide packet to RM for processing */
        if (status = Rm_receivePacket(rmTransportMap[transportMapEntry].transportHandle, rmPkt)) {
            System_printf("Error Core %d : RM failed to process received packet: %d\n", coreNum, status);
            testErrors++;
        }

        /* Free RM packet buffer and messageQ message */
        transportFree(rmMsg, rmPkt);
    }
}

void serviceCallback(Rm_ServiceRespInfo *serviceResponse)
{
    uint32_t qIndex = 0;
    
    /* Populate next free entry in the responseInfoQueue */
    while (responseInfoQueue[qIndex].serviceId != 0) {
        qIndex++;
        if (qIndex == MAX_QUEUED_SERVICE_RESPONSES) {
            qIndex = 0;
        }
    }

    /* Save the response in the response queue for the test task to pick up */
    memcpy((void *)&responseInfoQueue[qIndex], (void *)serviceResponse, sizeof(Rm_ServiceRespInfo));
}

void waitForResponse(Rm_ServiceRespInfo *respInfo)
{
    uint32_t qIndex = 0;

    if ((respInfo->serviceState == RM_SERVICE_PROCESSING) ||
        (respInfo->serviceState == RM_SERVICE_PENDING_SERVER_RESPONSE)) {
        /* Scan responseInfoQueue for the response received via the callback function */
        while((responseInfoQueue[qIndex].serviceId != respInfo->serviceId) ||
              (responseInfoQueue[qIndex].rmHandle != respInfo->rmHandle)) {
            qIndex++;
            if (qIndex == MAX_QUEUED_SERVICE_RESPONSES) {
                qIndex = 0;
            }
            
            /* Higher priority receive task will retrieve response */
        }  

        memcpy((void *)respInfo, (void *)&responseInfoQueue[qIndex], sizeof(Rm_ServiceRespInfo));
        memset((void *)&responseInfoQueue[qIndex], 0, sizeof(Rm_ServiceRespInfo));
    }
}

void setRmRequest(Rm_ServiceReqInfo *reqInfo, Rm_ServiceType type, const char *resName, int32_t resBase,
                  uint32_t resLen, int32_t resAlign, const char *nsName, int setCallback, Rm_ServiceRespInfo *respInfo)
{                                                                                
    memset((void *)reqInfo, 0, sizeof(Rm_ServiceReqInfo));                                        
    reqInfo->type = type;                                                                           
    reqInfo->resourceName = resName;                                                                
    reqInfo->resourceBase = resBase;                                                                
    reqInfo->resourceLength = resLen;                                                               
    reqInfo->resourceAlignment = resAlign;                                                          
    reqInfo->resourceNsName = nsName;
    if (setCallback) {
        reqInfo->callback.serviceCallback = serviceCallback;  
    }
    memset((void *)respInfo, 0, sizeof(Rm_ServiceRespInfo));                                     
}

void rmCleanupTsk(UArg arg0, UArg arg1)
{
    int32_t result;
    int32_t finalMallocFree;    
    
    /* Delete the RM test tasks */
    System_printf("Core %d : Deleting RM startup task...\n", coreNum);
    if (rmStartupTskHandle) {
       Task_delete(&rmStartupTskHandle);
       /* Set the task handle to be NULL so that the delete only occurs once */
       rmStartupTskHandle = NULL;
    }  
    
    if (coreNum == SYSINIT) {
        if (rmServerTskHandle) {
            System_printf("Core %d : Deleting RM server task...\n", coreNum);
            Task_delete(&rmServerTskHandle);
            /* Set the task handle to be NULL so that the delete only occurs once */
            rmServerTskHandle = NULL;
        }
    }
    else {
        if (rmClientTskHandle) {
            System_printf("Core %d : Deleting RM client task...\n", coreNum);        
            Task_delete(&rmClientTskHandle);
            /* Set the task handle to be NULL so that the delete only occurs once */
            rmClientTskHandle = NULL;
        }
    }
    
    /* Delete the RM receive task */
    System_printf("Core %d : Deleting RM receive task...\n", coreNum);
    if (rmReceiveTskHandle) {
        Task_delete(&rmReceiveTskHandle);
        /* Set the task handle to be NULL so that the delete only occurs once */
        rmReceiveTskHandle = NULL;
    }

    /* Cleanup all service ports, transport handles, RM instances, and IPC constructs */
    if (coreNum == SYSINIT) {
        result = Rm_serviceCloseHandle(rmServerServiceHandle);
        ERROR_CHECK(RM_OK, result, rmServerName, "Service handle close failed");

        result = Rm_transportUnregister(rmTransportMap[SERVER_TO_CD_MAP_ENTRY].transportHandle);
        ERROR_CHECK(RM_OK, result, rmServerName, "Unregister of CD transport failed");

        result = Rm_delete(rmServerHandle, RM_TEST_TRUE);  
        ERROR_CHECK(RM_OK, result, rmServerName, "Instance delete failed");
    }
    else {
        result = Rm_serviceCloseHandle(rmCdServiceHandle);
        ERROR_CHECK(RM_OK, result, rmCdName, "Service handle close failed");
        result = Rm_serviceCloseHandle(rmClientServiceHandle);
        ERROR_CHECK(RM_OK, result, rmClientName, "Service handle close failed");

        result = Rm_transportUnregister(rmTransportMap[CD_TO_SERVER_MAP_ENTRY].transportHandle);
        ERROR_CHECK(RM_OK, result, rmCdName, "Unregister of Server transport failed");
        result = Rm_transportUnregister(rmTransportMap[CD_TO_CLIENT_MAP_ENTRY].transportHandle);
        ERROR_CHECK(RM_OK, result, rmCdName, "Unregister of Client transport failed");
        result = Rm_transportUnregister(rmTransportMap[CLIENT_TO_CD_MAP_ENTRY].transportHandle);
        ERROR_CHECK(RM_OK, result, rmClientName, "Unregister of CD transport failed");

        result = Rm_delete(rmCdHandle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, rmCdName, "Instance delete failed");  
        result = Rm_delete(rmClientHandle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, rmClientName, "Instance delete failed");         
    }

    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("Core %d : ------------------ Memory Leak Check --------------------\n", coreNum);
    System_printf ("Core %d : -                       : malloc count   |   free count -\n", coreNum);
    System_printf ("Core %d : - Example Completion    :  %6d        |  %6d      -\n", coreNum,
                   rmMallocCounter, rmFreeCounter);
    finalMallocFree = rmMallocCounter - rmFreeCounter; 
    if (finalMallocFree > 0) {
        System_printf ("Core %d : - FAILED - %6d unfreed mallocs                       -\n",
                       coreNum, finalMallocFree);
        testErrors++;
    }
    else if (finalMallocFree < 0) {
        System_printf ("Core %d : - FAILED - %6d more frees than mallocs               -\n",
                       coreNum, -finalMallocFree);
        testErrors++;
    }
    else {
        System_printf ("Core %d : - PASSED                                                -\n",
                       coreNum);
    }
    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("\n");  

    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("Core %d : ------------------ Example Completion -------------------\n", coreNum);
    if (testErrors) {
        System_printf ("Core %d : - Test Errors: %-32d         -\n", coreNum, testErrors);
    }
    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("\n"); 

    BIOS_exit(0);
}

/* Receive task has priority of 2 so that it pre-empts the RM instance test tasks */
void rmReceiveTsk(UArg arg0, UArg arg1)
{
    while(1) {
        if (coreNum == SYSINIT) {
            transportReceive(SERVER_TO_CD_MAP_ENTRY);
        }
        else {
            transportReceive(CD_TO_SERVER_MAP_ENTRY);
            transportReceive(CD_TO_CLIENT_MAP_ENTRY);
            transportReceive(CLIENT_TO_CD_MAP_ENTRY);
        }
        /* Sleep for 1ms so that the main test tasks can run */
        Task_sleep(1);
    }
}

void rmServerTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;
    Task_Params        taskParams;

    /* BEGIN testing UNSPECIFIED base and alignment requests on Server */               
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);       
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);                 
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    
               
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, RM_RESOURCE_ALIGNMENT_UNSPECIFIED, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("---- Use Allocation w/ UNSPECIFIED Base & Alignment -----", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        RM_RESOURCE_ALIGNMENT_UNSPECIFIED, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 200, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNamePassQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- UNSPECIFIED Allocation Avoiding Policy Denial (Part 1)-", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNamePassQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- UNSPECIFIED Allocation Avoiding Policy Denial (Part 2)-", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNamePassQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- UNSPECIFIED Allocation Avoiding Policy Denial (Part 3)-", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNamePassQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- UNSPECIFIED Allocation Avoiding Policy Denial (Part 4)-", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END testing UNSPECIFIED base and alignment requests on Server */      

    /* Create new NameServer object */                
    setRmRequest(&requestInfo, Rm_service_RESOURCE_MAP_TO_NAME, resourceNameGpQ, 
                 1002, 1, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    POSITIVE_PASS_CHECK("--------------- Create NameServer Object ----------------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Wait for CD and Client retrieve resource via name, allocate the resource, free resource via name, and 
     * delete the NameServer object. */
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    GateMP_leave(clientGate, clientKey);
    serverKey = GateMP_enter(serverGate);

    /* Try to allocate the memory region taken by the Linux Kernel and not specified as shared */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameMemRegion, 
                 12, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    NEGATIVE_PASS_CHECK("------- Use Allocation of Resource Owned by Linux -------", 
                        coreNum, rmServerName, resourceNameMemRegion,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);       

    /* BEGIN testing expansion/contraction of resource nodes with the AIF RX CH resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 14, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);       
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Use Allocate) --", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);       
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 19, 31, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Use Allocate) --", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    /* Wait for Client and Client Delegate to do their allocations */
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    GateMP_leave(clientGate, clientKey);
    serverKey = GateMP_enter(serverGate);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 25, 3, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 34, 3, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      
 
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 28, 6, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 53, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  
    /* END testing expansion/contraction of resource nodes with the AIF RX CH resource */  
    
    /* Test exclusive rights to an allocated resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 2, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    NEGATIVE_PASS_CHECK("------ Use Allocation of Exclusively Owned Resource -----", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    /* Allocate small regions of general purpuse queues to prepare for CD local allocation testing */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 2100, 10, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-- Use Allocation Preparing for CD Local Alloc Testing --", 
                        coreNum, rmServerName, resourceNameGpQ,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    /* Wait for Client and Client Delegate to do their UNSPECIFIED allocates and CD testing */
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    GateMP_leave(clientGate, clientKey);
    serverKey = GateMP_enter(serverGate);

    /* Test allocation of a resource twice from the same instance with init and use privileges.  Both
     * should be approved but the instance should have only one owner instance in resource's owner list */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 6543, 10, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-- Init/Use Allocate of Resource from Same Inst (Init) --", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 6543, 10, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("--- Init/Use Allocate of Resource from Same Inst (Use) --", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);
    
    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 6543, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("--------- Status Check of Resources from Server ---------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount, responseInfo.serviceState,
                      RM_SERVICE_APPROVED, 1, 2); 
    
    /* Should take two frees to free both references */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 6543, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("------- Free of Resource from Same Inst (1st Ref) -------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 6543, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("--------- Status Check of Resources from Server ---------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount, responseInfo.serviceState,
                      RM_SERVICE_APPROVED, 1, 1); 

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 6543, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("------- Free of Resource from Same Inst (2nd Ref) -------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 6543, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("--------- Status Check of Resources from Server ---------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount, responseInfo.serviceState,
                      RM_SERVICE_APPROVED, 0, 0);     

    /* Allocate infrastructure queue taken by Linux kernel and shared with Rm_Client.  Expect error or denial. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameInfraQ, 
                 805, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    NEGATIVE_PASS_CHECK("- Init Allocation of Shared Linux and Client Resource  --", 
                        coreNum, rmServerName, resourceNameInfraQ,
                        805, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameAifRxCh, 
                 53, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("--------- Status Check of Resources from Server ---------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount, responseInfo.serviceState,
                      RM_SERVICE_APPROVED, 2, 1); 

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameQosCluster, 
                 1, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("--------- Status Check of Resources from Server ---------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 0, 0);

    /* BEGIN Testing CD local allocation feature from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 2051, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmServerServiceHandle->Rm_serviceHandler(rmServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("--------- CD Testing: Allocate Use From Server ----------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);
    /* END Testing CD local allocation feature from Server */


#if PRINT_USED_RESOURCES
    Rm_resourceStatus(rmServerHandle, RM_TEST_TRUE);
#endif

    /* Signal to ClientTsk that Server is ready for cleanup */
    GateMP_leave(serverGate, serverKey);
    
    /* Create the RM cleanup task. */
    System_printf("Core %d : Creating RM cleanup task...\n", coreNum);
    Task_Params_init (&taskParams);
    Task_create (rmCleanupTsk, &taskParams, NULL);
}

void rmClientTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;    
    Task_Params        taskParams;
    uint32_t           i, j;

    serverKey = GateMP_enter(serverGate);  

    /* Retrieve a resource via a NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_GET_BY_NAME, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("------- Retrieve Resource Via NameServer Object ---------", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        0, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Allocate the resource returned from the NameServer request */
    memset((void *)&requestInfo, 0, sizeof(Rm_ServiceReqInfo)); 
    requestInfo.type = Rm_service_RESOURCE_ALLOCATE_INIT;
    requestInfo.resourceName = responseInfo.resourceName;
    requestInfo.resourceBase = responseInfo.resourceBase;
    requestInfo.resourceLength = responseInfo.resourceLength;
    requestInfo.resourceNsName = NULL;
    requestInfo.callback.serviceCallback = serviceCallback;     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------- Init Allocate Using Retrieved Resource ---------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Retrieve the resource status via the NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    STATUS_PASS_CHECK("---- Retrieve Resource Status Via NameServer Object -----", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 1, 1);  

    /* Free resource via a NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("--- Free of Retrieved Resource Using NameServer Name ----", 
                        coreNum, rmClientName, nameServerNameFavQ,
                        0, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    /* Delete the NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("--------------- Delete NameServer Object ----------------", 
                        coreNum, rmClientName, nameServerNameFavQ,
                        0, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    GateMP_leave(clientGate, clientKey);
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    serverKey = GateMP_enter(serverGate);    

    /* BEGIN testing expansion/contraction of resource nodes with the AIF RX CH resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 0, 6, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);    
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Use Allocate) --", 
                        coreNum, rmClientName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameAifRxCh, 
                 50, 7, 0, NULL, RM_TEST_TRUE, &responseInfo);        
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);    
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Init Allocate) -", 
                        coreNum, rmCdName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END testing expansion/contraction of resource nodes with the AIF RX CH resource */

    GateMP_leave(clientGate, clientKey);
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    serverKey = GateMP_enter(serverGate);  


    /* BEGIN testing allocations with UNSPECIFIED base and alignment values */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh,
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 4, NULL, RM_TEST_TRUE, &responseInfo);
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------",
                        coreNum, rmCdName, resourceNameAccumCh,
                        responseInfo.resourceBase, responseInfo.resourceLength,
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh,
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, 1, NULL, RM_TEST_TRUE, &responseInfo);
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------",
                        coreNum, rmClientName, resourceNameAccumCh,
                        responseInfo.resourceBase, responseInfo.resourceLength,
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh,
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, RM_RESOURCE_ALIGNMENT_UNSPECIFIED, NULL, RM_TEST_TRUE, &responseInfo);
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);
    POSITIVE_PASS_CHECK("---- Use Allocation w/ UNSPECIFIED Base & Alignment -----",
                        coreNum, rmClientName, resourceNameAccumCh,
                        responseInfo.resourceBase, responseInfo.resourceLength,
                        RM_RESOURCE_ALIGNMENT_UNSPECIFIED, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameBcpQ,
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 1, NULL, RM_TEST_TRUE, &responseInfo);
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    NEGATIVE_PASS_CHECK("--- UNSPECIFIED Allocation Exclusion (Negative Test) ----",
                        coreNum, rmClientName, resourceNameBcpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength,
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameBcpQ,
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, 1, NULL, RM_TEST_TRUE, &responseInfo);
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("------------ UNSPECIFIED Allocation Exclusion -----------",
                        coreNum, rmClientName, resourceNameBcpQ,
                        responseInfo.resourceBase, responseInfo.resourceLength,
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);
    /* END testing allocations with UNSPECIFIED base and alignment values */

    /* Allocate infrastructure queue shared between Linux kernel and Client */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameInfraQ, 
                 800, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-- Init Allocation of Shared Linux and Client Resource --", 
                        coreNum, rmClientName, resourceNameInfraQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    /* BEGIN Allocating some resources without providing a callback function.  RM should block and not return until the result
     * is returned by the server. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 7000, 1, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("- Init Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7005, 25, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmCdName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7010, 5, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Init allocation of resource already owned by Client should return approved and there should only
     * be one instance of Client in resource's owner list. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 7011, 1, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("----- Use Allocation of Owned Resource (RM Blocked) -----", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END Allocating some resources without providing a callback function.  RM should block and not return
     * until the result is returned by the Server. */   

    /* BEGIN Getting the status of resources from Client and CD */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 7012, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);  
    STATUS_PASS_CHECK("-- Status Check of Resources from Client (Non-Blocking) -", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 2, 1);   
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 4025, 20, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("---- Status Check of Resources from Client (Blocking) ---", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 1, 0);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameInfraQ, 
                 800, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);     
    STATUS_PASS_CHECK("---- Status Check of Resources from CD (Non-Blocking) ---", 
                      coreNum, rmCdName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 2, 0); 

    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameInfraQ, 
                 805, 6, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("------ Status Check of Resources from CD (Blocking) -----", 
                      coreNum, rmCdName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount, 
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 1, 0);   
    /* END Getting the status of resources from Client and CD */    

    /* BEGIN Testing CD local allocation feature */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);   
    POSITIVE_PASS_CHECK("---- CD Testing: Allocate From CD (Non-Blocking) ----", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    POSITIVE_PASS_CHECK("------ CD Testing: Allocate From CD (Blocking) ------", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 50, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);    
    POSITIVE_PASS_CHECK("---- CD Testing: Allocate From Client (Non-Blocking) ----", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 50, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("------ CD Testing: Allocate From Client (Blocking) ------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 910, 2, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-- CD Testing: Alloc Local Res Explicitly From Client ---", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 965, 2, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("---- CD Testing: Alloc Local Res Explicitly From CD -----", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-- CD Testing: Alloc Local Res Unspecified From Client --", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("---- CD Testing: Alloc Local Res Unspecified From CD ----", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameLowPrioQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 7, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-- CD Testing: Alloc Local Res From Client For Sharing --", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        RM_RESOURCE_BASE_UNSPECIFIED, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameLowPrioQ, 
                 responseInfo.resourceBase, responseInfo.resourceLength, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    STATUS_PASS_CHECK("----- Status Check of Resource Alloc'd Locally to CD ----", 
                      coreNum, rmCdName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 1, 0); 

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameLowPrioQ, 
                 responseInfo.resourceBase, responseInfo.resourceLength, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- CD Testing: Alloc Shared Local Resource From CD ----", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameLowPrioQ, 
                 responseInfo.resourceBase, responseInfo.resourceLength, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("- Status Check of Res Alloc'd Locally to CD After Share -", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 2, 1);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 900, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    NEGATIVE_PASS_CHECK("-- Attempt Shared Alloc of Unshared Local CD Resourcee --", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameLowPrioQ, 
                 133, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- CD Testing: Free Shared Local Resource From CD -----", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameLowPrioQ, 
                 133, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-- CD Testing: Free Shared Local Resource From Client ---", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameLowPrioQ, 
                 responseInfo.resourceBase, responseInfo.resourceLength, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("-- Status Check After Free of Locally Shared Resource ---", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 0, 0);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 965, 2, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-------- CD Testing: Free Local Resource From CD --------", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED); 

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 898, 10, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-------- CD Testing: Free Local Resource From CD --------", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 972, 5, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-------- CD Testing: Free Local Resource From CD --------", 
                        coreNum, rmCdName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 910, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- CD Testing: Free Local Resource From CD Client -----", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 914, 50, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- CD Testing: Free Local Resource From CD Client -----", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 967, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- CD Testing: Free Local Resource From CD Client -----", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      
    /* END Testing CD local allocation feature */ 


    /* Verify static allocations were validated.  Validation responses should have been received after the
     * first service requests were made on the Client and CD post transport path registration. */
    while (numStaticResponses > 0) {
        /* Loop until all static request validations have been received */
        for (i = 0; i < MAX_STATIC_ALLOCATION_RESPS; i++) {
            if (staticResponseQueue[i].serviceId != 0) {            
                for (j = 0; j < MAX_QUEUED_SERVICE_RESPONSES; j++) {
                    if ((staticResponseQueue[i].serviceId == responseInfoQueue[j].serviceId) &&
                        (staticResponseQueue[i].rmHandle == responseInfoQueue[j].rmHandle)) {                   
                        POSITIVE_PASS_CHECK("------------- Static Allocation Validation --------------", 
                                            coreNum, rmClientName, responseInfoQueue[j].resourceName,
                                            responseInfoQueue[j].resourceBase, responseInfoQueue[j].resourceLength, 
                                            0, responseInfoQueue[j].serviceState, 
                                            RM_SERVICE_APPROVED); 
                        memset((void *)&staticResponseQueue[i], 0, sizeof(Rm_ServiceRespInfo));
                        memset((void *)&responseInfoQueue[j], 0, sizeof(Rm_ServiceRespInfo));
                        numStaticResponses--;                        
                        break;
                    }
                }
            }
        }    
    }  

#if PRINT_USED_RESOURCES
    Rm_resourceStatus(rmCdHandle, RM_TEST_TRUE);
#endif

    GateMP_leave(clientGate, clientKey);
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    /* Enter Server gate one last time to wait for Server to complete testing prior to entering cleanup */
    serverKey = GateMP_enter(serverGate);  
    
    /* Create the RM cleanup task. */
    System_printf("Core %d : Creating RM cleanup task...\n", coreNum);
    Task_Params_init (&taskParams);
    Task_create (rmCleanupTsk, &taskParams, NULL);
}

void rmStartupTsk(UArg arg0, UArg arg1)
{
    MessageQ_Handle    serverFromCdMsgQ, cdFromServerMsgQ, cdFromClientMsgQ, clientFromCdMsgQ;
    MessageQ_QueueId   serverToCdQId, cdToServerQId, cdToClientQId, clientToCdQId;    
    Int                status, i;
    GateMP_Params      gateParams;    
    HeapBufMP_Handle   msgQHeapHandle;
    HeapBufMP_Params   heapBufParams;
    Rm_TransportCfg    rmTransportCfg;
    int32_t            result = 0;
    Rm_TransportHandle serverCdHandle, cdServerHandle, cdClientHandle, clientCdHandle;
    Task_Params        taskParams;

    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rmTransportMap[i].transportHandle = NULL;
    } 

    if (coreNum == SYSINIT) {
        GateMP_Params_init(&gateParams);
        gateParams.name = RM_SERVER_GATE_NAME;
        /* Disable local protection since only concerned with sync'ing cores */
        gateParams.localProtect = GateMP_LocalProtect_NONE;
        serverGate = GateMP_create(&gateParams);

        serverKey = GateMP_enter(serverGate);  

        do {
            status = GateMP_open(RM_CLIENT_GATE_NAME, &clientGate);
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);        
        
        /* Create the heap that will be used to allocate RM messages. This
         * heap is a multi-processor heap.  It will be shared amongst
         * all RM instances. */     
        HeapBufMP_Params_init(&heapBufParams);
        heapBufParams.regionId       = 0;
        heapBufParams.name           = RM_PKT_HEAP_NAME;
        heapBufParams.numBlocks      = 64;
        heapBufParams.blockSize      = sizeof(Rm_Packet);
        rmPktHeapHandle = HeapBufMP_create(&heapBufParams);
        if (rmPktHeapHandle == NULL) {
            System_printf("Error Core %d : Failed to create RM packet heap\n", coreNum);
            testErrors++;
        }
        System_printf("Core %d : RM packet heap created\n", coreNum);

        /* Create the heap that will be used to allocate messageQ messages. */     
        HeapBufMP_Params_init(&heapBufParams);
        heapBufParams.regionId       = 0;
        heapBufParams.name           = MSGQ_HEAP_NAME;
        heapBufParams.numBlocks      = 64;
        heapBufParams.blockSize      = sizeof(MsgQ_RmPacket);
        msgQHeapHandle = HeapBufMP_create(&heapBufParams);
        if (msgQHeapHandle == NULL) {
            System_printf("Error Core %d : Failed to create HeapBufMP MessageQ heap\n", coreNum);
            testErrors++;
        } 
        System_printf("Core %d : IPC MessageQ message heap created\n", coreNum);
    }
    else {
        GateMP_Params_init(&gateParams);
        gateParams.name = RM_CLIENT_GATE_NAME;
        /* Disable local protection since only concerned with sync'ing cores */
        gateParams.localProtect = GateMP_LocalProtect_NONE;
        clientGate = GateMP_create(&gateParams);

        clientKey = GateMP_enter(clientGate); 
        
        do {
            status = GateMP_open(RM_SERVER_GATE_NAME, &serverGate);
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        
        /* Open the heaps created by the other processor. Loop until opened. */
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
        System_printf("Core %d : RM packet heap opened\n", coreNum);
        
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
        System_printf("Core %d : IPC MessageQ message heap opened\n", coreNum);
    }
    
    /* Register the MessageQ heap with MessageQ */
    MessageQ_registerHeap(msgQHeapHandle, MSGQ_HEAP_ID);

    /* Create the messageQ's for each RM instance connection
     * Need four queues.  Topology will be:
     * RM Server <---> RM Client Delegate <---> RM Client 
     * 1 queues on RM Server
     * 2 queues on RM Client Delegate
     * 1 queues on RM Client */
    if (coreNum == SYSINIT) {
        serverFromCdMsgQ = MessageQ_create(serverFromCdQueueName, NULL);
        if (serverFromCdMsgQ == NULL) {
            System_printf("Error Core %d : Failed to create Server's receive Q for CD\n", coreNum);
            testErrors++;
        }
        System_printf("Core %d : Created Server receive Q for CD\n", coreNum);
    }
    else {
        cdFromServerMsgQ = MessageQ_create(cdFromServerQueueName, NULL);
        if (cdFromServerMsgQ == NULL) {
            System_printf("Error Core %d : Failed to create CD's receive Q for Server\n", coreNum);
            testErrors++;
        }
        System_printf("Core %d : Created CD receive Q for Server\n", coreNum);
        
        cdFromClientMsgQ = MessageQ_create(cdFromClientQueueName, NULL);
        if (cdFromClientMsgQ == NULL) {
            System_printf("Error Core %d : Failed to create CD's receive Q for Client\n", coreNum);
            testErrors++;
        } 
        System_printf("Core %d : Created CD receive Q for Client\n", coreNum);        
        
        clientFromCdMsgQ = MessageQ_create(clientFromCdQueueName, NULL);
        if (clientFromCdMsgQ == NULL) {
            System_printf("Error Core %d : Failed to create Client's receive Q for CD\n", coreNum);
            testErrors++;
        }
        System_printf("Core %d : Created Client receive Q for CD\n", coreNum);
    }
    
    /* Open the remote message queues. Also register the RM transports with each RM instance */
    if (coreNum == SYSINIT) {
        /* Open the Client Delegate messageQ from the Server */
        do {
            status = MessageQ_open(cdFromServerQueueName, &serverToCdQId); 
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : RM CD MessageQ opened from RM Server\n", coreNum);

        /* Register the Client Delegate with the RM Server Instance */
        rmTransportCfg.rmHandle = rmServerHandle;
        rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) serverToCdQId;
        rmTransportCfg.remoteInstType = Rm_instType_CLIENT_DELEGATE;
        rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
        rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
        serverCdHandle = Rm_transportRegister(&rmTransportCfg, &result);

        /* Store the mapping information in the transport map */
        rmTransportMap[SERVER_TO_CD_MAP_ENTRY].transportHandle = serverCdHandle;
        rmTransportMap[SERVER_TO_CD_MAP_ENTRY].receiveMsgQ = serverFromCdMsgQ;
        System_printf("Core %d : Registered RM Server <=> RM CD transport with RM Server instance\n", coreNum);
    }
    else {
        /* Open the Server messageQ from the Client Delegate */
        do {
            status = MessageQ_open(serverFromCdQueueName, &cdToServerQId); 
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : RM Server MessageQ opened from RM CD\n", coreNum);

        /* Register the Server with the RM Client Delegate Instance */
        rmTransportCfg.rmHandle = rmCdHandle;
        rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) cdToServerQId;
        rmTransportCfg.remoteInstType = Rm_instType_SERVER;
        rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
        rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
        cdServerHandle = Rm_transportRegister(&rmTransportCfg, &result);

        /* Store the mapping information in the transport map */
        rmTransportMap[CD_TO_SERVER_MAP_ENTRY].transportHandle = cdServerHandle;
        rmTransportMap[CD_TO_SERVER_MAP_ENTRY].receiveMsgQ = cdFromServerMsgQ;
        System_printf("Core %d : Registered RM CD <=> RM Server transport with RM CD instance\n", coreNum);
        
        /* Open the Client messageQ from the Client Delegate */
        do {
            status = MessageQ_open(clientFromCdQueueName, &cdToClientQId); 
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : RM Client MessageQ opened from RM CD\n", coreNum);

        /* Register the Client with the RM Client Delegate Instance */
        rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) cdToClientQId;
        rmTransportCfg.remoteInstType = Rm_instType_CLIENT;
        rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
        rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
        cdClientHandle = Rm_transportRegister(&rmTransportCfg, &result);

        /* Store the mapping information in the transport map */
        rmTransportMap[CD_TO_CLIENT_MAP_ENTRY].transportHandle = cdClientHandle;
        rmTransportMap[CD_TO_CLIENT_MAP_ENTRY].receiveMsgQ = cdFromClientMsgQ;
        System_printf("Core %d : Registered RM CD <=> RM Client transport with RM CD instance\n", coreNum);

        /* Open the Client Delegate messageQ from the Client */        
        do {
            status = MessageQ_open(cdFromClientQueueName, &clientToCdQId); 
            /* 
             *  Sleep for 1 clock tick to avoid inundating remote processor
             *  with interrupts if open failed
             */
            if (status < 0) { 
                Task_sleep(1);
            }
        } while (status < 0);
        System_printf("Core %d : RM CD MessageQ opened from RM Client\n", coreNum);

        /* Register the Client Delegate with the RM Client Instance */
        rmTransportCfg.rmHandle = rmClientHandle;
        rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) clientToCdQId;
        rmTransportCfg.remoteInstType = Rm_instType_CLIENT_DELEGATE;
        rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
        rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
        clientCdHandle = Rm_transportRegister(&rmTransportCfg, &result);

        /* Store the mapping information in the transport map */
        rmTransportMap[CLIENT_TO_CD_MAP_ENTRY].transportHandle = clientCdHandle;
        rmTransportMap[CLIENT_TO_CD_MAP_ENTRY].receiveMsgQ = clientFromCdMsgQ;
        System_printf("Core %d : Registered RM Client <=> RM CD transport with RM Client instance\n", coreNum);
    }

    /* Create the RM receive task.  Assign higher priority than the test tasks so that
     * when they spin waiting for messages from other RM instances the receive task is
     * executed. */
    System_printf("Core %d : Creating RM receive task...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 2;
    rmReceiveTskHandle = Task_create (rmReceiveTsk, &taskParams, NULL);
    
    /* Create the RM test tasks. */
    if (coreNum == SYSINIT) {
        System_printf("Core %d : Creating RM server task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 1;
        rmServerTskHandle = Task_create (rmServerTsk, &taskParams, NULL);
    }
    else if (coreNum) {
        System_printf("Core %d : Creating RM client task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 1;
        rmClientTskHandle = Task_create (rmClientTsk, &taskParams, NULL);
    }
}

int main(Int argc, Char* argv[])
{
    Rm_InitCfg         rmInitCfg;
    Task_Params        taskParams; 
    int                status;
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;
    int32_t            result;

    System_printf ("*********************************************************\n");
    System_printf ("********************** RM Testing ***********************\n");
    System_printf ("*********************************************************\n");

    System_printf ("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
    testErrors = 0;

    /* Initialize the RM instances - RM must be initialized before anything else in the system
     * Core 0: 1 RM Instance  - RM Server
     * Core 1: 2 RM Instances - RM Client Delegate
     *                          RM Client
     */
    if (coreNum == SYSINIT) {
        /* Create the Server instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmServerName;
        rmInitCfg.instType = Rm_instType_SERVER;
        rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGRL;
        rmInitCfg.instCfg.serverCfg.linuxDtb = (void *)rmLinuxDtb;
        rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmGlobalPolicy;
        rmServerHandle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmServerName, "Initialization failed");

        /* Open Server service handle */
        rmServerServiceHandle = Rm_serviceOpenHandle(rmServerHandle, &result);
        ERROR_CHECK(RM_OK, result, rmServerName, "Service handle open failed");
    }
    else {
        /* Create the RM Client Delegate instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmCdName;
        rmInitCfg.instType = Rm_instType_CLIENT_DELEGATE;
        rmInitCfg.instCfg.cdCfg.cdPolicy = (void *)rmGlobalPolicy;
        rmCdHandle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_WARNING_CD_INSTANCE_NOT_STABLE, result, rmCdName, "Initialization failed");

        /* Open CD service handle */
        rmCdServiceHandle = Rm_serviceOpenHandle(rmCdHandle, &result);
        ERROR_CHECK(RM_OK, result, rmCdName, "Service handle open failed");

        /* Create the RM Client instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmClientName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmInitCfg.instCfg.clientCfg.staticPolicy = (void *)rmStaticPolicy;
        rmClientHandle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

        /* Open Client service handle */
        rmClientServiceHandle = Rm_serviceOpenHandle(rmClientHandle, &result);
        ERROR_CHECK(RM_OK, result, rmClientName, "Service handle open failed");

        /* Initialize the static allocation response queue */
        for (numStaticResponses = 0; numStaticResponses < MAX_STATIC_ALLOCATION_RESPS; numStaticResponses++) {
            memset((void *)&staticResponseQueue[numStaticResponses], 0, sizeof(Rm_ServiceRespInfo));
        }
        numStaticResponses = 0;

        /* Static allocation tests */
        setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameQosCluster, 
                     0, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);
        rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);               
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmCdName, resourceNameQosCluster,
                            requestInfo.resourceBase, requestInfo.resourceLength, 
                            requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED_STATIC);     
        if (responseInfo.serviceState == RM_SERVICE_APPROVED_STATIC) {        
            memcpy((void *)&staticResponseQueue[numStaticResponses++], (void *)&responseInfo, sizeof(responseInfo));
        }        
        
        setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameQosCluster, 
                     2, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);        
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmClientName, resourceNameQosCluster,
                            requestInfo.resourceBase, requestInfo.resourceLength, 
                            requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED_STATIC);     
        if (responseInfo.serviceState == RM_SERVICE_APPROVED_STATIC) {        
            memcpy((void *)&staticResponseQueue[numStaticResponses++], (void *)&responseInfo, sizeof(responseInfo));
        }           

        /* Request resource from Client that can only be allocated to CD according to static policy */
        setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameQosCluster, 
                     1, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);        
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
        NEGATIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmClientName, resourceNameQosCluster,
                            requestInfo.resourceBase, requestInfo.resourceLength, 
                            requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED_STATIC);          

        /* Request resource from both Client and CD that is shared according to static policy */
        setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameAifQ, 
                     525, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);        
        rmCdServiceHandle->Rm_serviceHandler(rmCdServiceHandle->rmHandle, &requestInfo, &responseInfo);
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmCdName, resourceNameAifQ,
                            requestInfo.resourceBase, requestInfo.resourceLength, 
                            requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED_STATIC);     
        if (responseInfo.serviceState == RM_SERVICE_APPROVED_STATIC) {        
            memcpy((void *)&staticResponseQueue[numStaticResponses++], (void *)&responseInfo, sizeof(responseInfo));
        }           

        setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameAifQ, 
                     525, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);        
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo); 
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmClientName, resourceNameAifQ,
                            requestInfo.resourceBase, requestInfo.resourceLength, 
                            requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED_STATIC);     
        if (responseInfo.serviceState == RM_SERVICE_APPROVED_STATIC) {        
            memcpy((void *)&staticResponseQueue[numStaticResponses++], (void *)&responseInfo, sizeof(responseInfo));
        }           
    }

    System_printf("Core %d : Starting IPC...\n", coreNum);
    status = Ipc_start();
    if (status < 0) {
        System_abort("Ipc_start failed\n");
    }

    /* Create the RM startup task */
    System_printf("Core %d : Creating RM startup task...\n", coreNum);
    Task_Params_init (&taskParams);
    rmStartupTskHandle = Task_create (rmStartupTsk, &taskParams, NULL);

    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();

    return (0);    
}

