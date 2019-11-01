/*
 *   rm_dsp_mt_test.c
 *
 *   Singlecore Multiple Task Resource Manager test that uses IPC to an application
 *   requesting RM services from a RM Server, and Client.
 *
 *  ============================================================================
 *
 * Copyright (c) 2015, Texas Instruments Incorporated
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
#include <ti/sysbios/knl/Semaphore.h>

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
#define NUM_CORES                    1

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

/* Application's core 0 registered RM transport indices */
#define SERVER_TO_CLIENT_MAP_ENTRY    0
#define CLIENT_TO_SERVER_MAP_ENTRY    1
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES           2

/* Size of RM static allocation response queue.  Must be greater than number of APPROVED
 * static allocations */
#define MAX_STATIC_ALLOCATION_RESPS   5 

/* Size of RM service response queue */
#define MAX_QUEUED_SERVICE_RESPONSES  10

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)                   \
    if (resultVal != checkVal) {                                                 \
        char errorMsgToPrint[] = printMsg;                                       \
        System_printf("Error Core %d : %s : ", coreNum, rmInstName);             \
        System_printf("%s with error code : %d\n", errorMsgToPrint, resultVal);  \
        testErrors++;                                                            \
        System_abort("Test Failure\n");                                          \
    }


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
/* RM test Global Policy provided to RM Server */
extern const char rmGlobalPolicy[];

/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* DSP core number according to DNUM */
uint16_t            coreNum;
/* Number of errors that occurred during the test */
uint16_t            testErrors;

/* Task to configure application transport code for RM */
Task_Handle         startupTskHandle = NULL;
/* High priority task for server receiving RM packets */
Task_Handle         rmReceiveTskHandleServer = NULL;
/* High priority task for client receiving RM packets */
Task_Handle         rmReceiveTskHandleClient = NULL;
/* RM client test task 1 */
Task_Handle         rmClientTskHandle1 = NULL;
/* RM client test task 2 */
Task_Handle         rmClientTskHandle2 = NULL;
// task completion count
int                 taskCompleteCount = 0;

// client task semaphore lock
uint32_t           *client_semaphore_lock = NULL;

/* Handle for heap that RM packets will be allocated from */
HeapBufMP_Handle    rmPktHeapHandle = NULL;

/* MessageQ used by RM Client to send packets to RM Server */
char                serverFromClientQueueName[30] = "RM_Server_From_Client_Queue";
/* MessageQ used by RM Server to send packets to RM Client */
char                clientFromServerQueueName[30] = "RM_Client_From_Server_Queue";

/* RM Server instance name (must match with RM Global Resource List (GRL) and policies */
char                rmServerName[RM_NAME_MAX_CHARS] = "RM_Server";
/* RM Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client";

/* RM instance handles */
Rm_Handle           rmServerHandle = NULL;
Rm_Handle           rmClientHandle = NULL;

/* RM instance service handles */
Rm_ServiceHandle   *rmServerServiceHandle = NULL;
Rm_ServiceHandle   *rmClientServiceHandle = NULL;

MessageQ_Handle     serverFromClientMsgQ, clientFromServerMsgQ;
MessageQ_QueueId    serverToClientQId, clientToServerQId;
Rm_TransportHandle  serverClientHandle, clientServerHandle;

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
Transport_MapEntry  rmTransportMap[MAX_MAPPING_ENTRIES];

/* RM resource names (must match resource node names in GRL and policies */
char                res_name_link_ram[RM_NAME_MAX_CHARS] = "link-ram";

/* Used to track allocations in each task */
#define REQUEST_ITERATIONS 16000
#define RESOURCE_PRINT_DIVISOR 1000
uint32_t            allocs[REQUEST_ITERATIONS];

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

/* Receive task has priority of 2 so that it pre-empts the RM instance test tasks */
void rmReceiveTskServer(UArg arg0, UArg arg1)
{
    while(1) {
        transportReceive(CLIENT_TO_SERVER_MAP_ENTRY);
        /* Sleep for 1ms so that the main test tasks can run */
        Task_sleep(1);
    }
}

void rmReceiveTskClient(UArg arg0, UArg arg1)
{
    while(1) {
    transportReceive(SERVER_TO_CLIENT_MAP_ENTRY);
        /* Sleep for 1ms so that the main test tasks can run */
        Task_sleep(1);
    }
}

void rmClientTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    uint32_t            i;

    if (rmServerServiceHandle) {
        /* Init request - request one resource at a time as unspecified.  Server
         * will return next available resource */
        memset(&request, 0, sizeof(request));
        request.type = Rm_service_RESOURCE_ALLOCATE_INIT;
        request.resourceName = res_name_link_ram;
        request.resourceBase = RM_RESOURCE_BASE_UNSPECIFIED;
        request.resourceLength = 1;

        System_printf("Client Task %d : Requesting %d resources...\n", (uint32_t)arg0, REQUEST_ITERATIONS / 2);
        for (i = 0; i < (REQUEST_ITERATIONS / 2); i++) {
            memset(&response, 0, sizeof(response));
            rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &request, &response);

            if (response.serviceState == RM_SERVICE_APPROVED) {
                allocs[response.resourceBase]++;
                if (arg0==2)
                    Task_sleep(1);

                if ((i % RESOURCE_PRINT_DIVISOR) == 0) {
                    System_printf("Client Task %d : Requested %d resources...\n", (uint32_t)arg0, RESOURCE_PRINT_DIVISOR);
                }
            }
            else {
                System_printf("Error Client Task %d : Allocate request was not approved for resource %d\n",
                    (uint32_t)arg0, request.resourceBase);
                testErrors++;
                goto error_exit;
            }
        }
    }

error_exit:
    taskCompleteCount++;
    System_printf("Client Task %d : Exiting...\n", (uint32_t)arg0);
    return;
}

void serverInit(void)
{
    HeapBufMP_Handle   msgQHeapHandle;
    HeapBufMP_Params   heapBufParams;
    int32_t            result = 0;
    Rm_InitCfg         rmInitCfg;

    // reset the allocation log
    memset(&allocs[0], 0, sizeof(allocs));

    // Server Instance Setup
    /* Initialize the RM instances - RM must be initialized before anything else in the system
     * Core 0: 1 RM Instance  - RM Server
     */
    /* Create the Server instance */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));
    rmInitCfg.instName = rmServerName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGRL;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmGlobalPolicy;
    rmServerHandle = Rm_init(&rmInitCfg, &result);
    ERROR_CHECK(RM_OK, result, rmServerName, "Initialization failed");

    /* Open Server service handle */
    rmServerServiceHandle = Rm_serviceOpenHandle(rmServerHandle, &result);
    ERROR_CHECK(RM_OK, result, rmServerName, "Service handle open failed");

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

    /* Register the MessageQ heap with MessageQ */
    MessageQ_registerHeap(msgQHeapHandle, MSGQ_HEAP_ID);

    /* Create the messageQ's for each RM instance connection
     * Need four queues.  Topology will be:
     * RM Server <---> RM Client
     * 1 queue on RM Server
     * 1 queue on RM Client */
    serverFromClientMsgQ = MessageQ_create(serverFromClientQueueName, NULL);
    if (serverFromClientMsgQ == NULL) {
        System_printf("Error Core %d : Failed to create Server's receive Q for Client\n", coreNum);
        testErrors++;
    }
    System_printf("Core %d : Created Server receive Q for Client\n", coreNum);
}

void clientInit(void)
{
    int32_t            result = 0;
    Rm_InitCfg         rmInitCfg;
    Semaphore_Params   semParams;
    HeapBufMP_Handle   msgQHeapHandle;
    Int                status;
    Rm_TransportCfg    rmTransportCfg;

    // Client Instance Setup
    /* Initialize the client semaphore lock */
    Semaphore_Params_init(&semParams);
    client_semaphore_lock = (uint32_t *)Semaphore_create(1, &semParams, NULL);
    if (client_semaphore_lock==NULL) {
        System_printf("Error : Failed to init client semaphore lock with error code: \n", client_semaphore_lock);
        testErrors++;
        return;
    }

    /* Create the RM Client instance */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));
    rmInitCfg.instName = rmClientName;
    rmInitCfg.instType = Rm_instType_CLIENT;
    rmInitCfg.mtSemObj = (uint32_t *)client_semaphore_lock;
    rmClientHandle = Rm_init(&rmInitCfg, &result);
    ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

    /* Open Client service handle */
    rmClientServiceHandle = Rm_serviceOpenHandle(rmClientHandle, &result);
    ERROR_CHECK(RM_OK, result, rmClientName, "Service handle open failed");

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

    /* Register the MessageQ heap with MessageQ */
    MessageQ_registerHeap(msgQHeapHandle, MSGQ_HEAP_ID);

    /* Create the messageQ's for each RM instance connection
     * Need four queues.  Topology will be:
     * RM Server <---> RM Client
     * 1 queue on RM Server
     * 1 queue on RM Client */
    clientFromServerMsgQ = MessageQ_create(clientFromServerQueueName, NULL);
    if (clientFromServerMsgQ == NULL) {
        System_printf("Error Core %d : Failed to create client's receive Q for Server\n", coreNum);
        testErrors++;
    }
    System_printf("Core %d : Created client receive Q for Server\n", coreNum);

    /* Open the remote message queues. Also register the RM transports with each RM instance */
    /* Open the Server messageQ from the Client Delegate */
    do {
        status = MessageQ_open(serverFromClientQueueName, &clientToServerQId);
        /*
         *  Sleep for 1 clock tick to avoid inundating remote processor
         *  with interrupts if open failed
         */
        if (status < 0) {
            Task_sleep(1);
        }
    } while (status < 0);
    System_printf("Core %d : RM Server MessageQ opened from RM Client\n", coreNum);

    /* Register the Server with the RM Client Instance */
    rmTransportCfg.rmHandle = rmClientHandle;
    rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) clientToServerQId;
    rmTransportCfg.remoteInstType = Rm_instType_SERVER;
    rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
    rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
    clientServerHandle = Rm_transportRegister(&rmTransportCfg, &result);

    /* Store the mapping information in the transport map */
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle = clientServerHandle;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].receiveMsgQ = clientFromServerMsgQ;
    System_printf("Core %d : Registered RM Client <=> RM Server transport with RM Client instance\n", coreNum);
}

void rmCleanup(void)
{
    int32_t result;
    int32_t finalMallocFree;

    /* Delete the RM server receive task */
    System_printf("Core %d : Deleting RM server receive task...\n", coreNum);
    if (rmReceiveTskHandleServer) {
        Task_delete(&rmReceiveTskHandleServer);
        /* Set the task handle to be NULL so that the delete only occurs once */
        rmReceiveTskHandleServer = NULL;
    }

    /* Delete the RM client receive task */
    System_printf("Core %d : Deleting RM client receive task...\n", coreNum);
    if (rmReceiveTskHandleClient) {
        Task_delete(&rmReceiveTskHandleClient);
        /* Set the task handle to be NULL so that the delete only occurs once */
        rmReceiveTskHandleClient = NULL;
    }

    /* Delete the RM client test task 1 */
    System_printf("Core %d : Deleting RM client test task 1...\n", coreNum);
    if (rmClientTskHandle1) {
        Task_delete(&rmClientTskHandle1);
        /* Set the task handle to be NULL so that the delete only occurs once */
        rmClientTskHandle1 = NULL;
    }

    /* Delete the RM client test task 2 */
    System_printf("Core %d : Deleting RM client test task 2...\n", coreNum);
    if (rmClientTskHandle2) {
        Task_delete(&rmClientTskHandle2);
        /* Set the task handle to be NULL so that the delete only occurs once */
        rmClientTskHandle2 = NULL;
    }

    /* Cleanup all service ports, transport handles, RM instances, and IPC constructs */
    result = Rm_transportUnregister(rmTransportMap[CLIENT_TO_SERVER_MAP_ENTRY].transportHandle);
    ERROR_CHECK(RM_OK, result, rmServerName, "Unregister of Server transport failed");
    result = Rm_serviceCloseHandle(rmServerServiceHandle);
    ERROR_CHECK(RM_OK, result, rmServerName, "Service handle close failed");
    result = Rm_delete(rmServerHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, result, rmServerName, "Instance delete failed");

    result = Rm_transportUnregister(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle);
    ERROR_CHECK(RM_OK, result, rmClientName, "Unregister of Client transport failed");
    result = Rm_serviceCloseHandle(rmClientServiceHandle);
    ERROR_CHECK(RM_OK, result, rmClientName, "Service handle close failed");
    result = Rm_delete(rmClientHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, result, rmClientName, "Instance delete failed");

    // delete the client_semaphore_lock
    if (client_semaphore_lock)
    {
        Semaphore_delete((Semaphore_Handle *)client_semaphore_lock);
    }

    System_printf("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf("Core %d : ------------------ Memory Leak Check --------------------\n", coreNum);
    System_printf("Core %d : -                       : malloc count   |   free count -\n", coreNum);
    System_printf("Core %d : - Example Completion    :  %6d        |  %6d      -\n", coreNum,
                   rmMallocCounter, rmFreeCounter);
    finalMallocFree = rmMallocCounter - rmFreeCounter;
    if (finalMallocFree > 0) {
        System_printf("Core %d : - FAILED - %6d unfreed mallocs                       -\n",
                       coreNum, finalMallocFree);
        testErrors++;
    }
    else if (finalMallocFree < 0) {
        System_printf("Core %d : - FAILED - %6d more frees than mallocs               -\n",
                       coreNum, -finalMallocFree);
        testErrors++;
    }
    else {
        System_printf("Core %d : - PASSED                                                -\n",
                       coreNum);
    }
    System_printf("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf("\n");

    System_printf("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf("Core %d : ------------------ Example Completion -------------------\n", coreNum);
    if (testErrors) {
        System_printf("Core %d : - Test Errors: %-32d         -\n", coreNum, testErrors);
    }
    System_printf("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf("\n");

    BIOS_exit(0);
}

void startupTsk(UArg arg0, UArg arg1)
{
    Int                status, i;
    Rm_TransportCfg    rmTransportCfg;
    int32_t            result = 0;
    Task_Params        taskParams;

    System_printf("*********************************************************\n");
    System_printf("***************** RM Startup Task ***********************\n");
    System_printf("*********************************************************\n");

    System_printf("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    // Server Instance Setup
    serverInit();

    // Client Instance Setup
    clientInit();

    /* Open the remote message queues. Also register the RM transports with each RM instance */
    /* Open the Client Delegate messageQ from the Server */
    do {
        status = MessageQ_open(clientFromServerQueueName, &serverToClientQId);
        /*
         *  Sleep for 1 clock tick to avoid inundating remote processor
         *  with interrupts if open failed
         */
        if (status < 0) {
            Task_sleep(1);
        }
    } while (status < 0);
    System_printf("Core %d : RM Client MessageQ opened from RM Server\n", coreNum);

    /* Register the Client with the RM Server Instance */
    rmTransportCfg.rmHandle = rmServerHandle;
    rmTransportCfg.appTransportHandle = (Rm_AppTransportHandle) serverToClientQId;
    rmTransportCfg.remoteInstType = Rm_instType_CLIENT;
    rmTransportCfg.transportCallouts.rmAllocPkt = transportAlloc;
    rmTransportCfg.transportCallouts.rmSendPkt = transportSend;
    serverClientHandle = Rm_transportRegister(&rmTransportCfg, &result);

    /* Store the mapping information in the transport map */
    rmTransportMap[CLIENT_TO_SERVER_MAP_ENTRY].transportHandle = serverClientHandle;
    rmTransportMap[CLIENT_TO_SERVER_MAP_ENTRY].receiveMsgQ = serverFromClientMsgQ;
    System_printf("Core %d : Registered RM Server <=> RM CLient transport with RM Server instance\n", coreNum);

    /* Create the RM server receive task */
    System_printf("Core %d : Creating RM receive task...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 4;
    rmReceiveTskHandleServer = Task_create (rmReceiveTskServer, &taskParams, NULL);

    /* Create the RM client receive task.  Assign higher priority than the test tasks so that
     * when they spin waiting for messages from other RM instances the receive task is
     * executed. */
    System_printf("Core %d : Creating RM client receive task...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 4;
    rmReceiveTskHandleClient = Task_create (rmReceiveTskClient, &taskParams, NULL);

    /* Create the RM client test task 1 */
    System_printf("Core %d : Creating RM client test task 1...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 1;
    taskParams.arg0 = 1;
    rmClientTskHandle1 = Task_create (rmClientTsk, &taskParams, NULL);

    /* Create the RM client test task 2 */
    System_printf("Core %d : Creating RM client test task 2...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 2;
    taskParams.arg0 = 2;
    rmClientTskHandle2 = Task_create (rmClientTsk, &taskParams, NULL);

    // wait for all the tasks to complete
    while (taskCompleteCount!=2)
    {
        Task_sleep(100);
    }

    // resource allocation checking
    if (!testErrors) {
        System_printf("Tasks complete - Checking for allocation errors\n");
        for (i = 0; i < REQUEST_ITERATIONS; i++) {
            if (allocs[i] != 1) {
                System_printf ("FAILED : Resource %d not allocated exactly once\n", i);
                testErrors++;
            }
        }

        if (!testErrors) {
            System_printf("PASSED : All Resources allocated once\n");
        }
    }

    // clean up
    rmCleanup();
}

int main(Int argc, Char* argv[])
{
    Task_Params taskParams;
    int         status, i;

    System_printf("*********************************************************\n");
    System_printf("********************** RM Testing ***********************\n");
    System_printf("*********************************************************\n");

    System_printf("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
    testErrors = 0;

    System_printf("Core %d : Start IPC...\n", coreNum);
    status = Ipc_start();
    if (status < 0) {
        System_abort("Server Ipc_start failed\n");
    }
    System_printf("Core %d : Server Task Started IPC\n", coreNum);

    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rmTransportMap[i].transportHandle = NULL;
    }
    taskCompleteCount = 0;

    /* Create the RM client1 task */
    System_printf("Core %d : Creating RM startup task...\n", coreNum);
    Task_Params_init (&taskParams);
    taskParams.priority = 8;
    startupTskHandle = Task_create (startupTsk, &taskParams, NULL);

    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();

    return (0);    
}

