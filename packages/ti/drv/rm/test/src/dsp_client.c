/*
 *   dsp_client.c
 *
 *   DSP portion of Resource Manager ARM+DSP test that uses RPMSG and sockets to 
 *   allow a DSP application to to request RM services from a RM Server running
 *   from Linux User-space.
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
 
/* Standard includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* XDC includes */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/System.h>

/* BIOS includes */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* IPC includes */
#include <ti/ipc/MultiProc.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/MessageQ.h>

/* CSL includes */
#include <ti/csl/csl_chip.h>

/* RM Includes */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

/**********************************************************************
 ************************** RM Test Symbols ***************************
 **********************************************************************/

/* Test will run on this core */
#define TEST_CORE                    0

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* RM packet heap name */
#define MSGQ_HEAP_ID                 0

/* MessageQ Name for DSP RM Client */
#define CLIENT_MESSAGEQ_NAME         "RM_CLIENT"

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

#define STATUS_PASS_CHECK(title, core, instName, resName, resStart, resLen, refCnt, state, check, expectRefCnt) \
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
        System_printf ("Core %d : -                                                       -\n", core);          \
        if ((state == check) && (refCnt == expectRefCnt)) {                                                     \
            System_printf ("Core %d : - PASSED                                                -\n", core);      \
        }                                                                                                       \
        else {                                                                                                  \
            if (refCnt != expectRefCnt) {                                                                       \
                System_printf ("Core %d : - FAILED - Owner Count Mismatch                         -\n",         \
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
    MessageQ_MsgHeader msgQHeader;
    /* Pointer to RM packet */
    Rm_Packet          rmPkt;
} MsgQ_RmPacket;

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter;
extern uint32_t rmFreeCounter;

/* RM test Static Policy provided to RM Client */
extern const char rmStaticPolicy[];

/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* Core number */
uint16_t            coreNum;
/* Number of errors that occurred during the test */
uint16_t            testErrors;

/* Task to configure application transport code for RM */
Task_Handle         rmStartupTskHandle;
/* High priority task for receiving RM packets */
Task_Handle         rmReceiveTskHandle;
/* RM client delegate and client test task */
Task_Handle         rmClientTskHandle;

/* Handle for heap that RM packets will be allocated from */
HeapBufMP_Handle    rmPktHeapHandle = NULL;

/* Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client";

/* Client MessageQ */
MessageQ_Handle     rmClientQ = NULL;

/* Linux MessageQ ID */
MessageQ_QueueId    linuxQueueId;

/* Client instance handles */
Rm_Handle           rmClientHandle = NULL;

/* Client instance service handles */
Rm_ServiceHandle   *rmClientServiceHandle = NULL;

/* Client from Server transport handle */
Rm_TransportHandle  clientFromServerTransportHandle;

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
        rmPkt = &(rmMsg->rmPkt);
        rmPkt->pktLenBytes = pktSize;
        *pktHandle = (Rm_PacketHandle)rmMsg;
    }
    return (rmPkt);    
}

void transportFree (MessageQ_Msg rmMsgQMsg)
{
    int32_t  status;

    status = MessageQ_free(rmMsgQMsg);
    if (status < 0) { 
        System_printf("Error Core %d : MessageQ_free failed to free message: %d\n", coreNum, status);
        testErrors++;
    }        
}

int32_t transportSend (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    MessageQ_QueueId *remoteQueueId = (MessageQ_QueueId *)appTransport;
    MsgQ_RmPacket    *rmMsg = pktHandle;
    int32_t           status;    
    
    /* Send the message to Linux */
    status = MessageQ_put(*remoteQueueId, (MessageQ_Msg)rmMsg);
    if (status < 0) {
        System_printf("Error Core %d : MessageQ_put failed to send message: %d\n", coreNum, status);
        testErrors++;
    }
    
    return (0);
}

void transportReceive (void)
{
    int32_t          numPkts;
    MessageQ_Msg     rmMsg = NULL;
    Rm_Packet       *rmPkt = NULL;
    int32_t          status;
    uint32_t         i;  

    /* Check if any packets available */
    numPkts = (int32_t) MessageQ_count(rmClientQ);

    /* Process all available packets */
    for (i = 0; i < numPkts; i++) {
        status = (int32_t) MessageQ_get(rmClientQ, &rmMsg, MessageQ_FOREVER);
        if (rmMsg == NULL) {
            System_printf("Error Core %d : MessageQ_get failed, returning a NULL packet\n", coreNum);
            testErrors++;
        }

        /* Extract the Rm_Packet from the RM msg */
        rmPkt = &(((MsgQ_RmPacket *)rmMsg)->rmPkt);

        /* Provide packet to RM for processing */
        if (status = Rm_receivePacket(clientFromServerTransportHandle, rmPkt)) {
            System_printf("Error Core %d : RM failed to process received packet: %d\n", coreNum, status);
            testErrors++;
        }

        /* Free RM packet buffer and messageQ message */
        transportFree(rmMsg);
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

/* Packets received via rpmsg port will issue callback vai transportReceive function */
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
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;     
    int32_t            result;
    int32_t            finalMallocFree; 

    /* Free all allocated resources */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAccumCh, 
                 0, 7, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAccumCh, 
                 40, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameQosCluster, 
                 0, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameQosCluster, 
                 2, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifQ, 
                 525, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifQ, 
                 525, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameInfraQ, 
                 800, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 7000, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED); 
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 7011, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 7010, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);   

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameGpQ, 
                 7005, 25, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 0, 6, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED); 
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 50, 7, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-------------------- Resource Cleanup -------------------", 
                        coreNum, rmClientName, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Cleanup all service ports, transport handles, RM instances, and IPC constructs */
    result = Rm_serviceCloseHandle(rmClientServiceHandle);
    ERROR_CHECK(RM_OK, result, rmClientName, "Service handle close failed");

    result = Rm_transportUnregister(clientFromServerTransportHandle);
    ERROR_CHECK(RM_OK, result, rmClientName, "Unregister of Server transport failed");

	result = MessageQ_delete(&rmClientQ);
    if (result < 0) {
        System_printf("Core %d : Error in MessageQ_delete [%d]\n", coreNum, result);
        testErrors++;
    }
 
    result = Rm_delete(rmClientHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, result, rmClientName, "Instance delete failed");

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
}

/* Receive task has priority of 2 so that it pre-empts the RM instance test tasks */
void rmReceiveTsk(UArg arg0, UArg arg1)
{
    while(1) {
        transportReceive();
        /* Sleep for 1ms so that the main test tasks can run */
        Task_sleep(1);
    }
}

void rmClientTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;  
    Task_Params        taskParams;
    uint32_t           i, j;

    /* Create new NameServer object */                
    setRmRequest(&requestInfo, Rm_service_RESOURCE_MAP_TO_NAME, resourceNameGpQ, 
                 1002, 1, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("--------------- Create NameServer Object ----------------", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, responseInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED); 

    /* Retrieve a resource via a NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_GET_BY_NAME, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("------- Retrieve Resource Via NameServer Object ---------", 
                        coreNum, rmClientName, responseInfo.resourceName,
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
                      responseInfo.resourceNumOwners, responseInfo.serviceState, RM_SERVICE_APPROVED, 1);  

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
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);    
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Init Allocate) -", 
                        coreNum, rmClientName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END testing expansion/contraction of resource nodes with the AIF RX CH resource */

    /* BEGIN testing allocations with UNSPECIFIED base and alignment values */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 5, 4, NULL, RM_TEST_TRUE, &responseInfo);        
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmClientName, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, 1, NULL, RM_TEST_TRUE, &responseInfo);      
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmClientName, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, RM_RESOURCE_ALIGNMENT_UNSPECIFIED, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("---- Use Allocation w/ UNSPECIFIED Base & Alignment -----", 
                        coreNum, rmClientName, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        RM_RESOURCE_ALIGNMENT_UNSPECIFIED, responseInfo.serviceState, RM_SERVICE_APPROVED);     
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
                 7000, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("- Init Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7005, 25, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7010, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    waitForResponse(&responseInfo); 
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmClientName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Init allocation of resource already owned by Client should return approved and there should only
     * be one instance of Client in resource's owner list. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 7011, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    waitForResponse(&responseInfo); 
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
                      responseInfo.resourceNumOwners, responseInfo.serviceState, RM_SERVICE_APPROVED, 1);   
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 4025, 20, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
    waitForResponse(&responseInfo);   
    STATUS_PASS_CHECK("---- Status Check of Resources from Client (Blocking) ---", 
                      coreNum, rmClientName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.serviceState, RM_SERVICE_APPROVED, 1);  
    /* END Getting the status of resources from Client and CD */    

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

    /* Create the RM cleanup task. */
    System_printf("Core %d : Creating RM cleanup task...\n", coreNum);
    Task_Params_init (&taskParams);
    Task_create (rmCleanupTsk, &taskParams, NULL);
}

void rmStartupTsk(UArg arg0, UArg arg1)
{
    Task_Params      taskParams;  
    Char             localQueueName[64];    
    MessageQ_Msg     msg;
    Rm_TransportCfg  rmTransCfg;
    int32_t          rm_result;

    /* Construct a MessageQ name adorned with core name: */
    System_sprintf(localQueueName, "%s_%s", CLIENT_MESSAGEQ_NAME,
                   MultiProc_getName(MultiProc_self()));

    rmClientQ = MessageQ_create(localQueueName, NULL);
    if (rmClientQ == NULL) {
        System_abort("MessageQ_create failed\n");
    }

    if (coreNum == TEST_CORE) {
        System_printf("Awaiting sync message from host...\n");
        MessageQ_get(rmClientQ, &msg, MessageQ_FOREVER);
        
        linuxQueueId = MessageQ_getReplyQueue(msg);
        MessageQ_put(linuxQueueId, msg);

        /* Register the Server with the Client instance */
        rmTransCfg.rmHandle = rmClientHandle;
        rmTransCfg.appTransportHandle = (Rm_AppTransportHandle) &linuxQueueId;
        rmTransCfg.remoteInstType = Rm_instType_SERVER;
        rmTransCfg.transportCallouts.rmAllocPkt = transportAlloc;
        rmTransCfg.transportCallouts.rmSendPkt = transportSend;
        clientFromServerTransportHandle = Rm_transportRegister(&rmTransCfg, &rm_result);  

        /* Create the RM receive task.  Assign higher priority than the test tasks so that
         * when they spin waiting for messages from other RM instances the receive task is
         * executed. */
        System_printf("Core %d : Creating RM receive task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 2;
        rmReceiveTskHandle = Task_create (rmReceiveTsk, &taskParams, NULL);

        System_printf("Core %d : Creating RM client task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 1;
        rmClientTskHandle = Task_create (rmClientTsk, &taskParams, NULL);
    }
}

int main(int argc, char *argv[])
{ 
    Task_Params        taskParams;     
    Rm_InitCfg         rmInitCfg;
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;
    int32_t            result;


    System_printf ("*********************************************************\n");
    System_printf ("************ RM DSP+ARM DSP Client Testing **************\n");
    System_printf ("*********************************************************\n");

    System_printf ("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);

    if (coreNum == TEST_CORE) {
        testErrors = 0;  

        /* Initialize the RM Client - RM must be initialized before anything else in the system */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmClientName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmInitCfg.instCfg.clientCfg.staticPolicy = (void *)rmStaticPolicy;
        rmClientHandle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

        System_printf("\n\nInitialized %s\n\n", rmClientName);

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
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);               
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmClientName, resourceNameQosCluster,
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
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &requestInfo, &responseInfo);
        POSITIVE_PASS_CHECK("---------------- Static Init Allocation -----------------", 
                            coreNum, rmClientName, resourceNameAifQ,
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
    else {
        System_printf("Core %d : RM DSP+ARM Linux test not executing on this core\n", coreNum);
    }

    /* Create the RM startup task */
    System_printf("Core %d : Creating RM startup task...\n", coreNum);
    Task_Params_init (&taskParams);
    rmStartupTskHandle = Task_create (rmStartupTsk, &taskParams, NULL);

    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();

    return (0);    
}

