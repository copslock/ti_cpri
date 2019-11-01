/*
 *   rm_linux_client_test.c
 *
 *   Multi-process Resource Manager test that uses sockets to allow a Linux
 *   User-space application to request RM services from a RM Server, 
 *   Client Delegate, and Client.
 *
 *  ============================================================================
 *
 * Copyright (c) 2012-2015, Texas Instruments Incorporated
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

 /* 
  * Shut off: remark #880-D: parameter "appTransport" was never referenced
  *
  * This is better than removing the argument since removal would break
  * backwards compatibility
  */
#ifdef _TMS320C6X
#pragma diag_suppress 880
#pragma diag_suppress 681
#elif defined(__GNUC__)
 /* Same for GCC:
  * warning: unused parameter ‘appTransport’ [-Wunused-parameter]
  */
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

 /* Standard includes */
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

/* Socket Includes */
#include "sockutils.h"

/* RM Includes */
#include <ti/drv/rm/rm_server_if.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

/**********************************************************************
 ************************** RM Test Symbols ***************************
 **********************************************************************/

#define error_msg printf
#define info_msg  printf

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* Socket timeout */
#define CLIENT_SOCK_TIMEOUT_USEC     (500)

/* Application's registered RM transport indices */
#define SERVER_TO_CLIENT_MAP_ENTRY   0
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES          1

/* Size of RM static allocation response queue.  Must be greater than number of APPROVED
 * static allocations */
#define MAX_STATIC_ALLOCATION_RESPS  5 

/* Size of RM service response queue */
#define MAX_QUEUED_SERVICE_RESPONSES 10

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)            \
    if (resultVal != checkVal) {                                          \
        char errorMsgToPrint[] = printMsg;                                \
        printf("Error Core %d : %s : ", coreNum, rmInstName);             \
        printf("%s with error code : %d\n", errorMsgToPrint, resultVal);  \
        testErrors++;                                                     \
    }

#define POSITIVE_PASS_CHECK(title, core, instName, resName, resStart, resLen, align, state, check)  \
    do {                                                                                            \
        int32_t start = resStart;                                                                   \
        int32_t alignment = align;                                                                  \
        char    titleMsg[] = title;                                                                 \
        printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        printf ("Core %d : %s\n", core, titleMsg);                                           \
        printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);               \
        printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                \
        if (start == RM_RESOURCE_BASE_UNSPECIFIED) {                                                \
            printf ("Core %d : - Start:         UNSPECIFIED                            -\n", \
                           core);                                                                   \
            printf ("Core %d : - Length:        %-16d                       -\n",            \
                           core, resLen);                                                           \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - Start:         %-16d                       -\n",            \
                           core, resStart);                                                         \
            printf ("Core %d : - End:           %-16d                       -\n", core,      \
                           (start + resLen - 1));                                                   \
        }                                                                                           \
        if (alignment == RM_RESOURCE_ALIGNMENT_UNSPECIFIED) {                                       \
            printf ("Core %d : - Alignment:     UNSPECIFIED                            -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - Alignment:     %-16d                       -\n",            \
                           core, alignment);                                                        \
        }                                                                                           \
        printf ("Core %d : -                                                       -\n",     \
                       core);                                                                       \
        if (state == check) {                                                                       \
            printf ("Core %d : - PASSED                                                -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - FAILED - Denial: %-6d                               -\n",   \
                           core, state);                                                            \
            testErrors++;                                                                           \
        }                                                                                           \
        printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        printf ("\n");                                                                       \
    } while(0);

#define NEGATIVE_PASS_CHECK(title, core, instName, resName, resStart, resLen, align, state, check)  \
    do {                                                                                            \
        int32_t start = resStart;                                                                   \
        int32_t alignment = align;                                                                  \
        char    titleMsg[] = title;                                                                 \
        printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        printf ("Core %d : %s\n", core, titleMsg);                                           \
        printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);               \
        printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                \
        if (start == RM_RESOURCE_BASE_UNSPECIFIED) {                                                \
            printf ("Core %d : - Start:         UNSPECIFIED                            -\n", \
                           core);                                                                   \
            printf ("Core %d : - Length:        %-16d                       -\n",            \
                           core, resLen);                                                           \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - Start:         %-16d                       -\n",            \
                           core, resStart);                                                         \
            printf ("Core %d : - End:           %-16d                       -\n", core,      \
                           (start + resLen - 1));                                                   \
        }                                                                                           \
        if (alignment == RM_RESOURCE_ALIGNMENT_UNSPECIFIED) {                                       \
            printf ("Core %d : - Alignment:     UNSPECIFIED                            -\n", \
                           core);                                                                   \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - Alignment:     %-16d                       -\n",            \
                           core, alignment);                                                        \
        }                                                                                           \
        printf ("Core %d : -                                                       -\n",     \
                       core);                                                                       \
        if (state != check) {                                                                       \
            printf ("Core %d : - PASSED - Denial: %-6d                               -\n",   \
                           core, state);                                                            \
        }                                                                                           \
        else {                                                                                      \
            printf ("Core %d : - FAILED - Expected Denial                              -\n", \
                           core);                                                                   \
            testErrors++;                                                                           \
        }                                                                                           \
        printf ("Core %d : ---------------------------------------------------------\n",     \
                       core);                                                                       \
        printf ("\n");                                                                       \
    } while(0);    

#define STATUS_PASS_CHECK(title, core, instName, resName, resStart, resLen, refCnt, state, check, expectRefCnt) \
    do {                                                                                                        \
        int32_t start = resStart;                                                                               \
        char    titleMsg[] = title;                                                                             \
        printf ("Core %d : ---------------------------------------------------------\n",                 \
                       core);                                                                                   \
        printf ("Core %d : %s\n", core, titleMsg);                                                       \
        printf ("Core %d : - Instance Name: %-32s       -\n", core, instName);                           \
        printf ("Core %d : - Resource Name: %-32s       -\n", core, resName);                            \
        printf ("Core %d : - Start:         %-16d                       -\n",                            \
                           core, resStart);                                                                     \
        printf ("Core %d : - End:           %-16d                       -\n", core,                      \
                           (start + resLen - 1));                                                               \
        printf ("Core %d : - Expected Owner Count: %-16d                -\n",                            \
                       core, expectRefCnt);                                                                     \
        printf ("Core %d : - Returned Owner Count: %-16d                -\n",                            \
                       core, refCnt);                                                                           \
        printf ("Core %d : -                                                       -\n", core);          \
        if ((state == check) && (refCnt == expectRefCnt)) {                                                     \
            printf ("Core %d : - PASSED                                                -\n", core);      \
        }                                                                                                       \
        else {                                                                                                  \
            if (refCnt != expectRefCnt) {                                                                       \
                printf ("Core %d : - FAILED - Owner Count Mismatch                         -\n",         \
                               core);                                                                           \
            }                                                                                                   \
            else {                                                                                              \
                printf ("Core %d : - FAILED - Denial: %-6d                               -\n",           \
                               core, state);                                                                    \
            }                                                                                                   \
            testErrors++;                                                                                       \
        }                                                                                                       \
        printf ("Core %d : ---------------------------------------------------------\n",                 \
                       core);                                                                                   \
        printf ("\n");                                                                                   \
    } while(0);

/**********************************************************************
 ********************** RM Test Data Structures ***********************
 **********************************************************************/

/* RM registered transport mapping structure */
typedef struct trans_map_entry_s {
    /* Registered RM transport handle */
    Rm_TransportHandle        transportHandle;
    /* Remote socket tied to the transport handle */
    sock_name_t              *remote_sock;
} Transport_MapEntry;

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter;
extern uint32_t rmFreeCounter;

/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* Core number */
uint16_t            coreNum;
/* Number of errors that occurred during the test */
uint16_t            testErrors;

/* Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client";

/* Client socket name */
char                rmClientSockName[] = "/var/run/rm/rm_client";

/* Client socket handle */
sock_h              rmClientSocket;

/* Client instance handles */
Rm_Handle           rmClientHandle = NULL;

/* Client instance service handles */
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

/* Test RM NameServer name */
char                nameServerNameFavQ[RM_NAME_MAX_CHARS]     = "My_Favorite_Queue";

/**********************************************************************
 *************************** Test Functions ***************************
 **********************************************************************/

Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rm_pkt = NULL;

    rm_pkt = calloc(1, sizeof(*rm_pkt));
    if (!rm_pkt) {
        error_msg("can't malloc for RM send message (err: %s)\n",
                  strerror(errno));
        return (NULL);
    }
    rm_pkt->pktLenBytes = pktSize;
    *pktHandle = rm_pkt;

    return(rm_pkt);
}

void transportFree (Rm_Packet *rm_pkt)
{
    if (rm_pkt) {
        free(rm_pkt);
    }
}

int32_t transportSend (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    sock_name_t *server_sock_name = (sock_name_t *)appTransport;
    Rm_Packet   *rm_pkt = (Rm_Packet *)pktHandle;
    
    if (sock_send(rmClientSocket, (char *)rm_pkt, (int) rm_pkt->pktLenBytes, server_sock_name)) {
        error_msg("send data failed\n");
    }
 
    return (0);
}

void transportReceive (void)
{
    int32_t             rm_result;
    int                 retval;
    int                 length = 0;
    sock_name_t         server_sock_addr;
    Rm_Packet          *rm_pkt = NULL;
    struct timeval      tv = {0, CLIENT_SOCK_TIMEOUT_USEC};
    struct sockaddr_un  server_addr;    
    
    retval = sock_wait(rmClientSocket, &length, &tv, -1);
    if (retval == -2) {
        /* Timeout */
        return;
    }
    else if (retval < 0) {
        error_msg("Error in reading from socket, error %d\n", retval);
        return;
    }
    
    if (length < ((int)sizeof(*rm_pkt))) {
        error_msg("invalid RM message length %d\n", length);
        return;
    }
    rm_pkt = calloc(1, length);
    if (!rm_pkt) {
        error_msg("can't malloc for recv'd RM message (err: %s)\n",
                  strerror(errno));
        return;
    }
    
    server_sock_addr.type = sock_addr_e;
    server_sock_addr.s.addr = &server_addr;
    retval = sock_recv(rmClientSocket, (char *)rm_pkt, length, &server_sock_addr);
    if (retval != length) {
        error_msg("recv RM pkt failed from socket, received = %d, expected = %d\n",
                  retval, length);
        return;
    }
    
    info_msg("received RM pkt of size %d bytes from %s\n", length, server_sock_addr.s.addr->sun_path);

    /* Provide packet to RM Server for processing */
    if ((rm_result = Rm_receivePacket(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle, rm_pkt))) {
        printf("RM failed to process received packet: %d\n", rm_result);
    }

    transportFree(rm_pkt);
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
            /* Check socket for response packets */
            transportReceive();
            
            qIndex++;
            if (qIndex == MAX_QUEUED_SERVICE_RESPONSES) {
                qIndex = 0;
            }
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

void cleanup(void)
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

    result = Rm_transportUnregister(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle);
    ERROR_CHECK(RM_OK, result, rmClientName, "Unregister of Server transport failed");

	sock_close(rmClientSocket);
 
    result = Rm_delete(rmClientHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, result, rmClientName, "Instance delete failed");

    printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    printf ("Core %d : ------------------ Memory Leak Check --------------------\n", coreNum);
    printf ("Core %d : -                       : malloc count   |   free count -\n", coreNum);
    printf ("Core %d : - Example Completion    :  %6d        |  %6d      -\n", coreNum,
                   rmMallocCounter, rmFreeCounter);
    finalMallocFree = rmMallocCounter - rmFreeCounter; 
    if (finalMallocFree > 0) {
        printf ("Core %d : - FAILED - %6d unfreed mallocs                       -\n",
                       coreNum, finalMallocFree);
        testErrors++;
    }
    else if (finalMallocFree < 0) {
        printf ("Core %d : - FAILED - %6d more frees than mallocs               -\n",
                       coreNum, -finalMallocFree);
        testErrors++;
    }
    else {
        printf ("Core %d : - PASSED                                                -\n",
                       coreNum);
    }
    printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    printf ("\n");  

    printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    printf ("Core %d : ------------------ Example Completion -------------------\n", coreNum);
    if (testErrors) {
        printf ("Core %d : - Test Errors: %-32d         -\n", coreNum, testErrors);
    }
    printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    printf ("\n"); 
}

void client_test(void)
{
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;    
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
}

void connection_setup(void)
{
    Rm_TransportCfg rmTransCfg;
    int32_t         rm_result;
    int             i;
    sock_name_t     sock_name;
    char            server_sock_name[] = RM_SERVER_SOCKET_NAME;
    
    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rmTransportMap[i].transportHandle = NULL;
    }

    sock_name.type = sock_name_e;
    sock_name.s.name = rmClientSockName;

    rmClientSocket = sock_open(&sock_name, 0, 0);
    if (!rmClientSocket) {
        error_msg("Client socket open failed\n");
        exit(EXIT_FAILURE);
    }

    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock = calloc(1, sizeof(sock_name_t));
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->type = sock_name_e;    
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name = calloc(1, strlen(server_sock_name)+1);
    strncpy(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name, server_sock_name, strlen(server_sock_name)+1);

    /* Register the Server with the Client instance */
    rmTransCfg.rmHandle = rmClientHandle;
    rmTransCfg.appTransportHandle = (Rm_AppTransportHandle) rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock;
    rmTransCfg.remoteInstType = Rm_instType_SERVER;
    rmTransCfg.transportCallouts.rmAllocPkt = transportAlloc;
    rmTransCfg.transportCallouts.rmSendPkt = transportSend;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle = Rm_transportRegister(&rmTransCfg, &rm_result);    
}

int main(int argc, char *argv[])
{
    int                 fd;
    struct stat         file_stat;
    char               *static_policy_addr = NULL;
    Rm_InitCfg          rmInitCfg;
    Rm_ServiceReqInfo   requestInfo;
    Rm_ServiceRespInfo  responseInfo;
    int32_t             result;

    printf ("*********************************************************\n");
    printf ("*************** RM Linux Client Testing *****************\n");
    printf ("*********************************************************\n");

    printf ("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    coreNum = 0;
    testErrors = 0;

    if (argc > 2)
    {
        error_msg("Invalid number of input arguments\n");
        exit(EXIT_FAILURE);
    }

    if (argc == 2){
        /* mmap static policy */
        fd = open(argv[1], O_RDONLY);
        if (fd == -1) {
            error_msg("Error opening static policy\n");
            exit(EXIT_FAILURE);
        }
        /* Obtain file size */
        if (fstat(fd, &file_stat) == -1) {
            error_msg("Error getting static policy size\n");
            exit(EXIT_FAILURE);
        }
        static_policy_addr = mmap(NULL, file_stat.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
        if (static_policy_addr == MAP_FAILED) {
            error_msg("mmap of static failed\n");
            exit(EXIT_FAILURE);
        }
    }   

    /* Initialize the RM Client - RM must be initialized before anything else in the system */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));
    rmInitCfg.instName = rmClientName;
    rmInitCfg.instType = Rm_instType_CLIENT;
    if (static_policy_addr) {
        rmInitCfg.instCfg.clientCfg.staticPolicy = (void *)static_policy_addr;
    }
    rmClientHandle = Rm_init(&rmInitCfg, &result);
    ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

    printf("\n\nInitialized %s\n\n", rmClientName);

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

    connection_setup();
    client_test();
    cleanup();

    return (0);
}

