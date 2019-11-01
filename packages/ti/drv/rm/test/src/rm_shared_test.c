/*
 *   rm_shared_test.c
 *
 *   Multicore Resource Manager test that uses the RM shared server to
 *   request resources from multiple cores.
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
#include <ti/drv/rm/rm_services.h>
#include <ti/drv/rm/rm_osal.h>

/**********************************************************************
 ********************** RM Shared Test Symbols ************************
 **********************************************************************/

#define PRINT_USED_RESOURCES         0  /* Make 1 and rebuild project to print resources allocated in example */

#define SYSINIT                      0
#define NUM_CORES                    2

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* Test cache align */
#define RM_TEST_MAX_CACHE_ALIGN      128

/* RM packet heap name */
#define RM_PKT_HEAP_NAME             "rmHeapBuf"
/* GateMP names used to synchronize the RM test tasks */
#define RM_SERVER_GATE_NAME          "rmServerGate"
#define RM_CLIENT_GATE_NAME          "rmClientGate"

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
 ****************** RM Shared Test Data Structures ********************
 **********************************************************************/

typedef struct {
    /* RM shared server handle */
    Rm_Handle sharedServerHandle;
    /** Padding required to make sure linker doesn't put something else
     * on the same cache line. */
    uint8_t pad[RM_TEST_MAX_CACHE_ALIGN - sizeof(Rm_Handle)];
} Test_SharedRmHandle;

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter[];
extern uint32_t rmFreeCounter[];

/* RM test Global Resource List (GRL) */
extern const char rmGRL[];
/* Example Linux DTB file provided to RM Server for automatic Linux Kernel resource extraction */
extern const char rmLinuxDtb[];
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
Task_Handle         rmStartupTskHandle;
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

/* RM Shared Server instance name (must match with RM Global Resource List (GRL) and policies */
char                rmServerName[RM_NAME_MAX_CHARS]        = "RM_Server";
/* First Shared Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmSharedClient1Name[RM_NAME_MAX_CHARS] = "RM_Client_Delegate";
/* Second Shared Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmSharedClient2Name[RM_NAME_MAX_CHARS] = "RM_Client";

/* RM shared server instance handle */
#pragma DATA_SECTION (rmSharedHandle, ".rmSharedHandleTest");
#pragma DATA_ALIGN (rmSharedHandle, RM_TEST_MAX_CACHE_ALIGN)
Test_SharedRmHandle rmSharedHandle = {NULL};

/* First Shared Client instance (local memory) */
Rm_Handle rmSharedClient1Handle = NULL;
/* Second Shared Client instance (local memory) */
Rm_Handle rmSharedClient2Handle = NULL;

/* RM shared server instance service handle */
Rm_ServiceHandle   *rmSharedServerServiceHandle        = NULL;
Rm_ServiceHandle   *rmSharedClient1ServiceHandle = NULL;
Rm_ServiceHandle   *rmSharedClient2ServiceHandle = NULL;

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
        reqInfo->callback.serviceCallback = NULL;  
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

    /* Cleanup all service ports, transport handles, RM instances, and IPC constructs */
    if (coreNum == SYSINIT) {
        result = Rm_serviceCloseHandle(rmSharedServerServiceHandle);
        ERROR_CHECK(RM_OK, result, rmServerName, "Service handle close failed");

        result = Rm_delete(rmSharedHandle.sharedServerHandle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, rmServerName, "Instance delete failed");

        /* Wait for remote core to verify it has deleted its Shared Clients */
        clientKey = GateMP_enter(clientGate);

        /* Only need to check for memory leaks from one Shared Server core since instance
         * was shared amongst all test cores */
        Osal_rmBeginMemAccess(rmMallocCounter, 128);
        Osal_rmBeginMemAccess(rmFreeCounter, 128);

        System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
        System_printf ("Core %d : ------------------ Memory Leak Check --------------------\n", coreNum);
        System_printf ("Core %d : -                       : malloc count   |   free count -\n", coreNum);
        System_printf ("Core %d : - Example Completion    :  %6d        |  %6d      -\n", coreNum,
                       rmMallocCounter[0], rmFreeCounter[0]);
        finalMallocFree = rmMallocCounter[0] - rmFreeCounter[0]; 
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
    }
    else {
        result = Rm_serviceCloseHandle(rmSharedClient1ServiceHandle);
        ERROR_CHECK(RM_OK, result, rmSharedClient1Handle, "Service handle close failed");
        result = Rm_serviceCloseHandle(rmSharedClient2ServiceHandle);
        ERROR_CHECK(RM_OK, result, rmSharedClient2Handle, "Service handle close failed");
        
        result = Rm_delete(rmSharedClient1Handle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, rmSharedClient1Name, "Instance delete failed");
        result = Rm_delete(rmSharedClient2Handle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, rmSharedClient2Name, "Instance delete failed");

        /* Signal to Shared Server core that the Shared Clients have been deleted */
        GateMP_leave(clientGate, clientKey);
    }

    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("Core %d : ------------------ Example Completion -------------------\n", coreNum);
    if (testErrors) {
        System_printf ("Core %d : - Test Errors: %-32d         -\n", coreNum, testErrors);
    }
    System_printf ("Core %d : ---------------------------------------------------------\n", coreNum);
    System_printf ("\n"); 

    BIOS_exit(0);
}

void rmServerTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceReqInfo  requestInfo;
    Rm_ServiceRespInfo responseInfo;
    Task_Params        taskParams;

    /* BEGIN testing UNSPECIFIED base and alignment requests on Server */               
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);       
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    
               
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, RM_RESOURCE_ALIGNMENT_UNSPECIFIED, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("---- Use Allocation w/ UNSPECIFIED Base & Alignment -----", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        RM_RESOURCE_ALIGNMENT_UNSPECIFIED, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 1, 200, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmServerName, resourceNameGpQ,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);        
    /* END testing UNSPECIFIED base and alignment requests on Server */      

    /* Create new NameServer object */                
    setRmRequest(&requestInfo, Rm_service_RESOURCE_MAP_TO_NAME, resourceNameGpQ, 
                 1002, 1, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo); 
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
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    NEGATIVE_PASS_CHECK("------- Use Allocation of Resource Owned by Linux -------", 
                        coreNum, rmServerName, resourceNameMemRegion,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);       

    /* BEGIN testing expansion/contraction of resource nodes with the AIF RX CH resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 14, 5, 0, NULL, RM_TEST_TRUE, &responseInfo);       
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Use Allocate) --", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);       
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 19, 31, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
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
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 34, 3, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      
 
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, resourceNameAifRxCh, 
                 28, 6, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 53, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("----- Resource Node Expand/Contract Testing (Free) ------", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);  
    /* END testing expansion/contraction of resource nodes with the AIF RX CH resource */  
    
    /* Test exclusive rights to an allocated resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 2, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    NEGATIVE_PASS_CHECK("------ Use Allocation of Exclusively Owned Resource -----", 
                        coreNum, rmServerName, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);       

    /* Wait for Client and Client Delegate to do their UNSPECIFIED allocates */
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    GateMP_leave(clientGate, clientKey);
    serverKey = GateMP_enter(serverGate);

    /* Test allocation of a resource twice from the same instance with init and use privileges.  Both
     * should be approved but the instance should have only one owner instance in resource's owner list */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 6543, 10, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);  
    POSITIVE_PASS_CHECK("-- Init/Use Allocate of Resource from Same Inst (Init) --", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 6543, 10, 0, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("--- Init/Use Allocate of Resource from Same Inst (Use) --", 
                        coreNum, rmServerName, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Allocate infrastructure queue taken by Linux kernel and shared with Rm_Client.  Expect error or denial. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameInfraQ, 
                 805, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    NEGATIVE_PASS_CHECK("- Init Allocation of Shared Linux and Client Resource  --", 
                        coreNum, rmServerName, resourceNameInfraQ,
                        805, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameAifRxCh, 
                 53, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("----- Status Check of Resources from Shared Server ------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 2, 1); 

    /* Get the status of a resource from Server */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameQosCluster, 
                 1, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedServerServiceHandle->Rm_serviceHandler(rmSharedServerServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("----- Status Check of Resources from Shared Server ------", 
                      coreNum, rmServerName, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 0, 0);

#if PRINT_USED_RESOURCES
    Rm_resourceStatus(rmSharedHandle.sharedServerHandle, RM_TEST_TRUE);
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

    serverKey = GateMP_enter(serverGate);  

    /* Retrieve a resource via a NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_GET_BY_NAME, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient1ServiceHandle->Rm_serviceHandler(rmSharedClient1ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("------- Retrieve Resource Via NameServer Object ---------", 
                        coreNum, rmSharedClient1Name, responseInfo.resourceName,
                        responseInfo.resourceBase, responseInfo.resourceLength, 
                        0, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Allocate the resource returned from the NameServer request */
    memset((void *)&requestInfo, 0, sizeof(Rm_ServiceReqInfo)); 
    requestInfo.type = Rm_service_RESOURCE_ALLOCATE_INIT;
    requestInfo.resourceName = responseInfo.resourceName;
    requestInfo.resourceBase = responseInfo.resourceBase;
    requestInfo.resourceLength = responseInfo.resourceLength;
    requestInfo.resourceNsName = NULL;
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("-------- Init Allocate Using Retrieved Resource ---------", 
                        coreNum, rmSharedClient2Name, responseInfo.resourceName,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Free resource via a NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_FREE, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("--- Free of Retrieved Resource Using NameServer Name ----", 
                        coreNum, rmSharedClient2Name, nameServerNameFavQ,
                        0, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    /* Delete the NameServer name */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nameServerNameFavQ, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);     
    POSITIVE_PASS_CHECK("--------------- Delete NameServer Object ----------------", 
                        coreNum, rmSharedClient2Name, nameServerNameFavQ,
                        0, 1, 0, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    GateMP_leave(clientGate, clientKey);
    GateMP_leave(serverGate, serverKey);
    clientKey = GateMP_enter(clientGate);
    serverKey = GateMP_enter(serverGate);    

    /* BEGIN testing expansion/contraction of resource nodes with the AIF RX CH resource */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAifRxCh, 
                 0, 6, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Use Allocate) --", 
                        coreNum, rmSharedClient2Name, resourceNameAifRxCh,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameAifRxCh, 
                 50, 7, 0, NULL, RM_TEST_TRUE, &responseInfo);        
    rmSharedClient1ServiceHandle->Rm_serviceHandler(rmSharedClient1ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("- Resource Node Expand/Contract Testing (Init Allocate) -", 
                        coreNum, rmSharedClient1Name, resourceNameAifRxCh,
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
    rmSharedClient1ServiceHandle->Rm_serviceHandler(rmSharedClient1ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmSharedClient1Name, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);    

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, 1, NULL, RM_TEST_TRUE, &responseInfo);      
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo); 
    POSITIVE_PASS_CHECK("---------- Use Allocation w/ UNSPECIFIED Base -----------", 
                        coreNum, rmSharedClient2Name, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameAccumCh, 
                 RM_RESOURCE_BASE_UNSPECIFIED, 2, RM_RESOURCE_ALIGNMENT_UNSPECIFIED, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("---- Use Allocation w/ UNSPECIFIED Base & Alignment -----", 
                        coreNum, rmSharedClient2Name, resourceNameAccumCh,
                        RM_RESOURCE_BASE_UNSPECIFIED, requestInfo.resourceLength, 
                        RM_RESOURCE_ALIGNMENT_UNSPECIFIED, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END testing allocations with UNSPECIFIED base and alignment values */    

    /* Allocate infrastructure queue shared between Linux kernel and Client */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameInfraQ, 
                 800, 1, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    POSITIVE_PASS_CHECK("-- Init Allocation of Shared Linux and Client Resource --", 
                        coreNum, rmSharedClient2Name, resourceNameInfraQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    /* BEGIN Allocating some resources without providing a callback function.  RM should block and not return until the result
     * is returned by the server. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 7000, 1, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("- Init Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmSharedClient2Name, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);      

    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7005, 25, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmSharedClient1ServiceHandle->Rm_serviceHandler(rmSharedClient1ServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmSharedClient1Name, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_USE, resourceNameGpQ, 
                 7010, 5, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("-- Use Allocation (RM Blocked Until Resource Returned) --", 
                        coreNum, rmSharedClient2Name, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     

    /* Init allocation of resource already owned by Client should return approved and there should only
     * be one instance of Client in resource's owner list. */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_ALLOCATE_INIT, resourceNameGpQ, 
                 7011, 1, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);   
    POSITIVE_PASS_CHECK("----- Use Allocation of Owned Resource (RM Blocked) -----", 
                        coreNum, rmSharedClient2Name, resourceNameGpQ,
                        requestInfo.resourceBase, requestInfo.resourceLength, 
                        requestInfo.resourceAlignment, responseInfo.serviceState, RM_SERVICE_APPROVED);     
    /* END Allocating some resources without providing a callback function.  RM should block and not return
     * until the result is returned by the server. */   

    /* BEGIN Getting the status of resources from the Shared Clients */
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 7012, 2, 0, NULL, RM_TEST_TRUE, &responseInfo);     
    rmSharedClient2ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("------- Resource Status Check from Shared Client --------", 
                      coreNum, rmSharedClient2Name, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 2, 1);   
    
    setRmRequest(&requestInfo, Rm_service_RESOURCE_STATUS, resourceNameGpQ, 
                 4025, 20, 0, NULL, RM_TEST_FALSE, &responseInfo);     
    rmSharedClient1ServiceHandle->Rm_serviceHandler(rmSharedClient2ServiceHandle->rmHandle, &requestInfo, &responseInfo);
    STATUS_PASS_CHECK("------- Resource Status Check from Shared Client --------", 
                      coreNum, rmSharedClient1Name, responseInfo.resourceName,
                      responseInfo.resourceBase, responseInfo.resourceLength, 
                      responseInfo.resourceNumOwners, responseInfo.instAllocCount,
                      responseInfo.serviceState, RM_SERVICE_APPROVED, 1, 0);        
    /* END Getting the status of resources from Client and CD */    

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
    Int           status;
    GateMP_Params gateParams;
    Task_Params   taskParams;

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
    }
 
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
    int32_t            result;

    System_printf ("*********************************************************\n");
    System_printf ("***************** RM Shared Server Test *****************\n");
    System_printf ("*********************************************************\n");

    System_printf ("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
    testErrors = 0;
    
    /* Initialize the heap in shared memory. Using IPC module to do that */ 
    System_printf("Core %d: Starting IPC...\n", coreNum);
    status = Ipc_start();
    if (status < 0) {
        System_abort("Ipc_start failed\n");
    }

    /* Initialize the RM instances - RM must be initialized before anything else in the system
     * Core 0: 1 RM Instance  - RM Server
     * Core 1: 2 RM Instances - RM Client Delegate
     *                          RM Client
     */
    if (coreNum == SYSINIT) {
        /* Create the Server instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmServerName;
        rmInitCfg.instType = Rm_instType_SHARED_SERVER;
        rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGRL;
        rmInitCfg.instCfg.serverCfg.linuxDtb = (void *)rmLinuxDtb;
        rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmGlobalPolicy;
        rmSharedHandle.sharedServerHandle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmServerName, "Initialization failed");

        /* Open Server service handle */
        rmSharedServerServiceHandle = Rm_serviceOpenHandle(rmSharedHandle.sharedServerHandle, &result);
        ERROR_CHECK(RM_OK, result, rmServerName, "Service handle open failed");

        Osal_rmEndMemAccess((void *)&rmSharedHandle, sizeof(Test_SharedRmHandle));
    }
    else {
        do {
            Osal_rmBeginMemAccess((void *)&rmSharedHandle, sizeof(Test_SharedRmHandle));
        } while (!rmSharedHandle.sharedServerHandle);

        /* Create the first Shared Client instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmSharedClient1Name;
        rmInitCfg.instType = Rm_instType_SHARED_CLIENT;
        rmInitCfg.instCfg.sharedClientCfg.sharedServerHandle = rmSharedHandle.sharedServerHandle;
        rmSharedClient1Handle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmSharedClient1Name, "Initialization failed");

        /* Open shared client service handle */
        rmSharedClient1ServiceHandle = Rm_serviceOpenHandle(rmSharedClient1Handle, &result);
        ERROR_CHECK(RM_OK, result, rmSharedClient1Name, "Service handle open failed");

        /* Create the second Shared Client instance */
        memset(&rmInitCfg, 0, sizeof(rmInitCfg));
        rmInitCfg.instName = rmSharedClient2Name;
        rmInitCfg.instType = Rm_instType_SHARED_CLIENT;
        rmInitCfg.instCfg.sharedClientCfg.sharedServerHandle = rmSharedHandle.sharedServerHandle;
        rmSharedClient2Handle = Rm_init(&rmInitCfg, &result);
        ERROR_CHECK(RM_OK, result, rmSharedClient2Name, "Initialization failed");

        /* Open shared client service handle */
        rmSharedClient2ServiceHandle = Rm_serviceOpenHandle(rmSharedClient2Handle, &result);
        ERROR_CHECK(RM_OK, result, rmSharedClient2Name, "Service handle open failed");
    }

    /* Create the RM startup task */
    System_printf("Core %d : Creating RM startup task...\n", coreNum);
    Task_Params_init (&taskParams);
    rmStartupTskHandle = Task_create (rmStartupTsk, &taskParams, NULL);

    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();

    return (0);    
}

