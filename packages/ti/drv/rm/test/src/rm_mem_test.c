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
#include <ti/ipc/GateMP.h>

/* BIOS Includes */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

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

#define SYSINIT                      0
#define NUM_CORES                    2

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* GateMP names used to synchronize instance tasks */
#define RM_SERVER_GATE_NAME            "serverGate"
#define RM_CLIENT_GATE_NAME            "clientGate"

/* Size of RM service response queue */
#define MAX_QUEUED_SERVICE_RESPONSES 10

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* GRL for testing multicore Server-Client interaction leaks */
extern const char rmGlobalResourceList[];
/* Global policy for testing multicore Server-Client interaction leaks */
extern const char rmDspPlusArmPolicy[];

/* GRL for testing Server-CD-Client interaction leaks */
extern const char rmGRL[];
/* Global policy for testing Server-CD-Client interaction leaks */
extern const char rmGlobalPolicy[];
/* Static policy for testing Server-CD-Client interaction leaks */
extern const char rmStaticPolicy[];
/* Example Linux DTB */
extern const char rmLinuxDtb[];

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter;
extern uint32_t rmFreeCounter;
extern int32_t rmByteAlloc;
extern int32_t rmByteFree;

extern void *Osal_rmMalloc (uint32_t num_bytes);
extern void  Osal_rmFree (void *ptr, uint32_t size);
extern void  Osal_rmBeginMemAccess(void *ptr, uint32_t size);
extern void  Osal_rmEndMemAccess(void *ptr, uint32_t size);

extern int setupRmTransConfig(uint32_t numTestCores, uint32_t systemInitCore, Task_FuncPtr testTask);
extern int deleteRmTrans(void);

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

/* GateMP handles used to synchronize instance tasks */
GateMP_Handle       serverGateHandle = NULL;
GateMP_Handle       clientGateHandle = NULL;
/* GateMP keys */
IArg                serverKey;
IArg                clientKey;

/* RM instance variables */
Rm_Handle           rmHandle = NULL;
char                rmInstName[RM_NAME_MAX_CHARS];

/* Malloc/Free tracking variables */
uint32_t            sMalloc, mMalloc, sByteAlloc, mByteAlloc;
uint32_t            sFree, mFree, sByteFree, mByteFree;

/* RM response info queue used to store service responses received via the callback function */
Rm_ServiceRespInfo  responseInfoQueue[MAX_QUEUED_SERVICE_RESPONSES];

/* rmGRL resource names (must match resource node names in rmGRL, rmGlobalPolicy, and rmStaticPolicy */
char                resNameGpQ[RM_NAME_MAX_CHARS]   = "gp-queue";
char                resNameQos[RM_NAME_MAX_CHARS]   = "qos-cluster";

/* rmGlobalResourceList names (must match resource node names in rmGlobalResourceList and rmDspPlusArmPolicy */
char                resNameMcGpQ[RM_NAME_MAX_CHARS] = "GENERAL_PURPOSE_QUEUE-qm1";

/* Test RM NameServer name */
char                nsNameFavQ[RM_NAME_MAX_CHARS]   = "My_Favorite_Queue";

/* RM initialization sync point */
#pragma DATA_SECTION (isRmInitialized, ".rm");
#pragma DATA_ALIGN (isRmInitialized, 128)
volatile int8_t   isRmInitialized[128];

/**********************************************************************
 ***************************** Test Macros ****************************
 **********************************************************************/

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)                  \
    if (resultVal != checkVal) {                                                \
        char errorMsgToPrint[] = printMsg;                                      \
        System_printf("Error Core %d : %s : ", coreNum, rmInstName);            \
        System_printf("%s with error code : %d\n", errorMsgToPrint, resultVal); \
        testErrors++;                                                           \
        System_abort("Test Failure\n");                                         \
    }

#define MEM_TEST_START_STORE() sMalloc = rmMallocCounter; sFree = rmFreeCounter; \
                               sByteAlloc = rmByteAlloc;  sByteFree = rmByteFree;
#define MEM_TEST_MID_STORE()   mMalloc = rmMallocCounter; mFree = rmFreeCounter; \
                               mByteAlloc = rmByteAlloc;  mByteFree = rmByteFree;
#define MEM_TEST_END_PRINT(title, sApiOrFunc, mApiOrFunc, eApiOrFunc)                                           \
    do {                                                                                                        \
        char    titleMsg[] = title;                                                                             \
        char    sApiOrFuncMsg[] = sApiOrFunc;                                                                   \
        char    mApiOrFuncMsg[] = mApiOrFunc;                                                                   \
        char    eApiOrFuncMsg[] = eApiOrFunc;                                                                   \
        int32_t mallocFreeBalance = (rmMallocCounter - sMalloc) - (rmFreeCounter - sFree);                      \
        int32_t mallocByteBalance = (rmByteAlloc - sByteAlloc) - (rmByteFree - sByteFree);                      \
                                                                                                                \
        System_printf ("Core %d : ---------------------------------------------------------------------\n",     \
                       coreNum);                                                                                \
        System_printf ("Core %d : ------%s------\n", coreNum, titleMsg);                                        \
        System_printf ("Core %d : - API/Functionality     :   malloc count (b)  |    free count (b)   -\n",     \
                       coreNum);                                                                                \
        System_printf ("Core %d : - %s :  %6d (%8d)  |  %6d (%8d)  -\n", coreNum, sApiOrFuncMsg,                \
                       sMalloc, sByteAlloc, sFree, sByteFree);                                                  \
        System_printf ("Core %d : - %s :  %6d (%8d)  |  %6d (%8d)  -\n", coreNum, mApiOrFuncMsg,                \
                       mMalloc, mByteAlloc, mFree, mByteFree);                                                  \
        System_printf ("Core %d : - %s :  %6d (%8d)  |  %6d (%8d)  -\n", coreNum,                               \
                       eApiOrFuncMsg, rmMallocCounter, rmByteAlloc, rmFreeCounter, rmByteFree);                 \
        if ((mallocFreeBalance > 0) || (mallocByteBalance >0)) {                                                \
            System_printf ("Core %d : - FAILED - %6d (%8d) unfreed memory                         -\n",         \
                           coreNum, mallocFreeBalance, mallocByteBalance);                                      \
            testErrors++;                                                                                       \
        }                                                                                                       \
        else if ((mallocFreeBalance < 0) || (mallocByteBalance < 0)) {                                          \
            System_printf ("Core %d : - FAILED - %6d (%8d) over freeing memory                    -\n",         \
                           coreNum, -mallocFreeBalance, -mallocByteBalance);                                    \
            testErrors++;                                                                                       \
        }                                                                                                       \
        else {                                                                                                  \
            System_printf ("Core %d : - PASSED                                                            -\n", \
                           coreNum);                                                                            \
        }                                                                                                       \
        System_printf ("Core %d : ---------------------------------------------------------------------\n",     \
                       coreNum);                                                                                \
        System_printf ("\n");                                                                                   \
    } while(0)


/**********************************************************************
 *************************** Test Functions ***************************
 **********************************************************************/

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
    memcpy((void *)&responseInfoQueue[qIndex], (void *)serviceResponse, sizeof(responseInfoQueue[qIndex]));
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

        memcpy((void *)respInfo, (void *)&responseInfoQueue[qIndex], sizeof(*respInfo));
        memset((void *)&responseInfoQueue[qIndex], 0, sizeof(responseInfoQueue[qIndex]));
    }
}

void setRmRequest(Rm_ServiceReqInfo *reqInfo, Rm_ServiceType type, const char *resName, int32_t resBase,
                  uint32_t resLen, int32_t resAlign, const char *nsName, int setCallback, Rm_ServiceRespInfo *respInfo)
{                                                                                
    memset((void *)reqInfo, 0, sizeof(*reqInfo));                                        
    reqInfo->type = type;                                                                           
    reqInfo->resourceName = resName;                                                                
    reqInfo->resourceBase = resBase;                                                                
    reqInfo->resourceLength = resLen;                                                               
    reqInfo->resourceAlignment = resAlign;                                                          
    reqInfo->resourceNsName = nsName;
    if (setCallback) {
        reqInfo->callback.serviceCallback = serviceCallback;  
    }
    memset((void *)respInfo, 0, sizeof(*respInfo));                                     
}

void rmCleanupTsk(UArg arg0, UArg arg1)
{
    int32_t rmResult;
    int32_t finalMallocFree, finalByteFree;    
    
    /* Delete the RM test tasks */
    System_printf("Core %d: Deleting RM startup task...\n", coreNum);
    if (rmStartupTskHandle) {
       Task_delete(&rmStartupTskHandle);
       /* Set the task handle to be NULL so that the delete only occurs once */
       rmStartupTskHandle = NULL;
    }  
    
    if (coreNum == SYSINIT) {
        if (rmServerTskHandle) {
            System_printf("Core %d: Deleting RM server task...\n", coreNum);
            Task_delete(&rmServerTskHandle);
            /* Set the task handle to be NULL so that the delete only occurs once */
            rmServerTskHandle = NULL;
        }
    }
    else {
        if (rmClientTskHandle) {
            System_printf("Core %d: Deleting RM client task...\n", coreNum);        
            Task_delete(&rmClientTskHandle);
            /* Set the task handle to be NULL so that the delete only occurs once */
            rmClientTskHandle = NULL;
        }
    }

    /* Cleanup RM instances and check final malloc/free numbers */
    rmResult = deleteRmTrans();
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Transport cleanup failed");    
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");

    System_printf ("Core %d : ---------------------------------------------------------------------\n", coreNum);
    System_printf ("Core %d : ------------------------- Example Completion ------------------------\n", coreNum);
    System_printf ("Core %d : - API/Functionality     :   malloc count (b)  |    free count (b)   -\n", coreNum);
    System_printf ("Core %d : - Example Completion    :  %6d (%8d)  |  %6d (%8d)  -\n", coreNum,
                   rmMallocCounter, rmByteAlloc, rmFreeCounter, rmByteFree);
    finalMallocFree = rmMallocCounter - rmFreeCounter;
    finalByteFree = rmByteAlloc - rmByteFree;
    if ((finalMallocFree > 0) || (finalByteFree > 0)) {
        System_printf ("Core %d : - FAILED - %6d (%8d) unfreed memory                         -\n",
                       coreNum, finalMallocFree, finalByteFree);
        testErrors++;
    }
    else if ((finalMallocFree < 0) || (finalByteFree < 0)) {
        System_printf ("Core %d : - FAILED - %6d (%8d) over freeing memory                    -\n",
                       coreNum, -finalMallocFree, finalByteFree);
        testErrors++;
    }
    else {
        System_printf ("Core %d : - PASSED                                                            -\n",
                       coreNum);
    }    
    if (testErrors) {
        System_printf ("Core %d : -                                                                   -\n", coreNum); 
        System_printf ("Core %d : - Test Errors: %-32d                     -\n", coreNum, testErrors);
    }    
    System_printf ("Core %d : ---------------------------------------------------------------------\n", coreNum);
    System_printf ("\n");  

    BIOS_exit(0);
}

void rmServerTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceHandle   *serviceHandle;
    Task_Params         taskParams;
    int32_t             rmResult;

    /* Open Client service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");


    MEM_TEST_START_STORE();
    /* Leave the server gate to let client task request resource */
    GateMP_leave(serverGateHandle, serverKey);
    /* Block until Client finishes allocating the resource */
    clientKey = GateMP_enter(clientGateHandle);
    MEM_TEST_MID_STORE();  
    GateMP_leave(clientGateHandle, clientKey);
    /* Block until Client finishes freeing the resource */
    serverKey = GateMP_enter(serverGateHandle);
    MEM_TEST_END_PRINT("------- Remote Alloc/Free From Client (Blocking) --------",
                       "Pre Client Alloc     ", "Post Client Alloc Req", "Post Client Free     ");   


    MEM_TEST_START_STORE();
    /* Leave the server gate to let client task request resources */
    GateMP_leave(serverGateHandle, serverKey);
    /* Block until Client finishes allocating the resources */
    clientKey = GateMP_enter(clientGateHandle); 
    MEM_TEST_MID_STORE();
    GateMP_leave(clientGateHandle, clientKey);
    /* Block until Client finishes freeing the resources */
    serverKey = GateMP_enter(serverGateHandle);           
    MEM_TEST_END_PRINT("--- Remote Multiple Alloc/Free From Client (Blocking) ---",
                       "Pre Client Alloc     ", "Post Client Alloc Req", "Post Client Free     ");    


    MEM_TEST_START_STORE();
    /* Leave the server gate to let client task map a resource in the NameServer */
    GateMP_leave(serverGateHandle, serverKey);
    /* Block until Client finishes mapping resource */
    clientKey = GateMP_enter(clientGateHandle);
    MEM_TEST_MID_STORE();
    GateMP_leave(clientGateHandle, clientKey);
    /* Block until Client finishes unmapping resource */
    serverKey = GateMP_enter(serverGateHandle);           
    MEM_TEST_END_PRINT("-- Remote NameServer Map/Unmap From Client (Blocking) ---",
                       "Pre Client NS Map    ", "Post Client NS Map   ", "Post Client NS Unmap ");    


    MEM_TEST_START_STORE();
    /* Leave the server gate to let client task request resources */
    GateMP_leave(serverGateHandle, serverKey);
    /* Block until Client finishes allocating the resources */
    clientKey = GateMP_enter(clientGateHandle); 
    MEM_TEST_MID_STORE();
    GateMP_leave(clientGateHandle, clientKey);
    /* Block until Client finishes freeing the resources */
    serverKey = GateMP_enter(serverGateHandle);           
    MEM_TEST_END_PRINT("- Remote Multiple Alloc/Free From Client (Non-Blocking) -",
                       "Pre Client Alloc     ", "Post Client Alloc Req", "Post Client Free     "); 


    Rm_serviceCloseHandle(serviceHandle);
    
    /* Create the RM cleanup task. */
    System_printf("Core %d: Creating RM cleanup task...\n", coreNum);
    Task_Params_init (&taskParams);
    Task_create (rmCleanupTsk, &taskParams, NULL);
}

void rmClientTsk(UArg arg0, UArg arg1)
{
    Rm_ServiceHandle   *serviceHandle;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;    
    int32_t             rmResult;    
    Task_Params         taskParams;

    /* Open Client service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");


    /* Block until server is ready to receive request */
    serverKey = GateMP_enter(serverGateHandle);
    MEM_TEST_START_STORE();      
    /* Allocate a resource to add a node to a resource tree */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 896, 1, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    /* Let server save memory usage for alloc */
    GateMP_leave(clientGateHandle, clientKey);    
    MEM_TEST_MID_STORE();
    /* Block until Server saves malloc/free info */
    clientKey = GateMP_enter(clientGateHandle);     
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 896, 1, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");
    MEM_TEST_END_PRINT("-------------- Client Alloc/Free (Blocking) -------------",
                       "Pre Alloc            ", "Post Alloc Request   ", "Post Free            ");
    /* Let server print memory usage */
    GateMP_leave(serverGateHandle, serverKey);


    /* Block until server is ready to receive request */
    serverKey = GateMP_enter(serverGateHandle);
    MEM_TEST_START_STORE();       
    /* Perform multiple allocs to force the tree to combine nodes */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 900, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 2000, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 1000, 1000, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 950, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed"); 
    /* Let server save memory usage for alloc */
    GateMP_leave(clientGateHandle, clientKey);
    MEM_TEST_MID_STORE();    
    /* Block until Server saves malloc/free info */
    clientKey = GateMP_enter(clientGateHandle);      
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 1000, 500, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");     
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 1500, 550, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");   
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 900, 100, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");  
    MEM_TEST_END_PRINT("--------- Client Multiple Alloc/Free (Blocking) ---------",
                       "Pre Alloc            ", "Post Alloc Requests  ", "Post Free            ");
    /* Let server print memory usage */
    GateMP_leave(serverGateHandle, serverKey);
    

    /* Block until server is ready to receive request */
    serverKey = GateMP_enter(serverGateHandle);
    MEM_TEST_START_STORE();       
    /* Map a resource in the NameServer */
    setRmRequest(&request, Rm_service_RESOURCE_MAP_TO_NAME, resNameMcGpQ, 
                 5000, 1, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "NameServer map failed");
    /* Let server save memory usage for alloc */
    GateMP_leave(clientGateHandle, clientKey);
    MEM_TEST_MID_STORE();    
    /* Block until Server saves malloc/free info */
    clientKey = GateMP_enter(clientGateHandle);      
    setRmRequest(&request, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "NameServer unmap failed");
    MEM_TEST_END_PRINT("-------- Client NameServer Map/Unmap (Blocking) ---------",
                       "Pre NS Map           ", "Name Mapped to NS    ", "Name Unmapped from NS");       
    /* Let server print memory usage */
    GateMP_leave(serverGateHandle, serverKey);


    /* Block until server is ready to receive request */
    serverKey = GateMP_enter(serverGateHandle);
    MEM_TEST_START_STORE();       
    /* Perform multiple allocs without RM blocking to force the tree to combine nodes */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 900, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 2000, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 1000, 1000, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameMcGpQ, 
                 950, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed"); 
    /* Let server save memory usage for alloc */
    GateMP_leave(clientGateHandle, clientKey);
    MEM_TEST_MID_STORE();    
    /* Block until Server saves malloc/free info */
    clientKey = GateMP_enter(clientGateHandle);      
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 1000, 500, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");     
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 1500, 550, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");   
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameMcGpQ, 
                 900, 100, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    waitForResponse(&response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");  
    MEM_TEST_END_PRINT("------- Client Multiple Alloc/Free (Non-Blocking) -------",
                       "Pre Alloc            ", "Post Alloc Requests  ", "Post Free            ");
    /* Let server print memory usage */
    GateMP_leave(serverGateHandle, serverKey);


    Rm_serviceCloseHandle(serviceHandle);
    
    /* Create the RM cleanup task. */
    System_printf("Core %d: Creating RM cleanup task...\n", coreNum);
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
        serverGateHandle = GateMP_create(&gateParams);

        serverKey = GateMP_enter(serverGateHandle);  

        do {
            status = GateMP_open(RM_CLIENT_GATE_NAME, &clientGateHandle);
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
        clientGateHandle = GateMP_create(&gateParams);

        clientKey = GateMP_enter(clientGateHandle);  
        
        do {
            status = GateMP_open(RM_SERVER_GATE_NAME, &serverGateHandle);
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
        System_printf("Core %d: Creating RM server task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 1;
        rmServerTskHandle = Task_create (rmServerTsk, &taskParams, NULL);
    }
    else if (coreNum) {
        System_printf("Core %d: Creating RM client task...\n", coreNum);
        Task_Params_init (&taskParams);
        taskParams.priority = 1;
        rmClientTskHandle = Task_create (rmClientTsk, &taskParams, NULL);
    }
}

Rm_Packet *rmLocalPktAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rmPkt = NULL;

    rmPkt = Osal_rmMalloc(sizeof(*rmPkt));
    rmPkt->pktLenBytes = pktSize;
    *pktHandle = (Rm_PacketHandle)rmPkt;
    return (rmPkt);
}

int32_t rmLocalPktSendRcv (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    Rm_TransportHandle  transportHandle = (Rm_TransportHandle)appTransport;
    Rm_Packet          *rmPkt = (Rm_Packet *) pktHandle;  
    int32_t             status;    

    /* Provide packet to remote instance for processing */
    if (status = Rm_receivePacket(transportHandle, rmPkt)) {
        System_printf("Error Core %d : Receiving RM packet : %d\n", coreNum, status);
        testErrors++;
    }

    /* Free the packet */
    Osal_rmFree(rmPkt, sizeof(*rmPkt));

    return (status);
}

void testServer(void)
{
    Rm_InitCfg          rmInitCfg;
    Rm_ServiceHandle   *serviceHandle;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    int32_t             rmResult;


    MEM_TEST_START_STORE(); 
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (rmInstName, "RM_Server");
    rmInitCfg.instName = rmInstName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGRL;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmGlobalPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");    
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("------------------ Server Inst Init/Del -----------------",
                       "Pre Rm_Init (Server) ", "Post Rm_Init (Server)", "Post Rm_Delete       ");    


    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    MEM_TEST_START_STORE();
    /* Open service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");
    MEM_TEST_MID_STORE();
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle close failed");
    MEM_TEST_END_PRINT("--------------- Open/Close Service Handle ---------------",
                       "Pre Service Hnd Open ", "Service Hnd Opened   ", "Service Hnd Closed   ");    
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");        


    MEM_TEST_START_STORE(); 
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    /* Open service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");
    /* Allocate a resource to add a node to a resource tree */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 896, 1, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    MEM_TEST_MID_STORE();
    /* Delete instance making sure allocation cleaned up properly from resource tree */
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle close failed");        
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("-------------- Server Inst Init/Del w/ Alloc ------------",
                       "Pre Rm_Init (Server) ", "Post Alloc Req       ", "Post Rm_Delete       ");
    

    /* Service request memory tests */
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    /* Open service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");


    MEM_TEST_START_STORE();        
    /* Allocate a resource to add a node to a resource tree */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 896, 1, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    MEM_TEST_MID_STORE();
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 896, 1, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");        
    MEM_TEST_END_PRINT("-------------------- Server Alloc/Free ------------------",
                       "Pre Alloc            ", "Post Alloc Request   ", "Post Free            ");    


    MEM_TEST_START_STORE();        
    /* Perform multiple allocs to force the tree to combine nodes */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 900, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 2000, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 1000, 1000, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 950, 50, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Allocate failed");   
    MEM_TEST_MID_STORE();
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 1000, 500, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");     
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 1500, 550, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");   
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 900, 100, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "Free failed");           
    MEM_TEST_END_PRINT("--------------- Server Multiple Alloc/Free --------------",
                       "Pre Alloc            ", "Post Alloc Requests  ", "Post Free            ");    


    MEM_TEST_START_STORE();        
    /* Map a resource in the NameServer */
    setRmRequest(&request, Rm_service_RESOURCE_MAP_TO_NAME, resNameGpQ, 
                 5000, 1, 0, nsNameFavQ, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "NameServer map failed");
    MEM_TEST_MID_STORE();
    setRmRequest(&request, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nsNameFavQ, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, rmInstName, "NameServer unmap failed");
    MEM_TEST_END_PRINT("-------------- Server NameServer Map/Unmap --------------",
                       "Pre NS Map           ", "Name Mapped to NS    ", "Name Unmapped from NS");
    

    /* Close server instance after tests complete */
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle close failed");    
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed"); 
}

void testCd(void)
{
    Rm_InitCfg          rmInitCfg;
    Rm_ServiceHandle   *serviceHandle;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    int32_t             rmResult;

    MEM_TEST_START_STORE();      
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (rmInstName, "RM_Client_Delegate");
    rmInitCfg.instName = rmInstName;
    rmInitCfg.instType = Rm_instType_CLIENT_DELEGATE;
    rmInitCfg.instCfg.cdCfg.cdPolicy = (void *)rmGlobalPolicy;    
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_WARNING_CD_INSTANCE_NOT_STABLE, rmResult, rmInstName, "Initialization failed");
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("--- Client Delegate Inst Init/Del (No Static Policy) ----",
                       "Pre Rm_Init (CD)     ", "Post Rm_Init (CD)    ", "Post Rm_Delete       ");


    MEM_TEST_START_STORE();
    rmInitCfg.instCfg.cdCfg.cdPolicy = (void *)rmStaticPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_WARNING_CD_INSTANCE_NOT_STABLE, rmResult, rmInstName, "Initialization failed");
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("---- Client Delegate Inst Init/Del (Static Policy) ------",
                       "Pre Rm_Init (CD)     ", "Post Rm_Init (CD)    ", "Post Rm_Delete       ");    


    MEM_TEST_START_STORE();
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_WARNING_CD_INSTANCE_NOT_STABLE, rmResult, rmInstName, "Initialization failed");
    /* Open service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");
    /* Static allocation */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameQos, 
                 0, 1, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    MEM_TEST_MID_STORE();
    /* Delete instance with deletion of pending transactions */
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle close failed");    
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("--- CD Inst Init/Del (Static Policy w/ Static Alloc) ----",
                       "Pre Rm_Init (CD)     ", "Post Static Alloc Req", "Post Rm_Delete       "); 
}

void testClient(void)
{
    Rm_InitCfg          rmInitCfg;
    Rm_ServiceHandle   *serviceHandle;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    int32_t             rmResult;

    MEM_TEST_START_STORE();      
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (rmInstName, "RM_Client");
    rmInitCfg.instName = rmInstName;
    rmInitCfg.instType = Rm_instType_CLIENT;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("-------- Client Inst Init/Del (No Static Policy) --------",
                       "Pre Rm_Init (Client) ", "Post Rm_Init (Client)", "Post Rm_Delete       ");


    MEM_TEST_START_STORE();
    rmInitCfg.instCfg.clientCfg.staticPolicy = (void *)rmStaticPolicy;
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("--------- Client Inst Init/Del (Static Policy) ----------",
                       "Pre Rm_Init (Client) ", "Post Rm_Init (Client)", "Post Rm_Delete       ");    


    MEM_TEST_START_STORE();
    rmHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
    /* Open service handle */
    serviceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle open failed");
    /* Static allocation */
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameQos, 
                 0, 1, 0, NULL, RM_TEST_TRUE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    MEM_TEST_MID_STORE();
    /* Delete instance with deletion of pending transactions */
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Service handle close failed");    
    rmResult = Rm_delete(rmHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, rmInstName, "Delete failed");
    MEM_TEST_END_PRINT("- Client Inst Init/Del (Static Policy w/ Static Alloc) --",
                       "Pre Rm_Init (Client) ", "Post Static Alloc Req", "Post Rm_Delete       ");   
}

/* Test features for memory leaks using a Server, CD, and Client all running on a single DSP core. 
 * The transport implemented between the instances is just direct function calls */
void testServerCdClient(void)
{
    Rm_InitCfg          rmInitCfg;
    char                serverName[RM_NAME_MAX_CHARS];
    char                cdName[RM_NAME_MAX_CHARS];
    char                clientName[RM_NAME_MAX_CHARS];
    Rm_Handle           serverHandle, cdHandle, clientHandle;
    Rm_TransportCfg     transCfg;
    Rm_TransportReCfg   transReCfg;
    Rm_TransportHandle  serverToCd, cdToServer, cdToClient, clientToCd;
    Rm_ServiceHandle   *serviceHandle;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    int32_t             rmResult;


    MEM_TEST_START_STORE();
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (serverName, "RM_Server");
    rmInitCfg.instName = serverName;
    rmInitCfg.instType = Rm_instType_SERVER;
    rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGRL;
    rmInitCfg.instCfg.serverCfg.linuxDtb = (void *)rmLinuxDtb;
    rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmGlobalPolicy;    
    serverHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Initialization failed");  
    MEM_TEST_MID_STORE();
    rmResult = Rm_delete(serverHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Delete failed");
    MEM_TEST_END_PRINT("----------- Server Inst Init/Del w/ Linux Dtb -----------",
                       "Pre Rm_Init (Server) ", "Post Rm_Init (Server)", "Post Rm_Delete       ");

    /* Init Server */
    serverHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Initialization failed");
    /* Init CD */
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (cdName, "RM_Client_Delegate");
    rmInitCfg.instName = cdName;
    rmInitCfg.instType = Rm_instType_CLIENT_DELEGATE;
    rmInitCfg.instCfg.cdCfg.cdPolicy = (void *)rmGlobalPolicy;
    cdHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_WARNING_CD_INSTANCE_NOT_STABLE, rmResult, cdName, "Initialization failed"); 
    /* Init Client */
    memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
    System_sprintf (clientName, "RM_Client");
    rmInitCfg.instName = clientName;
    rmInitCfg.instType = Rm_instType_CLIENT;      
    clientHandle = Rm_init(&rmInitCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Initialization failed");

    /* Connect Server transports */
    memset((void *)&transCfg, 0, sizeof(transCfg));
    transCfg.rmHandle = serverHandle;
    transCfg.remoteInstType = Rm_instType_CLIENT_DELEGATE;
    transCfg.appTransportHandle = NULL;
    transCfg.transportCallouts.rmAllocPkt = rmLocalPktAlloc;
    transCfg.transportCallouts.rmSendPkt = rmLocalPktSendRcv;
    serverToCd = Rm_transportRegister(&transCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Client Delegate transport registration failed");
    /* Connect Client transports */
    transCfg.rmHandle = clientHandle;
    transCfg.remoteInstType = Rm_instType_CLIENT_DELEGATE;
    clientToCd = Rm_transportRegister(&transCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Client Delegate transport registration failed");     
    /* Connect CD transports */
    transCfg.rmHandle = cdHandle;
    transCfg.remoteInstType = Rm_instType_SERVER;
    transCfg.appTransportHandle = (Rm_AppTransportHandle) serverToCd;
    cdToServer = Rm_transportRegister(&transCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, cdName, "Server transport registration failed");
    transCfg.rmHandle = cdHandle;
    transCfg.remoteInstType = Rm_instType_CLIENT;
    transCfg.appTransportHandle = (Rm_AppTransportHandle) clientToCd;
    cdToClient = Rm_transportRegister(&transCfg, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, cdName, "Client transport registration failed");
    /* Reconfigure the Server and Client transports to CD with the CD transport handles for proper routing
     * in the rmLocalPktSendRcv function */
    memset((void*)&transReCfg, 0, sizeof(transReCfg));
    transReCfg.appTransportHandle =(Rm_AppTransportHandle) cdToServer;
    rmResult = Rm_transportReconfig(serverToCd, &transReCfg);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Client Delegate transport registration failed");
    transReCfg.appTransportHandle = (Rm_AppTransportHandle) cdToClient;
    rmResult = Rm_transportReconfig(clientToCd, &transReCfg);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Client Delegate transport registration failed");    

    /* Open service handle on Client to send requests to Server via Client Delegate */
    serviceHandle = Rm_serviceOpenHandle(clientHandle, &rmResult);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Service handle open failed");

  
    /* Allocate a resource */
    MEM_TEST_START_STORE();        
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 896, 1, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed");   
    MEM_TEST_MID_STORE();    
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 896, 1, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Free failed");
    MEM_TEST_END_PRINT("--------------- Client Alloc/Free (via CD) --------------",
                       "Pre Alloc            ", "Post Alloc Request   ", "Post Free            ");

      
    /* Perform multiple allocs to force the tree to combine nodes */
    MEM_TEST_START_STORE(); 
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 900, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 2000, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 1000, 1000, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 950, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed"); 
    MEM_TEST_MID_STORE();         
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 1000, 500, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Free failed");     
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 1500, 550, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Free failed");   
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 900, 100, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Free failed");  
    MEM_TEST_END_PRINT("---------- Client Multiple Alloc/Free (via CD) ----------",
                       "Pre Alloc            ", "Post Alloc Requests  ", "Post Free            ");
    
     
    /* Map a resource in the NameServer */
    MEM_TEST_START_STORE();  
    setRmRequest(&request, Rm_service_RESOURCE_MAP_TO_NAME, resNameGpQ, 
                 5000, 1, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "NameServer map failed");
    MEM_TEST_MID_STORE();        
    setRmRequest(&request, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "NameServer unmap failed");
    MEM_TEST_END_PRINT("---------- Client NameServer Map/Unmap (via CD) ---------",
                       "Pre NS Map           ", "Name Mapped to NS    ", "Name Unmapped from NS");


    /* Check status of resource directly and through NameServer name */
    MEM_TEST_START_STORE(); 
    setRmRequest(&request, Rm_service_RESOURCE_ALLOCATE_INIT, resNameGpQ, 
                 900, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Allocate failed");
    setRmRequest(&request, Rm_service_RESOURCE_MAP_TO_NAME, resNameGpQ, 
                 900, 1, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "NameServer map failed");
    setRmRequest(&request, Rm_service_RESOURCE_STATUS, resNameGpQ, 
                 900, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Status check failed");
    setRmRequest(&request, Rm_service_RESOURCE_STATUS, resNameGpQ, 
                 0, 0, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Status check (via NS name) failed");
    MEM_TEST_MID_STORE();         
    setRmRequest(&request, Rm_service_RESOURCE_UNMAP_NAME, NULL, 
                 0, 0, 0, nsNameFavQ, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "NameServer unmap failed");  
    setRmRequest(&request, Rm_service_RESOURCE_FREE, resNameGpQ, 
                 900, 50, 0, NULL, RM_TEST_FALSE, &response);
    serviceHandle->Rm_serviceHandler(serviceHandle->rmHandle, &request, &response);
    ERROR_CHECK(RM_SERVICE_APPROVED, response.serviceState, clientName, "Free failed");  
    MEM_TEST_END_PRINT("--------- Client Check Resource Status (via CD) ---------",
                       "Pre Alloc            ", "Post Status Requests ", "Post Free            ");


    /* Cleanup */
    rmResult = Rm_serviceCloseHandle(serviceHandle);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Service handle close failed");       
    rmResult = Rm_transportUnregister(serverToCd);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Client Delegate transport unregister failed");
    rmResult = Rm_transportUnregister(cdToServer);
    ERROR_CHECK(RM_OK, rmResult, cdName, "Server transport unregister failed");
    rmResult = Rm_transportUnregister(cdToClient);
    ERROR_CHECK(RM_OK, rmResult, cdName, "Client transport unregister failed");
    rmResult = Rm_transportUnregister(clientToCd);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Client Delegate transport unregister failed");
    rmResult = Rm_delete(serverHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, serverName, "Delete failed");
    rmResult = Rm_delete(cdHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, cdName, "Delete failed");
    rmResult = Rm_delete(clientHandle, RM_TEST_TRUE);
    ERROR_CHECK(RM_OK, rmResult, clientName, "Delete failed");    
}

void main(Int argc, Char* argv[])
{
    Rm_InitCfg rmInitCfg;
    int32_t    rmResult;
    int        status;

    System_printf ("*********************************************************\n");
    System_printf ("******************** RM Memory Test *********************\n");
    System_printf ("*********************************************************\n");

    System_printf ("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    /* Reset the variable to indicate to other cores RM init is not yet done */
    isRmInitialized[0] = 0;

    coreNum = CSL_chipReadReg(CSL_CHIP_DNUM);
    testErrors = 0;

    System_printf("Core %d: Starting IPC...\n", coreNum);
    status = Ipc_start();
    if (status < 0) {
        testErrors++;
        System_abort("Ipc_start failed\n");
    }

    if (coreNum == SYSINIT) {
        /* Test individual instance init/deletes */
        testServer();
        testCd();
        testClient();
        
        /* Test Server-CD-Client interaction for memory leaks all from a single core */
        testServerCdClient();
        
        /* Create Server for multicore interaction memory tests */
        memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
        System_sprintf (rmInstName, "RM_Server");
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_SERVER;
        rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
        rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspPlusArmPolicy;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");
     
        /* Signal to remote cores that Server instance has passed all initialization
         * memory tests and is ready */
        isRmInitialized[0] = 1;
        Osal_rmEndMemAccess((void *)isRmInitialized,sizeof(isRmInitialized));
    }
    else {
        /* Create Client for multicore interaction memory tests */
        memset((void *)&rmInitCfg, 0, sizeof(rmInitCfg));
        System_sprintf (rmInstName, "RM_Client%d", coreNum);
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        ERROR_CHECK(RM_OK, rmResult, rmInstName, "Initialization failed");

        do{
            Osal_rmBeginMemAccess((void *)isRmInitialized, sizeof(isRmInitialized));
        } while (isRmInitialized[0] == 0);             
    }

    if (setupRmTransConfig(NUM_CORES, SYSINIT, rmStartupTsk) < 0)
    {
        System_printf ("Error core %d : Transport setup for RM error\n", coreNum);
        testErrors++;
        return;
    }

    System_printf("Core %d: Starting BIOS...\n", coreNum);
    BIOS_start();

    return;    
}

