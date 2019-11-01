/*
 *   rm_linux_mt_client_test.c
 *
 *   RM test that runs clients from multiple threads to test the
 *   multi-threaded serialization capabilities of RM clients.  The clients
 *   communicate with the RM server of sockets.
 *
 *  ============================================================================
 *
 * Copyright (c) 2012-2014, Texas Instruments Incorporated
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
#include <unistd.h>
#include <pthread.h>
#include <time.h>

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

/* Total resource request iterations between threads */
#define REQUEST_ITERATIONS           16000
#define RESOURCE_PRINT_DIVISOR       1000

/* Test FALSE */
#define RM_TEST_FALSE                0
/* Test TRUE */
#define RM_TEST_TRUE                 1

/* Socket timeout */
#define CLIENT_SOCK_TIMEOUT_USEC     (500)

/* Application's registered RM transport indices */
#define MAP_ENTRY_SERVER_TO_CLIENT   0
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES          1

/* Error checking macro */
#define ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)            \
    if (resultVal != checkVal) {                                          \
        char errorMsgToPrint[] = printMsg;                                \
        printf("Error : %s : ", rmInstName);                              \
        printf("%s with error code : %d\n", errorMsgToPrint, resultVal);  \
        test_errors++;                                                    \
    }

/**********************************************************************
 ********************** RM Test Data Structures ***********************
 **********************************************************************/

/* RM registered transport mapping structure */
typedef struct trans_map_entry_s {
    /* Registered RM transport handle */
    Rm_TransportHandle  transport_handle;
    /* Remote socket tied to the transport handle */
    sock_name_t        *remote_sock;
} trans_map_entry_t;

/**********************************************************************
 ********************** Extern Variables ******************************
 **********************************************************************/

/* Alloc and free OSAL variables */
extern uint32_t rmMallocCounter;
extern uint32_t rmFreeCounter;
extern int32_t rmByteAlloc;
extern int32_t rmByteFree;


/**********************************************************************
 ********************** Global Variables ******************************
 **********************************************************************/

/* Number of errors that occurred during the test */
uint16_t           test_errors;

/* RM Client Variables */
char               client_name[RM_NAME_MAX_CHARS] = "RM_Client";
Rm_Handle          client_handle = NULL;
Rm_ServiceHandle  *service_handle = NULL;

/* pthread mutex lock needed for testing */
pthread_mutex_t    client_mutex_lock;

/* Client socket name */
char               client_sock_name[] = "/var/run/rm/rm_client";

/* Client socket handles */
sock_h             client_sock;

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
trans_map_entry_t  rm_trans_map[MAX_MAPPING_ENTRIES];

/* RM resource names (must match resource node names in GRL and policies */
char               res_name_link_ram[RM_NAME_MAX_CHARS] = "link-ram";

/* Used to track allocations in each thread */
uint32_t           allocs[REQUEST_ITERATIONS];

/**********************************************************************
 *************************** Test Functions ***************************
 **********************************************************************/

uint32_t thread_num_one = 1;
uint32_t thread_num_two = 2;

typedef pthread_t task_handle;

#define DEFAULT_STACK_SIZE  0x8000
/** ============================================================================
 *   @n@b task_create
 *
 *   @b Description
 *   @n Create thread to run the test program
 *
 *   @param[in]  
 *   @n None
 * 
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
static int task_create (void *(start_routine)(void *), void* args, void* handle)
{
    int                max_priority, err;
    pthread_t          thread;
    pthread_attr_t     attr;
    struct sched_param param;

    max_priority = sched_get_priority_max(SCHED_FIFO);
    err = pthread_attr_init(&attr);
    if (err) {
        printf("pthread_attr_init failed: (%s)\n", strerror(err));
        return err;
    }
    err = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    if (err) {
        printf("pthread_attr_setdetachstate failed: (%s)\n", strerror(err));
        return err;
    }
    err = pthread_attr_setstacksize(&attr, DEFAULT_STACK_SIZE);
    if (err) {
        printf("pthread_attr_setstacksize failed: (%s)\n", strerror(err));
        return err;
    }
    err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (err) {
        printf("pthread_attr_setinheritsched failed: (%s)\n", strerror(err));
        return err;
    }
    err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (err) {
        printf("pthread_attr_setschedpolicy failed: (%s)\n", strerror(err));
        return err;
    }
    memset(&param, 0, sizeof(param));
    param.sched_priority = max_priority;
    err = pthread_attr_setschedparam(&attr, &param);
    if (err) {
        printf("pthread_attr_setschedparam failed: (%s)\n", strerror(err));
        return err;
    }
    if (err) return err;
    err = pthread_create(&thread, &attr, start_routine, args);
    if (err) {
        printf("pthread_create failed: (%s)\n", strerror(err));
        return err;
    }
    if (err) return err;
    *(pthread_t *)handle = thread;
    return 0;
}

/** ============================================================================
 *   @n@b task_wait
 *
 *   @b Description
 *   @n Wait for Task completion
 * 
 *   @return    void
 * =============================================================================
 */
static void task_wait (void *handle)
{
    pthread_join(*((pthread_t *)handle), NULL);
    return;
}


Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rm_pkt = NULL;

    rm_pkt = calloc(1, sizeof(*rm_pkt));
    if (!rm_pkt) {
        printf("Error : Can't malloc for RM send message (err: %s)\n", strerror(errno));
        test_errors++;
        return (NULL);
    }
    rm_pkt->pktLenBytes = pktSize;
    *pktHandle = rm_pkt;

    return(rm_pkt);
}

void transportFree (Rm_Packet *rm_pkt)
{
    if (rm_pkt) {
        free (rm_pkt);
    }         
}

void transportReceive (void)
{
    int32_t             rm_result;
    int                 retval;
    int                 length = 0;
    sock_name_t         server_sock_addr;
    Rm_Packet          *rm_pkt = NULL;
    struct sockaddr_un  server_addr;
    
    retval = sock_wait(client_sock, &length, NULL, -1);
    if (retval == -2) {
        /* Timeout */
        return;
    }
    else if (retval < 0) {
        printf("Error in reading from socket, error %d\n", retval);
        test_errors++;
        return;
    }
    
    if (length < ((int)sizeof(*rm_pkt))) {
        printf("invalid RM message length %d\n", length);
        test_errors++;
        return;
    }
    rm_pkt = calloc(1, length);
    if (!rm_pkt) {
        printf("can't malloc for recv'd RM message (err: %s)\n",
                  strerror(errno));
        test_errors++;
        return;
    }
    
    server_sock_addr.type = sock_addr_e;
    server_sock_addr.s.addr = &server_addr;
    retval = sock_recv(client_sock, (char *)rm_pkt, length, &server_sock_addr);
    if (retval != length) {
        printf("recv RM pkt failed from socket, received = %d, expected = %d\n",
                  retval, length);
        return;
    }

    /* Provide packet to RM Server for processing */       
    if ((rm_result = Rm_receivePacket(rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].transport_handle, rm_pkt))) {
        printf("RM failed to process received packet: %d\n", rm_result);
    }

    transportFree(rm_pkt);
}

int32_t transportSendRcv(Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    sock_name_t *server_sock_name = (sock_name_t *)appTransport;
    Rm_Packet   *rm_pkt = (Rm_Packet *)pktHandle;
    
    if (sock_send(client_sock, (char *)rm_pkt, (int) rm_pkt->pktLenBytes, server_sock_name)) {
        printf("send data failed\n");
    }

    /* Wait for response from Server */
    transportReceive();
 
    return (0);
}

/** ============================================================================
 *   @n@b odd_test
 *
 *   @b Description
 *   @n Resource request routine running as a separate thread
 * 
 *   @return    void *
 * =============================================================================
 */
void *allocate_test (void *args)
{
    uint32_t           *thread_num = (uint32_t *)args;
    Rm_ServiceReqInfo   request;
    Rm_ServiceRespInfo  response;
    uint32_t            i;
    struct timespec     tim;
    
    tim.tv_sec = 0;
    tim.tv_nsec = 1000;

    if (service_handle) {
        /* Init request - request one resource at a time as unspecified.  Server
         * will return next available resource */
        memset(&request, 0, sizeof(request));
        request.type = Rm_service_RESOURCE_ALLOCATE_INIT;
        request.resourceName = res_name_link_ram;
        request.resourceBase = RM_RESOURCE_BASE_UNSPECIFIED;
        request.resourceLength = 1;

        printf("Thread %d : Requesting %d resources...\n", *thread_num, REQUEST_ITERATIONS / 2);
        for (i = 0; i < (REQUEST_ITERATIONS / 2); i++) {
            memset(&response, 0, sizeof(response));
            service_handle->Rm_serviceHandler(service_handle->rmHandle, &request, &response);

            if (response.serviceState == RM_SERVICE_APPROVED) {
                allocs[response.resourceBase]++;
                nanosleep(&tim , NULL);

                if ((i % RESOURCE_PRINT_DIVISOR) == 0) {
                    printf("Thread %d : Requested %d resources...\n", *thread_num, RESOURCE_PRINT_DIVISOR);
                }
            }
            else {
                printf("Error Thread %d : Allocate request was not approved for resource %d\n",
                       *thread_num, request.resourceBase);
                test_errors++;
                goto error_exit;
            }
        }
    }

error_exit:
    printf("Thread %d : Exiting...\n", *thread_num);
    pthread_exit((void*) 0);
    return NULL;
}

void resource_cleanup(void)
{
    Rm_ServiceReqInfo  request;
    Rm_ServiceRespInfo response;
    uint32_t           i;

    if (service_handle) {
        printf("Freeing all %d resources...\n", REQUEST_ITERATIONS);
        for (i = 0; i < REQUEST_ITERATIONS; i++) {
            while (allocs[i]) {
                memset(&request, 0, sizeof(request));
                request.type = Rm_service_RESOURCE_FREE;
                request.resourceName = res_name_link_ram;
                request.resourceBase = i;
                request.resourceLength = 1;

                memset(&response, 0, sizeof(response));
                service_handle->Rm_serviceHandler(service_handle->rmHandle, &request, &response);

                if (response.serviceState != RM_SERVICE_APPROVED) {
                    printf("Error : Free request was not approved for resource %d - Exiting...\n",
                           request.resourceBase);
                    test_errors++;
                }

                allocs[i]--;
            }
        }
    }
}

void connection_setup(void)
{
    Rm_TransportCfg rm_trans_cfg;
    int32_t         rm_result;
    int             i;
    sock_name_t     sock_name;
    char            server_sock_name[] = RM_SERVER_SOCKET_NAME;

    printf("Setting up socket connection to RM Server\n");
    
    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rm_trans_map[i].transport_handle = NULL;
    }

    sock_name.type = sock_name_e;
    sock_name.s.name = client_sock_name;

    client_sock = sock_open(&sock_name, 0, 0);
    if (!client_sock) {
        printf("Client socket open failed\n");
        exit(EXIT_FAILURE);
    }

    rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].remote_sock = calloc(1, sizeof(sock_name_t));
    rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].remote_sock->type = sock_name_e;    
    rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].remote_sock->s.name = calloc(1, strlen(server_sock_name)+1);
    strncpy(rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].remote_sock->s.name, server_sock_name, strlen(server_sock_name)+1);

    /* Register the Server with the Client instance */
    rm_trans_cfg.rmHandle = client_handle;
    rm_trans_cfg.appTransportHandle = (Rm_AppTransportHandle) rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].remote_sock;
    rm_trans_cfg.remoteInstType = Rm_instType_SERVER;
    rm_trans_cfg.transportCallouts.rmAllocPkt = transportAlloc;
    rm_trans_cfg.transportCallouts.rmSendPkt = transportSendRcv;
    rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].transport_handle = Rm_transportRegister(&rm_trans_cfg, &rm_result);
}

void connection_close(void)
{
    int32_t result;

    printf("Closing connection to RM Server socket\n");
    result = Rm_transportUnregister(rm_trans_map[MAP_ENTRY_SERVER_TO_CLIENT].transport_handle);
    ERROR_CHECK(RM_OK, result, client_name, "Unregister of Server transport failed");

	sock_close(client_sock);
}

int32_t client_init(void)
{
    Rm_InitCfg rm_init_cfg;
    int        pthread_result;
    int32_t    result;

    /* Initialize the pthread mutex */
    if ((pthread_result = pthread_mutex_init(&client_mutex_lock, NULL))) {
        printf("Error : Failed to init client pthread mutex with error code: %d\n", pthread_result);
        test_errors++;
        goto error_exit;
    }

    printf("Initializing RM Client\n");
    memset(&rm_init_cfg, 0, sizeof(rm_init_cfg));
    rm_init_cfg.instName = client_name;
    rm_init_cfg.instType = Rm_instType_CLIENT;
    rm_init_cfg.mtSemObj = (uint32_t *) &client_mutex_lock;
    client_handle = Rm_init(&rm_init_cfg, &result);
    ERROR_CHECK(RM_OK, result, client_name, "Initialization failed");
    if (result != RM_OK) {
        goto error_exit;
    }  

    service_handle = Rm_serviceOpenHandle(client_handle, &result);
    ERROR_CHECK(RM_OK, result, client_name, "Service handle open failed"); 
    if (result != RM_OK) {
        goto error_exit;
    }

    return (RM_TEST_TRUE);
error_exit:
    return (RM_TEST_FALSE);
    
}

void client_cleanup(void)
{
    int32_t result;
    int     pthread_result;

    printf("Deleting RM Client\n");

    if (service_handle) {
        result = Rm_serviceCloseHandle(service_handle);
        ERROR_CHECK(RM_OK, result, client_name, "Service handle close failed");
    }
    
    if (client_handle) {
        result = Rm_delete(client_handle, RM_TEST_TRUE);
        ERROR_CHECK(RM_OK, result, client_name, "Instance delete failed");
    }
    
    if ((pthread_result = pthread_mutex_destroy(&client_mutex_lock))) {
        printf("Error : Failed to destroy client pthread mutex with error code: %d\n", pthread_result);
        test_errors++;
    }
}

int main(int argc, char *argv[])
{
    int32_t     malloc_free_diff;
    int32_t     byte_free_diff;
    task_handle first_th;
    task_handle second_th;
    int         status;
    uint32_t    i;

    printf("*********************************************************\n");
    printf("******** RM Linux Multi-Threaded Client Testing *********\n");
    printf("*********************************************************\n");

    printf("RM Version : 0x%08x\nVersion String: %s\n", Rm_getVersion(), Rm_getVersionStr());

    test_errors = 0;
    memset(&allocs[0], 0, sizeof(allocs));

    if (client_init()) {
        connection_setup();

        printf("Creating first allocate_test thread\n");
        if ((status = task_create(allocate_test, (void *)&thread_num_one, &first_th))) {
            printf("ERROR : 1st \"Allocate Test\" task-create failed (%d)\n", status);
            test_errors++;
            goto cleanup_test;
        }
        printf("Creating second allocate_test thread\n");        
        if ((status = task_create(allocate_test, (void *)&thread_num_two, &second_th))) {
            printf("ERROR : 2nd \"Allocate Test\" task-create failed (%d)\n", status);
            test_errors++;
            goto cleanup_test;
        }

        task_wait(&first_th);        
        task_wait(&second_th);

        if (!test_errors) {
            printf("Threads complete - Checking for allocation errors\n");
            for (i = 0; i < REQUEST_ITERATIONS; i++) {
                if (allocs[i] != 1) {
                    printf ("FAILED : Resource %d not allocated exactly once\n", i);
                    test_errors++;
                }
            }

            if (!test_errors) {
                printf("PASSED : All Resources allocated once\n");
            }
        }

        resource_cleanup();
cleanup_test:
        connection_close();
    }
    client_cleanup();
    
    printf ("---------------------------------------------------------\n");
    printf ("------------------ Memory Leak Check --------------------\n");
    printf ("-                     :  malloc count  |   free count   -\n");
    printf ("- Example Completion  :         %6d |         %6d -\n", rmMallocCounter, rmFreeCounter);
    printf ("-            (bytes)  :       %8d |       %8d -\n", rmByteAlloc, rmByteFree);
    malloc_free_diff = rmMallocCounter - rmFreeCounter;
    byte_free_diff = rmByteAlloc - rmByteFree;
    if (malloc_free_diff > 0) {
        printf ("- FAILED : %6d unfreed mallocs                       -\n", malloc_free_diff);
        test_errors++;
    }
    else if (byte_free_diff > 0) {
        printf ("- FAILED : %6d unfreed bytes                         -\n", byte_free_diff);
        test_errors++;
    }    
    else if (malloc_free_diff < 0) {
        printf ("- FAILED : %6d more frees than mallocs               -\n", -malloc_free_diff);
        test_errors++;
    }
    else if (byte_free_diff < 0) {
        printf ("- FAILED : %6d more bytes freed than malloc'd        -\n", -byte_free_diff);
        test_errors++;
    }    
    else {
        printf ("- PASSED                                                -\n");
    }
    printf ("---------------------------------------------------------\n");
    printf ("\n");  

    printf ("---------------------------------------------------------\n");
    printf ("------------------ Example Completion -------------------\n");
    if (test_errors) {
        printf ("- FAILED : Test Errors: %-23d         -\n", test_errors);
    }
    printf ("---------------------------------------------------------\n");
    printf ("\n"); 

    return (0);    
}
