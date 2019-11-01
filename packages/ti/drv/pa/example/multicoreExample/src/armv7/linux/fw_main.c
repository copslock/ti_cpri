/******************************************************************************
 * FILE PURPOSE:  Main function routine for Example
 ******************************************************************************
 * FILE NAME:   fw_main.c
 *
 * @brief
 *  Example to illustrate the usage of PA/QMSS/CPPI from User Land Linux environment
 *  on ARM
 *
 *  This example application does the following:
 *      (1) Initializes:
 *              (a) Queue Manager (QM) Subsystem
 *              (b) Packet Accelerator (PA) CPPI DMA
 *
 *      (2) Sets up the CPPI descriptors and Queues required for sending and
 *          receiving data using Ethernet.
 *              (a) Uses Host descriptors
 *              (b) Uses QMSS poll mode
 *
 *      (3) Sets up the example application's configuration (MAC address
 *          it uses to send/recv data; IP address and port number it's listening
 *          on) in PA Subsystem so as to enable the PASS to forward all packets
 *          matching this configuration onto the application for processing.
 *              (a) Switch MAC address configured   =   0x10:0x11:0x12:0x13:0x14:0x15
 *              (b) Example's IP address            =   192.168.1.10
 *              (c) Example App's listening port    =   0x5678
 *
 *      (4) Sends packets onto wire
 *          (constructed manually in code here with following settings):
 *              (a) Source MAC      =   0x00:0x01:0x02:0x03:0x04:0x05
 *                  Destination MAC =   0x10:0x11:0x12:0x13:0x14:0x15
 *              (b) Source IP       =   192.168.1.1
 *                  Destination IP  =   192.168.1.10
 *              (c) Source Port     =   0x1234
 *                  Destination Port=   0x5678
 *              (d) Payload Data (80 bytes)
 *
 *          The packets sent by the application are looped back at PA by default.The packets
 *          are received and passed back up to the example application for processing.
 *
 *      (5) Application receives all packets using QM poll mode
 *          Validates received packet against data sent.
 *
 *  Example Test Setup:
 *
 *          EVM for an SOC example running Linux environment on ARM processor.
 *          If packets are routed to network. You could put another PC on the Hub to observe packets
 *          being sent onto wire.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2013, Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>     /* Symbolic Constants */
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include "fw_mem_allocator.h"
#include "multicore_example.h"

void  *topLevelTest (void *args);


/* The exit code is a global. This is used so
 * the clock function can terminate the program with
 * the proper exit code
 */
int exitCode;
#define MAX_RETRIES 5
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
static int task_create ( void *(start_routine)(void*), void* args, void* handle)
{
    int max_priority, err;
    pthread_t thread;
    pthread_attr_t attr;
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
    *(pthread_t*)handle = thread;
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
    pthread_join(*((pthread_t*)handle), NULL);
    return;
}
/** ============================================================================
 *   @n@b task_sleep
 *
 *   @b Description
 *   @n Sleep the thread for msec duration
 *
 *   @return    void
 * =============================================================================
 */

static void task_sleep(int time_in_msec)
{
    pthread_mutex_t fake_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t fake_cond = PTHREAD_COND_INITIALIZER;
    struct timespec ts;
    unsigned int sec, nsec;

    sec = time_in_msec/1000;
    nsec = (time_in_msec - (sec*1000)) * 1000000;

    /* Use the wall-clock time */
    clock_gettime(CLOCK_REALTIME, &ts);

    ts.tv_sec += sec;
    ts.tv_nsec += nsec;

    pthread_mutex_lock(&fake_mutex);
    pthread_cond_timedwait(&fake_cond, &fake_mutex, &ts);
    pthread_mutex_unlock(&fake_mutex);
}
/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n test application main
 *
 *   @return    int
 * =============================================================================
 */
uint32_t  coreNum;
int main(int argc, char *argv[]) {
    task_handle test_th;
    int status;
    int child = 0;
    int parent = 0;
    int i;
    int retVal = 0;
    pid_t pid;

	/* Create the shared memory for multiprocess */
    fw_shmCreate();

    /* Initialize the semaphore for multiprocess variables */
    fw_SemInit ();
    if (argc == 2)
    {
        coreNum = atoi (argv[1]);
        printf ("Command line specified I am core %d\n", coreNum);
    }
    else
    {
        printf ("No command line given.  Forking off %d tasks\n", pa_MC_EXAMPLE_NUM_CORES);
        /* make qmss_EXAMPLE_NUM_CORES tasks */
        for (coreNum = 1; coreNum < pa_MC_EXAMPLE_NUM_CORES; coreNum++)
        {
            pid = fork();
            if (! pid)
            {
                child = 1;
                break;
            }
        }
        if (! child)
        {
            coreNum = 0;
            parent = 1;
        }
    }

    fw_osalInit();

    if (fw_memAllocInit((uint8_t*)MSMC_SRAM_BASE_ADDR,
                                  MSMC_TEST_PERM_MEM_SZ) == fw_FALSE) {
        printf("ERROR: \"Top Level Test\" fw_memAllocInit failed\n");
        return (-1);
    }


    /* Create virtual memory maps */
    /* QMSS CFG Regs */
    fw_qmssCfgVaddr = fw_memMap((void*)QMSS_CFG_BASE_ADDR,
                                            QMSS_CFG_BLK_SZ);
    if (!fw_qmssCfgVaddr)
    {
        printf("ERROR: Failed to map QMSS CFG registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:QMSS_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)QMSS_CFG_BASE_ADDR, fw_qmssCfgVaddr);
#endif

    /* QMSS DATA Regs */
    fw_qmssDataVaddr = fw_memMap((void*)QMSS_DATA_BASE_ADDR,
                                            QMSS_DATA_BLK_SZ);
    if (!fw_qmssDataVaddr)
    {
        printf("ERROR: Failed to map QMSS DATA registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:QMSS_DATA_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)QMSS_DATA_BASE_ADDR, fw_qmssDataVaddr);
#endif

#if defined(SOC_K2K) || defined(SOC_K2H)


    /* SRIO CFG Regs */
    fw_srioCfgVaddr = fw_memMap((void*)SRIO_CFG_BASE_ADDR,
                                            SRIO_CFG_BLK_SZ);
    if (!fw_srioCfgVaddr)
    {
        printf("ERROR: Failed to map SRIO CFG registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:SRIO_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)SRIO_CFG_BASE_ADDR, fw_srioCfgVaddr);
#endif

#endif

    /* PASS CFG Regs */
    fw_passCfgVaddr = fw_memMap((void*)PASS_CFG_BASE_ADDR,
                                            PASS_CFG_BLK_SZ);
    if (!fw_passCfgVaddr)
    {
        printf("ERROR: Failed to map PASS CFG registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:PASS_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)PASS_CFG_BASE_ADDR, fw_passCfgVaddr);
#endif


    if ((status = task_create(topLevelTest, NULL, &test_th))) {
        printf("ERROR: \"Top Level Test\" task-create failed (%d)\n", status);
        return (-1);
    }

    task_wait(&test_th);
    fw_osalshutdown();

    /* wait for children to exit */
    if (parent)
    {
        printf("Waiting for children to exit\n");

        for (i = 0; i < (pa_MC_EXAMPLE_NUM_CORES - 1); i++)
        {
            pid = wait (&status);
            if (WIFEXITED(status))
            {
                if (WEXITSTATUS(status))
                {
                    printf ("Child %d returned fail (%d)\n", pid, WEXITSTATUS(status));
                    retVal = 1;
                }

            }
            else
            {
                printf ("Child %d failed to exit\n", pid);
                retVal = 1;
            }
        }
        if ((! retVal) && (! errorCount))
        {
            printf ("All children passed\n");
        }
    }
    if (errorCount)
    {
        retVal = 1;
    }

    if (retVal)
    {
        printf("****FAIL****\n");
    }

    return retVal;
}

/** ============================================================================
 *   @n@b topLevelTest
 *
 *   @b Description
 *   @n Routine running as a separate thread
 *
 *   @return    int
 * =============================================================================
 */
void *topLevelTest (void *args)
{
    MultiCoreApp(args);

    pthread_exit((void*) 0);
    return ((void *) 0);
}


