/*  (C) Copyright 2012-2014, Texas Instruments, Inc.
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
#include <sys/types.h> 
#include <sys/wait.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include "fw_test.h"
#include "fw_mem_allocator.h"
#include "srio_test.h"
#include "srioPlatCfg.h"


uint32_t errorCount = 0;

void  *topLevelTest (void *args);
void  usageTsk (Cppi_Handle cppiHnd);

typedef pthread_t task_handle;

#define DEFAULT_STACK_SIZE  0x8000

/* look up value by name */
static uint32_t fw_name_lookup (const char *name)
{
    extern uint32_t ex_rm_name_lookup (const char *name);
    /* Bind to example's RM binding to avoid duplicating DSP code */
    return ex_rm_name_lookup (name);
}

/* set name to value */
static void fw_name_set (const char *name, uint32_t val)
{
    extern void ex_rm_name_set (const char *name, uint32_t val);
    /* Bind to example's RM binding to avoid duplicating DSP code */
    ex_rm_name_set (name, val);
}

/* delete name */
static void fw_name_del (const char *name)
{
    extern void ex_rm_name_del (const char *name);
    /* Bind to example's RM binding to avoid duplicating DSP code */
    ex_rm_name_del (name);
}

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
int fw_task_create ( void *(start_routine)(void*), void* args, void* handle)
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
void fw_task_wait (void *handle)
{
    pthread_join(*((pthread_t*)handle), NULL);
    return;
}

uint32_t l2_global_address(uint32_t x)
{
    return x;
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
int main(int argc, char *argv[]) {
    task_handle test_th;
    int status;
    int child = 0;
    int parent = 0;
    int i;
    int retVal = 0;
    pid_t pid;
    off_t pCMemBase;

    if (argc == 2)
    {
        coreNum = atoi (argv[1]);
        printf ("Command line specified I am core %d\n", coreNum);
    }
    else
    {
        printf ("No command line given.  Forking off %d tasks\n", srio_EXAMPLE_NUM_CORES);
        /* make srio_EXAMPLE_NUM_CORES tasks */
        for (coreNum = 1; coreNum < srio_EXAMPLE_NUM_CORES; coreNum++)
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

    printf ("coreNum: %d; pid=%d\n", coreNum, getpid());

    fw_osalInit();

    /* Setup RM */
    if (initRm())  {
        printf ("setupTestFramework: initRm returned error, exiting\n");
        return (-1);
    }

    /* core/task 0 will do cmem alloc and publish to RM; other tasks
     * will query RM */
    if (coreNum) 
    {
        uint32_t l, h;
        l = fw_name_lookup ("SRIOMT_CMA_PBASE_LOW");
        h = fw_name_lookup ("SRIOMT_CMA_PBASE_HIGH");
        pCMemBase = (off_t)l | (((off_t)h) << 32);
    }
    else
    {
        pCMemBase = 0;
    }

    if (fw_memAllocInit(&pCMemBase,
                        FW_QMSS_CMEM_SIZE) == fw_FALSE) {
        printf("ERROR: \"Top Level Test\" fw_memAllocInit failed\n");
        return (-1);
    }

    if (! coreNum)
    {
        fw_name_set ("SRIOMT_CMA_PBASE_LOW", (uint32_t)pCMemBase);
        fw_name_set ("SRIOMT_CMA_PBASE_HIGH", (uint32_t)(pCMemBase >> 32));
    }

    /* Create virtual memory maps */
    /* QMSS CFG Regs */
    fw_qmssCfgVaddr = fw_memMap((void*)QMSS_CFG_BASE_ADDR, QMSS_CFG_BLK_SZ);
    if (!fw_qmssCfgVaddr)
    {
        printf("ERROR: Failed to map QMSS CFG registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:QMSS_CFG_BASE_ADDR:0x%p Memory mapped at address %p.\n",(void*)QMSS_CFG_BASE_ADDR, fw_qmssCfgVaddr); 
#endif

    /* QMSS DATA Regs */
    fw_qmssDataVaddr = fw_memMap((void*)QMSS_DATA_BASE_ADDR, QMSS_DATA_BLK_SZ);
    if (!fw_qmssDataVaddr)
    {
        printf("ERROR: Failed to map QMSS DATA registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:QMSS_DATA_BASE_ADDR:0x%p Memory mapped at address %p.\n",(void*)QMSS_DATA_BASE_ADDR, fw_qmssDataVaddr);
#endif

    /* SRIO CFG Regs */
    fw_srioCfgVaddr = fw_memMap((void*)SRIO_CFG_BASE_ADDR, SRIO_CFG_BLK_SZ);
    if (!fw_srioCfgVaddr)
    {
        printf("ERROR: Failed to map SRIO registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:SRIO_CFG_BASE_ADDR:0x%p Memory mapped at address %p.\n",(void*)SRIO_CFG_BASE_ADDR, fw_srioCfgVaddr);
#endif

    /* SRIO SERDES CFG Regs */
    fw_srioSerdesCfgVaddr = fw_memMap((void*)SRIO_SERDES_CFG_BASE_ADDR, SRIO_SERDES_CFG_BLK_SZ);
    if (!fw_srioSerdesCfgVaddr)
    {
        printf("ERROR: Failed to map SRIO SERDES registers\n");
        return (-1);
    }
#ifdef EXT_DEBUG
    printf("main:SRIO_SERDES_CFG_BASE_ADDR:0x%p Memory mapped at address %p.\n",(void*)SRIO_SERDES_CFG_BASE_ADDR, fw_srioSerdesCfgVaddr);
#endif

    if ((status = fw_task_create(topLevelTest, NULL, &test_th))) {
        printf("ERROR: \"Top Level Test\" task-create failed (%d)\n", status);
        return (-1);
    }

    fw_task_wait(&test_th);
    fw_osalshutdown();

    /* wait for children to exit */
    if (parent)
    {
        printf("Waiting for children to exit\n");
        
        for (i = 0; i < (srio_EXAMPLE_NUM_CORES - 1); i++)
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

        fw_name_del ("SRIOMT_CMA_PBASE_LOW");
        fw_name_del ("SRIOMT_CMA_PBASE_HIGH");
        fw_memAllocExit ();
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
    Cppi_Handle cppiHnd;

    /* Initialize the QM and CPPI */
    if (setupTestFramework (&cppiHnd))  {
        printf ("topLevelTest (%s:%d): setupTestFramework returned error, exiting\n", __FILE__, __LINE__);
        return NULL;
    }

    /* Run the example */
    usageTsk(cppiHnd);

    pthread_exit((void*) 0);
    return NULL;
}

