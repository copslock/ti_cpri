/******************************************************************************
 * FILE PURPOSE:  Main function routine for NWAL unit test
 ******************************************************************************
 * FILE NAME:   fw_main.c
 *
 * DESCRIPTION: Header file for unit test package
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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

#define _GNU_SOURCE
#include <pthread.h>

#include "fw_test.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <pthread.h>
#include <string.h>


#define SINGLE_CORE_TEST

extern void  *nwalTest(void *args);
void topLevelTest (void);


/* The exit code is a global. This is used so
 * the clock function can terminate the program with
 * the proper exit code
 */
int exitCode;


typedef pthread_t task_handle;

#define DEFAULT_STACK_SIZE	0x8000
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

static void task_wait (void *handle)
{
    pthread_join(*((pthread_t*)handle), NULL);
    return;
}


static void task_sleep(int time_in_msec)
{
    pthread_mutex_t fake_mutex = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t fake_cond = PTHREAD_COND_INITIALIZER;
	struct timespec ts;
	int rt;
    unsigned int sec, nsec;

    sec = time_in_msec/1000;
    nsec = (time_in_msec - (sec*1000)) * 1000000;

    /* Use the wall-clock time */
    clock_gettime(CLOCK_REALTIME, &ts);

	ts.tv_sec += sec;
	ts.tv_nsec += nsec;

	pthread_mutex_lock(&fake_mutex);
	rt = pthread_cond_timedwait(&fake_cond, &fake_mutex, &ts);
	pthread_mutex_unlock(&fake_mutex);
}



/* Total Permanent memory required in NWAL test
 * for Packet buffers & descriptor buffers
 */
#define NWAL_TEST_PERM_MEM_SZ   (4*1024*1024)

/* Global variables to hold virtual address of various subsystems */
void *fw_qmssCfgVaddr;
void *fw_qmssDataVaddr;
void *fw_srioCfgVaddr;
void *fw_netcpCfgVaddr;

/* Global variablesto hold virtual address of various subsystems */
hplib_virtualAddrInfo_T nwal_VM_VirtAddr;
hplib_memPoolAttr_T nwal_VM_MempoolAttr[2];
/* test application main */
int main() {
    task_handle test_th;
    int status;
    int numMemPools = 2;
    int core_id;
    hplib_RetValue result;
    cpu_set_t cpu_set;
    void*pBase;


#ifdef NWAL_ENABLE_USER_MODE_CACHE
   /* Init attributes for DDR */
    nwal_VM_MempoolAttr[0].attr = HPLIB_ATTR_KM_CACHED0;
    nwal_VM_MempoolAttr[0].phys_addr = 0;
    nwal_VM_MempoolAttr[0].size = 0;

    /* Init attributes for un-cached MSMC */
    nwal_VM_MempoolAttr[1].attr = HPLIB_ATTR_UN_CACHED;
    nwal_VM_MempoolAttr[1].phys_addr = CSL_MSMC_SRAM_REGS;
    nwal_VM_MempoolAttr[1].size = NWAL_TEST_PERM_MEM_SZ;
#else
 /* Init attributes for un-cached MSMC */
    nwal_VM_MempoolAttr[0].attr = HPLIB_ATTR_UN_CACHED;
    nwal_VM_MempoolAttr[0].phys_addr = CSL_MSMC_SRAM_REGS;
    nwal_VM_MempoolAttr[0].size = NWAL_TEST_PERM_MEM_SZ;
    
   /* Init attributes for DDR */
    nwal_VM_MempoolAttr[1].attr = HPLIB_ATTR_KM_CACHED0;
    nwal_VM_MempoolAttr[1].phys_addr = 0;
    nwal_VM_MempoolAttr[1].size = 0;
    numMemPools = 1;
#endif
    /* assign main to run on core 0 */
    CPU_ZERO( &cpu_set);
    CPU_SET( 0, &cpu_set);
    hplib_utilSetupThread(0, &cpu_set,hplib_spinLock_Type_LOL);
    /* initialize all the memory we are going to use
       - chunk for buffers, descriptors
       - memory mapped peripherals we use, such as QMSS, PA, etc */
    result = hplib_vmInit(&nwal_VM_VirtAddr,
                          numMemPools, 
                          &nwal_VM_MempoolAttr[0]);

    if (result != hplib_OK)
    {
         printf("ERROR: main: hplib_vmInit returned error code %d\n", result);
          return (-1);
    }
    pBase = hplib_shmCreate(HPLIB_SHM_SIZE);

    if (pBase == NULL)
    {
        printf("main: hplib_shmCreate failure\n");
        return NULL;
    }
    else
        printf("main: hplib_shmCreate sucess\n");

    hplib_initMallocArea(0);
    hplib_initMallocArea(1);

    hplib_utilOsalCreate();
    /* Virtual memory maps  created as part of hplib_vmInit, now update local variables*/
    /* QMSS CFG Regs */
    fw_qmssCfgVaddr = nwal_VM_VirtAddr.qmssCfgVaddr;
    printf("main:QMSS_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)fw_qmssCfgVaddr, fw_qmssCfgVaddr);

    fw_qmssDataVaddr = nwal_VM_VirtAddr.qmssDataVaddr;
    printf("main:QMSS_DATA_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)fw_qmssDataVaddr, fw_qmssDataVaddr);

#if 0
    /* SRIO CFG Regs */
    fw_srioCfgVaddr = nwal_VM_VirtAddr.srioCfgVaddr;
    printf("main:SRIO_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)fw_srioCfgVaddr, fw_srioCfgVaddr);
#endif
    /* PASS CFG Regs */
    fw_netcpCfgVaddr = nwal_VM_VirtAddr.passCfgVaddr;
    printf("main:NETCP_CFG_BASE_ADDR:0x%x Memory mapped at address %p.\n",(void*)fw_netcpCfgVaddr, fw_netcpCfgVaddr);

    topLevelTest();

    printf("done with main\n");
    return 0;
}

static volatile int globalInitDone = 0;
static volatile int localInitDone = 0;

void  nwalLocalTest(int core_id)
{
    cpu_set_t cpu_set;
    System_printf("CORE: %d nwalLocalTest called\n", core_id);
    CPU_ZERO(&cpu_set);
    CPU_SET(core_id, &cpu_set);
    hplib_utilSetupThread(core_id, &cpu_set, hplib_spinLock_Type_LOL);
    if (core_id == 0)
    {
        if (testNwInit())
        {
            printf("ERROR: \"nwalTest\" testNwInit() failed \n");
            return;
        }
        System_printf("CORE: %dnwalLocalTest: returned from testNwInit\n", core_id);
        globalInitDone = 1;

#ifndef SINGLE_CORE_TEST
        while(!localInitDone)
        {
            task_sleep(1);
        }
#endif
         nwalTest(core_id);
    }
#ifndef SINGLE_CORE_TEST
    else
    {
        while(!globalInitDone)
        {
            sched_yield();
        }
        /* Start NWAL for each remote core */
        testNwalStartCore();
        localInitDone = 1;
        nwalTest(core_id);
    }
#endif
}

void nwal_coreSetup(int core_id, task_handle* nwal_test_th)
{
    int status;

   System_printf("CORE: %d nwal_coreSetup called\n", core_id);

    if (status = task_create((void*)nwalLocalTest, (void *)core_id, (void*)nwal_test_th))
    {
        printf("ERROR: \"nwal_coreSetup\" task-create failed (%d)\n", status);
        return;
    }
}

void topLevelTest(void)
{
    int core_id;
    task_handle nwal_test_th[CPU_NCORES];
#ifdef SINGLE_CORE_TEST
    for (core_id=0; core_id < 1;core_id++)
#else
    for (core_id=0; core_id < CPU_NCORES-2;core_id++)
#endif
    {
        nwal_coreSetup(core_id,&nwal_test_th[core_id]);
    }

#ifdef SINGLE_CORE_TEST
        for (core_id=0; core_id < 1;core_id++)
#else
        for (core_id=0; core_id < CPU_NCORES-2;core_id++)
#endif

    {
        task_wait(&nwal_test_th[core_id]);
        System_printf("topLeveTest: core id %d done\n",core_id);
    }
    return;
}


