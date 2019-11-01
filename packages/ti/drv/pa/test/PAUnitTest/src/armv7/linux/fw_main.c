/*  (C) Copyright 2012, Texas Instruments, Inc.
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
#include <time.h>
#include <pthread.h>
#include <string.h>
#include "fw_mem_allocator.h"
#include "../../pautest.h"


/* NULL terminated The list of tests */
paTest_t  paTestList[] = {
	{ paTestUnconfigured, 	  "Packet reception while unconfigured",    PA_TEST_NOT_RUN },
	{ paTestSrioRouting,      "Pa_addSrio and SRIO routing",            PA_TEST_NOT_RUN },
	{ paTestL2Routing,    	  "Pa_addMac and L2 routing",               PA_TEST_NOT_RUN },
	{ paTestL3Routing, 		    "Pa_addIp and L3 Routing",			          PA_TEST_NOT_RUN },
	{ paTestL4Routing, 		    "Pa_addPort and L4 Routing", 		          PA_TEST_NOT_RUN },
	{ paTestPatchRoute,       "Blind patch and route",                  PA_TEST_NOT_RUN },
	{ paTestTxFmtRt,      	  "Tx checksum and routing",                PA_TEST_NOT_RUN },
	{ paTestCustom,			      "Custom routing",					                PA_TEST_NOT_RUN },
	{ paTestIPv4FragReassem,  "IPv4 Fragmentation and Reassembly",      PA_TEST_NOT_RUN },
	{ paTestIPv6FragReassem,  "IPv6 Fragmentation and Reassembly",      PA_TEST_NOT_RUN },
#ifdef NSS_GEN2
	{ paTestACL, 		          "Pa_addAcl and ACL filtering",		        PA_TEST_NOT_RUN },
    { paTestACLRescore,       "Pa_addAcl and Rescore",                  PA_TEST_NOT_RUN },
	{ paTestEflow,  	        "Egress Flow and Packet Forwarding Test", PA_TEST_NOT_RUN },
#endif
 	{ paTestUnconfigured, 	  "Packet reception while unconfigured",    PA_TEST_NOT_RUN },
/*
 *  This test should not be done at the life network since unexpected boroadcast and/or multicast packets
 *  may be received.
	{ paTestMultiRouting,   "Multi-routing",					   PA_TEST_NOT_RUN },
 */
	{ NULL,                 NULL,                                  PA_TEST_NOT_RUN }
};

#define PAU_NUM_TESTS       ((sizeof(paTestList)/sizeof(paTest_t)) - 1)

void  *topLevelTest (void *args);

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
/** ============================================================================
 *   @n@b main
 *
 *   @b Description
 *   @n test application main
 *
 *   @return    int
 * =============================================================================
 */
int main() {
    task_handle test_th;
    int status;

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


    if (status = task_create(topLevelTest, NULL, &test_th)) {
        printf("ERROR: \"Top Level Test\" task-create failed (%d)\n", status);
        return (-1);
    }

    task_wait(&test_th);
    fw_osalshutdown();
    return 0;
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
    task_handle fw_test_th;
    int status;
    paTestArgs_t testArgs;
	int passCount;
	int failCount;
	int notRunCount;
    int i;

    testArgs.tf = &tFramework;

    printf ("\n\n ------- PA Unit Test Starting ---------\n");

	/* Clear the logs */
	clearPaInfo();

    /* Initialize the PA, PA cpdma, QM and CPPI. Each test will use
     * the same framework */
    if (setupTestFramework ())  {
    	printf ("topLevelTest (%s:%d): setupTestFramework returned error, exiting\n", __FILE__, __LINE__);
        return;
    }

    /* Make sure the setup matches what is expected */
    if (verifyTestFramework())  {
    	printf ("topLevelTest (%s:%d): verifyTestFramework returned error after initial framework setup, exiting\n", __FILE__, __LINE__);
        return;
    }

	/* Run the tests */
	for (i = 0; paTestList[i].testFunction != NULL; i++ )  {

        testArgs.pat = &paTestList[i];

        if (status = task_create(paTestList[i].testFunction, (void *)&testArgs, &fw_test_th)) {
            printf("ERROR: \"fwTest\" task-create failed (%d)\n", status);
            return;
        }

		/* The test task will terminate upon completion. */
        task_wait(&fw_test_th);

		if (paTestList[i].testStatus == PA_TEST_PASSED)
		  printf ("%s:  PASSED\n", paTestList[i].name);
		else
		  printf ("%s:  FAILED\n", paTestList[i].name);

		/* Do a quick check of the test framework */
		if (verifyTestFramework ())  {
			printf ("topLevelTest (%s:%d): verifyTestFramework returned error after test %s. Exiting.\n", __FILE__, __LINE__, paTestList[i].name);
            return;
        }
    }

	/* Summarize the test results */
	for (i = passCount = failCount = notRunCount = 0; paTestList[i].testFunction != NULL; i++)  {
		if (paTestList[i].testStatus == PA_TEST_PASSED)
			passCount += 1;
		else if (paTestList[i].testStatus == PA_TEST_FAILED)
			failCount += 1;
		else
			notRunCount += 1;
	}

	printf ("\n\nTest summary:\n\tTests Passed: %d\n\tTests Failed: %d\n\tTests not run: %d\n\n",
	  passCount, failCount, notRunCount);

    if(passCount == PAU_NUM_TESTS)
	    System_printf ("All tests have passed!");

	if (clearTestFramework() )
		System_printf ("\n\n ------- PA Unit Test Clean Failed ---------\n");

	printf ("\n\n ------- PA Unit Test Complete ---------\n");

    pthread_exit((void*) 0);
    return;
}


