/**
 *   @file  test_descAlloc.c
 *
 *   @brief
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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
 *  \par
*/

#ifndef __LINUX_USER_SPACE
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#else
#include "fw_test.h"
#include "fw_mem_allocator.h"
#endif

#include <string.h>

#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif
/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>
#endif

#ifdef __LINUX_USER_SPACE
void initRm();
#endif
/************************ USER DEFINES ********************/
#define NUM_MONOLITHIC_DESC         64
#define SIZE_MONOLITHIC_DESC        64

#define NUM_MISC_DESC               32
#define SIZE_MISC_DESC              16

/* Pick different prime numbers so can check if any sizes get mixed up from sum of 8 packets */
#define THREAD_A_DATA_SIZE          1009
#define THREAD_B_DATA_SIZE          1013
#define ATOMIC_PREFILL_ENTRIES      8
#define ATOMIC_PUSH_COUNT           10000000

/************************ GLOBAL VARIABLES ********************/

#ifndef __LINUX_USER_SPACE
#ifdef _TMS320C6X
#pragma DATA_ALIGN (linkingRAM0, 16)
uint64_t linkingRAM0[NUM_MONOLITHIC_DESC + NUM_MISC_DESC];
#else
uint64_t linkingRAM0[NUM_MONOLITHIC_DESC + NUM_MISC_DESC] __attribute__ ((aligned (16)));
#endif

/* Descriptor pool [Size of descriptor * Number of descriptors] */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (monolithicDescBuf, 16)
uint8_t monolithicDescBuf[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#pragma DATA_ALIGN (miscDescBuf, 16)
uint8_t miscDescBuf[SIZE_MISC_DESC * NUM_MISC_DESC];
#else
uint8_t monolithicDescBuf[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
uint8_t miscDescBuf[SIZE_MISC_DESC * NUM_MISC_DESC] __attribute__ ((aligned (16)));
#endif
#endif

uint8_t *monolithicDesc;
uint8_t *miscDesc;

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* Memory region configuration status */
Qmss_MemRegCfg          memRegStatus;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;
/* Store the queue handle for destination queues on which allocated descriptors are stored */
Qmss_QueueHnd           QueHnd[QMSS_MAX_MEM_REGIONS];

/************************ EXTERN VARIABLES ********************/
/* Error counter */
extern uint32_t                 errorCount;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;

/**
 *  @b Description
 *  @n
 *      Utility function which converts a local GEM L2 memory address
 *      to global memory address.
 *
 *  @param[in]  addr
 *      Local address to be converted
 *
 *  @retval
 *      Computed L2 global Address
 */
#ifndef __LINUX_USER_SPACE
static uint32_t l2_global_address (uint32_t addr)
{
#ifdef _TMS320C6X
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}
#else
static uint32_t l2_global_address (uint32_t addr)
{
    return addr;
}
#endif

#if defined(__ARM_ARCH_7A__) && defined(__LINUX_USER_SPACE)
typedef struct {
   Qmss_QueueHnd myQ;
   Qmss_QueueHnd otherQ;
   uint32_t expSize;
   char *name;
} taskParams;

taskParams taskAParams, taskBParams;
pthread_t taskBHnd;

void *atomicPushTask (void *params)
{
    taskParams *p = (taskParams *)params;
    void *syncDesc, *desc;
    uint32_t size, expSize1, expSize2;
    int i,j;

    System_printf("Thread %s: about to push %d descs on q %d\n", p->name, ATOMIC_PUSH_COUNT, p->myQ);

    /* Check initial size */
    expSize1 = ATOMIC_PREFILL_ENTRIES * p->expSize; /* no descriptors in flight */
    size = Qmss_getQueueByteCount (p->myQ);
    if (size != expSize1)
    {
        System_printf ("Error %s: got initial wrong size %d, expect %d\n",
                       p->name, size, expSize1);
        errorCount++;
        return NULL;
    }

    /* draw one descriptor to unlock/sync other thread */
    syncDesc = Qmss_queuePop (p->myQ);
    expSize1 = (ATOMIC_PREFILL_ENTRIES - 1) * p->expSize; /* no descriptors in flight */
    expSize2 = expSize1 - p->expSize; /* one descriptor in flight */

    /* Wait for other thread to draw his descriptor */
    while (Qmss_getQueueEntryCount (p->otherQ) == ATOMIC_PREFILL_ENTRIES);

    for (i = 0; i < ATOMIC_PUSH_COUNT; )
    {
        for (j = 0; j < (ATOMIC_PREFILL_ENTRIES - 1); j++, i++)
        {
            desc = Qmss_queuePop (p->myQ);
            if (desc)
            {
                Qmss_queuePush (p->myQ, desc, p->expSize, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
            }
            else
            {
                System_printf ("Error %s: pop failed queue empty after %d pop?\n", p->name, i);
                errorCount++;
                break;
            }
        }
        size = Qmss_getQueueByteCount (p->myQ);
        if ((size != expSize1) && (size != expSize2))
        {
            System_printf ("Error %s: got wrong size %d, expect %d or %d on desc %d\n",
                           p->name, size, expSize1, expSize2, i);
            errorCount++;
            break;
        }
    }

    /* Put back descriptor used for sync */
    Qmss_queuePush (p->myQ, syncDesc, size, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);

    return NULL;
}

/* Use 2 threads to simutaneously bang on 2 queues with different
 * packet sizes.  This proves NEON code works.  */
void threadedAtomicPushTest (Qmss_QueueHnd freeQA, Qmss_QueueHnd freeQB)
{
    uint8_t isAllocated;
    Qmss_QueueHnd threadAQHnd, threadBQHnd, tempQHnd;
    int32_t qmgrA, qmgrB, qmgrTemp;
    void *desc;
    int i;

    /* Open queue for each thread */
    threadAQHnd = Qmss_queueOpen
                      (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated);

    if (threadAQHnd < QMSS_SOK)
    {
        System_printf ("Error: Failed to open queue for thread A: %d\n", threadAQHnd);
        errorCount++;
        return;
    }

    threadBQHnd = Qmss_queueOpenInGroup
                      (Qmss_getQueueGroup (threadAQHnd),
                       Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated);

    if (threadBQHnd < QMSS_SOK)
    {
        System_printf ("Error: Failed to open queue for thread B: %d\n", threadBQHnd);
        errorCount++;
        return;
    }

    tempQHnd = Qmss_queueOpenInGroup
                      (Qmss_getQueueGroup (threadAQHnd),
                       Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                       QMSS_PARAM_NOT_SPECIFIED,
                       &isAllocated);

    if (tempQHnd < QMSS_SOK)
    {
        System_printf ("Error: Failed to open temp queue: %d\n", tempQHnd);
        errorCount++;
        return;
    }

    /* Make sure queues have the same qmgr */
    qmgrA    = Qmss_getQueueNumber (threadAQHnd).qMgr;
    qmgrB    = Qmss_getQueueNumber (threadBQHnd).qMgr;
    qmgrTemp = Qmss_getQueueNumber (tempQHnd).qMgr;
    if (qmgrA == qmgrB)
    {
        /* Don't need tempQHnd */
        Qmss_queueClose (tempQHnd);
    }
    else if (qmgrB == qmgrTemp)
    {
        /* Use tempQHnd for threadAQHnd */
        Qmss_queueClose (threadAQHnd);
        threadAQHnd = tempQHnd;
    }
    else
    {
        System_printf ("Error: Didn't get 2 queues in same qmgr\n");
        errorCount++;
        return;
    }

    /* Put 8 descriptors in each queue */
    for (i = 0; i < ATOMIC_PREFILL_ENTRIES; i ++)
    {
        desc = (void *)QMSS_DESC_PTR(Qmss_queuePop (freeQA));
        if (desc)
        {
            Qmss_queuePush (threadAQHnd, desc, THREAD_A_DATA_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
        }
        else
        {
            System_printf ("Error: Failed to pop thread A free Q\n");
            errorCount++;
            return;
        }
        desc = (void *)QMSS_DESC_PTR(Qmss_queuePop (freeQB));
        if (desc)
        {
            Qmss_queuePush (threadBQHnd, desc, THREAD_B_DATA_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
        }
        else
        {
            System_printf ("Error: Failed to pop thread B free Q\n");
            errorCount++;
            return;
        }
    }
    taskAParams.myQ     = threadAQHnd;
    taskAParams.otherQ  = threadBQHnd;
    taskAParams.expSize = THREAD_A_DATA_SIZE;
    taskAParams.name    = "thread A";
    taskBParams.myQ     = threadBQHnd;
    taskBParams.otherQ  = threadAQHnd;
    taskBParams.expSize = THREAD_B_DATA_SIZE;
    taskBParams.name    = "thread B";
    task_create (atomicPushTask, &taskBParams, &taskBHnd);
    /* Use this thread for thread A */
    atomicPushTask (&taskAParams);
    /* Wait for thread to finish */
    task_wait (&taskBHnd);
    /* Return the descriptors */
    while ((desc = Qmss_queuePop (threadAQHnd)))
    {
        Qmss_queuePushDesc (freeQA, desc);
    }
    while ((desc = Qmss_queuePop (threadBQHnd)))
    {
        Qmss_queuePushDesc (freeQB, desc);
    }
    Qmss_queueClose (threadAQHnd);
    Qmss_queueClose (threadBQHnd);
}
#endif

/***************************************************************************************
 * FUNCTION PURPOSE: Power up PA subsystem
 ***************************************************************************************
 * DESCRIPTION: this function powers up the PA subsystem domains
 ***************************************************************************************/
void passPowerUp (void)
{

#ifdef NSS_LITE
    /* PASS power domain is turned OFF by default. It needs to be turned on before doing any 
     * PASS device register access. This not required for the simulator. */

    /* Set NSS Power domain to ON */        
    CSL_PSC_enablePowerDomain (CSL_PSC_PD_NSS);

    /* Enable the clocks for NSS modules */
    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_NSS, PSC_MODSTATE_ENABLE);

    /* Start the state transition */
    CSL_PSC_startStateTransition (CSL_PSC_PD_NSS);

    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_NSS));
#endif  
  
}

void testDescAllocation (void)
{
    Qmss_Result             result;
    uint32_t                numAllocated, corenum, i;
    Qmss_QueueHnd           freeQueHnd, txFreeQueHnd, miscQueHnd;
    uint32_t                descCount;
    uint32_t                qGroup;
#ifdef TEST_QOS
    uint32_t                baseQueue;
    Qmss_QueueHnd           baseQueueHnd;
    Qmss_QosQueueCfg        queueCfg;
    Qmss_QosClusterCfg      clusterCfg;
    Qmss_QosClusterCfgRR   *clusterCfgRR;
    Qmss_QosClusterCfgTB   *clusterCfgTB;
#if QMSS_MAX_TRAFFIC_SHAPING_QUEUE < 64
    Qmss_QueueHnd           queueHnds[64];
#endif
#endif
    Qmss_Result             reg0, reg1;

    /* Get the core number. */
#ifndef __LINUX_USER_SPACE
    passPowerUp();
#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING DESCRIPTOR ALLOCATION ************\n", corenum);

#ifndef __LINUX_USER_SPACE
    memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));
    memset ((void *) &linkingRAM0, 0, sizeof (linkingRAM0));

    /* Set up the linking RAM. Use external Linking RAM.
     * LLD will configure the internal linking RAM address and default size if a value of zero is specified.
     * Linking RAM1 is not used */
    #ifndef NSS_LITE 
    qmssInitConfig.linkingRAM0Base = l2_global_address ((uint32_t) linkingRAM0);
    #else
    qmssInitConfig.linkingRAM0Base = 0;
    #endif
    qmssInitConfig.linkingRAM0Size = NUM_MONOLITHIC_DESC + NUM_MISC_DESC;
    qmssInitConfig.linkingRAM1Base = 0x0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC + NUM_MISC_DESC;

#ifdef TEST_QOS /* Not supported on simulator */
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP2;
    qmssInitConfig.pdspFirmware[0].firmware = &qos_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP2;
    qmssInitConfig.pdspFirmware[0].firmware = &qos_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_le);
#endif
#endif
    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", corenum, result);
        errorCount++;
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", corenum, result);
    }
    monolithicDesc = monolithicDescBuf;
    miscDesc = miscDescBuf;
#else
    initRm();
    initQmss((NUM_MONOLITHIC_DESC + NUM_MISC_DESC), false);

    /* Allocate memory for descriptor region */
    monolithicDesc = (uint8_t *)fw_memAlloc((SIZE_MONOLITHIC_DESC *
                                             NUM_MONOLITHIC_DESC),
                                            CACHE_LINESZ);
    if (monolithicDesc == NULL) {
        System_printf ("Core %d : Error allocating monolithic descriptor memory\n", corenum);
        return;
    }

    /* Allocate memory for descriptor region */
    miscDesc = (uint8_t *)fw_memAlloc((SIZE_MISC_DESC *
                                       NUM_MISC_DESC),
                                      CACHE_LINESZ);
    if (miscDesc == NULL) {
        System_printf ("Core %d : Error allocating misc descriptor memory\n", corenum);
        return;
    }
#endif

    /* Setup memory region for monolithic descriptors */
    memset ((void *) monolithicDesc, 0, (SIZE_MONOLITHIC_DESC *
                                         NUM_MONOLITHIC_DESC));
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) monolithicDesc);
    memInfo.descSize = SIZE_MONOLITHIC_DESC;
    memInfo.descNum = NUM_MONOLITHIC_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg0 = Qmss_insertMemoryRegion (&memInfo);
    if (reg0 < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, memInfo.memRegion, reg0);
        errorCount++;
    }

    /* Setup memory region for teardown descriptors */
    memset ((void *) miscDesc, 0, (SIZE_MISC_DESC * NUM_MISC_DESC));
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) miscDesc);
    memInfo.descSize = SIZE_MISC_DESC;
    memInfo.descNum = NUM_MISC_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg1 = Qmss_insertMemoryRegion (&memInfo);
    if (reg1 < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, memInfo.memRegion, reg1);
        errorCount++;
    }

    System_printf ("\n~~~~~~~~~~Core %d Memory region Configuration~~~~~~~~~~~~\n", corenum);
    result = Qmss_getMemoryRegionCfg (&memRegStatus);
    if (result != QMSS_SOK)
    {
        System_printf ("Error Core %d : Getting memory region configuration info error code : %d\n", corenum, result);
        errorCount++;
    }

    System_printf ("Current Desc count  : %d\n", memRegStatus.currDescCnt);
    for (i = 0; i < QMSS_MAX_MEM_REGIONS; i++)
    {
        if (memRegStatus.memRegInfo[i].descBase != 0)
        {
            System_printf ("\nMemory Region Index : %d\n", memRegStatus.memRegInfo[i].memRegion);
            System_printf ("Start Index         : %d\n", memRegStatus.memRegInfo[i].startIndex);
            System_printf ("Descriptor Size     : %d\n", memRegStatus.memRegInfo[i].descSize);
            System_printf ("Descriptor Num      : %d\n", memRegStatus.memRegInfo[i].descNum);
            System_printf ("Descriptor Base     : 0x%p\n", memRegStatus.memRegInfo[i].descBase);
            System_printf ("Managed Descriptor  : %d\n", memRegStatus.memRegInfo[i].manageDescFlag);
        }
    }

    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_MONOLITHIC_DESC / 2;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifndef NSS_LITE
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    
    /* Initialize the descriptors and push to free Queue */
    if ((txFreeQueHnd = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing Tx descriptor error code: %d \n", corenum, txFreeQueHnd);
        errorCount++;
    }
    else
    {
        if (descCfg.descNum != numAllocated)
        {
            errorCount++;
        }

        System_printf ("Core %d : Number of Tx descriptors requested : %d. Number of descriptors allocated : %d \n",
            corenum, descCfg.descNum, numAllocated);
    }
    /* Setup the descriptors for receive free queue */
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_MONOLITHIC_DESC / 2;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifndef NSS_LITE
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
   
    /* Initialize the descriptors and push to free Queue */
    if ((freeQueHnd = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing Rx descriptor error code: %d \n", corenum, freeQueHnd);
        errorCount++;
    }
    else
    {
        if (descCfg.descNum != numAllocated)
        {
            errorCount++;
        }
        System_printf ("Core %d : Number of Rx descriptors requested : %d. Number of descriptors allocated : %d \n",
            corenum, descCfg.descNum, numAllocated);
    }


    /* Setup the teardown free queue */
    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION1;
    descCfg.descNum = NUM_MISC_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((miscQueHnd = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing misc descriptor error code: %d \n", corenum, miscQueHnd);
        errorCount++;
    }
    else
    {
        if (descCfg.descNum != numAllocated)
        {
            errorCount++;
        }
        System_printf ("Core %d : Number of misc descriptors requested : %d. Number of descriptors allocated : %d \n",
                corenum, descCfg.descNum, numAllocated);
    }

    /* Test setting EOI vector for high priority interrupt number 1 */
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, 1);
    /* Test setting EOI vector for low priority interrupt number 2 */
    Qmss_setEoiVector (Qmss_IntdInterruptType_LOW, 2);
    /* Test setting EOI vector for CDMA interrupt number 0 */
    Qmss_setEoiVector (Qmss_IntdInterruptType_CDMA, 0);

#ifdef TEST_QOS
#if QMSS_MAX_TRAFFIC_SHAPING_QUEUE < 64
    /* Need 64 queues for RR cluster so reserve 64 general purpose queues */
    /* Find first queue aligned to 32 */
    baseQueueHnd = Qmss_queueBlockOpen (queueHnds, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 64, 32);
    if (baseQueueHnd < 0) {
        System_printf ("Core %d : Failed to find range of 64 queues for QoS\n", corenum);
        errorCount++;
    }
    baseQueue = (uint32_t)baseQueueHnd;
#else
    baseQueue = QMSS_TRAFFIC_SHAPING_QUEUE_BASE;
#endif
    if ((result = Qmss_setQosQueueBase (baseQueue)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Setting QoS queue base address error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Set QoS queue base address return code: %d \n", corenum, result);
    }

    if ((result = Qmss_getQosQueueBase (&baseQueue)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Getting QoS queue base address error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Get QoS queue base address queue number : %d return code: %d \n", corenum, baseQueue, result);
    }
    if ((result = Qmss_configureQosTimer (0x1234)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Setting QoS timer error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Set QoS timer return code: %d \n", corenum, result);
    }

    queueCfg.egressQueNum = 123;
    queueCfg.iterationCredit = 10;
    queueCfg.maxCredit = 15;
    queueCfg.congestionThreshold = 255;

    baseQueue = 0;
    if ((result = Qmss_configureQosQueue (baseQueue, &queueCfg)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Configuring QoS queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Configured QoS queue %d return code: %d \n", corenum, baseQueue, result);
    }

    queueCfg.egressQueNum = 124;
    queueCfg.iterationCredit = 16;
    queueCfg.maxCredit = 32;
    queueCfg.congestionThreshold = 254;

    baseQueue = 1;

    if ((result = Qmss_configureQosQueue (baseQueue, &queueCfg)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Configuring QoS queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Configured QoS queue %d return code: %d \n", corenum, baseQueue, result);
    }

    clusterCfgTB = &clusterCfg.u.cfgTB;
    clusterCfg.mode = Qmss_QosMode_TokenBucket;
    clusterCfgTB->maxGlobalCredit = 32;
    clusterCfgTB->qosQueCnt = 2;
    clusterCfgTB->qosQueNum[0] = 0;
    clusterCfgTB->qosQueNum[1] = 1;
    clusterCfgTB->qosQueRTFlags = 3;
    clusterCfgTB->egressQueCnt = 2;
    clusterCfgTB->egressQueNum[0].qMgr = 0;
    clusterCfgTB->egressQueNum[0].qNum = 124;
    clusterCfgTB->egressQueNum[1].qMgr = 0;
    clusterCfgTB->egressQueNum[1].qNum = 640;
    clusterCfgTB->egressCongestionThreshold1 = 8;
    clusterCfgTB->egressCongestionThreshold2 = 16;
    clusterCfgTB->egressCongestionThreshold3 = 32;
    clusterCfgTB->egressCongestionThreshold4 = 64;

    if ((result = Qmss_configureQosCluster (0, &clusterCfg)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Configuring QoS cluster error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Configured QoS cluster return code: %d \n", corenum, result);
    }

    baseQueue = 0;
    if ((result = Qmss_getQosQueueForwardPktStats (baseQueue)) == QMSS_QCMD_INVALID_INDEX)
    {
        System_printf ("Error Core %d : Get forwarded packet count for queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Get forwarded packet count for queue %d is : %d \n", corenum, baseQueue, result);
    }

    baseQueue = 1;
    if ((result = Qmss_getQosQueueForwardPktStats (baseQueue)) == QMSS_QCMD_INVALID_INDEX)
    {
        System_printf ("Error Core %d : Get forwarded packet count for queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Get forwarded packet count for queue %d is : %d \n", corenum, baseQueue, result);
    }

    baseQueue = 0;
    if ((result = Qmss_getQosQueueDroppedPktStats (baseQueue)) == QMSS_QCMD_INVALID_INDEX)
    {
        System_printf ("Error Core %d : Get dropped packet count for queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Get dropped packet count for queue %d is : %d \n", corenum, baseQueue, result);
    }

    baseQueue = 1;
    if ((result = Qmss_getQosQueueDroppedPktStats (baseQueue)) == QMSS_QCMD_INVALID_INDEX)
    {
        System_printf ("Error Core %d : Get dropped packet count for queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Get dropped packet count for queue %d is : %d \n", corenum, baseQueue, result);
    }

    baseQueue = 0;
    if ((result = Qmss_resetQosQueueStats (baseQueue)) == QMSS_QCMD_INVALID_INDEX)
    {
        System_printf ("Error Core %d : Reset stats for queue %d error code: %d \n", corenum, baseQueue, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Reset stats for queue %d is : %d \n", corenum, baseQueue, result);
    }

    if ((result = Qmss_enableQosCluster (0)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Enabling QoS cluster error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Enabled QoS cluster return code: %d \n", corenum, result);
    }

    if ((result = Qmss_disableQosCluster (0)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Disabling QoS cluster error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Disabled QoS cluster return code: %d \n", corenum, result);
    }

    /* Set up a Round Robin cluster */
    queueCfg.egressQueNum = 125;
    queueCfg.iterationCredit = 10;
    queueCfg.maxCredit = 15;
    queueCfg.congestionThreshold = 255;

    /* Set up input queues */
    for (baseQueue = 56; baseQueue < 64; baseQueue++)
    {
        if ((result = Qmss_configureQosQueue (baseQueue, &queueCfg)) != QCMD_RETCODE_SUCCESS)
        {
            System_printf ("Error Core %d : Configuring RR cluster QoS queue %d error code: %d \n", corenum, baseQueue, result);
            errorCount++;
        }
    }

    /* Set up cluster */
    memset (&clusterCfg, 0, sizeof(clusterCfg));
    clusterCfgRR                       = &clusterCfg.u.cfgRR;
    clusterCfg.mode                    = Qmss_QosMode_RoundRobin;
    clusterCfgRR->maxGlobalCredit      = 32;
    clusterCfgRR->qosQueHighCnt        = 4;
    clusterCfgRR->qosQueNumHigh[0]     = 56;
    clusterCfgRR->qosQueNumHigh[1]     = 57;
    clusterCfgRR->qosQueNumHigh[2]     = 58;
    clusterCfgRR->qosQueNumHigh[3]     = 59;
    clusterCfgRR->qosQueLowCnt         = 4;
    clusterCfgRR->qosQueNumLow[0]      = 60;
    clusterCfgRR->qosQueNumLow[1]      = 61;
    clusterCfgRR->qosQueNumLow[2]      = 62;
    clusterCfgRR->qosQueNumLow[3]      = 63;
    clusterCfgRR->sizeAdjust           = 24;
    clusterCfgRR->egressQueCnt         = 1;
    clusterCfgRR->egressQueNum[0].qMgr = 0;
    clusterCfgRR->egressQueNum[0].qNum = 125;
    clusterCfgRR->iterationCredit      = 10;
    clusterCfgRR->maxEgressBacklog     = 64;
    clusterCfgRR->queueDisableMask     = 0;

    /* Negative tests */
    if ((result = Qmss_configureQosCluster (0, &clusterCfg)) != QMSS_QCMD_INVALID_INDEX) {
        /* Should fail, only cluster 7 supports RR */
        System_printf ("Error Core %d : Configuring RR cluster 0 didn't fail: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    clusterCfgRR->qosQueHighCnt        = 0;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QMSS_QCMD_INVALID_RR_HIGH_Q) {
        /* Should fail, must have 4 high priority queues */
        System_printf ("Error Core %d : Configuring RR cluster didn't fail qosQueHighCnt: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    clusterCfgRR->qosQueHighCnt        = 4;
    clusterCfgRR->qosQueLowCnt         = 0;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QMSS_QCMD_INVALID_RR_LOW_Q) {
        /* Should fail, must have 4 high priority queues */
        System_printf ("Error Core %d : Configuring RR cluster didn't fail qosQueLowCnt: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    clusterCfgRR->qosQueLowCnt         = 4;
    clusterCfgRR->egressQueCnt         = 0;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QMSS_QCMD_INVALID_RR_EGRESS_Q) {
        /* Should fail, must have 4 high priority queues */
        System_printf ("Error Core %d : Configuring RR cluster didn't fail egressQueCnt: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    clusterCfgRR->egressQueCnt         = 1;
    clusterCfgRR->qosQueNumHigh[3]     = 0;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QMSS_QCMD_INVALID_RR_HIGH_Q) {
        /* Should fail, must have 4 high priority queues */
        System_printf ("Error Core %d : Configuring RR cluster didn't fail qosQueueNumHigh[]: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    clusterCfgRR->qosQueNumHigh[3]     = 59;
    clusterCfgRR->qosQueNumLow[3]      = 0;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QMSS_QCMD_INVALID_RR_LOW_Q) {
        /* Should fail, must have 4 high priority queues */
        System_printf ("Error Core %d : Configuring RR cluster didn't fail qosQueueNumLow[]: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    /* Positive test case */
    clusterCfgRR->qosQueNumLow[3]      = 63;
    if ((result = Qmss_configureQosCluster (7, &clusterCfg)) != QCMD_RETCODE_SUCCESS) {
        /* Should succeed */
        System_printf ("Error Core %d : Correctly onfiguring RR cluster failed: error code: %d \n",
                       corenum, result);
        errorCount++;
    }

    if ((result = Qmss_enableQosCluster (7)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Enabling QoS RR cluster error code: %d \n", corenum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Enabled QoS RR cluster return code: %d \n", corenum, result);
    }

    /* Clean up queues */
    for (i = 0; i < 64; i++)
    {
        if ((result = Qmss_queueClose (queueHnds[i])) < QMSS_SOK)
        {
            System_printf ("Error closing queueHnds[%d]: %d\n", i, result);
            errorCount++;
        }
    }
#endif


#ifndef NSS_LITE /* There is no startvation queue at NSS_LITE devices */

    /* Test starvation counter queue in each queue group */
    System_printf ("Core %d: testing starvation queues\n", corenum);
    for (qGroup = 0; qGroup < qmssLObj[0].p.maxQueMgrGroups; qGroup++)
    {
        uint8_t isAllocated;
        Qmss_QueueHnd starvQ = Qmss_queueOpenInGroup
                                   (qGroup,
                                    Qmss_QueueType_STARVATION_COUNTER_QUEUE,
                                    QMSS_PARAM_NOT_SPECIFIED,
                                    &isAllocated);

        if (starvQ >= QMSS_SOK)
        {
            uint32_t stats;
            /* Clear stats */
            stats = Qmss_getStarvationCount (starvQ);
            /* Check for 0 */
            stats = Qmss_getStarvationCount (starvQ);
            if (stats != 0)
            {
                System_printf ("Core %d: stats queue %d bad (exp 0): %d\n",
                               corenum, starvQ, stats);
                errorCount++;
            }
            else
            {
                void *desc = Qmss_queuePop (starvQ); /* cause starvation */
                if (! desc)
                {
                    /* Check stats */
                    stats = Qmss_getStarvationCount (starvQ);
                    if (stats != 1)
                    {
                        System_printf ("Core %d: stats queue %d bad (exp 1): %d\n",
                                        corenum, starvQ, stats);
                        errorCount++;
                    }
                    else
                    {
                        /* queue worked */
                        System_printf ("Core %d: stats for queue %d: %d\n",
                                        corenum, starvQ, stats);
                    }
                }
                else
                {
                    System_printf ("Core %d: Got an unexpected desc\n",
                                    corenum);
                    errorCount++;
                }
            }

            if ((result = Qmss_queueClose (starvQ)) < QMSS_SOK)
            {
                System_printf ("Core %d: failed to close starvQ (%d): %d\n",
                                corenum, starvQ, result);
                errorCount++;
            }
        }
    }

#if defined(__ARM_ARCH_7A__) && defined(__LINUX_USER_SPACE)
    threadedAtomicPushTest (freeQueHnd, txFreeQueHnd);
#endif
#endif

    /* Make sure no descriptors were lost */
    descCount  = Qmss_getQueueEntryCount (freeQueHnd); 
    descCount += Qmss_getQueueEntryCount (txFreeQueHnd);
    descCount += Qmss_getQueueEntryCount (miscQueHnd);

    if (descCount != (NUM_MONOLITHIC_DESC + NUM_MISC_DESC))
    {
        System_printf ("Core %d: Missing %d descriptors\n", corenum,
                       NUM_MONOLITHIC_DESC + NUM_MISC_DESC - descCount);
        errorCount++;
    }
    /* Discard all the descriptors, to make test case re-runnable without reset */
    Qmss_queueEmpty (freeQueHnd);
    Qmss_queueEmpty (txFreeQueHnd);
    Qmss_queueEmpty (miscQueHnd);

    /* Close the queues */
    if ( (result = Qmss_queueClose (freeQueHnd)) != QMSS_SOK)
    {
        System_printf ("Error closing freeQue %d: %d\n", freeQueHnd, result);
        errorCount++;
    }
    if ( (result = Qmss_queueClose (txFreeQueHnd)) != QMSS_SOK)
    {
        System_printf ("Error closing txFreeQue %d: %d\n", txFreeQueHnd, result);
        errorCount++;
    }
    if ( (result = Qmss_queueClose (miscQueHnd)) != QMSS_SOK)
    {
        System_printf ("Error closing miscQue %d: %d\n", miscQueHnd, result);
        errorCount++;
    }

    /* Free the regions */
    if ( (result = Qmss_removeMemoryRegion (reg0, 0)) != QMSS_SOK)
    {
        System_printf ("Error removing memory region 0: %d\n", result);
        errorCount++;
    }
    if ( (result = Qmss_removeMemoryRegion (reg1, 0)) != QMSS_SOK)
    {
        System_printf ("Error removing memory region 1: %d\n", result);
        errorCount++;
    }

    System_printf ("Core %d : exit QM\n", corenum);
    if ( (result = Qmss_exit()) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error exiting QM: %d \n", corenum, result);
    }

    if (errorCount == 0)
    {
        System_printf ("\nCore %d : Descriptor allocation tests Passed\n", corenum);
    }
    else
    {
        System_printf ("Core %d test failed with %d errors\n", corenum, errorCount);
    }
}

void run_test (void)
{
    testDescAllocation ();
}


