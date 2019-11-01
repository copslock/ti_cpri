/**
 *   @file  test_srioContext.c
 *
 *   @brief
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <string.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/drv/qmss/qmss_qos.h>

/* CPPI LLD includes */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>

/************************ USER DEFINES ********************/
#define NUM_MONOLITHIC_DESC         2048
#define SIZE_MONOLITHIC_DESC        32
#define PROFILE_DESCS               400
#define TOTAL_GARBAGE_DESCS         (((PROFILE_DESCS * QMSS_QOS_SRIO_MAX_TX_Q) / QMSS_QOS_SRIO_MAX_GARBAGE_Q) * QMSS_QOS_SRIO_MAX_GARBAGE_Q)

#if (PROFILE_DESCS * QMSS_QOS_SRIO_MAX_TX_Q) > NUM_MONOLITHIC_DESC
#error NUM_MONOLITHIC_DESC is too small
#endif

/************************ GLOBAL VARIABLES ********************/
/* Descriptor pool [Size of descriptor * Number of descriptors] */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (monolithicDesc, 16)
UInt8 monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#else
UInt8 monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
#endif

/* Timestamp when SRIO model moved each descriptor */
struct
{
    void    *descPtr;
    int      queueNum;
    uint32_t timestamp;
} timestamps[NUM_MONOLITHIC_DESC];

/* Global variable common to all test cases */

/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;
/* QM descriptor configuration */
Qmss_DescCfg            descCfg;
/* Store the queue handle for destination queues on which allocated descriptors are stored */
Qmss_QueueHnd           QueHnd[QMSS_MAX_MEM_REGIONS];

/************************ EXTERN VARIABLES ********************/
/* Error counter */
extern UInt32                   errorCount;
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
static UInt32 l2_global_address (UInt32 addr)
{
#ifdef _TMS320C6X
    UInt32 corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
}

/*****************************************************************************
 * Divert with error check
 *****************************************************************************/
void queue_divert_and_check (Qmss_QueueHnd src, Qmss_QueueHnd dst)
{
    Qmss_Result result;

    if ((result = Qmss_queueDivert (src, dst, Qmss_Location_TAIL)) != QMSS_SOK)
    {
        UInt32 corenum;

#ifdef _TMS320C6X
    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
        errorCount++;
        System_printf("core %d: queue divert from %d to %d failed: %d\n", corenum, src, dst, result);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to sleep N cycles.  This is immune to timer rollover.
 *
 *  @param[in]  n
 *      Number of cycles to sleep
 *
 */
void delay (uint32_t cycles)
{
#ifdef _TMS320C6X
    uint32_t start = TSCL;

    while ( (TSCL - start) < cycles);
#else
    uint32_t start, end;

    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(start));
    if ((0x100000000 - start) < cycles) {
        do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
        } while (end > start);
        cycles -= 0x100000000 - start;
        start = 0;
    }
    do {
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(end));
    } while ((end - start) < cycles);
#endif
}

Void testSrioContext (Void)
{
    Qmss_Result             result;
    UInt32                  numAllocated, corenum;
    Qmss_QueueHnd           freeQ;
    Qmss_QueueHnd           baseQueue;
    int                     numQueues;
    int                     queueNum;
    int                     nextGarbageQ;
    int                     descNum;
    Qmss_QueueHnd           fwQHnds[QMSS_QOS_SRIO_TX_MAX_FW_Q];
    Qmss_QueueHnd           srioSimQHnds[QMSS_QOS_SRIO_MAX_TX_Q];
    Qmss_QueueHnd           srioDivQHnds[QMSS_QOS_SRIO_MAX_TX_Q];
    Qmss_QueueHnd           srioGarbageRetQ[QMSS_QOS_SRIO_MAX_GARBAGE_Q];
    Qmss_QosSrioCfg         srioCfg;
    uint8_t                 isAllocated;
    uint32_t                fwQGroup;
    uint32_t                freeQGroup;

    uint32_t startTime, endTime;

#ifdef _TMS320C6X
    /* Reset timer */
    TSCL = 0;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING DESCRIPTOR ALLOCATION ************\n", corenum);

    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    /* Set up the linking RAM. Use internal Linking RAM.  */
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0x0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC;

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP2;
    qmssInitConfig.pdspFirmware[0].firmware = &qos_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP2;
    qmssInitConfig.pdspFirmware[0].firmware = &qos_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (qos_le);
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

    System_printf ("Core %d : QoS Firmware Rev 0x%08x\n", corenum,
                   Qmss_getQosFwVersion());

    /* Setup memory region for monolithic descriptors */
    memset ((Void *) &monolithicDesc, 0, SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC);
    memInfo.descBase = (UInt32 *) l2_global_address ((UInt32) monolithicDesc);
    memInfo.descSize = SIZE_MONOLITHIC_DESC;
    memInfo.descNum = NUM_MONOLITHIC_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    result = Qmss_insertMemoryRegion (&memInfo);
    if (result < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", corenum, memInfo.memRegion, result);
        errorCount++;
    }

    descCfg.memRegion = Qmss_MemRegion_MEMORY_REGION0;
    descCfg.descNum = NUM_MONOLITHIC_DESC;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;

    /* Initialize the descriptors and push to free Queue */
    if ((freeQ = Qmss_initDescriptor (&descCfg, &numAllocated)) < 0)
        {
        System_printf ("Error Core %d : Initializing descriptor error code: %d \n", corenum, freeQ);
        errorCount++;
        }
    else
    {
        if (descCfg.descNum != numAllocated)
        {
            errorCount++;
        }

        System_printf ("Core %d : Number of descriptors requested : %d. Number of descriptors allocated : %d \n",
            corenum, descCfg.descNum, numAllocated);
    }

    /* Get queue group of free Q, so we can allocate other queues that divert
     * to this queue from same group */
    freeQGroup = Qmss_getQueueGroup (freeQ);

    /* Allocate block of queues to be used by firmware */
    baseQueue = Qmss_queueBlockOpen (fwQHnds, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_QOS_SRIO_TX_MAX_FW_Q, 32);
    if (baseQueue < 0)
    {
        System_printf ("Core %d : Failed to open %d contiguous queues\n", corenum, QMSS_QOS_SRIO_TX_MAX_FW_Q);
        errorCount++;
    }
    memset(&srioCfg, 0, sizeof(srioCfg));
    srioCfg.queBase = baseQueue;

    /* Get queue gruop of firmware queues, so diversion queues can be
     * allocated in same group */
    fwQGroup = Qmss_getQueueGroup (fwQHnds[0]);

    /* Allocate queues to simulate SRIO HW */
    for (queueNum = 0; queueNum < QMSS_QOS_SRIO_MAX_TX_Q; queueNum++)
    {
        /* Allocate in same group as firmware owned queues, so we can
         * divert between the queues */
        srioSimQHnds[queueNum] =
            Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                            QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (srioSimQHnds[queueNum] < 0)
        {
            errorCount++;
            System_printf("Queue open failed: %d\n", srioSimQHnds[queueNum]);
            break;
        }
        srioCfg.TXQs[queueNum].threshold = 10;
        srioCfg.TXQs[queueNum].txQ = Qmss_getQueueNumber (srioSimQHnds[queueNum]);
    }

    /* Allocate queues to temporarily hold descriptors during test setup.
     * They will be diverted to shadow queues all at once
     */
    for (queueNum = 0; queueNum < QMSS_QOS_SRIO_MAX_TX_Q; queueNum++)
    {
        srioDivQHnds[queueNum] =
            Qmss_queueOpenInGroup (fwQGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                   QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (srioDivQHnds[queueNum] < 0)
        {
            errorCount++;
            System_printf("Queue open failed: %d\n", srioDivQHnds[queueNum]);
            break;
        }
    }

    /* Allocate queues for garbage return queues in same group
     * as free q, so descriptors can be diverted back to free q */
    for (queueNum = 0; queueNum < QMSS_QOS_SRIO_MAX_GARBAGE_Q; queueNum++)
    {
        srioGarbageRetQ[queueNum] =
            Qmss_queueOpenInGroup (freeQGroup, Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                   QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
        if (srioGarbageRetQ[queueNum] < 0)
        {
            errorCount++;
            System_printf("Queue open failed: %d\n", srioGarbageRetQ[queueNum]);
            break;
        }
        srioCfg.garbageRetQs[queueNum] = Qmss_getQueueNumber (srioGarbageRetQ[queueNum]);
    }

#if 1
    if ((result = Qmss_configureQosTimer (437) != QCMD_RETCODE_SUCCESS))
    {
        errorCount++;
        System_printf("Core %d : Failed to configure QoS timer: %d\n", corenum, result);
    }
#endif

    if (errorCount)
    {
        System_printf ("Test failed\n");
        return;
    }


    /* Iterate over each number of supported tx queues, and calculate throughput */
    for (numQueues = QMSS_QOS_SRIO_MIN_TX_Q; numQueues <= QMSS_QOS_SRIO_MAX_TX_Q; numQueues++)
    {
        int descID = 0;

        /* Set up the cluster */
        srioCfg.queCount = numQueues;
        if ((result = Qmss_configureQosSrioCluster (0, &srioCfg)) != QCMD_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to configure SRIO cluster : %d\n", corenum, result);
            errorCount++;
            break;
        }

        /* Place PROFILE_DESCS descriptors into each input queue */
        for (queueNum = 0; queueNum < numQueues; queueNum++)
        {
            for (descNum = 0; descNum < PROFILE_DESCS; descNum++)
            {
                uint32_t *desc;
                desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (freeQ));
                if (! desc)
                {
                    System_printf ("Core %d : failed to pop a free desc\n", corenum);
                    errorCount++;
                    break;
                }
                /* No cache operation is needed since only the same CPU will read */
                *desc = descID ++; /* write unique ID to each descriptor */
                Qmss_queuePushDesc (srioDivQHnds[queueNum], desc);
            }
        }

        /* Enable the cluster */
        if ((result = Qmss_enableQosSrioCluster (0)) != QCMD_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to enable SRIO cluster : %d\n", corenum, result);
            errorCount++;
            break;
        }

        /* Start the clock */
#ifdef _TMS320C6X
        startTime = TSCL;
#else
        __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(startTime));
#endif
        /* Divert each queue of descriptors to a firmware monitored shadow queue */
        for (queueNum = 0; queueNum < numQueues; queueNum++)
        {
            queue_divert_and_check (srioDivQHnds[queueNum],
                                    fwQHnds[queueNum + QMSS_QOS_SRIO_SHADOW_TX_Q_OFFSET]);
        }

        /* Run a simple model of the SRIO HW to move the descriptors placed on HW queues
         * to the shadow TX completion queues
         */
        queueNum = 0;
        for (descNum = 0; descNum < PROFILE_DESCS * numQueues; )
        {
           uint32_t *desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (srioSimQHnds[queueNum]));
           if (desc)
           {
#ifdef _TMS320C6X
                timestamps[descNum].timestamp = TSCL;
#else
                __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timestamps[descNum].timestamp));
#endif
                timestamps[descNum].queueNum  = queueNum;
                timestamps[descNum].descPtr   = desc;
                descNum++;
                Qmss_queuePushDesc (fwQHnds[queueNum + QMSS_QOS_SRIO_SHADOW_TX_COMPLETION_Q_OFFSET],
                                    desc);
           }
           else
           {
               /* move to next queue if this queue is empty */
               queueNum++;
               if (queueNum >= numQueues)
               {
                   queueNum = 0;
               }
           }
        }
        /* Wait for the descriptors to come back */
        do
        {
            descID = 0;

            for (queueNum = 0; queueNum < numQueues; queueNum++)
            {
                descID += Qmss_getQueueEntryCount (fwQHnds[queueNum + QMSS_QOS_SRIO_TX_COMPLETION_Q_OFFSET]);
            }
        } while (descID < (PROFILE_DESCS * numQueues));
#ifdef _TMS320C6X
        endTime = TSCL;
#else
        __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(endTime));
#endif
        System_printf ("Core %d: moved %d descriptors on %d queues (%d total) "
                       "in %d cycles (%d cycles per descriptor)\n",
                       corenum,
                       PROFILE_DESCS,
                       numQueues,
                       PROFILE_DESCS * numQueues,
                       endTime - startTime,
                       (endTime - startTime) / (PROFILE_DESCS * numQueues));


        /* Disable the cluster */
        if ((result = Qmss_disableQosSrioCluster (0)) != QCMD_RETCODE_SUCCESS)
        {
            System_printf ("Core %d : failed to disable SRIO cluster : %d\n", corenum, result);
            errorCount++;
            break;
        }

        /* Verify the results and return descriptors to free q */
        descID = 0;
        for (queueNum = 0; queueNum < numQueues; queueNum++)
        {
            for (descNum = 0; descNum < PROFILE_DESCS; descNum++)
            {
                uint32_t *desc;
                desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (
                          fwQHnds[queueNum + QMSS_QOS_SRIO_TX_COMPLETION_Q_OFFSET]));
                if (! desc)
                {
                    System_printf ("Core %d : failed to pop completed desc\n", corenum);
                    errorCount++;
                    break;
                }
                /* No cache operation is needed since only the same CPU wrote */
                if (*desc != descID)
                {
                    System_printf ("Core %d : completed descriptors out of "
                                   "order got %d expect %d\n",
                                   corenum,
                                   *desc,
                                   descID);
                    errorCount++;
                }
                descID = *desc + 1;
                Qmss_queuePushDesc (freeQ, desc);
            }
        }
    }

    /* Test garbage queues */
    System_printf ("Core %d: running test on shadow garbage queues\n", corenum);

    /* Set up the cluster */
    srioCfg.queCount = QMSS_QOS_SRIO_MAX_TX_Q;
    if ((result = Qmss_configureQosSrioCluster (0, &srioCfg)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to configure SRIO cluster : %d\n", corenum, result);
        errorCount++;
        goto fail;
    }

    /* Enable the cluster */
    if ((result = Qmss_enableQosSrioCluster (0)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to enable SRIO cluster : %d\n", corenum, result);
        errorCount++;
        goto fail;
    }

    /* Distribute GARBAGE_DESCS into each of the shadow TX queues */
    descNum = 0;
    queueNum = 0;
    for (descNum = 0; descNum < TOTAL_GARBAGE_DESCS; descNum++)
    {
        uint32_t *desc;
        Qmss_Queue returnQNum;
        desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (freeQ));
        if (! desc)
        {
            System_printf ("Core %d : failed to pop a free desc\n", corenum);
            errorCount++;
            break;
        }
        /* Only set the return queue as if the descriptor is CPPI and
         * set the descriptor type.  Otherwise QoS doesn't use any of the
         * fields, so don't set them.
         *
         * The SRIO context tracker frees the context if and only if
         * the return queue matches one of the shadow transmit completion
         * queues.
         *
         * In the prior test cases, firmware doesn't use the return queue,
         * so it was not set.
         */
        returnQNum = Qmss_getQueueNumber (fwQHnds[queueNum + QMSS_QOS_SRIO_SHADOW_TX_COMPLETION_Q_OFFSET]);
        Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, (Cppi_Desc *)desc, returnQNum);
        Cppi_setDescType ((Cppi_Desc *)desc, (Cppi_DescType_MONOLITHIC));
        Qmss_queuePushDesc (fwQHnds[queueNum + QMSS_QOS_SRIO_SHADOW_TX_Q_OFFSET], desc);

        queueNum++;
        if (queueNum >= QMSS_QOS_SRIO_MAX_TX_Q)
        {
            queueNum = 0;
        }
    }

    /* Now run a SRIO model that treats all the TX descriptors as garbage
     * instead of putting in normal completion queue */
    queueNum = 0;
    nextGarbageQ = 0;
    for (descNum = 0; descNum < TOTAL_GARBAGE_DESCS; )
    {
        uint32_t *desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (srioSimQHnds[queueNum]));
        if (desc)
        {
            descNum++;
            /* Put the descriptor in one of the garbage queues */
            Qmss_queuePushDesc (fwQHnds[nextGarbageQ + QMSS_QOS_SRIO_SHADOW_GARBAGE_Q_OFFSET],
                                desc);
            nextGarbageQ++;
            if (nextGarbageQ >= QMSS_QOS_SRIO_MAX_GARBAGE_Q)
            {
                nextGarbageQ = 0;
            }
        }
        else
        {
            /* move to next queue if this queue is empty */
            queueNum++;
            if (queueNum >= QMSS_QOS_SRIO_MAX_TX_Q)
            {
                queueNum = 0;
            }
        }
    }

    /* Delay to let the firmware finish */
    delay (1000000);

    /* Verify the results - all the descriptors should be moved to the corresponding garbage return queues */
    for (queueNum = 0; queueNum < QMSS_QOS_SRIO_MAX_GARBAGE_Q; queueNum++)
    {
        uint32_t queueDepth = Qmss_getQueueEntryCount (srioGarbageRetQ[queueNum]);
        if (queueDepth != TOTAL_GARBAGE_DESCS / QMSS_QOS_SRIO_MAX_GARBAGE_Q)
        {
            System_printf ("Core %d: found %d descriptors in garbage queue %d; expected %d\n",
                           corenum, queueDepth, queueNum,
                           TOTAL_GARBAGE_DESCS / QMSS_QOS_SRIO_MAX_GARBAGE_Q);
            errorCount++;
        }
        /* Put the descriptors back in free q */
        queue_divert_and_check (srioGarbageRetQ[queueNum], freeQ);
    }

    /* Disable the cluster */
    if ((result = Qmss_disableQosSrioCluster (0)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to disable SRIO cluster : %d\n", corenum, result);
        errorCount++;
    }

fail:
    if (errorCount == 0)
        System_printf ("\nCore %d : SRIO context tests Passed\n");
}

Void run_test (Void)
{
    testSrioContext ();
}


