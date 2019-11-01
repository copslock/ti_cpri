/**
 *   @file  test_qos.c
 *
 *   @brief
 *      This is the QMSS unit test code for QoS.  This version only
 *      considers round robin functionality on cluster 7.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2013, Texas Instruments, Inc.
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

/* CSL RL includes */
#include <ti/csl/csl_chip.h>

/* OSAL includes */
#include <qmss_osal.h>

/************************ USER DEFINES ********************/
#define NUM_MONOLITHIC_DESC         4096
#define SIZE_MONOLITHIC_DESC        32
#define PROFILE_DESCS               400
#define QOS_TX_QUEUES               8

/* 3us when clock is 983 MHz (or QM's clock is 983/3 = 327.67 MHz) */
/* The test case will still operate if the clocks are different */
#define QOS_TIMER_CONFIG            491

/* No data is actually transmitted, just used to allow QoS to calculate bandwidth */
#define QOS_DATA_PACKET_SIZE        60

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
    uint32_t corenum;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Compute the global address. */
    return (addr + (0x10000000 + (corenum * 0x1000000)));
#else
    return addr;
#endif
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

Void testQos (Void)
{
    Qmss_Result             result;
    UInt32                  numAllocated, corenum;
    Qmss_QueueHnd           freeQ;
    Qmss_QueueHnd           baseQueue;
    int                     queueNum;
    Qmss_QueueHnd           fwQHnds[64];
    Qmss_QueueHnd           qosRROutQHnd;
    Qmss_QosClusterCfg      clusterCfg;
    Qmss_QosClusterCfgRR   *clusterCfgRR;
    Qmss_QosQueueCfg        qosQueueCfg;
    uint8_t                 isAllocated;
    int                     descID;
    int                     descNum;

    uint32_t startTime, endTime;

#ifdef _TMS320C6X
    /* Reset timer */
    TSCL = 0;

    /* Get the core number. */
    corenum = CSL_chipReadReg (CSL_CHIP_DNUM);
#else
    corenum = 0;
#endif
    System_printf ("**********Core %d TESTING QoS ************\n", corenum);

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

    /* Allocate block of queues to be used by firmware */
    baseQueue = Qmss_queueBlockOpen (fwQHnds, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, 64, 32);
    if (baseQueue < 0)
    {
        System_printf ("Core %d : Failed to open 64 contiguous queues\n");
        errorCount++;
    }

    /* Set the FW's base queue */
    if ((result = Qmss_setQosQueueBase (baseQueue)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Error Core %d : Setting QoS queue base address error code: %d \n", corenum, result);
        errorCount++;
    }

    /* Configure the queue thresholds as required by the FW */
    for (queueNum = 56; queueNum < 64; queueNum++)
    {
        Qmss_setQueueThreshold (fwQHnds[queueNum], 1, 1);
    }

    /* Open output queue */
    qosRROutQHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                                   QMSS_PARAM_NOT_SPECIFIED, &isAllocated);
    if (qosRROutQHnd < 0)
    {
        errorCount++;
        System_printf("out Queue open failed: %d\n", qosRROutQHnd);
    }

    /* Set up QoS's timer */
    if ((result = Qmss_configureQosTimer (QOS_TIMER_CONFIG) != QCMD_RETCODE_SUCCESS))
    {
        errorCount++;
        System_printf("Core %d : Failed to configure QoS timer: %d\n", corenum, result);
    }

    memset (&qosQueueCfg, 0, sizeof(qosQueueCfg));
    qosQueueCfg.egressQueNum = (uint16_t)qosRROutQHnd;
    qosQueueCfg.iterationCredit = 0; /* not used */
    qosQueueCfg.maxCredit = 0;
    qosQueueCfg.congestionThreshold = 0xffffffff;

    /* Configure applicable QoS queues */
    for (queueNum = 56; queueNum < 64; queueNum++)
    {
        if ((result = Qmss_configureQosQueue (queueNum, &qosQueueCfg) != QCMD_RETCODE_SUCCESS))
        {
            System_printf ("Core %d : Failed to configure qos queue (%d)\n", corenum, result);
            errorCount++;
        }
    }
    /* Configure the QoS cluster to use for the Round-Robin traffic shaping. */
    memset (&clusterCfg, 0, sizeof(clusterCfg));
    clusterCfgRR                       = &clusterCfg.u.cfgRR;
    clusterCfg.mode                    = Qmss_QosMode_RoundRobin;
    clusterCfgRR->maxGlobalCredit      = 5000;
    clusterCfgRR->qosQueHighCnt        = 4;  /* Number of high prio queues, must be set to 4. */
    clusterCfgRR->qosQueNumHigh[0]     = 56; /* Offset to high queue 0, must be set to 56. */
    clusterCfgRR->qosQueNumHigh[1]     = 57; /* Offset to high queue 1, must be set to 57. */
    clusterCfgRR->qosQueNumHigh[2]     = 58; /* Offset to high queue 2, must be set to 58. */
    clusterCfgRR->qosQueNumHigh[3]     = 59; /* Offset to high queue 3, must be set to 59. */
    clusterCfgRR->qosQueLowCnt         = 4;  /* Number of low prio queues, must be set to 4. */
    clusterCfgRR->qosQueNumLow[0]      = 60; /* Offset to low queue 0, must be set to 60. */
    clusterCfgRR->qosQueNumLow[1]      = 61; /* Offset to low queue 1, must be set to 61. */
    clusterCfgRR->qosQueNumLow[2]      = 62; /* Offset to low queue 2, must be set to 62. */
    clusterCfgRR->qosQueNumLow[3]      = 63; /* Offset to low queue 3, must be set to 63. */
    clusterCfgRR->sizeAdjust           = 24; /* Recommended by TI in "Ericsson QoS Transmit Discussion Addendum". */
    clusterCfgRR->egressQueCnt         = 1;
    clusterCfgRR->egressQueNum[0]      = Qmss_getQueueNumber (qosRROutQHnd);
    clusterCfgRR->iterationCredit      = 252;
    clusterCfgRR->maxEgressBacklog     = 1000000;
    clusterCfgRR->queueDisableMask     = 0x0;/* Enable all ingress queues from the start. */

    result = Qmss_configureQosCluster(7, &clusterCfg);
    if (QCMD_RETCODE_SUCCESS != result)
    {
        System_printf ("Core %d : Failed to configure QoS (%d)\n", corenum, result);
        errorCount++;
    }


    /* Place PROFILE_DESCS descriptors into each high priority input queue */
    descID = 0;
    for (descNum = 0; descNum < PROFILE_DESCS; descNum++)
    {
        for (queueNum = 0; queueNum < 4; queueNum++)
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
            Qmss_queuePush (fwQHnds[queueNum + 56], desc, QOS_DATA_PACKET_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
        }
    }

    /* Place PROFILE_DESCS descriptors into each low priority input queue */
    for (descNum = 0; descNum < PROFILE_DESCS; descNum++)
    {
        for (queueNum = 4; queueNum < 8; queueNum++)
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
            Qmss_queuePush (fwQHnds[queueNum + 56], desc, QOS_DATA_PACKET_SIZE, SIZE_MONOLITHIC_DESC, Qmss_Location_TAIL);
        }
    }

    /* Enable the cluster */
    if ((result = Qmss_enableQosCluster (7)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to enable QoS cluster : %d\n", corenum, result);
        errorCount++;
    }

    /* Start the clock */
#ifdef _TMS320C6X
    startTime = TSCL;
#else
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(startTime));
#endif
    /* Timestamp arrival of each descriptor */
    for (descNum = 0; descNum < PROFILE_DESCS * QOS_TX_QUEUES; )
    {
        uint32_t *desc;
        desc = (uint32_t *)QMSS_DESC_PTR(Qmss_queuePop (qosRROutQHnd));
        if (desc)
        {
#ifdef _TMS320C6X
            timestamps[descNum].timestamp = TSCL;
#else
            __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(timestamps[descNum].timestamp));
#endif
            timestamps[descNum].descPtr   = desc;
            descNum ++;

        }
    }

#ifdef _TMS320C6X
    endTime = TSCL;
#else
    __asm__ __volatile__ ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(endTime));
#endif

    System_printf ("Core %d: moved %d descriptors on %d queues (%d total) "
                   "in %d cycles (%d cycles per descriptor)\n",
                   corenum,
                   PROFILE_DESCS,
                   QOS_TX_QUEUES,
                   PROFILE_DESCS * QOS_TX_QUEUES,
                   endTime - startTime,
                   (endTime - startTime) / (PROFILE_DESCS * QOS_TX_QUEUES));

    /* Disable the cluster */
    if ((result = Qmss_disableQosCluster (7)) != QCMD_RETCODE_SUCCESS)
    {
        System_printf ("Core %d : failed to disable QoS cluster : %d\n", corenum, result);
        errorCount++;
    }

    /* Verify the results and return descriptors to free q */
    descID = 0;
    for (descNum = 0; descNum < PROFILE_DESCS * QOS_TX_QUEUES; descNum++)
    {
        uint32_t *desc = timestamps[descNum].descPtr;
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

    if (errorCount == 0)
        System_printf ("\nCore %d : QoS tests Passed\n", corenum);
    else
        System_printf ("\nCore %d : ***********FAIL***********\n", corenum);
}

Void run_test (Void)
{
    testQos ();
}

