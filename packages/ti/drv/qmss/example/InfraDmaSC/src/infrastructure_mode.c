/**
 *   @file  infrastructure_mode.c
 *
 *   @brief
 *      This is the QMSS infrastructure mode example code. Runs both in polling and accumulator mode.
 *      (except on Linux user mode, only in polling mode)
 *      Uses both host and monolithic descriptors for data transfer.
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
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <xdc/cfg/global.h>
#ifdef __ARM_ARCH_7A__
#include <ti/sysbios/family/arm/a15/Mmu.h>
#endif
#else
#include "fw_test.h"
#include "fw_mem_allocator.h"
#include "linuxutil.h"
#endif
#include "qmss_test.h"
#include <string.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#ifndef __LINUX_USER_SPACE
#include <ti/drv/qmss/qmss_firmware.h>
#endif

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
/* Example configuration */
#include "infrastructure_mode_cfg.h"

#ifdef NSS_LITE
#include <ti/csl/cslr_device.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#endif

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#ifdef __ARM_ARCH_7A__
#include <ti/csl/cslr_msmc.h>
#endif

/************************ GLOBAL VARIABLES ********************/
#ifdef _TMS320C6X
#pragma DATA_ALIGN (linkingRAM0, 16)
uint64_t            linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC];
/* Descriptor pool [Size of descriptor * Number of descriptors] */
#pragma DATA_ALIGN (hostDesc, 16)
uint8_t                 hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC];
#pragma DATA_ALIGN (monolithicDesc, 16)
uint8_t                 monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#pragma DATA_ALIGN (txDataBuff, 16)
uint8_t                 txDataBuff[SIZE_DATA_BUFFER];
#pragma DATA_ALIGN (rxDataBuff, 16)
uint8_t                 rxDataBuff[SIZE_DATA_BUFFER * NUM_PACKETS];
#else
uint64_t            linkingRAM0[NUM_HOST_DESC + NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
/* Descriptor pool [Size of descriptor * Number of descriptors] */
uint8_t                 hostDesc[SIZE_HOST_DESC * NUM_HOST_DESC] __attribute__ ((aligned (16)));
uint8_t                 monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC] __attribute__ ((aligned (16)));
uint8_t                 txDataBuff[SIZE_DATA_BUFFER] __attribute__ ((aligned (16)));
uint8_t                 rxDataBuff[SIZE_DATA_BUFFER * NUM_PACKETS] __attribute__ ((aligned (16)));
#endif
/* CPDMA configuration */
Cppi_CpDmaInitCfg       cpdmaCfg;
#endif

/* Tx channel configuration */
Cppi_TxChInitCfg        txChCfg;
/* Rx channel configuration */
Cppi_RxChInitCfg        rxChCfg;
/* Rx flow configuration */
Cppi_RxFlowCfg          rxFlowCfg;
/* Memory region configuration information */
Qmss_MemRegInfo         monoMemInfo;
/* Memory region configuration information */
Qmss_MemRegInfo         hostMemInfo;

/* Error counter */
uint32_t                errorCount = 0;

/* Memory region numbers (return value of Qmss_insertMemoryRegion) */
Qmss_Result             hostReg, monoReg;
#ifndef __LINUX_USER_SPACE
/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Handle to CPPI heap */
IHeap_Handle            cppiHeap;
#endif
/* Core number */
uint32_t                coreNum = 0; /* Not used on linux */
/* Accumulator configuration */
#ifdef CONFIG_ACC
Qmss_AccCmdCfg          cfg;
/* List address for accumulator - twice the number of entries for Ping and Pong page */
#ifdef _TMS320C6X
#pragma DATA_ALIGN (hiPrioList, 16)
Uint32                  hiPrioList[(NUM_PACKETS + 1) * 2];
#else
Uint32                  hiPrioList[(NUM_PACKETS + 1) * 2] __attribute__ ((aligned (16)));
#endif
#endif

/* Thread arguments for receive threads */
typedef struct
{
    Qmss_QueueHnd rxQueHnd;  /* queue handle to pull packets */
    int           waitFd;    /* fd to select/read to wait for int */
    int           errors;    /* Errors detected by task */
} rxTaskArgs_t;

#ifndef __LINUX_USER_SPACE
/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/*************************** FUNCTIONS ************************/
/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to the CPPI memory heap.
 *      If the application is run on a multiple cores then place the CPPI global
 *      variables in shared memory.
 *
 *  @retval
 *      None
 */
#ifdef _TMS320C6X
static Void cppiHeapInit ()
{
    cppiHeap = HeapMem_Handle_upCast (cppiLocalHeap);
}
#endif
#endif

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

#ifdef CONFIG_ACC
Void testQueueReclaim (Qmss_QueueHnd freeHostQueHnd)
{
    Qmss_Result             result;
    Qmss_QueueHnd           reclaimQueHnd, returnQueHnd;
    Cppi_Desc               *hostDescPtr, *rxPkt;
    Qmss_Queue              queInfo;
    uint8_t                 isAllocated;

    /* Opens reclamation queue. */
    if ((reclaimQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening reclamation Queue Number\n", coreNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Reclaimation Queue Number : %d opened\n", coreNum, reclaimQueHnd);
    }

    /* Opens return queue. */
    if ((returnQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening return Queue Number\n", coreNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Return Queue Number : %d opened\n", coreNum, returnQueHnd);
    }


    if ((result = Qmss_programReclaimQueue (Qmss_PdspId_PDSP1, reclaimQueHnd)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Configuring reclamation queue : %d error code : %d\n",
                        coreNum, reclaimQueHnd, result);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Configured reclamation queue : %d\n",
                        coreNum, reclaimQueHnd);
    }

    if ((hostDescPtr = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd)) == NULL)
    {
        System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n", coreNum, freeHostQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Descriptor address 0x%p\n", coreNum, hostDescPtr);
    }

    queInfo = Qmss_getQueueNumber (returnQueHnd);

    Cppi_setReturnQueue (Cppi_DescType_HOST, hostDescPtr, queInfo);

    /* Push descriptor to Tx free queue */
    Qmss_queuePushDesc (reclaimQueHnd, (uint32_t *) hostDescPtr);
    while (Qmss_getQueueEntryCount (returnQueHnd) == 0);

    while ((rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (returnQueHnd))) != NULL)
    {
        System_printf ("Core %d : Reclaimed descriptor address 0x%p\n", coreNum, rxPkt);
    }

    if ((result = Qmss_programReclaimQueue (Qmss_PdspId_PDSP1, 0)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Disabling reclamation queue : %d error code : %d\n",
                        coreNum, reclaimQueHnd, result);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Disabled reclamation queue : %d\n",
                        coreNum, reclaimQueHnd);
    }
    if ((hostDescPtr = (Cppi_Desc *) Qmss_queuePop (freeHostQueHnd)) == NULL)
    {
        System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n", coreNum, freeHostQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Descriptor address 0x%p\n", coreNum, hostDescPtr);
    }

    Cppi_setReturnQueue (Cppi_DescType_HOST, hostDescPtr, queInfo);

    /* Push descriptor to Tx free queue */
    Qmss_queuePushDesc (reclaimQueHnd, (uint32_t *) hostDescPtr);

    /* Descriptor should not be reclaimed */
    if ((Qmss_getQueueEntryCount (returnQueHnd) != 0))
    {
        System_printf ("Error Core %d : Descriptor count on return queue %d is %d\n",
                        coreNum, returnQueHnd, Qmss_getQueueEntryCount (returnQueHnd));
        errorCount++;
        return;
    }

    /* Close the queues */
    if ((result = Qmss_queueClose (reclaimQueHnd)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Failed to close reclaimation queue %d (%d)\n",
                        coreNum, reclaimQueHnd, result);
        errorCount++;
        return;
    }
    if ((result = Qmss_queueClose (returnQueHnd)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Failed to close return queue %d (%d)\n",
                        coreNum, returnQueHnd, result);
        errorCount++;
        return;
    }
    System_printf ("\nCore %d : Queue reclamation feature Passed\n", coreNum);
    return;
}
#endif

/**
 *  @b Description
 *  @n
 *      Function can be run as thread.  Blocks for interrupt or
 *      polls for packets depending whether running on Linux or SYS/BIOS.
 *
 *  @param[in]  rxTaskArgs
 *      Pointer to rxTaskArgs_t that can be passed through threads library
 *
 *  @retval
 *      NULL (for threads library)
 */
void *hostModeRxTask (void *rxTaskArgs)
{
    rxTaskArgs_t   *args             = (rxTaskArgs_t *)rxTaskArgs;
    Qmss_QueueHnd   rxQueHnd         = args->rxQueHnd;
    int             packetsLeft      = NUM_PACKETS;
    int             packetsToProcess = 0;
    int             packet;
    Cppi_Desc      *rxPkt;
    uint32_t        i, length, destLen;
    uint8_t        *dataBuffPtr;
    Qmss_Queue      queInfo;

    while (packetsLeft)
    {
#ifndef __LINUX_USER_SPACE
        while ((packetsToProcess = Qmss_getQueueEntryCount (rxQueHnd)) == 0);
#else
        waitForInterrupt (args->waitFd);
        /* Process all the pending packets without needing interrupt overhead for each */
        packetsToProcess = Qmss_getQueueEntryCount (rxQueHnd);
#endif

        for (packet = 0; packet < packetsToProcess; packet++)
        {
            /* Get the rx packet */
            if ((rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (rxQueHnd))) == NULL)
            {
                System_printf ("ERROR: Core %d : expected packet but none found!\n", coreNum);
                args->errors++;
                continue;
            }
            packetsLeft --;
            length = Cppi_getPacketLen (Cppi_DescType_HOST, rxPkt);

            System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

            /* Get data buffer */
            Cppi_getData (Cppi_DescType_HOST, rxPkt, &dataBuffPtr, &destLen);

            /* Compare */
            for (i = 0; i < destLen; i++)
            {
                if (txDataBuff[i] != dataBuffPtr[i])
                {
                    args->errors++;
                    System_printf ("Error Core %d : In data buffer Tx: %02X - Rx: %02X \n", coreNum, txDataBuff[i], dataBuffPtr[i]);
                }
            }

            /* Recycle the descriptors */
            queInfo = Cppi_getReturnQueue (Cppi_DescType_HOST, rxPkt);

            /* Push descriptor back to Rx free queue */
            Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
        }
    }

    return NULL;
}


/**
 *  @b Description
 *  @n
 *      Data transfer using host descriptors
 *
 *  @param[in]  cppiHnd
 *      Handle to access CPPI queue manager CPDMA
 *
 *  @retval
 *      None
 */
void hostModeTransfer (Cppi_Handle cppiHnd)
{
    Qmss_Result             result;
#ifdef CONFIG_ACC
    uint32_t                length, destLen;
#endif
    uint32_t                numAllocated, i;
    uint8_t                 isAllocated;
    Cppi_ChHnd              rxChHnd, txChHnd;
    Qmss_QueueHnd           txQueHnd, rxQueHnd, freeQueHnd, txCmplQueHnd, txFreeQueHnd;
    Cppi_DescCfg            descCfg;
    Cppi_Desc               *hostDescPtr;
    Cppi_FlowHnd            rxFlowHnd;
    Qmss_Queue              queInfo;
    Cppi_DescTag            tag={0};
    rxTaskArgs_t            rxTaskArgs;

#ifdef __LINUX_USER_SPACE
    void                   *taskId;
#endif

#ifdef CONFIG_ACC
    uint32_t                index, base, count, packetCount;
    Bool                    usePing = TRUE;
    volatile uint32_t       temp;
    Cppi_Desc              *rxPkt;
    uint8_t                *dataBuffPtr;
#endif

    System_printf ("\n*************** Data transfer host mode *****************\n\n");

    /* Setup the descriptors for transmit free queue */
    /* Memory region obtained is zero since there is only Qmss_insertMemoryRegion() call.
     * else find the memory region using Qmss_getMemoryRegionCfg() */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)hostReg;
    descCfg.descNum = NUM_PACKETS;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifdef CONFIG_ACC
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_HOST;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;

    /* Descriptor should be recycled back to Queue Number 1000 */
    descCfg.returnQueue.qMgr = CPPI_COMPLETION_QUE_MGR;
    descCfg.returnQueue.qNum = CPPI_COMPLETION_QUE_NUM;

    /* Initialize the descriptors and push to free Queue */
    if ((txFreeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing Tx descriptor error code: %d \n", coreNum, txFreeQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Number of Tx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
    }

    /* Setup the descriptors for receive free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)hostReg;
    descCfg.descNum = NUM_HOST_DESC - NUM_PACKETS;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifdef CONFIG_ACC
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_HOST;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;

    /* Descriptor should be recycled back to queue allocated above */
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;

    /* Initialize the descriptors and push to free Queue */
    if ((freeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing Rx descriptor error code: %d \n", coreNum, freeQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Number of Rx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
    }

    /* Setup Rx descriptors with receive buffers */
    for (i = 0; i < numAllocated; i++)
    {
        Ptr dataPtr = rxDataBuff + i * SIZE_DATA_BUFFER;

        /* Get a descriptor */
        if ((hostDescPtr = (Cppi_Desc *) Qmss_queuePop (freeQueHnd)) == NULL)
        {
            System_printf ("Error Core %d : Getting descriptor from Queue Number: %d\n", coreNum, freeQueHnd);
            errorCount++;
            return;
        }

        /* Add data buffer */
        Cppi_setData (Cppi_DescType_HOST, hostDescPtr, (uint8_t *) l2_global_address ((uint32_t) dataPtr), SIZE_DATA_BUFFER);

        /* Save original buffer information */
        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, hostDescPtr, (uint8_t *) l2_global_address ((uint32_t) dataPtr), SIZE_DATA_BUFFER);

        /* Push descriptor to Rx free queue */
        Qmss_queuePushDesc (freeQueHnd, (uint32_t *) hostDescPtr);
    }

    /* Set up Rx Channel parameters */
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Rx Channel */
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
    }


    /* Set up Tx Channel parameters */
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    txChCfg.priority = 0;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Tx Channel */
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
    }

    /* Opens transmit queue. This is the infrastructure queue */
    if ((txQueHnd = Qmss_queueOpen (INFRASTRUCTURE_MODE_QUEUE_TYPE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening Transmit Queue: %d\n", coreNum, txQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Transmit Queue Number : %d opened\n", coreNum, txQueHnd);
    }

#ifdef __LINUX_USER_SPACE
    /* Opens receive queue */
    if ((rxTaskArgs.waitFd = acquireUioRxQueue ("qpend", &rxQueHnd)) < 0)
    {
        System_printf ("Error Core %d : cant find qpend queue in uio/proc (%d) \n", coreNum, rxTaskArgs.waitFd);
        errorCount++;
        return;
    }
#else
    /* Opens receive queue in first QM */
    #ifndef NSS_LITE
    if ((rxQueHnd = Qmss_queueOpenInGroup (0, Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #else
    if ((rxQueHnd = Qmss_queueOpenInGroup (0, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #endif
    {
        System_printf ("Error Core %d : Opening Receive Queue: %d \n", coreNum, rxQueHnd);
        errorCount++;
        return;
    }
    else
#endif
    {
        System_printf ("Core %d : Receive Queue Number : %d opened\n", coreNum, rxQueHnd);
    }

    /* Opens transmit completion queue. */
    if ((txCmplQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QUE_NUM, &isAllocated)) < 0)
    {
        System_printf ("Error Core %d : Opening Tx Completion Queue: %d \n", coreNum, txCmplQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Tx Completion Queue Number : %d opened\n", coreNum, txCmplQueHnd);
    }

    System_printf ("Core %d : Free Queue Number : %d opened\n", coreNum, freeQueHnd);
    System_printf ("Core %d : Transmit Free Queue Number : %d opened\n", coreNum, txFreeQueHnd);

#ifdef CONFIG_ACC

    if ((result = Qmss_configureAccTimer (Qmss_PdspId_PDSP1, 3500)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Changing accumulator %d %d tick to 20us error code : %d\n",
                       coreNum, cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Changed accumulator tick to 20us\n", coreNum);
    }

    /* program the high priority accumulator */
    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));
    cfg.channel = 0;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList); /* Should be global if reading on another core */
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (rxQueHnd);
    cfg.maxPageEntries = ACC_ENTRY_SIZE;
    cfg.timerLoadCount = 0;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_ENTRY_COUNT;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;

    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : high priority accumulator programmed for channel : %d queue : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex);
    }
#endif

    /* Set transmit queue threshold to high and when there is atleast one packet */
    /* Setting threshold on transmit queue is not required anymore. tx pending queue is not hooked to threshold.
     * Qmss_setQueueThreshold (txQueHnd, 1, 1);
     */

    /* Setup Rx flow parameters */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Don't specify flow number and let CPPI allocate the next available one */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxQueHnd);
    rxFlowCfg.rx_dest_qnum = queInfo.qNum;
    rxFlowCfg.rx_dest_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_desc_type = Cppi_DescType_HOST;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (freeQueHnd);
    rxFlowCfg.rx_fdq0_sz0_qnum = queInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr = queInfo.qMgr;

    /* Configure Rx flow */
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId(rxFlowHnd));
    }

    /* Enable transmit channel */
    if (Cppi_channelEnable (txChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Tx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (txChHnd));
    }

    /* Enable receive channel */
    if (Cppi_channelEnable (rxChHnd) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Rx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (rxChHnd));
    }

    rxTaskArgs.rxQueHnd = rxQueHnd;
    rxTaskArgs.errors = 0;
#ifdef __LINUX_USER_SPACE
    /* Make a receive task */
    if ( (taskId = makeReceiveTask (hostModeRxTask, &rxTaskArgs)) )
    {
        printf ("Created host mode RX task: %p\n", taskId);
    }
    else
    {
        System_printf ("Error Core %d : Failed to create host mode receive task\n", coreNum);
        errorCount++;
    }
#endif

    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++)
        txDataBuff[i] = i;

    System_printf ("\n--------------------Transmitting packets----------------------\n");

    /* Send out 8 packets */
    tag.srcTagLo = Cppi_getFlowId(rxFlowHnd);
    for (i = 0; i < NUM_PACKETS; i++)
    {
        /* Get a free descriptor */
        if ((hostDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n", coreNum, txFreeQueHnd);
            errorCount++;
            return;
        }

        /* Add data buffer */
        Cppi_setData (Cppi_DescType_HOST, hostDescPtr, (uint8_t *) l2_global_address ((uint32_t) txDataBuff), SIZE_DATA_BUFFER);

        Cppi_setOriginalBufInfo (Cppi_DescType_HOST, hostDescPtr, (uint8_t *) l2_global_address ((uint32_t) txDataBuff), SIZE_DATA_BUFFER);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_HOST, hostDescPtr, SIZE_DATA_BUFFER);

        /* Specify the CPPI flow for Rx channel */
        Cppi_setTag(Cppi_DescType_HOST, hostDescPtr, &tag);

        System_printf ("Core %d : Transmitting descriptor 0x%p\n", coreNum, hostDescPtr);

        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (uint32_t *) hostDescPtr, SIZE_HOST_DESC);
    }

    System_printf ("\n-------------------------Queue status-------------------------\n");
    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    System_printf ("\n--------------------Receiving packets-------------------------\n");

#ifndef CONFIG_ACC
#ifdef __LINUX_USER_SPACE
    /* wait for the receive task to finish */
    waitReceiveTask (taskId);
#else
    /* Run the receive task inline */
    hostModeRxTask (&rxTaskArgs);
#endif
#else
    {
        /* Burn some time for the accumulators to run. */
#if 1
        temp = 0;
        for (i = 0; i < 50000; i++)
        {
          temp = i;
        }
        temp = 1;
#endif
    }

    /* Get the Rx packet */
    /* 8 packets were sent out. Check how many where received. Count should be the first entry since acc
     * is programmed in entry count. Start with the ping side */
    packetCount = 0;
    while (1)
    {
        if (usePing == TRUE)
        {
            /* Use Ping side */
            base = 0;
            usePing = FALSE;
            System_printf ("Core %d : Processing ping page\n", coreNum);
        }
        else
        {
            /* Switch to pong side */
            base = ACC_ENTRY_SIZE;
            usePing = TRUE;
            System_printf ("Core %d : Processing pong page\n", coreNum);
        }

        /* Using Entry count mode */
        count = hiPrioList[base];
        hiPrioList[base] = 0;
        for (index = base + 1; index <= base + count; index++)
        {
            rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (hiPrioList[index]);
            hiPrioList[index] = 0;

            /* Get packet length */
            length = Cppi_getPacketLen (Cppi_DescType_HOST, rxPkt);

            System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

            /* Get data buffer */
            Cppi_getData (Cppi_DescType_HOST, rxPkt, &dataBuffPtr, &destLen);

            /* Compare */
            for (i = 0; i < destLen; i++)
            {
                if (txDataBuff[i] != dataBuffPtr[i])
                {
                    errorCount++;
                    System_printf ("Error Core %d : In data buffer Tx: %02X - Rx: %02X \n", coreNum, txDataBuff[i], dataBuffPtr[i]);
                }
            }

            /* Recycle the descriptors */
            queInfo = Cppi_getReturnQueue (Cppi_DescType_HOST, rxPkt);

            /* Push descriptor back to Rx free queue */
            Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
            packetCount++;
        }
        Qmss_ackInterrupt (cfg.channel, 1);
        Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, cfg.channel);

        if (packetCount == NUM_PACKETS)
            break;
    }
#endif

    System_printf ("\n--------------------Deinitializing----------------------------\n");

    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

#ifdef CONFIG_ACC
    /* Disable accumulator */
    if ((result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel)) != QMSS_ACC_SOK)
    {
        System_printf ("Error Core %d : Disabling high priority accumulator for channel : %d queue : %d error code: %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : high priority accumulator disabled for channel : %d queue : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex);
    }
#endif

#ifdef CONFIG_ACC
    /* Test queue reclamation feature */
    testQueueReclaim (txCmplQueHnd);
#endif

    /* Flush the queues */
    Qmss_queueEmpty (txFreeQueHnd);
    Qmss_queueEmpty (freeQueHnd);
    Qmss_queueEmpty (txCmplQueHnd);
    Qmss_queueEmpty (rxQueHnd);
    Qmss_queueEmpty (txQueHnd);

    /* Close Tx channel */
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Tx channel error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Tx Channel closed successfully. Ref count : %d\n", coreNum, result);
    }

    /* Close Rx channel */
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx channel error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Rx Channel closed successfully. Ref count : %d\n", coreNum, result);
    }

    /* Close Rx flow */
    if ((result = Cppi_closeRxFlow (rxFlowHnd)) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Rx flow error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Rx flow closed successfully. Ref count : %d\n", coreNum, result);
    }

#ifndef __LINUX_USER_SPACE
    /* Close the queues */
    if ((result = Qmss_queueClose (rxQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Rx queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Rx queue closed successfully. Ref count : %d\n", coreNum, result);
    }
#else
    if (releaseUioRxQueue (rxTaskArgs.waitFd, rxQueHnd) < 0)
    {
        System_printf ("Core %d : failed to release qpend fd/hnd\n", coreNum);
        errorCount++;
    }
#endif

    if ((result = Qmss_queueClose (txQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing tx queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Tx queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (freeQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing free queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Free queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (txCmplQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing transmit completion queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Transmit completion queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (txFreeQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing transmit freequeue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Transmit free queue closed successfully. Ref count : %d\n", coreNum, result);
    }
}

/**
 *  @b Description
 *  @n
 *      Function can be run as thread.  Blocks for interrupt or
 *      polls for packets depending whether running on Linux or SYS/BIOS.
 *
 *  @param[in]  rxTaskArgs
 *      Pointer to rxTaskArgs_t that can be passed through threads library
 *
 *  @retval
 *      NULL (for threads library)
 */
void *monoModeRxTask (void *rxTaskArgs)
{
    rxTaskArgs_t   *args             = (rxTaskArgs_t *)rxTaskArgs;
    Qmss_QueueHnd   rxQueHnd         = args->rxQueHnd;
    int             packetsLeft      = NUM_PACKETS;
    int             packetsToProcess = 0;
    int             packet;
    Cppi_Desc      *rxPkt;
    uint32_t        i, length, destLen;
    uint8_t        *dataBuffPtr;
    Qmss_Queue      queInfo;

    while (packetsLeft)
    {
#ifndef __LINUX_USER_SPACE
        while ((packetsToProcess = Qmss_getQueueEntryCount (rxQueHnd)) == 0);
#else
        waitForInterrupt (args->waitFd);
        /* Process all the pending packets without needing interrupt overhead for each */
        packetsToProcess = Qmss_getQueueEntryCount (rxQueHnd);
#endif

        for (packet = 0; packet < packetsToProcess; packet++)
        {
            /* Get the rx packet */
            if ((rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (Qmss_queuePop (rxQueHnd))) == NULL)
            {
                System_printf ("ERROR: Core %d : expected packet but none found!\n", coreNum);
                args->errors++;
                continue;
            }
            packetsLeft --;
            length = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, rxPkt);

            System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

            /* Get data buffer */
            Cppi_getData (Cppi_DescType_MONOLITHIC, rxPkt, &dataBuffPtr, &destLen);

            /* Compare */
            for (i = 0; i < destLen; i++)
            {
                if (txDataBuff[i] != dataBuffPtr[i])
                {
                    errorCount++;
                    System_printf ("Error Core %d : In data buffer Tx: %02X - Rx: %02X \n", coreNum, txDataBuff[i], dataBuffPtr[i]);
                }
            }

            /* Recycle the descriptors */
            queInfo = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, rxPkt);

            /* Push descriptor back to free queue */
            Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
        }
    }

    return NULL;
}

/**
 *  @b Description
 *  @n
 *      Data transfer using monolithic descriptors
 *
 *  @param[in]  cppiHnd
 *      Handle to access CPPI queue manager CPDMA
 *
 *  @retval
 *      None
 */

void monoModeTransfer (Cppi_Handle cppiHnd)
{
    Qmss_Result             result;
#ifdef CONFIG_ACC
    uint32_t                length, destLen;
#endif
    uint32_t                numAllocated, i;
    uint8_t                 isAllocated;
    Cppi_ChHnd              rxChHnd, txChHnd;
    Qmss_QueueHnd           txQueHnd, rxQueHnd, freeQueHnd, txCmplQueHnd, txFreeQueHnd;
    Cppi_DescCfg            descCfg;
    Cppi_Desc               *monoDescPtr;
    Cppi_FlowHnd            rxFlowHnd;
    Qmss_Queue              queInfo;
    Cppi_DescTag            tag={0};
    rxTaskArgs_t            rxTaskArgs;

#ifdef __LINUX_USER_SPACE
    void                   *taskId;
#endif

#ifdef CONFIG_ACC
    uint32_t                index, base, packetCount;
    Bool                    usePing = TRUE;
    volatile uint32_t       temp;
    Cppi_Desc              *rxPkt;
    uint8_t                *dataBuffPtr;
#endif

    System_printf ("\n***************** Data transfer monolithic mode ***************\n\n");

    /* Setup the descriptors for transmit free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)monoReg;
    descCfg.descNum = NUM_PACKETS;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifdef CONFIG_ACC
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;

    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;

    /* Descriptor should be recycled back to Queue Number 1000 */
    descCfg.returnQueue.qMgr = CPPI_COMPLETION_QUE_MGR;
    descCfg.returnQueue.qNum = CPPI_COMPLETION_QUE_NUM;


    /* Initialize the descriptors and push to free Queue */
    if ((txFreeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        System_printf ("Error Core %d : Initializing Tx descriptor error code: %d \n", coreNum, txFreeQueHnd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("Core %d : Number of Tx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
    }

    /* Setup the descriptors for receive free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)monoReg;
    descCfg.descNum = NUM_MONOLITHIC_DESC - NUM_PACKETS;
    descCfg.destQueueNum = QMSS_PARAM_NOT_SPECIFIED;
    #ifdef CONFIG_ACC
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    #else
    descCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;
    #endif
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;

    /* Descriptor should be recycled back to queue allocated above */
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;

    /* Initialize the descriptors and push to free Queue */
    if ((freeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Rx descriptor error code: %d \n", coreNum, freeQueHnd);
        return;
    }
    else
    {
        System_printf ("Core %d : Number of Rx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
    }

    /* Set up Rx Channel parameters */
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Rx Channel */
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Rx channel : %d\n", coreNum, rxChCfg.channelNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
    }


    /* Set up Tx Channel parameters */
    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    txChCfg.priority = 0;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Tx Channel */
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx channel : %d\n", coreNum, txChCfg.channelNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
    }

    /* Opens transmit queue. This is the infrastructure queue */
    if ((txQueHnd = Qmss_queueOpen (INFRASTRUCTURE_MODE_QUEUE_TYPE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Transmit Queue: %d\n", coreNum, txQueHnd);
        return;
    }
    else
    {
        System_printf ("Core %d : Transmit Queue Number : %d opened\n", coreNum, txQueHnd);
    }

#ifdef __LINUX_USER_SPACE
    /* Opens receive queue */
    if ((rxTaskArgs.waitFd = acquireUioRxQueue ("qpend", &rxQueHnd)) < 0)
    {
        System_printf ("Error Core %d : cant find qpend queue in uio/proc (%d) \n", coreNum, rxTaskArgs.waitFd);
        errorCount++;
        return;
    }
#else
    /* Opens receive queue in second QM (if present, else first QM if only 1).
     * This will fail if second QM exists and has no high priority queues
     */
    #ifndef NSS_LITE 
    if ((rxQueHnd = Qmss_queueOpenInGroup (qmssLObj[0].p.maxQueMgrGroups - 1, Qmss_QueueType_HIGH_PRIORITY_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #else
    if ((rxQueHnd = Qmss_queueOpenInGroup (qmssLObj[0].p.maxQueMgrGroups - 1, Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_PARAM_NOT_SPECIFIED, &isAllocated)) < 0)
    #endif
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Receive Queue: %d\n", coreNum, rxQueHnd);
        return;
    }
#endif
    else
    {
        System_printf ("Core %d : Receive Queue Number : %d opened\n", coreNum, rxQueHnd);
    }

    /* Opens transmit completion queue. */
    if ((txCmplQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QUE_NUM, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx Completion Queue: %d\n", coreNum, txCmplQueHnd);
        return;
    }
    else
    {
        System_printf ("Core %d : Tx Completion Queue Number : %d opened\n", coreNum, txCmplQueHnd);
    }

    System_printf ("Core %d : Free Queue Number : %d opened\n", coreNum, freeQueHnd);
    System_printf ("Core %d : Transmit Free Queue Number : %d opened\n", coreNum, txFreeQueHnd);

#ifdef CONFIG_ACC
    /* program the high priority accumulator */
    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));
    cfg.channel = 0;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList); /* Should be global if reading on another core */
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (rxQueHnd);
    cfg.maxPageEntries = ACC_ENTRY_SIZE;
    cfg.timerLoadCount = 0;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_NULL_TERMINATE;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;

    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        return;
    }
    else
    {
        System_printf ("Core %d : high priority accumulator programmed for channel : %d queue : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex);
    }
#endif

    /* Set transmit queue threshold to high and when there is atleast one packet */
    /* Setting threshold on transmit queue is not required anymore. tx pending queue is not hooked to threshold.
     * Qmss_setQueueThreshold (txQueHnd, 1, 1);
     */

    /* Setup Rx flow parameters */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Don't specify flow number and let CPPI allocate the next available one */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxQueHnd);
    rxFlowCfg.rx_dest_qnum = queInfo.qNum;
    rxFlowCfg.rx_dest_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_sop_offset = MONOLITHIC_DESC_DATA_OFFSET;
    rxFlowCfg.rx_desc_type = Cppi_DescType_MONOLITHIC;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (freeQueHnd);
    rxFlowCfg.rx_fdq0_sz0_qnum = queInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr = queInfo.qMgr;

    /* Configure Rx flow */
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Opened Rx flow : %d\n", coreNum, Cppi_getFlowId(rxFlowHnd));
    }

    /* Enable transmit channel */
    if (Cppi_channelEnable (txChHnd) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
    }
    else
    {
        System_printf ("Core %d : Tx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (txChHnd));
    }

    /* Enable receive channel */
    if (Cppi_channelEnable (rxChHnd) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
    }
    else
    {
        System_printf ("Core %d : Rx channel : %d enabled \n", coreNum, Cppi_getChannelNumber (rxChHnd));
    }

    rxTaskArgs.rxQueHnd = rxQueHnd;
    rxTaskArgs.errors = 0;
#ifdef __LINUX_USER_SPACE
    /* Make a receive task */
    if ( (taskId = makeReceiveTask (monoModeRxTask, &rxTaskArgs)) )
    {
        printf ("Created mono mode RX task: %p\n", taskId);
    }
    else
    {
        System_printf ("Error Core %d : Failed to create mono mode receive task\n", coreNum);
        errorCount++;
    }
#endif

    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++)
        txDataBuff[i] = i;

    System_printf ("\n--------------------Transmitting packets----------------------\n");
    /* Send out 8 packets */
    tag.srcTagLo = Cppi_getFlowId(rxFlowHnd);
    for (i = 0; i < NUM_PACKETS; i++)
    {
        /* Get a free descriptor */
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            errorCount++;
            System_printf ("Error Core %d : Getting descriptor from Queue Number: %d\n", coreNum, txFreeQueHnd);
            return;
        }

        /* Add data buffer */
        Cppi_setData (Cppi_DescType_MONOLITHIC, monoDescPtr, txDataBuff, SIZE_DATA_BUFFER);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_MONOLITHIC, monoDescPtr, SIZE_DATA_BUFFER);

        /* Specify the CPPI flow for Rx channel */
        Cppi_setTag(Cppi_DescType_MONOLITHIC, monoDescPtr, &tag);

        System_printf ("Core %d : Transmitting descriptor 0x%p\n", coreNum, monoDescPtr);

        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (uint32_t *) monoDescPtr, 16);
    }

    System_printf ("\n-------------------------Queue status-------------------------\n");
    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    System_printf ("\n--------------------Receiving packets-------------------------\n");

#ifndef CONFIG_ACC
#ifdef __LINUX_USER_SPACE
    /* wait for the receive task to finish */
    waitReceiveTask (taskId);
#else
    /* Run the receive task inline */
    monoModeRxTask (&rxTaskArgs);
#endif
#else
    {
        /* Burn some time for the accumulators to run. */
#if 1
        temp = 0;
        for (i = 0; i < 50000; i++)
        {
          temp = i;
        }
        temp = 1;
#endif
    }

    /* Get the Rx packet */
    /* 8 packets were sent out. The list is NULL terminated. Start with the ping side */

    packetCount = 0;
    while (1)
    {
        if (usePing == TRUE)
        {
            /* Use ping side */
            base = 0;
            usePing = FALSE;
            System_printf ("Core %d : Processing ping page\n", coreNum);
        }
        else
        {
            /* Switch to pong side */
            base = ACC_ENTRY_SIZE;
            usePing = TRUE;
            System_printf ("Core %d : Processing pong page\n", coreNum);
        }

        /* Using Null terminate mode */
        index = base;
        while (hiPrioList[index] != 0)
        {
            rxPkt = (Cppi_Desc *) QMSS_DESC_PTR (hiPrioList[index]);
            hiPrioList[index] = 0;

            /* Get packet length */
            length = Cppi_getPacketLen (Cppi_DescType_MONOLITHIC, rxPkt);

            System_printf ("Core %d : Received descriptor 0x%p of length : %d\n", coreNum, rxPkt, length);

            /* Get data buffer */
            Cppi_getData (Cppi_DescType_MONOLITHIC, rxPkt, &dataBuffPtr, &destLen);

            /* Compare */
            for (i = 0; i < destLen; i++)
            {
                if (txDataBuff[i] != dataBuffPtr[i])
                {
                    errorCount++;
                    System_printf ("Error Core %d : In data buffer Tx: %02X - Rx: %02X \n", coreNum, txDataBuff[i], dataBuffPtr[i]);
                }
            }

            /* Recycle the descriptors */
            queInfo = Cppi_getReturnQueue (Cppi_DescType_MONOLITHIC, rxPkt);

            /* Push descriptor back to free queue */
            Qmss_queuePushDesc (Qmss_getQueueHandle(queInfo), (uint32_t *) rxPkt);
            index++;
            packetCount++;
        }
        Qmss_ackInterrupt (cfg.channel, 1);
        Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, cfg.channel);

        if (packetCount == NUM_PACKETS)
            break;
    }
    Qmss_ackInterrupt (cfg.channel, 1);
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, cfg.channel);
#endif

    System_printf ("\n--------------------Deinitializing----------------------------\n");
#ifdef CONFIG_ACC
    /* Disable accumulator */
    if ((result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, cfg.channel)) != QMSS_ACC_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Disabling high priority accumulator for channel : %d queue : %d error code: %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        return;
    }
    else
    {
        System_printf ("Core %d : high priority accumulator disabled for channel : %d queue : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex);
    }
#endif

    result = Qmss_getQueueEntryCount (txQueHnd);
    System_printf ("Transmit Queue %d Entry Count : %d \n", txQueHnd, result);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count : %d \n", txFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (freeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count : %d \n", freeQueHnd, result);

    result = Qmss_getQueueEntryCount (rxQueHnd);
    System_printf ("Receive Queue %d Entry Count : %d \n", rxQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count : %d \n", txCmplQueHnd, result);

    /* Flush the queues */
    Qmss_queueEmpty (txFreeQueHnd);
    Qmss_queueEmpty (freeQueHnd);
    Qmss_queueEmpty (txCmplQueHnd);
    Qmss_queueEmpty (rxQueHnd);
    Qmss_queueEmpty (txQueHnd);

    /* Close Tx channel */
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Tx channel error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Tx Channel closed successfully. Ref count : %d\n", coreNum, result);
    }

    /* Close Rx channel */
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Rx channel error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Rx Channel closed successfully. Ref count : %d\n", coreNum, result);
    }

    /* Close Rx flow */
    if ((result = Cppi_closeRxFlow (rxFlowHnd)) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Rx flow error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Rx flow closed successfully. Ref count : %d\n", coreNum, result);
    }

    /* Close the queues */
#ifndef __LINUX_USER_SPACE
    if ((result = Qmss_queueClose (rxQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing Rx queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Rx queue closed successfully. Ref count : %d\n", coreNum, result);
    }
#else
    if (releaseUioRxQueue (rxTaskArgs.waitFd, rxQueHnd) < 0)
    {
        System_printf ("Core %d : failed to release qpend fd/hnd\n", coreNum);
        errorCount++;
    }
#endif

    if ((result = Qmss_queueClose (txQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing tx queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Tx queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (freeQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing free queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Free queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (txCmplQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing transmit completion queue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Transmit completion queue closed successfully. Ref count : %d\n", coreNum, result);
    }

    if ((result = Qmss_queueClose (txFreeQueHnd)) != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Closing transmit freequeue error code : %d\n", coreNum, result);
    }
    else
    {
        System_printf ("Core %d : Transmit free queue closed successfully. Ref count : %d\n", coreNum, result);
    }
}

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
/**
 *  @b Description
 *  @n
 *      Entry point for the example code.
 *      This is an QMSS infrastructure mode example. Works in both polled or
 *      accumulated mode (except on Linux)
 *
 *      It performs the following
 *          - Initializes the Queue Manager low level driver.
 *          - Initializes the CPPI low level driver.
 *          - Opens the CPPI CPDMA in queue manager
 *          - Initializes descriptors and pushes to free queue
 *          - Programs accumulator
 *          - Pushes packets on Tx channel
 *          - Processes the accumulated packets from Rx channel
 *          - Closes Rx and Tx channel
 *          - Closes all open queues
 *          - Closes CPDMA instance
 *          - Deinitializes CPPI LLD
 *  @retval
 *      Not Applicable.
 */
#ifndef __LINUX_USER_SPACE
void main (void)
#else
void example_main(Cppi_Handle cppiHnd)
#endif
{
    Qmss_Result             result;
#ifndef __LINUX_USER_SPACE
    Cppi_Handle             cppiHnd;
#ifdef __ARM_ARCH_7A__
    /* Add MMU entries for MMR's required for PCIE example */
    Uint32 privid, index;
    CSL_MsmcRegs *msmc = (CSL_MsmcRegs *)CSL_MSMC_CFG_REGS;
    Mmu_DescriptorAttrs attrs;
    extern char ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;
    uint32_t addr = (uint32_t)&ti_sysbios_family_arm_a15_Mmu_Module_State_0_secondLevelTableBuf_1__A;

    Mmu_initDescAttrs(&attrs);

    attrs.type = Mmu_DescriptorType_TABLE;
    attrs.shareable = 0;            // non-shareable
    attrs.accPerm = 1;              // read/write at any privelege level
    attrs.attrIndx = 0;             // Use MAIR0 Register Byte 3 for
                                    // determining the memory attributes
                                    // for each MMU entry


    // Update the first level table's MMU entry for 0x80000000 with the
    // new attributes.
    Mmu_setFirstLevelDesc((Ptr)0x40000000, (UInt64)addr, &attrs);

    // Set up SES & SMS to make all masters coherent
    for (privid = 0; privid < 16; privid++)
    {
      for (index = 0; index < 8; index++)
      {
        uint32_t ses_mpaxh = msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH;
        uint32_t sms_mpaxh = msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH;
        if (CSL_FEXT (ses_mpaxh, MSMC_SES_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          ses_mpaxh &= ~0x80;
          msmc->SES_MPAX_PER_PRIVID[privid].SES[index].MPAXH = ses_mpaxh;
        }
        if (CSL_FEXT (sms_mpaxh, MSMC_SMS_MPAXH_0_SEGSZ) != 0)
        {
          // Clear the "US" bit to make coherent.  This is at 0x80.
          sms_mpaxh &= ~0x80;
          msmc->SMS_MPAX_PER_PRIVID[privid].SMS[index].MPAXH = sms_mpaxh;
        }
      }
    }
#endif
#endif

    System_printf ("**************************************************\n");
    System_printf ("*********QMSS Infrastructure Mode Example ********\n");
    System_printf ("**************************************************\n");

#ifndef __LINUX_USER_SPACE
    passPowerUp();

#ifdef _TMS320C6X
    /* Get the core number. */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM);

    /* Initialize the heap in shared memory for CPPI data structures */
    cppiHeapInit ();
#else
    coreNum = 0;
#endif

    System_printf ("********************Test running on Core %d ********************\n", coreNum);

    System_printf ("\n-----------------------Initializing---------------------------\n");

    memset ((Void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));

    memset ((Void *) &linkingRAM0, 0, sizeof (linkingRAM0));
    /* Set up the linking RAM. Use the internal Linking RAM.
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if
     * a value of zero is specified.
     * Linking RAM1 is not used */
#ifdef INTERNAL_LINKING_RAM
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC + NUM_HOST_DESC;
#else
    qmssInitConfig.linkingRAM0Base = (uint32_t) l2_global_address((Uint32)&linkingRAM0[0]);
    qmssInitConfig.linkingRAM0Size = 0x3FFF;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC + NUM_HOST_DESC;
#endif

#ifdef CONFIG_ACC
#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = &acc48_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif
#endif

    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
        return;
    }

    /* Start Queue Manager SubSystem */
    result = Qmss_start ();
    if (result != QMSS_SOK)
    {
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Initialize CPPI LLD */
    result = Cppi_init (&cppiGblCfgParams);
    if (result != CPPI_SOK)
    {
        System_printf ("Error Core %d : Initializing CPPI LLD error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Set up QMSS CPDMA configuration */
    memset ((Void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
#ifndef NSS_LITE    
    cpdmaCfg.dmaNum = INFRASTRUCTURE_MODE_CPDMA_TYPE;
#else
    cpdmaCfg.dmaNum = Cppi_CpDma_NETCP_CPDMA;
#endif    

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing QMSS CPPI CPDMA %d\n", coreNum, cpdmaCfg.dmaNum);
        return;
    }
#endif

    /* Setup memory region for host descriptors */
    memset ((void *) hostDesc, 0, SIZE_HOST_DESC * NUM_HOST_DESC);
    hostMemInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) hostDesc);
    hostMemInfo.descSize = SIZE_HOST_DESC;
    hostMemInfo.descNum = NUM_HOST_DESC;
    hostMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    hostMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    hostMemInfo.startIndex = 0;

    /* Setup memory region for monolithic descriptors */
    memset ((void *) monolithicDesc, 0, SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC);
    monoMemInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) monolithicDesc);
    monoMemInfo.descSize = SIZE_MONOLITHIC_DESC;
    monoMemInfo.descNum = NUM_MONOLITHIC_DESC;
    monoMemInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    monoMemInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    monoMemInfo.startIndex = 0;

    /* For devices that require ordering, insert lowest address first */
    if (hostMemInfo.descBase < monoMemInfo.descBase)
    {
       /* host starts on lower address */
       hostReg = Qmss_insertMemoryRegion (&hostMemInfo);
       monoReg = Qmss_insertMemoryRegion (&monoMemInfo);
    } 
    else
    {
       /* Mono starts on lower address */
       monoReg = Qmss_insertMemoryRegion (&monoMemInfo);
       hostReg = Qmss_insertMemoryRegion (&hostMemInfo);
    }

    if (hostReg < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, hostMemInfo.memRegion, hostReg);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, hostReg);
    }

    if (monoReg < QMSS_SOK)
    {
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, monoMemInfo.memRegion, monoReg);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, monoReg);
    }

    /* Transfer data using host descriptors */
    hostModeTransfer (cppiHnd);

    /* Transfer data using monolithic descriptors */
    monoModeTransfer (cppiHnd);

    /* Close CPPI CPDMA instance */
    if ((result = Cppi_close (cppiHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing CPPI CPDMA error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : CPPI CPDMA closed successfully\n", coreNum);
    }

    /* Deinitialize CPPI LLD */
    if ((result = Cppi_exit ()) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Exiting CPPI error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : CPPI exit successful\n", coreNum);
    }

    /* Free the memory regions */
    if ((result = Qmss_removeMemoryRegion (monoReg, 0)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Remove mono region error code : %d\n", coreNum, result);
    }

    if ((result = Qmss_removeMemoryRegion (hostReg, 0)) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Remove host region error code : %d\n", coreNum, result);
    }

    /* Deinitialize QMSS LLD */
    if ((result = Qmss_exit ()) != QMSS_SOK)
    {
        System_printf ("Error Core %d : Exiting QMSS error code : %d\n", coreNum, result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : QMSS exit successful\n", coreNum);
    }

    if (errorCount)
    {
        System_printf ("Example FAILED with %d errors\n", errorCount);
    }
    else
    {
        System_printf ("*******************************************************\n");
        System_printf ("*****QMSS Infrastructure Mode Example Done (PASS)******\n");
        System_printf ("*******************************************************\n");
    }
}


