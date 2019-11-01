/**
 *   @file  qmss_interrupt_test_multiprocess.c
 *
 *   @brief
 *      This is an QMSS multiprocess interrupt test example code that can
 *      run on multiple cores. Example code includes benchmarking of
 *      interrupt latency.
 * 	    The test can be run with different test modes
 *      Testmode; 0 ->Multicore send , single core receive,
 *                    Receive thread dequeues single packet per interrupt
 *                1 ->Single core send, single core receive
 *                    Receive thread dequeues single packet per interrupt
 *                2 -> Multicore send, single core receive,
 *                    Receive thread dequeues all available packets on interrupt
 *      Code can be compiled with WITH_INFRADMA to use infraDMA to send Packets
 *      By default latency is measured by using timestamp passed through the descriptor
 *      Defining SHARED_MEMORY uses instead shared memory to store timestamps and also
 *        measures breakdown of cycles into latency, read, write & process cycles
 *  ============================================================================
 *  @n   (C) Copyright 2015, Texas Instruments, Inc.
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
 *
*/

#include <sched.h>
#include "fw_test.h"
#include "fw_mem_allocator.h"
#include "linuxutil.h"
#include "qmss_test.h"
#include <string.h>
#include <pthread.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* Example configuration */
#include "infrastructure_multicoremode_cfg.h"

/* Device specific include */
#include "qmssPlatCfg.h"

/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* Queue bases */
#include <ti/csl/csl_qm_queue.h>

#include <errno.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
/************************ USER DEFINES ********************/

#define MAPPED_VIRTUAL_ADDRESS      0x81000000

/* MPAX segment 2 registers */
#define XMPAXL2                     0x08000010
#define XMPAXH2                     0x08000014

#define SYSINIT_SHED_PRIORITY 33

/* Can be different from NUM_PACKETS */
#define NUM_PACKETS_TO_SEND 1000 /*NUM_PACKETS*/
#define TEST_MODE_1_SEND_CORE_NUMBER 0

#define EMUCNT_PROFILE

/* #define SHARED_MEMORY */
/* #define WITH_INFRADMA */
/* #define DEBUG */
#define THROUGHPUT_MULTIPLIER 300000000 /* 1200000000/4 on EVM */
#define NUM_PROFILE_POINTS 5

#ifdef EMUCNT_PROFILE
#include <ti/csl/csl_pllc.h>
CSL_PllcRegsOvly pllcRegs = (CSL_PllcRegsOvly)(CSL_PLLC_REGS);
int fd_mem;
uint32_t *emucnt0_ptr;
#ifdef SHARED_MEMORY

#if NUM_PACKETS_TO_SEND > NUM_PACKETS
#define MAX_NUM_INTERRUPTS NUM_PACKETS_TO_SEND
#else
#define MAX_NUM_INTERRUPTS NUM_PACKETS
#endif
#define SHARED_MEMORY_OBJECT_NAME "mySharedRegion"
#define SHARED_MEMORY_OBJECT_SIZE 0x4000
int shm_fd;
typedef struct sharedMemData_s {
    uint32_t recordPacketSendStart[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordPacketSendEnd[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordIntReceive[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordIntReceiveSecond[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordAfterRead[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordAfterProcess[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
    uint32_t recordAfterWrite[NUMBER_OF_CORES][MAX_NUM_INTERRUPTS];
} sharedMemData_t;
sharedMemData_t *sharedMemDataP;
#else
#define HISTOGRAM_MASK   0xf
#define HISTOGRAM_SHIFT  9
typedef struct latencyData_s {
    uint32_t min_latency_delta;
    uint32_t max_latency_delta;
    uint32_t min_latency_time;
    uint32_t max_latency_time;
    uint32_t min_latency_seqno;
    uint32_t max_latency_seqno;
    uint32_t average_latency_delta;
    uint32_t histogram[HISTOGRAM_MASK+1];
} latencyData_t;
latencyData_t latencyData[NUMBER_OF_CORES][NUM_PROFILE_POINTS];
#endif
#endif /* EMUCNT_PROFILE */
/************************ GLOBAL VARIABLES ********************/

uint32_t                retryCount = 0;
unsigned int totalPacketsReceived = 0;
unsigned long long interruptProcessStartTime, interruptProcessEndTime;


/* Global variables for command line parameters */
int numPacketsToSend;
int delayBetweenPacketsInMicrosec;
int testMode;

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
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;

uint32_t                    runCount;

/* Indicates the core or logical task ID test is running on */
uint32_t                coreNum;
/* Regions returned my Qmss_insertMemoryRegion */
Qmss_Result             reg1, reg2;

Cppi_Handle             cppiHnd;
Cppi_ChHnd              rxChHnd, txChHnd;
Qmss_QueueHnd           txQueHnd, rxQueHnd, rxFreeQueHnd, txCmplQueHnd, txFreeQueHnd, syncQueHnd, syncFreeQueHnd, syncCfgQueHnd;
int32_t                 rxQIDByCore[NUMBER_OF_CORES];
Qmss_QueueHnd           qpendRxQueHnd;
int                     qpendFd;
int32_t                 CPPI_FREE_TX_QID, CPPI_FREE_RX_QID, CPPI_COMPLETION_QID, QMSS_SYNC_CFG_QID, QMSS_SYNC_QID, QMSS_FREE_SYNC_QID;

Cppi_DescCfg            descCfg;
Qmss_DescCfg            syncDescCfg;
Qmss_DescCfg            txDescCfg;
Cppi_FlowHnd            rxFlowHnd;

int qpendUioFd[NUMBER_OF_CORES];
Qmss_QueueHnd qpendRxQueHndArray[NUMBER_OF_CORES];

#ifdef EMUCNT_PROFILE
void *map_dev_mem(uint32_t addr, int size)
{
     uint32_t base, offset, min_length, adjusted_length;
     uint32_t page_size = getpagesize();
     char *map1;

     base = addr  & (~(page_size-1));
     offset = (addr -base);
     min_length = offset+size;
     adjusted_length = min_length & (~(page_size-1));
     adjusted_length = (adjusted_length == min_length) ? min_length : adjusted_length + page_size;

     map1 = mmap(NULL, adjusted_length, PROT_READ|PROT_WRITE, MAP_SHARED,
                                           fd_mem, base);
     if (map1 == MAP_FAILED) {
          printf("\nmmap failed for addr 0x%x size %d (err: %s)",
          addr, size, strerror(errno));
          return (void *)(-1);
     }
     return((void *)(map1 + offset));
}
#ifdef SHARED_MEMORY

static void print_latency_stats(uint32_t core_id, uint32_t *startRecordP,
    uint32_t *endRecordP, char *param_name)
{
    unsigned long long accumulated_delay;
    unsigned int delay, average_delay;
    unsigned int min_delay, max_delay;
    int j;

    min_delay = 0xffffffff;
    max_delay = 0;
    accumulated_delay = 0;
    for(j=0; j< numPacketsToSend; j++) {
         delay = (endRecordP[j] - startRecordP[j])*4;
#ifdef DEBUG
         System_printf("Core num: %d: %s : %d cycles\n",
             core_id, param_name, delay);
#endif
         accumulated_delay += delay;
         min_delay = (delay < min_delay) ? delay : min_delay;
         max_delay = (delay > max_delay) ? delay : max_delay;
    }

    average_delay = accumulated_delay/numPacketsToSend;
    System_printf("Core num: %d: %s Cycles: "
        "Average = %d Min = %d Max %d\n\n",
         core_id, param_name, average_delay, min_delay, max_delay);
}
#else /* SHARED_MEMORY */
static void print_latency_stats(uint32_t core_id, uint32_t index, char *param_name,
    int summary)
{
    int i;

    if ( summary )
    {
        System_printf("Core num: %d: %s :Cycles:"
        "Average = %d Min = %d Max %d\n",
         core_id, param_name,
         latencyData[core_id][index].average_latency_delta*4,
         latencyData[core_id][index].min_latency_delta*4,
         latencyData[core_id][index].max_latency_delta*4);
    } else {
        System_printf("Core num: %d: %s "
            "Min time = 0x%x Min seqno %d Max time = 0x%x Max seqno %d\n",
            core_id, param_name,
            latencyData[core_id][index].min_latency_time,
            latencyData[core_id][index].min_latency_seqno,
            latencyData[core_id][index].max_latency_time,
            latencyData[core_id][index].max_latency_seqno);

        System_printf("Core num: %d: Histogram of Cycles\n", core_id);
        for ( i=0; i < HISTOGRAM_MASK; i++)
        {
            System_printf("  %2dK| %d\n", 4*(i+1)*(1<<HISTOGRAM_SHIFT)/1000,
                latencyData[core_id][index].histogram[i]);
        }
        System_printf("> %2dK| %d\n", 4*(i+1)*(1<<HISTOGRAM_SHIFT)/1000,
                latencyData[core_id][index].histogram[i]);
    }
}
void print_latency_overhead_summary (uint32_t core_id)
{

    System_printf("Core num: %d: Latency overhead Cycles:"
        "Average = %d\n",
         core_id, 
         (latencyData[core_id][0].average_latency_delta
          + latencyData[core_id][3].average_latency_delta
          + latencyData[core_id][4].average_latency_delta
          )*4);
    System_printf("Core num: %d: Latency overhead ( including push to freeQ )\n"
        " Cycles: Average = %d \n",
         core_id, 
         (latencyData[core_id][0].average_latency_delta
          + latencyData[core_id][1].average_latency_delta
          + latencyData[core_id][3].average_latency_delta
          + latencyData[core_id][4].average_latency_delta
          )*4);

}
#endif /* SHARED_MEMORY */
#endif /* EMUCNT_PROFILE */

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
static uint32_t l2_global_address (uint32_t addr)
{
    return addr;
}

/**
 *  @b Description
 *  @n
 *      Yields the task using something like sleep() or usleep().
 *
 *  @retval
 *      Not Applicable
 */
void yield (void)
{
//    sleep(0);
    sched_yield();
}

/**
 *  @b Description
 *  @n
 *      Utility to feed a command to rm
 *
 *  @param[in]  name
 *      Name to query
 *  @param[in]  val
 *      return value or value to set
 *  @param[in]  type
 *      transaction type
 *  @param[in]  name
 *      retry until success
 *
 *  @retval
 *      Value
 */
void ex_rm_cmd (const char *name, uint32_t *val, Rm_ServiceType type, int retry)
{
    Rm_ServiceReqInfo   rmServiceReq;
    Rm_ServiceRespInfo  rmServiceResp;
    int                 succeed;

    memset((void *)&rmServiceReq, 0, sizeof(rmServiceReq));
    memset((void *)&rmServiceResp, 0, sizeof(rmServiceResp));

    rmServiceReq.type             = type;
    rmServiceReq.resourceName     = name;
    rmServiceReq.resourceNsName   = name;
    rmServiceReq.resourceLength   = 1;
    if (val)
    {
        rmServiceReq.resourceBase = *val;
    }

    /* RM will block until resource is returned since callback is NULL */
    rmServiceReq.callback.serviceCallback = NULL;
    do {
        rmClientServiceHandle->Rm_serviceHandler(rmClientServiceHandle->rmHandle, &rmServiceReq, &rmServiceResp);
        succeed = (rmServiceResp.serviceState == RM_SERVICE_APPROVED) ||
                  (rmServiceResp.serviceState == RM_SERVICE_APPROVED_STATIC);
        if (retry && (! succeed))
        {
            yield();
        }
    } while (retry && (! succeed));

    if (succeed)
    {
        if ((type == Rm_service_RESOURCE_GET_BY_NAME) && (val))
        {
            *val = rmServiceResp.resourceBase;
        }
    }
    else
    {
        System_printf ("Core %d: failed rm transaction %d %s %d (%d)\n", coreNum, type, name, val ? *val : 0, rmServiceResp.serviceState);
        errorCount++;
    }
}

/**
 *  @b Description
 *  @n
 *      Utility to look up a name in RM
 *
 *  @param[in]  name
 *      Name to query
 *
 *  @retval
 *      Value
 */
uint32_t ex_rm_name_lookup (const char *name)
{
    uint32_t val;
    ex_rm_cmd (name, &val, Rm_service_RESOURCE_GET_BY_NAME, 1);

    return val;
}

/**
 *  @b Description
 *  @n
 *      Utility to set a name in RM
 *
 *  @param[in]  name
 *      Name to query
 *
 *  @param[in]  val
 *      Value
 *
 *  @retval
 *      None
 */
void ex_rm_name_set (const char *name, uint32_t val)
{
    ex_rm_cmd (name, &val, Rm_service_RESOURCE_MAP_TO_NAME, 0);
}

/**
 *  @b Description
 *  @n
 *      Utility to delete a name from RM
 *
 *  @param[in]  name
 *      Name to delete
 *
 *  @retval
 *      None
 */
void ex_rm_name_del (const char *name)
{
    ex_rm_cmd (name, NULL, Rm_service_RESOURCE_UNMAP_NAME, 0);
}

/**
 *  @b Description
 *  @n
 *      Utility function that prints entry count in various queues
 *
 *  @param[in]  msg
 *      Status message to be printed
 *
 *  @retval
 *      Not Applicable
 */
static void printQueueStats (char *msg)
{
    Qmss_Result     result;

    System_printf ("\n--------------------Queue status CORE %d----------------------\n", coreNum);
    System_printf ("                    %s\n\n", msg);

    result = Qmss_getQueueEntryCount (txFreeQueHnd);
    System_printf ("Tx Free Queue %d Entry Count            : %d \n", txFreeQueHnd, result);

#ifdef WITH_INFRADMA
    result = Qmss_getQueueEntryCount (rxFreeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count            : %d \n", rxFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count     : %d \n", txCmplQueHnd, result);
#endif /* WITH_INFRADMA */

    result = Qmss_getQueueEntryCount (syncQueHnd);
    System_printf ("Sync Queue %d Entry Count              : %d \n", syncQueHnd, result);

    result = Qmss_getQueueEntryCount (syncFreeQueHnd);
    System_printf ("Sync free Queue %d Entry Count         : %d \n", syncFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (syncCfgQueHnd);
    System_printf ("Sync Cfg Queue %d Entry Count          : %d \n", syncCfgQueHnd, result);

    System_printf ("-------------------------------------------------------------\n\n");
}
#ifdef WITH_INFRADMA
/**
 *  @b Description
 *  @n
 *      Closes queues, channel and flow used in a single iteration.
 *
 *  @retval
 *      Not Applicable
 */
static void cleanup (void)
{
    Qmss_Result     result;

    /* Close Rx queue */
    if ((result = Qmss_queueClose (rxQueHnd)) < QMSS_SOK)
    {
        System_printf ("Error Core %d : Closing Rx queue %d error code : %d\n", coreNum, rxQueHnd, result);
        errorCount++;
    }
    /* Close Tx queue */
    if ((result = Qmss_queueClose (txQueHnd)) < QMSS_SOK)
    {
        System_printf ("Error Core %d : Closing tx queue %d error code : %d\n", coreNum, txQueHnd, result);
        errorCount++;
    }

    /* Disable Tx channel */
    if ((result = Cppi_channelDisable (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling Tx channel error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Disable Rx channel */
    if ((result = Cppi_channelDisable (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Disabling Rx channel error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Close Tx channel */
    if ((result = Cppi_channelClose (txChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Tx channel error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Close Rx channel */
    if ((result = Cppi_channelClose (rxChHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx channel error code : %d\n", coreNum, result);
        errorCount++;
    }

    /* Close Rx flow */
    if ((result = Cppi_closeRxFlow (rxFlowHnd)) != CPPI_SOK)
    {
        System_printf ("Error Core %d : Closing Rx flow error code : %d\n", coreNum, result);
        errorCount++;
    }
}
#endif /* WITH_INFRADMA */


/**
 *  @b Description
 *  @n
 *
 *      Initializes the system
 *      It performs the following
 *          - Initializes the Queue Manager low level driver
 *          - COnfigures descriptor memory region
 *          - Initializes the CPPI low level driver
 *          - Opens the CPPI CPDMA in queue manager
 *          - Initializes descriptors and pushes to free queue
 *          - Opens synchronization queues
 *  @retval
 *      Not Applicable.
 */
static Int32 sysInit (void)
{
    int                     i;
#ifdef WITH_INFRADMA
    Qmss_Queue              queInfo;
    Cppi_Desc               *monoDescPtr;
#endif /* WITH_INFRADMA */
    uint32_t                numAllocated;
    uint8_t                 isAllocated;

    System_printf ("\n-----------------------Initializing---------------------------\n");

    /* Set QID to -1 to let RM allocate */
    CPPI_FREE_TX_QID    = QMSS_PARAM_NOT_SPECIFIED;
    CPPI_FREE_RX_QID    = QMSS_PARAM_NOT_SPECIFIED;
    CPPI_COMPLETION_QID = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_SYNC_CFG_QID   = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_SYNC_QID       = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_FREE_SYNC_QID  = QMSS_PARAM_NOT_SPECIFIED;

    /* Initilaize the number of times the test was run to zero */
    runCount = 0;

    /* Setup memory region for monolithic descriptors */
    memset(&memInfo, 0, sizeof(memInfo));
    memset ((void *) monolithicDesc, 0, SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC);
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) monolithicDesc);
    memInfo.descSize = SIZE_MONOLITHIC_DESC;
    memInfo.descNum = NUM_MONOLITHIC_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg1 = Qmss_insertMemoryRegion (&memInfo);
    if (reg1 < QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, memInfo.memRegion, reg1);
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, reg1);
    }

    /* Setup memory region for Sync descriptors */
    memset(&memInfo, 0, sizeof(memInfo));
    memset ((void *) syncDesc, 0, SIZE_SYNC_DESC * NUM_SYNC_DESC);
    memInfo.descBase = (uint32_t *) l2_global_address ((uint32_t) syncDesc);
    memInfo.descSize = SIZE_SYNC_DESC;
    memInfo.descNum = NUM_SYNC_DESC;
    memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
    memInfo.memRegion = Qmss_MemRegion_MEMORY_REGION_NOT_SPECIFIED;
    memInfo.startIndex = 0;

    reg2 = Qmss_insertMemoryRegion (&memInfo);
    if (reg2 < QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Inserting memory region %d error code : %d\n", coreNum, memInfo.memRegion, reg2);
    }
    else
    {
        System_printf ("Core %d : Memory region %d inserted\n", coreNum, reg2);
    }
#ifdef WITH_INFRADMA

    /* Opens transmit completion queue. */
    if ((txCmplQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx Completion Queue Number\n", coreNum);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Tx Completion Queue Number     : %d opened\n", coreNum, txCmplQueHnd);

        /* Save the QID received from RM */
        CPPI_COMPLETION_QID = Qmss_getQIDFromHandle (txCmplQueHnd);
    }
#endif
    /* Setup the descriptors for transmit free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)reg1;
    descCfg.descNum = NUM_MONOLITHIC_DESC / 2;
    descCfg.destQueueNum = CPPI_FREE_TX_QID;
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;

    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;
#ifdef WITH_INFRADMA
    /* Descriptor should be recycled back to Queue Number 1000 */
    queInfo = Qmss_getQueueNumber (txCmplQueHnd);
    descCfg.returnQueue.qMgr = queInfo.qMgr;
    descCfg.returnQueue.qNum = queInfo.qNum;
#else
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;
#endif
    /* Initialize the descriptors and push to free Queue */
    if ((txFreeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Tx descriptor error code: %d \n", coreNum, txFreeQueHnd);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Number of Tx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
#ifdef WITH_INFRADMA
        /* Set return queue to be the txFreeQ itself for all packets */
        for ( i=0; i < NUM_MONOLITHIC_DESC / 2; i++)
        {
            if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
            {
                errorCount++;
                System_printf ("Error Core %d : Getting descriptor from Queue Number %d\n",
                    coreNum, txFreeQueHnd);
                return -1;
            }

            queInfo = Qmss_getQueueNumber (txFreeQueHnd);

            Cppi_setReturnQueue (Cppi_DescType_MONOLITHIC, monoDescPtr, queInfo);

            /* Push descriptor back to Tx free queue */
            Qmss_queuePushDesc (txFreeQueHnd, (uint32_t *) monoDescPtr);
        }
#endif
        /* Save the QID received from RM */
        CPPI_FREE_TX_QID = Qmss_getQIDFromHandle (txFreeQueHnd);
    }
#ifdef WITH_INFRADMA
    /* Setup the descriptors for receive free queue */
    memset(&descCfg, 0, sizeof(descCfg));
    descCfg.memRegion = (Qmss_MemRegion)reg1;
    descCfg.descNum = NUM_MONOLITHIC_DESC / 2;
    descCfg.destQueueNum = CPPI_FREE_RX_QID;
    descCfg.queueType = Qmss_QueueType_STARVATION_COUNTER_QUEUE;
    descCfg.initDesc = Cppi_InitDesc_INIT_DESCRIPTOR;
    descCfg.descType = Cppi_DescType_MONOLITHIC;
    descCfg.epibPresent = Cppi_EPIB_NO_EPIB_PRESENT;
    descCfg.cfg.mono.dataOffset = MONOLITHIC_DESC_DATA_OFFSET;

    /* Descriptor should be recycled back to Queue Number 1000 */
    descCfg.returnQueue.qMgr = QMSS_PARAM_NOT_SPECIFIED;
    descCfg.returnQueue.qNum = QMSS_PARAM_NOT_SPECIFIED;

    /* Initialize the descriptors and push to free Queue */
    if ((rxFreeQueHnd = Cppi_initDescriptor (&descCfg, &numAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Rx descriptor error code: %d \n", coreNum, rxFreeQueHnd);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Number of Rx descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, descCfg.descNum, numAllocated);
        /* Save the QID received from RM */
        CPPI_FREE_RX_QID = Qmss_getQIDFromHandle (rxFreeQueHnd);
    }
#endif /* WITH_INFRADMA */
    /* Setup the descriptors for sync free queue */
    syncDescCfg.memRegion = (Qmss_MemRegion)reg2;
    syncDescCfg.descNum = NUM_SYNC_DESC;
    syncDescCfg.destQueueNum = QMSS_FREE_SYNC_QID;
    syncDescCfg.queueType = Qmss_QueueType_GENERAL_PURPOSE_QUEUE;

    /* Initialize the descriptors and push to free Queue */

    if ((syncFreeQueHnd = Qmss_initDescriptor (&syncDescCfg, &numAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Sync free descriptor error code: %d \n", coreNum, syncFreeQueHnd);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Number of Sync free descriptors requested : %d. Number of descriptors allocated : %d \n",
            coreNum, syncDescCfg.descNum, numAllocated);
        /* Save the QID received from RM */
        QMSS_FREE_SYNC_QID = Qmss_getQIDFromHandle (syncFreeQueHnd);

    }


    /* Opens sync queue */
    if ((syncQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_SYNC_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Sync Queue Number\n", coreNum);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Sync Queue Number              : %d opened\n", coreNum, syncQueHnd);
        /* Save the QID received from RM */
        QMSS_SYNC_QID = Qmss_getQIDFromHandle (syncQueHnd);
    }

    /* Opens sync Configuration queue */
    if ((syncCfgQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_SYNC_CFG_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Sync Cfg Queue Number\n", coreNum);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Sync Cfg Queue Number          : %d opened\n", coreNum, syncCfgQueHnd);
        /* Save the QID received from RM */
        QMSS_SYNC_CFG_QID = Qmss_getQIDFromHandle (syncCfgQueHnd);
    }

    /* Opens sync free queue. */

    System_printf ("Core %d : Sync Free Queue Number         : %d opened\n", coreNum, syncFreeQueHnd);
#ifdef WITH_INFRADMA
    System_printf ("Core %d : Receive Free Queue Number      : %d opened\n", coreNum, rxFreeQueHnd);
#endif /* WITH_INFRADMA */
    System_printf ("Core %d : Transmit Free Queue Number     : %d opened\n", coreNum, txFreeQueHnd);

    System_printf ("Core %d : System initialization completed: %d\n", coreNum, txFreeQueHnd);

    printQueueStats ("After Initialization");
    System_printf ("Core %d : Publishing RM nameserver names for shared queues\n", coreNum);
    ex_rm_name_set ("INFRAMC_FREE_TX",    CPPI_FREE_TX_QID);
#ifdef WITH_INFRADMA
    ex_rm_name_set ("INFRAMC_FREE_RX",    CPPI_FREE_RX_QID);
    ex_rm_name_set ("INFRAMC_COMPLETION", CPPI_COMPLETION_QID);
#endif /* WITH_INFRADMA */
    ex_rm_name_set ("INFRAMC_SYNC_CFG",   QMSS_SYNC_CFG_QID);
    ex_rm_name_set ("INFRAMC_SYNC",       QMSS_SYNC_QID);
    ex_rm_name_set ("INFRAMC_SYNC_FREE",  QMSS_FREE_SYNC_QID);

    /* Get rest of cores' receive queues */
    System_printf ("Core %d: Getting receive queues for other cores\n", coreNum);
    for (i = 0; i < NUMBER_OF_CORES; i++)
    {
        char corename[32];
        if( i == SYSINIT ) continue;
        snprintf (corename, sizeof(corename), "INFRAMC_CORE%d_RXQ", i);
        rxQIDByCore[i] = ex_rm_name_lookup (corename);
        printf ("core %d: Got core %d's RX QID: %d\n", coreNum, i, rxQIDByCore[i]);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function initializes the common queue handles opened by producer core on consumer cores
 *
 *  @retval
 *      Not Applicable
 */
static void getsysHandles (void)
{
    uint8_t                 isAllocated;

    /* Interrogate the queue IDs */
    CPPI_FREE_TX_QID    = ex_rm_name_lookup ("INFRAMC_FREE_TX");
#ifdef WITH_INFRADMA
    CPPI_FREE_RX_QID    = ex_rm_name_lookup ("INFRAMC_FREE_RX");
    CPPI_COMPLETION_QID = ex_rm_name_lookup ("INFRAMC_COMPLETION");
#endif /* WITH_INFRADMA */
    QMSS_SYNC_CFG_QID   = ex_rm_name_lookup ("INFRAMC_SYNC_CFG");
    QMSS_SYNC_QID       = ex_rm_name_lookup ("INFRAMC_SYNC");
    QMSS_FREE_SYNC_QID  = ex_rm_name_lookup ("INFRAMC_SYNC_FREE");
#ifdef WITH_INFRADMA
    if ((rxFreeQueHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, CPPI_FREE_RX_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Rx Free Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Rx Free Queue Number       : %d opened\n", coreNum, rxFreeQueHnd);
    }
#endif /* WITH_INFRADMA */
    if ((txFreeQueHnd = Qmss_queueOpen (Qmss_QueueType_STARVATION_COUNTER_QUEUE, CPPI_FREE_TX_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx Free Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Tx Free Queue Number       : %d opened\n", coreNum, txFreeQueHnd);
    }
#ifdef WITH_INFRADMA
    if ((txCmplQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, CPPI_COMPLETION_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx Completion Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Tx Completion Queue Number : %d opened\n", coreNum, txCmplQueHnd);
    }
#endif /* WITH_INFRADMA */

    if ((syncQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_SYNC_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Sync Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Sync Queue Number          : %d opened\n", coreNum, syncQueHnd);
    }

    if ((syncFreeQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_FREE_SYNC_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Sync Free Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Sync Free Queue Number     : %d opened\n", coreNum, syncFreeQueHnd);
    }

    /* Opens sync Configuration queue */
    if ((syncCfgQueHnd = Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE, QMSS_SYNC_CFG_QID, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Sync Cfg Queue Number\n", coreNum);
        return;
    }
    else
    {
        System_printf ("Core %d : Sync Cfg Queue Number      : %d opened\n", coreNum, syncCfgQueHnd);
    }

    /* Share my receive queue */
    {
        char corename[32];
        snprintf (corename, sizeof(corename), "INFRAMC_CORE%d_RXQ", coreNum);
        ex_rm_name_set (corename, Qmss_getQIDFromHandle(qpendRxQueHnd));
    }
}

/**
 *  @b Description
 *  @n
 *
 *      Deinitializes the system
 *      It performs the following
 *          - Closes all open common queues
 *          - Closes CPDMA instance
 *          - Deinitializes CPPI LLD
 *  @retval
 *      Not Applicable.
 */
static void sysExit (void)
{
    Qmss_Result     qmss_result;
    Cppi_Result     cppi_result;

    /* Drop the descriptors */
    if (coreNum == SYSINIT)
    {
#ifdef WITH_INFRADMA
        Qmss_queueEmpty (rxFreeQueHnd);
        Qmss_queueEmpty (txCmplQueHnd);
#endif /* WITH_INFRADMA */
        Qmss_queueEmpty (txFreeQueHnd);
        Qmss_queueEmpty (syncQueHnd);
        Qmss_queueEmpty (syncFreeQueHnd);
        Qmss_queueEmpty (syncCfgQueHnd);
    }

#ifdef WITH_INFRADMA
    /* Close the queues */
    if ((qmss_result = Qmss_queueClose (rxFreeQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing free queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Receive free queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }

    if ((qmss_result = Qmss_queueClose (txCmplQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing transmit completion queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Transmit completion queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }
#endif /* WITH_INFRADMA */

    if ((qmss_result = Qmss_queueClose (txFreeQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing transmit free queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Transmit free queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }

    if ((qmss_result = Qmss_queueClose (syncQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing sync queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Sync queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }

    if ((qmss_result = Qmss_queueClose (syncFreeQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing sync free queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Sync free queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }

    if ((qmss_result = Qmss_queueClose (syncCfgQueHnd)) < QMSS_SOK)
    {
        System_printf ("Core %d : Error closing sync cfg queue : %d\n", coreNum, qmss_result);
        errorCount++;
    }
    else
    {
        System_printf ("Core %d : Sync queue closed successfully. Ref count : %d\n", coreNum, qmss_result);
    }
    /* Delete my receive queue */
    if (coreNum != SYSINIT)
    {
        char corename[32];
        snprintf (corename, sizeof(corename), "INFRAMC_CORE%d_RXQ", coreNum);
        ex_rm_name_del (corename);
    }
    /* Clean my receive queue */
    if (releaseUioRxQueue (qpendFd, qpendRxQueHnd) < 0)
    {
        System_printf ("Core %d : failed to release qpend fd/hnd\n", coreNum);
        errorCount++;
    }

    {
        /* Close CPPI CPDMA instance */
        if ((cppi_result = Cppi_close (cppiHnd)) != CPPI_SOK)
        {
            errorCount++;
            System_printf ("Error Core %d : Closing CPPI CPDMA error code : %d\n", coreNum, cppi_result);
        }
        else
        {
            System_printf ("Core %d : CPPI CPDMA closed successfully\n", coreNum);
        }

        /* Deinitialize CPPI LLD */
        if ((cppi_result = Cppi_exit ()) != CPPI_SOK)
        {
            errorCount++;
            System_printf ("Error Core %d : Exiting CPPI error code : %d\n", coreNum, cppi_result);
        }
        else
        {
            System_printf ("Core %d : CPPI exit successful\n", coreNum);
        }
    }

    /* Remove my regions which were only intstalled from core 0 */
    if (coreNum == SYSINIT)
    {
        System_printf ("Core %d: Cleaning regions\n", coreNum);
        if ((qmss_result = Qmss_removeMemoryRegion (reg1, 0)))
        {
            errorCount++;
            System_printf ("Error Core %d : remove reg1 error code : %d\n", coreNum, qmss_result);
        }
        if ((qmss_result = Qmss_removeMemoryRegion (reg2, 0)))
        {
            errorCount++;
            System_printf ("Error Core %d : remove reg2 error code : %d\n", coreNum, qmss_result);
        }
    }

    /* Exit QMSS */
    System_printf ("Core %d: exit QMSS\n", coreNum);
    if ((qmss_result = Qmss_exit ()))
    {
        errorCount++;
        System_printf ("Error Core %d : exit error code : %d\n", coreNum, qmss_result);
    }
    if (coreNum == SYSINIT)
    {
        System_printf ("Core %d : Deleting RM nameserver names for shared queues\n", coreNum);

        ex_rm_name_del ("INFRAMC_FREE_TX");
#ifdef WITH_INFRADMA
        ex_rm_name_del ("INFRAMC_FREE_RX");
        ex_rm_name_del ("INFRAMC_COMPLETION");
#endif /* WITH_INFRADMA */
        ex_rm_name_del ("INFRAMC_SYNC_CFG");
        ex_rm_name_del ("INFRAMC_SYNC");
        ex_rm_name_del ("INFRAMC_SYNC_FREE");
    }
}

#ifdef EMUCNT_PROFILE
#ifndef SHARED_MEMORY
static void init_latency(void)
{
    int core_id, index;

    memset(latencyData, 0, sizeof(latencyData));
    for (core_id=0; core_id< NUMBER_OF_CORES; core_id++)
    {
        for (index=0; index< NUM_PROFILE_POINTS; index++)
        {
            latencyData[core_id][index].min_latency_delta = 0xffffffff;
        }
    }
}

static uint32_t record_latency (int core_id, int index, int seq_no, uint32_t start_time)
{
    uint32_t current_time, latency_delta;
    uint32_t histogram_index;

    current_time = emucnt0_ptr[0];

    latency_delta  = current_time - start_time;
    if (latency_delta < latencyData[core_id][index].min_latency_delta)
    {
        latencyData[core_id][index].min_latency_delta = latency_delta;
        latencyData[core_id][index].min_latency_time = current_time;
        latencyData[core_id][index].min_latency_seqno = seq_no;
    }
    if (latency_delta > latencyData[core_id][index].max_latency_delta)
    {
        latencyData[core_id][index].max_latency_delta = latency_delta;
        latencyData[core_id][index].max_latency_time = current_time;
        latencyData[core_id][index].max_latency_seqno = seq_no;
    }
    histogram_index = (latency_delta >> HISTOGRAM_SHIFT);
    if( histogram_index > HISTOGRAM_MASK )
        histogram_index = HISTOGRAM_MASK;

    latencyData[core_id][index].histogram[histogram_index]++;
    latencyData[core_id][index].average_latency_delta
        = (latencyData[core_id][index].average_latency_delta
            + latency_delta) >> 1;

    return(current_time);
}
#endif /* SHARED_MEMORY */
#endif /* EMUCNT_PROFILE */
/**
 *  @b Description
 *  @n
 *      Polling receive function for user space (or when accumulator isn't desired)
 *
 *  @retval
 *      Not Applicable.
 */
static void receive_data ()
{
    Cppi_Desc     *desc;
    int           index;
    int           packetsToProcess = 0;
    int           numfds=0;
    int           i, j;
    fd_set        fds;
    int interrupt_count[NUMBER_OF_CORES];
    uint32_t value;
    int ret;
    int first_interrupt_occurred =0;
    unsigned int common_interrupt_counter=0;
    unsigned int packets_received[NUMBER_OF_CORES];
#ifndef SHARED_MEMORY
    uint32_t pkt_seq_number;
    uint32_t send_time_stamp;
#endif

    memset(interrupt_count, 0, sizeof(int)*NUMBER_OF_CORES);
    memset(packets_received, 0, sizeof(int)*NUMBER_OF_CORES);
#ifdef EMUCNT_PROFILE
#ifndef SHARED_MEMORY
    init_latency();
#endif /* SHARED_MEMORY */
#endif /* EMUCNT_PROFILE */

    for (i=0; i < NUMBER_OF_CORES; i++) {
        if ( i == SYSINIT ) continue;
        numfds = (qpendUioFd[i] > numfds) ? qpendUioFd[i] : numfds;
    }
    System_printf ("core %d: waiting for packets \n", coreNum);
    do
    {
        FD_ZERO(&fds);
        for (i=0; i < NUMBER_OF_CORES; i++)
        {
            if ( i == SYSINIT ) continue;
            FD_SET(qpendUioFd[i], &fds);
        }

        ret = select(numfds+1, &fds, NULL, NULL, NULL);
        if (ret == -1) {
            errorCount++;
            System_printf("select failed for monitor (error: %s)",
                strerror(errno));
            return;
        }
        common_interrupt_counter++;
#ifdef EMUCNT_PROFILE
        if (!first_interrupt_occurred) {
            first_interrupt_occurred =1;
            interruptProcessStartTime = emucnt0_ptr[0];
            interruptProcessStartTime += ((unsigned long long)emucnt0_ptr[1] << 32);
        }
#endif
        for (i=0; i < NUMBER_OF_CORES; i++) {
            if ( i == SYSINIT ) continue;
            if( testMode == 1 )
            {
                if (i != TEST_MODE_1_SEND_CORE_NUMBER)
                    continue;
            }
            if (FD_ISSET(qpendUioFd[i], &fds)) {
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
                if (interrupt_count[i] < MAX_NUM_INTERRUPTS ) {
                    sharedMemDataP->recordIntReceive[i][interrupt_count[i]] = emucnt0_ptr[0];
                    sharedMemDataP->recordIntReceiveSecond[i][interrupt_count[i]] = emucnt0_ptr[0];
                }
#endif
#endif /* EMUCNT_PROFILE */
                /* Determine number of packets to process */
                if ( testMode == 2 )
                    packetsToProcess = Qmss_getQueueEntryCount (qpendRxQueHndArray[i]);
                else
                    packetsToProcess = 1;

#ifdef DEBUG
                System_printf ("core %d: got interrupt from process %d-"
                   "packets to process %d , common interrupt counter %d\n",
                   coreNum, i, packetsToProcess, common_interrupt_counter);
#endif
                if(!packetsToProcess)
                {
                    errorCount++;
                    System_printf("No packets after interrupt");
                    return;
                }
                /* Recycle Rx BDs */
                for (index = 0; index < packetsToProcess; index++)
                {
                    desc = Qmss_queuePop (qpendRxQueHndArray[i]);
#ifdef EMUCNT_PROFILE
#ifndef SHARED_MEMORY
                    /* Add code to record timestamp diffs */
                    pkt_seq_number = ((Cppi_MonolithicDesc *)desc)->softwareInfo0;
                    if( pkt_seq_number != packets_received[i] ) {
                       errorCount++;
                        System_printf ("Error Core %d :"
                            "Sequence number mismatch: from core %d:  %d != %d !\n",
                            coreNum, i, pkt_seq_number, packets_received[i]);
                    }

                    send_time_stamp = ((Cppi_MonolithicDesc *)desc)->softwareInfo1;
#endif
#endif
#ifdef WITH_INFRADMA
                    Qmss_queuePushDesc (rxFreeQueHnd, desc);
#else
                    Qmss_queuePushDesc (txFreeQueHnd, desc);
#endif
                    packets_received[i]++;
                }
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
                sharedMemDataP->recordAfterProcess[i][interrupt_count[i]] = emucnt0_ptr[0];
#endif
#endif
                ret = read( qpendUioFd[i], &value, sizeof(uint32_t));
                if(ret < 0)
                {
                    errorCount++;
                    System_printf("Read failed qpendUioFd[i]=%d",qpendUioFd[i]);
                    return;
                }
#ifdef DEBUG
                System_printf ("qpendUioFd value read : %d interrupt count[%d] %d\n",
                    value, i, interrupt_count[i]);
#endif
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
                sharedMemDataP->recordAfterRead[i][interrupt_count[i]] = emucnt0_ptr[0];
#endif
#endif
                value=1;
                ret = write(qpendUioFd[i], &value, sizeof(uint32_t));
                if (ret != sizeof(uint32_t)) {
                     errorCount++;
                    System_printf ("Error Core %d : Unable to enable interrupt!!!\n", coreNum);
                    return;
                }
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
                sharedMemDataP->recordAfterWrite[i][interrupt_count[i]] = emucnt0_ptr[0];
#else
                /* Record timestamp diffs */
                record_latency(i, 0, pkt_seq_number, send_time_stamp);

#endif
#endif /* EMUCNT_PROFILE */
                interrupt_count[i]++;
            }
        }

        /* Check on all cores, if all packets received */
        for ( j=0; j < NUMBER_OF_CORES; j++)
        {
            if ( j == SYSINIT )
                continue;
            if( testMode == 1 )
            {
                if (j != TEST_MODE_1_SEND_CORE_NUMBER)
                    continue;
            }
#ifdef DEBUG
                System_printf ("core %d: Packets received from core %d: %d\n",
                   coreNum, j, packets_received[j]);
#endif
            if( packets_received[j] < numPacketsToSend ) break;
        }
    } while ( j < NUMBER_OF_CORES );
#ifdef EMUCNT_PROFILE
    interruptProcessEndTime = emucnt0_ptr[0];
    interruptProcessEndTime += ((unsigned long long)emucnt0_ptr[1] << 32);
#endif /* EMUCNT_PROFILE */

    System_printf ("Finished receiving interrupts: core %d\n", coreNum);
#ifdef EMUCNT_PROFILE
    System_printf ("Start time 0x%llx: End time 0x%llx core %d\n",
        interruptProcessStartTime, interruptProcessEndTime, coreNum);
    System_printf ("Total interrupt processing time %lld cycles: core %d\n",
        (interruptProcessEndTime-interruptProcessStartTime)*4, coreNum);
        /* Count all packets received */
        for ( j=0; j < NUMBER_OF_CORES; j++)
        {
            if ( j == SYSINIT )
                continue;
            if( testMode == 1 )
            {
                if (j != TEST_MODE_1_SEND_CORE_NUMBER)
                    continue;
            }
            totalPacketsReceived += packets_received[j];
        }
#endif /* EMUCNT_PROFILE */
    System_printf("core %d: got %d interrupts\n", coreNum, common_interrupt_counter);
}
/**
 *  @b Description
 *  @n
 *
 *      Used to send data from the produce core to te consumer core
 *      It performs the following
 *          - Opens transmit and receive channels
 *          - Opens transmit and receive queues
 *          - Programs receive flow
 *          - Programs accumulator
 *          - Sets transmit threshold
 *          - Gets a free transmit descriptor, initializes and pushes packet to transmit queue
 *  @retval
 *      Not Applicable.
 */
static Int32 send_data (uint32_t channel, uint32_t core)
{
    Cppi_Desc               *monoDescPtr;
    uint32_t                i;
#ifdef WITH_INFRADMA
    uint8_t                 isAllocated;
    Cppi_DescTag            tag;
    Qmss_Queue              queInfo;
    uint32_t                txChan, rxChan, flowId;
    uint32_t                rxQueNum;
    Qmss_QueueType          rxQueType;

    rxQueNum  = rxQIDByCore[core];
    rxQueType = QMSS_PARAM_NOT_SPECIFIED;

    /* Set up Tx Channel parameters */
    memset ((void *) &txChCfg, 0, sizeof (Cppi_TxChInitCfg));

    txChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    txChCfg.priority = 0;
    txChCfg.filterEPIB = 0;
    txChCfg.filterPS = 0;
    txChCfg.aifMonoMode = 0;
    txChCfg.txEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Tx Channel (specific number doesn't matter) */
    txChHnd = (Cppi_ChHnd) Cppi_txChannelOpen (cppiHnd, &txChCfg, &isAllocated);
    if (txChHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Tx channel\n", coreNum);
        return -1;
    }
    else
    {
        txChan = Cppi_getChannelNumber (txChHnd);
        System_printf ("Core %d : Opened Tx channel      : %d\n", coreNum, txChan);
    }

    /* Set up Rx Channel parameters */
    memset ((void *) &rxChCfg, 0, sizeof (Cppi_RxChInitCfg));
    rxChCfg.channelNum = CPPI_PARAM_NOT_SPECIFIED;
    rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

    /* Open Rx Channel (specific number doesn't matter) */
    rxChHnd = (Cppi_ChHnd) Cppi_rxChannelOpen (cppiHnd, &rxChCfg, &isAllocated);
    if (rxChHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Rx channel\n", coreNum);
        return -1;
    }
    else
    {
        rxChan = Cppi_getChannelNumber (rxChHnd);
        System_printf ("Core %d : Opened Rx channel      : %d\n", coreNum, rxChan);
    }

    /* Opens transmit queue. This is the infrastructure queue associated with the txChannel */
    if ((txQueHnd = Qmss_queueOpen (INFRASTRUCTURE_MULTICOREMODE_QUEUE_TYPE,
                                    QMSS_INFRASTRUCTURE_QUEUE_BASE + txChan, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Transmit Queue Number %d: %d\n", coreNum,
                       QMSS_INFRASTRUCTURE_QUEUE_BASE + txChan, txQueHnd);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Transmit Queue Number  : %d\n", coreNum, txQueHnd);
    }

    /* Opens receive queue - this depends on interrupt wiring on device (and channel) */
    if ((rxQueHnd = Qmss_queueOpen (rxQueType, rxQueNum, &isAllocated)) < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Receive Queue Number %d (%d)\n", coreNum, rxQueNum, rxQueHnd);
        return -1;
    }
    else
    {
        System_printf ("Core %d : Receive Queue Number   : %d\n", coreNum, rxQueHnd);
    }

    /* Set transmit queue threshold to high and when there is atleast one packet */
    /* Setting threshold on transmit queue is not required anymore. tx pending queue is not hooked to threshold.
     * Qmss_setQueueThreshold (txQueHnd, 1, 1);
     */

    /* Setup Rx flow parameters */
    memset ((void *) &rxFlowCfg, 0, sizeof (Cppi_RxFlowCfg));

    /* Configure flow 0 */
    rxFlowCfg.flowIdNum = CPPI_PARAM_NOT_SPECIFIED;
    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxQueHnd);
    rxFlowCfg.rx_dest_qnum = queInfo.qNum;
    rxFlowCfg.rx_dest_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_sop_offset = MONOLITHIC_DESC_DATA_OFFSET;
    rxFlowCfg.rx_desc_type = Cppi_DescType_MONOLITHIC;

    /* Get queue manager and queue number from handle */
    queInfo = Qmss_getQueueNumber (rxFreeQueHnd);
    rxFlowCfg.rx_fdq0_sz0_qnum = queInfo.qNum;
    rxFlowCfg.rx_fdq0_sz0_qmgr = queInfo.qMgr;
    rxFlowCfg.rx_error_handling = 1;

    /* Configure Rx flow */
    rxFlowHnd = (Cppi_FlowHnd) Cppi_configureRxFlow (cppiHnd, &rxFlowCfg, &isAllocated);
    if (rxFlowHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening Rx flow : %d\n", coreNum, rxFlowCfg.flowIdNum);
        return -1;
    }
    else
    {
        flowId = Cppi_getFlowId(rxFlowHnd);
        System_printf ("Core %d : Opened Rx flow         : %d\n", coreNum, flowId);
    }

    /* Enable transmit channel */
    if (Cppi_channelEnable (txChHnd) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Enabling Tx channel : %d\n", coreNum, Cppi_getChannelNumber (txChHnd));
        return -1;
    }

    /* Enable receive channel */
    if (Cppi_channelEnable (rxChHnd) != CPPI_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Enabling Rx channel : %d\n", coreNum, Cppi_getChannelNumber (rxChHnd));
        return -1;
    }
#endif
    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++)
        dataBuff[i] = i;

    System_printf ("\nCore %d : Transmitting %d packets..........\n\n", coreNum, numPacketsToSend);
    System_printf ("*************************************************************\n");

    /* Send out 8 packets */
    for (i = 0; i < numPacketsToSend; i++)
    {
        /* Get a free descriptor */
        while((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            retryCount++;
#ifdef DEBUG
            System_printf ("Core %d :No Free descriptor from Queue Number: %d"
                "for packet %d: Retrying\n"
                , coreNum, txFreeQueHnd, i);
#endif
            usleep(1);
        }
#ifdef DEBUG
        System_printf ("\nCore %d : Pop from txfreeq complete %d\n", coreNum, i);
#endif
#ifdef WITH_INFRADMA
        /* Add data buffer */
        Cppi_setData (Cppi_DescType_MONOLITHIC, monoDescPtr, (uint8_t *) &dataBuff, SIZE_DATA_BUFFER);

        /* Set tag information - this selects the flow */
        tag.destTagLo = 0;
        tag.destTagHi = 0;
        tag.srcTagLo = flowId;
        tag.srcTagHi = 0;

        Cppi_setTag (Cppi_DescType_MONOLITHIC, monoDescPtr, &tag);

        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_MONOLITHIC, monoDescPtr, SIZE_DATA_BUFFER);
#else
        /* Set packet length */
        Cppi_setPacketLen (Cppi_DescType_MONOLITHIC, monoDescPtr, 0);
#endif
        ((Cppi_MonolithicDesc *)monoDescPtr)->softwareInfo0 = i;

#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
        sharedMemDataP->recordPacketSendStart[coreNum][i] = emucnt0_ptr[0];
#else
        ((Cppi_MonolithicDesc *)monoDescPtr)->softwareInfo1 = emucnt0_ptr[0];
#endif
#endif

#ifdef WITH_INFRADMA
        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (uint32_t *) monoDescPtr, MONOLITHIC_DESC_DATA_OFFSET);
#else
        Qmss_queuePushDesc (qpendRxQueHnd, (uint32_t *) monoDescPtr);
#endif
#ifdef DEBUG
        System_printf ("\nCore %d : Push to queue complete %d\n", coreNum, i);
#endif
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
        sharedMemDataP->recordPacketSendEnd[coreNum][i] = emucnt0_ptr[0];
#endif
#endif
    if(delayBetweenPacketsInMicrosec)
        usleep(delayBetweenPacketsInMicrosec);
    }

    System_printf ("\nCore %d : Send %d packets complete..........\n", coreNum, numPacketsToSend);
    System_printf ("core %d: Retries %d \n\n", coreNum, retryCount);

    return 0;
}

/**
 *  @b Description
 *  @n
 *
 *      Task that executes the example code
 *      This is an example code that shows producer core sending data to consumer
 *      core. Synchronization between different cores.
 *
 *  @retval
 *      Not Applicable
 */
void usageTsk(Cppi_Handle hnd, int inputNumPacketsToSend,
        int inputDelayBetweenPacketsInMicrosec, int inputTestMode)
{
    uint32_t            channel, index;
    volatile uint32_t   count;
    Cppi_Desc           *desc;
    int i;
    uint8_t                 isAllocated;
    int result;
    struct sched_param param;
#ifdef EMUCNT_PROFILE
    unsigned measured_throughput;
#ifdef SHARED_MEMORY
    int oflags, perms;
    int mprot = PROT_READ | PROT_WRITE;      /* protection flags to mmap */
    int mflags = MAP_SHARED;    /* mmap flags */
    int ret;
#endif
#endif
    if (coreNum == SYSINIT)
    {
        /* Set Real time priority to this thread  */
        param.sched_priority = SYSINIT_SHED_PRIORITY;
        result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (result != 0)
        {
                System_printf ("Error: Unable to set"
                    " the UINTC thread priority & policy [Error Code %d]\n",
                    result);
                return;
        }
        System_printf ("Setting Reader thread to SCHED FIFO, Priority %d\n", param.sched_priority);
    }

    /* Set Global variables based on input paramaters */
    numPacketsToSend = inputNumPacketsToSend;
    delayBetweenPacketsInMicrosec = inputDelayBetweenPacketsInMicrosec;
    testMode = inputTestMode;
#ifdef SHARED_MEMORY
    if( numPacketsToSend > MAX_NUM_INTERRUPTS)
    {
        errorCount++;
        System_printf ("Error Core %d : Number of packets larger than expected\n", coreNum);
        return;
    }
#endif
#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
    oflags = O_CREAT | O_RDWR;
    perms = S_IRUSR | S_IWUSR;
    shm_fd = shm_open(SHARED_MEMORY_OBJECT_NAME, oflags, perms);
    if (shm_fd == -1)
    {
        errorCount++;
        System_printf ("Error Core %d : Opening shared memory object\n", coreNum);
        return;
    }

    ret = ftruncate(shm_fd,sizeof(sharedMemData_t));
    if( ret != 0) {
        errorCount++;
        System_printf ("Error Core %d : Truncate shared memory file object failed\n", coreNum);
        return;
    }

    sharedMemDataP = mmap(NULL,
        sizeof(sharedMemData_t),
        mprot, mflags, shm_fd, (off_t)0);

    if( sharedMemDataP == MAP_FAILED)
    {
        errorCount++;
        System_printf ("Error Core %d : Mapping shared memory object\n", coreNum);
        return;
    }
#endif
    fd_mem = open("/dev/mem", (O_RDWR | O_SYNC));
    if (fd_mem < 0) {
        printf("\n Error opening device ");
        return;
    }
    /* Get user space mapping for emucnt0 */
    emucnt0_ptr  = (uint32_t *)map_dev_mem((uint32_t)(&pllcRegs->EMUCNT0), 8);
    if (emucnt0_ptr == NULL)
    {
        printf("\n Error with mmaping emucnt ");
        return;
    }
    /* Write emucnt0 to start counter */
    *(emucnt0_ptr) = 1;

#endif

    cppiHnd = hnd;

    /* Opens receive queue */
    if ((qpendFd = acquireUioRxQueue ("qpend", &qpendRxQueHnd)) < 0)
    {
        System_printf ("Error Core %d : cant find qpend queue in uio/proc (%d) \n", coreNum, qpendFd);
        errorCount++;
        return;
    }
    else
    {
        System_printf ("core %d: my rx queue: %d\n", coreNum, qpendRxQueHnd);
    }

    rxQIDByCore[coreNum] = Qmss_getQIDFromHandle(qpendRxQueHnd);

    /* Core 0 is treated as the producer core that
     * Initializes the system
     * setups the configuration
     * Pushes the data
     * De-initializes the system
     */

    /* Initializes the system */
    if (coreNum == SYSINIT)
    {
        if (sysInit () < 0)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing QMSS\n", coreNum);
            return;
        }
    }
    else
    {
        /* Get the handle for common queues on consumer cores */
        getsysHandles ();
    }

    for (channel = 0; channel < NUM_ITERATION; channel += NUMBER_OF_CORES)
    {
        /* Sync up all the cores after configuration is completed */
        if (coreNum == SYSINIT)
        {
            count = Qmss_getQueueEntryCount (syncCfgQueHnd);
            while (count != NUMBER_OF_CORES - 1)
            {
                yield();
                count = Qmss_getQueueEntryCount (syncCfgQueHnd);
            }
            /* put sync descs back on free queue */
            while ((desc = (Cppi_Desc *) Qmss_queuePop (syncCfgQueHnd)) != NULL)
            {
                /* Push descriptor to sync free queue */
                Qmss_queuePushDesc (syncFreeQueHnd, (uint32_t *) desc);
            }
            System_printf ("Core %d :Initial sync complete\n", coreNum);
        }
        else
        {
            do {
                if ((desc = (Cppi_Desc *) Qmss_queuePop (syncFreeQueHnd)) != NULL)
                {
                    /* Push descriptor to sync cfg queue */
                    Qmss_queuePushDesc (syncCfgQueHnd, (uint32_t *) desc);
                }
                else
                {
                    System_printf ("NOTE Core %d : No SYNC descriptor (waiting for first core to start)\n", coreNum);
                    yield();
                }
            } while (desc == NULL);
            System_printf ("Core %d :Initial sync complete\n", coreNum);
        }

        if (coreNum == SYSINIT)
        {
            for (i=0; i < NUMBER_OF_CORES; i++) {
                if (i == SYSINIT) continue;
                qpendUioFd[i] = openUioForQID(rxQIDByCore[i]);
                qpendRxQueHndArray[i] = Qmss_queueOpen (QMSS_PARAM_NOT_SPECIFIED, rxQIDByCore[i], &isAllocated);
                if (qpendRxQueHndArray[i] < 0)
                {
                    errorCount++;
                    System_printf ("Error Core %d : Opening Rx channel\n", coreNum);
                    return;
                }
                System_printf ("Opened %d: qpendUioFd[%d]: queueNum %d\n", qpendRxQueHndArray[i], i, rxQIDByCore[i]);
            }
            /* Recive data */
            receive_data ();
        }
        else
        {
            /* Wait for receive to get ready */
            sleep(5);

            if((( testMode == 1)
                && ( coreNum == TEST_MODE_1_SEND_CORE_NUMBER )) || (testMode != 1))
            {
                if (send_data (coreNum , coreNum) < 0)
                {
                    errorCount++;
                    System_printf ("Error Core %d : Sending data on channel %d core: %d\n", coreNum, channel + coreNum, coreNum);
                    return;
                }
                System_printf ("Send Data complete. core: %d\n", coreNum);
            }
       }

       /* Send the sync signal */
        if ((desc = Qmss_queuePop (syncFreeQueHnd)) != NULL)
        {
            Qmss_queuePushDesc (syncQueHnd, desc);
        }
        else
        {
            errorCount++;
            System_printf ("Error Core %d : No SYNC descriptor!!!\n", coreNum);
        }

        System_printf ("Core %d : Waiting for sync signal\n", coreNum);

        count = Qmss_getQueueEntryCount (syncQueHnd);
        while (count < NUMBER_OF_CORES)
        {
            yield();
            usleep(delayBetweenPacketsInMicrosec);
            count = Qmss_getQueueEntryCount (syncQueHnd);
        }

        System_printf ("Core %d : Got sync signal\n", coreNum);

        if (coreNum == SYSINIT)
        {
            for (i=0; i < NUMBER_OF_CORES; i++) {
                if (i == SYSINIT) continue;
                close(qpendUioFd[i]);
                System_printf ("Core %d : Closing %d\n", coreNum, qpendRxQueHndArray[i]);fflush(stdout);
               if ((result = Qmss_queueClose (qpendRxQueHndArray[i])) < QMSS_SOK)
                {
                    System_printf ("Error Core %d : Closing qpendRxQueHnd %d error code : %d\n",
                        coreNum, qpendRxQueHndArray[i], result);
                    errorCount++;
                }
            }
            System_printf ("Core %d : Closing of queues  complete\n", coreNum);fflush(stdout);
        }
        else
        {
#ifdef WITH_INFRADMA
            if((( testMode == 1)
                && ( coreNum == TEST_MODE_1_SEND_CORE_NUMBER )) || (testMode != 1))
            {
                System_printf ("Cleanup Start. core: %d\n", coreNum);fflush(stdout);
                /* Close Rx/Tx queues, channels and flows used in this data transfer */
                cleanup ();
                System_printf ("Cleanup complete. core: %d\n", coreNum);fflush(stdout);
            }
#endif
        }

        /* Put one descriptor on the syncQueCfgHnd to acknowledge the sync */
        if ((desc = (Cppi_Desc *) Qmss_queuePop (syncFreeQueHnd)) != NULL)
        {
            /* Push descriptor to sync free queue */
            Qmss_queuePushDesc (syncCfgQueHnd, (uint32_t *) desc);
        }
        else
        {
            errorCount++;
            System_printf ("Error Core %d : No SYNC descriptor!!!\n", coreNum);
        }


        System_printf ("*************************************************************\n\n");

        /* wait for "config" sync to know all cores have received previous sync */
        if (coreNum == SYSINIT)
        {
            System_printf ("Core %d : Waiting for other cores to ack sync signal\n", coreNum);
            /* Wait for all other cores to finish */
            count = Qmss_getQueueEntryCount (syncCfgQueHnd);
            while (count < NUMBER_OF_CORES)
            {
                yield();
                count = Qmss_getQueueEntryCount (syncCfgQueHnd);
            }
            System_printf ("Core %d : acks found\n", coreNum);

            for (index = 0; index < NUMBER_OF_CORES; index++)
            {
                if ((desc = (Cppi_Desc *) Qmss_queuePop (syncQueHnd)) != NULL)
                {
                    /* Push descriptor to sync free queue */
                    Qmss_queuePushDesc (syncFreeQueHnd, (uint32_t *) desc);
                }
                if ((desc = (Cppi_Desc *) Qmss_queuePop (syncQueHnd)) != NULL)
                {
                    /* Push descriptor to sync free queue */
                    Qmss_queuePushDesc (syncFreeQueHnd, (uint32_t *) desc);
                }
            }
        }
    }

    /* De-initializes the system */
    sysExit ();

#ifdef EMUCNT_PROFILE
#ifdef SHARED_MEMORY
    if (coreNum == SYSINIT)
    {
        for(i=0; i< NUMBER_OF_CORES; i++) {
            if ( i == SYSINIT ) continue;
            print_latency_stats(i, sharedMemDataP->recordPacketSendEnd[i],
                sharedMemDataP->recordIntReceive[i], "Latency");
            print_latency_stats(i, sharedMemDataP->recordPacketSendStart[i],
                sharedMemDataP->recordPacketSendEnd[i], "PktSend");
            print_latency_stats(i, sharedMemDataP->recordIntReceive[i],
                sharedMemDataP->recordIntReceiveSecond[i], "Dummy");
            print_latency_stats(i, sharedMemDataP->recordIntReceiveSecond[i],
                sharedMemDataP->recordAfterProcess[i], "Process");
            print_latency_stats(i, sharedMemDataP->recordAfterProcess[i],
                sharedMemDataP->recordAfterRead[i], "Read");
            print_latency_stats(i, sharedMemDataP->recordAfterRead[i],
                sharedMemDataP->recordAfterWrite[i], "Write");
        }
    }
    if(munmap(sharedMemDataP, sizeof(sharedMemData_t)))
    {
        errorCount++;
        System_printf ("Error Core %d : Error unmap shared Data area \n", coreNum);
    }
    if (coreNum == SYSINIT)
    {
        if(shm_unlink(SHARED_MEMORY_OBJECT_NAME))
        {
            errorCount++;
            System_printf ("Error Core %d : Error Unlink shared Memory\n", coreNum);
        }
    }
#else /* SHARED_MEMORY */
    /* Print stats here */
    if (coreNum == SYSINIT)
    {
        for(i=0; i< NUMBER_OF_CORES; i++) {
            if ( i == SYSINIT ) continue;

            if((testMode == 1 ) && (i != TEST_MODE_1_SEND_CORE_NUMBER)) continue;

            print_latency_stats(i, 0, "Latency", 1);
            print_latency_stats(i, 0, "Latency", 0);
        }
    }
#endif /* SHARED_MEMORY */
    if (coreNum == SYSINIT)
    {
        /* Throughput measured = (packets * Cycles/sec multiplier) / measuredtime in cycles */
        measured_throughput = (unsigned)((((unsigned long long)totalPacketsReceived
                                *THROUGHPUT_MULTIPLIER)
                                /(interruptProcessEndTime-interruptProcessStartTime)));
        System_printf("core %d: Total packets received %d:"
            "Throughput %d Packets/s \n",
            coreNum, totalPacketsReceived,
            measured_throughput);
    }
#endif /* EMUCNT_PROFILE */

    if (!errorCount)
    {
        System_printf ("*******************************************************\n");
        System_printf ("******** QMSS Multicore (%d) Example Done (PASS) *******\n", coreNum);
        System_printf ("*******************************************************\n");
        if (retryCount)
            System_printf ("******** Warning: (%d) Retry count %d ******\n", coreNum, retryCount);

    }
    else
    {
        System_printf ("core %d: QMSS example failed: %d errors\n", coreNum, errorCount);
    }
}
