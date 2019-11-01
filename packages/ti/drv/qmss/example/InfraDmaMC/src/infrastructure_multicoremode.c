/**
 *   @file  infrastructure_multicoremode.c
 *
 *   @brief   
 *      This is the QMSS infrastructure mode example code that can run on multiple cores. 
 *      Accumulator used to interrupt different cores they are mapped to.
 *      Shows synchronization uses queues
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2014, Texas Instruments, Inc.
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
/* XDC include */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <xdc/cfg/global.h>

/* SYSBIOS include */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>

/* IPC include */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>
#else
#include <sched.h>
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
#include "infrastructure_multicoremode_cfg.h"

/* Device specific include */
#include "qmssPlatCfg.h"

/* RM include */
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_services.h>

/* Queue bases */
#include <ti/csl/csl_qm_queue.h>

#ifndef __LINUX_USER_SPACE
/* CSL RL includes */
#include <ti/csl/csl_chip.h>
#include <csl_intc.h>
#include <csl_intcAux.h>
#include <ti/csl/csl_cacheAux.h>

/************************ USER DEFINES ********************/

#define MAPPED_VIRTUAL_ADDRESS      0x81000000


/* MPAX segment 2 registers */
#define XMPAXL2                     0x08000010 
#define XMPAXH2                     0x08000014


/************************ GLOBAL VARIABLES ********************/
#pragma DATA_ALIGN (linkingRAM0, 16)
uint64_t                linkingRAM0[NUM_MONOLITHIC_DESC + NUM_SYNC_DESC];
/* Descriptor pool [Size of descriptor * Number of descriptors] */
#pragma DATA_ALIGN (monolithicDesc, 16)
uint8_t                 monolithicDesc[SIZE_MONOLITHIC_DESC * NUM_MONOLITHIC_DESC];
#pragma DATA_ALIGN (syncDesc, 16)
uint8_t                 syncDesc[SIZE_SYNC_DESC * NUM_SYNC_DESC];
#pragma DATA_ALIGN (dataBuff, 16)
uint8_t                 dataBuff[SIZE_DATA_BUFFER];
/* List address for accumulator - twice the number of entries for Ping and Pong page */
#pragma DATA_ALIGN (hiPrioList, 16)
uint32_t                hiPrioList[34];
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
/* Memory region configuration information */
Qmss_MemRegInfo         memInfo;

#ifndef __LINUX_USER_SPACE
/* Error counter */
uint32_t                errorCount = 0;
/* QMSS configuration */
Qmss_InitCfg            qmssInitConfig;
/* Handle to CPPI heap */
IHeap_Handle            cppiHeap;
/* Accumulator configuration */
Qmss_AccCmdCfg          cfg;

CSL_IntcEventHandlerRecord  Record[2];
CSL_IntcParam               vectId;
CSL_IntcObj                 hiPrioIntcObj;
CSL_IntcHandle              hiPrioIntcHnd;
CSL_IntcContext             context;

#pragma DATA_SECTION (isQMSSInitialized, ".qmss");
volatile uint32_t           isQMSSInitialized;

#endif

#ifndef __LINUX_USER_SPACE
#pragma DATA_SECTION (runCount, ".qmss");
#endif
uint32_t                    runCount;

#if RM && ! defined(__LINUX_USER_SPACE)
/* RM initialization sync point */
#pragma DATA_SECTION (isRmInitialized, ".rm");
volatile uint32_t           isRmInitialized;

/* RM instance handle */
Rm_Handle                   rmHandle = NULL;
Rm_ServiceHandle           *rmClientServiceHandle = NULL;
#endif /* RM */

/* Indicates the core or logical task ID test is running on */
uint32_t                coreNum;
/* Regions returned my Qmss_insertMemoryRegion */
Qmss_Result             reg1, reg2;

Cppi_Handle             cppiHnd;
Cppi_ChHnd              rxChHnd, txChHnd;
Qmss_QueueHnd           txQueHnd, rxQueHnd, rxFreeQueHnd, txCmplQueHnd, txFreeQueHnd, syncQueHnd, syncFreeQueHnd, syncCfgQueHnd;
#ifdef __LINUX_USER_SPACE
int32_t                 rxQIDByCore[NUMBER_OF_CORES];
Qmss_QueueHnd           qpendRxQueHnd;
int                     qpendFd;
#endif
#if RM
int32_t                 CPPI_FREE_TX_QID, CPPI_FREE_RX_QID, CPPI_COMPLETION_QID, QMSS_SYNC_CFG_QID, QMSS_SYNC_QID, QMSS_FREE_SYNC_QID;
#endif

Cppi_DescCfg            descCfg;
Qmss_DescCfg            syncDescCfg;
Cppi_FlowHnd            rxFlowHnd;

#ifndef __LINUX_USER_SPACE
#if RM
/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
/* RM instance transport code */
extern int setupRmTransConfig(uint32_t numTestCores, uint32_t systemInitCore, Task_FuncPtr testTask);

#endif /* RM */
/************************ EXTERN VARIABLES ********************/

/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;
/* CPPI device specific configuration */
extern Cppi_GlobalConfigParams  cppiGblCfgParams;

/*************************** FUNCTIONS ************************/

void usageTsk(UArg arg0, UArg arg1);

/**
 *  @b Description
 *  @n
 *      The function is used to get the handle to the CPPI memory heap.
 *      If the application is run on a multiple cores then place the CPPI global 
 *      variables in shared memory.  
 *
 *  @retval
 *      Not Applicable
 */
static void cppiHeapInit ()
{
    cppiHeap = HeapMem_Handle_upCast (cppiSharedHeap);
}
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
    /* Compute the global address. */
    return (addr + (0x10000000 + (coreNum * 0x1000000)));
}
#else
static uint32_t l2_global_address (uint32_t addr)
{
    return addr;
}
#endif

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
#ifdef __LINUX_USER_SPACE
//    sleep(0);
    sched_yield();
#endif
}

#if RM

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
#endif

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

    result = Qmss_getQueueEntryCount (rxFreeQueHnd);
    System_printf ("Rx Free Queue %d Entry Count            : %d \n", rxFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (txCmplQueHnd);
    System_printf ("Tx completion Queue %d Entry Count     : %d \n", txCmplQueHnd, result);

    result = Qmss_getQueueEntryCount (syncQueHnd);
    System_printf ("Sync Queue %d Entry Count              : %d \n", syncQueHnd, result);

    result = Qmss_getQueueEntryCount (syncFreeQueHnd);
    System_printf ("Sync free Queue %d Entry Count         : %d \n", syncFreeQueHnd, result);

    result = Qmss_getQueueEntryCount (syncCfgQueHnd);
    System_printf ("Sync Cfg Queue %d Entry Count          : %d \n", syncCfgQueHnd, result);

    System_printf ("-------------------------------------------------------------\n\n");  
}

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
#ifndef __LINUX_USER_SPACE
    Qmss_Result             result;
#else
    int                     i;
#endif
    Qmss_Queue              queInfo;
    uint32_t                numAllocated; 
    uint8_t                 isAllocated;
#ifdef L2_CACHE
    uint32_t                *xmpaxPtr;
#endif
#if RM && !defined(__LINUX_USER_SPACE)
    int32_t                 rmResult;
    Cppi_StartCfg           cppiStartCfg;
#endif /* RM */

    System_printf ("\n-----------------------Initializing---------------------------\n");

#ifndef __LINUX_USER_SPACE
    /* Reset the variable to indicate to other cores system init is not yet done */
    isQMSSInitialized = 0;
#endif

#if RM
    /* Set QID to -1 to let RM allocate */
    CPPI_FREE_TX_QID    = QMSS_PARAM_NOT_SPECIFIED;
    CPPI_FREE_RX_QID    = QMSS_PARAM_NOT_SPECIFIED;
    CPPI_COMPLETION_QID = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_SYNC_CFG_QID   = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_SYNC_QID       = QMSS_PARAM_NOT_SPECIFIED;
    QMSS_FREE_SYNC_QID  = QMSS_PARAM_NOT_SPECIFIED;
#endif

    /* Initilaize the number of times the test was run to zero */
    runCount = 0;

#ifndef __LINUX_USER_SPACE
#if RM
    rmClientServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", coreNum, rmResult);
        errorCount++;
        return -1;
    } 
#endif /* RM */

    /* Initialize the heap in shared memory for CPPI data structures */ 
    cppiHeapInit ();

#ifdef L2_CACHE
    /* Set L2 cache to 512KB */
    CACHE_setL2Size (CACHE_512KCACHE);
#endif
    
    System_printf ("Core %d : L1D cache size %d. L2 cache size %d.\n", coreNum, CACHE_getL1DSize(), CACHE_getL2Size());

#ifdef L2_CACHE
    /* Define an MPAX segment in the virtual (CGEM) address space. 
     * Map MSMC physical address to virtual address.
     * Configure all +rwx permissions.
     */

    /* Phy address base: 0x0C00 0000
     * Size: 1MB (0x13 according to encoding)
     * Virtual address: 0x8100 0000 
     * Permission: 0xFF
     * MAR used: (0x8100 0000 >> 24) = 129
     */

    /* map using MPAX segment 2 registers */
    xmpaxPtr  = (uint32_t *)(XMPAXH2);
    *xmpaxPtr = ((MAPPED_VIRTUAL_ADDRESS >> 12) << 12) | (0x13);

    xmpaxPtr  = (uint32_t *)(XMPAXL2);
    *xmpaxPtr = ((0x0c000000 >> 12) << 8) | (0xFF);
    
    /* Enable caching for MAR 129. CSL does not define these MARS. Define a macro */
    CACHE_enableCaching ((MAPPED_VIRTUAL_ADDRESS) >> 24);
#endif

    memset ((void *) &qmssInitConfig, 0, sizeof (Qmss_InitCfg));
    memset ((void *) &linkingRAM0, 0, sizeof (linkingRAM0));
    /* Set up the linking RAM. Use the internal Linking RAM. 
     * LLD will configure the internal linking RAM address and maximum internal linking RAM size if 
     * a value of zero is specified.
     * Linking RAM1 is not used */

#ifdef INTERNAL_LINKING_RAM 
    qmssInitConfig.linkingRAM0Base = 0;
    qmssInitConfig.linkingRAM0Size = 0;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC + NUM_SYNC_DESC;
#else
    qmssInitConfig.linkingRAM0Base = (uint32_t) l2_global_address((uint32_t)&linkingRAM0[0]);
    qmssInitConfig.linkingRAM0Size = 0x3FFF;
    qmssInitConfig.linkingRAM1Base = 0;
    qmssInitConfig.maxDescNum      = NUM_MONOLITHIC_DESC + NUM_SYNC_DESC;
#endif

#ifdef xdc_target__bigEndian
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_be;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_be);
#else
    qmssInitConfig.pdspFirmware[0].pdspId = Qmss_PdspId_PDSP1;
    qmssInitConfig.pdspFirmware[0].firmware = (void *) &acc48_le;
    qmssInitConfig.pdspFirmware[0].size = sizeof (acc48_le);
#endif

#if RM
    qmssGblCfgParams.qmRmServiceHandle = rmClientServiceHandle;
#endif
    /* Initialize Queue Manager SubSystem */
    result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
    if (result != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing Queue Manager SubSystem error code : %d\n", coreNum, result);
        return -1;
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

#if RM
    /* Register RM with CPPI */
    cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
    Cppi_startCfg(&cppiStartCfg);
#endif /* RM */

    /* Set up QMSS CPDMA configuration */
    memset ((void *) &cpdmaCfg, 0, sizeof (Cppi_CpDmaInitCfg));
    cpdmaCfg.dmaNum = INFRASTRUCTURE_MULTICOREMODE_CPDMA_TYPE;

    /* Open QMSS CPDMA */
    cppiHnd = (Cppi_Handle) Cppi_open (&cpdmaCfg);
    if (cppiHnd == NULL)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing QMSS CPPI CPDMA %d\n", coreNum, cpdmaCfg.dmaNum);
        return -1;
    }
#endif
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
#if RM
        /* Save the QID received from RM */
        CPPI_COMPLETION_QID = Qmss_getQIDFromHandle (txCmplQueHnd);
#endif
    }

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
    
    /* Descriptor should be recycled back to Queue Number 1000 */
    queInfo = Qmss_getQueueNumber (txCmplQueHnd);
    descCfg.returnQueue.qMgr = queInfo.qMgr;
    descCfg.returnQueue.qNum = queInfo.qNum;

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
#if RM
        /* Save the QID received from RM */
        CPPI_FREE_TX_QID = Qmss_getQIDFromHandle (txFreeQueHnd);
#endif
    }

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
#if RM
        /* Save the QID received from RM */
        CPPI_FREE_RX_QID = Qmss_getQIDFromHandle (rxFreeQueHnd);
#endif
    }

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
#if RM
        /* Save the QID received from RM */
        QMSS_FREE_SYNC_QID = Qmss_getQIDFromHandle (syncFreeQueHnd);
#endif

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
#if RM
        /* Save the QID received from RM */
        QMSS_SYNC_QID = Qmss_getQIDFromHandle (syncQueHnd);
#endif
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
#if RM
        /* Save the QID received from RM */
        QMSS_SYNC_CFG_QID = Qmss_getQIDFromHandle (syncCfgQueHnd);
#endif
    }

    /* Opens sync free queue. */

    System_printf ("Core %d : Sync Free Queue Number         : %d opened\n", coreNum, syncFreeQueHnd);
    System_printf ("Core %d : Receive Free Queue Number      : %d opened\n", coreNum, rxFreeQueHnd);
    System_printf ("Core %d : Transmit Free Queue Number     : %d opened\n", coreNum, txFreeQueHnd);

    System_printf ("Core %d : System initialization completed: %d\n", coreNum, txFreeQueHnd);

#ifndef __LINUX_USER_SPACE
    /* Indicate to other cores system init is done */
    isQMSSInitialized = 1;

#ifdef L2_CACHE
    /* Writeback L2 */
    CACHE_wbL2 ((void *) &isQMSSInitialized, 4, CACHE_WAIT);
#else
    /* Writeback L1D */
    CACHE_wbL1d ((void *) &isQMSSInitialized, 4, CACHE_WAIT);
#endif
#endif

#if RM
    System_printf ("Core %d : Publishing RM nameserver names for shared queues\n", coreNum);
    ex_rm_name_set ("INFRAMC_FREE_TX",    CPPI_FREE_TX_QID);
    ex_rm_name_set ("INFRAMC_FREE_RX",    CPPI_FREE_RX_QID);
    ex_rm_name_set ("INFRAMC_COMPLETION", CPPI_COMPLETION_QID);
    ex_rm_name_set ("INFRAMC_SYNC_CFG",   QMSS_SYNC_CFG_QID);
    ex_rm_name_set ("INFRAMC_SYNC",       QMSS_SYNC_QID);
    ex_rm_name_set ("INFRAMC_SYNC_FREE",  QMSS_FREE_SYNC_QID);
#ifdef __LINUX_USER_SPACE
    /* Get rest of cores' receive queues */
    System_printf ("Core %d: Getting receive queues for other cores\n", coreNum);
    for (i = 1; i < NUMBER_OF_CORES; i++)
    {
        char corename[32];
        snprintf (corename, sizeof(corename), "INFRAMC_CORE%d_RXQ", i);
        rxQIDByCore[i] = ex_rm_name_lookup (corename);
        printf ("core %d: Got core %d's RX QID: %d\n", coreNum, i, rxQIDByCore[i]);
    }
#endif
#endif
    printQueueStats ("After Initialization");
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
#ifndef __LINUX_USER_SPACE
    volatile Qmss_Result    result;
#if RM
    int32_t                 rmResult;
    Qmss_StartCfg           qmssStartCfg;
    Cppi_StartCfg           cppiStartCfg;
#endif /* RM */
    
    /* Start Queue Manager SubSystem */
    System_printf ("Core %d : Waiting for QMSS to be initialized...\n\n", coreNum);

    /* Synchronize all consumer cores. They must wait for the producer core to finish initialization. */
    do{
#ifdef L2_CACHE
        /* Invalidate L2 */
        CACHE_invL2 ((void *) &isQMSSInitialized, 4, CACHE_WAIT);
#else
        CACHE_invL1d ((void *) &isQMSSInitialized, 4, CACHE_WAIT);
#endif
    } while (isQMSSInitialized == 0);

    System_printf ("\nCore %d : QMSS initialization done.\n\n", coreNum);

#if RM
    rmClientServiceHandle = Rm_serviceOpenHandle(rmHandle, &rmResult);
    if (rmResult != RM_OK)
    {
        errorCount++;
        System_printf ("Error Core %d : Creating RM service handle error code : %d\n", coreNum, rmResult);
        return;
    } 

    /* Start Queue Manager SubSystem with RM */
    qmssStartCfg.rmServiceHandle = rmClientServiceHandle;
    qmssStartCfg.pQmssGblCfgParams = &qmssGblCfgParams;
    result = Qmss_startCfg (&qmssStartCfg);
#else       
    /* Start Queue Manager SubSystem */
    result = Qmss_start ();    
#endif /* RM */
    if (result != QMSS_SOK)
    {
        errorCount++;
        System_printf ("Core %d : Error starting Queue Manager error code : %d\n", coreNum, result);
        return;
    }

#if RM
    /* Register RM with CPPI */
    cppiStartCfg.rmServiceHandle = rmClientServiceHandle;
    Cppi_startCfg(&cppiStartCfg);
#endif /* RM */
#endif

#if RM
    /* Interrogate the queue IDs */
    CPPI_FREE_TX_QID    = ex_rm_name_lookup ("INFRAMC_FREE_TX");
    CPPI_FREE_RX_QID    = ex_rm_name_lookup ("INFRAMC_FREE_RX");
    CPPI_COMPLETION_QID = ex_rm_name_lookup ("INFRAMC_COMPLETION");
    QMSS_SYNC_CFG_QID   = ex_rm_name_lookup ("INFRAMC_SYNC_CFG");
    QMSS_SYNC_QID       = ex_rm_name_lookup ("INFRAMC_SYNC");
    QMSS_FREE_SYNC_QID  = ex_rm_name_lookup ("INFRAMC_SYNC_FREE");
#endif

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

#ifdef __LINUX_USER_SPACE
    /* Share my receive queue */
    {
        char corename[32];
        snprintf (corename, sizeof(corename), "INFRAMC_CORE%d_RXQ", coreNum);
        ex_rm_name_set (corename, Qmss_getQIDFromHandle(qpendRxQueHnd));
    }
#endif

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

#ifndef __LINUX_USER_SPACE
    uint8_t         channel;    
    if (coreNum == SYSINIT)
    {
        System_printf ("\n--------------------Deinitializing---------------------------\n");

        printQueueStats ("Before exit");

        for (channel = 0; channel < NUMBER_OF_CORES; channel++) 
        {
            /* Disable accumulator */
            if ((qmss_result = Qmss_disableAccumulator (Qmss_PdspId_PDSP1, channel)) != QMSS_ACC_SOK)
            {
                errorCount++;
                System_printf ("Error Core %d : Disabling high priority accumulator for channel : %d error code: %d\n",
                               coreNum, channel, qmss_result);
            }
        }
    }
#endif

    /* Drop the descriptors */
    if (coreNum == SYSINIT)
    {
        Qmss_queueEmpty (rxFreeQueHnd);
        Qmss_queueEmpty (txCmplQueHnd);
        Qmss_queueEmpty (txFreeQueHnd);
        Qmss_queueEmpty (syncQueHnd);
        Qmss_queueEmpty (syncFreeQueHnd);
        Qmss_queueEmpty (syncCfgQueHnd);
    }

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
#ifdef __LINUX_USER_SPACE
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
#endif

#ifndef __LINUX_USER_SPACE
    if (coreNum == SYSINIT)
#endif
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
#ifdef __LINUX_USER_SPACE
    System_printf ("Core %d: exit QMSS\n", coreNum);
    if ((qmss_result = Qmss_exit ()))
    {
        errorCount++;
        System_printf ("Error Core %d : exit error code : %d\n", coreNum, qmss_result);
    }
#else
    if (coreNum == SYSINIT)
    {
        System_printf ("Core %d: exit QMSS\n", coreNum);
        while ((qmss_result = Qmss_exit()) != QMSS_SOK)
        {
            yield(); /* Wait for other cores to close their queues */
        }
    }
#endif
#if RM
    if (coreNum == SYSINIT)
    {
        System_printf ("Core %d : Deleting RM nameserver names for shared queues\n", coreNum);
        ex_rm_name_del ("INFRAMC_FREE_TX");
        ex_rm_name_del ("INFRAMC_FREE_RX");
        ex_rm_name_del ("INFRAMC_COMPLETION");
        ex_rm_name_del ("INFRAMC_SYNC_CFG");
        ex_rm_name_del ("INFRAMC_SYNC");
        ex_rm_name_del ("INFRAMC_SYNC_FREE");
    }
#endif
}

#ifdef __LINUX_USER_SPACE
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
    void         *desc;
    int           index;
    int           packetsLeft = NUM_PACKETS;
    int           packetsToProcess = 0;

    System_printf("core %d: waiting for packets on %d\n", coreNum, qpendRxQueHnd);

    while (packetsLeft)
    {
        waitForInterrupt (qpendFd);
        packetsToProcess = Qmss_getQueueEntryCount (qpendRxQueHnd);
        System_printf ("core %d: got interrupt - process %d\n", coreNum, packetsToProcess);
    
        /* Recycle Rx BDs */
        for (index = 0; index < packetsToProcess; index++)
        {
            desc = Qmss_queuePop (qpendRxQueHnd);
            Qmss_queuePushDesc (rxFreeQueHnd, desc);
        }

        packetsLeft -= packetsToProcess;
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

    System_printf("core %d: got %d packets\n", coreNum, NUM_PACKETS);
}
#endif
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
    uint8_t                 isAllocated;
#ifndef __LINUX_USER_SPACE
    Qmss_Result             result;
#endif
    Cppi_DescTag            tag;
    Qmss_Queue              queInfo;
    uint32_t                txChan, rxChan, flowId;
    uint32_t                rxQueNum;
    Qmss_QueueType          rxQueType;

#ifdef __LINUX_USER_SPACE
    rxQueNum  = rxQIDByCore[core];
    rxQueType = QMSS_PARAM_NOT_SPECIFIED;
#else
    rxQueNum  = QMSS_HIGH_PRIORITY_QUEUE_BASE + channel;
    rxQueType = Qmss_QueueType_HIGH_PRIORITY_QUEUE;
#endif
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

#ifndef __LINUX_USER_SPACE
    /* program the high priority accumulator */
    memset ((void *) &hiPrioList, 0, sizeof (hiPrioList));

    cfg.channel = channel;
    cfg.command = Qmss_AccCmd_ENABLE_CHANNEL;
    cfg.queueEnMask = 0;
    cfg.listAddress = l2_global_address ((uint32_t) hiPrioList);
    /* Get queue manager and queue number from handle */
    cfg.queMgrIndex = Qmss_getQIDFromHandle (rxQueHnd);
    cfg.maxPageEntries = NUM_PACKETS + 1;
    cfg.timerLoadCount = 0;
    cfg.interruptPacingMode = Qmss_AccPacingMode_NONE;
    cfg.listEntrySize = Qmss_AccEntrySize_REG_D;
    cfg.listCountMode = Qmss_AccCountMode_ENTRY_COUNT;
    cfg.multiQueueMode = Qmss_AccQueueMode_SINGLE_QUEUE;
    
    if ((result = Qmss_programAccumulator (Qmss_PdspId_PDSP1, &cfg)) != QMSS_ACC_SOK)
    {
        errorCount++;
        System_printf ("Error Core %d : Programming high priority accumulator for channel : %d queue : %d error code : %d\n",
                        coreNum, cfg.channel, cfg.queMgrIndex, result);
        return -1;
    }
#endif

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

#ifndef __LINUX_USER_SPACE
    System_printf ("Core %d : High priority accumulator programmed for channel : %d queue : %d\n", 
                        coreNum, cfg.channel, cfg.queMgrIndex);
#endif

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

    /* Fill in some data */
    for (i = 0; i < SIZE_DATA_BUFFER; i++) 
        dataBuff[i] = i;

    System_printf ("\nCore %d : Transmitting %d packets..........\n\n", coreNum, NUM_PACKETS);
    System_printf ("*************************************************************\n");  
    
    /* Send out 8 packets */
    for (i = 0; i < NUM_PACKETS; i++)
    {
        /* Get a free descriptor */
        if ((monoDescPtr = (Cppi_Desc *) Qmss_queuePop (txFreeQueHnd)) == NULL)
        {
            errorCount++;
            System_printf ("Error Core %d : Getting descriptor from Queue Number: %d\n", coreNum, txFreeQueHnd);
            return -1;
        }

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

        /* Push descriptor to Tx queue */
        Qmss_queuePushDescSize (txQueHnd, (uint32_t *) monoDescPtr, MONOLITHIC_DESC_DATA_OFFSET);
    }
    
    return 0;
}

#ifndef __LINUX_USER_SPACE
/**
 *  @b Description
 *  @n  
 *
 *      ISR function process the received packets by reading the accumulator list. It recycles the received BDs.
 *      Send the sync signal.
 *  @retval
 *      Not Applicable.
 */
void hiPrioInterruptHandler (uint32_t eventId)
{
    uint32_t        channel, index, temp, count;
    uint32_t        *buf;
    void            *desc;

    channel = (eventId - 48) * 4 + coreNum;

#if !RM /* RM uses IPC which doesn't get along with printf's in an ISR context */
    System_printf ("Core %d : HIGH PRIORITY Rx ISR - channel %d\n", coreNum, channel);
#endif

    /* Process ISR. Read accumulator list */
    temp  = l2_global_address ((uint32_t) hiPrioList);
    temp &= 0xf0ffffff;
    buf   = (uint32_t *) temp;
    count = buf[0];

#if !RM /* RM uses IPC which doesn't get along with printf's in an ISR context */
    System_printf ("Core %d :              Received %d packets\n", coreNum, count);
#endif

    /* Recycle Rx BDs */
    for (index = 0; index < count; index++)
        Qmss_queuePushDesc (rxFreeQueHnd, (void *) buf[index + 1]);

    /* Set the EOI to indicate host is done processing. The page is freed by host to accumulator.
     * Accumulator can start writing to the freed page.
     */
    Qmss_ackInterrupt (channel, 1);
    Qmss_setEoiVector (Qmss_IntdInterruptType_HIGH, channel);

#if !RM /* RM uses IPC which doesn't get along with printf's in an ISR context */
    System_printf ("Core %d :              Sending SYNC signal\n", coreNum);
#endif
    /* Send the sync signal */
    if ((desc = Qmss_queuePop (syncFreeQueHnd)) != NULL)
    {
        Qmss_queuePushDesc (syncQueHnd, desc);
    }
#if !RM /* RM uses IPC which doesn't get along with printf's in an ISR context */        
    else
    {
        System_printf ("Error Core %d : No SYNC descriptor!!!\n", coreNum); 
        errorCount++;
    }
#endif        
#ifndef __LINUX_USER_SPACE
#ifdef L2_CACHE
    CACHE_invL2 ((void *) &runCount, 4, CACHE_WAIT);
#else
    CACHE_invL1d ((void *) &runCount, 4, CACHE_WAIT);
#endif
#endif
    runCount++;

#ifndef __LINUX_USER_SPACE
#ifdef L2_CACHE
    /* writeback L2 */
    CACHE_wbL2 ((void *) &runCount, 4, CACHE_WAIT);
#else
    /* Writeback L1D */
    CACHE_wbL1d ((void *) &runCount, 4, CACHE_WAIT);
#endif
#endif
}
#endif

#ifndef __LINUX_USER_SPACE
#if !RM /* Using HWIs when RM is enabled due to IPC usage */
/**
 *  @b Description
 *  @n  
 *      The functions initializes the INTC module.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static Int32 intcInit (void)
{
    CSL_IntcGlobalEnableState   state;

    /* INTC module initialization */
    context.eventhandlerRecord = Record;
    context.numEvtEntries      = 2;
    if (CSL_intcInit (&context) != CSL_SOK) 
        return -1;

    /* Enable NMIs */
    if (CSL_intcGlobalNmiEnable () != CSL_SOK) 
        return -1;
 
    /* Enable global interrupts */
    if (CSL_intcGlobalEnable (&state) != CSL_SOK) 
        return -1;

    /* INTC has been initialized successfully. */
    return 0;
}
#endif

/**
 *  @b Description
 *  @n  
 *
 *      Function registers the high priority interrupt.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */

static Int32 registerHiInterrupt (uint32_t coreNum, uint32_t qNum)
{
    CSL_IntcParam       vectId;
    Int16               channel, eventId;
#if RM    
    Hwi_Params          hwiAttrs;    
#endif

#if !RM /* Don't do intcInit when testing with RM.  IPC is added when testing with RM.  intcInit wipes out
         * the interrupt tables that IPC setup for itself */
    if (intcInit () < 0)
    {
        errorCount++;
        System_printf ("Error Core %d : Initializing interrupts\n", coreNum);
        return -1;           
    }
#endif    

    channel = qNum - QMSS_HIGH_PRIORITY_QUEUE_BASE;
    if (channel < 0)
    {
        errorCount++;
        System_printf ("Invalid High Priority queue : %d\n", qNum);
        return -1;
    }

    if (coreNum != (channel % 4))
    {
        errorCount++;
        System_printf ("Invalid High Priority queue : %d and core number : %d combination\n", qNum, coreNum);
        return -1;
    }

    eventId = (channel / 4) + 48;

    System_printf ("Core %d : Registering High Priority interrupt channel : %d eventId : %d queue Number : %d\n", 
                    coreNum, channel, eventId, qNum);


    /* Open INTC */
    vectId = CSL_INTC_VECTID_10;
#if !RM    
    hiPrioIntcHnd = CSL_intcOpen (&hiPrioIntcObj, eventId, &vectId, NULL);
    if (hiPrioIntcHnd == NULL)
        return -1;
 
    /* Bind ISR to Interrupt */
    Record[0].handler = (CSL_IntcEventHandler) &hiPrioInterruptHandler;
    Record[0].arg     = (void *) eventId;
    CSL_intcPlugEventHandler (hiPrioIntcHnd, &Record[0]);

    /* Event Clear */
    CSL_intcHwControl (hiPrioIntcHnd, CSL_INTC_CMD_EVTCLEAR, NULL);

    /* Event Enable */
    CSL_intcHwControl (hiPrioIntcHnd, CSL_INTC_CMD_EVTENABLE, NULL);
#else
    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_SELF;
    hwiAttrs.arg         = (UArg) eventId;
    hwiAttrs.eventId     = eventId;
    Hwi_create(vectId, (Hwi_FuncPtr)hiPrioInterruptHandler, &hwiAttrs, NULL); 
    
    Hwi_enableInterrupt(vectId); 
#endif    

    return 0;
}
#endif

#ifndef __LINUX_USER_SPACE
/**
 *  @b Description
 *  @n  
 *
 *      Entry point for example code.
 *      This is an example code that shows producer core sending data to consumer 
 *      core. Synchronization between different cores.
 *      
 *  @retval
 *      Not Applicable
 */
void main (void)
{
#if RM    
    /* RM configuration */
    Rm_InitCfg          rmInitCfg;
    char                rmInstName[RM_NAME_MAX_CHARS];
    int32_t             rmResult;
#else
    Task_Params         taskParams;   
#endif     
    
    System_printf ("**************************************************\n");
    System_printf ("************ QMSS Multicore Example **************\n");
    System_printf ("**************************************************\n");
    
#ifndef __LINUX_USER_SPACE
    /* Get the core number. */
    coreNum = CSL_chipReadReg (CSL_CHIP_DNUM); 
#endif
    
#if RM 
    /* Reset the variable to indicate to other cores RM init is not yet done */
    isRmInitialized = 0;

    /* Initialize the heap in shared memory used by RM. Using IPC module to do that */ 
    Ipc_start();
    
    /* Initialize RM */
    if (coreNum == SYSINIT)
    { 
        /* Create the Server instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Server");
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_SERVER;
        rmInitCfg.instCfg.serverCfg.globalResourceList = (void *)rmGlobalResourceList;
        rmInitCfg.instCfg.serverCfg.globalPolicy = (void *)rmDspOnlyPolicy;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            return;
        }
        
        isRmInitialized = 1;

#ifndef __LINUX_USER_SPACE
#ifdef L2_CACHE
        /* Writeback L2 */
        CACHE_wbL2 ((void *) &isRmInitialized, 4, CACHE_WAIT);
#else
        /* Writeback L1D */
        CACHE_wbL1d ((void *) &isRmInitialized, 4, CACHE_WAIT);
#endif        
#endif        
    }
    else
    {
        /* Create a RM Client instance */
        memset((void *)&rmInitCfg, 0, sizeof(Rm_InitCfg));
        System_sprintf (rmInstName, "RM_Client%d", coreNum);
        rmInitCfg.instName = rmInstName;
        rmInitCfg.instType = Rm_instType_CLIENT;
        rmHandle = Rm_init(&rmInitCfg, &rmResult);
        if (rmResult != RM_OK)
        {
            errorCount++;
            System_printf ("Error Core %d : Initializing Resource Manager error code : %d\n", coreNum, rmResult);
            return;
        }

#ifndef __LINUX_USER_SPACE
        do{
#ifdef L2_CACHE
            /* Invalidate L2 */
            CACHE_invL2 ((void *) &isRmInitialized, 4, CACHE_WAIT);
#else
            CACHE_invL1d ((void *) &isRmInitialized, 4, CACHE_WAIT);
#endif
        } while (isRmInitialized == 0);        
#endif
    }

    if (setupRmTransConfig(NUMBER_OF_CORES, SYSINIT, usageTsk) < 0)
    {
        errorCount++;
        System_printf ("Error core %d : Transport setup for RM error\n", coreNum);
        return;
    }
#else
    /* Create the RM transport configuration task */
    Task_Params_init (&taskParams);
    taskParams.priority = 1;
    Task_create (usageTsk, &taskParams, NULL);    
#endif /* RM */

    System_printf("Core %d : Starting BIOS...\n", coreNum);
    BIOS_start();
}
#endif /* __LINUX_USER_SPACE */

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
#ifdef __LINUX_USER_SPACE
void usageTsk(Cppi_Handle hnd)
#else
void usageTsk(UArg arg0, UArg arg1)
#endif
{
    uint32_t            channel, index;
    volatile uint32_t   count;
    Cppi_Desc           *desc;    

#ifdef __LINUX_USER_SPACE
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
#endif


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
#ifndef __LINUX_USER_SPACE
        /* Hookup interrupts */
        if (registerHiInterrupt (coreNum, QMSS_HIGH_PRIORITY_QUEUE_BASE + channel + coreNum) < 0)
        {
            errorCount++;
            System_printf ("Error Core %d : Registering interrupts\n", coreNum);
            return;           
        }
#endif

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
        }

        if (coreNum == SYSINIT)
        {
            for (index = 0; index < NUMBER_OF_CORES; index++)
            {
                if (send_data (channel + index, index) < 0)
                {
                    errorCount++;
                    System_printf ("Error Core %d : Sending data on channel %d core: %d\n", coreNum, channel + index, index);
                    return;           
                }
                /* Close Rx/Tx queues, channels and flows used in this data transfer */
                cleanup ();
            }
        }
#ifdef __LINUX_USER_SPACE
        receive_data ();
#endif
        System_printf ("Core %d : Waiting for sync signal\n", coreNum);

        count = Qmss_getQueueEntryCount (syncQueHnd);
        while (count < NUMBER_OF_CORES)
        {
            yield();
            count = Qmss_getQueueEntryCount (syncQueHnd);
        }
        
        System_printf ("Core %d : Got sync signal\n", coreNum);

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
                

        if (coreNum == SYSINIT)
        {
            /* Recycle the Tx descriptors from Tx completion queue to Tx free queue */
            Qmss_queueDivert (txCmplQueHnd, txFreeQueHnd, Qmss_Location_TAIL);

            printQueueStats ("After packet processing");
        }


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
    
#ifndef __LINUX_USER_SPACE
    do
    {
#ifdef L2_CACHE
        CACHE_invL2 ((void *) &runCount, 4, CACHE_WAIT);
#else
        CACHE_invL1d ((void *) &runCount, 4, CACHE_WAIT);
#endif
    }while (runCount != NUMBER_OF_CORES);
#endif
    /* De-initializes the system */
    sysExit ();

#if (RM) && !defined(__LINUX_USER_SPACE)
    if (coreNum == SYSINIT)
    {
        int32_t rmResult;
        
        if ((rmResult = Rm_resourceStatus(rmHandle, FALSE)) != 0)
        {
            errorCount++;
            System_printf ("Error Core %d : Number of unfreed resources : %d\n", coreNum, rmResult);
        }
        else
        {
            System_printf ("Core %d : All resources freed successfully\n", coreNum);
        }
    }
#endif    


    if (!errorCount)
    {
        System_printf ("*******************************************************\n");
        System_printf ("******** QMSS Multicore (%d) Example Done (PASS) *******\n", coreNum);
        System_printf ("*******************************************************\n");
    }
    else
    {
        System_printf ("core %d: QMSS example failed: %d errors\n", coreNum, errorCount);
    }
}

#if !RM && defined(__LINUX_USER_SPACE)
#error __LINUX_USER_SPACE requires RM
#endif

