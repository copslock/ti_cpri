/******************************************************************************
 * FILE PURPOSE:  Functions to initialize framework resources for running NWAL
 ******************************************************************************
 * FILE NAME:   fw_init.c
 *
 * DESCRIPTION: Functions to initialize framework resources for running NWAL
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
#include "fw_test.h"
#include "sockutils.h"

/* CSL RL includes */
#include <ti/csl/cslr_device.h>
#include <ti/csl/cslr_qm_config.h>
#include <ti/csl/cslr_qm_descriptor_region_config.h>
#include <ti/csl/cslr_qm_queue_management.h>
#include <ti/csl/cslr_qm_queue_status_config.h>
#include <ti/csl/cslr_qm_intd.h>
#include <ti/csl/cslr_pdsp.h>
#include <ti/csl/csl_qm_queue.h>
#include <ti/csl/cslr_cppidma_global_config.h>
#include <ti/csl/cslr_cppidma_rx_channel_config.h>
#include <ti/csl/cslr_cppidma_rx_flow_config.h>
#include <ti/csl/cslr_cppidma_tx_channel_config.h>
#include <ti/csl/cslr_cppidma_tx_scheduler_config.h>
#include <ti/csl/csl_cppi.h>


#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* TBD: Need to be moved to shannon specific file.
 * CSL Semaphore module includes
 */
#include <ti/drv/qmss/qmss_qm.h>
#include <ti/drv/qmss/qmss_drv.h>



/* RM Includes */
#include <ti/drv/rm/rm_server_if.h>
#include <ti/drv/rm/rm.h>
#include <ti/drv/rm/rm_transport.h>
#include <ti/drv/rm/rm_services.h>

/* HPLIB Include */
#include <ti/runtime/hplib/hplib.h>

/* TBD Currently only single ARM core is being handled */
#define CORE_ID            0
#define CACHE_LINESZ    NWAL_CACHE_LINE_SIZE


#define System_printf   printf
#define ALIGN(x)    __attribute__((aligned (x)))


void Osal_MtCsEnter(uint32_t *key);
void Osal_MtCsExit(uint32_t *key);
unsigned int Osal_nwalLocToGlobAddr(unsigned int x);


#if 0
/*** NWAL Memory Buffer Configuration ***/
#define TEST_CONFIG_BUFSIZE_NWAL_HANDLE     3400
#pragma DATA_SECTION (nwalInstMem, ".nwalInstMem")
#pragma DATA_ALIGN(nwalInstMem, CACHE_LINESZ)
uint8_t nwalInstMem[TEST_CONFIG_BUFSIZE_NWAL_HANDLE]ALIGN(CACHE_LINESZ);
#endif

void* pNwalLocCtxMem = NULL;


#if 0
#define TEST_CONFIG_BUFSIZE_NWAL_PER_MAC                    384
#define TEST_CONFIG_BUFSIZE_NWAL_IPSEC_HANDLE_PER_CHAN      384
#define TEST_CONFIG_BUFSIZE_NWAL_DM_HANDLE_PER_CHAN         256
#define TEST_CONFIG_BUFSIZE_NWAL_PER_IP                     384
#define TEST_CONFIG_BUFSIZE_NWAL_PER_PORT                   256
#define TEST_CONFIG_BUFSIZE_NWAL_PER_L2L3_HDR               128
#define TEST_CONFIG_BUFSIZE_NWAL_PER_LOC_CONTEXT            384
#define NWAL_CHAN_HANDLE_SIZE    ((TEST_CONFIG_BUFSIZE_NWAL_PER_MAC * TEST_MAX_NUM_MAC) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_IPSEC_HANDLE_PER_CHAN * TEST_MAX_NUM_IPSEC_CHANNELS) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_DM_HANDLE_PER_CHAN * TEST_MAX_NUM_DM_SA_CHANNELS) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_IP * TEST_MAX_NUM_IP) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_PORT * TEST_MAX_NUM_PORTS)+ \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_LOC_CONTEXT * CPU_NCORES) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_L2L3_HDR * TEST_MAX_NUM_L2_L3_HDRS))

#pragma DATA_SECTION (nwalHandleMem, ".nwalHandleMem")
#pragma DATA_ALIGN(nwalHandleMem, CACHE_LINESZ)
uint8_t nwalHandleMem[NWAL_CHAN_HANDLE_SIZE]ALIGN(CACHE_LINESZ);
#endif
/* TBD: Check if below size information can be made available from pa interface file */
#define TEST_CONFIG_BUFSIZE_PA_BUF0         512
/* PA instance */
/* Memory used for the PA Instance. Needs to be assigned global uncached memory for chip */
#pragma DATA_SECTION (paBuf0, ".paBuf0")
#pragma DATA_ALIGN(paBuf0, CACHE_LINESZ)
uint8_t paBuf0[TEST_CONFIG_BUFSIZE_PA_BUF0]ALIGN(CACHE_LINESZ);

/* Memory used for PA handles */
#define TEST_CONFIG_BUFSIZE_PA_BUF1         128
#pragma DATA_SECTION (paBuf1, ".paBuf1")
#pragma DATA_ALIGN(paBuf1, CACHE_LINESZ)
uint8_t paBuf1[TEST_CONFIG_BUFSIZE_PA_BUF1]ALIGN(CACHE_LINESZ);

#define TEST_CONFIG_BUFSIZE_PA_BUF2_PER_IP  192
#define TEST_CONFIG_BUFSIZE_PA_BUF2 (TEST_CONFIG_BUFSIZE_PA_BUF2_PER_IP *(TEST_MAX_NUM_IP+1))
#pragma DATA_SECTION (paBuf2, ".paBuf2")
#pragma DATA_ALIGN(paBuf2, CACHE_LINESZ)
uint8_t paBuf2[TEST_CONFIG_BUFSIZE_PA_BUF2]ALIGN(CACHE_LINESZ);

#ifdef NWAL_ENABLE_SA
/* Memory used for SA LLD global Handle */
#define TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE           512
#pragma DATA_SECTION (salldHandle, ".salldHandle")
#pragma DATA_ALIGN(salldHandle, CACHE_LINESZ)
uint8_t salldHandle[TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE]ALIGN(CACHE_LINESZ);

/* Memory used for SA LLD global Handle */
#define TEST_CONFIG_BUFSIZE_SA_CONTEXT_PER_CHAN     512
#pragma DATA_SECTION (saContext, ".saContext")
#pragma DATA_ALIGN(saContext, CACHE_LINESZ)
uint8_t saContext[TEST_CONFIG_BUFSIZE_SA_CONTEXT_PER_CHAN * (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)]ALIGN(CACHE_LINESZ);

/* Memory used by SA LLD per Channel */
#define TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE_PER_CHAN  512
#pragma DATA_SECTION (salldChanHandle, ".salldChanHandle")
#pragma DATA_ALIGN(salldChanHandle, CACHE_LINESZ)
uint8_t salldChanHandle[TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE_PER_CHAN * (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)]ALIGN(CACHE_LINESZ);
#endif

/* Socket timeout */
#define CLIENT_SOCK_TIMEOUT_USEC     (500)

/* Application's registered RM transport indices */
#define SERVER_TO_CLIENT_MAP_ENTRY   0
/* Maximum number of registered RM transports */
#define MAX_MAPPING_ENTRIES          1

/* Error checking macro */
#define RM_ERROR_CHECK(checkVal, resultVal, rmInstName, printMsg)                 \
    if (resultVal != checkVal) {                                                  \
        char errorMsgToPrint[] = printMsg;                                        \
        printf("RM Inst : %s : ", rmInstName);                                    \
        printf("%s with error code : %d, exiting\n", errorMsgToPrint, resultVal); \
        return(-1);                                                               \
    }

/* RM registered transport mapping structure */
typedef struct trans_map_entry_s {
    /* Registered RM transport handle */
    Rm_TransportHandle        transportHandle;
    /* Remote socket tied to the transport handle */
    sock_name_t              *remote_sock;
} Transport_MapEntry;

extern void *fw_netcpCfgVaddr;

/* RM Client Vars */
Rm_Handle           rmClientHandle = NULL;
Rm_ServiceHandle   *rmClientServiceHandle = NULL;
sock_h              rmClientSocket;

/* Client instance name (must match with RM Global Resource List (GRL) and policies */
char                rmClientName[RM_NAME_MAX_CHARS] = "RM_Client0";

/* Client socket name */
char                rmClientSockName[] = "/tmp/var/run/rm/rm_client";

/* Transport map stores the RM transport handle to IPC MessageQ queue mapping */
Transport_MapEntry  rmTransportMap[MAX_MAPPING_ENTRIES];


/*****************************************************************************
 * Global Resources shared by all Cores
 *****************************************************************************/
uint8_t *QMemGlobDescRam = 0;

/*****************************************************************************
 * Local Resource allocated at each Core
 *****************************************************************************/
/* Descriptors in global shared */


/* Per proc configuration. In the case of ARM only one config
 * can be shared across multiple threads
 */
nwalLocCfg_t        nwalLocCfg;

void        Osal_invalidateCache (void *blockPtr, uint32_t size);
void        Osal_writeBackCache (void *blockPtr, uint32_t size);


/*****************************************************************************
 * FUNCTION PURPOSE: Global Initialization of Queue Manager. Once Per System
 *****************************************************************************
 * DESCRIPTION: The function will initialize the Queue Manager
 *****************************************************************************/
static nwal_Bool_t testNwLocQmInit (void)
{
    int32_t          result;
     result = Qmss_start();
     if (result != QMSS_SOK)
     {
         System_printf ("function setupQmMem: Qmss_start failed with error code %d\n", result);
         return (nwal_FALSE);
     }
     return (nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Initialize the Queue Manager
 *****************************************************************************
 * DESCRIPTION: The function will initialize the Queue Manager
 *****************************************************************************/
nwal_Bool_t testNwSetupQmMem(uint32_t          numDesc,
                            uint32_t          descSize,
                            uint32_t*         pDescMemBase,
                            Qmss_MemRegion    memRegion,
                            Qmss_QueueHnd*    pdescQ)
{
  Qmss_MemRegInfo   memInfo;
  Cppi_DescCfg      descCfg;
  Int32             result;

  memset(&memInfo,0,sizeof(Qmss_MemRegInfo));
  memset(&descCfg,0,sizeof(Cppi_DescCfg));
  memInfo.descBase       = pDescMemBase;
  memInfo.descSize       = descSize;
  memInfo.descNum        = numDesc;
  memInfo.manageDescFlag = Qmss_ManageDesc_MANAGE_DESCRIPTOR;
  memInfo.memRegion      = memRegion;

  memInfo.startIndex     = TUNE_NWAL_QM_START_BASE_INDEX;

  result = Qmss_insertMemoryRegion (&memInfo);
  if (result < QMSS_SOK)  {
    System_printf ("function testNwSetupQmMem: Qmss_insertMemoryRegion returned error code %d\n", result);
    return (nwal_FALSE);
  }

  return (nwal_TRUE);

}

/*****************************************************************************
 * FUNCTION PURPOSE: Memory Free callback from pktLib
 *****************************************************************************
 * DESCRIPTION: Memory Free callback from pktLib
 *****************************************************************************/
void testNwPktLibFree (uint8_t* ptrDataBuffer, uint32_t size)
{
    /* Not handled. Not expected as heap is created and is not freed */
    System_printf ("testNwPktLibFree()Called. No Actions being done %d \n");
}

/*****************************************************************************
 * FUNCTION PURPOSE: Setup the queues
 *****************************************************************************
 * DESCRIPTION: The function will setup all the queues used by the chip
 *****************************************************************************/
uint8_t*  testNwPktLibAlloc (uint32_t size)
{
    uint8_t*    pBuf;
    pBuf = (uint8_t *)hplib_vmMemAlloc(size,CACHE_LINESZ, 0);
    if (!pBuf)
    {
        printf("testNwPktLibAlloc:Buffer allocation failed \n" );
        return nwal_FALSE;
    }
    return(pBuf);
}

/***************************************************************************************
 * FUNCTION PURPOSE: NWAL start per core
 ***************************************************************************************
 * DESCRIPTION: function to start NWAL in a core where all cores share nwal Local 
 *              context.
 ***************************************************************************************/
nwal_Bool_t testNwalStartCore(void)
{
    nwal_RetValue       nwalRetVal;

    int core_id = Osal_nwalGetProcId();

    while(1)
    {
        nwalRetVal = nwal_start(testNwGlobContext.nwalInstHandle,&nwalLocCfg);
        if(nwalRetVal == nwal_ERR_INVALID_STATE)
        {
            continue;
        }
        break;
    }

    if(nwalRetVal != nwal_OK)
    {
        System_printf ("CORE: %d testNwalStartCore :Failed\n", core_id);
        return nwal_FALSE;
    }
    else
        System_printf ("CORE: %d testNwalStartCore :Passed\n", core_id);
    return nwal_TRUE;
}

/***************************************************************************************
 * FUNCTION PURPOSE: NWAL start per core
 ***************************************************************************************
 * DESCRIPTION: function to start NWAL in a core
 ***************************************************************************************/
nwal_Bool_t testNwalStart(void)
{
    nwal_RetValue       nwalRetVal;
    char                heapName[50];
    Qmss_MemRegion      memRegion;
    Pktlib_HeapCfg      heapCfg;
    int32_t             errCode;

    int core_id = Osal_nwalGetProcId();
    memset(&nwalLocCfg,0,sizeof(nwalLocCfg));
    memset(&testNwLocContext,0,sizeof(testNwLocContext_t));
    memRegion = TUNE_NWAL_QM_GLOBAL_REGION;

    /* Call back registration for the core */
    nwalLocCfg.pRxPktCallBack = testNWALRxPktCallback;
    nwalLocCfg.pCmdCallBack = testNWALCmdCallBack;
    nwalLocCfg.pPaStatsCallBack = testNWALCmdPaStatsReply;
    nwalLocCfg.pRxDmCallBack = testNWALRxDmBack;

    /* Initialize Buffer Pool for Control packets from NetCP to Host */
    nwalLocCfg.rxCtlPool.numBufPools = 1;
    nwalLocCfg.rxCtlPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.rxCtlPool.bufPool[0].bufSize = TEST_CONFIG_MAX_CTL_RX_BUF_SIZE;

    sprintf(heapName,"RX Ctl%d",core_id);

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));
    /* Populate the heap configuration */
    heapCfg.name                = heapName;
    heapCfg.memRegion           = memRegion;
    heapCfg.sharedHeap          = 0;
    heapCfg.useStarvationQueue  = 0;
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_CTL_RX_BUF_SIZE;
    //heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_RX_NBUFS;
    heapCfg.numZeroBufferPackets= 0;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    heapCfg.dataBufferPktThreshold   = 0;
    heapCfg.zeroBufferPktThreshold   = 0;

    nwalLocCfg.rxCtlPool.bufPool[0].heapHandle =
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.rxCtlPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for RX Ctl From NetCP, Error Code: %d\n",errCode);
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Control packets from Host to NetCP */
    nwalLocCfg.txCtlPool.numBufPools = 1;
    nwalLocCfg.txCtlPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.txCtlPool.bufPool[0].bufSize = TEST_CONFIG_MAX_CTL_TX_BUF_SIZE;


    sprintf(heapName,"TX Ctl%d",core_id);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_CTL_TX_BUF_SIZE;
    //heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_TX_NBUFS;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;


    nwalLocCfg.txCtlPool.bufPool[0].heapHandle =
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.txCtlPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for TX Ctl To NetCP\n");
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Packets from NetCP to Host */
    nwalLocCfg.rxPktPool.numBufPools = 1;
    nwalLocCfg.rxPktPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.rxPktPool.bufPool[0].bufSize = TEST_CONFIG_MAX_PKT_RX_BUF_SIZE;


    sprintf(heapName,"Rx Pkt%d",core_id);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_PKT_RX_BUF_SIZE;
    //heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_RX_NBUFS;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;

    nwalLocCfg.rxPktPool.bufPool[0].heapHandle =  Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.rxPktPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for Rx Pkt From NetCP\n");
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Packets from Host to NetCP */
    nwalLocCfg.txPktPool.numBufPools = 1;
    nwalLocCfg.txPktPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.txPktPool.bufPool[0].bufSize = TEST_CONFIG_MAX_PKT_TX_BUF_SIZE;

    sprintf(heapName,"Tx Pkt%d",core_id);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_PKT_TX_BUF_SIZE;
    //heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_TX_NBUFS;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    nwalLocCfg.txPktPool.bufPool[0].heapHandle =
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.txPktPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for TX Pkt To NetCP , Error Code: %d\n",errCode);
        nwal_SystemFlush();
        return nwal_FALSE;
    }
    
    while(1)
    {
        nwalRetVal = nwal_start(testNwGlobContext.nwalInstHandle,&nwalLocCfg);
        if(nwalRetVal == nwal_ERR_INVALID_STATE)
        {
            continue;
        }
        break;
    }

    if(nwalRetVal != nwal_OK)
    {
        System_printf ("nwal_start:Failed !!!\n");
        return nwal_FALSE;
    }
    return nwal_TRUE;
}


/*****************************************************************************
 * FUNCTION PURPOSE: Global Initializion of the Network  related resources
 *****************************************************************************
 * DESCRIPTION: Global Initializion of the Network  related resources
 *****************************************************************************/
nwal_Bool_t testNwGlobalInit (void)
{
    nwalGlobCfg_t   nwalGlobCfg;
    nwalSizeInfo_t  nwalSizeInfo;
    nwal_RetValue   nwalRetVal;
    uint8_t         count;
    int             sizes[nwal_N_BUFS];
    int             aligns[nwal_N_BUFS];
    void*           bases[nwal_N_BUFS];
    Pktlib_HeapCfg      heapCfg;
    int32_t             errCode;
    char                heapName[50];
    int i;
    void* pBase;
    testNwGlobContextShmMem_t* nwalEntry;
    nwalBaseAddrCfg_t     baseAddrCfg;
    uint32_t localCtxSize = 0;
    /* Global Network resource QM/CPPI/PA related initialization to
     * be done by Core 0
     */
    memset(&testNwGlobContext,0,sizeof(testNwGlobContext_t));
    memset(&nwalGlobCfg,0,sizeof(nwalGlobCfg_t));
    memset(sizes,0,sizeof(sizes));
    memset(aligns,0,sizeof(aligns));
    memset(bases,0,sizeof(bases));
    memset(&baseAddrCfg,0, sizeof(nwalBaseAddrCfg_t));

    QMemGlobDescRam = (uint8_t*)hplib_vmMemAlloc(((TEST_CONFIG_MAX_LOC_DESC +TEST_CONFIG_MAX_GLOB_DESC)*
                                        TEST_CONFIG_LOC_DESC_SIZE),
                                        CACHE_LINESZ, 0);
    if (!QMemGlobDescRam)
    {
        printf ("Memory allocation for Local Desc from global memory failed\n");
        return nwal_FALSE;
    }

#if 1
    /* Setup RM */
    if (initRm())  {
        printf ("setupTestFramework: initRm returned error, exiting\n");
        return (-1);
    }
#endif
    /* QM Global Init */
    if(!testNwGlobQmInit())
    {
        System_printf ("QM Global Initialization failed\n");
        return nwal_FALSE;
    }

    /* QM Local Init */
    if(!testNwLocQmInit())
    {
        System_printf ("QM Local Initialization failed\n");
        return nwal_FALSE;
    }

    if(!testNwGlobCppiInit())
    {
        System_printf ("CPPI Global Initialization failed\n");
        return nwal_FALSE;
    }

    /* Initialize descriptor for packet and control processing at each core from global shared external memory */
    /* Initialize Global Descriptors */
    if(testNwSetupQmMem((TEST_CONFIG_MAX_LOC_DESC + TEST_CONFIG_MAX_GLOB_DESC),
                        TEST_CONFIG_LOC_DESC_SIZE,
                        (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)QMemGlobDescRam),
                        TUNE_NWAL_QM_GLOBAL_REGION,
                        &testNwGlobContext.descQ)!= nwal_TRUE)
    {
        System_printf ("testNwGlobalInit:QM Memory region/Descriptor Initialization for per core descriptors failed\n");
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for NetCP PA to SA packets */
    nwalGlobCfg.pa2SaBufPool.numBufPools = 1;
    nwalGlobCfg.pa2SaBufPool.bufPool[0].descSize = TEST_CONFIG_GLOB_DESC_SIZE;
    nwalGlobCfg.pa2SaBufPool.bufPool[0].bufSize = TEST_CONFIG_GLOB_BUF_SIZE;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));
    /* Populate the heap configuration */

    sprintf(heapName,"nwal PA2SA%d",CORE_ID);
    heapCfg.name                = heapName;
    heapCfg.memRegion           = TUNE_NWAL_QM_GLOBAL_REGION;
    heapCfg.sharedHeap          = 0;
    heapCfg.useStarvationQueue  = 0;
    heapCfg.dataBufferSize      = TEST_CONFIG_GLOB_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PA_TO_SA_DESC;
    heapCfg.numZeroBufferPackets= 0;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    heapCfg.dataBufferPktThreshold   = 0;
    heapCfg.zeroBufferPktThreshold   = 0;

    nwalGlobCfg.pa2SaBufPool.bufPool[0].heapHandle =
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalGlobCfg.pa2SaBufPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for PA to SA Buffer Pool Error Code :%d\n",errCode);
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for NetCP SA to PA packets */
    sprintf(heapName,"nwal PA2SA%d",CORE_ID);
    heapCfg.dataBufferSize      = TEST_CONFIG_GLOB_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_SA_TO_PA_DESC;

    nwalGlobCfg.sa2PaBufPool.numBufPools = 1;
    nwalGlobCfg.sa2PaBufPool.bufPool[0].descSize = TEST_CONFIG_GLOB_DESC_SIZE;
    nwalGlobCfg.sa2PaBufPool.bufPool[0].bufSize = TEST_CONFIG_GLOB_BUF_SIZE;

    nwalGlobCfg.sa2PaBufPool.bufPool[0].heapHandle =
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalGlobCfg.sa2PaBufPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for SA to PA Buffer Pool Error Code :%d\n",errCode);
        nwal_SystemFlush();
        return nwal_FALSE;
    }

    nwalGlobCfg.hopLimit = 5;/* Default TTL / Hop Limit */
    nwalGlobCfg.paPowerOn = nwal_TRUE;
    nwalGlobCfg.saPowerOn = nwal_TRUE;
    nwalGlobCfg.paFwActive = nwal_TRUE;
    nwalGlobCfg.saFwActive = nwal_FALSE;

    /* Update Virtual Address */
    baseAddrCfg.paVirtBaseAddr = (uint32_t)fw_netcpCfgVaddr;

    baseAddrCfg.pSaVirtBaseAddr = (uint32_t)fw_netcpCfgVaddr +
                                 ((uint32_t)CSL_NETCP_CFG_SA_CFG_REGS -
                                 (uint32_t)CSL_NETCP_CFG_REGS);

    nwalGlobCfg.rxDefPktQ = QMSS_PARAM_NOT_SPECIFIED;

    /* Get the Buffer Requirement from NWAL */
    memset(&nwalSizeInfo,0,sizeof(nwalSizeInfo));
    nwalSizeInfo.nMaxMacAddress = TEST_MAX_NUM_MAC;
    nwalSizeInfo.nMaxIpAddress = TEST_MAX_NUM_IP;
    nwalSizeInfo.nMaxL4Ports = TEST_MAX_NUM_PORTS;
    nwalSizeInfo.nMaxIpSecChannels = TEST_MAX_NUM_IPSEC_CHANNELS;
    nwalSizeInfo.nMaxDmSecChannels = TEST_MAX_NUM_DM_SA_CHANNELS;
    nwalSizeInfo.nMaxL2L3Hdr = TEST_MAX_NUM_L2_L3_HDRS;
    nwalSizeInfo.nProc = CPU_NCORES;
    nwalRetVal = nwal_getBufferReq(&nwalSizeInfo,
                                   sizes,
                                   aligns);
    if(nwalRetVal != nwal_OK)
    {
        System_printf ("testNwGlobalInit:nwal_getBufferReq Failed \n");
        return nwal_FALSE;
    }

    
    pBase = hplib_shmOpen();
    if(hplib_shmAddEntry(pBase, sizeof(testNwGlobContextShmMem_t), NWAL_ENTRY) == 0)
    {
        printf("testNwGlobalInit: calling shmGetEntry for nwal\n");
        nwalEntry = (testNwGlobContextShmMem_t*)hplib_shmGetEntry(pBase,NWAL_ENTRY);
        //bases[nwal_BUF_INDEX_INST] = (void*)nwalEntry;
        printf("testNwGlobalInit: base address of nwal entry: 0x%x\n", (uint32_t) nwalEntry);
    }




    /* Check for memory size requirement and update the base */
    count = 0;
    bases[nwal_BUF_INDEX_INST] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)nwalEntry->nwalInstMem);
    if(TEST_CONFIG_BUFSIZE_NWAL_HANDLE < sizes[nwal_BUF_INDEX_INST])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_INT_HANDLES] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)nwalEntry->nwalHandleMem);
    if(NWAL_CHAN_HANDLE_SIZE  < sizes[nwal_BUF_INDEX_INT_HANDLES])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_PA_LLD_BUF0] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)paBuf0);
    if((TEST_CONFIG_BUFSIZE_PA_BUF0) < sizes[nwal_BUF_INDEX_PA_LLD_BUF0])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_PA_LLD_BUF1] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)paBuf1);
    if((TEST_CONFIG_BUFSIZE_PA_BUF1) < sizes[nwal_BUF_INDEX_PA_LLD_BUF1])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_PA_LLD_BUF2] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)paBuf2);
    if((TEST_CONFIG_BUFSIZE_PA_BUF2) < sizes[nwal_BUF_INDEX_PA_LLD_BUF2])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

#ifdef NWAL_ENABLE_SA
    bases[nwal_BUF_INDEX_SA_LLD_HANDLE] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)salldHandle);
    if((TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE) < sizes[nwal_BUF_INDEX_SA_LLD_HANDLE])
    {
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_SA_CONTEXT] =
        (uint8_t*)hplib_vmMemAlloc(sizes[nwal_BUF_INDEX_SA_CONTEXT],
                                    CACHE_LINESZ, 0);
    count++;

    bases[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)salldChanHandle);
    if((TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE_PER_CHAN * (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)) <
        sizes[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE])
    {
        /* Resize Memory */
        while(1);
    }
    count++;
#else
    bases[nwal_BUF_INDEX_SA_LLD_HANDLE] = 0;
    bases[nwal_BUF_INDEX_SA_CONTEXT] = 0;
    bases[nwal_BUF_INDEX_SA_LLD_CHAN_HANDLE] = 0;
    count = count+3;
#endif
    if(count != nwal_N_BUFS)
    {
        while(1);
    }

    //nwalGlobCfg.pDeviceCfg = &nwalDeviceGblCfgParam;




    /* Initialize NWAL module */
    nwal_getLocContextBufferReq(nwalSizeInfo.nProc, &localCtxSize);
    pNwalLocCtxMem = malloc(localCtxSize);

    baseAddrCfg.pInstPoolSaBaseAddr = bases[nwal_BUF_INDEX_SA_LLD_HANDLE];
    baseAddrCfg.pScPoolBaseAddr = bases[nwal_BUF_INDEX_SA_CONTEXT];
    baseAddrCfg.pInstPoolPaBaseAddr = bases[nwal_BUF_INDEX_PA_LLD_BUF0];


    nwal_createProc(bases[nwal_BUF_INDEX_INST],
                    pNwalLocCtxMem,
                    &baseAddrCfg);
    nwalGlobCfg.pBaseAddrCfg = &baseAddrCfg;
    
    nwalRetVal = nwal_create(&nwalGlobCfg,
                             &nwalSizeInfo,
                             sizes,
                             bases,
                             &testNwGlobContext.nwalInstHandle);
    if(nwalRetVal != nwal_OK)
    {
        System_printf ("testNwGlobalInit:nwal_create Failed %d\n",nwalRetVal);
        while(1);
    }

    for (i=0; i<5;i++)
    {
        /* Qos packet queue for receiving packets.*/
        nwalRetVal = nwal_CreateGenPurposeQueue(&testNwGlobContext.qosQ[i]);

        System_printf("testNwGlobalInit: qosQ[%d]: %d\n",
                        i,
                        testNwGlobContext.qosQ[i]);

        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }
    }
    System_printf("CORE: %d Global and Local Network initialization Successful \n",CORE_ID);

    testNwGlobContext.state = TEST_NW_CXT_GLOB_ACTIVE;
    return(nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Initialization per local core
 *****************************************************************************
 * DESCRIPTION: Initialization per local core
 *****************************************************************************/
nwal_Bool_t testNwLocaInit (void)
{
  int32_t            result;

  while(1)
  {
      /* Invalidate the Global context */
      if(testNwGlobContext.state == TEST_NW_CXT_GLOB_INACTIVE)
      {
          continue;
      }

      /* Block until Qmss_init() has completed by core 0 */
      result = Qmss_start();
      if(result == QMSS_NOT_INITIALIZED)
      {
          System_printf ("QMSS Not yet Initialized Waiting for Core %d\n", CORE_ID);
          continue;
      }
      else if (result != QMSS_SOK)  {
        System_printf ("function setupQmMem: Qmss_start failed with error code %d\n", result);
        return (nwal_FALSE);
      }

      if (result == QMSS_SOK)
      {
          break;
      }
  }

  return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Initialize the Network  related resources and NWAL
 *****************************************************************************
 * DESCRIPTION: Initialize the Network PA related resources and NWAL
 *****************************************************************************/
nwal_Bool_t testNwInit (void)
{

    /* Global Network resource QM/CPPI/PA related initialization to
     * be done by Core 0
     */

        if(!testNwGlobalInit())
        {
            System_printf("ERROR: Global Network initialization Failed \n");
            return nwal_FALSE;
        }
    /* Start NWAL */
    if(testNwalStart() != nwal_TRUE)
    {
        System_printf ("CORE: %d testNwGlobalInit:testNwalStart Failed \n",CORE_ID);
        while(1);
    }

    testNwLocContext.state = TEST_NW_CXT_LOC_ACTIVE;
    return 0;
}
Rm_Packet *transportAlloc(Rm_AppTransportHandle appTransport, uint32_t pktSize, Rm_PacketHandle *pktHandle)
{
    Rm_Packet *rm_pkt = NULL;

    rm_pkt = calloc(1, sizeof(*rm_pkt));
    if (!rm_pkt) {
        printf("can't malloc for RM send message (err: %s)\n", strerror(errno));
        return (NULL);
    }
    rm_pkt->pktLenBytes = pktSize;
    *pktHandle = rm_pkt;

    return(rm_pkt);
}

void transportFree (Rm_Packet *rm_pkt)
{
    if (rm_pkt) {
        free (rm_pkt);
    }
}

void transportReceive (void)
{
    int32_t             rm_result;
    int                 retval;
    int                 length = 0;
    sock_name_t         server_sock_addr;
    Rm_Packet          *rm_pkt = NULL;
    struct sockaddr_un  server_addr;

    retval = sock_wait(rmClientSocket, &length, NULL, -1);
    if (retval == -2) {
        /* Timeout */
        printf("Error socket timeout\n");
        return;
    }
    else if (retval < 0) {
        printf("Error in reading from socket, error %d\n", retval);
        return;
    }

    if (length < sizeof(*rm_pkt)) {
        printf("invalid RM message length %d\n", length);
        return;
    }
    rm_pkt = calloc(1, length);
    if (!rm_pkt) {
        printf("can't malloc for recv'd RM message (err: %s)\n",
               strerror(errno));
        return;
    }

    server_sock_addr.type = sock_addr_e;
    server_sock_addr.s.addr = &server_addr;
    retval = sock_recv(rmClientSocket, (char *)rm_pkt, length, &server_sock_addr);
    if (retval != length) {
        printf("recv RM pkt failed from socket, received = %d, expected = %d\n",
               retval, length);
        return;
    }

    //printf("received RM pkt of size %d bytes from %s\n", length, server_sock_addr.s.addr->sun_path);

    /* Provide packet to RM Client for processing */
    if ((rm_result = Rm_receivePacket(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle, rm_pkt))) {
        printf("RM failed to process received packet: %d\n", rm_result);
    }

    transportFree(rm_pkt);
}

int32_t transportSendRcv (Rm_AppTransportHandle appTransport, Rm_PacketHandle pktHandle)
{
    sock_name_t *server_sock_name = (sock_name_t *)appTransport;
    Rm_Packet   *rm_pkt = (Rm_Packet *)pktHandle;

    if (sock_send(rmClientSocket, (char *)rm_pkt, (int) rm_pkt->pktLenBytes, server_sock_name))     {
        printf("send data failed\n");
    }

    /* Wait for response from Server */
    transportReceive();
    return (0);
}

int connection_setup(void)
{
    Rm_TransportCfg rmTransCfg;
    int32_t         rm_result;
    int             i;
    sock_name_t     sock_name;
    char            server_sock_name[] = RM_SERVER_SOCKET_NAME;

    /* Initialize the transport map */
    for (i = 0; i < MAX_MAPPING_ENTRIES; i++) {
        rmTransportMap[i].transportHandle = NULL;
    }

    sock_name.type = sock_name_e;
    sock_name.s.name = rmClientSockName;

    rmClientSocket = sock_open(&sock_name);
    if (!rmClientSocket) {
        printf("connection_setup: Client socket open failed\n");
        return (-1);
    }

    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock = calloc(1, sizeof(sock_name_t));
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->type = sock_name_e;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name = calloc(1, strlen(server_sock_name)+1);
    strncpy(rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock->s.name, server_sock_name, strlen(server_sock_name)+1);

    /* Register the Server with the Client instance */
    rmTransCfg.rmHandle = rmClientHandle;
    rmTransCfg.appTransportHandle = (Rm_AppTransportHandle) rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].remote_sock;
    rmTransCfg.remoteInstType = Rm_instType_SERVER;
    rmTransCfg.transportCallouts.rmAllocPkt = transportAlloc;
    rmTransCfg.transportCallouts.rmSendPkt = transportSendRcv;
    rmTransportMap[SERVER_TO_CLIENT_MAP_ENTRY].transportHandle = Rm_transportRegister(&rmTransCfg, &rm_result);  

    return(0);
}

/** ============================================================================
 *   @n@b initRm
 *
 *   @b Description
 *   @n This API initializes the RM Client for the QMSS test establishing
 *      a socket connection with the RM Server
 *
 *   @return    int32_t
 *              -1      -   Error
 *              0       -   Success
 * =============================================================================
 */
int initRm (void)
{
    Rm_InitCfg         rmInitCfg;
    int32_t            result;

    /* Initialize the RM Client - RM must be initialized before anything else in the system */
    memset(&rmInitCfg, 0, sizeof(rmInitCfg));
    rmInitCfg.instName = rmClientName;
    rmInitCfg.instType = Rm_instType_CLIENT;
    rmClientHandle = Rm_init(&rmInitCfg, &result);
    RM_ERROR_CHECK(RM_OK, result, rmClientName, "Initialization failed");

    printf("\n\nInitialized %s\n\n", rmClientName);

    /* Open Client service handle */
    rmClientServiceHandle = Rm_serviceOpenHandle(rmClientHandle, &result);
    RM_ERROR_CHECK(RM_OK, result, rmClientName, "Service handle open failed");

    return(connection_setup());
}
