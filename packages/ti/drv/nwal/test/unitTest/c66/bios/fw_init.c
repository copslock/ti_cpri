/******************************************************************************
 * FILE PURPOSE:  Functions to initialize framework resources for running NWAL
 ******************************************************************************
 * FILE NAME:   nw_init.c
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
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>

/* TBD: Need to be moved to shannon specific file. 
 * CSL Semaphore module includes 
 */
#include <ti/csl/csl_semAux.h>
#include <ti/drv/qmss/qmss_qm.h>
#if defined(DEVICE_K2H) || defined(SOC_K2H)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2K) || defined(SOC_K2K)
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#elif defined (DEVICE_K2L) || defined(SOC_K2L)
#include <ti/drv/qmss/device/k2l/src/qmss_device.c>
#include <ti/drv/cppi/device/k2l/src/cppi_device.c>
#elif defined (DEVICE_K2E) || defined(SOC_K2E)
#include <ti/drv/qmss/device/k2e/src/qmss_device.c>
#include <ti/drv/cppi/device/k2e/src/cppi_device.c>
#elif defined(SOC_C6678)
#include <ti/drv/qmss/device/c6678/src/qmss_device.c>
#include <ti/drv/cppi/device/c6678/src/cppi_device.c>
#else /*Default */
#include <ti/drv/qmss/device/k2h/src/qmss_device.c>
#include <ti/drv/cppi/device/k2h/src/cppi_device.c>
#endif /* Device */

Void Osal_MtCsEnter(uint32_t *key);
Void Osal_MtCsExit(uint32_t *key);
unsigned int Osal_nwalLocToGlobAddr(unsigned int x);

/*** NWAL Memory Buffer Configuration ***/
#define TEST_CONFIG_BUFSIZE_NWAL_HANDLE     3400
#pragma DATA_SECTION (nwalInstMem, ".nwalInstMem")
#pragma DATA_ALIGN(nwalInstMem, CACHE_L2_LINESIZE)
uint8_t nwalInstMem[TEST_CONFIG_BUFSIZE_NWAL_HANDLE];

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
#pragma DATA_ALIGN(nwalHandleMem, CACHE_L2_LINESIZE)
uint8_t nwalHandleMem[NWAL_CHAN_HANDLE_SIZE];

/* TBD: Check if below size information can be made available from pa interface file */
#define TEST_CONFIG_BUFSIZE_PA_BUF0      512
/* PA instance */
/* Memory used for the PA Instance. Needs to be assigned global uncached memory for chip */
#pragma DATA_SECTION (paBuf0, ".paBuf0")
#pragma DATA_ALIGN(paBuf0, CACHE_L2_LINESIZE)
uint8_t paBuf0[TEST_CONFIG_BUFSIZE_PA_BUF0];

/* Memory used for PA handles */
#define TEST_CONFIG_BUFSIZE_PA_BUF1    128
#pragma DATA_SECTION (paBuf1, ".paBuf1")
#pragma DATA_ALIGN(paBuf1, CACHE_L2_LINESIZE)
uint8_t paBuf1[TEST_CONFIG_BUFSIZE_PA_BUF1];

#define TEST_CONFIG_BUFSIZE_PA_BUF2_PER_IP  192
#define TEST_CONFIG_BUFSIZE_PA_BUF2 (TEST_CONFIG_BUFSIZE_PA_BUF2_PER_IP *(TEST_MAX_NUM_IP+1))
#pragma DATA_SECTION (paBuf2, ".paBuf2")
#pragma DATA_ALIGN(paBuf2, CACHE_L2_LINESIZE)
uint8_t paBuf2[TEST_CONFIG_BUFSIZE_PA_BUF2];

#ifdef NWAL_ENABLE_SA
/* Memory used for SA LLD global Handle */
#define TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE    512
#pragma DATA_SECTION (salldHandle, ".salldHandle")
#pragma DATA_ALIGN(salldHandle, CACHE_L2_LINESIZE)
uint8_t salldHandle[TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE];

/* Memory used for SA LLD global Handle */
#define TEST_CONFIG_BUFSIZE_SA_CONTEXT_PER_CHAN             512
#pragma DATA_SECTION (saContext, ".saContext")
#pragma DATA_ALIGN(saContext, CACHE_L2_LINESIZE)
uint8_t saContext[TEST_CONFIG_BUFSIZE_SA_CONTEXT_PER_CHAN * 
                  (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)];

/* Memory used by SA LLD per Channel */
#define TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE_PER_CHAN          512
#pragma DATA_SECTION (salldChanHandle, ".salldChanHandle")
#pragma DATA_ALIGN(salldChanHandle, CACHE_L2_LINESIZE)
uint8_t salldChanHandle[TEST_CONFIG_BUFSIZE_SA_LLD_HANDLE_PER_CHAN * 
                        (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)];
#endif

/*****************************************************************************
 * Global Resources shared by all Cores
 *****************************************************************************/ 
#pragma DATA_SECTION (QMemGlobDescRam, ".QMemGlobDescRam");
#pragma DATA_ALIGN(QMemGlobDescRam, CACHE_L2_LINESIZE)
Uint8 QMemGlobDescRam[TEST_CONFIG_MAX_GLOB_DESC * TEST_CONFIG_GLOB_DESC_SIZE];

#pragma DATA_SECTION (cppiMemPaSaLinkBuf, ".cppiMemPaSaLinkBuf");
#pragma DATA_ALIGN(cppiMemPaSaLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemPaSaLinkBuf[TEST_CONFIG_MAX_PA_TO_SA_DESC][TEST_CONFIG_GLOB_BUF_SIZE];


#pragma DATA_SECTION (cppiMemSaPaLinkBuf, ".cppiMemSaPaLinkBuf");
#pragma DATA_ALIGN(cppiMemSaPaLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemSaPaLinkBuf[TEST_CONFIG_MAX_SA_TO_PA_DESC][TEST_CONFIG_GLOB_BUF_SIZE];


/*****************************************************************************
 * Local Resource allocated at each Core.Optimal memory placement would be L2D
 *****************************************************************************/ 
#pragma DATA_SECTION (QMemLocDescRam, ".QMemLocDescRam");
#pragma DATA_ALIGN(QMemLocDescRam, CACHE_L2_LINESIZE)

/* Descriptors in global shared */
Uint8 QMemLocDescRam[TEST_CONFIG_MAX_LOC_DESC * TEST_CONFIG_LOC_DESC_SIZE];



#pragma DATA_SECTION (cppiMemRxPktLinkBuf, ".cppiMemRxPktLinkBuf");
#pragma DATA_ALIGN(cppiMemRxPktLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemRxPktLinkBuf[TEST_CONFIG_MAX_PKT_RX_NBUFS][TEST_CONFIG_MAX_PKT_RX_BUF_SIZE];

#pragma DATA_SECTION (cppiMemTxPktLinkBuf, ".cppiMemTxPktLinkBuf");
#pragma DATA_ALIGN(cppiMemTxPktLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemTxPktLinkBuf[TEST_CONFIG_MAX_PKT_TX_NBUFS][TEST_CONFIG_MAX_PKT_TX_BUF_SIZE];

#pragma DATA_SECTION (cppiMemRxCtlLinkBuf, ".cppiMemRxCtlLinkBuf");
#pragma DATA_ALIGN(cppiMemRxCtlLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemRxCtlLinkBuf[TEST_CONFIG_MAX_CTL_RX_NBUFS][TEST_CONFIG_MAX_CTL_RX_BUF_SIZE];

#pragma DATA_SECTION (cppiMemTxCtlLinkBuf, ".cppiMemTxCtlLinkBuf");
#pragma DATA_ALIGN(cppiMemTxCtlLinkBuf, CACHE_L2_LINESIZE)
Uint8 cppiMemTxCtlLinkBuf[TEST_CONFIG_MAX_CTL_TX_NBUFS][TEST_CONFIG_MAX_CTL_TX_BUF_SIZE];

void        Osal_invalidateCache (void *blockPtr, uint32_t size);
void        Osal_writeBackCache (void *blockPtr, uint32_t size);


/*****************************************************************************
 * FUNCTION PURPOSE: Global Initialization of CPPI. Once Per System
 ***************************************************************************** 
 * DESCRIPTION: The function will initialize the CPPI
 *****************************************************************************/
static inline nwal_Bool_t testNwGlobCppiInit (void)
{
  int32_t          result;

  result = Cppi_init (&cppiGblCfgParams);
  if (result != CPPI_SOK)  {
    System_printf ("function testNwGlobCppiInit: Cppi_init failed with error code %d\n", result);
    return (nwal_FALSE);
  }
  return (nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Global Initialization of Queue Manager. Once Per System
 ***************************************************************************** 
 * DESCRIPTION: The function will initialize the Queue Manager
 *****************************************************************************/
nwal_Bool_t testNwGlobQmInit (void)
{
  Qmss_InitCfg     qmssInitConfig;
  int32_t          result;


  memset (&qmssInitConfig, 0, sizeof (Qmss_InitCfg));

  /* Use Internal Linking RAM for optimal performance */
  qmssInitConfig.linkingRAM0Base = 0;
  qmssInitConfig.linkingRAM0Size = 0;
  qmssInitConfig.linkingRAM1Base = 0;
  qmssInitConfig.maxDescNum      = TEST_CONFIG_MAX_DESC_NUM;

  result = Qmss_init (&qmssInitConfig, &qmssGblCfgParams);
  if (result != QMSS_SOK)  {
    System_printf ("function testNwGlobQmInit: qmss_Init failed with error code %d\n", result);
    return (nwal_FALSE);
  }
  return (nwal_TRUE);
}

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

  if(memRegion == TEST_CONFIG_BASE_QM_GLOBAL_MSMC_MEMORY_REGION)
  {
      memInfo.startIndex = 0;
  }else if(memRegion ==TEST_CONFIG_BASE_QM_GLOBAL_DDR_MEMORY_REGION)
  {
      /* Global shared memory for all descriptors to all cores */
      memInfo.startIndex     = TEST_CONFIG_MAX_GLOB_DESC;
  }
  else
  {
      memInfo.startIndex     = TEST_CONFIG_MAX_GLOB_DESC + (TEST_CONFIG_MAX_LOC_DESC_PER_CORE * DNUM);
  }  

  result = Qmss_insertMemoryRegion (&memInfo);
  if (result < QMSS_SOK)  {
    System_printf ("function testNwSetupQmMem: Qmss_insertMemoryRegion returned error code %d\n", result);
    return (nwal_FALSE);
  }

  return (nwal_TRUE);

}

/*****************************************************************************
 * FUNCTION PURPOSE: Setup the queues
 ***************************************************************************** 
 * DESCRIPTION: The function will setup all the queues used by the chip
 *****************************************************************************/
void testNwPktLibFree (uint8_t* ptrDataBuffer, uint32_t size)

{
    /* Not handled. Not expected as heap is created and is not freed */
    System_printf ("testNwPktLibFree()Called. No Actions being done %d \n");
    System_flush();
}

/*****************************************************************************
 * FUNCTION PURPOSE: MemAlloc Function
 ***************************************************************************** 
 * DESCRIPTION: Common Mem Alloc Function
 *****************************************************************************/
uint8_t*  testNwPktLibAlloc (uint8_t** ppBufCurBase,uint32_t* pAvailSize,uint32_t size)
{
    uint8_t* pBuf;

    if(size > *pAvailSize)
    {
        return NULL;
    }
    pBuf = *ppBufCurBase;
    *ppBufCurBase = pBuf + size;
    *pAvailSize = *pAvailSize - size;

    /* Return Global Address for the buffer */
    return((uint8_t*)Osal_nwalLocToGlobAddr((uint32_t)pBuf));
}  


/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for RX Control Heap passed to packet Lib
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for RX Control Heap passed to packet Lib
 *****************************************************************************/
uint8_t*  testNwPktLibRxCtlPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pRxCtlMem = 0;
    static uint32_t     rxCtlMemAvail = 0;

    if(!first)
    {
        /* First call */
#ifdef NWAL_TEST_DESC_GLOB_MEM        
        pRxCtlMem = cppiMemRxCtlLinkBuf[DNUM * TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE];
#else
        pRxCtlMem = cppiMemRxCtlLinkBuf[0];
#endif
        rxCtlMemAvail = 
            (TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE * TEST_CONFIG_MAX_CTL_RX_BUF_SIZE);
        first++;
    }
    return(testNwPktLibAlloc(&pRxCtlMem,&rxCtlMemAvail,size));
}

/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for TX Control Heap passed to packet Lib
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for TX Control Heap passed to packet Lib
 *****************************************************************************/
uint8_t*  testNwPktLibTxCtlPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pTxCtlMem = 0;
    static uint32_t     txCtlMemAvail = 0;

    if(!first)
    {
        /* First call */
#ifdef NWAL_TEST_DESC_GLOB_MEM        
        pTxCtlMem = cppiMemTxCtlLinkBuf[DNUM * TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE];
#else
        pTxCtlMem = cppiMemTxCtlLinkBuf[0];
#endif
        txCtlMemAvail = 
            (TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE * TEST_CONFIG_MAX_CTL_TX_BUF_SIZE);
        first++;
    }
    return(testNwPktLibAlloc(&pTxCtlMem,&txCtlMemAvail,size));
}


/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for RX Packet Heap passed to packet Lib
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for RX Packet Heap passed to packet Lib
 *****************************************************************************/
uint8_t*  testNwPktLibRxPktPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pRxPktMem = 0;
    static uint32_t     rxPktMemAvail = 0;
    uint8_t* pBuf;

    if(!first)
    {
        /* First call */
#ifdef NWAL_TEST_DESC_GLOB_MEM        
        pRxPktMem = cppiMemRxPktLinkBuf[DNUM * TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE];
#else
        pRxPktMem = cppiMemRxPktLinkBuf[0];
#endif
        rxPktMemAvail = 
            (TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE * TEST_CONFIG_MAX_PKT_RX_BUF_SIZE);
        first++;
    }
    pBuf = testNwPktLibAlloc(&pRxPktMem,&rxPktMemAvail,size);
    return(pBuf);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for TX Packet Heap passed to packet Lib
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for TX Packet Heap passed to packet Lib
 *****************************************************************************/
uint8_t*  testNwPktLibTxPktPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pTxPktMem = 0;
    static uint32_t     txPktMemAvail = 0;

    if(!first)
    {
        /* First call */
#ifdef NWAL_TEST_DESC_GLOB_MEM        
        pTxPktMem = cppiMemTxPktLinkBuf[DNUM * TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE];
#else
        pTxPktMem = cppiMemTxPktLinkBuf[0];
#endif
        txPktMemAvail = (TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE * TEST_CONFIG_MAX_PKT_TX_BUF_SIZE);
        first++;
    }
    return(testNwPktLibAlloc(&pTxPktMem,&txPktMemAvail,size));
}

/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for Global Packet Heap for
 *                   packets from PA to SA
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for Global Packet Heap for
 *              packets from PA to SA
 *****************************************************************************/
uint8_t*  testNwPktLibGlobPaSaPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pPktMem = 0;
    static uint32_t     pktMemAvail = 0;

    if(!first)
    {
        /* First call */
        pPktMem = cppiMemPaSaLinkBuf[0];
        pktMemAvail = ((TEST_CONFIG_MAX_PA_TO_SA_DESC) *  TEST_CONFIG_GLOB_BUF_SIZE);
        first++;
    }
    return(testNwPktLibAlloc(&pPktMem,&pktMemAvail,size));
}

/*****************************************************************************
 * FUNCTION PURPOSE: Memory Allocation for Global Packet Heap for
 *                   packets from SA to PA
 ***************************************************************************** 
 * DESCRIPTION: Memory Allocation for Global Packet Heap for
 *              packets from SA to PA
 *****************************************************************************/
uint8_t*  testNwPktLibGlobSaPaPoolAlloc (uint32_t size)
{
    static uint8_t      first=0;
    static uint8_t *    pPktMem = 0;
    static uint32_t     pktMemAvail = 0;

    if(!first)
    {
        /* First call */
        pPktMem = cppiMemSaPaLinkBuf[0];
        pktMemAvail = ((TEST_CONFIG_MAX_SA_TO_PA_DESC) * TEST_CONFIG_GLOB_BUF_SIZE);
        first++;
    }
    return(testNwPktLibAlloc(&pPktMem,&pktMemAvail,size));
}

/***************************************************************************************
 * FUNCTION PURPOSE: NWAL start per core
 ***************************************************************************************
 * DESCRIPTION: function to start NWAL in a core
 ***************************************************************************************/
nwal_Bool_t testNwalStart(void)
{
    nwalLocCfg_t        nwalLocCfg;
    nwal_RetValue       nwalRetVal;
    char                heapName[50];
    Qmss_MemRegion      memRegion;
    Pktlib_HeapCfg      heapCfg;
    int32_t             errCode;
    uint32_t            lockKey;

    memset(&nwalLocCfg,0,sizeof(nwalLocCfg));
    memset(&testNwLocContext,0,sizeof(testNwLocContext_t));

#ifndef NWAL_TEST_DESC_GLOB_MEM
    /* Initialize Descriptors to be used for all local core related resources */
    if(testNwSetupQmMem(TEST_CONFIG_MAX_LOC_DESC_PER_CORE, 
                        TEST_CONFIG_LOC_DESC_SIZE,
                        (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)QMemLocDescRam),
                        (Qmss_MemRegion)(TEST_CONFIG_BASE_QM_MEMORY_REGION + DNUM ),
                        &testNwLocContext.descQ)!= nwal_TRUE)
    {
        System_printf ("testNwalStart:QM Memory region/Descriptor Initialization for Local failed\n");
        return nwal_FALSE;
    }
#endif

#ifdef NWAL_TEST_DESC_GLOB_MEM
    memRegion = TEST_CONFIG_BASE_QM_GLOBAL_DDR_MEMORY_REGION;
#else
    memRegion = (Qmss_MemRegion)(TEST_CONFIG_BASE_QM_MEMORY_REGION + DNUM );
#endif

    /* Call back registration for the core */
    nwalLocCfg.pRxPktCallBack = testNWALRxPktCallback;
    nwalLocCfg.pCmdCallBack = testNWALCmdCallBack;
    nwalLocCfg.pPaStatsCallBack = testNWALCmdPaStatsReply;
    nwalLocCfg.pRxDmCallBack = testNWALRxDmBack;
#ifdef TEST_NWAL_ENABLE_PA_ASSISTED_REASSEMBLY
    nwalGlobCfg.pRxReassemProc = testNWALRxReassemProc;
#endif
    /* Initialize Buffer Pool for Control packets from NetCP to Host */    
    nwalLocCfg.rxCtlPool.numBufPools = 1;
    nwalLocCfg.rxCtlPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.rxCtlPool.bufPool[0].bufSize = TEST_CONFIG_MAX_CTL_RX_BUF_SIZE;

    sprintf(heapName,"RX Ctl%d",DNUM);

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));
    /* Populate the heap configuration */
    heapCfg.name                = heapName;
    heapCfg.memRegion           = memRegion;
    heapCfg.sharedHeap          = 0;
    heapCfg.useStarvationQueue  = 0;
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_CTL_RX_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE;
    heapCfg.numZeroBufferPackets= 0;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibRxCtlPoolAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    heapCfg.dataBufferPktThreshold   = 0;
    heapCfg.zeroBufferPktThreshold   = 0;


    nwalLocCfg.rxCtlPool.bufPool[0].heapHandle = 
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.rxCtlPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for RX Ctl From NetCP, Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Control packets from Host to NetCP */    
    nwalLocCfg.txCtlPool.numBufPools = 1;
    nwalLocCfg.txCtlPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.txCtlPool.bufPool[0].bufSize = TEST_CONFIG_MAX_CTL_TX_BUF_SIZE;

    
    sprintf(heapName,"TX Ctl%d",DNUM);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_CTL_TX_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibTxCtlPoolAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;

   
    nwalLocCfg.txCtlPool.bufPool[0].heapHandle = 
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.txCtlPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for TX Ctl To NetCP , Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Packets from NetCP to Host */    
    nwalLocCfg.rxPktPool.numBufPools = 1;
    nwalLocCfg.rxPktPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.rxPktPool.bufPool[0].bufSize = TEST_CONFIG_MAX_PKT_RX_BUF_SIZE;

    
    sprintf(heapName,"Rx Pkt%d",DNUM);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_PKT_RX_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibRxPktPoolAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;

    nwalLocCfg.rxPktPool.bufPool[0].heapHandle =  Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.rxPktPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for Rx Pkt From NetCP, Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for Packets from Host to NetCP */    
    nwalLocCfg.txPktPool.numBufPools = 1;
    nwalLocCfg.txPktPool.bufPool[0].descSize = TEST_CONFIG_LOC_DESC_SIZE;
    nwalLocCfg.txPktPool.bufPool[0].bufSize = TEST_CONFIG_MAX_PKT_TX_BUF_SIZE;

    sprintf(heapName,"Tx Pkt%d",DNUM);
    heapCfg.dataBufferSize      = TEST_CONFIG_MAX_PKT_TX_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibTxPktPoolAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    nwalLocCfg.txPktPool.bufPool[0].heapHandle = 
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalLocCfg.txPktPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for TX Pkt To NetCP , Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }
    
    memcpy(&testNwLocContext.nwalLocCfg,&nwalLocCfg,sizeof(nwalLocCfg_t));
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
    
    Osal_nwalCsEnter(&lockKey);
    Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
    testNwGlobContext.numCoresStarted++;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));
    Osal_nwalCsExit(lockKey);
    return nwal_TRUE;
}


/*****************************************************************************
 * FUNCTION PURPOSE: Global Initializion of the Network  related resources
 ***************************************************************************** 
 * DESCRIPTION: Global Initializion of the Network  related resources
 *****************************************************************************/
nwal_Bool_t testNwGlobalInit (void)
{
    nwalGlobCfg_t       nwalGlobCfg;
    nwalSizeInfo_t      nwalSizeInfo;
    nwal_RetValue       nwalRetVal;
    uint8_t             count;
    int                 sizes[nwal_N_BUFS];
    int                 aligns[nwal_N_BUFS];
    void*               bases[nwal_N_BUFS];
    Pktlib_HeapCfg      heapCfg;
    int32_t             errCode;     
    char                heapName[50];
    int i;
    nwal_Bool_t         enableSA = nwal_FALSE;
    /* Global Network resource QM/CPPI/PA related initialization to
     * be done by Core 0
     */
    memset(&testNwGlobContext,0,sizeof(testNwGlobContext_t));
    memset(&nwalGlobCfg,0,sizeof(nwalGlobCfg_t));
    memset(sizes,0,sizeof(sizes));
    memset(aligns,0,sizeof(aligns));
    memset(bases,0,sizeof(bases));

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
    /* Initialize Global Descriptors */
    if(testNwSetupQmMem(TEST_CONFIG_MAX_GLOB_DESC, 
                     TEST_CONFIG_GLOB_DESC_SIZE,
                     (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)QMemGlobDescRam),
                     TEST_CONFIG_BASE_QM_GLOBAL_MSMC_MEMORY_REGION,
                     &testNwGlobContext.descQ)!= nwal_TRUE)
    {
        System_printf ("testNwGlobalInit:QM Memory region/Descriptor Initialization for Global per System failed\n");
        return nwal_FALSE;
    }

#ifdef NWAL_TEST_DESC_GLOB_MEM
    /* Initialize descriptor for packet and control processing at each core from global shared external memory */
    /* Initialize Global Descriptors */
    if(testNwSetupQmMem(TEST_CONFIG_MAX_LOC_DESC, 
                        TEST_CONFIG_LOC_DESC_SIZE,
                        (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)QMemLocDescRam),
                        TEST_CONFIG_BASE_QM_GLOBAL_DDR_MEMORY_REGION,
                        &testNwGlobContext.descExtMemQ)!= nwal_TRUE)
    {
        System_printf ("testNwGlobalInit:QM Memory region/Descriptor Initialization for per core descriptors failed\n");
        return nwal_FALSE;
    }

#endif
    /* Initialize Buffer Pool for NetCP PA to SA packets */    
    nwalGlobCfg.pa2SaBufPool.numBufPools = 1;
    nwalGlobCfg.pa2SaBufPool.bufPool[0].descSize = TEST_CONFIG_GLOB_DESC_SIZE;
    nwalGlobCfg.pa2SaBufPool.bufPool[0].bufSize = TEST_CONFIG_GLOB_BUF_SIZE;

    /* Initialize the heap configuration. */
    memset ((void *)&heapCfg, 0, sizeof(Pktlib_HeapCfg));
    /* Populate the heap configuration */
    
    sprintf(heapName,"nwal PA2SA%d",DNUM);
    heapCfg.name                = heapName;
    heapCfg.memRegion           = TEST_CONFIG_BASE_QM_GLOBAL_MSMC_MEMORY_REGION;
    heapCfg.sharedHeap          = 0;
    heapCfg.useStarvationQueue  = 0;
    heapCfg.dataBufferSize      = TEST_CONFIG_GLOB_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_PA_TO_SA_DESC;
    heapCfg.numZeroBufferPackets= 0;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibGlobPaSaPoolAlloc;
    heapCfg.heapInterfaceTable.data_free    = testNwPktLibFree;
    heapCfg.dataBufferPktThreshold   = 0;
    heapCfg.zeroBufferPktThreshold   = 0;

    nwalGlobCfg.pa2SaBufPool.bufPool[0].heapHandle = 
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalGlobCfg.pa2SaBufPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for PA to SA Buffer Pool, Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }

    /* Initialize Buffer Pool for NetCP SA to PA packets */ 
    sprintf(heapName,"nwal PA2SA%d",DNUM);
    heapCfg.dataBufferSize      = TEST_CONFIG_GLOB_BUF_SIZE;
    heapCfg.numPkts             = TEST_CONFIG_MAX_SA_TO_PA_DESC;
    heapCfg.heapInterfaceTable.data_malloc  = testNwPktLibGlobSaPaPoolAlloc;

    nwalGlobCfg.sa2PaBufPool.numBufPools = 1;
    nwalGlobCfg.sa2PaBufPool.bufPool[0].descSize = TEST_CONFIG_GLOB_DESC_SIZE;    
    nwalGlobCfg.sa2PaBufPool.bufPool[0].bufSize = TEST_CONFIG_GLOB_BUF_SIZE;

    nwalGlobCfg.sa2PaBufPool.bufPool[0].heapHandle =         
        Pktlib_createHeap(&heapCfg, &errCode);
    if(nwalGlobCfg.sa2PaBufPool.bufPool[0].heapHandle == NULL)
    {
        System_printf ("Pktlib_createHeap:Heap Creation Failed for SA to PA Buffer Pool, Error Code: %d\n",errCode);
        System_flush();
        return nwal_FALSE;
    }
    
    nwalGlobCfg.hopLimit = 5;/* Default TTL / Hop Limit */
    nwalGlobCfg.paPowerOn = nwal_FALSE;
    nwalGlobCfg.saPowerOn = nwal_FALSE;
    nwalGlobCfg.paFwActive = nwal_FALSE;
    nwalGlobCfg.saFwActive = nwal_FALSE;

#ifdef TEST_NWAL_ENABLE_PA_ASSISTED_REASSEMBLY
    nwalGlobCfg.enablePAAssistReassem = nwal_TRUE;
#endif

    /* Pick Default Physical Address */
    //nwalGlobCfg.paVirtBaseAddr = 0x0;
    //nwalGlobCfg.saVirtBaseAddr = 0x0;
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

    /* Check for memory size requirement and update the base */
    count = 0;
    bases[nwal_BUF_INDEX_INST] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)nwalInstMem);
    if(TEST_CONFIG_BUFSIZE_NWAL_HANDLE < sizes[nwal_BUF_INDEX_INST])
    { 
        /* Resize Memory */
        while(1);
    }
    count++;

    bases[nwal_BUF_INDEX_INT_HANDLES] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)nwalHandleMem);
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
    
    bases[nwal_BUF_INDEX_SA_CONTEXT] = (uint32_t *)Osal_nwalLocToGlobAddr((uint32_t)saContext);
    if((TEST_CONFIG_BUFSIZE_SA_CONTEXT_PER_CHAN * (TEST_MAX_NUM_IPSEC_CHANNELS + TEST_MAX_NUM_DM_SA_CHANNELS)) < 
        sizes[nwal_BUF_INDEX_SA_CONTEXT])
    { 
        /* Resize Memory */
        while(1);
    }
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
    
    /* Initialize NWAL module */
    nwalRetVal = nwal_create(&nwalGlobCfg,
                             &nwalSizeInfo,
                             sizes,
                             bases,
                             &testNwGlobContext.nwalInstHandle);
    if(nwalRetVal != nwal_OK)
    {
        System_printf ("testNwGlobalInit:nwal_create Failed %d\n",nwalRetVal);
        System_flush();
        while(1);
    }
   /* Initialize the NETCP Modules */
    enableSA =
        (nwalSizeInfo.nMaxIpSecChannels ||
         nwalSizeInfo.nMaxDmSecChannels) ? nwal_TRUE : nwal_FALSE;
#if 0
    nwalRetVal = testNwPassPowerUp((nwalGlobContext_t*)testNwGlobContext.nwalInstHandle,enableSA);
    if(nwalRetVal != nwal_OK)
    {
        return (nwalRetVal);
    }
#endif
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
    System_printf("CORE: %d Global and Local Network initialization Successful \n",DNUM);

    testNwGlobContext.state = TEST_NW_CXT_GLOB_ACTIVE;
    Osal_writeBackCache(&testNwGlobContext,sizeof(testNwGlobContext));
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
      Osal_invalidateCache(&testNwGlobContext,sizeof(testNwGlobContext));
      /* Invalidate the Global context */
      if(testNwGlobContext.state == TEST_NW_CXT_GLOB_INACTIVE)
      {
          continue;
      }

      /* Block until Qmss_init() has completed by core 0 */
      result = Qmss_start();
      if(result == QMSS_NOT_INITIALIZED)
      {
          System_printf ("QMSS Not yet Initialized Waiting for Core %d\n", DNUM);
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
    uint16_t            count=0;

    /* Common Initialization for all cores */
    memset(&testNwLocContext,0,sizeof(testNwLocContext_t));
    while(count < TEST_MAX_NUM_TRANS)
    {
        testNwLocContext.transInfos[count].transId = count;
        count++;
    }

    #if 1
    /* Enable the L2 Cache */
    CACHE_setL2Size(CACHE_64KCACHE);
    if(CACHE_getL2Size() != CACHE_64KCACHE)
    {
        while(1);
    }
    #else
    CACHE_setL2Size(CACHE_0KCACHE);
    if(CACHE_getL2Size() != CACHE_0KCACHE)
    {
        while(1);
    }
   #endif
    

    if(!DNUM)
    {
        /* Switch Initialization */
#ifndef SIMULATOR_SUPPORT
        testNwCPSWInit(nwal_FALSE);
#endif
        /* Global Network resource QM/CPPI/PA related initialization to
         * be done by Core 0
         */
        if(!testNwGlobalInit())
        {
            System_printf("ERROR: Global Network initialization Failed \n");
            return nwal_FALSE;
        }        
    }
    else
    {        
        if(!testNwLocaInit())
        {
            System_printf("CORE: %d ERROR: Local Network initialization Failed \n",DNUM);
            return nwal_FALSE;
        }
        System_printf("CORE: %d Local Network initialization Successful \n",DNUM);
    }    
    
    /* Start NWAL on the core doing global initialization */
    if(testNwalStart() != nwal_TRUE)
    {
        System_printf ("CORE: %d testNwGlobalInit:testNwalStart Failed \n",DNUM);
        while(1);
    }

    testNwLocContext.state = TEST_NW_CXT_LOC_ACTIVE;

    return 0;
}
