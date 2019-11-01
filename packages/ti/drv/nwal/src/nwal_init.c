/******************************************************************************
 * FILE PURPOSE:  NWAL LLD Initialization functions
 ******************************************************************************
 * FILE NAME:   nwal_int.c
 *
 * DESCRIPTION: NWAL LLD initialization routines
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2012
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



#include <string.h>
#include "nwal_util.h"
#include "nwal_osal.h"
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/csl/csl_psc.h>
#include <stdio.h>
/* Firmware images */

#include <ti/drv/qmss/qmss_firmware.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_qm_queue.h>
#include <ti/csl/cslr_device.h>

#if defined(DEVICE_K2H) || defined(SOC_K2H)
#include <ti/drv/pa/device/k2h/src/nss_device.c>
#elif defined (DEVICE_K2K) || defined(SOC_K2K)
#include <ti/drv/pa/device/k2k/src/nss_device.c>
#elif defined (DEVICE_K2L) || defined(SOC_K2L)
#include <ti/drv/pa/device/k2l/src/nss_device.c>
#elif defined (DEVICE_K2E) || defined(SOC_K2E)
#include <ti/drv/pa/device/k2e/src/nss_device.c>
#elif defined (SOC_C6678)
#include <ti/drv/pa/device/c6678/src/nss_device.c>
#else /*Default */
#include <ti/drv/pa/device/k2h/src/nss_device.c>
#endif /* Device */


nwalLocContext_t*        gPLocContext = NULL;
#ifdef _TMS320C6X
#pragma DATA_SECTION (nwalProcessCtx, ".nwalProcessCtx");
#pragma DATA_ALIGN(nwalProcessCtx, 128)
#endif
nwalProcessContext_t     nwalProcessCtx;

/********************************************************************
 * FUNCTION PURPOSE: API to return buffer requirement from NWAL
 ********************************************************************
 * DESCRIPTION: API to return buffer requirement from NWAL
 ********************************************************************/
nwal_RetValue nwal_getBufferReq(nwalSizeInfo_t* pSizeInfo,
                                int             sizes[nwal_N_BUFS],
                                int             aligns[nwal_N_BUFS])
{
    paSizeInfo_t        paSizeCfg;
    int                 paSizes[pa_N_BUFS];
    int                 paAligns[pa_N_BUFS];
    int                 ret;
    uint16_t            cacheLineSize = NWAL_CACHE_LINE_SIZE ;
#ifdef NWAL_ENABLE_SA
    nwal_RetValue       nwalRetVal;
#endif


    memset(paSizes,0,sizeof(paSizes));

    sizes[nwal_BUF_INDEX_INST] =
        nwal_round_size(cacheLineSize,1,sizeof(nwalGlobContext_t));
    aligns[nwal_BUF_INDEX_INST] = cacheLineSize;

    /* BEGIN: Buffer for storing MAC/IPSec/IP/Port/TX Header Handles
     * Individual handles are aligned to cache size to avoid False
     * Sharing for multicore
     */
    sizes[nwal_BUF_INDEX_INT_HANDLES] =
        (pSizeInfo->nMaxMacAddress *
        (nwal_round_size(cacheLineSize,1,sizeof(nwalMacInfo_t))));

    /* Add Memory for IpSec channel */
    if(pSizeInfo->nMaxIpSecChannels)
#ifdef NWAL_ENABLE_SA
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
        (pSizeInfo->nMaxIpSecChannels *
        nwal_round_size(cacheLineSize, 1, sizeof(nwalIpSecInfo_t)));
#else
    return (nwal_ERR_SA_NOT_ENABLED);
#endif

    /* Add Memory for Data Mode SA channel */
    if(pSizeInfo->nMaxDmSecChannels)
#ifdef NWAL_ENABLE_SA
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
        (pSizeInfo->nMaxDmSecChannels *
        nwal_round_size(cacheLineSize, 1, sizeof(nwalDmSaInfo_t)));
#else
    return (nwal_ERR_SA_NOT_ENABLED);
#endif


    /* Add Memory for IP channel */
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
         (pSizeInfo->nMaxIpAddress *
         (nwal_round_size(cacheLineSize,1,sizeof(nwalIpInfo_t))));

    /* Add Memory for L4 ports */
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
         (pSizeInfo->nMaxL4Ports *
         (nwal_round_size(cacheLineSize,1,sizeof(nwalPortInfo_t))));

    /* Add Memory for TX Header buffer */
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
         (pSizeInfo->nMaxL2L3Hdr *
         (nwal_round_size(cacheLineSize,1,sizeof(nwalL2L3HdrInfo_t))));

     /* Add Memory for per Proc Context */
    sizes[nwal_BUF_INDEX_INT_HANDLES] +=
         (pSizeInfo->nProc *
         (nwal_round_size(cacheLineSize,1,sizeof(nwalLocContext_t))));

    aligns[nwal_BUF_INDEX_INT_HANDLES] = cacheLineSize;
    /* END: Buffer for storing MAC/IPSec/IP/Port/TX Header Handles */

    /* PA resources to be initialized by NWAL */
    memset(&paSizeCfg,0,sizeof(paSizeInfo_t));
    paSizeCfg.nMaxL2 = pSizeInfo->nMaxMacAddress;
    paSizeCfg.nMaxL3 =
        pSizeInfo->nMaxIpAddress + pSizeInfo->nMaxIpSecChannels;
    paSizeCfg.nUsrStats = 0;
    memset(paSizes,0,sizeof(paSizes));
    if((!pSizeInfo->pahandle) &&
       ((paSizeCfg.nMaxL2) || (paSizeCfg.nMaxL3)))
    {
        ret = Pa_getBufferReq(&paSizeCfg, paSizes, paAligns);
        if (ret != pa_OK)  {
            return (nwal_ERR_PA);
        }
        /* Update the memory buffer requirement for PA
         * Individual elements are not padded to cache size
         * considering multicore protection and prevent
         * false sharing
         */
        sizes[nwal_BUF_INDEX_PA_LLD_BUF0] =
            (nwal_round_size(cacheLineSize,1,paSizes[0]));
        if(paAligns[pa_BUF_INST] > cacheLineSize)
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF0] = paAligns[0];
        }
        else
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF0] = cacheLineSize;
        }

        sizes[nwal_BUF_INDEX_PA_LLD_BUF1] =
            (nwal_round_size(cacheLineSize,1,paSizes[1]));
        if(paAligns[pa_BUF_L2_TABLE] > cacheLineSize)
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF1] = paAligns[1];
        }
        else
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF1] = cacheLineSize;
        }

        sizes[nwal_BUF_INDEX_PA_LLD_BUF2] =
            (nwal_round_size(cacheLineSize,1,paSizes[2]));
        if(paAligns[pa_BUF_L3_TABLE] > cacheLineSize)
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF2] = paAligns[2];
        }
        else
        {
            aligns[nwal_BUF_INDEX_PA_LLD_BUF2] = cacheLineSize;
        }
        if(paSizes[pa_BUF_USR_STATS_TABLE])
        {
            /* Unexpected as user stats not enabled */
            return (nwal_ERR_PA);
        }
        if(pa_N_BUFS > (pa_BUF_USR_STATS_TABLE+1))
        {
            if(paSizes[pa_BUF_USR_STATS_TABLE+1])
            {
                /* Unexpected as user stats not enabled */
                return (nwal_ERR_PA);
            }
        }
    }
    else
    {
        /* PA Resources are initialized outside NWAL.*/
         sizes[nwal_BUF_INDEX_PA_LLD_BUF0] =  0;
         aligns[nwal_BUF_INDEX_PA_LLD_BUF0] = 0 ;
         sizes[nwal_BUF_INDEX_PA_LLD_BUF1] =  0;
         aligns[nwal_BUF_INDEX_PA_LLD_BUF1] = 0 ;
         sizes[nwal_BUF_INDEX_PA_LLD_BUF2] =  0;
         aligns[nwal_BUF_INDEX_PA_LLD_BUF2] = 0 ;
    }

#ifdef NWAL_ENABLE_SA
    nwalRetVal = nwal_getSaBufferReq(pSizeInfo,cacheLineSize,sizes,aligns);
    if(nwalRetVal != nwal_OK)
        return nwalRetVal;
#endif

    return (nwal_OK);

}

/************************************************************************
 * FUNCTION PURPOSE: API to return buffer requirement from NWAL local
 *                   contexts.
 ************************************************************************
 * DESCRIPTION: API to return buffer requirement from NWAL local contexts
 ************************************************************************/
nwal_RetValue nwal_getLocContextBufferReq(int nThreads, uint32_t* pSize)
{
    /* Add Memory for per Proc Context */
    *pSize = nThreads  *
        nwal_round_size(NWAL_CACHE_LINE_SIZE,1,sizeof(nwalLocContext_t));

    return nwal_OK;
}

/********************************************************************s**********
 * FUNCTION PURPOSE: Configure power domain
 ******************************************************************************
 * DESCRIPTION: function used to power on a domain and the module clock.
 *              Power domain is OFF by default. It needs to be turned on
 *              before doing any  device register access.
 *
 * Input:
 *        uint16_t psc_pd    : PSC Power Domain Assignment  (PSC_PD)
 *        uint16_t psc_lpsc  : PSC LPSC Module Assignment   (PSC_LPSC)
 *****************************************************************************/
nwal_RetValue nwal_DomainPowerUp(uint16_t psc_pd, uint16_t psc_lpsc)
{

    /* Set Power domain to ON */
    CSL_PSC_enablePowerDomain (psc_pd);
  
    /* Enable the clocks */
    CSL_PSC_setModuleNextState (psc_lpsc, PSC_MODSTATE_ENABLE);
  
    /* Start the state transition */
    CSL_PSC_startStateTransition (psc_pd);
  
    /* Wait until the state transition process is completed. */
    while (!CSL_PSC_isStateTransitionDone (psc_pd));
  
    /* Return PSC status */
    if ((CSL_PSC_getPowerDomainState(psc_pd) == PSC_PDSTATE_ON) &&
        (CSL_PSC_getModuleState (psc_lpsc) == PSC_MODSTATE_ENABLE))
    {
        /* Power ON. Ready for use */
        return nwal_OK;
    }
    else
    {
        /* Power on failed. Return error */
        return nwal_ERR_POWER_DOMAIN_FAIL;
    }
}

/******************************************************************************
 * FUNCTION PURPOSE: Power on PASS domains
 ******************************************************************************
 * DESCRIPTION: Function used to power up all necessary PASS domains
 *****************************************************************************/
nwal_RetValue nwal_PassPowerUp(nwalGlobContext_t*   pIhandle,
                               nwal_Bool_t          enableSA)
{
    nwal_RetValue retVal = nwal_OK;
  
    if(pIhandle->cfg.paPowerOn == nwal_FALSE)
    {
        retVal = nwal_DomainPowerUp(CSL_PSC_PD_NETCP,
                                     CSL_PSC_LPSC_PA);
        if(retVal != nwal_OK)
        {
          return retVal;
        }
        retVal = nwal_DomainPowerUp(CSL_PSC_PD_NETCP,
                                     CSL_PSC_LPSC_CPGMAC);
        if(retVal != nwal_OK)
        {
          return retVal;
        }
    }
    
    if(enableSA)
    {
        if(pIhandle->cfg.saPowerOn == nwal_FALSE)
        {
            retVal = nwal_DomainPowerUp(CSL_PSC_PD_NETCP,
                                         CSL_PSC_LPSC_SA);
            if(retVal != nwal_OK)
            {
              return retVal;
            }
        }
    }
    return retVal;
}

/*****************************************************************************
 * FUNCTION PURPOSE:    Download PA firmware
 *****************************************************************************
 * DESCRIPTION:         Download PA firmware
 *****************************************************************************/
nwal_RetValue nwal_PaDownloadFirmware (nwalGlobContext_t*   pIhandle)
{
    paSSstate_t   state;
    uint8_t i;
    
    state = Pa_resetControl (pIhandle->memSizeInfo.pahandle, pa_STATE_RESET);
    if(state != pa_STATE_RESET)
    {
        return (nwal_ERR_PA_DOWNLOAD);
    }

    for (i = 0; i < NSS_PA_NUM_PDSPS; i++)
    {
        if (Pa_downloadImage (pIhandle->memSizeInfo.pahandle,
                              i,
                              (Ptr)nssGblCfgParams.layout.paPdspImage[i],
                              nssGblCfgParams.layout.paPdspImageSize[i]) != pa_OK)
        {
                return (nwal_ERR_PA_DOWNLOAD);
        }
    }
    /* Enable the PA back */
    state = Pa_resetControl (pIhandle->memSizeInfo.pahandle,
                           pa_STATE_ENABLE);
    if(state != pa_STATE_ENABLE)
    {
        return (nwal_ERR_PA_DOWNLOAD);
    }
    return (nwal_OK);
}


/*****************************************************************************
 * FUNCTION PURPOSE:    Start the PA
 *****************************************************************************
 * DESCRIPTION:         Start the PA
 *****************************************************************************/
nwal_RetValue nwal_StartPa (nwalGlobContext_t*      pIhandle,
                            Pa_Handle paHandle)
{
  paStartCfg_t paCfg;
  memset(&paCfg, 0, sizeof(paStartCfg_t));
#ifndef NWAL_DISABLE_LINUX_MULTI_PROC_PA
  paCfg.baseAddr = (uint32_t)nwalProcessCtx.baseAddrCfg.paVirtBaseAddr;
  paCfg.instPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pInstPoolPaBaseAddr;
#endif
  Pa_startCfg(paHandle, &paCfg);  
  return (nwal_OK);
}

/*****************************************************************************
 * FUNCTION PURPOSE:    Setup the PA
 *****************************************************************************
 * DESCRIPTION:         Setup the PA
 *****************************************************************************/
nwal_RetValue nwal_SetupPa (nwalGlobContext_t*      pIhandle,
                            int                     sizes[nwal_N_BUFS],
                            void*                   bases[nwal_N_BUFS])
{
  paSizeInfo_t  paSizeCfg;
  paConfig_t    paCfg;
  int           ret;
  int           paSizes[pa_N_BUFS];
  int           paAligns[pa_N_BUFS];
  void*         paBases[pa_N_BUFS];

  memset(&paSizeCfg,0,sizeof(paSizeInfo_t));
  memset(&paCfg,0,sizeof(paConfig_t));
  paSizeCfg.nMaxL2 = pIhandle->memSizeInfo.nMaxMacAddress;
  paSizeCfg.nMaxL3 = pIhandle->memSizeInfo.nMaxIpAddress + \
                     pIhandle->memSizeInfo.nMaxIpSecChannels;
  paSizeCfg.nUsrStats = 0;
  memset(paSizes,0,sizeof(paSizes));
  ret = Pa_getBufferReq(&paSizeCfg, paSizes, paAligns);
  if (ret != pa_OK)
  {
      pIhandle->extErr = ret;
      return (nwal_ERR_PA);
  }

  paBases[0] = (void *)(bases[nwal_BUF_INDEX_PA_LLD_BUF0]);
  /* Verify and assign the buffers */
  if ((uint32_t)(paBases[pa_BUF_INST]) & (paAligns[pa_BUF_INST] - 1))
  {
      /* Alignment does not match to what PA had requested */
      return (nwal_ERR_PA);
  }
  if (sizes[nwal_BUF_INDEX_PA_LLD_BUF0] < paSizes[pa_BUF_INST])
  {
      /* Unexpected Size */
      return (nwal_ERR_PA);
  }
  pIhandle->memSizeInfo.pahandle =
      (Pa_Handle)bases[nwal_BUF_INDEX_PA_LLD_BUF0];

  /* The second buffer is the L2 table */
  paBases[pa_BUF_L2_TABLE] = (void *)(bases[nwal_BUF_INDEX_PA_LLD_BUF1]);
  if ((uint32_t)paBases[pa_BUF_L2_TABLE] & (paAligns[pa_BUF_L2_TABLE] - 1))
  {
      /* Alignment does not match to what PA had requested */
      return (nwal_ERR_PA);
  }
  if (sizes[nwal_BUF_INDEX_PA_LLD_BUF1] < paSizes[pa_BUF_L2_TABLE])
  {
      /* Unexpected Size */
      return (nwal_ERR_PA);
  }

  paBases[pa_BUF_L3_TABLE] = (void *)(bases[nwal_BUF_INDEX_PA_LLD_BUF2]);
  /* The third buffer is the L3 table */
  if ((uint32_t)(paBases[pa_BUF_L3_TABLE]) & (paAligns[pa_BUF_L3_TABLE] - 1))
  {
      /* Alignment does not match to what PA had requested */
    return (nwal_ERR_PA);
  }
  if (sizes[nwal_BUF_INDEX_PA_LLD_BUF2] < paSizes[pa_BUF_L3_TABLE])
  {
      /* Unexpected Size */
      return (nwal_ERR_PA);
  }

  /* Fourth Buffer User table Not expected */
  if(paSizes[pa_BUF_USR_STATS_TABLE])
  {
      /* Unexpected Size */
      return (nwal_ERR_PA);
  }

  paCfg.initTable = nwal_TRUE;

  paCfg.initDefaultRoute = !pIhandle->cfg.paFwActive;


  /* Update base address to virtual adddress if passed by application */
  if(nwalProcessCtx.baseAddrCfg.paVirtBaseAddr)
  {
      paCfg.baseAddr = (uint32_t)nwalProcessCtx.baseAddrCfg.paVirtBaseAddr;
  }
  else
  {
      paCfg.baseAddr = CSL_NETCP_CFG_REGS;
  }

  paCfg.sizeCfg = &paSizeCfg;
#ifndef NWAL_DISABLE_LINUX_MULTI_PROC_PA
  paCfg.instPoolBaseAddr = nwalProcessCtx.baseAddrCfg.pInstPoolPaBaseAddr;
#endif
  ret = Pa_create (&paCfg, paBases, &pIhandle->memSizeInfo.pahandle);
  if (ret != pa_OK)
  {
      pIhandle->extErr = ret;
      return (nwal_ERR_PA);
  }

  /* Download the firmware if not aleady downloaded */
  if(pIhandle->cfg.paFwActive == nwal_FALSE)
  {
      if (nwal_PaDownloadFirmware (pIhandle))
        return (nwal_ERR_PA);
  }
  return (nwal_OK);
}


/*****************************************************************************
 * FUNCTION PURPOSE: Initialize the CPDMA
 *****************************************************************************
 * DESCRIPTION: The function will initialize the CPPI CPDMA resources for PA
 *              Function assumes CPPI Init is already done in application
 *****************************************************************************/
nwal_RetValue nwal_SetupCpdma (nwalGlobContext_t*   pIhandle)
{
  Cppi_CpDmaInitCfg cpdmaCfg;
  Cppi_RxChInitCfg  rxChCfg;
  Cppi_TxChInitCfg  txChCfg;
  int               i;
  uint8_t           isAlloc;

  memset(&cpdmaCfg,0,sizeof(Cppi_CpDmaInitCfg));
  cpdmaCfg.dmaNum   = Cppi_CpDma_PASS_CPDMA;
  pIhandle->cppiHandle = Cppi_open (&cpdmaCfg);
  if (pIhandle->cppiHandle == NULL)
  {
    return (nwal_ERR_CPPI);
  }
  /* Open all CDMA RX channels for PASS */
  memset(&rxChCfg,0,sizeof(Cppi_RxChInitCfg));
  rxChCfg.rxEnable = Cppi_ChState_CHANNEL_DISABLE;

  for (i = 0; i < NSS_NUM_RX_PKTDMA_CHANNELS; i++)
  {
    rxChCfg.channelNum = i;
    pIhandle->rxChHnd[i] =
        Cppi_rxChannelOpen (pIhandle->cppiHandle, &rxChCfg, &isAlloc);
    

    if (pIhandle->rxChHnd[i] == NULL)
    {
        continue;
    }
    else
    {
        Cppi_channelEnable (pIhandle->rxChHnd[i]);
    }
  }

  /* Open all tx channels.  */
  memset(&txChCfg,0,sizeof(Cppi_TxChInitCfg));
  txChCfg.priority     = 2;
  txChCfg.txEnable     = Cppi_ChState_CHANNEL_DISABLE;
  txChCfg.filterEPIB   = nwal_FALSE;
  txChCfg.filterPS     = nwal_FALSE;
  txChCfg.aifMonoMode  = nwal_FALSE;

  for (i = 0; i < NSS_NUM_TX_PKTDMA_CHANNELS; i++)
  {
    txChCfg.channelNum = i;
    pIhandle->txChHnd[i]  =
        Cppi_txChannelOpen (pIhandle->cppiHandle, &txChCfg, &isAlloc);

    if (pIhandle->txChHnd[i] == NULL)  {
        continue;
    }
    else
    {
        Cppi_channelEnable (pIhandle->txChHnd[i]);
    }
  }

  /*Disable CPPI CPDMA loopback for PASS*/
  if(Cppi_setCpdmaLoopback(pIhandle->cppiHandle, 0) != CPPI_SOK)
  {
        return (nwal_ERR_CPPI);;
  }
  /* Need to store cppiHandle as as offset */

  return (nwal_OK);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Setup the global PA TX queues
 *****************************************************************************
 * DESCRIPTION: The function will setup all the queues to be used by NWAL
 *****************************************************************************/
nwal_RetValue nwal_SetupPATxQueues (nwalGlobContext_t*   pIhandle)
{
  int       i;
  uint8_t   isAlloc;

    /* Open all PA transmit queues (corresponding to TX CDMA channels */
    for (i = 0; i < NSS_NUM_TX_QUEUES; i++)
    {
        /* Explicitly indicating the queues in case if it is already opened
         * by another application outside NWAL
        */
        
        pIhandle->txQ[i] =
           Qmss_queueOpen (Qmss_QueueType_PASS_QUEUE,
                            QMSS_PASS_QUEUE_BASE + i,
                            &isAlloc);
        if (pIhandle->txQ[i] <= 0)
        {
            return (nwal_ERR_QMSS);
        }

        Qmss_setQueueThreshold (pIhandle->txQ[i], 1, 1);
    }

    return (nwal_OK);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Create next available general purpose Queue
 *****************************************************************************
 * DESCRIPTION: The function will create next available general purpose Queue
 *****************************************************************************/
nwal_RetValue nwal_CreateGenPurposeQueue (Qmss_QueueHnd*    pQHnd)
{

    uint8_t   isAlloc;

    /* Open next available queue from the pool */
    *pQHnd =
        Qmss_queueOpen (Qmss_QueueType_GENERAL_PURPOSE_QUEUE,
                        QMSS_PARAM_NOT_SPECIFIED,
                        &isAlloc);
    if(*pQHnd < 0)
    {
        return(nwal_ERR_QMSS);
    }
  return (nwal_OK);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Configure the Flow
 *****************************************************************************
 * DESCRIPTION: The function will setup the flow for the incoming packets at PA
 *****************************************************************************/
nwal_RetValue nwal_SetupFlow(   nwalGlobContext_t*  pIhandle,
                                nwalMbufPool_t      *pFlowCfg,
                                uint32_t            rxSopPktOffset,
                                uint32_t            rxPktTailRoomSz,
                                Cppi_FlowHnd        *pFlowHnd,
                                nwal_Bool_t         errCtl)
{
  Cppi_RxFlowCfg  rxFlowCfg;
  Uint8           isAlloc;
  Qmss_Queue      rxBufQ[NWAL_MAX_BUF_POOLS];
  Uint32          rxBufSize[NWAL_MAX_BUF_POOLS];
  int             i;
  Qmss_QueueHnd   qmssHnd;
  Qmss_Queue      tmpQ;
  nwalLocContext_t*   pLocContext;
  pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);

  memset(&rxFlowCfg,0,sizeof(Cppi_RxFlowCfg));
  for (i = 0; i < NWAL_MAX_BUF_POOLS; i++)
  {
      if (i >= pFlowCfg->numBufPools)
      {
          rxBufQ[i].qMgr = 0;
          rxBufQ[i].qNum = 0;
          rxBufSize[i] = 0;
      } else
      {
          qmssHnd = Pktlib_getInternalHeapQueue(pFlowCfg->bufPool[i].heapHandle);
          tmpQ = Qmss_getQueueNumber(qmssHnd);
          rxBufQ[i].qMgr = tmpQ.qMgr;
          rxBufQ[i].qNum =  tmpQ.qNum;
          rxBufSize[i]=
              (pFlowCfg->bufPool[i].bufSize - rxSopPktOffset - rxPktTailRoomSz);
      }
      if (i && (rxBufQ[i].qNum <= 0))
      {
          rxBufQ[i] = rxBufQ[i-1];
          rxBufSize[i] = 0;
      }
  }

  /* Configure Rx flow */
  rxFlowCfg.flowIdNum      = CPPI_PARAM_NOT_SPECIFIED;
  tmpQ = Qmss_getQueueNumber(pIhandle->defFlowQ);
  rxFlowCfg.rx_dest_qnum   = tmpQ.qNum;  /* Override in PA */
  rxFlowCfg.rx_dest_qmgr   = tmpQ.qMgr;
  rxFlowCfg.rx_sop_offset  = rxSopPktOffset;
  rxFlowCfg.rx_ps_location = Cppi_PSLoc_PS_IN_DESC;
  rxFlowCfg.rx_desc_type   = Cppi_DescType_HOST;
  if(errCtl)
  {
      rxFlowCfg.rx_error_handling = 1;
  }
  {
      rxFlowCfg.rx_error_handling = 0;
  }

  rxFlowCfg.rx_psinfo_present = 1;
  rxFlowCfg.rx_einfo_present  = 1;

  rxFlowCfg.rx_dest_tag_lo = 0;
  rxFlowCfg.rx_dest_tag_hi = 0;
  rxFlowCfg.rx_src_tag_lo  = 0;
  rxFlowCfg.rx_src_tag_hi  = 0;

  rxFlowCfg.rx_size_thresh0_en = rxBufSize[1] ? 1 : 0;
  rxFlowCfg.rx_size_thresh1_en = rxBufSize[2] ? 1 : 0;
  rxFlowCfg.rx_size_thresh2_en = rxBufSize[3] ? 1 : 0;

  rxFlowCfg.rx_dest_tag_lo_sel = 0;
  rxFlowCfg.rx_dest_tag_hi_sel = 0;
  rxFlowCfg.rx_src_tag_lo_sel  = 0;
  rxFlowCfg.rx_src_tag_hi_sel  = 0;

  rxFlowCfg.rx_fdq1_qnum = rxBufQ[1].qNum;
  rxFlowCfg.rx_fdq1_qmgr = rxBufQ[1].qMgr;
  
  rxFlowCfg.rx_fdq2_qnum = rxBufQ[2].qNum;
  rxFlowCfg.rx_fdq2_qmgr = rxBufQ[2].qMgr;
  
  rxFlowCfg.rx_fdq3_qnum = rxBufQ[3].qNum;
  rxFlowCfg.rx_fdq3_qmgr = rxBufQ[3].qMgr;

  rxFlowCfg.rx_size_thresh0 = rxBufSize[1] ? rxBufSize[0] : 0;
  rxFlowCfg.rx_size_thresh1 = rxBufSize[2] ? rxBufSize[1] : 0;
  rxFlowCfg.rx_size_thresh2 = rxBufSize[3] ? rxBufSize[2] : 0;

  rxFlowCfg.rx_fdq0_sz0_qnum = rxBufQ[0].qNum;
  rxFlowCfg.rx_fdq0_sz0_qmgr = rxBufQ[0].qMgr;
  
  rxFlowCfg.rx_fdq0_sz1_qnum = rxBufQ[1].qNum;
  rxFlowCfg.rx_fdq0_sz1_qmgr = rxBufQ[1].qMgr;

  rxFlowCfg.rx_fdq0_sz2_qnum = rxBufQ[2].qNum;
  rxFlowCfg.rx_fdq0_sz2_qmgr = rxBufQ[2].qMgr;

  rxFlowCfg.rx_fdq0_sz3_qnum = rxBufQ[3].qNum;
  rxFlowCfg.rx_fdq0_sz3_qmgr = rxBufQ[3].qMgr;

  if(pLocContext && pLocContext->cppiHandle)
  *pFlowHnd =
      Cppi_configureRxFlow(pLocContext->cppiHandle,
                           &rxFlowCfg,
                           &isAlloc);
  else
      *pFlowHnd =
          Cppi_configureRxFlow (pIhandle->cppiHandle,  &rxFlowCfg, &isAlloc);
    
  if (*pFlowHnd == NULL)
  {
      return (nwal_ERR_CPPI);
  }

  return (nwal_OK);

}
/***********************************************************************
 * FUNCTION PURPOSE: API to configure NWAL with its shared memory base
                     address, pointer and size of memory area to be 
                     used for its local context. Called once per process
 ***********************************************************************
 * DESCRIPTION: API to configure NWAL with its shared memory base
                     address, pointer and size of memory area to be 
                     used for its local context. Called once per process.
 ***********************************************************************/

nwal_RetValue nwal_createProc(void*     pSharedBase,
                              void*     pLocalCtxInstPool,
                              nwalBaseAddrCfg_t* pBaseAddrCfg)
{
    memset(&nwalProcessCtx, 0, sizeof(nwalProcessContext_t));
    /* pSharedBase will be process view base address of shared memory segment */
    nwalProcessCtx.pSharedMemBase = (nwalGlobContext_t*)pSharedBase;

    nwalProcessCtx.pLocalCtxInstPool = (nwalLocContext_t*) pLocalCtxInstPool;

    nwalProcessCtx.baseAddrCfg.pSaVirtBaseAddr = pBaseAddrCfg->pSaVirtBaseAddr;
    nwalProcessCtx.baseAddrCfg.pInstPoolSaBaseAddr = pBaseAddrCfg->pInstPoolSaBaseAddr;
    nwalProcessCtx.baseAddrCfg.pScPoolBaseAddr = pBaseAddrCfg->pScPoolBaseAddr;
    nwalProcessCtx.baseAddrCfg.paVirtBaseAddr = pBaseAddrCfg->paVirtBaseAddr;
    nwalProcessCtx.baseAddrCfg.pInstPoolPaBaseAddr = pBaseAddrCfg->pInstPoolPaBaseAddr;
    return (nwal_OK);
}

nwal_Inst nwal_getGlobalCtx()
{
    return nwalProcessCtx.pSharedMemBase;
}

/********************************************************************
 * FUNCTION PURPOSE: Create NWAL global instance
 ********************************************************************
 * DESCRIPTION: API instantiates the driver and is pre-requisite
 *              before calling all other APIs. This function
 *              initializes the NWAL driver based on user configuration
 ********************************************************************/
nwal_RetValue nwal_create ( const nwalGlobCfg_t*    pCfg,
                            nwalSizeInfo_t*         pSizeInfo,
                            int                     sizes[nwal_N_BUFS],
                            void*                   bases[nwal_N_BUFS],
                            nwal_Inst*              pNwalInst)

{
    nwalMacInfo_t*      pMacInfo;
    nwalIpInfo_t*       pIpInfo;
    nwalPortInfo_t*     pPortInfo;
    nwalL2L3HdrInfo_t*  pL2L3HdrInfo;
    int                 perInstSize;
    int                 count;
    int                 availSize;
    nwal_RetValue       nwalRetVal;
    nwalGlobContext_t*  pNwalContext;
    nwal_Bool_t         enableSA = nwal_FALSE;
    int                 tmpSizes[nwal_N_BUFS];
    int                 tmpAligns[nwal_N_BUFS];
    uint8_t*            pBufAddr;
    nwal_Bool_t         netCPCfgEnable=nwal_TRUE;

    enableSA =
        (pSizeInfo->nMaxIpSecChannels ||
         pSizeInfo->nMaxDmSecChannels) ? nwal_TRUE : nwal_FALSE;

    if((!enableSA) &&
       !(pSizeInfo->nMaxMacAddress) &&
       !(pSizeInfo->nMaxIpAddress) &&
       !(pSizeInfo->nMaxL4Ports))
    {
        netCPCfgEnable=nwal_FALSE;
    }
    nwal_getBufferReq(pSizeInfo,tmpSizes,tmpAligns);
    if(sizes[nwal_BUF_INDEX_INST]  < tmpSizes[nwal_BUF_INDEX_INST])
        return (nwal_ERR_MEM_ALLOC);

    pNwalContext = (nwalGlobContext_t*)(bases[nwal_BUF_INDEX_INST]);
    *pNwalInst = pNwalContext;
    nwalProcessCtx.pSharedMemBase = pNwalContext;
    memset(pNwalContext,0,sizeof(nwalGlobContext_t));
    memcpy(&pNwalContext->cfg,pCfg,sizeof(nwalGlobCfg_t));
    memcpy(&pNwalContext->memSizeInfo,pSizeInfo,sizeof(nwalSizeInfo_t));

    /* Store all handle buffers */
    if(sizes[nwal_BUF_INDEX_INT_HANDLES]  <
       tmpSizes[nwal_BUF_INDEX_INT_HANDLES])
        return (nwal_ERR_MEM_ALLOC);

    availSize = sizes[nwal_BUF_INDEX_INT_HANDLES];
    pBufAddr = bases[nwal_BUF_INDEX_INT_HANDLES];
    int i;
    for (i=0;i < nwal_N_BUFS;i++)
    memset(pBufAddr,0,availSize);

    /* NWAL MAC Handle */
    perInstSize =
        nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalMacInfo_t));
    if(availSize  < (perInstSize * pSizeInfo->nMaxMacAddress))
        return (nwal_ERR_MEM_ALLOC);

    pNwalContext->pMacInfo = (nwalMacInfo_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                              pBufAddr));

    pMacInfo = (nwalMacInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                            pNwalContext->pMacInfo));
    for(count=0;count < pSizeInfo->nMaxMacAddress;count++)
    {
        pMacInfo->handleHdr.handleId = NWAL_HANDLE_MAC_INST;
        pMacInfo->handleHdr.stateCount =
            NWAL_SET_STATE(pMacInfo->handleHdr.stateCount,
                           NWAL_STATE_INACTIVE);
        pMacInfo = (nwalMacInfo_t*)((uint8_t *)(pMacInfo) + perInstSize);
    }
    pBufAddr += (perInstSize * pSizeInfo->nMaxMacAddress);
    availSize = availSize - (perInstSize * pSizeInfo->nMaxMacAddress);

    if(enableSA)
    {
#ifdef NWAL_ENABLE_SA
        /* NWAL IPSec Handle */
        if(pSizeInfo->nMaxIpSecChannels)
        {
            perInstSize =
                nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                                1,
                                sizeof(nwalIpSecInfo_t));
            /*pNwalContext->saGlobContext.pIpSecInfo  =
                (nwalIpSecInfo_t *) pBufAddr; */
            pNwalContext->saGlobContext.pIpSecInfo = 
                (nwalIpSecInfo_t *)(NWAL_CONV_ADDRESS_TO_OFFSET( nwalProcessCtx.pSharedMemBase,
                                                      pBufAddr));
            pBufAddr += (perInstSize * pSizeInfo->nMaxIpSecChannels);
            availSize =
                availSize - (perInstSize * pSizeInfo->nMaxIpSecChannels);
        }

        /* NWAL DM Channel Handle */
        if(pSizeInfo->nMaxDmSecChannels)
        {
            perInstSize =
                nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                                1,
                                sizeof(nwalDmSaInfo_t));
            pNwalContext->saGlobContext.pDmSaInfo = 
            (nwalDmSaInfo_t *)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                      pBufAddr));
            pBufAddr += (perInstSize * pSizeInfo->nMaxDmSecChannels);
            availSize =
                availSize - (perInstSize * pSizeInfo->nMaxDmSecChannels);
        }
#else
        return (nwal_ERR_SA_NOT_ENABLED);
#endif
    }

    /* NWAL IP Handle */
    perInstSize =
        nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalIpInfo_t));
    if(availSize  < (perInstSize * pSizeInfo->nMaxIpAddress))
        return (nwal_ERR_MEM_ALLOC);

    pNwalContext->pIPInfo = 
            (nwalIpInfo_t *) NWAL_CONV_ADDRESS_TO_OFFSET((uint32_t)nwalProcessCtx.pSharedMemBase,
                                                         (uint32_t)pBufAddr);


    pIpInfo = 
        (nwalIpInfo_t *)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                    pNwalContext->pIPInfo));

    for(count=0;count < pSizeInfo->nMaxIpAddress;count++)
    {
        pIpInfo->handleHdr.handleId = NWAL_HANDLE_IP_INST;
        pIpInfo->handleHdr.stateCount =
            NWAL_SET_STATE(pIpInfo->handleHdr.stateCount,NWAL_STATE_INACTIVE);
        pIpInfo = (nwalIpInfo_t*)((uint8_t *)(pIpInfo) + perInstSize);
    }
    pBufAddr += (perInstSize * pSizeInfo->nMaxIpAddress);
    availSize = availSize - (perInstSize * pSizeInfo->nMaxIpAddress);

    /* NWAL Port Handle */
    perInstSize =
        nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalPortInfo_t));
    if(availSize  < (perInstSize * pSizeInfo->nMaxL4Ports))
        return (nwal_ERR_MEM_ALLOC);

    pNwalContext->pPortInfo = 
    (nwalPortInfo_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                  pBufAddr));


    pPortInfo = 
        (nwalPortInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                                      pNwalContext->pPortInfo));

    for(count=0;count < pSizeInfo->nMaxL4Ports;count++)
    {
        pPortInfo->handleHdr.handleId = NWAL_HANDLE_PORT_INST;
        pPortInfo->handleHdr.stateCount =
            NWAL_SET_STATE(pPortInfo->handleHdr.stateCount,
                           NWAL_STATE_INACTIVE);
        pPortInfo = (nwalPortInfo_t*)((uint8_t *)(pPortInfo) + perInstSize);
    }

    pBufAddr += (perInstSize * pSizeInfo->nMaxL4Ports);
    availSize = availSize - (perInstSize * pSizeInfo->nMaxL4Ports);

    /* L2L3 Header Buffer */
    perInstSize =
        nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalL2L3HdrInfo_t));
    if(availSize  < (perInstSize * pSizeInfo->nMaxL2L3Hdr))
        return (nwal_ERR_MEM_ALLOC);

    pNwalContext->pL2L3HdrInfo = 
    (nwalL2L3HdrInfo_t*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                                       pBufAddr));
     pL2L3HdrInfo = 
        (nwalL2L3HdrInfo_t*)(NWAL_CONV_OFFSET_TO_ADDRESS(nwalProcessCtx.pSharedMemBase,
                                             pNwalContext->pL2L3HdrInfo));

    for(count=0;count < pSizeInfo->nMaxL2L3Hdr;count++)
    {
        pL2L3HdrInfo->handleHdr.handleId = NWAL_HANDLE_L2_L3_HDR_INST;
        pL2L3HdrInfo->handleHdr.stateCount =
            NWAL_SET_STATE(pL2L3HdrInfo->handleHdr.stateCount,
                           NWAL_STATE_INACTIVE);
        pL2L3HdrInfo =
            (nwalL2L3HdrInfo_t*)((uint8_t *)(pL2L3HdrInfo) + perInstSize);
    }
    pBufAddr += (perInstSize * pSizeInfo->nMaxL2L3Hdr);
    availSize = availSize - (perInstSize * pSizeInfo->nMaxL2L3Hdr);

    perInstSize =
        nwal_round_size(tmpAligns[nwal_BUF_INDEX_INT_HANDLES],
                        1,
                        sizeof(nwalLocContext_t));

    
    if(availSize  < (perInstSize * pSizeInfo->nProc))
        return (nwal_ERR_MEM_ALLOC);


    if (nwalProcessCtx.pLocalCtxInstPool == 0)
    {
        pNwalContext->pLocContext = (nwalLocContext_t *)pBufAddr;
        nwalProcessCtx.pLocalCtxInstPool = pNwalContext->pLocContext;
        pBufAddr += (perInstSize * pSizeInfo->nProc);
        availSize = availSize - (perInstSize * pSizeInfo->nProc);
    }
    /* Initialize the NETCP Modules */
    nwalRetVal = nwal_PassPowerUp(pNwalContext,enableSA);
    if(nwalRetVal != nwal_OK)
    {
        return (nwalRetVal);
    }
    /* Initialize the CPDMA resources for NETCP */
    nwalRetVal = nwal_SetupCpdma(pNwalContext);
    if(nwalRetVal != nwal_OK)
    {
        return (nwalRetVal);
    }

    /* Setup the PA transmit queues */
    nwalRetVal = nwal_SetupPATxQueues(pNwalContext);
    if(nwalRetVal != nwal_OK)
    {
        return (nwalRetVal);
    }

    pNwalContext->rxPaSaFlowId = NWAL_FLOW_NOT_SPECIFIED;
    pNwalContext->rxSaPaFlowId = NWAL_FLOW_NOT_SPECIFIED;
    if(netCPCfgEnable)
    {
        /* Create General Purpose Queuues to be used global per system level */
        /* Default RX Packet queue for all flows. Should never get here */
        if(pNwalContext->cfg.rxDefPktQ == QMSS_PARAM_NOT_SPECIFIED)
        {
            nwalRetVal =
                nwal_CreateGenPurposeQueue(&pNwalContext->cfg.rxDefPktQ);
            if(nwalRetVal != nwal_OK)
            {
                return (nwalRetVal);
            }
        }

        if(pCfg->paHandle)
        {
            /* PA setup already done, only need to start PA for this core */
            nwal_StartPa(pNwalContext, pCfg->paHandle);
        }
        else
        {
            /* Setup the PA if not already initialized */
            if(!pNwalContext->memSizeInfo.pahandle)
            {
                /* Initialized the EMAC switch */
                nwalRetVal =  nwal_SetupPa(pNwalContext,
                                          sizes,
                                          bases);
                if(nwalRetVal != nwal_OK)
                {
                   return (nwalRetVal);
                }
            }
        }
        /* Default  queue for flows for catch all packets. One per system.*/
        nwalRetVal = nwal_CreateGenPurposeQueue(&pNwalContext->defFlowQ);
        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }

#ifdef NWAL_ENABLE_SA
        if(enableSA)
        {
            /* Create flow for RX packets from PA to SA */
            nwalRetVal = nwal_SetupFlow(pNwalContext,
                                        &pNwalContext->cfg.pa2SaBufPool,
                                        NWAL_DEFAULT_FLOW_SOP_OFFSET,
                                        NWAL_DEFAULT_TAILROOM_SIZE,
                                        &pNwalContext->rxPaSaFlow,
                                        nwal_FALSE);
            if(nwalRetVal != nwal_OK)
            {
                return (nwalRetVal);
            }
            pNwalContext->rxPaSaFlowId =
                (int16_t)Cppi_getFlowId(pNwalContext->rxPaSaFlow);
            /* Create flow for RX packets from SA to PA */
            nwalRetVal = nwal_SetupFlow(pNwalContext,
                                        &pNwalContext->cfg.sa2PaBufPool,
                                        NWAL_DEFAULT_FLOW_SOP_OFFSET,
                                        NWAL_DEFAULT_TAILROOM_SIZE,
                                        &pNwalContext->rxSaPaFlow,
                                        nwal_FALSE);
            if(nwalRetVal != nwal_OK)
            {
                return nwalRetVal;
            }
            pNwalContext->rxSaPaFlowId =
                (int16_t)Cppi_getFlowId(pNwalContext->rxSaPaFlow);

            nwalRetVal = nwal_GlobSaCreate(pNwalContext,
                                           pCfg,
                                           pSizeInfo,
                                           sizes,
                                           bases,
                                           tmpSizes,
                                           tmpAligns);
            if(nwalRetVal != nwal_OK)
                return nwalRetVal;
        }
#endif
    }
    /* Update the procID to indicate the master Processor */
    pNwalContext->masterProcId = NWAL_osalGetProcId();

    pNwalContext->state = NWAL_INSTANCE_ACTIVE;

    /* Write back the cache contents */
    for(count=0;count < nwal_N_BUFS;count++)
    {
        NWAL_osalWriteBackCache(bases[count],sizes[count]);
    }
    *pNwalInst = (nwal_Inst*)(NWAL_CONV_ADDRESS_TO_OFFSET(nwalProcessCtx.pSharedMemBase,
                                             pNwalContext));
    NWAL_osalWriteBackCache(&nwalProcessCtx,sizeof(nwalProcessContext_t));
    NWAL_osalWriteBackCache(pNwalContext, sizeof(nwalGlobContext_t));
    return (nwal_OK);
}

/********************************************************************
 * FUNCTION PURPOSE:    API to query global per system NWAL context info
 ********************************************************************
 * DESCRIPTION:         API to query global per system NWAL context info
 ********************************************************************/
nwal_RetValue nwal_getGlobCxtInfo(nwal_Inst             nwalInst,
                                  nwalGlobCxtInfo_t*    pInfo)
{
    uint8_t     count=0;
    if(nwalProcessCtx.pSharedMemBase->state != NWAL_INSTANCE_ACTIVE)
    {
        return(nwal_ERR_INVALID_STATE);
    }

    pInfo->rxPaSaFlowId     =
        (int16_t)nwalProcessCtx.pSharedMemBase->rxPaSaFlowId;
    pInfo->rxSaPaFlowId     =
        (int16_t)nwalProcessCtx.pSharedMemBase->rxSaPaFlowId;
    pInfo->rxDefPktQ        = nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ;
    pInfo->defFlowQ         = nwalProcessCtx.pSharedMemBase->defFlowQ;
    pInfo->extErr           = nwalProcessCtx.pSharedMemBase->extErr;
    pInfo->passCppiHandle   = nwalProcessCtx.pSharedMemBase->cppiHandle;

    /* Get NetCP PASS Firmware version which is active */
    pInfo->numPaPDSPs = NSS_PA_NUM_PDSPS;
    for(count=0;count < NSS_PA_NUM_PDSPS;count++)
    {
        Pa_getPDSPVersion(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                          count,
                          &pInfo->pdspVer[count]);
    }
    return (nwal_OK);
}
/********************************************************************
 * FUNCTION PURPOSE:    API to query per Local Process resources created
 *                      by NWAL
 ********************************************************************
 * DESCRIPTION:         API to query per Local Process resources created
 *                      by NWAL
 ********************************************************************/
nwal_RetValue nwal_PassGlobConfig(  nwalGlobContext_t*     pIhandle,
                                    nwalLocContext_t*      pLocContext)
{
    paCtrlInfo_t            ctrlInfo;
    Cppi_HostDesc*          pHd;
    paReturn_t              netCPRet;
    paCmdReply_t            cmdReply;
    uint16_t                cmdSize = pa_CONFIG_CMD_SET_MIN_CMD_BUF_SIZE_BYTES;
    int                     cmdDest;
    nwal_RetValue           retVal;
    paPacketControlConfig_t pktControlCfg;

    /* All global configuration from master processor to NetCP
     * will be done as blocking mode
     */
    pLocContext->transInfo.transId = NWAL_TRANSID_SPIN_WAIT;

    retVal = nwal_prepCmdBuf(pIhandle,
                             &cmdSize,
                             &cmdReply,
                             &pLocContext->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }

    /* Issue the command set */
    ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
    memset(&ctrlInfo.params.sysCfg,0,sizeof(paSysConfig_t));
    memset(&pktControlCfg,0,sizeof(paPacketControlConfig_t));
    pktControlCfg.ctrlBitMap = pa_PKT_CTRL_HDR_VERIFY_IP;
    ctrlInfo.params.sysCfg.pPktControl = &pktControlCfg;

    
    netCPRet = Pa_control(pIhandle->memSizeInfo.pahandle,
                          &ctrlInfo,
                          (paCmd_t) pHd->buffPtr,
                          &cmdSize,
                          &cmdReply,
                          &cmdDest);
    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        return nwal_ERR_PA;
    }

    retVal =  nwal_txCmdBuf(pIhandle,
                            cmdSize,
                            cmdDest,
                            pHd);
    if(retVal != nwal_OK)
    {
        return (retVal);
    }

    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    nwal_setTransInfo(&pLocContext->transInfo,
                      NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                      0);

    /* Block for response */
    retVal = nwal_pollCtlQ(pIhandle,
                           NULL,
                           NULL,
                           &pLocContext->transInfo.handleHdr);



    return (nwal_OK);
}



/********************************************************************
 * FUNCTION PURPOSE:    Start NWAL in a proc node
 ********************************************************************
 * DESCRIPTION:         Start NWAL in a proc node
 ********************************************************************/
nwal_RetValue nwal_start (  nwal_Inst                   nwalInst,
                            const nwalLocCfg_t*         pCfg)
{
    nwal_RetValue       nwalRetVal;
    nwalLocContext_t*   pLocContext;
    nwal_Bool_t         netCPCfgEnable=nwal_TRUE;
    Cppi_CpDmaInitCfg cpdmaCfg;


    NWAL_osalInvalidateCache(&nwalProcessCtx,sizeof(nwalProcessContext_t));
    NWAL_osalInvalidateCache(nwalProcessCtx.pSharedMemBase, sizeof(nwalGlobContext_t));
    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    NWAL_osalInvalidateCache(nwalProcessCtx.pSharedMemBase,sizeof(nwalGlobContext_t));
    if(nwalProcessCtx.pSharedMemBase->state != NWAL_INSTANCE_ACTIVE)
    {
        return (nwal_ERR_INVALID_STATE);
    }

    if(!(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxMacAddress) &&
       !(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpAddress) &&
       !(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxL4Ports) &&
       !(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpSecChannels) &&
       !(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxDmSecChannels))
    {
        netCPCfgEnable =  nwal_FALSE;
    }


    if(pLocContext->state == NWAL_LOC_INSTANCE_ACTIVE)
    {
        /* Local instance already active */
        return (nwal_OK);
    }

    memset(pLocContext,0,sizeof(nwalLocContext_t));

    /* Store the local configuration */
    if(pCfg)
        memcpy(&pLocContext->cfg,pCfg,sizeof(nwalLocCfg_t));

    if(pLocContext->cppiHandle == NULL)
    {
        memset(&cpdmaCfg,0,sizeof(Cppi_CpDmaInitCfg));
        cpdmaCfg.dmaNum   = Cppi_CpDma_PASS_CPDMA;
        pLocContext->cppiHandle = Cppi_open (&cpdmaCfg);
        
        if (pLocContext->cppiHandle == NULL)
        {
            return (nwal_ERR_CPPI);
        }
        if(nwalProcessCtx.pSharedMemBase->cppiHandle == NULL)
        {
            nwalProcessCtx.pSharedMemBase->cppiHandle = pLocContext->cppiHandle;
        }
    }

    if(netCPCfgEnable)
    {
        /* Default  queue for receiving Control command response from PA.*/
        nwalRetVal = nwal_CreateGenPurposeQueue(&pLocContext->rxCtlQ);
        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }

        /* Default Packet queue for receiving L4 packets.*/
        nwalRetVal = nwal_CreateGenPurposeQueue(&pLocContext->rxL4PktQ);
        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }

        /* Create flow for receiving control response from PA */
        nwalRetVal = nwal_SetupFlow(nwalProcessCtx.pSharedMemBase,
                                    &pLocContext->cfg.rxCtlPool,
                                    NWAL_DEFAULT_FLOW_SOP_OFFSET,
                                    0,
                                    &pLocContext->rxCtlFlow,
                                    nwal_TRUE);
        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }
        pLocContext->rxCtlFlowId =
            (int16_t)Cppi_getFlowId(pLocContext->rxCtlFlow);

        /* Create flow for receiving L4 packets from PA
         * For packets allow packets to be dropped in case of flood
         * by setting error control to be disabled in the flow configuration
         */
        nwalRetVal = nwal_SetupFlow(nwalProcessCtx.pSharedMemBase,
                                    &pLocContext->cfg.rxPktPool,
                                    pLocContext->cfg.rxSopPktOffset,
                                    pLocContext->cfg.rxPktTailRoomSz,
                                    &pLocContext->rxPktFlow,
                                    nwal_FALSE);
        if(nwalRetVal != nwal_OK)
        {
            return (nwalRetVal);
        }
        pLocContext->rxPktFlowId =
            (int16_t)Cppi_getFlowId(pLocContext->rxPktFlow);

        if((nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxIpSecChannels) ||
            (nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxDmSecChannels))
        {
#ifdef NWAL_ENABLE_SA
            if(nwalProcessCtx.pSharedMemBase->memSizeInfo.nMaxDmSecChannels)
            {
                /* Setup additional global level default queues for receiving
                 * Data mode Encrypted/Decrypted packets from SA
                 */
                /* Default  queue for receving side band crypto response from SA.*/
                nwalRetVal = nwal_CreateGenPurposeQueue(&pLocContext->rxSbSaQ);
                if(nwalRetVal != nwal_OK)
                {
                    return (nwalRetVal);
                }
            }
            nwalRetVal = nwal_SaStart(nwalProcessCtx.pSharedMemBase);
            if(nwalRetVal != nwal_OK)
            {
                return (nwalRetVal);
            }
#else
            return (nwal_ERR_SA_NOT_ENABLED);
#endif
        }

#ifdef NWAL_ENABLE_RX_NETCP_IP_INTENSIVE_CHECK
        if(nwalProcessCtx.pSharedMemBase->masterProcId == NWAL_osalGetProcId())
        {
            /* The core is master core which initiated nwal_create() API
             * Trigger NetCP PA global configuration
             */
            nwal_StartPa(nwalProcessCtx.pSharedMemBase, 0);
            nwalRetVal = nwal_PassGlobConfig(nwalProcessCtx.pSharedMemBase,pLocContext);
            if(nwalRetVal != nwal_OK)
            {
                return (nwalRetVal);
            }
        }
#endif
    }
    pLocContext->state = NWAL_LOC_INSTANCE_ACTIVE;
    NWAL_osalWriteBackCache(pLocContext,sizeof(nwalLocContext_t));
    return (nwal_OK);
}


/********************************************************************
 * FUNCTION PURPOSE:    API to query per Local Process resources created
 *                      by NWAL
 ********************************************************************
 * DESCRIPTION:         API to query per Local Process resources created
 *                      by NWAL
 ********************************************************************/
nwal_RetValue nwal_getLocCxtInfo(nwal_Inst              nwalInst,
                                 nwalLocCxtInfo_t*      pInfo)
{

    nwalLocContext_t*   pLocContext;

    if(nwalProcessCtx.pSharedMemBase->state != NWAL_INSTANCE_ACTIVE)
    {
        return(nwal_ERR_INVALID_STATE);
    }

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }

    if(pLocContext->state != NWAL_LOC_INSTANCE_ACTIVE)
    {
        return(nwal_ERR_INVALID_STATE);
    }

    pInfo->numPendPAReq = pLocContext->numPendPAReq;
    pInfo->rxCtlQ       = pLocContext->rxCtlQ;
    pInfo->rxL4PktQ     = pLocContext->rxL4PktQ;
    pInfo->rxSbSaQ       = pLocContext->rxSbSaQ;
    pInfo->extErr       = pLocContext->extErr;
    pInfo->rxPktFlowId    = (int16_t)Cppi_getFlowId(pLocContext->rxPktFlow);
    pInfo->rxCtlFlowId    = (int16_t)Cppi_getFlowId(pLocContext->rxCtlFlow);
    pInfo->cppiHandle     = pLocContext->cppiHandle;
    return(nwal_OK);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Delete the queues
 *****************************************************************************
 * DESCRIPTION: The function will pop all descriptors and delet the queue
 *****************************************************************************/
nwal_Bool_t nwal_DeleteQ (Qmss_QueueHnd queue)
{
    Cppi_HostDesc*  pHd=NULL;

    /* Pops off descriptors from TX linked buffer Q */
    pHd = (Cppi_HostDesc *)(QMSS_DESC_PTR(Qmss_queuePop (queue)));
    while(pHd != NULL)
    {
        pHd = (Cppi_HostDesc *)(QMSS_DESC_PTR(Qmss_queuePop (queue)));

    }

    if ((Qmss_queueClose (queue)) != CPPI_SOK)
    {
        return (nwal_FALSE);
    }

    return (nwal_TRUE);
}

/*****************************************************************************
 * FUNCTION PURPOSE: Delete the CPDMA channel
 *****************************************************************************
 * DESCRIPTION: The function deletes all CPPI CPDMA resources related resources
 *****************************************************************************/
nwal_Bool_t nwal_deleteCpdma (nwalGlobContext_t*   pIhandle)
{
  uint8_t         i;

  for (i = 0; i < NSS_NUM_TX_PKTDMA_CHANNELS; i++)
  {
      if(!pIhandle->txChHnd[i])
          continue;
      if ((Cppi_channelDisable (pIhandle->txChHnd[i])) != CPPI_SOK)
      {
          return nwal_FALSE;
      }

      if ((Cppi_channelClose (pIhandle->txChHnd[i])) != CPPI_SOK)
      {
          return nwal_FALSE;
      }
  }

  for (i = 0; i < NSS_NUM_RX_PKTDMA_CHANNELS; i++)
  {
      if (!pIhandle->rxChHnd[i])
          continue;
      if ((Cppi_channelDisable (pIhandle->rxChHnd[i])) != CPPI_SOK)
      {
          return nwal_FALSE;
      }

      if ((Cppi_channelClose (pIhandle->rxChHnd[i])) != CPPI_SOK)
      {
          return nwal_FALSE;
      }
  }
  return nwal_TRUE;
}

/*****************************************************************************
 * FUNCTION PURPOSE: Deinitialize all NetCP related resources created by NWAL
 *****************************************************************************
 * DESCRIPTION: The function will de-initialize the NetCP resources
 *****************************************************************************/
nwal_RetValue nwal_delete(nwal_Inst     nwalInst)
{
    uint8_t             count;
    nwalLocContext_t*   pLocContext;
    void*               bases[pa_N_BUFS];
#ifdef NWAL_ENABLE_SA
    void*               saBases[6];
#endif


    NWAL_osalInvalidateCache(nwalProcessCtx.pSharedMemBase,sizeof(nwalGlobContext_t));

    if(Pa_close(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,bases) != pa_OK)
    {
        return(nwal_ERR_PA);
    }
#ifdef NWAL_ENABLE_SA
    /* Delete SA Handle */
    if(Sa_close(nwalProcessCtx.pSharedMemBase->saGlobContext.salld_handle,saBases) != sa_ERR_OK)
    {
        return(nwal_ERR_SA);
    }
#endif

    //pLocContext = pNwalContext->pLocContext;
    pLocContext = nwalProcessCtx.pLocalCtxInstPool;
    for(count=0;count < nwalProcessCtx.pSharedMemBase->memSizeInfo.nProc;count++)
    {
        if(pLocContext->state == NWAL_LOC_INSTANCE_ACTIVE)
        {
            /* Close the flows created by NWAL */
            if ((Cppi_closeRxFlow (pLocContext->rxCtlFlow)) != CPPI_SOK)
            {
                return(nwal_ERR_CPPI);
            }
            if ((Cppi_closeRxFlow (pLocContext->rxPktFlow)) != CPPI_SOK)
            {
                return(nwal_ERR_CPPI);
            }

            /* Delete the queues */
            if(!(nwal_DeleteQ(pLocContext->rxCtlQ)))
            {
                return (nwal_ERR_QMSS);
            }

            if(!(nwal_DeleteQ(pLocContext->rxL4PktQ)))
            {
                return (nwal_ERR_QMSS);
            }

        }
        pLocContext = nwal_getNextInst(pLocContext,sizeof(nwalLocContext_t));
    }

    /* Cleanup global NWAL resources */
    if ((Cppi_closeRxFlow (nwalProcessCtx.pSharedMemBase->rxPaSaFlow)) != CPPI_SOK)
    {
        return(nwal_ERR_CPPI);
    }
    if ((Cppi_closeRxFlow (nwalProcessCtx.pSharedMemBase->rxSaPaFlow)) != CPPI_SOK)
    {
        return(nwal_ERR_CPPI);
    }

    /* Delete the queues */
    if(!(nwal_DeleteQ(nwalProcessCtx.pSharedMemBase->defFlowQ)))
    {
        return (nwal_ERR_QMSS);
    }
    if(!(nwal_DeleteQ(nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ)))
    {
        return (nwal_ERR_QMSS);
    }

    nwalProcessCtx.pSharedMemBase->state = 0;
    NWAL_osalWriteBackCache(nwalProcessCtx.pSharedMemBase,sizeof(nwalGlobContext_t));

    /* Free all per core related resources */
    return nwal_OK;
}
/**********************************************************************
 * FUNCTION PURPOSE:    API for run time NetCP EMAC Port Configuration
 **********************************************************************
 * DESCRIPTION:         API for run time NetCP EMAC Port Configuration
 **********************************************************************/
nwal_RetValue nwal_emacPortCfg(nwal_Inst                  nwalInst,
                               paEmacPortConfig_t*        pEmacPortCfg)
{
    nwalLocContext_t*       pLocContext = NULL;
    Cppi_HostDesc*          pHd;
    paReturn_t              netCPRet;
    paCmdReply_t            cmdReply;
    uint16_t                cmdSize = pa_EMAC_PORT_CONFIG_MIN_CMD_BUF_SIZE_BYTES;
    int                     cmdDest;
    nwal_RetValue           retVal;
    Ti_Pkt*                 pPkt;  
    paCtrlInfo_t            ctrlInfo;



    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }
    
    if(nwalProcessCtx.pSharedMemBase->masterProcId != NWAL_osalGetProcId())
    {
        /* Allowed only from Master Core */
        return(nwal_ERR_INVALID_PROC_ID);
    }

    /* All global configuration from master processor to NetCP
     * will be done as blocking mode
     */
    pLocContext->transInfo.transId = NWAL_TRANSID_SPIN_WAIT;

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             &pLocContext->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }
    memset(&ctrlInfo.params.emacPortCfg,0,sizeof(paEmacPortConfig_t));
    if(pEmacPortCfg->cfgType ==  pa_EMAC_PORT_CFG_EQoS_MODE)
    {
       
        paEQosModeConfig_t      paEQosModeCfg;
         ctrlInfo.code = pa_CONTROL_EMAC_PORT_CONFIG;
         ctrlInfo.params.emacPortCfg.cfgType = pa_EMAC_PORT_CFG_EQoS_MODE;
         ctrlInfo.params.emacPortCfg.numEntries = 1;

         memset(&paEQosModeCfg,0,sizeof(paEQosModeConfig_t));
         memcpy(&paEQosModeCfg, pEmacPortCfg->u.eQoSModeCfg, sizeof(paEQosModeConfig_t));
         ctrlInfo.params.emacPortCfg.u.eQoSModeCfg = &paEQosModeCfg;
    }
    if(pEmacPortCfg->cfgType ==  pa_EMAC_PORT_CFG_DEFAULT_ROUTE)
    {
        paDefRouteConfig_t       paDefRouteCfg;
        ctrlInfo.code = pa_CONTROL_EMAC_PORT_CONFIG;
        ctrlInfo.params.emacPortCfg.cfgType = pa_EMAC_PORT_CFG_DEFAULT_ROUTE;
        ctrlInfo.params.emacPortCfg.numEntries = 1;

        memset(&paDefRouteCfg,0,sizeof(paDefRouteConfig_t));
        memcpy(&paDefRouteCfg, pEmacPortCfg->u.defRouteCfg, sizeof(paDefRouteConfig_t));
        ctrlInfo.params.emacPortCfg.u.defRouteCfg = &paDefRouteCfg;
    }

    netCPRet = Pa_control(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                          &ctrlInfo,
                          (paCmd_t) pHd->buffPtr,
                           &cmdSize,
                           &cmdReply,
                           &cmdDest);
     if(netCPRet != pa_OK)
     {
         pLocContext->extErr = netCPRet;
         pPkt = Pktlib_getPacketFromDesc(pHd);
         Pktlib_freePacket(pPkt);
         return nwal_ERR_PA;
     }

     retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                             cmdSize,
                             cmdDest,
                             pHd);
     if(retVal != nwal_OK)
     {
         return (retVal);
     }

     /*update global info; need to protect from multicore*/
     pLocContext->numPendPAReq++;

     nwal_setTransInfo(&pLocContext->transInfo,
                       NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                       0);

    /* Block for response */
    retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                           NULL,
                           NULL,
                           &pLocContext->transInfo.handleHdr);
    return(retVal);

}
/********************************************************************
 * FUNCTION PURPOSE:    API for run time NetCP global
 *                      configuration
 ********************************************************************
 * DESCRIPTION:        API for run time NetCP global
 *                      configuration
 ********************************************************************/
nwal_RetValue nwal_control(nwal_Inst                  nwalInst,
                           nwalCtlInfo_t*             pCtlInfo)
{
    nwalLocContext_t*       pLocContext = NULL;
    Cppi_HostDesc*          pHd;
    paReturn_t              netCPRet;
    paCmdReply_t            cmdReply;
    uint16_t                cmdSize = pa_CONFIG_EXCEPTION_ROUTE_MIN_CMD_BUF_SIZE_BYTES;
    int                     cmdDest;
    nwal_RetValue           retVal;
    paRouteInfo2_t           eRoute[pa_EROUTE_MAX];
    int                     routeTypes[pa_EROUTE_MAX];
    uint8_t                 count;
    uint8_t                 maxExceptions = pa_EROUTE_MAX; 
    Ti_Pkt*                 pPkt;  
    nwal_Bool_t             enablePAAssistReassem;  

    pLocContext = nwal_getLocContext(nwalProcessCtx.pSharedMemBase);
    if(pLocContext == NULL)
    {
        return(nwal_ERR_INVALID_PROC_ID);
    }
    
    if(nwalProcessCtx.pSharedMemBase->masterProcId != NWAL_osalGetProcId())
    {
        /* Allowed only from Master Core */
        return(nwal_ERR_INVALID_PROC_ID);
    }

    /* All global configuration from master processor to NetCP
     * will be done as blocking mode
     */
    pLocContext->transInfo.transId = NWAL_TRANSID_SPIN_WAIT;

    retVal = nwal_prepCmdBuf(nwalProcessCtx.pSharedMemBase,
                             &cmdSize,
                             &cmdReply,
                             &pLocContext->transInfo,
                             &pHd);
    if(retVal != nwal_OK)
    {
        return retVal;
    }
    
    pLocContext->pRxReassemProc = pCtlInfo->pRxReassemProc;
    if(pCtlInfo->pktCtl  ==  NWAL_CTRL_CFG_PA_ASSISTED_REASSEM)
    {
        paCtrlInfo_t            ctrlInfo;
        paIpReassmConfig_t      paIpReassmCfg;
        
        ctrlInfo.code = pa_CONTROL_SYS_CONFIG;
        memset(&ctrlInfo.params.sysCfg,0,sizeof(paSysConfig_t));
        /* Enable PASS Assisted Reassembly functionality */
        memset(&paIpReassmCfg,0,sizeof(paIpReassmCfg));        
        if(pCtlInfo->matchAction ==  NWAL_MATCH_ACTION_HOST)
        {
            if(!(pLocContext->pRxReassemProc))
            {
                pPkt = Pktlib_getPacketFromDesc(pHd);
                Pktlib_freePacket(pPkt);
                return(nwal_ERR_INVALID_PARAM);
            }
                /* Callback not available */
            /* Always set to Maximum */
            paIpReassmCfg.numTrafficFlow = pa_MAX_IP_REASM_TRAFFIC_FLOWS;
            enablePAAssistReassem = nwal_TRUE;
        }
        else
        {
            /* Disabled the PA Assisted Reassembly */
            paIpReassmCfg.numTrafficFlow = 0;
            enablePAAssistReassem = nwal_FALSE;
        }
        
        if(pCtlInfo->appRxPktFlowId == NWAL_FLOW_NOT_SPECIFIED)
        {
            paIpReassmCfg.destFlowId = pLocContext->rxPktFlowId;
        }
        else
        {
            paIpReassmCfg.destFlowId = pCtlInfo->appRxPktFlowId;
        }

        if(pCtlInfo->appRxPktQueue == NWAL_QUEUE_NOT_SPECIFIED)
        {
            paIpReassmCfg.destQueue = nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ;
        }
        else
        {
            paIpReassmCfg.destQueue = pCtlInfo->appRxPktQueue;
        }
        ctrlInfo.params.sysCfg.pOutIpReassmConfig = &paIpReassmCfg;
        ctrlInfo.params.sysCfg.pInIpReassmConfig = &paIpReassmCfg;
        
        netCPRet = Pa_control(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                              &ctrlInfo,
                              (paCmd_t) pHd->buffPtr,
                              &cmdSize,
                              &cmdReply,
                              &cmdDest);
        if(netCPRet != pa_OK)
        {
            pLocContext->extErr = netCPRet;
            pPkt = Pktlib_getPacketFromDesc(pHd);
            Pktlib_freePacket(pPkt);
            return nwal_ERR_PA;
        }

        retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                                cmdSize,
                                cmdDest,
                                pHd);
        if(retVal != nwal_OK)
        {
            return (retVal);
        }

        /*update global info; need to protect from multicore*/
        pLocContext->numPendPAReq++;

        nwal_setTransInfo(&pLocContext->transInfo,
                          NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                          0);

        /* Block for response */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pLocContext->transInfo.handleHdr);
        
        if(retVal == nwal_OK)
        {
            pLocContext->enablePAAssistReassem = enablePAAssistReassem;
        }
        
        return (retVal);         
    }

    if(pCtlInfo->pktCtl == NWAL_CTRL_CFG_NATT)
    {
        paCtrlInfo_t            ctrlInfo;

        ctrlInfo.code = pa_CONTROL_IPSEC_NAT_T_CONFIG;
        memset(&ctrlInfo.params.ipsecNatTDetCfg,0,sizeof(paIpsecNatTConfig_t));

        ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap = (pCtlInfo->enableNatt) ?
                                                      pa_IPSEC_NAT_T_CTRL_ENABLE : 0;
#if defined(DEVICE_K2E) || defined(DEVICE_K2L) || defined(SOC_K2E) || defined(SOC_K2L)
        ctrlInfo.params.ipsecNatTDetCfg.ctrlBitMap |= pa_IPSEC_NAT_T_CTRL_LOC_LUT1;
#endif
        ctrlInfo.params.ipsecNatTDetCfg.udpPort = pCtlInfo->nattPort;

        netCPRet = Pa_control(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                              &ctrlInfo,
                              (paCmd_t) pHd->buffPtr,
                              &cmdSize,
                              &cmdReply,
                              &cmdDest);
        if(netCPRet != pa_OK)
        {
            pLocContext->extErr = netCPRet;
            pPkt = Pktlib_getPacketFromDesc(pHd);
            Pktlib_freePacket(pPkt);
            return nwal_ERR_PA;
        }

        retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                                cmdSize,
                                cmdDest,
                                pHd);
        if(retVal != nwal_OK)
        {
            return (retVal);
        }

        /*update global info; need to protect from multicore*/
        pLocContext->numPendPAReq++;

        nwal_setTransInfo(&pLocContext->transInfo,
                          NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                          0);

        /* Block for response */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pLocContext->transInfo.handleHdr);

        return (retVal);
    }

    if((pCtlInfo->pktCtl  ==  NWAL_CTRL_CFG_EMAC_IF_EGRESS_EQOS_MODE) ||
       (pCtlInfo->pktCtl  ==  NWAL_CTRL_CFG_EMAC_IF_INGRESS_DEFAULT_ROUTE))
    {
        paCtrlInfo_t            ctrlInfo;
        paPacketControl2Config_t pktControl2Cfg;
        memset(&ctrlInfo.params.sysCfg,0,sizeof(paSysConfig_t));
        memset(&pktControl2Cfg,0,sizeof(paPacketControl2Config_t));
        ctrlInfo.code = pa_CONTROL_SYS_CONFIG;


        if(pCtlInfo->pktCtl  ==  NWAL_CTRL_CFG_EMAC_IF_EGRESS_EQOS_MODE)
        {
            if(pCtlInfo->matchAction ==  NWAL_MATCH_ACTION_HOST)
            {
                pktControl2Cfg.ctrlBitMap = pa_PKT_CTRL_EMAC_IF_EGRESS_EQoS_MODE;
                pktControl2Cfg.egressDefPri = pCtlInfo->egressDefPri;
            }
            else
            {
                pktControl2Cfg.ctrlBitMap = 0;
            }

            pktControl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_EGRESS_EQoS_MODE;
            ctrlInfo.params.sysCfg.pPktControl2 = &pktControl2Cfg;
        }
        if(pCtlInfo->pktCtl  ==  NWAL_CTRL_CFG_EMAC_IF_INGRESS_DEFAULT_ROUTE)
        {
            if(pCtlInfo->matchAction ==  NWAL_MATCH_ACTION_HOST)
            {
                pktControl2Cfg.ctrlBitMap = pa_PKT_CTRL_EMAC_IF_INGRESS_DEFAULT_ROUTE;
                pktControl2Cfg.egressDefPri = pCtlInfo->egressDefPri;
            }
            else
            {
                pktControl2Cfg.ctrlBitMap = 0;
            }

            pktControl2Cfg.validBitMap = pa_PKT_CTRL2_VALID_EMAC_IF_INGRESS_DEFAULT_ROUTE;
            ctrlInfo.params.sysCfg.pPktControl2 = &pktControl2Cfg;
        }

        netCPRet = Pa_control(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                              &ctrlInfo,
                              (paCmd_t) pHd->buffPtr,
                              &cmdSize,
                              &cmdReply,
                              &cmdDest);
        if(netCPRet != pa_OK)
        {
            pLocContext->extErr = netCPRet;
            pPkt = Pktlib_getPacketFromDesc(pHd);
            Pktlib_freePacket(pPkt);
            return nwal_ERR_PA;
        }
        retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                                cmdSize,
                                cmdDest,
                                pHd);
        if(retVal != nwal_OK)
        {
            return (retVal);
        }
        /*update global info; need to protect from multicore*/
        pLocContext->numPendPAReq++;
        nwal_setTransInfo(&pLocContext->transInfo,
                          NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                          0);
         /* Block for response */
        retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                               NULL,
                               NULL,
                               &pLocContext->transInfo.handleHdr);
        return (retVal);
    }

    /* Get the Match and Route Info based on Application configuration
     */
    memset(eRoute,0,pa_EROUTE_MAX * sizeof(paRouteInfo2_t));
    if(pCtlInfo->matchAction !=  NWAL_MATCH_ACTION_HOST)
    {
        eRoute[0].dest = pa_DEST_DISCARD;
    }
    else
    {
        eRoute[0].dest = pa_DEST_HOST;
        eRoute[0].mRouteIndex = pa_NO_MULTI_ROUTE;
        eRoute[0].swInfo0 = (uint32_t)pCtlInfo->appId;

        if(pCtlInfo->appRxPktFlowId == NWAL_FLOW_NOT_SPECIFIED)
        {
            eRoute[0].flowId = pLocContext->rxPktFlowId;
        }
        else
        {
            eRoute[0].flowId = pCtlInfo->appRxPktFlowId;
        }

        if(pCtlInfo->appRxPktQueue == NWAL_QUEUE_NOT_SPECIFIED)
        {
            eRoute[0].queue = nwalProcessCtx.pSharedMemBase->cfg.rxDefPktQ;
        }
        else
        {
            eRoute[0].queue = pCtlInfo->appRxPktQueue;
        }
        
        if ((pCtlInfo->validParams & NWAL_CONTROL_VALID_PARAM_ROUTE_TYPE) ==
                (NWAL_CONTROL_VALID_PARAM_ROUTE_TYPE))
        {
            nwalUpdateRoutePriority(pCtlInfo->routeType, &eRoute[0]);
        }
    }  
    
    if((pCtlInfo->pktCtl & NWAL_CTRL_CFG_SINGLE_EXCEPTION) ==
        NWAL_CTRL_CFG_SINGLE_EXCEPTION)
    {
        maxExceptions = 1;
        routeTypes[0] = pCtlInfo->pa_EROUTE_Id;
    }
    else
    {
        routeTypes[0] = 0;
        for(count=0;count < maxExceptions;count++)
        {
            if((count == pa_EROUTE_IP_FRAG) &&
               (pLocContext->enablePAAssistReassem == nwal_TRUE))
            {
                /* Skip exception configuration for IP fragments */
                count++;
            }
            memcpy(&eRoute[count],&eRoute[0],sizeof(paRouteInfo2_t));
            /*Modify per Exception config */
            routeTypes[count] = count;
        }
    }

    netCPRet = Pa_configExceptionRoute2(nwalProcessCtx.pSharedMemBase->memSizeInfo.pahandle,
                                       maxExceptions,
                                       routeTypes,
                                       eRoute,
                                       (paCmd_t) pHd->buffPtr,
                                       &cmdSize,
                                       &cmdReply,
                                       &cmdDest);
    if(netCPRet != pa_OK)
    {
        pLocContext->extErr = netCPRet;
        return nwal_ERR_PA;
    }

    retVal =  nwal_txCmdBuf(nwalProcessCtx.pSharedMemBase,
                            cmdSize,
                            cmdDest,
                            pHd);
    if(retVal != nwal_OK)
    {
        return (retVal);
    }

    /*update global info; need to protect from multicore*/
    pLocContext->numPendPAReq++;

    nwal_setTransInfo(&pLocContext->transInfo,
                      NWAL_HANDLE_ID_TRANS_PA_GLOB_CFG,
                      0);

    /* Block for response */
    retVal = nwal_pollCtlQ(nwalProcessCtx.pSharedMemBase,
                           NULL,
                           NULL,
                           &pLocContext->transInfo.handleHdr);

    return (nwal_OK);
}
