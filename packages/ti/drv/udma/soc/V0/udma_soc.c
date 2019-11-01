/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 */

/**
 *  \file udma_soc.c
 *
 *  \brief File containing the UDMA driver SOC related configuration functions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/udma/src/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief External start channel of DRU0 UTC */
#define UDMA_UTC_START_CH_DRU0          (0U)
/** \brief Number of channels present in DRU0 UTC */
#define UDMA_UTC_NUM_CH_DRU0            (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_CNT)
/** \brief Start thread ID of DRU0 UTC */
#define UDMA_UTC_START_THREAD_ID_DRU0   (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILD_THREAD_OFFSET)
/** \brief DRU0 UTC baseaddress */
#define UDMA_UTC_BASE_DRU0              (CSL_COMPUTE_CLUSTER0_DRU_BASE)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Udma_initDrvHandle(Udma_DrvHandle drvHandle)
{
    uint32_t            instId;
    CSL_UdmapCfg       *pUdmapRegs;
    CSL_RingAccCfg     *pRaRegs;
    CSL_IntaggrCfg     *pIaRegs;
    Udma_UtcInstInfo   *utcInfo;
    CSL_ProxyCfg       *pProxyCfg;
    CSL_ProxyTargetParams *pProxyTargetRing;

    instId = drvHandle->initPrms.instId;

    /*
     * UDMA config init
     */
    /* Init the config structure - one time step */
    pUdmapRegs = &drvHandle->udmapRegs;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        pUdmapRegs->pGenCfgRegs     = ((CSL_udmap_gcfgRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE);
        pUdmapRegs->pRxFlowCfgRegs  = ((CSL_udmap_rxfcfgRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE);
        pUdmapRegs->pTxChanCfgRegs  = ((CSL_udmap_txccfgRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE);
        pUdmapRegs->pRxChanCfgRegs  = ((CSL_udmap_rxccfgRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE);
        pUdmapRegs->pTxChanRtRegs   = ((CSL_udmap_txcrtRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP_TCHANRT_BASE);
        pUdmapRegs->pRxChanRtRegs   = ((CSL_udmap_rxcrtRegs *) CSL_MCU_NAVSS0_UDMASS_UDMAP_RCHANRT_BASE);
        drvHandle->trigGemOffset    = CSL_NAVSS_GEM_MCU_UDMA_TRIGGER_OFFSET;
    }
    else
    {
        pUdmapRegs->pGenCfgRegs     = ((CSL_udmap_gcfgRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_BASE);
        pUdmapRegs->pRxFlowCfgRegs  = ((CSL_udmap_rxfcfgRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE);
        pUdmapRegs->pTxChanCfgRegs  = ((CSL_udmap_txccfgRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE);
        pUdmapRegs->pRxChanCfgRegs  = ((CSL_udmap_rxccfgRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE);
        pUdmapRegs->pTxChanRtRegs   = ((CSL_udmap_txcrtRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_TCHANRT_BASE);
        pUdmapRegs->pRxChanRtRegs   = ((CSL_udmap_rxcrtRegs *) CSL_NAVSS0_UDMASS_UDMAP0_CFG_RCHANRT_BASE);
        drvHandle->trigGemOffset    = CSL_NAVSS_GEM_MAIN_UDMA_TRIGGER_OFFSET;
    }
    drvHandle->clecRegs = NULL;
    /* Fill other SOC specific parameters by reading from UDMA config
     * registers */
    CSL_udmapGetCfg(pUdmapRegs);

    /*
     * RA config init
     */
    pRaRegs = &drvHandle->raRegs;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        pRaRegs->pGlbRegs   = (CSL_ringacc_gcfgRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_GCFG_BASE;
        pRaRegs->pCfgRegs   = (CSL_ringacc_cfgRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE;
        pRaRegs->pRtRegs    = (CSL_ringacc_rtRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE;
        pRaRegs->pMonRegs   = (CSL_ringacc_monitorRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE;
        pRaRegs->pFifoRegs  = (CSL_ringacc_fifosRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_FIFOS_BASE;
        pRaRegs->pIscRegs   = (CSL_ringacc_iscRegs *) CSL_MCU_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE;
        pRaRegs->maxRings   = CSL_NAVSS_MCU_RINGACC_RING_CNT;
    }
    else
    {
        pRaRegs->pGlbRegs   = (CSL_ringacc_gcfgRegs *) CSL_NAVSS0_UDMASS_RINGACC0_GCFG_BASE;
        pRaRegs->pCfgRegs   = (CSL_ringacc_cfgRegs *) CSL_NAVSS0_UDMASS_RINGACC0_CFG_BASE;
        pRaRegs->pRtRegs    = (CSL_ringacc_rtRegs *) CSL_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE;
        pRaRegs->pMonRegs   = (CSL_ringacc_monitorRegs *) CSL_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE;
        pRaRegs->pFifoRegs  = (CSL_ringacc_fifosRegs *) CSL_NAVSS0_UDMASS_RINGACC0_SRC_FIFOS_BASE;
        pRaRegs->pIscRegs   = (CSL_ringacc_iscRegs *) CSL_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE;
        pRaRegs->maxRings   = CSL_NAVSS_MAIN_RINGACC_RING_CNT;
    }
    pRaRegs->maxMonitors     = CSL_RINGACC_MAX_MONITORS;
    pRaRegs->bTraceSupported = (bool)true;

    /*
     * All interrupt related config should be based on core and not
     * based on NAVSS instance
     */
#if defined (BUILD_MCU1_0) || defined (BUILD_MCU1_1)
    /* IA config init */
    pIaRegs = &drvHandle->iaRegs;
    pIaRegs->pCfgRegs       = (CSL_intaggr_cfgRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_CFG_BASE;
    pIaRegs->pImapRegs      = (CSL_intaggr_imapRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE;
    pIaRegs->pIntrRegs      = (CSL_intaggr_intrRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_INTR_BASE;
    pIaRegs->pL2gRegs       = (CSL_intaggr_l2gRegs *) CSL_MCU_NAVSS0_PAR_UDMASS_UDMASS_INTA0_CFG_L2G_BASE;
    pIaRegs->pMcastRegs     = (CSL_intaggr_mcastRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_MCAST_BASE;
    pIaRegs->pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_GCNT_BASE;
    pIaRegs->pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) CSL_MCU_NAVSS0_UDMASS_INTA0_GCNTRTI_BASE;
    CSL_intaggrGetCfg(pIaRegs);

    drvHandle->iaGemOffset  = CSL_NAVSS_GEM_MCU_UDMA_INTA0_SEVI_OFFSET;
    drvHandle->devIdIa      = TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0;
#if defined (BUILD_MCU1_0)
    drvHandle->devIdCore    = TISCI_DEV_MCU_ARMSS0_CPU0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU1_0;
#else
    drvHandle->devIdCore    = TISCI_DEV_MCU_ARMSS0_CPU1;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU1_1;
#endif
#endif
#if defined (BUILD_MPU1_0)
    /* IA config init */
    pIaRegs = &drvHandle->iaRegs;
    pIaRegs->pCfgRegs       = (CSL_intaggr_cfgRegs *) CSL_NAVSS0_UDMASS_INTA0_CFG_BASE;
    pIaRegs->pImapRegs      = (CSL_intaggr_imapRegs *) CSL_NAVSS0_UDMASS_INTA0_IMAP_BASE;
    pIaRegs->pIntrRegs      = (CSL_intaggr_intrRegs *) CSL_NAVSS0_UDMASS_INTA0_CFG_INTR_BASE;
    pIaRegs->pL2gRegs       = (CSL_intaggr_l2gRegs *) CSL_NAVSS0_PAR_UDMASS_UDMASS_INTA0_CFG_L2G_BASE;
    pIaRegs->pMcastRegs     = (CSL_intaggr_mcastRegs *) CSL_NAVSS0_UDMASS_INTA0_CFG_MCAST_BASE;
    pIaRegs->pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) CSL_NAVSS0_UDMASS_INTA0_CFG_GCNTCFG_BASE;
    pIaRegs->pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) CSL_NAVSS0_UDMASS_INTA0_CFG_GCNTRTI_BASE;
    CSL_intaggrGetCfg(pIaRegs);

    drvHandle->iaGemOffset  = CSL_NAVSS_GEM_MAIN_UDMA_INTA0_SEVI_OFFSET;
    drvHandle->devIdIa      = TISCI_DEV_NAVSS0_UDMASS_INTA0;
    drvHandle->devIdCore    = TISCI_DEV_GIC0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MPU1_0;
#endif

    /*
     * UTC config init
     */
    /* Each UTC config */
    utcInfo = &drvHandle->utcInfo[UDMA_UTC_ID_MSMC_DRU0];
    utcInfo->utcId         = UDMA_UTC_ID_MSMC_DRU0;
    utcInfo->utcType       = UDMA_UTC_TYPE_DRU;
    utcInfo->startCh       = UDMA_UTC_START_CH_DRU0;
    utcInfo->numCh         = UDMA_UTC_NUM_CH_DRU0;
    utcInfo->startThreadId = UDMA_UTC_START_THREAD_ID_DRU0;
    utcInfo->txCredit      = 2U;  //TODO: Read from PSIL gasket
    utcInfo->druRegs       = ((CSL_DRU_t *) UDMA_UTC_BASE_DRU0);
    utcInfo->numQueue      = CSL_NAVSS_UTC_MSMC_DRU_QUEUE_CNT;

    /*
     * Proxy init
     */
    pProxyCfg           = &drvHandle->proxyCfg;
    pProxyTargetRing    = &drvHandle->proxyTargetRing;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        pProxyTargetRing->pTargetRegs   = (CSL_proxy_target0Regs *) CSL_MCU_NAVSS0_PROXY0_TARGET0_DATA_BASE;
        pProxyTargetRing->numChns       = CSL_NAVSS_MCU_PROXY_TARGET_RINGACC0_NUM_CHANNELS;
        pProxyTargetRing->chnSizeBytes  = CSL_NAVSS_MCU_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES;

        pProxyCfg->pGlbRegs             = (CSL_proxyRegs *) CSL_MCU_NAVSS0_PROXY_CFG_GCFG_BASE;
        pProxyCfg->pCfgRegs             = (CSL_proxy_cfgRegs *) CSL_MCU_NAVSS0_PROXY0_BUF_CFG_BASE;
        pProxyCfg->bufferSizeBytes      = CSL_NAVSS_MCU_PROXY_BUFFER_SIZE_BYTES;
        pProxyCfg->numTargets           = 1U;
        pProxyCfg->pProxyTargetParams   = pProxyTargetRing;

        drvHandle->proxyTargetNumRing   = CSL_NAVSS_MCU_PROXY_TARGET_NUM_RINGACC0;
    }
    else
    {
        pProxyTargetRing->pTargetRegs   = (CSL_proxy_target0Regs *) CSL_NAVSS0_PROXY_TARGET0_DATA_BASE;
        pProxyTargetRing->numChns       = CSL_NAVSS_MAIN_PROXY_TARGET_RINGACC0_NUM_CHANNELS;
        pProxyTargetRing->chnSizeBytes  = CSL_NAVSS_MAIN_PROXY_TARGET_RINGACC0_NUM_CHANNEL_SIZE_BYTES;

        pProxyCfg->pGlbRegs             = (CSL_proxyRegs *) CSL_NAVSS0_PROXY0_CFG_BUF_CFG_BASE;
        pProxyCfg->pCfgRegs             = (CSL_proxy_cfgRegs *) CSL_NAVSS0_PROXY0_BUF_CFG_BASE;
        pProxyCfg->bufferSizeBytes      = CSL_NAVSS_MAIN_PROXY_BUFFER_SIZE_BYTES;
        pProxyCfg->numTargets           = 1U;
        pProxyCfg->pProxyTargetParams   = pProxyTargetRing;

        drvHandle->proxyTargetNumRing   = CSL_NAVSS_MAIN_PROXY_TARGET_NUM_RINGACC0;
    }

    /* Init other variables */
    drvHandle->txChOffset   = 0U;
    drvHandle->extChOffset  = drvHandle->txChOffset + pUdmapRegs->txChanCnt;
    drvHandle->rxChOffset   =
        drvHandle->extChOffset + pUdmapRegs->txExtUtcChanCnt;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        drvHandle->udmapSrcThreadOffset = CSL_PSILCFG_NAVSS_MCU_UDMAP0_TSTRM_THREAD_OFFSET;
        drvHandle->udmapDestThreadOffset= CSL_PSILCFG_NAVSS_MCU_UDMAP0_RSTRM_THREAD_OFFSET;
        drvHandle->maxRings             = CSL_NAVSS_MCU_RINGACC_RING_CNT;
        drvHandle->maxProxy             = CSL_NAVSS_MCU_PROXY_NUM_PROXIES;
        drvHandle->maxRingMon           = CSL_NAVSS_MCU_RINGACC_NUM_MONITORS;
        drvHandle->devIdRing            = TISCI_DEV_MCU_NAVSS0_RINGACC0;
        drvHandle->devIdUdma            = TISCI_DEV_MCU_NAVSS0_UDMAP0;
        drvHandle->devIdPsil            = TISCI_DEV_MCU_NAVSS0;
    }
    else
    {
        drvHandle->udmapSrcThreadOffset = CSL_PSILCFG_NAVSS_MAIN_UDMAP0_TSTRM_THREAD_OFFSET;
        drvHandle->udmapDestThreadOffset= CSL_PSILCFG_NAVSS_MAIN_UDMAP0_RSTRM_THREAD_OFFSET;
        drvHandle->maxRings             = CSL_NAVSS_MAIN_RINGACC_RING_CNT;
        drvHandle->maxProxy             = CSL_NAVSS_MAIN_PROXY_NUM_PROXIES;
        drvHandle->maxRingMon           = CSL_NAVSS_MAIN_RINGACC_NUM_MONITORS;
        drvHandle->devIdRing            = TISCI_DEV_NAVSS0_RINGACC0;
        drvHandle->devIdUdma            = TISCI_DEV_NAVSS0_UDMAP0;
        drvHandle->devIdPsil            = TISCI_DEV_NAVSS0;
    }

    return;
}

void UdmaRmInitPrms_init(uint32_t instId, Udma_RmInitPrms *rmInitPrms)
{
    const Udma_RmInitPrms *rmInitDefaultCfg;

    if(NULL_PTR != rmInitPrms)
    {
        rmInitDefaultCfg = Udma_rmGetDefaultCfg(instId);
        (void) memcpy(rmInitPrms, rmInitDefaultCfg, sizeof (Udma_RmInitPrms));
    }

    return;
}

uint32_t Udma_getCoreId(void)
{
    uint32_t coreId;

#if defined (BUILD_MPU1_0)
    coreId = UDMA_CORE_ID_MPU1_0;
#endif
#if defined (BUILD_MCU1_0)
    coreId = UDMA_CORE_ID_MCU1_0;
#endif
#if defined (BUILD_MCU1_1)
    coreId = UDMA_CORE_ID_MCU1_1;
#endif

    return (coreId);
}

uint32_t Udma_isCacheCoherent(void)
{
    uint32_t isCacheCoherent;

#if defined (BUILD_MPU1_0)
    isCacheCoherent = TRUE;
#else
    isCacheCoherent = FALSE;
#endif

    return (isCacheCoherent);
}
