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
#define UDMA_UTC_START_CH_DRU0              (0U)
/** \brief Number of channels present in DRU0 UTC */
#define UDMA_UTC_NUM_CH_DRU0                (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_CNT)
/** \brief Start thread ID of DRU0 UTC */
#define UDMA_UTC_START_THREAD_ID_DRU0       (CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILD_THREAD_OFFSET)

/** \brief External start channel of VPAC TC0 UTC */
#define UDMA_UTC_START_CH_VPAC_TC0          (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC0_CC_PSILS_THREAD_OFFSET - CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_OFFSET)
/** \brief Number of channels present in VPAC TC0 UTC */
#define UDMA_UTC_NUM_CH_VPAC_TC0            (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC0_CC_PSILS_THREAD_CNT)
/** \brief Start thread ID of VPAC TC0 UTC */
#define UDMA_UTC_START_THREAD_ID_VPAC_TC0   (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC0_CC_PSILD_THREAD_OFFSET)

/** \brief External start channel of VPAC TC1 UTC */
#define UDMA_UTC_START_CH_VPAC_TC1          (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC1_CC_PSILS_THREAD_OFFSET - CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_OFFSET)
/** \brief Number of channels present in VPAC TC1 UTC */
#define UDMA_UTC_NUM_CH_VPAC_TC1            (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC1_CC_PSILS_THREAD_CNT)
/** \brief Start thread ID of VPAC TC1 UTC */
#define UDMA_UTC_START_THREAD_ID_VPAC_TC1   (CSL_PSILCFG_NAVSS_MAIN_VPAC_TC1_CC_PSILD_THREAD_OFFSET)

/** \brief External start channel of DMPAC TC0 UTC */
#define UDMA_UTC_START_CH_DMPAC_TC0         (CSL_PSILCFG_NAVSS_MAIN_DMPAC_TC0_CC_PSILS_THREAD_OFFSET - CSL_PSILCFG_NAVSS_MAIN_MSMC0_PSILS_THREAD_OFFSET)
/** \brief Number of channels present in DMPAC TC0 UTC */
#define UDMA_UTC_NUM_CH_DMPAC_TC0           (CSL_PSILCFG_NAVSS_MAIN_DMPAC_TC0_CC_PSILS_THREAD_CNT)
/** \brief Start thread ID of DMPAC TC0 UTC */
#define UDMA_UTC_START_THREAD_ID_DMPAC_TC0  (CSL_PSILCFG_NAVSS_MAIN_DMPAC_TC0_CC_PSILD_THREAD_OFFSET)

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
#if defined (LOKI_BUILD)
#define HOST_EMULATION (1U)
#endif

#if defined (HOST_EMULATION)
/* These variables are defined for supporting host emulation ( PC emulation ) and
will not be used for target*/
CSL_udmap_gcfgRegs       gHost_udmap_gcfgRegs;
CSL_udmap_rxfcfgRegs     gHost_udmap_rxfcfgRegs;
CSL_udmap_txccfgRegs     gHost_udmap_txccfgRegs;
CSL_udmap_rxccfgRegs     gHost_udmap_rxccfgRegs;
CSL_udmap_txcrtRegs      gHost_udmap_txcrtRegs;
CSL_udmap_rxcrtRegs      gHost_udmap_rxcrtRegs;
CSL_ringacc_gcfgRegs     gHost_ringacc_gcfgRegs;
CSL_ringacc_cfgRegs      gHost_ringacc_cfgRegs;
CSL_ringacc_rtRegs       gHost_ringacc_rtRegs;
CSL_ringacc_monitorRegs  gHost_ringacc_monitorRegs;
CSL_ringacc_fifosRegs    gHost_ringacc_fifosRegs;
CSL_ringacc_iscRegs      gHost_ringacc_iscRegs;
CSL_psilcfgRegs          gHost_psilcfgRegs;
CSL_intaggr_cfgRegs      gHost_intaggr_cfgRegs;
CSL_intaggr_imapRegs     gHost_intaggr_imapRegs;
CSL_intaggr_intrRegs     gHost_intaggr_intrRegs;
CSL_intaggr_l2gRegs      gHost_intaggr_l2gRegs;
CSL_intaggr_mcastRegs    gHost_intaggr_mcastRegs;
CSL_intaggr_gcntcfgRegs  gHost_intaggr_gcntcfgRegs;
CSL_intaggr_gcntrtiRegs  gHost_intaggr_gcntrtiRegs;
CSL_intr_router_cfgRegs  gHost_intr_router_cfgRegs;
CSL_CLEC_EVTRegs  gHost_clec_evtRegs;

CSL_DRU_t                gHost_DRU_t;
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE         (&gHost_udmap_gcfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE        (&gHost_udmap_rxfcfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE            (&gHost_udmap_txccfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE            (&gHost_udmap_rxccfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP_TCHANRT_BASE           (&gHost_udmap_txcrtRegs)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP_RCHANRT_BASE           (&gHost_udmap_rxcrtRegs)

#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_BASE                  (&gHost_udmap_gcfgRegs)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE            (&gHost_udmap_rxfcfgRegs)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE            (&gHost_udmap_txccfgRegs)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE            (&gHost_udmap_rxccfgRegs)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHANRT_BASE          (&gHost_udmap_txcrtRegs)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHANRT_BASE          (&gHost_udmap_rxcrtRegs)

#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_GCFG_BASE       (&gHost_ringacc_gcfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE            (&gHost_ringacc_cfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE         (&gHost_ringacc_rtRegs)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE        (&gHost_ringacc_monitorRegs)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_FIFOS_BASE          (&gHost_ringacc_fifosRegs)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE        (&gHost_ringacc_iscRegs)

#define UDMA_NAVSS0_UDMASS_RINGACC0_GCFG_BASE               (&gHost_ringacc_gcfgRegs)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_BASE                (&gHost_ringacc_cfgRegs)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE             (&gHost_ringacc_rtRegs)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE            (&gHost_ringacc_monitorRegs)
#define UDMA_NAVSS0_UDMASS_RINGACC0_SRC_FIFOS_BASE          (&gHost_ringacc_fifosRegs)
#define UDMA_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE            (&gHost_ringacc_iscRegs)

#define UDMA_MCU_NAVSS0_UDMASS_PSILSS_CFG0_PROXY_BASE       (&gHost_psilcfgRegs)
#define UDMA_NAVSS0_UDMASS_PSILCFG0_CFG_PROXY_BASE          (&gHost_psilcfgRegs)

#define UDMA_MCU_NAVSS0_UDMASS_INTA0_CFG_BASE               (&gHost_intaggr_cfgRegs )
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE              (&gHost_intaggr_imapRegs)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_INTR_BASE              (&gHost_intaggr_intrRegs)
#define UDMA_MCU_NAVSS0_PAR_UDMASS_UDMASS_INTA0_CFG_L2G_BASE    (&gHost_intaggr_l2gRegs)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_MCAST_BASE             (&gHost_intaggr_mcastRegs)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNT_BASE              (&gHost_intaggr_gcntcfgRegs)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNTRTI_BASE           (&gHost_intaggr_gcntrtiRegs)

#define UDMA_MCU_NAVSS0_INTR0_CFG_BASE                      (&gHost_intr_router_cfgRegs)

#define UDMA_NAVSS0_UDMASS_INTA0_CFG_BASE                   (&gHost_intaggr_cfgRegs )
#define UDMA_NAVSS0_UDMASS_INTA0_IMAP_BASE                  (&gHost_intaggr_imapRegs)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_INTR_BASE              (&gHost_intaggr_intrRegs)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_L2G_BASE               (&gHost_intaggr_l2gRegs)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_MCAST_BASE             (&gHost_intaggr_mcastRegs)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTCFG_BASE           (&gHost_intaggr_gcntcfgRegs)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTRTI_BASE           (&gHost_intaggr_gcntrtiRegs)

#define UDMA_NAVSS0_INTR0_INTR_ROUTER_CFG_BASE              (&gHost_intr_router_cfgRegs)

/** \brief DRU0 UTC baseaddress */
#define UDMA_UTC_BASE_DRU0              (&gHost_DRU_t)
#define UDMA_COMPUTE_CLUSTER0_CLEC_REGS_BASE (&gHost_clec_evtRegs)

#else

#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE             (CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE            (CSL_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE                (CSL_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE                (CSL_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP_TCHANRT_BASE               (CSL_MCU_NAVSS0_UDMASS_UDMAP_TCHANRT_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_UDMAP_RCHANRT_BASE               (CSL_MCU_NAVSS0_UDMASS_UDMAP_RCHANRT_BASE)

#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_BASE                      (CSL_NAVSS0_UDMASS_UDMAP0_CFG_BASE)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE                (CSL_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE                (CSL_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE                (CSL_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHANRT_BASE              (CSL_NAVSS0_UDMASS_UDMAP0_CFG_TCHANRT_BASE)
#define UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHANRT_BASE              (CSL_NAVSS0_UDMASS_UDMAP0_CFG_RCHANRT_BASE)

#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_GCFG_BASE           (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_GCFG_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE                (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE             (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE            (CSL_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_FIFOS_BASE              (CSL_MCU_NAVSS0_UDMASS_RINGACC0_FIFOS_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE            (CSL_MCU_NAVSS0_UDMASS_RINGACC0_ISC_BASE)

#define UDMA_NAVSS0_UDMASS_RINGACC0_GCFG_BASE                   (CSL_NAVSS0_UDMASS_RINGACC0_GCFG_BASE)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_BASE                    (CSL_NAVSS0_UDMASS_RINGACC0_CFG_BASE)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE                 (CSL_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE)
#define UDMA_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE                (CSL_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE)
#define UDMA_NAVSS0_UDMASS_RINGACC0_SRC_FIFOS_BASE              (CSL_NAVSS0_UDMASS_RINGACC0_SRC_FIFOS_BASE)
#define UDMA_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE                (CSL_NAVSS0_UDMASS_RINGACC0_ISC_BASE)

#define UDMA_MCU_NAVSS0_UDMASS_PSILSS_CFG0_PROXY_BASE           (CSL_MCU_NAVSS0_UDMASS_PSILSS_CFG0_PROXY_BASE)
#define UDMA_NAVSS0_UDMASS_PSILCFG0_CFG_PROXY_BASE              (CSL_NAVSS0_UDMASS_PSILCFG0_CFG_PROXY_BASE)

#define UDMA_MCU_NAVSS0_UDMASS_INTA0_CFG_BASE                   (CSL_MCU_NAVSS0_UDMASS_INTA0_CFG_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE                  (CSL_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_INTR_BASE                  (CSL_MCU_NAVSS0_UDMASS_INTA0_INTR_BASE)
#define UDMA_MCU_NAVSS0_PAR_UDMASS_UDMASS_INTA0_CFG_L2G_BASE    (CSL_MCU_NAVSS0_UDMASS_INTA0_I2G_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_MCAST_BASE                 (CSL_MCU_NAVSS0_UDMASS_INTA0_MCAST_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNT_BASE                  (CSL_MCU_NAVSS0_UDMASS_INTA0_GCNT_BASE)
#define UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNTRTI_BASE               (CSL_MCU_NAVSS0_UDMASS_INTA0_GCNTRTI_BASE)

#define UDMA_MCU_NAVSS0_INTR0_CFG_BASE                          (CSL_MCU_NAVSS0_INTR0_CFG_BASE)

#define UDMA_NAVSS0_UDMASS_INTA0_CFG_BASE                       (CSL_NAVSS0_UDMASS_INTA0_CFG_BASE)
#define UDMA_NAVSS0_UDMASS_INTA0_IMAP_BASE                      (CSL_NAVSS0_UDMASS_INTA0_IMAP_BASE)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_INTR_BASE                  (CSL_NAVSS0_UDMASS_INTA0_CFG_INTR_BASE)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_L2G_BASE                   (CSL_NAVSS0_UDMASS_INTA0_CFG_L2G_BASE)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_MCAST_BASE                 (CSL_NAVSS0_UDMASS_INTA0_CFG_MCAST_BASE)

#define UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTCFG_BASE               (CSL_NAVSS0_UDMASS_INTA0_CFG_GCNTCFG_BASE)
#define UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTRTI_BASE               (CSL_NAVSS0_UDMASS_INTA0_CFG_GCNTRTI_BASE)

#define UDMA_NAVSS0_INTR0_INTR_ROUTER_CFG_BASE                  (CSL_NAVSS0_INTR0_INTR_ROUTER_CFG_BASE)
/** \brief DRU0 UTC baseaddress */
#define UDMA_UTC_BASE_DRU0                                      (CSL_COMPUTE_CLUSTER0_DRU_BASE)
#define UDMA_COMPUTE_CLUSTER0_CLEC_REGS_BASE (CSL_COMPUTE_CLUSTER0_CLEC_REGS_BASE)
#endif
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

#if defined (HOST_EMULATION)
    gHost_udmap_gcfgRegs.CAP0 = 0x000B800F;
    gHost_udmap_gcfgRegs.CAP1 = 0;
    gHost_udmap_gcfgRegs.CAP2 = 0x02584078;
    gHost_udmap_gcfgRegs.CAP3 = 0x0000012C;

    gHost_intaggr_cfgRegs.INTCAP  = 0x0000000001001200;
    gHost_intaggr_cfgRegs.AUXCAP = 0x0000020000040200;
#endif

    /*
     * UDMA config init
     */
    /* Init the config structure - one time step */
    pUdmapRegs = &drvHandle->udmapRegs;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        pUdmapRegs->pGenCfgRegs     = ((CSL_udmap_gcfgRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_GCFG_BASE);
        pUdmapRegs->pRxFlowCfgRegs  = ((CSL_udmap_rxfcfgRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE);
        pUdmapRegs->pTxChanCfgRegs  = ((CSL_udmap_txccfgRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP0_TCHAN_BASE);
        pUdmapRegs->pRxChanCfgRegs  = ((CSL_udmap_rxccfgRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP0_RCHAN_BASE);
        pUdmapRegs->pTxChanRtRegs   = ((CSL_udmap_txcrtRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP_TCHANRT_BASE);
        pUdmapRegs->pRxChanRtRegs   = ((CSL_udmap_rxcrtRegs *) UDMA_MCU_NAVSS0_UDMASS_UDMAP_RCHANRT_BASE);
        drvHandle->trigGemOffset    = CSL_NAVSS_GEM_MCU_UDMA_TRIGGER_OFFSET;
    }
    else
    {
        pUdmapRegs->pGenCfgRegs     = ((CSL_udmap_gcfgRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_BASE);
        pUdmapRegs->pRxFlowCfgRegs  = ((CSL_udmap_rxfcfgRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RFLOW_BASE);
        pUdmapRegs->pTxChanCfgRegs  = ((CSL_udmap_txccfgRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHAN_BASE);
        pUdmapRegs->pRxChanCfgRegs  = ((CSL_udmap_rxccfgRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHAN_BASE);
        pUdmapRegs->pTxChanRtRegs   = ((CSL_udmap_txcrtRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_TCHANRT_BASE);
        pUdmapRegs->pRxChanRtRegs   = ((CSL_udmap_rxcrtRegs *) UDMA_NAVSS0_UDMASS_UDMAP0_CFG_RCHANRT_BASE);
        drvHandle->trigGemOffset    = CSL_NAVSS_GEM_MAIN_UDMA_TRIGGER_OFFSET;
    }
    drvHandle->clecRegs = (CSL_CLEC_EVTRegs *) UDMA_COMPUTE_CLUSTER0_CLEC_REGS_BASE;
/* UDMA not present in CC QT build. Only DRU is present */
#ifndef CC_QT_BUILD
    /* Fill other SOC specific parameters by reading from UDMA config
     * registers */
    CSL_udmapGetCfg(pUdmapRegs);
#endif

    /*
     * RA config init
     */
    pRaRegs = &drvHandle->raRegs;
    if(UDMA_INST_ID_MCU_0 == instId)
    {
        pRaRegs->pGlbRegs   = (CSL_ringacc_gcfgRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_GCFG_BASE;
        pRaRegs->pCfgRegs   = (CSL_ringacc_cfgRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_BASE;
        pRaRegs->pRtRegs    = (CSL_ringacc_rtRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE;
        pRaRegs->pMonRegs   = (CSL_ringacc_monitorRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE;
        pRaRegs->pFifoRegs  = (CSL_ringacc_fifosRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_FIFOS_BASE;
        pRaRegs->pIscRegs   = (CSL_ringacc_iscRegs *) UDMA_MCU_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE;
        pRaRegs->maxRings   = CSL_NAVSS_MCU_RINGACC_RING_CNT;
    }
    else
    {
        pRaRegs->pGlbRegs   = (CSL_ringacc_gcfgRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_GCFG_BASE;
        pRaRegs->pCfgRegs   = (CSL_ringacc_cfgRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_CFG_BASE;
        pRaRegs->pRtRegs    = (CSL_ringacc_rtRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_CFG_RT_BASE;
        pRaRegs->pMonRegs   = (CSL_ringacc_monitorRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_CFG_MON_BASE;
        pRaRegs->pFifoRegs  = (CSL_ringacc_fifosRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_SRC_FIFOS_BASE;
        pRaRegs->pIscRegs   = (CSL_ringacc_iscRegs *) UDMA_NAVSS0_UDMASS_RINGACC0_ISC_ISC_BASE;
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
    pIaRegs->pCfgRegs       = (CSL_intaggr_cfgRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_CFG_BASE;
    pIaRegs->pImapRegs      = (CSL_intaggr_imapRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_IMAP_BASE;
    pIaRegs->pIntrRegs      = (CSL_intaggr_intrRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_INTR_BASE;
    pIaRegs->pL2gRegs       = (CSL_intaggr_l2gRegs *) UDMA_MCU_NAVSS0_PAR_UDMASS_UDMASS_INTA0_CFG_L2G_BASE;
    pIaRegs->pMcastRegs     = (CSL_intaggr_mcastRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_MCAST_BASE;
    pIaRegs->pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNT_BASE;
    pIaRegs->pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) UDMA_MCU_NAVSS0_UDMASS_INTA0_GCNTRTI_BASE;
    CSL_intaggrGetCfg(pIaRegs);

    drvHandle->iaGemOffset  = CSL_NAVSS_GEM_MCU_UDMA_INTA0_SEVI_OFFSET;
    drvHandle->devIdIa      = TISCI_DEV_MCU_NAVSS0_INTAGGR_0;
#if defined (BUILD_MCU1_0)
    drvHandle->devIdCore    = TISCI_DEV_MCU_R5FSS0_CORE0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU1_0;
#else
    drvHandle->devIdCore    = TISCI_DEV_MCU_R5FSS0_CORE1;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU1_1;
#endif
#else
    /* IA config init */
    pIaRegs = &drvHandle->iaRegs;
    pIaRegs->pCfgRegs       = (CSL_intaggr_cfgRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_BASE;
    pIaRegs->pImapRegs      = (CSL_intaggr_imapRegs *) UDMA_NAVSS0_UDMASS_INTA0_IMAP_BASE;
    pIaRegs->pIntrRegs      = (CSL_intaggr_intrRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_INTR_BASE;
    pIaRegs->pL2gRegs       = (CSL_intaggr_l2gRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_L2G_BASE;
    pIaRegs->pMcastRegs     = (CSL_intaggr_mcastRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_MCAST_BASE;
    pIaRegs->pGcntCfgRegs   = (CSL_intaggr_gcntcfgRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTCFG_BASE;
    pIaRegs->pGcntRtiRegs   = (CSL_intaggr_gcntrtiRegs *) UDMA_NAVSS0_UDMASS_INTA0_CFG_GCNTRTI_BASE;
    /* UDMA not present in CC QT build. Only DRU is present */
#ifndef CC_QT_BUILD
    CSL_intaggrGetCfg(pIaRegs);
#endif

    drvHandle->iaGemOffset  = CSL_NAVSS_GEM_MAIN_UDMA_INTA0_SEVI_OFFSET;
    drvHandle->devIdIa      = TISCI_DEV_NAVSS0_UDMASS_INTAGGR_0;
    drvHandle->clecRtMap    = CSL_CLEC_RTMAP_DISABLE;
    drvHandle->clecOffset   = 0U;
#if defined (BUILD_MPU1_0)
    drvHandle->devIdCore    = TISCI_DEV_COMPUTE_CLUSTER0_GIC500SS;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MPU1_0;
#endif
#if defined (BUILD_MCU2_0)
    drvHandle->devIdCore    = TISCI_DEV_R5FSS0_CORE0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU2_0;
#endif
#if defined (BUILD_MCU2_1)
    drvHandle->devIdCore    = TISCI_DEV_R5FSS0_CORE1;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU2_1;
#endif
#if defined (BUILD_MCU3_0)
    drvHandle->devIdCore    = TISCI_DEV_R5FSS1_CORE0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU3_0;
#endif
#if defined (BUILD_MCU3_1)
    drvHandle->devIdCore    = TISCI_DEV_R5FSS1_CORE1;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_MCU3_1;
#endif
#if defined (BUILD_C66X_1)
    drvHandle->devIdCore    = TISCI_DEV_C66SS0_CORE0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_C66X_1;
#endif
#if defined (BUILD_C66X_2)
    drvHandle->devIdCore    = TISCI_DEV_C66SS1_CORE0;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_C66X_2;
#endif
#if defined (BUILD_C7X_1)
    drvHandle->devIdCore    = TISCI_DEV_COMPUTE_CLUSTER0_CLEC;
    drvHandle->druCoreId    = UDMA_DRU_CORE_ID_C7X_1;
    drvHandle->clecRtMap    = CSL_CLEC_RTMAP_CPU_4; /* CPU4 is C7x_1 in J721E */
    /* CLEC interrupt number 1024 is connected to GIC interrupt number 32 in J721E.
     * Due to this for CLEC programming one needs to add an offset of 992 (1024 - 32)
     * to the event number which is shared between GIC and CLEC. */
    drvHandle->clecOffset   = 1024U - 32U;
#endif
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
    utcInfo->txCredit      = 2U;
    utcInfo->druRegs       = ((CSL_DRU_t *) UDMA_UTC_BASE_DRU0);
    utcInfo->numQueue      = CSL_NAVSS_UTC_MSMC_DRU_QUEUE_CNT;

    utcInfo = &drvHandle->utcInfo[UDMA_UTC_ID_VPAC_TC0];
    utcInfo->utcId         = UDMA_UTC_ID_VPAC_TC0;
    utcInfo->utcType       = UDMA_UTC_TYPE_DRU_VHWA;
    utcInfo->startCh       = UDMA_UTC_START_CH_VPAC_TC0;
    utcInfo->numCh         = UDMA_UTC_NUM_CH_VPAC_TC0;
    utcInfo->startThreadId = UDMA_UTC_START_THREAD_ID_VPAC_TC0;
    utcInfo->txCredit      = 3U;
    utcInfo->druRegs       = ((CSL_DRU_t *) CSL_VPAC0_DRU_UTC_VPAC0_DRU_MMR_CFG_DRU_DRU_BASE);
    utcInfo->numQueue      = CSL_NAVSS_UTC_VPAC_TC0_QUEUE_CNT;

    utcInfo = &drvHandle->utcInfo[UDMA_UTC_ID_VPAC_TC1];
    utcInfo->utcId         = UDMA_UTC_ID_VPAC_TC1;
    utcInfo->utcType       = UDMA_UTC_TYPE_DRU_VHWA;
    utcInfo->startCh       = UDMA_UTC_START_CH_VPAC_TC1;
    utcInfo->numCh         = UDMA_UTC_NUM_CH_VPAC_TC1;
    utcInfo->startThreadId = UDMA_UTC_START_THREAD_ID_VPAC_TC1;
    utcInfo->txCredit      = 3U;
    utcInfo->druRegs       = ((CSL_DRU_t *) CSL_VPAC0_DRU_UTC_VPAC1_DRU_MMR_CFG_DRU_DRU_BASE);
    utcInfo->numQueue      = CSL_NAVSS_UTC_VPAC_TC1_QUEUE_CNT;

    utcInfo = &drvHandle->utcInfo[UDMA_UTC_ID_DMPAC_TC0];
    utcInfo->utcId         = UDMA_UTC_ID_DMPAC_TC0;
    utcInfo->utcType       = UDMA_UTC_TYPE_DRU_VHWA;
    utcInfo->startCh       = UDMA_UTC_START_CH_DMPAC_TC0;
    utcInfo->numCh         = UDMA_UTC_NUM_CH_DMPAC_TC0;
    utcInfo->startThreadId = UDMA_UTC_START_THREAD_ID_DMPAC_TC0;
    utcInfo->txCredit      = 3U;
    utcInfo->druRegs       = ((CSL_DRU_t *) CSL_DMPAC0_DRU_UTC_DMPAC0_DRU_MMR_CFG_DRU_DRU_BASE);
    utcInfo->numQueue      = CSL_NAVSS_UTC_DMPAC_QUEUE_CNT;

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
#if defined (BUILD_MCU2_0)
    coreId = UDMA_CORE_ID_MCU2_0;
#endif
#if defined (BUILD_MCU2_1)
    coreId = UDMA_CORE_ID_MCU2_1;
#endif
#if defined (BUILD_MCU3_0)
    coreId = UDMA_CORE_ID_MCU3_0;
#endif
#if defined (BUILD_MCU3_1)
    coreId = UDMA_CORE_ID_MCU3_1;
#endif
#if defined (BUILD_C7X_1)
    coreId = UDMA_CORE_ID_C7X_1;
#endif
#if defined (BUILD_C66X_1)
    coreId = UDMA_CORE_ID_C66X_1;
#endif
#if defined (BUILD_C66X_2)
    coreId = UDMA_CORE_ID_C66X_2;
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

#if defined (BUILD_MPU1_0) || defined (BUILD_C7X_1)
    isCacheCoherent = TRUE;
#else
    isCacheCoherent = FALSE;
#endif

    return (isCacheCoherent);
}
