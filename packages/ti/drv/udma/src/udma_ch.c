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
 *  \file udma_ch.c
 *
 *  \brief File containing the UDMA driver channel related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/udma/src/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t Udma_chCheckParams(Udma_DrvHandle drvHandle,
                                  uint32_t chType,
                                  const Udma_ChPrms *chPrms);
static int32_t Udma_chAllocResource(Udma_ChHandle chHandle);
static int32_t Udma_chFreeResource(Udma_ChHandle chHandle);
static int32_t Udma_chPair(Udma_ChHandle chHandle);
static int32_t Udma_chUnpair(Udma_ChHandle chHandle);
static int32_t Udma_chEnableLocal(Udma_ChHandle chHandle);
static int32_t Udma_chDisableBlkCpyChan(Udma_ChHandle chHandle, uint32_t timeout);
static int32_t Udma_chDisableTxChan(Udma_ChHandle chHandle, uint32_t timeout);
static int32_t Udma_chDisableRxChan(Udma_ChHandle chHandle, uint32_t timeout);
static int32_t Udma_chDisableExtChan(Udma_ChHandle chHandle, uint32_t timeout);
static uint32_t Udma_chGetTriggerEvent(Udma_DrvHandle drvHandle,
                                       Udma_ChHandle chHandle,
                                       uint32_t trigger);

static int32_t Udma_psilcfgSetRtEnable(Udma_DrvHandle drvHandle,
                                       uint32_t threadId,
                                       uint32_t bEnable);
static int32_t Udma_psilcfgSetEnable(Udma_DrvHandle drvHandle,
                                     uint32_t threadId,
                                     uint32_t bEnable);
static int32_t Udma_psilcfgWrite(Udma_DrvHandle drvHandle,
                                 uint32_t threadId,
                                 uint32_t regId,
                                 uint32_t data);
static int32_t Udma_psilcfgRead(Udma_DrvHandle drvHandle,
                                uint32_t threadId,
                                uint32_t regId,
                                uint32_t *pData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_chOpen(Udma_DrvHandle drvHandle,
                    Udma_ChHandle chHandle,
                    uint32_t chType,
                    const Udma_ChPrms *chPrms)
{
    int32_t     retVal = UDMA_SOK, tempRetVal;
    uint32_t    allocDone = (uint32_t) FALSE;

    /* Error check */
    if((drvHandle == NULL_PTR) || (NULL_PTR == chHandle) || (NULL_PTR == chPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(drvHandle->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_chCheckParams(drvHandle, chType, chPrms);
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy and init parameters */
        (void) memset(chHandle, 0, sizeof(struct Udma_ChObj));
        (void) memcpy(&chHandle->chPrms, chPrms, sizeof(Udma_ChPrms));
        chHandle->chType            = chType;
        chHandle->drvHandle         = drvHandle;
        chHandle->utcInfo           = (const Udma_UtcInstInfo *) NULL_PTR;
        chHandle->txChNum           = UDMA_DMA_CH_INVALID;
        chHandle->rxChNum           = UDMA_DMA_CH_INVALID;
        chHandle->extChNum          = UDMA_DMA_CH_INVALID;
        chHandle->pdmaChNum         = UDMA_DMA_CH_INVALID;
        chHandle->peerThreadId      = UDMA_THREAD_ID_INVALID;
        chHandle->fqRing            = (Udma_RingHandle) NULL_PTR;
        chHandle->cqRing            = (Udma_RingHandle) NULL_PTR;
        chHandle->tdCqRing          = (Udma_RingHandle) NULL_PTR;
        UdmaChTxPrms_init(&chHandle->txPrms, chType);
        UdmaChRxPrms_init(&chHandle->rxPrms, chType);
        UdmaChUtcPrms_init(&chHandle->utcPrms);
        chHandle->pTxCfgRegs        = (volatile CSL_udmap_txccfgRegs_chan *) NULL_PTR;
        chHandle->pTxRtRegs         = (volatile CSL_udmap_txcrtRegs_chan *) NULL_PTR;
        chHandle->pRxCfgRegs        = (volatile CSL_udmap_rxccfgRegs_chan *) NULL_PTR;
        chHandle->pRxRtRegs         = (volatile CSL_udmap_rxcrtRegs_chan *) NULL_PTR;
        chHandle->pExtCfgRegs       = (volatile CSL_udmap_txccfgRegs_chan *) NULL_PTR;
        chHandle->pExtRtRegs        = (volatile CSL_udmap_txcrtRegs_chan *) NULL_PTR;
        chHandle->pDruNrtRegs       = (volatile CSL_DRU_CHNRTRegs_CHNRT *) NULL_PTR;
        chHandle->pDruRtRegs        = (volatile CSL_DRU_CHRTRegs_CHRT *) NULL_PTR;
        chHandle->chOesAllocDone    = FALSE;
        chHandle->trigger           = CSL_UDMAP_TR_FLAGS_TRIGGER_NONE;

        if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            /* Get UTC instance object pointer */
            chHandle->utcInfo = Udma_chGetUtcInst(drvHandle, chPrms->utcId);
            if(NULL_PTR == chHandle->utcInfo)
            {
                retVal = UDMA_EINVALID_PARAMS;
                Udma_printf(drvHandle, "[Error] Invalid UTC ID!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Alloc UDMA channel/rings */
        retVal = Udma_chAllocResource(chHandle);
        if(UDMA_SOK == retVal)
        {
            allocDone = (uint32_t) TRUE;
        }
        else
        {
            Udma_printf(drvHandle, "[Error] Channel resource allocation failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Pair channels */
        retVal = Udma_chPair(chHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA channel paring failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        chHandle->chInitDone        = UDMA_INIT_DONE;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            tempRetVal = Udma_chFreeResource(chHandle);
            if(UDMA_SOK != tempRetVal)
            {
                Udma_printf(drvHandle, "[Error] Free resource failed!!!\n");
            }
        }
    }

    return (retVal);
}

int32_t Udma_chClose(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(TRUE == chHandle->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle, "[Error] Channel OES not de-allocated!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Unpair channels */
        retVal = Udma_chUnpair(chHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA channel unparing failed!!\n");
        }

        /* Free-up UDMA channel/rings */
        retVal += Udma_chFreeResource(chHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] Free resource failed!!!\n");
        }

        (void) memset(chHandle, 0, sizeof(*chHandle));
        chHandle->chInitDone = UDMA_DEINIT_DONE;
    }

    return (retVal);
}

int32_t Udma_chConfigTx(Udma_ChHandle chHandle, const Udma_ChTxPrms *txPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle      drvHandle;
    struct tisci_msg_rm_udmap_tx_ch_cfg_req     rmUdmaTxReq;
    struct tisci_msg_rm_udmap_tx_ch_cfg_resp    rmUdmaTxResp;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       ((chHandle->chType & UDMA_CH_FLAG_TX) != UDMA_CH_FLAG_TX))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

        /* Copy params */
        rmUdmaTxReq.valid_params        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FDEPTH_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_TX_CREDIT_COUNT_VALID;
        rmUdmaTxReq.nav_id              = drvHandle->devIdUdma;
        rmUdmaTxReq.index               = (uint16_t)chHandle->txChNum;
        rmUdmaTxReq.tx_pause_on_err     = txPrms->pauseOnError;
        rmUdmaTxReq.tx_filt_einfo       = txPrms->filterEinfo;
        rmUdmaTxReq.tx_filt_pswords     = txPrms->filterPsWords;
        rmUdmaTxReq.tx_atype            = txPrms->addrType;
        rmUdmaTxReq.tx_chan_type        = txPrms->chanType;
        rmUdmaTxReq.tx_fetch_size       = txPrms->fetchWordSize;
        rmUdmaTxReq.tx_priority         = txPrms->busPriority;
        rmUdmaTxReq.tx_qos              = txPrms->busQos;
        rmUdmaTxReq.tx_orderid          = txPrms->busOrderId;
        rmUdmaTxReq.fdepth              = txPrms->fifoDepth;
        rmUdmaTxReq.tx_burst_size       = txPrms->burstSize;
        rmUdmaTxReq.tx_sched_priority   = txPrms->dmaPriority;
        rmUdmaTxReq.tx_credit_count     = txPrms->txCredit;
        if(NULL_PTR != chHandle->tdCqRing)
        {
            Udma_assert(drvHandle,
                chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            /* used for pass by value and teardown */
            rmUdmaTxReq.txcq_qnum       = chHandle->tdCqRing->ringNum;
            rmUdmaTxReq.tx_supr_tdpkt   = txPrms->supressTdCqPkt;
        }
        else
        {
            /* TD CQ not used */
            rmUdmaTxReq.txcq_qnum       = UDMA_RING_INVALID;
            rmUdmaTxReq.tx_supr_tdpkt   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_ENABLED;
        }

        /* Config UDMAP TX channel */
        retVal = Sciclient_rmUdmapTxChCfg(
                     &rmUdmaTxReq, &rmUdmaTxResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA TX config failed!!!\n");
        }

        /* Copy the config */
        (void) memcpy(&chHandle->txPrms, txPrms, sizeof(chHandle->txPrms));
    }

    return (retVal);
}

int32_t Udma_chConfigRx(Udma_ChHandle chHandle, const Udma_ChRxPrms *rxPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle      drvHandle;
    struct tisci_msg_rm_udmap_rx_ch_cfg_req     rmUdmaRxReq;
    struct tisci_msg_rm_udmap_rx_ch_cfg_resp    rmUdmaRxResp;
    Udma_FlowPrms       flowPrms;
    uint16_t            cqRing, fqRing;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       ((chHandle->chType & UDMA_CH_FLAG_RX) != UDMA_CH_FLAG_RX))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Note: Block copy uses same RX channel as TX */
        Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);

        /* Copy params */
        rmUdmaRxReq.valid_params        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_START_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_RX_FLOWID_CNT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_SHORT_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_RX_IGNORE_LONG_VALID |
                                          TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_VALID;
        rmUdmaRxReq.nav_id              = drvHandle->devIdUdma;
        rmUdmaRxReq.index               = (uint16_t)chHandle->rxChNum;
        rmUdmaRxReq.rx_pause_on_err     = rxPrms->pauseOnError;
        rmUdmaRxReq.rx_atype            = rxPrms->addrType;
        rmUdmaRxReq.rx_chan_type        = rxPrms->chanType;
        rmUdmaRxReq.rx_fetch_size       = rxPrms->fetchWordSize;
        rmUdmaRxReq.rx_priority         = rxPrms->busPriority;
        rmUdmaRxReq.rx_qos              = rxPrms->busQos;
        rmUdmaRxReq.rx_orderid          = rxPrms->busOrderId;
        rmUdmaRxReq.rx_sched_priority   = rxPrms->dmaPriority;
        rmUdmaRxReq.flowid_start        = rxPrms->flowIdFwRangeStart;
        rmUdmaRxReq.flowid_cnt          = rxPrms->flowIdFwRangeCnt;
        rmUdmaRxReq.rx_ignore_short     = rxPrms->ignoreShortPkts;
        rmUdmaRxReq.rx_ignore_long      = rxPrms->ignoreLongPkts;
        rmUdmaRxReq.rx_burst_size       = rxPrms->burstSize;
        if(NULL_PTR != chHandle->tdCqRing)
        {
            Udma_assert(drvHandle,
                chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            /* used for pass by value and teardown */
            rmUdmaRxReq.rxcq_qnum          = chHandle->tdCqRing->ringNum;
        }
        else
        {
            /* TD CQ not used */
            rmUdmaRxReq.rxcq_qnum          = UDMA_RING_INVALID;
        }

        /* Config UDMAP RX channel */
        retVal = Sciclient_rmUdmapRxChCfg(
                     &rmUdmaRxReq, &rmUdmaRxResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA RX config failed!!!\n");
        }

        /* Configure default flow for PDMA and other PSIL channels */
        if((((chHandle->chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA) ||
                ((chHandle->chType & UDMA_CH_FLAG_PSIL) == UDMA_CH_FLAG_PSIL)) &&
           (TRUE == rxPrms->configDefaultFlow))
        {
            UdmaFlowPrms_init(&flowPrms, chHandle->chType);

            if(NULL_PTR == chHandle->cqRing)
            {
                /* Ring not allocated */
                cqRing = UDMA_RING_INVALID;
            }
            else
            {
                cqRing = chHandle->cqRing->ringNum;
                Udma_assert(drvHandle, cqRing != UDMA_RING_INVALID);
            }
            if(NULL_PTR == chHandle->fqRing)
            {
                /* Ring not allocated */
                fqRing = UDMA_RING_INVALID;
            }
            else
            {
                fqRing = chHandle->fqRing->ringNum;
                Udma_assert(drvHandle, fqRing != UDMA_RING_INVALID);
            }

            flowPrms.defaultRxCQ    = cqRing;
            /* Use the same free queue as default flow is not used in
             * selecting different queues based on threshold */
            flowPrms.fdq0Sz0Qnum    = fqRing;
            flowPrms.fdq0Sz1Qnum    = fqRing;
            flowPrms.fdq0Sz2Qnum    = fqRing;
            flowPrms.fdq0Sz3Qnum    = fqRing;
            flowPrms.fdq1Qnum       = fqRing;
            flowPrms.fdq2Qnum       = fqRing;
            flowPrms.fdq3Qnum       = fqRing;

            /* Config default flow. Caller can overwite again if required */
            retVal = Udma_flowConfig(chHandle->defaultFlow, 0U, &flowPrms);
            if(UDMA_SOK != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] UDMAP default RX flow config failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy the config */
        (void) memcpy(&chHandle->rxPrms, rxPrms, sizeof(chHandle->rxPrms));
    }

    return (retVal);
}

int32_t Udma_chConfigUtc(Udma_ChHandle chHandle, const Udma_ChUtcPrms *utcPrms)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                utcChNum;
    Udma_DrvHandle          drvHandle;
    struct tisci_msg_rm_udmap_tx_ch_cfg_req     rmUdmaTxReq;
    struct tisci_msg_rm_udmap_tx_ch_cfg_resp    rmUdmaTxResp;
    const Udma_UtcInstInfo *utcInfo;
    CSL_DruChConfig         druChCfg;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       ((chHandle->chType & UDMA_CH_FLAG_UTC) != UDMA_CH_FLAG_UTC))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        utcInfo = chHandle->utcInfo;
        Udma_assert(drvHandle, utcInfo != NULL_PTR);
        Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);
        Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
        utcChNum = chHandle->extChNum - utcInfo->startCh;

        /* Direct TR mode doesn't need UDMAP channel programming */
        if(CSL_DRU_OWNER_UDMAC_TR == utcPrms->druOwner)
        {
            /* Copy params */
            rmUdmaTxReq.valid_params        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERR_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_CHAN_TYPE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_FETCH_SIZE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_CQ_QNUM_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_PRIORITY_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_QOS_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_ORDER_ID_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_SCHED_PRIORITY_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_EINFO_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FILT_PSWORDS_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_TX_SUPR_TDPKT_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_TX_FDEPTH_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_VALID |
                                              TISCI_MSG_VALUE_RM_UDMAP_CH_TX_CREDIT_COUNT_VALID;
            rmUdmaTxReq.nav_id              = drvHandle->devIdUdma;
            rmUdmaTxReq.index               = (uint16_t)(chHandle->extChNum + drvHandle->extChOffset);
            rmUdmaTxReq.tx_pause_on_err     = utcPrms->pauseOnError;
            rmUdmaTxReq.tx_filt_einfo       = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED;
            rmUdmaTxReq.tx_filt_pswords     = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED;
            rmUdmaTxReq.tx_atype            = utcPrms->addrType;
            rmUdmaTxReq.tx_chan_type        = utcPrms->chanType;
            rmUdmaTxReq.tx_fetch_size       = utcPrms->fetchWordSize;
            rmUdmaTxReq.tx_priority         = utcPrms->busPriority;
            rmUdmaTxReq.tx_qos              = utcPrms->busQos;
            rmUdmaTxReq.tx_orderid          = utcPrms->busOrderId;
            rmUdmaTxReq.fdepth              = 0U;   /* Not used for external ch */
            rmUdmaTxReq.tx_burst_size       = utcPrms->burstSize;
            rmUdmaTxReq.tx_sched_priority   = utcPrms->dmaPriority;
            rmUdmaTxReq.tx_credit_count     = utcInfo->txCredit;
            if(NULL_PTR != chHandle->tdCqRing)
            {
                Udma_assert(drvHandle,
                    chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
                /* used for pass by value and teardown */
                rmUdmaTxReq.txcq_qnum       = chHandle->tdCqRing->ringNum;
                rmUdmaTxReq.tx_supr_tdpkt   = utcPrms->supressTdCqPkt;
            }
            else
            {
                /* TD CQ not used */
                rmUdmaTxReq.txcq_qnum       = UDMA_RING_INVALID;
                rmUdmaTxReq.tx_supr_tdpkt   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_ENABLED;
            }

            /* Config UDMA UTC channel */
            retVal = Sciclient_rmUdmapTxChCfg(
                         &rmUdmaTxReq, &rmUdmaTxResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] UDMA UTC config failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Configure DRU */
        if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
        {
            Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);

            /* Disable the channel before any configuration */
            retVal = CSL_druChDisable(utcInfo->druRegs, utcChNum);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] DRU channel disable failed!!\n");
            }

            if(UDMA_SOK == retVal)
            {
                druChCfg.type       = 0U;   /* Not used */
                druChCfg.owner      = utcPrms->druOwner;
                druChCfg.pauseOnErr = utcPrms->pauseOnError;
                druChCfg.evtNum     = UDMA_EVENT_INVALID;
                druChCfg.queueId    = (uint64_t)utcPrms->druQueueId;
                retVal = CSL_druChConfig(utcInfo->druRegs, utcChNum, &druChCfg);
                if(CSL_PASS != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] DRU channel config failed!!\n");
                }
            }
        }

        /* DRU in VHWA doesn't need enable/disable. Just config is sufficient */
        if(UDMA_UTC_TYPE_DRU_VHWA == utcInfo->utcType)
        {
            Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);

            druChCfg.type       = 0U;   /* Not used */
            druChCfg.owner      = CSL_DRU_OWNER_UDMAC_TR;   /* Always through UDMAC */
            druChCfg.pauseOnErr = utcPrms->pauseOnError;
            druChCfg.evtNum     = UDMA_EVENT_INVALID;
            druChCfg.queueId    = (uint64_t)utcPrms->druQueueId;
            retVal = CSL_druChConfig(utcInfo->druRegs, utcChNum, &druChCfg);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] VHWA DRU channel config failed!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy the config */
        (void) memcpy(&chHandle->utcPrms, utcPrms, sizeof(chHandle->utcPrms));
    }

    return (retVal);
}

int32_t Udma_chConfigPdma(Udma_ChHandle chHandle,
                          const Udma_ChPdmaPrms *pdmaPrms)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        regVal;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (NULL_PTR == pdmaPrms) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       ((chHandle->chType & UDMA_CH_FLAG_PDMA) != UDMA_CH_FLAG_PDMA))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->pTxRtRegs != NULL_PTR);

            regVal = CSL_REG32_RD(&chHandle->pTxRtRegs->PEER8);
            CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 0U);
            CSL_REG32_WR(&chHandle->pTxRtRegs->PEER8, regVal);

            regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_X, pdmaPrms->elemSize) |
                     CSL_FMK(PSILCFG_REG_STATIC_TR_Y, pdmaPrms->elemCnt);
            CSL_REG32_WR(&chHandle->pTxRtRegs->PEER0, regVal);

            regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_Z, pdmaPrms->fifoCnt);
            CSL_REG32_WR(&chHandle->pTxRtRegs->PEER1, regVal);
        }
        else
        {
            Udma_assert(drvHandle, chHandle->pRxRtRegs != NULL_PTR);

            regVal = CSL_REG32_RD(&chHandle->pRxRtRegs->PEER8);
            CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 0U);
            CSL_REG32_WR(&chHandle->pRxRtRegs->PEER8, regVal);

            regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_X, pdmaPrms->elemSize) |
                     CSL_FMK(PSILCFG_REG_STATIC_TR_Y, pdmaPrms->elemCnt);
            CSL_REG32_WR(&chHandle->pRxRtRegs->PEER0, regVal);

            regVal = CSL_FMK(PSILCFG_REG_STATIC_TR_Z, pdmaPrms->fifoCnt);
            CSL_REG32_WR(&chHandle->pRxRtRegs->PEER1, regVal);
        }
    }

    return (retVal);
}

int32_t Udma_chEnable(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Enable channel based on channel type */
        retVal = Udma_chEnableLocal(chHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA channel enable failed!!\n");
        }
    }

    return (retVal);
}

int32_t Udma_chDisable(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Call disable sequence for respective modes */
        if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            retVal = Udma_chDisableBlkCpyChan(chHandle, timeout);
        }
        else if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            retVal = Udma_chDisableExtChan(chHandle, timeout);
        }
        else
        {
            if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                retVal = Udma_chDisableTxChan(chHandle, timeout);
            }
            else
            {
                retVal = Udma_chDisableRxChan(chHandle, timeout);
            }
        }
    }

    return (retVal);
}

int32_t Udma_chPause(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    uint32_t                utcChNum;
    const Udma_UtcInstInfo *utcInfo;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
            (void) CSL_udmapPauseTxChan(&drvHandle->udmapRegs, chHandle->txChNum);
        }

        if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            /* Note: Block copy uses same RX channel. So do for both TX/RX */
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
            (void) CSL_udmapPauseRxChan(&drvHandle->udmapRegs, chHandle->rxChNum);
        }

        if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            utcInfo = chHandle->utcInfo;
            Udma_assert(drvHandle, utcInfo != NULL_PTR);

            /* Same TX channel CSL API is used for UTC as well - but need to
             * add the EXT channel offset */
            Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);

            /* Direct TR mode doesn't need UDMAP channel programming */
            if(CSL_DRU_OWNER_UDMAC_TR == chHandle->utcPrms.druOwner)
            {
                (void) CSL_udmapPauseTxChan(
                    &drvHandle->udmapRegs,
                    chHandle->extChNum + drvHandle->extChOffset);
            }

            /* Pause DRU incase of direct TR mode */
            if((UDMA_UTC_TYPE_DRU == utcInfo->utcType) &&
               (CSL_DRU_OWNER_DIRECT_TR == chHandle->utcPrms.druOwner))
            {
                Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
                Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
                utcChNum = chHandle->extChNum - utcInfo->startCh;

                retVal = CSL_druChPause(utcInfo->druRegs, utcChNum);
                if(CSL_PASS != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] DRU channel pause failed!!\n");
                }
            }
        }
    }

    return (retVal);
}

int32_t Udma_chResume(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    uint32_t                utcChNum;
    const Udma_UtcInstInfo *utcInfo;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
            (void) CSL_udmapUnpauseTxChan(&drvHandle->udmapRegs, chHandle->txChNum);
        }

        if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            /* Note: Block copy uses same RX channel. So do for both TX/RX */
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
            (void) CSL_udmapUnpauseRxChan(&drvHandle->udmapRegs, chHandle->rxChNum);
        }

        if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            utcInfo = chHandle->utcInfo;
            Udma_assert(drvHandle, utcInfo != NULL_PTR);

            /* Same TX channel CSL API is used for UTC as well - but need to
             * add the EXT channel offset */
            Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);

            /* Direct TR mode doesn't need UDMAP channel programming */
            if(CSL_DRU_OWNER_UDMAC_TR == chHandle->utcPrms.druOwner)
            {
                (void) CSL_udmapUnpauseTxChan(
                    &drvHandle->udmapRegs,
                    chHandle->extChNum + drvHandle->extChOffset);
            }

            /* Resume DRU incase of direct TR mode */
            if((UDMA_UTC_TYPE_DRU == utcInfo->utcType) &&
               (CSL_DRU_OWNER_DIRECT_TR == chHandle->utcPrms.druOwner))
            {
                Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
                Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
                utcChNum = chHandle->extChNum - utcInfo->startCh;

                retVal = CSL_druChResume(utcInfo->druRegs, utcChNum);
                if(CSL_PASS != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] DRU channel resume failed!!\n");
                }
            }
        }
    }

    return (retVal);
}

uint32_t Udma_chGetNum(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        chNum = UDMA_DMA_CH_INVALID;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            Udma_assert(drvHandle, chHandle->utcInfo != NULL_PTR);
            Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);
            Udma_assert(drvHandle, chHandle->extChNum >= chHandle->utcInfo->startCh);
            /* Provide the channel offset within a UTC */
            chNum = chHandle->extChNum - chHandle->utcInfo->startCh;
        }
        else
        {
            if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
                chNum = chHandle->txChNum;
            }
            else
            {
                Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
                chNum = chHandle->rxChNum;
            }
        }
    }

    return (chNum);
}

Udma_RingHandle Udma_chGetFqRingHandle(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_RingHandle fqRing = (Udma_RingHandle) NULL_PTR;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        fqRing = chHandle->fqRing;
    }

    return (fqRing);
}

Udma_RingHandle Udma_chGetCqRingHandle(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_RingHandle cqRing = (Udma_RingHandle) NULL_PTR;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        cqRing = chHandle->cqRing;
    }

    return (cqRing);
}

uint16_t Udma_chGetFqRingNum(Udma_ChHandle chHandle)
{
    uint16_t        ringNum = UDMA_RING_INVALID;
    Udma_RingHandle ringHandle;

    ringHandle = Udma_chGetFqRingHandle(chHandle);
    if(ringHandle != NULL_PTR)
    {
        ringNum = Udma_ringGetNum(ringHandle);
    }

    return (ringNum);
}

uint16_t Udma_chGetCqRingNum(Udma_ChHandle chHandle)
{
    uint16_t        ringNum = UDMA_RING_INVALID;
    Udma_RingHandle ringHandle;

    ringHandle = Udma_chGetCqRingHandle(chHandle);
    if(ringHandle != NULL_PTR)
    {
        ringNum = Udma_ringGetNum(ringHandle);
    }

    return (ringNum);
}

Udma_FlowHandle Udma_chGetDefaultFlowHandle(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_FlowHandle defaultFlow = (Udma_FlowHandle) NULL_PTR;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == chHandle) || (chHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        defaultFlow = chHandle->defaultFlow;
    }

    return (defaultFlow);
}

int32_t Udma_chDequeueTdResponse(Udma_ChHandle chHandle,
                                 CSL_UdmapTdResponse *tdResponse)
{
    int32_t     retVal = UDMA_SOK, cslRetVal;
    uint64_t    response;

    if((NULL_PTR != chHandle->tdCqRing) &&
       (chHandle->tdCqRing->ringNum != UDMA_RING_INVALID) &&
       (NULL_PTR != tdResponse))
    {
        cslRetVal = CSL_ringaccPop64(
            &chHandle->drvHandle->raRegs,
            &chHandle->tdCqRing->cfg,
            &response,
            &Udma_ringaccMemOps);
        if(0 != cslRetVal)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            CSL_udmapGetTdResponse(response, tdResponse);
        }
    }
    else
    {
        retVal = UDMA_EFAIL;
    }

    return (retVal);
}

int32_t Udma_chSetSwTrigger(Udma_ChHandle chHandle, uint32_t trigger)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    const Udma_UtcInstInfo *utcInfo;

    /* Error check */
    if((NULL_PTR == chHandle) ||
       (chHandle->chInitDone != UDMA_INIT_DONE) ||
       (trigger > CSL_UDMAP_TR_FLAGS_TRIGGER_LOCAL_EVENT) ||
       (trigger == CSL_UDMAP_TR_FLAGS_TRIGGER_NONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX))
        {
            Udma_assert(drvHandle, chHandle->pTxRtRegs != NULL_PTR);
            CSL_REG32_WR(
                &chHandle->pTxRtRegs->SWTRIG, ((uint32_t)1U << (trigger - 1U)));
        }
        else if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                        "[Error] SW trigger not supported for RX channels!!!\n");
        }
        else
        {
            utcInfo = chHandle->utcInfo;
            Udma_assert(drvHandle, utcInfo != NULL_PTR);
            if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
            {
                Udma_assert(drvHandle, chHandle->pDruRtRegs != NULL_PTR);
                CSL_REG64_WR(
                    &chHandle->pDruRtRegs->CHRT_SWTRIG,
                    ((uint64_t)1U << (trigger - 1U)));
            }
            else
            {
                retVal = UDMA_EFAIL;
                Udma_printf(drvHandle,
                            "[Error] SW trigger not supported for other UTCs !!!\n");
            }
        }
    }

    return (retVal);
}

int32_t Udma_chSetChaining(Udma_ChHandle triggerChHandle,
                           Udma_ChHandle chainedChHandle,
                           uint32_t trigger)
{
    int32_t                             retVal = UDMA_SOK;
    Udma_DrvHandle                      drvHandle;
    uint32_t                            triggerEvent;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    /* Error check */
    if((NULL_PTR == triggerChHandle) || (triggerChHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if((NULL_PTR == chainedChHandle) || (chainedChHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = triggerChHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(TRUE == triggerChHandle->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                "[Error] Trigger channel OES already allocated!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Get the global trigger event to set */
        triggerEvent =
            Udma_chGetTriggerEvent(drvHandle, chainedChHandle, trigger);
        if(UDMA_EVENT_INVALID == triggerEvent)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle, "[Error] UDMA invalid trigger mode!!\n");
       }
    }

    if(UDMA_SOK == retVal)
    {
        rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.src_id                 = drvHandle->devIdUdma;
        rmIrqReq.global_event           = (uint16_t) triggerEvent;
        rmIrqReq.src_index              = 0U;
        rmIrqReq.dst_id                 = 0U;
        rmIrqReq.dst_host_irq           = 0U;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        if(((triggerChHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((triggerChHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            /* Use RX channel for block copy as it signifies transfer done */
            Udma_assert(drvHandle,
                triggerChHandle->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandle->rxChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START;
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] RM RX Channel chain config failed!!!\n");
            }
        }
        else if((triggerChHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle,
                triggerChHandle->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandle->txChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START;
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] RM TX Channel chain config failed!!!\n");
            }
        }
        else
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                "[Error] UDMA chaining not supported for UTC channels!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Mark OES alloc flag */
        triggerChHandle->chOesAllocDone = TRUE;
        triggerChHandle->trigger        = trigger;
    }

    return (retVal);
}

int32_t Udma_chBreakChaining(Udma_ChHandle triggerChHandle,
                             Udma_ChHandle chainedChHandle)
{
    int32_t                             retVal = UDMA_SOK;
    Udma_DrvHandle                      drvHandle;
    uint32_t                            triggerEvent;
    struct tisci_msg_rm_irq_release_req rmIrqReq;

    /* Error check */
    if((NULL_PTR == triggerChHandle) || (triggerChHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if((NULL_PTR == chainedChHandle) || (chainedChHandle->chInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = triggerChHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        if(FALSE == triggerChHandle->chOesAllocDone)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                "[Error] Trigger channel OES not allocated!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Get the global trigger event to set */
        triggerEvent =
            Udma_chGetTriggerEvent(drvHandle, chainedChHandle, triggerChHandle->trigger);
        if(UDMA_EVENT_INVALID == triggerEvent)
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle, "[Error] UDMA invalid trigger mode!!\n");
       }
    }

    if(UDMA_SOK == retVal)
    {
        rmIrqReq.valid_params           = TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.src_id                 = drvHandle->devIdUdma;
        rmIrqReq.src_index              = 0U;
        rmIrqReq.global_event           = (uint16_t)triggerEvent;
        rmIrqReq.dst_id                 = 0U;
        rmIrqReq.dst_host_irq           = 0U;
        rmIrqReq.ia_id                  = 0U;
        rmIrqReq.vint                   = 0U;
        rmIrqReq.vint_status_bit_index  = 0U;
        rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

        if(((triggerChHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((triggerChHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            /* Use RX channel for block copy as it signifies transfer done */
            Udma_assert(drvHandle,
                triggerChHandle->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandle->rxChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START;
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] RM RX Channel chain reset failed!!!\n");
            }
        }
        else if((triggerChHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle,
                triggerChHandle->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)triggerChHandle->txChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START;
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] RM TX Channel chain reset failed!!!\n");
            }
        }
        else
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                "[Error] UDMA chaining not supported for UTC channels!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Mark OES alloc flag as free */
        triggerChHandle->chOesAllocDone = FALSE;
    }

    return (retVal);
}

void UdmaChPrms_init(Udma_ChPrms *chPrms, uint32_t chType)
{
    if(NULL_PTR != chPrms)
    {
        chPrms->chNum       = UDMA_DMA_CH_ANY;
        chPrms->peerChNum   = UDMA_DMA_CH_INVALID;
        if((UDMA_CH_TYPE_TR_BLK_COPY == chType) ||
           (UDMA_CH_TYPE_UTC == chType))
        {
            chPrms->peerChNum   = UDMA_DMA_CH_NA;
        }
        chPrms->utcId       = UDMA_UTC_ID_INVALID;
        chPrms->appData     = NULL_PTR;
        UdmaRingPrms_init(&chPrms->fqRingPrms);
        UdmaRingPrms_init(&chPrms->cqRingPrms);
        UdmaRingPrms_init(&chPrms->tdCqRingPrms);
        /* TD and TR response is always 8 byte irrespective of mode */
        chPrms->tdCqRingPrms.elemSize = UDMA_RING_ES_8BYTES;
    }

    return;
}

void UdmaChTxPrms_init(Udma_ChTxPrms *txPrms, uint32_t chType)
{
    if(NULL_PTR != txPrms)
    {
        txPrms->pauseOnError    = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED;
        txPrms->filterEinfo     = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_EINFO_DISABLED;
        txPrms->filterPsWords   = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_FILT_PSWORDS_DISABLED;
        txPrms->addrType        = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS;
        txPrms->chanType        = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_PACKET;
        if((chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            txPrms->chanType    = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF;
        }
        txPrms->fetchWordSize   = 16U;  /* sizeof(CSL_UdmapTR15) / sizeof(uint32_t) */
        txPrms->busPriority     = UDMA_DEFAULT_TX_CH_BUS_PRIORITY;
        txPrms->busQos          = UDMA_DEFAULT_TX_CH_BUS_QOS;
        txPrms->busOrderId      = UDMA_DEFAULT_TX_CH_BUS_ORDERID;
        txPrms->dmaPriority     = UDMA_DEFAULT_TX_CH_DMA_PRIORITY;
        txPrms->txCredit        = 0U;
        if((chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            txPrms->fifoDepth   = (uint16_t)CSL_NAVSS_UDMAP_TX_UHC_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else if((chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            txPrms->fifoDepth   = (uint16_t)CSL_NAVSS_UDMAP_TX_HC_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else
        {
            txPrms->fifoDepth   = (uint16_t)CSL_NAVSS_UDMAP_TX_CHANS_FDEPTH;
            txPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES;
        }
        txPrms->supressTdCqPkt  = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_DISABLED;
    }

    return;
}

void UdmaChRxPrms_init(Udma_ChRxPrms *rxPrms, uint32_t chType)
{
    if(NULL_PTR != rxPrms)
    {
        rxPrms->pauseOnError        = TISCI_MSG_VALUE_RM_UDMAP_CH_PAUSE_ON_ERROR_DISABLED;
        rxPrms->addrType            = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS;
        rxPrms->chanType            = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_PACKET;
        if((chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
        {
            rxPrms->chanType        = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_BLOCK_REF;
        }
        rxPrms->fetchWordSize       = 16U;  /* sizeof(CSL_UdmapTR15) / sizeof(uint32_t) */
        rxPrms->busPriority         = UDMA_DEFAULT_RX_CH_BUS_PRIORITY;
        rxPrms->busQos              = UDMA_DEFAULT_RX_CH_BUS_QOS;
        rxPrms->busOrderId          = UDMA_DEFAULT_RX_CH_BUS_ORDERID;
        rxPrms->dmaPriority         = UDMA_DEFAULT_RX_CH_DMA_PRIORITY;
        rxPrms->flowIdFwRangeStart  = 0U;       /* Reset value - to use default flow */
        rxPrms->flowIdFwRangeCnt    = 0x4000U;  /* Reset value - to use default flow */
        rxPrms->ignoreShortPkts     = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION;
        rxPrms->ignoreLongPkts      = TISCI_MSG_VALUE_RM_UDMAP_RX_CH_PACKET_EXCEPTION;
        rxPrms->configDefaultFlow   = TRUE;
        if((chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else if((chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_256_BYTES;
        }
        else
        {
            rxPrms->burstSize   = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_64_BYTES;
        }
    }

    return;
}

void UdmaChUtcPrms_init(Udma_ChUtcPrms *utcPrms)
{
    if(NULL_PTR != utcPrms)
    {
        utcPrms->pauseOnError   = (uint8_t)0U;
        utcPrms->addrType       = TISCI_MSG_VALUE_RM_UDMAP_CH_ATYPE_PHYS;
        utcPrms->chanType       = TISCI_MSG_VALUE_RM_UDMAP_CH_TYPE_3P_DMA_REF;
        utcPrms->fetchWordSize  = 16U;  /* sizeof(CSL_UdmapTR15) / sizeof(uint32_t) */
        utcPrms->busPriority    = UDMA_DEFAULT_UTC_CH_BUS_PRIORITY;
        utcPrms->busQos         = UDMA_DEFAULT_UTC_CH_BUS_QOS;
        utcPrms->busOrderId     = UDMA_DEFAULT_UTC_CH_BUS_ORDERID;
        utcPrms->dmaPriority    = UDMA_DEFAULT_UTC_CH_DMA_PRIORITY;
        utcPrms->burstSize      = TISCI_MSG_VALUE_RM_UDMAP_CH_BURST_SIZE_128_BYTES;
        utcPrms->supressTdCqPkt = TISCI_MSG_VALUE_RM_UDMAP_TX_CH_SUPPRESS_TD_DISABLED;
        utcPrms->druOwner       = CSL_DRU_OWNER_UDMAC_TR;
        utcPrms->druQueueId     = UDMA_DEFAULT_UTC_DRU_QUEUE_ID;
    }

    return;
}

void UdmaChPdmaPrms_init(Udma_ChPdmaPrms *pdmaPrms)
{
    if(NULL_PTR != pdmaPrms)
    {
        pdmaPrms->elemSize  = UDMA_PDMA_ES_8BITS;
        pdmaPrms->elemCnt   = 0U;
        pdmaPrms->fifoCnt   = 0U;
    }

    return;
}

const Udma_UtcInstInfo *Udma_chGetUtcInst(Udma_DrvHandle drvHandle,
                                          uint32_t utcId)

{
    const Udma_UtcInstInfo *utcInfo = (const Udma_UtcInstInfo *) NULL_PTR;
#if (UDMA_NUM_UTC_INSTANCE > 0)
    uint32_t                i;

    for(i = 0U; i < UDMA_NUM_UTC_INSTANCE; i++)
    {
        if(drvHandle->utcInfo[i].utcId == utcId)
        {
            utcInfo = &drvHandle->utcInfo[i];
            break;
        }
    }
#endif

    return (utcInfo);
}

int32_t Udma_chGetStats(Udma_ChHandle chHandle, Udma_ChStats *chStats)
{
    int32_t            retVal = UDMA_SOK;
    Udma_DrvHandle     drvHandle;
    CSL_UdmapChanStats chanStats;
    uint32_t           chNum;
    CSL_UdmapChanDir   chDir;

    /* Error check */
    if ((NULL_PTR == chHandle)                   ||
        (chHandle->chInitDone != UDMA_INIT_DONE) ||
        (chStats == NULL))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = chHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) ==
            UDMA_CH_FLAG_BLK_COPY)
        {
            chNum = chHandle->txChNum;
            chDir = CSL_UDMAP_CHAN_DIR_TX;
        }
        else if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
        {
            chNum = chHandle->extChNum + drvHandle->extChOffset;
            chDir = CSL_UDMAP_CHAN_DIR_TX;
        }
        else
        {
            if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                chNum = chHandle->txChNum;
                chDir = CSL_UDMAP_CHAN_DIR_TX;
            }
            else
            {
                chNum = chHandle->rxChNum;
                chDir = CSL_UDMAP_CHAN_DIR_RX;
            }
        }
        CSL_udmapGetChanStats(&drvHandle->udmapRegs, chNum, chDir, &chanStats);
        (void)memcpy(chStats, &chanStats, sizeof(Udma_ChStats));
    }

    return (retVal);
}

static int32_t Udma_chCheckParams(Udma_DrvHandle drvHandle,
                                  uint32_t chType,
                                  const Udma_ChPrms *chPrms)
{
    int32_t     retVal = UDMA_SOK;

    if((chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
    {
        if(UDMA_UTC_ID_INVALID == chPrms->utcId)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle, "[Error] Invalid UTC ID!!!\n");
        }
    }
    if((chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA)
    {
        if((UDMA_DMA_CH_INVALID == chPrms->peerChNum) ||
           (UDMA_DMA_CH_NA == chPrms->peerChNum))
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle, "[Error] Invalid Peer Channel Number!!!\n");
        }
    }

    return (retVal);
}

static int32_t Udma_chAllocResource(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK, tempRetVal;
    Udma_DrvHandle          drvHandle;
    uint32_t                utcChNum;
    uint16_t                ringNum;
    const Udma_UtcInstInfo *utcInfo;

    drvHandle = chHandle->drvHandle;

    /* Allocate UDMAP channel */
    if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
    {
        if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyHcCh(chHandle->chPrms.chNum, drvHandle);
        }
        else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyUhcCh(chHandle->chPrms.chNum, drvHandle);
        }
        else
        {
            chHandle->txChNum =
                Udma_rmAllocBlkCopyCh(chHandle->chPrms.chNum, drvHandle);
        }
        if(UDMA_DMA_CH_INVALID == chHandle->txChNum)
        {
            retVal = UDMA_EALLOC;
            Udma_printf(drvHandle, "[Error] RM Alloc Blkcpy Ch failed!!!\n");
        }
        else
        {
            /* RX channel same as TX channel for block copy */
            chHandle->rxChNum = chHandle->txChNum;
            /* Add thread offset and or RX relative channel as the thread
             * already has bit info (CSL_PSILCFG_DEST_THREAD_OFFSET)
             * for destination thread */
            chHandle->peerThreadId =
                chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
        }
    }
    else if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
    {
        utcInfo = chHandle->utcInfo;
        Udma_assert(drvHandle, utcInfo != NULL_PTR);
        /* Allocate external channel */
        chHandle->extChNum = Udma_rmAllocExtCh(
                                 chHandle->chPrms.chNum,
                                 drvHandle,
                                 utcInfo);
        if(UDMA_DMA_CH_INVALID == chHandle->extChNum)
        {
            retVal = UDMA_EALLOC;
            Udma_printf(drvHandle, "[Error] RM Alloc Ext Ch failed!!!\n");
        }
        else
        {
            Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
            utcChNum = chHandle->extChNum - utcInfo->startCh;
            chHandle->peerThreadId = utcChNum + utcInfo->startThreadId;
            if(NULL_PTR != utcInfo->druRegs)
            {
                Udma_assert(drvHandle, utcChNum < 512U);    /* Array check */
                chHandle->pDruNrtRegs  = &utcInfo->druRegs->CHNRT[utcChNum];
                chHandle->pDruRtRegs   = &utcInfo->druRegs->CHRT[utcChNum];
            }
        }
    }
    else
    {
        /* Allocate UDMAP for PDMA channels */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                chHandle->txChNum =
                    Udma_rmAllocTxHcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                chHandle->txChNum =
                    Udma_rmAllocTxUhcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else
            {
                chHandle->txChNum =
                    Udma_rmAllocTxCh(chHandle->chPrms.chNum, drvHandle);
            }
            if(UDMA_DMA_CH_INVALID == chHandle->txChNum)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] RM Alloc TX Ch failed!!!\n");
            }
        }
        else
        {
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxHcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxUhcCh(chHandle->chPrms.chNum, drvHandle);
            }
            else
            {
                chHandle->rxChNum =
                    Udma_rmAllocRxCh(chHandle->chPrms.chNum, drvHandle);
            }
            if(UDMA_DMA_CH_INVALID == chHandle->rxChNum)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] RM Alloc RX Ch failed!!!\n");
            }
            else
            {
                /* Assign default flow */
                chHandle->defaultFlow               = &chHandle->defaultFlowObj;
                chHandle->defaultFlow->drvHandle    = drvHandle;
                chHandle->defaultFlow->flowStart    = chHandle->rxChNum;
                chHandle->defaultFlow->flowCnt      = 1U;
                chHandle->defaultFlow->flowInitDone = UDMA_INIT_DONE;
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Allocate peer channel for PDMA channels */
            if((chHandle->chType & UDMA_CH_FLAG_PDMA) == UDMA_CH_FLAG_PDMA)
            {
                /* PDMA peer channel assignment */
                chHandle->pdmaChNum = chHandle->chPrms.peerChNum;
                /* Thread ID already added while getting PDMA channel num */
                chHandle->peerThreadId = chHandle->pdmaChNum;
            }

            if((chHandle->chType & UDMA_CH_FLAG_PSIL) == UDMA_CH_FLAG_PSIL)
            {
                chHandle->peerThreadId = chHandle->chPrms.peerChNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free queue ring number is same as UDMAP channel number */
        if(NULL_PTR != chHandle->chPrms.fqRingPrms.ringMem)
        {
            /* Allocate only when memory is provided */
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) ==
                UDMA_CH_FLAG_BLK_COPY)
            {
                /* Same as TX channel incase of block copy */
                ringNum = (uint16_t)(chHandle->txChNum + drvHandle->txChOffset);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
            {
                ringNum = (uint16_t)(chHandle->extChNum + drvHandle->extChOffset);
            }
            else
            {
                if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
                {
                    ringNum = (uint16_t)(chHandle->txChNum + drvHandle->txChOffset);
                }
                else
                {
                    ringNum = (uint16_t)(chHandle->rxChNum + drvHandle->rxChOffset);
                }
            }

            chHandle->fqRing = &chHandle->fqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->fqRing,
                         ringNum,
                         &chHandle->chPrms.fqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->fqRing = (Udma_RingHandle) NULL_PTR;
                Udma_printf(drvHandle, "[Error] FQ ring alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate completion ring only when memory is provided */
        if(NULL_PTR != chHandle->chPrms.cqRingPrms.ringMem)
        {
            chHandle->cqRing = &chHandle->cqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->cqRing,
                         UDMA_RING_ANY,
                         &chHandle->chPrms.cqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->cqRing = (Udma_RingHandle) NULL_PTR;
                Udma_printf(drvHandle, "[Error] CQ ring alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate teardown completion ring only when memory is provided */
        if(NULL_PTR != chHandle->chPrms.tdCqRingPrms.ringMem)
        {
            chHandle->tdCqRing = &chHandle->tdCqRingObj;
            retVal = Udma_ringAlloc(
                         drvHandle,
                         chHandle->tdCqRing,
                         UDMA_RING_ANY,
                         &chHandle->chPrms.tdCqRingPrms);
            if(UDMA_SOK != retVal)
            {
                chHandle->tdCqRing = (Udma_RingHandle) NULL_PTR;
                Udma_printf(drvHandle, "[Error] TD CQ ring alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK != retVal)
    {
        tempRetVal = Udma_chFreeResource(chHandle);
        if(UDMA_SOK != tempRetVal)
        {
            Udma_printf(drvHandle, "[Error] Free resource failed!!!\n");
        }
    }
    else
    {
        /* Assign overlay pointers */
        if(chHandle->txChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->txChNum <
                    (sizeof(CSL_udmap_txcrtRegs) /
                        sizeof(CSL_udmap_txcrtRegs_chan)));
            chHandle->pTxCfgRegs =
                &drvHandle->udmapRegs.pTxChanCfgRegs->CHAN[chHandle->txChNum];
            chHandle->pTxRtRegs  =
                &drvHandle->udmapRegs.pTxChanRtRegs->CHAN[chHandle->txChNum];
        }
        if(chHandle->rxChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pRxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pRxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->rxChNum <
                    (sizeof(CSL_udmap_rxcrtRegs) /
                        sizeof(CSL_udmap_rxcrtRegs_chan)));
            chHandle->pRxCfgRegs =
                &drvHandle->udmapRegs.pRxChanCfgRegs->CHAN[chHandle->rxChNum];
            chHandle->pRxRtRegs  =
                &drvHandle->udmapRegs.pRxChanRtRegs->CHAN[chHandle->rxChNum];
        }
        if(chHandle->extChNum != UDMA_DMA_CH_INVALID)
        {
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanCfgRegs != NULL_PTR);
            Udma_assert(drvHandle, drvHandle->udmapRegs.pTxChanRtRegs != NULL_PTR);
            Udma_assert(drvHandle,
                (chHandle->extChNum + drvHandle->extChOffset) <
                    (sizeof(CSL_udmap_txcrtRegs) /
                        sizeof(CSL_udmap_txcrtRegs_chan)));
            chHandle->pExtCfgRegs =
                &drvHandle->udmapRegs.pTxChanCfgRegs->CHAN[
                                chHandle->extChNum + drvHandle->extChOffset];
            chHandle->pExtRtRegs  =
                &drvHandle->udmapRegs.pTxChanRtRegs->CHAN[
                                chHandle->extChNum + drvHandle->extChOffset];
        }
    }

    return (retVal);
}

static int32_t Udma_chFreeResource(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    drvHandle = chHandle->drvHandle;
    if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
    {
        if(UDMA_DMA_CH_INVALID != chHandle->txChNum)
        {
            /* TX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeBlkCopyHcCh(chHandle->txChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeBlkCopyUhcCh(chHandle->txChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeBlkCopyCh(chHandle->txChNum, drvHandle);
            }
            chHandle->txChNum = UDMA_DMA_CH_INVALID;
            chHandle->rxChNum = UDMA_DMA_CH_INVALID;
        }
    }
    else
    {
        if(UDMA_DMA_CH_INVALID != chHandle->txChNum)
        {
            /* TX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeTxHcCh(chHandle->txChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeTxUhcCh(chHandle->txChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeTxCh(chHandle->txChNum, drvHandle);
            }
            chHandle->txChNum = UDMA_DMA_CH_INVALID;
        }
        if(UDMA_DMA_CH_INVALID != chHandle->rxChNum)
        {
            /* RX channel free */
            if((chHandle->chType & UDMA_CH_FLAG_HC) == UDMA_CH_FLAG_HC)
            {
                Udma_rmFreeRxHcCh(chHandle->rxChNum, drvHandle);
            }
            else if((chHandle->chType & UDMA_CH_FLAG_UHC) == UDMA_CH_FLAG_UHC)
            {
                Udma_rmFreeRxUhcCh(chHandle->rxChNum, drvHandle);
            }
            else
            {
                Udma_rmFreeRxCh(chHandle->rxChNum, drvHandle);
            }
            chHandle->rxChNum = UDMA_DMA_CH_INVALID;
        }

        chHandle->defaultFlowObj.drvHandle    = (Udma_DrvHandle) NULL_PTR;
        chHandle->defaultFlowObj.flowStart    = UDMA_FLOW_INVALID;
        chHandle->defaultFlowObj.flowCnt      = 0U;
        chHandle->defaultFlowObj.flowInitDone = UDMA_DEINIT_DONE;
        chHandle->defaultFlow                 = (Udma_FlowHandle) NULL_PTR;
    }
    if(UDMA_DMA_CH_INVALID != chHandle->extChNum)
    {
        /* External channel free */
        Udma_rmFreeExtCh(chHandle->extChNum, drvHandle, chHandle->utcInfo);
        chHandle->extChNum = UDMA_DMA_CH_INVALID;
    }
    chHandle->pdmaChNum = UDMA_DMA_CH_INVALID;
    chHandle->peerThreadId = UDMA_THREAD_ID_INVALID;

    if(NULL_PTR != chHandle->fqRing)
    {
        retVal += Udma_ringFree(chHandle->fqRing);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] RM Free FQ ring failed!!!\n");
        }
        chHandle->fqRing = (Udma_RingHandle) NULL_PTR;
    }
    if(NULL_PTR != chHandle->cqRing)
    {
        retVal += Udma_ringFree(chHandle->cqRing);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] RM Free CQ ring failed!!!\n");
        }
        chHandle->cqRing = (Udma_RingHandle) NULL_PTR;
    }
    if(NULL_PTR != chHandle->tdCqRing)
    {
        retVal += Udma_ringFree(chHandle->tdCqRing);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] RM Free TDCQ ring failed!!!\n");
        }
        chHandle->tdCqRing = (Udma_RingHandle) NULL_PTR;
    }

    return (retVal);
}

static int32_t Udma_chPair(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    struct tisci_msg_rm_psil_pair_req rmPairReq;

    drvHandle = chHandle->drvHandle;

    if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
    {
        /* For UTC, pairing not required. Enable done as part of enable API */
    }
    else
    {
        rmPairReq.nav_id = drvHandle->devIdPsil;
        /* Do TX check first so that TX becomes source thread for block copy */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

            /* For TX, UDMAP channel is source */
            rmPairReq.src_thread = chHandle->txChNum + drvHandle->udmapSrcThreadOffset;
            rmPairReq.dst_thread = chHandle->peerThreadId;
        }
        else    /* RX channel */
        {
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);

            /* For RX, UDMAP channel is destination */
            rmPairReq.src_thread = chHandle->peerThreadId;
            rmPairReq.dst_thread = chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
        }

        /* Pair source thread with destination thread */
        retVal = Sciclient_rmPsilPair(&rmPairReq, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] RM PSI Pairing failed!!!\n");
        }
    }

    return (retVal);
}

static int32_t Udma_chUnpair(Udma_ChHandle chHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;
    uint32_t        dstThreadId;
    struct tisci_msg_rm_psil_unpair_req rmUnpairReq;

    drvHandle = chHandle->drvHandle;

    if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
    {
        if(CSL_DRU_OWNER_UDMAC_TR == chHandle->utcPrms.druOwner)
        {
            /* For UTC, destination thread disable should be done */
            dstThreadId = chHandle->peerThreadId;
            retVal = Udma_psilcfgSetRtEnable(drvHandle, dstThreadId, FALSE);
            if(UDMA_SOK != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] PSI UTC destination thread RT disable failed!!!\n");
            }
            retVal += Udma_psilcfgSetEnable(drvHandle, dstThreadId, FALSE);
            if(UDMA_SOK != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] PSI UTC destination thread disable failed!!!\n");
            }
        }
    }
    else
    {
        rmUnpairReq.nav_id = drvHandle->devIdPsil;
        /* Do TX check first so that TX becomes source thread for block copy */
        if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

            /* For TX, UDMAP channel is source */
            rmUnpairReq.src_thread = chHandle->txChNum + drvHandle->udmapSrcThreadOffset;
            rmUnpairReq.dst_thread = chHandle->peerThreadId;
        }
        else    /* RX channel */
        {
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);

            /* For RX, UDMAP channel is destination */
            rmUnpairReq.src_thread = chHandle->peerThreadId;
            rmUnpairReq.dst_thread = chHandle->rxChNum + drvHandle->udmapDestThreadOffset;
        }

        /* Unpair source thread with destination thread */
        retVal = Sciclient_rmPsilUnpair(&rmUnpairReq, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] RM PSI Un-Pairing failed!!!\n");
        }
    }

    return (retVal);
}

static int32_t Udma_chEnableLocal(Udma_ChHandle chHandle)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                regVal;
    uint32_t                utcChNum;
    Udma_DrvHandle          drvHandle;
    const Udma_UtcInstInfo *utcInfo;
    CSL_UdmapRT             rtEnable;
    uint32_t                srcThreadId;

    drvHandle = chHandle->drvHandle;

    /* Set only enable and clear all other flags which might be set from
     * previous run */
    rtEnable.enable         = TRUE;
    rtEnable.teardown       = FALSE;
    rtEnable.forcedTeardown = FALSE;
    rtEnable.pause          = FALSE;
    rtEnable.error          = FALSE;

    if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
    {
        Udma_assert(drvHandle, chHandle->pTxRtRegs != NULL_PTR);
        Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

        regVal = CSL_REG32_RD(&chHandle->pTxRtRegs->PEER8);
        CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 1U);
        CSL_REG32_WR(&chHandle->pTxRtRegs->PEER8, regVal);

        (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtEnable);
    }

    if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
    {
        Udma_assert(drvHandle, chHandle->pRxRtRegs != NULL_PTR);
        Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);

        /*
         * Note: UDMAP shoule be enabled first (receiver) before enabling
         *       PEER/PDMA through PSIL (source)
         *       This ensures UDMAP RX is ready to receive data
         */
        /* Note: Block copy uses same RX channel. So do for both TX/RX */
        (void) CSL_udmapSetRxRT(
            &drvHandle->udmapRegs, chHandle->rxChNum, &rtEnable);

        regVal = CSL_REG32_RD(&chHandle->pRxRtRegs->PEER8);
        CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, (uint32_t) 1U);
        CSL_REG32_WR(&chHandle->pRxRtRegs->PEER8, regVal);
    }

    if((chHandle->chType & UDMA_CH_FLAG_UTC) == UDMA_CH_FLAG_UTC)
    {
        utcInfo = chHandle->utcInfo;
        Udma_assert(drvHandle, utcInfo != NULL_PTR);
        if(CSL_DRU_OWNER_DIRECT_TR == chHandle->utcPrms.druOwner)
        {
            /* Enable DRU incase of direct TR mode */
            Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
            Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
            utcChNum = chHandle->extChNum - utcInfo->startCh;

            retVal = CSL_druChEnable(utcInfo->druRegs, utcChNum);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] DRU channel enable failed!!\n");
            }
        }
        else
        {
            /* For UTC, destination thread enable should be done -
             * pairing not required. Use src thread instead of dest thread
             * (already has bit info CSL_PSILCFG_DEST_THREAD_OFFSET,
             * so just reset the bit) */
            srcThreadId = chHandle->peerThreadId &
                                ~((uint32_t) CSL_PSILCFG_DEST_THREAD_OFFSET);
            retVal = Udma_psilcfgSetEnable(drvHandle, srcThreadId, TRUE);
            if(UDMA_SOK != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] PSI UTC destination thread enable failed!!!\n");
            }
            else
            {
                retVal = Udma_psilcfgSetRtEnable(drvHandle, srcThreadId, TRUE);
                if(UDMA_SOK != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] PSI UTC destination thread RT enable failed!!!\n");
                }
            }

            /* Same TX channel CSL API is used for UTC as well - but need to
             * add the EXT channel offset */
            Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);
            (void) CSL_udmapSetTxRT(
                &drvHandle->udmapRegs,
                chHandle->extChNum + drvHandle->extChOffset,
                &rtEnable);
        }
    }

    return (retVal);
}

static int32_t Udma_chDisableBlkCpyChan(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        currTimeout = 0U;
    Udma_DrvHandle  drvHandle;
    CSL_UdmapRT     rtStatus;

    drvHandle = chHandle->drvHandle;
    Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);

    /* Initiate graceful teardown first - Source is udma thread for TX */
    retVal = CSL_udmapTeardownTxChan(
                 &drvHandle->udmapRegs, chHandle->txChNum, (bool)false, (bool)false);
    if(CSL_PASS != retVal)
    {
        Udma_printf(drvHandle, "[Error] UDMA Blkcpy teardown failed!!\n");
    }

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
        if(FALSE == rtStatus.enable)
        {
            /* Teardown complete */
            break;
        }

        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) Osal_delay(1U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        retVal = CSL_udmapTeardownTxChan(
                     &drvHandle->udmapRegs, chHandle->txChNum, (bool)true, (bool)false);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA Blkcpy force disable failed!!\n");
        }

        /* Wait for disable to complete */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
            if(FALSE == rtStatus.enable)
            {
                /* Teardown complete */
                break;
            }

            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                Udma_printf(drvHandle, "[Error] Blockcpy ch teardown timed out!!!\n");
            }
            else
            {
                (void) Osal_delay(1U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Clear teardown and enable bits in UDMAP */
        rtStatus.enable   = FALSE;
        rtStatus.teardown = FALSE;
        rtStatus.forcedTeardown = FALSE;
        (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
    }

    return (retVal);
}

static int32_t Udma_chDisableTxChan(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        peerRtEnable = 0U, currTimeout = 0U;
    Udma_DrvHandle  drvHandle;
    CSL_UdmapRT     rtStatus;
    uint32_t        rtEnableRegOffset;

    drvHandle = chHandle->drvHandle;
    Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
    rtEnableRegOffset = CSL_PSILCFG_REG_RT_ENABLE - CSL_PSILCFG_REG_STATIC_TR;

    /* Initiate graceful teardown first - Source is udma thread for TX */
    retVal = CSL_udmapTeardownTxChan(
                 &drvHandle->udmapRegs, chHandle->txChNum, (bool)false, (bool)false);
    if(CSL_PASS != retVal)
    {
        Udma_printf(drvHandle, "[Error] UDMA TX teardown failed!!\n");
    }

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
        if(FALSE == rtStatus.enable)
        {
            /* Teardown complete */
            break;
        }

        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) Osal_delay(1U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        retVal = CSL_udmapTeardownTxChan(
                     &drvHandle->udmapRegs, chHandle->txChNum, (bool)true, (bool)false);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA TX force disable failed!!\n");
        }

        /* Set flush in peer */
        (void) CSL_udmapGetChanPeerReg(
            &drvHandle->udmapRegs,
            chHandle->txChNum,
            CSL_UDMAP_CHAN_DIR_TX,
            rtEnableRegOffset,
            &peerRtEnable);
        CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_FLUSH, (uint32_t) 1U);
        (void) CSL_udmapSetChanPeerReg(
            &drvHandle->udmapRegs,
            chHandle->txChNum,
            CSL_UDMAP_CHAN_DIR_TX,
            rtEnableRegOffset,
            &peerRtEnable);

        /* Wait for disable to complete */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            (void) CSL_udmapGetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
            (void) CSL_udmapGetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->txChNum,
                CSL_UDMAP_CHAN_DIR_TX,
                rtEnableRegOffset, &peerRtEnable);
            if((FALSE == rtStatus.enable) &&
               (CSL_FEXT(peerRtEnable, PSILCFG_REG_RT_ENABLE_ENABLE) == FALSE))
            {
                /* Teardown complete */
                break;
            }

            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                Udma_printf(drvHandle, "[Error] TX ch teardown timed out!!!\n");
            }
            else
            {
                (void) Osal_delay(1U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Clear teardown and enable bits in both UDMAP and peer */
        rtStatus.enable   = FALSE;
        rtStatus.teardown = FALSE;
        rtStatus.forcedTeardown = FALSE;
        CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) 0U);
        (void) CSL_udmapSetTxRT(&drvHandle->udmapRegs, chHandle->txChNum, &rtStatus);
        (void) CSL_udmapSetChanPeerReg(
            &drvHandle->udmapRegs,
            chHandle->txChNum,
            CSL_UDMAP_CHAN_DIR_TX,
            rtEnableRegOffset,
            &peerRtEnable);
    }

    return (retVal);
}

static int32_t Udma_chDisableRxChan(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        currTimeout = 0U, regVal;
    Udma_DrvHandle  drvHandle;
    CSL_UdmapRT     rtStatus;
    uint32_t        peerRtEnable = 0U, peerRtEnableBit = 0U;
    uint32_t        rtEnableRegOffset;

    drvHandle = chHandle->drvHandle;
    Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
    Udma_assert(drvHandle, chHandle->peerThreadId != UDMA_THREAD_ID_INVALID);
    rtEnableRegOffset = CSL_PSILCFG_REG_RT_ENABLE - CSL_PSILCFG_REG_STATIC_TR;

    /* Initiate graceful teardown first - Source is peer thread for RX */
    regVal = CSL_REG32_RD(&chHandle->pRxRtRegs->PEER8);
    CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) 1U);
    CSL_REG32_WR(&chHandle->pRxRtRegs->PEER8, regVal);

    /* Check if graceful teardown is complete */
    while(UDMA_SOK == retVal)
    {
        (void) CSL_udmapGetRxRT(&drvHandle->udmapRegs, chHandle->rxChNum, &rtStatus);
        if(FALSE == rtStatus.enable)
        {
            /* Teardown complete */
            break;
        }

        if(currTimeout > timeout)
        {
            retVal = UDMA_ETIMEOUT;
        }
        else
        {
            (void) Osal_delay(1U);
            currTimeout++;
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Graceful teardown failed - initiate force teardown */
        retVal = CSL_udmapTeardownRxChan(
                     &drvHandle->udmapRegs, chHandle->rxChNum, (bool)true, (bool)false);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA RX force disable failed!!\n");
        }

        /* Wait for disable to complete - both locally and for peer thread */
        currTimeout = 0U;
        while(UDMA_SOK == retVal)
        {
            (void) CSL_udmapGetRxRT(&drvHandle->udmapRegs, chHandle->rxChNum, &rtStatus);
            (void) CSL_udmapGetChanPeerReg(
                &drvHandle->udmapRegs,
                chHandle->rxChNum,
                CSL_UDMAP_CHAN_DIR_RX,
                rtEnableRegOffset,
                &peerRtEnable);
            peerRtEnableBit = CSL_FEXT(peerRtEnable, PSILCFG_REG_RT_ENABLE_ENABLE);
            if((FALSE == rtStatus.enable) && (FALSE == peerRtEnableBit))
            {
                /* Teardown complete */
                break;
            }

            if(currTimeout > timeout)
            {
                retVal = UDMA_ETIMEOUT;
                Udma_printf(drvHandle, "[Error] RX ch teardown timed out!!!\n");
            }
            else
            {
                (void) Osal_delay(1U);
                currTimeout++;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Clear teardown bits in both the UDMAP and peer */
        rtStatus.teardown = FALSE;   /* Note that other bits are cleared from previous call */
        CSL_FINS(peerRtEnable, PSILCFG_REG_RT_ENABLE_TDOWN, (uint32_t) FALSE);
        (void) CSL_udmapSetRxRT(
            &drvHandle->udmapRegs, chHandle->rxChNum, &rtStatus);
        (void) CSL_udmapSetChanPeerReg(
            &drvHandle->udmapRegs,
            chHandle->rxChNum,
            CSL_UDMAP_CHAN_DIR_RX,
            rtEnableRegOffset,
            &peerRtEnable);
    }

    return (retVal);
}

static int32_t Udma_chDisableExtChan(Udma_ChHandle chHandle, uint32_t timeout)
{
    int32_t                 retVal = UDMA_SOK;
    Udma_DrvHandle          drvHandle;
    uint32_t                utcChNum, srcThreadId;
    uint32_t                status;
    uint32_t                currTimeout = 0U;
    const Udma_UtcInstInfo *utcInfo;

    drvHandle = chHandle->drvHandle;

    utcInfo = chHandle->utcInfo;
    Udma_assert(drvHandle, utcInfo != NULL_PTR);
    Udma_assert(drvHandle, chHandle->peerThreadId != UDMA_THREAD_ID_INVALID);
    Udma_assert(drvHandle, chHandle->extChNum != UDMA_DMA_CH_INVALID);
    if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
    {
        Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
        Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
        utcChNum = chHandle->extChNum - utcInfo->startCh;
    }

    if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
    {
        if(CSL_DRU_OWNER_DIRECT_TR == chHandle->utcPrms.druOwner)
        {
            retVal = CSL_druChTeardown(utcInfo->druRegs, utcChNum);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] DRU channel teardown failed!!\n");
            }

            /* Wait for teardown to complete */
            while(UDMA_SOK == retVal)
            {
                status = CSL_druChIsTeardownComplete(utcInfo->druRegs, utcChNum);
                if(TRUE == status)
                {
                    /* Teardown complete */
                    break;
                }

                if(currTimeout > timeout)
                {
                    retVal = UDMA_ETIMEOUT;
                    Udma_printf(drvHandle, "[Error] DRU ch teardown timed out!!!\n");
                }
                else
                {
                    (void) Osal_delay(1U);
                    currTimeout++;
                }
            }
        }
        else
        {
            /*
             * Initiate teardown - DRU is special - should be done as per below only
             */
            /* Use src thread instead of dest thread
             * (already has bit info CSL_PSILCFG_DEST_THREAD_OFFSET,
             * so just reset the bit) */
            srcThreadId = chHandle->peerThreadId &
                                ~((uint32_t) CSL_PSILCFG_DEST_THREAD_OFFSET);
            retVal = Udma_psilcfgWrite(
                         drvHandle,
                         srcThreadId,
                         CSL_PSILCFG_REG_ENABLE,
                         CSL_PSILCFG_REG_ENABLE_TEARDOWN_MASK);
            if(UDMA_SOK != retVal)
            {
                Udma_printf(drvHandle, "[Error] PSIL write teardown failed!!\n");
            }

            /* Wait for teardown to complete */
            while(UDMA_SOK == retVal)
            {
                status = CSL_druChIsTeardownComplete(utcInfo->druRegs, utcChNum);
                if(TRUE == status)
                {
                    /* Teardown complete */
                    break;
                }

                if(currTimeout > timeout)
                {
                    retVal = UDMA_ETIMEOUT;
                    Udma_printf(drvHandle, "[Error] DRU ch teardown timed out!!!\n");
                }
                else
                {
                    (void) Osal_delay(1U);
                    currTimeout++;
                }
            }

            /* Disable External channel - this will clear the Enable bit */
            retVal += CSL_udmapDisableTxChan(
                          &drvHandle->udmapRegs,
                          chHandle->extChNum + drvHandle->extChOffset);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] UDMA UTC disable failed!!\n");
            }
        }
    }
    else
    {
        /*
         * Initiate teardown - External channel is special - should be done
         * as per below only
         */
        /* Use src thread instead of dest thread
         * (already has bit info CSL_PSILCFG_DEST_THREAD_OFFSET,
         * so just reset the bit) */
        srcThreadId = chHandle->peerThreadId &
                            ~((uint32_t) CSL_PSILCFG_DEST_THREAD_OFFSET);
        retVal = Udma_psilcfgWrite(
                     drvHandle,
                     srcThreadId,
                     CSL_PSILCFG_REG_ENABLE,
                     CSL_PSILCFG_REG_ENABLE_TEARDOWN_MASK);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] PSIL UTC teardown failed!!\n");
        }

        /* Disable External channel - this will clear the Enable bit */
        retVal += CSL_udmapDisableTxChan(
                      &drvHandle->udmapRegs,
                      chHandle->extChNum + drvHandle->extChOffset);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] UDMA UTC disable failed!!\n");
        }
    }

    return (retVal);
}

static uint32_t Udma_chGetTriggerEvent(Udma_DrvHandle drvHandle,
                                       Udma_ChHandle chHandle,
                                       uint32_t trigger)
{
    uint32_t        triggerEvent = UDMA_EVENT_INVALID;

    if((CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0 == trigger) ||
       (CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger))
    {
        /* Global 0/1 triggers are interleaved - so multiply by 2 */
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX))
        {
            /* For block copy return the TX channel trigger */
            triggerEvent = (chHandle->txChNum * 2U);
            if(CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger)
            {
                triggerEvent++; /* Global1 trigger is next to global0 */
            }
            /* Add the global offset */
            triggerEvent += drvHandle->trigGemOffset;
        }
        else if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
        {
            /* RX trigger is after TX channel triggers
             * Note: There is no external channel triggers - hence not
             * using rxChOffset */
            triggerEvent  = (drvHandle->udmapRegs.txChanCnt * 2U);
            triggerEvent += (chHandle->rxChNum * 2U);
            if(CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL1 == trigger)
            {
                triggerEvent++; /* Global1 trigger is next to global0 */
            }
            /* Add the global offset */
            triggerEvent += drvHandle->trigGemOffset;
        }
        else
        {
            /* Trigger not supported for external channel -
             * return no event - already set */
        }
    }

    return (triggerEvent);
}

static int32_t Udma_psilcfgSetRtEnable(Udma_DrvHandle drvHandle,
                                       uint32_t threadId,
                                       uint32_t bEnable)
{
    uint32_t    regVal;
    int32_t     retVal;

    retVal = Udma_psilcfgRead(
                 drvHandle, threadId, CSL_PSILCFG_REG_RT_ENABLE, &regVal);
    if(UDMA_SOK == retVal)
    {
        uint32_t fieldVal;

        if(bEnable == TRUE)
        {
            fieldVal = 1U;
        }
        else
        {
            fieldVal = 0U;
        }
        CSL_FINS(regVal, PSILCFG_REG_RT_ENABLE_ENABLE, fieldVal);
        /* Enable/disable realtime thread */
        retVal = Udma_psilcfgWrite(
                     drvHandle, threadId, CSL_PSILCFG_REG_RT_ENABLE, regVal);
    }

    return (retVal);
}

static int32_t Udma_psilcfgSetEnable(Udma_DrvHandle drvHandle,
                                     uint32_t threadId,
                                     uint32_t bEnable)
{
    uint32_t    regVal;
    int32_t     retVal;

    retVal = Udma_psilcfgRead(
                 drvHandle, threadId, CSL_PSILCFG_REG_ENABLE, &regVal);
    if(UDMA_SOK == retVal)
    {
        uint32_t fieldVal;

        if(bEnable == TRUE)
        {
            fieldVal = 1U;
        }
        else
        {
            fieldVal = 0U;
        }
        CSL_FINS( regVal, PSILCFG_REG_ENABLE_ENABLE, fieldVal);
        /* Enable/disable thread */
        retVal = Udma_psilcfgWrite(
                     drvHandle, threadId, CSL_PSILCFG_REG_ENABLE, regVal);
    }

    return (retVal);
}

static int32_t Udma_psilcfgWrite(Udma_DrvHandle drvHandle,
                                 uint32_t threadId,
                                 uint32_t regId,
                                 uint32_t data)
{
    int32_t                             retVal;
    struct tisci_msg_rm_psil_write_req  rmPsilWrReq;

    rmPsilWrReq.valid_params = 0xFFFFU;         /* Not used */
    rmPsilWrReq.nav_id       = drvHandle->devIdPsil;
    rmPsilWrReq.thread       = (uint16_t)threadId;
    rmPsilWrReq.taddr        = (uint16_t)regId;
    rmPsilWrReq.data         = data;
    retVal = Sciclient_rmPsilWrite(&rmPsilWrReq, UDMA_SCICLIENT_TIMEOUT);
    if(CSL_PASS != retVal)
    {
        Udma_printf(drvHandle, "[Error] RM PSIL write failed!!!\n");
    }

    return (retVal);
}

static int32_t Udma_psilcfgRead(Udma_DrvHandle drvHandle,
                                uint32_t threadId,
                                uint32_t regId,
                                uint32_t *pData)
{
    int32_t                             retVal;
    struct tisci_msg_rm_psil_read_req   rmPsilRdReq;
    struct tisci_msg_rm_psil_read_resp  rmPsilRdResp;

    rmPsilRdReq.valid_params = 0xFFFFU;         /* Not used */
    rmPsilRdReq.nav_id       = drvHandle->devIdPsil;
    rmPsilRdReq.thread       = (uint16_t)threadId;
    rmPsilRdReq.taddr        = (uint16_t)regId;
    retVal = Sciclient_rmPsilRead(&rmPsilRdReq, &rmPsilRdResp, UDMA_SCICLIENT_TIMEOUT);
    if(CSL_PASS != retVal)
    {
        Udma_printf(drvHandle, "[Error] RM PSIL read failed!!!\n");
    }
    else
    {
        *pData = rmPsilRdResp.data;
    }

    return (retVal);
}
