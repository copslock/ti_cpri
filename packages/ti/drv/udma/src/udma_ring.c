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
 *  \file udma_ring.c
 *
 *  \brief File containing the UDMA driver ring related APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/udma/src/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Max number of door bell ring that can be performed at one go */
#define UDMA_RING_MAX_DB_RING_CNT       (127U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static inline int32_t Udma_ringProxyQueueRaw(Udma_RingHandle ringHandle,
                                             Udma_DrvHandle drvHandle,
                                             uint64_t phyDescMem);
static int32_t Udma_ringProxyDequeueRaw(Udma_RingHandle ringHandle,
                                        Udma_DrvHandle drvHandle,
                                        uint64_t *phyDescMem);
static int32_t Udma_ringCheckParams(Udma_DrvHandle drvHandle,
                             const Udma_RingPrms *ringPrms);
#if (UDMA_APPLY_RING_WORKAROUND == 1)
static int32_t Udma_ringReset(Udma_DrvHandle drvHandle,
                              Udma_RingHandle ringHandle);
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_ringAlloc(Udma_DrvHandle drvHandle,
                       Udma_RingHandle ringHandle,
                       uint16_t ringNum,
                       const Udma_RingPrms *ringPrms)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            allocDone = (uint32_t) FALSE;
    CSL_RingAccRingCfg *ringCfg;
    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    /* Error check */
    if((NULL_PTR == drvHandle) || (NULL_PTR == ringHandle) || (NULL_PTR == ringPrms))
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
        retVal = Udma_ringCheckParams(drvHandle, ringPrms);
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_RING_ANY == ringNum)
        {
            /* Alloc free ring */
            ringHandle->ringNum = Udma_rmAllocFreeRing(drvHandle);
            if(UDMA_RING_INVALID == ringHandle->ringNum)
            {
                retVal = UDMA_EALLOC;
            }
            else
            {
                allocDone = (uint32_t) TRUE;
            }
        }
        else
        {
            if(ringNum >= drvHandle->maxRings)
            {
                Udma_printf(drvHandle, "[Error] Out of range ring index!!!\n");
                retVal = UDMA_EINVALID_PARAMS;
            }
            else
            {
                ringHandle->ringNum = ringNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        ringCfg = &ringHandle->cfg;
        ringCfg->virtBase    = (void *) ringPrms->ringMem;
        ringCfg->physBase    =
            Udma_virtToPhyFxn(ringPrms->ringMem, drvHandle, (Udma_ChHandle) NULL_PTR);
        ringCfg->mode        = ringPrms->mode;
        ringCfg->elCnt       = ringPrms->elemCnt;
        /* CSL expects ring size in bytes */
        ringCfg->elSz        = ((uint32_t) 1U << (ringPrms->elemSize + 2U));
        ringCfg->evtNum      = UDMA_EVENT_INVALID;
        ringCfg->credSecure  = 0U;
        ringCfg->credPriv    = 0U;
        ringCfg->credPrivId  = CSL_RINGACC_CRED_PASSTHRU;
        ringCfg->credVirtId  = CSL_RINGACC_CRED_PASSTHRU;
        CSL_ringaccInitRingObj(ringHandle->ringNum, ringCfg);

        ringHandle->drvHandle = drvHandle;
        ringHandle->proxyAddr =
            CSL_proxyGetDataAddr(
                &drvHandle->proxyCfg,
                drvHandle->proxyTargetNumRing,
                drvHandle->initPrms.rmInitPrms.proxyThreadNum,
                ringCfg->elSz);
        Udma_assert(drvHandle, drvHandle->raRegs.pCfgRegs != NULL_PTR);
        Udma_assert(drvHandle, drvHandle->raRegs.pRtRegs != NULL_PTR);
        Udma_assert(drvHandle, ringHandle->ringNum < 1024U);
        ringHandle->pCfgRegs =
            &drvHandle->raRegs.pCfgRegs->RING[ringHandle->ringNum];
        ringHandle->pRtRegs         =
            &drvHandle->raRegs.pRtRegs->RINGRT[ringHandle->ringNum];

#if (UDMA_APPLY_RING_WORKAROUND == 1)
        /* Perform ring reset */
        retVal = Udma_ringReset(drvHandle, ringHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] Ring reset failed!!!\n");
        }
#endif
    }

    if(UDMA_SOK == retVal)
    {
        /* Configure ring */
        rmRingReq.valid_params  = TISCI_MSG_VALUE_RM_RING_ADDR_LO_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ADDR_HI_VALID |
                                  TISCI_MSG_VALUE_RM_RING_COUNT_VALID |
                                  TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                  TISCI_MSG_VALUE_RM_RING_SIZE_VALID |
                                  TISCI_MSG_VALUE_RM_RING_ORDER_ID_VALID;
        rmRingReq.nav_id        = drvHandle->devIdRing;
        rmRingReq.index         = ringHandle->ringNum;
        rmRingReq.addr_lo       = (uint32_t)ringCfg->physBase;
        rmRingReq.addr_hi       = (uint32_t)(ringCfg->physBase >> 32UL);
        rmRingReq.count         = ringPrms->elemCnt;
        rmRingReq.mode          = ringPrms->mode;
        rmRingReq.size          = ringPrms->elemSize;
        rmRingReq.order_id      = ringPrms->orderId;
        retVal = Sciclient_rmRingCfg(
                     &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] Ring config failed!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        ringHandle->ringInitDone = UDMA_INIT_DONE;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            Udma_rmFreeFreeRing(ringHandle->ringNum, drvHandle);
        }
    }

    return (retVal);
}

int32_t Udma_ringFree(Udma_RingHandle ringHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if(NULL_PTR == ringHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(ringHandle->ringInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free-up event resources */
        Udma_assert(drvHandle, ringHandle->ringNum != UDMA_RING_INVALID);
        Udma_rmFreeFreeRing(ringHandle->ringNum, drvHandle);
        ringHandle->ringNum         = UDMA_RING_INVALID;
        ringHandle->ringInitDone    = UDMA_DEINIT_DONE;
        ringHandle->pCfgRegs        = (volatile CSL_ringacc_cfgRegs_RING *) NULL_PTR;
        ringHandle->pRtRegs         = (volatile CSL_ringacc_rtRegs_RINGRT *) NULL_PTR;
        ringHandle->drvHandle       = (Udma_DrvHandle) NULL_PTR;
    }

    return (retVal);
}

int32_t Udma_ringAttach(Udma_DrvHandle drvHandle,
                        Udma_RingHandle ringHandle,
                        uint16_t ringNum)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            addrHi, addrLo, elemSize;
    CSL_RingAccRingCfg *ringCfg;

    /* Error check */
    if((NULL_PTR == drvHandle) || (NULL_PTR == ringHandle))
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
        if(ringNum >= drvHandle->maxRings)
        {
            Udma_printf(drvHandle, "[Error] Out of range ring index!!!\n");
            retVal = UDMA_EINVALID_PARAMS;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Configure ring object */
        ringHandle->drvHandle = drvHandle;
        ringHandle->ringNum = ringNum;
        Udma_assert(drvHandle, drvHandle->raRegs.pCfgRegs != NULL_PTR);
        Udma_assert(drvHandle, drvHandle->raRegs.pRtRegs != NULL_PTR);
        Udma_assert(drvHandle, ringHandle->ringNum < 1024U);
        ringHandle->pCfgRegs =
            &drvHandle->raRegs.pCfgRegs->RING[ringHandle->ringNum];
        ringHandle->pRtRegs         =
            &drvHandle->raRegs.pRtRegs->RINGRT[ringHandle->ringNum];

        /* Init CSL ring object */
        ringCfg = &ringHandle->cfg;
        addrHi = CSL_REG32_FEXT(&ringHandle->pCfgRegs->BA_HI, RINGACC_CFG_RING_BA_HI_ADDR_HI);
        addrLo = CSL_REG32_FEXT(&ringHandle->pCfgRegs->BA_LO, RINGACC_CFG_RING_BA_LO_ADDR_LO);
        ringCfg->physBase    = (uint64_t)((((uint64_t) addrHi) << 32UL) |
                                           ((uint64_t) addrLo));
        ringCfg->virtBase    = Udma_phyToVirtFxn(ringCfg->physBase, drvHandle, (Udma_ChHandle) NULL_PTR);
        ringCfg->mode        = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_QMODE);
        ringCfg->elCnt       = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELCNT);
        elemSize             = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELSIZE);
        /* CSL expects ring size in bytes */
        ringCfg->elSz        = ((uint32_t) 1U << (elemSize + 2U));
        ringCfg->evtNum      = UDMA_EVENT_INVALID;
        ringCfg->credSecure  = 0U;
        ringCfg->credPriv    = 0U;
        ringCfg->credPrivId  = CSL_RINGACC_CRED_PASSTHRU;
        ringCfg->credVirtId  = CSL_RINGACC_CRED_PASSTHRU;
        CSL_ringaccInitRingObj(ringHandle->ringNum, ringCfg);

        ringHandle->proxyAddr =
            CSL_proxyGetDataAddr(
                &drvHandle->proxyCfg,
                drvHandle->proxyTargetNumRing,
                drvHandle->initPrms.rmInitPrms.proxyThreadNum,
                ringCfg->elSz);

        ringHandle->ringInitDone = UDMA_INIT_DONE;
    }

    return (retVal);
}

int32_t Udma_ringDetach(Udma_RingHandle ringHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if(NULL_PTR == ringHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(ringHandle->ringInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Clear handle object */
        Udma_assert(drvHandle, ringHandle->ringNum != UDMA_RING_INVALID);
        ringHandle->ringInitDone    = UDMA_DEINIT_DONE;
        ringHandle->pCfgRegs        = (volatile CSL_ringacc_cfgRegs_RING *) NULL_PTR;
        ringHandle->pRtRegs         = (volatile CSL_ringacc_rtRegs_RINGRT *) NULL_PTR;
        ringHandle->drvHandle       = (Udma_DrvHandle) NULL_PTR;
    }

    return (retVal);
}

int32_t Udma_ringQueueRaw(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    int32_t         retVal = UDMA_SOK;
    uintptr_t       cookie;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == ringHandle) ||
       (ringHandle->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandle->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.disableAllIntr != (Udma_OsalDisableAllIntrFxn) NULL_PTR);
        cookie = drvHandle->initPrms.osalPrms.disableAllIntr();

        if(TISCI_MSG_VALUE_RM_RING_MODE_RING == ringHandle->cfg.mode)
        {
            /* Use direct memory access for RING mode */
            retVal = CSL_ringaccPush64(
                &ringHandle->drvHandle->raRegs,
                &ringHandle->cfg,
                (uint64_t) phyDescMem,
                &Udma_ringaccMemOps);
        }
        else
        {
            /* Use proxy for other modes */
            retVal = Udma_ringProxyQueueRaw(ringHandle, drvHandle, phyDescMem);
        }

        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.restoreAllIntr != (Udma_OsalRestoreAllIntrFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.restoreAllIntr(cookie);
    }

    return (retVal);
}

int32_t Udma_ringDequeueRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK, cslRetVal;
    uintptr_t       cookie;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == ringHandle) ||
       (ringHandle->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandle->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.disableAllIntr != (Udma_OsalDisableAllIntrFxn) NULL_PTR);
        cookie = drvHandle->initPrms.osalPrms.disableAllIntr();

        if(TISCI_MSG_VALUE_RM_RING_MODE_RING == ringHandle->cfg.mode)
        {
            /* Use direct memory access for RING mode */
            cslRetVal = CSL_ringaccPop64(
                &ringHandle->drvHandle->raRegs,
                &ringHandle->cfg,
                phyDescMem,
                &Udma_ringaccMemOps);
            if(0 != cslRetVal)
            {
                retVal = UDMA_ETIMEOUT;
            }
        }
        else
        {
            /* Use proxy for other modes */
            retVal = Udma_ringProxyDequeueRaw(ringHandle, drvHandle, phyDescMem);
        }

        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.restoreAllIntr != (Udma_OsalRestoreAllIntrFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.restoreAllIntr(cookie);
    }

    return (retVal);
}

int32_t Udma_ringFlushRaw(Udma_RingHandle ringHandle, uint64_t *phyDescMem)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == ringHandle) ||
       (ringHandle->ringInitDone != UDMA_INIT_DONE) ||
       (ringHandle->ringNum == UDMA_RING_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = ringHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Same as proxy dequeue API as proxy can be used in all modes of ring */
        retVal = Udma_ringProxyDequeueRaw(ringHandle, drvHandle, phyDescMem);
    }

    return (retVal);
}

void Udma_ringPrime(Udma_RingHandle ringHandle, uint64_t phyDescMem)
{
    volatile uint64_t        *ringPtr;
    CSL_RingAccRingCfg       *pRing;
    uintptr_t                 tempPtr;

    pRing = &ringHandle->cfg;
    tempPtr = (uintptr_t)(pRing->rwIdx * pRing->elSz) +
              (uintptr_t)pRing->virtBase;
    ringPtr = (volatile uint64_t *)(tempPtr);
    *ringPtr = phyDescMem;

    /* Book keeping */
    pRing->waiting++;
    pRing->rwIdx++;
    if(pRing->rwIdx >= pRing->elCnt)
    {
        pRing->rwIdx = 0U;
    }
    pRing->occ++;

    return;
}

void Udma_ringSetDoorBell(Udma_RingHandle ringHandle, int32_t count)
{
    uint32_t    regVal;
    int32_t     dbRingCnt, thisDbRingCnt;
    CSL_RingAccRingCfg       *pRing;

    pRing = &ringHandle->cfg;
    dbRingCnt = count;
    while(dbRingCnt != 0)
    {
        if(dbRingCnt < UDMA_RING_MAX_DB_RING_CNT)
        {
            thisDbRingCnt = dbRingCnt;
            regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
        }
        else
        {
            thisDbRingCnt = UDMA_RING_MAX_DB_RING_CNT;
            regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
        }
        CSL_REG32_WR(&ringHandle->pRtRegs->DB, regVal);
        pRing->waiting -= thisDbRingCnt;
        dbRingCnt -= thisDbRingCnt;
    }

    return;
}

uint16_t Udma_ringGetNum(Udma_RingHandle ringHandle)
{
    uint16_t        ringNum = UDMA_RING_INVALID;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringNum = ringHandle->ringNum;
    }

    return (ringNum);
}

void *Udma_ringGetMemPtr(Udma_RingHandle ringHandle)
{
    void   *ringMem = NULL_PTR;

    if((NULL_PTR != ringHandle) && (UDMA_INIT_DONE == ringHandle->ringInitDone))
    {
        ringMem = ringHandle->cfg.virtBase;
    }

    return (ringMem);
}

int32_t Udma_ringMonAlloc(Udma_DrvHandle drvHandle,
                          Udma_RingMonHandle monHandle,
                          uint16_t ringMonNum)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    allocDone = (uint32_t) FALSE;

    /* Error check */
    if((NULL_PTR == monHandle) || (NULL_PTR == drvHandle))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_RING_MON_ANY == ringMonNum)
        {
            /* Alloc free ring MONITOR */
            monHandle->ringMonNum = Udma_rmAllocRingMon(drvHandle);
            if(UDMA_RING_MON_INVALID == monHandle->ringMonNum)
            {
                retVal = UDMA_EALLOC;
            }
            else
            {
                allocDone = (uint32_t) TRUE;
            }
        }
        else
        {
            if(ringMonNum >= drvHandle->maxRingMon)
            {
                Udma_printf(drvHandle, "[Error] Out of range ring monitor index!!!\n");
                retVal = UDMA_EINVALID_PARAMS;
            }
            else
            {
                monHandle->ringMonNum = ringMonNum;
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        monHandle->drvHandle = drvHandle;
        Udma_assert(drvHandle, drvHandle->raRegs.pMonRegs != NULL_PTR);
        monHandle->pMonRegs =
            &drvHandle->raRegs.pMonRegs->MON[monHandle->ringMonNum];
        monHandle->ringMonInitDone = UDMA_INIT_DONE;
    }
    else
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            Udma_rmFreeRingMon(monHandle->ringMonNum, drvHandle);
        }
    }

    return (retVal);
}

int32_t Udma_ringMonFree(Udma_RingMonHandle monHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == monHandle) ||
       (monHandle->ringMonInitDone != UDMA_INIT_DONE) ||
       (monHandle->ringMonNum == UDMA_RING_MON_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = monHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Free-up event resources */
        Udma_assert(drvHandle, monHandle->ringMonNum != UDMA_RING_MON_INVALID);
        Udma_rmFreeRingMon(monHandle->ringMonNum, drvHandle);
        monHandle->drvHandle        = (Udma_DrvHandle) NULL_PTR;
        monHandle->ringMonNum       = UDMA_RING_MON_INVALID;
        monHandle->pMonRegs         = (volatile CSL_ringacc_monitorRegs_mon *) NULL_PTR;
        monHandle->ringMonInitDone  = UDMA_DEINIT_DONE;
    }

    return (retVal);
}

int32_t Udma_ringMonConfig(Udma_RingMonHandle monHandle,
                           const Udma_RingMonPrms *monPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandle      drvHandle;
    struct tisci_msg_rm_ring_mon_cfg_req    rmRingMonReq;
    struct tisci_msg_rm_ring_mon_cfg_resp   rmRingMonResp;

    /* Error check */
    if((NULL_PTR == monHandle) ||
       (NULL_PTR == monPrms) ||
       (monHandle->ringMonInitDone != UDMA_INIT_DONE) ||
       (monHandle->ringMonNum == UDMA_RING_MON_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = monHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        rmRingMonReq.valid_params   = TISCI_MSG_VALUE_RM_MON_SOURCE_VALID |
                                      TISCI_MSG_VALUE_RM_MON_MODE_VALID |
                                      TISCI_MSG_VALUE_RM_MON_QUEUE_VALID |
                                      TISCI_MSG_VALUE_RM_MON_DATA0_VAL_VALID |
                                      TISCI_MSG_VALUE_RM_MON_DATA1_VAL_VALID;
        rmRingMonReq.nav_id         = drvHandle->devIdRing;
        rmRingMonReq.index          = monHandle->ringMonNum;
        rmRingMonReq.source         = monPrms->source;
        rmRingMonReq.mode           = monPrms->mode;
        rmRingMonReq.queue          = monPrms->ringNum;
        rmRingMonReq.data0_val      = monPrms->data0;
        rmRingMonReq.data1_val      = monPrms->data1;
        retVal = Sciclient_rmRingMonCfg(
                     &rmRingMonReq, &rmRingMonResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] Ring monitor config failed!!!\n");
        }
    }

    return (retVal);
}

int32_t Udma_ringMonGetData(Udma_RingMonHandle monHandle,
                            Udma_RingMonData *monData)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if((NULL_PTR == monHandle) ||
       (NULL_PTR == monData) ||
       (monHandle->ringMonInitDone != UDMA_INIT_DONE) ||
       (monHandle->ringMonNum == UDMA_RING_MON_INVALID))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = monHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        Udma_assert(drvHandle, monHandle->pMonRegs != NULL_PTR);
        monData->data0 = CSL_REG32_RD(&monHandle->pMonRegs->DATA0);
        monData->data1 = CSL_REG32_RD(&monHandle->pMonRegs->DATA1);
    }

    return (retVal);
}

uint16_t Udma_ringMonGetNum(Udma_RingMonHandle monHandle)
{
    uint16_t        ringMonNum = UDMA_RING_MON_INVALID;

    if((NULL_PTR != monHandle) && (UDMA_INIT_DONE == monHandle->ringMonInitDone))
    {
        ringMonNum = monHandle->ringMonNum;
    }

    return (ringMonNum);
}

void UdmaRingPrms_init(Udma_RingPrms *ringPrms)
{
    if(NULL_PTR != ringPrms)
    {
        ringPrms->ringMem       = NULL_PTR;
        ringPrms->ringMemSize   = UDMA_RING_SIZE_CHECK_SKIP;
        ringPrms->mode          = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        ringPrms->elemCnt       = 0U;
        ringPrms->elemSize      = UDMA_RING_ES_8BYTES;
        ringPrms->orderId       = UDMA_DEFAULT_RING_ORDER_ID;
    }

    return;
}

void UdmaRingMonPrms_init(Udma_RingMonPrms *monPrms)
{
    if(NULL_PTR != monPrms)
    {
        monPrms->source     = TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT;
        monPrms->mode       = TISCI_MSG_VALUE_RM_MON_MODE_DISABLED;
        monPrms->ringNum    = UDMA_RING_INVALID;
        monPrms->data0      = 0U;
        monPrms->data1      = 0U;
    }

    return;
}

static inline int32_t Udma_ringProxyQueueRaw(Udma_RingHandle ringHandle,
                                             Udma_DrvHandle drvHandle,
                                             uint64_t phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            ringHwOcc;
    CSL_ProxyThreadCfg  threadCfg;

    /* Get ring occupancy. We should write to FIFO (through proxy) only
     * when there is room available. Otherwise ring will overflow */
    ringHwOcc = CSL_ringaccGetRingHwOcc(
                    &ringHandle->drvHandle->raRegs, ringHandle->ringNum);
    if(ringHwOcc >= ringHandle->cfg.elCnt)
    {
        /* Ring full */
        retVal = UDMA_EFAIL;
    }
    else
    {
        threadCfg.mode      = CSL_PROXY_QUEUE_ACCESS_MODE_TAIL;
        threadCfg.elSz      = (uint32_t)sizeof(uint64_t);
        threadCfg.queueNum  = ringHandle->ringNum;
        threadCfg.errEvtNum = UDMA_EVENT_INVALID;
        retVal = CSL_proxyCfgThread(
                     &drvHandle->proxyCfg,
                     drvHandle->proxyTargetNumRing,
                     drvHandle->initPrms.rmInitPrms.proxyThreadNum,
                     &threadCfg);
        if(UDMA_SOK == retVal)
        {
            Udma_proxyWrite64(ringHandle->proxyAddr, phyDescMem);
        }
    }

    return (retVal);
}

static int32_t Udma_ringProxyDequeueRaw(Udma_RingHandle ringHandle,
                                        Udma_DrvHandle drvHandle,
                                        uint64_t *phyDescMem)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            ringHwOcc;
    CSL_ProxyThreadCfg  threadCfg;

    /* Get ring occupancy. We should read from FIFO (through proxy) only
     * when something is available. Otherwise ring will underflow */
    ringHwOcc = CSL_ringaccGetRingHwOcc(
                    &ringHandle->drvHandle->raRegs, ringHandle->ringNum);
    if(0U == ringHwOcc)
    {
        /* Nothing to flush */
        retVal = UDMA_ETIMEOUT;
    }
    else
    {
        threadCfg.mode      = CSL_PROXY_QUEUE_ACCESS_MODE_HEAD;
        threadCfg.elSz      = (uint32_t)sizeof(uint64_t);
        threadCfg.queueNum  = ringHandle->ringNum;
        threadCfg.errEvtNum = UDMA_EVENT_INVALID;
        retVal = CSL_proxyCfgThread(
                     &drvHandle->proxyCfg,
                     drvHandle->proxyTargetNumRing,
                     drvHandle->initPrms.rmInitPrms.proxyThreadNum,
                     &threadCfg);
        if(UDMA_SOK == retVal)
        {
            Udma_proxyRead64(ringHandle->proxyAddr, phyDescMem);
        }
    }

    return (retVal);
}

static int32_t Udma_ringCheckParams(Udma_DrvHandle drvHandle,
                                    const Udma_RingPrms *ringPrms)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    ringMemSize;

    Udma_assert(drvHandle, ringPrms != NULL_PTR);

    if(NULL_PTR == ringPrms->ringMem)
    {
        retVal = UDMA_EINVALID_PARAMS;
        Udma_printf(drvHandle, "[Error] Ring memory should not be NULL_PTR!!!\n");
    }
    else
    {
        /* Check 128 byte alignment */
        if(((uintptr_t)ringPrms->ringMem & (UDMA_CACHELINE_ALIGNMENT - 1U)) != 0U)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle, "[Error] Ring memory not aligned!!!\n");
        }
    }

    if(0U == ringPrms->elemCnt)
    {
        retVal = UDMA_EINVALID_PARAMS;
        Udma_printf(drvHandle,
            "[Error] Ring element count should not be zero!!!\n");
    }

    if(UDMA_RING_SIZE_CHECK_SKIP != ringPrms->ringMemSize)
    {
        /* Get ring memory size */
        ringMemSize = UdmaUtils_getRingMemSize(
                          ringPrms->mode,
                          ringPrms->elemCnt,
                          ringPrms->elemSize);
        if(ringPrms->ringMemSize < ringMemSize)
        {
            retVal = UDMA_EALLOC;
            Udma_printf(drvHandle, "[Error] Ring memory not sufficient!!!\n");
        }
    }

    if (UDMA_RING_ORDERID_MAX < ringPrms->orderId)
    {
        retVal = UDMA_EINVALID_PARAMS;
        Udma_printf(drvHandle, "[Error] Ring orderId out of range (%u)!!!\n",
                    ringPrms->orderId);
    }

    return (retVal);
}

#if (UDMA_APPLY_RING_WORKAROUND == 1)
static int32_t Udma_ringReset(Udma_DrvHandle drvHandle,
                              Udma_RingHandle ringHandle)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    regVal;
    uint32_t    ringOcc, mode, elemSize;
    uint32_t    applyWorkaround = 0U, freeRingStart;
    uint32_t    dbRingCnt, thisDbRingCnt;
    struct tisci_msg_rm_ring_cfg_req    rmRingReq;
    struct tisci_msg_rm_ring_cfg_resp   rmRingResp;

    Udma_assert(drvHandle, drvHandle->raRegs.pGlbRegs != NULL_PTR);
    Udma_assert(drvHandle, ringHandle->pCfgRegs != NULL_PTR);
    Udma_assert(drvHandle, ringHandle->pRtRegs != NULL_PTR);

    /*-------------------------------------------------------------------------
     *  ringacc revision 1.0.19 and earlier has an issue where the UDMAP ring
     *  state (ring occupancy) will be out of sync with the ringacc ring state
     *  when a ring is reset.
     *
     *  Perform the software workaround to correct this.
     *-----------------------------------------------------------------------*/
    regVal = CSL_REG32_RD(&drvHandle->raRegs.pGlbRegs->REVISION);
    if(!((CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVMAJ) > 1U) ||
         (CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVMIN) > 0U) ||
         (CSL_FEXT(regVal, RINGACC_GCFG_REVISION_REVRTL) > 19U)))
    {
        /* Workaround required only for FQ ring */
        freeRingStart = drvHandle->udmapRegs.txChanCnt +
                        drvHandle->udmapRegs.txExtUtcChanCnt +
                        drvHandle->udmapRegs.rxChanCnt;
        if(ringHandle->ringNum < freeRingStart)
        {
            applyWorkaround = 1U;
        }
    }

    if(1U == applyWorkaround)
    {
        /* Call SCICLIENT with dummy cfg to get read access to the ring RT
         * By default this is is firewalled even for read access */
        rmRingReq.valid_params  = 0U;
        rmRingReq.nav_id        = drvHandle->devIdRing;
        rmRingReq.index         = ringHandle->ringNum;
        /* Not used */
        rmRingReq.mode          = 0U;
        rmRingReq.size          = 0U;
        rmRingReq.addr_lo       = 0U;
        rmRingReq.addr_hi       = 0U;
        rmRingReq.count         = 0U;
        rmRingReq.order_id      = UDMA_DEFAULT_RING_ORDER_ID;
        retVal = Sciclient_rmRingCfg(
                     &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle,
                "[Error] Ring config for read access failed!!!\n");
        }
    }

    if((1U == applyWorkaround) && (UDMA_SOK == retVal))
    {
        /*-------------------------------------------------------------------------
         *  1. Read the ring occupancy.
         *
         *  In the case of a credentials mode or qm mode ring, each ring write
         *  results in the ring occupancy increasing by 2 elements (one entry for
         *  the credentials, one entry for the data). However, the internal UDMAP
         *  ring state occupancy counter only records the number of writes, so we
         *  divide the ring occupancy by 2 to get back to the number of doorbell
         *  rings needed.
         *
         *  If the ring occupancy is not 0, then we need to execute the workaround.
         *-----------------------------------------------------------------------*/
        ringOcc = CSL_REG32_FEXT(&ringHandle->pRtRegs->OCC, RINGACC_RT_RINGRT_OCC_CNT);
        mode = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_QMODE);
        elemSize = CSL_REG32_FEXT(&ringHandle->pCfgRegs->SIZE, RINGACC_CFG_RING_SIZE_ELSIZE);
        if((CSL_RINGACC_RING_MODE_CREDENTIALS == mode) ||
           (CSL_RINGACC_RING_MODE_QM == mode))
        {
            ringOcc >>= 1U;
        }
        if(ringOcc != 0U)
        {
            /*---------------------------------------------------------------------
             *  2. Reset the ring by writing to size register
             *  3. At the same time setup the ring in ring/doorbell mode
             *-------------------------------------------------------------------*/
            rmRingReq.valid_params  = TISCI_MSG_VALUE_RM_RING_MODE_VALID |
                                      TISCI_MSG_VALUE_RM_RING_SIZE_VALID;
            rmRingReq.nav_id        = drvHandle->devIdRing;
            rmRingReq.index         = ringHandle->ringNum;
            rmRingReq.mode          = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            rmRingReq.size          = elemSize;
            /* Not used */
            rmRingReq.addr_lo       = 0U;
            rmRingReq.addr_hi       = 0U;
            rmRingReq.count         = 0U;
            rmRingReq.order_id      = UDMA_DEFAULT_RING_ORDER_ID;
            retVal = Sciclient_rmRingCfg(
                         &rmRingReq, &rmRingResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle, "[Error] Ring reset failed!!!\n");
            }
            else
            {
                /*--------------------------------------------------------------
                 *  4. Ring the doorbell 2**22 - ringOcc times. This will wrap
                 *     the internal UDMAP ring state occupancy counter
                 *     (which is 21-bits wide) to 0.
                 *------------------------------------------------------------*/
                dbRingCnt = ((uint32_t)1U << 22) - ringOcc;
                thisDbRingCnt = UDMA_RING_MAX_DB_RING_CNT;
                regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
                while(dbRingCnt != 0U)
                {
                    /*----------------------------------------------------------
                     *  Ring the doorbell with the maximum count each iteration
                     *  if possible to minimize the total # of writes
                     *--------------------------------------------------------*/
                    if(dbRingCnt < UDMA_RING_MAX_DB_RING_CNT)
                    {
                        thisDbRingCnt = dbRingCnt;
                        regVal = CSL_FMK(RINGACC_RT_RINGRT_DB_CNT, thisDbRingCnt);
                    }
                    CSL_REG32_WR(&ringHandle->pRtRegs->DB, regVal);
                    dbRingCnt -= thisDbRingCnt;
                }

                /*--------------------------------------------------------------
                 *  5. Restore the original ring mode (if not ring mode)
                 *------------------------------------------------------------*/
                /* Not done as we will reconfigure the ring after reset step */
            }
        }
    }

    return (retVal);
}
#endif
