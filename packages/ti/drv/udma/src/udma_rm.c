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
 *  \file udma_rm.c
 *
 *  \brief File containing the UDMA driver Resource Manager (RM)
 *  abstraction functions.
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

static int32_t Udma_rmCheckResLeak(Udma_DrvHandle drvHandle,
                                   const uint32_t *allocFlag,
                                   uint32_t numRes,
                                   uint32_t arrSize);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_rmInit(Udma_DrvHandle drvHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;
#if (UDMA_NUM_UTC_INSTANCE > 0)
    uint32_t            utcId;
#endif

    /* Check if the default config has any overlap */
    if(FALSE == drvHandle->initPrms.skipRmOverlapCheck)
    {
        retVal = Udma_rmCheckDefaultCfg();
    }

    if(UDMA_SOK == retVal)
    {
        /* Mark all resources as free */
        for(i = 0U; i < rmInitPrms->numBlkCopyCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->blkCopyChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numBlkCopyHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->blkCopyHcChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numBlkCopyUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->blkCopyUhcChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numTxCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->txChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numRxCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->rxChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numTxHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->txHcChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numRxHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->rxHcChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numTxUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->txUhcChFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numRxUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->rxUhcChFlag[offset] |= bitMask;
        }
#if (UDMA_NUM_UTC_INSTANCE > 0)
        for(utcId = 0U; utcId < UDMA_NUM_UTC_INSTANCE; utcId++)
        {
            for(i = 0U; i < rmInitPrms->numUtcCh[utcId]; i++)
            {
                offset = i >> 5U;
                Udma_assert(drvHandle, offset < UDMA_RM_UTC_CH_ARR_SIZE);
                bitPos = i - (offset << 5U);
                bitMask = (uint32_t) 1U << bitPos;
                drvHandle->utcChFlag[utcId][offset] |= bitMask;
            }
        }
#endif
        for(i = 0U; i < rmInitPrms->numFreeRing; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_FREE_RING_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->freeRingFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numFreeFlow; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_FREE_FLOW_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->freeFlowFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numGlobalEvent; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_GLOBAL_EVENT_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->globalEventFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numVintr; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_VINTR_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->vintrFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numIrIntr; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_CORE_INTR_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->coreIntrFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numProxy; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_PROXY_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->proxyFlag[offset] |= bitMask;
        }
        for(i = 0U; i < rmInitPrms->numRingMon; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RING_MON_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            drvHandle->ringMonFlag[offset] |= bitMask;
        }
    }

    return (retVal);
}

int32_t Udma_rmDeinit(Udma_DrvHandle drvHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;
#if (UDMA_NUM_UTC_INSTANCE > 0)
    uint32_t            utcId;
#endif

    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->blkCopyChFlag[0U],
                  rmInitPrms->numBlkCopyCh,
                  UDMA_RM_BLK_COPY_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->blkCopyHcChFlag[0U],
                  rmInitPrms->numBlkCopyHcCh,
                  UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->blkCopyUhcChFlag[0U],
                  rmInitPrms->numBlkCopyUhcCh,
                  UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->txChFlag[0U],
                  rmInitPrms->numTxCh,
                  UDMA_RM_TX_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->txHcChFlag[0U],
                  rmInitPrms->numTxHcCh,
                  UDMA_RM_TX_HC_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->txUhcChFlag[0U],
                  rmInitPrms->numTxUhcCh,
                  UDMA_RM_TX_UHC_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->rxChFlag[0U],
                  rmInitPrms->numRxCh,
                  UDMA_RM_RX_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->rxHcChFlag[0U],
                  rmInitPrms->numRxHcCh,
                  UDMA_RM_RX_HC_CH_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->rxUhcChFlag[0U],
                  rmInitPrms->numRxUhcCh,
                  UDMA_RM_RX_UHC_CH_ARR_SIZE);
#if (UDMA_NUM_UTC_INSTANCE > 0)
    for(utcId = 0U; utcId < UDMA_NUM_UTC_INSTANCE; utcId++)
    {
        retVal += Udma_rmCheckResLeak(
                      drvHandle,
                      &drvHandle->utcChFlag[utcId][0U],
                      rmInitPrms->numUtcCh[utcId],
                      UDMA_RM_UTC_CH_ARR_SIZE);
    }
#endif
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->freeRingFlag[0U],
                  rmInitPrms->numFreeRing,
                  UDMA_RM_FREE_RING_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->freeFlowFlag[0U],
                  rmInitPrms->numFreeFlow,
                  UDMA_RM_FREE_FLOW_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->globalEventFlag[0U],
                  rmInitPrms->numGlobalEvent,
                  UDMA_RM_GLOBAL_EVENT_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->vintrFlag[0U],
                  rmInitPrms->numVintr,
                  UDMA_RM_VINTR_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->coreIntrFlag[0U],
                  rmInitPrms->numIrIntr,
                  UDMA_RM_CORE_INTR_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->proxyFlag[0U],
                  rmInitPrms->numProxy,
                  UDMA_RM_PROXY_ARR_SIZE);
    retVal += Udma_rmCheckResLeak(
                  drvHandle,
                  &drvHandle->ringMonFlag[0U],
                  rmInitPrms->numRingMon,
                  UDMA_RM_RING_MON_ARR_SIZE);

    return (retVal);
}

uint32_t Udma_rmAllocBlkCopyCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from Blk Copy channel pool */
        for(i = 0U; i < rmInitPrms->numBlkCopyCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startBlkCopyCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific Block Copy channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startBlkCopyCh) &&
           (preferredChNum < (rmInitPrms->startBlkCopyCh + rmInitPrms->numBlkCopyCh)))
        {
            i = preferredChNum - rmInitPrms->startBlkCopyCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeBlkCopyCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startBlkCopyCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startBlkCopyCh + rmInitPrms->numBlkCopyCh));
    i = chNum - rmInitPrms->startBlkCopyCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->blkCopyChFlag[offset] & bitMask) == 0U);
    drvHandle->blkCopyChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocBlkCopyHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from Blk Copy high capacity channel pool */
        for(i = 0U; i < rmInitPrms->numBlkCopyHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyHcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startBlkCopyHcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific Block Copy channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startBlkCopyHcCh) &&
           (preferredChNum < (rmInitPrms->startBlkCopyHcCh + rmInitPrms->numBlkCopyHcCh)))
        {
            i = preferredChNum - rmInitPrms->startBlkCopyHcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyHcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeBlkCopyHcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startBlkCopyHcCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startBlkCopyHcCh + rmInitPrms->numBlkCopyHcCh));
    i = chNum - rmInitPrms->startBlkCopyHcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_HC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->blkCopyHcChFlag[offset] & bitMask) == 0U);
    drvHandle->blkCopyHcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocBlkCopyUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from Blk Copy ultra high capacity channel pool */
        for(i = 0U; i < rmInitPrms->numBlkCopyUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyUhcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startBlkCopyUhcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific Block Copy channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startBlkCopyUhcCh) &&
           (preferredChNum < (rmInitPrms->startBlkCopyUhcCh + rmInitPrms->numBlkCopyUhcCh)))
        {
            i = preferredChNum - rmInitPrms->startBlkCopyUhcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->blkCopyUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->blkCopyUhcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeBlkCopyUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startBlkCopyUhcCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startBlkCopyUhcCh + rmInitPrms->numBlkCopyUhcCh));
    i = chNum - rmInitPrms->startBlkCopyUhcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_BLK_COPY_UHC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->blkCopyUhcChFlag[offset] & bitMask) == 0U);
    drvHandle->blkCopyUhcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocTxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from TX channel pool */
        for(i = 0U; i < rmInitPrms->numTxCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startTxCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific TX channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startTxCh) &&
           (preferredChNum < (rmInitPrms->startTxCh + rmInitPrms->numTxCh)))
        {
            i = preferredChNum - rmInitPrms->startTxCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeTxCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startTxCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startTxCh + rmInitPrms->numTxCh));
    i = chNum - rmInitPrms->startTxCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_TX_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->txChFlag[offset] & bitMask) == 0U);
    drvHandle->txChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocRxCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from RX channel pool */
        for(i = 0U; i < rmInitPrms->numRxCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startRxCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific RX channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startRxCh) &&
           (preferredChNum < (rmInitPrms->startRxCh + rmInitPrms->numRxCh)))
        {
            i = preferredChNum - rmInitPrms->startRxCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeRxCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= rmInitPrms->startRxCh);
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startRxCh + rmInitPrms->numRxCh));
    i = chNum - rmInitPrms->startRxCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_RX_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->rxChFlag[offset] & bitMask) == 0U);
    drvHandle->rxChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocTxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from TX HC channel pool */
        for(i = 0U; i < rmInitPrms->numTxHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txHcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startTxHcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific TX HC channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startTxHcCh) &&
           (preferredChNum < (rmInitPrms->startTxHcCh + rmInitPrms->numTxHcCh)))
        {
            i = preferredChNum - rmInitPrms->startTxHcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txHcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeTxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startTxHcCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startTxHcCh + rmInitPrms->numTxHcCh));
    i = chNum - rmInitPrms->startTxHcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_TX_HC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->txHcChFlag[offset] & bitMask) == 0U);
    drvHandle->txHcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocRxHcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from RX HC channel pool */
        for(i = 0U; i < rmInitPrms->numRxHcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxHcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startRxHcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific RX HC channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startRxHcCh) &&
           (preferredChNum < (rmInitPrms->startRxHcCh + rmInitPrms->numRxHcCh)))
        {
            i = preferredChNum - rmInitPrms->startRxHcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_HC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxHcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxHcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeRxHcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= rmInitPrms->startRxHcCh);
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startRxHcCh + rmInitPrms->numRxHcCh));
    i = chNum - rmInitPrms->startRxHcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_RX_HC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->rxHcChFlag[offset] & bitMask) == 0U);
    drvHandle->rxHcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocTxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from TX UHC channel pool */
        for(i = 0U; i < rmInitPrms->numTxUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txUhcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startTxUhcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific TX UHC channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startTxUhcCh) &&
           (preferredChNum < (rmInitPrms->startTxUhcCh + rmInitPrms->numTxUhcCh)))
        {
            i = preferredChNum - rmInitPrms->startTxUhcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_TX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->txUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->txUhcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeTxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= (rmInitPrms->startTxUhcCh));
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startTxUhcCh + rmInitPrms->numTxUhcCh));
    i = chNum - rmInitPrms->startTxUhcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_TX_UHC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->txUhcChFlag[offset] & bitMask) == 0U);
    drvHandle->txUhcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocRxUhcCh(uint32_t preferredChNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from RX UHC channel pool */
        for(i = 0U; i < rmInitPrms->numRxUhcCh; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxUhcChFlag[offset] &= ~bitMask;
                chNum = i + rmInitPrms->startRxUhcCh;  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific RX UHC channel if free */
        /* Array bound check */
        if((preferredChNum >= rmInitPrms->startRxUhcCh) &&
           (preferredChNum < (rmInitPrms->startRxUhcCh + rmInitPrms->numRxUhcCh)))
        {
            i = preferredChNum - rmInitPrms->startRxUhcCh;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_RX_UHC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->rxUhcChFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->rxUhcChFlag[offset] &= ~bitMask;
                chNum = preferredChNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (chNum);
}

void Udma_rmFreeRxUhcCh(uint32_t chNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, chNum >= rmInitPrms->startRxUhcCh);
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startRxUhcCh + rmInitPrms->numRxUhcCh));
    i = chNum - rmInitPrms->startRxUhcCh;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_RX_UHC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->rxUhcChFlag[offset] & bitMask) == 0U);
    drvHandle->rxUhcChFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocExtCh(uint32_t preferredChNum,
                           Udma_DrvHandle drvHandle,
                           const Udma_UtcInstInfo *utcInfo)
{
    uint32_t            chNum = UDMA_DMA_CH_INVALID;
#if (UDMA_NUM_UTC_INSTANCE > 0)
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            utcId;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, utcInfo != NULL_PTR);
    utcId = utcInfo->utcId;
    Udma_assert(drvHandle, utcId <= UDMA_NUM_UTC_INSTANCE);
    Udma_assert(drvHandle,
        rmInitPrms->startUtcCh[utcId] >= utcInfo->startCh);
    Udma_assert(drvHandle,
        rmInitPrms->startUtcCh[utcId] < (utcInfo->startCh + utcInfo->numCh));
    Udma_assert(drvHandle,
        (rmInitPrms->startUtcCh[utcId] + rmInitPrms->numUtcCh[utcId]) <=
            (utcInfo->startCh + utcInfo->numCh));

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_DMA_CH_ANY == preferredChNum)
    {
        /* Search and allocate from specific external channel pool */
        for(i = 0U; i < rmInitPrms->numUtcCh[utcId]; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_UTC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->utcChFlag[utcId][offset] & bitMask) == bitMask)
            {
                drvHandle->utcChFlag[utcId][offset] &= ~bitMask;
                chNum = i + rmInitPrms->startUtcCh[utcId];  /* Add start offset */
                break;
            }
        }
    }
    else
    {
        if(preferredChNum < rmInitPrms->numUtcCh[utcId])
        {
            i = preferredChNum;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_UTC_CH_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->utcChFlag[utcId][offset] & bitMask) == bitMask)
            {
                drvHandle->utcChFlag[utcId][offset] &= ~bitMask;
                chNum = preferredChNum + rmInitPrms->startUtcCh[utcId];
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);
#endif

    return (chNum);
}

void Udma_rmFreeExtCh(uint32_t chNum,
                      Udma_DrvHandle drvHandle,
                      const Udma_UtcInstInfo *utcInfo)
{
#if (UDMA_NUM_UTC_INSTANCE > 0)
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            utcId;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, utcInfo != NULL_PTR);
    utcId = utcInfo->utcId;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);
    Udma_assert(drvHandle, chNum >= rmInitPrms->startUtcCh[utcId]);
    Udma_assert(drvHandle,
        chNum < (rmInitPrms->startUtcCh[utcId] + rmInitPrms->numUtcCh[utcId]));
    i = chNum - rmInitPrms->startUtcCh[utcId];
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_UTC_CH_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle,
        (drvHandle->utcChFlag[utcId][offset] & bitMask) == 0U);
    drvHandle->utcChFlag[utcId][offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);
#endif

    return;
}

uint16_t Udma_rmAllocFreeRing(Udma_DrvHandle drvHandle)
{
    uint16_t            i, offset, ringNum = UDMA_RING_INVALID, temp;
    uint32_t            bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < rmInitPrms->numFreeRing; i++)
    {
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_FREE_RING_ARR_SIZE);
        temp = i - (offset << 5U);
        bitPos = (uint32_t) temp;
        bitMask = (uint32_t) 1U << bitPos;
        if((drvHandle->freeRingFlag[offset] & bitMask) == bitMask)
        {
            drvHandle->freeRingFlag[offset] &= ~bitMask;
            ringNum = (uint16_t)rmInitPrms->startFreeRing;  /* Add start offset */
            ringNum += i;
            ringNum += (uint16_t)(drvHandle->udmapRegs.txChanCnt +
                                  drvHandle->udmapRegs.txExtUtcChanCnt +
                                  drvHandle->udmapRegs.rxChanCnt);
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (ringNum);
}

void Udma_rmFreeFreeRing(uint16_t ringNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;
    uint32_t            freeRingOffset = rmInitPrms->startFreeRing +
                                 drvHandle->udmapRegs.txChanCnt +
                                 drvHandle->udmapRegs.txExtUtcChanCnt +
                                 drvHandle->udmapRegs.rxChanCnt;

    /* Free up only the free ring - ignore FQ rings */
    if(ringNum >= freeRingOffset)
    {
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

        i = ringNum - freeRingOffset;
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_FREE_RING_ARR_SIZE);
        bitPos = i - (offset << 5U);
        bitMask = (uint32_t) 1U << bitPos;
        Udma_assert(drvHandle, (drvHandle->freeRingFlag[offset] & bitMask) == 0U);
        drvHandle->freeRingFlag[offset] |= bitMask;

        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);
    }

    return;
}

uint16_t Udma_rmAllocProxy(Udma_DrvHandle drvHandle)
{
    uint16_t            i, offset, proxyNum = UDMA_PROXY_INVALID, temp;
    uint32_t            bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < rmInitPrms->numProxy; i++)
    {
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_PROXY_ARR_SIZE);
        temp = i - (offset << 5U);
        bitPos = (uint32_t) temp;
        bitMask = (uint32_t) 1U << bitPos;
        if((drvHandle->proxyFlag[offset] & bitMask) == bitMask)
        {
            drvHandle->proxyFlag[offset] &= ~bitMask;
            proxyNum = (uint16_t)rmInitPrms->startProxy;  /* Add start offset */
            proxyNum += i;
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (proxyNum);
}

void Udma_rmFreeProxy(uint16_t proxyNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    i = proxyNum - rmInitPrms->startProxy;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_PROXY_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle, (drvHandle->proxyFlag[offset] & bitMask) == 0U);
    drvHandle->proxyFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocEvent(Udma_DrvHandle drvHandle)
{
    uint32_t            globalEvent = UDMA_EVENT_INVALID;
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < rmInitPrms->numGlobalEvent; i++)
    {
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_GLOBAL_EVENT_ARR_SIZE);
        bitPos = i - (offset << 5U);
        bitMask = (uint32_t) 1U << bitPos;
        if((drvHandle->globalEventFlag[offset] & bitMask) == bitMask)
        {
            drvHandle->globalEventFlag[offset] &= ~bitMask;
            globalEvent = i + rmInitPrms->startGlobalEvent;  /* Add start offset */
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (globalEvent);
}

void Udma_rmFreeEvent(uint32_t globalEvent, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle,
        globalEvent < (rmInitPrms->startGlobalEvent + rmInitPrms->numGlobalEvent));
    Udma_assert(drvHandle, globalEvent >= rmInitPrms->startGlobalEvent);

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    i = globalEvent - rmInitPrms->startGlobalEvent;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_GLOBAL_EVENT_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle, (drvHandle->globalEventFlag[offset] & bitMask) == 0U);
    drvHandle->globalEventFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocVintr(Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            vintrNum = UDMA_EVENT_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < rmInitPrms->numVintr; i++)
    {
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_VINTR_ARR_SIZE);
        bitPos = i - (offset << 5U);
        bitMask = (uint32_t) 1U << bitPos;
        if((drvHandle->vintrFlag[offset] & bitMask) == bitMask)
        {
            drvHandle->vintrFlag[offset] &= ~bitMask;
            vintrNum = i + rmInitPrms->startVintr;  /* Add start offset */
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (vintrNum);
}

void Udma_rmFreeVintr(uint32_t vintrNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle,
        vintrNum < (rmInitPrms->startVintr + rmInitPrms->numVintr));
    Udma_assert(drvHandle, vintrNum >= rmInitPrms->startVintr);

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    i = vintrNum - rmInitPrms->startVintr;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_VINTR_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle, (drvHandle->vintrFlag[offset] & bitMask) == 0U);
    drvHandle->vintrFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocVintrBit(Udma_EventHandle eventHandle)
{
    uint32_t                i;
    uint32_t                vintrBitNum = UDMA_EVENT_INVALID;
    uint64_t                bitMask;
    Udma_EventHandle        masterEventHandle;
    const Udma_EventPrms   *eventPrms;
    Udma_DrvHandle          drvHandle = eventHandle->drvHandle;

    masterEventHandle = eventHandle;
    eventPrms = &eventHandle->eventPrms;
    if(NULL_PTR != eventPrms->masterEventHandle)
    {
        /* Shared event. Get the master handle */
        masterEventHandle = eventPrms->masterEventHandle;
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < UDMA_MAX_EVENTS_PER_VINTR; i++)
    {
        bitMask = ((uint64_t) 1U << i);
        if((masterEventHandle->vintrBitAllocFlag & bitMask) == 0U)
        {
            masterEventHandle->vintrBitAllocFlag |= bitMask;
            vintrBitNum = i;
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (vintrBitNum);
}

void Udma_rmFreeVintrBit(uint32_t vintrBitNum,
                         Udma_DrvHandle drvHandle,
                         Udma_EventHandle eventHandle)
{
    uint64_t                bitMask;
    Udma_EventHandle        masterEventHandle;
    const Udma_EventPrms   *eventPrms;

    masterEventHandle = eventHandle;
    eventPrms = &eventHandle->eventPrms;
    if(NULL_PTR != eventPrms->masterEventHandle)
    {
        /* Shared event. Get the master handle */
        masterEventHandle = eventPrms->masterEventHandle;
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    Udma_assert(drvHandle, vintrBitNum < UDMA_MAX_EVENTS_PER_VINTR);
    bitMask = ((uint64_t) 1U << vintrBitNum);
    Udma_assert(drvHandle,
        (masterEventHandle->vintrBitAllocFlag & bitMask) == bitMask);
    masterEventHandle->vintrBitAllocFlag &= ~bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint32_t Udma_rmAllocCoreIntr(uint32_t preferredCoreIntrNum,
                              Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    uint32_t            coreIntrNum = UDMA_INTR_INVALID;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    if(UDMA_CORE_INTR_ANY == preferredCoreIntrNum)
    {
        /* Search and allocate from pool */
        for(i = 0U; i < rmInitPrms->numIrIntr; i++)
        {
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_CORE_INTR_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->coreIntrFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->coreIntrFlag[offset] &= ~bitMask;
                coreIntrNum = i + rmInitPrms->startIrIntr;    /* Add start offset */
                break;
            }
        }
    }
    else
    {
        /* Allocate specific core interrupt number if free */
        /* Array bound check */
        if((preferredCoreIntrNum >= rmInitPrms->startIrIntr) &&
           (preferredCoreIntrNum < (rmInitPrms->startIrIntr + rmInitPrms->numIrIntr)))
        {
            i = preferredCoreIntrNum - rmInitPrms->startIrIntr;
            offset = i >> 5U;
            Udma_assert(drvHandle, offset < UDMA_RM_CORE_INTR_ARR_SIZE);
            bitPos = i - (offset << 5U);
            bitMask = (uint32_t) 1U << bitPos;
            if((drvHandle->coreIntrFlag[offset] & bitMask) == bitMask)
            {
                drvHandle->coreIntrFlag[offset] &= ~bitMask;
                coreIntrNum = preferredCoreIntrNum;
            }
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (coreIntrNum);
}

void Udma_rmFreeCoreIntr(uint32_t coreIntrNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle,
        coreIntrNum < (rmInitPrms->startIrIntr + rmInitPrms->numIrIntr));
    Udma_assert(drvHandle, coreIntrNum >= rmInitPrms->startIrIntr);

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    i = coreIntrNum - rmInitPrms->startIrIntr;
    offset = i >> 5U;
    Udma_assert(drvHandle, offset < UDMA_RM_CORE_INTR_ARR_SIZE);
    bitPos = i - (offset << 5U);
    bitMask = (uint32_t) 1U << bitPos;
    Udma_assert(drvHandle, (drvHandle->coreIntrFlag[offset] & bitMask) == 0U);
    drvHandle->coreIntrFlag[offset] |= bitMask;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return;
}

uint16_t Udma_rmAllocRingMon(Udma_DrvHandle drvHandle)
{
    uint16_t            i, offset, ringMonNum = UDMA_RING_MON_INVALID, temp;
    uint32_t            bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

    for(i = 0U; i < rmInitPrms->numRingMon; i++)
    {
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_RING_MON_ARR_SIZE);
        temp = i - (offset << 5U);
        bitPos = (uint32_t) temp;
        bitMask = (uint32_t) 1U << bitPos;
        if((drvHandle->ringMonFlag[offset] & bitMask) == bitMask)
        {
            drvHandle->ringMonFlag[offset] &= ~bitMask;
            ringMonNum = (uint16_t)rmInitPrms->startRingMon;  /* Add start offset */
            ringMonNum += i;
            break;
        }
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);

    return (ringMonNum);
}

void Udma_rmFreeRingMon(uint16_t ringMonNum, Udma_DrvHandle drvHandle)
{
    uint32_t            i, offset, bitPos, bitMask;
    Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;
    uint32_t            ringMonOffset = rmInitPrms->startRingMon;

    if(ringMonNum >= ringMonOffset)
    {
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.lockMutex != (Udma_OsalMutexLockFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.lockMutex(drvHandle->rmLock);

        i = ringMonNum - ringMonOffset;
        offset = i >> 5U;
        Udma_assert(drvHandle, offset < UDMA_RM_RING_MON_ARR_SIZE);
        bitPos = i - (offset << 5U);
        bitMask = (uint32_t) 1U << bitPos;
        Udma_assert(drvHandle, (drvHandle->ringMonFlag[offset] & bitMask) == 0U);
        drvHandle->ringMonFlag[offset] |= bitMask;

        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unlockMutex != (Udma_OsalMutexUnlockFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.unlockMutex(drvHandle->rmLock);
    }

    return;
}

static int32_t Udma_rmCheckResLeak(Udma_DrvHandle drvHandle,
                                   const uint32_t *allocFlag,
                                   uint32_t numRes,
                                   uint32_t arrSize)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    i, offset, bitMask;

    offset = 0;
    i = numRes;
    while(i > 0U)
    {
        if(i >= (uint32_t)32U)          /* 32 resource per array element */
        {
            bitMask = (uint32_t) 0xFFFFFFFFU;
            i -= 32U;
        }
        else
        {
            bitMask = ((uint32_t)1U << i) - ((uint32_t)1U);
            i = 0U;
        }

        Udma_assert(drvHandle, offset < arrSize);
        if((allocFlag[offset] & bitMask) != bitMask)
        {
            retVal = UDMA_EFAIL;        /* Some resources not freed */
            break;
        }
        offset++;
    }

    return (retVal);
}
