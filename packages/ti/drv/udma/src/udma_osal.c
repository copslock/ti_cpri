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
 *  \file udma_osal.c
 *
 *  \brief File containing the UDMA driver OSAL abstraction functions.
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

static void *Udma_osalRegisterIntr(Udma_OsalIsrFxn isrFxn,
                                   uint32_t coreIntrNum,
                                   uint32_t intrPriority,
                                   void *arg);
static void Udma_osalUnRegisterIntr(void *hwiHandle);
static void Udma_osalDisableIntr(uint32_t coreIntrNum);

static void *Udma_osalMutexCreate(void);
static void Udma_osalMutexDelete(void *mutexHandle);
static void Udma_osalMutexLock(void *mutexHandle);
static void Udma_osalMutexUnlock(void *mutexHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Variable to store the OSAL cache parameter across driver handle */
static Udma_OsalCachePrms    gUdmaOsalCachePrms =
{
    &CacheP_Inv, &CacheP_wb
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_osalSetCachePrms(const Udma_OsalCachePrms *cachePrms)
{
    int32_t     retVal = UDMA_SOK;

    if((NULL_PTR == cachePrms) ||
       ((Udma_OsalCacheInv) NULL_PTR == cachePrms->cacheInv) ||
       ((Udma_OsalCacheWb) NULL_PTR == cachePrms->cacheWb))
    {
        retVal = UDMA_EBADARGS;
    }
    else
    {
        gUdmaOsalCachePrms.cacheInv  = cachePrms->cacheInv;
        gUdmaOsalCachePrms.cacheWb   = cachePrms->cacheWb;
    }

    return (retVal);
}

void UdmaOsalPrms_init(Udma_OsalPrms *osalPrms)
{
    if(NULL_PTR != osalPrms)
    {
        osalPrms->disableAllIntr    = &HwiP_disable;
        osalPrms->restoreAllIntr    = &HwiP_restore;
        osalPrms->disableIntr       = &Udma_osalDisableIntr;

        osalPrms->createMutex       = &Udma_osalMutexCreate;
        osalPrms->deleteMutex       = &Udma_osalMutexDelete;
        osalPrms->lockMutex         = &Udma_osalMutexLock;
        osalPrms->unlockMutex       = &Udma_osalMutexUnlock;

        osalPrms->registerIntr      = &Udma_osalRegisterIntr;
        osalPrms->unRegisterIntr    = &Udma_osalUnRegisterIntr;
    }

    return;
}

void UdmaOsalCachePrms_init(Udma_OsalCachePrms *cachePrms)
{
    if(NULL_PTR != cachePrms)
    {
        cachePrms->cacheInv = &CacheP_Inv;
        cachePrms->cacheWb  = &CacheP_wb;
    }

    return;
}

void Udma_ringaccMemOps(void *pVirtAddr, uint32_t size, uint32_t opsType)
{
    uint32_t    isCacheCoherent = Udma_isCacheCoherent();

    if(isCacheCoherent != TRUE)
    {
        if(CSL_RINGACC_MEM_OPS_TYPE_WR == opsType)
        {
            gUdmaOsalCachePrms.cacheWb(pVirtAddr, size);
        }

        if(CSL_RINGACC_MEM_OPS_TYPE_RD == opsType)
        {
            gUdmaOsalCachePrms.cacheInv(pVirtAddr, size);
        }
    }

    return;
}

static void *Udma_osalRegisterIntr(Udma_OsalIsrFxn isrFxn,
                                   uint32_t coreIntrNum,
                                   uint32_t intrPriority,
                                   void *arg)
{
    OsalRegisterIntrParams_t    intrPrms;
    OsalInterruptRetCode_e      osalRetVal;
    HwiP_Handle                 hwiHandle = NULL_PTR;

    Osal_RegisterInterrupt_initParams(&intrPrms);

    /* Populate the interrupt parameters */
    intrPrms.corepacConfig.arg              = (uintptr_t) arg;
    intrPrms.corepacConfig.isrRoutine       = isrFxn;
    intrPrms.corepacConfig.priority         = intrPriority;

#if defined (_TMS320C6X)
    /* On C66x, we use Event Combiner to map the interrupt to the CPU Intc.  To
     * do this, OSAL expects that event number holds the interrupt number and we
     * use the macro for interrupt number to specify we wish to use Event
     * Combiner.
     */
    intrPrms.corepacConfig.corepacEventNum  = (int32_t)coreIntrNum;
    intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
#else
    /* Other (non-C66x) CPUs don't use event number and interrupt number is
     * passed in and programmed to CPU Intc directly.
     */
    intrPrms.corepacConfig.corepacEventNum  = (int32_t)0;
    intrPrms.corepacConfig.intVecNum        = (int32_t)coreIntrNum;
#endif

    /* Register interrupts */
    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        hwiHandle = NULL_PTR;
    }

    return (hwiHandle);
}

static void Udma_osalUnRegisterIntr(void *hwiHandle)
{
    int32_t     corepacEventNum = (int32_t)0;   //TODO: Should be set based on core?

    /* Delete interrupts */
    (void) Osal_DeleteInterrupt((HwiP_Handle) hwiHandle, corepacEventNum);

    return;
}

static void Udma_osalDisableIntr(uint32_t coreIntrNum)
{
    int32_t     corepacEventNum = (int32_t)0;   //TODO: Should be set based on core?

    Osal_DisableInterrupt(corepacEventNum, (int32_t)coreIntrNum);

    return;
}

/* We need only mutex in UDMA driver. Use semaphore in binary mode */
static void *Udma_osalMutexCreate(void)
{
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   mutexHandle;

    SemaphoreP_Params_init(&semPrms);
    semPrms.mode = SemaphoreP_Mode_BINARY;
    mutexHandle = (void *) SemaphoreP_create(1U, &semPrms);

    return (mutexHandle);
}

static void Udma_osalMutexDelete(void *mutexHandle)
{
    (void) SemaphoreP_delete((SemaphoreP_Handle) mutexHandle);
}

static void Udma_osalMutexLock(void *mutexHandle)
{
    (void) SemaphoreP_pend((SemaphoreP_Handle) mutexHandle, SemaphoreP_WAIT_FOREVER);
}

static void Udma_osalMutexUnlock(void *mutexHandle)
{
    (void) SemaphoreP_post((SemaphoreP_Handle) mutexHandle);
}
