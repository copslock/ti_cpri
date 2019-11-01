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
 *  \ingroup DRV_UDMA_MODULE
 *  \defgroup DRV_UDMA_OSAL_MODULE UDMA Driver OSAL API
 *            This is UDMA driver OSAL related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_osal.h
 *
 *  \brief UDMA OSAL related parameters and API.
 */

#ifndef UDMA_OSAL_H_
#define UDMA_OSAL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \brief UDMA OSAL intr disable function prototype.
 *
 *  \return Cookie to be passed back to enable interrupt function
 */
typedef uintptr_t (*Udma_OsalDisableAllIntrFxn)(void);

/**
 *  \brief UDMA OSAL intr restore function prototype.
 *
 *  \param cookie       [IN] This is returned in disable interrupt function
 */
typedef void (*Udma_OsalRestoreAllIntrFxn)(uintptr_t cookie);

/**
 *  \brief UDMA OSAL intr disable function prototype.
 *
 *  \param coreIntrNum  [IN] Interrupt to disable
 */
typedef void (*Udma_OsalDisableIntrFxn)(uint32_t coreIntrNum);

/**
 *  \brief UDMA OSAL mutex create function prototype to protect critical section.
 *
 *  \return Pointer to mutex object
 */
typedef void * (*Udma_OsalMutexCreateFxn)(void);

/**
 *  \brief UDMA OSAL mutex delete function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef void (*Udma_OsalMutexDeleteFxn)(void *mutexHandle);

/**
 *  \brief UDMA OSAL mutex lock function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef void (*Udma_OsalMutexLockFxn)(void *mutexHandle);

/**
 *  \brief UDMA OSAL mutex lock function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef void (*Udma_OsalMutexUnlockFxn)(void *mutexHandle);

/**
 *  \brief UDMA OSAL ISR callback function prototype.
 *
 *  \param arg          [IN] App data
 */
typedef void (*Udma_OsalIsrFxn)(uintptr_t arg);

/**
 *  \brief UDMA OSAL ISR register function prototype.
 *
 *  \param isrFxn       [IN] ISR callback fxn pointer
 *  \param coreIntrNum  [IN] Core interrupt number to register
 *  \param intrPriority [IN] Priority
 *  \param arg          [IN] Arg that will be passed back in the ISR
 *
 *  \return Created HWI handle
 */
typedef void *(*Udma_OsalRegisterIntrFxn)(Udma_OsalIsrFxn isrFxn,
                                          uint32_t coreIntrNum,
                                          uint32_t intrPriority,
                                          void *arg);

/**
 *  \brief UDMA OSAL ISR unregister function prototype.
 *
 *  \param hwiHandle    [IN] HWI handle
 */
typedef void (*Udma_OsalUnRegisterIntrFxn)(void *hwiHandle);

/**
 *  \brief UDMA OSAL cache invalidate function prototype.
 *
 *  \param addr         [IN] Start address of the cache line/s
 *  \param size         [IN] size (in bytes) of the memory to invalidate
 */
typedef void (*Udma_OsalCacheInv)(const void *addr, int32_t size);

/**
 *  \brief UDMA OSAL cache writeback function prototype.
 *
 *  \param addr         [IN] Start address of the cache line/s
 *  \param size         [IN] size (in bytes) of the memory to be written back
 */
typedef void (*Udma_OsalCacheWb)(const void *addr, int32_t size);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA driver OSAL function pointers.
 */
typedef struct
{
    Udma_OsalDisableAllIntrFxn  disableAllIntr;
    /**< OSAL all interrupt disable function pointer */
    Udma_OsalRestoreAllIntrFxn  restoreAllIntr;
    /**< OSAL all interrupt restore function pointer */
    Udma_OsalDisableIntrFxn     disableIntr;
    /**< OSAL interrupt disable function pointer */

    Udma_OsalMutexCreateFxn     createMutex;
    /**< Create mutex function pointer */
    Udma_OsalMutexDeleteFxn     deleteMutex;
    /**< Delete mutex function pointer */
    Udma_OsalMutexLockFxn       lockMutex;
    /**< Lock mutex function pointer */
    Udma_OsalMutexUnlockFxn     unlockMutex;
    /**< Unlock mutex function pointer */

    Udma_OsalRegisterIntrFxn    registerIntr;
    /**< Register interrupt function pointer */
    Udma_OsalUnRegisterIntrFxn  unRegisterIntr;
    /**< Unregister interrupt function pointer */
} Udma_OsalPrms;

/**
 *  \brief UDMA driver OSAL cache function pointers.
 *
 *  Note: This structure is separated from the #Udma_OsalPrms so that
 *  the cache API are set independent of driver handle. The cache API
 *  function pointer is common to all handles.
 */
typedef struct
{
    Udma_OsalCacheInv           cacheInv;
    /**< Cache Invalidate function pointer */
    Udma_OsalCacheWb            cacheWb;
    /**< Cache writeback function pointer */
} Udma_OsalCachePrms;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief This API sets the OSAL cache paramaters incase user needs to
 *  override the default cache API.
 *
 *  Caution: This is common across all handles and should be set perferably
 *  once for a given core before calling #Udma_init.
 *
 *  Note: All the function pointer needs to be set.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2984)
 *
 *  \param cachePrms    [IN] Pointer to #Udma_OsalCachePrms structure.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_osalSetCachePrms(const Udma_OsalCachePrms *cachePrms);

/*
 * Structure Init functions
 */
/**
 *  \brief Udma_OsalPrms structure init function.
 *
 *  \param osalPrms     [IN] Pointer to #Udma_OsalPrms structure.
 *
 */
void UdmaOsalPrms_init(Udma_OsalPrms *osalPrms);

/**
 *  \brief Udma_OsalCachePrms structure init function.
 *
 *  \param cachePrms    [IN] Pointer to #Udma_OsalCachePrms structure.
 *
 */
void UdmaOsalCachePrms_init(Udma_OsalCachePrms *cachePrms);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_OSAL_H_ */

/* @} */
