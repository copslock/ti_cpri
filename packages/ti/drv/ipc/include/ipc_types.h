/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
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
 *  \file ipc_types.h
 *
 *  \brief data types definitions for ipc module.
 */

#ifndef IPC_TYPES_H_
#define IPC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 *  \anchor Ipc_ErrorCodes
 *  \name IPC Error Codes
 *
 *  Error codes returned by IPC APIs
 *
 *  @{
 */
/** \brief API call successful */
#define IPC_SOK                        (CSL_PASS)

/** \brief API call returned with error as failed. Used for generic error.
 *  It may be some hardware failure and/or software failure. */
#define IPC_EFAIL                      (CSL_EFAIL)

/** \brief API call returned with error as bad arguments.
 *  Typically, NULL pointer passed to the API where its not expected. */
#define IPC_EBADARGS                   (CSL_EBADARGS)

/** \brief API call returned with error as invalid parameters. Typically
 *  when parameters passed are not valid. */
#define IPC_EINVALID_PARAMS            (CSL_EINVALID_PARAMS)

/** \brief API call returned with error as timed out. Typically API is
 *  waiting for some condition and returned as condition not happened
 *  in the timeout period. */
#define IPC_ETIMEOUT                   (CSL_ETIMEOUT)

/** \brief API call returned with error as allocation failed. */
#define IPC_EALLOC                     (CSL_EALLOC)

/** \brief Un Supported service request, API/IOCTLs */
#define IPC_EUNSUPPORTED               (CSL_EUNSUPPORTED_CMD)

/** \brief IPC is unblocked mode */
#define IPC_E_UNBLOCKED                (-50)

/** \brief Macro used to specify that interrupt number is invalid. */
#define IPC_INTR_INVALID               (0xFFFF0000U)

/**
 * \brief Timeout foreever for IPC Recv
 */
#define IPC_RPMESSAGE_TIMEOUT_FOREVER   ~(0)

/**
 * \brief RPMessage Endpoint any available
 */
#define RPMESSAGE_ANY                   (0xFFFFFFFFU)

/**
 * \brief RPMessage type
 */
#define RPMESSAGE_ALL                   (0xFFFFFFFFU)

/**
 * \brief Mailbox interrupt router configuration
 */
typedef struct Ipc_MbConfig_s
{
    uint32_t   priority;
    uint32_t   eventId;
    uint32_t   inputIntrNum;
    uint32_t   outputIntrNum;
}Ipc_MbConfig;

/**
 *  \brief IPC OSAL High Level ISR type
 *
 *  \param uintptr_t    arg1, expected to be of type RPMessage_CallbackData
 *  \param uintptr_t    arg2, not used
 *
 *  \return None
 */
typedef void (*Ipc_OsalHIsrFxn)(uintptr_t arg0, uintptr_t arg1);

/**
 * \brief Second level ISR (HISR), used to complete ISR functions
 */
typedef struct Ipc_OsalHIsrHandle_s
{
    uintptr_t       arg0;
    /**< First argument, expected to be of type RPMessage_CallbackData */
    uintptr_t       arg1;
    /**< First argument, Not used in this implementation */
    Ipc_OsalHIsrFxn hIsrFxn;
    /**< Function to handle HIsr */
    void            *hLosHisrHandle;
    /**< When used to high level OS, TIRTOS, associated HISR handle */
}Ipc_OsalHIsrHandle;

/**
 *  \brief High Level ISR handle
 */
typedef void (* Ipc_OsalHIsrGateHandle);

/**
 *  \brief IPC Virtual to Physical address translation callback function.
 *
 *  This function is used by the driver to convert virtual address to physical
 *  address.
 *
 *  \param virtAddr [IN] Virtual address
 *
 *  \return Corresponding physical address
 */
typedef uint32_t (*Ipc_VirtToPhyFxn)(const void *virtAddr);

/**
 *  \brief IPC Physical to Virtual address translation callback function.
 *
 *  This function is used by the driver to convert physical address to virtual
 *  address.
 *
 *  \param phyAddr  [IN] Physical address
 *
 *  \return Corresponding virtual address
 */
typedef void *(*Ipc_PhyToVirtFxn)(uint32_t phyAddr);

/**
 *  \brief IPC OSAL interrupt disable function prototype.
 *
 *  \return Cookie to be passed back to enable interrupt function
 */
typedef uintptr_t (*Ipc_OsalDisableAllIntrFxn)(void);

/**
 *  \brief IPC OSAL interrupt restore function prototype.
 *
 *  \param cookie       [IN] This is returned in disable interrupt function
 */
typedef void (*Ipc_OsalRestoreAllIntrFxn)(uintptr_t cookie);

/**
 *  \brief IPC OSAL create high level interrupt lock
 *
 *  \param None
 *
 *  \return Ipc_OsalHIsrGateHandle on success
 */
typedef Ipc_OsalHIsrGateHandle (*Ipc_OsalCreateHIsrMutexFxn)(void);

/**
 *  \brief IPC OSAL delete high level interrupt lock
 *
 *  \param Ipc_OsalHIsrGateHandle Handle previously created gate
 *
 *  \return None
 */
typedef void (*Ipc_OsalDeleteHIsrMutexFxn)(Ipc_OsalHIsrGateHandle *handle);

/**
 *  \brief IPC OSAL Lock High level ISR's
 *
 *  \param Ipc_OsalHIsrGateHandle Handle previously created gate
 *
 *  \return int32_t Unique key value, which would be required to unlock
 */
typedef int32_t (*Ipc_OsalEnterHIsrMutexFxn)(Ipc_OsalHIsrGateHandle handle);

/**
 *  \brief IPC OSAL Un Lock High level ISR's
 *
 *  \param Ipc_OsalHIsrGateHandle Handle previously created gate
 *  \param int32_t Unique key value, which received when locking
 *
 *  \return None
 */
typedef void (*Ipc_OsalExitHIsrMutexFxn)(Ipc_OsalHIsrGateHandle handle,
                                            int32_t key);

/**
 *  \brief IPC OSAL mutex create function prototype to protect critical section.
 *
 *  \return Pointer to mutex object
 */
typedef void * (*Ipc_OsalMutexCreateFxn)(void);

/**
 *  \brief IPC OSAL mutex delete function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef void (*Ipc_OsalMutexDeleteFxn)(void *mutexHandle);

/**
 *  \brief IPC OSAL mutex lock function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef int32_t (*Ipc_OsalMutexLockFxn)(void *mutexHandle, uint32_t timeout);

/**
 *  \brief IPC OSAL mutex lock function prototype.
 *
 *  \param mutexHandle  [IN] Pointer to mutex object returned during create
 */
typedef void (*Ipc_OsalMutexUnlockFxn)(void *mutexHandle);

/**
 *  \brief IPC OSAL create high level interrupt handler
 *
 *  \param None
 *
 *  \return IPC_SOK on success
 */
typedef int32_t (*Ipc_OsalHIsrCreateFxn)(Ipc_OsalHIsrHandle *handle,
                                         Ipc_OsalHIsrFxn fxn, void *arg);

/**
 *  \brief IPC OSAL delete high level interrupt handler
 *
 *  \param Ipc_OsalHIsrHandle Handle previously created HISR
 *
 *  \return None
 */
typedef void (*Ipc_OsalHIsrDeleteFxn)(Ipc_OsalHIsrHandle *handle);

/**
 *  \brief IPC OSAL Lock High level ISR's
 *
 *  \param Ipc_OsalHIsrHandle Handle previously created HISR
 *
 *  \return int32_t IPC_SOK on success
 */
typedef int32_t (*Ipc_OsalHIsrPostFxn)(Ipc_OsalHIsrHandle *handle);

/**
 *  \brief IPC OSAL ISR callback function prototype.
 *
 *  \param arg          [IN] App data
 */
typedef void (*Ipc_OsalIsrFxn)(uintptr_t arg);

/**
 *  \brief IPC OSAL ISR register function prototype.
 *
 *  \param cfg          [IN] Defines interrupt details #Ipc_MbConfig
 *  \param isrFxn       [IN] ISR callback fxn pointer
 *  \param arg          [IN] Arg that will be passed back in the ISR
 *
 *  \return Created HWI handle
 */
typedef void *(*Ipc_OsalRegisterIntrFxn)(Ipc_MbConfig *cfg,
                                         Ipc_OsalIsrFxn isrFxn,
                                         uintptr_t arg);

/**
 *  \brief IPC OSAL ISR unregister function prototype.
 *
 *  \param hwiHandle    [IN] HWI handle
 */
typedef void (*Ipc_OsalUnRegisterIntrFxn)(void *hwiHandle);

/**
 *  \brief IPC New message notification
 *
 *  \param srcEndPt     [IN] Specifies the source end point number
 *  \param srcProcId    [IN] Specifies the source processor identifier
 */
typedef void (*Ipc_NewMsgReceivedFxn)(uint32_t srcEndPt, uint32_t procId);

/**
 *  \brief IPC driver OSAL function pointers.
 */
typedef struct Ipc_OsalPrms_s
{
    Ipc_OsalDisableAllIntrFxn   disableAllIntr;
    /**< OSAL all interrupt disable function pointer */
    Ipc_OsalRestoreAllIntrFxn   restoreAllIntr;
    /**< OSAL all interrupt restore function pointer */

    Ipc_OsalHIsrCreateFxn       createHIsr;
    /**< Create High ISR handler function */
    Ipc_OsalHIsrDeleteFxn       deleteHIsr;
    /**< Deleted, previously created High ISR handler function */
    Ipc_OsalHIsrPostFxn         postHIsr;
    /**< Trigger HISR */

    Ipc_OsalCreateHIsrMutexFxn  createHIsrGate;
    /**< Function pointer to create high level ISR locking mechanism */
    Ipc_OsalDeleteHIsrMutexFxn  deleteHIsrGate;
    /**< Function pointer to delete high level ISR locking mechanism */
    Ipc_OsalEnterHIsrMutexFxn   lockHIsrGate;
    /**< Function pointer to lock high level ISR */
    Ipc_OsalExitHIsrMutexFxn    unLockHIsrGate;
    /**< Function pointer to un lock high level ISR */


    Ipc_OsalMutexCreateFxn      createMutex;
    /**< Create mutex function pointer */
    Ipc_OsalMutexDeleteFxn      deleteMutex;
    /**< Delete mutex function pointer */
    Ipc_OsalMutexLockFxn        lockMutex;
    /**< Lock mutex function pointer */
    Ipc_OsalMutexUnlockFxn      unlockMutex;
    /**< Unlock mutex function pointer */

    Ipc_OsalRegisterIntrFxn     registerIntr;
    /**< Register interrupt function pointer */
    Ipc_OsalUnRegisterIntrFxn   unRegisterIntr;
    /**< Unregister interrupt function pointer */
} Ipc_OsalPrms;

#ifdef __cplusplus
}
#endif

#endif /* #ifndef IPC_TYPES_H_ */
