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
 *  \file ipc_osal.c
 *
 *  \brief Implementation of ipc sysbios osal
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ipc_osal.h"

/* Calling SysBios header file directly, need to move to OSAL */
#include <ti/sysbios/gates/GateSwi.h>
#include <xdc/runtime/System.h>

#include <ti/osal/SwiP.h>

#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/soc/ipc_soc.h>
#include <ti/drv/ipc/include/ipc_types.h>

#include <stdio.h>

/* GateSwi related */
static void *Ipc_osalHIsrGateCreate(void)
{
    GateSwi_Params   gatePrms;
    GateSwi_Params_init(&gatePrms);
    return (void *) GateSwi_create(&gatePrms, NULL);
}

static void Ipc_osalHIsrGateDelete(Ipc_OsalHIsrGateHandle *handle)
{
    GateSwi_delete((GateSwi_Handle *)handle);
}

static int32_t Ipc_osalHIsrGateEnter(Ipc_OsalHIsrGateHandle handle)
{
    return (int32_t)GateSwi_enter((GateSwi_Handle)handle);
}

static void Ipc_osalHIsrGateExit(Ipc_OsalHIsrGateHandle handle, int32_t key)
{
    GateSwi_leave((GateSwi_Handle)handle, (IArg)key);
}

static int32_t Ipc_osalHIsrCreate(Ipc_OsalHIsrHandle *handle,
                                    Ipc_OsalHIsrFxn fxn, void *arg)
{
    SwiP_Params       params;
    int32_t rtnVal = IPC_EBADARGS;

    if ((NULL != handle) && ((NULL != fxn) && (NULL != arg)))
    {
        SwiP_Params_init(&params);
        params.arg0     = (uintptr_t) arg;

        handle->arg0    = (uintptr_t)NULL;
        handle->arg1    = (uintptr_t)NULL;
        handle->hIsrFxn = NULL;
        handle->hLosHisrHandle = (void *)SwiP_create((SwiP_Fxn)fxn,
                                                        &params);
        rtnVal = IPC_SOK;
    }

    return (rtnVal);
}

static void Ipc_osalHIsrDelete(Ipc_OsalHIsrHandle *handle)
{
    if (NULL != handle)
    {
        SwiP_delete((SwiP_Handle *)handle->hLosHisrHandle);
    }
}

static int32_t Ipc_osalHIsrPost(Ipc_OsalHIsrHandle *handle)
{
    int32_t rtnVal = IPC_EBADARGS;

    if (NULL != handle)
    {
        SwiP_post((SwiP_Handle)handle->hLosHisrHandle);
        rtnVal = IPC_SOK;
    }
    return (rtnVal);
}

int32_t SystemP_printf(const char* fmt, ...)
{
    int32_t ret;
    va_list args;
    va_start(args, fmt);
    ret = System_vprintf(fmt, args);
    va_end(args);
    return ret;
}

void *Mailbox_plugInterrupt(Ipc_MbConfig *cfg, Ipc_OsalIsrFxn func, uintptr_t arg)
{
    OsalRegisterIntrParams_t    intrPrms;
    OsalInterruptRetCode_e      osalRetVal;
    HwiP_Handle                 hwiHandle = NULL;
#ifndef IPC_SUPPORT_SCICLIENT
    CSL_IntrRouterCfg           irRegs;
#endif

#ifdef DEBUG_PRINT
    SystemP_printf("Navss Rtr: input %d, output %d%d\n",
        cfg->inputIntrNum, cfg->outputIntrNum);
#endif

#ifndef IPC_SUPPORT_SCICLIENT
    /* Configure Main NavSS512 interrupt router */
    irRegs.pIntrRouterRegs = (CSL_intr_router_cfgRegs *)IPC_MCU_NAVSS0_INTR0_CFG_BASE;
    irRegs.pIntdRegs       = (CSL_intr_router_intd_cfgRegs *) NULL;
    irRegs.numInputIntrs   = MAIN_NAVSS_MAILBOX_INPUTINTR_MAX;
    irRegs.numOutputIntrs  = MAIN_NAVSS_MAILBOX_OUTPUTINTR_MAX;
    CSL_intrRouterCfgMux(&irRegs, cfg->inputIntrNum, cfg->outputIntrNum);

#if defined (SOC_AM65XX)
#if defined(BUILD_MCU1_0) || defined(BUILD_MCU1_1)
    Ipc_main2mcu_intRouter(cfg);
#endif
#endif


    /* Configure C66x Interrupt Router now */
#if defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
    Ipc_configC66xIntrRouter(cfg->eventId );
#endif

#endif  /* IPC_SUPPORT_SCICLIENT */

    /*
     * CLEC needs to be configured for all modes - CSL and Sciclient
     **/
#if defined(BUILD_C7X_1)
    Ipc_configClecRouter(cfg->eventId );
#endif

    /* Register interrupts */
    Osal_RegisterInterrupt_initParams(&intrPrms);
    intrPrms.corepacConfig.arg              = arg;
    intrPrms.corepacConfig.isrRoutine       = func;
    intrPrms.corepacConfig.priority         = cfg->priority;

#if defined(BUILD_C66X_1) || defined(BUILD_C66X_2)
    intrPrms.corepacConfig.corepacEventNum  = cfg->eventId;
    intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
#else
    intrPrms.corepacConfig.intVecNum        = cfg->eventId;
    intrPrms.corepacConfig.corepacEventNum  = 0;
#endif

    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        SystemP_printf("Mailbox_plugInterrupt : Failed to register ISR...\n");
    }

    return (hwiHandle);
}

/* We need only mutex in IPC driver. Use semaphore in binary mode */
static void *Ipc_osalMutexCreate(void)
{
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   mutexHandle;

    SemaphoreP_Params_init(&semPrms);
    mutexHandle = (void *) SemaphoreP_create(0U, &semPrms);

    return ((void*) mutexHandle);
}

static void Ipc_osalMutexDelete(void *mutexHandle)
{
    SemaphoreP_delete((SemaphoreP_Handle) mutexHandle);
}

static int32_t Ipc_osalMutexLock(void *mutexHandle, uint32_t timeout)
{
    int32_t retVal = IPC_SOK;
    SemaphoreP_Status  semStatus;

    semStatus = SemaphoreP_pend((SemaphoreP_Handle) mutexHandle, timeout);
    if (SemaphoreP_OK != semStatus)
    {
        retVal = IPC_ETIMEOUT;
    }

    return retVal;
}

static void Ipc_osalMutexUnlock(void *mutexHandle)
{
    SemaphoreP_post((SemaphoreP_Handle) mutexHandle);
}


void IpcOsalPrms_init (Ipc_OsalPrms *initPrms)
{
    if (NULL != initPrms)
    {
        initPrms->disableAllIntr = &HwiP_disable;
        initPrms->restoreAllIntr = &HwiP_restore;

        initPrms->createHIsr    = &Ipc_osalHIsrCreate;
        initPrms->deleteHIsr    = &Ipc_osalHIsrDelete;
        initPrms->postHIsr      = &Ipc_osalHIsrPost;

        initPrms->createHIsrGate = &Ipc_osalHIsrGateCreate;
        initPrms->deleteHIsrGate = &Ipc_osalHIsrGateDelete;
        initPrms->lockHIsrGate   = &Ipc_osalHIsrGateEnter;
        initPrms->unLockHIsrGate = &Ipc_osalHIsrGateExit;

        initPrms->createMutex   = &Ipc_osalMutexCreate;
        initPrms->deleteMutex   = &Ipc_osalMutexDelete;
        initPrms->lockMutex     = &Ipc_osalMutexLock;
        initPrms->unlockMutex   = &Ipc_osalMutexUnlock;

        initPrms->registerIntr = &Mailbox_plugInterrupt;
    }
    return;
}
