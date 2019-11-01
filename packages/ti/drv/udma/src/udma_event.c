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
 *  \file udma_event.c
 *
 *  \brief File containing the UDMA driver event management functions.
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

static void Udma_eventIsrFxn(uintptr_t arg);
static int32_t Udma_eventCheckParams(Udma_DrvHandle drvHandle,
                                     const Udma_EventPrms *eventPrms);
static int32_t Udma_eventAllocResource(Udma_DrvHandle drvHandle,
                                       Udma_EventHandle eventHandle);
static void Udma_eventFreeResource(Udma_DrvHandle drvHandle,
                                   Udma_EventHandle eventHandle);
static int32_t Udma_eventConfig(Udma_DrvHandle drvHandle,
                                Udma_EventHandle eventHandle);
static int32_t Udma_eventReset(Udma_DrvHandle drvHandle,
                               Udma_EventHandle eventHandle);
static int32_t Udma_eventProgramSteering(Udma_DrvHandle drvHandle,
                                         Udma_EventHandle eventHandle);
static void Udma_eventResetSteering(Udma_DrvHandle drvHandle,
                                    Udma_EventHandle eventHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_eventRegister(Udma_DrvHandle drvHandle,
                           Udma_EventHandle eventHandle,
                           Udma_EventPrms *eventPrms)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        allocDone = (uint32_t) FALSE;

    /* Error check */
    if((NULL_PTR == drvHandle) || (NULL_PTR == eventHandle) || (NULL_PTR == eventPrms))
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
        retVal = Udma_eventCheckParams(drvHandle, eventPrms);
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy and init parameters */
        (void) memcpy(
            &eventHandle->eventPrms, eventPrms, sizeof(eventHandle->eventPrms));
        eventHandle->drvHandle      = drvHandle;
        eventHandle->globalEvent    = UDMA_EVENT_INVALID;
        eventHandle->vintrNum       = UDMA_EVENT_INVALID;
        eventHandle->vintrBitNum    = UDMA_EVENT_INVALID;
        eventHandle->coreIntrNum    = UDMA_INTR_INVALID;
        eventHandle->nextEvent      = (Udma_EventHandle) NULL_PTR;
        eventHandle->prevEvent      = (Udma_EventHandle) NULL_PTR;
        eventHandle->hwiHandle      = NULL_PTR;
        eventHandle->vintrBitAllocFlag = 0U;
        eventHandle->pIaGeviRegs    = (volatile CSL_intaggr_imapRegs_gevi *) NULL_PTR;
        eventHandle->pIaVintrRegs   = (volatile CSL_intaggr_intrRegs_vint *) NULL_PTR;

        /* Alloc event resources */
        retVal = Udma_eventAllocResource(drvHandle, eventHandle);
        if(UDMA_SOK == retVal)
        {
            allocDone = (uint32_t) TRUE;
        }
        else
        {
            Udma_printf(drvHandle, "[Error] Event resource allocation failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Set init flag as events are allocated and event config expects
         * this flag to be set */
        eventHandle->eventInitDone = UDMA_INIT_DONE;

        /* Configure Event */
        retVal = Udma_eventConfig(drvHandle, eventHandle);
        if(UDMA_SOK == retVal)
        {
            allocDone = (uint32_t) TRUE;
        }
        else
        {
            Udma_printf(drvHandle, "[Error] Event config failed!!\n");
        }
    }

    if(UDMA_SOK != retVal)
    {
        /* Error. Free-up resource if allocated */
        if(((uint32_t) TRUE) == allocDone)
        {
            Udma_eventFreeResource(drvHandle, eventHandle);
            eventHandle->eventInitDone = UDMA_DEINIT_DONE;
        }
    }
    else
    {
        /* Copy the allocated resource info */
        Udma_assert(drvHandle, eventHandle->pIaVintrRegs != NULL_PTR);
        eventPrms->intrStatusReg    = &eventHandle->pIaVintrRegs->STATUSM;
        eventPrms->intrClearReg     = &eventHandle->pIaVintrRegs->STATUS_CLEAR;
        if(eventHandle->vintrBitNum != UDMA_EVENT_INVALID)
        {
            eventPrms->intrMask     = ((uint64_t)1U << eventHandle->vintrBitNum);
        }
        else
        {
            /* No VINT bit for global master event */
            eventPrms->intrMask     = 0U;
        }
        if(NULL_PTR == eventHandle->eventPrms.masterEventHandle)
        {
            /* This is master handle - copy directly from here itself */
            eventPrms->coreIntrNum  = eventHandle->coreIntrNum;
        }
        else
        {
            /* Copy core number from master handle */
            eventPrms->coreIntrNum    =
                eventHandle->eventPrms.masterEventHandle->coreIntrNum;
        }
    }

    return (retVal);
}

int32_t Udma_eventUnRegister(Udma_EventHandle eventHandle)
{
    int32_t         retVal = UDMA_SOK;
    Udma_DrvHandle  drvHandle;

    /* Error check */
    if(NULL_PTR == eventHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        if(eventHandle->eventInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = eventHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Can't free-up master event when shared events are still not yet
         * unregistered */
        if((NULL_PTR == eventHandle->eventPrms.masterEventHandle) &&
           (NULL_PTR != eventHandle->nextEvent))
        {
            retVal = UDMA_EFAIL;
            Udma_printf(drvHandle,
                "[Error] Can't free master event when shared events are still registered!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(NULL_PTR != eventHandle->hwiHandle)
        {
            /* Disable able core interrupt to avoid having insane
             * state/variables when an interrupt occurs while processing
             * event free */
            Udma_assert(drvHandle, eventHandle->coreIntrNum != UDMA_INTR_INVALID);
            Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.disableIntr != (Udma_OsalDisableIntrFxn) NULL_PTR);
            drvHandle->initPrms.osalPrms.disableIntr(eventHandle->coreIntrNum);
        }
        /* Reset and Free-up event resources */
        retVal = Udma_eventReset(drvHandle, eventHandle);
        if(UDMA_SOK != retVal)
        {
            Udma_printf(drvHandle, "[Error] Free Event resource failed!!!\n");
        }
        Udma_eventFreeResource(drvHandle, eventHandle);

        eventHandle->eventInitDone  = UDMA_DEINIT_DONE;
        eventHandle->pIaGeviRegs    = (volatile CSL_intaggr_imapRegs_gevi *) NULL_PTR;
        eventHandle->pIaVintrRegs   = (volatile CSL_intaggr_intrRegs_vint *) NULL_PTR;
        eventHandle->drvHandle      = (Udma_DrvHandle) NULL_PTR;
    }

    return (retVal);
}

uint32_t Udma_eventGetId(Udma_EventHandle eventHandle)
{
    uint32_t        evtNum = UDMA_EVENT_INVALID;
    Udma_DrvHandle  drvHandle;

    if((NULL_PTR != eventHandle) && (UDMA_INIT_DONE == eventHandle->eventInitDone))
    {
        drvHandle = eventHandle->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            evtNum = drvHandle->iaGemOffset + eventHandle->globalEvent;
        }
    }

    return (evtNum);
}

int32_t Udma_eventDisable(Udma_EventHandle eventHandle)
{
    int32_t         retVal = UDMA_EFAIL;
    Udma_DrvHandle  drvHandle;
    uint32_t        vintrBitNum;
    uint32_t        vintrNum;

    if((NULL_PTR != eventHandle) && (UDMA_INIT_DONE == eventHandle->eventInitDone))
    {
        drvHandle = eventHandle->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            vintrNum = eventHandle->vintrNum;
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandle->vintrBitNum;

            retVal = CSL_intaggrSetIntrEnable(
                         &drvHandle->iaRegs, vintrBitNum, (bool)false);
        }
    }

    return (retVal);
}

int32_t Udma_eventEnable(Udma_EventHandle eventHandle)
{
    int32_t         retVal = UDMA_EFAIL;
    Udma_DrvHandle  drvHandle;
    uint32_t        vintrBitNum;
    uint32_t        vintrNum;

    if((NULL_PTR != eventHandle) && (UDMA_INIT_DONE == eventHandle->eventInitDone))
    {
        drvHandle = eventHandle->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            vintrNum = eventHandle->vintrNum;
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandle->vintrBitNum;

            retVal = CSL_intaggrSetIntrEnable(
                         &drvHandle->iaRegs, vintrBitNum, (bool)true);
        }
    }

    return (retVal);
}

Udma_EventHandle Udma_eventGetGlobalHandle(Udma_DrvHandle drvHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_EventHandle    eventHandle = (Udma_EventHandle) NULL_PTR;

    /* Error check */
    if(NULL_PTR == drvHandle)
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
        eventHandle = drvHandle->globalEventHandle;
    }

    return (eventHandle);
}

int32_t Udma_eventGetRxFlowIdFwStatus(Udma_EventHandle eventHandle,
                                      Udma_EventRxFlowIdFwStatus *status)
{
    int32_t                                 retVal = UDMA_SOK;
    uint32_t                                regVal;
    Udma_DrvHandle                          drvHandle;
    struct tisci_msg_rm_udmap_gcfg_cfg_req  gcfgReq;

    if((NULL_PTR == eventHandle) ||
       (UDMA_INIT_DONE != eventHandle->eventInitDone) ||
       (NULL_PTR == status))
    {
        retVal = UDMA_EFAIL;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandle = eventHandle->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        regVal = CSL_REG32_RD(&drvHandle->udmapRegs.pGenCfgRegs->RFLOWFWSTAT);
        if(CSL_FEXT(regVal, UDMAP_GCFG_RFLOWFWSTAT_PEND) != 0U)
        {
            status->flowId  = CSL_FEXT(regVal, UDMAP_GCFG_RFLOWFWSTAT_FLOWID);
            status->chNum   = CSL_FEXT(regVal, UDMAP_GCFG_RFLOWFWSTAT_CHANNEL);
            status->isException = TRUE;

            /* Clear pend bit to allow another exception to be captured */
            gcfgReq.valid_params = TISCI_MSG_VALUE_RM_UDMAP_GCFG_RFLOWFWSTAT_VALID;
            gcfgReq.nav_id       = drvHandle->devIdUdma;
            gcfgReq.perf_ctrl    = 0U;  /* Not set/used */
            gcfgReq.emu_ctrl     = 0U;  /* Not set/used */
            gcfgReq.psil_to      = 0U;  /* Not set/used */
            gcfgReq.rflowfwstat  = 0U;  /* Write 0 to clear */
            retVal = Sciclient_rmUdmapGcfgCfg(
                         &gcfgReq, (struct tisci_msg_rm_udmap_gcfg_cfg_resp *) NULL_PTR, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                Udma_printf(drvHandle,
                    "[Error] Sciclient set UDMAP global config failed!!!\n");
            }
        }
        else
        {
            status->flowId  = 0U;
            status->chNum   = 0U;
            status->isException = FALSE;
        }
    }

    return (retVal);
}

void UdmaEventPrms_init(Udma_EventPrms *eventPrms)
{
    if(NULL_PTR != eventPrms)
    {
        eventPrms->eventType            = UDMA_EVENT_TYPE_DMA_COMPLETION;
        eventPrms->eventMode            = UDMA_EVENT_MODE_SHARED;
        eventPrms->chHandle             = (Udma_ChHandle) NULL_PTR;
        eventPrms->ringHandle           = (Udma_RingHandle) NULL_PTR;
        eventPrms->masterEventHandle    = (Udma_EventHandle) NULL_PTR;
        eventPrms->eventCb              = (Udma_EventCallback) NULL_PTR;
        eventPrms->intrPriority         = 1U;
        eventPrms->appData              = NULL_PTR;
        eventPrms->osalRegisterDisable  = FALSE;
        eventPrms->preferredCoreIntrNum = UDMA_CORE_INTR_ANY;
        eventPrms->monHandle            = (Udma_RingMonHandle) NULL_PTR;
        eventPrms->intrStatusReg        = (volatile uint64_t *) NULL_PTR;
        eventPrms->intrClearReg         = (volatile uint64_t *) NULL_PTR;
        eventPrms->intrMask             = 0U;
        eventPrms->coreIntrNum          = UDMA_INTR_INVALID;
    }

    return;
}

static void Udma_eventIsrFxn(uintptr_t arg)
{
    uint32_t            vintrBitNum;
    uint32_t            vintrNum;
    Udma_EventHandle    eventHandle = (Udma_EventHandle) arg;
    Udma_DrvHandle      drvHandle;
    Udma_EventPrms     *eventPrms;

    drvHandle = eventHandle->drvHandle;
    vintrNum = eventHandle->vintrNum;
    Udma_assert(drvHandle, vintrNum != UDMA_EVENT_INVALID);
    /* Loop through all the shared events. In case of exclusive events,
     * the next event is NULL_PTR and the logic remains same and the while breaks */
    while(eventHandle != NULL_PTR)
    {
        /* There is no valid VINT bit for global master event */
        if(UDMA_EVENT_TYPE_MASTER != eventHandle->eventPrms.eventType)
        {
            Udma_assert(drvHandle,
                eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandle->vintrBitNum;

            /* Check IA status */
            if((bool)true == CSL_intaggrIsIntrPending(&drvHandle->iaRegs, vintrBitNum, (bool)true))
            {
                /* Clear the interrupt */
                (void) CSL_intaggrClrIntr(&drvHandle->iaRegs, vintrBitNum);

                /* Notify through callback if registered */
                eventPrms = &eventHandle->eventPrms;
                if((Udma_EventCallback) NULL_PTR != eventPrms->eventCb)
                {
                    eventPrms->eventCb(
                        eventHandle, eventPrms->eventType, eventPrms->appData);
                }
            }
        }

        /* Move to next shared event */
        eventHandle = eventHandle->nextEvent;
    }

    return;
}

static int32_t Udma_eventCheckParams(Udma_DrvHandle drvHandle,
                                     const Udma_EventPrms *eventPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_EventHandle    masterEventHandle;

    Udma_assert(drvHandle, eventPrms != NULL_PTR);

    /* Exclusive event checks */
    if(UDMA_EVENT_MODE_EXCLUSIVE == eventPrms->eventMode)
    {
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Master event handle should be NULL_PTR for exclusive event!!!\n");
        }
    }

    /* Shared event checks */
    if(UDMA_EVENT_MODE_SHARED == eventPrms->eventMode)
    {
        /* Shared event slave checks */
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            /* Check if callback is non-null for slave shared events when
             * master has callback set - This is becasuse once the master has
             * interrupt registered, all slaves should have a callback as IA
             * is same and there is no individual control to disable
             * interrupt */
            if(FALSE == eventPrms->osalRegisterDisable)
            {
                masterEventHandle = eventPrms->masterEventHandle;
                if(((Udma_EventCallback) NULL_PTR != masterEventHandle->eventPrms.eventCb) &&
                   ((Udma_EventCallback) NULL_PTR == eventPrms->eventCb))
                {
                    retVal = UDMA_EINVALID_PARAMS;
                    Udma_printf(drvHandle,
                        "[Error] No callback set for slave shared events!!!\n");
                }
                /* Check if master has not registered a callback, the slave should not
                 * expect a callback either!! */
                if(((Udma_EventCallback) NULL_PTR == masterEventHandle->eventPrms.eventCb) &&
                   ((Udma_EventCallback) NULL_PTR != eventPrms->eventCb) &&
                   (UDMA_EVENT_TYPE_MASTER != masterEventHandle->eventPrms.eventType))
                {
                    retVal = UDMA_EINVALID_PARAMS;
                    Udma_printf(drvHandle,
                        "[Error] Callback set for slave shared events when master event didnot set a callback!!!\n");
                }
            }
        }
    }

    /* Channel handle should be provided to reconfigure channel related event */
    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TR == eventPrms->eventType))
    {
        if(NULL_PTR == eventPrms->chHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Channel handle should be provided for ring/ch OES programming!!!\n");
        }
    }

    /* Ring handle should be provided to configure ring event */
    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        if(NULL_PTR == eventPrms->ringHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Ring handle should be provided for ring OES programming!!!\n");
        }
    }

    /* Ring monitor handle should be provided to configure monitor event */
    if(UDMA_EVENT_TYPE_RING_MON == eventPrms->eventType)
    {
        if(NULL_PTR == eventPrms->monHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Monitor handle should be provided for ring monitor OES programming!!!\n");
        }
    }

    if(UDMA_EVENT_TYPE_MASTER == eventPrms->eventType)
    {
        if(UDMA_EVENT_MODE_SHARED != eventPrms->eventMode)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Event should be shareable for global master event type!!!\n");
        }

        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            Udma_printf(drvHandle,
                "[Error] Master handle should be NULL_PTR for master event type!!!\n");
        }
    }

    return (retVal);
}

static int32_t Udma_eventAllocResource(Udma_DrvHandle drvHandle,
                                       Udma_EventHandle eventHandle)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                vintrNum;
    const Udma_EventPrms   *eventPrms;
    Udma_EventHandle        lastEvent;
    uintptr_t               cookie;

    Udma_assert(drvHandle, eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    /* Allocate event irrespective of all modes except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        eventHandle->globalEvent = Udma_rmAllocEvent(drvHandle);
        if(UDMA_EVENT_INVALID == eventHandle->globalEvent)
        {
            retVal = UDMA_EALLOC;
            Udma_printf(drvHandle, "[Error] Global event alloc failed!!!\n");
        }
        else
        {
            Udma_assert(drvHandle, drvHandle->iaRegs.pImapRegs != NULL_PTR);
            eventHandle->pIaGeviRegs =
                &drvHandle->iaRegs.pImapRegs->GEVI[eventHandle->globalEvent];
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate IA register for master and exclusive events */
        if((UDMA_EVENT_MODE_EXCLUSIVE == eventPrms->eventMode) ||
            ((UDMA_EVENT_MODE_SHARED == eventPrms->eventMode) &&
                (NULL_PTR == eventPrms->masterEventHandle)))
        {
            eventHandle->vintrNum = Udma_rmAllocVintr(drvHandle);
            if(UDMA_EVENT_INVALID == eventHandle->vintrNum)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] VINTR alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate IA bit for all event modes except global master event */
        if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
        {
            eventHandle->vintrBitNum = Udma_rmAllocVintrBit(eventHandle);
            if(UDMA_EVENT_INVALID == eventHandle->vintrBitNum)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] VINTR bit alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate interrupt when callback is requested and only for
         * exclusive and master shared events (master handle is NULL_PTR) */
        if(((((Udma_EventCallback) NULL_PTR != eventPrms->eventCb) ||
                    (TRUE == eventPrms->osalRegisterDisable)) &&
                (NULL_PTR == eventPrms->masterEventHandle)) ||
            (UDMA_EVENT_TYPE_MASTER == eventPrms->eventType))
        {
            eventHandle->coreIntrNum =
                Udma_rmAllocCoreIntr(eventPrms->preferredCoreIntrNum, drvHandle);
            if(UDMA_INTR_INVALID == eventHandle->coreIntrNum)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] Core intr alloc failed!!!\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Do atomic link list update as the same is used in ISR */
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.disableAllIntr != (Udma_OsalDisableAllIntrFxn) NULL_PTR);
        cookie = drvHandle->initPrms.osalPrms.disableAllIntr();

        /* Link shared events to master event */
        eventHandle->prevEvent = (Udma_EventHandle) NULL_PTR;
        eventHandle->nextEvent = (Udma_EventHandle) NULL_PTR;
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            /* Go to the last node - insert node at the end */
            lastEvent = eventPrms->masterEventHandle;
            while(NULL_PTR != lastEvent->nextEvent)
            {
                /* Move to next node */
                lastEvent = lastEvent->nextEvent;
            }
            /* Cross link between last and current node */
            eventHandle->prevEvent = lastEvent;
            lastEvent->nextEvent   = eventHandle;
        }
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.restoreAllIntr != (Udma_OsalRestoreAllIntrFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.restoreAllIntr(cookie);
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
        {
            Udma_ChHandle chHandle;
            Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
            chHandle = eventPrms->chHandle;

            if(TRUE == chHandle->chOesAllocDone)
            {
                retVal = UDMA_EALLOC;
                Udma_printf(drvHandle, "[Error] Channel OES not de-allocated!!!\n");
            }
        }
    }

    if(UDMA_SOK != retVal)
    {
        Udma_eventFreeResource(drvHandle, eventHandle);
    }
    else
    {
        if(NULL_PTR == eventPrms->masterEventHandle)
        {
            vintrNum = eventHandle->vintrNum;
        }
        else
        {
            /* Use master event's info */
            vintrNum = eventPrms->masterEventHandle->vintrNum;
        }
        Udma_assert(drvHandle, drvHandle->iaRegs.pIntrRegs != NULL_PTR);
        eventHandle->pIaVintrRegs = &drvHandle->iaRegs.pIntrRegs->VINT[vintrNum];
    }

    return (retVal);
}

static void Udma_eventFreeResource(Udma_DrvHandle drvHandle,
                                   Udma_EventHandle eventHandle)
{
    uintptr_t   cookie;

    /* Do atomic link list update as the same is used in ISR */
    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.disableAllIntr != (Udma_OsalDisableAllIntrFxn) NULL_PTR);
    cookie = drvHandle->initPrms.osalPrms.disableAllIntr();

    /*
     * Remove this event node - link previous to next
     * Note: This is applicable only for shared mode. But the pointers will
     * be NULL_PTR for exclusive mode. Hence the logic is same.
     */
    /* Link previous's next to current's next */
    if(NULL_PTR != eventHandle->prevEvent)
    {
        eventHandle->prevEvent->nextEvent = eventHandle->nextEvent;
    }
    /* Link next's previous to current's previous */
    if(NULL_PTR != eventHandle->nextEvent)
    {
        eventHandle->nextEvent->prevEvent = eventHandle->prevEvent;
    }

    Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.restoreAllIntr != (Udma_OsalRestoreAllIntrFxn) NULL_PTR);
    drvHandle->initPrms.osalPrms.restoreAllIntr(cookie);

    if(NULL_PTR != eventHandle->hwiHandle)
    {
        Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.unRegisterIntr != (Udma_OsalUnRegisterIntrFxn) NULL_PTR);
        drvHandle->initPrms.osalPrms.unRegisterIntr(eventHandle->hwiHandle);
        eventHandle->hwiHandle = NULL_PTR;
    }
    if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
    {
        Udma_rmFreeCoreIntr(eventHandle->coreIntrNum, drvHandle);
        eventHandle->coreIntrNum = UDMA_INTR_INVALID;
    }

    if(UDMA_EVENT_INVALID != eventHandle->globalEvent)
    {
        /* Reset steering */
        Udma_eventResetSteering(drvHandle, eventHandle);
        Udma_rmFreeEvent(eventHandle->globalEvent, drvHandle);
        eventHandle->globalEvent = UDMA_EVENT_INVALID;
    }
    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        Udma_rmFreeVintrBit(eventHandle->vintrBitNum, drvHandle, eventHandle);
        eventHandle->vintrBitNum = UDMA_EVENT_INVALID;
    }
    if(UDMA_EVENT_INVALID != eventHandle->vintrNum)
    {
        Udma_rmFreeVintr(eventHandle->vintrNum, drvHandle);
        eventHandle->vintrNum = UDMA_EVENT_INVALID;
    }

    return;
}

static int32_t Udma_eventConfig(Udma_DrvHandle drvHandle,
                                Udma_EventHandle eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            vintrNum, coreIntrNum;
    Udma_ChHandle       chHandle;
    Udma_RingHandle     ringHandle;
    Udma_RingMonHandle  monHandle;
    Udma_EventPrms     *eventPrms;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    Udma_assert(drvHandle, eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = 0U;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = 0U;
    rmIrqReq.dst_host_irq           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* Event is always allocated except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.global_event   = (uint16_t)Udma_eventGetId(eventHandle);
    }

    /* IR setup */
    if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
    {
        /* Route Virtual interrupt (VINT) to core interrupt */
        Udma_assert(drvHandle, eventHandle->vintrNum != UDMA_EVENT_INVALID);

        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.dst_id        = drvHandle->devIdCore;
        rmIrqReq.dst_host_irq  = (uint16_t)eventHandle->coreIntrNum;
    }

    /* Get master IA register number for slaves */
    if(NULL_PTR != eventHandle->eventPrms.masterEventHandle)
    {
        vintrNum = eventHandle->eventPrms.masterEventHandle->vintrNum;
    }
    else
    {
        /* For master use the own register number */
        vintrNum = eventHandle->vintrNum;
    }
    Udma_assert(drvHandle, vintrNum != UDMA_EVENT_INVALID);
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
    rmIrqReq.ia_id         = drvHandle->devIdIa;
    rmIrqReq.vint          = (uint16_t)vintrNum;

    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        Udma_assert(drvHandle,
            eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;
        rmIrqReq.vint_status_bit_index  = (uint8_t)eventHandle->vintrBitNum;
    }

    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType))
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;

        rmIrqReq.src_id = drvHandle->devIdRing;
        if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType)
        {
            Udma_assert(drvHandle, chHandle->cqRing != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->cqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->cqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
        else
        {
            Udma_assert(drvHandle, chHandle->tdCqRing != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->tdCqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
    }

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;
        rmIrqReq.src_id = drvHandle->devIdUdma;
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)chHandle->rxChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START;
        }
        else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START;
        }
        else
        {
            /* DMSC RM doesn't program the DRU OES - program locally for now
             * in Udma_eventProgramSteering() */
            /* Use a SRC which doesn't need a OES programming so that DMSC will skip */
            rmIrqReq.src_id = drvHandle->devIdIa;
            rmIrqReq.src_index = 0U;                /* Not used by DMSC RM */
        }
    }

    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->ringHandle != NULL_PTR);
        ringHandle = eventPrms->ringHandle;
        Udma_assert(drvHandle, ringHandle->ringNum != UDMA_RING_INVALID);

        rmIrqReq.src_id     = drvHandle->devIdRing;
        rmIrqReq.src_index  = ringHandle->ringNum;
        rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
    }

    if(UDMA_EVENT_TYPE_RING_MON == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->monHandle != NULL_PTR);
        monHandle = eventPrms->monHandle;
        Udma_assert(drvHandle, monHandle->ringMonNum != UDMA_RING_MON_INVALID);

        rmIrqReq.src_id     = drvHandle->devIdRing;
        rmIrqReq.src_index  = monHandle->ringMonNum;
        rmIrqReq.src_index += TISCI_RINGACC0_MON_IRQ_SRC_IDX_START;
    }

    if(UDMA_EVENT_TYPE_ERR_OUT_OF_RANGE_FLOW == eventPrms->eventType)
    {
        rmIrqReq.src_id     = drvHandle->devIdUdma;
        rmIrqReq.src_index  = TISCI_UDMAP0_RX_FLOW_EOES_IRQ_SRC_IDX_START;
    }

    /* Program Output event steering based on event type */
    retVal = Udma_eventProgramSteering(drvHandle, eventHandle);
    if(UDMA_SOK != retVal)
    {
        Udma_printf(drvHandle, "[Error] OES program failed!!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        /* Config event */
        retVal = Sciclient_rmIrqSet(
                     &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
        if(CSL_PASS != retVal)
        {
            Udma_printf(drvHandle, "[Error] Sciclient event config failed!!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register after programming IA, so that when spurious interrupts
         * occur, we have a sane state/variables to handle it */
        if((UDMA_INTR_INVALID != eventHandle->coreIntrNum) &&
           (FALSE == eventHandle->eventPrms.osalRegisterDisable))
        {
            coreIntrNum = eventHandle->coreIntrNum;
#if defined (__C7100__)
            CSL_ClecEventConfig evtCfg;
            uint32_t            clecEvtNum;
            Udma_RmInitPrms    *rmInitPrms = &drvHandle->initPrms.rmInitPrms;

            /* CLEC programming required for C7x */
            coreIntrNum = eventHandle->coreIntrNum - rmInitPrms->startIrIntr;
            coreIntrNum += rmInitPrms->startC7xCoreIntr;
            evtCfg.secureClaimEnable = FALSE;
            evtCfg.evtSendEnable     = TRUE;
            evtCfg.rtMap             = drvHandle->clecRtMap;
            evtCfg.extEvtNum         = 0x00U;   /* Not used */
            evtCfg.c7xEvtNum         = coreIntrNum;
            clecEvtNum = eventHandle->coreIntrNum + drvHandle->clecOffset;
            CSL_clecConfigEvent(drvHandle->clecRegs, clecEvtNum, &evtCfg);
#endif

            /* Register interrupt only when asked for */
            Udma_assert(drvHandle, drvHandle->initPrms.osalPrms.registerIntr != (Udma_OsalRegisterIntrFxn) NULL_PTR);
            eventHandle->hwiHandle =
                drvHandle->initPrms.osalPrms.registerIntr(
                    &Udma_eventIsrFxn,
                    coreIntrNum,
                    eventHandle->eventPrms.intrPriority,
                    eventHandle);
            if(NULL_PTR == eventHandle->hwiHandle)
            {
                retVal = UDMA_EFAIL;
                Udma_printf(drvHandle,
                            "[Error] OSAL intr registration failed!!!\n");
            }
        }
    }

    return (retVal);
}

static int32_t Udma_eventReset(Udma_DrvHandle drvHandle,
                               Udma_EventHandle eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            vintrNum;
    Udma_ChHandle       chHandle;
    Udma_RingHandle     ringHandle;
    Udma_RingMonHandle  monHandle;
    Udma_EventPrms     *eventPrms;
    struct tisci_msg_rm_irq_release_req     rmIrqReq;

    Udma_assert(drvHandle, eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = 0U;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = 0U;
    rmIrqReq.dst_host_irq           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* Event is always allocated except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.global_event   = (uint16_t)Udma_eventGetId(eventHandle);
    }

    /* IR clear */
    if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
    {
        Udma_assert(drvHandle, eventHandle->vintrNum != UDMA_EVENT_INVALID);

        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.dst_id       = drvHandle->devIdCore;
        rmIrqReq.dst_host_irq = (uint16_t)eventHandle->coreIntrNum;
    }

    /* Get master IA register number for slaves */
    if(NULL_PTR != eventHandle->eventPrms.masterEventHandle)
    {
        vintrNum = eventHandle->eventPrms.masterEventHandle->vintrNum;
    }
    else
    {
        /* For master use the own register number */
        vintrNum = eventHandle->vintrNum;
    }
    Udma_assert(drvHandle, vintrNum != UDMA_EVENT_INVALID);
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
    rmIrqReq.ia_id         = drvHandle->devIdIa;
    rmIrqReq.vint          = (uint16_t)vintrNum;

    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        Udma_assert(drvHandle,
            eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;
        rmIrqReq.vint_status_bit_index  = (uint8_t)eventHandle->vintrBitNum;
    }

    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType))
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;

        rmIrqReq.src_id = drvHandle->devIdRing;
        if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType)
        {
            Udma_assert(drvHandle, chHandle->cqRing != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->cqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->cqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
        else
        {
            Udma_assert(drvHandle, chHandle->tdCqRing != NULL_PTR);
            Udma_assert(drvHandle,
                chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->tdCqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
    }

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;
        rmIrqReq.src_id = drvHandle->devIdUdma;
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
           ((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            Udma_assert(drvHandle, chHandle->rxChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)chHandle->rxChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_RX_OES_IRQ_SRC_IDX_START;
        }
        else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            Udma_assert(drvHandle, chHandle->txChNum != UDMA_DMA_CH_INVALID);
            rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
            rmIrqReq.src_index += TISCI_UDMAP0_TX_OES_IRQ_SRC_IDX_START;
        }
        else
        {
            /* DMSC RM doesn't program the DRU OES - program locally for now
             * in Udma_eventProgramSteering() */
            /* Use a SRC which doesn't need a OES programming so that DMSC will skip */
            rmIrqReq.src_id = drvHandle->devIdIa;
            rmIrqReq.src_index = 0U;                /* Not used by DMSC RM */
        }
    }

    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->ringHandle != NULL_PTR);
        ringHandle = eventPrms->ringHandle;
        Udma_assert(drvHandle, ringHandle->ringNum != UDMA_RING_INVALID);

        rmIrqReq.src_id     = drvHandle->devIdRing;
        rmIrqReq.src_index  = ringHandle->ringNum;
        rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
    }

    if(UDMA_EVENT_TYPE_RING_MON == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->monHandle != NULL_PTR);
        monHandle = eventPrms->monHandle;
        Udma_assert(drvHandle, monHandle->ringMonNum != UDMA_RING_MON_INVALID);

        rmIrqReq.src_id     = drvHandle->devIdRing;
        rmIrqReq.src_index  = monHandle->ringMonNum;
        rmIrqReq.src_index += TISCI_RINGACC0_MON_IRQ_SRC_IDX_START;
    }

    if(UDMA_EVENT_TYPE_ERR_OUT_OF_RANGE_FLOW == eventPrms->eventType)
    {
        rmIrqReq.src_id     = drvHandle->devIdUdma;
        rmIrqReq.src_index  = TISCI_UDMAP0_RX_FLOW_EOES_IRQ_SRC_IDX_START;
    }

    /* Reset event */
    retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
    if(CSL_PASS != retVal)
    {
        Udma_printf(drvHandle, "[Error] Sciclient event reset failed!!!\n");
    }

    return (retVal);
}

static int32_t Udma_eventProgramSteering(Udma_DrvHandle drvHandle,
                                         Udma_EventHandle eventHandle)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                evtNum;
    Udma_ChHandle           chHandle;
    Udma_EventPrms         *eventPrms;
    uint32_t                utcChNum;
    const Udma_UtcInstInfo *utcInfo;

    Udma_assert(drvHandle, eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;

        evtNum = Udma_eventGetId(eventHandle);
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
            ((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            /* Done by DMSC RM */
        }
        else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            /* Done by DMSC RM */
        }
        else
        {
            utcInfo = chHandle->utcInfo;
            Udma_assert(drvHandle, utcInfo != NULL_PTR);
            if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
            {
                Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
                Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
                utcChNum = chHandle->extChNum - utcInfo->startCh;

                retVal = CSL_druChSetEvent(utcInfo->druRegs, utcChNum, evtNum);
                if(CSL_PASS != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] DRU channel set event failed!!\n");
                }
            }
            else
            {
                retVal = UDMA_EFAIL;
                Udma_printf(drvHandle,
                    "[Error] TR events not possible in other external channels!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Mark OES alloc flag */
            chHandle->chOesAllocDone = TRUE;
        }
    }

    return (retVal);
}

static void Udma_eventResetSteering(Udma_DrvHandle drvHandle,
                                    Udma_EventHandle eventHandle)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                evtNum;
    Udma_ChHandle           chHandle;
    Udma_EventPrms         *eventPrms;
    uint32_t                utcChNum;
    const Udma_UtcInstInfo *utcInfo;

    Udma_assert(drvHandle, eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        Udma_assert(drvHandle, eventPrms->chHandle != NULL_PTR);
        chHandle = eventPrms->chHandle;

        evtNum = UDMA_EVENT_INVALID;
        if(((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY) ||
            ((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX))
        {
            /* Done in DMSC RM */
        }
        else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
        {
            /* Done in DMSC RM */
        }
        else
        {
            utcInfo = chHandle->utcInfo;
            Udma_assert(drvHandle, utcInfo != NULL_PTR);
            if(UDMA_UTC_TYPE_DRU == utcInfo->utcType)
            {
                Udma_assert(drvHandle, utcInfo->druRegs != NULL_PTR);
                Udma_assert(drvHandle, chHandle->extChNum >= utcInfo->startCh);
                utcChNum = chHandle->extChNum - utcInfo->startCh;

                retVal = CSL_druChSetEvent(utcInfo->druRegs, utcChNum, evtNum);
                if(CSL_PASS != retVal)
                {
                    Udma_printf(drvHandle,
                        "[Error] DRU channel set event failed!!\n");
                }
            }
            else
            {
                retVal = UDMA_EFAIL;
                Udma_printf(drvHandle,
                    "[Error] TR events not possible in other external channels!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Mark OES alloc flag */
            chHandle->chOesAllocDone = FALSE;
        }
    }

    return;
}
