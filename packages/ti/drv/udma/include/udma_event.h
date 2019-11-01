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
 *  \defgroup DRV_UDMA_EVENT_MODULE UDMA Driver Event API
 *            This is UDMA driver event related configuration parameters and
 *            API
 *
 *  @{
 */

/**
 *  \file udma_event.h
 *
 *  \brief UDMA event related parameters and API.
 */

#ifndef UDMA_EVENT_H_
#define UDMA_EVENT_H_

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

/** \brief Macro used to specify that event ID is invalid. */
#define UDMA_EVENT_INVALID              ((uint32_t) CSL_UDMAP_NO_EVENT)
/** \brief Macro used to specify that interrupt number is invalid. */
#define UDMA_INTR_INVALID               ((uint32_t) 0xFFFF0000U)
/**
 * \brief Macro used to specify any available free core interrupt while
 * requesting one. Used in the API #Udma_eventRegister.
 */
#define UDMA_CORE_INTR_ANY              ((uint32_t) 0xFFFF0001U)

/** \brief Max events per IA VINTR */
#define UDMA_MAX_EVENTS_PER_VINTR       (64U)

/**
 *  \anchor Udma_EventType
 *  \name UDMA Event Type
 *
 *  UDMA events supported.
 *
 *  @{
 */
/**
 *  \brief DMA completion event.
 *  Incase of TX/RX channel usage through ring, this represents the
 *  completion queue ring event and the application can dequeue the descriptor
 *  post this event.
 */
#define UDMA_EVENT_TYPE_DMA_COMPLETION          ((uint32_t) 0x0001U)
/**
 *  \brief DMA teardown completion event.
 *  Incase of TX/RX channel usage through ring, this represents the
 *  in-complete descriptor queued to the TD CQ ring during teardown operation.
 *  Note: This doesn't represent teardown completion of the channel.
 */
#define UDMA_EVENT_TYPE_TEARDOWN_PACKET         ((uint32_t) 0x0002U)
/**
 *  \brief TR event to IA.
 *
 *  This programs the channels event steering register with IA global event
 *  number to generate anytime the required event generation criteria
 *  specified in a TR are met.
 *  This can be used to get intermediate event or interrupt based on the
 *  event type programmed in the TR.
 *
 *  In case of TX and RX channel, this programs the correspinding UDMAP
 *  channel OES register.
 *  In case of blockcopy, this programs the UDMAP RX channel OES register.
 *  In case of external DRU channel, this programs the DRU OES register.
 */
#define UDMA_EVENT_TYPE_TR                      ((uint32_t) 0x0003U)
/**
 *  \brief Ring event used for getting callback when entries are there to be
 *  popped from ring.
 *  This is added to support usecases where the user can independently allocate
 *  and configure ring and requires callback when hardware occupancy in the
 *  ring goes to non-zero.
 *  Note: This is not tied to any channel handle. And hence the chHandle in
 *  the event params can be set to NULL (Ignored by driver)
 *
 *  Caution: Ring event will be triggered only when a transition from empty
 *  to non-empty ring occupancy occurs. Subsequent increment in ring occupancy
 *  will not trigger an event/interrupt. The user should take care of this
 *  behavior when dealing with multiple entires in a ring i.e. when a callback
 *  occurs, the user should dequeue as much as possible till the dequeue returns
 *  UDMA_ETIMEOUT and should not assume multiple callbacks will occur for
 *  each ring element (push from HW/SW).
 */
#define UDMA_EVENT_TYPE_RING                    ((uint32_t) 0x0004U)
/**
 *  \brief Event type used to register master event without providing
 *  source type like ring, DMA etc... This event type can be used to register
 *  the master event which reserves the IA and IR interrupt to a core
 *  without reserving global event ID and IMAP programming.
 *  Post this, the handle can be passed to masterEventHandle for other event
 *  registeration to share the same IA and IR.
 */
#define UDMA_EVENT_TYPE_MASTER                  ((uint32_t) 0x0005U)
/**
 *  \brief Event type used to register an event for trapping an out of range
 *  flow ID received on a packet.
 *
 *  Note: This is a global event per UDMA instance (Main and MCU separately)
 *  and is common for all flow/channel in an UDMA instance. Hence this should
 *  be registered only once per system (by a master core) to handle flow error
 *  events.
 */
#define UDMA_EVENT_TYPE_ERR_OUT_OF_RANGE_FLOW   ((uint32_t) 0x0006U)
/**
 *  \brief Ring monitor event used for getting callback when ring monitor
 *  event criteria is met.
 *
 *  Note: Not all modes of ring monitor generate events. Only the mode
 *  TISCI_MSG_VALUE_RM_MON_MODE_THRESHOLD generate event based on the
 *  low and high threshold programmed. In this case, the same event gets
 *  generated when both low and high thresholds are crossed.
 *
 *  When user want to get only the low threshold event, the high threshold
 *  can be programmed a value which is greater than than the ring element count
 *  When user want to get only the high threshold event, the low threshold
 *  can be programmed zero.
 *
 */
#define UDMA_EVENT_TYPE_RING_MON                ((uint32_t) 0x0007U)
/* @} */

/**
 *  \anchor Udma_EventMode
 *  \name UDMA Event Mode
 *
 *  UDMA event mode.
 *
 *  @{
 */
/** \brief Event is exclusively allocated at Interrupt Aggregator */
#define UDMA_EVENT_MODE_EXCLUSIVE       ((uint32_t) 0x0001U)
/**
 *  \brief Event is shared at Interrupt Aggregator and could be shared with
 *  any other events.
 */
#define UDMA_EVENT_MODE_SHARED          ((uint32_t) 0x0002U)
/* @} */

/**
 *  \brief UDMA event callback function.
 *
 *  \param eventHandle  [IN] UDMA event handle
 *  \param eventType    [IN] Event that occurred
 *  \param appData      [IN] Callback pointer passed during event register
 */
typedef void (*Udma_EventCallback)(Udma_EventHandle eventHandle,
                                   uint32_t eventType,
                                   void *appData);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA event related parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2628), DOX_REQ_TAG(PDK-2627)
 *               DOX_REQ_TAG(PDK-2626), DOX_REQ_TAG(PDK-2625)
 */
typedef struct
{
    uint32_t                eventType;
    /**< [IN] Event type to register. Refer \ref Udma_EventType */
    uint32_t                eventMode;
    /**< [IN] Event mode - exclusive or shared. Refer \ref Udma_EventMode.
     *   This parameter should be set to #UDMA_EVENT_MODE_SHARED for
     *   #UDMA_EVENT_TYPE_MASTER event type. */
    Udma_ChHandle           chHandle;
    /**< [IN] Channel handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_DMA_COMPLETION
     *          - #UDMA_EVENT_TYPE_TEARDOWN_PACKET
     *          - #UDMA_EVENT_TYPE_TR.
     *   This parameter can be NULL for other types. */
    Udma_RingHandle         ringHandle;
    /**< [IN] Ring handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_RING
     *   This parameter can be NULL for other types. */
    Udma_EventHandle        masterEventHandle;
    /**< [IN] Master event handle used to share the IA register when the event
     *   mode is set to #UDMA_EVENT_MODE_SHARED.
     *   This is typically used to share multiple events from same source
     *   like same peripheral to one IA register which eventually routes to
     *   a single core interrupt.
     *   For the first(or master) event this should be set to NULL. The driver
     *   will allocate the required resources (IA/IR) for the first event.
     *   For the subsequent shared event registration, the master event handle
     *   should be passed as reference and the driver will allocate only the
     *   IA status bits. At a maximum #UDMA_MAX_EVENTS_PER_VINTR number of
     *   events can be shared. Beyond that the driver will return error.
     *   This parameter should be set to NULL for #UDMA_EVENT_TYPE_MASTER
     *   event type. */
    Udma_EventCallback      eventCb;
    /**< [IN] When callback function is set (non-NULL), the driver will allocate
     *   core level interrupt through Interrupt Router and the function
     *   will be called when the registered event occurs.
     *   When set to NULL, the API will only allocate event and no interrupt
     *   routing is performed.
     *   Note: In case of shared events (multiple events mapped to same
     *   interrupt), the driver will call the callbacks in the order
     *   of event registration.
     *   This parameter should be set to NULL for #UDMA_EVENT_TYPE_MASTER
     *   event type. */
    uint32_t                intrPriority;
    /**< [IN] Priority of interrupt to register with OSAL. The interpretation
     *   depends on the OSAL implementation */
    void                   *appData;
    /**< [IN] Application/caller context pointer passed back in the event
     *    callback function. This could be used by the caller to identify
     *    the channel/event for which the callback is called.
     *    This can be set to NULL, if not required by caller. */
    uint32_t                osalRegisterDisable;
    /**< [IN] This flag is used to control whether the the interrupt
     *   needs to be registered with OSAL. In case AUTOSAR MCAL, interrupt
     *   registration is done by the integration layer at the start
     *   of the system init. Hence this flag is provided.
     *
     *   Note: The osal interrupt registration is expected only for the master
     *   event.
     *
     *   eventCb parameter can be NULL when this flag is set to TRUE as no
     *   interrupt registration is done
     *
     *      TRUE    - Disable osal registration
     *      FALSE   - Enable osal registration. This should be used by all
     *                TI-RTOS application.
     */
    uint32_t                preferredCoreIntrNum;
    /**< [IN] Preferred core interrupt number which goes to a core.
     *   If set to #UDMA_CORE_INTR_ANY, will allocate from free pool.
     *   Else will try to allocate the mentioned interrupt itself. */
    Udma_RingMonHandle      monHandle;
    /**< [IN] Ring monitor handle when the event type is one of below
     *          - #UDMA_EVENT_TYPE_RING_MON
     *   This parameter can be NULL for other types. */

    /*
     * Output parameters
     */
    volatile uint64_t      *intrStatusReg;
    /**< [OUT] Interrupt status register address of the allocated IA VINT
     *   register. This is used to check if interrupt occurred */
    volatile uint64_t      *intrClearReg;
    /**< [OUT] Interrupt clear register address of the allocated IA VINT
     *   register. This is used to clear if interrupt occurred */
    uint64_t                intrMask;
    /**< [OUT] Interrupt mask to check and clear */
    uint32_t                coreIntrNum;
    /**< [OUT] Core interrupt number allocated.
     *   This number can be used to register with the OSAL */
} Udma_EventPrms;

/**
 *  \brief UDMAP receive flow id firewall status
 *
 *  This structure contains status information collected whenever the receive
 *  flow ID firewall detects a flow ID that is out of range for an incoming
 *  packet.
 */
typedef struct
{
    uint32_t    isException;
    /**< This is set whenever the Flow ID firewall detects a Flow ID is out of
     *   range for an incoming packet. */
    uint32_t    flowId;
    /**< [OUT] The flow ID that was received on the trapped packet */
    uint32_t    chNum;
    /**< [OUT] The channel index on which the trapped packet was received */
} Udma_EventRxFlowIdFwStatus;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief UDMA event registration.
 *
 *  Register event based on UDMA channel based and event parameters.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2596)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *  \param eventHandle  [IN/OUT] UDMA event handle. The caller need to
 *                           allocate memory for this object and pass this
 *                           pointer to all further APIs. The caller should
 *                           not change any parameters as this is owned and
 *                           maintained by the driver.
 *  \param eventPrms    [IN] UDMA event parameters.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_eventRegister(Udma_DrvHandle drvHandle,
                           Udma_EventHandle eventHandle,
                           Udma_EventPrms *eventPrms);

/**
 *  \brief UDMA unregister event.
 *
 *  Unregister the event and frees all associated resources.
 *
 *  Note: In case of shared event, the master event should be unregistered last
 *  compared to other shared events since the resource is owned by the master
 *  event. This function returns error for master event if any other shared
 *  resource is still not unregistered.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2597)
 *
 *  \param eventHandle  [IN] UDMA event handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_eventUnRegister(Udma_EventHandle eventHandle);

/**
 *  \brief Returns the event ID allocated for this event.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2598)
 *
 *  \param eventHandle  [IN] UDMA event handle.
 *                           This parameter can't be NULL.
 *
 *  \return the event ID on success or #UDMA_EVENT_INVALID on error
 */
uint32_t Udma_eventGetId(Udma_EventHandle eventHandle);

/**
 *  \brief Disable the event at interrupt aggregator
 *
 *  Requirement: DOX_REQ_TAG(PDK-3583)
 *
 *  \param eventHandle  [IN] UDMA event handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_eventDisable(Udma_EventHandle eventHandle);

/**
 *  \brief Enable the event at interrupt aggregator
 *
 *  Note: By default the event will be enabled at the time of registration.
 *        This is API is used to enable the event again after a call to
 *        #Udma_eventDisable API
 *
 *  Requirement: DOX_REQ_TAG(PDK-3583)
 *
 *  \param eventHandle  [IN] UDMA event handle.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_eventEnable(Udma_EventHandle eventHandle);

/**
 *  \brief Get the global event handle of the driver handle.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2621)
 *
 *  \param drvHandle    [IN] UDMA driver handle pointer passed during
 *                           #Udma_init
 *
 *  \return Returns global event handle else NULL on error
 */
Udma_EventHandle Udma_eventGetGlobalHandle(Udma_DrvHandle drvHandle);

/**
 *  \brief Get the UDMA flow ID firewall status. This API will clear the status
 *  bit (RFLOWFWSTAT) by calling TISCI API
 *
 *  Requirement: DOX_REQ_TAG(PDK-3706)
 *
 *  \param eventHandle  [IN] UDMA event handle.
 *                           This parameter can't be NULL.
 *  \param status       [OUT] RX flow ID firewall status return structure.
 *                           This parameter can't be NULL.
 *
 *  \return \ref Udma_ErrorCodes
 */
int32_t Udma_eventGetRxFlowIdFwStatus(Udma_EventHandle eventHandle,
                                      Udma_EventRxFlowIdFwStatus *status);

/*
 * Structure Init functions
 */
/**
 *  \brief Udma_EventPrms structure init function.
 *
 *  \param eventPrms    [IN] Pointer to #Udma_EventPrms structure.
 *
 */
void UdmaEventPrms_init(Udma_EventPrms *eventPrms);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                  Internal/Private Structure Declarations                   */
/* ========================================================================== */

/**
 *  \brief UDMA event object.
 *
 *  Note: This is an internal/private driver structure and should not be
 *  used or modified by caller.
 */
struct Udma_EventObj
{
    Udma_DrvHandle          drvHandle;
    /**< Pointer to global driver handle. */
    Udma_EventPrms          eventPrms;
    /**< Event parameters passed during event registeration. */

    uint32_t                globalEvent;
    /**< Allocated IA global event. */
    uint32_t                vintrNum;
    /**< Allocated IA VINT register. */
    uint32_t                vintrBitNum;
    /**< Allocated IA VINT bit number - 0 to 63. */
    uint32_t                coreIntrNum;
    /**< Allocated core interrupt number. */

    Udma_EventHandle        nextEvent;
    /**< Pointer to next event - used in shared event for traversing in ISR */
    Udma_EventHandle        prevEvent;
    /**< Pointer to previous event - used in shared event for traversing during
     *   event un-registration */

    HwiP_Handle             hwiHandle;
    /**< HWI handle. */
    uint64_t                vintrBitAllocFlag;
    /**< For master event, this stores the alloc flag for each bit within
     *   IA register. This is not used for slave events and is always set to
     *   zero */

    /* Below register overlay pointers provided for debug purpose to
     * readily view the registers */
    volatile CSL_intaggr_imapRegs_gevi  *pIaGeviRegs;
    /**< Pointer to IA global event register overlay */
    volatile CSL_intaggr_intrRegs_vint  *pIaVintrRegs;
    /**< Pointer to IA virtual interrupt register overlay */

    uint32_t                eventInitDone;
    /**< Flag to set the event object is init. */
};

#ifdef __cplusplus
}
#endif

#endif /* #ifndef UDMA_EVENT_H_ */

/* @} */
