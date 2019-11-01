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
 *  \file cal_evtmgr.c
 *
 *  \brief File containing the CAL event manager
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>
#include <ti/csl/hw_types.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr_cal.h>
#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/core/cal_evtmgr.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum number of clients for Event Manager. */
#define CAL_EM_MAX_CLIENTS                  (10U)
#define CAL_EM_MAX_CLIENT_EVENTS            (10U)

/**
 *  \brief Flags used by clientinfo structures memory pool manager to
 *  indicate availability of pool memory
 */
#define CAL_EM_MEM_FLAG_FREE                (0xFFFFFFFFU)
#define CAL_EM_MEM_FLAG_ALLOC               (0x00000000U)

/**
 *  \brief Interrupt max level.
 *          0 Used for CAL top level interrupt (enable/disable/status/clear)
 *          1 Used module specific interrupt (enable/disable/status/clear)
 *          For some cases (such as CAL errors) there could additional level
 *          2 Used for low level errors / warning (such as CRC, ECC, FS, etc...)
 */
#define CAL_EM_MAX_LEVELS                  (3U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief structure containing instance specific information, there should be
 *         one instance object for each independent irq
 */
typedef struct Cal_EmInstObj
{
    uint32_t           instId;
    /**< IEM Instance ID. */
    uint32_t           irqNumber;
    /**< IRQ number at CPU level INTC. */
    uint32_t           eoiAddr;
    /**< Address of EOI register. */
    Fvid2Utils_Node     *headNode;
    /**< Pointer to the head node of the priority linked list. */
    HwiP_Handle         intrHandle;
    /**< Handle to Hardware Interrupt. */
    uint32_t           numIntr;
    /**< Total number of interrupts occured since last init. */
    Fvid2Utils_TsPrfLog  isrPrfLog;
    /**< ISR duration performance log. */
    Cal_EmInitParams_t   initPrms;
    /**< Init Parameters */
} Cal_EmInstObj_t;

typedef struct Cal_EmClientInfo
{
    uint32_t          statusReg[CAL_EM_MAX_CLIENT_EVENTS][CAL_EM_MAX_LEVELS];
    /**< Address of the status register for multiple levels,
     *   Level0 provides the actual IRQ,
     *   status is checked at the last level,
     *   status is cleared from level0 to last level in sequence */
    uint32_t          statusClrReg[CAL_EM_MAX_CLIENT_EVENTS][CAL_EM_MAX_LEVELS];
    /**< Address of the status clear register */
    uint32_t          mask[CAL_EM_MAX_CLIENT_EVENTS][CAL_EM_MAX_LEVELS];
    /**< Bitmask for checking and clearing the status of the event/irq */
    Bool              isFirstLevelReq[CAL_EM_MAX_CLIENT_EVENTS];
    /**< Flag to indicate if first level of interrupt handling is required. */
    Bool              isThirdLevelReq[CAL_EM_MAX_CLIENT_EVENTS];
    /**< Flag to indicate if third level of interrupt handling is required. */
    Cal_EmClientCbFxn  clientCb;
    /**< Client's callback function. This function will be called by the
     *   event manager whenever client registered event occurs */
    void             *arg;
    /**< client's private data to be passed to it when callback function is
     *   called */
    Cal_EmEventGroup_t eventGroup;
    /**< This tells the event group into which registered event belongs */
    uint32_t          numIntr;
    /**< Number of interrupts occurred for this client */
    Cal_EmPriority_t   priority;
    /**< Priority of these events */
    uint32_t          subModuleEoiAddr[CAL_EM_MAX_CLIENT_EVENTS];
    /**< Need to see if we could set this register */
    uint32_t          subModuleEoiLine[CAL_EM_MAX_CLIENT_EVENTS];
    /**< IRQ Id */
    Cal_EmInstObj_t   *instObj;
    /**< Pointer to the Instance Object */
    uint32_t          allEvents[CAL_EM_MAX_CLIENT_EVENTS];
    /**< List of events for which client has registered callback */
    uint32_t          setEvents[CAL_EM_MAX_CLIENT_EVENTS];
    /**< This array contains the list of events occurred at the time of
     *   interrupt. It will be passed down to the clients.*/
    uint32_t          numEvents;
    /**< Count of events for client has registered callback. */
} Cal_EmClientInfo_t;

/**
 *  \brief structure to keep track of commonly used variables in event manager.
 */
typedef struct Cal_EmCommonObj
{
    uint32_t          isInitDone;
    /**< Flag to indicate that Event Manager is initialized or not */
    SemaphoreP_Handle lockSem;
    /**< Semaphore handle. */
    Cal_EmClientInfo_t clientMemPool[CAL_EM_MAX_CLIENTS];
    /**< Pool of memory for the clientinfo structure. */
    uint32_t          clientMemFlag[CAL_EM_MAX_CLIENTS];
    /**< Flags for each instance to indicate whether it is free or allocated. */
    Fvid2Utils_Node     nodePool[CAL_EM_MAX_CLIENTS];
    /**< Pool of memory for the node. */
} Cal_EmCommonObj_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t CalEmFillClientInfo(Cal_EmClientInfo_t *clientInfo,
                                   Cal_EmEventGroup_t  eventGroup,
                                   const uint32_t    *event,
                                   uint32_t           numEvents,
                                   Cal_EmPriority_t    priority,
                                   Cal_EmClientCbFxn   callback,
                                   void              *arg);

static void CalEmDeleteClientInfo(const Cal_EmClientInfo_t *clientInfo);

static Cal_EmClientInfo_t *CalEmCreateClientInfo(Fvid2Utils_Node **node);

static void CalEmDeleteNode(const Fvid2Utils_Node *emNode);

static void CalEmAddNode(Fvid2Utils_Node *newNode, Cal_EmPriority_t priority);

static void CalEmMasterIsr(uintptr_t arg);

static Cal_EmInstObj_t *CalEmGetInstObj(Cal_EmInstId_t instId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/**
 *  \brief Global Cal Em Initialization Data
 */
static Cal_EmPlatformData_t gCalEmPlatData =
{
    1U,
    {
        {
            CAL_EM_INST_ID_CAL0,
            184,
            {CSL_CAL0_BASE}
        },
    }
};

/**
 *  \brief Global Instance objects
 */
static Cal_EmInstObj_t   gCalEmInstObj[CAL_EM_MAX_INST] = {FALSE};

/**
 *  \brief Global Common object
 */
static Cal_EmCommonObj_t gCalEmCommonObj = {FALSE};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

Cal_EmPlatformData_t *Cal_getEmPlatformData(void)
{
    return (&gCalEmPlatData);
}

void Cal_initEmPlatformData(UInt32 irqNum)
{
    /* ISS driver registers only one interrupt out of 6.
     * So replacing in first element. */
    gCalEmPlatData.calEmInitParam[0].irqNum = irqNum;
}


/**
 *  \brief Initializes the Cal Event Manager. It marks all the client info
 *  instances in the memory as available and register interrupt handler
 */
int32_t Cal_emInit(uint32_t numInst, const Cal_EmInitParams_t *initPrms)
{
    int32_t         retVal = FVID2_SOK;
    uint32_t        cnt;
    uint32_t        instCnt;
    Cal_EmInstObj_t *instObj;
    SemaphoreP_Params params;
    HwiP_Params hwiParams;

    /* Check for errors */
    GT_assert(CalTrace, (NULL != initPrms));
    GT_assert(CalTrace, (numInst <= CAL_EM_MAX_INST));
    GT_assert(CalTrace, (numInst != 0U));

    if ((uint32_t)FALSE == gCalEmCommonObj.isInitDone)
    {
        /* Init objects and mark flags as free */
        memset(gCalEmInstObj, 0, sizeof (gCalEmInstObj));
        memset(&gCalEmCommonObj, 0, sizeof (gCalEmCommonObj));

        for (instCnt = 0U; instCnt < CAL_EM_MAX_INST; instCnt++)
        {
            gCalEmInstObj[instCnt].instId = CAL_EM_INST_ID_MAX;
            Fvid2Utils_initTsPrfLog(&gCalEmInstObj[instCnt].isrPrfLog);
        }
        for (cnt = 0U; cnt < CAL_EM_MAX_CLIENTS; cnt++)
        {
            gCalEmCommonObj.clientMemFlag[cnt] = CAL_EM_MEM_FLAG_FREE;
        }

        /* Initialise the semaphore parameters and create Pool Semaphore */
        SemaphoreP_Params_init(&params);
        params.mode = SemaphoreP_Mode_BINARY;
        gCalEmCommonObj.lockSem = SemaphoreP_create(1U, &params);
        if (NULL == gCalEmCommonObj.lockSem)
        {
            retVal = FVID2_EALLOC;
        }

        if (FVID2_SOK == retVal)
        {
            for (instCnt = 0U; instCnt < numInst; instCnt ++)
            {
                /* Initialize instance object members */
                instObj             = &gCalEmInstObj[instCnt];
                instObj->instId     = initPrms->instId;
                instObj->irqNumber  = initPrms->irqNum;

                memcpy(&instObj->initPrms, initPrms,
                    sizeof(Cal_EmInitParams_t));

                /* Clear out any pending interrupts */
                HwiP_clearInterrupt(instObj->irqNumber);
 
                instObj->headNode = NULL;
                instObj->numIntr  = 0U;

                instObj->eoiAddr =
                    initPrms->baseAddress[instCnt] +
                    CSL_CAL_C2CTL_HL_IRQ_EOI;

                /* ----------------- Register ISR --------------- */
                HwiP_Params_init(&hwiParams);
                hwiParams.arg = (uintptr_t) instObj;
                instObj->intrHandle = HwiP_create(
                    instObj->irqNumber,
                    &CalEmMasterIsr,
                    &hwiParams);
                if (NULL == instObj->intrHandle)
                {
                    GT_1trace(CalTrace, GT_ERR,
                              "%s: Cannot Register IRQ", __FUNCTION__);
                    retVal = FVID2_EALLOC;
                    break;
                }

                initPrms ++;
            }
        }

        if (retVal != FVID2_SOK)
        {
            Cal_emDeInit();
        }
        else
        {
            gCalEmCommonObj.isInitDone = (uint32_t)TRUE;
        }
    }

    return (retVal);
}

/**
 *  \brief De-Initializes event Manager. It deletes semaphore and un-registers
 *  the IRQ handle and removes all the client info instance from the linked
 *  list.
 */
int32_t Cal_emDeInit(void)
{
    uint32_t        instCnt;
    Cal_EmInstObj_t *instObj;
    Fvid2Utils_Node  *tempNode;
    Fvid2Utils_Node  *emNode;

    for (instCnt = 0U; instCnt < CAL_EM_MAX_INST; instCnt++)
    {
        instObj = &gCalEmInstObj[instCnt];

        if (instObj->intrHandle != NULL)
        {
            /* ------------- Un-register ISR ------------- */
            HwiP_delete(instObj->intrHandle);
            instObj->intrHandle = NULL;
        }

        /* Free Entire Linked List */
        tempNode = instObj->headNode;
        while (NULL != tempNode)
        {
            emNode   = tempNode;
            tempNode = tempNode->next;
            CalEmDeleteClientInfo((Cal_EmClientInfo_t *) emNode->data);
            CalEmDeleteNode(emNode);
        }
    }

    if (gCalEmCommonObj.lockSem != NULL)
    {
        SemaphoreP_delete(gCalEmCommonObj.lockSem);
        gCalEmCommonObj.lockSem = NULL;
    }

    gCalEmCommonObj.isInitDone = (uint32_t)FALSE;

    return (FVID2_SOK);
}

/**
 *  \brief This function is used to register call back function for a specific
 *  event. It takes group number and event number within the group as the
 *  argument and returns handle.
 */
void *Cal_emRegister(Cal_EmInstId_t     instId,
                        Cal_EmEventGroup_t eventGroup,
                        const uint32_t   *event,
                        uint32_t          numEvents,
                        Cal_EmPriority_t   priority,
                        Cal_EmClientCbFxn  callback,
                        void             *arg)
{
    int32_t status = FVID2_SOK;
    Cal_EmInstObj_t    *instObj;
    Fvid2Utils_Node     *newNode    = NULL;
    Cal_EmClientInfo_t *clientInfo = NULL;

    /* Check for the error conditions */
    GT_assert(CalTrace, (NULL != event));
    GT_assert(CalTrace, (0U != numEvents));
    GT_assert(CalTrace, (NULL != callback));

    if ((uint32_t)TRUE != gCalEmCommonObj.isInitDone)
    {
        status = FVID2_EBADARGS;
    }

    if (FVID2_SOK == status)
    {
        SemaphoreP_pend(gCalEmCommonObj.lockSem, SemaphoreP_WAIT_FOREVER);

        instObj = CalEmGetInstObj(instId);
        GT_assert(CalTrace, (NULL != instObj));

        /* Allocate memory for the client info structure */
        clientInfo = CalEmCreateClientInfo(&newNode);

        if ((NULL != clientInfo) && (NULL != newNode))
        {
            newNode->data = (void *) clientInfo;

            clientInfo->instObj = instObj;
            clientInfo->numIntr = 0U;

            /* Fill up the details about events in the client info structure */
            CalEmFillClientInfo(
                clientInfo,
                eventGroup,
                event,
                numEvents,
                priority,
                callback,
                arg);

            /* Add the node in the linked list as per the priority */
            CalEmAddNode(newNode, priority);
        }

        SemaphoreP_post(gCalEmCommonObj.lockSem);
    }

    return ((void *) newNode);
}

/**
 *  \brief  Function to un-register callback from the event manager. It removes
 *          node from the linked list and frees up memory allocated to the node
 *          and client info structure. It takes handle to the callback and
 *          returns status of the whether callback is removed.
 *          must be valid handle to the callback already registered
 */
int32_t Cal_emUnRegister(void *handle)
{
    uint32_t           found = FALSE;
    uint32_t           cnt, regAddr, event, regVal;
    uint32_t           calIrqNum, calEvent;
    Cal_EmEventGroup_t  eventGroup;
    Cal_EmInstObj_t    *instObj    = NULL;
    Fvid2Utils_Node     *tempNode   = NULL;
    Fvid2Utils_Node     *emNode     = NULL;
    Cal_EmClientInfo_t *clientInfo = NULL;
    Cal_EmInitParams_t *initPrms   = NULL;

    /* Error condition Checking */
    GT_assert(CalTrace, (NULL != handle));
    tempNode = (Fvid2Utils_Node *) handle;
    GT_assert(CalTrace, (NULL != tempNode->data));
    instObj = ((Cal_EmClientInfo_t *) (tempNode->data))->instObj;
    GT_assert(CalTrace, (NULL != instObj));
    initPrms = &instObj->initPrms;

    /* Check to see if the node given is in the linked list */
    emNode = instObj->headNode;
    GT_assert(CalTrace, (NULL != emNode));

    found = (uint32_t) FALSE;
    while (NULL != emNode)
    {
        if (tempNode == emNode)
        {
            found = (uint32_t) TRUE;
            break;
        }
        emNode = emNode->next;
    }
    GT_assert(CalTrace, (FALSE != found));

    clientInfo = (Cal_EmClientInfo_t *) tempNode->data;
    GT_assert(CalTrace, (NULL != clientInfo));

    SemaphoreP_pend(gCalEmCommonObj.lockSem, SemaphoreP_WAIT_FOREVER);

    /* Delete the Node */
    CalEmDeleteNode(tempNode);

    /* Disable events in the CAL registers */
    eventGroup = clientInfo->eventGroup;
    for (cnt = 0U; cnt < clientInfo->numEvents; cnt++)
    {
        event = clientInfo->allEvents[cnt];

        if (CAL_EM_EG_CAL == eventGroup)
        {
            if (CAL_EM_EVENT_LL_SOF4 <= event)
            {
                calIrqNum = 4U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF4;
            }
            else if (CAL_EM_EVENT_LL_SOF0 <= event)
            {
                calIrqNum = 3U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF0;
            }
            else if (CAL_EM_EVENT_WDMA_START0 <= event)
            {
                calIrqNum = 2U;
                calEvent  = event - CAL_EM_EVENT_WDMA_START0;
            }
            else if (CAL_EM_EVENT_WDMA_END0 <= event)
            {
                calIrqNum = 1U;
                calEvent  = event - CAL_EM_EVENT_WDMA_END0;
            }
            else
            {
                calIrqNum = 0U;
                calEvent  = event;
            }

            /* Disable the interrupts at the CAL level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                      CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(calIrqNum);
            HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);
        }
        else if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
                 (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
        {
            if (CAL_CSI2_PPI_CMPLXIO_RSERVED_31 > event)
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1;
                }
                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);
                clientInfo->isThirdLevelReq[cnt] = FALSE;

                /* Disable the interrupt and clear status */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal &= ~((uint32_t) 1 << event);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);

            }
            else if ((CAL_CSI2_PPI_VC_SOF1 <= event) &&
                   (CAL_CSI2_PPI_VC_RESERVED_31 > event))
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO_VC;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1_VC;
                }

                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);
                clientInfo->isThirdLevelReq[cnt] = FALSE;

                calEvent = event - CAL_CSI2_PPI_VC_SOF1;

                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_VC_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal &= ~((uint32_t) 1 << calEvent);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);
            }
            else
            {
                GT_assert(CalTrace, FALSE);
            }
        }
#if defined(NOT_YET)
        else if ((CAL_EM_EG_LVDSRX0 <= eventGroup) &&
                 (CAL_EM_EG_LVDSRX3 >= eventGroup))
        {
            /* Disable the interrupts at the SIMCOP level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                      LVDSRX_IRQENABLE_CLR_0;
            regAddr = regAddr +
                      ((LVDSRX_IRQENABLE_CLR_1 -
                        LVDSRX_IRQENABLE_CLR_0) *
                       (eventGroup - CAL_EM_EG_LVDSRX0));
            HW_WR_REG32(regAddr, ((uint32_t) 1 << event));

            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);
        }
#endif
        else
        {
            GT_assert(CalTrace, FALSE);
        }

        clientInfo->statusReg[cnt][0U]    = 0x0U;
        clientInfo->statusClrReg[cnt][0U] = 0x0U;
        clientInfo->mask[cnt][0U]         = 0x0U;
        clientInfo->statusReg[cnt][1U]    = 0x0U;
        clientInfo->statusClrReg[cnt][1U] = 0x0U;
        clientInfo->mask[cnt][1U]         = 0x0U;
        clientInfo->statusReg[cnt][2U]    = 0x0U;
        clientInfo->statusClrReg[cnt][2U] = 0x0U;
        clientInfo->mask[cnt][2U]         = 0x0U;

        clientInfo->subModuleEoiAddr[cnt] = 0x0U;
        clientInfo->subModuleEoiLine[cnt] = 0x0U;
        clientInfo->isThirdLevelReq[cnt]  = FALSE;
        clientInfo->isFirstLevelReq[cnt]  = FALSE;
    }

    clientInfo->clientCb  = NULL;
    clientInfo->arg       = NULL;
    clientInfo->instObj   = NULL;
    clientInfo->numEvents = 0;

    /* Free up memory allocated to event arrays and clientInfo structure */
    CalEmDeleteClientInfo(clientInfo);

    SemaphoreP_post(gCalEmCommonObj.lockSem);

    return (FVID2_SOK);
}

int32_t Cal_emDisable(void *handle)
{
    uint32_t           found = FALSE;
    uint32_t           cnt, regAddr, regVal;
    uint32_t           eventGroup, event;
    uint32_t           calIrqNum, calEvent;
    Cal_EmInstObj_t    *instObj    = NULL;
    Fvid2Utils_Node     *tempNode   = NULL;
    Fvid2Utils_Node     *emNode     = NULL;
    Cal_EmClientInfo_t *clientInfo = NULL;
    Cal_EmInitParams_t *initPrms   = NULL;

    /* Error condition Checking */
    GT_assert(CalTrace, (NULL != handle));
    tempNode = (Fvid2Utils_Node *) handle;
    GT_assert(CalTrace, (NULL != tempNode->data));
    instObj = ((Cal_EmClientInfo_t *) (tempNode->data))->instObj;
    GT_assert(CalTrace, (NULL != instObj));
    initPrms = &instObj->initPrms;

    /* Check to see if the node given is in the linked list */
    emNode = instObj->headNode;
    GT_assert(CalTrace, (NULL != emNode));

    found = (uint32_t) FALSE;
    while (NULL != emNode)
    {
        if (tempNode == emNode)
        {
            found = (uint32_t) TRUE;
            break;
        }
        emNode = emNode->next;
    }
    GT_assert(CalTrace, (FALSE != found));

    clientInfo = (Cal_EmClientInfo_t *) tempNode->data;
    GT_assert(CalTrace, (NULL != clientInfo));

    SemaphoreP_pend(gCalEmCommonObj.lockSem, SemaphoreP_WAIT_FOREVER);

    eventGroup = (uint32_t) (clientInfo->eventGroup);
    for (cnt = 0U; cnt < clientInfo->numEvents; cnt++)
    {
        event = clientInfo->allEvents[cnt];

        if (CAL_EM_EG_CAL == eventGroup)
        {
            if (CAL_EM_EVENT_LL_SOF4 <= event)
            {
                calIrqNum = 4U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF4;
            }
            else if (CAL_EM_EVENT_LL_SOF0 <= event)
            {
                calIrqNum = 3U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF0;
            }
            else if (CAL_EM_EVENT_WDMA_START0 <= event)
            {
                calIrqNum = 2U;
                calEvent  = event - CAL_EM_EVENT_WDMA_START0;
            }
            else if (CAL_EM_EVENT_WDMA_END0 <= event)
            {
                calIrqNum = 1U;
                calEvent  = event - CAL_EM_EVENT_WDMA_END0;
            }
            else
            {
                calIrqNum = 0U;
                calEvent  = event;
            }

            /* Disable the interrupts at the CAL level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                      CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(calIrqNum);
            HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);
        }
        else if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
                 (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
        {
            /* Disable the event at the CAL TOP */
            if (CAL_CSI2_PPI_CMPLXIO_RSERVED_31 > event)
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1;
                }
                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);

                /* Disable the interrupt and clear status */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal &= ~((uint32_t) 1 << event);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);

            }
            else if ((CAL_CSI2_PPI_VC_SOF1 <= event) &&
                     (CAL_CSI2_PPI_VC_RESERVED_31 > event))
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO_VC;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1_VC;
                }
                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_CLR(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);

                calEvent = event - CAL_CSI2_PPI_VC_SOF1;
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_VC_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal &= ~((uint32_t) 1 << calEvent);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);
            }
            else
            {
                GT_assert(CalTrace, FALSE);
            }
        }
#if defined(NOT_YET)
        else if ((CAL_EM_EG_LVDSRX0 <= eventGroup) &&
            (CAL_EM_EG_LVDSRX3 >= eventGroup))
        {
            /* Disable the interrupts at the SIMCOP level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                      LVDSRX_IRQENABLE_CLR_0;
            regAddr = regAddr +
                      ((LVDSRX_IRQENABLE_CLR_1 -
                        LVDSRX_IRQENABLE_CLR_0) *
                       (eventGroup - CAL_EM_EG_LVDSRX0));
            HW_WR_REG32(regAddr, ((uint32_t) 1 << event));

            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);
        }
#endif
        else
        {
            GT_assert(CalTrace, FALSE);
        }
    }

    SemaphoreP_post(gCalEmCommonObj.lockSem);

    return (FVID2_SOK);
}

int32_t Cal_emEnable(void *handle)
{
    uint32_t           found = FALSE;
    uint32_t           cnt, regAddr, regVal;
    uint32_t           eventGroup, event;
    uint32_t           calIrqNum, calEvent;
    Cal_EmInstObj_t    *instObj    = NULL;
    Fvid2Utils_Node     *tempNode   = NULL;
    Fvid2Utils_Node     *emNode     = NULL;
    Cal_EmClientInfo_t *clientInfo = NULL;
    Cal_EmInitParams_t *initPrms   = NULL;

    /* Error condition Checking */
    GT_assert(CalTrace, (NULL != handle));
    tempNode = (Fvid2Utils_Node *) handle;
    GT_assert(CalTrace, (NULL != tempNode->data));
    instObj = ((Cal_EmClientInfo_t *) (tempNode->data))->instObj;
    GT_assert(CalTrace, (NULL != instObj));
    initPrms = &instObj->initPrms;

    /* Check to see if the node given is in the linked list */
    emNode = instObj->headNode;
    GT_assert(CalTrace, (NULL != emNode));

    found = (uint32_t) FALSE;
    while (NULL != emNode)
    {
        if (tempNode == emNode)
        {
            found = (uint32_t) TRUE;
            break;
        }
        emNode = emNode->next;
    }
    GT_assert(CalTrace, (FALSE != found));

    clientInfo = (Cal_EmClientInfo_t *) tempNode->data;
    GT_assert(CalTrace, (NULL != clientInfo));

    SemaphoreP_pend(gCalEmCommonObj.lockSem, SemaphoreP_WAIT_FOREVER);

    eventGroup = (uint32_t) (clientInfo->eventGroup);
    for (cnt = 0U; cnt < clientInfo->numEvents; cnt++)
    {
        event = clientInfo->allEvents[cnt];

        if (CAL_EM_EG_CAL == eventGroup)
        {
            if (CAL_EM_EVENT_LL_SOF4 <= event)
            {
                calIrqNum = 4U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF4;
            }
            else if (CAL_EM_EVENT_LL_SOF0 <= event)
            {
                calIrqNum = 3U;
                calEvent  = event - CAL_EM_EVENT_LL_SOF0;
            }
            else if (CAL_EM_EVENT_WDMA_START0 <= event)
            {
                calIrqNum = 2U;
                calEvent  = event - CAL_EM_EVENT_WDMA_START0;
            }
            else if (CAL_EM_EVENT_WDMA_END0 <= event)
            {
                calIrqNum = 1U;
                calEvent  = event - CAL_EM_EVENT_WDMA_END0;
            }
            else
            {
                calIrqNum = 0U;
                calEvent  = event;
            }
            /* Enable the interrupts at the CAL level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                      CSL_CAL_C2IRQ_HL_IRQENABLE_SET(calIrqNum);

            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);

            HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));
        }
        else if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
                 (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
        {
            /* Disable the event at the CAL TOP */
            if (CAL_CSI2_PPI_CMPLXIO_RSERVED_31 > event)
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1;
                }
                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_SET(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);

                /* Disable the interrupt and clear status */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal |= ((uint32_t) 1 << event);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);
            }
            else if ((CAL_CSI2_PPI_VC_SOF1 <= event) &&
                     (CAL_CSI2_PPI_VC_RESERVED_31 > event))
            {
                if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                {
                    calEvent = CAL_EM_EVENT_PPIO_VC;
                }
                else
                {
                    calEvent = CAL_EM_EVENT_PPIO1_VC;
                }
                /* Disable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_SET(0U);
                HW_WR_REG32(regAddr, ((uint32_t) 1 << calEvent));

                /* Clear the status of the interrupt */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);

                calEvent = event - CAL_CSI2_PPI_VC_SOF1;

                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2PPI_CSI2_VC_IRQENABLE(0U);
                regVal  = HW_RD_REG32(regAddr);
                regVal |= ((uint32_t) 1 << calEvent);
                HW_WR_REG32(regAddr, regVal);

                regAddr = clientInfo->statusReg[cnt][2U];
                regVal  = clientInfo->mask[cnt][2U];
                HW_WR_REG32(regAddr, regVal);
            }
            else
            {
                GT_assert(CalTrace, FALSE);
            }
        }
#if defined(NOT_YET)
        else if ((CAL_EM_EG_LVDSRX0 <= eventGroup) &&
                 (CAL_EM_EG_LVDSRX3 >= eventGroup))
        {
            /* Clear the status of the interrupt */
            HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                        clientInfo->mask[cnt][1U]);

            /* Enable the interrupts at the SIMCOP level */
            regAddr = initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                      LVDSRX_IRQENABLE_SET_0;
            regAddr = regAddr +
                      ((LVDSRX_IRQENABLE_SET_1 -
                        LVDSRX_IRQENABLE_SET_0) *
                       (eventGroup - CAL_EM_EG_LVDSRX0));
            HW_WR_REG32(regAddr, ((uint32_t) 1 << event));
        }
#endif
        else
        {
            GT_assert(CalTrace, FALSE);
        }
    }

    SemaphoreP_post(gCalEmCommonObj.lockSem);

    return (FVID2_SOK);
}

int32_t Cal_emDisableAll(void *handle)
{
    return (FVID2_SOK);
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

static void CalEmMasterIsr(uintptr_t arg)
{
    uint32_t           cnt;
    uint32_t           regVal, numEvents;
    uint32_t           numClients = 0;
    Fvid2Utils_Node     *tempNode;
    Cal_EmClientInfo_t *clientInfo = NULL;
    Cal_EmInstObj_t    *instObj;
    Bool               isThirdLevelValid = FALSE;
    instObj  = (Cal_EmInstObj_t *) arg;
    tempNode = instObj->headNode;

    Fvid2Utils_startTsPrfLog(&instObj->isrPrfLog);

    /* Traverse through the entire list of registered clients and
     * clear the top level interrupts */
    tempNode = instObj->headNode;
    while (NULL != tempNode)
    {
        clientInfo = (Cal_EmClientInfo_t *) tempNode->data;
        GT_assert(CalTrace, (NULL != clientInfo));

        for (cnt = 0; cnt < clientInfo->numEvents; cnt++)
        {
            if (TRUE == clientInfo->isFirstLevelReq[cnt])
            {
                regVal = HW_RD_REG32(clientInfo->statusReg[cnt][0U]);
                if (clientInfo->mask[cnt][0U] ==
                    (clientInfo->mask[cnt][0U] & regVal))
                {
                    HW_WR_REG32(clientInfo->statusClrReg[cnt][0U],
                                clientInfo->mask[cnt][0U]);
                }
            }
        }

        /* Move to the next client */
        tempNode = tempNode->next;
    }

    /* Traverse through the entire list of registered clients */
    tempNode = instObj->headNode;
    while (NULL != tempNode)
    {
        isThirdLevelValid = FALSE;
        numEvents  = 0U;
        clientInfo = (Cal_EmClientInfo_t *) tempNode->data;
        GT_assert(CalTrace, (NULL != clientInfo));

        for (cnt = 0; cnt < clientInfo->numEvents; cnt++)
        {
            /* See if any of registered events has occurred */
            regVal = HW_RD_REG32(clientInfo->statusReg[cnt][1U]);
            if ((uint32_t) 0 != (regVal & (clientInfo->mask[cnt][1U])))
            {
                /* Make an array of events which has occurred. */
                clientInfo->setEvents[numEvents] =
                    clientInfo->allEvents[cnt];
                numEvents++;
                /* Clear the status and call the callback */
                HW_WR_REG32(clientInfo->statusClrReg[cnt][1U],
                            clientInfo->mask[cnt][1U]);

                /* TODO: Check if this can be done early here or
                 *       is it really required to clear eoi at submodule
                 *      level
                 */
                if (FALSE == clientInfo->isThirdLevelReq[cnt])
                {
                    HW_WR_REG32(clientInfo->subModuleEoiAddr[cnt],
                                clientInfo->subModuleEoiLine[cnt]);
                }
                else
                {
                    /* Third level of interrupt handling */
                    isThirdLevelValid = TRUE;
                    numEvents--;
                    break;
                }
            }
        }

        if (TRUE == isThirdLevelValid)
        {
            /* Level 2 events is cleared, handle 3rd level events */
            for (; cnt < clientInfo->numEvents; cnt++)
            {
                regVal = HW_RD_REG32(clientInfo->statusReg[cnt][2U]);
                if ((uint32_t) 0 != (regVal & (clientInfo->mask[cnt][2U])))
                {
                    clientInfo->setEvents[numEvents] =
                        clientInfo->allEvents[cnt];
                    numEvents++;
                    HW_WR_REG32(clientInfo->statusClrReg[cnt][2U],
                                clientInfo->mask[cnt][2U]);
                }
            }
        }

        /* Call callback if any event occurs for the client */
        if (0U != numEvents)
        {
            clientInfo->clientCb(
                clientInfo->setEvents,
                numEvents,
                clientInfo->arg);

            /* Increment total number of interrupts */
            clientInfo->numIntr++;

            numClients++;
        }

        /* Move to next client */
        tempNode = tempNode->next;
    }

    HW_WR_REG32(instObj->eoiAddr, instObj->instId);

    /* Increment total number of interrupts */
    instObj->numIntr++;

    Fvid2Utils_endTsPrfLog(&instObj->isrPrfLog);

    return;
}

/**
 *  \brief Function to allocate memory for the client information structure
 *  and other arrays.
 */
static Cal_EmClientInfo_t *CalEmCreateClientInfo(Fvid2Utils_Node **node)
{
    uint32_t           cnt;
    Cal_EmClientInfo_t *clientInfo = NULL;

    for (cnt = 0U; cnt < CAL_EM_MAX_CLIENTS; cnt++)
    {
        if (CAL_EM_MEM_FLAG_FREE == gCalEmCommonObj.clientMemFlag[cnt])
        {
            clientInfo = &(gCalEmCommonObj.clientMemPool[cnt]);
            *node      = &gCalEmCommonObj.nodePool[cnt];
            gCalEmCommonObj.clientMemFlag[cnt] = CAL_EM_MEM_FLAG_ALLOC;
            break;
        }
    }

    return (clientInfo);
}

/**
 *  \brief Function to free up memory for the client information structure
 *  and other arrays.
 */
static void CalEmDeleteClientInfo(const Cal_EmClientInfo_t *clientInfo)
{
    uint32_t cnt;

    GT_assert(CalTrace, (NULL != clientInfo));

    for (cnt = 0U; cnt < CAL_EM_MAX_CLIENTS; cnt++)
    {
        if (clientInfo == &(gCalEmCommonObj.clientMemPool[cnt]))
        {
            gCalEmCommonObj.clientMemFlag[cnt] = CAL_EM_MEM_FLAG_FREE;
            break;
        }
    }

    return;
}

/**
 *  \brief Function to add a node to the linked list of clients at the
 *  appropriate position as per the given priority.
 */
static void CalEmAddNode(Fvid2Utils_Node *newNode, Cal_EmPriority_t priority)
{
    Cal_EmInstObj_t *instObj = NULL;
    Fvid2Utils_Node  *curr    = NULL;
    Fvid2Utils_Node  *prev    = NULL;

    GT_assert(CalTrace, (NULL != newNode));

    instObj = ((Cal_EmClientInfo_t *) (newNode->data))->instObj;
    GT_assert(CalTrace, (NULL != instObj));

    newNode->prev = newNode->next = NULL;
    curr          = instObj->headNode;
    prev          = NULL;

    /* Search for place to insert new node as per priority */
    while ((NULL != curr) &&
           (((Cal_EmClientInfo_t *) curr->data)->priority < priority))
    {
        prev = curr;
        curr = curr->next;
    }

    /* Update pointers */
    newNode->next = curr;
    newNode->prev = prev;

    if (NULL == prev)
    {
        instObj->headNode = newNode;
    }
    else
    {
        prev->next = newNode;
    }

    if (NULL != curr)
    {
        curr->prev = newNode;
    }

    return;
}

/**
 *  \brief Function to remove the node from the linked list and delete memory
 *  allocated to it.
 */
static void CalEmDeleteNode(const Fvid2Utils_Node *emNode)
{
    Cal_EmInstObj_t *instObj = NULL;
    Fvid2Utils_Node  *curr    = NULL;
    Fvid2Utils_Node  *prev    = NULL;

    GT_assert(CalTrace, (NULL != emNode));

    instObj = ((Cal_EmClientInfo_t *) (emNode->data))->instObj;
    prev    = emNode->prev;
    curr    = emNode->next;

    if (NULL != prev)
    {
        if (NULL != curr)
        {
            prev->next = curr;
        }
        else
        {
            prev->next = NULL;
        }
    }
    else
    {
        instObj->headNode = curr;
        if (NULL != instObj->headNode)
        {
            instObj->headNode->prev = NULL;
        }
    }
    if (NULL != curr)
    {
        curr->prev = prev;
    }

    return;
}

static int32_t CalEmFillClientInfo(Cal_EmClientInfo_t *clientInfo,
                                   Cal_EmEventGroup_t  eventGroup,
                                   const uint32_t    *event,
                                   uint32_t           numEvents,
                                   Cal_EmPriority_t    priority,
                                   Cal_EmClientCbFxn   callback,
                                   void              *arg)
{
    int32_t         status = FVID2_SOK;
    uint32_t        cnt;
    uint32_t        regAddr, regVal;
    uint32_t        calEvent, calIrqNum;
    Cal_EmInstObj_t *instObj = NULL;
    Cal_EmInitParams_t *initPrms    = NULL;

    GT_assert(CalTrace, (NULL != clientInfo));
    GT_assert(CalTrace, (NULL != event));
    GT_assert(CalTrace, (NULL != callback));

    clientInfo->eventGroup = eventGroup;
    clientInfo->numEvents  = numEvents;
    instObj = clientInfo->instObj;
    initPrms = &instObj->initPrms;

    for (cnt = 0U; cnt < numEvents; cnt++)
    {
        if ((CAL_EM_EG_LVDSRX0 <= eventGroup) &&
            (CAL_EM_EG_LVDSRX3 >= eventGroup))
        {
            if (0U == initPrms->baseAddress[CAL_EM_MODULE_LVDSRX])
            {
                status = FVID2_EFAIL;
            }
        }
        else if ((CAL_EM_EG_CAL == eventGroup) ||
                 (CAL_EM_EG_BYS == eventGroup))
        {
            if (0U == initPrms->baseAddress[CAL_EM_MODULE_CAL])
            {
                status = FVID2_EFAIL;
            }
        }
        else if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
                 (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
        {
            if (0U == initPrms->baseAddress[CAL_EM_MODULE_CAL])
            {
                status = FVID2_EFAIL;
            }
        }
        else
        {
            status = FVID2_EFAIL;
        }

        if (status == FVID2_EFAIL)
        {
            break;
        }
    }

    if (FVID2_SOK == status)
    {
        for (cnt = 0U; cnt < numEvents; cnt++)
        {
            clientInfo->isThirdLevelReq[cnt] = FALSE;
            clientInfo->isFirstLevelReq[cnt] = FALSE;

            clientInfo->allEvents[cnt] = event[cnt];

            if (CAL_EM_EG_CAL == eventGroup)
            {
                if (CAL_EM_EVENT_LL_SOF4 <= event[cnt])
                {
                    calIrqNum = 4U;
                    calEvent  = event[cnt] - CAL_EM_EVENT_LL_SOF4;
                }
                else if (CAL_EM_EVENT_LL_SOF0 <= event[cnt])
                {
                    calIrqNum = 3U;
                    calEvent  = event[cnt] - CAL_EM_EVENT_LL_SOF0;
                }
                else if (CAL_EM_EVENT_WDMA_START0 <= event[cnt])
                {
                    calIrqNum = 2U;
                    calEvent  = event[cnt] - CAL_EM_EVENT_WDMA_START0;
                }
                else if (CAL_EM_EVENT_WDMA_END0 <= event[cnt])
                {
                    calIrqNum = 1U;
                    calEvent  = event[cnt] - CAL_EM_EVENT_WDMA_END0;
                }
                else
                {
                    calIrqNum = 0U;
                    calEvent  = event[cnt];
                }

                /* Enable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQENABLE_SET(calIrqNum);
                regVal  = HW_RD_REG32(regAddr);
                regVal |= ((uint32_t) 1 << calEvent);
                HW_WR_REG32(regAddr, regVal);

                /* Store the register address in clientInfo instance */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                          CSL_CAL_C2IRQ_HL_IRQSTATUS(calIrqNum);
                clientInfo->statusReg[cnt][1U]    = regAddr;
                clientInfo->statusClrReg[cnt][1U] = regAddr;
                clientInfo->mask[cnt][1U]         = ((uint32_t) 1U << calEvent);

                clientInfo->subModuleEoiAddr[cnt] =
                    initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                    CSL_CAL_C2CTL_HL_IRQ_EOI;
                clientInfo->subModuleEoiLine[cnt] = 0U;
            }
#if defined(NOT_YET)
            else if ((CAL_EM_EG_LVDSRX0 <= eventGroup) &&
                     (CAL_EM_EG_LVDSRX3 >= eventGroup))
            {
                /* Enable the interrupts at the CAL level */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                          LVDSRX_IRQENABLE_SET_0;
                regAddr = regAddr +
                          ((LVDSRX_IRQENABLE_SET_1 -
                            LVDSRX_IRQENABLE_SET_0) *
                           (eventGroup - CAL_EM_EG_LVDSRX0));
                regVal  = HW_RD_REG32(regAddr);
                regVal |= ((uint32_t) 1 << event[cnt]);
                HW_WR_REG32(regAddr, regVal);

                /* Store the register address in clientInfo instance */
                regAddr = initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                          LVDSRX_IRQSTATUS_0;
                regAddr = regAddr +
                          ((LVDSRX_IRQSTATUS_1 - LVDSRX_IRQSTATUS_0) *
                           (eventGroup - CAL_EM_EG_LVDSRX0));
                clientInfo->statusReg[cnt][1U]    = regAddr;
                clientInfo->statusClrReg[cnt][1U] = regAddr;
                clientInfo->mask[cnt][1U]         = ((uint32_t) 1U <<
                    event[cnt]);

                clientInfo->subModuleEoiAddr[cnt] =
                    initPrms->baseAddress[CAL_EM_MODULE_LVDSRX] +
                    LVDSRX_IRQ_EOI;
                clientInfo->subModuleEoiLine[cnt] =
                    (eventGroup - CAL_EM_EG_LVDSRX0);
            }
#endif
            else if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
                     (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
            {
                /* Handled in section below */
            }
            else
            {
                GT_assert(CalTrace, FALSE);
            }
        }


        /* These events are to processed at 3 levels
            0 : Top CAL level - Above for loop address this
            1 : At CAL level either PPI, PPI_VC or BYS_OVR
            2 : Instance specific errors CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQSTATUS &
                    per virtual channel CSL_CAL_C2PPI_CSI2_VC_IRQENABLE */
        if ((CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup) ||
            (CAL_EM_EG_CAL_PPI1_CSI2 == eventGroup))
        {
            for (cnt = 0U; cnt < numEvents; cnt++)
            {
                /* Enable the event at the CAL TOP */
                if (CAL_CSI2_PPI_CMPLXIO_RSERVED_31 > event[cnt])
                {
                    if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                    {
                        calEvent = CAL_EM_EVENT_PPIO;
                    }
                    else
                    {
                        calEvent = CAL_EM_EVENT_PPIO1;
                    }
                }
                else if ((CAL_CSI2_PPI_VC_SOF1 <= event[cnt]) &&
                         (CAL_CSI2_PPI_VC_RESERVED_31 > event[cnt]))
                {
                    if (CAL_EM_EG_CAL_PPI0_CSI2 == eventGroup)
                    {
                        calEvent = CAL_EM_EVENT_PPIO_VC;
                    }
                    else
                    {
                        calEvent = CAL_EM_EVENT_PPIO1_VC;
                    }
                }
                else
                {
                    status = FVID2_EBADARGS;
                    GT_assert(CalTrace, FALSE);
                }

                if (FVID2_SOK == status)
                {
                    regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                              CSL_CAL_C2IRQ_HL_IRQENABLE_SET(0U);
                    regVal  = HW_RD_REG32(regAddr);
                    regVal |= ((uint32_t) 1 << calEvent);
                    HW_WR_REG32(regAddr, regVal);

                    /* Store the register address in clientInfo instance */
                    regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                              CSL_CAL_C2IRQ_HL_IRQSTATUS(0U);
                    clientInfo->statusReg[cnt][1U]    = regAddr;
                    clientInfo->statusClrReg[cnt][1U] = regAddr;
                    clientInfo->mask[cnt][1U]         = ((uint32_t) 1U << calEvent);

                    clientInfo->isThirdLevelReq[cnt] = TRUE;

                    if (CAL_CSI2_PPI_CMPLXIO_RSERVED_31 > event[cnt])
                    {
                        calEvent = event[cnt];
                        regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                                  CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQENABLE(0U);
                        regVal  = HW_RD_REG32(regAddr);
                        regVal |= ((uint32_t) 1 << calEvent);
                        HW_WR_REG32(regAddr, regVal);

                        /* Store the register address in clientInfo instance */
                        regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                                  CSL_CAL_C2PPI_CSI2_COMPLEXIO_IRQSTATUS(0U);
                        clientInfo->statusReg[cnt][2U]    = regAddr;
                        clientInfo->statusClrReg[cnt][2U] = regAddr;
                        clientInfo->mask[cnt][2U]         =
                            ((uint32_t) 1U << calEvent);
                    }
                    else if ((CAL_CSI2_PPI_VC_SOF1 <= event[cnt]) &&
                           (CAL_CSI2_PPI_VC_RESERVED_31 > event[cnt]))
                    {
                        calEvent = event[cnt] - CAL_CSI2_PPI_VC_SOF1;

                        regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                                  CSL_CAL_C2PPI_CSI2_VC_IRQENABLE(0U);
                        regVal  = HW_RD_REG32(regAddr);
                        regVal |= ((uint32_t) 1 << calEvent);
                        HW_WR_REG32(regAddr, regVal);

                        /* Store the register address in clientInfo instance */
                        regAddr = initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                                  CSL_CAL_C2PPI_CSI2_VC_IRQSTATUS(0U);
                        clientInfo->statusReg[cnt][2U]    = regAddr;
                        clientInfo->statusClrReg[cnt][2U] = regAddr;
                        clientInfo->mask[cnt][2U]         =
                            ((uint32_t) 1U << calEvent);
                    }
                    else
                    {
                        GT_assert(CalTrace, FALSE);
                    }

                    clientInfo->subModuleEoiAddr[cnt] =
                        initPrms->baseAddress[CAL_EM_MODULE_CAL] +
                        CSL_CAL_C2CTL_HL_IRQ_EOI;
                    clientInfo->subModuleEoiLine[cnt] = 0U;
                }
            }
        }

        clientInfo->arg      = arg;
        clientInfo->clientCb = callback;
        clientInfo->priority = priority;
    }

    return (status);
}

static Cal_EmInstObj_t *CalEmGetInstObj(Cal_EmInstId_t instId)
{
    uint32_t        instCnt;
    Cal_EmInstObj_t *instObj = NULL;

    if (instId < CAL_EM_INST_ID_MAX)
    {
        for (instCnt = 0U; instCnt < CAL_EM_MAX_INST; instCnt++)
        {
            if (gCalEmInstObj[instCnt].instId == instId)
            {
                instObj = &gCalEmInstObj[instCnt];
                break;
            }
        }
    }

    return (instObj);
}

