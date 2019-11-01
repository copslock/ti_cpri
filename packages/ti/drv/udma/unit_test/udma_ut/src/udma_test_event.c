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
 *  \file udma_test_event.c
 *
 *  \brief UDMA event related test case file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

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

static int32_t udmaTestEventOutOfRangeFlowLoop(UdmaTestTaskObj *taskObj,
                                               uint32_t loopCnt);
static void udmaTestEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Global test pass/fail flag */
static volatile int32_t gUdmaTestEventResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestEventOutOfRangeFlow(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Out Of Range Flow Event Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestEventResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform flow event test */
        retVal = udmaTestEventOutOfRangeFlowLoop(taskObj, loopCnt);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestEventResult;

    return (retVal);
}

static int32_t udmaTestEventOutOfRangeFlowLoop(UdmaTestTaskObj *taskObj,
                                               uint32_t loopCnt)
{
    int32_t                     retVal = UDMA_SOK;
    uint32_t                    instId;
    Udma_DrvHandle              drvHandle;
    Udma_EventPrms              eventPrms;
    struct Udma_EventObj        eventObj;
    Udma_EventHandle            eventHandle = NULL;
    Udma_EventRxFlowIdFwStatus  status;
    char                       *navssString[] = {"MAIN", "MCU"};

    for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
    {
        if(0U == loopCnt)
        {
            GT_1trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s ...\r\n", navssString[instId]);
        }

        drvHandle = &taskObj->testObj->drvObj[instId];

        /* Register flow error event */
        eventHandle = &eventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_ERR_OUT_OF_RANGE_FLOW;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
        eventPrms.eventCb           = &udmaTestEventCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR,
                " UDMA Flow error event register failed!!\n");
            break;
        }

        /* Note: Since this event can't be reproduced in a standalone UDMA
         * level testcase, this test doesn't wait for the event to happen.
         * This just checks register and unregister check
         */

        /* Just check status API sanity */
        retVal = Udma_eventGetRxFlowIdFwStatus(eventHandle, &status);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " RX flow ID FW status get failed!!\n");
        }
        else
        {
            if(0U == loopCnt)
            {
                GT_1trace(taskObj->traceMask, GT_INFO1,
                    " Rx Flow ID Status : %d\r\n", status.isException);
                GT_1trace(taskObj->traceMask, GT_INFO1,
                    " Rx Flow ID        : %d\r\n", status.flowId);
                GT_1trace(taskObj->traceMask, GT_INFO1,
                    " Rx Channel        : %d\r\n", status.chNum);
            }
        }

        retVal += Udma_eventUnRegister(eventHandle);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Event unregister failed!!\n");
            break;
        }

        if(0U == loopCnt)
        {
            GT_1trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s passed!!\r\n", navssString[instId]);
        }
    }

    return (retVal);
}

static void udmaTestEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData)
{
    SemaphoreP_Handle   transferDoneSem = (SemaphoreP_Handle) appData;

    if(transferDoneSem != NULL)
    {
        if(UDMA_EVENT_TYPE_ERR_OUT_OF_RANGE_FLOW == eventType)
        {
            SemaphoreP_post(transferDoneSem);
        }
        else
        {
            gUdmaTestEventResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaTestEventResult = UDMA_EFAIL;
    }

    return;
}
