/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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
 *  \file udma_test_ring_monitor.c
 *
 *  \brief UDMA ring monitor related test case file.
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

static int32_t udmaTestRingMonPushPopTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingMonLowThresholdTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingMonHighThresholdTestLoop(UdmaTestTaskObj *taskObj);
static void udmaTestRingMonEventCb(Udma_EventHandle eventHandle,
                                   uint32_t eventType,
                                   void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Global test pass/fail flag */
static volatile int32_t gUdmaTestRingMonResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestRingMonPushPopTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Monitor Push/Pop Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingMonResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        retVal = udmaTestRingMonPushPopTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingMonResult;

    return (retVal);
}

int32_t udmaTestRingMonLowThresholdTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Monitor Low Threshold Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingMonResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        retVal = udmaTestRingMonLowThresholdTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingMonResult;

    return (retVal);
}

int32_t udmaTestRingMonHighThresholdTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Monitor High Threshold Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingMonResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        retVal = udmaTestRingMonHighThresholdTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingMonResult;

    return (retVal);
}

static int32_t udmaTestRingMonPushPopTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                instId, qCnt, ringMode;
    uint32_t                elemCnt = 500U, ringMemSize;
    uint32_t                heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle          drvHandle;
    Udma_RingPrms           ringPrms;
    struct Udma_RingObj     ringObj;
    Udma_RingHandle         ringHandle = &ringObj;
    struct Udma_RingMonObj  ringMonObj;
    Udma_RingMonHandle      monHandle = &ringMonObj;
    Udma_RingMonPrms        monPrms;
    Udma_RingMonData        monData;
    void                   *ringMem = NULL;
    uint64_t                ringData;
    char                   *navssString[] = {"MAIN", "MCU"};
    char                   *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        instId = UDMA_TEST_DEFAULT_UDMA_INST;
        drvHandle = &taskObj->testObj->drvObj[instId];
        for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
            ringMode++)
        {
            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                      navssString[instId], ringModeString[ringMode]);

            UdmaRingPrms_init(&ringPrms);
            ringPrms.ringMem = ringMem;
            ringPrms.ringMemSize = ringMemSize;
            ringPrms.mode = ringMode;
            ringPrms.elemCnt = elemCnt;

            /* Allocate a free ring */
            retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                break;
            }

            /* Allocate and configure a ring monitor */
            retVal = Udma_ringMonAlloc(drvHandle, monHandle, UDMA_RING_MON_ANY);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor alloc failed!!\n");
                break;
            }
            UdmaRingMonPrms_init(&monPrms);
            monPrms.source  = TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT;
            monPrms.mode    = TISCI_MSG_VALUE_RM_MON_MODE_PUSH_POP;
            monPrms.ringNum = Udma_ringGetNum(ringHandle);
            retVal = Udma_ringMonConfig(monHandle, &monPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor config failed!!\n");
                break;
            }

            /* Queue ring */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                retVal = Udma_ringQueueRaw(ringHandle, ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Read monitor data */
            retVal = Udma_ringMonGetData(monHandle, &monData);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor data read failed!!\n");
                break;
            }
            /* Check monitor data is correct - push should match queue count and dequeue should be zero */
            if((monData.data0 != elemCnt) || (monData.data1 != 0U))
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor data mismatch error !!\n");
                break;
            }

            /* Check if the HW occupancy is same as what is queued */
            if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Dequeue ring */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = 0UL;
                retVal = Udma_ringDequeueRaw(ringHandle, &ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                    break;
                }

                if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Read monitor data */
            retVal = Udma_ringMonGetData(monHandle, &monData);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor data read failed!!\n");
                break;
            }
            /* Check monitor data is correct - push should zero and dequeue should match dequeue count */
            if((monData.data0 != 0U) || (monData.data1 != elemCnt))
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor data mismatch error !!\n");
                break;
            }

            /* Check if the HW occupancy is zero */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            retVal = Udma_ringMonFree(monHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor free failed!!\n");
                break;
            }

            retVal = Udma_ringFree(ringHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                break;
            }

            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s passed!!\r\n",
                      navssString[instId], ringModeString[ringMode]);
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingMonLowThresholdTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                instId, qCnt, ringMode;
    uint32_t                elemCnt = 500U, ringMemSize;
    uint32_t                heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle          drvHandle;
    Udma_RingPrms           ringPrms;
    struct Udma_RingObj     ringObj;
    Udma_RingHandle         ringHandle = &ringObj;
    struct Udma_RingMonObj  ringMonObj;
    Udma_RingMonHandle      monHandle = &ringMonObj;
    Udma_RingMonPrms        monPrms;
    void                   *ringMem = NULL;
    uint64_t                ringData;
    char                   *navssString[] = {"MAIN", "MCU"};
    char                   *ringModeString[] = {"RING", "MESSAGE"};
    Udma_EventPrms          eventPrms;
    SemaphoreP_Params       semPrms;
    SemaphoreP_Handle       transferDoneSem = NULL;
    struct Udma_EventObj    eventObj;
    Udma_EventHandle        eventHandle = NULL;

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        SemaphoreP_Params_init(&semPrms);
        transferDoneSem = SemaphoreP_create(0, &semPrms);
        if(NULL == transferDoneSem)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Sem create failed!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        instId = UDMA_TEST_DEFAULT_UDMA_INST;
        drvHandle = &taskObj->testObj->drvObj[instId];
        for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
            ringMode++)
        {
            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                      navssString[instId], ringModeString[ringMode]);

            UdmaRingPrms_init(&ringPrms);
            ringPrms.ringMem = ringMem;
            ringPrms.ringMemSize = ringMemSize;
            ringPrms.mode = ringMode;
            ringPrms.elemCnt = elemCnt;

            /* Allocate a free ring */
            retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                break;
            }

            /* Allocate and configure a ring monitor */
            retVal = Udma_ringMonAlloc(drvHandle, monHandle, UDMA_RING_MON_ANY);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor alloc failed!!\n");
                break;
            }
            UdmaRingMonPrms_init(&monPrms);
            monPrms.source  = TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT;
            monPrms.mode    = TISCI_MSG_VALUE_RM_MON_MODE_THRESHOLD;
            monPrms.ringNum = Udma_ringGetNum(ringHandle);
            monPrms.data0   = elemCnt / 2U;     /* Test low threshold */
            monPrms.data1   = elemCnt;          /* High threshold more than ring depth */
            retVal = Udma_ringMonConfig(monHandle, &monPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor config failed!!\n");
                break;
            }

            /* Register ring monitor event */
            eventHandle = &eventObj;
            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType         = UDMA_EVENT_TYPE_RING_MON;
            eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            eventPrms.monHandle         = monHandle;
            eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
            eventPrms.eventCb           = &udmaTestRingMonEventCb;
            eventPrms.appData           = transferDoneSem;
            retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA Ring monitor event register failed!!\n");
                break;
            }

            /* Queue ring */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                retVal = Udma_ringQueueRaw(ringHandle, ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Check if the HW occupancy is same as what is queued */
            if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Dequeue ring - one more than threshold so that it crosses and generates event */
            for(qCnt = 0U; qCnt < ((elemCnt / 2U) + 1U); qCnt++)
            {
                ringData = 0UL;
                retVal = Udma_ringDequeueRaw(ringHandle, &ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                    break;
                }

                if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Ring monitor will trigger a event at this point */
            SemaphoreP_pend(transferDoneSem, SemaphoreP_WAIT_FOREVER);

            /* Dequeue the remaining */
            for(qCnt = ((elemCnt / 2U) + 1U); qCnt < elemCnt; qCnt++)
            {
                ringData = 0UL;
                retVal = Udma_ringDequeueRaw(ringHandle, &ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                    break;
                }

                if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Check if the HW occupancy is zero */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            retVal = Udma_eventUnRegister(eventHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Event unregister failed!!\n");
                break;
            }

            retVal = Udma_ringMonFree(monHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor free failed!!\n");
                break;
            }

            retVal = Udma_ringFree(ringHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                break;
            }

            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s passed!!\r\n",
                      navssString[instId], ringModeString[ringMode]);
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    if(transferDoneSem != NULL)
    {
        SemaphoreP_delete(transferDoneSem);
    }

    return (retVal);
}

static int32_t udmaTestRingMonHighThresholdTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                instId, qCnt, ringMode;
    uint32_t                elemCnt = 500U, ringMemSize;
    uint32_t                heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle          drvHandle;
    Udma_RingPrms           ringPrms;
    struct Udma_RingObj     ringObj;
    Udma_RingHandle         ringHandle = &ringObj;
    struct Udma_RingMonObj  ringMonObj;
    Udma_RingMonHandle      monHandle = &ringMonObj;
    Udma_RingMonPrms        monPrms;
    void                   *ringMem = NULL;
    uint64_t                ringData;
    char                   *navssString[] = {"MAIN", "MCU"};
    char                   *ringModeString[] = {"RING", "MESSAGE"};
    Udma_EventPrms          eventPrms;
    SemaphoreP_Params       semPrms;
    SemaphoreP_Handle       transferDoneSem = NULL;
    struct Udma_EventObj    eventObj;
    Udma_EventHandle        eventHandle = NULL;

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        SemaphoreP_Params_init(&semPrms);
        transferDoneSem = SemaphoreP_create(0, &semPrms);
        if(NULL == transferDoneSem)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Sem create failed!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        instId = UDMA_TEST_DEFAULT_UDMA_INST;
        drvHandle = &taskObj->testObj->drvObj[instId];
        for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
            ringMode++)
        {
            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                      navssString[instId], ringModeString[ringMode]);

            UdmaRingPrms_init(&ringPrms);
            ringPrms.ringMem = ringMem;
            ringPrms.ringMemSize = ringMemSize;
            ringPrms.mode = ringMode;
            ringPrms.elemCnt = elemCnt;

            /* Allocate a free ring */
            retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                break;
            }

            /* Allocate and configure a ring monitor */
            retVal = Udma_ringMonAlloc(drvHandle, monHandle, UDMA_RING_MON_ANY);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor alloc failed!!\n");
                break;
            }
            UdmaRingMonPrms_init(&monPrms);
            monPrms.source  = TISCI_MSG_VALUE_RM_MON_SRC_ELEM_CNT;
            monPrms.mode    = TISCI_MSG_VALUE_RM_MON_MODE_THRESHOLD;
            monPrms.ringNum = Udma_ringGetNum(ringHandle);
            monPrms.data0   = 0U;               /* Disable low threshold */
            monPrms.data1   = elemCnt / 2U;     /* Test High threshold */
            retVal = Udma_ringMonConfig(monHandle, &monPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor config failed!!\n");
                break;
            }

            /* Register ring monitor event */
            eventHandle = &eventObj;
            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType         = UDMA_EVENT_TYPE_RING_MON;
            eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            eventPrms.monHandle         = monHandle;
            eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
            eventPrms.eventCb           = &udmaTestRingMonEventCb;
            eventPrms.appData           = transferDoneSem;
            retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA Ring monitor event register failed!!\n");
                break;
            }

            /* Queue ring - one more than the threshold */
            for(qCnt = 0U; qCnt < ((elemCnt / 2U) + 1U); qCnt++)
            {
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                retVal = Udma_ringQueueRaw(ringHandle, ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Ring monitor will trigger a event at this point */
            SemaphoreP_pend(transferDoneSem, SemaphoreP_WAIT_FOREVER);

            /* Queue the remaining */
            for(qCnt = ((elemCnt / 2U) + 1U); qCnt < elemCnt; qCnt++)
            {
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                retVal = Udma_ringQueueRaw(ringHandle, ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Check if the HW occupancy is same as what is queued */
            if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Dequeue ring */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = 0UL;
                retVal = Udma_ringDequeueRaw(ringHandle, &ringData);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                    break;
                }

                if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Check if the HW occupancy is zero */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            retVal = Udma_eventUnRegister(eventHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Event unregister failed!!\n");
                break;
            }

            retVal = Udma_ringMonFree(monHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring monitor free failed!!\n");
                break;
            }

            retVal = Udma_ringFree(ringHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                break;
            }

            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s passed!!\r\n",
                      navssString[instId], ringModeString[ringMode]);
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    if(transferDoneSem != NULL)
    {
        SemaphoreP_delete(transferDoneSem);
    }

    return (retVal);
}

static void udmaTestRingMonEventCb(Udma_EventHandle eventHandle,
                                   uint32_t eventType,
                                   void *appData)
{
    SemaphoreP_Handle   transferDoneSem = (SemaphoreP_Handle) appData;

    if(transferDoneSem != NULL)
    {
        if(UDMA_EVENT_TYPE_RING_MON == eventType)
        {
            SemaphoreP_post(transferDoneSem);
        }
        else
        {
            gUdmaTestRingMonResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaTestRingMonResult = UDMA_EFAIL;
    }

    return;
}
