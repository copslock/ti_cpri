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
 *  \file udma_test_ring.c
 *
 *  \brief UDMA ring related test case file.
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

static int32_t udmaTestRingProxyTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingFlushTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingEventTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingParamCheckTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingAttachTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingResetTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingPrimeTestLoop(UdmaTestTaskObj *taskObj);
static void udmaTestRingEventCb(Udma_EventHandle eventHandle,
                                uint32_t eventType,
                                void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Global test pass/fail flag */
static volatile int32_t gUdmaTestRingResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestRingProxyTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Proxy Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform proxy test */
        retVal = udmaTestRingProxyTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingFlushTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Flush API Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform proxy test */
        retVal = udmaTestRingFlushTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingEventTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Event API Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring event test */
        retVal = udmaTestRingEventTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingParamCheckTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Paramter Check Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        retVal = udmaTestRingParamCheckTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingUtilsMemSizeTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    ringMode, ringSize;
    uint32_t    elemCnt = 1U;
    uint32_t    ringMemSize, ringMemSizeExpected;
    char *ringModeString[] = {"RING", "MESSAGE", "CREDENTIAL", "QM"};
    uint32_t ringSizeArray[] = {4, 8, 16, 32, 64, 128, 256};

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Utils Mem Size Testcase ::\r\n", taskObj->taskId);

    for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
        ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_QM;
        ringMode++)
    {
        for(ringSize = UDMA_RING_ES_4BYTES;
            ringSize <= UDMA_RING_ES_256BYTES;
            ringSize++)
        {
            ringMemSize = UdmaUtils_getRingMemSize(ringMode, elemCnt, ringSize);
            GT_4trace(taskObj->traceMask, GT_INFO1,
                      " Ring mem size for %d elements:%4d bytes (Size:%3d, Mode:%12s)\r\n",
                      elemCnt, ringMemSize, ringSizeArray[ringSize], ringModeString[ringMode]);
            ringMemSizeExpected = elemCnt * ringSizeArray[ringSize];
            if(ringMode >= TISCI_MSG_VALUE_RM_RING_MODE_CREDENTIALS)
            {
                ringMemSizeExpected *= 2U;
            }
            if(ringMemSize != ringMemSizeExpected)
            {
                retVal = UDMA_EFAIL;
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " Ring memory size compare mismatched!!\r\n");
            }
        }
    }

    return (retVal);
}

int32_t udmaTestRingMemPtrTc(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, ringMode;
    uint32_t            elemCnt = 100U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Mem Pointer Testcase ::\r\n", taskObj->taskId);

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

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

                /* Check if the HW occupancy is zero */
                if(Udma_ringGetMemPtr(ringHandle) != ringMem)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring memory pointer mismatch!!\n");
                    retVal = UDMA_EFAIL;
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

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

int32_t udmaTestRingAttachTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Attach/Detach Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring attach test */
        retVal = udmaTestRingAttachTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingResetTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Reset Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring reset test */
        retVal = udmaTestRingResetTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingPrimeTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Prime Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring prime test */
        retVal = udmaTestRingPrimeTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

static int32_t udmaTestRingProxyTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

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

                /* Queue through proxy */
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

                /* Dequeue through proxy */
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

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingFlushTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

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

                /* Ring queue */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
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

                /* Dequeue using Flush API */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    retVal = Udma_ringFlushRaw(ringHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring flush failed!!\n");
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

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingEventTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};
    Udma_EventPrms      eventPrms;
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   transferDoneSem = NULL;
    struct Udma_EventObj eventObj;
    Udma_EventHandle    eventHandle = NULL;
    volatile uint64_t   intrStatusReg;

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
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing Ring Event for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

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

                GT_assert(taskObj->traceMask, (taskObj->ringPrms != NULL));
                if(UDMA_TEST_EVENT_NONE != taskObj->ringPrms->eventMode)
                {
                    /* Register ring completion */
                    eventHandle = &eventObj;
                    UdmaEventPrms_init(&eventPrms);
                    eventPrms.eventType         = UDMA_EVENT_TYPE_RING;
                    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
                    eventPrms.ringHandle        = ringHandle;
                    eventPrms.masterEventHandle = NULL;
                    if(UDMA_TEST_EVENT_INTR == taskObj->ringPrms->eventMode)
                    {
                        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
                        eventPrms.eventCb           = &udmaTestRingEventCb;
                        eventPrms.appData           = transferDoneSem;
                    }
                    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " UDMA Ring event register failed!!\n");
                        break;
                    }
                }

                /* Ring queue */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                if(UDMA_TEST_EVENT_NONE == taskObj->ringPrms->eventMode)
                {
                    /* Do nothing as ring is full the moment ring queue is done */
                }
                else
                {
                    if(UDMA_TEST_EVENT_INTR == taskObj->ringPrms->eventMode)
                    {
                        SemaphoreP_pend(transferDoneSem, SemaphoreP_WAIT_FOREVER);
                    }
                    else    /* Event polled mode */
                    {
                        /* Wait for event in polled mode */
                        while(1U)
                        {
                            intrStatusReg = CSL_REG64_RD(eventPrms.intrStatusReg);
                            if(intrStatusReg & eventPrms.intrMask)
                            {
                                /* Clear interrupt */
                                CSL_REG64_WR(eventPrms.intrClearReg, eventPrms.intrMask);
                                break;
                            }
                        #if !defined (UDMA_UT_BAREMETAL)
                            TaskP_yield();
                        #endif
                        }
                    }
                }

                /* Dequeue */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    retVal = Udma_ringFlushRaw(ringHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring flush failed!!\n");
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

                if(UDMA_TEST_EVENT_NONE != taskObj->ringPrms->eventMode)
                {
                    retVal = Udma_eventUnRegister(eventHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Event unregister failed!!\n");
                        break;
                    }
                }

                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing Ring Event for NAVSS Inst: %s, Ring Mode: %s passed!!\r\n",
                          navssString[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingParamCheckTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, ringMode;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing ring params check for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

                /* Ring memory NULL check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = NULL;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for mem NULL check!!\n");
                    break;
                }

                /* Ring memory alignment check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = (void *) ((uintptr_t)ringMem + 4U);
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for mem align check!!\n");
                    break;
                }

                /* Ring zero element count check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = 0U;
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for zero element count check!!\n");
                    break;
                }

                /* Ring mem size check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt + 1U;
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for memsize check!!\n");
                    break;
                }

                /* Ring mem size check skip with more memory - should not return error */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt + 1U;
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc memsize check failed when it shoudl not!!\n");
                    break;
                }
                else
                {
                    /* Free-up the allocated ring */
                    retVal = Udma_ringFree(ringHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                        break;
                    }
                }
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingAttachTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 100U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    uint16_t            ringNum;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj, attachRingObj;
    Udma_RingHandle     ringHandle = &ringObj, attachRingHandle = &attachRingObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

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

                /* Attach to the allocated ring */
                ringNum = Udma_ringGetNum(ringHandle);
                retVal = Udma_ringAttach(drvHandle, attachRingHandle, ringNum);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring attach failed!!\n");
                    break;
                }

                /* Queue and check ring operation through attach handle */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(attachRingHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is same as what is queued */
                if(udmaTestCompareRingHwOccDriver(attachRingHandle, elemCnt) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Dequeue and flush the ring */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    retVal = Udma_ringDequeueRaw(attachRingHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring dequeue failed!!\n");
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
                if(udmaTestCompareRingHwOccDriver(attachRingHandle, 0U) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                retVal = Udma_ringDetach(attachRingHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring detach failed!!\n");
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

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingResetTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode, ringToAlloc;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
                ringMode <= TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                          navssString[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                /* Allocate a FQ ring - same as first UDMA memcpy channel for the core */
                ringToAlloc = drvHandle->initPrms.rmInitPrms.startBlkCopyCh;

                /* Allocate a free ring */
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringToAlloc, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Queue through proxy */
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

                /* Free the ring and alloc again - this will do the ring reset automatically */
                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringToAlloc, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Check if the HW occupancy is zero */
                if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring reset failed!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Queue through proxy */
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

                /* Dequeue through proxy */
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

            if(UDMA_SOK != retVal)
            {
                break;
            }
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

static int32_t udmaTestRingPrimeTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *navssString[] = {"MAIN", "MCU"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for NAVSS Inst: %s, Ring Mode: %s...\r\n",
                      navssString[instId], ringModeString[ringMode]);
            drvHandle = &taskObj->testObj->drvObj[instId];

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

            /* Queue through prime API */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                Udma_ringPrime(ringHandle, ringData);
            }

            /* Check if the HW occupancy is zero as the queue is not committed */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Do Cache flush and commit to ring */
            Udma_appUtilsCacheWb(ringMem, ringMemSize);
            Udma_ringSetDoorBell(ringHandle, elemCnt);

            /* Check if the HW occupancy is same as what is queued */
            if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Dequeue through proxy */
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

static void udmaTestRingEventCb(Udma_EventHandle eventHandle,
                                uint32_t eventType,
                                void *appData)
{
    SemaphoreP_Handle   transferDoneSem = (SemaphoreP_Handle) appData;

    if(transferDoneSem != NULL)
    {
        if(UDMA_EVENT_TYPE_RING == eventType)
        {
            SemaphoreP_post(transferDoneSem);
        }
        else
        {
            gUdmaTestRingResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaTestRingResult = UDMA_EFAIL;
    }

    return;
}
