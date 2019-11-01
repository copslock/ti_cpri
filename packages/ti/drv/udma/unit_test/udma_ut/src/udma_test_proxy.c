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
 *  \file udma_test_proxy.c
 *
 *  \brief UDMA proxy related test case file.
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

static int32_t udmaTestProxyTest(UdmaTestTaskObj *taskObj);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestProxyPerformanceTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Proxy Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    /* Perform proxy test */
    retVal = udmaTestProxyTest(taskObj);

    return (retVal);
}

static int32_t udmaTestProxyTest(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            isRingAllocated;
    uint32_t            isQueueProxyAllocated, isDequeueProxyAllocated;
    uint32_t            elemCnt = 100U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    uint32_t            loopCnt;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    struct Udma_RingObj ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    struct Udma_ProxyObj queueProxyObj, dequeueProxyObj;
    Udma_ProxyHandle    queueProxyHandle = &queueProxyObj;
    Udma_ProxyHandle    dequeueProxyHandle = &dequeueProxyObj;
    Udma_ProxyCfg       proxyCfg;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char               *navssString[] = {"MAIN", "MCU"};
    char               *ringModeString[] = {"RING", "MESSAGE"};
    uint32_t            hrs, mins, secs, durationInSecs, msecs;
    uint32_t            startTime, elapsedTime;

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    /* Create TimeStamp object */
    taskObj->prfTsHandle = Utils_prfTsCreate(taskObj->prfTsName);
    GT_assert(taskObj->traceMask, (taskObj->prfTsHandle != NULL));

    if(UDMA_SOK == retVal)
    {
        for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
        {
            isRingAllocated = FALSE;
            isQueueProxyAllocated = FALSE;
            isDequeueProxyAllocated = FALSE;
            ringMode = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
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
            }
            else
            {
                isRingAllocated = TRUE;
            }

            if(UDMA_SOK == retVal)
            {
                /* Allocate a proxy for queue operation */
                retVal = Udma_proxyAlloc(drvHandle, queueProxyHandle, UDMA_PROXY_ANY);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy alloc failed!!\n");
                }
                else
                {
                    isQueueProxyAllocated = TRUE;

                    /* Config proxy for queue operation */
                    proxyCfg.proxyMode = CSL_PROXY_QUEUE_ACCESS_MODE_TAIL;
                    proxyCfg.elemSize  = UDMA_RING_ES_8BYTES;
                    proxyCfg.ringNum   = Udma_ringGetNum(ringHandle);
                    retVal = Udma_proxyConfig(queueProxyHandle, &proxyCfg);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy config failed!!\n");
                    }
                }
            }

            if(UDMA_SOK == retVal)
            {
                /* Allocate a proxy for dequeue operation */
                retVal = Udma_proxyAlloc(drvHandle, dequeueProxyHandle, UDMA_PROXY_ANY);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy alloc failed!!\n");
                }
                else
                {
                    isDequeueProxyAllocated = TRUE;

                    /* Config proxy for dequeue operation */
                    proxyCfg.proxyMode = CSL_PROXY_QUEUE_ACCESS_MODE_HEAD;
                    proxyCfg.elemSize  = UDMA_RING_ES_8BYTES;
                    proxyCfg.ringNum   = Udma_ringGetNum(ringHandle);
                    retVal = Udma_proxyConfig(dequeueProxyHandle, &proxyCfg);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy config failed!!\n");
                    }
                }
            }

            if(UDMA_SOK == retVal)
            {
                loopCnt = 0U;
                Utils_prfTsBegin(taskObj->prfTsHandle);
                startTime = AppUtils_getCurTimeInMsec();
                while(loopCnt < taskObj->loopCnt)
                {
                    /* Queue through proxy */
                    for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                    {
                        ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                        Udma_proxyQueue(queueProxyHandle, ringData);
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
                        Udma_proxyDequeue(dequeueProxyHandle, &ringData);

                        if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                            break;
                        }
                    }

                    /* Check if the HW occupancy is zero */
                    if(udmaTestCompareRingHwOccDriver(ringHandle, 0U) != UDMA_SOK)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                        retVal = UDMA_EFAIL;
                        break;
                    }

                    loopCnt++;
                }

                elapsedTime = AppUtils_getElapsedTimeInMsec(startTime);
                durationInSecs = ((elapsedTime) / 1000U);
                hrs  = durationInSecs / (60U * 60U);
                mins = (durationInSecs / 60U) - (hrs * 60U);
                secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
                msecs = elapsedTime - (((hrs * 60U * 60U) + (mins * 60U) + secs) * 1000U);
                GT_4trace(taskObj->traceMask, GT_INFO1,
                          " |Proxy Test Duration|:: %d:%0.2d:%0.2d:%0.3d \r\n",
                          hrs, mins, secs, msecs);
                if(TRUE == taskObj->testPrms->prfEnable)
                {
                    uint64_t ps;

                    ps = taskObj->loopCnt * elemCnt;
                    ps *= 1000U;   /* ms to sec */
                    ps /= elapsedTime;
                    GT_1trace(taskObj->traceMask, GT_INFO1,
                        " |Proxy Performance|:: Proxy operation (queue/dequeue pair) per second: %d ::\r\n", ps);
                }

                Utils_prfTsEnd(taskObj->prfTsHandle, (taskObj->loopCnt * elemCnt));
            }

            if(TRUE == isQueueProxyAllocated)
            {
                retVal += Udma_proxyFree(queueProxyHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy free failed!!\n");
                }
            }
            if(TRUE == isDequeueProxyAllocated)
            {
                retVal += Udma_proxyFree(dequeueProxyHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Proxy free failed!!\n");
                }
            }
            if(TRUE == isRingAllocated)
            {
                retVal += Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                }
            }

            if(UDMA_SOK != retVal)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s failed!!\r\n",
                          navssString[instId], ringModeString[ringMode]);
                break;
            }
            else
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for NAVSS Inst: %s, Ring Mode: %s passed!!\r\n",
                          navssString[instId], ringModeString[ringMode]);
            }
        }
    }

    if(NULL != taskObj->prfTsHandle)
    {
        Utils_prfTsDelete(taskObj->prfTsHandle);
        taskObj->prfTsHandle = NULL;
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
