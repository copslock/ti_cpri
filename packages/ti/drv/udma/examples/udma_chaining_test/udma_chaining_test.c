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
 *  \file udma_chaining_test.c
 *
 *  \brief UDMA chaining sample application performing a chain of block copy
 *  using channel global trigger: CH0 -> CH1 -> ... -> CHx.
 *  The first channel doesn't user a global trigger and each channel triggers
 *  the next channel's global trigger through the channel's TR event register.
 *
 *  A channel's source buffer is previous channel destination buffer. This
 *  ensures that chaining trigger worked in a synchronized manner when the
 *  memory compare matches.
 *
 *  Requirement: TODO
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <ti/drv/udma/udma.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/udma/examples/udma_apputils/udma_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Application test parameters
 */
/** \brief Number of bytes to copy and buffer allocation */
#define UDMA_TEST_APP_NUM_BYTES         (1000U)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_BUF_SIZE_ALIGN    ((UDMA_TEST_APP_NUM_BYTES + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))
/** \brief Number of times to perform the memcpy operation */
#define UDMA_TEST_APP_LOOP_CNT          (100U)
/** \brief Number of channels */
#define UDMA_TEST_APP_NUM_CH            (2U)

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much memcpy operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * \
                                         UDMA_TEST_APP_RING_ENTRY_SIZE)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_RING_MEM_SIZE_ALIGN ((UDMA_TEST_APP_RING_MEM_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))
/**
 *  \brief UDMA TR packet descriptor memory.
 *  This contains the CSL_UdmapCppi5TRPD + Padding to sizeof(CSL_UdmapTR15) +
 *  one Type_15 TR (CSL_UdmapTR15) + one TR response of 4 bytes.
 *  Since CSL_UdmapCppi5TRPD is less than CSL_UdmapTR15, size is just two times
 *  CSL_UdmapTR15 for alignment.
 */
#define UDMA_TEST_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_TRPD_SIZE_ALIGN   ((UDMA_TEST_APP_TRPD_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct
{
    int32_t                 chIdx;

    struct Udma_ChObj       chObj;
    struct Udma_EventObj    cqEventObj;
    struct Udma_EventObj    tdCqEventObj;

    Udma_ChHandle           chHandle;
    Udma_EventHandle        cqEventHandle;
    Udma_EventHandle        tdCqEventHandle;

    Udma_DrvHandle          drvHandle;
    SemaphoreP_Handle       transferDoneSem;
    /**< Semaphore to indicate transfer completion */

    uint8_t                 *txRingMem;
    uint8_t                 *txCompRingMem;
    uint8_t                 *txTdCompRingMem;
    uint8_t                 *trpdMem;

    uint8_t                 *srcBuf;
    uint8_t                 *destBuf;
} App_UdmaChObj;

typedef struct
{
    struct Udma_DrvObj      drvObj;
    App_UdmaChObj           appChObj[UDMA_TEST_APP_NUM_CH];
} App_UdmaObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_chainingTest(App_UdmaObj *appObj);
static int32_t App_udmaChaining(App_UdmaObj *appObj);

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData);
static void App_udmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData);

static int32_t App_init(App_UdmaObj *appObj);
static int32_t App_deinit(App_UdmaObj *appObj);

static int32_t App_create(App_UdmaObj *appObj);
static int32_t App_delete(App_UdmaObj *appObj);

static void App_udmaTrpdInit(App_UdmaChObj *appChObj);

static void App_print(const char *str);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * UDMA driver and channel objects
 */
App_UdmaObj gUdmaAppObj;

/*
 * UDMA Memories
 */
static uint8_t gTxRingMem[UDMA_TEST_APP_NUM_CH][UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxCompRingMem[UDMA_TEST_APP_NUM_CH][UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxTdCompRingMem[UDMA_TEST_APP_NUM_CH][UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTrpdMem[UDMA_TEST_APP_NUM_CH][UDMA_TEST_APP_TRPD_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/*
 * Application Buffers
 */
static uint8_t gUdmaTestSrcBuf[UDMA_TEST_APP_BUF_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTestDestBuf[UDMA_TEST_APP_NUM_CH][UDMA_TEST_APP_BUF_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Global test pass/fail flag */
static volatile int32_t gUdmaAppResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * UDMA chaining test
 */
int32_t Udma_chainingTest(void)
{
    int32_t         retVal;
    App_UdmaObj    *appObj = &gUdmaAppObj;

    retVal = App_init(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App init failed!!\n");
    }

    App_print("UDMA chaining application started...\n");

    if(UDMA_SOK == retVal)
    {
        retVal = App_create(appObj);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_chainingTest(appObj);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App chaining test failed!!\n");
        }
    }

    retVal += App_delete(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App delete failed!!\n");
    }

    retVal += App_deinit(appObj);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App deinit failed!!\n");
    }

    if((UDMA_SOK == retVal) && (UDMA_SOK == gUdmaAppResult))
    {
        App_print("UDMA chaining Passed!!\n");
        App_print("All tests have passed!!\n");
    }
    else
    {
        App_print("UDMA chaining Failed!!\n");
        App_print("Some tests have failed!!\n");
    }

    return (0);
}

static int32_t App_chainingTest(App_UdmaObj *appObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        loopCnt = 0U;
    int32_t         chIdx;
    App_UdmaChObj  *appChObj;
    uint32_t        i;
    uint8_t        *destBuf;

    while(loopCnt < UDMA_TEST_APP_LOOP_CNT)
    {
        /* Reset dest buffers */
        for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
        {
            appChObj = &appObj->appChObj[chIdx];
            destBuf  = appChObj->destBuf;
            for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
            {
                destBuf[i] = 0U;
            }
            /* Writeback destination buffer */
            Udma_appUtilsCacheWb(appChObj->destBuf, UDMA_TEST_APP_NUM_BYTES);
        }

        /* Perform UDMA memcpy */
        retVal = App_udmaChaining(appObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    return (retVal);
}

static int32_t App_udmaChaining(App_UdmaObj *appObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t       *pTrResp, trRespStatus;
    uint8_t        *trpdMem;
    uint64_t        pDesc = 0;
    int32_t         chIdx;
    uint32_t        i;
    App_UdmaChObj  *appChObj;
    Udma_ChHandle   chHandle;

    /* Update TR packet descriptor */
    for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
    {
        appChObj = &appObj->appChObj[chIdx];
        App_udmaTrpdInit(appChObj);
    }

    /* Submit TRPD to channel */
    for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
    {
        appChObj = &appObj->appChObj[chIdx];
        chHandle = appChObj->chHandle;
        retVal = Udma_ringQueueRaw(
                     Udma_chGetFqRingHandle(chHandle),
                     (uint64_t) appChObj->trpdMem);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] Channel queue failed!!\n");
            break;
        }
    }

    if(UDMA_SOK == retVal)
    {
        chIdx = UDMA_TEST_APP_NUM_CH - 1U;
        appChObj = &appObj->appChObj[chIdx];
        chHandle = appChObj->chHandle;
        /* Wait for return descriptor in completion ring for the last channel
         * This marks the entire transfer completion */
        SemaphoreP_pend(appChObj->transferDoneSem, SemaphoreP_WAIT_FOREVER);
    }

    if(UDMA_SOK == retVal)
    {
        for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
        {
            appChObj = &appObj->appChObj[chIdx];
            chHandle = appChObj->chHandle;
            trpdMem  = appChObj->trpdMem;

            /* Response received in completion queue */
            retVal =
                Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] No descriptor after callback!!\n");
                retVal = UDMA_EFAIL;
            }

            if(UDMA_SOK == retVal)
            {
                /*
                 * Sanity check
                 */
                /* Check returned descriptor pointer */
                if(pDesc != ((uint64_t) trpdMem))
                {
                    App_print("[Error] TR descriptor pointer returned doesn't "
                           "match the submitted address!!\n");
                    retVal = UDMA_EFAIL;
                }
            }

            if(UDMA_SOK == retVal)
            {
                /* Invalidate cache */
                Udma_appUtilsCacheInv(trpdMem, UDMA_TEST_APP_TRPD_SIZE_ALIGN);

                /* check TR response status */
                pTrResp = (uint32_t *) (trpdMem + (sizeof(CSL_UdmapTR15) * 2U));
                trRespStatus = CSL_FEXT(*pTrResp, UDMAP_TR_RESPONSE_STATUS_TYPE);
                if(trRespStatus != CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE)
                {
                    App_print("[Error] TR Response not completed!!\n");
                    retVal = UDMA_EFAIL;
                }
            }

            if(UDMA_SOK == retVal)
            {
                /* Compare data */
                /* Invalidate destination buffer */
                Udma_appUtilsCacheInv(appChObj->destBuf, UDMA_TEST_APP_NUM_BYTES);
                for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
                {
                    if(appChObj->srcBuf[i] != appChObj->destBuf[i])
                    {
                        App_print("[Error] Data mismatch!!\n");
                        retVal = UDMA_EFAIL;
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

    return (retVal);
}

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData)
{
    App_UdmaChObj *appChObj = (App_UdmaChObj *) appData;

    if(appChObj != NULL)
    {
        if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
        {
            SemaphoreP_post(appChObj->transferDoneSem);
        }
        else
        {
            gUdmaAppResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaAppResult = UDMA_EFAIL;
    }

    return;
}

static void App_udmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData)
{
    int32_t             retVal;
    CSL_UdmapTdResponse tdResp;
    App_UdmaChObj      *appChObj = (App_UdmaChObj *) appData;

    if(appChObj != NULL)
    {
        if(UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventType)
        {
            /* Response received in Teardown completion queue */
            retVal = Udma_chDequeueTdResponse(appChObj->chHandle, &tdResp);
            if(UDMA_SOK != retVal)
            {
                /* [Error] No TD response after callback!! */
                gUdmaAppResult = UDMA_EFAIL;
            }
        }
        else
        {
            gUdmaAppResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaAppResult = UDMA_EFAIL;
    }

    return;
}

static int32_t App_init(App_UdmaObj *appObj)
{
    int32_t         retVal;
    Udma_InitPrms   initPrms;
    uint32_t        instId;
    int32_t         chIdx;
    App_UdmaChObj  *appChObj;
    Udma_DrvHandle  drvHandle = &appObj->drvObj;
    uint32_t        i;
    uint8_t        *srcBuf;

    /* Use MCU NAVSS for MCU domain cores. Rest all cores uses Main NAVSS */
#if defined (BUILD_MCU1_0) || defined (BUILD_MCU1_1)
    instId = UDMA_INST_ID_MCU_0;
#else
    instId = UDMA_INST_ID_MAIN_0;
#endif
    /* UDMA driver init */
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = &App_print;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA init failed!!\n");
    }

    /* Init channel parameters */
    for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
    {
        appChObj                    = &appObj->appChObj[chIdx];
        appChObj->chIdx             = chIdx;

        appChObj->chHandle          = &appChObj->chObj;
        appChObj->cqEventHandle     = NULL;
        appChObj->tdCqEventHandle   = NULL;
        appChObj->drvHandle         = drvHandle;
        appChObj->transferDoneSem   = NULL;
        appChObj->txRingMem         = &gTxRingMem[chIdx][0U];
        appChObj->txCompRingMem     = &gTxCompRingMem[chIdx][0U];
        appChObj->txTdCompRingMem   = &gTxTdCompRingMem[chIdx][0U];
        appChObj->trpdMem           = &gUdmaTrpdMem[chIdx][0U];
        if(0U == chIdx)
        {
            appChObj->srcBuf        = &gUdmaTestSrcBuf[0U];
        }
        else
        {
            /* src buffer of subsequent ch is previous ch dest buffer */
            appChObj->srcBuf        = &gUdmaTestDestBuf[chIdx - 1U][0U];
        }
        appChObj->destBuf           = &gUdmaTestDestBuf[chIdx][0U];

        /* Init buffers */
        srcBuf = appChObj->srcBuf;
        for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
        {
            srcBuf[i] = i;
        }
        /* Writeback source destination buffer */
        Udma_appUtilsCacheWb(appChObj->srcBuf, UDMA_TEST_APP_NUM_BYTES);
    }

    return (retVal);
}

static int32_t App_deinit(App_UdmaObj *appObj)
{
    int32_t         retVal;
    Udma_DrvHandle  drvHandle = &appObj->drvObj;

    retVal = Udma_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA deinit failed!!\n");
    }

    return (retVal);
}

static int32_t App_create(App_UdmaObj *appObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    SemaphoreP_Params   semPrms;
    int32_t             chIdx;
    App_UdmaChObj      *appChObj;
    Udma_ChHandle       chHandle;
    Udma_DrvHandle      drvHandle = &appObj->drvObj;

    for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
    {
        appChObj = &appObj->appChObj[chIdx];
        chHandle = appChObj->chHandle;

        SemaphoreP_Params_init(&semPrms);
        appChObj->transferDoneSem = SemaphoreP_create(0, &semPrms);
        if(NULL == appChObj->transferDoneSem)
        {
            App_print("[Error] Sem create failed!!\n");
            retVal = UDMA_EFAIL;
        }

        if(UDMA_SOK == retVal)
        {
            /* Init channel parameters */
            chType = UDMA_CH_TYPE_TR_BLK_COPY;
            UdmaChPrms_init(&chPrms, chType);
            chPrms.fqRingPrms.ringMem   = appChObj->txRingMem;
            chPrms.cqRingPrms.ringMem   = appChObj->txCompRingMem;
            chPrms.tdCqRingPrms.ringMem = appChObj->txTdCompRingMem;
            chPrms.fqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
            chPrms.cqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
            chPrms.tdCqRingPrms.ringMemSize = UDMA_TEST_APP_RING_MEM_SIZE;
            chPrms.fqRingPrms.elemCnt   = UDMA_TEST_APP_RING_ENTRIES;
            chPrms.cqRingPrms.elemCnt   = UDMA_TEST_APP_RING_ENTRIES;
            chPrms.tdCqRingPrms.elemCnt = UDMA_TEST_APP_RING_ENTRIES;

            /* Open channel for block copy */
            retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA channel open failed!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Config TX channel */
            UdmaChTxPrms_init(&txPrms, chType);
            retVal = Udma_chConfigTx(chHandle, &txPrms);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA TX channel config failed!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Config RX channel - which is implicitly paired to TX channel in
             * block copy mode */
            UdmaChRxPrms_init(&rxPrms, chType);
            retVal = Udma_chConfigRx(chHandle, &rxPrms);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA RX channel config failed!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            if((UDMA_TEST_APP_NUM_CH - 1U) == chIdx)
            {
                /* Register ring completion callback - for the last channel only */
                eventHandle = &appChObj->cqEventObj;
                UdmaEventPrms_init(&eventPrms);
                eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
                eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
                eventPrms.chHandle          = chHandle;
                eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
                eventPrms.eventCb           = &App_udmaEventDmaCb;
                eventPrms.appData           = appChObj;
                retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
                if(UDMA_SOK != retVal)
                {
                    App_print("[Error] UDMA CQ event register failed!!\n");
                }
                else
                {
                    appChObj->cqEventHandle = eventHandle;
                }
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Register teardown ring completion callback */
            eventHandle = &appChObj->tdCqEventObj;
            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType         = UDMA_EVENT_TYPE_TEARDOWN_PACKET;
            eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            eventPrms.chHandle          = chHandle;
            eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
            eventPrms.eventCb           = &App_udmaEventTdCb;
            eventPrms.appData           = appChObj;
            retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA Teardown CQ event register failed!!\n");
            }
            else
            {
                appChObj->tdCqEventHandle = eventHandle;
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Channel enable */
            retVal = Udma_chEnable(chHandle);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA channel enable failed!!\n");
            }
        }

        if(UDMA_SOK != retVal)
        {
            break;
        }
    }

    /* After all channels are created, chain the channels.
     * Doing before will have un-init channel handles */
    if(UDMA_SOK == retVal)
    {
        for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
        {
            appChObj = &appObj->appChObj[chIdx];
            chHandle = appChObj->chHandle;

            if(chIdx < (UDMA_TEST_APP_NUM_CH - 1))
            {
                Udma_ChHandle chainedChHandle;

                chainedChHandle = appObj->appChObj[chIdx + 1].chHandle;
                retVal = Udma_chSetChaining(
                             chHandle,
                             chainedChHandle,
                             CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
                if(UDMA_SOK != retVal)
                {
                    App_print("[Error] UDMA channel chaining failed!!\n");
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    return (retVal);
}

static int32_t App_delete(App_UdmaObj *appObj)
{
    int32_t         retVal, tempRetVal;
    uint64_t        pDesc;
    int32_t         chIdx;
    App_UdmaChObj  *appChObj;
    Udma_ChHandle   chHandle;

    for(chIdx = 0U; chIdx < UDMA_TEST_APP_NUM_CH; chIdx++)
    {
        appChObj = &appObj->appChObj[chIdx];
        chHandle = appChObj->chHandle;

        retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA channel disable failed!!\n");
        }

        /* Flush any pending request from the free queue */
        while(1)
        {
            tempRetVal = Udma_ringFlushRaw(
                             Udma_chGetFqRingHandle(chHandle), &pDesc);
            if(UDMA_ETIMEOUT == tempRetVal)
            {
                break;
            }
        }

        if(chIdx < (UDMA_TEST_APP_NUM_CH - 1))
        {
            Udma_ChHandle chainedChHandle;

            chainedChHandle = appObj->appChObj[chIdx + 1].chHandle;
            /* Break channel chaining */
            retVal += Udma_chBreakChaining(chHandle, chainedChHandle);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA channel break chaining failed!!\n");
            }
        }
    }

    for(chIdx = UDMA_TEST_APP_NUM_CH - 1U; chIdx >=0 ; chIdx--)
    {
        appChObj = &appObj->appChObj[chIdx];
        chHandle = appChObj->chHandle;

        /* Unregister all events */
        if(NULL != appChObj->cqEventHandle)
        {
            retVal += Udma_eventUnRegister(appChObj->cqEventHandle);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA event unregister failed!!\n");
            }
            appChObj->cqEventHandle = NULL;
        }
        if(NULL != appChObj->tdCqEventHandle)
        {
            retVal += Udma_eventUnRegister(appChObj->tdCqEventHandle);
            if(UDMA_SOK != retVal)
            {
                App_print("[Error] UDMA event unregister failed!!\n");
            }
            appChObj->tdCqEventHandle = NULL;
        }

        retVal += Udma_chClose(chHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA channel close failed!!\n");
        }

        if(appChObj->transferDoneSem != NULL)
        {
            SemaphoreP_delete(appChObj->transferDoneSem);
            appChObj->transferDoneSem = NULL;
        }
    }

    return (retVal);
}


static void App_udmaTrpdInit(App_UdmaChObj *appChObj)
{
    CSL_UdmapCppi5TRPD *pTrpd = (CSL_UdmapCppi5TRPD *) appChObj->trpdMem;
    CSL_UdmapTR15 *pTr = (CSL_UdmapTR15 *)(appChObj->trpdMem + sizeof(CSL_UdmapTR15));
    uint32_t *pTrResp = (uint32_t *) (appChObj->trpdMem + (sizeof(CSL_UdmapTR15) * 2U));
    uint32_t cqRingNum = Udma_chGetCqRingNum(appChObj->chHandle);

    /* Make TRPD */
    UdmaUtils_makeTrpd(pTrpd, UDMA_TR_TYPE_15, 1U, cqRingNum);

    /* Setup TR */
    pTr->flags  = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOL, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    if(appChObj->chIdx != 0U)
    {
        pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
    }
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = UDMA_TEST_APP_NUM_BYTES;
    pTr->icnt1    = 1U;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) appChObj->srcBuf;
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem.
                                           Replace with CSL-FL API */
    pTr->dicnt0   = UDMA_TEST_APP_NUM_BYTES;
    pTr->dicnt1   = 1U;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) appChObj->destBuf;

    /* Clear TR response memory */
    *pTrResp = 0xFFFFFFFFU;

    /* Writeback cache */
    Udma_appUtilsCacheWb(appChObj->trpdMem, UDMA_TEST_APP_TRPD_SIZE_ALIGN);

    return;
}

static void App_print(const char *str)
{
    UART_printf("%s", str);

    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", str);
    }

    return;
}
