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
 *  \file udma_dru_direct_tr_test.c
 *
 *  \brief UDMA DRU memcpy sample application performing block copy using
 *  direct TR submission via DRU registers.
 *
 *  Requirement: DOX_REQ_TAG(PDK-2635)
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
#define UDMA_TEST_APP_NUM_BYTES         (100U)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_NUM_BYTES_ALIGN    ((UDMA_TEST_APP_NUM_BYTES + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/** \brief Number of times to perform the memcpy operation */
#define UDMA_TEST_APP_LOOP_CNT          (10U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_memcpyTest(Udma_ChHandle chHandle);
static void App_udmaMemcpy(Udma_ChHandle chHandle,
                           void *destBuf,
                           void *srcBuf,
                           uint32_t length);

static void App_udmaEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData);

static int32_t App_init(Udma_DrvHandle drvHandle);
static int32_t App_deinit(Udma_DrvHandle drvHandle);

static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle);
static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle);

static void App_udmaTrInit(Udma_ChHandle chHandle,
                           CSL_UdmapTR *pTr,
                           const void *destBuf,
                           const void *srcBuf,
                           uint32_t length);

static void App_print(const char *str);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaChObj;
struct Udma_EventObj    gUdmaTrEventObj;

/*
 * Application Buffers
 */
static uint8_t gUdmaTestSrcBuf[UDMA_TEST_APP_NUM_BYTES_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTestDestBuf[UDMA_TEST_APP_NUM_BYTES_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/* Semaphore to indicate transfer completion */
static SemaphoreP_Handle gUdmaAppDoneSem = NULL;

/* Global test pass/fail flag */
static volatile int32_t gUdmaAppResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Application main
 */
int32_t Udma_druDirectTrTest(void)
{
    int32_t         retVal;
    Udma_DrvHandle  drvHandle = &gUdmaDrvObj;
    Udma_ChHandle   chHandle = &gUdmaChObj;

    App_print("UDMA DRU Direct TR application started...\n");

    retVal = App_init(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App init failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_create(drvHandle, chHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_memcpyTest(chHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App memcpy test failed!!\n");
        }
    }

    retVal += App_delete(drvHandle, chHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App delete failed!!\n");
    }

    retVal += App_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App deinit failed!!\n");
    }

    if((UDMA_SOK == retVal) && (UDMA_SOK == gUdmaAppResult))
    {
        App_print("UDMA DRU memcpy using Direct TR15 block copy Passed!!\n");
        App_print("All tests have passed!!\n");
    }
    else
    {
        App_print("UDMA DRU memcpy using Direct TR15 block copy Failed!!\n");
        App_print("Some tests have failed!!\n");
    }

    return (0);
}

static int32_t App_memcpyTest(Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            i;
    uint32_t            loopCnt = 0U;
    uint8_t            *srcBuf = &gUdmaTestSrcBuf[0U];
    uint8_t            *destBuf = &gUdmaTestDestBuf[0U];

    /* Init buffers */
    for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
    {
        srcBuf[i] = i;
        destBuf[i] = 0U;
    }
    /* Writeback source and destination buffer */
    Udma_appUtilsCacheWb(&gUdmaTestSrcBuf[0U], UDMA_TEST_APP_NUM_BYTES);
    Udma_appUtilsCacheWb(&gUdmaTestDestBuf[0U], UDMA_TEST_APP_NUM_BYTES);

    while(loopCnt < UDMA_TEST_APP_LOOP_CNT)
    {
        /* Perform UDMA memcpy */
        App_udmaMemcpy(chHandle, destBuf, srcBuf, UDMA_TEST_APP_NUM_BYTES);

        /*
         * Compare data
         */
        /* Invalidate destination buffer */
        Udma_appUtilsCacheInv(&gUdmaTestDestBuf[0U], UDMA_TEST_APP_NUM_BYTES);
        for(i = 0U; i < UDMA_TEST_APP_NUM_BYTES; i++)
        {
            if(srcBuf[i] != destBuf[i])
            {
                App_print("[Error] Data mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }
        }

        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    return (retVal);
}

static void App_udmaMemcpy(Udma_ChHandle chHandle,
                           void *destBuf,
                           void *srcBuf,
                           uint32_t length)
{
    CSL_UdmapTR tr;

    /* Update TR packet descriptor */
    App_udmaTrInit(chHandle, &tr, destBuf, srcBuf, length);

    /* Submit TR to DRU channel */
    Udma_chDruSubmitTr(chHandle, &tr);

    /* Wait for TR event - this marks the transfer completion */
    SemaphoreP_pend(gUdmaAppDoneSem, SemaphoreP_WAIT_FOREVER);

    return;
}

static void App_udmaEventCb(Udma_EventHandle eventHandle,
                            uint32_t eventType,
                            void *appData)
{
    if(UDMA_EVENT_TYPE_TR == eventType)
    {
        SemaphoreP_post(gUdmaAppDoneSem);
    }
    else
    {
        gUdmaAppResult = UDMA_EFAIL;
    }

    return;
}

static int32_t App_init(Udma_DrvHandle drvHandle)
{
    int32_t             retVal;
    Udma_InitPrms       initPrms;
    uint32_t            instId;
    uint32_t            utcId;
    uint32_t            numQueue, queId;
    CSL_DruQueueConfig  queueCfg;

    /* Note: There is no external channel support in MCU NAVSS. So always use
     * main NAVSS even for MCU builds */
    /* UDMA driver init */
    instId = UDMA_INST_ID_MAIN_0;
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = &App_print;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA init failed!!\n");
    }

    /* Init all DRU queue */
    utcId = UDMA_UTC_ID_MSMC_DRU0;
    numQueue = Udma_druGetNumQueue(drvHandle, utcId);
    if(0U == numQueue)
    {
        App_print("[Error] Invalid queue number!!\n");
    }
    UdmaDruQueueConfig_init(&queueCfg);
    for(queId = CSL_DRU_QUEUE_ID_0; queId < numQueue; queId++)
    {
        retVal = Udma_druQueueConfig(drvHandle, utcId, queId, &queueCfg);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] DRU queue config failed!!\n");
            break;
        }
    }

    return (retVal);
}

static int32_t App_deinit(Udma_DrvHandle drvHandle)
{
    int32_t     retVal;

    retVal = Udma_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA deinit failed!!\n");
    }

    return (retVal);
}

static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChUtcPrms      utcPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    SemaphoreP_Params   semPrms;

    SemaphoreP_Params_init(&semPrms);
    gUdmaAppDoneSem = SemaphoreP_create(0, &semPrms);
    if(NULL == gUdmaAppDoneSem)
    {
        App_print("[Error] Sem create failed!!\n");
        retVal = UDMA_EFAIL;
    }

    if(UDMA_SOK == retVal)
    {
        /* Init channel parameters */
        chType = UDMA_CH_TYPE_UTC;
        UdmaChPrms_init(&chPrms, chType);
        chPrms.utcId                = UDMA_UTC_ID_MSMC_DRU0;
        /* Ring not used in direct TR submission via DRU */
        chPrms.fqRingPrms.ringMem   = NULL;
        chPrms.cqRingPrms.ringMem   = NULL;
        chPrms.tdCqRingPrms.ringMem = NULL;
        chPrms.fqRingPrms.elemCnt   = 0U;
        chPrms.cqRingPrms.elemCnt   = 0U;
        chPrms.tdCqRingPrms.elemCnt = 0U;

        /* Open channel for DRU */
        retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA channel open failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Config UTC channel */
        UdmaChUtcPrms_init(&utcPrms);
        utcPrms.druOwner    = CSL_DRU_OWNER_DIRECT_TR;
        utcPrms.druQueueId  = CSL_DRU_QUEUE_ID_3;
        retVal = Udma_chConfigUtc(chHandle, &utcPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA UTC channel config failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register TR completion callback */
        eventHandle = &gUdmaTrEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_TR;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = chHandle;
        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
        eventPrms.eventCb           = &App_udmaEventCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA TR event register failed!!\n");
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

    return (retVal);
}

static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle)
{
    int32_t             retVal;
    Udma_EventHandle    eventHandle;

    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA channel disable failed!!\n");
    }

    /* Unregister all events */
    eventHandle = &gUdmaTrEventObj;
    retVal += Udma_eventUnRegister(eventHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA event unregister failed!!\n");
    }

    retVal += Udma_chClose(chHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA channel close failed!!\n");
    }

    if(gUdmaAppDoneSem != NULL)
    {
        SemaphoreP_delete(gUdmaAppDoneSem);
        gUdmaAppDoneSem = NULL;
    }

    return (retVal);
}

static void App_udmaTrInit(Udma_ChHandle chHandle,
                           CSL_UdmapTR *pTr,
                           const void *destBuf,
                           const void *srcBuf,
                           uint32_t length)
{
    /* Setup TR */
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE)         |
                    CSL_FMK(UDMAP_TR_FLAGS_STATIC, FALSE)                                       |
                    CSL_FMK(UDMAP_TR_FLAGS_EOL, FALSE)                                          |   /* NA */
                    CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION)|
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U)                                       |   /* This will come back in TR response */
                    CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);
    pTr->icnt0    = length;
    pTr->icnt1    = 1U;
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) srcBuf;
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem.
                                           Replace with CSL-FL API */
    pTr->dicnt0   = length;
    pTr->dicnt1   = 1U;
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) destBuf;

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
