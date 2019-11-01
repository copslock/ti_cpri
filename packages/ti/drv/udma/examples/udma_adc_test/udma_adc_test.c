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
 *  \file udma_adc_test.c
 *
 *  \brief UDMA ADC Example. This performs PDMA RX data capture from ADC.
 *  ADC is configured in single shot mode and captures APP_ADC_NUM_CH channel
 *  of ADC data. The FIFO is configured to generate a DMA trigger after all
 *  channel data is captured.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <ti/drv/udma/udma.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_adc.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
#include <ti/drv/udma/examples/udma_apputils/udma_apputils.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Application test parameters
 */
#define APP_ADC_MODULE                  (CSL_MCU_ADC0_BASE)
#define APP_ADC_FIFO                    (ADC_FIFO_NUM_0)
#define APP_ADC_RX_PDMA_CH              (UDMA_PDMA_CH_MCU_ADC0_CH0_RX)
#define APP_ADC_NUM_CH                  (8U)

/*
 * Number of bytes transmitted by PDMA per RX event sent by ADC.
 * In ADC, this should be equal to the DMA trigger.
 * Testing single shot mode - so should be same as number of channels being
 * captured
 */
#define RX_BYTES_PER_EVENT              (APP_ADC_NUM_CH)

/** \brief Number of times to perform the ADC operation */
#define UDMA_TEST_APP_LOOP_CNT          (10U)

/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much ADC operations */
#define UDMA_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define UDMA_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define UDMA_TEST_APP_RING_MEM_SIZE     (UDMA_TEST_APP_RING_ENTRIES * \
                                         UDMA_TEST_APP_RING_ENTRY_SIZE)
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_RING_MEM_SIZE_ALIGN ((UDMA_TEST_APP_RING_MEM_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))
/** \brief UDMA host mode buffer descriptor memory size. */
#define UDMA_TEST_APP_DESC_SIZE         (sizeof(CSL_UdmapCppi5HMPD))
/** \brief This ensures every channel memory is aligned */
#define UDMA_TEST_APP_DESC_SIZE_ALIGN   ((UDMA_TEST_APP_DESC_SIZE + UDMA_CACHELINE_ALIGNMENT) & ~(UDMA_CACHELINE_ALIGNMENT - 1U))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t App_adcTest(Udma_ChHandle rxChHandle);
static int32_t App_udmaAdcRx(Udma_ChHandle rxChHandle, uint32_t *destBuf);

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData);
static void App_udmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData);

static int32_t App_init(Udma_DrvHandle drvHandle);
static int32_t App_deinit(Udma_DrvHandle drvHandle);
static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle);
static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle);

static void App_udmaRxHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const uint32_t *destBuf,
                              uint32_t length);

static void App_adcInit(void);
static void App_adcConfig(void);
static void App_adcStart(void);
static void App_adcStop(void);
static void App_adcDeInit(void);

static void App_print(const char *str);
static void App_printNum(const char *str, uint32_t num);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaRxChObj;
struct Udma_EventObj    gUdmaCqEventObj;
struct Udma_EventObj    gUdmaTdCqEventObj;

/*
 * UDMA Memories
 */
static uint8_t gRxFqRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gRxCqRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gRxTdCqRingMem[UDMA_TEST_APP_RING_MEM_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaRxHpdMem[UDMA_TEST_APP_DESC_SIZE_ALIGN] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

/*
 * Application Buffers
 */
uint32_t gAdcDestBuf[APP_ADC_NUM_CH] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

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
int32_t Udma_adcTest(void)
{
    int32_t         retVal;
    Udma_DrvHandle  drvHandle = &gUdmaDrvObj;
    Udma_ChHandle   rxChHandle = &gUdmaRxChObj;

    App_print("UDMA ADC application started...\n");

    retVal = App_init(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA App init failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_create(drvHandle, rxChHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = App_adcTest(rxChHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA App ADC test failed!!\n");
        }
    }

    retVal += App_delete(drvHandle, rxChHandle);
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
        App_print("UDMA ADC Test Passed!!\n");
        App_print("All tests have passed!!\n");
    }
    else
    {
        App_print("UDMA ADC Test Failed!!\n");
        App_print("Some tests have failed!!\n");
    }

    return (0);
}

static int32_t App_adcTest(Udma_ChHandle rxChHandle)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;
    uint32_t   *destBuf = &gAdcDestBuf[0U];

    while(loopCnt < UDMA_TEST_APP_LOOP_CNT)
    {
        /* Perform UDMA ADC RX */
        retVal = App_udmaAdcRx(rxChHandle, destBuf);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
        App_printNum("Loop Count: %d completed!!\n\n", loopCnt);
    }

    return (retVal);
}

static int32_t App_udmaAdcRx(Udma_ChHandle rxChHandle, uint32_t *destBuf)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        loopCnt, fifoData, adcData, stepId, chCnt;
    Udma_ChPdmaPrms pdmaPrms;
    uint64_t        pDesc = 0;
    uint8_t        *pHpdMem = &gUdmaRxHpdMem[0U];

    App_adcConfig();

    /* Config PDMA channel */
    UdmaChPdmaPrms_init(&pdmaPrms);
    pdmaPrms.elemSize   = UDMA_PDMA_ES_32BITS;
    pdmaPrms.elemCnt    = RX_BYTES_PER_EVENT;
    pdmaPrms.fifoCnt    = (APP_ADC_NUM_CH / RX_BYTES_PER_EVENT);
    retVal = Udma_chConfigPdma(rxChHandle, &pdmaPrms);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA RX PDMA config failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_chEnable(rxChHandle);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA RX channel enable failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Update host packet descriptor */
        App_udmaRxHpdInit(rxChHandle, pHpdMem, destBuf, APP_ADC_NUM_CH * 4U);

        /* Submit HPD to channel */
        retVal = Udma_ringQueueRaw(
                     Udma_chGetFqRingHandle(rxChHandle), (uint64_t) pHpdMem);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] Channel queue failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        App_adcStart();

        /* Wait for return descriptor in completion ring - this marks the
         * transfer completion */
        SemaphoreP_pend(gUdmaAppDoneSem, SemaphoreP_WAIT_FOREVER);

        /* Response received in completion queue */
        retVal = Udma_ringDequeueRaw(
                     Udma_chGetCqRingHandle(rxChHandle), &pDesc);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] No descriptor after callback!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /*
         * Sanity check
         */
        /* Check returned descriptor pointer */
        if(pDesc != ((uint64_t) pHpdMem))
        {
            App_print("[Error] Host packet descriptor pointer returned doesn't "
                   "match the submitted address!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Invalidate cache */
        Udma_appUtilsCacheInv(pHpdMem, UDMA_TEST_APP_DESC_SIZE);
        Udma_appUtilsCacheInv(&gAdcDestBuf[0U], sizeof(gAdcDestBuf));
        for (loopCnt = 0U; loopCnt < APP_ADC_NUM_CH; loopCnt++)
        {
            chCnt = loopCnt % APP_ADC_NUM_CH;
            fifoData = destBuf[loopCnt];
            stepId   = ((fifoData & ADC_FIFODATA_ADCCHNLID_MASK) >>
                        ADC_FIFODATA_ADCCHNLID_SHIFT);
            adcData = ((fifoData & ADC_FIFODATA_ADCDATA_MASK) >>
                        ADC_FIFODATA_ADCDATA_SHIFT);
            if(stepId != chCnt)     /* Both channel and step are 1:1 mapped */
            {
                retVal = UDMA_EFAIL;
                App_printNum("Step ID Error: %d\n", stepId);
            }
            App_printNum("CH %d ", chCnt);
            App_printNum("DATA: 0x%0.4x\n", adcData);
        }
    }

    App_adcStop();

    retVal += Udma_chDisable(rxChHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA channel disable failed!!\n");
    }

    return (retVal);
}

static void App_udmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData)
{
    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {
        SemaphoreP_post(gUdmaAppDoneSem);
    }

    return;
}

static void App_udmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData)
{
    int32_t             retVal;
    CSL_UdmapTdResponse tdResp;

    if(UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventType)
    {
        /* Response received in Teardown completion queue */
        retVal = Udma_chDequeueTdResponse(&gUdmaRxChObj, &tdResp);
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

    return;
}

static int32_t App_init(Udma_DrvHandle drvHandle)
{
    int32_t         retVal;
    Udma_InitPrms   initPrms;
    uint32_t        instId;

    /* UDMA driver init */
    instId = UDMA_INST_ID_MCU_0;        /* Use the same domain NAVSS instance - ADC is in MCU domain */
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = &App_print;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA init failed!!\n");
    }

    App_adcInit();

    return (retVal);
}

static int32_t App_deinit(Udma_DrvHandle drvHandle)
{
    int32_t     retVal;

    App_adcDeInit();

    retVal = Udma_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA deinit failed!!\n");
    }

    return (retVal);
}

static int32_t App_create(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChRxPrms       rxPrms;
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
        chType = UDMA_CH_TYPE_PDMA_RX;
        UdmaChPrms_init(&chPrms, chType);
        chPrms.peerChNum            = APP_ADC_RX_PDMA_CH;
        chPrms.fqRingPrms.ringMem   = &gRxFqRingMem[0U];
        chPrms.cqRingPrms.ringMem   = &gRxCqRingMem[0U];
        chPrms.tdCqRingPrms.ringMem = &gRxTdCqRingMem[0U];
        chPrms.fqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
        chPrms.cqRingPrms.ringMemSize   = UDMA_TEST_APP_RING_MEM_SIZE;
        chPrms.tdCqRingPrms.ringMemSize = UDMA_TEST_APP_RING_MEM_SIZE;
        chPrms.fqRingPrms.elemCnt   = UDMA_TEST_APP_RING_ENTRIES;
        chPrms.cqRingPrms.elemCnt   = UDMA_TEST_APP_RING_ENTRIES;
        chPrms.tdCqRingPrms.elemCnt = UDMA_TEST_APP_RING_ENTRIES;

        /* Open channel for block copy */
        retVal = Udma_chOpen(drvHandle, rxChHandle, chType, &chPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA channel open failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Config RX channel */
        UdmaChRxPrms_init(&rxPrms, UDMA_CH_TYPE_PDMA_RX);
        retVal = Udma_chConfigRx(rxChHandle, &rxPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA RX channel config failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register ring completion callback */
        eventHandle = &gUdmaCqEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = rxChHandle;
        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
        eventPrms.eventCb           = &App_udmaEventDmaCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA CQ event register failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register teardown ring completion callback */
        eventHandle = &gUdmaTdCqEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_TEARDOWN_PACKET;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = rxChHandle;
        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
        eventPrms.eventCb           = &App_udmaEventTdCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            App_print("[Error] UDMA Teardown CQ event register failed!!\n");
        }
    }

    return (retVal);
}

static int32_t App_delete(Udma_DrvHandle drvHandle, Udma_ChHandle rxChHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_EventHandle    eventHandle;

    /* Unregister all events */
    eventHandle = &gUdmaTdCqEventObj;
    retVal += Udma_eventUnRegister(eventHandle);
    eventHandle = &gUdmaCqEventObj;
    retVal += Udma_eventUnRegister(eventHandle);
    if(UDMA_SOK != retVal)
    {
        App_print("[Error] UDMA event unregister failed!!\n");
    }

    retVal += Udma_chClose(rxChHandle);
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

static void App_udmaRxHpdInit(Udma_ChHandle rxChHandle,
                              uint8_t *pHpdMem,
                              const uint32_t *destBuf,
                              uint32_t length)
{
    CSL_UdmapCppi5HMPD *pHpd = (CSL_UdmapCppi5HMPD *) pHpdMem;
    uint32_t descType = (uint32_t)CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_HOST;
    uint32_t cqRingNum = Udma_chGetCqRingNum(rxChHandle);

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pHpd, descType);
    CSL_udmapCppi5SetEpiDataPresent(pHpd, FALSE);
    CSL_udmapCppi5SetPsDataLoc(pHpd, 0U);
    CSL_udmapCppi5SetPsDataLen(pHpd, 0U);
    CSL_udmapCppi5SetPktLen(pHpd, descType, length);
    CSL_udmapCppi5SetPsFlags(pHpd, 0U);
    CSL_udmapCppi5SetIds(pHpd, descType, 0x321, UDMA_DEFAULT_FLOW_ID);
    CSL_udmapCppi5SetSrcTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pHpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetReturnPolicy(
        pHpd,
        descType,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
        cqRingNum);
    CSL_udmapCppi5LinkDesc(pHpd, 0U);
    CSL_udmapCppi5SetBufferAddr(pHpd, (uint64_t) destBuf);
    CSL_udmapCppi5SetBufferLen(pHpd, length);
    CSL_udmapCppi5SetOrgBufferAddr(pHpd, (uint64_t) destBuf);
    CSL_udmapCppi5SetOrgBufferLen(pHpd, length);

    /* Writeback cache */
    Udma_appUtilsCacheWb(pHpdMem, UDMA_TEST_APP_DESC_SIZE);

    return;
}

static void App_adcInit(void)
{
    /* Clear All interrupt status */
    ADCClearIntrStatus(APP_ADC_MODULE, ADC_INTR_STATUS_ALL);

    /* Power up AFE */
    ADCPowerUp(APP_ADC_MODULE, TRUE);
    /* Wait for 4us at least */
    Osal_delay(1);

    /* Do the internal calibration */
    ADCInit(APP_ADC_MODULE, FALSE, 0U, 0U);

    return;
}

static void App_adcConfig(void)
{
    uint32_t        chCnt;
    adcStepConfig_t adcConfig;

    /* Initialize ADC configuration params */
    adcConfig.mode             = ADC_OPERATION_MODE_SINGLE_SHOT;
    adcConfig.openDelay        = 0x1U;
    adcConfig.sampleDelay      = 0U;
    adcConfig.rangeCheckEnable = 0U;
    adcConfig.averaging        = ADC_AVERAGING_16_SAMPLES;
    adcConfig.fifoNum          = APP_ADC_FIFO;
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        /* Step configuration */
        adcConfig.channel = ADC_CHANNEL_1 + chCnt;
        ADCSetStepParams(APP_ADC_MODULE, ADC_STEP_1 + chCnt, &adcConfig);
        /* step enable */
        ADCStepEnable(APP_ADC_MODULE, ADC_STEP_1 + chCnt, TRUE);
    }

    ADCStepIdTagEnable(APP_ADC_MODULE, TRUE);
    ADCSetDMAFIFOThresholdLevel(APP_ADC_MODULE, APP_ADC_FIFO, APP_ADC_NUM_CH);

    return;
}

static void App_adcStart(void)
{
    adcSequencerStatus_t status;

    /* Enable DMA */
    ADCFIFODMAAccessEnable(APP_ADC_MODULE, APP_ADC_FIFO, TRUE);

    /* Check if FSM is idle */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    /* Start ADC conversion */
    ADCStart(APP_ADC_MODULE, TRUE);

    return;
}

static void App_adcStop(void)
{
    uint32_t                chCnt;
    adcSequencerStatus_t    status;

    /* Disable DMA */
    ADCFIFODMAAccessEnable(APP_ADC_MODULE, APP_ADC_FIFO, FALSE);

    /* Disable all/enabled steps */
    for(chCnt = 0U; chCnt < APP_ADC_NUM_CH; chCnt++)
    {
        ADCStepEnable(APP_ADC_MODULE, ADC_STEP_1 + chCnt, FALSE);
    }

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    /* Stop ADC */
    ADCStart(APP_ADC_MODULE, FALSE);

    /* Wait for FSM to go IDLE */
    ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    while((ADC_ADCSTAT_FSM_BUSY_IDLE != status.fsmBusy) &&
           ADC_ADCSTAT_STEP_ID_IDLE != status.stepId)
    {
        ADCGetSequencerStatus(APP_ADC_MODULE, &status);
    }

    return;
}

static void App_adcDeInit(void)
{
    /* Power down ADC */
    ADCPowerUp(APP_ADC_MODULE, FALSE);

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

static void App_printNum(const char *str, uint32_t num)
{
    static char printBuf[200U];

    snprintf(printBuf, 200U, str, num);
    UART_printf("%s", printBuf);

    if(TRUE == Udma_appIsPrintSupported())
    {
        printf("%s", printBuf);
    }

    return;
}
