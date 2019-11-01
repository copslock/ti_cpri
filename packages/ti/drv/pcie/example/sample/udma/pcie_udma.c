/*
 *  Copyright (c) Texas Instruments Incorporated 2018-2019
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
 *  \file pcie_udma.c
 *
 *  \brief UDMA memory copy sample application performing block copy using
 *  Type 15 Transfer Record (TR15) using Transfer Record Packet
 *  Descriptor (TRPD).
 *
 *  Requirement: DOX_REQ_TAG(PRSDK-3851)
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#ifdef QOS
#include "pcie_qos_sample.h"
#else
#include "pcie_sample.h"
#endif
#include "pcie_udma.h"

#include <stdio.h>
#include <ti/drv/udma/udma.h>
#include <ti/drv/sciclient/sciclient.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * Application test parameters
 */
/** \brief Number of times to perform the memcpy operation */
#ifdef QOS
#define PCIE_TEST_APP_UDMA_LOOPS          (10U)
#else
#define PCIE_TEST_APP_UDMA_LOOPS          (1U)
#endif
/*
 * Ring parameters
 */
/** \brief Number of ring entries - we can prime this much memcpy operations */
#define PCIE_TEST_APP_RING_ENTRIES      (1U)
/** \brief Size (in bytes) of each ring entry (Size of pointer - 64-bit) */
#define PCIE_TEST_APP_RING_ENTRY_SIZE   (sizeof(uint64_t))
/** \brief Total ring memory */
#define PCIE_TEST_APP_RING_MEM_SIZE     (PCIE_TEST_APP_RING_ENTRIES * \
                                         PCIE_TEST_APP_RING_ENTRY_SIZE)
/**
 *  \brief UDMA TR packet descriptor memory.
 *  This contains the CSL_UdmapCppi5TRPD + Padding to sizeof(CSL_UdmapTR15) +
 *  one Type_15 TR (CSL_UdmapTR15) + one TR response of 4 bytes.
 *  Since CSL_UdmapCppi5TRPD is less than CSL_UdmapTR15, size is just two times
 *  CSL_UdmapTR15 for alignment.
 */
#define PCIE_TEST_APP_TRPD_SIZE         ((sizeof(CSL_UdmapTR15) * 2U) + 4U)

#if defined (__aarch64__)
// Timestamp for A53 core
#if defined (PCIE_SMP_ENABLE)
#include <ti/sysbios/timers/dmtimer/TimestampProvider.h>
#else
#include <ti/sysbios/family/arm/v8a/TimestampProvider.h>
#endif
#define TIMESTAMP_GETFREQ(x)    TimestampProvider_getFreq(x)
#define TIMESTAMP_GET32()       TimestampProvider_get32()
#else
// Timestamp for R5F core
#include <xdc/runtime/Timestamp.h>
#define TIMESTAMP_GETFREQ(x)    Timestamp_getFreq(x)
#define TIMESTAMP_GET32()       Timestamp_get32()
#endif

#ifdef QOS
#define LOGSIZE                 4096 
uint32_t readLatency[LOGSIZE] __attribute__ ((aligned (64)));
#ifdef __TI_ARM_V7R4__
#pragma DATA_SECTION(readLatency, ".statBuf")
#endif
extern unsigned int lowPriAddr[3];
#endif
uint32_t loop_counter = 0;
uint32_t t_overhead;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

struct QosSetup {
    uint8_t priority;
    uint8_t qosLevel;
    uint8_t orderId;
    uint8_t dmaPriority;
};
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t pcieUdmaLoop (Udma_ChHandle chHandle,
                             void *src,
                             void *dst,
                             uint32_t length);

static int32_t pcieUdmaOneCopy (Udma_ChHandle chHandle,
                                void *destBuf,
                                void *srcBuf,
                                uint32_t length);

static void pcieUdmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData);
static void pcieUdmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData);

static int32_t pcieUdmaInit(Udma_DrvHandle drvHandle);
static int32_t pcieUdmaDeinit(Udma_DrvHandle drvHandle);

static int32_t pcieUdmaCreate(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle, struct QosSetup tx, struct QosSetup rx);
static int32_t pcieUdmaDelete(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle);

static void pcieUdmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *pTrpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length);

extern uint32_t get_clock(void);

#ifdef QOS
extern void printQosResults(uint32_t iteration, uint32_t dataArray[], uint32_t size);
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/*
 * UDMA driver objects
 */
struct Udma_DrvObj      gUdmaDrvObj;
struct Udma_ChObj       gUdmaChObj;
struct Udma_EventObj    gUdmaCqEventObj;
struct Udma_EventObj    gUdmaTdCqEventObj;

/*
 * UDMA Memories
 */
static uint8_t gTxRingMem[PCIE_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxCompRingMem[PCIE_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gTxTdCompRingMem[PCIE_TEST_APP_RING_MEM_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));
static uint8_t gUdmaTprdMem[PCIE_TEST_APP_TRPD_SIZE] __attribute__((aligned(UDMA_CACHELINE_ALIGNMENT)));

#ifndef QOS
/* Source buffer */
static uint32_t gSrcBuf[PCIE_EXAMPLE_LINE_SIZE];
#endif

/* Semaphore to indicate transfer completion */
static SemaphoreP_Handle gUdmaAppDoneSem = NULL;

/* Global test pass/fail flag */
static volatile int32_t gUdmaAppResult = UDMA_SOK;

static struct QosSetup txQosSetup = {5, 2, 7, 3};
static struct QosSetup rxQosSetup = {6, 7, 9, 3};

#ifdef __aarch64__
#define CPU_FREQ (800000000u)
#else
/* R5 */
#define CPU_FREQ (400000000u)
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * UDMA memcpy test
 */
int32_t pcieUdmaTest (void *pcieWindow, uint32_t windowSize)
{
    int32_t                 retVal;
    Sciclient_ConfigPrms_t  sciClientCfg;
    Udma_DrvHandle          drvHandle = &gUdmaDrvObj;
    Udma_ChHandle           chHandle = &gUdmaChObj;

    Sciclient_configPrmsInit(&sciClientCfg);
    Sciclient_init(&sciClientCfg);

    t_overhead = TIMESTAMP_GET32();
    t_overhead = TIMESTAMP_GET32() - t_overhead;

    retVal = pcieUdmaInit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA App init failed!!\n");
    }

    PCIE_logPrintf("UDMA memcpy application started...\n");

    if(UDMA_SOK == retVal)
    {
        retVal = pcieUdmaCreate(drvHandle, chHandle, txQosSetup, rxQosSetup);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA App create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
#ifndef QOS 
		retVal = pcieUdmaLoop(chHandle, gSrcBuf, pcieWindow, windowSize);
#else
	    /* PCIE VC0 read */  
		retVal = pcieUdmaLoop(chHandle, pcieWindow, (void *)lowPriAddr[0], windowSize);
#endif
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA App memcpy test failed!!\n");
        }
    }

    retVal += pcieUdmaDelete(drvHandle, chHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA App delete failed!!\n");
    }

    retVal += pcieUdmaDeinit (drvHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA App deinit failed!!\n");
    }

    if((UDMA_SOK == retVal) && (UDMA_SOK == gUdmaAppResult))
    {
        PCIE_logPrintf("UDMA memcpy using TR15 block copy Passed!!\n");
    }
    else
    {
        PCIE_logPrintf("UDMA memcpy using TR15 block copy Failed!!\n");
    }

    Sciclient_deinit();

    return (retVal);
}

static int32_t pcieUdmaLoop (Udma_ChHandle chHandle, void *src, void *dst, uint32_t size)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            i;
    uint8_t            *srcBuf = (void *)src;
    uint8_t            *destBuf = (void *)dst;

    /* Init buffers */
    for(i = 0U; i < size; i++)
    {
        srcBuf[i] = i;
        destBuf[i] = 0U;
    }
    /* Writeback source and destination buffer if not PCIE data space */
    if ((uint32_t)src >= 0x20000000)  {
        CacheP_wb((void *)src, size);
    }
    if ((uint32_t)dst >= 0x20000000)  {
        CacheP_wb((void *)dst, size);
    }

    /* Perform UDMA memcpy */
    for (i = 0; i < PCIE_TEST_APP_UDMA_LOOPS; i++)
    {
        retVal = pcieUdmaOneCopy (
                     chHandle,
                     destBuf,
                     srcBuf,
                     size);
        if(UDMA_SOK != retVal)
        {
            break;
        }
#ifdef QOS
    printQosResults(i, readLatency, LOGSIZE);
#endif
    }
#ifdef QOS
    PCIE_logPrintf("Checking RC side readLatency buffer for the last iteration log if needed\n");
#endif
    return (retVal);
}

static int32_t pcieUdmaOneCopy (Udma_ChHandle chHandle,
                                void *destBuf,
                                void *srcBuf,
                                uint32_t length)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t   *pTrResp, trRespStatus;
    uint64_t    pDesc = 0;
    uint8_t    *tprdMem = &gUdmaTprdMem[0U];
    uint32_t    t_read;
#ifndef QOS
    float       speed;
#else
    uint32_t    loop = 0;
#endif	
    /* Update TR packet descriptor */
    pcieUdmaTrpdInit(chHandle, tprdMem, destBuf, srcBuf, length);

    /* Submit TRPD to channel */
    t_read = TIMESTAMP_GET32();
    retVal = Udma_ringQueueRaw(
                 Udma_chGetFqRingHandle(chHandle), (uint64_t) tprdMem);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] Channel queue failed!!\n");
    }

#ifndef QOS 
    if(UDMA_SOK == retVal)
    {
        /* Wait for return descriptor in completion ring - this marks the
         * transfer completion */
        SemaphoreP_pend(gUdmaAppDoneSem, SemaphoreP_WAIT_FOREVER);
        t_read = TIMESTAMP_GET32() - t_read - t_overhead;
        /* Response received in completion queue */
        speed = (float)length;
        speed *= 8;
        speed /= (((float)t_read)/CPU_FREQ);
        speed /= 1000000;
        PCIE_logPrintf("Speed: %d Mbps\n", (int)speed);
        retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] No descriptor after callback!!\n");
            retVal = UDMA_EFAIL;
        }
    }
#else
	if(UDMA_SOK == retVal)
    {
#if defined (__TI_ARM_V7R4__)
        asm (" cpsid if ");
#endif
		/* The PCIE VC3 read with CPU is performed in the gap of UDMA transfer
           The total read duration needs to be shorter than the gap */ 
		while(loop < LOGSIZE) {
			t_read = TIMESTAMP_GET32();
			BARRIER
			(void)*(unsigned volatile int*)(uintptr_t)(PCIE_WINDOW_MEM_BASE_VC3);
			BARRIER
			t_read = TIMESTAMP_GET32() - t_read - t_overhead;
			readLatency[loop++] = t_read;
	    }
#if defined (__TI_ARM_V7R4__)		
		asm (" cpsie if ");
#endif		
		/* Wait for return descriptor in completion ring - this marks the
         * transfer completion */
		 
        SemaphoreP_pend(gUdmaAppDoneSem, SemaphoreP_WAIT_FOREVER);

        /* Response received in completion queue */
        retVal = Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chHandle), &pDesc);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] No descriptor after callback!!\n");
            retVal = UDMA_EFAIL;
        }
    }
#endif
    if(UDMA_SOK == retVal)
    {
        /*
         * Sanity check
         */
        /* Check returned descriptor pointer */
        if(pDesc != ((uint64_t) tprdMem))
        {
            PCIE_logPrintf("[Error] TR descriptor pointer returned doesn't "
                   "match the submitted address!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Invalidate cache */
        CacheP_Inv(&gUdmaTprdMem[0U], PCIE_TEST_APP_TRPD_SIZE);

        /* check TR response status */
        pTrResp = (uint32_t *) (tprdMem + (sizeof(CSL_UdmapTR15) * 2U));
        trRespStatus = CSL_FEXT(*pTrResp, UDMAP_TR_RESPONSE_STATUS_TYPE);
        if(trRespStatus != CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE)
        {
            PCIE_logPrintf("[Error] TR Response not completed!!\n");
            retVal = UDMA_EFAIL;
        }
    }

    return (retVal);
}

static void pcieUdmaEventDmaCb(Udma_EventHandle eventHandle,
                               uint32_t eventType,
                               void *appData)
{
    if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
    {
        SemaphoreP_post(gUdmaAppDoneSem);
    }
    else
    {
        gUdmaAppResult = UDMA_EFAIL;
    }

    return;
}

static void pcieUdmaEventTdCb(Udma_EventHandle eventHandle,
                              uint32_t eventType,
                              void *appData)
{
    int32_t             retVal;
    CSL_UdmapTdResponse tdResp;

    if(UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventType)
    {
        /* Response received in Teardown completion queue */
        retVal = Udma_chDequeueTdResponse(&gUdmaChObj, &tdResp);
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

static int32_t pcieUdmaInit(Udma_DrvHandle drvHandle)
{
    int32_t         retVal;
    Udma_InitPrms   initPrms;
    uint32_t        instId;

    /* UDMA driver init */
#if defined (__aarch64__)
    instId = UDMA_INST_ID_MAIN_0;
#else
#ifdef QOS
	instId = UDMA_INST_ID_MAIN_0;
#else
    instId = UDMA_INST_ID_MCU_0;
#endif
#endif
    UdmaInitPrms_init(instId, &initPrms);
    initPrms.printFxn = (Udma_PrintFxn)&PCIE_logPrintf;
    retVal = Udma_init(drvHandle, &initPrms);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA init failed!!\n");
    }

    return (retVal);
}

static int32_t pcieUdmaDeinit (Udma_DrvHandle drvHandle)
{
    int32_t     retVal;

    retVal = Udma_deinit(drvHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA deinit failed!!\n");
    }

    return (retVal);
}

static int32_t pcieUdmaCreate(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle, struct QosSetup txQosSetup, struct QosSetup rxQosSetup)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle;
    Udma_EventPrms      eventPrms;
    SemaphoreP_Params   semPrms;

    SemaphoreP_Params_init(&semPrms);
    gUdmaAppDoneSem = SemaphoreP_create(0, &semPrms);
    if(NULL == gUdmaAppDoneSem)
    {
        PCIE_logPrintf("[Error] Sem create failed!!\n");
        retVal = UDMA_EFAIL;
    }

    if(UDMA_SOK == retVal)
    {
        /* Init channel parameters */
        chType = UDMA_CH_TYPE_TR_BLK_COPY;
        UdmaChPrms_init(&chPrms, chType);
        chPrms.fqRingPrms.ringMem   = &gTxRingMem[0U];
        chPrms.cqRingPrms.ringMem   = &gTxCompRingMem[0U];
        chPrms.tdCqRingPrms.ringMem = &gTxTdCompRingMem[0U];
        chPrms.fqRingPrms.elemCnt   = PCIE_TEST_APP_RING_ENTRIES;
        chPrms.cqRingPrms.elemCnt   = PCIE_TEST_APP_RING_ENTRIES;
        chPrms.tdCqRingPrms.elemCnt = PCIE_TEST_APP_RING_ENTRIES;

        /* Open channel for block copy */
        retVal = Udma_chOpen(drvHandle, chHandle, chType, &chPrms);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA channel open failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Config TX channel */
        UdmaChTxPrms_init(&txPrms, chType);
        txPrms.busPriority = txQosSetup.priority;
        txPrms.busQos = txQosSetup.qosLevel;
        txPrms.busOrderId = txQosSetup.orderId;
        txPrms.dmaPriority = txQosSetup.dmaPriority;
        retVal = Udma_chConfigTx(chHandle, &txPrms);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA TX channel config failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Config RX channel - which is implicitly paired to TX channel in
         * block copy mode */
        UdmaChRxPrms_init(&rxPrms, chType);
        rxPrms.busPriority = rxQosSetup.priority;
        rxPrms.busQos = rxQosSetup.qosLevel;
        rxPrms.busOrderId = rxQosSetup.orderId;
        rxPrms.dmaPriority = rxQosSetup.dmaPriority;
        retVal = Udma_chConfigRx(chHandle, &rxPrms);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA RX channel config failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register ring completion callback */
        eventHandle = &gUdmaCqEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = chHandle;
        eventPrms.masterEventHandle = NULL;
        eventPrms.eventCb           = &pcieUdmaEventDmaCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA CQ event register failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register teardown ring completion callback */
        eventHandle = &gUdmaTdCqEventObj;
        UdmaEventPrms_init(&eventPrms);
        eventPrms.eventType         = UDMA_EVENT_TYPE_TEARDOWN_PACKET;
        eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
        eventPrms.chHandle          = chHandle;
        eventPrms.masterEventHandle = &gUdmaCqEventObj;
        eventPrms.eventCb           = &pcieUdmaEventTdCb;
        retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA Teardown CQ event register failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Channel enable */
        retVal = Udma_chEnable(chHandle);
        if(UDMA_SOK != retVal)
        {
            PCIE_logPrintf("[Error] UDMA channel enable failed!!\n");
        }
    }

    return (retVal);
}

static int32_t pcieUdmaDelete(Udma_DrvHandle drvHandle, Udma_ChHandle chHandle)
{
    int32_t             retVal, tempRetVal;
    uint64_t            pDesc;
    Udma_EventHandle    eventHandle;

    retVal = Udma_chDisable(chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA channel disable failed!!\n");
    }

    /* Unregister master event at the end */
    eventHandle = &gUdmaTdCqEventObj;
    retVal += Udma_eventUnRegister(eventHandle);
    eventHandle = &gUdmaCqEventObj;
    retVal += Udma_eventUnRegister(eventHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA event unregister failed!!\n");
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

    retVal += Udma_chClose(chHandle);
    if(UDMA_SOK != retVal)
    {
        PCIE_logPrintf("[Error] UDMA channel close failed!!\n");
    }

    if(gUdmaAppDoneSem != NULL)
    {
        SemaphoreP_delete(gUdmaAppDoneSem);
        gUdmaAppDoneSem = NULL;
    }

    return (retVal);
}

static void pcieUdmaTrpdInit(Udma_ChHandle chHandle,
                             uint8_t *pTrpdMem,
                             const void *destBuf,
                             const void *srcBuf,
                             uint32_t length)
{
    CSL_UdmapCppi5TRPD *pTrpd = (CSL_UdmapCppi5TRPD *) pTrpdMem;
    CSL_UdmapTR15 *pTr = (CSL_UdmapTR15 *)(pTrpdMem + sizeof(CSL_UdmapTR15));
    uint32_t *pTrResp = (uint32_t *) (pTrpdMem + (sizeof(CSL_UdmapTR15) * 2U));
    uint32_t cqRingNum = Udma_chGetCqRingNum(chHandle);
    uint32_t descType = CSL_UDMAP_CPPI5_PD_DESCINFO_DTYPE_VAL_TR;

    /* Setup descriptor */
    CSL_udmapCppi5SetDescType(pTrpd, descType);
    CSL_udmapCppi5TrSetReload(pTrpd, 0U, 0U);
    CSL_udmapCppi5SetPktLen(pTrpd, descType, 1U);       /* Only one TR in TRPD */
    CSL_udmapCppi5SetIds(pTrpd, descType, 0U, UDMA_DEFAULT_FLOW_ID); /* Flow ID and Packet ID */
    CSL_udmapCppi5SetSrcTag(pTrpd, 0x0000);     /* Not used */
    CSL_udmapCppi5SetDstTag(pTrpd, 0x0000);     /* Not used */
    CSL_udmapCppi5TrSetEntryStride(
        pTrpd, CSL_UDMAP_CPPI5_TRPD_PKTINFO_RECSIZE_VAL_64B);
    CSL_udmapCppi5SetReturnPolicy(
        pTrpd,
        descType,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPOLICY_VAL_ENTIRE_PKT,   /* Don't care for TR */
        CSL_UDMAP_CPPI5_PD_PKTINFO2_EARLYRET_VAL_NO,
        CSL_UDMAP_CPPI5_PD_PKTINFO2_RETPUSHPOLICY_VAL_TO_TAIL,
        cqRingNum);

    /* Setup TR */
    pTr->flags    = CSL_FMK(UDMAP_TR_FLAGS_TYPE, 15)                                            |
                    CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U)                                          |
                    CSL_FMK(UDMAP_TR_FLAGS_EOL, 0U)                                             |   /* NA */
                    CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION)|
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE)           |
                    CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL)  |
                    CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U)                                       |   /* This will come back in TR response */
                    CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U)                                     |
                    CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    /* Assume length is multiple of 32KB*/
    if (length <= 32768) {
		pTr->icnt0    = length;
		pTr->icnt1    = 1U;
	} else {
		pTr->icnt0    = 32768;
		pTr->icnt1    = length/32768U;
	}
    pTr->icnt2    = 1U;
    pTr->icnt3    = 1U;
    pTr->dim1     = pTr->icnt0;
    pTr->dim2     = (pTr->icnt0 * pTr->icnt1);
    pTr->dim3     = (pTr->icnt0 * pTr->icnt1 * pTr->icnt2);
    pTr->addr     = (uint64_t) srcBuf;
    pTr->fmtflags = 0x00000000U;        /* Linear addressing, 1 byte per elem.
                                           Replace with CSL-FL API */
    /* Assume length is multiple of 32KB*/
    if (length <= 32768) {
		pTr->dicnt0    = length;
		pTr->dicnt1    = 1U;
	} else {
		pTr->dicnt0    = 32768;
		pTr->dicnt1    = length/32768U;
	}
    pTr->dicnt2   = 1U;
    pTr->dicnt3   = 1U;
    pTr->ddim1    = pTr->dicnt0;
    pTr->ddim2    = (pTr->dicnt0 * pTr->dicnt1);
    pTr->ddim3    = (pTr->dicnt0 * pTr->dicnt1 * pTr->dicnt2);
    pTr->daddr    = (uint64_t) destBuf;

    /* Clear TR response memory */
    *pTrResp = 0xFFFFFFFFU;

    /* Writeback cache */
    CacheP_wb(pTrpdMem, PCIE_TEST_APP_TRPD_SIZE);

    return;
}
