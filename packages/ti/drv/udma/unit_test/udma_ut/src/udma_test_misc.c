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
 *  \file udma_test_misc.c
 *
 *  \brief UDMA other misc test case file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define UDMA_TEST_OSAL_NUM_FXNS         (11U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static uintptr_t udmaTestOsalDisableAllIntr(void);
static void udmaTestOsalRestoreAllIntr(uintptr_t cookie);
static void udmaTestOsalDisableIntr(uint32_t coreIntrNum);
static void *udmaTestOsalMutexCreate(void);
static void udmaTestOsalMutexDelete(void *mutexHandle);
static void udmaTestOsalMutexLock(void *mutexHandle);
static void udmaTestOsalMutexUnlock(void *mutexHandle);
static void *udmaTestOsalRegisterIntr(Udma_OsalIsrFxn isrFxn,
                                      uint32_t coreIntrNum,
                                      uint32_t intrPriority,
                                      void *arg);
static void udmaTestOsalUnRegisterIntr(void *hwiHandle);
static void udmaTestOsalCacheInv(const void *addr, int32_t size);
static void udmaTestOsalCacheWb(const void *addr, int32_t size);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static volatile uint32_t gUdmaTestOsalPrmsFlag[UDMA_TEST_OSAL_NUM_FXNS];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestPsilMacroTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: PSIL/PDMA Macro Verification Testcase ::\r\n", taskObj->taskId);

    retVal = udmaTestPrintPsilMacro(taskObj);
    retVal += udmaTestPrintPdmaMacro(taskObj);

    return (retVal);
}

int32_t udmaTestOsalPrmsTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;
    Udma_InitPrms   initPrms;
    uint32_t        instId, cnt, maxCnt;
    Udma_DrvHandle  drvHandle;
    UdmaTestObj     *testObj = taskObj->testObj;
    Udma_OsalCachePrms cachePrms;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: OSAL Params Testcase ::\r\n", taskObj->taskId);

    /* Deinit the drivers */
    retVal += udmaTestDeinitDriver(testObj);

    for(cnt = 0U; cnt < UDMA_TEST_OSAL_NUM_FXNS; cnt++)
    {
        gUdmaTestOsalPrmsFlag[cnt] = 0U;
    }

    /* Re-init driver with new params */
    UdmaOsalCachePrms_init(&cachePrms);
    cachePrms.cacheInv  = &udmaTestOsalCacheInv;
    cachePrms.cacheWb   = &udmaTestOsalCacheWb;
    retVal += Udma_osalSetCachePrms(&cachePrms);
    if(UDMA_SOK != retVal)
    {
        GT_0trace(testObj->traceMask, GT_ERR,
                  " Udma_osalSetCachePrms API failed!!\n");
    }

    for(instId = 0U; instId < UDMA_INST_ID_MAX; instId++)
    {
        /* UDMA driver init */
        drvHandle = &testObj->drvObj[instId];
        UdmaInitPrms_init(instId, &initPrms);
        initPrms.virtToPhyFxn       = &Udma_appVirtToPhyFxn;
        initPrms.phyToVirtFxn       = &Udma_appPhyToVirtFxn;
        initPrms.printFxn           = &udmaDrvPrint;

        /* Override with local OSAL API */
        initPrms.osalPrms.disableAllIntr    = &udmaTestOsalDisableAllIntr;
        initPrms.osalPrms.restoreAllIntr    = &udmaTestOsalRestoreAllIntr;
        initPrms.osalPrms.disableIntr       = &udmaTestOsalDisableIntr;
        initPrms.osalPrms.createMutex       = &udmaTestOsalMutexCreate;
        initPrms.osalPrms.deleteMutex       = &udmaTestOsalMutexDelete;
        initPrms.osalPrms.lockMutex         = &udmaTestOsalMutexLock;
        initPrms.osalPrms.unlockMutex       = &udmaTestOsalMutexUnlock;
        initPrms.osalPrms.registerIntr      = &udmaTestOsalRegisterIntr;
        initPrms.osalPrms.unRegisterIntr    = &udmaTestOsalUnRegisterIntr;

        retVal += Udma_init(drvHandle, &initPrms);
        if(UDMA_SOK != retVal)
        {
            GT_1trace(testObj->traceMask, GT_ERR,
                      " UDMA instance %d init failed!!\n", instId);
        }
    }

    /* Do blockcopy so that that all fxns gets called atleast once */
    retVal += udmaTestBlkcpyTc(taskObj);

    /* Deinit the drivers */
    retVal += udmaTestDeinitDriver(testObj);

    /* Check all flags are set */
    maxCnt = UDMA_TEST_OSAL_NUM_FXNS;
    if(TRUE == Udma_isCacheCoherent())
    {
        maxCnt -= 2U;       /* Cache API not called for coherent system. So skip the check */
    }
    for(cnt = 0U; cnt < maxCnt; cnt++)
    {
        if(gUdmaTestOsalPrmsFlag[cnt] == 0U)
        {
            retVal = UDMA_EFAIL;
            GT_1trace(testObj->traceMask, GT_ERR,
                      " Some OSAL fxns not called @ index: %d !!\n", cnt);
        }
    }

    /* Re-init driver with default params */
    retVal += udmaTestInitDriver(testObj);

    return (retVal);
}

int32_t udmaTestTrMakeTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        trSize, trSizeEncoded;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: TR Make Utility Testcase ::\r\n", taskObj->taskId);

    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_0);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_0);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR0  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_1);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_1);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR1  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_2);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_2);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR2  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_3);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_3);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR3  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_4);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_4);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR4  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_5);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_5);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR5  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_8);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_8);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR8  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_9);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_9);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR9  Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_10);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_10);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR10 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_11);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_11);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR11 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);
    trSize        = UdmaUtils_getTrSizeBytes(UDMA_TR_TYPE_15);
    trSizeEncoded = UdmaUtils_getTrSizeEncoded(UDMA_TR_TYPE_15);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " TR15 Size: %d Bytes (%db Encoded)\r\n", trSize, trSizeEncoded);

    return (retVal);
}

int32_t udmaTestStructSizeTc(UdmaTestTaskObj *taskObj)
{
    int32_t retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: UDMA Struct Size Print Testcase ::\r\n", taskObj->taskId);

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Driver Object Size              : %-5d Bytes\r\n", sizeof(struct Udma_DrvObj));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Channel Object Size             : %-5d Bytes\r\n", sizeof(struct Udma_ChObj));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Event Object Size               : %-5d Bytes\r\n", sizeof(struct Udma_EventObj));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Ring Object Size                : %-5d Bytes\r\n", sizeof(struct Udma_RingObj));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Flow Object Size                : %-5d Bytes\r\n", sizeof(struct Udma_FlowObj));

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_InitPrms Size              : %-5d Bytes\r\n", sizeof(Udma_InitPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_RmInitPrms Size            : %-5d Bytes\r\n", sizeof(Udma_RmInitPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChPrms Size                : %-5d Bytes\r\n", sizeof(Udma_ChPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChTxPrms Size              : %-5d Bytes\r\n", sizeof(Udma_ChTxPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChRxPrms Size              : %-5d Bytes\r\n", sizeof(Udma_ChRxPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChUtcPrms Size             : %-5d Bytes\r\n", sizeof(Udma_ChUtcPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_ChPdmaPrms Size            : %-5d Bytes\r\n", sizeof(Udma_ChPdmaPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_EventPrms Size             : %-5d Bytes\r\n", sizeof(Udma_EventPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_EventRxFlowIdFwStatus Size : %-5d Bytes\r\n", sizeof(Udma_EventRxFlowIdFwStatus));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_RingPrms Size              : %-5d Bytes\r\n", sizeof(Udma_RingPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_FlowPrms Size              : %-5d Bytes\r\n", sizeof(Udma_FlowPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_OsalPrms Size              : %-5d Bytes\r\n", sizeof(Udma_OsalPrms));
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " Udma_OsalCachePrms Size         : %-5d Bytes\r\n", sizeof(Udma_OsalCachePrms));

    return (retVal);
}

static uintptr_t udmaTestOsalDisableAllIntr(void)
{
    uintptr_t cookie = HwiP_disable();
    gUdmaTestOsalPrmsFlag[0U]++;
    return (cookie);
}

static void udmaTestOsalRestoreAllIntr(uintptr_t cookie)
{
    gUdmaTestOsalPrmsFlag[1U]++;
    HwiP_restore(cookie);

    return;
}

static void udmaTestOsalDisableIntr(uint32_t coreIntrNum)
{
    int32_t     corepacEventNum = 0U;

    gUdmaTestOsalPrmsFlag[2U]++;
    Osal_DisableInterrupt(corepacEventNum, coreIntrNum);

    return;
}

static void *udmaTestOsalMutexCreate(void)
{
    SemaphoreP_Params   semPrms;
    SemaphoreP_Handle   mutexHandle;

    SemaphoreP_Params_init(&semPrms);
    semPrms.mode = SemaphoreP_Mode_BINARY;
    mutexHandle = (void *) SemaphoreP_create(1U, &semPrms);

    gUdmaTestOsalPrmsFlag[3U]++;

    return (mutexHandle);
}

static void udmaTestOsalMutexDelete(void *mutexHandle)
{
    gUdmaTestOsalPrmsFlag[4U]++;
    SemaphoreP_delete((SemaphoreP_Handle) mutexHandle);
}

static void udmaTestOsalMutexLock(void *mutexHandle)
{
    gUdmaTestOsalPrmsFlag[5U]++;
    SemaphoreP_pend((SemaphoreP_Handle) mutexHandle, SemaphoreP_WAIT_FOREVER);
}

static void udmaTestOsalMutexUnlock(void *mutexHandle)
{
    gUdmaTestOsalPrmsFlag[6U]++;
    SemaphoreP_post((SemaphoreP_Handle) mutexHandle);
}

static void *udmaTestOsalRegisterIntr(Udma_OsalIsrFxn isrFxn,
                                      uint32_t coreIntrNum,
                                      uint32_t intrPriority,
                                      void *arg)
{
    OsalRegisterIntrParams_t    intrPrms;
    OsalInterruptRetCode_e      osalRetVal;
    HwiP_Handle                 hwiHandle = NULL;

    Osal_RegisterInterrupt_initParams(&intrPrms);

    /* Populate the interrupt parameters */
    intrPrms.corepacConfig.arg              = (uintptr_t) arg;
    intrPrms.corepacConfig.isrRoutine       = isrFxn;
    intrPrms.corepacConfig.priority         = intrPriority;
#if defined (_TMS320C6X)
    /* On C66x, we use Event Combiner to map the interrupt to the CPU Intc.  To
     * do this, OSAL expects that event number holds the interrupt number and we
     * use the macro for interrupt number to specify we wish to use Event
     * Combiner.
     */
    intrPrms.corepacConfig.corepacEventNum  = coreIntrNum;
    intrPrms.corepacConfig.intVecNum        = OSAL_REGINT_INTVEC_EVENT_COMBINER;
#else
    /* Other (non-C66x) CPUs don't use event number and interrupt number is
     * passed in and programmed to CPU Intc directly.
     */
    intrPrms.corepacConfig.corepacEventNum  = 0U;
    intrPrms.corepacConfig.intVecNum        = coreIntrNum;
#endif

    /* Register interrupts */
    osalRetVal = Osal_RegisterInterrupt(&intrPrms, &hwiHandle);
    if(OSAL_INT_SUCCESS != osalRetVal)
    {
        hwiHandle = NULL;
    }

    gUdmaTestOsalPrmsFlag[7U]++;

    return (hwiHandle);
}

static void udmaTestOsalUnRegisterIntr(void *hwiHandle)
{
    int32_t     corepacEventNum = 0U;

    /* Delete interrupts */
    Osal_DeleteInterrupt((HwiP_Handle) hwiHandle, corepacEventNum);

    gUdmaTestOsalPrmsFlag[8U]++;

    return;
}

static void udmaTestOsalCacheInv(const void *addr, int32_t size)
{
    CacheP_Inv(addr, size);
    gUdmaTestOsalPrmsFlag[9U]++;

    return;
}

static void udmaTestOsalCacheWb(const void *addr, int32_t size)
{
    CacheP_wb(addr, size);
    gUdmaTestOsalPrmsFlag[10U]++;

    return;
}
