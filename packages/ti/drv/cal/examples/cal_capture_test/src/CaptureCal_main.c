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
 *  \file CaptureCal_main.c
 *
 *  \brief CAL Capture demo application - Receives via CSI2/Parallel and
 *          write to memory
 *         Most board / EVM specifics are handled in function
 *          appCaptcreateVidSensor () In addition to configuring the sensor
 *          for the given config and enable sensor to stream.
 *         On AM65xx only CSI2 based capture is supported.
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <CaptureCal_main.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**< CSI2 PHY Clock Custom sensor. */
#define APP_CAPT_CAL_SENSOR_BYPASS_PHY_CLK_MHz (400U)

#define APP_CAPT_MAX_WIDTH  (1280U)
#define APP_CAPT_MAX_HEIGHT (720U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void appCaptInit(appCaptObj_t *appObj);
static void appCaptDeInit(appCaptObj_t *appObj);
static void appCapture4GivenConfig(appCaptObj_t *appObj, appCaptCfg_t *pCfg);
static void appCaptDeriveCfg(appCaptObj_t *appObj, appCaptCfg_t *pCfg);
static int32_t appCaptCreateDrv(appCaptObj_t *appObj);
static int32_t appCaptDeleteDrv(appCaptObj_t *appObj);
static int32_t appCaptSetDrvCfg(appCaptObj_t *appObj, appCaptCfg_t *pCfg);
static int32_t appCaptAllocAndQFrames(appCaptObj_t *appObj);
static int32_t appCaptFreeFrames(appCaptObj_t *appObj);
static void appCaptCountQ(appCaptObj_t *appObj, Fvid2_FrameList *frmList,
                          uint32_t streamId);
static void appCaptCountDQ(appCaptObj_t *appObj, Fvid2_FrameList *frmList,
                           uint32_t streamId);
static void appCaptChkDQCounts(appCaptObj_t *appObj);
static int32_t appCaptGetTestId(appCaptObj_t *appObj);
static void appCaptErrCb(const uint32_t *event, uint32_t numEvents, Ptr arg);
static int32_t appCaptEnableErrorReporting(appCaptObj_t *appObj);
static int32_t appCaptEnableFrameEventNotification(appCaptObj_t *appObj,
                                                    appCaptCfg_t *pCfg);
static void appCaptChkFrameEvents(appCaptObj_t *appObj);
void App_print(const char *format, ...);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**< demo application object */
appCaptObj_t gCaptAppObj;

/**< Different capture options supported by this demo */
appCaptCfg_t gTestCfg[] = {
    {"Sensor Config Bypassed CSI2 4Lanes capture color bars from UB954 ",   0U,
     TRUE,
     FVID2_VIFM_SCH_CSI2,
     FVID2_VIFW_4LANES, 1U, 0U, CAL_CSI2_RGB888, FVID2_CCSF_BITS8_PACKED,
     CAPT_APP_RUN_COUNT, 1280U, 720U, (1280U * 3U),
     FVID2_VID_SENSOR_BYPASS_CSI2_DRV,
     FVID2_STD_1080P_30, FVID2_DF_BAYER_BGGR, FVID2_CCSF_BITS8_PACKED
    },
    {"UB964 & SAT0088 and OV10635 CSI2 4 channel capture ",   1U,
     TRUE,
     FVID2_VIFM_SCH_CSI2,
     FVID2_VIFW_4LANES, 4U, 0U, CAL_CSI2_YUV422_8B, FVID2_CCSF_BITS10_PACKED,
     CAPT_APP_RUN_COUNT, 1280U, 720U, (1280U * 2U),
     FVID2_VID_SENSOR_MULDES_OV1063X_DRV,
     FVID2_STD_720P_60, FVID2_DF_BAYER_GRBG, FVID2_CCSF_BITS16_PACKED
    },
};

/**< Number of options supported */
#define APP_CAPT_NUM_OPTS              (sizeof (gTestCfg) / \
                                        sizeof (gTestCfg[0U]))

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Test task main
 */
void Cal_captureTest(void)
{
    CalUtils_MemHeapStatus startHeapStat1;
    appCaptObj_t          *appObj = &gCaptAppObj;
    uint32_t testId;

    memset(&gCaptAppObj, 0x0, sizeof (appCaptObj_t));
    appCaptInit(appObj);

    while (1U)
    {
        /* Get the TestId */
        testId = appCaptGetTestId(appObj);
        if (testId < APP_CAPT_NUM_OPTS)
        {
            CalUtils_memGetHeapStat(&startHeapStat1);

            appCapture4GivenConfig(
                appObj,
                &gTestCfg[testId]);

            CalUtils_memCheckHeapStat(&startHeapStat1);
        }
        else
        {
            /* Exit */
            break;
        }
    }
    GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Sample Application - DONE !!!\r\n");

    appCaptDeInit(appObj);

    return;
}

static void appCaptInit(appCaptObj_t *appObj)
{
    int32_t         retVal;
    Fvid2_InitPrms  initPrms;

    /* Fvid2 init */
    Fvid2InitPrms_init(&initPrms);
    initPrms.printFxn = &App_print;
    retVal = Fvid2_init(&initPrms);
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Fvid2 Init Failed!!!\r\n");
    }
	else
	{
		/* Prints require the fvid2 init to be done.
		 * So app start print is moved here. */
		GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Sample Application - STARTS !!!\r\n");
	}

    /* Cal init */
    retVal = Cal_init();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": CAL Init Failed!!!\r\n");
    }

    retVal = CalUtils_memInit();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Mem Init Failed!!!\r\n");
    }
    CalUtils_memClearOnAlloc(TRUE);

    if (FVID2_SOK == retVal)
    {
        GT_0trace(CalTrace, GT_INFO,
                  APP_NAME ": CAL APP Initialized\r\n");
    }

    return;
}

static void appCaptDeInit(appCaptObj_t *appObj)
{
	int32_t         retVal;
    /* Cal de-init */
	retVal = CalUtils_memDeInit();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Mem De Init Failed!!!\r\n");
    }

	retVal = Cal_deInit();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": CAL De Init Failed!!!\r\n");
    }

    retVal = Fvid2_deInit(NULL);
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Fvid2 De Init Failed!!!\r\n");
    }

    return;
}

/**
 *  appCapture4GivenConfig
 *  Capture test function.
 */
static void appCapture4GivenConfig(appCaptObj_t *appObj, appCaptCfg_t *pCfg)
{
    int32_t           retVal = FVID2_SOK;
    volatile uint32_t lastFrameNo = 0;

    /* Print test case information */
    GT_1trace(CalTrace, GT_INFO,
              APP_NAME ": Configured to capture %d frames \r\n",
              pCfg->numFrames);
    memcpy(&appObj->testPrms, pCfg, sizeof (appCaptCfg_t));

    appObj->instId = CAL_CAPT_INST_ID_A_ID;

    /* Make config required for open */
    appCaptDeriveCfg(appObj, pCfg);

    retVal = appCaptCreateDrv(appObj);
    if (FVID2_SOK != retVal)
    {
        return;
    }

    retVal = appCaptSetDrvCfg(appObj, pCfg);
    if (FVID2_SOK != retVal)
    {
        return;
    }

    retVal = appCaptEnableErrorReporting(appObj);
    if (FVID2_SOK != retVal)
    {
        return;
    }

    retVal = appCaptEnableFrameEventNotification(appObj, pCfg);
    if (FVID2_SOK != retVal)
    {
        return;
    }

    retVal = appCaptAllocAndQFrames(appObj);
    if (FVID2_SOK != retVal)
    {
        return;
    }

    CalUtils_sensorConfigInit(pCfg->sensorDriverId);

    appObj->rcvedFramesCount = 0x0;
    appObj->frameWithCrcErrorCnt = 0x0;
    appObj->frameWithWarning = 0x0;
    appObj->frameErrorCnt = 0x0;
    appObj->sofIntCount = 0x0;
    appObj->unExpectedIntCnt = 0x0;
    appObj->crcErrIntCnt = 0x0;
    appObj->eccErrIntCnt = 0x0;
    appObj->nLineEventIdx = 0x0;
    appObj->eofEventIdx[0U] = 0x0;
    appObj->eofEventIdx[1U] = 0x0;
    appObj->eofEventIdx[2U] = 0x0;
    appObj->eofEventIdx[3U] = 0x0;
    appObj->frameEventTrack = 1U;
    lastFrameNo = 0x0U;

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": Starting Capture now...\r\n");

    retVal    = Fvid2_start(appObj->drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Start Failed!!!\r\n");
        return;
    }

    /* Wait for reception completion */
    while (appObj->rcvedFramesCount <
           (appObj->numFramesToCapture * pCfg->numStreams))
    {
        if ((0U == lastFrameNo) && (appObj->rcvedFramesCount))
        {
            lastFrameNo = appObj->rcvedFramesCount;
            GT_0trace(CalTrace, GT_INFO, APP_NAME ": Stream Detected!!!\r\n");
        }
    }

    /* Disable time stamping of frame events */
    appObj->frameEventTrack = 0U;

    retVal = Fvid2_stop(appObj->drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed!!!\r\n");
        return;
    }

    retVal = appCaptFreeFrames(appObj);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed!!!\r\n");
        return;
    }

    appCaptChkDQCounts(appObj);

    appCaptChkFrameEvents(appObj);

    retVal = appCaptDeleteDrv(appObj);
    if (FVID2_SOK != retVal)
    {
        return;
    }
    CalUtils_sensorConfigDeInit(pCfg->sensorDriverId);

	GT_0trace(CalTrace, GT_INFO, APP_NAME "All Tests Have Passed\r\n");
    return;
}

/**
 *  appCaptCb
 *  \brief Driver callback function.
 */
static int32_t appCaptCb(Fvid2_Handle handle, Ptr appData, Ptr reserved)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t streamId, frmIdx;
    static Fvid2_FrameList frmList;
    Fvid2_Frame *pFrm;
    appCaptObj_t *appObj = (appCaptObj_t *) appData;
    GT_assert(CalTrace, appData != NULL);

    for (streamId = 0U; streamId < appObj->numStream; streamId++)
    {
        retVal = Fvid2_dequeue(
            appObj->drvHandle,
            &frmList,
            streamId,
            FVID2_TIMEOUT_NONE);
        if (FVID2_SOK == retVal)
        {
            for (frmIdx = 0; frmIdx < frmList.numFrames; frmIdx++)
            {
                pFrm = frmList.frames[frmIdx];
                if (FVID2_FRAME_STATUS_COMPLETED != pFrm->status)
                {
                    switch (pFrm->status)
                    {
                        case FVID2_FRAME_STATUS_CRC_ERROR:
                            appObj->frameWithCrcErrorCnt++;
                        break;

                        case FVID2_FRAME_STATUS_ECC_CORRECTED:
                            appObj->frameWithWarning++;
                        break;

                        case FVID2_FRAME_STATUS_OVERFLOW:
                        case FVID2_FRAME_STATUS_ECC_ERROR:
                            appObj->frameErrorCnt++;
                        break;
                        default:
                            appObj->frameErrorCnt++;
                        break;
                    }
                }
            }

            appCaptCountDQ(appObj, &frmList, streamId);
            appCaptCountQ(appObj, &frmList, streamId);

            retVal = Fvid2_queue(appObj->drvHandle, &frmList, streamId);
            if (FVID2_SOK != retVal)
            {
                GT_0trace(CalTrace, GT_ERR,
                          APP_NAME ": Capture Queue Failed!!!\r\n");
            }
        }
        appObj->rcvedFramesCount += frmList.numFrames;
        appObj->totalFrmCount    += frmList.numFrames;
    }
    return FVID2_SOK;
}

/**
 *  appCaptDeriveCfg
 *  Initialize the global variables and frame pointers.
 */
static void appCaptDeriveCfg(appCaptObj_t *appObj, appCaptCfg_t *pCfg)
{
    uint32_t streamId;
    Cal_CaptCreateParams  *createPrms;
    Cal_CmplxIoCfg_t *pCmplxIoCfg;

    appObj->maxWidth      = APP_CAPT_MAX_WIDTH;
    appObj->maxHeight     = APP_CAPT_MAX_HEIGHT;
    appObj->totalFrmCount = 0U;
    appObj->totalCpuLoad  = 0U;
    appObj->cpuLoadCount  = 0U;

    appObj->instId    = CAL_CAPT_INST_ID_A_ID;
    appObj->drvHandle = NULL;
    Fvid2CbParams_init(&appObj->cbPrms);
    appObj->cbPrms.cbFxn   = (Fvid2_CbFxn) &appCaptCb;
    appObj->cbPrms.appData = appObj;

    appObj->numStream = pCfg->numStreams;
    GT_assert(CalTrace, ((0U < pCfg->numStreams) &&
                            (pCfg->numStreams < CAL_CAPT_MAX_STREAMS)));

    createPrms = &appObj->createPrms;
    CalCaptCreateParams_init(&appObj->createPrms);
    createPrms->videoIfMode     = pCfg->interfaceType;
    createPrms->videoIfWidth    = pCfg->interfacewidth;
    createPrms->bufCaptMode     = CAL_CAPT_BCM_LAST_FRM_REPEAT;
    createPrms->numCh           = 1U;
    createPrms->numStream       = appObj->numStream;
    createPrms->pAdditionalArgs = &appObj->calOpenPrms;

    appObj->calOpenPrms.csi2PhyClock[0] =
                                APP_CAPT_CAL_SENSOR_BYPASS_PHY_CLK_MHz;

    if (FVID2_VIFM_SCH_CPI == pCfg->interfaceType)
    {
        appObj->calOpenPrms.subModules[0U] =
            (CAL_CAPT_INST_ID_SUB_CPORT_ID |
             CAL_CAPT_INST_ID_SUB_DMA_WR_ID |
             CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID |
             CAL_CAPT_INST_ID_SUB_PIX_PACK_ID |
             CAL_CAPT_INST_ID_SUB_BYS_IN_ID);
    }
    else
    {
        appObj->calOpenPrms.subModules[0U] =
            (CAL_CAPT_INST_ID_SUB_PPI_ID_0 |
             CAL_CAPT_INST_ID_SUB_CSI2_ID |
             CAL_CAPT_INST_ID_SUB_CPORT_ID |
             CAL_CAPT_INST_ID_SUB_DMA_WR_ID |
             CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID |
             CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID |
             CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID |
             CAL_CAPT_INST_ID_SUB_PIX_PACK_ID);
    }

    if (FVID2_VID_SENSOR_MULDES_OV1063X_DRV == pCfg->sensorDriverId)
    {
        appObj->calOpenPrms.subModules[1U] =
            (
                CAL_CAPT_INST_ID_SUB_PPI_ID_0 |
                CAL_CAPT_INST_ID_SUB_CSI2_ID |
                CAL_CAPT_INST_ID_SUB_CPORT_ID |
                CAL_CAPT_INST_ID_SUB_DMA_WR_ID |
                CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID |
                CAL_CAPT_INST_ID_SUB_PIX_PACK_ID);
        appObj->calOpenPrms.subModules[2U] =
            (
                CAL_CAPT_INST_ID_SUB_PPI_ID_0 |
                CAL_CAPT_INST_ID_SUB_CSI2_ID |
                CAL_CAPT_INST_ID_SUB_CPORT_ID |
                CAL_CAPT_INST_ID_SUB_DMA_WR_ID |
                CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID |
                CAL_CAPT_INST_ID_SUB_PIX_PACK_ID);
        appObj->calOpenPrms.subModules[3U] =
            (
                CAL_CAPT_INST_ID_SUB_PPI_ID_0 |
                CAL_CAPT_INST_ID_SUB_CSI2_ID |
                CAL_CAPT_INST_ID_SUB_CPORT_ID |
                CAL_CAPT_INST_ID_SUB_DMA_WR_ID |
                CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID |
                CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID |
                CAL_CAPT_INST_ID_SUB_PIX_PACK_ID);
    }

    for (streamId = 0U; streamId < CAL_CAPT_MAX_CMPLXIO_INST; streamId++)
    {
        appObj->calOpenPrms.isCmplxIoCfgValid[streamId] = FALSE;
    }
    appObj->calOpenPrms.isCmplxIoCfgValid[0] = TRUE;

    pCmplxIoCfg                = &appObj->calOpenPrms.cmplxIoCfg[0U];
    pCmplxIoCfg->enable        = TRUE;

    pCmplxIoCfg->clockLane.pol      = FALSE;
    pCmplxIoCfg->clockLane.position = 1U;
    pCmplxIoCfg->data1Lane.pol      = FALSE;
    pCmplxIoCfg->data1Lane.position = 2U;
    pCmplxIoCfg->data2Lane.pol      = FALSE;
    pCmplxIoCfg->data2Lane.position = 3U;
    pCmplxIoCfg->data3Lane.pol      = FALSE;
    pCmplxIoCfg->data3Lane.position = 4U;
    pCmplxIoCfg->data4Lane.pol      = FALSE;
    pCmplxIoCfg->data4Lane.position = 5U;

    if (FVID2_VIFW_3LANES == pCfg->interfacewidth)
    {
        pCmplxIoCfg->data4Lane.position = 0U;
    }
    if (FVID2_VIFW_2LANES == pCfg->interfacewidth)
    {
        pCmplxIoCfg->data3Lane.position = 0U;
        pCmplxIoCfg->data4Lane.position = 0U;
    }
    if (FVID2_VIFW_1LANES == pCfg->interfacewidth)
    {
        pCmplxIoCfg->data2Lane.position = 0U;
        pCmplxIoCfg->data3Lane.position = 0U;
        pCmplxIoCfg->data4Lane.position = 0U;
    }

    for (streamId = 0U; streamId < appObj->numStream; streamId++)
    {
        createPrms->chNumMap[streamId][0U] =
            Cal_captMakeChNum(appObj->instId, streamId, 0U);
    }

    return;
}

static int32_t appCaptCreateDrv(appCaptObj_t *appObj)
{
    int32_t retVal = FVID2_SOK;

    appObj->drvHandle = Fvid2_create(
        FVID2_CAL_CAPT_DRV_ID,
        appObj->instId,
        &appObj->createPrms,
        &appObj->createStatus,
        &appObj->cbPrms);
    if ((NULL == appObj->drvHandle) ||
        (appObj->createStatus.retVal != FVID2_SOK))
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Create Failed!!!\r\n");
        retVal = appObj->createStatus.retVal;
    }

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": CAL Capture created\r\n");

    return (retVal);
}

static int32_t appCaptDeleteDrv(appCaptObj_t *appObj)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t streamId;
    static Fvid2_FrameList frmList;

    for (streamId = 0U;
         streamId < appObj->createPrms.numStream;
         streamId++)
    {
        /* Dequeue all the request from the driver */
        while (1U)
        {
            retVal = Fvid2_dequeue(
                appObj->drvHandle,
                &frmList,
                streamId,
                FVID2_TIMEOUT_NONE);
            if (FVID2_SOK != retVal)
            {
                break;
            }
            else
            {
                appCaptCountDQ(appObj, &frmList, streamId);
            }
        }
    }
    retVal = Fvid2_delete(appObj->drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Delete Failed!!!\r\n");
    }
    else
    {
        appObj->drvHandle = NULL;
    }

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": Capture Driver deleted\r\n");

    return (retVal);
}

static int32_t appCaptSetDrvCfg(appCaptObj_t *appObj, appCaptCfg_t *pCfg)
{
    uint32_t i;
    int32_t  retVal = FVID2_SOK;
    appObj->numFramesToCapture = pCfg->numFrames;
    appObj->cfg.numStream      = pCfg->numStreams;
    for (i = 0; i < pCfg->numStreams; i++)
    {
        appObj->cfg.streamId[i] = i;

        appObj->cfg.inFmt[i].width  = pCfg->width;
        appObj->cfg.inFmt[i].height = pCfg->height;

        appObj->cfg.inFmt[i].pitch[0]   = pCfg->pitch;
        appObj->cfg.inFmt[i].dataFormat = 0x0U;
        appObj->cfg.inFmt[i].ccsFormat  = pCfg->bpp;
        appObj->cfg.csi2DataFormat[i]   = pCfg->inCsi2DataFormat;
        if (0 == i)
        {
            appObj->cfg.csi2VirtualChanNo[i] = pCfg->virtualChannel;
        }
        else if (1 == i)
        {
            appObj->cfg.csi2VirtualChanNo[i] = 1U;
        }
        else if (2 == i)
        {
            appObj->cfg.csi2VirtualChanNo[i] = 2U;
        }
        else
        {
            appObj->cfg.csi2VirtualChanNo[i] = 3U;
        }
        appObj->cfg.streamType[i] = CAL_TAG_PIX_DATA;

        appObj->cfg.isPixProcCfgValid[i] = FALSE;
        appObj->cfg.isBysOutCfgValid[i]  = FALSE;

        if (FVID2_VIFM_SCH_CPI == pCfg->interfaceType)
        {
            appObj->cfg.bysInEnable[0] = TRUE;
        }
        else
        {
            appObj->cfg.bysInEnable[i] = FALSE;
        }
        appObj->cfg.isVportCfgValid[i] = FALSE;

        appObj->cfg.writeToMem[i] = TRUE;

        if (((CAL_CSI2_RAW12 == pCfg->inCsi2DataFormat) ||
             (CAL_CSI2_RAW14 == pCfg->inCsi2DataFormat)) ||
            (CAL_CSI2_RAW10 == pCfg->inCsi2DataFormat))
        {
            appObj->cfg.pixProcCfg[i].extract = CAL_PIX_EXRCT_B14_MIPI;
            if (CAL_CSI2_RAW12 == pCfg->inCsi2DataFormat)
            {
                appObj->cfg.pixProcCfg[i].extract =
                    CAL_PIX_EXRCT_B12_MIPI;
            }
            if (CAL_CSI2_RAW10 == pCfg->inCsi2DataFormat)
            {
                appObj->cfg.pixProcCfg[i].extract =
                    CAL_PIX_EXRCT_B10_MIPI;
            }
            appObj->cfg.isPixProcCfgValid[i]   = TRUE;
            appObj->cfg.pixProcCfg[i].decCodec = CAL_DPCM_DEC_BYPASS;
            appObj->cfg.pixProcCfg[i].enableDpcmInitContext = FALSE;
            appObj->cfg.pixProcCfg[i].encCodec = CAL_DPCM_ENC_BYPASS;
            appObj->cfg.pixProcCfg[i].pack     = CAL_PIX_PACK_B16;
            /* Write in 16 bit container */
            appObj->cfg.inFmt[i].pitch[0] = appObj->cfg.inFmt[i].width * 2;
            appObj->cfg.inFmt[i].ccsFormat      = FVID2_CCSF_BITS8_PACKED;
            if (FVID2_VID_SENSOR_MULDES_OV1063X_DRV == pCfg->sensorDriverId)
            {
                /* SAT0088 + OV10635 sends 10 bit YUV, the UB96x is configured
                    to forward MS 8bits only, hence extract and process 8 bits
                    only */
                appObj->cfg.pixProcCfg[i].extract = CAL_PIX_EXRCT_B8;
                appObj->cfg.pixProcCfg[i].pack    = CAL_PIX_PACK_B8;
            }
            if (FVID2_VID_SENSOR_BYPASS_CSI2_DRV == pCfg->sensorDriverId)
            {
                appObj->cfg.isPixProcCfgValid[i]   = FALSE;
                appObj->cfg.inFmt[i].pitch[0] = appObj->cfg.inFmt[i].width * 3;
                appObj->cfg.inFmt[i].ccsFormat      = FVID2_CCSF_BITS8_PACKED;
            }
        }
    }

    retVal = Fvid2_control(appObj->drvHandle, IOCTL_CAL_CAPT_SET_PARAMS,
                           &appObj->cfg, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Set CAL Params Failed!!!\r\n");
    }

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": CAL Capture Configured\r\n");

    return (retVal);
}

/**
 *  \brief Allocate and queue frames to driver
 */
static int32_t appCaptAllocAndQFrames(appCaptObj_t *appObj)
{
    int32_t                  retVal = FVID2_SOK;
    uint32_t                 streamId, idx;
    uint32_t                 bufSize;
    Fvid2_Format           fmt;
    Fvid2_Frame           *pFrm;
    static Fvid2_FrameList frmList;
    static Char            fileStr[200U];
    char                  *fmtStr;

    /* for every stream and channel in a capture handle */
    Fvid2FrameList_init(&frmList);
    for (streamId = 0U; streamId < appObj->createPrms.numStream; streamId++)
    {
        Fvid2Format_init(&fmt);
        fmt.width      = appObj->testPrms.width;
        fmt.height     = appObj->testPrms.height;
        fmt.pitch[0]   = appObj->testPrms.pitch;
        fmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
        fmt.dataFormat = FVID2_DF_YUV422I_UYVY;

        idx = streamId * CAPT_APP_FRAMES_PER_STREAM;

        if (idx >= (CAPT_APP_FRAMES_PER_STREAM * CAL_CAPT_MAX_STREAMS))
        {
            GT_assert(CalTrace, FALSE);
        }

        pFrm = (Fvid2_Frame *) &appObj->frames[idx];

        /* fill format with channel specific values  */
        fmt.chNum = Cal_captMakeChNum(appObj->instId, streamId, 0U);

        /*
         * alloc memory based on 'format'
         * Allocated frame info is put in frames[]
         * CAPT_APP_FRAMES_PER_STREAM is the number of buffers per channel to
         * allocate
         */
        retVal = CalUtils_memFrameAlloc(&fmt, pFrm,
                                        CAPT_APP_FRAMES_PER_STREAM);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR,
                      APP_NAME ": Capture Frame Alloc Failed!!!\r\n");
            break;
        }

        fmtStr = "yuyv422";
        if(CAL_CSI2_RGB888 == appObj->testPrms.inCsi2DataFormat)
        {
            fmtStr = "rgb888";
        }
        snprintf(fileStr, sizeof (fileStr),
                 "captureCalOption%uStr%u_%s_prog_packed_%u_%u.tigf",
                 (unsigned int) appObj->testPrms.cfgId,
                 (unsigned int) streamId,
                 fmtStr,
                 (unsigned int) appObj->testPrms.width,
                 (unsigned int) appObj->testPrms.height);
        CalUtils_memFrameGetSize(&fmt, &bufSize, NULL);
        CacheP_wbInv((void *)pFrm[0].addr[0], (CAPT_APP_FRAMES_PER_STREAM * bufSize));
        GT_3trace(
            CalTrace, GT_INFO,
            "saveRaw(0, 0x%.8x, \"D:\\\\%s\", %d, "
            "32, false);\r\n",
            pFrm[0].addr[0], fileStr, (bufSize / 4U));

        /* Set number of frame in frame list */
        for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
        {
            /* Associate instance of sub-frame information structure with the
                frame */
            pFrm[idx].subFrameInfo = &appObj->subFrameInfo[idx];
            pFrm[idx].appData = appObj;

            frmList.frames[idx] = &pFrm[idx];
            GT_2trace(CalTrace, GT_INFO, APP_NAME ": Captured Frames [%d]"
                      " Available at 0x%x\r\n", idx, pFrm[idx].addr[0]);
        }

        frmList.numFrames = CAPT_APP_FRAMES_PER_STREAM;

        if (0x0 == streamId)
        {
            uint32_t idx;
            for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
            {
                appObj->qCount[idx][0]   = (Uint32) frmList.frames[idx];
                appObj->dQCount[idx][0]  = (Uint32) frmList.frames[idx];
                appObj->qCount[idx][1U]  = 0U;
                appObj->dQCount[idx][1U] = 0U;
            }
            appCaptCountQ(appObj, &frmList, 0);
        }
        if (0x1 == streamId)
        {
            uint32_t idx;
            for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
            {
                appObj->qCount[idx][2U]  = (Uint32) frmList.frames[idx];
                appObj->dQCount[idx][2U] = (Uint32) frmList.frames[idx];
                appObj->qCount[idx][3U]  = 0U;
                appObj->dQCount[idx][3U] = 0U;
            }
            appCaptCountQ(appObj, &frmList, 1);
        }
        if (0x2 == streamId)
        {
            uint32_t idx;
            for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
            {
                appObj->qCount[idx][4U]  = (Uint32) frmList.frames[idx];
                appObj->dQCount[idx][4U] = (Uint32) frmList.frames[idx];
                appObj->qCount[idx][5U]  = 0U;
                appObj->dQCount[idx][5U] = 0U;
            }
            appCaptCountQ(appObj, &frmList, 2);
        }
        if (0x3 == streamId)
        {
            uint32_t idx;
            for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
            {
                appObj->qCount[idx][6U]  = (Uint32) frmList.frames[idx];
                appObj->dQCount[idx][6U] = (Uint32) frmList.frames[idx];
                appObj->qCount[idx][7U]  = 0U;
                appObj->dQCount[idx][7U] = 0U;
            }
            appCaptCountQ(appObj, &frmList, 3);
        }
        /*
         * queue the frames in frmList
         * All allocated frames are queued here as an example.
         * In general atleast 2 frames per stream/channel need to queued
         * before capture can be started.
         * Failing which, frame could be dropped.
         */
        retVal = Fvid2_queue(appObj->drvHandle, &frmList, streamId);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR,
                      APP_NAME ": Capture Queue Failed!!!\r\n");
            break;
        }
    }

    return (retVal);
}

static int32_t appCaptFreeFrames(appCaptObj_t *appObj)
{
    int32_t                  retVal = FVID2_SOK;
    uint32_t                 streamId, idx;
    Fvid2_Format           fmt;
    Fvid2_Frame           *pFrm;
    static Fvid2_FrameList frmList;

    /* for every stream and channel in a capture handle */
    Fvid2FrameList_init(&frmList);
    for (streamId = 0U; streamId < appObj->createPrms.numStream; streamId++)
    {
        Fvid2Format_init(&fmt);
        fmt.width      = appObj->testPrms.width;
        fmt.height     = appObj->testPrms.height;
        fmt.pitch[0]   = appObj->testPrms.pitch;
        fmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
        fmt.dataFormat = FVID2_DF_YUV422I_UYVY;

        while (1U)
        {
            retVal = Fvid2_dequeue(
                appObj->drvHandle,
                &frmList,
                streamId,
                FVID2_TIMEOUT_NONE);
            if (FVID2_SOK != retVal)
            {
                break;
            }
            else
            {
                appCaptCountDQ(appObj, &frmList, streamId);
            }
        }

        idx  = streamId * CAPT_APP_FRAMES_PER_STREAM;
        pFrm = (Fvid2_Frame *) &appObj->frames[idx];

        retVal = CalUtils_memFrameFree(&fmt, pFrm, CAPT_APP_FRAMES_PER_STREAM);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR,
                      APP_NAME ": Capture Frame Free Failed!!!\r\n");
        }

        if (FVID2_SOK != retVal)
        {
            break;
        }
    }

    return (retVal);
}

static void appCaptCountQ(appCaptObj_t *appObj, Fvid2_FrameList *frmList,
                          uint32_t streamId)
{
    uint32_t frmIdx, frmAddr, cntIdx;
    for (frmIdx = 0U; frmIdx < frmList->numFrames; frmIdx++)
    {
        frmAddr = (Uint32) frmList->frames[frmIdx];
        for (cntIdx = 0U; cntIdx < CAPT_APP_FRAMES_PER_STREAM; cntIdx++)
        {
            if (0 == streamId)
            {
                if (appObj->qCount[cntIdx][0U] == frmAddr)
                {
                    appObj->qCount[cntIdx][1U]++;
                }
            }
            else if (1 == streamId)
            {
                if (appObj->qCount[cntIdx][2U] == frmAddr)
                {
                    appObj->qCount[cntIdx][3U]++;
                }
            }
            else if (2 == streamId)
            {
                if (appObj->qCount[cntIdx][4U] == frmAddr)
                {
                    appObj->qCount[cntIdx][5U]++;
                }
            }
            else
            {
                if (appObj->qCount[cntIdx][6U] == frmAddr)
                {
                    appObj->qCount[cntIdx][7U]++;
                }
            }
        }
    }
}

static void appCaptCountDQ(appCaptObj_t *appObj, Fvid2_FrameList *frmList,
                           uint32_t streamId)
{
    uint32_t frmIdx, frmAddr, cntIdx;
    for (frmIdx = 0U; frmIdx < frmList->numFrames; frmIdx++)
    {
        frmAddr = (Uint32) frmList->frames[frmIdx];
        for (cntIdx = 0U; cntIdx < CAPT_APP_FRAMES_PER_STREAM; cntIdx++)
        {
            if (0 == streamId)
            {
                if (appObj->dQCount[cntIdx][0U] == frmAddr)
                {
                    appObj->dQCount[cntIdx][1U]++;
                }
            }
            else if (1 == streamId)
            {
                if (appObj->dQCount[cntIdx][2U] == frmAddr)
                {
                    appObj->dQCount[cntIdx][3U]++;
                }
            }
            else if (2 == streamId)
            {
                if (appObj->dQCount[cntIdx][4U] == frmAddr)
                {
                    appObj->dQCount[cntIdx][5U]++;
                }
            }
            else
            {
                if (appObj->dQCount[cntIdx][6U] == frmAddr)
                {
                    appObj->dQCount[cntIdx][7U]++;
                }
            }
        }
    }
}

static void appCaptChkDQCounts(appCaptObj_t *appObj)
{
    uint32_t cntIdx, isError;
    isError = FALSE;
    for (cntIdx = 0U; cntIdx < CAPT_APP_FRAMES_PER_STREAM; cntIdx++)
    {
        if (appObj->dQCount[cntIdx][0U] != appObj->qCount[cntIdx][0U])
        {
            isError = TRUE;
            GT_1trace(
                CalTrace, GT_INFO, APP_NAME
                ": ERROR Q / DQ address are not in order [%d]!!!\r\n",
                cntIdx);
        }
        if (appObj->dQCount[cntIdx][1U] != appObj->qCount[cntIdx][1U])
        {
            isError = TRUE;
            GT_1trace(CalTrace, GT_INFO, APP_NAME
                      ": ERROR Q / DQ counts do not match up [%d]!!!\r\n", cntIdx);
        }
        if (appObj->dQCount[cntIdx][2U] != appObj->qCount[cntIdx][2U])
        {
            isError = TRUE;
            GT_1trace(
                CalTrace, GT_INFO, APP_NAME
                ": ERROR Q / DQ address are not in order [%d] - Stream 1!!!\r\n",
                cntIdx);
        }
        if (appObj->dQCount[cntIdx][3U] != appObj->qCount[cntIdx][3U])
        {
            isError = TRUE;
            GT_1trace(
                CalTrace, GT_INFO, APP_NAME
                ": ERROR Q / DQ counts do not match up [%d] - Stream 1!!!\r\n",
                cntIdx);
        }
    }
    if (FALSE == isError)
    {
        GT_0trace(CalTrace, GT_INFO, APP_NAME ": Q/Dq Counts match up\r\n");
        GT_0trace(CalTrace, GT_INFO, APP_NAME ": Q/Dq Order match\r\n");
    }
}

/**
 *  appCaptGetTestId
 *  Return the test ID to run.
 */
static int32_t appCaptGetTestId(appCaptObj_t *appObj)
{
    uint32_t       testCnt;
    static int32_t testId = 0;
    uint32_t       retVal;

    GT_0trace(CalTrace, GT_INFO, " \r\n");
    GT_0trace(CalTrace, GT_INFO,
              "--------------------------------------\r\n");
    GT_0trace(CalTrace, GT_INFO,
              "Select test to run as per below table:\r\n");
    GT_0trace(CalTrace, GT_INFO,
              "--------------------------------------\r\n");
    GT_0trace(CalTrace, GT_INFO, " \r\n");
    for (testCnt = 0; testCnt < APP_CAPT_NUM_OPTS; testCnt++)
    {
        GT_2trace(CalTrace, GT_INFO,
                  "%3d: %s\r\n", testCnt, gTestCfg[testCnt].testDescStr);
    }
    GT_2trace(CalTrace, GT_INFO,
              "%3d: %s\r\n", APP_CAPT_NUM_OPTS, "EXIT\r\n");
    GT_0trace(CalTrace, GT_INFO, " \r\n");
    GT_0trace(CalTrace, GT_INFO, "Enter Test to Run (in UART console): \r\n");

    while (1U)
    {
        UART_scanFmt("%d", &testId);
        GT_1trace(CalTrace, GT_INFO, "%d\r\n", testId);
        if ((testId >= 0) && (testId < APP_CAPT_NUM_OPTS))
        {
            retVal = testId;
            testId++;
            break;
        }
        else if (APP_CAPT_NUM_OPTS == testId)
        {
            retVal = testId;
            break;
        }
        GT_0trace(CalTrace, GT_INFO, "Invalid Test ID. Enter Agian!!\r\n");
    }

    return (retVal);
}

static void appCaptErrCb(const uint32_t *event, uint32_t numEvents, Ptr arg)
{
    uint32_t idx;
    appCaptObj_t *pAppObj = (appCaptObj_t *)arg;
    for (idx = 0U; idx < numEvents; idx++)
    {
        if (CAL_CSI2_PPI_VC_SOF1 == event[idx])
        {
            pAppObj->sofIntCount++;
        }
        else if ((CAL_CSI2_PPI_VC_CRC_MISMATCH_VC1 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_CRC_MISMATCH_VC2 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_CRC_MISMATCH_VC3 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_CRC_MISMATCH_VC4 == event[idx]))
        {
            pAppObj->crcErrIntCnt++;
        }
        else if ((CAL_CSI2_PPI_VC_ECC_CORRECTION_VC1 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_ECC_CORRECTION_VC2 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_ECC_CORRECTION_VC3 == event[idx]) ||
                 (CAL_CSI2_PPI_VC_ECC_CORRECTION_VC4 == event[idx]))
        {
            pAppObj->eccErrIntCnt++;
        }
        else
        {
            pAppObj->unExpectedIntCnt++;
        }
    }
}

static int32_t appCaptEnableErrorReporting(appCaptObj_t *appObj)
{
    int32_t retVal;
    Cal_ErrorCfg_t errCfg;

    errCfg.numErrorsToMonitor = 8U;
    errCfg.cmplxIoId = 0U;
    errCfg.errSrc[0U] = CAL_CSI2_PPI_VC_CRC_MISMATCH_VC1;
    errCfg.errSrc[1U] = CAL_CSI2_PPI_VC_CRC_MISMATCH_VC2;
    errCfg.errSrc[2U] = CAL_CSI2_PPI_VC_CRC_MISMATCH_VC3;
    errCfg.errSrc[3U] = CAL_CSI2_PPI_VC_CRC_MISMATCH_VC4;
    errCfg.errSrc[4U] = CAL_CSI2_PPI_VC_ECC_CORRECTION_VC1;
    errCfg.errSrc[5U] = CAL_CSI2_PPI_VC_ECC_CORRECTION_VC2;
    errCfg.errSrc[6U] = CAL_CSI2_PPI_VC_ECC_CORRECTION_VC3;
    errCfg.errSrc[7U] = CAL_CSI2_PPI_VC_ECC_CORRECTION_VC4;

    errCfg.appCb = &appCaptErrCb;
    errCfg.pAppCbArgs = (Ptr)appObj;

    retVal = Fvid2_control(appObj->drvHandle, IOCTL_CAL_CAPT_SET_ERR_PRMS,
                           &errCfg, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Set CAL Params Failed!!!\r\n");
    }
    return retVal;
}

static int32_t appCaptFrameEventNotifyCb(Fvid2_Handle handle, Fvid2_Frame *pFrame)
{
    appCaptObj_t *appObj;
    uint32_t streamId, idx;
    uint32_t static tsFrameEvents = 1U;

    GT_assert(CalTrace, (handle != NULL));
    GT_assert(CalTrace, (pFrame != NULL));
    GT_assert(CalTrace, (pFrame->subFrameInfo != NULL));
    GT_assert(CalTrace, (pFrame->appData != NULL));

    streamId = Cal_captGetStreamId(pFrame->chNum);
    appObj = (appCaptObj_t *)pFrame->appData;

    if ((CAPT_APP_NTH_LINE_FRAME_EVENT == pFrame->subFrameInfo->subFrameNum) &&
        (1U == tsFrameEvents))
    {
        idx = appObj->nLineEventIdx;
        appObj->nLineInfo[idx].subFrameNum  =
                                        pFrame->subFrameInfo->subFrameNum;
        appObj->nLineInfo[idx].numInLines   =
                                        pFrame->subFrameInfo->numInLines;
        appObj->nLineInfo[idx].numOutLines  =
                                        pFrame->subFrameInfo->numOutLines;

        idx++;
        if (idx >= CAPT_APP_FRAMES_PER_STREAM)
        {
            idx = 0U;
        }
        appObj->nLineEventIdx = idx;
    }
    if ((CAPT_APP_END_OF_FRAME_EVENT == pFrame->subFrameInfo->subFrameNum) &&
        (1U == tsFrameEvents))
    {
        if (4U > streamId)
        {
            idx = appObj->eofEventIdx[streamId];
            appObj->eofEventFrameAddr[idx][streamId] = (uint32_t)pFrame;
            idx++;
            if (idx >= CAPT_APP_FRAMES_PER_STREAM)
            {
                idx = 0U;
            }
            appObj->eofEventIdx[streamId] = idx;
        }
        if (1U != appObj->frameEventTrack)
        {
            tsFrameEvents = 0U;
        }
    }

    return (FVID2_SOK);
}

static int32_t appCaptEnableFrameEventNotification(appCaptObj_t *appObj,
                                                    appCaptCfg_t *pCfg)
{
    uint32_t streamId;
    Cal_FrameEventNotifyCfg_t ntyCfg;
    int32_t retVal = FVID2_SOK;

        ntyCfg.numStream = 0U;
        for (streamId = 0U; streamId < pCfg->numStreams; streamId++)
        {
            ntyCfg.streamId[streamId] = streamId;
            if (CAPT_APP_NTH_LINE_STREAM_ID == streamId)
            {
                ntyCfg.notifyAfterFirstXLines[streamId] = 180U;
            }
            else
            {
                ntyCfg.notifyAfterFirstXLines[streamId] = 0U;
            }
            ntyCfg.notifyAfterEndOfFrame[streamId] = TRUE;
            ntyCfg.numStream++;
        }

        ntyCfg.appCb = &appCaptFrameEventNotifyCb;
        ntyCfg.pAdditionalArgs = NULL;

        retVal = Fvid2_control(appObj->drvHandle,
                            IOCTL_CAL_CAPT_SET_FRAME_EVENT_NOTIFY_PRMS, &ntyCfg, NULL);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR,
            APP_NAME ": Capture Set CAL Frame Event Notification "
                     "Failed!!!\r\n");
        }
    return retVal;
}

static void appCaptChkFrameEvents(appCaptObj_t *appObj)
{
    uint32_t idx;
    /* Check the order in which events were reported */
    for (idx = 0U; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
    {
        if (appObj->dQCount[idx][0U] != appObj->eofEventFrameAddr[idx][0])
        {
            GT_1trace(
                CalTrace, GT_INFO, APP_NAME
                ": ERROR DQ/EOF Frame address are not in order [%d]!!!\r\n",
                idx);
            break;
        }
    }

}

void App_print(const char *format, ...)
{
    va_list     vaArgPtr;

    va_start(vaArgPtr, format);

    UART_printf(format, vaArgPtr);
    printf(format, vaArgPtr);

    va_end(vaArgPtr);

    return;
}
