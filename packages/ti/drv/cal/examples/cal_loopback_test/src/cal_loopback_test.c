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
 *  \file cal_loopback_test.c
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

/* This is needed for vsnprintf */
#include <stdio.h>
#include <stdarg.h>
#include <cal_loopback_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**< CSI2 PHY Clock Custom sensor. */
#define APP_CAPT_CAL_SENSOR_BYPASS_PHY_CLK_MHz (400U)

#define APP_CAPT_MAX_WIDTH (1280U)
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

static void appCaptErrCb(const uint32_t *event, uint32_t numEvents, void * arg);
static int32_t appCaptEnableErrorReporting(appCaptObj_t *appObj);

static void DispApp_create(appCaptObj_t *appObj);
static void DispApp_delete(appCaptObj_t *appObj);
static int32_t DispApp_configDctrl(appCaptObj_t *appObj);
static void DispApp_initParams(appCaptObj_t *appObj);
static int32_t DispApp_allocAndQueueFrames(const appCaptObj_t *appObj,
                                           DispApp_InstObj *instObj);
static int32_t DispApp_pipeCbFxn(Fvid2_Handle handle, void *appData);

void App_print(const char *format, ...);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**< demo application object */
appCaptObj_t gCaptAppObj;

/**< Different capture options supported by this demo */
appCaptCfg_t gTestCfg =
{
    "Sensor Config Bypassed CSI2 4Lanes capture color bars from UB954 to DSS",   0U,
    TRUE,
    FVID2_VIFM_SCH_CSI2,
    FVID2_VIFW_4LANES, 1U, 0U, CAL_CSI2_RGB888, FVID2_CCSF_BITS8_PACKED,
    CAPT_APP_RUN_COUNT, 1280U, 720U, (1280U * 3U),
    FVID2_VID_SENSOR_BYPASS_CSI2_DRV,
    FVID2_STD_1080P_30, FVID2_DF_BAYER_BGGR, FVID2_CCSF_BITS8_PACKED,
    /* Instance Id */
    DSS_DISP_INST_VID1,
    /* Pipe Id */
    CSL_DSS_VID_PIPE_ID_VID1,
    /* Pipe Node Id */
    DSS_DCTRL_NODE_VID1,
    /* Pipe Type */
    CSL_DSS_VID_PIPE_TYPE_VID,
    /* Data format */
    FVID2_DF_BGR24_888,
    /* Input buffer width */
    1280U,
    /* Input buffer height */
    720U,
    /* Scan format */
    FVID2_SF_PROGRESSIVE,
    /* Output buffer width */
    1280U,
    /* Output buffer height */
    720U,
    /* Scaler enable */
    FALSE,
    /* Global Alpha */
    0xFFU,
    /* Pre-multiply alpha */
    FALSE,
    /* X Position */
    0U,
    /* Y position */
    0U,
    /* Invalid Pipe Id */
    CSL_DSS_VID_PIPE_ID_VID1
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Test task main
 */
void Cal_loopbackTest(void)
{
    CalUtils_MemHeapStatus startHeapStat1;
    appCaptObj_t          *appObj = &gCaptAppObj;

    memset(&gCaptAppObj, 0x0, sizeof (appCaptObj_t));
    appCaptInit(appObj);

    CalUtils_memGetHeapStat(&startHeapStat1);

    appCapture4GivenConfig(appObj, &gTestCfg);

    CalUtils_memCheckHeapStat(&startHeapStat1);

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
                  APP_NAME ": System Init Failed!!!\r\n");
    }

    retVal = CalUtils_memInit();
    if (retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Mem Init Failed!!!\r\n");
    }
    CalUtils_memClearOnAlloc(TRUE);

    Dss_initParamsInit(&appObj->initParams);
    Dss_init(&appObj->initParams);

    if(FVID2_SOK == retVal)
    {
        /* Create DCTRL handle, used for common driver configuration */
        appObj->dctrlHandle = Fvid2_create(
            DSS_DCTRL_DRV_ID,
            DSS_DCTRL_INST_0,
            NULL,
            NULL,
            NULL);
        if(NULL == appObj->dctrlHandle)
        {
            GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DCTRL Create Failed!!!\r\n");
        }
    }

    if(FVID2_SOK == retVal)
    {
         GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": DispApp_init() - DONE !!!\r\n");
    }

    if (FVID2_SOK == retVal)
    {
        GT_0trace(CalTrace, GT_INFO,
                  APP_NAME ": CAL APP Initialized\r\n");
    }

    return;
}

static void appCaptDeInit(appCaptObj_t *appObj)
{
    int32_t  retVal = FVID2_SOK;
    /* Cal de-init */
	retVal = CalUtils_memDeInit();
	retVal += Cal_deInit();

    /* Delete DCTRL handle */
    retVal = Fvid2_delete(appObj->dctrlHandle, NULL);
    retVal += Dss_deInit();
    retVal += Fvid2_deInit(NULL);

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
    DispApp_InstObj instObj;

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
    lastFrameNo = 0x0U;

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": Starting Capture now...\r\n");

    GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Starting display ... !!!\r\n");
    GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Display in progress ... DO NOT HALT !!!\r\n");

    retVal    = Fvid2_start(appObj->drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Start Failed!!!\r\n");
        return;
    }

    /* Start driver */
    instObj = appObj->instObj;
    retVal = Fvid2_start(instObj.drvHandle, NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Display Start Failed!!!\r\n");
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

    retVal = Fvid2_stop(appObj->drvHandle, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed!!!\r\n");
        return;
    }

    instObj = appObj->instObj;
    retVal  = Fvid2_stop(instObj.drvHandle, NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Display Stop Failed!!!\r\n");
    }

    retVal = appCaptFreeFrames(appObj);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Stop Failed!!!\r\n");
        return;
    }

    retVal = appCaptDeleteDrv(appObj);
    if (FVID2_SOK != retVal)
    {
        return;
    }
    CalUtils_sensorConfigDeInit(pCfg->sensorDriverId);

	GT_0trace(CalTrace, GT_INFO, APP_NAME ": All Tests Have Passed\r\n");
    return;
}

/**
 *  appCaptCb
 *  \brief Driver callback function.
 */
static int32_t appCaptCb(Fvid2_Handle handle, void * appData, void * reserved)
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

            retVal = Fvid2_queue(
                        appObj->instObj.drvHandle,
                        &frmList,
                        streamId);
            if (FVID2_SOK != retVal)
            {
                while(1);
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

    DispApp_create(appObj);

    GT_0trace(CalTrace, GT_INFO, APP_NAME ": CAL Capture created\r\n");

    return (retVal);
}

static int32_t appCaptDeleteDrv(appCaptObj_t *appObj)
{
    int32_t  retVal = FVID2_SOK;

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

    DispApp_delete(appObj);

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
        appObj->cfg.inFmt[i].dataFormat = gTestCfg.inDataFmt;
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
    int32_t                 retVal = FVID2_SOK;
    uint32_t                streamId, idx;
    uint32_t                bufSize;
    Fvid2_Format            fmt;
    Fvid2_Frame            *pFrm;
    static Fvid2_FrameList  frmList;
    static Char             fileStr[200U];

    /* for every stream and channel in a capture handle */
    Fvid2FrameList_init(&frmList);
    for (streamId = 0U; streamId < appObj->createPrms.numStream; streamId++)
    {
        Fvid2Format_init(&fmt);
        fmt.width      = appObj->testPrms.width;
        fmt.height     = appObj->testPrms.height;
        fmt.pitch[0]   = appObj->testPrms.pitch;
        fmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
        fmt.dataFormat = gTestCfg.inDataFmt;

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

        CalUtils_memFrameGetSize(&fmt, &bufSize, NULL);
        CacheP_wbInv((void *)pFrm[0].addr[0], (CAPT_APP_FRAMES_PER_STREAM * bufSize));

        snprintf(fileStr, sizeof (fileStr),
                 "captureCalOption%uStr%u_prog_packed_%u_%u.tigf",
                 (unsigned int) appObj->testPrms.cfgId,
                 (unsigned int) streamId,
                 (unsigned int) appObj->testPrms.width,
                 (unsigned int) appObj->testPrms.height);
        GT_3trace(
            CalTrace, GT_INFO,
            "saveRaw(0, 0x%.8x, \"D:\\\\%s\", %d, 32, false);\r\n",
            pFrm[0].addr[0], fileStr, (bufSize / 4U));

        /* Set number of frame in frame list */
        for (idx = 0; idx < CAPT_APP_FRAMES_PER_STREAM; idx++)
        {
            /* Associate instance of sub-frame information structure with the
                frame */
            pFrm[idx].subFrameInfo = NULL;
            pFrm[idx].appData = appObj;

            frmList.frames[idx] = &pFrm[idx];
            GT_2trace(CalTrace, GT_INFO, APP_NAME ": Captured Frames [%d]"
                      " Available at 0x%x\r\n", idx, pFrm[idx].addr[0]);
        }
        frmList.numFrames = CAPT_APP_FRAMES_PER_STREAM;

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
        fmt.dataFormat = gTestCfg.inDataFmt;

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

static void appCaptErrCb(const uint32_t *event, uint32_t numEvents, void * arg)
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
    errCfg.pAppCbArgs = (void *)appObj;

    retVal = Fvid2_control(appObj->drvHandle, IOCTL_CAL_CAPT_SET_ERR_PRMS,
                           &errCfg, NULL);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Capture Set CAL Params Failed!!!\r\n");
    }
    return retVal;
}

static void DispApp_create(appCaptObj_t *appObj)
{
    int32_t retVal = FVID2_SOK;
    SemaphoreP_Params semParams;
    Dss_DctrlVpParams *vpParams;
    DispApp_InstObj *instObj;

    DispApp_initParams(appObj);
    vpParams = &appObj->vpParams;
    Dss_dctrlVpParamsInit(vpParams);

    vpParams->vpId = TEST_VP_ID;
    vpParams->lcdOpTimingCfg.mInfo.standard = FVID2_STD_CUSTOM;
    vpParams->lcdOpTimingCfg.mInfo.width = DISP_APP_LCD_WIDTH;
    vpParams->lcdOpTimingCfg.mInfo.height = DISP_APP_LCD_HEIGHT;
    vpParams->lcdOpTimingCfg.mInfo.hFrontPorch = 48U;
    vpParams->lcdOpTimingCfg.mInfo.hBackPorch = 80U;
    vpParams->lcdOpTimingCfg.mInfo.hSyncLen = 32U;
    vpParams->lcdOpTimingCfg.mInfo.vFrontPorch = 3U;
    vpParams->lcdOpTimingCfg.mInfo.vBackPorch = 14U;
    vpParams->lcdOpTimingCfg.mInfo.vSyncLen = 6U;
    vpParams->lcdOpTimingCfg.dvoFormat = FVID2_DV_GENERIC_DISCSYNC;
    vpParams->lcdOpTimingCfg.videoIfWidth = FVID2_VIFW_24BIT;

    vpParams->lcdPolarityCfg.actVidPolarity = FVID2_POL_HIGH;
    vpParams->lcdPolarityCfg.pixelClkPolarity = FVID2_EDGE_POL_FALLING;
    vpParams->lcdPolarityCfg.hsPolarity = FVID2_POL_HIGH;
    vpParams->lcdPolarityCfg.vsPolarity = FVID2_POL_HIGH;

    DispApp_configDctrl(appObj);

    instObj = &appObj->instObj;
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    instObj->drvHandle = Fvid2_create(
        DSS_DISP_DRV_ID,
        instObj->instId,
        &instObj->createParams,
        &instObj->createStatus,
        &instObj->cbParams);
    if((NULL == instObj->drvHandle) ||
       (instObj->createStatus.retVal != FVID2_SOK))
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Display Create Failed!!!\r\n");
        retVal = instObj->createStatus.retVal;
    }

    if(FVID2_SOK == retVal)
    {
        retVal = Fvid2_control(
            instObj->drvHandle,
            IOCTL_DSS_DISP_SET_DSS_PARAMS,
            &instObj->dispParams,
            NULL);
        if(retVal != FVID2_SOK)
        {
            GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DSS Set Params IOCTL Failed!!!\r\n");
        }
    }

    if(FVID2_SOK == retVal)
    {
        retVal = DispApp_allocAndQueueFrames(appObj, instObj);
        if(retVal != FVID2_SOK)
        {
            GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Display Alloc and Queue Failed!!!\r\n");
        }
    }

    if(FVID2_SOK == retVal)
    {
        GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Display create complete!!\r\n");
    }

    return;
}

static void DispApp_delete(appCaptObj_t *appObj)
{
    int32_t             retVal;
    Dss_DctrlVpParams  *vpParams;
    Dss_DctrlPathInfo  *pathInfo;
    DispApp_InstObj    *instObj;
    Fvid2_FrameList     frmList;
    Fvid2_Format        fmt;
    Fvid2_Frame        *pFrm;

    vpParams = &appObj->vpParams;
    pathInfo = &appObj->dctrlPathInfo;

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_CLEAR_PATH,
        pathInfo,
        NULL);
    if(FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DCTRL Clear Path Failed!!!\r\n");
    }

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_STOP_VP,
        vpParams,
        NULL);
    if(FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": VP Stop Failed!!!\r\n");
    }

    /* Dequeue all the request from the driver */
    instObj = &appObj->instObj;
    while (1U)
    {
        retVal = Fvid2_dequeue(
            instObj->drvHandle,
            &frmList,
            0U,
            FVID2_TIMEOUT_NONE);
        if(FVID2_SOK != retVal)
        {
            break;
        }
    }

    /* Free-up display buffers */
    Fvid2Format_init(&fmt);
    fmt.chNum      = 0U;
    fmt.width      = appObj->testPrms.width;
    fmt.height     = appObj->testPrms.height;
    fmt.pitch[0U]  = appObj->testPrms.pitch;
    fmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
    fmt.dataFormat = gTestCfg.inDataFmt;

    pFrm = (Fvid2_Frame *) &instObj->dispFrames[0U];
    retVal = CalUtils_memFrameFree(&fmt, pFrm, LPBK_APP_DISP_MAX_FRM);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Display Frame Free Failed!!!\r\n");
    }

    retVal = Fvid2_delete(instObj->drvHandle, NULL);
    if(FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Display Delete Failed!!!\r\n");
    }

    if(FVID2_SOK == retVal)
    {
         GT_0trace(CalTrace, GT_INFO,
              APP_NAME ": Display delete complete!!\r\n");
    }

    return;
}

static void DispApp_initParams(appCaptObj_t *appObj)
{
    uint32_t i;
    Dss_DispParams *dispParams;
    DispApp_InstObj *instObj;

    /* Initialize video pipes */
    instObj = &appObj->instObj;
    instObj->instId = gTestCfg.instId;
    Dss_dispCreateParamsInit(&instObj->createParams);
    Fvid2CbParams_init(&instObj->cbParams);
    instObj->cbParams.cbFxn = &DispApp_pipeCbFxn;
    instObj->cbParams.appData = appObj;

    dispParams = &instObj->dispParams;
    Dss_dispParamsInit(dispParams);
    dispParams->pipeCfg.pipeType = gTestCfg.pipeType;
    dispParams->pipeCfg.inFmt.width = gTestCfg.inWidth;
    dispParams->pipeCfg.inFmt.height = gTestCfg.inHeight;
    for(i=0U; i<FVID2_MAX_PLANES; i++)
    {
        dispParams->pipeCfg.inFmt.pitch[i] = gTestCfg.pitch;
    }
    dispParams->pipeCfg.inFmt.dataFormat = gTestCfg.inDataFmt;
    dispParams->pipeCfg.inFmt.scanFormat = gTestCfg.inScanFmt;
    dispParams->pipeCfg.inFmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
    dispParams->pipeCfg.outWidth = gTestCfg.outWidth;
    dispParams->pipeCfg.outHeight = gTestCfg.outHeight;
    dispParams->pipeCfg.scEnable = gTestCfg.scEnable;
    dispParams->alphaCfg.globalAlpha = gTestCfg.globalAlpha;
    dispParams->alphaCfg.preMultiplyAlpha = gTestCfg.preMultiplyAlpha;
    dispParams->layerPos.startX = gTestCfg.posx;
    dispParams->layerPos.startY = gTestCfg.posy;
}

/**
 *  \brief Allocate and queue frames to driver
 */
static Int32 DispApp_allocAndQueueFrames(const appCaptObj_t *appObj,
                                           DispApp_InstObj *instObj)
{
    Int32           retVal = FVID2_SOK;
    UInt32          frmId;
    UInt32          bufSize;
    Fvid2_Format    fmt;
    Fvid2_Frame    *frm;
    Fvid2_FrameList frmList;

    Fvid2FrameList_init(&frmList);

    Fvid2Format_init(&fmt);
    frm = &instObj->dispFrames[0U];

    /* fill format with channel specific values  */
    /* Set channel number as invalid so that the blank frame is used only
     * at the start */
    fmt.chNum      = 0U;
    fmt.width      = appObj->testPrms.width;
    fmt.height     = appObj->testPrms.height;
    fmt.pitch[0U]  = appObj->testPrms.pitch;
    fmt.ccsFormat  = FVID2_CCSF_BITS8_PACKED;
    fmt.dataFormat = gTestCfg.inDataFmt;

    /*
     * alloc memory based on 'format'
     * Allocated frame info is put in frames[]
     * LPBK_APP_DISP_MAX_FRM is the number of buffers per channel to
     * allocate
     */
    retVal = CalUtils_memFrameAlloc(&fmt, frm, LPBK_APP_DISP_MAX_FRM);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR,
                  APP_NAME ": Display Frame Alloc Failed!!!\r\n");
    }

    for (frmId = 0U; frmId < LPBK_APP_DISP_MAX_FRM; frmId++)
    {
        frm[frmId].fid     = FVID2_FID_FRAME;
        frm[frmId].appData = instObj;

        /* Fill with background color */
        CalUtils_memFrameGetSize(&fmt, &bufSize, NULL);
        /* Flush and invalidate the CPU write */
        CacheP_wbInv((void *)frm[frmId].addr[0U], (LPBK_APP_DISP_MAX_FRM * bufSize));

        /* Set number of frame in frame list - one at a time */
        frmList.numFrames  = 1U;
        frmList.frames[0U] = &frm[frmId];

        /*
         * queue the frames in frmList
         * All allocate frames are queued here as an example.
         * In general atleast 2 frames per channel need to queued
         * before starting display,
         * else frame will get dropped until frames are queued
         */
        retVal = Fvid2_queue(instObj->drvHandle, &frmList, 0U);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR,
                      APP_NAME ": Display Queue Failed!!!\r\n");
            break;
        }
    }

    return (retVal);
}

static int32_t DispApp_configDctrl(appCaptObj_t *appObj)
{
    int32_t retVal = FVID2_SOK;
    Dss_DctrlVpParams *vpParams;
    Dss_DctrlOverlayParams *overlayParams;
    Dss_DctrlOverlayLayerParams *layerParams;
    Dss_DctrlPathInfo *pathInfo;
#if defined (SOC_AM65XX)
    Dss_DctrlOldiParams *oldiParams;
    oldiParams = &appObj->oldiParams;
    Dss_dctrlOldiParamsInit(oldiParams);
#endif
    vpParams = &appObj->vpParams;
    overlayParams = &appObj->overlayParams;
    layerParams = &appObj->layerParams;
    pathInfo = &appObj->dctrlPathInfo;
    Dss_dctrlOverlayParamsInit(overlayParams);
    Dss_dctrlOverlayLayerParamsInit(layerParams);
    Dss_dctrlPathInfoInit(pathInfo);

    pathInfo->edgeInfo[pathInfo->numEdges].startNode = gTestCfg.pipeNodeId;
    pathInfo->edgeInfo[pathInfo->numEdges].endNode = TEST_DCTRL_OVERLAY_NODE_ID;
    pathInfo->numEdges++;
    pathInfo->edgeInfo[pathInfo->numEdges].startNode = TEST_DCTRL_OVERLAY_NODE_ID;
    pathInfo->edgeInfo[pathInfo->numEdges].endNode = TEST_DCTRL_VP_NODE_ID;
    pathInfo->numEdges++;
    pathInfo->edgeInfo[pathInfo->numEdges].startNode = TEST_DCTRL_VP_NODE_ID;
    pathInfo->edgeInfo[pathInfo->numEdges].endNode = TEST_DCTRL_OUT_NODE_ID;
    pathInfo->numEdges++;

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_PATH,
        pathInfo,
        NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Dctrl Set Path IOCTL Failed!!!\r\n");
    }

    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_VP_PARAMS,
        vpParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": Dctrl Set VP Params IOCTL Failed!!!\r\n");
    }

#if defined (SOC_AM65XX)
    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_OLDI_PARAMS,
        oldiParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DCTRL Set OLDI Params IOCTL Failed!!!\r\n");
    }
#endif

    overlayParams->overlayId = TEST_OVERLAY_ID;
    overlayParams->colorbarEnable = FALSE;
    overlayParams->overlayCfg.colorKeyEnable = TRUE;
    overlayParams->overlayCfg.colorKeySel = CSL_DSS_OVERLAY_TRANS_COLOR_DEST;
    overlayParams->overlayCfg.backGroundColor = 0x101010U;
    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_OVERLAY_PARAMS,
        overlayParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DCTRL Set Overlay Params IOCTL Failed!!!\r\n");
    }

    layerParams->overlayId = TEST_OVERLAY_ID;
    layerParams->pipeLayerNum[gTestCfg.pipeId] = CSL_DSS_OVERLAY_LAYER_NUM_1;
#if defined (SOC_J7)
    layerParams->pipeLayerNum[gTestCfg.invalidPipeId] =
                                                CSL_DSS_OVERLAY_LAYER_INVALID;
#endif
    retVal = Fvid2_control(
        appObj->dctrlHandle,
        IOCTL_DSS_DCTRL_SET_LAYER_PARAMS,
        layerParams,
        NULL);
    if(retVal != FVID2_SOK)
    {
        GT_0trace(CalTrace, GT_ERR,
              APP_NAME ": DCTRL Set Layer Params IOCTL Failed!!!\r\n");
    }

    return (retVal);
}

static int32_t DispApp_pipeCbFxn(Fvid2_Handle handle, void *appData)
{
    int32_t retVal  = FVID2_SOK;
    appCaptObj_t *appObj = (appCaptObj_t *) appData;
    GT_assert(DssTrace, (NULL != appObj));
    static Fvid2_FrameList frmList;

    retVal = Fvid2_dequeue(appObj->instObj.drvHandle, &frmList, 0U, FVID2_TIMEOUT_NONE);
    if(FVID2_SOK == retVal)
    {
        retVal = Fvid2_queue(appObj->drvHandle, &frmList, 0U);
        if (FVID2_SOK != retVal)
        {
            while(1);
        }
    }

    return (retVal);
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
