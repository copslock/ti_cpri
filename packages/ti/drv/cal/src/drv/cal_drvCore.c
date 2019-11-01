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
 *  \file cal_drvCore.c
 *
 *  \brief File containing the CAL capture driver functions related to core
 *  interactions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/cal/src/drv/cal_drvPriv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define CAL_TODO_DDR_MHz    (333U * (10U ^ 6U))

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t calDrvCaptOpenCoreInt(CalDrv_CaptInstObj *instObj);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t calDrvCaptGetPropCore(CalDrv_CaptInstObj *instObj)
{
    int32_t retVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->getProperty));

    /* Get core property */
    retVal = instObj->coreOps->getProperty(
        instObj->coreInstObj,
        &instObj->coreProperty);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR, "Get core property failed!!\r\n");
    }

    return (retVal);
}

/**
 *  \brief Open and configure the core connected to the driver instance.
 */
int32_t calDrvCaptOpenCore(CalDrv_CaptInstObj *instObj)
{
    int32_t retVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));

    instObj->coreHandle = NULL;

    /* Call the respective core open function */
    switch (instObj->coreProperty.name)
    {
        case CAL_CORE_CAPT:
            retVal = calDrvCaptOpenCoreInt(instObj);
            break;

        default:
            GT_0trace(CalTrace, GT_ERR, "Unknown core!!\r\n");
            retVal = FVID2_EFAIL;
            break;
    }

    return (retVal);
}

int32_t calDrvCaptCloseCore(CalDrv_CaptInstObj *instObj)
{
    int32_t  retVal = FVID2_SOK;
    
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->close));
    GT_assert(CalTrace, (NULL != instObj->coreHandle));

    if (NULL != instObj->coreHandle)
    {
        /* Close core */
        retVal = instObj->coreOps->close(instObj->coreHandle);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR, "Core close failed!!\r\n");
        }
        instObj->coreHandle = NULL;
    }

    return (retVal);
}

int32_t calDrvCaptStartCore(CalDrv_CaptInstObj *instObj)
{
    int32_t retVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->start));
    GT_assert(CalTrace, (NULL != instObj->coreHandle));

    /* Start core */
    retVal = instObj->coreOps->start(instObj->coreHandle);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR, "Core start failed!!\r\n");
    }

    return (retVal);
}

int32_t calDrvCaptStopCore(CalDrv_CaptInstObj *instObj)
{
    int32_t retVal = FVID2_SOK;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->stop));
    GT_assert(CalTrace, (NULL != instObj->coreHandle));

    /* Stop core */
    retVal = instObj->coreOps->stop(instObj->coreHandle);
    if (FVID2_SOK != retVal)
    {
        GT_0trace(CalTrace, GT_ERR, "Core stop failed!!\r\n");
    }

    return (retVal);
}

static int32_t calDrvCaptOpenCoreInt(CalDrv_CaptInstObj *instObj)
{
    int32_t                       retVal = FVID2_SOK;
    uint32_t                      index;
    Cal_CoreOpenPrms            openPrms;
    Cal_CaptOpenParams_t    *captOpenParms;
    Cal_CoreCaptOpenParams_t    coreOpenParms;
    Cal_CoreCaptOpenRetParams_t coreRtnParms;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->open));

    captOpenParms = (Cal_CaptOpenParams_t *)
                           instObj->createPrms.pAdditionalArgs;
    GT_assert(CalTrace, (NULL != captOpenParms));

    coreOpenParms.arg          = NULL;
    coreOpenParms.numStreams   = instObj->createPrms.numStream;

    if (FVID2_VIFM_SCH_CSI2 == instObj->createPrms.videoIfMode)
    {
        coreOpenParms.captIf = CAL_CORE_CAPT_IF_CSI2;
    }
    if (FVID2_VIFM_SCH_CPI == instObj->createPrms.videoIfMode)
    {
        coreOpenParms.captIf = CAL_CORE_CAPT_IF_CPI;
    }

    for (index = 0; index < coreOpenParms.numStreams; index++)
    {
        coreOpenParms.subModules[index] =
            captOpenParms->subModules[index];
    }
    for (index = 0; index < CSL_CAL_CMPLXIO_CNT; index++)
    {
        coreOpenParms.csi2PhyClock[index] =
                                    captOpenParms->csi2PhyClock[index];
        if (TRUE == captOpenParms->isCmplxIoCfgValid[index])
        {
            Fvid2Utils_memcpy(
                &coreOpenParms.cmplxIoCfg[index],
                &captOpenParms->cmplxIoCfg[index],
                sizeof (Cal_CmplxIoCfg_t));
        }
        coreOpenParms.isCmplxIoCfgValid[index] =
                                captOpenParms->isCmplxIoCfgValid[index];
    }

    instObj->coreHandle = NULL;

    Fvid2Utils_memset(&instObj->frmEvtNotifyPrms, 0,
                    sizeof (Cal_FrameEventNotifyCfg_t));

    openPrms.drvData   = instObj;
    openPrms.reqFrmCb  = &calDrvCaptCoreReqFrameCb;
    openPrms.frmDoneCb = &calDrvCaptCoreFrameDoneCb;

    /* Open the core instance */
    instObj->coreHandle = instObj->coreOps->open(
        instObj->coreInstObj,
        &openPrms,
        &coreOpenParms,
        &coreRtnParms);
    if (NULL != instObj->coreHandle)
    {
        for (index = 0; index < coreOpenParms.numStreams; index++)
        {
            instObj->chObj[index][0U].isStreamByPassed =
                coreRtnParms.isStreamOpt[index];
        }
    }
    else
    {
        GT_0trace(CalTrace, GT_ERR, "CAL Capture core open failed!!\r\n");
        retVal = FVID2_EFAIL;
    }
    return (retVal);
}

int32_t calDrvCaptSetCoreParams(CalDrv_CaptInstObj   *instObj,
                                 const Cal_Cfg_t *pPrms)
{
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->control));
    return (instObj->coreOps->control(instObj->coreHandle,
                                      CAL_CORE_CAPT_SET_PARAMS,
                                      (void *) pPrms, NULL));
}

int32_t calDrvCaptSetErrorParams(CalDrv_CaptInstObj        *instObj,
                                  const Cal_ErrorCfg_t *pPrms)
{
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->control));
    return (instObj->coreOps->control(instObj->coreHandle,
                                      CAL_CORE_CAPT_SET_ERR_PRMS,
                                      (void *) pPrms, NULL));
}

int32_t calDrvCaptSubFrmCbFxn (void *drvData,
                                const Cal_CoreFrame *coreFrm,
                                Fvid2_SubFrameInfo  *subFrmInfo)
{
    uintptr_t cookie;
    uint64_t curTimeStamp;
    CalDrv_CaptInstObj *instObj;
    CalDrv_CaptQueObj  *qObj;

    instObj = (CalDrv_CaptInstObj *) drvData;
    GT_assert(CalTrace, (NULL != instObj));

    /* Valid frame event notification */
    if (0U != instObj->frmEvtNotifyPrms.numStream)
    {
        GT_assert(CalTrace, (NULL != coreFrm));
        GT_assert(CalTrace, (NULL != subFrmInfo));
        GT_assert(CalTrace, (coreFrm->streamId < CAL_CAPT_MAX_STREAMS));
        GT_assert(CalTrace, (coreFrm->chId < instObj->createPrms.numCh));


        cookie = HwiP_disable();

        GT_assert(CalTrace, (NULL != instObj->getTimeStamp));
        curTimeStamp = instObj->getTimeStamp(NULL);

        /* Get reference of the queue head */
        qObj = (CalDrv_CaptQueObj *) coreFrm->drvData;
        GT_assert(CalTrace, (NULL != qObj));
        GT_assert(CalTrace, (NULL != qObj->frm));
        GT_assert(CalTrace, (NULL != qObj->frm->subFrameInfo));

        qObj->frm->timeStamp64 = curTimeStamp;

        qObj->frm->subFrameInfo->subFrameNum = subFrmInfo->subFrameNum;
        qObj->frm->subFrameInfo->numOutLines = subFrmInfo->numOutLines;
        qObj->frm->subFrameInfo->numInLines  = 0U;

        /* Let Apps know */
        instObj->frmEvtNotifyPrms.appCb((Fdrv_Handle) instObj, qObj->frm);

        HwiP_restore(cookie);
    }
    return (FVID2_SOK);
}

int32_t calDrvCaptSetSubFrmParams(CalDrv_CaptInstObj        *instObj,
                                  const Cal_FrameEventNotifyCfg_t *pPrms)
{
    int32_t rtnVal;
    uintptr_t cookie;
    uint32_t streamId;
    static Cal_CoreCaptSubFrameCfg_t corePrms;

    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != instObj->coreOps));
    GT_assert(CalTrace, (NULL != instObj->coreOps->control));

    cookie = HwiP_disable();
    instObj->frmEvtNotifyPrms.numStream = 0U;

    corePrms.numStream = pPrms->numStream;
    for (streamId = 0U; streamId < pPrms->numStream; streamId++)
    {
        corePrms.streamId[streamId] = pPrms->streamId[streamId];
        corePrms.notifyAfterFirstXLines[streamId] =
                                        pPrms->notifyAfterFirstXLines[streamId];
        corePrms.notifyAfterEndOfFrame[streamId]  =
                                        pPrms->notifyAfterEndOfFrame[streamId];
        corePrms.streamId[streamId] = pPrms->streamId[streamId];
    }
    corePrms.appCb = (Cal_CoreSubFrameCbFxn) &calDrvCaptSubFrmCbFxn;
    corePrms.pAppCbArgs = (void *)instObj;
    corePrms.pAdditionalArgs = (void *)NULL;

    rtnVal = instObj->coreOps->control(instObj->coreHandle,
                                      CAL_CORE_CAPT_SET_SUB_FRM_PRMS,
                                      (void *) &corePrms, NULL);
    if (FVID2_SOK == rtnVal)
    {
        Fvid2Utils_memcpy(&instObj->frmEvtNotifyPrms,
                        pPrms,
                        sizeof (Cal_FrameEventNotifyCfg_t));
    }
    HwiP_restore(cookie);
    return (rtnVal);
}

