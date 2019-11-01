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
 *  \file cal_drvApi.c
 *
 *  \brief File containing the CAL capture driver APIs.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/cal/src/drv/cal_drvPriv.h>

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
/**
 *  Below ifdef __cplusplus is added so that C++ build passes without
 *  typecasting. This is because the prototype is build as C type
 *  whereas this file is build as CPP file. Hence we get C++ build error.
 *  Also if tyecasting is used, then we get MisraC error Rule 11.1.
 */
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Capture API's
 */
static Fdrv_Handle CalDrv_captCreate(uint32_t                   drvId,
                                     uint32_t                   instId,
                                     void                      *createArgs,
                                     void                      *createStatusArgs,
                                     const Fvid2_DrvCbParams *fdmCbPrms);
static int32_t CalDrv_captDelete(Fdrv_Handle handle, void *reserved);
static int32_t CalDrv_captQueue(Fdrv_Handle      handle,
                              Fvid2_FrameList *frmList,
                              uint32_t           streamId);
static int32_t CalDrv_captDequeue(Fdrv_Handle      handle,
                                Fvid2_FrameList *frmList,
                                uint32_t           streamId,
                                uint32_t           timeout);
static int32_t CalDrv_captControl(Fdrv_Handle handle,
                                uint32_t      cmd,
                                void         *cmdArgs,
                                void         *cmdStatusArgs);

#ifdef __cplusplus
}
#endif

/*
 * Capture IOCTLs
 */
static int32_t calDrvCaptStartIoctl(CalDrv_CaptInstObj *instObj);
static int32_t calDrvCaptStopIoctl(CalDrv_CaptInstObj *instObj);
static int32_t calDrvCaptGetStatusIoctl(const CalDrv_CaptInstObj *instObj,
                                      Cal_CaptStatus           *captStat);
static int32_t calDrvCaptGetChStatusIoctl(const CalDrv_CaptInstObj   *instObj,
                                        const Cal_CaptChStatusArgs *chStatArgs,
                                        Cal_CaptChStatus           *chStat);
static int32_t calDrvCaptRegisterTimeStampIoctl(
    CalDrv_CaptInstObj        *instObj,
    const Fvid2_TimeStampParams *timeStampPrms);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief CAL capture driver init function.
 */
int32_t CalDrv_captInit(uint32_t numInst, const CalDrv_CaptInitParams *initPrms)
{
    int32_t retVal = FVID2_SOK;
    CalDrv_CaptCommonObj *pObj;

    retVal = calDrvCaptPrivInit(numInst, initPrms);
    if (FVID2_SOK == retVal)
    {
        pObj = &gCalCaptCommonObj;
        /* Initialize Driver operations */
        Fvid2DrvOps_init(&pObj->fvidDrvOps);

        pObj->fvidDrvOps.drvId      = FVID2_CAL_CAPT_DRV_ID;
        pObj->fvidDrvOps.createFxn  = &CalDrv_captCreate;
        pObj->fvidDrvOps.deleteFxn  = &CalDrv_captDelete;
        pObj->fvidDrvOps.controlFxn = &CalDrv_captControl;
        pObj->fvidDrvOps.queueFxn   = &CalDrv_captQueue;
        pObj->fvidDrvOps.dequeueFxn = &CalDrv_captDequeue;

        retVal = Fvid2_registerDriver(&pObj->fvidDrvOps);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "Registering to FVID2 driver manager failed\r\n");
            /* Uninitialize the internal objects if error occurs */
            calDrvCaptPrivDeInit();
        }
        else
        {
            /* Init successful */
            pObj->isRegistered = (uint32_t) TRUE;
            pObj->numInst      = numInst;
        }
    }

    return (retVal);
}

/**
 *  \brief CAL capture driver deinit function.
 */
int32_t CalDrv_captDeInit(void)
{
    int32_t retVal = FVID2_SOK;
    CalDrv_CaptCommonObj *pObj;

    pObj = &gCalCaptCommonObj;
    if (TRUE == pObj->isRegistered)
    {
        /* Unregister from driver manager */
        retVal = Fvid2_unRegisterDriver(&pObj->fvidDrvOps);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "Unregistering from FVID2 driver manager failed\r\n");
        }
        pObj->isRegistered = (uint32_t) FALSE;
    }

    retVal += calDrvCaptPrivDeInit();

    return (retVal);
}

/**
 *  \brief CAL capture driver create function.
 */
static Fdrv_Handle CalDrv_captCreate(uint32_t                   drvId,
                                     uint32_t                   instId,
                                     void                      *createArgs,
                                     void                      *createStatusArgs,
                                     const Fvid2_DrvCbParams   *fdmCbPrms)
{
    int32_t                 retVal = FVID2_SOK, tempRetVal;
    uint32_t                streamId;
    uint32_t                chCreateFlag = FALSE, coreCreateFlag = FALSE;
    Fdrv_Handle           drvHandle    = NULL;
    CalDrv_CaptInstObj   *instObj      = NULL;
    Cal_CaptCreateParams *createPrms;
    Cal_CaptCreateStatus *createStatus;

    /* Check for NULL pointers and invalid arguments */
    if ((NULL == createArgs) ||
        (NULL == createStatusArgs) ||
        (NULL == fdmCbPrms) ||
        (FVID2_CAL_CAPT_DRV_ID != drvId))
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid arguments\r\n");
        retVal = FVID2_EBADARGS;
    }
    else
    {
        /* Get the instance object for this instance */
        instObj = calDrvCaptGetInstObj(instId);
        if (NULL == instObj)
        {
            GT_0trace(CalTrace, GT_ERR, "Invalid instance ID\r\n");
            retVal = FVID2_EINVALID_PARAMS;
        }
    }

    if (NULL != instObj)
    {
        /* Take the instance semaphore */
        SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

        /* Check if the instance is already opened */
        if (CAL_CAPT_STATE_IDLE != instObj->state)
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "DEVICE_INUSE: Driver instance already created!!\r\n");
            retVal = FVID2_EDEVICE_INUSE;
        }
    }

    if (FVID2_SOK == retVal)
    {
        /* Check for valid create parameters and copy them */
        createPrms = (Cal_CaptCreateParams *) createArgs;
        retVal     = calDrvCaptCheckParams(instObj, createPrms);
        if (FVID2_SOK == retVal)
        {
            Fvid2Utils_memcpy(
                &instObj->createPrms,
                createPrms,
                sizeof (instObj->createPrms));
        }
    }

    if (FVID2_SOK == retVal)
    {
        /* Get core property */
        retVal = calDrvCaptGetPropCore(instObj);
    }

    if (FVID2_SOK == retVal)
    {
        retVal = calDrvCaptCreateChObj(instObj);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "Channel object create failed!!\r\n");
        }
        else
        {
            chCreateFlag = (uint32_t) TRUE;
        }
    }

    if (FVID2_SOK == retVal)
    {
        retVal = calDrvCaptOpenCore(instObj);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR, "Core open failed!!\r\n");
        }
        else
        {
            coreCreateFlag = (uint32_t) TRUE;
        }
    }

    if (FVID2_SOK == retVal)
    {
        for (streamId = 0U; streamId < CAL_CAPT_MAX_STREAMS; streamId++)
        {
            instObj->doneQ[streamId] = NULL;
        }

        for (streamId = 0U;
             streamId < instObj->createPrms.numStream;
             streamId++)
        {
            /* Create done Queue */
            tempRetVal = Fvid2Utils_constructQ(&instObj->doneLlObj[streamId]);
            GT_assert(CalTrace, (tempRetVal == FVID2_SOK));
            instObj->doneQ[streamId] = &instObj->doneLlObj[streamId];
        }

        Fvid2Utils_memcpy(
            &instObj->fdmCbPrms,
            fdmCbPrms,
            sizeof (instObj->fdmCbPrms));

        instObj->queueCount            = 0U;
        instObj->dequeueCount          = 0U;
        instObj->overflowCount         = 0U;
        instObj->asynOverflowCount     = 0U;
        instObj->actProtViolationCount = 0U;
        instObj->ancProtViolationCount = 0U;
        instObj->state = CAL_CAPT_STATE_CREATED;
        drvHandle      = instObj;
    }

    /* Return the status if possible */
    if (NULL != createStatusArgs)
    {
        createStatus         = (Cal_CaptCreateStatus *) createStatusArgs;
        createStatus->retVal = retVal;
    }

    if (NULL != instObj)
    {
        /* Deallocate if error occurs */
        if (FVID2_SOK != retVal)
        {
            if ((uint32_t) TRUE == coreCreateFlag)
            {
                tempRetVal = calDrvCaptCloseCore(instObj);
                GT_assert(CalTrace, (FVID2_SOK == tempRetVal));
            }

            if ((uint32_t) TRUE == chCreateFlag)
            {
                tempRetVal = calDrvCaptDeleteChObj(instObj);
                GT_assert(CalTrace, (FVID2_SOK == tempRetVal));
            }
        }

        /* Post the instance semaphore */
        SemaphoreP_post(instObj->lockSem);
    }
    
    return (drvHandle);
}

/**
 *  \brief CAL capture driver delete function.
 */
static int32_t CalDrv_captDelete(Fdrv_Handle handle, void *reserved)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t streamId;
    CalDrv_CaptInstObj *instObj = NULL;

    /* Global handle - do nothing */
    if (((Fdrv_Handle) & gCalCaptCommonObj) == handle)
    {
        retVal = (FVID2_SOK);
    }
    else
    {
        /* Check for NULL pointers and invalid arguments */
        if (NULL == handle)
        {
            GT_0trace(CalTrace, GT_ERR, "Invalid arguments\r\n");
            retVal = FVID2_EBADARGS;
        }
        else
        {
            instObj = (CalDrv_CaptInstObj *) handle;
        }

        if (NULL != instObj)
        {
            /* Take the instance semaphore */
            SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

            /* Check if the instance is already opened and in a stopped state */
            if ((instObj->state == CAL_CAPT_STATE_STOPPED) ||
                (instObj->state == CAL_CAPT_STATE_CREATED))
            {
                retVal = calDrvCaptCloseCore(instObj);
                GT_assert(CalTrace, (FVID2_SOK == retVal));

                retVal = calDrvCaptDeleteChObj(instObj);
                GT_assert(CalTrace, (FVID2_SOK == retVal));

                for (streamId = 0U;
                     streamId < instObj->createPrms.numStream;
                     streamId++)
                {
                    if (NULL != instObj->doneQ[streamId])
                    {
                        /* Delete the done Q */
                        Fvid2Utils_destructQ(instObj->doneQ[streamId]);
                        instObj->doneQ[streamId] = NULL;
                    }
                }

                /* Mark state as idle */
                instObj->state = CAL_CAPT_STATE_IDLE;
            }
            else
            {
                GT_0trace(
                    CalTrace, GT_ERR,
                    "Driver not in a state to delete. Try stopping it!!\r\n");
                retVal = FVID2_EFAIL;
            }
        }

        if (NULL != instObj)
        {
            /* Post the instance semaphore */
            SemaphoreP_post(instObj->lockSem);
        }
    }

    return (retVal);
}

/**
 *  \brief CAL capture driver queue function.
 */
static int32_t CalDrv_captQueue(Fdrv_Handle      handle,
                              Fvid2_FrameList *frmList,
                              uint32_t           streamId)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t frmCnt, chId, instId, lChNum, drvStreamId;
    uintptr_t cookie;
    Fvid2_Frame          *frm;
    CalDrv_CaptChObj     *chObj;
    CalDrv_CaptInstObj   *instObj;
    CalDrv_CaptCommonObj *pObj;
    CalDrv_CaptQueObj    *qObj;

    pObj = &gCalCaptCommonObj;

    /* Check for NULL pointers */
    if ((NULL == handle) || (NULL == frmList))
    {
        GT_0trace(CalTrace, GT_ERR, "NULL pointer\n");
        retVal = FVID2_EBADARGS;
    }
    else if (handle == ((Fdrv_Handle) & gCalCaptCommonObj))
    {
        GT_0trace(
            CalTrace, GT_ERR,
            "Queue not supported for global handle!!\r\n");
        retVal = FVID2_EFAIL;
    }
    else
    {
        instObj = (CalDrv_CaptInstObj *) handle;

        /* Check framelist for error and NULL pointer check */
        retVal = Fvid2_checkFrameList(frmList, (uint32_t) FVID2_MAX_FRAME_PTR);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR, "Check frame list error\n");
        }

        if (CAL_CAPT_STATE_IDLE == instObj->state)
        {
            /* If driver handle is not open then skip this frame queue */
            GT_0trace(
                CalTrace, GT_ERR,
                "Invalid state: Can't queue to an un-opened instance!!\r\n");
            retVal = FVID2_EFAIL;
        }

        if ((CAL_CAPT_STATE_RUNNING == instObj->state) &&
            (CAL_CAPT_BCM_CIRCULAR_FRM_REPEAT ==
             instObj->createPrms.bufCaptMode))
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "Invalid state: Can't queue after capture starts in circular "
                "frame repeat mode!!\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if (FVID2_SOK == retVal)
    {
        instObj->queueCount++;

        /* for all frames that need to be queued */
        for (frmCnt = 0; frmCnt < frmList->numFrames; frmCnt++)
        {
            /* Get FVID2 frame pointer - NULL check is already done in
             * check frame list function */
            frm = frmList->frames[frmCnt];
            GT_assert(CalTrace, (NULL != frm));

            if (retVal == FVID2_SOK)
            {
                /* Map user channel number to driver channel number */
                lChNum = pObj->fvidChNumToLogChNumMap[frm->chNum];

                /* extract driver instance ID from driver channel number */
                instId = Cal_captGetInstId(lChNum);
            }
            if (retVal == FVID2_SOK)
            {
                if (instId != instObj->drvInstId)
                {
                    /* If mismatch then chNum in FVID2 Frame is not correct,
                     * skip this frame queue */
                    GT_0trace(
                        CalTrace, GT_ERR, "Instance ID mismatch\r\n");
                    retVal = FVID2_EFAIL;
                }
            }
            if (retVal == FVID2_SOK)
            {
                drvStreamId = streamId;
                streamId = Cal_captGetStreamId(lChNum);
                if ((drvStreamId >= instObj->createPrms.numStream) ||
                    (drvStreamId != streamId))
                {
                    /* invalid stream ID skip this frame queue */
                    GT_0trace(CalTrace, GT_ERR, "Stream ID mismatch\r\n");
                    retVal = FVID2_EFAIL;
                }
            }
            if (retVal == FVID2_SOK)
            {
                /* Get channel ID */
                chId = Cal_captGetChId(lChNum);
                if (chId >= instObj->createPrms.numCh)
                {
                    /* Invalid channel ID skip this frame queue */
                    GT_0trace(CalTrace, GT_ERR, "Invalid channel ID\r\n");
                    retVal = FVID2_EFAIL;
                }
            }
            if (retVal == FVID2_SOK)
            {
                /*
                 * valid instance, stream and channel
                 */
                /* Get channel specific object in the required instance */
                chObj = &instObj->chObj[drvStreamId][chId];

                cookie = HwiP_disable();

                /* Allocate a free queue object from the pool */
                qObj =
                    (CalDrv_CaptQueObj *) Fvid2Utils_dequeue(chObj->bmObj.freeQ);
                if (NULL == qObj)
                {
                    HwiP_restore(cookie);
                    GT_0trace(
                        CalTrace, GT_ERR,
                        "ALLOC: Q object allocation failed\r\n");
                    retVal = FVID2_EALLOC;
                }
            }
            if (retVal == FVID2_SOK)
            {
                /* Copy the frame to the driver's queue object */
                qObj->frm = frm;
                /* Initial credit count should be zero!! */
                GT_assert(CalTrace, (0U == qObj->creditCnt));

                /* Add the queue object in driver's request queue */
                Fvid2Utils_queue(chObj->bmObj.reqQ, &qObj->qElem, qObj);
                chObj->stat.queueCount++;

                HwiP_restore(cookie);

                /* Mark frame in frmList as NULL */
                frmList->frames[frmCnt] = NULL;
            }
            if (retVal != FVID2_SOK)
            {
                break;
            }
        }
    }

    return (retVal);
}

/**
 *  \brief CAL capture driver dequeue function.
 */
static int32_t CalDrv_captDequeue(Fdrv_Handle      handle,
                                Fvid2_FrameList *frmList,
                                uint32_t           streamId,
                                uint32_t           timeout)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t chCnt;
    uintptr_t cookie;
    CalDrv_CaptChObj   *chObj;
    CalDrv_CaptInstObj *instObj;
    CalDrv_CaptQueObj  *qObj;

    /* Check for NULL pointers */
    if ((NULL == handle) ||
        (NULL == frmList) ||
        (streamId >= CAL_CAPT_MAX_STREAMS))
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid argument!!\r\n");
        retVal = FVID2_EBADARGS;
    }
    else if (handle == ((Fdrv_Handle) & gCalCaptCommonObj))
    {
        GT_0trace(
            CalTrace, GT_ERR,
            "Dequeue not supported for global handle!!\r\n");
        retVal = FVID2_EFAIL;
    }
    else
    {
        instObj = (CalDrv_CaptInstObj *) handle;
        if (instObj->state == CAL_CAPT_STATE_IDLE)
        {
            GT_0trace(
                CalTrace, GT_ERR,
                "Invalid state: Can't dequeue to an un-opened instance!!\r\n");
            retVal = FVID2_EFAIL;
        }

        /* validate stream ID */
        if (streamId >= instObj->createPrms.numStream)
        {
            GT_0trace(CalTrace, GT_ERR, "Invalid stream ID!!\r\n");
            retVal = FVID2_EFAIL;
        }
    }

    if (FVID2_SOK == retVal)
    {
        /* init frame list fields */
        frmList->numFrames  = 0U;
        frmList->perListCfg = NULL;

        /* Extract done frames from queue till the queue
         * is empty or framelist runs out of space!! */
        do
        {
            cookie = HwiP_disable();
            qObj   = (CalDrv_CaptQueObj *)
                     Fvid2Utils_dequeue(instObj->doneQ[streamId]);
            if (NULL == qObj)
            {
                /* No more buffers in instance object */
                HwiP_restore(cookie);
            }
            else
            {
                GT_assert(CalTrace, (NULL != qObj->frm));
                frmList->frames[frmList->numFrames] = qObj->frm;
                frmList->numFrames++;

                /* Give back the queue object back to the free pool */
                qObj->frm = NULL;
                /* At time of deqeue, credit should be zero!! */
                GT_assert(CalTrace, (0U == qObj->creditCnt));
                chObj = qObj->chObj;
                GT_assert(CalTrace, (NULL != chObj));
                Fvid2Utils_queue(chObj->bmObj.freeQ, &qObj->qElem, qObj);
                chObj->stat.dequeueCount++;

                HwiP_restore(cookie);
            }

            /* Max frames limit exceeded exit */
            if ((frmList->numFrames >= FVID2_MAX_FRAME_PTR) ||
                (NULL == qObj))
            {
                break;
            }

            /* TODO: Break for oneCallBackPerFrm */
        } while (1U);
    }

    /* If no frame is dequeued, then check if we could return frames from input
     * and current queue for stopped instances */
    if ((FVID2_SOK == retVal) && (0U == frmList->numFrames))
    {
        /* When an instance is stopped, return all the request in
         * channel's input and current queue as well. */
        GT_assert(CalTrace, (NULL != instObj));
        if ((CAL_CAPT_STATE_CREATED == instObj->state) ||
            (CAL_CAPT_STATE_STOPPED == instObj->state))
        {
            cookie = HwiP_disable();

            for (chCnt = 0U; chCnt < instObj->createPrms.numCh; chCnt++)
            {
                chObj = &instObj->chObj[streamId][chCnt];

                /* Extract frames from queue till the queue
                 * is empty or framelist runs out of space!! */
                do
                {
                    /* Give the buffers in current state */
                    qObj = (CalDrv_CaptQueObj *)
                           Fvid2Utils_dequeue(chObj->bmObj.curQ);
                    if (NULL == qObj)
                    {
                        /* At last give back buffers in request queue as well */
                        qObj = (CalDrv_CaptQueObj *)
                               Fvid2Utils_dequeue(chObj->bmObj.reqQ);
                    }

                    if (NULL != qObj)
                    {
                        GT_assert(CalTrace, (NULL != qObj->frm));
                        frmList->frames[frmList->numFrames] = qObj->frm;
                        frmList->numFrames++;

                        /* Give back the queue object back to the free pool */
                        qObj->frm = NULL;
                        /* At time of deqeue, credit should be zero!! */
                        GT_assert(CalTrace, (0U == qObj->creditCnt));
                        GT_assert(CalTrace, (chObj == qObj->chObj));
                        Fvid2Utils_queue(chObj->bmObj.freeQ, &qObj->qElem, qObj);
                        chObj->stat.dequeueCount++;
                    }

                    /* Max frames limit exceeded exit or no more frames for this
                     * channel, continue with next channel */
                    if ((frmList->numFrames >= FVID2_MAX_FRAME_PTR) ||
                        (NULL == qObj))
                    {
                        break;      /* do () */
                    }

                    /* TODO: Break for oneCallBackPerFrm */
                } while (1U);

                /* Max frames limit exceeded exit */
                if (frmList->numFrames >= FVID2_MAX_FRAME_PTR)
                {
                    break;      /* for (chCnt) */
                }
            }

            HwiP_restore(cookie);
        }
    }

    if (FVID2_SOK == retVal)
    {
        /* If no frame is dequeued, then return appropriate error */
        if (0U == frmList->numFrames)
        {
            if ((CAL_CAPT_STATE_CREATED == instObj->state) ||
                (CAL_CAPT_STATE_STOPPED == instObj->state))
            {
                GT_0trace(
                    CalTrace, GT_DEBUG,
                    "NO_MORE_BUFFERS: No more buffers with driver\n");
                retVal = FVID2_ENO_MORE_BUFFERS;
            }
            else
            {
                GT_0trace(
                    CalTrace, GT_DEBUG,
                    "AGAIN: Out queue Empty. Try again\r\n");
                retVal = FVID2_EAGAIN;
            }
        }
        else
        {
            instObj->dequeueCount++;
        }
    }

    return (retVal);
}

/**
 *  \brief CAL capture driver control function.
 */
static int32_t CalDrv_captControl(Fdrv_Handle handle,
                                  uint32_t    cmd,
                                  void       *cmdArgs,
                                  void       *cmdStatusArgs)
{
    int32_t retVal = FVID2_SOK;
    CalDrv_CaptInstObj *instObj;

    /* Check for NULL pointers */
    if (NULL == handle)
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid argument!!\r\n");
        retVal = FVID2_EBADARGS;
    }
    else if (handle == (Fdrv_Handle) & gCalCaptCommonObj)
    {
        switch (cmd)
        {
            default:
                GT_0trace(
                    CalTrace, GT_ERR,
                    "UNSUPPORTED_CMD: IOCTL not supported for global handle\r\n");
                retVal = FVID2_EUNSUPPORTED_CMD;
                break;
        }
    }
    else
    {
        instObj = (CalDrv_CaptInstObj *) handle;
        switch (cmd)
        {
            case FVID2_START:
                retVal = calDrvCaptStartIoctl(instObj);
                break;

            case FVID2_STOP:
                retVal = calDrvCaptStopIoctl(instObj);
                break;

            case IOCTL_CAL_CAPT_GET_STATUS:
            {
                retVal = calDrvCaptGetStatusIoctl(
                    instObj,
                    (Cal_CaptStatus *) cmdArgs);
                break;
            }

            case IOCTL_CAL_CAPT_GET_CH_STATUS:
            {
                retVal = calDrvCaptGetChStatusIoctl(
                    instObj,
                    (const Cal_CaptChStatusArgs *) cmdArgs,
                    (Cal_CaptChStatus *) cmdStatusArgs);
                break;
            }

            case FVID2_REGISTER_TIMESTAMP_FXN:
            {
                retVal = calDrvCaptRegisterTimeStampIoctl(
                    instObj,
                    (const Fvid2_TimeStampParams *) cmdArgs);
                break;
            }

            case IOCTL_CAL_CAPT_SET_PARAMS:
            {
                retVal = calDrvCaptSetCoreParams(instObj,
                                                    (Cal_Cfg_t *) cmdArgs);
                break;
            }
            case IOCTL_CAL_CAPT_SET_ERR_PRMS:
            {
                retVal = calDrvCaptSetErrorParams(instObj,
                                                    (Cal_ErrorCfg_t*)
                                                        cmdArgs);
                break;
            }
            case IOCTL_CAL_CAPT_SET_FRAME_EVENT_NOTIFY_PRMS:
            {
                retVal = calDrvCaptSetSubFrmParams(instObj,
                                    (Cal_FrameEventNotifyCfg_t*) cmdArgs);
                break;
            }
            default:
                GT_0trace(CalTrace, GT_ERR,
                            "UNSUPPORTED_CMD: IOCTL not supported\r\n");
                retVal = FVID2_EUNSUPPORTED_CMD;
                break;
        }
    }

    return (retVal);
}

static int32_t calDrvCaptStartIoctl(CalDrv_CaptInstObj *instObj)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t numElemInReqQ;
    uint32_t streamId, chId;
    CalDrv_CaptChObj *chObj;

    /* Check for NULL pointers */
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace,
              (instObj->createPrms.numStream <= CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace,
              (instObj->createPrms.numCh <= CAL_CAPT_CH_PER_PORT_MAX));
    /* Take the instance semaphore */
    SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

    /* For every stream and every channel */
    for (streamId = 0U; streamId < instObj->createPrms.numStream; streamId++)
    {
        for (chId = 0; chId < instObj->createPrms.numCh; chId++)
        {
            /* Get channel object */
            chObj = &instObj->chObj[streamId][chId];

            /* Check if the number of elements in the queue is sufficient to */
            numElemInReqQ = Fvid2Utils_getNumQElem(chObj->bmObj.reqQ);
            /* Check if the primed buffers are sufficient - For frame drop no
             * frames need to be queued!! */
            if ((numElemInReqQ == 0U) && (FALSE == chObj->isStreamByPassed))
            {
                GT_2trace(
                    CalTrace, GT_ERR,
                    "Insufficient buffers queued for stream %d channel %d\r\n",
                    streamId, chId);
                retVal = FVID2_EFAIL;
                break;
            }
        }
    }

    if (FVID2_SOK == retVal)
    {
        instObj->overflowCount = 0U;
        retVal = calDrvCaptStartCore(instObj);
        if (FVID2_SOK != retVal)
        {
            GT_0trace(CalTrace, GT_ERR, "Core start failed!!\r\n");
        }
    }

    if (FVID2_SOK == retVal)
    {
        instObj->state = CAL_CAPT_STATE_RUNNING;
    }

    /* Post the instance semaphore */
    SemaphoreP_post(instObj->lockSem);

    return (retVal);
}

static int32_t calDrvCaptStopIoctl(CalDrv_CaptInstObj *instObj)
{
    int32_t  retVal = FVID2_SOK;
    uintptr_t cookie;
    uint32_t streamId, chId;
    CalDrv_CaptChObj  *chObj;
    CalDrv_CaptQueObj *qObj;

    /* Check for NULL pointers */
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace,
              (instObj->createPrms.numStream <= CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace,
              (instObj->createPrms.numCh <= CAL_CAPT_CH_PER_PORT_MAX));

    /* Take the instance semaphore */
    SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

    retVal = calDrvCaptStopCore(instObj);
    if (FVID2_SOK == retVal)
    {
        cookie = HwiP_disable();

        instObj->state = CAL_CAPT_STATE_STOPPED;

        /* For every stream and every channel */
        for (streamId = 0U;
             streamId < instObj->createPrms.numStream;
             streamId++)
        {
            for (chId = 0; chId < instObj->createPrms.numCh; chId++)
            {
                /* Get channel object */
                chObj = &instObj->chObj[streamId][chId];

                /* Dequeue all the current frames and put back in
                 * request queue */
                while (1U)
                {
                    qObj = (CalDrv_CaptQueObj *)
                           Fvid2Utils_dequeue(chObj->bmObj.curQ);
                    if (NULL == qObj)
                    {
                        break;
                    }
                    qObj->creditCnt = 0U;
                    Fvid2Utils_queueBack(chObj->bmObj.reqQ, &qObj->qElem, qObj);
                }
            }
        }

        HwiP_restore(cookie);
    }

    /* Post the instance semaphore */
    SemaphoreP_post(instObj->lockSem);

    return (retVal);
}

static int32_t calDrvCaptGetStatusIoctl(const CalDrv_CaptInstObj *instObj,
                                      Cal_CaptStatus           *captStat)
{
    int32_t retVal = FVID2_SOK;

    /* Check for NULL pointers */
    GT_assert(CalTrace, (NULL != instObj));

    /* Take the instance semaphore */
    SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

    if (NULL == captStat)
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid argument!!\r\n");
        retVal = FVID2_EBADARGS;
    }
    else
    {
        captStat->queueCount    = instObj->queueCount;
        captStat->dequeueCount  = instObj->dequeueCount;
        captStat->overflowCount = instObj->overflowCount;
    }

    /* Post the instance semaphore */
    SemaphoreP_post(instObj->lockSem);

    return (retVal);
}

static int32_t calDrvCaptGetChStatusIoctl(const CalDrv_CaptInstObj   *instObj,
                                        const Cal_CaptChStatusArgs *chStatArgs,
                                        Cal_CaptChStatus           *chStat)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t chId, lChNum, drvStreamId;
    uint64_t curTime, curTimeMs;
    const CalDrv_CaptChObj *chObj;
    CalDrv_CaptCommonObj   *pObj;

    pObj = &gCalCaptCommonObj;
    GT_assert(CalTrace, (NULL != instObj));

    /* Take the instance semaphore */
    SemaphoreP_pend(instObj->lockSem, SemaphoreP_WAIT_FOREVER);

    /* Check for NULL pointers */
    if ((NULL == chStatArgs) || (NULL == chStat))
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid argument!!\r\n");
        retVal = FVID2_EBADARGS;
    }
    else
    {
        /* Map user channel number to driver channel number */
        lChNum = (uint32_t) pObj->fvidChNumToLogChNumMap[chStatArgs->chNum];

        /* Get channel and stream ID */
        chId        = Cal_captGetChId(lChNum);
        drvStreamId = Cal_captGetStreamId(lChNum);
        if (chId >= instObj->createPrms.numCh)
        {
            /* Invalid channel ID */
            GT_0trace(CalTrace, GT_ERR, "Invalid channel ID\r\n");
            retVal = FVID2_EINVALID_PARAMS;
        }
        if (drvStreamId >= instObj->createPrms.numStream)
        {
            /* Invalid stream ID */
            GT_0trace(CalTrace, GT_ERR, "Invalid stream ID\r\n");
            retVal = FVID2_EINVALID_PARAMS;
        }
    }

    if (FVID2_SOK == retVal)
    {
        /*
         * valid instance, stream and channel
         */
        /* Get channel specific object in the required instance */
        chObj = &instObj->chObj[drvStreamId][chId];
        Fvid2Utils_memcpy(chStat, &chObj->stat, sizeof (*chStat));

        /*
         * Detect video
         */
        if (instObj->state == CAL_CAPT_STATE_RUNNING)
        {
            /* get current time */
            GT_assert(CalTrace, (NULL != instObj->getTimeStamp));
            curTime   = instObj->getTimeStamp(NULL);
            curTimeMs = (curTime / 1000U);
            if (curTimeMs < (chObj->stat.lastFrmTimeStamp + curTimeMs))
            {
                chStat->isVideoDetected = (uint32_t) TRUE;
            }
            else
            {
                chStat->isVideoDetected = (uint32_t) FALSE;
            }
        }
        else
        {
            /* If captrue is stopped return video is detected as the IOCTL
             * could be called very late after stopping */
            chStat->isVideoDetected = (uint32_t) TRUE;
        }
    }

    /* Post the instance semaphore */
    SemaphoreP_post(instObj->lockSem);

    return (retVal);
}

static int32_t calDrvCaptRegisterTimeStampIoctl(
    CalDrv_CaptInstObj        *instObj,
    const Fvid2_TimeStampParams *timeStampPrms)
{
    int32_t  retVal = FVID2_SOK;
    uintptr_t cookie;

    GT_assert(CalTrace, (NULL != instObj));

    /* Check for NULL pointers */
    if (NULL == timeStampPrms)
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid argument!!\r\n");
        retVal = FVID2_EBADARGS;
    }

    if (FVID2_SOK == retVal)
    {
        /* ISR might be still happening while we change pointers */
        cookie = HwiP_disable();

        if (NULL == timeStampPrms->timeStampFxn)
        {
            instObj->getTimeStamp = &calDrvCaptClockGetTicks;
        }
        else
        {
            instObj->getTimeStamp = timeStampPrms->timeStampFxn;
        }

        HwiP_restore(cookie);
    }

    return (retVal);
}

