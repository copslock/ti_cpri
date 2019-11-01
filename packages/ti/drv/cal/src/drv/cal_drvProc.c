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
 *  \file cal_drvProc.c
 *
 *  \brief File containing the CAL capture driver process related functions.
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

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief Extern function to be implemented by driver to provide a new frame
 *  buffers from application to the core.
 */
Cal_CoreFrame *calDrvCaptCoreReqFrameCb(void  *drvData,
                                        uint32_t streamId,
                                        uint32_t chId)
{
    uintptr_t cookie;
    CalDrv_CaptChObj   *chObj;
    CalDrv_CaptInstObj *instObj;
    CalDrv_CaptQueObj  *qObj;
    Cal_CoreFrame      *coreFrm = NULL;

    instObj = (CalDrv_CaptInstObj *) drvData;
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (streamId < CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace, (chId < instObj->createPrms.numCh));

    /* Get channel specific object in the required instance */
    chObj = &instObj->chObj[streamId][chId];

    cookie = HwiP_disable();

    qObj = (CalDrv_CaptQueObj *) Fvid2Utils_dequeue(chObj->bmObj.reqQ);
    if(NULL != qObj)
    {
        /*
         * Buffer available in request queue
         */
        /* Copy the address and increment credit */
        qObj->coreFrm.addr[0] = qObj->frm->addr[0];
        qObj->coreFrm.addr[1] = qObj->frm->addr[1];
        qObj->coreFrm.addr[2] = qObj->frm->addr[2];
        qObj->coreFrm.addr[3] = qObj->frm->addr[3];
        qObj->coreFrm.addr[4] = qObj->frm->addr[4];
        qObj->coreFrm.addr[5] = qObj->frm->addr[5];
        qObj->coreFrm.rtParams   = qObj->frm->perFrameCfg;
        qObj->coreFrm.dropFrm = (uint32_t) FALSE;
        /* If app passes NULL pointer, then drop the frame */
        if (0x0U == qObj->coreFrm.addr[0])
        {
            qObj->coreFrm.dropFrm = (uint32_t) TRUE;
        }

        qObj->creditCnt++;
        coreFrm = &qObj->coreFrm;
        Fvid2Utils_queue(chObj->bmObj.curQ, &qObj->qElem, qObj);
    }
    else
    {
        /*
         * No more buffers available in request queue - decide what to do
         * based on buffer capture mode
         */
        if(CAL_CAPT_BCM_LAST_FRM_REPEAT == instObj->createPrms.bufCaptMode)
        {
            /* Repeat the last frame queued to the core */
            qObj = (CalDrv_CaptQueObj *)
                   Fvid2Utils_peakTail(chObj->bmObj.curQ);
            /* This can't return NULL unless there is a bug in the buffer
             * management */
            GT_assert(CalTrace, (NULL != qObj));
            qObj->creditCnt++;
            coreFrm = &qObj->coreFrm;
        }
        else if(CAL_CAPT_BCM_CIRCULAR_FRM_REPEAT == 
                instObj->createPrms.bufCaptMode)
        {
            /*
             * Repeat the oldest frame queued to the core and make it at
             * the last element in the current queue
             */
            qObj = (CalDrv_CaptQueObj *)
                   Fvid2Utils_dequeue(chObj->bmObj.curQ);
            /* This can't return NULL unless there is a bug in the buffer
             * management */
            GT_assert(CalTrace, (NULL != qObj));
            qObj->creditCnt++;
            coreFrm = &qObj->coreFrm;

            /* Queue back to the start */
            Fvid2Utils_queue(chObj->bmObj.curQ, &qObj->qElem, qObj);
        }
        else
        {
            GT_assert(CalTrace, FALSE);
        }
    }

    GT_1trace(CalTrace, GT_DEBUG,
              "CaptDrv: Queued buffer 0x%0.8x to core\r\n", coreFrm->addr[0]);

    HwiP_restore(cookie);

    GT_assert(CalTrace, (NULL != coreFrm));
    return (coreFrm);
}

/**
 *  \brief Extern callback function to be implemented by driver to free
 *  up buffers to the application specified by the core.
 */
int32_t calDrvCaptCoreFrameDoneCb(void *drvData, const Cal_CoreFrame *coreFrm)
{
    uintptr_t cookie;
    uint32_t curQCnt, reqQCnt;
    uint64_t curTimeStamp;
    Bool   frmQueuedToDoneQ = FALSE;
    CalDrv_CaptChObj   *chObj;
    CalDrv_CaptInstObj *instObj;
    CalDrv_CaptQueObj  *qObj, *qObjTemp;
    Cal_CaptRtParams   *rtPrms;
    uint32_t              isWrDescValid;

    instObj = (CalDrv_CaptInstObj *) drvData;
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != coreFrm));
    GT_assert(CalTrace, (coreFrm->streamId < CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace, (coreFrm->chId < instObj->createPrms.numCh));
    GT_assert(CalTrace, (coreFrm->fid < FVID2_MAX_FIELDS));

    /* Get channel specific object in the required instance */
    chObj = &instObj->chObj[coreFrm->streamId][coreFrm->chId];

    cookie = HwiP_disable();

    GT_assert(CalTrace, (NULL != instObj->getTimeStamp));
    curTimeStamp = instObj->getTimeStamp(NULL);
    GT_1trace(CalTrace, GT_DEBUG,
              "CaptDrv: Buffer 0x%0.8x capture done\r\n", coreFrm->addr[0]);

    /* Update channel statistics */
    chObj->stat.captFrmCount++;
    chObj->stat.fldCount[coreFrm->fid]++;
    if((chObj->bmObj.isProgressive == FALSE) &&
       (chObj->stat.lastFid == coreFrm->fid))
    {
        chObj->stat.fidRepeatCount++;
    }
    chObj->stat.lastFid = coreFrm->fid;
    chObj->stat.lastFrmTimeStamp = (uint32_t) (curTimeStamp / 1000U);
    isWrDescValid = (uint32_t) TRUE;
    if ((uint32_t) TRUE == isWrDescValid)
    {
        if(coreFrm->width > chObj->stat.maxRecvFrmWidth)
        {
            chObj->stat.maxRecvFrmWidth = coreFrm->width;
        }
        if(coreFrm->height > chObj->stat.maxRecvFrmHeight)
        {
            chObj->stat.maxRecvFrmHeight = coreFrm->height;
        }
        /* Init min values for the first frame capture */
        if(chObj->stat.captFrmCount == 1U)
        {
            chObj->stat.minRecvFrmWidth  = coreFrm->width;
            chObj->stat.minRecvFrmHeight = coreFrm->height;
        }
        else
        {
            if(coreFrm->width < chObj->stat.minRecvFrmWidth)
            {
                chObj->stat.minRecvFrmWidth = coreFrm->width;
            }
            if(coreFrm->height < chObj->stat.minRecvFrmHeight)
            {
                chObj->stat.minRecvFrmHeight = coreFrm->height;
            }
        }
        chObj->stat.lastFrmWidth     = coreFrm->width;
        chObj->stat.lastFrmHeight    = coreFrm->height;
    }

    /* Get reference of the queue head */
    qObj = (CalDrv_CaptQueObj *) coreFrm->drvData;
    GT_assert(CalTrace, (NULL != qObj));

    /* Decrement credit count as frame capture is complete - credit can't
     * be zero */
    GT_assert(CalTrace, (qObj->creditCnt > 0U));
    qObj->creditCnt--;
    if(qObj->creditCnt > 0U)
    {
        chObj->stat.repeatFrmCount++;
    }

    GT_assert(CalTrace, (NULL != qObj->frm));
    qObj->frm->status = coreFrm->status;

    if(CAL_CAPT_BCM_LAST_FRM_REPEAT == instObj->createPrms.bufCaptMode)
    {
        /* Get the current queue counts */
        curQCnt = Fvid2Utils_getNumQElem(chObj->bmObj.curQ);
        reqQCnt = Fvid2Utils_getNumQElem(chObj->bmObj.reqQ);

        if(0U == qObj->creditCnt)
        {
            qObjTemp = (CalDrv_CaptQueObj *)
                       Fvid2Utils_dequeue(chObj->bmObj.curQ);
            /* Head node and core frame should match */
            GT_assert(CalTrace, (qObj == qObjTemp));

            /* In last frame repeat mode, we could return all the frames to
             * the application if credit becomes 0 and there are some more
             * request in the current or request queue. Current queue is
             * checked for 1 element as the current frame is still present
             * in the queue. */
            if((curQCnt > 1U) || (reqQCnt > 0U))
            {
                GT_1trace(CalTrace, GT_DEBUG,
                          "CaptDrv: Dequeued buffer 0x%0.8x\r\n",
                          coreFrm->addr[0]);

                qObj->coreFrm.addr[0] = 0x0U;
                qObj->coreFrm.addr[1] = 0x0U;
                qObj->coreFrm.addr[2] = 0x0U;
                qObj->coreFrm.dropFrm    = (uint32_t) FALSE;

                /* Updated runtime parameters */
                GT_assert(CalTrace, (NULL != qObj->frm));
                qObj->frm->fid = qObj->coreFrm.fid;
                if(NULL != qObj->frm->perFrameCfg)
                {
                    rtPrms = (Cal_CaptRtParams *) qObj->frm->perFrameCfg;
                    rtPrms->capturedOutWidth  = qObj->coreFrm.width;
                    rtPrms->capturedOutHeight = qObj->coreFrm.height;
                }

                /* Return the frame to done queue */
                GT_assert(CalTrace, (NULL != qObj->frm));
                qObj->frm->timeStamp64 = curTimeStamp;
                Fvid2Utils_queue(
                    instObj->doneQ[coreFrm->streamId],
                    &qObj->qElem,
                    qObj);
                frmQueuedToDoneQ = (Bool) TRUE;
            }
            /* In last frame repeat mode, if credit becomes 0 and there are
             * no more request in the current and request queues, take this
             * request and queue it back to request queue so that when
             * the core asks for next buffer, we repeat the frame
             * automatically. This is needed because the user could
             * queue a request in between and this frame will end-up
             * in the current queue!!
             * Also increment the repeat frame counter here. */
            else if((curQCnt == 1U) && (reqQCnt == 0U))
            {
                chObj->stat.repeatFrmCount++;
                Fvid2Utils_queue(chObj->bmObj.reqQ, &qObj->qElem, qObj);
            }
            else
            {
                /* This can't happen as curQCnt can't be zero!! */
                GT_assert(CalTrace, FALSE);
            }
        }
    }
    else if(CAL_CAPT_BCM_CIRCULAR_FRM_REPEAT ==
            instObj->createPrms.bufCaptMode)
    {
        /* Do nothing, just increment repeat count */
        if(0U == qObj->creditCnt)
        {
            chObj->stat.repeatFrmCount++;
        }
    }
    else
    {
        GT_assert(CalTrace, FALSE);
    }

    HwiP_restore(cookie);

    if(NULL != instObj->fdmCbPrms.fdmCbFxn)
    {
        /* Give callback to application if frame is put in done queue */
        /* TODO - handle multi-channel case!! */
        if((Bool) TRUE == frmQueuedToDoneQ)
        {
            instObj->fdmCbPrms.fdmCbFxn(instObj->fdmCbPrms.fdmData);
        }
    }

    return (FVID2_SOK);
}

