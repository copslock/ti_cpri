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
 *  \file cal_drvPriv.c
 *
 *  \brief File containing the CAL capture driver functions.
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

static int32_t calDrvCaptCreateChQueues(const CalDrv_CaptInstObj *instObj,
                                      CalDrv_CaptChObj         *chObj);
static int32_t calDrvCaptDeleteChQueues(CalDrv_CaptChObj *chObj);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

CalDrv_CaptCommonObj gCalCaptCommonObj;
CalDrv_CaptInstObj   gCalCaptInstObj[CAL_CAPT_INST_ID_MAX];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  \brief CAL capture driver private init function.
 */
int32_t calDrvCaptPrivInit(uint32_t numInst, const CalDrv_CaptInitParams *initPrms)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t instCnt;
    CalDrv_CaptInstObj   *instObj;
    CalDrv_CaptCommonObj *pObj;
    SemaphoreP_Params     semPrms;

    /* Check for errors */
    GT_assert(CalTrace, (numInst <= CAL_CAPT_INST_ID_MAX));
    GT_assert(CalTrace, (NULL != initPrms));

    /* Init common object */

    pObj = &gCalCaptCommonObj;
    Fvid2Utils_memset(pObj, 0, sizeof (CalDrv_CaptCommonObj));
    if (numInst > CAL_CAPT_INST_ID_MAX)
    {
        /* numInst exceeds the global var used to store the instObj */
        GT_0trace(CalTrace, GT_ERR,
                  "numInst exceeds the global instObj array size!!\r\n");
        retVal = FVID2_EALLOC;
    }
    else
    {
        /* Initialize instance object members */
        pObj->instObj = &gCalCaptInstObj[0];
        Fvid2Utils_memset(pObj->instObj, 0, sizeof (gCalCaptInstObj));
    }

    if (FVID2_SOK == retVal)
    {
        pObj->numInst = numInst;
        instObj       = pObj->instObj;
        for (instCnt = 0U; instCnt < numInst; instCnt++)
        {
            /* Core should be present */
            GT_assert(CalTrace, (NULL != initPrms->coreOps));

            /* Copy the information */
            instObj->drvInstId   = initPrms->drvInstId;
            instObj->coreInstObj = initPrms->coreInstObj;
            instObj->coreOps     = initPrms->coreOps;

            /* Allocate instance semaphore */
            SemaphoreP_Params_init(&semPrms);
            semPrms.mode = SemaphoreP_Mode_BINARY;
            instObj->lockSem = SemaphoreP_create(1U, &semPrms);
            if (NULL == instObj->lockSem)
            {
                GT_0trace(
                    CalTrace, GT_ERR,
                    "Instance semaphore create failed!!\r\n");
                retVal = FVID2_EALLOC;
                break;
            }

            instObj->state        = CAL_CAPT_STATE_IDLE;
            instObj->getTimeStamp = &calDrvCaptClockGetTicks;

            initPrms++;
            instObj++;
        }
    }

    if (FVID2_SOK != retVal)
    {
        /* Uninitialize the internal objects if error occurs */
        calDrvCaptPrivDeInit();
    }

    return (retVal);
}

/**
 *  \brief CAL capture driver private deinit function.
 */
int32_t calDrvCaptPrivDeInit(void)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t instCnt;
    CalDrv_CaptInstObj   *instObj;
    CalDrv_CaptCommonObj *pObj;

    pObj = &gCalCaptCommonObj;
    if (NULL != pObj->instObj)
    {
        instObj = pObj->instObj;
        for (instCnt = 0U; instCnt < pObj->numInst; instCnt++)
        {
            if (instObj->state != CAL_CAPT_STATE_IDLE)
            {
                GT_0trace(
                    CalTrace, GT_ERR,
                    "Can't deinit when an instance is active\r\n");
                retVal = FVID2_EFAIL;
                break;
            }

            /* Delete the instance semaphore */
            if (NULL != instObj->lockSem)
            {
                SemaphoreP_delete(instObj->lockSem);
                instObj->lockSem = NULL;
            }
            Fvid2Utils_memset(pObj->instObj, 0, sizeof (CalDrv_CaptInstObj));

            instObj++;
        }
        if (retVal == FVID2_SOK)
        {
            pObj->instObj = NULL;
            pObj->numInst = 0U;
        }
    }

    return (retVal);
}

/**
 *  calDrvCaptGetInstObj
 *  Returns the instance object pointer for the instance id.
 */
CalDrv_CaptInstObj *calDrvCaptGetInstObj(uint32_t instId)
{
    uint32_t instCnt;
    CalDrv_CaptInstObj *instObj = NULL;
    CalDrv_CaptCommonObj *pObj;

    /* Find out the instance to which this channel belongs to */
    pObj = &gCalCaptCommonObj;
    GT_assert(CalTrace, (NULL != pObj));
    GT_assert(CalTrace, (NULL != pObj->instObj));
    for (instCnt = 0U; instCnt < pObj->numInst; instCnt++)
    {
        if (pObj->instObj[instCnt].drvInstId == instId)
        {
            instObj = &pObj->instObj[instCnt];
            break;
        }
    }

    return (instObj);
}

/**
 *  calDrvCaptCheckParams
 *  Checks for valid create parameters.
 */
int32_t calDrvCaptCheckParams(const CalDrv_CaptInstObj   *instObj,
                            const Cal_CaptCreateParams *createPrms)
{
    int32_t retVal = FVID2_SOK;

    /* NULL pointer check */
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != createPrms));

    if (createPrms->chInQueueLength <= 0U)
    {
        GT_1trace(
            CalTrace, GT_ERR,
            "Invalid in queue length(%d)\r\n",
            createPrms->chInQueueLength);
        retVal = FVID2_EINVALID_PARAMS;
    }

    if ((createPrms->numCh > CAL_CAPT_CH_PER_PORT_MAX) ||
        (createPrms->numCh <= 0U))
    {
        GT_2trace(
            CalTrace, GT_ERR,
            "Invalid number of channels(%d) - Supported max channels %d\r\n",
            createPrms->numCh,
            CAL_CAPT_CH_PER_PORT_MAX);
        retVal = FVID2_EINVALID_PARAMS;
    }

    if ((createPrms->numStream > CAL_CAPT_MAX_STREAMS) ||
        (createPrms->numStream <= 0U))
    {
        GT_2trace(
            CalTrace, GT_ERR,
            "Invalid number of streams(%d) - Supported max streams %d\r\n",
            createPrms->numStream,
            CAL_CAPT_MAX_STREAMS);
        retVal = FVID2_EINVALID_PARAMS;
    }

    if (createPrms->bufCaptMode >= CAL_CAPT_BCM_MAX)
    {
        GT_0trace(CalTrace, GT_ERR, "Invalid buffer capture mode\r\n");
        retVal = FVID2_EINVALID_PARAMS;
    }

    return (retVal);
}

int32_t calDrvCaptCreateChObj(CalDrv_CaptInstObj *instObj)
{
    int32_t  retVal = FVID2_SOK, tempRetVal;
    uint32_t streamId, chId;
    CalDrv_CaptChObj     *chObj;
    CalDrv_CaptCommonObj *pObj;

    pObj = &gCalCaptCommonObj;
    GT_assert(CalTrace, (NULL != pObj));
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace,
              (instObj->createPrms.numStream <= CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace,
              (instObj->createPrms.numCh <= CAL_CAPT_CH_PER_PORT_MAX));

    /* For every stream and every channel */
    for (streamId = 0U; streamId < instObj->createPrms.numStream; streamId++)
    {
        for (chId = 0U; chId < instObj->createPrms.numCh; chId++)
        {
            /* Get channel object */
            chObj = &instObj->chObj[streamId][chId];

            /* Make driver channel number from instance ID, stream ID,
             * thus given driver channel number we can know which
             * instance, stream, channel it belongs to chId */
            chObj->lChNum =
                Cal_captMakeChNum(instObj->drvInstId, streamId, chId);
            chObj->chNum = instObj->createPrms.chNumMap[streamId][chId];

            /* Make user channel number to driver channel number mapping */
            pObj->fvidChNumToLogChNumMap[chObj->chNum] = chObj->lChNum;

            /* Init other variables */
            chObj->instObj  = instObj;
            chObj->streamId = streamId;
            chObj->chIdx    = chId;
            chObj->isStreamByPassed = (uint32_t) FALSE;

            Fvid2Utils_memset(&chObj->stat, 0, sizeof (chObj->stat));

            /* Create channel queues */
            retVal = calDrvCaptCreateChQueues(instObj, chObj);
            if (FVID2_SOK != retVal)
            {
                break;
            }
        }
    }

    /* Deallocate if error occurs */
    if (FVID2_SOK != retVal)
    {
        tempRetVal = calDrvCaptDeleteChObj(instObj);
        GT_assert(CalTrace, (FVID2_SOK == tempRetVal));
    }

    return (retVal);
}

int32_t calDrvCaptDeleteChObj(CalDrv_CaptInstObj *instObj)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t streamId, chId;
    CalDrv_CaptChObj *chObj;

    GT_assert(CalTrace,
              (instObj->createPrms.numStream <= CAL_CAPT_MAX_STREAMS));
    GT_assert(CalTrace,
              (instObj->createPrms.numCh <= CAL_CAPT_CH_PER_PORT_MAX));

    /* For every stream and every channel */
    for (streamId = 0U; streamId < instObj->createPrms.numStream; streamId++)
    {
        for (chId = 0; chId < instObj->createPrms.numCh; chId++)
        {
            chObj  = &instObj->chObj[streamId][chId];
            retVal = calDrvCaptDeleteChQueues(chObj);
            if (FVID2_SOK != retVal)
            {
                break;
            }
        }

        /* Break if error */
        if (FVID2_SOK != retVal)
        {
            break;
        }
    }

    return (retVal);
}

/**
 *  \brief A simple wrapper function for DSP/BIOS clock get ticks.
 */
uint64_t calDrvCaptClockGetTicks(void *args)
{
    //TODO: Fix once OSAL implementation is done
    //return BspOsal_getCurTimeInUsec();
    return (0U);
}

static int32_t calDrvCaptCreateChQueues(const CalDrv_CaptInstObj *instObj,
                                      CalDrv_CaptChObj         *chObj)
{
    int32_t  retVal = FVID2_SOK, tempRetVal;
    uint32_t qCnt;
    CalDrv_CaptQueObj    *qObj;
    CalDrv_CaptBufManObj *bmObj;

    /* NULL pointer check */
    GT_assert(CalTrace, (NULL != instObj));
    GT_assert(CalTrace, (NULL != chObj));
    bmObj = &chObj->bmObj;

    bmObj->freeQ         = NULL;
    bmObj->reqQ          = NULL;
    bmObj->curQ          = NULL;
    bmObj->isProgressive = (uint32_t) TRUE;
    bmObj->bufFmt        = FVID2_BUF_FMT_FRAME;
    bmObj->fieldMerged   = (uint32_t) TRUE;
    bmObj->curFid        = 0U;
    bmObj->expectedFid   = 0U;

    /* Create Queues */
    retVal = Fvid2Utils_constructQ(&bmObj->freeLlObj);
    GT_assert(CalTrace, (retVal == FVID2_SOK));
    bmObj->freeQ = &bmObj->freeLlObj;

    retVal = Fvid2Utils_constructQ(&bmObj->reqLlObj);
    GT_assert(CalTrace, (retVal == FVID2_SOK));
    bmObj->reqQ = &bmObj->reqLlObj;

    retVal = Fvid2Utils_constructQ(&bmObj->curLlObj);
    GT_assert(CalTrace, (retVal == FVID2_SOK));
    bmObj->curQ = &bmObj->curLlObj;

    if (FVID2_SOK == retVal)
    {
        if (instObj->createPrms.chInQueueLength > CAL_CAPT_QUEUE_LEN_PER_CH)
        {
            GT_2trace(
                CalTrace, GT_ERR,
                "Cant create %d capture queue objects!!. Maximum supported capture queue objects are %d\r\n",
                instObj->createPrms.chInQueueLength,
                CAL_CAPT_QUEUE_LEN_PER_CH);
            retVal = FVID2_EALLOC;
        }
    }
    if (FVID2_SOK == retVal)
    {
        /* Initialize and queue the allocate queue object to free Q */
        for (qCnt = 0U; qCnt < instObj->createPrms.chInQueueLength; qCnt++)
        {
            qObj        = &bmObj->captQObj[qCnt];
            qObj->chObj = chObj;
            qObj->frm   = NULL;
            Fvid2Utils_memset(&qObj->coreFrm, 0, sizeof (qObj->coreFrm));
            qObj->coreFrm.streamId = chObj->streamId;
            qObj->coreFrm.chId     = chObj->chIdx;
            qObj->coreFrm.dropFrm  = (uint32_t) FALSE;
            qObj->coreFrm.rtParams = NULL;
            qObj->coreFrm.drvData  = qObj;
            qObj->creditCnt = 0U;

            Fvid2Utils_queue(bmObj->freeQ, &qObj->qElem, qObj);
        }
    }

    /* Deallocate if error occurs */
    if (FVID2_SOK != retVal)
    {
        tempRetVal = calDrvCaptDeleteChQueues(chObj);
        GT_assert(CalTrace, (FVID2_SOK == tempRetVal));
    }

    return (retVal);
}

static int32_t calDrvCaptDeleteChQueues(CalDrv_CaptChObj *chObj)
{
    int32_t retVal = FVID2_SOK;
    CalDrv_CaptQueObj    *qObj;
    CalDrv_CaptBufManObj *bmObj;

    /* NULL pointer check */
    GT_assert(CalTrace, (NULL != chObj));
    bmObj = &chObj->bmObj;

    if (NULL != bmObj->freeQ)
    {
        /* Free-up all the queued free queue objects */
        while (1U)
        {
            qObj = (CalDrv_CaptQueObj *) Fvid2Utils_dequeue(bmObj->freeQ);
            if (NULL == qObj)
            {
                /* No more in queue */
                break;
            }
        }

        /* Delete the free Q */
        Fvid2Utils_destructQ(bmObj->freeQ);
        bmObj->freeQ = NULL;
    }

    if (NULL != bmObj->reqQ)
    {
        /* Delete the request Q */
        Fvid2Utils_destructQ(bmObj->reqQ);
        bmObj->reqQ = NULL;
    }

    if (NULL != bmObj->curQ)
    {
        /* Delete the free Q */
        Fvid2Utils_destructQ(bmObj->curQ);
        bmObj->curQ = NULL;
    }

    return (retVal);
}

