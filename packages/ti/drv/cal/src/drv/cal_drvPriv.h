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
 *  \file cal_drvPriv.h
 *
 *  \brief CAL capture driver private header file.
 *
 */

#ifndef CAL_DRV_PRIV_H_
#define CAL_DRV_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>

#include <ti/csl/soc.h>

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/core/cal_core.h>
#include <ti/drv/cal/src/drv/cal_drvInternal.h>
#include <ti/drv/cal/src/core/cal_evtmgr.h>
#include <ti/drv/cal/src/core/cal_resrcMgr.h>
#include <ti/drv/cal/src/core/cal_common.h>
#include <ti/drv/cal/src/core/cal_corecapture.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/*
 * State transitions are explained below
 *
 * IDLE - initial or not open state
 *  |
 * CREATED - state after calling Fvid2_create
 *  |
 * DO_START - temporary state when Fvid2_start() is called
 *  |         this means driver has requested CLM to start the instance
 *  |
 * RUNNING  - this is the state after Fvid2_start() completes, this means
 *  |         CLM started capture and now it is running, it remains in this
 *  |         state
 *  |         until Fvid2_stop()  is called
 *  |
 * DO_STOP  - temporary state when Fvid2_stop is called()
 *  |         this means driver has request CLM to stop the instance
 *  |
 * STOPPED  - this is the state after Fvid2_stop() completes, this means
 *  |         CLM has stopped this instance
 *  |
 * IDLE     - this is the state after Fvid2_delete() completes
 */
/** \brief Driver is not open and is idle. */
#define CAL_CAPT_STATE_IDLE             (0)
/** \brief Driver is created. */
#define CAL_CAPT_STATE_CREATED          (1)
/** \brief Driver is requesting a start. */
#define CAL_CAPT_STATE_DO_START         (2)
/** \brief Driver is running. */
#define CAL_CAPT_STATE_RUNNING          (3)
/** \brief Driver is requesting a stop. */
#define CAL_CAPT_STATE_DO_STOP          (4)
/** \brief Driver is stopped. */
#define CAL_CAPT_STATE_STOPPED          (5)

#define CAL_CAPT_CH_MAP_IDX_MAX         (CAL_CAPT_MAX_STREAMS *   \
                                         CAL_CAPT_CH_PER_PORT_MAX * \
                                         CAL_CAPT_INST_ID_MAX)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief Forward declaration for instance object. */
typedef struct CalDrv_CaptInstObj_t CalDrv_CaptInstObj;

/** \brief Forward declaration for channel object. */
typedef struct CalDrv_CaptChObj_t CalDrv_CaptChObj;

/**
 *  struct CalDrv_CaptQueObj
 *  \brief Structure defining the queue object used in queue/dequeue operation.
 *  Instead of creating frame objects, this is used so that any other
 *  information could be queued/dequeued along with the frame.
 *  Already qElem is added to avoid dynamic allocation of Queue nodes.
 */
typedef struct
{
    Fvid2Utils_QElem       qElem;
    /**< Fvid2 utils queue element used in node addition. */
    CalDrv_CaptChObj    *chObj;
    /**< Reference to the channel object for this queue object. */
    Fvid2_Frame         *frm;
    /**< FVID2 frame to store the incoming/outgoing IO packets. */
    Cal_CoreFrame        coreFrm;
    /**< Frame information as needed by the core. This is used as the core
     *   requires different frame parameters compared to #Fvid2_Frame. */

    uint32_t               creditCnt;
    /**< Number of times the frame is queued to the core for capture.
     *   This counter is used to determine the number of times the frame is
     *   repeated.
     *
     *   Initial value of the counter is zero.
     *   For every queue to the core the counter is incremented.
     *   For every dequeue from the core the counter is decremented.
     *   When the counter becomes 0 in dequeue from core, the frame is ready
     *   to be given to the application depending on the buffer mode
     *   selected. */
} CalDrv_CaptQueObj;

/**
 *  struct CalDrv_CaptBufManObj
 *  \brief Structure to store the buffer management functionality variables.
 */
typedef struct
{
    uint32_t               isProgressive;
    /**< Flag to indicate whether the capture is progressive or interlaced. */
    uint32_t               bufFmt;
    /**< Frame capture or field capture for interlaced scan format. */
    uint32_t               fieldMerged;
    /**< This variable is used to specify whether two fields are merged for
     *   frame capture or they in separate planes. */

    uint32_t               curFid;
    /**< Indicates current field ID. Used in interlaced capture. */
    uint32_t               expectedFid;
    /**< Indicates the next set field expected. This is used to check if
     *   capture is occurring at proper sequence without any field misses. */

    /*
     * Initially all the queue elements will be in freeQ.
     *
     * For every application queue operation,   freeQ -> reqQ
     * For every request submitted to core,     reqQ  -> curQ
     * For every request completion from core,  curQ  -> doneQ (per Stream)
     * For every application dequeue operation, doneQ -> freeQ
     */
    Fvid2UtilsLinkListObj *freeQ;
    /**< Queue for queueing all the free queue objects. */
    Fvid2UtilsLinkListObj *reqQ;
    /**< Queue object to put the input requests. */
    Fvid2UtilsLinkListObj *curQ;
    /**< Buffers that are queued to hardware/core but not yet fully captured. */
    Fvid2UtilsLinkListObj  freeLlObj;
    /**< Linked List object for freeQ. */
    Fvid2UtilsLinkListObj  reqLlObj;
    /**< Linked List object for reqQ. */
    Fvid2UtilsLinkListObj  curLlObj;
    /**< Linked List object for curQ. */
    CalDrv_CaptQueObj    captQObj[CAL_CAPT_QUEUE_LEN_PER_CH];
    /**< Capture queue objects. */
} CalDrv_CaptBufManObj;

/*
 * Channel specific information
 */
struct CalDrv_CaptChObj_t
{
    CalDrv_CaptInstObj  *instObj;
    /**< Capture instance objects pointer to which this channel belongs. */

    uint32_t               chNum;
    /**< User provided channel number at create time. */
    uint32_t               lChNum;
    /**< Driver logical number - is unique across all capture handles. */

    uint32_t               streamId;
    /**< Stream ID index to which this channel belongs to. Used for
     *   dereferencing. */
    uint32_t               chIdx;
    /**< Channel index to which this channel belongs to. Used for
     *   dereferencing. */

    CalDrv_CaptBufManObj bmObj;
    /**< Channel buffer management object. */
    Cal_CaptChStatus     stat;
    /**< Channel statistics object. */

    uint32_t               isStreamByPassed;
    /**< Flag that indicates if reception on this stream is optional. */
};

/**
 *  struct CalDrv_CaptInstObj
 *  \brief Per instance information.
 */
struct CalDrv_CaptInstObj_t
{
    uint32_t                 drvInstId;
    /**< Instance ID. */
    Cal_CoreInst           coreInstObj;
    /**< Core instance object used in core open. */
    const Cal_CoreOps     *coreOps;
    /**< Core function pointers. */
    Cal_CoreProperty       coreProperty;
    /**< Core properties. */

    Cal_CaptCreateParams   createPrms;
    /**< Create parameters. */
    Fvid2_DrvCbParams      fdmCbPrms;
    /**< FVID2 driver manager callback function parameters. */

    CalDrv_CaptChObj       chObj[CAL_CAPT_MAX_STREAMS]
    [CAL_CAPT_CH_PER_PORT_MAX];
    /**< Channel object's for every channel and stream. */

    uint32_t                 state;
    /**< Instance state. */

    Fvid2UtilsLinkListObj   *doneQ[CAL_CAPT_MAX_STREAMS];
    /**< Queue object to put the processed output requests. This is kept
     *   common for all channels of a stream because frames can be given
     *   back to application in dequeue call without looping over each
     *   channel done queue. */

    Fvid2UtilsLinkListObj    doneLlObj[CAL_CAPT_MAX_STREAMS];
    /**< Linked List object for doneQ for all channels of each stream. */

    uint32_t                 queueCount;
    /**< Counter to keep track of how many requests are queued to the driver. */
    uint32_t                 dequeueCount;
    /**< Counter to keep track of how many requests are dequeued from the
     *  driver. */

    uint32_t                 overflowCount;
    /**< Counter to keep track of overflow error per instance. */
    uint32_t                 asynOverflowCount;
    /**< Counter to keep track of ASYNC FIFO overflow error per instance. */
    uint32_t                 actProtViolationCount;
    /**< Counter to keep track of active video protocol violation error per
     *   instance. */
    uint32_t                 ancProtViolationCount;
    /**< Counter to keep track of ancillary video protocol violation error per
     *   instance. */

    Fvid2_TimeStampFxn      getTimeStamp;
    /**< Get timestamp function. */

    Cal_CoreHandle          coreHandle;
    /**< Core handle. */

    SemaphoreP_Handle      lockSem;
    /**< Semaphore to protect the open/close calls and other memory
     *   allocation per instance. */
    Cal_FrameEventNotifyCfg_t frmEvtNotifyPrms;
    /**< Parameter that were supplied while configuring frame event
            notification */
};

/*
 * struct CalDrv_CaptCommonObj
 * \brief Capture driver global/common driver object.
 */
typedef struct
{
    Fvid2_DrvOps        fvidDrvOps;
    /**< FVID2 driver ops. */
    uint32_t              isRegistered;
    /**< FVID2 registeration complete flag. */

    uint32_t              numInst;
    /**< Number of valid instance. */
    CalDrv_CaptInstObj *instObj;
    /**< Capture instance objects pointer. */

    uint32_t              fvidChNumToLogChNumMap[CAL_CAPT_CH_MAP_IDX_MAX];
    /**< User channel number to driver channel number map. */
} CalDrv_CaptCommonObj;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/*
 * Global variables
 */
extern CalDrv_CaptCommonObj gCalCaptCommonObj;

/*
 * Process related functions
 */
void calDrvCaptVemIsr(const uint32_t *event, uint32_t numEvents, void *arg);
void calDrvCaptVemOverflowIsr(const uint32_t *event, uint32_t numEvents, void *arg);

void calDrvCaptVemSubFrmIsr(const uint32_t *event, uint32_t numEvents, void *arg);
Cal_CoreFrame *calDrvCaptCoreReqFrameCb(void  *drvData,
                                        uint32_t streamId,
                                        uint32_t chId);
int32_t calDrvCaptCoreFrameDoneCb(void *drvData, const Cal_CoreFrame *coreFrm);

/*
 * Private functions
 */
int32_t calDrvCaptPrivInit(uint32_t numInst, const CalDrv_CaptInitParams *initPrms);
int32_t calDrvCaptPrivDeInit(void);

CalDrv_CaptInstObj *calDrvCaptGetInstObj(uint32_t instId);
int32_t calDrvCaptCheckParams(const CalDrv_CaptInstObj   *instObj,
                            const Cal_CaptCreateParams *createPrms);

int32_t calDrvCaptCreateChObj(CalDrv_CaptInstObj *instObj);
int32_t calDrvCaptDeleteChObj(CalDrv_CaptInstObj *instObj);

uint64_t calDrvCaptClockGetTicks(void *args);

/*
 * Core functions
 */
int32_t calDrvCaptGetPropCore(CalDrv_CaptInstObj *instObj);
int32_t calDrvCaptOpenCore(CalDrv_CaptInstObj *instObj);
int32_t calDrvCaptCloseCore(CalDrv_CaptInstObj *instObj);
int32_t calDrvCaptStartCore(CalDrv_CaptInstObj *instObj);
int32_t calDrvCaptStopCore(CalDrv_CaptInstObj *instObj);

int32_t calDrvCaptSetCoreParams(CalDrv_CaptInstObj   *instObj,
                                 const Cal_Cfg_t *pPrms);

int32_t calDrvCaptSetErrorParams(CalDrv_CaptInstObj   *instObj,
                                    const Cal_ErrorCfg_t *pPrms);
int32_t calDrvCaptSetSubFrmParams(CalDrv_CaptInstObj   *instObj,
                                const Cal_FrameEventNotifyCfg_t *pPrms);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CAL_DRV_PRIV_H_ */
