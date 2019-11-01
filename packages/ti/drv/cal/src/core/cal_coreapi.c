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
 *  \file cal_coreapi.c
 *
 *  \brief This file implements core functions required by CAL. Primarily will
 *          be used by drivers.
 *
 *  TODO - The start / stop mechanism should be changed. Currently, we are
 *          relying on starting PPI for start / stop.
 *          In Start start PPI is not started already, start DMA write streams.
 *          In stop, stop just the DMA write. If the number of opens == 1
 *              then stop the PPI also. - Done
 *
 *          Handle error interrupt
 *
 *          When the last stream is stopped, ensure to turn OFF PPI
 *
 *          streamType - Supplied by apps, is not handled by APPs. This is
 *              required when meta data is to be received.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <stddef.h>

#include <ti/drv/cal/cal.h>

#include <ti/drv/cal/src/hal/cal_hal.h>
#include <ti/drv/cal/src/core/cal_core.h>
#include <ti/drv/cal/src/core/cal_evtmgr.h>
#include <ti/drv/cal/src/core/cal_resrcMgr.h>
#include <ti/drv/cal/src/core/cal_common.h>
#include <ti/drv/cal/src/core/cal_corecapture.h>

/* ========================================================================== */
/*                           Constants                                        */
/* ========================================================================== */

/** \brief All sub modules of CAL */
#define CAL_ALL_SUB_MODULE  (CAL_CAPT_INST_ID_SUB_PPI_ID_0          | \
                             CAL_CAPT_INST_ID_SUB_PPI_ID_1          | \
                             CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID    | \
                             CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID       | \
                             CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID       | \
                             CAL_CAPT_INST_ID_SUB_PIX_PACK_ID       | \
                             CAL_CAPT_INST_ID_SUB_BYS_OUT_ID        | \
                             CAL_CAPT_INST_ID_SUB_BYS_IN_ID         | \
                             CAL_CAPT_INST_ID_SUB_VPORT_ID          | \
                             CAL_CAPT_INST_ID_SUB_DMA_RD_ID         | \
                             CAL_CAPT_INST_ID_SUB_DMA_WR_ID         | \
                             CAL_CAPT_INST_ID_SUB_CPORT_ID)

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/** \brief Its possible to get 2 interrupts,
                1 when when DMA writes first byte of a given frame
                other when DMA writes last byte of a given frame

            By default we rely on start-of-frame interrupt to notify successful
            capture of previous frame.

            In cases where applications requires to be notified as soon a frame
            is captured, the end-of-frame notification can be enabled with
            CAL_CORE_CAPT_SET_SUB_FRM_PRMS

            This is a per instance flag, either CAL operates with dual
                interrupt or single interrupt */
#define CAL_USE_DUAL_INTERRUPT      ((uint32_t)FALSE)

/** \brief When defined, application will have follow below steps to start
            1. Init, Create & set config
            2. Prime with 1 buffer for each stream
            3. Start
                - Driver will request new "empty" frame when required.
            When un defined
            1. Init, Create & set config
            2. Start
                - Driver will request new "empty" frame when required.
                - Note no priming is required.
            */
/* #define CAL_SEPERATE_API_FOR_PRIME */

/** \brief When this macro is defined,
            The core frames that is currently programmed and frame under
            reception, will be returned on stop.
            when undefined, the caller will have to explicitly keep track of
            frames in the core. Core will not return the same, when stopped.
            */
/* #define CAL_FLUSH_ON_STOP */

/** \brief Maximum virtual channel, used to log virtual channel errors */
#define CAL_MAX_VIRTUAL_CHAN        (0x4U)
/** \brief Poistion at which BYS_IN overflow error is logged */
#define CAL_BYS_IN_POSITION         (0x4U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
/**< Forward Declaration */
typedef struct Cal_CoreCaptInstObj Cal_CoreCaptInstObj_t;

/** \brief Possible streams mode with in CAL. A given stream could be
            1. written out by CAL
            2. processed but not written
            3. processed by ISP (an entity outside CAL)*/
typedef enum Cal_CoreCaptStreamMode
{
    CAL_CORE_STREAM_MODE_NONE = 0x0,
    /**< The stream is not destined to any. Invalid */
    CAL_CORE_STREAM_MODE_CAL_WRITE = 0x1,
    /**< Is a valid stream within CAL, will be written by CAL to memory */
    CAL_CORE_STREAM_MODE_CAL = 0x2,
    /**< Is a valid stream with CAL, but CAL will not write it to memory */
    CAL_CORE_STREAM_MODE_ISP = 0x3,
    /**< Is a valid stream will be processed by ISP. No CAL operation required
            will be processed/written by ISP */
    CAL_CORE_STREAM_MODE_MAX = 0x4,
    /**< Max stream, end marker */
    CAL_CORE_STREAM_MODE_FORCE_INT = 0x7FFFFFFF
                                      /**< This will ensure enum is not packed,
                                          will always be contained in int */
} Cal_CoreCaptStreamMode_t;

/** \brief Error configuration */
typedef struct Cal_CoreCaptErrObj
{
    uint32_t                cmplxIoId;
    /**< Complex IO ID */
    Cal_CoreCaptInstObj_t *pInstObj;
    /**< Pointer back to instance object */
    Cal_ErrorCfg_t     errCfg;
    /**< Error configuration */
    void                    *emErrorHndl;
    /**< Event manager handle for CAL errors */
    uint32_t                fifoOverFlow[CAL_MAX_VIRTUAL_CHAN + 1U];
    /**< The PPI FIFO overflowed, +1 for BYS_IN port */
    uint32_t                eccCouldNotCorrect[CAL_MAX_VIRTUAL_CHAN + 1U];
    /**< There were more than 1 bit errors, could not be corrected */
    uint32_t                crcMisMatch[CAL_MAX_VIRTUAL_CHAN + 1U];
    /**< CRC mismatch occurred, +1 for BYS_IN port  */
    uint32_t                eccCorrected[CAL_MAX_VIRTUAL_CHAN + 1U];
    /**< 1 Bit ECC error was corrected, +1 for BYS_IN port  */
} Cal_CoreCaptErrObj_t;

/** \brief Capture Instance object */
struct Cal_CoreCaptInstObj
{
    uint32_t                    isInited;
    /**< Flag to specify if an instance is initialized */
    Cal_CaptInstId_t          instId;
    /**< Platform data */
    uint32_t                    useDmaEndIntr;
    /**< TRUE: Enables DMA Frame End Interrupt
     *   FALSE: No DMA Frame End Interrupt,
     *          all processing is done in DMA Frame Start interrupt */
    SemaphoreP_Handle           instLock;
    /**< Instance mutual exclusion */

    Cal_CoreCaptInitParams_t    initPrms;
    /**< Init Parameters for post initialization references. */

    Cal_HalHandle               calHalHandle;
    /**< CAL hal handle */
    Cal_HalCfg_t              calCfg;
    /**< Current configuration, as supplied by driver / application. */
    Cal_HalInstCfg_t          instCfg;
    /**< Current instance config */

    uint32_t                      isStarted;
    /**< Flag to indicate if capture on this instance has started */
    uint32_t                      numOpens;
    /**< Number of handles opened */
    Cal_CoreCaptErrObj_t      errObj[CAL_CAPT_MAX_CMPLXIO_INST];
    /**< Instance error handling configuration */
    uint32_t                    streamIdToVcMap[CAL_CAPT_MAX_STREAMS];
    /**< Maps stream ID to virtual channel. Value 0x4 is used to indicate
            data is sourced from BYS_IN */
    uint32_t                    streamIdToPpiMap[CAL_CAPT_MAX_STREAMS];
    /**< Stream ID and it's associated PPI/PHY ID */
    Cal_HalLineEventCfg_t  lineEventCfg;
    /**< Place holder for line event configurations */

} ;

/** \brief Capture CAL handle object */
typedef struct Cal_CoreCaptCalHndlObj
{
    uint32_t                     isAllocated;
    /**< Flag to keep track if the handle object is allocated or not */
    Cal_CoreCaptInstObj_t   *pInstObj;
    /**< Instance object */
    Cal_CoreOpenPrms           openPrms;
    /**< Core Open Parameters containing callbacks
     *   for requesting a new frame and for submitting completed frame
     *   to the upper layer
     *   It also has callback to mark completion of the frame */
    uint32_t                     numStreams;
    /**< Number of streams allocated */
    Cal_CaptBlocks_t      allocatedRes[CAL_CAPT_MAX_STREAMS];
    /**< Instances of resource allocated */

    void                      *emDmaStartHndl[CAL_CAPT_MAX_STREAMS];
    /**< Event manager handle for DMA start */
    void                      *emDmaCmpltHndl[CAL_CAPT_MAX_STREAMS];
    /**< Event manager handle for DMA completion */
    uint32_t                     dmaStartEvent[CAL_CAPT_MAX_STREAMS];
    /**< Events associated with this instance and streams that are active */
    uint32_t                     eventToStreamIdMap[CAL_CAPT_MAX_STREAMS];
    /**< Map between event to associated stream id */
    uint32_t                     dmaCmpltEvent[CAL_CAPT_MAX_STREAMS];
    /**< Events associated with this instance and streams that are active */

    uint32_t                     bitsPerPix[CAL_CAPT_MAX_STREAMS];
    /**< Actual bits per pixel required, computed from Fvid2_BitsPerPixel
     *      specified as part of setParams */

    uint32_t                     fvid2DataType;
    /**< FVID2 dataType computed / owned by the core
     *   Unused currently - TBD: use in getParams */
    uint32_t                     fvid2bpp;
    /**< FVID2 ccsFormat computed / owned by the core
     *   Unused currently - TBD: use in getParams */
    Cal_HalDmaVcCfg_t        calDmaVcCfg;
    /**< CAL configurations computed / owned by the core */

    Cal_HalBufferAddr_t      bufCfg;
    /**< Buffer to be used to update HAL with new buffers */
    volatile Cal_CoreFrame    *currBufs[CAL_CAPT_MAX_STREAMS][2U];
    /**< Pointer to buffer that is currently owned by this core */
    volatile uint32_t            curr[CAL_CAPT_MAX_STREAMS];
    /**< Index indicating the buffer that's is currently being captured into */
    volatile uint32_t            next[CAL_CAPT_MAX_STREAMS];
    /**< Index indicating the next buffer that programmed to be received into */
    uint32_t                   firstDmaStartIntr[CAL_CAPT_MAX_STREAMS];
    /**< Flag to indicate whether this is first DMA start Interrupt
     *   First DMA start interrupt should not mark frame as done as it is
     *   just started capturing. */
    Cal_CoreCaptOpenParams_t   coreOpenPrms;
    /**< Copy of the core open parameters, containing request frame
     *   and frame done callback. This will be used by the ISP layer */
    uint32_t                   bysOutCportId;
    /**< Cport id of the Bys output */
    uint32_t                   vportCportId;
    /**< Cport id of the vport output */

    Cal_CoreCaptStreamMode_t streamMode[CAL_CAPT_MAX_STREAMS];
    /**< Each of stream could be processed by CAL or other entity. Each stream
            when created will be associated an entity that processes it. */

    void                      *emSubFrmCmpltHndl;
    /**< Event manager handle for Sub Frame reception completion */
    uint32_t                   subFrmstreamId;
    /**< Stream ID, for which X line callback is enabled */
    Fvid2_SubFrameInfo         currSubFrame[CAL_CAPT_MAX_STREAMS][2U];
    /**< Place holder for sub-frame Info */
    uint32_t                   isSubFrameCfgValid;
    /**< Flag indicating valid / in valid sub-frame config. FALSE is invalid
        and TRUE is valid */
    Cal_CoreCaptSubFrameCfg_t   subFrameCfg;
    /**< Configuration for sub-frame and end of frame */
} Cal_CoreCaptCalHndlObj_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  Below ifdef __cplusplus is added so that C++ build passes without
 *  typecasting. This is because the prototype is build as C type
 *  whereas this file is build as CPP file. Hence we get C++ build error.
 *  Also if typecasting is used, then we get MisraC error Rule 11.1.
 */
#ifdef __cplusplus
extern "C" {
#endif
/** \brief Function to get the core properties
 *
 *  \param instObj     Pointer to instance Object
 *  \param property    Pointer to property, out parameter
 *
 *  \return            return FVID2_SOK on success
 *                     error code on fail
 */
int32_t calCoreCaptGetProperty(Cal_CoreInst      instObj,
                           Cal_CoreProperty *property);

/** \brief Opens the Core, in turn, opens the required hals, sets the defaults
 *         parameters
 *
 *  \param inst        Pointer to instance id
 *  \param openPrms    Core open parameters containing get/put frame
 *                     function pointers
 *  \param coreOpenPrms Core Specific Open Parameter, contains list of
 *                     modules required for this instance
 *  \param coreRetPrms Returns Params
 *
 *  \return            Valid core handle on success
 *                     NULL on Failure
 */
Cal_CoreHandle calCoreCaptOpen(Cal_CoreInst            inst,
                             const Cal_CoreOpenPrms *openPrms,
                             const void             *coreOpenPrms,
                             void                   *coreReturnPrms);

/** \brief Closes the core
 *
 *  \param handle      core handle
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptClose(Cal_CoreHandle handle);

/** \brief Sets the parameters, used to enable specific streams,
 *         frame size, frame dataformat etc.
 *
 *  \param handle      core handle
 *  \param params      Pointer to the parameters
 *  \param argsNotUsed Return parameters
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptSetParams(Cal_CoreHandle handle,
                           const void    *params,
                           void          *argsNotUsed);

/** \brief Start the capture
 *
 *  \param handle      core handle
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptStart(Cal_CoreHandle handle);

/** \brief Stop the capture
 *
 *  \param handle      core handle
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptStop(Cal_CoreHandle handle);

/** \brief Configures the frame to be received. Should be called only once
 *          before CAL is started, should rely on requestFrame and frameComplete
 *          supplied during open.
 *
 *  \param handle                   Core handle
 *  \param frame                    Pointer to frame
 *  \param bypassLowLatencyCheck    Low latency check bypass/enable
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptProgBuf(Cal_CoreHandle handle,
                       Cal_CoreFrame *frame,
                       uint32_t         bypassLowLatencyCheck);

/** \brief Get the current parameters
 *
 *  \param handle      core handle
 *  \param params      Pointer to the parameters
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptGetParams(Cal_CoreHandle handle,
                           void          *params);

/**  \brief Function to process control commands
 *
 *  \param handle      core handle
 *  \param cmd         control command
 *  \param appArgs     Command specific arguments
 *  \param drvArgs     Extra arguments
 *
 *  \return            TRUE on success
 *                     FALSE on Failure
 */
int32_t calCoreCaptControl(Cal_CoreHandle handle,
                         uint32_t         cmd,
                         void          *appArgs,
                         void          *drvArgs);

/**  \brief Write DMA has started writing, called by event manager.
 *              This ISR would be called only in case CAL is writing to memory.
 *
 *  \param event       Arrays of events which occurred
 *  \param numEvents   number of valid entries in event
 *  \param arg         Not used
 *
 */
static void calCoreCaptDmaStartCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void         *arg);

/** \brief Write DMA has completed writing, called by event manager
 *              This ISR would be called only in case CAL is writing to memory.
 *
 *  \param event       Arrays of events which occurred
 *  \param numEvents   number of valid entries in event
 *  \param arg         Not used
 *
 */
static void calCoreCaptDmaCmpltCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void         *arg);

/** \brief Configured line has been received, called by event manager
 *          This ISR would be called only when X line event is configured.
 *
 *  \param event       Arrays of events which occurred
 *  \param numEvents   number of valid entries in event
 *  \param arg         Not used
 *
 */
static void calCoreCaptXlineCmpltCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void           *arg);

/** \brief Frame Done Processing - provide frame info and execute callback
 *  function
 */
static int32_t calCoreCaptFrameDoneProc(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                      uint32_t                    streamId);
/** \brief Sub Frame completed - Will check for errors and issue callback's.
 *          Is expected to be called on write DMA completion also (end-of-frame)
 *
 */
static int32_t calCoreCaptSubFrameDoneProc(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                        uint32_t                    streamId,
                                        uint32_t                    isEof);
/** \brief Function to deallocate CAL sub modules
 *
 *  \param calId        CAL Id
 *  \param resDeAlloc   Pointers to array of size "cnt"
 *  \param cnt          Number of resources pointed by resDeAlloc
 *
 *  \return             FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptDeAllocCalRes(
    Cal_RmModules_t         calId,
    Cal_CaptBlocks_t resDeAlloc[
        CAL_CAPT_MAX_STREAMS],
    uint32_t                cnt);

/** \brief Function to allocate CAL handle and sub modules required. Also
 *          determines if a given stream is to be processed by CAL / Others
 *
 *  \param calId        CAL Id
 *  \param pInstObj     Pointer to instance
 *  \param pOpenParams  Application specified parameters
 *  \param pOpenRtnPrms Application specified return values
 *
 *  \return             FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptAllocCalRes(
    Cal_RmModules_t                   calId,
    Cal_CoreCaptCalHndlObj_t     *pHndlObj,
    const Cal_CoreCaptOpenParams_t *pOpenParams,
    Cal_CoreCaptOpenRetParams_t    *pOpenRtnPrms);

/** \brief Function to open all required HALs
 *
 *  \param calId        CAL Id
 *  \param pInstObj     Pointer to handle
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptOpenHals(Cal_RmModules_t                    calId,
                                 const Cal_CoreCaptCalHndlObj_t *pHndlObj);

/** \brief Function to close all required HALs
 *
 *  \param pInstObj     Pointer to instance object
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptCloseHals(const Cal_CoreCaptCalHndlObj_t *pHndlObj);

/** \brief Function to check if the config supplied is alright.
 *
 *  \param pHndl        Pointer to handle object
 *  \param pInstObj     Pointer to CAL configuration.
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptCheckCalCfg(const Cal_CoreCaptCalHndlObj_t *pHndl,
                                    const Cal_HalCfg_t             *pCalCfg);

/** \brief Function to prepare cal core config
 *
 *  \param pHndl        Pointer to handle object
 *  \param pInstObj     Pointer to CAL configuration.
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptMakeCalCoreCfg(Cal_CoreCaptCalHndlObj_t *pHndl,
                                       Cal_HalCfg_t             *pCalCfg);

/** \brief Function to associate allocated instances
 *
 *  \param pHndl        Pointer to handle object
 *  \param pInstObj     Pointer to CAL configuration.
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptDetermineInst(const Cal_CoreCaptCalHndlObj_t *pHndl,
                                      Cal_HalCfg_t                   *pCalCfg);

/** \brief Function to initialize the defaults
 *
 *  \param pCalCfg     Pointer to CAL config
 *  \param pCalCoreCfg Pointer to CAL core config / derived config
 *
 *  \return            FVID2_SOK on success, appropriate error code otherwise.
 */
static void calCoreCaptSetDefaultParams(Cal_HalCfg_t      *pCalCfg,
                                      Cal_HalDmaVcCfg_t *pCalCoreCfg);

/** \brief Allocates a handle object from pool of handle objects.
 *
 *  \param pInstObj Pointer to instance Object, used to identify CAL instance
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static Cal_CoreCaptCalHndlObj_t *calCoreCaptAllocHndlObj(
    Cal_CoreCaptInstObj_t *pInstObj);

/** \brief Registers interrupt handler for DMA writes and ISP writes
 *
 *  \param pInstObj Pointer to handle object, with valid IRQ no, and others
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptRegisterIsr(Cal_CoreCaptCalHndlObj_t *pHndlObj);

/** \brief Validates open time configurations.
 *
 *  \param hObj     Pointer to handle object.
 *  \param openPrms Pointer to open parameters
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptCheckOpenParams(
    const Cal_CoreCaptInstObj_t  *pInstObj,
    const Cal_CoreCaptOpenParams_t *pOpenParams);

/**  \brief Checks if the specified CSI2 data formats is YUV420 variant
 *  This will be used to increase the line count in CAL configuration
 */
static inline int32_t calCoreCaptCsi2IsDataYuv420(uint32_t dataFmt);

/**
 *  \brief Checks if the specified CSI2 data formats is pixel data
 */
static inline int32_t calCoreCaptCsi2IsPixData(uint32_t dataFmt);

/**
 *  \brief Checks and applies the given config. Essentially converts the config
 *          specified via Cal_Cfg_t to config based on Cal_HalCfg_t
 */
static int32_t calCoreCaptSetCalConfig(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                    const Cal_Cfg_t       *pCfg);

/**
 *  \brief Primes CAL module with 1 buffer and starts CAL based reception.
 */
static int32_t calCoreCaptPrimeStartCal(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                     void                       *argsNotUsed);

/**  \brief Write DMA has started writing, called by event manager.
 *              This ISR would be called only in case CAL is writing to memory.
 *
 *  \param event       Arrays of events which occurred
 *  \param numEvents   number of valid entries in event
 *  \param arg         Not used
 *
 */
static void calCoreCaptErrCb(const uint32_t *event,
                           uint32_t        numEvents,
                           void         *arg);
/** \brief Registers error interrupt handler
 *
 *  \param pInstObj Pointer to handle object
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptRegisterErrorIsr(const Cal_CoreCaptCalHndlObj_t
                                                                   *pHndlObj,
                                         const Cal_ErrorCfg_t *pErrCfg);

/** \brief Configure sub-frame parameters and End of frame notification.
 *
 *  \param pInstObj Pointer to handle object
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptCfgSubEof(
                                Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                const Cal_CoreCaptSubFrameCfg_t *pSubFrmCfg);

/** \brief Registers end of frame and x line interrupt handler
 *
 *  \param pInstObj Pointer to handle object
 *
 *  \return         FVID2_SOK on success, appropriate error code otherwise.
 */
static int32_t calCoreCaptRegisterSubEofIsr(
                                Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                const Cal_CoreCaptSubFrameCfg_t *pSubFrmCfg);
#ifdef __cplusplus
}
#endif

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/** \brief Core handles */
static Cal_CoreCaptCalHndlObj_t
    gCaptHndlObjs[CAL_CAPT_INST_ID_MAX][CAL_CORE_CAPT_MAX_OPEN];

/** \brief Capture Instance object */
static Cal_CoreCaptInstObj_t gCaptInstObjs[CAL_CAPT_INST_ID_MAX] =
{
    {(uint32_t) FALSE, /* We really require to set this to FALSE. */
     CAL_CAPT_INST_ID_MAX},
    {(uint32_t) FALSE, /* We really require to set this to FALSE. */
     CAL_CAPT_INST_ID_MAX},
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/** Initializes the data structures as per the init parameters,
 *  Also initializes the hal as per the driver instance
 */
int32_t Cal_coreCaptInit(
    uint32_t                        numInst,
    const Cal_CoreCaptInitParams_t *initParams,
    void                           *arg)
{
    Cal_CoreCaptInstObj_t          *pInstObj;
    const Cal_HalPlatformData_t     *pCalPlatData;
    Cal_HalInstParams_t            *pCalInstParams;
    int32_t          rtnVal = FVID2_SOK;
    uint32_t           inst, i;
    Cal_RmInitParams_t irmInitParams;
    SemaphoreP_Params params;

    pCalPlatData = NULL;

    /* Args check */
    if((NULL == initParams) || (CAL_CAPT_INST_ID_MAX < numInst))
    {
        rtnVal = FVID2_EBADARGS;
    }

    if (FVID2_SOK == rtnVal)
    {
        for(inst = 0x0U; inst < numInst; inst++)
        {
            pInstObj = NULL;

            /* Args check */
            if((FVID2_SOK == rtnVal) && (NULL == initParams->halPlatformData))
            {
                rtnVal = FVID2_EBADARGS;
            }

            if((FVID2_SOK == rtnVal) &&
               (initParams->instId >= CAL_CAPT_INST_ID_MAX))
            {
                rtnVal = FVID2_EBADARGS;
            }

            if(FVID2_SOK == rtnVal)
            {
                pInstObj = &gCaptInstObjs[initParams->instId];
                if(TRUE == pInstObj->isInited)
                {
                    /* already initialized */
                    pInstObj = NULL;
                }
                else
                {
                    memset((void *)&gCaptInstObjs[initParams->instId], 0x0,
                                    sizeof(Cal_CoreCaptInstObj_t));
                    gCaptInstObjs[initParams->instId].instId = initParams->instId;
                }
            }

            if((FVID2_SOK == rtnVal) && (NULL != pInstObj))
            {
                pCalPlatData = (Cal_HalPlatformData_t *) initParams->halPlatformData;
                GT_assert(CalTrace, (NULL != pCalPlatData));
                GT_assert(CalTrace, (NULL != pCalPlatData->calInstPrms));

                pCalInstParams = pCalPlatData->calInstPrms;
                GT_assert(CalTrace, (NULL != pCalInstParams));
                pInstObj->instId = initParams->instId;

                /* For each instance,
                 *  . initialize isscontrol
                 *  . cal hal
                 *  . copy the initparams
                 *  . create instance lock */
                irmInitParams.numCalBlocks = pCalPlatData->numCalInst;
                irmInitParams.arg          = NULL;
                rtnVal = Cal_rmInit(0x0, &irmInitParams);

                if(FVID2_SOK == rtnVal)
                {
                    rtnVal = Cal_halInit(pCalPlatData->numCalInst,
                                               pCalInstParams, NULL);
                }

                if(FVID2_SOK == rtnVal)
                {
                    rtnVal = FVID2_EALLOC;
                    SemaphoreP_Params_init(&params);
                    params.mode = SemaphoreP_Mode_BINARY;
                    pInstObj->instLock = SemaphoreP_create(1U, &params);
                    if(NULL != pInstObj->instLock)
                    {
                        memcpy((void *) (&pInstObj->initPrms),
                                        (const void *) (initParams),
                                        sizeof(Cal_CoreCaptInitParams_t));
                        memset((void *)&gCaptHndlObjs[inst][0U], 0x0,
                                        (sizeof(Cal_CoreCaptCalHndlObj_t) *
                                         CAL_CORE_CAPT_MAX_OPEN));
                        for(i = 0U; i < CAL_CORE_CAPT_MAX_OPEN; i++)
                        {
                            gCaptHndlObjs[inst][i].pInstObj = pInstObj;
                        }
                        pInstObj->isInited = (uint32_t)TRUE;
                        rtnVal = FVID2_SOK;
                    }
                }
            }

            initParams++;
        }

    }
    return (rtnVal);
}

/* Function to de-initialize all instances, it also de-initializes the hals */
int32_t Cal_coreCaptDeInit(void *arg)
{
    int32_t rtnVal = FVID2_SOK;
    Cal_CoreCaptInstObj_t     *pInstObj;
    uint32_t  i;

    for(i = 0U; i < CAL_CAPT_INST_ID_MAX; i++)
    {
        pInstObj = &gCaptInstObjs[i];
        if(TRUE == pInstObj->isInited)
        {
            rtnVal = Cal_halDeInit(NULL);
            GT_assert(CalTrace, (FVID2_SOK == rtnVal));

            SemaphoreP_delete(pInstObj->instLock);
        }
    }
    if(FVID2_SOK == rtnVal)
    {
        Cal_rmDeInit();
    }

    return (rtnVal);
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

/** API to return the property of the instance
 */
int32_t calCoreCaptGetProperty(Cal_CoreInst      instObj,
                           Cal_CoreProperty *property)
{
    int32_t retVal = FVID2_EFAIL;
    if(NULL != property)
    {
        property->name = CAL_CORE_CAPT;
        property->isDropDataSupport = FALSE;
        retVal = FVID2_SOK;
    }
    return (retVal);
}

/** Returns the core operations
 */
const Cal_CoreOps *Cal_coreCaptGetCoreOps(void)
{
    /** \brief Capture core function pointers. */
    static const Cal_CoreOps gCaptureCoreOps = {       \
        &calCoreCaptGetProperty,  /* Get property */     \
        &calCoreCaptOpen,         /* Open function */    \
        &calCoreCaptClose,        /* Close function */   \
        &calCoreCaptSetParams,    /* Set Params */       \
        NULL,                   /* Get Params */       \
        &calCoreCaptControl,      /* Control */          \
        &calCoreCaptStart,        /* Start Required */   \
        &calCoreCaptStop,         /* Stop Required */    \
        NULL,                   /* Process */          \
        &calCoreCaptProgBuf,      /* Core prog Buffer */ \
        NULL,                   /* putFrames */        \
        NULL,                   /* getFrames */        \
        NULL                    /* getErrorStat */     \
    };

    return (&gCaptureCoreOps);
}

/**
 *  A Valid pointer on success else a NULL pointer.
 */
Cal_CoreInst Cal_coreCaptGetCoreInstObj(Cal_CaptInstId_t inst)
{
    Cal_CoreInst rtnVal = NULL;
    if(inst < CAL_CAPT_INST_ID_MAX)
    {
        rtnVal = (Cal_CoreInst)&gCaptInstObjs[inst];
    }
    return (rtnVal);
}

/** \brief Typedef for core open function pointer. */
Cal_CoreHandle calCoreCaptOpen(Cal_CoreInst            inst,
                             const Cal_CoreOpenPrms *openPrms,
                             const void             *coreOpenPrms,
                             void                   *coreReturnPrms)
{
    const Cal_CoreCaptOpenParams_t *pOpenParams;
    Cal_CoreCaptInstObj_t        *pInstObj;
    Cal_CoreCaptCalHndlObj_t     *pHndlObj;
    int32_t         rtnVal;
    uint32_t        i;
    uint32_t      stepStatus;
    Cal_RmModules_t calId;

    /*  Steps
     *  . Args check
     *  . Config check
     *  . Acquire Instance lock
     *  . alloc Handle Object
     *  . alloc resource
     *  . open hals
     *  . Set Defaults
     *  . Register with event manager, for the frame start interrupt of
     *      DMA write.
     *  . set instance config, enable PPI & complex IO
     *  . Set Vmux settings for LVDS Rx Capture
     *  . Update status if all steps are successful.
     */

    /* Init */
    pHndlObj   = NULL;
    rtnVal     = FVID2_EBADARGS;
    stepStatus = 0x0U;

    /* .Args Check */
    if(NULL != inst)
    {
        rtnVal   = FVID2_SOK;
        pInstObj = (Cal_CoreCaptInstObj_t *) inst;
        GT_assert(CalTrace, (NULL != pInstObj));

/* C & C++ INVARIANT_CONDITION.GEN
 * Expression '0 == (pInstObj)' used in the condition always yields the same
 * result.
 * Name 'pInstObj'
 * KW State: Ignore -> Waiver -> Case by case
 * WAIVER:
 * In this perticular case, if an wrong inst is provided by the caller, the
 * pInstObj could be NULL and hence this check is required.
 */
        if(((NULL == openPrms) || (NULL == coreOpenPrms)) ||
           ((NULL == coreReturnPrms) || (NULL == pInstObj)))
        {
            rtnVal = FVID2_EBADARGS;
        }

        /* .Config checks */
        if(FVID2_SOK == rtnVal)
        {
            if(FALSE == pInstObj->isInited)
            {
                rtnVal = FVID2_EUNSUPPORTED_OPS;
            }
            else if(TRUE == pInstObj->isStarted)
            {
                /* Should be the second check */
                rtnVal = FVID2_EDEVICE_INUSE;
            }
            else if(CAL_CORE_CAPT_MAX_OPEN <= pInstObj->numOpens)
            {
                rtnVal = FVID2_EALLOC;
            }
            else
            {
                pOpenParams = (const Cal_CoreCaptOpenParams_t *) coreOpenPrms;
                rtnVal      = calCoreCaptCheckOpenParams(pInstObj, pOpenParams);
            }
        }

        SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);

        /* . Alloc handle Object */
        if((FVID2_SOK == rtnVal) &&
           (CAL_CAPT_INST_ID_MAX > pInstObj->instId))
        {
            pHndlObj = calCoreCaptAllocHndlObj(pInstObj);

            if(NULL != pHndlObj)
            {
                stepStatus |= 0x1U;
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }
        else
        {
            if((FVID2_SOK == rtnVal) &&
               (CAL_CAPT_INST_ID_MAX <= pInstObj->instId))
            {
                rtnVal = FVID2_EBADARGS;
            }
        }

        /* . Alloc resources */
        if(FVID2_SOK == rtnVal)
        {
            /* Event group is DMA Write start CAL A */
            calId = CAL_RM_MODULE_CAL_A;

            rtnVal = calCoreCaptAllocCalRes(
                calId,
                pHndlObj,
                pOpenParams,
                (Cal_CoreCaptOpenRetParams_t *) coreReturnPrms);

            if(FVID2_SOK == rtnVal)
            {
                stepStatus |= 0x2U;

                memcpy((void *) (&pHndlObj->openPrms),
                                (const void *) (openPrms),
                                sizeof(Cal_CoreOpenPrms));
                memcpy((void *) (&pHndlObj->coreOpenPrms),
                                (const void *) (pOpenParams),
                                sizeof(Cal_CoreCaptOpenParams_t));
            }
            else
            {
                rtnVal = FVID2_EALLOC;
            }
        }

        /* . Open Required HALs */
        if((FVID2_SOK == rtnVal) && (0x0 == pInstObj->numOpens))
        {
            rtnVal = calCoreCaptOpenHals(calId, pHndlObj);
            if(FVID2_SOK == rtnVal)
            {
                stepStatus |= 0x4U;
            }
        }

        /* Set Defaults */
        if(FVID2_SOK == rtnVal)
        {
            if(0x0 == pInstObj->numOpens)
            {
                calCoreCaptSetDefaultParams(&pInstObj->calCfg,
                                          &pHndlObj->calDmaVcCfg);
            }
            else
            {
                calCoreCaptSetDefaultParams(NULL, &pHndlObj->calDmaVcCfg);
            }
        }

        /* . Register Event Handle */
        if(FVID2_SOK == rtnVal)
        {
            rtnVal = calCoreCaptRegisterIsr(pHndlObj);

            if(FVID2_SOK == rtnVal)
            {
                stepStatus |= 0x8U;
            }
        }

        /* . Configure Phy/Complex IO */
        if(FVID2_SOK == rtnVal)
        {
            /* Except for the LVDS parallel interface,
               Phy configuration is required for all instance */
            if(CAL_CAPT_INST_ID_A_CPI != pInstObj->instId)
            {
                /* Check if opening for the first time, if so set
                 * the instance cfg
                 * HAL TODO. Ensure to not update m2m instance config */
                if (0x0 == pInstObj->numOpens)
                {
                    rtnVal = Cal_halControl(
                        pInstObj->calHalHandle,
                        IOCTL_CAL_HAL_GET_INSTANCECFG,
                        (void *) &pInstObj->instCfg, NULL);

                    for (i = 0; ((i < CAL_CAPT_MAX_CMPLXIO_INST) &&
                                 (FVID2_SOK == rtnVal)); i++)
                    {
                        pInstObj->instCfg.ppiCfg[i].enable = (uint32_t)FALSE;

                        if(TRUE == pOpenParams->isCmplxIoCfgValid[i])
                        {
                            pInstObj->instCfg.ppiCfg[i].enable = (uint32_t)TRUE;
                            pInstObj->instCfg.ppiCfg[i].instance = i;
                            /* Complex IO configuration is done in the HAL */
                            memcpy(
                                (void *) (&pInstObj->instCfg.cmplxIoCfg[i]),
                                (const void *) (&pOpenParams->cmplxIoCfg[i]),
                                (sizeof(Cal_CmplxIoCfg_t)));
                        }
                        else
                        {
                            pInstObj->instCfg.cmplxIoCfg[i].enable =
                                                                (uint32_t)FALSE;
                        }
                        if(pInstObj->instCfg.csi2PhyClock[i] !=
                                                pOpenParams->csi2PhyClock[i])
                        {
                            pInstObj->instCfg.csi2PhyClock[i] =
                                                pOpenParams->csi2PhyClock[i];
                        }
                    }

                    if (FVID2_SOK == rtnVal)
                    {
                        rtnVal = Cal_halControl(
                                        pInstObj->calHalHandle,
                                        IOCTL_CAL_HAL_SET_INSTANCECFG,
                                        (void *) &pInstObj->instCfg, NULL);
                    }
                }
            }
        }

        if(FVID2_SOK == rtnVal)
        {
            Cal_CoreCaptOpenRetParams_t *rtnPrms =
                (Cal_CoreCaptOpenRetParams_t *) coreReturnPrms;
            /* We have indeed opened for reception */
            pInstObj->numOpens++;

            /* Let the apps / driver know about allocated resources */
            rtnPrms->numStreamsAlloc = pOpenParams->numStreams;
            for(i = 0; i < pOpenParams->numStreams; i++)
            {
                pHndlObj->curr[i]          = 0U;
                pHndlObj->next[i]          = 1U;
                pHndlObj->currBufs[i][0x0] = NULL;
                pHndlObj->currBufs[i][0x1] = NULL;

                Fvid2SubFrameInfo_init(&pHndlObj->currSubFrame[i][0x0]);
                Fvid2SubFrameInfo_init(&pHndlObj->currSubFrame[i][0x1]);

                rtnPrms->cportIdAlloc[i]   = pHndlObj->allocatedRes[i].cport;
                if((CAL_CORE_STREAM_MODE_CAL_WRITE ==
                    pHndlObj->streamMode[i]) ||
                   (CAL_CORE_STREAM_MODE_ISP == pHndlObj->streamMode[i]))
                {
                    rtnPrms->isStreamOpt[i] = (uint32_t)FALSE;
                }
                else
                {
                    rtnPrms->isStreamOpt[i] = (uint32_t)TRUE;
                }
            }

            pHndlObj->isSubFrameCfgValid = FALSE;
        }
        else
        {
            /* One of the Step is failed, so undo the changes and return
             * null handle */
            if(0x8U == (stepStatus & 0x8U))
            {
                for(i = 0; i < pOpenParams->numStreams; i++)
                {
                    if(0U != (pOpenParams->subModules[i] &
                              CAL_CAPT_INST_ID_SUB_DMA_WR_ID))
                    {
                        if(NULL != pHndlObj->emDmaStartHndl[i])
                        {
                            Cal_emUnRegister(pHndlObj->emDmaStartHndl[i]);
                        }
                        if(NULL != pHndlObj->emDmaCmpltHndl[i])
                        {
                            Cal_emUnRegister(pHndlObj->emDmaCmpltHndl[i]);
                        }
                    }
                }
            }

            if(0x4U == (stepStatus & 0x4U))
            {
                calCoreCaptCloseHals(pHndlObj);
            }

            if(0x2U == (stepStatus & 0x2U))
            {
                calCoreCaptDeAllocCalRes(
                    calId,
                    pHndlObj->allocatedRes,
                    pHndlObj->numStreams);
            }

            if(0x1U == (stepStatus & 0x1U))
            {
                pHndlObj->isAllocated = (uint32_t)FALSE;
                pHndlObj = NULL;
            }
        }

        SemaphoreP_post(pInstObj->instLock);
    }

    return ((Cal_CoreHandle) pHndlObj);
}

int32_t calCoreCaptClose(Cal_CoreHandle handle)
{
    Cal_CoreCaptInstObj_t    *pInstObj;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    Cal_CoreCaptErrObj_t     *pErrObj;
    Cal_RmModules_t calId;
    int32_t       rtnVal = FVID2_EBADARGS;
    uint32_t      strmId, cmplxIoId;
    /*  Steps
     *  . Args check
     *  . Acquire Instance lock
     *  . DeAlloc resources
     *      . Close HALs - relying on HAL to ensure stop has been called.
     *      . DeRegister with event manager
     *  . If its the last close, power down - done by closeHals
     */

    /* .Args Check */
    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;
    if(NULL != pHndlObj)
    {
        rtnVal   = FVID2_SOK;
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            if(TRUE == pInstObj->isStarted)
            {
                rtnVal = FVID2_EDEVICE_INUSE;
            }
            else if(0x0 != pInstObj->numOpens)
            {
                SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);
            }
            else
            {
                /* Otherwise we can close it */
                rtnVal = FVID2_EBADARGS;
            }
        }
        else
        {
            rtnVal = FVID2_EBADARGS;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        if(FVID2_SOK == rtnVal)
        {
            /* Capture only through CAL A */
            calId  = CAL_RM_MODULE_CAL_A;
            rtnVal = calCoreCaptDeAllocCalRes(
                calId,
                pHndlObj->allocatedRes,
                pHndlObj->numStreams);
        }

        if(FVID2_SOK == rtnVal)
        {
            for(strmId = 0; strmId < pHndlObj->numStreams; strmId++)
            {
                if(CAL_CORE_STREAM_MODE_CAL_WRITE ==
                   pHndlObj->streamMode[strmId])
                {
                    if(NULL != pHndlObj->emDmaStartHndl[strmId])
                    {
                        rtnVal = Cal_emUnRegister(
                            pHndlObj->emDmaStartHndl[strmId]);
                        GT_assert(CalTrace, (FVID2_SOK == rtnVal));
                    }

                    if(NULL != pHndlObj->emDmaCmpltHndl[strmId])
                    {
                        rtnVal = Cal_emUnRegister(
                            pHndlObj->emDmaCmpltHndl[strmId]);
                        GT_assert(CalTrace, (FVID2_SOK == rtnVal));
                    }

                    if (NULL != pHndlObj->emSubFrmCmpltHndl)
                    {
                        rtnVal = Cal_emUnRegister(pHndlObj->emSubFrmCmpltHndl);
                        GT_assert(CalTrace, (FVID2_SOK == rtnVal));
                        pHndlObj->emSubFrmCmpltHndl = NULL;
                    }
                }
            }
        }

        if((FVID2_SOK == rtnVal) && (1U == pInstObj->numOpens))
        {
            for(strmId = 0U; strmId < CAL_CAPT_MAX_STREAMS; strmId++)
            {
                pInstObj->streamIdToVcMap[strmId] = 0U;
                pInstObj->streamIdToPpiMap[strmId] = 0U;
            }

            for (cmplxIoId = 0; cmplxIoId < CAL_CAPT_MAX_CMPLXIO_INST; cmplxIoId ++)
            {
                pErrObj = &pInstObj->errObj[cmplxIoId];

                for(strmId = 0U; strmId <= CAL_MAX_VIRTUAL_CHAN; strmId++)
                {
                    pErrObj->fifoOverFlow[strmId]       = 0U;
                    pErrObj->eccCouldNotCorrect[strmId] = 0U;
                    pErrObj->crcMisMatch[strmId]        = 0U;
                    pErrObj->eccCorrected[strmId]       = 0U;
                }

                if (NULL != pErrObj->emErrorHndl)
                {
                    rtnVal = Cal_emUnRegister(
                        pErrObj->emErrorHndl);
                    if(rtnVal == FVID2_SOK)
                    {
                        pErrObj->emErrorHndl = NULL;
                    }
                }
            }
        }

        if(FVID2_SOK == rtnVal)
        {
            rtnVal = calCoreCaptCloseHals(pHndlObj);
        }

        if(FVID2_SOK == rtnVal)
        {
            pInstObj->numOpens--;
            pHndlObj->isAllocated = (uint32_t)FALSE;
        }
        SemaphoreP_post(pInstObj->instLock);
    }

    return (rtnVal);
}

int32_t calCoreCaptSetParams(Cal_CoreHandle handle,
                           const void    *params,
                           void          *argsNotUsed)
{
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    Cal_CoreCaptInstObj_t    *pInstObj;
    int32_t rtnVal = FVID2_EBADARGS, i;

    const Cal_HalCfg_t       *pCalCfg = (const Cal_HalCfg_t *) params;
    /*  . Validate the config
     *  . Compute / update the HAL specifics
     *  . Apply the config
     */

    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;

    if((NULL != pCalCfg) && (NULL != pHndlObj))
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            rtnVal = FVID2_SOK;
            if(TRUE == pInstObj->isStarted)
            {
                rtnVal = FVID2_EDEVICE_INUSE;
                for(i = 0; i < pHndlObj->numStreams; i++)
                {
                    pHndlObj->firstDmaStartIntr[i] = (uint32_t)TRUE;
                }
            }
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        rtnVal = calCoreCaptCheckCalCfg(pHndlObj, pCalCfg);
    }

    if(FVID2_SOK == rtnVal)
    {
        SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);

        /* Compute the core config required */
        rtnVal = calCoreCaptMakeCalCoreCfg(pHndlObj,
                                         (Cal_HalCfg_t *)pCalCfg);

        if(FVID2_SOK == rtnVal)
        {
            /* Get the instance ID of sub-blocks used/required. */
            rtnVal = calCoreCaptDetermineInst(pHndlObj,
                                            (Cal_HalCfg_t *)pCalCfg);
        }

        if(FVID2_SOK == rtnVal)
        {
            /* Apply the config,
             *  Applying configuration after glbce start up sequence since
             *  glbce start up sequence code reset bysout/in configuration */
            rtnVal = Cal_halControl(
                pInstObj->calHalHandle,
                IOCTL_CAL_HAL_SETCFG,
                (void *) pCalCfg, NULL);
        }

        if(FVID2_SOK == rtnVal)
        {
            memcpy(&pInstObj->calCfg, pCalCfg, sizeof(Cal_HalCfg_t));
        }

        SemaphoreP_post(pInstObj->instLock);
    }

    return (rtnVal);
}

int32_t calCoreCaptStart(Cal_CoreHandle handle)
{
    int32_t  rtnVal = FVID2_EBADARGS;
    uintptr_t cookie;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    Cal_CoreCaptInstObj_t    *pInstObj;

    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;

    if(NULL != pHndlObj)
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            rtnVal = FVID2_SOK;

            if(TRUE == pInstObj->isStarted)
            {
                rtnVal = FVID2_EDEVICE_INUSE;
            }
        }
    }
    if((FVID2_SOK == rtnVal) && (FALSE == pInstObj->isStarted))
    {
        SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);

        rtnVal = calCoreCaptPrimeStartCal(pHndlObj, NULL);

        if(FVID2_SOK == rtnVal)
        {
            cookie = HwiP_disable();
            pInstObj->isStarted = (uint32_t)TRUE;
            HwiP_restore(cookie);
        }

        SemaphoreP_post(pInstObj->instLock);
    }

    return rtnVal;
}

int32_t calCoreCaptStop(Cal_CoreHandle handle)
{
    int32_t  rtnVal = FVID2_EBADARGS;
    uintptr_t cookie;
    uint32_t  i;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    Cal_CoreCaptInstObj_t    *pInstObj;
    Cal_HalDmaVcCfg_t        *pDmaVcCfg;

    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;

    if(NULL != pHndlObj)
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            rtnVal = FVID2_SOK;
        }
    }

    if((FVID2_SOK == rtnVal) && (TRUE == pInstObj->isStarted))
    {
        cookie = HwiP_disable();
        if (NULL != pHndlObj->emSubFrmCmpltHndl)
        {
            rtnVal = Cal_emDisable(pHndlObj->emSubFrmCmpltHndl);
        }
        for(i = 0; i < pHndlObj->numStreams; i++)
        {
            if(NULL != pHndlObj->emDmaStartHndl[i])
            {
                rtnVal = Cal_emDisable(pHndlObj->emDmaStartHndl[i]);
            }

            if(NULL != pHndlObj->emDmaCmpltHndl[i])
            {
                rtnVal = Cal_emDisable(pHndlObj->emDmaCmpltHndl[i]);
            }
        }
        HwiP_restore(cookie);
    }

    if((FVID2_SOK == rtnVal) && (TRUE == pInstObj->isStarted))
    {
        pDmaVcCfg = &pHndlObj->calDmaVcCfg;
        GT_assert(CalTrace, (NULL != pDmaVcCfg));
        SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);

        rtnVal = FVID2_EBADARGS;
        for(i = 0; i < pHndlObj->numStreams; i++)
        {
            if(CAL_CORE_STREAM_MODE_CAL_WRITE == pHndlObj->streamMode[i])
            {
                rtnVal = FVID2_SOK;
                pDmaVcCfg->wrDmaCfg[i].mode = CAL_HAL_DMA_WR_DISABLED;
            }
        }
        if(FVID2_SOK == rtnVal)
        {
            rtnVal = Cal_halCaptureStop(pInstObj->calHalHandle,
                                              pDmaVcCfg);
        }
        else
        {
            rtnVal = FVID2_SOK;
        }

        if(FVID2_SOK == rtnVal)
        {
            pInstObj->isStarted = (uint32_t)FALSE;
        }

        SemaphoreP_post(pInstObj->instLock);

#ifdef CAL_FLUSH_ON_STOP
        {
            uintptr_t cookie;
            uint32_t i, curr, next;
            volatile Cal_CoreFrame *currBuf, *nextBuf;

            cookie = HwiP_disable();
            for(i = 0; i < pHndlObj->numStreams; i++)
            {
                pHndlObj->curr[i] = 0U;
                pHndlObj->next[i] = 1U;

                curr    = pHndlObj->curr[i];
                currBuf = pHndlObj->currBufs[i][curr];

                next    = pHndlObj->next[i];
                nextBuf = pHndlObj->currBufs[i][next];
                pHndlObj->currBufs[i][0x0] = NULL;
                pHndlObj->currBufs[i][0x1] = NULL;

                if((nextBuf != NULL) && (NULL != pHndlObj->openPrms.frmDoneCb))
                {
                    pHndlObj->openPrms.frmDoneCb(pHndlObj->openPrms.drvData,
                                                 (Cal_CoreFrame *) nextBuf);
                }
                if((currBuf != NULL) && (NULL != pHndlObj->openPrms.frmDoneCb))
                {
                    pHndlObj->openPrms.frmDoneCb(pHndlObj->openPrms.drvData,
                                                 (Cal_CoreFrame *) currBuf);
                }
            }
            HwiP_restore(cookie);
        }
#endif  /* CAL_FLUSH_ON_STOP */
    }

    return rtnVal;
}

int32_t calCoreCaptProgBuf(Cal_CoreHandle handle,
                       Cal_CoreFrame *frame,
                       uint32_t         bypassLowLatencyCheck)
{
    /* Condition before programming
     *  . Should not be running
     *  . Should not be called second time curr should be NULL*/
    int32_t rtnVal = FVID2_EBADARGS;
    Cal_CoreCaptCalHndlObj_t  *pHndlObj;
    Cal_CoreCaptInstObj_t     *pInstObj;
    Cal_HalBufferAddr_t       *pBuf;
    uint32_t  strmId;

    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;

    if((NULL != pHndlObj) && (NULL != frame))
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            strmId = frame->streamId;
            if(strmId < CAL_CAPT_MAX_STREAMS)
            {
                rtnVal = FVID2_SOK;
            }
        }
    }

    if((FVID2_SOK == rtnVal) && (TRUE != pInstObj->isStarted))
    {
        if(CAL_CORE_STREAM_MODE_CAL_WRITE ==
           pHndlObj->streamMode[strmId])
        {
#ifdef CAL_SEPERATE_API_FOR_PRIME
            SemaphoreP_pend(pInstObj->instLock, SemaphoreP_WAIT_FOREVER);
#endif      /* CAL_SEPERATE_API_FOR_PRIME */

            if(NULL == pHndlObj->currBufs[frame->streamId]
               [pHndlObj->curr[frame->streamId]])
            {
                pBuf = &pHndlObj->bufCfg;
                pBuf->numBuff      = 0x1U;
                pBuf->cPortId[0U]  = pHndlObj->allocatedRes[strmId].cport;
                pBuf->wrDmaCtx[0U] = pHndlObj->allocatedRes[strmId].wrDma;
                pBuf->buffAddr[0U] = (uint32_t) frame->addr[0U];
                pBuf->pitch[0U]    =
                    pHndlObj->calDmaVcCfg.wrDmaCfg[strmId].format.pitch[0U];

                rtnVal = Cal_halUpdateBufAddr(pInstObj->calHalHandle,
                                                    pBuf);
                if(FVID2_SOK == rtnVal)
                {
                    pHndlObj->currBufs[frame->streamId]
                    [pHndlObj->curr[frame->streamId]] = frame;
                }
                else
                {
                    GT_assert(CalTrace, (FVID2_SOK == rtnVal));
                }
            }
            else
            {
                rtnVal = FVID2_EBADARGS;
            }

#ifdef CAL_SEPERATE_API_FOR_PRIME
            SemaphoreP_post(pInstObj->instLock);
#endif      /* CAL_SEPERATE_API_FOR_PRIME */
        }
        else
        {
            rtnVal = FVID2_EBADARGS;
        }
    }
    return rtnVal;
}

/* Get current parameters, copies it from local object */
int32_t calCoreCaptGetParams(Cal_CoreHandle handle,
                           void          *params)
{
    int32_t status = FVID2_SOK;
    /* TODO Do we really required this? */
    return (status);
}

/* Function to process control commands */
int32_t calCoreCaptControl(Cal_CoreHandle handle,
                         uint32_t         cmd,
                         void          *appArgs,
                         void          *drvArgs)
{
    int32_t retVal = FVID2_EBADARGS;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    Cal_CoreCaptInstObj_t    *pInstObj;

    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) handle;

    if(NULL != pHndlObj)
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            retVal = FVID2_SOK;
        }
    }

    if(FVID2_SOK == retVal)
    {
        switch(cmd)
        {
            case CAL_CORE_CAPT_SET_PARAMS:
                if(NULL != appArgs)
                {
                    /* No need to check if CAL is started or config is valid
                        or instance lock
                        The functions within below function will do so.
                        Rely on it */
                    retVal = calCoreCaptSetCalConfig(
                        pHndlObj,
                        (const Cal_Cfg_t *)
                        appArgs);
                }
                else
                {
                    retVal = FVID2_EBADARGS;
                }
                break;

            case CAL_CORE_CAPT_SET_ERR_PRMS:
                if(NULL != appArgs)
                {
                    retVal = calCoreCaptRegisterErrorIsr(
                        pHndlObj,
                        (const
                         Cal_ErrorCfg_t *)
                        appArgs);
                }
                else
                {
                    retVal = FVID2_EBADARGS;
                }
                break;

            case CAL_CORE_CAPT_SET_SUB_FRM_PRMS:
                if(NULL != appArgs)
                {
                    retVal = calCoreCaptCfgSubEof(pHndlObj,
                        (const Cal_CoreCaptSubFrameCfg_t *) appArgs);
                }
                else
                {
                    retVal = FVID2_EBADARGS;
                }
                break;

            default:
                retVal = FVID2_EBADARGS;
                break;
        }
    }

    return (retVal);
}

static void calCoreCaptDmaStartCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void         *arg)
{
    uint32_t         i, eventNo, streamId;
    uintptr_t        cookie;
    int32_t        rtnVal;
    Cal_CoreFrame *newFrame;
    Cal_CoreCaptInstObj_t    *pInstObj;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;

    rtnVal   = FVID2_EBADARGS;
    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) arg;

    if((NULL != pHndlObj) && (NULL != event))
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            rtnVal = FVID2_SOK;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        for(i = 0U; i < numEvents; i++)
        {
            eventNo = event[i];

            /* Translate event number to stream ID */
            if((CAL_EM_EVENT_WDMA_START0 <= eventNo) &&
               (eventNo <= CAL_EM_EVENT_WDMA_START7))
            {
                eventNo -= CAL_EM_EVENT_WDMA_START0;
                streamId = pHndlObj->eventToStreamIdMap[eventNo];
            }
            else
            {
                rtnVal   = FVID2_EBADARGS;
                streamId = 0U;
            }

            if((streamId < CAL_CAPT_MAX_STREAMS) && (FVID2_SOK == rtnVal))
            {
                if(CAL_CORE_STREAM_MODE_CAL_WRITE !=
                   pHndlObj->streamMode[streamId])
                {
                    /* If CAL is not writing, we do not expect start of frame
                        interrupt at all */
                    GT_assert(CalTrace, FALSE);
                }
            }
            else
            {
                rtnVal = FVID2_EBADARGS;
            }

            if (FVID2_SOK == rtnVal)
            {
                if((FALSE == pHndlObj->firstDmaStartIntr[streamId]) &&
                   (NULL != pHndlObj->openPrms.frmDoneCb))
                {
                    rtnVal = calCoreCaptFrameDoneProc(pHndlObj, streamId);
                }
            }

            if((NULL != pHndlObj->openPrms.reqFrmCb) &&
               (FVID2_SOK == rtnVal))
            {
                newFrame = pHndlObj->openPrms.reqFrmCb(
                    pHndlObj->openPrms.drvData,
                    streamId, 0x0);
                if(NULL != newFrame)
                {
                    /* Program the frame for reception */
                    pHndlObj->bufCfg.numBuff     = 0x1U;
                    pHndlObj->bufCfg.cPortId[0U] =
                        pHndlObj->allocatedRes[streamId].cport;
                    pHndlObj->bufCfg.wrDmaCtx[0U] =
                        pHndlObj->allocatedRes[streamId].wrDma;
                    pHndlObj->bufCfg.buffAddr[0U] =
                        (uint32_t) newFrame->addr[0U];
                    pHndlObj->bufCfg.pitch[0U] =
                        pHndlObj->calDmaVcCfg.wrDmaCfg[streamId].format.pitch[0U];

                    rtnVal = Cal_halUpdateBufAddr(pInstObj->calHalHandle,
                                                        &pHndlObj->bufCfg);
                    cookie = HwiP_disable();
                    pHndlObj->currBufs[streamId][pHndlObj->next[streamId]] =
                                                        newFrame;
                    /* Reset the sub-frame count */
                    pHndlObj->currSubFrame[streamId][pHndlObj->next[streamId]] \
                        .subFrameNum = 0xFFFFFFFFU;

                    HwiP_restore(cookie);
                }
                else
                {
                    /* The driver should ensure that this condition
                     *  should not occur. */
                    GT_assert(CalTrace, FALSE);
                }
            }

            if(FVID2_SOK == rtnVal)
            {
                pHndlObj->firstDmaStartIntr[streamId] = (uint32_t)FALSE;
            }
        }
    }
}

static void calCoreCaptXlineCmpltCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void           *arg)
{
    uint32_t  i, eventNo, streamId;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;
    int32_t rtnVal;

    rtnVal   = FVID2_SOK;
    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) arg;

    for(i = 0U; i < numEvents; i++)
    {
        eventNo = event[i];
        if (CAL_EM_EVENT_LINE_NUMBER == eventNo)
        {
            if((TRUE == pHndlObj->isSubFrameCfgValid) && (FVID2_SOK == rtnVal))
            {
                streamId = pHndlObj->subFrmstreamId;

                rtnVal = calCoreCaptSubFrameDoneProc(pHndlObj, streamId,
                                                            (uint32_t)FALSE);
            }
        }
        else
        {
            rtnVal = FVID2_EBADARGS;
        }
    }

    return;
}

static void calCoreCaptDmaCmpltCb(const uint32_t *event,
                                uint32_t        numEvents,
                                void         *arg)
{
    uint32_t  i, eventNo, streamId;
    int32_t rtnVal;
    Cal_CoreCaptInstObj_t    *pInstObj;
    Cal_CoreCaptCalHndlObj_t *pHndlObj;

    rtnVal   = FVID2_EBADARGS;
    pHndlObj = (Cal_CoreCaptCalHndlObj_t *) arg;

    if((NULL != pHndlObj) && (NULL != event))
    {
        pInstObj = pHndlObj->pInstObj;
        if(NULL != pInstObj)
        {
            rtnVal = FVID2_SOK;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        for(i = 0U; i < numEvents; i++)
        {
            eventNo = event[i];

            /* Translate event number to stream ID */
            if((CAL_EM_EVENT_WDMA_END0 <= eventNo) &&
               (eventNo <= CAL_EM_EVENT_WDMA_END7))
            {
                eventNo -= CAL_EM_EVENT_WDMA_END0;
                streamId = pHndlObj->eventToStreamIdMap[eventNo];
            }
            else
            {
                rtnVal   = FVID2_EBADARGS;
                streamId = 0U;
            }

            if((streamId < CAL_CAPT_MAX_STREAMS) && (FVID2_SOK == rtnVal))
            {
                if(CAL_CORE_STREAM_MODE_CAL_WRITE !=
                   pHndlObj->streamMode[streamId])
                {
                    /* If CAL is not writing, we do not expect end of frame
                        interrupt at all */
                    GT_assert(CalTrace, FALSE);
                }
            }
            else
            {
                rtnVal = FVID2_EBADARGS;
            }

            if((NULL != pHndlObj->emDmaCmpltHndl[streamId]) &&
               (FVID2_SOK == rtnVal))
            {
                rtnVal = calCoreCaptSubFrameDoneProc(pHndlObj, streamId,
                                                            (uint32_t)TRUE);
            }
        }
    }
}

static int32_t calCoreCaptSubFrameDoneProc(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                        uint32_t                    streamId,
                                        uint32_t                    isEof)
{
    uint32_t tmpVal, frmStatus, vc, curr, cmplxIdx;
    uintptr_t cookie;
    volatile Cal_CoreFrame  *tmpBuf;
    Fvid2_SubFrameInfo *tmpSubFrmStatus;
    Cal_CoreCaptInstObj_t *pInstObj = NULL;
    Cal_CoreCaptErrObj_t  *pErrObj = NULL;

    cookie      = HwiP_disable();
    curr        = pHndlObj->curr[streamId];
    /* Pending as, frame is not yet ready to be de-queued */
    frmStatus   = FVID2_FRAME_STATUS_PENDING;
    tmpBuf      = pHndlObj->currBufs[streamId][curr];
    tmpSubFrmStatus =
                &pHndlObj->currSubFrame[streamId][curr];

    /* Get errors status if any */
    pInstObj = pHndlObj->pInstObj;
    vc       = pInstObj->streamIdToVcMap[streamId];
    cmplxIdx = pInstObj->streamIdToPpiMap[streamId];
    GT_assert(CalTrace, (cmplxIdx < CAL_CAPT_MAX_CMPLXIO_INST));
    pErrObj = &pInstObj->errObj[cmplxIdx];

    if(CAL_MAX_VIRTUAL_CHAN >= vc)
    {
        /* Note that error status (stored locally in s/w : handleObject is not)
            cleared. This would be cleared while returning back the frame.

            The rational is, if there was one/more error in the first-n-lines,
            this error could be applicable for complete frame. Typically,
            applications want to know reception at end of frame also */
        tmpVal = pErrObj->fifoOverFlow[vc] | pErrObj->eccCouldNotCorrect[vc]
                    | pErrObj->crcMisMatch[vc] | pErrObj->eccCorrected[vc];
        if((uint32_t) FALSE != tmpVal)
        {
            if((uint32_t) TRUE == pErrObj->fifoOverFlow[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_OVERFLOW;
            }
            else if((uint32_t) TRUE == pErrObj->eccCouldNotCorrect[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_ECC_ERROR;
            }
            else if((uint32_t) TRUE == pErrObj->crcMisMatch[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_CRC_ERROR;
            }
            else if((uint32_t) TRUE == pErrObj->eccCorrected[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_ECC_CORRECTED;
            }
            else
            {
                frmStatus = FVID2_FRAME_STATUS_OVERFLOW;
            }
        }
        tmpSubFrmStatus->subFrameNum = 0U;
        tmpBuf->status = frmStatus;
        tmpSubFrmStatus->numOutLines =
                    (tmpSubFrmStatus->subFrameNum + 1U) *
                    pHndlObj->subFrameCfg.notifyAfterFirstXLines[streamId];
        if (TRUE == isEof)
        {
            tmpSubFrmStatus->subFrameNum = 1U;
            tmpSubFrmStatus->numOutLines =
                pHndlObj->calDmaVcCfg.csi2VcCfg[streamId].lines;
        }

        /* Let Applications know */
        pHndlObj->subFrameCfg.appCb(pHndlObj->subFrameCfg.pAppCbArgs,
                                 (Cal_CoreFrame *) tmpBuf,
                                 (Fvid2_SubFrameInfo *) tmpSubFrmStatus);
        if (TRUE == isEof)
        {
            tmpSubFrmStatus->subFrameNum = 0xFFFFFFFFU;
        }
        HwiP_restore(cookie);
    }
    return FVID2_SOK;
}

static int32_t calCoreCaptFrameDoneProc(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                      uint32_t                    streamId)
{
    uint32_t tmpVal, frmStatus, vc, cmplxIdx;
    uintptr_t cookie;
    volatile Cal_CoreFrame  *tmpBuf;
    Cal_CoreCaptInstObj_t *pInstObj = NULL;
    Cal_CoreCaptErrObj_t  *pErrObj = NULL;

    cookie    = HwiP_disable();
    frmStatus = FVID2_FRAME_STATUS_COMPLETED;
    tmpBuf    = pHndlObj->currBufs[streamId][pHndlObj->curr[streamId]];
    tmpVal    = pHndlObj->curr[streamId];
    pHndlObj->currBufs[streamId][pHndlObj->curr[streamId]] = NULL;
    pHndlObj->curr[streamId] = pHndlObj->next[streamId];
    pHndlObj->next[streamId] = tmpVal;
    pInstObj = pHndlObj->pInstObj;
    vc       = pInstObj->streamIdToVcMap[streamId];
    cmplxIdx = pInstObj->streamIdToPpiMap[streamId];

    GT_assert(CalTrace, (cmplxIdx < CAL_CAPT_MAX_CMPLXIO_INST));

    pErrObj = &pInstObj->errObj[cmplxIdx];
    /* Update error status if any */
    if(CAL_BYS_IN_POSITION >= vc)
    {
        tmpVal = pErrObj->fifoOverFlow[vc] |
                 pErrObj->eccCouldNotCorrect[vc] |
                 pErrObj->crcMisMatch[vc] |
                 pErrObj->eccCorrected[vc];

        if((uint32_t) FALSE != tmpVal)
        {
            if((uint32_t) TRUE == pErrObj->fifoOverFlow[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_OVERFLOW;
                pErrObj->fifoOverFlow[vc] = (uint32_t) FALSE;
            }
            else if((uint32_t) TRUE == pErrObj->eccCouldNotCorrect[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_ECC_ERROR;
                pErrObj->eccCouldNotCorrect[vc] = (uint32_t) FALSE;
            }
            else if((uint32_t) TRUE == pErrObj->crcMisMatch[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_CRC_ERROR;
                pErrObj->crcMisMatch[vc] = (uint32_t) FALSE;
            }
            else if((uint32_t) TRUE == pErrObj->eccCorrected[vc])
            {
                frmStatus = FVID2_FRAME_STATUS_ECC_CORRECTED;
                pErrObj->eccCorrected[vc] = (uint32_t) FALSE;
            }
            else
            {
                frmStatus = FVID2_FRAME_STATUS_OVERFLOW;
            }
        }
    }

    tmpBuf->status = frmStatus;
    HwiP_restore(cookie);
    pHndlObj->openPrms.frmDoneCb(pHndlObj->openPrms.drvData,
                                 (Cal_CoreFrame *) tmpBuf);
    return FVID2_SOK;
}

static int32_t calCoreCaptDeAllocCalRes(
    Cal_RmModules_t         calId,
    Cal_CaptBlocks_t resDeAlloc[
        CAL_CAPT_MAX_STREAMS],
    uint32_t                cnt)
{
    uint32_t  i;
    int32_t rtnVal = FVID2_EBADARGS;
    for(i = 0U; i < cnt; i++)
    {
        if(i >= CAL_CAPT_MAX_STREAMS)
        {
            rtnVal = FVID2_EBADARGS;
            break;
        }
        rtnVal = Cal_rmReleaseResource(0x0U, calId, &resDeAlloc[i]);
        GT_assert(CalTrace, (FVID2_SOK == rtnVal));
    }

    return (rtnVal);
}

static int32_t calCoreCaptAllocCalRes(
    Cal_RmModules_t                   calId,
    Cal_CoreCaptCalHndlObj_t     *pHndlObj,
    const Cal_CoreCaptOpenParams_t *pOpenParams,
    Cal_CoreCaptOpenRetParams_t    *pOpenRtnPrms)
{
    uint32_t i, j;
    int32_t  rtnVal = FVID2_SOK;
    Cal_CoreCaptInstObj_t *pInstObj;

    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));

    /* The ISR determines establishes the relation between stream id and event
     *  using a look up table.
     *  This implementation assume that there are equal number of DMA start &
     *  end events. Starting from 0 + offset.
     *  Where offset could vary for start & end events.
     *  If any of the above conditions is not met, the event handling in the
     *  ISR requires an update
     *  i.e. (CAL_EM_EVENT_WDMA_START7 - CAL_EM_EVENT_WDMA_START0) ==
     *  (CAL_EM_EVENT_WDMA_END7 - CAL_EM_EVENT_WDMA_END0)
     */

    /* TODO: Separate allocator for vport based capture and isp resources,
     *       Brijesh */
    for(i = 0U; ((i < pOpenParams->numStreams) && (FVID2_SOK == rtnVal));
        i++)
    {
        /*
            1. Assume this stream is not to be processed by CAL.
            2. Will be processed by CAL, if one of subModules is used
            3. Will be written out by CAL, if WR DMA module is used for this
                stream. As streams cannot be replicated with in CAL, the above
                conditions ensure correct mode of stream */
        pHndlObj->streamMode[i] = CAL_CORE_STREAM_MODE_ISP;
        if(0x0U != pOpenParams->subModules[i])
        {
            rtnVal = Cal_rmAllocResource(0x0, calId, pOpenParams->subModules[i],
                                       &pHndlObj->allocatedRes[i], (uint32_t)
                                       (
                                           CAL_RM_CAL_ALLOC_POLICY_CPORTID_0_LEAST_PREFFERED
                                           |
                                           CAL_RM_CAL_ALLOC_POLICY_WRDMA_0_LEAST_PREFFERED));

            /* Check this only if Write DMA resource is requested */
            if(FVID2_SOK == rtnVal)
            {
                pHndlObj->streamMode[i] = CAL_CORE_STREAM_MODE_CAL;

                if(0U != (pOpenParams->subModules[i] &
                          CAL_CAPT_INST_ID_SUB_DMA_WR_ID))
                {
                    pHndlObj->streamMode[i] = CAL_CORE_STREAM_MODE_CAL_WRITE;
                    j = pHndlObj->allocatedRes[i].wrDma;
                    if(j <=
                       (CAL_EM_EVENT_WDMA_START7 - CAL_EM_EVENT_WDMA_START0))
                    {
                        /* The DMA channel number is between 0U & 7.
                         * Anything else requires update! */
                        pHndlObj->dmaStartEvent[i] = j +
                                                     CAL_EM_EVENT_WDMA_START0;
                        pHndlObj->eventToStreamIdMap[j] = i;
                    }
                    else
                    {
                        /* ?? Resource manager allocated in valid DMA context*/
                        GT_assert(CalTrace, (FALSE));
                    }
                    /* Might seem redundant check, but required if the events
                     *  position is changed, this would require update! */
                    if(j <= (CAL_EM_EVENT_WDMA_END7 - CAL_EM_EVENT_WDMA_END0))
                    {
                        pHndlObj->dmaCmpltEvent[i] =
                            j + CAL_EM_EVENT_WDMA_END0;
                    }
                    else
                    {
                        /* ?? Resource manager allocated in valid DMA context*/
                        GT_assert(CalTrace, (FALSE));
                    }
                }
            }
            else
            {
                pOpenRtnPrms->streamAllocError = i;
                rtnVal = calCoreCaptDeAllocCalRes(
                    calId,
                    &pHndlObj->allocatedRes[0U],
                    i);
                GT_assert(CalTrace, (FVID2_SOK != rtnVal));
            }
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        pHndlObj->numStreams    = pOpenParams->numStreams;
        pHndlObj->bysOutCportId = CAL_CAPT_INST_ID_SUB_MAX;
        pHndlObj->vportCportId  = CAL_CAPT_INST_ID_SUB_MAX;

        /* Store the cport ids for the bys out and vport output ports */
        for(i = 0U; i < pOpenParams->numStreams; i++)
        {
            if(0U != (pOpenParams->subModules[i] &
                      CAL_CAPT_INST_ID_SUB_BYS_OUT_ID))
            {
                pHndlObj->bysOutCportId = pHndlObj->allocatedRes[i].cport;
            }
            if(0U != (pOpenParams->subModules[i] &
                      CAL_CAPT_INST_ID_SUB_VPORT_ID))
            {
                pHndlObj->vportCportId = pHndlObj->allocatedRes[i].cport;
            }
        }
    }

    return (rtnVal);
}

static int32_t calCoreCaptOpenHals(Cal_RmModules_t                    calId,
                                 const Cal_CoreCaptCalHndlObj_t *pHndlObj)
{
    int32_t  rtnVal = FVID2_SOK;
    uint32_t halCalId;
    Cal_HalOpenParams_t     openParms;
    Cal_CoreCaptInstObj_t  *pInstObj;

    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));

    if (CAL_CAPT_INST_ID_A_CPI != pInstObj->instId)
    {
        /* CAL A by default */
        halCalId = 0;
        if(CAL_RM_MODULE_CAL_B == calId)
        {
            halCalId = 1U;
        }

        openParms.instId = halCalId;
        openParms.mode   = CAL_HAL_MODE_CAPTURE;

        pInstObj->calHalHandle = Cal_halOpen(&openParms, NULL);
        if(NULL != pInstObj->calHalHandle)
        {
            rtnVal = FVID2_SOK;
        }
        else
        {
            GT_assert(CalTrace, (FALSE));
            rtnVal = FVID2_EALLOC;
        }
    }

    return (rtnVal);
}

static int32_t calCoreCaptCloseHals(const Cal_CoreCaptCalHndlObj_t *pHndlObj)
{
    int32_t rtnVal = FVID2_EBADARGS;
    Cal_CoreCaptInstObj_t *pInstObj;

    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));

    if(NULL != pInstObj)
    {
        if(NULL != pInstObj->calHalHandle)
        {
            rtnVal = Cal_halClose(pInstObj->calHalHandle, NULL);
            pInstObj->calHalHandle = NULL;
        }

        if((FVID2_SOK == rtnVal) && (0x1U == pInstObj->numOpens))
        {
            /* Nothing much to do now!!! */
        }
    }

    return (rtnVal);
}

static int32_t calCoreCaptCheckCalCfg(const Cal_CoreCaptCalHndlObj_t *pHndl,
                                    const Cal_HalCfg_t             *pCalCfg)
{
    int32_t rtnVal = FVID2_SOK;
    uint32_t  i, j, isCalStreamsDone;

    GT_assert(CalTrace, (NULL != pHndl));
    GT_assert(CalTrace, (NULL != pCalCfg));
    isCalStreamsDone = (uint32_t)FALSE;

    for(i = 0U; ((i < pHndl->numStreams) && (FVID2_SOK == rtnVal)); i++)
    {
        /* For all valid CAL streams, check if CPORT ID match up */
        if((CAL_CORE_STREAM_MODE_CAL == pHndl->streamMode[i]) ||
           (CAL_CORE_STREAM_MODE_CAL_WRITE == pHndl->streamMode[i]))
        {
            rtnVal = FVID2_EBADARGS;
            if(TRUE == isCalStreamsDone)
            {
                /* We expect all CAL streams to be specified first,
                    followed by other streams */
                break;
            }

            for(j = 0U; j < pHndl->numStreams; j++)
            {
                if(pCalCfg->cportId[i] == pHndl->allocatedRes[j].cport)
                {
                    rtnVal = FVID2_SOK;
                    break;
                }
            }
        }
        else if(CAL_CORE_STREAM_MODE_ISP == pHndl->streamMode[i])
        {
            /* No checks for other streams here, right now! */
            isCalStreamsDone = (uint32_t)TRUE;
            rtnVal           = FVID2_SOK;
        }
        else
        {
            isCalStreamsDone = (uint32_t)TRUE;
            rtnVal           = FVID2_EBADARGS;
        }
    }

    /* TODO check what has been allocated and what's being configured */

    return (rtnVal);
}

static int32_t calCoreCaptMakeCalCoreCfg(Cal_CoreCaptCalHndlObj_t *pHndl,
                                       Cal_HalCfg_t             *pCalCfg)
{
    int32_t rtnVal;
    uint32_t  i, j;
    uint32_t  dataFormat, ccsFormat;
    Cal_HalDmaVcCfg_t         *pDmaVcCfg;
    const Cal_CaptBlocks_t *allocRes;

    GT_assert(CalTrace, (NULL != pHndl));
    GT_assert(CalTrace, (NULL != pCalCfg));
    pDmaVcCfg = &pHndl->calDmaVcCfg;
    GT_assert(CalTrace, (NULL != pDmaVcCfg));

    rtnVal = FVID2_SOK;

    pDmaVcCfg->numCPortId = 0x0U;
    for(i = 0U; ((i < pHndl->numStreams) && (FVID2_SOK == rtnVal)); i++)
    {
        rtnVal = FVID2_EBADARGS;
        /* If CAL is processing a given stream, compute the core config
            required, else ignore it */
        if((CAL_CORE_STREAM_MODE_CAL_WRITE == pHndl->streamMode[i])
           ||
           (CAL_CORE_STREAM_MODE_CAL == pHndl->streamMode[i]))
        {
            /* Find associated cport ID */
            for(j = 0U; j < pHndl->numStreams; j++)
            {
                if(pCalCfg->cportId[i] == pHndl->allocatedRes[j].cport)
                {
                    rtnVal = FVID2_SOK;
                    pDmaVcCfg->numCPortId++;
                    allocRes = &pHndl->allocatedRes[j];
                    break;
                }
            }

            if(FVID2_SOK == rtnVal)
            {
                pDmaVcCfg->cportId[i] = allocRes->cport;
                if(TRUE == pCalCfg->isCsi2BasedCapture[i])
                {
                    pDmaVcCfg->isCsi2VcCfgValid[i] = (uint32_t)TRUE;
                    if (0xFFFFFFFFU != allocRes->ppi0Inst)
                    {
                        pDmaVcCfg->csi2VcCfg[i].instance        = 0x0;
                    }
                    else if (0xFFFFFFFFU != allocRes->ppi1Inst)
                    {
                        pDmaVcCfg->csi2VcCfg[i].instance        = 0x1;
                    }
                    else
                    {
                        rtnVal = FVID2_EBADARGS;
                    }
                    pDmaVcCfg->csi2VcCfg[i].contextToBeUsed = allocRes->csi2Ctx;
                    pDmaVcCfg->csi2VcCfg[i].virtualChanNum  =
                        pCalCfg->virtualChanNum[i];

                    /* Generate FVID2 format and ccsFormat from MIPI data type */
                    pDmaVcCfg->csi2VcCfg[i].dt  = pCalCfg->csiDataType[i];
                    pDmaVcCfg->csi2VcCfg[i].att = (uint32_t)TRUE;
                    if((int32_t) TRUE ==
                       calCoreCaptCsi2IsPixData(pCalCfg->csiDataType[i]))
                    {
                        pDmaVcCfg->csi2VcCfg[i].att = (uint32_t)FALSE;
                    }
#ifdef PLATFORM_SIM
                    pDmaVcCfg->csi2VcCfg[i].att = (uint32_t)FALSE;
#endif
                    if((int32_t)FALSE == calCoreCaptCsi2IsDataYuv420
                           (pDmaVcCfg->csi2VcCfg[i].dt))
                    {
                        pDmaVcCfg->csi2VcCfg[i].lines =
                            pCalCfg->streamFmt[i].height;
                    }
                    else
                    {
                        pDmaVcCfg->csi2VcCfg[i].lines =
                            ((uint32_t)(pCalCfg->streamFmt[i].height * 3) >> 1);
                    }
                }
                else
                {
                    pDmaVcCfg->isCsi2VcCfgValid[i] = (uint32_t)FALSE;
                }

                pDmaVcCfg->isWrDmaCfgValid[i] = (uint32_t)FALSE;
                if (TRUE == pCalCfg->writeToMem[i])
                {
                    pDmaVcCfg->isWrDmaCfgValid[i]          = (uint32_t)TRUE;
                    pDmaVcCfg->wrDmaCfg[i].contextToBeUsed = allocRes->wrDma;
                    pDmaVcCfg->wrDmaCfg[i].mode =
                        CAL_HAL_DMA_WR_DISABLED;

                    pDmaVcCfg->wrDmaCfg[i].stream = CAL_TAG_ATT_HDR;

                    if(((int32_t) TRUE ==
                        calCoreCaptCsi2IsPixData(pCalCfg->csiDataType[i]))
                       || (((uint32_t)TRUE == pCalCfg->isBysInCfgValid[i]) &&
                           ((uint32_t)TRUE == pCalCfg->bysInEnable[i])))
                    {
                        pDmaVcCfg->wrDmaCfg[i].stream =
                            CAL_TAG_PIX_DATA;
                    }

                    pDmaVcCfg->wrDmaCfg[i].stallM2MRd = (uint32_t)TRUE;
                    /* No Support for skip mode now TODO */
                    pDmaVcCfg->wrDmaCfg[i].ySkipMode  = 0x0;
                    pDmaVcCfg->wrDmaCfg[i].xPixelSkip = 0x0;
                    dataFormat = pCalCfg->streamFmt[i].dataFormat;
                    ccsFormat  = pCalCfg->streamFmt[i].ccsFormat;

                    /* Width expressed in-terms of 64 bit words. */
                    if ((int32_t) TRUE == Fvid2_isDataFmtRgb32bit(dataFormat))
                    {
                        pDmaVcCfg->wrDmaCfg[i].format.width =
                            ((4U * pCalCfg->streamFmt[i].width) + 7U) / 8U;
                    }
                    else if ((int32_t) TRUE == Fvid2_isDataFmtRgb24bit(dataFormat))
                    {
                        pDmaVcCfg->wrDmaCfg[i].format.width =
                            ((3U * pCalCfg->streamFmt[i].width) + 7U) / 8U;
                    }
                    else if ((int32_t) TRUE == Fvid2_isDataFmtRgb16bit(dataFormat))
                    {
                        pDmaVcCfg->wrDmaCfg[i].format.width =
                            ((2U * pCalCfg->streamFmt[i].width) + 7U) / 8U;
                    }
                    else if((int32_t) TRUE == Fvid2_isDataFmtBayer(dataFormat))
                    {
                        pDmaVcCfg->wrDmaCfg[i].format.width =
                            ((1U * pCalCfg->streamFmt[i].width) + 7U) / 8U;
                    }
                    else if((int32_t) TRUE == Fvid2_isDataFmtYuv422I(dataFormat))
                    {
                        if(FVID2_CCSF_BITS8_PACKED == ccsFormat)
                        {
                            pDmaVcCfg->wrDmaCfg[i].format.width =
                                ((2U * pCalCfg->streamFmt[i].width) + 7U) / 8U;
                        }
                        else if((FVID2_CCSF_BITS10_UNPACKED16 == ccsFormat) ||
                                (FVID2_CCSF_BITS12_UNPACKED16 == ccsFormat) ||
                                (FVID2_CCSF_BITS16_PACKED     == ccsFormat))
                        {
                            pDmaVcCfg->wrDmaCfg[i].format.width =
                                ((4U * pCalCfg->streamFmt[i].width) + 7U) / 8U;

                        }
                        else
                        {
                            rtnVal = FVID2_EBADARGS;
                        }
                    }
                    else
                    {
                        rtnVal = FVID2_EBADARGS;
                    }

                    pDmaVcCfg->wrDmaCfg[i].format.height =
                        pCalCfg->streamFmt[i].height;
                    pDmaVcCfg->wrDmaCfg[i].format.pitch[0U] =
                        pCalCfg->streamFmt[i].pitch[0U];
                }
                else
                {
                    pDmaVcCfg->isWrDmaCfgValid[i] = (uint32_t)FALSE;
                }
            }
        }
        else if(CAL_CORE_STREAM_MODE_ISP == pHndl->streamMode[i])
        {
            rtnVal = FVID2_SOK;
        }
        else
        {
            /* In valid stream */
            break;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        pCalCfg->pDmaVcCfg = &pHndl->calDmaVcCfg;
    }
    return rtnVal;
}

static int32_t calCoreCaptDetermineInst(const Cal_CoreCaptCalHndlObj_t *pHndl,
                                      Cal_HalCfg_t                   *pCalCfg)
{
    int32_t rtnVal;
    uint32_t  i, j;
    const Cal_CaptBlocks_t *allocRes;

    GT_assert(CalTrace, (NULL != pHndl));
    GT_assert(CalTrace, (NULL != pCalCfg));

    rtnVal = FVID2_SOK;

    for(i = 0U; ((i < pCalCfg->numCPortId) && (FVID2_SOK == rtnVal)); i++)
    {
        rtnVal = FVID2_EBADARGS;
        /* Find associated cport ID */
        for(j = 0U; j < pHndl->numStreams; j++)
        {
            if(pCalCfg->cportId[i] == pHndl->allocatedRes[j].cport)
            {
                allocRes = &pHndl->allocatedRes[j];
                rtnVal   = FVID2_SOK;
                break;
            }
        }

        if(FVID2_SOK == rtnVal)
        {
            if(TRUE == pCalCfg->isPixProcCfgValid[i])
            {
                if((allocRes->pixExtract != allocRes->dpmDecode) ||
                   (allocRes->dpmEncode != allocRes->pixPack))
                {
                    GT_assert(CalTrace, FALSE);
                }
                pCalCfg->pixProcCfg[i].contextToBeUsed = allocRes->pixExtract;
            }
            /* TODO for bys out/in & vport */
        }
    }
    return rtnVal;
}

static void calCoreCaptSetDefaultParams(Cal_HalCfg_t      *pCalCfg,
                                      Cal_HalDmaVcCfg_t *pCalCoreCfg)
{
    if(NULL != pCalCfg)
    {
        memset((void *) pCalCfg, 0x0, sizeof(Cal_HalCfg_t));
        pCalCfg->numCPortId    = 0x01U;
        pCalCfg->cportId[0x0U] = 0x01U;

        pCalCfg->streamFmt[0x0].chNum      = 0x0;    /* Do not care not used */
        pCalCfg->streamFmt[0x0].width      = 640U;
        pCalCfg->streamFmt[0x0].height     = 480U;
        pCalCfg->streamFmt[0x0].pitch[0x0] = 640U * 3U;
        pCalCfg->streamFmt[0x0].dataFormat = FVID2_DF_RGB24_888;
        pCalCfg->streamFmt[0x0].ccsFormat  = FVID2_CCSF_BITS8_PACKED;
        pCalCfg->csiDataType[0x0]          = CAL_CSI2_RGB888;

        pCalCfg->isCsi2BasedCapture[0x0] = (uint32_t)TRUE;
        pCalCfg->virtualChanNum[0x0]     = 0x03U;

        pCalCfg->isPixProcCfgValid[0x0] = (uint32_t)FALSE;
        pCalCfg->isBysOutCfgValid[0x0]  = (uint32_t)FALSE;
        pCalCfg->isBysInCfgValid[0x0]   = (uint32_t)FALSE;
        pCalCfg->isVportCfgValid[0x0]   = (uint32_t)FALSE;
    }
    if(NULL != pCalCoreCfg)
    {
        pCalCoreCfg->isCsi2VcCfgValid[0x0]         = (uint32_t)TRUE;
        pCalCoreCfg->csi2VcCfg[0U].instance        = 0x0;
        pCalCoreCfg->csi2VcCfg[0U].contextToBeUsed = 0x0;
        pCalCoreCfg->csi2VcCfg[0U].virtualChanNum  = 0x3U;
        pCalCoreCfg->csi2VcCfg[0U].dt              = 0x24; /* RGB 888 */
        pCalCoreCfg->csi2VcCfg[0U].att             = (uint32_t)FALSE;
        pCalCoreCfg->csi2VcCfg[0U].lines           = 480U;

        pCalCoreCfg->isWrDmaCfgValid[0U]          = (uint32_t)TRUE;
        pCalCoreCfg->wrDmaCfg[0U].contextToBeUsed = 0x1;
        pCalCoreCfg->wrDmaCfg[0U].mode =
            CAL_HAL_DMA_WR_CONST;
        pCalCoreCfg->wrDmaCfg[0U].stream           = CAL_TAG_PIX_DATA;
        pCalCoreCfg->wrDmaCfg[0U].stallM2MRd       = (uint32_t)TRUE;
        pCalCoreCfg->wrDmaCfg[0U].ySkipMode        = 0x0;
        pCalCoreCfg->wrDmaCfg[0U].xPixelSkip       = 0x0;
        pCalCoreCfg->wrDmaCfg[0U].format.width     = 640U;
        pCalCoreCfg->wrDmaCfg[0U].format.height    = 480U;
        pCalCoreCfg->wrDmaCfg[0U].format.pitch[0U] = 640U * 3U;

        pCalCoreCfg->isRdDmaCfgValid[0U] = (uint32_t)FALSE;
    }
}

static int32_t calCoreCaptCsi2IsPixData(uint32_t dataFmt)
{
    int32_t retVal;
    switch(dataFmt)
    {
        case CAL_CSI2_YUV420_8B:
        case CAL_CSI2_YUV420_10B:
        case CAL_CSI2_YUV420_8B_LEGACY:
        case CAL_CSI2_YUV420_8B_CHROMA_SHIFT:
        case CAL_CSI2_YUV420_10B_CHROMA_SHIFT:
        case CAL_CSI2_YUV422_8B:
        case CAL_CSI2_YUV422_10B:
        case CAL_CSI2_RGB444:
        case CAL_CSI2_RGB555:
        case CAL_CSI2_RGB565:
        case CAL_CSI2_RGB666:
        case CAL_CSI2_RGB888:
        case CAL_CSI2_RAW6:
        case CAL_CSI2_RAW7:
        case CAL_CSI2_RAW8:
        case CAL_CSI2_RAW10:
        case CAL_CSI2_RAW12:
        case CAL_CSI2_RAW14:
        case CAL_CSI2_ANY:
            retVal = (int32_t)TRUE;
            break;
        default:
            retVal = (int32_t)FALSE;
            break;
    }

    return retVal;
}

static inline int32_t calCoreCaptCsi2IsDataYuv420(uint32_t dataFmt)
{
    int32_t retVal;

    switch(dataFmt)
    {
        case CAL_CSI2_YUV420_8B:
        case CAL_CSI2_YUV420_10B:
        case CAL_CSI2_YUV420_8B_LEGACY:
        case CAL_CSI2_YUV420_8B_CHROMA_SHIFT:
        case CAL_CSI2_YUV420_10B_CHROMA_SHIFT:
            retVal = (int32_t) TRUE;
            break;

        default:
            retVal = (int32_t) FALSE;
            break;
    }

    return (retVal);
}

static int32_t calCoreCaptCheckOpenParams(
    const Cal_CoreCaptInstObj_t  *pInstObj,
    const Cal_CoreCaptOpenParams_t *pOpenParams)
{
    int32_t  retVal = FVID2_SOK;
    uint32_t i, lSubModules;

    GT_assert(CalTrace, (NULL != pInstObj));
    GT_assert(CalTrace, (NULL != pOpenParams));

    if(pOpenParams->numStreams > CAL_CAPT_MAX_STREAMS)
    {
        retVal = FVID2_EBADARGS;
    }

    if((CAL_CAPT_INST_ID_A_CPI == pInstObj->instId) &&
       (CAL_CORE_CAPT_IF_CPI != pOpenParams->captIf))
    {
        /* Only Parallel interface is supported on Vport instance */
        retVal = FVID2_EBADARGS;
    }

    for(i = 0U; ((i < pOpenParams->numStreams) && (FVID2_SOK == retVal));
        i++)
    {
        lSubModules = pOpenParams->subModules[i];
        if((CAL_CAPT_INST_ID_A_CPI == pInstObj->instId) && (0U != lSubModules))
        {
            /* No CAL modules supported for the VPort instance */
            retVal = FVID2_EBADARGS;
        }
    }

    return (retVal);
}

static Cal_CoreCaptCalHndlObj_t *calCoreCaptAllocHndlObj(
    Cal_CoreCaptInstObj_t *pInstObj)
{
    uint32_t cnt;
    Cal_CoreCaptCalHndlObj_t *hObj = NULL;

    GT_assert(CalTrace, (NULL != pInstObj));

    for(cnt = 0U; cnt < CAL_CORE_CAPT_MAX_OPEN; cnt++)
    {
        if(FALSE == gCaptHndlObjs[pInstObj->instId][cnt].isAllocated)
        {
            gCaptHndlObjs[pInstObj->instId][cnt].isAllocated = (uint32_t)TRUE;
            hObj = &gCaptHndlObjs[pInstObj->instId][cnt];
            memset((void *)
                            &gCaptHndlObjs[pInstObj->instId][cnt].allocatedRes,
                            (uint8_t) 0xFF,
                            (sizeof(Cal_CaptBlocks_t) *
                             CAL_CAPT_MAX_STREAMS));
            hObj->pInstObj = pInstObj;
            break;
        }
    }

    return (hObj);
}

static int32_t calCoreCaptRegisterIsr(Cal_CoreCaptCalHndlObj_t *pHndlObj)
{
    int32_t                   rtnVal = FVID2_SOK;
    uint32_t                  i;
    Cal_EmEventGroup_t         eventGroup;
    Cal_CoreCaptInstObj_t  *pInstObj;
    Cal_CoreCaptOpenParams_t *pOpenParams;

    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));

    /* Assumes open parameters are already copied to handle object */
    pOpenParams = &pHndlObj->coreOpenPrms;
    GT_assert(CalTrace, (NULL != pOpenParams));

    eventGroup = CAL_EM_EG_CAL;

    /* Register DMA write start and DMA write end interrupts */
    for(i = 0; ((i < pOpenParams->numStreams) && (FVID2_SOK == rtnVal));
        i++)
    {
        if(CAL_CORE_STREAM_MODE_CAL_WRITE == pHndlObj->streamMode[i])
        {
            pHndlObj->emDmaStartHndl[i] = Cal_emRegister(
                pInstObj->initPrms.irqNum,
                eventGroup,
                &pHndlObj->dmaStartEvent[i],
                1U,
                CAL_EM_PRIORITY4,
                calCoreCaptDmaStartCb,
                (void *) pHndlObj);
            if(NULL == pHndlObj->emDmaStartHndl[i])
            {
                rtnVal = FVID2_EFAIL;
            }

            if(TRUE == pInstObj->useDmaEndIntr)
            {
                pHndlObj->emDmaCmpltHndl[i] = Cal_emRegister(
                    pInstObj->initPrms.irqNum,
                    eventGroup,
                    &pHndlObj->dmaCmpltEvent[i],
                    1U,
                    CAL_EM_PRIORITY4,
                    calCoreCaptDmaCmpltCb,
                    (void *) pHndlObj);
                if(NULL == pHndlObj->emDmaCmpltHndl[i])
                {
                    rtnVal = FVID2_EFAIL;
                }
            }
        }
    }

    return (rtnVal);
}


static int32_t calCoreCaptSetCalConfig(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                    const Cal_Cfg_t       *pCfg)
{
    /* Static to minimize stack requirement, config computed by this function
        and supplied to local function to be applied. */
    static Cal_HalCfg_t    halCfg;
    int32_t retVal = FVID2_SOK;
    uint32_t i, strmId, cmplxIoId;
    Cal_CoreCaptInstObj_t *pInstObj = NULL;

    /* Check for NULL done by the caller, just assert */
    GT_assert(CalTrace, (NULL != pHndlObj));
    GT_assert(CalTrace, (NULL != pCfg));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));

    /* Steps
        . Get the current config for all the streams that were created
        . Updated the streams, for which config is given
        . For non-cal streams, nothing much to do ignore
        . For CAL streams, construct the config required as understood by core
        . Update the config for all the streams
        */
    memcpy((void *) (&halCfg), (const void *) (&pInstObj->calCfg),
                    sizeof(Cal_HalCfg_t));
    halCfg.numCPortId = 0;

    for (i = 0;i < pCfg->numStream; i ++)
    {
        cmplxIoId = pCfg->cmplxIoId[i];
        if ((cmplxIoId >= CAL_CAPT_MAX_CMPLXIO_INST) ||
            ((uint32_t)(TRUE) !=
                pHndlObj->coreOpenPrms.isCmplxIoCfgValid[cmplxIoId]))
        {
            retVal = FVID2_EINVALID_PARAMS;
            break;
        }
    }

    for(i = 0; ((i < pCfg->numStream) && (FVID2_SOK == retVal)); i++)
    {
        retVal = FVID2_EINVALID_PARAMS;
        strmId = pCfg->streamId[i];
        if(strmId < CAL_CAPT_MAX_STREAMS)
        {
            /* In case of OTF, we could have streams that need not go to ISP
                for processing. It could directly written to memory via CAL
                write */

            if((CAL_CORE_STREAM_MODE_CAL_WRITE ==
                pHndlObj->streamMode[strmId]) ||
               (CAL_CORE_STREAM_MODE_CAL == pHndlObj->streamMode[strmId]))
            {
                GT_assert(CalTrace, (CAL_RM_RES_CAL_INVALID !=
                                               pHndlObj->allocatedRes[strmId].
                                               cport));

                retVal = FVID2_SOK;
                halCfg.cportId[strmId] = pHndlObj->allocatedRes[strmId].cport;

                halCfg.streamFmt[strmId].width      = pCfg->inFmt[i].width;
                halCfg.streamFmt[strmId].height     = pCfg->inFmt[i].height;
                halCfg.streamFmt[strmId].pitch[0]   = pCfg->inFmt[i].pitch[0];
                halCfg.streamFmt[strmId].pitch[1U]  = pCfg->inFmt[i].pitch[1U];
                halCfg.streamFmt[strmId].pitch[2U]  = pCfg->inFmt[i].pitch[2U];
                halCfg.streamFmt[strmId].ccsFormat        = pCfg->inFmt[i].ccsFormat;
                halCfg.streamFmt[strmId].dataFormat = pCfg->inFmt[i].dataFormat;

                if((CAL_CORE_CAPT_IF_CSI2 == pHndlObj->coreOpenPrms.captIf)
                   && (CAL_CSI2_DISABLE_CONTEXT !=
                       pCfg->csi2DataFormat[i]))
                {
                    halCfg.isCsi2BasedCapture[strmId] = (uint32_t)TRUE;
                }
                else
                {
                    halCfg.isCsi2BasedCapture[strmId] = (uint32_t)FALSE;
                }

                halCfg.stream[strmId]         = pCfg->streamType[i];
                halCfg.csiDataType[strmId]    = pCfg->csi2DataFormat[i];
                halCfg.virtualChanNum[strmId] = pCfg->csi2VirtualChanNo[i];
                if(CAL_CORE_CAPT_IF_CSI2 == pHndlObj->coreOpenPrms.captIf)
                {
                    pInstObj->streamIdToVcMap[strmId] =
                        halCfg.virtualChanNum[strmId];
                    pInstObj->streamIdToPpiMap[strmId] =
                        pCfg->cmplxIoId[i];
                }

                halCfg.isPixProcCfgValid[strmId] = pCfg->isPixProcCfgValid[i];
                memcpy((void *) (&halCfg.pixProcCfg[strmId]),
                                (const void *) (&pCfg->pixProcCfg[i]),
                                sizeof(Cal_PixProc_t));

                halCfg.isBysOutCfgValid[strmId] = pCfg->isBysOutCfgValid[i];
                memcpy((void *) (&halCfg.bysOutCfg[strmId]),
                                (const void *) (&pCfg->bysOutCfg[i]),
                                sizeof(Cal_BysOut_t));

                halCfg.isBysInCfgValid[strmId] = halCfg.bysInEnable[strmId] =
                                                     pCfg->bysInEnable[i];
                if((uint32_t)TRUE == halCfg.isBysInCfgValid[strmId])
                {
                    pInstObj->streamIdToVcMap[strmId] = CAL_BYS_IN_POSITION;
                }

                halCfg.isVportCfgValid[strmId] = pCfg->isVportCfgValid[i];
                memcpy((void *) (&halCfg.vportCfg[strmId]),
                                (const void *) (&pCfg->vportCfg[i]),
                                sizeof(Cal_VPort_t));

                halCfg.writeToMem[strmId] = pCfg->writeToMem[i];

                halCfg.numCPortId++;
            }
            else if(CAL_CORE_STREAM_MODE_ISP == pHndlObj->streamMode[strmId])
            {
                retVal = FVID2_SOK;
            }
            else
            {
                break;
            }
        }
    }

    if(FVID2_SOK == retVal)
    {
        retVal = calCoreCaptSetParams((Cal_CoreHandle)pHndlObj, &halCfg, NULL);
    }

    return(retVal);
}

static int32_t calCoreCaptPrimeStartCal(Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                     void                       *argsNotUsed)
{
    int32_t  retVal = FVID2_SOK;
    Cal_CoreFrame           *newFrame = NULL;
    Cal_CoreCaptInstObj_t *pInstObj = NULL;
    Cal_HalDmaVcCfg_t   *pDmaVcCfg  = NULL;
    uint32_t i;
    uintptr_t cookie;

    /* Check for NULL done by the caller, just assert */
    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));
    pDmaVcCfg = &pHndlObj->calDmaVcCfg;
    GT_assert(CalTrace, (NULL != pDmaVcCfg));

    /* . Only for streams that CAL / ISP would write
        .. Request a frame
        .. Program the frame
       . Start CAL
        */
    for(i = 0; ((i < pHndlObj->numStreams) && (FVID2_SOK == retVal)); i++)
    {
        if((CAL_CORE_STREAM_MODE_CAL_WRITE == pHndlObj->streamMode[i]) ||
           ((CAL_CORE_STREAM_MODE_ISP == pHndlObj->streamMode[i])))
        {
            newFrame = pHndlObj->openPrms.reqFrmCb(pHndlObj->openPrms.drvData,
                                                   i, 0x0);
            if(NULL == newFrame)
            {
                retVal = FVID2_EAGAIN;
            }
            if(FVID2_SOK == retVal)
            {
                retVal = calCoreCaptProgBuf(pHndlObj, newFrame, (uint32_t) FALSE);
            }
            if(CAL_CORE_STREAM_MODE_CAL_WRITE == pHndlObj->streamMode[i])
            {
                pDmaVcCfg->wrDmaCfg[i].mode    = CAL_HAL_DMA_WR_CONST;
                pHndlObj->firstDmaStartIntr[i] = (uint32_t)TRUE;
#ifdef PLATFORM_SIM
                pDmaVcCfg->wrDmaCfg[i].mode = CAL_HAL_DMA_WR_CONST;
#endif

                cookie = HwiP_disable();
                if(NULL != pHndlObj->emDmaStartHndl[i])
                {
                    retVal = Cal_emEnable(pHndlObj->emDmaStartHndl[i]);
                }
                if((NULL != pHndlObj->emDmaCmpltHndl[i]) &&
                    (FVID2_SOK == retVal))
                {
                    retVal = Cal_emEnable(pHndlObj->emDmaCmpltHndl[i]);
                }
                if((TRUE == pHndlObj->isSubFrameCfgValid) &&
                   (FVID2_SOK == retVal))
                {
                    retVal = Cal_emEnable(pHndlObj->emSubFrmCmpltHndl);
                }
                HwiP_restore(cookie);
            }
        }
    }

    if(FVID2_SOK == retVal)
    {
        retVal = Cal_halCaptureStart(pInstObj->calHalHandle, pDmaVcCfg);
    }

    return (retVal);
}

static void calCoreCaptErrCb(const uint32_t *event,
                           uint32_t        numEvents,
                           void         *arg)
{
    /* The top level CAL interrupt is the same and same priority, so this
        ISR WILL not be pre-empted. Hence its OK to have local static */
    static uint32_t          localEvent[CAL_CAPT_MAX_ERROR_INTERRUPTS];
    uint32_t                 idx, vc;
    int32_t                  retVal;
    Cal_CoreCaptErrObj_t  *pErrObj = (Cal_CoreCaptErrObj_t *) arg;

    if(((NULL == pErrObj) || (NULL == event)) || (NULL == pErrObj->pInstObj) ||
       (CAL_CAPT_MAX_ERROR_INTERRUPTS <= numEvents))
    {
        retVal = FVID2_EFAIL;
    }
    else
    {
        retVal = FVID2_SOK;
    }

    for(idx = 0U; ((idx < numEvents) && (FVID2_SOK == retVal)); idx++)
    {
        localEvent[idx] = event[idx];
        if(CAL_CSI2_PPI_CMPLXIO_FIFO_OVR == localEvent[idx])
        {
            /* All virtual channels requires to know about this overflow
                as we cannot determine which VC caused this */
            for(vc = 0U; vc < CAL_MAX_VIRTUAL_CHAN; vc++)
            {
                pErrObj->fifoOverFlow[vc] = (uint32_t)TRUE;
            }
        }
        else if(CAL_CSI2_PPI_CMPLXIO_ECC_NO_CORRECTION == localEvent[idx])
        {
            for(vc = 0U; vc < CAL_MAX_VIRTUAL_CHAN; vc++)
            {
                pErrObj->eccCouldNotCorrect[vc] = (uint32_t)TRUE;
            }
        }
        else if(CAL_EM_EVENT_BYSIN_OVR == localEvent[idx])
        {
            pErrObj->fifoOverFlow[CAL_BYS_IN_POSITION] = (uint32_t)TRUE;
            localEvent[idx] = CAL_BYSIN_OVR;
        }
        else
        {
            if(((CAL_CSI2_PPI_VC_CRC_MISMATCH_VC1 == localEvent[idx]) ||
                (CAL_CSI2_PPI_VC_CRC_MISMATCH_VC2 == localEvent[idx])) ||
               ((CAL_CSI2_PPI_VC_CRC_MISMATCH_VC3 == localEvent[idx]) ||
                (CAL_CSI2_PPI_VC_CRC_MISMATCH_VC4 == localEvent[idx])))
            {
                vc = (localEvent[idx] - CAL_CSI2_PPI_VC_CRC_MISMATCH_VC1)
                     / 8U;
                pErrObj->crcMisMatch[vc] = (uint32_t)TRUE;
            }

            if(((CAL_CSI2_PPI_VC_ECC_CORRECTION_VC1 == localEvent[idx]) ||
                (CAL_CSI2_PPI_VC_ECC_CORRECTION_VC2 == localEvent[idx])) ||
               ((CAL_CSI2_PPI_VC_ECC_CORRECTION_VC3 == localEvent[idx]) ||
                (CAL_CSI2_PPI_VC_ECC_CORRECTION_VC4 == localEvent[idx])))
            {
                vc = (localEvent[idx] - CAL_CSI2_PPI_VC_ECC_CORRECTION_VC1)
                     / 8U;
                pErrObj->eccCorrected[vc] = (uint32_t)TRUE;
            }
        }
    }

    if(FVID2_SOK == retVal)
    {
        if(NULL != pErrObj->errCfg.appCb)
        {
            pErrObj->errCfg.appCb(&localEvent[0U], numEvents,
                                   pErrObj->errCfg.pAppCbArgs);
        }
    }
    return;
}

static int32_t calCoreCaptCfgSubEof(
                                Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                const Cal_CoreCaptSubFrameCfg_t *pSubFrmCfg)
{
    uint32_t streamId, idx, xLineIntCnt, cportId;
    int32_t retVal = FVID2_EBADARGS;
    Cal_CoreCaptInstObj_t *pInstObj = pHndlObj->pInstObj;

    /*  Steps
        1. Error Checks
        2. Set the CAL for Line event notification
        3. Register Line event and end-of-frame interrupts handlers
    */
    /* Step 1. Error Checks */
    /*
        1. Ensure valid stream id
        2. Ensure x line interrupt is being enabled for only 1 vc/stream
    */
    if ((NULL != pSubFrmCfg->appCb) &&
        (CAL_CAPT_MAX_STREAMS > pSubFrmCfg->numStream))
    {
        retVal = FVID2_SOK;
        xLineIntCnt = 0U;
        for (idx = 0U;
            ((idx < pSubFrmCfg->numStream) && (FVID2_SOK == retVal)); idx++)
        {
            streamId = pSubFrmCfg->streamId[idx];
            if (CAL_CAPT_MAX_STREAMS <= streamId)
            {
                retVal = FVID2_EBADARGS;
            }
            else if (CAL_CORE_STREAM_MODE_CAL_WRITE !=
                                                pHndlObj->streamMode[streamId])
            {
                retVal = FVID2_EBADARGS;
            }
            else
            {
                if (0U != pSubFrmCfg->notifyAfterFirstXLines[idx])
                {
                    xLineIntCnt++;
                    if (pInstObj->calCfg.streamFmt[streamId].height <
                            pSubFrmCfg->notifyAfterFirstXLines[idx])
                    {
                        retVal = FVID2_EBADARGS;
                    }

                    if (CAL_CAPT_MAX_X_LINE_MONITOR_CNT < xLineIntCnt)
                    {
                        retVal = FVID2_EBADARGS;
                    }
                }
            }
        }
    }
    /* Step 2. Set the CAL for Line event notification */
    if (FVID2_SOK == retVal)
    {
        pInstObj->lineEventCfg.numCPortId = 0U;
        for (idx = 0U;
            ((idx < pSubFrmCfg->numStream) && (FVID2_SOK == retVal)); idx++)
        {
            xLineIntCnt = pSubFrmCfg->notifyAfterFirstXLines[idx];
            if (0U != xLineIntCnt)
            {
                streamId = pSubFrmCfg->streamId[idx];
                cportId = pHndlObj->allocatedRes[streamId].cport;

                /* Always, 0, as we can enable line event only for 1 channel
                    and it should be present at first entry */
                pInstObj->lineEventCfg.cportId[0U] = cportId;

                pInstObj->lineEventCfg.lineNumber[cportId] = xLineIntCnt;
                pInstObj->lineEventCfg.numCPortId++;
            }
        }

        retVal = Cal_halControl(
                        pInstObj->calHalHandle,
                        IOCTL_CAL_HAL_LINE_EVENT_CFG,
                        (void *) &pInstObj->lineEventCfg, NULL);
    }
    /* Step 3. Register Line event and end-of-frame interrupts handlers */
    if (FVID2_SOK == retVal)
    {
        retVal = calCoreCaptRegisterSubEofIsr(pHndlObj, pSubFrmCfg);
        if (FVID2_SOK == retVal)
        {
            memcpy((void *) (&pHndlObj->subFrameCfg),
                            (const void *) (pSubFrmCfg),
                            sizeof(Cal_CoreCaptSubFrameCfg_t));
        }
    }

    return (retVal);
}

static int32_t calCoreCaptRegisterSubEofIsr(
                                Cal_CoreCaptCalHndlObj_t *pHndlObj,
                                const Cal_CoreCaptSubFrameCfg_t *pSubFrmCfg)
{
    uint32_t streamId, idx;
    int32_t retVal = FVID2_SOK;
    Cal_CoreCaptInstObj_t *pInstObj = NULL;
    uint32_t lineNumEvent = (uint32_t) CAL_EM_EVENT_LINE_NUMBER;
    /*  Register ISR's and don't yet enable them (enable at start) */
    pInstObj = pHndlObj->pInstObj;

    for (idx = 0U;
        ((idx < pSubFrmCfg->numStream) && (FVID2_SOK == retVal)); idx++)
    {
        streamId = pSubFrmCfg->streamId[idx];

        if (TRUE == pSubFrmCfg->notifyAfterEndOfFrame[idx])
        {
            pHndlObj->emDmaCmpltHndl[streamId] = Cal_emRegister(
                pInstObj->initPrms.irqNum,
                CAL_EM_EG_CAL,
                &pHndlObj->dmaCmpltEvent[streamId],
                1U,
                CAL_EM_PRIORITY4,
                calCoreCaptDmaCmpltCb,
                (void *) pHndlObj);

            if(NULL == pHndlObj->emDmaCmpltHndl[streamId])
            {
                retVal = FVID2_EFAIL;
            }
        }

        /* Need not check if multiple streams are enabled for X line
            callback, as the caller has already checked this */
        if (0U != pSubFrmCfg->notifyAfterFirstXLines[idx])
        {
            pHndlObj->emSubFrmCmpltHndl = Cal_emRegister(
                pInstObj->initPrms.irqNum,
                CAL_EM_EG_CAL,
                &lineNumEvent,
                1U,
                CAL_EM_PRIORITY4,
                calCoreCaptXlineCmpltCb,
                (void *) pHndlObj);
            if(NULL == pHndlObj->emSubFrmCmpltHndl)
            {
                retVal = FVID2_EFAIL;
            }
            else
            {
                pHndlObj->isSubFrameCfgValid = TRUE;
            }
        }

    }
    if (FVID2_SOK == retVal)
    {
        pHndlObj->subFrameCfg.appCb = pSubFrmCfg->appCb;
        pHndlObj->subFrameCfg.pAppCbArgs = pSubFrmCfg->pAppCbArgs;
    }

    return retVal;
}

static int32_t calCoreCaptRegisterErrorIsr(const Cal_CoreCaptCalHndlObj_t
                                                                   *pHndlObj,
                                         const Cal_ErrorCfg_t *pErrCfg)
{
    uint32_t idx, cmplxIoId;
    int32_t  rtnVal = FVID2_SOK;
    Cal_CoreCaptInstObj_t *pInstObj;
    Cal_CoreCaptErrObj_t *pErrObj;

    GT_assert(CalTrace, (NULL != pHndlObj));
    pInstObj = pHndlObj->pInstObj;
    GT_assert(CalTrace, (NULL != pInstObj));
    GT_assert(CalTrace, (NULL != pErrCfg));

    /*
        . Check for errors in the configuration, including double registration
        . Register ISR
    */
    if ((CAL_CAPT_MAX_ERROR_INTERRUPTS <= pErrCfg->numErrorsToMonitor) ||
        (CAL_CAPT_MAX_CMPLXIO_INST <= pErrCfg->cmplxIoId))
    {
        rtnVal = FVID2_EBADARGS;
    }
    else
    {
        cmplxIoId = pErrCfg->cmplxIoId;

        /* Check for multiple registration and valid errors */
        if(NULL != pInstObj->errObj[cmplxIoId].emErrorHndl)
        {
            rtnVal = FVID2_EDEVICE_INUSE;
        }
    }

    for(idx = 0U; ((idx < pErrCfg->numErrorsToMonitor) &&
                   (FVID2_SOK == rtnVal)); idx++)
    {
        if((CAL_CSI2_PPI_CMPLXIO_ERRSOTHS1 == pErrCfg->errSrc[idx]) ||
           (CAL_CSI2_FORCE_INT == pErrCfg->errSrc[idx]))
        {
            rtnVal = FVID2_EOUT_OF_RANGE;
        }
    }

    if(FVID2_SOK == rtnVal)
    {
        pErrObj = &pInstObj->errObj[cmplxIoId];

        /* Initialize Error Object */
        pErrObj->cmplxIoId = cmplxIoId;
        pErrObj->pInstObj = pInstObj;

        memcpy((void *) (&pErrObj->errCfg),
                        (const void *) (pErrCfg),
                        sizeof(Cal_ErrorCfg_t));
        for(idx = 0U; idx < pErrObj->errCfg.numErrorsToMonitor; idx++)
        {
            if(CAL_BYSIN_OVR == pErrObj->errCfg.errSrc[idx])
            {
                pErrObj->errCfg.errSrc[idx] =
                    (Cal_ErrorSource_t)CAL_EM_EVENT_BYSIN_OVR;
            }
        }
        pErrObj->emErrorHndl = Cal_emRegister(
            pInstObj->initPrms.irqNum,
            CAL_EM_EG_CAL_PPI0_CSI2,
            (const uint32_t *)
            &pErrObj->errCfg.errSrc[0U],
            pErrObj->errCfg.numErrorsToMonitor,
            CAL_EM_PRIORITY1,
            calCoreCaptErrCb,
            (void *) pErrObj);
        if(NULL == pErrObj->emErrorHndl)
        {
            rtnVal = FVID2_EFAIL;
        }
    }
    return (rtnVal);
}

