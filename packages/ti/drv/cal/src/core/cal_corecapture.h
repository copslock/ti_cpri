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
 *  \file cal_corecapture.h
 *
 *  \brief  Provides interfaces to control / configure CAL based capture.
 *              Methods to configure capture via CAL OR LVDS.
 *
 *          With each instance of capture opened, one or more stream can be
 *          captured.
 */

#ifndef CAL_CORECAPTURE_H_
#define CAL_CORECAPTURE_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/core/cal_core.h>
#include <ti/drv/cal/src/core/cal_evtmgr.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Maximum number of opens supported per instance of core */
#define CAL_CORE_CAPT_MAX_OPEN             (0x1U)

/** \brief IOCTL base address for the common IOCTLs listed below. TODO TBD
 *      Should derive from a common base */
#define CAL_CORE_CAPT_BASE_IOCTL           (0U)

/** \brief Configure CAL based on the config supplied by a pointer to instance
 *          of type Cal_Cfg_t as argument.
 */
#define CAL_CORE_CAPT_SET_PARAMS       (CAL_CORE_CAPT_BASE_IOCTL + 1U)

/** \brief Enable/Disable error in CAL */
#define CAL_CORE_CAPT_SET_ERR_PRMS     (CAL_CORE_CAPT_BASE_IOCTL + 2U)

/** \brief Enable CB on reception of Nth line and end of frame */
#define CAL_CORE_CAPT_SET_SUB_FRM_PRMS (CAL_CORE_CAPT_BASE_IOCTL + 3U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  struct Cal_CoreCaptInterface
 *  \brief Different physical interface supported.
 */
typedef enum Cal_CoreCaptInterface
{
    CAL_CORE_CAPT_IF_CSI2    = 0x0,
    /**< CSI2 Interface */
    CAL_CORE_CAPT_IF_LVDS    = 0x1,
    /**< LVDS Interface */
    CAL_CORE_CAPT_IF_CPI     = 0x2,
    /**< CPI / Parallel interface */
    CAL_CORE_CAPT_IF_MAX     = 0x3,
    /**< End Marker */
    CAL_CORE_CAPT_IF_FORCE_INT = 0x7FFFFFFF
                                    /**< This will ensure enum is not packed,
                                     *      will always be contained in int */
} Cal_CoreCaptInterface_t;

/**
 *  struct Cal_CoreCaptInitParams
 *  \brief Different modes of capture supported. LVDS & CAL based capture cannot
 *          not co-exist.
 */
typedef struct Cal_CoreCaptInitParams
{
    Cal_CaptInstId_t         instId;
    /**< Capture core instance ID, CAL OR LVDS */
    const Cal_HalPlatformData_t *halPlatformData;
    /**< HAL information */
    Cal_EmInstId_t              irqNum;
    /**< Interrupt number associated with this instance.
     *      The design / implementation, assumes single IRQ is used, if multi
     *      ple IRQ are used, we would require to guard ISR with interrupt
     *      enable / disable. */
    uint32_t                   subModules;
    /**< CAL Sub-modules required to be enabled for this instance */
    void                      *arg;
    /**< Not used for now, reserved */
} Cal_CoreCaptInitParams_t;

/**
 *  \brief Open Parameters.
 */
typedef struct Cal_CoreCaptOpenParams
{
    Cal_CoreCaptInterface_t captIf;
    /**< Type of interface
     *   Some interfaces are possible only in few instances,
     *   like CSI2 is possible only on CALA/B instances,
     #Cal_CoreCaptInterface_t  */
    uint32_t                numStreams;
    /**< Define number of streams that is required. */
    uint32_t                  subModules[CAL_CAPT_MAX_STREAMS];
    /**< Identify the modules required for each stream.
     *  e.g. modulesReq[0] =
     *      CAL_CAPT_INST_ID_SUB_PPI_ID | CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID
     *      resources are defined in Cal_CaptSubModuleId_t
     *  Only 2 modules required, 1 to decode CSI2 and write received data */
    uint32_t                isCmplxIoCfgValid[CSL_CAL_CMPLXIO_CNT];
    /**< Specify if the complex IO configurations should be applied or the
     *      default config should be applied.
     *  TRUE - Complex IO configuration provided below is valid and will be
     *          applied.
     *  FALSE - Default complex IO configuration would be applied. */
    Cal_CmplxIoCfg_t   cmplxIoCfg[CSL_CAL_CMPLXIO_CNT];
    /**< Specify the CSI2 lanes configurations */
    uint32_t                csi2PhyClock[CSL_CAL_CMPLXIO_CNT];
    /**< Specify the CSI2 PHY clock in MHz. e.g. if 400 MHz is the clock
     *      \code csi2PhyClock = 400; */
    void                   *arg;
    /**< Not Used as of now */
} Cal_CoreCaptOpenParams_t;

/**
 *  \brief Details returned by open operation
 */
typedef struct Cal_CoreCaptOpenRetParams
{
    int32_t  streamAllocError;
    /**< In case there no free streams available, status would be set to
     *      FVID2_EALLOC this variable will hold FVID2_EALLOC */
    uint32_t numStreamsAlloc;
    /**< Returns the number of streams allocated */
    uint32_t   cportIdAlloc[CAL_CAPT_MAX_STREAMS];
    /**< Instances of the sub modules allocated */
    uint32_t   isStreamOpt[CAL_CAPT_MAX_STREAMS];
    /**< Indicates streams that do not result in a write to memory.
     *      i.e. consumed internally by the CAL and for which memory need not
     *              be allocated. */
    void    *arg;
    /**< Not used as of now */
} Cal_CoreCaptOpenRetParams_t;

/**
 *  struct Cal_CoreCaptSubFrameCfg
 *  \brief CAL could be configured to notify application on end of frame or on
 *          reception of first X lines.
 *
 *  Limitation on first x lines callback : Currently this can be enabled for
 *      one stream and stream should be of type CAL_TAG_PIX_DATA
 *
 */
typedef struct Cal_CoreCaptSubFrameCfg
{
    uint32_t numStream;
    /**< Number of streams, which requires to be monitored for end of frame,
            and or first X lines */
    uint32_t streamId[CAL_CAPT_MAX_STREAMS];
    /**< Specify the stream ID, which requires to be monitored */
    uint32_t notifyAfterFirstXLines[CAL_CAPT_MAX_STREAMS];
    /**< Will call the application provided callback, after reception of
        number of lines specified here.
        Restriction :
            This can be supported for only CAL_CAPT_MAX_X_LINE_MONITOR_CNT
            virtual channel */
    uint32_t notifyAfterEndOfFrame[CAL_CAPT_MAX_STREAMS];
    /**< Will call the application provided callback, after reception of
        End Of Frame short packet */
    Cal_CoreSubFrameCbFxn appCb;
    /**< Application callback */
    void                 *pAppCbArgs;
    /**< Argument that would be passed when appCb is called. */
    void                 *pAdditionalArgs;
    /**< Not used for now - should be set to NULL */
} Cal_CoreCaptSubFrameCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Function to initialize capture via CAL. Performs h/w initializations
 *          if required.
 *
 *  \param numInst          number of instances to be initialized
 *  \param initParams       Init parameters, containing HAL
 *                          initialization parameters.
 *  \param arg              not used as of now
 *
 *  \return                 FVID2_SOK: on successful completion, otherwise
 *                          appropriate error code.
 */
int32_t Cal_coreCaptInit(
    uint32_t                  numInst,
    const Cal_CoreCaptInitParams_t *initParams,
    void                           *arg);

/**
 *  \brief DeInitializes Capture instances.
 *
 *  \param arg            Not used as of now.
 *
 *  \return                 FVID2_SOK: on successful completion, otherwise
 *                          appropriate error code.
 */
int32_t Cal_coreCaptDeInit(void *arg);

/**
 *  \brief Returns the pointer to core function pointer table.
 *
 *  \return  Returns the pointer to core function pointer table. NULL on errors.
 */
const Cal_CoreOps *Cal_coreCaptGetCoreOps(void);

/**
 *  \brief Returns the pointer to the core instance object
 *
 *  \return  A Valid pointer on success else a NULL pointer.
 */
Cal_CoreInst Cal_coreCaptGetCoreInstObj(Cal_CaptInstId_t inst);

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CAL_CORECAPTURE_H_ */
