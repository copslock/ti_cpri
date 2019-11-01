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
 *  \file CaptureCal_main.h
 *
 *  \brief Defines various strcutures / defines needed by capture demo app.
 *
 */

#ifndef CAPTURE_CAL_MAIN_H_
#define CAPTURE_CAL_MAIN_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* This is needed for vsnprintf */
#include <stdarg.h>

#ifndef BARE_METAL
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/hal/Cache.h>
#endif

#include <ti/drv/fvid2/fvid2.h>
#include <ti/drv/cal/cal.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>

#include <ti/drv/cal/examples/utils/calutils.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_NAME                        "CAL_CAPT_APP"

/**< Default run count in seconds in case of EVM else this is in frame count. */
#define CAPT_APP_RUN_COUNT              (1000U)

/**< Number of frames per stream */
#define CAPT_APP_FRAMES_PER_STREAM      (2U)

/**< End of frame event */
#define CAPT_APP_END_OF_FRAME_EVENT     (1U)
/**< Nth line frame event */
#define CAPT_APP_NTH_LINE_FRAME_EVENT   (0U)
/**< Stream for which Nth line frame events are to be enabled */
#define CAPT_APP_NTH_LINE_STREAM_ID     (0U)
/**< Control for which stream, the Nth line notification is enabled
        Valid range is from 0 to 3 */
#define FVID2_VID_SENSOR_MULDES_OV1063X_DRV (0x1001U)
/** \brief Multi Deserializer OV1063X video sensor driver ID. */

#define FVID2_VID_SENSOR_BYPASS_CSI2_DRV    (0x1002U)
/** \brief By Pass sensor configurations */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 * \brief Specify different config that would be used to cpature
 */
typedef struct appCaptCfg
{
    Char                   *testDescStr;
    /**< Test description. */
    uint32_t                  cfgId;
    /**< Config identifier */
    uint32_t                  useIt;
    /**< Capture with this config, could be bypassed, by set this to FALSE */
    Fvid2_VideoIfMode       interfaceType;
    /**< Either CSI2, LVDS, Parallel */
    uint32_t                interfacewidth;
    /**< 1,2,3,4 lanes for CSI2, 4 lane for LVDS, 8-16 bits for parallel */
    uint32_t                  numStreams;
    /**< Should be 1 for all sensor with exception of UB960/TIDA00262 in which
     *      case it can be upto 4. */
    uint32_t                  virtualChannel;
    /**< In case of CSI2 - specify the virtual channel */
    Cal_Csi2DataFormat inCsi2DataFormat;
    /**< Specify input data format */
    uint32_t                  inBpp;
    /**< Number of bits / pixel */
    uint32_t                  numFrames;
    /**< Specify number of frames to capture */
    uint32_t                  width;
    /**< Expected frame width that requires to be captured. If sensor streams
     *      frames with greater width, the extra pixels would not be written.
     *      If sensors streams frames with less line length, the received
     *      lines are written. */
    uint32_t                  height;
    /**< Expected frame height requires to be captured. If sensor streams
     *      frames with higher number of lines, the extra lines would not be
     *      written / captured.*/
    uint32_t                  pitch;
    /**< Associated sensor driver, that would stream */
    uint32_t                  sensorDriverId;
    uint32_t                  standard;
    /**< Resolution */
    uint32_t                  dataFormat;
    /**< Standard FVID2 resolution that sensor would be configured to stream */
    uint32_t                  bpp;
    /**< bits / pixel */
}appCaptCfg_t;

/**
 *  \brief Capture application object.
 */
typedef struct appCaptObj
{
    Cal_CaptOpenParams_t      calOpenPrms;
    /**< Cal create time parameters */
    Fvid2_VideoIfMode         videoIfMode;
    /**< CSI2, LVDS, Parallel etc... */
    uint32_t                  videoIfWidth;
    /**< Number of lanes in case CSI2 and number of bits in case of parallel */
    uint32_t                    numStream;
    /**< Number of channel in multi-ch case, must be 1 for single channel. */
    uint32_t                    virtualChannel;
    /**< Virtual channel of CSI2, if mode is CSI2 */
    Cal_Csi2DataFormat        dataFormat;
    /**< Data format of CSI2 stream */
    uint32_t                    instId;
    /**< Cal Instance ID. As defined in ti\drv\cal\include\cal_cfg.h*/
    Cal_CaptCreateParams      createPrms;
    /**< Create time parameters. */
    Cal_CaptCreateStatus      createStatus;
    /**< Create status returned by driver during Fvid2_create(). */
    Fvid2_Handle              drvHandle;
    /**< FVID2 capture driver handle. */
    Fvid2_CbParams            cbPrms;
    /**< Callback params. */

    Cal_Cfg_t                 cfg;
    /**< TODO currently done for 1 stream, update for mutiple streams */
    uint32_t                    numFramesToCapture;
    /**< Number of frames to receive for a given configuration */
    uint32_t                    maxWidth;
    /**< Max width in pixels - used for buffer allocation for all instance. */
    uint32_t                    maxHeight;
    /**< Max height in lines - used for buffer allocation for all instance. */

    appCaptCfg_t                testPrms;

    uint32_t                    totalFrmCount;
    /**< Count of all frames captured. */
    uint32_t                    totalCpuLoad;
    /**< Accumulated CPU load - added every frame. */
    uint32_t                    cpuLoadCount;
    /**< CPU load count used to get average CPU load - incremented every
     *   accumulation. */

    Fvid2_Frame               frames[CAPT_APP_FRAMES_PER_STREAM * \
                                     CAL_CAPT_MAX_STREAMS];
    /**< FVID2 Frames that will be used for capture. */
    Fvid2_SubFrameInfo        subFrameInfo[CAPT_APP_FRAMES_PER_STREAM * \
                                     CAL_CAPT_MAX_STREAMS];
    /**< Place holder to store sub-frameInfo */
    volatile uint32_t           rcvedFramesCount;
    /**< Received frames for a given config */
    uint32_t                    qCount[CAPT_APP_FRAMES_PER_STREAM][8];
    /**< Track Q frames */
    uint32_t                    dQCount[CAPT_APP_FRAMES_PER_STREAM][8];
    /**< Track DeQueued frames */
    uint32_t                    frameWithCrcErrorCnt;
    /**< Track the number of frames with CRC errors */
    uint32_t                    frameWithWarning;
    /**< Track the number of frames with warning, i.e. ecc corrected */
    uint32_t                    frameErrorCnt;
    /**< Other errors in frame */
    volatile uint32_t           sofIntCount;
    /**< Counts the number of Start Of Frame sync interrupts received
            CSI2 i/f */
    volatile uint32_t           crcErrIntCnt;
    /**< Counts the number of CRC failure on CSI2 i/f */
    volatile uint32_t           eccErrIntCnt;
    /**< Counts the number of ECC failure on CSI2 i/f */
    volatile uint32_t           unExpectedIntCnt;
    /**< in case we receive un expected interrupt */

    uint32_t                    frameEventTrack;
    /**< Flag to track frame event notifications */
    Fvid2_SubFrameInfo        nLineInfo[CAPT_APP_FRAMES_PER_STREAM];
    /**< Stores first CAPT_APP_FRAMES_PER_STREAM Nth line notification */
    uint32_t                    nLineEventTs[CAPT_APP_FRAMES_PER_STREAM];
    /**< Stores first CAPT_APP_FRAMES_PER_STREAM time stamp for Nth line */
    uint32_t                    nLineEventIdx;
    /**< Index into the above arrays */

    uint32_t                    eofEventFrameAddr[CAPT_APP_FRAMES_PER_STREAM][4];
    /**< Stores first CAPT_APP_FRAMES_PER_STREAM frame address returned at EOF */
    uint32_t                    eofEventTs[CAPT_APP_FRAMES_PER_STREAM][4];
    /**< Stores first CAPT_APP_FRAMES_PER_STREAM time stamp at EOF */
    uint32_t                    eofEventIdx[4];
    /**< Index into the above arrays */

}appCaptObj_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CAPTURE_CAL_MAIN_H_ */
