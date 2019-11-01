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
 *  \defgroup CAL_DRV_CAPTURE_API Capture API
 *
 *  This modules define APIs to capture video data using video ports in CAL.
 *  This module can be used for single channel capture as well as
 *  multi-channel capture.
 *
 *  Capture driver supports the following FVID2 APIs
 *
 * - <b> Creating the driver </b> - Fvid2_create()
 *   <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>drvId</td>
 *      <td>\ref FVID2_CAL_CAPT_DRV_ID</td>
 *    </tr>
 *    <tr>
 *      <td>instanceId</td>
 *      <td>
 *          #Cal_CaptInstId<br>
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>createArgs</td>
 *      <td>
 *      Cal_CaptCreateParams *
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>createStatusArgs</td>
 *      <td>
 *      Cal_CaptCreateStatus *
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>cbParams</td>
 *      <td>
 *      Fvid2_CbParams *
 *
 *      When Fvid2_CbParams.cbFxn is set, Fvid2_CbParams.cbFxn
 *      gets called
 *
 *      </td>
 *    </tr>
 *  </table>
 *  #Fvid2_Handle returned by Fvid2_create() is used in subsequent FVID2
 *  APIs
 *
 * - <b> Deleting the driver </b> - Fvid2_delete()
 *  <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>handle</td>
 *      <td>Capture driver handle</td>
 *    </tr>
 *    <tr>
 *      <td>deleteArgs</td>
 *      <td>NOT USED, set to NULL</td>
 *    </tr>
 *  </table>
 *
 * - <b> Starting the driver </b> - Fvid2_start()
 *  <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>handle</td>
 *      <td>Capture driver handle</td>
 *    </tr>
 *    <tr>
 *      <td>cmdArgs</td>
 *      <td>NOT USED, set to NULL</td>
 *    </tr>
 *  </table>
 *
 * - <b> Stopping the driver </b> - Fvid2_stop()
 *  <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>handle</td>
 *      <td>Capture driver handle</td>
 *    </tr>
 *    <tr>
 *      <td>cmdArgs</td>
 *      <td>NOT USED, set to NULL</td>
 *    </tr>
 *  </table>
 *
 *  - <b> Controlling the driver </b> - Fvid2_control() <br>
 *  See \ref CAL_DRV_IOCTL_CAL_CAPTURE for the list of IOCTLs supported by the
 *  driver.
 *
 *  - <b> Getting captured frames from the driver </b> - Fvid2_dequeue()
 *  <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>handle</td>
 *      <td>Capture driver handle</td>
 *    </tr>
 *    <tr>
 *      <td>frameList</td>
 *      <td>
 *      [OUT] Fvid2_FrameList.numFrames returns the number of captured frames
 *  returned in this Fvid2_dequeue() invocation. <br>
 *      [OUT] Fvid2_FrameList.frames[0..Fvid2_FrameList.numFrames-1] are the
 *  captured Fvid2_Frame pointers to the captured frames. <br>
 *      [OUT] For each Fvid2_Frame, Fvid2_Frame.perFrameCfg points to
 *  Cal_CaptRtParams that was set during Fvid2_queue()
 *  This structure gets filled with the captured width and height information.
 *  When sub-frame mode is enabled this structure is filled for every
 *  sub-frame. The parameter capturedOutHeight shall be invalid for the first
 *  field/frame. From the second field/frame onwards this value shall hold the
 *  captured height of the previous field/frame. For the last sub-frame in the
 *  EOF ISR the parameter numOutLines is updated with the capturedHeight. <br>
 *      [OUT] FID for every field/frame is filled when Fvid2_dequeue() is
 *  called.
 *  When sub-frame mode is enabled the FID is filled after the first
 *  sub-frame is captured. For the first field/frame this value shall be
 *  invalid. <br>
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>streamId</td>
 *      <td>
 *      Value can be from 0 .. Cal_CaptCreateParams.numStream-1
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>timeout</td>
 *      <td>
 *      Must be BIOS_NO_WAIT
 *      </td>
 *    </tr>
 *  </table>
 *
 *  - <b> Releasing used frames back to the driver </b> - Fvid2_queue() <br>
 *  Also used to queue initial frame buffers to the driver, before calling
 *  Fvid2_start().
 *  <table border="1">
 *    <tr>
 *      <th>Parameter</th>
 *      <th>Value</th>
 *    </tr>
 *    <tr>
 *      <td>handle</td>
 *      <td>Capture driver handle</td>
 *    </tr>
 *    <tr>
 *      <td>frameList</td>
 *      <td>
 *      [IN] Fvid2_FrameList.numFrames sets the number of captured frames given
 *  back to driver in this Fvid2_queue() invocation. <br>
 *      [IN] Fvid2_FrameList.frames[0..Fvid2_FrameList.numFrames-1] are the
 *  captured Fvid2_Frame pointers that are being returned. <br>
 *      [IN] For each Fvid2_Frame, Fvid2_Frame.perFrameCfg points to
 *  Cal_CaptRtParams.
 *            This structure is filled and returned when Fvid2_dequeue() is
 *  called. <br>
 *      </td>
 *    </tr>
 *    <tr>
 *      <td>streamId</td>
 *      <td>
 *      Value can be from 0 .. Cal_CaptCreateParams.numStream-1 
 *      </td>
 *    </tr>
 *  </table>
 *
 *  Capture driver application flow is as follows
 *
 *  - <b> Init all required modules by calling respective module init functions
 *  </b>
 *  - <b> Create driver using Fvid2_create() </b>
 *  - <b> Set driver specific parameters using Fvid2_control() </b> <br>
 *        \ref IOCTL_CAL_CAPT_SET_PARAMS
 *  - <b> Prime buffers using Fvid2_queue() </b>
 *  - <b> Create, configure and start external video decoder or sensor
 *  driver</b>
 *  - <b> Start capture using Fvid2_start() </b>
 *  - <b> Dequeue captured frame and queue back used frames using
 *  Fvid2_dequeue() and Fvid2_queue() </b>
 *  - <b> Stop capture using Fvid2_stop() </b>
 *  - <b> Stop and delete external video decoder or sensor driver </b>
 *  - <b> Delete capture driver using Fvid2_delete() </b>
 *  - <b> De-Init modules by calling respective module de-init functions </b>
 *
 *  @{
 */

/**
 *  \file cal_drv.h
 *
 *  \brief CAL driver capture API.
 */

#ifndef CAL_DRV_H_
#define CAL_DRV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

/* None */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief CAL capture driver ID used at the time of FVID2 create. */
#define FVID2_CAL_CAPT_DRV_ID           (FVID2_CAL_DRV_BASE + 0x00000000U)

/**
 *  \name Capture max limits or defaults.
 *
 *  @{
 */

/**
 *  \brief Maximum channels that can be captured per port. The number of
 *  channels varies for a given interface. Please refer the interface
 *  specific interface headers
 */
#define CAL_CAPT_CH_PER_PORT_MAX        (16U)

/* @} */

/* Capture IOCTL's */

/**
 *  \addtogroup CAL_DRV_IOCTL_CAL_CAPTURE
 *  @{
 */

/**
 *  \brief Get capture status IOCTL.
 *
 *  This IOCTL can be used to get the capture status like number of frames
 *  queued/dequeued.
 *  Note: These counters will be reset either at the time of driver create or
 *  while starting the capture operation. See respective counter comments for
 *  details.
 *
 *  \param cmdArgs       [OUT] Cal_CaptStatus *
 *  \param cmdArgsStatus [OUT] NULL
 *
 *  \return FVID2_SOK on success, else failure
 *
 */
#define IOCTL_CAL_CAPT_GET_STATUS       (FVID2_CAL_DRV_IOCTL_BASE + 0x0000U)

/**
 *  \brief Get channel status
 *
 *  This IOCTL can be used to get status about a specific channel, stream.
 *
 *  This control can be done independently for each stream, channel.
 *
 *  \param cmdArgs       [IN]  const Cal_CaptChStatusArgs *
 *  \param cmdArgsStatus [OUT] Cal_CaptChStatus *
 *
 *  \return FVID2_SOK on success, else failure.
 */
#define IOCTL_CAL_CAPT_GET_CH_STATUS    (FVID2_CAL_DRV_IOCTL_BASE + 0x0001U)

/**
 *  \brief Flush capture driver and dequeue all frames including those that
 *  are not captured.
 *
 *  Note: This IOCTL is not yet implemented!!
 *
 *  This API can be called only when driver is in stopped state.
 *  Driver is in stopped state when,
 *  - FIVD2_stop() is called
 *  - Fvid2_create() is called and Fvid2_start() is not called.
 *
 *  This will return frames from all streams for a given capture handle.
 *  If this IOCTL is called with global handle then this will return
 *  frames from all capture handles.
 *
 *  Since maximum FVID2_MAX_FRAME_PTR frames can be returned at a time.
 *  This function should be called in a loop until Fvid2_FrameList.numFrames = 0
 *  is returned or the return value is FVID2_ENO_MORE_BUFFERS in order to get
 *  back all the frames from the capture driver.
 *
 *  In case capture is in running state this function will return error.
 *
 *  This IOCTL will return capture frames, non-captured frames, as well as
 *  all frames held inside the hardware.
 *
 *  \param cmdArgs       [IN]  NULL
 *  \param cmdArgsStatus [OUT] Fvid2_FrameList
 *
 *  \return FVID2_SOK on success, FVID2_ENO_MORE_BUFFERS if all buffers are
 *  returned, other error values on failure.
 */
#define IOCTL_CAL_CAPT_FLUSH            (FVID2_CAL_DRV_IOCTL_BASE + 0x0002U)
/**
 *  \brief Set CAL / ISP parameters.
 *
 *  This IOCTL should be used to configure the CAL to receive streams.
 *
 *  \attention Once the capture is started this IOCTL can't be invoked and
 *              if invoked, an error would be returned.
 *
 *  Once the capture is stopped, application could call this IOCTL to
 *  reconfigure.
 *
 *  \param cmd              [IN]  IOCTL_CAL_CAPT_SET_PARAMS
 *  \param cmdArgs          [IN]  const Cal_Cfg_t *
 *  \param cmdArgsStatus    [OUT] NULL
 *
 *  \return FVID2_SOK on success, else failure.
 */
#define IOCTL_CAL_CAPT_SET_PARAMS   (FVID2_CAL_DRV_IOCTL_BASE + 0x0003U)

/**
 *  \brief Enable error notifications
 *
 *  When receiving from CSI2 interface or parallel interface, CAL provides
 *      mechanisms to detect / correct errors. Please refer
 *      Cal_ErrorSource_t for supported errors.
 *  The errors are detected on instance basis i.e. these cannot be configured
 *      per handle.
 *
 *  \attention The callback provided by the applications, via this IOCTL, will
 *              called in interrupt context.
 *
 *  \attention The control command should be the last control command.
 *
 *  \param cmd              [IN]  IOCTL_CAL_CAPT_SET_ERR_PRMS
 *  \param cmdArgs          [IN]  const Cal_ErrorCfg_t *
 *  \param cmdArgsStatus    [OUT] NULL
 *
 *  \return FVID2_SOK on success, else failure.
 */
#define IOCTL_CAL_CAPT_SET_ERR_PRMS   (FVID2_CAL_DRV_IOCTL_BASE + 0x0004U)

/**
 *  \brief Enable callback on Nth line and endOfFrame
 *
 *  For any one virtual channel, a callback could be enabled/attached on
 *      reception of Nth line.
 *
 *  For all virtual channels, a callback could be enabled/attached on
 *      reception of End-Of-Frame.
 *
 *  \attention The callback provided by the applications, via this IOCTL, will
 *              called in interrupt context.
 *
 *
 *  \param cmd              [IN]  IOCTL_CAL_CAPT_SET_FRAME_EVENT_NOTIFY_PRMS
 *  \param cmdArgs          [IN]  const Cal_FrameEventNotifyCfg_t *
 *  \param cmdArgsStatus    [OUT] NULL
 *
 *  \return FVID2_SOK on success, else failure.
 */
#define IOCTL_CAL_CAPT_SET_FRAME_EVENT_NOTIFY_PRMS  \
                                        (FVID2_CAL_DRV_IOCTL_BASE + 0x0005U)
/* @} */

/**
 *  \brief Buffer capture mode.
 *          These modes depends on the underlying hardware. Not all capture
 *          interfaces can support all mode. Please refer interface specific
 *          header for expections.
 */
typedef enum
{
    CAL_CAPT_BCM_LAST_FRM_REPEAT,
    /**< In this mode the driver will keep capturing the data to the last
     *   queued buffer when there are no more buffers at the input queue.
     *   The driver will hold the last buffer with it till the application
     *   queues any new buffer or the capture is stopped. */
    CAL_CAPT_BCM_CIRCULAR_FRM_REPEAT,
    /**< In this mode the driver will keep reusing all the sets of buffer
     *   with it in a circular fashion.
     *   Application cannot get back any buffer from the driver when streaming
     *   is on and dequeue call will result in error. */
    CAL_CAPT_BCM_MAX
    /**< Should be the last value of this enumeration.
     *   Will be used by driver for validating the input parameters. */
} Cal_CaptBufferCaptMode;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Capture driver create arguments, used when calling Fvid2_create().
 *          All features may not be supported by all the interfaces.
 *          The exceptions will be called out in the inteface specific headers.
 */
typedef struct
{
    uint32_t videoIfMode;
    /**< [IN] Video capture mode. For valid values see #Fvid2_VideoIfMode. */
    uint32_t videoIfWidth;
    /**< [IN] Video interface mode. For valid values see #Fvid2_VideoIfWidth. */
    uint32_t bufCaptMode;
    /**< [IN] Buffer capture mode.
     *   For valid values see #Cal_CaptBufferCaptMode. */
    uint32_t chInQueueLength;
    /**< [IN] Maximum number of request/frame per channel that can be submitted
     *   for capture without having to dequeue the captured requests. */
    uint32_t numCh;
    /**< [IN] Number of channel for multi-channel modes,
     *   Must be 1 for FVID2_VIFM_SCH_* capture modes. */
    uint32_t numStream;
    /**< [IN] Number of streams to capture,
     *   MUST be <= CAL_CAPT_MAX_STREAMS. */

    uint32_t chNumMap[CAL_CAPT_MAX_STREAMS][CAL_CAPT_CH_PER_PORT_MAX];
    /**< [IN] Channel Number to assign to each channel and stream of this
     *   handle.
     *   This is used during Fvid2_queue(), Fvid2_dequeue().
     *   Channel number must be unique across the whole system.
     *   Users can use Cal_captMakeChNum() to generate a system unique
     *   channel number. */
    void  *pAdditionalArgs;
    /**< [IN] Used by some driver to hold additional information. Please refer
     *          the device specific include, on expected use. */
} Cal_CaptCreateParams;

/**
 *  \brief Capture driver create status. Returned after calling Fvid2_create().
 */
typedef struct
{
    int32_t retVal;
    /**< [OUT] Create status, FVID2_SOK on success, else failure. */
} Cal_CaptCreateStatus;

/**
 *  \brief Additional Open parameters required. Used to allocate sub-modules
 *          of CAL. The address of a instance of this structure should be
 *          specified.
 *  \code Cal_CaptCreateParams->pAdditionalArgs =
 *          (Cal_CaptOpenParams_t *)&cfg; \endcode
 */
typedef struct Cal_CaptOpenParams
{
    uint32_t            subModules[CAL_CAPT_MAX_STREAMS];
    /**< Identify the modules required for each stream.
        e.g. modulesReq[0] =
            CAL_RM_CAL_SUB_PPI_ID | CAL_RM_CAL_SUB_WR_DMA_ID resources are defined
            in Cal_CaptSubModuleId_t
        Only 2 modules required, 1 to decode CSI2 and write received data */
    uint32_t            isCmplxIoCfgValid[CAL_CAPT_MAX_CMPLXIO_INST];
    /**< Specify if the complex IO configurations should be applied or the
            default config should be applied.
        TRUE - Complex IO configuration provided below is valid and will be
                applied.
        FALSE - Default complex IO configuration would be applied. */
    Cal_CmplxIoCfg_t    cmplxIoCfg[CAL_CAPT_MAX_CMPLXIO_INST];
    /**< Specify the CSI2 lanes configurations */
    uint32_t            csi2PhyClock[CAL_CAPT_MAX_CMPLXIO_INST];
    /**< Specify the CSI2 PHY Clock in MHz. e.g. if 400 MHz is the clock
            \code csi2PhyClock = 400; \endcode
        \warning In cases where CSI2 interface is not used. Set this to 400. */
    void               *arg;
    /**< Not Used as of now */
} Cal_CaptOpenParams_t;

/**
 *  \brief Capture driver run-time parameters.
 *
 *  - This structure is returned by capture driver when Fvid2_dequeue()
 *    is called by application
 *  - NOTE:this structure is NOT filled by driver when TILED mode is used
 *  - This structure is returned as part of Fvid2_Frame.perFrameCfg
 *  - Application should make sure Fvid2_Frame.perFrameCfg is set to a valid
 *    Cal_CaptRtParams pointer when queue-ing the frame back to capture driver
 *    \code
 *    Cal_CaptRtParams rtParams;
 *
 *    pFrame->perFrameCfg = &rtParams;
 *    \endcode
 *  - Alternatively, user can pass back the same Fvid2_Frame pointer without
 *    modifying Fvid2_Frame.perFrameCfg
 */
typedef struct
{
    uint32_t capturedOutWidth;
    /**< [OUT] Captured data width in pixels. */
    uint32_t capturedOutHeight;
    /**< [OUT] Captured data height in lines. */
} Cal_CaptRtParams;

/**
 *  struct Cal_CaptStatus
 *  \brief Capture status structure used to get the current status.
 */
typedef struct
{
    uint32_t queueCount;
    /**< [OUT] Counter to keep track of how many requests are queued to the
     *   driver.
     *   Note: This counter will be reset at the time of driver create. */
    uint32_t dequeueCount;
    /**< [OUT] Counter to keep track of how many requests are dequeued from the
     *   driver.
     *   Note: This counter will be reset at the time of driver create. */
    uint32_t overflowCount;
    /**< [OUT] Counter to keep track of the occurrence of overflow error.
     *   Note: This counter will be reset at the time of driver create and
     *   during driver start. */
} Cal_CaptStatus;

/**
 *  \brief Capture Channel Get Status IOCTL input arguments
 */
typedef struct
{
    uint32_t chNum;
    /**< [IN] Channel number for which status is requested. */
    uint32_t frameInterval;
    /**< [IN] Expected interval in units of timer ticks between frames.
     *   Cal_CaptChStatus.isVideoDetected is FALSE if no frame is captured
     *   for a duration of 'frameInterval x 2', else
     *   Cal_CaptChStatus.isVideoDetected is TRUE. */
} Cal_CaptChStatusArgs;

/**
 *  \brief Capture Channel Get Status IOCTL result
 */
typedef struct
{
    uint32_t isVideoDetected;
    /**< [OUT] TRUE: Video detected at this channel,
     *         FALSE: Video not detected at this channel.
     *   Note: This flag is updated properly only when the capture is in
     *   progress as the time stamp comparision makes sense only then.
     *   Otherwise when the capture is stopped, this will always return as
     *   TRUE. */

    uint32_t queueCount;
    /**< [OUT] Counter to keep track of how many requests are queued
     *   to the driver. */
    uint32_t dequeueCount;
    /**< [OUT] Counter to keep track of how many requests are dequeued from the
     *   driver. */

    uint32_t captFrmCount;
    /**< [OUT] Number of frame's captured by driver into the buffer
     *   provided by application. This will also get incremented in case of
     *   frame drop mode. */
    uint32_t fldCount[FVID2_MAX_FIELDS];
    /**< [OUT] Field count for each field. */

    uint32_t maxRecvFrmWidth;
    /**< [OUT] Maximum frame/field width received from start to now. */
    uint32_t minRecvFrmWidth;
    /**< [OUT] Minimum frame/field width received from start to now. */
    uint32_t maxRecvFrmHeight;
    /**< [OUT] Maximum frame/field height received from start to now. */
    uint32_t minRecvFrmHeight;
    /**< [OUT] Minimum frame/field height received from start to now. */

    uint32_t droppedFrmCount;
    /**< [OUT] Number of frame's dropped by driver due to unavailability
     *   of buffer from application - used in frame drop mode. */
    uint32_t repeatFrmCount;
    /**< [OUT] Repeated frame/field count - used in frame repeat mode. */
    uint32_t fidRepeatCount;
    /**< [OUT] Counter to keep track of getting two consecutive top or
     *   bottom fields. */

    uint32_t lastFrmWidth;
    /**< [OUT] Last frame/field captured width in pixels. */
    uint32_t lastFrmHeight;
    /**< [OUT] Last frame/field captured height in lines. */
    uint32_t lastFrmTimeStamp;
    /**< [OUT] Last frame/field captured timestamp in OS ticks. */
    uint32_t lastFid;
    /**< [OUT] Last FID received. */

    /*
     * Error Counts.
     */
    uint32_t descErrCount;
    /**< [OUT] Capture descriptor not written by hardware error count.
     *          Not all interfaces support this, please refer interface
     *          specific header for details. */
} Cal_CaptChStatus;

/**
 *  \brief Capture overflow parameters, will be filled by the driver during
 *   check for overflow. This structure is required to send to driver by
 *   application to get the status of overflow
 */
typedef struct
{
    uint32_t isOverflowOccured;
    /**< [IN] Flag to check if overflow has occurred or not. */
} Cal_CaptOverflowCheckParams;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief CAL init function.
 *
 *  Initializes the CAL driver.
 *
 *  \return FVID2_SOK on success else appropriate FVID2 error code on failure.
 */
int32_t Cal_init(void);

/**
 *  \brief CAL de init function.
 *
 *  De Initializes the CAL driver.
 *
 *  \return FVID2_SOK on success else appropriate FVID2 error code on failure.
 */
int32_t Cal_deInit(void);

/**
 *  \brief Get capture instance ID from channel number.
 *
 *  chNum is value which is a combination of
 *      - instance ID
 *      - stream ID for that instance
 *      - channel ID for that stream, instance
 *
 *  For details refer to video capture section in User Guide.
 *
 *  \param chNum        [IN] Channel number
 *
 *  \return instance ID (0 .. CAL_CAPT_INST_ID_MAX - 1)
 */
static inline uint32_t Cal_captGetInstId(uint32_t chNum);

/**
 *  \brief Get capture stream ID from channel number.
 *
 *  chNum is value which is a combination of
 *      - instance ID
 *      - stream ID for that instance
 *      - channel ID for that stream, instance
 *
 *  For details refer to video capture section in User Guide.
 *
 *  \param chNum        [IN] Channel number
 *
 *  \return stream ID (0 .. CAL_CAPT_MAX_STREAMS - 1)
 */
static inline uint32_t Cal_captGetStreamId(uint32_t chNum);

/**
 *  \brief Get capture channel ID from channel number.
 *
 *  chNum is value which is a combination of
 *      - instance ID
 *      - stream ID for that instance
 *      - channel ID for that stream, instance
 *
 *  For details refer to video capture section in User Guide.
 *
 *  \param chNum        [IN] Channel number
 *
 *  \return Channel ID (0 .. CAL_CAPT_CH_PER_PORT_MAX - 1)
 */
static inline uint32_t Cal_captGetChId(uint32_t chNum);

/**
 *  \brief Make a system unique channel number.
 *
 *  chNum is value which is a combination of
 *      - instance ID
 *      - stream ID for that instance
 *      - channel ID for that stream, instance
 *
 *  For details refer to video capture section in User Guide.
 *
 *  \param instId       [IN] Instance ID.
 *  \param streamId     [IN] Stream ID.
 *  \param chId         [IN] Channel ID within an instance.
 *
 *  \return Channel number
 */
static inline uint32_t Cal_captMakeChNum(uint32_t instId,
                                       uint32_t streamId,
                                       uint32_t chId);

/**
 *  \brief Cal_CaptCreateParams structure init function.
 *
 *  \param createPrms   [IN] Pointer to #Cal_CaptCreateParams structure.
 *
 */
static inline void CalCaptCreateParams_init(Cal_CaptCreateParams *createPrms);

/**
 *  \brief Cal_CaptChStatusArgs structure init function.
 *
 *  \param chStatusArgs [IN] Pointer to #Cal_CaptChStatusArgs structure.
 *
 */
static inline void CalCaptChStatusArgs_init(Cal_CaptChStatusArgs *chStatusArgs);

/* ========================================================================== */
/*                       Static Function Definitions                          */
/* ========================================================================== */

static inline uint32_t Cal_captGetInstId(uint32_t chNum)
{
    return (chNum / (CAL_CAPT_CH_PER_PORT_MAX * CAL_CAPT_MAX_STREAMS));
}

static inline uint32_t Cal_captGetStreamId(uint32_t chNum)
{
    uint32_t value;

    value  = chNum % (CAL_CAPT_CH_PER_PORT_MAX * CAL_CAPT_MAX_STREAMS);
    value /= CAL_CAPT_CH_PER_PORT_MAX;

    return (value);
}

static inline uint32_t Cal_captGetChId(uint32_t chNum)
{
    return (chNum % CAL_CAPT_CH_PER_PORT_MAX);
}

static inline uint32_t Cal_captMakeChNum(uint32_t instId,
                                       uint32_t streamId,
                                       uint32_t chId)
{
    return ((instId * CAL_CAPT_CH_PER_PORT_MAX * CAL_CAPT_MAX_STREAMS)
            + (streamId * CAL_CAPT_CH_PER_PORT_MAX) + chId);
}

static inline void CalCaptCreateParams_init(Cal_CaptCreateParams *createPrms)
{
    uint32_t streamId, chId;

    if (NULL != createPrms)
    {
        createPrms->videoIfMode      = FVID2_VIFM_SCH_ES;
        createPrms->videoIfWidth     = FVID2_VIFW_16BIT;
        createPrms->bufCaptMode      = CAL_CAPT_BCM_LAST_FRM_REPEAT;
        createPrms->chInQueueLength  = CAL_CAPT_QUEUE_LEN_PER_CH;
        createPrms->numCh     = 1U;
        createPrms->numStream = 1U;
        for (streamId = 0U; streamId < CAL_CAPT_MAX_STREAMS; streamId++)
        {
            for (chId = 0U; chId < CAL_CAPT_CH_PER_PORT_MAX; chId++)
            {
                createPrms->chNumMap[streamId][chId] = 0U;
            }
        }
        createPrms->pAdditionalArgs  = NULL;
    }

    return;
}

static inline void CalCaptChStatusArgs_init(Cal_CaptChStatusArgs *chStatusArgs)
{
    if (NULL != chStatusArgs)
    {
        chStatusArgs->chNum         = 0U;
        chStatusArgs->frameInterval = 16U;      /* Default to 1/(60FPS) */
    }

    return;
}

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CAL_DRV_H_ */

/* @} */
