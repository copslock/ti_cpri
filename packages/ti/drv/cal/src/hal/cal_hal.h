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
 *  \file cal_hal.h
 *
 *  \brief This file defines all abstractions for CAL module of CAL.
 *  This abstraction will support multiple instances of PPI, CSI. Operates 2
 *  modes primarily. memory to memory mode and capture mode.
 *
 *  NO SUPPORT for
 *  . Circular buffer
 *  . Sub frame processing
 *  . Lane merging
 *  . PPI Grouping
 */

#ifndef CAL_HAL_H_
#define CAL_HAL_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/csl/soc.h>

#include <ti/drv/cal/cal.h>
#include <ti/drv/cal/src/hal/cal_halCommon.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/** \brief Defines the total number of opens supported*/
#define CAL_HAL_OPEN_NUM            (8U)

/* CONTROL Command Supported */

/**
 *  IOCTL_CAL_HAL_SETCFG
 *  \brief This control is to be used to update the configuration.
 *
 *   Valid when in CAL_HAL_MODE_CAPTURE mode
 *   Valid when in CAL_HAL_MODE_M2M mode
 *   NOT Valid when in CAL_HAL_MODE_CMPLXIO_CTRL mode
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_SETCFG
 *  \param cmdArgs  A pointer of type Cal_HalCfg_t. All the enabled
 *                      modules should have a valid config.
 *  \param arg      Not used for now.
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_SETCFG (IOCTL_CAL_HAL_STOP + 1U)

/**
 *  IOCTL_CAL_HAL_SETCFG_CMPLXIO
 *  \brief This control is to be used to update the complex IO configuration.
 *
 *   Valid when in CAL_HAL_MODE_CAPTURE mode
 *   NOT Valid when in CAL_HAL_MODE_M2M mode
 *   Valid when in CAL_HAL_MODE_CMPLXIO_CTRL mode
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_SETCFG_CMPLXIO
 *  \param cmdArgs  A pointer of type Cal_HalCmplxIoCfg_t. All the enabled
 *                      modules should have a valid config.
 *  \param arg      Not used for now.
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_SETCFG_CMPLXIO (IOCTL_CAL_HAL_SETCFG + 1U)

/**
 *  IOCTL_CAL_HAL_UPDATE_BUFFERS
 *  \brief This control is to be used to update address of the frame to be read.
 *
 *  \warning Please ensure IPIPE IF has been configured read in a frame from
 *              memory.
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_UPDATE_BUFFERS
 *  \param cmdArgs  A pointer of type, Cal_HalBufferAddr_t.
 *  \param arg      Not used
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_UPDATE_BUFFERS \
    (IOCTL_CAL_HAL_SETCFG_CMPLXIO + 1U)

/**
 *  IOCTL_CAL_HAL_RESET
 *  \brief This control is to be used to perform soft reset of CAL
 *
 *  \warning Ensure not active operations are in progress.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_RESET
 *  \param cmdArgs  None.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_RESET \
    (IOCTL_CAL_HAL_UPDATE_BUFFERS + 1U)

/**
 *  IOCTL_CAL_HAL_GET_INSTANCECFG
 *  \brief Get the current CAL instance config
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_GET_INSTANCECFG
 *  \param cmdArgs  A pointer to structure of type #Cal_HalInstCfg_t.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_GET_INSTANCECFG \
    (IOCTL_CAL_HAL_RESET + 1U)

/**
 *  IOCTL_CAL_HAL_SET_INSTANCECFG
 *  \brief Updated the current instance config
 *
 *  \warning Ensure not active operations are in progress.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_SET_INSTANCECFG
 *  \param cmdArgs  A pointer to structure of type #Cal_HalInstCfg_t.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_SET_INSTANCECFG \
    (IOCTL_CAL_HAL_GET_INSTANCECFG + 1U)

/**
 *  IOCTL_CAL_HAL_SET_VPORT_CFG
 *  \brief Set the Vport Configuration
 *
 *  \warning Ensure not active operations are in progress.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_SET_VPORT_CFG
 *  \param cmdArgs  A pointer to structure of type #Cal_VPort_t.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_SET_VPORT_CFG \
    (IOCTL_CAL_HAL_SET_INSTANCECFG + 1U)

/**
 *  IOCTL_CAL_HAL_SET_VPORT_CFG
 *  \brief Set the Vport Configuration
 *
 *  \warning Ensure not active operations are in progress.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_SET_VPORT_CFG
 *  \param cmdArgs  A pointer to structure of type #Cal_BysOut_t.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_SET_BYSOUT_CFG \
    (IOCTL_CAL_HAL_SET_VPORT_CFG + 1U)

/**
 *  IOCTL_CAL_HAL_RD_FMT_UPDATE
 *  \brief Update the CAL read format, i.e. height, width, pitch and bpp.
 *          Primarily used ISP resizer driver, where luma and chroma is
 *          processed separately.
 *
 *  \warning This should be re-enterent, callable from an ISR.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_RD_FMT_UPDATE
 *  \param cmdArgs  A pointer to structure of type #Fvid2_Format.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
#define IOCTL_CAL_HAL_RD_FMT_UPDATE \
    (IOCTL_CAL_HAL_SET_BYSOUT_CFG + 1U)

/**
 *  IOCTL_CAL_HAL_LINE_EVENT_CFG
 *  \brief Configure the line number at which an event/interrupt should be
 *          raised.
 *
 *  \warning Ensure no active operations are in progress.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      IOCTL_CAL_HAL_LINE_EVENT_CFG
 *  \param cmdArgs  A pointer to structure of type #Cal_HalLineEventCfg_t.
 *  \param arg      None
 *
 *  \return         Returns FVID2_SOK always
 */
#define IOCTL_CAL_HAL_LINE_EVENT_CFG \
    (IOCTL_CAL_HAL_RD_FMT_UPDATE + 1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/** \brief HAL INSTACE X in Capture mode */
#define CAL_HAL_MODE_COMPLEXIO_CTRL    (2U)

typedef enum Cal_HalInstId
{
    CAL_HAL_INST_A = 0x0,
    /**< Cal Instance A */
    CAL_HAL_INST_B = 0x1,
    /**< Cal Instance B */
    CAL_HAL_INST_FORCE_INT = 0x7FFFFFFF
                                     /**< This will ensure enum is not packed,
                                             will always be contained in int */
} Cal_HalInstId_t;

/**
 *  struct Cal_HalMode
 *  \brief Lists the different operational modes of CAL hal.
 *
 */
typedef enum Cal_HalMode
{
    CAL_HAL_MODE_MIN = 0,
    /**< Begin Marker */
    CAL_HAL_MODE_CAPTURE = 1,
    /**< Intended to be used, when we are capturing from the sensors */
    CAL_HAL_MODE_M2M = 2,
    /**< Intended to be used, when data is read from memory via CAL */
    CAL_HAL_MODE_CMPLXIO_CTRL = 3,
    /**< Intended to be used, when its required to control the complex io */
    CAL_HAL_MODE_MAX = 4,
    /**< End Marker */
    CAL_HAL_MODE_FORCE_INT = 0x7FFFFFFF
                                     /**< This will ensure enum is not packed,
                                             will always be contained in int */
}Cal_HalMode_t;

/**
 *  struct Cal_HalDmaWrModes
 *  \brief Different DMA write modes.
 *
 */
typedef enum Cal_HalDmaWrModes
{
    CAL_HAL_DMA_WR_DISABLED = 0,
    /**< Write channel is disabled */
    CAL_HAL_DMA_WR_SHD = 1,
    /**< Shadowed, used for normal mode. */
    CAL_HAL_DMA_WR_CNT = 2,
    /**< Not supported for now */
    CAL_HAL_DMA_WR_SNT_INIT = 3,
    /**< Not supported for now */
    CAL_HAL_DMA_WR_CONST = 4,
    /**< Base address is provided for every frame */
    CAL_HAL_DMA_WR_MODES_MAX = 5,
    /**< End marker */
    CAL_HAL_DMA_WR_FORCE_INT = 0x7FFFFFFF
                                       /**< This will ensure enum is not packed,
                                               will always be contained in int */
}Cal_HalDmaWrModes_t;

/**
 *  struct Cal_HalBufferAddr
 *  \brief Provide the various buffer address required
 *
 */
typedef struct Cal_HalBufferAddr
{
    uint32_t numBuff;
    /**< Number of valid buffers */
    uint32_t cPortId[CAL_CAPT_MAX_STREAMS];
    /**< Associated C Port IDs */
    uint32_t wrDmaCtx[CAL_CAPT_MAX_STREAMS];
    /**< Used only for write DMA. Not used to read DMA.
     *      Specify the context of DMA to be used */
    uint32_t buffAddr[CAL_CAPT_MAX_STREAMS];
    /**< Pointer of type uint32_t, aligned on 32 bytes boundary */
    uint32_t pitch[CAL_CAPT_MAX_STREAMS];
    /**< Pointer of type uint32_t, aligned on 32 bytes boundary */
}Cal_HalBufferAddr_t;

/**
 *  struct Cal_HalCsi2Timing
 *  \brief CSI2 timing configuration parameters
 */
typedef struct Cal_HalCsi2Timing
{
    uint32_t force_rx_mode_0I01;
    /**< The value supplied in this is not used. This member will be removed */
    uint32_t stop_state_x16_I01;
    /**< Refer the spec for details, breaking the coding guidelines to match
     *      config name used in manuals */
    uint32_t stop_state_x4_I01;
    /**< Refer the spec for details, breaking the coding guidelines to match
     *      config name used in manuals */
    uint32_t stop_state_counter_I01;
    /**< Refer the spec for details, breaking the coding guidelines to match
     *      config name used in manuals */
} Cal_HalCsi2Timing_t;

/**
 *  struct Cal_HalPpiCfg
 *  \brief Configure control of the PHY.
 *
 */
typedef struct Cal_HalPpiCfg
{
    uint32_t              enable;
    /** TRUE enables PPI, configurations specified below is valid and would
     *      be applied. */
    uint32_t              instance;
    /**< Configure the instance of PPI to be used. Max value is
     *      CSL_CAL_PPI_CNT */
    uint32_t              frame;
    /**< TRUE configure the PHY complete reception of a frame, before it can
     *      be stopped. i.e. STOP will not partial frame capture. Always ends
     *      on a frame boundary.
     *   Otherwise, partial frame could be captured, while stopping */
    uint32_t              ecc;
    /**< TRUE enables the ECC block, ECC checks is done for short / long packets
     *      for all virtual channels.
     *  otherwise, EC is disabled. */
    Cal_HalCsi2Timing_t csi2Cfg;
    /**< CSI 2 Config. Applicable when enabled only  */
}Cal_HalPpiCfg_t;

/**
 *  struct Cal_HalCsi2VcCfg
 *  \brief CSI2 virtual channel configuration parameters
 */
typedef struct Cal_HalCsi2VcCfg
{
    uint32_t instance;
    /**< There are CSL_CAL_PPI_CNT possible  */
    uint32_t contextToBeUsed;
    /**< Specify the CSI context processing to be used.
     *      Valid values are 0 to 7*/
    uint32_t virtualChanNum;
    /**< Virtual channel number, to be used */
    uint32_t dt;
    /**< Configure to receive data types
     *  0x0 - Virtual channel disabled, no data is to be received. Should be
     *          0x0, when not used.
     *  0x1 - Receive any type of data.
     *  0x2 - 0xF - no valid
     *  0x10 to 0x3F - receive all data tagged as Data Type */
    uint32_t att;
    /**< TRUE configures the received data as attribute for further processing
     *      otherwise, data is tagged as Pixel Data */
    uint32_t lines;
    /**< Expected number of lines to be received. 0x0 if the number of lines
     *      to be received is unknown. */
} Cal_HalCsi2VcCfg_t;

/**
 *  struct Cal_HalWrDmaCfg
 *  \brief CAL DMA write configurations
 */
typedef struct Cal_HalWrDmaCfg
{
    uint32_t              contextToBeUsed;
    /**< Configure the write context to be used */
    Cal_HalDmaWrModes_t mode;
    /**< Configure the mode, disable is one of the valid mode */
    Cal_StreamType_t stream;
    /**< Configure the stream to capture */
    uint32_t              stallM2MRd;
    /**< TRUE configure Read DMA to stall, if the write DMA context is stalled.
     *      Required to ensure real-time stream (write DAM) has higher
     *      processing priority over read DMA */
    uint32_t              ySkipMode;
    /**< 0x0 - No line skips done
     *   0x2 - Writes 2 lines, skips next 2 lines
     *   0x3 - Writes 2 lines, skips next 4 lines */

    uint32_t              xPixelSkip;
    /**< Number of pixels to skip, should be a multiple of 64 bits */
    Fvid2_Format          format;
    /**< Following member of this structure is used & should be valid.
     *      format.width    Maximum line length, Should be multiple of 64 bits.
     *                          line length in bytes / 8.
     *                          0 for unlimited line length
     *      format.height   Maximum lines of video / data expected for a frame
     *                          0 is for unlimited
     *      format.pitch[0] Pitch, should be multiple of 128 bits.
     *                          pitch in byte count / 16
     */
} Cal_HalWrDmaCfg_t;

/**
 *  struct Cal_HalRdDmaCfg
 *  \brief CAL Read DMA configuration
 */
typedef struct Cal_HalRdDmaCfg
{
    uint32_t     enable;
    /**< TRUE, Read from memory. When enabled following configurations are
     *      applied. Disabled otherwise */
    uint32_t     pixClock;
    /**< Configure the pixel clock required to read. Essentially controlling DMA
     *      read data rate. 0x0, will disable READ DMA. */
    uint32_t     bwLimit;
    /**< Minimum number of cycles between two consecutive DMA read operations.
     *      Could be used limit DMA read rate */
    uint32_t     ocpTagCnt;
    /**< Maximum number of READ request that could pend */
    uint32_t     bysOutLeWait;
    /**< If BYS Out port is used and its inserting line blanking,
     *  TRUE configures the DMA read to wait until line blanking done before the
     *      the next read.
     *  Otherwise, will not wait for end of line blanking period (generated
     *      by the BYS Out ) */
    uint32_t     ySkipMode;
    /**<    0x0 No lines are skipped, linear data
     *      0x1 Data type of YUV 420, planar data.
     *      0x2 Read 2 line and skip 2 lines
     *      0x3 Read 2 line and skip 4 lines */
    Fvid2_Format format;
    /**< Following member of this structure is used & should be valid.
     *      format.width    - expressed as byte count and not PIXELS
     *      format.height
     *      format.pitch
     *      format.dataFormat - only in case of CAL Read.
     *                          FVID2_DF_YUV420SP_VU or FVID2_DF_BAYER_RAW */
} Cal_HalRdDmaCfg_t;

/**
 *  struct Cal_HalDmaVcCfg
 *  \brief CAL config, used to control all the modules of CAL. When the HAL
 *      is opened as CAPTURE device,
 *      i.e. Cal_HalOpenParams_t.mode == CAL_HAL_MODE_CAPTURE
 *          rdDmaCfg configuration parameters are not applied.
 *      When opened in M2M mode
 *      i.e. Cal_HalOpenParams_t.mode == CAL_HAL_MODE_M2M, csi2VcCfg
 *      configuration is not applied
 *      When opened in complex IO control mode
 *      Cal_HalOpenParams_t.mode == CAL_HAL_MODE_CMPLXIO_CTRL
 *      None of the parameters is applied.
 *
 */
typedef struct Cal_HalDmaVcCfg
{
    uint32_t             numCPortId;
    /**< Defines the total number of DMA contexts. Should 0x0 or <
     *      CAL_CAPT_MAX_STREAMS */
    uint32_t             cportId[CAL_CAPT_MAX_STREAMS];
    /**< Associated CPORT IDs. All other members of this struct uses the
     *  conents of cportId[] as index. */

    uint32_t             isCsi2VcCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid CSI2 virtual channel config, otherwise invalid
     *      config, not used. */
    Cal_HalCsi2VcCfg_t csi2VcCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure CSI2 Virtual channel parameters. There are 4 virtual channels
     *      available. */

    uint32_t             isWrDmaCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid DMA write channel config, otherwise invalid config
     *      not used. */
    Cal_HalWrDmaCfg_t  wrDmaCfg[CAL_CAPT_MAX_STREAMS];
    /**< DMA Channel configurations */

    uint32_t             isRdDmaCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid Read DMA config, otherwise invalid config
     *        not used. */
    Cal_HalRdDmaCfg_t  rdDmaCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure Read DMA configurations. If enabled, will always be read on
     *      cportId as 0x0.
     *  which means, pixProcCfg[0] and wrDmaCfg[0] should be valid and will be
     *  applied. */
} Cal_HalDmaVcCfg_t;

/**
 *  struct Cal_HalLineEventCfg
 *  \brief Line number event notification configuration.
 *
 */
typedef struct Cal_HalLineEventCfg
{
    uint32_t             numCPortId;
    /**< Defines the total captures streams, for which line notification should
     *  be configured. Should be 0x01 */
    uint32_t             cportId[CAL_CAPT_MAX_STREAMS];
    /**< Associated CPORT IDs. All other members of this struct uses the
     *  contents of cportId[] as index. */
    uint32_t             lineNumber[CAL_CAPT_MAX_STREAMS];
    /**< Specify the line number, at which notification should be generated.
     *  Valid range is between 1 & (2^14) - 1 */
} Cal_HalLineEventCfg_t;

/**
 *  struct Cal_HalCfg
 *  \brief CAL config, Primarily used by core to convey CAL config to HAL.
 */
typedef struct Cal_HalCfg
{
    uint32_t     numCPortId;
    /**< Defines the total number of streams that are required. Should 0x0 or <
            CAL_CAPT_MAX_STREAMS */
    uint32_t     cportId[CAL_CAPT_MAX_STREAMS];
    /**< Associated CPORT IDs. All other members of this struct uses the
        contents of cportId[] as index.
        Valid values are 1 to 7 */
    Fvid2_Format streamFmt[CAL_CAPT_MAX_STREAMS];
    /**< Specify the characteristics of streams that has to be received.
            Valid member of this structure are
            .width  - specify the width if known, else 0x0.
                        WARNING - Sufficient buffer should be allocated to acco
                            modate max line length.
            .height - expected number of lines, 0x0 for unknown
                        WARNING - Sufficient buffer should be allocated to acco
                            modate max lines.
            .pitch  - pitch
            In case of CSI capture using CAL,
              .bpp        - Bits per pixel #Fvid2_BitsPerPixel
              .dataFormat - Not used, ignored. Instead use csiDataType
            In case of M2M paths using CAL,
              .bpp        - Bits Per Pixel #Fvid2_BitsPerPixel
              .dataFormat - dataformat as per #Fvid2_DataFormat */
    uint32_t              csiDataType[CAL_CAPT_MAX_STREAMS];
    /**< Specify the CSI Type of streams that has to be received.
         Use #Cal_Csi2DataFormat. */
    uint32_t              isCsi2BasedCapture[CAL_CAPT_MAX_STREAMS];
    /**< Identify if CSI2 based captured is to be used. TRUE configures for CSI2
            based, otherwise it could be LVDS, BYS In based. In which
            case virtualChanNum is not used.
            For CSI2 based capture virtualChanNum should hold a valid value.
            For CSI2 based capture stream should hold a valid type */
    uint32_t              virtualChanNum[CAL_CAPT_MAX_STREAMS];
    /**< Applicable when capturing via CSI2 interface only.
            Specify the virtual channel number to be used. Valid rage 0 - 3 */
    Cal_StreamType_t stream[CAL_CAPT_MAX_STREAMS];
    /**< Applicable when capturing via CSI2 interface only.
            Specify the streams that required to be captured */
    uint32_t              isPixProcCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid pixel processing config, otherwise invalid config
            not used. */
    Cal_PixProc_t    pixProcCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure pixel processing contexts. There are 4 pixel processing
            contexts available. */

    /* NOTE there is one instance of BYS Out, BYS In, Video Port and RD DMA.
        At any point only one configuration should be valid, otherwise, the last
        config would be active. */
    uint32_t              isBysOutCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid BYS Out config, otherwise invalid config
              not used. */
    Cal_BysOut_t     bysOutCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure BYS Out */

    uint32_t              isBysInCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid BYS In config, otherwise invalid config
              not used. */
    uint32_t              bysInEnable[CAL_CAPT_MAX_STREAMS];
    /**< Configure BYS In. */

    uint32_t              isVportCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid Video Port In config, otherwise invalid config
              not used. */
    Cal_VPort_t      vportCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure Video Port */

    uint32_t              writeToMem[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates that the captured stream should be written to memory */

    void                 *pDmaVcCfg;
    /**< HAL config computed by core based on the config */
} Cal_HalCfg_t;

/**
 *  struct Cal_HalInstCfg
 *  \brief Instance specific configuration. Applies to instance of CAL, expected
 *          to be configured once.
 *          All possible use cases of this module should be comprehended before
 *          the config is decided.
 */
typedef struct Cal_HalInstCfg
{
    /* Generic config applicable for all memory 2 memory mode of CAL. These
     *  config are to have a valid value when operated in m2m mode. */
    uint32_t              mFlagH;
    /**< When no real time data is received, this should be 0xFF. A 8 bit value,
     *  please refer spec for details while capturing real time data. */
    uint32_t              mFlagL;
    /**< When no real time data is received, this should be 0xFF. A 8 bit value,
     *  please refer spec for details while capturing real time data. */
    uint32_t              rdDmaStall;
    /**< TRUE configures read dma to stall while stall flag (MFlag) is asserted
     *      FALSE/Others dosent stall DMA read on stall assertion.
     *      Recommended to be set to TRUE if CAL is also used capture data from
     *      sensors (i.e. real time data) */
    uint32_t              pwrScpClk;
    /**< TRUE configure pwrScpClk to be free running mode, even when not its not
     *      needed. FALSE gates it when not needed */
    uint32_t              llForceState;
    /**< Not used - reserved */
    uint32_t              dmaBurstSize;
    /**< maximum burst size of DMA write. Valid range is 0 to 3. 0 for 16 bytes
     *  1 for 32, 2 for 64 and 3 for 128 bytes per burst */
    uint32_t              tagCnt;
    /**< Maximum number of outstanding OCP transactions. tagCnt = tagCnt + 1.
     *      Maximum value supported is 0xF */
    uint32_t              postedWrites;
    /**< TRUE configures for posted writes only, non-posted writes otherwise */

    /* Applicable only when CAL is used to interface to sensor (via the
     *  complex IO, which is normal expected connection.
     * These config are to have a valid value when operated in capture mode.
     * Maps to following registers CSL_CAL_C2PPI_CSI2_COMPLEXIO_CFG_XX */

    uint32_t              numCmplxIoInst;
    /**< Number of complex IO used */
    Cal_CmplxIoCfg_t cmplxIoCfg[CSL_CAL_CMPLXIO_CNT];
    /**< Complex IO config */
    uint32_t              numPpiInst;
    /**< Configure the instance of PPI used */
    Cal_HalPpiCfg_t     ppiCfg[CSL_CAL_PPI_CNT];
    /**< PHY Configurations */
    uint32_t              csi2PhyClock[CSL_CAL_CMPLXIO_CNT];
    /**< Specify the CSI2 PHY Clock in MHz. e.g. if 400 MHz is the clock
            \code csi2PhyClock = 400; */
} Cal_HalInstCfg_t;

/**
 *  struct Cal_HalInstParams
 *  \brief Initialization parameters, These parameters cannot be changed once
 *          initialized.
 */
typedef struct Cal_HalInstParams
{
    uint32_t instId;
    /**< CAL instance ID */
    uint32_t baseAddress;
    /**< CAL registers base address */
    uint32_t phy0BaseAddress;
    /**< CAL PHY 0 base */
    uint32_t phy1BaseAddress;
    /**< CAL PHY 0 base */
    uint32_t prms;
    /**< Not used */
} Cal_HalInstParams_t;

/**
 *  struct Cal_HalOpenParams
 *  \brief Configuration required while opening.
 *
 */
typedef struct Cal_HalOpenParams
{
    uint32_t        instId;
    /**< Identifies the instance of CAL. */
    Cal_HalMode_t mode;
    /**< Configure the mode of operation. */
}Cal_HalOpenParams_t;

typedef struct Cal_HalPlatformData
{
    uint32_t                   numCalInst;
    /**< Nuumber of CAL instances */
    Cal_HalInstParams_t     *calInstPrms;
    /**< Cal init parameters */
} Cal_HalPlatformData_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 *  \brief Initializes CAL module and initializes the hal state.
 *
 *  \param numInst  Number of instances of CAL that requires to be initialized.
 *  \param initPrms Pointer to array of #Cal_HalInstParams_t. This array
 *                      should contain numInst valid entries.
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns FVID2_SOK on success else returns error value
 */
int32_t Cal_halInit(uint32_t numInst,
                          const Cal_HalInstParams_t *initPrms, void *arg);

/**
 *  \brief De-initializes the CAL modules
 *
 *  \param arg  Not used for now. Should be NULL
 *
 *  \return     Returns FVID2_SOK on success else returns error value
 */
int32_t Cal_halDeInit(void *arg);

/**
 *  Cal_halOpen
 *  \brief This function should be called prior to calling any of the CAL
 *  abstraction APIs are used. This should be called post initialization of
 *  CAL module.
 *
 *  \param openPrms Open Parameters
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         A non NULL pointers, NULL on errors.
 */
Cal_HalHandle Cal_halOpen(const Cal_HalOpenParams_t *openPrms,
                                void                        *arg);

/**
 *  Cal_halClose
 *  \brief This function should be called to close previously opened abstraction
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halClose(Cal_HalHandle handle, void *arg);

/**
 *  Cal_halControl
 *  \brief CAL specific control commands.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param cmd      A valid control command, refer above for valid commands
 *  \param cmdArgs  Arguments for the control command, depends on the control
 *                  command used.
 *  \param arg      Arguments for the control command, depends on the control
 *                  command used.
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halControl(Cal_HalHandle handle,
                             uint32_t        cmd,
                             void         *cmdArgs,
                             void         *arg);

/**
 *  Cal_halCaptureStart
 *  \brief This function should be called to start capture of frames.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param arg      A non NULL pointer of type Cal_HalDmaVcCfg_t, with
 *                      desired / valid DMA mode of operation.
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halCaptureStart(Cal_HalHandle handle, void *arg);

/**
 *  Cal_halCaptureStop
 *  \brief This function should be called to stop capture of frames.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param arg      A non NULL pointer of type Cal_HalDmaVcCfg_t, with
 *                      DMA mode as CAL_HAL_DMA_WR_DISABLED.
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halCaptureStop(Cal_HalHandle handle, void *arg);

/**
 *  Cal_halUpdateBufAddr
 *  \brief This function should be called to updated the buffer address of the
 *          frame to be captured.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param bufPtrs  A pointer of type, Cal_HalBufferAddr_t.
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halUpdateBufAddr(Cal_HalHandle          handle,
                                   const Cal_HalBufferAddr_t *bufPtrs);

/**
 *  Cal_halRdDmaStart
 *  \brief This function should be called start DMA read.
 *
 *  \param handle   A non NULL handle returned by function Cal_halOpen
 *  \param procMode Not used for now. Used for read in continious mode or one
 *                      frame mode.
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halRdDmaStart(Cal_HalHandle     handle,
                                Cal_HalCtrlProcMode_t procMode,
                                void             *arg);

/**
 *  Cal_halPhyEnClockAndReset
 *  \brief This function is specific to PHY/Platform, initializes the PHY.
 *
 *  \param baseAddr             Adress of the CAL peripheral
 *  \param cam0RxCoreBaseAddr   Address of the PHY connected to PPI 0
 *  \param cam1RxCoreBaseAddr   Address of the PHY connected to PPI 1
 *  \param pCfg                 A Valid CAL instance config
 *
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns 0 on success else returns error value
 */
int32_t Cal_halPhyEnClockAndReset(uint32_t            baseAddr,
                                     uint32_t            cam0RxCoreBaseAddr,
                                     uint32_t            cam1RxCoreBaseAddr,
                                     const Cal_HalInstCfg_t  *pCfg);

/**
 *  Cal_halIsBysOutEof
 *  \brief This function checks if Bys Ouf End of Frame bit set.
 *
 *  \param handle   Handle of the CAL
 *
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns value of the EOF bit
 */
uint32_t Cal_halIsBysOutEof(Cal_HalHandle handle);

/**
 *  Cal_halIsBysOutEof
 *  \brief Clears Vport Eof Flag
 *
 *  \param handle   Handle of the CAL
 *
 *  \param arg      Not used for now. Should be NULL
 */
void Cal_halClearVportEof(Cal_HalHandle handle);

/**
 *  Cal_halIsVportEof
 *  \brief This function checks if Vport End of Frame bit set.
 *
 *  \param handle   Handle of the CAL
 *
 *  \param arg      Not used for now. Should be NULL
 *
 *  \return         Returns value of the EOF bit
 */
uint32_t Cal_halIsVportEof(Cal_HalHandle handle);

Cal_HalPlatformData_t *Cal_halGetPlatformData(void);
#ifdef __cplusplus
}
#endif

#endif  /* #ifndef CAL_HAL_H_ */
