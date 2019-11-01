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
 *  \file cal_cfg.h
 *
 *  \brief  Defines the structures / control operations that could be used to
 *              configure / control CAL module
 */

/**
 *  \ingroup CAL_DRV_COMMON_API
 *  \addtogroup CAL_DRV_COMMON_CAPTURE_CAL - CAL Config API
 *
 *  @{
 */

#ifndef CAL_CFG_H_
#define CAL_CFG_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                   Macros                                   */
/* ========================================================================== */

/** \brief Number of queue length per capture channel in capture driver */
#define CAL_CAPT_QUEUE_LEN_PER_CH       (16U)

/** \brief Defines the total number of complex IO available per CAL (1) */
#if defined (SOC_AM65XX)
#define CAL_CAPT_MAX_CMPLXIO_INST       (1U)
#endif

/** \brief Defines total number of pixel processing context available */
#define CAL_CAPT_MAX_PIX_PROC_CONTEXT   (4U)

/** \brief Defines the total number of DMA writes that are possible per CAL
**/
#define CAL_CAPT_MAX_STREAMS            (8U)

/** \brief Defines the total number error interrupts
**/
#define CAL_CAPT_MAX_ERROR_INTERRUPTS   (20U)

/** \brief Defines the total number of VC, for which a callback can be issued
 *          on reception of X lines.
**/
#define CAL_CAPT_MAX_X_LINE_MONITOR_CNT (1U)

/** \brief Log enable for CAL modules. */
#define CalTrace                        (GT_INFO | GT_TraceState_Enable)

/**
 *  struct Cal_CaptInstId
 *  \brief Different instances of capture. As name suggests, each instance is
 *          associated with a particular method of capture. Multiple instances
 *          can co-exist, provided co-existence is supported in the h/w.
 *
 *  \warning Deviation from naming convention: To enable backward compatibility
 *
 */
typedef enum Cal_CaptInstId
{
    CAL_CAPT_INST_ID_A_ID = 0U,
    /**< Capture CSI2 streams via CAL. Requires to be first enum.
     *      This is requires to start with 0x0. Used an index internally in
     *      core */
    CAL_CAPT_INST_ID_A_CPI = 1U,
    /**< Capture From CPI - Via VPORT of IPIPEIF. Essentially via Parallel OR
     *      LVDS input which are interfaced to CPI port. */
    CAL_CAPT_INST_ID_MAX = 2U,
    /**< Enum end marker */
    CAL_CAPT_INST_ID_FORCE_INT = 0x7FFFFFFF
                        /**< This will ensure enum is not packed,
                         *      will always be contained in int */
} Cal_CaptInstId_t;   /**< Cal_CaptInstId_t */


/**
 *  enum Cal_CaptSubModuleId
 *  \brief Different sub-modules with in the CAL block.
 *
 *  \warning Deviation from naming convention: To enable backward compatibility
 *
 */
typedef enum Cal_CaptSubModuleId
{
    CAL_CAPT_INST_ID_SUB_MIN_ID = 0x000,
    /**< Begin marker */
    CAL_CAPT_INST_ID_SUB_PPI_ID_0 = 0x001,
    /**< PPI Interface, one per instance  */
    CAL_CAPT_INST_ID_SUB_PIX_EXTRACT_ID = 0x002,
    /**< Pixel extract */
    CAL_CAPT_INST_ID_SUB_DPCM_DEC_ID = 0x004,
    /**< DPCM Decode */
    CAL_CAPT_INST_ID_SUB_DPCM_ENC_ID = 0x008,
    /**< DPCM Encode */
    CAL_CAPT_INST_ID_SUB_PIX_PACK_ID = 0x010,
    /**< Pixel packing */
    CAL_CAPT_INST_ID_SUB_BYS_OUT_ID = 0x020,
    /**< BYS Out */
    CAL_CAPT_INST_ID_SUB_BYS_IN_ID = 0x040,
    /**< BYS IN */
    CAL_CAPT_INST_ID_SUB_VPORT_ID = 0x080,
    /**< BYS IN */
    CAL_CAPT_INST_ID_SUB_DMA_RD_ID = 0x100,
    /**< DMA Read */
    CAL_CAPT_INST_ID_SUB_DMA_WR_ID = 0x200,
    /**< DMA Write */
    CAL_CAPT_INST_ID_SUB_CSI2_ID = 0x400,
    /**< cport ID */
    CAL_CAPT_INST_ID_SUB_CPORT_ID = 0x800,
    /**< cport ID */
    CAL_CAPT_INST_ID_SUB_LVDS_ID = 0x1000,
    /**< LVDS Port ID */
    CAL_CAPT_INST_ID_SUB_CPI_ID = 0x2000,
    /**< CPI / Parallel interface */
    CAL_CAPT_INST_ID_SUB_PPI_ID_1 = 0x3000,
    /**< PPI Interface, one per instance  */
    CAL_CAPT_INST_ID_SUB_MAX = 0x3001,
    /**< Enum end marker */
    CAL_CAPT_INST_ID_SUB_FORCE_INT = 0x7FFFFFFF
                                /**< This will ensure enum is not packed,
                                 *      will always be contained in int */
} Cal_CaptSubModuleId_t;   /**< Cal_CaptSubModuleId_t */

/**
 *  enum Cal_Csi2DataFormat
 *  \brief CSI2 Data types.
 *
 */
typedef enum
{
    CAL_CSI2_YUV420_8B = 0x18,
    /**< YUV 4:2:0 with 8bit for each Y/U/V */
    CAL_CSI2_YUV420_10B = 0x19,
    /**< YUV 4:2:0 with 10bit for each Y/U/V */
    CAL_CSI2_YUV420_8B_LEGACY = 0x1A,
    /**< YUV 4:2:0 with 8bit for each Y/U/V */
    CAL_CSI2_YUV420_8B_CHROMA_SHIFT = 0x1C,
    /**< YUV 4:2:0 with 8bit for each Y/U/V with
     *   with phase shifted chroma */
    CAL_CSI2_YUV420_10B_CHROMA_SHIFT = 0x1D,
    /**< YUV 4:2:0 with 10bit for each Y/U/V with
     *   with phase shifted chroma */
    CAL_CSI2_YUV422_8B = 0x1E,
    /**< YUV 4:2:2 with 8bit for each Y/U/V */
    CAL_CSI2_YUV422_10B = 0x1F,
    /**< YUV 4:2:2 with 10bit for each Y/U/V */
    CAL_CSI2_RGB444 = 0x20,
    /**< RGB888 - 4-bits B, 4-bits G, 4-bits R */
    CAL_CSI2_RGB555 = 0x21,
    /**< RGB888 - 5-bits B, 5-bits G, 5-bits R */
    CAL_CSI2_RGB565 = 0x22,
    /**< RGB888 - 5-bits B, 6-bits G, 5-bits R */
    CAL_CSI2_RGB666 = 0x23,
    /**< RGB888 - 6-bits B, 6-bits G, 6-bits R */
    CAL_CSI2_RGB888 = 0x24,
    /**< RGB888 - 8-bits B, 8-bits G, 8-bits R */
    CAL_CSI2_RAW6 = 0x28,
    /**< 6 bit raw-data. */
    CAL_CSI2_RAW7 = 0x29,
    /**< 7 bit raw-data. */
    CAL_CSI2_RAW8 = 0x2A,
    /**< 8 bit raw-data. */
    CAL_CSI2_RAW10 = 0x2B,
    /**< 10 bit raw-data. */
    CAL_CSI2_RAW12 = 0x2C,
    /**< 12 bit raw-data. */
    CAL_CSI2_RAW14 = 0x2D,
    /**< 14 bit raw-data. */
    CAL_CSI2_ANY = 0x01,
    /**< Allow any data type for capture */
    CAL_CSI2_DISABLE_CONTEXT = 0x00
    /**< Disable capture. */
} Cal_Csi2DataFormat;

/**
 *  enum Cal_PixExtract
 *  \brief Valid pixel extraction supported, control extraction of "word" from
 *          byte stream.
 *
 */
typedef enum Cal_PixExtract
{
    CAL_PIX_EXRCT_MIN = 0,
    /**< Begin Marker */
    CAL_PIX_EXRCT_B6,
    /**< 6 bits represent a word */
    CAL_PIX_EXRCT_B7,
    /**< 7 bits represent a word */
    CAL_PIX_EXRCT_B8,
    /**< 8 bits represent a word */
    CAL_PIX_EXRCT_B10_LINEAR,
    /**< 10 bits represent a word */
    CAL_PIX_EXRCT_B10_MIPI,
    /**< 10 bits represent a word */
    CAL_PIX_EXRCT_B12_LINEAR,
    /**< 12 bits represent a word */
    CAL_PIX_EXRCT_B12_MIPI,
    /**< 12 bits represent a word */
    CAL_PIX_EXRCT_B14_LINEAR,
    /**< 14 bits represent a word */
    CAL_PIX_EXRCT_B14_MIPI,
    /**< 14 bits represent a word */
    CAL_PIX_EXRCT_B16_BE,
    /**< 16 bits represent a word, big endian */
    CAL_PIX_EXRCT_B16_LE,
    /**< BYPASS - Extraction / 16 bits represent a word, little endian */
    CAL_PIX_EXRCT_MAX
    /**< End Marker */
} Cal_PixExtract_t; /**< Cal_PixExtract_t */

/**
 *  enum Cal_PixDpcmDecoder
 *  \brief Valid de-compression methods supported.
 *
 */
typedef enum Cal_PixDpcmDecoder
{
    CAL_DPCM_DEC_BYPASS = 0x0U,
    /**< BY PASS mode */
    CAL_DPCM_DEC_10_8_10_1 = 0x2U,
    /**< 10_8_10_1 decode */
    CAL_DPCM_DEC_10_7_10_1 = 0x4U,
    /**< 10_7_10_1 */
    CAL_DPCM_DEC_10_7_10_2 = 0x5U,
    /**< 10_7_10_1 decode */
    CAL_DPCM_DEC_10_6_10_1 = 0x6U,
    /**< 10_6_10_1 decode */
    CAL_DPCM_DEC_10_6_10_2 = 0x7U,
    /**< 10_6_10_2 decode */
    CAL_DPCM_DEC_12_8_12_1 = 0x8U,
    /**< 12_8_12_1, decode */
    CAL_DPCM_DEC_12_7_12_1 = 0xAU,
    /**< 12_7_12_1, decode */
    CAL_DPCM_DEC_12_6_12_1 = 0xCU,
    /**< 12_6_12_1 decode */
    CAL_DPCM_DEC_14_10_14 = 0xEU,
    /**< 14_10_14 decode */
    CAL_DPCM_DEC_14_8_14_1 = 0x10U,
    /**< 14_8_14_1 decode */
    CAL_DPCM_DEC_16_12_16_1 = 0x12U,
    /**< 16_12_16_1 decode */
    CAL_DPCM_DEC_16_10_16_1 = 0x14U,
    /**< 16_10_16_1 decode */
    CAL_DPCM_DEC_16_8_16_1 = 0x16U,
    /**< 16_8_16_1 decode */
    CAL_DPCM_DEC_MAX = 0x17U
    /**< End marker */
} Cal_PixDpcmDecoder_t;    /**< Cal_PixDpcmDecoder_t */

/**
 *  enum Cal_PixDpcmEncoder
 *  \brief Valid compression methods supported.
 *
 */
typedef enum Cal_PixDpcmEncoder
{
    CAL_DPCM_ENC_BYPASS = 0,
    /**< Encoder bypassed */
    CAL_DPCM_ENC_10_8_10_1 = 0x02U,
    /**< 10-8-10 Predictor 1 */
    CAL_DPCM_ENC_12_8_12_1 = 0x08U,
    /**< 12-8-12 Predictor 1 */
    CAL_DPCM_ENC_14_10_14 = 0x0EU,
    /**< 14-10-14 Predictor 1 */
    CAL_DPCM_ENC_14_8_14_1 = 0x10U,
    /**< 14-8-14 Predictor 1 */
    CAL_DPCM_ENC_16_12_16_1 = 0x12U,
    /**< 16-12-16 Predictor 1 */
    CAL_DPCM_ENC_16_10_16_1 = 0x14U,
    /**< 16-10-16 Predictor 1 */
    CAL_DPCM_ENC_16_8_16_1 = 0x16U,
    /**< 16-8-16 Predictor 1 */
    CAL_DPCM_ENC_MAX = 0x17U
    /**< End marker */
} Cal_PixDpcmEncoder_t;    /**< Cal_PixDpcmEncoder_t */

/**
 *  enum Cal_PixPack
 *  \brief Valid pixel packing supported.
 *
 */
typedef enum Cal_PixPack
{
    CAL_PIX_PACK_B8 = 0,
    /**< 8 bit packing */
    CAL_PIX_PACK_B10_MIPI = 0x2U,
    /**< 10 bit mipi packing */
    CAL_PIX_PACK_B12 = 0x3U,
    /**< 12 bit packing */
    CAL_PIX_PACK_B12_MIPI = 0x4U,
    /**< 12 bit mipi packing */
    CAL_PIX_PACK_B16 = 0x5U,
    /**< 16 bit packing */
    CAL_PIX_PACK_ARGB = 0x6U,
    /**< ARGB packing, 8 x 3 packing, followed by 8 bit padding */
    CAL_PIX_PACK_MAX = 0x7U
    /**< End Marker */
} Cal_PixPack_t;   /**< Cal_PixPack_t */

/**
 *  enum Cal_StreamType
 *  \brief Identifies different types streams.
 *
 */
typedef enum Cal_StreamType
{
    CAL_TAG_ATT_HDR = 0,
    /**< Attribute packet headers */
    CAL_TAG_ATT_DATA,
    /**< Attribute data */
    CAL_TAG_CTRL,
    /**< Control packets */
    CAL_TAG_PIX_HDR,
    /**< Pixel packet data headers */
    CAL_TAG_PIX_DATA,
    /**< Pixel Data */
    CAL_TAG_MAX
    /**< End marker */
} Cal_StreamType_t; /**< Cal_StreamType_t */

/**
 *  \brief Lists the possible error source in CAL reception. Applications
 *          could enable any of these and attach a function to be called on
 *          error (s)
 *          When enabled, the frame would marked with appropriate error code
 *          to indicate an issue.
 *
 *  \warning If any elements are added/deleted from this structure, ensure to
 *              update CAL_CAPT_MAX_ERROR_INTERRUPTS
 */
typedef enum Cal_ErrorSource
{
    /* Following events correspond to IEM_CAL_EVENT_PPIO */
    CAL_CSI2_PPI_CMPLXIO_ERRSOTHS1 = 0,
    /**< Not supported for now ! */
    CAL_CSI2_PPI_CMPLXIO_FIFO_OVR = 27,
    /**< Event triggered when CAL un able to process the received data.
            Typically down stream modules (including write DMA) is unable to
            keep up with the speed of reception of data. */
    CAL_CSI2_PPI_CMPLXIO_ECC_NO_CORRECTION = 30,
    /**< ECC module was unable to correct, as there were more than 1 bit errors.
            Applicable for all virtual channels */
    CAL_CSI2_PPI_CMPLXIO_RSERVED_31 = 31,
    /**< Reserved event, will be used for boundary checks */

    /* Following events correspond to IEM_CAL_EVENT_PPIO_VC */
    CAL_CSI2_PPI_VC_SOF1 = 128 + 0,
    /**< Not an error, Will be used for boundary checks / debug. */
    CAL_CSI2_PPI_VC_CRC_MISMATCH_VC1 = 128 + 4,
    /**< CRC did not match, the frame could be BAD */
    CAL_CSI2_PPI_VC_ECC_CORRECTION_VC1 = 128 + 5,
    /**< ECC module was able to correct 1 bit error */
    CAL_CSI2_PPI_VC_SOF2 = 128 + 8,
    /**< Not an error, Will be used for boundary checks / debug. */
    CAL_CSI2_PPI_VC_CRC_MISMATCH_VC2 = 128 + 12,
    /**< CRC did not match, the frame could be BAD */
    CAL_CSI2_PPI_VC_ECC_CORRECTION_VC2 = 128 + 13,
    /**< ECC module was able to correct 1 bit error */
    CAL_CSI2_PPI_VC_SOF3 = 128 + 16,
    /**< Not an error, Will be used for boundary checks / debug. */
    CAL_CSI2_PPI_VC_CRC_MISMATCH_VC3 = 128 + 20,
    /**< CRC did not match, the frame could be BAD */
    CAL_CSI2_PPI_VC_ECC_CORRECTION_VC3 = 128 + 21,
    /**< ECC module was able to correct 1 bit error */
    CAL_CSI2_PPI_VC_SOF4 = 128 + 24,
    /**< Not an error, Will be used for boundary checks / debug. */
    CAL_CSI2_PPI_VC_CRC_MISMATCH_VC4 = 128 + 28,
    /**< CRC did not match, the frame could be BAD */
    CAL_CSI2_PPI_VC_ECC_CORRECTION_VC4 = 128 + 29,
    /**< ECC module was able to correct 1 bit error */
    CAL_CSI2_PPI_VC_RESERVED_31 = 128 + 31,
    /**< Reserved event, will be used for boundary checks */

    CAL_BYSIN_OVR = CAL_CSI2_PPI_VC_RESERVED_31 + 1,
    /**< When CAL is used to receive via BYS IN (Parallel input) and the down
            stream modules could not keep up with the incoming data rate, the
            BYS IN port could overflow. */
    CAL_CSI2_FORCE_INT = 0x7FFFFFFF
    /**< This will ensure enum is not packed,
     *      will always be contained in int
     */
} Cal_ErrorSource_t; /**< Cal_ErrorSource_t */

/* ========================================================================== */
/*                                 Function Types                             */
/* ========================================================================== */
/**
 *  \brief Defines expected function type that could be called by drivers on
 *          error conditions. Please refer driver specific header files on
 *          methods to attach callbacks on errors.
 *
 *  \param  event       Pointer to an array that holds event (s) that have
 *                          occurred. The numEvents determine the number of
 *                          elements that could accessed.
 *  \param  numEvents   Number of events (error conditions) that were detected.
 *  \param  arg         Argument provided while configuring the error parameters
 */
typedef void (*Cal_ErrorCallBack)(const uint32_t *event,
                                  uint32_t        numEvents,
                                  void           *arg);

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
 *  \brief Resources allocated instances. 0xFFFFFFFF in case not allocated.
 *          Otherwise valid instance number
 *
 *  \warning Deviation from naming convention: To enable backward compatibility
 *
 */
typedef struct Cal_CaptBlocks
{
    uint32_t ppi0Inst;
    /**< PPI interface instance,  0xFFFFFFFF in case not allocated. Otherwise
     *      valid instance number */
    uint32_t ppi1Inst;
    /**< PPI Instace 1 */
    uint32_t pixExtract;
    /**< Pixel Extract instance */
    uint32_t dpmDecode;
    /**< DPM Decode instance */
    uint32_t dpmEncode;
    /**< DPM Encode instance */
    uint32_t pixPack;
    /**< Pixel Pack instance */
    uint32_t bysOut;
    /**< BYS Out instance */
    uint32_t bysIn;
    /**< BYS IN instance */
    uint32_t vPort;
    /**< VPORT IN instance */
    uint32_t rdDma;
    /**< RD Dma instance */
    uint32_t wrDma;
    /**< WR Dma instance */
    uint32_t csi2Ctx;
    /**< CSI2 processing context */
    uint32_t cport;
    /**< CPort ID */
    uint32_t lvds;
    /**< LVDS port */
    uint32_t cpi;
    /**< Parallel / CPI port */
} Cal_CaptBlocks_t;

/**
 *  struct Cal_CmplxIoLaneCfg
 *  \brief Configure the position and order of lane for the complex IO.
 *
 */
typedef struct Cal_CmplxIoLaneCfg
{
    uint32_t pol;
    /**< TRUE configures for - / + order of differential signal.
     + / - order otherwise */
    uint32_t position;
    /**< Specify if this lane is to be used, if so, on which position.
     *  0x0 - Not used / disabled lane
     *  0x1 - Position 1
     *  0x2 - Position 2
     *  0x3 - Position 3
     *  0x4 - Position 4
     *  0x5 - Position 5 */
} Cal_CmplxIoLaneCfg_t;

/**
 *  struct Cal_CmplxIoCfg
 *  \brief Configure complex IO.
 *
 */
typedef struct Cal_CmplxIoCfg
{
    uint32_t             enable;
    /**< FALSE disables, otherwise indicates the instance to be used. Valid
     *  values are 1 and 2 */
    Cal_CmplxIoLaneCfg_t clockLane;
    /**< Configure position and order of differential signal */
    Cal_CmplxIoLaneCfg_t data1Lane;
    /**< Configure position and order of differential signal */
    Cal_CmplxIoLaneCfg_t data2Lane;
    /**< Configure position and order of differential signal */
    Cal_CmplxIoLaneCfg_t data3Lane;
    /**< Configure position and order of differential signal */
    Cal_CmplxIoLaneCfg_t data4Lane;
    /**< Configure position and order of differential signal */
    uint32_t             pwrAuto;
    /**< Auto power control mode of complex IO. TRUE configures it to enable
     *  auto power mode (in which power switches to ON / OFF based on ULP / ULPM
     *  commands from the sensor.
     *  Always powered on otherwise */
} Cal_CmplxIoCfg_t;

/**
 *  struct Cal_PixProc
 *  \brief CAL Pixel Processing configuration control.
 */
typedef struct Cal_PixProc
{
    Cal_PixExtract_t     extract;
    /**< Configure bit extraction */
    Cal_PixDpcmDecoder_t decCodec;
    /**< Decoder required */
    uint32_t             enableDpcmInitContext;
    /**< TRUE configure DPCM Decode to loaded from a specific memory location*/
    uint32_t             addr;
    /**< Address of word describing DPCM init context. Should be 16 bytes
     *      aligned */
    uint32_t             offSet;
    /**< Address offset for UV data, when data type if YUV420 */

    Cal_PixDpcmEncoder_t encCodec;
    /**< Encoder required */
    Cal_PixPack_t        pack;
    /**< Packing required */

    uint32_t             contextToBeUsed;
    /**< RESERVED - Internal to driver. Value specified here is ignored.
     *      Context of pixel processing to be used. */
} Cal_PixProc_t;

/**
 *  struct Cal_BysOut
 *  \brief CAL BYS Out configuration control.
 */
typedef struct Cal_BysOut
{
    uint32_t enable;
    /**< TRUE, enables stream on BYS Out */
    uint32_t pixClock;
    /**< Configure the pixel clock required. A non zero value will ensure that
     *      BYS ports pixel clock will be gated when idle. */
    uint32_t yBlk;
    /**< Configure the Y Blank to be inserted at end of frame */
    uint32_t xBlk;
    /**< Configures the X blank to be inserted at end of line */
    uint32_t copyStreamToEncode;
    /**< TRUE configures, streams sent out on BYS out to be copied into encode*/
    uint32_t freeRun;
    /**< TRUE configures Bys out in Free running mode, False gated in idle */
} Cal_BysOut_t;

/**
 *  struct Cal_VPort
 *  \brief CAL Video Port configuration control.
 */
typedef struct Cal_VPort
{
    uint32_t enable;
    /**< TRUE, enables stream on Video Port input */
    uint32_t pixClock;
    /**< Configure the pixel clock required.
     *      When disabled pixlClock will be 0x0. */
    uint32_t width;
    /**< 0, indicates 1 pixel / clock cycle. Else 2 pixels / clock cycle */
    uint32_t yBlk;
    /**< Configure the Y Blank to be inserted at end of frame */
    uint32_t xBlk;
    /**< Configures the X blank to be inserted at end of line */
    uint32_t rdyThr;
    /**< Configure the initial threashold value, before the first pixel is
     *  recevied and passed for furthure proessing */
    uint32_t fsReset;
    /**< TRUE ReSet the timing generator FSM on every end of frame received,
     *      otherwise data processed normally */
    uint32_t freeRun;
    /**< TRUE enables the pixel clock during idle period. FALSE gates the same.
     */
} Cal_VPort_t;

/**
 *  struct Cal_Cfg
 *  \brief CAL config, used to control all the sub-modules of CAL.
 *
 *  \attention  Ensure to allocate before trying to configure.
 *              There is only one instance of BYSOUT, BYSIN and VPORT and hence
 *                  can support only 1 channel. When these are used for
 *                  multiple channels, the last config is valid
 *
 *  \attention When non-CSI2 streams are being captured, following members
 *              are not applicable, stream, csi2VirtualChanNo, pixProcCfg &
 *              bysOutCfg.
 */
typedef struct Cal_Cfg
{
    uint32_t numStream;
    /**< Specify the number of streams that requires to be configured. Minimum
     *      should be 1 and maximum could be CAL_CAPT_MAX_STREAMS */
    uint32_t            streamId[CAL_CAPT_MAX_STREAMS];
    /**< Specify the stream ID, that requires to be configured.
     *      e.g. If first stream requires to be configured
     *      \code numStream = 1U;
     *            streamId[0] = 0U;
     *            inFmt[0].width = 1280U;
     *            inFmt[0].width = 800U;
     *      etc...
     *      e.g. if there are 3 streams and require to configure 2 of them, say
     *              stream 2 and 0
     *            numStream = 2U;
     *            streamId[0] = 0U;
     *            inFmt[0].width = 1280U;
     *            inFmt[0].width = 800U;
     *            streamId[1] = 2U;
     *            inFmt[1].width = 1280U;
     *            inFmt[1].width = 720U; \endcode */
    Fvid2_Format        inFmt[CAL_CAPT_MAX_STREAMS];
    /**< Specify the characteristics of streams that has to be received.
     *      Valid member of this structure are
     *      .width  - specify the width if known, else 0x0.
     *                  WARNING - Sufficient buffer should be allocated to
     *                      accommodate max line length.
     *      .height - expected number of lines, 0x0 for unknown
     *                  WARNING - Sufficient buffer should be allocated to
     *                      accommodate max lines.
     *      .pitch  - pitch
     *      .bpp    - Bits per pixel #Fvid2_BitsPerPixel
     *      .dataFormat - dataformat as per #Fvid2_DataFormat
     *                      \attention In case of CSI2 streams, this is not
     *                      valid. Please use #csi2DataFormat */
    Cal_StreamType_t    streamType[CAL_CAPT_MAX_STREAMS];
    /**< Applicable when capturing via CSI2 interface only.
     *      Specify the streams that required to be captured */
    Cal_Csi2DataFormat  csi2DataFormat[CAL_CAPT_MAX_STREAMS];
    /**< Applicable when capturing via CSI2 interface only.
     *      Specify the streams data format */
    uint32_t            csi2VirtualChanNo[CAL_CAPT_MAX_STREAMS];
    /**< Applicable when capturing via CSI2 interface only.
     *      Specify the virtual channel number to be used. Valid rage 0 - 3 */
    uint32_t            isPixProcCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid pixel processing config, otherwise invalid config
     *      not used. */
    Cal_PixProc_t       pixProcCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure pixel processing contexts. There are 4 pixel processing
     *      contexts available. */

    /* NOTE there is one instance of BYS Out, BYS In and Video Port.
     *  At any point only one configuration should be valid, otherwise, the last
     *  config would be active. */
    uint32_t            isBysOutCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid BYS Out config, otherwise invalid config
     *        not used. */
    Cal_BysOut_t        bysOutCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure BYS Out */

    uint32_t            bysInEnable[CAL_CAPT_MAX_STREAMS];
    /**< Configure BYS In. */

    uint32_t            isVportCfgValid[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates valid Video Port In config, otherwise invalid config
     *        not used. */
    Cal_VPort_t         vportCfg[CAL_CAPT_MAX_STREAMS];
    /**< Configure Video Port */

    uint32_t            writeToMem[CAL_CAPT_MAX_STREAMS];
    /**< TRUE indicates that the captured stream should be written to memory.
     *      \attention Note that a stream could not be duplicated, e.g. if
     *          you require to send out a stream via VPORT and also to write
     *          into memory, its not possible. */

    uint32_t            cmplxIoId[CAL_CAPT_MAX_STREAMS];
    /**< PPI Instance to be used for this stream,
         Must be less than CAL_CAPT_MAX_CMPLXIO_INST */

    void               *pAdditionalArgs;
    /**< Not used for now - should be set to NULL */
} Cal_Cfg_t;

/**
 *  struct Cal_ErrorCfg
 *  \brief
 *
 */
typedef struct Cal_ErrorCfg
{
    uint32_t            cmplxIoId;
    /**< Complex IO index,
         must be less than CAL_CAPT_MAX_CMPLXIO_INST */
    uint32_t            numErrorsToMonitor;
    /**< Specify the number of errors that driver should monitor/enable */
    Cal_ErrorSource_t   errSrc[CAL_CAPT_MAX_ERROR_INTERRUPTS];
    /**< Identify the error that have to be enabled */
    Cal_ErrorCallBack   appCb;
    /**< A function pointer, that would be called on error conditions with
            following arguments
            const uint32_t *event     : Will point to array that hold actual error
                                        of type Cal_ErrorSource_t and will
                                        have minimum 'numEvents' entries
            uint32_t        numEvents : A positive value that indicate number of
                                        errors detected
            void         *arg       : A pointer provided by application while
                                        enabling the interrupt 'pAppCbArgs'

            \attention The appCb could be NULL. In which case the frame status
                        would be updated. When appCb is not NULL, the provided
                        function would be called and frame status would be
                        updated.
        */
    void               *pAppCbArgs;
    /**< Argument that would be passed when appCb is called. */
    void               *pAdditionalArgs;
    /**< Not used for now - should be set to NULL */
} Cal_ErrorCfg_t;

/**
 *  struct Cal_FrameEventNotifyCfg
 *
 *  \brief CAL could be configured to notify application on end of frame or on
 *          reception of first X lines.
 *
 *  Limitation on first x lines callback : Currently this can be enabled for
 *      one stream and stream should be of type CAL_TAG_PIX_DATA
 *
 */
typedef struct Cal_FrameEventNotifyCfg
{
    uint32_t            numStream;
    /**< Number of streams, which requires to be monitored for end of frame,
            and or first X lines */
    uint32_t            streamId[CAL_CAPT_MAX_STREAMS];
    /**< Specify the stream ID, which requires to be monitored */
    uint32_t            notifyAfterFirstXLines[CAL_CAPT_MAX_STREAMS];
    /**< Will call the application provided callback, after reception of
        number of lines specified here.
        Restriction :
            This can be supported for only CAL_CAPT_MAX_X_LINE_MONITOR_CNT
            virtual channel */
    uint32_t            notifyAfterEndOfFrame[CAL_CAPT_MAX_STREAMS];
    /**< Will call the application provided callback, after reception of
        End Of Frame short packet */
    Fvid2_SubFrameCbFxn appCb;
    /**< Application callback */
    void               *pAdditionalArgs;
    /**< Not used for now - should be set to NULL */
} Cal_FrameEventNotifyCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 *  CalCfg_init
 *  \brief This function should be used to initialize variable of type
 *          #Cal_Cfg_t.
 *
 *  \param cfg   A pointer of type Cal_Cfg_t
 *  \return      None
 */
static inline void CalCfg_init(Cal_Cfg_t *cfg);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline void CalCfg_init(Cal_Cfg_t *cfg)
{
    if (NULL != cfg)
    {
        memset(cfg, 0x0, sizeof (Cal_Cfg_t));
    }
}

#ifdef __cplusplus
}
#endif

#endif /* CAL_CFG_H_ */

/* @} */
