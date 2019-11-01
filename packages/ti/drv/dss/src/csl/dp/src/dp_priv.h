/**********************************************************************
* Copyright (C) 2012-2019 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.03.9e11b77(origin/DRV-3827_extract_sanity_to_c_file)
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for the Cadence DisplayPort (DP) core. This header
* file lists the API providing a HAL (hardware abstraction layer)
* interface for the DP core
**********************************************************************/

#ifndef DP_PRIV_H
#define DP_PRIV_H

/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */
/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure" */

#include "dp_if.h"
#include "dp_mst_if.h"
#include "dp_mst_structs_if.h"
#include "dp_sd0801_structs_if.h"
#include "dp_sideband_msg_if.h"
#include "dp_sideband_msg_structs_if.h"
#include "dp_structs_if.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** Used to enable/disable HPD events */
#define DP_HPD_EVENT_ENABLE_BIT (1U)

#define DP_LINK_TRAINING_CR_DELAY_US 100U

#define DP_FAST_LINK_TRAINING_DELAY_US 500U

/** relative to 0x0000 or 0x2200 */
#define DP_TRAINING_AUX_RD_INTERVAL_OFF 0x0EU

#define DP_TRAINING_AUX_RD_INTERVAL_MASK 0x7FU

#define DP_EXT_RECV_CAP_FIELD_PRESENT_MASK 0x80U

/** relative to 0x0000 or 0x2200 */
#define DP_MAX_LINK_RATE_OFF 0x01U

/** relative to 0x0000 or 0x2200 */
#define DP_MAX_LANE_COUNT_OFF 0x02U

#define DP_MAX_LANE_COUNT_MASK 0x0FU

#define DP_TPS3_SUPPORTED_MASK 0x40U

#define DP_ENHANCED_FRAME_CAP_MASK 0x80U

/** relative to 0x0000 or 0x2200 */
#define DP_MAX_DOWNSPREAD_OFF 0x03U

#define DP_MAX_DOWNSPREAD_MASK 0x01U

#define DP_NO_AUX_TRAINING_MASK 0x40U

#define DP_TPS4_SUPPORTED_MASK 0x80U

/** relative to 0x0000 or 0x2200 */
#define DP_ML_CODING_OFF 0x06U

#define DP_CAPABLE_8B10B_MASK 0x01U

/** relative to 0x0000 or 0x2200 */
#define DP_EDP_CONF_CAP_OFF 0x0DU

#define DP_ASSR_CAP_MASK 0x01U

#define DP_SPREAD_AMP_MASK 0x10U

#define DP_SET_8B_10B_MASK 0x01U

#define DP_VOLTAGE_SWING_MASK 0x03U

#define DP_MAX_SWING_REACHED_MASK 0x04U

#define DP_PREEMPHASIS_SET_SHIFT 3U

#define DP_PREEMPHASIS_SET_MASK 0x18U

#define DP_MAX_PREEMPHASIS_REACHED_MASK 0x20U

#define DP_LANE_CR_DONE_MASK 0x01U

#define DP_LANE_EQ_DONE_MASK 0x02U

#define DP_LANE_SYMBOL_LOCKED_MASK 0x04U

#define DP_LANE_STATUS_ODD_LANE_SHIFT 4U

#define DP_INTERLANE_ALIGN_DONE_MASK 0x01U

#define DP_VOLTAGE_SWING_ADJ_MASK 0x03U

#define DP_PREEEMPHASIS_ADJ_SHIFT 2U

#define DP_PREEEMPHASIS_ADJ_MASK 0x0CU

#define DP_LANE_COUNT_SET_MASK 0x1FU

#define DP_ENHANCED_FRAME_EN_MASK 0x80U

#define DP_SCRAMBLING_DISABLE_MASK 0x20U

/** Address of DPCD register for configuring eDP */
#define DP_EDP_CONF_REG 0x10AU

#define DP_ASSR_ENABLE_MASK 0x01U

#define DP_SCRAMBLER_SEED_REGULAR 0xFFFFU

#define DP_SCRAMBLER_SEED_ALTERNATE 0xFFFEU

/** FEC overhead, in % */
#define DP_FEC_OVERHEAD 2.4

#define DP_GENERAL_TEST_ECHO_MAX_PAYLOAD (100U)

#define DP_GENERAL_TEST_ECHO_MIN_PAYLOAD (1U)

#define DP_MAX_MAILBOX_PAYLOAD (1019U)

#define DP_HDCP_TX_STATUS_SIZE (5U)

#define DP_IP_PART_NUMBER 0x8546U

#define DP_IP_VERSION 0x2000U

#define DP_AUX_VERSION 0x1000U

#define DP_PHY_VERSION_1100 0x1100U

#define DP_PHY_VERSION_1200 0x1200U

/**
 *  @}
 */

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct DP_DscConfigFull_s DP_DscConfigFull;
typedef struct DP_HardwareConfig_s DP_HardwareConfig;

/**********************************************************************
* Structures and unions
**********************************************************************/
/** DSC full configuration containing input parameters, calculated parameters and fixed parameters necessary to be put to PPS. Private structure used internally by driver. */
struct DP_DscConfigFull_s
{
    /** Bits / component for previous reconstructed line buffer */
    uint32_t linebufDepth;
    /** Bits / component to code */
    uint32_t bitsPerComponent;
    /** slice width in pixels */
    uint32_t sliceWidth;
    /** slice height in pixels */
    uint32_t sliceHeight;
    /** picture width */
    uint32_t picWidth;
    /** picture height */
    uint32_t picHeight;
    /** Offset to bits/group used by RC to determine QP adjustment */
    uint32_t rcTgtOffsetHi;
    /** Offset to bits/group used by RC to determine QP adjustment */
    uint32_t rcTgtOffsetLo;
    /** Bits/pixel target << 4 (ie., 4 fractional bits) */
    uint32_t bitsPerPixel;
    /** Factor to determine if an edge is present based on the bits produced */
    uint32_t rcEdgeFactor;
    /** Slow down incrementing once the range reaches this value */
    uint32_t rcQuantIncrLimit1;
    /** Slow down incrementing once the range reaches this value */
    uint32_t rcQuantIncrLimit0;
    /** Number of pixels to delay the initial transmission */
    uint32_t initialXmitDelay;
    /** Block prediction range (in pixels) */
    uint32_t blockPredEnable;
    /** Value to use for RC model offset at slice start */
    uint32_t initialOffset;
    /** Thresholds defining each of the buffer ranges */
    uint32_t rcBufThresh[DP_DSC_NUM_BUF_RANGES - 1U];
    /** Parameters for each of the RC ranges */
    DP_DscRangeCfg rcRangeParameters[DP_DSC_NUM_BUF_RANGES];
    /** Total size of RC model */
    uint32_t rcModelSize;
    /** Minimum QP where flatness information is sent */
    uint32_t flatnessMinQp;
    /** Maximum QP where flatness information is sent */
    uint32_t flatnessMaxQp;
    /** Enable on-off VBR (ie., disable stuffing bits) */
    uint32_t vbrEnable;
    /** Placeholder for PPS identifier */
    uint32_t ppsIdentifier;
    /**
     * When 'true', both encoders are used in parallel for one video stream (L and R split).
     * When 'false', one encoder is used.
     */
    bool splitPanel;
    /** Number of lines to wait before initiating transport in Command Mode */
    uint32_t initialLines;
    /** the upstream source timing controller total line time (in clock cycles, not in pixels) */
    uint32_t hTotal;
    /** Flatness Detection Threshold */
    uint32_t flatnessDetThresh;
    /** Number of pixels to delay the VLD on the decoder, not including SSM */
    uint32_t initialDecDelay;
    /** Bits/group offset to use for first line of the slice */
    uint32_t firstLineBpgOfs;
    /** Initial value for scale factor */
    uint32_t initialScaleValue;
    /** Decrement scale factor every scaleDecrementInterval groups */
    uint32_t scaleDecrementInterval;
    /** Increment scale factor every scaleIncrementInterval groups */
    uint32_t scaleIncrementInterval;
    /** non-first line BPG offset to uses */
    uint32_t nflBpgOffset;
    /** non-second line BPG offset to uses */
    uint32_t nslBpgOffset;
    /** BPG offset used to enforce slice bit constraint */
    uint32_t sliceBpgOffset;
    /** final RC linear transformation offset value */
    uint32_t finalOffset;
    /** The (max) size in bytes of the "chunks" that are used in slice multiplexing */
    uint32_t chunkSize;
    /** For 4:2:0, bits/group offset to use for the 2nd line of the slice */
    uint32_t secondLineBpgOfs;
    /** DSC minor version */
    uint32_t dscVersionMinor;
    /** 4:2:2 simple mode (from PPS, 4:2:2 conversion happens outside of DSC encode/decode algorithm) */
    uint32_t simple_422;
    /** 4:2:2 native mode (no conversion is done) */
    uint32_t native_422;
    /** 4:2:0 native mode (no conversion is done) */
    uint32_t native_420;
    /** Flag indicating to do RGB - YCoCg conversion and back (should be 1 for RGB input) */
    uint32_t convertRgb;
    /** Adjustment to offset for 2nd line in 4:2:0 (since chroma has no prediction) */
    uint32_t secondLineOfsAdj;
};

/** Hardware configuration parameters. */
struct DP_HardwareConfig_s
{
    /**
     * Number identifying the IP. Corresponds to Cadence IP Part Number.
     *
     */
    uint32_t ipPartNumber;
    /**
     * Number identifying the IP version.
     *
     */
    uint16_t ipVersion;
    /**
     * IP Family Code. 0x00: Display TX Controller, 0x01: Display RX Controller
     *
     */
    uint8_t ipFamilyCode;
    /**
     * Main configuration type.
     *
     */
    uint8_t mainConfigType;
    /**
     * AUX version.
     *
     */
    uint16_t auxVersion;
    /**
     * PHY version.
     *
     */
    uint16_t phyVersion;
    /**
     * Number identifying AUX type.
     *
     */
    uint16_t auxType;
    /**
     * Number identifying PHY type.
     *
     */
    uint16_t phyType;
    /**
     * DSC supported.
     *
     */
    bool dscSupport;
    /**
     * ASF supported.
     *
     */
    bool asfSupport;
    /**
     * Number of video streams.
     *
     */
    uint8_t videoStreams;
    /**
     * Number of audio streams.
     *
     */
    uint8_t audioStreams;
};

/**
 * Structure contains private data for Core Driver that should not be used by
 * upper layers. This is not a part of API and manipulation of those data may cause
 * unpredictable behaviour of Core Driver.
 */
struct DP_PrivateData_s
{
    /** Base address of the register space. */
    struct MHDP_ApbRegs_s* regBase;
    /** Base address of the SAPB (Secure APB) register space. */
    struct MHDP_ApbRegs_s* regBaseSapb;
    /** Pointer to PHY driver's private data */
    DP_SD0801_PrivateData* phyPd;
    /** Structure to pointers to callback functions. */
    DP_Callbacks cb;
    /** buffer for transmitting data to FW via mailbox. */
    uint8_t txBuffer[1024];
    /** buffer for receiving data from FW via mailbox. */
    uint8_t rxBuffer[1024];
    /** TX iterations */
    uint32_t txi;
    /** RX iterations */
    uint32_t rxi;
    /** data ready to send */
    uint8_t txEnable;
    /** data ready to receive */
    uint8_t rxEnable;
    uint8_t running;
    DP_BusType busType;
    uint32_t tmp;
    /** structure used to store capabilities of source (TX) device. */
    DP_SourceDeviceCapabilities sourceCaps;
    /** structure used to store capabilities of sink (RX) device. */
    DP_SinkDeviceCapabilities sinkCaps;
    /** Indicates (when !=0), if source capabilities are currently stored. */
    uint8_t sourceCapsStored;
    /** Indicates (when !=0), if sink capabilities are currently stored. */
    uint8_t sinkCapsStored;
    /** structure used to store current link state. */
    DP_LinkState linkState;
    /** Last value of register incremented by FW, used to determine, if it's alive */
    uint8_t lastAlive;
    /** Current dsc configuration */
    DP_DscConfigFull dscConfig[DP_NUMBER_OF_DSC_ENCODERS];
    /** Current video configuration for each of stream */
    DP_VideoParameters videoParameters[DP_MAX_NUMBER_OF_STREAMS];
    /** Whether or not FEC is enabled */
    bool fecEnabled;
    /** Whether or not MST feature is enabled */
    bool mstEnabled;
    /** Whether or not video format-related parameters were set for SST mode. */
    bool sstVicSet;
    /** Whether or not particular MST stream is enabled */
    bool streamEnabled[DP_MAX_NUMBER_OF_STREAMS];
    /** MST topology manager */
    MST_drm_dp_topology_mgr mstTopMgr;
    /** List of connected sink devices */
    DP_SinkDevice sinkList[DP_MAX_NUMBER_OF_STREAMS];
    /** SDP packet type */
    uint8_t sdpPacketType[DP_MAX_NUMBER_OF_STREAMS][DP_MAX_NUMBER_OF_SDPS];
    /** Hardware configuration parameters structure. */
    DP_HardwareConfig hwConfig;
};

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */

#endif  /* DP_PRIV_H */
