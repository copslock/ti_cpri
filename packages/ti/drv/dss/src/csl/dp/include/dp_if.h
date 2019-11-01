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

#ifndef DP_IF_H
#define DP_IF_H

/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "custom_types.h"
#include "dp_aux_if.h"
#include "dp_mst_if.h"
#include "dp_sd0801_if.h"
#include "dp_sideband_msg_if.h"
#include "dp_topology_mgr.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
/** Max. possible amount of DPCD bytes to write/read using a single request. */
#define DP_MAX_DPCD_TRANSFER_SIZE 1014U

/** Max. possible DPCD read retries before bailing out */
#define DP_MAX_DPCD_READ_RETRIES 3U

/**
 * Max. possible amount of bytes to write/read using I2C-over-AUX using a
 * single request.
 */
#define DP_MAX_I2C_TRANSFER_SIZE 1014U

/** Number of voltage swing levels. */
#define DP_SWING_LEVEL_COUNT 4U

/** Number of pre-emphasis levels. */
#define DP_EMPHASIS_LEVEL_COUNT 4U

#define DP_DSC_NUM_BUF_RANGES (15U)

#define DP_DSC_PPS_SIZE (128U)

/** number of existing encoders */
#define DP_NUMBER_OF_DSC_ENCODERS (2U)

/** maximum number of SDP packets per stream */
#define DP_MAX_NUMBER_OF_SDPS (16U)

/** maximum number of streams in MST mode */
#define DP_MAX_NUMBER_OF_STREAMS (4U)

#define DP_HDCP2_PUBLIC_KEY_N_LENGTH (384U)

#define DP_HDCP2_PUBLIC_KEY_E_LENGTH (3U)

#define DP_HDCP_CUSTOM_KEY_LENGTH (16U)

#define DP_HDCP_RECV_ID_LENGTH (5U)

#define DP_HDCP1_KEY_SET_LENGTH (280U)

#define DP_HDCP_M_LENGTH (16U)

#define DP_HDCP_KM_LENGTH (16U)

#define DP_HDCP_RANDOM_RN_LENGTH (8U)

#define DP_HDCP_RANDOM_KS_LENGTH (16U)

#define DP_HDCP_RANDOM_RIV_LENGTH (8U)

#define DP_HDCP_RANDOM_RTX_LENGTH (8U)

#define DP_HDCP_LC128_LENGTH (16U)

#define DP_HDCP_SEED_LENGTH (32U)

#define DP_MAX_NUMBER_OF_LANES (4U)

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
typedef struct DP_VideoFormatParams_s DP_VideoFormatParams;
typedef struct DP_SourceDeviceCapabilities_s DP_SourceDeviceCapabilities;
typedef struct DP_SinkDeviceCapabilities_s DP_SinkDeviceCapabilities;
typedef struct DP_ReadEdidResponse_s DP_ReadEdidResponse;
typedef struct DP_DpcdTransfer_s DP_DpcdTransfer;
typedef struct DP_I2cTransfer_s DP_I2cTransfer;
typedef struct DP_VideoParameters_s DP_VideoParameters;
typedef struct DP_LinkState_s DP_LinkState;
typedef struct DP_SdpEntry_s DP_SdpEntry;
typedef struct DP_HdcpTxConfiguration_s DP_HdcpTxConfiguration;
typedef struct DP_Hdcp2TxPublicKey_s DP_Hdcp2TxPublicKey;
typedef struct DP_HdcpTxKmEncCustomKey_s DP_HdcpTxKmEncCustomKey;
typedef struct DP_HdcpDebugRandomNumbers_s DP_HdcpDebugRandomNumbers;
typedef struct DP_HdcpPairingData_s DP_HdcpPairingData;
typedef struct DP_Hdcp1Keys_s DP_Hdcp1Keys;
typedef struct DP_HdcpTxStatus_s DP_HdcpTxStatus;
typedef struct DP_HdcpRecvIdList_s DP_HdcpRecvIdList;
typedef struct DP_AudioParams_s DP_AudioParams;
typedef struct DP_DscRangeCfg_s DP_DscRangeCfg;
typedef struct DP_DscConfig_s DP_DscConfig;
typedef struct DP_Config_s DP_Config;
typedef struct DP_Callbacks_s DP_Callbacks;
typedef struct DP_SinkDevice_s DP_SinkDevice;
typedef struct DP_FirmwareImage_s DP_FirmwareImage;
typedef struct DP_UcpuClock_s DP_UcpuClock;
typedef struct DP_AudioVideoClkCfg_s DP_AudioVideoClkCfg;

typedef struct DP_PrivateData_s DP_PrivateData;

/**********************************************************************
* Enumerations
**********************************************************************/
typedef enum
{
    /** Set sink device to D0 power state (normal operation) */
    DP_POWER_NORMAL_OPERATION = 1U,
    /** Set sink device to D3 power state (power down) */
    DP_POWER_DOWN = 2U,
    /** Similar to POWER_DOWN, but keep AUX block fully powered. */
    DP_POWER_AUX_ONLY = 5U
} DP_PwrMode;

typedef enum
{
    DP_LINK_RATE_1_62 = 0x00U,
    DP_LINK_RATE_2_16 = 0x01U,
    DP_LINK_RATE_2_43 = 0x02U,
    DP_LINK_RATE_2_70 = 0x03U,
    DP_LINK_RATE_3_24 = 0x04U,
    DP_LINK_RATE_4_32 = 0x05U,
    DP_LINK_RATE_5_40 = 0x06U,
    DP_LINK_RATE_8_10 = 0x07U
} DP_LinkRate;

/** Specifies, how physical PHY lanes are mapped to ports of receiver(s) */
typedef enum
{
    /** Single controller is used. Lanes are not crossed (0->0, 1->1, 2->2, 3->3). */
    DP_LANE_MAPPING_SINGLE_REGULAR = 0xE4U,
    /**
     * Up to two controllers are used with a PHY. Currently configured
     * controller uses only up to 2 lanes: lane 0 and lane 1. (0->0, 1->1)
     */
    DP_LANE_MAPPING_DUAL_LANES_01 = 0x04U,
    /**
     * Up to two controllers are used with a PHY. Currently configured
     * controller uses only up to 2 lanes: lane 2 and lane 3. (0->2, 1->3)
     */
    DP_LANE_MAPPING_DUAL_LANES_23 = 0x0EU
} DP_LaneMapping;

/** Number of controllers per PHY. */
typedef enum
{
    /** Only one controller is used with PHY. */
    DP_SINGLE_CONTROLLER = 0x01U,
    /** Two controllers are used with PHY. */
    DP_DUAL_CONTROLLER = 0x02U
} DP_ControllersPerPhy;

/** Selects test pattern to be transmitted. */
typedef enum
{
    /** Transmit PRBS7 pattern. */
    DP_PATTERN_PRBS7 = 0x00U,
    /** Transmit TPS1 pattern. */
    DP_PATTERN_TPS1 = 0x01U,
    /** Transmit TPS2 pattern. */
    DP_PATTERN_TPS2 = 0x02U,
    /** Transmit TPS3 pattern. */
    DP_PATTERN_TPS3 = 0x03U,
    /** Transmit TPS4 pattern. */
    DP_PATTERN_TPS4 = 0x04U,
    /** Transmit custom 80-bit pattern (set using DP_SetCustomPattern). */
    DP_PATTERN_80_BIT = 0x05U,
    /** Transmit D10.2 pattern (same as TPS1). */
    DP_PATTERN_D10_2 = 0x06U,
    /** Transmit Symbol Error Rate Measurement pattern. */
    DP_PATTERN_SYMBOL_ERM = 0x07U,
    /** Transmit CP2520 pattern 1. */
    DP_PATTERN_CP2520_1 = 0x08U,
    /** Transmit CP2520 pattern 2. */
    DP_PATTERN_CP2520_2 = 0x09U,
    /** Transmit CP2520 pattern 3. */
    DP_PATTERN_CP2520_3 = 0x0AU,
    /** Stop transmitting test pattern. */
    DP_PATTERN_DISABLE = 0x0BU
} DP_TestPattern;

/** DP events. Each value represents one bit and may be set or cleared. */
typedef enum
{
    /** HPD state has changed. */
    DP_TX_HPD_EVENT = 0x01U,
    /** HDCP TX status has changed. */
    DP_HDCP_TX_STATUS_EVENT = 0x10U,
    /**
     * Host needs to check, if pairing data (including km) associated with
     * Receiver ID is currently stored.
     */
    DP_HDCP2_TX_IS_KM_STORED_EVENT = 0x20U,
    /** HDCP FW has pairing data to store. */
    DP_HDCP2_TX_STORE_KM_EVENT = 0x40U,
    /** Host needs to check, if Receiver IDs are valid (not on revocation list). */
    DP_HDCP_TX_IS_RECV_ID_VALID_EVENT = 0x80U
} DP_Events;

/** HPD - related events. Each value represents one bit and may be set or cleared. */
typedef enum
{
    /** HPD line change to high. */
    DP_HPD_TO_HIGH = 0x01U,
    /** HPD line change to low. */
    DP_HPD_TO_LOW = 0x02U,
    /** HPD line pulse */
    DP_HPD_PULSE = 0x04U,
    /** HPD line current state (high - bit set, low - bit cleared) */
    DP_HPD_STATE = 0x08U
} DP_HpdEvents;

/**
 * Status of last AUX transaction. For meaning of values 0-2, please refer to
 * DisplayPort specification, "Reply Command Definition".
 */
typedef enum
{
    DP_AUX_ACK = 0U,
    DP_AUX_NACK = 1U,
    DP_AUX_DEFER = 2U,
    /**
     * Sink error -  Maximum number of bytes (16) was received by AUX, but
     * there was no STOP condition.
     */
    DP_AUX_SINK_ERROR = 3U,
    /** Firmware returned incorrect value */
    DP_AUX_UNKNOWN_ERROR = 5U
} DP_AuxStatus;

/**
 * Status of last I2C-over-AUX transaction. For meaning of values  0-2,
 * please refer to DisplayPort specification, "Reply Command Definition".
 * This value is only relevant, if AUX status of that transaction was
 * DP_AUX_ACK.
 */
typedef enum
{
    DP_I2C_ACK = 0U,
    DP_I2C_NACK = 1U,
    DP_I2C_DEFER = 2U,
    /** Firmware returned incorrect value */
    DP_I2C_UNKNOWN_ERROR = 5U
} DP_I2cStatus;

typedef enum
{
    DP_VIC_MODE_1_59_94HZ = 0U,
    DP_VIC_MODE_1_60HZ = 1U,
    DP_VIC_MODE_2_59_94HZ = 2U,
    DP_VIC_MODE_2_60HZ = 3U,
    DP_VIC_MODE_3_59_94HZ = 4U,
    DP_VIC_MODE_3_60HZ = 5U,
    DP_VIC_MODE_4_59_94HZ = 6U,
    DP_VIC_MODE_4_60HZ = 7U,
    DP_VIC_MODE_5_59_94HZ = 8U,
    DP_VIC_MODE_5_60HZ = 9U,
    DP_VIC_MODE_6_59_94HZ = 10U,
    DP_VIC_MODE_6_60HZ = 11U,
    DP_VIC_MODE_7_59_94HZ = 12U,
    DP_VIC_MODE_7_60HZ = 13U,
    DP_VIC_MODE_8_MODE1_59_94HZ = 14U,
    DP_VIC_MODE_8_MODE1_60HZ = 15U,
    DP_VIC_MODE_8_MODE2_59_94HZ = 16U,
    DP_VIC_MODE_8_MODE2_60HZ = 17U,
    DP_VIC_MODE_9_MODE1_59_94HZ = 18U,
    DP_VIC_MODE_9_MODE1_60HZ = 19U,
    DP_VIC_MODE_9_MODE2_59_94HZ = 20U,
    DP_VIC_MODE_9_MODE2_60HZ = 21U,
    DP_VIC_MODE_10_59_94HZ = 22U,
    DP_VIC_MODE_10_60HZ = 23U,
    DP_VIC_MODE_11_59_94HZ = 24U,
    DP_VIC_MODE_11_60HZ = 25U,
    DP_VIC_MODE_12_MODE_1_59_94HZ = 26U,
    DP_VIC_MODE_12_MODE_1_60HZ = 27U,
    DP_VIC_MODE_12_MODE_2_59_94HZ = 28U,
    DP_VIC_MODE_12_MODE_2_60HZ = 29U,
    DP_VIC_MODE_13_MODE_1_59_94HZ = 30U,
    DP_VIC_MODE_13_MODE_1_60HZ = 31U,
    DP_VIC_MODE_13_MODE_2_59_94HZ = 32U,
    DP_VIC_MODE_13_MODE_2_60HZ = 33U,
    DP_VIC_MODE_14_59_94HZ = 34U,
    DP_VIC_MODE_14_60HZ = 35U,
    DP_VIC_MODE_15_59_94HZ = 36U,
    DP_VIC_MODE_15_60HZ = 37U,
    DP_VIC_MODE_16_59_94HZ = 38U,
    DP_VIC_MODE_16_60HZ = 39U,
    DP_VIC_MODE_17_50HZ = 40U,
    DP_VIC_MODE_18_50HZ = 41U,
    DP_VIC_MODE_19_50HZ = 42U,
    DP_VIC_MODE_20_50HZ = 43U,
    DP_VIC_MODE_21_50HZ = 44U,
    DP_VIC_MODE_22_50HZ = 45U,
    DP_VIC_MODE_23_MODE_1_50HZ = 46U,
    DP_VIC_MODE_23_MODE_2_50HZ = 47U,
    DP_VIC_MODE_23_MODE_3_50HZ = 48U,
    DP_VIC_MODE_24_MODE_1_50HZ = 49U,
    DP_VIC_MODE_24_MODE_2_50HZ = 50U,
    DP_VIC_MODE_24_MODE_3_50HZ = 51U,
    DP_VIC_MODE_25_50HZ = 52U,
    DP_VIC_MODE_26_50HZ = 53U,
    DP_VIC_MODE_27_MODE_1_50HZ = 54U,
    DP_VIC_MODE_27_MODE_2_50HZ = 55U,
    DP_VIC_MODE_27_MODE_3_50HZ = 56U,
    DP_VIC_MODE_28_MODE_1_50HZ = 57U,
    DP_VIC_MODE_28_MODE_2_50HZ = 58U,
    DP_VIC_MODE_28_MODE_3_50HZ = 59U,
    DP_VIC_MODE_29_50HZ = 60U,
    DP_VIC_MODE_30_50HZ = 61U,
    DP_VIC_MODE_31_50HZ = 62U,
    DP_VIC_MODE_32_23_97HZ = 63U,
    DP_VIC_MODE_32_24HZ = 64U,
    DP_VIC_MODE_33_25HZ = 65U,
    DP_VIC_MODE_34_29_97HZ = 66U,
    DP_VIC_MODE_34_30HZ = 67U,
    DP_VIC_MODE_35_59_94HZ = 68U,
    DP_VIC_MODE_35_60HZ = 69U,
    DP_VIC_MODE_36_59_94HZ = 70U,
    DP_VIC_MODE_36_60HZ = 71U,
    DP_VIC_MODE_37_50HZ = 72U,
    DP_VIC_MODE_38_50HZ = 73U,
    DP_VIC_MODE_39_50HZ = 74U,
    DP_VIC_MODE_40_100HZ = 75U,
    DP_VIC_MODE_41_100HZ = 76U,
    DP_VIC_MODE_42_100HZ = 77U,
    DP_VIC_MODE_43_100HZ = 78U,
    DP_VIC_MODE_44_100HZ = 79U,
    DP_VIC_MODE_45_100HZ = 80U,
    DP_VIC_MODE_46_119_88HZ = 81U,
    DP_VIC_MODE_46_120HZ = 82U,
    DP_VIC_MODE_47_119_88HZ = 83U,
    DP_VIC_MODE_47_120HZ = 84U,
    DP_VIC_MODE_48_119_88HZ = 85U,
    DP_VIC_MODE_48_120HZ = 86U,
    DP_VIC_MODE_49_119_88HZ = 87U,
    DP_VIC_MODE_49_120HZ = 88U,
    DP_VIC_MODE_50_119_88HZ = 89U,
    DP_VIC_MODE_50_120HZ = 90U,
    DP_VIC_MODE_51_119_88HZ = 91U,
    DP_VIC_MODE_51_120HZ = 92U,
    DP_VIC_MODE_52_200HZ = 93U,
    DP_VIC_MODE_53_200HZ = 94U,
    DP_VIC_MODE_54_200HZ = 95U,
    DP_VIC_MODE_55_200HZ = 96U,
    DP_VIC_MODE_56_239HZ = 97U,
    DP_VIC_MODE_56_240HZ = 98U,
    DP_VIC_MODE_57_239HZ = 99U,
    DP_VIC_MODE_57_240HZ = 100U,
    DP_VIC_MODE_58_239HZ = 101U,
    DP_VIC_MODE_58_240HZ = 102U,
    DP_VIC_MODE_59_239HZ = 103U,
    DP_VIC_MODE_59_240HZ = 104U,
    DP_VIC_MODE_60_23_97HZ = 105U,
    DP_VIC_MODE_60_24HZ = 106U,
    DP_VIC_MODE_61_25HZ = 107U,
    DP_VIC_MODE_62_29_97HZ = 108U,
    DP_VIC_MODE_62_30HZ = 109U,
    DP_VIC_MODE_63_119_88HZ = 110U,
    DP_VIC_MODE_63_120HZ = 111U,
    DP_VIC_MODE_64_100HZ = 112U,
    DP_VIC_MODE_97_60HZ = 113U,
    DP_VIC_MODE_VESA_800X600P_60HZ = 114U,
    DP_VIC_MODE_VESA_720X400P_70HZ = 115U,
    DP_VIC_MODE_VESA_720X400P_88HZ = 116U,
    DP_VIC_MODE_VESA_640X480P_60HZ = 117U,
    DP_VIC_MODE_VESA_640X480P_67HZ = 118U,
    DP_VIC_MODE_VESA_640X480P_72HZ = 119U,
    DP_VIC_MODE_VESA_640X480P_75HZ = 120U,
    DP_VIC_MODE_VESA_800X600P_56HZ = 121U,
    DP_VIC_MODE_VESA_800X600P_72HZ = 122U,
    DP_VIC_MODE_VESA_832X624P_75HZ = 123U,
    DP_VIC_MODE_VESA_1024X768I_87HZ = 124U,
    DP_VIC_MODE_VESA_1024X768P_60HZ = 125U,
    DP_VIC_MODE_VESA_1024X768P_72HZ = 126U,
    DP_VIC_MODE_VESA_1024X768P_75HZ = 127U,
    DP_VIC_MODE_VESA_1280X1024P_75HZ = 128U,
    DP_VIC_MODE_VESA_1152X870P_75HZ = 129U,
    DP_VIC_MODE_126_60HZ = 130U,
    DP_VIC_MODE_DUMMY_1_59_94HZ = 131U,
    DP_VIC_MODE_DUMMY_2_59_94HZ = 132U,
    DP_VIC_MODE_DUMMY_4_59_94HZ = 133U,
    DP_VIC_MODE_DUMMY_14_59_94HZ = 134U,
    DP_VIC_MODE_DUMMY_16_59_94HZ = 135U,
    DP_VIC_MODE_DUMMY_17_50HZ = 136U,
    DP_VIC_MODE_DUMMY_25_50HZ = 137U,
    DP_VIC_MODE_DUMMY_28_MODE_1_50HZ = 138U,
    DP_VIC_MODE_DUMMY_31_50HZ = 139U,
    DP_VIC_MODE_DUMMY_32_23_97HZ = 140U,
    DP_VIC_MODE_DUMMY_35_59_94HZ = 141U,
    DP_VIC_MODE_DUMMY_47_119_88HZ = 142U,
    DP_VIC_MODE_DUMMY_49_119_88HZ = 143U,
    DP_VIC_MODE_DUMMY_52_200HZ = 144U,
    DP_VIC_MODE_DUMMY_57_239HZ = 145U,
    DP_VIC_MODE_DUMMY_58_239HZ = 146U,
    DP_VIC_MODE_DUMMY_61_25HZ = 147U,
    DP_VIC_MODE_DUMMY_63_119_88HZ = 148U,
    DP_VIC_MODE_DUMMY_126_60HZ = 149U,
    DP_VIC_MODE_COUNT = 150U
} DP_VicModes;

/** Selecting progressive or interlaced scan mode. */
typedef enum
{
    DP_SM_PROGRESSIVE = 0U,
    DP_SM_INTERLACED = 1U
} DP_ScanMode;

/** Selecting synchronization signal polarity */
typedef enum
{
    DP_SP_ACTIVE_LOW = 0U,
    DP_SP_ACTIVE_HIGH = 1U
} DP_SyncPolarity;

typedef enum
{
    DP_PXENC_PXL_RGB = 0x01U,
    DP_PXENC_YCBCR_4_4_4 = 0x02U,
    DP_PXENC_YCBCR_4_2_2 = 0x04U,
    DP_PXENC_YCBCR_4_2_0 = 0x08U,
    DP_PXENC_Y_ONLY = 0x10U
} DP_PixelEncodingFormat;

typedef enum
{
    DP_STEREO_VIDEO_LEFT = 0x00U,
    DP_STEREO_VIDEO_RIGHT = 0x01U
} DP_StereoVideoAttr;

/**
 * Only applicable to YCbCr 4:4:4 and 4:2:2 modes. Value to be placed in
 * MISC0, according to DisplayPort standard. Indicates adherence to
 * particular colorimetry format.
 */
typedef enum
{
    /** Video stream uses ITU-R BT.601 colorimetry specification. */
    DP_BT_601 = 0x00U,
    /** Video stream uses ITU-R BT.709 colorimetry specification. */
    DP_BT_709 = 0x01U
} DP_BtType;

/** Alignment of the input pixel data at the pixel interface. */
typedef enum
{
    /** Pixel data is aligned to most significant bit. (default) */
    DP_ALIGN_MSB = 0U,
    /** Pixel data is aligned to least significant bit. */
    DP_ALIGN_LSB = 1U
} DP_PxlAlignment;

/** Selecting SDP Active Idle mode. */
typedef enum
{
    /** SDP will be sent during the period when the video is disabled. */
    DP_SDP_ACTIVE_NO_VIDEO = 0x00U,
    /**
     * SDP will be sent during the vertical blank periods in the active video
     * mode.
     */
    DP_SDP_ACTIVE_VIDEO = 0x01U
} DP_SdpActiveIdleMode;

/** Selecting HDCP version support. */
typedef enum
{
    /** Support only HDCP 2.2 */
    DP_HDCP_2_SUPPORT = 0x00U,
    /** Support only HDCP 1.4 */
    DP_HDCP_1_SUPPORT = 0x01U,
    /** Support both HDCP versions, trying using HDCP 2.2 first. */
    DP_HDCP_BOTH_SUPPORT = 0x02U
} DP_HdcpVerSupport;

/** Selecting Content Stream type, when receiver is a repeater. */
typedef enum
{
    /** Stream may be transmitted to all HDCP devices. */
    DP_TYPE_0_CONTENT_STREAM = 0x00U,
    /**
     * Stream must not be transmitted to HDCP 1.x-compliant devices and HDCP
     * 2.0-compliant repeaters. This option is incompatible with MST operation.
     */
    DP_TYPE_1_CONTENT_STREAM = 0x01U
} DP_ContentStreamType;

/** HDCP Error codes. */
typedef enum
{
    /** No error occurred. */
    DP_HDCP_ERR_NO_ERROR = 0x00U,
    /** HPD is down. */
    DP_HDCP_ERR_HPD_DOWN = 0x01U,
    /** SRM failure */
    DP_HDCP_ERR_SRM_FAIL = 0x02U,
    /** signature verification error */
    DP_HDCP_ERR_SIGN_ERROR = 0x03U,
    /**
     * (for HDCP 2.x) Hash H', computed by receiver, differs from H computed
     * by transmitter.
     */
    DP_HDCP_ERR_H_HASH_MISMATCH = 0x04U,
    /**
     * (for HDCP 1.x) Hash V', computed by receiver, differs from V computed
     * by transmitter.
     */
    DP_HDCP_ERR_V_HASH_MISMATCH = 0x05U,
    /** (for HDCP 2.x) Locality check failed and could not be retried anymore. */
    DP_HDCP_ERR_LOCALITY_CHECK_FAIL = 0x06U,
    /** DDC (AUX channel) error. */
    DP_HDCP_ERR_DDC_ERROR = 0x07U,
    /** Re-authentication is required */
    DP_HDCP_ERR_REAUTH_REQ = 0x08U,
    /**
     * Topology error. Exceeded max number of devices (MAX_DEVS_EXCEEDED) or
     * max repeater cascade depth (MAX_CASCADE_EXCEEDED), or seq_num_V is invalid.
     */
    DP_HDCP_ERR_TOPOLOGY_ERROR = 0x09U,
    /**
     * Not all reserved (Rsvd) bytes in HDCP port were read as zero, or receiver
     * is not HDCP-capable.
     */
    DP_HDCP_ERR_RSVD_NOT_ZERO = 0x0BU,
    /** Link synchronization verification values RI differ. */
    DP_HDCP_ERR_RI_MISMATCH = 0x0DU,
    /**
     * Repeater's "KSV List ready" status bit was not set before 5-second
     * watchdog timer expired.
     */
    DP_HDCP_ERR_WATCHDOG_EXPIRED = 0x0EU
} DP_HdcpErrCode;

/** Audio stream sampling frequency */
typedef enum
{
    /** 32 kHz */
    DP_AUDIO_FREQ_32 = 0x00U,
    /** 48 kHz */
    DP_AUDIO_FREQ_48 = 0x01U,
    /** 96 kHz */
    DP_AUDIO_FREQ_96 = 0x02U,
    /** 192 kHz */
    DP_AUDIO_FREQ_192 = 0x03U,
    /** 44.1 kHz */
    DP_AUDIO_FREQ_44_1 = 0x04U,
    /** 88.2 kHz */
    DP_AUDIO_FREQ_88_2 = 0x05U,
    /** 176.4 kHz */
    DP_AUDIO_FREQ_176_4 = 0x06U
} DP_AudioFreq;

/** Audio stream sample size (width) */
typedef enum
{
    /** 16 bits */
    DP_AUDIO_WIDTH_16 = 0x00U,
    /** 24 bits */
    DP_AUDIO_WIDTH_24 = 0x01U,
    /** 32 bits */
    DP_AUDIO_WIDTH_32 = 0x02U
} DP_AudioWidth;

/** Audio enable/disable mode */
typedef enum
{
    /** Disable audio */
    DP_AUDIO_MODE_OFF = 0x00U,
    /** Enable audio */
    DP_AUDIO_MODE_ON = 0x01U
} DP_AudioMode;

/** Audio mute mode */
typedef enum
{
    /** Mute audio */
    DP_AUDIO_MUTE = 0x00U,
    /** Unmute audio */
    DP_AUDIO_UNMUTE = 0x01U
} DP_AudioMuteMode;

typedef enum
{
    /** 8 bits/component */
    DP_BITS_PER_COMPONENT_8 = 0U,
    /** 10 bits/component */
    DP_BITS_PER_COMPONENT_10 = 1U
} DP_BitsPerComponent;

/** Status codes used for Link Training process. */
typedef enum
{
    /** Link Training succeeded. */
    DP_LT_OK = 0U,
    /** Link Training finished prematurely, due to unexpected or invalid situation. */
    DP_LT_UNFINISHED = 1U,
    /** Link Training failed during Clock Recovery phase. */
    DP_LT_CR_FAIL = 2U,
    /** Link Training failed during Channel Equalization phase. */
    DP_LT_EQ_FAIL = 3U
} DP_TrainingStatus;

typedef enum
{
    /** Regular APB bus */
    DP_BUS_TYPE_APB = 0U,
    /** Secure APB bus */
    DP_BUS_TYPE_SAPB = 1U
} DP_BusType;

typedef enum
{
    /** Inject error to IRAM */
    DP_ECC_ERROR_MEM_TYPE_IRAM = 1U,
    /** Inject error to DRAM */
    DP_ECC_ERROR_MEM_TYPE_DRAM = 2U
} DP_EccErrorMemType;

typedef enum
{
    /** Inject error to data bits */
    DP_ECC_ERROR_TYPE_DATA = 1U,
    /** Inject error to check bits */
    DP_ECC_ERROR_TYPE_CHECK = 0U
} DP_EccErrorType;

/**********************************************************************
* Callbacks
**********************************************************************/
/**
 * Called on hardware interrupt.
 * Should call getEvent to check, which event-related bits have been set.
 */
typedef void (*DP_CbEvent)(DP_PrivateData* pD);

/**
 *  @}
 */

/** @defgroup DriverFunctionAPI Driver Function API
 *  Prototypes for the driver API functions. The user application can link statically to the
 *  necessary API functions and call them directly.
 *  @{
 */

/**********************************************************************
* API methods
**********************************************************************/

/**
 * Get the driver's memory requirements.
 * @param[in] config Proposed driver/hardware configuration.
 * @param[out] memReq Size of memory, that needs to be allocated (in bytes).
 */
uint32_t DP_Probe(const DP_Config* config, uint32_t* memReq);

/**
 * brief set up API, must be called before any other API call
 * @param[in,out] pD Driver state info specific to this instance.
 * @param[in] config Specifies driver/hardware configuration.
 * @param[in] callbacks Client-supplied callback functions.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD, config or callbacks is NULL.
 * @return CDN_ENOTSUP If HW is not supported by this driver.
 */
uint32_t DP_Init(DP_PrivateData* pD, const DP_Config* config, const DP_Callbacks* callbacks);

/**
 * Driver ISR. Platform-specific code is responsible for ensuring this
 * gets called when the corresponding hardware's interrupt is
 * asserted. Registering the ISR should be done after calling init,
 * and before calling start. The driver's ISR will not attempt to lock
 * any locks, but will perform client callbacks. If the client wishes
 * to defer processing to non-interrupt time, it is responsible for
 * doing so. This function must not be called after calling destroy
 * and releasing private data memory.
 * @param[in] pD Driver instance data.
 */
void DP_Isr(DP_PrivateData* pD);

/**
 * Start the Display Port Core Driver by enabling interrupts. This is
 * called after the client has successfully initialized the driver and
 * hooked the driver's ISR (the isr member of this struct) to the IRQ.
 * @param[in] pD Driver instance data.
 * @return CDN_EOK success
 * @return CDN_EINVAL pD is NULL
 */
uint32_t DP_Start(DP_PrivateData* pD);

/**
 * Stops the Display Port Core Driver by disabling interrupts.
 * @param[in] pD Driver instance data.
 * @return CDN_EOK success
 * @return CDN_EINVAL pD is NULL
 */
uint32_t DP_Stop(DP_PrivateData* pD);

/**
 * Destroy the driver.
 * @param[in] pD Driver instance data.
 * @return CDN_EOK success
 * @return CDN_EINVAL pD is NULL
 */
uint32_t DP_Destroy(const DP_PrivateData* pD);

/**
 * Set address of PHY driver's private data structure.
 * @param[in] pD Driver instance data.
 * @param[out] phyPd Pointer to private data of PHY driver's instance to use.
 * @return CDN_EOK success
 * @return CDN_EINVAL pD or phyPd is NULL
 */
uint32_t DP_SetPhyPd(DP_PrivateData* pD, DP_SD0801_PrivateData* phyPd);

/**
 * Releases uCPU reset and stalls it, then loads firmware from
 * provided image.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] image Pointer to structure with FW image information.
 * @return CDN_EOK success
 * @return CDN_EINVAL If IMEM size or DMEM size is not divisible by 4.
 */
uint32_t DP_LoadFirmware(const DP_PrivateData* pD, const DP_FirmwareImage* image);

/**
 * Starts uCPU. Should be called after loading firmware.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_StartUcpu(const DP_PrivateData* pD);

/**
 * Check, if there is a response awaiting in mailbox.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] awaits Pointer to store flag indicating presence of response.
 * @param[in] busType
 * @return CDN_EOK success
 * @return CDN_EINVAL If busType is outside enum range.
 */
uint32_t DP_CheckResponse(const DP_PrivateData* pD, bool* awaits, DP_BusType busType);

/**
 * Debug echo command for APB.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] message Value to echo
 * @param[in] busType
 * @return CDN_EOK success
 * @return CDN_EIO reply message doesn't match request
 * @return CDN_EINVAL If busType is outside enum range.
 */
uint32_t DP_TestEcho(DP_PrivateData* pD, uint32_t message, DP_BusType busType);

/**
 * Extended Echo test for mailbox. This test will send msg buffer to
 * firmware's mailbox and receive it back to the resp buffer. Received
 * data will be check against data sent and status will be returned as
 * well as received data.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] message Pointer to a buffer to send.
 * @param[out] response Pointer to buffer for receiving msg payload back.
 * @param[in] messageSize Number of bytes to send and receive.
 * @param[in] busType
 * @return CDN_EOK success
 * @return CDN_EIO reply message's size or content doesn't match request's.
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If number of bytes to use is invalid.
 */
uint32_t DP_TestEchoExt(DP_PrivateData* pD, const uint8_t* message, uint8_t* response, uint16_t messageSize, DP_BusType busType);

/**
 * Get current FW version.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] ver fw version
 * @param[out] verlib lib version
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or fw/lib version pointer is NULL.
 */
uint32_t DP_GetCurVersion(const DP_PrivateData* pD, uint16_t* ver, uint16_t* verlib);

/**
 * Read event registers (SW_EVENTS) value.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] events pointer to store 32-bit events value
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or events pointer is NULL.
 */
uint32_t DP_GetEvent(const DP_PrivateData* pD, uint32_t* events);

/**
 * Read debug register value.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] debug pointer to store 16-bit debug reg value
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or debug pointer is NULL.
 */
uint32_t DP_GetDebugRegVal(const DP_PrivateData* pD, uint16_t* debug);

/**
 * Check, if KEEP_ALIVE register has changed. To check if FW on uCPU
 * is still running, it is necessary to keep calling this function,
 * until 'updated' parameter is set to 'true' twice.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] updated Pointer to store flag, whether (true) or not (false) KEEP_ALIVE register
 *    changed since initialization or last call of this function, whichever
 *    happened later.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or updated pointer is NULL.
 */
uint32_t DP_CheckAlive(DP_PrivateData* pD, bool* updated);

/**
 * Wait, until KEEP_ALIVE register changes. WARNING: If Firmware on
 * uCPU is not running, this function will block indefinitely.
 * DP_CheckAlive is a non-blocking equivalent.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_WaitAlive(DP_PrivateData* pD);

/**
 * Sends request for setting uCPU/FW mode. DP_CheckResponse for
 * (regular) APB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode 1 for active, 0 for standby
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendMainControlRequest(DP_PrivateData* pD, uint8_t mode);

/**
 * Gets response for setting uCPU/FW mode.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] resp pointer to store response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or resp pointer is NULL.
 */
uint32_t DP_GetMainControlResponse(DP_PrivateData* pD, uint8_t* resp);

/**
 * Set uCPU/FW to standby or active. Await for response.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode 1 for active, 0 for standby
 * @param[out] resp Response: 1 for active, 0 for standby
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or resp pointer is NULL.
 */
uint32_t DP_MainControl(DP_PrivateData* pD, uint8_t mode, uint8_t* resp);

/**
 * Specify the Xtensa clock. This function shall be called before
 * turning on the CPU.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] ucpuClock Clock, that Xtensa uCPU is running at.
 * @return CDN_EOK success
 * @return CDN_EINVAL If clock is outside range (1.0 - 255.99 MHz).
 */
uint32_t DP_SetClock(DP_PrivateData* pD, const DP_UcpuClock* ucpuClock);

/**
 * set maximum and minimum watchdog counters
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] watchdogMin Minimum value of the watchdog counter.
 * @param[in] watchdogMax Maximum value of the watchdog counter.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetWatchdogConfig(DP_PrivateData* pD, uint32_t watchdogMin, uint32_t watchdogMax);

/**
 * Inject memory error. Function is used for testing ECC mechanism.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] errorMask It is the bit flip mask for check bits and databits for both IRAM and DRAM.
 * @param[in] memType Select memory where shall be injected IRAM or DRAM.
 * @param[in] errorType Select whether error shall be injected either in data bits or in check bits.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_InjectEccError(DP_PrivateData* pD, uint32_t errorMask, DP_EccErrorMemType memType, DP_EccErrorType errorType);

/**
 * Force fatal error. Fuction is used to test PFatalError output.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_ForceFatalError(DP_PrivateData* pD);

/**
 * set audio video clock configuration
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId stream number which should be configured
 *    for SST it should be always 0
 *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] audioVideoClkCfg audio video clock configuration to be set
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is has wrong value
 */
uint32_t DP_SetAudioVideoClkCfg(DP_PrivateData* pD, uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg);

/**
 * set audio video clock configuration
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId stream number which should be configured
 *    for SST it should be always 0
 *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[out] audioVideoClkCfg audio video clock configuration to be set
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is has wrong value
 */
uint32_t DP_GetAudioVideoClkCfg(DP_PrivateData* pD, uint8_t streamId, DP_AudioVideoClkCfg* audioVideoClkCfg);

/**
 * set HDCP clock configuration
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] hdcpClockEnable HDCP clock config
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetHdcpClockConfig(DP_PrivateData* pD, bool hdcpClockEnable);

/**
 * get HDCP clock configuration
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] hdcpClockEnable HDCP clock config
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_GetHdcpClockConfig(DP_PrivateData* pD, bool* hdcpClockEnable);

/**
 * Initialize part of PHY responsible for AUX channel. Has to be
 * called before performing any DPCD or EDID operations.
 * Alternatively, respective PHY driver's function may be called
 * instead.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_ConfigurePhyAuxCtrl(const DP_PrivateData* pD);

/**
 * Automatically initialize and configure DP Main Link on PHY. Has to
 * be called before performing Link Training. This is a recommended
 * way to bring up PHY, instead of manual initialization. AUX channel
 * still has to be initialized separately. alternatively, respective
 * PHY driver's function may be called instead.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] laneCount Number of lanes to initialize PHY with.
 * @param[in] linkRate Link rate to initialize PHY with.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL or parameters are invalid.
 */
uint32_t DP_ConfigurePhyStartUp(DP_PrivateData* pD, uint8_t laneCount, DP_LinkRate linkRate);

/**
 * Sends request for reading EDID from sink device. DP_checkResponse
 * for (regular) APB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] segment EDID segment to read.
 * @param[in] extension EDID extension to read
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendEdidReadRequest(DP_PrivateData* pD, uint8_t segment, uint8_t extension);

/**
 * Gets response with EDID read from sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] resp Pointer structure to be filled with response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Size of EDID read was not 128 bytes.
 * @return CDN_EINVAL If pD or resp pointer is NULL.
 */
uint32_t DP_GetEdidReadResponse(DP_PrivateData* pD, DP_ReadEdidResponse* resp);

/**
 * Reads EDID from sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] segment EDID segment to read.
 * @param[in] extension EDID extension to read
 * @param[in] resp Pointer structure to be filled with response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Size of EDID read was not 128 bytes.
 * @return CDN_EINVAL If pD or resp pointer is NULL.
 */
uint32_t DP_ReadEdid(DP_PrivateData* pD, uint8_t segment, uint8_t extension, DP_ReadEdidResponse* resp);

/**
 * Set power mode of sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Power mode
 * @return CDN_EOK success
 * @return CDN_EINVAL If PwrMode value is outside respective enum.
 */
uint32_t DP_SetPowerMode(DP_PrivateData* pD, DP_PwrMode mode);

/**
 * Sets source capabilities, according to filled
 * DP_SourceDeviceCapabilities structure, provided by user.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] caps Pointer to structure with source capabilities.
 * @return CDN_EOK success
 * @return CDN_EINVAL If there are invalid values in capabilities (outside range or enum).
 */
uint32_t DP_SetSourceCapabilities(DP_PrivateData* pD, const DP_SourceDeviceCapabilities* caps);

/**
 * Perform a series of DPCD register reads, used to determine
 * capabilities of sink device, and get structure with them.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] caps Pointer to structure to be filled with sink capabilities.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or caps is NULL.
 */
uint32_t DP_GetSinkCapabilities(DP_PrivateData* pD, DP_SinkDeviceCapabilities* caps);

/**
 * Set custom 80-bit pattern to be transmitted for testing. To
 * transmit that pattern, DP_setTestPattern needs to be called, with
 * PATTERN_80_BIT as "pattern" argument.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] customPattern Array of 10 bytes (80 bits) to be transmitted as the pattern. Bytes are
 *    to be transmitted in order they're placed in the table ([0] -> [9]).
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or customPattern is NULL.
 */
uint32_t DP_SetCustomPattern(DP_PrivateData * pD, uint8_t customPattern[10]);

/**
 * Transmit (or stop transmitting) selected test pattern at requested
 * main link parameters (link rate, lane count, voltage swing, pre-
 * emphasis). Does not inform sink about transmitted pattern (DPCD
 * registers 10Bh - 10Eh).
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] pattern Test pattern to be transmitted. Selecting PATTERN_DISABLE will stop
 *    transmitting test pattern.
 * @param[in] linkParams Pointer to structure containing main link parameters to be used. If
 *    pattern is PATTERN_DISABLE, main link parameters will not be changed.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pattern or any member of linkParams is out of range.
 */
uint32_t DP_SetTestPattern(DP_PrivateData* pD, DP_TestPattern pattern, DP_LinkState* linkParams);

/**
 * Sends request for reading DPCD register(s) from sink device.
 * DP_checkResponse for (regular) APB may be used to check, if reply
 * is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to DPCD read operation.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or request pointer is NULL.
 */
uint32_t DP_SendDpcdReadRequest(DP_PrivateData* pD, const DP_DpcdTransfer* request);

/**
 * Gets response with DPCD register(s) read from sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or count of DPCD registers, that were read, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_GetDpcdReadResponse(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

/**
 * Reads DPCD register(s) from sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or count of DPCD registers, that were read, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_ReadDpcd(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

/**
 * Sends request for writing DPCD register(s) to sink device.
 * DP_checkResponse for (regular) APB may be used to check, if reply
 * is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to DPCD write operation.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or request pointer is NULL.
 */
uint32_t DP_SendDpcdWriteRequest(DP_PrivateData* pD, const DP_DpcdTransfer* request);

/**
 * Gets response with DPCD register(s) write operation status.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or count of DPCD registers, that were written, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_GetDpcdWriteResponse(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

/**
 * Writes DPCD register(s) to sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from
 *    response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or count of DPCD registers, that were written, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_WriteDpcd(DP_PrivateData* pD, DP_DpcdTransfer* transfer);

/**
 * Sends request for I2C-over-AUX read from given I2C address, using
 * sink device. DP_checkResponse for (regular) APB may be used to
 * check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to I2C-over-AUX read operation.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or request pointer is NULL.
 */
uint32_t DP_SendI2cReadRequest(DP_PrivateData* pD, const DP_I2cTransfer* request);

/**
 * Gets response with bytes read over I2C-over-AUX, using sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or I2C slave or number of bytes, that were read, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_GetI2cReadResponse(DP_PrivateData* pD, DP_I2cTransfer* transfer);

/**
 * Reads bytes over I2C-over-AUX, using sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or I2C slave or number of bytes, that were read, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_I2cRead(DP_PrivateData* pD, DP_I2cTransfer* transfer);

/**
 * Sends request for I2C-over-AUX write to given I2C address, using
 * sink device. DP_checkResponse for (regular) APB may be used to
 * check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to I2C-over-AUX write operation.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or request pointer is NULL.
 */
uint32_t DP_SendI2cWriteRequest(DP_PrivateData* pD, const DP_I2cTransfer* request);

/**
 * Gets response with I2C-over-AUX write operation status.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or I2C slave or number of bytes, that were written, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_GetI2cWriteResponse(DP_PrivateData* pD, DP_I2cTransfer* transfer);

/**
 * Writes bytes over I2C-over-AUX, using sink device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from
 *    response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address or I2C slave or number of bytes, that were written, does not match request
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_I2cWrite(DP_PrivateData* pD, DP_I2cTransfer* transfer);

/**
 * Enables or disables ASSR (Alternate Scrambler Seed Reset) in sink
 * and source devices, if it's supported by sink.
 * DP_getSinkCapabilities function may also be used to check, if sink
 * supports that feature. Standard value of 0xFFFE is used as an
 * alternate seed.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] enable Whether to enable ('true') or disable ('false') ASSR.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO DPCD transaction or register write was unsuccessful.
 * @return CDN_ENOTSUP Enabling ASSR was requested, but sink device does not support it.
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetAssrEnable(DP_PrivateData* pD, bool enable);

/**
 * Enables or disables shortening of AUX transaction preamble (sync
 * pattern) from 16 to 8 pulses (as required by eDP standard). Default
 * (false) is 16 pulses, as required by regular DisplayPort standard.
 * It is required, by the eDP standard, to enable shortened preamble.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] enable Whether to shorten ('true') or not ('false') the AUX preamble.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetShortenedAuxPreamble(DP_PrivateData* pD, bool enable);

/**
 * Performs DisplayPort Link Training, according to capabilities set
 * via setHostCapabilities and sink capabilities read from DPCD.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] resultLt Result of Link Training process, according to TrainingStatus enum.
 * @return CDN_EOK success
 * @return CDN_ENOENT Source device capabilities were not set.
 * @return CDN_ECANCELED Sink device capabilities are not correct.
 * @return CDN_EINVAL If pD or result pointer is NULL.
 */
uint32_t DP_LinkTraining(DP_PrivateData* pD, DP_TrainingStatus* resultLt);

/**
 * Checks, if link is stable and synchronized, according to sink
 * device.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] resultLs Whether ('true') or not link is stable and synchronized.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or result pointer is NULL.
 */
uint32_t DP_CheckLinkStable(DP_PrivateData* pD, bool* resultLs);

/**
 * Sets mask disabling events.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mask Event mask (bit '1' at given position disables event).
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetEventMask(DP_PrivateData* pD, uint32_t mask);

/**
 * Gets mask disabling events.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mask Event mask (bit '1' at given position disables event).
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or mask pointer is NULL.
 * @return CDN_ENOTSUP Operation currently not implemented
 */
uint32_t DP_GetEventMask(const DP_PrivateData* pD, const uint32_t* mask);

/**
 * Sends request for reading events related to HPD from uCPU.
 * DP_CheckResponse for (regular) APB may be used to check, if reply
 * is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendReadHpdEventRequest(DP_PrivateData* pD);

/**
 * Gets response for reading events related to HPD from uCPU.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] hpdEvents pointer to store response.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or hpdEvents pointer is NULL.
 */
uint32_t DP_GetReadHpdEventResponse(DP_PrivateData* pD, uint8_t* hpdEvents);

/**
 * Reads events.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] hpdEvents Set of HPD-related events, as bits defined in DP_HpdEvents enum.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or hpdEvents pointer is NULL.
 */
uint32_t DP_ReadHpdEvent(DP_PrivateData* pD, uint8_t* hpdEvents);

/**
 * Fills DP_VideoFormatParams structure, based on entry in VIC
 * parameters table, as defined in VicModes enum.
 * @param[out] vicParams Structure with VIC-relared parameters to fill.
 * @param[in] vicMode VIC mode to take from table and fill into structure.
 * @return CDN_EOK success
 * @return CDN_EINVAL If vicMode is outside enum range.
 */
uint32_t DP_FillVideoFormat(DP_VideoFormatParams* vicParams, DP_VicModes vicMode);

/**
 * Set vic mode according to vic table, or requested video parameters.
 * May be called after successful Link Training. If different
 * synchronization pulse is desired for MSA data, than one present on
 * the Video Interface (VIF), DP_SetMsaSyncPolarity may be called
 * after this function.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1 DSC can be enabled only for stream 0 or 1.
 * @param[in] parameters Structure with video parameters to set.
 * @return CDN_EOK success
 * @return CDN_ENOENT PHY was not initialized and Link Training was not done yet.
 * @return CDN_EINVAL If incorrect video parameters were detected.
 */
uint32_t DP_SetVic(DP_PrivateData* pD, uint8_t streamId, const DP_VideoParameters* parameters);

/**
 * DP_SetVic function sets the same synchronization pulse polarity for
 * VIF (Video Interface) input and MSA (Main Steam Attribute). If it
 * is desired to use different polarity in MSA data, this function may
 * be called after DP_SetVic to override sync polarity for MSA
 * (leaving polarity for VIF as DP_SetVic had set it).
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] hSyncPolarity Horizontal sync pulse polarity to set in MSA.
 * @param[in] vSyncPolarity Vertical sync pulse polarity to set in MSA.
 * @return CDN_EOK success
 * @return CDN_EINVAL If there are invalid values in parameters (outside of enum).
 */
uint32_t DP_SetMsaSyncPolarity(DP_PrivateData* pD, uint8_t streamId, DP_SyncPolarity hSyncPolarity, DP_SyncPolarity vSyncPolarity);

/**
 * Turn framer on or off. Enabling framer on is required before
 * enabling video. Disabling framer is required before performing Link
 * Training.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] enable Framer enable (true - on, false - off)
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetFramerEnable(DP_PrivateData* pD, bool enable);

/**
 * Turn video on or off. Applicable to SST mode only. Disabling video
 * is required before performing Link Training.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Video Mode (true - on, false - off)
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetVideoSst(DP_PrivateData* pD, bool mode);

/**
 * Get current link status (link rate, lane count, voltage sing level
 * and pre-emphasis level). Values are valid after successful Link
 * Training.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] linkState Link State.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or linkState pointer is NULL.
 */
uint32_t DP_ReadLinkStat(DP_PrivateData* pD, DP_LinkState* linkState);

/**
 * Sends request for reading status of last AUX transaction.
 * DP_CheckResponse for (regular) APB may be used to check, if reply
 * is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendAuxStatusRequest(DP_PrivateData* pD);

/**
 * Gets response with status of last AUX transaction.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest AUX status
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetAuxStatusResponse(DP_PrivateData* pD, DP_AuxStatus* status);

/**
 * Reads and returns status of most recent AUX transaction.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest AUX status
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetAuxStatus(DP_PrivateData* pD, DP_AuxStatus* status);

/**
 * Sends request for reading status of last I2C-over-AUX transaction.
 * DP_CheckResponse for (regular) APB may be used to check, if reply
 * is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendI2cStatusRequest(DP_PrivateData* pD);

/**
 * Gets response with status of last I2C-over-AUX transaction.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest I2C-over-AUX status
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetI2cStatusResponse(DP_PrivateData* pD, DP_I2cStatus* status);

/**
 * Reads and returns status of most recent I2C-over-AUX transaction.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest I2C-over-AUX status
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetI2cStatus(DP_PrivateData* pD, DP_I2cStatus* status);

/**
 * Sends request for reading HPD status. DP_CheckResponse for
 * (regular) APB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD pointer is NULL.
 */
uint32_t DP_SendHpdStatusRequest(DP_PrivateData* pD);

/**
 * Gets response with HPD status.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Whether or not HPD is connected and stable.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetHpdStatusResponse(DP_PrivateData* pD, bool* status);

/**
 * Reads and returns status of HPD.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Whether or not HPD is connected and stable.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetHpdStatus(DP_PrivateData* pD, bool* status);

/**
 * Enables/Disables FEC on DP TX Controller
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] fecEnable informs if FEC should be enabled (true), or disabled (false).
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetFecEnable(DP_PrivateData* pD, bool fecEnable);

/**
 * Sets/clears FEC_READY bit in DPCD on sink device and in source
 * registers.     Function must be called before link training.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] enable informs if FEC_READY should be set (true), or cleared (false).
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetFecReady(DP_PrivateData* pD, bool enable);

/**
 * Sets provided SDP under corresponding entry_id into controller. SDP
 * currently residing under entry_id gets invalidated first.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] entryID 4 Most significant bits of the packet memory address to write.
 * @param[in] packetData Data of SDP entry to write
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_SetSdp(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID, const DP_SdpEntry* packetData);

/**
 * Invalidates SDP under corresponding entry_id in controller.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] entryID 4 Most significant bits of the packet memory containing SDP to remove
 * @return CDN_EOK success
 * @return CDN_EINVAL If entryId is outside range.
 */
uint32_t DP_RemoveSdp(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID);

/**
 * Set configuration of HDCP transmitter
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] config Pointer to structure with configuration for HDCP transmitter.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or config pointer is NULL.
 */
uint32_t DP_ConfigureHdcpTx(DP_PrivateData* pD, const DP_HdcpTxConfiguration* config);

/**
 * Set public key for HDCP 2.x transmitter
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] key Pointer to structure with HDCP 2.x public key.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or key pointer is NULL.
 */
uint32_t DP_SetHdcp2TxPublicKey(DP_PrivateData* pD, const DP_Hdcp2TxPublicKey* key);

/**
 * Set km-key used to decrypt other HDCP keys.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] key Pointer to structure with km-key.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or key pointer is NULL.
 */
uint32_t DP_SetHdcpKmEncCustomKey(DP_PrivateData* pD, const DP_HdcpTxKmEncCustomKey* key);

/**
 * Set HDCP 2.x random numbers for debug purposes.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] numbers Pointer to structure with random numbers.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or numbers pointer is NULL.
 */
uint32_t DP_SetHdcp2DebugRandom(DP_PrivateData* pD, const DP_HdcpDebugRandomNumbers* numbers);

/**
 * Send a response to HDCP 2.x engine, that pairing data (including
 * km) associated with Receiver ID is currently not stored.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_Hdcp2RespondKmNotStored(DP_PrivateData* pD);

/**
 * Send a response to HDCP 2.x engine containing stored pairing data
 * (including km) associated with Receiver ID.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] pairingData Pointer to filled structure containing HDCP 2.x pairing data.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or pairingData pointer is NULL.
 */
uint32_t DP_Hdcp2RespondKmStored(DP_PrivateData* pD, const DP_HdcpPairingData* pairingData);

/**
 * Set private keys for HDCP 1.x transmitter.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] keySet Pointer to filled structure containing HDCP 1.x Device Key Set.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or keySet pointer is NULL.
 */
uint32_t DP_SetHdcp1TxKeys(DP_PrivateData* pD, const DP_Hdcp1Keys* keySet);

/**
 * Set 'An' random number (for debug only), used in HDCP 1.x
 * authentication.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] an 8-byte array containing 'An' number.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or address of 'an' array is NULL.
 */
uint32_t DP_SetHdcp1RandomAn(DP_PrivateData* pD, const uint8_t an[8]);

/**
 * Send request for status of HDCP transmitter. DP_CheckResponse for
 * (secure) SAPB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendHdcpTxStatusRequest(DP_PrivateData* pD);

/**
 * Get response with status of HDCP transmitter.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Pointer to structure with HDCP TX status, to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetHdcpTxStatusResponse(DP_PrivateData* pD, DP_HdcpTxStatus* status);

/**
 * Get status of HDCP transmitter. Should be called in response to
 * HDCP status event.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Pointer to structure with HDCP TX status, to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or status pointer is NULL.
 */
uint32_t DP_GetHdcpTxStatus(DP_PrivateData* pD, DP_HdcpTxStatus* status);

/**
 * Send request for Receiver ID, which should looked up in storage, to
 * get pairing data (if exists in storage). DP_CheckResponse for
 * (secure) SAPB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendHdcp2RecvIdRequest(DP_PrivateData* pD);

/**
 * Get response with Receiver ID, which should looked up in storage.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] id 5-byte array to be filled with Receiver ID.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or address of 'id' array is NULL.
 */
uint32_t DP_GetHdcp2RecvIdResponse(DP_PrivateData * pD, uint8_t id[5]);

/**
 * Get Receiver ID, which should looked up in storage, to get pairing
 * data (if exists in storage).
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] id 5-byte array to be filled with Receiver ID.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or address of 'id' array is NULL.
 */
uint32_t DP_GetHdcp2RecvId(DP_PrivateData * pD, uint8_t id[5]);

/**
 * Send request for pairing data (receiver ID and associated
 * cryptographic keys and values) of receiver being currently
 * authenticated. DP_CheckResponse for (secure) SAPB may be used to
 * check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendHdcp2PairingDataRequest(DP_PrivateData* pD);

/**
 * Get response with pairing data of receiver being currently
 * authenticated.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] pairingData Pointer to structure with HDCP pairing data to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or pairingData pointer is NULL.
 */
uint32_t DP_GetHdcp2PairingDataResponse(DP_PrivateData* pD, DP_HdcpPairingData* pairingData);

/**
 * Get pairing data (receiver ID and associated cryptographic keys and
 * values) of receiver being currently authenticated.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] pairingData Pointer to structure with HDCP pairing data to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or pairingData pointer is NULL.
 */
uint32_t DP_GetHdcp2PairingData(DP_PrivateData* pD, DP_HdcpPairingData* pairingData);

/**
 * Send request for list of all receiver IDs connected directly or via
 * repeaters, for checking them in revocation list. DP_CheckResponse
 * for (secure) SAPB may be used to check, if reply is available.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SendHdcpRecvIdListRequest(DP_PrivateData* pD);

/**
 * Get response with list of all receiver IDs connected directly or
 * via repeaters.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] list Pointer to structure with list of all receiver IDs, and their count,
 *    to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or list pointer is NULL.
 */
uint32_t DP_GetHdcpRecvIdListResponse(DP_PrivateData* pD, DP_HdcpRecvIdList* list);

/**
 * Get list of all receiver IDs connected directly or via repeaters,
 * for checking them in revocation list.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] list Pointer to structure with list of all receiver IDs, and their count,
 *    to be filled.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EINVAL If pD or list pointer is NULL.
 */
uint32_t DP_GetHdcpRecvIdList(DP_PrivateData* pD, DP_HdcpRecvIdList* list);

/**
 * Inform HDCP engine, if all receiver IDs are valid (not present on
 * the revocation list)
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] valid 'true'  - no receiver ID was found on revocation list.
 *    'false' - at least one receiver was found on revocation list.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD is NULL.
 */
uint32_t DP_SetHdcpRecvValid(DP_PrivateData* pD, bool valid);

/**
 * Set 128-bit Global Constant lc.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] lc128 16-byte array with LC128 value to use. If km-key encryption is enabled,
 *    it is expected to be provided in encrypted form.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or address of lc128 array is NULL.
 */
uint32_t DP_SetHdcp2Lc(DP_PrivateData* pD, const uint8_t lc128[16]);

/**
 * Set random seed from external TRNG.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] seed 32-byte array with random seed. Number is expected to come from TRNG
 *    and have sufficiently high entropy level.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or address of seed array is NULL.
 */
uint32_t DP_SetHdcpSeed(DP_PrivateData* pD, const uint8_t seed[32]);

/**
 * Mute or unmute audio.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] muteMode Select mute/unmute.
 * @return CDN_EOK success
 * @return CDN_EINVAL If muteMode is outside of enum range.
 */
uint32_t DP_AudioSetMute(DP_PrivateData* pD, uint8_t streamId, DP_AudioMuteMode muteMode);

/**
 * Start playing audio with the given parameters.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] params Parameters to configure audio with.
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_AudioAutoConfig(DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params);

/**
 * Stop current audio, to allow setting up new one.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_AudioStop(DP_PrivateData* pD, uint8_t streamId);

/**
 * Set audio on or off in internal registers
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] mode Audio on/off mode.
 * @return CDN_EOK success
 * @return CDN_EINVAL If mode is outside of enum range.
 */
uint32_t DP_AudioSetMode(DP_PrivateData* pD, uint8_t streamId, DP_AudioMode mode);

/**
 * Set DSC configuration.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
 * @param[in] dscConfig Parameters to configure DSC module
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_SetDscConfig(DP_PrivateData* pD, uint8_t streamId, const DP_DscConfig* dscConfig);

/**
 * Get DSC configuration.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
 * @param[out] dscConfig Parameters current DSC configuration
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_GetDscConfig(DP_PrivateData* pD, uint8_t streamId, DP_DscConfig* dscConfig);

/**
 * Send PPS packet.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is 1.
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_DscSendPps(DP_PrivateData* pD, uint8_t streamId);

/**
 * Function set CompressedStream_Flag in VB_ID register.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] val Whether or not to set CompressedStreamFlag.
 * @return CDN_EOK success
 * @return CDN_EINVAL If streamId is outside range.
 */
uint32_t DP_SetCompressedStreamFlag(DP_PrivateData* pD, uint8_t streamId, bool val);

/**
 * Function resets DSC module.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t DP_DscReset(DP_PrivateData* pD);

/**
 * Function enables MST in DP controller and in directly connected
 * device.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t DP_MstEnable(DP_PrivateData* pD);

/**
 * Function disables MST in DP controller and in directly connected
 * device
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t DP_MstDisable(DP_PrivateData* pD);

/**
 * Function enables stream for MST mode in DP controller.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @return CDN_EOK success
 * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
 * @return CDN_ENOTSUP if MST feature is not enabled
 */
uint32_t DP_MstStreamEnable(DP_PrivateData* pD, uint8_t streamId);

/**
 * Function disables stream for MST mode in DP controller.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @return CDN_EOK success
 * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
 * @return CDN_ENOTSUP if MST feature is not enabled
 */
uint32_t DP_MstStreamDisable(DP_PrivateData* pD, uint8_t streamId);

/**
 * Function calculate payload base on current stream configuration
 * Next configures controller and sends payload allocate request.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] sinkDevice Sink device associated with stream
 * @return CDN_EOK success
 * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
 */
uint32_t DP_MstAllocateVcpi(DP_PrivateData* pD, uint8_t streamId, DP_SinkDevice* sinkDevice);

/**
 * Function clear payload for given stream and sink device Next
 * configures controller and sends payload allocate request.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream to be configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] sinkDevice Sink device associated with stream
 * @return CDN_EOK success
 * @return CDN_EINVAL if streamID is bigger than DP_MAX_NUMBER_OF_STREAMS - 1
 */
uint32_t DP_MstDeallocateVcpi(DP_PrivateData* pD, uint8_t streamId, DP_SinkDevice* sinkDevice);

/**
 * Get number of connected sink devices for MST mode
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] sinkCount Number of connected sinks
 * @return CDN_EOK success
 * @return CDN_EINVAL if cannot count all sink devices
 */
uint32_t DP_MstGetSinkCount(DP_PrivateData* pD, uint8_t* sinkCount);

/**
 * Returns pointers to sink devices according to number returned by
 * DP_MstGetSinkCount.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] sinkList Array for pointers to sink devices
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or sinkList pointer is NULL.
 */
uint32_t DP_MstGetSinkList(const DP_PrivateData* pD, const DP_SinkDevice** sinkList);

/**
 * Function used to enable encryption in MST mode. Function should be
 * called after authentication     succeed. Before any payload is
 * allocated.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] enable 'true'  - enable encryption
 *    'false' - disable encryption
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t DP_MstSetEncryptionEnable(DP_PrivateData* pD, bool enable);

/**
 * Function used to configure encryption for given stream. It enables
 * or disables encryption for all slots used by given stream.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId ID of stream which encryption is configured. Maximum stream ID is DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] enable 'true'  - enable encryption
 *    'false' - disable encryption
 * @return CDN_EOK success
 * @return CDN_EINVAL If no payload allocated for given streamId
 */
uint32_t DP_MstSetEncryption(DP_PrivateData* pD, uint8_t streamId, bool enable);

/**
 * Scan current topology. User can scan topology at initial state.
 * Scanning is done automatically by DP_MstHpdIrq function     always
 * when something is changed in topology.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 * @return CDN_ENOTSUP if MST is disabled
 */
uint32_t DP_MstScanTopology(DP_PrivateData* pD);

/**
 * Reads EDID from remote sink device using sideband channel.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] sinkDevice sink device from which EDID shall be read
 * @param[in] block EDID block to read
 * @param[in] edidResponse Pointer structure to be filled with response
 * @return CDN_EOK success
 * @return CDN_EINVAL if any of input parameter has wrong value or is a NULL pointer
 * @return CDN_EIO if MST is not configured correctly or if an transmission error occurs
 */
uint32_t DP_MstReadRemoteEdid(const DP_PrivateData* pD, DP_SinkDevice* sinkDevice, uint32_t block, DP_ReadEdidResponse* edidResponse);

/**
 * Handle HPD IRQ. It handles incoming sideband messages.     It
 * should be called if MST is enabled and HPD pulse event occurs.
 * This function must not be called inside interrupt handling routine.
 * @param[in] pD Driver state info specific to this instance.
 * @return CDN_EOK success
 * @return CDN_EINVAL if pD is NULL
 * @return CDN_EIO if MST is not configured correctly
 */
uint32_t DP_MstHpdIrq(DP_PrivateData* pD);

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */

#endif  /* DP_IF_H */
