/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure" */
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
#ifndef DP_STRUCTS_IF_H
#define DP_STRUCTS_IF_H

#include "cdn_stdtypes.h"
#include "dp_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Structures and unions
**********************************************************************/
struct DP_VideoFormatParams_s
{
    /**
     * Video Information Code - An integer value used to identify a particular
     * Video Format, as in ANSI-CEA-861-F standard. '0' means, that video format
     * timings do not correspond to any from the standard.
     * Informative only - when structure is filled manually, this
     * value is not used.
     */
    uint32_t vic;
    /**
     * Total amount of pixel clock periods in a video line, including blanking
     * period.
     */
    uint32_t hTotal;
    /**
     * Amount of pixel clock periods used for active pixels (conveying pixel
     * data) in a video line.
     * For RGB pixel encoding, it has to be a multiple of 2.
     * For other pixel encodings (like YCbCr), it has to be a multiple of 16.
     */
    uint32_t hActive;
    /**
     * Amount of pixel clock periods used for blank pixels (conveying data
     * other than pixel data) in a video line.
     */
    uint32_t hBlank;
    /**
     * Amount of pixel clock periods contributing to horizontal sync signal in
     * a video line.
     */
    uint32_t hSync;
    /**
     * Amount of pixel clock periods contributing to front porch in a video
     * line.
     */
    uint32_t hFrontPorch;
    /**
     * Amount of pixel clock periods contributing to back porch in a video
     * line.
     */
    uint32_t hBackPorch;
    /** Horizontal frequency, expressed in kilohertz (kHz). */
    float64_t hFreq;
    /** Total amount of lines in a video frame, including blanking period. */
    uint32_t vTotal;
    /**
     * Amount of active lines (containing both active and blanking pixels) in
     * a video frame.
     */
    uint32_t vActive;
    /**
     * Amount of blanking lines (containing only blanking pixels) in a video
     * frame. Might be fractional fir interlaced video formats.
     */
    float64_t vBlank;
    /** Amount of blanking lines contributing to vertical sync in a video frame. */
    uint32_t vSync;
    /**
     * Amount of blanking lines contributing to vertical front porch in a video
     * frame.
     */
    uint32_t vFrontPorch;
    /**
     * Amount of blanking lines contributing to vertical back porch in a video
     * frame.
     */
    uint32_t vBackPorch;
    /** Vertical frequency, expressed in hertz (Hz). */
    float64_t vFreq;
    /** Pixel clock frequency, expressed in megahertz (MHz). */
    float64_t pxlFreq;
    /** Video scan mode (progressive / interlaced) */
    DP_ScanMode scanMode;
    /** Polarity of horizontal sync pulse. */
    DP_SyncPolarity hSyncPolarity;
    /** Polarity of vertical sync pulse. */
    DP_SyncPolarity vSyncPolarity;
    /**
     * 4-bit value used as bits R3 - R0 in AVI InfoFrame, related to aspect
     * ratio of active frame potion.
     */
    uint8_t vicR;
    /**
     * 4-bit value used as bits PR3 - PR0 in AVI InfoFrame, related to pixel
     * repetition factor.
     */
    uint8_t vicPR;
};

/** Structure used to store source device capabilities. */
struct DP_SourceDeviceCapabilities_s
{
    /** Maximum link rate supported. */
    DP_LinkRate maxLinkRate;
    /** Maximum lane count supported. */
    uint8_t laneCount;
    /** Spread spectrum clock supported. */
    bool ssc;
    /** Prevents enabling scrambler in DPTX (also outside Link Training). */
    bool scramblerDisable;
    /** Fast Link Training supported. */
    bool fastLinkTraining;
    /** Use Training Pattern Sequence 3 (TPS3), when supported. */
    bool tps3;
    /** Use Training Pattern Sequence 4 (TPS4), when supported. */
    bool tps4;
    /** Maximum voltage swing supported. */
    uint8_t maxVoltageSwing;
    /** Maximum pre-emphasis supported. */
    uint8_t maxPreemphasis;
    /** Force voltage swing level to one set as maximum supported. */
    bool forceVoltageSwing;
    /** Force pre-emphasis level to one set as maximum supported. */
    bool forcePreemphasis;
    /** Specifies lane mapping. */
    DP_LaneMapping laneMapping;
    /** Number of controllers to be used with single PHY. */
    DP_ControllersPerPhy controllersPerPhy;
    /** Whether device is an eDP device or not. */
    bool isEdp;
};

/** Structure used to store sink device capabilities. */
struct DP_SinkDeviceCapabilities_s
{
    /** Maximum link rate supported. */
    DP_LinkRate maxLinkRate;
    /** Maximum lane count supported. */
    uint8_t laneCount;
    /** Spread spectrum clock supported. */
    bool ssc;
    /** Fast Link Training supported */
    bool fastLinkTraining;
    /** Training Pattern Sequence 3 (TPS3) is supported. */
    bool tps3;
    /** Training Pattern Sequence 3 (TPS3) is supported. */
    bool tps4;
    /** Enhanced framing mode is supported. */
    bool enhanced;
    /** ASSR (Alternate Scrambler Seed Reset) mode is supported. */
    bool assr;
    /**
     * Indicates, if sink supports selecting link rate using LINK_BW_SET DPCD
     * register (100h).
     */
    bool linkBwSupported;
    /**
     * Time interval required before reading status of Channel Equalization
     * during Link Training, in microseconds.
     */
    uint32_t TrainingInterval;
    /**
     * Indicates, if sink supports selecting link rate using LINK_RATE_SET DPCD
     * register (115h), specific to eDP.
     */
    bool linkRateSupported;
    /**
     * Number of valid (non-zero) 16-bit entries in SUPPORTED_LINK_RATES DPCD
     * registers(10h - 1Fh), indicating number of link rates sink device
     * supports using LINK_RATE_SET method, specific to eDP.
     */
    uint8_t edpRateCount;
    /**
     * Values of 16-bit entries in SUPPORTED_LINK_RATES DPCD registers,
     * specific to eDP.
     */
    uint16_t edpRatesRegisters[8];
    /**
     * Flags indicating support of DP 1.4 and eDP 1.4 link rates, as defined
     * in DP_LinkRate
     */
    bool ratesSupported[8];
};

/** Reply data struct for DP_ReadEdid and DP_GetEdidReadResponse. */
struct DP_ReadEdidResponse_s
{
    /**
     * Address of buffer used to store response with EDID.
     * Must fit at least 128 bytes.
     */
    uint8_t* buff;
    /** Size of data read (in bytes) */
    uint8_t size;
    /** Number of EDID block, that was read. */
    uint8_t blockNo;
};

/** Structure for performing DPCD read/write operations. */
struct DP_DpcdTransfer_s
{
    /**
     * For reading DPCD registers: Pointer to buffer used to store response
     * with DPCD register read. Must fit at least as many bytes, as were
     * requested to read.
     * For writing DPCD register: Pointer to buffer with values to write to
     * DPCD registers.
     */
    uint8_t* buff;
    /**
     * Address of first DPCD register, to read/write. After operation - address
     * of first DPCD register, that was read/written. If it differs from
     * requested address, an error occurred, which may be checked using
     * DP_GetAuxStatus.
     */
    uint32_t addr;
    /**
     * Size of data to read/write (in bytes). After operation - size of data
     * actually read/written. If it differs from requested size, an error
     * occurred, which may be checked using DP_GetAuxStatus.
     */
    uint16_t size;
};

/** Structure for performing I2C-over-AUX read/write operations. */
struct DP_I2cTransfer_s
{
    /**
     * For Reading from I2C slave over AUX: Pointer to buffer used to store
     * response with bytes read. Must fit at least as many bytes, as were
     * requested to read.
     * For writing to I2C slave over AUX: Pointer to buffer with values to
     * write to I2C slave device.
     */
    uint8_t* buff;
    /**
     * 7-bit address of I2C slave to read/write, placed at bits 0-6 (without
     * direction bit). After operation - address of I2C device, that was
     * read/written. If it differs from requested address, an error occurred,
     * which may be checked using DP_GetAuxStatus and DP_GetI2cStatus functions.
     */
    uint8_t addr;
    /**
     * Whether or not to set MoT (Middle-of-Transaction) flag during the last
     * I2C-over-AUX transaction handling this operation. If "false", DPRX will
     * assert a STOP condition after this operation. Otherwise, it will do the
     * clock stretching until the next transaction happens, asserting Repeated
     * Start condition if address or direction of transfer has changed.
     * If an error occurs, an address-only transaction with MoT bit cleared will
     * be sent regardless of this setting, to terminate I2C-over-AUX sequence.
     */
    bool mot;
    /**
     * Size of data to read/write (in bytes). After operation - size of data
     * actually read/written. If it differs from requested size, an error occurred,
     * which may be checked using DP_GetAuxStatus and DP_GetI2cStatus functions.
     */
    uint16_t size;
};

/** VIC-related video parameters. */
struct DP_VideoParameters_s
{
    /**
     * Pointer to structure containing parameters related to VIC. Structure
     * may be filled manually, or using DP_FillVideoFormat function.
     */
    DP_VideoFormatParams vicParams;
    /** Bits per subpixel or pixel component. */
    uint8_t bitsPerSubpixel;
    /** Pixel encoding format */
    DP_PixelEncodingFormat pxEncFormat;
    /** Stereo video attribute */
    DP_StereoVideoAttr stereoVidAttr;
    /** BT type */
    DP_BtType btType;
    /**
     * When set to 'true', force setting bit 6 in MSA_MISC1 value, for
     * configuring video using SDP, similarly to YCbCr 4:2:0 mode.
     */
    bool forceMiscIgnoreBit;
    /** DSC enabled, 'true' - enable DSC. */
    bool dscEnable;
    /** Alignment of the input pixel data at the pixel interface. */
    DP_PxlAlignment alignment;
};

/** Structure containing parameters of physical link. */
struct DP_LinkState_s
{
    /** Link Rate */
    DP_LinkRate linkRate;
    /** Lane count */
    uint8_t laneCount;
    /** Voltage swing level, one per lane. */
    uint8_t voltageSwing[DP_MAX_NUMBER_OF_LANES];
    /** Pre-emphasis level, one per lane. */
    uint8_t preEmphasis[DP_MAX_NUMBER_OF_LANES];
    /** SSC (Spread-Spectrum Clock) enabled. */
    bool ssc;
};

/** Structure describing single SDP (Secondary Data Packet) entry. */
struct DP_SdpEntry_s
{
    /** Length of data packet, expressed in 4-byte words ( = size in bytes / 4) */
    uint8_t length;
    /** 8-bit value describing SDP type. */
    uint8_t type;
    /** Pointer to beginning of buffer containing SDP itself. */
    uint32_t* packet;
    /** Selects, when SDP will be active. */
    DP_SdpActiveIdleMode activeMode;
};

/** Structure used for values used to configure and control HDCP transmitter. */
struct DP_HdcpTxConfiguration_s
{
    /**
     * HDCP version(s), that shall be supported. '0' - only HDCP 2.2,
     * '1' - only HDCP 1.4, '2' - both (HDCP 2.2 will be attempted first)
     */
    DP_HdcpVerSupport hdcpVerSupport;
    /** activate/stop transmitter (1 to activate, 0 to stop) */
    bool activate;
    /** Content Stream type. For MST operation, type 0 must be selected. */
    DP_ContentStreamType contentStreamType;
    /**
     * Enable km-key encryption. Once enabled (using value 'true'), cannot be
     * disabled, except for reset. When enabled, HDCP controller will expect
     * Master Key, HDCP 1.4 Device Keys and Global Constant (lc128) to be
     * encrypted by custom key, loaded using function DP_SetHdcpKmEncCustomKey
     */
    bool enableKmEncryption;
};

/**
 * Structure containing public key values for HDCP 2.x transmitter. It is
 * called "DCP LLC public key" in HDCP 2.2 specification.
 */
struct DP_Hdcp2TxPublicKey_s
{
    /** Modulus n */
    uint8_t modulusN[384];
    /** Public exponent e */
    uint8_t exponentE[3];
};

/** Structure containing custom key for decrypting other HDCP keys. */
struct DP_HdcpTxKmEncCustomKey_s
{
    /** 16-byte custom key for km-key encryption. */
    uint8_t kmEncCutomKey[16];
};

/** Structure containing random numbers for debug purposes. */
struct DP_HdcpDebugRandomNumbers_s
{
    /** KM random value */
    uint8_t km[16];
    /** RN random value */
    uint8_t rn[8];
    /** KS random value */
    uint8_t ks[16];
    /** RIV random value */
    uint8_t riv[8];
    /** RTX random value */
    uint8_t rtx[8];
};

/** Structure containing pairing associated with HDCP 2.x receiver. */
struct DP_HdcpPairingData_s
{
    /** Receiver ID. */
    uint8_t id[5];
    /** 'm' value associated with Receiver ID. */
    uint8_t m[16];
    /**
     * Value of km key associated with Receiver ID. If km-key encryption is
     * enabled, it will be encrypted while reading and is expected to be
     * encrypted for setting, using 16-byte custom key for km-key encryption.
     */
    uint8_t km[16];
    /**
     * Ekh(km) - km key in a form, that was encrypted (and can be
     * decrypted) by receiver with associated Receiver ID.
     */
    uint8_t ekhKm[16];
};

/** Structure containing HDCP 1.x device keys, with associated KSV. */
struct DP_Hdcp1Keys_s
{
    /** Key Seletion Vector */
    uint8_t ksv[5];
    /**
     * 40 Device Private Keys, 7-bytes each. If km-key encryption is
     * enabled, they are expected to be provided in encrypted form.
     */
    uint8_t keys[280];
};

/** Structure containing status for HDCP. */
struct DP_HdcpTxStatus_s
{
    /** Authenticated - 'true', if receiver is authenticated. */
    bool authenticated;
    /** Repeater - 'true', if receiver is a repeater. */
    bool repeater;
    /** HDCP receiver type - 1, if HDCP receiver supports 1.x . 2 if HDCP 2.x . */
    uint8_t rxType;
    /** AuthStreamId - 'true' for success. */
    bool authStreamIdSuccess;
    /** Last HDCP error. */
    DP_HdcpErrCode lastErr;
    /** Work with ENABLE_1.1_FEATURES */
    bool enable1d1Features;
};

/**
 * Structure containing Receiver IDs (HDCP 2.x) or KSVs (HDCP 1.x) of all
 * receivers, connected directly or via repeaters
 */
struct DP_HdcpRecvIdList_s
{
    /** List of 5-byte entries (Rx IDs / KSVs) - up to 127 of these. */
    uint8_t ids[635];
    /** Number of 5-byte entries in the list (Rx IDs / KSVs) */
    uint8_t count;
};

/** Aggregate of parameters for configuring audio. */
struct DP_AudioParams_s
{
    /** Number of audio channels */
    uint32_t channelCount;
    DP_AudioFreq freq;
    uint32_t laneCount;
    DP_AudioWidth width;
};

/** Configuration for a single RC model range */
struct DP_DscRangeCfg_s
{
    /** Specifies the minimum QP that is allowed to the current range */
    uint32_t rangeMinQp;
    /** Specifies the maximum QP that is allowed to the current range */
    uint32_t rangeMaxQp;
    /** Specifies the target bits per group adjustment to the current range */
    int32_t rangeBpgOffset;
};

/** DSC configuration */
struct DP_DscConfig_s
{
    /** Bits / component for previous reconstructed line buffer */
    uint32_t linebufDepth;
    /** Bits / component to code */
    DP_BitsPerComponent bitsPerComponent;
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
};

/** Configuration parameters passed to probe & init functions. */
struct DP_Config_s
{
    /**
     * Base address of the APB register space.
     *
     */
    struct MHDP_ApbRegs_s* regBase;
    /**
     * Base address of the SAPB (Secure APB) register space.
     *
     */
    struct MHDP_ApbRegs_s* regBaseSapb;
};

/**
 * Structure containing function pointers for event notification callbacks
 * issued by isr().
 * Each call passes the driver's privateData (pD) pointer for instance
 * identification if necessary.
 */
struct DP_Callbacks_s
{
    DP_CbEvent event;
};

/** Structure used for storing information about sink device */
struct DP_SinkDevice_s
{
    /** Connection port of device */
    MST_drm_dp_port* port;
    /** Stores source stream ID */
    uint8_t streamId;
};

/** Structure used for storing addreses and sizes of image of FW to load. */
struct DP_FirmwareImage_s
{
    /** Pointer to instruction memory image. */
    uint8_t* iMem;
    /** Size of instruction memory buffer, in bytes. */
    uint32_t iMemSize;
    /** Pointer to data memory image. */
    uint8_t* dMem;
    /** Size of data memory buffer, in bytes. */
    uint32_t dMemSize;
};

/** Structure used for specifying clock of embedded uCPU. */
struct DP_UcpuClock_s
{
    /** Clock, that Xtensa uCPU is running at, in MHz. */
    uint8_t mhz;
    /**
     * Fractional part of clock, that Xtensa uCPU is running at, in MHz.
     * Expressed in units of 1/100th of MHz (10 kHz)
     */
    uint8_t fraction;
};

/** Structure used for specifying audio video clocks configuration */
struct DP_AudioVideoClkCfg_s
{
    /** dptx video clock configuration (true - enabled, false - disabled) */
    bool videoClockEnable;
    /** source audio clock configuration (true - enabled, false - disabled) */
    bool audioClockEnable;
    /** source packet data clock configuration (true - enabled, false - disabled) */
    bool pktDataClockEnable;
};

/**
 *  @}
 */

#endif  /* DP_STRUCTS_IF_H */
