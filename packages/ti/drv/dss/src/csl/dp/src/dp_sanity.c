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

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */
/* parasoft-begin-suppress MISRA2012-RULE-8_7 "Functions and objects should not be defined with external linkage if they are referenced in only one translation unit" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "dp_priv.h"
#include "dp_sanity.h"
#include "dp_structs_if.h"

/**
 * Function to validate struct SourceDeviceCapabilities
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_SourceDeviceCapabilitiesSF(const DP_SourceDeviceCapabilities *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->maxLinkRate != DP_LINK_RATE_1_62) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_16) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_43) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_70) &&
            (obj->maxLinkRate != DP_LINK_RATE_3_24) &&
            (obj->maxLinkRate != DP_LINK_RATE_4_32) &&
            (obj->maxLinkRate != DP_LINK_RATE_5_40) &&
            (obj->maxLinkRate != DP_LINK_RATE_8_10)
            )
        {
            ret = CDN_EINVAL;
        }
        if (obj->maxVoltageSwing > (3U))
        {
            ret = CDN_EINVAL;
        }
        if (obj->maxPreemphasis > (3U))
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->laneMapping != DP_LANE_MAPPING_SINGLE_REGULAR) &&
            (obj->laneMapping != DP_LANE_MAPPING_DUAL_LANES_01) &&
            (obj->laneMapping != DP_LANE_MAPPING_DUAL_LANES_23)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->controllersPerPhy != DP_SINGLE_CONTROLLER) &&
            (obj->controllersPerPhy != DP_DUAL_CONTROLLER)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct ReadEdidResponse
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_ReadEdidResponseSF(const DP_ReadEdidResponse *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct DpcdTransfer
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_DpcdTransferSF(const DP_DpcdTransfer *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct I2cTransfer
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_I2cTransferSF(const DP_I2cTransfer *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (obj->addr > (127U))
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct VideoParameters
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_VideoParametersSF(const DP_VideoParameters *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (DP_VideoFormatParamsSF(&obj->vicParams) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->pxEncFormat != DP_PXENC_PXL_RGB) &&
            (obj->pxEncFormat != DP_PXENC_YCBCR_4_4_4) &&
            (obj->pxEncFormat != DP_PXENC_YCBCR_4_2_2) &&
            (obj->pxEncFormat != DP_PXENC_YCBCR_4_2_0) &&
            (obj->pxEncFormat != DP_PXENC_Y_ONLY)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->stereoVidAttr != DP_STEREO_VIDEO_LEFT) &&
            (obj->stereoVidAttr != DP_STEREO_VIDEO_RIGHT)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->btType != DP_BT_601) &&
            (obj->btType != DP_BT_709)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->alignment != DP_ALIGN_MSB) &&
            (obj->alignment != DP_ALIGN_LSB)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct LinkState
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_LinkStateSF(const DP_LinkState *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->linkRate != DP_LINK_RATE_1_62) &&
            (obj->linkRate != DP_LINK_RATE_2_16) &&
            (obj->linkRate != DP_LINK_RATE_2_43) &&
            (obj->linkRate != DP_LINK_RATE_2_70) &&
            (obj->linkRate != DP_LINK_RATE_3_24) &&
            (obj->linkRate != DP_LINK_RATE_4_32) &&
            (obj->linkRate != DP_LINK_RATE_5_40) &&
            (obj->linkRate != DP_LINK_RATE_8_10)
            )
        {
            ret = CDN_EINVAL;
        }
        if (obj->laneCount > ((DP_MAX_NUMBER_OF_LANES)))
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct SdpEntry
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_SdpEntrySF(const DP_SdpEntry *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->activeMode != DP_SDP_ACTIVE_NO_VIDEO) &&
            (obj->activeMode != DP_SDP_ACTIVE_VIDEO)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct HdcpTxConfiguration
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_HdcpTxConfigurationSF(const DP_HdcpTxConfiguration *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->hdcpVerSupport != DP_HDCP_2_SUPPORT) &&
            (obj->hdcpVerSupport != DP_HDCP_1_SUPPORT) &&
            (obj->hdcpVerSupport != DP_HDCP_BOTH_SUPPORT)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->contentStreamType != DP_TYPE_0_CONTENT_STREAM) &&
            (obj->contentStreamType != DP_TYPE_1_CONTENT_STREAM)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct Hdcp2TxPublicKey
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_Hdcp2TxPublicKeySF(const DP_Hdcp2TxPublicKey *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct HdcpTxKmEncCustomKey
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_HdcpTxKmEncCustomKeySF(const DP_HdcpTxKmEncCustomKey *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct HdcpDebugRandomNumbers
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_HdcpDebugRandomNumbersSF(const DP_HdcpDebugRandomNumbers *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct HdcpPairingData
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_HdcpPairingDataSF(const DP_HdcpPairingData *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct Hdcp1Keys
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_Hdcp1KeysSF(const DP_Hdcp1Keys *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct AudioParams
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_AudioParamsSF(const DP_AudioParams *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->freq != DP_AUDIO_FREQ_32) &&
            (obj->freq != DP_AUDIO_FREQ_48) &&
            (obj->freq != DP_AUDIO_FREQ_96) &&
            (obj->freq != DP_AUDIO_FREQ_192) &&
            (obj->freq != DP_AUDIO_FREQ_44_1) &&
            (obj->freq != DP_AUDIO_FREQ_88_2) &&
            (obj->freq != DP_AUDIO_FREQ_176_4)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->width != DP_AUDIO_WIDTH_16) &&
            (obj->width != DP_AUDIO_WIDTH_24) &&
            (obj->width != DP_AUDIO_WIDTH_32)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct DscConfig
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_DscConfigSF(const DP_DscConfig *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->bitsPerComponent != DP_BITS_PER_COMPONENT_8) &&
            (obj->bitsPerComponent != DP_BITS_PER_COMPONENT_10)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct Config
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_ConfigSF(const DP_Config *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct Callbacks
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_CallbacksSF(const DP_Callbacks *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct SinkDevice
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_SinkDeviceSF(const DP_SinkDevice *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct FirmwareImage
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_FirmwareImageSF(const DP_FirmwareImage *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct UcpuClock
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_UcpuClockSF(const DP_UcpuClock *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (obj->mhz < (1U))
        {
            ret = CDN_EINVAL;
        }
        if (obj->fraction > (99U))
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct AudioVideoClkCfg
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_AudioVideoClkCfgSF(const DP_AudioVideoClkCfg *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * Function to validate struct PrivateData
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_PrivateDataSF(const DP_PrivateData *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->busType != DP_BUS_TYPE_APB) &&
            (obj->busType != DP_BUS_TYPE_SAPB)
            )
        {
            ret = CDN_EINVAL;
        }
        if (DP_SourceDeviceCapabilitiesSF(&obj->sourceCaps) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (DP_SinkDeviceCapabilitiesSF(&obj->sinkCaps) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        if (DP_LinkStateSF(&obj->linkState) == CDN_EINVAL)
        {
            ret = CDN_EINVAL;
        }
        uint32_t idx_videoParameters;

        for (idx_videoParameters = 0; idx_videoParameters < DP_MAX_NUMBER_OF_STREAMS; idx_videoParameters++)
        {
            if (DP_VideoParametersSF(&obj->videoParameters[idx_videoParameters]) == CDN_EINVAL)
            {
                ret = CDN_EINVAL;
            }
        }
    }

    return ret;
}

/**
 * Function to validate struct VideoFormatParams
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_VideoFormatParamsSF(const DP_VideoFormatParams *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->scanMode != DP_SM_PROGRESSIVE) &&
            (obj->scanMode != DP_SM_INTERLACED)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->hSyncPolarity != DP_SP_ACTIVE_LOW) &&
            (obj->hSyncPolarity != DP_SP_ACTIVE_HIGH)
            )
        {
            ret = CDN_EINVAL;
        }
        if (
            (obj->vSyncPolarity != DP_SP_ACTIVE_LOW) &&
            (obj->vSyncPolarity != DP_SP_ACTIVE_HIGH)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * Function to validate struct SinkDeviceCapabilities
 *
 * @param[in] obj pointer to struct to be verified
 * @returns 0 for valid
 * @returns CDN_EINVAL for invalid
 */
uint32_t DP_SinkDeviceCapabilitiesSF(const DP_SinkDeviceCapabilities *obj)
{
    uint32_t ret = 0;

    if (obj == NULL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        if (
            (obj->maxLinkRate != DP_LINK_RATE_1_62) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_16) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_43) &&
            (obj->maxLinkRate != DP_LINK_RATE_2_70) &&
            (obj->maxLinkRate != DP_LINK_RATE_3_24) &&
            (obj->maxLinkRate != DP_LINK_RATE_4_32) &&
            (obj->maxLinkRate != DP_LINK_RATE_5_40) &&
            (obj->maxLinkRate != DP_LINK_RATE_8_10)
            )
        {
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] config Proposed driver/hardware configuration.
 * @param[out] memReq Size of memory, that needs to be allocated (in bytes).
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction1(const DP_Config* config, const uint32_t* memReq)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (memReq == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_ConfigSF(config) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in,out] pD Driver state info specific to this instance.
 * @param[in] config Specifies driver/hardware configuration.
 * @param[in] callbacks Client-supplied callback functions.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction2(const DP_PrivateData* pD, const DP_Config* config, const DP_Callbacks* callbacks)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (pD == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_ConfigSF(config) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_CallbacksSF(callbacks) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver instance data.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction3(const DP_PrivateData* pD)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver instance data.
 * @param[out] phyPd Pointer to private data of PHY driver's instance to use.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction7(const DP_PrivateData* pD, const DP_SD0801_PrivateData* phyPd)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (phyPd == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] image Pointer to structure with FW image information.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction8(const DP_PrivateData* pD, const DP_FirmwareImage* image)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_FirmwareImageSF(image) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] awaits Pointer to store flag indicating presence of response.
 * @param[in] busType
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction10(const DP_PrivateData* pD, const bool* awaits, const DP_BusType busType)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (awaits == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (busType != DP_BUS_TYPE_APB) &&
        (busType != DP_BUS_TYPE_SAPB)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] busType
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction11(const DP_PrivateData* pD, const DP_BusType busType)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (busType != DP_BUS_TYPE_APB) &&
        (busType != DP_BUS_TYPE_SAPB)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] message Pointer to a buffer to send.
 * @param[out] response Pointer to buffer for receiving msg payload back.
 * @param[in] messageSize Number of bytes to send and receive.
 * @param[in] busType
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction12(const DP_PrivateData* pD, const uint8_t* message, const uint8_t* response, const uint16_t messageSize, const DP_BusType busType)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (message == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (response == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if ((messageSize < ((DP_GENERAL_TEST_ECHO_MIN_PAYLOAD))) || (messageSize > ((DP_GENERAL_TEST_ECHO_MAX_PAYLOAD))))
    {
        ret = CDN_EINVAL;
    }
    else if (
        (busType != DP_BUS_TYPE_APB) &&
        (busType != DP_BUS_TYPE_SAPB)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] ver fw version
 * @param[out] verlib lib version
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction13(const DP_PrivateData* pD, const uint16_t* ver, const uint16_t* verlib)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (ver == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (verlib == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] events pointer to store 32-bit events value
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction14(const DP_PrivateData* pD, const uint32_t* events)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (events == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] debug pointer to store 16-bit debug reg value
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction15(const DP_PrivateData* pD, const uint16_t* debug)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (debug == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] updated Pointer to store flag, whether (true) or not (false) KEEP_ALIVE register
 *    changed since initialization or last call of this function, whichever
 *    happened later.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction16(const DP_PrivateData* pD, const bool* updated)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (updated == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] resp pointer to store response.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction19(const DP_PrivateData* pD, const uint8_t* resp)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (resp == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] ucpuClock Clock, that Xtensa uCPU is running at.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction21(const DP_PrivateData* pD, const DP_UcpuClock* ucpuClock)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_UcpuClockSF(ucpuClock) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] memType Select memory where shall be injected IRAM or DRAM.
 * @param[in] errorType Select whether error shall be injected either in data bits or in check bits.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction23(const DP_PrivateData* pD, const DP_EccErrorMemType memType, const DP_EccErrorType errorType)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (memType != DP_ECC_ERROR_MEM_TYPE_IRAM) &&
        (memType != DP_ECC_ERROR_MEM_TYPE_DRAM)
        )
    {
        ret = CDN_EINVAL;
    }
    else if (
        (errorType != DP_ECC_ERROR_TYPE_DATA) &&
        (errorType != DP_ECC_ERROR_TYPE_CHECK)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId stream number which should be configured
 *    for SST it should be always 0
 *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[in] audioVideoClkCfg audio video clock configuration to be set
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction25(const DP_PrivateData* pD, const uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (streamId > ((DP_MAX_NUMBER_OF_STREAMS - 1U)))
    {
        ret = CDN_EINVAL;
    }
    else if (DP_AudioVideoClkCfgSF(audioVideoClkCfg) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] streamId stream number which should be configured
 *    for SST it should be always 0
 *    for MST proper values are from to DP_MAX_NUMBER_OF_STREAMS - 1
 * @param[out] audioVideoClkCfg audio video clock configuration to be set
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction26(const DP_PrivateData* pD, const uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (audioVideoClkCfg == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (streamId > ((DP_MAX_NUMBER_OF_STREAMS - 1U)))
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] linkRate Link rate to initialize PHY with.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction30(const DP_PrivateData* pD, const DP_LinkRate linkRate)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (linkRate != DP_LINK_RATE_1_62) &&
        (linkRate != DP_LINK_RATE_2_16) &&
        (linkRate != DP_LINK_RATE_2_43) &&
        (linkRate != DP_LINK_RATE_2_70) &&
        (linkRate != DP_LINK_RATE_3_24) &&
        (linkRate != DP_LINK_RATE_4_32) &&
        (linkRate != DP_LINK_RATE_5_40) &&
        (linkRate != DP_LINK_RATE_8_10)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] resp Pointer structure to be filled with response.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction32(const DP_PrivateData* pD, const DP_ReadEdidResponse* resp)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_ReadEdidResponseSF(resp) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Power mode
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction34(const DP_PrivateData* pD, const DP_PwrMode mode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (mode != DP_POWER_NORMAL_OPERATION) &&
        (mode != DP_POWER_DOWN) &&
        (mode != DP_POWER_AUX_ONLY)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] caps Pointer to structure with source capabilities.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction35(const DP_PrivateData* pD, const DP_SourceDeviceCapabilities* caps)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_SourceDeviceCapabilitiesSF(caps) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] caps Pointer to structure to be filled with sink capabilities.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction36(const DP_PrivateData* pD, const DP_SinkDeviceCapabilities* caps)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (caps == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] pattern Test pattern to be transmitted. Selecting PATTERN_DISABLE will stop
 *    transmitting test pattern.
 * @param[in] linkParams Pointer to structure containing main link parameters to be used. If
 *    pattern is PATTERN_DISABLE, main link parameters will not be changed.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction38(const DP_PrivateData* pD, const DP_TestPattern pattern, const DP_LinkState* linkParams)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (pattern != DP_PATTERN_PRBS7) &&
        (pattern != DP_PATTERN_TPS1) &&
        (pattern != DP_PATTERN_TPS2) &&
        (pattern != DP_PATTERN_TPS3) &&
        (pattern != DP_PATTERN_TPS4) &&
        (pattern != DP_PATTERN_80_BIT) &&
        (pattern != DP_PATTERN_D10_2) &&
        (pattern != DP_PATTERN_SYMBOL_ERM) &&
        (pattern != DP_PATTERN_CP2520_1) &&
        (pattern != DP_PATTERN_CP2520_2) &&
        (pattern != DP_PATTERN_CP2520_3) &&
        (pattern != DP_PATTERN_DISABLE)
        )
    {
        ret = CDN_EINVAL;
    }
    else if (DP_LinkStateSF(linkParams) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to DPCD read operation.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction39(const DP_PrivateData* pD, const DP_DpcdTransfer* request)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_DpcdTransferSF(request) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction40(const DP_PrivateData* pD, const DP_DpcdTransfer* transfer)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (transfer == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Parameters related to I2C-over-AUX read operation.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction45(const DP_PrivateData* pD, const DP_I2cTransfer* request)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_I2cTransferSF(request) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with request structure, to be filled with results from the
 *    response. If, as result of an error, more bytes were read, than were
 *    requested, only as many bytes as were requested will be copied to the
 *    buffer, to avoid overflow.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction46(const DP_PrivateData* pD, const DP_I2cTransfer* transfer)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (transfer == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] resultLt Result of Link Training process, according to TrainingStatus enum.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction53(const DP_PrivateData* pD, const DP_TrainingStatus* resultLt)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (resultLt == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[out] vicParams Structure with VIC-relared parameters to fill.
 * @param[in] vicMode VIC mode to take from table and fill into structure.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction60(const DP_VideoFormatParams* vicParams, const DP_VicModes vicMode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (vicParams == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (vicMode != DP_VIC_MODE_1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_1_60HZ) &&
        (vicMode != DP_VIC_MODE_2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_2_60HZ) &&
        (vicMode != DP_VIC_MODE_3_59_94HZ) &&
        (vicMode != DP_VIC_MODE_3_60HZ) &&
        (vicMode != DP_VIC_MODE_4_59_94HZ) &&
        (vicMode != DP_VIC_MODE_4_60HZ) &&
        (vicMode != DP_VIC_MODE_5_59_94HZ) &&
        (vicMode != DP_VIC_MODE_5_60HZ) &&
        (vicMode != DP_VIC_MODE_6_59_94HZ) &&
        (vicMode != DP_VIC_MODE_6_60HZ) &&
        (vicMode != DP_VIC_MODE_7_59_94HZ) &&
        (vicMode != DP_VIC_MODE_7_60HZ) &&
        (vicMode != DP_VIC_MODE_8_MODE1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_8_MODE1_60HZ) &&
        (vicMode != DP_VIC_MODE_8_MODE2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_8_MODE2_60HZ) &&
        (vicMode != DP_VIC_MODE_9_MODE1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_9_MODE1_60HZ) &&
        (vicMode != DP_VIC_MODE_9_MODE2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_9_MODE2_60HZ) &&
        (vicMode != DP_VIC_MODE_10_59_94HZ) &&
        (vicMode != DP_VIC_MODE_10_60HZ) &&
        (vicMode != DP_VIC_MODE_11_59_94HZ) &&
        (vicMode != DP_VIC_MODE_11_60HZ) &&
        (vicMode != DP_VIC_MODE_12_MODE_1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_12_MODE_1_60HZ) &&
        (vicMode != DP_VIC_MODE_12_MODE_2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_12_MODE_2_60HZ) &&
        (vicMode != DP_VIC_MODE_13_MODE_1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_13_MODE_1_60HZ) &&
        (vicMode != DP_VIC_MODE_13_MODE_2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_13_MODE_2_60HZ) &&
        (vicMode != DP_VIC_MODE_14_59_94HZ) &&
        (vicMode != DP_VIC_MODE_14_60HZ) &&
        (vicMode != DP_VIC_MODE_15_59_94HZ) &&
        (vicMode != DP_VIC_MODE_15_60HZ) &&
        (vicMode != DP_VIC_MODE_16_59_94HZ) &&
        (vicMode != DP_VIC_MODE_16_60HZ) &&
        (vicMode != DP_VIC_MODE_17_50HZ) &&
        (vicMode != DP_VIC_MODE_18_50HZ) &&
        (vicMode != DP_VIC_MODE_19_50HZ) &&
        (vicMode != DP_VIC_MODE_20_50HZ) &&
        (vicMode != DP_VIC_MODE_21_50HZ) &&
        (vicMode != DP_VIC_MODE_22_50HZ) &&
        (vicMode != DP_VIC_MODE_23_MODE_1_50HZ) &&
        (vicMode != DP_VIC_MODE_23_MODE_2_50HZ) &&
        (vicMode != DP_VIC_MODE_23_MODE_3_50HZ) &&
        (vicMode != DP_VIC_MODE_24_MODE_1_50HZ) &&
        (vicMode != DP_VIC_MODE_24_MODE_2_50HZ) &&
        (vicMode != DP_VIC_MODE_24_MODE_3_50HZ) &&
        (vicMode != DP_VIC_MODE_25_50HZ) &&
        (vicMode != DP_VIC_MODE_26_50HZ) &&
        (vicMode != DP_VIC_MODE_27_MODE_1_50HZ) &&
        (vicMode != DP_VIC_MODE_27_MODE_2_50HZ) &&
        (vicMode != DP_VIC_MODE_27_MODE_3_50HZ) &&
        (vicMode != DP_VIC_MODE_28_MODE_1_50HZ) &&
        (vicMode != DP_VIC_MODE_28_MODE_2_50HZ) &&
        (vicMode != DP_VIC_MODE_28_MODE_3_50HZ) &&
        (vicMode != DP_VIC_MODE_29_50HZ) &&
        (vicMode != DP_VIC_MODE_30_50HZ) &&
        (vicMode != DP_VIC_MODE_31_50HZ) &&
        (vicMode != DP_VIC_MODE_32_23_97HZ) &&
        (vicMode != DP_VIC_MODE_32_24HZ) &&
        (vicMode != DP_VIC_MODE_33_25HZ) &&
        (vicMode != DP_VIC_MODE_34_29_97HZ) &&
        (vicMode != DP_VIC_MODE_34_30HZ) &&
        (vicMode != DP_VIC_MODE_35_59_94HZ) &&
        (vicMode != DP_VIC_MODE_35_60HZ) &&
        (vicMode != DP_VIC_MODE_36_59_94HZ) &&
        (vicMode != DP_VIC_MODE_36_60HZ) &&
        (vicMode != DP_VIC_MODE_37_50HZ) &&
        (vicMode != DP_VIC_MODE_38_50HZ) &&
        (vicMode != DP_VIC_MODE_39_50HZ) &&
        (vicMode != DP_VIC_MODE_40_100HZ) &&
        (vicMode != DP_VIC_MODE_41_100HZ) &&
        (vicMode != DP_VIC_MODE_42_100HZ) &&
        (vicMode != DP_VIC_MODE_43_100HZ) &&
        (vicMode != DP_VIC_MODE_44_100HZ) &&
        (vicMode != DP_VIC_MODE_45_100HZ) &&
        (vicMode != DP_VIC_MODE_46_119_88HZ) &&
        (vicMode != DP_VIC_MODE_46_120HZ) &&
        (vicMode != DP_VIC_MODE_47_119_88HZ) &&
        (vicMode != DP_VIC_MODE_47_120HZ) &&
        (vicMode != DP_VIC_MODE_48_119_88HZ) &&
        (vicMode != DP_VIC_MODE_48_120HZ) &&
        (vicMode != DP_VIC_MODE_49_119_88HZ) &&
        (vicMode != DP_VIC_MODE_49_120HZ) &&
        (vicMode != DP_VIC_MODE_50_119_88HZ) &&
        (vicMode != DP_VIC_MODE_50_120HZ) &&
        (vicMode != DP_VIC_MODE_51_119_88HZ) &&
        (vicMode != DP_VIC_MODE_51_120HZ) &&
        (vicMode != DP_VIC_MODE_52_200HZ) &&
        (vicMode != DP_VIC_MODE_53_200HZ) &&
        (vicMode != DP_VIC_MODE_54_200HZ) &&
        (vicMode != DP_VIC_MODE_55_200HZ) &&
        (vicMode != DP_VIC_MODE_56_239HZ) &&
        (vicMode != DP_VIC_MODE_56_240HZ) &&
        (vicMode != DP_VIC_MODE_57_239HZ) &&
        (vicMode != DP_VIC_MODE_57_240HZ) &&
        (vicMode != DP_VIC_MODE_58_239HZ) &&
        (vicMode != DP_VIC_MODE_58_240HZ) &&
        (vicMode != DP_VIC_MODE_59_239HZ) &&
        (vicMode != DP_VIC_MODE_59_240HZ) &&
        (vicMode != DP_VIC_MODE_60_23_97HZ) &&
        (vicMode != DP_VIC_MODE_60_24HZ) &&
        (vicMode != DP_VIC_MODE_61_25HZ) &&
        (vicMode != DP_VIC_MODE_62_29_97HZ) &&
        (vicMode != DP_VIC_MODE_62_30HZ) &&
        (vicMode != DP_VIC_MODE_63_119_88HZ) &&
        (vicMode != DP_VIC_MODE_63_120HZ) &&
        (vicMode != DP_VIC_MODE_64_100HZ) &&
        (vicMode != DP_VIC_MODE_97_60HZ) &&
        (vicMode != DP_VIC_MODE_VESA_800X600P_60HZ) &&
        (vicMode != DP_VIC_MODE_VESA_720X400P_70HZ) &&
        (vicMode != DP_VIC_MODE_VESA_720X400P_88HZ) &&
        (vicMode != DP_VIC_MODE_VESA_640X480P_60HZ) &&
        (vicMode != DP_VIC_MODE_VESA_640X480P_67HZ) &&
        (vicMode != DP_VIC_MODE_VESA_640X480P_72HZ) &&
        (vicMode != DP_VIC_MODE_VESA_640X480P_75HZ) &&
        (vicMode != DP_VIC_MODE_VESA_800X600P_56HZ) &&
        (vicMode != DP_VIC_MODE_VESA_800X600P_72HZ) &&
        (vicMode != DP_VIC_MODE_VESA_832X624P_75HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1024X768I_87HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1024X768P_60HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1024X768P_72HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1024X768P_75HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1280X1024P_75HZ) &&
        (vicMode != DP_VIC_MODE_VESA_1152X870P_75HZ) &&
        (vicMode != DP_VIC_MODE_126_60HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_1_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_2_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_4_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_14_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_16_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_17_50HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_25_50HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_28_MODE_1_50HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_31_50HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_32_23_97HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_35_59_94HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_47_119_88HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_49_119_88HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_52_200HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_57_239HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_58_239HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_61_25HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_63_119_88HZ) &&
        (vicMode != DP_VIC_MODE_DUMMY_126_60HZ) &&
        (vicMode != DP_VIC_MODE_COUNT)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] parameters Structure with video parameters to set.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction61(const DP_PrivateData* pD, const DP_VideoParameters* parameters)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_VideoParametersSF(parameters) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] hSyncPolarity Horizontal sync pulse polarity to set in MSA.
 * @param[in] vSyncPolarity Vertical sync pulse polarity to set in MSA.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction62(const DP_PrivateData* pD, const DP_SyncPolarity hSyncPolarity, const DP_SyncPolarity vSyncPolarity)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (hSyncPolarity != DP_SP_ACTIVE_LOW) &&
        (hSyncPolarity != DP_SP_ACTIVE_HIGH)
        )
    {
        ret = CDN_EINVAL;
    }
    else if (
        (vSyncPolarity != DP_SP_ACTIVE_LOW) &&
        (vSyncPolarity != DP_SP_ACTIVE_HIGH)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] linkState Link State.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction65(const DP_PrivateData* pD, const DP_LinkState* linkState)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (linkState == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest AUX status
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction67(const DP_PrivateData* pD, const DP_AuxStatus* status)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (status == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Latest I2C-over-AUX status
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction70(const DP_PrivateData* pD, const DP_I2cStatus* status)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (status == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] packetData Data of SDP entry to write
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction77(const DP_PrivateData* pD, const DP_SdpEntry* packetData)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_SdpEntrySF(packetData) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] entryID 4 Most significant bits of the packet memory containing SDP to remove
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction78(const DP_PrivateData* pD, const uint8_t entryID)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (entryID > ((DP_MAX_NUMBER_OF_SDPS - 1U)))
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] config Pointer to structure with configuration for HDCP transmitter.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction79(const DP_PrivateData* pD, const DP_HdcpTxConfiguration* config)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_HdcpTxConfigurationSF(config) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] key Pointer to structure with HDCP 2.x public key.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction80(const DP_PrivateData* pD, const DP_Hdcp2TxPublicKey* key)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_Hdcp2TxPublicKeySF(key) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] key Pointer to structure with km-key.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction81(const DP_PrivateData* pD, const DP_HdcpTxKmEncCustomKey* key)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_HdcpTxKmEncCustomKeySF(key) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] numbers Pointer to structure with random numbers.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction82(const DP_PrivateData* pD, const DP_HdcpDebugRandomNumbers* numbers)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_HdcpDebugRandomNumbersSF(numbers) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] pairingData Pointer to filled structure containing HDCP 2.x pairing data.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction84(const DP_PrivateData* pD, const DP_HdcpPairingData* pairingData)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_HdcpPairingDataSF(pairingData) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] keySet Pointer to filled structure containing HDCP 1.x Device Key Set.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction85(const DP_PrivateData* pD, const DP_Hdcp1Keys* keySet)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_Hdcp1KeysSF(keySet) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] status Pointer to structure with HDCP TX status, to be filled.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction88(const DP_PrivateData* pD, const DP_HdcpTxStatus* status)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (status == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] pairingData Pointer to structure with HDCP pairing data to be filled.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction94(const DP_PrivateData* pD, const DP_HdcpPairingData* pairingData)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (pairingData == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] list Pointer to structure with list of all receiver IDs, and their count,
 *    to be filled.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction97(const DP_PrivateData* pD, const DP_HdcpRecvIdList* list)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (list == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] muteMode Select mute/unmute.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction102(const DP_PrivateData* pD, const DP_AudioMuteMode muteMode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (muteMode != DP_AUDIO_MUTE) &&
        (muteMode != DP_AUDIO_UNMUTE)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] params Parameters to configure audio with.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction103(const DP_PrivateData* pD, const DP_AudioParams* params)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_AudioParamsSF(params) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Audio on/off mode.
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction105(const DP_PrivateData* pD, const DP_AudioMode mode)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (
        (mode != DP_AUDIO_MODE_OFF) &&
        (mode != DP_AUDIO_MODE_ON)
        )
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] dscConfig Parameters to configure DSC module
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction106(const DP_PrivateData* pD, const DP_DscConfig* dscConfig)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_DscConfigSF(dscConfig) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] dscConfig Parameters current DSC configuration
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction107(const DP_PrivateData* pD, const DP_DscConfig* dscConfig)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (dscConfig == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] sinkDevice Sink device associated with stream
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction115(const DP_PrivateData* pD, const DP_SinkDevice* sinkDevice)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_SinkDeviceSF(sinkDevice) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] sinkList Array for pointers to sink devices
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction118(const DP_PrivateData* pD, const DP_SinkDevice** sinkList)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (sinkList == NULL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/**
 * A common function to check the validity of API functions with
 * following parameter types
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] sinkDevice sink device from which EDID shall be read
 * @param[in] edidResponse Pointer structure to be filled with response
 * @return 0 success
 * @return CDN_EINVAL invalid parameters
 */
uint32_t DP_SanityFunction122(const DP_PrivateData* pD, const DP_SinkDevice* sinkDevice, const DP_ReadEdidResponse* edidResponse)
{
    /* Declaring return variable */
    uint32_t ret = 0;

    if (DP_PrivateDataSF(pD) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_SinkDeviceSF(sinkDevice) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else if (DP_ReadEdidResponseSF(edidResponse) == CDN_EINVAL)
    {
        ret = CDN_EINVAL;
    }
    else
    {
        /*
         * All 'if ... else if' constructs shall be terminated with an 'else' statement
         * (MISRA2012-RULE-15_7-3)
         */
    }

    return ret;
}

/* parasoft-end-suppress MISRA2012-RULE-8_7 */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRICS-18-3 */
