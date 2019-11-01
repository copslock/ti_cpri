/******************************************************************************
 *
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
 *
 ******************************************************************************
 *
 * dp_audio.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-41 "Number of comments blocks before and inside function" */

#include "dp_if.h"
#include "dp_priv.h"
#include "dp_mst.h"
#include "dp_sanity.h"
#include "dp_register.h"

#include "mhdp_apb_regs.h"
#include "cps_drv.h"

#include "cdn_errno.h"
#include "cdn_stdint.h"

typedef struct RegisterConfig_s {
    volatile uint32_t* regAddress;
    uint32_t regValue;
} RegisterConfig;

uint32_t DP_AudioSetMute(DP_PrivateData* pD, uint8_t streamId, DP_AudioMuteMode muteMode)
{
    const uint8_t audioMuteBitPos = 4U; /* As in DP specification, VB-ID. */
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VB_ID_p);
    addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    DP_WriteFieldRequest req = {.addr = addr,
                                .startBit = audioMuteBitPos,
                                .bitCount = 1U,
                                .val = ((DP_AUDIO_MUTE == muteMode) ? (1U << audioMuteBitPos) : 0U)};

    retVal = DP_AudioSetMuteSF(pD, muteMode);

    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal)
    {
        retVal = DP_WriteField(pD, &req);
    }

    if (CDN_EOK == retVal)
    {
        /* Register read - to make sure uCPU has finished writing the register. */
        regTransfer.addr = req.addr;
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_AudioSetMode(DP_PrivateData* pD, uint8_t streamId, DP_AudioMode mode)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_AudioSetModeSF(pD, mode);
    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal)
    {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].AUDIO_PACK_CONTROL_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__AUDIO_PACK_CONTROL_P, AUDIO_PACK_EN, 0U, (uint8_t)mode);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        /* Register read - to make sure uCPU has finished writing the register. */
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint16_t getLanesParameter(const DP_AudioParams* params)
{
    uint16_t lanesParam;

    if (params->channelCount == 2U) {
        if (params->laneCount == 1U) {
            lanesParam = 1U;
        } else {
            lanesParam = 3U;
        }
    } else {
        lanesParam = 0U;
    }

    return lanesParam;
}

static uint32_t getI2SPortEnableValue(uint32_t channelCount)
{
    uint32_t I2S_DEC_PORT_EN_Val;

    if (channelCount == 2U) {
        I2S_DEC_PORT_EN_Val = 0x1U;
    } else if (channelCount == 4U) {
        I2S_DEC_PORT_EN_Val = 0x3U;
    } else {
        I2S_DEC_PORT_EN_Val = 0xFU;
    }

    return I2S_DEC_PORT_EN_Val;
}

static void setStatusBitRegisters(const DP_PrivateData* pD, uint8_t streamId, uint32_t channelCount) {
    uint32_t i;
    uint32_t regVal;
    uint32_t wordLength = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__STTS_BIT_CH01_P, WORD_LENGTH_CH0, 0U, 0x2U)
                          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__STTS_BIT_CH01_P, WORD_LENGTH_CH1, 0U, 0x2U);

    volatile uint32_t* regAddress = &pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].STTS_BIT_CH01_p;

    for (i = 0U; i < ((channelCount + 1U) / 2U); i++) {
        regVal = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__STTS_BIT_CH01_P, CHANNEL_NUM_CH0, 0U, i * 2U)
                 | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__STTS_BIT_CH01_P, CHANNEL_NUM_CH1, 0U, (i * 2U) + 1U);

        CPS_REG_WRITE(regAddress, (wordLength | regVal));
        regAddress++;
    }
}

static void audioConfigCoreI2S(const DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params)
{
    uint8_t i;
    uint32_t channelCount = params->channelCount;
    uint32_t maxChannelNumber = channelCount - 1U;
    uint32_t numOfI2SPorts = (channelCount / 2U) - 1U;
    struct MHDP_ApbRegs_s* regBase = pD->regBase;

    typedef struct {
        uint8_t samplingFrequency;
        uint8_t orginalSamplingFrequency;

    } ChannelSampling;

    static ChannelSampling channelSamplings[] = {{0x3U, 0xCU},     /* DP_AUDIO_FREQ_32 */
                                                 {0x2U, 0xDU},     /* DP_AUDIO_FREQ_48 */
                                                 {0xAU, 0x5U},     /* DP_AUDIO_FREQ_96 */
                                                 {0xEU, 0x1U},     /* DP_AUDIO_FREQ_192 */
                                                 {0x0U, 0xFU},     /* DP_AUDIO_FREQ_44_1 */
                                                 {0x8U, 0x7U},     /* DP_AUDIO_FREQ_88_2 */
                                                 {0xCU, 0x3U}};    /* DP_AUDIO_FREQ_176_4 */

    RegisterConfig config[] = {
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].FIFO_CNTL_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__FIFO_CNTL_P, SYNC_WR_TO_CH_ZERO, 0U, 0x1U)},
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNFG_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, MAX_NUM_CH, 0U, maxChannelNumber) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, NUM_OF_I2S_PORTS, 0U, numOfI2SPorts) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, AUDIO_TYPE, 0U, 0x2U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, CFG_SUB_PCKT_NUM, 0U, getLanesParameter(params))},
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNFG_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNFG_P, TRANS_SMPL_WIDTH, 0U, 0x2U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNFG_P, AUDIO_SAMPLE_WIDTH, 0U, params->width) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNFG_P, AUDIO_CH_NUM, 0U, maxChannelNumber) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNFG_P, I2S_DEC_PORT_EN, 0U, getI2SPortEnableValue(channelCount))},
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].COM_CH_STTS_BITS_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__COM_CH_STTS_BITS_P, BYTE0, 0U, 0x4U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__COM_CH_STTS_BITS_P, SAMPLING_FREQ, 0U, channelSamplings[params->freq].samplingFrequency) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__COM_CH_STTS_BITS_P, ORIGINAL_SAMP_FREQ, 0U, channelSamplings[params->freq].orginalSamplingFrequency)},
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNTL_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNTL_P, I2S_DEC_START, 0U, 0x1U)},
        { &regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNTL_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNTL_P, SMPL2PKT_EN, 0U, 0x1U)}
    };

    setStatusBitRegisters(pD, streamId, channelCount);

    for (i = 0U; i < 6U; i++) {
        CPS_REG_WRITE(config[i].regAddress, config[i].regValue);
    }
}

#ifdef CDNS_NEVER_SPDIF
static void audioConfigCoreNoI2s(const DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params)
{
    uint8_t i;
    uint16_t lanesParameter = getLanesParameter(params);

    RegisterConfig config [] = {
        { &pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].FIFO_CNTL_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__FIFO_CNTL_P, SYNC_WR_TO_CH_ZERO, 0U, 0x1U)},
        { &pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNFG_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, MAX_NUM_CH, 0U, 0x1U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, AUDIO_TYPE, 0U, 0x2U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNFG_P, CFG_SUB_PCKT_NUM, 0U, lanesParameter)},
        { &pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNTL_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNTL_P, SMPL2PKT_EN, 0U, 0x1U)},
        { &pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SPDIF_CTRL_ADDR_p,
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_AVG_WIN, 0U, 0x7U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_THRSH, 0U, 0xE0U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_FIFO_MID_RANGE, 0U, 0xE0U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_BYPASS, 0U, 0x1U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_AVG_SEL, 0U, 0x1U) |
          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_ENABLE, 0U, 0x1U)}
    };

    for (i = 0U; i < 4U; i++) {
        CPS_REG_WRITE(config[i].regAddress, config[i].regValue);
    }
}
#endif /* CDNS_NEVER_SPDIF */

static uint32_t setCmLaneCtrl(DP_PrivateData* pD, uint8_t stream, const uint32_t referenceCycles)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint8_t streamId = stream;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_clock_meter[0].CM_LANE_CTRL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_clock_meter[0]);

    if (referenceCycles != 0U) {
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_CLOCK_METER__CM_LANE_CTRL_P, LANE_REF_CYC, 0U, referenceCycles);
    } else {
        regTransfer.val = 0U;
    }

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t setAudioPackStatus(DP_PrivateData* pD, uint8_t stream, const uint32_t audioTsVersion)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint8_t streamId = stream;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].AUDIO_PACK_STATUS_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__AUDIO_PACK_STATUS_P, AUDIO_TS_VERSION, 0U, audioTsVersion);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configAudioCore(DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params)
{
    DP_RegisterTransfer regTransfer = {0U};

#ifdef CDNS_NEVER_SPDIF
    if (params->type == DP_AUDIO_TYPE_I2S) {
        audioConfigCoreI2S(pD, streamId, params);
    } else {
        audioConfigCoreNoI2s(pD, streamId, params);
    }
#else /* CDNS_NEVER_SPDIF */
    audioConfigCoreI2S(pD, streamId, params);
#endif /* CDNS_NEVER_SPDIF */

    /* Register read - to make sure uCPU has finished writing the register. */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_clock_meter[0].CM_CTRL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_clock_meter[0]);
    return DP_ReadRegister(pD, &regTransfer);
}

uint32_t DP_AudioAutoConfig(DP_PrivateData* pD, uint8_t streamId, const DP_AudioParams* params)
{
    const uint32_t audioTsVersion = 0x11U; /* Audio timestamp version */
    const uint32_t referenceCycles = 0x8000U;
    uint32_t retVal;

    retVal = DP_AudioAutoConfigSF(pD, params);
    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal)
    {
        retVal = setAudioPackStatus(pD, streamId, audioTsVersion);
    }

    if (CDN_EOK == retVal)
    {
        retVal = setCmLaneCtrl(pD, streamId, referenceCycles);
    }

    if (CDN_EOK == retVal)
    {
        retVal = DP_AudioSetMode(pD, streamId, DP_AUDIO_MODE_ON);
    }

    if (CDN_EOK == retVal)
    {
        retVal = setCmLaneCtrl(pD, streamId, referenceCycles);
    }

#ifdef CDNS_NEVER_SPDIF
    if ((CDN_EOK == retVal) && (DP_AUDIO_TYPE_I2S != params->type))
    {
        DP_RegisterTransfer regTransfer = {0U};
        /* TODO: Handle CAR correctly */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_aif_car_p);
        regTransfer.val = 0xFFU;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }
#endif /* CDNS_NEVER_SPDIF */

    if (CDN_EOK == retVal)
    {
        retVal = setCmLaneCtrl(pD, streamId, 0U);
    }

    if (CDN_EOK == retVal)
    {
        retVal = configAudioCore(pD, streamId, params);
    }

    return retVal;
}

static void resetAudioSrcCntlRegister(const DP_PrivateData* pD, uint8_t streamId)
{
    struct MHDP_ApbRegs_s *mhdpApbReg = pD->regBase;

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNTL_p, 0U);

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNTL_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__AUDIO_SRC_CNTL_P, SW_RST, 0U, 0x1U));

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNTL_p, 0U);
}

static void resetSmpl2PktCntlRegister(const DP_PrivateData* pD, uint8_t streamId)
{
    struct MHDP_ApbRegs_s *mhdpApbReg = pD->regBase;

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNTL_p, 0U);

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNTL_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SMPL2PKT_CNTL_P, SW_RST, 0U, 0x1U));

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].SMPL2PKT_CNTL_p, 0U);

}

static void resetFifoCntlRegister(const DP_PrivateData* pD, uint8_t streamId)
{
    struct MHDP_ApbRegs_s *mhdpApbReg = pD->regBase;

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].FIFO_CNTL_p, 0U);

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].FIFO_CNTL_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__FIFO_CNTL_P, FIFO_SW_RST, 0U, 0x1U));

    CPS_REG_WRITE(&mhdpApbReg->mhdp_apb_regs.mhdp_audio_decoder[streamId].FIFO_CNTL_p, 0U);

}

#ifdef CDNS_NEVER_SPDIF
static void resetSpdifCtrlAddrRegister(const DP_PrivateData* pD, uint8_t streamId) {

/* Can be called as part of resetAudioRegisters */
    uint32_t reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_AVG_WIN, 0U, 0x7U)
                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_THRSH, 0U, 0xE0U)
                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_FIFO_MID_RANGE, 0U, 0xE0U)
                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_JITTER_BYPASS, 0U, 0x1U)
                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_AVG_SEL, 0U, 0x1U)
                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_AUDIO_DECODER__SPDIF_CTRL_ADDR_P, SPDIF_ENABLE, 0U, 0x0U);

    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].SPDIF_CTRL_ADDR_p, reg);

}
#endif /* CDNS_NEVER_SPDIF */

static void resetAudioSrcCnfgRegister(const DP_PrivateData* pD, uint8_t streamId) {
    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_audio_decoder[streamId].AUDIO_SRC_CNFG_p, 0U);
}

static void resetAudioRegisters(const DP_PrivateData* pD, uint8_t streamId)
{
    static void (*resetRegister[]) (const DP_PrivateData*, uint8_t) = {
        resetAudioSrcCnfgRegister,
#ifdef CDNS_NEVER_SPDIF
        resetSpdifCtrlAddrRegister,
#endif /* CDNS_NEVER_SPDIF */
        resetAudioSrcCntlRegister,
        resetSmpl2PktCntlRegister,
        resetFifoCntlRegister
    };

    uint8_t i;
    static uint8_t count = (uint8_t)(sizeof(resetRegister) / sizeof(resetRegister[0]));
    for (i = 0U; i < count; i++) {
        resetRegister[i](pD, streamId);
    }
}

uint32_t DP_AudioStop(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_AudioStopSF(pD);
    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal)
    {

        resetAudioRegisters(pD, streamId);

        /* TODO: Handle CAR correctly. */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_aif_car_p);
        regTransfer.val = 0x5FU;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_aif_car_p);
        regTransfer.val = 0x0FU;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        /* Register read - to make sure uCPU has finished writing the register. */
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    return retVal;
}
