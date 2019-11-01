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
 * dp_link_policy.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-36 "Function called from more than 5 different functions in one translation unit, DRV-3823" */

#include "dp_if.h"
#include "dp_structs_if.h"
#include "dp_priv.h"
#include "dp_sanity.h"

#include "dp_sd0801_if.h"
#include "dp_sd0801_structs_if.h"

#include "dp_mailbox.h"
#include "dp_register.h"
#include "dp_link_policy.h"
#include "dp_internal.h"

#include "mhdp_apb_regs.h"
#include "cps_drv.h"

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "cdn_log.h"

/**
 * Status of last Clock Recovery (CR) or Channel Equalization (EQ) Link
 * Training phase. For CR phase, there is one such value per lane. For EQ
 * phase, there is one value for entire link.
 */
typedef enum
{
    /** Phase of Link Training succeeded. */
    LT_PHASE_PASS = 0U,
    /** Phase of Link Training failed. */
    LT_PHASE_FAIL = 1U,
} TrainingPhaseStatus;

/**
 * Contains detailed result of single step of Link Training (per lane, where
 * applicable), taken from DPCD registers 0x202 - 0x204.
 */
typedef struct TrainingStepStatus_s
{
    /** Clock Recovery for given lane was successful (field LANEx_CR_DONE). */
    uint8_t crDone[4];
    /**
     * Channel Equalization for given lane was successful (field
     * LANEx_CHANNEL_EQ_DONE).
     */
    uint8_t eqDone[4];
    /** Symbol Lock for given lane was achieved (field LANEx_SYMBOL_LOCKED). */
    uint8_t symbolLocked[4];
    /**
     * Symbol boundaries of all the enabled lanes are aligned with one another
     * (field INTERLANE_ALIGN_DONE)
     */
    uint8_t interlaneAligned;
} TrainingStepStatus;

/*
 * Clear parameters of source device
 * @param[in] caps pointer to source capabilities object
 */
void clearSourceCapabilities(DP_SourceDeviceCapabilities *caps)
{
    /* set default source parameters */
    caps->maxLinkRate = DP_LINK_RATE_1_62;
    caps->laneCount = 0U;
    caps->ssc = false;
    caps->scramblerDisable = false;

    /* disable fast training and patterns */
    caps->fastLinkTraining = false;
    caps->tps3 = false;
    caps->tps4 = false;
    caps->maxVoltageSwing = 0U;
    caps->maxPreemphasis = 0U;
    caps->forceVoltageSwing = false;
    caps->forcePreemphasis = false;
    caps->laneMapping = DP_LANE_MAPPING_SINGLE_REGULAR;
    caps->controllersPerPhy = DP_SINGLE_CONTROLLER;
    caps->isEdp = false;
}

DP_SD0801_LinkRate toLinkRateSd(DP_LinkRate rate)
{
    /* 1:1 mapping */
    /* From  DP_LinkRate to DP_SD0801_LinkRate */
    DP_SD0801_LinkRate ret;
    switch (rate)
    {
    case DP_LINK_RATE_1_62:
        /* 1.62 Gb/s */
        ret = DP_SD0801_LINK_RATE_1_62;
        break;
    case DP_LINK_RATE_2_16:
        /* 2.16 Gb/s */
        ret = DP_SD0801_LINK_RATE_2_16;
        break;
    case DP_LINK_RATE_2_43:
        /* 2.43 Gb/s */
        ret = DP_SD0801_LINK_RATE_2_43;
        break;
    case DP_LINK_RATE_2_70:
        /* 2.7 Gb/s */
        ret = DP_SD0801_LINK_RATE_2_70;
        break;
    case DP_LINK_RATE_3_24:
        /* 3.24 Gb/s */
        ret = DP_SD0801_LINK_RATE_3_24;
        break;
    case DP_LINK_RATE_4_32:
        /* 4.32 Gb/s */
        ret = DP_SD0801_LINK_RATE_4_32;
        break;
    case DP_LINK_RATE_5_40:
        /* 5.4 Gb/s */
        ret = DP_SD0801_LINK_RATE_5_40;
        break;
    default:
        ret = DP_SD0801_LINK_RATE_8_10;
        break;
    }
    return ret;
}

static DP_LinkRate toLinkRateDp(DP_SD0801_LinkRate rate)
{
    /* 1:1 mapping */
    /* From  DP_SD0801_LinkRate to DP_LinkRate */
    DP_LinkRate ret;
    switch (rate)
    {
    case DP_SD0801_LINK_RATE_1_62:
        /* 1.62 Gb/s */
        ret = DP_LINK_RATE_1_62;
        break;
    case DP_SD0801_LINK_RATE_2_16:
        /* 2.16 Gb/s */
        ret = DP_LINK_RATE_2_16;
        break;
    case DP_SD0801_LINK_RATE_2_43:
        /* 2.43 Gb/s */
        ret = DP_LINK_RATE_2_43;
        break;
    case DP_SD0801_LINK_RATE_2_70:
        /* 2.7 Gb/s */
        ret = DP_LINK_RATE_2_70;
        break;
    case DP_SD0801_LINK_RATE_3_24:
        /* 3.24 Gb/s */
        ret = DP_LINK_RATE_3_24;
        break;
    case DP_SD0801_LINK_RATE_4_32:
        /* 4.32 Gb/s */
        ret = DP_LINK_RATE_4_32;
        break;
    case DP_SD0801_LINK_RATE_5_40:
        /* 5.4 Gb/s */
        ret = DP_LINK_RATE_5_40;
        break;
    default:
        ret = DP_LINK_RATE_8_10;
        break;
    }
    return ret;
}

/**
 * Converts LinkState struct from DP driver's one to SD0801 driver's one.
 */
void toSdLinkState (const DP_LinkState* dpState, DP_SD0801_LinkState* sdState)
{
    uint8_t i;

    /* Values for LinkRate are the same. */
    sdState->linkRate = toLinkRateSd(dpState->linkRate);
    sdState->laneCount = dpState->laneCount;
    sdState->ssc = dpState->ssc;

    for (i = 0; i < 4U; i++)
    {
        sdState->voltageSwing[i] = dpState->voltageSwing[i];
        sdState->preEmphasis[i] = dpState->preEmphasis[i];
    }
}

/*
 * Clear parameters of sink device
 * @param[in] caps pointer to sink capabilities object
 */
void clearSinkCapabilities(DP_SinkDeviceCapabilities *caps)
{
    uint8_t i;

    /* set default sink parameters */
    caps->maxLinkRate = DP_LINK_RATE_1_62;
    caps->laneCount = 0U;
    caps->ssc = false;

    /* disable fast training and patterns */
    caps->fastLinkTraining = false;
    caps->tps3 = false;
    caps->tps4 = false;
    caps->enhanced = false;
    caps->assr = false;
    caps->linkBwSupported = false;
    caps->TrainingInterval = 0U;
    caps->linkRateSupported = false;
    caps->edpRateCount = 0U;

    /* clear for each link rate */
    for (i = 0U; i < 8U; i++) {
        caps->edpRatesRegisters[i] = 0U;
        caps->ratesSupported[i] = false;
    }
}

/**
 * Initialize structure for link state to its default values. This does not
 * impact PHY itself in any way.
 */
void clearLinkState(DP_LinkState* state)
{
    uint8_t i;
    state->linkRate = DP_LINK_RATE_1_62;
    state->laneCount = 0U;
    state->ssc = false;

    /* clear link state for each lane */
    for (i = 0U; i < 4U; i++)
    {
        state->voltageSwing[i] = 0U;
        state->preEmphasis[i] = 0U;
    }
}

/**
 * Get most recent version of linkState structure from PHY driver and put values
 * into DP driver.
 */
uint32_t fetchLinkState(DP_PrivateData* pD)
{
    /* Temporary struct, for type conversion */
    DP_SD0801_LinkState tmp;
    uint8_t i;

    uint32_t retVal = DP_SD0801_ReadLinkStat(pD->phyPd, &tmp);

    if (CDN_EOK == retVal)
    {
        pD->linkState.linkRate = toLinkRateDp(tmp.linkRate);
        pD->linkState.laneCount = tmp.laneCount;
        pD->linkState.ssc = tmp.ssc;

        for (i = 0; i < 4U; i++)
        {
            pD->linkState.voltageSwing[i] = tmp.voltageSwing[i];
            pD->linkState.preEmphasis[i] = tmp.preEmphasis[i];
        }
    }
    return retVal;
}

/*
 * Clear training step status
 * @param[in] status pointer to TrainingStepStatus object
 */
static void clearTrainingResult(TrainingStepStatus* status)
{
    uint8_t i;

    /* clear for each lane */
    for (i = 0U; i < 4U; i++) {
        status->crDone[i] = 0U;
        status->eqDone[i] = 0U;
        status->symbolLocked[i] = 0U;
    }

    status->interlaneAligned = 0U;
}

/*
 * Calculate interval time for training
 * @param[in] dpcdReg value of dpcd register
 */
static uint32_t calcTrainingInterval(const uint8_t dpcdReg)
{
    uint32_t result;

    uint8_t trainingIntervalValue = dpcdReg & DP_TRAINING_AUX_RD_INTERVAL_MASK;

    if (trainingIntervalValue == 0x00U) {
        /* set default time */
        result = 400U;
    } else {
        result = (uint32_t)trainingIntervalValue * 4000U;
    }

    return result;
}

/*
 * Set rate support
 * @param[in] sinkCaps pointer to sink capabilities object
 * @param[in] regNumber
 */
static void setRateSupport(DP_SinkDeviceCapabilities* sinkCaps, uint16_t edpRateRegister) {

    sinkCaps->linkRateSupported = true;

    switch (edpRateRegister)
    {
    case 8100U:          /* RBR */
        sinkCaps->ratesSupported[DP_LINK_RATE_1_62] = true;
        break;
    case 10800U:         /* 2.16 Gb/s */
        sinkCaps->ratesSupported[DP_LINK_RATE_2_16] = true;
        break;
    case 12150U:         /* 2.43 Gb/s */
        sinkCaps->ratesSupported[DP_LINK_RATE_2_43] = true;
        break;
    case 13500U:         /* HBR */
        sinkCaps->ratesSupported[DP_LINK_RATE_2_70] = true;
        break;
    case 16200U:         /* 3.24 Gb/s */
        sinkCaps->ratesSupported[DP_LINK_RATE_3_24] = true;
        break;
    case 21600U:         /* 4.32 Gb/s */
        sinkCaps->ratesSupported[DP_LINK_RATE_4_32] = true;
        break;
    case 27000U:         /* HBR2 */
        sinkCaps->ratesSupported[DP_LINK_RATE_5_40] = true;
        break;
    case 40500U:         /* HBR3 */
        sinkCaps->ratesSupported[DP_LINK_RATE_8_10] = true;
        break;
    default:
        sinkCaps->linkRateSupported = true;
        break;
    }
}

/**
 * Based on values in DPCD registers 10h - 1Fh, determine which rates are
 * supported, and what are their indices.
 * @param[in] pD pointer to driver's private data object
 * @param[in] dpcdRegs pointer to DPCD registers
 */
static void fillEdpRates(DP_PrivateData* pD, const uint8_t* dpcdRegs)
{
    uint8_t i;
    uint8_t index;
    DP_SinkDeviceCapabilities* sinkCaps = &(pD->sinkCaps);

    for (i = 0U; i < 8U; i++) {
        /* Store value from 2 DPCD registers in a 16-bit one, to get its index later. */
        index = 2U * i;
        sinkCaps->edpRatesRegisters[i] = ((uint16_t)dpcdRegs[index + 1U] << 8) |
                                         (uint16_t)dpcdRegs[index];

        /* Stop on entry equal to 0. */
        if (0U ==  sinkCaps->edpRatesRegisters[i]) {
            break;
        }

        sinkCaps->edpRateCount++;
        setRateSupport(sinkCaps, sinkCaps->edpRatesRegisters[i]);
    }
}

/*
 * Returns maximum lane count for connection
 * @param[in] pD pointer to driver's private data object
 * @return lane count value
 */
static uint8_t getMaxLaneCount(const DP_PrivateData* pD)
{
    uint8_t laneCount = pD->sinkCaps.laneCount;

    if (pD->sourceCaps.laneCount < laneCount) {
        laneCount = pD->sourceCaps.laneCount;
    }

    return laneCount;
}

/*
 * Returns Spread-Spectrum clock support
 * @param[in] pD pointer to driver's private data object
 * @return true if SSC is supported
 * @return false if SSC is not supported
 */
static bool getSscSupported(const DP_PrivateData* pD)
{
    bool ssc = false;

    /* test if SSC is supported by both sides */
    if ((pD->sourceCaps.ssc) && (pD->sinkCaps.ssc)) {
        ssc = true;
    }
    return ssc;
}

/**
 * Find out, which Training Pattern Sequence (TPS) is the highest one, supported
 * by both Source and Sink device.
 * @param[in] pD pointer to driver's private data object
 * @return highest Common TPS value
 */
static uint8_t getHighestCommonTps(const DP_PrivateData* pD)
{
    uint8_t highest;

    /* test if TPS for source are sink are same */
    if (pD->sourceCaps.tps4 && pD->sinkCaps.tps4) {
        highest = 4U;
    } else if (pD->sourceCaps.tps3 && pD->sinkCaps.tps3) {
        highest = 3U;
    } else {
        highest = 2U;
    }

    return highest;
}

/*
 * Check support for fast training
 * @param[in] pD pointer to driver's private data object
 * @return true if fast training is supported
 * @return false if fast training is supported
 */
static bool getFastTrainingSupported(const DP_PrivateData* pD)
{
    bool fastLinkTrainingSupport = false;

    if ((pD->sourceCaps.fastLinkTraining) && (pD->sinkCaps.fastLinkTraining)) {
        fastLinkTrainingSupport = true;
    }

    return fastLinkTrainingSupport;
}

/*
 * Check if sink capabilities are correct
 * @param[in] dpcpCaps pointer to DPCD params
 * @return EOK if success
 * @return ECANCELLED if DPCD params are incorrect
 */
static uint32_t verifySinkCapabilities(uint8_t dpcdCaps[16])
{
    uint32_t retVal = CDN_EOK;
    const uint8_t laneCount = dpcdCaps[DP_MAX_LANE_COUNT_OFF] & DP_MAX_LANE_COUNT_MASK;

    /* test if MAX_LINK_RATE is correct */
    if ((dpcdCaps[DP_MAX_LINK_RATE_OFF] > 0x00U) && (dpcdCaps[DP_MAX_LINK_RATE_OFF] < 0x06U)) {
        /* Unless this rate selection method is not supported, max link rate */
        /* must be at least 1.62 Gb/s (RBR), represented by value 0x06. */
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Value in sink's MAX_LINK_RATE DPCD register is invalid.\n");
        retVal = CDN_ECANCELED;
    }

    /* test if MAX_LANE_COUNT is correct */
    if ((laneCount != 1U) && (laneCount != 2U) && (laneCount != 4U)) {
        /* incorrect lane count */
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Value in sink's MAX_LANE_COUNT DPCD register is invalid.\n");
        retVal = CDN_ECANCELED;
    }

    /* test if 8b/10b encoding is supported */
    if ((dpcdCaps[DP_ML_CODING_OFF] & DP_CAPABLE_8B10B_MASK) == 0U) {
        DbgMsg(DBG_GEN_MSG, DBG_WARN, "Sink doesn't indicate support for mandatory 8b/10b encoding.\n");
    }

    /* test if training interval is correct */
    if ((dpcdCaps[DP_TRAINING_AUX_RD_INTERVAL_OFF] & DP_TRAINING_AUX_RD_INTERVAL_MASK) > 0x04U) {
        DbgMsg(DBG_GEN_MSG, DBG_WARN, "Value of interval for reading training status is outside known range. "
                                      "Driver will attempt to predict correct behaviour.\n");
    }

    return retVal;
}

/*
 * Set maximium link rate for sink device
 * @param[in] pD pointer to driver's private data object
 * @param[in] maxLinkBw maximum link bandwidth
 */
static void setSinkMaxLinkRate(DP_SinkDeviceCapabilities* sinkCaps, const uint8_t maxLinkBw) {

    bool* linkBwSupported = &(sinkCaps->linkBwSupported);
    DP_LinkRate* maxLinkRate = &(sinkCaps->maxLinkRate);

    *linkBwSupported = true;

    /* set max link rate according to max link bandwidth */
    switch (maxLinkBw) {
    case 0x00:
        *linkBwSupported = false;
        break;
    case 0x06:
        *maxLinkRate = DP_LINK_RATE_1_62;
        break;
    case 0x0A:
        *maxLinkRate = DP_LINK_RATE_2_70;
        break;
    case 0x14:
        *maxLinkRate = DP_LINK_RATE_5_40;
        break;
    default:
        /* Future versions of DP may define higher link rates. In such case, */
        /* use 8.1 Gb/s */
        *maxLinkRate = DP_LINK_RATE_8_10;
        break;
    }
}

/*
 * Set support for "regular" DP rates
 * @params[in] sinkCaps pointer to sink capabilities object
 */
static void setRegularDpRateSupport(DP_SinkDeviceCapabilities* sinkCaps) {

    DP_LinkRate linkRate;
    uint8_t supportedRatesNumber;
    uint8_t i;
    DP_LinkRate regularLinkRates[4] = {DP_LINK_RATE_1_62,
                                       DP_LINK_RATE_2_70,
                                       DP_LINK_RATE_5_40,
                                       DP_LINK_RATE_8_10};

    /* set number of supported links */
    switch (sinkCaps->maxLinkRate)
    {
    case DP_LINK_RATE_8_10:
        supportedRatesNumber = 4U;
        break;

    case DP_LINK_RATE_5_40:
        supportedRatesNumber = 3U;
        break;

    case DP_LINK_RATE_2_70:
        supportedRatesNumber = 2U;
        break;

    case DP_LINK_RATE_1_62:
        supportedRatesNumber = 1U;
        break;

    default:
        supportedRatesNumber = 0U;
        break;
    }

    /* set 'ratesSupported' flags */
    for (i = 0U; i < supportedRatesNumber; i++) {
        linkRate = regularLinkRates[i];
        sinkCaps->ratesSupported[linkRate] = true;
    }
}

/*
 * Write into sink capabilities object data from DPCD
 * @param[in] sinkCaps pointer to sink capabilities object
 * @param[in] transfer pointer to DPCD transfer object
 */
static void fillSinkCapabilities(DP_SinkDeviceCapabilities* sinkCaps, const DP_DpcdTransfer* transfer) {

    sinkCaps->TrainingInterval = calcTrainingInterval(transfer->buff[DP_TRAINING_AUX_RD_INTERVAL_OFF]);

    /* set max link rate */
    uint8_t maxLinkBw = transfer->buff[DP_MAX_LINK_RATE_OFF];
    setSinkMaxLinkRate(sinkCaps, maxLinkBw);

    sinkCaps->laneCount = transfer->buff[DP_MAX_LANE_COUNT_OFF] & DP_MAX_LANE_COUNT_MASK;

    /* set SSC supported */
    sinkCaps->ssc = (0U != (transfer->buff[DP_MAX_DOWNSPREAD_OFF] & DP_MAX_DOWNSPREAD_MASK));

    /* set TPS support */
    sinkCaps->tps3 = (0U != (transfer->buff[DP_MAX_LANE_COUNT_OFF] & DP_TPS3_SUPPORTED_MASK));
    sinkCaps->tps4 = (0U != (transfer->buff[DP_MAX_DOWNSPREAD_OFF] & DP_TPS4_SUPPORTED_MASK));

    sinkCaps->fastLinkTraining = (0U != (transfer->buff[DP_MAX_DOWNSPREAD_OFF] & DP_NO_AUX_TRAINING_MASK));
    sinkCaps->enhanced = (0U != (transfer->buff[DP_MAX_LANE_COUNT_OFF] & DP_ENHANCED_FRAME_CAP_MASK));
    sinkCaps->assr = (0U != (transfer->buff[DP_EDP_CONF_CAP_OFF] & DP_ASSR_CAP_MASK));

    setRegularDpRateSupport(sinkCaps);

}
/*
 * Set parameters of DPCD transfer
 * @param[in] transfer pointer to DPCD transfer object
 * @param[in] address value of DPCD register address
 * @param[in] size value of transfer size
 * @param[in] buff pointer to data buffer
 */
static void setDpcdTransfer(DP_DpcdTransfer* transfer, uint32_t address, uint8_t size, uint8_t* buffer) {
    transfer->addr = address;
    transfer->size = size;
    transfer->buff = buffer;
}

/**
 * Performs a series of DPCD register reads, used to determine and
 * store capabilities of sink device.
 * @param[in] pD pointer to driver's private data object
 */
uint32_t readSinkCapabilities(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;
    DP_DpcdTransfer transfer;
    uint8_t dpcdRespBuff[16];

    DP_SinkDeviceCapabilities* sinkCaps = &(pD->sinkCaps);

    clearSinkCapabilities(sinkCaps);

    /* Read capabilities from DPCD registers starting at address 0h. */
    setDpcdTransfer(&transfer, 0U, 16U, dpcdRespBuff);
    retVal = DP_ReadDpcd(pD, &transfer);

    if (CDN_EOK == retVal) {
        /* Test for presence of Extended Receiver Capability at DPCD 2200h. */
        /* If present, read it, and use for further capability reading. */
        if (0U != (transfer.buff[DP_TRAINING_AUX_RD_INTERVAL_OFF] & DP_EXT_RECV_CAP_FIELD_PRESENT_MASK)) {
            setDpcdTransfer(&transfer, 0x2200U, 16U, dpcdRespBuff);
            retVal = DP_ReadDpcd(pD, &transfer);
        }
    }

    if (CDN_EOK == retVal) {
        retVal = verifySinkCapabilities(transfer.buff);
    }

    if (CDN_EOK == retVal) {
        fillSinkCapabilities(sinkCaps, &transfer);

        /* eDP 1.4 requires fast link training support from source device. */
        if (sinkCaps->fastLinkTraining) {
            /* Read supported link rates from DPCD 10h - 1Fh */
            setDpcdTransfer(&transfer, 0x10U, 16U, dpcdRespBuff);
            retVal = DP_ReadDpcd(pD, &transfer);
        }
    }

    if (CDN_EOK == retVal) {
        if (sinkCaps->fastLinkTraining) {
            fillEdpRates(pD, transfer.buff);
        }
        pD->sinkCapsStored = 1U;
    }

    return retVal;
}

/**
 * Return 1, if given link rate is specific to eDP. 0 otherwise.
 * @param[in] rate rate value
 * @return 1 if rate is specific to eDP
 * @return 0 if not
 */
static uint8_t isEdpRate(DP_LinkRate rate)
{
    uint8_t result;

    /*  check if rate is specific to eDP */
    switch (rate) {
    case DP_LINK_RATE_2_16:
    case DP_LINK_RATE_2_43:
    case DP_LINK_RATE_3_24:
    case DP_LINK_RATE_4_32:
        result = 1U;
        break;
    default:
        result = 0U;
        break;
    }

    return result;
}

/*
 * Return maximum available link rate
 * @param[in] sourceRate maximum link rate for source
 * @param[in] sinkRate maximum link rate for sink
 * @return maximum available link
 */
static DP_LinkRate getMaxCommonRate(DP_LinkRate sourceRate, DP_LinkRate sinkRate)
{
    DP_LinkRate rate = sourceRate;

    if (sinkRate < sourceRate) {
        rate = sinkRate;
    }

    return rate;
}

/*
 * Test if link rates are supported
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if are supported
 * @return CDN_ENOTSUP if not
 */
static uint32_t checkLinkRateSupport(const DP_PrivateData* pD)
{
    uint32_t retVal = CDN_ENOTSUP;

    if ((!pD->sourceCaps.fastLinkTraining) &&
        (0U != isEdpRate(pD->sourceCaps.maxLinkRate))) {
        /* eDP link rate requested without supporting fast link training. */
        /* Combination is not supported by driver. */
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Please enable fast training to use eDP link rates.\n");
    } else if ((!pD->sinkCaps.linkBwSupported) && (!pD->sinkCaps.linkRateSupported)) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Sink doesn't seem to support any form of rate selection.\n");
    } else if ((!pD->sinkCaps.fastLinkTraining) && (pD->sinkCaps.linkRateSupported)) {
        /* If sink declares support for flexible link rates, it must adhere to eDP v1.4. */
        /* Fast link training is required for sink device since eDP v1.2 */
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Sink violates eDP standard, by not supporting fast training\n");
    } else {
        retVal = CDN_EOK;
    }

    return retVal;
}
/**
 * Based on source device capabilities set by client, and sink device
 * capabilities read from DPCD, determine link rate to use, which also
 * determines, link rate selection method in DPCD.
 * @param[in] pD pointer to DP private data object
 * @param[out] rate value of rate
 * @return CDN_EOK, when acceptable link rate was found
 * @return CDN_ENOTSUP, when acceptable link rate was not found.
 */
static uint32_t getMaxLinkRate(DP_PrivateData* pD, DP_LinkRate* rate)
{
    uint32_t retVal;
    DP_SourceDeviceCapabilities* sourceCaps = &(pD->sourceCaps);
    DP_SinkDeviceCapabilities* sinkCaps = &(pD->sinkCaps);

    retVal = checkLinkRateSupport(pD);

    if (CDN_EOK == retVal) {
        /* Perform fast link training, if supported. */
        if ((sourceCaps->fastLinkTraining) && (sinkCaps->fastLinkTraining)) {
            /* Test if source maximum link rate is supported. If not, check if sink supports
             * link rate.
             */
            if (sinkCaps->ratesSupported[sourceCaps->maxLinkRate]) {
                *rate = sourceCaps->maxLinkRate;
                retVal = CDN_EOK;
            } else if (!(sinkCaps->linkRateSupported)) {
                *rate = getMaxCommonRate(sourceCaps->maxLinkRate, sinkCaps->maxLinkRate);
                retVal = CDN_EOK;
            } else {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Desired rate for fast training is not supported by sink.\n");
                retVal = CDN_ENOTSUP;
            }
        } else if (sinkCaps->linkBwSupported) {
            /* consider as regular DP device and find highest common rate, without fast training */
            *rate = getMaxCommonRate(sourceCaps->maxLinkRate, sinkCaps->maxLinkRate);
            retVal = CDN_EOK;
        } else {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Could not determine allowed link rate.\n");
            retVal = CDN_ENOTSUP;
        }
    }

    return retVal;
}

/**
 * Step link rate down, according to regular DP 1.4.
 * @param[in] linkRate current link rate value
 * @return reduced link rate value
 */
static DP_LinkRate reduceLinkRate(DP_LinkRate linkRate)
{
    DP_LinkRate reduced;

    /* Reduce link rate by one level */
    switch (linkRate) {
    case DP_LINK_RATE_8_10:
        reduced = DP_LINK_RATE_5_40;
        break;
    case DP_LINK_RATE_5_40:
        reduced = DP_LINK_RATE_2_70;
        break;
    case DP_LINK_RATE_2_70:
        reduced = DP_LINK_RATE_1_62;
        break;
    default:
        /* cannot reduce or link rate is unrecognized */
        reduced = DP_LINK_RATE_1_62;
        break;
    }

    return reduced;
}

/**
 * Check, if lane count reduction during CR phase of link training is allowed as
 * a fallback, when CR phase failed at RBR. If yes, calculate target lane count.
 * Check, if CR phase passed only for lowest-numbered lanes.
 * @param[in] pD pointer to DP private data object
 * @param[in] status array with current status of training
 * @param[out] target count reduced lane value
 * @return 0, if no lane passed CR phase, or if there is a lane that passed CR,
 * while any of lower-numbered ones did not.
 * @return 1 otherwise and sets variable pointed by targetCount.
 */
static uint8_t checkLaneCountFallback(const DP_PrivateData*     pD,
                                      const TrainingPhaseStatus status[4],
                                      uint8_t*                  targetCount)
{
    uint8_t result = 1U; /* fallback is allowed */
    uint8_t reduced = 0U;
    uint8_t i;
    uint8_t zeroEncountered = 0U;

    /* If only one lane was used, or even first lane didn't pass, return 0. */
    if ((1U >= pD->linkState.laneCount) || (LT_PHASE_FAIL == status[0])) {
        result = 0U;
    }

    if (1U == result) {
        for (i = 0U; i < pD->linkState.laneCount; i++) {
            if (LT_PHASE_PASS == status[i]) {
                if (0U == zeroEncountered) {
                    reduced++;
                } else {
                    result = 0U;
                    break;
                }
            } else {
                zeroEncountered = 1U;
            }
        }
    }

    if (1U == result) {
        /* if 3 first lanes passed, fall-back to 2. */
        if (3U == reduced) {
            reduced = 2U;
        }
        *targetCount = reduced;
    }

    return result;
}

/**
 * Step lane count down, according to regular DP 1.4.
 * @param[in] laneCount current lane count
 * @return reduced lane count
 */
static uint8_t reduceLaneCount(uint8_t laneCount)
{
    uint8_t reduced;

    /* Reduce lane count by one level */
    switch (laneCount) {
    case 4U:
        reduced = 2U;
        break;
    case 2U:
        reduced = 1U;
        break;
    default:
        reduced = 1U;
        break;
    }
    return reduced;
}

/*
 * Reset configuration of PHY
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot reset
 */
static uint32_t resetDigitalPhyConfig(DP_PrivateData* pD)
{
    uint32_t regTmp;
    DP_RegisterTransfer regTransfer = {0U};

    /* stop transmitting TPS */
    regTmp = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_ENABLE, 0U, 0U) |
             CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_TYPE, 0U, 1U) |
             CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_SCRAMBLER_BYPASS, 0U, pD->sourceCaps.scramblerDisable ? 1U : 0U) |
             CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_ENCODER_BYPASS, 0U, 0U) |
             COMMON_DPHY_CONFIG;

    regTransfer.val  = regTmp;
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_CONFIG_REG_p);

    return DP_WriteRegister(pD, &regTransfer);
}

/**
 * Resets Voltage Swing and Pre-Emphaisis levels for lanes to '0', or to values
 * requested in source capabilities.
 * @param[in] pD pointer to DP private data object
 * @param[in] laneCount count of lanes
 */
static void resetLaneSettings(DP_PrivateData* pD, uint8_t laneCount)
{
    uint8_t i;

    /* reset lane setting for each lane */
    for (i = 0U; i < laneCount; i++) {

        if (!pD->sourceCaps.forceVoltageSwing) {
            pD->linkState.voltageSwing[i] = 0U;
        } else {
            pD->linkState.voltageSwing[i] = pD->sourceCaps.maxVoltageSwing;
        }

        if (!pD->sourceCaps.forcePreemphasis) {
            pD->linkState.preEmphasis[i] = 0U;
        } else {
            pD->linkState.preEmphasis[i] = pD->sourceCaps.maxPreemphasis;
        }
    }
}

/**
 * Set Voltage Swing and Pre-emphasis levels as current ones, where applicable.
 * @param[in] pD pointer to DP private data object
 * @param[in] requestedLevels pointer to link state object
 */
static void adjustLaneSettings(DP_PrivateData* pD, const DP_LinkState* requestedLevels, uint8_t laneCount)
{
    uint8_t i;

    /* adjust lane setting for each lane */
    for (i = 0U; i < laneCount; i++) {

        if (!pD->sourceCaps.forceVoltageSwing) {
            pD->linkState.voltageSwing[i] = requestedLevels->voltageSwing[i];
        }

        if (!pD->sourceCaps.forcePreemphasis) {
            pD->linkState.preEmphasis[i] = requestedLevels->preEmphasis[i];
        }
    }
}

/**
 * Apply Voltage Swing and Pre-Emphasis levels for lanes.
 * @param[in] pD pointer to DP private data object
 * @param[in] laneCount count of lanes
 */
static uint32_t applyLaneSettings(DP_PrivateData* pD, uint8_t laneCount)
{
    uint8_t i;
    uint32_t retVal = CDN_EOK;
    DP_SD0801_LinkState sdState;
    toSdLinkState(&(pD->linkState), &sdState);

    for (i = 0U; i < laneCount; i++) {
        if (CDN_EOK == retVal) {
            /* Configure voltage swing and pre-emphasis for all enabled lanes. */
            retVal = DP_SD0801_ConfigLane(pD->phyPd, i, &sdState);
        }
    }

    return retVal;
}

/**
 * Check, if voltage swing / pre-emphasis levels requested by sink device differ
 * from ones currently set.
 * @param[in] pD pointer to DP private data object
 * @param[in] requestedLevels pointer to link state object
 * @param[in] laneCount count of lanes
 * @return 1 if there are differences
 * @return 0 if no differences
 */
static uint8_t laneSetingsDiffer(const DP_PrivateData* pD,
                                 const DP_LinkState*   requestedLevels,
                                 uint8_t               laneCount)
{
    uint8_t i;
    uint8_t retVal = 0U;

    /* check for each available lane */
    for (i = 0U; i < laneCount; i++) {
        if (pD->linkState.voltageSwing[i] != requestedLevels->voltageSwing[i]) {
            retVal = 1U;
        }
        if (pD->linkState.preEmphasis[i] != requestedLevels->preEmphasis[i]) {
            retVal = 1U;
        }
        if (1U == retVal) {
            break;
        }
    }

    return retVal;
}

/**
 * Checks, if there is a lane, that failed CR training while being set to
 * highest Voltage Swing + Pre-emphasis value.
 */
static uint8_t maxSwingCheck(const DP_PrivateData* pD, const TrainingStepStatus *status, uint8_t laneCount)
{
    uint8_t i;
    uint8_t retVal = 0U;

    for (i = 0U; i < laneCount; i++) {
        if (1U == status->crDone[i]) {
            continue; /* lane passed training */
        }
        if ((pD->linkState.voltageSwing[i] + pD->linkState.preEmphasis[i]) >= 3U) {
            /* at least one lane reached max allowed value. */
            retVal = 1U;
            break;
        }
    }
    return retVal;
}

/**
 * Check, if clock recovery for enabled lanes was reached or maintained.
 * @param[in] status pointer to training step status object
 * @param[in] laneCount number of lanes
 * @return 1 if clock recovery reached
 * @return 0 if clock recovery maintained
 */
static uint8_t trainingStepCrCheck(const TrainingStepStatus *status, uint8_t laneCount)
{
    uint8_t i;
    uint8_t retVal = 1U;

    /* check clock recovery for each lane */
    for (i = 0U; i < laneCount; i++) {
        if (1U != status->crDone[i]) {
            retVal = 0U;
            break;
        }
    }
    return retVal;
}

/**
 * Check, if channel equalization for enabled lanes was reached or maintained.
 * @param[in] status pointer to training step status object
 * @param[in] laneCount number of lanes
 * @return 1 if channel equalization reached
 * @return 0 if channel equalization maintained
 */
static uint8_t trainingStepEqCheck(const TrainingStepStatus *status, uint8_t laneCount)
{
    uint8_t i;
    uint8_t retVal = 1U;

    /* check channel equalization for each lane */
    for (i = 0U; i < laneCount; i++) {

        if (1U != status->crDone[i]) {
            retVal = 0U;
        } else if (1U != status->eqDone[i]) {
            retVal = 0U;
        } else if (1U != status->symbolLocked[i]) {
            retVal = 0U;
        } else {
            continue;
        }

        break;
    }

    /* check if interlane aligned */
    if (1U != status->interlaneAligned) {
        retVal = 0U;
    }

    return retVal;
}

/**
 * Check, if training for enabled lanes succeeded.
 * @param[in] status pointer to training step status object
 * @param[in] laneCount number of lanes
 * @return 1 if success
 * @return 0 if not success
 */
static uint8_t trainingPassCheck(const TrainingPhaseStatus *status, uint8_t laneCount)
{
    uint8_t i;
    uint8_t retVal = 1U;

    /* check for each lane */
    for (i = 0U; i < laneCount; i++) {
        if (LT_PHASE_PASS != status[i]) {
            retVal = 0U;
            break;
        }
    }

    return retVal;
}

/*
 * Write training pattern to DPCD
 * @param[in] pD pointer to DP private data object
 * @param[in] pattern value to write
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot write
 */
static uint32_t writePatternToDpcd(DP_PrivateData* pD, uint8_t pattern)
{
    DP_DpcdTransfer transfer;
    uint8_t dpcdWrBuff[4];

    setDpcdTransfer(&transfer, 0x102U, 1U, dpcdWrBuff);

    if (0U == pattern) {
        transfer.buff[0] = 0U;
    } else if (4U == pattern) {
        transfer.buff[0] = 0x07;
    } else {
        /* disable scrambling for patterns 1-3 */
        transfer.buff[0] = pattern | DP_SCRAMBLING_DISABLE_MASK;
    }

    return DP_WriteDpcd(pD, &transfer);
}

/**
 * Selects Training Pattern Sequence to be transmitted by source
 * device and informs sink about it using DPCD write.
 * @param[in] pattern Training Pattern to be used (1 - 4). Value of '0' disables transmission
 *    of Training Pattern.
 * @param[in] writeDpcd Whether or not to issue DPCD write transaction about pattern set.
 * @return CDN_EOK if paterrn was set correctly
 * @return CDN_EINVAL if pattern is out of range or cannot write pattern to DPCD
 */
static uint32_t setTrainingPattern(DP_PrivateData* pD, uint8_t pattern, uint8_t writeDpcd)
{
    uint32_t retVal = CDN_EOK;
    uint32_t regTmp;
    DP_RegisterTransfer regTransfer = {0U};

    /* test if pattern is out of range */
    if (4U < pattern) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        if (0U != pattern) {
            /* set training configuration */
            regTmp = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_ENABLE, 0U, 1U) |
                     CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_TYPE, 0U, pattern) |
                     CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_SCRAMBLER_BYPASS, 0U, (pattern == 4U) ? 0U : 1U) |
                     CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_ENCODER_BYPASS, 0U, 0U) |
                     COMMON_DPHY_CONFIG;

            regTransfer.val  = regTmp;
            regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_CONFIG_REG_p);
            retVal = DP_WriteRegister(pD, &regTransfer);
        } else {
            /* Stop transmitting TPS */
            retVal = resetDigitalPhyConfig(pD);
        }
    }

    if (CDN_EOK == retVal) {
        if (0U != writeDpcd) {
            /* write pattern to DPCD */
            retVal = writePatternToDpcd(pD, pattern);
        }
    }

    return retVal;
}

/**
 * Configure value for 101h DPCD register.
 * @param[in] pD pointer to DP private data object
 * @param[in] pointer to buffer with data
 */
static void fillDpcd101h(const DP_PrivateData* pD, uint8_t* dpcdBuff)
{
    dpcdBuff[0] = (pD->linkState.laneCount) & DP_LANE_COUNT_SET_MASK;
    if (pD->sinkCaps.enhanced) {
        dpcdBuff[0] |= DP_ENHANCED_FRAME_EN_MASK;
    }
}

/**
 * Configure and transmit values for 100h and 101h DPCD registers:
 * link rate, lane count and enhanced mode.
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot write data to DPCD
 */
static uint32_t sendDpcd100h(DP_PrivateData* pD)
{
    uint8_t dpcdBuff[2];
    DP_DpcdTransfer transfer;

    /* Set buffer according to link rate */
    switch (pD->linkState.linkRate) {
    case DP_LINK_RATE_1_62:
        dpcdBuff[0] = 0x06U;
        break;
    case DP_LINK_RATE_2_70:
        dpcdBuff[0] = 0x0AU;
        break;
    case DP_LINK_RATE_5_40:
        dpcdBuff[0] = 0x14U;
        break;
    case DP_LINK_RATE_8_10:
        dpcdBuff[0] = 0x1EU;
        break;
    default:
        dpcdBuff[0] = 0x00U;
        break;
    }

    /* fill buffer with data */
    fillDpcd101h(pD, &(dpcdBuff[1]));
    setDpcdTransfer(&transfer, 0x100U, 2U, dpcdBuff);

    return DP_WriteDpcd(pD, &transfer);
}

/**
 * configure and transmit value for 101h DPCD register:
 * lane count and enhanced mode.
 */
static uint32_t sendDpcd101h(DP_PrivateData* pD)
{
    uint8_t dpcdBuff[1];
    DP_DpcdTransfer transfer;

    fillDpcd101h(pD, &(dpcdBuff[0]));

    setDpcdTransfer(&transfer, 0x101U, 1U, dpcdBuff);

    return DP_WriteDpcd(pD, &transfer);
}

/*
 * Set DPCD rate according to link rate
 * @param[in] rate value of link rate
 * @param[in] rateDpcd pointer to DPCD rate
 * @return CDN_EOK if success
 * @return CDN_EINVAL if rate is not supported
 */
static uint32_t getRateDpcd(const DP_LinkRate rate, uint32_t* rateDpcd) {

    /* return CDN_EOK if link rate is supported */
    uint32_t retVal = CDN_EOK;

    /* designate rateDpcd depends to rate */
    switch (rate)
    {
    /* each link rate have own rateDpcd value */
    case DP_LINK_RATE_1_62:
        *rateDpcd = 8100U;
        break;
    case DP_LINK_RATE_2_16:
        *rateDpcd = 10800U;
        break;
    case DP_LINK_RATE_2_43:
        *rateDpcd = 12150U;
        break;
    case DP_LINK_RATE_2_70:
        *rateDpcd = 13500U;
        break;
    case DP_LINK_RATE_3_24:
        *rateDpcd = 16200U;
        break;
    case DP_LINK_RATE_4_32:
        *rateDpcd = 21600U;
        break;
    case DP_LINK_RATE_5_40:
        *rateDpcd = 27000U;
        break;
    case DP_LINK_RATE_8_10:
        *rateDpcd = 40500U;
        break;
    default:
        /* link rate is unsupported */
        retVal = CDN_EINVAL;
        break;
    }

    return retVal;
}

/**
 * Find, which 2-byte entry (taken from DPCD registers 0x10 - 0x1F) corresponds
 * to desired link rate. Return value of 'CDN_ENOTSUP' means, that rate was not found.
 * @param[in] pD pointer to DP private data object
 * @param[in] rate rate value
 * @param[out] idx index of link rate
 * @return CDN_EOK if rate was found
 * @retrun CDN_EINVAL if not found
 */
static uint32_t findLinkRateIndex(const DP_PrivateData* pD,
                                  const DP_LinkRate     rate,
                                  uint8_t*              idx)
{
    uint32_t rateDpcd;
    uint32_t retVal = getRateDpcd(rate, &rateDpcd);
    uint8_t i;

    if (CDN_EOK == retVal) {
        retVal = CDN_ENOTSUP;
        /* test if rate is supported by device */
        for (i = 0U; i < pD->sinkCaps.edpRateCount; i++) {
            if (rateDpcd == pD->sinkCaps.edpRatesRegisters[i]) {
                retVal = CDN_EOK;
                *idx = i;
            }
        }
    }

    return retVal;
}

/**
 * Configure and transmit value for 115h DPCD register:
 * link rate, when using eDP-specific LINK_RATE_SET rate selection approach.
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot find link rate or write data to DPCD
 */
static uint32_t sendDpcd115h(DP_PrivateData* pD)
{
    uint32_t retVal;
    uint8_t dpcdBuff[1];
    DP_DpcdTransfer transfer;

    /* find index of link rate in eDP register */
    retVal = findLinkRateIndex(pD, pD->linkState.linkRate, &(dpcdBuff[0]));

    if (CDN_EOK == retVal) {
        /* send data do DPCD register */
        setDpcdTransfer(&transfer, 0x115U, 1U, dpcdBuff);
        retVal = DP_WriteDpcd(pD, &transfer);
    }

    return retVal;
}

/*
 * Get value of downspread
 * @param[in] pD pointer to PDP private data object
 * @return downspread value
 */
static uint8_t configureDownspread(const DP_PrivateData* pD) {

    uint8_t retVal;

    if (getSscSupported(pD)) {
        retVal = DP_SPREAD_AMP_MASK;
    } else {
        retVal = 0U;
    }

    return retVal;
}

/**
 * Performs initial Link Training operations, common to full and fast
 * training. May be called once Source capabilities were set, and sink
 * capabilities were read.
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 */
static uint32_t initTraining(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;
    uint8_t dpcdWrBuff[16];
    DP_RegisterTransfer regTransfer = {0U};
    DP_DpcdTransfer transfer;

    /* Configure downspread and 8b/10 encoding */
    setDpcdTransfer(&transfer, 0x107U, 2U, dpcdWrBuff);
    transfer.buff[0] = configureDownspread(pD);
    transfer.buff[1] = DP_SET_8B_10B_MASK;
    retVal = DP_WriteDpcd(pD, &transfer);

    if (CDN_EOK == retVal) {
        /* Enable enhanced framing mode, if supported by Sink. */
        if (pD->sinkCaps.enhanced) {
            regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DPTX_ENHNCD_P, DPTX_ENHANCED_MODE, 0U, 1U);
        } else {
            regTransfer.val = 0U;
        }

        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_ENHNCD_p);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

/*
 * Enable software reset
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 */
static uint32_t resetRequest(DP_PrivateData* pD)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t retVal;

    /* set reset flag */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_SW_RESET_p);
    regTransfer.val = 1U;
    retVal = DP_WriteRegister(pD, &regTransfer);

    if (CDN_EOK == retVal) {
        /* clean reset flag */
        regTransfer.val = 0U;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}
/**
 * Performs operations done after Link Training.
 * @param[in] pD pointer to DP private data object
 * @param[in] writeDpcd whether or not to issue DPCD write transaction about
 * pattern set.
 * @return CDN_EOK if success
 */
static uint32_t cleanupTraining(DP_PrivateData* pD, uint8_t writeDpcd)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    /* request reset. */
    retVal = resetRequest(pD);

    if (CDN_EOK == retVal) {
        retVal = setTrainingPattern(pD, 0U, writeDpcd);
    }

    /* read DP_FRAMER_GLOBAL_CONFIG */
    if (CDN_EOK == retVal) {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    /* set lane count and WR_VHSYNC_FALL */
    if (CDN_EOK == retVal) {
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P, NUM_LANES, regTransfer.val, (uint32_t)pD->linkState.laneCount - 1U);
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P, WR_VHSYNC_FALL, regTransfer.val, 1U);
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

/**
 * Based on values read from DPCD registers, fills structure containing result
 * of Link Training's phase.
 * @param[in] dpcdBuff pointer to DPCD data buffer
 * @oaram[out] status pointer to training status object
 */
static void parseLanesStatus(const uint8_t* dpcdBuff, TrainingStepStatus* status)
{
    uint8_t i;
    uint8_t tmp;

    for (i = 0U; i < 4U; i++) {
        tmp = dpcdBuff[i / 2U]; /* 0 for lanes 0-1, 1 for lanes 2-3 */

        if (0U != (i & 0x01U)) { /* odd-numbered lane - values on most significant nibble */
            tmp = tmp >> DP_LANE_STATUS_ODD_LANE_SHIFT;
        }

        if (0U != (tmp & DP_LANE_CR_DONE_MASK))
        {
            status->crDone[i] = 1U;
        }
        if (0U != (tmp & DP_LANE_EQ_DONE_MASK)) {
            status->eqDone[i] = 1U;
        }

        if (0U != (tmp & DP_LANE_SYMBOL_LOCKED_MASK)) {
            status->symbolLocked[i] = 1U;
        }
    }

    if (0U != (dpcdBuff[2] & DP_INTERLANE_ALIGN_DONE_MASK)) {
        status->interlaneAligned = 1U;
    }
}

/**
 * Based on values read from DPCD registers, fills structure containing result
 * of Link Training's phase.
 * @param[in] pD pointer to DP private data object
 * @param[in] dpcdBuffer pointer to DPCD buffer
 * @param[out] status pointer to training status object
 * @param[out] requestedAdjust pointer to link state object
 */
static void parseTrainingResult(const DP_PrivateData* pD,
                                const uint8_t*        dpcdBuff,
                                TrainingStepStatus*   status,
                                DP_LinkState*         requestedAdjust)
{
    uint8_t i;
    uint8_t tmp;

    clearTrainingResult(status);

    parseLanesStatus(dpcdBuff, status);

    for (i = 0U; i < 4U; i++)
    {
        tmp = dpcdBuff[(i / 2U) + 4U]; /* 4 for lanes 0-1, 5 for lanes 2-3 */
        if (0U != (i & 0x01U)) { /* odd-numbered lane */
            tmp = tmp >> DP_LANE_STATUS_ODD_LANE_SHIFT;
        }

        requestedAdjust->voltageSwing[i] = tmp & DP_VOLTAGE_SWING_ADJ_MASK;
        requestedAdjust->preEmphasis[i] = (tmp & DP_PREEEMPHASIS_ADJ_MASK) >> DP_PREEEMPHASIS_ADJ_SHIFT;

        /* verify requested lane count and pre-emphasis, step-down if required. */
        if (requestedAdjust->voltageSwing[i] > pD->sourceCaps.maxVoltageSwing) {
            requestedAdjust->voltageSwing[i] = pD->sourceCaps.maxVoltageSwing;
        }
        if (requestedAdjust->preEmphasis[i] > pD->sourceCaps.maxPreemphasis) {
            requestedAdjust->preEmphasis[i] = pD->sourceCaps.maxPreemphasis;
        }

        if ((requestedAdjust->voltageSwing[i] + requestedAdjust->preEmphasis[i]) > 3U) {
            /* voltage swing level and pre-emphasis level combination is not allowed: */
            /* leaving pre-emphasis as-is, and adjusting voltage swing. */
            requestedAdjust->voltageSwing[i] = 3U - requestedAdjust->preEmphasis[i];
        }
    }
}

/**
 * Based on current Voltage Swing and Pre-emphasis levels, construct values to
 * write to DPCD registers (starting from address 103h) by uCPU.
 * @param[in,out] pD pointer to DP private data object
 * @param[out] dpcdBuff pointer to DPCP buffer
 * @param[in] laneCount nubmer of lanes
 */

/* parasoft-begin-suppress MISRA2012-RULE-8_13_a "Pass pD parameter as const, DRV-3854" */
static void configureDpcdLaneSettings(DP_PrivateData* pD,
                                      uint8_t*        dpcdBuff,
                                      uint8_t         laneCount)
{
    uint8_t i;
    uint8_t* voltageSwing;
    uint8_t* preEmphasis;

    uint8_t maxVoltageSwing = pD->sourceCaps.maxVoltageSwing;
    uint8_t maxPreemphasis = pD->sourceCaps.maxPreemphasis;

    for (i = 0U; i < laneCount; i++)
    {
        voltageSwing = &pD->linkState.voltageSwing[i];
        preEmphasis = &pD->linkState.preEmphasis[i];

        /* first, validate max voltage swing and pre-emphasis and adjust, if needed. */
        if (*voltageSwing > maxVoltageSwing) {
            *voltageSwing = maxVoltageSwing;
        }

        if (*preEmphasis > maxPreemphasis) {
            *preEmphasis = maxPreemphasis;
        }

        /* then, prepare data to be written to DPCD as a part of LT_ADJUST command. */
        dpcdBuff[i] = *voltageSwing & DP_VOLTAGE_SWING_MASK;

        if (*voltageSwing == maxVoltageSwing) {
            dpcdBuff[i] |= DP_MAX_SWING_REACHED_MASK;
        }

        dpcdBuff[i] |= (*preEmphasis << DP_PREEMPHASIS_SET_SHIFT) & DP_PREEMPHASIS_SET_MASK;
        if (*preEmphasis == maxPreemphasis) {
            dpcdBuff[i] |= DP_MAX_PREEMPHASIS_REACHED_MASK;
        }
    }
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a */

/**
 * Send request for do link training step
 * @param[in,out] pD pointer to DP private data object
 * @param[in] laneCount Number of lanes used.
 * @param[in] delay Minimum amount of time (in microseconds) to wait before reading result.
 *    Shall comply with value read from TRAINING_AUX_RD_INTERVAL DPCD
 *    register (0x00E).
 *
 */
static void sendTrainingStepRequest(DP_PrivateData* pD,
                                    uint8_t         laneCount,
                                    uint16_t        delay)
{
    uint8_t dpcdBuff[4] = {0U};
    configureDpcdLaneSettings(pD, dpcdBuff, laneCount);

    /* send training request via Mailbox */
    messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_LT_ADJUST);
    messageWriteUint8(pD, laneCount);
    messageWriteUint16(pD, delay);
    messageWriteBuffer(pD, dpcdBuff, laneCount);
    messageFinish(pD);
    messageTransmit(pD, DP_BUS_TYPE_APB);

}

/**
 * Performs single step of Full Link Training: writing current levels
 * of Voltage Swing and Pre-Emphasis via DPCD, waiting for required
 * amount of time (while training pattern is being transmitted) and
 * reading back result of training step.
 * @param[in] pD pointer to DP private data object
 * @param[out] status Detailed result of Link Training step.
 * @param[out] requestedAdjust Voltage swing and pre-emphasis level adjustments, required by sink,
 *    per lane.
 * @return CDN_EOK if success
 * @return CDN_EIO if incorrect length of DPCD response was received
 */
static uint32_t getTrainingStepResponse(DP_PrivateData*     pD,
                                        TrainingStepStatus* status,
                                        DP_LinkState*       requestedAdjust)
{
    uint32_t result = CDN_EOK;
    uint8_t dpcdBuff[6];
    uint16_t readSize;
    uint32_t readAddr;

    messageReceive(pD, DP_BUS_TYPE_APB);

    /* check if header is equal expected */
    if (0U == messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_READ_DPCD)) {
        result = CDN_ENOEXEC;
    }

    /* read buffer size */
    if (result == CDN_EOK) {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint16(pD, &readSize);
        if (6U != readSize) {
            result = CDN_EIO;
        }
    }

    /* read address and buffer */
    if (result == CDN_EOK) {
        messageRead3Bytes(pD, &readAddr);
        messageReadBuffer(pD, dpcdBuff, readSize);
        parseTrainingResult(pD, dpcdBuff, status, requestedAdjust);
    }

    return result;
}

/**
 * Performs single step of Full Link Training: writing current levels
 * of Voltage Swing and Pre-Emphasis via DPCD, waiting for required
 * amount of time (while training pattern is being transmitted) and
 * reading back result of training step.
 * @param[in] pD pointer to DP private data object
 * @param[in] laneCount Number of lanes used.
 * @param[in] delay Minimum amount of time (in microseconds) to wait before reading result.
 *    Shall comply with value read from TRAINING_AUX_RD_INTERVAL DPCD
 *    register (0x00E).
 * @param[out] status Detailed result of Link Training step.
 * @param[out] requestedAdjust Voltage swing and pre-emphasis level adjustments, required by sink,
 *    per lane.
 * @return CDN_EOK if success
 * @return CDN_EIO if incorrect length of DPCD response was received
 */
static uint32_t trainingSingleStep(DP_PrivateData*     pD,
                                   uint8_t             laneCount,
                                   uint16_t            delay,
                                   TrainingStepStatus* status,
                                   DP_LinkState*       requestedAdjust)
{
    uint32_t retVal = CDN_EOK;

    /* check if parameters are valid */
    if ((NULL == pD) || (NULL == status) || (NULL == requestedAdjust)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        /* send request and check response */
        sendTrainingStepRequest(pD, laneCount, delay);
        retVal = getTrainingStepResponse(pD, status, requestedAdjust);
    }

    return retVal;
}

/*
 * Send via Mailbox request for delay
 * @param[in,out] pD pointer to DP private data object
 * @param[in] delay length of delay in microseconds
 */
static void sendWaitMessage(DP_PrivateData* pD, uint32_t delay)
{
    messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_WAIT);
    messageWriteUint32(pD, delay);
    messageFinish(pD);
    messageTransmit(pD, DP_BUS_TYPE_APB);
}

/**
 * Perform delay on Xtensa.
 * Sends GENERAL_WAIT Command via regular Mailbox. Response means, that at least
 * specified time has passed. Completion of the delay is signified by response.
 * @param[in,out] pD pointer to DP private data object
 * @param[in] delay Amount of microseconds, that Xtensa shall wait before sending response.
 * @return CDN_EOK if success
 * @return CDN_ENOEXEC if header is not match expected
 * @return CDN_EIO delay in response didn't match one requested
 */
static uint32_t waitOnUcpu(DP_PrivateData* pD, uint32_t delay)
{
    uint32_t retVal = CDN_EOK;
    uint32_t resp;

    /* send wait request */
    sendWaitMessage(pD, delay);

    messageReceive(pD, DP_BUS_TYPE_APB);
    /* test if header matches */
    if (0U == messageHeaderMatches(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_WAIT)) {
        retVal = CDN_ENOEXEC;
    }

    if (CDN_EOK == retVal) {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint32(pD, &resp);
        /* test if response is equal expected value */
        if (resp != delay) {
            retVal = CDN_EIO;
        }
    }

    return retVal;
}

/*
 * Adjust drive settings for link establish. Do single training steps and check it as long
 * as max swing was reached, clock recovery was not passed 10 times or adjust is requested
 * for 5 times in row.
 * @param[in] pD pointer to DP private data object
 * @param[in] laneCount number of lanes
 * @param[out] stepStatus pointer to training step status object
 * @return CDN_EOK if success
 * @retrun CDN_EIO if single step was failed
 */
static uint32_t establishLinkCr(DP_PrivateData*     pD,
                                uint8_t             laneCount,
                                TrainingStepStatus* stepStatus) {
    uint8_t isAdjustRequested = 0U;
    uint8_t counterShort = 0U; /* retries with same voltage swing and pre-emphasis */
    uint8_t counterLong = 0U; /* retries with same link rate and lane count */
    uint8_t crDone = 0U;
    uint8_t maxSwingReached = 0U;
    DP_LinkState requestedAdjust;
    uint32_t retVal = CDN_EOK;

    while ((0U == crDone) && (counterShort < 5U) && (counterLong < 10U) && (0U == maxSwingReached)) {
        /* Try to establish link by adjusting drive settings, as described in DP v1.4 */
        if (0U != isAdjustRequested) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Voltage swing / pre-emphasis "
                                         "adjust requested during CR phase.\n");
            counterShort = 0U;
            adjustLaneSettings(pD, &requestedAdjust, laneCount);
            retVal = applyLaneSettings(pD, laneCount);
        }

        if (CDN_EOK == retVal) {
            /* do single training step */
            retVal = trainingSingleStep(pD,
                                        laneCount,
                                        DP_LINK_TRAINING_CR_DELAY_US,
                                        stepStatus,
                                        &requestedAdjust);
        }
        if (CDN_EOK != retVal) {
            break;
        }

        /* check clock recovery */
        crDone = trainingStepCrCheck(stepStatus, laneCount);

        /* calculate max reached swing */
        maxSwingReached = maxSwingCheck(pD, stepStatus, laneCount);

        if (0U == crDone) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Not all CR_DONE bits set.\n");
            counterLong++;
            isAdjustRequested = laneSetingsDiffer(pD, &requestedAdjust, laneCount);
            if (0U == isAdjustRequested) {
                counterShort++;
            }
        }
    }

    return retVal;
}
/**
 * Performs Full Link Training operations for Clock Recovery phase.
 * @param[in] pD pointer to DP private data object
 * @param[in] laneCount number of lanes
 * @param[out] phaseStatus array with current trainig phase status for lanes
 * @return CDN_EOK if success
 * @return other code if cannot establish clock recovery or set training pattern
 */
static uint32_t fullTrainingCr(DP_PrivateData*     pD,
                               uint8_t             laneCount,
                               TrainingPhaseStatus phaseStatus[4])
{
    uint32_t retVal = CDN_EOK;
    uint8_t i;
    TrainingStepStatus stepStatus;

    /* prepare lane settings */
    resetLaneSettings(pD, laneCount);
    retVal = applyLaneSettings(pD, laneCount);

    if (CDN_EOK == retVal) {
        /* send data to 100h DPCD register */
        retVal = sendDpcd100h(pD);
    }

    if (CDN_EOK == retVal) {
        /* set pattern for clock recovery phase */
        retVal = setTrainingPattern(pD, 1U, 1U);
    }

    if (CDN_EOK == retVal) {
        /* establish link */
        retVal = establishLinkCr(pD, laneCount, &stepStatus);
    }

    if (CDN_EOK == retVal) {
        /* set phase status for each lane */
        for (i = 0U; i < laneCount; i++) {
            if (0U != stepStatus.crDone[i]) {
                phaseStatus[i] = LT_PHASE_PASS;
            } else {
                phaseStatus[i] = LT_PHASE_FAIL;
            }
        }
    }

    return retVal;
}

/*
 * Adjust drive settings for link establish. Do single training steps and check it as long
 * as max swing was reached, clock recovery was not passed 10 times or adjust is requested
 * for 5 times in row.
 * @param[in] pD pointer to DP private data object
 * @param[in, out] crFailed clock recovery failed flag
 * @param[out] eqDone status of equalization phase
 * @param[in] laneCount number of lanes
 * @return CDN_EOK if success
 * @retrun CDN_EIO if single step was failed
 */
static uint32_t establishLinkEq(DP_PrivateData* pD, uint8_t* crFailed, uint8_t* eqDone, uint8_t laneCount)
{
    uint8_t isAdjustRequested = 0U;
    uint8_t failCounter = 0U; /* retries with same voltage swing and pre-emphasis */
    TrainingStepStatus stepStatus;
    DP_LinkState requestedAdjust;
    uint32_t retVal = CDN_EOK;

    while ((0U == *crFailed) && (0U == *eqDone) && (failCounter < 5U)) {
        /* Try to establish link by adjusting drive settings, as described in DP v1.4 */
        if (0U != isAdjustRequested) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Voltage swing / pre-emphasis "
                                         "adjust requested during EQ phase.\n");
            adjustLaneSettings(pD, &requestedAdjust, laneCount);
            retVal = applyLaneSettings(pD, laneCount);
        }

        if (CDN_EOK == retVal) {
            /* do single training step */
            retVal = trainingSingleStep(pD,
                                        laneCount,
                                        (uint16_t)pD->sinkCaps.TrainingInterval,
                                        &stepStatus,
                                        &requestedAdjust);
        }
        if (CDN_EOK != retVal) {
            break;
        }

        if (0U == trainingStepCrCheck(&stepStatus, laneCount)) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: CR_DONE no longer set during EQ phase.\n");
            *crFailed = 1U;
        } else {
            *eqDone = trainingStepEqCheck(&stepStatus, laneCount);
            if (0U == *eqDone)
            {
                DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Not all bits, required for "
                                             "successful EQ, were set.\n");
                failCounter++;
                isAdjustRequested = laneSetingsDiffer(pD, &requestedAdjust, laneCount);
            }
        }
    }

    return retVal;
}
/**
 * Performs Full Link Training operations for Channel Equalization phase.
 * @param[in] pD pointer to DP private datat object
 * @param[in] laneCount nubmer of lanes
 * @param[out] phaseStatus pointer to training phase status object
 * @return CDN_EOK if success
 * @return CDN_EIO if cannot establish for equalization or set training pattern
 */
static uint32_t fullTrainingEq(DP_PrivateData*      pD,
                               uint8_t              laneCount,
                               TrainingPhaseStatus* phaseStatus)
{
    uint32_t retVal = CDN_EOK;
    uint8_t crFailed = 0U;
    uint8_t eqDone = 0U;

    /* set pattern */
    retVal = setTrainingPattern(pD, getHighestCommonTps(pD), 1U);

    if (CDN_EOK == retVal) {
        /* establish link phase */
        retVal = establishLinkEq(pD, &crFailed, &eqDone, laneCount);
    }

    if (CDN_EOK == retVal) {
        /* set current phase status */
        if ((0U != crFailed) || (0U == eqDone)) {
            *phaseStatus = LT_PHASE_FAIL;
        } else {
            *phaseStatus = LT_PHASE_PASS;
        }
    }

    return retVal;
}

/**
 * Set both main link rate and lane count
 */
static uint32_t setPhyLink(DP_PrivateData* pD, DP_SD0801_LinkState* sdState)
{
    uint32_t retVal;

    retVal = DP_SD0801_SetLinkRate(pD->phyPd, sdState);

    if (CDN_EOK == retVal) {
        /* Set lane count if setting link rate succeeds. */
        retVal = DP_SD0801_EnableLanes(pD->phyPd, sdState);
    }

    return retVal;
}

/*
 * Enable lanes and configure link state
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_ENOTSUP if lane count is greater than max
 * @return other code if cannot enable lanes or stop transmitting TPS
 */
static uint32_t prepareFullTraining(DP_PrivateData* pD)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint8_t maxLaneCount;
    uint32_t retVal;
    DP_SD0801_LinkState sdState;

    DbgMsg(DBG_GEN_MSG, DBG_FYI, "Full Link Training start.\n");

    /* Stop transmitting TPS */
    retVal = setTrainingPattern(pD, 0U, 1U);

    if (CDN_EOK == retVal) {
        /* enable lanes for DPTX */
        maxLaneCount = getMaxLaneCount(pD);
        if (32U > maxLaneCount) {
            regTransfer.val =  ((uint32_t)1U << maxLaneCount) - 1U;
            regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_LANE_EN_p);
            retVal = DP_WriteRegister(pD, &regTransfer);
        }
        else {
            retVal = CDN_ENOTSUP;
        }
    }

    if (CDN_EOK == retVal) {

        /* configure link state */
        pD->linkState.laneCount = maxLaneCount;
        pD->linkState.linkRate = getMaxCommonRate(
            pD->sourceCaps.maxLinkRate,
            pD->sinkCaps.maxLinkRate);
        /* Use SSC, if enabled and supported by sink device. */
        pD->linkState.ssc = getSscSupported(pD);
        toSdLinkState(&(pD->linkState), &sdState);
        retVal = setPhyLink(pD, &sdState);
    }

    return retVal;
}

/*
 * Reduce link rate if not minimal or reduce lane count when possible for
 * clock recovery phase
 * @param[in, out] pointer to DP private data object
 * @param[in,out] lanesPassed number of passed lanes
 * @param[in] phaseStatus pointer to phase status object
 * @param[in,out] result pointer to current training status
 * @return CDN_EOK if fixing success or was failed
 * @return other code if cannot stop TPS transmiting
 */
static uint32_t fixPhaseCr(DP_PrivateData*            pD,
                           uint8_t*                   lanesPassed,
                           const TrainingPhaseStatus* phaseStatus,
                           DP_TrainingStatus*         result) {

    uint32_t retVal = CDN_EOK;
    DP_SD0801_LinkState sdState;

    if (DP_LINK_RATE_1_62 != pD->linkState.linkRate) {
        retVal = setTrainingPattern(pD, 0U, 1U);

        if (CDN_EOK == retVal) {
            /* Fallback: Reduce link rate, when possible. */
            pD->linkState.linkRate = reduceLinkRate(pD->linkState.linkRate);
            toSdLinkState(&(pD->linkState), &sdState);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Reducing Link Rate during CR phase.\n");
            retVal = DP_SD0801_SetLinkRate(pD->phyPd, &sdState);
        }

    } else if (0U != checkLaneCountFallback(pD, phaseStatus, lanesPassed)) {
        /* Fallback: Reduce lane count, when possible and allowed. */
        pD->linkState.laneCount = *lanesPassed;
        /* restore link rate */
        pD->linkState.linkRate = getMaxCommonRate(
            pD->sourceCaps.maxLinkRate,
            pD->sinkCaps.maxLinkRate);
        toSdLinkState(&(pD->linkState), &sdState);
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Reducing Lane Count during CR phase.\n");
        retVal = DP_SD0801_SetLinkRate(pD->phyPd, &sdState);
        if (CDN_EOK == retVal) {
            retVal = DP_SD0801_EnableLanes(pD->phyPd, &sdState);
        }
    } else {
        /* Clock Recovery (CR) failed. */
        *result = DP_LT_CR_FAIL;
    }

    return retVal;
}

/*
 * Reduce lane count if greater than 1 or reduce link rate when possible for
 * equalization phase
 * @param[in, out] pointer to DP private data object
 * @param[in,out] lanesPassed number of passed lanes
 * @param[in] phaseStatus pointer to phase status object
 * @param[in,out] result pointer to current training status
 * @return CDN_EOK if fixing success or was failed
 * @return other code if cannot stop TPS transmiting
 */
static uint32_t fixPhaseEq(DP_PrivateData* pD, DP_TrainingStatus* result) {

    uint32_t retVal = CDN_EOK;
    DP_SD0801_LinkState sdState;

    if (1U < pD->linkState.laneCount) {
        /* reduce lane count for device, when possible */
        pD->linkState.laneCount = reduceLaneCount(pD->linkState.laneCount);
        toSdLinkState(&(pD->linkState), &sdState);
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Reducing Lane Count during EQ phase.\n");
        retVal = DP_SD0801_EnableLanes(pD->phyPd, &sdState);
    } else if (DP_LINK_RATE_1_62 != pD->linkState.linkRate) {
        retVal = setTrainingPattern(pD, 0U, 1U);
        if (CDN_EOK == retVal) {
            /* reduce link rate for device, when possible */
            pD->linkState.linkRate = reduceLinkRate(pD->linkState.linkRate);
            /* restore lane count */
            pD->linkState.laneCount = getMaxLaneCount(pD);
            toSdLinkState(&(pD->linkState), &sdState);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Reducing Link Rate during EQ phase.\n");
            retVal = DP_SD0801_SetLinkRate(pD->phyPd, &sdState);
        }
        if (CDN_EOK == retVal) {
            retVal = DP_SD0801_EnableLanes(pD->phyPd, &sdState);
        }
    } else {
        *result = DP_LT_EQ_FAIL;
    }

    return retVal;
}

/*
 * Execute training for clock recovery phase
 * @param[in,out] pD pointer to DP private data object
 * @param[in,out] result current result of training phase
 * @return CDN_EOK if success or cannot reduce link rate/lane count
 * @return other error code if phase was not executed properly
 */
static uint32_t executeCrTrainingPhase(DP_PrivateData* pD, DP_TrainingStatus* result, uint8_t* crDone)
{
    uint32_t retVal;
    TrainingPhaseStatus phaseStatus[4];
    uint8_t lanesPassed;

    /* do full trainig for clock recovery phase */
    retVal = fullTrainingCr(pD, pD->linkState.laneCount, phaseStatus);

    if (CDN_EOK == retVal) {
        /* check if training passed */
        *crDone = trainingPassCheck(phaseStatus, pD->linkState.laneCount);
        if (0U == *crDone) {
            retVal = fixPhaseCr(pD, &lanesPassed, phaseStatus, result);
        }
    }

    return retVal;
}

/*
 * Execute training for equalization phase
 * @param[in,out] pD pointer to DP private data object
 * @param[in,out] result current result of training phase
 * @return CDN_EOK if success or cannot reduce link rate/lane count
 * @return other error code if phase was not executed properly
 */
static uint32_t executeEqTrainingPhase(DP_PrivateData* pD, DP_TrainingStatus* result, uint8_t* eqDone)
{
    TrainingPhaseStatus phaseStatus[4];
    uint32_t retVal;

    retVal = fullTrainingEq(pD, pD->linkState.laneCount, phaseStatus);

    if (retVal == CDN_EOK) {
        /* Check only first value, as it applies to all lanes during EQ phase */
        *eqDone = trainingPassCheck(phaseStatus, 1U);
        if (0U == *eqDone) {
            retVal = fixPhaseEq(pD, result);
        }
    }

    return retVal;
}

/*
 * Execute full training
 * @param[in,out] pD pointer to DP private data object
 * @param[in,out] result current result of training phase
 * @return CDN_EOK if success or cannot reduce link rate/lane count
 * @return other error code if training was not executed properly
 */
static uint32_t executeFullTraining(DP_PrivateData* pD, DP_TrainingStatus* result)
{
    uint32_t retVal;
    uint8_t eqDone = 0U;
    uint8_t crDone = 0U;

    *result = DP_LT_OK;

    while (DP_LT_OK == *result)
    {
        /* Perform CR training phase. */
        retVal = executeCrTrainingPhase(pD, result, &crDone);

        if ((CDN_EOK == retVal) && (DP_LT_OK == *result)) {
            if (0U == crDone) {
                /* Reducing was passed, try again */
                continue;
            }
            /* If this point is reached, CR phase has passed. */
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: CR phase done.\n");
            retVal = executeEqTrainingPhase(pD, result, &eqDone);
        }

        /* Need not check result, because if eqDone is 1U, it means that */
        /* executeEqTraining was executed and not change value of result, */
        /* so it is DP_LT_OK (previous if checked it) */
        if ((CDN_EOK == retVal) && (0U != eqDone)) {
            /* If this point is reached, EQ phase has passed */
            break;
        }

        if (CDN_EOK != retVal) {
            /* Something going wrong. If retVal is CDN_EOK, but result is not DP_LT_OK loop */
            /* will break by 'while' statement */
            *result = DP_LT_UNFINISHED;
        }
    }

    if ((CDN_EOK == retVal) && (DP_LT_OK == *result)) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "Link Training: Full Training finished.\n");
    }

    return retVal;
}
/**
 * Performs Full Link Training operations. May be called after calling
 * DP_InitTraining.
 * @param[in,out] pD pointer to DP private data object
 * @param[in,out] result pointer to training status object
 * @return CDN_EOK if success
 * @return CDN_ENOTSUP if sink doesn't support link method
 * @return other code if training was not executed correctly
 */
static uint32_t fullTraining(DP_PrivateData* pD, DP_TrainingStatus* result)
{
    uint32_t retVal;
    *result = DP_LT_OK;

    /* test if link method is supported by sink */
    if (0U == pD->sinkCaps.linkBwSupported) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Full training was requested, while sink device"
                                      "doesn't support LINK_BW_SET method.\n");
        retVal = CDN_ENOTSUP;
    } else {
        /* do full link training */
        retVal = prepareFullTraining(pD);
        if (CDN_EOK == retVal) {
            retVal = executeFullTraining(pD, result);
        }
        (void)cleanupTraining(pD, 1U);
    }

    return retVal;
}

/* Transmit patterns for fast training
 * @param[in,out] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return other code when cannot set training pattern or uCOU waits
 *  different time than expected
 */
static uint32_t fastTrainingTransmitPatterns(DP_PrivateData* pD)
{
    uint32_t retVal;

    /* Transmit TPS1 for 500 us */
    retVal = setTrainingPattern(pD, 1U, 0U);
    if (CDN_EOK == retVal) {
        retVal = waitOnUcpu(pD, 500U);
    }

    /* Transmit TPSx (2-4) for 500 us */
    if (CDN_EOK == retVal) {
        retVal = setTrainingPattern(pD, getHighestCommonTps(pD), 0U);
    }

    if (CDN_EOK == retVal) {
        retVal = waitOnUcpu(pD, 500U);
    }

    return retVal;
}

/*
 * Send display port configuration data
 * @param[in] pD pointer to DP private data object
 * @param[in] maxRate maximum rate
 * @return CDN_EOK if success
 * @return CDN_EIVAL if cannot send
 */
static uint32_t sendDpcd(DP_PrivateData* pD, DP_LinkRate maxRate) {

    uint32_t retVal = CDN_EOK;

    if ((0U == isEdpRate(maxRate)) &&
        (0U != pD->sinkCaps.linkBwSupported) &&
        (maxRate <= pD->sourceCaps.maxLinkRate))
    {
        /* use LINK_BW_SET DPCD approach */
        retVal = sendDpcd100h(pD);
    } else {
        /* use LINK_RATE_SET approach. */
        retVal = sendDpcd101h(pD);
        if (CDN_EOK == retVal) {
            retVal = sendDpcd115h(pD);
        }
    }

    return retVal;
}

/*
 * Prepare device for fast training
 * @param[in] pD pointer to driver's private data object
 * @param[in] linkRate pointer to link rate object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if transfer parameters not vailid
 */
static uint32_t prepareFastTraining(DP_PrivateData* pD, DP_LinkRate* linkRate)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t retVal = getMaxLinkRate(pD, linkRate);
    DP_SD0801_LinkState sdState;

    if (CDN_EOK == retVal) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "Fast Link Training start.\n");
        uint8_t maxLaneCount = getMaxLaneCount(pD);

        pD->linkState.laneCount = maxLaneCount;
        pD->linkState.linkRate = *linkRate;
        /* Use SSC, if enabled and supported by sink device. */
        pD->linkState.ssc = getSscSupported(pD);

        if (32U > maxLaneCount) {
            /* enable lanes */
            uint8_t value = (1U << maxLaneCount) - 1U;
            regTransfer.val = (uint32_t)value;
            regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_LANE_EN_p);
            retVal = DP_WriteRegister(pD, &regTransfer);
        }
        if (CDN_EOK == retVal)
        {
            /* configure phy for training */
            toSdLinkState(&(pD->linkState), &sdState);
            retVal = DP_SD0801_SetLinkRate(pD->phyPd, &sdState);
        }
        if (CDN_EOK == retVal)
        {
            retVal = DP_SD0801_EnableLanes(pD->phyPd, &sdState);
        }
    }

    return retVal;

}

/*
 * Execute fast training
 * @param[in,out] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return other code if not success
 */
static uint32_t fastTraining(DP_PrivateData* pD)
{
    DP_LinkRate maxRate;
    uint32_t retVal = prepareFastTraining(pD, &maxRate);

    if (CDN_EOK == retVal) {

        /* Set voltage swing and pre-emphasis levels to '0', or to ones forced. */
        resetLaneSettings(pD, pD->linkState.laneCount);
        retVal = applyLaneSettings(pD, pD->linkState.laneCount);

        if (CDN_EOK == retVal)
        {
            retVal = sendDpcd(pD, maxRate);
        }

        if (CDN_EOK == retVal) {
            /* transmit pattern for fast training */
            retVal = fastTrainingTransmitPatterns(pD);

            (void)cleanupTraining(pD, 0U);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "Fast Link Training finished.\n");
        }
    }

    return retVal;
}

uint32_t DP_SetSourceCapabilities(DP_PrivateData* pD, const DP_SourceDeviceCapabilities* caps)
{
    uint32_t retVal = DP_SetSourceCapabilitiesSF(pD, caps);

    if (CDN_EOK == retVal) {
        /* write source capabilities into driver's private data */
        CPS_BufferCopy((uint8_t*)&(pD->sourceCaps),
                       (const uint8_t*)caps,
                       (uint32_t)sizeof(DP_SourceDeviceCapabilities));
        pD->sourceCapsStored = 1U;
    }

    return retVal;
}

uint32_t DP_GetSinkCapabilities(DP_PrivateData* pD, DP_SinkDeviceCapabilities* caps)
{
    /* test if parameters are valid */
    uint32_t retVal = DP_GetSinkCapabilitiesSF(pD, caps);

    if (CDN_EOK == retVal) {
        retVal = readSinkCapabilities(pD);
    }

    if (CDN_EOK == retVal) {
        /* copy sink capabilities from driver's private data into sink capabilities object */
        CPS_BufferCopy((uint8_t*)caps,
                       (uint8_t*)(&(pD->sinkCaps)),
                       (uint32_t)sizeof(DP_SinkDeviceCapabilities));
    }

    return retVal;
}

uint32_t DP_LinkTraining(DP_PrivateData* pD, DP_TrainingStatus* resultLt)
{
    /* test if parameters are valid */
    uint32_t retVal = DP_LinkTrainingSF(pD, resultLt);

    if (CDN_EOK == retVal) {
        /* test if any source caps stored */
        if (0U == pD->sourceCapsStored) {
            retVal = CDN_ENOENT;
        }
    }

    if (CDN_EOK == retVal) {
        /* read sink capabilities */
        retVal = readSinkCapabilities(pD);
    }

    if (CDN_EOK == retVal) {
        /* prepare training */
        retVal = initTraining(pD);
    }

    if (CDN_EOK == retVal) {
        /* do full training if fast training is not supported */
        if (!getFastTrainingSupported(pD)) {
            retVal = fullTraining(pD, resultLt);
        } else {
            *resultLt = DP_LT_OK;
            retVal = fastTraining(pD);
        }

        if (CDN_EOK != retVal) {
            *resultLt = DP_LT_UNFINISHED;
        }
    }

    return retVal;
}

uint32_t DP_CheckLinkStable(DP_PrivateData* pD, bool* resultLs)
{
    uint32_t retVal;
    uint8_t dpcdBuff[3];
    uint8_t stepResult;
    DP_DpcdTransfer transfer = {0U};
    TrainingStepStatus status;

    /* check if parameters are valid. */
    retVal = DP_CheckLinkStableSF(pD, resultLs);

    if (CDN_EOK == retVal) {
        /* read 0x202 DPCD buffer */
        setDpcdTransfer(&transfer, 0x202U, 3U, dpcdBuff);
        retVal = DP_ReadDpcd(pD, &transfer);
    }

    if (CDN_EOK == retVal) {
        /* check if link is stable and synchronized with sink. */
        clearTrainingResult(&status);
        parseLanesStatus(transfer.buff, &status);
        stepResult = trainingStepEqCheck(&status, pD->linkState.laneCount);
        *resultLs = (0U != stepResult);
    }

    return retVal;
}

uint32_t DP_ReadLinkStat(DP_PrivateData* pD, DP_LinkState* linkState)
{
    uint32_t retVal = DP_ReadLinkStatSF(pD, linkState);
    uint8_t i;

    if (CDN_EOK == retVal) {
        /* get link state from PHY driver and update it in private data. */
        retVal = fetchLinkState(pD);
    }
    if (CDN_EOK == retVal) {
        /* copy link state from driver's private data to link state structure. */
        linkState->laneCount = pD->linkState.laneCount;
        linkState->linkRate = pD->linkState.linkRate;
        linkState->ssc = pD->linkState.ssc;
        for (i = 0; i < 4U; i++) {
            /* copy voltage levels, per-lane. */
            linkState->voltageSwing[i] = pD->linkState.voltageSwing[i];
            linkState->preEmphasis[i] = pD->linkState.preEmphasis[i];
        }
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-36 */
