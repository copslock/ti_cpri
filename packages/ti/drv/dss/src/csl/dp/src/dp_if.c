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
 * dp_if.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-36-3 "Function called from more than 5 different functions, DRV-3823" */
/* parasoft-begin-suppress METRICS-41 "Number of comments per block" */

#include "dp_structs_if.h"
#include "dp_priv.h"
#include "dp_sanity.h"
#include "dp_internal.h"
#include "dp_dsc.h"
#include "dp_link_policy.h"
#include "dp_utils.h"
#include "dp_register.h"
#include "dsc_utils.h"
#include "dp_aux.h"
#include "dp_aux_if.h"
#include "dp_aux_structs_if.h"
#include "dp_mst.h"
#include "dp_transaction.h"
#include "dp_mst_if.h"
#include "dp_mailbox.h"

#include "mhdp_apb_regs.h"
#include "cps_drv.h"

#include "cdn_log.h"
#include "cdn_errno.h"
#include "custom_types.h"

#include <string.h> /* for memset */

static bool isSinkPort(const MST_drm_dp_port *port)
{
    bool isOutputConnected = ((port->refcount != 0U) && (!port->input) && (port->ddps));
    bool isSink = (port->pdt == DP_PEER_DEVICE_SST_SINK) || (port->pdt == DP_PEER_DEVICE_DP_LEGACY_CONV);

    return isOutputConnected && isSink;
}

static uint32_t collectSinkPorts(const MST_drm_dp_branch*  mstBranch,
                                 const MST_drm_dp_port**   portList,
                                 const MST_drm_dp_branch** branchesArray,
                                 uint8_t*                  sinkCount,
                                 uint8_t*                  branchesArrayHead)
{
    uint32_t retVal = CDN_EOK;
    const MST_drm_dp_port *port;
    uint8_t i;

    for (i = 0U; i < MST_BRANCH_MAX_PORTS; i++) {
        port = &(mstBranch->ports[i]);
        if ((isSinkPort(port)) && (*sinkCount < DP_MAX_NUMBER_OF_STREAMS)) {
            portList[*sinkCount] = port;
            (*sinkCount)++;
        } else if ((port->pdt == DP_PEER_DEVICE_MST_BRANCHING) && (port->refcount != 0U)) {
            if (*branchesArrayHead < MST_MAX_BRANCH_NUMBER) {
                branchesArray[*branchesArrayHead] = port->mstb;
                (*branchesArrayHead)++;
            } else {
                retVal = CDN_EINVAL;
                break;
            }
        } else {
            /*
             * All 'if ... else if' constructs shall be terminated with an 'else' statement
             * (MISRA2012-RULE-15_7-3)
             */
        }
    }

    return retVal;
}

static uint32_t collectSinkPortList(const MST_drm_dp_branch* mstBranch,
                                    const MST_drm_dp_port**  portList,
                                    uint8_t*                 sinkCount)
{
    MST_drm_dp_branch* branchesArray[MST_MAX_BRANCH_NUMBER] = {NULL};
    bool maxDeepLevelReached;
    uint8_t branchesArrayHead = 0U;
    uint8_t branchesArrayActualPosition = 0U;
    uint8_t deepLevelIndex = 0U;
    uint8_t deepLevel = 1U;
    uint32_t retVal;
    MST_drm_dp_branch* actualBranch;

    /* Collect ports and branches for first node. */
    retVal = collectSinkPorts(mstBranch,
                              portList,
                              (const MST_drm_dp_branch**) branchesArray,
                              sinkCount,
                              &branchesArrayHead);

    actualBranch = branchesArray[branchesArrayActualPosition];
    deepLevelIndex = branchesArrayHead;

    while ((actualBranch != NULL) && (CDN_EOK == retVal)) {
        retVal = collectSinkPorts(actualBranch,
                                  portList,
                                  (const MST_drm_dp_branch**) branchesArray,
                                  sinkCount,
                                  &branchesArrayHead);

        branchesArrayActualPosition++;
        actualBranch = branchesArray[branchesArrayActualPosition];

        if (branchesArrayActualPosition == deepLevelIndex) {
            deepLevel++;
            deepLevelIndex = branchesArrayHead;
        }

        maxDeepLevelReached = (MST_MAX_BRANCH_DEEP_LEVEL == deepLevel);

        /* Max depth level of branches was reached, but there is more to check */
        if ((maxDeepLevelReached) && (NULL != actualBranch)) {
            retVal = CDN_EINVAL;
        }

        /* Break collecting sink ports upon reaching max depth level of branches or collecting max number of ports. */
        if ((*sinkCount == DP_MAX_NUMBER_OF_STREAMS) || (maxDeepLevelReached)) {
            break;
        }
    }
    return retVal;
}

static void removeDisconnectedSinks(DP_PrivateData*         pD,
                                    const MST_drm_dp_port** portList,
                                    uint8_t                 sinkCount)
{
    uint8_t i;
    uint8_t j;
    bool markRemove = false;

    for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {
        if (NULL != pD->sinkList[i].port) {
            markRemove = true;
            for (j = 0U; j < sinkCount; j++) {
                if (pD->sinkList[i].port == portList[j]) {
                    markRemove = false;
                }
            }
            if (markRemove) {
                pD->sinkList[i].port = NULL;
            }
        }
    }
}

static uint32_t updateConnectedSinks(DP_PrivateData*   pD,
                                     MST_drm_dp_port** port_list,
                                     uint8_t           sink_count) {
    uint32_t retVal = CDN_EOK;
    bool newSink = false;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < sink_count; i++) {
        newSink = true;
        for (j = 0U; j < DP_MAX_NUMBER_OF_STREAMS; j++) {
            if (pD->sinkList[j].port == port_list[i]) {
                newSink = false;
                break;
            }
        }

        if (newSink) {
            for (j = 0U; j < DP_MAX_NUMBER_OF_STREAMS; j++) {

                if (pD->sinkList[j].port == NULL) {
                    pD->sinkList[j].port = port_list[i];
                    newSink = false;
                    break;
                }
            }

            if (newSink) {
                retVal = CDN_EINVAL;
            }
        }
    }

    return retVal;
}

/**
 * Get symbol rate, in units of 1M symbols / second / lane (equal to tens of Mib/s)
 */
uint16_t getSymbolRate(DP_LinkRate linkRate)
{
    uint16_t result;
    switch (linkRate)
    {
    case DP_LINK_RATE_1_62:
        result = 162U;
        break;
    case DP_LINK_RATE_2_16:
        result = 216U;
        break;
    case DP_LINK_RATE_2_43:
        result = 243U;
        break;
    case DP_LINK_RATE_2_70:
        result = 270U;
        break;
    case DP_LINK_RATE_3_24:
        result = 324U;
        break;
    case DP_LINK_RATE_4_32:
        result = 432U;
        break;
    case DP_LINK_RATE_5_40:
        result = 540U;
        break;
    default:
        result = 810U;
        break;
    }
    return result;
}

/**
 * Function calculates bits per component, adjusted for pixel encoding format
 */
float64_t calculateBitsPerComponent(const DP_PrivateData* pD, uint8_t streamId)
{
    float64_t bitsPerCompCalc;
    DP_PixelEncodingFormat pixelEncodingFormat = pD->videoParameters[streamId].pxEncFormat;
    float64_t bitsPerComp = (float64_t)pD->videoParameters[streamId].bitsPerSubpixel;

    if (pixelEncodingFormat == DP_PXENC_YCBCR_4_2_2) {
        bitsPerCompCalc = (bitsPerComp * 2.0) / 3.0;
    }
    else if (pixelEncodingFormat == DP_PXENC_YCBCR_4_2_0) {
        bitsPerCompCalc = bitsPerComp / 2.0;
    }
    else {
        bitsPerCompCalc = bitsPerComp;
    }

    if (pD->videoParameters[streamId].dscEnable) {
        bitsPerComp = (float64_t)pD->dscConfig[streamId].bitsPerPixel;
        bitsPerCompCalc = (bitsPerComp / 16.0) / 3.0;
    }

    return bitsPerCompCalc;

}

/**
 * Local struct containing parameters used for calculation of Valid Symbols per
 * Transfer Unit.
 */
struct validSymbolCalcParams
{
    uint8_t tuSize;
    float64_t pxlClk;
    float64_t bitsPerComp;
    uint8_t laneCount;
    float64_t rate;
    float64_t validSymPercentOverhead;
};

/**
 * Calculate integer and fractional parts of valid symbols per transfer unit
 * (TU), based on TU size, pixel clock (in MHz), bits per component (adjusted
 * for pixel encoding), lane count and symbol rate (in M Symbols/lane/second)
 */
static void calculateValidSymbol(const struct validSymbolCalcParams* params,
                                 uint32_t *                          validSym,
                                 uint16_t *                          validSymF)
{
    float64_t tempValidSym = (float64_t)params->tuSize * params->pxlClk * params->bitsPerComp * 3000.0;
    tempValidSym = floor(tempValidSym);

    if (params->validSymPercentOverhead > 0.0) {
        tempValidSym *= ((params->validSymPercentOverhead / 100.0) + 1.0);
        tempValidSym = floor(tempValidSym);
    }

    tempValidSym /= (float64_t)params->laneCount * params->rate * 8.0;

    *validSymF = (uint16_t)tempValidSym % 1000U;
    *validSym = (uint32_t)tempValidSym / 1000U;
}

static bool isTuSizeOptimal(uint8_t  tu_size,
                            uint32_t validSym,
                            uint16_t validSymF,
                            uint32_t validSym2,
                            uint16_t validSym2F)
{
    bool tuOptimal = true;

    /* too few valid symbols (VS) per TU */
    if (validSym == 1U) {
        tuOptimal = false;
    }

    /* VS too close to TU size */
    if ((tu_size - validSym) < 2U) {
        tuOptimal = false;
    }

    /* down-spread changes integer part of VS per TU */
    if (validSym != validSym2) {
        tuOptimal = false;
    }

    /* fractional part of VS per TU too close to 1 (1000/1000) */
    if (validSymF > 850U) {
        tuOptimal = false;
    }

    /* fractional part of VS per TU too close to 0 (0/1000) */
    if (validSymF < 100U) {
        tuOptimal = false;
    }

    /* fractional part of VS per TU (down-spread) too close to 1 (1000/1000) */
    if (validSym2F > 850U) {
        tuOptimal = false;
    }

    /* fractional part of VS per TU (down-spread) too close to 0 (0/1000) */
    if (validSym2F < 100U) {
        tuOptimal = false;
    }

    /* Maximum TU size is reached - no much choice, have to accept. */
    if (tu_size >= 64U) {
        tuOptimal = true;
    }

    return tuOptimal;
}

/**
 * Copies structure for calculating valid symbols per TU, applying 0.5% down-spread.
 */
static void fillPostSpreadParams(const struct validSymbolCalcParams* preSpreadParams,
                                 struct validSymbolCalcParams*       postSpreadParams)
{
    postSpreadParams->tuSize = preSpreadParams->tuSize;
    postSpreadParams->pxlClk = preSpreadParams->pxlClk;
    postSpreadParams->bitsPerComp = preSpreadParams->bitsPerComp;
    postSpreadParams->laneCount = preSpreadParams->laneCount;
    /* link rate adjusted for 0.5% down-spread */
    postSpreadParams->rate = (preSpreadParams->rate * 0.995);
    postSpreadParams->validSymPercentOverhead = preSpreadParams->validSymPercentOverhead;
}

/**
 * Calculate transfer unit parametrs
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] validSymOverhead valid symbols overhead in percents,
 *              if no extra overhead to be added should be 0
 * @param[out] validSym valid symbols (VS) per TU - integer part
 * @param[out] validSymF valid symbols per TU - fractional part (thousandth parts, 0 - 999)
 * @param[out] tuSize Transfer Unit (TU) size
 * @return CDN_OK success
 * @return CDN_ERROR_NOT_SUPPORTED if link bandwidth is oversubscribed
 */

static uint32_t calculateTuParams(const DP_PrivateData* pD,
                                  float64_t             validSymOverhead,
                                  uint32_t *            validSym,
                                  uint16_t *            validSymF,
                                  uint8_t *             tuSize)
{
    const uint8_t streamId = 0U; /* SST mode */
    uint32_t retVal = CDN_EOK;
    /* value of validSym being currently tested */
    uint32_t currValidSym;
    /* value of validSymF being currently tested */
    uint16_t currValidSymF;
    /* like validSym, but taking 0.5% down-spread into consideration */
    uint32_t currValidSym2;
    /* like validSymF, but taking down-spread into consideration */
    uint16_t currValidSym2F;
    /* value of tuSize being currently tested */
    uint8_t currTuSize = 32U;

    bool optimalSize;
    float64_t bitsPerCompCalc = calculateBitsPerComponent(pD, streamId);
    float64_t pxlFreq = pD->videoParameters[streamId].vicParams.pxlFreq;
    float64_t symbolRate = (float64_t)(getSymbolRate(pD->linkState.linkRate));
    uint8_t laneCount = pD->linkState.laneCount;

    struct validSymbolCalcParams preSpreadParams;
    struct validSymbolCalcParams postSpreadParams;

    preSpreadParams.tuSize = currTuSize;
    preSpreadParams.pxlClk = pxlFreq;
    preSpreadParams.bitsPerComp = bitsPerCompCalc;
    preSpreadParams.laneCount = laneCount;
    preSpreadParams.rate = symbolRate;
    preSpreadParams.validSymPercentOverhead = validSymOverhead;

    fillPostSpreadParams(&preSpreadParams, &postSpreadParams);

    /* Find optimal value for the TU_SIZE, incrementally trying even values, */
    /* starting from minimum (32), up to maximum (64) */
    while (true) {
        calculateValidSymbol(&preSpreadParams, &currValidSym, &currValidSymF);

        calculateValidSymbol(&postSpreadParams, &currValidSym2, &currValidSym2F);

        optimalSize = isTuSizeOptimal(currTuSize,
                                      currValidSym,
                                      currValidSymF,
                                      currValidSym2,
                                      currValidSym2F);

        if (false == optimalSize) {
            currTuSize += 2U;
            preSpreadParams.tuSize = currTuSize;
            postSpreadParams.tuSize = currTuSize;
        } else {
            break;
        }
    }

    /* Link bandwidth is oversubscribed. */
    if (currValidSym >= 64U) {
        retVal = CDN_ENOTSUP;
    }

    if (CDN_EOK == retVal) {
        *tuSize = currTuSize;
        *validSym = currValidSym;
        *validSymF = currValidSymF;
    }
    return retVal;
}

/**
 * Calculate line threshold parameter for FIFO
 */
static uint8_t calcLineThresh(const DP_PrivateData* pD,
                              uint32_t              validSym)
{
    const uint8_t streamId = 0U; /* SST mode */
    float64_t lineThresh;
    float64_t bitsPerCompCalc = calculateBitsPerComponent(pD, streamId);
    float64_t pxlFreq = pD->videoParameters[streamId].vicParams.pxlFreq;
    uint16_t symbolRate = getSymbolRate(pD->linkState.linkRate);
    uint8_t laneCount = pD->linkState.laneCount;

    float64_t calcBpc = (3.0 * bitsPerCompCalc) / 8.0;
    float64_t validSymUp = (float64_t)validSym + 1.0;

    /* LINE_THRESH set according to presentation. */
    lineThresh = ((validSymUp * (float64_t)laneCount) -
                  (((pxlFreq / (float64_t)symbolRate) * validSymUp * calcBpc) - calcBpc)) /
                 (calcBpc * (float64_t)laneCount);

    lineThresh = ceil(lineThresh);
    lineThresh += 2.0;

    if (lineThresh > 12.0) {
        lineThresh = 12.0;
    }

    return (uint8_t)lineThresh;
}

/**
 * Calculate difference between TU size and VS per TU
 * @param[in] tuSize Transfer Unit (TU) size
 * @param[out] validSym valid symbols (VS) per TU - integer part
 * @return calculated difference between TU size and VS per TU.
 */
static uint32_t calcTuVsDiff(uint8_t tuSize, uint32_t validSym)
{
    uint32_t tuVsDiff = 0U;

    if ((tuSize - validSym) <= 3U ) {
        tuVsDiff = tuSize - validSym;
    }

    return tuVsDiff;
}

static float64_t calculateTargetAverageSlots(const DP_PrivateData* pD,
                                             uint8_t               streamId,
                                             bool                  fecEnabled)
{
    uint32_t payloadBandwidth;
    uint32_t payloadBandwidthDiv;
    float64_t targetAverageSlots;

    const float64_t pixelFrequency = pD->videoParameters[streamId].vicParams.pxlFreq * 1000.0;
    const float64_t bitsPerComponentCalc = calculateBitsPerComponent(pD, streamId) * 3.0;
    uint32_t symbolRate = (uint32_t)getSymbolRate(pD->linkState.linkRate);

    payloadBandwidth = DP_MST_MgrCalcPbnMode((uint32_t)pixelFrequency,
                                             (uint8_t)bitsPerComponentCalc,
                                             fecEnabled);

    payloadBandwidthDiv = ((uint32_t)pD->linkState.laneCount * symbolRate) / MST_LINK_RATE_MULTIPLIER;

    targetAverageSlots = (float64_t)payloadBandwidth / (float64_t)payloadBandwidthDiv;

    return targetAverageSlots;
}

static uint32_t calcSdpBlockVblank(const DP_PrivateData* pD,
                                   uint8_t               streamId,
                                   float64_t             targAvgSlots)
{
    const uint32_t vblankCyclesMax = 65535U; /* 16 bits */

    float64_t vblankCycles;
    uint8_t laneCountCalc; /* Number of lanes, used for calculations. */

    const uint8_t laneCount = pD->linkState.laneCount;
    const uint32_t vTotal = pD->videoParameters[streamId].vicParams.vTotal;
    const uint32_t symbolRate = (uint32_t)getSymbolRate(pD->linkState.linkRate);
    const float64_t pixelClock = pD->videoParameters[streamId].vicParams.pxlFreq;

    /* For MST, calculate like there are always 4 lanes. */
    laneCountCalc = (!pD->mstEnabled) ? laneCount : 4U;

    /* Calculate number of data_clk cycles. */
    uint32_t init = (vTotal * symbolRate) / 2U;
    vblankCycles = (float64_t)init / pixelClock;
    vblankCycles = floor(vblankCycles);

    if (pD->mstEnabled) {
        /* Quantize to slot allocation */
        vblankCycles *= (targAvgSlots / 64.0);
        vblankCycles = floor(vblankCycles);
    }

    /* Apply safety margin. */
    vblankCycles = (vblankCycles * 90.0) / 100.0;
    vblankCycles = floor(vblankCycles);

    float64_t marigin = (36.0 / (float64_t)laneCountCalc) + 1.0;
    if (vblankCycles > marigin) {
        vblankCycles -= marigin;
        vblankCycles = floor(vblankCycles);
    } else {
        vblankCycles = 0.0;
    }

    if (pD->mstEnabled) {
        /* Adjust for actual number of lanes */
        vblankCycles *= ((float64_t)laneCount / 4.0);
        vblankCycles = floor(vblankCycles);
    }

    if ((uint32_t)vblankCycles > vblankCyclesMax) {
        vblankCycles = (float64_t)vblankCyclesMax;
    }

    return (uint32_t)vblankCycles;
}

static uint32_t calcSdpBlockHblank(const DP_PrivateData* pD,
                                   uint8_t               streamId,
                                   float64_t             targAvgSlots)
{
    const uint16_t hblankCyclesMax = 32767U; /* 15 bits */
    float64_t hblankCycles;
    uint8_t laneCountCalc; /* Number of lanes, used for calculations. */

    const uint8_t laneCount = pD->linkState.laneCount;
    const uint32_t hTotal = pD->videoParameters[streamId].vicParams.hTotal;
    const uint32_t hActive = pD->videoParameters[streamId].vicParams.hActive;
    const uint16_t symbolRate = getSymbolRate(pD->linkState.linkRate);
    const float64_t pixelClock = pD->videoParameters[streamId].vicParams.pxlFreq;

    /* For MST, calculate like there are always 4 lanes. */
    laneCountCalc = (!pD->mstEnabled) ? laneCount : 4U;
    uint32_t init = ((hTotal - hActive) * (uint32_t)symbolRate) / 2U;
    hblankCycles = (float64_t)init / pixelClock;
    hblankCycles = floor(hblankCycles);

    if (pD->mstEnabled) {
        /* Quantize to slot allocation */
        hblankCycles *= (targAvgSlots / 64.0);
        hblankCycles = floor(hblankCycles);
    }

    /* Apply safety margin. */
    hblankCycles = (hblankCycles * 90.0) / 100.0;
    hblankCycles = floor(hblankCycles);

    float64_t marigin = (36.0 / (float64_t)laneCountCalc) + 1.0;
    if (hblankCycles > marigin) {
        hblankCycles -= marigin;
        hblankCycles = floor(hblankCycles);
    } else {
        hblankCycles = 0.0;
    }

    if (pD->mstEnabled) {
        /* Adjust for actual number of lanes */
        hblankCycles *= ((float64_t)laneCount / 4.0);
        hblankCycles = floor(hblankCycles);
    }

    if ((uint32_t)hblankCycles > hblankCyclesMax) {
        hblankCycles = (float64_t)hblankCyclesMax;
    }

    return (uint32_t)hblankCycles;
}

static uint32_t calcByteCount(const DP_PrivateData* pD,
                              uint8_t               streamId)
{

    uint8_t laneCountCalc; /* Number of lanes, used for calculations. */
    float64_t bitsPerPixel = calculateBitsPerComponent(pD, streamId) * 3.0;
    float64_t pixelsPerLane; /* Number of pixels to be distributed per lane */
    float64_t byteCount;

    /* For MST, calculate like there are always 4 lanes. */
    laneCountCalc = (!pD->mstEnabled) ? pD->linkState.laneCount : 4U;
    pixelsPerLane = (float64_t)pD->videoParameters[streamId].vicParams.hActive / (float64_t)laneCountCalc;
    pixelsPerLane = ceil(pixelsPerLane);
    if (DP_PXENC_YCBCR_4_2_0 == pD->videoParameters[streamId].pxEncFormat)
    {
        /* For 4:2:0 mode, each lane will store pairs of pixels consecutively */
        pixelsPerLane = 2.0 * ceil(pixelsPerLane / 2.0);
    }
    byteCount = pixelsPerLane * (float64_t)laneCountCalc * (bitsPerPixel / 8.0);
    byteCount = ceil(byteCount);

    return (uint32_t)byteCount;
}

static uint32_t configureSstVideoParams(DP_PrivateData* pD, uint16_t *validSymF, bool fecEnabled)
{
    const uint8_t framerTuUpdateBits = 16U;
    const uint8_t streamId = 0U; /* SST mode */
    uint32_t retVal;
    uint32_t validSym;
    uint8_t tuSize;
    uint32_t tuVsDiff; /* difference between TU size and VS per TU. 0 used for values > 3. */
    uint32_t lineThresh;
    DP_RegisterTransfer regTransfer = {0U};
    DP_WriteFieldRequest req;
    float64_t overhead = (fecEnabled) ? DP_FEC_OVERHEAD : 0.0;

    retVal = calculateTuParams(pD, overhead, &validSym, validSymF, &tuSize);

    if (CDN_EOK == retVal)
    {
        req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_TU_p);
        req.startBit = 0U;
        req.bitCount = framerTuUpdateBits;
        req.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_FRAMER_TU_P, TU_VALID_SYMBOLS, 0U, validSym) |
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_FRAMER_TU_P, TU_SIZE, 0U, tuSize);

        retVal = DP_WriteField(pD, &req);
    }

    if (CDN_EOK == retVal)
    {
        tuVsDiff = calcTuVsDiff(tuSize, validSym);
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].STREAM_CONFIG_2_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__STREAM_CONFIG_2_P, CFG_TU_VS_DIFF, 0U, tuVsDiff);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        lineThresh = calcLineThresh(pD, validSym);
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].LINE_THRESH_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__LINE_THRESH_P, CFG_ACTIVE_LINE_TRESH, 0U, lineThresh);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

/* Local structure used to configure horizontal and vertical params of stream */
typedef struct HorizontalVerticalParams_t {
    uint32_t backPorch;
    uint32_t frontPorch;
    uint32_t sync;
    uint32_t active;
    uint32_t total;
    DP_SyncPolarity syncPolarity;
} HorizontalVerticalParams;

/* Set of functions used to properly configure horizontal parameters of stream */
static uint32_t configureMsaHorizontal0(DP_PrivateData*                 pD,
                                        uint8_t                         streamId,
                                        const HorizontalVerticalParams* hParams)
{
    DP_RegisterTransfer regTransfer = {0U};

    uint32_t hStart = hParams->sync + hParams->backPorch;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_HORIZONTAL_0_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_0_P, PCK_STUFF_HTOTAL, 0U, hParams->total) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_0_P, PCK_STUFF_HSTART, 0U, hStart);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureMsaHorizontal1(DP_PrivateData*                 pD,
                                        uint8_t                         streamId,
                                        const HorizontalVerticalParams* hParams)
{
    DP_RegisterTransfer regTransfer = {0U};

    uint32_t hSyncPolarity = (DP_SP_ACTIVE_LOW == hParams->syncPolarity) ? 1U : 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_HORIZONTAL_1_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P, PCK_STUFF_HSYNCWIDTH, 0U, hParams->sync) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P, PCK_STUFF_HWIDTH, 0U, hParams->active) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P, PCK_STUFF_HSYNCPOLARITY, 0U, hSyncPolarity);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureDpFrontBackPorch(DP_PrivateData*                 pD,
                                          uint8_t                         streamId,
                                          const HorizontalVerticalParams* hParams)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_FRONT_BACK_PORCH_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRONT_BACK_PORCH_P, BACK_PORCH, 0U, hParams->backPorch) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRONT_BACK_PORCH_P, FRONT_PORCH, 0U, hParams->frontPorch);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureDpHorizontal(DP_PrivateData*                 pD,
                                      uint8_t                         streamId,
                                      const HorizontalVerticalParams* hParams)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_HORIZONTAL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_HORIZONTAL_P, HSYNCWIDTH, 0U, hParams->sync) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_HORIZONTAL_P, HWIDTH, 0U, hParams->active);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureHorizontalParams(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;
    DP_VideoFormatParams* vicParams = &(pD->videoParameters[streamId].vicParams);

    HorizontalVerticalParams hParams;
    hParams.backPorch  = vicParams->hBackPorch;
    hParams.frontPorch = vicParams->hFrontPorch;
    hParams.sync = vicParams->hSync;
    hParams.total = vicParams->hTotal;
    hParams.active = vicParams->hActive;
    hParams.syncPolarity = vicParams->hSyncPolarity;

    retVal = configureMsaHorizontal0(pD, streamId, &hParams);

    if (CDN_EOK == retVal) {
        retVal = configureMsaHorizontal1(pD, streamId, &hParams);
    }

    if (CDN_EOK == retVal) {

        if ((pD->videoParameters[streamId].dscEnable) && (pD->dscConfig[0].splitPanel)) {
            hParams.frontPorch /= 2U;
            hParams.backPorch /= 2U;
            hParams.sync /= 2U;
            hParams.active /= 2U;
        }

        retVal = configureDpFrontBackPorch(pD, streamId, &hParams);
    }

    if (CDN_EOK == retVal) {
        retVal = configureDpHorizontal(pD, streamId, &hParams);
    }

    return retVal;
}

/* Set of functions used to properly configure vertical params of stream -> analog to horizontal */
static uint32_t configureMsaVertical0(DP_PrivateData*                 pD,
                                      uint8_t                         streamId,
                                      const HorizontalVerticalParams* vParams)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t vStart = vParams->sync + vParams->backPorch;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_VERTICAL_0_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_0_P, PCK_STUFF_VSTART, 0U, vStart) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_0_P, PCK_STUFF_VTOTAL, 0U, vParams->total);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureMsaVertical1(DP_PrivateData*                 pD,
                                      uint8_t                         streamId,
                                      const HorizontalVerticalParams* vParams)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t vSyncPolarity = (DP_SP_ACTIVE_LOW == vParams->syncPolarity) ? 1U : 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_VERTICAL_1_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P, PCK_STUFF_VSYNCWIDTH, 0U, vParams->sync) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P, PCK_STUFF_VHEIGHT, 0U, vParams->active) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P, PCK_STUFF_VSYNCPOLARITY, 0U, vSyncPolarity);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureDpVertical0(DP_PrivateData*                 pD,
                                     uint8_t                         streamId,
                                     const HorizontalVerticalParams* vParams)
{
    DP_RegisterTransfer regTransfer = {0U};

    uint32_t vStart = vParams->backPorch + vParams->sync;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VERTICAL_0_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_VERTICAL_0_P, VSTART, 0U, vStart) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_VERTICAL_0_P, VHEIGHT, 0U, vParams->active);
    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureDpVertical1(DP_PrivateData*                 pD,
                                     uint8_t                         streamId,
                                     const HorizontalVerticalParams* vParams)
{
    DP_RegisterTransfer regTransfer = {0U};
    DP_VideoFormatParams* vicParams = &(pD->videoParameters[streamId].vicParams);

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VERTICAL_1_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_VERTICAL_1_P, VTOTAL, 0U, vParams->total);

    if ((0U == (vicParams->vTotal & 0x01U)) && (DP_SM_INTERLACED == vicParams->scanMode)) {
        /* Image is interlaced and has even number of total lines. */
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_VERTICAL_1_P, VTOTAL_EVEN, 0U, 1U);
    }

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureVerticalParams(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;
    DP_VideoFormatParams* vicParams = &(pD->videoParameters[streamId].vicParams);

    HorizontalVerticalParams vParams;
    vParams.backPorch  = vicParams->vBackPorch;
    vParams.frontPorch = 0U; /* unused there */
    vParams.sync = vicParams->vSync;
    vParams.total = vicParams->vTotal;
    vParams.active = vicParams->vActive;
    vParams.syncPolarity = vicParams->vSyncPolarity;

    if (DP_SM_INTERLACED == vicParams->scanMode) {
        vParams.total /= 2U;
        vParams.active /= 2U;
    }

    retVal = configureMsaVertical0(pD, streamId, &vParams);

    if (CDN_EOK == retVal) {
        retVal = configureMsaVertical1(pD, streamId, &vParams);
    }

    if (CDN_EOK == retVal) {
        retVal = configureDpVertical0(pD, streamId, &vParams);
    }

    if (CDN_EOK == retVal) {
        retVal = configureDpVertical1(pD, streamId, &vParams);
    }

    return retVal;
}

/**
 * Sets fractional part of TU_VALID in RAMER_PXL_REPR register, for DSC operation.
 */
static void setDscTuValidFract(uint32_t *reg, uint16_t validSymF)
{
    uint32_t m = (uint32_t)validSymF / 10U;
    uint32_t diff = 100U - m;

    *reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P, DIFF, *reg, diff);
    *reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P, M, *reg, m);
}

/*
 * Get MISC0[7:5] bits correlate with bitsPerComponent and colorDepth
 * Return CDN_EINVAL when bitPerComponent incorrect
 */
static uint32_t getColorDepth(uint8_t bitsPerComponent, uint8_t* colorDepth, uint8_t* miscBpc)
{
    uint32_t retVal = CDN_EOK;

    switch (bitsPerComponent)
    {
    case 6U:
        *colorDepth = 0x01U;
        *miscBpc = 0U;
        break;
    case 8U:
        *colorDepth = 0x02U;
        *miscBpc = 1U;
        break;
    case 10U:
        *colorDepth = 0x04U;
        *miscBpc = 2U;
        break;
    case 12U:
        *colorDepth = 0x08U;
        *miscBpc = 3U;
        break;
    case 16U:
        *colorDepth = 0x10U;
        *miscBpc = 4U;
        break;
    default:
        retVal = CDN_EINVAL;
        break;
    }

    return retVal;
}

static uint8_t getMisc0(const DP_VideoParameters* videoParameters, uint8_t miscBpc)
{
    uint8_t btType = (uint8_t)videoParameters->btType;
    uint8_t misc0;

    switch (videoParameters->pxEncFormat)
    {
    /* values are being set according to DisplayPort standard. */
    case DP_PXENC_PXL_RGB:
    case DP_PXENC_Y_ONLY:
        misc0 = (miscBpc << 5);
        break;
    case DP_PXENC_YCBCR_4_4_4:
        misc0 = (6U << 1) | (btType << 4) | (miscBpc << 5);
        break;
    case DP_PXENC_YCBCR_4_2_2:
        misc0 = (5U << 1) | (btType << 4) | (miscBpc << 5);
        break;
    case DP_PXENC_YCBCR_4_2_0:
        misc0 = 0U;      /* values are ignored for 4:2:0 */
        break;
    default:
        misc0 = 0U;
        break;
    }

    if (videoParameters->forceMiscIgnoreBit) {
        misc0 = 0U;
    }

    return misc0;
}

static uint8_t getMisc1(const DP_VideoParameters* videoParameters)
{
    uint8_t misc1;
    const DP_VideoFormatParams* vicParams;

    if ((DP_PXENC_YCBCR_4_2_0 == videoParameters->pxEncFormat) ||
        (videoParameters->forceMiscIgnoreBit)) {
        /* When this bit has been set, VSC SDP has to be used instead, to */
        /* indicate pixel encoding / colorimetry. This is required for 4:2:0. */
        misc1 = (1U << 6);
    } else {
        vicParams = &(videoParameters->vicParams);
        misc1 = 0U;

        if (DP_PXENC_Y_ONLY == videoParameters->pxEncFormat) {
            misc1 |= (1U << 7);
        }

        if ((0U == (vicParams->vTotal & 0x01U)) &&
            (DP_SM_INTERLACED == vicParams->scanMode)) {
            /* Image is interlaced and has even number of total lines. */
            misc1 |= 1U;
        }
    }
    return misc1;
}

static uint32_t configureColorimetry(DP_PrivateData* pD, uint8_t streamId, uint16_t validSymF)
{
    DP_VideoParameters* videoParameters = &(pD->videoParameters[streamId]);
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};
    uint8_t colorDepth;
    uint8_t misc0;
    uint8_t misc1;
    uint8_t miscBpc;

    retVal = getColorDepth(videoParameters->bitsPerSubpixel, &colorDepth, &miscBpc);

    if (CDN_EOK == retVal)
    {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_FRAMER_PXL_REPR_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P, COLOR_DEPTH, 0U, colorDepth) |
                          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P, PXL_ENC_FORMAT, 0U, videoParameters->pxEncFormat);

        if ((!pD->mstEnabled) && (pD->videoParameters[streamId].dscEnable)) {
            setDscTuValidFract(&(regTransfer.val), validSymF);
        }

        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        misc0 = getMisc0(videoParameters, miscBpc);
        misc1 = getMisc1(videoParameters);

        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_MISC_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_MISC_P, MSA_MISC0, 0U,  misc0) |
                          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_MISC_P, MSA_MISC1, 0U,  misc1);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint32_t configureByteCount(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;

    DP_RegisterTransfer regTransfer = {0U};
    uint32_t bytesInChunk;
    uint8_t chunkCount = 1U;
    uint32_t byteCount; /* Byte Count parameter calculated, when DSC is not being used. */

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_BYTE_COUNT_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);

    if (pD->videoParameters[streamId].dscEnable)
    {
        if (pD->mstEnabled) {
            bytesInChunk = (pD->dscConfig[streamId].chunkSize / 4U) + 1U;
        } else {
            bytesInChunk = (pD->dscConfig[streamId].chunkSize / pD->linkState.laneCount) + 1U;
        }

        if (pD->dscConfig[streamId].splitPanel) {
            chunkCount = 2U;
        }

        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BYTE_COUNT_P, BYTE_COUNT, 0U, bytesInChunk * chunkCount) |
                          CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BYTE_COUNT_P, BYTES_IN_CHUNK, 0U, bytesInChunk);
    } else {
        byteCount = calcByteCount(pD, streamId);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BYTE_COUNT_P, BYTE_COUNT, 0U, byteCount);
    }

    retVal = DP_WriteRegister(pD, &regTransfer);

    return retVal;
}

static uint32_t configureBndHsync2Vsync(DP_PrivateData* pD, uint8_t streamId)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].BND_HSYNC2VSYNC_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_vif_ctrl[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__BND_HSYNC2VSYNC_P, IP_VIF_BYPASS, 0U, 1U);

    if (DP_SM_INTERLACED == pD->videoParameters[streamId].vicParams.scanMode) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__BND_HSYNC2VSYNC_P, IP_DTCT_WIN, 0U, 0x20U)
                           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__BND_HSYNC2VSYNC_P, IP_DET_EN, 0U, 1U);
    }

    regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__BND_HSYNC2VSYNC_P,
                                     IP_VIF_ALIGNMENT,
                                     0U,
                                     (uint8_t)pD->videoParameters[streamId].alignment);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureHsync2VsyncPolCtrl(DP_PrivateData* pD, uint8_t streamId)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].HSYNC2VSYNC_POL_CTRL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_vif_ctrl[0]);

    if (DP_SP_ACTIVE_LOW == pD->videoParameters[streamId].vicParams.hSyncPolarity) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__HSYNC2VSYNC_POL_CTRL_P, HPOL, 0U, 1U);
    }

    if (DP_SP_ACTIVE_LOW == pD->videoParameters[streamId].vicParams.vSyncPolarity) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__HSYNC2VSYNC_POL_CTRL_P, VPOL, 0U, 1U);
    }

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureDpFramerPxlRepr(DP_PrivateData* pD, uint8_t streamId)
{
    DP_RegisterTransfer regTransfer = {0U};
    DP_VideoFormatParams* vicParams = &(pD->videoParameters[streamId].vicParams);

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_FRAMER_SP_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);

    if (DP_SP_ACTIVE_LOW == vicParams->hSyncPolarity) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_SP_P, HSP, 0U, 1U);
    }

    if (DP_SP_ACTIVE_LOW == vicParams->vSyncPolarity) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_SP_P, VSP, 0U, 1U);
    }

    if (DP_SM_INTERLACED == vicParams->scanMode) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_SP_P, INTERLACE_EN, 0U, 1U);
    }

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t setVerticalBlankingInterlaced(DP_PrivateData* pD, uint8_t streamId)
{
    const uint8_t vbIdInterlacedBitPos = 2U;
    uint32_t addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VB_ID_p);
    addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    DP_WriteFieldRequest req;

    req.addr = addr;
    req.startBit = vbIdInterlacedBitPos;
    req.bitCount = 1U;
    req.val = 0U;

    if (DP_SM_INTERLACED == pD->videoParameters[streamId].vicParams.scanMode) {
        req.val = 1U << vbIdInterlacedBitPos;
    }

    return DP_WriteField(pD, &req);

}

static uint32_t configureSync(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;

    retVal = configureBndHsync2Vsync(pD, streamId);

    if (CDN_EOK == retVal) {
        retVal = configureHsync2VsyncPolCtrl(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        retVal = configureDpFramerPxlRepr(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        retVal = setVerticalBlankingInterlaced(pD, streamId);
    }

    return retVal;
}

static uint32_t configureSdpBlocking(DP_PrivateData* pD, uint8_t streamId, bool fecEnabled)
{
    uint32_t retVal;

    DP_RegisterTransfer regTransfer = {0U};
    float64_t targAvgSlots = 0.0; /* calculated target average slots for stream. Applicable only for MST. */
    uint32_t vblankSdpCycles; /* block SDP scheduling after specified cycles after BS/VB-ID during vertical blank. */
    uint32_t hblankSdpCycles; /* block SDP scheduling after specified cycles after BS/VB-ID during horizontal blank. */

    if (pD->mstEnabled) {
        targAvgSlots = calculateTargetAverageSlots(pD, streamId, fecEnabled);
    }

    vblankSdpCycles = calcSdpBlockVblank(pD, streamId, targAvgSlots);
    hblankSdpCycles = calcSdpBlockHblank(pD, streamId, targAvgSlots);

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_BLOCK_SDP_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BLOCK_SDP_P, BS_SDP_STOP_OVR_EN, 0U, 1U)
                      | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BLOCK_SDP_P, BS_SDP_STOP_ACTIVE, 0U, hblankSdpCycles)
                      | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_BLOCK_SDP_P, BS_SDP_STOP_BLANK, 0U, vblankSdpCycles);

    retVal = DP_WriteRegister(pD, &regTransfer);

    return retVal;
}

static uint32_t startStream(DP_PrivateData* pD, uint8_t streamId)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].STREAM_CONFIG_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__STREAM_CONFIG_P, STREAM_EN, 0U, 1U);

    return DP_WriteRegister(pD, &regTransfer);
}

static uint32_t configureVicParams(DP_PrivateData*           pD,
                                   uint8_t                   streamId,
                                   const DP_VideoParameters* videoParameters)
{
    uint32_t retVal = CDN_EOK;
    uint8_t i;
    uint16_t validSymF = 0U; /* valid symbols per TU - fractional part (thousandth parts, 0 - 999) */

    uint32_t (*config[5]) (DP_PrivateData*, uint8_t) = {
        startStream,     /* Starting stream early, proper place to be determined. Temporary solution. */
        configureHorizontalParams,
        configureVerticalParams,
        configureByteCount,
        configureSync
    };

    pD->videoParameters[streamId] = *videoParameters;

    if (!pD->mstEnabled) {
        retVal = configureSstVideoParams(pD, &validSymF, pD->fecEnabled);
    }

    if (CDN_EOK == retVal) {
        for (i = 0U; i < (sizeof(config) / sizeof(config[0])); i++) {
            retVal = config[i](pD, streamId);
            if (CDN_EOK != retVal) {
                break;
            }
        }
    }

    if (CDN_EOK == retVal) {
        retVal = configureColorimetry(pD, streamId, validSymF);
    }

    if (CDN_EOK == retVal) {
        retVal = configureSdpBlocking(pD, streamId, pD->fecEnabled);
    }

    return retVal;
}

/* Check correctness of some video parameters. */
static uint32_t checkVideoParams(const DP_VideoParameters *parameters)
{
    uint32_t retVal = CDN_EOK;

    if (DP_PXENC_PXL_RGB  == parameters->pxEncFormat) {
        /* For RGB encoding, active video width has to be multiple of 2. */
        if (0U != (parameters->vicParams.hActive % 2U)) {
            retVal = CDN_EINVAL;
        }
    } else {
        /* For other encodings, active video width has to be multiple of 16. */
        if (0U != (parameters->vicParams.hActive % 16U)) {
            retVal = CDN_EINVAL;
        }
    }

    return retVal;
}

static uint32_t checkPhyInit(DP_PrivateData* pD)
{
    uint32_t retVal;

    /* In case PHY was initialized directly through PHY driver, valid link */
    /* parameters will only be known to PHY driver - need to fetch them first. */
    retVal = fetchLinkState(pD);
    if (CDN_EOK == retVal) {
        if (0U == pD->linkState.laneCount) {
            /* Not even PHY initialization was done since PHY driver's init. */
            retVal = CDN_ENOENT;
        }
    }

    return retVal;
}
static uint32_t setVifClock(DP_PrivateData* pD, uint32_t streamId, bool enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t reg;
    uint32_t retVal;
    uint8_t clockEnable = (enable) ? 1U : 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_dptx_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_RSTN_EN, reg, clockEnable);
            break;
        case 1U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN1, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_RSTN_EN1, reg, clockEnable);
            break;
        case 2U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN2, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_RSTN_EN2, reg, clockEnable);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN3, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_RSTN_EN3, reg, clockEnable);
            break;
        }

        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint32_t getVifClock(DP_PrivateData* pD, uint32_t streamId, bool* enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t reg;
    uint32_t retVal;
    uint32_t clockEnable = 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_dptx_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN, reg);
            break;
        case 1U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN1, reg);
            break;
        case 2U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN2, reg);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_DPTX_CAR_P, CFG_DPTX_VIF_CLK_EN3, reg);
            break;
        }

        if (clockEnable == 0U) {
            *enable = false;
        } else {
            *enable = true;
        }

    }

    return retVal;
}

static uint32_t setAifClock(DP_PrivateData* pD, uint32_t streamId, bool enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t reg;
    uint32_t retVal;
    uint8_t clockEnable = (enable) ? 1U : 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_aif_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_RSTN_EN, reg, clockEnable);
            break;
        case 1U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN1, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_RSTN_EN1, reg, clockEnable);
            break;
        case 2U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN2, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_RSTN_EN2, reg, clockEnable);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN3, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_RSTN_EN3, reg, clockEnable);
            break;
        }

        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint32_t getAifClock(DP_PrivateData* pD, uint32_t streamId, bool* enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t reg;
    uint32_t retVal;
    uint32_t clockEnable = 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_aif_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);
    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN, reg);
            break;
        case 1U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN1, reg);
            break;
        case 2U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN2, reg);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_AIF_CAR_P, SOURCE_AIF_PKT_CLK_EN3, reg);
            break;

        }

        if (clockEnable == 0U) {
            *enable = false;
        } else {
            *enable = true;
        }

    }

    return retVal;
}

static uint32_t setPktClock(DP_PrivateData* pD, uint32_t streamId, bool enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t retVal;
    uint32_t reg;
    uint8_t clockEnable = (enable) ? 1U : 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_pkt_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_RSTN_EN, reg, clockEnable);
            break;
        case 1U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN1, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_RSTN_EN1, reg, clockEnable);
            break;
        case 2U:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN2, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_RSTN_EN2, reg, clockEnable);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN3, reg, clockEnable);
            reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_RSTN_EN3, reg, clockEnable);
            break;

        }

        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint32_t getPktClock(DP_PrivateData* pD, uint32_t streamId, bool *enable)
{
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t retVal;
    uint32_t reg;
    uint32_t clockEnable = 0U;

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_pkt_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;

        switch (streamId) {
        case 0U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN, reg);
            break;
        case 1U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN1, reg);
            break;
        case 2U:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN2, reg);
            break;
        /*
         * 'Default' statement is called only for 'streamId' = 3U. 'streamId' is in range <0,3> and
         * out of range should be checked by sanity function.
         */
        default:
            clockEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_PKT_CAR_P, SOURCE_PKT_DATA_CLK_EN3, reg);
            break;
        }

        if (clockEnable == 0U) {
            *enable = false;
        } else {
            *enable = true;
        }

    }

    return retVal;
}

/**
 * Set FEC ready in sink in DPCD registers
 */
static uint32_t sinkSetFecReady(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;

    DP_DpcdTransfer transfer;
    uint8_t dpcdBuff[1];
    uint8_t fecCaps;

    transfer.addr = DP_DPCD_FEC_CAPABILITY;
    transfer.size = 1U;
    transfer.buff = dpcdBuff;

    retVal = DP_ReadDpcd(pD, &transfer);

    if (CDN_EOK == retVal) {
        fecCaps = transfer.buff[0];
        if ((fecCaps & DP_DPCD_FEC_CAPABILITY_FEC_CAPABLE_MASK) == 0U) {
            retVal = CDN_ENOTSUP;
        }
    }

    if (CDN_EOK == retVal) {
        transfer.addr = DP_DPCD_FEC_CONFIGURATION;
        transfer.size = 1U;
        retVal = DP_ReadDpcd(pD, &transfer);
    }

    if (CDN_EOK == retVal)
    {
        if (enable) {
            transfer.buff[0] |= (uint8_t)DP_DPCD_FEC_CONFIGURATION_FEC_READY;
        } else {
            transfer.buff[0] &= ~((uint8_t)DP_DPCD_FEC_CONFIGURATION_FEC_READY);
        }

        transfer.addr = DP_DPCD_FEC_CONFIGURATION;
        transfer.size = 1U;
        retVal = DP_WriteDpcd(pD, &transfer);
    }

    return retVal;
}

static uint32_t isSstVideoOn(DP_PrivateData* pD, bool *isOn)
{
    uint32_t retVal;

    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (CDN_EOK == retVal) {
        if ((regTransfer.val & MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P__NO_VIDEO_MASK) == 0U) {
            *isOn = true;
        } else {
            *isOn = false;
        }
    }

    return retVal;
}

static uint32_t checkIfCanSetFec(DP_PrivateData* pD, bool fecEnable)
{
    bool isOn;
    uint32_t retVal;

    retVal = DP_SetFecEnableSF(pD);

    if (retVal == CDN_EOK) {
        if (!pD->mstEnabled) {
            retVal = isSstVideoOn(pD, &isOn);
        } else {
            retVal = DP_MST_IsAnyVideoOn(pD, &isOn);
        }
    }

    if (retVal == CDN_EOK) {
        if (isOn) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Video must be disabled to change FEC configuration\n");
            retVal = CDN_EIO;
        }
    }

    if (CDN_EOK == retVal) {
        if (fecEnable == pD->fecEnabled) {
            /* Nothing to do. New setting equals current setting. */
            retVal = CDN_EAGAIN;
        }
    }

    return retVal;
}

static uint32_t configureSdpBlockingForStreams(DP_PrivateData* pD, bool fecEnable)
{
    uint32_t retVal = CDN_EOK;
    uint8_t i;

    if (pD->mstEnabled) {
        for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {

            if (pD->streamEnabled[i]) {
                retVal = configureSdpBlocking(pD, i, fecEnable);
            }

            if (CDN_EOK != retVal) {
                break;
            }
        }
    } else {
        retVal = configureSdpBlocking(pD, 0U, fecEnable);
    }

    return retVal;
}

/*Set numerator and diffrence beetween denominator and numerator of ratio.
 * It describes valid symbols distribution. Only if DSC is enabled */
static uint32_t setValidSymDistRatio(DP_PrivateData* pD, uint16_t validSymF)
{
    DP_WriteFieldRequest req = {0U};

    req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_FRAMER_PXL_REPR_p);
    req.startBit = MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P__M_SHIFT;
    /* bit count is calculated, taking into consideration gap between fields. */
    req.bitCount = (MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P__DIFF_SHIFT -
                    MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P__M_SHIFT) +
                   MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__DP_FRAMER_PXL_REPR_P__DIFF_WIDTH;

    setDscTuValidFract(&(req.val), validSymF);

    return DP_WriteField(pD, &req);
}

static uint32_t setFecEnable(DP_PrivateData* pD, bool fecEnable)
{
    uint32_t retVal = CDN_EOK;
    uint16_t validSymF;
    DP_WriteFieldRequest req = {0U};

    bool sstAndVideoParamsSet = (!pD->mstEnabled) && (pD->sstVicSet);
    bool dscEnable = pD->videoParameters[0].dscEnable;

    if (sstAndVideoParamsSet) {
        retVal = configureSstVideoParams(pD, &validSymF, fecEnable);

        if ((CDN_EOK == retVal) && (dscEnable)) {
            retVal = setValidSymDistRatio(pD, validSymF);
        }
    }

    if (CDN_EOK == retVal) {
        retVal = configureSdpBlockingForStreams(pD, fecEnable);
    }

    if (CDN_EOK == retVal) {

        uint32_t reqValue = (fecEnable) ? 1U : 0U;

        req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_FEC_CTRL_p);
        req.startBit = MHDP__MHDP_APB_REGS__DPTX_FEC_CTRL_P__CFG_FEC_EN_SHIFT;
        req.bitCount = MHDP__MHDP_APB_REGS__DPTX_FEC_CTRL_P__CFG_FEC_EN_WIDTH;
        req.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DPTX_FEC_CTRL_P, CFG_FEC_EN, 0U, reqValue);
        retVal = DP_WriteField(pD, &req);
    }

    return retVal;
}

/**
 * Set FEC ready in source register
 */
static uint32_t sourceSetFecReady(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;

    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_FEC_CTRL_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (CDN_EOK == retVal)
    {
        uint32_t reg = (enable) ? 1U : 0U;

        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DPTX_FEC_CTRL_P,
                                        CFG_FEC_READY,
                                        regTransfer.val,
                                        reg);
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_FEC_CTRL_p);

        retVal = DP_WriteRegister(pD, &regTransfer);
    }
    return retVal;
}

/**
 * Get value of KEEP_ALIVE register and update it in pD.
 * Return true, if it has changed.
 */
static bool updateAlive(DP_PrivateData* pD)
{
    uint32_t readVal;
    uint8_t aliveVal;
    bool updated = false;

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.KEEP_ALIVE_p);
    aliveVal = (uint8_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__KEEP_ALIVE_P, KEEP_ALIVE_CNT, readVal));
    if (aliveVal != pD->lastAlive)
    {
        updated = true;
        pD->lastAlive = aliveVal;
    }
    return updated;
}

/**
 * Reads signature and functionality registers
 */

static void updateSignatures(DP_PrivateData* pD)
{
    uint32_t readVal;

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.CDNS_DID_p);
    pD->hwConfig.ipPartNumber = readVal;

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.CDNS_RID0_p);
    pD->hwConfig.ipVersion = (uint16_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_RID0_P, IP_VERSION, readVal);

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.CDNS_RID1_p);
    pD->hwConfig.phyVersion = (uint16_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_RID1_P, PHY_VERSION, readVal);
    pD->hwConfig.auxVersion = (uint16_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_RID1_P, AUX_VERSION, readVal);
}

static void updateHardwareConfiguration(DP_PrivateData* pD)
{
    uint32_t readVal;

    updateSignatures(pD);

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.CDNS_CFGS0_p);
    pD->hwConfig.mainConfigType = (uint8_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, IP_NUMBER_CONFIGURATION, readVal);
    pD->hwConfig.ipFamilyCode = (uint8_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, IP_NUMBER_FAMILY, readVal);
    pD->hwConfig.dscSupport = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, DSC_SUPPORT, readVal) > 0U);
    pD->hwConfig.asfSupport = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, ASF_SUPPORT, readVal) > 0U);
    pD->hwConfig.videoStreams = (uint8_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, VIDEO_STREAM_NUMBER, readVal);
    pD->hwConfig.audioStreams = (uint8_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS0_P, AUDIO_STREAM_NUMBER, readVal);

    readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.CDNS_CFGS1_p);
    pD->hwConfig.phyType = (uint16_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS1_P, PHY_NUMBER, readVal);
    pD->hwConfig.auxType = (uint16_t) CPS_FLD_READ(MHDP__MHDP_APB_REGS__CDNS_CFGS1_P, AUX_NUMBER, readVal);
}

static void echoUint32(DP_PrivateData* pD, uint32_t message, DP_BusType busType)
{
    messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_TEST_ECHO);
    messageWriteUint32(pD, message);
    messageFinish(pD);
    messageTransmit(pD, busType);
    messageReceive(pD, busType);
}

static uint32_t checkEchoUint32(DP_PrivateData* pD, uint32_t message)
{
    uint32_t retVal = CDN_EOK;
    uint32_t readVal;
    uint8_t headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_TEST_ECHO);

    if (0U == headerMatching) {
        retVal = CDN_ENOEXEC;
    }

    if (CDN_EOK == retVal) {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint32(pD, &readVal);

        if (message != readVal) {
            retVal = CDN_EIO;
        }
    }

    return retVal;
}

static void echoBuffer(DP_PrivateData* pD, const uint8_t* message, uint16_t messageSize, DP_BusType busType)
{
    messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_TEST_ECHO);
    messageWriteBuffer(pD, message, messageSize);
    messageFinish(pD);
    messageTransmit(pD, busType);
    messageReceive(pD, busType);
}

static uint32_t checkEchoBuffer(DP_PrivateData* pD,
                                const uint8_t*  message,
                                uint8_t*        response,
                                uint16_t        messageSize)
{
    uint16_t readLen;
    uint32_t retVal = CDN_EOK;
    uint8_t headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_TEST_ECHO);

    if (0U == headerMatching) {
        retVal = CDN_ENOEXEC;
    }

    if (CDN_EOK == retVal) {
        uint8_t i;

        messageGetHeader(pD, NULL, NULL, &readLen);
        messageReadBuffer(pD, response, readLen);

        if (readLen != messageSize) {
            retVal = CDN_EIO;
        } else {
            for (i = 0U; i < messageSize; i++) {
                if (message[i] != response[i]) {
                    retVal = CDN_EIO;
                    break;
                }
            }
        }
    }

    return retVal;
}

static uint32_t waitFecStatus(DP_PrivateData* pD, bool fecBusy)
{
    uint32_t retVal;
    uint32_t actualFecBusy;
    uint32_t expectedFecBusy = (fecBusy) ? 1U : 0U;

    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_FEC_STATUS_p);

    do {
        retVal = DP_ReadRegister(pD, &regTransfer);
        actualFecBusy = CPS_FLD_READ(MHDP__MHDP_APB_REGS__DPTX_FEC_STATUS_P, FEC_BUSY, regTransfer.val);
    } while ((retVal == CDN_EOK) && (actualFecBusy != expectedFecBusy));

    return retVal;
}

static uint32_t setCustomPatternSF(const DP_PrivateData* pD, const uint8_t* customPattern)
{
    uint32_t retVal = DP_SetCustomPatternSF(pD);
    if (CDN_EOK == retVal) {
        if (NULL == customPattern) {
            retVal = CDN_EINVAL;
        }
    }

    return retVal;
}

static uint32_t configurePhy(DP_PrivateData* pD, const DP_LinkState* linkParams)
{
    uint32_t retVal;
    uint8_t i;
    /* 1:1 conversion from DP driver's structure to PHY driver's structure */
    DP_SD0801_LinkState sdState;
    toSdLinkState(linkParams, &sdState);

    retVal = DP_SD0801_SetLinkRate(pD->phyPd, &sdState);
    if (CDN_EOK == retVal) {
        retVal = DP_SD0801_EnableLanes(pD->phyPd, &sdState);
    }

    if (CDN_EOK == retVal)
    {
        for (i = 0U; i < (sdState.laneCount); i++) {
            retVal = DP_SD0801_ConfigLane(pD->phyPd, i, &sdState);
            if (CDN_EOK != retVal) {
                /* Stop execution in case of error */
                break;
            }
        }
    }
    return retVal;
}

static uint32_t updateAssrEnable(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal = CDN_EOK;
    DP_DpcdTransfer transfer = {0U};
    uint8_t dpcdBuffer[1];

    if (pD->sinkCaps.assr) {
        transfer.addr = DP_EDP_CONF_REG;
        transfer.size = 1U;
        transfer.buff = dpcdBuffer;

        retVal = DP_ReadDpcd(pD, &transfer);

        if (CDN_EOK == retVal) {
            bool actualEnable = (0U == (dpcdBuffer[0] & DP_ASSR_ENABLE_MASK));

            bool setEnable = (enable) && (!actualEnable);
            bool setDisable = (!enable) && (actualEnable);

            if ((setEnable) || (setDisable))
            {
                /* Value of DPCD register has to be updated */
                transfer.addr = DP_EDP_CONF_REG;
                transfer.size = 1U;

                if (enable) {
                    dpcdBuffer[0] |= DP_ASSR_ENABLE_MASK;
                } else {
                    dpcdBuffer[0] &= ~DP_ASSR_ENABLE_MASK;
                }

                retVal = DP_WriteDpcd(pD, &transfer);
            }
        }
    }

    return retVal;
}

static DP_AuxStatus getAuxStatus(uint8_t code)
{
    DP_AuxStatus result;

    static const DP_AuxStatus auxStatus[4] = {
        DP_AUX_ACK,
        DP_AUX_NACK,
        DP_AUX_DEFER,
        DP_AUX_SINK_ERROR
    };

    if (code <= 3U) {
        result = auxStatus[code];
    } else {
        result = DP_AUX_UNKNOWN_ERROR;
    }

    return result;
}

static DP_I2cStatus getI2cStatus(uint8_t code)
{
    DP_I2cStatus result;

    static const DP_I2cStatus i2cStatus[3] = {
        DP_I2C_ACK,
        DP_I2C_NACK,
        DP_I2C_DEFER
    };

    if (code <= 2U) {
        result = i2cStatus[code];
    } else {
        result = DP_I2C_UNKNOWN_ERROR;
    }

    return result;
}

static void clearRxTxBuffers(DP_PrivateData* pD)
{
    uint32_t i;

    pD->txi = 0U;
    pD->rxi = 0U;
    pD->txEnable = 0U;
    pD->rxEnable = 0U;

    for (i = 0U; i < sizeof(pD->txBuffer); i++) {
        pD->txBuffer[i] = 0U;
    }

    for (i = 0U; i < sizeof(pD->rxBuffer); i++) {
        pD->rxBuffer[i] = 0U;
    }
}

static void clearCapabilities(DP_PrivateData* pD) {
    clearSourceCapabilities(&(pD->sourceCaps));
    pD->sourceCapsStored = 0U;

    clearSinkCapabilities(&(pD->sinkCaps));
    pD->sinkCapsStored = 0U;
}

static uint32_t loadFirmwareSF(const DP_PrivateData* pD, const DP_FirmwareImage* image)
{
    uint32_t retVal;

    retVal = DP_LoadFirmwareSF(pD, image);

    if (CDN_EOK == retVal) {
        /* Size of instruction memory and data memory must be divisible by 4. */
        if ((NULL == image->iMem) ||
            (NULL == image->dMem) ||
            (0U != (image->iMemSize % 4U)) ||
            (0U != (image->dMemSize % 4U)))
        {
            retVal = CDN_EINVAL;
        }
    }

    return retVal;
}

static void writeFirmwareImageToMemory(volatile uint32_t* address, const uint8_t* image, uint32_t memorySize)
{
    uint32_t i;
    uint32_t reg;
    volatile uint32_t* localAddress = address;

    for (i = 0U; i < memorySize; i += 4U) {

        reg = ((uint32_t)image[i] << 0)  |
              ((uint32_t)image[i + 1U] << 8)  |
              ((uint32_t)image[i + 2U] << 16) |
              ((uint32_t)image[i + 3U] << 24);

        CPS_REG_WRITE(localAddress, reg);

        localAddress++;

    }
}

/**
 * Check, if PHY ID (version) is on list of versions supported by the driver.
 */
static bool checkPhyVersion(uint16_t phyVer)
{
    /* PHY version ID is 16-bit. */
    static const uint16_t supportedPhys[] = {
        DP_PHY_VERSION_1100,
        DP_PHY_VERSION_1200
    };
    bool supported = false;
    uint32_t i;

    for (i = 0; i < (sizeof(supportedPhys) / sizeof(uint16_t)); i++)
    {
        if (supportedPhys[i] == phyVer)
        {
            supported = true;
        }
    }

    return supported;
}

static uint32_t checkHardwareConfiguration(const DP_HardwareConfig* hwConfig)
{
    uint32_t retVal = CDN_EOK;

    if ((hwConfig->ipPartNumber != DP_IP_PART_NUMBER) ||
        (hwConfig->ipVersion != DP_IP_VERSION) ||
        (hwConfig->auxVersion != DP_AUX_VERSION) ||
        (!checkPhyVersion(hwConfig->phyVersion)))
    {
        retVal = CDN_ENOTSUP;
    }

    return retVal;
}

static void clearPrivateData(DP_PrivateData* pD)
{
    uint8_t i;

    pD->running = 0U;
    pD->busType = DP_BUS_TYPE_APB;
    pD->tmp = 0U;

    clearRxTxBuffers(pD);

    clearCapabilities(pD);

    clearLinkState(&(pD->linkState));

    (void)updateAlive(pD);

    pD->mstEnabled = false;
    pD->fecEnabled = false;
    pD->sstVicSet = false;

    for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {
        /* remaining parameters may be auto-initialized to '0' */
        (void)memset(&(pD->videoParameters[i]), 0, sizeof(DP_VideoParameters));
        pD->videoParameters[i].pxEncFormat = DP_PXENC_PXL_RGB;
        pD->streamEnabled[i] = false;
        pD->sinkList[i].port = NULL;

        uint8_t sdpIdx;

        for (sdpIdx = 0U; sdpIdx < DP_MAX_NUMBER_OF_SDPS; sdpIdx++) {
            pD->sdpPacketType[i][sdpIdx] = 0U;
        }
    }

    for (i = 0U; i < DP_NUMBER_OF_DSC_ENCODERS; i++) {
        pD->dscConfig[i] = (DP_DscConfigFull){0U};
    }
}

static uint32_t setCipherClockConfig(DP_PrivateData* pD, uint8_t clockEnable)
{
    uint32_t retVal;
    uint32_t reg;
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_cipher_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {

        reg = regTransfer.val;

        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CIPHER_CAR_P, SOURCE_CIPHER_CHAR_CLK_EN, reg, clockEnable);
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CIPHER_CAR_P, SOURCE_CIPHER_CHAR_CLK_RSTN_EN, reg, clockEnable);
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CIPHER_CAR_P, SOURCE_CIPHER_SYS_CLK_EN, reg, clockEnable);
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CIPHER_CAR_P, SOURCE_CIPHER_SYSTEM_CLK_RSTN_EN, reg, clockEnable);

        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

static uint32_t setCryptoClockConfig(DP_PrivateData* pD, uint8_t clockEnable)
{
    uint32_t retVal;
    uint32_t reg;
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_crypto_car_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {

        reg = regTransfer.val;

        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CRYPTO_CAR_P, SOURCE_CRYPTO_SYS_CLK_EN, reg, clockEnable);
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SOURCE_CRYPTO_CAR_P, SOURCE_CRYPTO_SYS_CLK_RSTN_EN, reg, clockEnable);

        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

/* API FUNCTIONS */

uint32_t DP_Probe(const DP_Config* config, uint32_t* memReq)
{
    uint32_t retVal = CDN_EOK;
    if (CDN_EOK != DP_ProbeSF(config, memReq)) {
        retVal = CDN_EINVAL;
    } else {
        *memReq = (uint32_t)(sizeof(DP_PrivateData));
    }
    return retVal;
}

uint32_t DP_Init(DP_PrivateData* pD, const DP_Config* config, const DP_Callbacks* callbacks)
{
    uint32_t retVal;
    static AUX_drm_dp aux;

    retVal = DP_InitSF(pD, config, callbacks);

    if (CDN_EOK == retVal) {

        pD->regBase = config->regBase;
        pD->regBaseSapb = config->regBaseSapb;

        updateHardwareConfiguration(pD);
        retVal = checkHardwareConfiguration(&(pD->hwConfig));
    }

    if (CDN_EOK == retVal) {

        pD->cb = *callbacks;

        clearPrivateData(pD);

        /* TODO this should me moved to topology manager */
        aux.pD = pD;
        DP_MST_MgrInit(&pD->mstTopMgr, &aux, 16);

    }

    return retVal;
}

void DP_Isr(DP_PrivateData* pD)
{
    if (CDN_EOK == DP_IsrSF(pD))
    {
        if (NULL != pD->cb.event) {
            (*(pD->cb.event))(pD);
        }
    }
}

uint32_t DP_Start(DP_PrivateData* pD)
{
    uint32_t reg = 0U;
    uint32_t retVal = DP_StartSF(pD);

    if (CDN_EOK == retVal) {
        /* Disable the mailbox interrupt. Else it will keep interrupting */
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.MAILBOX_INT_MASK_p, ~(0U));

        reg  = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_MAILBOX_INTR_MASK, 0U, 0U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_SW_INTR_MASK, 0U, 0U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_PIF_INTR_MASK, 0U, 0U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_CEC_INTR_MASK, 0U, 0U);

        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.APB_INT_MASK_p, reg);
    }

    return retVal;
}

uint32_t DP_Stop(DP_PrivateData* pD)
{
    uint32_t reg = 0U;
    uint32_t retVal = DP_StopSF(pD);

    if (CDN_EOK == retVal) {
        reg  = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_MAILBOX_INTR_MASK, 0U, 1U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_SW_INTR_MASK, 0U, 1U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_PIF_INTR_MASK, 0U, 1U)
               | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_INT_MASK_P, APB_CEC_INTR_MASK, 0U, 1U);

        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.APB_INT_MASK_p, reg);
    }

    return retVal;
}

uint32_t DP_Destroy(const DP_PrivateData* pD)
{
    return DP_DestroySF(pD);
}

uint32_t DP_SetPhyPd(DP_PrivateData* pD, DP_SD0801_PrivateData* phyPd)
{
    uint32_t retVal;

    retVal = DP_SetPhyPdSF(pD, phyPd);

    if (CDN_EOK == retVal)
    {
        pD->phyPd = phyPd;
    }

    return retVal;
}

uint32_t DP_LoadFirmware(const DP_PrivateData* pD, const DP_FirmwareImage* image)
{
    uint32_t retVal;
    uint32_t reg;

    retVal = loadFirmwareSF(pD, image);

    if (CDN_EOK == retVal)
    {
        /* Stall the uCPU and release reset to load FW */
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__APB_CTRL_P, APB_XT_RUNSTALL, 0U, 1U);
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.APB_CTRL_p, reg);

        volatile uint32_t* address = pD->regBase->mhdp_apb_regs.IRAM_REG_p;
        writeFirmwareImageToMemory(address, image->iMem, image->iMemSize);

        address = pD->regBase->mhdp_apb_regs.DRAM_REG_p;
        writeFirmwareImageToMemory(address, image->dMem, image->dMemSize);
    }

    return retVal;
}

uint32_t DP_StartUcpu(const DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_StartUcpuSF(pD);
    if (CDN_EOK == retVal)
    {
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.APB_CTRL_p, 0);
    }

    return retVal;
}

uint32_t DP_TestEcho(DP_PrivateData* pD,
                     uint32_t        message,
                     DP_BusType      busType)
{
    uint32_t retVal = DP_TestEchoSF(pD, busType);

    if (CDN_EOK == retVal)
    {
        echoUint32(pD, message, busType);
        retVal = checkEchoUint32(pD, message);
    }

    return retVal;
}

uint32_t DP_TestEchoExt(DP_PrivateData* pD,
                        const uint8_t*  message,
                        uint8_t*        response,
                        uint16_t        messageSize,
                        DP_BusType      busType)
{
    uint32_t retVal;

    retVal = DP_TestEchoExtSF(pD, message, response, messageSize, busType);

    if (CDN_EOK == retVal) {
        echoBuffer(pD, message, messageSize, busType);
        retVal = checkEchoBuffer(pD, message, response, messageSize);
    }

    return retVal;
}

uint32_t DP_GetCurVersion(const DP_PrivateData* pD, uint16_t* ver, uint16_t* verlib)
{
    uint32_t retVal;
    uint32_t lowByte;
    uint32_t highByte;

    retVal = DP_GetCurVersionSF(pD, ver, verlib);

    if (CDN_EOK == retVal)
    {

        lowByte  = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.VER_L_p);
        highByte = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.VER_H_p);
        *ver = (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__VER_H_P, VER_MSB, highByte) << 8)
               | (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__VER_L_P, VER_LSB, lowByte));

        lowByte  = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.VER_LIB_L_ADDR_p);
        highByte = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.VER_LIB_H_ADDR_p);
        *verlib = (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__VER_LIB_H_ADDR_P, SW_LIB_VER_H, highByte) << 8)
                  | (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__VER_LIB_L_ADDR_P, SW_LIB_VER_L, lowByte));
    }

    return retVal;
}

uint32_t DP_GetEvent(const DP_PrivateData* pD, uint32_t* events)
{
    uint32_t retVal;
    uint32_t readVal;

    retVal = DP_GetEventSF(pD, events);

    if (CDN_EOK == retVal)
    {
        *events = 0;
        readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.APB_INT_STATUS_p);
    }

    if ((CDN_EOK == retVal) &&
    (0U != readVal))
    {
        readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_EVENTS0_p);
        *events = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_EVENTS0_P, SW_EVENTS7_0, readVal);

        readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_EVENTS1_p);
        *events |= (CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_EVENTS1_P, SW_EVENTS15_8, readVal) << 8);

        readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_EVENTS2_p);
        *events |= (CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_EVENTS2_P, SW_EVENTS23_16, readVal) << 16);

        readVal = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_EVENTS3_p);
        *events |= (CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_EVENTS3_P, SW_EVENTS31_24, readVal) << 24);
    }

    return retVal;
}

uint32_t DP_GetDebugRegVal(const DP_PrivateData* pD, uint16_t* debug)
{
    uint32_t retVal;
    uint32_t lowByte;
    uint32_t highByte;

    retVal = DP_GetDebugRegValSF(pD, debug);

    if (CDN_EOK == retVal)
    {
        lowByte  = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_DEBUG_L_p);
        highByte = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.SW_DEBUG_H_p);
        *debug = (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_DEBUG_H_P, SW_DEBUG_15_8, highByte) << 8)
                 | (uint16_t)(CPS_FLD_READ(MHDP__MHDP_APB_REGS__SW_DEBUG_L_P, SW_DEBUG_7_0, lowByte));
    }

    return retVal;
}

uint32_t DP_CheckAlive(DP_PrivateData* pD, bool* updated)
{
    uint32_t retVal = DP_CheckAliveSF(pD, updated);

    if (CDN_EOK == retVal)
    {
        *updated = updateAlive(pD);
    }

    return retVal;
}

uint32_t DP_WaitAlive(DP_PrivateData* pD)
{
    uint32_t retVal;
    bool updated;

    retVal = DP_WaitAliveSF(pD);

    if (CDN_EOK == retVal)
    {
        updated = updateAlive(pD);
        /* Result of first updateAlive is insignificant here. It is performed */
        /* just to update value in pD */
        updated = false;

        while (!updated)
        {
            updated = updateAlive(pD);
        }
    }

    return retVal;
}

uint32_t DP_SendMainControlRequest(DP_PrivateData* pD, uint8_t mode)
{
    uint32_t retVal;

    retVal = DP_SendMainControlRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_MAIN_CONTROL);
        messageWriteUint8(pD, mode);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetMainControlResponse(DP_PrivateData* pD, uint8_t* resp)
{
    uint32_t retVal;
    uint8_t headerMatching;

    retVal = DP_GetMainControlResponseSF(pD, resp);

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_MAIN_CONTROL);

        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, resp);
    }

    return retVal;
}

uint32_t DP_MainControl(DP_PrivateData* pD, uint8_t mode, uint8_t* resp)
{
    uint32_t retVal;

    retVal = DP_MainControlSF(pD, resp);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendMainControlRequest(pD, mode);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetMainControlResponse(pD, resp);
    }

    return retVal;
}

uint32_t DP_SetClock(DP_PrivateData* pD, const DP_UcpuClock* ucpuClock)
{
    uint32_t retVal;

    retVal = DP_SetClockSF(pD, ucpuClock);

    if ((CDN_EOK == retVal) && (0U == ucpuClock->mhz)) {
        retVal = CDN_EPROTO;
    }

    if (CDN_EOK == retVal)
    {
        uint32_t writeVal;

        writeVal = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SW_CLK_H_P, SW_CLOCK_VAL_H, 0U, ucpuClock->mhz);
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.SW_CLK_H_p, writeVal);

        writeVal = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__SW_CLK_L_P, SW_CLOCK_VAL_L, 0U, ucpuClock->fraction);
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.SW_CLK_L_p, writeVal);
    }

    return retVal;
}

/**
 * Initialize part of PHY responsible for AUX channel.
 */
uint32_t DP_ConfigurePhyAuxCtrl(const DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_ConfigurePhyAuxCtrlSF(pD);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SD0801_ConfigurePhyAuxCtrl(pD->phyPd);
    }
    return retVal;
}

uint32_t DP_ConfigurePhyStartUp(DP_PrivateData* pD, uint8_t laneCount, DP_LinkRate linkRate)
{
    uint32_t retVal;

    retVal = DP_ConfigurePhyStartUpSF(pD, linkRate);

    if (CDN_EOK == retVal) {
        retVal = DP_SD0801_PhyStartUp(pD->phyPd, laneCount, toLinkRateSd(linkRate));
    }

    return retVal;
}

uint32_t DP_SendEdidReadRequest(DP_PrivateData* pD,
                                uint8_t         segment,
                                uint8_t         extension)
{
    uint32_t retVal;

    retVal = DP_SendEdidReadRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_EDID);
        messageWriteUint8(pD, segment);
        messageWriteUint8(pD, extension);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetEdidReadResponse(DP_PrivateData*      pD,
                                DP_ReadEdidResponse* resp)
{
    uint32_t retVal;
    uint8_t headerMatching;
    retVal = DP_GetEdidReadResponseSF(pD, resp);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with read EDID "
               "data was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_EDID);

        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, &(resp->size));
        messageReadUint8(pD, &(resp->blockNo));
        if (128U == resp->size)
        {
            messageReadBuffer(pD, resp->buff, resp->size);
        } else {
            retVal = CDN_EIO;
        }
    }

    return retVal;
}

uint32_t DP_ReadEdid(DP_PrivateData*      pD,
                     uint8_t              segment,
                     uint8_t              extension,
                     DP_ReadEdidResponse* resp)
{
    uint32_t retVal = DP_ReadEdidSF(pD, resp);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendEdidReadRequest(pD, segment, extension);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetEdidReadResponse(pD, resp);
    }

    return retVal;

}

uint32_t DP_SetPowerMode(DP_PrivateData* pD, DP_PwrMode mode)
{
    /* DP_PwrMode fits within range of uint8_t. */
    uint32_t retVal = DP_SetPowerModeSF(pD, mode);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_SET_POWER_MNG);
        messageWriteUint8(pD, (uint8_t)mode);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_SetCustomPattern(DP_PrivateData* pD, uint8_t customPattern[10])
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};
    uint8_t i;

    const static uint32_t trainingRegAddress[3] = {
        offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_TRAINING_01_04_p),
        offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_TRAINING_05_08_p),
        offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_TRAINING_09_10_p)
    };

    retVal = setCustomPatternSF(pD, customPattern);

    if (CDN_EOK == retVal) {
        const uint8_t patternRegisterNumber = 3U;
        uint8_t index;
        for (i = 0U; i < patternRegisterNumber; i++) {
            index = i * 4U;
            regTransfer.addr = trainingRegAddress[i];

            /* Same masks for each register */
            regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_TRAINING_01_04_P, DP_TX_PHY_TRAINING_01, 0U, customPattern[index])
                              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_TRAINING_01_04_P, DP_TX_PHY_TRAINING_02, 0U, customPattern[index + 1U]);

            /* Last training register have only 16bits of data */
            if (i < (patternRegisterNumber - 1U)) {
                regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_TRAINING_01_04_P, DP_TX_PHY_TRAINING_03, 0U, customPattern[index + 2U])
                                   | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_TRAINING_01_04_P, DP_TX_PHY_TRAINING_04, 0U, customPattern[index + 3U]);

            }

            retVal = DP_WriteRegister(pD, &regTransfer);

            if (CDN_EOK != retVal) {
                break;
            }
        }

    }

    return retVal;
}

/**
 * Set SCRAMBLER_BYPASS and ENCODER_BYPASS fields in DP_TX_PHY_CONFIG_REG.
 */
static void setPatternBypasses(DP_TestPattern pattern, DP_RegisterTransfer* regTransfer)
{
    /* Disable scrambling for selected patterns */
    if ((DP_PATTERN_TPS1 == pattern) ||
        (DP_PATTERN_TPS2 == pattern) ||
        (DP_PATTERN_TPS3 == pattern) ||
        (DP_PATTERN_D10_2 == pattern)) {
        regTransfer->val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_SCRAMBLER_BYPASS, 0U, 1U);
    }

    /* Disable 8b/10b encoding and scrambling for selected patterns */
    if ((DP_PATTERN_PRBS7 == pattern) || (DP_PATTERN_80_BIT == pattern)) {
        regTransfer->val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_SCRAMBLER_BYPASS, 0U, 1U);
        regTransfer->val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_ENCODER_BYPASS, 0U, 1U);
    }
}

uint32_t DP_SetTestPattern(DP_PrivateData* pD, DP_TestPattern pattern, DP_LinkState* linkParams)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_SetTestPatternSF(pD, pattern, linkParams);

    if (CDN_EOK == retVal)
    {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_CONFIG_REG_p);
        regTransfer.val = COMMON_DPHY_CONFIG;

        /* Enable training (pattern) and set pattern type, if disabling it was not chosen. */
        if (DP_PATTERN_DISABLE != pattern) {
            retVal = configurePhy(pD, linkParams);
            regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_ENABLE, 0U, 1U);
            regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_TRAINING_TYPE, 0U, pattern);
        }
        if (CDN_EOK == retVal) {
            /* Disable scrambler and/or encoder, if pattern requires it. */
            setPatternBypasses(pattern, &regTransfer);
            retVal = DP_WriteRegister(pD, &regTransfer);
        }
    }

    if (CDN_EOK == retVal) {
        /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 "Shifting operation should be checked, DRV-3828" */
        /* laneCount is checked by sanity function */
        uint8_t value = ((1U << linkParams->laneCount) - 1U);
        /* parasoft-end-suppress MISRA2012-RULE-12_2-2 */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DPTX_LANE_EN_p);
        regTransfer.val = (uint32_t)value;

        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_SendDpcdReadRequest(DP_PrivateData*        pD,
                                const DP_DpcdTransfer* request)
{
    uint32_t retVal;

    retVal = DP_SendDpcdReadRequestSF(pD, request);

    if (CDN_EOK == retVal) {
        if ((request->size > DP_MAX_DPCD_TRANSFER_SIZE) || (request->size < 1U)) {
            retVal = CDN_EINVAL;
        }
    }

    if (CDN_EOK == retVal) {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_READ_DPCD);
        messageWriteUint16(pD, request->size);
        messageWrite3Bytes(pD, request->addr);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }
    return retVal;
}

uint32_t DP_GetDpcdReadResponse(DP_PrivateData*  pD,
                                DP_DpcdTransfer* transfer)
{
    uint32_t retVal;
    uint32_t readAddr;
    uint16_t readSize;
    uint16_t copyLen;

    retVal = DP_GetDpcdReadResponseSF(pD, transfer);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with read DPCD "
               "values was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {

        messageReceive(pD, DP_BUS_TYPE_APB);
        uint8_t headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_READ_DPCD);
        if (0U == headerMatching)
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint16(pD, &readSize);
        messageRead3Bytes(pD, &readAddr);

        if ((readSize != transfer->size) || (readAddr != transfer->addr))
        {
            retVal = CDN_EIO;
            /* CDN_EIO error is not fatal in this case - continue execution. */
        }

        if (readSize <= transfer->size)
        {
            copyLen = readSize;
        } else {
            /* Prevent copying more data, than was requested (avoid buffer overflow). */
            copyLen = transfer->size;
        }

        transfer->size = readSize;
        transfer->addr = readAddr;
        messageReadBuffer(pD, transfer->buff, copyLen);
    }

    return retVal;
}

static uint32_t DP_ReadDpcdInternal(DP_PrivateData*  pD,
                     DP_DpcdTransfer* transfer)
{
    uint32_t retVal;

    retVal = DP_SendDpcdReadRequest(pD, transfer);

    if (CDN_EOK == retVal)
    {
        retVal = DP_GetDpcdReadResponse(pD, transfer);
    }

    return retVal;
}

uint32_t DP_ReadDpcd(DP_PrivateData*  pD,
                     DP_DpcdTransfer* transfer)
{
    uint32_t retVal;
    uint32_t i = DP_MAX_DPCD_READ_RETRIES;

    retVal = DP_ReadDpcdSF(pD, transfer);

    if(CDN_EOK == retVal)
    {
        /* Store addr and size, FW can corrupt it */
        uint32_t addr_c = transfer->addr;
        uint16_t size_c = transfer->size;

        while(i--) {
            transfer->addr = addr_c;
            transfer->size = size_c;

            retVal = DP_ReadDpcdInternal(pD, transfer);
            if(CDN_EOK == retVal)
            {
                break;
            }
        }
    }

    return retVal;
}

uint32_t DP_SendDpcdWriteRequest(DP_PrivateData*        pD,
                                 const DP_DpcdTransfer* request)
{
    uint32_t retVal;

    retVal = DP_SendDpcdWriteRequestSF(pD, request);

    if (CDN_EOK == retVal)
    {
        if ((request->size > DP_MAX_DPCD_TRANSFER_SIZE) || (request->size < 1U))
        {
            retVal = CDN_EINVAL;
        }
    }
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_WRITE_DPCD);
        messageWriteUint16(pD, request->size);
        messageWrite3Bytes(pD, request->addr);
        messageWriteBuffer(pD, request->buff, request->size);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }
    return retVal;
}

uint32_t DP_GetDpcdWriteResponse(DP_PrivateData*  pD,
                                 DP_DpcdTransfer* transfer)
{
    uint32_t retVal;
    uint32_t writtenAddr;
    uint16_t writtenSize;
    uint8_t headerMatching;

    retVal = DP_GetDpcdWriteResponseSF(pD, transfer);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response for DPCD write "
               "was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_WRITE_DPCD);
        if (0U == headerMatching)
        {
            retVal = CDN_ENOEXEC;
        }
    }
    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint16(pD, &writtenSize);
        messageRead3Bytes(pD, &writtenAddr);

        if ((writtenSize != transfer->size) || (writtenAddr != transfer->addr))
        {
            retVal = CDN_EIO;
            /* CDN_EIO error is not fatal in this case - continue execution. */
        }

        transfer->size = writtenSize;
        transfer->addr = writtenAddr;
    }

    return retVal;
}

uint32_t DP_WriteDpcd(DP_PrivateData*  pD,
                      DP_DpcdTransfer* transfer)
{
    uint32_t retVal;

    retVal = DP_WriteDpcdSF(pD, transfer);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendDpcdWriteRequest(pD, transfer);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetDpcdWriteResponse(pD, transfer);
    }
    return retVal;
}

uint32_t DP_SendI2cReadRequest(DP_PrivateData*       pD,
                               const DP_I2cTransfer* request)
{
    uint32_t retVal;

    retVal = DP_SendI2cReadRequestSF(pD, request);

    if (CDN_EOK == retVal) {
        if ((request->size > DP_MAX_I2C_TRANSFER_SIZE) || (request->size < 1U)) {
            retVal = CDN_EINVAL;
        }
    }

    if (CDN_EOK == retVal) {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_I2C_READ);
        messageWriteUint16(pD, request->size);
        messageWriteUint8(pD, request->addr);
        messageWriteUint8(pD, ((request->mot) ? 1U : 0U));
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }
    return retVal;
}

uint32_t DP_GetI2cReadResponse(DP_PrivateData* pD,
                               DP_I2cTransfer* transfer)
{
    uint32_t retVal;
    uint8_t readAddr;
    uint16_t readSize;
    uint16_t copyLen;

    retVal = DP_GetI2cReadResponseSF(pD, transfer);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with read I2C "
               "values was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {

        messageReceive(pD, DP_BUS_TYPE_APB);
        uint8_t headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_I2C_READ);
        if (0U == headerMatching)
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint16(pD, &readSize);
        messageReadUint8(pD, &readAddr);

        if ((readSize != transfer->size) || (readAddr != transfer->addr))
        {
            retVal = CDN_EIO;
            /* CDN_EIO error is not fatal in this case - continue execution. */
        }

        if (readSize <= transfer->size)
        {
            copyLen = readSize;
        } else {
            /* Prevent copying more data, than was requested (avoid buffer overflow). */
            copyLen = transfer->size;
        }

        transfer->size = readSize;
        transfer->addr = readAddr;
        messageReadBuffer(pD, transfer->buff, copyLen);
    }

    return retVal;
}

uint32_t DP_I2cRead(DP_PrivateData* pD,
                    DP_I2cTransfer* transfer)
{
    uint32_t retVal;

    retVal = DP_I2cReadSF(pD, transfer);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendI2cReadRequest(pD, transfer);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetI2cReadResponse(pD, transfer);
    }

    return retVal;
}

uint32_t DP_SendI2cWriteRequest(DP_PrivateData*       pD,
                                const DP_I2cTransfer* request)
{
    uint32_t retVal;

    retVal = DP_SendI2cWriteRequestSF(pD, request);

    if (CDN_EOK == retVal)
    {
        if ((request->size > DP_MAX_I2C_TRANSFER_SIZE) || (request->size < 1U))
        {
            retVal = CDN_EINVAL;
        }
    }
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_I2C_WRITE);
        messageWriteUint16(pD, request->size);
        messageWriteUint8(pD, request->addr);
        messageWriteUint8(pD, ((request->mot) ? 1U : 0U));
        messageWriteBuffer(pD, request->buff, request->size);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }
    return retVal;
}

uint32_t DP_GetI2cWriteResponse(DP_PrivateData* pD,
                                DP_I2cTransfer* transfer)
{
    uint32_t retVal;
    uint8_t writtenAddr;
    uint16_t writtenSize;
    uint8_t headerMatching;

    retVal = DP_GetI2cWriteResponseSF(pD, transfer);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response for I2C write "
               "was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_I2C_WRITE);
        if (0U == headerMatching)
        {
            retVal = CDN_ENOEXEC;
        }
    }
    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint16(pD, &writtenSize);
        messageReadUint8(pD, &writtenAddr);

        if ((writtenSize != transfer->size) || (writtenAddr != transfer->addr))
        {
            retVal = CDN_EIO;
            /* CDN_EIO error is not fatal in this case - continue execution. */
        }

        transfer->size = writtenSize;
        transfer->addr = writtenAddr;
    }

    return retVal;
}

uint32_t DP_I2cWrite(DP_PrivateData* pD,
                     DP_I2cTransfer* transfer)
{
    uint32_t retVal;

    retVal = DP_I2cWriteSF(pD, transfer);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendI2cWriteRequest(pD, transfer);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetI2cWriteResponse(pD, transfer);
    }
    return retVal;
}

uint32_t DP_SetAssrEnable(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};
    uint32_t seed;

    retVal = DP_SetAssrEnableSF(pD);

    if (CDN_EOK == retVal) {
        retVal = readSinkCapabilities(pD);
    }

    if (CDN_EOK == retVal)
    {
        if ((enable) && (!pD->sinkCaps.assr))
        {
            retVal = CDN_ENOTSUP;
        }
    }

    if (CDN_EOK == retVal) {
        seed = (enable) ? DP_SCRAMBLER_SEED_ALTERNATE : DP_SCRAMBLER_SEED_REGULAR;

        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_TX_PHY_SCRAMBLER_SEED_p);
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_SCRAMBLER_SEED_P,
                                        DP_TX_PHY_SCRAMBLER_SEED,
                                        0U,
                                        seed);

        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal) {
        retVal = updateAssrEnable(pD, enable);
    }

    return retVal;
}

uint32_t DP_SetShortenedAuxPreamble(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_SetShortenedAuxPreambleSF(pD);

    if (CDN_EOK == retVal) {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_AUX_TX_PREACHARGE_LENGTH_p);
        /* Set 8 pulses for shortened (eDP) preamble. */
        /* Set 16 pulses otherwise. */
        regTransfer.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_AUX_TX_PREACHARGE_LENGTH_P,
                                        AUX_HOST_PRECHARGE_LENGTH,
                                        0U,
                                        (enable ? 8 : 16));
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_SetEventMask(DP_PrivateData* pD, uint32_t mask)
{
    uint32_t retVal;
    uint8_t events = 0U;

    retVal = DP_SetEventMaskSF(pD);

    if (CDN_EOK == retVal)
    {
        if (0U == (mask & DP_HPD_EVENT_ENABLE_BIT)) {
            events = DP_HPD_EVENT_ENABLE_BIT;
        }

        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_ENABLE_EVENT);
        messageWriteUint8(pD, events);
        messageWriteUint32(pD, 0U); /* reserved bits */
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetEventMask(const DP_PrivateData* pD, const uint32_t* mask)
{
    uint32_t retVal;

    retVal = DP_GetEventMaskSF(pD, mask);

    if (CDN_EOK == retVal) {
        retVal = CDN_ENOTSUP;
    }

    return retVal;
}

uint32_t DP_SendReadHpdEventRequest(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_SendReadHpdEventRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_READ_EVENT);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetReadHpdEventResponse(DP_PrivateData* pD, uint8_t* hpdEvents)
{
    uint32_t retVal;
    uint8_t headerMatching;

    retVal = DP_GetReadHpdEventResponseSF(pD, hpdEvents);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HPD events"
               " was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_READ_EVENT);

        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, hpdEvents);
    }

    return retVal;
}

uint32_t DP_ReadHpdEvent(DP_PrivateData* pD, uint8_t* hpdEvents)
{
    uint32_t retVal;

    retVal = DP_ReadHpdEventSF(pD, hpdEvents);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendReadHpdEventRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetReadHpdEventResponse(pD, hpdEvents);
    }

    return retVal;
}

/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF parameter for function should be not higher than 4, DRV-3839" */
uint32_t DP_FillVideoFormat(DP_VideoFormatParams* vicParams, DP_VicModes vicMode)
{
    uint32_t retVal;

    /* Array of video timings format provided by the driver, based on */
    /* ANSI/CEA-861-F (August 2013), chapter 4, Table 1 and 2. */
    /* Each row represents one video format. Values in row correspond to */
    /* subsequent members of DP_VideoFormatParams structure. */
    static const DP_VideoFormatParams vicTable[] = {
        {1U, 800U, 640U, 160U, 96U, 16U, 48U, 31.469, 525U, 480U, 45.0, 2U, 10U, 33U, 59.94, 25.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {1U, 800U, 640U, 160U, 96U, 16U, 48U, 31.5, 525U, 480U, 45.0, 2U, 10U, 33U, 60.0, 25.20, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {2U, 858U, 720U, 138U, 62U, 16U, 60U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {2U, 858U, 720U, 138U, 62U, 16U, 60U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {3U, 858U, 720U, 138U, 62U, 16U, 60U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {3U, 858U, 720U, 138U, 62U, 16U, 60U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {4U, 1650U, 1280U, 370U, 40U, 110U, 220U, 44.955, 750U, 720U, 30.0, 5U, 5U, 20U, 59.94, 74.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {4U, 1650U, 1280U, 370U, 40U, 110U, 220U, 45.0, 750U, 720U, 30.0, 5U, 5U, 20U, 60.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {5U, 2200U, 1920U, 280U, 44U, 88U, 148U, 33.716, 1125U, 1080U, 22.51, 5U, 2U, 15U, 59.94, 74.18, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {5U, 2200U, 1920U, 280U, 44U, 88U, 148U, 33.75, 1125U, 1080U, 22.51, 5U, 2U, 15U, 60.0, 74.25, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {6U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 525U, 480U, 22.51, 3U, 4U, 15U, 59.94, 27.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {6U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 525U, 480U, 22.51, 3U, 4U, 15U, 60.0, 27.03, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {7U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 525U, 480U, 22.51, 3U, 4U, 15U, 59.94, 27.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {7U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 525U, 480U, 22.51, 3U, 4U, 15U, 60.0, 27.03, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {8U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 262U, 240U, 22.0, 3U, 4U, 15U, 60.054, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {8U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 262U, 240U, 22.0, 3U, 4U, 15U, 60.115, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {8U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 263U, 240U, 23.0, 3U, 5U, 15U, 59.826, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {8U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 263U, 240U, 23.0, 3U, 5U, 15U, 59.886, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {9U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 262U, 240U, 22.0, 3U, 4U, 15U, 60.054, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {9U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 262U, 240U, 22.0, 3U, 4U, 15U, 60.115, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {9U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.734, 263U, 240U, 23.0, 3U, 5U, 15U, 59.826, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {9U, 1716U, 1440U, 276U, 124U, 38U, 114U, 15.75, 263U, 240U, 23.0, 3U, 5U, 15U, 59.886, 27.03, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {10U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 525U, 480U, 22.51, 3U, 4U, 15U, 59.94, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {10U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 525U, 480U, 22.51, 3U, 4U, 15U, 60.0, 54.05, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {11U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 525U, 480U, 22.51, 3U, 4U, 15U, 59.94, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {11U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 525U, 480U, 22.51, 3U, 4U, 15U, 60.0, 54.05, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {12U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 262U, 240U, 22.0, 3U, 4U, 15U, 60.054, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {12U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 262U, 240U, 22.0, 3U, 4U, 15U, 60.115, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {12U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 263U, 240U, 23.0, 3U, 5U, 15U, 59.826, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {12U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 263U, 240U, 23.0, 3U, 5U, 15U, 59.886, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {13U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 262U, 240U, 22.0, 3U, 4U, 15U, 60.054, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {13U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 262U, 240U, 22.0, 3U, 4U, 15U, 60.115, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {13U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.734, 263U, 240U, 23.0, 3U, 5U, 15U, 59.826, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {13U, 3432U, 2880U, 552U, 248U, 76U, 228U, 15.75, 263U, 240U, 23.0, 3U, 5U, 15U, 59.886, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {14U, 1716U, 1440U, 276U, 124U, 32U, 120U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {14U, 1716U, 1440U, 276U, 124U, 32U, 120U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {15U, 1716U, 1440U, 276U, 124U, 32U, 120U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {15U, 1716U, 1440U, 276U, 124U, 32U, 120U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {16U, 2200U, 1920U, 280U, 44U, 88U, 148U, 67.433, 1125U, 1080U, 45.0, 5U, 4U, 36U, 59.94, 148.35, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {16U, 2200U, 1920U, 280U, 44U, 88U, 148U, 67.5, 1125U, 1080U, 45.0, 5U, 4U, 36U, 60.0, 148.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {17U, 864U, 720U, 144U, 64U, 12U, 68U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {18U, 864U, 720U, 144U, 64U, 12U, 68U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {19U, 1980U, 1280U, 700U, 40U, 440U, 220U, 37.5, 750U, 720U, 30.0, 5U, 5U, 20U, 50.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {20U, 2640U, 1920U, 720U, 44U, 528U, 148U, 28.125, 1125U, 1080U, 22.51, 5U, 2U, 15U, 50.0, 74.25, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {21U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 625U, 576U, 24.51, 3U, 2U, 19U, 50.0, 27.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {22U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 625U, 576U, 24.51, 3U, 2U, 19U, 50.0, 27.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {23U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 312U, 288U, 24.0, 3U, 2U, 19U, 50.08, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {23U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 313U, 288U, 25.0, 3U, 3U, 19U, 49.92, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {23U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 314U, 288U, 26.0, 3U, 4U, 19U, 49.761, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {24U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 312U, 288U, 24.0, 3U, 2U, 19U, 50.08, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {24U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 313U, 288U, 25.0, 3U, 3U, 19U, 49.92, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {24U, 1728U, 1440U, 288U, 126U, 24U, 138U, 15.625, 314U, 288U, 26.0, 3U, 4U, 19U, 49.761, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {25U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 625U, 576U, 24.51, 3U, 2U, 19U, 50.0, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {26U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 625U, 576U, 24.51, 3U, 2U, 19U, 50.0, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {27U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 312U, 288U, 24.0, 3U, 2U, 19U, 50.08, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {27U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 313U, 288U, 25.0, 3U, 3U, 19U, 49.92, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {27U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 314U, 288U, 26.0, 3U, 4U, 19U, 49.761, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {28U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 312U, 288U, 24.0, 3U, 2U, 19U, 50.08, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {28U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 313U, 288U, 25.0, 3U, 3U, 19U, 49.92, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {28U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 314U, 288U, 26.0, 3U, 4U, 19U, 49.761, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {29U, 1728U, 1440U, 288U, 128U, 24U, 136U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {30U, 1728U, 1440U, 288U, 128U, 24U, 136U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {31U, 2640U, 1920U, 720U, 44U, 528U, 148U, 56.25, 1125U, 1080U, 45.0, 5U, 4U, 36U, 50.0, 148.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {32U, 2750U, 1920U, 830U, 44U, 638U, 148U, 26.973, 1125U, 1080U, 45.0, 5U, 4U, 36U, 23.976, 74.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {32U, 2750U, 1920U, 830U, 44U, 638U, 148U, 27.0, 1125U, 1080U, 45.0, 5U, 4U, 36U, 24.0003, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {33U, 2640U, 1920U, 720U, 44U, 528U, 148U, 28.125, 1125U, 1080U, 45.0, 5U, 4U, 36U, 25.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {34U, 2200U, 1920U, 280U, 44U, 88U, 148U, 33.716, 1125U, 1080U, 45.0, 5U, 4U, 36U, 29.97, 74.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {34U, 2200U, 1920U, 280U, 44U, 88U, 148U, 33.75, 1125U, 1080U, 45.0, 5U, 4U, 36U, 30.0003, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {35U, 3432U, 2880U, 552U, 248U, 64U, 240U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {35U, 3432U, 2880U, 552U, 248U, 64U, 240U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 108.11, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {36U, 3432U, 2880U, 552U, 248U, 64U, 240U, 31.469, 525U, 480U, 45.0, 6U, 9U, 30U, 59.94, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {36U, 3432U, 2880U, 552U, 248U, 64U, 240U, 31.5, 525U, 480U, 45.0, 6U, 9U, 30U, 60.0, 108.11, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {37U, 3456U, 2880U, 576U, 256U, 48U, 272U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {38U, 3456U, 2880U, 576U, 256U, 48U, 272U, 31.25, 625U, 576U, 49.0, 5U, 5U, 39U, 50.0, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {39U, 2304U, 1920U, 384U, 168U, 32U, 184U, 31.25, 1250U, 1080U, 85.0, 5U, 23U, 57U, 50.0, 72.00, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_LOW, 8U, 0U},
        {40U, 2640U, 1920U, 720U, 44U, 528U, 148U, 56.25, 1125U, 1080U, 22.51, 5U, 2U, 15U, 100.0, 148.50, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {41U, 1980U, 1280U, 700U, 40U, 440U, 220U, 75.0, 750U, 720U, 30.0, 5U, 5U, 20U, 100.0, 148.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {42U, 864U, 720U, 144U, 64U, 12U, 68U, 62.5, 625U, 576U, 49.0, 5U, 5U, 39U, 100.0, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {43U, 864U, 720U, 144U, 64U, 12U, 68U, 62.5, 625U, 576U, 49.0, 5U, 5U, 39U, 100.0, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {44U, 1728U, 1440U, 288U, 126U, 24U, 138U, 31.25, 625U, 576U, 24.51, 3U, 2U, 19U, 100.0, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {45U, 1728U, 1440U, 288U, 126U, 24U, 138U, 31.25, 625U, 576U, 24.51, 3U, 2U, 19U, 100.0, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {46U, 2200U, 1920U, 280U, 44U, 88U, 148U, 67.432, 1125U, 1080U, 22.51, 5U, 2U, 15U, 119.88, 148.35, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {46U, 2200U, 1920U, 280U, 44U, 88U, 148U, 67.5, 1125U, 1080U, 22.51, 5U, 2U, 15U, 120.0, 148.50, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {47U, 1650U, 1280U, 370U, 40U, 110U, 220U, 89.909, 750U, 720U, 30.0, 5U, 5U, 20U, 119.88, 148.35, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {47U, 1650U, 1280U, 370U, 40U, 110U, 220U, 90.0, 750U, 720U, 30.0, 5U, 5U, 20U, 120.0, 148.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {48U, 858U, 720U, 138U, 62U, 16U, 60U, 62.937, 525U, 480U, 45.0, 6U, 9U, 30U, 119.88, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {48U, 858U, 720U, 138U, 62U, 16U, 60U, 63.0, 525U, 480U, 45.0, 6U, 9U, 30U, 120.0, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {49U, 858U, 720U, 138U, 62U, 16U, 60U, 62.937, 525U, 480U, 45.0, 6U, 9U, 30U, 119.88, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {49U, 858U, 720U, 138U, 62U, 16U, 60U, 63.0, 525U, 480U, 45.0, 6U, 9U, 30U, 120.0, 54.05, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {50U, 1716U, 1440U, 276U, 124U, 38U, 114U, 31.469, 525U, 480U, 22.51, 3U, 4U, 15U, 119.88, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {50U, 1716U, 1440U, 276U, 124U, 38U, 114U, 31.5, 525U, 480U, 22.51, 3U, 4U, 15U, 120.0, 54.05, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {51U, 1716U, 1440U, 276U, 124U, 38U, 114U, 31.469, 525U, 480U, 22.51, 3U, 4U, 15U, 119.88, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {51U, 1716U, 1440U, 276U, 124U, 38U, 114U, 31.5, 525U, 480U, 22.51, 3U, 4U, 15U, 120.0, 54.05, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {52U, 864U, 720U, 144U, 64U, 12U, 68U, 125.0, 625U, 576U, 49.0, 5U, 5U, 39U, 200.0, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {53U, 864U, 720U, 144U, 64U, 12U, 68U, 125.0, 625U, 576U, 49.0, 5U, 5U, 39U, 200.0, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {54U, 1728U, 1440U, 288U, 126U, 24U, 138U, 62.5, 625U, 576U, 24.51, 3U, 2U, 19U, 200.0, 108.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {55U, 1728U, 1440U, 288U, 126U, 24U, 138U, 62.5, 625U, 576U, 24.51, 3U, 2U, 19U, 200.0, 108.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {56U, 858U, 720U, 138U, 62U, 16U, 60U, 125.874, 525U, 480U, 45.0, 6U, 9U, 30U, 239.76, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {56U, 858U, 720U, 138U, 62U, 16U, 60U, 126.0, 525U, 480U, 45.0, 6U, 9U, 30U, 240.0, 108.11, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {57U, 858U, 720U, 138U, 62U, 16U, 60U, 125.874, 525U, 480U, 45.0, 6U, 9U, 30U, 239.76, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {57U, 858U, 720U, 138U, 62U, 16U, 60U, 126.0, 525U, 480U, 45.0, 6U, 9U, 30U, 240.0, 108.11, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {58U, 1716U, 1440U, 276U, 124U, 38U, 114U, 62.937, 525U, 480U, 22.51, 3U, 4U, 15U, 239.76, 108.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {58U, 1716U, 1440U, 276U, 124U, 38U, 114U, 63.0, 525U, 480U, 22.51, 3U, 4U, 15U, 240.0, 108.11, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {59U, 1716U, 1440U, 276U, 124U, 38U, 114U, 62.937, 525U, 480U, 22.51, 3U, 4U, 15U, 239.76, 108.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {59U, 1716U, 1440U, 276U, 124U, 38U, 114U, 63.0, 525U, 480U, 22.51, 3U, 4U, 15U, 240.0, 108.11, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {60U, 3300U, 1280U, 2020U, 40U, 1760U, 220U, 17.977, 750U, 720U, 30.0, 5U, 5U, 20U, 23.97, 59.34, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {60U, 3300U, 1280U, 2020U, 40U, 1760U, 220U, 18.0, 750U, 720U, 30.0, 5U, 5U, 20U, 24.0, 59.4, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {61U, 3960U, 1280U, 2680U, 40U, 2420U, 220U, 18.75, 750U, 720U, 30.0, 5U, 5U, 20U, 25.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {62U, 3300U, 1280U, 2020U, 40U, 1760U, 220U, 22.477, 750U, 720U, 30.0, 5U, 5U, 20U, 29.97, 74.176, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {62U, 3300U, 1280U, 2020U, 40U, 1760U, 220U, 22.5, 750U, 720U, 30.0, 5U, 5U, 20U, 30.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {63U, 2200U, 1920U, 280U, 44U, 88U, 148U, 134.865, 1125U, 1080U, 45.0, 5U, 4U, 36U, 119.88, 296.703, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {63U, 2200U, 1920U, 280U, 44U, 88U, 148U, 135.0, 1125U, 1080U, 45.0, 5U, 4U, 36U, 120.0, 297.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {64U, 2640U, 1920U, 720U, 44U, 528U, 148U, 112.5, 1125U, 1080U, 45.0, 5U, 4U, 36U, 100.0, 297.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {97U, 4400U, 3840U, 560U, 88U, 176U, 296U, 135.0, 2250U, 2160U, 90.0, 10U, 8U, 72U, 60.0, 594.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {114U, 1056U, 800U, 256U, 128U, 40U, 88U, 37.879, 628U, 600U, 28.0, 4U, 1U, 23U, 60.317, 40.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {115U, 896U, 720U, 176U, 64U, 24U, 88U, 24.833, 419U, 400U, 19.0, 10U, 3U, 6U, 59.26, 22.250, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {116U, 912U, 720U, 192U, 72U, 24U, 96U, 37.00657895, 424U, 400U, 24.0, 10U, 3U, 11U, 87.27966733, 33.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {117U, 800U, 640U, 160U, 64U, 16U, 80U, 29.6875, 500U, 480U, 20.0, 4U, 3U, 13U, 59.375, 23.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {118U, 800U, 640U, 160U, 64U, 16U, 80U, 33.4375, 502U, 480U, 22.0, 4U, 3U, 15U, 66.60856574, 26.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {119U, 816U, 640U, 176U, 64U, 24U, 88U, 36.15196078, 503U, 480U, 23.0, 4U, 3U, 16U, 71.87268546, 29.5, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {120U, 816U, 640U, 176U, 64U, 24U, 88U, 37.68382353, 504U, 480U, 24.0, 4U, 3U, 17U, 74.76949113, 30.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {121U, 1008U, 800U, 208U, 80U, 24U, 104U, 34.72222222, 623U, 600U, 23.0, 4U, 3U, 16U, 55.73390405, 35.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {122U, 1040U, 800U, 240U, 80U, 40U, 120U, 45.19230769, 628U, 600U, 28.0, 4U, 3U, 21U, 71.9622734, 47.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {123U, 1088U, 832U, 256U, 80U, 48U, 128U, 48.94301471, 654U, 624U, 30.0, 4U, 3U, 23U, 74.83641392, 53.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {124U, 1376U, 1024U, 352U, 104U, 72U, 176U, 74.49127907, 859U, 768U, 45.0, 4U, 3U, 38U, 86.71860194, 102.5, DP_SM_INTERLACED, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {125U, 1328U, 1024U, 304U, 104U, 48U, 152U, 47.81626506, 798U, 768U, 30.0, 4U, 3U, 23U, 59.92013165, 63.5, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {126U, 1360U, 1024U, 336U, 104U, 64U, 168U, 57.72058824, 803U, 768U, 35.0, 4U, 3U, 28U, 71.88118087, 78.5, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {127U, 1360U, 1024U, 336U, 104U, 64U, 168U, 60.29411765, 805U, 768U, 37.0, 4U, 3U, 30U, 74.89952503, 82.0, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {128U, 1728U, 1280U, 448U, 136U, 88U, 224U, 80.29513889, 1072U, 1024U, 48.0, 7U, 3U, 38U, 74.9021818, 138.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {129U, 1536U, 1152U, 384U, 120U, 72U, 192U, 68.19661458, 911U, 870U, 41.0, 10U, 3U, 28U, 74.85907199, 104.75, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {130U, 2200, 1920, 280, 44, 88, 148, 66.587,  1125, 1080, 45, 5, 4, 36, 59.93, 148.5, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8, 0},
        {131U, 2720, 2560, 160, 32, 48, 80, 88.787,  1481, 1440, 41, 5, 3, 33, 59.95, 240, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8, 0},
        {132U, 4000, 3840, 160, 32, 48, 80, 133.313, 2222, 2160, 62, 5, 3, 54, 59.99, 533.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8, 0},	
        {0U, 5500U, 5120U, 380U, 88U, 164U, 128U, 135.0, 2250U, 2160U, 90.0, 10U, 8U, 72U, 60.0, 742.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {1U, 800U, 640U, 160U, 96U, 16U, 48U, 31.469, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 25.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {2U, 858U, 720U, 138U, 62U, 16U, 60U, 31.469, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {4U, 1650U, 1280U, 370U, 40U, 110U, 220U, 44.955, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 74.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {14U, 1716U, 1440U, 276U, 124U, 32U, 120U, 31.469, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {16U, 2200U, 1920U, 280U, 44U, 88U, 148U, 67.433, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 148.35, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {17U, 864U, 720U, 144U, 64U, 12U, 68U, 31.25, 36U, 20U, 16.0, 2U, 3U, 11U, 50.0, 27.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {25U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 36U, 20U, 16.0, 2U, 3U, 11U, 50.0, 54.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {28U, 3456U, 2880U, 576U, 252U, 48U, 276U, 15.625, 36U, 20U, 16.0, 2U, 3U, 11U, 50.08, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 3U},
        {31U, 2640U, 1920U, 720U, 44U, 528U, 148U, 56.25, 36U, 20U, 16.0, 2U, 3U, 11U, 50.0, 148.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {32U, 2750U, 1920U, 830U, 44U, 638U, 148U, 26.973, 36U, 20U, 16.0, 2U, 3U, 11U, 23.976, 74.18, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {35U, 3432U, 2880U, 552U, 248U, 64U, 240U, 31.469, 36U, 20U, 16.0, 2U, 3U, 11U, 59.94, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {47U, 1650U, 1280U, 370U, 40U, 110U, 220U, 89.909, 36U, 20U, 16.0, 2U, 3U, 11U, 119.88, 148.35, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {49U, 858U, 720U, 138U, 62U, 16U, 60U, 62.937, 36U, 20U, 16.0, 2U, 3U, 11U, 119.88, 54.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {52U, 864U, 720U, 144U, 64U, 12U, 68U, 125.0, 36U, 20U, 16.0, 2U, 3U, 11U, 200.0, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {57U, 858U, 720U, 138U, 62U, 16U, 60U, 125.874, 36U, 20U, 16.0, 2U, 3U, 11U, 239.76, 108.00, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 0U},
        {58U, 1716U, 1440U, 276U, 124U, 38U, 114U, 62.937, 36U, 20U, 16.0, 2U, 3U, 11U, 239.76, 108.00, DP_SM_INTERLACED, DP_SP_ACTIVE_LOW, DP_SP_ACTIVE_LOW, 8U, 1U},
        {61U, 3960U, 1280U, 2680U, 40U, 2420U, 220U, 18.75, 36U, 20U, 16.0, 2U, 3U, 11U, 25.0, 74.25, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {63U, 2200U, 1920U, 280U, 44U, 88U, 148U, 134.865, 36U, 20U, 16.0, 2U, 3U, 11U, 119.88, 296.703, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U},
        {0U, 5500U, 5120U, 380U, 88U, 164U, 128U, 135.0, 36U, 20U, 16.0, 2U, 3U, 11U, 60.0, 742.50, DP_SM_PROGRESSIVE, DP_SP_ACTIVE_HIGH, DP_SP_ACTIVE_HIGH, 8U, 0U}
    };

    retVal = DP_FillVideoFormatSF(vicParams, vicMode);

    if (CDN_EOK == retVal) {
        if (DP_VIC_MODE_COUNT == vicMode) {
            retVal = CDN_EINVAL;
        }
    }

    if (CDN_EOK == retVal)
    {
        CPS_BufferCopy((uint8_t*)vicParams,
                       (const uint8_t*)&(vicTable[vicMode]),
                       (uint32_t)sizeof(DP_VideoFormatParams));
    }

    return retVal;
}
/* parasoft-end-suppress METRICS-39-3 */

uint32_t DP_SetVic(DP_PrivateData* pD, uint8_t streamId, const DP_VideoParameters *parameters)
{
    uint32_t retVal;

    retVal = DP_SetVicSF(pD, parameters);

    if (CDN_EOK == retVal) {
        retVal = checkVideoParams(parameters);
    }

    if (CDN_EOK == retVal) {
        retVal = checkPhyInit(pD);
    }

    if (CDN_EOK == retVal) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        retVal = configureVicParams(pD, streamId, parameters);
    }

    if (CDN_EOK == retVal) {
        retVal = setVifClock(pD, streamId, true);
    }

    if (CDN_EOK == retVal) {
        if (streamId < DP_NUMBER_OF_DSC_ENCODERS) {
            retVal = DP_DscStreamEnable(pD, streamId, pD->videoParameters[streamId].dscEnable);
        }
    }

    if (CDN_EOK == retVal) {
        if (!pD->mstEnabled) {
            pD->sstVicSet = true;
        }
    }

    return retVal;
}

uint32_t DP_SetMsaSyncPolarity(DP_PrivateData* pD,
                               uint8_t         streamId,
                               DP_SyncPolarity hSyncPolarity,
                               DP_SyncPolarity vSyncPolarity)
{
    uint32_t retVal;
    uint32_t addr;
    DP_WriteFieldRequest req;

    retVal = DP_SetMsaSyncPolaritySF(pD, hSyncPolarity, vSyncPolarity);

    if (CDN_EOK == retVal) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_HORIZONTAL_1_p);
        addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        req.addr = addr;
        req.startBit = MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P__PCK_STUFF_HSYNCPOLARITY_SHIFT;
        req.bitCount = MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P__PCK_STUFF_HSYNCPOLARITY_WIDTH;
        req.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_HORIZONTAL_1_P,
                                PCK_STUFF_HSYNCPOLARITY,
                                0U,
                                (DP_SP_ACTIVE_LOW == hSyncPolarity) ? 1U : 0U);
        retVal = DP_WriteField(pD, &req);
    }
    if (CDN_EOK == retVal) {
        addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].MSA_VERTICAL_1_p);
        addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        req.addr = addr;
        req.startBit = MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P__PCK_STUFF_VSYNCPOLARITY_SHIFT;
        req.bitCount = MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P__PCK_STUFF_VSYNCPOLARITY_WIDTH;
        req.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM__MSA_VERTICAL_1_P,
                                PCK_STUFF_VSYNCPOLARITY,
                                0U,
                                (DP_SP_ACTIVE_LOW == vSyncPolarity) ? 1U : 0U);
        retVal = DP_WriteField(pD, &req);
    }

    return retVal;
}

uint32_t DP_SetFramerEnable(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;
    const uint8_t FramerEnableBitPos = MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P__GLOBAL_EN_SHIFT;
    DP_RegisterTransfer regTransfer = {0};
    DP_WriteFieldRequest req;

    req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
    req.startBit = FramerEnableBitPos;
    req.bitCount = 1;
    req.val = enable ? MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P__GLOBAL_EN_MASK : 0U;

    retVal = DP_SetFramerEnableSF(pD);

    if (CDN_EOK == retVal)
    {
        retVal = DP_WriteField(pD, &req);
    }

    if (CDN_EOK == retVal)
    {
        /* Register read - to make sure uCPU has finished writing the register. */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_SetVideoSst(DP_PrivateData* pD, bool mode)
{
    uint32_t retVal;
    const uint8_t noVideoBitPos = MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P__NO_VIDEO_SHIFT;
    DP_RegisterTransfer regTransfer = {0};
    DP_WriteFieldRequest req;

    req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
    req.startBit = noVideoBitPos;
    req.bitCount = 1;
    req.val = mode ? 0U : MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P__NO_VIDEO_MASK;

    retVal = DP_SetVideoSstSF(pD);

    if (CDN_EOK == retVal)
    {
        retVal = DP_WriteField(pD, &req);
    }

    if (CDN_EOK == retVal)
    {
        /* Register read - to make sure uCPU has finished writing the register. */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_SendAuxStatusRequest(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_SendAuxStatusRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_LAST_AUX_STAUS);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetAuxStatusResponse(DP_PrivateData* pD, DP_AuxStatus* status)
{
    uint32_t retVal;
    uint8_t result;
    uint8_t headerMatching;

    retVal = DP_GetAuxStatusResponseSF(pD, status);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with AUX status"
               " was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_LAST_AUX_STAUS);

        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, &result);

        *status = getAuxStatus(result);
    }

    return retVal;
}

uint32_t DP_GetAuxStatus(DP_PrivateData* pD, DP_AuxStatus* status)
{
    uint32_t retVal;

    retVal = DP_GetAuxStatusSF(pD, status);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendAuxStatusRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetAuxStatusResponse(pD, status);
    }

    return retVal;
}

uint32_t DP_SendI2cStatusRequest(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_SendI2cStatusRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_LAST_I2C_STATUS);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetI2cStatusResponse(DP_PrivateData* pD, DP_I2cStatus* status)
{
    uint32_t retVal;
    uint8_t result;
    uint8_t headerMatching;

    retVal = DP_GetI2cStatusResponseSF(pD, status);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with I2C status"
               " was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_GET_LAST_I2C_STATUS);

        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, &result);

        *status = getI2cStatus(result);
    }

    return retVal;
}

uint32_t DP_GetI2cStatus(DP_PrivateData* pD, DP_I2cStatus* status)
{
    uint32_t retVal;

    retVal = DP_GetI2cStatusSF(pD, status);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendI2cStatusRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetI2cStatusResponse(pD, status);
    }

    return retVal;
}

uint32_t DP_SendHpdStatusRequest(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_SendHpdStatusRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_HPD_STATE);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_GetHpdStatusResponse(DP_PrivateData* pD, bool* status)
{
    uint32_t retVal;
    uint8_t statusRead;
    uint8_t headerMatching;

    retVal = DP_GetHpdStatusResponseSF(pD, status);
    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HPD status"
               " was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_APB);
        headerMatching = messageHeaderMatches(pD, MB_MODULE_ID_DP_TX, (uint8_t)DPTX_HPD_STATE);
        if (0U == headerMatching) {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, &statusRead);

        if (statusRead == 0U) {
            *status = false;
        } else {
            *status = true;
        }

    }

    return retVal;
}

uint32_t DP_GetHpdStatus(DP_PrivateData* pD, bool* status)
{
    uint32_t retVal;

    retVal = DP_GetHpdStatusSF(pD, status);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendHpdStatusRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetHpdStatusResponse(pD, status);
    }

    return retVal;
}

uint32_t DP_SetDscConfig(DP_PrivateData* pD, uint8_t streamId, const DP_DscConfig* dscConfig)
{
    uint32_t retVal;
    DP_DscConfigFull* currentDscConfig;

    retVal = DP_SetDscConfigSF(pD, dscConfig);

    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstDscSanity(pD, streamId, dscConfig->splitPanel);
    }

    if (CDN_EOK == retVal)
    {
        currentDscConfig =  &pD->dscConfig[streamId];
        retVal = DscCopyCfgToFull(dscConfig, currentDscConfig);
    }

    if (CDN_EOK == retVal)
    {
        retVal = DscCalcParameters(currentDscConfig);
    }

    if (CDN_EOK == retVal)
    {
        retVal = DP_DscWriteConfiguration(pD, currentDscConfig, streamId);
    }

    if (CDN_EOK == retVal)
    {
        /* udpate DSC registers for seleted encoder */
        retVal = DP_DscRegistersUpdate(pD, streamId);
    }

    if (CDN_EOK == retVal)
    {
        /* udpate DSC registers for encoder 1 */
        if (dscConfig->splitPanel) {
            retVal = DP_DscRegistersUpdate(pD, 1U);
        }
    }

    return retVal;
}

uint32_t DP_GetDscConfig(DP_PrivateData* pD, uint8_t streamId, DP_DscConfig* dscConfig)
{
    uint32_t retVal;

    retVal = DP_GetDscConfigSF(pD, dscConfig);

    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstDscSanity(pD, streamId, pD->dscConfig[0].splitPanel);
    }

    if (CDN_EOK == retVal)
    {
        retVal = DP_DscReadConfiguration(pD, &pD->dscConfig[streamId], streamId);
    }

    if (CDN_EOK == retVal)
    {
        DscCopyFromFull(dscConfig, &pD->dscConfig[streamId]);
    }

    return retVal;
}

uint32_t DP_DscSendPps(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;
    const uint8_t packetType = 0x10U; /* PPS packet type*/
    uint8_t pps[DP_DSC_PPS_SIZE + DP_SDP_HEADER_SIZE];
    DP_SdpEntry sdpEntry;

    retVal = DP_DscSendPpsSF(pD);

    if (CDN_EOK == retVal) {
        retVal = DP_StreamIdMstSstDscSanity(pD, streamId, pD->dscConfig[0].splitPanel);
    }

    if (CDN_EOK == retVal) {
        DscWritePps(&pps[DP_SDP_HEADER_SIZE], &pD->dscConfig[streamId]);

        /* fill pps header as defined in DP standard */
        pps[0] = 0U; /* datat packet ID*/
        pps[1] = packetType;
        pps[2] = (uint8_t)DP_DSC_PPS_SIZE - 1U;
        pps[3] = 0U; /* Reserved */

        sdpEntry.packet = (uint32_t*)pps;
        sdpEntry.type = packetType;
        sdpEntry.length = (uint8_t)DP_DSC_PPS_SIZE + (uint8_t)DP_SDP_HEADER_SIZE;
        sdpEntry.activeMode = DP_SDP_ACTIVE_VIDEO;

        retVal = DP_SetSdp(pD, streamId, 0U, &sdpEntry);
    }

    return retVal;
}

uint32_t DP_SetCompressedStreamFlag(DP_PrivateData* pD, uint8_t streamId, bool val)
{
    uint32_t retVal;

    DP_RegisterTransfer regTransfer = {0U};
    const uint32_t DP_VBID_COMPRESSED_STREAM_FLAG = 1U << 6;

    retVal = DP_SetCompressedStreamFlagSF(pD);

    if (CDN_EOK == retVal)
    {
        retVal = DP_StreamIdMstSstDscSanity(pD, streamId, pD->dscConfig[0].splitPanel);
    }

    if (CDN_EOK == retVal)
    {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VB_ID_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal)
    {
        if (val)
        {
            regTransfer.val |= DP_VBID_COMPRESSED_STREAM_FLAG;
        } else {
            regTransfer.val &= ~DP_VBID_COMPRESSED_STREAM_FLAG;
        }

        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_VB_ID_p);
        regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_DscReset(DP_PrivateData *pD)
{
    uint32_t retVal;

    retVal = DP_DscResetSF(pD);

    if (CDN_EOK == retVal) {
        retVal = DP_DscSoftwareReset(pD);
    } else {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "DSC reset failed\n");
    }

    return retVal;
}

uint32_t DP_SetWatchdogConfig(DP_PrivateData* pD, uint32_t watchdogMin,
                              uint32_t watchdogMax)
{
    uint32_t retVal;

    retVal = DP_SetWatchdogConfigSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_SET_WATCHDOG_CFG);
        messageWriteUint32(pD, watchdogMin);
        messageWriteUint32(pD, watchdogMax);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_InjectEccError(DP_PrivateData*    pD,
                           uint32_t           errorMask,
                           DP_EccErrorMemType memType,
                           DP_EccErrorType    errorType)
{
    uint32_t retVal;

    retVal = DP_InjectEccErrorSF(pD, memType, errorType);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_INJECT_ECC_ERROR);
        messageWriteUint32(pD, errorMask);
        messageWriteUint8(pD, (uint8_t)memType);
        messageWriteUint8(pD, (uint8_t)errorType);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_ForceFatalError(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_ForceFatalErrorSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_FORCE_FATAL_ERROR);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_SetFecEnable(DP_PrivateData* pD, bool fecEnable)
{
    uint32_t retVal;

    retVal = checkIfCanSetFec(pD, fecEnable);

    if (CDN_EOK == retVal) {
        retVal = setFecEnable(pD, fecEnable);
    }

    if (retVal == CDN_EOK) {
        retVal = waitFecStatus(pD, fecEnable);
    }

    if (CDN_EOK == retVal) {
        pD->fecEnabled = fecEnable;
    }

    if (CDN_EAGAIN == retVal) {
        retVal = CDN_EOK;
    }

    return retVal;
}

uint32_t DP_SetFecReady(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;

    retVal = DP_SetFecReadySF(pD);

    if (retVal == CDN_EOK) {
        retVal = sinkSetFecReady(pD, enable);
    }

    if (retVal == CDN_EOK) {
        retVal = sourceSetFecReady(pD, enable);
    }

    return retVal;
}

uint32_t DP_MstAllocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice)
{
    uint32_t retVal;

    retVal = DP_MstAllocateVcpiSF(pD, sinkDevice);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = DP_MST_AllocateVcpi(pD, streamId, sinkDevice);
    }

    return retVal;
}

uint32_t DP_MstDeallocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice)
{
    uint32_t retVal;

    retVal = DP_MstDeallocateVcpiSF(pD, sinkDevice);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = DP_MST_DeallocateVcpi(pD, streamId, sinkDevice);
    }

    return retVal;
}

uint32_t DP_MstEnable(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_MstEnableSF(pD);

    if (retVal == CDN_EOK) {
        retVal = DP_MST_SetMstEnable(pD, true);
    }

    return retVal;
}

uint32_t DP_MstDisable(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_MstDisableSF(pD);

    if (retVal == CDN_EOK) {
        retVal = DP_MST_SetMstEnable(pD, false);
    }

    return retVal;
}

uint32_t DP_MstStreamEnable(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;

    retVal = DP_MstStreamEnableSF(pD);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = DP_MST_SetStreamEnable(pD, streamId, true);
    }

    return retVal;
}

uint32_t DP_MstStreamDisable(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;

    retVal = DP_MstStreamDisableSF(pD);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = DP_MST_SetStreamEnable(pD, streamId, false);
    }

    return retVal;
}

uint32_t DP_SetAudioVideoClkCfg(DP_PrivateData*            pD,
                                uint8_t                    streamId,
                                const DP_AudioVideoClkCfg* audioVideoClkCfg)
{
    uint32_t retVal;

    retVal = DP_SetAudioVideoClkCfgSF(pD, streamId, audioVideoClkCfg);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = setPktClock(pD, streamId, audioVideoClkCfg->pktDataClockEnable);
    }
    if (retVal == CDN_EOK) {
        retVal = setAifClock(pD, streamId, audioVideoClkCfg->audioClockEnable);
    }
    if (retVal == CDN_EOK) {
        retVal = setVifClock(pD, streamId, audioVideoClkCfg->videoClockEnable);
    }

    return retVal;
}

uint32_t DP_GetAudioVideoClkCfg(DP_PrivateData*      pD,
                                uint8_t              streamId,
                                DP_AudioVideoClkCfg* audioVideoClkCfg)
{
    uint32_t retVal;

    retVal = DP_GetAudioVideoClkCfgSF(pD, streamId, audioVideoClkCfg);

    if (retVal == CDN_EOK) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = getPktClock(pD, streamId, &audioVideoClkCfg->pktDataClockEnable);
    }
    if (retVal == CDN_EOK) {
        retVal = getAifClock(pD, streamId, &audioVideoClkCfg->audioClockEnable);
    }
    if (retVal == CDN_EOK) {
        retVal = getVifClock(pD, streamId, &audioVideoClkCfg->videoClockEnable);
    }

    return retVal;
}

uint32_t DP_GetHdcpClockConfig(DP_PrivateData* pD, bool *hdcpClockEnable)
{
    uint32_t retVal;
    uint32_t clockEnabled;
    uint32_t reg;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_GetHdcpClockConfigSF(pD, hdcpClockEnable);

    if (retVal == CDN_EOK) {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.source_cipher_car_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    if (retVal == CDN_EOK) {
        reg = regTransfer.val;
        clockEnabled = CPS_FLD_READ(MHDP__MHDP_APB_REGS__SOURCE_CIPHER_CAR_P, SOURCE_CIPHER_CHAR_CLK_EN, reg);

        if (clockEnabled == 0U) {
            *hdcpClockEnable = false;
        } else {
            *hdcpClockEnable = true;
        }
    }

    return retVal;
}

uint32_t DP_SetHdcpClockConfig(DP_PrivateData* pD, bool hdcpClockEnable)
{
    uint32_t retVal;
    uint8_t clockEnable = (hdcpClockEnable) ? 1U : 0U;

    retVal = DP_SetHdcpClockConfigSF(pD);

    if (CDN_EOK == retVal) {
        retVal = setCipherClockConfig(pD, clockEnable);
    }

    if (CDN_EOK == retVal) {
        retVal = setCryptoClockConfig(pD, clockEnable);
    }

    return retVal;
}

uint32_t DP_MstGetSinkCount(DP_PrivateData *pD, uint8_t *sinkCount) {
    MST_drm_dp_port* portList[DP_MAX_NUMBER_OF_STREAMS];
    uint32_t retVal;

    retVal = DP_MstGetSinkCountSF(pD, sinkCount);

    if (retVal == CDN_EOK) {
        *sinkCount = 0U;
        retVal = collectSinkPortList(pD->mstTopMgr.mst_primary,
                                     (const MST_drm_dp_port**) portList,
                                     sinkCount);
    }

    if (retVal == CDN_EOK) {
        removeDisconnectedSinks(pD, (const MST_drm_dp_port**) portList, *sinkCount);

        retVal = updateConnectedSinks(pD, portList, *sinkCount);
    }

    return retVal;
}

uint32_t DP_MstGetSinkList(const DP_PrivateData *pD, const DP_SinkDevice** sinkList) {
    uint32_t retVal;
    uint8_t i;
    uint8_t sinkNumber = 0U;

    retVal = DP_MstGetSinkListSF(pD, sinkList);

    if (retVal == CDN_EOK) {
        for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {
            if (pD->sinkList[i].port != NULL) {
                sinkList[sinkNumber] = &pD->sinkList[i];
                sinkNumber++;
            }
        }
    }

    return retVal;
}

uint32_t DP_MstSetEncryption(DP_PrivateData* pD, uint8_t streamId, bool enable)
{
    uint32_t retVal;

    retVal = DP_MstSetEncryptionSF(pD);

    if (retVal == CDN_EOK) {
        retVal = DP_MST_SetEncryption(pD, streamId, enable);
    }

    return retVal;
}

uint32_t DP_MstSetEncryptionEnable(DP_PrivateData* pD, bool enable)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    retVal = DP_MstSetEncryptionEnableSF(pD);

    if (retVal == CDN_EOK) {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_MTPH_ECF_SLOTS_31_0_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    if (retVal == CDN_EOK) {

        if (enable) {
            regTransfer.val |= 1U;
        } else {
            regTransfer.val &= ~1U;
        }

        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    return retVal;
}

uint32_t DP_MstReadRemoteEdid(const DP_PrivateData* pD,
                              DP_SinkDevice *       sinkDevice,
                              uint32_t              block,
                              DP_ReadEdidResponse * edidResponse)
{
    uint32_t retVal;

    retVal = DP_MstReadRemoteEdidSF(pD, sinkDevice, edidResponse);

    if (retVal == CDN_EOK) {
        retVal = drm_do_probe_ddc_edid(sinkDevice->port,
                                       edidResponse->buff,
                                       block,
                                       edidResponse->size);
    }

    if (retVal == CDN_EOK) {
        edidResponse->blockNo = (uint8_t)block;
    }

    return retVal;
}

uint32_t DP_MstScanTopology(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_MstScanTopologySF(pD);

    if (retVal == CDN_EOK) {
        retVal = drm_dp_mst_link_probe_work(&pD->mstTopMgr);
    }

    return retVal;
}

uint32_t DP_MstHpdIrq(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_MstHpdIrqSF(pD);

    if (retVal == CDN_EOK) {
        (void)drm_dp_mst_handle_hpd(&pD->mstTopMgr);
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRICS-41 */
