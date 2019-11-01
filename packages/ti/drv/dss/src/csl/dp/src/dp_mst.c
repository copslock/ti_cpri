/******************************************************************************
 *
 * Copyright (C) 2014-2018 Cadence Design Systems, Inc.
 * All rights reserved worldwide
 * The material contained herein is the proprietary and confidential
 * information of Cadence or its licensors, and is supplied subject to, and may
 * be used only by Cadence's customer in accordance with a previously executed
 * license and maintenance agreement between Cadence and that customer.
 *
 ******************************************************************************
 *
 * dp_mst.c
 *
 ******************************************************************************
 */

#include "dp_if.h"
#include "dp_mst_if.h"
#include "dp_priv.h"
#include "dp_internal.h"
#include "dp_mst.h"
#include "dp_mst_structs_if.h"
#include "dp_topology_mgr.h"
#include "dp_utils.h"
#include "dp_register.h"
#include "mhdp_apb_regs.h"
#include "cdn_errno.h"
#include "cdn_math.h"
#include "cdn_log.h"
#include "cps_drv.h"
#include "custom_types.h"

typedef struct DP_RateGovConfig_s {
    uint32_t targAvSlotsX;
    uint32_t targAvSlotsY;
    uint32_t rateGovEn;
} DP_RateGovConfig;

typedef struct DP_SlotsCfg_s {
    uint8_t slots;
    float32_t fracSlots;
    uint32_t payloadBandwidth;
} DP_SlotsCfg;

uint32_t DP_StreamIdMstSanity(const DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t ret;

    /* Fail check for SST mode. */
    if (!pD->mstEnabled) {
        ret = CDN_ENOTSUP;
    }
    /* test if streamId is greater than max allowed by hardware */
    else if (streamId > (pD->hwConfig.videoStreams - 1U)) {
        ret = CDN_EINVAL;
    }
    else {
        ret = CDN_EOK;
    }

    return ret;
}

uint32_t DP_StreamIdMstSstDscSanity(const DP_PrivateData* pD,
                                    uint8_t               streamId,
                                    bool                  splitPanel)
{
    uint8_t maxStreamId = 0U;
    uint32_t ret = CDN_EOK;

    /* For Split Panel mode, only stream 0 can be used for DSC. */
    if ((pD->mstEnabled) && (!splitPanel)) {
        maxStreamId = DP_NUMBER_OF_DSC_ENCODERS - 1U;
    }

    /* test if streamId is greater than max allowed */
    if (streamId > maxStreamId) {
        ret = CDN_EINVAL;
    }

    return ret;
}

uint32_t DP_StreamIdMstSstSanity(const DP_PrivateData* pD, uint8_t streamId)
{
    uint8_t maxStreamId = 0U;
    uint32_t ret = CDN_EOK;

    /* With MST disabled, only stream 0 can be used. */
    if (pD->mstEnabled) {
        maxStreamId = pD->hwConfig.videoStreams - 1U;
    }

    /* test if streamId is greater than allowed by hardware */
    if (streamId > maxStreamId) {
        ret = CDN_EINVAL;
    }

    return ret;
}

/*
 * Calculate bits-per-component to bits-pet-pixel
 * @param[in] bitsPerComponent number of bits-per-component
 * return number of bits-per-pixel
 */
static inline uint8_t bpcToBpp(float64_t bitsPerComponent)
{
    /* For RGB and YCbCr 4:4:4 */
    float64_t result = bitsPerComponent * 3.0;
    return (uint8_t)result;
}

/*
 * Calculate time slots for sink's stream
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId number of stream
 * @param[in] port pointer to sink port object
 * @param[in] slotsCfg pointer to slot configuration object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if parameters are invalid
 * @return CDN_ENOSPEC if not enough slots
 */
static uint32_t calculateSlots(DP_PrivateData*  pD,
                               uint8_t          streamId,
                               MST_drm_dp_port* port,
                               DP_SlotsCfg*     slotsCfg)
{
    uint32_t ret;
    float64_t pixelClock = pD->videoParameters[streamId].vicParams.pxlFreq * 1000.0;
    float64_t bitsPerComponent = calculateBitsPerComponent(pD, streamId);
    uint8_t bitPerPixel = bpcToBpp(bitsPerComponent);

    slotsCfg->payloadBandwidth = DP_MST_MgrCalcPbnMode((uint32_t)pixelClock,
                                                       bitPerPixel,
                                                       pD->fecEnabled);

    ret = DP_MST_MgrFindVcpiSlots(&pD->mstTopMgr,
                                  port,
                                  slotsCfg->payloadBandwidth,
                                  &(slotsCfg->slots),
                                  &(slotsCfg->fracSlots));

    return ret;
}

/*
 * Set payload bandwidth for topology
 * @param[in] pD pointer to DP private data object
 */
static void setPayloadBandwidth(DP_PrivateData* pD)
{
    uint16_t symbolRate = getSymbolRate(pD->linkState.linkRate);

    DP_MST_MgrSetPayloadBandwidth(&pD->mstTopMgr,
                                  symbolRate,
                                  pD->linkState.laneCount);
}

/*
 * Read data from global config to check, if uCPU has finished writing
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if writing was not finished
 */
static uint32_t finishVcpiAllocation(DP_PrivateData* pD)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
    return DP_ReadRegister(pD, &regTransfer);
}

/*
 * Check, if video on given stream is on
 * @param[in] pD pointer to private data object
 * @param[in] streamId number of stream
 * @param[in, out] isOn is video or not
 * @return CDN_EOK if success
 * @return other error code if cannot read config register
 */
static uint32_t isVideoOn(DP_PrivateData* pD, uint8_t streamId, bool *isOn)

{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    /* read configuration for stream */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].STREAM_CONFIG_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {
        uint32_t videoBit = CPS_FLD_READ(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                                         STREAM_CONFIG_P__NO_VIDEO,
                                         regTransfer.val);
        /* set isOn due to videoBit */
        if (videoBit != 0U) {
            *isOn = false;
        } else {
            *isOn = true;
        }
    }

    return retVal;
}

/* Set slot allocation for particular stream in DP controller
 * @param[in] pD pointer to private data object
 * @param[in] streamId number of stream
 * @param[in] startSlot number of first slot
 * @param[in] endSlot number of last slot
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot do R/W operation
 */
static uint32_t setSlotAllocation(DP_PrivateData* pD,
                                  uint8_t         streamId,
                                  uint8_t         startSlot,
                                  uint8_t         endSlot)
{
    uint32_t reg = 0U;
    DP_RegisterTransfer regTransfer = {0U};

    /* set start slot */
    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                        DP_MST_SLOT_ALLOCATE_P__STREAM_START_SLOT,
                        reg,
                        startSlot);
    /* set end slot */
    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                        DP_MST_SLOT_ALLOCATE_P__STREAM_END_SLOT,
                        reg,
                        endSlot);

    /* write slot allocation configuration */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_MST_SLOT_ALLOCATE_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val = reg;

    return DP_WriteRegister(pD, &regTransfer);
}

/* Get slot allocation for particular stream in DP controller
 * @param[in] pD pointer to private data object
 * @param[in] streamId number of stream
 * @param[in] startSlot number of first slot
 * @param[in] endSlot number of last slot
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot do read data from register
 */
static uint32_t getSlotAllocation(DP_PrivateData* pD,
                                  uint8_t         streamId,
                                  uint8_t *       startSlot,
                                  uint8_t *       endSlot)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    /* read slot allocation from register */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].DP_MST_SLOT_ALLOCATE_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {

        *startSlot = (uint8_t)CPS_FLD_READ(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                                           DP_MST_SLOT_ALLOCATE_P__STREAM_START_SLOT,
                                           regTransfer.val);

        *endSlot = (uint8_t)CPS_FLD_READ(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                                         DP_MST_SLOT_ALLOCATE_P__STREAM_END_SLOT,
                                         regTransfer.val);
    }

    return retVal;
}

/* Calculate Target average number of slots per MTP configuration parameters
 * (with fractional part)
 * @param[in] targetAverageSlots
 * @param[out] targetAverageSlotsX
 * @param[out] targetAverageSlotsY
 */
static void calcRateGoverningComponents(float64_t         targetAverageSlots,
                                        DP_RateGovConfig* rateCfg)
{
    float64_t targetAverageSlotsFraction;
    float64_t yReal;

    /* Truncate */
    rateCfg->targAvSlotsX = (uint32_t)targetAverageSlots;
    targetAverageSlotsFraction = targetAverageSlots - (float64_t)rateCfg->targAvSlotsX;
    yReal = targetAverageSlotsFraction * 16.0;

    /* Round up */
    rateCfg->targAvSlotsY = (uint32_t)ceil(yReal);

    DbgMsg(DBG_GEN_MSG, DBG_FYI, "Rate governing components : X = %0d (%f) Y = %0d (%f)",
           rateCfg->targAvSlotsX, targetAverageSlots,
           rateCfg->targAvSlotsY, yReal);

}

/*
 * Calculate video FIFO latency threshold.
 * @param[in] targetAverageSlots average number of slots
 * @param[in] lanesNumber number of lanes
 * @return threshold
 */
static uint32_t calculateThreshold(float64_t targetAverageSlots, uint8_t lanesNumber)
{
    uint32_t threshold;
    float64_t targetEntries;
    float64_t threshTemp;

    /* calculate threshold */
    targetEntries  = targetAverageSlots / 2.0;
    threshTemp    = targetEntries - ((targetEntries * targetEntries) / 32.0);
    threshold = (uint32_t)ceil((threshTemp * (float64_t)lanesNumber) / 4.0);

    if (threshold < 2U) {
        threshold = 2U;
    }

    return threshold;
}

/*
 * Start ACT sequence
 * @param[in] pD pointer to DP private data object
 * @return CDN_EOK if success
 * @return CDN_EIO if cannot read payload table
 * @return CDN_EINVAL if cannot start sequence
 */
static uint32_t startActSequence(DP_PrivateData* pD)
{
    uint32_t retVal;
    DP_RegisterTransfer regTransfer = {0U};

    /* read MTPH control register */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_MTPH_CONTROL_p);
    retVal = DP_ReadRegister(pD, &regTransfer);

    /* set ACT bit in MTPH register */
    if (retVal == CDN_EOK) {
        regTransfer.val = CPS_FLD_SET(MHDP__MHDP_APB_REGS__DP_MTPH_CONTROL_P, MTPH_ACT_EN, regTransfer.val);
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_MTPH_CONTROL_p);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    DbgMsg(DBG_GEN_MSG, DBG_FYI, "ACT enabled in MTPH control\n");

#ifndef IS_SPEEDBRIDGE

    if (retVal == CDN_EOK) {
        uint32_t actStatus;
        do {
            regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_MTPH_STATUS_p);
            retVal = DP_ReadRegister(pD, &regTransfer);

            if (retVal != CDN_EOK) {
                break;
            }

            actStatus = CPS_FLD_READ(MHDP__MHDP_APB_REGS,
                                     DP_MTPH_STATUS_P__MTPH_ACT_STATUS,
                                     regTransfer.val);

        } while (actStatus != 0U);
    }

#endif

    /* check ACT status */
    if (retVal == CDN_EOK) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "ACT status in source is set\n");
        retVal = DP_MST_MgrCheckActStatus(&pD->mstTopMgr);
    }

    return retVal;
}

/*
 * Set configuration of range governing
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] cfg pointer to rate configuration object
 * @return CDN_EOK if success
 * @return CD_EINVAL if cannot set
 */
static uint32_t setRateGoverningConfig(DP_PrivateData *pD, uint8_t streamId, const DP_RateGovConfig *cfg)
{
    uint32_t retVal;
    uint32_t reg = 0U;
    DP_RegisterTransfer regTransfer = {0U};

    /* set X average slots */
    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                        RATE_GOVERNING_CTRL_P__TARG_AV_SLOTS_X,
                        reg,
                        cfg->targAvSlotsX);
    /* set Y average slots */
    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                        RATE_GOVERNING_CTRL_P__TARG_AV_SLOTS_Y,
                        reg,
                        cfg->targAvSlotsY);

    if (cfg->rateGovEn != 0U) {
        /* enable rate governing */
        reg = CPS_FLD_SET(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM,
                          RATE_GOVERNING_CTRL_P__RATE_GOV_EN,
                          reg);
    }

    /* write rate governing configuration */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].RATE_GOVERNING_CTRL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    regTransfer.val  = reg;

    retVal = DP_WriteRegister(pD, &regTransfer);

    return retVal;
}

/* Set video FIFO latency threshold based on fracSlots for given stream.
 * @param[in] pD pointer to private data object
 * @param[in] streamId number of streams
 * @param[in] fracSlots value of fractional part of slot
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot write data
 */
static uint32_t setLineThreshold(DP_PrivateData *pD, uint8_t streamId, float64_t fracSlots)
{
    DP_RegisterTransfer regTransfer = {0U};

    regTransfer.val = calculateThreshold(fracSlots, pD->linkState.laneCount);
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].LINE_THRESH_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);

    return DP_WriteRegister(pD, &regTransfer);
}

/*
 * Calculate start and end slot for payload
 * @param[in] payload pointer to MST payload object
 * @param[out] startSlot start slot number
 * @param[out] endSlot end slot number
 */
static inline void calculateSlotRange(const MST_drm_dp_payload *payload, uint8_t* startSlot, uint8_t* endSlot)
{
    *startSlot = payload->start_slot;
    *endSlot = (*startSlot + payload->num_slots) - 1U;
}

/*
 * Get payload (for port) and update if differences
 * @param[in] pD pointer to private data object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot set new allocation
 */
static uint32_t updateSlotAllocation(DP_PrivateData *pD)
{
    uint32_t retVal = CDN_EOK;
    MST_drm_dp_payload *payload;
    uint8_t startSlot;
    uint8_t endSlot;
    uint8_t i;

    for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {

        if (pD->sinkList[i].port == NULL) {
            continue;
        }

        /* get payload configuration from topology manager */
        payload = DP_MST_MgrGetPortPayload(&pD->mstTopMgr, pD->sinkList[i].port);
        if (payload == NULL) {
            continue;
        }

        /* get payload configuration from DPTX registers */
        retVal = getSlotAllocation(pD, pD->sinkList[i].streamId, &startSlot, &endSlot);

        if (retVal == CDN_EOK) {
            /* if payload configuration in topology manager is different than in DPTX registers
             *  then update DPTX registers */
            if (startSlot != payload->start_slot) {
                calculateSlotRange(payload, &startSlot, &endSlot);
                retVal = setSlotAllocation(pD, pD->sinkList[i].streamId, startSlot, endSlot);
            }
        }

        if (retVal != CDN_EOK) {
            break;
        }
    }

    return retVal;

}

/*
 * Clear slot allocation for stream
 * @param[in] pD pointer to private data object
 * @param[in] streamId stream number
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot reset
 */
static uint32_t clearSlotAllocation(DP_PrivateData* pD, uint8_t streamId)
{
    /* clear slot allocation in source registers */
    uint32_t retVal = setSlotAllocation(pD, streamId, 0U, 0U);

    if (retVal == CDN_EOK) {
        retVal = updateSlotAllocation(pD);
    }

    if (retVal == CDN_EOK) {
        retVal = startActSequence(pD);
    }

    return retVal;
}

/*
 * Clear SDP packets for stream
 * @param[in] pD pointer to private data object
 * @param[in] streamId stream number
 * @return CDN_EOK if success
 * @return CDN_EINVAL if parameters are invalid
 */
static uint32_t clearSdpPackets(DP_PrivateData *pD, uint8_t streamId)
{
    uint8_t i;
    uint32_t retVal;

    /* clear each SDP from stream */
    for (i = 0U; i < MST_NUMBER_OF_SDP; i++) {
        retVal = DP_RemoveSdp(pD, streamId, i);

        if (CDN_EOK != retVal) {
            break;
        }
    }

    return retVal;
}

/*
 * Calculate and set rate governing components
 * @param[in] pD pointer to private data object
 * @param[in] streamId stream number
 * @param[in] fracSlots fractional part of slots
 * @retrun CDN_EOK if success
 * @return CDN_EINVAL if cannot set rate governing
 */
static uint32_t configureRateGoverning(DP_PrivateData* pD, uint8_t streamId, float64_t fracSlots)
{
    DP_RateGovConfig rateGovConfig;

    calcRateGoverningComponents(fracSlots, &rateGovConfig);

    rateGovConfig.rateGovEn = 1U;

    return setRateGoverningConfig(pD, streamId, &rateGovConfig);

}

/*
 * Set clear rate governing components
 * @param[in] pD pointer to private data object
 * @param[in] streamId stream number
 * @retrun CDN_EOK if success
 * @return CDN_EINVAL if cannot reset rate governing
 */
static inline uint32_t clearRateGoverning(DP_PrivateData* pD, uint8_t streamId)
{
    DP_RateGovConfig rateGovConfig = {0U};

    return setRateGoverningConfig(pD, streamId, &rateGovConfig);
}

/*
 * Returns pointer to sink device
 * @param[in] pD pointer to private data object
 * @param[in] streamId stream number
 * @retrun pointer to sink device if found
 * @return NULL if sink device was not found
 */
/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass parameter 'pD' as const, DRV-3895" */
static DP_SinkDevice* findSinkDevice(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t i;
    DP_SinkDevice *sinkDevice = NULL;

    /* look for sink device in each stream */
    for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {
        if ((pD->sinkList[i].port != NULL) && (streamId == pD->sinkList[i].streamId)) {
            sinkDevice = &pD->sinkList[i];
            break;
        }
    }

    return sinkDevice;
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

/*
 * Set or reset bits of payload, due to enable flag
 * @param[in, out] payload actual payload
 * @param[in] mask mask value
 * @param[in] set if payload is mask directly (set = 'true') or by negation (set = 'false')
 */
static void maskPayload(uint32_t* payload, uint32_t mask, bool set)
{
    if (set) {
        *payload |= mask;
    } else {
        *payload &= ~mask;
    }
}

/*
 * Set timeslots for MST which should be encrypted
 * @param[in] pD pointer to DP private data object
 * @param[in] payload_31_0 payload for slots 0 - 31
 * @param[in] payload_32_63 payload for slots 32 - 63
 * @retrun CDN_EOK if success
 * @return CDN_EINVAL if parameters are invalid
 */
static uint32_t setEncryption(DP_PrivateData* pD, uint32_t payload_31_0,
                              uint32_t payload_63_32, bool enable)
{
    uint32_t retVal;
    uint8_t i;

    uint32_t payload[] = {payload_31_0, payload_63_32};

    /* array of payload registers addresses */
    uint32_t payloadAddr[] = {
        offsetof(MHDP_ApbRegs,
                 mhdp_apb_regs.DP_MTPH_ECF_SLOTS_31_0_p),
        offsetof(MHDP_ApbRegs,
                 mhdp_apb_regs.DP_MTPH_ECF_SLOTS_63_32_p)
    };

    uint8_t size = (uint8_t)(sizeof(payload) / sizeof(payload[0]));

    DP_RegisterTransfer regTransfer = {0U};

    for (i = 0U; i < size; i++) {

        /* read actual state of encryption */
        regTransfer.addr = payloadAddr[i];
        retVal = DP_ReadRegister(pD, &regTransfer);

        if (retVal == CDN_EOK) {

            /* set new encryption */
            maskPayload(&regTransfer.val, payload[i], enable);
            retVal = DP_WriteRegister(pD, &regTransfer);

        } else {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Set encryption failed, cannot read register\n");
        }

        if (retVal != CDN_EOK) {
            break;
        }
    }

    /* start ACT sequence */
    if (retVal == CDN_EOK) {
        retVal = startActSequence(pD);
    }

    return retVal;
}

/*
 * Calculate timeslots for payload
 * @param[in] payload pointer to MST payload object
 * @param[out] payload_31_0 payload for slots 0 - 31
 * @param[out] payload_32_63 payload for slots 32 - 63
 * @return CDN_EOK if success
 * @return CDN_EINVAL if endSlot greater than max
 */
static uint32_t calculateTimeSlotsForPayload(const MST_drm_dp_payload *payload, uint32_t* payload_31_0, uint32_t* payload_63_32)
{
    uint8_t i;
    uint8_t endSlot;
    uint8_t startSlot;
    uint64_t fullPayload = 0U;
    uint32_t retVal = CDN_EOK;

    calculateSlotRange(payload, &startSlot, &endSlot);

    /* test if endSlot is not greater than max */
    if (endSlot > 63U) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {

        /* generate payload */
        for (i = startSlot; i <= endSlot; i++) {
            /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 "Shift operation should be enclosed, DRV-3822" */
            fullPayload |= (uint64_t)1U << i;
            /* parasoft-end-suppress MISRA2012-RULE-12_2-2 */
        }

        /* truncate payloads */
        *payload_31_0 = (uint32_t)fullPayload;
        *payload_63_32 = (uint32_t)(fullPayload >> 32U);
    }

    return retVal;
}

/*
 * Set slots allocation for given source stream
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] port pointer to MST port object
 * @return CDN_EOK if success
 * @retrun CDN_EINVAL if cannot set
 */
static uint32_t setSourceSlotsAllocation(DP_PrivateData* pD, uint8_t streamId, MST_drm_dp_port* port)
{
    uint32_t retVal = CDN_EOK;
    MST_drm_dp_payload *payload = NULL;
    uint8_t startSlot;
    uint8_t endSlot;

    payload = DP_MST_MgrGetPortPayload(&pD->mstTopMgr, port);
    if (payload == NULL) {
        retVal = CDN_EINVAL;
    }

    if (retVal == CDN_EOK) {
        /* set slot allocation in source registers */
        calculateSlotRange(payload, &startSlot, &endSlot);
        retVal = setSlotAllocation(pD, streamId, startSlot, endSlot);
    }

    if (retVal == CDN_EOK) {
        retVal = startActSequence(pD);
    }

    return retVal;
}

/*
 * Configure payload for MST virtual channel
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] port pointer to MST port object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot configure payload
 */
static uint32_t configurePayload(DP_PrivateData* pD, uint8_t streamId, MST_drm_dp_port* port)
{
    /* update first part of payload */
    uint32_t retVal = DP_MST_MgrUpdatePayloadPart1(&pD->mstTopMgr);

    /* set slots allocation for port */
    if (retVal == CDN_EOK) {
        retVal = setSourceSlotsAllocation(pD, streamId, port);
    }

    /* update second part of payload */
    if (retVal == CDN_EOK) {
        retVal = DP_MST_MgrUpdatePayloadPart2(&pD->mstTopMgr);
    }

    return retVal;
}

uint32_t DP_MST_SetStreamEnable(DP_PrivateData* pD, uint8_t streamId, bool streamEnable)
{
    uint32_t retVal;
    uint32_t reg;
    DP_RegisterTransfer regTransfer = {0U};

    /* read current stream configuration */
    regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_dptx_stream[0].STREAM_CONFIG_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_dptx_stream[0]);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (retVal == CDN_EOK) {

        reg = regTransfer.val;

        /* enable stream and clear no-vieo if 'enable' is true */
        if (streamEnable) {
            reg = CPS_FLD_SET(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM, STREAM_CONFIG_P__STREAM_EN, reg);
            reg = CPS_FLD_CLEAR(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM, STREAM_CONFIG_P__NO_VIDEO, reg);
        } else {
            reg = CPS_FLD_CLEAR(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM, STREAM_CONFIG_P__STREAM_EN, reg);
            reg = CPS_FLD_SET(MHDP__MHDP_APB_REGS__MHDP_DPTX_STREAM, STREAM_CONFIG_P__NO_VIDEO, reg);
        }

        /* write current stream configuration */
        regTransfer.val = reg;
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (retVal == CDN_EOK) {
        /* set streamEnabled flag in pD */
        pD->streamEnabled[streamId] = streamEnable;
    }

    return retVal;
}

uint32_t DP_MST_AllocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice)
{
    DP_SlotsCfg slotsCfg = {0};
    uint32_t ret;

    setPayloadBandwidth(pD);

    ret = calculateSlots(pD, streamId, sinkDevice->port, &slotsCfg);

    if (ret == CDN_EOK) {
        /* allocate virtual channel */
        ret = DP_MST_MgrAllocateVcpi(&pD->mstTopMgr,
                                     sinkDevice->port,
                                     slotsCfg.payloadBandwidth,
                                     slotsCfg.slots);
    }

    if (ret == CDN_EOK) {
        ret = configurePayload(pD, streamId, sinkDevice->port);
    }

    if (ret == CDN_EOK) {
        /* configure rate governing for stream */
        ret = configureRateGoverning(pD, streamId, slotsCfg.fracSlots);
    }

    if (ret == CDN_EOK) {
        /* set threshold for stream */
        ret = setLineThreshold(pD, streamId, slotsCfg.fracSlots);
    }

    /* assign streamId to sink device */
    sinkDevice->streamId = streamId;

    if (ret == CDN_EOK) {
        /* check if allocation was finished */
        ret = finishVcpiAllocation(pD);
    }

    return ret;
}

uint32_t DP_MST_DeallocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice)
{
    MST_drm_dp_topology_mgr* topologyManager = &(pD->mstTopMgr);

    DP_MST_MgrDeallocateVcpi(&pD->mstTopMgr, sinkDevice->port);

    uint32_t retVal = DP_MST_MgrUpdatePayloadPart1(topologyManager);

    if (retVal == CDN_EOK) {
        /* clear slots allocation for stream */
        retVal = clearSlotAllocation(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        retVal = DP_MST_MgrUpdatePayloadPart2(topologyManager);
    }

    if (retVal == CDN_EOK) {
        /* clear rate govern configuration */
        retVal = clearRateGoverning(pD, streamId);
    }

    if (retVal == CDN_EOK) {
        /* clear SDP packets from current stream */
        retVal = clearSdpPackets(pD, streamId);
    }

    return retVal;
}

uint32_t DP_MST_SetMstEnable(DP_PrivateData* pD, bool mstEnable)
{
    uint32_t retVal;
    uint32_t reg;
    DP_RegisterTransfer regTransfer = {0U};

    /* set MST state */
    retVal = DP_MST_MgrSetState(&pD->mstTopMgr, mstEnable);

    if (retVal != CDN_EOK) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "MST set state to %d failed\n", mstEnable);
    }

    if (retVal == CDN_EOK) {
        /* read global configuration for framer */
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);
        retVal = DP_ReadRegister(pD, &regTransfer);
    }

    if (retVal == CDN_EOK) {

        /* set MST enable/disable */
        if (!mstEnable) {
            reg = CPS_FLD_CLEAR(MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P, MST_SST, regTransfer.val);
        } else {
            reg = CPS_FLD_SET(MHDP__MHDP_APB_REGS__DP_FRAMER_GLOBAL_CONFIG_P, MST_SST, regTransfer.val);
        }

        regTransfer.val  = reg;
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.DP_FRAMER_GLOBAL_CONFIG_p);

        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (retVal == CDN_EOK) {
        /* set 'mstEnable' flag in private data */
        pD->mstEnabled = mstEnable;
    }

    return retVal;
}

uint32_t DP_MST_SetEncryption(DP_PrivateData* pD, uint8_t streamId, bool enable)
{
    uint32_t retVal = CDN_EOK;
    uint32_t payload_31_0 = 0U;
    uint32_t payload_63_32 = 0U;
    MST_drm_dp_payload *payload;

    /* looking for sink device on stream */
    DP_SinkDevice* sinkDevice = findSinkDevice(pD, streamId);

    if (sinkDevice == NULL) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Sink device not exists\n");
        retVal = CDN_EINVAL;
    }

    if (retVal == CDN_EOK) {
        /* get payload for sink */
        payload = DP_MST_MgrGetPortPayload(&pD->mstTopMgr, sinkDevice->port);

        if (payload == NULL) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Payload for given stream not found\n");
            retVal = CDN_EINVAL;
        }
    }

    if (retVal == CDN_EOK) {
        /* calculate timeslots for payload */
        retVal = calculateTimeSlotsForPayload(payload, &payload_31_0, &payload_63_32);
    }

    if (retVal == CDN_EOK) {
        /* set encryption */
        retVal = setEncryption(pD, payload_31_0, payload_63_32, enable);
    }

    return retVal;
}

uint32_t DP_MST_IsAnyVideoOn(DP_PrivateData* pD, bool *isOn)
{
    uint8_t i;
    uint32_t retVal;

    /* check each stream */
    for (i = 0U; i < DP_MAX_NUMBER_OF_STREAMS; i++) {
        retVal = isVideoOn(pD, i, isOn);

        /* break if any stream have video on */
        if ((retVal != CDN_EOK) || (*isOn)) {
            break;
        }

    }

    return retVal;
}
