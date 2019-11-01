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
 * dp_sdp.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-41 "Number of blocks comments per statement" */

#include "dp_if.h"
#include "dp_priv.h"
#include "dp_mst.h"
#include "dp_utils.h"
#include "dp_sanity.h"
#include "mhdp_apb_regs.h"

#include "cdn_errno.h"
#include "cdn_stdint.h"

static uint32_t getActiveIdleBit(DP_SdpActiveIdleMode activeMode) {
    uint32_t activeIdleBit = 0U;
    if (DP_SDP_ACTIVE_VIDEO == activeMode) {
        activeIdleBit = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, ACTIVE_IDLE_TYPE, 0U, 1U);
    }
    return activeIdleBit;
}

static void writeEntryId(DP_PrivateData* pD, uint8_t streamId, uint8_t entryId) {

    /* write entry id */
    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_WR_ADDR_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_WR_ADDR_P, WR_ADDR, 0U, entryId));
    /* write request */
    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_WR_REQ_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_WR_REQ_P, HOST_WR, 0U, 1U));
}

static void writeDataToPacketMemory(DP_PrivateData* pD, uint8_t streamId, uint8_t size, const uint32_t* buffer) {

    uint8_t i;

    for (i = 0U; i < size; i++) {
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_DATA_WR_p,
                      buffer[i]);
    }
}

static void updateEntry(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID, const DP_SdpEntry* packetData) {

    uint32_t activeIdleBit = getActiveIdleBit(packetData->activeMode);

    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PKT_ALLOC_REG_p,
                  activeIdleBit |
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, TYPE_VALID, 0U, 1U) |
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, PACKET_TYPE, 0U, packetData->type) |
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, PKT_ALLOC_ADDRESS, 0U, entryID));

    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PKT_ALLOC_WR_EN_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_WR_EN_P, PKT_ALLOC_WR_EN, 0U, 1U));

    /* save packet type for given stream and given entryID */
    /* packet is neccessary to proper removing */
    pD->sdpPacketType[streamId][entryID] = packetData->type;
}
static void setPpsHeader(DP_PrivateData* pD, uint8_t streamId, uint32_t header) {

    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PPS_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PPS_P, PPS, 0U, 1U));

    /* write PPS header */
    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PPS_HEADER_p,
                  header);
}

static void setSdpEntry(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID, const DP_SdpEntry* packetData) {

    uint8_t index = 0U;

    /* flush fifo 1 */
    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_FIFO1_FLUSH_p,
                  CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_FIFO1_FLUSH_P, FIFO1_FLUSH, 0U, 1U));

    if (packetData->type == 0x10U) {    /* PPS */
        setPpsHeader(pD, streamId, packetData->packet[0]);
        index = 1U;
    }

    writeDataToPacketMemory(pD, streamId, (packetData->length - index), &packetData->packet[index]);

    writeEntryId(pD, streamId, entryID);

    updateEntry(pD, streamId, entryID, packetData);

}

uint32_t DP_SetSdp(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID, const DP_SdpEntry* packetData)
{
    uint32_t retVal = CDN_EOK;

    if ((NULL == pD) || (NULL == packetData)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        /* invalidate entry */
        retVal = DP_RemoveSdp(pD, streamId, entryID);
    }

    if (CDN_EOK == retVal) {
        setSdpEntry(pD, streamId, entryID, packetData);
    }

    return retVal;

}

uint32_t DP_RemoveSdp(DP_PrivateData* pD, uint8_t streamId, uint8_t entryID)
{
    uint32_t retVal = CDN_EOK;

    if (NULL == pD)
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        retVal = DP_StreamIdMstSstSanity(pD, streamId);
    }

    if (CDN_EOK == retVal) {
        retVal = DP_RemoveSdpSF(pD, entryID);
    }

    if (CDN_EOK == retVal)
    {
        /* invalidate entry */
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PKT_ALLOC_REG_p,
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, ACTIVE_IDLE_TYPE, 0U, 1U) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, PACKET_TYPE, 0U, pD->sdpPacketType[streamId][entryID]) |
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_REG_P, PKT_ALLOC_ADDRESS, 0U, entryID));
        CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.mhdp_sdp_control[streamId].SOURCE_PIF_PKT_ALLOC_WR_EN_p,
                      CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_SDP_CONTROL__SOURCE_PIF_PKT_ALLOC_WR_EN_P, PKT_ALLOC_WR_EN, 0U, 1U));

        /* reset packet type to 0 */
        pD->sdpPacketType[streamId][entryID] = 0U;
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-41 "Number of blocks comments per statement" */
