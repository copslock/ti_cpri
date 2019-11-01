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
 * dp_topology_mgr.c
 *
 ******************************************************************************
 */

/*
 * Copyright (C) 2014 Red Hat
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <stddef.h>
#include <stdlib.h>
#include "cdn_math.h"
#include "cdn_stdtypes.h"
#include "cps.h"
#include "dp_aux.h"
#include "dp_sideband_msg.h"
#include "dp_topology_mgr.h"
#include "dp_topology_utils.h"
#include "dp_transaction.h"
#include "cdn_log.h"

/**
 * DOC: dp mst helper
 *
 * These functions contain parts of the DisplayPort 1.2a MultiStream Transport
 * protocol. The helpers contain a topology manager and bandwidth manager.
 * The helpers encapsulate the sending and received of sideband msgs.
 */

/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement in the function" */

/* XXX: Linux macro, but it is idiom */
static inline uint32_t DIV_ROUND_UP (uint32_t numerator, uint32_t denominator) {
    return (numerator + (denominator - 1U )) / denominator;
}

/* parasoft-begin-suppress METRICS-36 "Function called from more than 5 different functions in one translation unit, DRV-3823" */
/**
 * Set requested bit in 64-bit value.
 */
static void set_bit(uint8_t bit, uint64_t *address) {
    if (bit < 64U) {
        *address |= (1UL << bit);
    }
}
/* parasoft-end-suppress METRICS-36-3 */

/**
 * Clear requested bit in 64-bit value.
 */
static void clear_bit(uint8_t bit, uint64_t *address) {
    if (bit < 64U) {
        *address &= ~(1UL << bit);
    }
}

/**
 * Find position of first cleared bit in 64-bit value, starting from lsb.
 */
static uint8_t find_first_zero_bit(const uint64_t *address, uint8_t size) {
    uint8_t i;
    uint8_t clampedSize = size;

    if (size > 64U)
    {
        clampedSize = 64U;
    }

    for (i = 0; i < clampedSize; i++)
    {
        if (i < 64U)
        {
            if (0U == (*address & (1UL << i)))
            {
                break;
            }
        }
    };

    /* Will return size or '64', if zero was not found. */
    return i;
}

static uint32_t drm_dp_dpcd_write_payload(MST_drm_dp_topology_mgr *mgr,
                                          int32_t id, const MST_drm_dp_payload *payload);

static uint32_t drm_dp_mst_assign_payload_id(MST_drm_dp_topology_mgr *mgr,
                                             MST_drm_dp_vcpi *        vcpi) {
    uint32_t retVal = CDN_EOK;
    uint8_t ret;
    uint8_t vcpi_ret;

    /* Find position of first unused Payload ID. */
    ret = find_first_zero_bit(&mgr->payload_mask, MST_MAX_PAYLOAD + 1U);
    if (ret > MST_MAX_PAYLOAD) {
        retVal = CDN_EINVAL;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "out of payload ids %d\n", ret);
    }

#ifdef IS_DEMO_TB
    if (!(mgr->vcpi_mask & (1 << (mgr->demotb.requestedVcpi - 1)))) {
        vcpi_ret = mgr->demotb.requestedVcpi - 1;
    } else {
        retVal = CDN_EINVAL;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Requested VCPI %d is in use\n",
               mgr->demotb.requestedVcpi);
    }
#else
    /* Find position of first unused VCPI. */
    vcpi_ret = find_first_zero_bit(&mgr->vcpi_mask, MST_MAX_PAYLOAD + 1U);
    if (vcpi_ret > MST_MAX_PAYLOAD) {
        retVal = CDN_EINVAL;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "out of vcpi ids %d\n", ret);
    }
#endif

    if (retVal == CDN_EOK) {
        /* Set Payload ID as used. */
        set_bit(ret, &mgr->payload_mask);
        /* Set VCPI as used. */
        set_bit(vcpi_ret, &mgr->vcpi_mask);
        vcpi->vcpi = (int32_t)vcpi_ret + 1;
        mgr->proposed_vcpis[ret - 1U] = vcpi;
    }

    return retVal;
}

static void drm_dp_mst_put_payload_id(MST_drm_dp_topology_mgr *mgr,
                                      int32_t                  vcpi) {

    uint32_t i;
    if (vcpi != 0) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "putting payload %d\n", vcpi);
        /* Set Payload ID as unused. */
        clear_bit((uint8_t)vcpi - 1U, &mgr->vcpi_mask);

        for (i = 0; i < MST_MAX_PAYLOAD; i++) {
            if (mgr->proposed_vcpis[i] != NULL) {
                if (mgr->proposed_vcpis[i]->vcpi == vcpi) {
                    mgr->proposed_vcpis[i] = NULL;
                    /* Set Payload ID as used. */
                    clear_bit((uint8_t)i + 1U, &mgr->payload_mask);
                }
            }
        }
    }
}

static uint32_t drm_dp_create_payload_step1(MST_drm_dp_topology_mgr *mgr,
                                            int32_t id, MST_drm_dp_payload *payload) {
    uint32_t ret;

    /* Write Payload-related data to DPCD and wait for sink to acknowledge the */
    /* change. */
    ret = drm_dp_dpcd_write_payload(mgr, id, payload);
    if (CDN_EOK != ret) {
        payload->payload_state = 0;
    } else {
        payload->payload_state = (int32_t)MST_PAYLOAD_LOCAL;
    }
    return ret;
}

static uint32_t drm_dp_create_payload_step2(MST_drm_dp_topology_mgr *mgr,
                                            MST_drm_dp_port *port, int32_t id, MST_drm_dp_payload *payload) {
    uint32_t ret = CDN_EOK;

#ifdef IS_DEMO_TB
    if (!mgr->demotb.disableMessaging) {
#endif
    /* Send Payload-related data through sideband channel */
    ret = drm_dp_payload_send_msg(mgr, port, id, port->vcpi.pbn);
#ifdef IS_DEMO_TB
}
#endif
    if (ret == 0U) {
        payload->payload_state = (int32_t)MST_PAYLOAD_REMOTE;
    }
    return ret;
}

static uint32_t drm_dp_destroy_payload_step1(MST_drm_dp_topology_mgr *mgr,
                                             MST_drm_dp_port *port, int32_t id, MST_drm_dp_payload *payload) {

    uint32_t ret = CDN_EOK;
    DbgMsg(DBG_GEN_MSG, DBG_FYI, "\n");
    /* its okay for these to fail */

    if (port != NULL) {
#ifdef IS_DEMO_TB
        if (!mgr->demotb.disableMessaging) {
#endif
        ret = drm_dp_payload_send_msg(mgr, port, id, 0);
#ifdef IS_DEMO_TB
    }
#endif
    }

    if (ret == CDN_EOK) {
        /* Write Payload-related data to DPCD and wait for sink to acknowledge */
        /* the change. */
        ret = drm_dp_dpcd_write_payload(mgr, id, payload);
        if (ret != CDN_EOK) {
            payload->payload_state = 0;
        } else {
            payload->payload_state = (int32_t)MST_PAYLOAD_DELETE_LOCAL;
        }
    }

    return ret;
}

/**
 * Check, if a given bit is set in 64-bit mask, with required size checks.
 */
static inline bool is64bBitSet(uint64_t mask, uint32_t bit)
{
    bool result = false;
    if (bit < 64U) {
        if (0U != (mask & (1ULL << bit))) {
            result = true;
        }
    }
    return result;
}

/**
 * solve the current payloads - compare to the hw ones - update the hw view.
 */
static inline uint32_t solvePayloads(MST_drm_dp_topology_mgr *mgr,
                                     MST_drm_dp_payload *     req_payload,
                                     MST_drm_dp_port **       port,
                                     int32_t                  cur_slots,
                                     uint32_t                 payloadId)
{
    uint32_t ret = CDN_EOK;
    MST_drm_dp_port **portTmp = port;

    req_payload->start_slot = (uint8_t)cur_slots;
    if (mgr->proposed_vcpis[payloadId] != NULL) {
        *port = mgr->proposed_vcpis[payloadId]->parent;
        *portTmp = DRM_DP_getValidatedPortRef(mgr, *port);
        *port = *portTmp;
        if (*port == NULL) {
            ret = CDN_EINVAL;
        }
        req_payload->num_slots = (uint8_t)mgr->proposed_vcpis[payloadId]->num_slots;
        req_payload->vcpi = mgr->proposed_vcpis[payloadId]->vcpi;
    } else {
        *port = NULL;
        req_payload->num_slots = 0U;
    }

    if (mgr->payloads[payloadId].start_slot != req_payload->start_slot) {
        mgr->payloads[payloadId].start_slot = req_payload->start_slot;
    }

    return ret;
}

/**
 * Push an update for given payload.
 */
static inline uint32_t pushPayloadUpdate(MST_drm_dp_topology_mgr *mgr,
                                         MST_drm_dp_port *        port,
                                         MST_drm_dp_payload *     req_payload,
                                         uint32_t                 payloadId)
{
    uint32_t ret = CDN_EOK;
    /* Get pointer to currently processed payload */
    MST_drm_dp_payload *currPayload = &(mgr->payloads[payloadId]);

    if (req_payload->num_slots > 0U) {
        ret = drm_dp_create_payload_step1(mgr, mgr->proposed_vcpis[payloadId]->vcpi, req_payload);
        if (CDN_EOK == ret) {
            currPayload->num_slots = req_payload->num_slots;
            currPayload->vcpi = req_payload->vcpi;
        }
    } else if (currPayload->num_slots > 0U) {
        currPayload->num_slots = 0;
        ret = drm_dp_destroy_payload_step1(mgr, port, currPayload->vcpi, currPayload);
        if (CDN_EOK == ret)
        {
            req_payload->payload_state = currPayload->payload_state;
            currPayload->start_slot = 0;
        }
    } else {
        /* TODO provide else */
    }
    return ret;
}

/**
 * Remove payloads, that were marked for deletion.
 */
static inline void removeMarkedPayloads(MST_drm_dp_topology_mgr *mgr)
{
    uint32_t i, j;

    for (i = 0; i < MST_MAX_PAYLOAD; i++) {
        if (mgr->payloads[i].payload_state == (int32_t)MST_PAYLOAD_DELETE_LOCAL) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "removing payload %d\n", i);
            /* Move subsequent payloads to the front (beginning) */
            for (j = i; j < (MST_MAX_PAYLOAD - 1U); j++) {
                (void)memcpy(&mgr->payloads[j], &mgr->payloads[j + 1U], sizeof(MST_drm_dp_payload));
                mgr->proposed_vcpis[j] = mgr->proposed_vcpis[j + 1U];
                if ((mgr->proposed_vcpis[j] != NULL) && (mgr->proposed_vcpis[j]->num_slots != 0)) {
                    set_bit((uint8_t)j + 1U, &mgr->payload_mask);
                } else {
                    clear_bit((uint8_t)j + 1U, &mgr->payload_mask);
                }
            }
            /* Clear structure of last (now unused) payload. */
            (void)memset(&mgr->payloads[MST_MAX_PAYLOAD - 1U], 0, sizeof(MST_drm_dp_payload));
            mgr->proposed_vcpis[MST_MAX_PAYLOAD - 1U] = NULL;
            clear_bit(MST_MAX_PAYLOAD, &mgr->payload_mask);
        }
    }
}

/**
 * drm_dp_update_payload_part1() - Execute payload update part 1
 * @mgr: manager to use.
 *
 * This iterates over all proposed virtual channels, and tries to
 * allocate space in the link for them. For 0->slots transitions,
 * this step just writes the VCPI to the MST device. For slots->0
 * transitions, this writes the updated VCPIs and removes the
 * remote VC payloads.
 *
 * after calling this the driver should generate ACT and payload
 * packets.
 */
uint32_t DP_MST_MgrUpdatePayloadPart1(MST_drm_dp_topology_mgr *mgr)
{
    uint32_t ret = CDN_EOK;
    uint32_t i;
    int32_t cur_slots = 1;
    MST_drm_dp_payload req_payload;
    MST_drm_dp_port *port;
    req_payload.payload_state = 0; /* TODO: kwitos, confirm it, initialize it prior to use */

    for (i = 0; i < MST_MAX_PAYLOAD; i++) {

        ret = solvePayloads(mgr, &req_payload, &port, cur_slots, i);
        if (CDN_EOK != ret) {
            break;
        }

        /* work out what is required to happen with this payload */
        if (mgr->payloads[i].num_slots != req_payload.num_slots) {

            ret =  pushPayloadUpdate(mgr, port, &req_payload, i);

            if (CDN_EOK == ret)
            {
                mgr->payloads[i].payload_state = req_payload.payload_state;
            }
        }
        if (CDN_EOK == ret)
        {
            cur_slots += (int32_t)req_payload.num_slots;

            if (port != NULL) {
                DRM_DP_putPort(port);
            }
        }
    }

    if (ret == CDN_EOK) {
        removeMarkedPayloads(mgr);
    }

    return ret;
}

/**
 * drm_dp_update_payload_part2() - Execute payload update part 2
 * @mgr: manager to use.
 *
 * This iterates over all proposed virtual channels, and tries to
 * allocate space in the link for them. For 0->slots transitions,
 * this step writes the remote VC payload commands. For slots->0
 * this just resets some internal state.
 */
uint32_t DP_MST_MgrUpdatePayloadPart2(MST_drm_dp_topology_mgr *mgr)
{
    MST_drm_dp_port *port;
    uint8_t i;
    uint32_t ret = CDN_EOK;

    for (i = 0; i < MST_MAX_PAYLOAD; i++) {

        if (mgr->proposed_vcpis[i] == NULL) {
            continue;
        }
        port = mgr->proposed_vcpis[i]->parent;

        DbgMsg(DBG_GEN_MSG, DBG_FYI, "payload %d %d\n", i, mgr->payloads[i].payload_state);
        if (mgr->payloads[i].payload_state == (int32_t)MST_PAYLOAD_LOCAL) {
            ret = drm_dp_create_payload_step2(mgr, port, mgr->proposed_vcpis[i]->vcpi, &mgr->payloads[i]);
        } else if (mgr->payloads[i].payload_state == (int32_t)MST_PAYLOAD_DELETE_LOCAL) {
            mgr->payloads[i].payload_state = 0;
        } else {
            /* TODO: check other states */
        }
        if (ret != CDN_EOK) {
            ret = CDN_EINVAL;
        }
    }
    return (uint32_t)ret;
}

MST_drm_dp_payload *DP_MST_MgrGetPortPayload(MST_drm_dp_topology_mgr *mgr,
                                             MST_drm_dp_port *        port) {
    uint8_t i;
    MST_drm_dp_payload *payload = NULL;
    MST_drm_dp_port *mport = NULL;

    mport = DRM_DP_getValidatedPortRef(mgr, port);
    if (mport != NULL) {
        int32_t vcpi_value = mport->vcpi.vcpi - 1;
        if (is64bBitSet(mgr->vcpi_mask, (uint32_t)vcpi_value)) {
            for (i = 0; i < MST_MAX_PAYLOAD; i++) {
                if ((mgr->payloads[i].vcpi == mport->vcpi.vcpi) &&
                    (is64bBitSet(mgr->payload_mask, (uint32_t)i))) {
                    payload = &mgr->payloads[i];
                    break;
                }
            }
        }
        DRM_DP_putPort(mport);
    }
    return payload;
}

void DP_MST_MgrSetPayloadBandwidth(MST_drm_dp_topology_mgr *mgr,
                                   uint32_t                 symbolRate,
                                   uint8_t                  laneCount) {

    uint32_t pbn_div = (laneCount * symbolRate) / MST_LINK_RATE_MULTIPLIER; /* TODO kwitos: confirm */
    mgr->pbn_div = (int32_t)pbn_div;
    DbgMsg(DBG_GEN_MSG, DBG_FYI, "laneCount %d, symbolRate %d\n",
           laneCount, symbolRate);
}

/**
 * drm_dp_topology_mgr_set_mst() - Set the MST state for a topology manager
 * @mgr: manager to set state for
 * @mst_state: true to enable MST on this connector - false to disable.
 *
 * This is called by the driver when it detects an MST capable device plugged
 * into a DP MST capable port, or when a DP MST capable device is unplugged.
 */
uint32_t DP_MST_MgrSetState(MST_drm_dp_topology_mgr *mgr, bool mst_state)
{
    uint32_t ret = CDN_EOK;
    MST_drm_dp_branch *mstb = NULL;
    MST_drm_dp_branch **primary = &(mgr->mst_primary);

    if (mst_state == mgr->mst_state) {
        /* CDN_EAGAIN is used to return CDN_EOK at the end, but skipping */
        /* execution of code. */
        ret = CDN_EAGAIN;
    } else {
        mgr->mst_state = mst_state;
        /* set the device into MST mode */
        if (mst_state) {
            /* add initial branch device at LCT 1 */
            mstb = DRM_DP_addBranchDevice(mgr, 1, NULL);
            if (mstb == NULL) {
                ret = CDN_ENOMEM;
            }
            if (CDN_EOK == ret) {
                mstb->mgr = mgr;

                /* give this the main reference */
                *primary = mstb;
                (*primary)->refcount++;

                ret = drm_dp_dpcd_write_byte(mgr->aux,
                                             MST_DPCD_MSTM_CTRL,
                                             MST_DPCD_MST_EN | MST_DPCD_UP_REQ_EN | MST_DPCD_UPSTREAM_IS_SRC);
                if (ret != CDN_EOK) {
                    ret = CDN_EIO;
                } else {
                    MST_drm_dp_payload reset_pay;
                    reset_pay.start_slot = 0;
                    reset_pay.num_slots = 0x3f;
                    /*FIXME result of this function is not checked*/
                    ret = drm_dp_dpcd_write_payload(mgr, 0, &reset_pay);
                }

            }
        } else {
            /* disable MST on the device */
            mstb = *primary;
            *primary = NULL;
            /* this can fail if the device is gone */
            ret = drm_dp_dpcd_write_byte(mgr->aux, MST_DPCD_MSTM_CTRL, 0);
            (void)memset(mgr->payloads, 0, MST_MAX_PAYLOAD * sizeof(MST_drm_dp_payload));
            mgr->payload_mask = 0;
            set_bit(0, &mgr->payload_mask);
            mgr->vcpi_mask = 0;
        }
    }

    if (mstb != NULL) {
        DRM_DP_putBranchDevice(mstb);
    }

    return ((CDN_EAGAIN == ret) ? CDN_EOK : ret);
}

/**
 * drm_dp_mst_hpd_irq() - MST hotplug IRQ notify
 * @mgr: manager to notify irq for.
 * @esi: 4 bytes from SINK_COUNT_ESI
 * @handled: whether the hpd interrupt was consumed or not
 *
 * This should be called from the driver when it detects a short IRQ,
 * along with the value of the DEVICE_SERVICE_IRQ_VECTOR_ESI0. The
 * topology manager will process the sideband messages received as a result
 * of this.
 */
void drm_dp_mst_hpd_irq(MST_drm_dp_topology_mgr *mgr, const uint8_t *esi,
                        bool *handled) {
    uint8_t sc;
    *handled = false;
    sc = esi[0] & 0x3fU;

    if (sc != mgr->sink_count) {
        mgr->sink_count = sc;
        *handled = true;
    }

    if ((esi[1] & DP_DOWN_REP_MSG_RDY) != 0U) {
        drm_dp_mst_handle_down_rep(mgr);
        *handled = true;
    }

    if ((esi[1] & DP_UP_REQ_MSG_RDY) != 0U) {
        drm_dp_mst_handle_up_req(mgr);
        *handled = true;
    }

}

#define DDC_SEGMENT_ADDR 0x30U
#define EDID_LENGTH 128U
#define DDC_ADDR 0x50U

uint32_t drm_do_probe_ddc_edid(MST_drm_dp_port *port, uint8_t *buf, uint32_t block,
                               uint8_t len)
{
    uint8_t start = (uint8_t)(block * EDID_LENGTH);
    uint8_t segment = (uint8_t)block >> 1U;
    uint8_t xfers = (segment != 0U) ? 3U : 2U;
    uint8_t retries = 5;
    uint32_t ret;

    /*
     * The core I2C driver will automatically retry the transfer if the
     * adapter reports CDN_EAGAIN. However, we find that bit-banging transfers
     * are susceptible to errors under a heavily loaded machine and
     * generate spurious NAKs and timeouts. Retrying the transfer
     * of the individual block a few times seems to overcome this.
     */
    do {
        struct mst_i2c_message msgs[3];

        msgs[0].address = DDC_SEGMENT_ADDR;
        msgs[0].length  = 1U;
        msgs[0].data    = &segment;

        msgs[1].address = DDC_ADDR;
        msgs[1].length  = 1U;
        msgs[1].data    = &start;

        msgs[2].address = DDC_ADDR;
        msgs[2].length  = len;
        msgs[2].data    = buf;

        /*
         * Avoid sending the segment addr to not upset non-compliant
         * DDC monitors.
         */
        ret = drm_dp_mst_i2c_xfer(port, &msgs[3U - xfers], xfers);
        retries--;
    } while ((ret != xfers) && (retries > 0U));

    return (ret == xfers) ? CDN_EOK : CDN_EINVAL;
}

static uint32_t drm_dp_init_vcpi(MST_drm_dp_topology_mgr *mgr,
                                 MST_drm_dp_vcpi *        vcpi,
                                 uint32_t                 pbn,
                                 uint32_t                 slots,
                                 MST_drm_dp_port*         port) {
    uint32_t ret = CDN_EOK;

    /* max. time slots - one slot for MTP header */
    if (slots > 63U) {
        ret = CDN_ENOSPC;
    } else {
        vcpi->pbn = (int32_t)pbn;
        vcpi->aligned_pbn = (int32_t)slots * mgr->pbn_div;
        vcpi->num_slots = (int32_t)slots;
        vcpi->parent = port;
        ret = drm_dp_mst_assign_payload_id(mgr, vcpi);
    }
    return ret;
}

/**
 * drm_dp_atomic_find_vcpi_slots() - Find and add vcpi slots to the state
 * @state: global atomic state
 * @mgr: MST topology manager for the port
 * @port: port to find vcpi slots for
 * @pbn: bandwidth required for the mode in PBN
 *
 * RETURNS:
 * Total slots in the atomic state assigned for this port or error
 */
/* int drm_dp_atomic_release_vcpi_slots(struct drm_atomic_state *state,
   MST_drm_dp_topology_mgr *mgr,
   int slots) */
uint32_t DP_MST_MgrFindVcpiSlots(MST_drm_dp_topology_mgr *mgr,
                                 MST_drm_dp_port *port, uint32_t pbn, uint8_t *slots, float32_t *fracSlots)
{
    uint32_t retVal = CDN_EOK;
    MST_drm_dp_port *mport = NULL;

    mport = DRM_DP_getValidatedPortRef(mgr, port);
    if ((mport == NULL) || (fracSlots == NULL) || (slots == NULL)) {
        retVal = CDN_EINVAL;
    }
    if (retVal == CDN_EOK) {
        *fracSlots = (float32_t)pbn / (float32_t)mgr->pbn_div;
        *slots = (uint8_t)(DIV_ROUND_UP(pbn, (uint32_t)mgr->pbn_div));

        DbgMsg(DBG_GEN_MSG, DBG_FYI, "pbn %d, pbn_div %d\n",
               pbn, mgr->pbn_div);

        DbgMsg(DBG_GEN_MSG, DBG_FYI, "floating slots=%f\n", *fracSlots);

        DbgMsg(DBG_GEN_MSG, DBG_FYI, "vcpi slots req=%d, avail=%d\n", *slots, mgr->avail_slots);

        if (*slots > (uint8_t)mgr->avail_slots) {
            retVal = CDN_ENOSPC;
        } else {
            mgr->avail_slots -= (int32_t)*slots;
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "vcpi slots avail=%d", mgr->avail_slots);
        }
    }

    DRM_DP_putPort(mport);
    return retVal;
}

/**
 * drm_dp_mst_allocate_vcpi() - Allocate a virtual channel
 * @mgr: manager for this port
 * @port: port to allocate a virtual channel for.
 * @pbn: payload bandwidth number to request
 * @slots: returned number of slots for this PBN.
 */
uint32_t DP_MST_MgrAllocateVcpi(MST_drm_dp_topology_mgr *mgr,
                                MST_drm_dp_port *port, uint32_t pbn, uint8_t slots) {
    uint32_t ret = CDN_EOK;
    MST_drm_dp_port *mport = NULL;
    mport = DRM_DP_getValidatedPortRef(mgr, port);

    if ((mport != NULL) && (mport->vcpi.vcpi > 0)) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI,
               "payload: vcpi %d already allocated for pbn %d - requested pbn %d\n",
               mport->vcpi.vcpi, mport->vcpi.pbn, pbn);
        if (pbn == (uint32_t)(mport->vcpi.pbn)) {
            DRM_DP_putPort(mport);
        }
    }

    if ((mport != NULL) && (pbn != (uint32_t)(mport->vcpi.pbn))) {
        DbgMsg(DBG_GEN_MSG, DBG_FYI, "initing vcpi for pbn=%d slots=%d\n", pbn, mport->vcpi.num_slots);
        ret = drm_dp_init_vcpi(mgr, &mport->vcpi, pbn, slots, mport);
        if (ret != CDN_EOK) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to init vcpi slots=%d max=63 ret=%d\n",
                   DIV_ROUND_UP(pbn, mgr->pbn_div), ret);
        } else {
            DRM_DP_putPort(mport);
        }
    }

    return ret;
}

/**
 * drm_dp_mst_deallocate_vcpi() - deallocate a VCPI
 * @mgr: manager for this port
 * @portIn: unverified port to deallocate vcpi for
 * */
void DP_MST_MgrDeallocateVcpi(MST_drm_dp_topology_mgr *mgr,
                              MST_drm_dp_port *        portIn) {
    MST_drm_dp_port *port = DRM_DP_getValidatedPortRef(mgr, portIn);
    if (NULL != port) {
        drm_dp_mst_put_payload_id(mgr, port->vcpi.vcpi);
        port->vcpi.num_slots = 0;
        port->vcpi.pbn = 0;
        port->vcpi.aligned_pbn = 0;
        port->vcpi.vcpi = 0;
        port->parent = NULL;
        DRM_DP_putPort(port);
    }
}

static uint32_t drm_dp_dpcd_write_payload(MST_drm_dp_topology_mgr *mgr,
                                          int32_t id, const MST_drm_dp_payload *payload) {
    uint8_t payload_alloc[3], status;
    uint32_t ret;
    uint32_t retries = 20;
    bool readError = false;

#ifdef IS_DEMO_TB
    retries = 1;
#endif

    ret = drm_dp_dpcd_write_byte(mgr->aux, MST_DPCD_PAYLOAD_TABLE_UPDATE_STATUS,
                                 MST_DPCD_PAYLOAD_TABLE_UPDATED);

    if (CDN_EOK == ret)
    {
        payload_alloc[0] = (uint8_t)id;
        payload_alloc[1] = (uint8_t)(payload->start_slot);
        payload_alloc[2] = (uint8_t)(payload->num_slots);

        ret = drm_dp_dpcd_write(mgr->aux, MST_DPCD_PAYLOAD_ALLOCATE_SET, payload_alloc,
                                3);
        if (ret != CDN_EOK) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to write payload allocation %d\n", ret);
        }
    }

    if (CDN_EOK == ret) {
        while (retries > 0U)
        {
            ret = drm_dp_dpcd_read_byte(mgr->aux, MST_DPCD_PAYLOAD_TABLE_UPDATE_STATUS,
                                        &status);
            if (ret != CDN_EOK) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to read payload table status %d\n", ret);
                readError = true;
            }

            /* Break if read error occurred, of if desired bits are set. */
            if (readError || (0U != (status & MST_DPCD_PAYLOAD_TABLE_UPDATED))) {
                break;
            }
            retries--;
            if (retries > 0U) {

                CPS_DelayNs(10000000);
                continue;
            }
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "status not set after read payload table status %d\n", status);
            ret = CDN_EINVAL;
        }
    }
    return ret;
}

/**
 * drm_dp_check_act_status() - Check ACT handled status.
 * @mgr: manager to use
 *
 * Check the payload status bits in the DPCD for ACT handled completion.
 */
uint32_t DP_MST_MgrCheckActStatus(MST_drm_dp_topology_mgr *mgr) {
    uint8_t status;
    uint32_t ret = CDN_EOK;
    uint32_t count = 0;

    do {
        ret = drm_dp_dpcd_read_byte(mgr->aux, MST_DPCD_PAYLOAD_TABLE_UPDATE_STATUS,
                                    &status);

        if (ret != CDN_EOK) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to read payload table status %d\n", ret);
            ret = CDN_EIO;
        }

        if (CDN_EOK == ret)
        {
            if (0U != (status & MST_DPCD_PAYLOAD_ACT_HANDLED)) {
                break;
            }
            count++;
            CPS_DelayNs(100000);
        }

    } while ((CDN_EOK == ret) && (count < 30U));

    if (CDN_EOK == ret)
    {
        if ((status & MST_DPCD_PAYLOAD_ACT_HANDLED) == 0U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to get ACT bit %d after %d retries\n", status, count);
            ret = CDN_EINVAL;
        }
    }

    return ret;
}

/**
 * drm_dp_calc_pbn_mode() - Calculate the PBN for a mode.
 * @dot_clock: dot clock for the mode
 * @bpp: bpp for the mode.
 *
 * This uses the formula in the spec to calculate the PBN value for a mode.
 */
uint32_t DP_MST_MgrCalcPbnMode(uint32_t dot_clock, uint8_t bpp, bool fecEnabled)
{
    float64_t kbps;
    float64_t numerator;
    float64_t denominator;

    DbgMsg(DBG_GEN_MSG, DBG_FYI, "clock %d, bpp %d\n",
           dot_clock, bpp);
    kbps = (float64_t)dot_clock * (float64_t)bpp;
    numerator = 64.0 * 1006.0;
    denominator = 54.0 * 8.0 * 1000.0 * 1000.0;
    if (fecEnabled) {
        numerator *= 100.0;
        denominator *= (100.0 - 3.0);
    }
    kbps *= numerator;
    return (uint32_t)ceil(kbps / denominator);
}

static uint32_t test_calc_pbn_mode(void)
{
    uint32_t res;
    uint32_t retVal = CDN_EOK;
    res = DP_MST_MgrCalcPbnMode(154000, 30, false);
    if (res != 689U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "PBN calculation test failed - clock %d, bpp %d, expected PBN %d, actual PBN %d.\n",
               154000, 30, 689, res);
        retVal = CDN_EINVAL;
    }
    if (CDN_EOK == retVal) {
        res = DP_MST_MgrCalcPbnMode(234000, 30, false);
        if (res != 1047U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "PBN calculation test failed - clock %d, bpp %d, expected PBN %d, actual PBN %d.\n",
                   234000, 30, 1047, res);
            retVal = CDN_EINVAL;
        }
    }
    if (CDN_EOK == retVal) {
        res = DP_MST_MgrCalcPbnMode(297000, 24, false);
        if (res != 1063U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "PBN calculation test failed - clock %d, bpp %d, expected PBN %d, actual PBN %d.\n",
                   297000, 24, 1063, res);
            retVal = CDN_EINVAL;
        }
    }
    return retVal;
}

/**
 * drm_dp_topology_mgr_init - initialise a topology manager
 * @mgr: manager struct to initialise
 * @dev: device providing this structure - for i2c addition.
 * @aux: DP helper aux channel to talk to this device
 * @max_dpcd_transaction_bytes: hw specific DPCD transaction limit
 * @max_payloads: maximum number of payloads this GPU can source
 * @conn_base_id: the connector object ID the MST device is connected to.
 *
 */
void DP_MST_MgrInit(MST_drm_dp_topology_mgr *mgr,
                    AUX_drm_dp *aux, uint32_t max_dpcd_transaction_bytes) {

    mgr->tx_msg_downq = NULL;
    mgr->int_enabled = false;
    mgr->mst_primary = NULL;

    mgr->aux = aux;
    mgr->max_dpcd_transaction_bytes = max_dpcd_transaction_bytes;

    (void)memset(mgr->payloads, 0, sizeof(mgr->payloads));
    (void)memset(mgr->proposed_vcpis, 0, sizeof(mgr->proposed_vcpis));
    mgr->payload_mask = 0;
    mgr->vcpi_mask = 0;
    set_bit(0, &mgr->payload_mask);

    if (test_calc_pbn_mode() != 0U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "MST PBN self-test failed\n");
    }

    mgr->avail_slots = 63;
    mgr->mst_state = false;

}

/* parasoft-end-suppress METRICS-41-3 */
