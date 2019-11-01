/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits" */
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
 * dp_transaction.c
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

#include <stdlib.h>
#include <cdn_log.h>
#include "cps.h"
#include "dp_aux.h"
#include "dp_aux_if.h"
#include "dp_transaction.h"
#include "dp_sideband_msg_if.h"
#include "dp_topology_utils.h"

/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */
/* parasoft-begin-suppress METRICS-36-3 "Function called from more than 5 different functions, DRV-3823" */

static uint32_t drm_dp_send_dpcd_write(MST_drm_dp_topology_mgr *mgr,
                                       MST_drm_dp_port *port,
                                       uint32_t offset, uint8_t size, uint8_t *bytes);

static void drm_dp_send_enum_path_resources(MST_drm_dp_topology_mgr *mgr,
                                            MST_drm_dp_branch *mstb, MST_drm_dp_port *port);

static void drm_dp_send_link_address(MST_drm_dp_topology_mgr *topologyManager,
                                     MST_drm_dp_branch *      mstb);

static bool drm_dp_validate_guid(uint8_t *guid) {
    int32_t i;
    bool ret = false;

    for (i = 0; i < 16; i++) {
        if (guid[i] != 0U) {
            ret = true;
        }
    }

    if (!ret) {
        for (i = 0; i < 16; i++) {
            guid[i] = (uint8_t)i;
        }
    }
    return ret;
}

static void drm_dp_check_mstb_guid(MST_drm_dp_branch *mstb, uint8_t *guid) {

    (void)memcpy(mstb->guid, guid, 16);
    if (!drm_dp_validate_guid(mstb->guid)) {
        if (mstb->port_parent != NULL) {
            (void)drm_dp_send_dpcd_write(mstb->mgr, mstb->port_parent,
                                         DP_GUID, 16U, mstb->guid);
        } else {
            (void)drm_dp_dpcd_write(mstb->mgr->aux,
                                    DP_GUID, mstb->guid, 16);
        }
    }
}

static void encode_sb_req_alloc_payload(const SBM_drm_dp_sideband_msg_req_body *req, uint8_t *buf,
                                        uint8_t *idx) {
    const SBM_drm_dp_allocate_payload *allocate_payload = &req->u.allocate_payload;
    uint8_t i;

    buf[*idx] = (uint8_t) ((allocate_payload->port_number & 0xfU) << 4U)
                | (allocate_payload->number_sdp_streams & 0xfU);
    buf[*idx + 1U] = (allocate_payload->vcpi & 0x7fU);
    buf[*idx + 2U] = (uint8_t) (allocate_payload->pbn >> 8);
    buf[*idx + 3U] = (uint8_t) (allocate_payload->pbn & 0xffU);
    (*idx) += 4U;
    for (i = 0U; i < (allocate_payload->number_sdp_streams / 2U); i++) {
        buf[*idx] = ((allocate_payload->sdp_stream_sink[i * 2U] & 0xfU) << 4)
                    | (allocate_payload->sdp_stream_sink[(i * 2U) + 1U] & 0xfU);
        (*idx)++;
    }
    if ((allocate_payload->number_sdp_streams & 1U) != 0U) {
        uint8_t j = allocate_payload->number_sdp_streams - 1U;
        buf[*idx] = (allocate_payload->sdp_stream_sink[j] & 0xfU) << 4;
        (*idx)++;
    }
}

static void encode_sb_req_rem_dpcd_read(const SBM_drm_dp_sideband_msg_req_body *req, uint8_t *buf,
                                        uint8_t *idx) {
    const SBM_drm_dp_remote_dpcd_read *remote_dpcd_read = &req->u.dpcd_read;

    buf[*idx] = (remote_dpcd_read->port_number & 0xfU) << 4;
    buf[*idx] |= (uint8_t) (((remote_dpcd_read->dpcd_address & 0xf0000U) >> 16) & 0xfU);
    (*idx)++;
    buf[*idx] = (uint8_t) ((remote_dpcd_read->dpcd_address & 0xff00U) >> 8);
    (*idx)++;
    buf[*idx] = (uint8_t) (remote_dpcd_read->dpcd_address & 0xffU);
    (*idx)++;
    buf[*idx] = remote_dpcd_read->num_bytes;
    (*idx)++;
}

static void encode_sb_req_rem_dpcd_write(const SBM_drm_dp_sideband_msg_req_body *req, uint8_t *buf,
                                         uint8_t *idx) {
    const SBM_drm_dp_remote_dpcd_write *remote_dpcd_write = &req->u.dpcd_write;

    buf[*idx] = (remote_dpcd_write->port_number & 0xfU) << 4;
    buf[*idx] |= (uint8_t) (((remote_dpcd_write->dpcd_address & 0xf0000U) >> 16) & 0xfU);
    (*idx)++;
    buf[*idx] = (uint8_t) ((remote_dpcd_write->dpcd_address & 0xff00U) >> 8);
    (*idx)++;
    buf[*idx] = (uint8_t) (remote_dpcd_write->dpcd_address & 0xffU);
    (*idx)++;
    buf[*idx] = (remote_dpcd_write->num_bytes);
    (*idx)++;
    (void) memcpy(&buf[*idx], remote_dpcd_write->bytes, remote_dpcd_write->num_bytes);
    *idx += remote_dpcd_write->num_bytes;
}

static void encode_sb_req_rem_i2c_read(const SBM_drm_dp_sideband_msg_req_body *req, uint8_t *buf,
                                       uint8_t *idx) {
    const SBM_drm_dp_remote_i2c_read *remote_i2c_read = &req->u.i2c_read;
    uint8_t i;

    buf[*idx] = (((remote_i2c_read->port_number & 0xfU) << 4)
                 | (remote_i2c_read->num_transactions & 0x3U));
    (*idx)++;
    for (i = 0U; i < (remote_i2c_read->num_transactions & 0x3U); i++) {
        const SBM_drm_dp_transactions *this_transaction = &remote_i2c_read->transactions[i];

        buf[*idx] = this_transaction->i2c_dev_id & 0x7fU;
        buf[*idx + 1U] = this_transaction->num_bytes;
        (*idx) += 2U;
        (void) memcpy(&buf[*idx], this_transaction->bytes,
                      this_transaction->num_bytes);
        *idx += this_transaction->num_bytes;

        buf[*idx] = (((this_transaction->no_stop_bit & 0x1U) << 5)
                     | (this_transaction->i2c_transaction_delay & 0xfU));
        (*idx)++;
    }
    buf[*idx] = (remote_i2c_read->read_i2c_device_id) & 0x7fU;
    buf[*idx + 1U] = (remote_i2c_read->num_bytes_read);
    (*idx) += 2U;
}

static void encode_sb_req_rem_i2c_write(const SBM_drm_dp_sideband_msg_req_body *req, uint8_t *buf,
                                        uint8_t *idx) {
    const SBM_drm_dp_remote_i2c_write *remote_i2c_write = &req->u.i2c_write;

    buf[*idx] = (remote_i2c_write->port_number & 0xfU) << 4;
    (*idx)++;
    buf[*idx] = (remote_i2c_write->write_i2c_device_id) & 0x7fU;
    (*idx)++;
    buf[*idx] = (remote_i2c_write->num_bytes);
    (*idx)++;
    (void) memcpy(&buf[*idx], remote_i2c_write->bytes, remote_i2c_write->num_bytes);
    *idx += remote_i2c_write->num_bytes;
}

static void drm_dp_encode_sideband_req(const SBM_drm_dp_sideband_msg_req_body *req,
                                       SBM_drm_dp_sideband_msg_tx *            raw) {
    uint8_t idx = 0;
    uint8_t *buf = raw->msg;
    buf[idx] = req->req_type & 0x7fU;
    idx++;

    switch (req->req_type) {
    case DP_ENUM_PATH_RESOURCES:
        buf[idx] = (req->u.port_num.port_number & 0xfU) << 4;
        idx++;
        break;
    case DP_ALLOCATE_PAYLOAD:
        encode_sb_req_alloc_payload(req, buf, &idx);
        break;
    case DP_REMOTE_DPCD_READ:
        encode_sb_req_rem_dpcd_read(req, buf, &idx);
        break;
    case DP_REMOTE_DPCD_WRITE:
        encode_sb_req_rem_dpcd_write(req, buf, &idx);
        break;
    case DP_REMOTE_I2C_READ:
        encode_sb_req_rem_i2c_read(req, buf, &idx);
        break;
    case DP_REMOTE_I2C_WRITE:
        encode_sb_req_rem_i2c_write(req, buf, &idx);
        break;

    default:
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Unknown req type %d \n", req->req_type);
        break;
    }
    raw->cur_len = idx;
}

static void drm_dp_encode_sideband_reply(
    const SBM_drm_dp_sideband_msg_reply_body *rep,
    SBM_drm_dp_sideband_msg_tx *              raw) {
    uint8_t idx = 0;
    uint8_t *buf = raw->msg;

    buf[idx] = ((rep->reply_type & 0x1U) << 7) | (rep->req_type & 0x7fU);
    idx++;

    raw->cur_len = idx;
}

static bool drm_dp_sb_parse_link_addr_iterative(
    SBM_drm_dp_sideband_msg_rx *raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg,
    uint32_t i, uint32_t *idx) {
    bool ret = true;

    if ((raw->msg[*idx] & 0x80U) != 0U) {
        repmsg->u.link_addr.ports[i].input_port = 1;
    }

    repmsg->u.link_addr.ports[i].peer_device_type = (raw->msg[*idx] >> 4) & 0x7U;
    repmsg->u.link_addr.ports[i].port_number = (raw->msg[*idx] & 0xfU);

    (*idx)++;
    if (*idx <= raw->curlen) {
        repmsg->u.link_addr.ports[i].mcs = ((raw->msg[*idx] >> 7) & 0x1U) != 0U;
        repmsg->u.link_addr.ports[i].ddps = ((raw->msg[*idx] >> 6) & 0x1U) != 0U;
        if (repmsg->u.link_addr.ports[i].input_port == 0) {
            repmsg->u.link_addr.ports[i].legacy_device_plug_status =
                ((raw->msg[*idx] >> 5) & 0x1U) != 0U;
        }
    }
    (*idx)++;
    if ((*idx <= raw->curlen) && (repmsg->u.link_addr.ports[i].input_port == 0)) {
        repmsg->u.link_addr.ports[i].dpcd_revision = (raw->msg[*idx]);
        (*idx)++;
        if (*idx <= raw->curlen) {
            (void) memcpy(repmsg->u.link_addr.ports[i].peer_guid, &raw->msg[*idx], 16);
        }
        *idx += 16U;
        if (*idx <= raw->curlen) {
            repmsg->u.link_addr.ports[i].num_sdp_streams = (raw->msg[*idx] >> 4) & 0xfU;
            repmsg->u.link_addr.ports[i].num_sdp_stream_sinks = (raw->msg[*idx] & 0xfU);
            (*idx)++;
        }
    }

    if (*idx > raw->curlen) {
        ret = false;
    }
    return ret;
}

static bool drm_dp_sideband_parse_link_address(
    SBM_drm_dp_sideband_msg_rx *        raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {

    bool ret = true;
    uint32_t idx = 1;
    uint32_t i;
    (void) memcpy(repmsg->u.link_addr.guid, &raw->msg[idx], 16);
    idx += 16U;
    repmsg->u.link_addr.nports = raw->msg[idx] & 0xfU;
    idx++;
    if (idx > raw->curlen) {
        ret = false;
    }
    for (i = 0; (i < repmsg->u.link_addr.nports) && ret; i++) {
        ret = drm_dp_sb_parse_link_addr_iterative(raw, repmsg, i, &idx);
    }

    if (!ret) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "link address reply parse length fail %d %d\n", idx,
               raw->curlen);
    }
    return ret;
}

static bool sideband_parse_remote_dpcd_read(
    SBM_drm_dp_sideband_msg_rx *        raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {
    bool ret;
    uint32_t idx = 1;
    repmsg->u.remote_dpcd_read_ack.port_number = raw->msg[idx] & 0xfU;
    idx++;
    if (idx > raw->curlen) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "link address reply parse length fail %d %d\n", idx,
               raw->curlen);
        ret = false;
    } else {
        repmsg->u.remote_dpcd_read_ack.num_bytes = raw->msg[idx];
        idx++;
        (void)memcpy(repmsg->u.remote_dpcd_read_ack.bytes, &raw->msg[idx],
                     repmsg->u.remote_dpcd_read_ack.num_bytes);
        ret = true;
    }
    return ret;
}

static bool sideband_parse_remote_dpcd_write(
    const SBM_drm_dp_sideband_msg_rx *  raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {
    bool ret;
    uint32_t idx = 1;
    repmsg->u.remote_dpcd_write_ack.port_number = raw->msg[idx] & 0xfU;
    idx++;
    if (idx > raw->curlen) {
        ret = false;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "parse length fail %d %d\n", idx, raw->curlen);
    } else {
        ret = true;
    }
    return ret;
}

static bool drm_dp_sideband_parse_remote_i2c_read_ack(
    SBM_drm_dp_sideband_msg_rx *        raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {
    bool ret;
    uint32_t idx = 1;
    repmsg->u.remote_i2c_read_ack.port_number = (raw->msg[idx] & 0xfU);
    idx++;
    if (idx > raw->curlen) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "remote i2c reply parse length fail %d %d\n", idx,
               raw->curlen);
        ret = false;
    } else {
        repmsg->u.remote_i2c_read_ack.num_bytes = raw->msg[idx];
        idx++;
        (void)memcpy(repmsg->u.remote_i2c_read_ack.bytes, &raw->msg[idx],
                     repmsg->u.remote_i2c_read_ack.num_bytes);
        ret = true;
    }
    return ret;
}

static bool drm_dp_sideband_parse_enum_path_resources_ack(
    const SBM_drm_dp_sideband_msg_rx *  raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {

    bool ret = true;
    uint32_t idx = 1;
    repmsg->u.path_resources.port_number = (raw->msg[idx] >> 4) & 0xfU;
    idx++;
    if (idx <= raw->curlen) {
        repmsg->u.path_resources.full_payload_bw_number = ((uint16_t)raw->msg[idx] << 8)
                                                          | (raw->msg[idx + 1U]);
        idx += 2U;
    }
    if (idx <= raw->curlen) {
        repmsg->u.path_resources.avail_payload_bw_number = ((uint16_t)raw->msg[idx] << 8)
                                                           | (raw->msg[idx + 1U]);
        idx += 2U;
    }
    if (idx > raw->curlen) {
        ret = false;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "enum resource parse length fail %d %d\n", idx,
               raw->curlen);
    }
    return ret;
}

static bool drm_dp_sideband_parse_allocate_payload_ack(
    const SBM_drm_dp_sideband_msg_rx *  raw,
    SBM_drm_dp_sideband_msg_reply_body *repmsg) {

    bool ret = true;
    uint8_t idx = 1;
    repmsg->u.allocate_payload.port_number = (raw->msg[idx] >> 4) & 0xfU;
    idx++;
    if (idx <= raw->curlen) {
        repmsg->u.allocate_payload.vcpi = raw->msg[idx];
        idx++;
    }
    if (idx <= raw->curlen) {
        repmsg->u.allocate_payload.allocated_pbn = ((uint16_t)raw->msg[idx] << 8)
                                                   | ((uint16_t)raw->msg[idx + 1U]);
        idx += 2U;
    }
    if (idx > raw->curlen) {
        ret = false;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "allocate payload parse length fail %d %d\n", idx,
               raw->curlen);
    }
    return ret;
}

static bool drm_dp_sb_parse_reply_prepare(SBM_drm_dp_sideband_msg_rx *        raw,
                                          SBM_drm_dp_sideband_msg_reply_body *msg) {
    bool ret = true;

    (void)memset(msg, 0, sizeof(*msg));

    msg->reply_type = (raw->msg[0] & 0x80U) >> 7;
    msg->req_type = (raw->msg[0] & 0x7fU);

    if (msg->reply_type > 0U)  {
        (void)memcpy(msg->u.nak.guid, &raw->msg[1], 16);
        msg->u.nak.reason = raw->msg[17];
        msg->u.nak.nak_data = raw->msg[18];
        ret = false;
    }
    return ret;
}

static bool drm_dp_sideband_parse_reply(SBM_drm_dp_sideband_msg_rx *        raw,
                                        SBM_drm_dp_sideband_msg_reply_body *msg) {
    bool ret = true;

    ret = drm_dp_sb_parse_reply_prepare(raw, msg);

    if (ret) {
        switch (msg->req_type) {
        case DP_LINK_ADDRESS:
            ret = drm_dp_sideband_parse_link_address(raw, msg);
            break;
        case DP_REMOTE_DPCD_READ:
            ret = sideband_parse_remote_dpcd_read(raw, msg);
            break;
        case DP_REMOTE_DPCD_WRITE:
            ret = sideband_parse_remote_dpcd_write(raw, msg);
            break;
        case DP_REMOTE_I2C_READ:
            ret = drm_dp_sideband_parse_remote_i2c_read_ack(raw, msg);
            break;
        case DP_ENUM_PATH_RESOURCES:
            ret = drm_dp_sideband_parse_enum_path_resources_ack(raw, msg);
            break;
        case DP_ALLOCATE_PAYLOAD:
            ret = drm_dp_sideband_parse_allocate_payload_ack(raw, msg);
            break;
        default:
            ret = false;
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got unknown reply 0x%02x\n", msg->req_type);
            break;
        }
    }
    return ret;
}

static bool drm_dp_sideband_parse_connection_status_notify(
    SBM_drm_dp_sideband_msg_rx *      raw,
    SBM_drm_dp_sideband_msg_req_body *msg) {

    bool ret = false;
    uint8_t idx = 1;

    msg->u.conn_stat.port_number = (raw->msg[idx] & 0xf0U) >> 4;
    idx++;
    if (idx <= raw->curlen) {
        (void)memcpy(msg->u.conn_stat.guid, &raw->msg[idx], 16);
        idx += 16U;
    }
    if (idx <= raw->curlen) {
        msg->u.conn_stat.legacy_device_plug_status = ((raw->msg[idx] >> 6) & 0x1U) != 0U;
        msg->u.conn_stat.displayport_device_plug_status = ((raw->msg[idx] >> 5)
                                                           & 0x1U) != 0U;
        msg->u.conn_stat.message_capability_status = ((raw->msg[idx] >> 4) & 0x1U) != 0U;
        msg->u.conn_stat.input_port = ((raw->msg[idx] >> 3) & 0x1U) != 0U;
        msg->u.conn_stat.peer_device_type = (raw->msg[idx] & 0x7U);
        idx++;
        ret = true;
    }

    if (!ret) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "connection status reply parse length fail %d %d\n", idx,
               raw->curlen);
    }
    return ret;
}

static bool drm_dp_sideband_parse_resource_status_notify(
    SBM_drm_dp_sideband_msg_rx *      raw,
    SBM_drm_dp_sideband_msg_req_body *msg) {
    bool ret = false;
    uint8_t idx = 1;

    msg->u.resource_stat.port_number = (raw->msg[idx] & 0xf0U) >> 4;
    idx++;
    if (idx <= raw->curlen) {
        (void)memcpy(msg->u.resource_stat.guid, &raw->msg[idx], 16);
        idx += 16U;
    }
    if (idx <= raw->curlen) {
        msg->u.resource_stat.available_pbn = ((uint16_t)raw->msg[idx] << 8)
                                             | (raw->msg[idx + 1U]);
        idx++;
        ret = true;
    }

    if (!ret) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "resource status reply parse length fail %d %d\n", idx,
               raw->curlen);
    }
    return ret;
}

static bool drm_dp_sideband_parse_req(SBM_drm_dp_sideband_msg_rx *      raw,
                                      SBM_drm_dp_sideband_msg_req_body *msg) {
    bool ret;
    (void)memset(msg, 0, sizeof(*msg));
    msg->req_type = (raw->msg[0] & 0x7fU);

    switch (msg->req_type) {
    case DP_CONNECTION_STATUS_NOTIFY:
        ret = drm_dp_sideband_parse_connection_status_notify(raw, msg);
        break;
    case DP_RESOURCE_STATUS_NOTIFY:
        ret = drm_dp_sideband_parse_resource_status_notify(raw, msg);
        break;
    default:
        ret = false;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got unknown request 0x%02x\n", msg->req_type);
        break;
    }

    return ret;
}

static void build_dpcd_write(SBM_drm_dp_sideband_msg_tx *msg, uint8_t port_num,
                             uint32_t offset, uint8_t num_bytes, uint8_t *bytes) {
    SBM_drm_dp_sideband_msg_req_body req;

    req.req_type = DP_REMOTE_DPCD_WRITE;
    req.u.dpcd_write.port_number = port_num;
    req.u.dpcd_write.dpcd_address = offset;
    req.u.dpcd_write.num_bytes = num_bytes;
    req.u.dpcd_write.bytes = bytes;
    drm_dp_encode_sideband_req(&req, msg);
}

static void build_link_address(SBM_drm_dp_sideband_msg_tx *msg) {
    SBM_drm_dp_sideband_msg_req_body req;

    req.req_type = DP_LINK_ADDRESS;
    drm_dp_encode_sideband_req(&req, msg);
}

static void build_enum_path_resources(SBM_drm_dp_sideband_msg_tx *msg,
                                      uint8_t                     port_num) {
    SBM_drm_dp_sideband_msg_req_body req;

    req.req_type = DP_ENUM_PATH_RESOURCES;
    req.u.port_num.port_number = port_num;
    drm_dp_encode_sideband_req(&req, msg);
    msg->path_msg = true;
}

static void build_allocate_payload(SBM_drm_dp_sideband_msg_tx *msg,
                                   uint8_t port_num, uint8_t vcpi, uint16_t pbn, uint8_t number_sdp_streams) {

    uint8_t i;
    uint8_t sinks[SBM_DRM_DP_MAX_SDP_STREAMS] = {0};
    SBM_drm_dp_sideband_msg_req_body req;

    (void)memset(&req, 0, sizeof(req));
    req.req_type = DP_ALLOCATE_PAYLOAD;
    req.u.allocate_payload.port_number = port_num;
    req.u.allocate_payload.vcpi = vcpi;
    req.u.allocate_payload.pbn = pbn;
    req.u.allocate_payload.number_sdp_streams = number_sdp_streams;

    for (i = 0U; i < number_sdp_streams; i++) {
        sinks[i] = i;
    }
    (void)memcpy(req.u.allocate_payload.sdp_stream_sink, sinks,
                 number_sdp_streams);
    drm_dp_encode_sideband_req(&req, msg);
    msg->path_msg = true;
}

static bool check_txmsg_state(const SBM_drm_dp_sideband_msg_tx *txmsg) {
    uint32_t state;

    /*
     * All updates to txmsg->state are protected by mgr->qlock, and the two
     * cases we check here are terminal states. For those the barriers
     * provided by the wake_up/wait_event pair are enough.
     */
    state = txmsg->state;
    return ((state == DRM_DP_SIDEBAND_TX_RX)
            || (state == DRM_DP_SIDEBAND_TX_TIMEOUT));
}

bool drm_dp_mst_handle_hpd(MST_drm_dp_topology_mgr *mgr) {
    uint8_t esi[4];
    uint32_t ret;
    bool handled = false;

    ret = drm_dp_dpcd_read(mgr->aux, DP_SINK_COUNT_ESI, esi, 4);
    if (ret == CDN_EOK) {
        (void)drm_dp_mst_hpd_irq(mgr, esi, &handled);
        if (handled) {
            uint8_t irq_handled = esi[1] & (uint8_t)((uint8_t)DP_DOWN_REP_MSG_RDY | (uint8_t)DP_UP_REQ_MSG_RDY);
            (void)drm_dp_dpcd_write_byte(mgr->aux, DP_DEV_SERVICE_IRQ_VECTOR_ESI0, irq_handled);
        }
    }

    return handled;
}

/*
 * Wait for sideband Tx reply. Timeout is set to 4 seconds
 * @param[in,out] mstBranch pointer to MST branch object
 * @param[in] txMessage pointer to sideband message object
 * @return CDN_EOK if response was received
 * @return CDN_EIO if timeout
 */
static uint32_t drm_dp_mst_wait_tx_reply(MST_drm_dp_branch *         mstBranch,
                                         SBM_drm_dp_sideband_msg_tx *txMessage)

{
    uint32_t timeout = 4000U;
    MST_drm_dp_topology_mgr *topologyManager = mstBranch->mgr;
    uint32_t retVal = CDN_EOK;
    bool isState = false;
    uint32_t state;

    /* wait for response or break if time expired */
    while (timeout != 0U) {
        CPS_DelayNs(1000000);
        if (!topologyManager->int_enabled) {
            (void)drm_dp_mst_handle_hpd(topologyManager);
        }
        isState = check_txmsg_state(txMessage);
        if (isState) {
            /* break if response was received */
            break;
        }
        timeout--;
    }

    state = txMessage->state;

    if (isState) {
        if (state == DRM_DP_SIDEBAND_TX_TIMEOUT) {
            retVal = CDN_EIO;
        } else {
            retVal = CDN_EPERM;
        }
    } else {

        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "ERROR: Message timeout. txMessage %p state %d seqno %d, mstBranch %p\n",
               txMessage, txMessage->state, txMessage->seqno, mstBranch);

        /* clean slot if Tx was during sending */
        if ((state == DRM_DP_SIDEBAND_TX_START_SEND) || (state == DRM_DP_SIDEBAND_TX_SENT)) {
            mstBranch->tx_slots[txMessage->seqno] = NULL;
        }

        retVal = CDN_EIO;
    }
    return retVal;
}

/*
 * calculate a new RAD for this MST branch device
 * if parent has an LCT of 2 then it has 1 nibble of RAD,
 * if parent has an LCT of 3 then it has 2 nibbles of RAD,
 */
static uint8_t drm_dp_calculate_rad(MST_drm_dp_port *port, uint8_t *rad) {
    uint8_t parent_lct = port->parent->lct;
    uint8_t shift = 4U;
    uint8_t idx = (parent_lct - 1U) / 2U;
    if (parent_lct > 1U) {
        uint8_t len = idx + 1U;
        (void)memcpy(rad, port->parent->rad, len);
        shift = ((parent_lct % 2U) != 0U) ? 4U : 0U;
    } else {
        rad[0] = 0;
    }
    /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 "Shifting operation should be checked, DRV-3828" */
    /* shift is always correct: 0U or 4U */
    rad[idx] |= (port->port_num << shift);
    /* parasoft-end-suppress MISRA2012-RULE-12_2-2 */

    return parent_lct + 1U;
}

/*
 * return sends link address for new mstb
 */
static bool drm_dp_port_setup_pdt(MST_drm_dp_port *port) {
    uint8_t rad[6], lct;
    bool send_link = false;
    MST_drm_dp_topology_mgr* mgr = port->mgr;
    switch (port->pdt) {
    case DP_PEER_DEVICE_DP_LEGACY_CONV:
    case DP_PEER_DEVICE_SST_SINK:
        break;
    case DP_PEER_DEVICE_MST_BRANCHING:
        lct = drm_dp_calculate_rad(port, rad);

        DbgMsg(DBG_GEN_MSG, DBG_FYI, "Add branch device on port %p lct %d rad %02d %02d %02d %02d %02d %02d\n",
               port, lct, rad[0], rad[1], rad[2], rad[3], rad[4], rad[5]);
        port->mstb = DRM_DP_addBranchDevice(mgr, lct, rad);
        if (port->mstb != NULL) {
            port->mstb->mgr = port->mgr;
            port->mstb->port_parent = port;
            send_link = true;
        }
        break;
    default:
        send_link = false;
        break;
    }
    return send_link;
}

static int8_t drm_dp_get_free_port(const MST_drm_dp_branch *mstb,
                                   uint8_t                  port_number) {
    int8_t free_port_number = -1;

    if (mstb->ports[port_number].refcount == 0U) {
        free_port_number = (int8_t)port_number;
    } else {
        int8_t i;
        for (i = 0; i < (int8_t)MST_BRANCH_MAX_PORTS; i++) {
            if (mstb->ports[i].refcount == 0U) {
                free_port_number = i;
            }
        }
    }
    return free_port_number;
}

/*
 * Returns reference to first empty port.
 * @param[in] mstb pointer to MST branch object
 * @param[in] portNumber number of port
 * @return pointer to port if success
 * @return NULL if no empty port available
 */
static MST_drm_dp_port* createNewPort(MST_drm_dp_branch *mstb,
                                      uint8_t            portNumber)
{
    MST_drm_dp_port* port = NULL;
    int8_t freePortNumber = drm_dp_get_free_port(mstb, portNumber);

    if (freePortNumber >= 0) {
        port = &(mstb->ports[freePortNumber]);
        port->refcount = 1U;
        port->parent = mstb;
        port->port_num = portNumber;
        port->mgr = mstb->mgr;
    }

    return port;
}

/*
 * Function fill port parameters
 * @param[in] pointer to MST port object to be filled
 * @param[in] portMsg
 */
static void fillPortParams(MST_drm_dp_port*                       port,
                           const SBM_drm_dp_link_addr_reply_port *portMsg)
{
    port->pdt = portMsg->peer_device_type;
    port->input = portMsg->input_port;
    port->mcs = portMsg->mcs;
    port->ddps = portMsg->ddps;
    port->ldps = portMsg->legacy_device_plug_status;
    port->dpcd_rev = portMsg->dpcd_revision;
    port->num_sdp_streams = portMsg->num_sdp_streams;
    port->num_sdp_stream_sinks = portMsg->num_sdp_stream_sinks;
}

/*
 * Clean available payload bandwidth if DisplayPort Device Plug Status was changed to 'false'
 * or get it from path resources if DisplayPort Device Plug Status was changed to 'true' and
 * port is output
 * @param[in] port pointer to MST port object
 * @param[in] mstBranch pointer to MST branch object
 * @param[in] oldDdps old state of DP Device Plug Status
 */
static void reloadAvailablePayloadBandwidth(MST_drm_dp_port*   port,
                                            MST_drm_dp_branch* mstBranch,
                                            bool               oldDpps)
{
    /* test if DP Device Plug Status was changed */
    if (oldDpps != port->ddps) {
        /* check if device is plug-in */
        if (port->ddps) {
            /* check if port is output */
            if (!port->input) {
                drm_dp_send_enum_path_resources(mstBranch->mgr, mstBranch, port);
            }
        } else {
            port->available_pbn = 0;
        }
    }
}

/*
 * Teardown port and set new peer-device-type
 * @param[in] port pointer to MST port object
 * @param[in] oldPdt old value of peer-device-type
 * @return true if link-address should be sent
 * @return false if not
 */
static bool reloadPeerDeviceType(MST_drm_dp_port* port,
                                 uint8_t          oldPdt)
{
    bool retVal = false;

    if ((oldPdt != port->pdt) && (!port->input)) {
        DRM_DP_teardownPort(port, oldPdt);
        retVal = drm_dp_port_setup_pdt(port);
    }

    return retVal;
}

static void drm_dp_add_port(MST_drm_dp_branch *              mstb,
                            SBM_drm_dp_link_addr_reply_port *port_msg)
{

    bool created = false;
    uint8_t old_pdt = 0U;
    bool old_ddps = false;
    MST_drm_dp_port *port = DRM_DP_getPort(mstb, port_msg->port_number);

    /* check if port exists */
    if (port == NULL) {

        port = createNewPort(mstb, port_msg->port_number);

        if (port != NULL) {
            created = true;
        }

    } else {
        /* save old parameters of port */
        old_pdt = port->pdt;
        old_ddps = port->ddps;
    }

    /* do if port exists or was created correctly */
    if (port != NULL) {
        fillPortParams(port, port_msg);

        if (created) {
            port->refcount++;
        }

        reloadAvailablePayloadBandwidth(port, mstb, old_ddps);

        (void)reloadPeerDeviceType(port, old_pdt);

        /* put reference to this port */
        DRM_DP_putPort(port);
    }
}

static void drm_dp_update_port(MST_drm_dp_branch *                  mstb,
                               SBM_drm_dp_connection_status_notify *conn_stat) {

    MST_drm_dp_port *port;
    uint8_t old_pdt;
    bool old_ddps;
    bool dowork = false;
    bool linkAddress;
    port = DRM_DP_getPort(mstb, conn_stat->port_number);

    if (port != NULL) {
        old_ddps = port->ddps;
        old_pdt = port->pdt;
        port->pdt = conn_stat->peer_device_type;
        port->mcs = conn_stat->message_capability_status;
        port->ldps = conn_stat->legacy_device_plug_status;
        port->ddps = conn_stat->displayport_device_plug_status;

        if (old_ddps != port->ddps) {
            if (port->ddps) {
                dowork = true;
            } else {
                port->available_pbn = 0;
            }
        }

        linkAddress = reloadPeerDeviceType(port, old_pdt);
        if (linkAddress) {
            dowork = true;
        }

        DRM_DP_putPort(port);
        if (dowork) {
            (void)drm_dp_mst_link_probe_work(mstb->mgr);
        }
    }
}

static MST_drm_dp_branch * drm_dp_configure_port(MST_drm_dp_branch *mstb, MST_drm_dp_port *port) {

    MST_drm_dp_branch *mstb_child = NULL;

    if (port->refcount > 0U) {
        if ((!port->input) && port->ddps) {
            if (port->available_pbn <= 0) {
                drm_dp_send_enum_path_resources(mstb->mgr, mstb, port);
            }

            if (port->mstb != NULL) {
                mstb_child = DRM_DP_getValidatedBranchRef(mstb->mgr, port->mstb);
            }
        }
    }
    return mstb_child;
}

static int8_t drm_dp_mst_build_next(const MST_drm_dp_branch *mstb_array[]) {
    int8_t i;
    int8_t ret = -1;
    for (i = 0; (uint8_t)i < MST_BRANCH_MAX_PORTS; ++i) {
        if (mstb_array[i] != NULL) {
            ret = i;
            break;
        }
    }
    return ret;
}

uint32_t drm_dp_mst_link_probe_work(MST_drm_dp_topology_mgr *mgr)
{
    uint32_t i;
    uint32_t ret = CDN_EOK;
    MST_drm_dp_branch *mstb_array[MST_BRANCH_MAX_PORTS] = {NULL};
    mstb_array[0] = mgr->mst_primary;

    while (drm_dp_mst_build_next((const MST_drm_dp_branch**)mstb_array) >= 0) {
        int8_t current_index = drm_dp_mst_build_next((const MST_drm_dp_branch**)mstb_array);
        MST_drm_dp_branch *mstb = mstb_array[current_index];
        if (!mstb->link_address_sent) {
            drm_dp_send_link_address(mgr, mstb);
            mstb_array[current_index] = NULL;
        }
        for (i = 0; i < MST_BRANCH_MAX_PORTS; i++) {
            MST_drm_dp_branch *mstb_child = NULL;
            mstb_child = drm_dp_configure_port(mstb, &mstb->ports[i]);
            if (mstb_child != NULL) {
                current_index++;
                mstb_array[current_index] = mstb_child;
            }
        }
    }

    return ret;
}

static void process_single_down_tx_qlock(MST_drm_dp_topology_mgr *mgr) {

    if (mgr->tx_msg_downq != NULL) {
        SBM_drm_dp_sideband_msg_tx *txmsg;
        uint32_t ret;

        txmsg = mgr->tx_msg_downq;
        ret = process_single_tx_qlock(mgr, txmsg, false);
        if (ret == 1U) {
            /* txmsg is sent it should be in the slots now */
            mgr->tx_msg_downq = NULL;
        } else if (ret > 0U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to send msg in q %d\n", ret);
            mgr->tx_msg_downq = NULL;
            if (txmsg->seqno != -1) {
                txmsg->dst->tx_slots[txmsg->seqno] = NULL;
            }
            txmsg->state = DRM_DP_SIDEBAND_TX_TIMEOUT;
        } else {
            /* TODO: add else action */
        }
    }
}

/* called holding qlock */
static void process_single_up_tx_qlock(MST_drm_dp_topology_mgr *   mgr,
                                       SBM_drm_dp_sideband_msg_tx *txmsg) {
    uint32_t ret;

    /* construct a chunk from the first msg in the tx_msg queue */
    ret = process_single_tx_qlock(mgr, txmsg, true);

    if (ret != 1U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "failed to send msg in q %d\n", ret);
    }

    txmsg->dst->tx_slots[txmsg->seqno] = NULL;
}

static void drm_dp_queue_down_tx(MST_drm_dp_topology_mgr *   mgr,
                                 SBM_drm_dp_sideband_msg_tx *txmsg) {

    if (mgr->tx_msg_downq != NULL) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Not enough space to add sideband message\n");
    } else {
        mgr->tx_msg_downq = txmsg;
        process_single_down_tx_qlock(mgr);
    }
}

/*
 * Send link address to branch device
 * param[in] mgr pointer to MST topology manager object
 * param[in] mstb pointer to MST branch device
 */
static void drm_dp_send_link_address(MST_drm_dp_topology_mgr *topologyManager,
                                     MST_drm_dp_branch *      mstb)
{
    SBM_drm_dp_link_address_ack_reply* linkAddress;
    SBM_drm_dp_sideband_msg_tx txmsg;
    SBM_drm_dp_link_addr_reply_port* port;
    uint32_t ret;

    /* send link address to branch device */
    (void)memset(&txmsg, 0, sizeof(txmsg));
    txmsg.dst = mstb;
    build_link_address(&txmsg);
    mstb->link_address_sent = true;
    drm_dp_queue_down_tx(topologyManager, &txmsg);

    ret = drm_dp_mst_wait_tx_reply(mstb, &txmsg);

    if (ret != CDN_EOK) {
        uint32_t i;

        /* check reply for message */
        if (txmsg.reply.reply_type == 1U) {
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "INFO: Link address nak received\n");
        }
        else {

            linkAddress = &(txmsg.reply.u.link_addr);
            DbgMsg(DBG_GEN_MSG, DBG_FYI, "INFO :Link address reply: %d\n", linkAddress->nports);

            /* print configuration of ports from link address */
            for (i = 0U; i < linkAddress->nports; i++) {
                port = &(linkAddress->ports[i]);
                DbgMsg(DBG_GEN_MSG, DBG_FYI,
                       "port %d: input %d, pdt: %d, pn: %d, dpcd_rev: %02x, mcs: %d, ddps: %d, ldps %d, sdp %d/%d\n",
                       i,
                       port->input_port,
                       port->peer_device_type,
                       port->port_number,
                       port->dpcd_revision,
                       port->mcs,
                       port->ddps,
                       port->legacy_device_plug_status,
                       port->num_sdp_streams,
                       port->num_sdp_stream_sinks);
            }

            drm_dp_check_mstb_guid(mstb, linkAddress->guid);

            for (i = 0U; i < linkAddress->nports; i++) {
                port = &(linkAddress->ports[i]);
                drm_dp_add_port(mstb, port);
            }
        }
    } else {
        mstb->link_address_sent = false;
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "ERROR: Link address failed %d\n", ret);
    }
}

static void drm_dp_send_enum_path_resources(MST_drm_dp_topology_mgr *mgr,
                                            MST_drm_dp_branch *mstb, MST_drm_dp_port *port) {

    SBM_drm_dp_sideband_msg_tx txmsg;
    uint32_t ret;

    (void)memset(&txmsg, 0, sizeof(txmsg));
    txmsg.dst = mstb;
    build_enum_path_resources(&txmsg, port->port_num);

    drm_dp_queue_down_tx(mgr, &txmsg);

    ret = drm_dp_mst_wait_tx_reply(mstb, &txmsg);
    if (ret > 0U) {
        if (txmsg.reply.reply_type == 1U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "enum path resources nak received\n");
        }
        else {
            if (port->port_num != txmsg.reply.u.path_resources.port_number) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "got incorrect port in response\n");
            }
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "enum path resources %d: %d %d\n",
                   txmsg.reply.u.path_resources.port_number,
                   txmsg.reply.u.path_resources.full_payload_bw_number,
                   txmsg.reply.u.path_resources.avail_payload_bw_number);
            port->available_pbn = (int16_t)
                                  txmsg.reply.u.path_resources.avail_payload_bw_number;
        }
    }
}

static MST_drm_dp_port *get_last_connected_port_to_mstb(MST_drm_dp_branch *mstb) {

    MST_drm_dp_branch *current_mstb = mstb;
    MST_drm_dp_port *port = current_mstb->port_parent;

    while (current_mstb->port_parent != NULL) {
        if (current_mstb->port_parent->mstb != current_mstb) {
            port = current_mstb->port_parent;
            break;
        } else {
            current_mstb = current_mstb->port_parent->parent;
        }
    }
    return port;
}

static MST_drm_dp_branch *get_last_connected_port_and_mstb(
    const MST_drm_dp_topology_mgr *mgr, MST_drm_dp_branch *mstb,
    uint8_t *port_num) {
    MST_drm_dp_branch *rmstb = NULL;
    MST_drm_dp_port *found_port;

    if (mgr->mst_primary != NULL) {
        found_port = get_last_connected_port_to_mstb(mstb);

        if (found_port != NULL) {
            rmstb = found_port->parent;
            rmstb->refcount++;
            *port_num = found_port->port_num;
        }
    }
    return rmstb;
}

/*
 * Secondary structure for sending payload messages
 */
typedef struct {
    uint8_t id;
    uint16_t payloadBandwidth;
    uint8_t sdpNumber;
    uint8_t portNumber;
} DP_PayloadMsg;

/*
 * Send payload message to topology manager
 * @param[in] topologyManager pointer to MST topology manager object
 * @param[in] branch pointer to MST branch object
 * @param[in] message pointer to DP_PayloadMsg object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if reply type is '1'
 */
static uint32_t sendPayloadMsg(MST_drm_dp_topology_mgr *topologyManager,
                               MST_drm_dp_branch *      branch,
                               const DP_PayloadMsg*     message)
{
    SBM_drm_dp_sideband_msg_tx txmsg;
    uint32_t retVal;

    (void)memset(&txmsg, 0, sizeof(txmsg));

    txmsg.dst = branch;

    /* allocate payload */
    build_allocate_payload(&txmsg,
                           message->portNumber,
                           message->id,
                           message->payloadBandwidth,
                           message->sdpNumber);

    /* put message into Tx queue */
    drm_dp_queue_down_tx(topologyManager, &txmsg);

    /* wait for reply */
    retVal = drm_dp_mst_wait_tx_reply(branch, &txmsg);

    if (retVal != CDN_EOK) {

        if (txmsg.reply.reply_type == 1U) {
            retVal = CDN_EINVAL;
        } else {
            retVal = CDN_EOK;
        }
    }

    return retVal;
}

/*
 * Look for correct branch and send payload message
 * @param[in] topologyManager pointer to MST topologyManager object
 * @param[in] port pointer to MST port object
 * @param[in] id identifier of message
 * @param[in] pbn value of payload bandwidth
 * @return CDN_EOK if success
 * @return CDN_EINVAL if cannot find branch or reply is incorrect
 */
uint32_t drm_dp_payload_send_msg(MST_drm_dp_topology_mgr *mgr,
                                 MST_drm_dp_port *        port,
                                 int32_t                  id,
                                 int32_t                  pbn)
{
    DP_PayloadMsg message;
    MST_drm_dp_branch *mstb;
    uint32_t retVal = CDN_EOK;
    uint8_t port_num;
    MST_drm_dp_port *dst_port;

    /* get reference to port */
    dst_port = DRM_DP_getValidatedPortRef(mgr, port);

    if (dst_port == NULL) {
        retVal = CDN_EINVAL;
    } else {
        /* find branch connected with port */
        port_num = dst_port->port_num;
        mstb = DRM_DP_getValidatedBranchRef(mgr, dst_port->parent);

        if (mstb == NULL) {
            mstb = get_last_connected_port_and_mstb(mgr, dst_port->parent, &port_num);
        }

        if (mstb == NULL) {
            DRM_DP_putPort(dst_port);
            retVal = CDN_EINVAL;
        }
    }

    if (retVal == CDN_EOK) {

        /* fill payload message */
        message.id = (uint8_t)id;
        message.payloadBandwidth = (uint16_t)pbn;
        message.portNumber = port_num;
        message.sdpNumber = dst_port->num_sdp_streams;

        retVal = sendPayloadMsg(mgr, mstb, &message);

        /* put device */
        DRM_DP_putBranchDevice(mstb);
        DRM_DP_putPort(dst_port);
    }

    return retVal;
}

static uint32_t drm_dp_send_dpcd_write(MST_drm_dp_topology_mgr *mgr,
                                       MST_drm_dp_port *port, uint32_t offset, uint8_t size, uint8_t *bytes) {

    uint32_t ret = CDN_EINVAL;
    SBM_drm_dp_sideband_msg_tx txmsg;
    MST_drm_dp_branch *mstb;

    (void)memset(&txmsg, 0, sizeof(txmsg));
    mstb = DRM_DP_getValidatedBranchRef(mgr, port->parent);
    if (mstb != NULL) {
        build_dpcd_write(&txmsg, port->port_num, offset, size, bytes);
        txmsg.dst = mstb;

        drm_dp_queue_down_tx(mgr, &txmsg);

        ret = drm_dp_mst_wait_tx_reply(mstb, &txmsg);
        if (ret > 0U) {
            if (txmsg.reply.reply_type == 1U) {
                ret = CDN_EINVAL;
            } else {
                ret = CDN_EOK;
            }
        }
        DRM_DP_putBranchDevice(mstb);
    }
    return ret;
}

/**************** RX **********************************************************/

static void drm_dp_encode_up_ack_reply(SBM_drm_dp_sideband_msg_tx *msg,
                                       uint8_t                     req_type) {
    SBM_drm_dp_sideband_msg_reply_body reply;

    reply.reply_type = 0;
    reply.req_type = req_type;
    drm_dp_encode_sideband_reply(&reply, msg);
}

static void drm_dp_send_up_ack_reply(MST_drm_dp_topology_mgr *mgr,
                                     MST_drm_dp_branch *mstb, uint8_t req_type, int32_t seqno) {
    SBM_drm_dp_sideband_msg_tx txmsg;

    (void)memset(&txmsg, 0, sizeof(txmsg));
    txmsg.dst = mstb;
    txmsg.seqno = seqno;
    drm_dp_encode_up_ack_reply(&txmsg, req_type);
    process_single_up_tx_qlock(mgr, &txmsg);
}

void drm_dp_mst_handle_down_rep(MST_drm_dp_topology_mgr *mgr) {
    SBM_drm_dp_sideband_msg_tx *txmsg;
    MST_drm_dp_branch *mstb;
    bool ret = drm_dp_get_one_sb_msg(mgr, false);
    if (ret && mgr->down_rep_recv.have_eomt) {
        SBM_drm_dp_sideband_msg_hdr *initial_hdr = &mgr->down_rep_recv.initial_hdr;
        mstb = DRM_DP_getBranchDevice(mgr, initial_hdr->lct, initial_hdr->rad);
        if (mstb == NULL) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got MST reply from unknown device %d\n",
                   initial_hdr->lct);
        } else {
            /* find the message */
            int32_t slot = initial_hdr->seqno ? 1 : 0;
            txmsg = mstb->tx_slots[slot];
            /* remove from slots */
            if (txmsg == NULL) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got MST reply with no msg %p segno %d lct %d rad[0] %02x msg[0] %02x\n",
                       mstb,
                       initial_hdr->seqno,
                       initial_hdr->lct,
                       initial_hdr->rad[0],
                       mgr->down_rep_recv.msg[0]);
                DRM_DP_putBranchDevice(mstb);
            } else {
                (void)drm_dp_sideband_parse_reply(&mgr->down_rep_recv, &txmsg->reply);
                if (txmsg->reply.reply_type == 1U) {
                    DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                           "Got NAK reply: req 0x%02x, reason 0x%02x, nak data 0x%02x\n",
                           txmsg->reply.req_type, txmsg->reply.u.nak.reason,
                           txmsg->reply.u.nak.nak_data);
                }
                DRM_DP_putBranchDevice(mstb);
                txmsg->state = DRM_DP_SIDEBAND_TX_RX;
                mstb->tx_slots[slot] = NULL;
            }
        }
        (void)memset(&mgr->down_rep_recv, 0, sizeof(SBM_drm_dp_sideband_msg_rx));
    }
}

static void handle_connection_status_notify(MST_drm_dp_topology_mgr *         mgr,
                                            MST_drm_dp_branch *               mstb,
                                            SBM_drm_dp_sideband_msg_req_body *msg) {

    int32_t seqno = mgr->up_req_recv.initial_hdr.seqno ? 1 : 0;
    MST_drm_dp_branch *current_mstb = mstb;
    drm_dp_send_up_ack_reply(mgr, mgr->mst_primary, msg->req_type, seqno);

    if (current_mstb == NULL) {
        current_mstb = DRM_DP_getBranchDeviceByGuid(mgr, msg->u.conn_stat.guid);
    }
    if (current_mstb == NULL) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got MST reply from unknown device %d\n",
               mgr->up_req_recv.initial_hdr.lct);
        (void)memset(&mgr->up_req_recv, 0,
                     sizeof(SBM_drm_dp_sideband_msg_rx));
    } else {
        drm_dp_update_port(current_mstb, &msg->u.conn_stat);
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got CSN: pn: %d ldps:%d ddps: %d mcs: %d ip: %d pdt: %d\n",
               msg->u.conn_stat.port_number,
               msg->u.conn_stat.legacy_device_plug_status,
               msg->u.conn_stat.displayport_device_plug_status,
               msg->u.conn_stat.message_capability_status,
               msg->u.conn_stat.input_port,
               msg->u.conn_stat.peer_device_type);
    }
}

static void handle_resource_status_notify(MST_drm_dp_topology_mgr *         mgr,
                                          MST_drm_dp_branch *               mstb,
                                          SBM_drm_dp_sideband_msg_req_body *msg) {

    int32_t seqno = mgr->up_req_recv.initial_hdr.seqno ? 1 : 0;
    MST_drm_dp_branch *current_mstb = mstb;

    drm_dp_send_up_ack_reply(mgr, mgr->mst_primary, msg->req_type, seqno);
    if (current_mstb == NULL) {
        current_mstb = DRM_DP_getBranchDeviceByGuid(mgr, msg->u.resource_stat.guid);
    }
    if (current_mstb == NULL) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got MST reply from unknown device %d\n",
               mgr->up_req_recv.initial_hdr.lct);
        (void)memset(&mgr->up_req_recv, 0,
                     sizeof(SBM_drm_dp_sideband_msg_rx));
    } else {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got RSN: pn: %d avail_pbn %d\n",
               msg->u.resource_stat.port_number,
               msg->u.resource_stat.available_pbn);
    }
}

void drm_dp_mst_handle_up_req(MST_drm_dp_topology_mgr *mgr) {
    bool cont = true;

    if (!drm_dp_get_one_sb_msg(mgr, true)) {
        (void)memset(&mgr->up_req_recv, 0, sizeof(SBM_drm_dp_sideband_msg_rx));
        cont = false;
    }

    if (cont && mgr->up_req_recv.have_eomt) {
        SBM_drm_dp_sideband_msg_req_body msg;
        MST_drm_dp_branch *mstb = NULL;
        SBM_drm_dp_sideband_msg_rx *up_req_recv = &mgr->up_req_recv;

        if (!up_req_recv->initial_hdr.broadcast) {
            SBM_drm_dp_sideband_msg_hdr *initial_hdr = &up_req_recv->initial_hdr;
            mstb = DRM_DP_getBranchDevice(mgr, initial_hdr->lct, initial_hdr->rad);
            if (mstb == NULL) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Got MST reply from unknown device %d\n",
                       initial_hdr->lct);
                (void)memset(up_req_recv, 0, sizeof(SBM_drm_dp_sideband_msg_rx));
                cont = false;
            }
        }
        if (cont) {
            (void)drm_dp_sideband_parse_req(up_req_recv, &msg);

            if (msg.req_type == DP_CONNECTION_STATUS_NOTIFY) {
                handle_connection_status_notify(mgr, mstb, &msg);
            } else if (msg.req_type == DP_RESOURCE_STATUS_NOTIFY) {
                handle_resource_status_notify(mgr, mstb, &msg);
            } else {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Other req type: %d\n", msg.req_type);
            }

            DRM_DP_putBranchDevice(mstb);
            (void)memset(up_req_recv, 0, sizeof(SBM_drm_dp_sideband_msg_rx));
        }
    }
}

uint32_t drm_dp_mst_i2c_xfer(MST_drm_dp_port *port,
                             struct mst_i2c_message *msgs, uint32_t num)
{
    MST_drm_dp_branch *mstb;
    MST_drm_dp_topology_mgr *mgr = port->mgr;
    SBM_drm_dp_sideband_msg_req_body msg;
    SBM_drm_dp_sideband_msg_tx txmsg;
    uint32_t ret = CDN_EOK;
    uint32_t i;

    mstb = DRM_DP_getValidatedBranchRef(mgr, port->parent);
    if (mstb == NULL) {
        ret = CDN_EIO;
    } else if ((num - 1U) > SBM_DP_REMOTE_I2C_READ_MAX_TRANSACTIONS) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Unsupported I2C transaction for MST device\n");
        ret = CDN_EIO;
    } else {
        SBM_drm_dp_remote_i2c_read *msg_i2c_read = &msg.u.i2c_read;
        (void)memset(&msg, 0, sizeof(msg));
        msg.req_type = DP_REMOTE_I2C_READ;
        msg_i2c_read->num_transactions = (uint8_t)num - 1U;
        msg_i2c_read->port_number = port->port_num;
        for (i = 0; i < (num - 1U); i++) {
            msg_i2c_read->transactions[i].i2c_dev_id = msgs[i].address;
            msg_i2c_read->transactions[i].num_bytes = msgs[i].length;
            msg_i2c_read->transactions[i].bytes = msgs[i].data;
        }
        msg_i2c_read->read_i2c_device_id = msgs[num - 1U].address;
        msg_i2c_read->num_bytes_read = msgs[num - 1U].length;

        (void)memset(&txmsg, 0, sizeof(txmsg));
        txmsg.dst = mstb;
        drm_dp_encode_sideband_req(&msg, &txmsg);
        drm_dp_queue_down_tx(mgr, &txmsg);
        ret = drm_dp_mst_wait_tx_reply(mstb, &txmsg);
        if (ret > 0U) {
            SBM_drm_dp_remote_i2c_read_ack_reply *remote_i2c_reply = \
                &txmsg.reply.u.remote_i2c_read_ack;
            if ((txmsg.reply.reply_type == 1U) \
                || (remote_i2c_reply->num_bytes != msgs[num - 1U].length)) {
                ret = CDN_EIO;
            } else {
                (void)memcpy(msgs[num - 1U].data, remote_i2c_reply->bytes, msgs[num - 1U].length);
                ret = num;
            }
        }
    }
    DRM_DP_putBranchDevice(mstb);
    return ret;
}
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits" */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-36-3 */
