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
 * dp_sideband_msg.c
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

#include <cdn_stdtypes.h>
#include <cdn_errno.h>
#include <cdn_log.h>

#include "dp_aux.h"
#include "dp_aux_if.h"
#include "dp_sideband_msg.h"
#include "dp_sideband_msg_if.h"
#include "dp_sideband_msg_structs_if.h"

/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */

static uint8_t drm_dp_msg_header_crc4(const uint8_t *data, uint32_t num_nibbles)
{
    uint8_t bitmask = 0x80;
    uint8_t bitshift = 7;
    uint8_t array_index = 0;
    uint32_t number_of_bits = num_nibbles * 4U;
    uint8_t remainder_value = 0;

    while (number_of_bits != 0U) {
        number_of_bits--;
        remainder_value <<= 1;
        if (bitshift < 8U) {
            remainder_value |= (data[array_index] & bitmask) >> bitshift;
        }
        bitmask >>= 1;
        bitshift--;
        if (bitmask == 0U) {
            bitmask = 0x80;
            bitshift = 7;
            array_index++;
        }
        if ((remainder_value & 0x10U) == 0x10U) {
            remainder_value ^= 0x13U;
        }
    }

    number_of_bits = 4;
    while (number_of_bits != 0U) {
        number_of_bits--;
        remainder_value <<= 1;
        if ((remainder_value & 0x10U) != 0U) {
            remainder_value ^= 0x13U;
        }
    }

    return remainder_value;
}

/** Return minimum of 2 - element set (uint32_t). */
static uint32_t min(uint32_t a, uint32_t b)
{
    uint32_t result = a;
    if (b < a) {
        result = b;
    }
    return result;
}

/** Return minimum of 3 - element set (uint32_t). */
static uint32_t min3(uint32_t x, uint32_t y, uint32_t z)
{
    return min(min(x, y), z);
}

/** Send sideband msg. Returns CDN_EOK on success.*/
static uint32_t drm_dp_send_sideband_msg(struct MST_drm_dp_topology_mgr_s *mgr,
                                         bool up, uint8_t *msg, uint32_t len)
{
    uint32_t ret = CDN_EOK;
    uint32_t regbase = up ? DP_SIDEBAND_MSG_UP_REP_BASE : DP_SIDEBAND_MSG_DOWN_REQ_BASE;
    uint32_t tosend, total, offset;
    uint32_t retries = 0;

    do {
        total = len;
        offset = 0;
        do {
            tosend = min3(mgr->max_dpcd_transaction_bytes, 16U, total);
            ret = drm_dp_dpcd_write(mgr->aux, regbase + offset,
                                    &msg[offset], tosend);
            if (ret != CDN_EOK) { /* if writing DPCD register failed */
                retries++;
                break;
            }
            offset += tosend;
            total -= tosend;
        } while (total > 0U);
    } while ((total > 0U) && (retries < 5U));  /* repeat max 5 times */

    if ((ret != CDN_EOK) || (total > 0U)) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "failed to dpcd write %d %d\n", tosend, ret);
    }
    return ret;
}

/** Calculate a 4-bit crc for a given uint8_t array. */
static uint8_t drm_dp_msg_data_crc4(const uint8_t *data, uint8_t number_of_bytes)
{
    uint8_t bitmask = 0x80;
    uint8_t bitshift = 7;
    uint8_t array_index = 0;
    /* data is treated as one continuous value to checksum */
    uint32_t number_of_bits = (uint32_t)number_of_bytes * 8U; /* number of bits in value to checksum */
    uint16_t remainder_value = 0;

    while (number_of_bits != 0U) {
        number_of_bits--;
        remainder_value <<= 1;
        if (bitshift < 8U) {
            uint8_t new_remainder_value = ((data[array_index]) & bitmask) >> bitshift;
            remainder_value |= (uint16_t) new_remainder_value;
        }
        bitmask >>= 1;
        bitshift--;
        if (bitmask == 0U) {
            bitmask = 0x80;
            bitshift = 7;
            array_index++;
        }
        if ((remainder_value & 0x100U) == 0x100U) {
            remainder_value ^= 0xd5U;
        }
    }
    /* last byte */
    number_of_bits = 8;
    while (number_of_bits != 0U) {
        number_of_bits--;
        remainder_value <<= 1;
        if ((remainder_value & 0x100U) != 0U) {
            remainder_value ^= 0xd5U;
        }
    }

    return (uint8_t)(remainder_value & 0xffU);
}

static void drm_dp_crc_sideband_chunk_req(uint8_t *msg, uint8_t len)
{
    uint8_t crc4;
    /* calculate the 4-bit crc and save it at the end of msg */
    crc4 = drm_dp_msg_data_crc4(msg, len);
    msg[len] = crc4;
}

/** Decode sideband message header. */
static void drm_dp_encode_sideband_msg_hdr(const SBM_drm_dp_sideband_msg_hdr *hdr,
                                           uint8_t *buf, uint32_t *len)
{
    uint32_t idx = 0;
    uint8_t i;
    uint8_t crc4;
    /* buf table is filled with data from SBM_drm_dp_sideband_msg_hdr hdr structure */
    buf[idx] = ((hdr->lct & 0xfU) << 4) | (hdr->lcr & 0xfU);
    ++idx;
    for (i = 0; i < (hdr->lct / 2U); i++) {
        buf[idx] = hdr->rad[i];
        ++idx;
    }
    buf[idx] = ((hdr->broadcast ? 1U : 0U) << 7) | ((hdr->path_msg ? 1U : 0U) << 6) |
               (hdr->msg_len & 0x3fU);
    ++idx;
    buf[idx] = ((hdr->somt ? 1U : 0U) << 7) | ((hdr->eomt ? 1U : 0U) << 6) | ((hdr->seqno ? 1U : 0U) << 4);
    ++idx;
    /* calculate 4-bit crc... */
    crc4 = drm_dp_msg_header_crc4(buf, (idx * 2U) - 1U);
    buf[idx - 1U] |= (crc4 & 0xfU); /*  and save it at the end of buf table. */

    *len = idx;
}

/** Calculate DRM DP Sideband header size */
static inline uint8_t drm_dp_calc_sb_hdr_size(const SBM_drm_dp_sideband_msg_hdr *hdr)
{
    uint8_t size = 3;
    size += (hdr->lct / 2U);
    return size;
}

static uint32_t set_hdr_from_dst_qlock(SBM_drm_dp_sideband_msg_hdr *hdr,
                                       SBM_drm_dp_sideband_msg_tx * txmsg)
{
    uint32_t ret = CDN_EOK;

    /* both msg slots are full */
    MST_drm_dp_branch *mstb = txmsg->dst;
    if (txmsg->seqno == -1) {
        if (mstb->tx_slots[0] != NULL) {
            if (mstb->tx_slots[1] != NULL) {
                /* ok, ok */
                DbgMsg(DBG_GEN_MSG, DBG_CRIT, "%s: failed to find slot\n", __func__);
                ret = CDN_EAGAIN;
            } else {
                /* ok, null */
                txmsg->seqno = 1;
                mstb->tx_slots[txmsg->seqno] = txmsg;
            }
        } else {
            if (mstb->tx_slots[1] == NULL) {
                /* null, null */
                txmsg->seqno = mstb->last_seqno;
                uint32_t new_last_segno = (uint32_t) mstb->last_seqno ^ 1U; /* need to be unsigned */
                mstb->last_seqno = (int32_t) new_last_segno;
                mstb->tx_slots[txmsg->seqno] = txmsg;
            } else {
                /* null, ok */
                txmsg->seqno = 0;
                mstb->tx_slots[txmsg->seqno] = txmsg;
            }
        }
    }

    if (ret == CDN_EOK) {
        switch (txmsg->msg[0] & 0x7fU) {  /* reg type */
        case DP_CONNECTION_STATUS_NOTIFY:
        case DP_RESOURCE_STATUS_NOTIFY:
            hdr->broadcast = 1;
            break;
        default:
            hdr->broadcast = 0;
            break;
        }

        hdr->path_msg = txmsg->path_msg;
        hdr->lct = mstb->lct;
        hdr->lcr = mstb->lct - 1U;
        if (mstb->lct > 1U) {
            (void) memcpy(hdr->rad, mstb->rad, (uint8_t)(mstb->lct / 2U));
        }
        hdr->seqno = (txmsg->seqno != 0) ? (bool)true : (bool)false;
    }
    return ret;
}

/** make hdr from dst mst */
static uint32_t single_tx_data_prepare(SBM_drm_dp_sideband_msg_hdr *hdr,
                                       SBM_drm_dp_sideband_msg_tx * txmsg,
                                       uint8_t *                    chunk,
                                       uint32_t *                   idx,
                                       uint32_t *                   tosend)
{
    uint32_t ret;
    uint32_t len, space;
    /* make hdr from dst mst - for replies use seqno
       otherwise assign one */
    ret = set_hdr_from_dst_qlock(hdr, txmsg);
    if (ret == CDN_EOK) {
        /* amount left to send in this message */
        len = (uint32_t) txmsg->cur_len - (uint32_t) txmsg->cur_offset;

        /* 48 - sideband msg size - 1 byte for data CRC, x header bytes */
        space = 48U - 1U - (uint32_t) drm_dp_calc_sb_hdr_size(hdr);

        *tosend = min(len, space);
        if (len == txmsg->cur_len) {
            hdr->somt = 1;
        }
        if (space >= len) {
            hdr->eomt = 1;
        }

        hdr->msg_len = (uint8_t)(*tosend + 1U);
        drm_dp_encode_sideband_msg_hdr(hdr, chunk, idx);
        (void) memcpy(&chunk[*idx], &txmsg->msg[txmsg->cur_offset], *tosend);
        /* add crc at end */
        drm_dp_crc_sideband_chunk_req(&chunk[*idx], (uint8_t)*tosend);
        *idx += *tosend + 1U;
    }
    return ret;
}

/*
 * process a single block of the next message in the sideband queue
 */
uint32_t process_single_tx_qlock(struct MST_drm_dp_topology_mgr_s *mgr,
                                 SBM_drm_dp_sideband_msg_tx *      txmsg,
                                 bool                              up)
{
    uint8_t chunk[48];
    SBM_drm_dp_sideband_msg_hdr hdr;
    uint32_t idx, tosend;
    uint32_t ret;

    /* Clear the header */
    (void) memset(&hdr, 0, sizeof(SBM_drm_dp_sideband_msg_hdr));

    if (txmsg->state == DRM_DP_SIDEBAND_TX_QUEUED) {
        txmsg->seqno = -1;
        txmsg->state = DRM_DP_SIDEBAND_TX_START_SEND;
    }

    /* Fill the header */
    ret = single_tx_data_prepare(&hdr, txmsg, &chunk[0], &idx, &tosend);
    /* single_tx_data_prepare also writes to uint32_t variables idx, tosend, which are declared here */

    if (ret == CDN_EOK) {
        ret = drm_dp_send_sideband_msg(mgr, up, chunk, idx);
        if (ret != CDN_EOK) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                   "sideband msg failed to send\n");
        }
    }

    if (ret == CDN_EOK) {
        txmsg->cur_offset += (uint8_t) tosend;
        if (txmsg->cur_offset == txmsg->cur_len) {
            txmsg->state = DRM_DP_SIDEBAND_TX_SENT;
            ret = 1U;
        }
    }

    return ret;
}

/****************** RX *********************************************/

/** Validate sideband message header. Helper function of drm_dp_decode_sideband_msg_hdr().
 * Returns false if header is corrupted. */
static bool drm_dp_dec_sb_msg_hdr_validate(const uint8_t *buf, int32_t buflen) {
    bool ret = true;
    if (buf[0] == 0U) {
        ret = false;
    }
    if (ret) {
        /* calculate length from the first element of buf */
        uint8_t len = 3U;
        len += ((buf[0] & 0xf0U) >> 4) / 2U;
        /* fail if calculated length is greater than length passed as an argument */
        if (len > (uint8_t) buflen) {
            ret = false;
        } else {
            /* calculate the 4-bit crc */
            uint8_t crc4 = drm_dp_msg_header_crc4(buf, (((uint32_t) len * 2U) - 1U));
            /* compare it with a previously calculated crc at the end of the buf table */
            if ((crc4 & 0xfU) != (buf[len - 1U] & 0xfU)) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                       "crc4 mismatch 0x%x 0x%x\n", crc4, buf[len - 1U]);
                ret = false;
            }
        }
    }
    return ret;
}

/** Decode sideband message header. Returns false if header is corrupted and was not decoded. */
static bool drm_dp_decode_sideband_msg_hdr(SBM_drm_dp_sideband_msg_hdr *hdr,
                                           const uint8_t *buf, int32_t buflen, uint8_t *hdrlen)
{
    bool ret = true;
    int32_t i;
    uint8_t idx;

    /* Validate the header */
    ret = drm_dp_dec_sb_msg_hdr_validate(buf, buflen);

    if (ret) {
        /* Decode the header */
        hdr->lct = (buf[0] & 0xf0U) >> 4;
        hdr->lcr = (buf[0] & 0xfU);
        idx = 1U;
        for (i = 0; i < ((int32_t) hdr->lct / 2); i++) {
            hdr->rad[i] = buf[idx];
            ++idx;
        }
        if (((buf[idx] >> 7) & 0x1U) != 0U) {
            hdr->broadcast = true;
        }

        hdr->broadcast = (((buf[idx] >> 7) & 0x1U) != 0U) ? (bool)true : (bool)false;
        hdr->path_msg = (((buf[idx] >> 6) & 0x1U) != 0U) ? (bool)true : (bool)false;
        hdr->msg_len = buf[idx] & 0x3fU;
        idx++;
        hdr->somt = (((buf[idx] >> 7) & 0x1U) != 0U) ? (bool)true : (bool)false;
        hdr->eomt = (((buf[idx] >> 6) & 0x1U) != 0U) ? (bool)true : (bool)false;
        hdr->seqno = (((buf[idx] >> 4) & 0x1U) != 0U) ? (bool)true : (bool)false;
        idx++;
        *hdrlen = idx;
    }
    return ret;
}

static bool drm_dp_sideband_msg_build(SBM_drm_dp_sideband_msg_rx *msg,
                                      uint8_t *replybuf, uint8_t replybuflen, bool hdr)
{
    bool ret = true;

    if (hdr) {
        uint8_t hdrlen;
        SBM_drm_dp_sideband_msg_hdr recv_hdr;
        ret = drm_dp_decode_sideband_msg_hdr(&recv_hdr, replybuf, (int32_t) replybuflen, &hdrlen);
        if (ret) {
            msg->curchunk_len = recv_hdr.msg_len;
            msg->curchunk_hdrlen = hdrlen;

            /* we have already gotten an somt - don't bother parsing */
            if (recv_hdr.somt && msg->have_somt) {
                ret = false;
            } else {
                if (recv_hdr.somt) {
                    (void) memcpy(&msg->initial_hdr, &recv_hdr, sizeof(SBM_drm_dp_sideband_msg_hdr));
                    msg->have_somt = true;
                }
                if (recv_hdr.eomt) {
                    msg->have_eomt = true;
                }
                /* copy the bytes for the remainder of this header chunk */
                msg->curchunk_idx = (uint8_t) min(msg->curchunk_len, (uint8_t)(replybuflen - hdrlen));
                (void) memcpy(&msg->chunk[0], &replybuf[hdrlen], msg->curchunk_idx);
            }
        }
    } else {
        (void) memcpy(&msg->chunk[msg->curchunk_idx], replybuf, replybuflen);
        msg->curchunk_idx += replybuflen;
    }

    if (ret && (msg->curchunk_idx >= msg->curchunk_len)) {
        /* copy chunk into bigger msg */
        uint8_t memcpy_size = msg->curchunk_len - 1U;
        (void) memcpy(&msg->msg[msg->curlen], msg->chunk, memcpy_size);
        msg->curlen += msg->curchunk_len - 1U;
    }
    return ret;
}

bool drm_dp_get_one_sb_msg(struct MST_drm_dp_topology_mgr_s *mgr, bool up)
{
    uint32_t len, curreply;
    uint8_t replyblock[32];
    int32_t replylen;
    bool ret = true;
    uint32_t dpcdReadRetVal;
    SBM_drm_dp_sideband_msg_rx *msg;
    uint32_t basereg;
    if (up) {
        basereg = DP_SIDEBAND_MSG_UP_REQ_BASE;
        msg = &mgr->up_req_recv;
    } else {
        basereg = DP_SIDEBAND_MSG_DOWN_REP_BASE;
        msg = &mgr->down_rep_recv;
    }

    len = min(mgr->max_dpcd_transaction_bytes, 16);
    dpcdReadRetVal = drm_dp_dpcd_read(mgr->aux, basereg,
                                      replyblock, len);
    if (dpcdReadRetVal != CDN_EOK) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "failed to read DPCD down rep %d %d\n", len, ret);
        ret = false;
    }
    if (ret) {
        ret = drm_dp_sideband_msg_build(msg, replyblock, (uint8_t) len, true);
        if (!ret) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                   "sideband msg build failed %d\n", replyblock[0]);
        }
    }

    if (ret) {
        bool has_to_break = false;
        replylen = (int32_t) msg->curchunk_len + (int32_t) msg->curchunk_hdrlen;
        replylen -= (int32_t) len;
        curreply = len;
        while (replylen > 0) {
            len = min3((uint32_t)replylen, mgr->max_dpcd_transaction_bytes, 16U);
            dpcdReadRetVal = drm_dp_dpcd_read(mgr->aux, basereg + curreply,
                                              replyblock, len);
            if (has_to_break) {
                break;
            } else if (dpcdReadRetVal != CDN_EOK) {
                DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                       "failed to read a chunk (len %d, ret %d)\n",
                       len, dpcdReadRetVal);
                ret = false;
                has_to_break = true;
            } else {
                ret = drm_dp_sideband_msg_build(msg, replyblock, (uint8_t)len, false);
                if (!ret) {
                    DbgMsg(DBG_GEN_MSG, DBG_CRIT,
                           "failed to build sideband msg\n");
                    ret = false;
                    has_to_break = true;
                }
                curreply += len;
                replylen -= (int32_t) len;
            }
        }
    }
    return ret;
}
/* parasoft-end-suppress METRICS-41-3 "Number of blocks of comments per statement" */
