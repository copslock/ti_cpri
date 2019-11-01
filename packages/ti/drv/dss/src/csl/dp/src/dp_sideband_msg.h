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
 * dp_sideband_msg.h
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

#ifndef DP_SIDEBAND_MSG
#define DP_SIDEBAND_MSG

#include <cdn_stdtypes.h>
#include "dp_sideband_msg_if.h"
#include "dp_sideband_msg_structs_if.h"

/***************************************/
/*TODO to be removed*/
/***************************************/
#include <stdio.h>
#include <string.h>
/***************************************/

#define DP_GUID                 0x030   /* 1.2 */

/* DPCD */

#define DP_SIDEBAND_MSG_DOWN_REQ_BASE       0x1000U   /* 1.2 MST */
#define DP_SIDEBAND_MSG_UP_REP_BASE     0x1200U   /* 1.2 MST */
#define DP_SIDEBAND_MSG_DOWN_REP_BASE       0x1400U   /* 1.2 MST */
#define DP_SIDEBAND_MSG_UP_REQ_BASE     0x1600U   /* 1.2 MST */
/* DPCD */

/* msg is queued to be put into a slot */
#define DRM_DP_SIDEBAND_TX_QUEUED 0U
/* msg has started transmitting on a slot - still on msgq */
#define DRM_DP_SIDEBAND_TX_START_SEND 1U
/* msg has finished transmitting on a slot - removed from msgq only in slot */
#define DRM_DP_SIDEBAND_TX_SENT 2U
/* msg has received a response - removed from slot */
#define DRM_DP_SIDEBAND_TX_RX 3U
#define DRM_DP_SIDEBAND_TX_TIMEOUT 4U

#define DP_SINK_COUNT_ESI           0x2002   /* 1.2 */
/* 0-5 sink count */
# define DP_SINK_COUNT_CP_READY             (1U << 6)
#define DP_DEV_SERVICE_IRQ_VECTOR_ESI0   0x2003   /* 1.2 */
# define DP_DOWN_REP_MSG_RDY            (1U << 4) /* 1.2 MST */
# define DP_UP_REQ_MSG_RDY          (1U << 5) /* 1.2 MST */

#define DP_DEV_SERVICE_IRQ_VECTOR_ESI1   0x2004   /* 1.2 */

/* DP 1.2 MST sideband request names DP 1.2a Table 2-80 */
#define DP_LINK_ADDRESS         0x01U
#define DP_CONNECTION_STATUS_NOTIFY 0x02U
#define DP_ENUM_PATH_RESOURCES      0x10U
#define DP_ALLOCATE_PAYLOAD     0x11U
#define DP_RESOURCE_STATUS_NOTIFY   0x13U
#define DP_CLEAR_PAYLOAD_ID_TABLE   0x14U
#define DP_REMOTE_DPCD_READ     0x20U
#define DP_REMOTE_DPCD_WRITE        0x21U
#define DP_REMOTE_I2C_READ      0x22U
#define DP_REMOTE_I2C_WRITE     0x23U
#define DP_POWER_UP_PHY         0x24U
#define DP_POWER_DOWN_PHY       0x25U
#define DP_SINK_EVENT_NOTIFY        0x30U
#define DP_QUERY_STREAM_ENC_STATUS  0x38U

uint32_t process_single_tx_qlock(MST_drm_dp_topology_mgr *   mgr,
                                 SBM_drm_dp_sideband_msg_tx *txmsg,
                                 bool                        up);

bool drm_dp_get_one_sb_msg(MST_drm_dp_topology_mgr *mgr, bool up);

#endif
