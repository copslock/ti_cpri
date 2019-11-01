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
 * dp_topology_mgr.h
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

#ifndef TOPOLOGY_MGR
#define TOPOLOGY_MGR

#include "dp_sideband_msg.h"
#include "dp_if.h"
#include "dp_mst_if.h"
#include "dp_mst_structs_if.h"
#include "dp_sideband_msg_if.h"
#include "dp_sideband_msg_structs_if.h"

#ifdef IS_DEMO_TB
#include "dp_demotb.h"
#endif

#include "cdn_stdtypes.h"
#include "cdn_errno.h"
#include "custom_types.h"
#include "cdn_math.h"

/* DP 1.2 Sideband message defines */
/* peer device type - DP 1.2a Table 2-92 */
#define DP_PEER_DEVICE_NONE     0x0U
#define DP_PEER_DEVICE_SOURCE_OR_SST    0x1U
#define DP_PEER_DEVICE_MST_BRANCHING    0x2U
#define DP_PEER_DEVICE_SST_SINK     0x3U
#define DP_PEER_DEVICE_DP_LEGACY_CONV   0x4U

void DP_MST_MgrInit(MST_drm_dp_topology_mgr *mgr,
                    struct AUX_drm_dp_s *    aux,
                    uint32_t                 max_dpcd_transaction_bytes);

uint32_t DP_MST_MgrSetState(MST_drm_dp_topology_mgr *mgr, bool mst_state);

void drm_dp_mst_hpd_irq(MST_drm_dp_topology_mgr *mgr, const uint8_t *esi, bool *handled);

void DP_MST_MgrSetPayloadBandwidth(MST_drm_dp_topology_mgr *mgr,
                                   uint32_t                 symbolRate,
                                   uint8_t                  laneCount);

uint32_t DP_MST_MgrCalcPbnMode(uint32_t dot_clock, uint8_t bpp, bool fecEnabled);

uint32_t DP_MST_MgrAllocateVcpi(MST_drm_dp_topology_mgr *mgr,
                                MST_drm_dp_port *port, uint32_t pbn, uint8_t slots);

void DP_MST_MgrDeallocateVcpi(MST_drm_dp_topology_mgr *mgr, MST_drm_dp_port *portIn);

uint32_t DP_MST_MgrUpdatePayloadPart1(MST_drm_dp_topology_mgr *mgr);

uint32_t DP_MST_MgrUpdatePayloadPart2(MST_drm_dp_topology_mgr *mgr);

uint32_t DP_MST_MgrCheckActStatus(MST_drm_dp_topology_mgr *mgr);

uint32_t DP_MST_MgrFindVcpiSlots(MST_drm_dp_topology_mgr *mgr,
                                 MST_drm_dp_port *port, uint32_t pbn, uint8_t *slots, float32_t *fracSlots);

MST_drm_dp_payload  *DP_MST_MgrGetPortPayload(MST_drm_dp_topology_mgr *mgr,
                                              MST_drm_dp_port *        port);

bool drm_dp_mst_handle_hpd(MST_drm_dp_topology_mgr *mgr);

uint32_t drm_do_probe_ddc_edid(MST_drm_dp_port *port, uint8_t *buf, uint32_t block,
                               uint8_t len);

#endif
