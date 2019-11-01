/* parasoft suppress item  MISRA2012-DIR-4_8 "Consider hiding implementation of structure" */
/**********************************************************************
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
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.03.9e11b77(origin/DRV-3827_extract_sanity_to_c_file)
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for the Cadence DisplayPort (DP) core. This header
* file lists the Supporting structures for the DP core driver
**********************************************************************/
#ifndef DP_MST_STRUCTS_IF_H
#define DP_MST_STRUCTS_IF_H

#include "cdn_stdtypes.h"
#include "dp_mst_if.h"

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Structures and unions
**********************************************************************/
/**
 * struct DP_drm_dp_vcpi - Virtual Channel Payload Identifier
 * @vcpi: Virtual channel ID.
 * @pbn: Payload Bandwidth Number for this channel
 * @aligned_pbn: PBN aligned with slot size
 * @num_slots: number of slots for this PBN
 * @parent: Pointer to port containing this VCPI
 */
struct MST_drm_dp_vcpi_s {
    int32_t vcpi;
    int32_t pbn;
    int32_t aligned_pbn;
    int32_t num_slots;
    MST_drm_dp_port* parent;
};

/**
 * struct DP_drm_dp_port - MST port
 * @kref: reference count for this port.
 * @port_num: port number
 * @input: if this port is an input port.
 * @mcs: message capability status - DP 1.2 spec.
 * @ddps: DisplayPort Device Plug Status - DP 1.2
 * @pdt: Peer Device Type
 * @ldps: Legacy Device Plug Status
 * @dpcd_rev: DPCD revision of device on this port
 * @num_sdp_streams: Number of simultaneous streams
 * @num_sdp_stream_sinks: Number of stream sinks
 * @available_pbn: Available bandwidth for this port.
 * @next: link to next port on this branch device
 * @mstb: branch device attach below this port
 * @aux: i2c aux transport to talk to device connected to this port.
 * @parent: branch device parent of this port
 * @vcpi: Virtual Channel Payload info for this port.
 * @connector: DRM connector this port is connected to.
 * @mgr: topology manager this port lives under.
 * This structure represents an MST port endpoint on a device somewhere
 * in the MST topology.
 */
struct MST_drm_dp_port_s {
    uint32_t refcount;
    uint8_t port_num;
    bool input;
    bool mcs;
    bool ddps;
    uint8_t pdt;
    bool ldps;
    uint8_t dpcd_rev;
    uint8_t num_sdp_streams;
    uint8_t num_sdp_stream_sinks;
    int16_t available_pbn;
    /** pointer to an mstb if this port has one. */
    MST_drm_dp_branch* mstb;
    MST_drm_dp_branch* parent;
    MST_drm_dp_vcpi vcpi;
    struct drm_connector* connector;
    MST_drm_dp_topology_mgr* mgr;
    /**
     * @cached_edid: for DP logical ports - make tiling work by ensuring
     * that the EDID for all connectors is read immediately.
     */
    struct edid* cached_edid;
    /**
     * @has_audio: Tracks whether the sink connector to this port is
     * audio-capable.
     */
    bool has_audio;
};

/**
 * struct DP_drm_dp_branch - MST branch device.
 * @kref: reference count for this port.
 * @rad: Relative Address to talk to this branch device.
 * @lct: Link count total to talk to this branch device.
 * @num_ports: number of ports on the branch.
 * @msg_slots: one bit per transmitted msg slot.
 * @ports: linked list of ports on this branch.
 * @port_parent: pointer to the port parent, NULL if toplevel.
 * @mgr: topology manager for this branch device.
 * @tx_slots: transmission slots for this device.
 * @last_seqno: last sequence number used to talk to this.
 * @link_address_sent: if a link address message has been sent to this device yet.
 * @guid: guid for DP 1.2 branch device. port under this branch can be
 * identified by port #.
 * This structure represents an MST branch device, there is one
 * primary branch device at the root, along with any other branches connected
 * to downstream port of parent branches.
 */
struct MST_drm_dp_branch_s {
    uint32_t refcount;
    uint8_t rad[8];
    uint8_t lct;
    uint8_t num_ports;
    int32_t msg_slots;
    MST_drm_dp_port ports[MST_BRANCH_MAX_PORTS];
    MST_drm_dp_port* port_parent;
    MST_drm_dp_topology_mgr* mgr;
    SBM_drm_dp_sideband_msg_tx* tx_slots[2];
    int32_t last_seqno;
    bool link_address_sent;
    uint8_t guid[16];
    /** @isOccupied: if memory slot is currently used */
    bool isOccupied;
};

struct MST_drm_dp_payload_s {
    int32_t payload_state;
    uint8_t start_slot;
    uint8_t num_slots;
    int32_t vcpi;
};

struct MST_demoTb_s {
    /** Disable sending sideband messages through AUX channel */
    bool disableMessaging;
    /** Inject requested number as VCPI */
    uint8_t requestedVcpi;
};

/**
 * struct DP_drm_dp_topology_mgr - DisplayPort MST manager
 * This struct represents the toplevel displayport MST topology manager.
 * There should be one instance of this for every MST capable DP connector
 * on the GPU.
 */
struct MST_drm_dp_topology_mgr_s {
    /** information if incoming communication is handled by interrupt or polling */
    bool int_enabled;
    /**
     * @max_dpcd_transaction_bytes: maximum number of bytes to read/write
     * in one go.
     */
    uint32_t max_dpcd_transaction_bytes;
    /**
     * @aux: AUX channel for the DP MST connector this topolgy mgr is
     * controlling.
     */
    struct AUX_drm_dp_s* aux;
    /**
     * @down_rep_recv: Message receiver state for down replies. This and
     * @up_req_recv are only ever access from the work item, which is
     * serialised.
     */
    SBM_drm_dp_sideband_msg_rx down_rep_recv;
    /**
     * @up_req_recv: Message receiver state for up requests. This and
     * @down_rep_recv are only ever access from the work item, which is
     * serialised.
     */
    SBM_drm_dp_sideband_msg_rx up_req_recv;
    /**
     * @mst_state: If this manager is enabled for an MST capable port. False
     * if no MST sink/branch devices is connected.
     */
    bool mst_state;
    /** @mst_primary: Pointer to the primary/first branch device. */
    MST_drm_dp_branch* mst_primary;
    /** @branch_list: list of branches  */
    MST_drm_dp_branch branch_list[MST_MAX_BRANCH_NUMBER];
    /** Cache of DPCD for primary port. */
    uint8_t dpcd[MST_DPCD_RECEIVER_CAP_SIZE];
    /** @sink_count: Sink count from DEVICE_SERVICE_IRQ_VECTOR_ESI0. */
    uint8_t sink_count;
    /** @pbn_div: PBN to slots divisor. */
    int32_t pbn_div;
    /** Number of available, free slots. */
    int32_t avail_slots;
    /** Implemented as single element null or message. */
    SBM_drm_dp_sideband_msg_tx* tx_msg_downq;
    /**
     * @proposed_vcpis: Array of pointers for the new VCPI allocation. The
     * VCPI structure itself is &DP_drm_dp_port.vcpi.
     */
    MST_drm_dp_vcpi* proposed_vcpis[MST_MAX_PAYLOAD];
    /** @payloads: Array of payloads. */
    MST_drm_dp_payload payloads[MST_MAX_PAYLOAD];
    /**
     * @payload_mask: Elements of @payloads actually in use. Since
     * reallocation of active outputs isn't possible gaps can be created by
     * disabling outputs out of order compared to how they've been enabled.
     */
    uint64_t payload_mask;
    /** @vcpi_mask: Similar to @payload_mask, but for @proposed_vcpis. */
    uint64_t vcpi_mask;
    /** Specifies the target bits */
    MST_demoTb demotb;
};

/**
 *  @}
 */

#endif  /* DP_MST_STRUCTS_IF_H */
