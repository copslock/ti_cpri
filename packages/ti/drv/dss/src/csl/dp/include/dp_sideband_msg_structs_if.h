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
#ifndef DP_SIDEBAND_MSG_STRUCTS_IF_H
#define DP_SIDEBAND_MSG_STRUCTS_IF_H

#include "cdn_stdtypes.h"
#include "dp_sideband_msg_if.h"

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
struct SBM_drm_dp_nak_reply_s {
    uint8_t guid[16];
    uint8_t reason;
    uint8_t nak_data;
};

struct SBM_drm_dp_link_addr_reply_port_s {
    bool input_port;
    uint8_t peer_device_type;
    uint8_t port_number;
    bool mcs;
    bool ddps;
    bool legacy_device_plug_status;
    uint8_t dpcd_revision;
    uint8_t peer_guid[16];
    uint8_t num_sdp_streams;
    uint8_t num_sdp_stream_sinks;
};

struct SBM_drm_dp_link_address_ack_reply_s {
    uint8_t guid[16];
    uint8_t nports;
    SBM_drm_dp_link_addr_reply_port ports[16];
};

struct SBM_drm_dp_remote_dpcd_read_ack_reply_s {
    uint8_t port_number;
    uint8_t num_bytes;
    uint8_t bytes[255];
};

struct SBM_drm_dp_remote_dpcd_write_ack_reply_s {
    uint8_t port_number;
};

struct SBM_drm_dp_remote_dpcd_write_nak_reply_s {
    uint8_t port_number;
    uint8_t reason;
    uint8_t bytes_written_before_failure;
};

struct SBM_drm_dp_remote_i2c_read_ack_reply_s {
    uint8_t port_number;
    uint8_t num_bytes;
    uint8_t bytes[255];
};

struct SBM_drm_dp_remote_i2c_read_nak_reply_s {
    uint8_t port_number;
    uint8_t nak_reason;
    uint8_t i2c_nak_transaction;
};

struct SBM_drm_dp_remote_i2c_write_ack_reply_s {
    uint8_t port_number;
};

struct SBM_drm_dp_allocate_payload_s {
    uint8_t port_number;
    uint8_t number_sdp_streams;
    uint8_t vcpi;
    uint16_t pbn;
    uint8_t sdp_stream_sink[SBM_DRM_DP_MAX_SDP_STREAMS];
};

struct SBM_drm_dp_allocate_payload_ack_reply_s {
    uint8_t port_number;
    uint8_t vcpi;
    uint16_t allocated_pbn;
};

struct SBM_drm_dp_connection_status_notify_s {
    uint8_t guid[16];
    uint8_t port_number;
    bool legacy_device_plug_status;
    bool displayport_device_plug_status;
    bool message_capability_status;
    bool input_port;
    uint8_t peer_device_type;
};

struct SBM_drm_dp_remote_dpcd_read_s {
    uint8_t port_number;
    uint32_t dpcd_address;
    uint8_t num_bytes;
};

struct SBM_drm_dp_remote_dpcd_write_s {
    uint8_t port_number;
    uint32_t dpcd_address;
    uint8_t num_bytes;
    uint8_t* bytes;
};

struct SBM_drm_dp_transactions_s {
    uint8_t i2c_dev_id;
    uint8_t num_bytes;
    uint8_t* bytes;
    uint8_t no_stop_bit;
    uint8_t i2c_transaction_delay;
};

struct SBM_drm_dp_remote_i2c_read_s {
    uint8_t num_transactions;
    uint8_t port_number;
    SBM_drm_dp_transactions transactions[SBM_DP_REMOTE_I2C_READ_MAX_TRANSACTIONS];
    uint8_t read_i2c_device_id;
    uint8_t num_bytes_read;
};

struct SBM_drm_dp_remote_i2c_write_s {
    uint8_t port_number;
    uint8_t write_i2c_device_id;
    uint8_t num_bytes;
    uint8_t* bytes;
};

struct SBM_drm_dp_port_number_req_s {
    uint8_t port_number;
};

struct SBM_drm_dp_enum_path_resources_ack_reply_s {
    uint8_t port_number;
    uint16_t full_payload_bw_number;
    uint16_t avail_payload_bw_number;
};

struct SBM_drm_dp_port_number_rep_s {
    uint8_t port_number;
};

struct SBM_drm_dp_resource_status_notify_s {
    uint8_t port_number;
    uint8_t guid[16];
    uint16_t available_pbn;
};

struct SBM_ack_req_s {
    SBM_drm_dp_connection_status_notify conn_stat;
    SBM_drm_dp_port_number_req port_num;
    SBM_drm_dp_resource_status_notify resource_stat;
    SBM_drm_dp_allocate_payload allocate_payload;
    SBM_drm_dp_remote_dpcd_read dpcd_read;
    SBM_drm_dp_remote_dpcd_write dpcd_write;
    SBM_drm_dp_remote_i2c_read i2c_read;
    SBM_drm_dp_remote_i2c_write i2c_write;
};

struct SBM_drm_dp_sideband_msg_req_body_s {
    uint8_t req_type;
    SBM_ack_req u;
};

struct SBM_ack_replies_s {
    SBM_drm_dp_nak_reply nak;
    SBM_drm_dp_link_address_ack_reply link_addr;
    SBM_drm_dp_port_number_rep port_number;
    SBM_drm_dp_enum_path_resources_ack_reply path_resources;
    SBM_drm_dp_allocate_payload_ack_reply allocate_payload;
    SBM_drm_dp_remote_dpcd_read_ack_reply remote_dpcd_read_ack;
    SBM_drm_dp_remote_dpcd_write_ack_reply remote_dpcd_write_ack;
    SBM_drm_dp_remote_dpcd_write_nak_reply remote_dpcd_write_nack;
    SBM_drm_dp_remote_i2c_read_ack_reply remote_i2c_read_ack;
    SBM_drm_dp_remote_i2c_read_nak_reply remote_i2c_read_nack;
    SBM_drm_dp_remote_i2c_write_ack_reply remote_i2c_write_ack;
};

struct SBM_drm_dp_sideband_msg_reply_body_s {
    uint8_t reply_type;
    uint8_t req_type;
    SBM_ack_replies u;
};

struct SBM_drm_dp_sideband_msg_tx_s {
    uint8_t msg[256];
    uint8_t chunk[48];
    uint8_t cur_offset;
    uint8_t cur_len;
    struct MST_drm_dp_branch_s* dst;
    int32_t seqno;
    uint32_t state;
    bool path_msg;
    SBM_drm_dp_sideband_msg_reply_body reply;
};

struct SBM_drm_dp_sideband_msg_hdr_s {
    uint8_t lct;
    uint8_t lcr;
    uint8_t rad[8];
    bool broadcast;
    bool path_msg;
    uint8_t msg_len;
    bool somt;
    bool eomt;
    bool seqno;
};

struct SBM_drm_dp_sideband_msg_rx_s {
    uint8_t chunk[48];
    uint8_t msg[256];
    uint8_t curchunk_len;
    uint8_t curchunk_idx;
    uint8_t curchunk_hdrlen;
    uint8_t curlen;
    bool have_somt;
    bool have_eomt;
    SBM_drm_dp_sideband_msg_hdr initial_hdr;
};

/**
 *  @}
 */

#endif  /* DP_SIDEBAND_MSG_STRUCTS_IF_H */
