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

#ifndef DP_MST_IF_H
#define DP_MST_IF_H

/* parasoft-begin-suppress MISRA2012-RULE-1_1_a_c90-2 "C90 - limits, DRV-3906" */
/* parasoft-begin-suppress MISRA2012-RULE-1_1_b_c90-2 "C90 - limits, DRV-3906" */

#include "cdn_errno.h"
#include "cdn_math.h"
#include "cdn_stdtypes.h"
#include "custom_types.h"
#include "dp_aux_if.h"
#include "dp_if.h"
#include "dp_sideband_msg_if.h"
#include "dp_sideband_msg_structs_if.h"

/** @defgroup ConfigInfo  Configuration and Hardware Operation Information
 *  The following definitions specify the driver operation environment that
 *  is defined by hardware configuration or client code. These defines are
 *  located in the header file of the core driver.
 *  @{
 */

/**********************************************************************
* Defines
**********************************************************************/
#define MST_DPCD_MSTM_CAP 0x021U

#define MST_DPCD_MST_CAP (1U << 0U)

#define MST_DPCD_LINK_BW_SET 0x100U

#define MST_DPCD_LINK_RATE_TABLE 0x00U

#define MST_DPCD_LINK_BW_1_62 0x06U

#define MST_DPCD_LINK_BW_2_7 0x0aU

#define MST_DPCD_LINK_BW_5_4 0x14U

#define MST_DPCD_LINK_BW_8_1 0x1eU

#define MST_DPCD_PAYLOAD_TABLE_UPDATE_STATUS 0x2c0U

#define MST_DPCD_PAYLOAD_TABLE_UPDATED (1U << 0U)

#define MST_DPCD_PAYLOAD_ACT_HANDLED (1U << 1U)

#define MST_DPCD_PAYLOAD_ALLOCATE_SET 0x1c0U

#define MST_DPCD_PAYLOAD_ALLOCATE_START_TIME_SLOT 0x1c1U

#define MST_DPCD_PAYLOAD_ALLOCATE_TIME_SLOT_COUNT 0x1c2U

#define MST_DPCD_RECEIVER_CAP_SIZE 0xfU

#define MST_DPCD_REV 0x000U

#define MST_DPCD_MAX_LANE_COUNT_MASK 0x1fU

#define MST_DPCD_MSTM_CTRL 0x111U

#define MST_DPCD_MST_EN (1U << 0U)

#define MST_DPCD_UP_REQ_EN (1U << 1U)

#define MST_DPCD_UPSTREAM_IS_SRC (1U << 2U)

#define MST_DPCD_FAUX_CAP 0x020U

#define MST_DPCD_BRANCH_OUI 0x500U

#define MST_DPCD_BRANCH_OUI_HEADER_SIZE 0xcU

#define MST_LINK_RATE_MULTIPLIER (54U)

#define MST_BRANCH_MAX_PORTS 16U

#define MST_MAX_PAYLOAD 63U

#define MST_PAYLOAD_LOCAL 1U

#define MST_PAYLOAD_REMOTE 2U

#define MST_PAYLOAD_DELETE_LOCAL 3U

#define MST_MAX_BRANCH_NUMBER 64U

#define MST_MAX_BRANCH_DEEP_LEVEL 15U

#define MST_NUMBER_OF_SDP 16U

/**
 *  @}
 */

/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
* Forward declarations
**********************************************************************/
typedef struct MST_drm_dp_vcpi_s MST_drm_dp_vcpi;
typedef struct MST_drm_dp_port_s MST_drm_dp_port;
typedef struct MST_drm_dp_branch_s MST_drm_dp_branch;
typedef struct MST_drm_dp_payload_s MST_drm_dp_payload;
typedef struct MST_demoTb_s MST_demoTb;
typedef struct MST_drm_dp_topology_mgr_s MST_drm_dp_topology_mgr;

/**
 *  @}
 */

/* parasoft-end-suppress MISRA2012-RULE-1_1_b_c90-2 */
/* parasoft-end-suppress MISRA2012-RULE-1_1_a_c90-2 */

#endif  /* DP_MST_IF_H */
