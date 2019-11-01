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
 * dp_link_policy.h
 *
 ******************************************************************************
 */

#ifndef DP_LINK_POLICY_H
#define DP_LINK_POLICY_H

#include "dp_if.h"
#include "dp_priv.h"

#include "dp_sd0801_if.h"

#define COMMON_DPHY_CONFIG CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_SKEW_BYPASS, 0, 0) | \
    CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_LANE0_SKEW, 0, 0) | \
    CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_LANE1_SKEW, 0, 1) | \
    CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_LANE2_SKEW, 0, 2) | \
    CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_LANE3_SKEW, 0, 3) | \
    CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__DP_TX_PHY_CONFIG_REG_P, DP_TX_PHY_10BIT_ENABLE, 0, 0)

void toSdLinkState (const DP_LinkState* dpState, DP_SD0801_LinkState* sdState);
DP_SD0801_LinkRate toLinkRateSd(DP_LinkRate rate);
void clearSourceCapabilities(DP_SourceDeviceCapabilities *caps);
void clearSinkCapabilities(DP_SinkDeviceCapabilities *caps);
uint32_t readSinkCapabilities(DP_PrivateData *pD);
void clearLinkState(DP_LinkState* state);
uint32_t fetchLinkState(DP_PrivateData* pD);

#endif /* DP_LINK_POLICY_H */
