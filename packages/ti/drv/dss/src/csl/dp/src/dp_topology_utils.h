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
 * dp_topology_utils.h
 *
 ******************************************************************************
 */

/*
 * Copyright C 2014 Red Hat.
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

/* parasoft-begin-suppress METRICS-36-3 "Function is called from more than 5 different functions, DRV-3823" */

#ifndef DP_TOPOLOGY_UTILS_H_
#define DP_TOPOLOGY_UTILS_H_

#include "dp_mst_if.h"
#include "dp_mst_structs_if.h"
#include "dp_topology_mgr.h"

/*
 * Returns address to new branch device
 * @param[in] topologyManager pointer to topology manager object
 * @param[in] linkCount number of links for branch
 * @param[in] relativeAddress pointer to buffer with relative address of branch
 * @return pointer to branch if success
 * @return NULL if no space for new branch
 */
MST_drm_dp_branch *DRM_DP_addBranchDevice(MST_drm_dp_topology_mgr* topologyManager,
                                          uint8_t                  linkCount,
                                          uint8_t *                relativeAddress);

/* Looking for branch in topology
 * @param[in] toplogyManager pointer to topology object
 * @param[in] mstBranch pointer to searched branch
 * @return pointer to founded branch if success
 * @return NULL if branch is not in topology
 */
MST_drm_dp_branch *DRM_DP_getValidatedBranchRef(MST_drm_dp_topology_mgr *topologyManager,
                                                const MST_drm_dp_branch *mstBranch);

/*
 * Looking for port in topology
 * @param[in] topologyManager pointer to topology object
 * @param[in] port pointer to searched port
 * @return pointer to searched port if success
 * @return NULL if port is not in topology
 */
MST_drm_dp_port *DRM_DP_getValidatedPortRef(MST_drm_dp_topology_mgr *topologyManager,
                                            const MST_drm_dp_port *  port);

/*
 * Return reference to searched branch
 * @param[in] topologyManager pointer to topology object
 * @param[in] linkCount number of links for branch
 * @param[in] relativeAddress pointer to buffer with relative address of branch
 * @return pointer to branch if success
 * @return NULL if branch not found
 */
MST_drm_dp_branch *DRM_DP_getBranchDevice(MST_drm_dp_topology_mgr* topologyManager,
                                          uint8_t                  linkCount,
                                          const uint8_t*           relativeAddress);

/*
 * Decrease reference count for branch and deallocate branches if should
 * @param[in] mstBranch pointer to branch object
 */
void DRM_DP_putBranchDevice(MST_drm_dp_branch *mstBranch);

/*
 * Decrease reference count for port and deallocate branches if should
 * @param[in] port pointer to port object
 */
void DRM_DP_putPort(MST_drm_dp_port *port);

/*
 * Return reference to searched port
 * @param[in] mstBranch pointer to branch object
 * @param[in] portNumber number of port
 * @return pointer to port if success
 * @return NULL if port not found
 */
MST_drm_dp_port *DRM_DP_getPort(MST_drm_dp_branch *mstBranch, uint8_t portNumber);

/*
 * Remove reference to branch in port when branch type is oldPeerDEviceType
 * and deallocate branches if should
 * @param[in] port pointer to port object
 * @param[in] oldPeerDeviceType peerDeviceType for port
 */
void DRM_DP_teardownPort(MST_drm_dp_port *port, uint8_t oldPeerDeviceType);

/*
 * Find reference to branch using GUID
 * @param[in] topologyManager pointer to topology object
 * @param[in] guid pointer to GUID buffer
 * @return reference to branch if found
 * @return NULL if not found
 * */
MST_drm_dp_branch *DRM_DP_getBranchDeviceByGuid(MST_drm_dp_topology_mgr *topologyManager,
                                                uint8_t *                guid);

#endif /* DP_TOPOLOGY_UTILS_H_ */

/* parasoft-end-suppress METRICS-36-3 */
