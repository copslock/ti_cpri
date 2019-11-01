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
 * dp_mst.h
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-36 "Function called from more then 5 functions in one translation unit" */

#ifndef DP_MST_H
#define DP_MST_H

#include "cdn_stdtypes.h"
#include "dp_if.h"
#include "dp_priv.h"

/*
 * Set MST (enable = 'true') or SST (enable = 'false') mode
 * @param[in] pD pointer to private data object
 * @param[in] enable indicates if mode is SST/MST
 * @return CDN_EOK if success
 * @return CDN_EINVAL if input parameters are invaild
 */
uint32_t DP_MST_SetMstEnable(DP_PrivateData* pD, bool mstEnable);

/*
 * Set stream disabled/enabled
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] enable indicator if disable/enable stream
 * @return CDN_EOK if success
 * @return CDN_EINVAL if input parameters are invaild
 */
uint32_t DP_MST_SetStreamEnable(DP_PrivateData* pD, uint8_t streamId, bool streamEnable);

/*
 * Allocate virtual channel for sink device stream
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] sinkDevice pointer to sink device object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if failed
 */
uint32_t DP_MST_AllocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice);

/*
 * Deallocate virtual channel for sink device stream
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] sinkDevice pointer to sink device object
 * @return CDN_EOK if success
 * @return CDN_EINVAL if failed
 */
uint32_t DP_MST_DeallocateVcpi(DP_PrivateData *pD, uint8_t streamId, DP_SinkDevice *sinkDevice);

/*
 * Set encryption for MST timeslots
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @param[in] enable if encyption should be disabled/enabled
 * @return CDN_EOK if success
 * @return CDN_EINVAL if failed
 */
uint32_t DP_MST_SetEncryption(DP_PrivateData* pD, uint8_t streamId, bool enable);

/*
 * Check if any stream has video on
 * @param[in] pD pointer to DP private data object
 * @param[in] isOn indicator if any video is on
 * @return CDN_EOK if success
 * @return CDN_EINVAL if failed
 */
uint32_t DP_MST_IsAnyVideoOn(DP_PrivateData* pD, bool *isOn);

/*
 * Check, if MST feature is enabled and if given stream ID is within number of
 * streams supported by hardware.
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @return CDN_EOK if success
 * @return CDN_ENOTSUP if MST mode is not enabled
 * @return CDN_EINVAL if streamId is greater than number of streams
 */
uint32_t DP_StreamIdMstSanity(const DP_PrivateData* pD, uint8_t streamId);

/*
 * Check, if given stream ID is valid for current DSC configuration.
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId number of stream
 * @param[in] splitPanel inicator if panel is splitted
 * @return CDN_EOK if success
 * @return CDN_EINVAL if stream is not supported
 */
uint32_t DP_StreamIdMstSstDscSanity(const DP_PrivateData* pD,
                                    uint8_t               streamId,
                                    bool                  splitPanel);

/*
 * Check, if given stream ID is within number of streams supported by hardware.
 * @param[in] pD pointer to DP private data object
 * @param[in] streamId stream number
 * @return CDN_EOK if success
 * @return CDN_EINVAL if stream is not supported
 */
uint32_t DP_StreamIdMstSstSanity(const DP_PrivateData* pD, uint8_t streamId);

#endif

/* parasoft-end-suppress METRICS-36 */
