/* parasoft-begin-suppress METRICS-36 "Function called from more than 5 functions" */
/* parasoft-begin-suppress MISRA2012-DIR-4_8 "Hidden implementation when never dereferenced structure" */
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
 * dp_aux.h
 *
 ******************************************************************************
 */

#ifndef DP_AUX_H
#define DP_AUX_H

#include <cdn_stdtypes.h>
#include "dp_if.h"
#include "cdn_log.h"
#include "dp_aux_if.h"

uint32_t drm_dp_dpcd_read(AUX_drm_dp *aux, uint32_t offset,
                          uint8_t *buffer, size_t size);

uint32_t drm_dp_dpcd_write(AUX_drm_dp *aux, uint32_t offset,
                           uint8_t *buffer, size_t size);

uint32_t drm_dp_dpcd_read_byte(AUX_drm_dp *aux, uint32_t offset,
                               uint8_t *value);

uint32_t drm_dp_dpcd_write_byte(AUX_drm_dp *aux, uint32_t offset,
                                uint8_t value);

#endif
