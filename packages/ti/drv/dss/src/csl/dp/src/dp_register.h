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
 * dp_register.h
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress MISRA2012-DIR-4_8-4 "Consider hiding of implementation" */
/* parasoft-begin-suppress METRICS-36-3 "Function is called from 5 different functions, DRV-3823" */

#ifndef DP_REGISTER_H
#define DP_REGISTER_H

#include "cdn_stdtypes.h"
#include "dp_if.h"
#include "dp_priv.h"

/** Structure used for reading/writing registers. */
struct DP_RegisterTransfer_s
{
    /** Address of register to read/write (relative to register base address) */
    uint32_t addr;
    /** Value read from / to write to register */
    uint32_t val;
    /** Bus Type to use. */
    DP_BusType busType;
};

/** Structure with request data for writing register field. */
struct DP_WriteFieldRequest_s
{
    /** Address of register, which contains field to be written. */
    uint32_t addr;
    /** Position of first bit of field in the register. */
    uint8_t startBit;
    /** Number of bits of the field. */
    uint8_t bitCount;
    /** Value to write to field (relative to register's least significant bit, not field's). */
    uint32_t val;
};

typedef struct DP_RegisterTransfer_s DP_RegisterTransfer;
typedef struct DP_WriteFieldRequest_s DP_WriteFieldRequest;

/**
 * Reads DP controller's register via Firmware.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with structure used to read register.
 * @return CDN_EOK success
 * @return CDN_ENOEXEC Wrong Module or Operation ID was received in response form FW.
 * @return CDN_EIO Address of read register does not match request, or is invalid.
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_ReadRegister(DP_PrivateData* pD, DP_RegisterTransfer* transfer);

/**
 * Writes DP controller's register via Firmware.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] transfer Pointer with structure used to write register.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_WriteRegister(DP_PrivateData* pD, const DP_RegisterTransfer* transfer);

/**
 * Writes field of DP controller's register via Firmware.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] request Pointer to structure with register field write request.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or request pointer is NULL.
 */
uint32_t DP_WriteField(DP_PrivateData* pD, const DP_WriteFieldRequest* request);

/**
 * Reads DP controller's register via APB.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in,out] transfer Pointer with structure used to read register.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_ReadLocalRegister(DP_PrivateData* pD, DP_RegisterTransfer* transfer);

/**
 * Writes DP controller's register via APB.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] transfer Pointer with structure used to write register.
 * @return CDN_EOK success
 * @return CDN_EINVAL If pD or transfer pointer is NULL.
 */
uint32_t DP_WriteLocalRegister(DP_PrivateData* pD, const DP_RegisterTransfer* transfer);

#endif /* DP_REGISTER_H */

/* parasoft-end-suppress MISRA2012-DIR-4_8-4 */
/* parasoft-end-suppress METRICS-36-3 */
