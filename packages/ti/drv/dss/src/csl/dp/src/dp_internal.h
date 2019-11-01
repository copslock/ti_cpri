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
 * dp_internal.h
 *
 ******************************************************************************
 */

#ifndef DP_INERNAL_H
#define DP_INERNAL_H

#include "cdn_stdtypes.h"

/**
 * Prototypes of non-API functions used in multiple driver's files.
 */

uint16_t getSymbolRate(DP_LinkRate linkRate);

float64_t calculateBitsPerComponent(const DP_PrivateData* pD, uint8_t streamId);

/**
 * General and DPTX command/response IDs
 */

typedef enum
{
    GENERAL_MAIN_CONTROL = 0x01,
    GENERAL_TEST_ECHO = 0x02,
    GENERAL_WRITE_REGISTER = 0x05,
    GENERAL_WRITE_FIELD = 0x06,
    GENERAL_READ_REGISTER = 0x07,
    GENERAL_GET_HPD_STATE = 0x11,
    GENERAL_WAIT = 0x08,
    GENERAL_SET_WATCHDOG_CFG = 0x09,
    GENERAL_INJECT_ECC_ERROR = 0x0A,
    GENERAL_FORCE_FATAL_ERROR = 0x0B
} GENERAL_MAILBOX_MSG_ID;

typedef enum
{
    DPTX_SET_POWER_MNG = 0x00,
    DPTX_GET_EDID = 0x02,
    DPTX_READ_DPCD = 0x03,
    DPTX_WRITE_DPCD = 0x04,
    DPTX_ENABLE_EVENT = 0x05,
    DPTX_READ_EVENT = 0x0A,
    DPTX_GET_LAST_AUX_STAUS = 0x0E,
    DPTX_HPD_STATE = 0x11,
    DPTX_LT_ADJUST = 0x12,
    DPTX_I2C_READ = 0x15,
    DPTX_I2C_WRITE = 0x16,
    DPTX_GET_LAST_I2C_STATUS = 0x17
} DPTX_MAILBOX_MSG_ID;

/* DisplayPort Configuration Data defines */
/*****************************************************************************/
/* FEC_CAPABILITY offset in DPCD */
#define DP_DPCD_FEC_CAPABILITY                      0x90U
/* FEC capable mask */
#define DP_DPCD_FEC_CAPABILITY_FEC_CAPABLE_MASK     (1U << 0)
/* FEC uncorrected block error count capable mask*/
#define DP_DPCD_FEC_CAPABILITY_UBECC_MASK           (1U << 1)
/* FEC corrected block error count capable mask*/
#define DP_DPCD_FEC_CAPABILITY_CBECC_MASK           (1U << 2)
/* FEC bit error count capable mask*/
#define DP_DPCD_FEC_CAPABILITY_BECC_MASK            (1U << 3)

/* FEC_CAPABILITY offset in DPCD */
#define DP_DPCD_FEC_CONFIGURATION                   0x120U
/* FEC_READY mask*/
#define DP_DPCD_FEC_CONFIGURATION_FEC_READY         (1U << 0)
/*****************************************************************************/

#endif /* DP_INERNAL_H */
