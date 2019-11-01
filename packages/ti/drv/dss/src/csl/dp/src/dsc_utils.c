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
 * dsc_utils.c
 *
 ******************************************************************************
 */
/***************************************************************************
*    Copyright (c) 2013-2016, Broadcom Ltd.
*    All rights reserved.
*
*  Statement regarding contribution of copyrighted materials to VESA:
*
*  This code is owned by Broadcom Limited and is contributed to VESA
*  for inclusion and use in its VESA Display Stream Compression specification.
*  Accordingly, VESA is hereby granted a worldwide, perpetual, non-exclusive
*  license to revise, modify and create derivative works to this code and
*  VESA shall own all right, title and interest in and to any derivative
*  works authored by VESA.
*
*  Terms and Conditions
*
*  Without limiting the foregoing, you agree that your use
*  of this software program does not convey any rights to you in any of
*  Broadcom's patent and other intellectual property, and you
*  acknowledge that your use of this software may require that
*  you separately obtain patent or other intellectual property
*  rights from Broadcom or third parties.
*
*  Except as expressly set forth in a separate written license agreement
*  between you and Broadcom, if applicable:
*
*  1. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
*  "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
*  REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
*  OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
*  DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
*  NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
*  ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
*  CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
*  OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
*
*  2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
*  BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
*  SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
*  IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
*  IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR (ii)
*  ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE ITSELF
*  OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
*  NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
***************************************************************************/

/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */
/* DRV-3823 */

#include "cdn_math.h"
#include <cdn_log.h>
#include "dsc_utils.h"

/** Return maximum of 2 - element set (int32_t). */
static int32_t max_signed_int_32(int32_t x, int32_t y) {
    return ((x > y) ? x : y);
}

/** Return minimum of 2 - element set (uint32_t). */
static uint32_t min_unsigned_int_32(uint32_t x, uint32_t y) {
    return ((x < y) ? x : y);
}

/** Return x clamped to <min, max> range (uint32_t). */
static uint32_t clamp_unsigned_int_32(uint32_t x, uint32_t min, uint32_t max) {
    return ((x > max) ? max : (x < min) ? min : x);
}

/** Return CDN_EINVAL if int32_t value is not in range <bottom, top>. */
static uint32_t RangeCheck(const char *s, int32_t val, int32_t bottom, int32_t top) {
    uint32_t ret = CDN_EOK;
    if ((val < bottom) || (val > top)) {
        const char *s_copy = s; /* this ensures parameter is always used in a function */
        if (s_copy != NULL) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "%s out of range, needs to be between %d and %d\n", s_copy, bottom, top);
        }
        ret = CDN_EINVAL;
    }
    return ret;
}

/** Return CDN_EINVAL if uint32_t value is not in range <bottom, top>. */
static uint32_t RangeCheck_unsigned(const char *s, uint32_t val, uint32_t bottom, uint32_t top) {
    uint32_t ret = CDN_EOK;
    if ((val < bottom) || (val > top)) {
        const char *s_copy = s; /* this ensures parameter is always used in a function */
        if (s_copy != NULL) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "%s out of range, needs to be between %d and %d\n", s_copy, bottom, top);
        }
        ret = CDN_EINVAL;
    }
    return ret;
}

/** Extracted part of ComputeOffset */
static void adjust_offset_in_native_420(const DP_DscConfigFull *dscCfg, uint32_t groupsPerLine,
                                        uint32_t grpcnt, int32_t *offset) {

    /* calculations are made in 2 steps to avoid casting composite expression of unsigned type to signed type. */
    if (grpcnt <= groupsPerLine) {
        uint32_t offset_subtract_val = (grpcnt * dscCfg->nslBpgOffset) >> OFFSET_FRACTIONAL_BITS;
        *offset -= (int32_t) offset_subtract_val;
    } else if (grpcnt <= (2U * groupsPerLine)) {
        uint32_t offset_add_val = (((grpcnt - groupsPerLine) * dscCfg->secondLineBpgOfs)
                                   - ((groupsPerLine * dscCfg->nslBpgOffset) >> OFFSET_FRACTIONAL_BITS));
        *offset += (int32_t) offset_add_val;
    } else {
        uint32_t offset_add_val = (((grpcnt - groupsPerLine) * dscCfg->secondLineBpgOfs)
                                   - (((grpcnt - groupsPerLine) * dscCfg->nslBpgOffset) >> OFFSET_FRACTIONAL_BITS));
        *offset += (int32_t) offset_add_val;
    }
}

/*!
 ************************************************************************
 * \brief
 *    ComputeOffset() - Compute offset value at a specific group
 *
 * \param dscCfg
 *    Pointer to DSC configuration structure
 * \param pixelsPerGroup
 *    Number of pixels per group
 * \param groupsPerLine
 *    Number of groups per line
 * \param grpcnt
 *    Group to Compute offset for
 * \return
 *    Offset value for the group grpcnt
 ************************************************************************
 */
static int32_t ComputeOffset(const DP_DscConfigFull *dscCfg, uint32_t pixelsPerGroup, uint32_t groupsPerLine, uint32_t grpcnt)
{
    int32_t offset = 0;
    uint32_t grpcntId = (uint32_t) ceil((float64_t) dscCfg->initialXmitDelay / (float64_t) pixelsPerGroup);
    float64_t bitsPerGroup = (float64_t) pixelsPerGroup * ((float64_t) dscCfg->bitsPerPixel / 16.0);

    if (grpcnt <= grpcntId) {
        offset = (int32_t) ceil((float64_t) grpcnt * bitsPerGroup);
    } else {
        /* calculation is made in 2 steps to avoid casting composite expression of unsigned type to signed type. */
        uint32_t offset_prepare_value = (uint32_t) ceil(
            (float64_t) grpcntId * bitsPerGroup)
                                        - (((grpcnt - grpcntId) * dscCfg->sliceBpgOffset) >> OFFSET_FRACTIONAL_BITS);
        offset = (int32_t) offset_prepare_value;
    }

    if (grpcnt <= groupsPerLine) {
        offset += ((int32_t) grpcnt * (int32_t) dscCfg->firstLineBpgOfs);
    } else {
        /* calculation is made in 2 steps to avoid casting composite expression of unsigned type to signed type. */
        uint32_t offset_add_val = (((groupsPerLine * dscCfg->firstLineBpgOfs)
                                    - (((grpcnt - groupsPerLine) * dscCfg->nflBpgOffset) >> OFFSET_FRACTIONAL_BITS)));
        offset += (int32_t) offset_add_val;
    }
    if (dscCfg->native_420 != 0U) {
        adjust_offset_in_native_420(dscCfg, groupsPerLine, grpcnt, &offset);
    }
    return (offset);
}

/** Compute first and second line bpg offsets - helper function of ComputeRcParameters() */
static void compute_line_bpg_offsets(DP_DscConfigFull *dscCfg) {
    uint32_t firstLineBpgOfs;
    uint32_t bitsPerComponent = dscCfg->bitsPerComponent;
    uint32_t useYuvInput = (dscCfg->convertRgb == 0U) ? 1U : 0U;
    uint32_t uncompressedBpgRate =
        (dscCfg->native_422 != 0U) ?
        ((3U * bitsPerComponent) * 4U) :
        (((3U * bitsPerComponent) + ((useYuvInput != 0U) ? 0U : 2U)) * 3U);

    if (dscCfg->sliceHeight >= 8U) {
        float32_t first_line_bpg_ofs_add_value = 0.09F * (float32_t) min_unsigned_int_32(34U, dscCfg->sliceHeight - 8U);
        firstLineBpgOfs = 12U + (uint32_t) first_line_bpg_ofs_add_value;
    } else {
        firstLineBpgOfs = 2U * (dscCfg->sliceHeight - 1U);
    }

    float32_t line_bpg_ofs_top_limit = (float32_t) uncompressedBpgRate - (3.0F * ((float32_t) dscCfg->bitsPerPixel / 16.0F));
    firstLineBpgOfs = clamp_unsigned_int_32(firstLineBpgOfs, 0U,
                                            (uint32_t) line_bpg_ofs_top_limit);

    uint32_t secondLineBpgOfs = (dscCfg->native_420 != 0U) ? 12U : 0U;
    secondLineBpgOfs = clamp_unsigned_int_32(secondLineBpgOfs, 0U,
                                             (uint32_t) line_bpg_ofs_top_limit);

    /* values are calculated - update the structure */
    dscCfg->firstLineBpgOfs = firstLineBpgOfs;
    (void) RangeCheck_unsigned("firstLineBpgOffset", dscCfg->firstLineBpgOfs, 0, 31U);
    dscCfg->secondLineBpgOfs = secondLineBpgOfs;
    (void) RangeCheck_unsigned("secondLineBpgOffset", dscCfg->secondLineBpgOfs, 0, 31U);
}

/** Compute initial lines parameter - helper function of ComputeRcParameters() */
static void compute_initial_lines(DP_DscConfigFull *dscCfg) {
    float64_t k1;
    const float64_t bitsPerPixel = (float64_t) dscCfg->bitsPerPixel / 16.0;

    if (dscCfg->bitsPerComponent == 8U) {
        k1 = 296.0;
    } else {
        k1 = 320.0;
    }

    if (dscCfg->splitPanel) { /* when both encoders are used in parallel for one video stream */
        dscCfg->initialLines = (uint32_t) ceil(
            (k1 + (float64_t) dscCfg->initialXmitDelay
             + ((((float64_t) dscCfg->chunkSize * 8.0) + 144.0) / bitsPerPixel))
            / (float64_t) dscCfg->sliceWidth);
    } else {
        dscCfg->initialLines = (uint32_t) ceil(
            (k1 + (float64_t) dscCfg->initialXmitDelay
             + ( (ceil( (1.0 - (bitsPerPixel / 48.0))
                        * ((float64_t) dscCfg->chunkSize * 8.0)
                        )
                  + 144.0
                  ) / bitsPerPixel
                 )
            ) / (float64_t) dscCfg->sliceWidth
            );
    }
}

/** Compute initialDecDelay parameter - helper function of ComputeRcParameters() */
static void compute_initial_dec_delay(DP_DscConfigFull *dscCfg, uint32_t pixelsPerGroup,
                                      uint32_t groupsPerLine) {
    uint32_t hrdDelay;
    uint32_t initialDelay = dscCfg->initialXmitDelay;
    int32_t initialFullnessOfs = (int32_t) dscCfg->initialOffset;
    int32_t rbsMin = ((int32_t) dscCfg->rcModelSize - initialFullnessOfs);
    const float64_t bitsPerPixel = (float64_t) dscCfg->bitsPerPixel / 16.0;

    if ((dscCfg->dscVersionMinor == 2U)
        && ((dscCfg->native_420 != 0U) || (dscCfg->native_422 != 0U))) {
        /* OPTIMIZED computation of rbsMin: */
        /* Compute max by sampling offset at points of inflection */
        /* *MODEL NOTE* MN_RBS_MIN */
        int32_t maxOffset = ComputeOffset(dscCfg, pixelsPerGroup, groupsPerLine,
                                          (uint32_t) ceil(((float64_t) initialDelay / (float64_t) pixelsPerGroup))); /* After initial delay */
        maxOffset = max_signed_int_32(maxOffset,
                                      ComputeOffset(dscCfg, pixelsPerGroup, groupsPerLine, groupsPerLine)); /* After first line */
        maxOffset = max_signed_int_32(maxOffset,
                                      ComputeOffset(dscCfg, pixelsPerGroup, groupsPerLine, 2U * groupsPerLine));
        rbsMin += maxOffset;
    } else {
        /* DSC 1.1 method */
        rbsMin += ((int32_t) ceil(
                       ((float64_t) initialDelay * bitsPerPixel)))
                  + ((int32_t) groupsPerLine * (int32_t) dscCfg->firstLineBpgOfs);
    }

    hrdDelay = (uint32_t) (ceil((float64_t) rbsMin / bitsPerPixel));
    dscCfg->initialDecDelay = hrdDelay - initialDelay;
    (void) RangeCheck_unsigned("initialDecDelay", dscCfg->initialDecDelay, 0, 65535U);
}

/** Compute non-first and non-second line bpg offsets - helper function of ComputeRcParameters() */
static void compute_nfl_nsl_bpg_offsets(DP_DscConfigFull *dscCfg, uint32_t *invalid) {
    /* Calculate nflBpgOffset */
    if (dscCfg->sliceHeight > 1U) {
        uint32_t first_line_bpg_ofs_shifted = dscCfg->firstLineBpgOfs << OFFSET_FRACTIONAL_BITS;
        dscCfg->nflBpgOffset = (uint32_t) ceil(
            (float64_t) first_line_bpg_ofs_shifted / ((float64_t) dscCfg->sliceHeight - 1.0));
    } else {
        dscCfg->nflBpgOffset = 0U;
    }
    if (dscCfg->nflBpgOffset > 65535U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "nflBpgOffset is too large for this slice height\n");
        *invalid = 1U; /* status of parent function is being modified, calculation continues though */
    }

    /* Calculate nslBpgOffset */
    if (dscCfg->sliceHeight > 2U) {
        uint32_t second_line_bpg_ofs_shifted = dscCfg->secondLineBpgOfs << OFFSET_FRACTIONAL_BITS;
        dscCfg->nslBpgOffset = (uint32_t) ceil(
            (float64_t) second_line_bpg_ofs_shifted / ((float64_t) dscCfg->sliceHeight - 1.0));
    } else {
        dscCfg->nslBpgOffset = 0U;
    }
    if (dscCfg->nslBpgOffset > 65535U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "nslBpgOffset is too large for this slice height\n");
        *invalid = 1U;
    }
}

/** Compute slice bpg offset - helper function of ComputeRcParameters() */
static void compute_slice_bpg_offset(DP_DscConfigFull *dscCfg, uint32_t numExtraMuxBits,
                                     uint32_t groupsTotal, uint32_t pixelsPerGroup) {
    dscCfg->sliceBpgOffset = (uint32_t) ceil(
        ((float64_t)((uint32_t) 1U << OFFSET_FRACTIONAL_BITS)
         * (((float64_t) dscCfg->rcModelSize - (float64_t) dscCfg->initialOffset) + (float64_t) numExtraMuxBits))
        / (float64_t) groupsTotal);
    (void) RangeCheck_unsigned("sliceBpgOffset", dscCfg->sliceBpgOffset, 0, 65535U);

    if (dscCfg->sliceHeight == 1U) {
        if (dscCfg->firstLineBpgOfs > 0U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "For sliceHeight == 1, the FIRST_LINE_BPG_OFFSET must be 0\n");
        }
    } else if ((((float32_t) pixelsPerGroup * ((float32_t) dscCfg->bitsPerPixel / 16.0F))
                - (((float32_t) dscCfg->sliceBpgOffset + (float32_t) dscCfg->nflBpgOffset)
                   / (float32_t) ((uint32_t) 1U << OFFSET_FRACTIONAL_BITS))) < (1.0F + (5.0F * (float32_t) pixelsPerGroup))) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "The bits/pixel allocation for non-first lines is too low (<5.33bpp).\n"
                                      "Consider decreasing FIRST_LINE_BPG_OFFSET.");
    } else {
        /* do nothing */
    }
}

/** Compute final offset and final scale - helper function of ComputeRcParameters() */
static void compute_final_offset_scale(DP_DscConfigFull *dscCfg, uint32_t *finalScale,
                                       uint32_t numExtraMuxBits) {
    uint32_t finalValue = (dscCfg->rcModelSize
                           - (((dscCfg->initialXmitDelay * dscCfg->bitsPerPixel) + 8U) >> 4U)) + numExtraMuxBits;
    dscCfg->finalOffset = finalValue;
    (void) RangeCheck_unsigned("finalOffset", dscCfg->finalOffset, 0, 65535U);
    if (finalValue >= dscCfg->rcModelSize) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "The finalOffset must be less than the rcModelSize.  "
                                      "Try increasing initialXmitDelay.\n");
    }
    /* update finalScale - variable is then used in the parent function */
    *finalScale = (8U * dscCfg->rcModelSize) / (dscCfg->rcModelSize - finalValue);
    if (*finalScale > 63U) {
        DbgMsg(DBG_GEN_MSG, DBG_WARN, "WARNING: A final scale value > than 63/8 may have undefined"
                                      " behavior on some implementations.  Try increasing initialXmitDelay.\n");
    }
}

/** Compute numExtraMuxBits - helper function of ComputeRcParameters() */
static void compute_num_extra_mux_bits(const DP_DscConfigFull *dscCfg, uint32_t *numExtraMuxBits,
                                       const uint32_t numSsps) {
    uint32_t muxWordSize = (dscCfg->bitsPerComponent <= 10U) ? 48U : 64U;
    uint32_t sliceBits;

    /* numExtraMuxBits is then used in the parent function */
    if (dscCfg->convertRgb != 0U) {
        *numExtraMuxBits = (numSsps * (muxWordSize + (((4U * dscCfg->bitsPerComponent) + 4U) - 2U)));
    } else if (dscCfg->native_422 == 0U) { /* YCbCr */
        *numExtraMuxBits = (numSsps * muxWordSize) + ((4U * dscCfg->bitsPerComponent) + 4U)
                           + ((2U * (4U * dscCfg->bitsPerComponent)) - 2U);
    } else {
        *numExtraMuxBits = (numSsps * muxWordSize) + ((4U * dscCfg->bitsPerComponent) + 4U)
                           + ((3U * (4U * dscCfg->bitsPerComponent)) - 2U);
    }
    sliceBits = 8U * dscCfg->chunkSize * dscCfg->sliceHeight;
    while ((*numExtraMuxBits > 0U) && (((sliceBits - *numExtraMuxBits) % muxWordSize) != 0U)) {
        (*numExtraMuxBits)--;
    }
}

/** Compute number of bytes per chunk - helper function of ComputeRcParameters() */
static void compute_chunk_size(DP_DscConfigFull *dscCfg, const uint32_t slicew) {
    dscCfg->chunkSize = (uint32_t) (ceil(
                                        ((float64_t) slicew * ((float64_t) dscCfg->bitsPerPixel / 16.0)) / 8.0));
    DbgMsg(DBG_GEN_MSG, DBG_CRIT, "%s: chunkSize = %d, bitsPerPixel %f, slicew %d\n", __func__,
           dscCfg->chunkSize, (float32_t) dscCfg->bitsPerPixel / 16.0F, slicew );
    (void) RangeCheck_unsigned("chunkSize", dscCfg->chunkSize, 0, 65535);
}

/** 1st function grouping various calculations for ComputeRcParameters() function */
static void compute_rc_params_1(DP_DscConfigFull *dscCfg, const uint32_t slicew,
                                uint32_t *numExtraMuxBits, const uint32_t numSsps) {
    compute_line_bpg_offsets(dscCfg);
    compute_chunk_size(dscCfg, slicew);
    compute_num_extra_mux_bits(dscCfg, numExtraMuxBits, numSsps);
}

/** 2nd function grouping various calculations for ComputeRcParameters() function */
static void compute_rc_params_2(DP_DscConfigFull *dscCfg, uint32_t numExtraMuxBits, uint32_t *invalid,
                                uint32_t groupsTotal, uint32_t pixelsPerGroup) {
    uint32_t finalScale;

    compute_final_offset_scale(dscCfg, &finalScale, numExtraMuxBits);
    compute_nfl_nsl_bpg_offsets(dscCfg, invalid);
    compute_slice_bpg_offset(dscCfg, numExtraMuxBits, groupsTotal, pixelsPerGroup);

    /* BEGIN scaleIncrementInterval fix */
    if (finalScale > 9U) {
        /* Note: the following calculation assumes that the rcXformOffset crosses 0 at some point. */
        /* If the zero-crossing doesn't occur in a configuration, we recommend to reconfigure the */
        /* rcModelSize and thresholds to be smaller for that configuration. */
        dscCfg->scaleIncrementInterval = ((((uint32_t) 1U << OFFSET_FRACTIONAL_BITS) * dscCfg->finalOffset)
                                          / ((finalScale - 9U)
                                             * (dscCfg->nflBpgOffset + dscCfg->sliceBpgOffset + dscCfg->nslBpgOffset)));
        if (dscCfg->scaleIncrementInterval > 65535U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "scaleIncrementInterval is too large for this slice height.\n");
            *invalid = 1U;
        }
    } else {
        dscCfg->scaleIncrementInterval = 0U;
    }
    /* END scaleIncrementInterval fix */
}

/** Compute RC parameters */
static uint32_t ComputeRcParameters(DP_DscConfigFull *dscCfg, uint32_t pixelsPerGroup, uint32_t numSsps) {
    uint32_t invalid = 0U;

    uint32_t numExtraMuxBits;
    uint32_t slicew = ((dscCfg->native_420 != 0U) || (dscCfg->native_422 != 0U)) ? (dscCfg->sliceWidth >> 1U) : dscCfg->sliceWidth;  /* /2 in 4:2:0 mode */
    uint32_t groupsPerLine = (slicew + (pixelsPerGroup - 1U)) / pixelsPerGroup;
    uint32_t groupsTotal = groupsPerLine * dscCfg->sliceHeight;

    if (groupsPerLine < (dscCfg->initialScaleValue - 8U)) {
        dscCfg->initialScaleValue = groupsPerLine + 8U;
    }

    dscCfg->scaleDecrementInterval = (dscCfg->initialScaleValue > 8U) ? (groupsPerLine / (dscCfg->initialScaleValue - 8U)) : 4095U;
    (void) RangeCheck_unsigned("scaleDecrementInterval", dscCfg->scaleDecrementInterval, 0, 4095U);

    /* call grouping functions that contains further steps - need to be called in order */
    compute_rc_params_1(dscCfg, slicew, &numExtraMuxBits, numSsps);
    compute_rc_params_2(dscCfg, numExtraMuxBits, &invalid, groupsTotal, pixelsPerGroup);

    /* calculate initialDecDelay */
    compute_initial_dec_delay(dscCfg, pixelsPerGroup, groupsPerLine);

    /* calculate initial lines */
    compute_initial_lines(dscCfg);

    return (invalid);
}

/** Part 1 out of 4 of CheckInputParameters()'s routine. */
static void check_input_parameters_1(const DP_DscConfigFull *dscCfg, uint32_t *status) {
    /* Range check performs checking and can optionally print a warning. */
    if (RangeCheck_unsigned("picWidth", dscCfg->picWidth, 0U, 65535U) != CDN_EOK) {
        *status = CDN_EINVAL; /* status of parent function is being modified */
    }
    /* Every parameter is being checked even if one is already invalid */
    if (RangeCheck_unsigned("picHeight", dscCfg->picHeight, 0U, 65535U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("linebufDepth", dscCfg->linebufDepth, 8U, 13U) != CDN_EOK) { /* 13 or 16? */
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("rcTgtOffsetHi", dscCfg->rcTgtOffsetHi, 0U, 15U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("rcTgtOffsetLo", dscCfg->rcTgtOffsetLo, 0U, 15U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("bitsPerPixel (*16)", dscCfg->bitsPerPixel, 96U, 1023U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("rcEdgeFactor", dscCfg->rcEdgeFactor, 0U, 15U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }
}

/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter is not used" */
/* This is a Parasoft bug, all parameters of check_in_params_2_iterative_2 are used. */

/** Part 1 of iterative checks to be called by check_input_parameters_2() */
static void check_in_params_2_iterative_1(const DP_DscConfigFull *dscCfg, uint32_t *status,
                                          const int32_t *prevOffset, const uint32_t *prevMaxQp, const uint32_t i) {

    if (RangeCheck("rangeBpgOffset", dscCfg->rcRangeParameters[i].rangeBpgOffset, -32,
                   31) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (i > 0U) { /* if values were updated by parent function at least 1 time */
        if ((*prevOffset < dscCfg->rcRangeParameters[i].rangeBpgOffset)) {
            DbgMsg(DBG_GEN_MSG, DBG_WARN, "WARNING: The RC_OFFSET values should not increase as the range increases\n");
        }
        if ((*prevMaxQp > dscCfg->rcRangeParameters[i].rangeMaxQp)) {
            DbgMsg(DBG_GEN_MSG, DBG_WARN, "WARNING: The RC_MAX_QP values should not decrease as the range increases\n");
        }
    }
    /* calculate part of the limit here instead of calculating it 4 different places */
    uint32_t range_max_qp_part_of_top_limit = 2U * (dscCfg->bitsPerComponent - 8U);
    if (RangeCheck_unsigned("rangeMaxQp", dscCfg->rcRangeParameters[i].rangeMaxQp, 0,
                            15U + range_max_qp_part_of_top_limit) != CDN_EOK) {
        *status = CDN_EINVAL;
    }
    if ((dscCfg->dscVersionMinor == 1U) && (dscCfg->convertRgb != 1U) /* when using YUV in DSC 1.1 */
        && (dscCfg->rcRangeParameters[i].rangeMaxQp
            > (12U + range_max_qp_part_of_top_limit))) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "ERROR: In DSC 1.1 mode with YCbCr, the max QP for range %d must be less than %d\n",
               i, 12U + range_max_qp_part_of_top_limit);
        *status = CDN_EINVAL;
    }
    if (RangeCheck_unsigned("rangeMinQp", dscCfg->rcRangeParameters[i].rangeMinQp, 0,
                            15U + range_max_qp_part_of_top_limit) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

}
/* parasoft-end-suppress MISRA2012-RULE-2_7-4 "Parameter is not used" */

/* parasoft-begin-suppress MISRA2012-RULE-2_7-4 "Parameter is not used" */
/* This is a Parasoft bug, all parameters of check_in_params_2_iterative_2 are used. */

/** Part 2 of iterative checks to be called by check_input_parameters_2() */
static void check_in_params_2_iterative_2(const DP_DscConfigFull *dscCfg, uint32_t *status,
                                          const uint32_t *prevMinQp, uint32_t *prevThresh, const uint32_t i) {
    if ((i > 0U) && (*prevMinQp > dscCfg->rcRangeParameters[i].rangeMinQp)) {
        DbgMsg(DBG_GEN_MSG, DBG_WARN, "WARNING: The rangeMinQp values should not decrease as the range increases\n");
    }

    /* parent function calls this function up to i=DP_DSC_NUM_BUF_RANGES-1, */
    if (i < (DP_DSC_NUM_BUF_RANGES - 1U)) { /* this excludes the last call */
        if (RangeCheck_unsigned("rcBufThresh", dscCfg->rcBufThresh[i], 0,
                                dscCfg->rcModelSize) != CDN_EOK) {
            *status = CDN_EINVAL;
        }

        if ((dscCfg->rcBufThresh[i] & 0x3fU) != 0U) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "All rcBufThresh must be evenly divisible by 64");
            *status = CDN_EINVAL;
        }
        if ((i > 0U) && (*prevThresh > dscCfg->rcBufThresh[i])) {
            DbgMsg(DBG_GEN_MSG, DBG_CRIT, "WARNING: The rcBufThresh values should not decrease as the range increases\n");
        }
        *prevThresh = dscCfg->rcBufThresh[i];
    }
}

/* parasoft-end-suppress MISRA2012-RULE-2_7-4 "Parameter is not used" */

/** Part 2 of CheckInputParameters()'s routine. */
static void check_input_parameters_2(const DP_DscConfigFull *dscCfg, uint32_t *status) {
    uint32_t prevMinQp = dscCfg->rcRangeParameters[0].rangeMinQp;
    uint32_t prevMaxQp = dscCfg->rcRangeParameters[0].rangeMaxQp;
    int32_t prevOffset = dscCfg->rcRangeParameters[0].rangeBpgOffset;
    uint32_t prevThresh = dscCfg->rcBufThresh[0];

    uint32_t i;
    if (RangeCheck_unsigned("rcQuantIncrLimit1", dscCfg->rcQuantIncrLimit1, 0U, 31U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("rcQuantIncrLimit0", dscCfg->rcQuantIncrLimit0, 0U, 31U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    for (i = 0U; i < DP_DSC_NUM_BUF_RANGES; ++i) {
        /* perform iterative actions */
        check_in_params_2_iterative_1(dscCfg, status, &prevOffset, &prevMaxQp, i);
        check_in_params_2_iterative_2(dscCfg, status, &prevMinQp, &prevThresh, i);

        /* update values */
        prevMinQp = dscCfg->rcRangeParameters[i].rangeMinQp;
        prevMaxQp = dscCfg->rcRangeParameters[i].rangeMaxQp;
        prevOffset = dscCfg->rcRangeParameters[i].rangeBpgOffset;
        /* prevThresh is updated inside check_input_params_2_iterative_2() except for last call. */
    }
}

/** Part 3 out of 4 of CheckInputParameters()'s routine. */
static void check_input_parameters_3(const DP_DscConfigFull *dscCfg, uint32_t *status) {
    /* Range check performs checking and can optionally print a warning. */
    if (RangeCheck_unsigned("rcBufThresh", dscCfg->rcBufThresh[0], 0,
                            dscCfg->rcModelSize) != CDN_EOK) {
        *status = CDN_EINVAL; /* status of parent function is being modified */
    }

    if (RangeCheck_unsigned("rcModelSize", dscCfg->rcModelSize, 0, 65535) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("initialXmitDelay", dscCfg->initialXmitDelay, 0, 1023) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("blockPredEnable", dscCfg->blockPredEnable, 0, 1) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("initialOffset", dscCfg->initialOffset, 0,
                            dscCfg->rcModelSize) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("flatnessMinQp", dscCfg->flatnessMinQp, 0, 31) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("flatnessMaxQp", dscCfg->flatnessMaxQp, 0, 31) != CDN_EOK) {
        *status = CDN_EINVAL;
    }
}

/** Part 4 out of 4 of CheckInputParameters()'s routine. */
static void check_input_parameters_4(const DP_DscConfigFull *dscCfg, uint32_t *status) {
    /* Range check performs checking and can optionally print a warning. */
    if (RangeCheck_unsigned("rcEdgeFactor", dscCfg->rcEdgeFactor, 0U, 15U) != CDN_EOK) {
        *status = CDN_EINVAL; /* status of parent function is being modified */
    }
    if (RangeCheck_unsigned("rcQuantIncrLimit1", dscCfg->rcQuantIncrLimit1, 0U, 31U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("rcQuantIncrLimit0", dscCfg->rcQuantIncrLimit0, 0U, 31U) != CDN_EOK) {
        *status = CDN_EINVAL;
    }
    if (RangeCheck_unsigned("vbrEnable", dscCfg->vbrEnable, 0, 1) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("sliceWidth", dscCfg->sliceWidth, 1, 65535) != CDN_EOK) {
        *status = CDN_EINVAL;
    }

    if (RangeCheck_unsigned("sliceHeight", dscCfg->sliceHeight, 1, 65535) != CDN_EOK) {
        *status = CDN_EINVAL;
    }
}

/** Verify if input parameters are valid. Returns CDN_OK if yes, CDN_EINVAL if no. */
static uint32_t CheckInputParameters(const DP_DscConfigFull *dscCfg)
{
    uint32_t status = CDN_EOK;

    /* Function's routine is split into 4 smaller functions */
    check_input_parameters_1(dscCfg, &status);
    check_input_parameters_2(dscCfg, &status);
    check_input_parameters_3(dscCfg, &status);
    check_input_parameters_4(dscCfg, &status);

    return status;
}

/** Extracted part of DscCopyCfgToFull */
static void dsc_copy_cfg_to_full_1(const DP_DscConfig *dscCfg, DP_DscConfigFull *dscCfgFull) {
    /* Configuration is copied from DP_DscConfig structure to DP_DscConfigFull structure */
    dscCfgFull->picWidth = dscCfg->picWidth;
    dscCfgFull->picHeight = dscCfg->picHeight;
    dscCfgFull->rcTgtOffsetHi = dscCfg->rcTgtOffsetHi;
    dscCfgFull->rcTgtOffsetLo = dscCfg->rcTgtOffsetLo;
    dscCfgFull->rcModelSize = dscCfg->rcModelSize;
    dscCfgFull->flatnessMinQp = dscCfg->flatnessMinQp;
    dscCfgFull->flatnessMaxQp = dscCfg->flatnessMaxQp;
    dscCfgFull->vbrEnable = dscCfg->vbrEnable;
    dscCfgFull->ppsIdentifier = dscCfg->ppsIdentifier;
    dscCfgFull->splitPanel = dscCfg->splitPanel;
    dscCfgFull->hTotal = dscCfg->hTotal;
    /* This function is only to be called by DscCopyCfgToFull() as a part of its routine */
}

/** Extracted part of DscCopyCfgToFull */
static void dsc_copy_cfg_to_full_2(const DP_DscConfig *dscCfg, DP_DscConfigFull *dscCfgFull) {
    /* Configuration is copied from DP_DscConfig structure to DP_DscConfigFull structure */
    dscCfgFull->linebufDepth = dscCfg->linebufDepth;
    dscCfgFull->sliceWidth = dscCfg->sliceWidth;
    dscCfgFull->sliceHeight = dscCfg->sliceHeight;
    dscCfgFull->bitsPerPixel = dscCfg->bitsPerPixel * 16U;
    dscCfgFull->rcEdgeFactor = dscCfg->rcEdgeFactor;
    dscCfgFull->rcQuantIncrLimit1 = dscCfg->rcQuantIncrLimit1;
    dscCfgFull->rcQuantIncrLimit0 = dscCfg->rcQuantIncrLimit0;
    dscCfgFull->initialXmitDelay = dscCfg->initialXmitDelay;
    dscCfgFull->blockPredEnable = dscCfg->blockPredEnable;
    dscCfgFull->initialOffset = dscCfg->initialOffset;
    dscCfgFull->flatnessDetThresh = dscCfg->flatnessDetThresh;
    /* This function is only to be called by DscCopyCfgToFull() as a part of its routine */
}

/** Copy the DSC configuration from DP_DscConfig structure to DP_DscConfigFull structure */
uint32_t DscCopyCfgToFull(const DP_DscConfig *dscCfg, DP_DscConfigFull *dscCfgFull)
{
    uint32_t i;
    uint32_t ret = CDN_EOK;
    dscCfgFull->native_420 = 0U;
    dscCfgFull->native_422 = 0U;
    dscCfgFull->simple_422 = 0U;
    dscCfgFull->convertRgb = 1U;
    dscCfgFull->secondLineBpgOfs = 0U;
    dscCfgFull->secondLineOfsAdj = 0U;
    dscCfgFull->dscVersionMinor = 1U;

    switch (dscCfg->bitsPerComponent) {
    case DP_BITS_PER_COMPONENT_8:
        dscCfgFull->bitsPerComponent = 8U;
        break;
    case DP_BITS_PER_COMPONENT_10:
        dscCfgFull->bitsPerComponent = 10U;
        break;
    /* if the value does not fit any DP_BitsPerComponent enum value */
    default:
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "Wrong BPC value\n");
        ret = CDN_ENOTSUP;
        break;
    }

    /* Copy next parts of the structure */
    dsc_copy_cfg_to_full_1(dscCfg, dscCfgFull);
    dsc_copy_cfg_to_full_2(dscCfg, dscCfgFull);

    /* Iterate over all elements of rcBufThresh (DP_DSC_NUM_BUF_RANGES - 1U) */
    for (i = 0U; i < (DP_DSC_NUM_BUF_RANGES - 1U); i++) {
        dscCfgFull->rcBufThresh[i] = dscCfg->rcBufThresh[i];
    }

    /* Iterate over all elements of rcRangeParameters (DP_DSC_NUM_BUF_RANGES) */
    for (i = 0U; i < DP_DSC_NUM_BUF_RANGES; i++) {
        dscCfgFull->rcRangeParameters[i].rangeBpgOffset =
            dscCfg->rcRangeParameters[i].rangeBpgOffset;
        dscCfgFull->rcRangeParameters[i].rangeMinQp = dscCfg->rcRangeParameters[i].rangeMinQp;
        dscCfgFull->rcRangeParameters[i].rangeMaxQp = dscCfg->rcRangeParameters[i].rangeMaxQp;
    }
    return ret;

}

/** Extracted part of DscCopyFromFull */
static void dsc_copy_from_full_1(DP_DscConfig *dscCfg, const DP_DscConfigFull *dscCfgFull) {
    /* Configuration is copied from DP_DscConfigFull structure to DP_DscConfig structure */
    dscCfg->linebufDepth = dscCfgFull->linebufDepth;
    dscCfg->sliceWidth = dscCfgFull->sliceWidth;
    dscCfg->sliceHeight = dscCfgFull->sliceHeight;
    dscCfg->rcModelSize = dscCfgFull->rcModelSize;
    dscCfg->flatnessMinQp = dscCfgFull->flatnessMinQp;
    dscCfg->flatnessMaxQp = dscCfgFull->flatnessMaxQp;
    dscCfg->vbrEnable = dscCfgFull->vbrEnable;
    dscCfg->ppsIdentifier = dscCfgFull->ppsIdentifier;
    dscCfg->splitPanel = dscCfgFull->splitPanel;
    dscCfg->hTotal = dscCfgFull->hTotal;
    dscCfg->flatnessDetThresh = dscCfgFull->flatnessDetThresh;
    /* This function is only to be called by DscCopyCfgFromFull() as a part of its routine */
}

/** Extracted part of DscCopyFromFull */
static void dsc_copy_from_full_2(DP_DscConfig *dscCfg, const DP_DscConfigFull *dscCfgFull) {
    /* Configuration is copied from DP_DscConfigFull structure to DP_DscConfig structure */
    dscCfg->picWidth = dscCfgFull->picWidth;
    dscCfg->picHeight = dscCfgFull->picHeight;
    dscCfg->rcTgtOffsetHi = dscCfgFull->rcTgtOffsetHi;
    dscCfg->rcTgtOffsetLo = dscCfgFull->rcTgtOffsetLo;
    dscCfg->bitsPerPixel = dscCfgFull->bitsPerPixel / 16U;
    dscCfg->rcEdgeFactor = dscCfgFull->rcEdgeFactor;
    dscCfg->rcQuantIncrLimit1 = dscCfgFull->rcQuantIncrLimit1;
    dscCfg->rcQuantIncrLimit0 = dscCfgFull->rcQuantIncrLimit0;
    dscCfg->initialXmitDelay = dscCfgFull->initialXmitDelay;
    dscCfg->blockPredEnable = dscCfgFull->blockPredEnable;
    dscCfg->initialOffset = dscCfgFull->initialOffset;
    /* This function is only to be called by DscCopyCfgFromFull() as a part of its routine */
}

/** Copy the DSC configuration from DP_DscConfigFull structure to DP_DscConfig structure */
void DscCopyFromFull(DP_DscConfig *dscCfg, const DP_DscConfigFull *dscCfgFull)
{
    uint32_t i;

    switch (dscCfgFull->bitsPerComponent) {
    case 8U:
        dscCfg->bitsPerComponent = DP_BITS_PER_COMPONENT_8;
        break;
    case 10U:
        dscCfg->bitsPerComponent = DP_BITS_PER_COMPONENT_10;
        break;
    default:
        /* default clause and comment */
        break;
    }

    /* Copy next parts of the structure */
    dsc_copy_from_full_1(dscCfg, dscCfgFull);
    dsc_copy_from_full_2(dscCfg, dscCfgFull);

    /* Continue copying */
    for (i = 0U; i < (DP_DSC_NUM_BUF_RANGES - 1U); i++) {
        dscCfg->rcBufThresh[i] = dscCfgFull->rcBufThresh[i];
    }

    for (i = 0U; i < DP_DSC_NUM_BUF_RANGES; i++) {
        dscCfg->rcRangeParameters[i].rangeBpgOffset =
            dscCfgFull->rcRangeParameters[i].rangeBpgOffset;
        dscCfg->rcRangeParameters[i].rangeMinQp = dscCfgFull->rcRangeParameters[i].rangeMinQp;
        dscCfg->rcRangeParameters[i].rangeMaxQp = dscCfgFull->rcRangeParameters[i].rangeMaxQp;
    }
}

uint32_t DscCalcParameters(DP_DscConfigFull *dscCfg)
{
    uint32_t numSsps;
    uint32_t pixelsPerGroup = 3U;
    uint32_t status;

    dscCfg->initialScaleValue = (8U * dscCfg->rcModelSize)
                                / (dscCfg->rcModelSize - dscCfg->initialOffset);

    /* Compute rate buffer size for auto mode */
    if (dscCfg->native_422 != 0U) {
        numSsps = 4U;
    } else {
        numSsps = 3U;
    }

    status = CheckInputParameters(dscCfg);
    /* continue if input parameters are correct */
    if (status == CDN_EOK) {
        status = ComputeRcParameters(dscCfg, pixelsPerGroup, numSsps);
    }

    return status;
}

/* ! Put bits into a buffer in memory */
/*! \param val Value to write
   \param size  Number of bits to write
   \param buf       Pointer to buffer location
   \param bitCount Bit index into buffer (modified) */
static void PutBits(uint32_t val, uint32_t size, uint8_t *buf, uint32_t *bitCount)
{
    int32_t i;
    uint32_t curbit;
    uint32_t bitcntmod8;
    uint32_t bufidx;

    if (size > 32U) {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT, "error: PutBits supports max of 32 bits\n");
    }
    for (i = (int32_t) size - 1; i >= 0; --i) {
        bitcntmod8 = (*bitCount) % 8U;
        bufidx = (*bitCount) >> 3U;
        curbit = (val >> (uint32_t)i) & 1U;
        if (bitcntmod8 == 0U) {
            buf[bufidx] = 0U;       /* Zero current byte */
        }
        if (curbit != 0U) {
            /* parasoft-begin-suppress MISRA2012-RULE-12_2-2 "Shifting operation should be
               enclosed with if checking that RHS operand does not exceed the upper limit" */
            buf[bufidx] |= (1U << (7U - bitcntmod8));
            /* parasoft-end-suppress MISRA2012-RULE-12_2-2 */
        }
        (*bitCount)++;
    }
}

void static dsc_write_pps_1(uint8_t *buf, const DP_DscConfigFull *dscCfg, uint32_t *nbits) {
    uint32_t i;

    /* For every row of this array we have a pair of value and size: arguments of PutBits() */
    uint32_t parameter_array[25][2] = {
        {1U, 4U},     /* dsc major version */
        {dscCfg->dscVersionMinor, 4U},
        {dscCfg->ppsIdentifier, 8U},
        {0U, 8U},     /* reserved */
        {dscCfg->bitsPerComponent, 4U},
        {dscCfg->linebufDepth, 4U},
        {0U, 2U},     /* reserved */
        {dscCfg->blockPredEnable, 1U},
        {dscCfg->convertRgb, 1U},
        {dscCfg->simple_422, 1U},
        {dscCfg->vbrEnable, 1U},
        {dscCfg->bitsPerPixel, 10U},
        {dscCfg->picHeight, 16U},
        {dscCfg->picWidth, 16U},
        {dscCfg->sliceHeight, 16U},
        {dscCfg->sliceWidth, 16U},
        {dscCfg->chunkSize, 16U},
        {0U, 6U},     /* reserved */
        {dscCfg->initialXmitDelay, 10U},
        {dscCfg->initialDecDelay, 16U},
        {0U, 10U},     /* reserved */
        {dscCfg->initialScaleValue, 6U},
        {dscCfg->scaleIncrementInterval, 16U},
        {0U, 4U},     /* reserved */
        {dscCfg->scaleDecrementInterval, 12U},
    };

    for (i = 0U; i < 25U; i++) {
        PutBits(parameter_array[i][0], parameter_array[i][1], buf, nbits);
    }
}

void static dsc_write_pps_2(uint8_t *buf, const DP_DscConfigFull *dscCfg, uint32_t *nbits) {
    uint32_t i;

    uint32_t parameter_array[19][2] = {
        {0U, 11U},     /* reserved */
        {dscCfg->firstLineBpgOfs, 5U},
        {dscCfg->nflBpgOffset, 16U},
        {dscCfg->sliceBpgOffset, 16U},

        {dscCfg->initialOffset, 16U},
        {dscCfg->finalOffset, 16U},
        {0U, 3U},     /* reserved */
        {dscCfg->flatnessMinQp, 5U},
        {0U, 3U},     /* reserved */
        {dscCfg->flatnessMaxQp, 5U},

        /* RC parameter set */
        {dscCfg->rcModelSize, 16U},
        {0U, 4U},     /* reserved */
        {dscCfg->rcEdgeFactor, 4U},
        {0U, 3U},     /* reserved */
        {dscCfg->rcQuantIncrLimit0, 5U},
        {0U, 3U},     /* reserved */
        {dscCfg->rcQuantIncrLimit1, 5U},
        {dscCfg->rcTgtOffsetHi, 4U},
        {dscCfg->rcTgtOffsetLo, 4U}
    };
    for (i = 0U; i < 19U; i++) {
        PutBits(parameter_array[i][0], parameter_array[i][1], buf, nbits);
    }
}

void static dsc_write_pps_3(uint8_t *buf, const DP_DscConfigFull *dscCfg, uint32_t *nbits) {
    uint32_t i;

    for (i = 0U; i < 14U; ++i) {
        PutBits(dscCfg->rcBufThresh[i] >> 6U, 8U, buf, nbits);
    }
    for (i = 0U; i < 15U; ++i) {
        PutBits(dscCfg->rcRangeParameters[i].rangeMinQp, 5U, buf, nbits);
        PutBits(dscCfg->rcRangeParameters[i].rangeMaxQp, 5U, buf, nbits);
        PutBits((uint32_t) dscCfg->rcRangeParameters[i].rangeBpgOffset, 6U, buf, nbits);
    }
    /* / DSC 1.2 bits */
    if (dscCfg->dscVersionMinor == 2U) {
        uint32_t dsc_1_2_parameter_array[7][2] = {
            {0U, 6U},     /* Reserved */
            {dscCfg->native_420, 1U},
            {dscCfg->native_422, 1U},
            {0U, 3U},     /* Reserved (ignored) */
            {dscCfg->secondLineBpgOfs, 5U},
            {dscCfg->nslBpgOffset, 16U},
            {dscCfg->secondLineOfsAdj, 16U}
        };
        for (i = 0U; i < 7U; ++i) {
            PutBits(dsc_1_2_parameter_array[i][0], dsc_1_2_parameter_array[i][1], buf, nbits);
        }
    }
}

/************************************************************************
 * \brief
 *    DscwritePps() - Construct picture parameter set (PPS)
 *
 * \param buf
 *    Pointer to PPS buffer
 * \param dscCfg
 *    Configuration structure
 *
 ************************************************************************
 */
void DscWritePps(uint8_t *buf, const DP_DscConfigFull *dscCfg)
{
    uint32_t nbits = 0U;

    dsc_write_pps_1(buf, dscCfg, &nbits);
    dsc_write_pps_2(buf, dscCfg, &nbits);
    dsc_write_pps_3(buf, dscCfg, &nbits);
}

/* parasoft-end-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */

