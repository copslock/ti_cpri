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
 * dp_dsc.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-41 "Number of blocks of comments per statement in the function" */

#include "dp_if.h"
#include "dp_dsc.h"
#include "dp_priv.h"
#include "dsc_utils.h"
#include "dp_register.h"

#include "mhdp_apb_regs.h"
#include "cps_drv.h"

/* Output Buffer (max pointer address)
 * For 8k max in split mode, 4K per Hard Slice
 * use 3667U value
 *
 * For 4k max in split mode, 2K per Hard Slice
 * use 1885U value
 */

#define DSC_OUTPUT_BUFFER_MAX_ADDRESS        3667U

#define DSC_CFG_READ_WRITE_FUNC_NUM 16U
#define DSC_BPG_OFFSET_SIGN_MASK 0x20U
#define DSC_BGP_OFFSET_MASK 0x3FU

static volatile uint32_t* getEncAddress(const DP_PrivateData* pD, uint8_t encIdx) {

    volatile uint32_t* retVal;
    if (encIdx == 1U) {
        retVal = &pD->regBase->mhdp_apb_regs.ENC1_MAIN_CONF_p;
    } else {
        retVal = &pD->regBase->mhdp_apb_regs.ENC0_MAIN_CONF_p;
    }
    return retVal;
}

static void writeMainConf(const DP_PrivateData* pD, const DP_DscConfigFull* dscCfg){

    uint32_t splitPanel = 0U;
    uint32_t reg;

    if (dscCfg->splitPanel) {
        splitPanel = 1U;
    }

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, REGS_SPLIT_PANEL, 0U, splitPanel)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, REGS_MULTIPLEX_MODE, 0U, splitPanel)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, REGS_MULTIPLEX_SEL_OUT, 0U, 0U)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, REGS_DE_RASTER_ENABLE, 0U, 0U)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, INPUT_MODE, 0U, 1U)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, MULTIPLEX_MODE_EOC_ENABLE, 0U, 1U)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, AUTO_REGS_DB_UPDATE, 0U, 0U);

    CPS_REG_WRITE(&pD->regBase->mhdp_apb_regs.COM_MAIN_CONF_p, reg);
}

/*
 *****************************************************************************************************************
 * Create write functions for each ENC configuration register
 *****************************************************************************************************************
 */
static void writeEncMainConf(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg = 0U;

    if (dscCfg->bitsPerComponent != 8U) {
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, INPUT_BPC, 0U, 1U);
    }

    reg |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, CONVERT_RGB, 0U, dscCfg->convertRgb)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, ENABLE_422, 0U, 0U)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, LINEBUF_DEPTH, 0U, dscCfg->linebufDepth)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, BITS_PER_PIXEL, 0U, dscCfg->bitsPerPixel)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, BLOCK_PRED_ENABLE, 0U, dscCfg->blockPredEnable)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, VIDEO_MODE, 0U, 1U)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, ICH_RST_EOL, 0U, 0U)
           | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, INITIAL_LINES, 0U, dscCfg->initialLines);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncPictureSize(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_PICTURE_SIZE_P, PICTURE_WIDTH, 0U, dscCfg->picWidth)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_PICTURE_SIZE_P, PICTURE_HEIGHT, 0U, dscCfg->picHeight);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncSliceWidth (volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg){

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_SLICE_SIZE_P, SLICE_WIDTH, 0U, dscCfg->sliceWidth)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_SLICE_SIZE_P, SLICE_HEIGHT, 0U, dscCfg->sliceHeight);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncMiscSize(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint32_t sliceWidth = ((uint32_t)dscCfg->sliceWidth + 2U) % 3U;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MISC_SIZE_P, SLICE_LAST_GROUP_SIZE, 0U, sliceWidth)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MISC_SIZE_P, OB_MAX_ADDR, 0U, DSC_OUTPUT_BUFFER_MAX_ADDRESS)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_MISC_SIZE_P, CHUNK_SIZE, 0U, dscCfg->chunkSize);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncHrdDelays(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_HRD_DELAYS_P, INITIAL_XMIT_DELAY, 0U, dscCfg->initialXmitDelay)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_HRD_DELAYS_P, INITIAL_DEC_DELAY, 0U, dscCfg->initialDecDelay);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcScale(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_P, INITIAL_SCALE_VALUE, 0U, dscCfg->initialScaleValue);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcScaleIncDec(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_INC_DEC_P, SCALE_INCREMENT_INTERVAL, 0U, dscCfg->scaleIncrementInterval)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_INC_DEC_P, SCALE_DECREMENT_INTERVAL, 0U, dscCfg->scaleDecrementInterval);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcOffsets(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_1_P, FIRST_LINE_BPG_OFFSET, 0U, dscCfg->firstLineBpgOfs);
    CPS_REG_WRITE(*regAddress, reg);
    (*regAddress)++;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_2_P, NFL_BPG_OFFSET, 0U, dscCfg->nflBpgOffset)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_2_P, SLICE_BPG_OFFSET, 0U, dscCfg->sliceBpgOffset);
    CPS_REG_WRITE(*regAddress, reg);
    (*regAddress)++;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_3_P, INITIAL_OFFSET, 0U, dscCfg->initialOffset)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_3_P, FINAL_OFFSET, 0U, dscCfg->finalOffset);
    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncFlatnessDetection(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_MIN_QP, 0U, dscCfg->flatnessMinQp)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_MAX_QP, 0U, dscCfg->flatnessMaxQp)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_DET_THRESH, 0U, dscCfg->flatnessDetThresh);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncModelSize(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MODEL_SIZE_P, RC_MODEL_SIZE, 0U, dscCfg->rcModelSize);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcConfig(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_EDGE_FACTOR, 0U, dscCfg->rcEdgeFactor)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_QUANT_INCR_LIMIT0, 0U, dscCfg->rcQuantIncrLimit0)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_QUANT_INCR_LIMIT1, 0U, dscCfg->rcQuantIncrLimit1)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_TGT_OFFSET_HI, 0U, dscCfg->rcTgtOffsetHi)
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_TGT_OFFSET_LO, 0U, dscCfg->rcTgtOffsetLo);

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcBufThresh(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    /* each BUF_THRESH buffer have same masks and shifts */
    for (i = 0U; i < 3U; i++) {
        index = 4U * i;
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_0, 0U, (dscCfg->rcBufThresh[index] >> 6U))
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_1, 0U, (dscCfg->rcBufThresh[index + 1U] >> 6U))
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_2, 0U, (dscCfg->rcBufThresh[index + 2U] >> 6U))
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_3, 0U, (dscCfg->rcBufThresh[index + 3U] >> 6U));

        CPS_REG_WRITE(*regAddress, reg);
        (*regAddress)++;
    }

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_3_P, RC_BUF_THRESH_12, 0U, (dscCfg->rcBufThresh[12] >> 6U))
          | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_3_P, RC_BUF_THRESH_13, 0U, (dscCfg->rcBufThresh[13] >> 6U));

    CPS_REG_WRITE(*regAddress, reg);
}

static void writeEncRcMinQp(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    /* each RANGE_MIN buffer have same masks and shifts */
    for (i = 0U; i < 3U; i++) {
        index = i * 5U;
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_0, 0U, dscCfg->rcRangeParameters[index].rangeMinQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_1, 0U, dscCfg->rcRangeParameters[index + 1U].rangeMinQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_2, 0U, dscCfg->rcRangeParameters[index + 2U].rangeMinQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_3, 0U, dscCfg->rcRangeParameters[index + 3U].rangeMinQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_4, 0U, dscCfg->rcRangeParameters[index + 4U].rangeMinQp);

        CPS_REG_WRITE(*regAddress, reg);
        if (2U != i) {
            (*regAddress)++;
        }
    }
}

static void writeEncRcMaxQp(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    /* each RANGE_MAX buffer have same masks and shifts */
    for (i = 0U; i < 3U; i++) {
        index = 5U * i;
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_0, 0U, dscCfg->rcRangeParameters[index].rangeMaxQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_1, 0U, dscCfg->rcRangeParameters[index + 1U].rangeMaxQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_2, 0U, dscCfg->rcRangeParameters[index + 2U].rangeMaxQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_3, 0U, dscCfg->rcRangeParameters[index + 3U].rangeMaxQp)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_4, 0U, dscCfg->rcRangeParameters[index + 4U].rangeMaxQp);

        CPS_REG_WRITE(*regAddress, reg);
        if (2U != i) {
            (*regAddress)++;
        }
    }
}

static void writeEncRcRangeBpgOffsets(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    /* each RANGE_BPG_OFFSET buffer have same masks and shifts */
    for (i = 0U; i < 3U; i++) {
        index = 5U * i;
        reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_0, 0U, dscCfg->rcRangeParameters[index].rangeBpgOffset)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_1, 0U, dscCfg->rcRangeParameters[index + 1U].rangeBpgOffset)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_2, 0U, dscCfg->rcRangeParameters[index + 2U].rangeBpgOffset)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_3, 0U, dscCfg->rcRangeParameters[index + 3U].rangeBpgOffset)
              | CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_4, 0U, dscCfg->rcRangeParameters[index + 4U].rangeBpgOffset);

        CPS_REG_WRITE(*regAddress, reg);
        if (2U != i) {
            (*regAddress)++;
        }
    }
}

static void writeEncDpiCtrlOutDelay(volatile uint32_t** regAddress, const DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint32_t regVal = (dscCfg->initialLines * dscCfg->hTotal);

    reg = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__ENC0_DPI_CTRL_OUT_DELAY_P, DPI_CTRL_OUT_DELAY, 0U, regVal);

    CPS_REG_WRITE(*regAddress, reg);
}
/*
 **************************************************************************************************************************
 *  End of write functions section
 **************************************************************************************************************************
 */

static void writeEncConf(const DP_PrivateData* pD, const DP_DscConfigFull *dscCfg, uint8_t encIdx)
{
    /* Array with pointers to write functions	 */
    void (*writeCfg[DSC_CFG_READ_WRITE_FUNC_NUM]) (volatile uint32_t * *, const DP_DscConfigFull*) = {
        writeEncMainConf,
        writeEncPictureSize,
        writeEncSliceWidth,
        writeEncMiscSize,
        writeEncHrdDelays,
        writeEncRcScale,
        writeEncRcScaleIncDec,
        writeEncRcOffsets,
        writeEncFlatnessDetection,
        writeEncModelSize,
        writeEncRcConfig,
        writeEncRcBufThresh,
        writeEncRcMinQp,
        writeEncRcMaxQp,
        writeEncRcRangeBpgOffsets,
        writeEncDpiCtrlOutDelay
    };

    volatile uint32_t** actualRegAddress;
    volatile uint32_t* firstRegAddress;
    uint8_t i;

    firstRegAddress = getEncAddress(pD, encIdx);
    actualRegAddress = &firstRegAddress;

    for (i = 0U; i < DSC_CFG_READ_WRITE_FUNC_NUM; i++) {
        (*writeCfg[i])(actualRegAddress, dscCfg);
        (*actualRegAddress)++;
    }

}

uint32_t DP_DscWriteConfiguration(const DP_PrivateData* pD, const DP_DscConfigFull *dscCfg, uint8_t streamId)
{
    writeMainConf(pD, dscCfg);
    writeEncConf(pD, dscCfg, streamId);

    /* if split panel mode enabled then write the same configuration to the second
     * encoder */
    if (dscCfg->splitPanel) {
        writeEncConf(pD, dscCfg, 1U);
    }

    return CDN_EOK;
}

static void extendSignedInt(int32_t *value)
{
    uint32_t tempValue = (uint32_t)*value;

    if ((tempValue & DSC_BPG_OFFSET_SIGN_MASK) != 0U) {
        tempValue |= ~DSC_BGP_OFFSET_MASK;
        *value = (int32_t)tempValue;
    }

}

static void readMainConf(const DP_PrivateData* pD, DP_DscConfigFull* dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(&pD->regBase->mhdp_apb_regs.COM_MAIN_CONF_p);

    dscCfg->splitPanel = (0U != CPS_FLD_READ(MHDP__MHDP_APB_REGS__COM_MAIN_CONF_P, REGS_SPLIT_PANEL, reg));
}

/*
 * *********************************************************************************************************
 * Create read functions for each register
 * *********************************************************************************************************
 */

static void readEncMainConf(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint32_t bitsPerComponent;

    reg = CPS_REG_READ(*regAddress);
    bitsPerComponent = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, INPUT_BPC, reg);

    if (bitsPerComponent == 0U) {
        dscCfg->bitsPerComponent = 8U;
    } else {
        dscCfg->bitsPerComponent = 10U;
    }

    dscCfg->convertRgb = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, CONVERT_RGB, reg);
    dscCfg->linebufDepth = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, LINEBUF_DEPTH, reg);
    dscCfg->bitsPerPixel = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, BITS_PER_PIXEL, reg);
    dscCfg->blockPredEnable = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, BLOCK_PRED_ENABLE, reg);
    dscCfg->initialLines = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MAIN_CONF_P, INITIAL_LINES, reg);
}

static void readEncPictureSize(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->picWidth = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_PICTURE_SIZE_P, PICTURE_WIDTH, reg);
    dscCfg->picHeight = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_PICTURE_SIZE_P, PICTURE_HEIGHT, reg);
}

static void readEncSliceWidth (volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg){

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->sliceWidth = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_SLICE_SIZE_P, SLICE_WIDTH, reg);
    dscCfg->sliceHeight = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_SLICE_SIZE_P, SLICE_HEIGHT, reg);
}

static void readEncMiscSize(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->chunkSize = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_MISC_SIZE_P, CHUNK_SIZE, reg);

}

static void readEncHrdDelays(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->initialXmitDelay = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_HRD_DELAYS_P, INITIAL_XMIT_DELAY, reg);
    dscCfg->initialDecDelay = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_HRD_DELAYS_P, INITIAL_DEC_DELAY, reg);
}

static void readEncRcScale(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->initialScaleValue = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_P, INITIAL_SCALE_VALUE, reg);
}

static void readEncRcScaleIncDec(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->scaleIncrementInterval = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_INC_DEC_P, SCALE_INCREMENT_INTERVAL, reg);
    dscCfg->scaleDecrementInterval = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_SCALE_INC_DEC_P, SCALE_DECREMENT_INTERVAL, reg);

}

static void readEncRcOffsets(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);
    dscCfg->firstLineBpgOfs = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_1_P, FIRST_LINE_BPG_OFFSET, reg);
    (*regAddress)++;

    reg = CPS_REG_READ(*regAddress);
    dscCfg->nflBpgOffset = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_2_P, NFL_BPG_OFFSET, reg);
    dscCfg->sliceBpgOffset = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_2_P, SLICE_BPG_OFFSET, reg);
    (*regAddress)++;

    reg = CPS_REG_READ(*regAddress);
    dscCfg->initialOffset = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_3_P, INITIAL_OFFSET, reg);
    dscCfg->finalOffset = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_OFFSETS_3_P, FINAL_OFFSET, reg);
}

static void readEncFlatnessDetection(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->flatnessMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_MIN_QP, reg);
    dscCfg->flatnessMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_MAX_QP, reg);
    dscCfg->flatnessDetThresh = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_FLATNESS_DETECTION_P, FLATNESS_DET_THRESH, reg);
}

static void readEncModelSize(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->rcModelSize = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MODEL_SIZE_P, RC_MODEL_SIZE, reg);
}

static void readEncRcConfig(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    reg = CPS_REG_READ(*regAddress);

    dscCfg->rcEdgeFactor = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_EDGE_FACTOR, reg);
    dscCfg->rcQuantIncrLimit0 = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_QUANT_INCR_LIMIT0, reg);
    dscCfg->rcQuantIncrLimit1 = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_QUANT_INCR_LIMIT1, reg);
    dscCfg->rcTgtOffsetHi = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_TGT_OFFSET_HI, reg);
    dscCfg->rcTgtOffsetLo = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_CONFIG_P, RC_TGT_OFFSET_LO, reg);
}

static void readEncRcBufThresh(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    for (i = 0U; i < 3U; i++) {
        reg = CPS_REG_READ(*regAddress);
        index = 4U * i;
        dscCfg->rcBufThresh[index] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_0, reg) << 6U);
        dscCfg->rcBufThresh[1U + index] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_1, reg) << 6U);
        dscCfg->rcBufThresh[2U + index] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_2, reg) << 6U);
        dscCfg->rcBufThresh[3U + index] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_0_P, RC_BUF_THRESH_3, reg) << 6U);
        (*regAddress)++;
    }

    reg = CPS_REG_READ(*regAddress);

    dscCfg->rcBufThresh[12] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_3_P, RC_BUF_THRESH_12, reg) << 6U);
    dscCfg->rcBufThresh[13] = (CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_BUF_THRESH_3_P, RC_BUF_THRESH_13, reg) << 6U);
}

static void readEncRcMinQp(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    for (i = 0U; i < 3U; i++) {
        reg = CPS_REG_READ(*regAddress);
        index = 5U * i;
        dscCfg->rcRangeParameters[index].rangeMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_0, reg);
        dscCfg->rcRangeParameters[1U + index].rangeMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_1, reg);
        dscCfg->rcRangeParameters[2U + index].rangeMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_2, reg);
        dscCfg->rcRangeParameters[3U + index].rangeMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_3, reg);
        dscCfg->rcRangeParameters[4U + index].rangeMinQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MIN_QP_0_P, RANGE_MIN_QP_4, reg);
        if (2U != i) {
            (*regAddress)++;
        }
    }
}

static void readEncEcMaxQp(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t index;

    for (i = 0U; i < 3U; i++) {
        reg = CPS_REG_READ(*regAddress);
        index = 5U * i;
        dscCfg->rcRangeParameters[index].rangeMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_0, reg);
        dscCfg->rcRangeParameters[1U + index].rangeMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_1, reg);
        dscCfg->rcRangeParameters[2U + index].rangeMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_2, reg);
        dscCfg->rcRangeParameters[3U + index].rangeMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_3, reg);
        dscCfg->rcRangeParameters[4U + index].rangeMaxQp = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_MAX_QP_0_P, RANGE_MAX_QP_4, reg);
        if (2U != i) {
            (*regAddress)++;
        }
    }
}

static void readEncRcRangeBpgOffsets(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;
    uint8_t i;
    uint8_t j;
    uint8_t index;
    uint32_t value[5];

    for (i = 0U; i < 3U; i++) {
        reg = CPS_REG_READ(*regAddress);
        index = 5U * i;

        value[0] = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_0, reg);
        value[1] = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_1, reg);
        value[2] = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_2, reg);
        value[3] = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_3, reg);
        value[4] = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_RC_RANGE_BPG_OFFSETS_0_P, RANGE_BPG_OFFSET_4, reg);

        for (j = 0U; j < 5U; j++) {
            dscCfg->rcRangeParameters[index + j].rangeBpgOffset = (int32_t)value[j];
        }

        if (2U != i) {
            (*regAddress)++;
        }
    }

    for (i = 0U; i < 15U; i++) {
        extendSignedInt(&dscCfg->rcRangeParameters[i].rangeBpgOffset);
    }
}

static void readEncDpiCtrlOutDelay(volatile uint32_t** regAddress, DP_DscConfigFull *dscCfg) {

    uint32_t reg;

    if (dscCfg->initialLines != 0U ) {
        reg = CPS_REG_READ(*regAddress);
        dscCfg->hTotal = CPS_FLD_READ(MHDP__MHDP_APB_REGS__ENC0_DPI_CTRL_OUT_DELAY_P, DPI_CTRL_OUT_DELAY, reg)
                         / dscCfg->initialLines;
    }
}
/*
 * *******************************************************************************************************
 * End READ FUNC section
 * *******************************************************************************************************
 */

/* parasoft-begin-suppress MISRA2012-RULE-8_13_a-4 "Pass 'dscCfg' parameter as const, DRV-3900" */
static void readEncConf(const DP_PrivateData* pD, DP_DscConfigFull *dscCfg, uint8_t encIdx)
{
    /* Array with pointers to read functions	 */
    void (*readCfg[DSC_CFG_READ_WRITE_FUNC_NUM]) (volatile uint32_t * *, DP_DscConfigFull*) = {
        readEncMainConf,
        readEncPictureSize,
        readEncSliceWidth,
        readEncMiscSize,
        readEncHrdDelays,
        readEncRcScale,
        readEncRcScaleIncDec,
        readEncRcOffsets,
        readEncFlatnessDetection,
        readEncModelSize,
        readEncRcConfig,
        readEncRcBufThresh,
        readEncRcMinQp,
        readEncEcMaxQp,
        readEncRcRangeBpgOffsets,
        readEncDpiCtrlOutDelay
    };

    volatile uint32_t** actualRegAddress;
    volatile uint32_t* firstRegAddress;
    uint8_t i;

    firstRegAddress = getEncAddress(pD, encIdx);
    actualRegAddress = &firstRegAddress;

    for (i = 0U; i < DSC_CFG_READ_WRITE_FUNC_NUM; i++) {
        (*readCfg[i])(actualRegAddress, dscCfg);
        (*actualRegAddress)++;
    }
}
/* parasoft-end-suppress MISRA2012-RULE-8_13_a-4 */

uint32_t DP_DscReadConfiguration(const DP_PrivateData* pD, DP_DscConfigFull* dscCfg, uint8_t streamId)
{
    readMainConf(pD, dscCfg);
    readEncConf(pD, dscCfg, streamId);

    return CDN_EOK;
}

uint32_t DP_DscSoftwareReset(DP_PrivateData* pD)
{
    uint32_t retVal;
    uint32_t reg;
    uint32_t softwareReset;
    DP_RegisterTransfer regTransfer = {0};

    regTransfer.addr =  offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].DSC_CTRL_p);
    /* Setting reset bit in any stream resets entire DSC - stream '0' may always be used for it. */
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (CDN_EOK == retVal) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P, DSC_SW_RST, 0U, 1U);
        regTransfer.addr =  offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].DSC_CTRL_p);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal) {
        regTransfer.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].DSC_CTRL_p);
        /* Might need a timeout// */
        do {
            retVal = DP_ReadRegister(pD, &regTransfer);
            if (CDN_EOK != retVal) {
                break;
            }
            reg = (regTransfer.val);
            softwareReset = CPS_FLD_READ(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P, DSC_SW_RST, reg);
        } while (softwareReset != 0U);
    }
    return retVal;
}

uint32_t DP_DscRegistersUpdate(DP_PrivateData* pD, uint8_t streamId)
{
    uint32_t retVal;
    uint32_t reg;
    uint32_t registersUpdate;
    DP_RegisterTransfer regTransfer = {0};

    regTransfer.addr =  offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].DSC_CTRL_p);
    regTransfer.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_vif_ctrl[0]);
    retVal = DP_ReadRegister(pD, &regTransfer);

    if (CDN_EOK == retVal) {
        regTransfer.val |= CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P, DSC_REG_UPDATE, 0U, 1U);
        retVal = DP_WriteRegister(pD, &regTransfer);
    }

    if (CDN_EOK == retVal) {
        /* Might need a timeout// */
        do {
            retVal = DP_ReadRegister(pD, &regTransfer);
            if (CDN_EOK != retVal) {
                break;
            }
            reg = (regTransfer.val);
            registersUpdate = CPS_FLD_READ(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P, DSC_REG_UPDATE, reg);
        } while (registersUpdate != 0U);
    }

    return retVal;
}

uint32_t DP_DscStreamEnable(DP_PrivateData* pD, uint8_t streamId, bool enable)
{
    DP_WriteFieldRequest req = {0};

    req.addr = offsetof(MHDP_ApbRegs, mhdp_apb_regs.mhdp_vif_ctrl[0].DSC_CTRL_p);
    req.addr += streamId * (uint32_t)sizeof(pD->regBase->mhdp_apb_regs.mhdp_vif_ctrl[0]);
    req.startBit = MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P__DSC_EN_SHIFT;
    req.bitCount = MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P__DSC_EN_WIDTH;
    req.val = CPS_FLD_WRITE(MHDP__MHDP_APB_REGS__MHDP_VIF_CTRL__DSC_CTRL_P, DSC_EN, 0U, (enable) ? 1U : 0U);

    return DP_WriteField(pD, &req);
}

/* parasoft-end-suppress METRICS-41 */
