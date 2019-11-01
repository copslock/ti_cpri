/**
 *  \file   tcp3d_reg.c
 *
 *  \brief  TCP3D Driver functions for TCP3D register preparation functions.
 *
 *  Copyright (C) Texas Instruments Incorporated 2009
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/**
 *  Include Files
 */
#include <ti/drv/tcp3d/tcp3d_drv.h>

/**
 *  \brief      TCP3D Driver function for preparing the common control registers
 *              from the input structure parameters using the CSL_FINS macro.
 * 
 *              The outputs could be used to write into the actual TCP3 decoder
 *              memory registers directly or DMAed to bring the TCP3 decoder
 *              state machine to WAIT for inputs state.
 * 
 */
void Tcp3d_prepControlRegs( IN  Tcp3d_CtrlParams    *ctrl,
                            OUT uint32_t              *modeReg,
                            OUT uint32_t              *endReg,
                            OUT uint32_t              *exeRegP0,
                            OUT uint32_t              *exeRegP1)
{
    /* Set MODE register parameters */
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_MODE_SEL, ctrl->mode);
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_IN_MEM_DB_EN, ctrl->doubleBuf);
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_ITG_EN, ctrl->intTable);
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_ERROR_IGNORE_EN, ctrl->errIgnore);
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_AUTO_TRIG_EN, ctrl->autoTrig);
    CSL_FINS (*modeReg, TCP3D_CFG_TCP3_MODE_LTE_CRC_ISEL, ctrl->lteCrcSel);

    /* Set ENDIAN register parameters */
    CSL_FINS (*endReg, TCP3D_CFG_TCP3_END_ENDIAN_INTR, ctrl->endInt);
    CSL_FINS (*endReg, TCP3D_CFG_TCP3_END_ENDIAN_INDATA, ctrl->endInData);

    /* Set EXECUTE P0 register parameters */
    CSL_FINS (*exeRegP0, TCP3D_CFG_TCP3_EXE_P0_EXE_CMD, ctrl->exeP0cmd);

    /* Set EXECUTE P1 register parameters */
    CSL_FINS (*exeRegP1, TCP3D_CFG_TCP3_EXE_P1_EXE_CMD, ctrl->exeP1cmd);

} /* end of Tcp3d_prepControlRegs() */

/**
 *  \brief      This is a utility function provided as part of TCP3D Driver for
 *              preparing a fixed set of input config registers that would be
 *              fixed for a typical configuration and will not vary from 
 *              code block to code block.
 * 
 *              This function is used for preparing IC2, IC3, IC8-IC11 registers
 *              only out of 15 registers (IC0-IC14) using CSL_FINS macro.
 * 
 *              The output outICRegs could be used as template IC registers
 *              array when preparing the input config registers for code blocks.
 * 
 */
void Tcp3d_prepFixedConfigRegs(IN Tcp3d_InCfgParams * const RESTRICT param,
                               OUT uint32_t           * const RESTRICT outICRegs)
{
    uint32_t  *reg;

    /* Prepare input config register 2 */
    reg = &outICRegs[2];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_INTER_LOAD_SEL, param->intLoadSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_MAXST_EN, param->maxStar);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_OUT_FLAG_EN, param->outStsRead);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_OUT_ORDER_SEL, param->outOrderSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_EXT_SCALE_EN, param->extScale);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_SOFT_OUT_FLAG_EN, param->softOutRead);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_SOFT_OUT_ORDER_SEL, param->softOutOrderSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_SOFT_OUT_FMT, param->softOutFrmtSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_MIN_ITR, param->minIter);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_MAX_ITR, param->maxIter);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_SNR_VAL, param->snrVal);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_SNR_REP, param->snrReport);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_STOP_SEL, param->stopSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_CRC_ITER_PASS, param->crcIterSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG2_P0_CRC_SEL, param->crcPolySel);

    /* Prepare input config register 3 */
    reg = &outICRegs[3];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG3_P0_MAXST_THOLD, param->maxStarThres);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG3_P0_MAXST_VALUE, param->maxStarValue);

    /* Prepare input config register 8 */
    reg = &outICRegs[8];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG8_P0_EXT_SCALE_0, param->extrScale[0]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG8_P0_EXT_SCALE_1, param->extrScale[1]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG8_P0_EXT_SCALE_2, param->extrScale[2]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG8_P0_EXT_SCALE_3, param->extrScale[3]);

    /* Prepare input config register 9 */
    reg = &outICRegs[9];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG9_P0_EXT_SCALE_4, param->extrScale[4]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG9_P0_EXT_SCALE_5, param->extrScale[5]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG9_P0_EXT_SCALE_6, param->extrScale[6]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG9_P0_EXT_SCALE_7, param->extrScale[7]);

    /* Prepare input config register 10 */
    reg = &outICRegs[10];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG10_P0_EXT_SCALE_8, param->extrScale[8]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG10_P0_EXT_SCALE_9, param->extrScale[9]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG10_P0_EXT_SCALE_10, param->extrScale[10]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG10_P0_EXT_SCALE_11, param->extrScale[11]);

    /* Prepare input config register 11 */
    reg = &outICRegs[11];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG11_P0_EXT_SCALE_12, param->extrScale[12]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG11_P0_EXT_SCALE_13, param->extrScale[13]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG11_P0_EXT_SCALE_14, param->extrScale[14]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG11_P0_EXT_SCALE_15, param->extrScale[15]);

} /* end of Tcp3d_prepFixedConfigRegs() */

/**
 *  \brief      This is a utility function provided as part of TCP3D Driver for
 *              preparing the input config registers that will be used for
 *              sending to TCP3 decoder IP memory before sending the LLR data.
 * 
 *              This function is used for preparing all the 15 input config
 *              registers (IC0-IC14) using CSL_FINS macro.
 *
 */
void Tcp3d_prepConfigRegs( IN  uint8_t                             mode,
                           IN  Tcp3d_InCfgParams* const RESTRICT param,
                           OUT uint32_t           * const RESTRICT outICRegs,
                           IN  uint32_t           * const RESTRICT tempICRegs,
                           IN  uint8_t                             copyFixedReg)
{
    uint32_t      *reg;

    /* Prepare input config register 0 */
    reg = &outICRegs[0];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG0_P0_NUM_SW0, param->numsw0);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG0_P0_BLK_LN, param->blockLen);

    /* Prepare input config register 1 */
    reg = &outICRegs[1];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW0_LN_SEL, param->sw0LenSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW2_LN_SEL, param->sw2LenSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW1_LN, param->sw1Len);

    /* Prepare input config register - 2,3,8-11 */
    if (copyFixedReg)
    {
        /* Copy fixed registers from template IC */
        outICRegs[2]    = tempICRegs[2];
        outICRegs[3]    = tempICRegs[3];
        outICRegs[8]    = tempICRegs[8];
        outICRegs[9]    = tempICRegs[9];
        outICRegs[10]   = tempICRegs[10];
        outICRegs[11]   = tempICRegs[11];
    }
    else
    {
        /* Prepare fixed registers from inCfgParams */
        Tcp3d_prepFixedConfigRegs ( param, outICRegs );
    }

    /* Prepare input config register 4 */
    reg = &outICRegs[4];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST0_MAP0, param->betaMap0[0]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST1_MAP0, param->betaMap0[1]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST2_MAP0, param->betaMap0[2]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST3_MAP0, param->betaMap0[3]);

    /* Prepare input config register 5 */
    reg = &outICRegs[5];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST4_MAP0, param->betaMap0[4]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST5_MAP0, param->betaMap0[5]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST6_MAP0, param->betaMap0[6]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST7_MAP0, param->betaMap0[7]);

    /* Prepare input config register 6 */
    reg = &outICRegs[6];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST0_MAP1, param->betaMap1[0]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST1_MAP1, param->betaMap1[1]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST2_MAP1, param->betaMap1[2]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST3_MAP1, param->betaMap1[3]);

    /* Prepare input config register 7 */
    reg = &outICRegs[7];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST4_MAP1, param->betaMap1[4]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST5_MAP1, param->betaMap1[5]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST6_MAP1, param->betaMap1[6]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST7_MAP1, param->betaMap1[7]);

    /* LTE or WIMAX */
    if ( ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE ) ||
         ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX ) )
    {
        /* Prepare input config register 12 */
        reg = &outICRegs[12];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG12_P0_ITG_PARAM0, param->itgParam[0]);
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG12_P0_ITG_PARAM1, param->itgParam[1]);

        /* Prepare input config register 13 */
        reg = &outICRegs[13];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG13_P0_ITG_PARAM2, param->itgParam[2]);
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG13_P0_ITG_PARAM3, param->itgParam[3]);

        /* Prepare input config register 14 */
        reg = &outICRegs[14];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG14_P0_ITG_PARAM4, param->itgParam[4]);
    }
    else
    {
        /* ITG Params are not required for 3GPP */
        outICRegs[12] = 0;
        outICRegs[13] = 0;
        outICRegs[14] = 0;
    }

} /* end of Tcp3d_prepConfigRegs() */

/**
 *  \brief      This is a utility function is provided as part of TCP3D Driver
 *              for preparing the specific input config registers which depend
 *              on the block size.
 *
 *              This function can be used for preparing IC0, IC1, IC12-IC14
 *              registers only out of 15 registers (IC0-IC14) using 
 *              CSL_FINS macro.
 *
 */ 
void Tcp3d_prepBlockSizeDepConfigRegs ( IN  uint8_t                   mode,
                                        OUT uint32_t * const RESTRICT outICRegs,
                                        IN  uint8_t                   numsw0,
                                        IN  uint16_t                  blockLen,
                                        IN  uint8_t                   sw0LenSel,
                                        IN  uint8_t                   sw2LenSel,
                                        IN  uint8_t                   sw1Len,
                                        IN  uint16_t * const RESTRICT itgParam)
{
    uint32_t      *reg;

    /* Prepare input config register 0 */
    reg = &outICRegs[0];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG0_P0_NUM_SW0, numsw0);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG0_P0_BLK_LN, blockLen);

    /* Prepare input config register 1 */
    reg = &outICRegs[1];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW0_LN_SEL, sw0LenSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW2_LN_SEL, sw2LenSel);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG1_P0_SW1_LN, sw1Len);

    /* LTE or WIMAX */
    if ( ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_LTE ) ||
         ( mode == CSL_TCP3D_CFG_TCP3_MODE_MODE_SEL_WIMAX ) )
    {
        /* Prepare input config register 12 */
        reg = &outICRegs[12];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG12_P0_ITG_PARAM0, itgParam[0]);
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG12_P0_ITG_PARAM1, itgParam[1]);

        /* Prepare input config register 13 */
        reg = &outICRegs[13];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG13_P0_ITG_PARAM2, itgParam[2]);
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG13_P0_ITG_PARAM3, itgParam[3]);

        /* Prepare input config register 14 */
        reg = &outICRegs[14];
        CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG14_P0_ITG_PARAM4, itgParam[4]);
    }

} /* end of Tcp3d_prepBlockSizeDepConfigRegs() */

/**
 *  \brief      This is a utility function is provided as part of TCP3D Driver
 *              for preparing the beta state value dependent input config
 *              registers only.
 *
 *              This function can be used for preparing IC4-IC7 registers only
 *              out of 15 registers (IC0-IC14) using CSL_FINS macro.
 *
 */ 
void Tcp3d_prepBetaStateConfigRegs( IN  uint8_t                   mode,
                                    OUT uint32_t * const RESTRICT outICRegs,
                                    IN  int8_t   * const RESTRICT betaMap0,
                                    IN  int8_t   * const RESTRICT betaMap1)
{
    uint32_t      *reg;

    /* Prepare input config register 4 */
    reg = &outICRegs[4];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST0_MAP0, betaMap0[0]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST1_MAP0, betaMap0[1]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST2_MAP0, betaMap0[2]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG4_P0_BETA_ST3_MAP0, betaMap0[3]);

    /* Prepare input config register 5 */
    reg = &outICRegs[5];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST4_MAP0, betaMap0[4]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST5_MAP0, betaMap0[5]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST6_MAP0, betaMap0[6]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG5_P0_BETA_ST7_MAP0, betaMap0[7]);

    /* Prepare input config register 6 */
    reg = &outICRegs[6];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST0_MAP1, betaMap1[0]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST1_MAP1, betaMap1[1]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST2_MAP1, betaMap1[2]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG6_P0_BETA_ST3_MAP1, betaMap1[3]);

    /* Prepare input config register 7 */
    reg = &outICRegs[7];
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST4_MAP1, betaMap1[4]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST5_MAP1, betaMap1[5]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST6_MAP1, betaMap1[6]);
    CSL_FINS (*reg, TCP3D_DMA_TCP3D_IC_CFG7_P0_BETA_ST7_MAP1, betaMap1[7]);

} /* end of Tcp3d_prepBetaStateConfigRegs() */

/* end of file */
