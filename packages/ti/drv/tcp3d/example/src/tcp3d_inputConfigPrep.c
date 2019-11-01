/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
#include <stdio.h>
#include <string.h>

#include "tcp3d_main.h"

/* XDC includes */
#include <xdc/std.h>
#include <xdc/cfg/global.h>

/* CSL includes */
#include <ti/csl/soc.h>
#include <ti/csl/cslr_tcp3d_cfg.h>
#include <ti/csl/cslr_tcp3d_dma.h>
#include <ti/csl/cslr_tcp3d_dma_offsets.h>

#if TEST_PREPARE_ONLY_BETASTATE
/*******************************************************************************
 ******************************************************************************/
Void prepareBlockSizeDepICParams(cbDataDesc *cbPtr)
{
    Int32       frameLenInd;
    UInt8       numMAP;
    
    if ( ( cbPtr->mode == TEST_MODE_SINGLE ) ||
         ( cbPtr->mode == TEST_MODE_SPLIT ) ) 
        numMAP = 1; /* for 3GPP numMAP=1 */
    else
        numMAP = 2; /* LTE or WIMAX */

    /* IC0 - IC1 */
    cbPtr->inCfgParams->blockLen   = (cbPtr->blockSize-1);
    TCP3D_codeBlkSeg (  cbPtr->blockSize,
                        numMAP,
                        &cbPtr->sw0LengthUsed,
                        &cbPtr->inCfgParams->sw0LenSel,
                        &cbPtr->inCfgParams->sw1Len,
                        &cbPtr->inCfgParams->sw2LenSel,
                        &cbPtr->inCfgParams->numsw0);

    /* IC12 - IC14 */
    if ( cbPtr->mode == TEST_MODE_LTE )
    {   /* LTE */
        frameLenInd = LTE_interleaver_index(cbPtr->blockSize);
        cbPtr->inCfgParams->itgParam[0] = (UInt16) ((2*TCP3_LteInterleaverTable[frameLenInd][2]) % TCP3_LteInterleaverTable[frameLenInd][0]);
        cbPtr->inCfgParams->itgParam[1] = TCP3_LteInterleaverTable[frameLenInd][6];
        cbPtr->inCfgParams->itgParam[2] = TCP3_LteInterleaverTable[frameLenInd][3];
        cbPtr->inCfgParams->itgParam[3] = TCP3_LteInterleaverTable[frameLenInd][4];
        cbPtr->inCfgParams->itgParam[4] = TCP3_LteInterleaverTable[frameLenInd][5];
    }
    else if ( cbPtr->mode == TEST_MODE_WIMAX )
    {   /* WIMAX */
        /* NOTE: Finding Index function is not implemented */
        frameLenInd = WIMAX_interleaver_index(cbPtr->blockSize);
        cbPtr->inCfgParams->itgParam[0] = 0;
        cbPtr->inCfgParams->itgParam[1] = TCP3_WimaxInterleaverTable[frameLenInd][0]; 
        cbPtr->inCfgParams->itgParam[2] = TCP3_WimaxInterleaverTable[frameLenInd][1]; 
        cbPtr->inCfgParams->itgParam[3] = TCP3_WimaxInterleaverTable[frameLenInd][2]; 
        cbPtr->inCfgParams->itgParam[4] = TCP3_WimaxInterleaverTable[frameLenInd][3];
    }

    Tcp3d_prepBlockSizeDepConfigRegs (  cbPtr->mode,
                                            &cbPtr->inCfg[0],
                                            cbPtr->inCfgParams->numsw0,
                                            cbPtr->inCfgParams->blockLen,
                                            cbPtr->inCfgParams->sw0LenSel,
                                            cbPtr->inCfgParams->sw2LenSel,
                                            cbPtr->inCfgParams->sw1Len,
                                            &cbPtr->inCfgParams->itgParam[0]);
}

/*******************************************************************************
 ******************************************************************************/
Void prepareBetaStateICParams(cbDataDesc *cbPtr, UInt8 mode)
{
    if ( mode != TEST_MODE_WIMAX )
    {
        Tcp3d_betaStates ( cbPtr->tailBits,
                            //cbPtr->tailMap1,
                            1, /* change to -1 for sign change */
                            COMPUTE_KT(cbPtr->blockSize),
                            cbPtr->inCfgParams->betaMap0,
                            cbPtr->inCfgParams->betaMap1);

        Tcp3d_prepBetaStateConfigRegs ( mode,
                                            &cbPtr->inCfg[0],
                                            &cbPtr->inCfgParams->betaMap0[0],
                                            &cbPtr->inCfgParams->betaMap1[0]);
    }
}

#else
/*******************************************************************************
 ******************************************************************************/
/**
 * This function is used for filling the changing values and calling the 
 * utility prepare functions for constructing all the 15 input config registers 
 */
Void prepareIC(cbDataDesc *cbPtr, UInt32 *tempIC, UInt8 copyFlag)
{
    UInt8       numMAP;
    Int32       frameLenInd;
    Int         i;
    Tcp3d_InCfgParams *inCfgParams = cbPtr->inCfgParams;
    
    if ( ( cbPtr->mode == TEST_MODE_SINGLE ) ||
         ( cbPtr->mode == TEST_MODE_SPLIT ) ) 
        numMAP = 1; /* for 3GPP numMAP=1 */
    else
        numMAP = 2; /* LTE or WIMAX */

    /* IC0 - IC1 */
    inCfgParams->blockLen   = (cbPtr->blockSize-1);
    TCP3D_codeBlkSeg (  cbPtr->blockSize,
                        numMAP,
                        &cbPtr->sw0LengthUsed,
                        &inCfgParams->sw0LenSel,
                        &inCfgParams->sw1Len,
                        &inCfgParams->sw2LenSel,
                        &inCfgParams->numsw0);

    /* IC2 - IC3 */
    /* Fixed values filled once during init */
    /* IC4 - IC7 */
    if ( mode != TEST_MODE_WIMAX )
    {
        /* compute the beta state values from tail bits */
        Tcp3d_betaStates ( cbPtr->tailBits,
                            1, /* change to -1 for sign change */
                            COMPUTE_KT(cbPtr->blockSize),
                            inCfgParams->betaMap0,
                            inCfgParams->betaMap1);
    }
    /* IC8 - IC11 */
    /* Fixed values filled once during init */
    /* IC12 - IC14 */
    if ( cbPtr->mode == TEST_MODE_LTE )
    {   /* LTE */
        frameLenInd = LTE_interleaver_index(cbPtr->blockSize);
        inCfgParams->itgParam[0] = (UInt16) ((2*TCP3_LteInterleaverTable[frameLenInd][2]) % TCP3_LteInterleaverTable[frameLenInd][0]);
        inCfgParams->itgParam[1] = TCP3_LteInterleaverTable[frameLenInd][6];
        inCfgParams->itgParam[2] = TCP3_LteInterleaverTable[frameLenInd][3];
        inCfgParams->itgParam[3] = TCP3_LteInterleaverTable[frameLenInd][4];
        inCfgParams->itgParam[4] = TCP3_LteInterleaverTable[frameLenInd][5];
    }
    else if ( cbPtr->mode == TEST_MODE_WIMAX )
    {   /* WIMAX */
        /* NOTE: Finding Index function is not implemented */
        frameLenInd = WIMAX_interleaver_index(cbPtr->blockSize);
        inCfgParams->itgParam[0] = 0;
        inCfgParams->itgParam[1] = TCP3_WimaxInterleaverTable[frameLenInd][0]; 
        inCfgParams->itgParam[2] = TCP3_WimaxInterleaverTable[frameLenInd][1]; 
        inCfgParams->itgParam[3] = TCP3_WimaxInterleaverTable[frameLenInd][2]; 
        inCfgParams->itgParam[4] = TCP3_WimaxInterleaverTable[frameLenInd][3];
    }

    /* All Input Config Registers are populated */
    Tcp3d_prepConfigRegs (  cbPtr->mode,
                                inCfgParams,
                                cbPtr->inCfg,
                                tempIC,
                                copyFlag);
}
#endif

/*******************************************************************************
 ******************************************************************************/
/**
 * This function is used for filling the fixed values in the inCfgParams
 * structure.
 */
Void fillICParams(Tcp3d_InCfgParams *inCfgParams, cbConfig *cbCfg)
{
    /* copy the config info into inCfgParams */
    /* IC0 - IC1 */
    /* Filled by the TCP3D_codeBlkSeg() function call */
    /* IC2 */
    inCfgParams->intLoadSel     = CSL_TCP3D_DMA_TCP3D_IC_CFG2_P0_INTER_LOAD_SEL_SET;
    inCfgParams->maxStar        = cbCfg->maxst_en;
    inCfgParams->outStsRead     = cbCfg->out_flag_en;
#ifdef _BIG_ENDIAN
    inCfgParams->outOrderSel    = CSL_TCP3D_DMA_TCP3D_IC_CFG2_P0_OUT_ORDER_SEL_SWAP;
#else
    inCfgParams->outOrderSel    = CSL_TCP3D_DMA_TCP3D_IC_CFG2_P0_OUT_ORDER_SEL_NO_SWAP;
#endif
    inCfgParams->extScale       = cbCfg->ext_scale_en;
    inCfgParams->softOutRead    = cbCfg->soft_out_flag_en;
#ifdef _BIG_ENDIAN
    inCfgParams->softOutOrderSel = CSL_TCP3D_DMA_TCP3D_IC_CFG2_P0_SOFT_OUT_ORDER_SEL_8_BIT;
#else
    inCfgParams->softOutOrderSel = CSL_TCP3D_DMA_TCP3D_IC_CFG2_P0_SOFT_OUT_ORDER_SEL_32_BIT;
#endif
    inCfgParams->softOutFrmtSel = cbCfg->soft_out_fmt;
    inCfgParams->minIter        = cbCfg->min_itr;
    inCfgParams->maxIter        = cbCfg->max_itr;
    inCfgParams->snrVal         = cbCfg->snr_val;
    inCfgParams->snrReport      = cbCfg->snr_rep;
    inCfgParams->stopSel        = cbCfg->stop_sel;
    inCfgParams->crcIterSel     = cbCfg->crc_iter_pass;
    inCfgParams->crcPolySel     = cbCfg->crc_sel;
    /* IC3 */
    inCfgParams->maxStarThres   = cbCfg->maxst_thold;
    inCfgParams->maxStarValue   = cbCfg->maxst_value;
    /* IC4 - IC7 */
    /* Filling with defaults - updated based on tail bits */
    memset(inCfgParams->betaMap0,0,8);
    memset(inCfgParams->betaMap0,0,8);
    /* IC8 - IC11 */
    inCfgParams->extrScale[0]   = cbCfg->ext_scale_0;
    inCfgParams->extrScale[1]   = cbCfg->ext_scale_1;
    inCfgParams->extrScale[2]   = cbCfg->ext_scale_2;
    inCfgParams->extrScale[3]   = cbCfg->ext_scale_3;
    inCfgParams->extrScale[4]   = cbCfg->ext_scale_4;
    inCfgParams->extrScale[5]   = cbCfg->ext_scale_5;
    inCfgParams->extrScale[6]   = cbCfg->ext_scale_6;
    inCfgParams->extrScale[7]   = cbCfg->ext_scale_7;
    inCfgParams->extrScale[8]   = cbCfg->ext_scale_8;
    inCfgParams->extrScale[9]   = cbCfg->ext_scale_9;
    inCfgParams->extrScale[10]  = cbCfg->ext_scale_10;
    inCfgParams->extrScale[11]  = cbCfg->ext_scale_11;
    inCfgParams->extrScale[12]  = cbCfg->ext_scale_12;
    inCfgParams->extrScale[13]  = cbCfg->ext_scale_13;
    inCfgParams->extrScale[14]  = cbCfg->ext_scale_14;
    inCfgParams->extrScale[15]  = cbCfg->ext_scale_15;
    /* IC12 - IC14 */
    /* Filling with defaults - updated based on block size */
    inCfgParams->itgParam[0]    = 0;
    inCfgParams->itgParam[1]    = 0;
    inCfgParams->itgParam[2]    = 0;
    inCfgParams->itgParam[3]    = 0;
    inCfgParams->itgParam[4]    = 0;
}

/**
 *  @b Description
 *  @n  
 *      This function compares the prepared beta state values with the 
 *      reference test vector file. 
 *
 *  @param[in]  inCfg
 *       Address of input configuration registers array.
 *
 *  @retval
 *      Not Applicable.
 */
void checkBetaValues (uint32_t inCfg[])
{
#if TEST_BETA_VALUE_CHECK
    Int             idx, i;
    Char            fileName[300];
    FILE            *fid;
    UInt32          inCfgRef[15];
    UInt32          tmp;

    /* Check Beta state values with reference */
    sprintf(fileName, "%s\\reference\\block%d_inp_cfg.dat", testFolder[testCntr], sendBlockCnt);
    if ( !(fid = fopen(fileName,"r")) )
    {
        System_printf("\t Reference Input configuration file open failed : %s\n", fileName);
        System_exit(0);
    }
    for(i=0;i<15;i++)
    {
        fscanf(fid, "%x", &tmp);
        inCfgRef[i] = tmp;
    }
    fclose(fid);

    for (idx = 4; idx < 8; ++idx)
    {
        if ( inCfgRef[idx] != inCfg[idx] )
        {
            System_printf("\t Block Count %d, INCFG mismatch %d\n", sendBlockCnt, idx);
        }
    }
#else
    return;
#endif
}

/* end of file */
