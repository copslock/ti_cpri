/** 
 *   @file  bcp_lld.c
 *
 *   @brief  
 *      This file contains the BCP configuration MMR APIs and Helper APIs 
 *      useful in setting up the packet for BCP processing. It also contains
 *      various APIs required to retrieve and setup the BCP error keeping related 
 *      registers.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
 *  ============================================================================
 *
*/
/* BCP LLD types include */
#include <bcp_types.h>

/* BCP LLD include */
#include <ti/drv/bcp/bcp_lld.h>

/** @addtogroup BCP_LLD_FUNCTION
 @{ */

/**
 * ============================================================================
 *  @n@b Bcp_addGlobalHeader
 *
 *  @b  brief
 *  @n  This API translates the Global header settings specified in the 'pGlobalHdrCfg'
 *      input parameter to a format that the BCP would understand.
 *      It appends the Global header thus built to the data buffer passed to this
 *      API in 'pData' and increments the data buffer length by the Global header
 *      length added.
 *
 *  @param[in]    
        pGlobalHdrCfg       Global header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the Global header config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added global header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addGlobalHeader (
    Bcp_GlobalHdrCfg*                   pGlobalHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    if (!pGlobalHdrCfg || !pData || !pLen)
        return -1;

    *(uint32_t *)pData  =   CSL_FMK (GLBL_HDR_WORD0_PKT_TYPE, pGlobalHdrCfg->pkt_type) |
                            CSL_FMK (GLBL_HDR_WORD0_FLUSH, pGlobalHdrCfg->flush) |
                            CSL_FMK (GLBL_HDR_WORD0_DROP, pGlobalHdrCfg->drop) |
                            CSL_FMK (GLBL_HDR_WORD0_HALT, pGlobalHdrCfg->halt) |
                            CSL_FMK (GLBL_HDR_WORD0_RADIO_STANDARD, pGlobalHdrCfg->radio_standard) | 
                            CSL_FMK (GLBL_HDR_WORD0_GHDR_END_PTR, pGlobalHdrCfg->hdr_end_ptr) |
                            CSL_FMK (GLBL_HDR_WORD0_FLOW_ID, pGlobalHdrCfg->flow_id);
    pData   +=  4;
    *pLen   +=  4;
    
    *(uint32_t *)pData  =   CSL_FMK (GLBL_HDR_WORD1_GLBL_TAG, pGlobalHdrCfg->destn_tag);
    pData   +=  4;
    *pLen   +=  4;

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addTMHeader
 *
 *  @b  brief
 *  @n  This API translates the TM header settings specified in the 'pTmHdrCfg'
 *      input parameter to a format that the TM in BCP would understand.
 *      It appends the TM header thus built to the data buffer passed to this
 *      API in 'pData' and increments the data buffer length by the TM header
 *      length added.
 *
 *  @param[in]    
        pTmHdrCfg           TM header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the TM config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient TM configuration provided.
 *  @li                     0   -   Successfully added TM header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */

int32_t Bcp_addTMHeader (
    Bcp_TmHdrCfg*                       pTmHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            padded_ps_size;                
        
    if (!pTmHdrCfg || !pData || !pLen)
        return -1;

    /* User info data must always start on a 128 bit boundary. Pad PS Info such that 
     * info data aligns on 128 bit boundary.
     */
    if (pTmHdrCfg->ps_data_size % 4)
        padded_ps_size  =   pTmHdrCfg->ps_data_size + (4 - (pTmHdrCfg->ps_data_size % 4));
    else    
        padded_ps_size  =   pTmHdrCfg->ps_data_size;                 

    /* Configure the header length and module Ids to the well known values. */
    *(uint32_t *)pData  =   CSL_FMK (TM_HDR_WORD0_LOCAL_HDR_LEN, 
                                     BCP_TM_HDR_LEN + padded_ps_size + (pTmHdrCfg->info_data_size * 4)) |
                            CSL_FMK (TM_HDR_WORD0_MOD_ID, Bcp_ModuleId_TM);
    pData   +=  4;
    *pLen   +=  4;
    
    *(uint32_t *)pData  =   CSL_FMK (TM_HDR_TM_HDR_CTRL_PS_DATA_SIZE, pTmHdrCfg->ps_data_size) |
                            CSL_FMK (TM_HDR_TM_HDR_CTRL_INFO_DATA_SIZE, pTmHdrCfg->info_data_size);
    pData   +=  4;
    *pLen   +=  4;

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addCRCHeader
 *
 *  @b  brief
 *  @n  This API translates the CRC header settings specified in the 'pCrcHdrCfg'
 *      input parameter to a format that the CRC engine in BCP would understand.
 *      It appends the CRC header thus built to the data buffer passed to this
 *      API in 'pData' and increments the data buffer length by the CRC header
 *      length added.
 *
 *  @param[in]    
        pCrcHdrCfg          CRC header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the CRC config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient CRC configuration provided.
 *  @li                     0   -   Successfully added CRC header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addCRCHeader (
    Bcp_CrcHdrCfg*                      pCrcHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    /* Validate Input */        
    if (!pCrcHdrCfg || !pData || !pLen || pCrcHdrCfg->local_hdr_len > 8)
        return -1;

    for (i = 0; i < pCrcHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0: 
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD0_LOCAL_HDR_LEN, pCrcHdrCfg->local_hdr_len) |
                                        CSL_FMK (CRC_HDR_WORD0_MOD_ID, Bcp_ModuleId_CRC) |
                                        CSL_FMK (CRC_HDR_WORD0_FILLER_BITS, pCrcHdrCfg->filler_bits) |
                                        CSL_FMK (CRC_HDR_WORD0_LTE_ORDER, pCrcHdrCfg->bit_order) |
                                        CSL_FMK (CRC_HDR_WORD0_DTX_FORMAT, pCrcHdrCfg->dtx_format);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD1_NUM_SCRAMBLE_SYS, pCrcHdrCfg->num_scramble_sys);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD2_METHOD2_ID, pCrcHdrCfg->method2_id);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD3_VA_BLK_LEN, pCrcHdrCfg->va_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD3_VA_CRC, pCrcHdrCfg->va_crc) |
                                        CSL_FMK (CRC_HDR_WORD3_VA_BLKS, pCrcHdrCfg->va_blks);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD4_VB_BLK_LEN, pCrcHdrCfg->vb_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD4_VB_CRC, pCrcHdrCfg->vb_crc) |
                                        CSL_FMK (CRC_HDR_WORD4_VB_BLKS, pCrcHdrCfg->vb_blks);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD5_VC_BLK_LEN, pCrcHdrCfg->vc_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD5_VC_CRC, pCrcHdrCfg->vc_crc) |
                                        CSL_FMK (CRC_HDR_WORD5_VC_BLKS, pCrcHdrCfg->vc_blks);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD6_D1_BLK_LEN, pCrcHdrCfg->d1_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD6_D1_CRC, pCrcHdrCfg->d1_crc) |
                                        CSL_FMK (CRC_HDR_WORD6_D1_BLKS, pCrcHdrCfg->d1_blks);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD7_D2_BLK_LEN, pCrcHdrCfg->d2_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD7_D2_CRC, pCrcHdrCfg->d2_crc) |
                                        CSL_FMK (CRC_HDR_WORD7_D2_BLKS, pCrcHdrCfg->d2_blks);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (CRC_HDR_WORD8_DC_BLK_LEN, pCrcHdrCfg->dc_blk_len) |
                                        CSL_FMK (CRC_HDR_WORD8_DC_CRC, pCrcHdrCfg->dc_crc) |
                                        CSL_FMK (CRC_HDR_WORD8_DC_BLKS, pCrcHdrCfg->dc_blks);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen       +=  ((pCrcHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addxCdma_RMHeader
 *
 *  @b  brief
 *  @n  This API translates the WCDMA/TD-SCDMA Rate Modulation (RM) header settings 
 *      specified in the 'pRmHdrCfg' input parameter to a format that the 
 *      RM engine in BCP would understand. It appends the RM header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RM header length added.
 *
 *  @param[in]    
        pRmHdrCfg           WCDMA/TD-SCDMA RM header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the xCDMA RM config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient xCDMA RM configuration provided.
 *  @li                     0   -   Successfully added xCDMA RM header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addxCdma_RMHeader (
    Bcp_RmHdr_xCdmaCfg*                 pRmHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i, numWords;

    if (!pRmHdrCfg || !pData || !pLen || pRmHdrCfg->local_hdr_len > 39)
        return -1;            

    for (i = 0; (i < pRmHdrCfg->local_hdr_len + 1) && (i < 21); i++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD0_LOCAL_HDR_LEN, pRmHdrCfg->local_hdr_len) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD0_MOD_ID, Bcp_ModuleId_RM) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD0_INPUT_ORDER, pRmHdrCfg->input_order) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD0_HALF_RATE, pRmHdrCfg->half_rate);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD1_COLLECT_COLS, pRmHdrCfg->collect_cols) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD1_COLLECT_ROWS, pRmHdrCfg->collect_rows);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD2_NUM_SCRAM, pRmHdrCfg->num_scram);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD3_SYS0_LEN, pRmHdrCfg->sys0_len);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD4_SYS0_INIT2, pRmHdrCfg->sys0_init2);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD5_SYS0_MINUS2, pRmHdrCfg->sys0_minus2);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD6_SYS0_PLUS2, pRmHdrCfg->sys0_plus2) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD6_SYS0_ALPHA, pRmHdrCfg->sys0_alpha) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD6_SYS0_BETA, pRmHdrCfg->sys0_beta) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD6_SYS0_PUNCTURE, pRmHdrCfg->sys0_puncture) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD6_SYS0_TURBO, pRmHdrCfg->sys0_turbo);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD7_P0_PAR1_LEN, pRmHdrCfg->p0_par1_len);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD8_P0_PAR1_INIT1, pRmHdrCfg->p0_par1_init1);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD9_P0_PAR1_MINUS1, pRmHdrCfg->p0_par1_minus1);
                pData   +=  4;
                break;
            }
            case 10:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD10_P0_PAR1_PLUS1, pRmHdrCfg->p0_par1_plus1);
                pData   +=  4;
                break;
            }
            case 11:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD11_P0_PAR1_INIT2, pRmHdrCfg->p0_par1_init2);
                pData   +=  4;
                break;
            }
            case 12:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD12_P0_PAR1_MINUS2, pRmHdrCfg->p0_par1_minus2);
                pData   +=  4;
                break;
            }
            case 13:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD13_P0_PAR1_PLUS2, pRmHdrCfg->p0_par1_plus2);
                pData   +=  4;
                break;
            }
            case 14:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD14_P0_PAR2_LEN, pRmHdrCfg->p0_par2_len);
                pData   +=  4;
                break;
            }
            case 15:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD15_P0_PAR2_INIT1, pRmHdrCfg->p0_par2_init1);
                pData   +=  4;
                break;
            }
            case 16:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD16_P0_PAR2_MINUS1, pRmHdrCfg->p0_par2_minus1);
                pData   +=  4;
                break;
            }
            case 17:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD17_P0_PAR2_PLUS1, pRmHdrCfg->p0_par2_plus1);
                pData   +=  4;
                break;
            }
            case 18:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD18_P0_PAR2_INIT2, pRmHdrCfg->p0_par2_init2);
                pData   +=  4;
                break;
            }
            case 19:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD19_P0_PAR2_MINUS2, pRmHdrCfg->p0_par2_minus2);
                pData   +=  4;
                break;
            }
            case 20:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD20_P0_PAR2_PLUS2, pRmHdrCfg->p0_par2_plus2);
                pData   +=  4;
                break;
            }
        }
    }

    i       =   0;
    numWords=   21;  
    while ((numWords < pRmHdrCfg->local_hdr_len + 1) && (i < 5))
    {
        *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD21_SYS1_LEN, pRmHdrCfg->channelCfg[i].sys_len) |
                                CSL_FMK (RM_XCDMA_HDR_WORD21_SYS1_INIT2, pRmHdrCfg->channelCfg[i].sys_init2);
        pData   +=  4;

        *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD22_SYS1_MINUS2, pRmHdrCfg->channelCfg[i].sys_minus2) |
                                CSL_FMK (RM_XCDMA_HDR_WORD22_SYS1_PLUS2, pRmHdrCfg->channelCfg[i].sys_plus2);
        pData   +=  4;

        *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD23_SYS1_ALPHA, pRmHdrCfg->channelCfg[i].sys_aplha) |
                                CSL_FMK (RM_XCDMA_HDR_WORD23_SYS1_BETA, pRmHdrCfg->channelCfg[i].sys_beta) |
                                CSL_FMK (RM_XCDMA_HDR_WORD23_SYS1_PUNCTURE, pRmHdrCfg->channelCfg[i].sys_puncture) |
                                CSL_FMK (RM_XCDMA_HDR_WORD23_SYS1_TURBO, pRmHdrCfg->channelCfg[i].sys_turbo);

        pData   +=  4;

        i       +=  1;
        numWords+=  3;
    }

    for (i = 36; i < pRmHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 36:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD36_P1_PAR1_LEN, pRmHdrCfg->p1_par1_len) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD36_P1_PAR1_INIT2, pRmHdrCfg->p1_par1_init2);
                pData   +=  4;
                break;
            }
            case 37:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD37_P1_PAR1_MINUS2, pRmHdrCfg->p1_par1_minus2) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD37_P1_PAR1_PLUS2, pRmHdrCfg->p1_par1_plus2);
                pData   +=  4;
                break;
            }
            case 38:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD38_P1_PAR2_LEN, pRmHdrCfg->p1_par2_len) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD38_P1_PAR2_INIT2, pRmHdrCfg->p1_par2_init2);
                pData   +=  4;
                break;
            }
            case 39:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_XCDMA_HDR_WORD39_P1_PAR2_MINUS2, pRmHdrCfg->p1_par2_minus2) |
                                        CSL_FMK (RM_XCDMA_HDR_WORD39_P1_PAR2_PLUS2, pRmHdrCfg->p1_par2_plus2);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRmHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addLte_RMHeader
 *
 *  @b  brief
 *  @n  This API translates the LTE Rate Modulation (RM) header settings 
 *      specified in the 'pRmHdrCfg' input parameter to a format that the 
 *      RM engine in BCP would understand. It appends the RM header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RM header length added.
 *
 *  @param[in]    
        pRmHdrCfg           LTE RM header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the LTE RM config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient LTE RM configuration provided.
 *  @li                     0   -   Successfully added LTE RM header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addLte_RMHeader (
    Bcp_RmHdr_LteCfg*                   pRmHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pRmHdrCfg || !pData || !pLen || pRmHdrCfg->local_hdr_len > 6)
        return -1;            

    for (i = 0; i < pRmHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD0_LOCAL_HDR_LEN, pRmHdrCfg->local_hdr_len) |
                                        CSL_FMK (RM_LTE_HDR_WORD0_MOD_ID, Bcp_ModuleId_RM);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD1_CHANNEL_TYPE, pRmHdrCfg->channel_type) |
                                        CSL_FMK (RM_LTE_HDR_WORD1_INPUT_BIT_FORMAT, pRmHdrCfg->input_bit_format) |
                                        CSL_FMK (RM_LTE_HDR_WORD1_OUTPUT_BIT_FORMAT, pRmHdrCfg->output_bit_format) |
                                        CSL_FMK (RM_LTE_HDR_WORD1_NUM_FILLER_BITS_F, pRmHdrCfg->num_filler_bits_f) |
                                        CSL_FMK (RM_LTE_HDR_WORD1_RV_START_COLUMN1, pRmHdrCfg->rv_start_column1) |
                                        CSL_FMK (RM_LTE_HDR_WORD1_RV_START_COLUMN2, pRmHdrCfg->rv_start_column2);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD2_PARAM_NCB1_COLUMN, pRmHdrCfg->param_ncb1_column) |
                                        CSL_FMK (RM_LTE_HDR_WORD2_PARAM_NCB1_ROW, pRmHdrCfg->param_ncb1_row) |
                                        CSL_FMK (RM_LTE_HDR_WORD2_PARAM_NCB2_COLUMN, pRmHdrCfg->param_ncb2_column) |
                                        CSL_FMK (RM_LTE_HDR_WORD2_PARAM_NCB2_ROW, pRmHdrCfg->param_ncb2_row);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD3_NUM_CODE_BLOCKS_C1, pRmHdrCfg->num_code_blocks_c1) |
                                        CSL_FMK (RM_LTE_HDR_WORD3_BLOCK_SIZE_K1, pRmHdrCfg->block_size_k1);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD4_NUM_CODE_BLOCKS_CE1, pRmHdrCfg->num_code_blocks_ce1) |
                                        CSL_FMK (RM_LTE_HDR_WORD4_BLOCK_SIZE_E1, pRmHdrCfg->block_size_e1);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD5_NUM_CODE_BLOCKS_C2, pRmHdrCfg->num_code_blocks_c2) |
                                        CSL_FMK (RM_LTE_HDR_WORD5_BLOCK_SIZE_K2, pRmHdrCfg->block_size_k2);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_LTE_HDR_WORD6_NUM_CODE_BLOCKS_CE2, pRmHdrCfg->num_code_blocks_ce2) |
                                        CSL_FMK (RM_LTE_HDR_WORD6_BLOCK_SIZE_E2, pRmHdrCfg->block_size_e2);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRmHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addWiMax_RMHeader
 *
 *  @b  brief
 *  @n  This API translates the WiMax Rate Modulation (RM) header settings 
 *      specified in the 'pRmHdrCfg' input parameter to a format that the 
 *      RM engine in BCP would understand. It appends the RM header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RM header length added.
 *
 *  @param[in]    
        pRmHdrCfg           WiMax RM header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the WiMax RM config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient WiMax RM configuration provided.
 *  @li                     0   -   Successfully added WiMax RM header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addWiMax_RMHeader (
    Bcp_RmHdr_WiMaxCfg*                 pRmHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pRmHdrCfg || !pData || !pLen || pRmHdrCfg->local_hdr_len > 6)
        return -1;            

    /* Add Words 0-6 of the header to the data buffer */
    for (i = 0; i < pRmHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD0_LOCAL_HDR_LEN, pRmHdrCfg->local_hdr_len) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD0_MOD_ID, Bcp_ModuleId_RM);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD1_NUM_CODE_BLOCKS_C1, pRmHdrCfg->num_code_blocks_c1) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD1_BLOCK_SIZE_K1, pRmHdrCfg->block_size_k1);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD2_PARAM_M1, pRmHdrCfg->param_m1) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD2_PARAM_J1, pRmHdrCfg->param_j1) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD2_BLOCK_SIZE_E1, pRmHdrCfg->block_size_e1);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD3_NUM_CODE_BLOCKS_C2, pRmHdrCfg->num_code_blocks_c2) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD3_BLOCK_SIZE_K2, pRmHdrCfg->block_size_k2);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD4_PARAM_M2, pRmHdrCfg->param_m2) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD4_PARAM_J2, pRmHdrCfg->param_j2) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD4_BLOCK_SIZE_E2, pRmHdrCfg->block_size_e2);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD5_NUM_CODE_BLOCKS_C3, pRmHdrCfg->num_code_blocks_c3) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD5_BLOCK_SIZE_K3, pRmHdrCfg->block_size_k3);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RM_WIMAX_HDR_WORD6_PARAM_M3, pRmHdrCfg->param_m3) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD6_PARAM_J3, pRmHdrCfg->param_j3) |
                                        CSL_FMK (RM_WIMAX_HDR_WORD6_BLOCK_SIZE_E3, pRmHdrCfg->block_size_e3);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRmHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addEncoderHeader
 *
 *  @b  brief
 *  @n  This API translates the Encoder (ENC) submodule header settings 
 *      specified in the 'pEncHdrCfg' input parameter to a format that the 
 *      encoder engine in BCP would understand. It appends the ENC header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the ENC header length added.
 *
 *  @param[in]    
        pEncHdrCfg          Encoder header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the Encoder config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient encoder configuration provided.
 *  @li                     0   -   Successfully added encoder header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addEncoderHeader (
    Bcp_EncHdrCfg*                      pEncHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pEncHdrCfg || !pData || !pLen || pEncHdrCfg->local_hdr_len > 9)
        return -1;            

    for (i = 0; i < pEncHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD0_LOCAL_HDR_LEN, pEncHdrCfg->local_hdr_len) |
                                        CSL_FMK (ENC_HDR_WORD0_MOD_ID, Bcp_ModuleId_ENC) |
                                        CSL_FMK (ENC_HDR_WORD0_TURBO_CONV_SEL, pEncHdrCfg->turbo_conv_sel) |
                                        CSL_FMK (ENC_HDR_WORD0_SCR_CRC_ENABLE, pEncHdrCfg->scr_crc_en) |
                                        CSL_FMK (ENC_HDR_WORD0_CODE_RATE_FLAG, pEncHdrCfg->code_rate_flag);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD1_BLOCK_SIZE_1, pEncHdrCfg->blockCfg[0].block_size) |
                                        CSL_FMK (ENC_HDR_WORD1_NUM_CODE_BLKS, pEncHdrCfg->blockCfg[0].num_code_blks);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD2_INTVPAR0, pEncHdrCfg->blockCfg[0].intvpar0) |
                                        CSL_FMK (ENC_HDR_WORD2_INTVPAR1, pEncHdrCfg->blockCfg[0].intvpar1);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD3_INTVPAR2, pEncHdrCfg->blockCfg[0].intvpar2) |
                                        CSL_FMK (ENC_HDR_WORD3_INTVPAR3, pEncHdrCfg->blockCfg[0].intvpar3);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD4_BLOCK_SIZE_2, pEncHdrCfg->blockCfg[1].block_size) |
                                        CSL_FMK (ENC_HDR_WORD4_NUM_CODE_BLKS, pEncHdrCfg->blockCfg[1].num_code_blks);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD5_INTVPAR0, pEncHdrCfg->blockCfg[1].intvpar0) |
                                        CSL_FMK (ENC_HDR_WORD5_INTVPAR1, pEncHdrCfg->blockCfg[1].intvpar1);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD6_INTVPAR2, pEncHdrCfg->blockCfg[1].intvpar2) |
                                        CSL_FMK (ENC_HDR_WORD6_INTVPAR3, pEncHdrCfg->blockCfg[1].intvpar3);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD7_BLOCK_SIZE_3, pEncHdrCfg->blockCfg[2].block_size) |
                                        CSL_FMK (ENC_HDR_WORD7_NUM_CODE_BLKS, pEncHdrCfg->blockCfg[2].num_code_blks);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD8_INTVPAR0, pEncHdrCfg->blockCfg[2].intvpar0) |
                                        CSL_FMK (ENC_HDR_WORD8_INTVPAR1, pEncHdrCfg->blockCfg[2].intvpar1);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (ENC_HDR_WORD9_INTVPAR2, pEncHdrCfg->blockCfg[2].intvpar2) |
                                        CSL_FMK (ENC_HDR_WORD9_INTVPAR3, pEncHdrCfg->blockCfg[2].intvpar3);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pEncHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addModulatorHeader
 *
 *  @b  brief
 *  @n  This API translates the Modulator (MOD) submodule header settings 
 *      specified in the 'pModHdrCfg' input parameter to a format that the 
 *      modulator engine in BCP would understand. It appends the MOD header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the MOD header length added.
 *
 *  @param[in]    
        pModHdrCfg          Modulator header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the Modulator config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient modulator configuration provided.
 *  @li                     0   -   Successfully added modulator header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addModulatorHeader (
    Bcp_ModHdrCfg*                      pModHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pModHdrCfg || !pData || !pLen || pModHdrCfg->local_hdr_len > 4)
        return -1;            

    for (i = 0; i < pModHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (MOD_HDR_WORD0_LOCAL_HDR_LEN, pModHdrCfg->local_hdr_len) |
                                        CSL_FMK (MOD_HDR_WORD0_MOD_ID, Bcp_ModuleId_MOD) |
                                        CSL_FMK (MOD_HDR_WORD0_UVA_VAL, pModHdrCfg->uva_val);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (MOD_HDR_WORD1_SH_MOD_SEL, pModHdrCfg->sh_mod_sel) |
                                        CSL_FMK (MOD_HDR_WORD1_SPLIT_MODE_EN, pModHdrCfg->split_mode_en) |
                                        CSL_FMK (MOD_HDR_WORD1_SCR_EN, pModHdrCfg->scr_en) |
                                        CSL_FMK (MOD_HDR_WORD1_MOD_TYPE_SEL, pModHdrCfg->mod_type_sel) |
                                        CSL_FMK (MOD_HDR_WORD1_CMUX_LN, pModHdrCfg->cmux_ln) |
                                        CSL_FMK (MOD_HDR_WORD1_Q_FORMAT, pModHdrCfg->q_format) |
                                        CSL_FMK (MOD_HDR_WORD1_B_TABLE_INDEX, pModHdrCfg->b_table_index) |
                                        CSL_FMK (MOD_HDR_WORD1_JACK_BIT, pModHdrCfg->jack_bit);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (MOD_HDR_WORD2_CINIT_P2, pModHdrCfg->cinit_p2);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (MOD_HDR_WORD3_RMUX_LN, pModHdrCfg->rmux_ln) |
                                        CSL_FMK (MOD_HDR_WORD3_CQI_LN, pModHdrCfg->cqi_ln);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (MOD_HDR_WORD4_RI_LN, pModHdrCfg->ri_ln) |
                                        CSL_FMK (MOD_HDR_WORD4_ACK_LN, pModHdrCfg->ack_ln);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pModHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addInterleaverHeader
 *
 *  @b  brief
 *  @n  This API translates the Interleaver (INT) submodule header settings 
 *      specified in the 'pIntHdrCfg' input parameter to a format that the 
 *      interleaver engine in BCP would understand. It appends the INT header thus built 
 *      to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the INT header length added.
 *
 *  @param[in]    
        pIntHdrCfg          Interleaver header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the Interleaver config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient interleaver configuration provided.
 *  @li                     0   -   Successfully added interleaver header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addInterleaverHeader (
    Bcp_IntHdrCfg*                      pIntHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;        

    if (!pIntHdrCfg || !pData || !pLen || pIntHdrCfg->local_hdr_len > 8)
        return -1;            

    for (i = 0; i < pIntHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD0_LOCAL_HDR_LEN, pIntHdrCfg->local_hdr_len) |
                                        CSL_FMK (INT_HDR_WORD0_MOD_ID, Bcp_ModuleId_INT);
                                        //CSL_FMK (INT_HDR_WORD0_PPB_CONFIG_HEADER, pIntHdrCfg->ppb_config_header);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD1_NUM_CONSTELLATION, pIntHdrCfg->num_constellation) |
                                        CSL_FMK (INT_HDR_WORD1_FLAG_IN_ORDER, pIntHdrCfg->flag_in_order) |
                                        CSL_FMK (INT_HDR_WORD1_FLAG_HALF_RATE, pIntHdrCfg->flag_half_rate) |
                                        CSL_FMK (INT_HDR_WORD1_NUM_DATA_FORMAT_OUT, pIntHdrCfg->num_data_format_out) |
                                        CSL_FMK (INT_HDR_WORD1_NUM_FRAME_COUNT, pIntHdrCfg->num_frame_count) |
                                        CSL_FMK (INT_HDR_WORD1_NUM_INT_WAYS, pIntHdrCfg->num_int_ways);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD2_NUM_ADD_DTX, pIntHdrCfg->num_add_dtx);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD3_NUM_R2_LENGTH_0, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD3_NUM_DUMMY_0, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD3_NUM_DATA_FORMAT_IN_0, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD4_NUM_R2_LENGTH_1, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD4_NUM_DUMMY_1, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD4_NUM_DATA_FORMAT_IN_1, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD5_NUM_R2_LENGTH_2, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD5_NUM_DUMMY_2, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD5_NUM_DATA_FORMAT_IN_2, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD6_NUM_R2_LENGTH_3, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD6_NUM_DUMMY_3, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD6_NUM_DATA_FORMAT_IN_3, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD7_NUM_R2_LENGTH_4, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD7_NUM_DUMMY_4, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD7_NUM_DATA_FORMAT_IN_4, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (INT_HDR_WORD8_NUM_R2_LENGTH_5, pIntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (INT_HDR_WORD8_NUM_DUMMY_5, pIntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (INT_HDR_WORD8_NUM_DATA_FORMAT_IN_5, pIntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pIntHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addCorrelationHeader
 *
 *  @b  brief
 *  @n  This API translates the Correlation (COR) engine header settings 
 *      specified in the 'pCorHdrCfg' input parameter to a format that the 
 *      correlation engine in BCP would understand. It appends the COR header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the COR header length added.
 *
 *  @param[in]    
        pCorHdrCfg          Correlation header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the Correlation config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient correlation configuration provided.
 *  @li                     0   -   Successfully added correlation header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addCorrelationHeader (
    Bcp_CorHdrCfg*                      pCorHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pCorHdrCfg || !pData || !pLen || pCorHdrCfg->local_hdr_len > 7)
        return -1;            

    for (i = 0; i < pCorHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD0_LOCAL_HDR_LEN, pCorHdrCfg->local_hdr_len) |
                                        CSL_FMK (COR_HDR_WORD0_MOD_ID, Bcp_ModuleId_COR) |
                                        CSL_FMK (COR_HDR_WORD0_PUCCH_DESPREAD_SEL, pCorHdrCfg->pucch_despread_sel) |
                                        CSL_FMK (COR_HDR_WORD0_DESPREAD_FLAG_CPLX, pCorHdrCfg->despread_flag_cplx);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD1_PUCCH_A, pCorHdrCfg->pucch_a) |
                                        CSL_FMK (COR_HDR_WORD1_PUCCH_SCR_SEQ, pCorHdrCfg->pucch_scr_seq) |
                                        CSL_FMK (COR_HDR_WORD1_PUCCH_NR, pCorHdrCfg->pucch_nr) |
                                        CSL_FMK (COR_HDR_WORD1_PUCCH_MODE, pCorHdrCfg->pucch_mode);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD2_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD2_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD3_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD3_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD4_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD4_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD5_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD5_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD6_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD6_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (COR_HDR_WORD7_DESPREAD_LENGTH, pCorHdrCfg->block_params[i-2].despreading_length) |
                                        CSL_FMK (COR_HDR_WORD7_SF_RATIO, pCorHdrCfg->block_params[i-2].sf_ratio);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pCorHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addLTE_SSLHeader
 *
 *  @b  brief
 *  @n  This API translates the LTE Soft Slicer (SSL) engine header settings 
 *      specified in the 'pSslHdrCfg' input parameter to a format that the 
 *      SSL engine in BCP would understand. It appends the SSL header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the SSL header length added.
 *
 *  @param[in]    
        pSslHdrCfg          LTE SSL header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the SSL config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added SSL header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addLTE_SSLHeader (
    Bcp_SslHdr_LteCfg*                  pSslHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pSslHdrCfg || !pData || !pLen || pSslHdrCfg->local_hdr_len > 27)
        return -1;            

    for (i = 0; i < pSslHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_LOCAL_HDR_LEN, pSslHdrCfg->local_hdr_len) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_MOD_ID, Bcp_ModuleId_SSL) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_UVA, pSslHdrCfg->uva);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_FDD_TDD_SELECT, pSslHdrCfg->modeSelCfg.fdd_tdd_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_SPLIT_MODE_EN, pSslHdrCfg->modeSelCfg.split_mode_en) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_JACK_BIT, pSslHdrCfg->modeSelCfg.jack_bit) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_LTE_DESCRAMBLER_EN, pSslHdrCfg->modeSelCfg.lte_descrambler_en) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_MOD_TYPE_SEL, pSslHdrCfg->modeSelCfg.mod_type_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_CMUX_LN, pSslHdrCfg->modeSelCfg.cmux_ln) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_Q_FORMAT, pSslHdrCfg->modeSelCfg.q_format) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_B_MATRIX_SEL, pSslHdrCfg->modeSelCfg.b_matrix_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_TTI_2MS_10MS_SEL, pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_WCDMA_NUM_SLOTS, pSslHdrCfg->modeSelCfg.wcdma_num_slots) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_WCDMA_NUM_PHY_CH, pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_RMUX_LN_INDEX, pSslHdrCfg->modeSelCfg.rmux_ln_index);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD2_RI_LN, pSslHdrCfg->ri_ln) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD2_ACK_LN, pSslHdrCfg->ack_ln);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD3_CINIT_P2, pSslHdrCfg->cinit_p2);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD4_SCALE_C0_0, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD5_SCALE_C0_1, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD6_SCALE_C0_2, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD7_SCALE_C0_3, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD8_SCALE_C0_4, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD9_SCALE_C0_5, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 10:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD10_SCALE_C0_6, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 11:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD11_SCALE_C0_7, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 12:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD12_SCALE_C0_8, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 13:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD13_SCALE_C0_9, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 14:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD14_SCALE_C0_10, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 15:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD15_SCALE_C0_11, pSslHdrCfg->scale_c0[i-4]);
                pData   +=  4;
                break;
            }
            case 16:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD16_SCALE_C1_0, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 17:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD17_SCALE_C1_1, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 18:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD18_SCALE_C1_2, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 19:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD19_SCALE_C1_3, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 20:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD20_SCALE_C1_4, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 21:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD21_SCALE_C1_5, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 22:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD22_SCALE_C1_6, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 23:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD23_SCALE_C1_7, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 24:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD24_SCALE_C1_8, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 25:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD25_SCALE_C1_9, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 26:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD26_SCALE_C1_10, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
            case 27:
            {
                /* If LTE Split mode enabled, add the column 1,n scaling factor values too to
                 * the header, otherwise pad them to zeros
                 */
                if (pSslHdrCfg->modeSelCfg.split_mode_en)
                {
                    *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD27_SCALE_C1_11, pSslHdrCfg->scale_c1[i-16]);
                    pData   +=  4;
                }
                break;
            }
        }
    }

    *pLen   +=  ((pSslHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;
}


/**
 * ============================================================================
 *  @n@b Bcp_addWCDMAFDD_SSLHeader
 *
 *  @b  brief
 *  @n  This API translates the WCDMA FDD Soft Slicer (SSL) engine header settings 
 *      specified in the 'pSslHdrCfg' input parameter to a format that the 
 *      SSL engine in BCP would understand. It appends the SSL header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the SSL header length added.
 *
 *  @param[in]    
        pSslHdrCfg          WCDMA FDD SSL header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the SSL config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added SSL header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addWCDMAFDD_SSLHeader (
    Bcp_SslHdr_WcdmaFddCfg*             pSslHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            numHdrWords;

    if (!pSslHdrCfg || !pData || !pLen || pSslHdrCfg->local_hdr_len > 19)
        return -1;            

    for (numHdrWords = 0; numHdrWords < pSslHdrCfg->local_hdr_len + 1; numHdrWords ++)
    {
        switch (numHdrWords)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD0_LOCAL_HDR_LEN, pSslHdrCfg->local_hdr_len) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD0_MOD_ID, Bcp_ModuleId_SSL) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD0_WCDMA_SYB_SEG, pSslHdrCfg->wcdma_symb_seq);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_FDD_TDD_SELECT, pSslHdrCfg->modeSelCfg.fdd_tdd_sel) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_SPLIT_MODE_EN, pSslHdrCfg->modeSelCfg.split_mode_en) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_JACK_BIT, pSslHdrCfg->modeSelCfg.jack_bit) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_LTE_DESCRAMBLER_EN, pSslHdrCfg->modeSelCfg.lte_descrambler_en) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_MOD_TYPE_SEL, pSslHdrCfg->modeSelCfg.mod_type_sel) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_CMUX_LN, pSslHdrCfg->modeSelCfg.cmux_ln) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_Q_FORMAT, pSslHdrCfg->modeSelCfg.q_format) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_B_MATRIX_SEL, pSslHdrCfg->modeSelCfg.b_matrix_sel) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_TTI_2MS_10MS_SEL, pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_WCDMA_NUM_SLOTS, pSslHdrCfg->modeSelCfg.wcdma_num_slots) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_WCDMA_NUM_PHY_CH, pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch) |
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD1_RMUX_LN_INDEX, pSslHdrCfg->modeSelCfg.rmux_ln_index);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD2_UVA_0, pSslHdrCfg->uva [0]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD2_UVA_1, pSslHdrCfg->uva [1]);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD3_UVA_2, pSslHdrCfg->uva [2]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD3_UVA_3, pSslHdrCfg->uva [3]);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD4_UVA_4, pSslHdrCfg->uva [4]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD4_UVA_5, pSslHdrCfg->uva [5]);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD5_SCALE_C0_0, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD6_SCALE_C0_1, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD7_SCALE_C0_2, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD8_SCALE_C0_3, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD9_SCALE_C0_4, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 10:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD10_SCALE_C0_5, pSslHdrCfg->scale_c0[numHdrWords - 5]);
                pData   +=  4;
                break;
            }
            case 11:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD11_UVA_6, pSslHdrCfg->uva [6]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD11_UVA_7, pSslHdrCfg->uva [7]);
                pData   +=  4;
                break;
            }
            case 12:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD12_UVA_8, pSslHdrCfg->uva [8]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD12_UVA_9, pSslHdrCfg->uva [9]);
                pData   +=  4;
                break;
            }
            case 13:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD13_UVA_10, pSslHdrCfg->uva [10]) | 
                                        CSL_FMK (SSL_WCDMA_FDD_HDR_WORD13_UVA_11, pSslHdrCfg->uva [11]);
                pData   +=  4;
                break;
            }
            case 14:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD14_SCALE_C0_6, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
            case 15:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD15_SCALE_C0_7, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
            case 16:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD16_SCALE_C0_8, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
            case 17:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD17_SCALE_C0_9, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
            case 18:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD18_SCALE_C0_10, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
            case 19:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_FDD_HDR_WORD19_SCALE_C0_11, pSslHdrCfg->scale_c0[numHdrWords - 14 + 6]);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pSslHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addWCDMATDSCDMA_SSLHeader
 *
 *  @b  brief
 *  @n  This API translates the WCDMA TD-SCDMA Soft Slicer (SSL) engine header settings 
 *      specified in the 'pSslHdrCfg' input parameter to a format that the 
 *      SSL engine in BCP would understand. It appends the SSL header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the SSL header length added.
 *
 *  @param[in]    
        pSslHdrCfg          WCDMA TD-SCDMA SSL header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the SSL config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added SSL header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addWCDMATDSCDMA_SSLHeader (
    Bcp_SslHdr_WcdmaTdScdmaCfg*         pSslHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            numHdrWords;

    if (!pSslHdrCfg || !pData || !pLen || pSslHdrCfg->local_hdr_len > 13)
        return -1;            

    for (numHdrWords = 0; numHdrWords < pSslHdrCfg->local_hdr_len + 1; numHdrWords ++)
    {
        switch (numHdrWords)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD0_LOCAL_HDR_LEN, pSslHdrCfg->local_hdr_len) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD0_MOD_ID, Bcp_ModuleId_SSL);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_FDD_TDD_SELECT, pSslHdrCfg->modeSelCfg.fdd_tdd_sel) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_SPLIT_MODE_EN, pSslHdrCfg->modeSelCfg.split_mode_en) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_JACK_BIT, pSslHdrCfg->modeSelCfg.jack_bit) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_LTE_DESCRAMBLER_EN, pSslHdrCfg->modeSelCfg.lte_descrambler_en) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_MOD_TYPE_SEL, pSslHdrCfg->modeSelCfg.mod_type_sel) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_WCDMA_OUTPUT_PACK, pSslHdrCfg->modeSelCfg.tdd_output_pack) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_CMUX_LN, pSslHdrCfg->modeSelCfg.cmux_ln) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_Q_FORMAT, pSslHdrCfg->modeSelCfg.q_format) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_B_MATRIX_SEL, pSslHdrCfg->modeSelCfg.b_matrix_sel) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_TTI_2MS_10MS_SEL, pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_WCDMA_NUM_SLOTS, pSslHdrCfg->modeSelCfg.wcdma_num_slots) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_WCDMA_NUM_PHY_CH, pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD1_RMUX_LN_INDEX, pSslHdrCfg->modeSelCfg.rmux_ln_index);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD2_WCDMA_SIZE_SLOT_0, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD2_UVA_0, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD4_WCDMA_SIZE_SLOT_1, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD4_UVA_1, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD6_WCDMA_SIZE_SLOT_2, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD6_UVA_2, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD8_WCDMA_SIZE_SLOT_3, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD8_UVA_3, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 10:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD10_WCDMA_SIZE_SLOT_4, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD10_UVA_4, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 12:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD12_WCDMA_SIZE_SLOT_5, pSslHdrCfg->wcdma_size_slot[(numHdrWords - 2)/2]) |
                                        CSL_FMK (SSL_WCDMA_TDD_HDR_WORD12_UVA_5, pSslHdrCfg->uva[(numHdrWords - 2)/2]);

                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD3_SCALE_C0_0, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD5_SCALE_C0_1, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD7_SCALE_C0_2, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD9_SCALE_C0_3, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
            case 11:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD11_SCALE_C0_4, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
            case 13:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_WCDMA_TDD_HDR_WORD13_SCALE_C0_5, pSslHdrCfg->scale_c0[(numHdrWords - 3)/2]);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pSslHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addWiMax_SSLHeader
 *
 *  @b  brief
 *  @n  This API translates the WiMAX TD-SCDMA Soft Slicer (SSL) engine header settings 
 *      specified in the 'pSslHdrCfg' input parameter to a format that the 
 *      SSL engine in BCP would understand. It appends the SSL header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the SSL header length added.
 *
 *  @param[in]    
        pSslHdrCfg          WiMAX TD-SCDMA SSL header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the SSL config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added SSL header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addWiMax_SSLHeader (
    Bcp_SslHdr_WiMaxCfg*                pSslHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            numHdrWords;

    if (!pSslHdrCfg || !pData || !pLen || pSslHdrCfg->local_hdr_len > 27)
        return -1;            

    for (numHdrWords = 0; numHdrWords < pSslHdrCfg->local_hdr_len + 1; numHdrWords ++)
    {
        switch (numHdrWords)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_LOCAL_HDR_LEN, pSslHdrCfg->local_hdr_len) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_MOD_ID, Bcp_ModuleId_SSL) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD0_UVA, pSslHdrCfg->uva);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_FDD_TDD_SELECT, pSslHdrCfg->modeSelCfg.fdd_tdd_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_SPLIT_MODE_EN, pSslHdrCfg->modeSelCfg.split_mode_en) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_JACK_BIT, pSslHdrCfg->modeSelCfg.jack_bit) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_LTE_DESCRAMBLER_EN, pSslHdrCfg->modeSelCfg.lte_descrambler_en) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_MOD_TYPE_SEL, pSslHdrCfg->modeSelCfg.mod_type_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_CMUX_LN, pSslHdrCfg->modeSelCfg.cmux_ln) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_Q_FORMAT, pSslHdrCfg->modeSelCfg.q_format) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_B_MATRIX_SEL, pSslHdrCfg->modeSelCfg.b_matrix_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_TTI_2MS_10MS_SEL, pSslHdrCfg->modeSelCfg.tti_2ms_10ms_sel) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_WCDMA_NUM_SLOTS, pSslHdrCfg->modeSelCfg.wcdma_num_slots) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_WCDMA_NUM_PHY_CH, pSslHdrCfg->modeSelCfg.wcmda_num_phy_ch) |
                                        CSL_FMK (SSL_LTE_WIMAX_HDR_WORD1_RMUX_LN_INDEX, pSslHdrCfg->modeSelCfg.rmux_ln_index);
                pData   +=  4;
                break;
            }
            default:
            {
                /* Add 28 (Words 2-27) words of padding if specified */
                *(uint32_t *)pData  =   0;
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pSslHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_addDeinterleaverHeader
 *
 *  @b  brief
 *  @n  This API translates the De-interleaver (DNT) engine header settings 
 *      specified in the 'pDntHdrCfg' input parameter to a format that the 
 *      DNT engine in BCP would understand. It appends the DNT header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the DNT header length added.
 *
 *  @param[in]    
        pDntHdrCfg          DNT header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the DNT config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added DNT header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addDeinterleaverHeader (
    Bcp_DntHdrCfg*                      pDntHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    // if (!pDntHdrCfg || !pData || !pLen || pDntHdrCfg->local_hdr_len > 8)
    if (!pDntHdrCfg || !pData || !pLen )
        return -1;            

    for (i = 0; i < pDntHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   //CSL_FMK (DNT_HDR_WORD0_PPB_HEADER, pDntHdrCfg->ppb_config_header);
                                        CSL_FMK (DNT_HDR_WORD0_LOCAL_HDR_LEN, pDntHdrCfg->local_hdr_len) |
                                        CSL_FMK (DNT_HDR_WORD0_MOD_ID, Bcp_ModuleId_DNT);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD1_NUM_CONSTELLATION, pDntHdrCfg->num_constellation) |
                                        CSL_FMK (DNT_HDR_WORD1_NUM_FRAME_COUNT, pDntHdrCfg->num_frame_count) |
                                        CSL_FMK (DNT_HDR_WORD1_NUM_DESCRAMBLE, pDntHdrCfg->num_descramble);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD2_NUM_DATA_VALUE, pDntHdrCfg->num_data_value);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD3_NUM_R2_LENGTH_0, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD3_NUM_DUMMY_0, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD3_NUM_DATA_FORMAT_0, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD4_NUM_R2_LENGTH_1, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD4_NUM_DUMMY_1, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD4_NUM_DATA_FORMAT_1, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD5_NUM_R2_LENGTH_2, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD5_NUM_DUMMY_2, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD5_NUM_DATA_FORMAT_2, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD6_NUM_R2_LENGTH_3, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD6_NUM_DUMMY_3, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD6_NUM_DATA_FORMAT_3, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD7_NUM_R2_LENGTH_4, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD7_NUM_DUMMY_4, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD7_NUM_DATA_FORMAT_4, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (DNT_HDR_WORD8_NUM_R2_LENGTH_5, pDntHdrCfg->tblCfg[i-3].num_r2_length) |
                                        CSL_FMK (DNT_HDR_WORD8_NUM_DUMMY_5, pDntHdrCfg->tblCfg[i-3].num_dummy) |
                                        CSL_FMK (DNT_HDR_WORD8_NUM_DATA_FORMAT_5, pDntHdrCfg->tblCfg[i-3].num_data_format_in);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pDntHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;        
}

/**
 * ============================================================================
 *  @n@b Bcp_addLTE_RDHeader
 *
 *  @b  brief
 *  @n  This API translates the LTE Rate-dematcher (RD) engine header settings 
 *      specified in the 'pRdHdrCfg' input parameter to a format that the 
 *      RD engine in BCP would understand. It appends the RD header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RD header length added.
 *
 *  @param[in]    
        pRdHdrCfg           RD header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the RD config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added RD header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addLTE_RDHeader (
    Bcp_RdHdr_LteCfg*                   pRdHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pRdHdrCfg || !pData || !pLen || pRdHdrCfg->local_hdr_len > 7)
        return -1;            

    for (i = 0; i < pRdHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD0_LOCAL_HDR_LEN, pRdHdrCfg->local_hdr_len) |
                                        CSL_FMK (RD_LTE_HDR_WORD0_MOD_ID, Bcp_ModuleId_RD) |
                                        CSL_FMK (RD_LTE_HDR_WORD0_ENABLEHARQINPUT, pRdHdrCfg->enable_harq_input) |
                                        CSL_FMK (RD_LTE_HDR_WORD0_ENABLEHARQOUTPUT, pRdHdrCfg->enable_harq_output) |
                                        CSL_FMK (RD_LTE_HDR_WORD0_TCP3DSCALEFACTOR, pRdHdrCfg->tcp3d_scale_factor) |
                                        CSL_FMK (RD_LTE_HDR_WORD0_TURBODECDYNRNG, pRdHdrCfg->tcp3d_dyn_range);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD1_CQIPASSEDTHROUGH, pRdHdrCfg->cqi_pass_through) |
                                        CSL_FMK (RD_LTE_HDR_WORD1_RVSTARTCOLUMN, pRdHdrCfg->rv_start_column) |
                                        CSL_FMK (RD_LTE_HDR_WORD1_NUMFILLERBITSF, pRdHdrCfg->num_filler_bits_f) |
                                        CSL_FMK (RD_LTE_HDR_WORD1_INITCBFLOWID, pRdHdrCfg->init_cb_flowId) |
                                        CSL_FMK (RD_LTE_HDR_WORD1_FLOWIDHI, pRdHdrCfg->flowId_hi) |
                                        CSL_FMK (RD_LTE_HDR_WORD1_FLOWIDCQIOFFSET, pRdHdrCfg->flowId_cqi_offset);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD2_NUMCODEBLOCKSC1, pRdHdrCfg->num_code_blocks_c1) |
                                        CSL_FMK (RD_LTE_HDR_WORD2_NUMCODEBLOCKSCE1, pRdHdrCfg->num_code_blocks_ce1) |
                                        CSL_FMK (RD_LTE_HDR_WORD2_NUMCODEBLOCKSC2, pRdHdrCfg->num_code_blocks_c2) |
                                        CSL_FMK (RD_LTE_HDR_WORD2_NUMCODEBLOCKSCE2, pRdHdrCfg->num_code_blocks_ce2);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD3_BLOCKSIZEK1, pRdHdrCfg->block_size_k1) |
                                        CSL_FMK (RD_LTE_HDR_WORD3_BLOCKSIZEE1, pRdHdrCfg->block_size_e1);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD4_BLOCKSIZEK2, pRdHdrCfg->block_size_k2) |
                                        CSL_FMK (RD_LTE_HDR_WORD4_BLOCKSIZEE2, pRdHdrCfg->block_size_e2);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD5_BLOCKSIZEOUTQCQI, pRdHdrCfg->block_size_outq_cqi) |
                                        CSL_FMK (RD_LTE_HDR_WORD5_BLOCKSIZEINQCQI, pRdHdrCfg->block_size_inq_cqi);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD6_HARQINPUTADDRESS, pRdHdrCfg->harq_input_address);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_LTE_HDR_WORD7_HARQOUTPUTADDRESS, pRdHdrCfg->harq_output_address);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRdHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;        
}

/**
 * ============================================================================
 *  @n@b Bcp_addxCdma_RDHeader
 *
 *  @b  brief
 *  @n  This API translates the xCDMA (TD_SCDMA/WCDMA) Rate-dematcher (RD) engine 
 *      header settings specified in the 'pRdHdrCfg' input parameter to a format 
 *      that the RD engine in BCP would understand. It appends the RD header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RD header length added.
 *
 *  @param[in]    
        pRdHdrCfg           RD header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the RD config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added RD header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addxCdma_RDHeader (
    Bcp_RdHdr_xCdmaCfg*                 pRdHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pRdHdrCfg || !pData || !pLen || pRdHdrCfg->local_hdr_len > 41)
        return -1;            

    /* Add xCDMA RD header to the data buffer */
    for (i = 0; i < pRdHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD0_LOCAL_HDR_LEN, pRdHdrCfg->local_hdr_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_MOD_ID, Bcp_ModuleId_RD) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_FDD, pRdHdrCfg->fdd) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_EN_HARQ_IN, pRdHdrCfg->en_harq_in) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_EN_HARQ_OUT, pRdHdrCfg->en_harq_out) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_TCP3_DYN_RNG, pRdHdrCfg->tcp3d_dyn_range) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD0_TCP3_SCALE, pRdHdrCfg->tcp3d_scale_factor);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD1_COLLECT_COLS, pRdHdrCfg->collect_cols) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD1_COLLECT_ROWS, pRdHdrCfg->collect_rows);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD2_TURBO_LENGTH, pRdHdrCfg->turbo_length) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD2_TURBO_COUNT, pRdHdrCfg->turbo_count) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD2_FLOW_ID_INIT, pRdHdrCfg->flow_id_init) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD2_FLOW_ID_MAX, pRdHdrCfg->flow_id_max);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD3_HARQ_IN_ADDR, pRdHdrCfg->harq_in_addr);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD4_HARQ_OUT_ADDR, pRdHdrCfg->harq_out_addr);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD5_SYS0_LEN, pRdHdrCfg->sys0_len);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD6_SYS0_INIT2, pRdHdrCfg->sys0_init2);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD7_SYS0_MINUS2, pRdHdrCfg->sys0_minus2);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD8_SYS0_PLUS2, pRdHdrCfg->sys0_plus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD8_SYS0_ALPHA, pRdHdrCfg->sys0_alpha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD8_SYS0_BETA, pRdHdrCfg->sys0_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD8_SYS0_PUNCTURE, pRdHdrCfg->sys0_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD8_SYS0_TURBO, pRdHdrCfg->sys0_turbo);
                pData   +=  4;
                break;
            }
            case 9:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD9_P0_PAR1_LEN, pRdHdrCfg->p0_par1_len);
                pData   +=  4;
                break;
            }
            case 10:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD10_P0_PAR1_INIT1, pRdHdrCfg->p0_par1_init1);
                pData   +=  4;
                break;
            }
            case 11:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD11_P0_PAR1_MINUS1, pRdHdrCfg->p0_par1_minus1);
                pData   +=  4;
                break;
            }
            case 12:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD12_P0_PAR1_PLUS1, pRdHdrCfg->p0_par1_plus1);
                pData   +=  4;
                break;
            }
            case 13:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD13_P0_PAR1_INIT2, pRdHdrCfg->p0_par1_init2);
                pData   +=  4;
                break;
            }
            case 14:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD14_P0_PAR1_MINUS2, pRdHdrCfg->p0_par1_minus2);
                pData   +=  4;
                break;
            }
            case 15:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD15_P0_PAR1_PLUS2, pRdHdrCfg->p0_par1_plus2);
                pData   +=  4;
                break;
            }
            case 16:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD16_P0_PAR2_LEN, pRdHdrCfg->p0_par2_len);
                pData   +=  4;
                break;
            }
            case 17:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD17_P0_PAR2_INIT1, pRdHdrCfg->p0_par2_init1);
                pData   +=  4;
                break;
            }
            case 18:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD18_P0_PAR2_MINUS1, pRdHdrCfg->p0_par2_minus1);
                pData   +=  4;
                break;
            }
            case 19:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD19_P0_PAR2_PLUS1, pRdHdrCfg->p0_par2_plus1);
                pData   +=  4;
                break;
            }
            case 20:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD20_P0_PAR2_INIT2, pRdHdrCfg->p0_par2_init2);
                pData   +=  4;
                break;
            }
            case 21:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD21_P0_PAR2_MINUS2, pRdHdrCfg->p0_par2_minus2);
                pData   +=  4;
                break;
            }
            case 22:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD22_P0_PAR2_PLUS2, pRdHdrCfg->p0_par2_plus2);
                pData   +=  4;
                break;
            }
            case 23:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD23_SYS1_LEN, pRdHdrCfg->channelCfg[(i-23)/3].sys_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD23_SYS1_INIT2, pRdHdrCfg->channelCfg[(i-23)/3].sys_init2);
                pData   +=  4;
                break;
            }
            case 26:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD26_SYS2_LEN, pRdHdrCfg->channelCfg[(i-23)/3].sys_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD26_SYS2_INIT2, pRdHdrCfg->channelCfg[(i-23)/3].sys_init2);
                pData   +=  4;
                break;
            }
            case 29:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD29_SYS3_LEN, pRdHdrCfg->channelCfg[(i-23)/3].sys_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD29_SYS3_INIT2, pRdHdrCfg->channelCfg[(i-23)/3].sys_init2);
                pData   +=  4;
                break;
            }
            case 32:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD32_SYS4_LEN, pRdHdrCfg->channelCfg[(i-23)/3].sys_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD32_SYS4_INIT2, pRdHdrCfg->channelCfg[(i-23)/3].sys_init2);
                pData   +=  4;
                break;
            }
            case 35:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD35_SYS5_LEN, pRdHdrCfg->channelCfg[(i-23)/3].sys_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD35_SYS5_INIT2, pRdHdrCfg->channelCfg[(i-23)/3].sys_init2);
                pData   +=  4;
                break;
            }
            case 24:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD24_SYS1_MINUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD24_SYS1_PLUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_plus2);
                pData   +=  4;
                break;
            }
            case 27:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD27_SYS2_MINUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD27_SYS2_PLUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_plus2);
                pData   +=  4;
                break;
            }
            case 30:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD30_SYS3_MINUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD30_SYS3_PLUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_plus2);
                pData   +=  4;
                break;
            }
            case 33:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD33_SYS4_MINUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD33_SYS4_PLUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_plus2);
                pData   +=  4;
                break;
            }
            case 36:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD36_SYS5_MINUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD36_SYS5_PLUS2, pRdHdrCfg->channelCfg[(i-24)/3].sys_plus2);
                pData   +=  4;
                break;
            }
            case 25:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD25_SYS1_ALPHA, pRdHdrCfg->channelCfg[(i-25)/3].sys_aplha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD25_SYS1_BETA, pRdHdrCfg->channelCfg[(i-25)/3].sys_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD25_SYS1_PUNCTURE, pRdHdrCfg->channelCfg[(i-25)/3].sys_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD25_SYS1_TURBO, pRdHdrCfg->channelCfg[(i-25)/3].sys_turbo);
                pData   +=  4;
                break;
            }
            case 28:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD28_SYS2_ALPHA, pRdHdrCfg->channelCfg[(i-25)/3].sys_aplha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD28_SYS2_BETA, pRdHdrCfg->channelCfg[(i-25)/3].sys_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD28_SYS2_PUNCTURE, pRdHdrCfg->channelCfg[(i-25)/3].sys_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD28_SYS2_TURBO, pRdHdrCfg->channelCfg[(i-25)/3].sys_turbo);
                pData   +=  4;
                break;
            }
            case 31:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD31_SYS3_ALPHA, pRdHdrCfg->channelCfg[(i-25)/3].sys_aplha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD31_SYS3_BETA, pRdHdrCfg->channelCfg[(i-25)/3].sys_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD31_SYS3_PUNCTURE, pRdHdrCfg->channelCfg[(i-25)/3].sys_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD31_SYS3_TURBO, pRdHdrCfg->channelCfg[(i-25)/3].sys_turbo);
                pData   +=  4;
                break;
            }
            case 34:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD34_SYS4_ALPHA, pRdHdrCfg->channelCfg[(i-25)/3].sys_aplha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD34_SYS4_BETA, pRdHdrCfg->channelCfg[(i-25)/3].sys_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD34_SYS4_PUNCTURE, pRdHdrCfg->channelCfg[(i-25)/3].sys_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD34_SYS4_TURBO, pRdHdrCfg->channelCfg[(i-25)/3].sys_turbo);
                pData   +=  4;
                break;
            }
            case 37:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD37_SYS5_ALPHA, pRdHdrCfg->channelCfg[(i-25)/3].sys_aplha) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD37_SYS5_BETA, pRdHdrCfg->channelCfg[(i-25)/3].sys_beta) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD37_SYS5_PUNCTURE, pRdHdrCfg->channelCfg[(i-25)/3].sys_puncture) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD37_SYS5_TURBO, pRdHdrCfg->channelCfg[(i-25)/3].sys_turbo);
                pData   +=  4;
                break;
            }
            case 38:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD38_P1_PAR1_LEN, pRdHdrCfg->p1_par1_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD38_P1_PAR1_INIT2, pRdHdrCfg->p1_par1_init2);
                pData   +=  4;
                break;
            }
            case 39:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD39_P1_PAR1_MINUS2, pRdHdrCfg->p1_par1_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD39_P1_PAR1_PLUS2, pRdHdrCfg->p1_par1_plus2);
                pData   +=  4;
                break;
            }
            case 40:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD40_P1_PAR2_LEN, pRdHdrCfg->p1_par2_len) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD40_P1_PAR2_INIT2, pRdHdrCfg->p1_par2_init2);
                pData   +=  4;
                break;
            }
            case 41:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_XCDMA_HDR_WORD41_P1_PAR2_MINUS2, pRdHdrCfg->p1_par2_minus2) |
                                        CSL_FMK (RD_XCDMA_HDR_WORD41_P1_PAR2_PLUS2, pRdHdrCfg->p1_par2_plus2);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRdHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;        
}

#if 0
/**
 * ============================================================================
 *  @n@b Bcp_addWiMax_RDHeader
 *
 *  @b  brief
 *  @n  This API translates the WiMax Rate-dematcher (RD) engine header settings 
 *      specified in the 'pRdHdrCfg' input parameter to a format that the 
 *      RD engine in BCP would understand. It appends the RD header thus 
 *      built to the data buffer passed to this API in 'pData' and increments the 
 *      data buffer length by the RD header length added.
 *
 *  @param[in]    
        pRdHdrCfg           RD header input that needs to be formatted 
                            and added to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the RD config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added RD header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addWiMax_RDHeader (
    Bcp_RdHdr_WiMaxCfg*                 pRdHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pRdHdrCfg || !pData || !pLen || pRdHdrCfg->local_hdr_len > 8)
        return -1;            

    for (i = 0; i < pRdHdrCfg->local_hdr_len + 1; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD0_LOCAL_HDR_LEN, pRdHdrCfg->local_hdr_len) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD0_MOD_ID, Bcp_ModuleId_RD) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD0_EN_HARQ_INPUT, pRdHdrCfg->enable_harq_input) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD0_EN_HARQ_OUTPUT, pRdHdrCfg->enable_harq_output) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD0_TCP3D_SCALE_FACTOR, pRdHdrCfg->tcp3d_scale_factor) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD0_TURBO_DEC_DYN_RANGE, pRdHdrCfg->tcp3d_dyn_range);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD1_NUM_CODE_BLOCKS_C1, pRdHdrCfg->num_code_blocks_c1) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD1_FLOW_ID_START_OFFSET, pRdHdrCfg->flowId_starting_offset) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD1_FLOW_ID_END_OFFSET, pRdHdrCfg->flowId_ending_offset) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD1_BLOCK_SIZE_K1, pRdHdrCfg->block_size_k1);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD2_PARAM_M1, pRdHdrCfg->param_m1) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD2_PARAM_J1, pRdHdrCfg->param_j1) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD2_BLOCK_SIZE_E1, pRdHdrCfg->block_size_e1);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD3_NUM_CODE_BLOCKS_C2, pRdHdrCfg->num_code_blocks_c2) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD3_BLOCK_SIZE_K2, pRdHdrCfg->block_size_k2);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD4_PARAM_M2, pRdHdrCfg->param_m2) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD4_PARAM_J2, pRdHdrCfg->param_j2) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD4_BLOCK_SIZE_E2, pRdHdrCfg->block_size_e2);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD5_NUM_CODE_BLOCKS_C3, pRdHdrCfg->num_code_blocks_c3) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD5_BLOCK_SIZE_K3, pRdHdrCfg->block_size_k3);
                pData   +=  4;
                break;
            }
            case 6:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD6_PARAM_M3, pRdHdrCfg->param_m3) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD6_PARAM_J3, pRdHdrCfg->param_j3) |
                                        CSL_FMK (RD_WIMAX_HDR_WORD6_BLOCK_SIZE_E3, pRdHdrCfg->block_size_e3);
                pData   +=  4;
                break;
            }
            case 7:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD7_HARQ_IN_ADDRESS, pRdHdrCfg->harq_input_address);
                pData   +=  4;
                break;
            }
            case 8:
            {
                *(uint32_t *)pData  =   CSL_FMK (RD_WIMAX_HDR_WORD8_HARQ_OUT_ADDRESS, pRdHdrCfg->harq_output_address);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pRdHdrCfg->local_hdr_len + 1) * 4);

    /* Return success */        
    return 0;        
}
#endif

/**
 * ============================================================================
 *  @n@b Bcp_addDIOHeader
 *
 *  @b  brief
 *  @n  This API translates the DIO header settings specified in the 'pDioHdrCfg'
 *      input parameter to a format that the BCP would understand.
 *      It appends the DIO header thus built to the data buffer passed to this
 *      API in 'pData' and increments the data buffer length by the DIO header
 *      length added.
 *
 *  @param[in]    
        pDioHdrCfg          DIO header input that needs to be formatted and added
                            to the data buffer.

 *  @param[out]                               
        pData               Data buffer filled in as a result of successful translation 
                            of the DIO header config provided as input to this function.

*  @param[out]
        pLen                Data buffer length pointer that will be incremented by the 
                            number of bytes configured by this API. This parameter will
                            be updated only if the API was successful.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid/Insufficient configuration provided.
 *  @li                     0   -   Successfully added DIO header to the data buffer.
 *
 *  @pre
 *  @n  The Data buffer pointer 'pData' must have been allocated memory before passing 
 *      to this API. Also 'pLen' pointer passed to this API must be a valid pointer.
 *
 *  @post
 *  @n  The output params structure pData is populated accordingly and pLen parameter is 
 *      updated with the number of bytes of configuration data filled in by this API.
 * ============================================================================
 */
int32_t Bcp_addDIOHeader (
    Bcp_DioHdrCfg*                      pDioHdrCfg, 
    uint8_t*                            pData,
    uint32_t*                           pLen
)
{
    uint32_t                            i;

    if (!pDioHdrCfg || !pData || !pLen)
        return -1;

    *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD0_LOCAL_HDR_LEN, pDioHdrCfg->local_hdr_len) |
                            CSL_FMK (DIO_HDR_WORD0_MOD_ID, Bcp_ModuleId_DIO) | 
                            CSL_FMK (DIO_HDR_WORD0_DIO_ENDIAN, pDioHdrCfg->dio_endian) | 
                            CSL_FMK (DIO_HDR_WORD0_DIO_RD_WR, pDioHdrCfg->dio_rd_wr) | 
                            CSL_FMK (DIO_HDR_WORD0_DIO_BLK_CNT, pDioHdrCfg->dio_blk_cnt);
    pData   +=  4;
    *pLen   +=  4;
   
    for (i = 0; i < pDioHdrCfg->dio_blk_cnt; i ++)
    {
        switch (i)
        {
            case 0:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD1_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD2_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
            case 1:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD3_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD4_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
            case 2:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD5_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD6_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
            case 3:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD7_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD8_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
            case 4:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD9_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD10_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
            case 5:
            {
                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD11_DIO_ADDR, pDioHdrCfg->dio_dmablk_cfg[i].dio_addr);
                pData   +=  4;

                *(uint32_t *)pData  =   CSL_FMK (DIO_HDR_WORD12_DIO_CNT, pDioHdrCfg->dio_dmablk_cfg[i].dio_cnt);
                pData   +=  4;
                break;
            }
        }
    }

    *pLen   +=  ((pDioHdrCfg->dio_blk_cnt * 2) * 4);
    
    /* Return success */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_lldOpen
 *
 *  @b  brief
 *  @n  This API initializes the output param 'pBcpLldObj' with the configuration 
 *      register address for the peripheral instance corresponding 
 *      to the instance number passed in 'instNum' parameter. This API MUST be
 *      called to initialize the BCP LLD object corresponding to a specific 
 *      BCP peripheral instance before any of the BCP LLD APIs are invoked 
 *      to configure that instance MMR.
 *
 *  @param[in]    
        instNum             BCP peripheral instance number for which the LLD
                            object needs to be initialized.

 *  @param[in]    
        cfgRegsBaseAddress  Configuration registers (MMR) base address for this
                            BCP instance.

 *  @param[out]    
        pBcpLldObj          Pointer to BCP LLD Object structure that needs to be
                            initialized with the instance's configuration register 
                            overlay base address.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid LLD object handle/ invalid instance number
 *                                  provided.
 *  @li                     0   -   Successfully initialized the LLD object with
 *                                  the appropriate base address for the instance 
 *                                  number provided.
 *
 *  @pre
 *  @n  The output BCP LLD object structure 'pBcpLldObj' passed must be a valid
 *      pointer. 
 *
 *  @post
 *  @n  The BCP LLD object pointer passed is populated with appropriate base address
 *      for the configuration registers for the corresponding instance number.
 * 
 *  @code
        Bcp_LldObj                 bcpLldObj;

        if (Bcp_lldOpen (0, 0x35200000, &bcpLldObj) != 0)
        {
            // Error opening BCP LLD for 0 instance
            ...
        }
        else
        {
            // Successfully opened BCP LLD for instance specified
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Bcp_lldOpen 
(
    uint8_t                     instNum,
    uint32_t                    cfgRegsBaseAddress,
    Bcp_LldObj*                 pBcpLldObj
)
{
    int32_t                     i;        

    /* Validate the BCP LLD Object Handle passed */        
    if (!pBcpLldObj)
        return -1;

    pBcpLldObj->instNum     =   instNum;

    /* Setup the base addresses for CFG, Interrupt and Data logger registers for all BCP submodules. */
    for (i = 0; i < BCP_NUM_SUBMODULES; i ++)
    {
        switch (i)
        {
            case Bcp_ModuleId_TM:
            {
                /* PPB0..PPB3 are all part of TM and hence share the same set of MMRs */                            
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0000);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0080);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x00F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x2000);

                break;
            }
            case Bcp_ModuleId_INT:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0500);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0580);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x05F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x4000);

                break;
            }
            case Bcp_ModuleId_RM:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0600);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0680);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x06F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x5000);

                break;
            }
            case Bcp_ModuleId_ENC:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0700);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0780);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x07F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x6000);

                break;
            }
            case Bcp_ModuleId_MOD:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0800);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0880);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x08F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x7000);

                break;
            }
            case Bcp_ModuleId_CRC:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0D00);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0D80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x0DF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0xC000);

                break;
            }
            case Bcp_ModuleId_SSL:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0A00);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0A80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x0AF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x9000);

                break;
            }
            case Bcp_ModuleId_RD:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0B00);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0B80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x0BF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0xA000);

                break;
            }
            case Bcp_ModuleId_COR:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0C00);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0C80);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x0CF0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0xB000);

                break;
            }
            case Bcp_ModuleId_DNT:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0900);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0980);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x09F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x8000);

                break;
            }
            case Bcp_ModuleId_DIO:
            {
                pBcpLldObj->modCfgRegs [i]      =   (void *) (cfgRegsBaseAddress + 0x0400);            
                pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (cfgRegsBaseAddress + 0x0480);
                pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (cfgRegsBaseAddress + 0x04F0);
                pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (cfgRegsBaseAddress + 0x3000);

                break;
            }
            default:
            {
                break;                    
            }
        }
    }

    /* BCP LLD Open successful. Return success. */
    return 0;
}

/**
 * ============================================================================
 *  @n@b Bcp_lldClose
 *
 *  @b  brief
 *  @n  This API resets the contents of the BCP LLD Object handle passed to 
 *      this function. The BCP Object handle is no longer valid for use with
 *      any of the BCP LLD MMR access APIs.
 *
 *  @param[out]    
        pBcpLldObj          Pointer to BCP LLD Object structure that needs to be
                            de-initialized.
 *
 *  @return     int32_t
 *  @li                     -1  -   Invalid LLD object handle provided.
 *  @li                     0   -   Successfully de-initialized the LLD object provided.
 *
 *  @pre
 *  @n  The output BCP LLD object structure 'pBcpLldObj' passed must be a valid
 *      pointer.
 *
 *  @post
 *  @n  The BCP LLD object pointer's contents are all de-initialized. This object 
 *      pointer is no longer valid for use with any of the BCP LLD MMR access
 *      APIs. 
 * 
 *  @code
        Bcp_LldObj                 bcpLldObj;

        if (Bcp_lldClose (&bcpLldObj) != 0)
        {
            // Error closing BCP LLD for instance specified
            ...
        }
        else
        {
            // Successfully closed BCP LLD for instance specified
            ...
        }
        
     @endcode
 * ============================================================================
 */
int32_t Bcp_lldClose 
(
    Bcp_LldObj*                 pBcpLldObj
)
{
    uint32_t                    i;

    /* Validate the BCP LLD Object Handle passed */        
    if (!pBcpLldObj || !pBcpLldObj->modCfgRegs[0])
        return -1;

    pBcpLldObj->instNum    =   0;

    for (i = 0; i < BCP_NUM_SUBMODULES; i ++)
    {
        pBcpLldObj->modCfgRegs [i]      =   (void *) (0x00000000);            
        pBcpLldObj->modIntRegs [i]      =   (CSL_Bcp_IntRegs *) (0x00000000);
        pBcpLldObj->modDlgRegs [i]      =   (CSL_Bcp_DataLoggerRegs *) (0x00000000);
        pBcpLldObj->modDlgRamRegs [i]   =   (CSL_Bcp_DataLoggerRAM *) (0x00000000);
    }
            
    /* BCP LLD close successful. Return success. */
    return 0;
}

/**
@}
*/
