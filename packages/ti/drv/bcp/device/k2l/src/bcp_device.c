/** 
 *   @file  bcp_device.c
 *
 *   @brief  
 *      This file contains any device (SoC) specific definitions and functions
 *      required for BCP setup.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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
/* BCP types include */
#include <bcp_types.h>

/* BCP include */
#include <ti/drv/bcp/bcp.h>

/**
 * ============================================================================
 *  @n@b Bcp_initDevice
 *
 *  @b  brief
 *  @n  This API provides a sample initialization sequence for BCP that can be
 *      called as part of system level initialization during device startup. 
 *      The initialization sequence provided here is just an example and 
 *      is expected to be customized as per the customer's system and its needs.
 *
 *  @param[in]    
 *      pBcpLldObj          BCP LLD Object Handle.
 *
 *  @return     int32_t
 *  @li         0       -   BCP device init succesfully done.                                              
 *  @li         <0      -   BCP device init failed.                                              
 * ============================================================================
 */
int32_t Bcp_initDevice (Bcp_LldObj* pBcpLldObj)
{
    uint8_t             destSel [BCP_MAX_NUM_TXQUEUES], prioVal [BCP_MAX_NUM_TXQUEUES];
        
	/* Setup TM MMR */
    Bcp_setCdmaHpSrcId (pBcpLldObj, CSL_BCP_TM_TM_CONTROL_CDMAHP_SRC_ID_RESETVAL);        
    Bcp_enableTxCdmaHpReadArbPrio (pBcpLldObj);

    /* Setup the Tx queue --> PPB (Port) Mapping.
     *
     * Lets use the following configuration:
     * Tx Q0 --> PPB0   Tx Q1 --> PPB1  TX Q2 --> PPB2  Tx Q3 --> PPB3
     * Tx Q4 --> PPB0   Tx Q5 --> PPB1  TX Q6 --> PPB2  Tx Q7 --> PPB3 
     *
     * All are at the same priority for now.
     */
    destSel [0] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_0_RESETVAL;
    destSel [1] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_1_RESETVAL;
    destSel [2] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_2_RESETVAL;
    destSel [3] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_3_RESETVAL;
    destSel [4] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_4_RESETVAL;
    destSel [5] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_5_RESETVAL;
    destSel [6] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_6_RESETVAL;
    destSel [7] = CSL_BCP_TM_TM_TX_QFIFO_RD_DEST_SEL_TX_QFIFO_RD_DEST_SEL_7_RESETVAL;
    memset (prioVal, 0, sizeof (uint8_t) * BCP_MAX_NUM_TXQUEUES);
    Bcp_setTxQfifoReadDestSelReg (pBcpLldObj, destSel, prioVal);

    /* Initialize the Encoder engine block */        
    Bcp_setEncPolyCoef1Reg (pBcpLldObj, CSL_BCP_ENC_POLY_COEF1_POLYCOEF1_RESETVAL);      //WCDMA code rate 1/2
    Bcp_setEncPolyCoef2Reg (pBcpLldObj, CSL_BCP_ENC_POLY_COEF2_POLYCOEF2_RESETVAL);      //WCDMA code rate 1/3
    Bcp_setEncPolyCoef3Reg (pBcpLldObj, CSL_BCP_ENC_POLY_COEF3_POLYCOEF3_RESETVAL);      //LTE code rate 1/3

    Bcp_setEncScrInit0Reg (pBcpLldObj, CSL_BCP_ENC_SCR_INIT_0_SCRINIT_RESETVAL);            //scrambler initialization
    Bcp_setEncScrPoly0Reg (pBcpLldObj, CSL_BCP_ENC_SCR_POLY_0_SCRPOLY_RESETVAL);            //scrambler polynomial
    Bcp_setEncCrc24Init0Reg (pBcpLldObj, CSL_BCP_ENC_CRC24_INIT_0_CRC24INIT_RESETVAL);          //crc24 initialization
    Bcp_setEncCrc24Poly0Reg (pBcpLldObj, CSL_BCP_ENC_CRC24_POLY_0_CRC24POLY_RESETVAL);     //crc24 polynomial

    Bcp_setEncScrInit1Reg (pBcpLldObj, CSL_BCP_ENC_SCR_INIT_1_SCRINIT_RESETVAL);         //scrambler initialization
    Bcp_setEncScrPoly1Reg (pBcpLldObj, CSL_BCP_ENC_SCR_POLY_1_SCRPOLY_RESETVAL);         //scrambler polynomial
    Bcp_setEncCrc16Init1Reg (pBcpLldObj, CSL_BCP_ENC_CRC16_INIT_1_CRC16INIT_RESETVAL);          //crc16 initialization
    Bcp_setEncCrc16Poly1Reg (pBcpLldObj, CSL_BCP_ENC_CRC16_POLY_1_CRC16POLY_RESETVAL);       //crc16 polynomial

    Bcp_setEncScrInit2Reg (pBcpLldObj, CSL_BCP_ENC_SCR_INIT_2_SCRINIT_RESETVAL);         //scrambler initialization
    Bcp_setEncScrPoly2Reg (pBcpLldObj, CSL_BCP_ENC_SCR_POLY_2_SCRPOLY_RESETVAL);         //scrambler polynomial

    /* Initialize the Correlator MMR */
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 0, CSL_BCP_COR_M0_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 1, CSL_BCP_COR_M1_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 2, CSL_BCP_COR_M2_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 3, CSL_BCP_COR_M3_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 4, CSL_BCP_COR_M4_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 5, CSL_BCP_COR_M5_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 6, CSL_BCP_COR_M6_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 7, CSL_BCP_COR_M7_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 8, CSL_BCP_COR_M8_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 9, CSL_BCP_COR_M9_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 10, CSL_BCP_COR_M10_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 11, CSL_BCP_COR_M11_MOD_EN_RESETVAL);
    Bcp_setCorReedMullerTableColumn (pBcpLldObj, 12, CSL_BCP_COR_M12_MOD_EN_RESETVAL);
    Bcp_setCorControlReg (pBcpLldObj, CSL_BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K0_RESETVAL,
                                      CSL_BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K10_C2K1_RESETVAL,
                                      CSL_BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K0_RESETVAL,
                                      CSL_BCP_COR_CONTROL_REGISTER_QPSK_MAPPING_C2K11_C2K1_RESETVAL);

    return 0;        
}
