/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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

/** @file dfe_fl_txAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_TX CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_TXAUX_H_
#define _DFE_FL_TXAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_tx.h>

/** ============================================================================
 *   @n@b dfeFl_TxConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_INITS_SSEL
 *       DFE_TX_TXRX_TXRX_718_REG_CLEAR_DATA
 *       DFE_TX_TXRX_TXRX_718_REG_INIT_STATE
 *       DFE_TX_TXRX_TXRX_718_REG_INIT_CLK_GATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_TxConfigInits(DfeFl_TxHandle hTx, DfeFl_SublkInitsConfig * arg)
{
    
    uint32_t data = hTx->regs->txrx_txrx_718;
    
    CSL_FINS(data, DFE_TX_TXRX_TXRX_718_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_TX_TXRX_TXRX_718_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_TX_TXRX_TXRX_718_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_TX_TXRX_TXRX_718_REG_CLEAR_DATA, arg->clearData);
    
    hTx->regs->txrx_txrx_718 = data;    
}

/** ============================================================================
 *   @n@b dfeFl_TxGetInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_INITS_SSEL
 *       DFE_TX_TXRX_TXRX_718_REG_CLEAR_DATA
 *       DFE_TX_TXRX_TXRX_718_REG_INIT_STATE
 *       DFE_TX_TXRX_TXRX_718_REG_INIT_CLK_GATE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetInits(DfeFl_TxHandle hTx, DfeFl_SublkInitsConfig * arg)
{

    uint32_t data = hTx->regs->txrx_txrx_718;

    arg->ssel = CSL_FEXT(data, DFE_TX_TXRX_TXRX_718_REG_INITS_SSEL);
    arg->initClkGate = CSL_FEXT(data, DFE_TX_TXRX_TXRX_718_REG_INIT_CLK_GATE);
    arg->initState = CSL_FEXT(data, DFE_TX_TXRX_TXRX_718_REG_INIT_STATE);
    arg->clearData = CSL_FEXT(data, DFE_TX_TXRX_TXRX_718_REG_CLEAR_DATA);

}

/** ============================================================================
 *   @n@b dfeFl_TxSetTestbusSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_TB_SEL
 *       DFE_TX_TXA_TXA_11_REG_TB_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetTestbusSel(DfeFl_TxHandle hDfeTx, DfeFl_TxTestBusSel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_TB_SEL, arg->sel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_TB_SEL, arg->sel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetTestbusSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_TB_SEL
 *       DFE_TX_TXA_TXA_11_REG_TB_SEL
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetTestbusSel(DfeFl_TxHandle hDfeTx, DfeFl_TxTestBusSel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->sel = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_TB_SEL);
    	break;
    case DFE_FL_TXB:
    	arg->sel = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_TB_SEL);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TX_TX_110_REG_SSEL_CKSUM
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetChksumSsel(DfeFl_TxHandle hDfeTx, uint32_t ssel)
{
    CSL_FINS(hDfeTx->regs->tx_tx_110, DFE_TX_TX_TX_110_REG_SSEL_CKSUM, ssel);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetSiggenMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_GEN_FRAME
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_GEN_DATA
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_SEED
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_FRAME_LEN_M1
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_RAMP_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetSiggenMode(DfeFl_TxHandle hDfeTx, DfeFl_TxSiggenMode *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeTx->regs->signal_gen_signal_gen_96;


    regs[0] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_FRAME_LEN_M1, arg->frameLenM1)
    		| CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_SEED, arg->seed)
            | CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_RAMP_MODE, arg->mode)
            | CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_GEN_FRAME, arg->frame)
            | CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_96_REG_GEN_DATA, arg->enable);
}

/** ============================================================================
 *   @n@b dfeFl_TxConfigSiggenRamp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_100_REG_RAMP_STOP_31_16
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_102_REG_RAMP_SLOPE_31_16
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_97_REG_RAMP_START_15_0
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_101_REG_RAMP_SLOPE_15_0
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_98_REG_RAMP_START_31_16
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_99_REG_RAMP_STOP_15_0
 *       DFE_TX_SIGNAL_GEN_SIGNAL_GEN_103_REG_GEN_TIMER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxConfigSiggenRamp(DfeFl_TxHandle hDfeTx, DfeFl_TxSiggenRampConfig *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeTx->regs->signal_gen_signal_gen_97;

    // ramp start
    regs[0] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_97_REG_RAMP_START_15_0, arg->rampStart);
    regs[1] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_98_REG_RAMP_START_31_16, arg->rampStart >> 16);

    // ramp stop
    regs[2] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_99_REG_RAMP_STOP_15_0, arg->rampStop);
    regs[3] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_100_REG_RAMP_STOP_31_16, arg->rampStop >> 16);

    // ramp slope
    regs[4] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_101_REG_RAMP_SLOPE_15_0, arg->rampSlope);
    regs[5] = CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_102_REG_RAMP_SLOPE_31_16, arg->rampSlope >> 16);

    // gen_timer
    regs[6]= CSL_FMK(DFE_TX_SIGNAL_GEN_SIGNAL_GEN_103_REG_GEN_TIMER, arg->genTimer);
}

/** ============================================================================
 *   @n@b dfeFl_TxConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_CHECK_SUM_CHECK_SUM_105_REG_MODE
 *       DFE_TX_CHECK_SUM_CHECK_SUM_106_REG_SIGNAL_LEN
 *       DFE_TX_CHECK_SUM_CHECK_SUM_105_REG_STABLE_LEN
 *       DFE_TX_CHECK_SUM_CHECK_SUM_107_REG_CHAN_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxConfigChksum(DfeFl_TxHandle hDfeTx, DfeFl_TxChksumConfig *arg)
{
    hDfeTx->regs->check_sum_check_sum_105 = CSL_FMK(DFE_TX_CHECK_SUM_CHECK_SUM_105_REG_MODE, arg->chksumMode)
                                          | CSL_FMK(DFE_TX_CHECK_SUM_CHECK_SUM_105_REG_STABLE_LEN, arg->latencyMode.stableLen);
    hDfeTx->regs->check_sum_check_sum_106 = CSL_FMK(DFE_TX_CHECK_SUM_CHECK_SUM_106_REG_SIGNAL_LEN, arg->latencyMode.signalLen);
    hDfeTx->regs->check_sum_check_sum_107 = CSL_FMK(DFE_TX_CHECK_SUM_CHECK_SUM_107_REG_CHAN_SEL, arg->latencyMode.chanSel);
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_CHECK_SUM_CHECK_SUM_109_REG_RESULT_31_16
 *       DFE_TX_CHECK_SUM_CHECK_SUM_108_REG_RESULT_15_0
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryChksumResult(DfeFl_TxHandle hDfeTx, uint32_t *arg)
{
    uint32_t data;

    data = CSL_FEXT(hDfeTx->regs->check_sum_check_sum_108, DFE_TX_CHECK_SUM_CHECK_SUM_108_REG_RESULT_15_0);
    data |= CSL_FEXT(hDfeTx->regs->check_sum_check_sum_109, DFE_TX_CHECK_SUM_CHECK_SUM_109_REG_RESULT_31_16) << 16;

	*arg = data;
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_128_REG_RSMPA_COEF
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpCoeff(DfeFl_TxHandle hDfeTx, uint32_t txDev, uint32_t idx, uint32_t coeff)
{
	volatile uint32_t *regs;

	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
		regs = &hDfeTx->regs->txa_txa_128[0];
		break;
	case DFE_FL_TXA_PATHB:
		regs = &hDfeTx->regs->txa_txa_256[0];
		break;
	case DFE_FL_TXB_PATHA:
		regs = &hDfeTx->regs->txb_txb_384[0];
		break;
	case DFE_FL_TXB_PATHB:
		regs = &hDfeTx->regs->txb_txb_512[0];
		break;
	default:
		return;
	}
	regs[idx] = CSL_FMK(DFE_TX_TXA_TXA_128_REG_RSMPA_COEF, coeff);
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryRsmpCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_128_REG_RSMPA_COEF
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryRsmpCoeff(DfeFl_TxHandle hDfeTx, uint32_t txDev, uint32_t idx, uint32_t *coeff)
{
	volatile uint32_t *regs;

	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
		regs = &hDfeTx->regs->txa_txa_128[0];
		break;
	case DFE_FL_TXA_PATHB:
		regs = &hDfeTx->regs->txa_txa_256[0];
		break;
	case DFE_FL_TXB_PATHA:
		regs = &hDfeTx->regs->txb_txb_384[0];
		break;
	case DFE_FL_TXB_PATHB:
		regs = &hDfeTx->regs->txb_txb_512[0];
		break;
	default:
		return;
	}
	*coeff = CSL_FEXT(regs[idx], DFE_TX_TXA_TXA_128_REG_RSMPA_COEF);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpCfSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_CF_SWAP
 *       DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_CF_SWAP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpCfSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_CF_SWAP, arg->ssel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_CF_SWAP, arg->ssel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpCfSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_CF_SWAP
 *       DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_CF_SWAP
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpCfSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_CF_SWAP);
    	break;
    case DFE_FL_TXB:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_CF_SWAP);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpInitPhSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_PH_INIT
 *       DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_PH_INIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpInitPhSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_PH_INIT, arg->ssel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_PH_INIT, arg->ssel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpInitPhSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_PH_INIT
 *       DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_PH_INIT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpInitPhSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_RSMP_PH_INIT);
    	break;
    case DFE_FL_TXB:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_RSMP_PH_INIT);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpAntPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_21_REG_RSMP_ANT_PH_A
 *       DFE_TX_TXA_TXA_5_REG_RSMP_ANT_PH_A
 *       DFE_TX_TXB_TXB_22_REG_RSMP_ANT_PH_B
 *       DFE_TX_TXA_TXA_6_REG_RSMP_ANT_PH_B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpAntPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_5, DFE_TX_TXA_TXA_5_REG_RSMP_ANT_PH_A, arg->phase);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_6, DFE_TX_TXA_TXA_6_REG_RSMP_ANT_PH_B, arg->phase);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_21, DFE_TX_TXB_TXB_21_REG_RSMP_ANT_PH_A, arg->phase);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_22, DFE_TX_TXB_TXB_22_REG_RSMP_ANT_PH_B, arg->phase);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpAntPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_21_REG_RSMP_ANT_PH_A
 *       DFE_TX_TXA_TXA_5_REG_RSMP_ANT_PH_A
 *       DFE_TX_TXB_TXB_22_REG_RSMP_ANT_PH_B
 *       DFE_TX_TXA_TXA_6_REG_RSMP_ANT_PH_B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpAntPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_5, DFE_TX_TXA_TXA_5_REG_RSMP_ANT_PH_A);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_6, DFE_TX_TXA_TXA_6_REG_RSMP_ANT_PH_B);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_21, DFE_TX_TXB_TXB_21_REG_RSMP_ANT_PH_A);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_22, DFE_TX_TXB_TXB_22_REG_RSMP_ANT_PH_B);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpInitPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_22_REG_RSMP_INIT_PH_B
 *       DFE_TX_TXA_TXA_5_REG_RSMP_INIT_PH_A
 *       DFE_TX_TXB_TXB_21_REG_RSMP_INIT_PH_A
 *       DFE_TX_TXA_TXA_6_REG_RSMP_INIT_PH_B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpInitPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_5, DFE_TX_TXA_TXA_5_REG_RSMP_INIT_PH_A, arg->phase);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_6, DFE_TX_TXA_TXA_6_REG_RSMP_INIT_PH_B, arg->phase);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_21, DFE_TX_TXB_TXB_21_REG_RSMP_INIT_PH_A, arg->phase);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_22, DFE_TX_TXB_TXB_22_REG_RSMP_INIT_PH_B, arg->phase);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpInitPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_22_REG_RSMP_INIT_PH_B
 *       DFE_TX_TXA_TXA_5_REG_RSMP_INIT_PH_A
 *       DFE_TX_TXB_TXB_21_REG_RSMP_INIT_PH_A
 *       DFE_TX_TXA_TXA_6_REG_RSMP_INIT_PH_B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpInitPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_5, DFE_TX_TXA_TXA_5_REG_RSMP_INIT_PH_A);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_6, DFE_TX_TXA_TXA_6_REG_RSMP_INIT_PH_B);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_21, DFE_TX_TXB_TXB_21_REG_RSMP_INIT_PH_A);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_22, DFE_TX_TXB_TXB_22_REG_RSMP_INIT_PH_B);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_RSMP_MODE
 *       DFE_TX_TXB_TXB_23_REG_RSMP_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpMode(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_RSMP_MODE, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_RSMP_MODE, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_RSMP_MODE
 *       DFE_TX_TXB_TXB_23_REG_RSMP_MODE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpMode(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_RSMP_MODE);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_RSMP_MODE);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetRsmpState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_RSMP_STATE
 *       DFE_TX_TXB_TXB_23_REG_RSMP_STATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetRsmpState(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_RSMP_STATE, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_RSMP_STATE, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetRsmpState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_RSMP_STATE
 *       DFE_TX_TXB_TXB_23_REG_RSMP_STATE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetRsmpState(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_RSMP_STATE);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_RSMP_STATE);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMixSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_MIX
 *       DFE_TX_TXB_TXB_23_REG_SSEL_MIX
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMixSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_MIX, arg->ssel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_MIX, arg->ssel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMixSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_SSEL_MIX
 *       DFE_TX_TXB_TXB_23_REG_SSEL_MIX
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMixSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_MIX);
    	break;
    case DFE_FL_TXB:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_MIX);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMixFreq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_1_REG_FREQ_0_LSB
 *       DFE_TX_TXB_TXB_19_REG_FREQ_1_LSB
 *       DFE_TX_TXA_TXA_3_REG_FREQ_1_LSB
 *       DFE_TX_TXB_TXB_20_REG_FREQ_1_MSB
 *       DFE_TX_TXB_TXB_18_REG_FREQ_0_MSB
 *       DFE_TX_TXA_TXA_4_REG_FREQ_1_MSB
 *       DFE_TX_TXB_TXB_17_REG_FREQ_0_LSB
 *       DFE_TX_TXA_TXA_2_REG_FREQ_0_MSB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMixFreq(DfeFl_TxHandle hDfeTx, DfeFl_TxMixFreq *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_1, DFE_TX_TXA_TXA_1_REG_FREQ_0_LSB, arg->lower32);
    	CSL_FINS(hDfeTx->regs->txa_txa_2, DFE_TX_TXA_TXA_2_REG_FREQ_0_MSB, arg->upper16);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_3, DFE_TX_TXA_TXA_3_REG_FREQ_1_LSB, arg->lower32);
    	CSL_FINS(hDfeTx->regs->txa_txa_4, DFE_TX_TXA_TXA_4_REG_FREQ_1_MSB, arg->upper16);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_17, DFE_TX_TXB_TXB_17_REG_FREQ_0_LSB, arg->lower32);
    	CSL_FINS(hDfeTx->regs->txb_txb_18, DFE_TX_TXB_TXB_18_REG_FREQ_0_MSB, arg->upper16);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_19, DFE_TX_TXB_TXB_19_REG_FREQ_1_LSB, arg->lower32);
    	CSL_FINS(hDfeTx->regs->txb_txb_20, DFE_TX_TXB_TXB_20_REG_FREQ_1_MSB, arg->upper16);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMixFreq
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_1_REG_FREQ_0_LSB
 *       DFE_TX_TXB_TXB_19_REG_FREQ_1_LSB
 *       DFE_TX_TXA_TXA_3_REG_FREQ_1_LSB
 *       DFE_TX_TXB_TXB_20_REG_FREQ_1_MSB
 *       DFE_TX_TXB_TXB_18_REG_FREQ_0_MSB
 *       DFE_TX_TXA_TXA_4_REG_FREQ_1_MSB
 *       DFE_TX_TXB_TXB_17_REG_FREQ_0_LSB
 *       DFE_TX_TXA_TXA_2_REG_FREQ_0_MSB
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMixFreq(DfeFl_TxHandle hDfeTx, DfeFl_TxMixFreq *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	arg->lower32 = CSL_FEXT(hDfeTx->regs->txa_txa_1, DFE_TX_TXA_TXA_1_REG_FREQ_0_LSB);
    	arg->upper16 = CSL_FEXT(hDfeTx->regs->txa_txa_2, DFE_TX_TXA_TXA_2_REG_FREQ_0_MSB);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	arg->lower32 = CSL_FEXT(hDfeTx->regs->txa_txa_3, DFE_TX_TXA_TXA_3_REG_FREQ_1_LSB);
    	arg->upper16 = CSL_FEXT(hDfeTx->regs->txa_txa_4, DFE_TX_TXA_TXA_4_REG_FREQ_1_MSB);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	arg->lower32 = CSL_FEXT(hDfeTx->regs->txb_txb_17, DFE_TX_TXB_TXB_17_REG_FREQ_0_LSB);
    	arg->upper16 = CSL_FEXT(hDfeTx->regs->txb_txb_18, DFE_TX_TXB_TXB_18_REG_FREQ_0_MSB);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	arg->lower32 = CSL_FEXT(hDfeTx->regs->txb_txb_19, DFE_TX_TXB_TXB_19_REG_FREQ_1_LSB);
    	arg->upper16 = CSL_FEXT(hDfeTx->regs->txb_txb_20, DFE_TX_TXB_TXB_20_REG_FREQ_1_MSB);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMixPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_20_REG_PHASE_1
 *       DFE_TX_TXA_TXA_4_REG_PHASE_1
 *       DFE_TX_TXB_TXB_18_REG_PHASE_0
 *       DFE_TX_TXA_TXA_2_REG_PHASE_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMixPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_2, DFE_TX_TXA_TXA_2_REG_PHASE_0, arg->phase);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_4, DFE_TX_TXA_TXA_4_REG_PHASE_1, arg->phase);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_18, DFE_TX_TXB_TXB_18_REG_PHASE_0, arg->phase);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_20, DFE_TX_TXB_TXB_20_REG_PHASE_1, arg->phase);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMixPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_20_REG_PHASE_1
 *       DFE_TX_TXA_TXA_4_REG_PHASE_1
 *       DFE_TX_TXB_TXB_18_REG_PHASE_0
 *       DFE_TX_TXA_TXA_2_REG_PHASE_0
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMixPhase(DfeFl_TxHandle hDfeTx, DfeFl_TxPhase *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_2, DFE_TX_TXA_TXA_2_REG_PHASE_0);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txa_txa_4, DFE_TX_TXA_TXA_4_REG_PHASE_1);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_18, DFE_TX_TXB_TXB_18_REG_PHASE_0);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->phase = CSL_FEXT(hDfeTx->regs->txb_txb_20, DFE_TX_TXB_TXB_20_REG_PHASE_1);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMixDitEn
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_DIT_EN
 *       DFE_TX_TXB_TXB_23_REG_DIT_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMixDitEn(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_DIT_EN, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_DIT_EN, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMixDitEn
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_DIT_EN
 *       DFE_TX_TXB_TXB_23_REG_DIT_EN
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMixDitEn(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_DIT_EN);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_DIT_EN);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMcMix
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_MC_MIXI
 *       DFE_TX_TXB_TXB_23_REG_MC_MIXI
 *       DFE_TX_TXA_TXA_7_REG_MC_MIXQ
 *       DFE_TX_TXB_TXB_23_REG_MC_MIXQ
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMcMix(DfeFl_TxHandle hDfeTx, DfeFl_TxMcMix *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_MC_MIXI, arg->mixi);
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_MC_MIXQ, arg->mixq);
    	break;
    }
    case DFE_FL_TXB:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_MC_MIXI, arg->mixi);
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_MC_MIXQ, arg->mixq);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMcMix
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_7_REG_MC_MIXI
 *       DFE_TX_TXB_TXB_23_REG_MC_MIXI
 *       DFE_TX_TXA_TXA_7_REG_MC_MIXQ
 *       DFE_TX_TXB_TXB_23_REG_MC_MIXQ
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMcMix(DfeFl_TxHandle hDfeTx, DfeFl_TxMcMix *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	arg->mixi = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_MC_MIXI);
    	arg->mixq = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_MC_MIXQ);
    	break;
    }
    case DFE_FL_TXB:
    {
    	arg->mixi = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_MC_MIXI);
    	arg->mixq = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_MC_MIXQ);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMcRsmp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPAA
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPAB
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPBA
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPBB
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPAA
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPAB
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPBA
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPBB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMcRsmp(DfeFl_TxHandle hDfeTx, DfeFl_TxMcRsmp *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPAA, arg->rsmpa);
    	CSL_FINS(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPAB, arg->rsmpb);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPBA, arg->rsmpa);
    	CSL_FINS(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPBB, arg->rsmpb);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPAA, arg->rsmpa);
    	CSL_FINS(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPAB, arg->rsmpb);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPBA, arg->rsmpa);
    	CSL_FINS(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPBB, arg->rsmpb);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMcRsmp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPAA
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPAB
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPBA
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPBB
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPAA
 *       DFE_TX_TXA_TXA_8_REG_MC_RSMPAB
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPBA
 *       DFE_TX_TXB_TXB_24_REG_MC_RSMPBB
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMcRsmp(DfeFl_TxHandle hDfeTx, DfeFl_TxMcRsmp *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	arg->rsmpa = CSL_FEXT(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPAA);
    	arg->rsmpb = CSL_FEXT(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPAB);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	arg->rsmpa = CSL_FEXT(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPBA);
    	arg->rsmpb = CSL_FEXT(hDfeTx->regs->txa_txa_8, DFE_TX_TXA_TXA_8_REG_MC_RSMPBB);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	arg->rsmpa = CSL_FEXT(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPAA);
    	arg->rsmpb = CSL_FEXT(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPAB);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	arg->rsmpa = CSL_FEXT(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPBA);
    	arg->rsmpb = CSL_FEXT(hDfeTx->regs->txb_txb_24, DFE_TX_TXB_TXB_24_REG_MC_RSMPBB);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetMcBuc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCIL
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCQE
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCIE
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCQL
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCQE
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCIL
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCQL
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCIE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetMcBuc(DfeFl_TxHandle hDfeTx, DfeFl_TxMcBuc *arg)
{
    uint32_t data;

	switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	data = hDfeTx->regs->txa_txa_8;
    	CSL_FINS(data, DFE_TX_TXA_TXA_8_REG_MC_BUCIE, arg->bucie);
    	CSL_FINS(data, DFE_TX_TXA_TXA_8_REG_MC_BUCIL, arg->bucil);
    	CSL_FINS(data, DFE_TX_TXA_TXA_8_REG_MC_BUCQE, arg->bucqe);
    	CSL_FINS(data, DFE_TX_TXA_TXA_8_REG_MC_BUCQL, arg->bucql);
    	hDfeTx->regs->txa_txa_8 = data;
    	break;
    }
    case DFE_FL_TXB:
    {
    	data = hDfeTx->regs->txb_txb_24;
    	CSL_FINS(data, DFE_TX_TXB_TXB_24_REG_MC_BUCIE, arg->bucie);
    	CSL_FINS(data, DFE_TX_TXB_TXB_24_REG_MC_BUCIL, arg->bucil);
    	CSL_FINS(data, DFE_TX_TXB_TXB_24_REG_MC_BUCQE, arg->bucqe);
    	CSL_FINS(data, DFE_TX_TXB_TXB_24_REG_MC_BUCQL, arg->bucql);
    	hDfeTx->regs->txb_txb_24 = data;
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetMcBuc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCIL
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCQE
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCIE
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCQL
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCQE
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCIL
 *       DFE_TX_TXB_TXB_24_REG_MC_BUCQL
 *       DFE_TX_TXA_TXA_8_REG_MC_BUCIE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetMcBuc(DfeFl_TxHandle hDfeTx, DfeFl_TxMcBuc *arg)
{
    uint32_t data;

	switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	data = hDfeTx->regs->txa_txa_8;
    	arg->bucie = CSL_FEXT(data, DFE_TX_TXA_TXA_8_REG_MC_BUCIE);
    	arg->bucil = CSL_FEXT(data, DFE_TX_TXA_TXA_8_REG_MC_BUCIL);
    	arg->bucqe = CSL_FEXT(data, DFE_TX_TXA_TXA_8_REG_MC_BUCQE);
    	arg->bucql = CSL_FEXT(data, DFE_TX_TXA_TXA_8_REG_MC_BUCQL);
    	hDfeTx->regs->txa_txa_8 = data;
    	break;
    }
    case DFE_FL_TXB:
    {
    	data = hDfeTx->regs->txb_txb_24;
    	arg->bucie = CSL_FEXT(data, DFE_TX_TXB_TXB_24_REG_MC_BUCIE);
    	arg->bucil = CSL_FEXT(data, DFE_TX_TXB_TXB_24_REG_MC_BUCIL);
    	arg->bucqe = CSL_FEXT(data, DFE_TX_TXB_TXB_24_REG_MC_BUCQE);
    	arg->bucql = CSL_FEXT(data, DFE_TX_TXB_TXB_24_REG_MC_BUCQL);
    	hDfeTx->regs->txb_txb_24 = data;
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetCcRsmp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_RSB
 *       DFE_TX_TXA_TXA_11_REG_CC_RSB
 *       DFE_TX_TXA_TXA_11_REG_CC_RSA
 *       DFE_TX_TXB_TXB_27_REG_CC_RSA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetCcRsmp(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_RSA, arg->data);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_RSB, arg->data);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_RSA, arg->data);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_RSB, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetCcRsmp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_RSB
 *       DFE_TX_TXA_TXA_11_REG_CC_RSB
 *       DFE_TX_TXA_TXA_11_REG_CC_RSA
 *       DFE_TX_TXB_TXB_27_REG_CC_RSA
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetCcRsmp(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_RSA);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_RSB);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_RSA);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_RSB);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetCcBuc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_CC_BUCB
 *       DFE_TX_TXA_TXA_11_REG_CC_BUCA
 *       DFE_TX_TXB_TXB_27_REG_CC_BUCB
 *       DFE_TX_TXB_TXB_27_REG_CC_BUCA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetCcBuc(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_BUCA, arg->data);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_BUCB, arg->data);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_BUCA, arg->data);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_BUCB, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetCcBuc
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_CC_BUCB
 *       DFE_TX_TXA_TXA_11_REG_CC_BUCA
 *       DFE_TX_TXB_TXB_27_REG_CC_BUCB
 *       DFE_TX_TXB_TXB_27_REG_CC_BUCA
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetCcBuc(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_BUCA);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_BUCB);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_BUCA);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_BUCB);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetCcMix
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_MIX
 *       DFE_TX_TXA_TXA_11_REG_CC_MIX
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetCcMix(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_MIX, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_MIX, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetCcMix
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_MIX
 *       DFE_TX_TXA_TXA_11_REG_CC_MIX
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetCcMix(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_MIX);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_MIX);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetCcPa
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_PA
 *       DFE_TX_TXA_TXA_11_REG_CC_PA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetCcPa(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_PA, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_PA, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetCcPa
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_27_REG_CC_PA
 *       DFE_TX_TXA_TXA_11_REG_CC_PA
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetCcPa(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_CC_PA);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_CC_PA);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetDCoffSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_23_REG_SSEL_DCOFF
 *       DFE_TX_TXA_TXA_7_REG_SSEL_DCOFF
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetDCoffSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_DCOFF, arg->ssel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_DCOFF, arg->ssel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetDCoffSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_23_REG_SSEL_DCOFF
 *       DFE_TX_TXA_TXA_7_REG_SSEL_DCOFF
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetDCoffSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txa_txa_7, DFE_TX_TXA_TXA_7_REG_SSEL_DCOFF);
    	break;
    case DFE_FL_TXB:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txb_txb_23, DFE_TX_TXB_TXB_23_REG_SSEL_DCOFF);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetDcOffsets
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AI
 *       DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AQ
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetDcOffsets(DfeFl_TxHandle hDfeTx, DfeFl_TxDcOffsets *arg)
{
    volatile uint32_t *regs;

	switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
        regs = &hDfeTx->regs->txa_txa_9;
    	break;
    case DFE_FL_TXA_PATHB:
        regs = &hDfeTx->regs->txa_txa_10;
    	break;
    case DFE_FL_TXB_PATHA:
        regs = &hDfeTx->regs->txb_txb_25;
    	break;
    case DFE_FL_TXB_PATHB:
        regs = &hDfeTx->regs->txb_txb_26;
    	break;
    default:
    	return;
    }

    regs[0] = CSL_FMK(DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AI, arg->dcOffsetI)
            | CSL_FMK(DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AQ, arg->dcOffsetQ);

}

/** ============================================================================
 *   @n@b dfeFl_TxGetDcOffsets
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AI
 *       DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AQ
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetDcOffsets(DfeFl_TxHandle hDfeTx, DfeFl_TxDcOffsets *arg)
{
    volatile uint32_t *regs;

	switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
        regs = &hDfeTx->regs->txa_txa_9;
    	break;
    case DFE_FL_TXA_PATHB:
        regs = &hDfeTx->regs->txa_txa_10;
    	break;
    case DFE_FL_TXB_PATHA:
        regs = &hDfeTx->regs->txb_txb_25;
    	break;
    case DFE_FL_TXB_PATHB:
        regs = &hDfeTx->regs->txb_txb_26;
    	break;
    default:
    	return;
    }

	arg->dcOffsetI = CSL_FEXT(regs[0], DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AI);
	arg->dcOffsetQ = CSL_FEXT(regs[0], DFE_TX_TXA_TXA_9_REG_DC_OFFSET_AQ);

}

/** ============================================================================
 *   @n@b dfeFl_TxSetAddbEn
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_ADDB_EN
 *       DFE_TX_TXB_TXB_27_REG_ADDB_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetAddbEn(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_ADDB_EN, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_ADDB_EN, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetAddbEn
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_ADDB_EN
 *       DFE_TX_TXB_TXB_27_REG_ADDB_EN
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetAddbEn(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_ADDB_EN);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_ADDB_EN);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_32_REG_BUCA_COEF
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucCoeff(DfeFl_TxHandle hDfeTx, uint32_t txDev, uint32_t idx, uint32_t coeff)
{
	volatile uint32_t *regs;

	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
		regs = &hDfeTx->regs->txa_txa_32[0];
		break;
	case DFE_FL_TXA_PATHB:
		regs = &hDfeTx->regs->txa_txa_48[0];
		break;
	case DFE_FL_TXB_PATHA:
		regs = &hDfeTx->regs->txb_txb_64[0];
		break;
	case DFE_FL_TXB_PATHB:
		regs = &hDfeTx->regs->txb_txb_80[0];
		break;
    default:
    	return;
	}
	regs[idx] = CSL_FMK(DFE_TX_TXA_TXA_32_REG_BUCA_COEF, coeff);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_32_REG_BUCA_COEF
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucCoeff(DfeFl_TxHandle hDfeTx, uint32_t txDev, uint32_t idx, uint32_t *coeff)
{
	volatile uint32_t *regs;

	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
		regs = &hDfeTx->regs->txa_txa_32[0];
		break;
	case DFE_FL_TXA_PATHB:
		regs = &hDfeTx->regs->txa_txa_48[0];
		break;
	case DFE_FL_TXB_PATHA:
		regs = &hDfeTx->regs->txb_txb_64[0];
		break;
	case DFE_FL_TXB_PATHB:
		regs = &hDfeTx->regs->txb_txb_80[0];
		break;
    default:
    	return;
	}
	*coeff = CSL_FEXT(regs[idx], DFE_TX_TXA_TXA_32_REG_BUCA_COEF);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucTwoAnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_TWO_ANT
 *       DFE_TX_TXA_TXA_47_REG_TWO_ANT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucTwoAnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_TWO_ANT, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_TWO_ANT, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucTwoAnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_TWO_ANT
 *       DFE_TX_TXA_TXA_47_REG_TWO_ANT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucTwoAnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_TWO_ANT);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_TWO_ANT);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucCascade
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_CASCADE
 *       DFE_TX_TXB_TXB_79_REG_CASCADE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucCascade(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_CASCADE, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_CASCADE, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucCascade
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_CASCADE
 *       DFE_TX_TXB_TXB_79_REG_CASCADE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucCascade(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_CASCADE);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_CASCADE);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucEven
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_EVEN_A
 *       DFE_TX_TXB_TXB_79_REG_EVEN_A
 *       DFE_TX_TXB_TXB_79_REG_EVEN_B
 *       DFE_TX_TXA_TXA_47_REG_EVEN_B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucEven(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_EVEN_A, arg->data);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_EVEN_B, arg->data);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_EVEN_A, arg->data);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_EVEN_B, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucEven
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_EVEN_A
 *       DFE_TX_TXB_TXB_79_REG_EVEN_A
 *       DFE_TX_TXB_TXB_79_REG_EVEN_B
 *       DFE_TX_TXA_TXA_47_REG_EVEN_B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucEven(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_EVEN_A);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_EVEN_B);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_EVEN_A);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_EVEN_B);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucInterp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_INTERP
 *       DFE_TX_TXB_TXB_79_REG_INTERP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucInterp(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_INTERP, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_INTERP, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucInterp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_47_REG_INTERP
 *       DFE_TX_TXB_TXB_79_REG_INTERP
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucInterp(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_INTERP);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_INTERP);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucAddMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_ADD_MUX_A
 *       DFE_TX_TXA_TXA_47_REG_ADD_MUX_A
 *       DFE_TX_TXA_TXA_47_REG_ADD_MUX_B
 *       DFE_TX_TXB_TXB_79_REG_ADD_MUX_B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucAddMux(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_ADD_MUX_A, arg->data);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_ADD_MUX_B, arg->data);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_ADD_MUX_A, arg->data);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_ADD_MUX_B, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucAddMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_ADD_MUX_A
 *       DFE_TX_TXA_TXA_47_REG_ADD_MUX_A
 *       DFE_TX_TXA_TXA_47_REG_ADD_MUX_B
 *       DFE_TX_TXB_TXB_79_REG_ADD_MUX_B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucAddMux(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_ADD_MUX_A);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_ADD_MUX_B);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_ADD_MUX_A);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_ADD_MUX_B);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucBPathEnable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_B_PATH_ENABLE
 *       DFE_TX_TXA_TXA_47_REG_B_PATH_ENABLE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucBPathEnable(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_B_PATH_ENABLE, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_B_PATH_ENABLE, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucBPathEnable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_B_PATH_ENABLE
 *       DFE_TX_TXA_TXA_47_REG_B_PATH_ENABLE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucBPathEnable(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_47, DFE_TX_TXA_TXA_47_REG_B_PATH_ENABLE);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_79, DFE_TX_TXB_TXB_79_REG_B_PATH_ENABLE);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetBucInMuxCtl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_BB_CTL
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_BA_CTL
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_AB_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_BA_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_BB_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_AB_CTL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetBucInMuxCtl(DfeFl_TxHandle hDfeTx, DfeFl_TxBucInMuxCtl *arg)
{
    uint32_t data;
	switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	data = hDfeTx->regs->txa_txa_47;
    	CSL_FINS(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_AB_CTL, arg->in_mux_ab);
    	CSL_FINS(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_BA_CTL, arg->in_mux_ba);
    	CSL_FINS(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_BB_CTL, arg->in_mux_bb);
    	hDfeTx->regs->txa_txa_47 = data;
    	break;
    }
    case DFE_FL_TXB:
    {
    	data = hDfeTx->regs->txb_txb_79;
    	CSL_FINS(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_AB_CTL, arg->in_mux_ab);
    	CSL_FINS(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_BA_CTL, arg->in_mux_ba);
    	CSL_FINS(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_BB_CTL, arg->in_mux_bb);
    	hDfeTx->regs->txb_txb_79 = data;
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetBucInMuxCtl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_BB_CTL
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_BA_CTL
 *       DFE_TX_TXB_TXB_79_REG_IN_MUX_AB_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_BA_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_BB_CTL
 *       DFE_TX_TXA_TXA_47_REG_IN_MUX_AB_CTL
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetBucInMuxCtl(DfeFl_TxHandle hDfeTx, DfeFl_TxBucInMuxCtl *arg)
{
    uint32_t data;
	switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	data = hDfeTx->regs->txa_txa_47;
    	arg->in_mux_ab = CSL_FEXT(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_AB_CTL);
    	arg->in_mux_ba = CSL_FEXT(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_BA_CTL);
    	arg->in_mux_bb = CSL_FEXT(data, DFE_TX_TXA_TXA_47_REG_IN_MUX_BB_CTL);

    	break;
    }
    case DFE_FL_TXB:
    {
    	data = hDfeTx->regs->txb_txb_79;
    	arg->in_mux_ab = CSL_FEXT(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_AB_CTL);
    	arg->in_mux_ba = CSL_FEXT(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_BA_CTL);
    	arg->in_mux_bb = CSL_FEXT(data, DFE_TX_TXB_TXB_79_REG_IN_MUX_BB_CTL);

    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaTwoAntMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_672_REG_TWO_ANT_MODE
 *       DFE_TX_TXA_TXA_640_REG_TWO_ANT_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaTwoAntMode(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_TWO_ANT_MODE, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_TWO_ANT_MODE, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaTwoAntMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_672_REG_TWO_ANT_MODE
 *       DFE_TX_TXA_TXA_640_REG_TWO_ANT_MODE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaTwoAntMode(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_TWO_ANT_MODE);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_TWO_ANT_MODE);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaFactoryTest
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_640_REG_FACTORY_TEST_FORCE_A_OUTPUT_0
 *       DFE_TX_TXB_TXB_673_REG_FACTORY_TEST_FORCE_B_OUTPUT_0
 *       DFE_TX_TXB_TXB_672_REG_FACTORY_TEST_FORCE_A_OUTPUT_0
 *       DFE_TX_TXA_TXA_641_REG_FACTORY_TEST_FORCE_B_OUTPUT_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaFactoryTest(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_FACTORY_TEST_FORCE_A_OUTPUT_0, arg->data);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_FACTORY_TEST_FORCE_B_OUTPUT_0, arg->data);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_FACTORY_TEST_FORCE_A_OUTPUT_0, arg->data);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_FACTORY_TEST_FORCE_B_OUTPUT_0, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaFactoryTest
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_640_REG_FACTORY_TEST_FORCE_A_OUTPUT_0
 *       DFE_TX_TXB_TXB_673_REG_FACTORY_TEST_FORCE_B_OUTPUT_0
 *       DFE_TX_TXB_TXB_672_REG_FACTORY_TEST_FORCE_A_OUTPUT_0
 *       DFE_TX_TXA_TXA_641_REG_FACTORY_TEST_FORCE_B_OUTPUT_0
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaFactoryTest(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_FACTORY_TEST_FORCE_A_OUTPUT_0);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_FACTORY_TEST_FORCE_B_OUTPUT_0);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_FACTORY_TEST_FORCE_A_OUTPUT_0);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_FACTORY_TEST_FORCE_B_OUTPUT_0);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_SSEL_PA
 *       DFE_TX_TXB_TXB_27_REG_SSEL_PA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_SSEL_PA, arg->ssel);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_SSEL_PA, arg->ssel);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_11_REG_SSEL_PA
 *       DFE_TX_TXB_TXB_27_REG_SSEL_PA
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaSsel(DfeFl_TxHandle hDfeTx, DfeFl_TxSsel *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txa_txa_11, DFE_TX_TXA_TXA_11_REG_SSEL_PA);
    	break;
    case DFE_FL_TXB:
    	arg->ssel = CSL_FEXT(hDfeTx->regs->txb_txb_27, DFE_TX_TXB_TXB_27_REG_SSEL_PA);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_640_REG_THRESHOLD_A
 *       DFE_TX_TXB_TXB_673_REG_THRESHOLD_B
 *       DFE_TX_TXA_TXA_641_REG_THRESHOLD_B
 *       DFE_TX_TXB_TXB_672_REG_THRESHOLD_A
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THRESHOLD_A, arg->theshld);
    	break;
    case DFE_FL_TXA_PATHB:
    	CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THRESHOLD_B, arg->theshld);
    	break;
    case DFE_FL_TXB_PATHA:
    	CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THRESHOLD_A, arg->theshld);
    	break;
    case DFE_FL_TXB_PATHB:
    	CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THRESHOLD_B, arg->theshld);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_640_REG_THRESHOLD_A
 *       DFE_TX_TXB_TXB_673_REG_THRESHOLD_B
 *       DFE_TX_TXA_TXA_641_REG_THRESHOLD_B
 *       DFE_TX_TXB_TXB_672_REG_THRESHOLD_A
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    	arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THRESHOLD_A);
    	break;
    case DFE_FL_TXA_PATHB:
    	arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THRESHOLD_B);
    	break;
    case DFE_FL_TXB_PATHA:
    	arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THRESHOLD_A);
    	break;
    case DFE_FL_TXB_PATHB:
    	arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THRESHOLD_B);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaIirMu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_672_REG_MU_1_A
 *       DFE_TX_TXB_TXB_673_REG_MU_0_B
 *       DFE_TX_TXA_TXA_641_REG_MU_1_B
 *       DFE_TX_TXA_TXA_641_REG_MU_0_B
 *       DFE_TX_TXB_TXB_673_REG_MU_1_B
 *       DFE_TX_TXA_TXA_640_REG_MU_0_A
 *       DFE_TX_TXA_TXA_640_REG_MU_1_A
 *       DFE_TX_TXB_TXB_672_REG_MU_0_A
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaIirMu(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirMu *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_MU_0_A, arg->mu0);
    	CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_MU_1_A, arg->mu1);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_MU_0_B, arg->mu0);
    	CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_MU_1_B, arg->mu1);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_MU_0_A, arg->mu0);
    	CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_MU_1_A, arg->mu1);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_MU_0_B, arg->mu0);
    	CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_MU_1_B, arg->mu1);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaIirMu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_672_REG_MU_1_A
 *       DFE_TX_TXB_TXB_673_REG_MU_0_B
 *       DFE_TX_TXA_TXA_641_REG_MU_1_B
 *       DFE_TX_TXA_TXA_641_REG_MU_0_B
 *       DFE_TX_TXB_TXB_673_REG_MU_1_B
 *       DFE_TX_TXA_TXA_640_REG_MU_0_A
 *       DFE_TX_TXA_TXA_640_REG_MU_1_A
 *       DFE_TX_TXB_TXB_672_REG_MU_0_A
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaIirMu(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirMu *arg)
{
    switch(arg->txDev)
    {
    case DFE_FL_TXA_PATHA:
    {
    	arg->mu0 = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_MU_0_A);
    	arg->mu1 = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_MU_1_A);
    	break;
    }
    case DFE_FL_TXA_PATHB:
    {
    	arg->mu0 = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_MU_0_B);
    	arg->mu1 = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_MU_1_B);
    	break;
    }
    case DFE_FL_TXB_PATHA:
    {
    	arg->mu0 = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_MU_0_A);
    	arg->mu1 = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_MU_1_A);
    	break;
    }
    case DFE_FL_TXB_PATHB:
    {
    	arg->mu0 = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_MU_0_B);
    	arg->mu1 = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_MU_1_B);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaIirThSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_673_REG_THB6_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA2_SEL
 *       DFE_TX_TXB_TXB_673_REG_THB2_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA2_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA1_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA6_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB2_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA6_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB6_SEL
 *       DFE_TX_TXB_TXB_673_REG_THB1_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA1_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB1_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaIirThSel(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirThSel *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA1_SEL, arg->th1Sel);
		CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA2_SEL, arg->th2Sel);
		CSL_FINS(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA6_SEL, arg->th6Sel);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB1_SEL, arg->th1Sel);
		CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB2_SEL, arg->th2Sel);
		CSL_FINS(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB6_SEL, arg->th6Sel);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA1_SEL, arg->th1Sel);
		CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA2_SEL, arg->th2Sel);
		CSL_FINS(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA6_SEL, arg->th6Sel);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB1_SEL, arg->th1Sel);
		CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB2_SEL, arg->th2Sel);
		CSL_FINS(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB6_SEL, arg->th6Sel);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaIirThSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_673_REG_THB6_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA2_SEL
 *       DFE_TX_TXB_TXB_673_REG_THB2_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA2_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA1_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA6_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB2_SEL
 *       DFE_TX_TXB_TXB_672_REG_THA6_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB6_SEL
 *       DFE_TX_TXB_TXB_673_REG_THB1_SEL
 *       DFE_TX_TXA_TXA_640_REG_THA1_SEL
 *       DFE_TX_TXA_TXA_641_REG_THB1_SEL
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaIirThSel(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirThSel *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		arg->th1Sel = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA1_SEL);
		arg->th2Sel = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA2_SEL);
		arg->th6Sel = CSL_FEXT(hDfeTx->regs->txa_txa_640, DFE_TX_TXA_TXA_640_REG_THA6_SEL);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		arg->th1Sel = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB1_SEL);
		arg->th2Sel = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB2_SEL);
		arg->th6Sel = CSL_FEXT(hDfeTx->regs->txa_txa_641, DFE_TX_TXA_TXA_641_REG_THB6_SEL);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		arg->th1Sel = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA1_SEL);
		arg->th2Sel = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA2_SEL);
		arg->th6Sel = CSL_FEXT(hDfeTx->regs->txb_txb_672, DFE_TX_TXB_TXB_672_REG_THA6_SEL);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		arg->th1Sel = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB1_SEL);
		arg->th2Sel = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB2_SEL);
		arg->th6Sel = CSL_FEXT(hDfeTx->regs->txb_txb_673, DFE_TX_TXB_TXB_673_REG_THB6_SEL);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaIirThVal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_647_REG_TH_4_A
 *       DFE_TX_TXA_TXA_647_REG_TH_4_B
 *       DFE_TX_TXB_TXB_679_REG_TH_4_A
 *       DFE_TX_TXB_TXB_679_REG_TH_4_B
 *       DFE_TX_TXB_TXB_678_REG_TH_2_A
 *       DFE_TX_TXB_TXB_678_REG_TH_2_B
 *       DFE_TX_TXA_TXA_646_REG_TH_2_A
 *       DFE_TX_TXA_TXA_646_REG_TH_2_B
 *       DFE_TX_TXA_TXA_645_REG_TH_1_B
 *       DFE_TX_TXB_TXB_677_REG_TH_1_B
 *       DFE_TX_TXB_TXB_677_REG_TH_1_A
 *       DFE_TX_TXA_TXA_645_REG_TH_1_A
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaIirThVal(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirThVal *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_645, DFE_TX_TXA_TXA_645_REG_TH_1_A, arg->TH1);
		CSL_FINS(hDfeTx->regs->txa_txa_646, DFE_TX_TXA_TXA_646_REG_TH_2_A, arg->TH2);
		CSL_FINS(hDfeTx->regs->txa_txa_647, DFE_TX_TXA_TXA_647_REG_TH_4_A, arg->TH4);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_645, DFE_TX_TXA_TXA_645_REG_TH_1_B, arg->TH1);
		CSL_FINS(hDfeTx->regs->txa_txa_646, DFE_TX_TXA_TXA_646_REG_TH_2_B, arg->TH2);
		CSL_FINS(hDfeTx->regs->txa_txa_647, DFE_TX_TXA_TXA_647_REG_TH_4_B, arg->TH4);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_677, DFE_TX_TXB_TXB_677_REG_TH_1_A, arg->TH1);
		CSL_FINS(hDfeTx->regs->txb_txb_678, DFE_TX_TXB_TXB_678_REG_TH_2_A, arg->TH2);
		CSL_FINS(hDfeTx->regs->txb_txb_679, DFE_TX_TXB_TXB_679_REG_TH_4_A, arg->TH4);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_677, DFE_TX_TXB_TXB_677_REG_TH_1_B, arg->TH1);
		CSL_FINS(hDfeTx->regs->txb_txb_678, DFE_TX_TXB_TXB_678_REG_TH_2_B, arg->TH2);
		CSL_FINS(hDfeTx->regs->txb_txb_679, DFE_TX_TXB_TXB_679_REG_TH_4_B, arg->TH4);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaIirThVal
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_647_REG_TH_4_A
 *       DFE_TX_TXA_TXA_647_REG_TH_4_B
 *       DFE_TX_TXB_TXB_679_REG_TH_4_A
 *       DFE_TX_TXB_TXB_679_REG_TH_4_B
 *       DFE_TX_TXB_TXB_678_REG_TH_2_A
 *       DFE_TX_TXB_TXB_678_REG_TH_2_B
 *       DFE_TX_TXA_TXA_646_REG_TH_2_A
 *       DFE_TX_TXA_TXA_646_REG_TH_2_B
 *       DFE_TX_TXA_TXA_645_REG_TH_1_B
 *       DFE_TX_TXB_TXB_677_REG_TH_1_B
 *       DFE_TX_TXB_TXB_677_REG_TH_1_A
 *       DFE_TX_TXA_TXA_645_REG_TH_1_A
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaIirThVal(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIirThVal *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		arg->TH1 = CSL_FEXT(hDfeTx->regs->txa_txa_645, DFE_TX_TXA_TXA_645_REG_TH_1_A);
		arg->TH2 = CSL_FEXT(hDfeTx->regs->txa_txa_646, DFE_TX_TXA_TXA_646_REG_TH_2_A);
		arg->TH4 = CSL_FEXT(hDfeTx->regs->txa_txa_647, DFE_TX_TXA_TXA_647_REG_TH_4_A);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		arg->TH1 = CSL_FEXT(hDfeTx->regs->txa_txa_645, DFE_TX_TXA_TXA_645_REG_TH_1_B);
		arg->TH2 = CSL_FEXT(hDfeTx->regs->txa_txa_646, DFE_TX_TXA_TXA_646_REG_TH_2_B);
		arg->TH4 = CSL_FEXT(hDfeTx->regs->txa_txa_647, DFE_TX_TXA_TXA_647_REG_TH_4_B);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		arg->TH1 = CSL_FEXT(hDfeTx->regs->txb_txb_677, DFE_TX_TXB_TXB_677_REG_TH_1_A);
		arg->TH2 = CSL_FEXT(hDfeTx->regs->txb_txb_678, DFE_TX_TXB_TXB_678_REG_TH_2_A);
		arg->TH4 = CSL_FEXT(hDfeTx->regs->txb_txb_679, DFE_TX_TXB_TXB_679_REG_TH_4_A);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		arg->TH1 = CSL_FEXT(hDfeTx->regs->txb_txb_677, DFE_TX_TXB_TXB_677_REG_TH_1_B);
		arg->TH2 = CSL_FEXT(hDfeTx->regs->txb_txb_678, DFE_TX_TXB_TXB_678_REG_TH_2_B);
		arg->TH4 = CSL_FEXT(hDfeTx->regs->txb_txb_679, DFE_TX_TXB_TXB_679_REG_TH_4_B);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaCcTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_674_REG_CCA_THR
 *       DFE_TX_TXA_TXA_643_REG_CCB_THR
 *       DFE_TX_TXB_TXB_675_REG_CCB_THR
 *       DFE_TX_TXA_TXA_642_REG_CCA_THR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaCcTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_642, DFE_TX_TXA_TXA_642_REG_CCA_THR, arg->theshld);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_643, DFE_TX_TXA_TXA_643_REG_CCB_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_674, DFE_TX_TXB_TXB_674_REG_CCA_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_675, DFE_TX_TXB_TXB_675_REG_CCB_THR, arg->theshld);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaCcTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_674_REG_CCA_THR
 *       DFE_TX_TXA_TXA_643_REG_CCB_THR
 *       DFE_TX_TXB_TXB_675_REG_CCB_THR
 *       DFE_TX_TXA_TXA_642_REG_CCA_THR
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaCcTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_642, DFE_TX_TXA_TXA_642_REG_CCA_THR);
		break;
	case DFE_FL_TXA_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_643, DFE_TX_TXA_TXA_643_REG_CCB_THR);
		break;
	case DFE_FL_TXB_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_674, DFE_TX_TXB_TXB_674_REG_CCA_THR);
		break;
	case DFE_FL_TXB_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_675, DFE_TX_TXB_TXB_675_REG_CCB_THR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaCcCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_642_REG_CCA_CNT
 *       DFE_TX_TXA_TXA_643_REG_CCB_CNT
 *       DFE_TX_TXB_TXB_675_REG_CCB_CNT
 *       DFE_TX_TXB_TXB_674_REG_CCA_CNT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaCcCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_642, DFE_TX_TXA_TXA_642_REG_CCA_CNT, arg->cnt);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_643, DFE_TX_TXA_TXA_643_REG_CCB_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_674, DFE_TX_TXB_TXB_674_REG_CCA_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_675, DFE_TX_TXB_TXB_675_REG_CCB_CNT, arg->cnt);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaCcCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_642_REG_CCA_CNT
 *       DFE_TX_TXA_TXA_643_REG_CCB_CNT
 *       DFE_TX_TXB_TXB_675_REG_CCB_CNT
 *       DFE_TX_TXB_TXB_674_REG_CCA_CNT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaCcCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_642, DFE_TX_TXA_TXA_642_REG_CCA_CNT);
		break;
	case DFE_FL_TXA_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_643, DFE_TX_TXA_TXA_643_REG_CCB_CNT);
		break;
	case DFE_FL_TXB_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_674, DFE_TX_TXB_TXB_674_REG_CCA_CNT);
		break;
	case DFE_FL_TXB_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_675, DFE_TX_TXB_TXB_675_REG_CCB_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaPeakTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_676_REG_TH_0_A
 *       DFE_TX_TXB_TXB_676_REG_TH_0_B
 *       DFE_TX_TXA_TXA_644_REG_TH_0_A
 *       DFE_TX_TXA_TXA_644_REG_TH_0_B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaPeakTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_644, DFE_TX_TXA_TXA_644_REG_TH_0_A, arg->theshld);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_644, DFE_TX_TXA_TXA_644_REG_TH_0_B, arg->theshld);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_676, DFE_TX_TXB_TXB_676_REG_TH_0_A, arg->theshld);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_676, DFE_TX_TXB_TXB_676_REG_TH_0_B, arg->theshld);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaPeakTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_676_REG_TH_0_A
 *       DFE_TX_TXB_TXB_676_REG_TH_0_B
 *       DFE_TX_TXA_TXA_644_REG_TH_0_A
 *       DFE_TX_TXA_TXA_644_REG_TH_0_B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaPeakTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_644, DFE_TX_TXA_TXA_644_REG_TH_0_A);
		break;
	case DFE_FL_TXA_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_644, DFE_TX_TXA_TXA_644_REG_TH_0_B);
		break;
	case DFE_FL_TXB_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_676, DFE_TX_TXB_TXB_676_REG_TH_0_A);
		break;
	case DFE_FL_TXB_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_676, DFE_TX_TXB_TXB_676_REG_TH_0_B);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaPeakCntTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_651_REG_PEAK_B_THR
 *       DFE_TX_TXB_TXB_682_REG_PEAK_A_THR
 *       DFE_TX_TXA_TXA_650_REG_PEAK_A_THR
 *       DFE_TX_TXB_TXB_683_REG_PEAK_B_THR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaPeakCntTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_650, DFE_TX_TXA_TXA_650_REG_PEAK_A_THR, arg->theshld);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_651, DFE_TX_TXA_TXA_651_REG_PEAK_B_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_682, DFE_TX_TXB_TXB_682_REG_PEAK_A_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_683, DFE_TX_TXB_TXB_683_REG_PEAK_B_THR, arg->theshld);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaPeakCntTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_651_REG_PEAK_B_THR
 *       DFE_TX_TXB_TXB_682_REG_PEAK_A_THR
 *       DFE_TX_TXA_TXA_650_REG_PEAK_A_THR
 *       DFE_TX_TXB_TXB_683_REG_PEAK_B_THR
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaPeakCntTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_650, DFE_TX_TXA_TXA_650_REG_PEAK_A_THR);
		break;
	case DFE_FL_TXA_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_651, DFE_TX_TXA_TXA_651_REG_PEAK_B_THR);
		break;
	case DFE_FL_TXB_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_682, DFE_TX_TXB_TXB_682_REG_PEAK_A_THR);
		break;
	case DFE_FL_TXB_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_683, DFE_TX_TXB_TXB_683_REG_PEAK_B_THR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaPeakCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_651_REG_PEAK_B_CNT
 *       DFE_TX_TXB_TXB_682_REG_PEAK_A_CNT
 *       DFE_TX_TXA_TXA_650_REG_PEAK_A_CNT
 *       DFE_TX_TXB_TXB_683_REG_PEAK_B_CNT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaPeakCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_650, DFE_TX_TXA_TXA_650_REG_PEAK_A_CNT, arg->cnt);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_651, DFE_TX_TXA_TXA_651_REG_PEAK_B_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_682, DFE_TX_TXB_TXB_682_REG_PEAK_A_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_683, DFE_TX_TXB_TXB_683_REG_PEAK_B_CNT, arg->cnt);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaPeakCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_651_REG_PEAK_B_CNT
 *       DFE_TX_TXB_TXB_682_REG_PEAK_A_CNT
 *       DFE_TX_TXA_TXA_650_REG_PEAK_A_CNT
 *       DFE_TX_TXB_TXB_683_REG_PEAK_B_CNT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaPeakCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_650, DFE_TX_TXA_TXA_650_REG_PEAK_A_CNT);
		break;
	case DFE_FL_TXA_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_651, DFE_TX_TXA_TXA_651_REG_PEAK_B_CNT);
		break;
	case DFE_FL_TXB_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_682, DFE_TX_TXB_TXB_682_REG_PEAK_A_CNT);
		break;
	case DFE_FL_TXB_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_683, DFE_TX_TXB_TXB_683_REG_PEAK_B_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaPeakGainTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_THR
 *       DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_THR
 *       DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_THR
 *       DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_THR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaPeakGainTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_652, DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_THR, arg->theshld);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_653, DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_684, DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_THR, arg->theshld);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_685, DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_THR, arg->theshld);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaPeakGainTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_THR
 *       DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_THR
 *       DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_THR
 *       DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_THR
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaPeakGainTh(DfeFl_TxHandle hDfeTx, DfeFl_TxPaTh *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_652, DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_THR);
		break;
	case DFE_FL_TXA_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txa_txa_653, DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_THR);
		break;
	case DFE_FL_TXB_PATHA:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_684, DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_THR);
		break;
	case DFE_FL_TXB_PATHB:
		arg->theshld = CSL_FEXT(hDfeTx->regs->txb_txb_685, DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_THR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaPeakGainCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_CNT
 *       DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_CNT
 *       DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_CNT
 *       DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_CNT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaPeakGainCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_652, DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_CNT, arg->cnt);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_653, DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_684, DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_CNT, arg->cnt);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_685, DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_CNT, arg->cnt);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaPeakGainCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_CNT
 *       DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_CNT
 *       DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_CNT
 *       DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_CNT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaPeakGainCnt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaCnt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_652, DFE_TX_TXA_TXA_652_REG_PEAKGAIN_A_CNT);
		break;
	case DFE_FL_TXA_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txa_txa_653, DFE_TX_TXA_TXA_653_REG_PEAKGAIN_B_CNT);
		break;
	case DFE_FL_TXB_PATHA:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_684, DFE_TX_TXB_TXB_684_REG_PEAKGAIN_A_CNT);
		break;
	case DFE_FL_TXB_PATHB:
		arg->cnt = CSL_FEXT(hDfeTx->regs->txb_txb_685, DFE_TX_TXB_TXB_685_REG_PEAKGAIN_B_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaMaxmagClr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_689_REG_MAX_MAGB
 *       DFE_TX_TXA_TXA_656_REG_MAX_MAGA
 *       DFE_TX_TXB_TXB_688_REG_MAX_MAGA
 *       DFE_TX_TXA_TXA_657_REG_MAX_MAGB
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaMaxmagClr(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_656, DFE_TX_TXA_TXA_656_REG_MAX_MAGA);
		break;
	case DFE_FL_TXA_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_657, DFE_TX_TXA_TXA_657_REG_MAX_MAGB);
		break;
	case DFE_FL_TXB_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_688, DFE_TX_TXB_TXB_688_REG_MAX_MAGA);
		break;
	case DFE_FL_TXB_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_689, DFE_TX_TXB_TXB_689_REG_MAX_MAGB);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaMaxmagNoClr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_656_REG_MAX_MAGB
 *       DFE_TX_TXB_TXB_689_REG_MAX_MAGA
 *       DFE_TX_TXA_TXA_657_REG_MAX_MAGA
 *       DFE_TX_TXB_TXB_688_REG_MAX_MAGB
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaMaxmagNoClr(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_657, DFE_TX_TXA_TXA_657_REG_MAX_MAGA);
		break;
	case DFE_FL_TXA_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_656, DFE_TX_TXA_TXA_656_REG_MAX_MAGB);
		break;
	case DFE_FL_TXB_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_689, DFE_TX_TXB_TXB_689_REG_MAX_MAGA);
		break;
	case DFE_FL_TXB_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_688, DFE_TX_TXB_TXB_688_REG_MAX_MAGB);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaD50
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_648_REG_D50B
 *       DFE_TX_TXA_TXA_648_REG_D50A
 *       DFE_TX_TXB_TXB_680_REG_D50A
 *       DFE_TX_TXB_TXB_680_REG_D50B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaD50(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_648, DFE_TX_TXA_TXA_648_REG_D50A);
		break;
	case DFE_FL_TXA_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_648, DFE_TX_TXA_TXA_648_REG_D50B);
		break;
	case DFE_FL_TXB_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_680, DFE_TX_TXB_TXB_680_REG_D50A);
		break;
	case DFE_FL_TXB_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_680, DFE_TX_TXB_TXB_680_REG_D50B);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaD51
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_649_REG_D51B
 *       DFE_TX_TXA_TXA_649_REG_D51A
 *       DFE_TX_TXB_TXB_681_REG_D51A
 *       DFE_TX_TXB_TXB_681_REG_D51B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaD51(DfeFl_TxHandle hDfeTx, DfeFl_TxDevData *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_649, DFE_TX_TXA_TXA_649_REG_D51A);
		break;
	case DFE_FL_TXA_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txa_txa_649, DFE_TX_TXA_TXA_649_REG_D51B);
		break;
	case DFE_FL_TXB_PATHA:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_681, DFE_TX_TXB_TXB_681_REG_D51A);
		break;
	case DFE_FL_TXB_PATHB:
		arg->data = CSL_FEXT(hDfeTx->regs->txb_txb_681, DFE_TX_TXB_TXB_681_REG_D51B);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaMask
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXB_TXB_686_REG_MASKA
 *       DFE_TX_TXB_TXB_686_REG_MASKB
 *       DFE_TX_TXA_TXA_654_REG_MASKA
 *       DFE_TX_TXA_TXA_654_REG_MASKB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaMask(DfeFl_TxHandle hDfeTx, DfeFl_TxPaMask *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		CSL_FINS(hDfeTx->regs->txa_txa_654, DFE_TX_TXA_TXA_654_REG_MASKA, arg->mask);
		break;
	case DFE_FL_TXA_PATHB:
		CSL_FINS(hDfeTx->regs->txa_txa_654, DFE_TX_TXA_TXA_654_REG_MASKB, arg->mask);
		break;
	case DFE_FL_TXB_PATHA:
		CSL_FINS(hDfeTx->regs->txb_txb_686, DFE_TX_TXB_TXB_686_REG_MASKA, arg->mask);
		break;
	case DFE_FL_TXB_PATHB:
		CSL_FINS(hDfeTx->regs->txb_txb_686, DFE_TX_TXB_TXB_686_REG_MASKB, arg->mask);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaMask
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXB_TXB_686_REG_MASKA
 *       DFE_TX_TXB_TXB_686_REG_MASKB
 *       DFE_TX_TXA_TXA_654_REG_MASKA
 *       DFE_TX_TXA_TXA_654_REG_MASKB
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaMask(DfeFl_TxHandle hDfeTx, DfeFl_TxPaMask *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
		arg->mask = CSL_FEXT(hDfeTx->regs->txa_txa_654, DFE_TX_TXA_TXA_654_REG_MASKA);
		break;
	case DFE_FL_TXA_PATHB:
		arg->mask = CSL_FEXT(hDfeTx->regs->txa_txa_654, DFE_TX_TXA_TXA_654_REG_MASKB);
		break;
	case DFE_FL_TXB_PATHA:
		arg->mask = CSL_FEXT(hDfeTx->regs->txb_txb_686, DFE_TX_TXB_TXB_686_REG_MASKA);
		break;
	case DFE_FL_TXB_PATHB:
		arg->mask = CSL_FEXT(hDfeTx->regs->txb_txb_686, DFE_TX_TXB_TXB_686_REG_MASKB);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaIntrpt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT6A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT6B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT6A
 *       DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_A
 *       DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT6B
 *       DFE_TX_TXB_TXB_687_REG_STOP_DPD_A
 *       DFE_TX_TXB_TXB_687_REG_STOP_DPD_B
 *       DFE_TX_TXA_TXA_655_REG_SHUTDOWN_A
 *       DFE_TX_TXA_TXA_655_REG_SHUTDOWN_B
 *       DFE_TX_TXB_TXB_687_REG_SHUTDOWN_B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT1A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT1B
 *       DFE_TX_TXB_TXB_687_REG_SHUTDOWN_A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT2A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT2B
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT3B
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT3A
 *       DFE_TX_TXA_TXA_655_REG_STOP_DPD_B
 *       DFE_TX_TXA_TXA_655_REG_STOP_DPD_A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT3B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT2A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT2B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT3A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT1A
 *       DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_B
 *       DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT1B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaIntrpt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIntrpt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_STOP_DPD_A, arg->stopDpd);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT1A, arg->intrpt1);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT2A, arg->intrpt2);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT3A, arg->intrpt3);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT6A, arg->intrpt6);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_SHUTDOWN_A, arg->shutdown);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_A, arg->lowerCfrGain);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_STOP_DPD_B, arg->stopDpd);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT1B, arg->intrpt1);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT2B, arg->intrpt2);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT3B, arg->intrpt3);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT6B, arg->intrpt6);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_SHUTDOWN_B, arg->shutdown);
		CSL_FINS(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_B, arg->lowerCfrGain);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_STOP_DPD_A, arg->stopDpd);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT1A, arg->intrpt1);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT2A, arg->intrpt2);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT3A, arg->intrpt3);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT6A, arg->intrpt6);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_SHUTDOWN_A, arg->shutdown);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_A, arg->lowerCfrGain);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_STOP_DPD_B, arg->stopDpd);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT1B, arg->intrpt1);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT2B, arg->intrpt2);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT3B, arg->intrpt3);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT6B, arg->intrpt6);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_SHUTDOWN_B, arg->shutdown);
		CSL_FINS(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_B, arg->lowerCfrGain);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryPaIntrpt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT6A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT6B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT6A
 *       DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_A
 *       DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT6B
 *       DFE_TX_TXB_TXB_687_REG_STOP_DPD_A
 *       DFE_TX_TXB_TXB_687_REG_STOP_DPD_B
 *       DFE_TX_TXA_TXA_655_REG_SHUTDOWN_A
 *       DFE_TX_TXA_TXA_655_REG_SHUTDOWN_B
 *       DFE_TX_TXB_TXB_687_REG_SHUTDOWN_B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT1A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT1B
 *       DFE_TX_TXB_TXB_687_REG_SHUTDOWN_A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT2A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT2B
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT3B
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT3A
 *       DFE_TX_TXA_TXA_655_REG_STOP_DPD_B
 *       DFE_TX_TXA_TXA_655_REG_STOP_DPD_A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT3B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT2A
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT2B
 *       DFE_TX_TXB_TXB_687_REG_INTERRUPT3A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT1A
 *       DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_B
 *       DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_A
 *       DFE_TX_TXA_TXA_655_REG_INTERRUPT1B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryPaIntrpt(DfeFl_TxHandle hDfeTx, DfeFl_TxPaIntrpt *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		arg->stopDpd = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_STOP_DPD_A);
		arg->intrpt1 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT1A);
		arg->intrpt2 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT2A);
		arg->intrpt3 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT3A);
		arg->intrpt6 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT6A);
		arg->shutdown = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_SHUTDOWN_A);
		arg->lowerCfrGain = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_A);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		arg->stopDpd = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_STOP_DPD_B);
		arg->intrpt1 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT1B);
		arg->intrpt2 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT2B);
		arg->intrpt3 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT3B);
		arg->intrpt6 = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_INTERRUPT6B);
		arg->shutdown = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_SHUTDOWN_B);
		arg->lowerCfrGain = CSL_FEXT(hDfeTx->regs->txa_txa_655, DFE_TX_TXA_TXA_655_REG_LOWER_CFR_GAIN_B);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		arg->stopDpd = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_STOP_DPD_A);
		arg->intrpt1 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT1A);
		arg->intrpt2 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT2A);
		arg->intrpt3 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT3A);
		arg->intrpt6 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT6A);
		arg->shutdown = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_SHUTDOWN_A);
		arg->lowerCfrGain = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_A);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		arg->stopDpd = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_STOP_DPD_B);
		arg->intrpt1 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT1B);
		arg->intrpt2 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT2B);
		arg->intrpt3 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT3B);
		arg->intrpt6 = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_INTERRUPT6B);
		arg->shutdown = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_SHUTDOWN_B);
		arg->lowerCfrGain = CSL_FEXT(hDfeTx->regs->txb_txb_687, DFE_TX_TXB_TXB_687_REG_LOWER_CFR_GAIN_B);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaLutP
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65536_REG_PAA_LUT_P
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaLutP(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65536[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_65920[0];
		break;
	default:
		return;
	}
	regs[idx] = CSL_FMK(DFE_TX_TXA_TXA_65536_REG_PAA_LUT_P, coeff);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaLutP
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65536_REG_PAA_LUT_P
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaLutP(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t *coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65536[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_65920[0];
		break;
	default:
		return;
	}
	*coeff = CSL_FEXT(regs[idx], DFE_TX_TXA_TXA_65536_REG_PAA_LUT_P);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaLutQ
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65664_REG_PAA_LUT_Q
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaLutQ(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65664[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_66048[0];
		break;
	default:
		return;
	}
	regs[idx] = CSL_FMK(DFE_TX_TXA_TXA_65664_REG_PAA_LUT_Q, coeff);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaLutQ
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65664_REG_PAA_LUT_Q
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaLutQ(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t *coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65664[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_66048[0];
		break;
	default:
		return;
	}
	*coeff = CSL_FEXT(regs[idx], DFE_TX_TXA_TXA_65664_REG_PAA_LUT_Q);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetPaLutT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65792_REG_PAA_LUT_T
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetPaLutT(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65792[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_66176[0];
		break;
    default:
    	return;
	}
	regs[idx] = CSL_FMK(DFE_TX_TXA_TXA_65792_REG_PAA_LUT_T, coeff);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetPaLutT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txPath    [add content]
         idx    [add content]
         coeff    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXA_TXA_65792_REG_PAA_LUT_T
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetPaLutT(DfeFl_TxHandle hDfeTx, uint32_t txPath, uint32_t idx, uint32_t *coeff)
{
	volatile uint32_t *regs;

	switch(txPath)
	{
	case DFE_FL_TXA:
		regs = &hDfeTx->regs->txa_txa_65792[0];
		break;
	case DFE_FL_TXB:
		regs = &hDfeTx->regs->txb_txb_66176[0];
		break;
    default:
    	return;
	}
	*coeff = CSL_FEXT(regs[idx], DFE_TX_TXA_TXA_65792_REG_PAA_LUT_T);
}


/** ============================================================================
 *   @n@b dfeFl_TxSetTimeStep
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_711_REG_TIME_STEP
 *       DFE_TX_TXRX_TXRX_704_REG_TIME_STEP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetTimeStep(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_704, DFE_TX_TXRX_TXRX_704_REG_TIME_STEP, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_711, DFE_TX_TXRX_TXRX_711_REG_TIME_STEP, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetTimeStep
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_711_REG_TIME_STEP
 *       DFE_TX_TXRX_TXRX_704_REG_TIME_STEP
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetTimeStep(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_704, DFE_TX_TXRX_TXRX_704_REG_TIME_STEP);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_711, DFE_TX_TXRX_TXRX_711_REG_TIME_STEP);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetResetInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_712_REG_RESET_INT
 *       DFE_TX_TXRX_TXRX_705_REG_RESET_INT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetResetInit(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_705, DFE_TX_TXRX_TXRX_705_REG_RESET_INT, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_712, DFE_TX_TXRX_TXRX_712_REG_RESET_INT, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetResetInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_712_REG_RESET_INT
 *       DFE_TX_TXRX_TXRX_705_REG_RESET_INT
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetResetInit(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_705, DFE_TX_TXRX_TXRX_705_REG_RESET_INT);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_712, DFE_TX_TXRX_TXRX_712_REG_RESET_INT);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetTddPeriod
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_706_REG_TDD_PERIOD
 *       DFE_TX_TXRX_TXRX_713_REG_TDD_PERIOD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetTddPeriod(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_706, DFE_TX_TXRX_TXRX_706_REG_TDD_PERIOD, arg->data);
    	break;
    case DFE_FL_TXB:
    	CSL_FINS(hDfeTx->regs->txrx_txrx_713, DFE_TX_TXRX_TXRX_713_REG_TDD_PERIOD, arg->data);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetTddPeriod
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_706_REG_TDD_PERIOD
 *       DFE_TX_TXRX_TXRX_713_REG_TDD_PERIOD
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetTddPeriod(DfeFl_TxHandle hDfeTx, DfeFl_TxPathData *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_706, DFE_TX_TXRX_TXRX_706_REG_TDD_PERIOD);
    	break;
    case DFE_FL_TXB:
    	arg->data = CSL_FEXT(hDfeTx->regs->txrx_txrx_713, DFE_TX_TXRX_TXRX_713_REG_TDD_PERIOD);
    	break;
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxSetTddOnOff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_710_REG_TDD_OFF_1
 *       DFE_TX_TXRX_TXRX_709_REG_TDD_ON_1
 *       DFE_TX_TXRX_TXRX_717_REG_TDD_OFF_1
 *       DFE_TX_TXRX_TXRX_714_REG_TDD_ON_0
 *       DFE_TX_TXRX_TXRX_715_REG_TDD_OFF_0
 *       DFE_TX_TXRX_TXRX_708_REG_TDD_OFF_0
 *       DFE_TX_TXRX_TXRX_707_REG_TDD_ON_0
 *       DFE_TX_TXRX_TXRX_716_REG_TDD_ON_1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetTddOnOff(DfeFl_TxHandle hDfeTx, DfeFl_TxTddOnOff *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	CSL_FINS(hDfeTx->regs->txrx_txrx_707, DFE_TX_TXRX_TXRX_707_REG_TDD_ON_0, arg->tdd_on_0);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_708, DFE_TX_TXRX_TXRX_708_REG_TDD_OFF_0, arg->tdd_off_0);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_709, DFE_TX_TXRX_TXRX_709_REG_TDD_ON_1, arg->tdd_on_1);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_710, DFE_TX_TXRX_TXRX_710_REG_TDD_OFF_1, arg->tdd_off_1);
    	break;
    }
    case DFE_FL_TXB:
    {
    	CSL_FINS(hDfeTx->regs->txrx_txrx_714, DFE_TX_TXRX_TXRX_714_REG_TDD_ON_0, arg->tdd_on_0);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_715, DFE_TX_TXRX_TXRX_715_REG_TDD_OFF_0, arg->tdd_off_0);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_716, DFE_TX_TXRX_TXRX_716_REG_TDD_ON_1, arg->tdd_on_1);
    	CSL_FINS(hDfeTx->regs->txrx_txrx_717, DFE_TX_TXRX_TXRX_717_REG_TDD_OFF_1, arg->tdd_off_1);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxGetTddOnOff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_710_REG_TDD_OFF_1
 *       DFE_TX_TXRX_TXRX_709_REG_TDD_ON_1
 *       DFE_TX_TXRX_TXRX_717_REG_TDD_OFF_1
 *       DFE_TX_TXRX_TXRX_714_REG_TDD_ON_0
 *       DFE_TX_TXRX_TXRX_715_REG_TDD_OFF_0
 *       DFE_TX_TXRX_TXRX_708_REG_TDD_OFF_0
 *       DFE_TX_TXRX_TXRX_707_REG_TDD_ON_0
 *       DFE_TX_TXRX_TXRX_716_REG_TDD_ON_1
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetTddOnOff(DfeFl_TxHandle hDfeTx, DfeFl_TxTddOnOff *arg)
{
    switch(arg->txPath)
    {
    case DFE_FL_TXA:
    {
    	arg->tdd_on_0 = CSL_FEXT(hDfeTx->regs->txrx_txrx_707, DFE_TX_TXRX_TXRX_707_REG_TDD_ON_0);
    	arg->tdd_off_0 = CSL_FEXT(hDfeTx->regs->txrx_txrx_708, DFE_TX_TXRX_TXRX_708_REG_TDD_OFF_0);
    	arg->tdd_on_1 = CSL_FEXT(hDfeTx->regs->txrx_txrx_709, DFE_TX_TXRX_TXRX_709_REG_TDD_ON_1);
    	arg->tdd_off_1 = CSL_FEXT(hDfeTx->regs->txrx_txrx_710, DFE_TX_TXRX_TXRX_710_REG_TDD_OFF_1);
    	break;
    }
    case DFE_FL_TXB:
    {
    	arg->tdd_on_0 = CSL_FEXT(hDfeTx->regs->txrx_txrx_714, DFE_TX_TXRX_TXRX_714_REG_TDD_ON_0);
    	arg->tdd_off_0 = CSL_FEXT(hDfeTx->regs->txrx_txrx_715, DFE_TX_TXRX_TXRX_715_REG_TDD_OFF_0);
    	arg->tdd_on_1 = CSL_FEXT(hDfeTx->regs->txrx_txrx_716, DFE_TX_TXRX_TXRX_716_REG_TDD_ON_1);
    	arg->tdd_off_1 = CSL_FEXT(hDfeTx->regs->txrx_txrx_717, DFE_TX_TXRX_TXRX_717_REG_TDD_OFF_1);
    	break;
    }
    default:
    	return;
    }
}

/** ============================================================================
 *   @n@b dfeFl_TxClearPapowerIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5A
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5A
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5B
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxClearPapowerIntrStatus(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5A, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5B, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5A, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5B, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxClearPapeakIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6A
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6A
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6B
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6B
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxClearPapeakIntrStatus(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6A, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6B, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6A, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6B, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxEnablePapowerIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5B_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5B_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxEnablePapowerIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5A_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5B_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5A_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5B_INTR_MASK, 1);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxEnablePapeakIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6B_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6B_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxEnablePapeakIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6A_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6B_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6A_INTR_MASK, 1);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6B_INTR_MASK, 1);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxDisablePapowerIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5B_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5B_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxDisablePapowerIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5A_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_5B_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5A_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_5B_INTR_MASK, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxDisablePapeakIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6B_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6A_INTR_MASK
 *       DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6B_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxDisablePapeakIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6A_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXA_INTERRUPT_6B_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6A_INTR_MASK, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_719, DFE_TX_TXRX_TXRX_719_REG_TXB_INTERRUPT_6B_INTR_MASK, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetForcePapowerIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5A_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5A_FORCE_INTR_BIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetForcePapowerIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5A_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5B_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5A_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5B_FORCE_INTR_BIT, 1);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetForcePapeakIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6A_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6A_FORCE_INTR_BIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetForcePapeakIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6A_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6B_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6A_FORCE_INTR_BIT, 1);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6B_FORCE_INTR_BIT, 1);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxClearForcePapowerIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5A_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5A_FORCE_INTR_BIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxClearForcePapowerIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5A_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_5B_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5A_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_5B_FORCE_INTR_BIT, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxClearForcePapeakIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         txDev    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6A_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6B_FORCE_INTR_BIT
 *       DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6A_FORCE_INTR_BIT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxClearForcePapeakIntr(DfeFl_TxHandle hDfeTx, DfeFl_TxDev txDev)
{
	switch(txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6A_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXA_INTERRUPT_6B_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6A_FORCE_INTR_BIT, 0);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		CSL_FINS(hDfeTx->regs->txrx_txrx_721, DFE_TX_TXRX_TXRX_721_REG_TXB_INTERRUPT_6B_FORCE_INTR_BIT, 0);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryGetPapowerIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5A
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5A
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5B
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryGetPapowerIntrStatus(DfeFl_TxHandle hDfeTx, DfeFl_TxIntrStatus *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5A);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_5B);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5A);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_5B);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxQueryGetPapeakIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6A
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6A
 *       DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6B
 *       DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6B
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxQueryGetPapeakIntrStatus(DfeFl_TxHandle hDfeTx, DfeFl_TxIntrStatus *arg)
{
	switch(arg->txDev)
	{
	case DFE_FL_TXA_PATHA:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6A);
		break;
	}
	case DFE_FL_TXA_PATHB:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXA_INTERRUPT_6B);
		break;
	}
	case DFE_FL_TXB_PATHA:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6A);
		break;
	}
	case DFE_FL_TXB_PATHB:
	{
		arg->status = CSL_FEXT(hDfeTx->regs->txrx_txrx_720, DFE_TX_TXRX_TXRX_720_REG_TXB_INTERRUPT_6B);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_TxSetDpdBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_DPD_BYPASS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetDpdBypass(DfeFl_TxHandle hDfeTx, uint32_t arg)
{
	CSL_FINS(hDfeTx->regs->txrx_txrx_718, DFE_TX_TXRX_TXRX_718_REG_DPD_BYPASS, arg);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetDpdBypass
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_DPD_BYPASS
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetDpdBypass(DfeFl_TxHandle hDfeTx, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeTx->regs->txrx_txrx_718, DFE_TX_TXRX_TXRX_718_REG_DPD_BYPASS);
}

/** ============================================================================
 *   @n@b dfeFl_TxSetTxCkenDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_TX_CKEN_DLY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxSetTxCkenDly(DfeFl_TxHandle hDfeTx, uint32_t arg)
{
	CSL_FINS(hDfeTx->regs->txrx_txrx_718, DFE_TX_TXRX_TXRX_718_REG_TX_CKEN_DLY, arg);
}

/** ============================================================================
 *   @n@b dfeFl_TxGetTxCkenDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx    [add content]
         arg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_TX_TXRX_TXRX_718_REG_TX_CKEN_DLY
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_TxGetTxCkenDly(DfeFl_TxHandle hDfeTx, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeTx->regs->txrx_txrx_718, DFE_TX_TXRX_TXRX_718_REG_TX_CKEN_DLY);
}

#endif /* _DFE_FL_TXAUX_H_ */
