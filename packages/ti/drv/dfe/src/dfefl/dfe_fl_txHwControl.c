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

/** @file dfe_fl_txHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_TxHwControl()
 *
 *
 */
/* =============================================================================
 * Revision History
 * ===============
 *
 *
 * =============================================================================
 */
#include <ti/drv/dfe/dfe_fl_txAux.h>
 
/** ============================================================================
 *   @n@b dfeFl_TxHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeTx         Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_TxOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Tx.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_TxObj objDfeTx[DFE_FL_TX_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_TxHandle hDfeTx[DFE_FL_TX_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_SublkInitsConfig inits;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_TX_PER_CNT; i++)
         {
		 	 hDfeTx[i] = dfeFl_TxOpen(hDfe, &objDfeTx[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset Tx
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_TxHwControl(hDfeTx[0], DFE_FL_TX_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_TxHwControl
(
    DfeFl_TxHandle             hDfeTx,
    DfeFl_TxHwControlCmd       ctrlCmd,
    void                        *arg
)
{
    int i;
	DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_TX_CMD_CFG_INITS:
        dfeFl_TxConfigInits(hDfeTx, (DfeFl_SublkInitsConfig *)arg);
        break;
    /*
     *  Signal Generator and CheckSum
     */
    case DFE_FL_TX_CMD_SET_TESTBUS_SEL:
    {
    	dfeFl_TxSetTestbusSel(hDfeTx, (DfeFl_TxTestBusSel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_CHKSUM_SSEL:
    {
    	dfeFl_TxSetChksumSsel(hDfeTx, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_SIGGEN_MODE:
    {
    	dfeFl_TxSetSiggenMode(hDfeTx, (DfeFl_TxSiggenMode *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CFG_SIGGEN_RAMP:
    {
    	dfeFl_TxConfigSiggenRamp(hDfeTx, (DfeFl_TxSiggenRampConfig *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CFG_CHKSUM:
    {
    	dfeFl_TxConfigChksum(hDfeTx, (DfeFl_TxChksumConfig *)arg);
    	break;
    }

    /*
     * Resampler
     */
    case DFE_FL_TX_CMD_UPDATE_RSMP_COEFF:
    {
    	DfeFl_TxRsmpCoeff *cfg = (DfeFl_TxRsmpCoeff *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_TxSetRsmpCoeff(hDfeTx, cfg->txDev, i, cfg->coeff[i]);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_CF_SSEL:
    {
    	dfeFl_TxSetRsmpCfSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_INIT_PH_SSEL:
    {
    	dfeFl_TxSetRsmpInitPhSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_ANT_PH:
    {
    	dfeFl_TxSetRsmpAntPhase(hDfeTx, (DfeFl_TxPhase *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_INIT_PH:
    {
    	dfeFl_TxSetRsmpInitPhase(hDfeTx, (DfeFl_TxPhase *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_MODE:
    {
    	dfeFl_TxSetRsmpMode(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RSMP_STATE:
    {
    	dfeFl_TxSetRsmpState(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * Mix
     */
    case DFE_FL_TX_CMD_SET_MIX_SSEL:
    {
    	dfeFl_TxSetMixSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_MIX_FREQ:
    {
    	dfeFl_TxSetMixFreq(hDfeTx, (DfeFl_TxMixFreq *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_MIX_PHASE:
	{
		dfeFl_TxSetMixPhase(hDfeTx, (DfeFl_TxPhase *)arg);
		break;
	}
    case DFE_FL_TX_CMD_SET_MIX_DIT_EN:
    {
    	dfeFl_TxSetMixDitEn(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * Mux Control
     */
    case DFE_FL_TX_CMD_SET_MC_MIX:
    {
    	dfeFl_TxSetMcMix(hDfeTx, (DfeFl_TxMcMix *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_MC_RSMP:
    {
    	dfeFl_TxSetMcRsmp(hDfeTx, (DfeFl_TxMcRsmp *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_MC_BUC:
    {
    	dfeFl_TxSetMcBuc(hDfeTx, (DfeFl_TxMcBuc *)arg);
    	break;
    }

    /*
     * Clock Control
     */
    case DFE_FL_TX_CMD_SET_CC_RSMP:
    {
    	dfeFl_TxSetCcRsmp(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_CC_BUC:
    {
    	dfeFl_TxSetCcBuc(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_CC_MIX:
    {
    	dfeFl_TxSetCcMix(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_CC_PA:
    {
    	dfeFl_TxSetCcPa(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * DC offset
     */
    case DFE_FL_TX_CMD_SET_DCOFF_SSEL:
    {
    	dfeFl_TxSetDCoffSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_DC_OFFSETS:
    {
    	dfeFl_TxSetDcOffsets(hDfeTx, (DfeFl_TxDcOffsets *)arg);
    	break;
    }

    case DFE_FL_TX_CMD_SET_ADDB_EN:
    {
    	dfeFl_TxSetAddbEn(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * BUC
     */
    case DFE_FL_TX_CMD_SET_BUC_COEF:
    {
    	DfeFl_TxBucCoeff *cfg = (DfeFl_TxBucCoeff *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_TxSetBucCoeff(hDfeTx, cfg->txDev, i, cfg->coeff[i]);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_TWO_ANT:
    {
    	dfeFl_TxSetBucTwoAnt(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_CASCADE:
    {
    	dfeFl_TxSetBucCascade(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_EVEN:
    {
    	dfeFl_TxSetBucEven(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_INTERP:
    {
    	dfeFl_TxSetBucInterp(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_ADD_MUX:
    {
    	dfeFl_TxSetBucAddMux(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_B_PATH_ENABLE:
    {
    	dfeFl_TxSetBucBPathEnable(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_BUC_IN_MUX_CTL:
    {
    	dfeFl_TxSetBucInMuxCtl(hDfeTx, (DfeFl_TxBucInMuxCtl *)arg);
    	break;
    }

    /*
     * PA protect
     */
    case DFE_FL_TX_CMD_SET_PA_TWO_ANT_MODE:
    {
    	dfeFl_TxSetPaTwoAntMode(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_FACTORY_TEST:
    {
    	dfeFl_TxSetPaFactoryTest(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_SSEL:
    {
    	dfeFl_TxSetPaSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_TH:
    {
    	dfeFl_TxSetPaTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_IIR_MU:
    {
    	dfeFl_TxSetPaIirMu(hDfeTx, (DfeFl_TxPaIirMu *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_IIR_TH_SEL:
    {
    	dfeFl_TxSetPaIirThSel(hDfeTx, (DfeFl_TxPaIirThSel *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_IIR_TH_VAL:
    {
    	dfeFl_TxSetPaIirThVal(hDfeTx, (DfeFl_TxPaIirThVal *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_CC_TH:
    {
    	dfeFl_TxSetPaCcTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_PEAK_TH:
    {
    	dfeFl_TxSetPaPeakTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_PEAK_CNT_TH:
    {
    	dfeFl_TxSetPaPeakCntTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_PEAK_GAIN_TH:
    {
    	dfeFl_TxSetPaPeakGainTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_CC_CNT:
    {
    	dfeFl_TxSetPaCcCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_PEAK_CNT:
    {
    	dfeFl_TxSetPaPeakCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_PEAKGAIN_CNT:
    {
    	dfeFl_TxSetPaPeakGainCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_MASK:
    {
    	dfeFl_TxSetPaMask(hDfeTx, (DfeFl_TxPaMask *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_INTERRUPT:
    {
    	dfeFl_TxSetPaIntrpt(hDfeTx, (DfeFl_TxPaIntrpt *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_LUTP:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxSetPaLutP(hDfeTx, cfg->txPath, i, *(cfg->lut_data+i));
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_LUTQ:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxSetPaLutQ(hDfeTx, cfg->txPath, i, *(cfg->lut_data+i));
    	break;
    }
    case DFE_FL_TX_CMD_SET_PA_LUTT:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxSetPaLutT(hDfeTx, cfg->txPath, i, *(cfg->lut_data+i));
    	break;
    }

    /*
     * TDD
     */
    case DFE_FL_TX_CMD_SET_TIME_STEP:
    {
    	dfeFl_TxSetTimeStep(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_RESET_INT:
    {
    	dfeFl_TxSetResetInit(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_TDD_PERIOD:
    {
    	dfeFl_TxSetTddPeriod(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_TDD_ON_OFF:
    {
    	dfeFl_TxSetTddOnOff(hDfeTx, (DfeFl_TxTddOnOff *)arg);
    	break;
    }
    /*
     * TXRX
     */
    case DFE_FL_TX_CMD_ENB_PAPOWER_INTR:
    {
    	dfeFl_TxEnablePapowerIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_ENB_PAPEAK_INTR:
    {
    	dfeFl_TxEnablePapeakIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_DIS_PAPOWER_INTR:
    {
    	dfeFl_TxDisablePapowerIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_DIS_PAPEAK_INTR:
    {
    	dfeFl_TxDisablePapeakIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_FORCE_PAPOWER_INTR:
    {
    	dfeFl_TxSetForcePapowerIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_FORCE_PAPEAK_INTR:
    {
    	dfeFl_TxSetForcePapeakIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CLR_FORCE_PAPOWER_INTR:
    {
    	dfeFl_TxClearForcePapowerIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CLR_FORCE_PAPEAK_INTR:
    {
    	dfeFl_TxClearForcePapeakIntr(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CLR_PAPOWER_INTR_STATUS:
    {
    	dfeFl_TxClearPapowerIntrStatus(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_CLR_PAPEAK_INTR_STATUS:
    {
    	dfeFl_TxClearPapeakIntrStatus(hDfeTx, *(DfeFl_TxDev *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_DPD_BYPASS:
    {
    	dfeFl_TxSetDpdBypass(hDfeTx, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_TX_CMD_SET_TX_CKEN_DLY:
    {
    	dfeFl_TxSetTxCkenDly(hDfeTx, *(uint32_t *)arg);
    	break;
    }
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
