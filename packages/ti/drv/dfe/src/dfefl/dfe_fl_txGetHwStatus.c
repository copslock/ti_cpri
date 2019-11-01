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

/** @file dfe_fl_txGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_SummerGetHwStatus()
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
 *   @n@b dfeFl_TxGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Summer module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer     Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_SummerOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
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
         DfeFl_TxPaIntrpt TxPaIntrpt;

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

         // check Tx interrupt status
         TxPaIntrpt.txDev = DFE_FL_TXA_PATHA;
         dfeFl_TxGetHwStatus(hDfeTx[0], DFE_FL_TX_QUERY_GET_PA_INTERRUPT, &TxPaIntrpt);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_TxGetHwStatus
(
    DfeFl_TxHandle             hDfeTx,
    DfeFl_TxHwStatusQuery      queryId,
    void                        *arg
)
{
    int i;
	DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    case DFE_FL_TX_QUERY_GET_INITS:
        dfeFl_TxGetInits(hDfeTx, (DfeFl_SublkInitsConfig *)arg);
        break;
    /*
     * Signal Generator and CheckSum
     */
    case DFE_FL_TX_QUERY_GET_TESTBUS_SEL:
    {
    	dfeFl_TxGetTestbusSel(hDfeTx, (DfeFl_TxTestBusSel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_CHKSUM_RESULT:
    {
    	dfeFl_TxQueryChksumResult(hDfeTx, (uint32_t *)arg);
    	break;
    }

    /*
     * Resampler
     */
    case DFE_FL_TX_QUERY_GET_RSMP_COEFF:
    {
    	DfeFl_TxRsmpCoeff *cfg = (DfeFl_TxRsmpCoeff *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_TxQueryRsmpCoeff(hDfeTx, cfg->txDev, i, &cfg->coeff[i]);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_CF_SSEL:
    {
    	dfeFl_TxGetRsmpCfSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_INIT_PH_SSEL:
    {
    	dfeFl_TxGetRsmpInitPhSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_ANT_PH:
    {
    	dfeFl_TxGetRsmpAntPhase(hDfeTx, (DfeFl_TxPhase *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_INIT_PH:
    {
    	dfeFl_TxGetRsmpInitPhase(hDfeTx, (DfeFl_TxPhase *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_MODE:
    {
    	dfeFl_TxGetRsmpMode(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RSMP_STATE:
    {
    	dfeFl_TxGetRsmpState(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * Mix
     */
    case DFE_FL_TX_QUERY_GET_MIX_SSEL:
    {
    	dfeFl_TxGetMixSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_MIX_FREQ:
    {
    	dfeFl_TxGetMixFreq(hDfeTx, (DfeFl_TxMixFreq *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_MIX_PHASE:
	{
		dfeFl_TxGetMixPhase(hDfeTx, (DfeFl_TxPhase *)arg);
		break;
	}
    case DFE_FL_TX_QUERY_GET_MIX_DIT_EN:
    {
    	dfeFl_TxGetMixDitEn(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * Mux Control
     */
    case DFE_FL_TX_QUERY_GET_MC_MIX:
    {
    	dfeFl_TxGetMcMix(hDfeTx, (DfeFl_TxMcMix *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_MC_RSMP:
    {
    	dfeFl_TxGetMcRsmp(hDfeTx, (DfeFl_TxMcRsmp *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_MC_BUC:
    {
    	dfeFl_TxGetMcBuc(hDfeTx, (DfeFl_TxMcBuc *)arg);
    	break;
    }

    /*
     * Clock Control
     */
    case DFE_FL_TX_QUERY_GET_CC_RSMP:
    {
    	dfeFl_TxGetCcRsmp(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_CC_BUC:
    {
    	dfeFl_TxGetCcBuc(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_CC_MIX:
    {
    	dfeFl_TxGetCcMix(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_CC_PA:
    {
    	dfeFl_TxGetCcPa(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * DC offset
     */
    case DFE_FL_TX_QUERY_GET_DCOFF_SSEL:
    {
    	dfeFl_TxGetDCoffSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_DC_OFFSETS:
    {
    	dfeFl_TxGetDcOffsets(hDfeTx, (DfeFl_TxDcOffsets *)arg);
    	break;
    }

    case DFE_FL_TX_QUERY_GET_ADDB_EN:
    {
    	dfeFl_TxGetAddbEn(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }

    /*
     * BUC
     */
    case DFE_FL_TX_QUERY_GET_BUC_COEF:
    {
    	DfeFl_TxBucCoeff *cfg = (DfeFl_TxBucCoeff *)arg;
    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_TxGetBucCoeff(hDfeTx, cfg->txDev, i, &cfg->coeff[i]);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_TWO_ANT:
    {
    	dfeFl_TxGetBucTwoAnt(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_CASCADE:
    {
    	dfeFl_TxGetBucCascade(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_EVEN:
    {
    	dfeFl_TxGetBucEven(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_INTERP:
    {
    	dfeFl_TxGetBucInterp(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_ADD_MUX:
    {
    	dfeFl_TxGetBucAddMux(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_B_PATH_ENABLE:
    {
    	dfeFl_TxGetBucBPathEnable(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_BUC_IN_MUX_CTL:
    {
    	dfeFl_TxGetBucInMuxCtl(hDfeTx, (DfeFl_TxBucInMuxCtl *)arg);
    	break;
    }

    /*
     * PA protect
     */
    case DFE_FL_TX_QUERY_GET_PA_TWO_ANT_MODE:
    {
    	dfeFl_TxGetPaTwoAntMode(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_FACTORY_TEST:
    {
    	dfeFl_TxGetPaFactoryTest(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_SSEL:
    {
    	dfeFl_TxGetPaSsel(hDfeTx, (DfeFl_TxSsel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_TH:
    {
    	dfeFl_TxGetPaTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_IIR_MU:
    {
    	dfeFl_TxGetPaIirMu(hDfeTx, (DfeFl_TxPaIirMu *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_IIR_TH_SEL:
    {
    	dfeFl_TxGetPaIirThSel(hDfeTx, (DfeFl_TxPaIirThSel *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_IIR_TH_VAL:
    {
    	dfeFl_TxGetPaIirThVal(hDfeTx, (DfeFl_TxPaIirThVal *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_CC_TH:
    {
    	dfeFl_TxGetPaCcTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_PEAK_TH:
    {
    	dfeFl_TxGetPaPeakTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_PEAK_CNT_TH:
    {
    	dfeFl_TxGetPaPeakCntTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_PEAK_GAIN_TH:
    {
    	dfeFl_TxGetPaPeakGainTh(hDfeTx, (DfeFl_TxPaTh *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_CC_CNT:
    {
    	dfeFl_TxQueryPaCcCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_PEAK_CNT:
    {
    	dfeFl_TxQueryPaPeakCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_PEAKGAIN_CNT:
    {
    	dfeFl_TxQueryPaPeakGainCnt(hDfeTx, (DfeFl_TxPaCnt *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_MASK:
    {
    	dfeFl_TxGetPaMask(hDfeTx, (DfeFl_TxPaMask *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_INTERRUPT:
    {
    	dfeFl_TxQueryPaIntrpt(hDfeTx, (DfeFl_TxPaIntrpt *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_MAXMAG_CLR:
    {
    	dfeFl_TxQueryPaMaxmagClr(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_MAXMAG_NO_CLR:
    {
    	dfeFl_TxQueryPaMaxmagNoClr(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_D50:
    {
    	dfeFl_TxQueryPaD50(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_D51:
    {
    	dfeFl_TxQueryPaD51(hDfeTx, (DfeFl_TxDevData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_LUTP:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxGetPaLutP(hDfeTx, cfg->txPath, i, (cfg->lut_data+i));
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_LUTQ:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxGetPaLutQ(hDfeTx, cfg->txPath, i, (cfg->lut_data+i));
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PA_LUTT:
    {
    	DfeFl_TxPaLut *cfg = (DfeFl_TxPaLut *)arg;
    	for (i = 0; i<cfg->numLut; i++)
    		dfeFl_TxGetPaLutT(hDfeTx, cfg->txPath, i, (cfg->lut_data+i));
    	break;
    }

    /*
     * TDD
     */
    case DFE_FL_TX_QUERY_GET_TIME_STEP:
    {
    	dfeFl_TxGetTimeStep(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_RESET_INT:
    {
    	dfeFl_TxGetResetInit(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_TDD_PERIOD:
    {
    	dfeFl_TxGetTddPeriod(hDfeTx, (DfeFl_TxPathData *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_TDD_ON_OFF:
    {
    	dfeFl_TxGetTddOnOff(hDfeTx, (DfeFl_TxTddOnOff *)arg);
    	break;
    }
    /*
     * TXRX
     */
    case DFE_FL_TX_QUERY_GET_PAPOWER_INTR_STATUS:
    {
    	dfeFl_TxQueryGetPapowerIntrStatus(hDfeTx, (DfeFl_TxIntrStatus *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_PAPEAK_INTR_STATUS:
    {
    	dfeFl_TxQueryGetPapeakIntrStatus(hDfeTx, (DfeFl_TxIntrStatus *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_DPD_BYPASS:
    {
    	dfeFl_TxGetDpdBypass(hDfeTx, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_TX_QUERY_GET_TX_CKEN_DLY:
    {
    	dfeFl_TxGetTxCkenDly(hDfeTx, (uint32_t *)arg);
    	break;
    }
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
