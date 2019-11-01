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

/** @file dfe_fl_dducHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DducHwControl()
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
#include <ti/drv/dfe/dfe_fl_dducAux.h>

/** ============================================================================
 *   @n@b dfeFl_DducHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc       Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DducOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Dduc.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_DducObj objDfeDduc[DFE_FL_DDUC_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_DducHandle hDfeDduc[DFE_FL_DDUC_PER_CNT];
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

         for(i = 0; i < DFE_FL_DDUC_PER_CNT; i++)
         {
		 	 hDfeDduc[i] = dfeFl_DducOpen(hDfe, &objDfeDduc[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset Dduc
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_DducHwControl(hDfeDduc[0], DFE_FL_DDUC_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DducHwControl
(
    DfeFl_DducHandle           hDfeDduc,
    DfeFl_DducHwControlCmd     ctrlCmd,
    void                        *arg
)
{
    uint32_t i;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_DDUC_CMD_CFG_INITS:
        dfeFl_DducConfigInits(hDfeDduc, (DfeFl_SublkInitsConfig *)arg);
        break;
    /**
     * frw
     */
    case DFE_FL_DDUC_CMD_SET_FRW_PHASE_SSEL:
        dfeFl_DducSetFrwPhaseSsel(hDfeDduc, *(uint32_t *)arg);
        break;
    case DFE_FL_DDUC_CMD_CFG_FRW_PHASE:
        {
            DfeFl_DducFrwPhaseConfig *cfg = arg;
            
            dfeFl_DducConfigFrwPhase(hDfeDduc, cfg->phase);
            break;
        }
    case DFE_FL_DDUC_CMD_SET_FRW_CHKSUM_SSEL:
        {
        	dfeFl_DducSetFrwChksumSsel(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
    case DFE_FL_DDUC_CMD_SET_FRW_SIG_SSEL:
        {
        	dfeFl_DducSetFrwSigSsel(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	/**
	 * hop
	 */
    case DFE_FL_DDUC_CMD_SET_HOP_OFFSET_SSEL:
        dfeFl_DducSetHopOffsetSsel(hDfeDduc, *(uint32_t *)arg);
        break;
    case DFE_FL_DDUC_CMD_CFG_HOP_OFFSET:
        {
            DfeFl_DducHopOffsetConfig *cfg = arg;
            
            dfeFl_DducConfigHopOffset(hDfeDduc, cfg->offset);
            break;
        }        
    case DFE_FL_DDUC_CMD_SET_HOP_SSEL:
        dfeFl_DducSetHopSsel(hDfeDduc, *(uint32_t *)arg);
        break;
    case DFE_FL_DDUC_CMD_CFG_HOP_FRQWORD:
        {
            DfeFl_DducHopFrqwordConfig *cfg = arg;
            
            dfeFl_DducConfigHopFrqword(hDfeDduc, cfg);
            break;
        }
    case DFE_FL_DDUC_CMD_SET_SYNC_DELAY:
    	{
    		dfeFl_DducSetSyncDelay(hDfeDduc, *(uint32_t *)arg);
    	}
	/**
	 * mix
	 */
    case DFE_FL_DDUC_CMD_SET_MIX_PHASE_SSEL:
		{
			dfeFl_DducSetMixPhaseSsel(hDfeDduc, *(uint32_t *)arg);
			break;
		}
    case DFE_FL_DDUC_CMD_SET_MIX_NCO_SSEL:
		{
			DfeFl_DducMixNcoSsel *cfg = arg;

			dfeFl_DducSetMixNcoSsel(hDfeDduc, cfg->iMix, cfg->data);
			break;
		}
    case DFE_FL_DDUC_CMD_CFG_MIX_DIT:
		{
			dfeFl_DducCfgMixDit(hDfeDduc, *(uint32_t *)arg);
			break;
		}
    case DFE_FL_DDUC_CMD_CFG_MIX_MODE:
		{
			DfeFl_DducMixModeCfg *cfg = arg;

			dfeFl_DducCfgMixMode(hDfeDduc, cfg->iMixMode, cfg->data);
			break;
		}
	case DFE_FL_DDUC_CMD_SET_MIX_PHASE:
	    {
        	DfeFl_DducMixNcoPhase *cfg = arg;

        	dfeFl_DducSetMixPhase(hDfeDduc, cfg->iMix, cfg->data);
        	break;
        }

	/**
	 * cic
	 */
	case DFE_FL_DDUC_CMD_SET_CIC_FLUSH_SSEL:
        {
        	dfeFl_DducSetCicFlushSsel(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_CIC:
        {
        	dfeFl_DducCfgCic(hDfeDduc, (DfeFl_DducCicCfg *)arg);
        	break;
        }

	/**
	 * Selector
	 */
	case DFE_FL_DDUC_CMD_SET_SELECTOR_SSEL:
        {
        	dfeFl_DducSetSelectorSsel(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_MIX_SEL:
        {
        	dfeFl_DducSetMixSel(hDfeDduc, (DfeFl_DducMixSel *)arg);
        	break;
        }

	/**
	 * pfir
	 */
	case DFE_FL_DDUC_CMD_SET_PFIR_COEF_SSEL:
        {
        	dfeFl_DducSetPfirCoefSsel(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_PFIR_COEF_OFFSET:
        {
        	dfeFl_DducSetPfirCoefOffset(hDfeDduc, (DfeFl_DducPfirCoefOffset *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_PFIR_FCMUX:
        {
        	dfeFl_DducSetPfirFcMux(hDfeDduc, (DfeFl_DducPfirFcMux *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_PFIR_PCSYM:
        {
        	dfeFl_DducSetPfirPcSym(hDfeDduc, (DfeFl_DducPfirPcSym *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_PFIR_COEF:
        {
        	DfeFl_DducPfirCoeff *cfg = arg;
        	for (i=0; i<cfg->numCoeff; i++)
        		dfeFl_DducSetPfirCoeff(hDfeDduc, i, *(cfg->coeff+i));
        	break;
        }

	/**
	 * test bus
	 */
	case DFE_FL_DDUC_CMD_CFG_TESTBUS_MUX:
        {
        	dfeFl_DducCfgTestbusMux(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_TESTBUS_ICG_DLY:
        {
        	dfeFl_DducCfgTestbusIcgdly(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_TESTBUS_BBMUX:
        {
        	dfeFl_DducCfgTestbusBbmux(hDfeDduc, *(uint32_t *)arg);
        	break;
        }

	/**
	 * Signal gen and check sum
	 */
	case DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_MODE:
        {
        	dfeFl_DducCfgTxSiggenMode(hDfeDduc, (DfeFl_DducSiggenMode *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_TX_SIGGEN_RAMP:
        {
        	dfeFl_DducCfgTxSiggenRamp(hDfeDduc, (DfeFl_DducSiggenRampConfig *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_MODE:
        {
        	dfeFl_DducCfgRxSiggenMode(hDfeDduc, (DfeFl_DducSiggenMode *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_RX_SIGGEN_RAMP:
	    {
        	dfeFl_DducCfgRxSiggenRamp(hDfeDduc, (DfeFl_DducSiggenRampConfig *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_TX_CHKSUM:
        {
        	dfeFl_DducCfgTxChksum(hDfeDduc, (DfeFl_DducChksumConfig *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CFG_RX_CHKSUM:
        {
        	dfeFl_DducCfgRxChksum(hDfeDduc, (DfeFl_DducChksumConfig *)arg);
        	break;
        }

	/**
	 * Intr_mask and clk gater
	 */
	case DFE_FL_DDUC_CMD_ENB_CICOV_INTR:
        {
			dfeFl_DducEnableCicovIntr(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_ENB_HOPROLLOVER_INTR:
        {
        	dfeFl_DducEnableHopRolloverIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_ENB_HOPHALFWAY_INTR:
        {
        	dfeFl_DducEnableHopHalfwayIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_DIS_CICOV_INTR:
        {
        	dfeFl_DducDisableCicovIntr(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_DIS_HOPROLLOVER_INTR:
        {
        	dfeFl_DducDisableHopRolloverIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_DIS_HOPHALFWAY_INTR:
        {
        	dfeFl_DducDisableHopHalfwayIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_FORCE_CICOV_INTR:
        {
        	dfeFl_DducSetForceCicovIntr(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_FORCE_HOPROLLOVER_INTR:
        {
        	dfeFl_DducSetForceHopRolloverIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_FORCE_HOPHALFWAY_INTR:
        {
        	dfeFl_DducSetForceHopHalfwayIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_FORCE_CICOV_INTR:
        {
        	dfeFl_DducClearForceCicovIntr(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_FORCE_HOPROLLOVER_INTR:
        {
        	dfeFl_DducClearForceHopRolloverIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_FORCE_HOPHALFWAY_INTR:
        {
        	dfeFl_DducClearForceHopHalfwayIntr(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_CICOV_INTR_STATUS:
        {
        	dfeFl_DducClearCicovIntrStatus(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_HOPROLLOVER_INTR_STATUS:
        {
        	dfeFl_DducClearHopRolloverIntrStatus(hDfeDduc);
        	break;
        }
	case DFE_FL_DDUC_CMD_CLR_HOPHALFWAY_INTR_STATUS:
        {
        	dfeFl_DducClearHopHalfwayIntrStatus(hDfeDduc);
        	break;
        }

	case DFE_FL_DDUC_CMD_SET_TIME_STEP:
        {
        	dfeFl_DducSetTimestep(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_RESET_INT:
        {
        	dfeFl_DducSetResetint(hDfeDduc, *(uint32_t *)arg);
        	break;
        }

	case DFE_FL_DDUC_CMD_SET_TDD_PERIOD:
        {
        	dfeFl_DducSetTddPeriod(hDfeDduc, *(uint32_t *)arg);
        	break;
        }
	case DFE_FL_DDUC_CMD_SET_TDD_ON_OFF:
        {
        	dfeFl_DducSetTddOnoff(hDfeDduc, (DfeFl_DducTddOnOff *)arg);
        	break;
        }
    default:
        err = DFE_FL_INVCMD;
        break;                   
    }
    
    return err;
}
