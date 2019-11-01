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

/** @file dfe_fl_cbHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_CbHwControl()
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
#include <ti/drv/dfe/dfe_fl_cbAux.h>
 
/** ============================================================================
 *   @n@b dfeFl_CbHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb     Valid handle
         ctrlCmd    The command to this API
         arg        The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_CbOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Cb.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_CbObj objDfeCb[DFE_FL_CB_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_CbHandle hDfeCb[DFE_FL_CB_PER_CNT];
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

         for(i = 0; i < DFE_FL_CB_PER_CNT; i++)
         {
		 	 hDfeCb[i] = dfeFl_CbOpen(hDfe, &objDfeCb[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset Cb
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
    	 dfeFl_CbHwControl(hDfeCb[0], DFE_FL_CB_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_CbHwControl
(
    DfeFl_CbHandle             hDfeCb,
    DfeFl_CbHwControlCmd       ctrlCmd,
    void                        *arg
)
{
	int i;
	DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_CB_CMD_CFG_INITS:
        dfeFl_CbConfigInits(hDfeCb, (DfeFl_SublkInitsConfig *)arg);
        break;
    /*
     * CB arm
     */
    case DFE_FL_CB_CMD_SET_CBC_ARM:
    {
    	dfeFl_CbSetCbcArm(hDfeCb, (DfeFl_CbArm *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBC_FORCE_RESET:
    {
    	dfeFl_CbSetCbcForceReset(hDfeCb, (DfeFl_CbForceReset *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBF_ARM:
    {
    	dfeFl_CbSetCbfArm(hDfeCb, (DfeFl_CbArm *)arg);
    	break;
    }
//    case DFE_FL_CB_CMD_SET_CBF_FORCE_RESET:
//    {
//    	dfeFl_CbSetCbfForceReset(hDfeCb, (DfeFl_CbForceReset *)arg);
//    	break;
//    }
    case DFE_FL_CB_CMD_SET_CBF_SUBSAMPLE:
    {
    	dfeFl_CbSetCbfSubsample(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_BUS_CTRL:
    {
    	dfeFl_CbSetBusCtrl(hDfeCb, (DfeFl_CbBusCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_DSP_CTRL:
    {
    	dfeFl_CbSetDspCtrl(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_DPD_MODE:
    {
    	dfeFl_CbSetDpdMode(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_REF_FB_LATENCY:
    {
    	dfeFl_CbSetRefFbLatency(hDfeCb, (DfeFl_CbFbLatency *)arg);
    	break;
    }

    /*
     * DPDA
     */
    case DFE_FL_CB_CMD_SET_SKIP_CHUNK:
    {
    	dfeFl_CbSetSkipChunk(hDfeCb, (DfeFl_CbSkipChunk *)arg);
    	break;
    }

    /*
     * CB buffer mode select
     */
    case DFE_FL_CB_CMD_SET_CB_BUF_MODE:
    {
    	dfeFl_CbSetBufMode(hDfeCb, (DfeFl_CbBufMode *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_MODE_SET:
    {
    	dfeFl_CbSetModeSet(hDfeCb, (DfeFl_CbModeSet *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_DLY:
    {
    	dfeFl_CbSetCbDly(hDfeCb, (DfeFl_CbDly *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_RATE_MODE:
    {
    	dfeFl_CbSetCbRateMode(hDfeCb, (DfeFl_CbRateMode *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_FRAC_CNT:
    {
    	dfeFl_CbSetCbFracCnt(hDfeCb, (DfeFl_CbFracCnt *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_SSEL:
    {
    	dfeFl_CbSetCbSsel(hDfeCb, (DfeFl_CbSsel *)arg);
    	break;
    }

    /*
     * Node setup
     */
    case DFE_FL_CB_CMD_SET_NODE_CONFIG:
    {
    	dfeFl_CbSetNodeConfig(hDfeCb, (DfeFl_CbNodeCfg *)arg);
    	break;
    }

    /*
     * Multi capture
     */
    case DFE_FL_CB_CMD_SET_MULTI_CTRL:
    {
    	dfeFl_CbSetMultiCtrl(hDfeCb, (DfeFl_CbMultiCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_MULTI_TIMER:
    {
    	dfeFl_CbSetMultiTimer(hDfeCb, (DfeFl_CbMultiTimer *)arg);
    	break;
    }

    /*
     * Trigger monitor
     */
    case DFE_FL_CB_CMD_SET_TRIG_SET:
    {
    	dfeFl_CbSetTrigSet(hDfeCb, (DfeFl_CbTrigSet *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_TRIG_CONFIG:
    {
    	dfeFl_CbSetTrigConfig(hDfeCb, (DfeFl_CbTrigCfg *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_TRIG_TH:
    {
    	dfeFl_CbSetTrigTh(hDfeCb, (DfeFl_CbTrigTh *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_TRIG_DECODER:
    {
    	dfeFl_CbSetTrigDecoder(hDfeCb, *(uint32_t *)arg);
    	break;
    }

    /*
     * GSG
     */
    case DFE_FL_CB_CMD_SET_GSG_MODE:
    {
    	dfeFl_CbSetGsgMode(hDfeCb, (DfeFl_CbGsgMode *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_DELAY:
    {
    	dfeFl_CbSetGsgDelay(hDfeCb, (DfeFl_CbGsgDelay *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_TIMER:
    {
    	dfeFl_CbSetGsgTimer(hDfeCb, (DfeFl_CbGsgTimer *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_SSEL:
    {
    	dfeFl_CbSetGsgSsel(hDfeCb, (DfeFl_CbGsgSsel *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_C_SEL:
    {
    	dfeFl_CbSetGsgCsel(hDfeCb, (DfeFl_CbGsgCsel *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_F_SEL:
    {
    	dfeFl_CbSetGsgFsel(hDfeCb, (DfeFl_CbGsgFsel *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_GSG_TRIG_SEL:
    {
    	dfeFl_CbSetGsgTrigSel(hDfeCb, (DfeFl_CbGsgTrigSel *)arg);
    	break;
    }

    case DFE_FL_CB_CMD_SET_SILENT_DETECT:
    {
    	dfeFl_CbSetSilentDet(hDfeCb, (DfeFl_CbSilentDet *)arg);
    	break;
    }

    /*
     * cb_f
     */
    case DFE_FL_CB_CMD_SET_CBF_CHUNK_SEL:
    {
    	dfeFl_CbSetCbfChunkSel(hDfeCb, (DfeFl_CbfChunkSel *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBF_BROKEN_CHAIN_DETECT:
    {
    	dfeFl_CbSetCbfBrokenChainDet(hDfeCb, (DfeFl_CbfBrokenChainDet *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBF_DELTA_POW:
    {
    	dfeFl_CbSetCbfDeltaPow(hDfeCb, (DfeFl_CbfDeltaPow *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBF_BADBUFF_DETECT:
    {
    	dfeFl_CbSetCbfBadbuffDet(hDfeCb, (DfeFl_CbfBadbuffDet *)arg);
    	break;
    }

    /*
     * Power monitor
     */
    case DFE_FL_CB_CMD_SET_PM_SYNC_DLY:
    {
    	dfeFl_CbSetPmSyncDly(hDfeCb, (DfeFl_CbPmSyncDly *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_PM_INTG_PD:
    {
    	dfeFl_CbSetPmIntgPd(hDfeCb, (DfeFl_CbPmIntgPd *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_PM_CONFIG:
    {
    	dfeFl_CbSetPmConfig(hDfeCb, (DfeFl_CbPmCfg *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_PM_NODE_SEL:
    {
    	dfeFl_CbSetPmNodeSel(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_PM_SSEL:
    {
    	dfeFl_CbSetPmSsel(hDfeCb, (DfeFl_CbPmSsel *)arg);
    	break;
    }

    /*
     * Sourcing
     */
    case DFE_FL_CB_CMD_SET_SOURCING_CTRL:
    {
    	dfeFl_CbSetSourcingCtrl(hDfeCb, (DfeFl_CbSrcCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_SRC_NODE_CTRL:
    {
    	dfeFl_CbSetSrcNodeCtrl(hDfeCb, (DfeFl_CbSrcNodeCtrl *)arg);
    	break;
    }

    /*
     * TDD
     */
    case DFE_FL_CB_CMD_SET_TIME_STEP:
    {
    	dfeFl_CbSetTimeStep(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_RESET_INT:
    {
    	dfeFl_CbSetResetInt(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_TDD_PERIOD:
    {
    	dfeFl_CbSetTddPeriod(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_TDD_ON_OFF:
    {
    	dfeFl_CbSetTddOnOff(hDfeCb, (DfeFl_CbTddOnOff *)arg);
    	break;
    }

    /*
     * SSEL
     */
    case DFE_FL_CB_CMD_SET_CBC_START_SSEL:
    {
    	dfeFl_CbSetCbcStartSsel(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CBF_START_SSEL:
    {
    	dfeFl_CbSetCbfStartSsel(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_SOURCE_SSEL:
    {
    	dfeFl_CbSetSourceSsel(hDfeCb, *(uint32_t *)arg);
    	break;
    }
//    case DFE_FL_CB_CMD_SET_PM_SSEL:
//    {
//    	dfeFl_CbSetPmSsel(hDfeCb, *(uint32_t *)arg);
//    	break;
//    }

    /*
     * init
     */
    case DFE_FL_CB_CMD_SET_INIT_FRAC_PHASE_CTRL:
    {
    	dfeFl_CbSetInitFracPhaseCtrl(hDfeCb, (DfeFl_CbInitFracPhCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_INIT_SSEL:
    {
    	dfeFl_CbSetInitSsel(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_INIT_CLK_GATE:
    {
    	dfeFl_CbSetInitClkGate(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_INIT_STATE:
    {
    	dfeFl_CbSetInitState(hDfeCb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_CMD_SET_INIT_CLR_DATA:
    {
    	dfeFl_CbSetInitClrData(hDfeCb, *(uint32_t *)arg);
    	break;
    }

    /*
     * CB
     */
    case DFE_FL_CB_CMD_SET_CB_MSB:
    {
    	DfeFl_CbData *cfg = (DfeFl_CbData *)arg;
    	for (i = 0; i<cfg->size; i++)
    		dfeFl_CbSetCbMSB(hDfeCb, cfg->cbBuf, i, cfg->data+i);
    	break;
    }
    case DFE_FL_CB_CMD_SET_CB_LSB:
    {
    	DfeFl_CbData *cfg = (DfeFl_CbData *)arg;
    	for (i = 0; i<cfg->size; i++)
    		dfeFl_CbSetCbLSB(hDfeCb, cfg->cbBuf, i, cfg->data+i);
    	break;
    }
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
