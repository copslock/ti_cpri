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

/** @file dfe_fl_cbGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_CbGetHwStatus()
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
 *   @n@b dfeFl_CbGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Cb module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb     Valid handle
         queryId    The queryId to this API
         arg        The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
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
 *   @n  None
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
         DfeFl_CbArm cbArm;

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

         // get cb arm status
         dfeFl_CbGetHwStatus(hDfeCb[0], DFE_FL_CB_QUERY_GET_CBC_ARM, &cbArm);

         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_CbGetHwStatus
(
    DfeFl_CbHandle             hDfeCb,
    DfeFl_CbHwStatusQuery      queryId,
    void                        *arg
)
{
    int i;
	DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    /*
     * Init
     */
    case DFE_FL_CB_QUERY_GET_INIT_FRAC_PHASE_CTRL:
    {
    	dfeFl_CbGetInitFracPhaseCtrl(hDfeCb, (DfeFl_CbInitFracPhCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_INITS:
	{
		dfeFl_CbGetInits(hDfeCb, (DfeFl_SublkInitsConfig *) arg);
		break;
	}
    case DFE_FL_CB_QUERY_GET_BUS_CTRL:
    {
    	dfeFl_CbGetBusCtrl(hDfeCb, (DfeFl_CbBusCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_DSP_CTRL:
    {
    	dfeFl_CbGetDspCtrl(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_DPD_MODE:
    {
    	dfeFl_CbGetDpdMode(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_REF_FB_LATENCY:
    {
    	dfeFl_CbGetRefFbLatency(hDfeCb, (DfeFl_CbFbLatency *)arg);
    	break;
    }

    /*
     * DPDA
     */
    case DFE_FL_CB_QUERY_GET_SKIP_CHUNK:
    {
    	dfeFl_CbGetSkipChunk(hDfeCb, (DfeFl_CbSkipChunk *)arg);
    	break;
    }

    /*
     * CB buffer mode select
     */
    case DFE_FL_CB_QUERY_GET_CB_BUF_MODE:
    {
    	dfeFl_CbGetBufMode(hDfeCb, (DfeFl_CbBufMode *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_MODE_SET:
    {
    	dfeFl_CbGetModeSet(hDfeCb, (DfeFl_CbModeSet *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_DLY:
    {
    	dfeFl_CbGetCbDly(hDfeCb, (DfeFl_CbDly *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_RATE_MODE:
    {
    	dfeFl_CbGetCbRateMode(hDfeCb, (DfeFl_CbRateMode *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_FRAC_CNT:
    {
    	dfeFl_CbGetCbFracCnt(hDfeCb, (DfeFl_CbFracCnt *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_SSEL:
    {
    	dfeFl_CbGetCbSsel(hDfeCb, (DfeFl_CbSsel *)arg);
    	break;
    }

    /*
     * Node setup
     */
    case DFE_FL_CB_QUERY_GET_NODE_CONFIG:
    {
    	dfeFl_CbGetNodeConfig(hDfeCb, (DfeFl_CbNodeCfg *)arg);
    	break;
    }

    /*
     * Multi capture
     */
    case DFE_FL_CB_QUERY_GET_MULTI_CTRL:
    {
    	dfeFl_CbGetMultiCtrl(hDfeCb, (DfeFl_CbMultiCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_MULTI_TIMER:
    {
    	dfeFl_CbGetMultiTimer(hDfeCb, (DfeFl_CbMultiTimer *)arg);
    	break;
    }

    /*
     * Trigger monitor
     */
    case DFE_FL_CB_QUERY_GET_TRIG_SET:
    {
    	dfeFl_CbGetTrigSet(hDfeCb, (DfeFl_CbTrigSet *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_TRIG_CONFIG:
    {
    	dfeFl_CbGetTrigConfig(hDfeCb, (DfeFl_CbTrigCfg *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_TRIG_TH:
    {
    	dfeFl_CbGetTrigTh(hDfeCb, (DfeFl_CbTrigTh *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_TRIG_DECODER:
    {
    	dfeFl_CbGetTrigDecoder(hDfeCb, (uint32_t *)arg);
    	break;
    }

	case DFE_FL_CB_QUERY_GET_TRIG_OUTPWR:
	{
		dfeFl_CbQueryTrigOutpwr(hDfeCb, (DfeFl_CbTrigOutpwr *)arg);
		break;
	}

    /*
     * GSG
     */
	case DFE_FL_CB_QUERY_GET_GSG_MODE:
	{
    	dfeFl_CbGetGsgMode(hDfeCb, (DfeFl_CbGsgMode *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_DELAY:
	{
    	dfeFl_CbGetGsgDelay(hDfeCb, (DfeFl_CbGsgDelay *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_TIMER:
	{
    	dfeFl_CbGetGsgTimer(hDfeCb, (DfeFl_CbGsgTimer *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_SSEL:
	{
    	dfeFl_CbGetGsgSsel(hDfeCb, (DfeFl_CbGsgSsel *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_C_SEL:
	{
    	dfeFl_CbGetGsgCsel(hDfeCb, (DfeFl_CbGsgCsel *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_F_SEL:
	{
    	dfeFl_CbGetGsgFsel(hDfeCb, (DfeFl_CbGsgFsel *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_GSG_TRIG_SEL:
	{
    	dfeFl_CbGetGsgTrigSel(hDfeCb, (DfeFl_CbGsgTrigSel *)arg);
    	break;
	}

	case DFE_FL_CB_QUERY_GET_SILENT_DETECT:
	{
    	dfeFl_CbGetSilentDet(hDfeCb, (DfeFl_CbSilentDet *)arg);
    	break;
	}

    /*
     * cb_f
     */
	case DFE_FL_CB_QUERY_GET_CBF_CHUNK_SEL:
	{
    	dfeFl_CbGetCbfChunkSel(hDfeCb, (DfeFl_CbfChunkSel *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_CBF_BROKEN_CHAIN_DETECT:
	{
    	dfeFl_CbGetCbfBrokenChainDet(hDfeCb, (DfeFl_CbfBrokenChainDet *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_CBF_DELTA_POW:
	{
    	dfeFl_CbGetCbfDeltaPow(hDfeCb, (DfeFl_CbfDeltaPow *)arg);
    	break;
	}
	case DFE_FL_CB_QUERY_GET_CBF_BADBUFF_DETECT:
	{
    	dfeFl_CbGetCbfBadbuffDet(hDfeCb, (DfeFl_CbfBadbuffDet *)arg);
    	break;
	}

    case DFE_FL_CB_QUERY_GET_CBF_MAXREF_POW:
    {
    	dfeFl_CbQueryCbfMaxrefPow(hDfeCb, (DfeFl_CbfMaxrefPow *)arg);
    	break;
    }

    /*
     * Power monitor
     */
    case DFE_FL_CB_QUERY_GET_PM_SYNC_DLY:
    {
    	dfeFl_CbGetPmSyncDly(hDfeCb, (DfeFl_CbPmSyncDly *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_PM_INTG_PD:
    {
    	dfeFl_CbGetPmIntgPd(hDfeCb, (DfeFl_CbPmIntgPd *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_PM_CONFIG:
    {
    	dfeFl_CbGetPmConfig(hDfeCb, (DfeFl_CbPmCfg *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_PM_NODE_SEL:
    {
    	dfeFl_CbGetPmNodeSel(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_PM_SSEL:
    {
    	dfeFl_CbGetPmSsel(hDfeCb, (DfeFl_CbPmSsel *)arg);
    	break;
    }

    /*
     * Sourcing
     */
    case DFE_FL_CB_QUERY_GET_SOURCING_CTRL:
    {
    	dfeFl_CbGetSourcingCtrl(hDfeCb, (DfeFl_CbSrcCtrl *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_SRC_NODE_CTRL:
    {
    	dfeFl_CbGetSrcNodeCtrl(hDfeCb, (DfeFl_CbSrcNodeCtrl *)arg);
    	break;
    }

    /*
     * TDD
     */
    case DFE_FL_CB_QUERY_GET_TIME_STEP:
    {
    	dfeFl_CbGetTimeStep(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_RESET_INT:
    {
    	dfeFl_CbGetResetInt(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_TDD_PERIOD:
    {
    	dfeFl_CbGetTddPeriod(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_TDD_ON_OFF:
    {
    	dfeFl_CbGetTddOnOff(hDfeCb, (DfeFl_CbTddOnOff *)arg);
    	break;
    }

    /*
     * SSEL
     */
    case DFE_FL_CB_QUERY_GET_CBC_START_SSEL:
    {
    	dfeFl_CbGetCbcStartSsel(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CBF_START_SSEL:
    {
    	dfeFl_CbGetCbfStartSsel(hDfeCb, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_SOURCE_SSEL:
    {
    	dfeFl_CbGetSourceSsel(hDfeCb, (uint32_t *)arg);
    	break;
    }
    //DFE_FL_CB_QUERY_GET_PM_SSEL,

    /*
     * CB arm
     */
    case DFE_FL_CB_QUERY_GET_CBC_ARM:
    {
    	dfeFl_CbQueryCbcArm(hDfeCb, (DfeFl_CbArm *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CBC_FORCE_RESET:
    {
    	dfeFl_CbQueryCbcForceReset(hDfeCb, (DfeFl_CbForceReset *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CBF_ARM:
    {
    	dfeFl_CbQueryCbfArm(hDfeCb, (DfeFl_CbArm *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CBF_SUBSAMPLE:
    {
    	dfeFl_CbQueryCbfSubsample(hDfeCb, (uint32_t *)arg);
    	break;
    }
//    case DFE_FL_CB_QUERY_GET_CBF_FORCE_RESET:
//    {
//    	dfeFl_CbQueryCbfForceReset(hDfeCb, (DfeFl_CbForceReset *)arg);
//    	break;
//    }
    /*
     * CB done
     */
    case DFE_FL_CB_QUERY_GET_CB_DONE_FRAC_CNT:
    {
    	dfeFl_CbQueryCbDoneFracCnt(hDfeCb, (DfeFl_CbDoneInfo *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_DONE_ADDR:
    {
    	dfeFl_CbQueryCbDoneAddr(hDfeCb, (DfeFl_CbDoneInfo *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_DONE_LEN_CNT:
    {
    	dfeFl_CbQueryCbDoneLenCnt(hDfeCb, (DfeFl_CbDoneInfo *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_BUF_FULL:
    {
    	dfeFl_CbQueryCbBufFull(hDfeCb, (DfeFl_CbDoneInfo *)arg);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_STATUS:
    {
        DfeFl_CbStatus *cbStatus = (DfeFl_CbStatus *)arg;
        DfeFl_CbDoneInfo doneInfo;
        
        doneInfo.cbBuf = cbStatus->cbBuf;

    	dfeFl_CbQueryCbDoneAddr(hDfeCb, &doneInfo);
    	cbStatus->doneAddr = doneInfo.data;    	
    	dfeFl_CbQueryCbDoneLenCnt(hDfeCb, &doneInfo);
    	cbStatus->doneLenCnt = doneInfo.data;
    	dfeFl_CbQueryCbDoneFracCnt(hDfeCb, &doneInfo);
    	cbStatus->doneFracCnt = doneInfo.data;
    	dfeFl_CbQueryCbBufFull(hDfeCb, &doneInfo);
    	cbStatus->bufferFullFlag = doneInfo.data;
    	
    	break;
    }

    /*
     * chunk
     */
//    case DFE_FL_CB_QUERY_GET_REF_CHUNK_DONE_ADDR:
//    {
//    	dfeFl_CbQueryRefChunkDoneAddr(hDfeCb, (DfeFl_CbChunkDoneAddr *)arg);
//    	break;
//    }
//    case DFE_FL_CB_QUERY_GET_FB_CHUNK_DONE_ADDR:
//    {
//    	dfeFl_CbQueryFbChunkDoneAddr(hDfeCb, (DfeFl_CbChunkDoneAddr *)arg);
//    	break;
//    }
	case DFE_FL_CB_QUERY_GET_CHUNK_DONE_ADDR:
	{
		dfeFl_CbQueryChunkDoneAddr(hDfeCb, (DfeFl_CbChunkDoneAddr *)arg);
		break;
	}

    /*
     * CB
     */
    case DFE_FL_CB_QUERY_GET_CB_MSB:
    {
    	DfeFl_CbData *cfg = (DfeFl_CbData *)arg;
    	for (i = 0; i<cfg->size; i++)
    		dfeFl_CbQueryCbMSB(hDfeCb, cfg->cbBuf, i+cfg->startPos, cfg->data+i);
    	break;
    }
    case DFE_FL_CB_QUERY_GET_CB_LSB:
    {
    	DfeFl_CbData *cfg = (DfeFl_CbData *)arg;
    	for (i = 0; i<cfg->size; i++)
    		dfeFl_CbQueryCbLSB(hDfeCb, cfg->cbBuf, i+cfg->startPos, cfg->data+i);
    	break;
    }
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
