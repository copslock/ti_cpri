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

/** @file dfe_fl_cfrHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_CfrHwControl()
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
#include <ti/drv/dfe/dfe_fl_cfrAux.h>

/** ============================================================================
 *   @n@b dfeFl_CfrHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr        Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_CfrOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Cfr.
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_CfrObj objDfeCfr[DFE_FL_CFR_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_CfrHandle hDfeCfr[DFE_FL_CFR_PER_CNT];
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

         for(i = 0; i < DFE_FL_CFR_PER_CNT; i++)
         {
		 	 hDfeCfr[i] = dfeFl_CfrOpen(hDfe, &objDfeCfr[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // reset Cfr
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_CfrHwControl(hDfeCfr[0], DFE_FL_CFR_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_CfrHwControl
(
    DfeFl_CfrHandle         hDfeCfr,
    DfeFl_CfrHwControlCmd   ctrlCmd,
    void                        *arg
)
{
    uint32_t i;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {   
    case DFE_FL_CFR_CMD_CFG_INITS:
        dfeFl_CfrConfigInits(hDfeCfr, (DfeFl_SublkInitsConfig *)arg);
        break;
        
    case DFE_FL_CFR_CMD_SET_PREM_SSEL:
    {
        DfeFl_CfrMultSsel *cfg = (DfeFl_CfrMultSsel *)arg;
        
        dfeFl_CfrSetPremSsel(hDfeCfr, cfg->path, cfg->ssel);
        break;
    }
    
    case DFE_FL_CFR_CMD_SET_POSTM_SSEL:
    {
        DfeFl_CfrMultSsel *cfg = (DfeFl_CfrMultSsel *)arg;
        
        dfeFl_CfrSetPostmSsel(hDfeCfr, cfg->path, cfg->ssel);
        break;
    }
    
    case DFE_FL_CFR_CMD_SET_PREGAIN:
    {
    	DfeFl_CfrMultGain *cfg = (DfeFl_CfrMultGain *)arg;

        dfeFl_CfrSetPreGain(hDfeCfr, cfg->path, cfg->gain);
        break;
    }

    case DFE_FL_CFR_CMD_SET_POSTGAIN:
    {
    	DfeFl_CfrMultGain *cfg = (DfeFl_CfrMultGain *)arg;

        dfeFl_CfrSetPostGain(hDfeCfr, cfg->path, cfg->gain);
        break;
    }

    case DFE_FL_CFR_CMD_SET_PROTPAGAIN:
    {
    	DfeFl_CfrMultGain *cfg = (DfeFl_CfrMultGain *)arg;

        dfeFl_CfrSetProtPAGain(hDfeCfr, cfg->path, cfg->gain);
        break;
    }
    /*
     * interrupt
     */
    case DFE_FL_CFR_CMD_ENB_PDC0_INTR:
    {
    	dfeFl_CfrEnablePdc0Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_ENB_PDC1_INTR:
    {
    	dfeFl_CfrEnablePdc1Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_ENB_AGC_INOUT_INTR:
    {
    	dfeFl_CfrEnableAgcInOutIntr(hDfeCfr, (DfeFl_CfrAgcInOutIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_ENB_AGC_SYNC_INTR:
    {
    	dfeFl_CfrEnableAgcSyncIntr(hDfeCfr, (DfeFl_CfrAgcSyncIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_ENB_DTH_INTR:
    {
    	dfeFl_CfrEnableDthIntr(hDfeCfr, (DfeFl_CfrDthIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_DIS_PDC0_INTR:
    {
    	dfeFl_CfrDisablePdc0Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_DIS_PDC1_INTR:
    {
    	dfeFl_CfrDisablePdc1Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_DIS_AGC_INOUT_INTR:
    {
    	dfeFl_CfrDisableAgcInOutIntr(hDfeCfr, (DfeFl_CfrAgcInOutIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_DIS_AGC_SYNC_INTR:
    {
    	dfeFl_CfrDisableAgcSyncIntr(hDfeCfr, (DfeFl_CfrAgcSyncIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_DIS_DTH_INTR:
    {
    	dfeFl_CfrDisableDthIntr(hDfeCfr, (DfeFl_CfrDthIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_SET_FORCE_PDC0_INTR:
    {
    	dfeFl_CfrSetForcePdc0Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_SET_FORCE_PDC1_INTR:
    {
    	dfeFl_CfrSetForcePdc1Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_SET_FORCE_AGC_INOUT_INTR:
    {
    	dfeFl_CfrSetForceAgcInOutIntr(hDfeCfr, (DfeFl_CfrAgcInOutIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_SET_FORCE_AGC_SYNC_INTR:
    {
    	dfeFl_CfrSetForceAgcSyncIntr(hDfeCfr, (DfeFl_CfrAgcSyncIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_SET_FORCE_DTH_INTR:
    {
    	dfeFl_CfrSetForceDthIntr(hDfeCfr, (DfeFl_CfrDthIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_FORCE_PDC0_INTR:
    {
    	dfeFl_CfrClearForcePdc0Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_FORCE_PDC1_INTR:
    {
    	dfeFl_CfrClearForcePdc1Intr(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_FORCE_AGC_INOUT_INTR:
    {
    	dfeFl_CfrClearForceAgcInOutIntr(hDfeCfr, (DfeFl_CfrAgcInOutIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_FORCE_AGC_SYNC_INTR:
    {
    	dfeFl_CfrClearForceAgcSyncIntr(hDfeCfr, (DfeFl_CfrAgcSyncIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_FORCE_DTH_INTR:
    {
    	dfeFl_CfrClearForceDthIntr(hDfeCfr, (DfeFl_CfrDthIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_PDC0_INTR_STATUS:
    {
    	dfeFl_CfrClearPdc0IntrStatus(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_PDC1_INTR_STATUS:
    {
    	dfeFl_CfrClearPdc1IntrStatus(hDfeCfr, *(DfeFl_CfrPdcIntr *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_AGC_INOUT_INTR_STATUS:
    {
    	dfeFl_CfrClearAgcInOutIntrStatus(hDfeCfr, (DfeFl_CfrAgcInOutIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_AGC_SYNC_INTR_STATUS:
    {
    	dfeFl_CfrClearAgcSyncIntrStatus(hDfeCfr, (DfeFl_CfrAgcSyncIntrCfg *)arg);
    	break;
    }
    case DFE_FL_CFR_CMD_CLR_DTH_INTR_STATUS:
    {
    	dfeFl_CfrClearDthIntrStatus(hDfeCfr, (DfeFl_CfrDthIntrCfg *)arg);
    	break;
    }

    /*
     * luts
     */
    case DFE_FL_CFR_CMD_LUT_SSEL:
    {
        DfeFl_CfrMultSsel *cfg = (DfeFl_CfrMultSsel *)arg;

    	dfeFl_CfrSetLutSsel(hDfeCfr, cfg->path, cfg->ssel);
    	break;
    }
    case DFE_FL_CFR_CMD_HDLY_SSEL:
    {
        DfeFl_CfrMultSsel *cfg = (DfeFl_CfrMultSsel *)arg;

        dfeFl_CfrSetHdlySsel(hDfeCfr, cfg->path, cfg->ssel);
    	break;
    }
    case DFE_FL_CFR_CMD_UPDT_LUT_COEFF:
    {
    	DfeFl_CfrLutCoeffCfg *cfg = (DfeFl_CfrLutCoeffCfg *)arg;

    	for (i = 0; i<cfg->numCoeff; i++)
    		dfeFl_CfrUpdtLutCoeff(hDfeCfr, cfg->location, i, cfg->coeff[i]);
    	break;
    }
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
