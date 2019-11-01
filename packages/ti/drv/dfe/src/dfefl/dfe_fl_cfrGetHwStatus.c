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

/** @file dfe_fl_cfrGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_CfrGetHwStatus()
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
 *   @n@b dfeFl_CfrGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Cfr module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr        Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
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
 *   @n  None
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
         DfeFl_CfrBusyStatus CfrStatus;

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

         // check cfr busy status
	     CfrStatus.path = DFE_FL_CFR_PATH_ALL;
	     CfrStatus.status = 0;
         dfeFl_CfrGetHwStatus(hDfeCfr[cfrDevice], DFE_FL_CFR_QUERY_GET_BUSY_STATUS, &CfrStatus);

         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_CfrGetHwStatus
(
    DfeFl_CfrHandle         hDfeCfr,
    DfeFl_CfrHwStatusQuery  queryId,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    case DFE_FL_CFR_QUERY_GET_BUSY_STATUS:
    {
    	DfeFl_CfrBusyStatus *cfg = (DfeFl_CfrBusyStatus *)arg;

    	DfeFl_CfrGetBusyStatus(hDfeCfr, cfg->path, &cfg->status);
    	break;
    }
    case DFE_FL_CFR_QUERY_GET_PDC0_INTR_STATUS:
    {
    	DfeFl_CfrPdcIntrStatus *cfg = (DfeFl_CfrPdcIntrStatus *)arg;

    	dfeFl_CfrGetPdc0IntrStatus(hDfeCfr, cfg->intrcfg, &cfg->status);
    	break;
    }
    case DFE_FL_CFR_QUERY_GET_PDC1_INTR_STATUS:
    {
    	DfeFl_CfrPdcIntrStatus *cfg = (DfeFl_CfrPdcIntrStatus *)arg;

    	dfeFl_CfrGetPdc1IntrStatus(hDfeCfr, cfg->intrcfg, &cfg->status);
    	break;
    }
    case DFE_FL_CFR_QUERY_GET_AGC_INOUT_INTR_STATUS:
    {
    	DfeFl_CfrAgcInOutIntrStatus *cfg = (DfeFl_CfrAgcInOutIntrStatus *)arg;

    	dfeFl_CfrGetAgcInOutIntrStatus(hDfeCfr, &cfg->intrcfg, &cfg->status);
    	break;
    }
    case DFE_FL_CFR_QUERY_GET_AGC_SYNC_INTR_STATUS:
    {
    	DfeFl_CfrAgcSyncIntrStatus *cfg = (DfeFl_CfrAgcSyncIntrStatus *)arg;

    	dfeFl_CfrGetAgcSyncIntrStatus(hDfeCfr, &cfg->intrcfg, &cfg->status);
    	break;
    }
    case DFE_FL_CFR_QUERY_GET_DTH_INTR_STATUS:
    {
    	DfeFl_CfrDthIntrStatus *cfg = (DfeFl_CfrDthIntrStatus *)arg;

    	dfeFl_CfrGetDthIntrStatus(hDfeCfr, &cfg->intrcfg, &cfg->status);
    	break;
    }
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
