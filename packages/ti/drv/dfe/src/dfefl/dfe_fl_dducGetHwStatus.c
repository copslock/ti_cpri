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

/** @file dfe_fl_dducGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DducGetHwStatus()
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
 *   @n@b dfeFl_DducGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Dduc module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc       Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
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
 *   @n  None
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
         int SyncDelay;

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

         // check the sync delay
         dfeFl_DducGetHwStatus(hDfeDduc[0], DFE_FL_DDUC_QUERY_SYNC_DELAY, &SyncDelay);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DducGetHwStatus
(
    DfeFl_DducHandle           hDfeDduc,
    DfeFl_DducHwStatusQuery    queryId,
    void                        *arg
)
{
//    int i;
	DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    case DFE_FL_DDUC_QUERY_DIRECT:
    	{
    		dfeFl_DducGetDirection(hDfeDduc, (uint32_t *)arg);
    		break;
    	}
    case DFE_FL_DDUC_QUERY_SYNC_DELAY:
    	{
    		dfeFl_DducGetSyncDelay(hDfeDduc, (uint32_t *)arg);
    		break;
    	}
    case DFE_FL_DDUC_QUERY_GET_FRW_PHASE:
    	{
            DfeFl_DducFrwPhaseConfig *cfg = arg;

            dfeFl_DducGetFrwPhase(hDfeDduc, &cfg->phase[0]);
            break;
    	}
    case DFE_FL_DDUC_QUERY_CIC_CONFIG:
    	{
    		dfeFl_DducGetCicConfig(hDfeDduc, (DfeFl_DducCicCfg *)arg);
    		break;
    	}
    case DFE_FL_DDUC_QUERY_TX_CHKSUM_RESULT:
        {
        	dfeFl_DducQueryTxChksumResult(hDfeDduc, (uint32_t *)arg);
        	break;
        }
    case DFE_FL_DDUC_QUERY_RX_CHKSUM_RESULT:
		{
			dfeFl_DducQueryRxChksumResult(hDfeDduc, (uint32_t *)arg);
			break;
		}

	/**
	 * Intr Mask
	 */
    case DFE_FL_DDUC_QUERY_GET_CICOV_INTR_STATUS:
		{
			DfeFl_DducCicovIntrStatus *qry = arg;

			dfeFl_DducGetCicovIntrStatus(hDfeDduc, qry->intr, &qry->data);
			break;
		}
    case DFE_FL_DDUC_QUERY_GET_HOPROLLOVER_INTR_STATUS:
		{
			dfeFl_DducGetHopRolloverIntrStatus(hDfeDduc, (uint32_t *)arg);
			break;
		}
    case DFE_FL_DDUC_QUERY_GET_HOPHALFWAY_INTR_STATUS:
		{
			dfeFl_DducGetHopHalfwayIntrStatus(hDfeDduc, (uint32_t *)arg);
			break;
		}
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
