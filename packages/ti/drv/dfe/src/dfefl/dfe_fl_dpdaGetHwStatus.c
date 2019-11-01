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

/** @file dfe_fl_dpdaGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_DpdaGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_dpdaAux.h>

/** ============================================================================
 *   @n@b dfeFl_DpdaGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Dpda module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpda       Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_DpdaOpen() must be invoked before this call.
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
         DfeFl_DpdaObj objDfeDpda[DFE_FL_DPDA_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_DpdaHandle hDfeDpda[DFE_FL_DPDA_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         int dpdaStatus;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_DPDA_PER_CNT; i++)
         {
		 	 hDfeDpda[i] = dfeFl_DpdaOpen(hDfe, &objDfeDpda[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // check interrupt status
         dfeFl_DpdaGetHwStatus(hDfeDpda[0], DFE_FL_DPDA_QUERY_GET_INT_PROCESSED_INTR_STATUS, &dpdaStatus);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_DpdaGetHwStatus
(
    DfeFl_DpdaHandle         hDfeDpda,
    DfeFl_DpdaHwStatusQuery  queryId,
    void                        *arg
)
{
	uint32_t i;
	DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
	/**
	 * interrupt
	 */
    case DFE_FL_DPDA_QUERY_GET_INT_READ_COMPLETE_INTR_STATUS:
    {
    	dfeFl_DpdaGetIntreadcompleteIntrStatus(hDfeDpda, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_QUERY_GET_INT_PROCESSED_INTR_STATUS:
    {
    	dfeFl_DpdaGetIntprocessedIntrStatus(hDfeDpda, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_DPDA_QUERY_GET_IDLE_INTR_STATUS:
    {
      dfeFl_DpdaGetIdleIntrStatus(hDfeDpda, (uint32_t *)arg);
      break;
    }
	/**
	 * lut
	 */
    case DFE_FL_DPDA_QUERY_GET_LUT_MASTER:
    {
    	DfeFl_DpdaGeneric *cfg = (DfeFl_DpdaGeneric *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaGetLutMaster(hDfeDpda, i, (cfg->data+i));
    	break;
    }
    case DFE_FL_DPDA_QUERY_GET_LUT:
    {
    	DfeFl_DpdaLut *cfg = (DfeFl_DpdaLut *)arg;
    	for (i=0; i<cfg->lut.numEntry; i++)
    		dfeFl_DpdaGetLut(hDfeDpda, cfg->idx, i, (cfg->lut.data+i));
    	break;
    }

	/*
	 * iram
	 */
    case DFE_FL_DPDA_QUERY_GET_IRAM:
    {
    	DfeFl_DpdaGeneric *cfg = (DfeFl_DpdaGeneric *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaGetIram(hDfeDpda, i, (cfg->data+i));
    	break;
    }

	/*
	 * preg
	 */
    case DFE_FL_DPDA_QUERY_GET_PREG:
    {
    	DfeFl_DpdaPreg *cfg = (DfeFl_DpdaPreg *)arg;
    	for (i=0; i<cfg->numEntry; i++)
    		dfeFl_DpdaGetPreg(hDfeDpda, cfg->idx, i, (cfg->iedata+i), (cfg->qdata+i));
    	break;
    }

    /*
     * scalar
     */
    case DFE_FL_DPDA_QUERY_GET_SCALAR:
    {
        DfeFl_DpdaPreg *cfg = (DfeFl_DpdaPreg *)arg;
        dfeFl_DpdaGetScalar(hDfeDpda, cfg->idx, cfg->iedata, cfg->qdata);
    }

    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
