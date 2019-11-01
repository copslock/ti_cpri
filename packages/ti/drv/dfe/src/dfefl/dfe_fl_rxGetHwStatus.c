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

/** @file dfe_fl_rxGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_RxGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_rxAux.h>

/** ============================================================================
 *   @n@b dfeFl_RxGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Rx module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeRx         Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_RxOpen() must be invoked before this call.
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
         DfeFl_RxObj objDfeRx[DFE_FL_RX_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_RxHandle hDfeRx[DFE_FL_RX_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_RxPowmtrResult RxPmResult;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_RX_PER_CNT; i++)
         {
		 	 hDfeRx[i] = dfeFl_RxOpen(hDfe, &objDfeRx[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // check the Rx powermeter
         RxPmResult.powmtr = 0;
	     dfeFl_RxGetHwStatus(hDfeRx[0], DFE_FL_RX_QUERY_POWMTR_RESULT, &RxPmResult);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_RxGetHwStatus
(
    DfeFl_RxHandle             hDfeRx,
    DfeFl_RxHwStatusQuery      queryId,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    /*
     *  Signal Generator and CheckSum
     */
    case DFE_FL_RX_QUERY_CHKSUM_RESULT:
    {
        dfeFl_RxQueryChksumResult(hDfeRx, (uint32_t *)arg);
        break;
    }
    /*
     *  Power meter
     */
    case DFE_FL_RX_QUERY_POWMTR_INTR_STATUS:
    {
        dfeFl_RxQueryPowmtrInterruptStatus(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }
    case DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_MISS:
    {
        dfeFl_RxGetPowmtrHandshakeReadMiss(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }
    case DFE_FL_RX_QUERY_POWMTR_HANDSHAKE_READ_ACK:
    {
        dfeFl_RxGetPowmtrHandshakeReadAck(hDfeRx, (DfeFl_RxPowmtrGeneric *)arg);
        break;
    }
    case DFE_FL_RX_QUERY_POWMTR_RESULT:
    {
        dfeFl_RxQueryPowmtrResult(hDfeRx, (DfeFl_RxPowmtrResult *)arg);
        break;
    }
    case DFE_FL_RX_QUERY_GET_POWMTR:
    {
    	dfeFl_RxGetPowmtr(hDfeRx, (DfeFl_RxPowmtrConfig *)arg);
        break;
    }
    
    /*
     *  Rx NCO
     */
    case DFE_FL_RX_QUERY_GET_NCO_BYPASS:
    {
    	dfeFl_RxGetNcoBypass(hDfeRx, (uint32_t *)arg);
    	break;
    }
    case DFE_FL_RX_QUERY_NCO_SSEL:
    {
        dfeFl_RxGetNcoSsel(hDfeRx, (DfeFl_RxNcoSsel *)arg);
        break;
    }
     
    /*
     *  Rx EQR
     */
    case DFE_FL_RX_QUERY_EQR_SHIFT:
    {
        dfeFl_RxGetEqrShift(hDfeRx, (DfeFl_RxEqrGeneric *)arg);
        break;
    }
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
