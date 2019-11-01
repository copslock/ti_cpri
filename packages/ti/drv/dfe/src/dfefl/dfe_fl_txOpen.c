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
/** @file dfe_fl_txOpen.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_TxOpen()
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
#include <ti/drv/dfe/dfe_fl_tx.h>

/** ============================================================================
 *   @n@b dfeFl_TxOpen
 *
 *   @b Description
 *   @n The API would open the Tx module in Dfe. It returns a valid
 *      handle which is used for subsequent Tx APIs.
 *
 *   @b Arguments
 *   @verbatim
         hDfe           Valid handle of the Dfe
         pDfeTxObj      Pointer to the Dfe Tx object
         txNum          Instance number
         pStatus        Pointer for the returning status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @li	Valid DfeFl_TxHandle if open properly
 *   @li	NULL if any error
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_Open() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n   1.    Tx object structure is populated
 *   @n   2.    The status is returned in the status variable. If status
 *              returned is
 *   @li            DFE_FL_SOK             	Valid Dfe Tx handle is returned
 *   @li            DFE_FL_INVPARAMS		The open command fails
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n    1. The status variable
 *   @n    2. Dfe Tx object structure
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
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_TxHandle dfeFl_TxOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_TxObj                *pDfeTxObj,
    DfeFl_InstNum                 txNum,
    DfeFl_Status                  *pStatus
)
{
    DfeFl_TxHandle hDfeTx;
    
    // check parameters
    if(pStatus == NULL)
    	return NULL;

    if(hDfe == NULL || pDfeTxObj == NULL)
    {
        *pStatus = DFE_FL_INVPARAMS;
        return NULL;
    }
    
    switch(txNum)
    {
    case DFE_FL_TX_0:
        pDfeTxObj->hDfe = hDfe;
        pDfeTxObj->regs = (DfeFl_TxRegsOvly)((uint32_t)hDfe->regs + DFE_FL_TX_0_OFFSET);
        pDfeTxObj->perNum = txNum;
        hDfeTx = (DfeFl_TxHandle)pDfeTxObj;
        *pStatus = DFE_FL_SOK;
        break;
        
//    case DFE_FL_TX_1:
//        pDfeTxObj->hDfe = hDfe;
//        pDfeTxObj->regs = (DfeFl_TxRegsOvly)((uint32_t)hDfe->regs + DFE_FL_TX_1_OFFSET);
//        pDfeTxObj->perNum = txNum;
//        hDfeTx = (DfeFl_TxHandle)pDfeTxObj;
//        *pStatus = DFE_FL_SOK;
//        break;

    default:
        *pStatus = DFE_FL_INVPARAMS;
        pDfeTxObj->hDfe = NULL;
        pDfeTxObj->regs = (DfeFl_TxRegsOvly)NULL;
        pDfeTxObj->perNum = (DfeFl_InstNum)-1;
        hDfeTx = (DfeFl_TxHandle)NULL;
        break;
    }
    
    return hDfeTx;
}

