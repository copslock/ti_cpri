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
/** @file dfe_fl_cbOpen.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_CbOpen()
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
#include <ti/drv/dfe/dfe_fl_cb.h>

/** ============================================================================
 *   @n@b dfeFl_CbOpen
 *
 *   @b Description
 *   @n The API would open the Cb module in Dfe. It returns a valid
 *      handle which is used for subsequent Cb APIs.
 *
 *   @b Arguments
 *   @verbatim
         hDfe          Valid handle of the Dfe
         pDfeCbObj     Pointer to the Dfe Cb object
         cbNum         Instance number
         pStatus       Pointer for the returning status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @li	Valid DfeFl_CbHandle if open properly
 *   @li	NULL if any error
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_Open() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n   1.    Cb object structure is populated
 *   @n   2.    The status is returned in the status variable. If status
 *              returned is
 *   @li            DFE_FL_SOK             	Valid Dfe Cb handle is returned
 *   @li            DFE_FL_INVPARAMS		The open command fails
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n    1. The status variable
 *   @n    2. Dfe Cb object structure
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
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_CbHandle dfeFl_CbOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_CbObj                *pDfeCbObj,
    DfeFl_InstNum                 cbNum,
    DfeFl_Status                  *pStatus
)
{
    DfeFl_CbHandle hDfeCb;
    
    // check parameters
    if(pStatus == NULL)
    	return NULL;

    if(hDfe == NULL || pDfeCbObj == NULL)
    {
        *pStatus = DFE_FL_INVPARAMS;
        return NULL;
    }
    
    switch(cbNum)
    {
    case DFE_FL_CB_0:
        pDfeCbObj->hDfe = hDfe;
        pDfeCbObj->regs = (DfeFl_CbRegsOvly)((uint32_t)hDfe->regs + DFE_FL_CB_0_OFFSET);
        pDfeCbObj->perNum = cbNum;
        hDfeCb = (DfeFl_CbHandle)pDfeCbObj;
        *pStatus = DFE_FL_SOK;
        break;
        
    default:
        *pStatus = DFE_FL_INVPARAMS;
        pDfeCbObj->hDfe = NULL;
        pDfeCbObj->regs = (DfeFl_CbRegsOvly)NULL;
        pDfeCbObj->perNum = (DfeFl_InstNum)-1;
        hDfeCb = (DfeFl_CbHandle)NULL;
        break;
    }
    
    return hDfeCb;
}

