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
/** @file dfe_fl_jesdOpen.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_JesdOpen()
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
#include <ti/drv/dfe/dfe_fl_jesd.h>

/** ============================================================================
 *   @n@b dfeFl_JesdOpen
 *
 *   @b Description
 *   @n The API would open the Jesd module in Dfe. It returns a valid
 *      handle which is used for subsequent Jesd APIs.
 *
 *   @b Arguments
 *   @verbatim
         hDfe           Valid handle of the Dfe
         pDfeJesdObj    Pointer to the Dfe Jesd object
         perNum         Instance number
         pStatus        Pointer for the returning status
     @endverbatim
 *
 *   <b> Return Value </b>
 *   @li	Valid DfeFl_JesdHandle if open properly
 *   @li	NULL if any error
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_Open() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n   1.    Jesd object structure is populated
 *   @n   2.    The status is returned in the status variable. If status
 *              returned is
 *   @li            DFE_FL_SOK             	Valid Dfe Jesd handle is returned
 *   @li            DFE_FL_INVPARAMS		The open command fails
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n    1. The status variable
 *   @n    2. Dfe Jesd object structure
 *
 *   @b Example
 *   @verbatim

         DfeFl_Context dfeCtx;
         DfeFl_Param dfeParam;
         DfeFl_Obj objDfe;
         DfeFl_JesdObj objDfeJesd[DFE_FL_JESD_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_JesdHandle hDfeJesd[DFE_FL_JESD_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_JESD_PER_CNT; i++)
         {
		 	 hDfeJesd[i] = dfeFl_JesdOpen(hDfe, &objDfeJesd[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_JesdHandle dfeFl_JesdOpen
(
    DfeFl_Handle               hDfe,
    DfeFl_JesdObj              *pDfeJesdObj,
    DfeFl_InstNum                 perNum,
    DfeFl_Status                  *pStatus
)
{
    DfeFl_JesdHandle hDfeJesd;
    
    // check parameters
    if(pStatus == NULL)
    	return NULL;

    if(hDfe == NULL || pDfeJesdObj == NULL)
    {
        *pStatus = DFE_FL_INVPARAMS;
        return NULL;
    }
    
    switch(perNum)
    {
    case DFE_FL_JESD_0:
        pDfeJesdObj->hDfe = hDfe;
        pDfeJesdObj->regs = (DfeFl_JesdRegsOvly)((uint32_t)hDfe->regs + DFE_FL_JESD_0_OFFSET);
        pDfeJesdObj->perNum = perNum;
        hDfeJesd = (DfeFl_JesdHandle)pDfeJesdObj;
        *pStatus = DFE_FL_SOK;
        break;
        
    default:
        *pStatus = DFE_FL_INVPARAMS;
        pDfeJesdObj->hDfe = NULL;
        pDfeJesdObj->regs = (DfeFl_JesdRegsOvly)NULL;
        pDfeJesdObj->perNum = (DfeFl_InstNum)-1;
        hDfeJesd = (DfeFl_JesdHandle)NULL;
        break;
    }
    
    return hDfeJesd;
}

