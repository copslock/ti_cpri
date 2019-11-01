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

/** @file dfe_fl_miscGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_SummerGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_miscAux.h>

/** ============================================================================
 *   @n@b dfeFl_MiscGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Summer module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer     Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_SummerOpen() must be invoked before this call.
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
         DfeFl_MiscObj objDfeMisc[DFE_FL_MISC_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_MiscHandle hDfeMisc[DFE_FL_MISC_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         uint32_t inVal;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_MISC_PER_CNT; i++)
         {
		 	 hDfeMisc[i] = dfeFl_MiscOpen(hDfe, &objDfeMisc[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // check Misc GPIO Bank
         inVal = 0;
         dfeFl_MiscGetHwStatus(hDfeMisc[0], DFE_FL_MISC_QUERY_GET_GPIO_BANK, &inVal);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_MiscGetHwStatus
(
    DfeFl_MiscHandle           hDfeMisc,
    DfeFl_MiscHwStatusQuery    queryId,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    case DFE_FL_MISC_QUERY_GET_GPIO_BANK:
        dfeFl_MiscGpioGetGpioBank(hDfeMisc, (uint32_t *)arg);
        break;
    case DFE_FL_MISC_QUERY_GET_GPIO_PIN:
        {
            DfeFl_MiscGpioPinIo *qry = arg;
            
            dfeFl_MiscGpioGetGpioPin(hDfeMisc, qry->pin, &qry->value);
            break;
        }
                
    case DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTR_STATUS:
        {
            DfeFl_MiscIntrStatus *qry = arg;
            
            dfeFl_MiscGetMasterLowPriIntrStatus(hDfeMisc, qry->intr, &qry->data);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_MASTER_LOWPRI_INTRGRP_STATUS:
        {
            DfeFl_MiscMasterLowPriIntrGroup *qry = arg;
            
            dfeFl_MiscGetMasterLowPriIntrGroupStatus(hDfeMisc, qry);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTR_STATUS:
        {
            DfeFl_MiscIntrStatus *qry = arg;
            
            dfeFl_MiscGetMasterHiPriIntrStatus(hDfeMisc, qry->intr, &qry->data);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_MASTER_HIPRI_INTRGRP_STATUS:
        {
            DfeFl_MiscMasterHiPriIntrGroup *qry = arg;
            
            dfeFl_MiscGetMasterHiPriIntrGroupStatus(hDfeMisc, qry);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_SYNC_INTR_STATUS:
        {
            DfeFl_MiscIntrStatus *qry = arg;
            
            dfeFl_MiscGetSyncIntrStatus(hDfeMisc, qry->intr, &qry->data);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_SYNC_INTRGRP_STATUS:
        {
            DfeFl_MiscSyncIntrGroup *qry = arg;
            
            dfeFl_MiscGetSyncIntrGroupStatus(hDfeMisc, qry);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTR_STATUS:
        {
            DfeFl_MiscIntrStatus *qry = arg;
            
            dfeFl_MiscGetCppDmaDoneIntrStatus(hDfeMisc, qry->intr, &qry->data);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_CPP_DMA_DONE_INTRGRP_STATUS:
        {
            DfeFl_MiscCppIntrGroup *qry = arg;
            
            dfeFl_MiscGetCppDmaDoneIntrGroupStatus(hDfeMisc, qry);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_MISC_INTR_STATUS:
        {
            DfeFl_MiscIntrStatus *qry = arg;
            
            dfeFl_MiscGetMiscIntrStatus(hDfeMisc, qry->intr, &qry->data);
            break;
        }
    case DFE_FL_MISC_QUERY_GET_MISC_INTRGRP_STATUS:
        {
            DfeFl_MiscMiscIntrGroup *qry = arg;
            
            dfeFl_MiscGetMiscIntrGroupStatus(hDfeMisc, qry);
            break;
        }
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
