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

/** @file dfe_fl_jesdGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_JesdGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_jesdAux.h>

/** ============================================================================
 *   @n@b dfeFl_JesdGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Jesd module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeJesd       Valid handle
         queryId        The queryId to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_JesdOpen() must be invoked before this call.
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
         DfeFl_JesdObj objDfeJesd[DFE_FL_JESD_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_JesdHandle hDfeJesd[DFE_FL_JESD_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_JesdLaneConfig laneCfg;

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

         // check the lane cfg
         laneCfg.lane = 0;
         dfeFl_JesdGetHwStatus(hDfeJesd[0], DFE_FL_JESDTX_QUERY_GET_LANE_CFG, &laneCfg);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_JesdGetHwStatus
(
    DfeFl_JesdHandle         hDfeJesd,
    DfeFl_JesdHwStatusQuery  queryId,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      TX QUERYs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
	case DFE_FL_JESDTX_QUERY_GET_INITSSEL:
	{
		dfeFl_JesdTxGetInitSsel(hDfeJesd, (uint32_t *)arg);
		break;
	}
    case DFE_FL_JESDTX_QUERY_GET_BB_ERR:
    {
        *(uint32_t *)arg = dfeFl_JesdTxGetBbErr(hDfeJesd);
        break;
    }
    case DFE_FL_JESDTX_QUERY_GET_SYNCN_LOOPBACK:
    {
        DfeFl_JesdTxSyncnLoopbackConfig *qry = (DfeFl_JesdTxSyncnLoopbackConfig *)arg;
        err = dfeFl_JesdTxGetSyncnLoopback(hDfeJesd, qry->link, &qry->syncnLoopback);
        break;
    }    
    case DFE_FL_JESDTX_QUERY_GET_SYSREF_ALGNCNT:
        *(uint32_t *)arg = dfeFl_JesdTxGetSysRefAlignmentCounter(hDfeJesd);
        break;
    case DFE_FL_JESDTX_QUERY_GET_LANE_SYNC_STATE:
    {
        DfeFl_JesdTxLaneSyncStateQuery *qry = (DfeFl_JesdTxLaneSyncStateQuery *)arg;
        qry->state = dfeFl_JesdTxGetSyncState(hDfeJesd, qry->lane);
        break;
    }         
    case DFE_FL_JESDTX_QUERY_GET_FIRST_SYNC_REQUEST:
    {
        DfeFl_JesdTxFirstSyncRequestQuery *qry = (DfeFl_JesdTxFirstSyncRequestQuery *)arg;
        qry->request = dfeFl_JesdTxGetFirstSyncReqStatus(hDfeJesd, qry->link);
        break;
    }         
    case DFE_FL_JESDTX_QUERY_CHKSUM_RESULT:
    {
        DfeFl_JesdChksumResultQuery *qry = (DfeFl_JesdChksumResultQuery *)arg;
        
        err = dfeFl_JesdTxGetChecksumResult(hDfeJesd, qry->chksumLane, &qry->result);
        break;
    }                     
    case DFE_FL_JESDTX_QUERY_GET_LINK_ERR_CNT:
    {               
        DfeFl_JesdLinkErrCntQuery *qry = (DfeFl_JesdLinkErrCntQuery *)arg;
        
        err = dfeFl_JesdTxGetLinkErrCnt(hDfeJesd, qry->link, &qry->errCnt);
        break;
    }        
    case DFE_FL_JESDTX_QUERY_GET_LANE_INTRGRP_STATUS:
    {
        DfeFl_JesdTxLaneIntrs *qry = (DfeFl_JesdTxLaneIntrs *)arg;
        
        err = dfeFl_JesdTxGetLaneIntrsStatus(hDfeJesd, qry);
        break;
    }        
    case DFE_FL_JESDTX_QUERY_GET_SYSREF_INTRGRP_STATUS:
    {
        DfeFl_JesdTxSysrefIntrs *qry = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxGetSysrefIntrsStatus(hDfeJesd, qry);
        break;
    }        
    case DFE_FL_JESDTX_QUERY_GET_SYSREF_MODE:
    {               
        DfeFl_JesdLinkSysrefModeConfig *qry = (DfeFl_JesdLinkSysrefModeConfig *)arg;
        
        err = dfeFl_JesdTxGetSysrefMode(hDfeJesd, qry->link, &qry->mode);
        break;
    }        
    case DFE_FL_JESDTX_QUERY_GET_LANE_CFG:
    {
        DfeFl_JesdLaneConfig *qry = (DfeFl_JesdLaneConfig *)arg;
        
        err = dfeFl_JesdTxGetLaneConfig(hDfeJesd, qry->lane, &qry->laneEnable, &qry->linkAssign, &qry->laneId);
        break;
    }        
    
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      RX QUERYs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
	case DFE_FL_JESDRX_QUERY_GET_INITSSEL:
	{
		dfeFl_JesdRxGetInitSsel(hDfeJesd, (uint32_t *)arg);
		break;
	}
    case DFE_FL_JESDRX_QUERY_GET_SYSREF_ALGNCNT:
    {
        *(uint32_t *)arg = dfeFl_JesdRxGetSysRefAlignmentCounter(hDfeJesd);
        break;        
    }
    case DFE_FL_JESDRX_QUERY_GET_LANE_SYNC_STATE:
    {
        DfeFl_JesdRxLaneSyncStateQuery *qry = (DfeFl_JesdRxLaneSyncStateQuery *)arg;
        err = dfeFl_JesdRxGetSyncState(hDfeJesd, qry);
        break;
    }         
    case DFE_FL_JESDRX_QUERY_CHKSUM_RESULT:
    {
        DfeFl_JesdChksumResultQuery *qry = (DfeFl_JesdChksumResultQuery *)arg;
        
        err = dfeFl_JesdRxGetChecksumResult(hDfeJesd, qry->chksumLane, &qry->result);
        break;
    }                     
    case DFE_FL_JESDRX_QUERY_GET_LINK_ERR_CNT:
    {               
        DfeFl_JesdLinkErrCntQuery *qry = (DfeFl_JesdLinkErrCntQuery *)arg;
        
        err = dfeFl_JesdRxGetLinkErrCnt(hDfeJesd, qry->link, &qry->errCnt);
        break;
    }        
    case DFE_FL_JESDRX_QUERY_GET_LANE_INTRGRP_STATUS:
    {
        DfeFl_JesdRxLaneIntrs *qry = (DfeFl_JesdRxLaneIntrs *)arg;
        
        err = dfeFl_JesdRxGetLaneIntrsStatus(hDfeJesd, qry);
        break;
    }        
    case DFE_FL_JESDRX_QUERY_GET_SYSREF_INTRGRP_STATUS:
    {
        DfeFl_JesdRxSysrefIntrs *qry = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxGetSysrefIntrsStatus(hDfeJesd, qry);
        break;
    }        
    case DFE_FL_JESDRX_QUERY_GET_LANE_TEST_DATA:
    {
        DfeFl_JesdRxLaneTestDataQuery *qry = (DfeFl_JesdRxLaneTestDataQuery *)arg;
        
        err = dfeFl_JesdRxGetLaneTestData(hDfeJesd, qry);
        break;
    }        
    case DFE_FL_JESDRX_QUERY_GET_SYSREF_MODE:
    {               
        DfeFl_JesdLinkSysrefModeConfig *qry = (DfeFl_JesdLinkSysrefModeConfig *)arg;
        
        err = dfeFl_JesdRxGetSysrefMode(hDfeJesd, qry->link, &qry->mode);
        break;
    }        
    case DFE_FL_JESDRX_QUERY_GET_LOOPBACK:
    {
        DfeFl_JesdRxLoopbackConfig *qry = (DfeFl_JesdRxLoopbackConfig *)arg;
        
        dfeFl_JesdRxGetLoopback(hDfeJesd, qry);
        break;
    }
    case DFE_FL_JESDRX_QUERY_GET_LANE_CFG:
    {
        DfeFl_JesdLaneConfig *qry = (DfeFl_JesdLaneConfig *)arg;
        
        err = dfeFl_JesdRxGetLaneConfig(hDfeJesd, qry->lane, &qry->laneEnable, &qry->linkAssign, &qry->laneId);
        break;
    }        
        
    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}
