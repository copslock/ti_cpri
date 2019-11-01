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

/** @file dfe_fl_bbGetHwStatus.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_BbGetHwStatus()
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
#include <ti/drv/dfe/dfe_fl_bbAux.h>

/** ============================================================================
 *   @n@b dfeFl_BbGetHwStatus
 *
 *   @b Description
 *   @n Retrieve status or configuration information from Dfe Bb module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb     Valid handle
         queryId    The queryId to this API
         arg        The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b> DfeFl_Status
 *   @li                   DFE_FL_SOK               - Status info return successful
 *   @li                   DFE_FL_INVQUERY     - Invalid query
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_BbOpen() must be invoked before this call.
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
         DfeFl_BbObj objDfeBb[DFE_FL_BB_PER_CNT];
         DfeFl_Handle hDfe;
         DfeFl_BbHandle hDfeBb[DFE_FL_BB_PER_CNT];
         DfeFl_Status status = DFE_FL_SOK;
         DfeFl_BbTxRxGainCarrierTypeData BbGainCtStatus;

         // open DFE
         dfeFl_Init(&dfeCtx);
         dfeParam.flags = 0;
         hDfe = dfeFl_Open(&objDfe, 0, &dfeParam, &status);
         if(status != DFE_FL_SOK)
         {
         	 return FAIL;
         }

         for(i = 0; i < DFE_FL_BB_PER_CNT; i++)
         {
		 	 hDfeBb[i] = dfeFl_BbOpen(hDfe, &objDfeBb[i], i, &status);
		 	 if(status != DFE_FL_SOK)
			 {
				 return FAIL;
			 }
         }

         // check TxGain update status
         BbGainCtStatus.ct = 4;
         BbGainCtStatus.data = 0;
         do
         {
         	 dfeFl_BbGetHwStatus(hDfeBb[0], DFE_FL_BB_QUERY_TXGAIN_UPDATE_STATUS, &BbGainCtStatus);
         }while(BbGainCtStatus.data);

         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_BbGetHwStatus
(
    DfeFl_BbHandle             hDfeBb,
    DfeFl_BbHwStatusQuery      queryId,
    void                        *arg
)
{
    DfeFl_Status err = DFE_FL_SOK;

    switch(queryId)
    {
    // loopback config
    case DFE_FL_BB_QUERY_AID_LOOPBACK_CFG:
        dfeFl_BbQueryAidLoopbackConfig(hDfeBb, (uint32_t *)arg);
        break;
    // loopback config
    case DFE_FL_BB_QUERY_LOOPBACK_CFG:
        dfeFl_BbQueryLoopbackConfig(hDfeBb, (DfeFl_BbLoopbackConfig *)arg);
        break;
    // capture buffer config
    case DFE_FL_BB_QUERY_CAPBUFF_CFG:
        dfeFl_BbQueryCapBuffConfig(hDfeBb, (DfeFl_BbCapBuffConfig *)arg);
        break;
    // test signal generation config
    case DFE_FL_BB_QUERY_TESTGEN_CFG:
        dfeFl_BbQueryTestGenConfig(hDfeBb, (DfeFl_BbTestGenConfig *)arg);
        break;
    case DFE_FL_BB_QUERY_TESTGEN_SSEL:
    {
        DfeFl_BbTestGenSsel *qry = (DfeFl_BbTestGenSsel *) arg;
        
        dfeFl_BbGetTestGenSsel(hDfeBb, qry->tgDev, &qry->ssel);
        break;
    }
    case DFE_FL_BB_QUERY_CHKSUM_SSEL:
    {
    	dfeFl_BbGetChksumSsel(hDfeBb, (DfeFl_BbChksumSsel *)arg);
    	break;
    }
    case DFE_FL_BB_QUERY_CHKSUM_RESULT:
    {
        dfeFl_BbGetChksumResult(hDfeBb, (DfeFl_BbChksumResult *)arg);
        break;
    }
    case DFE_FL_BB_QUERY_CT_UL_SYNC_STROBE:
    {
        DfeFl_BbCarrierTypeUlSyncStrobeConfig *qry = (DfeFl_BbCarrierTypeUlSyncStrobeConfig *)arg;

        qry->strobe = dfeFl_BbGetCarrierTypeUlSyncStrobe(hDfeBb, qry->ct);
        break;        
    }
    case DFE_FL_BB_QUERY_AID_ULSTROBE_DLY:
    {
        DfeFl_BbAidUlStrobeDelayConfig *qry = (DfeFl_BbAidUlStrobeDelayConfig *)arg;

        qry->dly = dfeFl_BbGetAidUlStrobeDelay(hDfeBb, qry->ct);
        break;        
    }
        
    case DFE_FL_BB_QUERY_TXGAIN_UPDATE_STATUS:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbGetTxGainUpdateStatus(hDfeBb, qry->ct, &qry->data);
        break;        
    }
    // config sync selection for tx gain update
    case DFE_FL_BB_QUERY_TXGAIN_SSEL:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbGetTxGainCtSsel(hDfeBb, qry->ct, &qry->data);
        break;
    }    
    case DFE_FL_BB_QUERY_TXGAIN_INTR_STATUS:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;
        
        dfeFl_BbQueryTxGainIntrStatus(hDfeBb, qry->ct, &qry->data);
        break;
    }            
            
    // TX TDD timer config
    case DFE_FL_BB_QUERY_TXTDD_CFG:
        dfeFl_BbQueryTxTddConfig(hDfeBb, (DfeFl_BbTddConfig *)arg);
        break;
    // TX TDD timer ssel
    case DFE_FL_BB_QUERY_TXTDD_SSEL:
        dfeFl_BbGetTxTddSsel(hDfeBb, (uint32_t *)arg);
        break;
    // RX TDD timer config
    case DFE_FL_BB_QUERY_RXTDD_CFG:
        dfeFl_BbQueryRxTddConfig(hDfeBb, (DfeFl_BbTddConfig *)arg);
        break;
    // RX TDD timer ssel
    case DFE_FL_BB_QUERY_RXTDD_SSEL:
        dfeFl_BbGetRxTddSsel(hDfeBb, (uint32_t *)arg);
        break;
    
    /* TX power meter queries
     */    
    case DFE_FL_BB_QUERY_TXPM_CFG:
    {
        DfeFl_BbPowerMeterConfig *qry = (DfeFl_BbPowerMeterConfig *)arg;
        dfeFl_BbQueryTxpmConfig(hDfeBb, qry);
        break;
    }
    case DFE_FL_BB_QUERY_TXPM_SSEL:
    {
        DfeFl_BbPowerMeterSsel *qry = (DfeFl_BbPowerMeterSsel *)arg;
        dfeFl_BbGetTxpmSsel(hDfeBb, qry->pmId, &qry->ssel);
        break;
    }
    case DFE_FL_BB_QUERY_DIS_TXPM_UPDATE:
        dfeFl_BbQueryDisableTxpmUpdate(hDfeBb, (DfeFl_BbDisablePowMterUpdateConfig *)arg);
        break;
    case DFE_FL_BB_QUERY_TXPM_INTR_STATUS:
    {
        DfeFl_BbPowMtrIntrStatus *qry = (DfeFl_BbPowMtrIntrStatus *)arg;
        
        dfeFl_BbQueryTxpmIntrStatus(hDfeBb, qry->pmId, &qry->status);
        break;
    }
    case DFE_FL_BB_QUERY_TXPM_RESULT:
    {
    	DfeFl_BbPowMtrResult *qry = (DfeFl_BbPowMtrResult *)arg;

    	dfeFl_BbQueryTxpmResult(hDfeBb, qry);
    	break;
    }
    
    /* RX power meter queries
     */    
    case DFE_FL_BB_QUERY_RXPM_CFG:
    {
        DfeFl_BbPowerMeterConfig *qry = (DfeFl_BbPowerMeterConfig *)arg;
        dfeFl_BbQueryRxpmConfig(hDfeBb, qry);
        break;
    }
    case DFE_FL_BB_QUERY_RXPM_SSEL:
    {
        DfeFl_BbPowerMeterSsel *qry = (DfeFl_BbPowerMeterSsel *)arg;
        dfeFl_BbGetRxpmSsel(hDfeBb, qry->pmId, &qry->ssel);
        break;
    }
    case DFE_FL_BB_QUERY_DIS_RXPM_UPDATE:
        dfeFl_BbQueryDisableRxpmUpdate(hDfeBb, (DfeFl_BbDisablePowMterUpdateConfig *)arg);
        break;
    case DFE_FL_BB_QUERY_RXPM_INTR_STATUS:
    {
        DfeFl_BbPowMtrIntrStatus *qry = (DfeFl_BbPowMtrIntrStatus *)arg;
        
        dfeFl_BbQueryRxpmIntrStatus(hDfeBb, qry->pmId, &qry->status);
        break;
    }
    case DFE_FL_BB_QUERY_RXPM_RESULT:
    {
    	DfeFl_BbPowMtrResult *qry = (DfeFl_BbPowMtrResult *)arg;

    	dfeFl_BbQueryRxpmResult(hDfeBb, qry);
    	break;
    }
    /* Antenna Calibration queries
     */    
    case DFE_FL_BB_QUERY_ANTCAL_GLOBAL_CFG:
        dfeFl_BbQueryAntcalGlobalConfig(hDfeBb, (DfeFl_BbAntCalGlobalConfig *)arg);
        break;
    case DFE_FL_BB_QUERY_ANTCAL_CFG:
        dfeFl_BbQueryAntcalConfig(hDfeBb, (DfeFl_BbAntCalConfig *)arg);
        break;
    case DFE_FL_BB_QUERY_ANTCAL_RESULT:
    	break;

    case DFE_FL_BB_QUERY_GENERAL_INTR_STATUS:
    {
        DfeFl_BbGeneralIntrQuery *qry = (DfeFl_BbGeneralIntrQuery *)arg;
        dfeFl_BbQueryGeneralIntrStatus(hDfeBb, qry->intr, &qry->result);
        break;
    }
    case DFE_FL_BB_QUERY_GENERAL_INTRGRP_STATUS:
        dfeFl_BbQueryGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    case DFE_FL_BB_QUERY_RXGAIN_UPDATE_STATUS:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbGetRxGainUpdateStatus(hDfeBb, qry->ct, &qry->data);
        break;        
    }
    // config sync selection for rx gain update
    case DFE_FL_BB_QUERY_RXGAIN_SSEL:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbGetRxGainCtSsel(hDfeBb, qry->ct, &qry->data);
        break;
    }    
    // query rx gain update interrupt status
    case DFE_FL_BB_QUERY_RXGAIN_INTR_STATUS:
    {
        DfeFl_BbTxRxGainCarrierTypeData *qry = (DfeFl_BbTxRxGainCarrierTypeData *)arg;
        
        dfeFl_BbQueryRxGainIntrStatus(hDfeBb, qry->ct, &qry->data);
        break;
    }            
    // config beAGC globals
    case DFE_FL_BB_QUERY_BEAGC_GLOBAL_CFG:
    {
        DfeFl_BbBeagcGlobalConfig *qry = (DfeFl_BbBeagcGlobalConfig *)arg;
        
        dfeFl_BbQueryBeagcGlobalConfig(hDfeBb, qry);
        break;
    }
    // config a beAGC control loop
    case DFE_FL_BB_QUERY_BEAGC_CFG:
    {
        DfeFl_BbBeagcConfig *qry = (DfeFl_BbBeagcConfig *)arg;
        
        dfeFl_BbQueryBeagcConfig(hDfeBb, qry);
        break;
    }
    case DFE_FL_BB_QUERY_BEAGC_SSEL:
    {
        DfeFl_BbBeagcSsel *qry = (DfeFl_BbBeagcSsel *)arg;
        
        dfeFl_BbGetBeagcSsel(hDfeBb, qry->beagc, &qry->ssel);
        break;
    }
    // config Rx Notch filter globals
    case DFE_FL_BB_QUERY_RXNOTCH_GLOBAL_CFG:
    {
        DfeFl_BbRxNotchGlobalConfig *qry = (DfeFl_BbRxNotchGlobalConfig *)arg;
        
        dfeFl_BbQueryRxNotchGlobalConfig(hDfeBb, &qry->carrierType, &qry->tddMode);
        break;
    }
    case DFE_FL_BB_QUERY_RXNOTCH_SSEL:
    {
        dfeFl_BbGetRxNotchSsel(hDfeBb, (uint32_t *)arg);
        break;
    }
    // config a Rx Notch filter
    case DFE_FL_BB_QUERY_RXNOTCH_CFG:
    {
        DfeFl_BbRxNotch *qry = (DfeFl_BbRxNotch *)arg;
        
        // query notch filter    
        dfeFl_BbQueryRxNotchConfig(hDfeBb, qry);
        
        break;
    }

    default:
        err = DFE_FL_INVQUERY;
        break;
    }
    
    return err;
}

