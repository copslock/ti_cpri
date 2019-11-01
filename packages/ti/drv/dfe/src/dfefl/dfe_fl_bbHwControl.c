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

/** @file dfe_fl_bbHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_BbHwControl()
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
 *   @n@b dfeFl_BbHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeBb     Valid handle
         ctrlCmd    The command to this API
         arg        The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
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
 *   @n  The hardware registers of Dfe Bb.
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
         DfeFl_SublkInitsConfig inits;

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

         // reset BB
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_BbHwControl(hDfeBb[0], DFE_FL_BB_CMD_CFG_INITS, &inits);

         return PASS;
     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_BbHwControl
(
    DfeFl_BbHandle             hDfeBb,
    DfeFl_BbHwControlCmd       ctrlCmd,
    void                        *arg
)
{
    uint32_t i;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {
    case DFE_FL_BB_CMD_ENB_AID_LOOPBACK:
        dfeFl_BbEnableAidLoopback(hDfeBb);
        break;        
    case DFE_FL_BB_CMD_DIS_AID_LOOPBACK:
        dfeFl_BbDisableAidLoopback(hDfeBb);
        break;        
    case DFE_FL_BB_CMD_CFG_INITS:
        dfeFl_BbConfigInits(hDfeBb, (DfeFl_SublkInitsConfig *)arg);
        break;
    // loopback config
    case DFE_FL_BB_CMD_CFG_LOOPBACK:
        dfeFl_BbConfigLoopback(hDfeBb, (DfeFl_BbLoopbackConfig *)arg);
        break;
    // capture buffer config
    case DFE_FL_BB_CMD_CFG_CAPBUFF:
        dfeFl_BbConfigCapBuff(hDfeBb, (DfeFl_BbCapBuffConfig *)arg);
        break;
    // test signal generation config
    case DFE_FL_BB_CMD_CFG_TESTGEN:
        dfeFl_BbConfigTestGen(hDfeBb, (DfeFl_BbTestGenConfig *)arg);
        break;
    case DFE_FL_BB_CMD_SET_TESTGEN_SSEL:
    {
        DfeFl_BbTestGenSsel *cfg = (DfeFl_BbTestGenSsel *) arg;
        
        dfeFl_BbSetTestGenSsel(hDfeBb, cfg->tgDev, cfg->ssel);
        break;
    }
    case DFE_FL_BB_CMD_CFG_CHKSUM:
    {
        dfeFl_BbConfigChksum(hDfeBb, (DfeFl_BbChksumConfig *)arg);
        break;
    }
    case DFE_FL_BB_CMD_SET_CHKSUM_SSEL:
    {
        dfeFl_BbSetChksumSsel(hDfeBb, (DfeFl_BbChksumSsel *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CFG_CT_UL_SYNC_STROBE:
    {
        DfeFl_BbCarrierTypeUlSyncStrobeConfig *cfg = (DfeFl_BbCarrierTypeUlSyncStrobeConfig *)arg;
        
        dfeFl_BbSetCarrierTypeUlSyncStrobe(hDfeBb, cfg->ct, cfg->strobe);
        break;
    }
    case DFE_FL_BB_CMD_CFG_AID_ULSTROBE_DLY:
    {
        DfeFl_BbAidUlStrobeDelayConfig *cfg = (DfeFl_BbAidUlStrobeDelayConfig *)arg;
        
        dfeFl_BbSetAidUlStrobeDelay(hDfeBb, cfg->ct, cfg->dly);
        break;
    }
    
    case DFE_FL_BB_CMD_CFG_TXIF_AXC:
    {
    	DfeFl_BbTxifAxcConfig *cfg = (DfeFl_BbTxifAxcConfig *)arg;

    	for(i = 0; i < cfg->numAxCs; i++)
        {
    		dfeFl_BbCfgTxifAxc(hDfeBb, cfg->txifAxc[i].axc, &cfg->txifAxc[i].txif);
        }
        break;
    }
    case DFE_FL_BB_CMD_CFG_RXIF_AXC:
    {
    	DfeFl_BbRxifAxcConfig *cfg = (DfeFl_BbRxifAxcConfig *)arg;

    	for(i = 0; i < cfg->numAxCs; i++)
        {
    		dfeFl_BbCfgRxifAxc(hDfeBb, cfg->rxifAxc[i].axc, &cfg->rxifAxc[i].rxif);
        }
        break;
    }
    // config sync selection for tx gain update        
    case DFE_FL_BB_CMD_SET_TXGAIN_SSEL:
    {
        DfeFl_BbTxRxGainCarrierTypeData *cfg = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbSetTxGainCtSsel(hDfeBb, cfg->ct, cfg->data);        
        break;
    }
    // TX gain config
    case DFE_FL_BB_CMD_UPD_TXGAIN:
    {
        DfeFl_BbTxGainConfig *cfg = (DfeFl_BbTxGainConfig *)arg;
/*            
        // collect carrier types involving        
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->txifAxc[cfg->axcGain[i].axc].carrierType;
            ctBit[1 << ct] |= 1;
        }
        // wait till no pending update
        do 
        {
            data = hDfeBb->regs->txgain_update_status;
        } while((ctBits & data) != 0);
        // turn off sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->txifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetTxGainCtSsel(hDfeBb, ct, <TODO: turn off sync>); 
        }
*/        
        for(i = 0; i < cfg->numAxCs; i++)
        {
            dfeFl_BbUpdateTxGain(hDfeBb, cfg->axcGain[i].axc, cfg->axcGain[i].gainI, cfg->axcGain[i].gainQ);
        }
/*        
        // set sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->txifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetTxGainCtSsel(hDfeBb, ct, hDfeBb->sselTxGainUpdate[ct]); 
        }
        // TODO: issue & wait sync
        
        // turn off sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->txifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetTxGainCtSsel(hDfeBb, ct, <TODO: turn off sync>); 
        }          
*/        
        break;
    }
    case DFE_FL_BB_CMD_ENB_TXGAIN_INTR:
    {    
        dfeFl_BbEnableTxGainIntr(hDfeBb, *(uint32_t *)arg); 
        break;
    }
    case DFE_FL_BB_CMD_DIS_TXGAIN_INTR:
    {    
        dfeFl_BbDisableTxGainIntr(hDfeBb, *(uint32_t *)arg); 
        break;
    }
    case DFE_FL_BB_CMD_CLR_TXGAIN_INTR_STATUS:
    {
        dfeFl_BbClearTxGainIntrStatus(hDfeBb, *(uint32_t *)arg);    
        break;
    }            
    case DFE_FL_BB_CMD_SET_FORCE_TXGAIN_INTR:
    {
        dfeFl_BbSetForceTxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }            
    case DFE_FL_BB_CMD_CLR_FORCE_TXGAIN_INTR:
    {
        dfeFl_BbClearForceTxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }            
    // TX TDD timer config
    case DFE_FL_BB_CMD_CFG_TXTDD:
        dfeFl_BbConfigTxTdd(hDfeBb, (DfeFl_BbTddConfig *)arg);
        break;
    // TX TDD timer ssel
    case DFE_FL_BB_CMD_SET_TXTDD_SSEL:
        dfeFl_BbSetTxTddSsel(hDfeBb, *(uint32_t *)arg);
        break;
    // RX TDD timer config
    case DFE_FL_BB_CMD_CFG_RXTDD:
        dfeFl_BbConfigRxTdd(hDfeBb, (DfeFl_BbTddConfig *)arg);
        break;
    // RX TDD timer ssel
    case DFE_FL_BB_CMD_SET_RXTDD_SSEL:
        dfeFl_BbSetRxTddSsel(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_CFG_TXPM:
    {
        DfeFl_BbPowerMeterConfig *cfg = (DfeFl_BbPowerMeterConfig *)arg;
        dfeFl_BbConfigTxpm(hDfeBb, cfg);
        break;
    }
    case DFE_FL_BB_CMD_SET_TXPM_SSEL:
    {
        DfeFl_BbPowerMeterSsel *cfg = (DfeFl_BbPowerMeterSsel *)arg;
        dfeFl_BbSetTxpmSsel(hDfeBb, cfg->pmId, cfg->ssel);
        break;
    }
    case DFE_FL_BB_CMD_DIS_TXPM_UPDATE:
        dfeFl_BbDisableTxpmUpdate(hDfeBb, (DfeFl_BbDisablePowMterUpdateConfig *)arg);
        break;
    case DFE_FL_BB_CMD_ENB_TXPM_INTR:
    {
        dfeFl_BbEnableTxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_DIS_TXPM_INTR:
    {
        dfeFl_BbDisableTxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CLR_TXPM_INTR_STATUS:
    {
        dfeFl_BbClearTxpmIntrStatus(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_SET_FORCE_TXPM_INTR:
    {
        dfeFl_BbSetForceTxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CLR_FORCE_TXPM_INTR:
    {
        dfeFl_BbClearForceTxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_ENB_TXPM_AUXINTR:
    {
    	dfeFl_BbEnableTxpmAuxIntr(hDfeBb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_BB_CMD_DIS_TXPM_AUXINTR:
    {
    	dfeFl_BbDisableTxpmAuxIntr(hDfeBb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_BB_CMD_CFG_RXPM:
    {
        DfeFl_BbPowerMeterConfig *cfg = (DfeFl_BbPowerMeterConfig *)arg;
        dfeFl_BbConfigRxpm(hDfeBb, cfg);
        break;
    }
    case DFE_FL_BB_CMD_SET_RXPM_SSEL:
    {
        DfeFl_BbPowerMeterSsel *cfg = (DfeFl_BbPowerMeterSsel *)arg;
        dfeFl_BbSetRxpmSsel(hDfeBb, cfg->pmId, cfg->ssel);
        break;
    }
    case DFE_FL_BB_CMD_DIS_RXPM_UPDATE:
        dfeFl_BbDisableRxpmUpdate(hDfeBb, (DfeFl_BbDisablePowMterUpdateConfig *)arg);
        break;
    case DFE_FL_BB_CMD_ENB_RXPM_INTR:
    {
        dfeFl_BbEnableRxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_DIS_RXPM_INTR:
    {
        dfeFl_BbDisableRxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CLR_RXPM_INTR_STATUS:
    {
        dfeFl_BbClearRxpmIntrStatus(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_SET_FORCE_RXPM_INTR:
    {
        dfeFl_BbSetForceRxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CLR_FORCE_RXPM_INTR:
    {
        dfeFl_BbClearForceRxpmIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_ENB_RXPM_AUXINTR:
    {
    	dfeFl_BbEnableRxpmAuxIntr(hDfeBb, *(uint32_t *)arg);
    	break;
    }
    case DFE_FL_BB_CMD_DIS_RXPM_AUXINTR:
    {
        dfeFl_BbDisableRxpmAuxIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CFG_ANTCAL_GLOBAL:
        dfeFl_BbConfigAntcalGlobal(hDfeBb, (DfeFl_BbAntCalGlobalConfig *)arg);
        break;
    case DFE_FL_BB_CMD_CFG_ANTCAL:
        dfeFl_BbConfigAntcal(hDfeBb, (DfeFl_BbAntCalConfig *)arg);
        break;
    case DFE_FL_BB_CMD_ENB_GENERAL_INTR:
        dfeFl_BbEnableGeneralIntr(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_DIS_GENERAL_INTR:
        dfeFl_BbDisableGeneralIntr(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_CLR_GENERAL_INTR_STATUS:
        dfeFl_BbClearGeneralIntrStatus(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTR:
        dfeFl_BbSetForceGeneralIntr(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_CLR_FORCE_GENERAL_INTR:
        dfeFl_BbClearForceGeneralIntr(hDfeBb, *(uint32_t *)arg);
        break;
    case DFE_FL_BB_CMD_ENB_GENERAL_INTRGRP:
        dfeFl_BbEnableGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    /// disable group of BB general interrupts
    case DFE_FL_BB_CMD_DIS_GENERAL_INTRGRP:
        dfeFl_BbDisableGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    /// clear status of group of BB general interrupts
    case DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS:
        dfeFl_BbClearGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    /// force generating group of BB general interrupts
    case DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTRGRP:
        dfeFl_BbSetForceGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    /// clear force generating group of BB general interrupts
    case DFE_FL_BB_CMD_CLR_FORCE_GENERAL_INTRGRP:
        dfeFl_BbClearForceGeneralIntrGroup(hDfeBb, (DfeFl_BbGeneralIntrGroup *)arg);
        break;
    // config sync selection for rx gain update
    case DFE_FL_BB_CMD_SET_RXGAIN_SSEL:
    {
        DfeFl_BbTxRxGainCarrierTypeData *cfg = (DfeFl_BbTxRxGainCarrierTypeData *)arg;

        dfeFl_BbSetRxGainCtSsel(hDfeBb, cfg->ct, cfg->data);        
        break;
    }
    // update RX gain of axc
    case DFE_FL_BB_CMD_UPD_RXGAIN:
    {
        DfeFl_BbRxGainConfig *cfg = (DfeFl_BbRxGainConfig *)arg;
/*            
        // collect carrier types involving        
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->rxifAxc[cfg->axcGain[i].axc].carrierType;
            ctBit[1 << ct] |= 1;
        }
        // wait till no pending update
        do 
        {
            data = hDfeBb->regs->rxgain_update_status;
        } while((ctBits & data) != 0);
        // turn off sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->rxifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetRxGainCtSsel(hDfeBb, ct, <TODO: turn off sync>); 
        }
*/        
        for(i = 0; i < cfg->numAxCs; i++)
        {
            dfeFl_BbUpdateRxGain(hDfeBb, cfg->axcGain[i].axc, cfg->axcGain[i].gainInteger, cfg->axcGain[i].gainFraction);
        }
/*        
        // set sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->rxifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetRxGainCtSsel(hDfeBb, ct, hDfeBb->sselRxGainUpdate[ct]); 
        }
        // TODO: issue & wait sync
        
        // turn off sync
        for(i = 0; i < cfg->numAxCs; i++)
        {
            ct = hDfeBb->txifAxc[cfg->axcGain[i].axc].carrierType;
            dfeFl_BbSetRxGainCtSsel(hDfeBb, ct, <TODO: turn off sync>); 
        }          
*/        
        break;
    }
    // enable rx gain update Intr
    case DFE_FL_BB_CMD_ENB_RXGAIN_INTR:
    {
        dfeFl_BbEnableRxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_DIS_RXGAIN_INTR:
    {
        dfeFl_BbDisableRxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    // clear rx gain update Intr status
    case DFE_FL_BB_CMD_CLR_RXGAIN_INTR_STATUS:
    {
        dfeFl_BbClearRxGainIntrStatus(hDfeBb, *(uint32_t *)arg);
        break;
    }
    // set rx gain update Intr status
    case DFE_FL_BB_CMD_SET_FORCE_RXGAIN_INTR:
    {
        dfeFl_BbSetForceRxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_BB_CMD_CLR_FORCE_RXGAIN_INTR:
    {
        dfeFl_BbClearForceRxGainIntr(hDfeBb, *(uint32_t *)arg);
        break;
    }
    // config beAGC globals
    case DFE_FL_BB_CMD_CFG_BEAGC_GLOBAL:
    {
        DfeFl_BbBeagcGlobalConfig *cfg = (DfeFl_BbBeagcGlobalConfig *)arg;
        
        dfeFl_BbConfigBeagcGlobal(hDfeBb, cfg);
        break;
    }
    // config a beAGC control loop
    case DFE_FL_BB_CMD_CFG_BEAGC:
    {
        DfeFl_BbBeagcConfig *cfg = (DfeFl_BbBeagcConfig *)arg;
        
        dfeFl_BbConfigBeagc(hDfeBb, cfg);
        break;
    }
    case DFE_FL_BB_CMD_SET_BEAGC_SSEL:
    {
        DfeFl_BbBeagcSsel *cfg = (DfeFl_BbBeagcSsel *)arg;
        
        dfeFl_BbSetBeagcSsel(hDfeBb, cfg->beagc, cfg->ssel);
        break;
    }
    // config Rx Notch filter globals
    case DFE_FL_BB_CMD_CFG_RXNOTCH_GLOBAL:
    {
        DfeFl_BbRxNotchGlobalConfig *cfg = (DfeFl_BbRxNotchGlobalConfig *)arg;
        
        dfeFl_BbConfigRxNotchGlobal(hDfeBb, cfg->carrierType, cfg->tddMode);
        break;
    }
    case DFE_FL_BB_CMD_SET_RXNOTCH_SSEL:
    {
        dfeFl_BbSetRxNotchSsel(hDfeBb, *(uint32_t *)arg);
        break;
    }
    // config a Rx Notch filter
    case DFE_FL_BB_CMD_CFG_RXNOTCH:
    {
        DfeFl_BbRxNotch *cfg = (DfeFl_BbRxNotch *)arg;
        
        // disable rxnotch update ssel
        //dfeFl_BbSetRxNotchSsel(hDfeBb, <TODO: turn off ssel>);
            
        // update notch filter    
        dfeFl_BbConfigRxNotch(hDfeBb, cfg);
        
        // set sync
        //dfeFl_BbSetRxNotchSsel(hDfeBb, hDfeBb->sselRxNotchUpdate);
        
        // TODO: issue & wait sync

        // disable rxnotch update ssel
        //dfeFl_BbSetRxNotchSsel(hDfeBb, <TODO: turn off ssel>);
            
        break;
    }
    
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}

