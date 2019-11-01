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

/** @file dfe_fl_miscHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_MiscHwControl()
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
 *   @n@b dfeFl_MiscHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeMisc       Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  dfeFl_MiscOpen() must be invoked before this call.
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  The hardware registers of Dfe Misc.
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
         DfeFl_SublkInitsConfig inits;

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

         // reset Misc
         inits.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         inits.initClkGate = 1;
         inits.initState = 1;
         inits.clearData = 1;
         dfeFl_MiscHwControl(hDfeMisc[0], DFE_FL_MISC_CMD_CFG_INITS, &inits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_MiscHwControl
(
    DfeFl_MiscHandle           hDfeMisc,
    DfeFl_MiscHwControlCmd     ctrlCmd,
    void                        *arg
)
{
    uint32_t i, data;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {   
    case DFE_FL_MISC_CMD_CFG_INITS:
        dfeFl_MiscConfigInits(hDfeMisc, (DfeFl_SublkInitsConfig *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_MEM_MPU_ACCESS:
        dfeFl_MiscSetMemMpuAccess(hDfeMisc, *(uint32_t *)arg);
        break;        
    case DFE_FL_MISC_CMD_CLR_MEM_MPU_ACCESS:
        dfeFl_MiscClearMemMpuAccess(hDfeMisc, *(uint32_t *)arg);
        break;        
    case DFE_FL_MISC_CMD_ISSUE_SYNC:
        {
            DfeFl_MiscSyncGenIssueSyncConfig *cfg = arg;
            
            // MPU?
            if(cfg->syncSig == (uint32_t)DFE_FL_SYNC_GEN_SIG_NEVER)
            {
                break;
            }
            
            // clear sigal status
            dfeFl_MiscClearSyncIntrStatus(hDfeMisc, cfg->syncSig);
            
            // MPU?
            if(cfg->syncSig == (uint32_t)DFE_FL_SYNC_GEN_SIG_MPU_SYNC)
            {
                // generate MPU pulse
                dfeFl_MiscSyncGenSetMpuSync(hDfeMisc, 0);
                dfeFl_MiscSyncGenSetMpuSync(hDfeMisc, 1);
                dfeFl_MiscSyncGenSetMpuSync(hDfeMisc, 0);
            }
            
            // wait?
            if(cfg->waitCnt == DFE_FL_MISC_SYNC_WAITFOREVER)
            {
                do
                {
                    dfeFl_MiscGetSyncIntrStatus(hDfeMisc, cfg->syncSig, &data);
                } while(data == 0);
            }
            else if(cfg->waitCnt != DFE_FL_MISC_SYNC_NOWAIT)
            {
                i = 0;
                data = 0;
                
                do
                {
                    dfeFl_MiscGetSyncIntrStatus(hDfeMisc, cfg->syncSig, &data);
                } while(data == 0 && i < cfg->waitCnt);
                
                if(data == 0)
                {
                    err = DFE_FL_SYNC_TIMEOUT;
                }
            }
            
            break;                        
        }        
    case DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT:
        {
            DfeFl_MiscSyncGenGeneric *cfg = arg;
            
            dfeFl_MiscSyncGenSetOneShot(hDfeMisc, cfg->syncSig, cfg->data);
            break;
        }
    case DFE_FL_MISC_CMD_SET_UL_SIG_FSTROBE:
        {
            DfeFl_MiscSyncGenGeneric *cfg = arg;
            
            dfeFl_MiscSyncGenSetUlSigFstrobe(hDfeMisc, cfg->syncSig, cfg->data);
            break;
        }
    case DFE_FL_MISC_CMD_SET_DL_SIG_FSTART:
        {
            DfeFl_MiscSyncGenGeneric *cfg = arg;
            
            dfeFl_MiscSyncGenSetDlSigFstart(hDfeMisc, cfg->syncSig, cfg->data);
            break;
        }
    case DFE_FL_MISC_CMD_RST_SYNC_CNTR:
        {
            dfeFl_MiscSyncGenResetCounter(hDfeMisc, *(uint32_t *)arg);
            break;
        }
    case DFE_FL_MISC_CMD_CFG_SYNC_CNTR:
        {
            DfeFl_MiscSyncCntrConfig *cfg = arg;
            
            dfeFl_MiscSyncGenConfigCounter(hDfeMisc, cfg);
            break;
        }
    case DFE_FL_MISC_CMD_SET_SYNC_CNTR_SSEL:
        {
            DfeFl_MiscSyncGenCntrSsel *cfg = arg;
            
            dfeFl_MiscSyncGenSetCounterSsel(hDfeMisc, cfg->cntr, cfg->startSsel, cfg->progSsel);
            break;
        }                
    case DFE_FL_MISC_CMD_SET_GPIO_PIN_MUX:
        {
            DfeFl_MiscGpioPinMuxConfig *cfg = arg;
            
            dfeFl_MiscGpioSetPinMux(hDfeMisc, cfg->pin, cfg->mux);
            break;
        }
    case DFE_FL_MISC_CMD_SET_GPIO_SYNC_OUT_SSEL:
        {
            DfeFl_MiscGpioSyncOutSselConfig *cfg = arg;
            
            dfeFl_MiscGpioSetSyncOutSsel(hDfeMisc, cfg->syncout, cfg->ssel);
            break;
        }
    case DFE_FL_MISC_CMD_SET_GPIO_BANK:
        dfeFl_MiscGpioSetGpioBank(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_GPIO_PIN:
        {
            DfeFl_MiscGpioPinIo *cfg = arg;
            
            dfeFl_MiscGpioSetGpioPin(hDfeMisc, cfg->pin, cfg->value);
            break;
        }
    case DFE_FL_MISC_CMD_SET_ARBITER_CMD:
    	dfeFl_MiscArbiterSetCmd(hDfeMisc, *(uint32_t *)arg);
    	break;
    case DFE_FL_MISC_CMD_SET_ARBITER_CMD_PAR:
    	dfeFl_MiscArbiterSetCmdPar(hDfeMisc, *(uint32_t *)arg);
    	break;
    case DFE_FL_MISC_CMD_SET_DVGA_ENABLE:
        dfeFl_MiscDvgaSetEnable(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CFG_DVGA:
        dfeFl_MiscDvgaConfig(hDfeMisc, arg);
        break;
        
    case DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR:
        dfeFl_MiscEnableMasterLowPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTR:
        dfeFl_MiscDisableMasterLowPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MASTER_LOWPRI_INTR:
        dfeFl_MiscSetForceMasterLowPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MASTER_LOWPRI_INTR:
        dfeFl_MiscClearForceMasterLowPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS:    
        dfeFl_MiscClearMasterLowPriIntrStatus(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTRGRP:
        dfeFl_MiscEnableMasterLowPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterLowPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MASTER_LOWPRI_INTRGRP:
        dfeFl_MiscDisableMasterLowPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterLowPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MASTER_LOWPRI_INTRGRP:
        dfeFl_MiscSetForceMasterLowPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterLowPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MASTER_LOWPRI_INTRGRP:
        dfeFl_MiscClearForceMasterLowPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterLowPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTRGRP_STATUS:
        dfeFl_MiscClearMasterLowPriIntrGroupStatus(hDfeMisc, (DfeFl_MiscMasterLowPriIntrGroup *)arg);
        break;            
    case DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTR:
        dfeFl_MiscEnableMasterHiPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTR:
        dfeFl_MiscDisableMasterHiPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MASTER_HIPRI_INTR:
        dfeFl_MiscSetForceMasterHiPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MASTER_HIPRI_INTR:
        dfeFl_MiscClearForceMasterHiPriIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTR_STATUS:
        dfeFl_MiscClearMasterHiPriIntrStatus(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_MASTER_HIPRI_INTRGRP:
        dfeFl_MiscEnableMasterHiPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterHiPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MASTER_HIPRI_INTRGRP:
        dfeFl_MiscDisableMasterHiPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterHiPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MASTER_HIPRI_INTRGRP:
        dfeFl_MiscSetForceMasterHiPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterHiPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MASTER_HIPRI_INTRGRP:
        dfeFl_MiscClearForceMasterHiPriIntrGroup(hDfeMisc, (DfeFl_MiscMasterHiPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MASTER_HIPRI_INTRGRP_STATUS:  
        dfeFl_MiscClearMasterHiPriIntrGroupStatus(hDfeMisc, (DfeFl_MiscMasterHiPriIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_SYNC_INTR:
        dfeFl_MiscEnableSyncIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_SYNC_INTR:
        dfeFl_MiscDisableSyncIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_SYNC_INTR:    
        dfeFl_MiscSetForceSyncIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_SYNC_INTR:   
        dfeFl_MiscClearForceSyncIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS:
        dfeFl_MiscClearSyncIntrStatus(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_SYNC_INTRGRP:
        dfeFl_MiscEnableSyncIntrGroup(hDfeMisc, (DfeFl_MiscSyncIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_SYNC_INTRGRP:
        dfeFl_MiscDisableSyncIntrGroup(hDfeMisc, (DfeFl_MiscSyncIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_SYNC_INTRGRP:    
        dfeFl_MiscSetForceSyncIntrGroup(hDfeMisc, (DfeFl_MiscSyncIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_SYNC_INTRGRP:    
        dfeFl_MiscClearForceSyncIntrGroup(hDfeMisc, (DfeFl_MiscSyncIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_SYNC_INTRGRP_STATUS:
        dfeFl_MiscClearSyncIntrGroupStatus(hDfeMisc, (DfeFl_MiscSyncIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTR:
        dfeFl_MiscEnableCppDmaDoneIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTR:
        dfeFl_MiscDisableCppDmaDoneIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_CPP_DMA_DONE_INTR:    
        dfeFl_MiscSetForceCppDmaDoneIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_CPP_DMA_DONE_INTR:   
        dfeFl_MiscClearForceCppDmaDoneIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTR_STATUS:
        dfeFl_MiscClearCppDmaDoneIntrStatus(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_CPP_DMA_DONE_INTRGRP:
        dfeFl_MiscEnableCppDmaDoneIntrGroup(hDfeMisc, (DfeFl_MiscCppIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_CPP_DMA_DONE_INTRGRP:
        dfeFl_MiscDisableCppDmaDoneIntrGroup(hDfeMisc, (DfeFl_MiscCppIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_CPP_DMA_DONE_INTRGRP:
        dfeFl_MiscSetForceCppDmaDoneIntrGroup(hDfeMisc, (DfeFl_MiscCppIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_CPP_DMA_DONE_INTRGRP:    
        dfeFl_MiscClearForceCppDmaDoneIntrGroup(hDfeMisc, (DfeFl_MiscCppIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_CPP_DMA_DONE_INTRGRP_STATUS:    
        dfeFl_MiscClearCppDmaDoneIntrGroupStatus(hDfeMisc, (DfeFl_MiscCppIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_MISC_INTR:
        dfeFl_MiscEnableMiscIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MISC_INTR:
        dfeFl_MiscDisableMiscIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MISC_INTR:     
        dfeFl_MiscSetForceMiscIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MISC_INTR:    
        dfeFl_MiscClearForceMiscIntr(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MISC_INTR_STATUS:
        dfeFl_MiscClearMiscIntrStatus(hDfeMisc, *(uint32_t *)arg);
        break;
    case DFE_FL_MISC_CMD_ENB_MISC_INTRGRP:
        dfeFl_MiscEnableMiscIntrGroup(hDfeMisc, (DfeFl_MiscMiscIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_DIS_MISC_INTRGRP:
        dfeFl_MiscDisableMiscIntrGroup(hDfeMisc, (DfeFl_MiscMiscIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_SET_FORCE_MISC_INTRGRP:
        dfeFl_MiscSetForceMiscIntrGroup(hDfeMisc, (DfeFl_MiscMiscIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_FORCE_MISC_INTRGRP:     
        dfeFl_MiscClearForceMiscIntrGroup(hDfeMisc, (DfeFl_MiscMiscIntrGroup *)arg);
        break;
    case DFE_FL_MISC_CMD_CLR_MISC_INTRGRP_STATUS:
        dfeFl_MiscClearMiscIntrGroupStatus(hDfeMisc, (DfeFl_MiscMiscIntrGroup *)arg);
        break;
    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
