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

/** @file dfe_fl_jesdHwControl.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief File for functional layer of CSL API dfeFl_JesdHwControl()
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
 *   @n@b dfeFl_JesdHwControl
 *
 *   @b Description
 *   @n Perform a control-operation. This API is used to invoke any of the
 *      supported control-operations supported by the module.
 *
 *   @b Arguments
 *   @verbatim
         hDfeJesd       Valid handle
         ctrlCmd        The command to this API
         arg            The pointer to the argument
     @endverbatim
 *
 *   <b> Return Value </b>  DfeFl_Status
 *   @li                    DFE_FL_SOK             - HwControl successful.
 *   @li                    DFE_FL_INVCMD     - Invalid command
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
 *   @n  The hardware registers of Dfe Jesd.
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
         DfeFl_JesdInitsConfig jesdInits;

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

         // reset JESD
         jesdInits.cmn.ssel = DFE_FL_SYNC_GEN_SIG_ALWAYS;
         jesdInits.cmn.initClkGate = 1;
         jesdInits.cmn.initState = 1;
         jesdInits.cmn.clearData = 1;
         jesdInits.clearDataLane[0] = 1;
         jesdInits.clearDataLane[1] = 1;
         jesdInits.clearDataLane[2] = 1;
         jesdInits.clearDataLane[3] = 1;

         // reset JESDTX
         dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CFG_INITS, &jesdInits);
         // reset JESDRX
         dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDRX_CMD_CFG_INITS, &jesdInits);
         return PASS;

     @endverbatim
 * ===========================================================================
 */
DfeFl_Status  dfeFl_JesdHwControl
(
    DfeFl_JesdHandle         hDfeJesd,
    DfeFl_JesdHwControlCmd   ctrlCmd,
    void                        *arg
)
{
//    uint32_t i, data;
    DfeFl_Status err = DFE_FL_SOK;
    
    switch(ctrlCmd)
    {   
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      TX CMDs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
    case DFE_FL_JESDTX_CMD_CFG_INITS:
    {
        DfeFl_JesdInitsConfig *cfg = (DfeFl_JesdInitsConfig *)arg;
        
        dfeFl_JesdTxConfigInits(hDfeJesd, cfg);
        break;
    }    
    case DFE_FL_JESDTX_CMD_CFG_INITSSEL:
    {
        dfeFl_JesdTxConfigInitSsel(hDfeJesd, *(uint32_t *)arg);
        break;
    }    
    case DFE_FL_JESDTX_CMD_CFG_INITSTATE:
    {
        dfeFl_JesdTxConfigInitState(hDfeJesd, *(uint32_t *)arg);
        break;
    }	
    case DFE_FL_JESDTX_CMD_CFG_CLRLANE:
    {
        DfeFl_JesdClearDataLaneConfig *cfg = (DfeFl_JesdClearDataLaneConfig *)arg;
        
        dfeFl_JesdTxSetClearDataLane(hDfeJesd, cfg->lane, cfg->clearData);
        break;
    }        
    case DFE_FL_JESDTX_CMD_CFG_TX_INPUTS:
    {
        dfeFl_JesdTxConfigTxInputs(hDfeJesd, (DfeFl_JesdTxInputsConfig *)arg);
        break;
    }        
    case DFE_FL_JESDTX_CMD_CFG_TEST_BUS:
    {
        dfeFl_JesdTxSetTestBusSel(hDfeJesd, *(uint32_t *)arg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_TEST_SEQ:
    {
        DfeFl_JesdTxTestSeqConfig *cfg = (DfeFl_JesdTxTestSeqConfig *)arg;
        
        dfeFl_JesdTxSetTestSeqSel(hDfeJesd, cfg->lane, cfg->testSeq);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_SYNCN_LOOPBACK:
    {
        DfeFl_JesdTxSyncnLoopbackConfig *cfg = (DfeFl_JesdTxSyncnLoopbackConfig *)arg;
        
        dfeFl_JesdTxSetSyncnLoopback(hDfeJesd, cfg->link, cfg->syncnLoopback);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_SYNCN_INVERT:
    {
        DfeFl_JesdTxSyncnInvertConfig *cfg = (DfeFl_JesdTxSyncnInvertConfig *)arg;
        
        dfeFl_JesdTxSetSyncnPolarityInvert(hDfeJesd, cfg->link, cfg->syncnInvert);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_BB_CTRL:
    {
        DfeFl_JesdTxBbCtrlConfig *cfg = (DfeFl_JesdTxBbCtrlConfig *)arg;
        
        dfeFl_JesdTxSetBbCtrl(hDfeJesd, cfg->laneEnable, cfg->linkSel);
        break;
    }
    case DFE_FL_JESDTX_CMD_CLR_BB_ERR:
        dfeFl_JesdTxClearBbErr(hDfeJesd);
        break;
    case DFE_FL_JESDTX_CMD_SET_FIFO_READ_DLY:
        dfeFl_JesdTxSetFifoReadDelay(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDTX_CMD_SET_FIFO_ERROR_ZERO_DATA:
    	dfeFl_JesdTxSetFifoErrorZeroData(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDTX_CMD_SET_SYSREF_DLY:
        dfeFl_JesdTxSetSysrefDelay(hDfeJesd, *(uint32_t *)arg);
        break;        
    case DFE_FL_JESDTX_CMD_FORCE_SYSREF_REQUEST:
    {
        DfeFl_JesdForceSysrefReqConfig *cfg = (DfeFl_JesdForceSysrefReqConfig *)arg;
        
        dfeFl_JesdTxForceSysRefRequest(hDfeJesd, cfg->force, cfg->autoOffTimer);
        break;
    }
    case DFE_FL_JESDTX_CMD_SET_TESTGEN_SSEL:
    {
        DfeFl_JesdTxTestGenSselConfig *cfg = (DfeFl_JesdTxTestGenSselConfig *)arg;
        
        dfeFl_JesdTxSetSignalGenSsel(hDfeJesd, cfg->tgDev, cfg->ssel);
        break;
    }
    case DFE_FL_JESDTX_CMD_SET_CHKSUM_SSEL:
    {
        DfeFl_JesdChksumSselConfig *cfg = (DfeFl_JesdChksumSselConfig *)arg;
        
        dfeFl_JesdTxSetChksumSsel(hDfeJesd, cfg->chksumLane, cfg->ssel);
        break;
    }        
    case DFE_FL_JESDTX_CMD_SET_INITSTATELINK_SSEL:
    {
        DfeFl_JesdInitStateLinkSselConfig *cfg = (DfeFl_JesdInitStateLinkSselConfig *)arg;
        
        dfeFl_JesdTxSetChksumSsel(hDfeJesd, cfg->link, cfg->ssel);
        break;
    }        
    case DFE_FL_JESDTX_CMD_SET_SYSREF_CNTR_SSEL:
        dfeFl_JesdTxSetSysRefAlignmentCounterSsel(hDfeJesd, *(uint32_t *)arg);
        break;        
    case DFE_FL_JESDTX_CMD_SET_SYSREF_MODE_SSEL:
        dfeFl_JesdTxSetSysRefModeSsel(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDTX_CMD_SET_SYSREF_MODE:
    {
        DfeFl_JesdLinkSysrefModeConfig *cfg = (DfeFl_JesdLinkSysrefModeConfig *)arg;
        
        dfeFl_JesdTxSetSysrefMode(hDfeJesd, cfg->link, cfg->mode);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_TESTGEN:
    {
        DfeFl_JesdTxTestGenConfig *cfg = (DfeFl_JesdTxTestGenConfig *)arg;
        
        dfeFl_JesdTxConfigTestGen(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_CHKSUM:
    {
        DfeFl_JesdTxChksumConfig *cfg = (DfeFl_JesdTxChksumConfig *)arg;
        
        dfeFl_JesdTxConfigChksum(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_CLKGATE:
    {
        DfeFl_JesdClockGateConfig *cfg = (DfeFl_JesdClockGateConfig *)arg;
        
        dfeFl_JesdTxConfigClockGate(hDfeJesd, cfg->linkPath, &cfg->cgCfg);
        break;
        
    }
    case DFE_FL_JESDTX_CMD_CFG_LANE:
    {
        DfeFl_JesdLaneConfig *cfg = (DfeFl_JesdLaneConfig *)arg;
        
        dfeFl_JesdTxConfigLane(hDfeJesd, cfg->lane, cfg->laneEnable, cfg->linkAssign, cfg->laneId);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_LINK:
    {
        DfeFl_JesdTxLinkConfig *cfg = (DfeFl_JesdTxLinkConfig *)arg;
        
        dfeFl_JesdTxConfigLink(hDfeJesd, cfg);
        break;
    }        
    case DFE_FL_JESDTX_CMD_CLR_LINK_ERR_CNT:
        dfeFl_JesdTxClearLinkErrCnt(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDTX_CMD_ENB_LANE_INTRGRP:
    {
        DfeFl_JesdTxLaneIntrs *cfg = (DfeFl_JesdTxLaneIntrs *)arg;
        
        dfeFl_JesdTxEnableLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_DIS_LANE_INTRGRP:
    {
        DfeFl_JesdTxLaneIntrs *cfg = (DfeFl_JesdTxLaneIntrs *)arg;
        
        dfeFl_JesdTxDisableLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS:
    {
        DfeFl_JesdTxLaneIntrs *cfg = (DfeFl_JesdTxLaneIntrs *)arg;
        
        dfeFl_JesdTxClearLaneIntrsStatus(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_SET_FORCE_LANE_INTRGRP:
    {
        DfeFl_JesdTxLaneIntrs *cfg = (DfeFl_JesdTxLaneIntrs *)arg;
        
        dfeFl_JesdTxSetForceLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CLR_FORCE_LANE_INTRGRP:
    {
        DfeFl_JesdTxLaneIntrs *cfg = (DfeFl_JesdTxLaneIntrs *)arg;
        
        dfeFl_JesdTxClearForceLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_ENB_SYSREF_INTRGRP:
    {
        DfeFl_JesdTxSysrefIntrs *cfg = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxEnableSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_DIS_SYSREF_INTRGRP:
    {
        DfeFl_JesdTxSysrefIntrs *cfg = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxDisableSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CLR_SYSREF_INTRGRP_STATUS:
    {
        DfeFl_JesdTxSysrefIntrs *cfg = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxClearSysrefIntrsStatus(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_SET_FORCE_SYSREF_INTRGRP:
    {
        DfeFl_JesdTxSysrefIntrs *cfg = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxSetForceSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CLR_FORCE_SYSREF_INTRGRP:
    {
        DfeFl_JesdTxSysrefIntrs *cfg = (DfeFl_JesdTxSysrefIntrs *)arg;
        
        dfeFl_JesdTxClearForceSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_MAP_LANE:
    {
        DfeFl_JesdTxMapLaneConfig *cfg = (DfeFl_JesdTxMapLaneConfig *)arg;
        
        dfeFl_JesdTxConfigMapLane(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_MAP_NIBB:
    {
        DfeFl_JesdTxMapNibbConfig *cfg = (DfeFl_JesdTxMapNibbConfig *)arg;
        
        dfeFl_JesdTxConfigMapNibb(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_ENABLE:
    {
        DfeFl_JesdTxNibbTestEnableConfig *cfg = (DfeFl_JesdTxNibbTestEnableConfig *)arg;
        
        dfeFl_JesdTxConfigNibbTestEnable(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDTX_CMD_CFG_NIBB_TEST_DATA:
    {
        DfeFl_JesdTxNibbTestDataConfig *cfg = (DfeFl_JesdTxNibbTestDataConfig *)arg;
        
        dfeFl_JesdTxConfigNibbTestData(hDfeJesd, cfg);
        break;
    }
    
    
    ///////////////////////////////////////////////////////////////////////////////////////
    //
    //      RX CMDs
    //
    ///////////////////////////////////////////////////////////////////////////////////////
    
    case DFE_FL_JESDRX_CMD_CFG_INITS:
    {
        DfeFl_JesdInitsConfig *cfg = (DfeFl_JesdInitsConfig *)arg;
        
        dfeFl_JesdRxConfigInits(hDfeJesd, cfg);
        break;
    }        
    case DFE_FL_JESDRX_CMD_CFG_INITSSEL:
    {
        dfeFl_JesdRxConfigInitSsel(hDfeJesd, *(uint32_t *)arg);
        break;
    }    
    case DFE_FL_JESDRX_CMD_CFG_INITSTATE:
    {
        dfeFl_JesdRxConfigInitState(hDfeJesd, *(uint32_t *)arg);
        break;
    }		
    case DFE_FL_JESDRX_CMD_CFG_CLRLANE:
    {
        DfeFl_JesdClearDataLaneConfig *cfg = (DfeFl_JesdClearDataLaneConfig *)arg;
        
        dfeFl_JesdRxSetClearDataLane(hDfeJesd, cfg->lane, cfg->clearData);
        break;
    }
    case DFE_FL_JESDRX_CMD_SET_TESTBUS_SEL:
        dfeFl_JesdRxSetTestBusSel(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDRX_CMD_CFG_TESTSEQ:
    {
        DfeFl_JesdRxTestSeqConfig *cfg = (DfeFl_JesdRxTestSeqConfig *)arg;
        
        dfeFl_JesdRxSetTestSeqSel(hDfeJesd, cfg->lane, cfg->testSeqSel);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_LOOPBACK:        
    {
        DfeFl_JesdRxLoopbackConfig *cfg = (DfeFl_JesdRxLoopbackConfig *)arg;
        
        dfeFl_JesdRxConfigLoopback(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_BBRXCTRL:
    {
        DfeFl_JesdRxBbRxCtrlConfig *cfg = (DfeFl_JesdRxBbRxCtrlConfig *)arg;
        
        CSL_defJesdRxConfigBbRxCtrl(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_FIFO:
    {
        DfeFl_JesdRxFifoConfig *cfg = (DfeFl_JesdRxFifoConfig *)arg;
        
        CSL_defJesdRxConfigFifo(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_SYNCN_OUT:
    {
        DfeFl_JesdRxSyncnOutConfig *cfg = (DfeFl_JesdRxSyncnOutConfig *)arg;
        
        dfeFl_JesdRxConfigSyncnOut(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_SET_SYSREF_DLY:
        dfeFl_JesdRxSetSysrefDelay(hDfeJesd, *(uint32_t *)arg);
        break;        
    case DFE_FL_JESDRX_CMD_FORCE_SYSREF_REQUEST:
    {
    	DfeFl_JesdForceSysrefReqConfig *cfg = (DfeFl_JesdForceSysrefReqConfig *)arg;
        
        dfeFl_JesdRxForceSysRefRequest(hDfeJesd, cfg->force, cfg->autoOffTimer);
        break;
    }
    case DFE_FL_JESDRX_CMD_SET_CHKSUM_SSEL:
    {
        DfeFl_JesdChksumSselConfig *cfg = (DfeFl_JesdChksumSselConfig *)arg;
        
        dfeFl_JesdRxSetChksumSsel(hDfeJesd, cfg->chksumLane, cfg->ssel);
        break;
    }        
    case DFE_FL_JESDRX_CMD_SET_INITSTATELINK_SSEL:
    {
        DfeFl_JesdInitStateLinkSselConfig *cfg = (DfeFl_JesdInitStateLinkSselConfig *)arg;
        
        dfeFl_JesdRxSetChksumSsel(hDfeJesd, cfg->link, cfg->ssel);
        break;
    }        
    case DFE_FL_JESDRX_CMD_SET_SYSREF_CNTR_SSEL:
        dfeFl_JesdRxSetSysRefAlignmentCounterSsel(hDfeJesd, *(uint32_t *)arg);
        break;        
    case DFE_FL_JESDRX_CMD_SET_SYNCN_OUT_BUS_SSEL:
    {
        DfeFl_JesdRxSycnOutBusSselConfig *cfg = (DfeFl_JesdRxSycnOutBusSselConfig *)arg;
        
        dfeFl_JesdRxSetSyncnOutSyncBusSsel(hDfeJesd, cfg->bus, cfg->ssel);
        break;
    }        
    case DFE_FL_JESDRX_CMD_SET_SYSREF_MODE_SSEL:
        dfeFl_JesdRxSetSysRefModeSsel(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDRX_CMD_SET_SYSREF_MODE:
    {
        DfeFl_JesdLinkSysrefModeConfig *cfg = (DfeFl_JesdLinkSysrefModeConfig *)arg;
        
        dfeFl_JesdRxSetSysrefMode(hDfeJesd, cfg->link, cfg->mode);
        break;
    }

    case DFE_FL_JESDRX_CMD_CFG_CHKSUM:
    {
        DfeFl_JesdRxChksumConfig *cfg = (DfeFl_JesdRxChksumConfig *)arg;
        
        dfeFl_JesdRxConfigChksum(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_LINK_CLKGATE:
    {
        DfeFl_JesdClockGateConfig *cfg = (DfeFl_JesdClockGateConfig *)arg;
        
        dfeFl_JesdRxConfigLinkClockGate(hDfeJesd, cfg->linkPath, &cfg->cgCfg);
        break;
        
    }
    case DFE_FL_JESDRX_CMD_CFG_PATH_CLKGATE:
    {
        DfeFl_JesdClockGateConfig *cfg = (DfeFl_JesdClockGateConfig *)arg;
        
        dfeFl_JesdRxConfigPathClockGate(hDfeJesd, cfg->linkPath, &cfg->cgCfg);
        break;
        
    }
    case DFE_FL_JESDRX_CMD_CFG_LANE:
    {
        DfeFl_JesdLaneConfig *cfg = (DfeFl_JesdLaneConfig *)arg;
        
        dfeFl_JesdRxConfigLane(hDfeJesd, cfg->lane, cfg->laneEnable, cfg->linkAssign, cfg->laneId);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_LINK:
    {
        DfeFl_JesdRxLinkConfig *cfg = (DfeFl_JesdRxLinkConfig *)arg;
        
        dfeFl_JesdRxConfigLink(hDfeJesd, cfg);
        break;
    }        
    case DFE_FL_JESDRX_CMD_CLR_LINK_ERR_CNT:
        dfeFl_JesdRxClearLinkErrCnt(hDfeJesd, *(uint32_t *)arg);
        break;
    case DFE_FL_JESDRX_CMD_ENB_LANE_INTRGRP:
    {
        DfeFl_JesdRxLaneIntrs *cfg = (DfeFl_JesdRxLaneIntrs *)arg;
        
        dfeFl_JesdRxEnableLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_DIS_LANE_INTRGRP:
    {
        DfeFl_JesdRxLaneIntrs *cfg = (DfeFl_JesdRxLaneIntrs *)arg;
        
        dfeFl_JesdRxDisableLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CLR_LANE_INTRGRP_STATUS:
    {
        DfeFl_JesdRxLaneIntrs *cfg = (DfeFl_JesdRxLaneIntrs *)arg;
        
        dfeFl_JesdRxClearLaneIntrsStatus(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_SET_FORCE_LANE_INTRGRP:
    {
        DfeFl_JesdRxLaneIntrs *cfg = (DfeFl_JesdRxLaneIntrs *)arg;
        
        dfeFl_JesdRxSetForceLaneIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CLR_FORCE_LANE_INTRGRP:
    {
        DfeFl_JesdRxLaneIntrs *cfg = (DfeFl_JesdRxLaneIntrs *)arg;
        
        dfeFl_JesdRxClearForceLaneIntrs(hDfeJesd, cfg);
        break;
    }

    case DFE_FL_JESDRX_CMD_ENB_SYSREF_INTRGRP:
    {
        DfeFl_JesdRxSysrefIntrs *cfg = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxEnableSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_DIS_SYSREF_INTRGRP:
    {
        DfeFl_JesdRxSysrefIntrs *cfg = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxDisableSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CLR_SYSREF_INTRGRP_STATUS:
    {
        DfeFl_JesdRxSysrefIntrs *cfg = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxClearSysrefIntrsStatus(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_SET_FORCE_SYSREF_INTRGRP:
    {
        DfeFl_JesdRxSysrefIntrs *cfg = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxSetForceSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CLR_FORCE_SYSREF_INTRGRP:
    {
        DfeFl_JesdRxSysrefIntrs *cfg = (DfeFl_JesdRxSysrefIntrs *)arg;
        
        dfeFl_JesdRxClearForceSysrefIntrs(hDfeJesd, cfg);
        break;
    }
    case DFE_FL_JESDRX_CMD_CFG_MAP_NIBB:
    {
        DfeFl_JesdRxMapNibbConfig *cfg = (DfeFl_JesdRxMapNibbConfig *)arg;
        
        dfeFl_JesdRxConfigMapNibb(hDfeJesd, cfg);
        break;
    }


    default:
        err = DFE_FL_INVCMD;
        break;        
    }
    
    return err;
}
