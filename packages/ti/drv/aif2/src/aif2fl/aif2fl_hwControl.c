/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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

/** ===========================================================================
 *   @file  aif2fl_hwControl.c
 *
 *
 *   @brief  Antenna Interface 2 HW control 3.x function
 *
 */

/* =============================================================================
 * Revision History
 * ===============
 *  03-Jun-2009  Albert  File Created.
 *  06-June-2015 Seb     File imported in the driver
 *  
 *
 * =============================================================================
 */

#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>

/** ===========================================================================
 *   @n@b Aif2Fl_hwControl
 *
 *   @b Description
 *   @n This function performs various control operations on aif2 link,
 *      based on the command passed.
 *
 *   @b Arguments
 *   @verbatim
            haif2        Handle to the aif2 instance and link number for an argument
 
            cmd         Operation to be performed on the aif2
 
            arg         Argument specific to the command 
 
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *   @li                    AIF2FL_SOK            - Command execution successful.
 *   @li                    AIF2FL_BADHANDLE - Invalid handle
 *   @li                    AIF2FL_INVCMD    - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  Registers of aif2 instance are configured according to the command
 *       and the command arguments. The command determines which registers are
 *       modified.
 *
 *   @b Writes
 *   @n Registers determined by the command
 *
 *   @b Example
 *   @verbatim
        // handle for AIF2
        Aif2Fl_Handle hAif2;
        // other related declarations
        ...
        // ctrl argument for hw command
        uint16_t ctrlArg;

        // Open handle  - for use 
        hAif2 = Aif2Fl_open(&Aif2Obj, 0, &aif2Param, &status);

        if ((hAif2 == NULL) || (status != AIF2FL_SOK)) 
        {
           printf ("\nError opening AIF2FL");
           exit(1);
        }

        // Do config 
        Config.globalSetup = &gblCfg;
        ...
        //Do setup 
        Aif2Fl_hwSetup(handleAif2, &Config);

        ctrlArg = AIF2FL_CTRL_RX_LINK_ENABLE;
        hAif2->arg_link = AIF2FL_LINK_0; //link 0 enable

        // Send hw control command to enable Tx/Rx of link 0 
        Aif2Fl_hwControl(hAif2, AIF2FL_CMD_ENABLE_DISABLE_RX_LINK, (void *)&ctrlArg);
     @endverbatim
 * ============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Aif2Fl_hwControl, ".text:aif2fl");
#endif
Aif2Fl_Status  Aif2Fl_hwControl(
    Aif2Fl_Handle          hAif2,
    Aif2Fl_HwControlCmd    cmd,
    void                   *arg
)
{
    Aif2Fl_Status status = AIF2FL_SOK ;
    
    if (hAif2 == NULL)
        return AIF2FL_BADHANDLE;
    
    switch (cmd) {
        
    /** Starts Rx link (use hAif2->arg_link to select link)  */
    case AIF2FL_CMD_ENABLE_DISABLE_RX_LINK:
        Aif2Fl_enDisRxLink (hAif2, *(uint16_t *)arg);
        break;
    /** Starts Tx link (use hAif2->arg_link to select link)  */    
    case AIF2FL_CMD_ENABLE_DISABLE_TX_LINK:
        Aif2Fl_enDisTxLink(hAif2, *(uint16_t *)arg);
        break;
#ifndef K2
	/** Enable loopback mode for specific link (use hAif2->arg_link to select link)*/
   case AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK:
	   Aif2Fl_enDisLinkLoopback (hAif2, *(uint16_t *)arg);
	   break;
#endif
     /** Control Aif2 Emulation (argument type:  Aif2Fl_VcEmu*)  */   
    case AIF2FL_CMD_VC_EMU_CONTROL:
        Aif2Fl_vcEmuControl(hAif2, *(Aif2Fl_VcEmu *)arg);
        break;
#ifndef K2
	/** Select Serdes link Tx test pattern   (argument type:  Aif2Fl_SdTestPattern *, use hAif2->arg_link to select link)  */
	case AIF2FL_CMD_SD_LINK_TX_TEST_PATTERN:
		Aif2Fl_sdLinkTxTestPattern (hAif2, *(Aif2Fl_SdTestPattern *)arg);
		break;
	/** Select Serdes link Rx test pattern   (argument type:  Aif2Fl_SdTestPattern *, use hAif2->arg_link to select link)  */
	case AIF2FL_CMD_SD_LINK_RX_TEST_PATTERN:
		Aif2Fl_sdLinkRxTestPattern (hAif2, *(Aif2Fl_SdTestPattern *)arg);
		break;
#endif
    /** Enable SD B8 PLL */   
    case AIF2FL_CMD_ENABLE_DISABLE_SD_B8_PLL:
        Aif2Fl_enDisSdB8Pll(hAif2, *(uint16_t *)arg);
        break;
    /** Enable SD B4 PLL */   
    case AIF2FL_CMD_ENABLE_DISABLE_SD_B4_PLL:
        Aif2Fl_enDisSdB4Pll(hAif2, *(uint16_t *)arg);
        break;
    /** Force RM Sync State (argument type:  Aif2Fl_RmForceSyncState *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_RM_FORCE_STATE:
        Aif2Fl_rmLinkForceRxState  (hAif2, *(Aif2Fl_RmForceSyncState *)arg);
        break;
    /** TM L1 Inband Control Signal Set (argument type:  uint8_t *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_TM_L1_INBAND_SET:
        Aif2Fl_tmLinkL1InbandSet  (hAif2, *(uint8_t *)arg);
        break;
    /** Force TM Flush FIFO (argument type:  uint16_t *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_TM_FLUSH_FIFO:
        Aif2Fl_forceTmFlush (hAif2, *(uint16_t *)arg);
        break;
    /** Force TM Idle state (argument type:  uint16_t *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_TM_IDLE:
        Aif2Fl_forceTmIdle (hAif2, *(uint16_t *)arg);
        break;
     /** Force TM Resync state (argument type:  uint16_t *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_TM_RESYNC:
        Aif2Fl_forceTmReSync (hAif2, *(uint16_t *)arg);
        break;
    /** Dynamic configuration of PD CPRI id lut register (argument type:  Aif2Fl_PdCpriIdLut  *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_PD_CPRI_ID_LUT_SETUP:
        Aif2Fl_pdCpriIdLutDynamicSetup (hAif2, *(Aif2Fl_PdCpriIdLut  *)arg);
        break;
    /** Dynamic configuration of PD CPRI Control Word lut register (argument type:  Aif2Fl_PdCpriCwLut  *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_PD_CPRI_CW_LUT_SETUP:
        Aif2Fl_pdCpriCwLutDynamicSetup (hAif2, *(Aif2Fl_CpriCwLut  *)arg);
        break;
    /** Dynamic configuration of PD DBMR  (argument type:  Aif2Fl_DualbitMap  *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_PD_LINK_DBMR_SETUP:
        Aif2Fl_pdLinkDbmDynamicSetup (hAif2, *(Aif2Fl_DualBitMap   *)arg);
        break;
    /** Dynamic configuration of PD channel config registers (argument type:  Aif2Fl_PdChannelConfig  *)  */   
    case AIF2FL_CMD_PD_CH_CONFIG_SETUP:
        Aif2Fl_pdChConfigDynamicSetup (hAif2, *(Aif2Fl_PdChannelConfig  *)arg);
        break;
     /** Dynamic configuration of PE CPRI Control Word lut register (argument type:  Aif2Fl_CpriCwLut  *, use hAif2->arg_link to select link)  */   
    case AIF2FL_CMD_PE_CPRI_CW_LUT_SETUP:
        Aif2Fl_peCpriCwLutDynamicSetup (hAif2, *(Aif2Fl_CpriCwLut  *)arg);
        break;
    /** Dynamic configuration of PE obsai header register (argument type:  Aif2Fl_PeObsaiHeader  *)  */   
    case AIF2FL_CMD_PE_OBSAI_HEADER_SETUP:
        Aif2Fl_peObsaiHeaderSetup (hAif2, *(Aif2Fl_PeObsaiHeader  *)arg);
        break;
    /** Dynamic configuration of PE DBMR  (argument type:  Aif2Fl_PeDbmr  *)  */   
    case AIF2FL_CMD_PE_LINK_DBMR_SETUP:
        Aif2Fl_peDbmrDynamicSetup (hAif2, *(Aif2Fl_PeDbmr   *)arg);
        break;
    /** Dynamic configuration of PE Modulo rule  (argument type:  Aif2Fl_DualbitMap  *)  */   
    case AIF2FL_CMD_PE_MODULO_RULE_SETUP:
        Aif2Fl_peModuloTxRuleSetup (hAif2, *(Aif2Fl_PeModuloRule  *)arg);
        break;
    /** Dynamic configuration of PE channel config registers (argument type:  Aif2Fl_PeChannelConfig  *)  */   
    case AIF2FL_CMD_PE_CH_CONFIG_SETUP:
        Aif2Fl_peChConfigDynamicSetup (hAif2, *(Aif2Fl_PeChannelConfig  *)arg);
        break;
    /** Dynamic configuration of PE channel rule LUT config registers (argument type:  Aif2Fl_PeChRuleLut  *)  */   
    case AIF2FL_CMD_PE_CH_RULE_LUT_SETUP:
        Aif2Fl_peChRuleLutDynamicSetup (hAif2, *(Aif2Fl_PeChRuleLut  *)arg);
        break;
    /** Enables Trace data and framing data capture (use hAif2->arg_link to select link, argument type: uint16_t *) */ 
    case AIF2FL_CMD_ENABLE_DISABLE_LINK_DATA_CAPTURE:
        Aif2Fl_enRxLinkDataCapture(hAif2, *(uint16_t*)arg);
        break;
    /** Data Trace Sync Enable.  (argument type: uint16_t *) */
    case AIF2FL_CMD_ENABLE_DISABLE_DATA_TRACE_SYNC:
        Aif2Fl_enRxTraceDataSync(hAif2, *(uint16_t*)arg);
        break;
    /** Enables Ingress DB Debug mode (argument type: uint16_t *) */
    case AIF2FL_CMD_DB_IN_ENABLE_DISABLE_DEBUG_MODE:
        Aif2Fl_inDbEnDisDebug(hAif2, *(uint16_t*)arg);
        break;
    /** Debug data written to bits 128:0 of Ingress DB RAM (argument type: uint32_t *) */
    case AIF2FL_CMD_DB_IN_DEBUG_DATA_SETUP:
        Aif2Fl_inDbDebugDataSetup(hAif2, (uint32_t*)arg);
        break;
   /** Ingress DB debug side band data setup (argument type: Aif2Fl_DbSideData *)  */
    case AIF2FL_CMD_DB_IN_DEBUG_SIDE_DATA_SETUP:
        Aif2Fl_inDbDebugSideDataSetup(hAif2, *(Aif2Fl_DbSideData *)arg);
        break;
   /** Writes the data in the following registers into the Ingress DB and sideband RAMS
         DB_IDB_DEBUG_D0, DB_IDB_DEBUG_D1, DB_IDB_DEBUG_D2, DB_IDB_DEBUG_D3, DB_IDB_DEBUG_SBDN (argument type: uint16_t *)  */
    case AIF2FL_CMD_DB_IN_DEBUG_WRITE:
        Aif2Fl_inDbDebugWrite(hAif2, *(uint16_t *)arg);
        break;
   /** Set Read and Write Address used to access write or read Offset RAM for DB Debug 
   (argument type: uint8_t * arg[0] : write offset addr  arg[1] : read offset addr)  */
    case AIF2FL_CMD_DB_IN_DEBUG_OFFSET_ADDR:
        Aif2Fl_inDbDebugOffsetAddr(hAif2, (uint8_t *)arg);
        break;
   /** Enable or Disable Ingress DB channel to add or remove channel dynamically (argument type: uint32_t *)  */
    case AIF2FL_CMD_DB_IN_ENABLE_DISABLE_CHANNEL:
        Aif2Fl_inDbChEnable(hAif2, (uint32_t *)arg);
        break;
   /** Setup Ingress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)  */
    case AIF2FL_CMD_DB_IN_CHANNEL_SETUP:
        Aif2Fl_inDbChannelSetup(hAif2, *(Aif2Fl_DbChannel *)arg);
        break;
    /** Enables Egress DB Debug mode (argument type: uint16_t *) */
    case AIF2FL_CMD_DB_E_ENABLE_DISABLE_DEBUG_MODE:
        Aif2Fl_eDbEnDisDebug(hAif2, *(uint16_t*)arg);
        break;
   /** Setup Side band data control info like dio and fifo write enable and channel id and dio base address.(argument type: Aif2Fl_DbSideData *) */
    case AIF2FL_CMD_DB_E_DEBUG_READ_CONTROL:
        Aif2Fl_eDbDebugReadControl(hAif2, *(Aif2Fl_DbSideData*)arg);
        break;
   /** the value loaded into DB_EDB_DEBUG_RD_CNTL.CH_ID being issued to the AxC Token FIFO.(argument type: uint16_t *) */
    case AIF2FL_CMD_DB_E_DEBUG_WRITE_TOKEN:
        Aif2Fl_eDbDebugWrToken(hAif2, *(uint16_t *)arg);
        break;
   /** Reads the data in the following registers from the Egress DB and sideband RAMS
         DB_EDB_DEBUG_D0, DB_EDB_DEBUG_D1, DB_EDB_DEBUG_D2, DB_EDB_DEBUG_D3, DB_EDB_DEBUG_SBDN (argument type: uint16_t *)  */
    case AIF2FL_CMD_DB_E_DEBUG_READ:
        Aif2Fl_eDbDebugRead(hAif2, *(uint16_t *)arg);
        break;
   /** Set Read and Write Address used to access write or read Offset RAM for DB Debug 
   (argument type: uint8_t * arg[0] : write offset addr  arg[1] : read offset addr)  */
    case AIF2FL_CMD_DB_E_DEBUG_OFFSET_ADDR:
        Aif2Fl_eDbDebugOffsetAddr(hAif2, (uint8_t *)arg);
        break;
   /** Enable or Disable Egress DB channel to add or remove channel dynamically (argument type: uint32_t *)  */
    case AIF2FL_CMD_DB_E_ENABLE_DISABLE_CHANNEL:
        Aif2Fl_eDbChEnable(hAif2, (uint32_t *)arg);
        break;
   /** Setup Egress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)  */
    case AIF2FL_CMD_DB_E_CHANNEL_SETUP:
        Aif2Fl_eDbChannelSetup(hAif2, *(Aif2Fl_DbChannel *)arg);
        break;
   /** Enable or Disable Global AD module dynamically (argument type: uint16_t *)  */
    case AIF2FL_CMD_AD_IN_ENABLE_DISABLE_GLOBAL:
        Aif2Fl_adInGlobalEnableDisable(hAif2, *(uint16_t *)arg);
        break;
    /** Enable or Disable Global AD module dynamically (argument type: uint16_t *)  */
    case AIF2FL_CMD_AD_E_ENABLE_DISABLE_GLOBAL:
        Aif2Fl_adEGlobalEnableDisable(hAif2, *(uint16_t *)arg);
        break;
    /** Enable or Disable Global Ingress DIO mode dynamically (argument type: uint16_t *)  */
    case AIF2FL_CMD_AD_IN_ENABLE_DISABLE_DIO_GLOBAL:
        Aif2Fl_adInDioGlobalEnableDisable(hAif2, *(uint16_t *)arg);
        break;
    /** Enable or Disable Global Egress DIO mode dynamically (argument type: uint16_t *)  */
    case AIF2FL_CMD_AD_E_ENABLE_DISABLE_DIO_GLOBAL:
        Aif2Fl_adEDioGlobalEnableDisable(hAif2, *(uint16_t *)arg);
        break;
   /** Change Ingress DIO table selection dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_IN_DIO_TABLE_SELECT:
        Aif2Fl_adInDioTableSelect(hAif2, *(uint8_t *)arg);
        break;
   /** Change Ingress DIO num of AxC dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_IN_DIO_NUM_AXC_CHANGE:
        Aif2Fl_adInDioNumAxC(hAif2, *(uint8_t *)arg);
        break;
   /** Change Ingress DIO BCN table dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_IN_DIO_BCN_TABLE_CHANGE:
        Aif2Fl_adInDioBcnTableSetup(hAif2, (uint8_t *)arg);
        break;
   /** Change Egress DIO table selection dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_E_DIO_TABLE_SELECT:
        Aif2Fl_adEDioTableSelect(hAif2, *(uint8_t *)arg);
        break;
   /** Change Egress DIO num of AxC dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_E_DIO_NUM_AXC_CHANGE:
        Aif2Fl_adEDioNumAxC(hAif2, *(uint8_t *)arg);
        break;
   /** Change Egress DIO BCN table dynamically (use hAif2->arg_dioEngine to select dio engine.  argument type: uint8_t *)  */
    case AIF2FL_CMD_AD_E_DIO_BCN_TABLE_CHANGE:
        Aif2Fl_adEDioBcnTableSetup(hAif2, (uint8_t *)arg);
        break;
   /** Set Enable or disable Data trace DMA channel  (argument type: uint16_t *)  */
    case AIF2FL_CMD_AD_TRACE_DATA_DMA_CHANNEL_ON_OFF:
        Aif2Fl_adEnDisDtDmaCh(hAif2, *(uint16_t *)arg);
   /** Set Trace data base address  (argument type: uint32_t *)  */
    case AIF2FL_CMD_AD_TRACE_DATA_BASE_ADDR:
	 Aif2Fl_adDataTraceBaseAddr(hAif2, *(uint32_t *)arg);
        break;
   /** Set Trace side data base address  (argument type: uint32_t *)  */
    case AIF2FL_CMD_AD_TRACE_FRAMING_DATA_BASE_ADDR:
	 Aif2Fl_adFramingDataBaseAddr(hAif2, *(uint32_t *)arg);
        break;
    /** Sets the number of burst transfers before the destination address wraps back to the base address(argument type: uint32_t *)  */
    case AIF2FL_CMD_AD_TRACE_CPPI_DMA_BURST_WRAP:
        Aif2Fl_adDtDmaWrap(hAif2, *(uint32_t *)arg);
        break;
   /** Sets AT External Rad timer event dynamically (argument type: Aif2Fl_AtEvent *)  */
    case AIF2FL_CMD_AT_EVENT_SETUP:
        Aif2Fl_atEventSetup(hAif2, *(Aif2Fl_AtEvent  *)arg);
        break;
   /** Sets AT Delta offset  (use hAif2->arg_link to select link   argument type: Aif2Fl_AtEvent *)  */
    case AIF2FL_CMD_AT_DELTA_SETUP:
        Aif2Fl_atDeltaSetup(hAif2, *(uint32_t  *)arg);
        break;
   /** Sets AT Halt  timer  (argument type:  uint16_t  *)  */
    case AIF2FL_CMD_AT_HALT_TIMER:
        Aif2Fl_atHaltTimer(hAif2, *(uint16_t  *)arg);
        break;
   /** Sets AT diable all events for debug purpose (argument type:  uint16_t  *)  */
    case AIF2FL_CMD_AT_DISABLE_ALL_EVENTS:
        Aif2Fl_atDisableAllEvents(hAif2, *(uint16_t  *)arg);
        break;
    /** Sets AT Arm timer  (argument type:  uint16_t  *)  */
    case AIF2FL_CMD_AT_ARM_TIMER:
        Aif2Fl_atArmTimer(hAif2, *(uint16_t  *)arg);
        break;
   /** Sets AT Phy debug sync  (argument type:  uint16_t  *)  */
    case AIF2FL_CMD_AT_DEBUG_SYNC:
        Aif2Fl_atSwDebugSync(hAif2, *(uint16_t  *)arg);
        break;
    /** Sets AT radt wcdma clock divider terminal count  (argument type:  uint8_t  *)  */
    case AIF2FL_CMD_AT_RAD_WCDMA_DIV:
        Aif2Fl_atRadWcdmaDiv(hAif2, *(uint8_t  *)arg);
        break;
    /** Sets AT Rad terminal count  (argument type:  Aif2Fl_AtTcObj  *)  */
    case AIF2FL_CMD_AT_RAD_TC_SETUP:
        Aif2Fl_atRadTcSetup(hAif2, *(Aif2Fl_AtTcObj  *)arg);
        break;
    /** Sets AT GSM Tcount  (argument type:  Aif2Fl_AtGsmTCount  *)  */
    case AIF2FL_CMD_AT_GSM_TCOUNT_SETUP:
        Aif2Fl_atGsmTcountSetup(hAif2, *(Aif2Fl_AtGsmTCount  *)arg);
        break;
    /** Enable Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
    case AIF2FL_CMD_AT_ENABLE_EVENT:
        Aif2Fl_atEnableEvent(hAif2, *(Aif2Fl_AtEventIndex  *)arg);
        break;
    /** Disable Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
    case AIF2FL_CMD_AT_DISABLE_EVENT:
        Aif2Fl_atDisableEvent(hAif2, *(Aif2Fl_AtEventIndex  *)arg);
        break;
    /** Force set Eight Rad and Six DIO Events  (argument type:  Aif2Fl_AtEventIndex  *)  */
    case AIF2FL_CMD_AT_FORCE_EVENT:
        Aif2Fl_atForceEvent(hAif2, *(Aif2Fl_AtEventIndex  *)arg);
        break;
    /** EE End of interrupt vector value setup  (argument type:  uint8_t  *)  */
    case AIF2FL_CMD_EE_EOI_SETUP:
        Aif2Fl_eeEoiSetup(hAif2, *(uint8_t  *)arg);
        break;
    /** EE VB error interrupt set or clear (use hAif2->ee_arg to select between set and clear   argument type:  Aif2Fl_EeAif2Int  *)  */
    case AIF2FL_CMD_EE_AIF2_ERROR_INT:
        Aif2Fl_eeAif2ErrorIntSetup(hAif2, *(Aif2Fl_EeAif2Int  *)arg);
        break;
    /** EE DB interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeDbInt  *)  */
    case AIF2FL_CMD_EE_DB_INT:
        Aif2Fl_eeDbIntSetup(hAif2, *(Aif2Fl_EeDbInt  *)arg);
        break;
    /** EE AD interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeAdInt  *)  */
    case AIF2FL_CMD_EE_AD_INT:
        Aif2Fl_eeAdIntSetup(hAif2, *(Aif2Fl_EeAdInt  *)arg);
        break;
    /** EE CD(PKTDMA) interrupt set, clear, enable set or clear for EV0  (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeCdInt  *)  */
    case AIF2FL_CMD_EE_CD_INT:
        Aif2Fl_eeCdIntSetup(hAif2, *(Aif2Fl_EeCdInt  *)arg);
        break;
    /** EE SD interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeSdInt  *)  */
    case AIF2FL_CMD_EE_SD_INT:
        Aif2Fl_eeSdIntSetup(hAif2, *(Aif2Fl_EeSdInt  *)arg);
        break;
    /** EE VC interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeVcInt  *)  */
    case AIF2FL_CMD_EE_VC_INT:
        Aif2Fl_eeVcIntSetup(hAif2, *(Aif2Fl_EeVcInt  *)arg);
        break;
     /** EE Aif2 run control register setup (argument type:  Aif2Fl_EeAif2Run  *)  */
    case AIF2FL_CMD_EE_AIF2_RUN:
        Aif2Fl_eeAif2RunSetup(hAif2, *(Aif2Fl_EeAif2Run  *)arg);
        break;
    /** EE Link A interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function  and hAif2->arg_link to select link argument type:  Aif2Fl_EeLinkAInt  *)  */
    case AIF2FL_CMD_EE_LINKA_INT:
        Aif2Fl_eeLinkAIntSetup(hAif2, *(Aif2Fl_EeLinkAInt  *)arg);
        break;
    /** EE Link B interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function  and hAif2->arg_link to select link argument type:  Aif2Fl_EeLinkBInt  *)  */
    case AIF2FL_CMD_EE_LINKB_INT:
        Aif2Fl_eeLinkBIntSetup(hAif2, *(Aif2Fl_EeLinkBInt  *)arg);
        break;
    /** EE AT interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EeAtInt  *)  */
    case AIF2FL_CMD_EE_AT_INT:
        Aif2Fl_eeAtIntSetup(hAif2, *(Aif2Fl_EeAtInt  *)arg);
        break;
    /** EE PD common interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EePdInt  *)  */
    case AIF2FL_CMD_EE_PD_INT:
        Aif2Fl_eePdIntSetup(hAif2, *(Aif2Fl_EePdInt  *)arg);
        break;
    /** EE PE common interrupt set, clear, enable set or clear for EV0 and EV1 (use hAif2->ee_arg to select function   argument type:  Aif2Fl_EePeInt  *)  */
    case AIF2FL_CMD_EE_PE_INT:
        Aif2Fl_eePeIntSetup(hAif2, *(Aif2Fl_EePeInt  *)arg);
        break;
    default:
        status = AIF2FL_INVCMD;
    }
    
    return status;
}

