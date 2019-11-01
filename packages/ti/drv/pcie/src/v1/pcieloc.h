/*
 *
 * Copyright (C) 2010-2016 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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

/* ================================================================= */
/*  file  pcieloc.h
 *
 *  Internal module data structures and definitions
 *
 */
#ifndef PCIEV1LOC__H
#define PCIEV1LOC__H

#ifdef __cplusplus
extern "C" {
#endif


/* System level header files */
#include "pcie.h"
/* this is v1-specific driver so it can use v1 CSL directly */
#include <ti/csl/src/ip/pcie/V1/cslr_rc_cfg_dbics.h>
#include <ti/csl/src/ip/pcie/V1/cslr_ep_cfg_dbics.h>
#include <ti/csl/src/ip/pcie/V1/cslr_pl_conf.h>
#include <ti/csl/src/ip/pcie/V1/cslr_pcie.h>

/* Common utility macros */

/******************************************************************************
 * Prototypes
 ******************************************************************************/

/*****************************************************************************************
*Application Registers
*****************************************************************************************/
typedef void *CSL_Pciess_appRegs;

/*****************************************************************************************
*Configuration Registers
*****************************************************************************************/

/*Type 0, Type1 Common Registers*/
pcieRet_e pciev1_read_vndDevId_reg
(
  volatile const Uint32 *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
);
pcieRet_e pciev1_write_vndDevId_reg
(
  volatile Uint32 *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
);
pcieRet_e pciev1_read_statusCmd_reg
(
  volatile const Uint32 *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
);
pcieRet_e pciev1_write_statusCmd_reg
(
  volatile Uint32 *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
);
pcieRet_e pciev1_read_revId_reg
(
  volatile const Uint32 *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
);
pcieRet_e pciev1_write_revId_reg
(
  volatile Uint32 *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
);
pcieRet_e pciev1_read_bist_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr, 
  pcieBistReg_t *swReg
);
pcieRet_e pciev1_write_bist_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr, 
  pcieBistReg_t *swReg
);

/*Type 0 (EP-only) Registers*/
pcieRet_e pciev1_read_type0Bar_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_write_type0Bar_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_read_type0Bar32bit_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_write_type0Bar32bit_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_read_subId_reg
(
  const CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieSubIdReg_t *swReg
);
pcieRet_e pciev1_write_subId_reg
(
  CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieSubIdReg_t *swReg
);
pcieRet_e pciev1_read_cardbusCisPointer_reg
(
  const CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieCardbusCisPointerReg_t *swReg
);
pcieRet_e pciev1_write_cardbusCisPointer_reg
(
  CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieCardbusCisPointerReg_t *swReg
);
pcieRet_e pciev1_read_expRom_reg
(
  const CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieExpRomReg_t *swReg
);
pcieRet_e pciev1_write_expRom_reg
(
  CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieExpRomReg_t *swReg
);
pcieRet_e pciev1_read_capPtr_reg
(
  const CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieCapPtrReg_t *swReg
);
pcieRet_e pciev1_write_capPtr_reg
(
  CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieCapPtrReg_t *swReg
);
pcieRet_e pciev1_read_intPin_reg
(
  const CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieIntPinReg_t *swReg
);
pcieRet_e pciev1_write_intPin_reg
(
  CSL_EpCfgDbIcsRegs   *baseAddr, 
  pcieIntPinReg_t *swReg
);

/* power management capabilities*/
pcieRet_e pciev1_read_pmCap_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapReg_t            *swReg
);
pcieRet_e pciev1_write_pmCap_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapReg_t      *swReg
);
pcieRet_e pciev1_read_pmCapCtlStat_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapCtlStatReg_t     *swReg
);
pcieRet_e pciev1_write_pmCapCtlStat_reg
(
  CSL_EpCfgDbIcsRegs    *baseAddr,  
  pciePMCapCtlStatReg_t *swReg
);
/* MSI capabilities*/
pcieRet_e pciev1_read_msiCap_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiCapReg_t           *swReg
);
pcieRet_e pciev1_write_msiCap_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiCapReg_t     *swReg
);
pcieRet_e pciev1_read_msiLo32_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiLo32Reg_t          *swReg
);
pcieRet_e pciev1_write_msiLo32_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiLo32Reg_t    *swReg
);
pcieRet_e pciev1_read_msiUp32_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiUp32Reg_t          *swReg
);
pcieRet_e pciev1_write_msiUp32_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiUp32Reg_t    *swReg
);
pcieRet_e pciev1_read_msiData_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiDataReg_t          *swReg
);
pcieRet_e pciev1_write_msiData_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiDataReg_t    *swReg
);

/*Type 1 Registers*/
pcieRet_e pciev1_read_type1BistHeader_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieType1BistHeaderReg_t *swReg
);
pcieRet_e pciev1_write_type1BistHeader_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieType1BistHeaderReg_t *swReg
);
pcieRet_e pciev1_read_type1Bar_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_write_type1Bar_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_read_type1Bar32bit_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_write_type1Bar32bit_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev1_read_type1BusNum_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BusNumReg_t *swReg
);
pcieRet_e pciev1_write_type1BusNum_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BusNumReg_t *swReg
);
pcieRet_e pciev1_read_type1SecStat_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1SecStatReg_t *swReg
);
pcieRet_e pciev1_write_type1SecStat_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1SecStatReg_t *swReg
);
pcieRet_e pciev1_read_type1Memspace_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1MemspaceReg_t *swReg
);
pcieRet_e pciev1_write_type1Memspace_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1MemspaceReg_t *swReg
);
pcieRet_e pciev1_read_prefMem_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefMemReg_t *swReg
);
pcieRet_e pciev1_write_prefMem_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefMemReg_t *swReg
);
pcieRet_e pciev1_read_prefBaseUpper_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *swReg
);
pcieRet_e pciev1_write_prefBaseUpper_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *swReg
);
pcieRet_e pciev1_read_prefLimitUpper_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *swReg
);
pcieRet_e pciev1_write_prefLimitUpper_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *swReg
);
pcieRet_e pciev1_read_type1IOSpace_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *swReg
);
pcieRet_e pciev1_write_type1IOSpace_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *swReg
);
pcieRet_e pciev1_read_type1CapPtr_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1CapPtrReg_t *swReg
);
pcieRet_e pciev1_write_type1CapPtr_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1CapPtrReg_t *swReg
);
pcieRet_e pciev1_read_type1ExpnsnRom_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *swReg
);
pcieRet_e pciev1_write_type1ExpnsnRom_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *swReg
);
pcieRet_e pciev1_read_type1BridgeInt_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *swReg
);
pcieRet_e pciev1_write_type1BridgeInt_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *swReg
);

/*Capabilities Registers*/
pcieRet_e pciev1_read_pciesCap_reg
(
  volatile const Uint32 *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg 
);
pcieRet_e pciev1_write_pciesCap_reg
(
  volatile Uint32 *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg 
);
pcieRet_e pciev1_read_deviceCap_reg
(
  volatile const Uint32 *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg 
);
pcieRet_e pciev1_write_deviceCap_reg
(
  volatile Uint32 *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg 
);
pcieRet_e pciev1_read_devStatCtrl_reg
(
  volatile const Uint32 *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
);
pcieRet_e pciev1_write_devStatCtrl_reg
(
  volatile Uint32 *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
);
pcieRet_e pciev1_read_linkCap_reg
(      
  volatile const Uint32 *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg 
);
pcieRet_e pciev1_write_linkCap_reg
(      
  volatile Uint32 *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg 
);
pcieRet_e pciev1_read_linkStatCtrl_reg
(
  volatile const Uint32 *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg 
);
pcieRet_e pciev1_write_linkStatCtrl_reg
(
  volatile Uint32 *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg 
);
pcieRet_e pciev1_read_slotCap_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotCapReg_t *swReg 
);
pcieRet_e pciev1_write_slotCap_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotCapReg_t *swReg 
);
pcieRet_e pciev1_read_slotStatCtrl_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *swReg 
);
pcieRet_e pciev1_write_slotStatCtrl_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *swReg 
);
pcieRet_e pciev1_read_rootCtrlCap_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *swReg 
);
pcieRet_e pciev1_write_rootCtrlCap_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *swReg 
);
pcieRet_e pciev1_read_rootStatus_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootStatusReg_t *swReg 
);
pcieRet_e pciev1_write_rootStatus_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootStatusReg_t *swReg 
);
pcieRet_e pciev1_read_devCap2_reg
(
  volatile const Uint32 *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg 
);
pcieRet_e pciev1_write_devCap2_reg
(
  volatile Uint32 *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg 
);
pcieRet_e pciev1_read_devStatCtrl2_reg
(
  volatile const Uint32 *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg 
);
pcieRet_e pciev1_write_devStatCtrl2_reg
(
  volatile Uint32 *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg 
);
pcieRet_e pciev1_read_linkCap2_reg
(
  volatile const Uint32 *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg 
);
pcieRet_e pciev1_write_linkCap2_reg
(
  volatile Uint32 *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg 
);
pcieRet_e pciev1_read_linkCtrl2_reg
(
  volatile const Uint32 *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg 
);
pcieRet_e pciev1_write_linkCtrl2_reg
(
  volatile Uint32 *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg 
);

/*Capabilities Extended Registers*/
/* No extended registers available on rev 1 hw */

/*Port Logic Registers*/
pcieRet_e pciev1_read_plAckTimer_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pciePlAckTimerReg_t *swReg
);
pcieRet_e pciev1_write_plAckTimer_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pciePlAckTimerReg_t *swReg
);
pcieRet_e pciev1_read_plOMsg_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pciePlOMsgReg_t *swReg
);
pcieRet_e pciev1_write_plOMsg_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pciePlOMsgReg_t *swReg
);
pcieRet_e pciev1_read_plForceLink_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pciePlForceLinkReg_t *swReg
);
pcieRet_e pciev1_write_plForceLink_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pciePlForceLinkReg_t *swReg
);
pcieRet_e pciev1_read_ackFreq_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieAckFreqReg_t *swReg
);
pcieRet_e pciev1_write_ackFreq_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieAckFreqReg_t *swReg
);
pcieRet_e pciev1_read_lnkCtrl_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieLnkCtrlReg_t *swReg
);
pcieRet_e pciev1_write_lnkCtrl_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieLnkCtrlReg_t *swReg
);
pcieRet_e pciev1_read_laneSkew_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieLaneSkewReg_t *swReg
);
pcieRet_e pciev1_write_laneSkew_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieLaneSkewReg_t *swReg
);
pcieRet_e pciev1_read_symNum_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieSymNumReg_t *swReg
);
pcieRet_e pciev1_write_symNum_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieSymNumReg_t *swReg
);
pcieRet_e pciev1_read_symTimerFltMask_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieSymTimerFltMaskReg_t *swReg
);
pcieRet_e pciev1_write_symTimerFltMask_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieSymTimerFltMaskReg_t *swReg
);
pcieRet_e pciev1_read_fltMask2_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieFltMask2Reg_t *swReg
);
pcieRet_e pciev1_write_fltMask2_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieFltMask2Reg_t *swReg
);
pcieRet_e pciev1_read_gen2_reg
(
  const CSL_PlConfRegs  *baseAddr,  
  pcieGen2Reg_t *swReg
);
pcieRet_e pciev1_write_gen2_reg
(
  CSL_PlConfRegs  *baseAddr,  
  pcieGen2Reg_t *swReg
);
/* hw rev 1 only PL CONF regs */
pcieRet_e pciev1_read_plconfObnpSubreqCtrl_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
);
pcieRet_e pciev1_write_plconfObnpSubreqCtrl_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
);
pcieRet_e pciev1_read_plconfTrPStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrPStsRReg_t *swReg
);
pcieRet_e pciev1_read_plconfTrNpStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrNpStsRReg_t *swReg
);
pcieRet_e pciev1_read_plconfTrCStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrCStsRReg_t *swReg
);
pcieRet_e pciev1_read_plconfQStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
);
pcieRet_e pciev1_write_plconfQStsR_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
);
pcieRet_e pciev1_read_plconfVcTrAR1_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVcTrAR1Reg_t *swReg
);
pcieRet_e pciev1_read_plconfVcTrAR2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVcTrAR2Reg_t *swReg
);
pcieRet_e pciev1_read_plconfVc0PrQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0PrQCReg_t *swReg
);
pcieRet_e pciev1_write_plconfVc0PrQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0PrQCReg_t *swReg
);
pcieRet_e pciev1_read_plconfVc0NprQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0NprQCReg_t *swReg
);
pcieRet_e pciev1_write_plconfVc0NprQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0NprQCReg_t *swReg
);
pcieRet_e pciev1_read_plconfVc0CrQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0CrQCReg_t *swReg
);
pcieRet_e pciev1_write_plconfVc0CrQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0CrQCReg_t *swReg
);
pcieRet_e pciev1_read_plconfPhyStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPhyStsRReg_t *swReg
);
pcieRet_e pciev1_read_plconfPhyCtrlR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
);
pcieRet_e pciev1_write_plconfPhyCtrlR_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
);
pcieRet_e pciev1_read_plconfMsiCtrlAddress_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
);
pcieRet_e pciev1_write_plconfMsiCtrlAddress_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
);
pcieRet_e pciev1_read_plconfMsiCtrlUpperAddress_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
);
pcieRet_e pciev1_write_plconfMsiCtrlUpperAddress_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
);
pcieRet_e pciev1_read_plconfMsiCtrlIntEnable_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_write_plconfMsiCtrlIntEnable_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_read_plconfMsiCtrlIntMask_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_write_plconfMsiCtrlIntMask_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_read_plconfMsiCtrlIntStatus_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_write_plconfMsiCtrlIntStatus_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
);
pcieRet_e pciev1_read_plconfMsiCtrlGpio_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
);
pcieRet_e pciev1_write_plconfMsiCtrlGpio_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
);
pcieRet_e pciev1_read_plconfPipeLoopback_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
);
pcieRet_e pciev1_write_plconfPipeLoopback_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
);
pcieRet_e pciev1_read_plconfDbiRoWrEn_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
);
pcieRet_e pciev1_write_plconfDbiRoWrEn_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
);
pcieRet_e pciev1_read_plconfAxiSlvErrResp_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
);
pcieRet_e pciev1_write_plconfAxiSlvErrResp_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
);
pcieRet_e pciev1_read_plconfAxiSlvTimeout_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
);
pcieRet_e pciev1_write_plconfAxiSlvTimeout_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuIndex_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuIndexReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuIndex_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuIndexReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegCtrl1_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl1Reg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegCtrl1_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl1Reg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegCtrl2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl2Reg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegCtrl2_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl2Reg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegLowerBase_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerBaseReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegLowerBase_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerBaseReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegUpperBase_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperBaseReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegUpperBase_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperBaseReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegLimit_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLimitReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegLimit_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLimitReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegLowerTarget_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerTargetReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegLowerTarget_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerTargetReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegUpperTarget_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperTargetReg_t *swReg
);
pcieRet_e pciev1_write_plconfIatuRegUpperTarget_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperTargetReg_t *swReg
);
pcieRet_e pciev1_read_plconfIatuRegCtrl3_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl3Reg_t *swReg
);


/* TI CONF registers */
pcieRet_e pciev1_read_tiConfRevision_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfRevisionReg_t *swReg
);
pcieRet_e pciev1_read_tiConfSysConfig_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfSysConfigReg_t *swReg
);
pcieRet_e pciev1_write_tiConfSysConfig_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfSysConfigReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqEoi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEoiReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqEoi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEoiReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqStatusRawMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMainReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqStatusRawMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMainReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqStatusMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMainReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqStatusMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMainReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqEnableSetMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMainReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqEnableSetMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMainReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqEnableClrMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMainReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqEnableClrMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMainReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqStatusRawMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMsiReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqStatusRawMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMsiReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqStatusMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMsiReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqStatusMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMsiReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqEnableSetMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMsiReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqEnableSetMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMsiReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIrqEnableClrMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMsiReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIrqEnableClrMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMsiReg_t *swReg
);
pcieRet_e pciev1_read_tiConfDeviceType_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceTypeReg_t *swReg
);
pcieRet_e pciev1_write_tiConfDeviceType_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceTypeReg_t *swReg
);
pcieRet_e pciev1_read_tiConfDeviceCmd_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceCmdReg_t *swReg
);
pcieRet_e pciev1_write_tiConfDeviceCmd_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceCmdReg_t *swReg
);
pcieRet_e pciev1_read_tiConfPmCtrl_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfPmCtrlReg_t *swReg
);
pcieRet_e pciev1_write_tiConfPmCtrl_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfPmCtrlReg_t *swReg
);
pcieRet_e pciev1_read_tiConfPhyCs_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfPhyCsReg_t *swReg
);
pcieRet_e pciev1_write_tiConfPhyCs_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfPhyCsReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIntxAssert_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIntxAssertReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIntxAssert_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIntxAssertReg_t *swReg
);
pcieRet_e pciev1_read_tiConfIntxDeassert_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIntxDeassertReg_t *swReg
);
pcieRet_e pciev1_write_tiConfIntxDeassert_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIntxDeassertReg_t *swReg
);
pcieRet_e pciev1_read_tiConfMsiXmt_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfMsiXmtReg_t *swReg
);
pcieRet_e pciev1_write_tiConfMsiXmt_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfMsiXmtReg_t *swReg
);
pcieRet_e pciev1_read_tiConfDebugCfg_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDebugCfgReg_t *swReg
);
pcieRet_e pciev1_write_tiConfDebugCfg_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDebugCfgReg_t *swReg
);
pcieRet_e pciev1_read_tiConfDebugData_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDebugDataReg_t *swReg
);
pcieRet_e pciev1_read_tiConfDiagCtrl_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDiagCtrlReg_t *swReg
);
pcieRet_e pciev1_write_tiConfDiagCtrl_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDiagCtrlReg_t *swReg
);

/* MASK/SHIFT for backwards compatibility with old APIs */
#define PCIE_EP_BAR_BASE_FULL_MASK (CSL_EPCFGDBICS_BAR_BASE_ADDR_RO_MASK | CSL_EPCFGDBICS_BAR_BASE_ADDR_RW_MASK)
#define PCIE_EP_BAR_BASE_FULL_SHIFT (CSL_EPCFGDBICS_BAR_BASE_ADDR_RO_SHIFT)

#define PCIE_RC_BAR_BASE_FULL_MASK (CSL_RCCFGDBICS_BAR_BASE_ADDR_RO_MASK | CSL_RCCFGDBICS_BAR_BASE_ADDR_RW_MASK)
#define PCIE_RC_BAR_BASE_FULL_SHIFT (CSL_RCCFGDBICS_BAR_BASE_ADDR_RO_SHIFT)

#ifdef __cplusplus
}
#endif

#endif  /* _PCIEV1LOC_H */

/* Nothing past this point */

