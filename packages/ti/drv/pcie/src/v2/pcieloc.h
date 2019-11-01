/*
 *
 * Copyright (C) 2010-2018 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef PCIEV2LOC__H
#define PCIEV2LOC__H

#ifdef __cplusplus
extern "C" {
#endif


/* System level header files */
#include "pcie.h"
/* this is v2-specific driver so it can use v1 CSL directly */

/* this is v2-specific driver so it can use v2 CSL directly */
#include <ti/csl/src/ip/pcie/V2/cslr_pcie_rc.h>
#include <ti/csl/src/ip/pcie/V2/cslr_pcie_ep.h>
#include <ti/csl/src/ip/pcie/V2/cslr_vmap.h>

/* Common utility macros */

/******************************************************************************
 * Prototypes
 ******************************************************************************/

/*****************************************************************************************
*Application Registers
*****************************************************************************************/
pcieRet_e pciev2_read_pid_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePidReg_t               *swReg
);
pcieRet_e pciev2_read_cmdStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCmdStatusReg_t         *reg
);
pcieRet_e pciev2_write_cmdStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCmdStatusReg_t   *reg
);
pcieRet_e pciev2_read_rstCmd_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieRstCmdReg_t            *reg
);
pcieRet_e pciev2_write_rstCmd_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieRstCmdReg_t      *reg
);
pcieRet_e pciev2_read_ptmCfg_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmCfgReg_t            *reg
);
pcieRet_e pciev2_write_ptmCfg_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmCfgReg_t      *reg
);
pcieRet_e pciev2_read_pmCmd_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmCmdReg_t             *reg
);
pcieRet_e pciev2_write_pmCmd_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePmCmdReg_t       *reg
);
pcieRet_e pciev2_read_irqEOI_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieIrqEOIReg_t            *reg
);
pcieRet_e pciev2_write_irqEOI_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieIrqEOIReg_t      *reg
);
pcieRet_e pciev2_read_msiIrq_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqReg_t            *reg
);
pcieRet_e pciev2_write_msiIrq_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqReg_t      *reg
);
pcieRet_e pciev2_read_epIrqSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqSetReg_t          *reg
);
pcieRet_e pciev2_write_epIrqSet_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqSetReg_t    *reg
);
pcieRet_e pciev2_read_epIrqClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqClrReg_t          *reg
);
pcieRet_e pciev2_write_epIrqClr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqClrReg_t    *reg
);
pcieRet_e pciev2_read_epIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqStatusReg_t       *reg
);
pcieRet_e pciev2_write_epIrqStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqStatusReg_t *reg
);
pcieRet_e pciev2_read_genPurpose_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieGenPurposeReg_t        *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_genPurpose_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieGenPurposeReg_t  *reg,
  int_fast32_t          regNum
);
pcieRet_e pciev2_read_msiIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqStatusRawReg_t   *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_msiIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs      *baseAddr,
  pcieMsiIrqStatusRawReg_t  *reg,
  int_fast32_t               regNum
);
pcieRet_e pciev2_read_msiIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqStatusReg_t      *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_msiIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiIrqStatusReg_t *reg,
  int_fast32_t           regNum
);
pcieRet_e pciev2_read_msiIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqEnableSetReg_t   *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_msiIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieMsiIrqEnableSetReg_t *reg,
  int_fast32_t              regNum
);
pcieRet_e pciev2_read_msiIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqEnableClrReg_t   *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_msiIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieMsiIrqEnableClrReg_t *reg,
  int_fast32_t              regNum
);
pcieRet_e pciev2_read_legacyIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqStatusRawReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_write_legacyIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqStatusRawReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_read_legacyIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieLegacyIrqStatusReg_t   *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev2_write_legacyIrqStatus_reg
(
  CSL_pcie_ep_coreRegs      *baseAddr,
  pcieLegacyIrqStatusReg_t  *reg,
  int_fast32_t               regNum
);
pcieRet_e pciev2_read_legacyIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqEnableSetReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_write_legacyIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqEnableSetReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_read_legacyIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqEnableClrReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_write_legacyIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqEnableClrReg_t *reg,
  int_fast32_t                 regNum
);
pcieRet_e pciev2_read_errIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqStatusRawReg_t   *reg
);
pcieRet_e pciev2_write_errIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqStatusRawReg_t *reg
);
pcieRet_e pciev2_read_errIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqStatusReg_t      *reg
);
pcieRet_e pciev2_write_errIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieErrIrqStatusReg_t *reg
);
pcieRet_e pciev2_read_errIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqEnableSetReg_t   *reg
);
pcieRet_e pciev2_write_errIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqEnableSetReg_t *reg
);
pcieRet_e pciev2_read_errIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqEnableClrReg_t   *reg
);
pcieRet_e pciev2_write_errIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqEnableClrReg_t *reg
);
pcieRet_e pciev2_read_pmRstIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqStatusRawReg_t *reg
);
pcieRet_e pciev2_write_pmRstIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqStatusRawReg_t *reg
);
pcieRet_e pciev2_read_pmRstIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqStatusReg_t    *reg
);
pcieRet_e pciev2_write_pmRstIrqStatus_reg
(
  CSL_pcie_ep_coreRegs    *baseAddr,
  pciePmRstIrqStatusReg_t *reg
);
pcieRet_e pciev2_read_pmRstIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqEnableSetReg_t *reg
);
pcieRet_e pciev2_write_pmRstIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqEnableSetReg_t *reg
);
pcieRet_e pciev2_read_pmRstIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqEnableClrReg_t *reg
);
pcieRet_e pciev2_write_pmRstIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqEnableClrReg_t *reg
);
pcieRet_e pciev2_read_ptmIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqStatusRawReg_t   *reg
);
pcieRet_e pciev2_write_ptmIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqStatusRawReg_t *reg
);
pcieRet_e pciev2_read_ptmIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqStatusReg_t      *reg
);
pcieRet_e pciev2_write_ptmIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePtmIrqStatusReg_t *reg
);
pcieRet_e pciev2_read_ptmIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqEnableSetReg_t   *reg
);
pcieRet_e pciev2_write_ptmIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqEnableSetReg_t *reg
);
pcieRet_e pciev2_read_ptmIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqEnableClrReg_t   *reg
);
pcieRet_e pciev2_write_ptmIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqEnableClrReg_t *reg
);

/*****************************************************************************************
*Configuration Registers
*****************************************************************************************/

/*Type 0, Type1 Common Registers*/
pcieRet_e pciev2_read_vndDevId_reg
(
  volatile const uint32_t *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
);
pcieRet_e pciev2_write_vndDevId_reg
(
  volatile uint32_t *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
);
pcieRet_e pciev2_read_statusCmd_reg
(
  volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
);
pcieRet_e pciev2_write_statusCmd_reg
(
  volatile uint32_t *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
);
pcieRet_e pciev2_read_revId_reg
(
  volatile const uint32_t *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
);
pcieRet_e pciev2_write_revId_reg
(
  volatile uint32_t *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t *swReg
);
pcieRet_e pciev2_read_bist_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBistReg_t *swReg
);
pcieRet_e pciev2_write_bist_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBistReg_t *swReg
);

/*Type 0 (EP-only) Registers*/
pcieRet_e pciev2_read_type0Bar_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev2_write_type0Bar_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBarReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev2_read_type0Bar32bit_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev2_write_type0Bar32bit_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieBar32bitReg_t *swReg,
  int32_t barNum
);
pcieRet_e pciev2_read_subId_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieSubIdReg_t *swReg
);
pcieRet_e pciev2_write_subId_reg
(
  CSL_pcie_ep_coreRegs   *baseAddr,
  pcieSubIdReg_t *swReg
);
pcieRet_e pciev2_read_cardbusCisPointer_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieCardbusCisPointerReg_t *swReg
);
pcieRet_e pciev2_write_cardbusCisPointer_reg
(
  CSL_pcie_ep_coreRegs   *baseAddr,
  pcieCardbusCisPointerReg_t *swReg
);
pcieRet_e pciev2_read_expRom_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieExpRomReg_t *swReg
);
pcieRet_e pciev2_write_expRom_reg
(
  CSL_pcie_ep_coreRegs   *baseAddr,
  pcieExpRomReg_t *swReg
);
pcieRet_e pciev2_read_capPtr_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieCapPtrReg_t *swReg
);
pcieRet_e pciev2_write_capPtr_reg
(
  CSL_pcie_ep_coreRegs   *baseAddr,
  pcieCapPtrReg_t *swReg
);
pcieRet_e pciev2_read_intPin_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieIntPinReg_t *swReg
);
pcieRet_e pciev2_write_intPin_reg
(
  CSL_pcie_ep_coreRegs   *baseAddr,
  pcieIntPinReg_t *swReg
);

/* power management capabilities*/
pcieRet_e pciev2_read_pmCap_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapReg_t            *swReg
);
pcieRet_e pciev2_write_pmCap_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapReg_t      *swReg
);
pcieRet_e pciev2_read_pmCapCtlStat_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapCtlStatReg_t     *swReg
);
pcieRet_e pciev2_write_pmCapCtlStat_reg
(
  CSL_pcie_ep_coreRegs    *baseAddr,
  pciePMCapCtlStatReg_t *swReg
);
/* MSI capabilities*/
pcieRet_e pciev2_read_msiCap_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapReg_t           *swReg
);
pcieRet_e pciev2_write_msiCap_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapReg_t     *swReg
);
pcieRet_e pciev2_read_msiLo32_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiLo32Reg_t          *swReg
);
pcieRet_e pciev2_write_msiLo32_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiLo32Reg_t    *swReg
);
pcieRet_e pciev2_read_msiUp32_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiUp32Reg_t          *swReg
);
pcieRet_e pciev2_write_msiUp32_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiUp32Reg_t    *swReg
);
pcieRet_e pciev2_read_msiData_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiDataReg_t          *swReg
);
pcieRet_e pciev2_write_msiData_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiDataReg_t    *swReg
);
pcieRet_e pciev2_read_msiCapOff10H_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff10HReg_t          *swReg
);
pcieRet_e pciev2_write_msiCapOff10H_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff10HReg_t    *swReg
);
pcieRet_e pciev2_read_msiCapOff14H_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff14HReg_t          *swReg
);
pcieRet_e pciev2_write_msiCapOff14H_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff14HReg_t    *swReg
);

/*Type 1 Registers*/
pcieRet_e pciev2_read_type1BistHeader_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieType1BistHeaderReg_t *swReg
);
pcieRet_e pciev2_write_type1BistHeader_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieType1BistHeaderReg_t *swReg
);
pcieRet_e pciev2_read_type1BusNum_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BusNumReg_t *swReg
);
pcieRet_e pciev2_write_type1BusNum_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BusNumReg_t *swReg
);
pcieRet_e pciev2_read_type1SecStat_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1SecStatReg_t *swReg
);
pcieRet_e pciev2_write_type1SecStat_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1SecStatReg_t *swReg
);
pcieRet_e pciev2_read_type1Memspace_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1MemspaceReg_t *swReg
);
pcieRet_e pciev2_write_type1Memspace_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1MemspaceReg_t *swReg
);
pcieRet_e pciev2_read_prefMem_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefMemReg_t *swReg
);
pcieRet_e pciev2_write_prefMem_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefMemReg_t *swReg
);
pcieRet_e pciev2_read_prefBaseUpper_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefBaseUpperReg_t *swReg
);
pcieRet_e pciev2_write_prefBaseUpper_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefBaseUpperReg_t *swReg
);
pcieRet_e pciev2_read_prefLimitUpper_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefLimitUpperReg_t *swReg
);
pcieRet_e pciev2_write_prefLimitUpper_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefLimitUpperReg_t *swReg
);
pcieRet_e pciev2_read_type1IOSpace_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1IOSpaceReg_t *swReg
);
pcieRet_e pciev2_write_type1IOSpace_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1IOSpaceReg_t *swReg
);
pcieRet_e pciev2_read_type1CapPtr_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1CapPtrReg_t *swReg
);
pcieRet_e pciev2_write_type1CapPtr_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1CapPtrReg_t *swReg
);
pcieRet_e pciev2_read_type1ExpnsnRom_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1ExpnsnRomReg_t *swReg
);
pcieRet_e pciev2_write_type1ExpnsnRom_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1ExpnsnRomReg_t *swReg
);
pcieRet_e pciev2_read_type1BridgeInt_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BridgeIntReg_t *swReg
);
pcieRet_e pciev2_write_type1BridgeInt_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BridgeIntReg_t *swReg
);

/*Capabilities Registers*/
pcieRet_e pciev2_read_pciesCap_reg
(
  volatile const uint32_t *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg
);
pcieRet_e pciev2_write_pciesCap_reg
(
  volatile uint32_t *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg
);
pcieRet_e pciev2_read_deviceCap_reg
(
  volatile const uint32_t *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg
);
pcieRet_e pciev2_write_deviceCap_reg
(
  volatile uint32_t *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg
);
pcieRet_e pciev2_read_devStatCtrl_reg
(
  volatile const uint32_t *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
);
pcieRet_e pciev2_write_devStatCtrl_reg
(
  volatile uint32_t *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
);
pcieRet_e pciev2_read_linkCap_reg
(
  volatile const uint32_t *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg
);
pcieRet_e pciev2_write_linkCap_reg
(
  volatile uint32_t *hwReg_LNK_CAP,
  pcieLinkCapReg_t *swReg
);
pcieRet_e pciev2_read_linkStatCtrl_reg
(
  volatile const uint32_t *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg
);
pcieRet_e pciev2_write_linkStatCtrl_reg
(
  volatile uint32_t *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg
);
pcieRet_e pciev2_read_slotCap_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotCapReg_t *swReg
);
pcieRet_e pciev2_write_slotCap_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotCapReg_t *swReg
);
pcieRet_e pciev2_read_slotStatCtrl_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotStatCtrlReg_t *swReg
);
pcieRet_e pciev2_write_slotStatCtrl_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotStatCtrlReg_t *swReg
);
pcieRet_e pciev2_read_rootCtrlCap_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootCtrlCapReg_t *swReg
);
pcieRet_e pciev2_write_rootCtrlCap_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootCtrlCapReg_t *swReg
);
pcieRet_e pciev2_read_rootStatus_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootStatusReg_t *swReg
);
pcieRet_e pciev2_write_rootStatus_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootStatusReg_t *swReg
);
pcieRet_e pciev2_read_devCap2_reg
(
  volatile const uint32_t *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg
);
pcieRet_e pciev2_write_devCap2_reg
(
  volatile uint32_t *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t *swReg
);
pcieRet_e pciev2_read_devStatCtrl2_reg
(
  volatile const uint32_t *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg
);
pcieRet_e pciev2_write_devStatCtrl2_reg
(
  volatile uint32_t *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg
);
pcieRet_e pciev2_read_linkCap2_reg
(
  volatile const uint32_t *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg
);
pcieRet_e pciev2_write_linkCap2_reg
(
  volatile uint32_t *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t *swReg
);
pcieRet_e pciev2_read_linkCtrl2_reg
(
  volatile const uint32_t *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg
);
pcieRet_e pciev2_write_linkCtrl2_reg
(
  volatile uint32_t *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg
);

/*Capabilities Extended Registers*/
pcieRet_e pciev2_read_extCap_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieExtCapReg_t            *reg
);
pcieRet_e pciev2_read_uncErr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrReg_t            *reg
);
pcieRet_e pciev2_write_uncErr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrReg_t      *reg
);
pcieRet_e pciev2_read_uncErrMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrMaskReg_t        *reg
);
pcieRet_e pciev2_write_uncErrMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrMaskReg_t  *reg
);
pcieRet_e pciev2_read_uncErrSvrty_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrSvrtyReg_t       *reg
);
pcieRet_e pciev2_write_uncErrSvrty_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrSvrtyReg_t *reg
);
pcieRet_e pciev2_read_corErr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrReg_t            *reg
);
pcieRet_e pciev2_write_corErr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrReg_t      *reg
);
pcieRet_e pciev2_read_corErrMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrMaskReg_t        *reg
);
pcieRet_e pciev2_write_corErrMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrMaskReg_t  *reg
);
pcieRet_e pciev2_read_accr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieAccrReg_t              *reg
);
pcieRet_e pciev2_write_accr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieAccrReg_t        *reg
);
pcieRet_e pciev2_read_hdrLog_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieHdrLogReg_t            *reg,
  int32_t                    regNum
);
pcieRet_e pciev2_read_rootErrCmd_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrCmdReg_t        *reg
);
pcieRet_e pciev2_write_rootErrCmd_reg
(
  CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrCmdReg_t  *reg
);
pcieRet_e pciev2_read_rootErrSt_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrStReg_t         *reg
);
pcieRet_e pciev2_write_rootErrSt_reg
(
  CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrStReg_t   *reg
);
pcieRet_e pciev2_read_errSrcID_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieErrSrcIDReg_t          *reg
);

/*Port Logic Registers*/
pcieRet_e pciev2_read_plAckTimer_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlAckTimerReg_t *swReg
);
pcieRet_e pciev2_write_plAckTimer_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlAckTimerReg_t *swReg
);
pcieRet_e pciev2_read_plOMsg_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlOMsgReg_t *swReg
);
pcieRet_e pciev2_write_plOMsg_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlOMsgReg_t *swReg
);
pcieRet_e pciev2_read_plForceLink_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlForceLinkReg_t *swReg
);
pcieRet_e pciev2_write_plForceLink_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePlForceLinkReg_t *swReg
);
pcieRet_e pciev2_read_ackFreq_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieAckFreqReg_t *swReg
);
pcieRet_e pciev2_write_ackFreq_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieAckFreqReg_t *swReg
);
pcieRet_e pciev2_read_lnkCtrl_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLnkCtrlReg_t *swReg
);
pcieRet_e pciev2_write_lnkCtrl_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLnkCtrlReg_t *swReg
);
pcieRet_e pciev2_read_laneSkew_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLaneSkewReg_t *swReg
);
pcieRet_e pciev2_write_laneSkew_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLaneSkewReg_t *swReg
);
pcieRet_e pciev2_read_symNum_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieSymNumReg_t *swReg
);
pcieRet_e pciev2_write_symNum_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieSymNumReg_t *swReg
);
pcieRet_e pciev2_read_symTimerFltMask_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
);
pcieRet_e pciev2_write_symTimerFltMask_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
);
pcieRet_e pciev2_read_fltMask2_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieFltMask2Reg_t *swReg
);
pcieRet_e pciev2_write_fltMask2_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieFltMask2Reg_t *swReg
);
pcieRet_e pciev2_read_debug0_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieDebug0Reg_t            *reg
);
pcieRet_e pciev2_read_debug1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieDebug1Reg_t            *reg
);
pcieRet_e pciev2_read_gen2_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieGen2Reg_t *swReg
);
pcieRet_e pciev2_write_gen2_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieGen2Reg_t *swReg
);

/* hw rev 1/2 only PL CONF regs */
pcieRet_e pciev2_read_plconfObnpSubreqCtrl_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
);
pcieRet_e pciev2_write_plconfObnpSubreqCtrl_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
);
pcieRet_e pciev2_read_plconfTrPStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrPStsRReg_t *swReg
);
pcieRet_e pciev2_read_plconfTrNpStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrNpStsRReg_t *swReg
);
pcieRet_e pciev2_read_plconfTrCStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrCStsRReg_t *swReg
);
pcieRet_e pciev2_read_plconfQStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
);
pcieRet_e pciev2_write_plconfQStsR_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
);
pcieRet_e pciev2_read_plconfVcTrAR1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcTrAR1Reg_t *swReg
);
pcieRet_e pciev2_read_plconfVcTrAR2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcTrAR2Reg_t *swReg
);
pcieRet_e pciev2_read_plconfVcPrQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcPrQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_write_plconfVcPrQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcPrQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_read_plconfVcNprQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcNprQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_write_plconfVcNprQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcNprQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_read_plconfVcCrQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcCrQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_write_plconfVcCrQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcCrQCReg_t *swReg,
  int32_t vcNum
);
pcieRet_e pciev2_read_plconfPhyStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyStsRReg_t *swReg
);
pcieRet_e pciev2_read_plconfPhyCtrlR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
);
pcieRet_e pciev2_write_plconfPhyCtrlR_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
);
pcieRet_e pciev2_read_plconfMsiCtrlAddress_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
);
pcieRet_e pciev2_write_plconfMsiCtrlAddress_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
);
pcieRet_e pciev2_read_plconfMsiCtrlUpperAddress_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
);
pcieRet_e pciev2_write_plconfMsiCtrlUpperAddress_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
);
pcieRet_e pciev2_read_plconfMsiCtrlIntEnable_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_write_plconfMsiCtrlIntEnable_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_read_plconfMsiCtrlIntMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_write_plconfMsiCtrlIntMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_read_plconfMsiCtrlIntStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_write_plconfMsiCtrlIntStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
);
pcieRet_e pciev2_read_plconfMsiCtrlGpio_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
);
pcieRet_e pciev2_write_plconfMsiCtrlGpio_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
);
pcieRet_e pciev2_read_plconfPipeLoopback_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
);
pcieRet_e pciev2_write_plconfPipeLoopback_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
);
pcieRet_e pciev2_read_plconfDbiRoWrEn_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
);
pcieRet_e pciev2_write_plconfDbiRoWrEn_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
);
pcieRet_e pciev2_read_plconfAxiSlvErrResp_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
);
pcieRet_e pciev2_write_plconfAxiSlvErrResp_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
);
pcieRet_e pciev2_read_plconfAxiSlvTimeout_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
);
pcieRet_e pciev2_write_plconfAxiSlvTimeout_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegCtrl1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl1Reg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegCtrl1_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl1Reg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegCtrl2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl2Reg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegCtrl2_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl2Reg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegLowerBase_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerBaseReg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegLowerBase_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerBaseReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegUpperBase_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperBaseReg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegUpperBase_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperBaseReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegLimit_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLimitReg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegLimit_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLimitReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegLowerTarget_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerTargetReg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegLowerTarget_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerTargetReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegUpperTarget_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperTargetReg_t *swReg
);
pcieRet_e pciev2_write_plconfIatuRegUpperTarget_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperTargetReg_t *swReg
);
pcieRet_e pciev2_read_plconfIatuRegCtrl3_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl3Reg_t *swReg
);

/*****************************************************************************
 * pciev2 instance/context internal definition
 *****************************************************************************/
typedef struct
{
   /**
    * @brief Used to simulate v1 behavior where only one IATU config
    * is visible at time.  Instead of adding 16 windows to API, keep using old
    * set index model, except remember the index and direction to be read or
    * modified here
    */
   pciePlconfIatuIndexReg_t simIatuWindow[pcie_MAX_PERIPHS];
} Pciev2_LocalObj;

#ifdef __cplusplus
}
#endif

#endif  /* _PCIEV2LOC_H */

/* Nothing past this point */

