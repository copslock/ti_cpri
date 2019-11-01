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
#ifndef PCIEV0LOC__H
#define PCIEV0LOC__H

#ifdef __cplusplus
extern "C" {
#endif


/* System level header files */
#include "pcie.h"
/* this is v0-specific driver so it can use v0 CSL directly */
#include <ti/csl/src/ip/pcie/V0/cslr_pcie_cfg_space_endpoint.h>
#include <ti/csl/src/ip/pcie/V0/cslr_pcie_cfg_space_rootcomplex.h>
#include <ti/csl/src/ip/pcie/V0/cslr_pciess_app.h>

/* Common utility macros */

/* Get base address for Local Configuration Space */
#define pcie_get_loc_cfg_base(appBase, ep_loc_base, rc_loc_base) \
  {                                                \
    uint32_t temp_var = ((uint32_t)(appBase)) + 0x1000U; \
    (ep_loc_base) = (CSL_Pcie_cfg_space_endpointRegs *) (temp_var); \
    (rc_loc_base) = (CSL_Pcie_cfg_space_rootcomplexRegs *) (temp_var); \
  }
    
/* Get base address for Remote Configuration Space */
#define pcie_get_rem_cfg_base(appBase, ep_rem_base, rc_rem_base) \
  {                                                \
    uint32_t temp_var = ((uint32_t)(appBase)) + 0x2000U; \
    (ep_rem_base) = (CSL_Pcie_cfg_space_endpointRegs *) (temp_var); \
    (rc_rem_base) = (CSL_Pcie_cfg_space_rootcomplexRegs *) (temp_var); \
  }



/******************************************************************************
 * Prototypes
 ******************************************************************************/

/*****************************************************************************************
*Application Registers
*****************************************************************************************/

pcieRet_e pciev0_read_pid_reg(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePidReg_t               *reg);
pcieRet_e pciev0_read_cmdStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieCmdStatusReg_t         *reg);
pcieRet_e pciev0_write_cmdStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieCmdStatusReg_t   *reg);
pcieRet_e pciev0_read_cfgTrans_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieCfgTransReg_t          *reg);
pcieRet_e pciev0_write_cfgTrans_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieCfgTransReg_t    *reg);
pcieRet_e pciev0_read_ioBase_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIoBaseReg_t            *reg);
pcieRet_e pciev0_write_ioBase_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIoBaseReg_t      *reg);
pcieRet_e pciev0_read_tlpCfg_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieTlpCfgReg_t            *reg);
pcieRet_e pciev0_write_tlpCfg_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieTlpCfgReg_t      *reg);
pcieRet_e pciev0_read_rstCmd_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieRstCmdReg_t            *reg);
pcieRet_e pciev0_write_rstCmd_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieRstCmdReg_t      *reg);
pcieRet_e pciev0_read_pmCmd_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmCmdReg_t             *reg);
pcieRet_e pciev0_write_pmCmd_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmCmdReg_t       *reg);
pcieRet_e pciev0_read_pmCfg_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmCfgReg_t             *reg);
pcieRet_e pciev0_write_pmCfg_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmCfgReg_t       *reg);
pcieRet_e pciev0_read_actStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieActStatusReg_t         *reg);
pcieRet_e pciev0_read_obSize_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObSizeReg_t            *reg);
pcieRet_e pciev0_write_obSize_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObSizeReg_t      *reg);
pcieRet_e pciev0_read_diagCtrl_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieDiagCtrlReg_t          *reg 
);
pcieRet_e pciev0_write_diagCtrl_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieDiagCtrlReg_t    *reg 
);
pcieRet_e pciev0_read_endian_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEndianReg_t            *reg 
);
pcieRet_e pciev0_write_endian_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEndianReg_t      *reg 
);
pcieRet_e pciev0_read_priority_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePriorityReg_t          *reg 
);
pcieRet_e pciev0_write_priority_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePriorityReg_t    *reg 
);
pcieRet_e pciev0_read_irqEOI_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIrqEOIReg_t            *reg 
);
pcieRet_e pciev0_write_irqEOI_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIrqEOIReg_t      *reg 
);
pcieRet_e pciev0_read_msiIrq_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqReg_t            *reg 
);
pcieRet_e pciev0_write_msiIrq_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqReg_t      *reg 
);
pcieRet_e pciev0_read_epIrqSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqSetReg_t          *reg 
);
pcieRet_e pciev0_write_epIrqSet_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqSetReg_t    *reg 
);
pcieRet_e pciev0_read_epIrqClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqClrReg_t          *reg 
);
pcieRet_e pciev0_write_epIrqClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqClrReg_t    *reg 
);
pcieRet_e pciev0_read_epIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqStatusReg_t       *reg 
);
pcieRet_e pciev0_write_epIrqStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqStatusReg_t *reg 
);
pcieRet_e pciev0_read_genPurpose_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieGenPurposeReg_t        *reg,
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_genPurpose_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieGenPurposeReg_t  *reg,
  int_fast32_t          regNum
);
pcieRet_e pciev0_read_msiIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqStatusRawReg_t   *reg, 
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_msiIrqStatusRaw_reg
(
  CSL_Pciess_appRegs        *baseAddr, 
  pcieMsiIrqStatusRawReg_t  *reg, 
  int_fast32_t               regNum
);
pcieRet_e pciev0_read_msiIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqStatusReg_t      *reg, 
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_msiIrqStatus_reg
(
  CSL_Pciess_appRegs    *baseAddr, 
  pcieMsiIrqStatusReg_t *reg, 
  int_fast32_t           regNum
);
pcieRet_e pciev0_read_msiIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqEnableSetReg_t   *reg, 
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_msiIrqEnableSet_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieMsiIrqEnableSetReg_t *reg, 
  int_fast32_t              regNum
);
pcieRet_e pciev0_read_msiIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqEnableClrReg_t   *reg, 
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_msiIrqEnableClr_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieMsiIrqEnableClrReg_t *reg, 
  int_fast32_t              regNum
);
pcieRet_e pciev0_read_legacyIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs    *baseAddr, 
  pcieLegacyIrqStatusRawReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_write_legacyIrqStatusRaw_reg
(
  CSL_Pciess_appRegs          *baseAddr, 
  pcieLegacyIrqStatusRawReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_read_legacyIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqStatusReg_t   *reg, 
  int_fast32_t                regNum
);
pcieRet_e pciev0_write_legacyIrqStatus_reg
(
  CSL_Pciess_appRegs        *baseAddr, 
  pcieLegacyIrqStatusReg_t  *reg, 
  int_fast32_t               regNum
);
pcieRet_e pciev0_read_legacyIrqEnableSet_reg
(
  const CSL_Pciess_appRegs    *baseAddr, 
  pcieLegacyIrqEnableSetReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_write_legacyIrqEnableSet_reg
(
  CSL_Pciess_appRegs          *baseAddr, 
  pcieLegacyIrqEnableSetReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_read_legacyIrqEnableClr_reg
(
  const CSL_Pciess_appRegs    *baseAddr, 
  pcieLegacyIrqEnableClrReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_write_legacyIrqEnableClr_reg
(
  CSL_Pciess_appRegs          *baseAddr, 
  pcieLegacyIrqEnableClrReg_t *reg, 
  int_fast32_t                 regNum
);
pcieRet_e pciev0_read_errIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusRawReg_t   *reg
);
pcieRet_e pciev0_write_errIrqStatusRaw_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieErrIrqStatusRawReg_t *reg
);
pcieRet_e pciev0_read_errIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusReg_t      *reg
);
pcieRet_e pciev0_write_errIrqStatus_reg
(
  CSL_Pciess_appRegs    *baseAddr, 
  pcieErrIrqStatusReg_t *reg
);
pcieRet_e pciev0_read_errIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableSetReg_t   *reg
);
pcieRet_e pciev0_write_errIrqEnableSet_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieErrIrqEnableSetReg_t *reg
);
pcieRet_e pciev0_read_errIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableClrReg_t   *reg
);
pcieRet_e pciev0_write_errIrqEnableClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableClrReg_t *reg
);
pcieRet_e pciev0_read_pmRstIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusRawReg_t *reg
);
pcieRet_e pciev0_write_pmRstIrqStatusRaw_reg
(
  CSL_Pciess_appRegs         *baseAddr, 
  pciePmRstIrqStatusRawReg_t *reg
);
pcieRet_e pciev0_read_pmRstIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusReg_t    *reg
);
pcieRet_e pciev0_write_pmRstIrqStatus_reg
(
  CSL_Pciess_appRegs      *baseAddr, 
  pciePmRstIrqStatusReg_t *reg
);
pcieRet_e pciev0_read_pmRstIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableSetReg_t *reg
);
pcieRet_e pciev0_write_pmRstIrqEnableSet_reg
(
  CSL_Pciess_appRegs         *baseAddr, 
  pciePmRstIrqEnableSetReg_t *reg
);
pcieRet_e pciev0_read_pmRstIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableClrReg_t *reg
);
pcieRet_e pciev0_write_pmRstIrqEnableClr_reg
(
  CSL_Pciess_appRegs         *baseAddr, 
  pciePmRstIrqEnableClrReg_t *reg
);
pcieRet_e pciev0_read_obOffsetLo_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetLoReg_t        *reg,
  int_fast32_t   regNum);
pcieRet_e pciev0_write_obOffsetLo_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetLoReg_t  *reg,
  int_fast32_t   regNum);
pcieRet_e pciev0_read_obOffsetHi_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetHiReg_t        *reg,
  int_fast32_t                regNum);
pcieRet_e pciev0_write_obOffsetHi_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetHiReg_t  *reg,
  int_fast32_t          regNum);
pcieRet_e pciev0_read_ibBar_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbBarReg_t             *reg,
  int_fast32_t                regNum);
pcieRet_e pciev0_write_ibBar_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbBarReg_t       *reg,
  int_fast32_t          regNum);
pcieRet_e pciev0_read_ibStartLo_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartLoReg_t         *reg,
  int_fast32_t                regNum);
pcieRet_e pciev0_write_ibStartLo_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartLoReg_t   *reg,
  int_fast32_t          regNum);
pcieRet_e pciev0_read_ibStartHi_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartHiReg_t         *reg,
  int_fast32_t                regNum);
pcieRet_e pciev0_write_ibStartHi_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartHiReg_t   *reg,
  int_fast32_t          regNum);
pcieRet_e pciev0_read_ibOffset_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbOffsetReg_t          *reg,
  int_fast32_t                regNum);
pcieRet_e pciev0_write_ibOffset_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbOffsetReg_t    *reg,
  int_fast32_t          regNum);
pcieRet_e pciev0_read_pcsCfg0_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg0Reg_t           *reg 
);
pcieRet_e pciev0_write_pcsCfg0_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg0Reg_t     *reg 
);
pcieRet_e pciev0_read_pcsCfg1_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg1Reg_t           *reg 
);
pcieRet_e pciev0_write_pcsCfg1_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg1Reg_t     *reg 
);
pcieRet_e pciev0_read_pcsStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsStatusReg_t         *reg 
);

pcieRet_e pciev0_read_serdesCfg0_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg0Reg_t        *reg 
);
pcieRet_e pciev0_write_serdesCfg0_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg0Reg_t  *reg 
);
pcieRet_e pciev0_read_serdesCfg1_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg1Reg_t        *reg 
);
pcieRet_e pciev0_write_serdesCfg1_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg1Reg_t  *reg 
);


/*****************************************************************************************
*Configuration Registers
*****************************************************************************************/

/*Type 0, Type1 Common Registers*/
pcieRet_e pciev0_read_vndDevId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieVndDevIdReg_t                       *reg
);
pcieRet_e pciev0_write_vndDevId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieVndDevIdReg_t                 *reg
);
pcieRet_e pciev0_read_statusCmd_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieStatusCmdReg_t                      *reg
);
pcieRet_e pciev0_write_statusCmd_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieStatusCmdReg_t                *reg
);
pcieRet_e pciev0_read_revId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRevIdReg_t                          *reg
);
pcieRet_e pciev0_write_revId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRevIdReg_t                    *reg
);

/*Type 0 Registers*/
pcieRet_e pciev0_read_bist_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieBistReg_t                           *reg
);
pcieRet_e pciev0_write_bist_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieBistReg_t                     *reg
);
pcieRet_e pciev0_read_type0Bar_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBarReg_t                           *reg,
  int32_t                                 barNum
);
pcieRet_e pciev0_write_type0Bar_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBarReg_t                     *reg,
  int32_t                           barNum
);
pcieRet_e pciev0_read_type0Bar32bit_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBar32bitReg_t                      *reg,
  int32_t                                 barNum
);
pcieRet_e pciev0_write_type0Bar32bit_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBar32bitReg_t                *reg,
  int32_t                           barNum
);
pcieRet_e pciev0_read_subId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSubIdReg_t                          *reg
);
pcieRet_e pciev0_write_subId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSubIdReg_t                    *reg
);
pcieRet_e pciev0_read_expRom_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieExpRomReg_t                         *reg
);
pcieRet_e pciev0_write_expRom_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieExpRomReg_t                   *reg
);
pcieRet_e pciev0_read_capPtr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCapPtrReg_t                         *reg
);
pcieRet_e pciev0_write_capPtr_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCapPtrReg_t                   *reg
);
pcieRet_e pciev0_read_intPin_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieIntPinReg_t                         *reg
);
pcieRet_e pciev0_write_intPin_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieIntPinReg_t                   *reg
);

/*Type 1 Registers*/
pcieRet_e pciev0_read_type1BistHeader_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieType1BistHeaderReg_t                   *reg
);
pcieRet_e pciev0_write_type1BistHeader_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieType1BistHeaderReg_t             *reg
);
pcieRet_e pciev0_read_type1Bar_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBarReg_t                              *reg,
  int32_t                                    barNum
);
pcieRet_e pciev0_write_type1Bar_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBarReg_t                        *reg,
  int32_t                              barNum
);
pcieRet_e pciev0_read_type1Bar32bit_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBar32bitReg_t                         *reg,
  int32_t                                    barNum
);
pcieRet_e pciev0_write_type1Bar32bit_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBar32bitReg_t                   *reg,
  int32_t                              barNum
);
pcieRet_e pciev0_read_type1BusNum_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BusNumReg_t                      *reg
);
pcieRet_e pciev0_write_type1BusNum_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BusNumReg_t                *reg
);
pcieRet_e pciev0_read_type1SecStat_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1SecStatReg_t                     *reg
);
pcieRet_e pciev0_write_type1SecStat_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1SecStatReg_t               *reg
);
pcieRet_e pciev0_read_type1Memspace_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1MemspaceReg_t                    *reg
);
pcieRet_e pciev0_write_type1Memspace_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1MemspaceReg_t              *reg
);
pcieRet_e pciev0_read_prefMem_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefMemReg_t                          *reg
);
pcieRet_e pciev0_write_prefMem_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefMemReg_t                    *reg
);
pcieRet_e pciev0_read_prefBaseUpper_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefBaseUpperReg_t                    *reg
);
pcieRet_e pciev0_write_prefBaseUpper_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefBaseUpperReg_t              *reg
);
pcieRet_e pciev0_read_prefLimitUpper_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefLimitUpperReg_t                   *reg
);
pcieRet_e pciev0_write_prefLimitUpper_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefLimitUpperReg_t             *reg
);
pcieRet_e pciev0_read_type1IOSpace_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1IOSpaceReg_t                     *reg
);
pcieRet_e pciev0_write_type1IOSpace_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1IOSpaceReg_t               *reg
);
pcieRet_e pciev0_read_type1CapPtr_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1CapPtrReg_t                      *reg
);
pcieRet_e pciev0_write_type1CapPtr_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1CapPtrReg_t                *reg
);
pcieRet_e pciev0_read_type1ExpnsnRom_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t                   *reg
);
pcieRet_e pciev0_write_type1ExpnsnRom_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t             *reg
);
pcieRet_e pciev0_read_type1BridgeInt_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BridgeIntReg_t                   *reg
);
pcieRet_e pciev0_write_type1BridgeInt_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BridgeIntReg_t             *reg
);

/* power management capabilities*/
pcieRet_e pciev0_read_pmCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapReg_t                         *reg
);
pcieRet_e pciev0_write_pmCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapReg_t                   *reg
);
pcieRet_e pciev0_read_pmCapCtlStat_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapCtlStatReg_t                  *reg
);
pcieRet_e pciev0_write_pmCapCtlStat_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapCtlStatReg_t            *reg
);
/* MSI capabilities*/
pcieRet_e pciev0_read_msiCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiCapReg_t                        *reg
);
pcieRet_e pciev0_write_msiCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiCapReg_t                  *reg
);
pcieRet_e pciev0_read_msiLo32_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiLo32Reg_t                       *reg
);
pcieRet_e pciev0_write_msiLo32_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiLo32Reg_t                 *reg
);
pcieRet_e pciev0_read_msiUp32_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiUp32Reg_t                       *reg
);
pcieRet_e pciev0_write_msiUp32_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiUp32Reg_t                 *reg
);
pcieRet_e pciev0_read_msiData_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiDataReg_t                       *reg
);
pcieRet_e pciev0_write_msiData_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiDataReg_t                 *reg
);

/*Capabilities Registers*/
pcieRet_e pciev0_read_pciesCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePciesCapReg_t                       *reg 
);
pcieRet_e pciev0_write_pciesCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePciesCapReg_t                 *reg 
);
pcieRet_e pciev0_read_deviceCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDeviceCapReg_t                      *reg 
);
pcieRet_e pciev0_write_deviceCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDeviceCapReg_t                *reg 
);
pcieRet_e pciev0_read_devStatCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDevStatCtrlReg_t                   *reg
);
pcieRet_e pciev0_write_devStatCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDevStatCtrlReg_t             *reg
);
pcieRet_e pciev0_read_linkCap_reg
(      
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCapReg_t                        *reg 
);

pcieRet_e pciev0_write_linkCap_reg
(      
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCapReg_t                  *reg 
);
pcieRet_e pciev0_read_linkStatCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkStatCtrlReg_t                   *reg 
);
pcieRet_e pciev0_write_linkStatCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkStatCtrlReg_t             *reg 
);
pcieRet_e pciev0_read_slotCap_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotCapReg_t                           *reg 
);
pcieRet_e pciev0_write_slotCap_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotCapReg_t                     *reg 
);
pcieRet_e pciev0_read_slotStatCtrl_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t                      *reg 
);
pcieRet_e pciev0_write_slotStatCtrl_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t                *reg 
);
pcieRet_e pciev0_read_rootCtrlCap_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootCtrlCapReg_t                       *reg 
);
pcieRet_e pciev0_write_rootCtrlCap_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootCtrlCapReg_t                 *reg 
);
pcieRet_e pciev0_read_rootStatus_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootStatusReg_t                        *reg 
);
pcieRet_e pciev0_write_rootStatus_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootStatusReg_t                  *reg 
);
pcieRet_e pciev0_read_devCap2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevCap2Reg_t                        *reg 
);
pcieRet_e pciev0_write_devCap2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevCap2Reg_t                  *reg 
);
pcieRet_e pciev0_read_devStatCtrl2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevStatCtrl2Reg_t                   *reg 
);
pcieRet_e pciev0_write_devStatCtrl2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevStatCtrl2Reg_t             *reg 
);
pcieRet_e pciev0_read_linkCtrl2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCtrl2Reg_t                      *reg 
);
pcieRet_e pciev0_write_linkCtrl2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCtrl2Reg_t                *reg 
);

/*Capabilities Extended Registers*/
pcieRet_e pciev0_read_extCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieExtCapReg_t                        *reg
);
pcieRet_e pciev0_read_uncErr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrReg_t                        *reg
);
pcieRet_e pciev0_write_uncErr_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrReg_t                  *reg
);
pcieRet_e pciev0_read_uncErrMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrMaskReg_t                    *reg
);
pcieRet_e pciev0_write_uncErrMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrMaskReg_t              *reg
);
pcieRet_e pciev0_read_uncErrSvrty_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrSvrtyReg_t                   *reg
);
pcieRet_e pciev0_write_uncErrSvrty_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieUncErrSvrtyReg_t             *reg
);
pcieRet_e pciev0_read_corErr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieCorErrReg_t                        *reg
);
pcieRet_e pciev0_write_corErr_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieCorErrReg_t                  *reg
);
pcieRet_e pciev0_read_corErrMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieCorErrMaskReg_t                    *reg
);
pcieRet_e pciev0_write_corErrMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieCorErrMaskReg_t              *reg
);
pcieRet_e pciev0_read_accr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAccrReg_t                          *reg
);
pcieRet_e pciev0_write_accr_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAccrReg_t                    *reg
);
pcieRet_e pciev0_read_hdrLog_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieHdrLogReg_t                        *reg,
  int32_t                                 regNum
);
pcieRet_e pciev0_read_rootErrCmd_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieRootErrCmdReg_t                    *reg
);
pcieRet_e pciev0_write_rootErrCmd_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieRootErrCmdReg_t              *reg
);
pcieRet_e pciev0_read_rootErrSt_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieRootErrStReg_t                     *reg
);
pcieRet_e pciev0_write_rootErrSt_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieRootErrStReg_t               *reg
);
pcieRet_e pciev0_read_errSrcID_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieErrSrcIDReg_t                      *reg
);

/*Port Logic Registers*/
pcieRet_e pciev0_read_plAckTimer_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlAckTimerReg_t                    *reg
);
pcieRet_e pciev0_write_plAckTimer_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlAckTimerReg_t              *reg
);
pcieRet_e pciev0_read_plOMsg_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlOMsgReg_t                        *reg
);
pcieRet_e pciev0_write_plOMsg_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlOMsgReg_t                  *reg
);
pcieRet_e pciev0_read_plForceLink_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlForceLinkReg_t                   *reg
);
pcieRet_e pciev0_write_plForceLink_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePlForceLinkReg_t             *reg
);
pcieRet_e pciev0_read_ackFreq_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAckFreqReg_t                       *reg
);
pcieRet_e pciev0_write_ackFreq_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAckFreqReg_t                 *reg
);
pcieRet_e pciev0_read_lnkCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLnkCtrlReg_t                       *reg
);
pcieRet_e pciev0_write_lnkCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLnkCtrlReg_t                 *reg
);
pcieRet_e pciev0_read_laneSkew_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLaneSkewReg_t                      *reg
);
pcieRet_e pciev0_write_laneSkew_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLaneSkewReg_t                *reg
);
pcieRet_e pciev0_read_symNum_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieSymNumReg_t                        *reg
);
pcieRet_e pciev0_write_symNum_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieSymNumReg_t                  *reg
);
pcieRet_e pciev0_read_symTimerFltMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieSymTimerFltMaskReg_t               *reg
);
pcieRet_e pciev0_write_symTimerFltMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieSymTimerFltMaskReg_t         *reg
);
pcieRet_e pciev0_read_fltMask2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieFltMask2Reg_t                      *reg
);
pcieRet_e pciev0_write_fltMask2_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieFltMask2Reg_t                *reg
);
pcieRet_e pciev0_read_debug0_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDebug0Reg_t                        *reg
);
pcieRet_e pciev0_read_debug1_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDebug1Reg_t                         *reg 
);
pcieRet_e pciev0_read_gen2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieGen2Reg_t                          *reg
);
pcieRet_e pciev0_write_gen2_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieGen2Reg_t                    *reg
);

#ifdef __cplusplus
}
#endif

#endif  /* _PCIEV0LOC_H */

/* Nothing past this point */

