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

/*
 *  File Name: pciev1_ep.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/


#define PCIE_EP_EXPROM_BAR_BASE_FULL_MASK (CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_MASK | CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_MASK)
#define PCIE_EP_EXPROM_BAR_BASE_FULL_SHIFT (CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_SHIFT)


/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev1_read_bist_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBistReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BIST_HEAD_LAT_CACH;

  pcie_getbits(val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_CACH_LN_SZE,  swReg->cacheLnSize);
  pcie_getbits(val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_MSTR_LAT_TIM, swReg->latTmr);
  pcie_getbits(val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_HEAD_TYP,     swReg->hdrType);
  pcie_getbits(val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_MFD,          swReg->mulfunDev);
  pcie_getbits(val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_BIST,         swReg->bist);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->bistCap   = 0;
  swReg->startBist = 0;
  swReg->compCode  = 0;

  return pcie_RET_OK;
} /* pciev1_read_bist_reg */

/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev1_write_bist_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBistReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_getbits(new_val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_CACH_LN_SZE,  swReg->cacheLnSize);
  pcie_getbits(new_val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_MSTR_LAT_TIM, swReg->latTmr);
  pcie_getbits(new_val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_HEAD_TYP,     swReg->hdrType);
  pcie_getbits(new_val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_MFD,          swReg->mulfunDev);
  pcie_getbits(new_val, CSL_EPCFGDBICS_BIST_HEAD_LAT_CACH_BIST,         swReg->bist);

  baseAddr->BIST_HEAD_LAT_CACH = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_bist_reg */

/*****************************************************************************
 * Read and split up the BAR register
 ****************************************************************************/
pcieRet_e pciev1_read_type0Bar_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t val = swReg->raw = baseAddr->BAR[barNum];

  pcie_getbits(val, CSL_EPCFGDBICS_BAR_SPACE_DECODER, swReg->memSpace);
  pcie_getbits(val, CSL_EPCFGDBICS_BAR_AS,            swReg->type);
  pcie_getbits(val, CSL_EPCFGDBICS_BAR_PREFETCHABLE,  swReg->prefetch);
  pcie_getbits(val, PCIE_EP_BAR_BASE_FULL,            swReg->base);

  return pcie_RET_OK;
} /* pciev1_read_type0Bar_reg */

/*****************************************************************************
 * Combine and write the BAR register
 ****************************************************************************/
pcieRet_e pciev1_write_type0Bar_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t new_val = swReg->raw;

  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_BAR_SPACE_DECODER, swReg->memSpace);
  pcie_setbits(new_val, CSL_EPCFGDBICS_BAR_AS,            swReg->type);
  pcie_setbits(new_val, CSL_EPCFGDBICS_BAR_PREFETCHABLE,  swReg->prefetch);
  pcie_setbits(new_val, PCIE_EP_BAR_BASE_FULL,            swReg->base);

  baseAddr->BAR[barNum] = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_type0Bar_reg */


/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev1_read_type0Bar32bit_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  swReg->reg32 = swReg->raw = baseAddr->BAR[barNum];
  return pcie_RET_OK;
} /* pciev1_read_type0Bar32bit_reg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev1_write_type0Bar32bit_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  baseAddr->BAR[barNum] = swReg->raw = swReg->reg32;
  return pcie_RET_OK;
} /* pciev1_write_type0Bar32bit_reg */

/*****************************************************************************
 * Read and split up the Cardbus CIS Pointer register
 ****************************************************************************/
pcieRet_e pciev1_read_cardbusCisPointer_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieCardbusCisPointerReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CARDBUS_CIS_POINTER;

  pcie_getbits(val, CSL_EPCFGDBICS_CARDBUS_CIS_POINTER_CARDBUS_CIS_PTR_N, swReg->cisPointer);

  return pcie_RET_OK;
} /* pciev1_read_cardbusCisPointer_reg */

/*****************************************************************************
 * Combine and write the Cardbus CIS Pointer register
 ****************************************************************************/
pcieRet_e pciev1_write_cardbusCisPointer_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieCardbusCisPointerReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_CARDBUS_CIS_POINTER_CARDBUS_CIS_PTR_N, swReg->cisPointer);

  baseAddr->CARDBUS_CIS_POINTER = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_epcfgdbicsCardbusCisPointer_reg */

/*****************************************************************************
 * Read and split up the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev1_read_subId_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieSubIdReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SUBID_SUBVENDORID;

  pcie_getbits(val, CSL_EPCFGDBICS_SUBID_SUBVENDORID_SUBSYS_VENDOR_ID_N, swReg->subVndId);
  pcie_getbits(val, CSL_EPCFGDBICS_SUBID_SUBVENDORID_SUBSYS_DEV_ID_N,    swReg->subId);

  return pcie_RET_OK;
} /* pciev1_read_subId_reg */

/*****************************************************************************
 * Combine and write the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev1_write_subId_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieSubIdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_SUBID_SUBVENDORID_SUBSYS_VENDOR_ID_N, swReg->subVndId);
  pcie_setbits(new_val, CSL_EPCFGDBICS_SUBID_SUBVENDORID_SUBSYS_DEV_ID_N,    swReg->subId);

  baseAddr->SUBID_SUBVENDORID = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_subId_reg */

/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev1_read_expRom_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieExpRomReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->EXPANSION_ROM_BAR;

  pcie_getbits(val, CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_EN, swReg->enable);
  pcie_getbits(val, PCIE_EP_EXPROM_BAR_BASE_FULL,              swReg->expRomAddr);

  return pcie_RET_OK;
} /* pciev1_read_expRom_reg */

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev1_write_expRom_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieExpRomReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  uint32_t origReg = baseAddr->EXPANSION_ROM_BAR;
  uint32_t origRoBase;
  uint32_t newRoBase;
  pcieRet_e ret_val;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_EN, swReg->enable);
  pcie_setbits(new_val, PCIE_EP_EXPROM_BAR_BASE_FULL,              swReg->expRomAddr);

  /* Extract origin read only bits */
  pcie_getbits(origReg, CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO, origRoBase);
  /* Extract new read only bits */
  pcie_getbits(new_val, CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO, newRoBase);
  /* Puke if user tried to change RO bits */
  if (origRoBase != newRoBase)
  {
    ret_val = pcie_RET_RO_CHANGED;
  }
  else
  {
    baseAddr->EXPANSION_ROM_BAR = swReg->raw = new_val;
    ret_val = pcie_range_check_return; 
  }
  return ret_val;
} /* pciev1_write_expRom_reg */

/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev1_read_capPtr_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieCapPtrReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CAPPTR;

  pcie_getbits(val, CSL_EPCFGDBICS_CAPPTR_CAPTR, swReg->ptr);

  return pcie_RET_OK;
} /* pciev1_read_capPtr_reg */

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev1_write_capPtr_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieCapPtrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_CAPPTR_CAPTR, swReg->ptr);

  baseAddr->CAPPTR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_capPtr_reg */

/*****************************************************************************
 * Read and split up the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev1_read_intPin_reg
(
  const CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieIntPinReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->INTERRUPT;

  pcie_getbits(val, CSL_EPCFGDBICS_INTERRUPT_INT_LIN, swReg->intLine);
  pcie_getbits(val, CSL_EPCFGDBICS_INTERRUPT_INT_PIN, swReg->intPin);

  return pcie_RET_OK;
} /* pciev1_read_intPin_reg */

/*****************************************************************************
 * Combine and write the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev1_write_intPin_reg
(
  CSL_EpCfgDbIcsRegs *baseAddr, 
  pcieIntPinReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_INTERRUPT_INT_LIN, swReg->intLine);
  pcie_setbits(new_val, CSL_EPCFGDBICS_INTERRUPT_INT_PIN, swReg->intPin);

  baseAddr->INTERRUPT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_intPin_reg */

/* power management capabilities*/
/*****************************************************************************
 * Read and split up the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev1_read_pmCap_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PM_CAP;

  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_CAP_ID,    swReg->pmCapID);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_PM_NX_PTR, swReg->pmNextPtr);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_PMC_VER,   swReg->pmeSpecVer);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_PME_CLK,   swReg->pmeClk);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_DSI,       swReg->dsiN);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_AUX_CUR,   swReg->auxCurrN);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_D1_SP,     swReg->d1SuppN);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_D2_SP,     swReg->d2SuppN);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CAP_PME_SP,    swReg->pmeSuppN);

  return pcie_RET_OK;
} /* pciev1_read_pmCap_reg */

/*****************************************************************************
 * Combine and write the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev1_write_pmCap_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_CAP_ID,    swReg->pmCapID);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_PM_NX_PTR, swReg->pmNextPtr);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_PMC_VER,   swReg->pmeSpecVer);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_PME_CLK,   swReg->pmeClk);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_DSI,       swReg->dsiN);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_AUX_CUR,   swReg->auxCurrN);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_D1_SP,     swReg->d1SuppN);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_D2_SP,     swReg->d2SuppN);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CAP_PME_SP,    swReg->pmeSuppN);

  baseAddr->PM_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_pmCap_reg */

/*****************************************************************************
 * Read and split up the Power Management Control and Status register
 ****************************************************************************/
pcieRet_e pciev1_read_pmCapCtlStat_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pciePMCapCtlStatReg_t     *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PM_CSR;

  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_PM_STATE,   swReg->pwrState);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_NSR,        swReg->noSoftRst);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_PME_EN,     swReg->pmeEn);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_DATA_SEL,   swReg->dataSelect);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_DATA_SCALE, swReg->dataScale);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_PME_STATUS, swReg->pmeStatus);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_B2B3_SP,    swReg->b2b3Support);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_BP_CCE,     swReg->clkCtrlEn);
  pcie_getbits(val, CSL_EPCFGDBICS_PM_CSR_DATA1,      swReg->dataReg);

  return pcie_RET_OK;
} /* pciev1_read_pmCapCtlStat_reg */

/*****************************************************************************
 * Combine and write the Power Management Control and Status register
 ****************************************************************************/
pcieRet_e pciev1_write_pmCapCtlStat_reg
(
  CSL_EpCfgDbIcsRegs    *baseAddr,  
  pciePMCapCtlStatReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_PM_STATE,   swReg->pwrState);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_NSR,        swReg->noSoftRst);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_PME_EN,     swReg->pmeEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_DATA_SEL,   swReg->dataSelect);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_DATA_SCALE, swReg->dataScale);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_PME_STATUS, swReg->pmeStatus);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_B2B3_SP,    swReg->b2b3Support);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_BP_CCE,     swReg->clkCtrlEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_PM_CSR_DATA1,      swReg->dataReg);

  baseAddr->PM_CSR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_pmCapCtlStat_reg */

/* MSI capabilities*/
/*****************************************************************************
 * Read and split up the Message Signaled Interrupt Capability register
 ****************************************************************************/
pcieRet_e pciev1_read_msiCap_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiCapReg_t           *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP;

  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_CAP_ID,     swReg->capId);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_MSI_NX_PTR, swReg->nextCap);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_MSI_EN,     swReg->msiEn);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_MMC,        swReg->multMsgCap);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_MME,        swReg->multMsgEn);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_MSI_64_EN,  swReg->en64bit);
  pcie_getbits(val, CSL_EPCFGDBICS_MSI_CAP_PVM_EN,     swReg->pvmEn);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->extDataCap = 0u;
  swReg->extDataEn  = 0u;

  return pcie_RET_OK;
} /* pciev1_read_msiCap_reg */

/*****************************************************************************
 * Combine and write the Message Signaled Interrupt Capability register
 ****************************************************************************/
pcieRet_e pciev1_write_msiCap_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiCapReg_t     *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_CAP_ID,     swReg->capId);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_MSI_NX_PTR, swReg->nextCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_MSI_EN,     swReg->msiEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_MMC,        swReg->multMsgCap);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_MME,        swReg->multMsgEn);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_MSI_64_EN,  swReg->en64bit);
  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_CAP_PVM_EN,     swReg->pvmEn);

  baseAddr->MSI_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_msiCap_reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
pcieRet_e pciev1_read_msiLo32_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiLo32Reg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_ADDR_L32;

  pcie_getbits(val, CSL_EPCFGDBICS_MSI_ADDR_L32_ADDR, swReg->addr);

  return pcie_RET_OK;
} /* pciev1_read_epcfgdbicsMsiAddrL32_reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
pcieRet_e pciev1_write_msiLo32_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiLo32Reg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_ADDR_L32_ADDR, swReg->addr);

  baseAddr->MSI_ADDR_L32 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_epcfgdbicsMsiAddrL32_reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
pcieRet_e pciev1_read_msiUp32_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiUp32Reg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_ADDR_U32;

  pcie_getbits(val, CSL_EPCFGDBICS_MSI_ADDR_U32_ADDR, swReg->addr);

  return pcie_RET_OK;
} /* pciev1_read_epcfgdbicsMsiAddrU32_reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
pcieRet_e pciev1_write_msiUp32_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiUp32Reg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_ADDR_U32_ADDR, swReg->addr);

  baseAddr->MSI_ADDR_U32 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_epcfgdbicsMsiAddrU32_reg */

/*****************************************************************************
 * Read and split up the Data of MSI write TLP req register
 ****************************************************************************/
pcieRet_e pciev1_read_msiData_reg
(
  const CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiDataReg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_DATA;

  pcie_getbits(val, CSL_EPCFGDBICS_MSI_DATA_DATA, swReg->data);

  return pcie_RET_OK;
} /* pciev1_read_epcfgdbicsMsiData_reg */

/*****************************************************************************
 * Combine and write the Data of MSI write TLP req register
 ****************************************************************************/
pcieRet_e pciev1_write_msiData_reg
(
  CSL_EpCfgDbIcsRegs  *baseAddr,  
  pcieMsiDataReg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_EPCFGDBICS_MSI_DATA_DATA, swReg->data);

  baseAddr->MSI_DATA = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_epcfgdbicsMsiData_reg */

/* Nothing past this point */

