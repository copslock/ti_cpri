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

/*
 *  File Name: pciev2_ep.c
 *
 *  Processing/configuration functions for the PCIe driver.
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v2/pcieloc.h>

/*****************************************************************************
 **********  External APIs **********************
 ****************************************************************************/


#define PCIE_EP_EXPROM_BAR_BASE_FULL_MASK (CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_MASK | CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_MASK)
#define PCIE_EP_EXPROM_BAR_BASE_FULL_SHIFT (CSL_EPCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_SHIFT)


/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev2_read_bist_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieBistReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE,      swReg->cacheLnSize);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_HEADER_TYPE,          swReg->hdrType);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_MULTI_FUNC,           swReg->mulfunDev);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_BIST,                 swReg->bist);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->bistCap   = 0;
  swReg->startBist = 0;
  swReg->compCode  = 0;

  return pcie_RET_OK;
} /* pciev2_read_bist_reg */

/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev2_write_bist_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieBistReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE,      swReg->cacheLnSize);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_HEADER_TYPE,          swReg->hdrType);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_MULTI_FUNC,           swReg->mulfunDev);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG_BIST,                 swReg->bist);

  baseAddr->BIST_HEADER_TYPE_LATENCY_CACHE_LINE_SIZE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_bist_reg */

/*****************************************************************************
 * Read and split up the BAR register
 ****************************************************************************/
pcieRet_e pciev2_read_type0Bar_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t val = swReg->raw = baseAddr->BAR_REG[barNum];

  pcie_getbits(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_MEM_IO,   swReg->memSpace);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_TYPE,     swReg->type);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_PREFETCH, swReg->prefetch);
  pcie_getbits(val, CSL_PCIE_EP_CORE_BAR_REG_BAR_START,    swReg->base);

  return pcie_RET_OK;
} /* pciev2_read_type0Bar_reg */

/*****************************************************************************
 * Combine and write the BAR register
 ****************************************************************************/
pcieRet_e pciev2_write_type0Bar_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t new_val = swReg->raw;

  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_MEM_IO,   swReg->memSpace);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_TYPE,     swReg->type);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_PREFETCH, swReg->prefetch);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_BAR_REG_BAR_START,    swReg->base);

  baseAddr->BAR_REG[barNum] = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type0Bar_reg */


/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev2_read_type0Bar32bit_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  swReg->reg32 = swReg->raw = baseAddr->BAR_REG[barNum];
  return pcie_RET_OK;
} /* pciev2_read_type0Bar32bit_reg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev2_write_type0Bar32bit_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  baseAddr->BAR_REG[barNum] = swReg->raw = swReg->reg32;
  return pcie_RET_OK;
} /* pciev2_write_type0Bar32bit_reg */

/*****************************************************************************
 * Read and split up the Cardbus CIS Pointer register
 ****************************************************************************/
pcieRet_e pciev2_read_cardbusCisPointer_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCardbusCisPointerReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CARDBUS_CIS_PTR_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CARDBUS_CIS_PTR_REG_CARDBUS_CIS_POINTER, swReg->cisPointer);

  return pcie_RET_OK;
} /* pciev2_read_cardbusCisPointer_reg */

/*****************************************************************************
 * Combine and write the Cardbus CIS Pointer register
 ****************************************************************************/
pcieRet_e pciev2_write_cardbusCisPointer_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCardbusCisPointerReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CARDBUS_CIS_PTR_REG_CARDBUS_CIS_POINTER, swReg->cisPointer);

  baseAddr->CARDBUS_CIS_PTR_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epcfgdbicsCardbusCisPointer_reg */

/*****************************************************************************
 * Read and split up the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev2_read_subId_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieSubIdReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_VENDOR_ID, swReg->subVndId);
  pcie_getbits(val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_DEV_ID,    swReg->subId);

  return pcie_RET_OK;
} /* pciev2_read_subId_reg */

/*****************************************************************************
 * Combine and write the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev2_write_subId_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieSubIdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_VENDOR_ID, swReg->subVndId);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG_SUBSYS_DEV_ID,    swReg->subId);

  baseAddr->SUBSYSTEM_ID_SUBSYSTEM_VENDOR_ID_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_subId_reg */

/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev2_read_expRom_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieExpRomReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->EXP_ROM_BASE_ADDR_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_ROM_BAR_ENABLE,       swReg->enable);
  pcie_getbits(val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomAddr);

  return pcie_RET_OK;
} /* pciev2_read_expRom_reg */

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev2_write_expRom_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieExpRomReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e ret_val;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_ROM_BAR_ENABLE,       swReg->enable);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_EXP_ROM_BASE_ADDR_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomAddr);

  baseAddr->EXP_ROM_BASE_ADDR_REG = swReg->raw = new_val;
  ret_val = pcie_range_check_return;

  return ret_val;
} /* pciev2_write_expRom_reg */

/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev2_read_capPtr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCapPtrReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PCI_CAP_PTR_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_CAP_PTR_REG_CAP_POINTER, swReg->ptr);

  return pcie_RET_OK;
} /* pciev2_read_capPtr_reg */

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev2_write_capPtr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCapPtrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_CAP_PTR_REG_CAP_POINTER, swReg->ptr);

  baseAddr->PCI_CAP_PTR_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_capPtr_reg */

/*****************************************************************************
 * Read and split up the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev2_read_intPin_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieIntPinReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_LINE, swReg->intLine);
  pcie_getbits(val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_PIN,  swReg->intPin);

  return pcie_RET_OK;
} /* pciev2_read_intPin_reg */

/*****************************************************************************
 * Combine and write the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev2_write_intPin_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieIntPinReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_LINE, swReg->intLine);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG_INT_PIN,  swReg->intPin);

  baseAddr->MAX_LATENCY_MIN_GRANT_INTERRUPT_PIN_INTERRUPT_LINE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_intPin_reg */

/* power management capabilities*/
/*****************************************************************************
 * Read and split up the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev2_read_pmCap_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CAP_ID_NXT_PTR_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID,       swReg->pmCapID);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER, swReg->pmNextPtr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER,     swReg->pmeSpecVer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_CLK,         swReg->pmeClk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_DSI,             swReg->dsiN);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR,        swReg->auxCurrN);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT,      swReg->d1SuppN);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT,      swReg->d2SuppN);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT,     swReg->pmeSuppN);

  return pcie_RET_OK;
} /* pciev2_read_pmCap_reg */

/*****************************************************************************
 * Combine and write the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev2_write_pmCap_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_CAP_ID,       swReg->pmCapID);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_NEXT_POINTER, swReg->pmNextPtr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PM_SPEC_VER,     swReg->pmeSpecVer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_CLK,         swReg->pmeClk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_DSI,             swReg->dsiN);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_AUX_CURR,        swReg->auxCurrN);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D1_SUPPORT,      swReg->d1SuppN);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_D2_SUPPORT,      swReg->d2SuppN);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CAP_ID_NXT_PTR_REG_PME_SUPPORT,     swReg->pmeSuppN);

  baseAddr->CAP_ID_NXT_PTR_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmCap_reg */

/*****************************************************************************
 * Read and split up the Power Management Control and Status register
 ****************************************************************************/
pcieRet_e pciev2_read_pmCapCtlStat_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pciePMCapCtlStatReg_t     *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CON_STATUS_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_POWER_STATE,        swReg->pwrState);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_NO_SOFT_RST,        swReg->noSoftRst);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_ENABLE,         swReg->pmeEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SELECT,        swReg->dataSelect);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SCALE,         swReg->dataScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_STATUS,         swReg->pmeStatus);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_B2_B3_SUPPORT,      swReg->b2b3Support);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN, swReg->clkCtrlEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO,  swReg->dataReg);

  return pcie_RET_OK;
} /* pciev2_read_pmCapCtlStat_reg */

/*****************************************************************************
 * Combine and write the Power Management Control and Status register
 ****************************************************************************/
pcieRet_e pciev2_write_pmCapCtlStat_reg
(
  CSL_pcie_ep_coreRegs    *baseAddr,
  pciePMCapCtlStatReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_POWER_STATE,        swReg->pwrState);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_NO_SOFT_RST,        swReg->noSoftRst);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_ENABLE,         swReg->pmeEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SELECT,        swReg->dataSelect);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_SCALE,         swReg->dataScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_PME_STATUS,         swReg->pmeStatus);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_B2_B3_SUPPORT,      swReg->b2b3Support);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_BUS_PWR_CLK_CON_EN, swReg->clkCtrlEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CON_STATUS_REG_DATA_REG_ADD_INFO,  swReg->dataReg);

  baseAddr->CON_STATUS_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmCapCtlStat_reg */


/* Nothing past this point */

