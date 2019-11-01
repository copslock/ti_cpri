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
 *  File Name: pciev2_rc.c
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


#define PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL_MASK (CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_MASK | CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_MASK)
#define PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL_SHIFT (CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_SHIFT)


/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev2_read_type1BistHeader_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieType1BistHeaderReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE,      swReg->cacheLnSize);
  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE,          swReg->hdrType);
  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC,           swReg->mulFunDev);
  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST,                 swReg->bist);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->bistCap   = 0u;
  swReg->startBist = 0u;
  swReg->compCode  = 0u;

  return pcie_RET_OK;
} /* pciev2_read_bist_reg */


/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev2_write_type1BistHeader_reg
(
  CSL_pcie_rc_coreRegs *baseAddr,
  pcieType1BistHeaderReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_getbits(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_CACHE_LINE_SIZE,      swReg->cacheLnSize);
  pcie_getbits(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_LATENCY_MASTER_TIMER, swReg->latTmr);
  pcie_getbits(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_HEADER_TYPE,          swReg->hdrType);
  pcie_getbits(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_MULTI_FUNC,           swReg->mulFunDev);
  pcie_getbits(new_val, CSL_PCIE_RC_CORE_TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG_BIST,                 swReg->bist);

  baseAddr->TYPE1_BIST_HDR_TYPE_LAT_CACHE_LINE_SIZE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_bist_reg */


/*****************************************************************************
 * Read and split up the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev2_read_type1BusNum_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BusNumReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS,      swReg->priBusNum);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS,       swReg->secBusNum);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS,       swReg->subBusNum);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER, swReg->secLatTmr);

  return pcie_RET_OK;
} /* pciev2_read_type1BusNum_reg */


/*****************************************************************************
 * Combine and write the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev2_write_type1BusNum_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BusNumReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_PRIM_BUS,      swReg->priBusNum);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_BUS,       swReg->secBusNum);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SUB_BUS,       swReg->subBusNum);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG_SEC_LAT_TIMER, swReg->secLatTmr);

  baseAddr->SEC_LAT_TIMER_SUB_BUS_SEC_BUS_PRI_BUS_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1BusNum_reg */


/*****************************************************************************
 * Read and split up the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev2_read_type1SecStat_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1SecStatReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SEC_STAT_IO_LIMIT_IO_BASE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE,               swReg->IOBaseAddr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE,                 swReg->IOBase);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8,          swReg->IOLimitAddr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT,                swReg->IOLimit);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE,           swReg->mstDPErr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT,  swReg->txTgtAbort);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT, swReg->rxTgtAbort);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT, swReg->rxMstAbort);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR,   swReg->rxSysError);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE,            swReg->dtctPError);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->devselTiming = 0u;
  swReg->c66mhzCapa   = 0u;
  swReg->fastB2bCap   = 0u;

  return pcie_RET_OK;
} /* pciev2_read_type1SecStat_reg */


/*****************************************************************************
 * Combine and write the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev2_write_type1SecStat_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1SecStatReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE,               swReg->IOBaseAddr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_BASE,                 swReg->IOBase);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_DECODE_BIT8,          swReg->IOLimitAddr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_IO_LIMIT,                swReg->IOLimit);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_MDPE,           swReg->mstDPErr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_SIG_TRGT_ABRT,  swReg->txTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_TRGT_ABRT, swReg->rxTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_MSTR_ABRT, swReg->rxMstAbort);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_RCVD_SYS_ERR,   swReg->rxSysError);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SEC_STAT_IO_LIMIT_IO_BASE_REG_SEC_STAT_DPE,            swReg->dtctPError);

  baseAddr->SEC_STAT_IO_LIMIT_IO_BASE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1SecStat_reg */


/*****************************************************************************
 * Read and split up the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev2_read_type1Memspace_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1MemspaceReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MEM_LIMIT_MEM_BASE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE,  swReg->base);
  pcie_getbits(val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT, swReg->limit);

  return pcie_RET_OK;
} /* pciev2_read_type1Memspace_reg */


/*****************************************************************************
 * Combine and write the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev2_write_type1Memspace_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1MemspaceReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_BASE,  swReg->base);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_MEM_LIMIT_MEM_BASE_REG_MEM_LIMIT, swReg->limit);

  baseAddr->MEM_LIMIT_MEM_BASE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1Memspace_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev2_read_prefMem_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefMemReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PREF_MEM_LIMIT_PREF_MEM_BASE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE,       swReg->baseAddr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE,         swReg->base);
  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE, swReg->limitAddr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT,        swReg->limit);

  return pcie_RET_OK;
} /* pciev2_read_prefMem_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev2_write_prefMem_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefMemReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_DECODE,       swReg->baseAddr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_BASE,         swReg->base);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT_DECODE, swReg->limitAddr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_MEM_LIMIT_PREF_MEM_BASE_REG_PREF_MEM_LIMIT,        swReg->limit);

  baseAddr->PREF_MEM_LIMIT_PREF_MEM_BASE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_prefMem_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev2_read_prefBaseUpper_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefBaseUpperReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PREF_BASE_UPPER_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER, swReg->base);

  return pcie_RET_OK;
} /* pciev2_read_prefBaseUp_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev2_write_prefBaseUpper_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefBaseUpperReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_BASE_UPPER_REG_PREF_MEM_BASE_UPPER, swReg->base);

  baseAddr->PREF_BASE_UPPER_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_prefBaseUp_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit Upper register
 ****************************************************************************/
pcieRet_e pciev2_read_prefLimitUpper_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefLimitUpperReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PREF_LIMIT_UPPER_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER, swReg->limit);

  return pcie_RET_OK;
} /* pciev2_read_prefLimitUp_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit Upper  register
 ****************************************************************************/
pcieRet_e pciev2_write_prefLimitUpper_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pciePrefLimitUpperReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_PREF_LIMIT_UPPER_REG_PREF_MEM_LIMIT_UPPER, swReg->limit);

  baseAddr->PREF_LIMIT_UPPER_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_prefLimitUp_reg */


/*****************************************************************************
 * Read and split up the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev2_read_type1IOSpace_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1IOSpaceReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IO_LIMIT_UPPER_IO_BASE_UPPER_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER,  swReg->IOBase);
  pcie_getbits(val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER, swReg->IOLimit);

  return pcie_RET_OK;
} /* pciev2_read_type1IOSpace_reg */


/*****************************************************************************
 * Combine and write the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev2_write_type1IOSpace_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1IOSpaceReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_BASE_UPPER,  swReg->IOBase);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_IO_LIMIT_UPPER_IO_BASE_UPPER_REG_IO_LIMIT_UPPER, swReg->IOLimit);

  baseAddr->IO_LIMIT_UPPER_IO_BASE_UPPER_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1IOSpace_reg */


/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev2_read_type1CapPtr_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1CapPtrReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_CAP_PTR_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER, swReg->capPtr);

  return pcie_RET_OK;
} /* pciev2_read_type1CapPtr_reg */


/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev2_write_type1CapPtr_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1CapPtrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_TYPE1_CAP_PTR_REG_CAP_POINTER, swReg->capPtr);

  baseAddr->TYPE1_CAP_PTR_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1CapPtr_reg */


/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev2_read_type1ExpnsnRom_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1ExpnsnRomReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TYPE1_EXP_ROM_BASE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE,       swReg->expRomEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomBaseAddr);

  return pcie_RET_OK;
} /* pciev2_read_type1ExpnsnRom_reg */


/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev2_write_type1ExpnsnRom_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1ExpnsnRomReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_ROM_BAR_ENABLE,       swReg->expRomEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_TYPE1_EXP_ROM_BASE_REG_EXP_ROM_BASE_ADDRESS, swReg->expRomBaseAddr);

  baseAddr->TYPE1_EXP_ROM_BASE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1ExpnsnRom_reg */


/*****************************************************************************
 * Read and split up the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev2_read_type1BridgeInt_reg
(
  const CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BridgeIntReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BRIDGE_CTRL_INT_PIN_INT_LINE_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE,        swReg->intLine);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN,         swReg->intPin);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE,            swReg->pErrRespEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN,         swReg->serrEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN,          swReg->isaEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN,          swReg->vgaEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC,     swReg->vgaDecode);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE, swReg->mstAbortMode);
  pcie_getbits(val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR,             swReg->secBusRst);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->b2bEn        = 0u;
  swReg->priTimer     = 0u;
  swReg->secTimer     = 0u;
  swReg->timerStatus  = 0u;
  swReg->serrEnStatus = 0u;

  return pcie_RET_OK;
} /* pciev2_read_type1BridgeInt_reg */


/*****************************************************************************
 * Combine and write the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev2_write_type1BridgeInt_reg
(
  CSL_pcie_rc_coreRegs  *baseAddr,
  pcieType1BridgeIntReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_LINE,        swReg->intLine);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_INT_PIN,         swReg->intPin);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_PERE,            swReg->pErrRespEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SERR_EN,         swReg->serrEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_ISA_EN,          swReg->isaEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_EN,          swReg->vgaEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_VGA_16B_DEC,     swReg->vgaDecode);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_MSTR_ABORT_MODE, swReg->mstAbortMode);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_BRIDGE_CTRL_INT_PIN_INT_LINE_REG_SBR,             swReg->secBusRst);

  baseAddr->BRIDGE_CTRL_INT_PIN_INT_LINE_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_type1BridgeInt_reg */


/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS **********************
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Slot Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_read_slotCap_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotCapReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SLOT_CAPABILITIES_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON, swReg->attnButton);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER,           swReg->pwrCtl);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR,                 swReg->mrlSensor);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR,        swReg->attnInd);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR,            swReg->pwrInd);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE,          swReg->hpSurprise);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE,           swReg->hpCap);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE,     swReg->pwrLmtValue);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE,     swReg->pwrLmtScale);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK,      swReg->emlPresent);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT,         swReg->cmdCompSupp);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM,               swReg->slotNum);

  return pcie_RET_OK;
} /* pciev2_read_slotCap_reg */


/*****************************************************************************
 * Combine and write the Slot Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_write_slotCap_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotCapReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR_BUTTON, swReg->attnButton);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_CONTROLLER,           swReg->pwrCtl);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_MRL_SENSOR,                 swReg->mrlSensor);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ATTENTION_INDICATOR,        swReg->attnInd);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_POWER_INDICATOR,            swReg->pwrInd);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_SURPRISE,          swReg->hpSurprise);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_HOT_PLUG_CAPABLE,           swReg->hpCap);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_VALUE,     swReg->pwrLmtValue);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_SLOT_POWER_LIMIT_SCALE,     swReg->pwrLmtScale);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_ELECTROMECH_INTERLOCK,      swReg->emlPresent);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_NO_CMD_CPL_SUPPORT,         swReg->cmdCompSupp);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CAPABILITIES_REG_PCIE_CAP_PHY_SLOT_NUM,               swReg->slotNum);

  baseAddr->SLOT_CAPABILITIES_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_slotCap_reg */


/*****************************************************************************
 * Read and split up the Slot Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_read_slotStatCtrl_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotStatCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SLOT_CONTROL_SLOT_STATUS;

  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN,  swReg->attnButtEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN,      swReg->pwrFltDetEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN,        swReg->mrlChgEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN,    swReg->prsDetChgEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN,               swReg->cmdCmpIntEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN,              swReg->hpIntEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL,     swReg->attnIndCtl);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL,         swReg->pmIndCtl);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL,        swReg->pmCtl);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL,   swReg->emLockCtl);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN,         swReg->dllChgEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED,     swReg->attnPressed);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED,         swReg->pwrFault);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED,           swReg->mrlChange);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED,    swReg->presenceChg);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD,                     swReg->cmdComplete);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE,             swReg->mrlState);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE,        swReg->presenceDet);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS, swReg->emLock);
  pcie_getbits(val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED,            swReg->dllState);

  return pcie_RET_OK;
} /* pciev2_read_slotStatCtrl_reg */


/*****************************************************************************
 * Combine and write the Slot Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_write_slotStatCtrl_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieSlotStatCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED_EN,  swReg->attnButtEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED_EN,      swReg->pwrFltDetEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED_EN,        swReg->mrlChgEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_CHANGE_EN,    swReg->prsDetChgEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPL_INT_EN,               swReg->cmdCmpIntEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_HOT_PLUG_INT_EN,              swReg->hpIntEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_INDICATOR_CTRL,     swReg->attnIndCtl);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_INDICATOR_CTRL,         swReg->pmIndCtl);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_CONTROLLER_CTRL,        swReg->pmCtl);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_CTRL,   swReg->emLockCtl);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED_EN,         swReg->dllChgEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ATTENTION_BUTTON_PRESSED,     swReg->attnPressed);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_POWER_FAULT_DETECTED,         swReg->pwrFault);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_CHANGED,           swReg->mrlChange);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECTED_CHANGED,    swReg->presenceChg);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_CMD_CPLD,                     swReg->cmdComplete);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_MRL_SENSOR_STATE,             swReg->mrlState);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_PRESENCE_DETECT_STATE,        swReg->presenceDet);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_ELECTROMECH_INTERLOCK_STATUS, swReg->emLock);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_SLOT_CONTROL_SLOT_STATUS_PCIE_CAP_DLL_STATE_CHANGED,            swReg->dllState);
  baseAddr->SLOT_CONTROL_SLOT_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_slotStatCtrl_reg */


/*****************************************************************************
 * Read and split up the Root Control and Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_read_rootCtrlCap_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootCtrlCapReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_CONTROL_ROOT_CAPABILITIES_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN,      swReg->serrEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN, swReg->serrNFatalErr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN,     swReg->serrFatalErr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN,                  swReg->pmeIntEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN,        swReg->crsSwEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY,           swReg->crsSw);

  return pcie_RET_OK;
} /* pciev2_read_rootCtrlCap_reg */


/*****************************************************************************
 * Combine and write the Root Control and Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_write_rootCtrlCap_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootCtrlCapReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN,      swReg->serrEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN, swReg->serrNFatalErr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN,     swReg->serrFatalErr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_PME_INT_EN,                  swReg->pmeIntEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY_EN,        swReg->crsSwEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_CONTROL_ROOT_CAPABILITIES_REG_PCIE_CAP_CRS_SW_VISIBILITY,           swReg->crsSw);

  baseAddr->ROOT_CONTROL_ROOT_CAPABILITIES_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_rootCtrlCap_reg */


/*****************************************************************************
 * Read and split up the Root Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_read_rootStatus_reg
(
  const CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootStatusReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_STATUS_REG;

  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID,  swReg->pmeReqID);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS,  swReg->pmeStatus);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING, swReg->pmePend);

  return pcie_RET_OK;
} /* pciev2_read_rootStatus_reg */


/*****************************************************************************
 * Combine and write the Root Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_write_rootStatus_reg
(
  CSL_pcie_rc_coreRegs   *baseAddr,
  pcieRootStatusReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_REQ_ID,  swReg->pmeReqID);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_STATUS,  swReg->pmeStatus);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_STATUS_REG_PCIE_CAP_PME_PENDING, swReg->pmePend);

  baseAddr->ROOT_STATUS_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_rootStatus_reg */

/* Nothing past this point */

