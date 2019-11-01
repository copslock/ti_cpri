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
 *  File Name: pciev1_rc.c
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


#define PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL_MASK (CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_MASK | CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_MASK)
#define PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL_SHIFT (CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO_SHIFT)


/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev1_read_type1BistHeader_reg
(
  const CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieType1BistHeaderReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BIST_HEAD_LAT_CACH;

  pcie_getbits(val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_CACH_LN_SZE,  swReg->cacheLnSize);
  pcie_getbits(val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_MSTR_LAT_TIM, swReg->latTmr);
  pcie_getbits(val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_HEAD_TYP,     swReg->hdrType);
  pcie_getbits(val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_MFD,          swReg->mulFunDev);
  pcie_getbits(val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_BIST,         swReg->bist);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->bistCap   = 0;
  swReg->startBist = 0;
  swReg->compCode  = 0;

  return pcie_RET_OK;
} /* pciev1_read_bist_reg */


/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev1_write_type1BistHeader_reg
(
  CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieType1BistHeaderReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_getbits(new_val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_CACH_LN_SZE,  swReg->cacheLnSize);
  pcie_getbits(new_val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_MSTR_LAT_TIM, swReg->latTmr);
  pcie_getbits(new_val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_HEAD_TYP,     swReg->hdrType);
  pcie_getbits(new_val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_MFD,          swReg->mulFunDev);
  pcie_getbits(new_val, CSL_RCCFGDBICS_BIST_HEAD_LAT_CACH_BIST,         swReg->bist);

  baseAddr->BIST_HEAD_LAT_CACH = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_bist_reg */


/*****************************************************************************
 * Read and split up the BAR register
 ****************************************************************************/
pcieRet_e pciev1_read_type1Bar_reg
(
  const CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t val = swReg->raw = baseAddr->BAR[barNum];

  pcie_getbits(val, CSL_RCCFGDBICS_BAR_SPACE_INDICATOR, swReg->memSpace);
  pcie_getbits(val, CSL_RCCFGDBICS_BAR_AS,              swReg->type);
  pcie_getbits(val, CSL_RCCFGDBICS_BAR_PREFETCHABLE,    swReg->prefetch);
  pcie_getbits(val, PCIE_RC_BAR_BASE_FULL,              swReg->base);

  return pcie_RET_OK;
} /* pciev1_read_type1Bar_reg */


/*****************************************************************************
 * Combine and write the BAR register
 ****************************************************************************/
pcieRet_e pciev1_write_type1Bar_reg
(
  CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieBarReg_t *swReg,
  int32_t barNum
)
{
  uint32_t new_val = swReg->raw;

  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_BAR_SPACE_INDICATOR, swReg->memSpace);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BAR_AS,              swReg->type);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BAR_PREFETCHABLE,    swReg->prefetch);
  pcie_setbits(new_val, PCIE_RC_BAR_BASE_FULL,              swReg->base);

  baseAddr->BAR[barNum] = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_type1Bar_reg */


/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev1_read_type1Bar32bit_reg
(
  const CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  swReg->reg32 = swReg->raw = baseAddr->BAR[barNum];
  return pcie_RET_OK;
} /* pciev1_read_type1Bar32bit_reg */


/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev1_write_type1Bar32bit_reg
(
  CSL_RcCfgDbIcsRegs *baseAddr,  
  pcieBar32bitReg_t *swReg,
  int32_t barNum
)
{
  baseAddr->BAR[barNum] = swReg->raw = swReg->reg32;
  return pcie_RET_OK;
} /* pciev1_write_type1Bar32bit_reg */


/*****************************************************************************
 * Read and split up the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev1_read_type1BusNum_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BusNumReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BUS_NUM_REG;

  pcie_getbits(val, CSL_RCCFGDBICS_BUS_NUM_REG_PRIM_BUS_NUM,   swReg->priBusNum);
  pcie_getbits(val, CSL_RCCFGDBICS_BUS_NUM_REG_SEC_BUS_NUM,    swReg->secBusNum);
  pcie_getbits(val, CSL_RCCFGDBICS_BUS_NUM_REG_SUBORD_BUS_NUM, swReg->subBusNum);
  pcie_getbits(val, CSL_RCCFGDBICS_BUS_NUM_REG_SEC_LAT_TIMER,  swReg->secLatTmr);

  return pcie_RET_OK;
} /* pciev1_read_type1BusNum_reg */


/*****************************************************************************
 * Combine and write the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev1_write_type1BusNum_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BusNumReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_BUS_NUM_REG_PRIM_BUS_NUM,   swReg->priBusNum);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BUS_NUM_REG_SEC_BUS_NUM,    swReg->secBusNum);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BUS_NUM_REG_SUBORD_BUS_NUM, swReg->subBusNum);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BUS_NUM_REG_SEC_LAT_TIMER,  swReg->secLatTmr);

  baseAddr->BUS_NUM_REG = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1BusNum_reg */


/*****************************************************************************
 * Read and split up the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev1_read_type1SecStat_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1SecStatReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IOBASE_LIMIT_SEC_STATUS;

  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IODECODE_32_0,      swReg->IOBaseAddr);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IO_SPACE_BASE,      swReg->IOBase);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IODECODE_32,        swReg->IOLimitAddr);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IO_SPACE_LIMIT,     swReg->IOLimit);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_C66MHZ_CAPA,        swReg->c66mhzCapa);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_FAST_B2B_CAP,       swReg->fastB2bCap);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_MSTR_DATA_PRTY_ERR, swReg->mstDPErr);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_DEVSEL_TIMING,      swReg->devselTiming);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_SGNLD_TRGT_ABORT,   swReg->txTgtAbort);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_TRGT_ABORT,    swReg->rxTgtAbort);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_MSTR_ABORT,    swReg->rxMstAbort);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_SYS_ERR,       swReg->rxSysError);
  pcie_getbits(val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_DET_PAR_ERR,        swReg->dtctPError);

  return pcie_RET_OK;
} /* pciev1_read_type1SecStat_reg */


/*****************************************************************************
 * Combine and write the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev1_write_type1SecStat_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1SecStatReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IODECODE_32_0,      swReg->IOBaseAddr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IO_SPACE_BASE,      swReg->IOBase);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IODECODE_32,        swReg->IOLimitAddr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_IO_SPACE_LIMIT,     swReg->IOLimit);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_C66MHZ_CAPA,        swReg->c66mhzCapa);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_FAST_B2B_CAP,       swReg->fastB2bCap);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_MSTR_DATA_PRTY_ERR, swReg->mstDPErr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_DEVSEL_TIMING,      swReg->devselTiming);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_SGNLD_TRGT_ABORT,   swReg->txTgtAbort);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_TRGT_ABORT,    swReg->rxTgtAbort);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_MSTR_ABORT,    swReg->rxMstAbort);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_RCVD_SYS_ERR,       swReg->rxSysError);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IOBASE_LIMIT_SEC_STATUS_DET_PAR_ERR,        swReg->dtctPError);

  baseAddr->IOBASE_LIMIT_SEC_STATUS = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1SecStat_reg */


/*****************************************************************************
 * Read and split up the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev1_read_type1Memspace_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1MemspaceReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MEM_BASE_LIMIT;

  pcie_getbits(val, CSL_RCCFGDBICS_MEM_BASE_LIMIT_MEM_BASE_ADDR,  swReg->base);
  pcie_getbits(val, CSL_RCCFGDBICS_MEM_BASE_LIMIT_MEM_LIMIT_ADDR, swReg->limit);

  return pcie_RET_OK;
} /* pciev1_read_type1Memspace_reg */


/*****************************************************************************
 * Combine and write the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev1_write_type1Memspace_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1MemspaceReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_MEM_BASE_LIMIT_MEM_BASE_ADDR,  swReg->base);
  pcie_setbits(new_val, CSL_RCCFGDBICS_MEM_BASE_LIMIT_MEM_LIMIT_ADDR, swReg->limit);

  baseAddr->MEM_BASE_LIMIT = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1Memspace_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev1_read_prefMem_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefMemReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PREF_MEM_BASE_LIMIT;

  pcie_getbits(val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_MEMDECODE_64_0,   swReg->baseAddr);
  pcie_getbits(val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_UPPPREF_MEM_ADDR, swReg->base);
  pcie_getbits(val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_MEMDECODE_64,     swReg->limitAddr);
  pcie_getbits(val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_PREF_MEM_ADDR,    swReg->limit);

  return pcie_RET_OK;
} /* pciev1_read_prefMem_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev1_write_prefMem_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefMemReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_MEMDECODE_64_0,   swReg->baseAddr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_UPPPREF_MEM_ADDR, swReg->base);
  pcie_setbits(new_val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_MEMDECODE_64,     swReg->limitAddr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_PREF_MEM_BASE_LIMIT_PREF_MEM_ADDR,    swReg->limit);

  baseAddr->PREF_MEM_BASE_LIMIT = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_prefMem_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev1_read_prefBaseUpper_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->UPPER_32BIT_PREF_BASEADDR;

  pcie_getbits(val, CSL_RCCFGDBICS_UPPER_32BIT_PREF_BASEADDR_ADDRUPP, swReg->base);

  return pcie_RET_OK;
} /* pciev1_read_prefBaseUp_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev1_write_prefBaseUpper_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_UPPER_32BIT_PREF_BASEADDR_ADDRUPP, swReg->base);

  baseAddr->UPPER_32BIT_PREF_BASEADDR = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_prefBaseUp_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit Upper register
 ****************************************************************************/
pcieRet_e pciev1_read_prefLimitUpper_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->UPPER_32BIT_PREF_LIMITADDR;

  pcie_getbits(val, CSL_RCCFGDBICS_UPPER_32BIT_PREF_LIMITADDR_ADDRUPP_LIMIT, swReg->limit);

  return pcie_RET_OK;
} /* pciev1_read_prefLimitUp_reg */


/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit Upper  register
 ****************************************************************************/
pcieRet_e pciev1_write_prefLimitUpper_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_UPPER_32BIT_PREF_LIMITADDR_ADDRUPP_LIMIT, swReg->limit);

  baseAddr->UPPER_32BIT_PREF_LIMITADDR = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_prefLimitUp_reg */


/*****************************************************************************
 * Read and split up the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev1_read_type1IOSpace_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IO_BASE_LIMIT;

  pcie_getbits(val, CSL_RCCFGDBICS_IO_BASE_LIMIT_UPP16_IOBASE,  swReg->IOBase);
  pcie_getbits(val, CSL_RCCFGDBICS_IO_BASE_LIMIT_UPP16_IOLIMIT, swReg->IOLimit);

  return pcie_RET_OK;
} /* pciev1_read_type1IOSpace_reg */


/*****************************************************************************
 * Combine and write the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev1_write_type1IOSpace_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_IO_BASE_LIMIT_UPP16_IOBASE,  swReg->IOBase);
  pcie_setbits(new_val, CSL_RCCFGDBICS_IO_BASE_LIMIT_UPP16_IOLIMIT, swReg->IOLimit);

  baseAddr->IO_BASE_LIMIT = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1IOSpace_reg */


/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev1_read_type1CapPtr_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1CapPtrReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CAPPTR;

  pcie_getbits(val, CSL_RCCFGDBICS_CAPPTR_CAPTR, swReg->capPtr);

  return pcie_RET_OK;
} /* pciev1_read_type1CapPtr_reg */


/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev1_write_type1CapPtr_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1CapPtrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_CAPPTR_CAPTR, swReg->capPtr);

  baseAddr->CAPPTR = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1CapPtr_reg */


/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev1_read_type1ExpnsnRom_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->EXPANSION_ROM_BAR;

  pcie_getbits(val, CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXP_ROM_EN, swReg->expRomEn);
  pcie_getbits(val, PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL,         swReg->expRomBaseAddr);

  return pcie_RET_OK;
} /* pciev1_read_type1ExpnsnRom_reg */


/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev1_write_type1ExpnsnRom_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  uint32_t origReg = baseAddr->EXPANSION_ROM_BAR;
  uint32_t origRoBase;
  uint32_t newRoBase;
  pcieRet_e ret_val;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXP_ROM_EN, swReg->expRomEn);
  pcie_setbits(new_val, PCIE_RC_EXPANSION_ROM_BAR_BASE_FULL,         swReg->expRomBaseAddr);

  /* Extract origin read only bits */
  pcie_getbits(origReg, CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO, origRoBase);
  /* Extract new read only bits */
  pcie_getbits(new_val, CSL_RCCFGDBICS_EXPANSION_ROM_BAR_EXROM_ADDRESS_RO, newRoBase);

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
} /* pciev1_write_type1ExpnsnRom_reg */


/*****************************************************************************
 * Read and split up the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev1_read_type1BridgeInt_reg
(
  const CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->BRIDGE_INT;

  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_INT_LIN,      swReg->intLine);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_INT_PIN,      swReg->intPin);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_PERR_RESP_EN, swReg->pErrRespEn);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_SERR_EN,      swReg->serrEn);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_ISA_EN,       swReg->isaEn);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_VGA_EN,       swReg->vgaEn);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_VGA_16B_DEC,  swReg->vgaDecode);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_MST_ABT_MOD,  swReg->mstAbortMode);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_SEC_BUS_RST,  swReg->secBusRst);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_FAST_B2B_EN,  swReg->b2bEn);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_PRI_DT,       swReg->priTimer);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_SEC_DT,       swReg->secTimer);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_DT_STS,       swReg->timerStatus);
  pcie_getbits(val, CSL_RCCFGDBICS_BRIDGE_INT_DT_SERR_EN,   swReg->serrEnStatus);

  return pcie_RET_OK;
} /* pciev1_read_type1BridgeInt_reg */


/*****************************************************************************
 * Combine and write the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev1_write_type1BridgeInt_reg
(
  CSL_RcCfgDbIcsRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_INT_LIN,      swReg->intLine);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_INT_PIN,      swReg->intPin);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_PERR_RESP_EN, swReg->pErrRespEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_SERR_EN,      swReg->serrEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_ISA_EN,       swReg->isaEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_VGA_EN,       swReg->vgaEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_VGA_16B_DEC,  swReg->vgaDecode);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_MST_ABT_MOD,  swReg->mstAbortMode);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_SEC_BUS_RST,  swReg->secBusRst);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_FAST_B2B_EN,  swReg->b2bEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_PRI_DT,       swReg->priTimer);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_SEC_DT,       swReg->secTimer);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_DT_STS,       swReg->timerStatus);
  pcie_setbits(new_val, CSL_RCCFGDBICS_BRIDGE_INT_DT_SERR_EN,   swReg->serrEnStatus);

  baseAddr->BRIDGE_INT = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev1_write_type1BridgeInt_reg */


/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS **********************
 ****************************************************************************/

/*****************************************************************************  
 * Read and split up the Slot Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_read_slotCap_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotCapReg_t *swReg 
)
{
  uint32_t val = swReg->raw = baseAddr->PCIE_CAP_STRUC.SLOT_CAP;

  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_ABP,   swReg->attnButton);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_PCP,   swReg->pwrCtl);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_MRLSP, swReg->mrlSensor);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_AIP,   swReg->attnInd);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_PIP,   swReg->pwrInd);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_HPS,   swReg->hpSurprise);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_HPC,   swReg->hpCap);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_SPLV,  swReg->pwrLmtValue);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_SPLS,  swReg->pwrLmtScale);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_EIP,   swReg->emlPresent);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_NCCS,  swReg->cmdCompSupp);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAP_PSN,   swReg->slotNum);

  return pcie_RET_OK;
} /* pciev1_read_slotCap_reg */


/*****************************************************************************  
 * Combine and write the Slot Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_write_slotCap_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotCapReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_ABP,   swReg->attnButton);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_PCP,   swReg->pwrCtl);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_MRLSP, swReg->mrlSensor);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_AIP,   swReg->attnInd);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_PIP,   swReg->pwrInd);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_HPS,   swReg->hpSurprise);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_HPC,   swReg->hpCap);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_SPLV,  swReg->pwrLmtValue);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_SPLS,  swReg->pwrLmtScale);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_EIP,   swReg->emlPresent);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_NCCS,  swReg->cmdCompSupp);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAP_PSN,   swReg->slotNum);

  baseAddr->PCIE_CAP_STRUC.SLOT_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_slotCap_reg */


/*****************************************************************************  
 * Read and split up the Slot Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_read_slotStatCtrl_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *swReg 
)
{
  uint32_t val = swReg->raw = baseAddr->PCIE_CAP_STRUC.SLOT_CAS;

  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_ABP_EN,   swReg->attnButtEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PFD_EN,   swReg->pwrFltDetEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_MRLSC_EN, swReg->mrlChgEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PDC_EN,   swReg->prsDetChgEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_CCI_EN,   swReg->cmdCmpIntEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_HPI_EN,   swReg->hpIntEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_AIC,      swReg->attnIndCtl);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PIC,      swReg->pmIndCtl);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PCC,      swReg->pmCtl);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_EIC,      swReg->emLockCtl);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_DSC_EN,   swReg->dllChgEn);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_ABP,      swReg->attnPressed);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PFD,      swReg->pwrFault);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_MRCSC,    swReg->mrlChange);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PDC,      swReg->presenceChg);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_CC,       swReg->cmdComplete);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_MRLSS,    swReg->mrlState);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_PDS,      swReg->presenceDet);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_EIS,      swReg->emLock);
  pcie_getbits(val, CSL_RCCFGDBICS_SLOT_CAS_DSC,      swReg->dllState);

  return pcie_RET_OK;
} /* pciev1_read_slotStatCtrl_reg */


/*****************************************************************************  
 * Combine and write the Slot Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_write_slotStatCtrl_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_ABP_EN,   swReg->attnButtEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PFD_EN,   swReg->pwrFltDetEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_MRLSC_EN, swReg->mrlChgEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PDC_EN,   swReg->prsDetChgEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_CCI_EN,   swReg->cmdCmpIntEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_HPI_EN,   swReg->hpIntEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_AIC,      swReg->attnIndCtl);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PIC,      swReg->pmIndCtl);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PCC,      swReg->pmCtl);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_EIC,      swReg->emLockCtl);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_DSC_EN,   swReg->dllChgEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_ABP,      swReg->attnPressed);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PFD,      swReg->pwrFault);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_MRCSC,    swReg->mrlChange);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PDC,      swReg->presenceChg);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_CC,       swReg->cmdComplete);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_MRLSS,    swReg->mrlState);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_PDS,      swReg->presenceDet);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_EIS,      swReg->emLock);
  pcie_setbits(new_val, CSL_RCCFGDBICS_SLOT_CAS_DSC,      swReg->dllState);

  baseAddr->PCIE_CAP_STRUC.SLOT_CAS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_slotStatCtrl_reg */


/*****************************************************************************
 * Read and split up the Root Control and Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_read_rootCtrlCap_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *swReg 
)
{
  uint32_t val = swReg->raw = baseAddr->PCIE_CAP_STRUC.ROOT_CAC;

  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_SECE_EN,  swReg->serrEn);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_SENE_EN,  swReg->serrNFatalErr);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_SEFE_EN,  swReg->serrFatalErr);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_PMEI_EN,  swReg->pmeIntEn);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_CRSSV_EN, swReg->crsSwEn);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_CAC_CRSSV,    swReg->crsSw);

  return pcie_RET_OK;
} /* pciev1_read_rootCtrlCap_reg */


/*****************************************************************************  
 * Combine and write the Root Control and Capabilities register
 ****************************************************************************/  
pcieRet_e pciev1_write_rootCtrlCap_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_SECE_EN,  swReg->serrEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_SENE_EN,  swReg->serrNFatalErr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_SEFE_EN,  swReg->serrFatalErr);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_PMEI_EN,  swReg->pmeIntEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_CRSSV_EN, swReg->crsSwEn);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_CAC_CRSSV,    swReg->crsSw);

  baseAddr->PCIE_CAP_STRUC.ROOT_CAC = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_rootCtrlCap_reg */


/*****************************************************************************  
 * Read and split up the Root Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_read_rootStatus_reg
(
  const CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootStatusReg_t *swReg 
)
{
  uint32_t val = swReg->raw = baseAddr->PCIE_CAP_STRUC.ROOT_STS;

  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_STS_PME_RID, swReg->pmeReqID);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_STS_PME_STS, swReg->pmeStatus);
  pcie_getbits(val, CSL_RCCFGDBICS_ROOT_STS_PME_PND, swReg->pmePend);

  return pcie_RET_OK;
} /* pciev1_read_rootStatus_reg */


/*****************************************************************************  
 * Combine and write the Root Status and Control register
 ****************************************************************************/  
pcieRet_e pciev1_write_rootStatus_reg
(
  CSL_RcCfgDbIcsRegs   *baseAddr, 
  pcieRootStatusReg_t *swReg 
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_STS_PME_RID, swReg->pmeReqID);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_STS_PME_STS, swReg->pmeStatus);
  pcie_setbits(new_val, CSL_RCCFGDBICS_ROOT_STS_PME_PND, swReg->pmePend);

  baseAddr->PCIE_CAP_STRUC.ROOT_STS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_rootStatus_reg */

/* Nothing past this point */

