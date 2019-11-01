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
 *  File Name: pciev0_cfg.c
 *
 *  Processing/configuration functions for the PCIe Configuration Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v0/pcieloc.h>


/*****************************************************************************
 **********  PCIe CONFIG REGISTERS COMMON TO TYPE0 AND TYPE1  *****************
 ****************************************************************************/

/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev0_read_vndDevId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieVndDevIdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->VENDOR_DEVICE_ID;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_VENDOR_DEVICE_ID_VENDOR_ID,  reg->vndId);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_VENDOR_DEVICE_ID_DEVICE_ID,  reg->devId);

  return pcie_RET_OK;
} /* pciev0_read_vndDevId_reg */

/*****************************************************************************
 * Combine and write the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev0_write_vndDevId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieVndDevIdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_VENDOR_DEVICE_ID_VENDOR_ID,  reg->vndId);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_VENDOR_DEVICE_ID_DEVICE_ID,  reg->devId);

  baseAddr->VENDOR_DEVICE_ID = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_vndDevId_reg */



/*****************************************************************************
 * Read and split up the Status and Command register
 ****************************************************************************/
pcieRet_e pciev0_read_statusCmd_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieStatusCmdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->STATUS_COMMAND;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_PARITY_ERROR,          reg->parity);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SIGNALED_SYSTEM_ERROR, reg->sysError);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_RECEIVED_MASTER_ABORT, reg->mstAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_RECEIVED_TARGET_ABORT, reg->tgtAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SIGNALED_TARGET_ABORT, reg->sigTgtAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_DATA_PARITY_ERROR,     reg->parError);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_CAPABILITIES_LIST,     reg->capList);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_INTERRUPT_STATUS,      reg->stat);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_INTX_DISABLE,          reg->dis);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SERR_ENABLE,           reg->serrEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_PARITY_ERROR_RESPONSE, reg->resp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_BUS_MASTER,            reg->busMs);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_MEMORY_SPACE,          reg->memSp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_IO_SPACE,              reg->ioSp);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->devSelTime  = 0;
  reg->fastB2B     = 0;
  reg->c66MhzCap   = 0;
  reg->idselCtrl   = 0;
  reg->vgaSnoop    = 0;
  reg->memWrInva   = 0;
  reg->specCycleEn = 0;

  return pcie_RET_OK;
} /* pciev0_read_statusCmd_reg */

/*****************************************************************************
 * Combine and write the Status and Command register
 ****************************************************************************/
pcieRet_e pciev0_write_statusCmd_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieStatusCmdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_PARITY_ERROR,          reg->parity);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SIGNALED_SYSTEM_ERROR, reg->sysError);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_RECEIVED_MASTER_ABORT, reg->mstAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_RECEIVED_TARGET_ABORT, reg->tgtAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SIGNALED_TARGET_ABORT, reg->sigTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_DATA_PARITY_ERROR,     reg->parError);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_CAPABILITIES_LIST,     reg->capList);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_INTERRUPT_STATUS,      reg->stat);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_INTX_DISABLE,          reg->dis);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_SERR_ENABLE,           reg->serrEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_PARITY_ERROR_RESPONSE, reg->resp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_BUS_MASTER,            reg->busMs);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_MEMORY_SPACE,          reg->memSp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_STATUS_COMMAND_IO_SPACE,              reg->ioSp);

  baseAddr->STATUS_COMMAND = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_statusCmd_reg */

/*****************************************************************************
 * Read and split up the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev0_read_revId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRevIdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CLASSCODE_REVID;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_CLASSCODE_REVID_CLASS_CODE,   reg->classCode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_CLASSCODE_REVID_REVISION_ID,  reg->revId);

  return pcie_RET_OK;
} /* pciev0_read_revId_reg */

/*****************************************************************************
 * Combine and write the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev0_write_revId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRevIdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_CLASSCODE_REVID_CLASS_CODE,   reg->classCode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_CLASSCODE_REVID_REVISION_ID,  reg->revId);

  baseAddr->CLASSCODE_REVID = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_revId_reg */


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 0 REGISTERS  *****************
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev0_read_bist_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieBistReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->BIST_HEADER;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_BIST_CAPABLE,           reg->bistCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_START_BIST,             reg->startBist);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_COMPLETION_CODE,        reg->compCode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_MULTI_FUNCTION_DEVICE,  reg->mulfunDev);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_HEADER_TYPE,            reg->hdrType);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_LATENCY_TIMER,          reg->latTmr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_CACHE_LINE_SIZE,        reg->cacheLnSize);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->bist = 0;

  return pcie_RET_OK;
} /* pciev0_read_bist_reg */

/*****************************************************************************
 * Combine and write the BIST and Header register
 ****************************************************************************/
pcieRet_e pciev0_write_bist_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieBistReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_BIST_CAPABLE,           reg->bistCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_START_BIST,             reg->startBist);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_COMPLETION_CODE,        reg->compCode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_MULTI_FUNCTION_DEVICE,  reg->mulfunDev);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_HEADER_TYPE,            reg->hdrType);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_LATENCY_TIMER,          reg->latTmr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BIST_HEADER_CACHE_LINE_SIZE,        reg->cacheLnSize);

  baseAddr->BIST_HEADER = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_bist_reg */

/*****************************************************************************
 * Read and split up the BAR register
 ****************************************************************************/
pcieRet_e pciev0_read_type0Bar_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBarReg_t *reg,
  int32_t barNum
)
{
  uint32_t val = reg->raw = baseAddr->BAR[barNum];

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_BASE_ADDRESS,     reg->base);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_PREFETCHABLE,     reg->prefetch);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_TYPE,             reg->type);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_MEMORY_SPACE,     reg->memSpace);

  return pcie_RET_OK;
} /* pciev0_read_type0Bar_reg */

/*****************************************************************************
 * Combine and write the BAR register
 ****************************************************************************/
pcieRet_e pciev0_write_type0Bar_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBarReg_t *reg,
  int32_t barNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_BASE_ADDRESS,     reg->base);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_PREFETCHABLE,     reg->prefetch);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_TYPE,             reg->type);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_BAR_MEMORY_SPACE,     reg->memSpace);

  baseAddr->BAR[barNum] = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type0Bar_reg */


/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev0_read_type0Bar32bit_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBar32bitReg_t *reg,
  int32_t barNum
)
{
  reg->reg32 = reg->raw = baseAddr->BAR[barNum];
  return pcie_RET_OK;
} /* pciev0_read_type0Bar32bit_reg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev0_write_type0Bar32bit_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieBar32bitReg_t *reg,
  int32_t barNum
)
{
  baseAddr->BAR[barNum] = reg->raw = reg->reg32;
  return pcie_RET_OK;
} /* pciev0_write_type0Bar32bit_reg */


/*****************************************************************************
 * Read and split up the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev0_read_subId_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSubIdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SUBSYS_VNDR_ID;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SUBSYS_VNDR_ID_SUBSYSTEM_ID,         reg->subId);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SUBSYS_VNDR_ID_SUBSYSTEM_VENDOR_ID,  reg->subVndId);

  return pcie_RET_OK;
} /* pciev0_read_subId_reg */

/*****************************************************************************
 * Combine and write the Subsystem and Subsystem Vendor ID register
 ****************************************************************************/
pcieRet_e pciev0_write_subId_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSubIdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SUBSYS_VNDR_ID_SUBSYSTEM_ID,         reg->subId);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SUBSYS_VNDR_ID_SUBSYSTEM_VENDOR_ID,  reg->subVndId);

  baseAddr->SUBSYS_VNDR_ID = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_subId_reg */


/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev0_read_expRom_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieExpRomReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->EXPNSN_ROM;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_EXPNSN_ROM_EXPANSION_ROM_BASE_ADDRESS,  reg->expRomAddr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_EXPNSN_ROM_EXPANSION_ROM_ENABLE,        reg->enable);

  return pcie_RET_OK;
} /* pciev0_read_expRom_reg */

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev0_write_expRom_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieExpRomReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_EXPNSN_ROM_EXPANSION_ROM_BASE_ADDRESS,  reg->expRomAddr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_EXPNSN_ROM_EXPANSION_ROM_ENABLE,        reg->enable);

  baseAddr->EXPNSN_ROM = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_expRom_reg */


/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev0_read_capPtr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCapPtrReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CAP_PTR;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_CAP_PTR_CAP_PTR,  reg->ptr);

  return pcie_RET_OK;
} /* pciev0_read_capPtr_reg */

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev0_write_capPtr_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCapPtrReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_CAP_PTR_CAP_PTR,  reg->ptr);

  baseAddr->CAP_PTR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_capPtr_reg */


/*****************************************************************************
 * Read and split up the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev0_read_intPin_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieIntPinReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->INT_PIN;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_INT_PIN_INT_PIN,  reg->intPin);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_INT_PIN_INT_LINE, reg->intLine);

  return pcie_RET_OK;
} /* pciev0_read_intPin_reg */

/*****************************************************************************
 * Combine and write the Interrupt Pin register
 ****************************************************************************/
pcieRet_e pciev0_write_intPin_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieIntPinReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_INT_PIN_INT_PIN,  reg->intPin);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_INT_PIN_INT_LINE, reg->intLine);

  baseAddr->INT_PIN = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_intPin_reg */



/*****************************************************************************
 **********  PCIe LOCAL/REMOTE CONFIG TYPE 1 REGISTERS  *****************
 ****************************************************************************/
/*****************************************************************************
 * Read and split up the BIST, Header Type, Latency Time, and Cache Line Size register
 ****************************************************************************/
pcieRet_e pciev0_read_type1BistHeader_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieType1BistHeaderReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->BIST_HEADER;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_BISTCAPABLE,           reg->bistCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_START_BIST,            reg->startBist);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_COMPLETION_CODE,       reg->compCode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_MULTI_FUNCTION_DEVICE, reg->mulFunDev);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_HEADER_TYPE,           reg->hdrType);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_LATENCY_TIMER,         reg->latTmr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_CACHE_LINE_SIZE,       reg->cacheLnSize);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->bist = 0;

  return pcie_RET_OK;
} /* pciev0_read_type1BistHeader_reg */

/*****************************************************************************
 * Combine and write the BIST, Header Type, Latency Time, and Cache Line Size register
 ****************************************************************************/
pcieRet_e pciev0_write_type1BistHeader_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieType1BistHeaderReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_BISTCAPABLE,           reg->bistCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_START_BIST,            reg->startBist);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_COMPLETION_CODE,       reg->compCode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_MULTI_FUNCTION_DEVICE, reg->mulFunDev);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_HEADER_TYPE,           reg->hdrType);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_LATENCY_TIMER,         reg->latTmr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BIST_HEADER_CACHE_LINE_SIZE,       reg->cacheLnSize);

  baseAddr->BIST_HEADER = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_type1BistHeader_reg */

/*****************************************************************************
 * Read and split up the BAR register
 ****************************************************************************/
pcieRet_e pciev0_read_type1Bar_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBarReg_t *reg,
  int32_t barNum
)
{
  uint32_t val = reg->raw = baseAddr->BAR[barNum];

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_BASE_ADDRESS,     reg->base);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_PREFETCHABLE,     reg->prefetch);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_TYPE,             reg->type);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_MEMORY_SPACE,     reg->memSpace);

  return pcie_RET_OK;
} /* pciev0_read_type1Bar_reg */

/*****************************************************************************
 * Combine and write the BAR register
 ****************************************************************************/
pcieRet_e pciev0_write_type1Bar_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBarReg_t *reg,
  int32_t barNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_BASE_ADDRESS,     reg->base);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_PREFETCHABLE,     reg->prefetch);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_TYPE,             reg->type);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BAR_MEMORY_SPACE,     reg->memSpace);

  baseAddr->BAR[barNum] = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1Bar_reg */

/*****************************************************************************
 * Read and split up the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev0_read_type1Bar32bit_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBar32bitReg_t *reg,
  int32_t barNum
)
{
  reg->reg32 = reg->raw = baseAddr->BAR[barNum];
  return pcie_RET_OK;
} /* pciev0_read_type1Bar32bit_reg */

/*****************************************************************************
 * Combine and write the BAR 32bits register
 ****************************************************************************/
pcieRet_e pciev0_write_type1Bar32bit_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieBar32bitReg_t *reg,
  int32_t barNum
)
{
  baseAddr->BAR[barNum] = reg->raw = reg->reg32;
  
  return pcie_RET_OK;
} /* pciev0_write_type1Bar32bit_reg */

/*****************************************************************************
 * Read and split up the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev0_read_type1BusNum_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BusNumReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->BUSNUM;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SECONDARY_LATENCY_TIMER, reg->secLatTmr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SUBORDINATE_BUS_NUMBER,  reg->subBusNum);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SECONDARY_BUS_NUMBER,    reg->secBusNum);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_PRIMARY_BUS_NUMBER,      reg->priBusNum);

  return pcie_RET_OK;
} /* pciev0_read_type1BusNum_reg */

/*****************************************************************************
 * Combine and write the Latency Timer and Bus Number register
 ****************************************************************************/
pcieRet_e pciev0_write_type1BusNum_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BusNumReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SECONDARY_LATENCY_TIMER, reg->secLatTmr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SUBORDINATE_BUS_NUMBER,  reg->subBusNum);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_SECONDARY_BUS_NUMBER,    reg->secBusNum);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BUSNUM_PRIMARY_BUS_NUMBER,      reg->priBusNum);

  baseAddr->BUSNUM = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1BusNum_reg */

/*****************************************************************************
 * Read and split up the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev0_read_type1SecStat_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1SecStatReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->SECSTAT;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_DTCT_PERROR,    reg->dtctPError);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_SYS_ERROR,   reg->rxSysError);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_MST_ABORT,   reg->rxMstAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_TGT_ABORT,   reg->rxTgtAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_TX_TGT_ABORT,   reg->txTgtAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_MST_DPERR,      reg->mstDPErr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_LIMIT,       reg->IOLimit);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_ADDRESSING,  reg->IOLimitAddr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_BASE,        reg->IOBase);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_ADDRESSING2, reg->IOBaseAddr);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->devselTiming = 0;
  reg->fastB2bCap   = 0;
  reg->c66mhzCapa   = 0;

  return pcie_RET_OK;
} /* pciev0_read_type1SecStat_reg */

/*****************************************************************************
 * Combine and write the Secondary Status and IO Base/Limit Register
 ****************************************************************************/
pcieRet_e pciev0_write_type1SecStat_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1SecStatReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_DTCT_PERROR,    reg->dtctPError);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_SYS_ERROR,   reg->rxSysError);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_MST_ABORT,   reg->rxMstAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_RX_TGT_ABORT,   reg->rxTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_TX_TGT_ABORT,   reg->txTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_MST_DPERR,      reg->mstDPErr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_LIMIT,       reg->IOLimit);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_ADDRESSING,  reg->IOLimitAddr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_BASE,        reg->IOBase);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SECSTAT_IO_ADDRESSING2, reg->IOBaseAddr);

  baseAddr->SECSTAT = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1SecStat_reg */

/*****************************************************************************
 * Read and split up the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev0_read_type1Memspace_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1MemspaceReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->MEMSPACE;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_MEMSPACE_MEMORY_LIMIT,   reg->limit);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_MEMSPACE_MEMORY_BASE,    reg->base);

  return pcie_RET_OK;
} /* pciev0_read_type1Memspace_reg */

/*****************************************************************************
 * Combine and write the Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev0_write_type1Memspace_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1MemspaceReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_MEMSPACE_MEMORY_LIMIT,   reg->limit);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_MEMSPACE_MEMORY_BASE,    reg->base);

  baseAddr->MEMSPACE = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1Memspace_reg */

/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev0_read_prefMem_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefMemReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PREFETCH_MEM;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_END_ADDRESS,         reg->limit);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_MEMORY_ADDRESSING,   reg->limitAddr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_STARTADDRESS,        reg->base);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_MEMORY_ADDRESSING2,  reg->baseAddr);

  return pcie_RET_OK;
} /* pciev0_read_prefMem_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit and Base register
 ****************************************************************************/
pcieRet_e pciev0_write_prefMem_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefMemReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_END_ADDRESS,         reg->limit);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_MEMORY_ADDRESSING,   reg->limitAddr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_STARTADDRESS,        reg->base);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_MEM_MEMORY_ADDRESSING2,  reg->baseAddr);

  baseAddr->PREFETCH_MEM = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_prefMem_reg */

/*****************************************************************************
 * Read and split up the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev0_read_prefBaseUpper_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PREFETCH_BASE;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_BASE_BASE_ADDRESS,  reg->base);

  return pcie_RET_OK;
} /* pciev0_read_prefBaseUp_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Base Upper register
 ****************************************************************************/
pcieRet_e pciev0_write_prefBaseUpper_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefBaseUpperReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_BASE_BASE_ADDRESS,  reg->base);

  baseAddr->PREFETCH_BASE = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_prefBaseUp_reg */


/*****************************************************************************
 * Read and split up the Prefetchable Memory Limit Upper register
 ****************************************************************************/
pcieRet_e pciev0_read_prefLimitUpper_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PREFETCH_LIMIT;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_LIMIT_LIMIT_ADDRESS, reg->limit);

  return pcie_RET_OK;
} /* pciev0_read_prefLimitUp_reg */

/*****************************************************************************
 * Combine and write the Prefetchable Memory Limit Upper  register
 ****************************************************************************/
pcieRet_e pciev0_write_prefLimitUpper_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pciePrefLimitUpperReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_PREFETCH_LIMIT_LIMIT_ADDRESS, reg->limit);

  baseAddr->PREFETCH_LIMIT = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_prefLimitUp_reg */

/*****************************************************************************
 * Read and split up the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev0_read_type1IOSpace_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->IOSPACE;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_IOSPACE_IOBASE,  reg->IOBase);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_IOSPACE_IOLIMIT, reg->IOLimit);

  return pcie_RET_OK;
} /* pciev0_read_type1IOSpace_reg */

/*****************************************************************************
 * Combine and write the IO Base and Limit Upper 16 bits register
 ****************************************************************************/
pcieRet_e pciev0_write_type1IOSpace_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1IOSpaceReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_IOSPACE_IOBASE,  reg->IOBase);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_IOSPACE_IOLIMIT, reg->IOLimit);

  baseAddr->IOSPACE = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1IOSpace_reg */

/*****************************************************************************
 * Read and split up the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev0_read_type1CapPtr_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1CapPtrReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CAP_PTR;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_CAP_PTR_CAP_PTR, reg->capPtr);

  return pcie_RET_OK;
} /* pciev0_read_type1CapPtr_reg */

/*****************************************************************************
 * Combine and write the Capabilities Pointer register
 ****************************************************************************/
pcieRet_e pciev0_write_type1CapPtr_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1CapPtrReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_CAP_PTR_CAP_PTR, reg->capPtr);

  baseAddr->CAP_PTR = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1CapPtr_reg */

/*****************************************************************************
 * Read and split up the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev0_read_type1ExpnsnRom_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->EXPNSN_ROM;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_EXPNSN_ROM_EXPANSION_ROM_BASE_ADDRESS, reg->expRomBaseAddr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_EXPNSN_ROM_EXPANSION_ROM_ENABLE,       reg->expRomEn);

  return pcie_RET_OK;
} /* pciev0_read_type1ExpnsnRom_reg */

/*****************************************************************************
 * Combine and write the Expansion ROM Base Address register
 ****************************************************************************/
pcieRet_e pciev0_write_type1ExpnsnRom_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1ExpnsnRomReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_EXPNSN_ROM_EXPANSION_ROM_BASE_ADDRESS, reg->expRomBaseAddr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_EXPNSN_ROM_EXPANSION_ROM_ENABLE,       reg->expRomEn);

  baseAddr->EXPNSN_ROM = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1ExpnsnRom_reg */

/*****************************************************************************
 * Read and split up the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev0_read_type1BridgeInt_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->BRIDGE_INT;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SERREN_STATUS,  reg->serrEnStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_TIMER_STATUS,   reg->timerStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SEC_TIMER,      reg->secTimer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_PRI_TIMER,      reg->priTimer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_B2B_EN,         reg->b2bEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SEC_BUS_RST,    reg->secBusRst);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_MST_ABORT_MODE, reg->mstAbortMode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_VGA_DECODE,     reg->vgaDecode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_VGA_EN,         reg->vgaEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_ISA_EN,         reg->isaEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SERR_EN,        reg->serrEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_PERR_RESP_EN,   reg->pErrRespEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_INT_PIN,        reg->intPin);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_INT_LINE,       reg->intLine);

  return pcie_RET_OK;
} /* pciev0_read_type1BridgeInt_reg */

/*****************************************************************************
 * Combine and write the Bridge Control and Interrupt register
 ****************************************************************************/
pcieRet_e pciev0_write_type1BridgeInt_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs  *baseAddr,  
  pcieType1BridgeIntReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SERREN_STATUS,  reg->serrEnStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_TIMER_STATUS,   reg->timerStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SEC_TIMER,      reg->secTimer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_PRI_TIMER,      reg->priTimer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_B2B_EN,         reg->b2bEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SEC_BUS_RST,    reg->secBusRst);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_MST_ABORT_MODE, reg->mstAbortMode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_VGA_DECODE,     reg->vgaDecode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_VGA_EN,         reg->vgaEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_ISA_EN,         reg->isaEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_SERR_EN,        reg->serrEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_PERR_RESP_EN,   reg->pErrRespEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_INT_PIN,        reg->intPin);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_BRIDGE_INT_INT_LINE,       reg->intLine);

  baseAddr->BRIDGE_INT = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_type1BridgeInt_reg */

/*****************************************************************************
 **********  Power Management Capability Registers ***************************
 ****************************************************************************/

/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev0_read_pmCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMCAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_SUPP_N,   reg->pmeSuppN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_D2_SUPP_N,    reg->d2SuppN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_D1_SUPP_N,    reg->d1SuppN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_AUX_CURR_N,   reg->auxCurrN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_DSI_N,        reg->dsiN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_CLK,      reg->pmeClk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_SPEC_VER, reg->pmeSpecVer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PM_NEXT_PTR,  reg->pmNextPtr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PM_CAP_ID,    reg->pmCapID);

  return pcie_RET_OK;
} /* pciev0_read_pmCap_reg */

/*****************************************************************************
 * Combine and write the Power Management Capability register
 ****************************************************************************/
pcieRet_e pciev0_write_pmCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_SUPP_N,   reg->pmeSuppN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_D2_SUPP_N,    reg->d2SuppN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_D1_SUPP_N,    reg->d1SuppN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_AUX_CURR_N,   reg->auxCurrN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_DSI_N,        reg->dsiN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_CLK,      reg->pmeClk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PME_SPEC_VER, reg->pmeSpecVer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PM_NEXT_PTR,  reg->pmNextPtr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PMCAP_PM_CAP_ID,    reg->pmCapID);

  baseAddr->PMCAP = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_pmCap_reg */

/*****************************************************************************
 * Read and split up the Power Management Capabilties Control and Status register
 ****************************************************************************/
pcieRet_e pciev0_read_pmCapCtlStat_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapCtlStatReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PM_CTL_STAT;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_REG,      reg->dataReg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_CLK_CTRL_EN,   reg->clkCtrlEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_B2_B3_SUPPORT, reg->b2b3Support);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PME_STATUS,    reg->pmeStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_SCALE,    reg->dataScale);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_SELECT,   reg->dataSelect);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PME_EN,        reg->pmeEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_NO_SOFT_RST,   reg->noSoftRst);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PWR_STATE,     reg->pwrState);

  return pcie_RET_OK;
} /* pciev0_read_pmCapCtlStat_reg */

/*****************************************************************************
 * Combine and write the Power Management Capabilties Control and Status register
 ****************************************************************************/
pcieRet_e pciev0_write_pmCapCtlStat_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pciePMCapCtlStatReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_REG,      reg->dataReg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_CLK_CTRL_EN,   reg->clkCtrlEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_B2_B3_SUPPORT, reg->b2b3Support);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PME_STATUS,    reg->pmeStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_SCALE,    reg->dataScale);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_DATA_SELECT,   reg->dataSelect);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PME_EN,        reg->pmeEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_NO_SOFT_RST,   reg->noSoftRst);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PM_CTL_STAT_PWR_STATE,     reg->pwrState);

  baseAddr->PM_CTL_STAT = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_pmCapCtlStat_reg */

/*****************************************************************************
 **********  Message Signaling Interrupt  REGISTERS  *************************
 ****************************************************************************/

/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/


/*****************************************************************************
 * Read and split up the MSI Capabilities register
 ****************************************************************************/
pcieRet_e pciev0_read_msiCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiCapReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->MSI_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_64BIT_EN,     reg->en64bit);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MULT_MSG_EN,  reg->multMsgEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MULT_MSG_CAP, reg->multMsgCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MSI_EN,       reg->msiEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_NEXT_CAP,     reg->nextCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_CAP_ID,       reg->capId);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->pvmEn      = 0u;
  reg->extDataCap = 0u;
  reg->extDataEn  = 0u;

  return pcie_RET_OK;
} /* pciev0_read_msiCap_reg */

/*****************************************************************************
 * Combine and write the MSI Capabilities register
 ****************************************************************************/
pcieRet_e pciev0_write_msiCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiCapReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_64BIT_EN,    reg->en64bit);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MULT_MSG_EN, reg->multMsgEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MULT_MSG_CAP,reg->multMsgCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_MSI_EN,      reg->msiEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_NEXT_CAP,    reg->nextCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_CAP_CAP_ID,      reg->capId);

  baseAddr->MSI_CAP = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_msiCap_reg */


/*****************************************************************************
 * Read and split up the MSI Lower 32 Bits register
 ****************************************************************************/
pcieRet_e pciev0_read_msiLo32_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiLo32Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->MSI_LOW32;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_LOW32_LOW32_ADDR, reg->addr);

  return pcie_RET_OK;
} /* pciev0_read_msiLo32_reg */

/*****************************************************************************
 * Combine and write the MSI Lower 32 Bits register
 ****************************************************************************/
pcieRet_e pciev0_write_msiLo32_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiLo32Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_LOW32_LOW32_ADDR, reg->addr);

  baseAddr->MSI_LOW32 = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_msiLo32_reg */


/*****************************************************************************
 * Read and split up the MSI Upper 32 Bits register
 ****************************************************************************/
pcieRet_e pciev0_read_msiUp32_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiUp32Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->MSI_UP32;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_UP32_UP32_ADDR, reg->addr);

  return pcie_RET_OK;
} /* pciev0_read_msiUp32_reg */

/*****************************************************************************
 * Combine and write the MSI Upper 32 Bits register
 ****************************************************************************/
pcieRet_e pciev0_write_msiUp32_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiUp32Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_UP32_UP32_ADDR, reg->addr);

  baseAddr->MSI_UP32 = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_msiUp32_reg */


/*****************************************************************************
 * Read and split up the MSI Data register
 ****************************************************************************/
pcieRet_e pciev0_read_msiData_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiDataReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->MSI_DATA;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_DATA_MSI_DATA, reg->data);

  return pcie_RET_OK;
} /* pciev0_read_msiData_reg */

/*****************************************************************************
 * Combine and write the MSI Data register
 ****************************************************************************/
pcieRet_e pciev0_write_msiData_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieMsiDataReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_MSI_DATA_MSI_DATA, reg->data);

  baseAddr->MSI_DATA = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_msiData_reg */


/*****************************************************************************
 **********  PCIe CAPABILITIES  REGISTERS **********************
 ****************************************************************************/


/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************  
 * Read and split up the PCIE Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_read_pciesCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePciesCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIES_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_INT_MSG,    reg->intMsg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_SLT_IMPL_N, reg->sltImplN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_DPORT_TYPE, reg->dportType);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_PCIE_CAP,   reg->pcieCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_NEXT_CAP,   reg->nextCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_CAP_ID,     reg->capId);

  return pcie_RET_OK;
} /* pciev0_read_pciesCap_reg */


/*****************************************************************************  
 * Combine and write the PCIE Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_write_pciesCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePciesCapReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_INT_MSG,    reg->intMsg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_SLT_IMPL_N, reg->sltImplN);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_DPORT_TYPE, reg->dportType);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_PCIE_CAP,   reg->pcieCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_NEXT_CAP,   reg->nextCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIES_CAP_CAP_ID,     reg->capId);

  baseAddr->PCIES_CAP = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pciesCap_reg */

/*****************************************************************************  
 * Read and split up the Device Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_read_deviceCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDeviceCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->DEVICE_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PWR_LIMIT_SCALE, reg->pwrLimitScale);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PWR_LIMIT_VALUE, reg->pwrLimitValue);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_ERR_RPT,         reg->errRpt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_L1_LATENCY,      reg->l1Latency);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_L0_LATENCY,      reg->l0Latency);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_EXT_TAG_FLD,     reg->extTagFld);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PHANTOM_FLD,     reg->phantomFld);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_MAX_PAYLD_SZ,    reg->maxPayldSz);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->flrEn = 0;

  return pcie_RET_OK;
} /* pciev0_read_deviceCap_reg */


/*****************************************************************************  
 * Combine and write the Device Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_write_deviceCap_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDeviceCapReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PWR_LIMIT_SCALE, reg->pwrLimitScale);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PWR_LIMIT_VALUE, reg->pwrLimitValue);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_ERR_RPT,         reg->errRpt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_L1_LATENCY,      reg->l1Latency);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_L0_LATENCY,      reg->l0Latency);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_EXT_TAG_FLD,     reg->extTagFld);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_PHANTOM_FLD,     reg->phantomFld);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEVICE_CAP_MAX_PAYLD_SZ,    reg->maxPayldSz);

  baseAddr->DEVICE_CAP = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_deviceCap_reg */

/*****************************************************************************
 * Read and split up the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev0_read_devStatCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDevStatCtrlReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->DEV_STAT_CTRL;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_TPEND,          reg->tpend);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_AUX_PWR,        reg->auxPwr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_UNSUP_RQ_DET,   reg->rqDet);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_FATAL_ERR,      reg->fatalEr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NFATAL_ERR,     reg->nFatalEr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_CORR_ERR,       reg->corrEr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_MAX_REQ_SZ,     reg->maxSz);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NO_SNOOP,       reg->noSnoop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_AUX_PWR_PM_EN,  reg->auxPwrEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_PHANTOM_EN,     reg->phantomEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_XTAG_FIELD_EN,  reg->xtagEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_MAX_PAYLOAD,    reg->maxPayld);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_RELAXED,        reg->relaxed);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_UNSUP_REQ_REP,  reg->reqRp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_FATAL_ERR_REP,  reg->fatalErRp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NFATAL_ERR_REP, reg->nFatalErRp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_CORR_ERR_REP,   reg->corErRp);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->initFLR = 0u;

  return pcie_RET_OK;
} /* pciev0_read_devStatCtrl_reg */


/*****************************************************************************
 * Combine and write the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev0_write_devStatCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDevStatCtrlReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_TPEND,          reg->tpend);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_AUX_PWR,        reg->auxPwr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_UNSUP_RQ_DET,   reg->rqDet);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_FATAL_ERR,      reg->fatalEr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NFATAL_ERR,     reg->nFatalEr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_CORR_ERR,       reg->corrEr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_MAX_REQ_SZ,     reg->maxSz);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NO_SNOOP,       reg->noSnoop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_AUX_PWR_PM_EN,  reg->auxPwrEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_PHANTOM_EN,     reg->phantomEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_XTAG_FIELD_EN,  reg->xtagEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_MAX_PAYLOAD,    reg->maxPayld);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_RELAXED,        reg->relaxed);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_UNSUP_REQ_REP,  reg->reqRp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_FATAL_ERR_REP,  reg->fatalErRp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_NFATAL_ERR_REP, reg->nFatalErRp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL_CORR_ERR_REP,   reg->corErRp);

  baseAddr->DEV_STAT_CTRL = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_devStatCtrl_reg */

/*****************************************************************************  
 * Read and split up the Link Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_read_linkCap_reg
(      
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->LINK_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_PORT_NUM,         reg->portNum);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_BW_NOTIFY_CAP,    reg->bwNotifyCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_DLL_REP_CAP,      reg->dllRepCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_DOWN_ERR_REP_CAP, reg->downErrRepCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_CLK_PWR_MGMT,     reg->clkPwrMgmt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_L1_EXIT_LAT,      reg->l1ExitLat);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_LOS_EXIT_LAT,     reg->losExitLat);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_AS_LINK_PM,       reg->asLinkPm);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_MAX_LINK_WIDTH,   reg->maxLinkWidth);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_MAX_LINK_SPEED,   reg->maxLinkSpeed);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->aspmOptComp = 0;

  return pcie_RET_OK;
} /* pciev0_read_linkCap_reg */


/*****************************************************************************  
 * Combine and write the Link Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_write_linkCap_reg
(      
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCapReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_PORT_NUM,         reg->portNum);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_BW_NOTIFY_CAP,    reg->bwNotifyCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_DLL_REP_CAP,      reg->dllRepCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_DOWN_ERR_REP_CAP, reg->downErrRepCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_CLK_PWR_MGMT,     reg->clkPwrMgmt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_L1_EXIT_LAT,      reg->l1ExitLat);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_LOS_EXIT_LAT,     reg->losExitLat);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_AS_LINK_PM,       reg->asLinkPm);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_MAX_LINK_WIDTH,   reg->maxLinkWidth);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CAP_MAX_LINK_SPEED,   reg->maxLinkSpeed);

  baseAddr->LINK_CAP = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_linkCap_reg */


/*****************************************************************************  
 * Read and split up the Link Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_read_linkStatCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkStatCtrlReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->LINK_STAT_CTRL;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_STATUS,      reg->linkBwStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_MGMT_STATUS, reg->linkBwMgmtStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_DLL_ACTIVE,          reg->dllActive);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_SLOT_CLK_CFG,        reg->slotClkCfg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_TRAINING,       reg->linkTraining);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_UNDEF,               reg->undef);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_NEGOTIATED_LINK_WD,  reg->negotiatedLinkWd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_SPEED,          reg->linkSpeed);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_INT_EN,      reg->linkBwIntEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_MGMT_INT_EN, reg->linkBwMgmtIntEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_HW_AUTO_WIDTH_DIS,   reg->hwAutoWidthDis);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_CLK_PWR_MGMT_EN,     reg->clkPwrMgmtEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_EXT_SYNC,            reg->extSync);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_COMMON_CLK_CFG,      reg->commonClkCfg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_RETRAIN_LINK,        reg->retrainLink);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_DISABLE,        reg->linkDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_RCB,                 reg->rcb);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_ACTIVE_LINK_PM,      reg->activeLinkPm);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->drsSigCtrl = 0u;

  return pcie_RET_OK;
} /* pciev0_read_linkStatCtrl_reg */


/*****************************************************************************  
 * Combine and write the Link Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_write_linkStatCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkStatCtrlReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_STATUS,      reg->linkBwStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_MGMT_STATUS, reg->linkBwMgmtStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_DLL_ACTIVE,          reg->dllActive);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_SLOT_CLK_CFG,        reg->slotClkCfg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_TRAINING,       reg->linkTraining);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_UNDEF,               reg->undef);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_NEGOTIATED_LINK_WD,  reg->negotiatedLinkWd);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_SPEED,          reg->linkSpeed);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_INT_EN,      reg->linkBwIntEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_BW_MGMT_INT_EN, reg->linkBwMgmtIntEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_HW_AUTO_WIDTH_DIS,   reg->hwAutoWidthDis);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_CLK_PWR_MGMT_EN,     reg->clkPwrMgmtEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_EXT_SYNC,            reg->extSync);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_COMMON_CLK_CFG,      reg->commonClkCfg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_RETRAIN_LINK,        reg->retrainLink);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_LINK_DISABLE,        reg->linkDisable);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_RCB,                 reg->rcb);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_STAT_CTRL_ACTIVE_LINK_PM,      reg->activeLinkPm);

  baseAddr->LINK_STAT_CTRL = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_linkStatCtrl_reg */


/*****************************************************************************  
 * Read and split up the Slot Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_read_slotCap_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SLOT_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_SLOT_NUM,      reg->slotNum);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_CMD_COMP_SUPP, reg->cmdCompSupp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_EML_PRESENT,   reg->emlPresent);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_LMT_SCALE, reg->pwrLmtScale);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_LMT_VALUE, reg->pwrLmtValue);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_HP_CAP,        reg->hpCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_HP_SURPRISE,   reg->hpSurprise);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_IND,       reg->pwrInd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_ATTN_IND,      reg->attnInd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_MRL_SENSOR,    reg->mrlSensor);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_CTL,       reg->pwrCtl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_ATTN_BUTTON,   reg->attnButton);

  return pcie_RET_OK;
} /* pciev0_read_slotCap_reg */

/*****************************************************************************  
 * Combine and write the Slot Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_write_slotCap_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotCapReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_SLOT_NUM,      reg->slotNum);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_CMD_COMP_SUPP, reg->cmdCompSupp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_EML_PRESENT,   reg->emlPresent);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_LMT_SCALE, reg->pwrLmtScale);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_LMT_VALUE, reg->pwrLmtValue);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_HP_CAP,        reg->hpCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_HP_SURPRISE,   reg->hpSurprise);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_IND,       reg->pwrInd);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_ATTN_IND,      reg->attnInd);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_MRL_SENSOR,    reg->mrlSensor);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_PWR_CTL,       reg->pwrCtl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_CAP_ATTN_BUTTON,   reg->attnButton);

  baseAddr->SLOT_CAP = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_slotCap_reg */


/*****************************************************************************  
 * Read and split up the Slot Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_read_slotStatCtrl_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SLOT_STAT_CTRL;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_DLL_STATE,      reg->dllState);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_EM_LOCK,        reg->emLock);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRESENCE_DET,   reg->presenceDet);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_STATE,      reg->mrlState);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_CMD_COMLETE,    reg->cmdComplete);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRESENCE_CHG,   reg->presenceChg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_CHANGE,     reg->mrlChange);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PWR_FAULT,      reg->pwrFault);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_PRESSED,   reg->attnPressed);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_DLL_CHG_EN,     reg->dllChgEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_EM_LOCK_CTL,    reg->emLockCtl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PM_CTL,         reg->pmCtl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PM_IND_CTL,     reg->pmIndCtl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_IND_CTL,   reg->attnIndCtl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_HP_INT_EN,      reg->hpIntEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_CMD_CMP_INT_EN, reg->cmdCmpIntEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRS_DET_CHG_EN, reg->prsDetChgEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_CHG_EN,     reg->mrlChgEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PWR_FLT_DET_EN, reg->pwrFltDetEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_BUTT_EN,   reg->attnButtEn);

  return pcie_RET_OK;
} /* pciev0_read_slotStatCtrl_reg */

/*****************************************************************************  
 * Combine and write the Slot Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_write_slotStatCtrl_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieSlotStatCtrlReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_DLL_STATE,      reg->dllState);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_EM_LOCK,        reg->emLock);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRESENCE_DET,   reg->presenceDet);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_STATE,      reg->mrlState);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_CMD_COMLETE,    reg->cmdComplete);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRESENCE_CHG,   reg->presenceChg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_CHANGE,     reg->mrlChange);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PWR_FAULT,      reg->pwrFault);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_PRESSED,   reg->attnPressed);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_DLL_CHG_EN,     reg->dllChgEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_EM_LOCK_CTL,    reg->emLockCtl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PM_CTL,         reg->pmCtl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PM_IND_CTL,     reg->pmIndCtl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_IND_CTL,   reg->attnIndCtl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_HP_INT_EN,      reg->hpIntEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_CMD_CMP_INT_EN, reg->cmdCmpIntEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PRS_DET_CHG_EN, reg->prsDetChgEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_MRL_CHG_EN,     reg->mrlChgEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_PWR_FLT_DET_EN, reg->pwrFltDetEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_SLOT_STAT_CTRL_ATTN_BUTT_EN,   reg->attnButtEn);

  baseAddr->SLOT_STAT_CTRL = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_slotStatCtrl_reg */


/*****************************************************************************
 * Read and split up the Root Control and Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_read_rootCtrlCap_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->ROOT_CTRL_CAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_CRS_SW,          reg->crsSw);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_CRS_SW_EN,       reg->crsSwEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_PME_INT_EN,      reg->pmeIntEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_FATAL_ERR,  reg->serrFatalErr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_NFATAL_ERR, reg->serrNFatalErr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_EN,         reg->serrEn);

  return pcie_RET_OK;
} /* pciev0_read_rootCtrlCap_reg */


/*****************************************************************************  
 * Combine and write the Root Control and Capabilities register
 ****************************************************************************/  
pcieRet_e pciev0_write_rootCtrlCap_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootCtrlCapReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_CRS_SW,          reg->crsSw);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_CRS_SW_EN,       reg->crsSwEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_PME_INT_EN,      reg->pmeIntEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_FATAL_ERR,  reg->serrFatalErr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_NFATAL_ERR, reg->serrNFatalErr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_CTRL_CAP_SERR_EN,         reg->serrEn);

  baseAddr->ROOT_CTRL_CAP = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_rootCtrlCap_reg */


/*****************************************************************************  
 * Read and split up the Root Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_read_rootStatus_reg
(
  const CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootStatusReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->ROOT_STATUS;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_PEND,   reg->pmePend);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_STATUS, reg->pmeStatus);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_REQ_ID, reg->pmeReqID);

  return pcie_RET_OK;
} /* pciev0_read_rootStatus_reg */

/*****************************************************************************  
 * Combine and write the Root Status and Control register
 ****************************************************************************/  
pcieRet_e pciev0_write_rootStatus_reg
(
  CSL_Pcie_cfg_space_rootcomplexRegs   *baseAddr, 
  pcieRootStatusReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_PEND,   reg->pmePend);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_STATUS, reg->pmeStatus);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ROOTCOMPLEX_ROOT_STATUS_PME_REQ_ID, reg->pmeReqID);

  baseAddr->ROOT_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_rootStatus_reg */

/*****************************************************************************  
 * Read and split up the Device Capabilities 2 register
 ****************************************************************************/  
pcieRet_e pciev0_read_devCap2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevCap2Reg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->DEV_CAP2;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_CAP2_CMPL_TO_DIS_SUPP, reg->cmplToDisSupp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_CAP2_CMPL_TO_EN,       reg->cmplToEn);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->ariFwdSp         = 0u;
  reg->aorSp            = 0u;
  reg->aoc32Sp          = 0u;
  reg->aoc64Sp          = 0u;
  reg->casc128Sp        = 0u;
  reg->noRoPR           = 0u;
  reg->tphcSp           = 0u;
  reg->ltrSupp          = 0u;
  reg->lnSysCls         = 0u;
  reg->tag10bitCompSupp = 0u;
  reg->tag10bitReqSupp  = 0u;
  reg->obffSupp         = 0u;

  return pcie_RET_OK;
} /* pciev0_read_devCap2_reg */


/*****************************************************************************  
 * Combine and write the Device Capabilities 2 register
 ****************************************************************************/  
pcieRet_e pciev0_write_devCap2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevCap2Reg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_CAP2_CMPL_TO_DIS_SUPP, reg->cmplToDisSupp);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_CAP2_CMPL_TO_EN,       reg->cmplToEn);

  baseAddr->DEV_CAP2 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_devCap2_reg */


/*****************************************************************************  
 * Read and split up the Device Status and Control Register 2 register
 ****************************************************************************/  
pcieRet_e pciev0_read_devStatCtrl2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevStatCtrl2Reg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->DEV_STAT_CTRL2;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL2_CMPL_TO_DIS, reg->cmplToDis);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL2_CMPL_TO,     reg->cmplTo);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->ariFwdSp = 0;
  reg->aopReqEn = 0;
  reg->aopEgBlk = 0;
  reg->idoReqEn = 0;
  reg->idoCplEn = 0;
  reg->ltrEn    = 0;
  reg->obffEn   = 0;

  return pcie_RET_OK;
} /* pciev0_read_devStatCtrl2_reg */


/*****************************************************************************  
 * Combine and write the Device Status and Control Register 2 register
 ****************************************************************************/  
pcieRet_e pciev0_write_devStatCtrl2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDevStatCtrl2Reg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL2_CMPL_TO_DIS, reg->cmplToDis);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEV_STAT_CTRL2_CMPL_TO,     reg->cmplTo);

  baseAddr->DEV_STAT_CTRL2 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_devStatCtrl2_reg */

/*****************************************************************************  
 * Read and split up the Link Control 2 register
 ****************************************************************************/  
pcieRet_e pciev0_read_linkCtrl2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCtrl2Reg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->LINK_CTRL2;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_DE_EMPH,           reg->deEmph);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_POLL_DEEMPH,       reg->pollDeemph);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_CMPL_SOS,          reg->cmplSos);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_ENTR_MOD_COMPL,    reg->entrModCompl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_TX_MARGIN,         reg->txMargin);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_SEL_DEEMPH,        reg->selDeemph);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_HW_AUTO_SPEED_DIS, reg->hwAutoSpeedDis);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_ENTR_COMPL,        reg->entrCompl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_TGT_SPEED,         reg->tgtSpeed);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->complPrstDeemph = 0u;
  reg->eqComplete      = 0u;
  reg->eqPh1           = 0u;
  reg->eqPh2           = 0u;
  reg->eqPh3           = 0u;
  reg->linkEqReq       = 0u;
  reg->downCompPres    = 0u;
  reg->drsMsgRecv      = 0u;

  return pcie_RET_OK;
} /* pciev0_read_linkCtrl2_reg */


/*****************************************************************************  
 * Combine and write the Link Control 2 register
 ****************************************************************************/  
pcieRet_e pciev0_write_linkCtrl2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLinkCtrl2Reg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_DE_EMPH,           reg->deEmph);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_POLL_DEEMPH,       reg->pollDeemph);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_CMPL_SOS,          reg->cmplSos);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_ENTR_MOD_COMPL,    reg->entrModCompl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_TX_MARGIN,         reg->txMargin);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_SEL_DEEMPH,        reg->selDeemph);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_HW_AUTO_SPEED_DIS, reg->hwAutoSpeedDis);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_ENTR_COMPL,        reg->entrCompl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LINK_CTRL2_TGT_SPEED,         reg->tgtSpeed);

  baseAddr->LINK_CTRL2 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_linkCtrl2_reg */


/*****************************************************************************
 **********  PCIe EXTENDED CAPABILITIES  REGISTERS **********************
 ****************************************************************************/

/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************  
 * Read and split up the PCIE Extended Capabilities Header register
 ****************************************************************************/  
pcieRet_e pciev0_read_extCap_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieExtCapReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_EXTCAP;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_EXTCAP_NEXT_CAP,    reg->nextCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_EXTCAP_EXT_CAP_VER, reg->extCapVer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_EXTCAP_EXT_CAP_ID,  reg->extCapID);

  return pcie_RET_OK;
} /* pciev0_read_extCap_reg */


/*****************************************************************************  
 * Read and split up the Uncorrectable Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_uncErr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_UNCERR;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_UR_ERR_ST,    reg->urErrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_ECRC_ERR_ST,  reg->ecrcErrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MTLP_ERR_ST,  reg->mtlpErrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_RCVR_OF_ST,   reg->rcvrOfSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_UCMP_ST,      reg->ucmpSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_CMPL_ABRT_ST, reg->cmplAbrtSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_CMPL_TMOT_ST, reg->cmplTmotSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_FCP_ERR_ST,   reg->fcpErrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_PSND_TLP_ST,  reg->psndTlpSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SRPS_DN_ST,   reg->srpsDnSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_DLP_ERR_ST,   reg->dlpErrSt);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->tlpPrfxBlockedErrSt = 0u;
  reg->intErrSt            = 0u;

  return pcie_RET_OK;
} /* pciev0_read_uncErr_reg */

/*****************************************************************************  
 * Combine and write the Uncorrectable Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_uncErr_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_UR_ERR_ST,    reg->urErrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_ECRC_ERR_ST,  reg->ecrcErrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MTLP_ERR_ST,  reg->mtlpErrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_RCVR_OF_ST,   reg->rcvrOfSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_UCMP_ST,      reg->ucmpSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_CMPL_ABRT_ST, reg->cmplAbrtSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_CMPL_TMOT_ST, reg->cmplTmotSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_FCP_ERR_ST,   reg->fcpErrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_PSND_TLP_ST,  reg->psndTlpSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SRPS_DN_ST,   reg->srpsDnSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_DLP_ERR_ST,   reg->dlpErrSt);

  baseAddr->PCIE_UNCERR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_uncErr_reg */



/*****************************************************************************  
 * Read and split up the Uncorrectable Error Mask register
 ****************************************************************************/  
pcieRet_e pciev0_read_uncErrMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrMaskReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_UNCERR_MASK;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_UR_ERR_MSK,    reg->urErrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_ECRC_ERR_MSK,  reg->ecrcErrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_MTLP_ERR_MSK,  reg->mtlpErrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_RCVR_OF_MSK,   reg->rcvrOfMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_UCMP_MSK,      reg->ucmpMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_CMPL_ABRT_MSK, reg->cmplAbrtMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_CMPL_TMOT_MSK, reg->cmplTmotMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_FCP_ERR_MSK,   reg->fcpErrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_PSND_TLP_MSK,  reg->psndTlpMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_SRPS_DN_MSK,   reg->srpsDnMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_DLP_ERR_MSK,   reg->dlpErrMsk);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->tlpPrfxBlockedErrMsk      = 0u;
  reg->atomicEgressBlockedErrMsk = 0u;
  reg->intErrMsk                 = 0u;

  return pcie_RET_OK;
} /* pciev0_read_uncErrMask_reg */

/*****************************************************************************  
 * Combine and write the Uncorrectable Error Mask register
 ****************************************************************************/  
pcieRet_e pciev0_write_uncErrMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrMaskReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_UR_ERR_MSK,    reg->urErrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_ECRC_ERR_MSK,  reg->ecrcErrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_MTLP_ERR_MSK,  reg->mtlpErrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_RCVR_OF_MSK,   reg->rcvrOfMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_UCMP_MSK,      reg->ucmpMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_CMPL_ABRT_MSK, reg->cmplAbrtMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_CMPL_TMOT_MSK, reg->cmplTmotMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_FCP_ERR_MSK,   reg->fcpErrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_PSND_TLP_MSK,  reg->psndTlpMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_SRPS_DN_MSK,   reg->srpsDnMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_MASK_DLP_ERR_MSK,   reg->dlpErrMsk);

  baseAddr->PCIE_UNCERR_MASK = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_uncErrMask_reg */


/*****************************************************************************  
 * Read and split up the Uncorrectable Error Severity register
 ****************************************************************************/  
pcieRet_e pciev0_read_uncErrSvrty_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrSvrtyReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_UNCERR_SVRTY;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_UR_ERR_SVRTY,    reg->urErrSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_ECRC_ERR_SVRTY,  reg->ecrcErrSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_MTLP_ERR_SVRTY,  reg->mtlpErrSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_RCVR_OF_SVRTY,   reg->rcvrOfSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_UCMP_SVRTY,      reg->ucmpSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_CMPL_ABRT_SVRTY, reg->cmplAbrtSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_CMPL_TMOT_SVRTY, reg->cmplTmotSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_FCP_ERR_SVRTY,   reg->fcpErrSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_PSND_TLP_SVRTY,  reg->psndTlpSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_SRPS_DN_SVRTY,   reg->srpsDnSvrty);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_DLP_ERR_SVRTY,   reg->dlpErrSvrty);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->tlpPrfxBlockedErrSvrty      = 0u;
  reg->atomicEgressBlockedErrSvrty = 0u;
  reg->intErrSvrty                 = 0u;

  return pcie_RET_OK;
} /* pciev0_read_uncErrSvrty_reg */

/*****************************************************************************  
 * Combine and write the Uncorrectable Error Severity register
 ****************************************************************************/  
pcieRet_e pciev0_write_uncErrSvrty_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieUncErrSvrtyReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_UR_ERR_SVRTY,    reg->urErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_ECRC_ERR_SVRTY,  reg->ecrcErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_MTLP_ERR_SVRTY,  reg->mtlpErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_RCVR_OF_SVRTY,   reg->rcvrOfSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_UCMP_SVRTY,      reg->ucmpSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_CMPL_ABRT_SVRTY, reg->cmplAbrtSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_CMPL_TMOT_SVRTY, reg->cmplTmotSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_FCP_ERR_SVRTY,   reg->fcpErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_PSND_TLP_SVRTY,  reg->psndTlpSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_SRPS_DN_SVRTY,   reg->srpsDnSvrty);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_UNCERR_SVRTY_DLP_ERR_SVRTY,   reg->dlpErrSvrty);

  baseAddr->PCIE_UNCERR_SVRTY = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_uncErrSvrty_reg */


/*****************************************************************************  
 * Read and split up the Correctable Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_corErr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCorErrReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_CERR;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_ADV_NFERR_ST, reg->advNFErrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RPLY_TMR_ST,  reg->rplyTmrSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RPLT_RO_ST,   reg->rpltRoSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_BAD_DLLP_ST,  reg->badDllpSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_BAD_TLP_ST,   reg->badTlpSt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RCVR_ERR_ST,  reg->rcvrErrSt);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->hdrLogOverflowErrSt = 0u;
  reg->corrIntErrSt        = 0u;

  return pcie_RET_OK;
} /* pciev0_read_corErr_reg */

/*****************************************************************************  
 * Combine and write the Correctable Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_corErr_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCorErrReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_ADV_NFERR_ST, reg->advNFErrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RPLY_TMR_ST,  reg->rplyTmrSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RPLT_RO_ST,   reg->rpltRoSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_BAD_DLLP_ST,  reg->badDllpSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_BAD_TLP_ST,   reg->badTlpSt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_RCVR_ERR_ST,  reg->rcvrErrSt);

  baseAddr->PCIE_CERR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_corErr_reg */


/*****************************************************************************  
 * Read and split up the Correctable Error Mask register
 ****************************************************************************/  
pcieRet_e pciev0_read_corErrMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCorErrMaskReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_CERR_MASK;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_ADV_NFERR_MSK, reg->advNFErrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RPLY_TMR_MSK,  reg->rplyTmrMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RPLT_RO_MSK,   reg->rpltRoMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_BAD_DLLP_MSK,  reg->badDllpMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_BAD_TLP_MSK,   reg->badTlpMsk);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RCVR_ERR_MSK,  reg->rcvrErrMsk);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->hdrLogOverflowErrMsk = 0u;
  reg->corrIntErrMsk        = 0u;

  return pcie_RET_OK;
} /* pciev0_read_corErrMask_reg */

/*****************************************************************************  
 * Combine and write the Correctable Error Mask register
 ****************************************************************************/  
pcieRet_e pciev0_write_corErrMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieCorErrMaskReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_ADV_NFERR_MSK, reg->advNFErrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RPLY_TMR_MSK,  reg->rplyTmrMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RPLT_RO_MSK,   reg->rpltRoMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_BAD_DLLP_MSK,  reg->badDllpMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_BAD_TLP_MSK,   reg->badTlpMsk);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_CERR_MASK_RCVR_ERR_MSK,  reg->rcvrErrMsk);

  baseAddr->PCIE_CERR_MASK = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_corErrMask_reg */


/*****************************************************************************
 * Read and split up the Advanced Capabilities and Control register
 ****************************************************************************/
pcieRet_e pciev0_read_accr_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAccrReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PCIE_ACCR;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_CHK_EN,  reg->chkEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_CHK_CAP, reg->chkCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_GEN_EN,  reg->genEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_GEN_CAP, reg->genCap);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_FRST_ERR_PTR, reg->erPtr);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->multHdrEn  = 0u;
  reg->multHdrCap = 0u;

  return pcie_RET_OK;
} /* pciev0_read_accr_reg */


/*****************************************************************************
 * Combine and write the Advanced Capabilities and Control register
 ****************************************************************************/
pcieRet_e pciev0_write_accr_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieAccrReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_CHK_EN,  reg->chkEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_CHK_CAP, reg->chkCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_GEN_EN,  reg->genEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_ECRC_GEN_CAP, reg->genCap);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PCIE_ACCR_FRST_ERR_PTR, reg->erPtr);

  baseAddr->PCIE_ACCR = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_accr_reg */

/*****************************************************************************  
 * Read and split up the Header Log register
 ****************************************************************************/  
pcieRet_e pciev0_read_hdrLog_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieHdrLogReg_t *reg,
  int32_t              regNum
)
{
  reg->raw = reg->hdrDW = baseAddr->HDR_LOG[regNum];

  return pcie_RET_OK;
} /* pciev0_read_hdrLog_reg */


/*****************************************************************************  
 * Read and split up the Root Error Command register
 ****************************************************************************/  
pcieRet_e pciev0_read_rootErrCmd_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRootErrCmdReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->RC_ERR_CMD;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_FERR_RPT_EN,  reg->ferrRptEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_NFERR_RPT_EN, reg->nferrRptEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_CERR_RPT_EN,  reg->cerrRptEn);

  return pcie_RET_OK;
} /* pciev0_read_rootErrCmd_reg */

/*****************************************************************************  
 * Combine and write the Root Error Command register
 ****************************************************************************/  
pcieRet_e pciev0_write_rootErrCmd_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRootErrCmdReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_FERR_RPT_EN,  reg->ferrRptEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_NFERR_RPT_EN, reg->nferrRptEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_CMD_CERR_RPT_EN,  reg->cerrRptEn);

  baseAddr->RC_ERR_CMD = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_rootErrCmd_reg */


/*****************************************************************************  
 * Read and split up the Root Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_rootErrSt_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRootErrStReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->RC_ERR_ST;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_AER_INT_MSG, reg->aerIntMsg);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_FERR_RCV,    reg->ferrRcv);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_NFERR,       reg->nfErr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_UNCOR_FATAL, reg->uncorFatal);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_MULT_FNF,    reg->multFnf);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_ERR_FNF,     reg->errFnf);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_MULT_COR,    reg->multCor);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_CORR_ERR,    reg->corrErr);

  return pcie_RET_OK;
} /* pciev0_read_rootErrSt_reg */

/*****************************************************************************  
 * Combine and write the Root Error Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_rootErrSt_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieRootErrStReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_AER_INT_MSG, reg->aerIntMsg);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_FERR_RCV,    reg->ferrRcv);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_NFERR,       reg->nfErr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_UNCOR_FATAL, reg->uncorFatal);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_MULT_FNF,    reg->multFnf);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_ERR_FNF,     reg->errFnf);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_MULT_COR,    reg->multCor);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_RC_ERR_ST_CORR_ERR,    reg->corrErr);

  baseAddr->RC_ERR_ST = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_rootErrSt_reg */


/*****************************************************************************  
 * Read and split up the Error Source Identification register
 ****************************************************************************/  
pcieRet_e pciev0_read_errSrcID_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieErrSrcIDReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->ERR_SRC_ID;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ERR_SRC_ID_FNF_SRC_ID,  reg->fnfSrcID);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ERR_SRC_ID_CORR_SRC_ID, reg->corrSrcID);

  return pcie_RET_OK;
} /* pciev0_read_errSrcID_reg */


/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **********************
 ****************************************************************************/


/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************  
 * Read and split up the Ack Latency and Replay Timer register
 ****************************************************************************/  
pcieRet_e pciev0_read_plAckTimer_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlAckTimerReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PL_ACKTIMER;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_ACKTIMER_RPLY_LIMT,   reg->rplyLmt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_ACKTIMER_RND_TRP_LMT, reg->rndTrpLmt);

  return pcie_RET_OK;
} /* pciev0_read_plAckTimer_reg */

/*****************************************************************************  
 * Combine and write the Ack Latency and Replay Timer register
 ****************************************************************************/  
pcieRet_e pciev0_write_plAckTimer_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlAckTimerReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_ACKTIMER_RPLY_LIMT,   reg->rplyLmt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_ACKTIMER_RND_TRP_LMT, reg->rndTrpLmt);

  baseAddr->PL_ACKTIMER = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_plAckTimer_reg */


/*****************************************************************************  
 * Read and split up the Other Message register
 ****************************************************************************/  
pcieRet_e pciev0_read_plOMsg_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlOMsgReg_t *reg 
)
{
  reg->raw = reg->oMsg = baseAddr->PL_OMSG;

  return pcie_RET_OK;
} /* pciev0_read_plOMsg_reg */

/*****************************************************************************  
 * Combine and write the Other Message register
 ****************************************************************************/  
pcieRet_e pciev0_write_plOMsg_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlOMsgReg_t *reg 
)
{
  baseAddr->PL_OMSG = reg->raw = reg->oMsg;

  return pcie_RET_OK;
} /* pciev0_write_plOMsg_reg */


/*****************************************************************************  
 * Read and split up the Port Force Link register
 ****************************************************************************/  
pcieRet_e pciev0_read_plForceLink_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlForceLinkReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PL_FORCE_LINK;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LPE_CNT,    reg->lpeCnt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LNK_STATE,  reg->lnkState);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_FORCE_LINK, reg->forceLink);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LINK_NUM,   reg->linkNum);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->forcedLtssmState = 0;
  reg->doDeskewForSris = 0u;

  return pcie_RET_OK;
} /* pciev0_read_plForceLink_reg */

/*****************************************************************************  
 * Combine and write the Port Force Link register
 ****************************************************************************/  
pcieRet_e pciev0_write_plForceLink_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pciePlForceLinkReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LPE_CNT,    reg->lpeCnt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LNK_STATE,  reg->lnkState);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_FORCE_LINK, reg->forceLink);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_FORCE_LINK_LINK_NUM,   reg->linkNum);

  baseAddr->PL_FORCE_LINK = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_plForceLink_reg */


/*****************************************************************************  
 * Read and split up the Ack Frequency register
 ****************************************************************************/  
pcieRet_e pciev0_read_ackFreq_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieAckFreqReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->ACK_FREQ;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_ASPM_L1,           reg->aspmL1);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_L1_ENTRY_LATENCY,  reg->l1EntryLatency);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_L0S_ENTRY_LATENCY, reg->l0sEntryLatency);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_COMM_NFTS,         reg->commNFts);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_NFTS,              reg->nFts);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_ACK_FREQ,          reg->ackFreq);

  return pcie_RET_OK;
} /* pciev0_read_ackFreq_reg */

/*****************************************************************************  
 * Combine and write the Ack Frequency register
 ****************************************************************************/  
pcieRet_e pciev0_write_ackFreq_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieAckFreqReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_ASPM_L1,           reg->aspmL1);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_L1_ENTRY_LATENCY,  reg->l1EntryLatency);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_L0S_ENTRY_LATENCY, reg->l0sEntryLatency);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_COMM_NFTS,         reg->commNFts);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_NFTS,              reg->nFts);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_ACK_FREQ_ACK_FREQ,          reg->ackFreq);

  baseAddr->ACK_FREQ = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_ackFreq_reg */


/*****************************************************************************
 * Read and split up the Port Link Control register
 ****************************************************************************/
pcieRet_e pciev0_read_lnkCtrl_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLnkCtrlReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PL_LINK_CTRL;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LNK_MODE,  reg->lnkMode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LNK_RATE,  reg->lnkRate);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_FLNK_MODE, reg->fLnkMode);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_DLL_EN,    reg->dllEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_RST_ASRT,  reg->rstAsrt);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LPBK_EN,   reg->lpbkEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_SCRM_DIS,  reg->scrmDis);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_OMSG_REQ,  reg->msgReq);

  /* Set unused fields to 0 (only used by rev 1 hw) */
  reg->crosslinkAct = 0;
  reg->crosslinkEn  = 0;

  return pcie_RET_OK;
} /* pciev0_read_lnkCtrl_reg */

/*****************************************************************************
 * Combine and write the Port Link Control register
 ****************************************************************************/
pcieRet_e pciev0_write_lnkCtrl_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieLnkCtrlReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LNK_MODE,  reg->lnkMode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LNK_RATE,  reg->lnkRate);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_FLNK_MODE, reg->fLnkMode);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_DLL_EN,    reg->dllEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_RST_ASRT,  reg->rstAsrt);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_LPBK_EN,   reg->lpbkEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_SCRM_DIS,  reg->scrmDis);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_LINK_CTRL_OMSG_REQ,  reg->msgReq);

  baseAddr->PL_LINK_CTRL = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_lnkCtrl_reg */


/*****************************************************************************  
 * Read and split up the Lane Skew register
 ****************************************************************************/  
pcieRet_e pciev0_read_laneSkew_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLaneSkewReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->LANE_SKEW;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_L2L_DESKEW,  reg->l2Deskew);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_ACK_DISABLE, reg->ackDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_FC_DISABLE,  reg->fcDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_LANE_SKEW,   reg->laneSkew);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->implementNumLanes = 0;

  return pcie_RET_OK;
} /* pciev0_read_laneSkew_reg */

/*****************************************************************************  
 * Combine and write the Lane Skew register
 ****************************************************************************/  
pcieRet_e pciev0_write_laneSkew_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieLaneSkewReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_L2L_DESKEW,  reg->l2Deskew);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_ACK_DISABLE, reg->ackDisable);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_FC_DISABLE,  reg->fcDisable);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_LANE_SKEW_LANE_SKEW,   reg->laneSkew);

  baseAddr->LANE_SKEW = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_laneSkew_reg */


/*****************************************************************************  
 * Read and split up the Symbol Number register
 ****************************************************************************/  
pcieRet_e pciev0_read_symNum_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSymNumReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SYM_NUM;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_MAX_FUNC,          reg->maxFunc);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_FCWATCH_TIMER,     reg->fcWatchTimer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_ACK_LATENCY_TIMER, reg->ackLatencyTimer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_REPLAY_TIMER,      reg->replayTimer);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_SKP_COUNT,         reg->skpCount);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_NUM_TS2_SYMBOLS,   reg->numTs2Symbols);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_TS_COUNT,          reg->tsCount);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->fastLinkScalingFactor = 0u;

  return pcie_RET_OK;
} /* pciev0_read_symNum_reg */

/*****************************************************************************  
 * Combine and write the Symbol Number register
 ****************************************************************************/  
pcieRet_e pciev0_write_symNum_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSymNumReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_MAX_FUNC,          reg->maxFunc);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_FCWATCH_TIMER,     reg->fcWatchTimer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_ACK_LATENCY_TIMER, reg->ackLatencyTimer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_REPLAY_TIMER,      reg->replayTimer);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_SKP_COUNT,         reg->skpCount);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_NUM_TS2_SYMBOLS,   reg->numTs2Symbols);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYM_NUM_TS_COUNT,          reg->tsCount);

  baseAddr->SYM_NUM = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_symNum_reg */


/*****************************************************************************  
 * Read and split up the Symbol Timer and Filter Mask register
 ****************************************************************************/  
pcieRet_e pciev0_read_symTimerFltMask_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSymTimerFltMaskReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SYMTIMER_FLTMASK;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CFG_DROP,        reg->f1CfgDrop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_IO_DROP,         reg->f1IoDrop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_MSG_DROP,        reg->f1MsgDrop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_ECRC_DROP,   reg->f1CplEcrcDrop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_ECRC_DROP,       reg->f1EcrcDrop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_LEN_TEST,    reg->f1CplLenTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_ATTR_TEST,   reg->f1CplAttrTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_TC_TEST,     reg->f1CplTcTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_FUNC_TEST,   reg->f1CplFuncTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_REQID_TEST,  reg->f1CplReqIDTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_TAGERR_TEST, reg->f1CplTagErrTest);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_LOCKED_RD_AS_UR, reg->f1LockedRdAsUr);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CFG1_RE_AS_US,   reg->f1Cfg1ReAsUs);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_OUT_OF_BAR,   reg->f1UrOutOfBar);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_POISON,       reg->f1UrPoison);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_FUN_MISMATCH, reg->f1UrFunMismatch);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_FC_WDOG_DISABLE,    reg->fcWdogDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_SKP_VALUE,          reg->skpValue);

  return pcie_RET_OK;
} /* pciev0_read_symTimerFltMask_reg */

/*****************************************************************************  
 * Combine and write the Symbol Timer and Filter Mask register
 ****************************************************************************/  
pcieRet_e pciev0_write_symTimerFltMask_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieSymTimerFltMaskReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CFG_DROP,        reg->f1CfgDrop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_IO_DROP,         reg->f1IoDrop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_MSG_DROP,        reg->f1MsgDrop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_ECRC_DROP,   reg->f1CplEcrcDrop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_ECRC_DROP,       reg->f1EcrcDrop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_LEN_TEST,    reg->f1CplLenTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_ATTR_TEST,   reg->f1CplAttrTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_TC_TEST,     reg->f1CplTcTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_FUNC_TEST,   reg->f1CplFuncTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_REQID_TEST,  reg->f1CplReqIDTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CPL_TAGERR_TEST, reg->f1CplTagErrTest);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_LOCKED_RD_AS_UR, reg->f1LockedRdAsUr);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_CFG1_RE_AS_US,   reg->f1Cfg1ReAsUs);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_OUT_OF_BAR,   reg->f1UrOutOfBar);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_POISON,       reg->f1UrPoison);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_F1_UR_FUN_MISMATCH, reg->f1UrFunMismatch);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_FC_WDOG_DISABLE,    reg->fcWdogDisable);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_SYMTIMER_FLTMASK_SKP_VALUE,          reg->skpValue);

  baseAddr->SYMTIMER_FLTMASK = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_symTimerFltMask_reg */


/*****************************************************************************  
 * Read and split up the Filter Mask 2 register
 ****************************************************************************/  
pcieRet_e pciev0_read_fltMask2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieFltMask2Reg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->FLT_MASK2;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_FLUSH_REQ,  reg->flushReq);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_DLLP_ABORT, reg->dllpAbort);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_VMSG1_DROP, reg->vmsg1Drop);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_VMSG0_DROP, reg->vmsg0Drop);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->dropPRS      = 0u;
  reg->unmaskTD     = 0u;
  reg->unmaskUrPOIS = 0u;
  reg->dropLN       = 0u;

  return pcie_RET_OK;
} /* pciev0_read_fltMask2_reg */

/*****************************************************************************  
 * Combine and write the Filter Mask 2 register
 ****************************************************************************/  
pcieRet_e pciev0_write_fltMask2_reg
(
  CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieFltMask2Reg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_FLUSH_REQ,  reg->flushReq);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_DLLP_ABORT, reg->dllpAbort);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_VMSG1_DROP, reg->vmsg1Drop);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_FLT_MASK2_VMSG0_DROP, reg->vmsg0Drop);

  baseAddr->FLT_MASK2 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_fltMask2_reg */


/*****************************************************************************
 * Read and split up the Debug 0 register
 ****************************************************************************/
pcieRet_e pciev0_read_debug0_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieDebug0Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->DEBUG0;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_TS_LINK_CTRL,  reg->tsLnkCtrl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_TS_LANE_K237,  reg->tsLaneK237);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_TS_LINK_K237,  reg->tsLinkK237);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_RCVD_IDLE0,    reg->rcvdIdle0);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_RCVD_IDLE1,    reg->rcvdIdle1);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_PIPE_TXDATA,   reg->pipeTxData);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_PIPE_TXDATAK,  reg->pipeTxDataK);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_TXB_SKIP_TX,   reg->skipTx);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG0_LTSSM_STATE,   reg->ltssmState);

  return pcie_RET_OK;
} /* pciev0_read_debug0_reg */


/*****************************************************************************  
 * Read and split up the Debug 1 register
 ****************************************************************************/  
pcieRet_e pciev0_read_debug1_reg
(
  const CSL_Pcie_cfg_space_endpointRegs   *baseAddr, 
  pcieDebug1Reg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->DEBUG1;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_SCRAMBLER_DISABLE,  reg->scramblerDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_LINK_DISABLE,       reg->linkDisable);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_LINK_IN_TRAINING,   reg->linkInTraining);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RCVR_REVRS_POL_EN,  reg->rcvrRevrsPolEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_TRAINING_RST_N,     reg->trainingRstN);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_PIPE_TXDETECTRX_LB, reg->pipeTxdetectrxLb);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_PIPE_TXELECIDLE,    reg->pipeTxelecidle);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_PIPE_TXCOMPLIANCE,  reg->pipeTxcompliance);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_APP_INIT_RST,       reg->appInitRst);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RMLH_TS_LINK_NUM,   reg->rmlhTsLinkNum);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_XMLH_LINK_UP,       reg->xmlhLinkUp);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RMLH_INSKIP_RCV,    reg->rmlhInskipRcv);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RMLH_TS1_RCVD,      reg->rmlhTs1Rcvd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RMLH_TS2_RCVD,      reg->rmlhTs2Rcvd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_DEBUG1_RMLH_RCVD_LANE_REV, reg->rmlhRcvdLaneRev);

  return pcie_RET_OK;
} /* pciev0_read_debug1_reg */

/*****************************************************************************
 * Read and split up the Gen 2 register
 ****************************************************************************/
pcieRet_e pciev0_read_gen2_reg
(
  const CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieGen2Reg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PL_GEN2;

  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_DEEMPH,       reg->deemph);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_CFG_TX_CMPL,  reg->txCmpl);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_CFG_TX_SWING, reg->txSwing);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_DIR_SPD,      reg->dirSpd);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_LN_EN,        reg->lnEn);
  pcie_getbits(val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_NUM_FTS,      reg->numFts);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->preDetLane      = 0u;
  reg->autoFlipEn      = 0u;
  reg->gen1EiInference = 0u;

  return pcie_RET_OK;
} /* pciev0_read_gen2_reg */

/*****************************************************************************
 * Combine and write the Gen 2 register
 ****************************************************************************/
pcieRet_e pciev0_write_gen2_reg
(
  CSL_Pcie_cfg_space_endpointRegs  *baseAddr,  
  pcieGen2Reg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_DEEMPH,       reg->deemph);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_CFG_TX_CMPL,  reg->txCmpl);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_CFG_TX_SWING, reg->txSwing);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_DIR_SPD,      reg->dirSpd);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_LN_EN,        reg->lnEn);
  pcie_setbits(new_val, CSL_PCIE_CFG_SPACE_ENDPOINT_PL_GEN2_NUM_FTS,      reg->numFts);

  baseAddr->PL_GEN2 = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_gen2_reg */

/* Nothing past this point */

