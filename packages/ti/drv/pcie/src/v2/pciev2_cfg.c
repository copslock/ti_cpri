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
 *  File Name: pciev2_cfg.c
 *
 *  Processing/configuration functions for the PCIe Configuration Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v2/pcieloc.h>


/*****************************************************************************
 **********  PCIe CONFIG REGISTERS COMMON TO TYPE0 AND TYPE1  *****************
 ****************************************************************************/

/* pack/unpack for backwards compatibility */
#define PCIE_REV2_CLASSCODE_MASK ( \
           CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_PROGRAM_INTERFACE_MASK | \
           CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_SUBCLASS_CODE_MASK | \
           CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_BASE_CLASS_CODE_MASK)
#define PCIE_REV2_CLASSCODE_SHIFT (CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_PROGRAM_INTERFACE_SHIFT)

#define PCIE_REV2_TPH_CMPLT_SUPPORT_MASK ( \
           CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_MASK | \
           CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_1_MASK)
#define PCIE_REV2_TPH_CMPLT_SUPPORT_SHIFT (CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_TPH_CMPLT_SUPPORT_0_SHIFT)


/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same register layout, in the same location.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev2_read_vndDevId_reg
(
  volatile const uint32_t *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t       *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEVICE_VENDORID;

  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_VENDOR_ID, swReg->vndId);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_DEVICE_ID, swReg->devId);

  return pcie_RET_OK;
} /* pciev2_read_vndDevId_reg */

/*****************************************************************************
 * Combine and write the Vendor and Device Identification register
 ****************************************************************************/
pcieRet_e pciev2_write_vndDevId_reg
(
  volatile uint32_t *hwReg_DEVICE_VENDORID,
  pcieVndDevIdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_VENDOR_ID, swReg->vndId);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_ID_VENDOR_ID_REG_PCI_TYPE0_DEVICE_ID, swReg->devId);

  *hwReg_DEVICE_VENDORID = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_vndDevId_reg */



/*****************************************************************************
 * Read and split up the Status and Command register
 ****************************************************************************/
pcieRet_e pciev2_read_statusCmd_reg
(
  volatile const uint32_t *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t      *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_STATUS_COMMAND_REGISTER;

  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_IO_EN,                   swReg->ioSp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_MEM_SPACE_EN,            swReg->memSp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_BUS_MASTER_EN,           swReg->busMs);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SPECIAL_CYCLE_OPERATION, swReg->specCycleEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_MWI_ENABLE,               swReg->memWrInva);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_VGA_PALETTE_SNOOP,        swReg->vgaSnoop);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_PARITY_ERR_EN,           swReg->resp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_IDSEL_STEPPING,           swReg->idselCtrl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SERREN,                  swReg->serrEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_INT_EN,                  swReg->dis);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_INT_STATUS,                        swReg->stat);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_CAP_LIST,                          swReg->capList);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_66MHZ_CAP,                    swReg->c66MhzCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_B2B_CAP,                      swReg->fastB2B);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_MASTER_DPE,                        swReg->parError);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DEV_SEL_TIMING,                    swReg->devSelTime);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT,             swReg->sigTgtAbort);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_TARGET_ABORT,                 swReg->tgtAbort);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_MASTER_ABORT,                 swReg->mstAbort);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_SYS_ERR,                  swReg->sysError);
  pcie_getbits(val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DETECTED_PARITY_ERR,               swReg->parity);

  return pcie_RET_OK;
} /* pciev2_read_statusCmd_reg */

/*****************************************************************************
 * Combine and write the Status and Command register
 ****************************************************************************/
pcieRet_e pciev2_write_statusCmd_reg
(
  volatile uint32_t  *hwReg_STATUS_COMMAND_REGISTER,
  pcieStatusCmdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_IO_EN,                   swReg->ioSp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_MEM_SPACE_EN,            swReg->memSp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_BUS_MASTER_EN,           swReg->busMs);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SPECIAL_CYCLE_OPERATION, swReg->specCycleEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_MWI_ENABLE,               swReg->memWrInva);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_VGA_PALETTE_SNOOP,        swReg->vgaSnoop);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_PARITY_ERR_EN,           swReg->resp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE_IDSEL_STEPPING,           swReg->idselCtrl);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_SERREN,                  swReg->serrEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_PCI_TYPE0_INT_EN,                  swReg->dis);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_INT_STATUS,                        swReg->stat);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_CAP_LIST,                          swReg->capList);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_66MHZ_CAP,                    swReg->c66MhzCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_FAST_B2B_CAP,                      swReg->fastB2B);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_MASTER_DPE,                        swReg->parError);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DEV_SEL_TIMING,                    swReg->devSelTime);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_TARGET_ABORT,             swReg->sigTgtAbort);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_TARGET_ABORT,                 swReg->tgtAbort);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_RCVD_MASTER_ABORT,                 swReg->mstAbort);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_SIGNALED_SYS_ERR,                  swReg->sysError);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_STATUS_COMMAND_REG_DETECTED_PARITY_ERR,               swReg->parity);

  *hwReg_STATUS_COMMAND_REGISTER = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_statusCmd_reg */

/*****************************************************************************
 * Read and split up the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev2_read_revId_reg
(
  volatile const uint32_t *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t          *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_CLASSCODE_REVISIONID;

  pcie_getbits(val, PCIE_REV2_CLASSCODE,                                 swReg->classCode);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_REVISION_ID, swReg->revId);

  return pcie_RET_OK;
} /* pciev2_read_revId_reg */

/*****************************************************************************
 * Combine and write the Class Code and Revision ID register
 ****************************************************************************/
pcieRet_e pciev2_write_revId_reg
(
  volatile uint32_t *hwReg_CLASSCODE_REVISIONID,
  pcieRevIdReg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, PCIE_REV2_CLASSCODE,                                 swReg->classCode);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CLASS_CODE_REVISION_ID_REVISION_ID, swReg->revId);

  *hwReg_CLASSCODE_REVISIONID = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_revId_reg */


/* MSI capabilities*/
/*****************************************************************************
 * Read and split up the Message Signaled Interrupt Capability register
 ****************************************************************************/
pcieRet_e pciev2_read_msiCap_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiCapReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PCI_MSI_CAP_ID_NEXT_CTRL_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID,           swReg->capId);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET,  swReg->nextCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE,           swReg->msiEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP, swReg->multMsgCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN,  swReg->multMsgEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP,  swReg->en64bit);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT,          swReg->pvmEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP,     swReg->extDataCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN,      swReg->extDataEn);

  return pcie_RET_OK;
} /* pciev2_read_msiCap_reg */

/*****************************************************************************
 * Combine and write the Message Signaled Interrupt Capability register
 ****************************************************************************/
pcieRet_e pciev2_write_msiCap_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiCapReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_ID,           swReg->capId);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_CAP_NEXT_OFFSET,  swReg->nextCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_ENABLE,           swReg->msiEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_CAP, swReg->multMsgCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_MULTIPLE_MSG_EN,  swReg->multMsgEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_64_BIT_ADDR_CAP,  swReg->en64bit);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_PVM_SUPPORT,          swReg->pvmEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_CAP,     swReg->extDataCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCI_MSI_CAP_ID_NEXT_CTRL_REG_PCI_MSI_EXT_DATA_EN,      swReg->extDataEn);

  baseAddr->PCI_MSI_CAP_ID_NEXT_CTRL_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiCap_reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
pcieRet_e pciev2_read_msiLo32_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiLo32Reg_t           *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_04H_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H, swReg->addr);

  return pcie_RET_OK;
} /* pciev2_read_epcfgdbicsMsiAddrL32_reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req lower 32 bits register
 ****************************************************************************/
pcieRet_e pciev2_write_msiLo32_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiLo32Reg_t     *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_04H_REG_PCI_MSI_CAP_OFF_04H, swReg->addr);

  baseAddr->MSI_CAP_OFF_04H_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epcfgdbicsMsiAddrL32_reg */

/*****************************************************************************
 * Read and split up the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
pcieRet_e pciev2_read_msiUp32_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiUp32Reg_t           *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_08H_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H, swReg->addr);

  return pcie_RET_OK;
} /* pciev2_read_epcfgdbicsMsiAddrU32_reg */

/*****************************************************************************
 * Combine and write the Address of MSI write TLP req upper 32 bits register
 ****************************************************************************/
pcieRet_e pciev2_write_msiUp32_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiUp32Reg_t     *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_08H_REG_PCI_MSI_CAP_OFF_08H, swReg->addr);

  baseAddr->MSI_CAP_OFF_08H_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epcfgdbicsMsiAddrU32_reg */

/*****************************************************************************
 * Read and split up the Data of MSI write TLP req register
 ****************************************************************************/
pcieRet_e pciev2_read_msiData_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiDataReg_t           *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_0CH_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH, swReg->data);

  return pcie_RET_OK;
} /* pciev2_read_epcfgdbicsMsiData_reg */

/*****************************************************************************
 * Combine and write the Data of MSI write TLP req register
 ****************************************************************************/
pcieRet_e pciev2_write_msiData_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiDataReg_t     *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_0CH_REG_PCI_MSI_CAP_OFF_0CH, swReg->data);

  baseAddr->MSI_CAP_OFF_0CH_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epcfgdbicsMsiData_reg */

/*****************************************************************************
 * Read and split up the Data of MSI CAP OFF 10H Reg
 ****************************************************************************/
pcieRet_e pciev2_read_msiCapOff10H_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiCapOff10HReg_t      *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_10H_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H, swReg->data);

  return pcie_RET_OK;
} /* pciev2_read_msiCapOff10H_reg */

/*****************************************************************************
 * Combine and write the Data of MSI CAP OFF 10H register
 ****************************************************************************/
pcieRet_e pciev2_write_msiCapOff10H_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff10HReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_10H_REG_PCI_MSI_CAP_OFF_10H, swReg->data);

  baseAddr->MSI_CAP_OFF_10H_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiCapOff10H_reg */

/*****************************************************************************
 * Read and split up the Data of MSI CAP OFF 14H Reg
 ****************************************************************************/
pcieRet_e pciev2_read_msiCapOff14H_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiCapOff14HReg_t      *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CAP_OFF_14H_REG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H, swReg->data);

  return pcie_RET_OK;
} /* pciev2_read_msiCapOff14H_reg */

/*****************************************************************************
 * Combine and write the Data of MSI CAP OFF 14H register
 ****************************************************************************/
pcieRet_e pciev2_write_msiCapOff14H_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiCapOff14HReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CAP_OFF_14H_REG_PCI_MSI_CAP_OFF_14H, swReg->data);

  baseAddr->MSI_CAP_OFF_14H_REG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiCapOff14H_reg */


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
pcieRet_e pciev2_read_pciesCap_reg
(
  volatile const uint32_t *hwReg_PCIE_CAP,
  pciePciesCapReg_t       *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_PCIE_CAP;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID,        swReg->capId);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR,  swReg->nextCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG,       swReg->pcieCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE, swReg->dportType);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP,      swReg->sltImplN);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM,   swReg->intMsg);

  return pcie_RET_OK;
} /* pciev2_read_pciesCap_reg */


/*****************************************************************************
 * Combine and write the PCIE Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_write_pciesCap_reg
(
  volatile uint32_t *hwReg_PCIE_CAP,
  pciePciesCapReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_ID,        swReg->capId);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_NEXT_PTR,  swReg->nextCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_CAP_REG,       swReg->pcieCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_DEV_PORT_TYPE, swReg->dportType);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_SLOT_IMP,      swReg->sltImplN);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PCIE_CAP_ID_PCIE_NEXT_CAP_PTR_PCIE_CAP_REG_PCIE_INT_MSG_NUM,   swReg->intMsg);

  *hwReg_PCIE_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pciesCap_reg */

/*****************************************************************************
 * Read and split up the Device Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_read_deviceCap_reg
(
  volatile const uint32_t *hwReg_DEV_CAP,
  pcieDeviceCapReg_t      *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP;

  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE,       swReg->maxPayldSz);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT,   swReg->phantomFld);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP,           swReg->extTagFld);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L0S_ACCPT_LATENCY,   swReg->l0Latency);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L1_ACCPT_LATENCY,    swReg->l1Latency);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT,  swReg->errRpt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_VALUE, swReg->pwrLimitValue);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_SCALE, swReg->pwrLimitScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_FLR_CAP,                swReg->flrEn);

  return pcie_RET_OK;
} /* pciev2_read_deviceCap_reg */


/*****************************************************************************
 * Combine and write the Device Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_write_deviceCap_reg
(
  volatile uint32_t  *hwReg_DEV_CAP,
  pcieDeviceCapReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_MAX_PAYLOAD_SIZE,       swReg->maxPayldSz);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_PHANTOM_FUNC_SUPPORT,   swReg->phantomFld);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EXT_TAG_SUPP,           swReg->extTagFld);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L0S_ACCPT_LATENCY,   swReg->l0Latency);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_EP_L1_ACCPT_LATENCY,    swReg->l1Latency);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_ROLE_BASED_ERR_REPORT,  swReg->errRpt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_VALUE, swReg->pwrLimitValue);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_CAP_SLOT_PWR_LMT_SCALE, swReg->pwrLimitScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES_REG_PCIE_CAP_FLR_CAP,                swReg->flrEn);

  *hwReg_DEV_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_deviceCap_reg */

/*****************************************************************************
 * Read and split up the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_read_devStatCtrl_reg
(
  volatile const uint32_t *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t    *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN,       swReg->corErRp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN,  swReg->nFatalErRp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN,      swReg->fatalErRp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN,     swReg->reqRp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER,             swReg->relaxed);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS,      swReg->maxPayld);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN,               swReg->xtagEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN,          swReg->phantomEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN,          swReg->auxPwrEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP,              swReg->noSnoop);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE,        swReg->maxSz);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR,             swReg->initFLR);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED,        swReg->corrEr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED,   swReg->nFatalEr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED,       swReg->fatalEr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED, swReg->rqDet);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED,       swReg->auxPwr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING,            swReg->tpend);

  return pcie_RET_OK;
} /* pciev2_read_devStatCtrl_reg */


/*****************************************************************************
 * Combine and write the Device Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_write_devStatCtrl_reg
(
  volatile uint32_t    *hwReg_DEV_CAS,
  pcieDevStatCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_REPORT_EN,       swReg->corErRp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_REPORT_EN,  swReg->nFatalErRp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_REPORT_EN,      swReg->fatalErRp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORT_REQ_REP_EN,     swReg->reqRp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_REL_ORDER,             swReg->relaxed);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_PAYLOAD_SIZE_CS,      swReg->maxPayld);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EXT_TAG_EN,               swReg->xtagEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_PHANTOM_FUNC_EN,          swReg->phantomEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_PM_EN,          swReg->auxPwrEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_EN_NO_SNOOP,              swReg->noSnoop);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_MAX_READ_REQ_SIZE,        swReg->maxSz);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_INITIATE_FLR,             swReg->initFLR);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_CORR_ERR_DETECTED,        swReg->corrEr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_NON_FATAL_ERR_DETECTED,   swReg->nFatalEr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_FATAL_ERR_DETECTED,       swReg->fatalEr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_UNSUPPORTED_REQ_DETECTED, swReg->rqDet);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_AUX_POWER_DETECTED,       swReg->auxPwr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL_DEVICE_STATUS_PCIE_CAP_TRANS_PENDING,            swReg->tpend);

  *hwReg_DEV_CAS = swReg->raw = new_val;
  return pcie_range_check_return;
} /* pciev2_write_devStatCtrl_reg */

/*****************************************************************************
 * Read and split up the Link Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_read_linkCap_reg
(
  volatile const uint32_t *hwReg_LNK_CAP,
  pcieLinkCapReg_t        *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAP;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED,               swReg->maxLinkSpeed);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH,               swReg->maxLinkWidth);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT, swReg->asLinkPm);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY,             swReg->losExitLat);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY,              swReg->l1ExitLat);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN,              swReg->clkPwrMgmt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP,    swReg->downErrRepCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP,           swReg->dllRepCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP,              swReg->bwNotifyCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE,          swReg->aspmOptComp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM,                     swReg->portNum);

  return pcie_RET_OK;
} /* pciev2_read_linkCap_reg */


/*****************************************************************************
 * Combine and write the Link Capabilities register
 ****************************************************************************/
pcieRet_e pciev2_write_linkCap_reg
(
  volatile uint32_t *hwReg_LNK_CAP,
  pcieLinkCapReg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_SPEED,               swReg->maxLinkSpeed);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_MAX_LINK_WIDTH,               swReg->maxLinkWidth);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_SUPPORT, swReg->asLinkPm);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L0S_EXIT_LATENCY,             swReg->losExitLat);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_L1_EXIT_LATENCY,              swReg->l1ExitLat);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_CLOCK_POWER_MAN,              swReg->clkPwrMgmt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_SURPRISE_DOWN_ERR_REP_CAP,    swReg->downErrRepCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_DLL_ACTIVE_REP_CAP,           swReg->dllRepCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_LINK_BW_NOT_CAP,              swReg->bwNotifyCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_ASPM_OPT_COMPLIANCE,          swReg->aspmOptComp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES_REG_PCIE_CAP_PORT_NUM,                     swReg->portNum);

  *hwReg_LNK_CAP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_linkCap_reg */


/*****************************************************************************
 * Read and split up the Link Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_read_linkStatCtrl_reg
(
  volatile const uint32_t *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t   *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, swReg->activeLinkPm);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB,                          swReg->rcb);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE,                 swReg->linkDisable);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK,                 swReg->retrainLink);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG,            swReg->commonClkCfg);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH,               swReg->extSync);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN,             swReg->clkPwrMgmtEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE,        swReg->hwAutoWidthDis);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN,           swReg->linkBwMgmtIntEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN,          swReg->linkBwIntEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL,        swReg->drsSigCtrl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED,                   swReg->linkSpeed);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH,              swReg->negotiatedLinkWd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING,                swReg->linkTraining);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG,              swReg->slotClkCfg);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE,                   swReg->dllActive);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS,           swReg->linkBwMgmtStatus);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS,          swReg->linkBwStatus);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->undef = 0u;

  return pcie_RET_OK;
} /* pciev2_read_linkStatCtrl_reg */


/*****************************************************************************
 * Combine and write the Link Status and Control register
 ****************************************************************************/
pcieRet_e pciev2_write_linkStatCtrl_reg
(
  volatile uint32_t     *hwReg_LNK_CAS,
  pcieLinkStatCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_ACTIVE_STATE_LINK_PM_CONTROL, swReg->activeLinkPm);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RCB,                          swReg->rcb);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_DISABLE,                 swReg->linkDisable);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_RETRAIN_LINK,                 swReg->retrainLink);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_COMMON_CLK_CONFIG,            swReg->commonClkCfg);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EXTENDED_SYNCH,               swReg->extSync);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_EN_CLK_POWER_MAN,             swReg->clkPwrMgmtEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_HW_AUTO_WIDTH_DISABLE,        swReg->hwAutoWidthDis);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_INT_EN,           swReg->linkBwMgmtIntEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_INT_EN,          swReg->linkBwIntEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DRS_SIGNALING_CONTROL,        swReg->drsSigCtrl);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_SPEED,                   swReg->linkSpeed);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_NEGO_LINK_WIDTH,              swReg->negotiatedLinkWd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_TRAINING,                swReg->linkTraining);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_SLOT_CLK_CONFIG,              swReg->slotClkCfg);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_DLL_ACTIVE,                   swReg->dllActive);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_BW_MAN_STATUS,           swReg->linkBwMgmtStatus);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL_LINK_STATUS_REG_PCIE_CAP_LINK_AUTO_BW_STATUS,          swReg->linkBwStatus);

  *hwReg_LNK_CAS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_linkStatCtrl_reg */


/*****************************************************************************
 * Read and split up the Device Capabilities 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_devCap2_reg
(
  volatile const uint32_t *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t        *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAP_2;

  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE,           swReg->cmplToEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT, swReg->cmplToDisSupp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT,         swReg->ariFwdSp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP,         swReg->aorSp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP,          swReg->aoc32Sp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP,          swReg->aoc64Sp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP,            swReg->casc128Sp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR,          swReg->noRoPR);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP,                    swReg->ltrSupp);
  pcie_getbits(val, PCIE_REV2_TPH_CMPLT_SUPPORT,                                                    swReg->tphcSp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_LN_SYS_CLS,                 swReg->lnSysCls);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT,    swReg->tag10bitCompSupp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT,     swReg->tag10bitReqSupp);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT,                swReg->obffSupp);

  return pcie_RET_OK;
} /* pciev2_read_devCap2_reg */


/*****************************************************************************
 * Combine and write the Device Capabilities 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_devCap2_reg
(
  volatile uint32_t *hwReg_DEV_CAP_2,
  pcieDevCap2Reg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_RANGE,           swReg->cmplToEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE_SUPPORT, swReg->cmplToDisSupp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT,         swReg->ariFwdSp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_ATOMIC_ROUTING_SUPP,         swReg->aorSp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_32_ATOMIC_CPL_SUPP,          swReg->aoc32Sp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_64_ATOMIC_CPL_SUPP,          swReg->aoc64Sp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_128_CAS_CPL_SUPP,            swReg->casc128Sp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_NO_RO_EN_PR2PR_PAR,          swReg->noRoPR);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_LTR_SUPP,                    swReg->ltrSupp);
  pcie_setbits(new_val, PCIE_REV2_TPH_CMPLT_SUPPORT,                                                    swReg->tphcSp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_LN_SYS_CLS,                 swReg->lnSysCls);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_COMP_SUPPORT,    swReg->tag10bitCompSupp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP2_10_BIT_TAG_REQ_SUPPORT,     swReg->tag10bitReqSupp);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CAPABILITIES2_REG_PCIE_CAP_OBFF_SUPPORT,                swReg->obffSupp);

  *hwReg_DEV_CAP_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_devCap2_reg */


/*****************************************************************************
 * Read and split up the Device Status and Control Register 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_devStatCtrl2_reg
(
  volatile const uint32_t *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t   *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_DEV_CAS_2;

  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE,      swReg->cmplTo);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE,    swReg->cmplToDis);
  pcie_getbits(val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS, swReg->ariFwdSp);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->aopReqEn = 0u;
  swReg->aopEgBlk = 0u;
  swReg->idoReqEn = 0u;
  swReg->idoCplEn = 0u;
  swReg->ltrEn    = 0u;
  swReg->obffEn   = 0u;

  return pcie_RET_OK;
} /* pciev2_read_devStatCtrl2_reg */


/*****************************************************************************
 * Combine and write the Device Status and Control Register 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_devStatCtrl2_reg
(
  volatile uint32_t     *hwReg_DEV_CAS_2,
  pcieDevStatCtrl2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_VALUE,      swReg->cmplTo);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_CPL_TIMEOUT_DISABLE,    swReg->cmplToDis);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_DEVICE_CONTROL2_DEVICE_STATUS2_REG_PCIE_CAP_ARI_FORWARD_SUPPORT_CS, swReg->ariFwdSp);

  *hwReg_DEV_CAS_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_devStatCtrl2_reg */

/*****************************************************************************
 * Read and split up the Link Capabilities 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_linkCap2_reg
(
  volatile const uint32_t *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t        *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAP_2;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR, swReg->spLsVec);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT,        swReg->crosslinkSp);

  return pcie_RET_OK;
} /* pciev2_read_linkCap2_reg */


/*****************************************************************************
 * Combine and write the Link Capabilites 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_linkCap2_reg
(
  volatile uint32_t *hwReg_LNK_CAP_2,
  pcieLnkCap2Reg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_SUPPORT_LINK_SPEED_VECTOR, swReg->spLsVec);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CAPABILITIES2_REG_PCIE_CAP_CROSS_LINK_SUPPORT,        swReg->crosslinkSp);

  *hwReg_LNK_CAP_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_linkCap2_reg */

/*****************************************************************************
 * Read and split up the Link Control 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_linkCtrl2_reg
(
  volatile const uint32_t *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t      *swReg
)
{
  uint32_t val = swReg->raw = *hwReg_LNK_CAS_2;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED,         swReg->tgtSpeed);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE,          swReg->entrCompl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE,     swReg->hwAutoSpeedDis);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS,            swReg->selDeemph);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN,                 swReg->txMargin);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE, swReg->entrModCompl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS,            swReg->cmplSos);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET,         swReg->complPrstDeemph);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS,           swReg->deEmph);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL,                    swReg->eqComplete);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1,                 swReg->eqPh1);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2,                 swReg->eqPh2);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3,                 swReg->eqPh3);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ,               swReg->linkEqReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE,          swReg->downCompPres);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED,               swReg->drsMsgRecv);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->pollDeemph = 0; /* Note: low bit of complPrstDeeph */

  return pcie_RET_OK;
} /* pciev2_read_linkCtrl2_reg */


/*****************************************************************************
 * Combine and write the Link Control 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_linkCtrl2_reg
(
  volatile uint32_t  *hwReg_LNK_CAS_2,
  pcieLinkCtrl2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TARGET_LINK_SPEED,         swReg->tgtSpeed);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_COMPLIANCE,          swReg->entrCompl);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_HW_AUTO_SPEED_DISABLE,     swReg->hwAutoSpeedDis);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_SEL_DEEMPHASIS,            swReg->selDeemph);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_TX_MARGIN,                 swReg->txMargin);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_ENTER_MODIFIED_COMPLIANCE, swReg->entrModCompl);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_SOS,            swReg->cmplSos);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_COMPLIANCE_PRESET,         swReg->complPrstDeemph);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_CURR_DEEMPHASIS,           swReg->deEmph);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL,                    swReg->eqComplete);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P1,                 swReg->eqPh1);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P2,                 swReg->eqPh2);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_EQ_CPL_P3,                 swReg->eqPh3);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_PCIE_CAP_LINK_EQ_REQ,               swReg->linkEqReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DOWNSTREAM_COMPO_PRESENCE,          swReg->downCompPres);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LINK_CONTROL2_LINK_STATUS2_REG_DRS_MESSAGE_RECEIVED,               swReg->drsMsgRecv);

  *hwReg_LNK_CAS_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_linkCtrl2_reg */


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
pcieRet_e pciev2_read_extCap_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieExtCapReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->AER_EXT_CAP_HDR_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_NEXT_OFFSET, swReg->nextCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_CAP_VERSION, swReg->extCapVer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AER_EXT_CAP_HDR_OFF_CAP_ID,      swReg->extCapID);

  return pcie_RET_OK;
} /* pciev2_read_extCap_reg */


/*****************************************************************************
 * Read and split up the Uncorrectable Error Status register
 ****************************************************************************/
pcieRet_e pciev2_read_uncErr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS, swReg->tlpPrfxBlockedErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS,         swReg->intErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS,  swReg->urErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS,             swReg->ecrcErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS,         swReg->mtlpErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS,     swReg->rcvrOfSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS,      swReg->ucmpSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS,      swReg->cmplAbrtSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS,    swReg->cmplTmotSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS,      swReg->fcpErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS,         swReg->psndTlpSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS,    swReg->srpsDnSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS,      swReg->dlpErrSt);

  return pcie_RET_OK;
} /* pciev2_read_uncErr_reg */

/*****************************************************************************
 * Combine and write the Uncorrectable Error Status register
 ****************************************************************************/
pcieRet_e pciev2_write_uncErr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_TLP_PRFX_BLOCKED_ERR_STATUS, swReg->tlpPrfxBlockedErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_INTERNAL_ERR_STATUS,         swReg->intErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNSUPPORTED_REQ_ERR_STATUS,  swReg->urErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_ECRC_ERR_STATUS,             swReg->ecrcErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_MALF_TLP_ERR_STATUS,         swReg->mtlpErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_REC_OVERFLOW_ERR_STATUS,     swReg->rcvrOfSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_UNEXP_CMPLT_ERR_STATUS,      swReg->ucmpSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_ABORT_ERR_STATUS,      swReg->cmplAbrtSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_CMPLT_TIMEOUT_ERR_STATUS,    swReg->cmplTmotSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_FC_PROTOCOL_ERR_STATUS,      swReg->fcpErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_POIS_TLP_ERR_STATUS,         swReg->psndTlpSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_SURPRISE_DOWN_ERR_STATUS,    swReg->srpsDnSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_STATUS_OFF_DL_PROTOCOL_ERR_STATUS,      swReg->dlpErrSt);

  baseAddr->UNCORR_ERR_STATUS_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_uncErr_reg */



/*****************************************************************************
 * Read and split up the Uncorrectable Error Mask register
 ****************************************************************************/
pcieRet_e pciev2_read_uncErrMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrMaskReg_t        *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_MASK_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK,      swReg->tlpPrfxBlockedErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK, swReg->atomicEgressBlockedErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK,              swReg->intErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK,       swReg->urErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK,                  swReg->ecrcErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK,              swReg->mtlpErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK,          swReg->rcvrOfMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK,           swReg->ucmpMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK,           swReg->cmplAbrtMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK,         swReg->cmplTmotMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK,           swReg->fcpErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK,              swReg->psndTlpMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK,         swReg->srpsDnMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK,           swReg->dlpErrMsk);

  return pcie_RET_OK;
} /* pciev2_read_uncErrMask_reg */

/*****************************************************************************
 * Combine and write the Uncorrectable Error Mask register
 ****************************************************************************/
pcieRet_e pciev2_write_uncErrMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrMaskReg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_TLP_PRFX_BLOCKED_ERR_MASK,      swReg->tlpPrfxBlockedErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ATOMIC_EGRESS_BLOCKED_ERR_MASK, swReg->atomicEgressBlockedErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_INTERNAL_ERR_MASK,              swReg->intErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNSUPPORTED_REQ_ERR_MASK,       swReg->urErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_ECRC_ERR_MASK,                  swReg->ecrcErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_MALF_TLP_ERR_MASK,              swReg->mtlpErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_REC_OVERFLOW_ERR_MASK,          swReg->rcvrOfMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_UNEXP_CMPLT_ERR_MASK,           swReg->ucmpMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_ABORT_ERR_MASK,           swReg->cmplAbrtMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_CMPLT_TIMEOUT_ERR_MASK,         swReg->cmplTmotMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_FC_PROTOCOL_ERR_MASK,           swReg->fcpErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_POIS_TLP_ERR_MASK,              swReg->psndTlpMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_SURPRISE_DOWN_ERR_MASK,         swReg->srpsDnMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_MASK_OFF_DL_PROTOCOL_ERR_MASK,           swReg->dlpErrMsk);

  baseAddr->UNCORR_ERR_MASK_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_uncErrMask_reg */


/*****************************************************************************
 * Read and split up the Uncorrectable Error Severity register
 ****************************************************************************/
pcieRet_e pciev2_read_uncErrSvrty_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrSvrtyReg_t       *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->UNCORR_ERR_SEV_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY,      swReg->tlpPrfxBlockedErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY, swReg->atomicEgressBlockedErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY,              swReg->intErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY,       swReg->urErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY,                  swReg->ecrcErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY,              swReg->mtlpErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY,          swReg->rcvrOfSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY,           swReg->ucmpSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY,           swReg->cmplAbrtSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY,         swReg->cmplTmotSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY,           swReg->fcpErrSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY,              swReg->psndTlpSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY,           swReg->srpsDnSvrty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY,           swReg->dlpErrSvrty);

  return pcie_RET_OK;
} /* pciev2_read_uncErrSvrty_reg */

/*****************************************************************************
 * Combine and write the Uncorrectable Error Severity register
 ****************************************************************************/
pcieRet_e pciev2_write_uncErrSvrty_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieUncErrSvrtyReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_TLP_PRFX_BLOCKED_ERR_SEVERITY,      swReg->tlpPrfxBlockedErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ATOMIC_EGRESS_BLOCKED_ERR_SEVERITY, swReg->atomicEgressBlockedErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_INTERNAL_ERR_SEVERITY,              swReg->intErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNSUPPORTED_REQ_ERR_SEVERITY,       swReg->urErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_ECRC_ERR_SEVERITY,                  swReg->ecrcErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_MALF_TLP_ERR_SEVERITY,              swReg->mtlpErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_REC_OVERFLOW_ERR_SEVERITY,          swReg->rcvrOfSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_UNEXP_CMPLT_ERR_SEVERITY,           swReg->ucmpSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_ABORT_ERR_SEVERITY,           swReg->cmplAbrtSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_CMPLT_TIMEOUT_ERR_SEVERITY,         swReg->cmplTmotSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_FC_PROTOCOL_ERR_SEVERITY,           swReg->fcpErrSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_POIS_TLP_ERR_SEVERITY,              swReg->psndTlpSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_SURPRISE_DOWN_ERR_SVRITY,           swReg->srpsDnSvrty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_UNCORR_ERR_SEV_OFF_DL_PROTOCOL_ERR_SEVERITY,           swReg->dlpErrSvrty);

  baseAddr->UNCORR_ERR_SEV_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_uncErrSvrty_reg */


/*****************************************************************************
 * Read and split up the Correctable Error Status register
 ****************************************************************************/
pcieRet_e pciev2_read_corErr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CORR_ERR_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS,    swReg->hdrLogOverflowErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS,      swReg->corrIntErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS, swReg->advNFErrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS,      swReg->rplyTmrSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS,     swReg->rpltRoSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS,               swReg->badDllpSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS,                swReg->badTlpSt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS,                 swReg->rcvrErrSt);

  return pcie_RET_OK;
} /* pciev2_read_corErr_reg */

/*****************************************************************************
 * Combine and write the Correctable Error Status register
 ****************************************************************************/
pcieRet_e pciev2_write_corErr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_HEADER_LOG_OVERFLOW_STATUS,    swReg->hdrLogOverflowErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_CORRECTED_INT_ERR_STATUS,      swReg->corrIntErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_ADVISORY_NON_FATAL_ERR_STATUS, swReg->advNFErrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RPL_TIMER_TIMEOUT_STATUS,      swReg->rplyTmrSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_REPLAY_NO_ROLEOVER_STATUS,     swReg->rpltRoSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_DLLP_STATUS,               swReg->badDllpSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_BAD_TLP_STATUS,                swReg->badTlpSt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_STATUS_OFF_RX_ERR_STATUS,                 swReg->rcvrErrSt);

  baseAddr->CORR_ERR_STATUS_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_corErr_reg */


/*****************************************************************************
 * Read and split up the Correctable Error Mask register
 ****************************************************************************/
pcieRet_e pciev2_read_corErrMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrMaskReg_t        *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CORR_ERR_MASK_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK,    swReg->hdrLogOverflowErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK,      swReg->corrIntErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK, swReg->advNFErrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK,      swReg->rplyTmrMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK,     swReg->rpltRoMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK,               swReg->badDllpMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK,                swReg->badTlpMsk);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK,                 swReg->rcvrErrMsk);

  return pcie_RET_OK;
} /* pciev2_read_corErrMask_reg */

/*****************************************************************************
 * Combine and write the Correctable Error Mask register
 ****************************************************************************/
pcieRet_e pciev2_write_corErrMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCorErrMaskReg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_HEADER_LOG_OVERFLOW_MASK,    swReg->hdrLogOverflowErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_CORRECTED_INT_ERR_MASK,      swReg->corrIntErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_ADVISORY_NON_FATAL_ERR_MASK, swReg->advNFErrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RPL_TIMER_TIMEOUT_MASK,      swReg->rplyTmrMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_REPLAY_NO_ROLEOVER_MASK,     swReg->rpltRoMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_DLLP_MASK,               swReg->badDllpMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_BAD_TLP_MASK,                swReg->badTlpMsk);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CORR_ERR_MASK_OFF_RX_ERR_MASK,                 swReg->rcvrErrMsk);

  baseAddr->CORR_ERR_MASK_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_corErrMask_reg */


/*****************************************************************************
 * Read and split up the Advanced Capabilities and Control register
 ****************************************************************************/
pcieRet_e pciev2_read_accr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieAccrReg_t              *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ADV_ERR_CAP_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN,  swReg->multHdrEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP, swReg->multHdrCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN,       swReg->chkEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP,      swReg->chkCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN,         swReg->genEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP,        swReg->genCap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER,   swReg->erPtr);

  return pcie_RET_OK;
} /* pciev2_read_accr_reg */


/*****************************************************************************
 * Combine and write the Advanced Capabilities and Control register
 ****************************************************************************/
pcieRet_e pciev2_write_accr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieAccrReg_t        *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_EN,  swReg->multHdrEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_MULTIPLE_HEADER_CAP, swReg->multHdrCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_EN,       swReg->chkEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_CHECK_CAP,      swReg->chkCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_EN,         swReg->genEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_ECRC_GEN_CAP,        swReg->genCap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ADV_ERR_CAP_CTRL_OFF_FIRST_ERR_POINTER,   swReg->erPtr);

  baseAddr->ADV_ERR_CAP_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_accr_reg */

/*****************************************************************************
 * Read and split up the Header Log register
 ****************************************************************************/
pcieRet_e pciev2_read_hdrLog_reg
(
  const CSL_pcie_ep_coreRegs   *baseAddr,
  pcieHdrLogReg_t *reg,
  int32_t              regNum
)
{
  reg->raw = reg->hdrDW = baseAddr->HDR_LOG_OFF[regNum];

  return pcie_RET_OK;
} /* pciev2_read_hdrLog_reg */


/*****************************************************************************
 * Read and split up the Root Error Command register
 ****************************************************************************/
pcieRet_e pciev2_read_rootErrCmd_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrCmdReg_t        *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_ERR_CMD_OFF;

  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN,     swReg->ferrRptEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN, swReg->nferrRptEn);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN,      swReg->cerrRptEn);

  return pcie_RET_OK;
} /* pciev2_read_rootErrCmd_reg */

/*****************************************************************************
 * Combine and write the Root Error Command register
 ****************************************************************************/
pcieRet_e pciev2_write_rootErrCmd_reg
(
  CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrCmdReg_t  *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_FATAL_ERR_REPORTING_EN,     swReg->ferrRptEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_NON_FATAL_ERR_REPORTING_EN, swReg->nferrRptEn);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_CMD_OFF_CORR_ERR_REPORTING_EN,      swReg->cerrRptEn);

  baseAddr->ROOT_ERR_CMD_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_rootErrCmd_reg */


/*****************************************************************************
 * Read and split up the Root Error Status register
 ****************************************************************************/
pcieRet_e pciev2_read_rootErrSt_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrStReg_t         *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ROOT_ERR_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM,        swReg->aerIntMsg);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX,           swReg->ferrRcv);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX,       swReg->nfErr);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL,         swReg->uncorFatal);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX, swReg->multFnf);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX,     swReg->errFnf);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX,             swReg->multCor);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX,                 swReg->corrErr);

  return pcie_RET_OK;
} /* pciev2_read_rootErrSt_reg */

/*****************************************************************************
 * Combine and write the Root Error Status register
 ****************************************************************************/
pcieRet_e pciev2_write_rootErrSt_reg
(
  CSL_pcie_rc_coreRegs *baseAddr,
  pcieRootErrStReg_t   *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ADV_ERR_INT_MSG_NUM,        swReg->aerIntMsg);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FATAL_ERR_MSG_RX,           swReg->ferrRcv);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_NON_FATAL_ERR_MSG_RX,       swReg->nfErr);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_FIRST_UNCORR_FATAL,         swReg->uncorFatal);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_FATAL_NON_FATAL_RX, swReg->multFnf);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_FATAL_NON_FATAL_RX,     swReg->errFnf);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_MUL_ERR_COR_RX,             swReg->multCor);
  pcie_setbits(new_val, CSL_PCIE_RC_CORE_ROOT_ERR_STATUS_OFF_ERR_COR_RX,                 swReg->corrErr);

  baseAddr->ROOT_ERR_STATUS_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_rootErrSt_reg */


/*****************************************************************************
 * Read and split up the Error Source Identification register
 ****************************************************************************/
pcieRet_e pciev2_read_errSrcID_reg
(
  const CSL_pcie_rc_coreRegs *baseAddr,
  pcieErrSrcIDReg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ERR_SRC_ID_OFF;

  pcie_getbits(val, CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_FATAL_NON_FATAL_SOURCE_ID, swReg->fnfSrcID);
  pcie_getbits(val, CSL_PCIE_RC_CORE_ERR_SRC_ID_OFF_ERR_COR_SOURCE_ID,             swReg->corrSrcID);

  return pcie_RET_OK;
} /* pciev2_read_errSrcID_reg */

/* Nothing past this point */

