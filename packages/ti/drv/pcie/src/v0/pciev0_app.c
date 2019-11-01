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
 *  Processing/configuration functions for the PCIe Application Registers
 *
 */


#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v0/pcieloc.h>

/*****************************************************************************
 **********  PCIe APPLICATION REGISTERS  *****************
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Peripheral Version and ID register
 ****************************************************************************/
pcieRet_e pciev0_read_pid_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePidReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PID;

  pcie_getbits(val, CSL_PCIESS_APP_PID_SCHEME,     reg->scheme);
  pcie_getbits(val, CSL_PCIESS_APP_PID_FUNC,       reg->func);
  pcie_getbits(val, CSL_PCIESS_APP_PID_RTL,        reg->rtl);
  pcie_getbits(val, CSL_PCIESS_APP_PID_MAJOR,      reg->revMaj);
  pcie_getbits(val, CSL_PCIESS_APP_PID_CUSTOM,     reg->cust);
  pcie_getbits(val, CSL_PCIESS_APP_PID_MINOR,      reg->revMin);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->modId = 0u;

  return pcie_RET_OK;
} /* pciev0_read_pid_reg */

/*****************************************************************************
 * Read and split up the Command Status register
 ****************************************************************************/
pcieRet_e pciev0_read_cmdStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieCmdStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CMD_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_DBI_CS2,       reg->dbi);
  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_APP_RETRY_EN,  reg->appRetryEn);
  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_POSTED_WR_EN,  reg->postedWrEn);
  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_IB_XLT_EN,     reg->ibXltEn);
  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_OB_XLT_EN,     reg->obXltEn);
  pcie_getbits(val, CSL_PCIESS_APP_CMD_STATUS_LTSSM_EN,      reg->ltssmEn);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->rxLaneFlipEn = 0u;
  reg->txLaneFlipEn = 0u;

  return pcie_RET_OK;
} /* pciev0_read_cmdStatus_reg */

/*****************************************************************************
 * Combine and write the Command Status register
 ****************************************************************************/
pcieRet_e pciev0_write_cmdStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieCmdStatusReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;
  
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_DBI_CS2,       reg->dbi);
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_APP_RETRY_EN,  reg->appRetryEn);
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_POSTED_WR_EN,  reg->postedWrEn);
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_IB_XLT_EN,     reg->ibXltEn);
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_OB_XLT_EN,     reg->obXltEn);
  pcie_setbits(new_val, CSL_PCIESS_APP_CMD_STATUS_LTSSM_EN,      reg->ltssmEn);

  baseAddr->CMD_STATUS = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_cmdStatus_reg */


/*****************************************************************************
 * Read and split up the Configuration Transaction Setup register
 ****************************************************************************/
pcieRet_e pciev0_read_cfgTrans_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieCfgTransReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->CFG_SETUP;

  pcie_getbits(val, CSL_PCIESS_APP_CFG_SETUP_CFG_TYPE,    reg->type);
  pcie_getbits(val, CSL_PCIESS_APP_CFG_SETUP_CFG_BUS,     reg->bus);
  pcie_getbits(val, CSL_PCIESS_APP_CFG_SETUP_CFG_DEVICE,  reg->device);
  pcie_getbits(val, CSL_PCIESS_APP_CFG_SETUP_CFG_FUNC,    reg->func);

  return pcie_RET_OK;
} /* pciev0_read_cfgTrans_reg */

/*****************************************************************************
 * Combine and write the Configuration Transaction Setup register
 ****************************************************************************/
pcieRet_e pciev0_write_cfgTrans_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieCfgTransReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_CFG_SETUP_CFG_TYPE,    reg->type);
  pcie_setbits(new_val, CSL_PCIESS_APP_CFG_SETUP_CFG_BUS,     reg->bus);
  pcie_setbits(new_val, CSL_PCIESS_APP_CFG_SETUP_CFG_DEVICE,  reg->device);
  pcie_setbits(new_val, CSL_PCIESS_APP_CFG_SETUP_CFG_FUNC,    reg->func);

  baseAddr->CFG_SETUP = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_cfgTrans_reg */


/*****************************************************************************
 * Read and split up the IO TLP Base register
 ****************************************************************************/
pcieRet_e pciev0_read_ioBase_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIoBaseReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->IOBASE;

  pcie_getbits(val, CSL_PCIESS_APP_IOBASE_IOBASE, reg->ioBase);

  return pcie_RET_OK;
} /* pciev0_read_ioBase_reg */

/*****************************************************************************
 * Combine and write the IO TLP Base register
 ****************************************************************************/
pcieRet_e pciev0_write_ioBase_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIoBaseReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IOBASE_IOBASE, reg->ioBase);
  
  baseAddr->IOBASE = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_ioBase_reg */


/*****************************************************************************
 * Read and split up the TLP Attribute Configuration register
 ****************************************************************************/
pcieRet_e pciev0_read_tlpCfg_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieTlpCfgReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->TLPCFG;

  pcie_getbits(val, CSL_PCIESS_APP_TLPCFG_RELAXED,    reg->relaxed);
  pcie_getbits(val, CSL_PCIESS_APP_TLPCFG_NO_SNOOP,   reg->noSnoop);

  return pcie_RET_OK;
} /* pciev0_read_tlpCfg_reg */

/*****************************************************************************
 * Combine and write the TLP Attribute Configuration register
 ****************************************************************************/
pcieRet_e pciev0_write_tlpCfg_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieTlpCfgReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_TLPCFG_RELAXED,    reg->relaxed);
  pcie_setbits(new_val, CSL_PCIESS_APP_TLPCFG_NO_SNOOP,   reg->noSnoop);

  baseAddr->TLPCFG = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_tlpCfg_reg */


/*****************************************************************************
 * Read and split up the Reset Command register
 ****************************************************************************/
pcieRet_e pciev0_read_rstCmd_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieRstCmdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->RSTCMD;

  pcie_getbits(val, CSL_PCIESS_APP_RSTCMD_FLUSH_N,    reg->flush);
  pcie_getbits(val, CSL_PCIESS_APP_RSTCMD_INIT_RST,   reg->initRst);

  /* Set unused fields to 0 (only used by rev 1/2 hw) */
  reg->flrPfActive = 0u;

  return pcie_RET_OK;
} /* pciev0_read_rstCmd_reg */

/*****************************************************************************
 * Combine and write the Reset Command register
 ****************************************************************************/
pcieRet_e pciev0_write_rstCmd_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieRstCmdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_RSTCMD_FLUSH_N,    reg->flush);
  pcie_setbits(new_val, CSL_PCIESS_APP_RSTCMD_INIT_RST,   reg->initRst);

  baseAddr->RSTCMD = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_rstCmd_reg */


/*****************************************************************************
 * Read and split up the Power Management Command register
 ****************************************************************************/
pcieRet_e pciev0_read_pmCmd_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmCmdReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMCMD;

  pcie_getbits(val, CSL_PCIESS_APP_PMCMD_PM_XMT_TURNOFF,  reg->turnOff);
  pcie_getbits(val, CSL_PCIESS_APP_PMCMD_PM_XMT_PME,      reg->pme);

  return pcie_RET_OK;
} /* pciev0_read_pmCmd_reg */

/*****************************************************************************
 * Combine and write the Power Management Command register
 ****************************************************************************/
pcieRet_e pciev0_write_pmCmd_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmCmdReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMCMD_PM_XMT_TURNOFF,  reg->turnOff);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMCMD_PM_XMT_PME,      reg->pme);

  baseAddr->PMCMD = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_pmCmd_reg */

/*****************************************************************************
 * Read and split up the Power Mangement Configuration register
 ****************************************************************************/
pcieRet_e pciev0_read_pmCfg_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmCfgReg_t *reg
)
{

#ifdef CSL_PCIESS_APP_PMCFG_ENTR_L23_RESETVAL
  uint32_t val = reg->raw = baseAddr->PMCFG;

  pcie_getbits(val, CSL_PCIESS_APP_PMCFG_ENTR_L23,  reg->entrL23);

  return pcie_RET_OK;
#else
  return pcie_RET_UNSUPPORTED;
#endif
  
} /* pcie_read_pmCfg_reg */

/*****************************************************************************
 * Combine and write the Power Mangement Configuration register
 ****************************************************************************/
pcieRet_e pciev0_write_pmCfg_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmCfgReg_t *reg
)
{
#ifdef CSL_PCIESS_APP_PMCFG_ENTR_L23_RESETVAL
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMCFG_ENTR_L23, reg->entrL23);

  baseAddr->PMCFG = reg->raw = new_val;
  return pcie_range_check_return;
#else
  return pcie_RET_UNSUPPORTED;
#endif
} /* pcie_write_pmCfg_reg */

/*****************************************************************************
 * Read and split up the Activity Status register
 ****************************************************************************/
pcieRet_e pciev0_read_actStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieActStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ACT_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_ACT_STATUS_OB_NOT_EMPTY, reg->obNotEmpty);
  pcie_getbits(val, CSL_PCIESS_APP_ACT_STATUS_IB_NOT_EMPTY, reg->ibNotEmpty);

  return pcie_RET_OK;
} /* pciev0_read_actStatus_reg */

/*****************************************************************************
 * Read and split up the Outbound Size register
 ****************************************************************************/
pcieRet_e pciev0_read_obSize_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObSizeReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->OB_SIZE;

  pcie_getbits(val, CSL_PCIESS_APP_OB_SIZE_OB_SIZE, reg->size);

  return pcie_RET_OK;
} /* pciev0_read_obSize_reg */


/*****************************************************************************
 * Combine and write the Outbound Size register
 ****************************************************************************/
pcieRet_e pciev0_write_obSize_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObSizeReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_OB_SIZE_OB_SIZE,  reg->size);

  baseAddr->OB_SIZE = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_obSize_reg */

/*****************************************************************************  
 * Read and split up the Diagnostic Control register
 ****************************************************************************/  
pcieRet_e pciev0_read_diagCtrl_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieDiagCtrlReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->DIAG_CTRL;

  pcie_getbits(val, CSL_PCIESS_APP_DIAG_CTRL_INV_ECRC, reg->invEcrc);
  pcie_getbits(val, CSL_PCIESS_APP_DIAG_CTRL_INV_LCRC, reg->invLcrc);

  return pcie_RET_OK;
} /* pciev0_read_diagCtrl_reg */


/*****************************************************************************  
 * Combine and write the Diagnostic Control register
 ****************************************************************************/  
pcieRet_e pciev0_write_diagCtrl_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieDiagCtrlReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_DIAG_CTRL_INV_ECRC, reg->invEcrc);
  pcie_setbits(new_val, CSL_PCIESS_APP_DIAG_CTRL_INV_LCRC, reg->invLcrc);

  baseAddr->DIAG_CTRL = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_diagCtrl_reg */

/*****************************************************************************  
 * Read and split up the Endian Mode register
 ****************************************************************************/  
pcieRet_e pciev0_read_endian_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEndianReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->ENDIAN;

  pcie_getbits(val, CSL_PCIESS_APP_ENDIAN_ENDIAN_MODE, reg->mode);

  return pcie_RET_OK;
} /* pciev0_read_endian_reg */


/*****************************************************************************  
 * Combine and write the Endian Mode register
 ****************************************************************************/  
pcieRet_e pciev0_write_endian_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEndianReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_ENDIAN_ENDIAN_MODE, reg->mode);

  baseAddr->ENDIAN = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_endian_reg */

/*****************************************************************************  
 * Read and split up the transaction priority register
 ****************************************************************************/  
pcieRet_e pciev0_read_priority_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePriorityReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PRIORITY;

  pcie_getbits(val, CSL_PCIESS_APP_PRIORITY_MST_PRIV, reg->mstPriv);
  pcie_getbits(val, CSL_PCIESS_APP_PRIORITY_MST_PRIVID, reg->mstPrivID);
  pcie_getbits(val, CSL_PCIESS_APP_PRIORITY_MST_PRIORITY, reg->mstPriority);

  return pcie_RET_OK;
} /* pciev0_read_priority_reg */

/*****************************************************************************  
 * Combine and write the transaction priority register
 ****************************************************************************/  
pcieRet_e pciev0_write_priority_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePriorityReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PRIORITY_MST_PRIV, reg->mstPriv);
  pcie_setbits(new_val, CSL_PCIESS_APP_PRIORITY_MST_PRIVID, reg->mstPrivID);
  pcie_setbits(new_val, CSL_PCIESS_APP_PRIORITY_MST_PRIORITY, reg->mstPriority);

  baseAddr->PRIORITY = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_priority_reg */

/*****************************************************************************  
 * Read and split up the End of Interrupt register
 ****************************************************************************/  
pcieRet_e pciev0_read_irqEOI_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIrqEOIReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->IRQ_EOI;

  pcie_getbits(val, CSL_PCIESS_APP_IRQ_EOI_EOI, reg->EOI);

  return pcie_RET_OK;
} /* pciev0_read_irqEOI_reg */


/*****************************************************************************  
 * Combine and write the End of Interrupt register
 ****************************************************************************/  
pcieRet_e pciev0_write_irqEOI_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIrqEOIReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IRQ_EOI_EOI, reg->EOI);

  baseAddr->IRQ_EOI = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_irqEOI_reg */

/*****************************************************************************  
 * Read and split up the MSI Interrupt IRQ register
 ****************************************************************************/  
pcieRet_e pciev0_read_msiIrq_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->MSI_IRQ;

  pcie_getbits(val, CSL_PCIESS_APP_MSI_IRQ_MSI_IRQ, reg->msiIrq);

  return pcie_RET_OK;
} /* pciev0_read_msiIrq_reg */


/*****************************************************************************  
 * Combine and write the MSI Interrupt IRQ register
 ****************************************************************************/  
pcieRet_e pciev0_write_msiIrq_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_MSI_IRQ_MSI_IRQ, reg->msiIrq);

  baseAddr->MSI_IRQ = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_msiIrq_reg */

/*****************************************************************************  
 * Read and split up the Endpoint Interrupt Request Set register
 ****************************************************************************/  
pcieRet_e pciev0_read_epIrqSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqSetReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->EP_IRQ_SET;

  pcie_getbits(val, CSL_PCIESS_APP_EP_IRQ_SET_EP_IRQ_SET, reg->epIrqSet);

  return pcie_RET_OK;
} /* pciev0_read_epIrqSet_reg */


/*****************************************************************************  
 * Combine and write the Endpoint Interrupt Request Set register
 ****************************************************************************/  
pcieRet_e pciev0_write_epIrqSet_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqSetReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_EP_IRQ_SET_EP_IRQ_SET, reg->epIrqSet);

  baseAddr->EP_IRQ_SET = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_epIrqSet_reg */

/*****************************************************************************  
 * Read and split up the Endpoint Interrupt Request Clear register
 ****************************************************************************/  
pcieRet_e pciev0_read_epIrqClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqClrReg_t *reg 
)
{
  uint32_t val = reg->raw = baseAddr->EP_IRQ_CLR;

  pcie_getbits(val, CSL_PCIESS_APP_EP_IRQ_CLR_EP_IRQ_CLR, reg->epIrqClr);

  return pcie_RET_OK;
} /* pciev0_read_epIrqClr_reg */

/*****************************************************************************  
 * Combine and write the Endpoint Interrupt Request Clear register
 ****************************************************************************/  
pcieRet_e pciev0_write_epIrqClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqClrReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_EP_IRQ_CLR_EP_IRQ_CLR, reg->epIrqClr);

  baseAddr->EP_IRQ_CLR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_epIrqClr_reg */

/*****************************************************************************  
 * Read and split up the Endpoint Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_epIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqStatusReg_t       *reg 
)
{
  uint32_t val = reg->raw = baseAddr->EP_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_EP_IRQ_STATUS_EP_IRQ_STATUS, reg->epIrqStatus);

  return pcie_RET_OK;
} /* pciev0_read_epIrqStatus_reg */


/*****************************************************************************  
 * Combine and write the Endpoint Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_epIrqStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieEpIrqStatusReg_t *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_EP_IRQ_STATUS_EP_IRQ_STATUS, reg->epIrqStatus);

  baseAddr->EP_IRQ_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_epIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a General Purpose register [0-3]
 ****************************************************************************/  
pcieRet_e pciev0_read_genPurpose_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieGenPurposeReg_t        *reg,
  int_fast32_t                regNum 
)
{
  reg->raw = reg->genPurpose = baseAddr->GPR[regNum];

  return pcie_RET_OK;
} /* pciev0_read_epIrqStatus_reg */


/*****************************************************************************  
 * Combine and write a General Purpose register [0-3]
 ****************************************************************************/  
pcieRet_e pciev0_write_genPurpose_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieGenPurposeReg_t  *reg,
  int_fast32_t          regNum 
)
{
  baseAddr->GPR[regNum] = reg->raw = reg->genPurpose;

  return pcie_RET_OK;
} /* pciev0_write_epIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a MSI Raw Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_msiIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqStatusRawReg_t   *reg,
  int_fast32_t                regNum
)
{
  uint32_t val = reg->raw = baseAddr->MSIX_IRQ[regNum].MSI_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIESS_APP_MSI_IRQ_STATUS_RAW_MSI_RAW_STATUS, reg->msiRawStatus);

  return pcie_RET_OK;
} /* pciev0_read_msiIrqStatusRaw_reg */


/*****************************************************************************  
 * Combine and write a MSI Raw Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_msiIrqStatusRaw_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieMsiIrqStatusRawReg_t *reg, 
  int_fast32_t              regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_MSI_IRQ_STATUS_RAW_MSI_RAW_STATUS, reg->msiRawStatus);

  baseAddr->MSIX_IRQ[regNum].MSI_IRQ_STATUS_RAW = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_msiIrqStatusRaw_reg */

/*****************************************************************************  
 * Read and split up a MSI Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_msiIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqStatusReg_t      *reg, 
  int_fast32_t                regNum
)
{
  uint32_t val = reg->raw = baseAddr->MSIX_IRQ[regNum].MSI_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_MSI_IRQ_STATUS_MSI_IRQ_STATUS, reg->msiIrqStatus);

  return pcie_RET_OK;
} /* pciev0_read_msiIrqStatus_reg */


/*****************************************************************************  
 * Combine and write a MSI Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_msiIrqStatus_reg
(
  CSL_Pciess_appRegs    *baseAddr, 
  pcieMsiIrqStatusReg_t *reg, 
  int_fast32_t           regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_MSI_IRQ_STATUS_MSI_IRQ_STATUS, reg->msiIrqStatus);

  baseAddr->MSIX_IRQ[regNum].MSI_IRQ_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_msiIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a MSI Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_read_msiIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqEnableSetReg_t   *reg, 
  int_fast32_t                regNum
)
{
  uint32_t val = reg->raw = baseAddr->MSIX_IRQ[regNum].MSI_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIESS_APP_MSI_IRQ_ENABLE_SET_MSI_IRQ_EN_SET, reg->msiIrqEnSet);

  return pcie_RET_OK;
} /* pciev0_read_msiIrqEnableSet_reg */


/*****************************************************************************  
 * Combine and write a MSI Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_write_msiIrqEnableSet_reg
(
  CSL_Pciess_appRegs        *baseAddr, 
  pcieMsiIrqEnableSetReg_t  *reg, 
  int_fast32_t               regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_MSI_IRQ_ENABLE_SET_MSI_IRQ_EN_SET, reg->msiIrqEnSet);

  baseAddr->MSIX_IRQ[regNum].MSI_IRQ_ENABLE_SET = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_msiIrqEnableSet_reg */

/*****************************************************************************  
 * Read and split up the MSI Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_read_msiIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieMsiIrqEnableClrReg_t   *reg, 
  int_fast32_t                regNum
)
{
  uint32_t val = reg->raw = baseAddr->MSIX_IRQ[regNum].MSI_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIESS_APP_MSI_IRQ_ENABLE_CLR_MSI_IRQ_EN_CLR, reg->msiIrqEnClr);

  return pcie_RET_OK;
} /* pciev0_read_msiIrqEnableClr_reg */


/*****************************************************************************  
 * Combine and write the MSI Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_write_msiIrqEnableClr_reg
(
  CSL_Pciess_appRegs       *baseAddr, 
  pcieMsiIrqEnableClrReg_t *reg, 
  int_fast32_t              regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_MSI_IRQ_ENABLE_CLR_MSI_IRQ_EN_CLR, reg->msiIrqEnClr);

  baseAddr->MSIX_IRQ[regNum].MSI_IRQ_ENABLE_CLR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_msiIrqEnableClr_reg */

/*****************************************************************************  
 * Read and split up a Legacy Raw Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_legacyIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqStatusRawReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIESS_APP_LEGACY_IRQ_STATUS_RAW_LEGACY_RAW_STATUS, reg->legacyRawStatus);

  return pcie_RET_OK;
} /* pciev0_read_legacyIrqStatusRaw_reg */


/*****************************************************************************  
 * Combine and write a Legacy Raw Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_legacyIrqStatusRaw_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqStatusRawReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_LEGACY_IRQ_STATUS_RAW_LEGACY_RAW_STATUS, reg->legacyRawStatus);

  baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_STATUS_RAW = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_legacyIrqStatusRaw_reg */

/*****************************************************************************  
 * Read and split up a Legacy Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_legacyIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqStatusReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS, reg->legacyIrqStatus);

  return pcie_RET_OK;
} /* pciev0_read_legacyIrqStatus_reg */


/*****************************************************************************  
 * Combine and write a Legacy Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_legacyIrqStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqStatusReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS, reg->legacyIrqStatus);

  baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_legacyIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a Legacy Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_read_legacyIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqEnableSetReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIESS_APP_LEGACY_IRQ_ENABLE_SET_LEGACY_IRQ_EN_SET, reg->legacyIrqEnSet);

  return pcie_RET_OK;
} /* pciev0_read_legacyIrqEnableSet_reg */

/*****************************************************************************  
 * Combine and write a Legacy Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_write_legacyIrqEnableSet_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqEnableSetReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_LEGACY_IRQ_ENABLE_SET_LEGACY_IRQ_EN_SET, reg->legacyIrqEnSet);

  baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_ENABLE_SET = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_legacyIrqEnableSet_reg */

/*****************************************************************************  
 * Read and split up the Legacy Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_read_legacyIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqEnableClrReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIESS_APP_LEGACY_IRQ_ENABLE_CLR_LEGACY_IRQ_EN_CLR, reg->legacyIrqEnClr);

  return pcie_RET_OK;
} /* pciev0_read_legacyIrqEnableClr_reg */


/*****************************************************************************  
 * Combine and write the Legacy Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_write_legacyIrqEnableClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieLegacyIrqEnableClrReg_t *reg, 
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_LEGACY_IRQ_ENABLE_CLR_LEGACY_IRQ_EN_CLR, reg->legacyIrqEnClr);

  baseAddr->LEGACY_X_IRQ[regNum].LEGACY_IRQ_ENABLE_CLR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_legacyIrqEnableClr_reg */

/*****************************************************************************  
 * Read and split up a Raw ERR Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_errIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusRawReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ERR_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_AER,      reg->errAer);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_AXI,      reg->errAxi);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_CORR,     reg->errCorr);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_NONFATAL, reg->errNonFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_FATAL,    reg->errFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_SYS,      reg->errSys);

  return pcie_RET_OK;
} /* pciev0_read_errIrqStatusRaw_reg */


/*****************************************************************************  
 * Combine and write a Raw ERR Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_errIrqStatusRaw_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusRawReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_AER,      reg->errAer);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_AXI,      reg->errAxi);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_CORR,     reg->errCorr);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_NONFATAL, reg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_FATAL,    reg->errFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_RAW_ERR_SYS,      reg->errSys);

  baseAddr->ERR_IRQ_STATUS_RAW = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_errIrqStatusRaw_reg */

/*****************************************************************************  
 * Read and split up a Err Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_errIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ERR_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_AER,      reg->errAer);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_AXI,      reg->errAxi);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_CORR,     reg->errCorr);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_NONFATAL, reg->errNonFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_FATAL,    reg->errFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_SYS,      reg->errSys);

  return pcie_RET_OK;
} /* pciev0_read_errIrqStatus_reg */

/*****************************************************************************  
 * Combine and write a Err Interrupt Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_errIrqStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqStatusReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_AER,      reg->errAer);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_AXI,      reg->errAxi);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_CORR,     reg->errCorr);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_NONFATAL, reg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_FATAL,    reg->errFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_STATUS_ERR_SYS,      reg->errSys);

  baseAddr->ERR_IRQ_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_errIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a Err Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_read_errIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableSetReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ERR_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_AER,      reg->errAer);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_AXI,      reg->errAxi);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_CORR,     reg->errCorr);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_NONFATAL, reg->errNonFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_FATAL,    reg->errFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_SYS,      reg->errSys);

  return pcie_RET_OK;
} /* pciev0_read_errIrqEnableSet_reg */

/*****************************************************************************  
 * Combine and write a Err Interrupt Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_write_errIrqEnableSet_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableSetReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_AER,      reg->errAer);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_AXI,      reg->errAxi);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_CORR,     reg->errCorr);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_NONFATAL, reg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_FATAL,    reg->errFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_SET_ERR_SYS,      reg->errSys);

  baseAddr->ERR_IRQ_ENABLE_SET = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_errIrqEnableSet_reg */

/*****************************************************************************  
 * Read and split up the Err Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_read_errIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableClrReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->ERR_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_AER,      reg->errAer);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_AXI,      reg->errAxi);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_CORR,     reg->errCorr);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL, reg->errNonFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_FATAL,    reg->errFatal);
  pcie_getbits(val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_SYS,      reg->errSys);

  return pcie_RET_OK;
} /* pciev0_read_errIrqEnableClr_reg */


/*****************************************************************************  
 * Combine and write the Err Interrupt Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_write_errIrqEnableClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieErrIrqEnableClrReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_AER,      reg->errAer);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_AXI,      reg->errAxi);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_CORR,     reg->errCorr);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL, reg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_FATAL,    reg->errFatal);
  pcie_setbits(new_val, CSL_PCIESS_APP_ERR_IRQ_ENABLE_CLR_ERR_SYS,      reg->errSys);

  baseAddr->ERR_IRQ_ENABLE_CLR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_errIrqEnableClr_reg */

/*****************************************************************************  
 * Read and split up a Raw Power Management and Reset Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_pmRstIrqStatusRaw_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusRawReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMRST_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_LINK_RST_REQ, reg->linkRstReq);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_PME,       reg->pmPme);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_TO_ACK,    reg->pmToAck);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_TURNOFF,   reg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev0_read_pmRstIrqStatusRaw_reg */


/*****************************************************************************  
 * Combine and write a Raw Power Management and Reset Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_pmRstIrqStatusRaw_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusRawReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_LINK_RST_REQ, reg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_PME,       reg->pmPme);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_TO_ACK,    reg->pmToAck);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_RAW_PM_TURNOFF,   reg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS_RAW = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pmRstIrqStatusRaw_reg */

/*****************************************************************************  
 * Read and split up a Power Management and Reset Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_pmRstIrqStatus_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMRST_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_LINK_RST_REQ, reg->linkRstReq);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_PME,       reg->pmPme);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_TO_ACK,    reg->pmToAck);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_TURNOFF,   reg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev0_read_pmRstIrqStatus_reg */

/*****************************************************************************  
 * Combine and write a Power Management and Reset Status register
 ****************************************************************************/  
pcieRet_e pciev0_write_pmRstIrqStatus_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqStatusReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_LINK_RST_REQ, reg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_PME,       reg->pmPme);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_TO_ACK,    reg->pmToAck);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_IRQ_STATUS_PM_TURNOFF,   reg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pmRstIrqStatus_reg */

/*****************************************************************************  
 * Read and split up a Power Management and Reset Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_read_pmRstIrqEnableSet_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableSetReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMRST_ENABLE_SET;

  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_SET_LINK_RST_REQ, reg->linkRstReq);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_PME,       reg->pmPme);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_TO_ACK,    reg->pmToAck);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_TURNOFF,   reg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev0_read_pmRstIrqEnableSet_reg */

/*****************************************************************************  
 * Combine and write a Power Management and Reset Enable Set register
 ****************************************************************************/  
pcieRet_e pciev0_write_pmRstIrqEnableSet_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableSetReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_SET_LINK_RST_REQ, reg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_PME,       reg->pmPme);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_TO_ACK,    reg->pmToAck);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_SET_PM_TURNOFF,   reg->pmTurnoff);

  baseAddr->PMRST_ENABLE_SET = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pmRstIrqEnableSet_reg */

/*****************************************************************************  
 * Read and split up the Power Management and Reset Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_read_pmRstIrqEnableClr_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableClrReg_t *reg
)
{
  uint32_t val = reg->raw = baseAddr->PMRST_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_LINK_RST_REQ, reg->linkRstReq);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_PME,       reg->pmPme);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_TO_ACK,    reg->pmToAck);
  pcie_getbits(val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_TURNOFF,   reg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev0_read_pmRstIrqEnableClr_reg */


/*****************************************************************************  
 * Combine and write the Power Management and Reset Enable Clear register
 ****************************************************************************/  
pcieRet_e pciev0_write_pmRstIrqEnableClr_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePmRstIrqEnableClrReg_t *reg
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_LINK_RST_REQ, reg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_PME,       reg->pmPme);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_TO_ACK,    reg->pmToAck);
  pcie_setbits(new_val, CSL_PCIESS_APP_PMRST_ENABLE_CLR_PM_TURNOFF,   reg->pmTurnoff);

  baseAddr->PMRST_ENABLE_CLR = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pmRstIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up the Outbound Translation Region Offset Low and Index register
 ****************************************************************************/
pcieRet_e pciev0_read_obOffsetLo_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetLoReg_t        *reg,
  int_fast32_t                regNum
)
{
  uint32_t val = reg->raw = baseAddr->OUTBOUND_TRANSLATION[regNum].OB_OFFSET_INDEX;

  pcie_getbits(val, CSL_PCIESS_APP_OB_OFFSET_INDEX_OB_OFFSET_LO, reg->offsetLo);
  pcie_getbits(val, CSL_PCIESS_APP_OB_OFFSET_INDEX_OB_ENABLE,    reg->enable);

  return pcie_RET_OK;
} /* pciev0_read_obOffsetLo_reg */

/*****************************************************************************
 * Combine and write the Outbound Translation Region Offset Low and Index register
 ****************************************************************************/
pcieRet_e pciev0_write_obOffsetLo_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetLoReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_OB_OFFSET_INDEX_OB_OFFSET_LO, reg->offsetLo);
  pcie_setbits(new_val, CSL_PCIESS_APP_OB_OFFSET_INDEX_OB_ENABLE,    reg->enable);

  baseAddr->OUTBOUND_TRANSLATION[regNum].OB_OFFSET_INDEX = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_obOffsetLo_reg */


/*****************************************************************************
 * Read and split up the Outbound Translation Region Offset High register
 ****************************************************************************/
pcieRet_e pciev0_read_obOffsetHi_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetHiReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->OUTBOUND_TRANSLATION[regNum].OB_OFFSET_HI;

  pcie_getbits(val, CSL_PCIESS_APP_OB_OFFSET_HI_OB_OFFSET_HI,    reg->offsetHi);

  return pcie_RET_OK;
} /* pciev0_read_obOffsetHi_reg */

/*****************************************************************************
 * Combine and write the Outbound Translation Region Offset High register
 ****************************************************************************/
pcieRet_e pciev0_write_obOffsetHi_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieObOffsetHiReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_OB_OFFSET_HI_OB_OFFSET_HI,    reg->offsetHi);

  baseAddr->OUTBOUND_TRANSLATION[regNum].OB_OFFSET_HI = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_obOffsetHi_reg */

/*****************************************************************************
 * Read and split up the Inbound Translation BAR Match register
 ****************************************************************************/
pcieRet_e pciev0_read_ibBar_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbBarReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->INBOUND_TRANSLATION[regNum].IB_BAR;

  pcie_getbits(val, CSL_PCIESS_APP_IB_BAR_IB_BAR, reg->ibBar);

  return pcie_RET_OK;
} /* pciev0_read_ibBar_reg */

/*****************************************************************************
 * Combine and write the Inbound Translation BAR Match register
 ****************************************************************************/
pcieRet_e pciev0_write_ibBar_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbBarReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IB_BAR_IB_BAR, reg->ibBar);

  baseAddr->INBOUND_TRANSLATION[regNum].IB_BAR = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_ibBar_reg */

/*****************************************************************************
 * Read and split up the Inbound Translation Start Address Low register
 ****************************************************************************/
pcieRet_e pciev0_read_ibStartLo_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartLoReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->INBOUND_TRANSLATION[regNum].IB_START_LO;

  pcie_getbits(val, CSL_PCIESS_APP_IB_START_LO_IB_START_LO, reg->ibStartLo);

  return pcie_RET_OK;
} /* pciev0_read_ibStartLo_reg */

/*****************************************************************************
 * Combine and write the Inbound Translation Start Address Low register
 ****************************************************************************/
pcieRet_e pciev0_write_ibStartLo_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartLoReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IB_START_LO_IB_START_LO, reg->ibStartLo);

  baseAddr->INBOUND_TRANSLATION[regNum].IB_START_LO = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_ibStartLo_reg */


/*****************************************************************************
 * Read and split up the Inbound Translation Start Address High register
 ****************************************************************************/
pcieRet_e pciev0_read_ibStartHi_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartHiReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->INBOUND_TRANSLATION[regNum].IB_START_HI;

  pcie_getbits(val, CSL_PCIESS_APP_IB_START_HI_IB_START_HI, reg->ibStartHi);

  return pcie_RET_OK;
} /* pciev0_read_ibStartHi_reg */

/*****************************************************************************
 * Combine and write the Inbound Translation Start Address High register
 ****************************************************************************/
pcieRet_e pciev0_write_ibStartHi_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbStartHiReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IB_START_HI_IB_START_HI, reg->ibStartHi);

  baseAddr->INBOUND_TRANSLATION[regNum].IB_START_HI = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_ibStartHi_reg */

/*****************************************************************************
 * Read and split up the Inbound Translation Offset register
 ****************************************************************************/
pcieRet_e pciev0_read_ibOffset_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieIbOffsetReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t val = reg->raw = baseAddr->INBOUND_TRANSLATION[regNum].IB_OFFSET;

  pcie_getbits(val, CSL_PCIESS_APP_IB_OFFSET_IB_OFFSET, reg->ibOffset);

  return pcie_RET_OK;
} /* pciev0_read_ibOffset_reg */

/*****************************************************************************
 * Combine and write the Inbound Translation Offset register
 ****************************************************************************/
pcieRet_e pciev0_write_ibOffset_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieIbOffsetReg_t *reg,
  int_fast32_t regNum
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_IB_OFFSET_IB_OFFSET, reg->ibOffset);

  baseAddr->INBOUND_TRANSLATION[regNum].IB_OFFSET = reg->raw = new_val;
  return pcie_range_check_return;
} /* pciev0_write_ibOffset_reg */

/*****************************************************************************  
 * Read and split up the PCS Configuration 0 register
 ****************************************************************************/  
pcieRet_e pciev0_read_pcsCfg0_reg 
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg0Reg_t           *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCS_CFG0;

  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_SYNC,         reg->pcsSync);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_HOLDOFF,      reg->pcsHoldOff);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_RC_DELAY,     reg->pcsRCDelay);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_DET_DELAY,    reg->pcsDetDelay);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_SHRT_TM,      reg->pcsShrtTM);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_STAT186,      reg->pcsStat186);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_FIX_TERM,     reg->pcsFixTerm);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_L2_ENIDL_OFF, reg->pcsL2EnidlOff);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_L0S_RX_OFF,   reg->pcsL2L0SRxOff);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_RXTX_ON,      reg->pcsRxTxOn);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG0_PCS_RXTX_RST,     reg->pcsRxTxRst);

  return pcie_RET_OK;
} /* pciev0_read_pcsCfg0_reg */

/*****************************************************************************  
 * Combine and write the PCS Configuration 0 register
 ****************************************************************************/  
pcieRet_e pciev0_write_pcsCfg0_reg 
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg0Reg_t     *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_SYNC,         reg->pcsSync);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_HOLDOFF,      reg->pcsHoldOff);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_RC_DELAY,     reg->pcsRCDelay);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_DET_DELAY,    reg->pcsDetDelay);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_SHRT_TM,      reg->pcsShrtTM);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_STAT186,      reg->pcsStat186);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_FIX_TERM,     reg->pcsFixTerm);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_L2_ENIDL_OFF, reg->pcsL2EnidlOff);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_L0S_RX_OFF,   reg->pcsL2L0SRxOff);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_RXTX_ON,      reg->pcsRxTxOn);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG0_PCS_RXTX_RST,     reg->pcsRxTxRst);

  baseAddr->PCS_CFG0 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pcsCfg0_reg */

/*****************************************************************************  
 * Read and split up the PCS Configuration 1 register
 ****************************************************************************/  
pcieRet_e pciev0_read_pcsCfg1_reg 
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg1Reg_t           *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCS_CFG1;

  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_BIT,  reg->pcsErrBit);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_LN,   reg->pcsErrLn);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_MODE, reg->pcsErrMode);

  return pcie_RET_OK;
} /* pciev0_read_pcsCfg1_reg */

/*****************************************************************************  
 * Combine and write the PCS Configuration 1 register
 ****************************************************************************/  
pcieRet_e pciev0_write_pcsCfg1_reg 
(
  CSL_Pciess_appRegs   *baseAddr, 
  pciePcsCfg1Reg_t     *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_BIT,  reg->pcsErrBit);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_LN,   reg->pcsErrLn);
  pcie_setbits(new_val, CSL_PCIESS_APP_PCS_CFG1_PCS_ERR_MODE, reg->pcsErrMode);

  baseAddr->PCS_CFG1 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_pcsCfg1_reg */

/*****************************************************************************  
 * Read and split up the PCS Status register
 ****************************************************************************/  
pcieRet_e pciev0_read_pcsStatus_reg 
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pciePcsStatusReg_t         *reg 
)
{
  uint32_t val = reg->raw = baseAddr->PCS_STATUS;

  pcie_getbits(val, CSL_PCIESS_APP_PCS_STATUS_PCS_REV,   reg->pcsRev);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_STATUS_PCS_LN_EN, reg->pcsLnEn);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_STATUS_PCS_TX_EN, reg->pcsTxEn);
  pcie_getbits(val, CSL_PCIESS_APP_PCS_STATUS_PCS_RX_EN, reg->pcsRxEn);

  return pcie_RET_OK;
} /* pciev0_read_pcsStatus_reg */

/*****************************************************************************  
 * Read and split up the SERDES Configuration Lane 0 register
 ****************************************************************************/  
pcieRet_e pciev0_read_serdesCfg0_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg0Reg_t        *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CFG0;

  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_TX_LOOPBACK, reg->txLoopback);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_TX_MSYNC,    reg->txMsync);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_TX_CM,       reg->txCm);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_TX_INVPAIR,  reg->txInvpair);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_LOOPBACK, reg->rxLoopback);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_ENOC,     reg->rxEnoc);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_EQ,       reg->rxEq);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_CDR,      reg->rxCdr);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_LOS,      reg->rxLos);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_ALIGN,    reg->rxAlign);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG0_RX_INVPAIR,  reg->rxInvpair);

  return pcie_RET_OK;
} /* pciev0_read_serdesCfg0_reg */


/*****************************************************************************  
 * Combine and write the SERDES Configuration Lane 0 register
 ****************************************************************************/  
pcieRet_e pciev0_write_serdesCfg0_reg
(
  CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg0Reg_t  *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_TX_LOOPBACK, reg->txLoopback);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_TX_MSYNC,    reg->txMsync);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_TX_CM,       reg->txCm);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_TX_INVPAIR,  reg->txInvpair);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_LOOPBACK, reg->rxLoopback);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_ENOC,     reg->rxEnoc);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_EQ,       reg->rxEq);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_CDR,      reg->rxCdr);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_LOS,      reg->rxLos);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_ALIGN,    reg->rxAlign);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG0_RX_INVPAIR,  reg->rxInvpair);

  baseAddr->SERDES_CFG0 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_serdesCfg0_reg */

/*****************************************************************************  
 * Read and split up the SERDES Configuration Lane 1 register
 ****************************************************************************/  
pcieRet_e pciev0_read_serdesCfg1_reg
(
  const CSL_Pciess_appRegs   *baseAddr, 
  pcieSerdesCfg1Reg_t        *reg 
)
{
  uint32_t val = reg->raw = baseAddr->SERDES_CFG1;

  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_TX_LOOPBACK, reg->txLoopback);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_TX_MSYNC,    reg->txMsync);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_TX_CM,       reg->txCm);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_TX_INVPAIR,  reg->txInvpair);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_LOOPBACK, reg->rxLoopback);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_ENOC,     reg->rxEnoc);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_EQ,       reg->rxEq);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_CDR,      reg->rxCdr);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_LOS,      reg->rxLos);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_ALIGN,    reg->rxAlign);
  pcie_getbits(val, CSL_PCIESS_APP_SERDES_CFG1_RX_INVPAIR,  reg->rxInvpair);

  return pcie_RET_OK;
} /* pciev0_read_serdesCfg1_reg */


/*****************************************************************************  
 * Combine and write the SERDES Configuration Lane 1 register
 ****************************************************************************/  
pcieRet_e pciev0_write_serdesCfg1_reg
(
  CSL_Pciess_appRegs    *baseAddr, 
  pcieSerdesCfg1Reg_t   *reg 
)
{
  uint32_t new_val = reg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_TX_LOOPBACK, reg->txLoopback);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_TX_MSYNC,    reg->txMsync);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_TX_CM,       reg->txCm);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_TX_INVPAIR,  reg->txInvpair);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_LOOPBACK, reg->rxLoopback);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_ENOC,     reg->rxEnoc);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_EQ,       reg->rxEq);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_CDR,      reg->rxCdr);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_LOS,      reg->rxLos);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_ALIGN,    reg->rxAlign);
  pcie_setbits(new_val, CSL_PCIESS_APP_SERDES_CFG1_RX_INVPAIR,  reg->rxInvpair);

  baseAddr->SERDES_CFG1 = reg->raw = new_val;

  return pcie_range_check_return;
} /* pciev0_write_serdesCfg1_reg */

/* Nothing past this point */

