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
 *  File Name: pciev1_ticonf.c
 *
 *  Processing/configuration functions for the PCIe vendor-specific registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>

/*****************************************************************************
 **********  PCIe TI configuration registers
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the TI CONF Revision register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfRevision_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfRevisionReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->REVISION;

  pcie_getbits(val, CSL_PCIE_REVISION_Y_MINOR, swReg->yMinor);
  pcie_getbits(val, CSL_PCIE_REVISION_CUSTOM,  swReg->custom);
  pcie_getbits(val, CSL_PCIE_REVISION_X_MAJOR, swReg->xMajor);
  pcie_getbits(val, CSL_PCIE_REVISION_R_RTL,   swReg->rRtl);
  pcie_getbits(val, CSL_PCIE_REVISION_FUNC,    swReg->func);
  pcie_getbits(val, CSL_PCIE_REVISION_SCHEME,  swReg->scheme);
  pcie_getbits(val, CSL_PCIE_REVISION_BU,      swReg->bu);

  return pcie_RET_OK;
} /* pciev1_read_tiConfRevision_reg */


/*****************************************************************************
 * Combine and write the TI CONF Revision register
 ****************************************************************************/
/* Not applicable - read-only register */

/*****************************************************************************
 * Read and split up the TI CONF SysConfig register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfSysConfig_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfSysConfigReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SYSCONFIG;

  pcie_getbits(val, CSL_PCIE_SYSCONFIG_IDLEMODE,     swReg->idlemode);
  pcie_getbits(val, CSL_PCIE_SYSCONFIG_STANDBYMODE,  swReg->standbymode);
  pcie_getbits(val, CSL_PCIE_SYSCONFIG_MCOHERENT_EN, swReg->mcoherentEn);

  return pcie_RET_OK;
} /* pciev1_read_tiConfSysConfig_reg */


/*****************************************************************************
 * Combine and write the TI CONF SysConfig register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfSysConfig_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfSysConfigReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_SYSCONFIG_IDLEMODE,     swReg->idlemode);
  pcie_setbits(new_val, CSL_PCIE_SYSCONFIG_STANDBYMODE,  swReg->standbymode);
  pcie_setbits(new_val, CSL_PCIE_SYSCONFIG_MCOHERENT_EN, swReg->mcoherentEn);

  baseAddr->SYSCONFIG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfSysConfig_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ EOI register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqEoi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEoiReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQ_EOI;

  pcie_getbits(val, CSL_PCIE_IRQ_EOI_LINE_NUMBER, swReg->lineNumber);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqEoi_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ EOI register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqEoi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEoiReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQ_EOI_LINE_NUMBER, swReg->lineNumber);

  baseAddr->IRQ_EOI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqEoi_reg */

 
/*****************************************************************************
 * Read and split up the TI CONF IRQ Status Raw Main register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqStatusRawMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMainReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQSTATUS_RAW_MAIN;

  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_SYS,      swReg->errSys);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_FATAL,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_NONFATAL, swReg->errNonfatal);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_COR,      swReg->errCor);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_AXI,      swReg->errAxi);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_ECRC,     swReg->errEcrc);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PME_TO_ACK,   swReg->pmeToAck);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PM_PME,       swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_LINK_REQ_RST, swReg->linkReqRst);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_LINK_UP_EVT,  swReg->linkUpEvt);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_CFG_BME_EVT,  swReg->cfgBmeEvt);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MAIN_CFG_MSE_EVT,  swReg->cfgMseEvt);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqStatusRawMain_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Status Raw Main register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqStatusRawMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMainReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_SYS,      swReg->errSys);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_FATAL,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_NONFATAL, swReg->errNonfatal);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_COR,      swReg->errCor);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_AXI,      swReg->errAxi);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_ERR_ECRC,     swReg->errEcrc);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PME_TO_ACK,   swReg->pmeToAck);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_PM_PME,       swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_LINK_REQ_RST, swReg->linkReqRst);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_LINK_UP_EVT,  swReg->linkUpEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_CFG_BME_EVT,  swReg->cfgBmeEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MAIN_CFG_MSE_EVT,  swReg->cfgMseEvt);

  baseAddr->IRQSTATUS_RAW_MAIN = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqStatusRawMain_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Status Main register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqStatusMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMainReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQSTATUS_MAIN;

  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_SYS,      swReg->errSys);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_FATAL,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_NONFATAL, swReg->errNonfatal);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_COR,      swReg->errCor);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_AXI,      swReg->errAxi);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_ERR_ECRC,     swReg->errEcrc);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_PME_TO_ACK,   swReg->pmeToAck);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_PM_PME,       swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_LINK_REQ_RST, swReg->linkReqRst);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_LINK_UP_EVT,  swReg->linkUpEvt);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_CFG_BME_EVT,  swReg->cfgBmeEvt);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MAIN_CFG_MSE_EVT,  swReg->cfgMseEvt);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqStatusMain_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Status Main register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqStatusMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMainReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_SYS,      swReg->errSys);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_FATAL,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_NONFATAL, swReg->errNonfatal);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_COR,      swReg->errCor);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_AXI,      swReg->errAxi);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_ERR_ECRC,     swReg->errEcrc);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_PME_TO_ACK,   swReg->pmeToAck);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_PM_PME,       swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_LINK_REQ_RST, swReg->linkReqRst);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_LINK_UP_EVT,  swReg->linkUpEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_CFG_BME_EVT,  swReg->cfgBmeEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MAIN_CFG_MSE_EVT,  swReg->cfgMseEvt);

  baseAddr->IRQSTATUS_MAIN = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqStatusMain_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Enable Set Main register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqEnableSetMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMainReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQENABLE_SET_MAIN;

  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_SYS_EN,      swReg->errSys);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_FATAL_EN,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_NONFATAL_EN, swReg->errNonfatal);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_COR_EN,      swReg->errCor);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_AXI_EN,      swReg->errAxi);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_ECRC_EN,     swReg->errEcrc);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_PME_TURN_OFF_EN, swReg->pmeTurnOff);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_PME_TO_ACK_EN,   swReg->pmeToAck);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_PM_PME_EN,       swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_LINK_REQ_RST_EN, swReg->linkReqRst);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_LINK_UP_EVT_EN,  swReg->linkUpEvt);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_CFG_BME_EVT_EN,  swReg->cfgBmeEvt);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MAIN_CFG_MSE_EVT_EN,  swReg->cfgMseEvt);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqEnableSetMain_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Enable Set Main register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqEnableSetMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMainReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_SYS_EN,      swReg->errSys);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_FATAL_EN,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_NONFATAL_EN, swReg->errNonfatal);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_COR_EN,      swReg->errCor);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_AXI_EN,      swReg->errAxi);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_ERR_ECRC_EN,     swReg->errEcrc);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_PME_TURN_OFF_EN, swReg->pmeTurnOff);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_PME_TO_ACK_EN,   swReg->pmeToAck);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_PM_PME_EN,       swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_LINK_REQ_RST_EN, swReg->linkReqRst);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_LINK_UP_EVT_EN,  swReg->linkUpEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_CFG_BME_EVT_EN,  swReg->cfgBmeEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MAIN_CFG_MSE_EVT_EN,  swReg->cfgMseEvt);

  baseAddr->IRQENABLE_SET_MAIN = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqEnableSetMain_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Enable Clr Main register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqEnableClrMain_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMainReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQENABLE_CLR_MAIN;

  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_SYS_EN,      swReg->errSys);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_FATAL_EN,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_NONFATAL_EN, swReg->errNonfatal);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_COR_EN,      swReg->errCor);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_AXI_EN,      swReg->errAxi);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_ECRC_EN,     swReg->errEcrc);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_PME_TURN_OFF_EN, swReg->pmeTurnOff);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_PME_TO_ACK_EN,   swReg->pmeToAck);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_PM_PME_EN,       swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_LINK_REQ_RST_EN, swReg->linkReqRst);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_LINK_UP_EVT_EN,  swReg->linkUpEvt);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_CFG_BME_EVT_EN,  swReg->cfgBmeEvt);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MAIN_CFG_MSE_EVT_EN,  swReg->cfgMseEvt);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqEnableClrMain_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Enable Clr Main register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqEnableClrMain_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMainReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_SYS_EN,      swReg->errSys);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_FATAL_EN,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_NONFATAL_EN, swReg->errNonfatal);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_COR_EN,      swReg->errCor);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_AXI_EN,      swReg->errAxi);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_ERR_ECRC_EN,     swReg->errEcrc);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_PME_TURN_OFF_EN, swReg->pmeTurnOff);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_PME_TO_ACK_EN,   swReg->pmeToAck);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_PM_PME_EN,       swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_LINK_REQ_RST_EN, swReg->linkReqRst);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_LINK_UP_EVT_EN,  swReg->linkUpEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_CFG_BME_EVT_EN,  swReg->cfgBmeEvt);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MAIN_CFG_MSE_EVT_EN,  swReg->cfgMseEvt);

  baseAddr->IRQENABLE_CLR_MAIN = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqEnableClrMain_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Status Raw MSI register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqStatusRawMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMsiReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQSTATUS_RAW_MSI;

  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTA, swReg->inta);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTB, swReg->intb);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTC, swReg->intc);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTD, swReg->intd);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_RAW_MSI_MSI,  swReg->msi);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqStatusRawMsi_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Status Raw MSI register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqStatusRawMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusRawMsiReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTA, swReg->inta);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTB, swReg->intb);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTC, swReg->intc);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MSI_INTD, swReg->intd);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_RAW_MSI_MSI,  swReg->msi);

  baseAddr->IRQSTATUS_RAW_MSI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqStatusRawMsi_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Status MSI register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqStatusMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMsiReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQSTATUS_MSI;

  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MSI_INTA, swReg->inta);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MSI_INTB, swReg->intb);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MSI_INTC, swReg->intc);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MSI_INTD, swReg->intd);
  pcie_getbits(val, CSL_PCIE_IRQSTATUS_MSI_MSI,  swReg->msi);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqStatusMsi_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Status MSI register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqStatusMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqStatusMsiReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MSI_INTA, swReg->inta);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MSI_INTB, swReg->intb);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MSI_INTC, swReg->intc);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MSI_INTD, swReg->intd);
  pcie_setbits(new_val, CSL_PCIE_IRQSTATUS_MSI_MSI,  swReg->msi);

  baseAddr->IRQSTATUS_MSI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqStatusMsi_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Enable Set MSI register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqEnableSetMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMsiReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQENABLE_SET_MSI;

  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MSI_INTA_EN, swReg->inta);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MSI_INTB_EN, swReg->intb);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MSI_INTC_EN, swReg->intc);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MSI_INTD_EN, swReg->intd);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_SET_MSI_MSI_EN,  swReg->msi);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqEnableSetMsi_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Enable Set MSI register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqEnableSetMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableSetMsiReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MSI_INTA_EN, swReg->inta);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MSI_INTB_EN, swReg->intb);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MSI_INTC_EN, swReg->intc);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MSI_INTD_EN, swReg->intd);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_SET_MSI_MSI_EN,  swReg->msi);

  baseAddr->IRQENABLE_SET_MSI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqEnableSetMsi_reg */


/*****************************************************************************
 * Read and split up the TI CONF IRQ Enable Clr MSI register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIrqEnableClrMsi_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMsiReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQENABLE_CLR_MSI;

  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MSI_INTA_EN, swReg->inta);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MSI_INTB_EN, swReg->intb);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MSI_INTC_EN, swReg->intc);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MSI_INTD_EN, swReg->intd);
  pcie_getbits(val, CSL_PCIE_IRQENABLE_CLR_MSI_MSI_EN,  swReg->msi);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIrqEnableClrMsi_reg */


/*****************************************************************************
 * Combine and write the TI CONF IRQ Enable Clr MSI register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIrqEnableClrMsi_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIrqEnableClrMsiReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MSI_INTA_EN, swReg->inta);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MSI_INTB_EN, swReg->intb);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MSI_INTC_EN, swReg->intc);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MSI_INTD_EN, swReg->intd);
  pcie_setbits(new_val, CSL_PCIE_IRQENABLE_CLR_MSI_MSI_EN,  swReg->msi);

  baseAddr->IRQENABLE_CLR_MSI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIrqEnableClrMsi_reg */


/*****************************************************************************
 * Read and split up the TI CONF Device Type register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfDeviceType_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceTypeReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->DEVICE_TYPE;

  pcie_getbits(val, CSL_PCIE_DEVICE_TYPE_TYPE, swReg->type);

  return pcie_RET_OK;
} /* pciev1_read_tiConfDeviceType_reg */


/*****************************************************************************
 * Combine and write the TI CONF Device Type register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfDeviceType_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceTypeReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_DEVICE_TYPE_TYPE, swReg->type);

  baseAddr->DEVICE_TYPE = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfDeviceType_reg */


/*****************************************************************************
 * Read and split up the TI CONF Device CMD register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfDeviceCmd_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceCmdReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->DEVICE_CMD;

  pcie_getbits(val, CSL_PCIE_DEVICE_CMD_LTSSM_STATE,      swReg->ltssmState);
  pcie_getbits(val, CSL_PCIE_DEVICE_CMD_LTSSM_EN,         swReg->ltssmEn);
  pcie_getbits(val, CSL_PCIE_DEVICE_CMD_APP_REQ_RETRY_EN, swReg->appReqRetryEn);
  pcie_getbits(val, CSL_PCIE_DEVICE_CMD_DEV_NUM,          swReg->devNum);
  pcie_getbits(val, CSL_PCIE_DEVICE_CMD_BUS_NUM,          swReg->busNum);

  return pcie_RET_OK;
} /* pciev1_read_tiConfDeviceCmd_reg */


/*****************************************************************************
 * Combine and write the TI CONF Device CMD register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfDeviceCmd_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDeviceCmdReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_DEVICE_CMD_LTSSM_STATE,      swReg->ltssmState);
  pcie_setbits(new_val, CSL_PCIE_DEVICE_CMD_LTSSM_EN,         swReg->ltssmEn);
  pcie_setbits(new_val, CSL_PCIE_DEVICE_CMD_APP_REQ_RETRY_EN, swReg->appReqRetryEn);
  pcie_setbits(new_val, CSL_PCIE_DEVICE_CMD_DEV_NUM,          swReg->devNum);
  pcie_setbits(new_val, CSL_PCIE_DEVICE_CMD_BUS_NUM,          swReg->busNum);

  baseAddr->DEVICE_CMD = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfDeviceCmd_reg */


/*****************************************************************************
 * Read and split up the TI CONF PM Ctrl register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfPmCtrl_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfPmCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PM_CTRL;

  pcie_getbits(val, CSL_PCIE_PM_CTRL_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_getbits(val, CSL_PCIE_PM_CTRL_PM_PME,       swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_PM_CTRL_L23_READY,    swReg->l23Ready);
  pcie_getbits(val, CSL_PCIE_PM_CTRL_REQ_ENTR_L1,  swReg->reqEntrL1);
  pcie_getbits(val, CSL_PCIE_PM_CTRL_REQ_EXIT_L1,  swReg->reqExitL1);
  pcie_getbits(val, CSL_PCIE_PM_CTRL_AUX_PWR_DET,  swReg->auxPwrDet);

  return pcie_RET_OK;
} /* pciev1_read_tiConfPmCtrl_reg */


/*****************************************************************************
 * Combine and write the TI CONF PM Ctrl register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfPmCtrl_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfPmCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_PME_TURN_OFF, swReg->pmeTurnOff);
  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_PM_PME,       swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_L23_READY,    swReg->l23Ready);
  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_REQ_ENTR_L1,  swReg->reqEntrL1);
  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_REQ_EXIT_L1,  swReg->reqExitL1);
  pcie_setbits(new_val, CSL_PCIE_PM_CTRL_AUX_PWR_DET,  swReg->auxPwrDet);

  baseAddr->PM_CTRL = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfPmCtrl_reg */


/*****************************************************************************
 * Read and split up the TI CONF Phy CS register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfPhyCs_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfPhyCsReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PHY_CS;

  pcie_getbits(val, CSL_PCIE_PHY_CS_LINK_UP,       swReg->linkUp);
  pcie_getbits(val, CSL_PCIE_PHY_CS_REVERSE_LANES, swReg->reverseLanes);

  return pcie_RET_OK;
} /* pciev1_read_tiConfPhyCs_reg */


/*****************************************************************************
 * Combine and write the TI CONF Phy CS register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfPhyCs_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfPhyCsReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_PHY_CS_LINK_UP,       swReg->linkUp);
  pcie_setbits(new_val, CSL_PCIE_PHY_CS_REVERSE_LANES, swReg->reverseLanes);

  baseAddr->PHY_CS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfPhyCs_reg */


/*****************************************************************************
 * Read and split up the TI CONF INTX Assert register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIntxAssert_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIntxAssertReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->INTX_ASSERT;

  pcie_getbits(val, CSL_PCIE_INTX_ASSERT_ASSERT_F0, swReg->assertF0);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIntxAssert_reg */


/*****************************************************************************
 * Combine and write the TI CONF INTX Assert register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIntxAssert_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIntxAssertReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_INTX_ASSERT_ASSERT_F0, swReg->assertF0);

  baseAddr->INTX_ASSERT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIntxAssert_reg */


/*****************************************************************************
 * Read and split up the TI CONF INTX Deassert register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfIntxDeassert_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfIntxDeassertReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->INTX_DEASSERT;

  pcie_getbits(val, CSL_PCIE_INTX_DEASSERT_DEASSERT_F0, swReg->deassertF0);

  return pcie_RET_OK;
} /* pciev1_read_tiConfIntxDeassert_reg */


/*****************************************************************************
 * Combine and write the TI CONF INTX Deassert register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfIntxDeassert_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfIntxDeassertReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_INTX_DEASSERT_DEASSERT_F0, swReg->deassertF0);

  baseAddr->INTX_DEASSERT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfIntxDeassert_reg */


/*****************************************************************************
 * Read and split up the TI CONF MSI XMT register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfMsiXmt_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfMsiXmtReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_XMT;

  pcie_getbits(val, CSL_PCIE_MSI_XMT_MSI_REQ_GRANT, swReg->msiReqGrant);
  pcie_getbits(val, CSL_PCIE_MSI_XMT_MSI_FUNC_NUM,  swReg->msiFuncNum);
  pcie_getbits(val, CSL_PCIE_MSI_XMT_MSI_VECTOR,    swReg->msiVector);
  pcie_getbits(val, CSL_PCIE_MSI_XMT_MSI_TC,        swReg->msiTc);

  return pcie_RET_OK;
} /* pciev1_read_tiConfMsiXmt_reg */


/*****************************************************************************
 * Combine and write the TI CONF MSI XMT register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfMsiXmt_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfMsiXmtReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_MSI_XMT_MSI_REQ_GRANT, swReg->msiReqGrant);
  pcie_setbits(new_val, CSL_PCIE_MSI_XMT_MSI_FUNC_NUM,  swReg->msiFuncNum);
  pcie_setbits(new_val, CSL_PCIE_MSI_XMT_MSI_VECTOR,    swReg->msiVector);
  pcie_setbits(new_val, CSL_PCIE_MSI_XMT_MSI_TC,        swReg->msiTc);

  baseAddr->MSI_XMT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfMsiXmt_reg */


/*****************************************************************************
 * Read and split up the TI CONF Debug CFG register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfDebugCfg_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDebugCfgReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->DEBUG_CFG;

  pcie_getbits(val, CSL_PCIE_DEBUG_CFG_SEL, swReg->sel);

  return pcie_RET_OK;
} /* pciev1_read_tiConfDebugCfg_reg */


/*****************************************************************************
 * Combine and write the TI CONF Debug CFG register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfDebugCfg_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDebugCfgReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_DEBUG_CFG_SEL, swReg->sel);

  baseAddr->DEBUG_CFG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfDebugCfg_reg */


/*****************************************************************************
 * Read and split up the TI CONF Debug Data register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfDebugData_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDebugDataReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->DEBUG_DATA;

  pcie_getbits(val, CSL_PCIE_DEBUG_DATA_DEBUG, swReg->debug);

  return pcie_RET_OK;
} /* pciev1_read_tiConfDebugData_reg */

/*****************************************************************************
 * Combine and write the TI CONF Debug Data register
 ****************************************************************************/
/* Not applicable - read-only register */

/*****************************************************************************
 * Read and split up the TI CONF Diag Ctrl register
 ****************************************************************************/
pcieRet_e pciev1_read_tiConfDiagCtrl_reg
(
  const CSL_PcieRegs *baseAddr,
  pcieTiConfDiagCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->DIAG_CTRL;

  pcie_getbits(val, CSL_PCIE_DIAG_CTRL_INV_LCRC,       swReg->invLcrc);
  pcie_getbits(val, CSL_PCIE_DIAG_CTRL_INV_ECRC,       swReg->invEcrc);
  pcie_getbits(val, CSL_PCIE_DIAG_CTRL_FAST_LINK_MODE, swReg->fastLinkMode);

  return pcie_RET_OK;
} /* pciev1_read_tiConfDiagCtrl_reg */


/*****************************************************************************
 * Combine and write the TI CONF Diag Ctrl register
 ****************************************************************************/
pcieRet_e pciev1_write_tiConfDiagCtrl_reg
(
  CSL_PcieRegs *baseAddr,
  pcieTiConfDiagCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_DIAG_CTRL_INV_LCRC,       swReg->invLcrc);
  pcie_setbits(new_val, CSL_PCIE_DIAG_CTRL_INV_ECRC,       swReg->invEcrc);
  pcie_setbits(new_val, CSL_PCIE_DIAG_CTRL_FAST_LINK_MODE, swReg->fastLinkMode);

  baseAddr->DIAG_CTRL = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_tiConfDiagCtrl_reg */

/* Nothing past this point */

