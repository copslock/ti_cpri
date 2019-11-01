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
 *  File Name: pciev2_app.c
 *
 *  Processing/configuration functions for the PCIe Application Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v2/pcieloc.h>


/*****************************************************************************
 **********  PCIe CONFIG REGISTERS COMMON TO TYPE0 AND TYPE1  *****************
 ****************************************************************************/


/*****************************************************************************
 * These APIs are using the endpoint (Type 0) structure and #defines, but they
 * should be used for both EP and RC (Type 0 and Type 1) PCIe modes.
 * Both types have the same swRegister layout, in the same location.
 ****************************************************************************/

/*****************************************************************************
 **********  PCIe APPLICATION REGISTERS  *****************
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the Peripheral Version and ID swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pid_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePidReg_t               *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PID;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PID_MODID,  swReg->modId);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PID_RTL,    swReg->rtl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PID_MAJOR,  swReg->revMaj);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PID_CUSTOM, swReg->cust);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PID_MINOR,  swReg->revMin);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->scheme = 0u;
  swReg->func   = 0u;

  return pcie_RET_OK;
} /* pciev2_read_pid_reg */

/*****************************************************************************
 * Read and split up the Command Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_cmdStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieCmdStatusReg_t         *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CMD_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_CMD_STATUS_RX_LANE_FLIP_EN, swReg->rxLaneFlipEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CMD_STATUS_TX_LANE_FLIP_EN, swReg->txLaneFlipEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CMD_STATUS_DBI_CS2,         swReg->dbi);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CMD_STATUS_APP_RETRY_EN,    swReg->appRetryEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_CMD_STATUS_LTSSM_EN,        swReg->ltssmEn);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->postedWrEn = 0u;
  swReg->ibXltEn    = 0u;
  swReg->obXltEn    = 0u;

  return pcie_RET_OK;
} /* pciev2_read_cmdStatus_reg */

/*****************************************************************************
 * Combine and write the Command Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_cmdStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieCmdStatusReg_t   *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_RX_LANE_FLIP_EN, swReg->rxLaneFlipEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_TX_LANE_FLIP_EN, swReg->txLaneFlipEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_DBI_CS2,         swReg->dbi);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_APP_RETRY_EN,    swReg->appRetryEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_CMD_STATUS_LTSSM_EN,        swReg->ltssmEn);

  baseAddr->CMD_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_cmdStatus_reg */

/*****************************************************************************
 * Read and split up the Reset Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_rstCmd_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieRstCmdReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->RSTCMD;

  pcie_getbits(val, CSL_PCIE_EP_CORE_RSTCMD_FLR_PF_ACTIVE, swReg->flrPfActive);
  pcie_getbits(val, CSL_PCIE_EP_CORE_RSTCMD_INIT_RST,      swReg->initRst);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->flush = 0u;

  return pcie_RET_OK;
} /* pciev2_read_rstCmd_reg */

/*****************************************************************************
 * Combine and write the Reset Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_rstCmd_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieRstCmdReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_RSTCMD_FLR_PF_ACTIVE, swReg->flrPfActive);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_RSTCMD_INIT_RST,      swReg->initRst);

  baseAddr->RSTCMD = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_rstCmd_reg */

 /*****************************************************************************
 * Read and split up the PTM Config Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_ptmCfg_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmCfgReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PTMCFG;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CLK_SEL,       swReg->ptmClkSel);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CONTEXT_VALID, swReg->ptmContextValid);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_MANUAL_UPDATE, swReg->ptmManualUpdate);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PTMCFG_PTM_AUTO_UPDATE,   swReg->ptmAutoUpdate);

  return pcie_RET_OK;
} /* pciev2_read_ptmCfg_reg */

/*****************************************************************************
 * Combine and write the PTM Config Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_ptmCfg_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmCfgReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CLK_SEL,       swReg->ptmClkSel);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_CONTEXT_VALID, swReg->ptmContextValid);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_MANUAL_UPDATE, swReg->ptmManualUpdate);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTMCFG_PTM_AUTO_UPDATE,   swReg->ptmAutoUpdate);

  baseAddr->PTMCFG = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ptmCfg_reg */

/*****************************************************************************
 * Read and split up the Power Management Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pmCmd_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmCmdReg_t             *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PMCMD;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_TURNOFF, swReg->turnOff);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_PE,      swReg->pme);

  return pcie_RET_OK;
} /* pciev2_read_pmCmd_reg */

/*****************************************************************************
 * Combine and write the Power Management Command swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_pmCmd_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePmCmdReg_t       *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_TURNOFF, swReg->turnOff);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMCMD_PM_XMT_PE,      swReg->pme);

  baseAddr->PMCMD = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmCmd_reg */

/*****************************************************************************
 * Read and split up the End of Interrupt swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_irqEOI_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieIrqEOIReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IRQ_EOI;

  pcie_getbits(val, CSL_PCIE_EP_CORE_IRQ_EOI_EOI, swReg->EOI);

  return pcie_RET_OK;
} /* pciev2_read_irqEOI_reg */

/*****************************************************************************
 * Combine and write the End of Interrupt swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_irqEOI_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieIrqEOIReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_IRQ_EOI_EOI, swReg->EOI);

  baseAddr->IRQ_EOI = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_irqEOI_reg */

/*****************************************************************************
 * Read and split up the MSI Interrupt IRQ swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_msiIrq_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqReg_t            *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MMR_IRQ_MMR_IRQ, swReg->msiIrq);

  return pcie_RET_OK;
} /* pciev2_read_msiIrq_reg */


/*****************************************************************************
 * Combine and write the MSI Interrupt IRQ swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_msiIrq_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqReg_t      *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_MMR_IRQ, swReg->msiIrq);

  baseAddr->MMR_IRQ = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiIrq_reg */

/*****************************************************************************
 * Read and split up the Endpoint Interrupt Request Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_epIrqSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqSetReg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0, swReg->epIrqSet);

  return pcie_RET_OK;
} /* pciev2_read_epIrqSet_reg */


/*****************************************************************************
 * Combine and write the Endpoint Interrupt Request Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_epIrqSet_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqSetReg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_SET_LEGACY_IRQ_SET_0, swReg->epIrqSet);

  baseAddr->LEGACY_IRQ_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epIrqSet_reg */

/*****************************************************************************
 * Read and split up the Endpoint Interrupt Request Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_epIrqClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqClrReg_t          *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0, swReg->epIrqClr);

  return pcie_RET_OK;
} /* pciev2_read_epIrqClr_reg */

/*****************************************************************************
 * Combine and write the Endpoint Interrupt Request Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_epIrqClr_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqClrReg_t    *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_CLR_LEGACY_IRQ_CLR_0, swReg->epIrqClr);

  baseAddr->LEGACY_IRQ_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epIrqClr_reg */

/*****************************************************************************
 * Read and split up the Endpoint Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_epIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqStatusReg_t       *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0, swReg->epIrqStatus);

  return pcie_RET_OK;
} /* pciev2_read_epIrqStatus_reg */


/*****************************************************************************
 * Combine and write the Endpoint Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_epIrqStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieEpIrqStatusReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_LEGACY_IRQ_STATUS_0, swReg->epIrqStatus);

  baseAddr->LEGACY_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_epIrqStatus_reg */

/*****************************************************************************
 * Read and split up a General Purpose swRegister [0-3]
 ****************************************************************************/
pcieRet_e pciev2_read_genPurpose_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieGenPurposeReg_t        *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  swReg->raw = swReg->genPurpose = baseAddr->GPR[swRegNum];

  return pcie_RET_OK;
} /* pciev2_read_epIrqStatus_reg */


/*****************************************************************************
 * Combine and write a General Purpose swRegister [0-3]
 ****************************************************************************/
pcieRet_e pciev2_write_genPurpose_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieGenPurposeReg_t  *swReg,
  int_fast32_t          swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  baseAddr->GPR[swRegNum] = swReg->raw = swReg->genPurpose;

  return pcie_RET_OK;
} /* pciev2_write_epIrqStatus_reg */

/*****************************************************************************
 * Read and split up a MSI Raw Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_msiIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqStatusRawReg_t   *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW, swReg->msiRawStatus);

  return pcie_RET_OK;
} /* pciev2_read_msiIrqStatusRaw_reg */


/*****************************************************************************
 * Combine and write a MSI Raw Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_msiIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieMsiIrqStatusRawReg_t *swReg,
  int_fast32_t              swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_RAW_MMR_IRQ_STATUS_RAW, swReg->msiRawStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS_RAW = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a MSI Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_msiIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqStatusReg_t      *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS, swReg->msiIrqStatus);

  return pcie_RET_OK;
} /* pciev2_read_msiIrqStatus_reg */


/*****************************************************************************
 * Combine and write a MSI Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_msiIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieMsiIrqStatusReg_t *swReg,
  int_fast32_t           swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_STATUS_MMR_IRQ_STATUS, swReg->msiIrqStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiIrqStatus_reg */

/*****************************************************************************
 * Read and split up a MSI Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_msiIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqEnableSetReg_t   *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET, swReg->msiIrqEnSet);

  return pcie_RET_OK;
} /* pciev2_read_msiIrqEnableSet_reg */


/*****************************************************************************
 * Combine and write a MSI Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_msiIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs      *baseAddr,
  pcieMsiIrqEnableSetReg_t  *swReg,
  int_fast32_t               swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_SET_MMR_IRQ_EN_SET, swReg->msiIrqEnSet);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the MSI Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_msiIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieMsiIrqEnableClrReg_t   *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR, swReg->msiIrqEnClr);

  return pcie_RET_OK;
} /* pciev2_read_msiIrqEnableClr_reg */


/*****************************************************************************
 * Combine and write the MSI Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_msiIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieMsiIrqEnableClrReg_t *swReg,
  int_fast32_t              swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MMR_IRQ_ENABLE_CLR_MMR_IRQ_EN_CLR, swReg->msiIrqEnClr);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->MMR_IRQ_FLAGS[swRegNum].MMR_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_msiIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up a Legacy Raw Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_legacyIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqStatusRawReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW, swReg->legacyRawStatus);

  return pcie_RET_OK;
} /* pciev2_read_legacyIrqStatusRaw_reg */


/*****************************************************************************
 * Combine and write a Legacy Raw Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_legacyIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqStatusRawReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_RAW_INT_RAW, swReg->legacyRawStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS_RAW = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_legacyIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Legacy Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_legacyIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieLegacyIrqStatusReg_t   *swReg,
  int_fast32_t                swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_INT, swReg->legacyIrqStatus);

  return pcie_RET_OK;
} /* pciev2_read_legacyIrqStatus_reg */


/*****************************************************************************
 * Combine and write a Legacy Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_legacyIrqStatus_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieLegacyIrqStatusReg_t *swReg,
  int_fast32_t              swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_STATUS_INT, swReg->legacyIrqStatus);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_legacyIrqStatus_reg */

/*****************************************************************************
 * Read and split up a Legacy Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_legacyIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqEnableSetReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_ENABLE_SET_INT_EN_SET, swReg->legacyIrqEnSet);

  return pcie_RET_OK;
} /* pciev2_read_legacyIrqEnableSet_reg */

/*****************************************************************************
 * Combine and write a Legacy Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_legacyIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqEnableSetReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_ENABLE_SET_INT_EN_SET, swReg->legacyIrqEnSet);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_ENABLE_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_legacyIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the Legacy Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_legacyIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs  *baseAddr,
  pcieLegacyIrqEnableClrReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  /* swRegNum generated internally no need for bounds check */
  uint32_t val = swReg->raw = baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LEGACY_IRQ_ENABLE_CLR_INT_EN_CLR, swReg->legacyIrqEnClr);

  return pcie_RET_OK;
} /* pciev2_read_legacyIrqEnableClr_reg */


/*****************************************************************************
 * Combine and write the Legacy Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_legacyIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs        *baseAddr,
  pcieLegacyIrqEnableClrReg_t *swReg,
  int_fast32_t                 swRegNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LEGACY_IRQ_ENABLE_CLR_INT_EN_CLR, swReg->legacyIrqEnClr);

  /* swRegNum generated internally no need for bounds check */
  baseAddr->LEGACY_IRQ_FLAGS[swRegNum].LEGACY_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_legacyIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up a Raw ERR Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_errIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqStatusRawReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW,      swReg->errAer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW,     swReg->errCorr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW, swReg->errNonFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW,      swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return pcie_RET_OK;
} /* pciev2_read_errIrqStatusRaw_reg */


/*****************************************************************************
 * Combine and write a Raw ERR Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_errIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqStatusRawReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_AER_RAW,      swReg->errAer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_CORR_RAW,     swReg->errCorr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_NONFATAL_RAW, swReg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_FATAL_RAW,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_RAW_ERR_SYS_RAW,      swReg->errSys);

  baseAddr->ERR_IRQ_STATUS_RAW = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_errIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Err Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_errIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqStatusReg_t      *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_AER,      swReg->errAer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_CORR,     swReg->errCorr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_NONFATAL, swReg->errNonFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_FATAL,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_SYS,      swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return pcie_RET_OK;
} /* pciev2_read_errIrqStatus_reg */

/*****************************************************************************
 * Combine and write a Err Interrupt Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_errIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pcieErrIrqStatusReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_AER,      swReg->errAer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_CORR,     swReg->errCorr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_NONFATAL, swReg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_FATAL,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_STATUS_ERR_SYS,      swReg->errSys);

  baseAddr->ERR_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_errIrqStatus_reg */

/*****************************************************************************
 * Read and split up a Err Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_errIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqEnableSetReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET,      swReg->errAer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET,     swReg->errCorr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET, swReg->errNonFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET,      swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return pcie_RET_OK;
} /* pciev2_read_errIrqEnableSet_reg */

/*****************************************************************************
 * Combine and write a Err Interrupt Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_errIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqEnableSetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_AER_EN_SET,      swReg->errAer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_CORR_EN_SET,     swReg->errCorr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_NONFATAL_EN_SET, swReg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_FATAL_EN_SET,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_SET_ERR_SYS_EN_SET,      swReg->errSys);

  baseAddr->ERR_IRQ_ENABLE_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_errIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the Err Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_errIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieErrIrqEnableClrReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ERR_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR,      swReg->errAer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR,     swReg->errCorr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR, swReg->errNonFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR,    swReg->errFatal);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR,      swReg->errSys);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->errAxi = 0u;

  return pcie_RET_OK;
} /* pciev2_read_errIrqEnableClr_reg */


/*****************************************************************************
 * Combine and write the Err Interrupt Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_errIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pcieErrIrqEnableClrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_AER_EN_CLR,      swReg->errAer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_CORR_EN_CLR,     swReg->errCorr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_NONFATAL_EN_CLR, swReg->errNonFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_FATAL_EN_CLR,    swReg->errFatal);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ERR_IRQ_ENABLE_CLR_ERR_SYS_EN_CLR,      swReg->errSys);

  baseAddr->ERR_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_errIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up a Raw Power Management and Reset Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pmRstIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqStatusRawReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW, swReg->linkRstReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW,      swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW,   swReg->pmToAck);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW,  swReg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev2_read_pmRstIrqStatusRaw_reg */


/*****************************************************************************
 * Combine and write a Raw Power Management and Reset Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_pmRstIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqStatusRawReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_LNK_RST_REQ_RAW, swReg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_PME_RAW,      swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TO_ACK_RAW,   swReg->pmToAck);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_RAW_PM_TURNOFF_RAW,  swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS_RAW = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmRstIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Power Management and Reset Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pmRstIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqStatusReg_t    *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ, swReg->linkRstReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_PME,      swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TO_ACK,   swReg->pmToAck);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TURNOFF,  swReg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev2_read_pmRstIrqStatus_reg */

/*****************************************************************************
 * Combine and write a Power Management and Reset Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_pmRstIrqStatus_reg
(
  CSL_pcie_ep_coreRegs    *baseAddr,
  pciePmRstIrqStatusReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_LNK_RST_REQ, swReg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_PME,      swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TO_ACK,   swReg->pmToAck);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_STATUS_PM_TURNOFF,  swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmRstIrqStatus_reg */

/*****************************************************************************
 * Read and split up a Power Management and Reset Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pmRstIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqEnableSetReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET, swReg->linkRstReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET,      swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET,   swReg->pmToAck);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET,  swReg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev2_read_pmRstIrqEnableSet_reg */

/*****************************************************************************
 * Combine and write a Power Management and Reset Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_pmRstIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqEnableSetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_LNK_RST_REQ_EN_SET, swReg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_PME_EN_SET,      swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TO_ACK_EN_SET,   swReg->pmToAck);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_SET_PM_TURNOFF_EN_SET,  swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_ENABLE_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmRstIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the Power Management and Reset Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_pmRstIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePmRstIrqEnableClrReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PMRST_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR, swReg->linkRstReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR,      swReg->pmPme);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR,   swReg->pmToAck);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR,  swReg->pmTurnoff);

  return pcie_RET_OK;
} /* pciev2_read_pmRstIrqEnableClr_reg */


/*****************************************************************************
 * Combine and write the Power Management and Reset Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_pmRstIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs       *baseAddr,
  pciePmRstIrqEnableClrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_LNK_RST_REQ_EN_CLR, swReg->linkRstReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_PME_EN_CLR,      swReg->pmPme);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TO_ACK_EN_CLR,   swReg->pmToAck);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PMRST_IRQ_ENABLE_CLR_PM_TURNOFF_EN_CLR,  swReg->pmTurnoff);

  baseAddr->PMRST_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_pmRstIrqEnableClr_reg */

/*****************************************************************************
 * Read and split up a Raw Precision Time Measurement Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_ptmIrqStatusRaw_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqStatusRawReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_STATUS_RAW;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW, swReg->ptmClkUpdated);

  return pcie_RET_OK;
} /* pciev2_read_ptmIrqStatusRaw_reg */


/*****************************************************************************
 * Combine and write a Raw Precision Time Measurement Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_ptmIrqStatusRaw_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqStatusRawReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_RAW_PTM_CLK_UPDATED_RAW, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_STATUS_RAW = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ptmIrqStatusRaw_reg */

/*****************************************************************************
 * Read and split up a Precision Time Measurement Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_ptmIrqStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqStatusReg_t      *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_STATUS;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED, swReg->ptmClkUpdated);

  return pcie_RET_OK;
} /* pciev2_read_ptmIrqStatus_reg */

/*****************************************************************************
 * Combine and write a Precision Time Measurement Status swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_ptmIrqStatus_reg
(
  CSL_pcie_ep_coreRegs  *baseAddr,
  pciePtmIrqStatusReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_STATUS_PTM_CLK_UPDATED, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ptmIrqStatus_reg */

/*****************************************************************************
 * Read and split up a Precision Time Measurement Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_ptmIrqEnableSet_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqEnableSetReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_ENABLE_SET;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET, swReg->ptmClkUpdated);

  return pcie_RET_OK;
} /* pciev2_read_ptmIrqEnableSet_reg */

/*****************************************************************************
 * Combine and write a Precision Time Measurement Enable Set swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_ptmIrqEnableSet_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqEnableSetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_SET_PTM_CLK_UPDATED_EN_SET, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_ENABLE_SET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ptmIrqEnableSet_reg */

/*****************************************************************************
 * Read and split up the Precision Time Measurement Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_read_ptmIrqEnableClr_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePtmIrqEnableClrReg_t   *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PTM_IRQ_ENABLE_CLR;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR, swReg->ptmClkUpdated);

  return pcie_RET_OK;
} /* pciev2_read_ptmIrqEnableClr_reg */


/*****************************************************************************
 * Combine and write the Precision Time Measurement Enable Clear swRegister
 ****************************************************************************/
pcieRet_e pciev2_write_ptmIrqEnableClr_reg
(
  CSL_pcie_ep_coreRegs     *baseAddr,
  pciePtmIrqEnableClrReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PTM_IRQ_ENABLE_CLR_PTM_CLK_UPDATED_EN_CLR, swReg->ptmClkUpdated);

  baseAddr->PTM_IRQ_ENABLE_CLR = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ptmIrqEnableClr_reg */

/* Nothing past this point */

