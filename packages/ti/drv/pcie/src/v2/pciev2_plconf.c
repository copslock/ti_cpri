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
 *  File Name: pciev2_plconf.c
 *
 *  Processing/configuration functions for the PCIe PLCONF Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v2/pcieloc.h>

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **********************
 ****************************************************************************/

  /* Define bitfield positions */
#define PCIEV2_FLTMASK1_CFG_DROP_MASK         (0x00008000U)
#define PCIEV2_FLTMASK1_CFG_DROP_SHIFT        (0x0000000FU)
#define PCIEV2_FLTMASK1_IO_DROP_MASK          (0x00004000U)
#define PCIEV2_FLTMASK1_IO_DROP_SHIFT         (0x0000000EU)
#define PCIEV2_FLTMASK1_MSG_DROP_MASK         (0x00002000U)
#define PCIEV2_FLTMASK1_MSG_DROP_SHIFT        (0x0000000DU)
#define PCIEV2_FLTMASK1_CPL_ECRC_DROP_MASK    (0x00001000U)
#define PCIEV2_FLTMASK1_CPL_ECRC_DROP_SHIFT   (0x0000000CU)

#define PCIEV2_FLTMASK1_ECRC_DROP_MASK        (0x00000800U)
#define PCIEV2_FLTMASK1_ECRC_DROP_SHIFT       (0x0000000BU)
#define PCIEV2_FLTMASK1_CPL_LEN_TEST_MASK     (0x00000400U)
#define PCIEV2_FLTMASK1_CPL_LEN_TEST_SHIFT    (0x0000000AU)
#define PCIEV2_FLTMASK1_CPL_ATTR_TEST_MASK    (0x00000200U)
#define PCIEV2_FLTMASK1_CPL_ATTR_TEST_SHIFT   (0x00000009U)
#define PCIEV2_FLTMASK1_CPL_TC_TEST_MASK      (0x00000100U)
#define PCIEV2_FLTMASK1_CPL_TC_TEST_SHIFT     (0x00000008U)

#define PCIEV2_FLTMASK1_CPL_FUNC_TEST_MASK    (0x00000080U)
#define PCIEV2_FLTMASK1_CPL_FUNC_TEST_SHIFT   (0x00000007U)
#define PCIEV2_FLTMASK1_CPL_REQID_TEST_MASK   (0x00000040U)
#define PCIEV2_FLTMASK1_CPL_REQID_TEST_SHIFT  (0x00000006U)
#define PCIEV2_FLTMASK1_CPL_TAGERR_TEST_MASK  (0x00000020U)
#define PCIEV2_FLTMASK1_CPL_TAGERR_TEST_SHIFT (0x00000005U)
#define PCIEV2_FLTMASK1_LOCKED_RD_AS_UR_MASK  (0x00000010U)
#define PCIEV2_FLTMASK1_LOCKED_RD_AS_UR_SHIFT (0x00000004U)

#define PCIEV2_FLTMASK1_CFG1_RE_AS_US_MASK    (0x00000008U)
#define PCIEV2_FLTMASK1_CFG1_RE_AS_US_SHIFT   (0x00000003U)
#define PCIEV2_FLTMASK1_UR_OUT_OF_BAR_MASK    (0x00000004U)
#define PCIEV2_FLTMASK1_UR_OUT_OF_BAR_SHIFT   (0x00000002U)
#define PCIEV2_FLTMASK1_UR_POISON_MASK        (0x00000002U)
#define PCIEV2_FLTMASK1_UR_POISON_SHIFT       (0x00000001U)
#define PCIEV2_FLTMASK1_UR_FUN_MISMATCH_MASK  (0x00000001U)
#define PCIEV2_FLTMASK1_UR_FUN_MISMATCH_SHIFT (0x00000000U)


/* Define bitfield positions */
#define PCIEV2_FLTMASK2_DROP_PRS_MASK        (0x00000080U)
#define PCIEV2_FLTMASK2_DROP_PRS_SHIFT       (0x00000007U)
#define PCIEV2_FLTMASK2_UNMASK_TD_MASK       (0x00000040U)
#define PCIEV2_FLTMASK2_UNMASK_TD_SHIFT      (0x00000006U)
#define PCIEV2_FLTMASK2_UNMASK_UR_POIS_MASK  (0x00000020U)
#define PCIEV2_FLTMASK2_UNMASK_UR_POIS_SHIFT (0x00000005U)
#define PCIEV2_FLTMASK2_DROP_LN_MASK         (0x00000010U)
#define PCIEV2_FLTMASK2_DROP_LN_SHIFT        (0x00000004U)
#define PCIEV2_FLTMASK2_FLUSH_REQ_MASK       (0x00000008U)
#define PCIEV2_FLTMASK2_FLUSH_REQ_SHIFT      (0x00000003U)
#define PCIEV2_FLTMASK2_DLLP_ABORT_MASK      (0x00000004U)
#define PCIEV2_FLTMASK2_DLLP_ABORT_SHIFT     (0x00000002U)
#define PCIEV2_FLTMASK2_VMSG1_DROP_MASK      (0x00000002U)
#define PCIEV2_FLTMASK2_VMSG1_DROP_SHIFT     (0x00000001U)
#define PCIEV2_FLTMASK2_VMSG0_DROP_MASK      (0x00000001U)
#define PCIEV2_FLTMASK2_VMSG0_DROP_SHIFT     (0x00000000U)

/* DEBUG0 */
#define PCIEV2_DEBUG0_TS_LINK_CTRL_MASK      (0xF0000000u)
#define PCIEV2_DEBUG0_TS_LINK_CTRL_SHIFT     (0x0000001Cu)

#define PCIEV2_DEBUG0_TS_LANE_K237_MASK      (0x08000000u)
#define PCIEV2_DEBUG0_TS_LANE_K237_SHIFT     (0x0000001Bu)

#define PCIEV2_DEBUG0_TS_LINK_K237_MASK      (0x04000000u)
#define PCIEV2_DEBUG0_TS_LINK_K237_SHIFT     (0x0000001Au)

#define PCIEV2_DEBUG0_RCVD_IDLE0_MASK        (0x02000000u)
#define PCIEV2_DEBUG0_RCVD_IDLE0_SHIFT       (0x00000019u)

#define PCIEV2_DEBUG0_RCVD_IDLE1_MASK        (0x01000000u)
#define PCIEV2_DEBUG0_RCVD_IDLE1_SHIFT       (0x00000018u)

#define PCIEV2_DEBUG0_PIPE_TXDATA_MASK       (0x00FFFF00u)
#define PCIEV2_DEBUG0_PIPE_TXDATA_SHIFT      (0x00000008u)

#define PCIEV2_DEBUG0_PIPE_TXDATAK_MASK      (0x000000C0u)
#define PCIEV2_DEBUG0_PIPE_TXDATAK_SHIFT     (0x00000006u)

#define PCIEV2_DEBUG0_TXB_SKIP_TX_MASK       (0x00000020u)
#define PCIEV2_DEBUG0_TXB_SKIP_TX_SHIFT      (0x00000005u)

#define PCIEV2_DEBUG0_LTSSM_STATE_MASK       (0x0000001Fu)
#define PCIEV2_DEBUG0_LTSSM_STATE_SHIFT      (0x00000000u)

/* DEBUG1 */

#define PCIEV2_DEBUG1_SCRAMBLER_DISABLE_MASK (0x80000000u)
#define PCIEV2_DEBUG1_SCRAMBLER_DISABLE_SHIFT (0x0000001Fu)

#define PCIEV2_DEBUG1_LINK_DISABLE_MASK      (0x40000000u)
#define PCIEV2_DEBUG1_LINK_DISABLE_SHIFT     (0x0000001Eu)

#define PCIEV2_DEBUG1_LINK_IN_TRAINING_MASK  (0x20000000u)
#define PCIEV2_DEBUG1_LINK_IN_TRAINING_SHIFT (0x0000001Du)

#define PCIEV2_DEBUG1_RCVR_REVRS_POL_EN_MASK (0x10000000u)
#define PCIEV2_DEBUG1_RCVR_REVRS_POL_EN_SHIFT (0x0000001Cu)

#define PCIEV2_DEBUG1_TRAINING_RST_N_MASK    (0x08000000u)
#define PCIEV2_DEBUG1_TRAINING_RST_N_SHIFT   (0x0000001Bu)

#define PCIEV2_DEBUG1_PIPE_TXDETECTRX_LB_MASK (0x00400000u)
#define PCIEV2_DEBUG1_PIPE_TXDETECTRX_LB_SHIFT (0x00000016u)

#define PCIEV2_DEBUG1_PIPE_TXELECIDLE_MASK   (0x00200000u)
#define PCIEV2_DEBUG1_PIPE_TXELECIDLE_SHIFT  (0x00000015u)

#define PCIEV2_DEBUG1_PIPE_TXCOMPLIANCE_MASK (0x00100000u)
#define PCIEV2_DEBUG1_PIPE_TXCOMPLIANCE_SHIFT (0x00000014u)

#define PCIEV2_DEBUG1_APP_INIT_RST_MASK      (0x00080000u)
#define PCIEV2_DEBUG1_APP_INIT_RST_SHIFT     (0x00000013u)

#define PCIEV2_DEBUG1_RMLH_TS_LINK_NUM_MASK  (0x0000FF00u)
#define PCIEV2_DEBUG1_RMLH_TS_LINK_NUM_SHIFT (0x00000008u)

#define PCIEV2_DEBUG1_XMLH_LINK_UP_MASK      (0x00000010u)
#define PCIEV2_DEBUG1_XMLH_LINK_UP_SHIFT     (0x00000004u)

#define PCIEV2_DEBUG1_RMLH_INSKIP_RCV_MASK   (0x00000008u)
#define PCIEV2_DEBUG1_RMLH_INSKIP_RCV_SHIFT  (0x00000003u)

#define PCIEV2_DEBUG1_RMLH_TS1_RCVD_MASK     (0x00000004u)
#define PCIEV2_DEBUG1_RMLH_TS1_RCVD_SHIFT    (0x00000002u)

#define PCIEV2_DEBUG1_RMLH_TS2_RCVD_MASK     (0x00000002u)
#define PCIEV2_DEBUG1_RMLH_TS2_RCVD_SHIFT    (0x00000001u)

#define PCIEV2_DEBUG1_RMLH_RCVD_LANE_REV_MASK (0x00000001u)
#define PCIEV2_DEBUG1_RMLH_RCVD_LANE_REV_SHIFT (0x00000000u)

/*****************************************************************************
 * These APIs work on both EP and RC.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
pcieRet_e pciev2_read_plAckTimer_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlAckTimerReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ACK_LATENCY_TIMER_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT,             swReg->rplyLmt);

  return pcie_RET_OK;
} /* pciev2_read_plAckTimer_reg */


/*****************************************************************************
 * Combine and write the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
pcieRet_e pciev2_write_plAckTimer_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlAckTimerReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_ROUND_TRIP_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_LATENCY_TIMER_OFF_REPLAY_TIME_LIMIT,             swReg->rplyLmt);

  baseAddr->ACK_LATENCY_TIMER_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plAckTimer_reg */


/*****************************************************************************
 * Read and split up the PL CONF Vendor Specific DLLP register
 ****************************************************************************/
pcieRet_e pciev2_read_plOMsg_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlOMsgReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VENDOR_SPEC_DLLP_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP, swReg->oMsg);

  return pcie_RET_OK;
} /* pciev2_read_plOMsg_reg */


/*****************************************************************************
 * Combine and write the PL CONF Vendor Specific DLL register
 ****************************************************************************/
pcieRet_e pciev2_write_plOMsg_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlOMsgReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VENDOR_SPEC_DLLP_OFF_VENDOR_SPEC_DLLP, swReg->oMsg);

  baseAddr->VENDOR_SPEC_DLLP_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plOMsg_reg */


/*****************************************************************************
 * Read and split up the PL CONF Port Force Link register
 ****************************************************************************/
pcieRet_e pciev2_read_plForceLink_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlForceLinkReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PORT_FORCE_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_NUM,           swReg->linkNum);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCE_EN,           swReg->forceLink);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_STATE,         swReg->lnkState);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCED_LTSSM,       swReg->forcedLtssmState);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS, swReg->doDeskewForSris);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->lpeCnt = 0u;

  return pcie_RET_OK;
} /* pciev2_read_plForceLink_reg */


/*****************************************************************************
 * Combine and write the PL CONF Port Force Link register
 ****************************************************************************/
pcieRet_e pciev2_write_plForceLink_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlForceLinkReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_NUM,           swReg->linkNum);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCE_EN,           swReg->forceLink);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_LINK_STATE,         swReg->lnkState);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_FORCED_LTSSM,       swReg->forcedLtssmState);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_FORCE_OFF_DO_DESKEW_FOR_SRIS, swReg->doDeskewForSris);

  baseAddr->PORT_FORCE_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plForceLink_reg */


/*****************************************************************************
 * Read and split up the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
pcieRet_e pciev2_read_ackFreq_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieAckFreqReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ACK_F_ASPM_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ,             swReg->ackFreq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS,            swReg->nFts);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS,     swReg->commNFts);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY, swReg->l0sEntryLatency);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY,  swReg->l1EntryLatency);
  pcie_getbits(val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM,           swReg->aspmL1);

  return pcie_RET_OK;
} /* pciev2_read_ackFreq_reg */


/*****************************************************************************
 * Combine and write the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
pcieRet_e pciev2_write_ackFreq_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieAckFreqReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_FREQ,             swReg->ackFreq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ACK_N_FTS,            swReg->nFts);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_COMMON_CLK_N_FTS,     swReg->commNFts);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L0S_ENTRANCE_LATENCY, swReg->l0sEntryLatency);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_L1_ENTRANCE_LATENCY,  swReg->l1EntryLatency);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_ACK_F_ASPM_CTRL_OFF_ENTER_ASPM,           swReg->aspmL1);

  baseAddr->ACK_F_ASPM_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_ackFreq_reg */


/*****************************************************************************
 * Read and split up the PL CONF Port Link Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_lnkCtrl_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieLnkCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PORT_LINK_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_VENDOR_SPECIFIC_DLLP_REQ, swReg->msgReq);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_SCRAMBLE_DISABLE,         swReg->scrmDis);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_LOOPBACK_ENABLE,          swReg->lpbkEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_RESET_ASSERT,             swReg->rstAsrt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_DLL_LINK_EN,              swReg->dllEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_FAST_LINK_MODE,           swReg->fLnkMode);
  pcie_getbits(val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_LINK_CAPABLE,             swReg->lnkMode);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->lnkRate = 0;
  swReg->crosslinkAct = 0;
  swReg->crosslinkEn  = 0;

  return pcie_RET_OK;
} /* pciev2_read_lnkCtrl_reg */


/*****************************************************************************
 * Combine and write the PL CONF Port Link Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_lnkCtrl_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieLnkCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_VENDOR_SPECIFIC_DLLP_REQ, swReg->msgReq);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_SCRAMBLE_DISABLE,         swReg->scrmDis);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_LOOPBACK_ENABLE,          swReg->lpbkEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_RESET_ASSERT,             swReg->rstAsrt);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_DLL_LINK_EN,              swReg->dllEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_FAST_LINK_MODE,           swReg->fLnkMode);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PORT_LINK_CTRL_OFF_LINK_CAPABLE,             swReg->lnkMode);

  baseAddr->PORT_LINK_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_lnkCtrl_reg */


/*****************************************************************************
 * Read and split up the PL CONF Lane Skew register
 ****************************************************************************/
pcieRet_e pciev2_read_laneSkew_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieLaneSkewReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LANE_SKEW_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW,            swReg->laneSkew);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE,           swReg->fcDisable);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE,             swReg->ackDisable);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES,         swReg->implementNumLanes);
  pcie_getbits(val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW, swReg->l2Deskew);

  return pcie_RET_OK;
} /* pciev2_read_laneSkew_reg */


/*****************************************************************************
 * Combine and write the PL CONF Lane Skew register
 ****************************************************************************/
pcieRet_e pciev2_write_laneSkew_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieLaneSkewReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_INSERT_LANE_SKEW,            swReg->laneSkew);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_FLOW_CTRL_DISABLE,           swReg->fcDisable);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_ACK_NAK_DISABLE,             swReg->ackDisable);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_IMPLEMENT_NUM_LANES,         swReg->implementNumLanes);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_LANE_SKEW_OFF_DISABLE_LANE_TO_LANE_DESKEW, swReg->l2Deskew);

  baseAddr->LANE_SKEW_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_laneSkew_reg */


/*****************************************************************************
 * Read and split up the PL CONF Timer Control and Symbol Number (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_symNum_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieSymNumReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TIMER_CTRL_MAX_FUNC_NUM_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM,             swReg->maxFunc);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER,   swReg->replayTimer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK,        swReg->ackLatencyTimer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR, swReg->fastLinkScalingFactor);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->fcWatchTimer  = 0;
  swReg->skpCount      = 0;
  swReg->numTs2Symbols = 0;
  swReg->tsCount       = 0;

  return pcie_RET_OK;
} /* pciev2_read_symNum_reg */


/*****************************************************************************
 * Combine and write the PL CONF Timer Control and Symbol Number (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_symNum_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieSymNumReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_MAX_FUNC_NUM,             swReg->maxFunc);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_REPLAY_TIMER,   swReg->replayTimer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_TIMER_MOD_ACK_NAK,        swReg->ackLatencyTimer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_TIMER_CTRL_MAX_FUNC_NUM_OFF_FAST_LINK_SCALING_FACTOR, swReg->fastLinkScalingFactor);

  baseAddr->TIMER_CTRL_MAX_FUNC_NUM_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_symNum_reg */


/*****************************************************************************
 * Read and split up the PL CONF Symbol Timer and Filter Mask (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_symTimerFltMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SYMBOL_TIMER_FILTER_1_OFF;
  uint32_t mask1;

  pcie_getbits(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL,         swReg->skpValue);
  pcie_getbits(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER, swReg->fcWdogDisable);

  /* Extract MASK_RADM_1 */
  pcie_getbits(val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1,         mask1);

  /* Repack into named bits per rev 0, they actually match */
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CFG_DROP,        swReg->f1CfgDrop);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_IO_DROP,         swReg->f1IoDrop);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_MSG_DROP,        swReg->f1MsgDrop);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_ECRC_DROP,   swReg->f1CplEcrcDrop);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_ECRC_DROP,       swReg->f1EcrcDrop);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_LEN_TEST,    swReg->f1CplLenTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_ATTR_TEST,   swReg->f1CplAttrTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_TC_TEST,     swReg->f1CplTcTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_FUNC_TEST,   swReg->f1CplFuncTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_REQID_TEST,  swReg->f1CplReqIDTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CPL_TAGERR_TEST, swReg->f1CplTagErrTest);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_LOCKED_RD_AS_UR, swReg->f1LockedRdAsUr);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_CFG1_RE_AS_US,   swReg->f1Cfg1ReAsUs);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_UR_OUT_OF_BAR,   swReg->f1UrOutOfBar);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_UR_POISON,       swReg->f1UrPoison);
  pcie_getbits(mask1, PCIEV2_FLTMASK1_UR_FUN_MISMATCH, swReg->f1UrFunMismatch);

  return pcie_RET_OK;
} /* pciev2_read_symTimerFltMask_reg */


/*****************************************************************************
 * Combine and write the PL CONF Symbol Timer and Filter Mask (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_symTimerFltMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;
  uint32_t mask1 = 0;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_SKP_INT_VAL,         swReg->skpValue);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_DISABLE_FC_WD_TIMER, swReg->fcWdogDisable);

  /* Repack into named bits per rev 0, they actually match */
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CFG_DROP,         swReg->f1CfgDrop);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_IO_DROP,          swReg->f1IoDrop);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_MSG_DROP,         swReg->f1MsgDrop);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_ECRC_DROP,    swReg->f1CplEcrcDrop);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_ECRC_DROP,        swReg->f1EcrcDrop);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_LEN_TEST,     swReg->f1CplLenTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_ATTR_TEST,    swReg->f1CplAttrTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_TC_TEST,      swReg->f1CplTcTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_FUNC_TEST,    swReg->f1CplFuncTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_REQID_TEST,   swReg->f1CplReqIDTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CPL_TAGERR_TEST,  swReg->f1CplTagErrTest);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_LOCKED_RD_AS_UR,  swReg->f1LockedRdAsUr);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_CFG1_RE_AS_US,    swReg->f1Cfg1ReAsUs);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_UR_OUT_OF_BAR,    swReg->f1UrOutOfBar);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_UR_POISON,        swReg->f1UrPoison);
  pcie_setbits(mask1, PCIEV2_FLTMASK1_UR_FUN_MISMATCH,  swReg->f1UrFunMismatch);

  /* Put mask into register image */
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_SYMBOL_TIMER_FILTER_1_OFF_MASK_RADM_1,         mask1);

  baseAddr->SYMBOL_TIMER_FILTER_1_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfSymbTR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Filter Mask 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_fltMask2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieFltMask2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->FILTER_MASK_2_OFF;

  /* hw rev 0 bits are still valid plus 4 new ones.  Other 24 bits are reserved */
  pcie_getbits(val, PCIEV2_FLTMASK2_DROP_PRS,       swReg->dropPRS);
  pcie_getbits(val, PCIEV2_FLTMASK2_UNMASK_TD,      swReg->unmaskTD);
  pcie_getbits(val, PCIEV2_FLTMASK2_UNMASK_UR_POIS, swReg->unmaskUrPOIS);
  pcie_getbits(val, PCIEV2_FLTMASK2_DROP_LN,        swReg->dropLN);
  pcie_getbits(val, PCIEV2_FLTMASK2_FLUSH_REQ,      swReg->flushReq);
  pcie_getbits(val, PCIEV2_FLTMASK2_DLLP_ABORT,     swReg->dllpAbort);
  pcie_getbits(val, PCIEV2_FLTMASK2_VMSG1_DROP,     swReg->vmsg1Drop);
  pcie_getbits(val, PCIEV2_FLTMASK2_VMSG0_DROP,     swReg->vmsg0Drop);

  return pcie_RET_OK;
} /* pciev2_read_fltMask2_reg */


/*****************************************************************************
 * Combine and write the PL CONF Filter Mask 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_fltMask2_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieFltMask2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, PCIEV2_FLTMASK2_DROP_PRS,       swReg->dropPRS);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_UNMASK_TD,      swReg->unmaskTD);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_UNMASK_UR_POIS, swReg->unmaskUrPOIS);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_DROP_LN,        swReg->dropLN);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_FLUSH_REQ,      swReg->flushReq);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_DLLP_ABORT,     swReg->dllpAbort);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_VMSG1_DROP,     swReg->vmsg1Drop);
  pcie_setbits(new_val, PCIEV2_FLTMASK2_VMSG0_DROP,     swReg->vmsg0Drop);

  baseAddr->FILTER_MASK_2_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_fltMask2_reg */


/*****************************************************************************
 * Read and split up the PL CONF AXI Multiple Outbound Decomposed NP
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfObnpSubreqCtrl_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN, swReg->enObnpSubreq);

  return pcie_RET_OK;
} /* pciev2_read_plconfObnpSubreqCtrl_reg */


/*****************************************************************************
 * Combine and write the PL CONF AXI Multiple Outbound Decomposed NP
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfObnpSubreqCtrl_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF_OB_RD_SPLIT_BURST_EN, swReg->enObnpSubreq);

  baseAddr->AMBA_MUL_OB_DECOMP_NP_SUB_REQ_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfObnpSubreqCtrl_reg */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfTrPStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrPStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TX_P_FC_CREDIT_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_DATA_FC_CREDIT, swReg->pdCrdt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_P_FC_CREDIT_STATUS_OFF_TX_P_HEADER_FC_CREDIT, swReg->phCrdt);

  return pcie_RET_OK;
} /* pciev2_read_plconfTrPStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Non-Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfTrNpStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrNpStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TX_NP_FC_CREDIT_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_DATA_FC_CREDIT, swReg->npdCrdt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_NP_FC_CREDIT_STATUS_OFF_TX_NP_HEADER_FC_CREDIT, swReg->nphCrdt);

  return pcie_RET_OK;
} /* pciev2_read_plconfTrNpStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Non-Posted FC Credit Status
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Completion FC Credit Status
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfTrCStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfTrCStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TX_CPL_FC_CREDIT_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_DATA_FC_CREDIT, swReg->cpldCrdt);
  pcie_getbits(val, CSL_PCIE_EP_CORE_TX_CPL_FC_CREDIT_STATUS_OFF_TX_CPL_HEADER_FC_CREDIT, swReg->cplhCrdt);

  return pcie_RET_OK;
} /* pciev2_read_plconfTrCStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Completion FC Credit Status
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfQStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->QUEUE_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN,  swReg->crdtNotRtrn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE,           swReg->rtybNotEmpty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY,           swReg->rcvqNotEmpty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW,            swReg->rxQueueOverflow);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY, swReg->rxSerQNEmpty);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR, swReg->rxSerQWErr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR,  swReg->rxSerRErr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL,       swReg->fcLatencyOvr);
  pcie_getbits(val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN,    swReg->fcLatencyOvrEn);

  return pcie_RET_OK;
} /* pciev2_read_plconfQStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfQStsR_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_TLP_FC_CREDIT_NON_RETURN,  swReg->crdtNotRtrn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TX_RETRY_BUFFER_NE,           swReg->rtybNotEmpty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_NON_EMPTY,           swReg->rcvqNotEmpty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_QUEUE_OVERFLOW,            swReg->rxQueueOverflow);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_NON_EMPTY, swReg->rxSerQNEmpty);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_WRITE_ERR, swReg->rxSerQWErr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_RX_SERIALIZATION_Q_READ_ERR,  swReg->rxSerRErr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL,       swReg->fcLatencyOvr);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_QUEUE_STATUS_OFF_TIMER_MOD_FLOW_CONTROL_EN,    swReg->fcLatencyOvrEn);

  baseAddr->QUEUE_STATUS_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfQStsR_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 1 (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfVcTrAR1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcTrAR1Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC_TX_ARBI_1_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_0, swReg->wrrVc0);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_1, swReg->wrrVc1);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_2, swReg->wrrVc2);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_1_OFF_WRR_WEIGHT_VC_3, swReg->wrrVc3);

  return pcie_RET_OK;
} /* pciev2_read_plconfVcTrAR1_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC Transmit Arbitration 1 (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 2 (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfVcTrAR2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcTrAR2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC_TX_ARBI_2_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_4, swReg->wrrVc4);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_5, swReg->wrrVc5);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_6, swReg->wrrVc6);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_TX_ARBI_2_OFF_WRR_WEIGHT_VC_7, swReg->wrrVc7);

  return pcie_RET_OK;
} /* pciev2_read_plconfVcTrAR2_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC Transmit Arbitration 2 (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF VC# Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfVcPrQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcPrQCReg_t *swReg,
  int32_t vcNum
)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_P_RX_Q_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT,     swReg->pDcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT,   swReg->pHcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE,      swReg->pDataScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE,       swReg->pHdrScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC, swReg->orderingRules);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q,     swReg->strictVcPriority);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->pQmode = 0u;

  return pcie_RET_OK;
} /* pciev2_read_plconfVcPrQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfVcPrQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcPrQCReg_t *swReg,
  int32_t vcNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_CREDIT,     swReg->pDcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HEADER_CREDIT,   swReg->pHcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_DATA_SCALE,      swReg->pDataScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_P_HDR_SCALE,       swReg->pHdrScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_TLP_TYPE_ORDERING_VC, swReg->orderingRules);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_P_RX_Q_CTRL_OFF_VC_ORDERING_RX_Q,     swReg->strictVcPriority);

  /* Note: not checking vcNum as its internally generated */
  baseAddr->VC_RX_Q_CTRL[vcNum].VC_P_RX_Q_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfVcPrQC_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfVcNprQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcNprQCReg_t *swReg,
  int32_t vcNum
)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_NP_RX_Q_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT,   swReg->npDcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT, swReg->npHcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE,    swReg->npDataScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE,     swReg->npHdrScale);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->npQmode = 0u;

  return pcie_RET_OK;
} /* pciev2_read_plconfVcNprQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfVcNprQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcNprQCReg_t *swReg,
  int32_t vcNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_CREDIT,   swReg->npDcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HEADER_CREDIT, swReg->npHcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_DATA_SCALE,    swReg->npDataScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_NP_RX_Q_CTRL_OFF_VC_NP_HDR_SCALE,     swReg->npHdrScale);

  /* Note: not checking vcNum as its internally generated */
  baseAddr->VC_RX_Q_CTRL[vcNum].VC_NP_RX_Q_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfVcNprQC_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfVcCrQC_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcCrQCReg_t *swReg,
  int32_t vcNum
)
{
  /* Note: not checking vcNum as its internally generated */
  uint32_t val = swReg->raw = baseAddr->VC_RX_Q_CTRL[vcNum].VC_CPL_RX_Q_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT,   swReg->cplDcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT, swReg->cplHcrd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE,    swReg->cplDataScale);
  pcie_getbits(val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE,     swReg->cplHdrScale);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->cplQmode = 0u;

  return pcie_RET_OK;
} /* pciev2_read_plconfVcCrQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfVcCrQC_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfVcCrQCReg_t *swReg,
  int32_t vcNum
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_CREDIT,   swReg->cplDcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HEADER_CREDIT, swReg->cplHcrd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_DATA_SCALE,    swReg->cplDataScale);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_VC_CPL_RX_Q_CTRL_OFF_VC_CPL_HDR_SCALE,     swReg->cplHdrScale);

  /* Note: not checking vcNum as its internally generated */
  baseAddr->VC_RX_Q_CTRL[vcNum].VC_CPL_RX_Q_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfVcCrQC_reg */


/*****************************************************************************
 * Read and split up the Debug 0 register
 ****************************************************************************/
pcieRet_e pciev2_read_debug0_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieDebug0Reg_t            *reg
)
{
  uint32_t val = reg->raw = baseAddr->PL_DEBUG0_OFF;

  pcie_getbits(val, PCIEV2_DEBUG0_TS_LINK_CTRL,  reg->tsLnkCtrl);
  pcie_getbits(val, PCIEV2_DEBUG0_TS_LANE_K237,  reg->tsLaneK237);
  pcie_getbits(val, PCIEV2_DEBUG0_TS_LINK_K237,  reg->tsLinkK237);
  pcie_getbits(val, PCIEV2_DEBUG0_RCVD_IDLE0,    reg->rcvdIdle0);
  pcie_getbits(val, PCIEV2_DEBUG0_RCVD_IDLE1,    reg->rcvdIdle1);
  pcie_getbits(val, PCIEV2_DEBUG0_PIPE_TXDATA,   reg->pipeTxData);
  pcie_getbits(val, PCIEV2_DEBUG0_PIPE_TXDATAK,  reg->pipeTxDataK);
  pcie_getbits(val, PCIEV2_DEBUG0_TXB_SKIP_TX,   reg->skipTx);
  pcie_getbits(val, PCIEV2_DEBUG0_LTSSM_STATE,   reg->ltssmState);

  return pcie_RET_OK;
} /* pciev2_read_debug0_reg */


/*****************************************************************************
 * Read and split up the Debug 1 register
 ****************************************************************************/
pcieRet_e pciev2_read_debug1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieDebug1Reg_t            *reg
)
{
  uint32_t val = reg->raw = baseAddr->PL_DEBUG1_OFF;

  pcie_getbits(val, PCIEV2_DEBUG1_SCRAMBLER_DISABLE,  reg->scramblerDisable);
  pcie_getbits(val, PCIEV2_DEBUG1_LINK_DISABLE,       reg->linkDisable);
  pcie_getbits(val, PCIEV2_DEBUG1_LINK_IN_TRAINING,   reg->linkInTraining);
  pcie_getbits(val, PCIEV2_DEBUG1_RCVR_REVRS_POL_EN,  reg->rcvrRevrsPolEn);
  pcie_getbits(val, PCIEV2_DEBUG1_TRAINING_RST_N,     reg->trainingRstN);
  pcie_getbits(val, PCIEV2_DEBUG1_PIPE_TXDETECTRX_LB, reg->pipeTxdetectrxLb);
  pcie_getbits(val, PCIEV2_DEBUG1_PIPE_TXELECIDLE,    reg->pipeTxelecidle);
  pcie_getbits(val, PCIEV2_DEBUG1_PIPE_TXCOMPLIANCE,  reg->pipeTxcompliance);
  pcie_getbits(val, PCIEV2_DEBUG1_APP_INIT_RST,       reg->appInitRst);
  pcie_getbits(val, PCIEV2_DEBUG1_RMLH_TS_LINK_NUM,   reg->rmlhTsLinkNum);
  pcie_getbits(val, PCIEV2_DEBUG1_XMLH_LINK_UP,       reg->xmlhLinkUp);
  pcie_getbits(val, PCIEV2_DEBUG1_RMLH_INSKIP_RCV,    reg->rmlhInskipRcv);
  pcie_getbits(val, PCIEV2_DEBUG1_RMLH_TS1_RCVD,      reg->rmlhTs1Rcvd);
  pcie_getbits(val, PCIEV2_DEBUG1_RMLH_TS2_RCVD,      reg->rmlhTs2Rcvd);
  pcie_getbits(val, PCIEV2_DEBUG1_RMLH_RCVD_LANE_REV, reg->rmlhRcvdLaneRev);

  return pcie_RET_OK;
} /* pciev2_read_debug1_reg */


/*****************************************************************************
 * Read and split up the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_gen2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pcieGen2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->GEN2_CTRL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ,      swReg->numFts);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_NUM_OF_LANES,           swReg->lnEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_PRE_DET_LANE,           swReg->preDetLane);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN, swReg->autoFlipEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE,    swReg->dirSpd);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE,   swReg->txSwing);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX,      swReg->txCmpl);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS,         swReg->deemph);
  pcie_getbits(val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE,      swReg->gen1EiInference);

  return pcie_RET_OK;
} /* pciev2_read_plconfWidthSpeedCtl_reg */


/*****************************************************************************
 * Combine and write the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_gen2_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pcieGen2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_FAST_TRAINING_SEQ,      swReg->numFts);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_NUM_OF_LANES,           swReg->lnEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_PRE_DET_LANE,           swReg->preDetLane);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_AUTO_LANE_FLIP_CTRL_EN, swReg->autoFlipEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_DIRECT_SPEED_CHANGE,    swReg->dirSpd);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_PHY_TX_CHANGE,   swReg->txSwing);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_CONFIG_TX_COMP_RX,      swReg->txCmpl);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_SEL_DEEMPHASIS,         swReg->deemph);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_GEN2_CTRL_OFF_GEN1_EI_INFERENCE,      swReg->gen1EiInference);

  baseAddr->GEN2_CTRL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfWidthSpeedCtl_reg */


/*****************************************************************************
 * Read and split up the PL CONF PHY Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfPhyStsR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PHY_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PHY_STATUS_OFF_PHY_STATUS, swReg->phySts);

  return pcie_RET_OK;
} /* pciev2_read_plconfPhyStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF PHY Status (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfPhyCtrlR_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PHY_CONTROL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PHY_CONTROL_OFF_PHY_CONTROL, swReg->phyCtrl);

  return pcie_RET_OK;
} /* pciev2_read_plconfPhyCtrlR_reg */


/*****************************************************************************
 * Combine and write the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfPhyCtrlR_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PHY_CONTROL_OFF_PHY_CONTROL, swReg->phyCtrl);

  baseAddr->PHY_CONTROL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfPhyCtrlR_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlAddress_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL_ADDR_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR, swReg->msiCtrlAddress);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlAddress_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlAddress_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_ADDR_OFF_MSI_CTRL_ADDR, swReg->msiCtrlAddress);

  baseAddr->MSI_CTRL_ADDR_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlAddress_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Upper Address
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlUpperAddress_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL_UPPER_ADDR_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR, swReg->msiCtrlUpperAddress);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlUpperAddress_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Upper Address
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlUpperAddress_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_UPPER_ADDR_OFF_MSI_CTRL_UPPER_ADDR, swReg->msiCtrlUpperAddress);

  baseAddr->MSI_CTRL_UPPER_ADDR_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlUpperAddress_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Enable
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlIntEnable_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_EN_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN, swReg->msiCtrlIntEnable);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlIntEnable_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Enable
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlIntEnable_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_EN_OFF_MSI_CTRL_INT_EN, swReg->msiCtrlIntEnable);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_EN_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlIntEnable_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Mask
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlIntMask_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlIntMask_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Mask
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlIntMask_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_MASK_OFF_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlIntMask_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Status
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlIntStatus_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlIntStatus_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Status
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlIntStatus_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_CTRL_INT_STATUS_OFF_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlIntStatus_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller General Purpose IO
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfMsiCtrlGpio_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_GPIO_IO_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG, swReg->msiCtrlGpio);

  return pcie_RET_OK;
} /* pciev2_read_plconfMsiCtrlGpio_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller General Purpose IO
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfMsiCtrlGpio_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MSI_GPIO_IO_OFF_MSI_GPIO_REG, swReg->msiCtrlGpio);

  baseAddr->MSI_GPIO_IO_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfMsiCtrlGpio_reg */


/*****************************************************************************
 * Read and split up the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfPipeLoopback_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PIPE_LOOPBACK_CONTROL_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK, swReg->loopbackEn);

  return pcie_RET_OK;
} /* pciev2_read_plconfPipeLoopback_reg */


/*****************************************************************************
 * Combine and write the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfPipeLoopback_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_PIPE_LOOPBACK_CONTROL_OFF_PIPE_LOOPBACK, swReg->loopbackEn);

  baseAddr->PIPE_LOOPBACK_CONTROL_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfPipeLoopback_reg */


/*****************************************************************************
 * Read and split up the PL CONF DIF Read-Only register Write Enable (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfDbiRoWrEn_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MISC_CONTROL_1_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);
  pcie_getbits(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET, swReg->defaultTarget);
  pcie_getbits(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1, swReg->urCaMask4Trgt1);
  pcie_getbits(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER, swReg->simpReplayTimer);
  pcie_getbits(val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_RSVDP_4, swReg->ariDevNumber);

  return pcie_RET_OK;
} /* pciev2_read_plconfDbiRoWrEn_reg */


/*****************************************************************************
 * Combine and write the PL CONF DIF Read-Only register Write Enable (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfDbiRoWrEn_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_DEFAULT_TARGET, swReg->defaultTarget);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_UR_CA_MASK_4_TRGT1, swReg->urCaMask4Trgt1);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_SIMPLIFIED_REPLAY_TIMER, swReg->simpReplayTimer);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_MISC_CONTROL_1_OFF_RSVDP_4, swReg->ariDevNumber);

  baseAddr->MISC_CONTROL_1_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfDbiRoWrEn_reg */


/*****************************************************************************
 * Read and split up the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfAxiSlvErrResp_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_ERROR_RESPONSE_DEFAULT_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL,   swReg->slaveErrMap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID, swReg->noVidErrMap);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS,      swReg->errorResponseCrs);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP,      swReg->errorResponseMap);

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->dbiErrMap          = 0u;
  swReg->resetTimeoutErrMap = 0u;

  return pcie_RET_OK;
} /* pciev2_read_plconfAxiSlvErrResp_reg */


/*****************************************************************************
 * Combine and write the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfAxiSlvErrResp_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_GLOBAL,   swReg->slaveErrMap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_VENDORID, swReg->noVidErrMap);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_CRS,      swReg->errorResponseCrs);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_ERROR_RESPONSE_DEFAULT_OFF_AMBA_ERROR_RESPONSE_MAP,      swReg->errorResponseMap);

  baseAddr->AMBA_ERROR_RESPONSE_DEFAULT_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfAxiSlvErrResp_reg */


/*****************************************************************************
 * Read and split up the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfAxiSlvTimeout_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->AMBA_LINK_TIMEOUT_OFF;

  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT, swReg->timeoutValue);
  pcie_getbits(val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT, swReg->flushEn);

  return pcie_RET_OK;
} /* pciev2_read_plconfAxiSlvTimeout_reg */


/*****************************************************************************
 * Combine and write the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfAxiSlvTimeout_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_PERIOD_DEFAULT, swReg->timeoutValue);
  pcie_setbits(new_val, CSL_PCIE_EP_CORE_AMBA_LINK_TIMEOUT_OFF_LINK_TIMEOUT_ENABLE_DEFAULT, swReg->flushEn);

  baseAddr->AMBA_LINK_TIMEOUT_OFF = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev2_write_plconfAxiSlvTimeout_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 1 register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegCtrl1_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl1Reg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE,                 swReg->type);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC,                   swReg->tc);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD,                   swReg->td);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR,                 swReg->attr);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM,      swReg->functionNumber);
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE,                 swReg->type);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC,                   swReg->tc);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD,                   swReg->td);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR,                 swReg->attr);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM,      swReg->functionNumber);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  /* Set unused fields to 0 (only used by rev 0/1 hw) */
  swReg->at = 0u;

  return retVal;
} /* pciev2_read_plconfIatuRegCtrl1_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 1 register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegCtrl1_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl1Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TYPE,                 swReg->type);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TC,                   swReg->tc);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_TD,                   swReg->td);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_ATTR,                 swReg->attr);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_OUTBOUND_CTRL_1_FUNC_NUM,      swReg->functionNumber);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TYPE,                 swReg->type);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TC,                   swReg->tc);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_TD,                   swReg->td);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_ATTR,                 swReg->attr);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_INCREASE_REGION_SIZE, swReg->increaseRegionSize);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_1_OFF_INBOUND_CTRL_1_FUNC_NUM,      swReg->functionNumber);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_1_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegCtrl1_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 2 register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegCtrl2_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl2Reg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE,             swReg->messagecode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG,                  swReg->tag);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN,    swReg->tagSubstEn);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS,          swReg->functionNumberMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP,                  swReg->SNP);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD,      swReg->inhibitPayload);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN, swReg->headerSubstEn);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE,       swReg->cfgShiftMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE,          swReg->invertMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN,            swReg->regionEnable);

       /* Set unused fields to 0 (only used by rev 0/1 hw or inbound) */
       swReg->barNumber                        = 0u;
       swReg->tcMatchEnable                    = 0u;
       swReg->tdMatchEnable                    = 0u;
       swReg->attrMatchEnable                  = 0u;
       swReg->atMatchEnable                    = 0u;
       swReg->virtualFunctionNumberMatchEnable = 0u;
       swReg->messageCodeMatchEnable           = 0u;
       swReg->responseCode                     = 0u;
       swReg->fuzzyTypeMatchMode               = 0u;
       swReg->singleAddrLocTransEn             = 0u;
       swReg->matchMode                        = 0u;
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE,                 swReg->messagecode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM,                  swReg->barNumber);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE,      swReg->msgTypeMatchMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN,              swReg->tcMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN,              swReg->tdMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN,            swReg->attrMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN,        swReg->functionNumberMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN,        swReg->messageCodeMatchEnable);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN, swReg->singleAddrLocTransEn);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE,            swReg->responseCode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE,    swReg->fuzzyTypeMatchMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE,           swReg->cfgShiftMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE,              swReg->invertMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE,               swReg->matchMode);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN,                swReg->regionEnable);

       /* Set unused fields to 0 (only used by rev 0/1 hw or outbound) */
       swReg->tag                              = 0u;
       swReg->tagSubstEn                       = 0u;
       swReg->atMatchEnable                    = 0u;
       swReg->virtualFunctionNumberMatchEnable = 0u;
       swReg->SNP                              = 0u;
       swReg->inhibitPayload                   = 0u;
       swReg->headerSubstEn                    = 0u;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegCtrl2_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 2 register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegCtrl2_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegCtrl2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_MSG_CODE,             swReg->messagecode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG,                  swReg->tag);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_TAG_SUBSTITUTE_EN,    swReg->tagSubstEn);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_FUNC_BYPASS,          swReg->functionNumberMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_SNP,                  swReg->SNP);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INHIBIT_PAYLOAD,      swReg->inhibitPayload);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_HEADER_SUBSTITUTE_EN, swReg->headerSubstEn);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_CFG_SHIFT_MODE,       swReg->cfgShiftMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_INVERT_MODE,          swReg->invertMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_OUTBOUND_REGION_EN,            swReg->regionEnable);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE,                 swReg->messagecode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_BAR_NUM,                  swReg->barNumber);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_TYPE_MATCH_MODE,      swReg->msgTypeMatchMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TC_MATCH_EN,              swReg->tcMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_TD_MATCH_EN,              swReg->tdMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_ATTR_MATCH_EN,            swReg->attrMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUNC_NUM_MATCH_EN,        swReg->functionNumberMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MSG_CODE_MATCH_EN,        swReg->messageCodeMatchEnable);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_SINGLE_ADDR_LOC_TRANS_EN, swReg->singleAddrLocTransEn);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_RESPONSE_CODE,            swReg->responseCode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_FUZZY_TYPE_MATCH_CODE,    swReg->fuzzyTypeMatchMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_CFG_SHIFT_MODE,           swReg->cfgShiftMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_INVERT_MODE,              swReg->invertMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_MATCH_MODE,               swReg->matchMode);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_REGION_CTRL_2_OFF_INBOUND_REGION_EN,                swReg->regionEnable);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_REGION_CTRL_2_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegCtrl2_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegLowerBase_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerBaseReg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW, swReg->zero);
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW, swReg->zero);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegLowerBase_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegLowerBase_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerBaseReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_OUTBOUND_LWR_BASE_HW, swReg->zero);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_RW, swReg->iatuRegLowerBase);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_BASE_ADDR_OFF_INBOUND_LWR_BASE_HW, swReg->zero);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_BASE_ADDR_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegLowerBase_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegUpperBase_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperBaseReg_t *swReg
)
{

  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegUpperBase_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegUpperBase_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperBaseReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_BASE_ADDR_OFF_INBOUND_UPPER_BASE_RW, swReg->iatuRegUpperBase);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_BASE_ADDR_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegUpperBase_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegLimit_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLimitReg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW, swReg->ones);
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW, swReg->ones);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegLimit_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegLimit_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLimitReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_OUTBOUND_LIMIT_ADDR_HW, swReg->ones);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_RW, swReg->iatuRegLimit);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LIMIT_ADDR_OFF_INBOUND_LIMIT_ADDR_HW, swReg->ones);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_LIMIT_ADDR_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegLimit_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegLowerTarget_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerTargetReg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND, swReg->iatuRegLowerTarget);
       swReg->zero = swReg->iatuRegLowerTarget & 0xffffu;
       swReg->iatuRegLowerTarget >>= 16;
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW, swReg->iatuRegLowerTarget);
       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW, swReg->zero);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegLowerTarget_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegLowerTarget_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegLowerTargetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_LWR_TARGET_RW_OUTBOUND,
                    (swReg->iatuRegLowerTarget << 16) | (swReg->zero & 0xffffu));
       swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_RW, swReg->iatuRegLowerTarget);
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_LWR_TARGET_ADDR_OFF_INBOUND_LWR_TARGET_HW, swReg->zero);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_LWR_TARGET_ADDR_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegLowerTarget_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
pcieRet_e pciev2_read_plconfIatuRegUpperTarget_reg
(
  const CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperTargetReg_t *swReg
)
{
  pcieRet_e retVal = pcie_RET_OK;
  uint32_t val;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
    }
    else
    {
       /* INBOUND */
       val = swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_INBOUND;

       pcie_getbits(val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_read_plconfIatuRegUpperTarget_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
pcieRet_e pciev2_write_plconfIatuRegUpperTarget_reg
(
  CSL_pcie_ep_coreRegs *baseAddr,
  const pciePlconfIatuIndexReg_t *simIatuWindow,
  pciePlconfIatuRegUpperTargetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcieRet_e retVal = pcie_RET_OK;
  /* Don't need to check NULL this is internal API */
  uint8_t regionIndex = simIatuWindow->regionIndex;

  pcie_range_check_begin;

  if (regionIndex < 16)
  {
    if (simIatuWindow->regionDirection == 0U)
    {
       /* 0U == OUTBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);
       swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND = new_val;
    }
    else
    {
       /* INBOUND */
       pcie_setbits(new_val, CSL_PCIE_EP_CORE_IATU_UPPER_TARGET_ADDR_OFF_INBOUND_UPPER_TARGET_RW, swReg->iatuRegUpperTarget);

       swReg->raw = baseAddr->iatu[regionIndex].IATU_UPPER_TARGET_ADDR_OFF_INBOUND = new_val;

       retVal = pcie_range_check_return;
    }
  }
  else
  {
    retVal = pcie_RET_RANGECHK;
  }

  return retVal;
} /* pciev2_write_plconfIatuRegUpperTarget_reg */


/* Nothing past this point */

