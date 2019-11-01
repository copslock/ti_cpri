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
 *  File Name: pciev1_plconf.c
 *
 *  Processing/configuration functions for the PCIe PLCONF Registers
 *
 */

#include <ti/drv/pcie/pcie.h>
#include <ti/drv/pcie/src/pcieloc.h>
#include <ti/drv/pcie/src/v1/pcieloc.h>

/*****************************************************************************
 **********  PCIe LOCAL/REMOTE PORT LOGIC REGISTERS **********************
 ****************************************************************************/

  /* Define bitfield positions */
#define PCIEV1_FLTMASK1_CFG_DROP_MASK         0x00008000U
#define PCIEV1_FLTMASK1_CFG_DROP_SHIFT        0x0000000FU
#define PCIEV1_FLTMASK1_IO_DROP_MASK          0x00004000U
#define PCIEV1_FLTMASK1_IO_DROP_SHIFT         0x0000000EU
#define PCIEV1_FLTMASK1_MSG_DROP_MASK         0x00002000U
#define PCIEV1_FLTMASK1_MSG_DROP_SHIFT        0x0000000DU
#define PCIEV1_FLTMASK1_CPL_ECRC_DROP_MASK    0x00001000U
#define PCIEV1_FLTMASK1_CPL_ECRC_DROP_SHIFT   0x0000000CU

#define PCIEV1_FLTMASK1_ECRC_DROP_MASK        0x00000800U
#define PCIEV1_FLTMASK1_ECRC_DROP_SHIFT       0x0000000BU
#define PCIEV1_FLTMASK1_CPL_LEN_TEST_MASK     0x00000400U
#define PCIEV1_FLTMASK1_CPL_LEN_TEST_SHIFT    0x0000000AU
#define PCIEV1_FLTMASK1_CPL_ATTR_TEST_MASK    0x00000200U
#define PCIEV1_FLTMASK1_CPL_ATTR_TEST_SHIFT   0x00000009U
#define PCIEV1_FLTMASK1_CPL_TC_TEST_MASK      0x00000100U
#define PCIEV1_FLTMASK1_CPL_TC_TEST_SHIFT     0x00000008U

#define PCIEV1_FLTMASK1_CPL_FUNC_TEST_MASK    0x00000080U
#define PCIEV1_FLTMASK1_CPL_FUNC_TEST_SHIFT   0x00000007U
#define PCIEV1_FLTMASK1_CPL_REQID_TEST_MASK   0x00000040U
#define PCIEV1_FLTMASK1_CPL_REQID_TEST_SHIFT  0x00000006U
#define PCIEV1_FLTMASK1_CPL_TAGERR_TEST_MASK  0x00000020U
#define PCIEV1_FLTMASK1_CPL_TAGERR_TEST_SHIFT 0x00000005U
#define PCIEV1_FLTMASK1_LOCKED_RD_AS_UR_MASK  0x00000010U
#define PCIEV1_FLTMASK1_LOCKED_RD_AS_UR_SHIFT 0x00000004U

#define PCIEV1_FLTMASK1_CFG1_RE_AS_US_MASK    0x00000008U
#define PCIEV1_FLTMASK1_CFG1_RE_AS_US_SHIFT   0x00000003U
#define PCIEV1_FLTMASK1_UR_OUT_OF_BAR_MASK    0x00000004U
#define PCIEV1_FLTMASK1_UR_OUT_OF_BAR_SHIFT   0x00000002U
#define PCIEV1_FLTMASK1_UR_POISON_MASK        0x00000002U
#define PCIEV1_FLTMASK1_UR_POISON_SHIFT       0x00000001U
#define PCIEV1_FLTMASK1_UR_FUN_MISMATCH_MASK  0x00000001U
#define PCIEV1_FLTMASK1_UR_FUN_MISMATCH_SHIFT 0x00000000U


  /* Define bitfield positions */
#define PCIEV1_FLTMASK2_FLUSH_REQ_MASK   0x00000008U
#define PCIEV1_FLTMASK2_FLUSH_REQ_SHIFT  0x00000003U
#define PCIEV1_FLTMASK2_DLLP_ABORT_MASK  0x00000004U
#define PCIEV1_FLTMASK2_DLLP_ABORT_SHIFT 0x00000002U
#define PCIEV1_FLTMASK2_VMSG1_DROP_MASK  0x00000002U
#define PCIEV1_FLTMASK2_VMSG1_DROP_SHIFT 0x00000001U
#define PCIEV1_FLTMASK2_VMSG0_DROP_MASK  0x00000001U
#define PCIEV1_FLTMASK2_VMSG0_DROP_SHIFT 0x00000000U


/*****************************************************************************
 * These APIs work on both EP and RC.
 ****************************************************************************/

/*****************************************************************************
 * Read and split up the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
pcieRet_e pciev1_read_plAckTimer_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlAckTimerReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LAT_REL_TIM;

  pcie_getbits(val, CSL_PLCONF_LAT_REL_TIM_ACK_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
  pcie_getbits(val, CSL_PLCONF_LAT_REL_TIM_REPLAY_TIME_LIMIT,      swReg->rplyLmt);

  return pcie_RET_OK;
} /* pciev1_read_plconfLatRelTim_reg */


/*****************************************************************************
 * Combine and write the PL CONF Ack Latency and Replay Timer register
 ****************************************************************************/
pcieRet_e pciev1_write_plAckTimer_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlAckTimerReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_LAT_REL_TIM_ACK_LATENCY_TIME_LIMIT, swReg->rndTrpLmt);
  pcie_setbits(new_val, CSL_PLCONF_LAT_REL_TIM_REPLAY_TIME_LIMIT,      swReg->rplyLmt);

  baseAddr->LAT_REL_TIM = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfLatRelTim_reg */


/*****************************************************************************
 * Read and split up the PL CONF Vendor Specific DLLP register
 ****************************************************************************/
pcieRet_e pciev1_read_plOMsg_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlOMsgReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VENDOR_SPECIFIC_DLLP;

  pcie_getbits(val, CSL_PLCONF_VENDOR_SPECIFIC_DLLP_VEN_DLLP_REG, swReg->oMsg);

  return pcie_RET_OK;
} /* pciev1_read_plconfVendorSpecificDllp_reg */


/*****************************************************************************
 * Combine and write the PL CONF Vendor Specific DLL register
 ****************************************************************************/
pcieRet_e pciev1_write_plOMsg_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlOMsgReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_VENDOR_SPECIFIC_DLLP_VEN_DLLP_REG, swReg->oMsg);

  baseAddr->VENDOR_SPECIFIC_DLLP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfVendorSpecificDllp_reg */


/*****************************************************************************
 * Read and split up the PL CONF Port Force Link register
 ****************************************************************************/
pcieRet_e pciev1_read_plForceLink_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlForceLinkReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PT_LNK_R;

  pcie_getbits(val, CSL_PLCONF_PT_LNK_R_LINK_NUM,            swReg->linkNum);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_R_FORCE_LINK,          swReg->forceLink);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_R_FORCED_LINK_COMMAND, swReg->lnkState);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_R_LOW_POWER_ENTR_CNT,  swReg->lpeCnt);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_R_FORCED_LTSSM_STATE,  swReg->forcedLtssmState);

  /* Set unused fields to 0 (only used by rev 2 hw) */
  swReg->doDeskewForSris = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfPtLnkR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Port Force Link register
 ****************************************************************************/
pcieRet_e pciev1_write_plForceLink_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlForceLinkReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_R_LINK_NUM,            swReg->linkNum);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_R_FORCE_LINK,          swReg->forceLink);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_R_FORCED_LINK_COMMAND, swReg->lnkState);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_R_LOW_POWER_ENTR_CNT,  swReg->lpeCnt);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_R_FORCED_LTSSM_STATE,  swReg->forcedLtssmState);

  baseAddr->PT_LNK_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfPtLnkR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
pcieRet_e pciev1_read_ackFreq_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieAckFreqReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->ACK_FREQ_ASPM;

  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_ACK_FREQ,         swReg->ackFreq);
  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_N_FTS,            swReg->nFts);
  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_COMMOM_CLK_N_FTS, swReg->commNFts);
  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_L0S_ENTR_LAT,     swReg->l0sEntryLatency);
  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_L1_ENTR_LAT,      swReg->l1EntryLatency);
  pcie_getbits(val, CSL_PLCONF_ACK_FREQ_ASPM_L1_ENTR_WO_L0S,   swReg->aspmL1);

  return pcie_RET_OK;
} /* pciev1_read_plconfAckFreqAspm_reg */


/*****************************************************************************
 * Combine and write the PL CONF Ack Frequency and L0-L1 ASPM register
 ****************************************************************************/
pcieRet_e pciev1_write_ackFreq_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieAckFreqReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_ACK_FREQ,         swReg->ackFreq);
  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_N_FTS,            swReg->nFts);
  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_COMMOM_CLK_N_FTS, swReg->commNFts);
  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_L0S_ENTR_LAT,     swReg->l0sEntryLatency);
  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_L1_ENTR_LAT,      swReg->l1EntryLatency);
  pcie_setbits(new_val, CSL_PLCONF_ACK_FREQ_ASPM_L1_ENTR_WO_L0S,   swReg->aspmL1);

  baseAddr->ACK_FREQ_ASPM = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfAckFreqAspm_reg */


/*****************************************************************************
 * Read and split up the PL CONF Port Link Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_lnkCtrl_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieLnkCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PT_LNK_CTRL_R;

  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_VEN_DLLP_REQ,  swReg->msgReq);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_SCRAMBLE_DIS,  swReg->scrmDis);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_LB_EN,         swReg->lpbkEn);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_RESET_ASSERT,  swReg->rstAsrt);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_DL_EN,         swReg->dllEn);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_FAST_LINK,     swReg->fLnkMode);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_LINK_MODE,     swReg->lnkMode);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_CROSSLINK_EN,  swReg->crosslinkEn);
  pcie_getbits(val, CSL_PLCONF_PT_LNK_CTRL_R_CROSSLINK_ACT, swReg->crosslinkAct);

  /* Set unused fields to 0 (only used by rev 0 hw) */
  swReg->lnkRate = 0; /* Note: same bit on hw rev 0 as RESERVED on hw rev 1 */

  return pcie_RET_OK;
} /* pciev1_read_plconfPtLnkCtrlR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Port Link Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_lnkCtrl_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieLnkCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_VEN_DLLP_REQ,  swReg->msgReq);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_SCRAMBLE_DIS,  swReg->scrmDis);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_LB_EN,         swReg->lpbkEn);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_RESET_ASSERT,  swReg->rstAsrt);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_DL_EN,         swReg->dllEn);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_FAST_LINK,     swReg->fLnkMode);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_LINK_MODE,     swReg->lnkMode);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_CROSSLINK_EN,  swReg->crosslinkEn);
  pcie_setbits(new_val, CSL_PLCONF_PT_LNK_CTRL_R_CROSSLINK_ACT, swReg->crosslinkAct);

  baseAddr->PT_LNK_CTRL_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfPtLnkCtrlR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Lane Skew register
 ****************************************************************************/
pcieRet_e pciev1_read_laneSkew_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieLaneSkewReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->LN_SKW_R;

  pcie_getbits(val, CSL_PLCONF_LN_SKW_R_LANE_SKEW,    swReg->laneSkew);
  pcie_getbits(val, CSL_PLCONF_LN_SKW_R_FC_DIS,       swReg->fcDisable);
  pcie_getbits(val, CSL_PLCONF_LN_SKW_R_ACKNAK_DIS,   swReg->ackDisable);
  pcie_getbits(val, CSL_PLCONF_LN_SKW_R_DIS_L2L_SKEW, swReg->l2Deskew);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->implementNumLanes = 0;

  return pcie_RET_OK;
} /* pciev1_read_plconfLnSkwR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Lane Skew register
 ****************************************************************************/
pcieRet_e pciev1_write_laneSkew_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieLaneSkewReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_LN_SKW_R_LANE_SKEW,    swReg->laneSkew);
  pcie_setbits(new_val, CSL_PLCONF_LN_SKW_R_FC_DIS,       swReg->fcDisable);
  pcie_setbits(new_val, CSL_PLCONF_LN_SKW_R_ACKNAK_DIS,   swReg->ackDisable);
  pcie_setbits(new_val, CSL_PLCONF_LN_SKW_R_DIS_L2L_SKEW, swReg->l2Deskew);

  baseAddr->LN_SKW_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfLnSkwR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Timer Control and Symbol Number (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_symNum_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieSymNumReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SYMB_N_R;

  pcie_getbits(val, CSL_PLCONF_SYMB_N_R_MAX_FUNC,        swReg->maxFunc);
  pcie_getbits(val, CSL_PLCONF_SYMB_N_R_REPLAY_ADJ,      swReg->replayTimer);
  pcie_getbits(val, CSL_PLCONF_SYMB_N_R_ACK_LATENCY_INC, swReg->ackLatencyTimer);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->fcWatchTimer          = 0u;
  swReg->skpCount              = 0u;
  swReg->numTs2Symbols         = 0u;
  swReg->tsCount               = 0u;
  swReg->fastLinkScalingFactor = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfSymbNR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Timer Control and Symbol Number (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_symNum_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieSymNumReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_SYMB_N_R_MAX_FUNC,        swReg->maxFunc);
  pcie_setbits(new_val, CSL_PLCONF_SYMB_N_R_REPLAY_ADJ,      swReg->replayTimer);
  pcie_setbits(new_val, CSL_PLCONF_SYMB_N_R_ACK_LATENCY_INC, swReg->ackLatencyTimer);

  baseAddr->SYMB_N_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfSymbNR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Symbol Timer and Filter Mask (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_symTimerFltMask_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->SYMB_T_R;
  uint32_t mask1;

  pcie_getbits(val, CSL_PLCONF_SYMB_T_R_SKP_INT,    swReg->skpValue);
  pcie_getbits(val, CSL_PLCONF_SYMB_T_R_DIS_FC_TIM, swReg->fcWdogDisable);

  /* Extract FLT_MSK_1 */
  pcie_getbits(val, CSL_PLCONF_SYMB_T_R_FLT_MSK_1,  mask1);

  /* Repack into named bits per rev 0, they actually match */
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CFG_DROP,        swReg->f1CfgDrop);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_IO_DROP,         swReg->f1IoDrop);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_MSG_DROP,        swReg->f1MsgDrop);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_ECRC_DROP,   swReg->f1CplEcrcDrop);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_ECRC_DROP,       swReg->f1EcrcDrop);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_LEN_TEST,    swReg->f1CplLenTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_ATTR_TEST,   swReg->f1CplAttrTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_TC_TEST,     swReg->f1CplTcTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_FUNC_TEST,   swReg->f1CplFuncTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_REQID_TEST,  swReg->f1CplReqIDTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CPL_TAGERR_TEST, swReg->f1CplTagErrTest);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_LOCKED_RD_AS_UR, swReg->f1LockedRdAsUr);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_CFG1_RE_AS_US,   swReg->f1Cfg1ReAsUs);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_UR_OUT_OF_BAR,   swReg->f1UrOutOfBar);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_UR_POISON,       swReg->f1UrPoison);
  pcie_getbits(mask1, PCIEV1_FLTMASK1_UR_FUN_MISMATCH, swReg->f1UrFunMismatch);

  return pcie_RET_OK;
} /* pciev1_read_plconfSymbTR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Symbol Timer and Filter Mask (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_symTimerFltMask_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieSymTimerFltMaskReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;
  uint32_t mask1 = 0;

  pcie_setbits(new_val, CSL_PLCONF_SYMB_T_R_SKP_INT,    swReg->skpValue);
  pcie_setbits(new_val, CSL_PLCONF_SYMB_T_R_DIS_FC_TIM, swReg->fcWdogDisable);

  /* Repack into named bits per rev 0, they actually match */
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CFG_DROP,         swReg->f1CfgDrop);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_IO_DROP,          swReg->f1IoDrop);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_MSG_DROP,         swReg->f1MsgDrop);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_ECRC_DROP,    swReg->f1CplEcrcDrop);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_ECRC_DROP,        swReg->f1EcrcDrop);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_LEN_TEST,     swReg->f1CplLenTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_ATTR_TEST,    swReg->f1CplAttrTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_TC_TEST,      swReg->f1CplTcTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_FUNC_TEST,    swReg->f1CplFuncTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_REQID_TEST,   swReg->f1CplReqIDTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CPL_TAGERR_TEST,  swReg->f1CplTagErrTest);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_LOCKED_RD_AS_UR,  swReg->f1LockedRdAsUr);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_CFG1_RE_AS_US,    swReg->f1Cfg1ReAsUs);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_UR_OUT_OF_BAR,    swReg->f1UrOutOfBar);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_UR_POISON,        swReg->f1UrPoison);
  pcie_setbits(mask1, PCIEV1_FLTMASK1_UR_FUN_MISMATCH,  swReg->f1UrFunMismatch);

  /* Put mask into register image */
  pcie_setbits(new_val, CSL_PLCONF_SYMB_T_R_FLT_MSK_1,  mask1);

  baseAddr->SYMB_T_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfSymbTR_reg */


/*****************************************************************************
 * Read and split up the PL CONF Filter Mask 2 register
 ****************************************************************************/
pcieRet_e pciev1_read_fltMask2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieFltMask2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->FL_MSK_R2;

  /* hw rev 0 bits are still valid.  Other 28 bits are reserved */
  pcie_getbits(val, PCIEV1_FLTMASK2_FLUSH_REQ,  swReg->flushReq);
  pcie_getbits(val, PCIEV1_FLTMASK2_DLLP_ABORT, swReg->dllpAbort);
  pcie_getbits(val, PCIEV1_FLTMASK2_VMSG1_DROP, swReg->vmsg1Drop);
  pcie_getbits(val, PCIEV1_FLTMASK2_VMSG0_DROP, swReg->vmsg0Drop);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->dropPRS      = 0u;
  swReg->unmaskTD     = 0u;
  swReg->unmaskUrPOIS = 0u;
  swReg->dropLN       = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfFlMskR2_reg */


/*****************************************************************************
 * Combine and write the PL CONF Filter Mask 2 register
 ****************************************************************************/
pcieRet_e pciev1_write_fltMask2_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieFltMask2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, PCIEV1_FLTMASK2_FLUSH_REQ,  swReg->flushReq);
  pcie_setbits(new_val, PCIEV1_FLTMASK2_DLLP_ABORT, swReg->dllpAbort);
  pcie_setbits(new_val, PCIEV1_FLTMASK2_VMSG1_DROP, swReg->vmsg1Drop);
  pcie_setbits(new_val, PCIEV1_FLTMASK2_VMSG0_DROP, swReg->vmsg0Drop);

  baseAddr->FL_MSK_R2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfFlMskR2_reg */


/*****************************************************************************
 * Read and split up the PL CONF AXI Multiple Outbound Decomposed NP 
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfObnpSubreqCtrl_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->OBNP_SUBREQ_CTRL;

  pcie_getbits(val, CSL_PLCONF_OBNP_SUBREQ_CTRL_EN_OBNP_SUBREQ, swReg->enObnpSubreq);

  return pcie_RET_OK;
} /* pciev1_read_plconfObnpSubreqCtrl_reg */


/*****************************************************************************
 * Combine and write the PL CONF AXI Multiple Outbound Decomposed NP 
 * SubRequests Control Register (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfObnpSubreqCtrl_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfObnpSubreqCtrlReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_OBNP_SUBREQ_CTRL_EN_OBNP_SUBREQ, swReg->enObnpSubreq);

  baseAddr->OBNP_SUBREQ_CTRL = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfObnpSubreqCtrl_reg */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Posted FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfTrPStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrPStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TR_P_STS_R;

  pcie_getbits(val, CSL_PLCONF_TR_P_STS_R_PD_CRDT, swReg->pdCrdt);
  pcie_getbits(val, CSL_PLCONF_TR_P_STS_R_PH_CRDT, swReg->phCrdt);

  return pcie_RET_OK;
} /* pciev1_read_plconfTrPStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Posted FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Non-Posted FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfTrNpStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrNpStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TR_NP_STS_R;

  pcie_getbits(val, CSL_PLCONF_TR_NP_STS_R_NPD_CRDT, swReg->npdCrdt);
  pcie_getbits(val, CSL_PLCONF_TR_NP_STS_R_NPH_CRDT, swReg->nphCrdt);

  return pcie_RET_OK;
} /* pciev1_read_plconfTrNpStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Non-Posted FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Transmit Completion FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfTrCStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfTrCStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->TR_C_STS_R;

  pcie_getbits(val, CSL_PLCONF_TR_C_STS_R_CPLD_CRDT, swReg->cpldCrdt);
  pcie_getbits(val, CSL_PLCONF_TR_C_STS_R_CPLH_CRDT, swReg->cplhCrdt);

  return pcie_RET_OK;
} /* pciev1_read_plconfTrCStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Transmit Completion FC Credit Status 
 * (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfQStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->Q_STS_R;

  pcie_getbits(val, CSL_PLCONF_Q_STS_R_CRDT_NOT_RTRN,     swReg->crdtNotRtrn);
  pcie_getbits(val, CSL_PLCONF_Q_STS_R_RTYB_NOT_EMPTY,    swReg->rtybNotEmpty);
  pcie_getbits(val, CSL_PLCONF_Q_STS_R_RCVQ_NOT_EMPTY,    swReg->rcvqNotEmpty);
  pcie_getbits(val, CSL_PLCONF_Q_STS_R_FC_LATENCY_OVR,    swReg->fcLatencyOvr);
  pcie_getbits(val, CSL_PLCONF_Q_STS_R_FC_LATENCY_OVR_EN, swReg->fcLatencyOvrEn);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->rxQueueOverflow = 0u;
  swReg->rxSerQNEmpty    = 0u;
  swReg->rxSerQWErr      = 0u;
  swReg->rxSerRErr       = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfQStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF Queue Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfQStsR_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfQStsRReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_Q_STS_R_CRDT_NOT_RTRN,     swReg->crdtNotRtrn);
  pcie_setbits(new_val, CSL_PLCONF_Q_STS_R_RTYB_NOT_EMPTY,    swReg->rtybNotEmpty);
  pcie_setbits(new_val, CSL_PLCONF_Q_STS_R_RCVQ_NOT_EMPTY,    swReg->rcvqNotEmpty);
  pcie_setbits(new_val, CSL_PLCONF_Q_STS_R_FC_LATENCY_OVR,    swReg->fcLatencyOvr);
  pcie_setbits(new_val, CSL_PLCONF_Q_STS_R_FC_LATENCY_OVR_EN, swReg->fcLatencyOvrEn);

  baseAddr->Q_STS_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfQStsR_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 1 (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfVcTrAR1_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVcTrAR1Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC_TR_A_R1;

  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R1_WRR_VC0, swReg->wrrVc0);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R1_WRR_VC1, swReg->wrrVc1);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R1_WRR_VC2, swReg->wrrVc2);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R1_WRR_VC3, swReg->wrrVc3);

  return pcie_RET_OK;
} /* pciev1_read_plconfVcTrAR1_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC Transmit Arbitration 1 (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF VC Transmit Arbitration 2 (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfVcTrAR2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVcTrAR2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC_TR_A_R2;

  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R2_WRR_VC4, swReg->wrrVc4);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R2_WRR_VC5, swReg->wrrVc5);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R2_WRR_VC6, swReg->wrrVc6);
  pcie_getbits(val, CSL_PLCONF_VC_TR_A_R2_WRR_VC7, swReg->wrrVc7);

  return pcie_RET_OK;
} /* pciev1_read_plconfVcTrAR2_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC Transmit Arbitration 2 (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF VC0 Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfVc0PrQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0PrQCReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC0_PR_Q_C;

  pcie_getbits(val, CSL_PLCONF_VC0_PR_Q_C_P_DCRD,             swReg->pDcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_PR_Q_C_P_HCRD,             swReg->pHcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_PR_Q_C_ORDERING_RULES,     swReg->orderingRules);
  pcie_getbits(val, CSL_PLCONF_VC0_PR_Q_C_STRICT_VC_PRIORITY, swReg->strictVcPriority);
  pcie_getbits(val, CSL_PLCONF_VC0_PR_Q_C_P_QMODE,            swReg->pQmode);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->pDataScale = 0u;
  swReg->pHdrScale  = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfVc0PrQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Posted Receive Queue Control (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfVc0PrQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0PrQCReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_VC0_PR_Q_C_P_DCRD,             swReg->pDcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_PR_Q_C_P_HCRD,             swReg->pHcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_PR_Q_C_ORDERING_RULES,     swReg->orderingRules);
  pcie_setbits(new_val, CSL_PLCONF_VC0_PR_Q_C_STRICT_VC_PRIORITY, swReg->strictVcPriority);
  pcie_setbits(new_val, CSL_PLCONF_VC0_PR_Q_C_P_QMODE,            swReg->pQmode);

  baseAddr->VC0_PR_Q_C = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfVc0PrQC_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfVc0NprQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0NprQCReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC0_NPR_Q_C;

  pcie_getbits(val, CSL_PLCONF_VC0_NPR_Q_C_NP_DCRD,  swReg->npDcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_NPR_Q_C_NP_HCRD,  swReg->npHcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_NPR_Q_C_NP_QMODE, swReg->npQmode);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->npDataScale = 0u;
  swReg->npHdrScale  = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfVc0NprQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Non-Posted Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfVc0NprQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0NprQCReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_VC0_NPR_Q_C_NP_DCRD,  swReg->npDcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_NPR_Q_C_NP_HCRD,  swReg->npHcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_NPR_Q_C_NP_QMODE, swReg->npQmode);

  baseAddr->VC0_NPR_Q_C = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfVc0NprQC_reg */


/*****************************************************************************
 * Read and split up the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfVc0CrQC_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfVc0CrQCReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->VC0_CR_Q_C;

  pcie_getbits(val, CSL_PLCONF_VC0_CR_Q_C_CPL_DCRD,  swReg->cplDcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_CR_Q_C_CPL_HCRD,  swReg->cplHcrd);
  pcie_getbits(val, CSL_PLCONF_VC0_CR_Q_C_CPL_QMODE, swReg->cplQmode);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->cplDataScale = 0u;
  swReg->cplHdrScale  = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfVc0CrQC_reg */


/*****************************************************************************
 * Combine and write the PL CONF VC0 Completion Receive Queue Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfVc0CrQC_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfVc0CrQCReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_VC0_CR_Q_C_CPL_DCRD,  swReg->cplDcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_CR_Q_C_CPL_HCRD,  swReg->cplHcrd);
  pcie_setbits(new_val, CSL_PLCONF_VC0_CR_Q_C_CPL_QMODE, swReg->cplQmode);

  baseAddr->VC0_CR_Q_C = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfVc0CrQC_reg */


/*****************************************************************************
 * Read and split up the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_gen2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pcieGen2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->WIDTH_SPEED_CTL;

  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_GEN2_N_FTS,            swReg->numFts);
  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_LANE_EN,               swReg->lnEn);
  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_DIRECTED_SPEED_CHANGE, swReg->dirSpd);
  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_PHY_TXSWING,           swReg->txSwing);
  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_TX_COMPLIANCE_RCV,     swReg->txCmpl);
  pcie_getbits(val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_UP_SEL_DEEMPH,         swReg->deemph);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->preDetLane      = 0u;
  swReg->autoFlipEn      = 0u;
  swReg->gen1EiInference = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfWidthSpeedCtl_reg */


/*****************************************************************************
 * Combine and write the PL CONF Link Width and Speed Change Control (Sticky)
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_gen2_reg
(
  CSL_PlConfRegs *baseAddr,
  pcieGen2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_GEN2_N_FTS,            swReg->numFts);
  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_LANE_EN,               swReg->lnEn);
  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_DIRECTED_SPEED_CHANGE, swReg->dirSpd);
  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_PHY_TXSWING,           swReg->txSwing);
  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_TX_COMPLIANCE_RCV,     swReg->txCmpl);
  pcie_setbits(new_val, CSL_PLCONF_WIDTH_SPEED_CTL_CFG_UP_SEL_DEEMPH,         swReg->deemph);

  baseAddr->WIDTH_SPEED_CTL = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfWidthSpeedCtl_reg */


/*****************************************************************************
 * Read and split up the PL CONF PHY Status (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfPhyStsR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPhyStsRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PHY_STS_R;

  pcie_getbits(val, CSL_PLCONF_PHY_STS_R_PHY_STS, swReg->phySts);

  return pcie_RET_OK;
} /* pciev1_read_plconfPhyStsR_reg */


/*****************************************************************************
 * Combine and write the PL CONF PHY Status (Sticky) register
 ****************************************************************************/
/* Not applicable - read-only register */


/*****************************************************************************
 * Read and split up the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfPhyCtrlR_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->PHY_CTRL_R;

  pcie_getbits(val, CSL_PLCONF_PHY_CTRL_R_PHY_CTRL, swReg->phyCtrl);

  return pcie_RET_OK;
} /* pciev1_read_plconfPhyCtrlR_reg */


/*****************************************************************************
 * Combine and write the PL CONF PHY Control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfPhyCtrlR_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfPhyCtrlRReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_PHY_CTRL_R_PHY_CTRL, swReg->phyCtrl);

  baseAddr->PHY_CTRL_R = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfPhyCtrlR_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlAddress_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_ADDRESS.MSI_CTRL_ADDRESS;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_ADDRESS_MSI_CTRL_ADDRESS, swReg->msiCtrlAddress);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlAddress_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Address (RC-mode MSI receiver)
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlAddress_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlAddressReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_ADDRESS_MSI_CTRL_ADDRESS, swReg->msiCtrlAddress);

  baseAddr->MSI_ADDRESS.MSI_CTRL_ADDRESS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlAddress_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Upper Address 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlUpperAddress_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_ADDRESS.MSI_CTRL_UPPER_ADDRESS;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_UPPER_ADDRESS_MSI_CTRL_UPPER_ADDRESS, swReg->msiCtrlUpperAddress);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlUpperAddress_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Upper Address 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlUpperAddress_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlUpperAddressReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_UPPER_ADDRESS_MSI_CTRL_UPPER_ADDRESS, swReg->msiCtrlUpperAddress);

  baseAddr->MSI_ADDRESS.MSI_CTRL_UPPER_ADDRESS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlUpperAddress_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Enable 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlIntEnable_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_ENABLE;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_INT_ENABLE_MSI_CTRL_INT_ENABLE, swReg->msiCtrlIntEnable);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlIntEnable_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Enable 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlIntEnable_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntEnableReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_INT_ENABLE_MSI_CTRL_INT_ENABLE, swReg->msiCtrlIntEnable);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_ENABLE = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlIntEnable_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Mask 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlIntMask_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_INT_MASK_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlIntMask_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Mask 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlIntMask_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntMaskReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_INT_MASK_MSI_CTRL_INT_MASK, swReg->msiCtrlIntMask);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_MASK = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlIntMask_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller Interrupt #N(1) Status 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlIntStatus_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
)
{
  uint32_t val = swReg->raw = baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_INT_STATUS_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlIntStatus_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller Interrupt #N(1) Status 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlIntStatus_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlIntStatusReg_t *swReg,
  int32_t n
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_INT_STATUS_MSI_CTRL_INT_STATUS, swReg->msiCtrlIntStatus);

  baseAddr->MSI_CTRL[n].MSI_CTRL_INT_STATUS = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlIntStatus_reg */


/*****************************************************************************
 * Read and split up the PL CONF MSI Controller General Purpose IO 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfMsiCtrlGpio_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CONFIG.MSI_CTRL_GPIO;

  pcie_getbits(val, CSL_PLCONF_MSI_CTRL_GPIO_MSI_CTRL_GPIO, swReg->msiCtrlGpio);

  return pcie_RET_OK;
} /* pciev1_read_plconfMsiCtrlGpio_reg */


/*****************************************************************************
 * Combine and write the PL CONF MSI Controller General Purpose IO 
 * (RC-mode MSI receiver) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfMsiCtrlGpio_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfMsiCtrlGpioReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_MSI_CTRL_GPIO_MSI_CTRL_GPIO, swReg->msiCtrlGpio);

  baseAddr->CONFIG.MSI_CTRL_GPIO = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfMsiCtrlGpio_reg */


/*****************************************************************************
 * Read and split up the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfPipeLoopback_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CONFIG.PIPE_LOOPBACK;

  pcie_getbits(val, CSL_PLCONF_PIPE_LOOPBACK_LOOPBACK_EN, swReg->loopbackEn);

  return pcie_RET_OK;
} /* pciev1_read_plconfPipeLoopback_reg */


/*****************************************************************************
 * Combine and write the PL CONF PIPE loopback control (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfPipeLoopback_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfPipeLoopbackReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_PIPE_LOOPBACK_LOOPBACK_EN, swReg->loopbackEn);

  baseAddr->CONFIG.PIPE_LOOPBACK = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfPipeLoopback_reg */


/*****************************************************************************
 * Read and split up the PL CONF DIF Read-Only register Write Enable (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfDbiRoWrEn_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CONFIG.DBI_RO_WR_EN;

  pcie_getbits(val, CSL_PLCONF_DBI_RO_WR_EN_CX_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->defaultTarget   = 0u;
  swReg->urCaMask4Trgt1  = 0u;
  swReg->simpReplayTimer = 0u;
  swReg->ariDevNumber    = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfDbiRoWrEn_reg */


/*****************************************************************************
 * Combine and write the PL CONF DIF Read-Only register Write Enable (Sticky) 
 * register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfDbiRoWrEn_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfDbiRoWrEnReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_DBI_RO_WR_EN_CX_DBI_RO_WR_EN, swReg->cxDbiRoWrEn);

  baseAddr->CONFIG.DBI_RO_WR_EN = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfDbiRoWrEn_reg */


/*****************************************************************************
 * Read and split up the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfAxiSlvErrResp_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CONFIG.AXI_SLV_ERR_RESP;

  pcie_getbits(val, CSL_PLCONF_AXI_SLV_ERR_RESP_SLAVE_ERR_MAP,         swReg->slaveErrMap);
  pcie_getbits(val, CSL_PLCONF_AXI_SLV_ERR_RESP_DBI_ERR_MAP,           swReg->dbiErrMap);
  pcie_getbits(val, CSL_PLCONF_AXI_SLV_ERR_RESP_NO_VID_ERR_MAP,        swReg->noVidErrMap);
  pcie_getbits(val, CSL_PLCONF_AXI_SLV_ERR_RESP_RESET_TIMEOUT_ERR_MAP, swReg->resetTimeoutErrMap);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->errorResponseCrs   = 0u;
  swReg->errorResponseMap   = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfAxiSlvErrResp_reg */


/*****************************************************************************
 * Combine and write the PL CONF AXI Slave Error Response (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfAxiSlvErrResp_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvErrRespReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_ERR_RESP_SLAVE_ERR_MAP,         swReg->slaveErrMap);
  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_ERR_RESP_DBI_ERR_MAP,           swReg->dbiErrMap);
  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_ERR_RESP_NO_VID_ERR_MAP,        swReg->noVidErrMap);
  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_ERR_RESP_RESET_TIMEOUT_ERR_MAP, swReg->resetTimeoutErrMap);

  baseAddr->CONFIG.AXI_SLV_ERR_RESP = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfAxiSlvErrResp_reg */


/*****************************************************************************
 * Read and split up the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfAxiSlvTimeout_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->CONFIG.AXI_SLV_TIMEOUT;

  pcie_getbits(val, CSL_PLCONF_AXI_SLV_TIMEOUT_TIMEOUT_VALUE, swReg->timeoutValue);
  pcie_getbits(val, CSL_PLCONF_AXI_SLV_TIMEOUT_FLUSH_EN,      swReg->flushEn);

  return pcie_RET_OK;
} /* pciev1_read_plconfAxiSlvTimeout_reg */


/*****************************************************************************
 * Combine and write the PL CONF Link Down AXI Slave Timeout (Sticky) register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfAxiSlvTimeout_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfAxiSlvTimeoutReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_TIMEOUT_TIMEOUT_VALUE, swReg->timeoutValue);
  pcie_setbits(new_val, CSL_PLCONF_AXI_SLV_TIMEOUT_FLUSH_EN,      swReg->flushEn);

  baseAddr->CONFIG.AXI_SLV_TIMEOUT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfAxiSlvTimeout_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Viewport register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuIndex_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuIndexReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_INDEX;

  pcie_getbits(val, CSL_PLCONF_IATU_INDEX_REGION_DIRECTION, swReg->regionDirection);
  pcie_getbits(val, CSL_PLCONF_IATU_INDEX_REGION_INDEX,     swReg->regionIndex);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuIndex_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Viewport register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuIndex_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuIndexReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_INDEX_REGION_DIRECTION, swReg->regionDirection);
  pcie_setbits(new_val, CSL_PLCONF_IATU_INDEX_REGION_INDEX,     swReg->regionIndex);

  baseAddr->IATU.IATU_INDEX = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuIndex_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 1 register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegCtrl1_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl1Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_CTRL_1;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_TYPE,            swReg->type);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_TC,              swReg->tc);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_TD,              swReg->td);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_ATTR,            swReg->attr);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_AT,              swReg->at);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_1_FUNCTION_NUMBER, swReg->functionNumber);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->increaseRegionSize = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegCtrl1_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 1 register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegCtrl1_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl1Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_TYPE,            swReg->type);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_TC,              swReg->tc);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_TD,              swReg->td);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_ATTR,            swReg->attr);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_AT,              swReg->at);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_1_FUNCTION_NUMBER, swReg->functionNumber);

  baseAddr->IATU.IATU_REG_CTRL_1 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegCtrl1_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 2 register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegCtrl2_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl2Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_CTRL_2;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_MESSAGECODE,                          swReg->messagecode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_BAR_NUMBER,                           swReg->barNumber);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_TC_MATCH_ENABLE,                      swReg->tcMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_TD_MATCH_ENABLE,                      swReg->tdMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_ATTR_MATCH_ENABLE,                    swReg->attrMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_AT_MATCH_ENABLE,                      swReg->atMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_FUNCTION_NUMBER_MATCH_ENABLE,         swReg->functionNumberMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_VIRTUAL_FUNCTION_NUMBER_MATCH_ENABLE, swReg->virtualFunctionNumberMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_MESSAGE_CODE_MATCH_ENABLE,            swReg->messageCodeMatchEnable);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_RESPONSE_CODE,                        swReg->responseCode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_FUZZY_TYPE_MATCH_MODE,                swReg->fuzzyTypeMatchMode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_CFG_SHIFT_MODE,                       swReg->cfgShiftMode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_INVERT_MODE,                          swReg->invertMode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE,                           swReg->matchMode);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_2_REGION_ENABLE,                        swReg->regionEnable);

  /* Set unused fields to 0 (only used by rev 0/2 hw) */
  swReg->tag                  = 0u;
  swReg->tagSubstEn           = 0u;
  swReg->SNP                  = 0u;
  swReg->inhibitPayload       = 0u;
  swReg->headerSubstEn        = 0u;
  swReg->msgTypeMatchMode     = 0u;
  swReg->singleAddrLocTransEn = 0u;

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegCtrl2_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 2 register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegCtrl2_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl2Reg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_MESSAGECODE,                          swReg->messagecode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_BAR_NUMBER,                           swReg->barNumber);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_TC_MATCH_ENABLE,                      swReg->tcMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_TD_MATCH_ENABLE,                      swReg->tdMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_ATTR_MATCH_ENABLE,                    swReg->attrMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_AT_MATCH_ENABLE,                      swReg->atMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_FUNCTION_NUMBER_MATCH_ENABLE,         swReg->functionNumberMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_VIRTUAL_FUNCTION_NUMBER_MATCH_ENABLE, swReg->virtualFunctionNumberMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_MESSAGE_CODE_MATCH_ENABLE,            swReg->messageCodeMatchEnable);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_RESPONSE_CODE,                        swReg->responseCode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_FUZZY_TYPE_MATCH_MODE,                swReg->fuzzyTypeMatchMode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_CFG_SHIFT_MODE,                       swReg->cfgShiftMode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_INVERT_MODE,                          swReg->invertMode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_MATCH_MODE,                           swReg->matchMode);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_CTRL_2_REGION_ENABLE,                        swReg->regionEnable);

  baseAddr->IATU.IATU_REG_CTRL_2 = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegCtrl2_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegLowerBase_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerBaseReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_LOWER_BASE;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_LOWER_BASE_IATU_REG_LOWER_BASE, swReg->iatuRegLowerBase);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_LOWER_BASE_ZERO,                swReg->zero);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegLowerBase_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Base Address register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegLowerBase_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerBaseReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LOWER_BASE_IATU_REG_LOWER_BASE, swReg->iatuRegLowerBase);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LOWER_BASE_ZERO,                swReg->zero);

  baseAddr->IATU.IATU_REG_LOWER_BASE = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegLowerBase_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegUpperBase_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperBaseReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_UPPER_BASE;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_UPPER_BASE_IATU_REG_UPPER_BASE, swReg->iatuRegUpperBase);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegUpperBase_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegUpperBase_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperBaseReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_UPPER_BASE_IATU_REG_UPPER_BASE, swReg->iatuRegUpperBase);

  baseAddr->IATU.IATU_REG_UPPER_BASE = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegUpperBase_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegLimit_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLimitReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_LIMIT;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_LIMIT_IATU_REG_LIMIT, swReg->iatuRegLimit);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_LIMIT_ONES,           swReg->ones);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegLimit_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Limit Address register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegLimit_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLimitReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LIMIT_IATU_REG_LIMIT, swReg->iatuRegLimit);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LIMIT_ONES,           swReg->ones);

  baseAddr->IATU.IATU_REG_LIMIT = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegLimit_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegLowerTarget_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerTargetReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_LOWER_TARGET;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_LOWER_TARGET_IATU_REG_LOWER_TARGET, swReg->iatuRegLowerTarget);
  pcie_getbits(val, CSL_PLCONF_IATU_REG_LOWER_TARGET_ZERO,                  swReg->zero);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegLowerTarget_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Lower Target Address register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegLowerTarget_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegLowerTargetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LOWER_TARGET_IATU_REG_LOWER_TARGET, swReg->iatuRegLowerTarget);
  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_LOWER_TARGET_ZERO,                  swReg->zero);

  baseAddr->IATU.IATU_REG_LOWER_TARGET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegLowerTarget_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegUpperTarget_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperTargetReg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_UPPER_TARGET;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_UPPER_TARGET_IATU_REG_UPPER_TARGET, swReg->iatuRegUpperTarget);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegUpperTarget_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Upper Target Address register
 ****************************************************************************/
pcieRet_e pciev1_write_plconfIatuRegUpperTarget_reg
(
  CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegUpperTargetReg_t *swReg
)
{
  uint32_t new_val = swReg->raw;
  pcie_range_check_begin;

  pcie_setbits(new_val, CSL_PLCONF_IATU_REG_UPPER_TARGET_IATU_REG_UPPER_TARGET, swReg->iatuRegUpperTarget);

  baseAddr->IATU.IATU_REG_UPPER_TARGET = swReg->raw = new_val;

  return pcie_range_check_return;
} /* pciev1_write_plconfIatuRegUpperTarget_reg */


/*****************************************************************************
 * Read and split up the PL CONF iATU Region Control 3 Register register
 ****************************************************************************/
pcieRet_e pciev1_read_plconfIatuRegCtrl3_reg
(
  const CSL_PlConfRegs *baseAddr,
  pciePlconfIatuRegCtrl3Reg_t *swReg
)
{
  uint32_t val = swReg->raw = baseAddr->IATU.IATU_REG_CTRL_3;

  pcie_getbits(val, CSL_PLCONF_IATU_REG_CTRL_3_IATU_REG_CTRL_3, swReg->iatuRegCtrl3);

  return pcie_RET_OK;
} /* pciev1_read_plconfIatuRegCtrl3_reg */


/*****************************************************************************
 * Combine and write the PL CONF iATU Region Control 3 Register register
 ****************************************************************************/
/* Not applicable - read-only register */

/* Nothing past this point */

