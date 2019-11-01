/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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
#include <ti/drv/iqn2/iqn2fl.h>
#include <stdio.h>

/* Prototypes for enum dumping functions.  */
char *dump_Iqn2Fl_AilInstance (Iqn2Fl_AilInstance value);
char *dump_Iqn2Fl_ChanEnetCtlMode (Iqn2Fl_ChanEnetCtlMode value);
char *dump_Iqn2Fl_ChanFrcOff (Iqn2Fl_ChanFrcOff value);
char *dump_Iqn2Fl_ChanObsaiCtl (Iqn2Fl_ChanObsaiCtl value);
char *dump_Iqn2Fl_ChanRadioSel (Iqn2Fl_ChanRadioSel value);
char *dump_Iqn2Fl_DataWidth (Iqn2Fl_DataWidth value);
char *dump_Iqn2Fl_HwControlCmd (Iqn2Fl_HwControlCmd value);
char *dump_Iqn2Fl_HwStatusQuery (Iqn2Fl_HwStatusQuery value);
char *dump_Iqn2Fl_LinkRate (Iqn2Fl_LinkRate value);
char *dump_Iqn2Fl_Status (Iqn2Fl_Status value);
/* Prototypes for struct dumping functions.  */
void dump_Iqn2Fl_Aid2IqEfeCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqEfeCfgGrpSetup *value);
void dump_Iqn2Fl_Aid2IqEfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Aid2IqEfeRadioStdGrpSetup *value);
void dump_Iqn2Fl_Aid2IqIfeChanCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqIfeChanCfgGrpSetup *value);
void dump_Iqn2Fl_Aid2IqIfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Aid2IqIfeRadioStdGrpSetup *value);
void dump_Iqn2Fl_Aid2IqIngChanCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqIngChanCfgGrpSetup *value);
void dump_Iqn2Fl_Aid2IqUatGenCtlSetup (FILE *output, Iqn2Fl_Aid2IqUatGenCtlSetup *value);
void dump_Iqn2Fl_Aid2Setup (FILE *output, Iqn2Fl_Aid2Setup *value);
void dump_Iqn2Fl_AilEctlPktIf (FILE *output, Iqn2Fl_AilEctlPktIf *value);
void dump_Iqn2Fl_AilEctlRegGrp (FILE *output, Iqn2Fl_AilEctlRegGrp *value);
void dump_Iqn2Fl_AilEdcRegGrp (FILE *output, Iqn2Fl_AilEdcRegGrp *value);
void dump_Iqn2Fl_AilEgrSetup (FILE *output, Iqn2Fl_AilEgrSetup *value);
void dump_Iqn2Fl_AilIctlIdcIf (FILE *output, Iqn2Fl_AilIctlIdcIf *value);
void dump_Iqn2Fl_AilIgrSetup (FILE *output, Iqn2Fl_AilIgrSetup *value);
void dump_Iqn2Fl_AilIqEfeChanCfg (FILE *output, Iqn2Fl_AilIqEfeChanCfg *value);
void dump_Iqn2Fl_AilIqEfeRadStdCfg (FILE *output, Iqn2Fl_AilIqEfeRadStdCfg *value);
void dump_Iqn2Fl_AilIqFrmTcCfg (FILE *output, Iqn2Fl_AilIqFrmTcCfg *value);
void dump_Iqn2Fl_AilIqIfeChanCfgGrp (FILE *output, Iqn2Fl_AilIqIfeChanCfgGrp *value);
void dump_Iqn2Fl_AilIqIfeRadStdCfg (FILE *output, Iqn2Fl_AilIqIfeRadStdCfg *value);
void dump_Iqn2Fl_AilIqIfeRadStdGrp (FILE *output, Iqn2Fl_AilIqIfeRadStdGrp *value);
void dump_Iqn2Fl_AilIqIgrCfgGrp (FILE *output, Iqn2Fl_AilIqIgrCfgGrp *value);
void dump_Iqn2Fl_AilIqIgrChCfgGrp (FILE *output, Iqn2Fl_AilIqIgrChCfgGrp *value);
void dump_Iqn2Fl_AilIqPeObsaiModtxruleCfg (FILE *output, Iqn2Fl_AilIqPeObsaiModtxruleCfg *value);
void dump_Iqn2Fl_AilPdCommon (FILE *output, Iqn2Fl_AilPdCommon *value);
void dump_Iqn2Fl_AilPdCpriAxc0Cfg (FILE *output, Iqn2Fl_AilPdCpriAxc0Cfg *value);
void dump_Iqn2Fl_AilPdCpriAxcCfg (FILE *output, Iqn2Fl_AilPdCpriAxcCfg *value);
void dump_Iqn2Fl_AilPdCpriCwCfg (FILE *output, Iqn2Fl_AilPdCpriCwCfg *value);
void dump_Iqn2Fl_AilPdCpriCwChanCfg (FILE *output, Iqn2Fl_AilPdCpriCwChanCfg *value);
void dump_Iqn2Fl_AilPdCpriTdmFsmCfg (FILE *output, Iqn2Fl_AilPdCpriTdmFsmCfg *value);
void dump_Iqn2Fl_AilPdObsaiCfg (FILE *output, Iqn2Fl_AilPdObsaiCfg *value);
void dump_Iqn2Fl_AilPdObsaiChanCfg (FILE *output, Iqn2Fl_AilPdObsaiChanCfg *value);
void dump_Iqn2Fl_AilPdObsaiLutCfg (FILE *output, Iqn2Fl_AilPdObsaiLutCfg *value);
void dump_Iqn2Fl_AilPdObsaiRouteCfg (FILE *output, Iqn2Fl_AilPdObsaiRouteCfg *value);
void dump_Iqn2Fl_AilPdObsaiTypeLutCfg (FILE *output, Iqn2Fl_AilPdObsaiTypeLutCfg *value);
void dump_Iqn2Fl_AilPdSetup (FILE *output, Iqn2Fl_AilPdSetup *value);
void dump_Iqn2Fl_AilPeCommon (FILE *output, Iqn2Fl_AilPeCommon *value);
void dump_Iqn2Fl_AilPeCommonChanCfg (FILE *output, Iqn2Fl_AilPeCommonChanCfg *value);
void dump_Iqn2Fl_AilPeCpriCw (FILE *output, Iqn2Fl_AilPeCpriCw *value);
void dump_Iqn2Fl_AilPeCpriCwChanCfg (FILE *output, Iqn2Fl_AilPeCpriCwChanCfg *value);
void dump_Iqn2Fl_AilPeCpriCwFastEth4b5b (FILE *output, Iqn2Fl_AilPeCpriCwFastEth4b5b *value);
void dump_Iqn2Fl_AilPeCpriCwLut (FILE *output, Iqn2Fl_AilPeCpriCwLut *value);
void dump_Iqn2Fl_AilPeObsaiHdrLut (FILE *output, Iqn2Fl_AilPeObsaiHdrLut *value);
void dump_Iqn2Fl_AilPeSetup (FILE *output, Iqn2Fl_AilPeSetup *value);
void dump_Iqn2Fl_AilPhyCiCoLutCfg (FILE *output, Iqn2Fl_AilPhyCiCoLutCfg *value);
void dump_Iqn2Fl_AilPhyGlbCfg (FILE *output, Iqn2Fl_AilPhyGlbCfg *value);
void dump_Iqn2Fl_AilPhyLutParams (FILE *output, Iqn2Fl_AilPhyLutParams *value);
void dump_Iqn2Fl_AilPhyLutSetup (FILE *output, Iqn2Fl_AilPhyLutSetup *value);
void dump_Iqn2Fl_AilPhyRmCfg (FILE *output, Iqn2Fl_AilPhyRmCfg *value);
void dump_Iqn2Fl_AilPhyRmClkDetCfg (FILE *output, Iqn2Fl_AilPhyRmClkDetCfg *value);
void dump_Iqn2Fl_AilPhyRmDpCfg (FILE *output, Iqn2Fl_AilPhyRmDpCfg *value);
void dump_Iqn2Fl_AilPhyRtCfg (FILE *output, Iqn2Fl_AilPhyRtCfg *value);
void dump_Iqn2Fl_AilPhySetup (FILE *output, Iqn2Fl_AilPhySetup *value);
void dump_Iqn2Fl_AilPhyTmCfg (FILE *output, Iqn2Fl_AilPhyTmCfg *value);
void dump_Iqn2Fl_AilPhyTmL1InbCfg (FILE *output, Iqn2Fl_AilPhyTmL1InbCfg *value);
void dump_Iqn2Fl_AilPhyTmL1InbEnCfg (FILE *output, Iqn2Fl_AilPhyTmL1InbEnCfg *value);
void dump_Iqn2Fl_AilSetup (FILE *output, Iqn2Fl_AilSetup *value);
void dump_Iqn2Fl_AilSiIqCpriRadstdCfg (FILE *output, Iqn2Fl_AilSiIqCpriRadstdCfg *value);
void dump_Iqn2Fl_AilSiIqESchCpriBubFsmCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriBubFsmCfg *value);
void dump_Iqn2Fl_AilSiIqESchCpriContCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriContCfg *value);
void dump_Iqn2Fl_AilSiIqESchCpriTdmFsmCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriTdmFsmCfg *value);
void dump_Iqn2Fl_AilSiIqEfeCfgGrp (FILE *output, Iqn2Fl_AilSiIqEfeCfgGrp *value);
void dump_Iqn2Fl_AilSiIqEfeRadStdGrp (FILE *output, Iqn2Fl_AilSiIqEfeRadStdGrp *value);
void dump_Iqn2Fl_AilSiIqEgrObsaiDbmRule (FILE *output, Iqn2Fl_AilSiIqEgrObsaiDbmRule *value);
void dump_Iqn2Fl_AilSiIqEgrSchCpri (FILE *output, Iqn2Fl_AilSiIqEgrSchCpri *value);
void dump_Iqn2Fl_AilSiIqEgrTdmLutRam (FILE *output, Iqn2Fl_AilSiIqEgrTdmLutRam *value);
void dump_Iqn2Fl_AilUatSetup (FILE *output, Iqn2Fl_AilUatSetup *value);
void dump_Iqn2Fl_At2Events24Array (FILE *output, Iqn2Fl_At2Events24Array *value);
void dump_Iqn2Fl_At2RadtStatus (FILE *output, Iqn2Fl_At2RadtStatus *value);
void dump_Iqn2Fl_At2Setup (FILE *output, Iqn2Fl_At2Setup *value);
void dump_Iqn2Fl_BaseAddress (FILE *output, Iqn2Fl_BaseAddress *value);
void dump_Iqn2Fl_Dio2CoreDmaCfg0 (FILE *output, Iqn2Fl_Dio2CoreDmaCfg0 *value);
void dump_Iqn2Fl_Dio2CoreEgressSetup (FILE *output, Iqn2Fl_Dio2CoreEgressSetup *value);
void dump_Iqn2Fl_Dio2CoreIngressSetup (FILE *output, Iqn2Fl_Dio2CoreIngressSetup *value);
void dump_Iqn2Fl_Dio2DbcntxRamMmr (FILE *output, Iqn2Fl_Dio2DbcntxRamMmr *value);
void dump_Iqn2Fl_Dio2DbcntxRamMmrCfg (FILE *output, Iqn2Fl_Dio2DbcntxRamMmrCfg *value);
void dump_Iqn2Fl_Dio2Dt (FILE *output, Iqn2Fl_Dio2Dt *value);
void dump_Iqn2Fl_Dio2FrmTcCfg (FILE *output, Iqn2Fl_Dio2FrmTcCfg *value);
void dump_Iqn2Fl_Dio2IqIdcChCfgGrp (FILE *output, Iqn2Fl_Dio2IqIdcChCfgGrp *value);
void dump_Iqn2Fl_Dio2IqIfeChanCfgGrpSetup (FILE *output, Iqn2Fl_Dio2IqIfeChanCfgGrpSetup *value);
void dump_Iqn2Fl_Dio2IqIfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Dio2IqIfeRadioStdGrpSetup *value);
void dump_Iqn2Fl_Dio2Setup (FILE *output, Iqn2Fl_Dio2Setup *value);
void dump_Iqn2Fl_Dio2SiIqEfeCfgGrpSetup (FILE *output, Iqn2Fl_Dio2SiIqEfeCfgGrpSetup *value);
void dump_Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup *value);
void dump_Iqn2Fl_HwControlMultiArgs (FILE *output, Iqn2Fl_HwControlMultiArgs *value);
void dump_Iqn2Fl_Iqs2Setup (FILE *output, Iqn2Fl_Iqs2Setup *value);
void dump_Iqn2Fl_IqsEgrPktdmaCfgSetup (FILE *output, Iqn2Fl_IqsEgrPktdmaCfgSetup *value);
void dump_Iqn2Fl_IqsEgressChanCfgSetup (FILE *output, Iqn2Fl_IqsEgressChanCfgSetup *value);
void dump_Iqn2Fl_IqsIngressCfgSetup (FILE *output, Iqn2Fl_IqsIngressCfgSetup *value);
void dump_Iqn2Fl_IqsIngressChanCfgSetup (FILE *output, Iqn2Fl_IqsIngressChanCfgSetup *value);
void dump_Iqn2Fl_IqsIngressChanLutCfg (FILE *output, Iqn2Fl_IqsIngressChanLutCfg *value);
void dump_Iqn2Fl_Obj (FILE *output, Iqn2Fl_Obj *value);
void dump_Iqn2Fl_Param (FILE *output, Iqn2Fl_Param *value);
void dump_Iqn2Fl_RadtOffsetCfg (FILE *output, Iqn2Fl_RadtOffsetCfg *value);
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
void dump_Iqn2Fl_TopPsrConfigSetup (FILE *output, Iqn2Fl_TopPsrConfigSetup *value);
void dump_Iqn2Fl_TopSetup (FILE *output, Iqn2Fl_TopSetup *value);
void dump_Iqn2Fl_TopVCSwResetStbSetup (FILE *output, Iqn2Fl_TopVCSwResetStbSetup *value);
void dump_Iqn2Fl_TopVcSysStsSetup (FILE *output, Iqn2Fl_TopVcSysStsSetup *value);
void dump_Iqn2Fl_UatCfg (FILE *output, Iqn2Fl_UatCfg *value);
void dump_Iqn2Fl_UatRadtEvtSetup (FILE *output, Iqn2Fl_UatRadtEvtSetup *value);

/* Enum dumping functions.  */
char *dump_Iqn2Fl_AilInstance (Iqn2Fl_AilInstance value) {
  switch(value) {
  case IQN2FL_AIL_0:
    return "IQN2FL_AIL_0";
  case IQN2FL_AIL_1:
    return "IQN2FL_AIL_1";
  case IQN2FL_AIL_MAX:
    return "IQN2FL_AIL_MAX";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_ChanEnetCtlMode (Iqn2Fl_ChanEnetCtlMode value) {
  switch(value) {
  case IQN2FL_CHAN_ENET_CTL_ENET:
    return "IQN2FL_CHAN_ENET_CTL_ENET";
  case IQN2FL_CHAN_ENET_CTL_NON_ENET:
    return "IQN2FL_CHAN_ENET_CTL_NON_ENET";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_ChanFrcOff (Iqn2Fl_ChanFrcOff value) {
  switch(value) {
  case IQN2FL_CHAN_FRC_SYM_OFF:
    return "IQN2FL_CHAN_FRC_SYM_OFF";
  case IQN2FL_CHAN_NO_FRC_OFF_SYM:
    return "IQN2FL_CHAN_NO_FRC_OFF_SYM";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_ChanObsaiCtl (Iqn2Fl_ChanObsaiCtl value) {
  switch(value) {
  case IQN2FL_CHAN_OBSAI_AXC:
    return "IQN2FL_CHAN_OBSAI_AXC";
  case IQN2FL_CHAN_OBSAI_CTL:
    return "IQN2FL_CHAN_OBSAI_CTL";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_ChanRadioSel (Iqn2Fl_ChanRadioSel value) {
  switch(value) {
  case IQN2FL_CHAN_RADIO_SEL_STD_0:
    return "IQN2FL_CHAN_RADIO_SEL_STD_0";
  case IQN2FL_CHAN_RADIO_SEL_STD_1:
    return "IQN2FL_CHAN_RADIO_SEL_STD_1";
  case IQN2FL_CHAN_RADIO_SEL_STD_2:
    return "IQN2FL_CHAN_RADIO_SEL_STD_2";
  case IQN2FL_CHAN_RADIO_SEL_STD_3:
    return "IQN2FL_CHAN_RADIO_SEL_STD_3";
  case IQN2FL_CHAN_RADIO_SEL_STD_4:
    return "IQN2FL_CHAN_RADIO_SEL_STD_4";
  case IQN2FL_CHAN_RADIO_SEL_STD_5:
    return "IQN2FL_CHAN_RADIO_SEL_STD_5";
  case IQN2FL_CHAN_RADIO_SEL_STD_6:
    return "IQN2FL_CHAN_RADIO_SEL_STD_6";
  case IQN2FL_CHAN_RADIO_SEL_STD_7:
    return "IQN2FL_CHAN_RADIO_SEL_STD_7";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_DataWidth (Iqn2Fl_DataWidth value) {
  switch(value) {
  case IQN2FL_DATA_WIDTH_15_BIT:
    return "IQN2FL_DATA_WIDTH_15_BIT";
  case IQN2FL_DATA_WIDTH_16_BIT:
    return "IQN2FL_DATA_WIDTH_16_BIT";
  case IQN2FL_DATA_WIDTH_7_BIT:
    return "IQN2FL_DATA_WIDTH_7_BIT";
  case IQN2FL_DATA_WIDTH_8_BIT:
    return "IQN2FL_DATA_WIDTH_8_BIT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_HwControlCmd (Iqn2Fl_HwControlCmd value) {
  switch(value) {
  case IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AID2_EFE_CONFIG_LOOPBACK_EN:
    return "IQN2FL_CMD_AID2_EFE_CONFIG_LOOPBACK_EN";
  case IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AID2_UAT_EGR_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_AID2_UAT_EGR_RADT_OFFSET_CFG";
  case IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG:
    return "IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG";
  case IQN2FL_CMD_AID2_UAT_ING_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_AID2_UAT_ING_RADT_OFFSET_CFG";
  case IQN2FL_CMD_AID2_UAT_RADT_EVT_PER_RADT:
    return "IQN2FL_CMD_AID2_UAT_RADT_EVT_PER_RADT";
  case IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AIL_EFE_CFG_LOOPBACK_EN_REG:
    return "IQN2FL_CMD_AIL_EFE_CFG_LOOPBACK_EN_REG";
  case IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_AIL_PD_CPRI_AXC_RADSTD_CFG:
    return "IQN2FL_CMD_AIL_PD_CPRI_AXC_RADSTD_CFG";
  case IQN2FL_CMD_AIL_PE_CPRI_RADSTD_CFG:
    return "IQN2FL_CMD_AIL_PE_CPRI_RADSTD_CFG";
  case IQN2FL_CMD_AIL_PHY_CI_LUTA_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CI_LUTA_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_CI_LUTB_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CI_LUTB_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_CI_LUT_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CI_LUT_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_CO_LUTA_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CO_LUTA_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_CO_LUTB_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CO_LUTB_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_CO_LUT_CFG_REG:
    return "IQN2FL_CMD_AIL_PHY_CO_LUT_CFG_REG";
  case IQN2FL_CMD_AIL_PHY_RM_CFG_RX_EN_REG:
    return "IQN2FL_CMD_AIL_PHY_RM_CFG_RX_EN_REG";
  case IQN2FL_CMD_AIL_PHY_TM_CFG_EN_REG:
    return "IQN2FL_CMD_AIL_PHY_TM_CFG_EN_REG";
  case IQN2FL_CMD_AIL_SI_IQ_E_SCH_PHY_EN_REG:
    return "IQN2FL_CMD_AIL_SI_IQ_E_SCH_PHY_EN_REG";
  case IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_EGR_RADT_TC_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_EGR_RADT_TC_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_EVT_CLK_CNT_TC_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_EVT_CLK_CNT_TC_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_EVT_RADT_CMP_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_EVT_RADT_CMP_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG:
    return "IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG";
  case IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_TERMINAL_COUNT_REG:
    return "IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_TERMINAL_COUNT_REG";
  case IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_ING_RADT_TC_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_ING_RADT_TC_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_PE_FB_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_PE_FB_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_PIMAX_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_PIMAX_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_PIMIN_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_PIMIN_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_RT_FB_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_RT_FB_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_TM_BFN_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_TM_BFN_CFG_REG";
  case IQN2FL_CMD_AIL_UAT_TM_FB_CFG_REG:
    return "IQN2FL_CMD_AIL_UAT_TM_FB_CFG_REG";
  case IQN2FL_CMD_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL:
    return "IQN2FL_CMD_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL";
  case IQN2FL_CMD_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL:
    return "IQN2FL_CMD_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL";
  case IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL:
    return "IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL";
  case IQN2FL_CMD_AT2_EVENTS_24ARRAY_CFG:
    return "IQN2FL_CMD_AT2_EVENTS_24ARRAY_CFG";
  case IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG:
    return "IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG";
  case IQN2FL_CMD_AT2_EVT_FORCE:
    return "IQN2FL_CMD_AT2_EVT_FORCE";
  case IQN2FL_CMD_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB:
    return "IQN2FL_CMD_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB";
  case IQN2FL_CMD_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB:
    return "IQN2FL_CMD_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB";
  case IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN:
    return "IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN";
  case IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN:
    return "IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN";
  case IQN2FL_CMD_DIO2_CORE_EGRESS_CFG:
    return "IQN2FL_CMD_DIO2_CORE_EGRESS_CFG";
  case IQN2FL_CMD_DIO2_CORE_INGRESS_CFG:
    return "IQN2FL_CMD_DIO2_CORE_INGRESS_CFG";
  case IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_DIO2_DT_START:
    return "IQN2FL_CMD_DIO2_DT_START";
  case IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_DIO2_E_DBCNT_RAM_MMR_CFG:
    return "IQN2FL_CMD_DIO2_E_DBCNT_RAM_MMR_CFG";
  case IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_CLEAR:
    return "IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_CLEAR";
  case IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET:
    return "IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET";
  case IQN2FL_CMD_DIO2_I_DBCNT_RAM_MMR_CFG:
    return "IQN2FL_CMD_DIO2_I_DBCNT_RAM_MMR_CFG";
  case IQN2FL_CMD_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG";
  case IQN2FL_CMD_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG";
  case IQN2FL_CMD_DIO2_UAT_EGR_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_DIO2_UAT_EGR_RADT_OFFSET_CFG";
  case IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG:
    return "IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG";
  case IQN2FL_CMD_DIO2_UAT_ING_RADT_OFFSET_CFG:
    return "IQN2FL_CMD_DIO2_UAT_ING_RADT_OFFSET_CFG";
  case IQN2FL_CMD_DIO2_UAT_RADT_EVT_PER_RADT:
    return "IQN2FL_CMD_DIO2_UAT_RADT_EVT_PER_RADT";
  case IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB:
    return "IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_HwStatusQuery (Iqn2Fl_HwStatusQuery value) {
  switch(value) {
  case IQN2FL_QUERY_AID2_ECTL_CHAN_ON_STS:
    return "IQN2FL_QUERY_AID2_ECTL_CHAN_ON_STS";
  case IQN2FL_QUERY_AID2_ECTL_EOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_ECTL_EOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_ECTL_INPKT_STS:
    return "IQN2FL_QUERY_AID2_ECTL_INPKT_STS";
  case IQN2FL_QUERY_AID2_ECTL_SOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_ECTL_SOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_ICTL_CHAN_ON_STS:
    return "IQN2FL_QUERY_AID2_ICTL_CHAN_ON_STS";
  case IQN2FL_QUERY_AID2_ICTL_EOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_ICTL_EOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_ICTL_INPKT_STS:
    return "IQN2FL_QUERY_AID2_ICTL_INPKT_STS";
  case IQN2FL_QUERY_AID2_ICTL_SOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_ICTL_SOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_IQ_EDC_EOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_IQ_EDC_EOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_IQ_EDC_SOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_IQ_EDC_SOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_IQ_EFE_CHAN_ON_STS:
    return "IQN2FL_QUERY_AID2_IQ_EFE_CHAN_ON_STS";
  case IQN2FL_QUERY_AID2_IQ_EFE_DMA_SYNC_STS:
    return "IQN2FL_QUERY_AID2_IQ_EFE_DMA_SYNC_STS";
  case IQN2FL_QUERY_AID2_IQ_EFE_IN_PKT_STS:
    return "IQN2FL_QUERY_AID2_IQ_EFE_IN_PKT_STS";
  case IQN2FL_QUERY_AID2_IQ_IDC_EOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_IQ_IDC_EOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_IQ_IDC_INPKT_STS:
    return "IQN2FL_QUERY_AID2_IQ_IDC_INPKT_STS";
  case IQN2FL_QUERY_AID2_IQ_IDC_SOP_CNTR_STS:
    return "IQN2FL_QUERY_AID2_IQ_IDC_SOP_CNTR_STS";
  case IQN2FL_QUERY_AID2_IQ_IDC_STS:
    return "IQN2FL_QUERY_AID2_IQ_IDC_STS";
  case IQN2FL_QUERY_AID2_IQ_IFE_CHAN_ON_STS:
    return "IQN2FL_QUERY_AID2_IQ_IFE_CHAN_ON_STS";
  case IQN2FL_QUERY_AID2_IQ_IFE_IN_PKT_STS:
    return "IQN2FL_QUERY_AID2_IQ_IFE_IN_PKT_STS";
  case IQN2FL_QUERY_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_AID2_UAT_EGR_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_AID2_UAT_ING_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_AID2_UAT_SYNC_BCN_CAPTURE_STS:
    return "IQN2FL_QUERY_AID2_UAT_SYNC_BCN_CAPTURE_STS";
  case IQN2FL_QUERY_AT2_BCN_FRM_VALUE_LSB_STS:
    return "IQN2FL_QUERY_AT2_BCN_FRM_VALUE_LSB_STS";
  case IQN2FL_QUERY_AT2_BCN_FRM_VALUE_MSB_STS:
    return "IQN2FL_QUERY_AT2_BCN_FRM_VALUE_MSB_STS";
  case IQN2FL_QUERY_AT2_BCN_PA_TSCOMP_CAPT_STS:
    return "IQN2FL_QUERY_AT2_BCN_PA_TSCOMP_CAPT_STS";
  case IQN2FL_QUERY_AT2_BCN_PHYSYNC_CAPT_STS:
    return "IQN2FL_QUERY_AT2_BCN_PHYSYNC_CAPT_STS";
  case IQN2FL_QUERY_AT2_BCN_RADSYNC_CAPT_STS:
    return "IQN2FL_QUERY_AT2_BCN_RADSYNC_CAPT_STS";
  case IQN2FL_QUERY_AT2_BCN_RP1_SYNC_CAPT_STS:
    return "IQN2FL_QUERY_AT2_BCN_RP1_SYNC_CAPT_STS";
  case IQN2FL_QUERY_AT2_BCN_SLVSYNC_CAPT_STS:
    return "IQN2FL_QUERY_AT2_BCN_SLVSYNC_CAPT_STS";
  case IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS:
    return "IQN2FL_QUERY_AT2_EVENTS_ENABLE_STS";
  case IQN2FL_QUERY_AT2_GSM_TCOUNT_VALUE_STS:
    return "IQN2FL_QUERY_AT2_GSM_TCOUNT_VALUE_STS";
  case IQN2FL_QUERY_AT2_RADT_STS:
    return "IQN2FL_QUERY_AT2_RADT_STS";
  case IQN2FL_QUERY_DIO2_IQ_EDC_EOP_CNTR_STS:
    return "IQN2FL_QUERY_DIO2_IQ_EDC_EOP_CNTR_STS";
  case IQN2FL_QUERY_DIO2_IQ_EDC_SOP_CNTR_STS:
    return "IQN2FL_QUERY_DIO2_IQ_EDC_SOP_CNTR_STS";
  case IQN2FL_QUERY_DIO2_IQ_EFE_CHAN_ON_STS:
    return "IQN2FL_QUERY_DIO2_IQ_EFE_CHAN_ON_STS";
  case IQN2FL_QUERY_DIO2_IQ_EFE_DMA_SYNC_STS:
    return "IQN2FL_QUERY_DIO2_IQ_EFE_DMA_SYNC_STS";
  case IQN2FL_QUERY_DIO2_IQ_EFE_IN_PKT_STS:
    return "IQN2FL_QUERY_DIO2_IQ_EFE_IN_PKT_STS";
  case IQN2FL_QUERY_DIO2_IQ_IDC_EOP_CNTR_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IDC_EOP_CNTR_STS";
  case IQN2FL_QUERY_DIO2_IQ_IDC_INPKT_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IDC_INPKT_STS";
  case IQN2FL_QUERY_DIO2_IQ_IDC_SOP_CNTR_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IDC_SOP_CNTR_STS";
  case IQN2FL_QUERY_DIO2_IQ_IDC_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IDC_STS";
  case IQN2FL_QUERY_DIO2_IQ_IFE_CHAN_ON_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IFE_CHAN_ON_STS";
  case IQN2FL_QUERY_DIO2_IQ_IFE_IN_PKT_STS:
    return "IQN2FL_QUERY_DIO2_IQ_IFE_IN_PKT_STS";
  case IQN2FL_QUERY_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_DIO2_UAT_DIO_EGR_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_DIO2_UAT_DIO_ING_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_DIO2_UAT_EGR_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS:
    return "IQN2FL_QUERY_DIO2_UAT_ING_SYNC_RADT_CAPTURE_STS";
  case IQN2FL_QUERY_DIO2_UAT_SYNC_BCN_CAPTURE_STS:
    return "IQN2FL_QUERY_DIO2_UAT_SYNC_BCN_CAPTURE_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_H_ENABLE_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_H_ENABLE_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_H_PKT_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_H_PKT_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_H_TDOWN_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_H_TDOWN_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_L_ENABLE_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_L_ENABLE_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_L_PKT_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_L_PKT_STS";
  case IQN2FL_QUERY_VC_CDMA_RSTAT_L_TDOWN_STS:
    return "IQN2FL_QUERY_VC_CDMA_RSTAT_L_TDOWN_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_H_ENABLE_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_H_ENABLE_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_H_PKT_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_H_PKT_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_H_TDOWN_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_H_TDOWN_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_L_ENABLE_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_L_ENABLE_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_L_PKT_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_L_PKT_STS";
  case IQN2FL_QUERY_VC_CDMA_TSTAT_L_TDOWN_STS:
    return "IQN2FL_QUERY_VC_CDMA_TSTAT_L_TDOWN_STS";
  case IQN2FL_QUERY_VC_SD_RX_STS:
    return "IQN2FL_QUERY_VC_SD_RX_STS";
  case IQN2FL_QUERY_VC_SD_TX_STS:
    return "IQN2FL_QUERY_VC_SD_TX_STS";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_LinkRate (Iqn2Fl_LinkRate value) {
  switch(value) {
  case IQN2FL_LINK_RATE_10x:
    return "IQN2FL_LINK_RATE_10x";
  case IQN2FL_LINK_RATE_16x:
    return "IQN2FL_LINK_RATE_16x";
  case IQN2FL_LINK_RATE_2x:
    return "IQN2FL_LINK_RATE_2x";
  case IQN2FL_LINK_RATE_4x:
    return "IQN2FL_LINK_RATE_4x";
  case IQN2FL_LINK_RATE_5x:
    return "IQN2FL_LINK_RATE_5x";
  case IQN2FL_LINK_RATE_8x:
    return "IQN2FL_LINK_RATE_8x";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Iqn2Fl_Status (Iqn2Fl_Status value) {
  switch(value) {
  case IQN2FL_SOK:
    return "IQN2FL_SOK";
  default:
    return "Illegal Enumeration Value";
  }
}

/* Structure dumping functions.  */
void dump_Iqn2Fl_Aid2IqEfeCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqEfeCfgGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqEfeCfgGrpSetup  */\n");
  fprintf(output, "  chan_en[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    chan_en[%d] = %d\n", i, value->chan_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  chan_radio_sel[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    chan_radio_sel[%d] = %d\n", i, value->chan_radio_sel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  chan_tdd_frc_off[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    chan_tdd_frc_off[%d] = %d\n", i, value->chan_tdd_frc_off[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  loopback_en = %d\n", value->loopback_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2IqEfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Aid2IqEfeRadioStdGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqEfeRadioStdGrpSetup  */\n");
  fprintf(output, "  index_sc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    index_sc[%d] = %d\n", i, value->index_sc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  index_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    index_tc[%d] = %d\n", i, value->index_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  sym_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    sym_tc[%d] = %d\n", i, value->sym_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  tdd_en_cfg[8][5] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "  -- %d", i);
    for(j=0; j<5; j++) {
      fprintf(output, "    tdd_en_cfg[%d][%d] = %d\n", i, j, value->tdd_en_cfg[i][j]);
    }
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  tdd_first_sym[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    tdd_first_sym[%d] = %d\n", i, value->tdd_first_sym[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  tdd_lut_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    tdd_lut_en[%d] = %d\n", i, value->tdd_lut_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2IqIfeChanCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqIfeChanCfgGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqIfeChanCfgGrpSetup  */\n");
  fprintf(output, "  chan_axc_offset = %d\n", value->chan_axc_offset);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  chan_radio_sel = %d\n", value->chan_radio_sel);
  fprintf(output, "  chan_tdd_frc_off = %d\n", value->chan_tdd_frc_off);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2IqIfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Aid2IqIfeRadioStdGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqIfeRadioStdGrpSetup  */\n");
  fprintf(output, "  index_sc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    index_sc[%d] = %d\n", i, value->index_sc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  index_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    index_tc[%d] = %d\n", i, value->index_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  sym_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    sym_tc[%d] = %d\n", i, value->sym_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  tdd_en_cfg[8][5] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "  -- %d", i);
    for(j=0; j<5; j++) {
      fprintf(output, "    tdd_en_cfg[%d][%d] = %d\n", i, j, value->tdd_en_cfg[i][j]);
    }
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  tdd_lut_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    tdd_lut_en[%d] = %d\n", i, value->tdd_lut_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2IqIngChanCfgGrpSetup (FILE *output, Iqn2Fl_Aid2IqIngChanCfgGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqIngChanCfgGrpSetup  */\n");
  fprintf(output, "  chan_frc_off = %d\n", value->chan_frc_off);
  fprintf(output, "  dat_swap = %d\n", value->dat_swap);
  fprintf(output, "  iq_order = %d\n", value->iq_order);
  fprintf(output, "  pkt_type = %d\n", value->pkt_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2IqUatGenCtlSetup (FILE *output, Iqn2Fl_Aid2IqUatGenCtlSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2IqUatGenCtlSetup  */\n");
  fprintf(output, "  bcn_offset_cfg_val = %d\n", value->bcn_offset_cfg_val);
  fprintf(output, "  bcn_tc_cfg_val = %d\n", value->bcn_tc_cfg_val);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Aid2Setup (FILE *output, Iqn2Fl_Aid2Setup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Aid2Setup  */\n");
  fprintf(output, "  aid2_ictl_idc_if_ch_cfg[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    aid2_ictl_idc_if_ch_cfg[%d] = ", i);
    dump_Iqn2Fl_Aid2IqIngChanCfgGrpSetup(output, &(value->aid2_ictl_idc_if_ch_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  aid2_ictl_idc_if_ictl_cfg_fail_mark_only = %d\n", value->aid2_ictl_idc_if_ictl_cfg_fail_mark_only);
  fprintf(output, "  aid2_ictl_idc_if_ictl_cfg_force_off_all = %d\n", value->aid2_ictl_idc_if_ictl_cfg_force_off_all);
    fprintf(output, "  aid2_iq_efe_cfg_grp = ");
    dump_Iqn2Fl_Aid2IqEfeCfgGrpSetup(output, &(value->aid2_iq_efe_cfg_grp));
    fprintf(output, "  aid2_iq_efe_radio_std_grp = ");
    dump_Iqn2Fl_Aid2IqEfeRadioStdGrpSetup(output, &(value->aid2_iq_efe_radio_std_grp));
  fprintf(output, "  aid2_iq_idc_chan_cfg_grp[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    aid2_iq_idc_chan_cfg_grp[%d] = ", i);
    dump_Iqn2Fl_Aid2IqIngChanCfgGrpSetup(output, &(value->aid2_iq_idc_chan_cfg_grp[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  aid2_iq_ife_chan_cfg_grp[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    aid2_iq_ife_chan_cfg_grp[%d] = ", i);
    dump_Iqn2Fl_Aid2IqIfeChanCfgGrpSetup(output, &(value->aid2_iq_ife_chan_cfg_grp[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  aid2_iq_ife_radio_std_grp = ");
    dump_Iqn2Fl_Aid2IqIfeRadioStdGrpSetup(output, &(value->aid2_iq_ife_radio_std_grp));
    fprintf(output, "  aid2_iq_uat_gen_ctl = ");
    dump_Iqn2Fl_Aid2IqUatGenCtlSetup(output, &(value->aid2_iq_uat_gen_ctl));
  fprintf(output, "  ectl_ch_cfg_dat_swap[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    ectl_ch_cfg_dat_swap[%d] = %d\n", i, value->ectl_ch_cfg_dat_swap[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ectl_ch_cfg_iq_order[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    ectl_ch_cfg_iq_order[%d] = %d\n", i, value->ectl_ch_cfg_iq_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ectl_chan_en[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    ectl_chan_en[%d] = %d\n", i, value->ectl_chan_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ectl_channel[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    ectl_channel[%d] = %d\n", i, value->ectl_channel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ectl_rate_ctl_cfg = %d\n", value->ectl_rate_ctl_cfg);
  fprintf(output, "  edc_cfg_psi_err_chk_disable = %d\n", value->edc_cfg_psi_err_chk_disable);
  fprintf(output, "  edc_ch_cfg_dat_swap[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    edc_ch_cfg_dat_swap[%d] = %d\n", i, value->edc_ch_cfg_dat_swap[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  edc_ch_cfg_iq_order[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    edc_ch_cfg_iq_order[%d] = %d\n", i, value->edc_ch_cfg_iq_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_axc_offset[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    efe_axc_offset[%d] = %d\n", i, value->efe_axc_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_chan_index_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_chan_index_cfg[%d] = %d\n", i, value->efe_chan_index_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_chan_index_en_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_chan_index_en_cfg[%d] = %d\n", i, value->efe_chan_index_en_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_samp_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_samp_tc[%d] = %d\n", i, value->efe_samp_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_tdm_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_tdm_en[%d] = %d\n", i, value->efe_tdm_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_tdm_len[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_tdm_len[%d] = %d\n", i, value->efe_tdm_len[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_tdm_start[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_tdm_start[%d] = %d\n", i, value->efe_tdm_start[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ictl_pkt_if_chan_en[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    ictl_pkt_if_chan_en[%d] = %d\n", i, value->ictl_pkt_if_chan_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ictl_rate_ctl_cfg_rate = %d\n", value->ictl_rate_ctl_cfg_rate);
  fprintf(output, "  idc_fail_mark_only = %d\n", value->idc_fail_mark_only);
  fprintf(output, "  idc_frc_off_all = %d\n", value->idc_frc_off_all);
  fprintf(output, "  idc_rate_ctl_cfg_rate = %d\n", value->idc_rate_ctl_cfg_rate);
  fprintf(output, "  ife_samp_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ife_samp_tc[%d] = %d\n", i, value->ife_samp_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_egr_radt_offset_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_egr_radt_offset_cfg_val[%d] = %d\n", i, value->uat_egr_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_egr_radt_tc_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_egr_radt_tc_cfg_val[%d] = %d\n", i, value->uat_egr_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_evt_clk_cnt_tc_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    uat_evt_clk_cnt_tc_cfg_val[%d] = %d\n", i, value->uat_evt_clk_cnt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_evt_radt_cmp_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    uat_evt_radt_cmp_cfg_val[%d] = %d\n", i, value->uat_evt_radt_cmp_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_ing_radt_offset_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_ing_radt_offset_cfg_val[%d] = %d\n", i, value->uat_ing_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_ing_radt_tc_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_ing_radt_tc_cfg_val[%d] = %d\n", i, value->uat_ing_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilEctlPktIf (FILE *output, Iqn2Fl_AilEctlPktIf *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilEctlPktIf  */\n");
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  channel = %d\n", value->channel);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilEctlRegGrp (FILE *output, Iqn2Fl_AilEctlRegGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilEctlRegGrp  */\n");
  fprintf(output, "  dat_swap[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    dat_swap[%d] = %d\n", i, value->dat_swap[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  iq_order[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    iq_order[%d] = %d\n", i, value->iq_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  rate = %d\n", value->rate);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilEdcRegGrp (FILE *output, Iqn2Fl_AilEdcRegGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilEdcRegGrp  */\n");
  fprintf(output, "  dat_swap[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    dat_swap[%d] = %d\n", i, value->dat_swap[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  iq_order[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    iq_order[%d] = %d\n", i, value->iq_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  psi_err_chk_disable = %d\n", value->psi_err_chk_disable);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilEgrSetup (FILE *output, Iqn2Fl_AilEgrSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilEgrSetup  */\n");
  fprintf(output, "  ail_ectl_pkt_if[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail_ectl_pkt_if[%d] = ", i);
    dump_Iqn2Fl_AilEctlPktIf(output, &(value->ail_ectl_pkt_if[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  ail_ectl_reg_grp = ");
    dump_Iqn2Fl_AilEctlRegGrp(output, &(value->ail_ectl_reg_grp));
    fprintf(output, "  ail_iq_edc_reg_grp = ");
    dump_Iqn2Fl_AilEdcRegGrp(output, &(value->ail_iq_edc_reg_grp));
  fprintf(output, "  ail_iq_efe_chan_axc_offset[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_iq_efe_chan_axc_offset[%d] = %d\n", i, value->ail_iq_efe_chan_axc_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_frm_samp_tc_mmr_ram[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ail_iq_efe_frm_samp_tc_mmr_ram[%d] = %d\n", i, value->ail_iq_efe_frm_samp_tc_mmr_ram[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_pe_axc_tdm_lut_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ail_iq_pe_axc_tdm_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_AilSiIqEgrTdmLutRam(output, &(value->ail_iq_pe_axc_tdm_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_pe_obsai_modtxrule_cfg[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    ail_iq_pe_obsai_modtxrule_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqPeObsaiModtxruleCfg(output, &(value->ail_iq_pe_obsai_modtxrule_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_si_iq_e_obsai_dbm_bitmap_ram[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ail_si_iq_e_obsai_dbm_bitmap_ram[%d] = %d\n", i, value->ail_si_iq_e_obsai_dbm_bitmap_ram[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_si_iq_e_obsai_dbm_rule[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    ail_si_iq_e_obsai_dbm_rule[%d] = ", i);
    dump_Iqn2Fl_AilSiIqEgrObsaiDbmRule(output, &(value->ail_si_iq_e_obsai_dbm_rule[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  ail_si_iq_e_sch_cpri = ");
    dump_Iqn2Fl_AilSiIqEgrSchCpri(output, &(value->ail_si_iq_e_sch_cpri));
    fprintf(output, "  ail_si_iq_efe_config_group = ");
    dump_Iqn2Fl_AilSiIqEfeCfgGrp(output, &(value->ail_si_iq_efe_config_group));
    fprintf(output, "  ail_si_iq_efe_radio_std_group = ");
    dump_Iqn2Fl_AilSiIqEfeRadStdGrp(output, &(value->ail_si_iq_efe_radio_std_group));
  fprintf(output, "  phy_en = %d\n", value->phy_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIctlIdcIf (FILE *output, Iqn2Fl_AilIctlIdcIf *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIctlIdcIf  */\n");
    fprintf(output, "  ictl_cfg = ");
    dump_Iqn2Fl_AilIqIgrCfgGrp(output, &(value->ictl_cfg));
  fprintf(output, "  ictl_chan_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ictl_chan_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqIgrChCfgGrp(output, &(value->ictl_chan_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIgrSetup (FILE *output, Iqn2Fl_AilIgrSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIgrSetup  */\n");
    fprintf(output, "  ail_ictl_idc_if = ");
    dump_Iqn2Fl_AilIctlIdcIf(output, &(value->ail_ictl_idc_if));
  fprintf(output, "  ail_ictl_pkt_if_chan_en[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail_ictl_pkt_if_chan_en[%d] = %d\n", i, value->ail_ictl_pkt_if_chan_en[i]);
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  ail_iq_idc_cfg_grp = ");
    dump_Iqn2Fl_AilIqIgrCfgGrp(output, &(value->ail_iq_idc_cfg_grp));
  fprintf(output, "  ail_iq_idc_ch_cfg_grp[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_iq_idc_ch_cfg_grp[%d] = ", i);
    dump_Iqn2Fl_AilIqIgrChCfgGrp(output, &(value->ail_iq_idc_ch_cfg_grp[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_chan_config_group[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_iq_ife_chan_config_group[%d] = ", i);
    dump_Iqn2Fl_AilIqIfeChanCfgGrp(output, &(value->ail_iq_ife_chan_config_group[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  ail_iq_ife_radio_std_group = ");
    dump_Iqn2Fl_AilIqIfeRadStdGrp(output, &(value->ail_iq_ife_radio_std_group));
  fprintf(output, "  rate_ictl = %d\n", value->rate_ictl);
  fprintf(output, "  rate_idc = %d\n", value->rate_idc);
  fprintf(output, "  samp_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    samp_tc[%d] = %d\n", i, value->samp_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqEfeChanCfg (FILE *output, Iqn2Fl_AilIqEfeChanCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqEfeChanCfg  */\n");
  fprintf(output, "  axc_fine_offset = %d\n", value->axc_fine_offset);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  chan_enet_ctl = %s\n", dump_Iqn2Fl_ChanEnetCtlMode(value->chan_enet_ctl));
  fprintf(output, "  chan_obsai_ctl = %s\n", dump_Iqn2Fl_ChanObsaiCtl(value->chan_obsai_ctl));
  fprintf(output, "  chan_radio_sel = %s\n", dump_Iqn2Fl_ChanRadioSel(value->chan_radio_sel));
  fprintf(output, "  chan_tdd_frc_off = %s\n", dump_Iqn2Fl_ChanFrcOff(value->chan_tdd_frc_off));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqEfeRadStdCfg (FILE *output, Iqn2Fl_AilIqEfeRadStdCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqEfeRadStdCfg  */\n");
  fprintf(output, "  gsm_axc_bbhop_mode = %d\n", value->gsm_axc_bbhop_mode);
  fprintf(output, "  gsm_cmp_mode = %d\n", value->gsm_cmp_mode);
  fprintf(output, "  tdd_first_sym = %d\n", value->tdd_first_sym);
  fprintf(output, "  tdd_lut_en = %d\n", value->tdd_lut_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqFrmTcCfg (FILE *output, Iqn2Fl_AilIqFrmTcCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqFrmTcCfg  */\n");
  fprintf(output, "  index_sc = %d\n", value->index_sc);
  fprintf(output, "  index_tc = %d\n", value->index_tc);
  fprintf(output, "  sym_tc = %d\n", value->sym_tc);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqIfeChanCfgGrp (FILE *output, Iqn2Fl_AilIqIfeChanCfgGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqIfeChanCfgGrp  */\n");
  fprintf(output, "  chan_axc_offset = %d\n", value->chan_axc_offset);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  chan_enet_ctl = %s\n", dump_Iqn2Fl_ChanEnetCtlMode(value->chan_enet_ctl));
  fprintf(output, "  chan_obsai_ctl = %s\n", dump_Iqn2Fl_ChanObsaiCtl(value->chan_obsai_ctl));
  fprintf(output, "  chan_radio_sel = %s\n", dump_Iqn2Fl_ChanRadioSel(value->chan_radio_sel));
  fprintf(output, "  chan_tdd_frc_off = %s\n", dump_Iqn2Fl_ChanFrcOff(value->chan_tdd_frc_off));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqIfeRadStdCfg (FILE *output, Iqn2Fl_AilIqIfeRadStdCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqIfeRadStdCfg  */\n");
  fprintf(output, "  tdd_lut_en = %d\n", value->tdd_lut_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqIfeRadStdGrp (FILE *output, Iqn2Fl_AilIqIfeRadStdGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqIfeRadStdGrp  */\n");
  fprintf(output, "  ail_iq_ife_frm_tc_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_iq_ife_frm_tc_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqFrmTcCfg(output, &(value->ail_iq_ife_frm_tc_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_rad_std_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_iq_ife_rad_std_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqIfeRadStdCfg(output, &(value->ail_iq_ife_rad_std_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg0[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg0[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg0[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg1[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg1[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg2[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg2[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg3[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg3[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg4[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg4[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg5[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg5[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg5[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg6[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg6[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg6[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_ife_tdd_en_cfg7[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_ife_tdd_en_cfg7[%d] = %d\n", i, value->ail_iq_ife_tdd_en_cfg7[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqIgrCfgGrp (FILE *output, Iqn2Fl_AilIqIgrCfgGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqIgrCfgGrp  */\n");
  fprintf(output, "  fail_mark_only = %d\n", value->fail_mark_only);
  fprintf(output, "  frc_off_all = %d\n", value->frc_off_all);
  fprintf(output, "  rm_fail_frc_off_en = %d\n", value->rm_fail_frc_off_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqIgrChCfgGrp (FILE *output, Iqn2Fl_AilIqIgrChCfgGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqIgrChCfgGrp  */\n");
  fprintf(output, "  chan_frc_off = %d\n", value->chan_frc_off);
  fprintf(output, "  dat_swap = %d\n", value->dat_swap);
  fprintf(output, "  iq_order = %d\n", value->iq_order);
  fprintf(output, "  pkt_type = %d\n", value->pkt_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilIqPeObsaiModtxruleCfg (FILE *output, Iqn2Fl_AilIqPeObsaiModtxruleCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilIqPeObsaiModtxruleCfg  */\n");
  fprintf(output, "  rule_ctl_msg = %d\n", value->rule_ctl_msg);
  fprintf(output, "  rule_en = %d\n", value->rule_en);
  fprintf(output, "  rule_index = %d\n", value->rule_index);
  fprintf(output, "  rule_mod = %d\n", value->rule_mod);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCommon (FILE *output, Iqn2Fl_AilPdCommon *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCommon  */\n");
  fprintf(output, "  axc_offset = %d\n", value->axc_offset);
  fprintf(output, "  rad_std = %d\n", value->rad_std);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCpriAxc0Cfg (FILE *output, Iqn2Fl_AilPdCpriAxc0Cfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCpriAxc0Cfg  */\n");
  fprintf(output, "  cont_lut_en = %d\n", value->cont_lut_en);
  fprintf(output, "  cont_lut_grp = %d\n", value->cont_lut_grp);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCpriAxcCfg (FILE *output, Iqn2Fl_AilPdCpriAxcCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCpriAxcCfg  */\n");
    fprintf(output, "  ail_iq_pd_cpri_axc_radstd_cfg = ");
    dump_Iqn2Fl_AilSiIqCpriRadstdCfg(output, &(value->ail_iq_pd_cpri_axc_radstd_cfg));
  fprintf(output, "  ail_pd_cpri_axc0_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_pd_cpri_axc0_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdCpriAxc0Cfg(output, &(value->ail_pd_cpri_axc0_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_pd_cpri_tdm_fsm_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_pd_cpri_tdm_fsm_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdCpriTdmFsmCfg(output, &(value->ail_pd_cpri_tdm_fsm_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  axc_tdm_lut_cfg_axc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    axc_tdm_lut_cfg_axc[%d] = %d\n", i, value->axc_tdm_lut_cfg_axc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  axc_tdm_lut_cfg_en[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    axc_tdm_lut_cfg_en[%d] = %d\n", i, value->axc_tdm_lut_cfg_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bub_fsm2_cfg_gap_frac[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    bub_fsm2_cfg_gap_frac[%d] = %d\n", i, value->bub_fsm2_cfg_gap_frac[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bub_fsm2_cfg_gap_int[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    bub_fsm2_cfg_gap_int[%d] = %d\n", i, value->bub_fsm2_cfg_gap_int[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bub_fsm_cfg_knc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    bub_fsm_cfg_knc[%d] = %d\n", i, value->bub_fsm_cfg_knc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCpriCwCfg (FILE *output, Iqn2Fl_AilPdCpriCwCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCpriCwCfg  */\n");
  fprintf(output, "  chan_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    chan_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdCpriCwChanCfg(output, &(value->chan_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  cpri_4b5b_cfg_bit_order[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    cpri_4b5b_cfg_bit_order[%d] = %d\n", i, value->cpri_4b5b_cfg_bit_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  cpri_4b5b_cfg_hdr[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    cpri_4b5b_cfg_hdr[%d] = %d\n", i, value->cpri_4b5b_cfg_hdr[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  cpri_4b5b_cfg_ssd_order[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    cpri_4b5b_cfg_ssd_order[%d] = %d\n", i, value->cpri_4b5b_cfg_ssd_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  crc_cfg_crc8_comp = %d\n", value->crc_cfg_crc8_comp);
  fprintf(output, "  crc_cfg_crc8_poly = %d\n", value->crc_cfg_crc8_poly);
  fprintf(output, "  hypfrm0_lut_cfg_hf_en = %d\n", value->hypfrm0_lut_cfg_hf_en);
  fprintf(output, "  hypfrm1_lut_cfg_hf_en = %d\n", value->hypfrm1_lut_cfg_hf_en);
  fprintf(output, "  hypfrm2_lut_cfg_hf_en = %d\n", value->hypfrm2_lut_cfg_hf_en);
  fprintf(output, "  hypfrm3_lut_cfg_hf_en = %d\n", value->hypfrm3_lut_cfg_hf_en);
  fprintf(output, "  hypfrm4_lut_cfg_hf_en = %d\n", value->hypfrm4_lut_cfg_hf_en);
  fprintf(output, "  lut_cfg_cw_chan[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    lut_cfg_cw_chan[%d] = %d\n", i, value->lut_cfg_cw_chan[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  lut_cfg_cw_en[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    lut_cfg_cw_en[%d] = %d\n", i, value->lut_cfg_cw_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  null_cfg_null_char = %d\n", value->null_cfg_null_char);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCpriCwChanCfg (FILE *output, Iqn2Fl_AilPdCpriCwChanCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCpriCwChanCfg  */\n");
  fprintf(output, "  byte_en = %d\n", value->byte_en);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  crc_init = %d\n", value->crc_init);
  fprintf(output, "  crc_sel = %d\n", value->crc_sel);
  fprintf(output, "  delin_sel = %d\n", value->delin_sel);
  fprintf(output, "  dlmt_imux = %d\n", value->dlmt_imux);
  fprintf(output, "  dlmt_omux = %d\n", value->dlmt_omux);
  fprintf(output, "  hdlc_rvrs_crc = %d\n", value->hdlc_rvrs_crc);
  fprintf(output, "  hf_lut_en = %d\n", value->hf_lut_en);
  fprintf(output, "  qwd_omux = %d\n", value->qwd_omux);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdCpriTdmFsmCfg (FILE *output, Iqn2Fl_AilPdCpriTdmFsmCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdCpriTdmFsmCfg  */\n");
//  fprintf(output, "  en = %d\n", value->en);
  fprintf(output, "  ncont = %d\n", value->ncont);
  fprintf(output, "  start_lut = %d\n", value->start_lut);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdObsaiCfg (FILE *output, Iqn2Fl_AilPdObsaiCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdObsaiCfg  */\n");
  fprintf(output, "  frm_msg_tc_cfg_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    frm_msg_tc_cfg_tc[%d] = %d\n", i, value->frm_msg_tc_cfg_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  frm_tc_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    frm_tc_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqFrmTcCfg(output, &(value->frm_tc_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  gsm_bbhop_cfg_off_stb[2] = [\n");
  for(i=0; i<2; i++) {
    fprintf(output, "    gsm_bbhop_cfg_off_stb[%d] = %d\n", i, value->gsm_bbhop_cfg_off_stb[i]);
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  lut_cfg = ");
    dump_Iqn2Fl_AilPdObsaiLutCfg(output, &(value->lut_cfg));
  fprintf(output, "  radstd_cfg_axcoffset_win[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd_cfg_axcoffset_win[%d] = %d\n", i, value->radstd_cfg_axcoffset_win[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  radstd_cfg_wdog_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd_cfg_wdog_tc[%d] = %d\n", i, value->radstd_cfg_wdog_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  radt_cfg_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radt_cfg_tc[%d] = %d\n", i, value->radt_cfg_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdObsaiChanCfg (FILE *output, Iqn2Fl_AilPdObsaiChanCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdObsaiChanCfg  */\n");
  fprintf(output, "  gsm_ul = %d\n", value->gsm_ul);
  fprintf(output, "  wdog_en = %d\n", value->wdog_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdObsaiLutCfg (FILE *output, Iqn2Fl_AilPdObsaiLutCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdObsaiLutCfg  */\n");
  fprintf(output, "  chan_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    chan_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdObsaiChanCfg(output, &(value->chan_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  route_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    route_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdObsaiRouteCfg(output, &(value->route_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  type_lut_cfg[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    type_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPdObsaiTypeLutCfg(output, &(value->type_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdObsaiRouteCfg (FILE *output, Iqn2Fl_AilPdObsaiRouteCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdObsaiRouteCfg  */\n");
  fprintf(output, "  chan_adr = %d\n", value->chan_adr);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  chan_mask = %d\n", value->chan_mask);
  fprintf(output, "  chan_ts = %d\n", value->chan_ts);
  fprintf(output, "  chan_type = %d\n", value->chan_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdObsaiTypeLutCfg (FILE *output, Iqn2Fl_AilPdObsaiTypeLutCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdObsaiTypeLutCfg  */\n");
  fprintf(output, "  crc_en = %d\n", value->crc_en);
  fprintf(output, "  crc_hdr = %d\n", value->crc_hdr);
  fprintf(output, "  crc_type = %d\n", value->crc_type);
  fprintf(output, "  enet_strip = %d\n", value->enet_strip);
  fprintf(output, "  obsai_pkt_en = %d\n", value->obsai_pkt_en);
  fprintf(output, "  rp3_01 = %d\n", value->rp3_01);
  fprintf(output, "  rp3_01_rst = %d\n", value->rp3_01_rst);
  fprintf(output, "  ts_format = %d\n", value->ts_format);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPdSetup (FILE *output, Iqn2Fl_AilPdSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPdSetup  */\n");
  fprintf(output, "  ail_pd_common[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_pd_common[%d] = ", i);
    dump_Iqn2Fl_AilPdCommon(output, &(value->ail_pd_common[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  ail_pd_cpri_axc_cfg = ");
    dump_Iqn2Fl_AilPdCpriAxcCfg(output, &(value->ail_pd_cpri_axc_cfg));
    fprintf(output, "  ail_pd_cpri_cw_cfg = ");
    dump_Iqn2Fl_AilPdCpriCwCfg(output, &(value->ail_pd_cpri_cw_cfg));
    fprintf(output, "  ail_pd_obsai_cfg = ");
    dump_Iqn2Fl_AilPdObsaiCfg(output, &(value->ail_pd_obsai_cfg));
  fprintf(output, "  ail_pd_obsai_frm_msg_tc_cfg_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ail_pd_obsai_frm_msg_tc_cfg_tc[%d] = %d\n", i, value->ail_pd_obsai_frm_msg_tc_cfg_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCommon (FILE *output, Iqn2Fl_AilPeCommon *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCommon  */\n");
  fprintf(output, "  ail_pe_common_chan_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_pe_common_chan_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPeCommonChanCfg(output, &(value->ail_pe_common_chan_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  enet_hdr_sel = %d\n", value->enet_hdr_sel);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCommonChanCfg (FILE *output, Iqn2Fl_AilPeCommonChanCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCommonChanCfg  */\n");
  fprintf(output, "  crc_en = %d\n", value->crc_en);
  fprintf(output, "  crc_hdr = %d\n", value->crc_hdr);
  fprintf(output, "  crc_type = %d\n", value->crc_type);
  fprintf(output, "  ethernet = %d\n", value->ethernet);
  fprintf(output, "  rt_ctl = %d\n", value->rt_ctl);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCpriCw (FILE *output, Iqn2Fl_AilPeCpriCw *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCpriCw  */\n");
  fprintf(output, "  ail_pe_cpri_cw_chan_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail_pe_cpri_cw_chan_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPeCpriCwChanCfg(output, &(value->ail_pe_cpri_cw_chan_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_pe_cpri_cw_fast_eth_4b5b[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail_pe_cpri_cw_fast_eth_4b5b[%d] = ", i);
    dump_Iqn2Fl_AilPeCpriCwFastEth4b5b(output, &(value->ail_pe_cpri_cw_fast_eth_4b5b[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_pe_cpri_cw_lut[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ail_pe_cpri_cw_lut[%d] = ", i);
    dump_Iqn2Fl_AilPeCpriCwLut(output, &(value->ail_pe_cpri_cw_lut[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  crc8_poly = %d\n", value->crc8_poly);
  fprintf(output, "  hf_en_part0 = %d\n", value->hf_en_part0);
  fprintf(output, "  hf_en_part1 = %d\n", value->hf_en_part1);
  fprintf(output, "  hf_en_part2 = %d\n", value->hf_en_part2);
  fprintf(output, "  hf_en_part3 = %d\n", value->hf_en_part3);
  fprintf(output, "  hf_en_part4 = %d\n", value->hf_en_part4);
  fprintf(output, "  null_char = %d\n", value->null_char);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCpriCwChanCfg (FILE *output, Iqn2Fl_AilPeCpriCwChanCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCpriCwChanCfg  */\n");
  fprintf(output, "  byte_en = %d\n", value->byte_en);
  fprintf(output, "  crc_init = %d\n", value->crc_init);
  fprintf(output, "  crc_rvrs = %d\n", value->crc_rvrs);
  fprintf(output, "  crc_sel = %d\n", value->crc_sel);
  fprintf(output, "  delin_sel = %d\n", value->delin_sel);
  fprintf(output, "  dlmt_imux = %d\n", value->dlmt_imux);
  fprintf(output, "  dlmt_omux = %d\n", value->dlmt_omux);
  fprintf(output, "  hf_lut_en = %d\n", value->hf_lut_en);
  fprintf(output, "  imux = %d\n", value->imux);
  fprintf(output, "  rt_ctl = %d\n", value->rt_ctl);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCpriCwFastEth4b5b (FILE *output, Iqn2Fl_AilPeCpriCwFastEth4b5b *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCpriCwFastEth4b5b  */\n");
  fprintf(output, "  bit_order = %d\n", value->bit_order);
  fprintf(output, "  hdr = %d\n", value->hdr);
  fprintf(output, "  hdr_preamble = %d\n", value->hdr_preamble);
  fprintf(output, "  hdr_sop = %d\n", value->hdr_sop);
  fprintf(output, "  ssd_order = %d\n", value->ssd_order);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeCpriCwLut (FILE *output, Iqn2Fl_AilPeCpriCwLut *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeCpriCwLut  */\n");
  fprintf(output, "  cw_chan = %d\n", value->cw_chan);
  fprintf(output, "  cw_en = %d\n", value->cw_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeObsaiHdrLut (FILE *output, Iqn2Fl_AilPeObsaiHdrLut *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeObsaiHdrLut  */\n");
  fprintf(output, "  adr = %d\n", value->adr);
  fprintf(output, "  ps_insert = %d\n", value->ps_insert);
  fprintf(output, "  ts_adr = %d\n", value->ts_adr);
  fprintf(output, "  ts_frmt = %d\n", value->ts_frmt);
  fprintf(output, "  ts_mask = %d\n", value->ts_mask);
  fprintf(output, "  typ = %d\n", value->typ);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPeSetup (FILE *output, Iqn2Fl_AilPeSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPeSetup  */\n");
    fprintf(output, "  ail_pe_common = ");
    dump_Iqn2Fl_AilPeCommon(output, &(value->ail_pe_common));
    fprintf(output, "  ail_pe_cpri_cw = ");
    dump_Iqn2Fl_AilPeCpriCw(output, &(value->ail_pe_cpri_cw));
  fprintf(output, "  ail_pe_obsai_hdr_lut[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_pe_obsai_hdr_lut[%d] = ", i);
    dump_Iqn2Fl_AilPeObsaiHdrLut(output, &(value->ail_pe_obsai_hdr_lut[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyCiCoLutCfg (FILE *output, Iqn2Fl_AilPhyCiCoLutCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyCiCoLutCfg  */\n");
  fprintf(output, "  ail_phy_ci_luta_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_phy_ci_luta_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPhyLutParams(output, &(value->ail_phy_ci_luta_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_phy_ci_lutb_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_phy_ci_lutb_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPhyLutParams(output, &(value->ail_phy_ci_lutb_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_phy_co_luta_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_phy_co_luta_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPhyLutParams(output, &(value->ail_phy_co_luta_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_phy_co_lutb_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_phy_co_lutb_cfg[%d] = ", i);
    dump_Iqn2Fl_AilPhyLutParams(output, &(value->ail_phy_co_lutb_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  phy_ci_lut_cfg_sel = %d\n", value->phy_ci_lut_cfg_sel);
  fprintf(output, "  phy_co_lut_cfg_sel = %d\n", value->phy_co_lut_cfg_sel);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyGlbCfg (FILE *output, Iqn2Fl_AilPhyGlbCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyGlbCfg  */\n");
  fprintf(output, "  link_rate = %d\n", value->link_rate);
  fprintf(output, "  obsai_cpri = %d\n", value->obsai_cpri);
  fprintf(output, "  short_frm_en = %d\n", value->short_frm_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyLutParams (FILE *output, Iqn2Fl_AilPhyLutParams *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyLutParams  */\n");
  fprintf(output, "  smpl_count = %d\n", value->smpl_count);
  fprintf(output, "  smpl_last = %d\n", value->smpl_last);
  fprintf(output, "  smpl_offset = %d\n", value->smpl_offset);
  fprintf(output, "  smpl_type = %d\n", value->smpl_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyLutSetup (FILE *output, Iqn2Fl_AilPhyLutSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyLutSetup  */\n");
  fprintf(output, "  index = %d\n", value->index);
  fprintf(output, "  smpl_count = %d\n", value->smpl_count);
  fprintf(output, "  smpl_last = %d\n", value->smpl_last);
  fprintf(output, "  smpl_offset = %d\n", value->smpl_offset);
  fprintf(output, "  smpl_type = %d\n", value->smpl_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyRmCfg (FILE *output, Iqn2Fl_AilPhyRmCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyRmCfg  */\n");
  fprintf(output, "  phy_rm_cfg_data_trc_sel = %d\n", value->phy_rm_cfg_data_trc_sel);
  fprintf(output, "  phy_rm_cfg_rst_en = %d\n", value->phy_rm_cfg_rst_en);
  fprintf(output, "  phy_rm_cfg_rx_en = %d\n", value->phy_rm_cfg_rx_en);
    fprintf(output, "  phy_rm_clk_det_cfg = ");
    dump_Iqn2Fl_AilPhyRmClkDetCfg(output, &(value->phy_rm_clk_det_cfg));
    fprintf(output, "  phy_rm_dp_cfg = ");
    dump_Iqn2Fl_AilPhyRmDpCfg(output, &(value->phy_rm_dp_cfg));
  fprintf(output, "  phy_rm_dscr_ctrl_cfg_scr_en = %d\n", value->phy_rm_dscr_ctrl_cfg_scr_en);
  fprintf(output, "  phy_rm_fsm_sync_cfg_frame_sync_t = %d\n", value->phy_rm_fsm_sync_cfg_frame_sync_t);
  fprintf(output, "  phy_rm_fsm_sync_cfg_sync_t = %d\n", value->phy_rm_fsm_sync_cfg_sync_t);
  fprintf(output, "  phy_rm_fsm_unsync_cfg_frame_unsync_t = %d\n", value->phy_rm_fsm_unsync_cfg_frame_unsync_t);
  fprintf(output, "  phy_rm_fsm_unsync_cfg_unsync_t = %d\n", value->phy_rm_fsm_unsync_cfg_unsync_t);
  fprintf(output, "  phy_rm_lcv_ctrl_cfg_en = %d\n", value->phy_rm_lcv_ctrl_cfg_en);
  fprintf(output, "  phy_rm_lcv_ctrl_cfg_los_det_thold = %d\n", value->phy_rm_lcv_ctrl_cfg_los_det_thold);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyRmClkDetCfg (FILE *output, Iqn2Fl_AilPhyRmClkDetCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyRmClkDetCfg  */\n");
  fprintf(output, "  cq_en = %d\n", value->cq_en);
  fprintf(output, "  mon_wrap = %d\n", value->mon_wrap);
  fprintf(output, "  wd_en = %d\n", value->wd_en);
  fprintf(output, "  wd_wrap = %d\n", value->wd_wrap);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyRmDpCfg (FILE *output, Iqn2Fl_AilPhyRmDpCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyRmDpCfg  */\n");
  fprintf(output, "  error_suppress = %d\n", value->error_suppress);
  fprintf(output, "  force_rx_state = %d\n", value->force_rx_state);
  fprintf(output, "  lcv_unsync_en = %d\n", value->lcv_unsync_en);
  fprintf(output, "  sd_auto_align_en = %d\n", value->sd_auto_align_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyRtCfg (FILE *output, Iqn2Fl_AilPhyRtCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyRtCfg  */\n");
  fprintf(output, "  bf_delay = %d\n", value->bf_delay);
  fprintf(output, "  ci_link = %d\n", value->ci_link);
  fprintf(output, "  config = %d\n", value->config);
  fprintf(output, "  em_en = %d\n", value->em_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhySetup (FILE *output, Iqn2Fl_AilPhySetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhySetup  */\n");
    fprintf(output, "  ail_phy_ci_co_lut_cfg = ");
    dump_Iqn2Fl_AilPhyCiCoLutCfg(output, &(value->ail_phy_ci_co_lut_cfg));
    fprintf(output, "  ail_phy_glb_cfg = ");
    dump_Iqn2Fl_AilPhyGlbCfg(output, &(value->ail_phy_glb_cfg));
    fprintf(output, "  ail_phy_rm_regs = ");
    dump_Iqn2Fl_AilPhyRmCfg(output, &(value->ail_phy_rm_regs));
    fprintf(output, "  ail_phy_rt_cfg = ");
    dump_Iqn2Fl_AilPhyRtCfg(output, &(value->ail_phy_rt_cfg));
    fprintf(output, "  ail_phy_tm_regs = ");
    dump_Iqn2Fl_AilPhyTmCfg(output, &(value->ail_phy_tm_regs));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyTmCfg (FILE *output, Iqn2Fl_AilPhyTmCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyTmCfg  */\n");
    fprintf(output, "  ail_phy_tm_l1_inb_cfg = ");
    dump_Iqn2Fl_AilPhyTmL1InbCfg(output, &(value->ail_phy_tm_l1_inb_cfg));
    fprintf(output, "  ail_phy_tm_l1_inb_en_cfg = ");
    dump_Iqn2Fl_AilPhyTmL1InbEnCfg(output, &(value->ail_phy_tm_l1_inb_en_cfg));
  fprintf(output, "  phy_tm_cfg_en = %d\n", value->phy_tm_cfg_en);
  fprintf(output, "  phy_tm_cpri_portid_a_cfg_z116_0 = %d\n", value->phy_tm_cpri_portid_a_cfg_z116_0);
  fprintf(output, "  phy_tm_cpri_portid_a_cfg_z116_1 = %d\n", value->phy_tm_cpri_portid_a_cfg_z116_1);
  fprintf(output, "  phy_tm_cpri_portid_a_cfg_z52_0 = %d\n", value->phy_tm_cpri_portid_a_cfg_z52_0);
  fprintf(output, "  phy_tm_cpri_portid_a_cfg_z52_1 = %d\n", value->phy_tm_cpri_portid_a_cfg_z52_1);
  fprintf(output, "  phy_tm_cpri_portid_b_cfg_z180_0 = %d\n", value->phy_tm_cpri_portid_b_cfg_z180_0);
  fprintf(output, "  phy_tm_cpri_portid_b_cfg_z180_1 = %d\n", value->phy_tm_cpri_portid_b_cfg_z180_1);
  fprintf(output, "  phy_tm_cpri_portid_b_cfg_z244_0 = %d\n", value->phy_tm_cpri_portid_b_cfg_z244_0);
  fprintf(output, "  phy_tm_cpri_portid_b_cfg_z244_1 = %d\n", value->phy_tm_cpri_portid_b_cfg_z244_1);
  fprintf(output, "  phy_tm_cpri_ptrp_cfg_ptr_p = %d\n", value->phy_tm_cpri_ptrp_cfg_ptr_p);
  fprintf(output, "  phy_tm_cpri_scr_ctrl_cfg_seed_value = %d\n", value->phy_tm_cpri_scr_ctrl_cfg_seed_value);
  fprintf(output, "  phy_tm_cpri_startup_cfg_startup = %d\n", value->phy_tm_cpri_startup_cfg_startup);
  fprintf(output, "  phy_tm_cpri_version_cfg_prot_vers = %d\n", value->phy_tm_cpri_version_cfg_prot_vers);
  fprintf(output, "  phy_tm_ctrl_cfg_flush = %d\n", value->phy_tm_ctrl_cfg_flush);
  fprintf(output, "  phy_tm_ctrl_cfg_idle = %d\n", value->phy_tm_ctrl_cfg_idle);
  fprintf(output, "  phy_tm_ctrl_cfg_los_en = %d\n", value->phy_tm_ctrl_cfg_los_en);
  fprintf(output, "  phy_tm_ctrl_cfg_resync = %d\n", value->phy_tm_ctrl_cfg_resync);
  fprintf(output, "  phy_tm_loferr_sel_cfg_link_loferr = %d\n", value->phy_tm_loferr_sel_cfg_link_loferr);
  fprintf(output, "  phy_tm_lofrx_sel_cfg_link_lofrx = %d\n", value->phy_tm_lofrx_sel_cfg_link_lofrx);
  fprintf(output, "  phy_tm_loserr_sel_cfg_link_loserr = %d\n", value->phy_tm_loserr_sel_cfg_link_loserr);
  fprintf(output, "  phy_tm_losrx_sel_cfg_link_losrx = %d\n", value->phy_tm_losrx_sel_cfg_link_losrx);
  fprintf(output, "  phy_tm_rairx_sel_cfg_link_rairx = %d\n", value->phy_tm_rairx_sel_cfg_link_rairx);
  fprintf(output, "  phy_tm_rstrx_sel_cfg_link_rstrx = %d\n", value->phy_tm_rstrx_sel_cfg_link_rstrx);
  fprintf(output, "  phy_tm_scr_ctrl_cfg_scr_en = %d\n", value->phy_tm_scr_ctrl_cfg_scr_en);
  fprintf(output, "  phy_tm_scr_ctrl_cfg_seed_value = %d\n", value->phy_tm_scr_ctrl_cfg_seed_value);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyTmL1InbCfg (FILE *output, Iqn2Fl_AilPhyTmL1InbCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyTmL1InbCfg  */\n");
  fprintf(output, "  l1_inband_lof = %d\n", value->l1_inband_lof);
  fprintf(output, "  l1_inband_los = %d\n", value->l1_inband_los);
  fprintf(output, "  l1_inband_rai = %d\n", value->l1_inband_rai);
  fprintf(output, "  l1_inband_rst = %d\n", value->l1_inband_rst);
  fprintf(output, "  l1_inband_sdi = %d\n", value->l1_inband_sdi);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilPhyTmL1InbEnCfg (FILE *output, Iqn2Fl_AilPhyTmL1InbEnCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilPhyTmL1InbEnCfg  */\n");
  fprintf(output, "  lof_loferr_en = %d\n", value->lof_loferr_en);
  fprintf(output, "  lof_lofrx_en = %d\n", value->lof_lofrx_en);
  fprintf(output, "  los_loserr_en = %d\n", value->los_loserr_en);
  fprintf(output, "  los_losrx_en = %d\n", value->los_losrx_en);
  fprintf(output, "  rai_loferr_en = %d\n", value->rai_loferr_en);
  fprintf(output, "  rai_lofrx_en = %d\n", value->rai_lofrx_en);
  fprintf(output, "  rai_loserr_en = %d\n", value->rai_loserr_en);
  fprintf(output, "  rai_losrx_en = %d\n", value->rai_losrx_en);
  fprintf(output, "  rai_rairx_en = %d\n", value->rai_rairx_en);
  fprintf(output, "  sdi_rstrx_en = %d\n", value->sdi_rstrx_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSetup (FILE *output, Iqn2Fl_AilSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSetup  */\n");
  fprintf(output, "  ailInstNum = %s\n", dump_Iqn2Fl_AilInstance(value->ailInstNum));
  if (value->pAilEgrSetup != NULL) {
    fprintf(output, "  pAilEgrSetup = ");
    dump_Iqn2Fl_AilEgrSetup(output, value->pAilEgrSetup);
  } else {
    fprintf(output, "  pAilEgrSetup = NULL\n");
  }
  if (value->pAilIgrSetup != NULL) {
    fprintf(output, "  pAilIgrSetup = ");
    dump_Iqn2Fl_AilIgrSetup(output, value->pAilIgrSetup);
  } else {
    fprintf(output, "  pAilIgrSetup = NULL\n");
  }
  if (value->pAilPdSetup != NULL) {
    fprintf(output, "  pAilPdSetup = ");
    dump_Iqn2Fl_AilPdSetup(output, value->pAilPdSetup);
  } else {
    fprintf(output, "  pAilPdSetup = NULL\n");
  }
  if (value->pAilPeSetup != NULL) {
    fprintf(output, "  pAilPeSetup = ");
    dump_Iqn2Fl_AilPeSetup(output, value->pAilPeSetup);
  } else {
    fprintf(output, "  pAilPeSetup = NULL\n");
  }
  if (value->pAilPhySetup != NULL) {
    fprintf(output, "  pAilPhySetup = ");
    dump_Iqn2Fl_AilPhySetup(output, value->pAilPhySetup);
  } else {
    fprintf(output, "  pAilPhySetup = NULL\n");
  }
  if (value->pAilUatSetup != NULL) {
    fprintf(output, "  pAilUatSetup = ");
    dump_Iqn2Fl_AilUatSetup(output, value->pAilUatSetup);
  } else {
    fprintf(output, "  pAilUatSetup = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqCpriRadstdCfg (FILE *output, Iqn2Fl_AilSiIqCpriRadstdCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqCpriRadstdCfg  */\n");
  fprintf(output, "  radstd1_cfg_bfrm_offset[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd1_cfg_bfrm_offset[%d] = %d\n", i, value->radstd1_cfg_bfrm_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  radstd1_cfg_hfrm_offset[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd1_cfg_hfrm_offset[%d] = %d\n", i, value->radstd1_cfg_hfrm_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  radstd2_cfg_bfrm_num[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd2_cfg_bfrm_num[%d] = %d\n", i, value->radstd2_cfg_bfrm_num[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  radstd_cfg_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    radstd_cfg_en[%d] = %d\n", i, value->radstd_cfg_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqESchCpriBubFsmCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriBubFsmCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqESchCpriBubFsmCfg  */\n");
  fprintf(output, "  gap_frac[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    gap_frac[%d] = %d\n", i, value->gap_frac[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  gap_int[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    gap_int[%d] = %d\n", i, value->gap_int[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  knc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    knc[%d] = %d\n", i, value->knc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqESchCpriContCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriContCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqESchCpriContCfg  */\n");
  fprintf(output, "  lut_en[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    lut_en[%d] = %d\n", i, value->lut_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  lut_grp[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    lut_grp[%d] = %d\n", i, value->lut_grp[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqESchCpriTdmFsmCfg (FILE *output, Iqn2Fl_AilSiIqESchCpriTdmFsmCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqESchCpriTdmFsmCfg  */\n");
  fprintf(output, "  lutstrt[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    lutstrt[%d] = %d\n", i, value->lutstrt[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ncont[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ncont[%d] = %d\n", i, value->ncont[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqEfeCfgGrp (FILE *output, Iqn2Fl_AilSiIqEfeCfgGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqEfeCfgGrp  */\n");
  fprintf(output, "  ail_si_iq_efe_chan_config[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail_si_iq_efe_chan_config[%d] = ", i);
    dump_Iqn2Fl_AilIqEfeChanCfg(output, &(value->ail_si_iq_efe_chan_config[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  lpbk_en = %d\n", value->lpbk_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqEfeRadStdGrp (FILE *output, Iqn2Fl_AilSiIqEfeRadStdGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqEfeRadStdGrp  */\n");
  fprintf(output, "  ail_iq_efe_frm_tc_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_iq_efe_frm_tc_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqFrmTcCfg(output, &(value->ail_iq_efe_frm_tc_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_rad_std_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_iq_efe_rad_std_cfg[%d] = ", i);
    dump_Iqn2Fl_AilIqEfeRadStdCfg(output, &(value->ail_iq_efe_rad_std_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg0[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg0[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg0[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg1[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg1[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg2[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg2[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg3[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg3[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg4[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg4[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg5[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg5[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg5[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg6[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg6[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg6[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_iq_efe_tdd_en_cfg7[5] = [\n");
  for(i=0; i<5; i++) {
    fprintf(output, "    ail_iq_efe_tdd_en_cfg7[%d] = %d\n", i, value->ail_iq_efe_tdd_en_cfg7[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqEgrObsaiDbmRule (FILE *output, Iqn2Fl_AilSiIqEgrObsaiDbmRule *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqEgrObsaiDbmRule  */\n");
  fprintf(output, "  dbm_1mult = %d\n", value->dbm_1mult);
  fprintf(output, "  dbm_1size = %d\n", value->dbm_1size);
  fprintf(output, "  dbm_2size = %d\n", value->dbm_2size);
  fprintf(output, "  dbm_en = %d\n", value->dbm_en);
  fprintf(output, "  dbm_lutstrt = %d\n", value->dbm_lutstrt);
  fprintf(output, "  dbm_radstd = %d\n", value->dbm_radstd);
  fprintf(output, "  dbm_x = %d\n", value->dbm_x);
  fprintf(output, "  dbm_xbubble = %d\n", value->dbm_xbubble);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqEgrSchCpri (FILE *output, Iqn2Fl_AilSiIqEgrSchCpri *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqEgrSchCpri  */\n");
    fprintf(output, "  ail_iq_pe_cpri_bub_fsm_cfg = ");
    dump_Iqn2Fl_AilSiIqESchCpriBubFsmCfg(output, &(value->ail_iq_pe_cpri_bub_fsm_cfg));
    fprintf(output, "  ail_iq_pe_cpri_cont_cfg = ");
    dump_Iqn2Fl_AilSiIqESchCpriContCfg(output, &(value->ail_iq_pe_cpri_cont_cfg));
    fprintf(output, "  ail_iq_pe_cpri_radstd_cfg = ");
    dump_Iqn2Fl_AilSiIqCpriRadstdCfg(output, &(value->ail_iq_pe_cpri_radstd_cfg));
    fprintf(output, "  ail_iq_pe_cpri_tdm_fsm_cfg = ");
    dump_Iqn2Fl_AilSiIqESchCpriTdmFsmCfg(output, &(value->ail_iq_pe_cpri_tdm_fsm_cfg));
  fprintf(output, "  axc_cont_tc = %d\n", value->axc_cont_tc);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilSiIqEgrTdmLutRam (FILE *output, Iqn2Fl_AilSiIqEgrTdmLutRam *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilSiIqEgrTdmLutRam  */\n");
  fprintf(output, "  axc = %d\n", value->axc);
  fprintf(output, "  en = %d\n", value->en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_AilUatSetup (FILE *output, Iqn2Fl_AilUatSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_AilUatSetup  */\n");
  fprintf(output, "  ail_uat_bcn_offset_cfg_val = %d\n", value->ail_uat_bcn_offset_cfg_val);
  fprintf(output, "  ail_uat_bcn_tc_cfg_val = %d\n", value->ail_uat_bcn_tc_cfg_val);
  fprintf(output, "  ail_uat_egr_radt_offset_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_uat_egr_radt_offset_cfg_val[%d] = %d\n", i, value->ail_uat_egr_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_egr_radt_tc_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_uat_egr_radt_tc_cfg_val[%d] = %d\n", i, value->ail_uat_egr_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_evt_clk_cnt_tc_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    ail_uat_evt_clk_cnt_tc_cfg_val[%d] = %d\n", i, value->ail_uat_evt_clk_cnt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_evt_radt_cmp_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    ail_uat_evt_radt_cmp_cfg_val[%d] = %d\n", i, value->ail_uat_evt_radt_cmp_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_ing_radt_offset_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_uat_ing_radt_offset_cfg_val[%d] = %d\n", i, value->ail_uat_ing_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_ing_radt_tc_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ail_uat_ing_radt_tc_cfg_val[%d] = %d\n", i, value->ail_uat_ing_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail_uat_pe_fb_cfg_val = %d\n", value->ail_uat_pe_fb_cfg_val);
  fprintf(output, "  ail_uat_pimax_cfg_val = %d\n", value->ail_uat_pimax_cfg_val);
  fprintf(output, "  ail_uat_pimin_cfg_val = %d\n", value->ail_uat_pimin_cfg_val);
  fprintf(output, "  ail_uat_rt_fb_cfg_val = %d\n", value->ail_uat_rt_fb_cfg_val);
  fprintf(output, "  ail_uat_tm_bfn_cfg_wr_val = %d\n", value->ail_uat_tm_bfn_cfg_wr_val);
  fprintf(output, "  ail_uat_tm_fb_cfg_val = %d\n", value->ail_uat_tm_fb_cfg_val);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_At2Events24Array (FILE *output, Iqn2Fl_At2Events24Array *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_At2Events24Array  */\n");
  fprintf(output, "  at2_events_24array_mask_lsbs_cfg_val[24] = [\n");
  for(i=0; i<24; i++) {
    fprintf(output, "    at2_events_24array_mask_lsbs_cfg_val[%d] = %d\n", i, value->at2_events_24array_mask_lsbs_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_events_24array_mask_msbs_cfg_val[24] = [\n");
  for(i=0; i<24; i++) {
    fprintf(output, "    at2_events_24array_mask_msbs_cfg_val[%d] = %d\n", i, value->at2_events_24array_mask_msbs_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_events_24array_mod_tc_cfg_val[24] = [\n");
  for(i=0; i<24; i++) {
    fprintf(output, "    at2_events_24array_mod_tc_cfg_val[%d] = %d\n", i, value->at2_events_24array_mod_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_events_24array_offset_cfg_evt_strb_sel[24] = [\n");
  for(i=0; i<24; i++) {
    fprintf(output, "    at2_events_24array_offset_cfg_evt_strb_sel[%d] = %d\n", i, value->at2_events_24array_offset_cfg_evt_strb_sel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_events_24array_offset_cfg_val[24] = [\n");
  for(i=0; i<24; i++) {
    fprintf(output, "    at2_events_24array_offset_cfg_val[%d] = %d\n", i, value->at2_events_24array_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_At2RadtStatus (FILE *output, Iqn2Fl_At2RadtStatus *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_At2RadtStatus  */\n");
  fprintf(output, "  radt_chip = %d\n", value->radt_chip);
  fprintf(output, "  radt_frm = %d\n", value->radt_frm);
  fprintf(output, "  radt_frm_val_lsb = %d\n", value->radt_frm_val_lsb);
  fprintf(output, "  radt_frm_val_msb = %d\n", value->radt_frm_val_msb);
  fprintf(output, "  radt_num = %d\n", value->radt_num);
  fprintf(output, "  radt_sampcnt_val = %d\n", value->radt_sampcnt_val);
  fprintf(output, "  radt_slot = %d\n", value->radt_slot);
  fprintf(output, "  radt_symcnt_val = %d\n", value->radt_symcnt_val);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_At2Setup (FILE *output, Iqn2Fl_At2Setup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_At2Setup  */\n");
  fprintf(output, "  at2_bcn_clkcnt_tc_cfg_val = %d\n", value->at2_bcn_clkcnt_tc_cfg_val);
  fprintf(output, "  at2_bcn_frame_tc_lsb_cfg_val = %d\n", value->at2_bcn_frame_tc_lsb_cfg_val);
  fprintf(output, "  at2_bcn_frame_tc_msb_cfg_val = %d\n", value->at2_bcn_frame_tc_msb_cfg_val);
  fprintf(output, "  at2_bcn_frm_init_lsbs_cfg_wr_val = %d\n", value->at2_bcn_frm_init_lsbs_cfg_wr_val);
  fprintf(output, "  at2_bcn_frm_init_msbs_cfg_wr_val = %d\n", value->at2_bcn_frm_init_msbs_cfg_wr_val);
  fprintf(output, "  at2_bcn_offset_cfg_val = %d\n", value->at2_bcn_offset_cfg_val);
  fprintf(output, "  at2_bcn_slvsel_cfg_val = %d\n", value->at2_bcn_slvsel_cfg_val);
    fprintf(output, "  at2_events_24array = ");
    dump_Iqn2Fl_At2Events24Array(output, &(value->at2_events_24array));
  fprintf(output, "  at2_evt_enable_cfg = %d\n", value->at2_evt_enable_cfg);
  fprintf(output, "  at2_gsm_tcount_init_cfg_t1 = %d\n", value->at2_gsm_tcount_init_cfg_t1);
  fprintf(output, "  at2_gsm_tcount_init_cfg_t2 = %d\n", value->at2_gsm_tcount_init_cfg_t2);
  fprintf(output, "  at2_gsm_tcount_init_cfg_t3 = %d\n", value->at2_gsm_tcount_init_cfg_t3);
  fprintf(output, "  at2_radt_0_cfg_clkcnt_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_0_cfg_clkcnt_tc[%d] = %d\n", i, value->at2_radt_0_cfg_clkcnt_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_0_cfg_lutindex_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_0_cfg_lutindex_tc[%d] = %d\n", i, value->at2_radt_0_cfg_lutindex_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_0_cfg_symb_tc[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_0_cfg_symb_tc[%d] = %d\n", i, value->at2_radt_0_cfg_symb_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_1_cfg_frm_tc_lsb[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_1_cfg_frm_tc_lsb[%d] = %d\n", i, value->at2_radt_1_cfg_frm_tc_lsb[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_2_cfg_frm_tc_msb[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_2_cfg_frm_tc_msb[%d] = %d\n", i, value->at2_radt_2_cfg_frm_tc_msb[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_3_cfg_lut_index_strt[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_3_cfg_lut_index_strt[%d] = %d\n", i, value->at2_radt_3_cfg_lut_index_strt[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_4_cfg_bcn_sync_cmp[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_4_cfg_bcn_sync_cmp[%d] = %d\n", i, value->at2_radt_4_cfg_bcn_sync_cmp[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_init_1_cfg_frm_init_lsb[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_init_1_cfg_frm_init_lsb[%d] = %d\n", i, value->at2_radt_init_1_cfg_frm_init_lsb[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_init_2_cfg_frm_init_msb[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    at2_radt_init_2_cfg_frm_init_msb[%d] = %d\n", i, value->at2_radt_init_2_cfg_frm_init_msb[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_radt_sym_lut_ram_cfg_symbcnt_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    at2_radt_sym_lut_ram_cfg_symbcnt_tc[%d] = %d\n", i, value->at2_radt_sym_lut_ram_cfg_symbcnt_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  at2_rp1_ctrl_cfg_crc_flip = %d\n", value->at2_rp1_ctrl_cfg_crc_flip);
  fprintf(output, "  at2_rp1_ctrl_cfg_crc_init_ones = %d\n", value->at2_rp1_ctrl_cfg_crc_init_ones);
  fprintf(output, "  at2_rp1_ctrl_cfg_crc_invert = %d\n", value->at2_rp1_ctrl_cfg_crc_invert);
  fprintf(output, "  at2_rp1_ctrl_cfg_crc_use = %d\n", value->at2_rp1_ctrl_cfg_crc_use);
  fprintf(output, "  at2_start_timer_enables_cfg_radt_en = %d\n", value->at2_start_timer_enables_cfg_radt_en);
  fprintf(output, "  at2_start_timer_enables_cfg_run_bcn = %d\n", value->at2_start_timer_enables_cfg_run_bcn);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_BaseAddress (FILE *output, Iqn2Fl_BaseAddress *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_BaseAddress  */\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2CoreDmaCfg0 (FILE *output, Iqn2Fl_Dio2CoreDmaCfg0 *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2CoreDmaCfg0  */\n");
  fprintf(output, "  dma_brst_ln = %d\n", value->dma_brst_ln);
  fprintf(output, "  dma_eng_en = %d\n", value->dma_eng_en);
  fprintf(output, "  dma_num_blks = %d\n", value->dma_num_blks);
  fprintf(output, "  dma_num_qwd = %d\n", value->dma_num_qwd);
  fprintf(output, "  rsa_cnvrt_en = %d\n", value->rsa_cnvrt_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2CoreEgressSetup (FILE *output, Iqn2Fl_Dio2CoreEgressSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2CoreEgressSetup  */\n");
  fprintf(output, "  bcn_table_sel[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    bcn_table_sel[%d] = %d\n", i, value->bcn_table_sel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_cfg0[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_cfg0[%d] = ", i);
    dump_Iqn2Fl_Dio2CoreDmaCfg0(output, &(value->dma_cfg0[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_cfg1_dma_blk_addr_stride[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_cfg1_dma_blk_addr_stride[%d] = %d\n", i, value->dma_cfg1_dma_blk_addr_stride[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_num_axc[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_num_axc[%d] = %d\n", i, value->dma_num_axc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2CoreIngressSetup (FILE *output, Iqn2Fl_Dio2CoreIngressSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2CoreIngressSetup  */\n");
  fprintf(output, "  bcn_table_sel[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    bcn_table_sel[%d] = %d\n", i, value->bcn_table_sel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_cfg0[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_cfg0[%d] = ", i);
    dump_Iqn2Fl_Dio2CoreDmaCfg0(output, &(value->dma_cfg0[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_cfg1_dma_blk_addr_stride[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_cfg1_dma_blk_addr_stride[%d] = %d\n", i, value->dma_cfg1_dma_blk_addr_stride[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_num_axc[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dma_num_axc[%d] = %d\n", i, value->dma_num_axc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2DbcntxRamMmr (FILE *output, Iqn2Fl_Dio2DbcntxRamMmr *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2DbcntxRamMmr  */\n");
  fprintf(output, "  ch_en[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    ch_en[%d] = %d\n", i, value->ch_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ch_id[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    ch_id[%d] = %d\n", i, value->ch_id[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dma_vbus_base_addr_axc[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    dma_vbus_base_addr_axc[%d] = %d\n", i, value->dma_vbus_base_addr_axc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2DbcntxRamMmrCfg (FILE *output, Iqn2Fl_Dio2DbcntxRamMmrCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2DbcntxRamMmrCfg  */\n");
    fprintf(output, "  cfg_dbcn_cnt = ");
    dump_Iqn2Fl_Dio2DbcntxRamMmr(output, &(value->cfg_dbcn_cnt));
  fprintf(output, "  engine_idx = %d\n", value->engine_idx);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2Dt (FILE *output, Iqn2Fl_Dio2Dt *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2Dt  */\n");
  fprintf(output, "  dt_cfg0_dt_en = %d\n", value->dt_cfg0_dt_en);
  fprintf(output, "  dt_cfg0_dt_endian_sel = %d\n", value->dt_cfg0_dt_endian_sel);
  fprintf(output, "  dt_cfg0_dt_start_mode = %d\n", value->dt_cfg0_dt_start_mode);
  fprintf(output, "  dt_cfg0_dt_stop_mode = %d\n", value->dt_cfg0_dt_stop_mode);
  fprintf(output, "  dt_cfg1_dt_dma_base_addr = %d\n", value->dt_cfg1_dt_dma_base_addr);
  fprintf(output, "  dt_cfg2_dt_dma_wrap = %d\n", value->dt_cfg2_dt_dma_wrap);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2FrmTcCfg (FILE *output, Iqn2Fl_Dio2FrmTcCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2FrmTcCfg  */\n");
  fprintf(output, "  index_sc = %d\n", value->index_sc);
  fprintf(output, "  index_tc = %d\n", value->index_tc);
  fprintf(output, "  sym_tc = %d\n", value->sym_tc);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2IqIdcChCfgGrp (FILE *output, Iqn2Fl_Dio2IqIdcChCfgGrp *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2IqIdcChCfgGrp  */\n");
  fprintf(output, "  chan_frc_off = %d\n", value->chan_frc_off);
  fprintf(output, "  dat_swap = %d\n", value->dat_swap);
  fprintf(output, "  iq_order = %d\n", value->iq_order);
  fprintf(output, "  pkt_type = %d\n", value->pkt_type);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2IqIfeChanCfgGrpSetup (FILE *output, Iqn2Fl_Dio2IqIfeChanCfgGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2IqIfeChanCfgGrpSetup  */\n");
  fprintf(output, "  chan_axc_offset = %d\n", value->chan_axc_offset);
  fprintf(output, "  chan_en = %d\n", value->chan_en);
  fprintf(output, "  chan_radio_sel = %d\n", value->chan_radio_sel);
  fprintf(output, "  chan_tdd_frc_off = %d\n", value->chan_tdd_frc_off);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2IqIfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Dio2IqIfeRadioStdGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2IqIfeRadioStdGrpSetup  */\n");
  fprintf(output, "  ife_frm_tc_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ife_frm_tc_cfg[%d] = ", i);
    dump_Iqn2Fl_Dio2FrmTcCfg(output, &(value->ife_frm_tc_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ife_rad_std_cfg_tdd_lut_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    ife_rad_std_cfg_tdd_lut_en[%d] = %d\n", i, value->ife_rad_std_cfg_tdd_lut_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ife_tdd_en_cfg_tdd_en[8][5] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "  -- %d", i);
    for(j=0; j<5; j++) {
      fprintf(output, "    ife_tdd_en_cfg_tdd_en[%d][%d] = %d\n", i, j, value->ife_tdd_en_cfg_tdd_en[i][j]);
    }
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2Setup (FILE *output, Iqn2Fl_Dio2Setup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2Setup  */\n");
    fprintf(output, "  dio2_core_egress = ");
    dump_Iqn2Fl_Dio2CoreEgressSetup(output, &(value->dio2_core_egress));
    fprintf(output, "  dio2_core_ingress = ");
    dump_Iqn2Fl_Dio2CoreIngressSetup(output, &(value->dio2_core_ingress));
    fprintf(output, "  dio2_dt = ");
    dump_Iqn2Fl_Dio2Dt(output, &(value->dio2_dt));
  fprintf(output, "  dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[%d] = %d\n", i, value->dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_e_dbcntx_ram_mmr[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dio2_e_dbcntx_ram_mmr[%d] = ", i);
    dump_Iqn2Fl_Dio2DbcntxRamMmr(output, &(value->dio2_e_dbcntx_ram_mmr[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  dio2_efe_cfg_grp = ");
    dump_Iqn2Fl_Dio2SiIqEfeCfgGrpSetup(output, &(value->dio2_efe_cfg_grp));
    fprintf(output, "  dio2_efe_radio_std_grp = ");
    dump_Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup(output, &(value->dio2_efe_radio_std_grp));
  fprintf(output, "  dio2_global_cfg_rsa_big_endian = %d\n", value->dio2_global_cfg_rsa_big_endian);
  fprintf(output, "  dio2_global_cfg_vbusm_priority = %d\n", value->dio2_global_cfg_vbusm_priority);
  fprintf(output, "  dio2_i_axc_off_mmr_cfg_4samp_offset[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_i_axc_off_mmr_cfg_4samp_offset[%d] = %d\n", i, value->dio2_i_axc_off_mmr_cfg_4samp_offset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_i_dbcntx_ram_mmr[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    dio2_i_dbcntx_ram_mmr[%d] = ", i);
    dump_Iqn2Fl_Dio2DbcntxRamMmr(output, &(value->dio2_i_dbcntx_ram_mmr[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_idc_ch_cfg_grp[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_idc_ch_cfg_grp[%d] = ", i);
    dump_Iqn2Fl_Dio2IqIdcChCfgGrp(output, &(value->dio2_idc_ch_cfg_grp[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_ife_chan_cfg_grp[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_ife_chan_cfg_grp[%d] = ", i);
    dump_Iqn2Fl_Dio2IqIfeChanCfgGrpSetup(output, &(value->dio2_ife_chan_cfg_grp[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  dio2_ife_radio_std_grp = ");
    dump_Iqn2Fl_Dio2IqIfeRadioStdGrpSetup(output, &(value->dio2_ife_radio_std_grp));
  fprintf(output, "  dio2_iq_edc_cfg_psi_err_chk_disable = %d\n", value->dio2_iq_edc_cfg_psi_err_chk_disable);
  fprintf(output, "  dio2_iq_edc_ch_cfg_dat_swap[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_iq_edc_ch_cfg_dat_swap[%d] = %d\n", i, value->dio2_iq_edc_ch_cfg_dat_swap[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_iq_edc_ch_cfg_iq_order[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_iq_edc_ch_cfg_iq_order[%d] = %d\n", i, value->dio2_iq_edc_ch_cfg_iq_order[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_iq_idc_rate_ctl_cfg_rate = %d\n", value->dio2_iq_idc_rate_ctl_cfg_rate);
  fprintf(output, "  efe_chan_axc_offset_cfg[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    efe_chan_axc_offset_cfg[%d] = %d\n", i, value->efe_chan_axc_offset_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_chan_tdm_lut_cfg_chan_idx_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_chan_tdm_lut_cfg_chan_idx_cfg[%d] = %d\n", i, value->efe_chan_tdm_lut_cfg_chan_idx_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_chan_tdm_lut_cfg_chan_idx_en_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_chan_tdm_lut_cfg_chan_idx_en_cfg[%d] = %d\n", i, value->efe_chan_tdm_lut_cfg_chan_idx_en_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_frm_samp_tc_cfg[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    efe_frm_samp_tc_cfg[%d] = %d\n", i, value->efe_frm_samp_tc_cfg[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_rad_std_sch_cfg_tdm_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_rad_std_sch_cfg_tdm_en[%d] = %d\n", i, value->efe_rad_std_sch_cfg_tdm_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_rad_std_sch_cfg_tdm_len[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_rad_std_sch_cfg_tdm_len[%d] = %d\n", i, value->efe_rad_std_sch_cfg_tdm_len[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_rad_std_sch_cfg_tdm_start[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_rad_std_sch_cfg_tdm_start[%d] = %d\n", i, value->efe_rad_std_sch_cfg_tdm_start[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  idc_cfg_grp_fail_mark_only = %d\n", value->idc_cfg_grp_fail_mark_only);
  fprintf(output, "  idc_cfg_grp_frc_off_all = %d\n", value->idc_cfg_grp_frc_off_all);
  fprintf(output, "  ife_frm_samp_tc_cfg_samp_tc[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    ife_frm_samp_tc_cfg_samp_tc[%d] = %d\n", i, value->ife_frm_samp_tc_cfg_samp_tc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_dio_egr_radt_offset_cfg_val[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    uat_dio_egr_radt_offset_cfg_val[%d] = %d\n", i, value->uat_dio_egr_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_dio_egr_radt_tc_cfg_val[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    uat_dio_egr_radt_tc_cfg_val[%d] = %d\n", i, value->uat_dio_egr_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_dio_ing_radt_offset_cfg_val[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    uat_dio_ing_radt_offset_cfg_val[%d] = %d\n", i, value->uat_dio_ing_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_dio_ing_radt_tc_cfg_val[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    uat_dio_ing_radt_tc_cfg_val[%d] = %d\n", i, value->uat_dio_ing_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_egr_radt_offset_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_egr_radt_offset_cfg_val[%d] = %d\n", i, value->uat_egr_radt_offset_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_egr_radt_tc_cfg_val[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    uat_egr_radt_tc_cfg_val[%d] = %d\n", i, value->uat_egr_radt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_evt_clk_cnt_tc_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    uat_evt_clk_cnt_tc_cfg_val[%d] = %d\n", i, value->uat_evt_clk_cnt_tc_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  uat_evt_radt_cmp_cfg_val[22] = [\n");
  for(i=0; i<22; i++) {
    fprintf(output, "    uat_evt_radt_cmp_cfg_val[%d] = %d\n", i, value->uat_evt_radt_cmp_cfg_val[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2SiIqEfeCfgGrpSetup (FILE *output, Iqn2Fl_Dio2SiIqEfeCfgGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2SiIqEfeCfgGrpSetup  */\n");
  fprintf(output, "  chan_en[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    chan_en[%d] = %d\n", i, value->chan_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  chan_radio_sel[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    chan_radio_sel[%d] = %d\n", i, value->chan_radio_sel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  chan_tdd_frc_off[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    chan_tdd_frc_off[%d] = %d\n", i, value->chan_tdd_frc_off[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  loopback_en = %d\n", value->loopback_en);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup (FILE *output, Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup  */\n");
  fprintf(output, "  efe_frm_tc_cfg[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_frm_tc_cfg[%d] = ", i);
    dump_Iqn2Fl_Dio2FrmTcCfg(output, &(value->efe_frm_tc_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_rad_std_cfg_tdd_first_sym[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_rad_std_cfg_tdd_first_sym[%d] = %d\n", i, value->efe_rad_std_cfg_tdd_first_sym[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_rad_std_cfg_tdd_lut_en[8] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "    efe_rad_std_cfg_tdd_lut_en[%d] = %d\n", i, value->efe_rad_std_cfg_tdd_lut_en[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  efe_tdd_en_cfg_tdd_en[8][5] = [\n");
  for(i=0; i<8; i++) {
    fprintf(output, "  -- %d", i);
    for(j=0; j<5; j++) {
      fprintf(output, "    efe_tdd_en_cfg_tdd_en[%d][%d] = %d\n", i, j, value->efe_tdd_en_cfg_tdd_en[i][j]);
    }
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_HwControlMultiArgs (FILE *output, Iqn2Fl_HwControlMultiArgs *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_HwControlMultiArgs  */\n");
  fprintf(output, "  reg_index = %d\n", value->reg_index);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Iqs2Setup (FILE *output, Iqn2Fl_Iqs2Setup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Iqs2Setup  */\n");
    fprintf(output, "  iqs2_egress_chan_cfg = ");
    dump_Iqn2Fl_IqsEgressChanCfgSetup(output, &(value->iqs2_egress_chan_cfg));
    fprintf(output, "  iqs2_ingress_cfg = ");
    dump_Iqn2Fl_IqsIngressCfgSetup(output, &(value->iqs2_ingress_cfg));
    fprintf(output, "  iqs2_ingress_chan_cfg = ");
    dump_Iqn2Fl_IqsIngressChanCfgSetup(output, &(value->iqs2_ingress_chan_cfg));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_IqsEgrPktdmaCfgSetup (FILE *output, Iqn2Fl_IqsEgrPktdmaCfgSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_IqsEgrPktdmaCfgSetup  */\n");
  fprintf(output, "  arb_pri = %d\n", value->arb_pri);
  fprintf(output, "  chan = %d\n", value->chan);
  fprintf(output, "  dest = %d\n", value->dest);
  fprintf(output, "  psi_pri = %d\n", value->psi_pri);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_IqsEgressChanCfgSetup (FILE *output, Iqn2Fl_IqsEgressChanCfgSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_IqsEgressChanCfgSetup  */\n");
  fprintf(output, "  egr_dio2_cfg[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    egr_dio2_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsEgrPktdmaCfgSetup(output, &(value->egr_dio2_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  egr_pktdma_cfg[48] = [\n");
  for(i=0; i<48; i++) {
    fprintf(output, "    egr_pktdma_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsEgrPktdmaCfgSetup(output, &(value->egr_pktdma_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_IqsIngressCfgSetup (FILE *output, Iqn2Fl_IqsIngressCfgSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_IqsIngressCfgSetup  */\n");
  fprintf(output, "  aid2_axc_cfg_allow_pushback = %d\n", value->aid2_axc_cfg_allow_pushback);
  fprintf(output, "  aid2_axc_cfg_pri = %d\n", value->aid2_axc_cfg_pri);
  fprintf(output, "  aid2_ctl_cfg_allow_pushback = %d\n", value->aid2_ctl_cfg_allow_pushback);
  fprintf(output, "  aid2_ctl_cfg_pri = %d\n", value->aid2_ctl_cfg_pri);
  fprintf(output, "  ail0_axc_cfg_allow_pushback = %d\n", value->ail0_axc_cfg_allow_pushback);
  fprintf(output, "  ail0_axc_cfg_pri = %d\n", value->ail0_axc_cfg_pri);
  fprintf(output, "  ail0_ctl_cfg_allow_pushback = %d\n", value->ail0_ctl_cfg_allow_pushback);
  fprintf(output, "  ail0_ctl_cfg_pri = %d\n", value->ail0_ctl_cfg_pri);
  fprintf(output, "  ail1_axc_cfg_allow_pushback = %d\n", value->ail1_axc_cfg_allow_pushback);
  fprintf(output, "  ail1_axc_cfg_pri = %d\n", value->ail1_axc_cfg_pri);
  fprintf(output, "  ail1_ctl_cfg_allow_pushback = %d\n", value->ail1_ctl_cfg_allow_pushback);
  fprintf(output, "  ail1_ctl_cfg_pri = %d\n", value->ail1_ctl_cfg_pri);
  fprintf(output, "  ail2_axc_cfg_allow_pushback = %d\n", value->ail2_axc_cfg_allow_pushback);
  fprintf(output, "  ail2_axc_cfg_pri = %d\n", value->ail2_axc_cfg_pri);
  fprintf(output, "  ail2_ctl_cfg_allow_pushback = %d\n", value->ail2_ctl_cfg_allow_pushback);
  fprintf(output, "  ail2_ctl_cfg_pri = %d\n", value->ail2_ctl_cfg_pri);
  fprintf(output, "  ail3_axc_cfg_allow_pushback = %d\n", value->ail3_axc_cfg_allow_pushback);
  fprintf(output, "  ail3_axc_cfg_pri = %d\n", value->ail3_axc_cfg_pri);
  fprintf(output, "  ail3_ctl_cfg_allow_pushback = %d\n", value->ail3_ctl_cfg_allow_pushback);
  fprintf(output, "  ail3_ctl_cfg_pri = %d\n", value->ail3_ctl_cfg_pri);
  fprintf(output, "  pktdma_cfg_pb_sel = %d\n", value->pktdma_cfg_pb_sel);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_IqsIngressChanCfgSetup (FILE *output, Iqn2Fl_IqsIngressChanCfgSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_IqsIngressChanCfgSetup  */\n");
  fprintf(output, "  aid2_axc_lut_cfg[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    aid2_axc_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->aid2_axc_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  aid2_ctl_lut_cfg[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    aid2_ctl_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->aid2_ctl_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail0_axc_lut_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail0_axc_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail0_axc_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail0_ctl_lut_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail0_ctl_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail0_ctl_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail1_axc_lut_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail1_axc_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail1_axc_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail1_ctl_lut_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail1_ctl_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail1_ctl_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail2_axc_lut_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail2_axc_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail2_axc_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail2_ctl_lut_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail2_ctl_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail2_ctl_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail3_axc_lut_cfg[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    ail3_axc_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail3_axc_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ail3_ctl_lut_cfg[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    ail3_ctl_lut_cfg[%d] = ", i);
    dump_Iqn2Fl_IqsIngressChanLutCfg(output, &(value->ail3_ctl_lut_cfg[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  dio2_psi_cfg_pri[16] = [\n");
  for(i=0; i<16; i++) {
    fprintf(output, "    dio2_psi_cfg_pri[%d] = %d\n", i, value->dio2_psi_cfg_pri[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_IqsIngressChanLutCfg (FILE *output, Iqn2Fl_IqsIngressChanLutCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_IqsIngressChanLutCfg  */\n");
  fprintf(output, "  chan = %d\n", value->chan);
  fprintf(output, "  dest = %d\n", value->dest);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Obj (FILE *output, Iqn2Fl_Obj *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Obj  */\n");
  fprintf(output, "  arg_ail = %s\n", dump_Iqn2Fl_AilInstance(value->arg_ail));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Param (FILE *output, Iqn2Fl_Param *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Param  */\n");
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_RadtOffsetCfg (FILE *output, Iqn2Fl_RadtOffsetCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_RadtOffsetCfg  */\n");
  fprintf(output, "  offset = %d\n", value->offset);
  fprintf(output, "  radio_std = %d\n", value->radio_std);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_Setup  */\n");
  if (value->aid2Setup != NULL) {
    fprintf(output, "  aid2Setup = ");
    dump_Iqn2Fl_Aid2Setup(output, value->aid2Setup);
  } else {
    fprintf(output, "  aid2Setup = NULL\n");
  }
  fprintf(output, "  ailSetup[IQN2FL_AIL_MAX] = [\n");
  for(i=0; i<IQN2FL_AIL_MAX; i++) {
    if (value->ailSetup != NULL) {
    fprintf(output, "    ailSetup[%d] = ", i);
    dump_Iqn2Fl_AilSetup(output, value->ailSetup[i]);
    } else {
      fprintf(output, "    ailSetup[%d] = NULL\n", i);
    }
  }
  fprintf(output, "  ]\n");
  if (value->at2Setup != NULL) {
    fprintf(output, "  at2Setup = ");
    dump_Iqn2Fl_At2Setup(output, value->at2Setup);
  } else {
    fprintf(output, "  at2Setup = NULL\n");
  }
  if (value->dio2Setup != NULL) {
    fprintf(output, "  dio2Setup = ");
    dump_Iqn2Fl_Dio2Setup(output, value->dio2Setup);
  } else {
    fprintf(output, "  dio2Setup = NULL\n");
  }
  if (value->iqs2Setup != NULL) {
    fprintf(output, "  iqs2Setup = ");
    dump_Iqn2Fl_Iqs2Setup(output, value->iqs2Setup);
  } else {
    fprintf(output, "  iqs2Setup = NULL\n");
  }
  if (value->topSetup != NULL) {
    fprintf(output, "  topSetup = ");
    dump_Iqn2Fl_TopSetup(output, value->topSetup);
  } else {
    fprintf(output, "  topSetup = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_TopPsrConfigSetup (FILE *output, Iqn2Fl_TopPsrConfigSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_TopPsrConfigSetup  */\n");
  fprintf(output, "  arb_priority[48] = [\n");
  for(i=0; i<48; i++) {
    fprintf(output, "    arb_priority[%d] = %d\n", i, value->arb_priority[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bw_limit = %d\n", value->bw_limit);
  fprintf(output, "  drop_pkt[48] = [\n");
  for(i=0; i<48; i++) {
    fprintf(output, "    drop_pkt[%d] = %d\n", i, value->drop_pkt[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  force_flush[48] = [\n");
  for(i=0; i<48; i++) {
    fprintf(output, "    force_flush[%d] = %d\n", i, value->force_flush[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  pack_ps_data[48] = [\n");
  for(i=0; i<48; i++) {
    fprintf(output, "    pack_ps_data[%d] = %d\n", i, value->pack_ps_data[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ps_data_ext = %d\n", value->ps_data_ext);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_TopSetup (FILE *output, Iqn2Fl_TopSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_TopSetup  */\n");
    fprintf(output, "  top_psr_cfg = ");
    dump_Iqn2Fl_TopPsrConfigSetup(output, &(value->top_psr_cfg));
    fprintf(output, "  top_vc_sys_sts_cfg = ");
    dump_Iqn2Fl_TopVcSysStsSetup(output, &(value->top_vc_sys_sts_cfg));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_TopVCSwResetStbSetup (FILE *output, Iqn2Fl_TopVCSwResetStbSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_TopVCSwResetStbSetup  */\n");
  fprintf(output, "  sw_reset = %d\n", value->sw_reset);
  fprintf(output, "  sw_reset_aid = %d\n", value->sw_reset_aid);
  fprintf(output, "  sw_reset_ail0 = %d\n", value->sw_reset_ail0);
  fprintf(output, "  sw_reset_ail1 = %d\n", value->sw_reset_ail1);
  fprintf(output, "  sw_reset_ail2 = %d\n", value->sw_reset_ail2);
  fprintf(output, "  sw_reset_ail3 = %d\n", value->sw_reset_ail3);
  fprintf(output, "  sw_reset_dio = %d\n", value->sw_reset_dio);
  fprintf(output, "  sw_reset_pktdma = %d\n", value->sw_reset_pktdma);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_TopVcSysStsSetup (FILE *output, Iqn2Fl_TopVcSysStsSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_TopVcSysStsSetup  */\n");
  fprintf(output, "  at_dfe_clk_sel = %d\n", value->at_dfe_clk_sel);
  fprintf(output, "  frc_shutdown = %d\n", value->frc_shutdown);
  fprintf(output, "  freerun = %d\n", value->freerun);
  fprintf(output, "  rt_sel = %d\n", value->rt_sel);
  fprintf(output, "  scratch = %d\n", value->scratch);
  fprintf(output, "  soft = %d\n", value->soft);
  fprintf(output, "  sysclk_sel = %d\n", value->sysclk_sel);
  fprintf(output, "  vc_dtmux = %d\n", value->vc_dtmux);
    fprintf(output, "  vc_sys_sts_sw_reset_stb = ");
    dump_Iqn2Fl_TopVCSwResetStbSetup(output, &(value->vc_sys_sts_sw_reset_stb));
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_UatCfg (FILE *output, Iqn2Fl_UatCfg *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_UatCfg  */\n");
  fprintf(output, "  diag_sync = %d\n", value->diag_sync);
  fprintf(output, "  uat_run = %d\n", value->uat_run);
  fprintf(output, "}\n");
}

void dump_Iqn2Fl_UatRadtEvtSetup (FILE *output, Iqn2Fl_UatRadtEvtSetup *value) {
  int i, j;

  fprintf(output, "{ /* Type: Iqn2Fl_UatRadtEvtSetup  */\n");
  fprintf(output, "  clk_cnt_tc = %d\n", value->clk_cnt_tc);
  fprintf(output, "  cmp_cfg_val = %d\n", value->cmp_cfg_val);
  fprintf(output, "  event_num = %d\n", value->event_num);
  fprintf(output, "}\n");
}

