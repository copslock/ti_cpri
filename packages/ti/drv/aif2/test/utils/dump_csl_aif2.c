#include <ti/drv/aif2/aif2fl.h>
#include <stdio.h>

/* Prototypes for enum dumping functions.  */
char *dump_Aif2Fl_AdBcnTable (Aif2Fl_AdBcnTable value);
char *dump_Aif2Fl_AdEgrPriority (Aif2Fl_AdEgrPriority value);
char *dump_Aif2Fl_AdFailMode (Aif2Fl_AdFailMode value);
char *dump_Aif2Fl_AdIngrPriority (Aif2Fl_AdIngrPriority value);
char *dump_Aif2Fl_AdNumQWord (Aif2Fl_AdNumQWord value);
char *dump_Aif2Fl_AtCrcFlip (Aif2Fl_AtCrcFlip value);
char *dump_Aif2Fl_AtCrcInitOnes (Aif2Fl_AtCrcInitOnes value);
char *dump_Aif2Fl_AtCrcInvert (Aif2Fl_AtCrcInvert value);
char *dump_Aif2Fl_AtCrcUse (Aif2Fl_AtCrcUse value);
char *dump_Aif2Fl_AtEventIndex (Aif2Fl_AtEventIndex value);
char *dump_Aif2Fl_AtEvtStrobe (Aif2Fl_AtEvtStrobe value);
char *dump_Aif2Fl_AtReSyncMode (Aif2Fl_AtReSyncMode value);
char *dump_Aif2Fl_AtRp1CRCUsage (Aif2Fl_AtRp1CRCUsage value);
char *dump_Aif2Fl_AtRp1TypeField (Aif2Fl_AtRp1TypeField value);
char *dump_Aif2Fl_AtSyncMode (Aif2Fl_AtSyncMode value);
char *dump_Aif2Fl_AtSyncSource (Aif2Fl_AtSyncSource value);
char *dump_Aif2Fl_CppiDio (Aif2Fl_CppiDio value);
char *dump_Aif2Fl_CpriAxCPack (Aif2Fl_CpriAxCPack value);
char *dump_Aif2Fl_CpriCwPktDelim (Aif2Fl_CpriCwPktDelim value);
char *dump_Aif2Fl_CrcLen (Aif2Fl_CrcLen value);
char *dump_Aif2Fl_DataWidth (Aif2Fl_DataWidth value);
char *dump_Aif2Fl_DbDataSwap (Aif2Fl_DbDataSwap value);
char *dump_Aif2Fl_DbFifoDepth (Aif2Fl_DbFifoDepth value);
char *dump_Aif2Fl_DbIqOrder (Aif2Fl_DbIqOrder value);
char *dump_Aif2Fl_DbPmControl (Aif2Fl_DbPmControl value);
char *dump_Aif2Fl_DioEngineIndex (Aif2Fl_DioEngineIndex value);
char *dump_Aif2Fl_DioLen (Aif2Fl_DioLen value);
char *dump_Aif2Fl_EeArgIndex (Aif2Fl_EeArgIndex value);
char *dump_Aif2Fl_FrameMode (Aif2Fl_FrameMode value);
char *dump_Aif2Fl_GSMDataFormat (Aif2Fl_GSMDataFormat value);
char *dump_Aif2Fl_HwControlCmd (Aif2Fl_HwControlCmd value);
char *dump_Aif2Fl_HwStatusQuery (Aif2Fl_HwStatusQuery value);
char *dump_Aif2Fl_LinkDataType (Aif2Fl_LinkDataType value);
char *dump_Aif2Fl_LinkIndex (Aif2Fl_LinkIndex value);
char *dump_Aif2Fl_LinkProtocol (Aif2Fl_LinkProtocol value);
char *dump_Aif2Fl_LinkRate (Aif2Fl_LinkRate value);
char *dump_Aif2Fl_ObsaiTsMask (Aif2Fl_ObsaiTsMask value);
char *dump_Aif2Fl_PdDataMode (Aif2Fl_PdDataMode value);
char *dump_Aif2Fl_PdWatchDogReport (Aif2Fl_PdWatchDogReport value);
char *dump_Aif2Fl_PeRtContol (Aif2Fl_PeRtContol value);
char *dump_Aif2Fl_PllMpyFactor (Aif2Fl_PllMpyFactor value);
char *dump_Aif2FlRmErrorSuppress (Aif2FlRmErrorSuppress value);
char *dump_Aif2Fl_RmFifoThold (Aif2Fl_RmFifoThold value);
char *dump_Aif2Fl_RmForceSyncState (Aif2Fl_RmForceSyncState value);
char *dump_Aif2Fl_RmSyncState (Aif2Fl_RmSyncState value);
char *dump_Aif2Fl_RouteMask (Aif2Fl_RouteMask value);
char *dump_Aif2Fl_RtConfig (Aif2Fl_RtConfig value);
char *dump_Aif2Fl_SdClockBypass (Aif2Fl_SdClockBypass value);
char *dump_Aif2Fl_SdClockSelect (Aif2Fl_SdClockSelect value);
char *dump_Aif2Fl_SdLoopBandwidth (Aif2Fl_SdLoopBandwidth value);
char *dump_Aif2Fl_SdRxAlign (Aif2Fl_SdRxAlign value);
char *dump_Aif2Fl_SdRxCdrAlg (Aif2Fl_SdRxCdrAlg value);
char *dump_Aif2Fl_SdRxEqConfig (Aif2Fl_SdRxEqConfig value);
char *dump_Aif2Fl_SdRxInvertPolarity (Aif2Fl_SdRxInvertPolarity value);
char *dump_Aif2Fl_SdRxLos (Aif2Fl_SdRxLos value);
char *dump_Aif2Fl_SdRxTerm (Aif2Fl_SdRxTerm value);
char *dump_Aif2Fl_SdSleepPll (Aif2Fl_SdSleepPll value);
char *dump_Aif2Fl_SdTestPattern (Aif2Fl_SdTestPattern value);
char *dump_Aif2Fl_SdTxInvertPolarity (Aif2Fl_SdTxInvertPolarity value);
char *dump_Aif2Fl_SdTxOutputSwing (Aif2Fl_SdTxOutputSwing value);
char *dump_Aif2Fl_SdTxPostcursorTabWeight (Aif2Fl_SdTxPostcursorTabWeight value);
char *dump_Aif2Fl_SdTxPrecursorTabWeight (Aif2Fl_SdTxPrecursorTabWeight value);
char *dump_Aif2Fl_SdVoltRange (Aif2Fl_SdVoltRange value);
char *dump_Aif2Fl_TmSyncState (Aif2Fl_TmSyncState value);
char *dump_Aif2Fl_TstampFormat (Aif2Fl_TstampFormat value);
/* Prototypes for struct dumping functions.  */
void dump_Aif2Fl_AdCommonSetup (FILE *output, Aif2Fl_AdCommonSetup *value);
void dump_Aif2Fl_AdDioEngine (FILE *output, Aif2Fl_AdDioEngine *value);
void dump_Aif2Fl_AdDioSetup (FILE *output, Aif2Fl_AdDioSetup *value);
void dump_Aif2Fl_AtCaptRadt (FILE *output, Aif2Fl_AtCaptRadt *value);
void dump_Aif2Fl_AtCommonSetup (FILE *output, Aif2Fl_AtCommonSetup *value);
void dump_Aif2Fl_AtCountObj (FILE *output, Aif2Fl_AtCountObj *value);
void dump_Aif2Fl_AtEvent (FILE *output, Aif2Fl_AtEvent *value);
void dump_Aif2Fl_AtEventSetup (FILE *output, Aif2Fl_AtEventSetup *value);
void dump_Aif2Fl_AtGsmTCount (FILE *output, Aif2Fl_AtGsmTCount *value);
void dump_Aif2Fl_AtInitObj (FILE *output, Aif2Fl_AtInitObj *value);
void dump_Aif2Fl_AtLinkSetup (FILE *output, Aif2Fl_AtLinkSetup *value);
void dump_Aif2Fl_AtTcObj (FILE *output, Aif2Fl_AtTcObj *value);
void dump_Aif2Fl_AtWcdmaCount (FILE *output, Aif2Fl_AtWcdmaCount *value);
void dump_Aif2Fl_BaseAddress (FILE *output, Aif2Fl_BaseAddress *value);
void dump_Aif2Fl_CommonLinkSetup (FILE *output, Aif2Fl_CommonLinkSetup *value);
void dump_Aif2Fl_CommonSetup (FILE *output, Aif2Fl_CommonSetup *value);
void dump_Aif2Fl_CpriCwLut (FILE *output, Aif2Fl_CpriCwLut *value);
void dump_Aif2Fl_CpriTmSetup (FILE *output, Aif2Fl_CpriTmSetup *value);
void dump_Aif2Fl_DbChannel (FILE *output, Aif2Fl_DbChannel *value);
void dump_Aif2Fl_DbSideData (FILE *output, Aif2Fl_DbSideData *value);
void dump_Aif2Fl_DualBitMap (FILE *output, Aif2Fl_DualBitMap *value);
void dump_Aif2Fl_EeAdInt (FILE *output, Aif2Fl_EeAdInt *value);
void dump_Aif2Fl_EeAif2Int (FILE *output, Aif2Fl_EeAif2Int *value);
void dump_Aif2Fl_EeAif2Run (FILE *output, Aif2Fl_EeAif2Run *value);
void dump_Aif2Fl_EeAtInt (FILE *output, Aif2Fl_EeAtInt *value);
void dump_Aif2Fl_EeCdInt (FILE *output, Aif2Fl_EeCdInt *value);
void dump_Aif2Fl_EeDbInt (FILE *output, Aif2Fl_EeDbInt *value);
void dump_Aif2Fl_EeLinkAInt (FILE *output, Aif2Fl_EeLinkAInt *value);
void dump_Aif2Fl_EeLinkBInt (FILE *output, Aif2Fl_EeLinkBInt *value);
void dump_Aif2Fl_EeOrigin (FILE *output, Aif2Fl_EeOrigin *value);
void dump_Aif2Fl_EePdInt (FILE *output, Aif2Fl_EePdInt *value);
void dump_Aif2Fl_EePeInt (FILE *output, Aif2Fl_EePeInt *value);
void dump_Aif2Fl_EeSdInt (FILE *output, Aif2Fl_EeSdInt *value);
void dump_Aif2Fl_EeVcInt (FILE *output, Aif2Fl_EeVcInt *value);
void dump_Aif2Fl_EgrDbSetup (FILE *output, Aif2Fl_EgrDbSetup *value);
void dump_Aif2Fl_FrameCounter (FILE *output, Aif2Fl_FrameCounter *value);
void dump_Aif2Fl_GlobalSetup (FILE *output, Aif2Fl_GlobalSetup *value);
void dump_Aif2Fl_IngrDbSetup (FILE *output, Aif2Fl_IngrDbSetup *value);
void dump_Aif2Fl_LinkSetup (FILE *output, Aif2Fl_LinkSetup *value);
void dump_Aif2Fl_ModuloTc (FILE *output, Aif2Fl_ModuloTc *value);
void dump_Aif2Fl_Obj (FILE *output, Aif2Fl_Obj *value);
void dump_Aif2Fl_Param (FILE *output, Aif2Fl_Param *value);
void dump_Aif2Fl_PdChConfig (FILE *output, Aif2Fl_PdChConfig *value);
void dump_Aif2Fl_PdChConfig1 (FILE *output, Aif2Fl_PdChConfig1 *value);
void dump_Aif2Fl_PdChannelConfig (FILE *output, Aif2Fl_PdChannelConfig *value);
void dump_Aif2Fl_PdCommonSetup (FILE *output, Aif2Fl_PdCommonSetup *value);
void dump_Aif2Fl_PdCpriIdLut (FILE *output, Aif2Fl_PdCpriIdLut *value);
void dump_Aif2Fl_PdLinkSetup (FILE *output, Aif2Fl_PdLinkSetup *value);
void dump_Aif2Fl_PdRoute (FILE *output, Aif2Fl_PdRoute *value);
void dump_Aif2Fl_PdTypeLut (FILE *output, Aif2Fl_PdTypeLut *value);
void dump_Aif2Fl_PeChRuleLut (FILE *output, Aif2Fl_PeChRuleLut *value);
void dump_Aif2Fl_PeChannelConfig (FILE *output, Aif2Fl_PeChannelConfig *value);
void dump_Aif2Fl_PeCommonSetup (FILE *output, Aif2Fl_PeCommonSetup *value);
void dump_Aif2Fl_PeDbmr (FILE *output, Aif2Fl_PeDbmr *value);
void dump_Aif2Fl_PeDmaCh0 (FILE *output, Aif2Fl_PeDmaCh0 *value);
void dump_Aif2Fl_PeInFifoControl (FILE *output, Aif2Fl_PeInFifoControl *value);
void dump_Aif2Fl_PeLinkSetup (FILE *output, Aif2Fl_PeLinkSetup *value);
void dump_Aif2Fl_PeModuloRule (FILE *output, Aif2Fl_PeModuloRule *value);
void dump_Aif2Fl_PeObsaiHeader (FILE *output, Aif2Fl_PeObsaiHeader *value);
void dump_Aif2Fl_PidStatus (FILE *output, Aif2Fl_PidStatus *value);
void dump_Aif2Fl_RmLinkSetup (FILE *output, Aif2Fl_RmLinkSetup *value);
void dump_Aif2Fl_RmStatus0 (FILE *output, Aif2Fl_RmStatus0 *value);
void dump_Aif2Fl_RmStatus1 (FILE *output, Aif2Fl_RmStatus1 *value);
void dump_Aif2Fl_RmStatus2 (FILE *output, Aif2Fl_RmStatus2 *value);
void dump_Aif2Fl_RmStatus3 (FILE *output, Aif2Fl_RmStatus3 *value);
void dump_Aif2Fl_RmStatus4 (FILE *output, Aif2Fl_RmStatus4 *value);
void dump_Aif2Fl_RtHeaderStatus (FILE *output, Aif2Fl_RtHeaderStatus *value);
void dump_Aif2Fl_RtLinkSetup (FILE *output, Aif2Fl_RtLinkSetup *value);
void dump_Aif2Fl_RtStatus (FILE *output, Aif2Fl_RtStatus *value);
void dump_Aif2Fl_SdCommonSetup (FILE *output, Aif2Fl_SdCommonSetup *value);
void dump_Aif2Fl_SdLinkSetup (FILE *output, Aif2Fl_SdLinkSetup *value);
void dump_Aif2Fl_SdRxStatus (FILE *output, Aif2Fl_SdRxStatus *value);
void dump_Aif2Fl_SdTxStatus (FILE *output, Aif2Fl_SdTxStatus *value);
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
void dump_Aif2Fl_TmLinkSetup (FILE *output, Aif2Fl_TmLinkSetup *value);
void dump_Aif2Fl_TmStatus (FILE *output, Aif2Fl_TmStatus *value);
void dump_Aif2Fl_VcEmu (FILE *output, Aif2Fl_VcEmu *value);

/* Enum dumping functions.  */
char *dump_Aif2Fl_AdBcnTable (Aif2Fl_AdBcnTable value) {
  switch(value) {
  case AIF2FL_AD_DIO_BCN_TABLE_0:
    return "AIF2FL_AD_DIO_BCN_TABLE_0";
  case AIF2FL_AD_DIO_BCN_TABLE_1:
    return "AIF2FL_AD_DIO_BCN_TABLE_1";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AdEgrPriority (Aif2Fl_AdEgrPriority value) {
  switch(value) {
  case AIF2FL_AD_AXC_PRI:
    return "AIF2FL_AD_AXC_PRI";
  case AIF2FL_AD_NON_AXC_PRI:
    return "AIF2FL_AD_NON_AXC_PRI";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AdFailMode (Aif2Fl_AdFailMode value) {
  switch(value) {
  case AIF2FL_AD_DROP:
    return "AIF2FL_AD_DROP";
  case AIF2FL_AD_MARK:
    return "AIF2FL_AD_MARK";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AdIngrPriority (Aif2Fl_AdIngrPriority value) {
  switch(value) {
  case AIF2FL_AD_DIO_PRI:
    return "AIF2FL_AD_DIO_PRI";
  case AIF2FL_AD_PKT_PRI:
    return "AIF2FL_AD_PKT_PRI";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AdNumQWord (Aif2Fl_AdNumQWord value) {
  switch(value) {
  case AIF2FL_AD_1QUAD:
    return "AIF2FL_AD_1QUAD";
  case AIF2FL_AD_2QUAD:
    return "AIF2FL_AD_2QUAD";
  case AIF2FL_AD_4QUAD:
    return "AIF2FL_AD_4QUAD";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtCrcFlip (Aif2Fl_AtCrcFlip value) {
  switch(value) {
  case AIF2FL_AT_CRC_NORMAL:
    return "AIF2FL_AT_CRC_NORMAL";
  case AIF2FL_AT_CRC_REVERSE:
    return "AIF2FL_AT_CRC_REVERSE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtCrcInitOnes (Aif2Fl_AtCrcInitOnes value) {
  switch(value) {
  case AIF2FL_AT_CRC_INIT0:
    return "AIF2FL_AT_CRC_INIT0";
  case AIF2FL_AT_CRC_INIT1:
    return "AIF2FL_AT_CRC_INIT1";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtCrcInvert (Aif2Fl_AtCrcInvert value) {
  switch(value) {
  case AIF2FL_AT_CRC_INVERT:
    return "AIF2FL_AT_CRC_INVERT";
  case AIF2FL_AT_CRC_NOINVERT:
    return "AIF2FL_AT_CRC_NOINVERT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtCrcUse (Aif2Fl_AtCrcUse value) {
  switch(value) {
  case AIF2FL_AT_CRC_DONT_USE:
    return "AIF2FL_AT_CRC_DONT_USE";
  case AIF2FL_AT_CRC_USE:
    return "AIF2FL_AT_CRC_USE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtEventIndex (Aif2Fl_AtEventIndex value) {
  switch(value) {
  case AIF2FL_EVENT_0:
    return "AIF2FL_EVENT_0";
  case AIF2FL_EVENT_1:
    return "AIF2FL_EVENT_1";
  case AIF2FL_EVENT_10:
    return "AIF2FL_EVENT_10";
  case AIF2FL_EVENT_2:
    return "AIF2FL_EVENT_2";
  case AIF2FL_EVENT_3:
    return "AIF2FL_EVENT_3";
  case AIF2FL_EVENT_4:
    return "AIF2FL_EVENT_4";
  case AIF2FL_EVENT_5:
    return "AIF2FL_EVENT_5";
  case AIF2FL_EVENT_6:
    return "AIF2FL_EVENT_6";
  case AIF2FL_EVENT_7:
    return "AIF2FL_EVENT_7";
  case AIF2FL_EVENT_8:
    return "AIF2FL_EVENT_8";
  case AIF2FL_EVENT_9:
    return "AIF2FL_EVENT_9";
  case AIF2FL_E_DIO_EVENT_0:
    return "AIF2FL_E_DIO_EVENT_0";
  case AIF2FL_E_DIO_EVENT_1:
    return "AIF2FL_E_DIO_EVENT_1";
  case AIF2FL_E_DIO_EVENT_2:
    return "AIF2FL_E_DIO_EVENT_2";
  case AIF2FL_IN_DIO_EVENT_0:
    return "AIF2FL_IN_DIO_EVENT_0";
  case AIF2FL_IN_DIO_EVENT_1:
    return "AIF2FL_IN_DIO_EVENT_1";
  case AIF2FL_IN_DIO_EVENT_2:
    return "AIF2FL_IN_DIO_EVENT_2";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtEvtStrobe (Aif2Fl_AtEvtStrobe value) {
  switch(value) {
  case AIF2FL_DLRADT_FRAME:
    return "AIF2FL_DLRADT_FRAME";
  case AIF2FL_DLRADT_SYMBOL:
    return "AIF2FL_DLRADT_SYMBOL";
  case AIF2FL_PHYT_FRAME:
    return "AIF2FL_PHYT_FRAME";
  case AIF2FL_RADT_FRAME:
    return "AIF2FL_RADT_FRAME";
  case AIF2FL_RADT_SYMBOL:
    return "AIF2FL_RADT_SYMBOL";
  case AIF2FL_ULRADT_FRAME:
    return "AIF2FL_ULRADT_FRAME";
  case AIF2FL_ULRADT_SYMBOL:
    return "AIF2FL_ULRADT_SYMBOL";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtReSyncMode (Aif2Fl_AtReSyncMode value) {
  switch(value) {
  case AIF2FL_AUTO_RESYNC_MODE:
    return "AIF2FL_AUTO_RESYNC_MODE";
  case AIF2FL_NO_AUTO_RESYNC_MODE:
    return "AIF2FL_NO_AUTO_RESYNC_MODE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtRp1CRCUsage (Aif2Fl_AtRp1CRCUsage value) {
  switch(value) {
  case AIF2FL_DISCARD_SYNC_BURST_ON_CRC_FAIL:
    return "AIF2FL_DISCARD_SYNC_BURST_ON_CRC_FAIL";
  case AIF2FL_USE_SYNC_BURST_ON_CRC_FAIL:
    return "AIF2FL_USE_SYNC_BURST_ON_CRC_FAIL";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtRp1TypeField (Aif2Fl_AtRp1TypeField value) {
  switch(value) {
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtSyncMode (Aif2Fl_AtSyncMode value) {
  switch(value) {
  case AIF2FL_NON_RP1_MODE:
    return "AIF2FL_NON_RP1_MODE";
  case AIF2FL_RP1_MODE:
    return "AIF2FL_RP1_MODE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_AtSyncSource (Aif2Fl_AtSyncSource value) {
  switch(value) {
  case AIF2FL_CHIP_INPUT_SYNC:
    return "AIF2FL_CHIP_INPUT_SYNC";
  case AIF2FL_PHYT_CMP_SYNC:
    return "AIF2FL_PHYT_CMP_SYNC";
  case AIF2FL_RM_AT_SYNC:
    return "AIF2FL_RM_AT_SYNC";
  case AIF2FL_RP1_SYNC:
    return "AIF2FL_RP1_SYNC";
  case AIF2FL_SW_SYNC:
    return "AIF2FL_SW_SYNC";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_CppiDio (Aif2Fl_CppiDio value) {
  switch(value) {
  case AIF2FL_CPPI:
    return "AIF2FL_CPPI";
  case AIF2FL_DIO:
    return "AIF2FL_DIO";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_CpriAxCPack (Aif2Fl_CpriAxCPack value) {
  switch(value) {
  case AIF2FL_CPRI_15BIT_SAMPLE:
    return "AIF2FL_CPRI_15BIT_SAMPLE";
  case AIF2FL_CPRI_16BIT_SAMPLE:
    return "AIF2FL_CPRI_16BIT_SAMPLE";
  case AIF2FL_CPRI_7BIT_SAMPLE:
    return "AIF2FL_CPRI_7BIT_SAMPLE";
  case AIF2FL_CPRI_8BIT_SAMPLE:
    return "AIF2FL_CPRI_8BIT_SAMPLE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_CpriCwPktDelim (Aif2Fl_CpriCwPktDelim value) {
  switch(value) {
  case AIF2FL_CW_DELIM_4B5B:
    return "AIF2FL_CW_DELIM_4B5B";
  case AIF2FL_CW_DELIM_HYP_FRM:
    return "AIF2FL_CW_DELIM_HYP_FRM";
  case AIF2FL_CW_DELIM_NO_CW:
    return "AIF2FL_CW_DELIM_NO_CW";
  case AIF2FL_CW_DELIM_NULLDELM:
    return "AIF2FL_CW_DELIM_NULLDELM";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_CrcLen (Aif2Fl_CrcLen value) {
  switch(value) {
  case AIF2FL_CRC_16BIT:
    return "AIF2FL_CRC_16BIT";
  case AIF2FL_CRC_32BIT:
    return "AIF2FL_CRC_32BIT";
  case AIF2FL_CRC_8BIT:
    return "AIF2FL_CRC_8BIT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DataWidth (Aif2Fl_DataWidth value) {
  switch(value) {
  case AIF2FL_DATA_WIDTH_15_BIT:
    return "AIF2FL_DATA_WIDTH_15_BIT";
  case AIF2FL_DATA_WIDTH_16_BIT:
    return "AIF2FL_DATA_WIDTH_16_BIT";
  case AIF2FL_DATA_WIDTH_7_BIT:
    return "AIF2FL_DATA_WIDTH_7_BIT";
  case AIF2FL_DATA_WIDTH_8_BIT:
    return "AIF2FL_DATA_WIDTH_8_BIT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DbDataSwap (Aif2Fl_DbDataSwap value) {
  switch(value) {
  case AIF2FL_DB_BYTE_SWAP:
    return "AIF2FL_DB_BYTE_SWAP";
  case AIF2FL_DB_HALF_WORD_SWAP:
    return "AIF2FL_DB_HALF_WORD_SWAP";
  case AIF2FL_DB_NO_SWAP:
    return "AIF2FL_DB_NO_SWAP";
  case AIF2FL_DB_WORD_SWAP:
    return "AIF2FL_DB_WORD_SWAP";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DbFifoDepth (Aif2Fl_DbFifoDepth value) {
  switch(value) {
  case AIF2FL_DB_FIFO_DEPTH_QW128:
    return "AIF2FL_DB_FIFO_DEPTH_QW128";
  case AIF2FL_DB_FIFO_DEPTH_QW16:
    return "AIF2FL_DB_FIFO_DEPTH_QW16";
  case AIF2FL_DB_FIFO_DEPTH_QW256:
    return "AIF2FL_DB_FIFO_DEPTH_QW256";
  case AIF2FL_DB_FIFO_DEPTH_QW32:
    return "AIF2FL_DB_FIFO_DEPTH_QW32";
  case AIF2FL_DB_FIFO_DEPTH_QW64:
    return "AIF2FL_DB_FIFO_DEPTH_QW64";
  case AIF2FL_DB_FIFO_DEPTH_QW8:
    return "AIF2FL_DB_FIFO_DEPTH_QW8";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DbIqOrder (Aif2Fl_DbIqOrder value) {
  switch(value) {
  case AIF2FL_DB_IQ_16BIT_SWAP:
    return "AIF2FL_DB_IQ_16BIT_SWAP";
  case AIF2FL_DB_IQ_BYTE_SWAP:
    return "AIF2FL_DB_IQ_BYTE_SWAP";
  case AIF2FL_DB_IQ_NO_SWAP:
    return "AIF2FL_DB_IQ_NO_SWAP";
  case AIF2FL_DB_IQ_NO_SWAP1:
    return "AIF2FL_DB_IQ_NO_SWAP1";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DbPmControl (Aif2Fl_DbPmControl value) {
  switch(value) {
  case AIF2FL_DB_AXC_TOKEN_FIFO:
    return "AIF2FL_DB_AXC_TOKEN_FIFO";
  case AIF2FL_DB_PM_TOKEN_FIFO:
    return "AIF2FL_DB_PM_TOKEN_FIFO";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DioEngineIndex (Aif2Fl_DioEngineIndex value) {
  switch(value) {
  case AIF2FL_DIO_ENGINE_0:
    return "AIF2FL_DIO_ENGINE_0";
  case AIF2FL_DIO_ENGINE_1:
    return "AIF2FL_DIO_ENGINE_1";
  case AIF2FL_DIO_ENGINE_2:
    return "AIF2FL_DIO_ENGINE_2";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_DioLen (Aif2Fl_DioLen value) {
  switch(value) {
  case AIF2FL_DB_DIO_LEN_128:
    return "AIF2FL_DB_DIO_LEN_128";
  case AIF2FL_DB_DIO_LEN_256:
    return "AIF2FL_DB_DIO_LEN_256";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_EeArgIndex (Aif2Fl_EeArgIndex value) {
  switch(value) {
  case AIF2FL_EE_INT_CLR:
    return "AIF2FL_EE_INT_CLR";
  case AIF2FL_EE_INT_EN_CLR_EV0:
    return "AIF2FL_EE_INT_EN_CLR_EV0";
  case AIF2FL_EE_INT_EN_CLR_EV1:
    return "AIF2FL_EE_INT_EN_CLR_EV1";
  case AIF2FL_EE_INT_EN_EV0:
    return "AIF2FL_EE_INT_EN_EV0";
  case AIF2FL_EE_INT_EN_EV1:
    return "AIF2FL_EE_INT_EN_EV1";
  case AIF2FL_EE_INT_EN_SET_EV0:
    return "AIF2FL_EE_INT_EN_SET_EV0";
  case AIF2FL_EE_INT_EN_SET_EV1:
    return "AIF2FL_EE_INT_EN_SET_EV1";
  case AIF2FL_EE_INT_EN_STATUS_EV0:
    return "AIF2FL_EE_INT_EN_STATUS_EV0";
  case AIF2FL_EE_INT_EN_STATUS_EV1:
    return "AIF2FL_EE_INT_EN_STATUS_EV1";
  case AIF2FL_EE_INT_RAW_STATUS:
    return "AIF2FL_EE_INT_RAW_STATUS";
  case AIF2FL_EE_INT_SET:
    return "AIF2FL_EE_INT_SET";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_FrameMode (Aif2Fl_FrameMode value) {
  switch(value) {
  case AIF2FL_FRAME_MODE_NORMAL:
    return "AIF2FL_FRAME_MODE_NORMAL";
  case AIF2FL_FRAME_MODE_SHORT:
    return "AIF2FL_FRAME_MODE_SHORT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_GSMDataFormat (Aif2Fl_GSMDataFormat value) {
  switch(value) {
  case AIF2FL_GSM_DATA_OTHER:
    return "AIF2FL_GSM_DATA_OTHER";
  case AIF2FL_GSM_DATA_UL:
    return "AIF2FL_GSM_DATA_UL";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_HwControlCmd (Aif2Fl_HwControlCmd value) {
  switch(value) {
  case AIF2FL_CMD_AD_E_DIO_BCN_TABLE_CHANGE:
    return "AIF2FL_CMD_AD_E_DIO_BCN_TABLE_CHANGE";
  case AIF2FL_CMD_AD_E_DIO_NUM_AXC_CHANGE:
    return "AIF2FL_CMD_AD_E_DIO_NUM_AXC_CHANGE";
  case AIF2FL_CMD_AD_E_DIO_TABLE_SELECT:
    return "AIF2FL_CMD_AD_E_DIO_TABLE_SELECT";
  case AIF2FL_CMD_AD_E_ENABLE_DISABLE_DIO_GLOBAL:
    return "AIF2FL_CMD_AD_E_ENABLE_DISABLE_DIO_GLOBAL";
  case AIF2FL_CMD_AD_E_ENABLE_DISABLE_GLOBAL:
    return "AIF2FL_CMD_AD_E_ENABLE_DISABLE_GLOBAL";
  case AIF2FL_CMD_AD_IN_DIO_BCN_TABLE_CHANGE:
    return "AIF2FL_CMD_AD_IN_DIO_BCN_TABLE_CHANGE";
  case AIF2FL_CMD_AD_IN_DIO_NUM_AXC_CHANGE:
    return "AIF2FL_CMD_AD_IN_DIO_NUM_AXC_CHANGE";
  case AIF2FL_CMD_AD_IN_DIO_TABLE_SELECT:
    return "AIF2FL_CMD_AD_IN_DIO_TABLE_SELECT";
  case AIF2FL_CMD_AD_IN_ENABLE_DISABLE_DIO_GLOBAL:
    return "AIF2FL_CMD_AD_IN_ENABLE_DISABLE_DIO_GLOBAL";
  case AIF2FL_CMD_AD_IN_ENABLE_DISABLE_GLOBAL:
    return "AIF2FL_CMD_AD_IN_ENABLE_DISABLE_GLOBAL";
  case AIF2FL_CMD_AD_TRACE_CPPI_DMA_BURST_WRAP:
    return "AIF2FL_CMD_AD_TRACE_CPPI_DMA_BURST_WRAP";
  case AIF2FL_CMD_AD_TRACE_DATA_BASE_ADDR:
    return "AIF2FL_CMD_AD_TRACE_DATA_BASE_ADDR";
  case AIF2FL_CMD_AD_TRACE_DATA_DMA_CHANNEL_ON_OFF:
    return "AIF2FL_CMD_AD_TRACE_DATA_DMA_CHANNEL_ON_OFF";
  case AIF2FL_CMD_AD_TRACE_FRAMING_DATA_BASE_ADDR:
    return "AIF2FL_CMD_AD_TRACE_FRAMING_DATA_BASE_ADDR";
  case AIF2FL_CMD_AT_ARM_TIMER:
    return "AIF2FL_CMD_AT_ARM_TIMER";
  case AIF2FL_CMD_AT_DEBUG_SYNC:
    return "AIF2FL_CMD_AT_DEBUG_SYNC";
  case AIF2FL_CMD_AT_DELTA_SETUP:
    return "AIF2FL_CMD_AT_DELTA_SETUP";
  case AIF2FL_CMD_AT_DISABLE_ALL_EVENTS:
    return "AIF2FL_CMD_AT_DISABLE_ALL_EVENTS";
  case AIF2FL_CMD_AT_DISABLE_EVENT:
    return "AIF2FL_CMD_AT_DISABLE_EVENT";
  case AIF2FL_CMD_AT_ENABLE_EVENT:
    return "AIF2FL_CMD_AT_ENABLE_EVENT";
  case AIF2FL_CMD_AT_EVENT_SETUP:
    return "AIF2FL_CMD_AT_EVENT_SETUP";
  case AIF2FL_CMD_AT_FORCE_EVENT:
    return "AIF2FL_CMD_AT_FORCE_EVENT";
  case AIF2FL_CMD_AT_GSM_TCOUNT_SETUP:
    return "AIF2FL_CMD_AT_GSM_TCOUNT_SETUP";
  case AIF2FL_CMD_AT_HALT_TIMER:
    return "AIF2FL_CMD_AT_HALT_TIMER";
  case AIF2FL_CMD_AT_RAD_TC_SETUP:
    return "AIF2FL_CMD_AT_RAD_TC_SETUP";
  case AIF2FL_CMD_AT_RAD_WCDMA_DIV:
    return "AIF2FL_CMD_AT_RAD_WCDMA_DIV";
  case AIF2FL_CMD_DB_E_CHANNEL_SETUP:
    return "AIF2FL_CMD_DB_E_CHANNEL_SETUP";
  case AIF2FL_CMD_DB_E_DEBUG_OFFSET_ADDR:
    return "AIF2FL_CMD_DB_E_DEBUG_OFFSET_ADDR";
  case AIF2FL_CMD_DB_E_DEBUG_READ:
    return "AIF2FL_CMD_DB_E_DEBUG_READ";
  case AIF2FL_CMD_DB_E_DEBUG_READ_CONTROL:
    return "AIF2FL_CMD_DB_E_DEBUG_READ_CONTROL";
  case AIF2FL_CMD_DB_E_DEBUG_WRITE_TOKEN:
    return "AIF2FL_CMD_DB_E_DEBUG_WRITE_TOKEN";
  case AIF2FL_CMD_DB_E_ENABLE_DISABLE_CHANNEL:
    return "AIF2FL_CMD_DB_E_ENABLE_DISABLE_CHANNEL";
  case AIF2FL_CMD_DB_E_ENABLE_DISABLE_DEBUG_MODE:
    return "AIF2FL_CMD_DB_E_ENABLE_DISABLE_DEBUG_MODE";
  case AIF2FL_CMD_DB_IN_CHANNEL_SETUP:
    return "AIF2FL_CMD_DB_IN_CHANNEL_SETUP";
  case AIF2FL_CMD_DB_IN_DEBUG_DATA_SETUP:
    return "AIF2FL_CMD_DB_IN_DEBUG_DATA_SETUP";
  case AIF2FL_CMD_DB_IN_DEBUG_OFFSET_ADDR:
    return "AIF2FL_CMD_DB_IN_DEBUG_OFFSET_ADDR";
  case AIF2FL_CMD_DB_IN_DEBUG_SIDE_DATA_SETUP:
    return "AIF2FL_CMD_DB_IN_DEBUG_SIDE_DATA_SETUP";
  case AIF2FL_CMD_DB_IN_DEBUG_WRITE:
    return "AIF2FL_CMD_DB_IN_DEBUG_WRITE";
  case AIF2FL_CMD_DB_IN_ENABLE_DISABLE_CHANNEL:
    return "AIF2FL_CMD_DB_IN_ENABLE_DISABLE_CHANNEL";
  case AIF2FL_CMD_DB_IN_ENABLE_DISABLE_DEBUG_MODE:
    return "AIF2FL_CMD_DB_IN_ENABLE_DISABLE_DEBUG_MODE";
  case AIF2FL_CMD_EE_AD_INT:
    return "AIF2FL_CMD_EE_AD_INT";
  case AIF2FL_CMD_EE_AIF2_ERROR_INT:
    return "AIF2FL_CMD_EE_AIF2_ERROR_INT";
  case AIF2FL_CMD_EE_AIF2_RUN:
    return "AIF2FL_CMD_EE_AIF2_RUN";
  case AIF2FL_CMD_EE_AT_INT:
    return "AIF2FL_CMD_EE_AT_INT";
  case AIF2FL_CMD_EE_CD_INT:
    return "AIF2FL_CMD_EE_CD_INT";
  case AIF2FL_CMD_EE_DB_INT:
    return "AIF2FL_CMD_EE_DB_INT";
  case AIF2FL_CMD_EE_EOI_SETUP:
    return "AIF2FL_CMD_EE_EOI_SETUP";
  case AIF2FL_CMD_EE_LINKA_INT:
    return "AIF2FL_CMD_EE_LINKA_INT";
  case AIF2FL_CMD_EE_LINKB_INT:
    return "AIF2FL_CMD_EE_LINKB_INT";
  case AIF2FL_CMD_EE_PD_INT:
    return "AIF2FL_CMD_EE_PD_INT";
  case AIF2FL_CMD_EE_PE_INT:
    return "AIF2FL_CMD_EE_PE_INT";
  case AIF2FL_CMD_EE_SD_INT:
    return "AIF2FL_CMD_EE_SD_INT";
  case AIF2FL_CMD_EE_VC_INT:
    return "AIF2FL_CMD_EE_VC_INT";
  case AIF2FL_CMD_ENABLE_DISABLE_DATA_TRACE_SYNC:
    return "AIF2FL_CMD_ENABLE_DISABLE_DATA_TRACE_SYNC";
  case AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK:
    return "AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK";
  case AIF2FL_CMD_ENABLE_DISABLE_RX_LINK:
    return "AIF2FL_CMD_ENABLE_DISABLE_RX_LINK";
  case AIF2FL_CMD_ENABLE_DISABLE_SD_B4_PLL:
    return "AIF2FL_CMD_ENABLE_DISABLE_SD_B4_PLL";
  case AIF2FL_CMD_ENABLE_DISABLE_SD_B8_PLL:
    return "AIF2FL_CMD_ENABLE_DISABLE_SD_B8_PLL";
  case AIF2FL_CMD_ENABLE_DISABLE_TX_LINK:
    return "AIF2FL_CMD_ENABLE_DISABLE_TX_LINK";
  case AIF2FL_CMD_PD_CH_CONFIG_SETUP:
    return "AIF2FL_CMD_PD_CH_CONFIG_SETUP";
  case AIF2FL_CMD_PD_CPRI_CW_LUT_SETUP:
    return "AIF2FL_CMD_PD_CPRI_CW_LUT_SETUP";
  case AIF2FL_CMD_PD_CPRI_ID_LUT_SETUP:
    return "AIF2FL_CMD_PD_CPRI_ID_LUT_SETUP";
  case AIF2FL_CMD_PD_LINK_DBMR_SETUP:
    return "AIF2FL_CMD_PD_LINK_DBMR_SETUP";
  case AIF2FL_CMD_PE_CH_CONFIG_SETUP:
    return "AIF2FL_CMD_PE_CH_CONFIG_SETUP";
  case AIF2FL_CMD_PE_CH_RULE_LUT_SETUP:
    return "AIF2FL_CMD_PE_CH_RULE_LUT_SETUP";
  case AIF2FL_CMD_PE_CPRI_CW_LUT_SETUP:
    return "AIF2FL_CMD_PE_CPRI_CW_LUT_SETUP";
  case AIF2FL_CMD_PE_LINK_DBMR_SETUP:
    return "AIF2FL_CMD_PE_LINK_DBMR_SETUP";
  case AIF2FL_CMD_PE_MODULO_RULE_SETUP:
    return "AIF2FL_CMD_PE_MODULO_RULE_SETUP";
  case AIF2FL_CMD_PE_OBSAI_HEADER_SETUP:
    return "AIF2FL_CMD_PE_OBSAI_HEADER_SETUP";
  case AIF2FL_CMD_RM_FORCE_STATE:
    return "AIF2FL_CMD_RM_FORCE_STATE";
  case AIF2FL_CMD_SD_LINK_RX_TEST_PATTERN:
    return "AIF2FL_CMD_SD_LINK_RX_TEST_PATTERN";
  case AIF2FL_CMD_SD_LINK_TX_TEST_PATTERN:
    return "AIF2FL_CMD_SD_LINK_TX_TEST_PATTERN";
  case AIF2FL_CMD_TM_FLUSH_FIFO:
    return "AIF2FL_CMD_TM_FLUSH_FIFO";
  case AIF2FL_CMD_TM_IDLE:
    return "AIF2FL_CMD_TM_IDLE";
  case AIF2FL_CMD_TM_L1_INBAND_SET:
    return "AIF2FL_CMD_TM_L1_INBAND_SET";
  case AIF2FL_CMD_TM_RESYNC:
    return "AIF2FL_CMD_TM_RESYNC";
  case AIF2FL_CMD_VC_EMU_CONTROL:
    return "AIF2FL_CMD_VC_EMU_CONTROL";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_HwStatusQuery (Aif2Fl_HwStatusQuery value) {
  switch(value) {
  case AIF2FL_QUERY_AD_I_EOP_COUNT:
    return "AIF2FL_QUERY_AD_I_EOP_COUNT";
  case AIF2FL_QUERY_AT_DLRAD_CLOCK_COUNT:
    return "AIF2FL_QUERY_AT_DLRAD_CLOCK_COUNT";
  case AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_LSB:
    return "AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_LSB";
  case AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_MSB:
    return "AIF2FL_QUERY_AT_DLRAD_FRAME_COUNT_MSB";
  case AIF2FL_QUERY_AT_DLRAD_SYMBOL_COUNT:
    return "AIF2FL_QUERY_AT_DLRAD_SYMBOL_COUNT";
  case AIF2FL_QUERY_AT_DLRAD_WCDMA_VALUE:
    return "AIF2FL_QUERY_AT_DLRAD_WCDMA_VALUE";
  case AIF2FL_QUERY_AT_GSM_TCOUNT_VALUE:
    return "AIF2FL_QUERY_AT_GSM_TCOUNT_VALUE";
  case AIF2FL_QUERY_AT_LINK_PI_CAPTURE:
    return "AIF2FL_QUERY_AT_LINK_PI_CAPTURE";
  case AIF2FL_QUERY_AT_PHY_CLOCK_COUNT:
    return "AIF2FL_QUERY_AT_PHY_CLOCK_COUNT";
  case AIF2FL_QUERY_AT_PHY_FRAME_COUNT_LSB:
    return "AIF2FL_QUERY_AT_PHY_FRAME_COUNT_LSB";
  case AIF2FL_QUERY_AT_PHY_FRAME_COUNT_MSB:
    return "AIF2FL_QUERY_AT_PHY_FRAME_COUNT_MSB";
  case AIF2FL_QUERY_AT_RADT_CAPTURE:
    return "AIF2FL_QUERY_AT_RADT_CAPTURE";
  case AIF2FL_QUERY_AT_RAD_CLOCK_COUNT:
    return "AIF2FL_QUERY_AT_RAD_CLOCK_COUNT";
  case AIF2FL_QUERY_AT_RAD_FRAME_COUNT_LSB:
    return "AIF2FL_QUERY_AT_RAD_FRAME_COUNT_LSB";
  case AIF2FL_QUERY_AT_RAD_FRAME_COUNT_MSB:
    return "AIF2FL_QUERY_AT_RAD_FRAME_COUNT_MSB";
  case AIF2FL_QUERY_AT_RAD_SYMBOL_COUNT:
    return "AIF2FL_QUERY_AT_RAD_SYMBOL_COUNT";
  case AIF2FL_QUERY_AT_RAD_TSTAMP_CLOCK_COUNT:
    return "AIF2FL_QUERY_AT_RAD_TSTAMP_CLOCK_COUNT";
  case AIF2FL_QUERY_AT_RAD_WCDMA_VALUE:
    return "AIF2FL_QUERY_AT_RAD_WCDMA_VALUE";
  case AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_LSB:
    return "AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_LSB";
  case AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_MSB:
    return "AIF2FL_QUERY_AT_RP1_RAD_CAPTURE_MSB";
  case AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_LSB:
    return "AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_LSB";
  case AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_MSB:
    return "AIF2FL_QUERY_AT_RP1_RP3_CAPTURE_MSB";
  case AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_LSB:
    return "AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_LSB";
  case AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_MSB:
    return "AIF2FL_QUERY_AT_RP1_TOD_CAPTURE_MSB";
  case AIF2FL_QUERY_AT_RP1_TYPE_CAPTURE:
    return "AIF2FL_QUERY_AT_RP1_TYPE_CAPTURE";
  case AIF2FL_QUERY_AT_ULRAD_CLOCK_COUNT:
    return "AIF2FL_QUERY_AT_ULRAD_CLOCK_COUNT";
  case AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_LSB:
    return "AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_LSB";
  case AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_MSB:
    return "AIF2FL_QUERY_AT_ULRAD_FRAME_COUNT_MSB";
  case AIF2FL_QUERY_AT_ULRAD_SYMBOL_COUNT:
    return "AIF2FL_QUERY_AT_ULRAD_SYMBOL_COUNT";
  case AIF2FL_QUERY_AT_ULRAD_WCDMA_VALUE:
    return "AIF2FL_QUERY_AT_ULRAD_WCDMA_VALUE";
  case AIF2FL_QUERY_DB_E_DEBUG_DATA:
    return "AIF2FL_QUERY_DB_E_DEBUG_DATA";
  case AIF2FL_QUERY_DB_E_DEBUG_OFFSET_DATA:
    return "AIF2FL_QUERY_DB_E_DEBUG_OFFSET_DATA";
  case AIF2FL_QUERY_DB_E_DEBUG_SIDE_DATA:
    return "AIF2FL_QUERY_DB_E_DEBUG_SIDE_DATA";
  case AIF2FL_QUERY_DB_E_EOP_COUNT:
    return "AIF2FL_QUERY_DB_E_EOP_COUNT";
  case AIF2FL_QUERY_DB_IN_DEBUG_OFFSET_DATA:
    return "AIF2FL_QUERY_DB_IN_DEBUG_OFFSET_DATA";
  case AIF2FL_QUERY_EE_AD_INT_STATUS:
    return "AIF2FL_QUERY_EE_AD_INT_STATUS";
  case AIF2FL_QUERY_EE_AIF2_ORIGINATION:
    return "AIF2FL_QUERY_EE_AIF2_ORIGINATION";
  case AIF2FL_QUERY_EE_AIF2_RUN_STATUS:
    return "AIF2FL_QUERY_EE_AIF2_RUN_STATUS";
  case AIF2FL_QUERY_EE_AT_INT_STATUS:
    return "AIF2FL_QUERY_EE_AT_INT_STATUS";
  case AIF2FL_QUERY_EE_CD_INT_STATUS:
    return "AIF2FL_QUERY_EE_CD_INT_STATUS";
  case AIF2FL_QUERY_EE_DB_INT_STATUS:
    return "AIF2FL_QUERY_EE_DB_INT_STATUS";
  case AIF2FL_QUERY_EE_LINKA_INT_STATUS:
    return "AIF2FL_QUERY_EE_LINKA_INT_STATUS";
  case AIF2FL_QUERY_EE_LINKB_INT_STATUS:
    return "AIF2FL_QUERY_EE_LINKB_INT_STATUS";
  case AIF2FL_QUERY_EE_PD_INT_STATUS:
    return "AIF2FL_QUERY_EE_PD_INT_STATUS";
  case AIF2FL_QUERY_EE_PE_INT_STATUS:
    return "AIF2FL_QUERY_EE_PE_INT_STATUS";
  case AIF2FL_QUERY_EE_SD_INT_STATUS:
    return "AIF2FL_QUERY_EE_SD_INT_STATUS";
  case AIF2FL_QUERY_EE_VC_INT_STATUS:
    return "AIF2FL_QUERY_EE_VC_INT_STATUS";
  case AIF2FL_QUERY_PD_CHANNEL_STATUS:
    return "AIF2FL_QUERY_PD_CHANNEL_STATUS";
  case AIF2FL_QUERY_PD_PACKET_STATUS:
    return "AIF2FL_QUERY_PD_PACKET_STATUS";
  case AIF2FL_QUERY_PE_CHANNEL_STATUS:
    return "AIF2FL_QUERY_PE_CHANNEL_STATUS";
  case AIF2FL_QUERY_PE_PACKET_STATUS:
    return "AIF2FL_QUERY_PE_PACKET_STATUS";
  case AIF2FL_QUERY_RM_LINK_STATUS_0:
    return "AIF2FL_QUERY_RM_LINK_STATUS_0";
  case AIF2FL_QUERY_RM_LINK_STATUS_1:
    return "AIF2FL_QUERY_RM_LINK_STATUS_1";
  case AIF2FL_QUERY_RM_LINK_STATUS_2:
    return "AIF2FL_QUERY_RM_LINK_STATUS_2";
  case AIF2FL_QUERY_RM_LINK_STATUS_3:
    return "AIF2FL_QUERY_RM_LINK_STATUS_3";
  case AIF2FL_QUERY_RM_LINK_STATUS_4:
    return "AIF2FL_QUERY_RM_LINK_STATUS_4";
  case AIF2FL_QUERY_RT_FIFO_DEPTH_STATUS:
    return "AIF2FL_QUERY_RT_FIFO_DEPTH_STATUS";
  case AIF2FL_QUERY_RT_HEADER_ERROR_STATUS:
    return "AIF2FL_QUERY_RT_HEADER_ERROR_STATUS";
  case AIF2FL_QUERY_RT_LINK_STATUS:
    return "AIF2FL_QUERY_RT_LINK_STATUS";
  case AIF2FL_QUERY_SD_B4_PLL_LOCK:
    return "AIF2FL_QUERY_SD_B4_PLL_LOCK";
  case AIF2FL_QUERY_SD_B8_PLL_LOCK:
    return "AIF2FL_QUERY_SD_B8_PLL_LOCK";
  case AIF2FL_QUERY_SD_RX_LINK_STATUS:
    return "AIF2FL_QUERY_SD_RX_LINK_STATUS";
  case AIF2FL_QUERY_SD_TX_LINK_STATUS:
    return "AIF2FL_QUERY_SD_TX_LINK_STATUS";
  case AIF2FL_QUERY_TM_LINK_CPRI_HFN:
    return "AIF2FL_QUERY_TM_LINK_CPRI_HFN";
  case AIF2FL_QUERY_TM_LINK_STATUS:
    return "AIF2FL_QUERY_TM_LINK_STATUS";
  case AIF2FL_QUERY_VC_STAT:
    return "AIF2FL_QUERY_VC_STAT";
  case AIF2FL_QUERY_VERSION:
    return "AIF2FL_QUERY_VERSION";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_LinkDataType (Aif2Fl_LinkDataType value) {
  switch(value) {
  case AIF2FL_LINK_DATA_TYPE_NORMAL:
    return "AIF2FL_LINK_DATA_TYPE_NORMAL";
  case AIF2FL_LINK_DATA_TYPE_RSA:
    return "AIF2FL_LINK_DATA_TYPE_RSA";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_LinkIndex (Aif2Fl_LinkIndex value) {
  switch(value) {
  case AIF2FL_LINK_0:
    return "AIF2FL_LINK_0";
  case AIF2FL_LINK_1:
    return "AIF2FL_LINK_1";
  case AIF2FL_LINK_2:
    return "AIF2FL_LINK_2";
  case AIF2FL_LINK_3:
    return "AIF2FL_LINK_3";
  case AIF2FL_LINK_4:
    return "AIF2FL_LINK_4";
  case AIF2FL_LINK_5:
    return "AIF2FL_LINK_5";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_LinkProtocol (Aif2Fl_LinkProtocol value) {
  switch(value) {
  case AIF2FL_LINK_PROTOCOL_CPRI:
    return "AIF2FL_LINK_PROTOCOL_CPRI";
  case AIF2FL_LINK_PROTOCOL_OBSAI:
    return "AIF2FL_LINK_PROTOCOL_OBSAI";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_LinkRate (Aif2Fl_LinkRate value) {
  switch(value) {
  case AIF2FL_LINK_RATE_2x:
    return "AIF2FL_LINK_RATE_2x";
  case AIF2FL_LINK_RATE_4x:
    return "AIF2FL_LINK_RATE_4x";
  case AIF2FL_LINK_RATE_5x:
    return "AIF2FL_LINK_RATE_5x";
  case AIF2FL_LINK_RATE_8x:
    return "AIF2FL_LINK_RATE_8x";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_ObsaiTsMask (Aif2Fl_ObsaiTsMask value) {
  switch(value) {
  case AIF2FL_TSTAMP_MASK_4INS_2GEN:
    return "AIF2FL_TSTAMP_MASK_4INS_2GEN";
  case AIF2FL_TSTAMP_MASK_FULL_GEN:
    return "AIF2FL_TSTAMP_MASK_FULL_GEN";
  case AIF2FL_TSTAMP_MASK_FULL_INS:
    return "AIF2FL_TSTAMP_MASK_FULL_INS";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_PdDataMode (Aif2Fl_PdDataMode value) {
  switch(value) {
  case AIF2FL_PD_DATA_AXC:
    return "AIF2FL_PD_DATA_AXC";
  case AIF2FL_PD_DATA_PKT:
    return "AIF2FL_PD_DATA_PKT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_PdWatchDogReport (Aif2Fl_PdWatchDogReport value) {
  switch(value) {
  case AIF2FL_PD_WD_REPORT_ALL:
    return "AIF2FL_PD_WD_REPORT_ALL";
  case AIF2FL_PD_WD_REPORT_EOP:
    return "AIF2FL_PD_WD_REPORT_EOP";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_PeRtContol (Aif2Fl_PeRtContol value) {
  switch(value) {
  case AIF2FL_PE_RT_ADD16:
    return "AIF2FL_PE_RT_ADD16";
  case AIF2FL_PE_RT_ADD8:
    return "AIF2FL_PE_RT_ADD8";
  case AIF2FL_PE_RT_INSERT:
    return "AIF2FL_PE_RT_INSERT";
  case AIF2FL_PE_RT_RETRANS:
    return "AIF2FL_PE_RT_RETRANS";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_PllMpyFactor (Aif2Fl_PllMpyFactor value) {
  switch(value) {
  case AIF2FL_PLL_MUL_FACTOR_10X:
    return "AIF2FL_PLL_MUL_FACTOR_10X";
  case AIF2FL_PLL_MUL_FACTOR_12X:
    return "AIF2FL_PLL_MUL_FACTOR_12X";
  case AIF2FL_PLL_MUL_FACTOR_12_5X:
    return "AIF2FL_PLL_MUL_FACTOR_12_5X";
  case AIF2FL_PLL_MUL_FACTOR_15X:
    return "AIF2FL_PLL_MUL_FACTOR_15X";
  case AIF2FL_PLL_MUL_FACTOR_16X:
    return "AIF2FL_PLL_MUL_FACTOR_16X";
  case AIF2FL_PLL_MUL_FACTOR_16_5X:
    return "AIF2FL_PLL_MUL_FACTOR_16_5X";
  case AIF2FL_PLL_MUL_FACTOR_20X:
    return "AIF2FL_PLL_MUL_FACTOR_20X";
  case AIF2FL_PLL_MUL_FACTOR_22X:
    return "AIF2FL_PLL_MUL_FACTOR_22X";
  case AIF2FL_PLL_MUL_FACTOR_25X:
    return "AIF2FL_PLL_MUL_FACTOR_25X";
  case AIF2FL_PLL_MUL_FACTOR_4X:
    return "AIF2FL_PLL_MUL_FACTOR_4X";
  case AIF2FL_PLL_MUL_FACTOR_5X:
    return "AIF2FL_PLL_MUL_FACTOR_5X";
  case AIF2FL_PLL_MUL_FACTOR_6X:
    return "AIF2FL_PLL_MUL_FACTOR_6X";
  case AIF2FL_PLL_MUL_FACTOR_8X:
    return "AIF2FL_PLL_MUL_FACTOR_8X";
  case AIF2FL_PLL_MUL_FACTOR_8_25X:
    return "AIF2FL_PLL_MUL_FACTOR_8_25X";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2FlRmErrorSuppress (Aif2FlRmErrorSuppress value) {
  switch(value) {
  case AIF2FL_RM_ERROR_ALLOW:
    return "AIF2FL_RM_ERROR_ALLOW";
  case AIF2FL_RM_ERROR_SUPPRESS:
    return "AIF2FL_RM_ERROR_SUPPRESS";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_RmFifoThold (Aif2Fl_RmFifoThold value) {
  switch(value) {
  case AIF2FL_RM_FIFO_THOLD_16DUAL:
    return "AIF2FL_RM_FIFO_THOLD_16DUAL";
  case AIF2FL_RM_FIFO_THOLD_4DUAL:
    return "AIF2FL_RM_FIFO_THOLD_4DUAL";
  case AIF2FL_RM_FIFO_THOLD_8DUAL:
    return "AIF2FL_RM_FIFO_THOLD_8DUAL";
  case AIF2FL_RM_FIFO_THOLD_IMMEDIATELY:
    return "AIF2FL_RM_FIFO_THOLD_IMMEDIATELY";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_RmForceSyncState (Aif2Fl_RmForceSyncState value) {
  switch(value) {
  case AIF2FL_RM_FORCE_ST0:
    return "AIF2FL_RM_FORCE_ST0";
  case AIF2FL_RM_FORCE_ST1:
    return "AIF2FL_RM_FORCE_ST1";
  case AIF2FL_RM_FORCE_ST2:
    return "AIF2FL_RM_FORCE_ST2";
  case AIF2FL_RM_FORCE_ST3:
    return "AIF2FL_RM_FORCE_ST3";
  case AIF2FL_RM_FORCE_ST4:
    return "AIF2FL_RM_FORCE_ST4";
  case AIF2FL_RM_FORCE_ST5:
    return "AIF2FL_RM_FORCE_ST5";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_RmSyncState (Aif2Fl_RmSyncState value) {
  switch(value) {
  case AIF2FL_RM_ST_0:
    return "AIF2FL_RM_ST_0";
  case AIF2FL_RM_ST_1:
    return "AIF2FL_RM_ST_1";
  case AIF2FL_RM_ST_2:
    return "AIF2FL_RM_ST_2";
  case AIF2FL_RM_ST_3:
    return "AIF2FL_RM_ST_3";
  case AIF2FL_RM_ST_4:
    return "AIF2FL_RM_ST_4";
  case AIF2FL_RM_ST_5:
    return "AIF2FL_RM_ST_5";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_RouteMask (Aif2Fl_RouteMask value) {
  switch(value) {
  case AIF2FL_ROUTE_MASK_4LSB:
    return "AIF2FL_ROUTE_MASK_4LSB";
  case AIF2FL_ROUTE_MASK_ALL:
    return "AIF2FL_ROUTE_MASK_ALL";
  case AIF2FL_ROUTE_MASK_NONE:
    return "AIF2FL_ROUTE_MASK_NONE";
  case AIF2FL_ROUTE_MASK_RESERVED:
    return "AIF2FL_ROUTE_MASK_RESERVED";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_RtConfig (Aif2Fl_RtConfig value) {
  switch(value) {
  case AIF2FL_RT_MODE_AGGREGATE:
    return "AIF2FL_RT_MODE_AGGREGATE";
  case AIF2FL_RT_MODE_RETRANSMIT:
    return "AIF2FL_RT_MODE_RETRANSMIT";
  case AIF2FL_RT_MODE_TRANSMIT:
    return "AIF2FL_RT_MODE_TRANSMIT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdClockBypass (Aif2Fl_SdClockBypass value) {
  switch(value) {
  case AIF2FL_PLL_CLOCK_FUNCTIONAL_BYPASS:
    return "AIF2FL_PLL_CLOCK_FUNCTIONAL_BYPASS";
  case AIF2FL_PLL_CLOCK_NO_BYPASS:
    return "AIF2FL_PLL_CLOCK_NO_BYPASS";
  case AIF2FL_PLL_CLOCK_REFCLK_OBSERVE:
    return "AIF2FL_PLL_CLOCK_REFCLK_OBSERVE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdClockSelect (Aif2Fl_SdClockSelect value) {
  switch(value) {
  case AIF2FL_SD_BYTECLOCK_FROM_B4:
    return "AIF2FL_SD_BYTECLOCK_FROM_B4";
  case AIF2FL_SD_BYTECLOCK_FROM_B8:
    return "AIF2FL_SD_BYTECLOCK_FROM_B8";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdLoopBandwidth (Aif2Fl_SdLoopBandwidth value) {
  switch(value) {
  case AIF2FL_PLL_LOOP_BAND_HIGH:
    return "AIF2FL_PLL_LOOP_BAND_HIGH";
  case AIF2FL_PLL_LOOP_BAND_LOW:
    return "AIF2FL_PLL_LOOP_BAND_LOW";
  case AIF2FL_PLL_LOOP_BAND_MID:
    return "AIF2FL_PLL_LOOP_BAND_MID";
  case AIF2FL_PLL_LOOP_BAND_UHIGH:
    return "AIF2FL_PLL_LOOP_BAND_UHIGH";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxAlign (Aif2Fl_SdRxAlign value) {
  switch(value) {
  case AIF2FL_SD_RX_ALIGNMENT_DISABLE:
    return "AIF2FL_SD_RX_ALIGNMENT_DISABLE";
  case AIF2FL_SD_RX_ALIGNMENT_JOG:
    return "AIF2FL_SD_RX_ALIGNMENT_JOG";
  case AIF2FL_SD_RX_COMMA_ALIGNMENT_ENABLE:
    return "AIF2FL_SD_RX_COMMA_ALIGNMENT_ENABLE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxCdrAlg (Aif2Fl_SdRxCdrAlg value) {
  switch(value) {
  case AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_1:
    return "AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_1";
  case AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_17:
    return "AIF2FL_SD_RX_CDR_FIRST_ORDER_THRESH_17";
  case AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_1:
    return "AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_1";
  case AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_17:
    return "AIF2FL_SD_RX_CDR_FO_PERIODIC_THRESH_17";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxEqConfig (Aif2Fl_SdRxEqConfig value) {
  switch(value) {
  case AIF2FL_SD_RX_EQ_ADAPTIVE:
    return "AIF2FL_SD_RX_EQ_ADAPTIVE";
  case AIF2FL_SD_RX_EQ_MAXIMUM:
    return "AIF2FL_SD_RX_EQ_MAXIMUM";
  case AIF2FL_SD_RX_EQ_POSTCURSOR:
    return "AIF2FL_SD_RX_EQ_POSTCURSOR";
  case AIF2FL_SD_RX_EQ_PRECURSOR:
    return "AIF2FL_SD_RX_EQ_PRECURSOR";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxInvertPolarity (Aif2Fl_SdRxInvertPolarity value) {
  switch(value) {
  case AIF2FL_SD_RX_INVERTED_POLARITY:
    return "AIF2FL_SD_RX_INVERTED_POLARITY";
  case AIF2FL_SD_RX_NORMAL_POLARITY:
    return "AIF2FL_SD_RX_NORMAL_POLARITY";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxLos (Aif2Fl_SdRxLos value) {
  switch(value) {
  case AIF2FL_SD_RX_LOS_DISABLE:
    return "AIF2FL_SD_RX_LOS_DISABLE";
  case AIF2FL_SD_RX_LOS_ENABLE:
    return "AIF2FL_SD_RX_LOS_ENABLE";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdRxTerm (Aif2Fl_SdRxTerm value) {
  switch(value) {
  case AIF2FL_SD_RX_TERM_COMMON_POINT_0_7:
    return "AIF2FL_SD_RX_TERM_COMMON_POINT_0_7";
  case AIF2FL_SD_RX_TERM_COMMON_POINT_FLOATING:
    return "AIF2FL_SD_RX_TERM_COMMON_POINT_FLOATING";
  case AIF2FL_SD_RX_TERM_COMMON_POINT_VDDT:
    return "AIF2FL_SD_RX_TERM_COMMON_POINT_VDDT";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdSleepPll (Aif2Fl_SdSleepPll value) {
  switch(value) {
  case AIF2FL_PLL_AWAKE:
    return "AIF2FL_PLL_AWAKE";
  case AIF2FL_PLL_SLEEP:
    return "AIF2FL_PLL_SLEEP";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdTestPattern (Aif2Fl_SdTestPattern value) {
  switch(value) {
  case AIF2FL_SD_ALTERNATING_0_1:
    return "AIF2FL_SD_ALTERNATING_0_1";
  case AIF2FL_SD_PRBS_23BIT_LFSR:
    return "AIF2FL_SD_PRBS_23BIT_LFSR";
  case AIF2FL_SD_PRBS_31BIT_LFSR:
    return "AIF2FL_SD_PRBS_31BIT_LFSR";
  case AIF2FL_SD_PRBS_7BIT_LFSR:
    return "AIF2FL_SD_PRBS_7BIT_LFSR";
  case AIF2FL_SD_TEST_DISABLED:
    return "AIF2FL_SD_TEST_DISABLED";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdTxInvertPolarity (Aif2Fl_SdTxInvertPolarity value) {
  switch(value) {
  case AIF2FL_SD_TX_PAIR_INVERTED_POLARITY:
    return "AIF2FL_SD_TX_PAIR_INVERTED_POLARITY";
  case AIF2FL_SD_TX_PAIR_NORMAL_POLARITY:
    return "AIF2FL_SD_TX_PAIR_NORMAL_POLARITY";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdTxOutputSwing (Aif2Fl_SdTxOutputSwing value) {
  switch(value) {
  case AIF2FL_SD_TX_OUTPUT_SWING_0:
    return "AIF2FL_SD_TX_OUTPUT_SWING_0";
  case AIF2FL_SD_TX_OUTPUT_SWING_1:
    return "AIF2FL_SD_TX_OUTPUT_SWING_1";
  case AIF2FL_SD_TX_OUTPUT_SWING_10:
    return "AIF2FL_SD_TX_OUTPUT_SWING_10";
  case AIF2FL_SD_TX_OUTPUT_SWING_11:
    return "AIF2FL_SD_TX_OUTPUT_SWING_11";
  case AIF2FL_SD_TX_OUTPUT_SWING_12:
    return "AIF2FL_SD_TX_OUTPUT_SWING_12";
  case AIF2FL_SD_TX_OUTPUT_SWING_13:
    return "AIF2FL_SD_TX_OUTPUT_SWING_13";
  case AIF2FL_SD_TX_OUTPUT_SWING_14:
    return "AIF2FL_SD_TX_OUTPUT_SWING_14";
  case AIF2FL_SD_TX_OUTPUT_SWING_15:
    return "AIF2FL_SD_TX_OUTPUT_SWING_15";
  case AIF2FL_SD_TX_OUTPUT_SWING_2:
    return "AIF2FL_SD_TX_OUTPUT_SWING_2";
  case AIF2FL_SD_TX_OUTPUT_SWING_3:
    return "AIF2FL_SD_TX_OUTPUT_SWING_3";
  case AIF2FL_SD_TX_OUTPUT_SWING_4:
    return "AIF2FL_SD_TX_OUTPUT_SWING_4";
  case AIF2FL_SD_TX_OUTPUT_SWING_5:
    return "AIF2FL_SD_TX_OUTPUT_SWING_5";
  case AIF2FL_SD_TX_OUTPUT_SWING_6:
    return "AIF2FL_SD_TX_OUTPUT_SWING_6";
  case AIF2FL_SD_TX_OUTPUT_SWING_7:
    return "AIF2FL_SD_TX_OUTPUT_SWING_7";
  case AIF2FL_SD_TX_OUTPUT_SWING_8:
    return "AIF2FL_SD_TX_OUTPUT_SWING_8";
  case AIF2FL_SD_TX_OUTPUT_SWING_9:
    return "AIF2FL_SD_TX_OUTPUT_SWING_9";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdTxPostcursorTabWeight (Aif2Fl_SdTxPostcursorTabWeight value) {
  switch(value) {
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_0:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_0";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_1:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_1";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_10:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_10";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_11:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_11";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_12:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_12";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_13:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_13";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_14:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_14";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_15:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_15";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_16:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_16";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_17:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_17";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_18:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_18";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_19:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_19";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_2:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_2";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_20:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_20";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_21:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_21";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_22:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_22";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_23:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_23";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_24:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_24";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_25:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_25";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_26:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_26";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_27:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_27";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_28:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_28";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_29:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_29";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_3:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_3";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_30:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_30";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_31:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_31";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_4:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_4";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_5:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_5";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_6:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_6";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_7:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_7";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_8:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_8";
  case AIF2FL_SD_TX_POST_TAP_WEIGHT_9:
    return "AIF2FL_SD_TX_POST_TAP_WEIGHT_9";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdTxPrecursorTabWeight (Aif2Fl_SdTxPrecursorTabWeight value) {
  switch(value) {
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_0:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_0";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_1:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_1";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_2:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_2";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_3:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_3";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_4:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_4";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_5:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_5";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_6:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_6";
  case AIF2FL_SD_TX_PRE_TAP_WEIGHT_7:
    return "AIF2FL_SD_TX_PRE_TAP_WEIGHT_7";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_SdVoltRange (Aif2Fl_SdVoltRange value) {
  switch(value) {
  case AIF2FL_PLL_VOLTAGE_HIGH:
    return "AIF2FL_PLL_VOLTAGE_HIGH";
  case AIF2FL_PLL_VOLTAGE_LOW:
    return "AIF2FL_PLL_VOLTAGE_LOW";
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_TmSyncState (Aif2Fl_TmSyncState value) {
  switch(value) {
  default:
    return "Illegal Enumeration Value";
  }
}

char *dump_Aif2Fl_TstampFormat (Aif2Fl_TstampFormat value) {
  switch(value) {
  case AIF2FL_TSTAMP_FORMAT_ETHERNET:
    return "AIF2FL_TSTAMP_FORMAT_ETHERNET";
  case AIF2FL_TSTAMP_FORMAT_GEN_PKT:
    return "AIF2FL_TSTAMP_FORMAT_GEN_PKT";
  case AIF2FL_TSTAMP_FORMAT_GSM:
    return "AIF2FL_TSTAMP_FORMAT_GSM";
  case AIF2FL_TSTAMP_FORMAT_GSM_DL:
    return "AIF2FL_TSTAMP_FORMAT_GSM_DL";
  case AIF2FL_TSTAMP_FORMAT_NORM_TS:
    return "AIF2FL_TSTAMP_FORMAT_NORM_TS";
  case AIF2FL_TSTAMP_FORMAT_NO_TS:
    return "AIF2FL_TSTAMP_FORMAT_NO_TS";
  case AIF2FL_TSTAMP_FORMAT_ROUTE_CHECK:
    return "AIF2FL_TSTAMP_FORMAT_ROUTE_CHECK";
  default:
    return "Illegal Enumeration Value";
  }
}

/* Structure dumping functions.  */
void dump_Aif2Fl_AdCommonSetup (FILE *output, Aif2Fl_AdCommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AdCommonSetup  */\n");
  fprintf(output, "  EgrGlobalDioEnable = %d\n", value->EgrGlobalDioEnable);
  fprintf(output, "  EgrGlobalEnable = %d\n", value->EgrGlobalEnable);
  fprintf(output, "  EgrPriority = %s\n", dump_Aif2Fl_AdEgrPriority(value->EgrPriority));
  fprintf(output, "  FailMode = %s\n", dump_Aif2Fl_AdFailMode(value->FailMode));
  fprintf(output, "  IngrEopCount = %d\n", value->IngrEopCount);
  fprintf(output, "  IngrGlobalDioEnable = %d\n", value->IngrGlobalDioEnable);
  fprintf(output, "  IngrGlobalEnable = %d\n", value->IngrGlobalEnable);
  fprintf(output, "  IngrPriority = %s\n", dump_Aif2Fl_AdIngrPriority(value->IngrPriority));
  fprintf(output, "  Tx_QueManager = %d\n", value->Tx_QueManager);
  fprintf(output, "  Tx_QueNum = %d\n", value->Tx_QueNum);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AdDioEngine (FILE *output, Aif2Fl_AdDioEngine *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AdDioEngine  */\n");
  fprintf(output, "  BcnTableSelect = %s\n", dump_Aif2Fl_AdBcnTable(value->BcnTableSelect));
  fprintf(output, "  DBCN[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    DBCN[%d] = %d\n", i, value->DBCN[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  DmaBaseAddr = %d\n", value->DmaBaseAddr);
  fprintf(output, "  DmaBlockAddrStride = %d\n", value->DmaBlockAddrStride);
  fprintf(output, "  DmaBurstAddrStride = %d\n", value->DmaBurstAddrStride);
  fprintf(output, "  DmaBurstLen = %s\n", dump_Aif2Fl_AdNumQWord(value->DmaBurstLen));
  fprintf(output, "  DmaNumBlock = %d\n", value->DmaNumBlock);
  fprintf(output, "  NumAxC = %d\n", value->NumAxC);
  fprintf(output, "  NumQuadWord = %s\n", dump_Aif2Fl_AdNumQWord(value->NumQuadWord));
  fprintf(output, "  bEnDmaChannel = %d\n", value->bEnDmaChannel);
  fprintf(output, "  bEnEgressRsaFormat = %d\n", value->bEnEgressRsaFormat);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AdDioSetup (FILE *output, Aif2Fl_AdDioSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AdDioSetup  */\n");
  fprintf(output, "  EgrDioEngine[3] = [\n");
  for(i=0; i<3; i++) {
	/* Check added manually.  */
	if (value->EgrDioEngineEnable[i]) {
		fprintf(output, "    EgrDioEngine[%d] = ", i);
		dump_Aif2Fl_AdDioEngine(output, &(value->EgrDioEngine[i]));
	} else {
		fprintf(output, "    EgrDioEngine[%d] = DISABLED\n", i);
	}
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  EgrDioEngineEnable[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    EgrDioEngineEnable[%d] = %d\n", i, value->EgrDioEngineEnable[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  IngrDioEngine[3] = [\n");
  for(i=0; i<3; i++) {
	/* Check added manually.  */
	if (value->IngrDioEngineEnable[i]) {
		fprintf(output, "    IngrDioEngine[%d] = ", i);
		dump_Aif2Fl_AdDioEngine(output, &(value->IngrDioEngine[i]));
	} else {
	    fprintf(output, "    IngrDioEngine[%d] = DISABLED\n", i);
	}
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  IngrDioEngineEnable[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    IngrDioEngineEnable[%d] = %d\n", i, value->IngrDioEngineEnable[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtCaptRadt (FILE *output, Aif2Fl_AtCaptRadt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtCaptRadt  */\n");
  fprintf(output, "  clock = %d\n", value->clock);
  fprintf(output, "  frame = %d\n", value->frame);
  fprintf(output, "  symbol = %d\n", value->symbol);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtCommonSetup (FILE *output, Aif2Fl_AtCommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtCommonSetup  */\n");
    fprintf(output, "  AtGsmTcount = ");
    dump_Aif2Fl_AtGsmTCount(output, &(value->AtGsmTcount));
    fprintf(output, "  AtInit = ");
    dump_Aif2Fl_AtInitObj(output, &(value->AtInit));
    fprintf(output, "  AtTerminalCount = ");
    dump_Aif2Fl_AtTcObj(output, &(value->AtTerminalCount));
  fprintf(output, "  AutoResyncMode = %s\n", dump_Aif2Fl_AtReSyncMode(value->AutoResyncMode));
  fprintf(output, "  CrcFlip = %s\n", dump_Aif2Fl_AtCrcFlip(value->CrcFlip));
  fprintf(output, "  CrcInitOnes = %s\n", dump_Aif2Fl_AtCrcInitOnes(value->CrcInitOnes));
  fprintf(output, "  CrcInvert = %s\n", dump_Aif2Fl_AtCrcInvert(value->CrcInvert));
  fprintf(output, "  CrcMode = %s\n", dump_Aif2Fl_AtCrcUse(value->CrcMode));
  fprintf(output, "  PhySyncSel = %s\n", dump_Aif2Fl_AtSyncSource(value->PhySyncSel));
  fprintf(output, "  PhytCompValue = %d\n", value->PhytCompValue);
  fprintf(output, "  RadSyncSel = %s\n", dump_Aif2Fl_AtSyncSource(value->RadSyncSel));
  fprintf(output, "  Rp1PhytFrameLoad = %d\n", value->Rp1PhytFrameLoad);
  fprintf(output, "  Rp1RadtFrameLoad = %d\n", value->Rp1RadtFrameLoad);
  fprintf(output, "  Rp1Type = %s\n", dump_Aif2Fl_AtRp1TypeField(value->Rp1Type));
  fprintf(output, "  SyncMode = %s\n", dump_Aif2Fl_AtSyncMode(value->SyncMode));
  fprintf(output, "  SyncSampleWindow = %d\n", value->SyncSampleWindow);
  fprintf(output, "  WcdmaDivTC = %d\n", value->WcdmaDivTC);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtCountObj (FILE *output, Aif2Fl_AtCountObj *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtCountObj  */\n");
  fprintf(output, "  ClockNum = %d\n", value->ClockNum);
  fprintf(output, "  FcbMinusOne = %d\n", value->FcbMinusOne);
  fprintf(output, "  FrameLsbNum = %d\n", value->FrameLsbNum);
  fprintf(output, "  FrameMsbNum = %d\n", value->FrameMsbNum);
  fprintf(output, "  LutIndexNum = %d\n", value->LutIndexNum);
  fprintf(output, "  SymbolNum = %d\n", value->SymbolNum);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtEvent (FILE *output, Aif2Fl_AtEvent *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtEvent  */\n");
  fprintf(output, "  DioFrameEventOffset = %d\n", value->DioFrameEventOffset);
  fprintf(output, "  DioFrameStrobeSel = %s\n", dump_Aif2Fl_AtEvtStrobe(value->DioFrameStrobeSel));
  fprintf(output, "  EventMaskLsb = %d\n", value->EventMaskLsb);
  fprintf(output, "  EventMaskMsb = %d\n", value->EventMaskMsb);
  fprintf(output, "  EventModulo = %d\n", value->EventModulo);
  fprintf(output, "  EventOffset = %d\n", value->EventOffset);
  fprintf(output, "  EventSelect = %s\n", dump_Aif2Fl_AtEventIndex(value->EventSelect));
  fprintf(output, "  EvtStrobeSel = %s\n", dump_Aif2Fl_AtEvtStrobe(value->EvtStrobeSel));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtEventSetup (FILE *output, Aif2Fl_AtEventSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtEventSetup  */\n");
  fprintf(output, "  AtEgrDioEvent[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    AtEgrDioEvent[%d] = ", i);
    dump_Aif2Fl_AtEvent(output, &(value->AtEgrDioEvent[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  AtIngrDioEvent[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    AtIngrDioEvent[%d] = ", i);
    dump_Aif2Fl_AtEvent(output, &(value->AtIngrDioEvent[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  AtRadEvent[11] = [\n");
  for(i=0; i<11; i++) {
    fprintf(output, "    AtRadEvent[%d] = ", i);
    dump_Aif2Fl_AtEvent(output, &(value->AtRadEvent[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableEgrDioEvent[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    bEnableEgrDioEvent[%d] = %d\n", i, value->bEnableEgrDioEvent[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableIngrDioEvent[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    bEnableIngrDioEvent[%d] = %d\n", i, value->bEnableIngrDioEvent[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableRadEvent[11] = [\n");
  for(i=0; i<11; i++) {
    fprintf(output, "    bEnableRadEvent[%d] = %d\n", i, value->bEnableRadEvent[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtGsmTCount (FILE *output, Aif2Fl_AtGsmTCount *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtGsmTCount  */\n");
  fprintf(output, "  t1 = %d\n", value->t1);
  fprintf(output, "  t2 = %d\n", value->t2);
  fprintf(output, "  t3 = %d\n", value->t3);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtInitObj (FILE *output, Aif2Fl_AtInitObj *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtInitObj  */\n");
  if (value->pDlRadTimerInit != NULL) {
    fprintf(output, "  pDlRadTimerInit = ");
    dump_Aif2Fl_AtCountObj(output, value->pDlRadTimerInit);
  } else {
    fprintf(output, "  pDlRadTimerInit = NULL\n");
  }
  if (value->pPhyTimerInit != NULL) {
    fprintf(output, "  pPhyTimerInit = ");
    dump_Aif2Fl_AtCountObj(output, value->pPhyTimerInit);
  } else {
    fprintf(output, "  pPhyTimerInit = NULL\n");
  }
  if (value->pRadTimerInit != NULL) {
    fprintf(output, "  pRadTimerInit = ");
    dump_Aif2Fl_AtCountObj(output, value->pRadTimerInit);
  } else {
    fprintf(output, "  pRadTimerInit = NULL\n");
  }
  if (value->pUlRadTimerInit != NULL) {
    fprintf(output, "  pUlRadTimerInit = ");
    dump_Aif2Fl_AtCountObj(output, value->pUlRadTimerInit);
  } else {
    fprintf(output, "  pUlRadTimerInit = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtLinkSetup (FILE *output, Aif2Fl_AtLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtLinkSetup  */\n");
  fprintf(output, "  DeltaOffset = %d\n", value->DeltaOffset);
  fprintf(output, "  IsNegativeDelta = %d\n", value->IsNegativeDelta);
  fprintf(output, "  PE1Offset = %d\n", value->PE1Offset);
  fprintf(output, "  PE2Offset = %d\n", value->PE2Offset);
  fprintf(output, "  PiMax = %d\n", value->PiMax);
  fprintf(output, "  PiMin = %d\n", value->PiMin);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtTcObj (FILE *output, Aif2Fl_AtTcObj *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtTcObj  */\n");
  fprintf(output, "  RadClockCountTc[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    RadClockCountTc[%d] = %d\n", i, value->RadClockCountTc[i]);
  }
  fprintf(output, "  ]\n");
  if (value->pPhyTimerTc != NULL) {
    fprintf(output, "  pPhyTimerTc = ");
    dump_Aif2Fl_AtCountObj(output, value->pPhyTimerTc);
  } else {
    fprintf(output, "  pPhyTimerTc = NULL\n");
  }
  if (value->pRadTimerTc != NULL) {
    fprintf(output, "  pRadTimerTc = ");
    dump_Aif2Fl_AtCountObj(output, value->pRadTimerTc);
  } else {
    fprintf(output, "  pRadTimerTc = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Aif2Fl_AtWcdmaCount (FILE *output, Aif2Fl_AtWcdmaCount *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_AtWcdmaCount  */\n");
  fprintf(output, "  chip = %d\n", value->chip);
  fprintf(output, "  frame = %d\n", value->frame);
  fprintf(output, "  slot = %d\n", value->slot);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_BaseAddress (FILE *output, Aif2Fl_BaseAddress *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_BaseAddress  */\n");
  fprintf(output, "}\n");
}

void dump_Aif2Fl_CommonLinkSetup (FILE *output, Aif2Fl_CommonLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_CommonLinkSetup  */\n");
  fprintf(output, "  EgrDataWidth = %s\n", dump_Aif2Fl_DataWidth(value->EgrDataWidth));
  fprintf(output, "  IngrDataWidth = %s\n", dump_Aif2Fl_DataWidth(value->IngrDataWidth));
  fprintf(output, "  linkProtocol = %s\n", dump_Aif2Fl_LinkProtocol(value->linkProtocol));
  fprintf(output, "  linkRate = %s\n", dump_Aif2Fl_LinkRate(value->linkRate));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_CommonSetup (FILE *output, Aif2Fl_CommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_CommonSetup  */\n");
  if (value->pAdCommonSetup != NULL) {
    fprintf(output, "  pAdCommonSetup = ");
    dump_Aif2Fl_AdCommonSetup(output, value->pAdCommonSetup);
  } else {
    fprintf(output, "  pAdCommonSetup = NULL\n");
  }
  if (value->pAdDioSetup != NULL) {
    fprintf(output, "  pAdDioSetup = ");
    dump_Aif2Fl_AdDioSetup(output, value->pAdDioSetup);
  } else {
    fprintf(output, "  pAdDioSetup = NULL\n");
  }
  if (value->pAtCommonSetup != NULL) {
    fprintf(output, "  pAtCommonSetup = ");
    dump_Aif2Fl_AtCommonSetup(output, value->pAtCommonSetup);
  } else {
    fprintf(output, "  pAtCommonSetup = NULL\n");
  }
  if (value->pAtEventSetup != NULL) {
    fprintf(output, "  pAtEventSetup = ");
    dump_Aif2Fl_AtEventSetup(output, value->pAtEventSetup);
  } else {
    fprintf(output, "  pAtEventSetup = NULL\n");
  }
  if (value->pEgrDbSetup != NULL) {
    fprintf(output, "  pEgrDbSetup = ");
    dump_Aif2Fl_EgrDbSetup(output, value->pEgrDbSetup);
  } else {
    fprintf(output, "  pEgrDbSetup = NULL\n");
  }
  if (value->pIngrDbSetup != NULL) {
    fprintf(output, "  pIngrDbSetup = ");
    dump_Aif2Fl_IngrDbSetup(output, value->pIngrDbSetup);
  } else {
    fprintf(output, "  pIngrDbSetup = NULL\n");
  }
  if (value->pPdCommonSetup != NULL) {
    fprintf(output, "  pPdCommonSetup = ");
    dump_Aif2Fl_PdCommonSetup(output, value->pPdCommonSetup);
  } else {
    fprintf(output, "  pPdCommonSetup = NULL\n");
  }
  if (value->pPeCommonSetup != NULL) {
    fprintf(output, "  pPeCommonSetup = ");
    dump_Aif2Fl_PeCommonSetup(output, value->pPeCommonSetup);
  } else {
    fprintf(output, "  pPeCommonSetup = NULL\n");
  }
  if (value->pSdCommonSetup != NULL) {
    fprintf(output, "  pSdCommonSetup = ");
    dump_Aif2Fl_SdCommonSetup(output, value->pSdCommonSetup);
  } else {
    fprintf(output, "  pSdCommonSetup = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Aif2Fl_CpriCwLut (FILE *output, Aif2Fl_CpriCwLut *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_CpriCwLut  */\n");
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
  fprintf(output, "  CpriCwChannel = %d\n", value->CpriCwChannel);
  fprintf(output, "  bEnableCpriCw = %d\n", value->bEnableCpriCw);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_CpriTmSetup (FILE *output, Aif2Fl_CpriTmSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_CpriTmSetup  */\n");
  fprintf(output, "  L1InbandEn = %d\n", value->L1InbandEn);
  fprintf(output, "  RmLinkLofError = %s\n", dump_Aif2Fl_LinkIndex(value->RmLinkLofError));
  fprintf(output, "  RmLinkLofRx = %s\n", dump_Aif2Fl_LinkIndex(value->RmLinkLofRx));
  fprintf(output, "  RmLinkLosError = %s\n", dump_Aif2Fl_LinkIndex(value->RmLinkLosError));
  fprintf(output, "  RmLinkLosRx = %s\n", dump_Aif2Fl_LinkIndex(value->RmLinkLosRx));
  fprintf(output, "  RmLinkRaiRx = %s\n", dump_Aif2Fl_LinkIndex(value->RmLinkRaiRx));
  fprintf(output, "  TxPointerP = %d\n", value->TxPointerP);
  fprintf(output, "  TxProtocolVer = %d\n", value->TxProtocolVer);
  fprintf(output, "  TxStartup = %d\n", value->TxStartup);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_DbChannel (FILE *output, Aif2Fl_DbChannel *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_DbChannel  */\n");
  fprintf(output, "  BaseAddress = %d\n", value->BaseAddress);
  fprintf(output, "  BufDepth = %s\n", dump_Aif2Fl_DbFifoDepth(value->BufDepth));
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
  fprintf(output, "  DataSwap = %s\n", dump_Aif2Fl_DbDataSwap(value->DataSwap));
  fprintf(output, "  EgressDioOffset = %d\n", value->EgressDioOffset);
  fprintf(output, "  IQOrder = %s\n", dump_Aif2Fl_DbIqOrder(value->IQOrder));
  fprintf(output, "  PacketType = %d\n", value->PacketType);
  fprintf(output, "  bEnablePsData = %d\n", value->bEnablePsData);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_DbSideData (FILE *output, Aif2Fl_DbSideData *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_DbSideData  */\n");
  fprintf(output, "  ChannelId = %d\n", value->ChannelId);
  fprintf(output, "  DioAddress = %d\n", value->DioAddress);
  fprintf(output, "  Symbol = %d\n", value->Symbol);
  fprintf(output, "  bEnDioBufferRead = %d\n", value->bEnDioBufferRead);
  fprintf(output, "  bEnDioBufferWrite = %d\n", value->bEnDioBufferWrite);
  fprintf(output, "  bEnFifoBufferWrite = %d\n", value->bEnFifoBufferWrite);
  fprintf(output, "  bEop = %d\n", value->bEop);
  fprintf(output, "  bSop = %d\n", value->bSop);
  fprintf(output, "  xcnt = %d\n", value->xcnt);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_DualBitMap (FILE *output, Aif2Fl_DualBitMap *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_DualBitMap  */\n");
  fprintf(output, "  Dbm1Map[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    Dbm1Map[%d] = %d\n", i, value->Dbm1Map[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  Dbm1Mult = %d\n", value->Dbm1Mult);
  fprintf(output, "  Dbm1Size = %d\n", value->Dbm1Size);
  fprintf(output, "  Dbm2Map[3] = [\n");
  for(i=0; i<3; i++) {
    fprintf(output, "    Dbm2Map[%d] = %d\n", i, value->Dbm2Map[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  Dbm2Size = %d\n", value->Dbm2Size);
  fprintf(output, "  DbmX = %d\n", value->DbmX);
  fprintf(output, "  DbmXBubble = %d\n", value->DbmXBubble);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeAdInt (FILE *output, Aif2Fl_EeAdInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeAdInt  */\n");
  fprintf(output, "  ad_ee_e_cd_sch_err = %d\n", value->ad_ee_e_cd_sch_err);
  fprintf(output, "  ad_ee_e_dma_0_err = %d\n", value->ad_ee_e_dma_0_err);
  fprintf(output, "  ad_ee_e_dma_1_err = %d\n", value->ad_ee_e_dma_1_err);
  fprintf(output, "  ad_ee_e_dma_2_err = %d\n", value->ad_ee_e_dma_2_err);
  fprintf(output, "  ad_ee_i_cd_data_err = %d\n", value->ad_ee_i_cd_data_err);
  fprintf(output, "  ad_ee_i_dma_0_err = %d\n", value->ad_ee_i_dma_0_err);
  fprintf(output, "  ad_ee_i_dma_1_err = %d\n", value->ad_ee_i_dma_1_err);
  fprintf(output, "  ad_ee_i_dma_2_err = %d\n", value->ad_ee_i_dma_2_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeAif2Int (FILE *output, Aif2Fl_EeAif2Int *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeAif2Int  */\n");
  fprintf(output, "  Alarm_intr = %d\n", value->Alarm_intr);
  fprintf(output, "  Cdma_intr = %d\n", value->Cdma_intr);
  fprintf(output, "  Error_intr = %d\n", value->Error_intr);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeAif2Run (FILE *output, Aif2Fl_EeAif2Run *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeAif2Run  */\n");
  fprintf(output, "  aif2_global_run = %d\n", value->aif2_global_run);
  fprintf(output, "  aif2_phy_run = %d\n", value->aif2_phy_run);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeAtInt (FILE *output, Aif2Fl_EeAtInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeAtInt  */\n");
  fprintf(output, "  at_ee_phyt_sync_err = %d\n", value->at_ee_phyt_sync_err);
  fprintf(output, "  at_ee_pi0_err = %d\n", value->at_ee_pi0_err);
  fprintf(output, "  at_ee_pi1_err = %d\n", value->at_ee_pi1_err);
  fprintf(output, "  at_ee_pi2_err = %d\n", value->at_ee_pi2_err);
  fprintf(output, "  at_ee_pi3_err = %d\n", value->at_ee_pi3_err);
  fprintf(output, "  at_ee_pi4_err = %d\n", value->at_ee_pi4_err);
  fprintf(output, "  at_ee_pi5_err = %d\n", value->at_ee_pi5_err);
  fprintf(output, "  at_ee_radt_sync_err = %d\n", value->at_ee_radt_sync_err);
  fprintf(output, "  at_ee_rp1_bit_width_err = %d\n", value->at_ee_rp1_bit_width_err);
  fprintf(output, "  at_ee_rp1_crc_err = %d\n", value->at_ee_rp1_crc_err);
  fprintf(output, "  at_ee_rp1_rp3_err = %d\n", value->at_ee_rp1_rp3_err);
  fprintf(output, "  at_ee_rp1_sys_err = %d\n", value->at_ee_rp1_sys_err);
  fprintf(output, "  at_ee_rp1_type_rp3_rcvd_err = %d\n", value->at_ee_rp1_type_rp3_rcvd_err);
  fprintf(output, "  at_ee_rp1_type_rsvd_err = %d\n", value->at_ee_rp1_type_rsvd_err);
  fprintf(output, "  at_ee_rp1_type_spare_err = %d\n", value->at_ee_rp1_type_spare_err);
  fprintf(output, "  at_ee_rp1_type_sys_rcvd_err = %d\n", value->at_ee_rp1_type_sys_rcvd_err);
  fprintf(output, "  at_ee_rp1_type_tod_rcvd_err = %d\n", value->at_ee_rp1_type_tod_rcvd_err);
  fprintf(output, "  at_ee_rp1_type_unsel_err = %d\n", value->at_ee_rp1_type_unsel_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeCdInt (FILE *output, Aif2Fl_EeCdInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeCdInt  */\n");
  fprintf(output, "  cd_ee_mop_desc_starve_err = %d\n", value->cd_ee_mop_desc_starve_err);
  fprintf(output, "  cd_ee_sop_desc_starve_err = %d\n", value->cd_ee_sop_desc_starve_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeDbInt (FILE *output, Aif2Fl_EeDbInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeDbInt  */\n");
  fprintf(output, "  db_ee_e_cd_data_err = %d\n", value->db_ee_e_cd_data_err);
  fprintf(output, "  db_ee_e_cd_data_type_err = %d\n", value->db_ee_e_cd_data_type_err);
  fprintf(output, "  db_ee_e_ps_axc_err = %d\n", value->db_ee_e_ps_axc_err);
  fprintf(output, "  db_ee_i_fifo_ovfl_err = %d\n", value->db_ee_i_fifo_ovfl_err);
  fprintf(output, "  db_ee_i_pd2db_full_err = %d\n", value->db_ee_i_pd2db_full_err);
  fprintf(output, "  db_ee_i_token_ovfl_err = %d\n", value->db_ee_i_token_ovfl_err);
  fprintf(output, "  db_ee_i_trc_ram_ovfl_err = %d\n", value->db_ee_i_trc_ram_ovfl_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeLinkAInt (FILE *output, Aif2Fl_EeLinkAInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeLinkAInt  */\n");
  fprintf(output, "  rm_ee_block_bndry_det_err = %d\n", value->rm_ee_block_bndry_det_err);
  fprintf(output, "  rm_ee_frame_bndry_det_err = %d\n", value->rm_ee_frame_bndry_det_err);
  fprintf(output, "  rm_ee_hfnsync_state_err = %d\n", value->rm_ee_hfnsync_state_err);
  fprintf(output, "  rm_ee_k30p7_det_err = %d\n", value->rm_ee_k30p7_det_err);
  fprintf(output, "  rm_ee_lcv_det_err = %d\n", value->rm_ee_lcv_det_err);
  fprintf(output, "  rm_ee_loc_det_err = %d\n", value->rm_ee_loc_det_err);
  fprintf(output, "  rm_ee_lof_err = %d\n", value->rm_ee_lof_err);
  fprintf(output, "  rm_ee_lof_state_err = %d\n", value->rm_ee_lof_state_err);
  fprintf(output, "  rm_ee_los_err = %d\n", value->rm_ee_los_err);
  fprintf(output, "  rm_ee_missing_k28p5_err = %d\n", value->rm_ee_missing_k28p5_err);
  fprintf(output, "  rm_ee_missing_k28p7_err = %d\n", value->rm_ee_missing_k28p7_err);
  fprintf(output, "  rm_ee_num_los_det_err = %d\n", value->rm_ee_num_los_det_err);
  fprintf(output, "  rm_ee_rcvd_lof_err = %d\n", value->rm_ee_rcvd_lof_err);
  fprintf(output, "  rm_ee_rcvd_los_err = %d\n", value->rm_ee_rcvd_los_err);
  fprintf(output, "  rm_ee_rcvd_rai_err = %d\n", value->rm_ee_rcvd_rai_err);
  fprintf(output, "  rm_ee_rcvd_sdi_err = %d\n", value->rm_ee_rcvd_sdi_err);
  fprintf(output, "  rm_ee_rx_fifo_ovf_err = %d\n", value->rm_ee_rx_fifo_ovf_err);
  fprintf(output, "  rm_ee_sync_status_change_err = %d\n", value->rm_ee_sync_status_change_err);
  fprintf(output, "  tm_ee_fifo_starve_err = %d\n", value->tm_ee_fifo_starve_err);
  fprintf(output, "  tm_ee_frm_misalign_err = %d\n", value->tm_ee_frm_misalign_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeLinkBInt (FILE *output, Aif2Fl_EeLinkBInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeLinkBInt  */\n");
  fprintf(output, "  pd_ee_axc_fail_err = %d\n", value->pd_ee_axc_fail_err);
  fprintf(output, "  pd_ee_cpri_frame_err = %d\n", value->pd_ee_cpri_frame_err);
  fprintf(output, "  pd_ee_crc_err = %d\n", value->pd_ee_crc_err);
  fprintf(output, "  pd_ee_eop_err = %d\n", value->pd_ee_eop_err);
  fprintf(output, "  pd_ee_obsai_frm_err = %d\n", value->pd_ee_obsai_frm_err);
  fprintf(output, "  pd_ee_sop_err = %d\n", value->pd_ee_sop_err);
  fprintf(output, "  pd_ee_wr2db_err = %d\n", value->pd_ee_wr2db_err);
  fprintf(output, "  pe_ee_db_starve_err = %d\n", value->pe_ee_db_starve_err);
  fprintf(output, "  pe_ee_mf_fifo_overflow_err = %d\n", value->pe_ee_mf_fifo_overflow_err);
  fprintf(output, "  pe_ee_mf_fifo_underflow_err = %d\n", value->pe_ee_mf_fifo_underflow_err);
  fprintf(output, "  pe_ee_modrule_err = %d\n", value->pe_ee_modrule_err);
  fprintf(output, "  pe_ee_pkt_starve_err = %d\n", value->pe_ee_pkt_starve_err);
  fprintf(output, "  pe_ee_rt_if_err = %d\n", value->pe_ee_rt_if_err);
  fprintf(output, "  pe_ee_sym_err = %d\n", value->pe_ee_sym_err);
  fprintf(output, "  rt_ee_em_err = %d\n", value->rt_ee_em_err);
  fprintf(output, "  rt_ee_frm_err = %d\n", value->rt_ee_frm_err);
  fprintf(output, "  rt_ee_hdr_err = %d\n", value->rt_ee_hdr_err);
  fprintf(output, "  rt_ee_ovfl_err = %d\n", value->rt_ee_ovfl_err);
  fprintf(output, "  rt_ee_unfl_err = %d\n", value->rt_ee_unfl_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeOrigin (FILE *output, Aif2Fl_EeOrigin *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeOrigin  */\n");
  fprintf(output, "  ad_en_sts = %d\n", value->ad_en_sts);
  fprintf(output, "  at_en_sts = %d\n", value->at_en_sts);
  fprintf(output, "  cd_en_sts = %d\n", value->cd_en_sts);
  fprintf(output, "  db_en_sts = %d\n", value->db_en_sts);
  fprintf(output, "  lk_en_sts_a0 = %d\n", value->lk_en_sts_a0);
  fprintf(output, "  lk_en_sts_a1 = %d\n", value->lk_en_sts_a1);
  fprintf(output, "  lk_en_sts_a2 = %d\n", value->lk_en_sts_a2);
  fprintf(output, "  lk_en_sts_a3 = %d\n", value->lk_en_sts_a3);
  fprintf(output, "  lk_en_sts_a4 = %d\n", value->lk_en_sts_a4);
  fprintf(output, "  lk_en_sts_a5 = %d\n", value->lk_en_sts_a5);
  fprintf(output, "  lk_en_sts_b0 = %d\n", value->lk_en_sts_b0);
  fprintf(output, "  lk_en_sts_b1 = %d\n", value->lk_en_sts_b1);
  fprintf(output, "  lk_en_sts_b2 = %d\n", value->lk_en_sts_b2);
  fprintf(output, "  lk_en_sts_b3 = %d\n", value->lk_en_sts_b3);
  fprintf(output, "  lk_en_sts_b4 = %d\n", value->lk_en_sts_b4);
  fprintf(output, "  lk_en_sts_b5 = %d\n", value->lk_en_sts_b5);
  fprintf(output, "  sd_en_sts = %d\n", value->sd_en_sts);
  fprintf(output, "  vc_en_sts = %d\n", value->vc_en_sts);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EePdInt (FILE *output, Aif2Fl_EePdInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EePdInt  */\n");
  fprintf(output, "  pd_ee_ts_wdog_err = %d\n", value->pd_ee_ts_wdog_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EePeInt (FILE *output, Aif2Fl_EePeInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EePeInt  */\n");
  fprintf(output, "  pe_ee_dat_req_ovfl_err = %d\n", value->pe_ee_dat_req_ovfl_err);
  fprintf(output, "  pe_ee_rd2db_err = %d\n", value->pe_ee_rd2db_err);
  fprintf(output, "  pe_ee_token_req_ovfl_err = %d\n", value->pe_ee_token_req_ovfl_err);
  fprintf(output, "  pe_ee_token_wr_err = %d\n", value->pe_ee_token_wr_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeSdInt (FILE *output, Aif2Fl_EeSdInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeSdInt  */\n");
  fprintf(output, "  sd_ee_stspll_b4_err = %d\n", value->sd_ee_stspll_b4_err);
  fprintf(output, "  sd_ee_stspll_b8_err = %d\n", value->sd_ee_stspll_b8_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EeVcInt (FILE *output, Aif2Fl_EeVcInt *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EeVcInt  */\n");
  fprintf(output, "  vc_ee_vbus_err = %d\n", value->vc_ee_vbus_err);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_EgrDbSetup (FILE *output, Aif2Fl_EgrDbSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_EgrDbSetup  */\n");
  fprintf(output, "  DioBufferLen = %s\n", dump_Aif2Fl_DioLen(value->DioBufferLen));
  fprintf(output, "  EgrDbChannel[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    EgrDbChannel[%d] = ", i);
    dump_Aif2Fl_DbChannel(output, &(value->EgrDbChannel[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PmControl = %s\n", dump_Aif2Fl_DbPmControl(value->PmControl));
  fprintf(output, "  bEnableChannel[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    bEnableChannel[%d] = %d\n", i, value->bEnableChannel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableEgrDb = %d\n", value->bEnableEgrDb);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_FrameCounter (FILE *output, Aif2Fl_FrameCounter *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_FrameCounter  */\n");
  fprintf(output, "  FrameIndexSc = %d\n", value->FrameIndexSc);
  fprintf(output, "  FrameIndexTc = %d\n", value->FrameIndexTc);
  fprintf(output, "  FrameSymbolTc = %d\n", value->FrameSymbolTc);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_GlobalSetup (FILE *output, Aif2Fl_GlobalSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_GlobalSetup  */\n");
  fprintf(output, "  ActiveLink[6] = [\n");
  for(i=0; i<6; i++) {
    fprintf(output, "    ActiveLink[%d] = %d\n", i, value->ActiveLink[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  frameMode = %s\n", dump_Aif2Fl_FrameMode(value->frameMode));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_IngrDbSetup (FILE *output, Aif2Fl_IngrDbSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_IngrDbSetup  */\n");
  fprintf(output, "  DioBufferLen = %s\n", dump_Aif2Fl_DioLen(value->DioBufferLen));
  fprintf(output, "  IngrDbChannel[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    IngrDbChannel[%d] = ", i);
    dump_Aif2Fl_DbChannel(output, &(value->IngrDbChannel[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChannel[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    bEnableChannel[%d] = %d\n", i, value->bEnableChannel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableIngrDb = %d\n", value->bEnableIngrDb);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_LinkSetup (FILE *output, Aif2Fl_LinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_LinkSetup  */\n");
  fprintf(output, "  linkIndex = %s\n", dump_Aif2Fl_LinkIndex(value->linkIndex));
  if (value->pAtLinkSetup != NULL) {
    fprintf(output, "  pAtLinkSetup = ");
    dump_Aif2Fl_AtLinkSetup(output, value->pAtLinkSetup);
  } else {
    fprintf(output, "  pAtLinkSetup = NULL\n");
  }
  if (value->pComLinkSetup != NULL) {
    fprintf(output, "  pComLinkSetup = ");
    dump_Aif2Fl_CommonLinkSetup(output, value->pComLinkSetup);
  } else {
    fprintf(output, "  pComLinkSetup = NULL\n");
  }
  if (value->pPdLinkSetup != NULL) {
    fprintf(output, "  pPdLinkSetup = ");
    dump_Aif2Fl_PdLinkSetup(output, value->pPdLinkSetup);
  } else {
    fprintf(output, "  pPdLinkSetup = NULL\n");
  }
  if (value->pPeLinkSetup != NULL) {
    fprintf(output, "  pPeLinkSetup = ");
    dump_Aif2Fl_PeLinkSetup(output, value->pPeLinkSetup);
  } else {
    fprintf(output, "  pPeLinkSetup = NULL\n");
  }
  if (value->pRmLinkSetup != NULL) {
    fprintf(output, "  pRmLinkSetup = ");
    dump_Aif2Fl_RmLinkSetup(output, value->pRmLinkSetup);
  } else {
    fprintf(output, "  pRmLinkSetup = NULL\n");
  }
  if (value->pRtLinkSetup != NULL) {
    fprintf(output, "  pRtLinkSetup = ");
    dump_Aif2Fl_RtLinkSetup(output, value->pRtLinkSetup);
  } else {
    fprintf(output, "  pRtLinkSetup = NULL\n");
  }
  if (value->pSdLinkSetup != NULL) {
    fprintf(output, "  pSdLinkSetup = ");
    dump_Aif2Fl_SdLinkSetup(output, value->pSdLinkSetup);
  } else {
    fprintf(output, "  pSdLinkSetup = NULL\n");
  }
  if (value->pTmLinkSetup != NULL) {
    fprintf(output, "  pTmLinkSetup = ");
    dump_Aif2Fl_TmLinkSetup(output, value->pTmLinkSetup);
  } else {
    fprintf(output, "  pTmLinkSetup = NULL\n");
  }
  fprintf(output, "}\n");
}

void dump_Aif2Fl_ModuloTc (FILE *output, Aif2Fl_ModuloTc *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_ModuloTc  */\n");
  fprintf(output, "  RuleIndex = %d\n", value->RuleIndex);
  fprintf(output, "  RuleLink = %d\n", value->RuleLink);
  fprintf(output, "  RuleModulo = %d\n", value->RuleModulo);
  fprintf(output, "  bEnableRule = %d\n", value->bEnableRule);
  fprintf(output, "  bRuleObsaiCtlMsg = %d\n", value->bRuleObsaiCtlMsg);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_Obj (FILE *output, Aif2Fl_Obj *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_Obj  */\n");
  fprintf(output, "  arg_dioEngine = %s\n", dump_Aif2Fl_DioEngineIndex(value->arg_dioEngine));
  fprintf(output, "  arg_link = %s\n", dump_Aif2Fl_LinkIndex(value->arg_link));
  fprintf(output, "  ee_arg = %s\n", dump_Aif2Fl_EeArgIndex(value->ee_arg));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_Param (FILE *output, Aif2Fl_Param *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_Param  */\n");
    fprintf(output, "  gSetup = ");
    dump_Aif2Fl_GlobalSetup(output, &(value->gSetup));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdChConfig (FILE *output, Aif2Fl_PdChConfig *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdChConfig  */\n");
  fprintf(output, "  DataFormat = %s\n", dump_Aif2Fl_LinkDataType(value->DataFormat));
  fprintf(output, "  bChannelEn = %d\n", value->bChannelEn);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdChConfig1 (FILE *output, Aif2Fl_PdChConfig1 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdChConfig1  */\n");
  fprintf(output, "  DataFormat = %s\n", dump_Aif2Fl_GSMDataFormat(value->DataFormat));
  fprintf(output, "  DioOffset = %d\n", value->DioOffset);
  fprintf(output, "  FrameCounter = %d\n", value->FrameCounter);
  fprintf(output, "  TddEnable = %d\n", value->TddEnable);
  fprintf(output, "  bTsWatchDogEn = %d\n", value->bTsWatchDogEn);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdChannelConfig (FILE *output, Aif2Fl_PdChannelConfig *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdChannelConfig  */\n");
  fprintf(output, "  AxCOffset = %d\n", value->AxCOffset);
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
    fprintf(output, "  PdChConfig = ");
    dump_Aif2Fl_PdChConfig(output, &(value->PdChConfig));
    fprintf(output, "  PdChConfig1 = ");
    dump_Aif2Fl_PdChConfig1(output, &(value->PdChConfig1));
  fprintf(output, "  PdFrameMsgTc = %d\n", value->PdFrameMsgTc);
    fprintf(output, "  PdRoute = ");
    dump_Aif2Fl_PdRoute(output, &(value->PdRoute));
  fprintf(output, "  TddEnable1 = %d\n", value->TddEnable1);
  fprintf(output, "  TddEnable2 = %d\n", value->TddEnable2);
  fprintf(output, "  TddEnable3 = %d\n", value->TddEnable3);
  fprintf(output, "  TddEnable4 = %d\n", value->TddEnable4);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdCommonSetup (FILE *output, Aif2Fl_PdCommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdCommonSetup  */\n");
  fprintf(output, "  AxCOffset[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    AxCOffset[%d] = %d\n", i, value->AxCOffset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  AxCOffsetWin = %d\n", value->AxCOffsetWin);
  fprintf(output, "  PdChConfig[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PdChConfig[%d] = ", i);
    dump_Aif2Fl_PdChConfig(output, &(value->PdChConfig[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PdChConfig1[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PdChConfig1[%d] = ", i);
    dump_Aif2Fl_PdChConfig1(output, &(value->PdChConfig1[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PdCppiDioSel = %s\n", dump_Aif2Fl_CppiDio(value->PdCppiDioSel));
  fprintf(output, "  PdFrameMsgTc[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PdFrameMsgTc[%d] = %d\n", i, value->PdFrameMsgTc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PdFrameTC[6] = [\n");
  for(i=0; i<6; i++) {
    fprintf(output, "    PdFrameTC[%d] = ", i);
    dump_Aif2Fl_FrameCounter(output, &(value->PdFrameTC[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PdGlobalEnable = %d\n", value->PdGlobalEnable);
  fprintf(output, "  PdRadtTC = %d\n", value->PdRadtTC);
  fprintf(output, "  PdRoute[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PdRoute[%d] = ", i);
    dump_Aif2Fl_PdRoute(output, &(value->PdRoute[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TddEnable1[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    TddEnable1[%d] = %d\n", i, value->TddEnable1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TddEnable2[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    TddEnable2[%d] = %d\n", i, value->TddEnable2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TddEnable3[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    TddEnable3[%d] = %d\n", i, value->TddEnable3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TddEnable4[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    TddEnable4[%d] = %d\n", i, value->TddEnable4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TsWatchDogCount = %d\n", value->TsWatchDogCount);
  fprintf(output, "  WatchDogEopAdd = %d\n", value->WatchDogEopAdd);
  fprintf(output, "  WatchDogReport = %s\n", dump_Aif2Fl_PdWatchDogReport(value->WatchDogReport));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdCpriIdLut (FILE *output, Aif2Fl_PdCpriIdLut *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdCpriIdLut  */\n");
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
  fprintf(output, "  Cpri8WordOffset = %d\n", value->Cpri8WordOffset);
  fprintf(output, "  CpriDmaCh = %d\n", value->CpriDmaCh);
  fprintf(output, "  bEnableCpriPkt = %d\n", value->bEnableCpriPkt);
  fprintf(output, "  bEnableCpriX = %d\n", value->bEnableCpriX);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdLinkSetup (FILE *output, Aif2Fl_PdLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdLinkSetup  */\n");
  fprintf(output, "  Cpri8WordOffset[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    Cpri8WordOffset[%d] = %d\n", i, value->Cpri8WordOffset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriAxCPack = %s\n", dump_Aif2Fl_CpriAxCPack(value->CpriAxCPack));
  fprintf(output, "  CpriCwChannel[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    CpriCwChannel[%d] = %d\n", i, value->CpriCwChannel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriCwNullDelimitor = %d\n", value->CpriCwNullDelimitor);
  fprintf(output, "  CpriCwPktDelimitor[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    CpriCwPktDelimitor[%d] = %s\n", i, dump_Aif2Fl_CpriCwPktDelim(value->CpriCwPktDelimitor[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriDmaCh[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    CpriDmaCh[%d] = %d\n", i, value->CpriDmaCh[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriEnetStrip = %d\n", value->CpriEnetStrip);
  fprintf(output, "  Crc8Poly = %d\n", value->Crc8Poly);
  fprintf(output, "  Crc8Seed = %d\n", value->Crc8Seed);
  fprintf(output, "  PdCpriCrcType[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    PdCpriCrcType[%d] = %s\n", i, dump_Aif2Fl_CrcLen(value->PdCpriCrcType[i]));
  }
  fprintf(output, "  ]\n");
    fprintf(output, "  PdCpriDualBitMap = ");
    dump_Aif2Fl_DualBitMap(output, &(value->PdCpriDualBitMap));
  fprintf(output, "  PdPackDmaCh[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    PdPackDmaCh[%d] = %d\n", i, value->PdPackDmaCh[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PdTypeLut[32] = [\n");
  for(i=0; i<32; i++) {
    fprintf(output, "    PdTypeLut[%d] = ", i);
    dump_Aif2Fl_PdTypeLut(output, &(value->PdTypeLut[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableCpriCrc[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    bEnableCpriCrc[%d] = %d\n", i, value->bEnableCpriCrc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableCpriCw[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    bEnableCpriCw[%d] = %d\n", i, value->bEnableCpriCw[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableCpriPkt[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    bEnableCpriPkt[%d] = %d\n", i, value->bEnableCpriPkt[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableCpriX[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    bEnableCpriX[%d] = %d\n", i, value->bEnableCpriX[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnablePack[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    bEnablePack[%d] = %d\n", i, value->bEnablePack[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnablePdLink = %d\n", value->bEnablePdLink);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdRoute (FILE *output, Aif2Fl_PdRoute *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdRoute  */\n");
  fprintf(output, "  RouteAddr = %d\n", value->RouteAddr);
  fprintf(output, "  RouteLink = %d\n", value->RouteLink);
  fprintf(output, "  RouteMask = %s\n", dump_Aif2Fl_RouteMask(value->RouteMask));
  fprintf(output, "  RouteTs = %d\n", value->RouteTs);
  fprintf(output, "  RouteType = %d\n", value->RouteType);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PdTypeLut (FILE *output, Aif2Fl_PdTypeLut *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PdTypeLut  */\n");
  fprintf(output, "  ObsaiTsFormat = %s\n", dump_Aif2Fl_TstampFormat(value->ObsaiTsFormat));
  fprintf(output, "  PdCrcType = %s\n", dump_Aif2Fl_CrcLen(value->PdCrcType));
  fprintf(output, "  PdObsaiMode = %s\n", dump_Aif2Fl_PdDataMode(value->PdObsaiMode));
  fprintf(output, "  bEnableCrc = %d\n", value->bEnableCrc);
  fprintf(output, "  bEnableCrcHeader = %d\n", value->bEnableCrcHeader);
  fprintf(output, "  bEnableEnetStrip = %d\n", value->bEnableEnetStrip);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeChRuleLut (FILE *output, Aif2Fl_PeChRuleLut *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeChRuleLut  */\n");
  fprintf(output, "  ChIndex = %d\n", value->ChIndex);
  fprintf(output, "  LutNum = %d\n", value->LutNum);
  fprintf(output, "  RuleNum = %d\n", value->RuleNum);
  fprintf(output, "  bCpriPktEnable = %d\n", value->bCpriPktEnable);
  fprintf(output, "  bEnableChIndex = %d\n", value->bEnableChIndex);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeChannelConfig (FILE *output, Aif2Fl_PeChannelConfig *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeChannelConfig  */\n");
  fprintf(output, "  AxCOffset = %d\n", value->AxCOffset);
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
    fprintf(output, "  PeDmaCh0 = ");
    dump_Aif2Fl_PeDmaCh0(output, &(value->PeDmaCh0));
    fprintf(output, "  PeInFifo = ");
    dump_Aif2Fl_PeInFifoControl(output, &(value->PeInFifo));
  fprintf(output, "  bEnableAxC = %d\n", value->bEnableAxC);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeCommonSetup (FILE *output, Aif2Fl_PeCommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeCommonSetup  */\n");
  fprintf(output, "  ChIndex0[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex0[%d] = %d\n", i, value->ChIndex0[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex1[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex1[%d] = %d\n", i, value->ChIndex1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex2[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex2[%d] = %d\n", i, value->ChIndex2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex3[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex3[%d] = %d\n", i, value->ChIndex3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex4[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex4[%d] = %d\n", i, value->ChIndex4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex5[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex5[%d] = %d\n", i, value->ChIndex5[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex6[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex6[%d] = %d\n", i, value->ChIndex6[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  ChIndex7[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    ChIndex7[%d] = %d\n", i, value->ChIndex7[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn0[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn0[%d] = %d\n", i, value->CpriPktEn0[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn1[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn1[%d] = %d\n", i, value->CpriPktEn1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn2[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn2[%d] = %d\n", i, value->CpriPktEn2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn3[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn3[%d] = %d\n", i, value->CpriPktEn3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn4[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn4[%d] = %d\n", i, value->CpriPktEn4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriPktEn5[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    CpriPktEn5[%d] = %d\n", i, value->CpriPktEn5[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  EnetHeaderSelect = %d\n", value->EnetHeaderSelect);
  fprintf(output, "  GlobalDioLen = %s\n", dump_Aif2Fl_DioLen(value->GlobalDioLen));
  fprintf(output, "  PeAxcOffset[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeAxcOffset[%d] = %d\n", i, value->PeAxcOffset[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeBbHop[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeBbHop[%d] = %d\n", i, value->PeBbHop[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeChObsaiAddr[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeChObsaiAddr[%d] = %d\n", i, value->PeChObsaiAddr[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeChObsaiTS[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeChObsaiTS[%d] = %d\n", i, value->PeChObsaiTS[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeChObsaiTsMask[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeChObsaiTsMask[%d] = %s\n", i, dump_Aif2Fl_RouteMask(value->PeChObsaiTsMask[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeChObsaiTsfomat[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeChObsaiTsfomat[%d] = %s\n", i, dump_Aif2Fl_TstampFormat(value->PeChObsaiTsfomat[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeChObsaiType[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeChObsaiType[%d] = %d\n", i, value->PeChObsaiType[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeDmaCh0[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeDmaCh0[%d] = ", i);
    dump_Aif2Fl_PeDmaCh0(output, &(value->PeDmaCh0[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeFrameMsgTc[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeFrameMsgTc[%d] = %d\n", i, value->PeFrameMsgTc[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeFrameTC[6] = [\n");
  for(i=0; i<6; i++) {
    fprintf(output, "    PeFrameTC[%d] = ", i);
    dump_Aif2Fl_FrameCounter(output, &(value->PeFrameTC[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeGlobalEnable = %d\n", value->PeGlobalEnable);
  fprintf(output, "  PeInFifo[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeInFifo[%d] = ", i);
    dump_Aif2Fl_PeInFifoControl(output, &(value->PeInFifo[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeModuloTc[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    PeModuloTc[%d] = ", i);
    dump_Aif2Fl_ModuloTc(output, &(value->PeModuloTc[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeObsaiDualBitMap[64] = [\n");
  for(i=0; i<64; i++) {
    fprintf(output, "    PeObsaiDualBitMap[%d] = ", i);
    dump_Aif2Fl_DualBitMap(output, &(value->PeObsaiDualBitMap[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeObsaiPkt[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    PeObsaiPkt[%d] = %d\n", i, value->PeObsaiPkt[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  PeTokenPhase = %d\n", value->PeTokenPhase);
  fprintf(output, "  bEnableCh[128] = [\n");
  for(i=0; i<128; i++) {
    fprintf(output, "    bEnableCh[%d] = %d\n", i, value->bEnableCh[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex0[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex0[%d] = %d\n", i, value->bEnableChIndex0[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex1[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex1[%d] = %d\n", i, value->bEnableChIndex1[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex2[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex2[%d] = %d\n", i, value->bEnableChIndex2[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex3[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex3[%d] = %d\n", i, value->bEnableChIndex3[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex4[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex4[%d] = %d\n", i, value->bEnableChIndex4[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex5[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex5[%d] = %d\n", i, value->bEnableChIndex5[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex6[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex6[%d] = %d\n", i, value->bEnableChIndex6[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnableChIndex7[512] = [\n");
  for(i=0; i<512; i++) {
    fprintf(output, "    bEnableChIndex7[%d] = %d\n", i, value->bEnableChIndex7[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeDbmr (FILE *output, Aif2Fl_PeDbmr *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeDbmr  */\n");
  fprintf(output, "  CpriLinkNum = %d\n", value->CpriLinkNum);
    fprintf(output, "  DualBitMap = ");
    dump_Aif2Fl_DualBitMap(output, &(value->DualBitMap));
  fprintf(output, "  RuleNum = %d\n", value->RuleNum);
  fprintf(output, "  isCpri = %d\n", value->isCpri);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeDmaCh0 (FILE *output, Aif2Fl_PeDmaCh0 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeDmaCh0  */\n");
  fprintf(output, "  CrcObsaiHeader = %d\n", value->CrcObsaiHeader);
  fprintf(output, "  CrcType = %s\n", dump_Aif2Fl_CrcLen(value->CrcType));
  fprintf(output, "  FrameTC = %d\n", value->FrameTC);
  fprintf(output, "  RtControl = %s\n", dump_Aif2Fl_PeRtContol(value->RtControl));
  fprintf(output, "  bCrcEn = %d\n", value->bCrcEn);
  fprintf(output, "  isEthernet = %d\n", value->isEthernet);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeInFifoControl (FILE *output, Aif2Fl_PeInFifoControl *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeInFifoControl  */\n");
  fprintf(output, "  MFifoFullLevel = %d\n", value->MFifoFullLevel);
  fprintf(output, "  MFifoWmark = %d\n", value->MFifoWmark);
  fprintf(output, "  SyncSymbol = %d\n", value->SyncSymbol);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeLinkSetup (FILE *output, Aif2Fl_PeLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeLinkSetup  */\n");
  fprintf(output, "  CpriAxCPack = %s\n", dump_Aif2Fl_CpriAxCPack(value->CpriAxCPack));
  fprintf(output, "  CpriCwChannel[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    CpriCwChannel[%d] = %d\n", i, value->CpriCwChannel[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  CpriCwNullDelimitor = %d\n", value->CpriCwNullDelimitor);
  fprintf(output, "  CpriCwPktDelimitor[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    CpriCwPktDelimitor[%d] = %s\n", i, dump_Aif2Fl_CpriCwPktDelim(value->CpriCwPktDelimitor[i]));
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  Crc8Poly = %d\n", value->Crc8Poly);
  fprintf(output, "  Crc8Seed = %d\n", value->Crc8Seed);
  fprintf(output, "  PeCppiDioSel = %s\n", dump_Aif2Fl_CppiDio(value->PeCppiDioSel));
    fprintf(output, "  PeCpriDualBitMap = ");
    dump_Aif2Fl_DualBitMap(output, &(value->PeCpriDualBitMap));
  fprintf(output, "  PeDelay = %d\n", value->PeDelay);
  fprintf(output, "  PePackDmaCh[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    PePackDmaCh[%d] = %d\n", i, value->PePackDmaCh[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  TddAxc = %d\n", value->TddAxc);
  fprintf(output, "  bEnObsaiBubbleBW = %d\n", value->bEnObsaiBubbleBW);
  fprintf(output, "  bEnableCpriCw[256] = [\n");
  for(i=0; i<256; i++) {
    fprintf(output, "    bEnableCpriCw[%d] = %d\n", i, value->bEnableCpriCw[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnablePack[4] = [\n");
  for(i=0; i<4; i++) {
    fprintf(output, "    bEnablePack[%d] = %d\n", i, value->bEnablePack[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  bEnablePeLink = %d\n", value->bEnablePeLink);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeModuloRule (FILE *output, Aif2Fl_PeModuloRule *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeModuloRule  */\n");
    fprintf(output, "  PeModuloTc = ");
    dump_Aif2Fl_ModuloTc(output, &(value->PeModuloTc));
  fprintf(output, "  RuleNum = %d\n", value->RuleNum);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PeObsaiHeader (FILE *output, Aif2Fl_PeObsaiHeader *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PeObsaiHeader  */\n");
  fprintf(output, "  ChannelNum = %d\n", value->ChannelNum);
  fprintf(output, "  PeChObsaiAddr = %d\n", value->PeChObsaiAddr);
  fprintf(output, "  PeChObsaiTs = %d\n", value->PeChObsaiTs);
  fprintf(output, "  PeChObsaiTsMask = %s\n", dump_Aif2Fl_ObsaiTsMask(value->PeChObsaiTsMask));
  fprintf(output, "  PeChObsaiTsfomat = %s\n", dump_Aif2Fl_TstampFormat(value->PeChObsaiTsfomat));
  fprintf(output, "  PeChObsaiType = %d\n", value->PeChObsaiType);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_PidStatus (FILE *output, Aif2Fl_PidStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_PidStatus  */\n");
  fprintf(output, "  RTL = %d\n", value->RTL);
  fprintf(output, "  custom = %d\n", value->custom);
  fprintf(output, "  func = %d\n", value->func);
  fprintf(output, "  major = %d\n", value->major);
  fprintf(output, "  minor = %d\n", value->minor);
  fprintf(output, "  scheme = %d\n", value->scheme);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmLinkSetup (FILE *output, Aif2Fl_RmLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmLinkSetup  */\n");
  fprintf(output, "  ClockMonitorWrap = %d\n", value->ClockMonitorWrap);
  fprintf(output, "  FrameSyncThreshold = %d\n", value->FrameSyncThreshold);
  fprintf(output, "  FrameUnsyncThreshold = %d\n", value->FrameUnsyncThreshold);
  fprintf(output, "  RmErrorSuppress = %s\n", dump_Aif2FlRmErrorSuppress(value->RmErrorSuppress));
  fprintf(output, "  RmFifoThold = %s\n", dump_Aif2Fl_RmFifoThold(value->RmFifoThold));
  fprintf(output, "  SyncThreshold = %d\n", value->SyncThreshold);
  fprintf(output, "  UnsyncThreshold = %d\n", value->UnsyncThreshold);
  fprintf(output, "  WatchDogWrap = %d\n", value->WatchDogWrap);
  fprintf(output, "  bEnableClockQuality = %d\n", value->bEnableClockQuality);
  fprintf(output, "  bEnableLcvControl = %d\n", value->bEnableLcvControl);
  fprintf(output, "  bEnableLcvUnsync = %d\n", value->bEnableLcvUnsync);
  fprintf(output, "  bEnableRmLink = %d\n", value->bEnableRmLink);
  fprintf(output, "  bEnableScrambler = %d\n", value->bEnableScrambler);
  fprintf(output, "  bEnableSdAutoAlign = %d\n", value->bEnableSdAutoAlign);
  fprintf(output, "  bEnableWatchDog = %d\n", value->bEnableWatchDog);
  fprintf(output, "  losDetThreshold = %d\n", value->losDetThreshold);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmStatus0 (FILE *output, Aif2Fl_RmStatus0 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmStatus0  */\n");
  fprintf(output, "  rmFifoOverflow = %d\n", value->rmFifoOverflow);
  fprintf(output, "  rmLoc = %d\n", value->rmLoc);
  fprintf(output, "  rmLos = %d\n", value->rmLos);
  fprintf(output, "  rmNumLosDetect = %d\n", value->rmNumLosDetect);
  fprintf(output, "  rmSyncStatus = %s\n", dump_Aif2Fl_RmSyncState(value->rmSyncStatus));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmStatus1 (FILE *output, Aif2Fl_RmStatus1 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmStatus1  */\n");
  fprintf(output, "  rmLcvCountValue = %d\n", value->rmLcvCountValue);
  fprintf(output, "  rmNumLos = %d\n", value->rmNumLos);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmStatus2 (FILE *output, Aif2Fl_RmStatus2 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmStatus2  */\n");
  fprintf(output, "  rmClockQuality = %d\n", value->rmClockQuality);
  fprintf(output, "  rmScrValue = %d\n", value->rmScrValue);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmStatus3 (FILE *output, Aif2Fl_RmStatus3 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmStatus3  */\n");
  fprintf(output, "  rmBfnHigh = %d\n", value->rmBfnHigh);
  fprintf(output, "  rmBfnLow = %d\n", value->rmBfnLow);
  fprintf(output, "  rmHfn = %d\n", value->rmHfn);
  fprintf(output, "  rmHfsyncState = %d\n", value->rmHfsyncState);
  fprintf(output, "  rmLofState = %d\n", value->rmLofState);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RmStatus4 (FILE *output, Aif2Fl_RmStatus4 *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RmStatus4  */\n");
  fprintf(output, "  rmL1LOF = %d\n", value->rmL1LOF);
  fprintf(output, "  rmL1LOS = %d\n", value->rmL1LOS);
  fprintf(output, "  rmL1PointerP = %d\n", value->rmL1PointerP);
  fprintf(output, "  rmL1RAI = %d\n", value->rmL1RAI);
  fprintf(output, "  rmL1RST = %d\n", value->rmL1RST);
  fprintf(output, "  rmL1SDI = %d\n", value->rmL1SDI);
  fprintf(output, "  rmL1Startup = %d\n", value->rmL1Startup);
  fprintf(output, "  rmL1Version = %d\n", value->rmL1Version);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RtHeaderStatus (FILE *output, Aif2Fl_RtHeaderStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RtHeaderStatus  */\n");
  fprintf(output, "  DmaChannel = %d\n", value->DmaChannel);
  fprintf(output, "  HeaderError = %d\n", value->HeaderError);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RtLinkSetup (FILE *output, Aif2Fl_RtLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RtLinkSetup  */\n");
  fprintf(output, "  CiSelect = %s\n", dump_Aif2Fl_LinkIndex(value->CiSelect));
  fprintf(output, "  RtConfig = %s\n", dump_Aif2Fl_RtConfig(value->RtConfig));
  fprintf(output, "  bEnableEmptyMsg = %d\n", value->bEnableEmptyMsg);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_RtStatus (FILE *output, Aif2Fl_RtStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_RtStatus  */\n");
  fprintf(output, "  rtEmptyMessage = %d\n", value->rtEmptyMessage);
  fprintf(output, "  rtFifoOverflow = %d\n", value->rtFifoOverflow);
  fprintf(output, "  rtFifoUnderflow = %d\n", value->rtFifoUnderflow);
  fprintf(output, "  rtFrameError = %d\n", value->rtFrameError);
  fprintf(output, "  rtHeaderError = %d\n", value->rtHeaderError);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_SdCommonSetup (FILE *output, Aif2Fl_SdCommonSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_SdCommonSetup  */\n");
  fprintf(output, "  CLKBYP_B4 = %s\n", dump_Aif2Fl_SdClockBypass(value->CLKBYP_B4));
  fprintf(output, "  CLKBYP_B8 = %s\n", dump_Aif2Fl_SdClockBypass(value->CLKBYP_B8));
  fprintf(output, "  DisableLinkClock[6] = [\n");
  for(i=0; i<6; i++) {
    fprintf(output, "    DisableLinkClock[%d] = %d\n", i, value->DisableLinkClock[i]);
  }
  fprintf(output, "  ]\n");
  fprintf(output, "  LB_B4 = %s\n", dump_Aif2Fl_SdLoopBandwidth(value->LB_B4));
  fprintf(output, "  LB_B8 = %s\n", dump_Aif2Fl_SdLoopBandwidth(value->LB_B8));
  fprintf(output, "  SleepPllB4 = %s\n", dump_Aif2Fl_SdSleepPll(value->SleepPllB4));
  fprintf(output, "  SleepPllB8 = %s\n", dump_Aif2Fl_SdSleepPll(value->SleepPllB8));
  fprintf(output, "  SysClockSelect = %s\n", dump_Aif2Fl_SdClockSelect(value->SysClockSelect));
  fprintf(output, "  VoltRangeB4 = %s\n", dump_Aif2Fl_SdVoltRange(value->VoltRangeB4));
  fprintf(output, "  VoltRangeB8 = %s\n", dump_Aif2Fl_SdVoltRange(value->VoltRangeB8));
  fprintf(output, "  bEnablePllB4 = %d\n", value->bEnablePllB4);
  fprintf(output, "  bEnablePllB8 = %d\n", value->bEnablePllB8);
  fprintf(output, "  pllMpyFactorB4 = %s\n", dump_Aif2Fl_PllMpyFactor(value->pllMpyFactorB4));
  fprintf(output, "  pllMpyFactorB8 = %s\n", dump_Aif2Fl_PllMpyFactor(value->pllMpyFactorB8));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_SdLinkSetup (FILE *output, Aif2Fl_SdLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_SdLinkSetup  */\n");
  fprintf(output, "  bEnableTxSyncMater = %d\n", value->bEnableTxSyncMater);
  fprintf(output, "  bRxEqHold = %d\n", value->bRxEqHold);
  fprintf(output, "  bRxOffsetComp = %d\n", value->bRxOffsetComp);
  fprintf(output, "  bTxFirFilterUpdate = %d\n", value->bTxFirFilterUpdate);
  fprintf(output, "  rxAlign = %s\n", dump_Aif2Fl_SdRxAlign(value->rxAlign));
  fprintf(output, "  rxCdrAlgorithm = %s\n", dump_Aif2Fl_SdRxCdrAlg(value->rxCdrAlgorithm));
  fprintf(output, "  rxEqualizerConfig = %s\n", dump_Aif2Fl_SdRxEqConfig(value->rxEqualizerConfig));
  fprintf(output, "  rxInvertPolarity = %s\n", dump_Aif2Fl_SdRxInvertPolarity(value->rxInvertPolarity));
  fprintf(output, "  rxLos = %s\n", dump_Aif2Fl_SdRxLos(value->rxLos));
  fprintf(output, "  rxTermination = %s\n", dump_Aif2Fl_SdRxTerm(value->rxTermination));
  fprintf(output, "  txInvertPolarity = %s\n", dump_Aif2Fl_SdTxInvertPolarity(value->txInvertPolarity));
  fprintf(output, "  txOutputSwing = %s\n", dump_Aif2Fl_SdTxOutputSwing(value->txOutputSwing));
  fprintf(output, "  txPostcursorTapWeight = %s\n", dump_Aif2Fl_SdTxPostcursorTabWeight(value->txPostcursorTapWeight));
  fprintf(output, "  txPrecursorTapWeight = %s\n", dump_Aif2Fl_SdTxPrecursorTabWeight(value->txPrecursorTapWeight));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_SdRxStatus (FILE *output, Aif2Fl_SdRxStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_SdRxStatus  */\n");
  fprintf(output, "  sdRxBusWidth = %d\n", value->sdRxBusWidth);
  fprintf(output, "  sdRxEqOver = %d\n", value->sdRxEqOver);
  fprintf(output, "  sdRxEqUnder = %d\n", value->sdRxEqUnder);
  fprintf(output, "  sdRxLosDetect = %d\n", value->sdRxLosDetect);
  fprintf(output, "  sdRxOCIP = %d\n", value->sdRxOCIP);
  fprintf(output, "  sdRxSync = %d\n", value->sdRxSync);
  fprintf(output, "  sdRxTestFail = %d\n", value->sdRxTestFail);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_SdTxStatus (FILE *output, Aif2Fl_SdTxStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_SdTxStatus  */\n");
  fprintf(output, "  sdTxBusWidth = %d\n", value->sdTxBusWidth);
  fprintf(output, "  sdTxTestFail = %d\n", value->sdTxTestFail);
  fprintf(output, "}\n");
}

void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_Setup  */\n");
  if (value->commonSetup != NULL) {
    fprintf(output, "  commonSetup = ");
    dump_Aif2Fl_CommonSetup(output, value->commonSetup);
  } else {
    fprintf(output, "  commonSetup = NULL\n");
  }
  if (value->globalSetup != NULL) {
    fprintf(output, "  globalSetup = ");
    dump_Aif2Fl_GlobalSetup(output, value->globalSetup);
  } else {
    fprintf(output, "  globalSetup = NULL\n");
  }
  fprintf(output, "  linkSetup[6] = [\n");
  for(i=0; i<6; i++) {
	/* Check added manually. */
	if (value->globalSetup->ActiveLink[i]) {
		if (value->linkSetup != NULL) {
		fprintf(output, "    linkSetup[%d] = ", i);
		dump_Aif2Fl_LinkSetup(output, value->linkSetup[i]);
		} else {
		  fprintf(output, "    linkSetup[%d] = NULL\n", i);
		}
	} else {
		  fprintf(output, "    linkSetup[%d] = INACTIVE\n", i);
	}
  }
  fprintf(output, "  ]\n");
  fprintf(output, "}\n");
}

void dump_Aif2Fl_TmLinkSetup (FILE *output, Aif2Fl_TmLinkSetup *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_TmLinkSetup  */\n");
  fprintf(output, "  SeedValue = %d\n", value->SeedValue);
  fprintf(output, "  bEnableRmLos = %d\n", value->bEnableRmLos);
  fprintf(output, "  bEnableScrambler = %d\n", value->bEnableScrambler);
  fprintf(output, "  bEnableTmLink = %d\n", value->bEnableTmLink);
    fprintf(output, "  pCpriTmSetup = ");
    dump_Aif2Fl_CpriTmSetup(output, &(value->pCpriTmSetup));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_TmStatus (FILE *output, Aif2Fl_TmStatus *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_TmStatus  */\n");
  fprintf(output, "  tmFail = %d\n", value->tmFail);
  fprintf(output, "  tmFifoUnderflow = %d\n", value->tmFifoUnderflow);
  fprintf(output, "  tmFrameMisalign = %d\n", value->tmFrameMisalign);
  fprintf(output, "  tmFrameStatus = %s\n", dump_Aif2Fl_TmSyncState(value->tmFrameStatus));
  fprintf(output, "}\n");
}

void dump_Aif2Fl_VcEmu (FILE *output, Aif2Fl_VcEmu *value) {
  int i;

  fprintf(output, "{ /* Type: Aif2Fl_VcEmu  */\n");
  fprintf(output, "  Freerun = %d\n", value->Freerun);
  fprintf(output, "  RtSel = %d\n", value->RtSel);
  fprintf(output, "  Soft = %d\n", value->Soft);
  fprintf(output, "}\n");
}

