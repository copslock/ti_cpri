/**********************************************************************
* Copyright (C) 2012-2019 Cadence Design Systems, Inc.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* WARNING: This file is auto-generated using api-generator utility.
*          api-generator: 12.03.9e11b77(origin/DRV-3827_extract_sanity_to_c_file)
*          Do not edit it manually.
**********************************************************************
* Cadence Core Driver for the Cadence DisplayPort (DP) core. This header
* file lists the API providing a HAL (hardware abstraction layer)
* interface for the DP core
**********************************************************************/

/* parasoft-begin-suppress METRICS-18-3 "Follow the Cyclomatic Complexity limit of 10" */
/* parasoft-begin-suppress METRICS-36-3 "A function should not be called from more than 5 different functions" */
/* parasoft-begin-suppress METRICS-39-3 "The value of VOCF metric for a function should not be higher than 4" */
/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */
/* parasoft-begin-suppress MISRA2012-RULE-8_7 "Functions and objects should not be defined with external linkage if they are referenced in only one translation unit" */

/**
 * This file contains sanity API functions. The purpose of sanity functions
 * is to check input parameters validity. They take the same parameters as
 * original API functions and return 0 on success or CDN_EINVAL on wrong parameter
 * value(s).
 */

#ifndef DP_SANITY_H
#define DP_SANITY_H

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "dp_if.h"

uint32_t DP_AudioParamsSF(const DP_AudioParams *obj);
uint32_t DP_AudioVideoClkCfgSF(const DP_AudioVideoClkCfg *obj);
uint32_t DP_CallbacksSF(const DP_Callbacks *obj);
uint32_t DP_ConfigSF(const DP_Config *obj);
uint32_t DP_DpcdTransferSF(const DP_DpcdTransfer *obj);
uint32_t DP_DscConfigSF(const DP_DscConfig *obj);
uint32_t DP_FirmwareImageSF(const DP_FirmwareImage *obj);
uint32_t DP_Hdcp1KeysSF(const DP_Hdcp1Keys *obj);
uint32_t DP_Hdcp2TxPublicKeySF(const DP_Hdcp2TxPublicKey *obj);
uint32_t DP_HdcpDebugRandomNumbersSF(const DP_HdcpDebugRandomNumbers *obj);
uint32_t DP_HdcpPairingDataSF(const DP_HdcpPairingData *obj);
uint32_t DP_HdcpTxConfigurationSF(const DP_HdcpTxConfiguration *obj);
uint32_t DP_HdcpTxKmEncCustomKeySF(const DP_HdcpTxKmEncCustomKey *obj);
uint32_t DP_I2cTransferSF(const DP_I2cTransfer *obj);
uint32_t DP_LinkStateSF(const DP_LinkState *obj);
uint32_t DP_PrivateDataSF(const DP_PrivateData *obj);
uint32_t DP_ReadEdidResponseSF(const DP_ReadEdidResponse *obj);
uint32_t DP_SdpEntrySF(const DP_SdpEntry *obj);
uint32_t DP_SinkDeviceCapabilitiesSF(const DP_SinkDeviceCapabilities *obj);
uint32_t DP_SinkDeviceSF(const DP_SinkDevice *obj);
uint32_t DP_SourceDeviceCapabilitiesSF(const DP_SourceDeviceCapabilities *obj);
uint32_t DP_UcpuClockSF(const DP_UcpuClock *obj);
uint32_t DP_VideoFormatParamsSF(const DP_VideoFormatParams *obj);
uint32_t DP_VideoParametersSF(const DP_VideoParameters *obj);

uint32_t DP_SanityFunction1(const DP_Config* config, const uint32_t* memReq);
uint32_t DP_SanityFunction2(const DP_PrivateData* pD, const DP_Config* config, const DP_Callbacks* callbacks);
uint32_t DP_SanityFunction3(const DP_PrivateData* pD);
uint32_t DP_SanityFunction7(const DP_PrivateData* pD, const DP_SD0801_PrivateData* phyPd);
uint32_t DP_SanityFunction8(const DP_PrivateData* pD, const DP_FirmwareImage* image);
uint32_t DP_SanityFunction10(const DP_PrivateData* pD, const bool* awaits, const DP_BusType busType);
uint32_t DP_SanityFunction11(const DP_PrivateData* pD, const DP_BusType busType);
uint32_t DP_SanityFunction12(const DP_PrivateData* pD, const uint8_t* message, const uint8_t* response, const uint16_t messageSize, const DP_BusType busType);
uint32_t DP_SanityFunction13(const DP_PrivateData* pD, const uint16_t* ver, const uint16_t* verlib);
uint32_t DP_SanityFunction14(const DP_PrivateData* pD, const uint32_t* events);
uint32_t DP_SanityFunction15(const DP_PrivateData* pD, const uint16_t* debug);
uint32_t DP_SanityFunction16(const DP_PrivateData* pD, const bool* updated);
uint32_t DP_SanityFunction19(const DP_PrivateData* pD, const uint8_t* resp);
uint32_t DP_SanityFunction21(const DP_PrivateData* pD, const DP_UcpuClock* ucpuClock);
uint32_t DP_SanityFunction23(const DP_PrivateData* pD, const DP_EccErrorMemType memType, const DP_EccErrorType errorType);
uint32_t DP_SanityFunction25(const DP_PrivateData* pD, const uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg);
uint32_t DP_SanityFunction26(const DP_PrivateData* pD, const uint8_t streamId, const DP_AudioVideoClkCfg* audioVideoClkCfg);
uint32_t DP_SanityFunction30(const DP_PrivateData* pD, const DP_LinkRate linkRate);
uint32_t DP_SanityFunction32(const DP_PrivateData* pD, const DP_ReadEdidResponse* resp);
uint32_t DP_SanityFunction34(const DP_PrivateData* pD, const DP_PwrMode mode);
uint32_t DP_SanityFunction35(const DP_PrivateData* pD, const DP_SourceDeviceCapabilities* caps);
uint32_t DP_SanityFunction36(const DP_PrivateData* pD, const DP_SinkDeviceCapabilities* caps);
uint32_t DP_SanityFunction38(const DP_PrivateData* pD, const DP_TestPattern pattern, const DP_LinkState* linkParams);
uint32_t DP_SanityFunction39(const DP_PrivateData* pD, const DP_DpcdTransfer* request);
uint32_t DP_SanityFunction40(const DP_PrivateData* pD, const DP_DpcdTransfer* transfer);
uint32_t DP_SanityFunction45(const DP_PrivateData* pD, const DP_I2cTransfer* request);
uint32_t DP_SanityFunction46(const DP_PrivateData* pD, const DP_I2cTransfer* transfer);
uint32_t DP_SanityFunction53(const DP_PrivateData* pD, const DP_TrainingStatus* resultLt);
uint32_t DP_SanityFunction60(const DP_VideoFormatParams* vicParams, const DP_VicModes vicMode);
uint32_t DP_SanityFunction61(const DP_PrivateData* pD, const DP_VideoParameters* parameters);
uint32_t DP_SanityFunction62(const DP_PrivateData* pD, const DP_SyncPolarity hSyncPolarity, const DP_SyncPolarity vSyncPolarity);
uint32_t DP_SanityFunction65(const DP_PrivateData* pD, const DP_LinkState* linkState);
uint32_t DP_SanityFunction67(const DP_PrivateData* pD, const DP_AuxStatus* status);
uint32_t DP_SanityFunction70(const DP_PrivateData* pD, const DP_I2cStatus* status);
uint32_t DP_SanityFunction77(const DP_PrivateData* pD, const DP_SdpEntry* packetData);
uint32_t DP_SanityFunction78(const DP_PrivateData* pD, const uint8_t entryID);
uint32_t DP_SanityFunction79(const DP_PrivateData* pD, const DP_HdcpTxConfiguration* config);
uint32_t DP_SanityFunction80(const DP_PrivateData* pD, const DP_Hdcp2TxPublicKey* key);
uint32_t DP_SanityFunction81(const DP_PrivateData* pD, const DP_HdcpTxKmEncCustomKey* key);
uint32_t DP_SanityFunction82(const DP_PrivateData* pD, const DP_HdcpDebugRandomNumbers* numbers);
uint32_t DP_SanityFunction84(const DP_PrivateData* pD, const DP_HdcpPairingData* pairingData);
uint32_t DP_SanityFunction85(const DP_PrivateData* pD, const DP_Hdcp1Keys* keySet);
uint32_t DP_SanityFunction88(const DP_PrivateData* pD, const DP_HdcpTxStatus* status);
uint32_t DP_SanityFunction94(const DP_PrivateData* pD, const DP_HdcpPairingData* pairingData);
uint32_t DP_SanityFunction97(const DP_PrivateData* pD, const DP_HdcpRecvIdList* list);
uint32_t DP_SanityFunction102(const DP_PrivateData* pD, const DP_AudioMuteMode muteMode);
uint32_t DP_SanityFunction103(const DP_PrivateData* pD, const DP_AudioParams* params);
uint32_t DP_SanityFunction105(const DP_PrivateData* pD, const DP_AudioMode mode);
uint32_t DP_SanityFunction106(const DP_PrivateData* pD, const DP_DscConfig* dscConfig);
uint32_t DP_SanityFunction107(const DP_PrivateData* pD, const DP_DscConfig* dscConfig);
uint32_t DP_SanityFunction115(const DP_PrivateData* pD, const DP_SinkDevice* sinkDevice);
uint32_t DP_SanityFunction118(const DP_PrivateData* pD, const DP_SinkDevice** sinkList);
uint32_t DP_SanityFunction122(const DP_PrivateData* pD, const DP_SinkDevice* sinkDevice, const DP_ReadEdidResponse* edidResponse);

#define DP_ProbeSF DP_SanityFunction1
#define DP_InitSF DP_SanityFunction2
#define DP_IsrSF DP_SanityFunction3
#define DP_StartSF DP_SanityFunction3
#define DP_StopSF DP_SanityFunction3
#define DP_DestroySF DP_SanityFunction3
#define DP_SetPhyPdSF DP_SanityFunction7
#define DP_LoadFirmwareSF DP_SanityFunction8
#define DP_StartUcpuSF DP_SanityFunction3
#define DP_CheckResponseSF DP_SanityFunction10
#define DP_TestEchoSF DP_SanityFunction11
#define DP_TestEchoExtSF DP_SanityFunction12
#define DP_GetCurVersionSF DP_SanityFunction13
#define DP_GetEventSF DP_SanityFunction14
#define DP_GetDebugRegValSF DP_SanityFunction15
#define DP_CheckAliveSF DP_SanityFunction16
#define DP_WaitAliveSF DP_SanityFunction3
#define DP_SendMainControlRequestSF DP_SanityFunction3
#define DP_GetMainControlResponseSF DP_SanityFunction19
#define DP_MainControlSF DP_SanityFunction19
#define DP_SetClockSF DP_SanityFunction21
#define DP_SetWatchdogConfigSF DP_SanityFunction3
#define DP_InjectEccErrorSF DP_SanityFunction23
#define DP_ForceFatalErrorSF DP_SanityFunction3
#define DP_SetAudioVideoClkCfgSF DP_SanityFunction25
#define DP_GetAudioVideoClkCfgSF DP_SanityFunction26
#define DP_SetHdcpClockConfigSF DP_SanityFunction3
#define DP_GetHdcpClockConfigSF DP_SanityFunction16
#define DP_ConfigurePhyAuxCtrlSF DP_SanityFunction3
#define DP_ConfigurePhyStartUpSF DP_SanityFunction30
#define DP_SendEdidReadRequestSF DP_SanityFunction3
#define DP_GetEdidReadResponseSF DP_SanityFunction32
#define DP_ReadEdidSF DP_SanityFunction32
#define DP_SetPowerModeSF DP_SanityFunction34
#define DP_SetSourceCapabilitiesSF DP_SanityFunction35
#define DP_GetSinkCapabilitiesSF DP_SanityFunction36
#define DP_SetCustomPatternSF DP_SanityFunction3
#define DP_SetTestPatternSF DP_SanityFunction38
#define DP_SendDpcdReadRequestSF DP_SanityFunction39
#define DP_GetDpcdReadResponseSF DP_SanityFunction40
#define DP_ReadDpcdSF DP_SanityFunction40
#define DP_SendDpcdWriteRequestSF DP_SanityFunction39
#define DP_GetDpcdWriteResponseSF DP_SanityFunction40
#define DP_WriteDpcdSF DP_SanityFunction40
#define DP_SendI2cReadRequestSF DP_SanityFunction45
#define DP_GetI2cReadResponseSF DP_SanityFunction46
#define DP_I2cReadSF DP_SanityFunction46
#define DP_SendI2cWriteRequestSF DP_SanityFunction45
#define DP_GetI2cWriteResponseSF DP_SanityFunction46
#define DP_I2cWriteSF DP_SanityFunction46
#define DP_SetAssrEnableSF DP_SanityFunction3
#define DP_SetShortenedAuxPreambleSF DP_SanityFunction3
#define DP_LinkTrainingSF DP_SanityFunction53
#define DP_CheckLinkStableSF DP_SanityFunction16
#define DP_SetEventMaskSF DP_SanityFunction3
#define DP_GetEventMaskSF DP_SanityFunction14
#define DP_SendReadHpdEventRequestSF DP_SanityFunction3
#define DP_GetReadHpdEventResponseSF DP_SanityFunction19
#define DP_ReadHpdEventSF DP_SanityFunction19
#define DP_FillVideoFormatSF DP_SanityFunction60
#define DP_SetVicSF DP_SanityFunction61
#define DP_SetMsaSyncPolaritySF DP_SanityFunction62
#define DP_SetFramerEnableSF DP_SanityFunction3
#define DP_SetVideoSstSF DP_SanityFunction3
#define DP_ReadLinkStatSF DP_SanityFunction65
#define DP_SendAuxStatusRequestSF DP_SanityFunction3
#define DP_GetAuxStatusResponseSF DP_SanityFunction67
#define DP_GetAuxStatusSF DP_SanityFunction67
#define DP_SendI2cStatusRequestSF DP_SanityFunction3
#define DP_GetI2cStatusResponseSF DP_SanityFunction70
#define DP_GetI2cStatusSF DP_SanityFunction70
#define DP_SendHpdStatusRequestSF DP_SanityFunction3
#define DP_GetHpdStatusResponseSF DP_SanityFunction16
#define DP_GetHpdStatusSF DP_SanityFunction16
#define DP_SetFecEnableSF DP_SanityFunction3
#define DP_SetFecReadySF DP_SanityFunction3
#define DP_SetSdpSF DP_SanityFunction77
#define DP_RemoveSdpSF DP_SanityFunction78
#define DP_ConfigureHdcpTxSF DP_SanityFunction79
#define DP_SetHdcp2TxPublicKeySF DP_SanityFunction80
#define DP_SetHdcpKmEncCustomKeySF DP_SanityFunction81
#define DP_SetHdcp2DebugRandomSF DP_SanityFunction82
#define DP_Hdcp2RespondKmNotStoredSF DP_SanityFunction3
#define DP_Hdcp2RespondKmStoredSF DP_SanityFunction84
#define DP_SetHdcp1TxKeysSF DP_SanityFunction85
#define DP_SetHdcp1RandomAnSF DP_SanityFunction3
#define DP_SendHdcpTxStatusRequestSF DP_SanityFunction3
#define DP_GetHdcpTxStatusResponseSF DP_SanityFunction88
#define DP_GetHdcpTxStatusSF DP_SanityFunction88
#define DP_SendHdcp2RecvIdRequestSF DP_SanityFunction3
#define DP_GetHdcp2RecvIdResponseSF DP_SanityFunction3
#define DP_GetHdcp2RecvIdSF DP_SanityFunction3
#define DP_SendHdcp2PairingDataRequeSF DP_SanityFunction3
#define DP_GetHdcp2PairingDataResponSF DP_SanityFunction94
#define DP_GetHdcp2PairingDataSF DP_SanityFunction94
#define DP_SendHdcpRecvIdListRequestSF DP_SanityFunction3
#define DP_GetHdcpRecvIdListResponseSF DP_SanityFunction97
#define DP_GetHdcpRecvIdListSF DP_SanityFunction97
#define DP_SetHdcpRecvValidSF DP_SanityFunction3
#define DP_SetHdcp2LcSF DP_SanityFunction3
#define DP_SetHdcpSeedSF DP_SanityFunction3
#define DP_AudioSetMuteSF DP_SanityFunction102
#define DP_AudioAutoConfigSF DP_SanityFunction103
#define DP_AudioStopSF DP_SanityFunction3
#define DP_AudioSetModeSF DP_SanityFunction105
#define DP_SetDscConfigSF DP_SanityFunction106
#define DP_GetDscConfigSF DP_SanityFunction107
#define DP_DscSendPpsSF DP_SanityFunction3
#define DP_SetCompressedStreamFlagSF DP_SanityFunction3
#define DP_DscResetSF DP_SanityFunction3
#define DP_MstEnableSF DP_SanityFunction3
#define DP_MstDisableSF DP_SanityFunction3
#define DP_MstStreamEnableSF DP_SanityFunction3
#define DP_MstStreamDisableSF DP_SanityFunction3
#define DP_MstAllocateVcpiSF DP_SanityFunction115
#define DP_MstDeallocateVcpiSF DP_SanityFunction115
#define DP_MstGetSinkCountSF DP_SanityFunction19
#define DP_MstGetSinkListSF DP_SanityFunction118
#define DP_MstSetEncryptionEnableSF DP_SanityFunction3
#define DP_MstSetEncryptionSF DP_SanityFunction3
#define DP_MstScanTopologySF DP_SanityFunction3
#define DP_MstReadRemoteEdidSF DP_SanityFunction122
#define DP_MstHpdIrqSF DP_SanityFunction3

#endif  /* DP_SANITY_H */

/* parasoft-end-suppress MISRA2012-RULE-8_7 */
/* parasoft-end-suppress METRICS-41-3 */
/* parasoft-end-suppress METRICS-39-3 */
/* parasoft-end-suppress METRICS-36-3 */
/* parasoft-end-suppress METRICS-18-3 */
