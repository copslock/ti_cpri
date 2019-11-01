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

#include "dp_obj_if.h"

/* parasoft suppress item METRICS-41-3 "Number of blocks of comments per statement" */

DP_OBJ *DP_GetInstance(void)
{
    static DP_OBJ driver =
    {
        .probe = DP_Probe,
        .init = DP_Init,
        .isr = DP_Isr,
        .start = DP_Start,
        .stop = DP_Stop,
        .destroy = DP_Destroy,
        .setPhyPd = DP_SetPhyPd,
        .loadFirmware = DP_LoadFirmware,
        .startUcpu = DP_StartUcpu,
        .checkResponse = DP_CheckResponse,
        .testEcho = DP_TestEcho,
        .testEchoExt = DP_TestEchoExt,
        .getCurVersion = DP_GetCurVersion,
        .getEvent = DP_GetEvent,
        .getDebugRegVal = DP_GetDebugRegVal,
        .checkAlive = DP_CheckAlive,
        .waitAlive = DP_WaitAlive,
        .sendMainControlRequest = DP_SendMainControlRequest,
        .getMainControlResponse = DP_GetMainControlResponse,
        .mainControl = DP_MainControl,
        .setClock = DP_SetClock,
        .setWatchdogConfig = DP_SetWatchdogConfig,
        .injectEccError = DP_InjectEccError,
        .forceFatalError = DP_ForceFatalError,
        .setAudioVideoClkCfg = DP_SetAudioVideoClkCfg,
        .getAudioVideoClkCfg = DP_GetAudioVideoClkCfg,
        .setHdcpClockConfig = DP_SetHdcpClockConfig,
        .getHdcpClockConfig = DP_GetHdcpClockConfig,
        .configurePhyAuxCtrl = DP_ConfigurePhyAuxCtrl,
        .configurePhyStartUp = DP_ConfigurePhyStartUp,
        .sendEdidReadRequest = DP_SendEdidReadRequest,
        .getEdidReadResponse = DP_GetEdidReadResponse,
        .readEdid = DP_ReadEdid,
        .setPowerMode = DP_SetPowerMode,
        .setSourceCapabilities = DP_SetSourceCapabilities,
        .getSinkCapabilities = DP_GetSinkCapabilities,
        .setCustomPattern = DP_SetCustomPattern,
        .setTestPattern = DP_SetTestPattern,
        .sendDpcdReadRequest = DP_SendDpcdReadRequest,
        .getDpcdReadResponse = DP_GetDpcdReadResponse,
        .readDpcd = DP_ReadDpcd,
        .SendDpcdWriteRequest = DP_SendDpcdWriteRequest,
        .GetDpcdWriteResponse = DP_GetDpcdWriteResponse,
        .writeDpcd = DP_WriteDpcd,
        .sendI2cReadRequest = DP_SendI2cReadRequest,
        .getI2cReadResponse = DP_GetI2cReadResponse,
        .i2cRead = DP_I2cRead,
        .SendI2cWriteRequest = DP_SendI2cWriteRequest,
        .GetI2cWriteResponse = DP_GetI2cWriteResponse,
        .i2cWrite = DP_I2cWrite,
        .setAssrEnable = DP_SetAssrEnable,
        .setShortenedAuxPreamble = DP_SetShortenedAuxPreamble,
        .linkTraining = DP_LinkTraining,
        .checkLinkStable = DP_CheckLinkStable,
        .setEventMask = DP_SetEventMask,
        .getEventMask = DP_GetEventMask,
        .sendReadHpdEventRequest = DP_SendReadHpdEventRequest,
        .getReadHpdEventResponse = DP_GetReadHpdEventResponse,
        .readHpdEvent = DP_ReadHpdEvent,
        .fillVideoFormat = DP_FillVideoFormat,
        .setVic = DP_SetVic,
        .setMsaSyncPolarity = DP_SetMsaSyncPolarity,
        .setFramerEnable = DP_SetFramerEnable,
        .setVideoSst = DP_SetVideoSst,
        .readLinkStat = DP_ReadLinkStat,
        .sendAuxStatusRequest = DP_SendAuxStatusRequest,
        .getAuxStatusResponse = DP_GetAuxStatusResponse,
        .getAuxStatus = DP_GetAuxStatus,
        .sendI2cStatusRequest = DP_SendI2cStatusRequest,
        .getI2cStatusResponse = DP_GetI2cStatusResponse,
        .getI2cStatus = DP_GetI2cStatus,
        .sendHpdStatusRequest = DP_SendHpdStatusRequest,
        .getHpdStatusResponse = DP_GetHpdStatusResponse,
        .getHpdStatus = DP_GetHpdStatus,
        .setFecEnable = DP_SetFecEnable,
        .setFecReady = DP_SetFecReady,
        .setSdp = DP_SetSdp,
        .removeSdp = DP_RemoveSdp,
        .configureHdcpTx = DP_ConfigureHdcpTx,
        .setHdcp2TxPublicKey = DP_SetHdcp2TxPublicKey,
        .setHdcpKmEncCustomKey = DP_SetHdcpKmEncCustomKey,
        .setHdcp2DebugRandom = DP_SetHdcp2DebugRandom,
        .hdcp2RespondKmNotStored = DP_Hdcp2RespondKmNotStored,
        .hdcp2RespondKmStored = DP_Hdcp2RespondKmStored,
        .setHdcp1TxKeys = DP_SetHdcp1TxKeys,
        .setHdcp1RandomAn = DP_SetHdcp1RandomAn,
        .sendHdcpTxStatusRequest = DP_SendHdcpTxStatusRequest,
        .getHdcpTxStatusResponse = DP_GetHdcpTxStatusResponse,
        .getHdcpTxStatus = DP_GetHdcpTxStatus,
        .sendHdcp2RecvIdRequest = DP_SendHdcp2RecvIdRequest,
        .getHdcp2RecvIdResponse = DP_GetHdcp2RecvIdResponse,
        .getHdcp2RecvId = DP_GetHdcp2RecvId,
        .sendHdcp2PairingDataRequest = DP_SendHdcp2PairingDataRequest,
        .getHdcp2PairingDataResponse = DP_GetHdcp2PairingDataResponse,
        .getHdcp2PairingData = DP_GetHdcp2PairingData,
        .sendHdcpRecvIdListRequest = DP_SendHdcpRecvIdListRequest,
        .getHdcpRecvIdListResponse = DP_GetHdcpRecvIdListResponse,
        .getHdcpRecvIdList = DP_GetHdcpRecvIdList,
        .setHdcpRecvValid = DP_SetHdcpRecvValid,
        .setHdcp2Lc = DP_SetHdcp2Lc,
        .setHdcpSeed = DP_SetHdcpSeed,
        .audioSetMute = DP_AudioSetMute,
        .audioAutoConfig = DP_AudioAutoConfig,
        .audioStop = DP_AudioStop,
        .audioSetMode = DP_AudioSetMode,
        .setDscConfig = DP_SetDscConfig,
        .getDscConfig = DP_GetDscConfig,
        .dscSendPps = DP_DscSendPps,
        .setCompressedStreamFlag = DP_SetCompressedStreamFlag,
        .dscReset = DP_DscReset,
        .MstEnable = DP_MstEnable,
        .MstDisable = DP_MstDisable,
        .MstStreamEnable = DP_MstStreamEnable,
        .MstStreamDisable = DP_MstStreamDisable,
        .MstAllocateVcpi = DP_MstAllocateVcpi,
        .MstDeallocateVcpi = DP_MstDeallocateVcpi,
        .MstGetSinkCount = DP_MstGetSinkCount,
        .MstGetSinkList = DP_MstGetSinkList,
        .MstSetEncryptionEnable = DP_MstSetEncryptionEnable,
        .MstSetEncryption = DP_MstSetEncryption,
        .MstScanTopology = DP_MstScanTopology,
        .MstReadRemoteEdid = DP_MstReadRemoteEdid,
        .MstHpdIrq = DP_MstHpdIrq,
    };

    return &driver;
}
