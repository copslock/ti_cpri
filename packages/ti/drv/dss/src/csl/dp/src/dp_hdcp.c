/******************************************************************************
 *
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
 *
 ******************************************************************************
 *
 * dp_hdcp.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-41-3 "Number of blocks of comments per statement" */

#include "dp_hdcp.h"
#include "dp_if.h"
#include "dp_structs_if.h"
#include "dp_priv.h"
#include "dp_sanity.h"
#include "dp_mailbox.h"
#include "cdn_log.h"
#include "cdn_errno.h"
#include "cdn_stdtypes.h"

static uint32_t getErrorCode(uint8_t i, DP_HdcpErrCode* error)
{
    uint32_t retVal = CDN_EOK;

    static const DP_HdcpErrCode errorsArray[] = {
        DP_HDCP_ERR_NO_ERROR,
        DP_HDCP_ERR_HPD_DOWN,
        DP_HDCP_ERR_SRM_FAIL,
        DP_HDCP_ERR_SIGN_ERROR,
        DP_HDCP_ERR_H_HASH_MISMATCH,
        DP_HDCP_ERR_V_HASH_MISMATCH,
        DP_HDCP_ERR_LOCALITY_CHECK_FAIL,
        DP_HDCP_ERR_DDC_ERROR,
        DP_HDCP_ERR_REAUTH_REQ,
        DP_HDCP_ERR_TOPOLOGY_ERROR,
        DP_HDCP_ERR_RSVD_NOT_ZERO,
        DP_HDCP_ERR_RI_MISMATCH,
        DP_HDCP_ERR_WATCHDOG_EXPIRED
    };

    if (i >= (sizeof(errorsArray) / sizeof(DP_HdcpErrCode))) {
        retVal = CDN_EINVAL;
    }

    if (retVal == CDN_EOK) {
        *error = errorsArray[i];
    }

    return retVal;
}

static void fillHdcpConfigVal(const DP_HdcpTxConfiguration* config, uint8_t* val)
{
    *val = (uint8_t)config->hdcpVerSupport & 0x03U;
    *val |= config->activate ? 0x04U : 0U;
    *val |= (DP_TYPE_1_CONTENT_STREAM == config->contentStreamType) ? 0x08U : 0U;
    *val |= config->enableKmEncryption ? 0x10U : 0U;
}

uint32_t DP_ConfigureHdcpTx(DP_PrivateData*               pD,
                            const DP_HdcpTxConfiguration* config)
{
    uint32_t retVal;
    uint8_t val;

    retVal = DP_ConfigureHdcpTxSF(pD, config);
    if (CDN_EOK == retVal)
    {
        fillHdcpConfigVal(config, &val);

        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_CONFIGURATION);
        messageWriteUint8(pD, val);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }
    return retVal;
}

uint32_t DP_SetHdcp2TxPublicKey(DP_PrivateData*            pD,
                                const DP_Hdcp2TxPublicKey* key)
{
    uint32_t retVal;

    retVal = DP_SetHdcp2TxPublicKeySF(pD, key);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_SET_PUBLIC_KEY_PARAMS);
        messageWriteBuffer(pD, key->modulusN, DP_HDCP2_PUBLIC_KEY_N_LENGTH);
        messageWriteBuffer(pD, key->exponentE, DP_HDCP2_PUBLIC_KEY_E_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }
    return retVal;
}

uint32_t DP_SetHdcpKmEncCustomKey(DP_PrivateData*                pD,
                                  const DP_HdcpTxKmEncCustomKey* key)
{
    uint32_t retVal;

    retVal = DP_SetHdcpKmEncCustomKeySF(pD, key);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_SET_KM_KEY_PARAMS);
        messageWriteBuffer(pD, key->kmEncCutomKey, DP_HDCP_CUSTOM_KEY_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }
    return retVal;
}

uint32_t DP_SetHdcp2DebugRandom(DP_PrivateData*                  pD,
                                const DP_HdcpDebugRandomNumbers* numbers)
{
    uint32_t retVal;

    retVal = DP_SetHdcp2DebugRandomSF(pD, numbers);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_SET_KM_KEY_PARAMS);
        messageWriteBuffer(pD, numbers->km, DP_HDCP_KM_LENGTH);
        messageWriteBuffer(pD, numbers->rn, DP_HDCP_RANDOM_RN_LENGTH);
        messageWriteBuffer(pD, numbers->ks, DP_HDCP_RANDOM_KS_LENGTH);
        messageWriteBuffer(pD, numbers->riv, DP_HDCP_RANDOM_RIV_LENGTH);
        messageWriteBuffer(pD, numbers->rtx, DP_HDCP_RANDOM_RTX_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_Hdcp2RespondKmNotStored(DP_PrivateData* pD)
{
    uint32_t retVal;

    retVal = DP_Hdcp2RespondKmNotStoredSF(pD);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_RESPOND_KM);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_Hdcp2RespondKmStored(DP_PrivateData*           pD,
                                 const DP_HdcpPairingData* pairingData)
{
    uint32_t retVal;

    retVal = DP_Hdcp2RespondKmStoredSF(pD, pairingData);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_RESPOND_KM);
        messageWriteBuffer(pD, pairingData->id, DP_HDCP_RECV_ID_LENGTH);
        messageWriteBuffer(pD, pairingData->m, DP_HDCP_M_LENGTH);
        messageWriteBuffer(pD, pairingData->km, DP_HDCP_KM_LENGTH);
        messageWriteBuffer(pD, pairingData->ekhKm, DP_HDCP_KM_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_SetHdcp1TxKeys(DP_PrivateData*     pD,
                           const DP_Hdcp1Keys* keySet)
{
    uint32_t retVal;

    retVal = DP_SetHdcp1TxKeysSF(pD, keySet);
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP1_TX_SEND_KEYS);
        messageWriteBuffer(pD, keySet->ksv, DP_HDCP_RECV_ID_LENGTH);
        messageWriteBuffer(pD, keySet->keys, DP_HDCP1_KEY_SET_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_SetHdcp1RandomAn(DP_PrivateData* pD,
                             const uint8_t   an[8])
{
    uint32_t retVal;

    retVal = DP_SetHdcp1RandomAnSF(pD);
    if ((NULL == an))
    {
        retVal = CDN_EINVAL;
    }
    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP1_TX_SEND_RANDOM_AN);
        messageWriteBuffer(pD, an, 8U);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_SendHdcpTxStatusRequest(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SendHdcpTxStatusRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_STATUS_CHANGE);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_GetHdcpTxStatusResponse(DP_PrivateData*  pD,
                                    DP_HdcpTxStatus* status)
{
    uint8_t resp[DP_HDCP_TX_STATUS_SIZE];
    uint32_t retVal = CDN_EOK;
    uint8_t errorCode;
    DP_HdcpErrCode error;

    retVal = DP_GetHdcpTxStatusResponseSF(pD, status);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HDCP TX "
               "status was called incorrectly. DPTX Mailbox may not be able to "
               "operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_SAPB);
        if (0U == messageHeaderMatches(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_STATUS_CHANGE))
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadBuffer(pD, resp, DP_HDCP_TX_STATUS_SIZE);

        errorCode = ((resp[1] & 0xE0U) >> 5) | ((resp[0] & 0x01U) << 3);
        retVal = getErrorCode(errorCode, &error);

        if (CDN_EOK == retVal) {
            status->authenticated = (0U != (resp[1] & 0x01U));
            status->repeater = (0U != (resp[1] & 0x02U));
            status->rxType = (resp[1] >> 2) & 0x03U;
            status->authStreamIdSuccess = (0U != (resp[1] & 0x10U));
            status->lastErr = error;
            status->enable1d1Features = (0U != (resp[0] & 0x02U));
        }
    }

    return retVal;
}

uint32_t DP_GetHdcpTxStatus(DP_PrivateData*  pD,
                            DP_HdcpTxStatus* status)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcpTxStatusSF(pD, status);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendHdcpTxStatusRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetHdcpTxStatusResponse(pD, status);
    }

    return retVal;
}

uint32_t DP_SendHdcp2RecvIdRequest(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SendHdcp2RecvIdRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, (uint8_t)MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_IS_KM_STORED);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_GetHdcp2RecvIdResponse(DP_PrivateData* pD,
                                   uint8_t         id[5])
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcp2RecvIdResponseSF(pD);
    if (NULL == id)
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HDCP2.x "
               "Receiver ID was called incorrectly. DPTX Mailbox may not be "
               "able to operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_SAPB);
        if (0U == messageHeaderMatches(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_IS_KM_STORED))
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadBuffer(pD, id, DP_HDCP_RECV_ID_LENGTH);
    }

    return retVal;
}

uint32_t DP_GetHdcp2RecvId(DP_PrivateData* pD,
                           uint8_t         id[5])
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcp2RecvIdSF(pD);
    if (NULL == id)
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendHdcp2RecvIdRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetHdcp2RecvIdResponse(pD, id);
    }

    return retVal;
}

uint32_t DP_SendHdcp2PairingDataRequest(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SendHdcp2PairingDataRequeSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_STORE_KM);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_GetHdcp2PairingDataResponse(DP_PrivateData*     pD,
                                        DP_HdcpPairingData* pairingData)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcp2PairingDataResponSF(pD, pairingData);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HDCP 2.x "
               "receiver pairing data was called incorrectly. DPTX Mailbox may "
               "not be able to operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_SAPB);
        if (0U == messageHeaderMatches(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP2_TX_STORE_KM))
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadBuffer(pD, pairingData->id, DP_HDCP_RECV_ID_LENGTH);
        messageReadBuffer(pD, pairingData->m, DP_HDCP_M_LENGTH);
        messageReadBuffer(pD, pairingData->km, DP_HDCP_KM_LENGTH);
        messageReadBuffer(pD, pairingData->ekhKm, DP_HDCP_KM_LENGTH);
    }

    return retVal;
}

uint32_t DP_GetHdcp2PairingData(DP_PrivateData*     pD,
                                DP_HdcpPairingData* pairingData)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcp2PairingDataSF(pD, pairingData);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendHdcp2PairingDataRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetHdcp2PairingDataResponse(pD, pairingData);
    }

    return retVal;
}

uint32_t DP_SendHdcpRecvIdListRequest(DP_PrivateData* pD)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_SendHdcpRecvIdListRequestSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_IS_RECEIVER_ID_VALID);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_GetHdcpRecvIdListResponse(DP_PrivateData*    pD,
                                      DP_HdcpRecvIdList* list)
{
    uint32_t retVal = CDN_EOK;
    uint8_t dummy;

    retVal = DP_GetHdcpRecvIdListResponseSF(pD, list);

    if (CDN_EOK != retVal)
    {
        DbgMsg(DBG_GEN_MSG, DBG_CRIT,
               "Function for receiving data containing response with HDCP "
               "Receiver ID/KSV list was called incorrectly. DPTX Mailbox may "
               "not be able to operate any more.\n");
    }

    if (CDN_EOK == retVal)
    {
        messageReceive(pD, DP_BUS_TYPE_SAPB);
        if (0U == messageHeaderMatches(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_IS_RECEIVER_ID_VALID))
        {
            retVal = CDN_ENOEXEC;
        }
    }

    if (CDN_EOK == retVal)
    {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint8(pD, &(list->count));
        messageReadUint8(pD, &dummy);
        messageReadBuffer(pD, list->ids, ((uint16_t)list->count * DP_HDCP_RECV_ID_LENGTH));
    }

    return retVal;
}

uint32_t DP_GetHdcpRecvIdList(DP_PrivateData*    pD,
                              DP_HdcpRecvIdList* list)
{
    uint32_t retVal = CDN_EOK;

    retVal = DP_GetHdcpRecvIdListSF(pD, list);

    if (CDN_EOK == retVal)
    {
        retVal = DP_SendHdcpRecvIdListRequest(pD);
    }
    if (CDN_EOK == retVal)
    {
        retVal = DP_GetHdcpRecvIdListResponse(pD, list);
    }

    return retVal;
}

uint32_t DP_SetHdcpRecvValid(DP_PrivateData* pD,
                             bool            valid)
{
    uint32_t retVal;

    retVal = DP_SetHdcpRecvValidSF(pD);

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_TX, (uint8_t)HDCP_TX_RESPOND_RECEIVER_ID_VALID);
        messageWriteUint8(pD, valid ? 1U : 0U);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }
    return retVal;
}

uint32_t DP_SetHdcp2Lc(DP_PrivateData* pD,
                       const uint8_t   lc128[DP_HDCP_LC128_LENGTH])
{
    uint32_t retVal;

    retVal = DP_SetHdcp2LcSF(pD);
    if (NULL == lc128)
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_GENERAL, (uint8_t)HDCP_GENERAL_SET_LC_128);
        messageWriteBuffer(pD, lc128, DP_HDCP_LC128_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

uint32_t DP_SetHdcpSeed(DP_PrivateData* pD,
                        const uint8_t   seed[DP_HDCP_SEED_LENGTH])
{
    uint32_t retVal;

    retVal = DP_SetHdcpSeedSF(pD);
    if (NULL == seed)
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal)
    {
        messageStart(pD, MB_MODULE_ID_HDCP_GENERAL, (uint8_t)HDCP_SET_SEED);
        messageWriteBuffer(pD, seed, DP_HDCP_SEED_LENGTH);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_SAPB);
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-41-3 */
