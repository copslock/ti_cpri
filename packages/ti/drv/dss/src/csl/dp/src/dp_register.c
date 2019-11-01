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
 * dp_register.c
 *
 ******************************************************************************
 */

/* parasoft-begin-suppress METRICS-41-4 "Number of blocks of comments per statement in function" */

#include "dp_register.h"
#include "dp_if.h"
#include "dp_priv.h"
#include "dp_mailbox.h"
#include "dp_utils.h"
#include "mhdp_apb_regs.h"

#include "cdn_errno.h"
#include "cdn_stdtypes.h"
#include "cps_drv.h"

#include "dp_internal.h"

static void sendReadRegisterRequest(DP_PrivateData* pD, const DP_RegisterTransfer* transfer)
{
    messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_READ_REGISTER);
    messageWriteUint32(pD, transfer->addr);
    messageFinish(pD);
    messageTransmit(pD, transfer->busType);

}

static uint32_t getReadRegisterResponse(DP_PrivateData* pD, DP_RegisterTransfer* transfer)
{
    uint32_t retVal = CDN_EOK;
    uint32_t readAddr;

    messageReceive(pD, transfer->busType);
    if (0U == messageHeaderMatches(pD,
                                   MB_MODULE_ID_GENERAL,
                                   (uint8_t)GENERAL_READ_REGISTER)) {
        retVal = CDN_ENOEXEC;
    }

    if (CDN_EOK == retVal) {
        messageGetHeader(pD, NULL, NULL, NULL);
        messageReadUint32(pD, &readAddr);
        messageReadUint32(pD, &(transfer->val));

        if ((readAddr != transfer->addr) || (0U == readAddr)) {
            retVal = CDN_EIO;
        }
        transfer->addr = readAddr;
    }

    return retVal;
}

uint32_t DP_ReadRegister(DP_PrivateData*      pD,
                         DP_RegisterTransfer* transfer)
{
    uint32_t retVal = CDN_EOK;

    if ((NULL == pD) || (NULL == transfer) || (0U == transfer->addr)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        sendReadRegisterRequest(pD, transfer);
        retVal = getReadRegisterResponse(pD, transfer);
    }

    return retVal;
}

uint32_t DP_WriteRegister(DP_PrivateData* pD, const DP_RegisterTransfer* transfer)
{
    uint32_t retVal = CDN_EOK;

    if ((NULL == pD) || (NULL == transfer) || (0U == transfer->addr)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_WRITE_REGISTER);
        messageWriteUint32(pD, transfer->addr);
        messageWriteUint32(pD, transfer->val);
        messageFinish(pD);
        messageTransmit(pD, transfer->busType);
    }

    return retVal;
}

uint32_t DP_WriteField(DP_PrivateData* pD, const DP_WriteFieldRequest* request)
{
    uint32_t retVal = CDN_EOK;

    if ((NULL == pD) || (NULL == request)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        messageStart(pD, MB_MODULE_ID_GENERAL, (uint8_t)GENERAL_WRITE_FIELD);
        messageWriteUint32(pD, request->addr);
        messageWriteUint8(pD, request->startBit);
        messageWriteUint8(pD, request->bitCount);
        messageWriteUint32(pD, request->val);
        messageFinish(pD);
        messageTransmit(pD, DP_BUS_TYPE_APB);
    }

    return retVal;
}

uint32_t DP_ReadLocalRegister(DP_PrivateData* pD, DP_RegisterTransfer* transfer)
{
    uint32_t retVal = CDN_EOK;
    MHDP_ApbRegs* regBase;
    uintptr_t address;

    if ((NULL == pD) || (NULL == transfer)) {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal) {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "Cast pointer to integral and vice-versa" */
        regBase = selectRegBase(pD, transfer->busType);
        address = (uintptr_t)regBase + (transfer->addr);
        transfer->val = CPS_REG_READ((volatile uint32_t*)address);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    }

    return retVal;
}

uint32_t DP_WriteLocalRegister(DP_PrivateData* pD, const DP_RegisterTransfer* transfer)
{
    uint32_t retVal = CDN_EOK;
    MHDP_ApbRegs* regBase;
    uintptr_t address;

    if ((NULL == pD) || (NULL == transfer))
    {
        retVal = CDN_EINVAL;
    }

    if (CDN_EOK == retVal)
    {
        /* parasoft-begin-suppress MISRA2012-RULE-11_4-4 "Cast pointer to integral and vice-versa" */
        regBase = selectRegBase(pD, transfer->busType);
        address = (uintptr_t)regBase + (transfer->addr);
        CPS_REG_WRITE((volatile uint32_t*)address, transfer->val);
        /* parasoft-end-suppress MISRA2012-RULE-11_4-4 */
    }

    return retVal;
}

/* parasoft-end-suppress METRICS-41-4 */
