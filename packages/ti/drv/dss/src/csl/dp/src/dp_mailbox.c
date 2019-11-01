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
 * dp_mailbox.c
 *
 ******************************************************************************
 */

#include "dp_if.h"
#include "dp_priv.h"
#include "dp_sanity.h"

#include "dp_mailbox.h"
#include "dp_utils.h"
#include "mhdp_apb_regs.h"

#include "cdn_errno.h"
#include "cdn_stdint.h"
#include "cps_drv.h"

void messageStart(DP_PrivateData *pD, uint8_t module, uint8_t opcode)
{
    /* put IDs of module and code into txBuffer */
    pD->txBuffer[0] = opcode;
    pD->txBuffer[1] = module;

    /* set size of message header */
    pD->txi = DP_MAILBOX_HEADER_SIZE;
}

void messageWriteUint8(DP_PrivateData *pD, uint8_t value)
{
    /* put 1B message to write into buffer */
    pD->txBuffer[pD->txi] = value;

    /* add size of message to counter */
    pD->txi += 1U;
}

void messageWriteUint16(DP_PrivateData *pD, uint16_t value)
{
    /* put 2B message to write into buffer */
    pD->txBuffer[pD->txi] = (uint8_t)((value & 0xFF00U) >> 8);
    pD->txBuffer[pD->txi + 1U] = (uint8_t)(value & 0x00FFU);

    /* add size of message to counter */
    pD->txi += 2U;
}

void messageWriteUint32(DP_PrivateData *pD, uint32_t value)
{
    /* put 4B message to write into buffer */
    pD->txBuffer[pD->txi] = (uint8_t)((value & 0xFF000000U) >> 24);
    pD->txBuffer[pD->txi + 1U] = (uint8_t)((value & 0x00FF0000U) >> 16);
    pD->txBuffer[pD->txi + 2U] = (uint8_t)((value & 0x0000FF00U) >> 8);
    pD->txBuffer[pD->txi + 3U] = (uint8_t)(value & 0x000000FFU);

    /* add size of message to counter */
    pD->txi += 4U;
}

void messageWrite3Bytes(DP_PrivateData *pD, uint32_t value)
{
    /* put 3B message to write into buffer */
    pD->txBuffer[pD->txi] = (uint8_t)((value & 0x00FF0000U) >> 16);
    pD->txBuffer[pD->txi + 1U]  = (uint8_t)((value & 0x0000FF00U) >> 8);
    pD->txBuffer[pD->txi + 2U]  = (uint8_t)(value & 0x000000FFU);

    /* add size of message to counter */
    pD->txi += 3U;
}

void messageWriteBuffer(DP_PrivateData *pD, const uint8_t *buffer, uint16_t length)
{
    /* put message to write into buffer */
    CPS_BufferCopy(&(pD->txBuffer[pD->txi]), buffer, length);

    /* add size of message to counter */
    pD->txi += length;
}

void messageFinish(DP_PrivateData *pD)
{
    uint16_t length = (uint16_t)(pD->txi - DP_MAILBOX_HEADER_SIZE);

    /* put length of message into buffer */
    pD->txBuffer[2] = (uint8_t)((length & 0xFF00U) >> 8);
    pD->txBuffer[3] = (uint8_t)(length & 0x00FFU);
}

void messageTransmit(DP_PrivateData *pD, DP_BusType busType)
{
    uint16_t i = 0U;
    uint32_t mailboxFull;
    MHDP_ApbRegs* regBase = selectRegBase(pD, busType);

    /* loop through all bytes to send */
    while (i < pD->txi)
    {
        mailboxFull = CPS_REG_READ(&regBase->mhdp_apb_regs.MAILBOX_FULL_ADDR_p);

        /* test if mailbox is not full */
        if (0U == mailboxFull) {
            CPS_REG_WRITE(&regBase->mhdp_apb_regs.mailbox0_wr_data_p, pD->txBuffer[i]);
            i++;
        }
    }
}

uint32_t DP_CheckResponse(const DP_PrivateData* pD, bool* awaits, DP_BusType busType)
{
    uint32_t mailboxEmpty;
    uint32_t retVal;

    retVal = DP_CheckResponseSF(pD, awaits, busType);

    if (CDN_EOK == retVal)
    {
        /* select properly regBase */
        MHDP_ApbRegs* regBase = selectRegBase(pD, busType);

        mailboxEmpty = CPS_REG_READ(&regBase->mhdp_apb_regs.MAILBOX_EMPTY_ADDR_p);

        /* set if message was not received */
        *awaits = (0U == mailboxEmpty);
    }

    return retVal;
}

void messageReceive(DP_PrivateData *pD, DP_BusType busType)
{
    uint16_t i = 0U;
    uint16_t messageSize;
    uint32_t mailboxEmpty;
    uint32_t readVal;
    MHDP_ApbRegs* regBase = selectRegBase(pD, busType);

    /* read header first, to get size of message */
    while (i < DP_MAILBOX_HEADER_SIZE)
    {
        mailboxEmpty = CPS_REG_READ(&regBase->mhdp_apb_regs.MAILBOX_EMPTY_ADDR_p);

        /* test if mailbox is not empty */
        if (0U == mailboxEmpty)
        {
            readVal = CPS_REG_READ(&regBase->mhdp_apb_regs.mailbox0_rd_data_p);
            pD->rxBuffer[i] = (uint8_t)(readVal & 0x000000FFU);
            i++;
        }
    }

    messageSize = (uint16_t)(((uint16_t)pD->rxBuffer[2] << 8) |
                             (uint16_t)pD->rxBuffer[3]);
    messageSize += DP_MAILBOX_HEADER_SIZE;

    /* read message */
    while (i < messageSize)
    {
        mailboxEmpty = CPS_REG_READ(&regBase->mhdp_apb_regs.MAILBOX_EMPTY_ADDR_p);

        /* test if mailbox is not empty */
        if (0U == mailboxEmpty)
        {
            readVal = CPS_REG_READ(&regBase->mhdp_apb_regs.mailbox0_rd_data_p);
            pD->rxBuffer[i] = (uint8_t)(readVal & 0x000000FFU);
            i++;
        }
    }
}

void messageGetHeader(DP_PrivateData *pD, uint8_t *module, uint8_t *opcode, uint16_t *length)
{
    /* get header of received package */
    if (NULL != opcode)
    {
        *opcode = pD->rxBuffer[0];
    }

    /* get moduleID from received package */
    if (NULL != module)
    {
        *module = pD->rxBuffer[1];
    }

    /* get length of message */
    if (NULL != length)
    {
        *length = (uint16_t)(((uint16_t)pD->rxBuffer[2] << 8) |
                             (uint16_t)pD->rxBuffer[3]);
    }

    pD->rxi = DP_MAILBOX_HEADER_SIZE;
}

uint8_t messageHeaderMatches(const DP_PrivateData *pD, uint8_t module, uint8_t opcode)
{
    uint8_t retVal = 0U;

    /* test if received header and moduleID as same as expected */
    if ((opcode == pD->rxBuffer[0]) && (module == pD->rxBuffer[1]))
    {
        retVal = 1U;
    }

    return retVal;
}

void messageReadUint8(DP_PrivateData *pD, uint8_t *value)
{
    /* read 1B from buffer */
    *value = pD->rxBuffer[pD->rxi];

    /* add message size to counter */
    pD->rxi += 1U;
}

void messageReadUint16(DP_PrivateData *pD, uint16_t *value)
{
    /* read 2B from buffer */
    *value = (((uint16_t)pD->rxBuffer[pD->rxi] << 8) |
              (uint16_t)pD->rxBuffer[pD->rxi + 1U]);

    /* add message size to counter */
    pD->rxi += 2U;
}

void messageReadUint32(DP_PrivateData *pD, uint32_t *value)
{
    /* read 4B from buffer */
    *value = (((uint32_t)pD->rxBuffer[pD->rxi] << 24) |
              ((uint32_t)pD->rxBuffer[pD->rxi + 1U] << 16) |
              ((uint32_t)pD->rxBuffer[pD->rxi + 2U] << 8) |
              (uint32_t)pD->rxBuffer[pD->rxi + 3U]);

    /* add message size to counter */
    pD->rxi += 4U;
}

void messageRead3Bytes(DP_PrivateData *pD, uint32_t *value)
{
    /* read 3B from buffer */
    *value = (((uint32_t)pD->rxBuffer[pD->rxi] << 16) |
              ((uint32_t)pD->rxBuffer[pD->rxi + 1U] << 8) |
              (uint32_t)pD->rxBuffer[pD->rxi + 2U]);

    /* add message size to counter */
    pD->rxi += 3U;
}

void messageReadBuffer(DP_PrivateData *pD, uint8_t *buffer, uint16_t length)
{
    /* read message from buffer */
    CPS_BufferCopy(buffer, &(pD->rxBuffer[pD->rxi]), length);

    /* add size of message to counter */
    pD->rxi += length;
}
