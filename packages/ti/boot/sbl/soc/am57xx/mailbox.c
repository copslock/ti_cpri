/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ti/csl/tistdtypes.h>
#include "mailbox.h"

#define HW_WR_REG32(addr, data)   *(unsigned int*)(addr) =(unsigned int)(data)

#define HW_RD_REG32(x)             (*((volatile uint32_t *)(x)))

 uint32_t MailboxGetMessage(uint32_t baseAdd, uint32_t queueId, uint32_t *msgPtr)
{
    uint32_t msgCount;
    uint32_t retval;

    msgCount = HW_RD_REG32(baseAdd + MAILBOX_MSGSTATUS(queueId));

    if (msgCount > 0U)
    {
        /*    Read message    */
        *msgPtr = HW_RD_REG32(baseAdd + MAILBOX_MESSAGE(queueId));
        retval  = MESSAGE_VALID;
    }
    else
    {
        /*    Queue empty*/
        retval = MESSAGE_INVALID;
    }

    return retval;
}

/* ========================================================================== */
/**
 * @fn This function writes message in the queue
 *
 * @see hal_mailbox.h
 */
uint32_t MailboxSendMessage(uint32_t baseAdd, uint32_t queueId, uint32_t msg)
{
    uint32_t fifoFull;
    uint32_t retval;

    /* Read the FIFO Status */
    fifoFull = HW_RD_REG32(baseAdd + MAILBOX_FIFOSTATUS(queueId));
    if (fifoFull == 0)
    {
        /* FIFO not full write msg */
        HW_WR_REG32(baseAdd + MAILBOX_MESSAGE(queueId), msg);
        retval = MESSAGE_VALID;
    }
    else
    {
        retval = MESSAGE_INVALID;
    }

    return retval;
}
