/*
 * Copyright (c) 2018, Texas Instruments Incorporated
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

#include <ti/csl/cslr_device.h>
#include <ti/drv/uart/UART_stdio.h>
#include <stdlib.h>
#include <ti/board/board.h>

#include "mailbox.h"

#define MPU_MBX_MAGIC_STR  0xFF33BB00

int main()
{
    Board_initCfg boardCfg;
    uint32_t msg = 0U;
    uint32_t ipu1Cpu0 = 0U;
	uint32_t eve1Flag = 0U;

    boardCfg = BOARD_INIT_UART_STDIO;

    /* Board Library Init. */
    Board_init(boardCfg);

    UART_printf("Multicore Boot Test application \n");

    /* Send message to IPU1_CPU0(MBX2_Q0) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_0,
                              MPU_MBX_MAGIC_STR)) ;
    
    while (1)
    {
        if ( !ipu1Cpu0 && MESSAGE_INVALID !=
            MailboxGetMessage(CSL_MPU_MAILBOX3_REGS, MAILBOX_QUEUE_0, &msg))
        {
            UART_printf("\n IPU1 CPU0 boot-up Successful \n");
            ipu1Cpu0 = 1;
            break;
        }
    }

    if ( ipu1Cpu0 == 1)
    {
        /* Send message to EVE1(MBX2_Q4) */
            while (MESSAGE_INVALID ==
                    MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_1,
                              MPU_MBX_MAGIC_STR)) ;

            /* Read message from EVE1 */
            while(1)
            {
                if(!eve1Flag && MESSAGE_INVALID !=
                    MailboxGetMessage(CSL_MPU_MAILBOX3_REGS, MAILBOX_QUEUE_1, &msg))
                {
                    UART_printf("\r\n EVE1 firmware loaded and boot-up successfully \r\n");
                    eve1Flag = 1;
                    break;
                }
            }
    }

    UART_printf("End of Program\n");
    while(1);
}
