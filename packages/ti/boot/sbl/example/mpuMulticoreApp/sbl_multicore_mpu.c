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
    uint32_t dsp1Flag = 0U;
    uint32_t ipu1Cpu0 = 0U;
    #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
    uint32_t mpu1Flag = 0U;
    uint32_t dsp2Flag = 0U;
    uint32_t ipu2Cpu0 = 0U;
    #endif

    boardCfg = BOARD_INIT_UART_STDIO;

    /* Board Library Init. */
    Board_init(boardCfg);

    UART_printf("Multicore Boot Test application \n");

    /* Send message to A15 CORE1(MBX2_Q10) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_10,
                              MPU_MBX_MAGIC_STR)) ;

    /* Send message to DSP1(MBX2_Q4) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_4,
                              MPU_MBX_MAGIC_STR)) ;

    #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
    /* Send message to DSP2(MBX2_Q5) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_5,
                              MPU_MBX_MAGIC_STR)) ;
    #endif

    /* Send message to IPU1_CPU0(MBX2_Q0) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_0,
                              MPU_MBX_MAGIC_STR)) ;

    #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
    /* Send message to IPU2_CPU0(MBX2_Q0) */
    while (MESSAGE_INVALID ==
           MailboxSendMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_2,
                              MPU_MBX_MAGIC_STR)) ;
    #endif

    while(1)
    {
		#if defined (AM572x_BUILD) || defined (AM574x_BUILD)
        if ( !ipu1Cpu0 && MESSAGE_INVALID !=
            MailboxGetMessage(CSL_MPU_MAILBOX2_REGS, MAILBOX_QUEUE_11, &msg))
        {
            UART_printf("\n MPU Core-1 boot-up Successful \n");
            mpu1Flag = 1;
        }
		#endif

        /* Read message from DSP1 */
        if (!dsp1Flag && MESSAGE_INVALID !=
            MailboxGetMessage(CSL_MPU_MAILBOX7_REGS, MAILBOX_QUEUE_0, &msg))
        {
            UART_printf("\n DSP1 boot-up Successful \n");
            dsp1Flag = 1;
        }

        #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
        /* Read message from DSP2 */
        if (!dsp2Flag && MESSAGE_INVALID !=
            MailboxGetMessage(CSL_MPU_MAILBOX8_REGS, MAILBOX_QUEUE_0, &msg))
        {
            UART_printf("\n DSP2 boot-up Successful \n");
            dsp2Flag = 1;
        }
        #endif

        if ( !ipu1Cpu0 && MESSAGE_INVALID !=
            MailboxGetMessage(CSL_MPU_MAILBOX3_REGS, MAILBOX_QUEUE_0, &msg))
        {
            UART_printf("\n IPU1 CPU0 boot-up Successful \n");
            ipu1Cpu0 = 1;
        }

        #if defined (AM572x_BUILD) || defined (AM574x_BUILD)
        if (!ipu2Cpu0 && MESSAGE_INVALID !=
           MailboxGetMessage(CSL_MPU_MAILBOX5_REGS, MAILBOX_QUEUE_0, &msg))
        {
            UART_printf("\n IPU2 CPU0 boot-up Successful \n");
            ipu2Cpu0 = 1;
        }
        #endif
    }
}
