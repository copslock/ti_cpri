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

/** ============================================================================
 *  @file       stack_api.h
 *
 *  @brief      IO-Link Master stack interface
 *
 *  ============================================================================
 */

#ifndef STACK_API_H
#define STACK_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drv/iolink/src/v0/IOLINK_v0.h>

/**
 *  \brief IO-Link handle definitions
 */
#define IO_LINK_HANDLE_ICSS0_PRU0        ((uint8_t)IOLINK_HANDLE_ICSS0_PRU0)
#define IO_LINK_HANDLE_ICSS0_PRU1        ((uint8_t)IOLINK_HANDLE_ICSS0_PRU1)
#define IO_LINK_HANDLE_ICSS1_PRU0        ((uint8_t)IOLINK_HANDLE_ICSS1_PRU0)
#define IO_LINK_HANDLE_ICSS1_PRU1        ((uint8_t)IOLINK_HANDLE_ICSS1_PRU1)

/**
 *  \brief IO-Link commands
 */
#define IO_LINK_COMMAND_STARTPULSE       ((uint8_t)IOLINK_COMMAND_STARTPULSE)
#define IO_LINK_COMMAND_SETCOM           ((uint8_t)IOLINK_COMMAND_SETCOM)

/**
 *  \brief IO-Link adjuster timer types
 */
#define IO_LINK_TIMER_TYPE_TREN         ((uint8_t)IOLINK_TIMER_ADJ_TREN)
#define IO_LINK_TIMER_TYPE_TDMT         ((uint8_t)IOLINK_TIMER_ADJ_TDMT)

/*
 * @brief  Function to send user specified command to the PRU
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @param  command   command code
 *
 * @param  arg       argument of the command
 *
 * @return 0: success, -1: fail.
 *
 * @note: commands:
 *
 * command = IO_LINK_COMMAND_STARTPULSE
 * - This command will generate a startpulse
 * - arg: can be set to anything
 *
 * command = IO_LINK_COMMAND_SETCOM
 * - This command will set the COM rate to arg
 * - arg: sets the COM rate (1 = COM1; 2 = COM2; 3 = COM3)
 */
extern int8_t IO_Link_sendCommand(uint8_t instance, uint8_t port, uint8_t command, uint8_t arg);

/*
 * @brief  Function to set the channel tx and rx buffer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @param  txBufLen  length (<= 128) of the the data in bytes stored in the tx buffer
 *
 * @param  rxBufLen  length (<= 128) of the the data in bytes stored in the rx buffer
 *
 * @param  txBuf     tx buffer pointer
 *
 * @param  rxBuf     rx buffer pointer
 *
 * @return 0: success, -1: fail.
 *
 */
extern int8_t IO_Link_setBuffer(uint8_t instance, uint8_t port, uint8_t txBufLen, uint8_t rxBufLen, uint8_t *txBuf, uint8_t *rxBuf);

/*
 * @brief  Function to send the data to PRU
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @return none
 *
 * @note: This API call start a new communication cycle
 *
 */
extern void IO_Link_sendBuffer(uint8_t instance, uint8_t port);

/*
 * @brief  Function to start the 10 ms timer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_start10msTimer(uint8_t instance, uint8_t port);

/*
 * @brief  Function to stop the 10 ms timer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_stop10msTimer(uint8_t instance, uint8_t port);

/*
 * @brief  Function to control the cycle time of a channel
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @param  delay     delay time of the cycle timer
 *
 * @return none
 *
 */
extern void IO_Link_setCycleTimer(uint8_t instance, uint8_t port, uint32_t delay);

/*
 * @brief  Function to start the cycle timer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_startCycleTimer(uint8_t instance, uint8_t port);

/*
 * @brief  Function to stop the cycle timer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_stopCycleTimer(uint8_t instance, uint8_t port);

/*
 * @brief  Function to start the adjustable timer
 *
 * @param  instance  IOLINK ICSS PRU instance #
 *
 * @param  port      IOLINK channel #
 *
 * @param  type      adjustable timer type IOLINK_TIMER_TYPE_TREN or IOLINK_TIMER_TYPE_TDMT
 *
 * @param  t         adjustable time t in usec
 *
 * @return none
 *
 * @note: the adjustable timer is used to measure the start up sequence timings.
 *        It will generate an Interrupt as soon as the time t (given in us) is elapsed.
 *
 */
extern void IO_Link_startAdjustableTimer(uint8_t instance, uint8_t port, uint8_t type, double t);

/*
 * @brief  Function to stop the adjustable timer
 *
 * @return none
 */
extern void IO_Link_stopAdjustableTimer(void);

/*
 * @brief  10usec timer callback function to the stack
 *
 * @param  handle    IOLINK handle #
 *
 * @param  channel   IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_cycleTimerCallback(IOLINK_Handle handle, uint32_t channel);

/*
 * @brief  10ms timer callback function to the stack
 *
 * @param  handle    IOLINK handle #
 *
 * @param  channel   IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_swTimerCallback(IOLINK_Handle handle, uint32_t channel);

/*
 * @brief  adjustable timer callback function to the stack
 *
 * @param  handle    IOLINK handle #
 *
 * @param  channel   IOLINK channel #
 *
 * @param  delayType Adjustable timer delay type
 *
 * @return none
 *
 */
extern void IO_Link_adjTimerCallback(IOLINK_Handle handle, uint32_t channel, uint32_t delayType);

/*
 * @brief  data transfer response callback function to the stack
 *
 * @param  handle    IOLINK handle #
 *
 * @param  channel   IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_xferRspCallback(IOLINK_Handle handle, uint32_t channel);

/*
 * @brief  data transfer error response callback function to the stack
 *
 * @param  handle    IOLINK handle #
 *
 * @param  channel   IOLINK channel #
 *
 * @return none
 *
 */
extern void IO_Link_xferErrRspCallback(IOLINK_Handle handle, uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* STACK_API_H */
