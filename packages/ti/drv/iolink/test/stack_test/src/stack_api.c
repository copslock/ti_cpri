/*
 *  Copyright (C) 2018 Texas Instruments Incorporated - http:;www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 *  @file       stack_api.c
 *
 *  @brief      IO-Link Master stack interface
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <ti/drv/iolink/test/stack_test/src/stack_api.h>
#include <mst_pl.h>


IOLINK_v0_Callbacks iolinkCallbacks =
{
    IO_Link_cycleTimerCallback,
    IO_Link_swTimerCallback,
    IO_Link_adjTimerCallback,
    IO_Link_xferRspCallback,
    IO_Link_xferErrRspCallback
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

extern IOLINK_Handle iolinkHandles[];

int8_t IO_Link_sendCommand(uint8_t instance, uint8_t port, uint8_t command, uint8_t arg)
{
    int8_t   retVal = 0;
    uint32_t data[3];

    if ((iolinkHandles[instance] == NULL) || (port >= (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        retVal = -1;
    }

    if ((command != IO_LINK_COMMAND_STARTPULSE) && ((command != IO_LINK_COMMAND_SETCOM)))
    {
        retVal = -1;
    }

    if (retVal == 0)
    {
        data[0] = (uint32_t)port;
        data[1] = (uint32_t)command;
        data[2] = (uint32_t)arg;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_SEND_CMD, (void *)data);
    }

    return (retVal);
}

int8_t IO_Link_setBuffer(uint8_t instance, uint8_t port, uint8_t txBufLen, uint8_t rxBufLen, uint8_t *txBuf, uint8_t *rxBuf)
{
    int8_t   retVal = 0;
    uint32_t data[5];

    if ((iolinkHandles[instance] == NULL) || (port >= (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        retVal = -1;
    }

    if (retVal == 0)
    {
        data[0] = (uint32_t)port;
        data[1] = (uint32_t)txBufLen;
        data[2] = (uint32_t)rxBufLen;
        data[3] = (uint32_t)txBuf;
        data[4] = (uint32_t)rxBuf;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_SET_XFER_BUFFER, (void *)data);
    }

    return (retVal);
}

void IO_Link_sendBuffer(uint8_t instance, uint8_t port)
{
    uint32_t  channel;

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        channel = (uint32_t)port;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_START_XFER, (void *)&channel);
    }
}

void IO_Link_start10msTimer(uint8_t instance, uint8_t port)
{
    uint32_t data[4] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = IOLINK_TIMER_TYPE_10MS;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_START_TIMER, (void *)data);
    }
}

void IO_Link_stop10msTimer(uint8_t instance, uint8_t port)
{
    uint32_t data[2] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = IOLINK_TIMER_TYPE_10MS;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_STOP_TIMER, (void *)data);
    }
}

void IO_Link_setCycleTimer(uint8_t instance, uint8_t port, uint32_t delay)
{
    uint32_t data[2] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = delay;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_SET_CYCLE_TIMER, (void *)data);
    }
}

void IO_Link_startCycleTimer(uint8_t instance, uint8_t port)
{
    uint32_t data[4] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = IOLINK_TIMER_TYPE_CYCLE;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_START_TIMER, (void *)data);
    }
}

void IO_Link_stopCycleTimer(uint8_t instance, uint8_t port)
{
    uint32_t data[2] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = IOLINK_TIMER_TYPE_CYCLE;
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_STOP_TIMER, (void *)data);
    }
}

void IO_Link_startAdjustableTimer(uint8_t instance, uint8_t port, uint8_t type, double t)
{
    uint32_t data[4] = {0, };

    if ((iolinkHandles[instance] != NULL) && (port < (uint8_t)IOLINK_MAX_NUM_CHN))
    {
        data[0] = (uint32_t)port;
        data[1] = IOLINK_TIMER_TYPE_ADJ;
        data[2] = (uint32_t)type;
        data[3] = (uint32_t)(t * 24.0); /* t in usec, t * 24 timeout match count with 24 MHz clock */
        IOLINK_control(iolinkHandles[instance], IOLINK_CTRL_START_TIMER, (void *)data);
    }
}

void IO_Link_stopAdjustableTimer()
{
    uint32_t timerType = IOLINK_TIMER_TYPE_ADJ;

    IOLINK_control(iolinkHandles[0], IOLINK_CTRL_STOP_TIMER, (void *)&timerType);
}

void IO_Link_cycleTimerCallback(IOLINK_Handle handle, uint32_t channel)
{
    MPL_TimerCallback((uint8_t)channel, MPL_CYCLE_TIMER_DELAY);
}

void IO_Link_swTimerCallback(IOLINK_Handle handle, uint32_t channel)
{
    MPL_SWTimerCallback((uint8_t)channel);
}

void IO_Link_adjTimerCallback(IOLINK_Handle handle, uint32_t channel, uint32_t delayType)
{
    if (delayType == IOLINK_TIMER_ADJ_TREN)
    {
        MPL_TimerCallback((uint8_t)channel, MPL_TREN_MASTER_MESSAGE_DELAY);
    }
    else
    {
        MPL_TimerCallback((uint8_t)channel, MPL_TDMT_MASTER_MESSAGE_DELAY);
    }
}

void IO_Link_xferRspCallback(IOLINK_Handle handle, uint32_t channel)
{
    MPL_SendRecv_rsp((uint8_t)channel);
}

void IO_Link_xferErrRspCallback(IOLINK_Handle handle, uint32_t channel)
{
    MPL_SendRecvError_rsp((uint8_t)channel);
}
