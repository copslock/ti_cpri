/**
 *  \file   sbl_prcm.c
 *
 *  \brief  This file contains PRCM functions used in SBL to enable clocks
 *		    to the slave cores.
 *
 */

/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
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

/* TI RTOS header files */
#include <stdint.h>
#include <ti/csl/cslr_device.h>
#include <ti/csl/hw_types.h>
#include <ti/drv/uart/UART_stdio.h>

#include "sbl_prcm.h"

void SBL_PRCMModuleEnable(uint32_t domainOffset,
                          uint32_t clkCtrlReg,
                          uint32_t clkStCtrlReg,
                          uint32_t clkActMask)
{
    /* Enable the module */
    HW_WR_REG32(domainOffset + clkCtrlReg, PRCM_MODULEMODE_AUTO);
    /* Check for module enable status */
    while(PRCM_MODULEMODE_AUTO !=
        (HW_RD_REG32(domainOffset + clkCtrlReg) & PRCM_MODULEMODE_MASK));

    /* Check clock activity - ungated */
    while(clkActMask != (HW_RD_REG32(domainOffset + clkStCtrlReg) & clkActMask));
}

void SBL_PRCMModuleDisable(uint32_t domainOffset,
                          uint32_t clkCtrlReg,
                          uint32_t clkStCtrlReg)
{
    /* Enable the module */
    HW_WR_REG32(domainOffset + clkCtrlReg, PRCM_MODULEMODE_DISABLE);

    /* Check for module enable status */
    while(PRCM_MODULEMODE_DISABLE !=
        (HW_RD_REG32(domainOffset + clkCtrlReg) & PRCM_MODULEMODE_MASK));
}

void SBL_PRCMSetClkOperMode(uint32_t domainOffset,
                            uint32_t clkStCtrlReg,
                            uint32_t clkMode)
{
    /* Enable the module */
    HW_WR_FIELD32(domainOffset + clkStCtrlReg, CM_DSP1_CLKSTCTRL_CLKTRCTRL, clkMode);

    while(clkMode != HW_RD_FIELD32(domainOffset + clkStCtrlReg,
        CM_DSP1_CLKSTCTRL_CLKTRCTRL));
}

void SBL_ResetAssert(uint32_t ctrlRegAddr, uint32_t ctrlregShift)
{
    uint32_t resetCtrlMask;

    resetCtrlMask = (uint32_t) 1U << ctrlregShift;
    HW_WR_FIELD32_RAW(ctrlRegAddr, resetCtrlMask, ctrlregShift, 0x1U);
}

void SBL_ResetRelease(uint32_t ctrlRegAddr,
                 uint32_t ctrlregShift,
                 uint32_t stsRegAddr,
                 uint32_t stsRegShift)
{
    uint32_t rstStsMask, resetCtrlMask;

    resetCtrlMask = (uint32_t) 1U << ctrlregShift;
    HW_WR_FIELD32_RAW(ctrlRegAddr, resetCtrlMask, ctrlregShift, 0x0U);

    rstStsMask = (uint32_t) 1U << stsRegShift;
    while(0x1U != ((uint32_t) HW_RD_FIELD32_RAW(stsRegAddr, rstStsMask,
                            stsRegShift)));
}

void SBL_ResetGetStatus(uint32_t stsRegAddr,
                   uint32_t stsRegShift,
                   uint32_t *rStsMask)
{
    uint32_t rstStsMask;

    rstStsMask = (uint32_t) 1U << stsRegShift;
    *rStsMask = (uint32_t) HW_RD_FIELD32_RAW(stsRegAddr, rstStsMask,
                            stsRegShift);
}

void SBL_ResetClearStatus(uint32_t stsRegAddr,
                          uint32_t stsRegShift)
{
    uint32_t rstStMask;

    rstStMask = (uint32_t) 1U << stsRegShift;
    HW_WR_FIELD32_RAW(stsRegAddr, rstStMask, stsRegShift, (uint32_t) 0x1U);
}
