/*
 *  Copyright (c) Texas Instruments Incorporated 2018
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file app_utils_am65xx.c
 *
 *  \brief DSS example utility APIs for AM65xx
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <ti/csl/csl.h>
#include <ti/board/board.h>
#include <ti/csl/csl_gpio.h>
#include <ti/board/src/evmKeystone3/include/board_i2c_io_exp.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/pm/pmlib.h>
#include "app_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* PWM pin number */
#define PWM_GPIO_PIN                    (86U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void App_configureSoC(void)
{
    /* Configure OLDI IOs */
    uint32_t regVal;
    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                          CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT0_IO_CTRL);
    CSL_FINS(regVal,
             MAIN_CTRL_MMR_CFG0_OLDI_DAT0_IO_CTRL_PWRDN_TX,
             FALSE);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                 CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT0_IO_CTRL, regVal);

    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                          CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT1_IO_CTRL);
    CSL_FINS(regVal,
             MAIN_CTRL_MMR_CFG0_OLDI_DAT1_IO_CTRL_PWRDN_TX,
             FALSE);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                 CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT1_IO_CTRL, regVal);

    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                          CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT2_IO_CTRL);
    CSL_FINS(regVal,
             MAIN_CTRL_MMR_CFG0_OLDI_DAT2_IO_CTRL_PWRDN_TX,
             FALSE);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                 CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT2_IO_CTRL, regVal);

    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                          CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT3_IO_CTRL);
    CSL_FINS(regVal,
             MAIN_CTRL_MMR_CFG0_OLDI_DAT3_IO_CTRL_PWRDN_TX,
             FALSE);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                 CSL_MAIN_CTRL_MMR_CFG0_OLDI_DAT3_IO_CTRL, regVal);

    regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                          CSL_MAIN_CTRL_MMR_CFG0_OLDI_CLK_IO_CTRL);
    CSL_FINS(regVal,
             MAIN_CTRL_MMR_CFG0_OLDI_CLK_IO_CTRL_PWRDN_TX,
             FALSE);
    CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                 CSL_MAIN_CTRL_MMR_CFG0_OLDI_CLK_IO_CTRL, regVal);
}

void App_configureLCD(uint32_t app_output)
{
    int32_t status = PM_SUCCESS;
    Sciclient_init(NULL);
    status = PMLIBClkRateSet(TISCI_DEV_OLDI_TX_CORE_MAIN_0,
                             TISCI_DEV_OLDI_TX_CORE_MAIN_0_BUS_OLDI_PLL_CLK,
                             497500000);

    if(PM_SUCCESS == status)
    {
        Board_i2cIoExpInit();
        Board_i2cIoExpSetPinDirection(BOARD_I2C_IOEXP_DEVICE2_ADDR,
                                    PORTNUM_1,
                                    PIN_NUM_0,
                                    PIN_DIRECTION_OUTPUT);

        /* Pull the LCD enable to high */
        Board_i2cIoExpPinLevelSet(BOARD_I2C_IOEXP_DEVICE2_ADDR,
                                PORTNUM_1,
                                PIN_NUM_0,
                                GPIO_SIGNAL_LEVEL_HIGH);

        /* Set PWM pin as GPIO for max brightness */
        GPIOSetDirMode_v0(CSL_GPIO1_BASE, PWM_GPIO_PIN, GPIO_DIRECTION_OUTPUT);
        GPIOPinWrite_v0(CSL_GPIO1_BASE, PWM_GPIO_PIN, GPIO_PIN_LOW);
        CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                    CSL_MAIN_CTRL_MMR_CFG0_PADCONFIG190, 0x50007U);
    }
}
