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
 *  \file app_utils.c
 *
 *  \brief DSS example utility APIs for J7
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <ti/board/src/j721e_evm/include/board_control.h>
#include <ti/csl/soc/cslr_soc_ctrl_mmr.h>
#include <ti/drv/sciclient/sciclient.h>
#include <ti/drv/pm/pmlib.h>
#include "app_utils.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


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
    /* Set drive strength */
    CSL_REG32_WR(CSL_WKUP_CTRL_MMR0_CFG0_BASE +
                 CSL_WKUP_CTRL_MMR_CFG0_H_IO_DRVSTRNGTH0_PROXY, 0xFU);

    CSL_REG32_WR(CSL_WKUP_CTRL_MMR0_CFG0_BASE +
                 CSL_WKUP_CTRL_MMR_CFG0_V_IO_DRVSTRNGTH0_PROXY, 0xFU);
}

void App_configureLCD(uint32_t app_output)
{
    int32_t status = PM_SUCCESS;
    uint32_t regVal;

    if(APP_OUTPUT_HDMI == app_output)
    {
        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleClkParent(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK_PARENT_DPI0_EXT_CLKSEL_OUT0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(TISCI_DEV_DSS0,
                    TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                    TISCI_MSG_FLAG_AOP,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmModuleClkRequest(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_FUNC_CLK,
                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleClkFreq(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    148500000ULL,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmModuleClkRequest(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }
        if(PM_SUCCESS == status)
        {
            Board_control(BOARD_CTRL_CMD_SET_HDMI_MUX, (void*) 0U);
            Board_control(BOARD_CTRL_CMD_SET_HDMI_PD_HIGH, (void*) 0U);
        }
    }
    else
    {
        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(TISCI_DEV_SERDES_10G0,
                    TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                    TISCI_MSG_FLAG_AOP,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(TISCI_DEV_DSS_EDP0,
                    TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                    TISCI_MSG_FLAG_AOP,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        /* Very Ugly Hack: Select DPI0 clk from DPI1_clk */
        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleClkParent(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK_PARENT_DPI0_EXT_CLKSEL_OUT0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleState(TISCI_DEV_DSS0,
                    TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                    TISCI_MSG_FLAG_AOP,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmModuleClkRequest(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_FUNC_CLK,
                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmSetModuleClkFreq(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    148500000ULL,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            status = Sciclient_pmModuleClkRequest(TISCI_DEV_DSS0,
                    TISCI_DEV_DSS0_DSS_INST0_DPI_1_IN_2X_CLK,
                    TISCI_MSG_VALUE_CLOCK_SW_STATE_REQ,
                    0,
                    SCICLIENT_SERVICE_WAIT_FOREVER);
        }

        if(PM_SUCCESS == status)
        {
            regVal = CSL_REG32_RD(CSL_CTRL_MMR0_CFG0_BASE +
                    CSL_MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3);
            CSL_FINS(regVal,
                    MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3_DPI3_PCLK,
                    0x5U);
            CSL_REG32_WR(CSL_CTRL_MMR0_CFG0_BASE +
                    CSL_MAIN_CTRL_MMR_CFG0_DSS_DISPC0_CLKSEL3, regVal);
        }
    }

}
