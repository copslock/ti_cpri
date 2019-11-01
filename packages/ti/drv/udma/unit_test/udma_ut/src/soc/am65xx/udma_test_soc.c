/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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
 *  \file udma_test_soc_v0.c
 *
 *  \brief UDMA SOC specific file for AM65xx.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

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

int32_t udmaTestPrintPsilMacro(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_0trace(taskObj->traceMask, GT_INFO1, " \r\n");
    GT_0trace(taskObj->traceMask, GT_INFO1,
              " List of all PSIL Threads and Counts:\r\n");
    GT_0trace(taskObj->traceMask, GT_INFO1,
              " ------------------------------------\r\n");

    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_SAUL0_TX       : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_SAUL0_TX, UDMA_PSIL_CH_MAIN_SAUL0_TX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G0_TX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G0_TX, UDMA_PSIL_CH_MAIN_ICSS_G0_TX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G1_TX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G1_TX, UDMA_PSIL_CH_MAIN_ICSS_G1_TX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G2_TX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G2_TX, UDMA_PSIL_CH_MAIN_ICSS_G2_TX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_CAL0_TX        : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_CAL0_TX, UDMA_PSIL_CH_MAIN_CAL0_TX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MCU_CPSW0_TX        : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MCU_CPSW0_TX, UDMA_PSIL_CH_MCU_CPSW0_TX_CNT);
    GT_0trace(taskObj->traceMask, GT_INFO1, " \r\n");

    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_SAUL0_RX       : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_SAUL0_RX, UDMA_PSIL_CH_MAIN_SAUL0_RX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G0_RX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G0_RX, UDMA_PSIL_CH_MAIN_ICSS_G0_RX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G1_RX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G1_RX, UDMA_PSIL_CH_MAIN_ICSS_G1_RX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_ICSS_G2_RX     : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_ICSS_G2_RX, UDMA_PSIL_CH_MAIN_ICSS_G2_RX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MAIN_CAL0_RX        : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MAIN_CAL0_RX, UDMA_PSIL_CH_MAIN_CAL0_RX_CNT);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " MCU_CPSW0_RX        : Thread Offset: 0x%0.4X, Thread Count: %d\r\n",
              UDMA_PSIL_CH_MCU_CPSW0_RX, UDMA_PSIL_CH_MCU_CPSW0_RX_CNT);

    return (retVal);
}

int32_t udmaTestPrintPdmaMacro(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_0trace(taskObj->traceMask, GT_INFO1, " \r\n");
    GT_0trace(taskObj->traceMask, GT_INFO1,
              " List of all PDMA Threads:\r\n");
    GT_0trace(taskObj->traceMask, GT_INFO1,
              " -------------------------\r\n");

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP0_TX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP1_TX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP2_TX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH0_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH1_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH2_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH3_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH0_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH1_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH2_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH3_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH0_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH1_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH2_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH3_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH0_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH1_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH2_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH3_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH0_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH1_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH2_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH3_TX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART0_TX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART1_TX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART2_TX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH0_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH1_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH2_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH3_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH0_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH1_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH2_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH3_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH0_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH1_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH2_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH3_TX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH3_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH0_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH1_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH2_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH0_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH1_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH1_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH2_TX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH2_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_UART0_TX                : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_UART0_TX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP0_RX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP1_RX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCASP2_RX              : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCASP2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH0_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH1_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH2_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI0_CH3_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI0_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH0_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH1_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH2_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI1_CH3_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI1_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH0_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH1_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH2_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI2_CH3_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI2_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH0_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH1_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH2_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI3_CH3_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI3_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH0_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH1_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH2_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_MCSPI4_CH3_RX          : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_MCSPI4_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART0_RX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART1_RX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MAIN_UART2_RX               : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MAIN_UART2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_ADC0_CH0_RX             : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_ADC0_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_ADC0_CH1_RX             : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_ADC0_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_ADC1_CH0_RX             : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_ADC1_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_ADC1_CH1_RX             : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_ADC1_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH0_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH1_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH2_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI0_CH3_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI0_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH0_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH1_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH2_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI1_CH3_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI1_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH0_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH1_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH2_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCSPI2_CH3_RX           : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCSPI2_CH3_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH0_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH1_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN0_CH2_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN0_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH0_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH0_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH1_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH1_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_MCAN1_CH2_RX            : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_MCAN1_CH2_RX);
    GT_1trace(taskObj->traceMask, GT_INFO1,
              " MCU_UART0_RX                : Thread ID: 0x%0.4X\r\n",
              UDMA_PDMA_CH_MCU_UART0_RX);

    return (retVal);
}
