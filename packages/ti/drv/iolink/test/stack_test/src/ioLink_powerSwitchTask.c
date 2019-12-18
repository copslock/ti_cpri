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
 *              file:    ioLink_powerSwitchTask.c
 *
 *              brief:   PRU IO-Link master power switch driver
 */

//NOTE: This file is not final and should be used with caution.
//      There is a quick fix applied to this driver which deactivates its functionality and switches all outputs on.

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/BIOS.h>
#include <ti/drv/gpio/GPIO.h>
#include <xdc/cfg/global.h>

#include <ti/board/board.h>
#include <ti/starterware/include/hw/am437x.h>
#include <ti/starterware/include/hw/hw_control_am43xx.h>
#include <ti/starterware/include/hw/hw_tsc_adc_ss.h>
#include <ti/starterware/include/tsc_adc_ss.h>

#include <ti/csl/src/ip/gpio/V1/hw_gpio.h>
#include <ti/csl/csl_adc.h>
#include <ti/csl/cslr_synctimer.h>
#include <ti/csl/csl_types.h>
//#include <ti/csl/soc.h>
#include <ti/csl/hw_types.h>

#include <ti/drv/iolink/test/stack_test/src/ioLink_powerSwitchTask.h>

/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

static Mailbox_Handle mBoxPower;

struct powerMSGstruct{
    int port;
    enum portStateenum state;
};

static int enable_mappig[] = {
    /* see board_gpioLed.c for ports */
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14
};

/*
 * 15 - L_SEL
 * 16 - L_SEH
 * 17 - H_SEL
 * 18 - H_SEH
 * 19 - L_Fault
 * 20 - H_Fault
 */

static int port_state[8] =
{
     0,0,0,0,0,0,0,0
};

enum adc_enum{ADC1, ADC5};

struct current_sense_mapping{
    enum adc_enum adc;
    int pin1;
    int pin2;
    int fault_in;
};

static struct current_sense_mapping current_sense[] = {
    {ADC5, 16, -1, 19},
    {ADC5, 16, 15, 19},
    {ADC5, 15, -1, 19},
    {ADC5, -1, -1, 19},
    {ADC1, 18, -1, 20},
    {ADC1, 18, 17, 20},
    {ADC1, 17, -1, 20},
    {ADC1, -1, -1, 20},
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 * Power up or down a port
 */
void IO_Link_Power(int port, enum portStateenum state){
    struct powerMSGstruct msg;
    if(mBoxPower == 0) return;
    msg.port = port;
    msg.state = state;
    Mailbox_post(mBoxPower, &msg, BIOS_WAIT_FOREVER);
}

/**
 * Return power state of a port
 */
int IO_Link_getPower(int port){
    return port_state[port];
}

/* Macro representing the module clock of TSCADC. */
#define TSCADC_MODULE_CLOCK             (24000000U)

/* Macro representing the AFE clock of TSCADC. */
#define TSCADC_AFE_CLOCK                (3000000U)

/* Macro representing the base address of TSCADC module. */
#define TSCADC_BASE_ADDR                (0x44E0D000U)

/**
 * internal use
 * init and read two channels of the ADC
 */
static void adc_get(int *ch0, int *ch1, int channel0, int channel1){
    tscAdcStepCfg_t adcStepCfg;
    uint32_t fifoNum[2U];
    uint32_t index;
    int fifoCount;

    /* Configure the ADC AFE clock. */
    TSCADCClkDivConfig(TSCADC_BASE_ADDR, TSCADC_MODULE_CLOCK, TSCADC_AFE_CLOCK);

    /* Enable the step ID tag. */
    TSCADCStepIdTagEnable(TSCADC_BASE_ADDR, TRUE);

    /* Disable the write protection for Step config registers. */
    TSCADCStepConfigProtectionEnable(TSCADC_BASE_ADDR, FALSE);

    adcStepCfg.adcNegativeInpRef = TSCADC_NEGATIVE_REF_VSSA;
    adcStepCfg.adcPositiveInpRef = TSCADC_POSITIVE_REF_VDDA;
    adcStepCfg.adcNegativeInpChan = TSCADC_INPUT_VREFN;
    adcStepCfg.adcPositiveInpChan = channel0;
    adcStepCfg.enableXppsw = FALSE;
    adcStepCfg.enableXnpsw = FALSE;
    adcStepCfg.enableYppsw = FALSE;
    adcStepCfg.enableXnnsw = FALSE;
    adcStepCfg.enableYpnsw = FALSE;
    adcStepCfg.enableYnnsw = FALSE;
    adcStepCfg.enableWpnsw = FALSE;

    fifoNum[0U] = TSCADC_FIFO_SEL_0;
    fifoNum[1U] = TSCADC_FIFO_SEL_1;

    for (index = 0U; index < 2U; index++)
    {
        if (1U == index)
        {
            adcStepCfg.adcPositiveInpChan = channel1;
        }

        /* Configure the ADC steps. */
        TSCADCStepConfig(TSCADC_BASE_ADDR, (index + 1U),
                         FALSE, &adcStepCfg);

        /* Configure the ADC FIFO. */
        TSCADCFIFOIRQThresholdLevelConfig(TSCADC_BASE_ADDR, (index + 1U),
                             fifoNum[index], 2U, TRUE);

        /* Configure the Step mode. */
        TSCADCStepMode(TSCADC_BASE_ADDR, (index + 1U),
                       TSCADC_STEP_MODE_SW_ENABLED_ONE_SHOT);

        /* Enable the ADC steps. */
        TSCADCConfigureStepEnable(TSCADC_BASE_ADDR, (index + 1U), TRUE);
    }

    TSCADCIntrClear(TSCADC_BASE_ADDR,
                    (TSCADC_INTR_MASK_HW_PEN_EVENT_ASYNC |
                    TSCADC_INTR_MASK_END_OF_SEQUENCE |
                    TSCADC_INTR_MASK_FIFO0_THRESHOLD |
                    TSCADC_INTR_MASK_FIFO0_OVERRUN |
                    TSCADC_INTR_MASK_FIFO0_UNDERFLOW |
                    TSCADC_INTR_MASK_FIFO1_THRESHOLD |
                    TSCADC_INTR_MASK_FIFO1_OVERRUN |
                    TSCADC_INTR_MASK_FIFO1_UNDERFLOW |
                    TSCADC_INTR_MASK_OUT_OF_RANGE |
                    TSCADC_INTR_MASK_PEN_UP_EVENT |
                    TSCADC_INTR_MASK_HW_PEN_EVENT_SYNC));

    TSCADCEventInterruptEnable(TSCADC_BASE_ADDR, TSCADC_INTR_MASK_END_OF_SEQUENCE);

    TSCADCEnable(TSCADC_BASE_ADDR, TRUE);

    //wait for finished
    while(!(TSCADCIntStatus(TSCADC_BASE_ADDR) & TSCADC_INTR_MASK_END_OF_SEQUENCE));
    TSCADCIntrClear(TSCADC_BASE_ADDR, TSCADC_INTR_MASK_END_OF_SEQUENCE);

    fifoCount = TSCADCGetFifoWordCount(TSCADC_BASE_ADDR, TSCADC_FIFO_SEL_0);
    if(fifoCount){
        *ch0 = TSCADCFIFOADCDataRead(TSCADC_BASE_ADDR, TSCADC_FIFO_SEL_0);
    }

    fifoCount = TSCADCGetFifoWordCount(TSCADC_BASE_ADDR, TSCADC_FIFO_SEL_1);
    if(fifoCount){
        *ch1 = TSCADCFIFOADCDataRead(TSCADC_BASE_ADDR, TSCADC_FIFO_SEL_1);
    }

    //UART_printf("fifo 0: 0x%x fifo 1: 0x%x\n", *ch0, *ch1);

    TSCADCEnable(TSCADC_BASE_ADDR, FALSE);
}

/**
 * internal use
 * select one current sense port of the high side switch
 */
static void select_input(int port){
    int i;

    for(i=0; i<4; i++)
        GPIO_write(15+i, 0); // clear all sel outputs

    // select input
    if(current_sense[port].pin1 != -1)
        GPIO_write(current_sense[port].pin1, 1);
    if(current_sense[port].pin2 != -1)
        GPIO_write(current_sense[port].pin2, 1);

    Task_sleep(10); // some time to settle
}

/**
 * return current in mA at specified port
 */
float IO_Link_get_current(int port){
    int ch0, ch1;
    select_input(port);
    adc_get(&ch0, &ch1, TSCADC_INPUT_CHANNEL1, TSCADC_INPUT_CHANNEL5);

    switch(current_sense[port].adc){
    case ADC5: //L_CS
        return ch1*1150.0/4096.0;
    case ADC1: //H_CS
        return ch0*1150.0/4096.0;
    }
    return -1;  //1.15A FS
}

/**
 * return input current measured by INA253 in mA
 */
float IO_Link_get_input_current(void){
    int ch0, ch1;
    adc_get(&ch0, &ch1, TSCADC_INPUT_CHANNEL3, TSCADC_INPUT_CHANNEL3);
    return ((ch0+ch1)/2)*9000.0/4096.0; //9A FS
}

/**
 * return state of the fault output of high side switch
 */
int IO_Link_get_fault(int port){
    select_input(port);
    return GPIO_read(current_sense[port].fault_in);
}

void IOLink_powerSwitchTask(UArg arg0){
    struct powerMSGstruct msg;
    int i;
    int current;
    int fault;
    int current_sense_channel=0;
    GPIO_init();

    Mailbox_Params mboxParams;
    Error_Block eb;
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mBoxPower = Mailbox_create(sizeof(msg), 5, &mboxParams, &eb);
    if (mBoxPower == NULL) {
        System_printf("taskFxn(): %s\n", Error_getMsg(&eb) );
        System_abort("Mailbox create failed");
    }

    /* enable signals */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_UART0_RTSN), 0x13000007); /* configure UART0_RTSN pin to as GPIO1_9 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_SPI0_D0), 0x13000007); /* configure SPI0_D0 pin to as GPIO0_3 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_GPMC_AD14), 0x13000007); /* configure GPMC_AD14 pin to as GPIO1_14 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_GPMC_AD15), 0x13000007); /* configure GPMC_AD15 pin to as GPIO1_15 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_GPMC_AD12), 0x13000007); /* configure GPMC_AD12 pin to as GPIO1_12 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_GPMC_AD13), 0x13000007); /* configure GPMC_AD13 pin to as GPIO1_13 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_UART3_RTSN), 0x13000007); /* configure UART3_RTSN pin to as GPIO5_1 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_LCD_DATA6), 0x13000007); /* configure DSS_DATA6 pin to as GPIO2_12 */

    /* current sense select signals */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_CCDC0_DATA1), 0x13000007); /* configure CAM0_DATA1 pin to as GPIO5_20 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_CCDC1_DATA7), 0x13000007); /* configure CAM1_DAT7 pin to as GPIO4_21 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_UART1_TXD), 0x13000007); /* configure UART1_TXD pin to as GPIO0_15 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_CCDC1_DATA0), 0x13000007); /* configure CAM1_DAT0 pin to as GPIO4_14 */

    /* fault signals */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_CCDC1_DATA1), 0x13070007); /* configure CAM1_DAT1 pin to as GPIO4_15 */
    HW_WR_REG32((SOC_CONTROL_MODULE_REG + CTRL_CONF_UART1_RXD), 0x13070007); /* configure UART1_RXD pin to as GPIO0_14 */


    for(i=0; i<sizeof(enable_mappig)/sizeof(enable_mappig[0]); i++)
        GPIO_setConfig(enable_mappig[i], GPIO_CFG_OUTPUT|GPIO_CFG_OUT_LOW);

    while(1){
        if (Mailbox_pend(mBoxPower, &msg, 200) == TRUE) {
            GPIO_write(enable_mappig[msg.port], msg.state);
            port_state[msg.port] = msg.state;
        }
        current = IO_Link_get_current(current_sense_channel);
        fault = IO_Link_get_fault(current_sense_channel);
        /* detect fault condition and reset port */
        if(fault == 0 && current > 1148){
            GPIO_write(enable_mappig[current_sense_channel], 0);
            port_state[current_sense_channel] = 0;
        }
        //current sense debug output (console)
        //printf("Port %d, Current %d, Total %d\n", current_sense_channel, current, (int)IO_Link_get_input_current());
        current_sense_channel++;
        if(current_sense_channel>7) current_sense_channel = 0;
    }
}

