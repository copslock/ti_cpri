/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 *  \file   IOLINK_soc.c
 *
 *  \brief  AM437x SoC specific IOLINK hardware attributes.
 *
 *   This file contains the software attributes of IOLINK firmware IP, like
 *   base address, interrupt number etc.
 */


#include <stdint.h>
#include <ti/drv/iolink/IOLINK.h>
#include <ti/drv/iolink/soc/IOLINK_soc.h>
#include <ti/starterware/include/hw/am437x.h>
#include <ti/drv/pruss/soc/pruicss_v1.h>

/* IOLINK Soft IP configuration structure */
IOLINK_v0_SwAttrs iolinkInitCfg[IOLINK_SWIP_MAX_CNT] =
{
    {
        /* version */
        0,
        /* pruIcssConfig */
        {
            1,                         /* icssNum, 1: ICSS0, 2: ICSS1 */
            0,                         /* pruNum */
            PRU_ICSS_DATARAM(0),       /* dataMem0 */
            PRU_ICSS_DATARAM(1),       /* dataMem1 */
            PRU_ICSS_IRAM(0),          /* instMem */
            SOC_PRU_ICSS0_U_DATA_RAM0, /* dataMemBaseAddr */
            SOC_PRU_ICSS0_U_INTC_REG   /* intcBaseAddr */
        },
        /* cycleCounterIntConfig */
        {
            191, /* coreIntNum */
            7,   /* socEvId */
            1    /* intPrio */
        },
        /* adjustableTimerIntConfig */
        {
            127, /* coreIntNum */
            7,   /* socEvId */
            1    /* intPrio */
        },
        /* pruCompleteIntConfig */
        {
            192, /* coreIntNum */
            16,  /* socEvId */
            1    /* intPrio */
        }
    },
};

/* IOLINK objects */
IOLINK_v0_Object iolinkObjects[IOLINK_SWIP_MAX_CNT];

/* IOLINK configuration structure */
IOLINK_config_list IOLINK_config =
{
    {
        &IOLINK_v0_FxnTable,
        &iolinkObjects[0],
        &iolinkInitCfg[0]
    }
};

/**
 * \brief  This API gets the IOLINK FW configuration
 *
 * \param  index     IOLINK instance index.
 * \param  cfg       Pointer to IOLINK FW config.
 *
 * \return 0 success: -1: error
 *
 */
int32_t IOLINK_socGetFwCfg(uint32_t index, IOLINK_v0_SwAttrs *cfg)
{
    int32_t ret = 0;
    
    if (index < IOLINK_SWIP_MAX_CNT)
    {
        *cfg = iolinkInitCfg[index];
    }
    else
    {
        ret = (-((int32_t)1));
    }

    return ret;
}

/**
 * \brief  This API sets the IOLINK FW configuration
 *
 * \param  index     IOLINK instance index.
 * \param  cfg       Pointer to IOLINK FW config.
 *
 * \return           0 success: -1: error
 *
 */
int32_t IOLINK_socSetFwCfg(uint32_t index, const IOLINK_v0_SwAttrs *cfg)
{
    int32_t ret = 0;

    if (index < IOLINK_SWIP_MAX_CNT)
    {
        iolinkInitCfg[index] = *cfg;
    }
    else
    {
        ret = (-((int32_t)1));
    }

    return ret;
}

void IOLINK_pruIcssPinMuxCfg(void)
{
    /* input configuration RX0 ... RX7 */
    *((uint32_t *) 0x44E10990)      =  ((1<<18) | 0x6);      /* enable GPI0 for PRU0-ICSS0; register mcasp0_aclkx; IDK Pin31 */
    *((uint32_t *) 0x44E10994)      =  ((1<<18) | 0x6);      /* enable GPI1 for PRU0-ICSS0; register mcasp0_fsx;   IDK Pin33 */
    *((uint32_t *) 0x44E10998)      =  ((1<<18) | 0x6);      /* enable GPI2 for PRU0-ICSS0; register mcasp0_axr0;  IDK Pin35 */
    *((uint32_t *) 0x44E1099C)      =  ((1<<18) | 0x6);      /* enable GPI3 for PRU0-ICSS0; register mcasp0_ahclkr; IDK Pin32 */
    *((uint32_t *) 0x44E109A0)      =  ((1<<18) | 0x6);      /* enable GPI4 for PRU0-ICSS0; register mcasp0_aclkr; IDK Pin34 */
    *((uint32_t *) 0x44E109A4)      =  ((1<<18) | 0x6);      /* enable GPI5 for PRU0-ICSS0; register mcasp0_fsr;   IDK Pin36 */
    *((uint32_t *) 0x44E109A8)      =  ((1<<18) | 0x6);      /* enable GPI6 for PRU0-ICSS0; register mcasp0_axr1;  IDK Pin52 */
    *((uint32_t *) 0x44E109AC)      =  ((1<<18) | 0x6);      /* enable GPI7 for PRU0-ICSS0; register mcasp0_ahclkx; IDK Pin54 */

    /* output configuration TX0 ... TX7 */
    *((uint32_t *) 0x44E108F0)      =  ((1<<16) | 0x5);      /* enable GPO8 for PRU0-ICSS0; register mmc0_dat3; IDK Pin56 */
    *((uint32_t *) 0x44E108F4)      =  ((1<<16) | 0x5);      /* enable GPO9 for PRU0-ICSS0; register mmc0_dat2; IDK Pin57 */
    *((uint32_t *) 0x44E108F8)      =  ((1<<16) | 0x5);      /* enable GPO10 for PRU0-ICSS0; register mmc0_dat1; IDK Pin38 */
    *((uint32_t *) 0x44E108FC)      =  ((1<<16) | 0x5);      /* enable GPO11 for PRU0-ICSS0; register mmc0_dat0; IDK Pin58 */
    *((uint32_t *) 0x44E10900)      =  ((1<<16) | 0x5);      /* enable GPO12 for PRU0-ICSS0; register mmc0_clk; IDK Pin53 (bluewire) */
    *((uint32_t *) 0x44E10904)      =  ((1<<16) | 0x5);      /* enable GPO13 for PRU0-ICSS0; register mmc0_cmd; IDK Pin55 (bluewire) */
    *((uint32_t *) 0x44E10A28)      =  ((1<<16) | 0x4);      /* enable GPO18 for PRU0-ICSS0; register uart3_rxd; IDK Pin57 (bluewire) */
    *((uint32_t *) 0x44E10A2C)      =  ((1<<16) | 0x4);      /* enable GPO19 for PRU0-ICSS0; register uart3_txd; IDK Pin5 (bluewire) */

    /* output configuration GPIO's for TX_EN function */
    *((uint32_t *) (0x44DF8800 + 0x478)) = 0x2;             /* Enable the GPIO1 module */
    *((uint32_t *) (0x44DF8800 + 0x490)) = 0x2;             /* Enable the GPIO4 module */
    *((uint32_t *) (0x44DF8800 + 0x498)) = 0x2;             /* Enable the GPIO5 module */

    *((uint32_t *) 0x4804C134)      &=  ~((1<<8));    /* Enable the corresponding outputs of GPIO1 */
    *((uint32_t *) 0x48320134)      &=  ~((1<<10) | (1<<12));  /* Enable the corresponding outputs of GPIO4 */
    *((uint32_t *) 0x48322134)      &=  ~((1<<4) | (1<<6) | (1<<26) | (1<<23) | (1<<25));  /* Enable the corresponding outputs of GPIO5 */

    *((uint32_t *) 0x4804C190)      =  ((1<<8));    /* Clear the corresponding outputs of GPIO1 */
    *((uint32_t *) 0x48320190)      =  ((1<<10) | (1<<12));  /* Clear the corresponding outputs of GPIO4 */
    *((uint32_t *) 0x48322190)      =  ((1<<4) | (1<<6) | (1<<26) | (1<<23) | (1<<25));  /* Clear the corresponding outputs of GPIO5 */

    *((uint32_t *) 0x44E109D8)      =  ((1<<16) | 0x7);     /* MUX GPIO4_10 to AC23, register cam1_vd; IDK Pin4 */
    *((uint32_t *) 0x44E10A50)      =  ((1<<16) | 0x7);     /* MUX GPIO5_4 to P25, register spi4_sclk; IDK Pin6 */
    *((uint32_t *) 0x44E10A58)      =  ((1<<16) | 0x7);     /* MUX GPIO5_6 to P24, register spi4_d1; IDK Pin8 */
    *((uint32_t *) 0x44E1082C)      =  ((1<<16) | 0x9);     /* MUX GPIO5_23 to D11, register spi4_d1; IDK Pin10 */
    *((uint32_t *) 0x44E109E0)      =  ((1<<16) | 0x7);     /* MUX GPIO4_12 to AC25, register cam1_field; IDK Pin12 */
    *((uint32_t *) 0x44E10820)      =  ((1<<16) | 0x9);     /* MUX GPIO5_26 to B10, register gpmc_ad8; IDK Pin14 */
    *((uint32_t *) 0x44E10824)      =  ((1<<16) | 0x9);     /* MUX GPIO5_25 to A10, register gpmc_ad9; IDK Pin16 */
    *((uint32_t *) 0x44E10968)      =  ((1<<16) | 0x7);     /* MUX GPIO1_8 to L25, register uart0_ctsn; IDK Pin18 */
}

/* TBD: need to check if can use timer apis in osal */
void IOLINK_cTimerInit(void)
{
    /* Set the counters default increment value to 5 */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_GLOBAL_CFG)) &= ~(CSL_ICSSM_IEP_GLOBAL_CFG_DEFAULT_INC_MASK);
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_GLOBAL_CFG)) |= (0x5<<CSL_ICSSM_IEP_GLOBAL_CFG_DEFAULT_INC_SHIFT);
    /* Clear the counter overflow status bits */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_GLOBAL_STATUS)) = 0x1;
    /* Enable compare register 0 and the counter reset on compare event */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_CMP_CFG)) = 0x3;
    /* Clear all compare match status bits */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_CMP_STATUS)) = 0xFF;
    /* Set the compare value to 100000 ns (100 us) */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_CMP0)) = 100000;
    /* Enable the IEP Timer of ICSS0 PRU0 */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_GLOBAL_CFG)) |= 0x1;

    /* ICSS0 INTC configuration */
    /* Set the interrupt polarity of system event 7 to active high */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_SIPR0)) |= (1<<7);
    /* Set the type of system event 7 to level or pulse interrupt */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_SITR0)) &= ~(1<<7);
    /* map system event with index 7 to channel 2 */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_CMR1)) &= ~(CSL_ICSSM_INTC_CMR1_CH_MAP_7_MASK);
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_CMR1)) |= (0x2<<CSL_ICSSM_INTC_CMR1_CH_MAP_7_SHIFT);
    /* map channel 2 to interrupt with index 2 */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_HMR0)) &= ~(CSL_ICSSM_INTC_HMR0_HINT_MAP_2_MASK);
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_HMR0)) |= (0x2<<CSL_ICSSM_INTC_HMR0_HINT_MAP_2_SHIFT);
    /* clear the system event with index 7  (pr0_iep_tim_cap_cmp_pend) */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_SICR)) = 0x7;
    /* enable the system event with index 7  (pr0_iep_tim_cap_cmp_pend) */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_EISR)) = 0x7;
    /* enable host interrupt output with index 2 */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_HIEISR)) = 0x2;
    /* globally enable all interrupts */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_GER)) = 0x1;
}

#define ADJUSTABLE_TIMER_ADR    SOC_DMTIMER7_REG
#define TIMER_CLKSEL_REG  0x18
#define TIMER_CLKCTRL_REG  0x558
void IOLINK_adjustableTimerInit(void)
{
    /* enable the timer 7 module */
    *((uint32_t*) (SOC_CM_PER_REG + TIMER_CLKCTRL_REG)) = 0x2;
    /* timer clock = M_OSC (24 MHz) */
    *((uint32_t*) (SOC_CM_DPLL_REG + TIMER_CLKSEL_REG))    = 0x1;
    /* clear pending compare match events */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_IRQSTATUS)) |= (1<<TIMER_IRQSTATUS_MAT_IT_FLAG_SHIFT);
    /* enable irq for compare match events */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_IRQENABLE_SET)) |= (1<<TIMER_IRQENABLE_SET_MAT_EN_FLAG_SHIFT);
    /* enable the compare mode */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TCLR)) |= (1<<TIMER_TCLR_CE_SHIFT);
}

void IOLINK_adjustableTimerStart(uint32_t compare)
{
    /* stop the timer */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TCLR)) &= ~((1<<TIMER_TCLR_ST_SHIFT));
    /* clear pending compare match events */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_IRQSTATUS)) |= (1<<TIMER_IRQSTATUS_MAT_IT_FLAG_SHIFT);
    /* reset timer to 0 */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TCRR)) = 0;
    /* set the compare value for this timer */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TMAR)) = compare;
    /* set timer start bit */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TCLR)) |= (1<<TIMER_TCLR_ST_SHIFT);
}

void IOLINK_adjustableTimerStop(void)
{
    /* clear timer start bit */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_TCLR)) &= ~(1<<TIMER_TCLR_ST_SHIFT);
}

void IOLINK_clearCycleTimerInt(void)
{
    /* Clear all compare match status bits */
    *((uint32_t*) (SOC_PRU_ICSS0_U_IEP_REG + CSL_ICSSM_IEP_CMP_STATUS)) = 0xFF;
    /* clear the system event with index 7  (pr0_iep_tim_cap_cmp_pend) */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_SICR)) = 0x7;
}

void IOLINK_clearAdjTimerInt(void)
{
    /* clear pending compare match events */
    *((uint32_t*) (ADJUSTABLE_TIMER_ADR + TIMER_IRQSTATUS)) |= (1<<TIMER_IRQSTATUS_MAT_IT_FLAG_SHIFT);
}

void IOLINK_clearPruCompInt(void)
{
    /* clear the system event with index 16  (pr0_iep_tim_cap_cmp_pend) */
    *((uint32_t*) (SOC_PRU_ICSS0_U_INTC_REG + CSL_ICSSM_INTC_SICR)) = 16;
}


