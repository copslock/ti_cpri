/*
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
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
 *
*/

/*
 *  This is an example to demonstrate multiple interrupts through
 *  hyperlink interace.
 *  It can work between two devices or in loopback on one device.
 *  ( Loopback configuration can be enabled in hyplnkPlatCfg.h )
 *
 *  It uses cic0 & cic1 for interrupt DSP cores and Cic2 for ARM cores
 *
 *
 *  For other configurations, change the definitions starting
 *  at "Beginning of user configuration" as required.
 *
 */

#include "hyplnkResource.h"
#include <ti/drv/hyplnk/hyplnk.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/csl_device_interrupt.h>
#include <ti/csl/csl_bootcfgAux.h>

#include "hyplnkLLDIFace.h"
#include "hyplnkPlatCfg.h"

#define EMUCNT_PROFILE
#ifdef EMUCNT_PROFILE
#if !defined(CSL_PLLC_REGS)
#define CSL_PLLC_REGS CSL_PLL_CONTROLLER_REGS
#endif
#include <ti/csl/csl_pllc.h>
CSL_PllcRegsOvly pllcRegs = (CSL_PllcRegsOvly)(CSL_PLLC_REGS);
#endif
/*=============================================================================
 *=============================================================================
 * Beginning of user configuration
 *=============================================================================
 ============================================================================*/

/*****************************************************************************
 * There are configuration parameters such as link speed and loopback
 * in ti/drv/hyplnk/example/common/hyplnkLLDCfg.h
 ****************************************************************************/

/*****************************************************************************
 * Select one or more test cases
 *
 *****************************************************************************/
/* Test to send interrupt to remote ARM device through CIC2 */
#define hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT

/* Test to send interrupt to remote DSP device through CIC0, CIC1 */
#define hyplnk_EXAMPLE_GENERATE_REMOTE_CIC0_CIC1_INTERRUPT

/* Testing only DSP to ARM interrupts */
/* #define hyplnk_TEST_INTERRUPT_DSP_TO_ARM_ONLY */

/* Configure number of interrupts to be generated */
#define hyplnk_EXAMPLE_NUM_INTERRUPTS 100

/* Configuration for DSP to ARM interrupt */
#define hyplnk_EXAMPLE_CIC2_DELAY_CYCLES_BETWEEN_INTERRUPTS 40000
/* #define hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES 8 */
/* #define hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS 3 */

/* Configuration for to-DSP interrupt */
#if defined(DEVICE_K2E) || defined(SOC_K2E)
#define hyplnk_EXAMPLE_NUM_DSP_CORES 1
#else
#define hyplnk_EXAMPLE_NUM_DSP_CORES 8
#endif

#define hyplnk_EXAMPLE_CIC0_1_DELAY_CYCLES_BETWEEN_INTERRUPTS 2000

#ifdef EMUCNT_PROFILE
volatile uint32_t recordEmucnt0End[hyplnk_EXAMPLE_NUM_DSP_CORES][hyplnk_EXAMPLE_NUM_INTERRUPTS];
volatile uint32_t recordEmucnt0Start[hyplnk_EXAMPLE_NUM_DSP_CORES][hyplnk_EXAMPLE_NUM_INTERRUPTS];
#endif

/* ARM side needs 24 independent interrupts, for example:
 *   - Using mostly reserved interrupts for this purpose
 *   - First 8 can be used for the first remote DSP with one interrupt
 *     per core
 *   - Second 8 can be used for the second remote DSP with one interrupt
 *     per core
 *   - Last 8 can be used for local DSP with one interrupt per core
 *   - If using a different remote device, the following entries need to be
 *     replaced with event numbers corresponding to the remote device.
 */
unsigned int rsv_cic2_sys_interrupt[24] = {
	9, /* CSL_CIC2_RESERVED_9 */
	10, /* CSL_CIC2_RESERVED_10 */
	23, /* CSL_CIC2_DFT_PBIST_CPU_INT */
	70, /* CSL_CIC2_MPU_3_INT */
	75, /* CSL_CIC2_RESERVED_75 */
	92, /* CSL_CIC2_RESERVED_92 */
	189, /* CSL_CIC2_RESERVED_189 */
	190, /* CSL_CIC2_RESERVED_190 */

	191, /*CSL_CIC2_RESERVED_191 */
	192, /*CSL_CIC2_RESERVED_192 */
	193, /* CSL_CIC2_RESERVED_193 */
	194, /* CSL_CIC2_RESERVED_194 */
	195, /*CSL_CIC2_RESERVED_195 */
	196, /* CSL_CIC2_RESERVED_196 */
	197, /* CSL_CIC2_RESERVED_197 */
	198, /* CSL_CIC2_RESERVED_198 */

	203, /* CSL_CIC2_MPU_4_INT */
	205, /* CSL_CIC2_MPU_7_INT */
	349, /* CSL_CIC2_AIF_ATEVT15 */
	350, /* CSL_CIC2_RESERVED_350 */
	351, /* CSL_CIC2_RESERVED_351, */
	352, /* CSL_CIC2_RESERVED_352, */
	353, /* CSL_CIC2_RESERVED_353 */
	354 /* CSL_CIC2_RESERVED_386 */
};

/* cic outputs corresponding to event numbers for different DSP cores */
/* 68 is the output for core 8                                        */
#define CIC_OUTPUT_START_NUMBER 68
unsigned int cic0_cic1_output[8] = {
	CIC_OUTPUT_START_NUMBER,
	CIC_OUTPUT_START_NUMBER+1,
	CIC_OUTPUT_START_NUMBER+10*1,
	CIC_OUTPUT_START_NUMBER+10*1+1,
	CIC_OUTPUT_START_NUMBER+10*2,
	CIC_OUTPUT_START_NUMBER+10*2+1,
	CIC_OUTPUT_START_NUMBER+10*2,
	CIC_OUTPUT_START_NUMBER+10*2+1
};

/* Below settings for interrupt to DSP cores
 * CIC0/CIC1 are written to generate interrupt to DSP cores
 * - If using a different remote device, the following entries can be replaced
 *     with numbers corresponding to the remote device.
 */
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
/* CIC0 is for corepac 0-3 */
unsigned int rsv_cic0_sys_interrupt[8] = {
	CSL_CIC0_RESERVED_7,
	CSL_CIC0_RESERVED_23,
	CSL_CIC0_RESERVED_37,
	CSL_CIC0_RESERVED_46,
	CSL_CIC0_RESERVED_62,
	CSL_CIC0_RESERVED_101,
	CSL_CIC0_RESERVED_283,
	CSL_CIC0_RESERVED_284
};
/* CIC1 is for corepac 4-7 */
unsigned int rsv_cic1_sys_interrupt[8] = {
	CSL_CIC1_RESERVED_7,
	CSL_CIC1_RESERVED_23,
	CSL_CIC1_RESERVED_37,
	CSL_CIC1_RESERVED_46,
	CSL_CIC1_RESERVED_62,
	CSL_CIC1_RESERVED_101,
	CSL_CIC1_RESERVED_283,
	CSL_CIC1_RESERVED_284
};
#endif
#if defined(DEVICE_K2E) || defined(SOC_K2E)
unsigned int rsv_cic0_sys_interrupt[8] = {
	CSL_CIC0_RESERVED_7,
	CSL_CIC0_RESERVED_23,
	CSL_CIC0_RESERVED_37,
	CSL_CIC0_RESERVED_46,
	CSL_CIC0_RESERVED_62,
	CSL_CIC0_RESERVED_74,
	CSL_CIC0_RESERVED_75,
	CSL_CIC0_RESERVED_76
};
#endif
/*=============================================================================
 *=============================================================================
 * End of user configuration
 *=============================================================================
 ============================================================================*/

/* Adjust timings based on loopback */
#ifdef hyplnk_EXAMPLE_LOOPBACK
	#define hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT   hyplnk_EXAMPLE_uS_TO_CYCLES(100)
	#define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT    hyplnk_EXAMPLE_uS_TO_CYCLES(100)
#else
	#define hyplnk_EXAMPLE_FIRST_TOKEN_WAIT_LIMIT   hyplnk_EXAMPLE_uS_TO_CYCLES(30000000)
	#ifdef hyplnk_EXAMPLE_TEST_CPU_BLOCK_XFER
		#define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT  hyplnk_EXAMPLE_uS_TO_CYCLES(1000)
	#else
		#define hyplnk_EXAMPLE_BULK_TOKEN_WAIT_LIMIT  hyplnk_EXAMPLE_uS_TO_CYCLES(10)
	#endif
#endif

/* Convert speed to string for printing */
#if defined(hyplnk_EXAMPLE_SERRATE_01p250)
	#define hyplnk_EXAMPLE_LINK_SPEED "1.25G"
#elif defined(hyplnk_EXAMPLE_SERRATE_03p125)
	#define hyplnk_EXAMPLE_LINK_SPEED "3.125"
#elif defined(hyplnk_EXAMPLE_SERRATE_06p250)
	#define hyplnk_EXAMPLE_LINK_SPEED "6.25"
#elif defined(hyplnk_EXAMPLE_SERRATE_07p500)
	#define hyplnk_EXAMPLE_LINK_SPEED "7.5"
#elif defined(hyplnk_EXAMPLE_SERRATE_10p000)
	#define hyplnk_EXAMPLE_LINK_SPEED "10.0"
#elif defined(hyplnk_EXAMPLE_SERRATE_12p500)
	#define hyplnk_EXAMPLE_LINK_SPEED "12.5"
#else
	#define hyplnk_EXAMPLE_LINK_SPEED "Unknown - add #define"
#endif

#ifdef hyplnk_EXAMPLE_ALLOW_4_LANES
	#define hyplnk_EXAMPLE_MAX_LANES 4
#else
	#define hyplnk_EXAMPLE_MAX_LANES 1
#endif

#ifndef __ARMv7
int isr5counter = 0;
int isr6counter = 0;
CSL_IntcHandle   hIntc0, hIntc1;
CSL_CPINTC_Handle hnd0, hnd1;
CSL_IntcEventHandlerRecord  EventHandler[8];
CSL_IntcGlobalEnableState state;
CSL_IntcObj    intcObj0, intcObj1;

/* Event numbers to be used for DSP cores: Uses event number 26, 27*/
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
unsigned int corepac_event_number[2] = {
	CSL_C66X_COREPAC_CIC_OUT68_PLUS_10_MUL_N,
	CSL_C66X_COREPAC_CIC_OUT69_PLUS_10_MUL_N
		};
#endif
#if defined(DEVICE_K2E) || defined(SOC_K2E)
unsigned int corepac_event_number[1] = {
	CSL_C66X_COREPAC_CIC_0_OUT68
	};
#endif

interrupt void int5_isr()
{
	/* clear system interrupt */
	if (DNUM < 4)
		CSL_CPINTC_clearSysInterrupt(hnd0, rsv_cic0_sys_interrupt[DNUM*2]);
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
	else
		CSL_CPINTC_clearSysInterrupt(hnd1, rsv_cic1_sys_interrupt[(DNUM-4)*2]);
#endif

	/* Acknowledge CorePac interrupt */
	CSL_intcHwControl(hIntc0, CSL_INTC_CMD_EVTCLEAR, NULL);

	isr5counter++;
	/* printf("Core: %d get from isr4\n", DNUM);*/
}

interrupt void int6_isr()
{
	/* clear system interrupt */
	if (DNUM < 4)
		CSL_CPINTC_clearSysInterrupt(hnd0, rsv_cic0_sys_interrupt[DNUM*2+1]);
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
	else
		CSL_CPINTC_clearSysInterrupt(hnd1, rsv_cic1_sys_interrupt[(DNUM-4)*2+1]);
#endif

	/* Acknowledge CorePac interrupt */
	CSL_intcHwControl(hIntc1, CSL_INTC_CMD_EVTCLEAR, NULL);

	isr6counter++;
	/* printf("Core: %d get from isr5\n", DNUM); */
}

void cpintc_config()
{

	hnd0 = CSL_CPINTC_open(0);
	if (hnd0 == 0) {
		printf("Error: Unable to open CPINTC 0\n");
		return;
	}
	hnd1 = CSL_CPINTC_open(1);
	if (hnd1 == 0) {
		printf("Error: Unable to open CPINTC 1\n");
		return;
	}

	/* Disable all host interrupts. */
	CSL_CPINTC_disableAllHostInterrupt(hnd0);
	CSL_CPINTC_disableAllHostInterrupt(hnd1);

	/* Configure no nesting support in the CPINTC Module. */
	CSL_CPINTC_setNestingMode(hnd0, CPINTC_NO_NESTING);
	CSL_CPINTC_setNestingMode(hnd1, CPINTC_NO_NESTING);

	/* We now map System Interrupt to channel
	 * enable system interrupt
	 * enable host interrupt. */
	if (DNUM < 4) {
		CSL_CPINTC_mapSystemIntrToChannel(hnd0,
				rsv_cic0_sys_interrupt[DNUM*2+0], cic0_cic1_output[DNUM*2+0]);
		CSL_CPINTC_mapSystemIntrToChannel(hnd0,
				rsv_cic0_sys_interrupt[DNUM*2+1], cic0_cic1_output[DNUM*2+1]);
		CSL_CPINTC_enableSysInterrupt(hnd0, rsv_cic0_sys_interrupt[DNUM*2+0]);
		CSL_CPINTC_enableSysInterrupt(hnd0, rsv_cic0_sys_interrupt[DNUM*2+1]);
		CSL_CPINTC_enableHostInterrupt(hnd0, cic0_cic1_output[DNUM*2+0]);
		CSL_CPINTC_enableHostInterrupt(hnd0, cic0_cic1_output[DNUM*2+1]);
	} 
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
    else {
		CSL_CPINTC_mapSystemIntrToChannel(hnd1,
			rsv_cic1_sys_interrupt[(DNUM-4)*2+0], cic0_cic1_output[(DNUM-4)*2+0]);
		CSL_CPINTC_mapSystemIntrToChannel(hnd1,
			rsv_cic1_sys_interrupt[(DNUM-4)*2+1], cic0_cic1_output[(DNUM-4)*2+1]);
		CSL_CPINTC_enableSysInterrupt(hnd1, rsv_cic1_sys_interrupt[(DNUM-4)*2+0]);
		CSL_CPINTC_enableSysInterrupt(hnd1, rsv_cic1_sys_interrupt[(DNUM-4)*2+1]);
		CSL_CPINTC_enableHostInterrupt(hnd1, cic0_cic1_output[(DNUM-4)*2+0]);
		CSL_CPINTC_enableHostInterrupt(hnd1, cic0_cic1_output[(DNUM-4)*2+1]);
	}
#endif

	/* Enable all host interrupts also. */
	CSL_CPINTC_enableAllHostInterrupt(hnd0);
	CSL_CPINTC_enableAllHostInterrupt(hnd1);
}

void Intc_config(void)
{
	CSL_IntcParam    vectId0;
	CSL_IntcContext  context;
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
	CSL_IntcParam vectId1;
#endif
	/* Setup the global Interrupt */
	context.numEvtEntries = 8;
	context.eventhandlerRecord = EventHandler;
	CSL_intcInit(&context);
	/* Enable NMIs  */
	CSL_intcGlobalNmiEnable();
	/* Enable Global Interrupts  */
	CSL_intcGlobalEnable(&state);

	/* VectorID for the Global Event  */
	vectId0 = CSL_INTC_VECTID_5;

	hIntc0 = CSL_intcOpen(&intcObj0, corepac_event_number[0], &vectId0,
		NULL);

	/* Hook the ISRs */
	CSL_intcHookIsr(vectId0,  &int5_isr);

	/* Clear the Interrupt */
	CSL_intcHwControl(hIntc0, CSL_INTC_CMD_EVTCLEAR,  NULL);

	/* Enable the Event & the interrupt */
	CSL_intcHwControl(hIntc0, CSL_INTC_CMD_EVTENABLE,  NULL);

#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
	/* VectorID for the Global Event  */
	vectId1 = CSL_INTC_VECTID_6;
	hIntc1 = CSL_intcOpen(&intcObj1, corepac_event_number[1], &vectId1,
		NULL);
	/* Hook the ISRs */
	CSL_intcHookIsr(vectId1,  &int6_isr);

	/* Clear the Interrupt */
	CSL_intcHwControl(hIntc1, CSL_INTC_CMD_EVTCLEAR,  NULL);

	/* Enable the Event & the interrupt */
	CSL_intcHwControl(hIntc1, CSL_INTC_CMD_EVTENABLE,  NULL);
#endif
}
#endif

#ifdef hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT

/* This number is the cic2 output numbers correspond to ARM interrupt numbers
 * 451-458, 459-466, 467-474 */
unsigned int cic2_output[24] = {
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	18, 19, 22, 23, 50, 51, 66, 67
};

void cic2InterruptSetup(CSL_CPINTC_RegsOvly remote, uint32_t dspNum,
	int coreNum)
{
	uint32_t cic2_output_value;
	uint32_t intNum;

	intNum = rsv_cic2_sys_interrupt[coreNum + dspNum*8];
	cic2_output_value = cic2_output[coreNum + dspNum*8];

	/* Enable system interrupt */
	remote->ENABLE_SET_INDEX_REG = intNum;

	/* Map system interrupt to channel, note channel to host is fixed */
	remote->CH_MAP[intNum] = cic2_output_value;

}

void cic2InterruptGenerate(CSL_CPINTC_RegsOvly remote, uint32_t dspNum,
	int coreNum)
{
	uint32_t cic2_output_value;
	uint32_t intNum;

	intNum = rsv_cic2_sys_interrupt[coreNum + dspNum*8];
	cic2_output_value = cic2_output[coreNum + dspNum*8];

	/* Enable host interrupt */
	remote->HINT_ENABLE_SET_INDEX_REG = cic2_output_value;

	/* Generate system interrupt */
	remote->STATUS_SET_INDEX_REG = intNum;

}
/*****************************************************************************
 * Setup remote interrupt by writing to Remote registers through hyperlink
 *
 ****************************************************************************/
void hyplnkExampleSetupRemoteCic2Interrupt(uint32_t *remote)
{
	int dspNum, coreNum;
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS
	for (dspNum = 0; dspNum < hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS; dspNum++) {
#else
		dspNum = 0;
#endif
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES
		for (coreNum = 0; coreNum < hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES;
				coreNum++) {
#else
#ifndef __ARMv7
			coreNum = DNUM;
#else
			coreNum = 0;
#endif
#endif
			/* Generate cic2 interrupt */
			cic2InterruptSetup((CSL_CPINTC_RegsOvly)remote, dspNum, coreNum);
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES
		}
#endif
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS
	}
#endif
}
/*****************************************************************************
 * Generate remote interrupt by writing to Remote registers through hyperlink
 *
 ****************************************************************************/
void hyplnkExampleGenerateRemoteCic2Interrupt(uint32_t *remote,
	int numInterrupts)
{
	int i, dspNum, coreNum;
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS
	for (dspNum = 0; dspNum < hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS; dspNum++) {
#else
		dspNum = 0;
#endif
		for (i = 0; i < numInterrupts; i++) {
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES
			for (coreNum = 0; coreNum < hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES;
				coreNum++) {
#else
#ifndef __ARMv7
				coreNum = DNUM;
#else
				coreNum = 0;
#endif
#endif
#ifdef EMUCNT_PROFILE
				recordEmucnt0Start[dspNum][i] = pllcRegs->EMUCNT0;
#endif
				/* Generate cic2 interrupt */
				cic2InterruptGenerate((CSL_CPINTC_RegsOvly)remote, dspNum, coreNum);
#ifdef EMUCNT_PROFILE
				recordEmucnt0End[dspNum][i] = pllcRegs->EMUCNT0;
#endif
				hyplnkExampleDelay(hyplnk_EXAMPLE_CIC2_DELAY_CYCLES_BETWEEN_INTERRUPTS);
			}
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_DSP_CORES
		}
#endif
#ifdef hyplnk_EXAMPLE_NUM_SIMULATED_REMOTE_DSPS
	}
#endif
}
#endif /* hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT */

#ifdef hyplnk_EXAMPLE_GENERATE_REMOTE_CIC0_CIC1_INTERRUPT

void cic0InterruptGenerate(CSL_CPINTC_RegsOvly cicRegs, int coreNum, int interruptId)
{
	cicRegs->STATUS_SET_INDEX_REG = rsv_cic0_sys_interrupt[coreNum*2+interruptId];
	return;
}
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
void cic1InterruptGenerate(CSL_CPINTC_RegsOvly cicRegs, int coreNum, int interruptId)
{
	cicRegs->STATUS_SET_INDEX_REG = rsv_cic1_sys_interrupt[coreNum*2+interruptId];
	return;
}
#endif
/*****************************************************************************
 * Generate remote interrupt by writing to Remote registers through hyperlink
 *
 ****************************************************************************/
void hyplnkExampleGenerateRemoteCic0Cic1Interrupt(uint32_t *remote_cic0, uint32_t *remote_cic1,
		int numCores, int numInterrupts)
{
	int i, coreNum;

	for (i = 0; i < numInterrupts; i++) {
		for (coreNum = 0; coreNum < numCores; coreNum++) {
			if (coreNum < 4) {
				cic0InterruptGenerate((CSL_CPINTC_RegsOvly)remote_cic0, coreNum, 0);
				hyplnkExampleDelay(hyplnk_EXAMPLE_CIC0_1_DELAY_CYCLES_BETWEEN_INTERRUPTS);
				cic0InterruptGenerate((CSL_CPINTC_RegsOvly)remote_cic0, coreNum, 1);
			} else {
#if defined(DEVICE_K2H) || defined(DEVICE_K2K) || defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_C6678)
				cic1InterruptGenerate((CSL_CPINTC_RegsOvly)remote_cic1, coreNum, 0);
				hyplnkExampleDelay(hyplnk_EXAMPLE_CIC0_1_DELAY_CYCLES_BETWEEN_INTERRUPTS);
				cic1InterruptGenerate((CSL_CPINTC_RegsOvly)remote_cic1, coreNum, 1);
#else
				printf("Number of cores exceeds interrupt configuration\n");
#endif
			}
		}
		/* Cycle delay */
		hyplnkExampleDelay(hyplnk_EXAMPLE_CIC0_1_DELAY_CYCLES_BETWEEN_INTERRUPTS);
	}
	return;
}
#endif /* hyplnk_EXAMPLE_GENERATE_REMOTE_CIC0_CIC1_INTERRUPT */

/*
 * These pointer are the local address within the Hyperlink's address
 * space which will access cic register spaces via HyperLink.
 *
 */
void *cic0ThroughHypLnk, *cic1ThroughHypLnk, *cic2ThroughHypLnk;

int main(void)
{
	hyplnkRet_e retVal;

#ifndef __ARMv7
	TSCL = 1;
#endif
#if defined (SOC_C6678)
    CSL_BootCfgUnlockKicker();
#endif
	if (hyplnk_memAllocInit() != 0)	{
		printf("Memory Alloc Init failed!");
		exit(1);
	}

	printf("Version #: 0x%08x; string %s\n", Hyplnk_getVersion(),
		Hyplnk_getVersionStr());

	/* Pass device config to LLD */
	retVal = Hyplnk_init(&hyplnkInitCfg);
	if (retVal != hyplnk_RET_OK) {
		printf("LLD device configuration failed\n");
		exit(1);
	}
#if defined(__ARMv7) || !defined(hyplnk_EXAMPLE_LOOPBACK) \
	|| defined(hyplnk_TEST_INTERRUPT_DSP_TO_ARM_ONLY)
	{
		/* Set up the system PLL, PSC, and DDR as required for this HW */
		printf("About to do system setup (PLL, PSC, and DDR)\n");
		retVal = hyplnkExampleSysSetup();
		if (retVal != hyplnk_RET_OK) {
			printf("system setup failed (%d)\n", (int)retVal);
			exit(1);
		}
		printf("system setup worked\n");

		/* Enable the peripheral */
		printf("About to set up HyperLink Peripheral\n");
		retVal = hyplnkExamplePeriphSetup();
		if (retVal != hyplnk_RET_OK) {
			printf("Peripheral setup failed (%d)\n", (int)retVal);
			exit(1);
		}
		printf("Peripheral setup worked\n");
	}
#endif
#ifndef __ARMv7
	/* Setup interrupt service routines */
	cpintc_config();
	Intc_config();
#endif

#if defined(hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT) \
	|| defined(hyplnk_EXAMPLE_GENERATE_REMOTE_CIC0_CIC1_INTERRUPT)
	/* Map Cic registers through Hyperlink */
	retVal = hyplnkExampleAddrMap((void *)CSL_CIC_0_REGS,
		(void **)&cic0ThroughHypLnk,0);
	if (retVal != hyplnk_RET_OK) {
		printf("Address map setup failed for cic0 registers (%d)\n",
			(int)retVal);
		exit(1);
	}
#endif
#ifdef hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT
	if (CSL_CIC_2_REGS  < (CSL_CIC_0_REGS+0x400000)) {
		cic2ThroughHypLnk = (uint8_t *)cic0ThroughHypLnk + CSL_CIC_2_REGS
			- CSL_CIC_0_REGS;
	} else {
		printf("Error mapping cic2 register\n ");
	}
	printf("Address map for Cic2 complete\n");
	pllcRegs->EMUCNT0 = 1;
	hyplnkExampleSetupRemoteCic2Interrupt(cic2ThroughHypLnk);
#endif
#ifdef __ARMv7
	printf("\n Ready to send interrupts: Press any key to continue...");
	getchar();
#else
#ifndef hyplnk_TEST_INTERRUPT_DSP_TO_ARM_ONLY
	printf("\n Waiting for interrupt events"); fflush(stdout);
	while ((isr5counter != hyplnk_EXAMPLE_NUM_INTERRUPTS) &&
		(isr6counter != hyplnk_EXAMPLE_NUM_INTERRUPTS));
	printf("\n Interrupt events complete"); fflush(stdout);
#endif
#endif

#ifdef hyplnk_EXAMPLE_GENERATE_REMOTE_CIC0_CIC1_INTERRUPT
#ifdef CSL_CIC_1_REGS
	/* Get mapped address for cic1 registers */
	if (CSL_CIC_1_REGS  < (CSL_CIC_0_REGS+0x400000)) {
		cic1ThroughHypLnk = (uint8_t *)cic0ThroughHypLnk + CSL_CIC_1_REGS
			- CSL_CIC_0_REGS;
	} else {
		printf("Error mapping cic1 register\n ");
	}
#else
	cic1ThroughHypLnk = NULL;
#endif
	printf("Address map for cic0 & cic1 complete\n");
	/* Generate interrupts to DSP using Cic0 & Cic1 */
	hyplnkExampleGenerateRemoteCic0Cic1Interrupt(cic0ThroughHypLnk,
			cic1ThroughHypLnk, hyplnk_EXAMPLE_NUM_DSP_CORES,
			hyplnk_EXAMPLE_NUM_INTERRUPTS);
	printf("Remote Cic0Cic1 interrupt generation complete\n");
#endif
#ifdef hyplnk_EXAMPLE_GENERATE_REMOTE_CIC2_INTERRUPT
	hyplnkExampleGenerateRemoteCic2Interrupt(cic2ThroughHypLnk,
		hyplnk_EXAMPLE_NUM_INTERRUPTS);
	printf("Remote interrupt generation complete\n");
#endif
#if defined(__ARMv7) || !defined(hyplnk_EXAMPLE_LOOPBACK) \
	|| defined(hyplnk_TEST_INTERRUPT_DSP_TO_ARM_ONLY)
#ifdef __ARMv7
	printf("\n Waiting for test to complete: Press any key to continue...");
	getchar();
#else
	/* Wait a bit to allow interrupts to be received */
	hyplnkExampleDelay(50000);
#endif
		hyplnkReset(hyplnk_EXAMPLE_PORT);
#endif
#ifndef __ARMv7
	printf("\n isr5counter: %d , isr6counter %d\n", isr5counter, isr6counter);
	printf("Hyperlink LLD Cic Interrupt Example Completed Successfully!\n");
	if( (isr5counter >= hyplnk_EXAMPLE_NUM_INTERRUPTS)
		&&  (isr6counter >= hyplnk_EXAMPLE_NUM_INTERRUPTS))
		printf("\nTest passed\n");
	else
		printf("\nTest failed\n");
#endif

	return 0;
}
