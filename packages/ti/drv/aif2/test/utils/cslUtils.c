/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#ifdef __ARMv7
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <c6x.h>
#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#if EVM_TYPE == 0
#include <EVM.h>
#endif

#include <ti/csl/csl.h>
#if defined(DEVICE_K2K)
#include <ti/csl/soc/k2k/src/cslr_device.h>
#include <ti/csl/soc/k2k/src/csl_device_interrupt.h>
#include <ti/csl/soc/k2k/src/csl_qm_queue.h>
#endif
#if defined(DEVICE_K2H)
#include <ti/csl/soc/k2h/src/cslr_device.h>
#include <ti/csl/soc/k2h/src/csl_device_interrupt.h>
#include <ti/csl/soc/k2h/src/csl_qm_queue.h>
#endif
#include <ti/csl/cslr_tpcc.h>
#include <ti/csl/csl_pllc.h>
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H)
#include <ti/csl/csl_pllcAux.h>
#endif
#include <ti/csl/csl_cpIntc.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntcAux.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#include <ti/csl/csl_idma.h>
#include <ti/csl/csl_idmaAux.h>
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>
#include <ti/csl/soc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>
#include <ti/csl/csl_pllc.h>
#include <ti/csl/csl_edma3.h>
#include <ti/csl/csl_bootcfgAux.h>

#ifdef _TMS320C6X
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#endif

#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/drv/aif2/aif2.h>
#include <ti/drv/aif2/aif2_osal.h>
#include <ti/drv/aif2/device/aif2_device.h>
#include <ti/drv/aif2/device/k2hk/src/aif2_device.c>

#define __CSLUTILS_C
#include "cslUtils.h"

#if EVM_TYPE == 5 || EVM_TYPE == 7
#include "appletonScbpSync.h"
#include "RTWP_evm.c"
#endif

#ifdef _TMS320C6X

#pragma CODE_SECTION(UTILS_GPIO8_setup, ".text:tools");
#pragma CODE_SECTION(UTILS_doCleanup, ".text:tools");
#pragma CODE_SECTION(UTILS_resetTimer, ".text:tools");
#pragma CODE_SECTION(UTILS_aifIntcSetup, ".text:tools");
#pragma CODE_SECTION(Aif2_RadT_Sevt7_FSEVT1_ISR, ".text:tools");
#pragma CODE_SECTION(Aif2_RadT_Sevt5_ISR, ".text:tools");
#pragma CODE_SECTION(Aif2_RadT_Sevt6_ISR, ".text:tools");
#pragma CODE_SECTION(Aif2_RadT_Sevt0_5_FSEVT2_7_ISR, ".text:tools");
#pragma CODE_SECTION(Aif2_Exception_ISR, ".text:tools");
#pragma CODE_SECTION(UTILS_waitForHw, ".text:tools");
#pragma CODE_SECTION(UTILS_bytecmp, ".text:tools");
#pragma CODE_SECTION(UTILS_triggerFsync, ".text:tools");
#pragma CODE_SECTION(UTILS_triggerBoardsSync, ".text:tools");
#pragma CODE_SECTION(UTILS_configRtwp, ".text:tools");
#pragma CODE_SECTION(UTILS_readRtwpAnt01, ".text:tools");
#pragma CODE_SECTION(UTILS_boardsSync, ".text:tools");
#pragma CODE_SECTION(UTILS_resetDsp, ".text:tools");


////// Global variables for test utils
CSL_TmrHandle     hTmr = NULL;
CSL_TmrObj        TmrObj;
CSL_TmrHwSetup    TmrSetup;

#if EVM_TYPE == 5 || EVM_TYPE == 7
#ifdef EXT_CLOCK
CSL_TmrHandle     hTmr10MHz = NULL;
CSL_TmrObj        Tmr10MHzObj;
CSL_TmrHwSetup    Tmr10MHzSetup;
#endif
CSL_TmrHandle     hTmrBoards = NULL;
CSL_TmrObj        TmrBoardsObj;
CSL_TmrHwSetup    TmrBoardsSetup;
#endif

/////////
//KeyStone II compatibility
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define CSL_PSC_PD_AI 							CSL_PSC_PD_AIF
#define CSL_PSC_LPSC_AI 						CSL_PSC_LPSC_AIF
#define AIF2FL_CONTROL_REGS					CSL_AIF_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_GLOBAL_CFG_REGS 	CSL_AIF_CFG_PKTDMA_GLOBAL_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_RX_CFG_REGS 		CSL_AIF_CFG_PKTDMA_RX_CFG_REGS
#define AIF2FL_CFG_CPPI_DMA_TX_CFG_REGS		CSL_AIF_CFG_PKTDMA_TX_CFG_REGS
#define CSL_CGEM0_5_REG_BASE_ADDRESS_REGS		CSL_C66X_COREPAC_REG_BASE_ADDRESS_REGS
#define	CSL_CP_INTC_0_REGS						CSL_CIC_0_REGS
#define	CSL_CP_INTC_1_REGS						CSL_CIC_1_REGS
#define	CSL_CP_INTC_2_REGS						CSL_CIC_2_REGS
#define CSL_GEM_AIF_EVENT0						CSL_C66X_COREPAC_AIF_ATEVT0
#define CSL_GEM_AIF_EVENT1						CSL_C66X_COREPAC_AIF_ATEVT1
#define CSL_GEM_AIF_EVENT2						CSL_C66X_COREPAC_AIF_ATEVT2
#define CSL_GEM_AIF_EVENT3						CSL_C66X_COREPAC_AIF_ATEVT3
#define CSL_GEM_AIF_EVENT4						CSL_C66X_COREPAC_AIF_ATEVT4
#define CSL_GEM_AIF_EVENT5						CSL_C66X_COREPAC_AIF_ATEVT5
#define CSL_GEM_AIF_EVENT6						CSL_C66X_COREPAC_AIF_ATEVT6
#define CSL_GEM_AIF_EVENT7						CSL_C66X_COREPAC_AIF_ATEVT7
#define CSL_GEM_INTC0_OUT_8_PLUS_16_MUL_N		CSL_C66X_COREPAC_CIC_OUT8_PLUS_16_MUL_N
#define CSL_GEM_EVT3							CSL_C66X_COREPAC_EVT3
#define CSL_INTC0_AIF_INTD						CSL_CIC0_AIF_INT
#define CSL_TPCC_0								CSL_EDMACC_0
#define CSL_TPCC_1								CSL_EDMACC_1
#endif

/*define interrupt vector ID for events*/
#define 	COM_AIF2_SEVT7_EVENT 				CSL_INTC_VECTID_4
#define 	COM_AIF2_SEVT0_5_EVENT				CSL_INTC_VECTID_5
#define 	COM_AIF2_EXCEP_EVENT		 		CSL_INTC_VECTID_6
#define     COM_AIF2_SEVT6_EVENT                CSL_INTC_VECTID_10
#define     COM_AIF2_SEVT5_EVENT                CSL_INTC_VECTID_11
#define     COM_AIF2_SEVT4_EVENT                CSL_INTC_VECTID_13
#if EVM_TYPE == 5 || EVM_TYPE == 7
#define     COM_TIMER5_EVENT                    CSL_INTC_VECTID_12
#endif

#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
volatile int32_t K2LteCount = 0;
volatile int32_t K2Count = 0;
#endif




CSL_IntcRegs     *IntcRegs= (CSL_IntcRegs *)CSL_CGEM0_5_REG_BASE_ADDRESS_REGS;
CSL_CPINTCRegs   *CicRegs[3];
CSL_CPINTC_Handle hIntc0Handle;

#ifndef USESYSBIOS
CSL_IntcObj       intcObj[16];
CSL_IntcHandle    hIntc[16];
static CSL_IntcContext     intcContext;
CSL_IntcEventHandlerRecord EventRecord[16];
#endif

CSL_PllcHandle    hPllc;

uint8_t          DSP_procId;

uint8_t		   keepAifOff = 0;
uint8_t		   keepStartupDelay = 1;

uint8_t 			onLoss = 0;
uint8_t			count = 0;

#ifndef RUNTIME
extern AIF_ConfigHandle hConfigAif;
#endif
//#define NUM_AxC 15
//extern uint32_t dio_data[NUM_AxC * 32 * 2];

////// Local functions to this file - prototype
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_RadT_Sevt7_FSEVT1_ISR();
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_RadT_Sevt4_ISR();
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_RadT_Sevt5_ISR();
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_RadT_Sevt6_ISR();
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_RadT_Sevt0_5_FSEVT2_7_ISR();
#ifndef USESYSBIOS
interrupt
#endif
void Aif2_Exception_ISR();
#if EVM_TYPE == 5 || EVM_TYPE == 7
#ifndef USESYSBIOS
interrupt
#endif
void Timer5Isr();
uint32_t         timer5Count=0;
#endif

void           dummyIsr();
UTILS_FxnPtr   fsevt1_userIsr = dummyIsr;
UTILS_FxnPtr   aif2evt5_userIsr = dummyIsr;
UTILS_FxnPtr   aif2evt6_userIsr = dummyIsr;
UTILS_FxnPtr   aif2evt4_userIsr = dummyIsr;

void dummyIsr() {

}

CSL_GpioHandle    hGpio = NULL;

void UTILS_GPIO8_setup()
{
uint32_t pinNum = 8;

    hGpio = CSL_GPIO_open(0);
    if ((hGpio == NULL)) {
        printf("GPIO: Error opening the instance.\n");
    }

    // Set GPIO pin number 8 as an input pin
    CSL_GPIO_setPinDirInput(hGpio, pinNum);
}
#endif // _TMS320C6X

void
AIF_enable(
)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = aif2_mmap(CSL_PSC_REGS, 0x10000);
    if (CSL_FEXT(((CSL_PscRegs *) mem_base_PSC)->PDSTAT[CSL_PSC_PD_AIF], PSC_PDSTAT_STATE) != PSC_PDSTATE_ON)
    {
        /* Enable the domain */
        CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[CSL_PSC_PD_AIF], PSC_PDCTL_NEXT, ON);
    }
    /* Enable MDCTL */
    CSL_FINS (((CSL_PscRegs *) mem_base_PSC)->MDCTL[CSL_PSC_LPSC_AIF], PSC_MDCTL_NEXT, PSC_MODSTATE_ENABLE);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << CSL_PSC_PD_AIF);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, CSL_PSC_PD_AIF, CSL_PSC_PD_AIF) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    if (CSL_PSC_getModuleState (CSL_PSC_LPSC_AI) != PSC_MODSTATE_ENABLE) {
        /* Turn on the power domain */
        CSL_PSC_enablePowerDomain (CSL_PSC_PD_AI);
        /* Enable MDCTL */
        CSL_PSC_setModuleNextState (CSL_PSC_LPSC_AI, PSC_MODSTATE_ENABLE);
        /* Apply the domain */
        CSL_PSC_startStateTransition (CSL_PSC_PD_AI);
        /* Wait for it to finish */
        while (! CSL_PSC_isStateTransitionDone (CSL_PSC_PD_AI));

        /* Log  PSC status if not ok */
        if (!((CSL_PSC_getPowerDomainState(CSL_PSC_PD_AI) == PSC_PDSTATE_ON) &&
                (CSL_PSC_getModuleState (CSL_PSC_LPSC_AI) == PSC_MODSTATE_ENABLE)))
        {
#ifndef USESYSBIOS
            printf("Error: failed to turn AIF2 power domain on\n");
#else
            System_printf("Error: failed to turn AIF2 power domain on\n");
#endif
        }
    }
#endif

}


void
AIF_disable(
)
{
	/* Power OFF AIF2 */
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = aif2_mmap(CSL_PSC_REGS, 0x10000);
    /* Power Off */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->PDCTL[CSL_PSC_PD_AIF], PSC_PDCTL_NEXT, OFF);
    /* Apply the domain */
    ((CSL_PscRegs *) mem_base_PSC)->PTCMD =   (1 << CSL_PSC_PD_AIF);
    /* Wait for it to finish */
    while(CSL_FEXTR (((CSL_PscRegs *) mem_base_PSC)->PTSTAT, CSL_PSC_PD_AIF, CSL_PSC_PD_AIF) == 1);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    //Wait for any previous transitions to complete
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_AI));
    //Write Switch input into the corresponding PDCTL register
    CSL_PSC_disablePowerDomain (CSL_PSC_PD_AI);
    //Write PTCMD to start the transition
    CSL_PSC_startStateTransition (CSL_PSC_PD_AI);
    //Wait for the transition to complete
    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_AI));
#endif
}

void AIF_pscDisableResetIso(
)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    uint32_t mem_base_PSC;
    mem_base_PSC = aif2_mmap(CSL_PSC_REGS, 0x10000);
    /* Disable MDCTL */
    CSL_FINST (((CSL_PscRegs *) mem_base_PSC)->MDCTL[CSL_PSC_LPSC_AIF], PSC_MDCTL_RSTISO, DISABLE);
    munmap((void *) mem_base_PSC, 0x10000);
#else
    /* Disable reset isolation */
    CSL_PSC_disableModuleResetIsolation(CSL_PSC_LPSC_AI);
#endif
}

#ifdef _TMS320C6X

void
UTILS_doCleanup(
    AIF_ConfigHandle    hAif,
    uint32_t timer
)
{
	
	/* Clear the Interrupt */
#ifndef USESYSBIOS
		//disable global interrupt
	CSR&= 0xFFFFFFFE;
    //CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS,CSL_INTC0_AIF_INTD);
 	IER= 0;
	ICR= 0xFFF0;
	
#else
	// modifications for BIOS6 usage
	{
	Hwi_disable();

    Hwi_disableInterrupt(COM_AIF2_SEVT7_EVENT);
	Hwi_disableInterrupt(COM_AIF2_SEVT5_EVENT);
	Hwi_disableInterrupt(COM_AIF2_SEVT6_EVENT);
	Hwi_disableInterrupt(COM_AIF2_EXCEP_EVENT);
#if EVM_TYPE == 5 || EVM_TYPE == 7
	Hwi_disableInterrupt(COM_TIMER5_EVENT);
#endif
	}

#endif

	// disable all reset isolation bits
	* (volatile int*)0x023100F0 = 0;

	if (DSP_procId == 1) UTILS_resetTimer(timer);
	AIF_resetFsync(hAif);

	// Disable AIF reset isolation mode (bootloader sets it on)
	CSL_PSC_disableModuleResetIsolation(CSL_PSC_LPSC_AI);
	AIF_resetAif(hAif);
	AIF_disable();

	UTILS_resetQmss();
	
	Aif2_osalResetMulticoreSyncBarrier();

	/* Take AIF out of power saver again since AIF_resetAif is powering it off */
	if (keepAifOff == 0) AIF_enable();
		
}

void
UTILS_resetTimer(
		uint32_t timer
)
{
CSL_Status status;

   CSL_TmrEnamode tmrMode;
   tmrMode = CSL_TMR_ENAMODE_DISABLE;
   
   if (hTmr == NULL) {
      CSL_tmrInit(NULL);
      hTmr = CSL_tmrOpen(&TmrObj,timer,NULL,&status);
   }

   CSL_tmrHwControl(hTmr,CSL_TMR_CMD_STOP64,&tmrMode);
   hTmr->regs->TGCR  = 0x00000000;
   hTmr->regs->TCR   = 0x00000000; 
   if (hTmr != NULL){CSL_tmrClose(hTmr);hTmr=NULL;}

}

void
UTILS_startTimer(
)
{
    CSL_TmrEnamode tmrMode;

#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6 || EVM_TYPE == 7
    // one shot timer in clock mode as signal integrity of the setup is not optimal for timer 0 output -> physync path
    tmrMode = CSL_TMR_ENAMODE_ENABLE;
#else
    tmrMode = CSL_TMR_ENAMODE_CONT;
#endif
    //TmrSetup.tmrTimerPeriodLo  = prdLo;
    /*------------------------------------------------------------------*/
    /* Start timer in continuous mode                                   */
    /*------------------------------------------------------------------*/
    if (hTmr != NULL) {
    	CSL_tmrHwControl(hTmr,CSL_TMR_CMD_START64,&tmrMode);
    	hTmr->regs->TGCR = 0x00000003;
    }
    else printf (" ERROR no timer start \n");

}

//Timer is set as a parameter to allow the user to choose the timer he want for EVM external sync.
void
UTILS_initTimer(
		uint32_t timer
)

{
	/*start timer 0 for EVM external sync */
	
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6 || EVM_TYPE == 7
    uint32_t prdLo = 0x00000200;
#else
    uint32_t prdLo = 0x00190000; // 10 ms, timer runs at CPU/6
#endif

    uint32_t prdHi = 0x00000000;
    uint32_t cntLo = 0x00000000; 
    uint32_t cntHi = 0x00000000; 
    CSL_Status status;

	// Module Initialization
    CSL_tmrInit(NULL);

    //Open Handle 
    hTmr = CSL_tmrOpen(&TmrObj,timer,NULL,&status);

    CSL_tmrHwControl(hTmr,CSL_TMR_CMD_RESET_TIMLO,NULL);
    CSL_tmrHwControl(hTmr,CSL_TMR_CMD_RESET_TIMHI,NULL);

      /*------------------------------------------------------------------*/
      /* Timer Dual 64-bit GP Onetime Mode Test				*/
      /*------------------------------------------------------------------*/    

      /*------------------------------------------------------------------*/
      /* Configuring timer0 for 64-bit GP Mode		 		*/
      /*------------------------------------------------------------------*/
    CSL_tmrGetHwSetup(hTmr, &TmrSetup);

    TmrSetup.tmrClksrcLo       = CSL_TMR_CLKSRC_INTERNAL;
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6 || EVM_TYPE == 7
    TmrSetup.tmrClockPulseLo   = CSL_TMR_CP_CLOCK;
#else
    TmrSetup.tmrClockPulseLo   = CSL_TMR_CP_PULSE;
#endif
	TmrSetup.tmrInvInpLo       = CSL_TMR_INVINP_INVERTED;
	TmrSetup.tmrInvOutpLo      = CSL_TMR_INVOUTP_INVERTED;
	TmrSetup.tmrIpGateLo       = CSL_TMR_CLOCK_INP_NOGATE;
    TmrSetup.tmrPulseWidthLo   = CSL_TMR_PWID_THREECLKS;
    TmrSetup.tmrTimerPeriodLo  = prdLo;    
    TmrSetup.tmrClksrcHi      = CSL_TMR_CLKSRC_INTERNAL;    
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6 || EVM_TYPE == 7
    TmrSetup.tmrClockPulseHi   = CSL_TMR_CP_CLOCK;
#else
    TmrSetup.tmrClockPulseHi   = CSL_TMR_CP_PULSE;
#endif
    TmrSetup.tmrInvInpHi       = CSL_TMR_INVINP_INVERTED;
	TmrSetup.tmrInvOutpHi      = CSL_TMR_INVOUTP_INVERTED;	
	TmrSetup.tmrIpGateHi       = CSL_TMR_CLOCK_INP_NOGATE;
    TmrSetup.tmrPulseWidthHi   = CSL_TMR_PWID_THREECLKS;    
    TmrSetup.tmrTimerPeriodHi  = prdHi;
    TmrSetup.tmrTimerMode      = CSL_TMR_TIMMODE_GPT;
 
      /*------------------------------------------------------------------*/
      /* Set count to 0							*/
      /*------------------------------------------------------------------*/
    TmrSetup.tmrTimerCounterLo     = cntLo;
    TmrSetup.tmrTimerCounterHi     = cntHi;
    CSL_tmrHwSetup(hTmr,&TmrSetup);

#if (defined(DEVICE_K2K) || defined(DEVICE_K2H))
    CSL_BootCfgUnlockKicker();
    CSL_BootCfgSetTimerOutputSelection ((uint8_t)(2*timer),(uint8_t)(2*timer));
    //CSL_BootCfgLockKicker();
#else
    CSL_BootCfgSetTimerOutputSelection ((uint8_t)(2*timer),NULL);
#endif
}

void
UTILS_triggerFsync(
	AIF_ConfigHandle hAif
)
{
    uint16_t ctrlArg;

	ctrlArg = true;
	if (hAif->aif2TimerSyncSource== AIF2FL_SW_SYNC) {
		Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_DEBUG_SYNC, (void *)&ctrlArg);
	} else {
		if (keepStartupDelay == 1) UTILS_waitForHw(200000000);
		UTILS_startTimer();
	}

}

#if EVM_TYPE == 5 || EVM_TYPE == 7
#ifdef EXT_CLOCK
void routeTimer16HzOutToTimerOut1Pin(
		uint32_t timer
)
{
    *TOUTPSEL &= ~0x3E0;
    *TOUTPSEL |=  0x180; // TMR6L on TIMO1
    *TOUTPSEL |= ((2*timer)<<5);
}

void
UTILS_config10MHzTimer(
		uint32_t timer		
)
{

    CSL_Status status;
    CSL_TmrEnamode tmrMode;
    uint32_t prdLo = 312500-1; // 10MHz/16Hz=625,000; half of that because clock mode
    uint32_t prdHi = 0x00000000;
    uint32_t cntLo = 0x00000000;
    uint32_t cntHi = 0x00000000;

   tmrMode = CSL_TMR_ENAMODE_DISABLE;

   if (hTmr10MHz == NULL) {
      CSL_tmrInit(NULL);
      hTmr10MHz = CSL_tmrOpen(&Tmr10MHzObj,timer,NULL,&status);
   }

   CSL_tmrHwControl(hTmr10MHz,CSL_TMR_CMD_STOP64,&tmrMode);
   hTmr10MHz->regs->TGCR  = 0x00000000;
   hTmr10MHz->regs->TCR   = 0x00000000;
   if (hTmr10MHz != NULL){CSL_tmrClose(hTmr10MHz);hTmr10MHz=NULL;}

   // Module Initialization
   CSL_tmrInit(NULL);

   //Open Handle
   hTmr10MHz = CSL_tmrOpen(&Tmr10MHzObj,timer,NULL,&status);

   CSL_tmrHwControl(hTmr10MHz,CSL_TMR_CMD_RESET_TIMLO,NULL);
   CSL_tmrHwControl(hTmr10MHz,CSL_TMR_CMD_RESET_TIMHI,NULL);

   CSL_tmrGetHwSetup(hTmr10MHz, &Tmr10MHzSetup);

   Tmr10MHzSetup.tmrClksrcLo       = CSL_TMR_CLKSRC_TMRINP;
   Tmr10MHzSetup.tmrClockPulseLo   = CSL_TMR_CP_CLOCK;
   Tmr10MHzSetup.tmrInvInpLo       = CSL_TMR_INVINP_INVERTED;
   Tmr10MHzSetup.tmrInvOutpLo      = CSL_TMR_INVOUTP_INVERTED;
   Tmr10MHzSetup.tmrIpGateLo       = CSL_TMR_CLOCK_INP_NOGATE;
   Tmr10MHzSetup.tmrPulseWidthLo   = CSL_TMR_PWID_THREECLKS;
   Tmr10MHzSetup.tmrTimerPeriodLo  = prdLo;
   Tmr10MHzSetup.tmrClksrcHi      = CSL_TMR_CLKSRC_INTERNAL;
   Tmr10MHzSetup.tmrClockPulseHi   = CSL_TMR_CP_PULSE;
   Tmr10MHzSetup.tmrInvInpHi       = CSL_TMR_INVINP_INVERTED;
   Tmr10MHzSetup.tmrInvOutpHi      = CSL_TMR_INVOUTP_INVERTED;
   Tmr10MHzSetup.tmrIpGateHi       = CSL_TMR_CLOCK_INP_NOGATE;
   Tmr10MHzSetup.tmrPulseWidthHi   = CSL_TMR_PWID_THREECLKS;
   Tmr10MHzSetup.tmrTimerPeriodHi  = prdHi;
   Tmr10MHzSetup.tmrTimerMode      = CSL_TMR_TIMMODE_GPT;

      /*------------------------------------------------------------------*/
      /* Set count to 0                         */
      /*------------------------------------------------------------------*/
   Tmr10MHzSetup.tmrTimerCounterLo     = cntLo;
   Tmr10MHzSetup.tmrTimerCounterHi     = cntHi;
   CSL_tmrHwSetup(hTmr10MHz,&Tmr10MHzSetup);

   tmrMode = CSL_TMR_ENAMODE_CONT;

   /*------------------------------------------------------------------*/
   /* Start timer in continuous mode                                   */
   /*------------------------------------------------------------------*/
   if (hTmr10MHz != NULL) CSL_tmrHwControl(hTmr10MHz,CSL_TMR_CMD_START64,&tmrMode);
   else printf (" ERROR no timer start \n");

   /* Route 16Hz signal to TIMO1 */
   routeTimer16HzOutToTimerOut1Pin(timer);


}

#endif

// Configure timer at 10ms pace to call the Boards sync function on Appleton EVM
void
UTILS_triggerBoardsSync(
		uint32_t timer
)
{

	CSL_Status status;
    CSL_TmrEnamode tmrMode;
    uint32_t prdLo = 0x00190000; // 10 ms, timer runs at CPU/6
    uint32_t prdHi = 0x00000000;
    uint32_t cntLo = 0x00000000;
    uint32_t cntHi = 0x00000000;

   tmrMode = CSL_TMR_ENAMODE_DISABLE;

   if (hTmrBoards == NULL) {
	  CSL_tmrInit(NULL);
	  hTmrBoards = CSL_tmrOpen(&TmrBoardsObj,timer,NULL,&status);
   }

   CSL_tmrHwControl(hTmrBoards,CSL_TMR_CMD_STOP64,&tmrMode);
   hTmrBoards->regs->TGCR  = 0x00000000;
   hTmrBoards->regs->TCR   = 0x00000000;
   if (hTmrBoards != NULL){CSL_tmrClose(hTmrBoards);hTmrBoards=NULL;}

   // Module Initialization
   CSL_tmrInit(NULL);

   //Open Handle
   hTmrBoards = CSL_tmrOpen(&TmrBoardsObj,timer,NULL,&status);

   CSL_tmrHwControl(hTmrBoards,CSL_TMR_CMD_RESET_TIMLO,NULL);
   CSL_tmrHwControl(hTmrBoards,CSL_TMR_CMD_RESET_TIMHI,NULL);

   CSL_tmrGetHwSetup(hTmrBoards, &TmrBoardsSetup);

   TmrBoardsSetup.tmrClksrcLo       = CSL_TMR_CLKSRC_INTERNAL;
   TmrBoardsSetup.tmrClockPulseLo   = CSL_TMR_CP_PULSE;
   TmrBoardsSetup.tmrInvInpLo       = CSL_TMR_INVINP_INVERTED;
   TmrBoardsSetup.tmrInvOutpLo      = CSL_TMR_INVOUTP_INVERTED;
   TmrBoardsSetup.tmrIpGateLo       = CSL_TMR_CLOCK_INP_NOGATE;
   TmrBoardsSetup.tmrPulseWidthLo   = CSL_TMR_PWID_THREECLKS;
   TmrBoardsSetup.tmrTimerPeriodLo  = prdLo;
   TmrBoardsSetup.tmrClksrcHi      = CSL_TMR_CLKSRC_INTERNAL;
   TmrBoardsSetup.tmrClockPulseHi   = CSL_TMR_CP_PULSE;
   TmrBoardsSetup.tmrInvInpHi       = CSL_TMR_INVINP_INVERTED;
   TmrBoardsSetup.tmrInvOutpHi      = CSL_TMR_INVOUTP_INVERTED;
   TmrBoardsSetup.tmrIpGateHi       = CSL_TMR_CLOCK_INP_NOGATE;
   TmrBoardsSetup.tmrPulseWidthHi   = CSL_TMR_PWID_THREECLKS;
   TmrBoardsSetup.tmrTimerPeriodHi  = prdHi;
   TmrBoardsSetup.tmrTimerMode      = CSL_TMR_TIMMODE_GPT;

      /*------------------------------------------------------------------*/
      /* Set count to 0							*/
      /*------------------------------------------------------------------*/
   TmrBoardsSetup.tmrTimerCounterLo     = cntLo;
   TmrBoardsSetup.tmrTimerCounterHi     = cntHi;
   CSL_tmrHwSetup(hTmrBoards,&TmrBoardsSetup);

   tmrMode = CSL_TMR_ENAMODE_CONT;

   /*------------------------------------------------------------------*/
   /* Start timer in continuous mode                                   */
   /*------------------------------------------------------------------*/
   if (hTmrBoards != NULL) CSL_tmrHwControl(hTmrBoards,CSL_TMR_CMD_START64,&tmrMode);
   else printf (" ERROR no timer start \n");


}

#ifndef RUNTIME
//Timer is set as a parameter to allow the user to choose the timer he want to generate the 10 ms event.
void
UTILS_boardsSync (
		uint32_t timer,
		uint32_t timerpll
#ifdef EXT_CLOCK
		,uint32_t timer10Mhz
#endif
)
{

#if (defined(DEVICE_K2K) || defined(DEVICE_K2H))
	CSL_BootCfgUnlockKicker();
#endif
	spi_init();
	if (hConfigAif->mode == AIF_WCDMA_MODE) {
#if EVM_TYPE == 5
		// configure SCBP FPGA for RTWP measurement
		RTWP_open(1.0);
#endif
	}
#if EVM_TYPE == 5
#ifdef DBG
    memset((void *) &gDbg, 0, sizeof(dbg_t));
#endif
    BoardsSyncInit (&BoardsSync, TIMER_COUNTS_IN_1SEC, BOARDSSYNC_AVG_ERR_PPB_REQUIREMENT,
                 BOARDSSYNC_MAX_ERR_PPB_REQUIREMENT,
                 BOARDSSYNC_ENABLE_CONTROLLED_RATE_OF_TIMING_ADJUSTMENT,
                 BOARDSSYNC_RESYNC_STATE_MAX_TIMING_ADJUSTMENT_RATE,
                 timerpll);

    /* initialize DAC */
    dac_write_val(DAC_INIT_VAL);

    /* wait a little */
    {
       volatile int j;
       for(j = 0; j < 1000000; j++);
    }

    switchOffPps();
    configTimer(BoardsSync.ppsLossTimeCount);
    switchOnPps();

#ifdef EXT_CLOCK
    UTILS_config10MHzTimer(timer10Mhz);
#endif

    /* Start the synchronisation over the AIF2_CLOCK_SYNC function.
     * BEYOND THIS POINT, REAL-TIME IS NEEDED TO KEEP BOARDS IN SYNC
     */
    UTILS_triggerBoardsSync(timer);

    // let DSP_2 go beyond this point and start timer 0 on DSP_1 to generate the required external frame synchronization
	while (BoardsSync.pll.est.converged != 1)
	{
		asm("     IDLE");
	}
#if (defined(DEVICE_K2K) || defined(DEVICE_K2H))
	//CSL_BootCfgLockKicker();
#endif
#endif
}
#endif

#endif // Appleton EVM

/* Master Salve configuration for the Lyrtech EVM */
void 
UTILS_configMasterSlave(
)
{
#if EVM_TYPE == 0
    int32_t   retcode;
    // Initialize EVM module from Lyrtech
    // Must always be the first BSL functions called
    if ((retcode=EVM_init())!=EVM_INITOK)
    {
        exit(retcode);
    }

    //Enable AIF2 module power
    if(EVM_dspid(&DSP_procId))
    {
    	exit(0);
    }
    DSP_procId++; // Make DSP_1 and DSP_2	
#endif
}

#endif // _TMS320C6X

/*print links configuration for this test */
void
UTILS_printLinkConfig(
    AIF_ConfigHandle    hAif
)
{
	int32_t i;
#ifndef USESYSBIOS
	if(hAif->protocol==AIF2FL_LINK_PROTOCOL_CPRI)
        printf("AIF CPRI mode\n");
	else
		printf("AIF OBSAI mode\n");

	for(i=0; i<6; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
			printf("link %d runs at %dx rate\n", i, hAif->linkConfig[i].linkRate);

			if(AIF_LINK_DATA_TYPE_DL==hAif->linkConfig[i].outboundDataType)
				printf("        outbound data type: DL \n");
			else
				printf("        outbound data type: UL_RSA\n");

			if(AIF2FL_DATA_WIDTH_7_BIT==hAif->linkConfig[i].outboundDataWidth)
				printf("        outbound data width: 7 bit\n");
			else if(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].outboundDataWidth)
				printf("        outbound data width: 8 bit\n");
			else if(AIF2FL_DATA_WIDTH_15_BIT==hAif->linkConfig[i].outboundDataWidth)
				printf("        outbound data width: 15 bit\n");
			else 
				printf("        outbound data width: 16 bit\n");

			if(AIF_LINK_DATA_TYPE_DL==hAif->linkConfig[i].inboundDataType)
				printf("        inbound data type: DL \n");
			else
				printf("        inbound data type: UL_RSA\n");

			if(AIF2FL_DATA_WIDTH_7_BIT==hAif->linkConfig[i].inboundDataWidth)
				printf("        inbound data width: 7 bit\n");
			else if(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].inboundDataWidth)
				printf("        inbound data width: 8 bit\n");
			else if(AIF2FL_DATA_WIDTH_15_BIT==hAif->linkConfig[i].inboundDataWidth)
				printf("        inbound data width: 15 bit\n");
			else 
				printf("        inbound data width: 16 bit\n");
		}
		else
			printf("link %d is disabled\n", i);
	}
#else
	if(hAif->protocol==AIF2FL_LINK_PROTOCOL_CPRI)
		System_printf("AIF CPRI mode\n");
	else
		System_printf("AIF OBSAI mode\n");

	for(i=0; i<6; i++)
	{
		if(1==hAif->linkConfig[i].linkEnable)
		{
			System_printf("link %d runs at %dx rate\n", i, hAif->linkConfig[i].linkRate);

			if(AIF_LINK_DATA_TYPE_DL==hAif->linkConfig[i].outboundDataType)
				System_printf("        outbound data type: DL \n");
			else
				System_printf("        outbound data type: UL_RSA\n");

			if(AIF2FL_DATA_WIDTH_7_BIT==hAif->linkConfig[i].outboundDataWidth)
				System_printf("        outbound data width: 7 bit\n");
			else if(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].outboundDataWidth)
				System_printf("        outbound data width: 8 bit\n");
			else if(AIF2FL_DATA_WIDTH_15_BIT==hAif->linkConfig[i].outboundDataWidth)
				System_printf("        outbound data width: 15 bit\n");
			else
				System_printf("        outbound data width: 16 bit\n");

			if(AIF_LINK_DATA_TYPE_DL==hAif->linkConfig[i].inboundDataType)
				System_printf("        inbound data type: DL \n");
			else
				System_printf("        inbound data type: UL_RSA\n");

			if(AIF2FL_DATA_WIDTH_7_BIT==hAif->linkConfig[i].inboundDataWidth)
				System_printf("        inbound data width: 7 bit\n");
			else if(AIF2FL_DATA_WIDTH_8_BIT==hAif->linkConfig[i].inboundDataWidth)
				System_printf("        inbound data width: 8 bit\n");
			else if(AIF2FL_DATA_WIDTH_15_BIT==hAif->linkConfig[i].inboundDataWidth)
				System_printf("        inbound data width: 15 bit\n");
			else
				System_printf("        inbound data width: 16 bit\n");
		}
		else
			System_printf("link %d is disabled\n", i);
	}
#endif
}

#ifdef _TMS320C6X

void 
UTILS_aifIntcSetup()
{
	CicRegs[0]= (CSL_CPINTCRegs * ) CSL_CP_INTC_0_REGS;
	CicRegs[1]= (CSL_CPINTCRegs * ) CSL_CP_INTC_1_REGS;
	CicRegs[2]= (CSL_CPINTCRegs * ) CSL_CP_INTC_2_REGS;	

	CicRegs[0]->ENABLE_CLR_INDEX_REG = CSL_CPINTC_ENABLE_CLR_INDEX_REG_RESETVAL;
	CicRegs[1]->ENABLE_CLR_INDEX_REG = CSL_CPINTC_ENABLE_CLR_INDEX_REG_RESETVAL;
	CicRegs[2]->ENABLE_CLR_INDEX_REG = CSL_CPINTC_ENABLE_CLR_INDEX_REG_RESETVAL;

	 //Clear all events
	IntcRegs->EVTCLR[0]= 	0xFFFFFFFF;
	IntcRegs->EVTCLR[1]= 	0xFFFFFFFF;
	IntcRegs->EVTCLR[2]= 	0xFFFFFFFF;
	IntcRegs->EVTCLR[3]= 	0xFFFFFFFF;

	/* Routing Aif2 exception event thru INTC0 */
	hIntc0Handle = CSL_CPINTC_open(0); // handle for INTC0
	CSL_CPINTC_disableAllHostInterrupt(hIntc0Handle);
    CSL_CPINTC_setNestingMode(hIntc0Handle, CPINTC_NO_NESTING);
    CSL_CPINTC_mapSystemIntrToChannel (hIntc0Handle, CSL_INTC0_AIF_INTD , 8); // assuming core0
    CSL_CPINTC_enableSysInterrupt (hIntc0Handle, CSL_INTC0_AIF_INTD );
    //CSL_CPINTC_enableHostInterrupt (hIntc0Handle, 8);


#ifndef USESYSBIOS
    {
  	CSL_IntcParam                   vectId; 	
  	CSL_IntcGlobalEnableState       state;

	intcContext.eventhandlerRecord = EventRecord;
	intcContext.numEvtEntries = 16;
	CSL_intcInit(&intcContext);
 
#ifndef NO_AIF_LLD
	/* map AIF2_SEVT7 for time tracking - 10ms tick */
	vectId = COM_AIF2_SEVT7_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_AIF2_SEVT7_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT7_EVENT], CSL_GEM_AIF_EVENT7, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_AIF2_SEVT7_EVENT,&Aif2_RadT_Sevt7_FSEVT1_ISR);
	/* map AIF2_SEVT0-5 one per link for EDMA - FOR DEBUG ONLY TO CHECK 16-chip tick for each of the links */
	vectId = COM_AIF2_SEVT0_5_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_AIF2_SEVT0_5_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT0_5_EVENT], CSL_GEM_EVT3, &vectId , NULL);
	//hIntc[COM_AIF2_SEVT0_5_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT0_5_EVENT], CSL_TPCC1_AIF_SEVT0, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_AIF2_SEVT0_5_EVENT,&Aif2_RadT_Sevt0_5_FSEVT2_7_ISR);
    /* map AIF2_SEVT4 for Cpri Hyper-Frame int */
	vectId = COM_AIF2_SEVT4_EVENT;
	/* Opening a intc handle for this event */
	hIntc[COM_AIF2_SEVT4_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT4_EVENT], CSL_GEM_AIF_EVENT4, &vectId , NULL);
	// hook ISR for this CPU interrupt
	CSL_intcHookIsr(COM_AIF2_SEVT4_EVENT,&Aif2_RadT_Sevt4_ISR);
#endif
    /* map AIF2_SEVT5 for Lte slot int */
    vectId = COM_AIF2_SEVT5_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_AIF2_SEVT5_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT5_EVENT], CSL_GEM_AIF_EVENT5, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_AIF2_SEVT5_EVENT,&Aif2_RadT_Sevt5_ISR);
#ifndef NO_AIF_LLD
    /* map AIF2_SEVT6 for lte SF int */
    vectId = COM_AIF2_SEVT6_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_AIF2_SEVT6_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_SEVT6_EVENT], CSL_GEM_AIF_EVENT6, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_AIF2_SEVT6_EVENT,&Aif2_RadT_Sevt6_ISR);
    /* map AIF2_EXCEPTION for Debug*/
    vectId = COM_AIF2_EXCEP_EVENT;
    /* Opening a intc handle for this event */
    hIntc[COM_AIF2_EXCEP_EVENT] = CSL_intcOpen (&intcObj[COM_AIF2_EXCEP_EVENT], CSL_GEM_INTC0_OUT_8_PLUS_16_MUL_N, &vectId , NULL);
    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_AIF2_EXCEP_EVENT,&Aif2_Exception_ISR);
#if EVM_TYPE == 5 || EVM_TYPE == 7
    /* map TIMER5 int for Boards sync */
    vectId = COM_TIMER5_EVENT;
    /* Opening a intc handle for this event */
#if (defined(DEVICE_K2K) || defined(DEVICE_K2H))
	hIntc[COM_TIMER5_EVENT] = CSL_intcOpen (&intcObj[COM_TIMER5_EVENT], CSL_C66X_COREPAC_TIMER_8_INTL, &vectId , NULL);
#else
    hIntc[COM_TIMER5_EVENT] = CSL_intcOpen (&intcObj[COM_TIMER5_EVENT], CSL_GEM_TINT5L, &vectId , NULL);
#endif

    // hook ISR for this CPU interrupt
    CSL_intcHookIsr(COM_TIMER5_EVENT,&Timer5Isr);
#endif
#endif

#ifndef NO_AIF_LLD
	/* Clear the Event & the interrupt */
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT7_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL); 
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT0_5_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT4_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
#endif
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT5_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
#ifndef NO_AIF_LLD
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT6_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
    CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS,CSL_INTC0_AIF_INTD);
    CSL_intcHwControl(hIntc[COM_AIF2_EXCEP_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
#if EVM_TYPE == 5 || EVM_TYPE == 7
    CSL_intcHwControl(hIntc[COM_TIMER5_EVENT], CSL_INTC_CMD_EVTCLEAR, NULL);
#endif
#endif
	  
    /* Configure the combiner 0 masks for FSYNC1 */
  	//NA use direct CSL_GEM_AIF_EVENT7 so disable all combined event
  	//IntcRegs->EVTMASK[0]= 	~((1<<CSL_INTC_EVENTID_FSEVT1)); 	
	IntcRegs->EVTMASK[0]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[1]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[2]= 	0xFFFFFFFF; 	//mask out other events
	IntcRegs->EVTMASK[3]= 	0xFFFFFFFF; 	//mask out other events

	/* Enable the Event & the interrupt */
    // Enable global host interrupts flag
#ifndef NO_AIF_LLD
    CSL_CPINTC_enableAllHostInterrupt(hIntc0Handle);    
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT7_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT4_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
#endif
	CSL_intcHwControl(hIntc[COM_AIF2_SEVT5_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
#ifndef NO_AIF_LLD
    CSL_intcHwControl(hIntc[COM_AIF2_SEVT6_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
    CSL_intcHwControl(hIntc[COM_AIF2_EXCEP_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
#if EVM_TYPE == 5 || EVM_TYPE == 7
    CSL_intcHwControl(hIntc[COM_TIMER5_EVENT], CSL_INTC_CMD_EVTENABLE, NULL);
#endif
#endif
	CSL_intcGlobalEnable(&state);
	/* Enable NMIs */
	CSL_intcGlobalNmiEnable();

    }
	 
#else
	// modifications for BIOS6 usage
	{Hwi_Params hwiParams;
	Hwi_disable();

    Hwi_disableInterrupt(COM_AIF2_SEVT7_EVENT);
	Hwi_disableInterrupt(COM_AIF2_SEVT5_EVENT);
	Hwi_disableInterrupt(COM_AIF2_SEVT6_EVENT);
	Hwi_disableInterrupt(COM_AIF2_EXCEP_EVENT);
#if EVM_TYPE == 5 || EVM_TYPE == 7
	Hwi_disableInterrupt(COM_TIMER5_EVENT);
#endif

		/* Map events to CPU interrupts */
        /* And plug ISRs into BIOS dispatcher */

   	Hwi_Params_init(&hwiParams);
   	hwiParams.enableInt = false;
   	hwiParams.eventId = CSL_GEM_AIF_EVENT7;
   	Hwi_create(COM_AIF2_SEVT7_EVENT, Aif2_RadT_Sevt7_FSEVT1_ISR, &hwiParams, NULL);

   	hwiParams.eventId = CSL_GEM_AIF_EVENT5;
   	Hwi_create(COM_AIF2_SEVT5_EVENT, Aif2_RadT_Sevt5_ISR, &hwiParams, NULL);

   	hwiParams.eventId = CSL_GEM_AIF_EVENT6;
   	Hwi_create(COM_AIF2_SEVT6_EVENT, Aif2_RadT_Sevt6_ISR, &hwiParams, NULL);

   	hwiParams.eventId = CSL_GEM_INTC0_OUT_8_PLUS_16_MUL_N;
   	Hwi_create(COM_AIF2_EXCEP_EVENT, Aif2_Exception_ISR, &hwiParams, NULL);

#if EVM_TYPE == 5 || EVM_TYPE == 7
   	hwiParams.eventId = CSL_GEM_TINT5L;
   	Hwi_create(COM_TIMER5_EVENT, Timer5Isr, &hwiParams, NULL);
#endif

   	Hwi_clearInterrupt(COM_AIF2_SEVT7_EVENT);
   	Hwi_clearInterrupt(COM_AIF2_SEVT5_EVENT);
   	Hwi_clearInterrupt(COM_AIF2_SEVT6_EVENT);
   	Hwi_clearInterrupt(COM_AIF2_EXCEP_EVENT);
#if EVM_TYPE == 5 || EVM_TYPE == 7
   	Hwi_clearInterrupt(COM_TIMER5_EVENT);
#endif

    CSL_CPINTC_enableAllHostInterrupt(hIntc0Handle);

    Hwi_enableInterrupt(COM_AIF2_SEVT7_EVENT);
    Hwi_enableInterrupt(COM_AIF2_SEVT5_EVENT);
    Hwi_enableInterrupt(COM_AIF2_SEVT6_EVENT);
    Hwi_enableInterrupt(COM_AIF2_EXCEP_EVENT);
#if EVM_TYPE == 5 || EVM_TYPE == 7
    Hwi_enableInterrupt(COM_TIMER5_EVENT);
#endif

    Hwi_enable();
	}

#endif
	
}


/*
* AIF2 exception handler
* Enter this one only a 1000 times per 10ms frame
* And allow test case to complete without hanging in the exceptions
*/
#ifndef RUNTIME
static uint32_t aif2EEventCnt = 0;
static int32_t aif2EFrameCur  = -1;
#if EVM_TYPE == 8
static uint32_t aif2ECpriLossCnt = 0;
#endif
Aif2Fl_EeLinkAInt eeLinkAInt[16];
uint32_t eeLastCpriLossInFrame = 0;
#endif
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_Exception_ISR()
{
#ifndef RUNTIME
    uint8_t eoi = 0;

      if (aifFsyncEventCount[1]>aif2EFrameCur) {
            //if (aif2EFrameCur<16) memcpy(&eeLinkAInt[aif2EFrameCur%16],hConfigAif->aif2EeCount.eeLinkAIntCnt[0],sizeof(Aif2Fl_EeLinkAInt));
            aif2EFrameCur = aifFsyncEventCount[1];
            aif2EEventCnt = 0;
      }

      /* For interrupts routed through the chip-level INTC
      * Disable CPINTC0 Host interrupt (CPINTC output)
      */
      CSL_CPINTC_disableHostInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS, 8);
      /* Clear the CPINTC0 Interrupt */
      CSL_CPINTC_clearSysInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS,CSL_INTC0_AIF_INTD);

      /* Do the required servicing here */
      AIF_getException(hConfigAif);
      /* Enable CPINTC0 Host interrupt (CPINTC output) */
      if (aif2EEventCnt < 1000) CSL_CPINTC_enableHostInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS, 8);
      /* For CPRI relay, handle situation where relay is ON and Appleton/hawking EVM is power-cycled */
#if EVM_TYPE == 4
#ifdef USE_SMA
      if (hConfigAif->aif2EeCount.eeLinkAIntCnt[5].rm_ee_lcv_det_err >= 10){
    	  CSL_CPINTC_disableHostInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS, 8);
    	  hConfigAif->aif2EeCount.eeLinkAIntCnt[5].rm_ee_lcv_det_err = 0;
      }
#else
      if (hConfigAif->aif2EeCount.eeLinkAIntCnt[3].rm_ee_lcv_det_err >= 10) {
    	  CSL_CPINTC_disableHostInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS, 8);
    	  hConfigAif->aif2EeCount.eeLinkAIntCnt[3].rm_ee_lcv_det_err = 0;
      }
#endif
#endif
#if EVM_TYPE == 8
      /* Tag at which frame last Cpri los occurred */
      if (hConfigAif->aif2EeCount.eeLinkAIntCnt[4].rm_ee_los_err > 0)
      {
    	  if (aif2ECpriLossCnt < hConfigAif->aif2EeCount.eeLinkAIntCnt[4].rm_ee_los_err)
    	  {
    		  eeLastCpriLossInFrame = hConfigAif->hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
    		  aif2ECpriLossCnt      = hConfigAif->aif2EeCount.eeLinkAIntCnt[4].rm_ee_los_err;
    	  }
      }
#endif
      /* Re-arming for next interruption */
      Aif2Fl_eeEoiSetup(hConfigAif->hFl, eoi);
      aif2EEventCnt++;
#endif
}

void
UTILS_aif2ExceptIntDisable()
{
#ifndef USESYSBIOS
	CSL_intcHwControl(hIntc[COM_AIF2_EXCEP_EVENT], CSL_INTC_CMD_EVTDISABLE, NULL);
#else
	Hwi_disableInterrupt(COM_AIF2_EXCEP_EVENT);
#endif
}


void
UTILS_aif2IsOnLoss(
		AIF_ConfigHandle hAif,
		uint32_t activeLink
)
{

	if ((hAif->aif2EeCount.eeLinkAIntCnt[activeLink].rm_ee_los_err != 0) && (onLoss == 0))
		{
			UTILS_aif2ExceptIntDisable();
			onLoss = 1;
//			memset(dio_data, 0xAB, sizeof(dio_data));
		}

		if ((CSL_FEXT(hAif->hFl->regs->G_RM_LKS[activeLink].RM_LK_STS0,AIF2_RM_LK_STS0_SYNC_STATUS) == 1) && (onLoss == 1))
		{
			count++;
			if (count == 100)
			{
				AIF_enableException(hAif);
				onLoss = 0;
			}
		}
}


// 10ms frame boundary from PhyTimer
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_RadT_Sevt7_FSEVT1_ISR()
{
//#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
//	  if(K2Count < 15)
//	  {
//		  K2Count++;
//	  } else {
//		  K2Count = 0;
//#endif
		  AIF_fsync1Event7Count();
		  fsevt1_userIsr();
      // Re-enable Aif2 exception handling every 10 ms and ignore first Cpri frame
      if (aifFsyncEventCount[1] > 1)
    	  CSL_CPINTC_enableHostInterrupt((CSL_CPINTC_Handle)CSL_CP_INTC_0_REGS, 8);
//#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
//	  }
//#endif
}

/* combined event to know which incoming EDMA gets triggered*/
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_RadT_Sevt0_5_FSEVT2_7_ISR()
{
	volatile uint32_t intFlag= IntcRegs->MEVTFLAG[3];

	while(intFlag)
	{
		IntcRegs->EVTCLR[3]= intFlag;
		AIF_fsyncEvent2_7Count(intFlag);
		//make sure all pending events are handled before return
		intFlag= IntcRegs->MEVTFLAG[3];
	}
}

// For Cpri control words, using this ISR to monitor cpri HFNs
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_RadT_Sevt4_ISR()
{
   aif2evt4_userIsr();
}

// Used by ABT mechanism, for instance, to implement ping-pong on the TX direction
// For Lte, using this ISR is slot based
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_RadT_Sevt5_ISR()
{
   aif2evt5_userIsr();
}


// Used by ABT mechanism, for instance, to implement ping-pong on the TX direction
// For Lte, using this ISR is subframe based
#ifndef USESYSBIOS
interrupt
#endif
void
Aif2_RadT_Sevt6_ISR()
{
#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
	  if(K2LteCount < 14)
	  {
		  K2LteCount++;
	  } else {
		  K2LteCount = 0;
#endif
		  aif2evt6_userIsr();
#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
	  }
#endif
}    

#if EVM_TYPE == 5 || EVM_TYPE == 7
volatile float rtwp[4];
extern int isAnyInterrupt(void);
extern void TimerInterruptHandler (void *arg);
#ifndef USESYSBIOS
interrupt
#endif
void
Timer5Isr()
{
	float ant1_mW, ant2_mW, ant1_raw, ant2_raw;
	uint32_t retval1, retval2;
	timer5Count++;
	//while(1) {
#if EVM_TYPE == 5
	if (isAnyInterrupt()) {
      TimerInterruptHandler(NULL);
    }
#endif
	//}
	if (hConfigAif->mode == AIF_WCDMA_MODE) {
		// read RTWP measurements for both antennas
		retval1 = RTWP_read(0x00310030, &ant1_mW, &ant1_raw);
		retval2 = RTWP_read(0x10311030, &ant2_mW, &ant2_raw);
		if (retval1 == 0) rtwp[0] = ant1_mW;
		else			  rtwp[0] = -1.0;
		if (retval2 == 0) rtwp[1] = ant2_mW;
		else			  rtwp[1] = -1.0;
	}
}
#endif

#endif // _TMS320C6X

/* Spin in a delay loop */
void
UTILS_waitForHw(
   uint32_t delay
)
{
    volatile uint32_t i, n;

    n = 0;
    for (i = 0; i < delay; i++)
    {
        n = n + 1;
    }
}

int32_t
UTILS_bytecmp(void *x1, void *x2, uint32_t byteCount) {
	int32_t i;
	int8_t *first = (int8_t *)x1, *second = (int8_t*)x2;
	for (i=0;i<byteCount;i++) {
		if (first[i] != second[i]) {
			/* Found the first differing byte.  */
			return (int32_t)(i);
		}
	}
	return -1;
}

#ifdef _TMS320C6X

void
UTILS_resetDsp(
)
{
#if !defined(DEVICE_K2K) && !defined(DEVICE_K2H)
CSL_PllcHandle  hPllc;
	// Disable reset Iso
	CSL_PSC_disableModuleResetIsolation(CSL_PSC_LPSC_AI);
	// Perform a hard reset from PLL controller
	hPllc = CSL_PLLC_open(0);
	CSL_PLLC_setResetCtrlReg (hPllc, CSL_PLLC_RSTCTRL_VALID_KEY, 0);
	CSL_PLLC_setResetCtrlReg (hPllc, CSL_PLLC_RSTCTRL_VALID_KEY, 0);
#endif
}

void
UTILS_deinterleaveLteSuperPacket(
		uint32_t* superPktAxCPtr,
		uint32_t* dstPktPtr,
		uint32_t  numPackedSamples,
		uint32_t  numAxCs,
		uint32_t  payloadSize
)
{

	CSL_Edma3ChannelObj    chObj;
	CSL_Edma3ChannelAttr   chAttr;
	CSL_Status             status;
	CSL_Edma3ChannelHandle hEdmaDataChan;
	CSL_Edma3ParamHandle   hParamBasic;
	CSL_Edma3ParamSetup    myParamSetup;
	CSL_Edma3Handle        hEdma;
	CSL_Edma3Obj           edmaObj;

	hEdma            = CSL_edma3Open(&edmaObj,CSL_TPCC_1,NULL,&status);
	// Channel open
	chAttr.regionNum = CSL_EDMA3_REGION_GLOBAL;
	chAttr.chaNum    = 0;
	status           = 0;

	hEdmaDataChan    = CSL_edma3ChannelOpen(&chObj, CSL_TPCC_1, &chAttr, &status);
	status           = CSL_edma3HwChannelSetupParam(hEdmaDataChan, 0);

	// Change Channel Default queue setup from 0 to 2
	status           = CSL_edma3HwChannelSetupQue(hEdmaDataChan, CSL_EDMA3_QUE_0);

	// Set priority level on this queue
	CSL_edma3SetEventQueuePriority(hEdma, CSL_EDMA3_QUE_0, CSL_EDMA3_QUE_PRI_4);

	hParamBasic      = CSL_edma3GetParamHandle(hEdmaDataChan, 0, &status);
	// Setup the EDMA parameters
	myParamSetup.option = CSL_EDMA3_OPT_MAKE(CSL_EDMA3_ITCCH_DIS,
	                                         CSL_EDMA3_TCCH_DIS,
	                                         CSL_EDMA3_ITCINT_DIS,
	                                         CSL_EDMA3_TCINT_EN,
	                                         0,
	                                         CSL_EDMA3_TCC_NORMAL,
	                                         CSL_EDMA3_FIFOWIDTH_NONE,
	                                         CSL_EDMA3_STATIC_DIS,
	                                         CSL_EDMA3_SYNC_AB,
	                                         CSL_EDMA3_ADDRMODE_INCR,
	                                         CSL_EDMA3_ADDRMODE_INCR);

	  myParamSetup.aCntbCnt    = CSL_EDMA3_CNT_MAKE(numPackedSamples*4, payloadSize/(numPackedSamples*4));
	  myParamSetup.dstAddr     = (uint32_t)UTILS_local2GlobalAddr(dstPktPtr);
	  myParamSetup.cCnt        = 1; // One symbol at a time
	  myParamSetup.linkBcntrld = CSL_EDMA3_LINKBCNTRLD_MAKE(0, 0);
	  myParamSetup.srcDstBidx  = CSL_EDMA3_BIDX_MAKE(numPackedSamples*4*numAxCs, numPackedSamples*4);
	  myParamSetup.srcDstCidx  = CSL_EDMA3_CIDX_MAKE(0, 0);
	  myParamSetup.srcAddr     = (uint32_t)UTILS_local2GlobalAddr(superPktAxCPtr);

	  status = CSL_edma3ParamSetup(hParamBasic, &myParamSetup);

	  if (CSL_FEXT(hEdma->regs->TPCC_IPR,TPCC_TPCC_IPR_IPR0) == 1) {
	  		  hEdma->regs->TPCC_ICR = 1;
	  }

	  //Clear channel event
	  status = CSL_edma3HwChannelControl(hEdmaDataChan, CSL_EDMA3_CMD_CHANNEL_CLEAR, NULL);

	  //trigger channel
	  status = CSL_edma3HwChannelControl(hEdmaDataChan, CSL_EDMA3_CMD_CHANNEL_SET, NULL);

	  //wait for completion
	  while(CSL_FEXT(hEdma->regs->TPCC_IPR,TPCC_TPCC_IPR_IPR0) != 1);

	  //status = CSL_edma3HwControl(hEdma,CSL_EDMA3_CMD_INTRPEND_CLEAR,&cmd);
	  hEdma->regs->TPCC_ICR = 1;

	  status = CSL_edma3ChannelClose(hEdmaDataChan);

	  if (hEdma != NULL) {CSL_edma3Close(hEdma);hEdma = NULL;}

}


CSL_Edma3ChannelObj    chObj0[7];
CSL_Edma3ChannelHandle hEdmaDataChan0[7];
CSL_Edma3ParamHandle   hParamBasic0[7];
CSL_Edma3Handle        hEdma0;
CSL_Edma3Obj           edmaObj0;
void
UTILS_openEdmaSampleCapt(
)
{

	CSL_Edma3ChannelAttr   chAttr;
	CSL_Status             status;
	uint32_t				   i;

	hEdma0            = CSL_edma3Open(&edmaObj0,CSL_TPCC_0,NULL,&status);

	for (i=0;i<7;i++)  //7 symbols captured every slot
	{
		// Channel open
		chAttr.regionNum = CSL_EDMA3_REGION_GLOBAL;
		chAttr.chaNum    = i;
		status           = 0;

		hEdmaDataChan0[i]    = CSL_edma3ChannelOpen(&chObj0[i], CSL_TPCC_0, &chAttr, &status);
		status               = CSL_edma3HwChannelSetupParam(hEdmaDataChan0[i], i);
		// Change Channel Default queue setup from 0 to 2
		status               = CSL_edma3HwChannelSetupQue(hEdmaDataChan0[i], CSL_EDMA3_QUE_0);
		hParamBasic0[i]      = CSL_edma3GetParamHandle(hEdmaDataChan0[i], i, &status);
	}

	// Set priority level on this queue
	CSL_edma3SetEventQueuePriority(hEdma0, CSL_EDMA3_QUE_0, CSL_EDMA3_QUE_PRI_4);

}

void
UTILS_triggerEdmaSampleCapt(
		uint32_t  symbolNum,
		uint32_t* dstAddr,
		uint32_t* srcAddr,
		uint32_t  numBytes
)
{
	CSL_Edma3ParamSetup    myParamSetup;

    // Setup the EDMA parameters
    if (symbolNum < 6) {
        myParamSetup.option = CSL_EDMA3_OPT_MAKE(CSL_EDMA3_ITCCH_DIS,
                                                 CSL_EDMA3_TCCH_EN,
                                                 CSL_EDMA3_ITCINT_DIS,
                                                 CSL_EDMA3_TCINT_DIS,
                                                 symbolNum+1,
                                                 CSL_EDMA3_TCC_NORMAL,
                                                 CSL_EDMA3_FIFOWIDTH_NONE,
                                                 CSL_EDMA3_STATIC_DIS,
                                                 CSL_EDMA3_SYNC_AB,
                                                 CSL_EDMA3_ADDRMODE_INCR,
                                                 CSL_EDMA3_ADDRMODE_INCR);
    } else {
        myParamSetup.option = CSL_EDMA3_OPT_MAKE(CSL_EDMA3_ITCCH_DIS,
                                                 CSL_EDMA3_TCCH_DIS,
                                                 CSL_EDMA3_ITCINT_DIS,
                                                 CSL_EDMA3_TCINT_EN,
                                                 symbolNum,
                                                 CSL_EDMA3_TCC_NORMAL,
                                                 CSL_EDMA3_FIFOWIDTH_NONE,
                                                 CSL_EDMA3_STATIC_DIS,
                                                 CSL_EDMA3_SYNC_AB,
                                                 CSL_EDMA3_ADDRMODE_INCR,
                                                 CSL_EDMA3_ADDRMODE_INCR);
    }


	myParamSetup.aCntbCnt    = CSL_EDMA3_CNT_MAKE(numBytes,1);
	myParamSetup.dstAddr     = (uint32_t)UTILS_local2GlobalAddr(dstAddr);
	myParamSetup.cCnt        = 1; // One symbol at a time
	myParamSetup.linkBcntrld = CSL_EDMA3_LINKBCNTRLD_MAKE(0, 0);
	myParamSetup.srcDstBidx  = CSL_EDMA3_BIDX_MAKE(0,0);
	myParamSetup.srcDstCidx  = CSL_EDMA3_CIDX_MAKE(0,0);
	myParamSetup.srcAddr     = (uint32_t)UTILS_local2GlobalAddr(srcAddr);

	CSL_edma3ParamSetup(hParamBasic0[symbolNum], &myParamSetup);

    if (symbolNum == 6)
    {
        /*if (CSL_FEXT(hEdma0->regs->TPCC_IPR,TPCC_TPCC_IPR_IPR0) == 1) {
                  hEdma0->regs->TPCC_ICR = 1;
        }*/
        if (((hEdma0->regs->TPCC_IPR >> symbolNum)&1) == 1) {
            hEdma0->regs->TPCC_ICR = (1 << symbolNum);    //FIXME, it was set to 1
        }
        //Clear channel event
        CSL_edma3HwChannelControl(hEdmaDataChan0[0], CSL_EDMA3_CMD_CHANNEL_CLEAR, NULL);

        //trigger channel
        CSL_edma3HwChannelControl(hEdmaDataChan0[0], CSL_EDMA3_CMD_CHANNEL_SET, NULL);
	}

    //wait for completion
    //while(CSL_FEXT(hEdma->regs->TPCC_IPR,TPCC_TPCC_IPR_IPR0) != 1);

    //status = CSL_edma3HwControl(hEdma,CSL_EDMA3_CMD_INTRPEND_CLEAR,&cmd);
    //hEdma0->regs->TPCC_ICR = 1;

    //status = CSL_edma3ChannelClose(hEdmaDataChan);

    //if (hEdma != NULL) {CSL_edma3Close(hEdma);hEdma = NULL;}

}

#endif // TMS320C6X

#ifdef _VIRTUAL_ADDR_SUPPORT
static int dev_mem_fd;
#endif

uint32_t aif2_mmap(uint32_t addr, uint32_t size)
{
#ifdef _VIRTUAL_ADDR_SUPPORT
        uint32_t virt_addr;
        uint32_t page_size;
        page_size = sysconf(_SC_PAGE_SIZE);
        if (size%page_size)
        {
#ifndef USESYSBIOS
                printf("Size does not align with page size. Size given: %d\n", size);
#else
                System_printf("Size does not align with page size. Size given: %d\n", size);
#endif
                return 0;
        }
        if ((uint32_t)addr % page_size)
        {
#ifndef USESYSBIOS
                printf("Address does not align with page size. Address given: 0x%08x\n", (uint32_t) addr);
#else
                System_printf("Address does not align with page size. Address given: 0x%08x\n", (uint32_t) addr);
#endif
                return 0;
        }
        virt_addr = (uint32_t) mmap(0, size, (PROT_READ|PROT_WRITE), MAP_SHARED, dev_mem_fd, (off_t)addr);
        if (virt_addr == -1)
        {
#ifndef USESYSBIOS
                printf("mmap failed!\n");
#else
                System_printf("mmap failed!\n");
#endif
                return 0;
        }
        return virt_addr;
#else
    return addr;
#endif
}

uint32_t aif2_mapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    if ((dev_mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1)
    {
#ifndef USESYSBIOS
        printf("Failed to open /dev/mem \n");
#else
        System_printf("Failed to open /dev/mem \n");
#endif
        return -1;
    }
    aif2InitCfg.dev.bases[0].cfgBase = (void *) aif2_mmap((uint32_t)(aif2InitCfg.dev.bases[0].cfgBase), 0x80000);
    aif2InitCfg.dev.bases[0].serDesB4CfgBase = (void *) aif2_mmap((uint32_t)(aif2InitCfg.dev.bases[0].serDesB4CfgBase), 0x2000);
    aif2InitCfg.dev.bases[0].serDesB8CfgBase = (void *) aif2_mmap((uint32_t)(aif2InitCfg.dev.bases[0].serDesB8CfgBase), 0x2000);
    return 0;
#else
    return 0;
#endif
}

uint32_t aif2_unmapDevice()
{
#ifdef _VIRTUAL_ADDR_SUPPORT
    munmap(aif2InitCfg.dev.bases[0].cfgBase, 0x80000);
    munmap(aif2InitCfg.dev.bases[0].serDesB4CfgBase, 0x2000);
    munmap(aif2InitCfg.dev.bases[0].serDesB8CfgBase, 0x2000);
    close(dev_mem_fd);
#endif
    return 0;
}


////////////////////
