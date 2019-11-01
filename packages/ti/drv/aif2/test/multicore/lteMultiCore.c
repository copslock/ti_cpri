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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include <ti/ipc/Ipc.h>
#include <ti/ipc/HeapBufMP.h>
#include <ti/ipc/MultiProc.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#include <ti/csl/cslr_cppidma_rx_channel_config.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/rm/rm.h>


#if EVM_TYPE == 3 || EVM_TYPE == 4
#include <ti/platform/evmc6670l/platform_lib/include/evmc66x_pllc.h>
#endif


//include for GPIO
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>

#include "cslUtils.h"
#include "mnavUtils.h"
#include "lte_process.h"


/*
 * This is an AIF2 Single Tone example test for a LTE 20MHz configuration.
 * In this test, the Symbol IQ samples are populated with a sine or cosine wave.
 * The signal is continuous across the 14 symbols of an AxC subframe.
 * This test can be run as a sanity check for the LTE RF DL chain.
 * The test is configured to work at the granularity of a slot.
 * For LTE, AIF2 requires usage of monolithic descriptor packets with a 16-byte header.
 * In the header, a protocol specific word is required to indicate the antenna carrier
 * number, the symbol number and the direction (Ingress/Egress)
 * That means that every 0.5 ms, the test will push and pop from the AIF2 HW queues:
 * 7 symbols * numAxCs per link * numLinks * 2 (for Egress/Ingress traffic)
 * This then makes a total of 56 outstanding monolithic packets in the case of
 * LTE 20MHz / 2 AxCs / 1 Links / both directions
 * When defining a descriptor memory region, the following rules apply:
 * Descriptor size is multiple of 16 bytes, min 32.
 * Descriptor count is a power of 2, minimum 25.
 * Memory region base address must be aligned to a 16-byte address boundary.
 * Here are some considerations regarding PktDMA traffic:
 * 		- For Egress, it is up to the user to insure that DMA data is available prior
 * 		to beginning PE message construction (PE2 event). Breaking real time on DMA has
 * 		similar effects to breaking CorePac MIPS real time. Specifically, AIF2 will fail
 * 		each affected AxC until the beginning of the next radio frame boundary.
 * 		- For Ingress, it is important that DMA is efficient enough that the AIF2 input
 * 		buffers never overflow.
 */

#define     TRIG_TIMER          7
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define 	SYNC_TIMER			8
#else
#define     SYNC_TIMER          5
#endif
#define     PLL_TIMER           4

//Enable control words
#define     CPRI_FAST_CM        0
#define     FASTCM              1

#define		NUMCORE				AIF_CFG_MAX_CORE

#if	FASTCM == 1
#define		CPRI_FAST_CM_SIZE   1408  // every hyperframe or 256 basic frames (BFs), 176 BFs can be used for
									 // fast C&M. These are BF[20:63], BF[84:127], BF[148:191], BF[212:255]
									 // in 4x rate, word size is 32 bits. So 176*4 = CPRI_FAST_CM_SIZE bytes.
#define		ETH_FRAME_LEN		(7+1+12+2+80+4)
#define		TX_FAST_CM_PKT_NUM	1
#define		RX_FAST_CM_PKT_NUM	2
uint32_t      ethPkt[(ETH_FRAME_LEN+4)/4];
#endif
#if	FASTCM
#define     PPOINTER			20
#endif

#if EVM_TYPE == 3 || EVM_TYPE == 4
#define     LOOPBACK
#define     NEVER_END_TEST      1     //Enable the infinite loop for SCBP test with spectrum analyzer
#else
#define     NEVER_END_TEST      0     //Goes thru packet checking after a certain number of slots
#endif

//#define     TEST_NUM            2
#define 	_AIF2FL_DUMP		0     //Enable the DUMP of the AIF configuration
														// For debug, size of monitoring arrays

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM, the Frequency is based on an external 153.60 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define     PLLC_PREDIV_CLK   	1       // PLLC_PREDIV_CLK - 1 set to register
#define     PLLC_PLLM_CLK     	8       // PLLC_PLLM_CLK - 1 set to register (983MHz CPU clock)

// DIO engines are not used in LTE mode - instead PKTDMA and HW queues are used to carry the antenna data from/to AIF2
#define     DIO_0               AIF2FL_DIO_ENGINE_0

volatile unsigned int verbose       = 1;
volatile unsigned int NB_ITERATIONS = 1;
volatile unsigned int ntest         = 0;
volatile unsigned int testcheck     = 1;
#ifdef LOOPBACK
volatile unsigned int intLoopback   = 1;
volatile unsigned int swSync        = 1;
#else
volatile unsigned int intLoopback   = 0;
volatile unsigned int swSync        = 0;
#endif

//////////////////////
// Global variables
#pragma DATA_ALIGN   (aifObj, 128)
#pragma DATA_SECTION (aifObj, ".aif2");
AIF_ConfigObj               aifObj;
#pragma DATA_ALIGN   (hConfigAif, 128)
#pragma DATA_SECTION (hConfigAif, ".aif2");
AIF_ConfigHandle            hConfigAif = &aifObj;

/* Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index */
#define MONO_RX_FDQ            2024
#define MONO_RX_Q              1024
#define MONO_RX_FDQ_CW         2048
#define MONO_RX_Q_CW           1048

/* Monolithic descriptor region for all outstanding symbols in a subframe (1ms)
 * Region base address is 16-byte aligned. (aligning to a L1D 64-byte cache line boundary)
 * The linker will place this >1MBytes buffer in internal shared memory (MSM)
 * symbolcount size is 8.8K bytes for Normal cyclic prefix 20 MHz LTE
 */


#if DESCMSM == 1
#pragma DATA_SECTION(mono_region_test,".aifdescmsm");
#pragma DATA_SECTION(rxBuffer,".aifdescmsm");
#else
#pragma DATA_SECTION(mono_region_test,".aifdescddr");
#pragma DATA_SECTION(rxBuffer,".aifdescddr");
#endif
#pragma DATA_ALIGN (mono_region_test, 64);
uint8_t   mono_region_test[NBDESCMAX * LTESYMBOLSIZE * NBCHANNELINIT];


uint8_t   rxBuffer[NBCHANNELPERLINK][LTESYMBOLSIZE-16];

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Dummy variables for debug */
uint32_t nbchanneltotal   = 0; // Default for 1 link LTE 20 MHz
uint32_t firstChan		= 0;

/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS*NBCHANNELMAXPERLINK];
/* Define Rx flow for control packets */
Cppi_RxFlowCfg rxFlowCpriCw[AIF2_CPRI_MAX_CW_SUBSTREAM];


volatile uint32_t activeLink;

extern uint8_t		keepAifOff;

extern uint32_t		numcore2sync;

#pragma DATA_ALIGN   (synchronize, 128)
#pragma DATA_SECTION (synchronize, ".appSyncSharedMem");  //.appSyncSharedMem
uint32_t synchronize = 0;

uint32_t              *ctrlCw, ctrlCwLen, entries, txPktCountCM, txPktCountVendor, rxPktCount, rxPktCountVendor, rxErrorLengh, rxErrorLenghVendor, dataerror, dataerrorvendor, nberror, nberrorVendor, errorcount, errorcountVendor;

/*
 * This flag is used for enabling and printing out exceptions at the end of the test
 */
uint32_t printexceptions = 1;
#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

Swi_Handle slotRxSwi;
Swi_Handle slotTxSwi;
Swi_Handle slotCntSwi;

void slotRecyclingSwi();
void slotPushingSwi();

void appWritebackBuffer(void* ptr, uint32_t size)
{
	CACHE_wbL1d (ptr, size, CACHE_WAIT);
}

void appInvalidateBuffer(void* ptr, uint32_t size)
{
	CACHE_invL1d (ptr, size, CACHE_WAIT);
}


/*
 * User Isr that is called from pre-defined Isr in cslUtils.c
 */
void slotIsr()
{
		Swi_post(slotRxSwi);
		Swi_post(slotTxSwi);
		if(DNUM==0)
		{
			getCpuTimestamp[slotcount] 	   = TSCL;
			getPhyTimerFrames[slotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
			getPhyTimerClk[slotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

			if (slotcount == (SLOT_NUM_FIRST_PUSH-1)) {
				AIF_enableException(&aifObj);
			}

			if (slotcount == (SLOT_NUM_DATA_CHECK-1)) UTILS_aif2ExceptIntDisable();

			aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
			aif2SymbolEgressCount[slotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
		}
}

void myAifTest();
void myAifLLDInit();

/* Startup function */
void dspProcIdConfig(void) {
	uint16_t myboard	= EVM_TYPE;

    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
    else if (myboard <= 2)  DSP_procId = EVM_TYPE; // For Advantech: myboard  = 1 (DSP_1) or 2 (DSP_2)
    else if (EVM_TYPE == 6) DSP_procId = 1;
    else DSP_procId = (uint8_t)(EVM_TYPE - 2);       // For SCBP: myboard  = 3 (DSP_1) or 4  (DSP_2)

}

/* Aif Configuration */
extern AIF_CfgObj    gAifCfgObj;       // Shared object
extern AIF_CfgObj    gAifCfgLocalObj;  // Core0 local configuration used to init the shared configuration object

/* Core0 Resource Manager Handle */
AIF_CoreCfgObj*      ptrAifCoreCfg;
AIF_PktDmaConfigObj  pktDmaObj;


void main(void)
{
	//create a task
    Task_Params  tskParams;
    int32_t        status;

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

	if (DNUM == 0)
	{
		// initialization function for qmss and cppi low-level drivers
		UTILS_initQmss((uint32_t*)mono_region_test, NBDESCMAX, LTESYMBOLSIZE * NBCHANNELINIT, 0, NULL);
	}

	/* Call Ipc_start() */
	status = Ipc_start();
	if (status < 0) {
	    System_abort("Ipc_start failed\n");
	}

    if (DNUM != 0) {

		Task_Params_init(&tskParams);
		tskParams.stackSize =0x10000;

		Task_create(myAifTest, &tskParams, NULL);

		//create a Swi that will be trigger by the slot Hwi to procces the recycling and pushing of packet at a slot time base.
		Swi_Params swiParams;


		Swi_Params_init (&swiParams);
		swiParams.arg0 = 0;
		swiParams.arg1 = 0;
		swiParams.priority = 2;
		swiParams.trigger = 0;
		slotRxSwi = Swi_create(slotRecyclingSwi, &swiParams, NULL);

		Swi_Params_init (&swiParams);
		swiParams.arg0 = 0;
		swiParams.arg1 = 0;
		swiParams.priority = 1;
		swiParams.trigger = 0;
		slotTxSwi = Swi_create(slotPushingSwi, &swiParams, NULL);
    }

    if (DNUM == 0) {
		Task_Params_init(&tskParams);
		tskParams.stackSize =0x10000;

		Task_create(myAifLLDInit, &tskParams, NULL);

		//create a Swi that will be trigger by the slot Hwi to procces the recycling and pushing of packet at a slot time base.
		Swi_Params swiParams;

		Swi_Params_init (&swiParams);
		swiParams.arg0 = 0;
		swiParams.arg1 = 0;
		swiParams.priority = 2;
		swiParams.trigger = 0;
		slotRxSwi = Swi_create(slotRecyclingSwi, &swiParams, NULL);

		Swi_Params_init (&swiParams);
		swiParams.arg0 = 0;
		swiParams.arg1 = 0;
		swiParams.priority = 1;
		swiParams.trigger = 0;
		slotTxSwi = Swi_create(slotPushingSwi, &swiParams, NULL);

    }

    BIOS_start();

}

//Swi dedicated for packet recycling
void slotRecyclingSwi()
{
	slotRecycling(&pktDmaObj, &rxBuffer[0], nbchanneltotal, firstChan, activeLink);
}

//Swi dedicated for packet pushing
void slotPushingSwi()
{
	slotPushing(&pktDmaObj, nbchanneltotal, firstChan, &firstSample[0], &symbolPkt[0]);
}

void myAifLLDInit()
{
	uint32_t i;
	numcore2sync = NUMCORE;

    System_printf("Beginning AIF2 LTE RF tests:\n");
    UTILS_waitForHw(100000);
	
	// Take AIF out of power saver
	AIF_enable();

	UTILS_initTimer(TRIG_TIMER);

	aif2evt5_userIsr = slotIsr;

	// Init the shared configuration object for AIF2 LLD
    memcpy(&gAifCfgObj,&gAifCfgLocalObj,sizeof(AIF_CfgObj));

	//process for DSP 0
	ptrAifCoreCfg = AIF_getCfg(&gAifCfgObj,AIF_CFG_MAX_CORE,DNUM);

	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		if (ptrAifCoreCfg->linkEnable[i] == 1)
			activeLink = i;
	}
	Utils_populatePktDmaCfgObj(ptrAifCoreCfg, &pktDmaObj, mono_region_test, LTESYMBOLSIZE, NBSYMBOL, MONO_RX_Q, MONO_RX_FDQ, rxFlow);
	firstChan = ptrAifCoreCfg->firstAxCIndex[activeLink];
	nbchanneltotal = ptrAifCoreCfg->numAxC[activeLink];


	// initialization function for the AT timer and EDMA3 ISRs
	UTILS_aifIntcSetup();

	// initialization of Pktdma and Qmss resources given this user configuration
	UTILS_initPktDma(&pktDmaObj);
	UTILS_initCppiChannel(&gAifCfgObj, &pktDmaObj);
	/*
	 * Pre-load with one packet prior to Aif start
	 */
	if (nbchanneltotal != 0)
		load_data(&pktDmaObj, nbchanneltotal, ptrAifCoreCfg->firstAxCIndex[activeLink], &firstSample[0], &symbolPkt[0]);

	AIF_populateAifObj(&aifObj, &gAifCfgObj, SYSCLK_INPUT_KHZ, swSync, intLoopback, LTE_RATE);

	// Compute default AIF2 parameters given this user configuration
	AIF_calcParameters(&aifObj);

	if (DSP_procId == 1)
		UTILS_printLinkConfig(&aifObj);

	// initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
	AIF_initHw(&aifObj);

#if _AIF2FL_DUMP == 1
	if (DSP_procId == 1)
	{
	   FILE *fout;
	   fout = fopen("aif2fl_dump.txt","w");
	   if (fout)
	   {
		   dump_Aif2Fl_Setup(fout,aifObj.hAif2Setup);
		   fclose(fout);
	   }

	}
#endif

	appInvalidateBuffer(&synchronize, 128);
	synchronize = 3;
	appWritebackBuffer(&synchronize, 128);

	// start AIF2 HW and PktDMA and then wait for frame synchronization
	AIF_startHw(&aifObj);

	if (DSP_procId == 1)
		UTILS_triggerFsync(&aifObj);

	while(1)
	{
		Task_yield();

#if NEVER_END_TEST == 0
		// wait for one extra slots to run out of Rx free pkts
		if(slotcount == (SLOT_NUM_DATA_CHECK + 2))
		{
			//AT disable all events and halt timer
			CSR&= 0xFFFFFFFE;
			//keepAifOff = 1;
			UTILS_doCleanup(&aifObj,TRIG_TIMER);
			break;
		}
#endif
	}

	if (nbchanneltotal != 0)
		lteFinalCheck(&pktDmaObj, &rxBuffer[0], nbchanneltotal, ptrAifCoreCfg->firstAxCIndex[activeLink], activeLink);

	if (printexceptions == 1)
	{
		AIF_printException(&aifObj);
	}

    if (testcheck == 1) {
           testcheck = 0;
           System_printf("All tests have passed\n");
    } else {
           System_printf("Some tests have failed\n");
    }

    System_abort("End of Program!!\n");
}

void myAifTest()
{

    uint32_t	 i;



    numcore2sync = NUMCORE;
	// AIF2 LLD programs AT event 5 with period 0.5ms in case of LTE
	aif2evt5_userIsr = slotIsr;

	while (synchronize != 3)
	{
		appInvalidateBuffer(&synchronize, 128);
	}


	UTILS_initQmss((uint32_t*)mono_region_test, NBDESCMAX, LTESYMBOLSIZE * NBCHANNELINIT, 0, NULL);

	ptrAifCoreCfg = AIF_getCfg(&gAifCfgObj,AIF_CFG_MAX_CORE,DNUM);

	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		if (ptrAifCoreCfg->linkEnable[i] == 1)
			activeLink = i;
	}


	Utils_populatePktDmaCfgObj(ptrAifCoreCfg, &pktDmaObj,mono_region_test, LTESYMBOLSIZE, NBSYMBOL, MONO_RX_Q, MONO_RX_FDQ, rxFlow);
	firstChan = ptrAifCoreCfg->firstAxCIndex[activeLink];
	nbchanneltotal = ptrAifCoreCfg->numAxC[activeLink];

	// initialization function for the AT timer and EDMA3 ISRs
	UTILS_aifIntcSetup();

	// initialization of Pktdma and Qmss resources given this user configuration
	UTILS_initPktDma(&pktDmaObj);

	if(nbchanneltotal != 0)
		load_data(&pktDmaObj, nbchanneltotal, ptrAifCoreCfg->firstAxCIndex[activeLink], &firstSample[0], &symbolPkt[0]);

	AIF_startCfg();

	while(1)
	{
		Task_yield();

#if NEVER_END_TEST == 0
		// wait for one extra slots to run out of Rx free pkts
		if(slotcount == (SLOT_NUM_DATA_CHECK + 2))
		{
			//AT disable all events and halt timer
			CSR&= 0xFFFFFFFE;
			break;
		}
#endif
	}

	if(nbchanneltotal != 0)
		lteFinalCheck(&pktDmaObj, &rxBuffer[0], nbchanneltotal, ptrAifCoreCfg->firstAxCIndex[activeLink], activeLink);

    if (testcheck == 1) {
           testcheck = 0;
           System_printf("All tests have passed\n");
    } else {
           System_printf("Some tests have failed\n");
    }

	System_abort("End of Program!!\n");
}

////////////

