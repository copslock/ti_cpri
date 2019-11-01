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
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

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
#if EVM_TYPE == 3 || EVM_TYPE == 4
#include <ti/platform/evmc6670l/platform_lib/include/evmc66x_pllc.h>
#endif

//include for GPIO
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>


#if EVM_TYPE == 0
#include <EVM.h>
#endif

#include "cslUtils.h"
#include "mnavUtils.h"

#if EVM_TYPE == 5
#include "appletonScbpSync.h"
#endif


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
#define     SYNC_TIMER          5
#define     PLL_TIMER           4

//Value for Sin calculation
#define     Q_FACTOR            8191
#if LTE_RATE == 20
#define     SAMP_FREQ_FLOAT     30.72
#elif LTE_RATE == 10
#define     SAMP_FREQ_FLOAT     15.36
#elif LTE_RATE == 5
#define     SAMP_FREQ_FLOAT     7.68
#endif
#define     PI                  3.14159265358979323846

//define the tone mode, only one of the three patterns can be enable at a time
#define     SINGLE_TONE         1
#define     DUAL_TONE_ONE_TWO   0
#define     DUAL_TONE_FIVE_NINE 0
//Enable control words
#define     CPRI_FAST_CM        0

#if EVM_TYPE == 3 || EVM_TYPE == 4
#define     NEVER_END_TEST      1     //Enable the infinite loop for SCBP test with spectrum analyzer
#else
#define     NEVER_END_TEST      0     //Goes thru packet checking after a certain number of slots
#endif

#define     TEST_NUM            2
#define     TEST_ALL            0
#define 	_AIF2FL_DUMP		0     //Enable the DUMP of the AIF configuration
#define     NBCHANNELPERLINK	2
#if LTE_RATE == 20
#define     NBCHANNELMAXPERLINK	2
#elif LTE_RATE == 10
#define     NBCHANNELMAXPERLINK	4
#elif LTE_RATE == 5
#define     NBCHANNELMAXPERLINK	8
#endif
#define     NBCHANNELMAX		(NBCHANNELPERLINK*2) 		                                    // 2-link max for this test
#define     NBSYMBOL			AIF2_LTE_SYMBOL_NUM											// Number of symbols per slot
#if LTE_RATE == 20
#define		LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#elif LTE_RATE == 10
#define		LTESYMBOLSIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)    
#define 	LTESYMBOL2SIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL_SIZE)*4 + 16)     
#elif LTE_RATE == 5
#define		LTESYMBOLSIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)    
#define 	LTESYMBOL2SIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL_SIZE)*4 + 16)     
#endif

#ifdef SUPERPACKET
#define		NBCHANNELINIT		NBCHANNELMAXPERLINK
#else
#define		NBCHANNELINIT		1
#endif

#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6
#define     DESCMSM				1															// Lyrtech has no stable DDR, so using MSM and reduing the number of mono desc
#define		NBDESCMAX			64                                                         // Descriptor count rounded to the next power of 2
																							// 7 sym * 2 AxCs * 2 pingpong * 2 directions + 16 control packets
#else
#define     DESCMSM				0															// mono desc region going to DDR
#define		NBDESCMAX			128                                                         // Descriptor count rounded to the next power of 2
																							// 14 sym * 2 AxCs * 2 pingpong * 2 directions * 2 links
#endif
#define		MAX_SLOT				400															// For debug, size of monitoring arrays

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

volatile unsigned char testEnable[TEST_NUM] =
    {  // Each of those need to be run individually
        1,     // CPRI 4x for LTE 20Mhz - Link 0
        0      // CPRI 4x for LTE 20Mhz - Link 2
    };

typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;

typedef struct {
        // Test name
        const unsigned char name[32];
        //AIF link enable or disable, 1 or 0
        uint32_t                   linkEnable[AIF_MAX_NUM_LINKS];
        //AIF link rate (1=1x; 2=2x; 4= 4x).
        uint32_t                   linkRate[AIF_MAX_NUM_LINKS];
        //AIF link data type for outbound burst traffic.
        Aif2Fl_LinkDataType     outboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for outbound burst traffic.
        Aif2Fl_DataWidth        outboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link data type for inbound burst traffic.
        Aif2Fl_LinkDataType     inboundDataType[AIF_MAX_NUM_LINKS];
        //AIF link data width for inbound burst traffic.
        Aif2Fl_DataWidth        inboundDataWidth[AIF_MAX_NUM_LINKS];
        //AIF link DIO engine used
        Aif2Fl_DioEngineIndex   dioEngine[AIF_MAX_NUM_LINKS];
        } TestObj;


//////////////////////
// Global variables
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st Test - CPRI 4x for LTE 20MHz
	  "LTE_20MHZ_RF_SINGLE_TONE", // test name
	 // link0          link1          link2          link3          link4          link5
	  {1,             0,             0,             0,             0,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_16},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
	},
	{//1st Test - CPRI 4x for DL LTE 20Mhz - Link
      "CPRI_4X_LTE_1L", // test name
   // link0          link1          link2          link3          link4          link5
     {0,             0,             1,             0,             0,             0            },  // link enable
     {4,             4,             4,             4,             4,             4            },  // link rate
     {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
     {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
     {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
     {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	 {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
    }
   };


/* Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index */
#define MONO_RX_FDQ            2012
#define MONO_RX_Q              1000

/* Monolithic descriptor region for all outstanding symbols in a subframe (1ms)
 * Region base address is 16-byte aligned. (aligning to a L1D 64-byte cache line boundary)
 * The linker will place this >1MBytes buffer in internal shared memory (MSM)
 * symbolcount size is 8.8K bytes for Normal cyclic prefix 20 MHz LTE
 */
#if DESCMSM == 1
#pragma DATA_SECTION(mono_region_test,".aifdescmsm");
#else
#pragma DATA_SECTION(mono_region_test,".aifdescddr");
#endif
#pragma DATA_ALIGN (mono_region_test, 64);
#ifdef SUPERPACKET
uint8_t   mono_region_test[NBDESCMAX * LTESYMBOLSIZE * NBCHANNELINIT];
#else
uint8_t   mono_region_test[NBDESCMAX * LTESYMBOLSIZE];
#endif
uint8_t   rxBuffer[NBCHANNELPERLINK][LTESYMBOLSIZE-16];

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Dummy variables for debug */
uint32_t nblink            = 1; 				 // default at one but set depending on test
uint32_t nbchanneltotal    = NBCHANNELPERLINK; // Default for 1 link LTE 20 MHz
#ifdef SUPERPACKET
uint32_t nbchannelSuperPacket = 1;             // Default is 1 AxC for super packet
#else
uint32_t nbchannelSuperPacket = NBCHANNELPERLINK;
#endif
/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS][NBCHANNELMAXPERLINK];
/* Define Rx flow for control packets */
Cppi_RxFlowCfg rxFlowCpriCw;

/*
 * Slot and symbolcount counters
 */
volatile uint32_t slotcount = 0;
volatile uint32_t symbolcount = 0;
volatile uint32_t packets_already_pushed = 0;
volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
volatile uint32_t getCpuTimestamp[MAX_SLOT];
volatile uint32_t getPhyTimerFrames[MAX_SLOT];
volatile uint32_t getPhyTimerClk[MAX_SLOT];
volatile uint32_t getRxPktCnt[MAX_SLOT]; // for first channel
volatile uint32_t rxPktCnt[NBCHANNELPERLINK * 2]; // assuming 2 link max
volatile uint32_t symbolNum[MAX_SLOT][7];
unsigned char   saveDesc[2][14][32];
volatile uint32_t swiCount = 0;
volatile uint32_t activeLink;

extern uint8_t		   keepAifOff;

/*
 * This flag is used for enabling and printing out exceptions at the end of the test
 */
uint32_t printexceptions = 1;
#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

#define SLOT_NUM_FIRST_PUSH	60  // The first push needs to happen on the last subframe before frame boundary
#define SLOT_NUM_DATA_CHECK	130 // Frame boundary when subframecount = 31, so data can be checked on next subframe so 32

#if EVM_TYPE == 4
// Fonction that are used to set a 1 Hz pps signal in order to sync with appleton board
extern void routeTimerLowOutToTimerOut0Pin(void);
extern int ppsGen (void);
#endif

Swi_Handle slotRxSwi;
Swi_Handle slotTxSwi;

void slotRecyclingSwi();
void slotPushingSwi();

/*
 * User Isr that is called from pre-defined Isr in cslUtils.c
 */
void slotIsr()
{
	Swi_post(slotRxSwi);
	Swi_post(slotTxSwi);
}

void getcomplex(Complex16* payload,uint32_t index, uint32_t tx)
{
	float freq;
	Complex16 x1;
	Complex16 y;
		if (tx == 0)
		{
			// 1 MHz signal
                freq = 1;
                x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                y = x1;
		} else if (tx == 1){

                Complex16         x2;
                // 1 MHz signal
                freq = 1;
                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // 2 MHz signal
                freq = 2;
                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // dual tone signal
                y.re = x1.re + x2.re;
                y.im = x1.im + x2.im;
		}
#if DUAL_TONE_FIVE_NINE == 1
                Complex16         x2;
                // 5 MHz signal
                freq = 5;
                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // 9 MHz signal
                freq = 9;
                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // dual tone signal
                y.re = x1.re + x2.re;
                y.im = x1.im + x2.im;
#endif

                payload->re = y.re;
                payload->im = y.im;
}

void myAifTest();

/* Startup function */
void dspProcIdConfig(void) {
	uint16_t myboard	= EVM_TYPE;
#if EVM_TYPE == 3 || EVM_TYPE == 4
    PllcHwSetup pllc_hwSetup;
#endif

    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
    else if (myboard <= 2)  DSP_procId = EVM_TYPE; // For Advantech: myboard  = 1 (DSP_1) or 2 (DSP_2)
    else DSP_procId = (uint8_t)(EVM_TYPE - 2);       // For SCBP: myboard  = 3 (DSP_1) or 4  (DSP_2)
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    DSP_procId = 1;
#endif

#if EVM_TYPE == 3 || EVM_TYPE == 4
    /* Clear local data structures */
	memset(&pllc_hwSetup, 0, sizeof(PllcHwSetup));

	/* Setup PLLC hardware parameters */
	pllc_hwSetup.divEnable  = (CSL_BitMask32) (PLLC_DIVEN_PLLDIV2 |
											   PLLC_DIVEN_PLLDIV5 |
											   PLLC_DIVEN_PLLDIV8) ;

	/* Setup PLLC hardware parameters */
	pllc_hwSetup.pllM       = 16 -1;
	pllc_hwSetup.preDiv   = 1 - 1;
	pllc_hwSetup.pllDiv2  = 3 - 1;
	pllc_hwSetup.pllDiv5  = 5 - 1;
	pllc_hwSetup.pllDiv8  = 64 - 1;
	pllc_hwSetup.postDiv  = 2 -1;

	/* set Pll */
	CorePllcHwSetup(&pllc_hwSetup);
#endif
#if EVM_TYPE == 3 // SCBP standalone test
	swSync = 1;
#endif
}

void main(void)
{
	//create a task
    Task_Params  tskParams;

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


    BIOS_start();

}

//Swi dedicated for packet recycling
void slotRecyclingSwi()
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;

	slotcount++;
	getCpuTimestamp[slotcount] 	   = TSCL;
	getPhyTimerFrames[slotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
	getPhyTimerClk[slotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

#if NEVER_END_TEST == 0
	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */
	if (slotcount < SLOT_NUM_DATA_CHECK){
#endif
	for(chan = 0; chan < nbchannelSuperPacket; chan++)
	{
		rxSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);

		for(i=0;i<rxSymCnt;i++)
		{
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
			// Invalidate symbol packet header and get the updated payload address + length
			CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);

			if((rxPktCnt[chan]%NBSYMBOL) == 0)
			{
				 //check payload length for first symbol

				if (payloadLen != ((LTESYMBOLSIZE-16)*NBCHANNELINIT)) System_printf("unexpected payload length:%d, %d\n",((LTESYMBOLSIZE-16)*NBCHANNELINIT),payloadLen);
#ifdef SUPERPACKET
				//CACHE_invL1d((void *)rxBuffer[0], (LTESYMBOLSIZE-16), CACHE_WAIT);
                UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr, (uint32_t *)rxBuffer[0], \
                                         (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
                //CACHE_invL1d((void *)rxBuffer[1], (LTESYMBOLSIZE-16), CACHE_WAIT);
				UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr + aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
										 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
#else
				CACHE_invL1d((void *)payloadPtr, (LTESYMBOLSIZE-16), CACHE_WAIT);
				memcpy(&rxBuffer[chan][0],(uint32_t *) payloadPtr, (LTESYMBOLSIZE-16));										 
#endif
			} else {
				//check payload length for the other 6 symbols
				if (payloadLen != ((LTESYMBOL2SIZE-16)*NBCHANNELINIT)) System_printf("unexpected payload length:%d, %d\n",((LTESYMBOL2SIZE-16)*NBCHANNELINIT),payloadLen);
			}

			CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
			Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
			symbolNum [slotcount][i] = ((payloadPtr[0] >> 7) & 0xFF);//add symbol number into PS field
			CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM

			// recycle symbol packet on rx free queue
			Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
			rxPktCnt[chan]++;
		}
	}
#if NEVER_END_TEST == 0
	}
#endif

	getRxPktCnt[slotcount]      = rxPktCnt[0];
	aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
}

//Swi dedicated for packet pushing
void slotPushingSwi()
{
	int32_t                i,chan;
	uint32_t               payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
	Complex16			*checkPtr;
	/*
	 * In this Isr, at this slot, push all s7 ymbols for all AxCs, links at once
	 * It assumes that all symbolPkt were first popped from the FDQs, and properly populated.
	 * AIF2 will returned them in FDQs following the actual transmission via AIF2 protocol encoder.
	 * Note that AIF2 Rx PktDMA channels will starve for data checking since we don't perform
	 * Rx packet recycling at runtime. We don't keep pushing packets beyond a certain slot.
	 */
	if (!packets_already_pushed) {
		/* The first two slots we push the packets that have been initialized and are stored in symbolPkt[][]  */
		if((slotcount == SLOT_NUM_FIRST_PUSH) || (slotcount == (SLOT_NUM_FIRST_PUSH+1)))
		{
			for(i=symbolcount;i<(symbolcount + NBSYMBOL);i++)
			{
				for(chan = 0; chan < nbchanneltotal; chan++)
				{
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)symbolPkt[chan][i], (uint8_t**)&payloadPtr, &payloadLen);
					checkPtr = (Complex16*)payloadPtr;
					if ((checkPtr->re != firstSample[chan][i].re) || (checkPtr->im != firstSample[chan][i].im)) {
						System_printf("unexpected first sample in this packet\n");
					}
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)symbolPkt[chan][i]);
				}
			}
			symbolcount += NBSYMBOL;
			if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
			if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
				packets_already_pushed = 1;
			}
		}
	} else {
#if NEVER_END_TEST == 0
		if((slotcount > (SLOT_NUM_FIRST_PUSH+1)) && (slotcount < SLOT_NUM_DATA_CHECK)){
#endif
		for(i=symbolcount;i<(symbolcount + NBSYMBOL);i++)
		{
			for(chan = 0; chan < nbchanneltotal; chan++)
			{
				// pop from AxC Tx free descriptor queue
				ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
				// Recreate PS data
				CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
				Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
				payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
				CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
				checkPtr = (Complex16*)payloadPtr;
				if ((checkPtr->re != firstSample[chan][i%(2*NBSYMBOL)].re) || (checkPtr->im != firstSample[chan][i%(2*NBSYMBOL)].im)) {
					System_printf("unexpected first sample in this packet\n");
				}
				memcpy(&saveDesc[chan][i%(2*NBSYMBOL)][0],ptrMonoDesc,32);
				Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)ptrMonoDesc);
			}
		}
		symbolcount += NBSYMBOL;
		if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
#if NEVER_END_TEST == 0
		}
#endif
	}
	/* PktDMA activity monitoring */
	aif2SymbolEgressCount[slotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
	// Make sure no overflow occurs on the monitoring arrays
	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
}

void myAifTest()
{

    uint32_t               idx, idx1, idx2,idx3 , i, payloadLen;
    uint16_t               testpass = 0;
	uint32_t               monoRxCount, monoTxCount;
	Complex16            *payloadPtr;
	Complex16			 sampleCheck;

	uint32_t               chan, rx_count, value;
    Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Qmss_Queue           queueInfo;
    Cppi_DescTag 		 descTag;
    uint32_t			     gie, ctrlPktCount, frameCount, entries;
    Cppi_MonolithicDesc *ptrMonoDesc;

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

    System_printf("Beginning AIF2 LTE RF tests:\n");
    UTILS_waitForHw(100000);


#if EVM_TYPE == 4
	UTILS_GPIO8_setup();
#endif

   // AIF2 LLD programs AT event 5 with period 0.5ms in case of LTE
   aif2evt5_userIsr = slotIsr;

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
	aifObj.mode               = AIF_LTE_FDD_MODE;
#ifdef SUPERPACKET
	aifObj.superPacket        = true;
#endif
	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
	else
		aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

    for(ntest=0;ntest<TEST_NUM;ntest++)
   	{
    	if (testEnable[ntest] == 1 )
    	{
    		chan 	= 0;
    		nblink 	= 0 ;
    		for (i=0;i<AIF_MAX_NUM_LINKS;i++)
    		{
    			if (testObjTab[ntest].linkEnable[i]==1)
    			{
    				nblink++;
    				// PktDma parameters
    				for (idx=0 ; idx<NBCHANNELPERLINK ; idx++)
    				{
    					aifObj.pktDmaConfig.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
    					aifObj.pktDmaConfig.txNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
    					aifObj.pktDmaConfig.txDescSizeAxC[chan] = LTESYMBOLSIZE;
    					aifObj.pktDmaConfig.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
    					aifObj.pktDmaConfig.rxNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
    					aifObj.pktDmaConfig.rxDescSizeAxC[chan] = LTESYMBOLSIZE*NBCHANNELINIT;

    					memset(&rxFlow[i][idx], 0, sizeof(Cppi_RxFlowCfg));
    					rxFlow[i][idx].rx_dest_qnum     = MONO_RX_Q+chan;
    					rxFlow[i][idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ+chan;
    					rxFlow[i][idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
    					rxFlow[i][idx].rx_fdq1_qnum     = MONO_RX_FDQ+chan;
    					rxFlow[i][idx].rx_fdq2_qnum     = MONO_RX_FDQ+chan;
    					rxFlow[i][idx].rx_fdq3_qnum     = MONO_RX_FDQ+chan;
    					rxFlow[i][idx].rx_psinfo_present = 1;
    			//		rxFlow[i][idx].rx_ps_location   = 1;
    					rxFlow[i][idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS

    					aifObj.pktDmaConfig.hRxFlowAxC[chan]   	= &rxFlow[i][idx];
    					aifObj.pktDmaConfig.hRxFlowCtrl[chan]   = NULL;
    					chan++;
    				}
    			} else {
    				aifObj.pktDmaConfig.hRxFlowAxC[chan]    = NULL;
    			}
    		}
    		if (CPRI_FAST_CM) {
    			// PktDma parameters for control words (CPRI control word FastC&M)
    			aifObj.pktDmaConfig.txRegionCtrl[0]   = UTILS_getMemRegionNum(mono_region_test);
    			aifObj.pktDmaConfig.txNumDescCtrl[0]  = 8;
    			aifObj.pktDmaConfig.txDescSizeCtrl[0] = 704;
    			aifObj.pktDmaConfig.rxRegionCtrl[0]   = UTILS_getMemRegionNum(mono_region_test);
    			aifObj.pktDmaConfig.rxNumDescCtrl[0]  = 8;
    			aifObj.pktDmaConfig.rxDescSizeCtrl[0] = 704;

    			memset(&rxFlowCpriCw, 0, sizeof(Cppi_RxFlowCfg));
    			rxFlowCpriCw.rx_dest_qnum     = MONO_RX_Q+chan;;
    			rxFlowCpriCw.rx_fdq0_sz0_qnum = MONO_RX_FDQ+chan;
    			rxFlowCpriCw.rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // monolythic
    			rxFlowCpriCw.rx_sop_offset    = 12;   // desc header size

    			aifObj.pktDmaConfig.hRxFlowCtrl[0]    = &rxFlowCpriCw;
    			aifObj.pktDmaConfig.hRxFlowCtrl[1]    = NULL;
    			aifObj.pktDmaConfig.hRxFlowCtrl[2]    = NULL;
    			aifObj.pktDmaConfig.hRxFlowCtrl[3]    = NULL;
    		}

    		nbchanneltotal = NBCHANNELPERLINK*nblink;

    		memset(mono_region_test, 0, NBDESCMAX * LTESYMBOLSIZE * NBCHANNELINIT);

    		System_printf("test: %s\n", testObjTab[ntest].name);
    		// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
    		UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
#ifdef SUPERPACKET
				 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELMAXPERLINK;
				 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELMAXPERLINK;
#else
				 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELPERLINK;
				 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELPERLINK;
#endif
				 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
#if LTE_RATE == 20
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_30P72MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_8b8;
#elif LTE_RATE == 10
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_15P36MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_4b4;
#elif LTE_RATE == 5
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_7P68MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_2b2;
#endif
				 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				 aifObj.linkConfig[i].psMsgEnable        = CPRI_FAST_CM;
				 if (intLoopback == 1 ) aifObj.linkConfig[i].comType = AIF2_LOOPBACK;
				 else	 				aifObj.linkConfig[i].comType = AIF2_2_AIF2;
				 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i]; //NA for pkDMA
            }

        	//printf("test: %s\n", testObjTab[ntest].name);
	        if (DSP_procId == 1)
				UTILS_printLinkConfig(&aifObj);

    		/* For SCBP-Appleton EVM run, we'll now proceed with SW clock sync */
    		if (DSP_procId == 1)
    		{
    			UTILS_initTimer(TRIG_TIMER);
    		}

    		// initialization function for the AT timer and EDMA3 ISRs
    		UTILS_aifIntcSetup();

#if EVM_TYPE == 5 // DSP_ProcId = 1
    		UTILS_boardsSync(SYNC_TIMER,PLL_TIMER);
#endif
#if EVM_TYPE == 4
    		if(DSP_procId == 2)
    		{
    			UTILS_resetTimer();
    			routeTimerLowOutToTimerOut0Pin(); //choose Timer0 output to generate the 1 Hz pps signal
    			ppsGen();                         //generate the 1 Hz pps signal
    		}
#endif

			// Compute default AIF2 parameters given this user configuration
			AIF_calcParameters(&aifObj);

			// initialization function for qmss and cppi low-level drivers
			UTILS_initQmss((uint32_t*)mono_region_test, NBDESCMAX, LTESYMBOLSIZE * NBCHANNELINIT, 0 ,NULL);

			// initialization of Pktdma and Qmss resources given this user configuration
			AIF_initPktDma(&aifObj);

			// Adjust AIF2 timing parameters if calcParameters didn't implement this LTE case or doesn't match the application Tx/Rx delays
			for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++){
				if (aifObj.linkConfig[i].linkEnable) {
					aifObj.linkConfig[i].pe2Offset   = 310;
					aifObj.linkConfig[i].deltaOffset = aifObj.linkConfig[i].pe2Offset + 70;
					aifObj.linkConfig[i].piMin       = aifObj.linkConfig[i].pe2Offset + 70; // + 40 ;
					activeLink = i;
				}
			}

			// initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
			AIF_initHw(&aifObj);

			/*
			 *  LTE Tx packet symbol initialization at once
			 */
			for(chan = 0;chan < nbchanneltotal;chan++)
			{
				idx1=0;
			    for(idx = 0;idx < (NBSYMBOL*2);idx++)
			    {
					symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
					mono_pkt = symbolPkt[chan][idx];

					//Create Mono packet
					Cppi_setDescType((Cppi_Desc *)mono_pkt,Cppi_DescType_MONOLITHIC);
					Cppi_setPacketType(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,0);
					Cppi_setDataOffset(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,16);

					if((idx%NBSYMBOL) == 0) {
						Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(LTESYMBOLSIZE-16));
					} else {
						Cppi_setPacketLen(Cppi_DescType_MONOLITHIC, (Cppi_Desc *)mono_pkt,(LTESYMBOL2SIZE-16));
					}

					Cppi_setPSFlags (Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,1);
					Cppi_setPSLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,4);

					queueInfo = Qmss_getQueueNumber(aifObj.pktDmaConfig.txFqAxC[chan]);
					descQueue.qMgr = queueInfo.qMgr;
					descQueue.qNum = queueInfo.qNum;
					Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,descQueue);

					descTag.destTagHi = 0;					descTag.destTagLo = 0;
					descTag.srcTagHi  = 0;					descTag.srcTagLo  = 0;
					Cppi_setTag(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,&descTag);

					// Get payload address
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);

					if((idx%NBSYMBOL) == 0)
					{
						 //payload data setup(first symbol)
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							getcomplex(&payloadPtr[idx2],idx1,chan);
							idx1++;
						}
					} else {
						//payload data setup(other six symbols)
						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
						{
							getcomplex(&payloadPtr[idx2],idx1,chan);
							idx1++;
						}
					}
					firstSample[chan][idx].re =  payloadPtr[0].re;
					firstSample[chan][idx].im =  payloadPtr[0].im;

					//Create PS data
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
					*((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

					//Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
			    }
			}
			/*
			 * Perform block writeback on L1D to make sure all symbol packets are coherent with MSM
			 */
			for(idx = 0;idx < (NBDESCMAX * NBCHANNELMAXPERLINK);idx++) {
				CACHE_wbInvL1d((void *)(mono_region_test+(idx*LTESYMBOLSIZE)), LTESYMBOLSIZE, CACHE_WAIT);
			}

			// start AIF2 HW and PktDMA and then wait for frame synchronization
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
			AIF_startHw(&aifObj);

			monoRxCount = 0;
			monoTxCount = 0;
	        monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]);
	        monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txQAxC[0]);
#ifdef LOOPBACK
			UTILS_triggerFsync(&aifObj);
#else
			if (DSP_procId == 1){
				UTILS_triggerFsync(&aifObj);
			}
#endif
			if (printexceptions == 1) {
				// wait 4 slots and then enable exceptions
				while (slotcount<= 14);
					AIF_enableException(&aifObj);
			}

			monoRxCount = 0;
			monoTxCount = 0;
			frameCount = aifFsyncEventCount[1];
			ctrlPktCount = 0;
			while(1)
			{
				Task_yield();
				if (CPRI_FAST_CM) {
					gie = _disable_interrupts();
					if (frameCount < aifFsyncEventCount[1]) {
						frameCount = aifFsyncEventCount[1];
						// pop from control stream free queue
						ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqCtrl[0]));
						// push control data on tx queue
						if (ptrMonoDesc != NULL) Qmss_queuePushDesc(aifObj.pktDmaConfig.txQCtrl[0], (uint32_t*)ptrMonoDesc);
						// check if new packet available on the receive side
						entries = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQCtrl[0]);
						if (entries > 0){
							ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQCtrl[0]));
							// recycle control packet on rx free queue
							Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqCtrl[0], (uint32_t*)ptrMonoDesc);
							ctrlPktCount++;
						}
					}
					_restore_interrupts(gie);
				}

#if NEVER_END_TEST == 0
				// wait for one extra slots to run out of Rx free pkts
				if(slotcount == (SLOT_NUM_DATA_CHECK + 2))
				{

					/*while (monoRxCount < (NBSYMBOL*2) )
					{
						// Get current descriptor count for monolithic RX queue
						monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]);
						monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[0]);
					}*/
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					//keepAifOff = 1;
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					break;
				}
#endif
			}



			if (printexceptions == 1) {
				// disable interrupts before checking for data
				CSR&= 0xFFFFFFFE;
			}

			monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]); // get number of received packed on first channel
			monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[0]); // channel 0 only
			System_printf(" Number of monolithic packets received in RX queue channel0: %d\n", monoRxCount);
			System_printf(" Number of monolithic packets in TX free queue for channel0: %d\n", monoTxCount);
			monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[1]);
			monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[1]);
			System_printf(" Number of monolithic packets received in RX queue channel1: %d\n", monoRxCount);
			System_printf(" Number of monolithic packets in TX free queue for channel1: %d\n", monoTxCount);

			/* Check received symbol packet data */
			for(chan =0; chan < nbchannelSuperPacket; chan++)
			{
				idx1 = 0;
				idx3 = 0;
				testpass = 1;
				rx_count = Qmss_getQueueEntryCount((aifObj.pktDmaConfig.rxQAxC[chan]));
				if (rx_count == 0) testpass =0;

				for (idx = 0; idx < NBSYMBOL*2; idx ++)
				{
					symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
					payloadPtr = (Complex16 *)symbolPkt[chan][idx];
					payloadPtr += 4; //skip pkt header and PS field (16 bytes)
					testpass = 1;
					if((idx%NBSYMBOL) == 0)
					{
#ifdef SUPERPACKET
						UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr, (uint32_t *)rxBuffer[0], \
								(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, LTESYMBOLSIZE-16);						
						UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr + aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
												 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
						payloadPtr = (Complex16 *)rxBuffer[0];
#else
						CACHE_invL1d((void *)payloadPtr, (LTESYMBOLSIZE-16), CACHE_WAIT);						
#endif
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							getcomplex(&sampleCheck,idx1,chan);
							if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
							{
								testpass = 0; testcheck++;
							}
							idx1++;
						}

#ifdef SUPERPACKET
						payloadPtr = (Complex16 *)rxBuffer[1];
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							getcomplex(&sampleCheck,idx3,chan+1);
							if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
							{
								testpass = 0; testcheck++;
							}
							idx3++;
						}
#endif
					} else {						
#ifdef SUPERPACKET
						UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr, (uint32_t *)rxBuffer[0], \
								(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, LTESYMBOL2SIZE-16);						
						UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr + aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
								(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, LTESYMBOL2SIZE-16);
						payloadPtr = (Complex16 *)rxBuffer[0];
#else
						CACHE_invL1d((void *)payloadPtr, (LTESYMBOL2SIZE-16), CACHE_WAIT);												
#endif
						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
						{
							getcomplex(&sampleCheck,idx1,chan);
								if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
							{
								testpass = 0; testcheck++;
							}
							idx1++;
						}
#ifdef SUPERPACKET
						payloadPtr = (Complex16 *)rxBuffer[1];
							for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
							{
								getcomplex(&sampleCheck,idx3,chan+1);
									if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
								{
									testpass = 0; testcheck++;
								}
								idx3++;
							}
#endif
					}
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

				}
				System_printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
			}

			if (testpass == 1) {
				System_printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
				//testcheck = 0;
			} else {
				System_printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
			}

			/* read the descriptor counts of the Monolithic queues. */
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txQAxC[i]);
			}
			if (value != 0) System_printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
			else System_printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[i]); // channel0
			}
			if (value != nbchanneltotal*NBSYMBOL*2) System_printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
			else System_printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchannelSuperPacket; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[i]);
			}
			if (value != 0) System_printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
			else System_printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchannelSuperPacket; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxFqAxC[i]);
			}
			if (value != nbchannelSuperPacket*NBSYMBOL*2) System_printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
			else System_printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

			UTILS_doCleanup(&aifObj,TRIG_TIMER);

			if (printexceptions == 1)
			{
			   	AIF_printException(&aifObj);
			}

			System_printf("\nEnding %s test\n", testObjTab[ntest].name);
 	    }
    }

    System_printf("Test: ending AIF2 LTE 20Mhz RF test \n");

    if (testcheck == 1) {
           testcheck = 0;
           System_printf("All tests have passed\n");
    } else {
           System_printf("Some tests have failed\n");
    }

	System_abort("End of Program!!\n");

}

////////////

