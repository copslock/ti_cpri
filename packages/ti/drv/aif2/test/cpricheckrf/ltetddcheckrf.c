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

//////////////////////////////////////////////////////////////////////////////
// Include Files

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#if EVM_TYPE == 3
#include <ti/platform/evmc6670l/platform_lib/include/evmc66x_pllc.h>
#endif

#if EVM_TYPE == 0
#include <EVM.h>
#endif

//include for GPIO
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>

#include "cslUtils.h"
#include "mnavUtils.h"
#include "mathUtils.h"

#include <DSP_fft16x16.h>
#include <gen_twiddle_fft16x16.h>
#include <gen_twiddle_fft16x16.c>

#if EVM_TYPE == 5 || EVM_TYPE == 7
#include "appletonScbpSync.h"
#endif

/* TEST DESCRIPTION
 * This CPRI LTE TDD Check RF test is designed to work either in internal loopback mode, or with a dual EVM setup connected with a break-out card, or with a
 * CPRI Relay setup (tci6614 evm + scbp + tsw3726), or with a Scbp/Tsw3726 standalone setup.
 * The default project configurations in the pdk-generated workspace are supporting:
 * - EVM_LBACK: test compiled for internal loopback mode. Can be used with device simulators as well.
 * - EVM6614: test compiled for CPRI Relay setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - SCBP_DSP1: test compiled for Scbp/Tsw3726 standalone setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - EVM6670_DSP1 and EVM6670_DSP2: test compiled for first and second EVMs of a dual EVM setup connected with a break-out card
 *
 * This CPRI LTE TDD Check RF test runs on a single link of AIF2 and configures 2 AxCs. Both AxCs IQ data are pure single or dual tone sinewaves at given frequencies,
 * such that upon reception of LTE symbols for both antennas, IQ data can be checked with fft software. If the signal doesn't go from digital to analog and back, it
 * can also be checked bitwise. For the purpose of TDD validation in RF loopback mode, only UL symbols are pushed in egress direction.
 *
 * Pre-processing constants of interest:
 * - CPRI_RELAY_CFG: Set it to 1 as default. Set to 2 if testing towards tsw3726 RF board.
 * - CPRI_RELAY_RATE: selects the LTE rate to be used. Values: 5, 10, or 20
 * - EVM_TYPE: 1 - KeyStone-I internal loopback or EVM DSP1 (dual EVM setup), 2 - EVM DSP2 (dual EVM setup), 3 - SCBP DSP1, 5 - EVM6614 (Cpri Relay setup), (0 - old Lyrtech C6670 EVM)
 *             6 - KeyStone-II internal loopback (EVM), 7 - EVM6638K2K (Cpri Relay setup)
 * - LOOPBACK: selects internal loopback mode (SerDes level, AIF2 SW trigger)
 *
 * Note on usage with CPRI relay setup:
 * 		0. FPGA firmware upgrades:
 *          a. Scbp board must be upgraded to the proper SCBP FPGA LTE version for the given LTE rate of interest.
 *          b. Tci6614 EVM boards must be upgraded to v0004 of Fpga firmware
 * 		1. In order to run the CPRI relay test, go to the SCBP CPRI relay web interface and run the following config:
 * 			a. For the CPRI relay loopback (loopback on the link0 through Scbp FPGA, CPRI_RELAY_CFG=1):
 * 				- click on "Relay Init" to configure the pll with the 10Mhz reference clock, enable the SFP and SPI port.
 * 				- Run the "relay LOOPBACK" button to configure the link0 for internal loopback in FPGA.
 * 				- Load and run the code on the Appleton DSP.
 * 			b. For the CPRI relay going through TI's RF (CPRI_RELAY_CFG=2), the radio needs to be configured as well as SCBP FPGA:
 * 				- click on "Radio Init"
 * 				- click on "Relay Init" to configure the pll with the 10Mhz reference clock, enable the SFP and SPI port.
 * 				- load and run the Appleton code.
 * 				- when the boards are synchronized (after few seconds), do a Radio ON to enable the radio.
 *
 *		In case of Cpri relay setup, the reference clock use to synchronize both board is a 10Mhz clock at 3.3V connected to the SCBP.
 *		Then the SCBP generate a 16Hz pps signal that will be used by the Tci6614 board to synchronize with SCBP via the BoardsSync function.
 *
 * Note on Lte Symbol packet descriptors
 * 14 symbols of an AxC subframe time are contained in 14 different monolithic descriptors. The pure single or dual tone sinewaves are generated in a continuous manner
 * across the payload of the 14 symbols of a particular AxC. The test is configured to work at the granularity of a Lte slot. For LTE, AIF2 requires usage of monolithic
 *  descriptor packets with a 16-byte header. In the header, a protocol specific word is required to indicate the antenna carrier number, the symbol number and the direction
 *  (Ingress/Egress). That means that every 0.5 ms, the test will push and pop from the AIF2 HW queues:
 * 7 symbols * numAxCs per link * numLinks * 2 (for Egress/Ingress traffic)
 * This then makes a total of 56 outstanding monolithic packets in the case of LTE 20MHz / 2 AxCs / 1 Links / both directions
 * When defining a descriptor memory region, the following rules apply:
 * - Descriptor size is multiple of 16 bytes, min 32.
 * - Descriptor count is a power of 2, minimum 25.
 * - Memory region base address must be aligned to a 16-byte address boundary.
 * Here are some considerations regarding PktDMA traffic:
 * 		- For Egress, it is up to the user to insure that DMA data is available prior to beginning PE message construction (PE2 event). Breaking real time on DMA has
 * 		similar effects to breaking CorePac MIPS real time. Specifically, AIF2 will fail each affected AxC until the beginning of the next radio frame boundary.
 * 		- For Ingress, it is important that DMA is efficient enough that the AIF2 input	buffers never overflow.
 * 		- TDD specifics: for TX, TDD_AXC mode should be enabled in PE, if it is turned on, PE will insert symbol only when it receives the packet from packet DMA.
 * 		So, it is user’s responsibility to not push packets into the packet DMA TX queue during inactive TDD time slots; and for active TDD timer slots, user should
 * 		take care the time to push packet into the TX queue. A packet should  only be pushed into TX queue just one packet ahead, for example, if packet of
 * 		LTE symbol N need be sent in active TDD time slot, then packet N must be pushed into the TX queue  in the time period of symbol (N-1).
 * 		For RX, PD implements programmable bit map for each channel, each bit corresponds to one packet in a radio Frame, “1” means corresponding packet will be
 * 		received, “0” means that packet will be dropped. Please note, in LTE protocol level, the TDD is controlled in the unit of sub frame, but the AIF2 PD TDD
 * 		bit map is defined in the unit of packets, and one LTE sub frame includes multiple packets (symbols), so the LTE TDD “bit map” in sub frame should be
 * 		expanded to PD TDD “bit map” in packets, that is, we need to program multiple bits to select each LTE symbol in a sub frame.
 * 		Example with AIF2 LLD:
 * 		aifObj.linkConfig[i].lteTddUlDlCfg[j]   = AIF2_LTETDD_ULDL_CFG0;	// UL DL configuration for Lte TDD
 * 		aifObj.linkConfig[i].lteTddSsfNcpCfg[j] = AIF2_LTETDD_SSF_NCP_CFG0;	// Special SF configuration for Lte TDD
 */


////////////////////////////////////////////////////////////////////////////////
// Constant definitions

#define     TRIG_TIMER          0 	// Used to start AIF2 timers, Frame sync with one shot pulse - could be any timer
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define 	SYNC_TIMER			8
#else
#define     SYNC_TIMER          5	// Used to call the SW PLL periodically - Cpri relay setup only - could be an OS PRD thread as well
#endif
#define     PLL_TIMER           4	// Used by the SW PLL for adjustments against the external 16Hz clock - Cpri relay setup only - could be any timer

//Value for Sin calculation
#if CPRI_RELAY_CFG == 1
#define     Q_FACTOR            511		// When the signal stays digital, reducing Q factor to avoid overflow in fft software
#else
#define     Q_FACTOR            2047 	// 8191
#endif
#if CPRI_RELAY_RATE == 20
#define     SAMP_FREQ_FLOAT     30.72 	// LTE20 sample rate
#elif CPRI_RELAY_RATE == 10
#define     SAMP_FREQ_FLOAT     15.36	// LTE10 sample rate
#elif CPRI_RELAY_RATE == 5
#define     SAMP_FREQ_FLOAT     7.68	// LTE5 sample rate
#else
#error 		"CPRI_RELAY_RATE not valid."
#endif
#define     PI                  3.14159265358979323846

#define     TEST_NUM            2	// Number of test configurations
#define 	_AIF2FL_DUMP		0	// Enable the DUMP of the AIF configuration, for debug purposes
#define     NBCHANNELPERLINK	2	// Number of AxCs in use per link
#if CPRI_RELAY_RATE == 20
#define     NBCHANNELMAXPERLINK 2	// Max number of AxCs per link
#elif CPRI_RELAY_RATE == 10
#define     NBCHANNELMAXPERLINK 4	// Max number of AxCs per link
#elif CPRI_RELAY_RATE == 5
#define     NBCHANNELMAXPERLINK 8	// Max number of AxCs per link
#endif
#define     NBSYMBOL			AIF2_LTE_SYMBOL_NUM											// Number of symbols per slot
#if CPRI_RELAY_RATE == 20
#define		LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#define		FFT_SIZE			AIF2_LTE20_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE20_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE20_CYPRENORMAL_SIZE
#elif CPRI_RELAY_RATE == 10
#define		LTESYMBOLSIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)    
#define 	LTESYMBOL2SIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL_SIZE)*4 + 16)
#define		FFT_SIZE			AIF2_LTE10_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE10_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE10_CYPRENORMAL_SIZE
#elif CPRI_RELAY_RATE == 5
#define		LTESYMBOLSIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL_SIZE)*4 + 16)
#define		FFT_SIZE			AIF2_LTE5_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE5_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE5_CYPRENORMAL_SIZE
#endif

#define		NBCHANNELINIT		1

#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 5 || EVM_TYPE == 7
#define     DESCMSM				1			// Putting packet descriptors in internal MSM
#else
#define     DESCMSM				1			// else going to DDR
#endif
#define		NBDESCMAX			64          // Descriptor count rounded to the next power of 2
#define		MAX_DEBUG_SYMBOL	280			// For debug, size of monitoring arrays
#if EVM_TYPE == 0 || EVM_TYPE == 3
#define     SYSCLK_INPUT_KHZ    153600  	// on Lyrtech and Scbp EVM, the Frequency is based on an external 153.60 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  	// on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif

#define     DIO_0               AIF2FL_DIO_ENGINE_0 // DIO engines are not used in LTE mode - instead PKTDMA and HW queues are used to carry the antenna data from/to AIF2

#define 	MONO_RX_FDQ         2012 // Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index
#define 	MONO_RX_Q           1000

#define 	RX_SUCCESS_CNT		(32000*7) // Define a number of consecutive runtime RF check success prior to monitoring runtime failures
										  // RF checks now done at symbol pace

#define 	FREQ				1.0
#define     PEAK_LOC_2MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ*2) // Used for peak detection after fft on Rx symbols  - 2048/30.72*2MHz, same for Lte5 or Lte10
#define 	PEAK_LOC_1MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ)   // Used for peak detection after fft on Rx symbols  - 2048/30.72*1MHz, same for Lte5 or Lte10
#define		N   (2048)											  // Used for twiddle factor generation
#define 	PAD (16)											  // Used for twiddle factor generation

#ifdef LOOPBACK
#define FIRST_SYMBOL_PUSH	139  			// The first push needs to happen on the last symbol number before frame boundary
#else
#define FIRST_SYMBOL_PUSH	138
#endif
#if CPRI_RELAY_CFG == 2
#define SYMBOL_NUM_DATA_CHECK	(75000*7)	// If NEVER_END_TEST == 0, test stops after this amount of Lte slots
#else
#define SYMBOL_NUM_DATA_CHECK	(7500*7)	// If NEVER_END_TEST == 0, test stops after this amount of Lte slots
#endif

//////////////////////////////////////////////////////////////////////////////
// Typedefs

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

//////////////////////////////////////////////////////////////////////////////
// Global variables
AIF_ConfigObj               aifObj;					// Main AIF2 LLD object for AIF2 configuration
AIF_ConfigHandle            hConfigAif = &aifObj;

#if EVM_TYPE == 3
uint32_t NEVER_END_TEST=1; 				// Enable the infinite loop for visual check when running on SCBP-TSW standalone setup
#else
uint32_t NEVER_END_TEST = 0; 				// Test stops after a given number of Lte slots
#endif

volatile unsigned int ntest = 0; 		// used for running multiple tests
volatile unsigned int testcheck = 1; 	// reports pass fail at the end of the test

#ifdef LOOPBACK
volatile unsigned int swSync        = 1;	// AIF2 SW trigger
volatile unsigned int intLoopback   = 1;	// SerDes loopback mode
#else
volatile unsigned int intLoopback   = 0;
#if EVM_TYPE == 3
volatile unsigned int swSync        = 1;
#else
volatile unsigned int swSync        = 0;
#endif
#endif

volatile unsigned char testEnable[TEST_NUM] =
    {
#if EVM_TYPE == 5 || EVM_TYPE == 7
        1,     // CPRI 4x - Link 3 is used on CPRI relay setup to connect TCI6614 EVM and SCBP via SFP
        0
#else
        0,     // CPRI 4x - Link 0
		1
#endif
    };

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st configuration - CPRI 4x - Link 3 is used on CPRI relay setup to connect TCI6614 EVM and SCBP via SFP
#if CPRI_RELAY_RATE == 20
	  "LTETDD_RELAY_20MHZ_RF_CHECK", // test name
#elif CPRI_RELAY_RATE == 10
	  "LTETDD_RELAY_10MHZ_RF_CHECK", // test name
#elif CPRI_RELAY_RATE == 5
	  "LTETDD_RELAY_5MHZ_RF_CHECK", // test name
#endif
	 // link0          link1          link2          link3          link4          link5
	  {0,             0,             0,             1,             0,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
	},
	{//2nd configuration - CPRI 4x - Link 0
#if CPRI_RELAY_RATE == 20
	  "LTETDD_RELAY_20MHZ_RF_CHECK", // test name
#elif CPRI_RELAY_RATE == 10
	  "LTETDD_RELAY_10MHZ_RF_CHECK", // test name
#elif CPRI_RELAY_RATE == 5
	  "LTETDD_RELAY_5MHZ_RF_CHECK", // test name
#endif
	 // link0          link1          link2          link3          link4          link5
	  {1,             0,             0,             0,             0,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
	}
   };

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
uint8_t   mono_region_test[NBDESCMAX * LTESYMBOLSIZE * NBCHANNELINIT];

/*
 * Buffers storing TX and RX symbols for debug and verification
 */
#pragma DATA_SECTION(txSamples,".aifdescddr");
uint8_t   txSamples[NBCHANNELPERLINK][NBSYMBOL*2][LTESYMBOLSIZE - 16];
#pragma DATA_SECTION(rxSamples,".aifdescddr");
uint8_t   rxSamples[NBCHANNELPERLINK][NBSYMBOL*2][LTESYMBOLSIZE - 16];

/*
 * Variables used for RF checks
 */
uint8_t   rxBuffer[NBCHANNELPERLINK+1][LTESYMBOLSIZE-16]; // MUST BE IN LL2 to enable HW cache coherency
uint8_t   rxCheck[NBCHANNELPERLINK][NBSYMBOL];			// Keeps track of current check of all LTE symbols on a slot per AxC
uint16_t  rxRuntimeFail[NBCHANNELPERLINK][NBSYMBOL];		// Keeps track of the number of failures per LTE symbols on a slot per AxC
uint8_t   rxCheckCnt=0;									// Counter
uint32_t  rxRuntimeSuccess=0;								// Counter

/*
 *	Variables used fft software
 */
#pragma DATA_ALIGN(twiddleFacts, 8);
int16_t   twiddleFacts[2*N + 2*PAD];

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS][NBCHANNELMAXPERLINK];

/* Variables for test purposes */
uint32_t nbchanneltotal    = NBCHANNELPERLINK; // Default to 1 link
uint32_t   activeLink;						// link configured at test init time

/*
 * Slot and symbolcount counters - some used for debug purposes
 */
#ifdef LOOPBACK
volatile uint32_t symbolcount = 1; 		//for Loopback test, the frame boundary occurs with a delay of 1 symbol (sw sync issue)
volatile uint32_t symbolslotcount = 1;	// count from 0 to 6 and is used to determine the symbol length
#else
volatile uint32_t symbolcount = 2; 		//for no-Loopback test, the frame boundary occurs with a delay of 2 symbols (phy sync issue)
volatile uint32_t symbolslotcount = 2;
#endif

volatile uint32_t totalsymbolcount = 0;
volatile uint32_t aif2SymbolEgressCount[MAX_DEBUG_SYMBOL];
volatile uint32_t aif2SymbolIngressCount[MAX_DEBUG_SYMBOL];
volatile uint32_t getCpuTimestamp[MAX_DEBUG_SYMBOL];
volatile uint32_t getPhyTimerFrames[MAX_DEBUG_SYMBOL];
volatile uint32_t getPhyTimerClk[MAX_DEBUG_SYMBOL];
volatile uint32_t getRadtSymbolCnt[MAX_DEBUG_SYMBOL];
volatile uint32_t egressSymEnable[144];
volatile uint32_t receivedPacket[NBCHANNELPERLINK];
volatile uint32_t dbgSymbol = 0;
volatile uint32_t checkRfTimerStamp[3];
volatile uint32_t checkchan = 0;
#if CPRI_RELAY_CFG != 2		// if not using the CPRI relay with the RF, we can check every packet first data in order to verify if it's not null.
volatile uint32_t badPacket = 0;
volatile uint32_t goodPacket = 0;
#endif

/*
 * Flag used for printing out exceptions at the end of the test.
 * Please note that exceptions are always enabled during this test.
 * Having AIF2 exceptions reported during this test should be considered as a test failure.
 *
 */
uint32_t printexceptions = 1;

//////////////////////////////////////////////////////////////////////////////
// References to external variables

extern uint8_t		   keepAifOff;
extern uint8_t		   keepStartupDelay;
//extern int 		       PEAKDEBUG;

//////////////////////////////////////////////////////////////////////////////
// Function prototypes

#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif
void getcomplex(Complex16* payload,uint32_t index,uint32_t tx);
void UTILS_setLteTddDlBitmap(AIF2_LteTddUlDlCfg tddSubFrameBitMap, AIF2_LteTddSsfNcpCfg tddSpecSubFrameBitMap);

//////////////////////////////////////////////////////////////////////////////
// Function bodies

/*
 * User Isr that is called from pre-defined Isr in cslUtils.c, called every Lte symbol
 */
void symbolIsr()
{
	uint32_t chan, i, checkrf;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc, *ptrMonoDescRx;
	uint32_t              *payloadPtr;
	int32_t				testrf;

    getPhyTimerFrames[dbgSymbol]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
    getPhyTimerClk[dbgSymbol]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;
    getRadtSymbolCnt[dbgSymbol]	  = (aifObj.hFl->regs->AT_RADT_VALUE_LSBS >> 19);

    aif2SymbolEgressCount[dbgSymbol]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
   	aif2SymbolIngressCount[dbgSymbol] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;

   	checkrf = 0;

	if ((totalsymbolcount <= SYMBOL_NUM_DATA_CHECK) || (NEVER_END_TEST == 1))
	{

		for(chan = 0; chan < nbchanneltotal; chan++)
		{
			rxSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);

			for(i=0;i<rxSymCnt;i++)
			{
				ptrMonoDescRx = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
				CACHE_invL1d((void *)ptrMonoDescRx, 16+8, CACHE_WAIT);
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDescRx, (uint8_t**)&payloadPtr, &payloadLen);
				 //check payload length
				if ((payloadLen != (LTESYMBOLSIZE-16)*NBCHANNELINIT) && (payloadLen != (LTESYMBOL2SIZE-16)*NBCHANNELINIT))
				{
					UTILS_aif2ExceptIntDisable();
					totalsymbolcount = SYMBOL_NUM_DATA_CHECK  + DSP_procId;
					NEVER_END_TEST = 0;
					printf("FATAL: unexpected payload length:%d\n",payloadLen);
					AIF_printException(&aifObj);
				} else {
					if (checkchan == chan) {
					CACHE_invL1d((void *)payloadPtr, payloadLen, CACHE_WAIT);
					memcpy(&rxBuffer[chan][0],(uint32_t *) payloadPtr, payloadLen);
#if	EVM_TYPE == 5 || EVM_TYPE == 7 || EVM_TYPE == 8
					checkrf = 1;
#endif
					}
				}
#if CPRI_RELAY_CFG != 2		//store the number of symbols that are good or bad.
				if ((payloadPtr[0] == 0)&&(payloadPtr[1] == 0))
				{
					badPacket++;
				} else{
					goodPacket++;
				}
#endif
				// recycle symbol packet on rx free queue
				Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*)ptrMonoDescRx);
			}
			receivedPacket[chan] += rxSymCnt;
		}
	}

	if (totalsymbolcount == SYMBOL_NUM_DATA_CHECK) {
    	UTILS_aif2ExceptIntDisable();
    }

	if ((totalsymbolcount>=FIRST_SYMBOL_PUSH))
	{
		for(chan = 0; chan < nbchanneltotal; chan++)
		{
			if((1==egressSymEnable[symbolcount])||(totalsymbolcount<(FIRST_SYMBOL_PUSH+7)))
			{
				ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
				if(ptrMonoDesc!= NULL)
				{
					CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
					if((symbolslotcount) == 0) {
						Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)ptrMonoDesc,(LTESYMBOLSIZE-16));
					} else {
						Cppi_setPacketLen(Cppi_DescType_MONOLITHIC, (Cppi_Desc *)ptrMonoDesc,(LTESYMBOL2SIZE-16));
					}
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
					payloadPtr[0] = (uint32_t )(0x00008000 + chan + ((symbolcount) << 7));//add symbol number into PS field
					CACHE_wbL1d((void *)ptrMonoDesc, 16, CACHE_WAIT); // writeback the descriptor header in MSM
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)ptrMonoDesc);
				} else {
					printf ("Tx free Q is empty at symbol %d\n",totalsymbolcount);
					UTILS_aif2ExceptIntDisable();
					totalsymbolcount = SYMBOL_NUM_DATA_CHECK  + DSP_procId;
					NEVER_END_TEST = 0;
					printf("FATAL: unexpected payload length:%d\n",payloadLen);
					AIF_printException(&aifObj);
				}
			}
		}
	}

	if (checkrf == 1)
	{
		checkRfTimerStamp[0] = TSCL;
		DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[checkchan][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[2][0]);
		checkRfTimerStamp[1] = TSCL;
		if (checkchan == 0) {
			testrf = checkSingleTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ,0);
		} else {
			testrf = checkDualTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ,PEAK_LOC_2MHZ,1);
		}
		if (testrf)
		{
			checkRfTimerStamp[2] = TSCL;
			rxCheck[checkchan][symbolslotcount] = 1;
			if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
		} else {
			checkRfTimerStamp[2] = TSCL;
			rxCheck[checkchan][symbolslotcount] = 0;
			if (rxRuntimeSuccess == RX_SUCCESS_CNT){
				rxRuntimeFail[checkchan][symbolslotcount]++;
			}
			else rxRuntimeSuccess = 0;
		}
		checkchan++;
		if (checkchan == nbchanneltotal) checkchan=0;
	}

    if (totalsymbolcount == FIRST_SYMBOL_PUSH) {
        AIF_enableException(&aifObj);
    }

	/* PktDMA activity monitoring */
	aif2SymbolEgressCount[dbgSymbol]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
	aif2SymbolIngressCount[dbgSymbol] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
    symbolcount++;
    symbolslotcount++;
	totalsymbolcount++;
	dbgSymbol++;
	if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
	if (symbolslotcount == AIF2_LTE_SYMBOL_NUM) symbolslotcount = 0;
	if (dbgSymbol == MAX_DEBUG_SYMBOL) dbgSymbol = 0;
}


int main(void)
{
    uint32_t               idx, idx1, idx2, i, payloadLen, testpass, j, nblink = 1;
    uint16_t               myboard	= EVM_TYPE;
	uint32_t               monoRxCount, monoTxCount, rx_count, value;
	Complex16            *payloadPtr;
	Cppi_MonolithicDesc  *ptrMonoDesc;

	uint32_t               chan;
    Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Qmss_Queue           queueInfo;
    Cppi_DescTag 		 descTag;
#if EVM_TYPE == 3
    PllcHwSetup pllc_hwSetup;
#endif
#ifdef SIMULATOR_SUPPORT
	printf ("\n Error: This example code does not run on the simulator.\n\n");
	return(0);
#endif

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

#if CPRI_RELAY_CFG == 1
    printf("Beginning AIF2 LTE TDD CPRI relay Software testing\n");
#else
    printf("Starting AIF2 LTE TDD CPRI relay Software\n");
#endif

    UTILS_waitForHw(100000);

    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
	else if (EVM_TYPE == 6) DSP_procId = 1;
    else if (EVM_TYPE == 7) DSP_procId = 1;
    else if (myboard <= 2)  DSP_procId = EVM_TYPE; // For Advantech: myboard  = 1 (DSP_1) or 2 (DSP_2)
    else DSP_procId = (uint8_t)(EVM_TYPE - 2);       // For SCBP: myboard  = 3 (DSP_1) or 4  (DSP_2)

#if EVM_TYPE == 3
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

   // AIF2 LLD programs AT event 6 with lte symbol period in case of LTE
   aif2evt6_userIsr = symbolIsr;

	// Take AIF out of power saver
	AIF_enable();

#ifdef USE_SMA
	// disable link 0 on DSP_1
	testObjTab[ntest].linkEnable[3]  = 0;
	testObjTab[ntest].linkEnable[4]  = 1;
	activeLink = 4;
#endif


	// General parameters
	memset(&aifObj, 0, sizeof(aifObj));
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
	aifObj.mode               = AIF_LTE_TDD_MODE;
	aifObj.superPacket 		  = false;

	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
	else
		aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

	UTILS_setLteTddDlBitmap(AIF2_LTETDD_ULDL_CFG0, AIF2_LTETDD_SSF_NCP_CFG0);

    for(ntest=0;ntest<TEST_NUM;ntest++)
   	{
    	if (testEnable[ntest] == 1 )
    	{
    		chan 	= 0;
    		nblink 	= 0 ;
			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				if (testObjTab[ntest].linkEnable[i]==1 && (aifObj.linkConfig[i].RtEnabled==0))
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
						rxFlow[i][idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS

						aifObj.pktDmaConfig.hRxFlowAxC[chan]   	= &rxFlow[i][idx];
						aifObj.pktDmaConfig.hRxFlowCtrl[chan]   = NULL;
						chan++;
					}
				} else {
					aifObj.pktDmaConfig.hRxFlowAxC[chan]    = NULL;
				}
			}
    		nbchanneltotal = NBCHANNELPERLINK*nblink;
    		memset(mono_region_test, 0, NBDESCMAX * LTESYMBOLSIZE * NBCHANNELINIT);
    		printf("Software configuration: %s\n", testObjTab[ntest].name);

    		// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
    		UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{

				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				 if (testObjTab[ntest].linkEnable[i] == 1)
				 {
					 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELPERLINK;
					 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELPERLINK;
				 }
				 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
#if CPRI_RELAY_RATE == 20
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_30P72MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_8b8;
#elif CPRI_RELAY_RATE ==10
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_15P36MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_4b4;
#elif CPRI_RELAY_RATE ==5
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_7P68MHZ;
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_2b2;
#endif
				 // avoid superpacket workaround (Advisory 12 for c6670)
#if EVM_TYPE == 0 || EVM_TYPE == 1 || EVM_TYPE == 2 || EVM_TYPE == 3
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
#endif

				 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				 aifObj.linkConfig[i].psMsgEnable        = 0;
				 if (intLoopback == 1 ) aifObj.linkConfig[i].comType = AIF2_LOOPBACK;
				 else	 				aifObj.linkConfig[i].comType = AIF2_2_AIF2;
				 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i]; //NA for pkDMA
				 for (j=0;j<NBCHANNELPERLINK;j++)
				 {
					 aifObj.linkConfig[i].lteTddUlDlCfg[j]   = AIF2_LTETDD_ULDL_CFG0;
					 aifObj.linkConfig[i].lteTddSsfNcpCfg[j] = AIF2_LTETDD_SSF_NCP_CFG0;
				 }
            }

			for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++)
			{
				if (aifObj.linkConfig[i].linkEnable)
				{
					activeLink = i;
				}
			}

#if EVM_TYPE == 5 || EVM_TYPE == 7
				aifObj.linkConfig[activeLink].nodeTx      = 0;
				aifObj.linkConfig[activeLink].nodeRx      = 2;
#endif

    		if (DSP_procId == 1)
    		{
    			UTILS_initTimer(TRIG_TIMER);
    		}

    		// Interrupt initialization
    		UTILS_aifIntcSetup();

    		/* For SCBP-Appleton EVM run, we'll now proceed with SW clock sync */
#if EVM_TYPE == 5 // DSP_ProcId = 1
    		UTILS_boardsSync(SYNC_TIMER,PLL_TIMER); // required board sync process when using CPRI relay setups
#endif

			// Compute default AIF2 parameters given this user configuration
			AIF_calcParameters(&aifObj);

            // initialization function for qmss and cppi low-level drivers
            UTILS_initQmss((uint32_t*)mono_region_test, NBDESCMAX, LTESYMBOLSIZE * NBCHANNELINIT, 0, NULL);

			// initialization of Pktdma and Qmss resources given this user configuration
			AIF_initPktDma(&aifObj);

			// Adjust AIF2 timing parameters if calcParameters didn't implement this LTE case or doesn't match the application Tx/Rx delays
#if EVM_TYPE == 5 || EVM_TYPE == 7
#if CPRI_RELAY_CFG == 2
				aifObj.linkConfig[activeLink].piMin += 20 + 64; // taking into account SCBP FPGA specific extra delay on Rx path when going thru RF
#endif
#endif

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

					// Always set for longer CP and then adjust length at runtime
					Cppi_setPacketLen(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)mono_pkt,(LTESYMBOLSIZE-16));

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

					 //payload data setup(first and longest symbol)
					for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
					{
						getcomplex(&payloadPtr[idx2],idx1,chan);
//							payloadPtr[idx2] = idx2+1;
						idx1++;
					}
					memcpy((void*)&txSamples[chan][idx][0],(void*)payloadPtr,LTESYMBOLSIZE-16);
					firstSample[chan][idx].re =  payloadPtr[0].re;
					firstSample[chan][idx].im =  payloadPtr[0].im;

					//Create PS data
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
					*((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

					Qmss_queuePushDesc((aifObj.pktDmaConfig.txFqAxC[chan]), (uint32_t*)mono_pkt);
				}
			}
			/*
			 * Perform block writeback on L1D to make sure all symbol packets are coherent with MSM
			 */
			for(idx = 0;idx < (NBDESCMAX * NBCHANNELMAXPERLINK);idx++) {
				CACHE_wbInvL1d((void *)(mono_region_test+(idx*LTESYMBOLSIZE)), LTESYMBOLSIZE, CACHE_WAIT);
			}
			gen_twiddle_fft16x16(&twiddleFacts[PAD], FFT_SIZE);
			memset(rxCheck,0x00,sizeof(rxCheck));
			memset(rxRuntimeFail,0x00,sizeof(rxRuntimeFail));
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

			// Disabling this mode, so that, on Cpri relay setup, once AIF2 timers are triggered, they keep running based on the initial pulse. Note there isn't a periodic 10ms pulse generated from EVMTCI6614, just a one shot pulse.
			aifObj.hAif2Setup->commonSetup->pAtCommonSetup->AutoResyncMode = AIF2FL_NO_AUTO_RESYNC_MODE;

#if CPRI_RELAY_CFG == 2
			// Compensating data delay due to Rf chain in the case of RF loopback
			aifObj.hAif2Setup->commonSetup->pPdCommonSetup->AxCOffset[0] = 122;
			aifObj.hAif2Setup->commonSetup->pPdCommonSetup->AxCOffset[1] = 122;
#endif

			// Now programming AIF2 registers
			AIF_startHw(&aifObj);

			// Now triggering AIF2 physical and radio timers
			if (DSP_procId == 1)
			{
				keepStartupDelay = 0;
				UTILS_triggerFsync(&aifObj);
			}

			// entering the idle loop and monitoring the end of the test, if any.
			monoRxCount = 0;
			monoTxCount = 0;
			while(1)
			{

				asm (" IDLE");
				// wait for one extra symbols to run out of Rx free pkts
				if((totalsymbolcount > (SYMBOL_NUM_DATA_CHECK)) && (NEVER_END_TEST == 0))
				{
					// Wait for ingress symbols from 2 lte slots as we have stopped recycling
					while (Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]) != (NBSYMBOL*2)) {
						asm (" IDLE");
					}
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					// Wait for all egress symbols to be recycled 
					while (Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[0]) != (NBSYMBOL*2));					
					//Stop AIF2 and proceed to verification
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					break;
				}
			}

			// disable interrupts before checking for data
			CSR&= 0xFFFFFFFE;


			monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]); // get number of received packed on first channel
			monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[0]); // channel 0 only
			printf(" Number of monolithic packets received in RX queue channel0: %d\n", monoRxCount);
			printf(" Number of monolithic packets in TX free queue for channel0: %d\n", monoTxCount);
			monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[1]);
			monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[1]);
			printf(" Number of monolithic packets received in RX queue channel1: %d\n", monoRxCount);
			printf(" Number of monolithic packets in TX free queue for channel1: %d\n", monoTxCount);

			/* Check received symbol packet data */
			for(chan =0; chan < nbchanneltotal; chan++)
			{
				idx1 = 0;
				testpass = 1;
				PEAKDEBUG= 0;
				rx_count = Qmss_getQueueEntryCount((aifObj.pktDmaConfig.rxQAxC[chan]));
				if (rx_count == 0) {testpass =0;testcheck++;}

				for (idx = 0; idx < NBSYMBOL*2; idx ++)
				{
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
					CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
					CACHE_invL1d((void *)payloadPtr, payloadLen, CACHE_WAIT);
					memcpy((void*)&rxBuffer[chan][0],(void*)payloadPtr,payloadLen);
					memcpy((void*)&rxSamples[chan][idx][0],(void*)rxBuffer[chan],payloadLen);
					if (payloadLen == (LTESYMBOLSIZE-16)) {
						DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan][CYPRENORMAL1_SIZE*4], (int16_t*)&rxBuffer[2][0]);
					}
					if (payloadLen == (LTESYMBOL2SIZE-16)) {
						DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[2][0]);
					}
					if(chan == 0)
					{
						if (!checkSingleTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ,chan)) {
								testpass = 0;testcheck++;
							rxCheck[0][idx%NBSYMBOL] = 0;
#if CPRI_RELAY_CFG != 2
							badPacket++;
#endif
						} else {
							rxCheck[0][idx%NBSYMBOL] = 1;
#if CPRI_RELAY_CFG != 2
							goodPacket++;
#endif
						}
					} else {
						if (!checkDualTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,chan)) {
							testpass = 0;testcheck++;
							rxCheck[1][idx%NBSYMBOL] = 0;
#if CPRI_RELAY_CFG != 2
							badPacket++;
#endif
						} else {
							rxCheck[1][idx%NBSYMBOL] = 1;
#if CPRI_RELAY_CFG != 2
							goodPacket++;
#endif
						}
					}
					receivedPacket[chan] += 1;
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) ptrMonoDesc);
				}
				printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
			}

			if (testpass == 1) {
				printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
			} else {
				printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
			}

			/* read the descriptor counts of the Monolithic queues. */
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txQAxC[i]);
			}
			if (value != 0) printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b1) Monolithic Packet Tx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[i]); // channel0
			}
			if (value != nbchanneltotal*NBSYMBOL*2) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[i]);
			}
			if (value != 0) printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
			value =0;
			for (i =0 ; i< nbchanneltotal; i++) {
				value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxFqAxC[i]);
			}
			if (value != nbchanneltotal*NBSYMBOL*2) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

			//UTILS_doCleanup(&aifObj,TRIG_TIMER);

			if (printexceptions == 1)
			{
			   	AIF_printException(&aifObj);
			}

#if CPRI_RELAY_CFG != 2
			printf ("received bad packets: %d \n", badPacket);
			printf ("received good packets: %d \n",goodPacket);
#endif
			printf ("total transmitted packets: %d \n",aif2SymbolEgressCount[dbgSymbol]);
			printf ("total received packets: %d \n",aif2SymbolIngressCount[dbgSymbol]);
			printf ("total received packets for channel 0: %d \n",receivedPacket[0]);
			printf ("total received packets for channel 1: %d \n",receivedPacket[1]);
			printf("\nEnding %s test\n", testObjTab[ntest].name);
 	    }
    }
#if CPRI_RELAY_RATE == 20
    printf("Test: ending AIF2 LTE TDD 20Mhz RF test \n");
#elif CPRI_RELAY_RATE == 10
    printf("Test: ending AIF2 LTE TDD 10Mhz RF test \n");
#endif

    if (aifObj.aif2EeCount.eeFlag != 0) testcheck++;

    if (testcheck == 1) {
           testcheck = 0;
           printf("All tests have passed\n");
    } else {
           printf("Some tests have failed\n");
    }

    return (0);
}

void getcomplex(Complex16* payload,uint32_t index,uint32_t tx)
{
	float freq;
	Complex16 x1;
	Complex16 y;
	if (tx==0)
	{
                // 1 MHz signal
                freq = FREQ;
                x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                y = x1;
	} else if (tx==1)
	{
                Complex16         x2;
                // 1 MHz signal
                freq = FREQ;
                x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // 2 MHz signal
                freq = 2*FREQ;
                x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // dual tone signal
                y.re = x1.re + x2.re;
                y.im = x1.im + x2.im;
	}
                payload->re = y.re;
                payload->im = y.im;
}

//This function is used to select the symbol we need to push or not regarding the symbol number starting from the frame boundary.
void UTILS_setLteTddDlBitmap(AIF2_LteTddUlDlCfg tddSubFrameBitMap, AIF2_LteTddSsfNcpCfg tddSpecSubFrameBitMap)
{
	int32_t k, j;
	uint32_t tddSpecSubFrameBitMap2;

	for(k=0;k<144;k++)
		egressSymEnable[k]= 1; //enable all symbols by default

	for (k=0;k<10;k++)
	{
		// DL SF
		if ((tddSubFrameBitMap & 0x00000003) == 0x00000000) {
			for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
			{
				egressSymEnable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 0;
			}
		}
		// UL SF
		if ((tddSubFrameBitMap & 0x00000003) == 0x00000003) {
			for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
			{
				egressSymEnable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 1;
			}
		}
		// Special SF
		if ((tddSubFrameBitMap & 0x00000003) == 0x00000001) {
			tddSpecSubFrameBitMap2 = tddSpecSubFrameBitMap;
			for (j=0;j<(AIF2_LTE_SYMBOL_NUM*2);j++)
			{
				if ((tddSpecSubFrameBitMap2 & 0x00000003) == 0x00000003) {
					egressSymEnable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 1;
				} else {
					egressSymEnable[j+AIF2_LTE_SYMBOL_NUM*2*k] = 0;
				}
				tddSpecSubFrameBitMap2>>=2;
			}
		}
		tddSubFrameBitMap>>=2;
	}
	/*for(k=0;k<7;k++)
		egressSymEnable[k]= 1; //enable all symbols by default*/

}

////////////

