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
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

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

#include <DSP_fft16x16.h>
#include <gen_twiddle_fft16x16.h>
#include <gen_twiddle_fft16x16.c>


#include "cslUtils.h"
#include "mnavUtils.h"
#include "mathUtils.h"

#if EVM_TYPE == 5 || EVM_TYPE == 7
#include "appletonScbpSync.h"
#include "RTWP.h"
#endif

/* TEST DESCRIPTION
 * This CPRI WCDMA Check RF test is designed to work either in internal loopback mode, or with a dual EVM setup connected with a break-out card, or with a
 * CPRI Relay setup (tci6614 evm + scbp + tsw3726), or with a Scbp/Tsw3726 standalone setup.
 * The default project configurations in the pdk-generated workspace are supporting:
 * - EVM_LBACK: test compiled for internal loopback mode. Can be used with device simulators as well.
 * - EVM6614: test compiled for CPRI Relay setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - SCBP_DSP1: test compiled for Scbp/Tsw3726 standalone setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - EVM6670_DSP1 and EVM6670_DSP2: test compiled for first and second EVMs of a dual EVM setup connected with a break-out card
 *
 * This CPRI WCDMA Check RF test runs on a single link of AIF2 and configures 2 AxCs. Both AxCs IQ data are pure single or dual tone sinewaves at given frequencies,
 * such that upon reception of Wcdma frames for both antennas, IQ data can be checked with fft software. If the signal doesn't go from digital to analog and back, it
 * can also be checked bitwise.
 *
 * Pre-processing constants of interest:
 * - CPRI_RELAY_CFG: Set it to 1 as default. Set to 2 if testing towards tsw3726 RF board.
 * - EVM_TYPE: 1 - internal loopback or EVM DSP1 (dual EVM setup), 2 - EVM DSP2 (dual EVM setup), 3 - SCBP DSP1, 5 - EVM6614 (Cpri Relay setup), (0 - old Lyrtech C6670 EVM)
 *             6 - KeyStone-II internal loopback (EVM), 7 - EVM6638K2K (Cpri Relay setup) 
 * - LOOPBACK: selects internal loopback mode (SerDes level, AIF2 SW trigger)
 *
 * Note on usage with CPRI relay setup:
 * 		0. FPGA firmware upgrades:
 *          a. Scbp board must be upgraded to the proper SCBP FPGA WCDMA version .
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
 * Note on AIF2 DIO buffers: users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
 * The buffer (dio_data and dio_result) are circular buffers for IQ samples.
 * By default, the AIF2 LLD expects the IQ data to be organized like: (1QW = 16 bytes, address increments from left to right)
 * AIF2 LLD DL buffer format for a given number of blocks
 * | 1QW AxC0   |  1QW AxC1  |  1QW AxC0  |  1QW AxC1   | ... | 1QW AxC0   |  1QW AxC1  | 1QW AxC0   | 1QW AxC1   |
 * -------------- wrap 1 ---->
 * -----------------------------------wrap 2 (end of circular buffer)--------------------------------------------->
 * For each burst (if smaller than wrap 1), address increments by DmaBurstAddrStride
 * For each DmaNumBlock (wrap 1), address increments by DmaBlockAddrStride
 *
 * AdDioSetup.EgrDioEngine[i].NumQuadWord        = AIF2FL_AD_1QUAD;//Use 1 QW per channel
 * AdDioSetup.EgrDioEngine[i].DmaBlockAddrStride = hAif->dioConfig[i].numPeDBCH; // 2 channels enabled in this test
 * AdDioSetup.EgrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
 * AdDioSetup.EgrDioEngine[i].DmaBurstAddrStride = 4;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
 *
 * wrap 1 = NumQuadWord * 2 AxCs = 2 QWs, so DmaBurstAddrStride (4QW) doesn't play any role here. Both AxCs are contiguous in memory.
 * DmaBlockAddrStride = 2 QWs, so address increments linearly until the end of the circular buffer: wrap 2 = DmaNumBlock * wrap1
 * It should be noted that DmaNumBlock (aifObj.dioConfig[i].outNumBlock in AIF2 LLD object) is limited to 8191, which limits the
 * size of circular buffer.
 *
 * AIF2 LLD UL buffer format
 * | 1QW AxC0S0 | 1QW AxC0S1 | 1QW AxC1S0 |  1QW AxC1S1 | ... | 1QW AxC0S0 | 1QW AxC0S1 | 1QW AxC1S0 | 1QW AxC1S1 |
 * AdDioSetup.IngrDioEngine[i].NumQuadWord        = AIF2FL_AD_2QUAD;//Use 2 QW per channel in UL
 * AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = 2 * (hAif->dioConfig[i].numPdDBCH);
 * AdDioSetup.IngrDioEngine[i].DmaBurstLen        = AIF2FL_AD_4QUAD;//1 max QW burst per one transfer
 * AdDioSetup.IngrDioEngine[i].DmaBurstAddrStride = 4;//DMA burst stride to 1 (wrap1)if 1QW and 4 for 4QW for DMABurstLen
 *
 * ******************************************** FOR CHECK RF **********************************************************
 * However, for the purpose of this RF check tests, we need to separate the AxCs in lower and upper part of the circular buffers
 * DL buffer format for a given number of blocks
 * | 1QW AxC0   |  1QW AxC0  |  1QW AxC0  |  1QW AxC0   | ... | 1QW AxC1   |  1QW AxC1  | 1QW AxC1   | 1QW AxC1   | ...
 * AdDioSetup.EgrDioEngine[i].DmaBlockAddrStride = 1;
 * AdDioSetup.EgrDioEngine[i].DmaBurstAddrStride = 2400; (in QWs)
 * AdDioSetup.EgrDioEngine[i].NumQuadWord        = AIF2FL_AD_1QUAD;
 * AdDioSetup.EgrDioEngine[i].DmaBurstLen        = AIF2FL_AD_1QUAD;
 *
 * wrap 1 = 2 QWs, DmaBurstLen = 1 QW, so in between each AxC for each burst, address increments by DmaBurstAddrStride (38400)
 * aifObj.dioConfig[0].outNumBlock = 2400, so buffer size (wrap2) is 2400 * 2QWs = 76800 bytes or 19200 chips (16-bit I/Q)
 * So this corresponds to half a UMTS frame for both AxCs, hence 1/4 th of a frame.
 *
 *
 * UL buffer format
 * | 1QW AxC0S0 | 1QW AxC0S1 | 1QW AxC0S0 |  1QW AxC0S1 | ... | 1QW AxC1S0 | 1QW AxC1S1 | 1QW AxC1S0 | 1QW AxC1S1 | ...
 * AdDioSetup.IngrDioEngine[i].DmaBlockAddrStride = 2;
 * AdDioSetup.IngrDioEngine[i].DmaBurstAddrStride = 2400;
 * AdDioSetup.IngrDioEngine[i].DmaBurstLen = AIF2FL_AD_2QUAD;
 * AdDioSetup.IngrDioEngine[i].NumQuadWord = AIF2FL_AD_2QUAD;
 *
 * for uplink, we work at the granularity of 2 QWs, and the computations need to take this into account
 *
 * ********************************************************************************************************************
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Constant definitions

#define     TRIG_TIMER          0 	// Used to start AIF2 timers, Frame sync with one shot pulse - could be any timer
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define     SYNC_TIMER          8	// Used to call the SW PLL periodically - Cpri relay setup only - could be an OS PRD thread as well
#else
#define     SYNC_TIMER          5	// Used to call the SW PLL periodically - Cpri relay setup only - could be an OS PRD thread as well
#endif
#define     PLL_TIMER           6	// Used by the SW PLL for adjustments against the external 16Hz clock - Cpri relay setup only - could be any timer

#define 	_AIF2FL_DUMP		0	// Enable the DUMP of the AIF configuration, for debug purposes

//#define     OBSAI

//Value for Sin calculation
#if CPRI_RELAY_CFG == 2
#define     Q_FACTOR            8191
#else
#define     Q_FACTOR            512		// When the signal stays digital, reducing Q factor to avoid overflow in fft software
#endif
#define     SAMP_FREQ_FLOAT     			3.84  	// Wcdma rate
#define     PI                  			3.14159265358979323846
#define     CPRIRELAY_CONFIG_NUM            2

#if EVM_TYPE == 0 || EVM_TYPE == 3
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM , the Frequency is based on an external 153.6 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define		DUAL_CARRIER_SUPPORT 1								// Enabled - requires dual carrier firmware
#define     DIO_0               AIF2FL_DIO_ENGINE_0 			// selects AIF2 DIO engine to be used - 0 is default
#ifndef OBSAI
#define 	NUM_AxC 			4								// Defines the number of Wcdma AxCs to be used in this test
#else
#define 	NUM_AxC 			2								// Defines the number of Wcdma AxCs to be used in this test
#endif
#define		DIO_DMA_QW			4								// Num of 32-bit words in one quadword
#define		DIO_DMA_NUM_BLOCKS	2400							// selects the size of the dio circular buffers in quadwords x NUM_AxC
#define		DIO_BUFFER_SIZE_AXC	(DIO_DMA_NUM_BLOCKS*DIO_DMA_QW)	// Computes the size of buffer for one AxC in 32-bit words
#define		DIO_DMA_BURSTSTRIDE	(DIO_BUFFER_SIZE_AXC/DIO_DMA_QW)// Computes the address increment for each DIO burst so that AxCs are de-interleaved in DIO send/receive buffers
#define 	FFT_SIZE	   		2048							// size of FFT that is used to check incoming Rx signal
#define 	FREQ				0.1								// Single tone frequency of Tx1 pure sine wave
#define 	PEAK_LOC_1MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ)  // Used for peak detection after fft on Rx symbols
#define 	PEAK_LOC_2MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ*2)// AxC0, AxC1 single tone and AxC2, AxC3 dual tone
#define		N   				(2048)							// Used for twiddle factor generation
#define 	PAD 				(16)							// Used for twiddle factor generation
#define 	RX_SUCCESS_CNT		(800*NUM_AxC)					// Define a number of consecutive runtime RF check success prior to monitoring runtime failures
#ifndef OBSAI
#if CPRI_RELAY_CFG == 2
#define		FRAME_NUM_DATA_CHECK 7500							//calculated from the LTE RF test (150000 slots = 7500 frames).
#else
#define		FRAME_NUM_DATA_CHECK 500							// random value, Loopback test no need to wait for Radio On to be active (Cpri relay setup).
#endif
#else
#define		FRAME_NUM_DATA_CHECK 500							// random value, Loopback test no need to wait for Radio On to be active (Cpri relay setup).
#endif
#define		DISABLE_AXC_TRANSMISSION	0						// Can be used to test AIF2LLD AIF_disablePeCh() APIs - Once disabled, zeros are inserted for this particular AxC
#define		DISABLE_AXC_RECEPTION		0						// Can be used to test AIF2LLD AIF_disablePeCh() APIs - Once disabled, non-zero stale data is kept for this particular AxC

//////////////////////////////////////////////////////////////////////////////
// Typedefs

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


typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;

//////////////////////////////////////////////////////////////////////////////
// Global variables

AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

#if EVM_TYPE == 3
uint32_t NEVER_END_TEST=1; 									// Enable the infinite loop for visual check when running on SCBP-TSW standalone setup
#else
uint32_t NEVER_END_TEST = 0; 									// Test stops after a given number of Wcdma frames
#endif

volatile unsigned int ntest         = CPRI_RELAY_CFG - 1; 	// used for running multiple tests
volatile unsigned int testcheck     = 1;					// reports pass fail at the end of the test
#ifdef LOOPBACK
volatile unsigned int intLoopback   = 1; 					// AIF2 SW trigger
volatile unsigned int swSync        = 1;					// SerDes loopback mode
#else
volatile unsigned int intLoopback   = 0;
#if EVM_TYPE == 3
volatile unsigned int swSync        = 1;
#else
volatile unsigned int swSync        = 0;
#endif
#endif

volatile TestObj                     testObjTab[CPRIRELAY_CONFIG_NUM] = {
	{//CPRI_RELAY_CFG = 1 or DL traffic configuration (DSP_2)
#ifndef OBSAI
      "CPRI_RELAY_DL_TEST", // test name
#else
      "OBSAI_DL_TEST", // test name
#endif
      // link0          link1          link2          link3          link4          link5
      {1,             0,             0,             1,             0,             0            },  // link enable
      {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // dio engine
	},
	{//CPRI_RELAY_CFG = 2 or CPRI relay configuration (DSP_2) - used when exercising RF path
#ifndef OBSAI
      "WCDMA_CPRI_RELAY_SW", // test name
#else
      "WCDMA_OBSAI_SW", // test name
#endif
      // link0          link1          link2          link3          link4          link5
      {1,             0,             0,             1,             0,             0            },  // link enable
      {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_UL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_8,  DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_UL,  DATA_TYPE_DL,  DATA_TYPE_UL,  DATA_TYPE_UL },  // inboundDataType
	  {DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_8,  DATA_WIDTH_16, DATA_WIDTH_8,  DATA_WIDTH_8},   // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // dio engine
	}
   };

/*
 *	Variables used fft software
 */
#pragma DATA_ALIGN(twiddleFacts, 8);
int16_t   twiddleFacts[2*N + 2*PAD];

/*
 *	Buffers used to send and receive antenna IQ samples
 *	To be kept in LL2 to enable HW cache coherency
 */
#pragma DATA_SECTION (dio_data, ".dioData");
#pragma DATA_ALIGN (dio_data, 16);
uint32_t   dio_data[NUM_AxC][DIO_BUFFER_SIZE_AXC];
#pragma DATA_SECTION (dio_result, ".dioData");
#pragma DATA_ALIGN (dio_result, 16);
uint32_t   dio_result[NUM_AxC][DIO_BUFFER_SIZE_AXC];

/*
 * Variables used for RF checks
 */
#pragma DATA_ALIGN (rxBuffer, 16);
uint32_t  rxBuffer[2][FFT_SIZE];		// Used by fft software
uint8_t 	rxCheck[NUM_AxC];			// Keeps track of current check of all AxCs
uint8_t 	rxCheckCnt=0;				// Counter
uint16_t   rxRuntimeSuccess=0;			// Counter
uint16_t   rxRuntimeFail[NUM_AxC];		// Keeps track of the number of failures per AxC

/*
 * Buffers storing RX samples for debug and verification
 */
#pragma DATA_ALIGN (rxSamples, 16);
uint32_t   rxSamples[FFT_SIZE];


/*
 * Flag used for printing out configuration or exceptions at the end of the test.
 * Please note that exceptions are always enabled during this test.
 * Having AIF2 exceptions reported during this test should be considered as a test failure.
 *
 */
uint32_t startupprint = 1;
uint32_t printexceptions = 1;
uint32_t enableexceptions = 1;

// link configured at test init time
uint32_t   activeLink;

//////////////////////////////////////////////////////////////////////////////
// References to external variables

extern uint8_t		   keepStartupDelay;
extern uint32_t		   thresholdDividerDualTone;

//////////////////////////////////////////////////////////////////////////////
// Function prototypes
void frameIsr();
void getcomplex(Complex16* payload, uint32_t index, uint32_t chan);
#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

//////////////////////////////////////////////////////////////////////////////
// Function bodies

void frameIsr()
{
	uint32_t chan;
	int32_t  rfcheck;

	for (chan=0;chan<aifObj.linkConfig[activeLink].numPdAxC;chan++)
	{
#if CPRI_RELAY_CFG == 2
		BufferConvertion(dio_result[chan], rxBuffer[0], FFT_SIZE);
#else
		memcpy(&rxBuffer[0][0], &dio_result[chan][0], (FFT_SIZE*4));
#endif
		memcpy(&rxSamples[0], &rxBuffer[0][0], (FFT_SIZE*4));
		DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[0][0], (int16_t*)&rxBuffer[1][0]);
		if ( chan==0 || chan==1) {
			rfcheck = checkSingleTone((int16_t*)&rxBuffer[1][0],FFT_SIZE,PEAK_LOC_1MHZ,chan);
		} else if ( chan==2 || chan==3) {
			rfcheck = checkDualTone((int16_t*)&rxBuffer[1][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,chan);
		}
		if (rfcheck)
		{
			rxCheck[chan] = 1;
			if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
		} else {
			rxCheck[chan] = 0;
			if (rxRuntimeSuccess == RX_SUCCESS_CNT)
				rxRuntimeFail[chan]++;
		}
	}
	rxCheckCnt++;

}

void getcomplex(Complex16* payload, uint32_t index, uint32_t chan)
{

	Complex16 x1;
	Complex16 y;
	if (chan == 0 || chan == 1)
	{
		float freq;
		freq = FREQ;
		x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		y = x1;
	} else if (chan == 2 || chan == 3)
	{
		float freq;
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

int main(void)
{

    uint32_t idx, i, numBlock, numWord, chan, frameCurrent;
    uint16_t testpass = 0, myboard = EVM_TYPE;
    int32_t rfcheck;
#if EVM_TYPE == 3
    PllcHwSetup pllc_hwSetup;
#endif
#ifdef SIMULATOR_SUPPORT
	printf ("\n Error: This example code does not run on the simulator.\n\n");
	return(0);
#endif

#ifndef OBSAI
#if CPRI_RELAY_CFG == 0 || CPRI_RELAY_CFG == 1
    printf("Beginning AIF2 WCDMA CPRI relay Software testing\n");
#else
    printf("Starting AIF2 WCDMA CPRI relay Software\n");
#endif
#else
    printf("Beginning AIF2 WCDMA OBSAI Software testing\n");
#endif

    UTILS_waitForHw(100000);

    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
    else if (myboard <= 2)  DSP_procId = EVM_TYPE; // For Advantech: myboard  = 1 (DSP_1) or 2 (DSP_2)
    else if (EVM_TYPE == 6) DSP_procId = 1;
    else if (EVM_TYPE == 7) DSP_procId = 1;
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

	fsevt1_userIsr = frameIsr;

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	memset(&aifObj, 0, sizeof(aifObj));
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
#ifndef OBSAI
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
#else
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_OBSAI;
#endif
	aifObj.pktdmaOrDioEngine  = AIF2FL_DIO;
	aifObj.mode               = AIF_WCDMA_MODE;
	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
	else
		aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

	numBlock = DIO_DMA_NUM_BLOCKS;
	numWord  = DIO_DMA_QW;

#if EVM_TYPE == 3
	// disable link 3 on DSP_1
	testObjTab[ntest].linkEnable[3]  = 0;
	activeLink = 0;
#else
	testObjTab[ntest].linkEnable[0]  = 0;
	activeLink = 3;
#endif

#ifdef USE_SMA
	// disable link 0 on DSP_1
	testObjTab[ntest].linkEnable[0]  = 0;
	testObjTab[ntest].linkEnable[3]  = 0;
	testObjTab[ntest].linkEnable[4]  = 1;
	activeLink = 4;
#endif

	// Dio parameters for engine 0 - used for validation purposes on DSP_1
	aifObj.dioConfig[0].out          = UTILS_local2GlobalAddr(&dio_data[0][0]);
	aifObj.dioConfig[0].in           = UTILS_local2GlobalAddr(&dio_result[0][0]);
	aifObj.dioConfig[0].outNumBlock  = numBlock;
#if CPRI_RELAY_CFG == 2
	aifObj.dioConfig[0].inNumBlock	 = numBlock/2;
#else
	aifObj.dioConfig[0].inNumBlock	 = numBlock;
#endif
#if EVM_TYPE == 5 || EVM_TYPE == 7
	aifObj.linkConfig[activeLink].nodeTx      = 0;
	aifObj.linkConfig[activeLink].nodeRx      = 2;
#endif

	printf("Software configuration: %s\n", testObjTab[ntest].name);

	// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
	UTILS_doCleanup(&aifObj,TRIG_TIMER);

	if (DSP_procId == 1)
	{
		UTILS_initTimer(TRIG_TIMER);
	}

	// initialization function for the AT timer and EDMA3 ISRs
	UTILS_aifIntcSetup();

	/* For SCBP-Appleton EVM run, we'll now proceed with SW clock sync */
#if EVM_TYPE == 5
	UTILS_boardsSync(SYNC_TIMER,PLL_TIMER); // required board sync process when using CPRI relay setups
#endif
	/* No board sync on Hawking, still we want to configure RTWP for Wcdma */
#if EVM_TYPE == 7
	spi_init();
	RTWP_open(activeLink, 1.0, 1.0);
	UTILS_triggerBoardsSync(SYNC_TIMER); // RTWP reads will occurs every 10ms on timer8 cpu interrupt
#endif
	
	// Initialize all link strutures based on the test objects
	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
		 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
		 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_3P84MHZ;
		 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
		 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
		 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
		 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
		 aifObj.linkConfig[i].psMsgEnable        = 0;
		 if (intLoopback == 1 ) aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
		  else	 aifObj.linkConfig[i].comType            = AIF2_2_AIF2;
		 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i];
	}

	if ((DSP_procId == 2) && (startupprint == 1)) UTILS_printLinkConfig(&aifObj);
	startupprint = 0;

	// leave some time - might not be necessary in the end
	UTILS_waitForHw(1000);

	aifObj.linkConfig[activeLink].numPeAxC = NUM_AxC;
	aifObj.linkConfig[activeLink].numPdAxC = NUM_AxC; // assuming Tx and RX diversity to be default

#if CPRI_RELAY_CFG == 2
	aifObj.linkConfig[activeLink].outboundDataType  = DATA_TYPE_DL;
	aifObj.linkConfig[activeLink].outboundDataWidth = DATA_WIDTH_16;
	aifObj.linkConfig[activeLink].inboundDataType   = DATA_TYPE_UL;
	aifObj.linkConfig[activeLink].inboundDataWidth  = DATA_WIDTH_8;
	aifObj.linkConfig[activeLink].pe2Offset    		= (320 - 60);
#endif

	// compute parameters given this AIF configuration
	AIF_calcParameters(&aifObj);

	// initialization function for qmss and cppi low-level drivers
	UTILS_initQmss((uint32_t*)NULL, 0, 0, 0, NULL);

	// initialization function for the AIF2 H/W of the TMS320TCI6616
	AIF_initDio(&aifObj);
	AIF_initPktDma(&aifObj);
	AIF_initHw(&aifObj);

	memset(dio_result, 0xFF, NUM_AxC * numBlock * numWord * 4);
	memset(rxBuffer, 0x00, sizeof (rxBuffer));

	/*
	 *
	 *
	 */
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[0].DmaBlockAddrStride = 1;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[0].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[0].NumQuadWord 		= AIF2FL_AD_1QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[0].DmaBurstLen 		= AIF2FL_AD_1QUAD;
#if CPRI_RELAY_CFG == 2
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBlockAddrStride = 2;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstLen 		 = AIF2FL_AD_2QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].NumQuadWord 		 = AIF2FL_AD_2QUAD;
#else
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBlockAddrStride = 1;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstLen 		 = AIF2FL_AD_1QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].NumQuadWord		 = AIF2FL_AD_1QUAD;
#endif

	// Disabling this mode, so that, on Cpri relay setup, once AIF2 timers are triggered, they keep running based on the initial pulse. Note there isn't a periodic 10ms pulse generated from EVMTCI6614, just a one shot pulse.
	aifObj.hAif2Setup->commonSetup->pAtCommonSetup->AutoResyncMode = AIF2FL_NO_AUTO_RESYNC_MODE;

	/* Initialize Tx IQ samples for each of the AxCs */
	for(chan=0;chan<NUM_AxC;chan++)
	{
		for (idx = 0; idx < (numBlock/2 * NUM_AxC * numWord); idx++)
		{
			getcomplex((Complex16*) &dio_data[chan][idx],idx,chan);
		}
		CACHE_wbInvL1d((void *)(dio_data[chan]), (numBlock/2 * NUM_AxC * numWord)*4, CACHE_WAIT);
	}

	/* Fft software initialization */
	gen_twiddle_fft16x16(&twiddleFacts[PAD], FFT_SIZE);
	memset(rxCheck,0x00,sizeof(rxCheck));
	memset(rxRuntimeFail,0x00,sizeof(rxRuntimeFail));
#if CPRI_RELAY_CFG == 2
	// In case of 8-bit IQ sample resolution, lower the threshold divider for proper peak detection on dual tone signal
	thresholdDividerDualTone = 3;
#endif

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

	// configure AIF2 HW now that all required CSL structures have been filled
	AIF_startHw(&aifObj);

	if (DSP_procId == 1)
	{
		//Trig the SCBP by sending a rising edge through timer0 on appleton side connected to PHYSYNC (SCBP side)
		keepStartupDelay = 0;
		UTILS_triggerFsync(&aifObj);
	}

	// enable exceptions if required after some Wcdma frames
	if (enableexceptions == 1) {
		while (aifFsyncEventCount[1] <= 10) asm("    IDLE");
		AIF_enableException(&aifObj);
	}

	/* Main IDLE LOOP - monitoring the end of the test, if any. */
	if (NEVER_END_TEST == 0) {
		while (aifFsyncEventCount[1] <= FRAME_NUM_DATA_CHECK) {
			asm("    IDLE");
		}
	} else {
		while (1) {
			asm("    IDLE");
		}
	}

	UTILS_aif2ExceptIntDisable();

	// wait 40ms before resetting AIF2
	frameCurrent = aifFsyncEventCount[1];
	while (aifFsyncEventCount[1] <= (4+frameCurrent)) {
	    asm("    IDLE");
	}

#if DISABLE_AXC_TRANSMISSION == 1
	AIF_disablePeCh(&aifObj,aifObj.linkConfig[activeLink].numPdAxC-1);
	frameCurrent = aifFsyncEventCount[1]; // wait one frame
	while (aifFsyncEventCount[1] <= (1+frameCurrent)) {
	    asm("    IDLE");
	}
#endif
#if DISABLE_AXC_RECEPTION == 1
	AIF_disablePdCh(&aifObj,aifObj.linkConfig[activeLink].numPdAxC-1);
	frameCurrent = aifFsyncEventCount[1]; // wait one frame
	while (aifFsyncEventCount[1] <= (1+frameCurrent)) {
	    asm("    IDLE");
	}
#endif
#if DISABLE_AXC_TRANSMISSION == 1 || DISABLE_AXC_RECEPTION == 1
	aifObj.linkConfig[activeLink].numPdAxC--;
#endif

	if (enableexceptions == 1) {
		// disable interrupts before checking for data
		CSR&= 0xFFFFFFFE;
	}

	if (printexceptions == 1) {
		AIF_printException(&aifObj);
	}


	UTILS_doCleanup(&aifObj,TRIG_TIMER);

#if DUAL_CARRIER_SUPPORT == 0
	for (chan=0;chan<2;chan++)
#else
	for (chan=0;chan<aifObj.linkConfig[activeLink].numPdAxC;chan++)
#endif
	{
		CACHE_invL1d((void *)dio_result[chan], (FFT_SIZE*4), CACHE_WAIT);
#if CPRI_RELAY_CFG == 2
		BufferConvertion(dio_result[chan], rxBuffer[0], FFT_SIZE);
#else
		memcpy(&rxBuffer[0][0], &dio_result[chan][0], (FFT_SIZE*4));
#endif
		DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[0][0], (int16_t*)&rxBuffer[1][0]);
		if ( chan==0 || chan==1) {
			rfcheck = checkSingleTone((int16_t*)&rxBuffer[1][0],FFT_SIZE,PEAK_LOC_1MHZ,chan);
		} else if ( chan==2 || chan==3) {
			rfcheck = checkDualTone((int16_t*)&rxBuffer[1][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,chan);
		}
		if (!rfcheck)
		{
			rxCheck[chan] = 0;
			testpass = 1;
		} else {
			rxCheck[chan] = 1;
		}
	}
#if DISABLE_AXC_TRANSMISSION == 1 && CPRI_RELAY_CFG == 1
	memset(&rxBuffer[0][0], 0x00, (FFT_SIZE*4));
	rfcheck = memcmp(&rxBuffer[0][0], &dio_result[aifObj.linkConfig[activeLink].numPdAxC][0], (FFT_SIZE*4));
	if (rfcheck != 0)
	{
		rxCheck[aifObj.linkConfig[activeLink].numPdAxC] = 0;
		testpass = 1;
	} else {
		rxCheck[aifObj.linkConfig[activeLink].numPdAxC] = 1;
		rxRuntimeFail[aifObj.linkConfig[activeLink].numPdAxC] = 0; // reset runtime check since we disabled transmission on that AxC
	}
#endif

#if DISABLE_AXC_RECEPTION == 1 && CPRI_RELAY_CFG == 1
	rxRuntimeFail[aifObj.linkConfig[activeLink].numPdAxC] = 0; // reset runtime check since we disabled reception was cut on that AxC
#endif

	if (testpass != 0) {
#ifndef OBSAI
		printf(" DIO CPRI Data for WCDMA AxC0 or AxC1 or AxC2 or AxC3: FAIL\n");
#else
		printf(" DIO OBSAI Data for WCDMA: FAIL\n");
#endif
		testcheck++;
	}

	if (rxRuntimeFail[0] != 0)
	{
		printf("AxC0: Run time peak detection failed\n");
		testcheck++;
	}

	if (rxRuntimeFail[1] != 0)
	{
		printf("AxC1: Run time peak detection failed\n");
		testcheck++;
	}

#if DUAL_CARRIER_SUPPORT == 1
	if (rxRuntimeFail[2] != 0)
	{
		printf("AxC2: Run time peak detection failed\n");
		testcheck++;
	}

	if (rxRuntimeFail[3] != 0)
	{
		printf("AxC3: Run time peak detection failed\n");
		testcheck++;
	}
#endif

	if (testpass == 0) {
		if (DSP_procId == 1) {
#ifndef OBSAI
			printf("Test success: stopping AIF2 WCDMA CPRI relay test\n");
#else
			printf("Test success: stopping AIF2 WCDMA OBSAI test\n");
#endif
		}
		if (DSP_procId == 2) {
			printf("Ending AIF2 WCDMA CPRI relay Software\n");
		}
	}
	
    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed\n");
    } else {
       printf("Some tests have failed\n");
    }
	
	return(0);

}


////////////////////////////////////////////////////////////////////////////////////////
