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

// SPECIAL_SEQUENCE of 1 only used for extensive RF tests on CPRI relay setups, only SPECIAL_SEQUENCE=0 tested upon AIF2 LLD release
#ifndef		SPECIAL_SEQUENCE
#define     SPECIAL_SEQUENCE    0
#endif
// NB_LINKS, only NB_LINKS = 1 tested upon AIF2 LLD release
#ifndef		NB_LINKS
#define     NB_LINKS    1
#endif

// RF test vectors for RF verification
#if SPECIAL_SEQUENCE == 1
#if LTE_RATE == 20
#if DFE_RATE == 20
#include "LTEFDD_rfverif_20M_30_72.c"
#elif DFE_RATE == 10
#include "LTEFDD_rfverif_10M_30_72.c"
#endif
#elif LTE_RATE == 10
#if DFE_RATE == 10
#include "LTEFDD_rfverif_10M_15_36.c"
#endif
#elif LTE_RATE == 5
#if DFE_RATE == 5
#include "LTEFDD_rfverif_5M_7_68.c"
#endif
#endif
#endif
#if SPECIAL_SEQUENCE == 2
#if LTE_RATE == 20
#include "LTEFDD_TM3p1_20M_CellID1.c"
#elif LTE_RATE == 10
#include "LTEFDD_TM3p1_10M_CellID1.c"
#elif LTE_RATE == 5
#include "LTEFDD_TM3p1_5M_CellID1.c"
#endif
#endif
#if  SPECIAL_SEQUENCE == 3
#if LTE_RATE == 20
#include "LTEFDD_TM1p1_20M_CellID1.c"
#elif LTE_RATE == 5
#include "LTEFDD_TM1p1_5M_CellID1.c"
#endif
#endif
#if  SPECIAL_SEQUENCE == 4
#ifndef TM_3_1
#include "LTETDD_1x20_64QAM_PN9_ID1.c"
#else
#include "LTETDD_TM3p1_20M_CellID1_cfg3.c"
#endif
#endif
/* TEST DESCRIPTION
 * This CPRI LTE Check RF test is designed to work either in internal loopback mode, or with a dual EVM setup connected with a break-out card, or with a
 * CPRI Relay setup (tci6614 evm + scbp + tsw3726), or with a Scbp/Tsw3726 standalone setup.
 * The default project configurations in the pdk-generated workspace are supporting:
 * - EVM_LBACK: test compiled for internal loopback mode. Can be used with device simulators as well.
 * - EVM6614: test compiled for CPRI Relay setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - SCBP_DSP1: test compiled for Scbp/Tsw3726 standalone setup and run in RF loopback mode (rx1/rx2 and tx1/tx2 on tsw3726 are connected together)
 * - EVM6670_DSP1 and EVM6670_DSP2: test compiled for first and second EVMs of a dual EVM setup connected with a break-out card
 *
 * This CPRI LTE Check RF test runs on a single link of AIF2 and configures 2 AxCs. Both AxCs IQ data are pure single or dual tone sinewaves at given frequencies,
 * such that upon reception of LTE symbols for both antennas, IQ data can be checked with fft software. If the signal doesn't go from digital to analog and back, it
 * can also be checked bitwise.
 *
 * Pre-processing constants of interest:
 * - SUPERPACKET: must be defined if using early versions (<2.0, note most EVMs are equipped with 1.0, correcting Advisory 12) of c6670. Not needed on tci6614
 * - CPRI_RELAY_CFG: Set it to 1 as default. Set to 2 if testing towards tsw3726 RF board.
 * - LTE_RATE: selects the LTE rate to be used. Values: 5, 10, or 20
 * - EVM_TYPE: 1 - KeyStone-I internal loopback or EVM DSP1 (dual EVM setup), 2 - EVM DSP2 (dual EVM setup), 3 - SCBP DSP1, 5 - EVM6614 (Cpri Relay setup), (0 - old Lyrtech C6670 EVM)
 *             6 - KeyStone-II internal loopback (EVM), 7 - EVM6638K2K (Cpri Relay setup), 8 - EVM6638K2K (Drboc+Marconi setup)
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
#ifndef SAMP_FREQ_FLOAT
#if LTE_RATE == 20
#define     SAMP_FREQ_FLOAT     30.72 	// LTE20 sample rate
#elif LTE_RATE == 10
#define     SAMP_FREQ_FLOAT     15.36	// LTE10 sample rate
#elif LTE_RATE == 5
#define     SAMP_FREQ_FLOAT     7.68	// LTE5 sample rate
#elif LTE_RATE == 40
#define     SAMP_FREQ_FLOAT     61.44	// LTE5 sample rate
#else
#error 		"LTE_RATE not valid."
#endif
#endif
#define     PI                  3.14159265358979323846



#define     TEST_NUM            2	// Number of test configurations
#define 	_AIF2FL_DUMP		0	// Enable the DUMP of the AIF configuration, for debug purposes
#if LTE_RATE == 20
#define     NBCHANNELMAXPERLINK 2	// Max number of AxCs per link
#elif LTE_RATE == 10
#define     NBCHANNELMAXPERLINK 4	// Max number of AxCs per link
#elif LTE_RATE == 5
#define     NBCHANNELMAXPERLINK 8	// Max number of AxCs per link
#elif LTE_RATE == 40
#define     NBCHANNELMAXPERLINK 1	// Max number of AxCs per link
#endif
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3
#define     NBCHANNELPERLINK	NBCHANNELMAXPERLINK	// Number of AxCs in use per link (fixed to max for this rf test)
#else
#if LTE_RATE == 40
#define     NBCHANNELPERLINK	1	                // Number of AxCs in use per link (fixed to 1)
#else
#define     NBCHANNELPERLINK	2	                // Number of AxCs in use per link (fixed to 2)
#endif
#endif
#define     NBSYMBOL			AIF2_LTE_SYMBOL_NUM	// Number of symbols per slot
#if LTE_RATE == 20
#define		LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#define		FFT_SIZE			AIF2_LTE20_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE20_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE20_CYPRENORMAL_SIZE
#elif LTE_RATE == 10
#define		LTESYMBOLSIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)    
#define 	LTESYMBOL2SIZE		((AIF2_LTE10_FFT_SIZE+AIF2_LTE10_CYPRENORMAL_SIZE)*4 + 16)
#define		FFT_SIZE			AIF2_LTE10_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE10_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE10_CYPRENORMAL_SIZE
#elif LTE_RATE == 5
#define		LTESYMBOLSIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((AIF2_LTE5_FFT_SIZE+AIF2_LTE5_CYPRENORMAL_SIZE)*4 + 16)
#define		FFT_SIZE			AIF2_LTE5_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE5_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE5_CYPRENORMAL_SIZE
#elif LTE_RATE == 40
#define		LTESYMBOLSIZE		((AIF2_LTE40_FFT_SIZE+AIF2_LTE40_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((AIF2_LTE40_FFT_SIZE+AIF2_LTE40_CYPRENORMAL_SIZE)*4 + 16)
#define		FFT_SIZE			AIF2_LTE40_FFT_SIZE
#define		CYPRENORMAL1_SIZE	AIF2_LTE40_CYPRENORMAL1_SIZE
#define		CYPRENORMAL_SIZE	AIF2_LTE40_CYPRENORMAL_SIZE
#endif

#ifdef SUPERPACKET
#define		NBCHANNELINIT		NBCHANNELMAXPERLINK	// In superpacket mode, multiple Lte symbols received within the same Rx monolithic packet
#else
#define		NBCHANNELINIT		1
#endif

#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 5 || EVM_TYPE == 6 || EVM_TYPE == 7 || EVM_TYPE == 8
//#define     DESCMSM				1			// Putting packet descriptors in internal MSM
//#else
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
#define     DESCMSM				0			// else going to DDR
#else
#define     DESCMSM				1			// keep in MSM
#endif
#else
#define     DESCMSM				0			// else going to DDR
#endif
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
#if LTE_RATE == 20
#define		NBDESCMAX			(1024*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 10
#define		NBDESCMAX			(2048*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 5
#define		NBDESCMAX			(4096*NB_LINKS)          // Descriptor count rounded to the next power of 2
#endif
#else
#if LTE_RATE == 20
#define		NBDESCMAX			(64*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 10
#define		NBDESCMAX			(128*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 5
#define		NBDESCMAX			(256*NB_LINKS)          // Descriptor count rounded to the next power of 2
#elif LTE_RATE == 40
#define		NBDESCMAX			(32*NB_LINKS)          // Descriptor count rounded to the next power of 2
#endif
#endif
#define		MAX_DEBUG_SLOT		400			// For debug, size of monitoring arrays
#if EVM_TYPE == 0 || EVM_TYPE == 3
#define     SYSCLK_INPUT_KHZ    153600  	// on Lyrtech and Scbp EVM, the Frequency is based on an external 153.60 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  	// on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif

#define     DIO_0               AIF2FL_DIO_ENGINE_0 // DIO engines are not used in LTE mode - instead PKTDMA and HW queues are used to carry the antenna data from/to AIF2

#define 	MONO_RX_FDQ         2012 //9030 //2012 // Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index
#define 	MONO_RX_Q           1000 //9040

#define 	RX_SUCCESS_CNT		32000 // Define a number of consecutive runtime RF check success prior to monitoring runtime failures

#define 	FREQ				1.0
#define     PEAK_LOC_2MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ*2) // Used for peak detection after fft on Rx symbols  - 2048/30.72*2MHz, same for Lte5 or Lte10
#define 	PEAK_LOC_1MHZ		(FFT_SIZE/SAMP_FREQ_FLOAT*FREQ)   // Used for peak detection after fft on Rx symbols  - 2048/30.72*1MHz, same for Lte5 or Lte10
#if LTE_RATE == 40
#define		N   (4096)											  // Used for twiddle factor generation
#else
#define		N   (2048)											  // Used for twiddle factor generation
#endif
#define 	PAD (16)											  // Used for twiddle factor generation

#define SLOT_NUM_FIRST_PUSH	100  			// The first push needs to happen on the last slot number before frame boundary
#if CPRI_RELAY_CFG == 2
#define SLOT_NUM_DATA_CHECK	150002 			// If testInfo.never_end == 0, test stops after this amount of Lte slots
#else
#define SLOT_NUM_DATA_CHECK	7502			// If testInfo.never_end == 0, test stops after this amount of Lte slots
#endif

#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
#define	NBSF	10
#else
#define	NBSF	1
#endif

#define USEPATTERN 2
#define TM 1
#define RAMP 0

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

typedef struct {
	    		int32_t	magic;
	    		int32_t   revision;
	const 		int32_t 	type;
	const		int32_t	loopback;
	volatile 	int32_t	test_case;
	volatile	int32_t	capture_requested;
	volatile	int32_t	never_end;
	volatile 	int32_t	use_cio;
				int32_t	error_condition;
	volatile	int32_t	force_stop;
				} test_info_t;

typedef struct {
				uint8_t	firstsymbol[LTESYMBOLSIZE  - 16];
				uint8_t	othersymbol[6][LTESYMBOL2SIZE - 16];
	} lte_slot_t;

//////////////////////////////////////////////////////////////////////////////
// Global variables
AIF_ConfigObj               aifObj;					// Main AIF2 LLD object for AIF2 configuration
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile unsigned int ntest = 0; 		// used for running multiple tests
volatile unsigned int testcheck = 1; 	// reports pass fail at the end of the test

#ifdef LOOPBACK
volatile unsigned int swSync        = 1;	// AIF2 SW trigger
volatile unsigned int intLoopback   = 1;	// SerDes loopback mode
#else
volatile unsigned int intLoopback   = 0;
#if EVM_TYPE == 3 || EVM_TYPE == 8
volatile unsigned int swSync        = 1;
#else
volatile unsigned int swSync        = 0;
#endif
#endif

volatile unsigned char testEnable[TEST_NUM] =
    {
#if EVM_TYPE == 5 || EVM_TYPE == 7 || EVM_TYPE == 8
        1,     // CPRI 4x - Link 3 is used on CPRI relay setup to connect TCI6614 EVM and SCBP via SFP
        0
#else
        0,     // CPRI 4x - Link 0
		1
#endif
    };

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st configuration - CPRI 4x - Link 3 is used on CPRI relay setup to connect TCI6614 EVM and SCBP via SFP
#if LTE_RATE == 20
	  "LTE_RELAY_20MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 10
	  "LTE_RELAY_10MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 5
	  "LTE_RELAY_5MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 40
	  "LTE_RELAY_40MHZ_RF_SINGLE_TONE", // test name
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
#if LTE_RATE == 20
	  "LTE_RELAY_20MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 10
	  "LTE_RELAY_10MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 5
	  "LTE_RELAY_5MHZ_RF_SINGLE_TONE", // test name
#elif LTE_RATE == 40
	  "LTE_RELAY_40MHZ_RF_SINGLE_TONE", // test name
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
#pragma DATA_SECTION(txSamples,".aifrawtx");
uint8_t   	txSamples[NBCHANNELPERLINK*NB_LINKS][NBSYMBOL*2][LTESYMBOLSIZE - 16];
#pragma DATA_SECTION(rxSamples,".aifrawrx");
uint8_t   	rxSamples[NBCHANNELPERLINK*NB_LINKS][NBSYMBOL*2][LTESYMBOLSIZE - 16];
#pragma DATA_SECTION(rxSlots,".aifrxcapture");
lte_slot_t  rxSlots[NBCHANNELPERLINK*NB_LINKS][20]; // can capture up to 10ms per channel

#pragma DATA_SECTION(testInfo,".aifcaptcmd");
test_info_t	testInfo = 	{
		0x50CCA06F, 		//magic;
		0x00000000, 		//revision;
		SPECIAL_SEQUENCE, 	//type;
		#ifdef LOOPBACK
		1, 					//loopback;
		#else
		0, 					//no loopback;
		#endif
		-1, 				//test_case;
		0, 					//capture_requested;
		#if EVM_TYPE == 3
		1, 					//never_end;
		#else
		0, 					//no never_end;
		#endif
		1,					//use_cio
		0, 					//error_condition
		0,					//force_stop
};

uint32_t  capture_ongoing   = 0;
uint32_t  capture_AxC   = 0;

/*
 * Variables used for RF checks
 */
uint8_t   rxBuffer[(NBCHANNELPERLINK*NB_LINKS)+1][LTESYMBOLSIZE-16]; // MUST BE IN LL2 to enable HW cache coherency
uint8_t   rxCheck[NBCHANNELPERLINK*NB_LINKS][NBSYMBOL];			// Keeps track of current check of all LTE symbols on a slot per AxC
uint16_t  rxRuntimeFail[NBCHANNELPERLINK*NB_LINKS][NBSYMBOL];		// Keeps track of the number of failures per LTE symbols on a slot per AxC
uint8_t   rxCheckCnt=0;									// Counter
uint16_t  rxRuntimeSuccess=0;								// Counter

/*
 *	Variables used fft software
 */
#pragma DATA_ALIGN(twiddleFacts, 8);
int16_t   twiddleFacts[2*N + 2*PAD];


#pragma DATA_SECTION(symbolPkt,".aifrawrx");
/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK*NB_LINKS][NBSYMBOL*2*NBSF];

/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[NBCHANNELMAXPERLINK*NB_LINKS][NBSYMBOL*2*NBSF];

/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS][NBCHANNELMAXPERLINK*NB_LINKS];

/* Variables for test purposes */
uint32_t nbchanneltotal    = NBCHANNELPERLINK*NB_LINKS; // Default to 1 link
#ifdef SUPERPACKET
uint32_t nbchannelSuperPacket = 1;             // Default is 1 AxC for a super packet
#else
uint32_t nbchannelSuperPacket = NBCHANNELPERLINK*NB_LINKS;
#endif
uint32_t   activeLink;						// link configured at test init time

/*
 * Slot and symbolcount counters - some used for debug purposes
 */
volatile uint32_t slotcount = 0;
volatile uint32_t symbolcount = 0;
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
volatile uint32_t packets_already_pushed = 1;
#else
volatile uint32_t packets_already_pushed = 0;
#endif
volatile uint32_t aif2SymbolEgressCount[MAX_DEBUG_SLOT];
volatile uint32_t aif2SymbolIngressCount[MAX_DEBUG_SLOT];
volatile uint32_t getCpuTimestamp[MAX_DEBUG_SLOT];
volatile uint32_t getPhyTimerFrames[MAX_DEBUG_SLOT];
volatile uint32_t getPhyTimerClk[MAX_DEBUG_SLOT];
volatile uint32_t getRxPktCnt[MAX_DEBUG_SLOT]; // for first channel
volatile uint32_t rxPktCnt[NBCHANNELPERLINK*NB_LINKS]; // assuming 2 link max
unsigned char 	saveDesc[NBCHANNELPERLINK*NB_LINKS][14][32];
volatile uint32_t dbgslotcount = 0;
volatile uint32_t rxSymNum;

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
extern int 		       PEAKDEBUG;
extern uint32_t 		   eeLastCpriLossInFrame;
#if SPECIAL_SEQUENCE == 1
// test vectors
#if LTE_RATE == 20
extern uint32_t txSeqOdd[2048];
extern uint32_t txSeqEven[2048];
extern uint32_t txSeq1MHz[3072];
extern uint32_t txSeqImd[3072];
extern uint32_t txSeqNoComb[2048];
extern uint32_t txSeqWideBand[2048];
#elif LTE_RATE == 10
extern uint32_t txSeqOdd[1024];
extern uint32_t txSeqEven[1024];
extern uint32_t txSeq1MHz[1536];
extern uint32_t txSeqImd[1536];
extern uint32_t txSeqNoComb[1024];
extern uint32_t txSeqWideBand[1024];
#elif LTE_RATE == 5
extern uint32_t txSeqOdd[512];
extern uint32_t txSeqEven[512];
extern uint32_t txSeq1MHz[768];
extern uint32_t txSeqImd[768];
extern uint32_t txSeqNoComb[512];
extern uint32_t txSeqWideBand[512];
#endif
#endif

//////////////////////////////////////////////////////////////////////////////
// Function prototypes

#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif
void getcomplex(Complex16* payload,uint32_t index,uint32_t tx);
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
void getTMdata(Complex16* payload,uint32_t tx);
void testPattern(Complex16* payload, uint32_t index, uint32_t tx);
#endif
#if SPECIAL_SEQUENCE == 1
void getcomplex1(Complex16* payload,uint32_t index, uint32_t testType1);
#endif

//////////////////////////////////////////////////////////////////////////////
// Function bodies

/*
 * User Isr that is called from pre-defined Isr in cslUtils.c, called every Lte slot
 */
void slotIsr()
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen, *payloadPtr;
	Cppi_MonolithicDesc *ptrMonoDesc;
	Complex16			*checkPtr;

	slotcount++;dbgslotcount++;
	getCpuTimestamp[dbgslotcount]     = TSCL;
	getPhyTimerFrames[dbgslotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
	getPhyTimerClk[dbgslotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

	if (slotcount == (SLOT_NUM_FIRST_PUSH - 20))
	{
		for(chan = 0; chan < nbchannelSuperPacket; chan++)
		{
			AIF_enablePdCh(&aifObj,chan);
		}
	}

	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */
	if ((slotcount < SLOT_NUM_DATA_CHECK) || (testInfo.never_end == 1))
	{

		for(chan = 0; chan < nbchannelSuperPacket; chan++)
		{
			rxSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);

			for(i=0;i<rxSymCnt;i++)
			{
				  ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
				  // Invalidate symbol packet header and get the updated payload address + length
				  CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
				  Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);

				   // Store Rx symbol number
				   rxSymNum = *(((uint32_t*)ptrMonoDesc) + 3);
				   rxSymNum = (rxSymNum>>7)&0x000000FF;

				  if((rxPktCnt[chan]%NBSYMBOL) == 0)
				  {
						//check payload length for first symbol
						if (payloadLen != ((LTESYMBOLSIZE-16)*NBCHANNELINIT)) {
							UTILS_aif2ExceptIntDisable();
							slotcount = SLOT_NUM_DATA_CHECK  + DSP_procId;
							testInfo.never_end = 0;
							testInfo.error_condition = -2;
							printf("FATAL: unexpected payload length:%d, %d\n",((LTESYMBOLSIZE-16)*NBCHANNELINIT),payloadLen);
							AIF_printException(&aifObj);
						}

						if (rxCheckCnt == 0) {
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr, (uint32_t *)rxBuffer[0], \
													 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
							UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr + aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
													 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
						}
#else
							CACHE_invL1d((void *)payloadPtr, (LTESYMBOLSIZE-16), CACHE_WAIT);
#if SPECIAL_SEQUENCE == 0
							memcpy(&rxBuffer[chan][0],(uint32_t *) payloadPtr, (LTESYMBOLSIZE-16));
#endif
						}

#if EVM_TYPE == 6 || EVM_TYPE == 8
                        if ((capture_ongoing == 1) && (capture_AxC == chan)) {
                            UTILS_triggerEdmaSampleCapt(rxSymNum%NBSYMBOL,(uint32_t *)&rxSlots[capture_AxC][rxSymNum/NBSYMBOL].firstsymbol[0] ,(uint32_t *)payloadPtr, (LTESYMBOLSIZE-16));
                        }
#endif
#endif


				  } else {
						//check payload length for the other 6 symbols
						if (payloadLen != ((LTESYMBOL2SIZE-16)*NBCHANNELINIT)) {
							UTILS_aif2ExceptIntDisable();
							slotcount = SLOT_NUM_DATA_CHECK  + DSP_procId;
							testInfo.never_end = 0;
							testInfo.error_condition = -2;
							printf("FATAL: unexpected payload length:%d, %d\n",((LTESYMBOL2SIZE-16)*NBCHANNELINIT),payloadLen);
							AIF_printException(&aifObj);
						}

						if (rxCheckCnt == (rxPktCnt[chan]%NBSYMBOL)) {
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr, (uint32_t *)rxBuffer[0], \
													 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOL2SIZE-16));
							UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr + aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
													 (uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOL2SIZE-16));
						}
#else
							CACHE_invL1d((void *)payloadPtr, (LTESYMBOL2SIZE-16), CACHE_WAIT);
#if SPECIAL_SEQUENCE == 0
							memcpy(&rxBuffer[chan][0],(uint32_t *) payloadPtr, (LTESYMBOL2SIZE-16));
#endif
						}

#if EVM_TYPE == 6 || EVM_TYPE == 8
                        if ((capture_ongoing == 1) && (capture_AxC == chan)) {
                            UTILS_triggerEdmaSampleCapt(rxSymNum%NBSYMBOL,(uint32_t *)&rxSlots[capture_AxC][rxSymNum/NBSYMBOL].othersymbol[(rxSymNum%NBSYMBOL)-1][0],(uint32_t *)payloadPtr, (LTESYMBOL2SIZE-16));
                        }
#endif
#endif

				  }
				  // recycle symbol packet on rx free queue
				  Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
				  rxPktCnt[chan]++;
			}
		}
	}

	getRxPktCnt[dbgslotcount]      = rxPktCnt[0];

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
						UTILS_aif2ExceptIntDisable();
						slotcount = SLOT_NUM_DATA_CHECK  + DSP_procId;
						testInfo.never_end = 0;
						testInfo.error_condition = -1;
						printf("FATAL: unexpected first sample in this packet\n");
						AIF_printException(&aifObj);
					}
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)symbolPkt[chan][i]);
				}
			}
			symbolcount += NBSYMBOL;
			if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
			if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
				packets_already_pushed = 1;
				AIF_enableException(&aifObj); // From this point, exceptions counter will be incremented until Lte slot prior to data check
			}
		}
	} else {
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
		if(((slotcount >= (SLOT_NUM_FIRST_PUSH)) && ((slotcount < (SLOT_NUM_DATA_CHECK + (NBSF*2) - 2)) || (testInfo.never_end == 1))))
#else	
		if(((slotcount > (SLOT_NUM_FIRST_PUSH+1)) && (slotcount < SLOT_NUM_DATA_CHECK)) || (testInfo.never_end == 1))
#endif		
		{

			// Flag exceptions
			if (testInfo.error_condition == 0) {
				if (aifObj.aif2EeCount.eeFlag) testInfo.error_condition = -3;
			}
			// Reset global exception counter
			if (testInfo.error_condition == 1) {
				aifObj.aif2EeCount.eeFlag = 0;
				testInfo.error_condition = 0;
			}

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
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
					if ((checkPtr->re != firstSample[chan][i].re) || (checkPtr->im != firstSample[chan][i].im)) {
#else					
					if ((checkPtr->re != firstSample[chan][i%(2*NBSYMBOL)].re) || (checkPtr->im != firstSample[chan][i%(2*NBSYMBOL)].im)) {
#endif					
						UTILS_aif2ExceptIntDisable();
						slotcount = SLOT_NUM_DATA_CHECK  + DSP_procId;
						testInfo.never_end = 0;
						testInfo.error_condition = -1;
						printf("FATAL: unexpected first sample in this packet\n");
						AIF_printException(&aifObj);
					}
					memcpy(&saveDesc[chan][i%(2*NBSYMBOL)][0],ptrMonoDesc,32);
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)ptrMonoDesc);
				}
			}
			symbolcount += NBSYMBOL;
			if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;
			if ((slotcount == (SLOT_NUM_DATA_CHECK-1)) && (testInfo.never_end == 0)) UTILS_aif2ExceptIntDisable();
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
			if (slotcount == (SLOT_NUM_FIRST_PUSH+1)) {
				AIF_enableException(&aifObj); // From this point, exceptions counter will be incremented until Lte slot prior to data check
			}
#endif
		}
	}

	if ((slotcount < (SLOT_NUM_DATA_CHECK-1)) || (testInfo.never_end == 1))
	{
#if SPECIAL_SEQUENCE == 0
#if LTE_RATE != 40
		// Run-time RF checks
		if (rxCheckCnt == 0)
		{
			DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[0][CYPRENORMAL1_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
			if (checkSingleTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ,0))
			{
				rxCheck[0][0] = 1;
				if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
			} else {
				rxCheck[0][0] = 0;
				if (rxRuntimeSuccess == RX_SUCCESS_CNT){
					rxRuntimeFail[0][0]++;
				}
				else rxRuntimeSuccess = 0;
			}
			DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[1][CYPRENORMAL1_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
			if (checkDualTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1))
			{
				rxCheck[1][0] = 1;
				if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
			} else {
				rxCheck[1][0] = 0;
				if (rxRuntimeSuccess == RX_SUCCESS_CNT) {
					rxRuntimeFail[1][0]++;
				}
				else rxRuntimeSuccess = 0;
			}
		} else {
			DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[0][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
			if (checkSingleTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ,0))
			{
				rxCheck[0][rxCheckCnt] = 1;
				if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
			} else {
				rxCheck[0][rxCheckCnt] = 0;
				if (rxRuntimeSuccess == RX_SUCCESS_CNT) rxRuntimeFail[0][rxCheckCnt]++;
				else rxRuntimeSuccess = 0;
			}
			DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[1][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
			if (checkDualTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1))
			{
				rxCheck[1][rxCheckCnt] = 1;
				if (rxRuntimeSuccess < RX_SUCCESS_CNT) rxRuntimeSuccess++;
			} else {
				rxCheck[1][rxCheckCnt] = 0;
				if (rxRuntimeSuccess == RX_SUCCESS_CNT) rxRuntimeFail[1][rxCheckCnt]++;
				else rxRuntimeSuccess = 0;
			}
		}
#endif		
#endif
		rxCheckCnt++;
		if (rxCheckCnt == NBSYMBOL) {
			rxCheckCnt = 0;
		}
		if (rxSymNum == (AIF2_LTE_FRAME_SYMBOL_NUM - 1))
		{
#if EVM_TYPE == 6 || EVM_TYPE == 8
			if ((testInfo.capture_requested==1) && (capture_ongoing==1)) {
				capture_AxC++;
				if (capture_AxC == aifObj.linkConfig[activeLink].numPeAxC)
				{
					capture_ongoing   = 0;
					testInfo.capture_requested = 0;
				}
			}
            if ((testInfo.capture_requested==1) && (capture_ongoing==0)) {
                capture_ongoing   = 1;
                capture_AxC       = 0;
            }
#endif
		}
	}

	/* PktDMA activity monitoring */
	aif2SymbolEgressCount[dbgslotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
	aif2SymbolIngressCount[dbgslotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
	// Make sure no overflow occurs on the monitoring arrays
	if (dbgslotcount == (MAX_DEBUG_SLOT-1)) dbgslotcount = 0;
}


int main(void)
{
    uint32_t               idx, idx1, idx2, i, payloadLen, monoRxCount, monoTxCount, *payloadPtr2, nblink = 1, chan, rx_count, value,z;
    uint16_t               testpass = 0, myboard	= EVM_TYPE;
	Complex16            *payloadPtr;
    Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Cppi_DescTag 		 descTag;
    Qmss_Queue           queueInfo;
    uint32_t               thisPiCaptured;

#if SPECIAL_SEQUENCE == 1
    int32_t			     testType;
    uint32_t			     idx1Max;
	uint8_t                *txSamPtr0, *rxSamPtr0, *txSamPtr1, *rxSamPtr1;
    int8_t                 verbTest[20];
    FILE                 *infile = NULL;
    FILE                 *outfile = NULL;
#endif
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
	FILE                 *outfile = NULL;
#endif
#if (CPRI_RELAY_CFG == 1) && (SPECIAL_SEQUENCE == 0)
	Complex16            sampleCheck;
#endif
#if EVM_TYPE == 3
    PllcHwSetup pllc_hwSetup;
#endif
#ifdef SIMULATOR_SUPPORT
	printf ("\n Error: This example code does not run on the simulator.\n\n");
	return(0);
#endif

	TSCL = 0;

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

    /* Make 3 * 16MB cacheable starting from 0xA1000000*/
    CACHE_setMemRegionInfo(161,1,0);// MAR161 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(162,1,0);// MAR162 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(163,1,0);// MAR163 - cacheable (always), not prefetchable

	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

#if SPECIAL_SEQUENCE == 1
	while ((testInfo.test_case == -1) && (infile == NULL))
	{
	if (testInfo.use_cio == 1) outfile = fopen("outtest.dat","w");
    if (outfile) fclose(outfile);
    if (testInfo.use_cio == 1) infile  = fopen("intest.dat","r");
    if(infile == NULL)
    {
        //printf("Input test file doesn't exist");
    	while (testInfo.test_case == -1);
        testType = testInfo.test_case; /* default */
    }
    else
    {
        fscanf(infile,"%s%d",verbTest,&testType);
        fclose(infile);
    }

    if (testInfo.use_cio == 1) outfile = fopen("outtest.dat","a");
    if (outfile) fprintf(outfile,"%s%6d\n","Test type: ",testType);
    if (outfile) fclose(outfile);
    printf("%s%6d\n","Test type: ",testType);
    if(testType == 0) // zero signal
    {
        idx1Max = 1;
    }
    else if(testType == 1) // 1200 consecutive tones
    {
        idx1Max = sizeof(txSeqNoComb)/4;
    }
    else if(testType == 2) // 600 even tones
    {
        idx1Max = sizeof(txSeqEven)/4;
    }
    else if(testType == 3) // 600 odd tones
    {
        idx1Max = sizeof(txSeqOdd)/4;
    }
    else if(testType == 4)  // IMD tones: +/-1MHz
    {
        idx1Max = sizeof(txSeqImd)/4;
    }
    else if(testType == 5)  // Wide band signal
    {
        idx1Max = sizeof(txSeqWideBand)/4;
    }
    else if(testType == 10) /* 1MHz tone */
    {
        idx1Max = sizeof(txSeq1MHz)/4;
    }
    else /* Default: 1MHz tone */
    {
        idx1Max = sizeof(txSeq1MHz)/4;
    }
#endif
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
    // Wait for test_case to be set by ARM Linux or CCS debugger
	while (testInfo.test_case == -1);
    printf("%s%6d\n","Test case: ",SPECIAL_SEQUENCE);
#endif
#if CPRI_RELAY_CFG == 1
    printf("Beginning AIF2 LTE CPRI relay Software testing\n");
#else
    printf("Starting AIF2 LTE CPRI relay Software\n");
#endif

    UTILS_waitForHw(100000);

    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
	else if (EVM_TYPE == 6) DSP_procId = 1;
    else if (EVM_TYPE == 7) DSP_procId = 1;
    else if (EVM_TYPE == 8) DSP_procId = 1;
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

	// AIF2 LLD programs AT event 5 with period 0.5ms in case of LTE
	aif2evt5_userIsr = slotIsr;

	// Take AIF out of power saver
	AIF_enable();

#ifdef USE_SMA
	// disable link 0 on DSP_1
	testObjTab[ntest].linkEnable[3]  = 0;
	testObjTab[ntest].linkEnable[4]  = 1;
	activeLink = 4;
#endif

#if EVM_TYPE == 6 || EVM_TYPE == 8
	UTILS_openEdmaSampleCapt();
#endif

	// General parameters
	memset(&aifObj, 0, sizeof(aifObj));
	memset(&rxSlots, 0, sizeof(rxSlots));
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
	aifObj.mode               = AIF_LTE_FDD_MODE;
#ifdef SUPERPACKET
	aifObj.superPacket 		  = true;
#else
	aifObj.superPacket 		  = false;
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
    		nblink 	= 0;
    		slotcount = 0;
    		symbolcount = 0;
    		dbgslotcount = 0;
#if SPECIAL_SEQUENCE == 1
    		packets_already_pushed = 0;
    		testcheck = 1;
#endif

			for (i=0;i<AIF_MAX_NUM_LINKS;i++)
			{
				if (testObjTab[ntest].linkEnable[i]==1 && (aifObj.linkConfig[i].RtEnabled==0))
				{
					nblink++;
					// Enable next link if required
					if ((NB_LINKS == 2) && (i<(AIF_MAX_NUM_LINKS-1))) testObjTab[ntest].linkEnable[i+1]=1;
					// PktDma parameters
					for (idx=0 ; idx<NBCHANNELPERLINK ; idx++)
					{
						aifObj.pktDmaConfig.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
						aifObj.pktDmaConfig.txNumDescAxC[chan]  = NBSYMBOL*2*NBSF; // double num of Pkts
						aifObj.pktDmaConfig.txDescSizeAxC[chan] = LTESYMBOLSIZE;
						aifObj.pktDmaConfig.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
						aifObj.pktDmaConfig.rxNumDescAxC[chan]  = NBSYMBOL*2*NBSF; // double num of Pkts
						aifObj.pktDmaConfig.rxDescSizeAxC[chan] = LTESYMBOLSIZE*NBCHANNELINIT;

						memset(&rxFlow[i][idx], 0, sizeof(Cppi_RxFlowCfg));
						rxFlow[i][idx].rx_dest_qnum     = MONO_RX_Q+chan;
						rxFlow[i][idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ+chan;
						rxFlow[i][idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
						rxFlow[i][idx].rx_fdq1_qnum     = MONO_RX_FDQ+chan;
						rxFlow[i][idx].rx_fdq2_qnum     = MONO_RX_FDQ+chan;
						rxFlow[i][idx].rx_fdq3_qnum     = MONO_RX_FDQ+chan;
						rxFlow[i][idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS
						rxFlow[i][idx].rx_psinfo_present = 1;

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

			{
				aifObj.radTimerConfig[0].userSpecified                      = true;
				aifObj.radTimerConfig[0].frameTerminalCount                 = 4096;
				if (swSync == 1)
					aifObj.radTimerConfig[0].initClockNum                       = 0;
				else
					aifObj.radTimerConfig[0].initClockNum                       = 2;
				aifObj.radTimerConfig[0].initSymbolNum                      = 0;
				aifObj.radTimerConfig[0].initFrameLsbNum                    = 0;
				aifObj.radTimerConfig[0].initFrameMsbNum                    = 0;
				aifObj.radTimerConfig[0].lte.cpType                         = AIF2_LTE_CPTYPE_NORMAL;
				aifObj.radTimerConfig[0].lte.numSymbolsForSymbolStrobe      = 1;
				aifObj.radTimerConfig[0].lte.numSymbolStrobesForFrameStrobe = 140;
			}

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{

				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
#ifdef SUPERPACKET
				 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELMAXPERLINK;
				 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELMAXPERLINK;
#else
				 if (testObjTab[ntest].linkEnable[i] == 1)
				 {
					 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELPERLINK;
					 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELPERLINK;
				 }
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
#elif LTE_RATE == 40
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_61P44MHZ;
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
            }

			for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++)
			{
				if (aifObj.linkConfig[i].linkEnable)
				{
					activeLink = i;
					break;
				}
			}

#if EVM_TYPE == 5 || EVM_TYPE == 7
			aifObj.linkConfig[activeLink].nodeTx      = 0;
			aifObj.linkConfig[activeLink].nodeRx      = 2;
#endif
#if EVM_TYPE == 8
			aifObj.linkConfig[activeLink].nodeTx      = 0;
			aifObj.linkConfig[activeLink].nodeRx      = 1;
#endif
			if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
				aifObj.linkConfig[activeLink+1].nodeTx      = aifObj.linkConfig[activeLink].nodeTx;
				aifObj.linkConfig[activeLink+1].nodeRx      = aifObj.linkConfig[activeLink].nodeRx;
			}

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
#if EVM_TYPE == 8
#if CPRI_RELAY_CFG == 1 || CPRI_RELAY_CFG == 2
			//aifObj.linkConfig[activeLink].piMin = 1500; //10100; // taking into account SCBP FPGA specific extra delay on Rx path when going thru RF
			aifObj.linkConfig[activeLink].piMin = 500; //10100; // taking into account SCBP FPGA specific extra delay on Rx path when going thru RF
#endif
#endif
			if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
				aifObj.linkConfig[activeLink+1].piMin      = aifObj.linkConfig[activeLink].piMin;
			}

			// initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
			AIF_initHw(&aifObj);

			/*
			 *  LTE Tx packet symbol initialization at once
			 */


			for(chan = 0;chan < nbchanneltotal;chan++)
			{
				for (z=0;z<NBSF;z++) {
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
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr2, &payloadLen);

					if((idx%NBSYMBOL) == 0)
					{
						 //payload data setup(first symbol)
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							#if SPECIAL_SEQUENCE == 0
							getcomplex(&payloadPtr[idx2],idx1,chan);
							idx1++;
							#elif SPECIAL_SEQUENCE == 1
							getcomplex1(&payloadPtr[idx2],idx1,testType);
							idx1++;
							if(idx1 == idx1Max)
								idx1 = 0;
							#elif SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
								#if USEPATTERN == 1
									testPatternFPGA(&payloadPtr[idx2],idx1,chan);
								#elif USEPATTERN == 2
									getTMdata(&payloadPtr[idx2],chan);
								#endif										
								idx1++;								
							#endif
						}
						if (z == 0) memcpy((void*)&txSamples[chan][idx][0],(void*)payloadPtr,LTESYMBOLSIZE-16);
					} else {
						//payload data setup(other six symbols)
						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
						{
							#if SPECIAL_SEQUENCE == 0
							getcomplex(&payloadPtr[idx2],idx1,chan);
							idx1++;
							#elif SPECIAL_SEQUENCE == 1
							getcomplex1(&payloadPtr[idx2],idx1,testType);
							idx1++;
							if(idx1 == idx1Max)
								idx1 = 0;
							#elif SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
								#if USEPATTERN == 1
									testPatternFPGA(&payloadPtr[idx2],idx1,chan);
								#elif USEPATTERN == 2
									getTMdata(&payloadPtr[idx2],chan);
								#endif										
								idx1++;									
							#endif								
						}
						if (z == 0 )memcpy((void*)&txSamples[chan][idx][0],(void*)payloadPtr,LTESYMBOL2SIZE-16);
					}
					firstSample[chan][idx+(z*(NBSYMBOL*2))].re =  payloadPtr[0].re;
					firstSample[chan][idx+(z*(NBSYMBOL*2))].im =  payloadPtr[0].im;

					//Create PS data
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
					*((uint32_t*)payloadPtr) = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

					if (packets_already_pushed == 1) Qmss_queuePushDesc((aifObj.pktDmaConfig.txFqAxC[chan]), (uint32_t*)mono_pkt);
					//Please note that Tx queue push will be done in slot ISR at the pace of 7 symbols / AxCs
				}
				}
			}
			/*
			 * Perform block writeback on L1D to make sure all symbol packets are coherent with MSM
			 */
			for(idx = 0;idx < NBDESCMAX;idx++) {
				CACHE_wbInvL1d((void *)(mono_region_test+(idx*LTESYMBOLSIZE)), LTESYMBOLSIZE, CACHE_WAIT);
			}
#if SPECIAL_SEQUENCE == 0
			gen_twiddle_fft16x16(&twiddleFacts[PAD], FFT_SIZE);
			memset(rxCheck,0x00,sizeof(rxCheck));
			memset(rxRuntimeFail,0x00,sizeof(rxRuntimeFail));
#endif


#ifdef SUPERPACKET
			// In Lte 5 and 10, we disable the PE channels that are not in use. This tests uses 2 AxCs per link, regardless of the LTE rate definition.
#if LTE_RATE == 10 || LTE_RATE == 5
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[2] = false;
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[3] = false;
#endif
#if LTE_RATE == 5
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[4] = false;
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[5] = false;
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[6] = false;
			aifObj.hAif2Setup->commonSetup->pPeCommonSetup->bEnableCh[7] = false;
#endif
#endif

			// Disabling this mode, so that, on Cpri relay setup, once AIF2 timers are triggered, they keep running based on the initial pulse. Note there isn't a periodic 10ms pulse generated from EVMTCI6614, just a one shot pulse.
			aifObj.hAif2Setup->commonSetup->pAtCommonSetup->AutoResyncMode = AIF2FL_NO_AUTO_RESYNC_MODE;

#if EVM_TYPE == 8
#if CPRI_RELAY_CFG == 1 || CPRI_RELAY_CFG == 2
			aifObj.hAif2Setup->linkSetup[activeLink]->pAtLinkSetup->PiMax += 300; // adding some extra margin on Pi window for debug
#endif
#endif
			if ((NB_LINKS == 2) && (activeLink<(AIF_MAX_NUM_LINKS-1))) {
				aifObj.hAif2Setup->linkSetup[activeLink+1]->pAtLinkSetup->PiMax      = aifObj.hAif2Setup->linkSetup[activeLink]->pAtLinkSetup->PiMax;
			}

			for(chan = 0;chan < nbchanneltotal;chan++)
			{
				aifObj.hAif2Setup->commonSetup->pPdCommonSetup->PdChConfig[chan].bChannelEn = 0;
			}

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
			// Now programming AIF2 registers
			AIF_startHw(&aifObj);

			// Now triggering AIF2 physical and radio timers
			slotcount = 0;
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
				// wait for one extra slots to run out of Rx free pkts
#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
				if((slotcount >= (SLOT_NUM_DATA_CHECK + (NBSF*2) - 1)) && (testInfo.never_end == 0) ||
#else
				if((slotcount >= (SLOT_NUM_DATA_CHECK + DSP_procId)) && (testInfo.never_end == 0) ||
#endif
				   (testInfo.force_stop == 1) && (testInfo.never_end == 1))
				{
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					//keepAifOff = 1;
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					break;
				}

				if ((CSL_FEXT(aifObj.hFl->regs->G_RM_LKS[activeLink].RM_LK_STS0,AIF2_RM_LK_STS0_SYNC_STATUS) != CSL_AIF2_RM_LK_STS0_SYNC_STATUS_ST3) &&
					(slotcount > /*SLOT_NUM_FIRST_PUSH*/37500))
				{
					thisPiCaptured = CSL_FEXT(aifObj.hFl->regs->PI_DATA[activeLink].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					//keepAifOff = 1;
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					printf("FATAL: CPRI link not in HFNSYNC, initial captured pi: %d\n", thisPiCaptured);
					break;
				}
				if ( (NB_LINKS == 2) &&
					(CSL_FEXT(aifObj.hFl->regs->G_RM_LKS[activeLink+1].RM_LK_STS0,AIF2_RM_LK_STS0_SYNC_STATUS) != CSL_AIF2_RM_LK_STS0_SYNC_STATUS_ST3) &&
					(slotcount > /*SLOT_NUM_FIRST_PUSH*/37500))
				{
					thisPiCaptured = CSL_FEXT(aifObj.hFl->regs->PI_DATA[activeLink+1].AT_PIVALUE_LK,AIF2_AT_PIVALUE_LK_PICAPTURED_VALUE);
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					//keepAifOff = 1;
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					printf("FATAL: CPRI link not in HFNSYNC, initial captured pi: %d\n", thisPiCaptured);
					break;
				}
			}

			// disable interrupts before checking for data
			CSR&= 0xFFFFFFFE;

			if (testInfo.force_stop == 0)
			{

				for(chan =0; chan < nbchannelSuperPacket; chan++)
				{
					monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]); // get number of received packed on first channel
					monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[chan]); // channel 0 only
					printf(" Number of monolithic packets received in RX queue channel%d: %d\n", chan, monoRxCount);
					printf(" Number of monolithic packets in TX free queue for channel%d: %d\n", chan, monoTxCount);
				}

				/* Check received symbol packet data */
				testpass = 1;
				for(chan =0; chan < nbchannelSuperPacket; chan++)
				{
					idx1 = 0;
					PEAKDEBUG= 0;
					rx_count = Qmss_getQueueEntryCount((aifObj.pktDmaConfig.rxQAxC[chan]));
					if (rx_count == 0) {testpass =0;testcheck++;}

					for (idx = 0; idx < NBSYMBOL*2*NBSF; idx ++)
					{
						symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
						CACHE_invL1d((void *)symbolPkt[chan][idx], 16, CACHE_WAIT);
						payloadPtr2 = (uint32_t *)symbolPkt[chan][idx];
						payloadPtr2 += 4; //skip pkt header and PS field (16 bytes)
						//testpass = 1;
						if((idx%NBSYMBOL) == 0)
						{
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr2, (uint32_t *)rxBuffer[0], \
									(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
#else
							CACHE_invL1d((void *)payloadPtr2, (LTESYMBOLSIZE-16), CACHE_WAIT);
							memcpy((void*)&rxBuffer[chan][0],(void*)payloadPtr2,LTESYMBOLSIZE-16);
#endif
#if (CPRI_RELAY_CFG == 1) && (SPECIAL_SEQUENCE == 0)
							payloadPtr = (Complex16 *)rxBuffer[chan];
							for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
							{
								getcomplex(&sampleCheck,idx1,chan);
									if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
								{
									testpass = 0; testcheck++;
									printf("receive fail \n");
								}
								idx1++;
							}
#endif
							if (idx < (NBSYMBOL*2)) memcpy((void*)&rxSamples[chan][idx][0],(void*)rxBuffer[chan],LTESYMBOLSIZE-16);
#if SPECIAL_SEQUENCE == 0
							DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan][CYPRENORMAL1_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
							if((chan == 0) || (chan == 2)||(chan == 4) || (chan == 6))
							{
								if (!checkSingleTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ,0)) {
									testpass = 0;testcheck++;
									rxCheck[chan][0] = 0;
									rxRuntimeFail[chan][0]++;
								}
							} else {
								if (!checkDualTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1)) {
									testpass = 0;testcheck++;
									rxCheck[chan][0] = 0;
									rxRuntimeFail[chan][0]++;
								}
							}
#endif
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr2+aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
									(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOLSIZE-16));
							memcpy((void*)&rxSamples[chan+1][idx][0],(void*)rxBuffer[chan+1],LTESYMBOLSIZE-16);
#if SPECIAL_SEQUENCE == 0
							DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan+1][CYPRENORMAL1_SIZE*4], (int16_t*)&rxBuffer[2][0]);
							if (!checkDualTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1)) {
								testpass = 0;testcheck++;
								rxCheck[1][0] = 0;
							}
#endif
#endif
						} else {
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)payloadPtr2, (uint32_t *)rxBuffer[0], \
									(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOL2SIZE-16));
#else
							CACHE_invL1d((void *)payloadPtr2, (LTESYMBOL2SIZE-16), CACHE_WAIT);
							memcpy((void*)&rxBuffer[chan][0],(void*)payloadPtr2,LTESYMBOL2SIZE-16);
#endif
#if (CPRI_RELAY_CFG == 1) && (SPECIAL_SEQUENCE == 0)
							payloadPtr = (Complex16 *)rxBuffer[chan];
							for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
							{
								getcomplex(&sampleCheck,idx1,chan);
									if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
								{
									testpass = 0; testcheck++;
									printf("receive fail \n");
								}
								idx1++;
							}
#endif
							if (idx < (NBSYMBOL*2)) memcpy((void*)&rxSamples[chan][idx][0],(void*)rxBuffer[chan],LTESYMBOL2SIZE-16);
#if SPECIAL_SEQUENCE == 0
							DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0]);
							if((chan == 0) || (chan == 2)||(chan == 4) || (chan == 6))
							{
								if (!checkSingleTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ,0)) {
									testpass = 0;testcheck++;
									rxCheck[chan][idx%NBSYMBOL] = 0;
									rxRuntimeFail[chan][idx%NBSYMBOL]++;
								}
							} else {
								if (!checkDualTone((int16_t*)&rxBuffer[(NBCHANNELPERLINK*NB_LINKS)][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1)) {
									testpass = 0;testcheck++;
									rxCheck[chan][idx%NBSYMBOL] = 0;
									rxRuntimeFail[chan][idx%NBSYMBOL]++;
								}
							}
#endif
#ifdef SUPERPACKET
							UTILS_deinterleaveLteSuperPacket((uint32_t *)(payloadPtr2+aifObj.linkConfig[activeLink].cpriPackMode), (uint32_t *)rxBuffer[1], \
									(uint32_t) aifObj.linkConfig[activeLink].cpriPackMode, aifObj.linkConfig[activeLink].numPeAxC, (LTESYMBOL2SIZE-16));
							memcpy((void*)&rxSamples[chan+1][idx][0],(void*)rxBuffer[chan+1],LTESYMBOL2SIZE-16);
#if SPECIAL_SEQUENCE == 0
							DSP_fft16x16(&twiddleFacts[PAD], FFT_SIZE, (int16_t*)&rxBuffer[chan+1][CYPRENORMAL_SIZE*4], (int16_t*)&rxBuffer[2][0]);
							if (!checkDualTone((int16_t*)&rxBuffer[2][0],FFT_SIZE,PEAK_LOC_1MHZ, PEAK_LOC_2MHZ,1)) {
								testpass = 0;testcheck++;
								rxCheck[1][idx%NBSYMBOL] = 0;
							}
#endif
#endif
						}
						Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

					}
					printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rx_count);
				}


#if SPECIAL_SEQUENCE == 1
				/* Save RF loopback data */
				txSamPtr0 =  &txSamples[0][0][0];
				rxSamPtr0 =  &rxSamples[0][0][0];
				txSamPtr1 =  &txSamples[1][0][0];
				rxSamPtr1 =  &rxSamples[1][0][0];
#if LTE_RATE == 20
				payloadLen = 4*30912;
#elif LTE_RATE == 10
				payloadLen = 4*15456;
#elif LTE_RATE == 5
				payloadLen = 4*7728;
#endif

				if(testType == 0) // zero signal
				{

					if (testInfo.use_cio == 1) outfile = fopen("test0rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1) outfile = fopen("test0rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 1) // 1200 consecutive tones
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test1tx0.bin","wb");  // antenna 0, tx
					if (outfile) fwrite(txSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test1rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test1tx1.bin","wb");  // antenna 1, tx
					if (outfile) fwrite(txSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test1rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 2) // 600 even tones
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test2rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test2rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 3) // 600 odd tones
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test3rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test3rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 4)  // IMD tones: +/-1MHz
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test4rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test4rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 5)  // Wide band signal
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test5tx0.bin","wb");  // antenna 0, tx
					if (outfile) fwrite(txSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test5rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test5tx1.bin","wb");  // antenna 1, tx
					if (outfile) fwrite(txSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test5rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else if(testType == 10) /* 1MHz tone */
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test10tx0.bin","wb");  // antenna 0, tx
					if (outfile) fwrite(txSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10tx1.bin","wb");  // antenna 1, tx
					if (outfile) fwrite(txSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
				else /* Default: 1MHz tone */
				{
					if (testInfo.use_cio == 1)  outfile = fopen("test10tx0.bin","wb");  // antenna 0, tx
					if (outfile) fwrite(txSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10rx0.bin","wb");  // antenna 0, rx
					if (outfile) fwrite(rxSamPtr0, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10tx1.bin","wb");  // antenna 1, tx
					if (outfile) fwrite(txSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
					if (testInfo.use_cio == 1)  outfile = fopen("test10rx1.bin","wb");  // antenna 1, rx
					if (outfile) fwrite(rxSamPtr1, 1, payloadLen, outfile);
					if (outfile) fclose(outfile);
				}
#endif


				if (testpass == 1) {
					printf(" Test a) Monolithic Packet Data Send/Recv: PASS on channel: %d \n", chan);
				} else {
					printf(" Test a) Monolithic Packet Data Send/Recv: FAIL\n");
					for(chan =0; chan < nbchannelSuperPacket; chan++)
					{
						for (idx = 0; idx < NBSYMBOL; idx ++)
						{
							if (rxRuntimeFail[chan][idx]) {
								printf("Rx data check failed on Antenna%d Symbol%d\n",chan,idx);
							}
						}
					}
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
				if (value != nbchanneltotal*NBSYMBOL*2*NBSF) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
				else printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d PASS\n",value);
				value =0;
				for (i =0 ; i< nbchannelSuperPacket; i++) {
					value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[i]);
				}
				if (value != 0) printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d FAIL\n",value);
				else printf(" Test b3) Monolithic Packet Rx Descriptor Counts:%d PASS\n",value);
				value =0;
				for (i =0 ; i< nbchannelSuperPacket; i++) {
					value += Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxFqAxC[i]);
				}
				if (value != nbchannelSuperPacket*NBSYMBOL*2*NBSF) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
				else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
				// Store all received symbols into a dat file
				for(chan =0; chan < nbchannelSuperPacket; chan++)
				{

#if SPECIAL_SEQUENCE == 2 //TM == 3
#if LTE_RATE == 20
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_20M_CellID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_20M_CellID1_AxC1.dat","wb");
					}
					else if (chan == 2) { // when 2 links enabled
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_20M_CellID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_20M_CellID1_AxC3.dat","wb");
					}
#elif LTE_RATE == 10
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_10M_CellID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_10M_CellID1_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_10M_CellID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_10M_CellID1_AxC3.dat","wb");
					}
#elif LTE_RATE == 5
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC3.dat","wb");
					}
					else if (chan == 4) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC4.dat","wb");
					}
					else if (chan == 5) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC5.dat","wb");
					}
					else if (chan == 6) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC6.dat","wb");
					}
					else if (chan == 7) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM3p1_5M_CellID1_AxC7.dat","wb");
					}
#endif
#elif SPECIAL_SEQUENCE == 3
#if LTE_RATE == 20
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_20M_CellID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_20M_CellID1_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_20M_CellID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_20M_CellID1_AxC3.dat","wb");
					}
#elif LTE_RATE == 5
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC3.dat","wb");
					}
					else if (chan == 4) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC4.dat","wb");
					}
					else if (chan == 5) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC5.dat","wb");
					}
					else if (chan == 6) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC6.dat","wb");
					}
					else if (chan == 7) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTEFDD_TM1p1_5M_CellID1_AxC7.dat","wb");
					}
#endif
#elif SPECIAL_SEQUENCE == 4
#ifndef TM_3_1
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_1x20_64QAM_PN9_ID1_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_1x20_64QAM_PN9_ID1_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_1x20_64QAM_PN9_ID1_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_1x20_64QAM_PN9_ID1_AxC3.dat","wb");
					}
#else
					if (chan == 0) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_TM3p1_20M_CellID1_cfg3_AxC0.dat","wb");
					}
					else if (chan == 1) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_TM3p1_20M_CellID1_cfg3_AxC1.dat","wb");
					}
					else if (chan == 2) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_TM3p1_20M_CellID1_cfg3_AxC2.dat","wb");
					}
					else if (chan == 3) {
						if (testInfo.use_cio == 1)  outfile = fopen("LTETDD_TM3p1_20M_CellID1_cfg3_AxC3.dat","wb");
					}
#endif
#endif

					for (idx = 0; idx < NBSYMBOL*2*NBSF; idx ++)
					{
						payloadPtr2 = (uint32_t *)symbolPkt[chan][idx];
						payloadPtr2 += 4; //skip pkt header and PS field (16 bytes)

						if((idx%NBSYMBOL) == 0) payloadLen = LTESYMBOLSIZE-16;
						else                    payloadLen = LTESYMBOL2SIZE-16;
						CACHE_invL1d((void *)payloadPtr2, payloadLen, CACHE_WAIT);
						payloadPtr = (Complex16 *)payloadPtr2;
						for (idx2 = 0; idx2 < payloadLen/4; idx2 ++)
						{
							if (outfile) fprintf(outfile,"%d %d\n",payloadPtr[idx2].re,payloadPtr[idx2].im );
						}
					}
					if (outfile) fclose(outfile);
				}
#endif
			} // force_stop == 0

			for (i =0 ; i< nbchannelSuperPacket; i++) {
				Qmss_queueEmpty(aifObj.pktDmaConfig.txQAxC[i]);
				Qmss_queueEmpty(aifObj.pktDmaConfig.rxQAxC[i]);
				Qmss_queueEmpty(aifObj.pktDmaConfig.txFqAxC[i]);
				Qmss_queueEmpty(aifObj.pktDmaConfig.rxFqAxC[i]);

				Qmss_queueClose(aifObj.pktDmaConfig.txQAxC[i]);
				Qmss_queueClose(aifObj.pktDmaConfig.rxQAxC[i]);
				Qmss_queueClose(aifObj.pktDmaConfig.txFqAxC[i]);
				Qmss_queueClose(aifObj.pktDmaConfig.rxFqAxC[i]);
			}
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
			Qmss_removeMemoryRegion(aifObj.pktDmaConfig.txRegionAxC[0], 0);
			Qmss_exit();
#endif
#if USERM
			UTILS_resetRm();
#endif

			UTILS_doCleanup(&aifObj,TRIG_TIMER);

			if (printexceptions == 1)
			{
				if (eeLastCpriLossInFrame)
					printf(" Last CPRI loss occurred at frame number:%d \n",eeLastCpriLossInFrame);
				AIF_printException(&aifObj);
			}

			printf("\nEnding %s test\n", testObjTab[ntest].name);
 	    }
    }

    CACHE_wbInvAllL1dWait();

#if LTE_RATE == 20
    printf("Test: ending AIF2 LTE 20MHz RF test \n");
#elif LTE_RATE == 10
    printf("Test: ending AIF2 LTE 10MHz RF test \n");
#elif LTE_RATE == 5
    printf("Test: ending AIF2 LTE 5MHz RF test \n");
#elif LTE_RATE == 40
    printf("Test: ending AIF2 LTE 40MHz RF test \n");
#endif

    if (aifObj.aif2EeCount.eeFlag != 0) testcheck++;

    if (testcheck == 1) {
           testcheck = 0;
           printf("All tests have passed\n");
    } else {
           printf("Some tests have failed\n");
    }

#if SPECIAL_SEQUENCE == 1
    testInfo.test_case = -1;
	} // while
#endif

    return (0);
}


// These modes are only used for extensive RF tests on CPRI relay setups
#define     DUAL_TONE_FIVE_NINE 0
#define     QUAD_TONE_ONE_TWO   0
#define     QUAD_TONE_FIVE_NINE 0
#define     EIGHTEEN_TONES      0
#define     TWENTY_FOUR_TONES   0
#define     FORTY_EIGHT_TONES   0
#define     NINETY_SIX_TONES    0

#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
void testPattern(Complex16* payload, uint32_t index, uint32_t tx)
{
	/* note it is 15 bits signed!!! */
	Complex16 x1;
	Complex16 y;
	/* constants */
	if (RAMP == 0)
	{
	if (tx == 0)
	{
		x1.re = 0x5234;/*1234*/
		x1.im = 0x33D5;/*1678*/
		y = x1;
	}
	else if (tx == 1)
	{
		x1.re =0x23d5;/*1555*/
		x1.im =0xF210;/*2AAA*/
		y = x1;
	}
	}
	/* ramp */
	else if (RAMP == 1)
	{
	if (tx == 0)
	{
		x1.re = (int16_t) (index%1024-512);
		x1.im = (int16_t) (index%1024);
		y = x1;
	}
	else if (tx == 1)
	{
		x1.re =(int16_t) (index%1024-256);
		x1.im =(int16_t) (index%1024-128);
		y = x1;
	}
	}


    payload->re = y.re;
    payload->im = y.im;
}
#endif

void getcomplex(Complex16* payload,uint32_t index,uint32_t tx)
{
	float freq;
	Complex16 x1;
	Complex16 y;
	if ((tx==0)||(tx==2)||(tx==4)||(tx==6))
	{
                // 1 MHz signal
                freq = FREQ;
                x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                y = x1;
	} else if ((tx==1)|| (tx==3)||(tx==5)|| (tx==7))
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

#elif QUAD_TONE_ONE_TWO == 1
                Complex16         x2,x3,x4;
                // 1 MHz signal
                freq = 1;
                x1.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // 2 MHz signal
                freq = 2;
                x2.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // -1 MHz signal
                freq = 1;
                x3.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x3.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // -2 MHz signal
                freq = 2;
                x4.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x4.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // quad tone signal
                y.re = x1.re + x2.re + x3.re + x4.re;
                y.im = x1.im + x2.im + x3.im + x4.im;

#elif QUAD_TONE_FIVE_NINE == 1
                Complex16         x2,x3,x4;
                // 5 MHz signal
                freq = 5;
                x1.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x1.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // 9 MHz signal
                freq = 9;
                x2.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x2.im = (int16_t) (0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // -5 MHz signal
                freq = 5;
                x3.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x3.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // -9 MHz signal
                freq = 9;
                x4.re = (int16_t) (0.25 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                x4.im = (int16_t) (-0.25 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                // quad tone signal
                y.re = x1.re + x2.re + x3.re + x4.re;
                y.im = x1.im + x2.im + x3.im + x4.im;

#elif EIGHTEEN_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/18.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 9; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif TWENTY_FOUR_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/24.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 12; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif FORTY_EIGHT_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale;

                scale = 1.0f/48.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 24; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    x1.re = (int16_t) (scale * (cos(PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }

#elif NINETY_SIX_TONES == 1
                Complex16         x2;
                uint32_t            ii;
                float             scale,freq1;

                scale = 1.0f/96.0f;
                y.re = 0;
                y.im = 0;
                for (ii = 0; ii < 48; ii ++)
                {
                    // Plus MHz signal
                    freq = ii + 1;
                    freq1 = (float)freq/2;
                    x1.re = (int16_t) (scale * (cos(PI*freq1*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x1.im = (int16_t) (scale * (sin(PI*freq1*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    // Minus MHz signal
                    x2.re = (int16_t) (scale * (cos(PI*freq1*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    x2.im = (int16_t) (-scale * (sin(PI*freq1*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
                    y.re = y.re + x1.re + x2.re;
                    y.im = y.im + x1.im + x2.im;
                }
#endif
                payload->re = y.re;
                payload->im = y.im;
}

#if SPECIAL_SEQUENCE == 2 || SPECIAL_SEQUENCE == 3 || SPECIAL_SEQUENCE == 4
#if SPECIAL_SEQUENCE == 2
#if LTE_RATE == 20
Complex16 *tx0Ptr = (Complex16 *)LTEFDD_TM3p1_20M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTEFDD_TM3p1_20M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTEFDD_TM3p1_20M_CellID1; // when 2 links enabled
Complex16 *tx3Ptr = (Complex16 *)LTEFDD_TM3p1_20M_CellID1;
#elif LTE_RATE == 10
Complex16 *tx0Ptr = (Complex16 *)LTEFDD_TM3p1_10M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTEFDD_TM3p1_10M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTEFDD_TM3p1_10M_CellID1;
Complex16 *tx3Ptr = (Complex16 *)LTEFDD_TM3p1_10M_CellID1;
#elif LTE_RATE == 5
Complex16 *tx0Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx3Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx4Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx5Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx6Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
Complex16 *tx7Ptr = (Complex16 *)LTEFDD_TM3p1_5M_CellID1;
#endif
#endif
#if SPECIAL_SEQUENCE == 3
#if LTE_RATE == 20
Complex16 *tx0Ptr = (Complex16 *)LTEFDD_TM1p1_20M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTEFDD_TM1p1_20M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTEFDD_TM1p1_20M_CellID1;
Complex16 *tx3Ptr = (Complex16 *)LTEFDD_TM1p1_20M_CellID1;
#elif LTE_RATE == 5
Complex16 *tx0Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx3Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx4Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx5Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx6Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
Complex16 *tx7Ptr = (Complex16 *)LTEFDD_TM1p1_5M_CellID1;
#endif
#endif
#if SPECIAL_SEQUENCE == 4
#ifndef TM_3_1
Complex16 *tx0Ptr = (Complex16 *)LTETDD_20M_CellID1;
Complex16 *tx1Ptr = (Complex16 *)LTETDD_20M_CellID1;
Complex16 *tx2Ptr = (Complex16 *)LTETDD_20M_CellID1;
Complex16 *tx3Ptr = (Complex16 *)LTETDD_20M_CellID1;
#else
Complex16 *tx0Ptr = (Complex16 *)LTETDD_TM3p1_20M_CellID1_cfg3;
Complex16 *tx1Ptr = (Complex16 *)LTETDD_TM3p1_20M_CellID1_cfg3;
Complex16 *tx2Ptr = (Complex16 *)LTETDD_TM3p1_20M_CellID1_cfg3;
Complex16 *tx3Ptr = (Complex16 *)LTETDD_TM3p1_20M_CellID1_cfg3;
#endif
#endif

void getTMdata(Complex16* payload,uint32_t tx)
{
	Complex16 y;
	if (tx==0)
	{
       y.re = tx0Ptr->re;
       y.im = tx0Ptr->im;
       tx0Ptr++;
	} else if (tx==1)
	{
	   y.re = tx1Ptr->re;
	   y.im = tx1Ptr->im;
	   tx1Ptr++;
	}
	else if (tx==2)
	{
	   y.re = tx2Ptr->re;
	   y.im = tx2Ptr->im;
	   tx2Ptr++;
	}  else if (tx==3)
	{
	   y.re = tx3Ptr->re;
	   y.im = tx3Ptr->im;
	   tx3Ptr++;
	}
#if LTE_RATE == 5
	else if (tx==4)
	{
	   y.re = tx4Ptr->re;
	   y.im = tx4Ptr->im;
	   tx4Ptr++;
	}  else if (tx==5)
	{
	   y.re = tx5Ptr->re;
	   y.im = tx5Ptr->im;
	   tx5Ptr++;
	}  else if (tx==6)
	{
	   y.re = tx6Ptr->re;
	   y.im = tx6Ptr->im;
	   tx6Ptr++;
	}  else if (tx==7)
	{
	   y.re = tx7Ptr->re;
	   y.im = tx7Ptr->im;
	   tx7Ptr++;
	}
#endif
    payload->re = y.re;
    payload->im = y.im;
}
#endif

#if SPECIAL_SEQUENCE == 1

void getcomplex1(Complex16* payload,uint32_t index, uint32_t testType1)
{
	Complex16 y;

    if(testType1 == 0) // Zero signal
    {
    	y.re = 0;
	    y.im = 0;
    }
    else if(testType1 == 1) // 1200 consecutive tones
    {
    	y.re = (int16_t)(txSeqNoComb[index] >> 16);
	    y.im = (int16_t)((txSeqNoComb[index]<<16)>>16);
    }
    else if(testType1 == 2) // 600 even tones comb
    {
    	y.re = (int16_t)(txSeqEven[index] >> 16);
	    y.im = (int16_t)((txSeqEven[index]<<16)>>16);
    }
    else if(testType1 == 3) // 600 odd tones comb
    {
    	y.re = (int16_t)(txSeqOdd[index] >> 16);
	    y.im = (int16_t)((txSeqOdd[index]<<16)>>16);
    }
    else if(testType1 == 4) // IMD tones: +/-1MHz
    {
    	y.re = (int16_t)(txSeqImd[index] >> 16);
	    y.im = (int16_t)((txSeqImd[index]<<16)>>16);
    }
    else if(testType1 == 5) // Wide band signal
    {
    	y.re = (int16_t)(txSeqWideBand[index] >> 16);
	    y.im = (int16_t)((txSeqWideBand[index]<<16)>>16);
    }
    else if(testType1 == 10) // 1MHz tone
    {
    	y.re = (int16_t)(txSeq1MHz[index] >> 16);
	    y.im = (int16_t)((txSeq1MHz[index]<<16)>>16);
    }
    else // Default: 1MHz tone
    {
    	y.re = (int16_t)(txSeq1MHz[index] >> 16);
	    y.im = (int16_t)((txSeq1MHz[index]<<16)>>16);
    }


    payload->re = y.re;
    payload->im = y.im;
}

void getcomplex2(Complex16* payload,uint32_t index)
{
	Complex16 y;

	y.re = (int16_t)(txSeq1MHz[index] >> 16);
	y.im = (int16_t)((txSeq1MHz[index]<<16)>>16);

    payload->re = y.re;
    payload->im = y.im;

//    payload->re = 0;
//    payload->im = 0;
}

#endif

////////////

