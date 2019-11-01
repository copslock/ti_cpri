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
#include <ti/sysbios/knl/Clock.h>

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
#include "4B5Bencdec.h"

#if EVM_TYPE == 5
#include "appletonScbpSync.h"
#endif


/* TEST DESCRIPTION
 * This CPRI LTE FAST C&M test is designed to work either in internal loopback mode, or with a dual EVM setup connected with a break-out card
 * The default project configurations in the pdk-generated workspace are supporting:
 * - EVM_LBACK: test compiled for internal loopback mode. Can be used with device simulators as well.
 * - EVM6670_DSP1 and EVM6670_DSP2: test compiled for first and second EVMs of a dual EVM setup connected with a break-out card
 *
 * This CPRI LTE FAST C&M test is designed to work either in internal loopback mode, or with a dual EVM setup connected with a break-out card
 * The  project configurations are:
 * - Appleton_D1C0: DSP core 0 configuration.
 * - Appleton_D1C1: DSP core 1 configuration.
 *
 * This CPRI LTE FAST C&M test runs on a single link of AIF2 and configures 2 AxCs. Both AxCs IQ data are pure single or dual tone sinewaves at given frequencies.
 * On top of LTE antenna IQ traffic, 1 other traffic is running concurrently: cpri fast c&m
 *
 * Pre-processing constants of interest:
 * - EVM_TYPE: 1 - internal loopback or EVM DSP1 (dual EVM setup), 2 - EVM DSP2 (dual EVM setup), 3 - SCBP DSP1, 5 - EVM6614 (Cpri Relay setup), (0 - old Lyrtech C6670 EVM)
 * - LOOPBACK: selects internal loopback mode (SerDes level, AIF2 SW trigger)
 *
 * Note on packet descriptors: all descriptors are of monolithic types (AIF2 LLD doesn't support host type for CPRI control word packets)
 * - Lte Symbol packets: 14 symbols of an AxC subframe time are contained in 14 different monolithic descriptors. The pure single or dual tone sinewaves are generated in a continuous manner
 * 		across the payload of the 14 symbols of a particular AxC. The test is configured to work at the granularity of a Lte slot. For LTE, AIF2 requires usage of monolithic
 *  	descriptor packets with a 16-byte header. In the header, a protocol specific word is required to indicate the antenna carrier number, the symbol number and the direction
 *  	(Ingress/Egress). That means that every 0.5 ms, the test will push and pop from the AIF2 HW queues:
 * 		7 symbols * numAxCs per link * numLinks * 2 (for Egress/Ingress traffic)
 * 		This then makes a total of 56 outstanding monolithic packets in the case of LTE 20MHz / 2 AxCs / 1 Links / both directions
 * 	- Fast c&m packets on Cpri control stream #0: TX_FAST_CM_PKT_NUM/RX_FAST_CM_PKT_NUM of size CPRI_FAST_CM_SIZE (max allocated size for biggest ethernet frame). Note that every hyperframe or 256
 * 	 	basic frames (BFs), 176 BFs can be used for fast C&M. These are BF[20:63], BF[84:127], BF[148:191], BF[212:255] corresponding in 4x rate to 704 bytes. If packet size
 * 	 	is bigger than that, multiple hyperframes will be used to transmit that packet. Note that in this example, ethernet packets are of size ETH_TEST_LEN. Also, the TX PPOINTER
 * 	 	is set to 20 here. Then fast c&m data packets are delimited using 4B/5B encoding where SOP & EOP are defined.
 *
 * General considerations of Cpri control word traffic using AIF2:
 * 	- The use of delimiters was done to actually allocate some bandwidth by selecting given Cpri control word positions, 16, 17, ... and then rely on delimiters to identify the packets.
 * 	So using AIF2 and NULL delimiters guaranty the BW is allocated for a given control word stream, but does not guaranty the exact positions of the packet payload data within the allocated
 * 	control word positions. So when transmitting packets, you'll need on the other side some logic to identify the NULL delimiters.
 * 	-  AIF2FL_CW_DELIM_HYP_FRM option can not be reliably used for TX on AIF2. We experimented that option for Vendor specific info packets with AIF2 LLD, and found out Hyperframe boundary
 * 	 delimiters on AIF2 cannot be used. So you need to go for NULL or 4B5B delimiter option.
 * 	- TCI6614 usage note 24 or C6670 usage note 26 describes some hardware (HW) limitation and restriction of the AIF2 Fast Ethernet CPRI control word channel operation. This includes one HW limitation
 * 	 about Fast Ethernet SSD (Start of Stream Delimiter) handling. This usage note also explains the HW restrictions of 4B5B encoded data nibble level swapping, Hyperframe boundary delimitation and
 * 	 Ethernet packet CRC32 usage. In this test example, we have chosen to implement the SW workarounds, as described in the AIF2LLD user's guide.
 *
 * General considerations when using packet descriptors
 * - When defining a descriptor memory region, the following rules apply:
 * 		- Descriptor size is multiple of 16 bytes, min 32.
 * 		- Descriptor count is a power of 2, minimum 25.
 * 		- Memory region base address must be aligned to a 16-byte address boundary.
 * - Here are some considerations regarding PktDMA traffic:
 * 		- For Egress, it is up to the user to insure that DMA data is available prior to beginning PE message construction (PE2 event). Breaking real time on DMA has
 * 		similar effects to breaking CorePac MIPS real time. Specifically, AIF2 will fail each affected AxC until the beginning of the next radio frame boundary.
 * 		- For Ingress, it is important that DMA is efficient enough that the AIF2 input	buffers never overflow.
 */

////////////////////////////////////////////////////////////////////////////////
// Constant definitions

#define     TRIG_TIMER          3
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

//Enable control words
#define     CPRI_FAST_CM        1
#define     FASTCM              1

#if	FASTCM == 1
#define		CPRI_FAST_CM_SIZE   1920  	 // every hyperframe or 256 basic frames (BFs), 176 BFs can be used for
									 	 // fast C&M. These are BF[20:63], BF[84:127], BF[148:191], BF[212:255]
									 	 // in 4x rate, word size is 32 bits. So 176*4 = 704 bytes.
										 // largest Ethernet frame is 1526 bytes, resulting in 1907 bytes after SW workaround for 4b5b encoding
										 // so choosing packets of size 1920 bytes (multiple of 16 bytes)
#define		ETH_FRAME_LEN		1526
#define		ETH_TEST_LEN		68
#define		TX_FAST_CM_PKT_NUM	2
#define		RX_FAST_CM_PKT_NUM	4
#define     PPOINTER			20
#endif

/* Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index */
#define MONO_RX_FDQ            2024
#define MONO_RX_Q              1024
#define MONO_RX_FDQ_CW         2048
#define MONO_RX_Q_CW           1048

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

#define		NBCHANNELINIT		1

#if EVM_TYPE == 0 || EVM_TYPE == 1 || EVM_TYPE == 2 ||EVM_TYPE == 3 || EVM_TYPE == 4 || EVM_TYPE == 5 || EVM_TYPE == 6
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

#define SLOT_NUM_FIRST_PUSH	20  // The first push needs to happen on the last subframe before frame boundary
#define SLOT_NUM_DATA_CHECK	130 // Frame boundary when subframecount = 31, so data can be checked on next subframe so 32


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
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st Test - CPRI 4x for LTE 20MHz
	  "LTE_20MHZ_FASTCM", // test name
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
        1,     // CPRI 4x for LTE 20Mhz - Link 0 - FASTCM
        0      // CPRI 4x for LTE 20Mhz - Link 2
    };

/*
 * define buffer for largest Ethernet frame
 * define buffer for largest 4b5b encoded Ethernet frame
 */
uint8_t ethPkt[ETH_FRAME_LEN] = {0};
uint8_t txCtrlBuff[CPRI_FAST_CM_SIZE];
uint8_t outputData[OUT_PACKET_LEN];
uint8_t preamble[8] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,0x55, 0x5D};

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAXPERLINK][NBSYMBOL*2];

/* Storing first sample of each packet to check packet sequence at runtime */
Complex16 firstSample[NBCHANNELMAXPERLINK][NBSYMBOL*2];

uint32_t nblink            = 1; 				 // default at one but set depending on test
uint32_t nbchanneltotal    = NBCHANNELPERLINK; // Default for 1 link LTE 20 MHz
uint32_t nbchannelSuperPacket = NBCHANNELPERLINK;

/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS][NBCHANNELMAXPERLINK];
/* Define Rx flow for control packets */
Cppi_RxFlowCfg rxFlowCpriCw[AIF2_CPRI_MAX_CW_SUBSTREAM];

/*
 * Slot and symbolcount counters
 */
volatile uint32_t slotcount = 0;
volatile uint32_t runcount = 0;
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
volatile uint32_t activeLink;

extern uint8_t		   keepAifOff;

uint32_t                 txPktCountCM, rxPktCount, errorcount;

/*
 * This flag is used for enabling and printing out exceptions at the end of the test
 */
uint32_t printexceptions = 1;

Swi_Handle slotRxSwi;
Swi_Handle slotTxSwi;

int gSentPacketCount = 0;
int gSentSlotCountArr[400]= {0};
int gSentEntrycountArr[400] = {0};
int gRcvdPacketCount = 0;
int gRcvdSlotCountArr[400]= {0};
int gRcvdEntrycountArr[400] = {0};

#define DUMP_SIZE 20
#pragma DATA_SECTION (rxpacket,".aifdescddr");
unsigned char rxpacket[DUMP_SIZE][ETH_FRAME_LEN] = {0};
#pragma DATA_SECTION (txpacket,".aifdescddr");
unsigned char txpacket[DUMP_SIZE][ETH_FRAME_LEN] = {0};

/* Monolithic descriptor region
 * for Lte IQ sample / symbol traffic
 */
#if DESCMSM == 1
#pragma DATA_SECTION(mono_region_ltesymbols,".aifdescmsm");
#else
#pragma DATA_SECTION(mono_region_ltesymbols,".aifdescddr");
#endif
#pragma DATA_ALIGN (mono_region_ltesymbols, 64);
uint8_t   mono_region_ltesymbols[NBDESCMAX * LTESYMBOLSIZE /** NBCHANNELMAXPERLINK*/];

/* Monolithic descriptor region
 * for Cpri control word traffic
 */
#if DESCMSM == 1
#pragma DATA_SECTION(mono_region_fastcm,".aifdescmsm");
#else
#pragma DATA_SECTION(mono_region_fastcm,".aifdescddr");
#endif
#pragma DATA_ALIGN (mono_region_fastcm, 64);
uint8_t   mono_region_fastcm[32 * CPRI_FAST_CM_SIZE];		// 32 descriptors of CPRI_FAST_CM_SIZE for fast c&m packets

//////////////////////////////////////////////////////////////////////////////
// Function prototypes
void myAifSetupAndFastCM();
void getcomplex(Complex16* payload,uint32_t index, uint32_t tx);
void slotRecyclingSwi();
void slotPushingSwi();

void dump_rxpacket(unsigned char *start_of_eth, char data_len, int pkt_cnt);
void dump_txpacket(unsigned char *start_of_eth, char data_len, int pkt_cnt);
void dump_packet(unsigned char *start_of_eth,int data_len);

#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

//////////////////////////////////////////////////////////////////////////////
// Function bodies

/*
 * User Isr that is called from pre-defined Isr in cslUtils.c
 */
#pragma FUNCTION_OPTIONS (slotIsr, "-o3" );
void slotIsr()
{
	slotcount++;

	getCpuTimestamp[slotcount] 	   = TSCL;
	getPhyTimerFrames[slotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
	getPhyTimerClk[slotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

	Swi_post(slotRxSwi);
	aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;

	Swi_post(slotTxSwi);
	aif2SymbolEgressCount[slotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;

	// Make sure no overflow occurs on the monitoring arrays
	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
}

void main(void)
{
	//create a task
    Task_Params  tskParams;

    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable

	CACHE_setL1DSize (CACHE_L1_32KCACHE);
	CACHE_setL1PSize (CACHE_L1_32KCACHE);

	Task_Params_init(&tskParams);
	tskParams.stackSize = 0x10000;
	tskParams.priority  = 1;

	Task_create(myAifSetupAndFastCM, &tskParams, NULL);

	//create a Swi that will be trigger by the slot Hwi to procces the recycling and pushing of packet at a slot time base.
	Swi_Params swiParams;

	Swi_Params_init (&swiParams);
	swiParams.arg0 = 0;
	swiParams.arg1 = 0;
	swiParams.priority = 3;
	swiParams.trigger = 0;
	slotRxSwi = Swi_create(slotRecyclingSwi, &swiParams, NULL);

	Swi_Params_init (&swiParams);
	swiParams.arg0 = 0;
	swiParams.arg1 = 0;
	swiParams.priority = 2;
	swiParams.trigger = 0;
	slotTxSwi = Swi_create(slotPushingSwi, &swiParams, NULL);

    BIOS_start();

}

void myAifSetupAndFastCM()
{

    uint32_t               idx, idx1, idx2, i, payloadLen, chan, value, iLoop =0, txSymCnt, rxCount=0, crc =0;
    uint16_t               testpass = 0;
	Complex16           *payloadPtr, sampleCheck;
    Cppi_MonolithicDesc *mono_pkt, *ptrMonoDesc;
    Qmss_Queue 			 descQueue;
    Cppi_DescTag 		 descTag;
    int32_t 				 outputlength =0;
    uint8_t               *crcptr =NULL;
   	uint32_t				 dataBuffLen, dataBuffLen5B;
   	uint32_t				 rxDataLength = 0;
    Qmss_Queue           queueInfo;

    System_printf("Beginning AIF2 LTE RF tests:\n");
    UTILS_waitForHw(100000);

    // AIF2 LLD programs AT event 5 with period 0.5ms in case of LTE
    aif2evt5_userIsr = slotIsr;

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
	aifObj.mode               = AIF_LTE_FDD_MODE;
	aifObj.superPacket        = false;

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
    					aifObj.pktDmaConfig.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_ltesymbols);
    					aifObj.pktDmaConfig.txNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
    					aifObj.pktDmaConfig.txDescSizeAxC[chan] = LTESYMBOLSIZE;
    					aifObj.pktDmaConfig.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_ltesymbols);
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
    		if (CPRI_FAST_CM) {
				for (i=0;i<AIF2_CPRI_MAX_CW_SUBSTREAM;i++)
				{

					// PktDma parameters for control words (CPRI control word FastC&M)
					aifObj.pktDmaConfig.txRegionCtrl[i]   = (Qmss_MemRegion)(UTILS_getMemRegionNum(mono_region_ltesymbols)+1);
					aifObj.pktDmaConfig.rxRegionCtrl[i]   = (Qmss_MemRegion)(UTILS_getMemRegionNum(mono_region_ltesymbols)+1);

					memset(&rxFlowCpriCw[i], 0, sizeof(Cppi_RxFlowCfg));
					rxFlowCpriCw[i].rx_dest_qnum     = MONO_RX_Q_CW + i;
					rxFlowCpriCw[i].rx_fdq0_sz0_qnum = MONO_RX_FDQ_CW + i;
					rxFlowCpriCw[i].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // monolythic
					rxFlowCpriCw[i].rx_sop_offset    = 12;   // desc header size
					aifObj.pktDmaConfig.hRxFlowCtrl[i]    = NULL;
#if	FASTCM == 1
					if(i == 0)
					{
						aifObj.pktDmaConfig.txNumDescCtrl[i]  = TX_FAST_CM_PKT_NUM;
						aifObj.pktDmaConfig.rxNumDescCtrl[i]  = RX_FAST_CM_PKT_NUM;
						aifObj.pktDmaConfig.txDescSizeCtrl[i] = (CPRI_FAST_CM_SIZE);
						aifObj.pktDmaConfig.rxDescSizeCtrl[i] = (CPRI_FAST_CM_SIZE);
						aifObj.pktDmaConfig.hRxFlowCtrl[i]    = &rxFlowCpriCw[i];
					}
#endif
				}
    		}

    		nbchanneltotal = NBCHANNELPERLINK*nblink;
    		memset(mono_region_fastcm,     0, sizeof(mono_region_fastcm));
    		memset(mono_region_ltesymbols, 0, sizeof(mono_region_ltesymbols));

    		System_printf("test: %s\n", testObjTab[ntest].name);
    		// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
    		UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELPERLINK;
				 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELPERLINK;
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
#if EVM_TYPE == 1 || EVM_TYPE == 2
				 // this test example doesn't handle the superpacket workaround required if using C6670 rev. 1
				 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
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

	        if (DSP_procId == 1)
				UTILS_printLinkConfig(&aifObj);

    		if (DSP_procId == 1)
    		{
    			UTILS_initTimer(TRIG_TIMER);
    		}

    		// initialization function for the AT timer and EDMA3 ISRs
    		UTILS_aifIntcSetup();

			// Compute default AIF2 parameters given this user configuration
			AIF_calcParameters(&aifObj);

			// initialization function for qmss and cppi low-level drivers
			if (CPRI_FAST_CM)
				UTILS_initQmss((uint32_t*)mono_region_ltesymbols, NBDESCMAX, LTESYMBOLSIZE /** NBCHANNELMAXPERLINK*/, CPRI_FAST_CM, (uint32_t*)mono_region_fastcm);
			else
				UTILS_initQmss((uint32_t*)mono_region_ltesymbols, NBDESCMAX, LTESYMBOLSIZE /** NBCHANNELMAXPERLINK*/, CPRI_FAST_CM, (uint32_t*)NULL);

			// initialization of Pktdma and Qmss resources given this user configuration
			AIF_initPktDma(&aifObj);

			for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++){
				if (aifObj.linkConfig[i].linkEnable) {
					activeLink = i;
				}
			}

			// initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
			AIF_initHw(&aifObj);

			/*
			 *  LTE IQ samples: Tx packet symbol initialization at once
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
					CACHE_wbInvL1d((void *)mono_pkt, LTESYMBOLSIZE, CACHE_WAIT);
			    }
			}
  	        if (CPRI_FAST_CM) {
#if FASTCM == 1
  	        	aifObj.hAif2Setup->linkSetup[activeLink]->pPeLinkSetup->bEnablePack[1] = false;
  	        	aifObj.hAif2Setup->linkSetup[activeLink]->pPdLinkSetup->bEnablePack[1] = false;

  				aifObj.hAif2Setup->linkSetup[activeLink]->pTmLinkSetup->pCpriTmSetup.TxPointerP = PPOINTER; // needs to be inline with bandwith allocated to fastCM control channel

  				aifObj.hAif2Setup->linkSetup[activeLink]->pPdLinkSetup->CpriCwPktDelimitor[0] = AIF2FL_CW_DELIM_NULLDELM;
  				aifObj.hAif2Setup->linkSetup[activeLink]->pPeLinkSetup->CpriCwPktDelimitor[0] = AIF2FL_CW_DELIM_NULLDELM;

  				aifObj.hAif2Setup->linkSetup[activeLink]->pPdLinkSetup->CpriCwNullDelimitor = 0XFF;
  				aifObj.hAif2Setup->linkSetup[activeLink]->pPeLinkSetup->CpriCwNullDelimitor = 0XFF;
#endif
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

			txPktCountCM = 0; rxPktCount = 0; errorcount = 0;

			AIF_startHw(&aifObj);

			init_crc32_table();
			processRxCmReset();

#ifdef LOOPBACK
			UTILS_triggerFsync(&aifObj);
#else
			if (DSP_procId == 1){
				UTILS_triggerFsync(&aifObj);
			}
#endif

			/*
			 * *********** FastCM traffic runtime starts here **********************
			 */

			while(1)
			{
				Task_yield();

				if ((CPRI_FAST_CM) && (slotcount>SLOT_NUM_FIRST_PUSH))
				{
#if FASTCM == 1
					/************ RX Side for fastCM traffic ***********************/
					/*Check if there is any descriptor available in Rx Ctrl Queue*/
					rxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQCtrl[0]);
					while (rxCount != 0)
					{
						if( processRxCM(rxCount, &outputlength, outputData, aifObj.pktDmaConfig.rxQCtrl[0], aifObj.pktDmaConfig.rxFqCtrl[0] ) == 1)
						{
							 //datalength of the recvd data (actual data)
							 rxDataLength = outputlength - (sizeof(preamble) + sizeof(crc));
							 //Reset the memory for the recvd data
							 memset(ethPkt ,0, sizeof(ethPkt));
							 //copy the data without the Preamble...
							 memcpy(ethPkt, outputData + sizeof(preamble), rxDataLength );
							 //Calculate the CRC over data rcvd from RRH...
							 crc = crc32(ethPkt,rxDataLength);
							 // crc check
							 if (crc != *(uint32_t*)(&outputData[outputlength-sizeof(crc)]))
							 {
								 errorcount++;
								 //System_printf("CRC error at fastCM receiver\n");
							 }
							 if (gRcvdPacketCount < DUMP_SIZE) dump_rxpacket((unsigned char*)outputData, outputlength,gRcvdPacketCount);
							 gRcvdSlotCountArr[gRcvdPacketCount]= ethPkt[0];
							 gRcvdEntrycountArr[gRcvdPacketCount] = rxCount;
							 gRcvdPacketCount++;
							 if (gRcvdPacketCount == 400) gRcvdPacketCount = 0;
							 rxPktCount++;
							 break;
					     }
						 rxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQCtrl[0]);
					}

					Task_yield();

					/************ TX Side for fastCM traffic ***********************/
					txSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqCtrl[0]);

					if ( (txSymCnt > 0) && (runcount != slotcount) )
					{

						runcount = slotcount;
						dataBuffLen = ETH_TEST_LEN;
						/*Reset the contents of the ethpkt*/
						memset( ethPkt, 0, sizeof(ethPkt));
						//Copy the 8 byte Premable...
						memcpy( ethPkt, preamble, sizeof(preamble));
						//Copy the data rcvd from the msgcom after preamble...
						memset( (ethPkt+sizeof(preamble)), runcount%256, dataBuffLen);
						// Simulate situation where 4b5b encoded payload data contains the Null delimiter
						if (runcount%256 == SLOT_NUM_FIRST_PUSH + 10) *(uint32_t*)(ethPkt+sizeof(preamble)+4) = 0x07070707;
						//update the overall length for preamble..
						dataBuffLen = dataBuffLen + 8; // original len + preamble Len
                        //Calculate the CRC over data rcvd from msgcom...
						//init_crc32_table();
						crc = crc32( (ethPkt + sizeof(preamble)),((dataBuffLen - sizeof(preamble))));
						//Copy the the CRC last 4 bytes at the end of the data....
						crcptr = (uint8_t *)&crc;
						ethPkt[dataBuffLen]= crcptr[0];	ethPkt[dataBuffLen+1]= crcptr[1]; ethPkt[dataBuffLen+2]= crcptr[2];	ethPkt[dataBuffLen+3]= crcptr[3];
						//update the overall length..
						dataBuffLen = dataBuffLen + 4; // original len + CRC len
						// pop from control stream free queue
						ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqCtrl[0]));

						queueInfo = Qmss_getQueueNumber(aifObj.pktDmaConfig.txFqCtrl[0]);
						descQueue.qMgr = queueInfo.qMgr;
						descQueue.qNum = queueInfo.qNum;
						Cppi_setReturnQueue(Cppi_DescType_MONOLITHIC,(Cppi_Desc *)ptrMonoDesc,descQueue);

						gSentSlotCountArr[gSentPacketCount]= runcount;
						gSentEntrycountArr[gSentPacketCount] = txSymCnt;
						if (gSentPacketCount < DUMP_SIZE) dump_txpacket((unsigned char*)ethPkt, dataBuffLen,gSentPacketCount);
						gSentPacketCount++;
						if (gSentPacketCount == 400) gSentPacketCount = 0;

                        dataBuffLen5B = ((dataBuffLen*10 )+17)/8;
						fourBitToFiveBitEncoder(ethPkt,txCtrlBuff,dataBuffLen, dataBuffLen5B);
						Cppi_setData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t*)txCtrlBuff, dataBuffLen5B);

						CACHE_wbL1d((void *)ptrMonoDesc, dataBuffLen5B + (3*4) , CACHE_WAIT);

						// Push packet on the control tx queue
						if (ptrMonoDesc != NULL) Qmss_queuePushDesc(aifObj.pktDmaConfig.txQCtrl[0], (uint32_t*)ptrMonoDesc);
						else {
							System_printf("Tx control packet descriptor is NULL\n");
						}
						txPktCountCM++;
					}
#endif // FASTCM == 1
				}

#if NEVER_END_TEST == 0
				// wait for one extra slots to run out of Rx free pkts
				if(slotcount >= (SLOT_NUM_DATA_CHECK + 2))
				{
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

			System_printf("Sent counter =%d Rcvd Conter =%d", gSentPacketCount, gRcvdPacketCount);
			for(iLoop=0; iLoop< gSentPacketCount || iLoop< gRcvdPacketCount ; iLoop++)
			{
				//System_printf("\nSent packet data=%d, and entry count at that time=%d", gSentSlotCountArr[iLoop], gSentEntrycountArr[iLoop]);
				//System_printf("\t Rcvd packet data=%d, and entry count at that time=%d", gRcvdSlotCountArr[iLoop], gRcvdEntrycountArr[iLoop]);
				if (iLoop < DUMP_SIZE) {
					if (memcmp(&rxpacket[iLoop][sizeof(preamble)],&txpacket[iLoop][sizeof(preamble)],dataBuffLen-sizeof(preamble)-sizeof(crc)) != 0) {
						System_printf("\t Rcvd packet data Incorrect", gRcvdSlotCountArr[iLoop], gRcvdEntrycountArr[iLoop]);
					}
				}
			}

			System_printf(" \nNumber of monolithic packets received in RX queue channel0: %d\n", Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]));
			System_printf(" Number of monolithic packets in TX free queue for channel0: %d\n", Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[0]));
			System_printf(" Number of monolithic packets received in RX queue channel1: %d\n", Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[1]));
			System_printf(" Number of monolithic packets in TX free queue for channel1: %d\n", Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txFqAxC[1]));

			/* Check received symbol packet data */
			for(chan =0; chan < nbchannelSuperPacket; chan++)
			{
				idx1 = 0;
				testpass = 1;
				rxCount = Qmss_getQueueEntryCount((aifObj.pktDmaConfig.rxQAxC[chan]));
				if (rxCount == 0) testpass =0;

				for (idx = 0; idx < NBSYMBOL*2; idx ++)
				{
					symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
					payloadPtr = (Complex16 *)symbolPkt[chan][idx];
					payloadPtr += 4; //skip pkt header and PS field (16 bytes)
					testpass = 1;
					if((idx%NBSYMBOL) == 0)
					{
						CACHE_invL1d((void *)payloadPtr, (LTESYMBOLSIZE-16), CACHE_WAIT);
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							getcomplex(&sampleCheck,idx1,chan);
							if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
							{
								testpass = 0; testcheck++;
							}
							idx1++;
						}

					} else {
						CACHE_invL1d((void *)payloadPtr, (LTESYMBOL2SIZE-16), CACHE_WAIT);
						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
						{
							getcomplex(&sampleCheck,idx1,chan);
								if ((payloadPtr[idx2].re != sampleCheck.re) || (payloadPtr[idx2].im != sampleCheck.im ))
							{
								testpass = 0; testcheck++;
							}
							idx1++;
						}
					}
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);
				}
				System_printf(" Test a) Monolithic Packet Data Recv: on chan: %d are :%d \n", chan, rxCount);
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

			if(CPRI_FAST_CM)
			{
#if FASTCM ==1
				if (rxPktCount == 0) {
					System_printf(" No CPRI Fast C&M received: FAIL\n");
					testcheck++;
				}
				if (errorcount) {
					System_printf(" CPRI Fast C&M data check: FAIL\n");
					testcheck++;
				}
				System_printf("Number of control packets Fast CM transmitted: %d\n", txPktCountCM);
				System_printf("Number of control packets Fast CM received: %d\n", rxPktCount);
#endif
			}
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

//Swi dedicated for packet recycling
#pragma FUNCTION_OPTIONS (slotRecyclingSwi, "-o3" );
void slotRecyclingSwi()
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;

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
	//aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
}

//Swi dedicated for packet pushing
#pragma FUNCTION_OPTIONS (slotPushingSwi, "-o3" );
void slotPushingSwi()
{
	int32_t                i,chan;
	uint32_t               payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;
	Complex16			*checkPtr;

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
				AIF_enableException(&aifObj);
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
		if (slotcount == (SLOT_NUM_DATA_CHECK-1)) UTILS_aif2ExceptIntDisable();
#if NEVER_END_TEST == 0
		}
#endif
	}
}

#pragma FUNCTION_OPTIONS (dump_rxpacket, "-o3" );
void dump_rxpacket(unsigned char *start_of_eth, char data_len, int pkt_cnt)
{
	int i;
	//rxpacket[pkt_cnt][0] = data_len;
	for (i=0;i<data_len;i++) {
		rxpacket[pkt_cnt][i] = *start_of_eth++;
	}
}

#pragma FUNCTION_OPTIONS (dump_txpacket, "-o3" );
void dump_txpacket(unsigned char *start_of_eth, char data_len, int pkt_cnt)
{
	int i;
	//txpacket[pkt_cnt][0] = data_len;
	for (i=0;i<data_len;i++) {
		txpacket[pkt_cnt][i] = *start_of_eth++;
	}
}

#pragma FUNCTION_OPTIONS (dump_packet, "-o3" );
void dump_packet(unsigned char *start_of_eth,int data_len)
{

	int iLoop =0 ;
	System_printf(" Start(length=%d) *** ", data_len);
	 /*print the IP packet and TCP packet*/
	  for(iLoop=0;iLoop<data_len;iLoop++)
	  {
	      System_printf(" %.2x ", *start_of_eth);
	      start_of_eth++;
	  }
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
        payload->re = y.re;
        payload->im = y.im;
}

/* Startup function */
void dspProcIdConfig(void) {
	uint16_t myboard	= EVM_TYPE;
    if (myboard == 0)  UTILS_configMasterSlave();
    else if (EVM_TYPE == 5) DSP_procId = 1;
    else if (myboard <= 2)  DSP_procId = EVM_TYPE; // For Advantech: myboard  = 1 (DSP_1) or 2 (DSP_2)
    else DSP_procId = (uint8_t)(EVM_TYPE - 2);       // For SCBP: myboard  = 3 (DSP_1) or 4  (DSP_2)
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    DSP_procId = 1;
#endif
}


//////////////////


