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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>

#include <ti/csl/csl.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>
#include <ti/csl/csl_cache.h>
#if !defined(SIMULATOR_SUPPORT)
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#endif

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#if EVM_TYPE == 0
#include <EVM.h>
#endif

#include "cslUtils.h"
#include "mnavUtils.h"

/*
 * This is an AIF2 LLD example test for a LTE 20MHz configuration.
 * The test is configured to work at the granularity of a sub-frame in CPRI or OBSAI mode.
 * For LTE, AIF2 requires usage of monolithic descriptor packets with a 16-byte header.
 * In the header, a protocol specific word is required to indicate the antenna carrier
 * number, the symbol number and the direction (Ingress/Egress)
 * That means that every ms, the test will push and pop from the AIF2 HW queues:
 * 14 symbols * numAxCs per link * numLinks * 2 (for Egress/Ingress traffic)
 * This then makes a total of 112 outstanding monolithic packets in the case of
 * LTE 20MHz / 2 AxCs / 2 Links / both directions
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
 *
 * Kepler 2 Simulator:
 * This test support also a configuration to run on Kepler 2 simulator. The Kepler 2 specific piece of code are labeled with the flag DEVICE_K2K.
 * The test basically is the same but due to the simulator restriction we had to make some
 * minor change regarding the timing:
 * The simulator's limitations are:
 * 	1.	Simulator only support Rad symbol strobe for event, so it will be auto converted to
 * 	 Rad symbol strobe whatever you set in the code.
 *	2.	Simulator doesn’t support UL and DL Rad timers
 *	3.	Simulator doesn’t care about all protocol layer stuff like PD, PE
 *	4.	Ingress DIO frame event offset (only for OBSAI) should be around half to the real case. (if real : 1200, it should be around 600 for simulator)
 *
 *	The Multicore Navigator has changed from K1 to K2. The Hw have now two queue managers.
 *	For this test we choose to use the internal linking Ram (32K descriptor) in shared mode.
 * 	These limitations have been taken into account in the LLD itself, but beacause of the
 * 	Rad symbol strobe event limitation we had to modify on wich interrupt we start the ISR.
 * 	As our application run on a Sub frame based, it have to wait 14 interrupts to process the
 * 	push and pull the descriptor (14 symbols in one sub frame).
 *
 *  As the simulator is way slower than the real device, the test is performing on less frame otherwise it takes too long to run.
 *	The simulator specific piece of code are those under the flag SIMULATOR_SUPPORT.
 */


#define     TRIG_TIMER          0

#define     TEST_NUM            1
#define     TEST_ALL            0
#define 	_AIF2FL_DUMP		0
#define     NBCHANNELPERLINK	2
#define     NBCHANNELMAX		(NBCHANNELPERLINK*2)                                        // 2-link max for this test
#define     NBSYMBOL			AIF2_LTE_SYMBOL_NUM											// Number of symbols per slot
#define		LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#if EVM_TYPE == 0 || EVM_TYPE == 6
#define     DESCMSM				1															// Using MSM and reduing the number of mono desc
#define		NBDESCMAX			128                                                         // Descriptor count rounded to the next power of 2
																							// 14 sym * 2 AxCs * 2 pingpong * 2 directions
#else
#define     DESCMSM				0															// mono desc region going to DDR
#define		NBDESCMAX			256                                                         // Descriptor count rounded to the next power of 2
																							// 14 sym * 2 AxCs * 2 pingpong * 2 directions * 2 links
#endif
#define		MAX_SF				300															// For debug, size of monitoring arrays

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0
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
#ifdef TESTCPRI
    		1,     // CPRI 4x for LTE 20Mhz - Link 0,1
#else // OBSAI
    		1,     // OBSAI 4x for LTE 20Mhz - Link 0,1
#endif
    };

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
AIF_DataTraceObj			aif2TraceObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
#ifdef TESTCPRI
	{//1st Test - CPRI 4x for DL LTE 20Mhz - 2 consecutive Links - If EVM_TYPE == 0, only 1 link used
	  "CPRI_4X_LTE_20MHZ", // test name
	 // link0          link1          link2          link3          link4          link5
	  {0,             0,             0,             0,             1,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
	},
#else
	{//1st Test - OBSAI 4x for DL LTE 20Mhz - 2 consecutive Links - If EVM_TYPE == 0, only 1 link used
	  "OBSAI_4X_LTE_20MHZ", // test name
	 // link0          link1          link2          link3          link4          link5
	  {1,             1,             0,             0,             0,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0,         DIO_0},          // not applicable for LTE test
	},
#endif
   };


/* Define Rx queue base index for ingress traffic and the associated Rx free descriptor queue base index */
/* For Kepler 2, the Tx free queue is hardcoded to 900 (+i when i = channel number)*/
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
uint8_t   mono_region_test[NBDESCMAX * LTESYMBOLSIZE];

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPkt[NBCHANNELMAX][NBSYMBOL*2*2];

// AIF2 data trace
#pragma DATA_SECTION(trace_data,".aif2tracedata")
#pragma DATA_ALIGN (trace_data, 16)
uint32_t  trace_data[16 * 2000];
#pragma DATA_SECTION(trace_framingdata,".aif2tracedata")
#pragma DATA_ALIGN (trace_framingdata, 16)
uint32_t  trace_framingdata[4 * 2000];

/* Dummy variables for debug */
uint32_t nblink            = 1; 				 // default at one but set depending on test
uint32_t nbchanneltotal    = NBCHANNELPERLINK; // Default for 1 link
uint32_t nbchanneltotal2    = NBCHANNELPERLINK; // Default for 1 link
/*
 * Define Rx flows for AIF2 ingress traffic
 * The flow will tell AIF2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[AIF_MAX_NUM_LINKS][NBCHANNELPERLINK];

/*
 * Subframe and symbolcount counters
 */
volatile uint32_t subframecount = 0;
volatile uint32_t dbgsubframecount = 0;
volatile uint32_t zerocount = 0;
volatile uint32_t symbolcount = 0;
volatile uint32_t aif2SymbolEgressCount[MAX_SF];
volatile uint32_t aif2SymbolIngressCount[MAX_SF];
volatile uint32_t deltaEgressCount[MAX_SF];
volatile uint32_t deltaIngressCount[MAX_SF];
volatile uint32_t getCpuTimestamp[MAX_SF];
volatile uint32_t getPhyTimerFrames[MAX_SF];
volatile uint32_t getPhyTimerClk[MAX_SF];
volatile uint32_t getRxPktCnt[MAX_SF]; // for first channel
volatile uint32_t rxPktCnt[NBCHANNELPERLINK * 2]; // assuming 2 link max
volatile uint32_t symbolNum[NBCHANNELPERLINK][140]; // assuming 2 link max

volatile uint32_t currentsubframe = 0;
volatile uint8_t 	wait4sync = 0;
volatile uint32_t peoff = 0;
volatile uint32_t peon = 0;

volatile uint32_t lossSyncComplete = 0;

extern uint8_t		   keepAifOff;
extern uint8_t		   onLoss;

/*
 * This flag is used for enabling and printing out exceptions at the end of the test
 */
uint32_t printexceptions = 1;
#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
#define SF_NUM_FIRST_PUSH	10
#define SF_NUM_DATA_CHECK	20
#else
#define SF_NUM_FIRST_PUSH	30  // The first push needs to happen on the last subframe before frame boundary
#define SF_NUM_DATA_CHECK	20000 // Make the test run for one frame (10 subframes)
#define SF_NUM_PE_OFF		4213 // Make the test run for one frame (10 subframes)
#define SF_NUM_PE_ON		5685 // Make the test run for one frame (10 subframes)
#endif

/*
 * User Isr that is called from pre-defined Isr in cslUtils.c
 */
void subframeIsr()
{
	int32_t                i,chan;
	uint32_t               rxSymCnt, payloadLen;
	Cppi_MonolithicDesc *ptrMonoDesc;
	uint32_t              *payloadPtr;

		subframecount++;
		dbgsubframecount++;
		// Make sure no overflow occurs on the monitoring arrays
		if (dbgsubframecount == (MAX_SF)) dbgsubframecount = 0;

		getCpuTimestamp[dbgsubframecount] 	   = TSCL;
		getPhyTimerFrames[dbgsubframecount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
		getPhyTimerClk[dbgsubframecount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

#ifdef TESTCPRI
		if((subframecount > (SF_NUM_FIRST_PUSH+1)) && (subframecount < SF_NUM_DATA_CHECK)){

			UTILS_aif2IsOnLoss(&aifObj,3);

			if (SF_NUM_PE_OFF == subframecount)
			{
				UTILS_aif2ExceptIntDisable();
				onLoss = 1;
			}

			if (SF_NUM_PE_ON == subframecount)
			{
				AIF_enableException(&aifObj);
				onLoss = 0;
			}

			if ((onLoss == 1) && (wait4sync == 0))
			{
				if ((currentsubframe == 0) && (subframecount%10 == 0)) //stop pushing packet just before the next frame.
				{
					currentsubframe = subframecount;
					nbchanneltotal2 = 0;								//stop pushing packet in Tx queues
				}
				if (subframecount == (currentsubframe + 1))				//disable PE and PD 1 ms after stopping the push of packet.
				{
					for(chan = 0; chan < nbchanneltotal; chan++)
					{
						AIF_disablePeCh(&aifObj,chan);
						AIF_disablePdCh(&aifObj,chan);
					}
					wait4sync = 1;
					peoff = subframecount;								//this is for debug purpose (to verify that we disable the PE and PD at the right subframe)
				}
			}

			if (wait4sync == 1 && onLoss == 0) 							//if we have the link status in Sync again, then we restart the PD channel first
			{
				if (subframecount%10 == 0)								//Just before the frame boundary
				{
					currentsubframe = 0;
					for(chan = 0; chan < nbchanneltotal; chan++)
					{
						AIF_enablePdCh(&aifObj,chan);
					}
					rxPktCnt[0] = 0;
					rxPktCnt[1] = 0;
					wait4sync = 0;
					peon = subframecount;
				}
			}

			if (subframecount == (peon + 10))							// And 1 frame after enabling the PD channels, we can enabled the PE channel as well.
			{
				for(chan = 0; chan < nbchanneltotal; chan++)
				{
					AIF_enablePeCh(&aifObj,chan);
				}
				nbchanneltotal2 = 2;									//start pushing packet to Tx queues
				lossSyncComplete = 1;
			}
		}
#endif
		/*
		 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
		 */
#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
		if (subframecount < SF_NUM_DATA_CHECK-1){ // we don't recycle at the last push in order to receive 28 descriptors for the final check.
#else
		if (subframecount < SF_NUM_DATA_CHECK){
#endif
			for(chan = 0; chan < nbchanneltotal; chan++)
			{
				rxSymCnt = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[chan]);

				for(i=0;i<rxSymCnt;i++)
				{
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
					// Invalidate symbol packet header and get the updated payload address + length
#if !defined(SIMULATOR_SUPPORT)
					CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
#endif
					Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);

					if((rxPktCnt[chan]%NBSYMBOL) == 0)
					{
						 //check payload length for first symbol
//						if (payloadLen != (LTESYMBOLSIZE-16))
//							printf("unexpected payload length:%d, %d\n",(LTESYMBOLSIZE-16),payloadLen);
					} else {
						//check payload length for the other 6 symbols
//						if (payloadLen != (LTESYMBOL2SIZE-16))
//							printf("unexpected payload length:%d, %d\n",(LTESYMBOL2SIZE-16),payloadLen);
					}

					if (payloadPtr[6] == 0) zerocount++;

					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
					if (lossSyncComplete == 1)
					{
						lossSyncComplete = 2;
					}
					symbolNum[chan][(subframecount%10)*14 + i] = (payloadPtr[0] >> 7);

					// recycle symbol packet on rx free queue
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*)ptrMonoDesc);
					rxPktCnt[chan]++;
				}
			}
		}
		getRxPktCnt[dbgsubframecount]      = rxPktCnt[0];

		/*
		 * In this Isr, at this subframe, push all symbols for all AxCs, links at once
		 * This is a one-shot example. It assumes that all symbolPkt were first popped
		 * from the FDQs, and properly populated. AIF2 will returned them in FDQs following
		 * the actual transmission via AIF2 protocol encoder.
		 * Here we are pushing packets just on the first valid frame boundary, so subframe 10
		 * All packets will be transmitted and received by the next node on subframe 11.
		 * This is when we do the data checking for this test. Note that AIF2 Rx PktDMA channels
		 * will starve at this point since we don't perform Rx packet recycling at runtime.
		 * We don't keep pushing packets beyond subframe 10 either.
		 */
		if((subframecount == SF_NUM_FIRST_PUSH) || (subframecount == (SF_NUM_FIRST_PUSH+1))){
			for(chan = 0; chan < nbchanneltotal; chan++)
			{
				for(i=symbolcount;i<(symbolcount + NBSYMBOL*2);i++)
				{
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)symbolPkt[chan][i]);
				}
			}
			symbolcount += NBSYMBOL*2;
			zerocount = 0;
		}


		if((subframecount > (SF_NUM_FIRST_PUSH+1)) && (subframecount < SF_NUM_DATA_CHECK)){

			for(chan = 0; chan < nbchanneltotal2; chan++)
			{
				for(i=symbolcount;i<(symbolcount + NBSYMBOL*2);i++)
				{
					// pop from AxC Tx free descriptor queue
					ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqAxC[chan]));
					// Recreate PS data
#if !defined(SIMULATOR_SUPPORT)
					CACHE_invL1d((void *)ptrMonoDesc, 16, CACHE_WAIT);
#endif
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
					payloadPtr[0] = (uint32_t )(0x00008000 + chan + (i << 7));//add symbol number into PS field
#if !defined(SIMULATOR_SUPPORT)
					CACHE_wbL1d((void *)payloadPtr, 4, CACHE_WAIT); // writeback the updated PS data in MSM
#endif
					Qmss_queuePushDesc((aifObj.pktDmaConfig.txQAxC[chan]), (uint32_t*)ptrMonoDesc);
				}
			}
			symbolcount += NBSYMBOL*2;
			if (symbolcount == AIF2_LTE_FRAME_SYMBOL_NUM) symbolcount = 0;

			if (subframecount == (SF_NUM_DATA_CHECK-1)) {
#ifndef TESTCPRI
				AIF_disableDataTrace(&aifObj);
#endif
				UTILS_aif2ExceptIntDisable();
			}
		}

		if (subframecount == (SF_NUM_FIRST_PUSH+1)) {
#ifndef TESTCPRI
				aif2TraceObj.linkIndex        			= AIF2FL_LINK_0;
				aif2TraceObj.dataCaptureEnable			= 1;
				aif2TraceObj.framingDataCaptureEnable	= 1;
				aif2TraceObj.syncCaptureEnable			= 1;
				aif2TraceObj.dataAddress				= &trace_data[0];
				aif2TraceObj.framingDataAddress			= &trace_framingdata[0];
				aif2TraceObj.dtWrap 					= 2000 - 1;
				memset(trace_data, 0xFF, sizeof(trace_data));
				memset(trace_framingdata, 0xFF, sizeof(trace_framingdata));
				AIF_enableDataTrace(&aifObj,&aif2TraceObj);
#endif
				AIF_enableException(&aifObj);
		}

		/* PktDMA activity monitoring */
		aif2SymbolEgressCount[dbgsubframecount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
		aif2SymbolIngressCount[dbgsubframecount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
		deltaEgressCount[dbgsubframecount] = aif2SymbolEgressCount[dbgsubframecount] - aif2SymbolEgressCount[(dbgsubframecount-1)];
		deltaIngressCount[dbgsubframecount] = aif2SymbolIngressCount[dbgsubframecount] - aif2SymbolIngressCount[(dbgsubframecount-1)];

}


int main(void)
{
    uint32_t               idx, idx2, i, firsterror, payloadLen;
    uint16_t               testpass = 0;
    uint16_t               myboard	= EVM_TYPE;
	uint32_t               monoRxCount, monoTxCount;
    uint32_t              *payloadPtr;
    uint32_t               chan, rx_count, value;
    Cppi_MonolithicDesc *mono_pkt;
    Qmss_Queue 			 descQueue;
    Cppi_DescTag 		 descTag;
    Qmss_Queue           queueInfo;

#if !defined(SIMULATOR_SUPPORT)
    /* Make shared memory (MSM) non cacheable for the purpose of testing */
    CSL_XMC_invalidatePrefetchBuffer();
    CACHE_setMemRegionInfo(12,1,0); // MAR12 - cacheable (always), not prefetchable
    CACHE_setMemRegionInfo(13,1,0); // MAR13 - cacheable (always), not prefetchable
#endif

    printf("Beginning AIF2 LTE testing:\n");
//    UTILS_waitForHw(100000);
    
    if (myboard == 0)  UTILS_configMasterSlave();
    else DSP_procId = EVM_TYPE ; //in case of Advantech it could be board 1 or 2

#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
   DSP_procId = 1;
#endif

   	// AIF2 LLD programs AT event 6 with period 1ms in case of LTE
   	aif2evt6_userIsr = subframeIsr;

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
#ifdef TESTCPRI
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
#else // OBSAI
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_OBSAI;
#endif
	aifObj.pktdmaOrDioEngine  = AIF2FL_CPPI;
	aifObj.mode               = AIF_LTE_FDD_MODE;
	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_CHIP_INPUT_SYNC;
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
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
    				testObjTab[ntest].linkEnable[1] = 0; //if on Kepler 2 simulator, we only test on one link.
#endif
    				// PktDma parameters
    				for (idx=0 ; idx<NBCHANNELPERLINK ; idx++)
    				{
    					aifObj.pktDmaConfig.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
    					aifObj.pktDmaConfig.txNumDescAxC[chan]  = NBSYMBOL*2*2; // double num of Pkts
    					aifObj.pktDmaConfig.txDescSizeAxC[chan] = LTESYMBOLSIZE;
    					aifObj.pktDmaConfig.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region_test);
    					aifObj.pktDmaConfig.rxNumDescAxC[chan]  = NBSYMBOL*2*2; // double num of Pkts
    					aifObj.pktDmaConfig.rxDescSizeAxC[chan] = LTESYMBOLSIZE;

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
    		nbchanneltotal2 = nbchanneltotal;

    		memset(mono_region_test, 0, NBDESCMAX * LTESYMBOLSIZE);

    		// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
    		//UTILS_doCleanup(&aifObj,TRIG_TIMER);

        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_30P72MHZ;
				 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				 aifObj.linkConfig[i].cpriPackMode		 = AIF2_LTE_CPRI_1b1;
				 aifObj.linkConfig[i].numPeAxC			 = NBCHANNELPERLINK;
				 aifObj.linkConfig[i].numPdAxC			 = NBCHANNELPERLINK;
				 aifObj.linkConfig[i].psMsgEnable        = 0;
				 if (intLoopback == 1 ) aifObj.linkConfig[i].comType = AIF2_LOOPBACK;
				 else	 				aifObj.linkConfig[i].comType = AIF2_2_AIF2;
				 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i]; //NA for pkDMA
            }

        	printf("test: %s\n", testObjTab[ntest].name);
	        if (DSP_procId == 1)
				UTILS_printLinkConfig(&aifObj);

			// initialization function for the interrupt controllers
			UTILS_aifIntcSetup();

			// Compute default AIF2 parameters given this user configuration
			AIF_calcParameters(&aifObj);

			// initialization function for qmss and cppi low-level drivers
			UTILS_initQmss((uint32_t*)mono_region_test, NBDESCMAX, LTESYMBOLSIZE, 0, NULL);

			// initialization of Pktdma and Qmss resources given this user configuration
			AIF_initPktDma(&aifObj);

#ifndef TESTCPRI
			// enable DIO Rx for data trace purpose
			{
			  Cppi_RxChInitCfg     dioRxCfg;
			  uint8_t                isAllocated;

				    // Enable channel 128 for DIO mode
					// enable Rx and Tx for DIO
					memset(&dioRxCfg, 0, sizeof(dioRxCfg));
					dioRxCfg.channelNum = 128;
					dioRxCfg.rxEnable   = Cppi_ChState_CHANNEL_DISABLE;
					aifObj.pktDmaConfig.dioRxChAxC  = Cppi_rxChannelOpen(aifObj.pktDmaConfig.hCppi, &dioRxCfg, &isAllocated);
					Cppi_channelEnable (aifObj.pktDmaConfig.dioRxChAxC);
			}
#endif

#if defined(SIMULATOR_SUPPORT) && (defined(DEVICE_K2K) || defined(DEVICE_K2H))
			// Setting aif mono mode to 1 in K2 simulators
			{CSL_Cppidma_tx_channel_configRegs* pktDMATxCfg = (CSL_Cppidma_tx_channel_configRegs*)CSL_AIF_CFG_PKTDMA_TX_CFG_REGS;
	         CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[0].TX_CHANNEL_GLOBAL_CONFIG_REG_B, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE, (uint32_t)1);
	         CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[1].TX_CHANNEL_GLOBAL_CONFIG_REG_B, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE, (uint32_t)1);
	         CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[2].TX_CHANNEL_GLOBAL_CONFIG_REG_B, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE, (uint32_t)1);
	         CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[3].TX_CHANNEL_GLOBAL_CONFIG_REG_B, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_B_TX_AIF_MONO_MODE, (uint32_t)1);
			}
#endif

			// Adjust AIF2 timing parameters if calcParameters didn't implement this LTE case or doesn't match the application Tx/Rx delays
			for (i= 0 ; i<AIF_MAX_NUM_LINKS; i++){
				if (aifObj.linkConfig[i].linkEnable) {
					aifObj.linkConfig[i].pe2Offset   = 310;
					aifObj.linkConfig[i].deltaOffset = aifObj.linkConfig[i].pe2Offset + 70;
					aifObj.linkConfig[i].piMin       = aifObj.linkConfig[i].pe2Offset + 70;
				}
			}

			// initialization function for the AIF2 H/W CSL structure (can still be overridden afterwards)
			AIF_initHw(&aifObj);

			/*
			 *  LTE Tx packet symbol initialization at once
			 */
			for(chan = 0;chan < nbchanneltotal;chan++)
			{
			    for(idx = 0;idx < (NBSYMBOL*2*2);idx++)
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
						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++) payloadPtr[idx2] = (chan << 24) +  (idx << 16) + idx2;
					} else {
						//payload data setup(other six symbols)
						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++) payloadPtr[idx2] = (chan << 24) + (idx << 16) + idx2;
					}

					//Create PS data
					Cppi_getPSData (Cppi_DescType_MONOLITHIC, Cppi_PSLoc_PS_IN_DESC, (Cppi_Desc*)mono_pkt, (uint8_t**)&payloadPtr, &payloadLen);
					payloadPtr[0] = (uint32_t )(0x00008000 + chan + (idx << 7));//add symbol number into PS field

					//Tx queue push will be done in subframe ISR at once
			    }
			}
			/*
			 * Perform block writeback on L1D to make sure all symbol packets are coherent with MSM
			 */
#if !defined(SIMULATOR_SUPPORT)
			for(idx = 0;idx < (NBDESCMAX * NBCHANNELPERLINK);idx++) {
				CACHE_wbInvL1d((void *)(mono_region_test+(idx*LTESYMBOLSIZE)), LTESYMBOLSIZE, CACHE_WAIT);
			}
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
			// start AIF2 HW and PktDMA and then wait for frame synchronization
			AIF_startHw(&aifObj);
			
			UTILS_initTimer(TRIG_TIMER);
			monoRxCount = 0;
			monoTxCount = 0;
	        monoRxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQAxC[0]);
	        monoTxCount = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.txQAxC[0]);
			if (DSP_procId == 1)
			{
				UTILS_triggerFsync(&aifObj);
			}

			while(1)
			{

				asm (" IDLE");
				// wait for one extra subframe to run out of Rx free pkts
				if(subframecount == (SF_NUM_DATA_CHECK + 2))
				{
					//AT disable all events and halt timer
					CSR&= 0xFFFFFFFE;
					//keepAifOff = 1;
					UTILS_doCleanup(&aifObj,TRIG_TIMER);
					break;
				}
			}

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
				testpass = 1;
				firsterror = 0;
				rx_count = Qmss_getQueueEntryCount((aifObj.pktDmaConfig.rxQAxC[chan]));
				if (rx_count == 0) testpass =0;
				for (idx = 0; idx < rx_count; idx ++)
				{
					symbolPkt[chan][idx] = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQAxC[chan]));
					payloadPtr = (uint32_t *)symbolPkt[chan][idx];
					payloadPtr += 4; //skip pkt header and PS field (16 bytes)
					testpass = 1;
					if((idx%NBSYMBOL) == 0)
					{

						for (idx2 = 0; idx2 < (LTESYMBOLSIZE-16)/4; idx2 ++)
						{
							if (payloadPtr[idx2] != (chan << 24) +  (idx << 16) + idx2)
							{
								if (testpass == 1 ) firsterror =idx2;
									testpass = 0; testcheck++;
							}
						}
					} else {

						for (idx2 = 0; idx2 < (LTESYMBOL2SIZE-16)/4; idx2 ++)
						{
							if (payloadPtr[idx2] != (chan << 24) + (idx << 16) + idx2)
							{
								if (testpass == 1 ) firsterror =idx2;
									testpass = 0; testcheck++;
							}
						}
					}
					if (testpass == 0)
					{
						printf(" Test a) Packet Payload: FAIL on chan:%d\t symbol:%d\t received:%d\t expected:%d\t position:%d \n", \
								 chan , idx, payloadPtr[firsterror],(chan << 24) +  (idx << 16) + firsterror, firsterror);
						firsterror =0;
						testcheck++;
					}
					Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqAxC[chan], (uint32_t*) symbolPkt[chan][idx]);

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
			if (value != nbchanneltotal*NBSYMBOL*2*2) printf(" Test b2) Monolithic Packet Tx Complete Descriptor Counts:%d FAIL\n",value);
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
			if (value != nbchanneltotal*NBSYMBOL*2*2) printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d FAIL\n",value);
			else printf(" Test b4) Monolithic Packet Rx Free Descriptor Counts:%d PASS\n",value);

			UTILS_doCleanup(&aifObj,TRIG_TIMER);

			if (printexceptions == 1)
			{
			   	AIF_printException(&aifObj);
			}

			printf("\nEnding %s test\n", testObjTab[ntest].name);
 	    }
    }

    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed\n");
    } else {
       printf("Some tests have failed\n");
    }

    return (0);
}

////////////

