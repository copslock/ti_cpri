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

#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/cslr_tmr.h>
#include <ti/csl/csl_tmrAux.h>

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#if EVM_TYPE == 3 || EVM_TYPE == 4
#include <ti/platform/evmc6670l/platform_lib/include/evmc66x_pllc.h>
#endif
#include "cslUtils.h"
#include "mnavUtils.h"

#define     TEST_NUM            1
#define     TEST_ALL            0

#define     TRIG_TIMER          0



//define infinite loop for RADIO test
#if EVM_TYPE == 1 || EVM_TYPE == 2 || EVM_TYPE == 0 || EVM_TYPE == 6
#define     NEVER_END           0
#else
#define     NEVER_END           1
#endif
#if NEVER_END == 0
#define     LOOPBACK
#endif
//Value for Sin calculation
#define     Q_FACTOR            8191
#define     SAMP_FREQ_FLOAT     3.84  //3.84
#define     PI                  3.14159265358979323846

//define the tone mode, only one of the three patterns can be enable at a time
#define     SINGLE_TONE         1
#define     DUAL_TONE_ONE_TWO   0
#define     DUAL_TONE_FIVE_NINE 0

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2, 3 = SCBP_DSP_1, 4 = SCBP_DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // on Lyrtech  EVM , the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM , the Frequency is based on an external 153.6 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define     PLLC_PREDIV_CLK   	1       // PLLC_PREDIV_CLK - 1 set to register
#define     PLLC_PLLM_CLK     	8       // PLLC_PLLM_CLK - 1 set to register (983MHz CPU clock)
#define     CPRI_FAST_CM        0

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
        1     // CPRI 4x for UL WCDMA - Link 0
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


typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;

//////////////////////
// Global variables
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj                     testObjTab[TEST_NUM] = {
	{//1st Test - CPRI 4x for UL WCDMA - Link 0
	  "CPRI_4X_UL_L0", // test name
	 // link0          link1          link2          link3          link4          link5
	  {1,             0,             0,             0,             0,             0            },  // link enable
	  {4,             4,             4,             4,             4,             4            },  // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // outboundDataType
	  {DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL },  // inboundDataType
	  {DATA_WIDTH_16,  DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16, DATA_WIDTH_16},  // oinboundDataWidth
	  {AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0, AIF2FL_DIO_ENGINE_0}, //Dio engine
	}
};

// !!!!! ONLY ONE DIO ENGINE IN THIS TEST !!!!!
// Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
// DIO programming for WDMA UL:
//       wrap 1 = 2 Qwords * NUM_AxC = 32 bytes * NUM_AxC
//       wrap 2 = NumBlocks * wrap1 = 4 * 32 bytes * NUM_AxC
// DIO programming for WDMA DL:
//       wrap 1 = 1 Qwords * NUM_AxC = 16 bytes * NUM_AxC
//       wrap 2 = NumBlocks * wrap1 = 4 * 32 bytes * NUM_AxC
// We can't use a full buffer that will store an entire frame cause it is too memory costly.
// So we take buffer that will store half of a frame data: 76800 byte so 19200 words.
// numBlock for DL: 76800/wrap1 = 76800 / 16 = 4800.
// numBlock for UL: 76800/wrap1 = 76800 / 32 = 2400.
#define NUM_AxC 1
#pragma DATA_ALIGN (dio_data, 16)
uint32_t   dio_data[19200];
#pragma DATA_ALIGN (dio_result, 16)
uint32_t   dio_result[19200];


/* Setup Memory Region 0 for 32 * 720B Monolithic descriptors.
 * Mono descriptors will be 12 bytes, plus 704 bytes of payload(FastC&M) and
 * descriptor size must be a multiple of 16 bytes
 * So we take mono descriptor size of 720 Bytes.
 */

#pragma DATA_ALIGN (mono_region, 16)
uint8_t   mono_region[32 * 720];//payload size is 704 bytes for CPRI control word FastC&M + 12-byte

//Number of channel enable per link
uint32_t numAxC = NUM_AxC;

uint32_t printtimers = 1;
uint32_t getRadTimers[10];
uint32_t getUlRadTimers[10];

uint32_t printexceptions = 1;

Cppi_RxFlowCfg rxFlowCpriCw;

void getcomplex(Complex16* payload,uint32_t index)
{

	Complex16 x1;
	Complex16 y;
#if SINGLE_TONE == 1
                x1.re = (int16_t) ( (cos(2*PI*(1.0/10.0)*index)) * Q_FACTOR );
                x1.im = (int16_t) ( (sin(2*PI*(1.0/10.0)*index)) * Q_FACTOR );
                y = x1;
#elif DUAL_TONE_ONE_TWO == 1
            	float freq;
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
#elif DUAL_TONE_FIVE_NINE == 1
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
	if(EVM_TYPE == 6) DSP_procId = 1;

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
    Task_Params  tskParams;

    Task_Params_init(&tskParams);
    tskParams.stackSize =0x10000;

    Task_create(myAifTest, &tskParams, NULL);

    BIOS_start();

}

void myAifTest()
{
	
    uint32_t  idx, i, numWord, numBlock, activeLink;
    uint16_t  testpass = 0;
    uint32_t *ctrlCw, ctrlCwLen, hfnCount, entries, gie, pktCount;
    Cppi_MonolithicDesc *ptrMonoDesc;


    System_printf("Beginning AIF2 WCDMA CPRI single tone test:\n");
    UTILS_waitForHw(100000);

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	memset(&aifObj, 0, sizeof(aifObj));
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_DIO;
	aifObj.mode               = AIF_WCDMA_MODE;
	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
	else
		aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;

	numBlock = 4800;    // 19200 / 4 = 4800.
	numWord  = 4;		// 1 Quad Word.

	// Dio parameters for engine 0
	aifObj.dioConfig[0].out          = UTILS_local2GlobalAddr(&dio_data[0]);
	aifObj.dioConfig[0].outNumBlock  = numBlock;
	aifObj.dioConfig[0].in           = UTILS_local2GlobalAddr(&dio_result[0]);
#if NEVER_END == 0
	aifObj.dioConfig[0].inNumBlock	 = numBlock;
#else
	aifObj.dioConfig[0].inNumBlock	 = numBlock/2;
#endif

	if (CPRI_FAST_CM) {
		// PktDma parameters for control words (CPRI control word FastC&M)
		aifObj.pktDmaConfig.txRegionCtrl[0]   = UTILS_getMemRegionNum(mono_region);
		aifObj.pktDmaConfig.txNumDescCtrl[0]  = 8;
		aifObj.pktDmaConfig.txDescSizeCtrl[0] = 704;
		aifObj.pktDmaConfig.rxRegionCtrl[0]   = UTILS_getMemRegionNum(mono_region);
		aifObj.pktDmaConfig.rxNumDescCtrl[0]  = 8;
		aifObj.pktDmaConfig.rxDescSizeCtrl[0] = 704;

		memset(&rxFlowCpriCw, 0, sizeof(Cppi_RxFlowCfg));
		rxFlowCpriCw.rx_dest_qnum     = 900;
		rxFlowCpriCw.rx_fdq0_sz0_qnum = 2001;
		rxFlowCpriCw.rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // monolythic
		rxFlowCpriCw.rx_sop_offset    = 12;   // desc header size

		aifObj.pktDmaConfig.hRxFlowCtrl[0]    = &rxFlowCpriCw;
		aifObj.pktDmaConfig.hRxFlowCtrl[1]    = NULL;
		aifObj.pktDmaConfig.hRxFlowCtrl[2]    = NULL;
		aifObj.pktDmaConfig.hRxFlowCtrl[3]    = NULL;
	}

    /* Initialize descriptor regions to zero */
    memset(mono_region, 0, 32 * 720);

	// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
	UTILS_doCleanup(&aifObj,TRIG_TIMER);

	UTILS_initTimer(TRIG_TIMER);

	// initialization function for the AT timer and EDMA3 ISRs
	UTILS_aifIntcSetup();

    for(ntest=0;ntest<TEST_NUM;ntest++)
    {
 	    if (testEnable[ntest] == 1 )
 	    {
        	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
        	{
				 aifObj.linkConfig[i].linkEnable         = testObjTab[ntest].linkEnable[i];
				 if (aifObj.linkConfig[i].linkEnable == 1)
					 activeLink = i;
				 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
				 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_3P84MHZ;
				 aifObj.linkConfig[i].outboundDataType   = testObjTab[ntest].outboundDataType[i];
				 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[ntest].outboundDataWidth[i];
				 aifObj.linkConfig[i].inboundDataType    = testObjTab[ntest].inboundDataType[i];
				 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[ntest].inboundDataWidth[i];
				 aifObj.linkConfig[i].psMsgEnable        = CPRI_FAST_CM;
				 if (intLoopback == 1 ) aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
				  else	 aifObj.linkConfig[i].comType            = AIF2_2_AIF2;
				 aifObj.linkConfig[i].dioEngine          = testObjTab[ntest].dioEngine[i];
            }

        	aifObj.linkConfig[activeLink].numPeAxC = NUM_AxC;
        	aifObj.linkConfig[activeLink].numPdAxC = NUM_AxC;
#if NEVER_END == 1
        	//If we're doing the no_loopback mode and using the TSW we need to configure the inbound for UL data.
        	aifObj.linkConfig[activeLink].inboundDataType = DATA_TYPE_UL;
        	aifObj.linkConfig[activeLink].inboundDataWidth = DATA_WIDTH_8;
#if EVM_TYPE == 3 || EVM_TYPE == 4
        	aifObj.linkConfig[activeLink].piMin += 30; //delay introduce by Azcom FPGA.
#endif
#endif

        	System_printf("test: %s\n", testObjTab[ntest].name);
	        if (DSP_procId == 1)
				UTILS_printLinkConfig(&aifObj);

			// leave some time - might not be necessary in the end
			UTILS_waitForHw(1000);

			// compute parameters given this AIF configuration
			AIF_calcParameters(&aifObj);

			// initialization function for qmss and cppi low-level drivers
			if (CPRI_FAST_CM) UTILS_initQmss((uint32_t*)mono_region, 32, 720, 0, NULL);
			else              UTILS_initQmss((uint32_t*)NULL, 0, 0, 0, NULL);
			// initialization function for the AIF2 H/W of the TMS320TCI6616
			AIF_initDio(&aifObj);
			AIF_initPktDma(&aifObj);
			AIF_initHw(&aifObj);

			memset(dio_result, 0xFF, 19200 * 4);
			memset(dio_data, 0xAA, 19200 * 4);

#if EVM_TYPE != 0
			// Add the delay introduced by azcom FPGA (SCBP + TSW test)
			aifObj.linkConfig[0].piMin += 30;
#endif
			for (idx = 0; idx < (numBlock * NUM_AxC * numWord); idx++)
			{
				getcomplex((Complex16*) &dio_data[idx],idx);
			}

  	        if (CPRI_FAST_CM) {
				/* Init payload for 8 Monolithic packets for CPRI control words (1 packet sent every 10ms) */
				for (idx = 0; idx < 8; idx ++)
				{
					  ctrlCw = (uint32_t *)(mono_region + (idx * 720) + 12);
					  for (i = 0; i< 176;i++) ctrlCw[i] = i;
				}
  	        }

			// start AIF2 HW now that the DIO out buffer is pre-filled
			AIF_startHw(&aifObj);
			// let DSP2 go behond this point and start timer 0 on DSP1 for external Fsync
#ifdef LOOPBACK
			UTILS_triggerFsync(&aifObj);
#else
			if (DSP_procId == 1){
				UTILS_triggerFsync(&aifObj);
			}
#endif

			if (printexceptions == 1) {
				while (aifFsyncEventCount[1] <= 1);
				AIF_enableException(&aifObj);
 	    	}

			// wait for few Fsync before comparing data
			hfnCount = aifFsyncEventCount[1];
			pktCount = 0;
#if NEVER_END == 0
			while (aifFsyncEventCount[1] <= 100) {
#else
			while (1) {
#endif
				asm("    IDLE");
				if (CPRI_FAST_CM) {
					gie = _disable_interrupts();
					if (hfnCount < aifFsyncEventCount[1]) {
						hfnCount = aifFsyncEventCount[1];
						// pop from control stream free queue
						ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.txFqCtrl[0]));
						// push control data on tx queue
						Qmss_queuePushDesc(aifObj.pktDmaConfig.txQCtrl[0], (uint32_t*)ptrMonoDesc);
						// check if new packet available on the receive side
						entries = Qmss_getQueueEntryCount(aifObj.pktDmaConfig.rxQCtrl[0]);
						if (entries > 0){
							ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(aifObj.pktDmaConfig.rxQCtrl[0]));
							// Get payload address
							Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&ctrlCw, &ctrlCwLen);
							if (ctrlCwLen == 704) {
								for (i = 0; i< 176;i++) {
									if (ctrlCw[i] != i) System_printf("CPRI CW %d issue on frame %d\n",i,hfnCount);
									else ctrlCw[i] = 0; // reset Rx payload for next validation round
								}
							}
							// recycle control packet on rx free queue
							Qmss_queuePushDesc(aifObj.pktDmaConfig.rxFqCtrl[0], (uint32_t*)ptrMonoDesc);
							pktCount++;
						}
					}
					_restore_interrupts(gie);
				}
			}
			if (printexceptions == 1) {
				// disable interrupts before checking for data
				CSR&= 0xFFFFFFFE;
			}

      		/* Compare the DIO loopback data */
			testpass |= memcmp(&dio_data[0], &dio_result[0], NUM_AxC * numBlock * 4);

			if (printexceptions == 1) {
				AIF_printException(&aifObj);
			}
			UTILS_doCleanup(&aifObj,TRIG_TIMER);
			if (testpass != 0) {
				System_printf(" DIO CPRI Data for WCDMA: FAIL\n");
				testcheck++;
			}

		}
	}

    if (testcheck == 1) {
           testcheck = 0;
           System_printf("All tests have passed\n");
    } else {
           System_printf("Some tests have failed\n");
    }

	System_abort("Ending AIF2 WCDMA CPRI single tone test!!\n");
    
}
