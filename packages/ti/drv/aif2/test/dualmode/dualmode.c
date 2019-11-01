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
 * Target processors : TMS320C6616											*
 * 					   TMS320C6614	                                        *
 * 					   TMS320C6638	                                        *
 *                                                                          *
\****************************************************************************/

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>

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

//Lib for EDMA3
#include <ti/sdo/edma3/drv/edma3_drv.h>
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#include <ti/sdo/edma3/drv/sample/src/platforms/sample_tci6638k2k_cfg.c>
#include <ti/sdo/edma3/drv/sample/src/platforms/sample_tci6638k2k_int_reg.c>
#else
#include <ti/sdo/edma3/drv/sample/src/platforms/sample_c6670_cfg.c>
#include <ti/sdo/edma3/drv/sample/src/platforms/sample_c6670_int_reg.c>
#endif

//include for GPIO
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>


#if EVM_TYPE == 0
#include <EVM.h>
#endif

#if EVM_TYPE == 5
#include "appletonScbpSync.h"
#endif

#include "cslUtils.h"
#include "mnavUtils.h"
#include "edmalld.h"
#include "mathUtils.h"

/////////
//KeyStone II compatibility
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
#define CSL_TPCC_1								CSL_EDMACC_1
#define CSL_TPCC_2								CSL_EDMACC_2
#define CSL_QM_SS_CFG_QM_QUEUE_DEQUEUE_REGS		CSL_QMSS_CFG_QM_1_QUEUE_MANAGEMENT_REGS
#endif

#define 	_AIF2FL_DUMP		0

#define     TRIG_TIMER          7
#define     SYNC_TIMER          5
#define     PLL_TIMER           4


#define     Q_FACTOR            4095		// When the signal stays digital, reducing Q factor to avoid overflow in fft software
#define     SAMP_FREQ_FLOAT     3.84  	// Wcdma rate
#define     PI                  3.14159265358979323846
#define 	FREQ				0.1

#define     NEVER_END_TEST 		0
//#define     CPRI_RELAY_CFG 		2

/*
 * TEST_NUM is used to select wich tet to run. They are 4 different test available:
 *  - TEST_NUM = 0: Appleton test, 2 LTE AxC + 2 WCDMA AxC.
 *  - TEST_NUM = 1: Hawking test,  4 LTE AxC + 2 WCDMA AxC.
 *  - TEST_NUM = 2: Kepler2 test,  4 LTE AxC + 24 WCDMA AxC.
 *  - TEST_NUM = 3: Kepler2 test,  4 LTE AxC + 48 WCDMA AxC.
 */
#define TEST_NUM                0

// pre-proc constant: EVM_TYPE -> 0 = Lyrtech board, 1 = Advantech board DSP_1, 2 = Advantech setup DSP_2, 3 = SCBP_DSP_1, 4 = SCBP_DSP_2
#define     CLKIN1_INPUT_KHZ    122880  // on Lyrtech  EVM , the Frequency is based on an external 122.88 MHz clock
#if EVM_TYPE == 0 || EVM_TYPE == 3 || EVM_TYPE == 4
#define     SYSCLK_INPUT_KHZ    153600  // on Lyrtech EVM , the Frequency is based on an external 153.6 MHz clock
#else
#define     SYSCLK_INPUT_KHZ    122880  // on Advantech EVM , the Frequency is based on an external 122.88 MHz clock
#endif
#define     PLLC_PREDIV_CLK   	1       // PLLC_PREDIV_CLK - 1 set to register
#define     PLLC_PLLM_CLK     	8       // PLLC_PLLM_CLK - 1 set to register (983MHz CPU clock)
#define     CPRI_FAST_CM        0       // Disabling support for CPRI fast C&M as not required for this version of CPRI relay Software
#define     DIO_0               AIF2FL_DIO_ENGINE_0
#define     DIO_1               AIF2FL_DIO_ENGINE_1

volatile unsigned int verbose       = 1;
volatile unsigned int NB_ITERATIONS = 1;
volatile unsigned int ntest         = 0;
volatile unsigned int testcheck     = 1;
#ifdef LOOPBACK // Not used for CPRI relay SW
volatile unsigned int intLoopback   = 1;
volatile unsigned int swSync        = 1;
#else
volatile unsigned int intLoopback   = 0;
volatile unsigned int swSync        = 0;
#endif

//////////////////////
/*
 *
 */
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
        //link mode, need to select LTE or WCDMA
        AIF2_LinkMode			 mode[AIF_MAX_NUM_LINKS];
        } TestObj;


typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;

//////////////////////
// Global variables
AIF_ConfigObj               aifObj;
AIF_ConfigHandle            hConfigAif = &aifObj;

volatile TestObj                     testObjTab[4] = {
	{//CPRI 4X DUAL Mode Test: link0 and link1 configure for LTE, link2, link3 and link4 for WCDMA
	  "CPRI 4X DUAL Mode Test", // test name
	 // link0          link1          link2          link3          link4          link5
	  {1,             0,             1,             0,             0,             0            }, // link enable
	  {4,             4,             4,             4,             4,             4            }, // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15, DATA_WIDTH_16, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_1,         DIO_0,         DIO_1,         DIO_1},         // dio engine
	  {LTE,           LTE,           WCDMA,         LTE,           WCDMA,         WCDMA},
	},
	{
	  "CPRI 4X DUAL Mode Test", // test name
	  // link0          link1          link2          link3          link4          link5
	  {1,             1,             1,             0,             0,             0            }, // link enable
	  {4,             4,             4,             4,             4,             4            }, // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_1,         DIO_1,         DIO_1,         DIO_1},         // dio engine
	  {LTE,           LTE,           WCDMA,         WCDMA,         WCDMA,         WCDMA},
	},
	{
	  "CPRI 4X DUAL Mode Test", // test name
	  // link0          link1          link2          link3          link4          link5
	  {1,             1,             1,             1,             0,             0            }, // link enable
	  {4,             4,             4,             4,             4,             4            }, // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_1,         DIO_1,         DIO_1,         DIO_1},         // dio engine
	  {LTE,           LTE,           WCDMA,         WCDMA,         WCDMA,         WCDMA},
	},
	{
	  "CPRI 4X DUAL Mode Test", // test name
	  // link0          link1          link2          link3          link4          link5
	  {1,             1,             1,             1,             1,             0            }, // link enable
	  {4,             4,             4,             4,             4,             4            }, // link rate
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // outboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // outboundDataWidth
	  {DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL,  DATA_TYPE_DL }, // inboundDataType
	  {DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15, DATA_WIDTH_15},  // inboundDataWidth
	  {DIO_0,         DIO_0,         DIO_1,         DIO_1,         DIO_1,         DIO_1},         // dio engine
	  {LTE,           LTE,           WCDMA,         WCDMA,         WCDMA,         WCDMA},
	}
	};

/* Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
 */
/* Antenna carrier data
 * DIO programming for WDMA UL (for test purposes):
 *       wrap 1 = 2 Qwords * NUM_AxC = 32 bytes * NUM_AxC
 *       wrap 2 = NumBlocks * wrap1 = 4 * 32 bytes * NUM_AxC
 * So NumBlocks = 4, hence dio_result = 32 words * NUM_AxC
 * DIO programming for WDMA DL (for test purposes):
 *       wrap 1 = 1 Qwords * NUM_AxC = 16 bytes * NUM_AxC
 *       wrap 2 = NumBlocks * wrap1 = 8 * 16 bytes * NUM_AxC = 4 * 32 bytes * NUM_AxC
 * So NumBlocks = 8, hence dio_result = 32 words * NUM_AxC
 */

#if TEST_NUM == 0
#define LTE_CH           2              // this is LTE 20 Mhz, meaning that there are 2 AxC per link, so 4 for 2 link enabled.
#define LTE_NUM_BLOCK    320			// Match the width of 7 20Mhz LTE symbols.
#define LTE_BLOCK_DATA   960

#define WCDMA_CH         2				// data width of 15 bits, so 16 AxC per link in CPRI. 3 links enable for this test so 48 AxC.
#define WCDMA_NUM_BLOCK  48             // 32 block
#define	DIO_DMA_QW			4								// Num of 32-bit words in one quadword
#define	WCDMA_DIO_BUFFER_SIZE_AXC	(WCDMA_NUM_BLOCK*DIO_DMA_QW)	// Computes the size of buffer for one AxC in 32-bit words
#define	DIO_DMA_BURSTSTRIDE	(WCDMA_DIO_BUFFER_SIZE_AXC/DIO_DMA_QW)
#endif

#if TEST_NUM == 1
#define LTE_CH           4              // this is LTE 20 Mhz, meaning that there are 2 AxC per link, so 4 for 2 link enabled.
#define LTE_NUM_BLOCK    320			// Match the width of 7 20Mhz LTE symbols.

#define WCDMA_CH         2				// data width of 15 bits, so 16 AxC per link in CPRI. 3 link enble for this test so 48 AxC.
#define WCDMA_NUM_BLOCK  32             // 32 block
#endif

#if TEST_NUM == 2
#define LTE_CH           4              // this is LTE 20 Mhz, meaning that there are 2 AxC per link, so 4 for 2 link enabled.
#define LTE_NUM_BLOCK    320			// Match the width of 7 20Mhz LTE symbols.

#define WCDMA_CH         24				// data width of 15 bits, so 16 AxC per link in CPRI. 3 link enble for this test so 48 AxC.
#define WCDMA_NUM_BLOCK  32             // 32 block
#endif

#if TEST_NUM == 3
#define LTE_CH           4              // this is LTE 20 Mhz, meaning that there are 2 AxC per link, so 4 for 2 link enabled.
#define LTE_NUM_BLOCK    320			// Match the width of 7 20Mhz LTE symbols.

#define WCDMA_CH         48				// data width of 15 bits, so 16 AxC per link in CPRI. 3 link enble for this test so 48 AxC.
#define WCDMA_NUM_BLOCK  32             // 32 block
#endif

#pragma DATA_ALIGN (wcdma_dio_data, 16)
uint32_t   wcdma_dio_data[WCDMA_CH][WCDMA_DIO_BUFFER_SIZE_AXC];
#pragma DATA_ALIGN (wcdma_dio_result, 16)
uint32_t   wcdma_dio_result[WCDMA_CH][WCDMA_DIO_BUFFER_SIZE_AXC];
#pragma DATA_ALIGN (rxWcdmaBuffer, 16);
uint32_t  rxWcdmaBuffer[WCDMA_CH][WCDMA_DIO_BUFFER_SIZE_AXC];		// Used by fft software


#pragma DATA_SECTION(lte_dio_data,".aifdescmsm");
#pragma DATA_ALIGN (lte_dio_data, 16)
uint32_t   lte_dio_data[LTE_CH][16 * LTE_BLOCK_DATA]; //The Dio data should be long enought to store one symbol per AxC.
#pragma DATA_ALIGN (lte_dio_result, 16)
uint32_t   lte_dio_result[LTE_CH][16 * LTE_NUM_BLOCK]; //this buffer can store up to 5120 sample per AxC.

/* This buffer is used for the re-packetization check procedure, it store one symbol per AxC.*/
#pragma DATA_SECTION(packetBuffer,".aifdescmsm");
#pragma DATA_ALIGN (packetBuffer, 16)
uint32_t   packetBuffer[15360 * LTE_CH];


#define NUMDESC  14
#define	LTESYMBOLSIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL1_SIZE)*4)    // 2208: FFT size + CP first
#define LTESYMBOL2SIZE		((AIF2_LTE20_FFT_SIZE+AIF2_LTE20_CYPRENORMAL_SIZE)*4)     // 2192: FFT size + CP 2nd-6th

#define HOST_RX_Q   900
#define HOST_RX_FDQ 2000

#define MAX_SLOT    400

#pragma DATA_ALIGN (host_region, 16)
uint8_t   host_region[4 * 8 * 16 * 64];	//32 64 byte descriptors. the number of descriptor within a memory region should be a power of 2

//uint32_t  tmp0[NUM_DESC * LTE_CH];
//uint32_t  tmp1[NUM_DESC * LTE_CH];
//Cppi_RxFlowCfg rxFlowCpriDM[AIF_MAX_NUM_LINKS];

//global variable for packetization
Qmss_QueueHnd txFq[LTE_CH];
Qmss_QueueHnd txQ[LTE_CH];
Qmss_QueueHnd rxFq[LTE_CH];
Qmss_QueueHnd rxQ[LTE_CH];
Qmss_QueueHnd txCmplQueHnd[LTE_CH];

volatile uint32_t slotcount = 0;
volatile uint32_t symbolcount = 0;
volatile uint32_t packets_already_pushed = 0;
volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
volatile uint32_t getCpuTimestamp[MAX_SLOT];
volatile uint32_t getPhyTimerFrames[MAX_SLOT];
volatile uint32_t getPhyTimerClk[MAX_SLOT];
volatile uint32_t getRxPktCnt[MAX_SLOT]; // for first channel
volatile uint32_t symbolPushed[MAX_SLOT];
volatile uint32_t symbolPush = 0;
volatile uint32_t Fqentries[MAX_SLOT];
uint32_t EDMAcheck = 0;


CSL_Edma3Handle        hEdma;
CSL_Edma3Obj           edmaObjCSL;

/* Enabling AIF2 exceptions for the purpose of testing
 * In this case, exceptions are printed out at the end of the test, when interrupts are disabled
 */
uint32_t printexceptions = 0;
uint32_t enableexceptions = 0;
#if _AIF2FL_DUMP == 1
void dump_Aif2Fl_Setup (FILE *output, Aif2Fl_Setup *value);
#endif

/****************************
 * SYSBIOS Global object   **
 ****************************/
Swi_Handle symbolSwi;
Swi_Handle slotSwi;
Semaphore_Handle        semTestDone;
Semaphore_Handle        semTransfer;

/***************************************
 * Swi and Tasks Functions declaration**
 ***************************************/
void TransferTask();
void symbolIsrSwi();
void slotIsrSwi();


/***************************************
 * Function prototype                 **
 ***************************************/
void getcomplex(Complex16* payload, uint32_t index, uint32_t chan);
void frameIsr();

/******************************
 * Hwi functions developpment**
 ******************************/
void symbolIsr()
{
	Swi_post(symbolSwi);
}

/******************************
 * Functions bodies          **
 ******************************/
void getcomplex(Complex16* payload, uint32_t index, uint32_t chan)
{

	Complex16 x1;
	Complex16 y;
	if (chan == 0) //while waiting for Tx diversity FPGA image, send both antenna a single tone.
	{
		float freq;
		freq = FREQ;
		x1.re = (int16_t) ( (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		x1.im = (int16_t) ( (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		y = x1;
	} else if (chan == 1)	// FIXME: will be fix when Tx diversity is ready in FPGA
	{
		float freq;
		Complex16         x2;
		// 1 MHz signal
		freq = FREQ;
		x1.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		x1.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		// 2 MHz signal
		freq = FREQ*2;
		x2.re = (int16_t) (0.5 * (cos(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		x2.im = (int16_t) (0.5 * (sin(2*PI*freq*(1/SAMP_FREQ_FLOAT)*index)) * Q_FACTOR );
		// dual tone signal
		y.re = x1.re + x2.re;
		y.im = x1.im + x2.im;
	}

		payload->re = y.re;
		payload->im = y.im;
}

void frameIsr()
{
	uint32_t chan;

	for (chan=0;chan<WCDMA_CH;chan++)
	{
#if CPRI_RELAY_CFG == 2
		BufferConvertion(wcdma_dio_result[chan], rxWcdmaBuffer[chan], WCDMA_DIO_BUFFER_SIZE_AXC);
#endif
	}
}

void slotIsr()
{
	Swi_post(slotSwi);
}


/**********************************************************
 * Function call before the BIOS in order to use BSL lib **
 **********************************************************/
void myAifTest();

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

void main(void)
{
    Task_Params  tskParams;

    Task_Params_init(&tskParams);
    tskParams.stackSize =0x10000;
    tskParams.priority = 1;

    Task_create(myAifTest, &tskParams, NULL);

    Task_Params_init(&tskParams);
    tskParams.stackSize =0x4000;
    tskParams.priority = 2;

    Task_create(TransferTask, &tskParams, NULL);

    Semaphore_Params semParams;
    /* Initialize the default Semaphore parameters */
    Semaphore_Params_init(&semParams);

    /* Create the Binary Semaphore */
    semParams.mode = Semaphore_Mode_BINARY;
    semTransfer = Semaphore_create(0, &semParams, NULL);


    Swi_Params swiParams;

    Swi_Params_init (&swiParams);
    swiParams.arg0 = 0;
    swiParams.arg1 = 0;
    swiParams.priority = 1;
    swiParams.trigger = 0;
    symbolSwi = Swi_create(symbolIsrSwi, &swiParams, NULL);

    Swi_Params_init (&swiParams);
	swiParams.arg0 = 0;
	swiParams.arg1 = 0;
	swiParams.priority = 2;
	swiParams.trigger = 0;
	slotSwi = Swi_create(slotIsrSwi, &swiParams, NULL);

    BIOS_start();

}

/*************************************************
 *				Task and function				 *
 ************************************************/


/************ TRANSFER TASK FUNCTION ************/
/***********************************************/
void TransferTask()
{
	uint16_t value;
	uint32_t *scr;
	uint32_t *dest;
	uint32_t k;
	CSL_Status             status;

    Task_Params taskParams;
    Semaphore_Params semParams;

    /* Initialize the default Task parameters */
    Task_Params_init(&taskParams);

    /* Initialize the default Semaphore parameters */
    Semaphore_Params_init(&semParams);

    /* Create the Binary Semaphore */
    semParams.mode = Semaphore_Mode_BINARY;
    semTestDone = Semaphore_create(0, &semParams, NULL);

    value = Semaphore_pend(semTransfer,BIOS_WAIT_FOREVER);

    if (!value)
    {
    	System_printf("error in Transfer task\n");
    }

	/******** EDMA3 Driver related variables ********/
	edma_object edmaObj;
	unsigned int region = 0;

	/******** Initialize EDMA3 object first ********/
	edmaObj.hEdma		=	NULL;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
	edmaObj.edma_nbr 	= 	2;	//EDMA instance
#else
	edmaObj.edma_nbr 	= 	1;	//EDMA instance
#endif
	edmaObj.isError 	= 	EDMA3_DRV_SOK;


#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
	hEdma  = CSL_edma3Open(&edmaObjCSL,CSL_TPCC_2,NULL,&status);
#else
	hEdma  = CSL_edma3Open(&edmaObjCSL,CSL_TPCC_1,NULL,&status);
#endif

	/*
	 * AT event 6 (triggered at Lte symbol pace -> EDMA ch16 (K-I) pop/push AxC0
	 * -> chained event 17 (K-I) -> EDMA ch17 (K-I) pop/push AxC1
	 *
	 */
	for(k=0;k<LTE_CH;k++)
	{
		scr  = (uint32_t*)(CSL_QM_SS_CFG_QM_QUEUE_DEQUEUE_REGS + 0xC + (16 * txFq[k]));
		dest = (uint32_t*)(CSL_QM_SS_CFG_QM_QUEUE_DEQUEUE_REGS + 0xC + (16 * txQ[k]));
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
		edmaObj.chan[k].tccCh			=	EDMA3_DRV_HW_CHANNEL_EVENT_22+k;
		edmaObj.chan[k].chanNum			=	EDMA3_DRV_HW_CHANNEL_EVENT_22+k;
#else
		edmaObj.chan[k].tccCh			=	EDMA3_DRV_HW_CHANNEL_EVENT_16+k;
		edmaObj.chan[k].chanNum			=	EDMA3_DRV_HW_CHANNEL_EVENT_16+k;
#endif
		edmaObj.chan[k].triggerType		=	EDMA3_DRV_TRIG_MODE_EVENT; //EDMA3_DRV_TRIG_MODE_MANUAL; //
		edmaObj.chan[k].syncType		=	EDMA3_DRV_SYNC_AB;		//transfer stopped, when aCnt size bytes are written, until event
		edmaObj.chan[k].aCnt			=	4;						// 1register of 32bits = 32/8bytes = 4bytes
		edmaObj.chan[k].bCnt			=	1;						// there are 4 LSU registers to transfer in the table
		edmaObj.chan[k].cCnt			=	1;						// the overall is done 1 time only
		edmaObj.chan[k].srcBoffset		=	0x0000;
		edmaObj.chan[k].srcCoffset		=	0x0000;
		edmaObj.chan[k].dstBoffset		=	0x0000;
		edmaObj.chan[k].dstCoffset		=	0x0000;
		edmaObj.chan[k].srcAddr			=	(uint32_t) scr;
		edmaObj.chan[k].dstAddr			=	(uint32_t) dest;
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
		edmaObj.chan[k].queue			=	0;
#else
		edmaObj.chan[k].queue			=	3;
#endif
		if (k == (LTE_CH-1))
		{
			edmaObj.chan[k].chainChan	=	-1;	//chain channel 17
			edmaObj.chan[k].opt.tcchEn	=	EDMA3_DRV_TCCHEN_DIS;		// Transfer complete chaining enable
		} else {
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
			edmaObj.chan[k].chainChan	=	EDMA3_DRV_HW_CHANNEL_EVENT_23+k;	//chain channel 22
#else
			edmaObj.chan[k].chainChan	=	EDMA3_DRV_HW_CHANNEL_EVENT_17+k;	//chain channel 17
#endif
			edmaObj.chan[k].opt.tcchEn	=	EDMA3_DRV_TCCHEN_EN;		// Transfer complete chaining enable
		}
		edmaObj.chan[k].opt.itcchEn		=	EDMA3_DRV_ITCCHEN_DIS;		// Intermediate Transfer complete chaining enable
		edmaObj.chan[k].opt.itcintEn	=	EDMA3_DRV_ITCINTEN_DIS;	 	// Intermediate Transfer complete interrupt enable
		edmaObj.chan[k].opt.tcintEn		=	EDMA3_DRV_TCINTEN_DIS;		// Transfer complete interrupt enable

	}


	//each channel, related to an EDMA handle, must be in the same region
#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
	region = edmaObj.chan[0].chanNum/8;
#else
	region = edmaObj.chan[0].chanNum/16;	//regions are 16 multiple. For example, channel 0-15 are in the same region
#endif


	/******** Initialize EDMA3 ********/
	edmaObj.hEdma 		=	edmalldinit(edmaObj.edma_nbr, &edmaObj.isError, region);

#if defined(DEVICE_K2K) || defined(DEVICE_K2H)
	EDMA_intr_init(&edmaObj.hEdma, EDMA3_DRV_HW_CHANNEL_EVENT_22);
#else
	EDMA_intr_init(&edmaObj.hEdma, EDMA3_DRV_HW_CHANNEL_EVENT_16);
#endif

	/******** Open (request) EDMA channels ********/
	for (k=0;k<LTE_CH;k++)
	edmaOpenChannels(&edmaObj.hEdma,&edmaObj.chan[k]);
//	edmaOpenChannels(&edmaObj.hEdma,&edmaObj.chan[1]);

	/******** Start EDMA transfer ********/
	edma_start(&edmaObj.hEdma, edmaObj.chan[0].chanNum,edmaObj.chan[0].triggerType);	//in event mode, edma_start sets the EESR bit corresponding to the event

/* Remove all creations - to make graceful system exit */
    Semaphore_delete(&semTestDone);

}

void symbolIsrSwi()
{
	symbolPush++;
}

void slotIsrSwi()
{
	int32_t                i,k;
	uint32_t               rxSymCnt;
	Cppi_HostDesc 		*ptrHostDesc;

	slotcount++;


	/*
	 * Rx packet recycling until final check, we need available descriptors in Rx FDQs when actual data is pushed
	 */
	for (k=0;k<LTE_CH;k++)
	{
		rxSymCnt = Qmss_getQueueEntryCount(rxQ[k]);
		for(i=0;i<rxSymCnt;i++)
		{
			ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(rxQ[k]);
			// recycle symbol packet on rx free queue
			Qmss_queuePushDesc(rxFq[k], (uint32_t*)ptrHostDesc);
		}
	}

	aif2SymbolEgressCount[slotcount]  = aifObj.hFl->regs->DB_EDB_EOP_CNT;
	aif2SymbolIngressCount[slotcount] = aifObj.hFl->regs->AD_ISCH_EOP_CNT;
	getRxPktCnt[slotcount] = rxSymCnt;
	getCpuTimestamp[slotcount] 	   = TSCL;
	getPhyTimerFrames[slotcount]   = aifObj.hFl->regs->AT_PHYT_FRM_VALUE_LSBS;
	getPhyTimerClk[slotcount]      = aifObj.hFl->regs->AT_PHYT_CLKCNT_VALUE;

	for (k=0;k<LTE_CH;k++)
	{
		rxSymCnt = Qmss_getQueueEntryCount(txCmplQueHnd[k]);
		for(i=0;i<rxSymCnt;i++)
		{
			ptrHostDesc = (Cppi_HostDesc *)Qmss_queuePop(txCmplQueHnd[k]);
			/* Push descriptor back to free queue */
			Qmss_queuePushDescSize ((Qmss_QueueHnd) txFq[k], (uint32_t *) ptrHostDesc, 48);
		}
	}

	/* PktDMA activity monitoring */

	// Make sure no overflow occurs on the monitoring arrays
	if (slotcount == (MAX_SLOT-1)) slotcount = 0;
}

void myAifTest()
{


    uint32_t  idx, i, chan;
    uint32_t  testpass[LTE_CH];
    uint32_t  testpass1 = 0;
    uint32_t  testpass2 = 0;


    System_printf("Starting AIF2 DUAL MODE CPRI 4X Software\n");

    UTILS_waitForHw(100000);

    aif2evt5_userIsr = slotIsr;
    aif2evt6_userIsr = symbolIsr;
    fsevt1_userIsr = frameIsr;

	// Take AIF out of power saver
	AIF_enable();

	// General parameters
	memset(&aifObj, 0, sizeof(aifObj));
	aifObj.aif2ClkSpeedKhz    = (uint32_t)SYSCLK_INPUT_KHZ;
	aifObj.protocol           = AIF2FL_LINK_PROTOCOL_CPRI;
	aifObj.pktdmaOrDioEngine  = AIF2FL_DIO;                      // The DUAL MODE use DIO engine, mandatory.
	aifObj.mode               = AIF_LTE_WCDMA_MODE;                // Configure for DUAL MODE
	if (swSync == 0)
		aifObj.aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
	else
		aifObj.aif2TimerSyncSource= AIF2FL_SW_SYNC;


	// DIO engine 0 for LTE
	// DIO engine 1 for WCDMA
	aifObj.dioConfig[0].out          = UTILS_local2GlobalAddr(&lte_dio_data[0]);
	aifObj.dioConfig[0].in           = UTILS_local2GlobalAddr(&lte_dio_result[0]);
	aifObj.dioConfig[0].outNumBlock  = LTE_BLOCK_DATA;
	aifObj.dioConfig[0].inNumBlock	 = LTE_NUM_BLOCK;
	aifObj.dioConfig[0].sampleRate = AIF_SRATE_30P72MHZ;
	aifObj.dioConfig[1].out          = UTILS_local2GlobalAddr(&wcdma_dio_data[0][0]);
	aifObj.dioConfig[1].in           = UTILS_local2GlobalAddr(&wcdma_dio_result[0][0]);
	aifObj.dioConfig[1].outNumBlock  = WCDMA_NUM_BLOCK;
	aifObj.dioConfig[1].sampleRate = AIF_SRATE_3P84MHZ;
#if CPRI_RELAY_CFG == 2
	aifObj.dioConfig[1].inNumBlock	 = WCDMA_NUM_BLOCK/2;
#if EVM_TYPE == 5
	aifObj.linkConfig[3].nodeTx      = 0;
	aifObj.linkConfig[3].nodeRx      = 2;
#endif
#else
	aifObj.dioConfig[1].inNumBlock	 = WCDMA_NUM_BLOCK;
#endif

#if TEST_NUM == 0
	aifObj.linkConfig[2].numPeAxC = WCDMA_CH;
	aifObj.linkConfig[2].numPdAxC = WCDMA_CH;
#endif

#if TEST_NUM == 1
	aifObj.linkConfig[2].numPeAxC = 2;
	aifObj.linkConfig[2].numPdAxC = 2;
#endif

#if TEST_NUM == 2
	aifObj.linkConfig[3].numPeAxC = 8;
	aifObj.linkConfig[3].numPdAxC = 8;
#endif

	aifObj.pktDmaConfig.hRxFlowCtrl[0]    = NULL;
	aifObj.pktDmaConfig.hRxFlowCtrl[1]    = NULL;
	aifObj.pktDmaConfig.hRxFlowCtrl[2]    = NULL;
	aifObj.pktDmaConfig.hRxFlowCtrl[3]    = NULL;

	memset(host_region, 0, 4 * 8 * 16 * 64);

	System_printf("Software configuration: %s\n", testObjTab[TEST_NUM].name);

	// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness
	UTILS_doCleanup(&aifObj,TRIG_TIMER);

	if (DSP_procId == 1)
	{
		UTILS_initTimer(TRIG_TIMER);
	}

	// initialization function for the AT timer and EDMA3 ISRs
	UTILS_aifIntcSetup();

#if EVM_TYPE == 5
//	UTILS_boardsSync(SYNC_TIMER,PLL_TIMER); // required board sync process when using CPRI relay setups
#endif

	// Initialize all link strutures based on the test objects
	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		 aifObj.linkConfig[i].linkEnable         = testObjTab[TEST_NUM].linkEnable[i];
		 aifObj.linkConfig[i].linkRate           = (Aif2Fl_LinkRate)testObjTab[TEST_NUM].linkRate[i];
		 aifObj.linkConfig[i].sampleRate		 = AIF_SRATE_30P72MHZ; //AIF_SRATE_3P84MHZ
		 aifObj.linkConfig[i].outboundDataType   = testObjTab[TEST_NUM].outboundDataType[i];
		 aifObj.linkConfig[i].outboundDataWidth  = testObjTab[TEST_NUM].outboundDataWidth[i];
		 aifObj.linkConfig[i].inboundDataType    = testObjTab[TEST_NUM].inboundDataType[i];
		 aifObj.linkConfig[i].inboundDataWidth   = testObjTab[TEST_NUM].inboundDataWidth[i];
		 aifObj.linkConfig[i].mode               = testObjTab[TEST_NUM].mode[i];
		 aifObj.linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
		 aifObj.linkConfig[i].psMsgEnable        = 0;
		 if (intLoopback == 1) aifObj.linkConfig[i].comType            = AIF2_LOOPBACK;
		  else	 aifObj.linkConfig[i].comType            = AIF2_2_AIF2;
		 aifObj.linkConfig[i].dioEngine          = testObjTab[TEST_NUM].dioEngine[i];
	}
	aifObj.linkConfig[2].sampleRate		    = AIF_SRATE_3P84MHZ; //AIF_SRATE_3P84MHZ
	aifObj.linkConfig[4].comType            = AIF2_LOOPBACK;

#if CPRI_RELAY_CFG == 2
	aifObj.linkConfig[4].outboundDataType  = DATA_TYPE_DL;
	aifObj.linkConfig[4].outboundDataWidth = DATA_WIDTH_16;
	aifObj.linkConfig[4].inboundDataType   = DATA_TYPE_UL;
	aifObj.linkConfig[4].inboundDataWidth  = DATA_WIDTH_8;
	aifObj.linkConfig[4].pe2Offset    		= (320 - 60);
#endif

	UTILS_printLinkConfig(&aifObj);

	// leave some time - might not be necessary in the end
	UTILS_waitForHw(1000);

	// compute parameters given this AIF configuration
	AIF_calcParameters(&aifObj);

	// initialization function for qmss and cppi low-level drivers
	memset(lte_dio_result,0xff,sizeof(lte_dio_result));
	UTILS_hostLocalTransferInit((uint32_t*)host_region,(Qmss_MemRegion) 0, &lte_dio_result[0][0], &packetBuffer[0], txFq, txQ, rxFq, rxQ, txCmplQueHnd,LTE_CH);

	// initialization function for the AIF2 H/W of the TMS320TCI6616
	AIF_initDio(&aifObj);
	AIF_initPktDma(&aifObj);
	AIF_initHw(&aifObj);

    memset(lte_dio_result, 0xFF, sizeof(lte_dio_result));//clear dest buffer
    memset(lte_dio_data, 0x00, sizeof(lte_dio_data));//clear src buffer

    memset(wcdma_dio_result, 0xFF, sizeof(wcdma_dio_result));//clear dest buffer
    memset(wcdma_dio_data, 0xAA, sizeof(wcdma_dio_data));//clear src buffer

    //fill source data for LTE
	for(chan =0; chan < LTE_CH; chan++){//8 LTE AxC
       for (idx = 0; idx < LTE_BLOCK_DATA * 16; idx++) { // 7 symbols
		lte_dio_data[chan][idx] = (chan << 24) + idx;
//		getcomplex((Complex16*) &lte_dio_data[chan][idx],idx,chan);
       }
	}


	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[1].DmaBlockAddrStride = 1;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[1].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[1].NumQuadWord 		= AIF2FL_AD_1QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->EgrDioEngine[1].DmaBurstLen 		= AIF2FL_AD_1QUAD;
#if CPRI_RELAY_CFG == 2
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBlockAddrStride = 2;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBurstLen 		 = AIF2FL_AD_2QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].NumQuadWord 		 = AIF2FL_AD_2QUAD;
#else
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBlockAddrStride = 1;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBurstAddrStride = DIO_DMA_BURSTSTRIDE;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].DmaBurstLen 		 = AIF2FL_AD_1QUAD;
	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[1].NumQuadWord		 = AIF2FL_AD_1QUAD;

//	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBlockAddrStride = 1;
//	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstAddrStride = 1 * LTE_NUM_BLOCK;
//	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].DmaBurstLen 		 = AIF2FL_AD_1QUAD;
//	aifObj.hAif2Setup->commonSetup->pAdDioSetup->IngrDioEngine[0].NumQuadWord		 = AIF2FL_AD_1QUAD;
#endif


	/* Initialize Tx IQ samples for each of the AxCs */
	for(chan=0;chan<WCDMA_CH;chan++)
	{
		for (idx = 0; idx < (WCDMA_NUM_BLOCK * DIO_DMA_QW); idx++)
		{
//			getcomplex((Complex16*) &wcdma_dio_data[chan][idx],idx,chan);
			wcdma_dio_data[chan][idx] = (chan << 24) + idx;
		}
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

	Semaphore_post(semTransfer);

	// configure AIF2 HW now that all required CSL structures have been filled
	AIF_startHw(&aifObj);

#ifdef LOOPBACK
	UTILS_triggerFsync(&aifObj);
#else
	if (DSP_procId == 1)
	{
		//Trig the SCBP by sending a rising edge through timer0 on appleton side connected to PHYSYNC (SCBP side)
		UTILS_triggerFsync(&aifObj);
	}
#endif

	// enable exceptions if required after 10ms
	if (enableexceptions == 1) {
		while (aifFsyncEventCount[1] <= 1);
		AIF_enableException(&aifObj);
	}

#if NEVER_END_TEST == 1
    while (aifFsyncEventCount[1] <= 100000) {
#else
	while (aifFsyncEventCount[1] <= 100) {
#endif
		asm("    IDLE");
	}
	UTILS_doCleanup(&aifObj,TRIG_TIMER);

	// Check data for LTE buffer
	for(chan=0;chan<LTE_CH;chan++)
	{
		testpass[chan]=0;
		testpass[chan] |= memcmp(&lte_dio_data[chan][0], &packetBuffer[15360*chan], (15360*4));	/* Compare the DIO loopback data */
	}

#if CPRI_RELAY_CFG == 2
	for(chan=0;chan<WCDMA_CH;chan++)
	{
		BufferConvertion(wcdma_dio_result[chan], rxWcdmaBuffer[chan], WCDMA_DIO_BUFFER_SIZE_AXC);
//		testpass1 |= memcmp(&wcdma_dio_data[chan][0], &rxWcdmaBuffer[chan][0], sizeof(wcdma_dio_result[chan]));
	}
#else
	// Check data for WCDMA buffer
	testpass1 |= memcmp(&wcdma_dio_data[0][0], &wcdma_dio_result[0][0], sizeof(wcdma_dio_result));
#endif
	testpass2 |= memcmp(&packetBuffer[15360], &lte_dio_data[0][0], (15360*4));

	if (enableexceptions == 1) {
		// disable interrupts before checking for data
		CSR&= 0xFFFFFFFE;
	}

	if (printexceptions == 1) {
		AIF_printException(&aifObj);
	}

	for(i=0;i<LTE_CH;i++)
	{
		if (testpass[i] != 0) {
			System_printf(" data check for lte dio buffer %d: FAIL\n",i);
			testcheck++;
		} else {
			System_printf(" data check for lte dio buffer %d: PASS\n",i);
		}
	}

	if (testpass1 != 0) {
			System_printf(" data check for wcdma dio buffer: FAIL\n");
			testcheck++;
		} else {
			System_printf(" data check for wcdma dio buffer: PASS\n");
		}

    if (testcheck == 1) {
       testcheck = 0;
       System_printf("All tests have passed\n");
    } else {
       System_printf("Some tests have failed\n");
    }

	System_abort("End of Program!!\n");

}
