/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifdef RUNTIME

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_xmcAux.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/dfe/dfe_drv.h>

#include <ti/drv/iqn2/test/utils/dfess.c>
#include <ti/drv/iqn2/test/utils/cslUtils.h>
#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#include <ti/drv/iqn2/test/utils/lteUtilsMulti.h>
#include <aidCfg.h>


#define 	LTE_ON

#define     TEST_NUM_MAX            1

//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".iqn2descmsm")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 64)
#endif
uint8_t   mono_region[NBDESCMAX * ((SYMBOLSIZE20/64)+1) * 64]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching

/*
 * Define an array that can contain all descriptor addresses.
 * This way, all packets can be popped from the FDQ and prepared at once for the purpose of this test.
 */
Cppi_MonolithicDesc* symbolPktCtrl[1][NBSYMBOL];

/* Storing first sample of each packet to check packet sequence at runtime */
//Complex16 firstSample[NUM_AXCS_LTE][NBSYMBOL*2];

/*
 * Define Rx flows for IQN2 ingress traffic
 * The flow will tell IQN2 from which FDQ to pop new symbol descriptors
 */
Cppi_RxFlowCfg rxFlow[NUM_AXCS_LTE_AID];
Cppi_RxFlowCfg rxFlowCtrl[4];

/* IQN2 FL for runtime access */
PktDmaConfigObj  pktDmaObj;
Iqn2Fl_Obj      Iqn2Obj;// Iqn2 CSL object
Iqn2Fl_Handle   hIqn2Fl;// Iqn2 handle
Iqn2Fl_Status   iqn2Status;
Iqn2Fl_Context  Iqn2Context;//Iqn2 context
Iqn2Fl_InitCfg  Iqn2InitCfg;

/* PKTDMA */

uint32_t ctrlArg; // Ctrl Argument;

extern int DFESS_ProgPll(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);

volatile unsigned int int4_result = 0;
volatile unsigned int testcheck = 1;    // reports pass fail at the end of the test

// Register/Memory Read/Write Functions
void reg_write_32(Uint32 addr,Uint32 write_data)
{
  *(Uint32 *)(addr) = write_data;
}
void int4_isr(){ 	//interrupt
	slotCount(hIqn2Fl);
	slotRecycling(&pktDmaObj);
	slotPushing(&pktDmaObj);
	packetCount(hIqn2Fl, 1);
	int4_result++;
}


int main(void)
{

    uint32_t  chan, idx;
    Iqn2Fl_TopVCSwResetStbSetup reset;
    uint32_t pll_mult, pll_divmode;

    UTILS_setCache();

#if DFE_RATE == 20
    pll_mult = 8 - 1; // to 737.28
    pll_divmode = 0;  // div by 2 = 368.64
#elif DFE_RATE == 10
    pll_mult = 6 - 1; // to 737.28
    pll_divmode = 1;  // div by 2 = 368.64
#endif

    if(DFESS_ProgPll_csl(1-1/*prediv*/, pll_mult /*mult*/, 1-1 /*postdiv*/, pll_divmode /*divmode*/) == 0){
    printf("FATAL: DFE PLL failed!\n");
    return -1;
    }

    //DFE pll should be set correctly before turning on DFE power domains
    UTILS_iqn2DfeEnable(1); // need to init PktDMA part of IQN2

    printf("\nStarting LTE runtime program\n");
    memset(&pktDmaObj, 0, sizeof(pktDmaObj));
    // Initialize CSL library, this step is required
    Iqn2Fl_init(&Iqn2Context);
    // Open Iqn2 and get handle
    Iqn2InitCfg.dev.bases[0].cfgBase = (void*)CSL_IQN_CFG_REGS;
    hIqn2Fl = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, &Iqn2InitCfg, &iqn2Status);
    if ((hIqn2Fl == NULL) || (iqn2Status != IQN2FL_SOK))
    {
       printf ("\nError opening CSL_IQN");
       exit(1);
    }

	// initialization function for the interrupt controllers
	atevt2_userIsr = int4_isr;
    // initialization function for the interrupt controllers
	UTILS_iqn2IntcSetup();

	// PktDma parameters
    pktDmaObj.firstAxC	= 0;
		pktDmaObj.numAxC    = NUM_AXCS_LTE_AID;
    chan = 0;
	for (idx=0 ; idx<NUM_AXCS_LTE_AID ; idx++)
	{
		pktDmaObj.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.txNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.txDescSizeAxC[chan] = SYMBOLSIZE20;
		pktDmaObj.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.rxNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.rxDescSizeAxC[chan] = SYMBOLSIZE20;
        pktDmaObj.txDesc2SizeAxC[idx] = SYMBOL2SIZE20;
		memset(&rxFlow[idx], 0, sizeof(Cppi_RxFlowCfg));
		rxFlow[idx].rx_dest_qnum     = MONO_RX_Q+chan;
		rxFlow[idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
		rxFlow[idx].rx_fdq1_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_fdq2_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_fdq3_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS
        rxFlow[idx].rx_psinfo_present = 1;

		pktDmaObj.hRxFlowAxC[chan]   = &rxFlow[idx];
		pktDmaObj.hRxFlowCtrl[chan]  = NULL;
		chan++;
	}

	memset(mono_region, 0, sizeof(mono_region));

	// initialization function for qmss and cppi low-level drivers
	UTILS_initQmss((uint32_t*)mono_region, NBDESCMAX, ((SYMBOLSIZE20/64)+1) * 64, NBDESCMAX);

	UTILS_initPktDma(&pktDmaObj);
	UTILS_initCppiChannel(&pktDmaObj);

	load_data(&pktDmaObj);

	printf("\nReady for LTE20MHz 4 AxCs test\n");

    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");

        if(int4_result >= SLOT_NUM_DATA_CHECK+2)// Wait 1 ms (no frame delay required for DFE)
         {
        	reset.sw_reset =1;//IQN2 full reset
        	Iqn2Fl_hwControl(hIqn2Fl, IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB, (void *)&reset);
            break;
         }
    }

    lteFinalCheck(&pktDmaObj);
	
	if (testcheck == 1) {
	   testcheck = 0;
	   printf("All tests have passed \n");
	} else {
	   printf("Some tests have failed : testcheck = %d\n", testcheck);
	}	

    printf("\nEnding LTE20MHz 4AxCs test\n");
    UTILS_resetQmss(&pktDmaObj);
#ifdef USERM
    UTILS_resetRm();
#endif

    return(0);

}

#endif

//////////////////////////////
