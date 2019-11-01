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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_chip.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>

#include <ti/drv/iqn2/test/utils/cslUtils.h>
#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#include <ti/drv/iqn2/test/utils/lteUtilsMulti.h>
#include <ti/drv/iqn2/test/utils/wcdmaUtils.h>
#include "ailCfg.h"

#define _IQN2FL_DUMP		0

#define NUM_AXCS_LTE_AIL0       NUM_AXCS_LTE20_AIL0+NUM_AXCS_LTE10_AIL0+NUM_AXCS_LTE5_AIL0+NUM_AXCS_LTE15_AIL0

#if NUM_AXCS_LTE_AIL0 != 0
#define 	LTE_ON
#endif 
#if NUM_AXCS_WCDMA_AIL0 != 0
#define		WCDMA_ON
#endif

#define     TEST_NUM_MAX            1

#ifdef LTE_ON
//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".iqn2descddr")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 64)
#endif
uint8_t   mono_region[NBDESCMAX * ((SYMBOLSIZE20/64)+1) * 64]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching
#endif

#ifdef WCDMA_ON
//Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
#pragma DATA_SECTION(wcdma_dio_data,".intData_sect")//use SL2 memory for WCDMA test
#pragma DATA_ALIGN (wcdma_dio_data, 64)
uint32_t  wcdma_dio_data[NUM_AXCS_WCDMA_AIL0][NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];
#pragma DATA_SECTION(wcdma_dio_result,".intData_sect")//use SL2 memory for WCDMA test
#pragma DATA_ALIGN (wcdma_dio_result, 64)
uint32_t  wcdma_dio_result[NUM_AXCS_WCDMA_AIL0][NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];
#endif

/* IQN2 Global structures and variables  */
IQN2_ConfigObj	 iqn2LldObj;
PktDmaConfigObj  pktDmaObj;
IQN2_At2EventObj at2Evt2;

TestObj testObjTab = {
    "CPRI test",                                             //test name
    IQN2FL_PROTOCOL_CPRI,                                   //CPRI or OBSAI
    {AIL0_RATE,             AIL1_RATE           },          //link rate.
    {NUM_AXCS_WCDMA_AIL0,   NUM_AXCS_WCDMA_AIL1 },          //number of WCDMA AxC. Here it is just LTE application so no WCDMA AxC are required.
    {NUM_AXCS_LTE20_AIL0,   NUM_AXCS_LTE20_AIL1 },          //number of LTE AxC.
    {NUM_AXCS_LTE10_AIL0,   NUM_AXCS_LTE10_AIL1 },          //number of LTE AxC.
    {NUM_AXCS_LTE5_AIL0,    NUM_AXCS_LTE5_AIL1  },          //number of LTE AxC.
    {NUM_AXCS_LTE15_AIL0,   NUM_AXCS_LTE15_AIL1 },          //number of LTE AxC.
    {300,                   300                 },          //peoffset
    {IQN2_COM_SD_LOOPBACK,  IQN2_COM_SD_LOOPBACK},
};



/*
 * Define Rx flows for IQN2 ingress traffic
 * The flow will tell IQN2 from which FDQ to pop new symbol descriptors
 */
#ifdef LTE_ON
Cppi_RxFlowCfg rxFlow[NUM_AXCS_LTE_AIL0];
extern volatile uint32_t slotcount;
#endif

volatile unsigned int testcheck     = 1;
volatile unsigned int int4_result = 0;

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif


void int4_isr(){    //interrupt
#ifdef LTE_ON
	slotCount(iqn2LldObj.hFl);
	slotRecycling(&pktDmaObj);
	slotPushing(&pktDmaObj);
	packetCount(iqn2LldObj.hFl, iqn2LldObj.aidConfig.aidEnable);
	if (slotcount == (SLOT_NUM_FIRST_PUSH + 1)) {
		IQN2_enableException(&iqn2LldObj);
	}
	if (slotcount == (SLOT_NUM_DATA_CHECK-1)) {
		UTILS_iqn2ExceptIntDisable();
	}
#else
    if (int4_result == 20) {
        IQN2_enableException(&iqn2LldObj);
    }
    if (int4_result == (SLOT_NUM_DATA_CHECK-1)) {
        UTILS_iqn2ExceptIntDisable();
    }
#endif
    int4_result++;
}

int main(void)
{
    
#ifdef LTE_ON
    uint32_t  chan, radtId;
#endif
	uint32_t  idx;

    UTILS_setCache();
    printf("\nBeginning test\n");
    memset(&iqn2LldObj, 0, sizeof(iqn2LldObj));
    memset(&pktDmaObj, 0, sizeof(pktDmaObj));

    iqn2_mapDevice();
    UTILS_fillIqn2Obj(&testObjTab, &iqn2LldObj);
    if (EXTERNAL_SYNC) {
        iqn2LldObj.timerSyncSource = IQN2_PHY_SYNC;
        UTILS_resetTimer(EXTERNAL_SYNC_TIMER);
        UTILS_initTimer(EXTERNAL_SYNC_TIMER);
    } else {
        iqn2LldObj.timerSyncSource = IQN2_DIAG_SW_SYNC;
    }

    //DFE pll should be set correctly before turning on DFE power domains
    UTILS_iqn2DfeEnable(iqn2LldObj.aidConfig.aidEnable);

#ifdef LTE_ON
    memset(mono_region, 0, sizeof(mono_region));
	atevt2_userIsr           = int4_isr;
#endif
#ifdef WCDMA_ON
    memset(wcdma_dio_result, 0xFF, sizeof(wcdma_dio_result));//clear dest buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_result, sizeof(wcdma_dio_result)); // writeback in MSM
    memset(wcdma_dio_data, 0x00, sizeof(wcdma_dio_data));//clear src buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_data, sizeof(wcdma_dio_data)); // writeback in MSM
	atevt0_userIsr           = int4_isr;
#endif

    // initialization function for the interrupt controllers
    iqn2ev0except_userIsr    = UTILS_getIqn2EV0Exception;
    iqn2pktdmaexcept_userIsr = UTILS_getIqn2PktDMAException;
    UTILS_iqn2IntcSetup();

    IQN2_calcParameters(&iqn2LldObj);
    /**********************************************************************************************************/
    /***************************** PktDma parameters and initialization ***************************************/
    /**********************************************************************************************************/
#ifdef LTE_ON
	chan    = 0;
	// PktDma parameters
	for (idx = 0; idx < IQN2_MAX_NUM_AIL; idx++)
	{
		if (pktDmaObj.firstAxC > iqn2LldObj.ailConfig[idx].firstLteAxC)
			pktDmaObj.firstAxC  = iqn2LldObj.ailConfig[idx].firstLteAxC;
		pktDmaObj.numAxC    += iqn2LldObj.ailConfig[idx].numLtePeAxC;
	}
	for (idx=0 ; idx < pktDmaObj.numAxC ; idx++)
	{
		pktDmaObj.txRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.txNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.rxRegionAxC[chan]   = UTILS_getMemRegionNum(mono_region);
		pktDmaObj.rxNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		if(iqn2LldObj.AxCconfig[iqn2LldObj.ailConfig[0].firstLteAxC + idx].sampleRate == IQN2_SRATE_30P72MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE20;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE20;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE20;
		} else if (iqn2LldObj.AxCconfig[iqn2LldObj.ailConfig[0].firstLteAxC + idx].sampleRate == IQN2_SRATE_15P36MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE10;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE10;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE10;
		} else if (iqn2LldObj.AxCconfig[iqn2LldObj.ailConfig[0].firstLteAxC + idx].sampleRate == IQN2_SRATE_7P68MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE5;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE5;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE5;
		} else if (iqn2LldObj.AxCconfig[iqn2LldObj.ailConfig[0].firstLteAxC + idx].sampleRate == IQN2_SRATE_23P04MHZ)
        {
            pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE15;
            pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE15;
            pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE15;
        }

		memset(&rxFlow[idx], 0, sizeof(Cppi_RxFlowCfg));
		rxFlow[idx].rx_dest_qnum     = MONO_RX_Q+chan;
		rxFlow[idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // MONO
		rxFlow[idx].rx_fdq1_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_fdq2_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_fdq3_qnum     = MONO_RX_FDQ+chan;
		rxFlow[idx].rx_sop_offset    = 12+4;   // desc header size for Monolithic packet with PS

		pktDmaObj.hRxFlowAxC[chan]   = &rxFlow[idx];
		pktDmaObj.hRxFlowCtrl[chan]  = NULL;

		chan++;
	}


	// initialization function for qmss and cppi low-level drivers
#if LTE_RATE == 1
	UTILS_initQmss((uint32_t*)mono_region, NBDESCMAX, ((1024/64)+1) * 64);

#else
	UTILS_initQmss((uint32_t*)mono_region, NBDESCMAX, ((SYMBOLSIZE20/64)+1) * 64, NBDESCMAX);
#endif

	UTILS_initPktDma(&pktDmaObj);
	UTILS_initCppiChannel(&pktDmaObj);
#endif
	/**********************************************************************************************************/


    /**********************************************************************************************************/
    /******************************** Dio engine parameters and initialization ********************************/
    /**********************************************************************************************************/
#ifdef WCDMA_ON
	for (idx=0;idx<iqn2LldObj.ailConfig[0].numWcdmaPeAxC;idx++)
	{
	    iqn2LldObj.dioConfig[0].in[idx] = &(wcdma_dio_result[idx][0]);
	    iqn2LldObj.dioConfig[0].out[idx] = &(wcdma_dio_data[idx][0]);
    }
	iqn2LldObj.dioConfig[0].inNumBlock = DIO_NUM_BLOCK;
	iqn2LldObj.dioConfig[0].outNumBlock = DIO_NUM_BLOCK;
	iqn2LldObj.dioConfig[0].rsaOn = RSA_ON;
	IQN2_initDio(&iqn2LldObj);
#endif
    /**********************************************************************************************************/

//    //SERDES configuration
    serdesCfg0_mapDevice();
    bootCfg_mapDevice();
    UTILS_iqn2SerdesConfig(&iqn2LldObj, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_4p9152G);
    
    IQN2_initHw(&iqn2LldObj,&iqn2InitCfg);

    // all AxC in same radio standard in this test, so get radtId from AxC0, and then use default LLD LTE values for this radio timer
#ifdef LTE_ON
	radtId = IQN2_getEgressRadioTimerId(iqn2LldObj.ailConfig[0].firstLteAxC);
	IQN2_initRadioTimer(&iqn2LldObj);
	// use At Event 2, init here and enable after IQN2_startHw()
	memset(&at2Evt2, 0, sizeof(at2Evt2));
	at2Evt2.EventSelect   = IQN2_AT2_EVENT_2;                    // Evt 2
	at2Evt2.EventOffset   = 3256;                 // Offset from frame boundary in ns
	at2Evt2.EvtStrobeSel  = IQN2_getRadioTimerFrameStrobe(radtId); // Radio timer 1, frame strobe
	at2Evt2.EventModulo   = 500000;               // 0.5ms in ns (lte slot)
	// Convert to byte clocks prior to IQN2_initAt2Event()
	IQN2_initNanoSecsToByteClocks(&iqn2LldObj,&at2Evt2);
	IQN2_initAt2Event(&iqn2LldObj,&at2Evt2);
#else
	IQN2_initRadioTimer(&iqn2LldObj);
	// use At Event 2, init here and enable after IQN2_startHw()
	memset(&at2Evt2, 0, sizeof(at2Evt2));
	at2Evt2.EventSelect   = IQN2_AT2_EVENT_0;                    // Evt 2
	at2Evt2.EventOffset   = 0;                 // Offset from frame boundary in ns
	at2Evt2.EvtStrobeSel  = IQN2FL_RADT0_SYMBOL; // Radio timer 1, frame strobe
	at2Evt2.EventModulo   = -1;               // 0.5ms in ns (lte slot)
	// Convert to byte clocks prior to IQN2_initAt2Event()
	IQN2_initAt2Event(&iqn2LldObj,&at2Evt2);
#endif

#if _IQN2FL_DUMP == 1
    {
       FILE *fout;
       fout = fopen("iqn2fl_dump.txt","w");
       if (fout)
       {
           dump_Iqn2Fl_Setup(fout,iqn2LldObj.hIqn2Setup);
           fclose(fout);
       }

    }
#endif

#ifdef WCDMA_ON
    load_dioData(&iqn2LldObj.dioConfig[0]);
#endif

#ifdef LTE_ON
        load_data(&pktDmaObj);
#endif


	IQN2_startHw(&iqn2LldObj);
    // Start BCN timer free running
    IQN2_runBcnTimer(&iqn2LldObj);
	// Enable sync process if not SW diag sync
    if (EXTERNAL_SYNC) {
        // Enable AT2 exception to catch the external sync
        IQN2_enableAt2Exception(&iqn2LldObj,0);
        // Trigger external sync and proceed with SW sync in exception interrupt
        UTILS_triggerExternalSync(&iqn2LldObj);
        while (ext_sync == 0)
        {
            asm (" NOP 5 ");
            asm (" NOP 5 ");
        }
    }
    // Enable At event 2 now that IQN2 HW is started
#ifdef LTE_ON
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_2);
#else
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_0);
#endif


    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");
        if(int4_result >= (SLOT_NUM_DATA_CHECK + 2))
         {
#ifdef LTE_ON
			UTILS_resetCppi(&pktDmaObj);
#endif
            UTILS_doCleanup(&iqn2LldObj);
            break;
         }
    }

    IQN2_printException(&iqn2LldObj);

#ifdef LTE_ON
    lteFinalCheck(&pktDmaObj);
#endif

#ifdef WCDMA_ON
    wcdmaFinalCheck(&iqn2LldObj.dioConfig[0]);
#endif

    if (testcheck == 1) {
       testcheck = 0;
       printf("All tests have passed \n");
    } else {
       printf("Some tests have failed : testcheck = %d\n", testcheck);
    }

#ifdef LTE_ON
    UTILS_resetQmss(&pktDmaObj);
#endif
    printf("\nEnding test\n");
    return (0);
}


