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
#include <ti/drv/dfe/dfe_drv.h>

#include <ti/drv/dfe/test/utils/dfetest.h>
#include <ti/drv/iqn2/test/utils/cslUtils.h>
#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#include <ti/drv/iqn2/test/utils/lteUtilsMulti.h>
#include <ti/drv/iqn2/test/utils/wcdmaUtils.h>
#include <ti/drv/iqn2/test/utils/dfess.c>

#define RACTAC_TESTxx
#ifdef RACTAC_TEST
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#endif


#include "aidCfg.h"

#define BB_TEST 			1
#define CB_TEST 			0

#define _IQN2FL_DUMP		0
#if LTE_RATE == 40
#define LOADDATCONFIG       1
#elif LTE_RATE == 80
#define LOADDATCONFIG       1
#else
#define LOADDATCONFIG       0
#endif

#if NUM_AXCS_LTE_AID != 0
#define 	LTE_ON
#endif 
#if NUM_AXCS_WCDMA_AID != 0
#define		WCDMA_ON
#endif

#define     TEST_NUM_MAX            1

#ifdef DUAL_MODE
#define SYMBOLSIZE  SYMBOLSIZE5
#else
#if LTE_RATE == 80
#define SYMBOLSIZE  SYMBOLSIZE80
#elif LTE_RATE == 40
#define SYMBOLSIZE  SYMBOLSIZE40
#else
#define SYMBOLSIZE  SYMBOLSIZE20
#endif
#endif

#ifdef LTE_ON
//Users should use 16 bytes aligned data for IQN2 and PktDMA test
#ifdef _TMS320C6X
#pragma DATA_SECTION(mono_region,".iqn2descmsm")//use MSMC memory for test
#pragma DATA_ALIGN (mono_region, 64)
#endif
uint8_t   mono_region[NBDESCMAX * ((SYMBOLSIZE/64)+1) * 64]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching
#endif

#ifdef WCDMA_ON
//Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
#pragma DATA_SECTION(wcdma_dio_data,".intData_sect")//use SL2 memory for WCDMA test
#pragma DATA_ALIGN (wcdma_dio_data, 64)
uint32_t  wcdma_dio_data[NUM_AXCS_WCDMA_AID][NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];
#pragma DATA_SECTION(wcdma_dio_result,".intData_sect")//use SL2 memory for WCDMA test
#pragma DATA_ALIGN (wcdma_dio_result, 64)
uint32_t  wcdma_dio_result[NUM_AXCS_WCDMA_AID][NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];
#endif

#ifdef DFE_CTL
#define NUM_CTL					3                   // Number of control channels
#define NUM_DESC_CTL            256

#ifdef HOSTCTL
#define CTL_SIZE                1024
#pragma DATA_SECTION(mono_region_ctrl,".iqn2ctldesc")//use MSMC memory for test mode
#pragma DATA_ALIGN (mono_region_ctrl, 64)
uint8_t   mono_region_ctrl[NUM_DESC_CTL * 64];//assuming 8 Tx/Rx packets for 3 DFE control channels (8 * 2 * 3 rounded to the next power of 2)
#pragma DATA_SECTION(hostBufferTx,".iqn2buffer")
#pragma DATA_ALIGN (hostBufferTx, 64)
uint32_t	hostBufferTx[NUM_CTL][CTL_SIZE*NUM_DESC_CTL/4];
#pragma DATA_SECTION(hostBufferRx,".iqn2buffer")
#pragma DATA_ALIGN (hostBufferRx, 64)
uint32_t	hostBufferRx[NUM_CTL][CTL_SIZE*NUM_DESC_CTL/4];
#else
//#pragma DATA_SECTION(mono_region_ctrl,".iqn2descmsm")//use MSMC memory for test mode
#pragma DATA_SECTION(mono_region_ctrl,".iqn2descddr")//use MSMC memory for test mode
#pragma DATA_ALIGN (mono_region_ctrl, 64)
#define CTL_SIZE               32784 // (1024 + 12)
uint8_t   mono_region_ctrl[NUM_DESC_CTL * ((CTL_SIZE/64)+1) * 64];//assuming 8 Tx/Rx packets for 3 DFE control channels (8 * 2 * 3 rounded to the next power of 2)
#endif
Cppi_RxFlowCfg rxFlowCtrl[NUM_CTL];
#else
#define NUM_CTL					0
#define NUM_DESC_CTL            0
#endif

#if DFE_JESD_LOOPBACK == 1 || DFE_BB_LOOPBACK == 1
#define SD_LOOPBACK 1
#endif

extern int DFESS_ProgPll(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);


#ifdef LTE_ONLY
extern void preloadTgtcfg_local();
#if DFE_BB_LOOPBACK == 1
extern void iqn2_bb_loopback_config();
#else
extern void iqn2_jesd_loopback_config();
extern void iqn2_dfe_ctl_jesd_loopback_config();
#endif
#endif
#ifdef WCDMA_ONLY
extern void preloadTgtcfg_local();
extern void iqn2_bb_loopback_wcdma_dl_config();
#endif
#ifdef DUAL_MODE
extern void preloadTgtcfg_local();
extern void iqn2_bb_loopback_config();
#endif
extern void dfe_init();
extern int  openDfe_local();
extern DFE_Err startDfe(DFE_Handle hDfe);
extern void loadDfe_local();
#ifdef DFE_CTL
extern int  testDfeCtlSimple(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
extern DFE_Err doTestLLD1(DFE_Handle hDfe);
extern int resultTx;
extern int resultRx;
#endif

/* IQN2 Global structures and variables  */
IQN2_ConfigObj	 iqn2LldObj;
DFE_Obj          dfeLldObj;
PktDmaConfigObj  pktDmaObj;
IQN2_At2EventObj at2Evt2;
UTILS_dfeSerDesCfg           dfeSerDesCfg;

TestAidObj testObjTab = {
    "CPRI test",                                            //test name
#if DFE_RATE == 10
    IQN2FL_PROTOCOL_DFE_368_64,
#else
    IQN2FL_PROTOCOL_DFE_245_76,                             //CPRI or OBSAI
#endif
    AID_RATE,                                               //link rate.
    NUM_AXCS_WCDMA_AID,                                     //number of WCDMA AxC.
    NUM_AXCS_LTE20_AID,                                     //number of LTE20 AxC.
    NUM_AXCS_LTE10_AID,                                     //number of LTE10 AxC.
    NUM_AXCS_LTE5_AID,                                      //number of LTE5 AxC.
    NUM_AXCS_LTE15_AID,                                     //number of LTE15 AxC.
    NUM_AXCS_LTE40_AID,                                     //number of LTE40 AxC.
    NUM_AXCS_LTE80_AID,                                     //number of LTE80 AxC.
};

volatile unsigned int testcheck = 1;    // reports pass fail at the end of the test


/*
 * Define Rx flows for IQN2 ingress traffic
 * The flow will tell IQN2 from which FDQ to pop new symbol descriptors
 */
#ifdef LTE_ON
Cppi_RxFlowCfg rxFlow[NUM_AXCS_LTE_AID];
extern volatile uint32_t slotcount;
#endif

volatile unsigned int int4_result = 0;
volatile unsigned int resultBbPm = 0;
volatile unsigned int cbQueueCount = 0;
volatile unsigned int closeCb = 0;

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif

// Register/Memory Read/Write Functions
void reg_write_32(Uint32 addr,uint32_t write_data)
{
  *(uint32_t *)(addr) = write_data;
}



void int4_isr(){    //interrupt
#ifdef LTE_ON
	slotCount(iqn2LldObj.hFl);
	slotRecycling(&pktDmaObj);
	slotPushing(&pktDmaObj);
	packetCount(iqn2LldObj.hFl, iqn2LldObj.aidConfig.aidEnable);
	if (slotcount == (SLOT_NUM_FIRST_PUSH + 1)) {
		IQN2_enableException(&iqn2LldObj);
		Dfe_enableException(&dfeLldObj);
	}
	if (slotcount == (SLOT_NUM_DATA_CHECK-1)) {
		UTILS_iqn2ExceptIntDisable();
	}
#else
    if (int4_result == 20) {
        IQN2_enableException(&iqn2LldObj);
        //Dfe_enableException(&dfeLldObj);
    }
    if (int4_result == (SLOT_NUM_DATA_CHECK-1)) {
        UTILS_iqn2ExceptIntDisable();
    }
#endif
#ifdef DFE_CTL
#if BB_TEST == 1
    resultTestBbPm(&dfeLldObj, &pktDmaObj, slotcount);
    if(int4_result == (SLOT_NUM_DATA_CHECK))
    {
    	disableCppDma(&dfeLldObj);
    }
#endif
#if CB_TEST == 1
    if(int4_result >= SLOT_NUM_FIRST_PUSH + 60)
    {
    	cbQueueCount += readTestCbDma(&dfeLldObj, &pktDmaObj);
    }
#endif
#endif
    int4_result++;
}

volatile uint32_t tti_cpuclock = 0;
uint32_t tti[100];
uint32_t ttiCnt=0;

void tti_isr(){    //interrupt

    if (ttiCnt==0)
    {
        tti_cpuclock = TSCL;
    } else {
        tti[ttiCnt-1]  = TSCL - tti_cpuclock;
        tti_cpuclock   = TSCL;
    }
    ttiCnt++;
}

void testDfeInterruptEnable()
{
//DfeFl_JesdTxLaneIntrs       jesdTxLaneErr;
DfeFl_BbGeneralIntrGroup    bbErr;
DfeFl_MiscMasterLowPriIntr  masterLowPrioErr;

  /*jesdTxLaneErr.lane = DFE_FL_JESD_LANE_0;
  jesdTxLaneErr.fifoEmptyIntr = 1;
  jesdTxLaneErr.fifoFullIntr = 0;
  jesdTxLaneErr.fifoReadErrIntr = 0;
  jesdTxLaneErr.fifoWriteErrIntr = 0;*/
  /*dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_CLR_LANE_INTRGRP_STATUS, &jesdTxLaneErr);
  dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_ENB_LANE_INTRGRP, &jesdTxLaneErr);
  masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_JESD;*/


  memset(&bbErr,0x00,sizeof(DfeFl_BbGeneralIntrGroup));
  bbErr.txaidOverflow = 1;
  dfeFl_BbHwControl(dfeLldObj.hDfeBb[0], DFE_FL_BB_CMD_CLR_GENERAL_INTRGRP_STATUS, &bbErr);
  dfeFl_BbHwControl(dfeLldObj.hDfeBb[0], DFE_FL_BB_CMD_ENB_GENERAL_INTRGRP, &bbErr);
  masterLowPrioErr =  DFE_FL_MISC_MASTER_LOWPRI_BB;

  dfeFl_MiscHwControl(dfeLldObj.hDfeMisc[0], DFE_FL_MISC_CMD_CLR_MASTER_LOWPRI_INTR_STATUS, &masterLowPrioErr);
  dfeFl_MiscHwControl(dfeLldObj.hDfeMisc[0], DFE_FL_MISC_CMD_ENB_MASTER_LOWPRI_INTR, &masterLowPrioErr);
}

void testDfeInterruptForce() {
//DfeFl_JesdTxLaneIntrs       jesdTxLaneErr;
DfeFl_BbGeneralIntrGroup    bbErr;

  /*jesdTxLaneErr.lane = DFE_FL_JESD_LANE_0;
  jesdTxLaneErr.fifoEmptyIntr = 1;
  jesdTxLaneErr.fifoFullIntr = 0;
  jesdTxLaneErr.fifoReadErrIntr = 0;
  jesdTxLaneErr.fifoWriteErrIntr = 0;*/
  //dfeFl_JesdHwControl(hDfeJesd[0], DFE_FL_JESDTX_CMD_SET_FORCE_LANE_INTRGRP, &jesdTxLaneErr);

  memset(&bbErr,0x00,sizeof(DfeFl_BbGeneralIntrGroup));
  bbErr.txaidOverflow = 1;
  dfeFl_BbHwControl(dfeLldObj.hDfeBb[0], DFE_FL_BB_CMD_SET_FORCE_GENERAL_INTRGRP, &bbErr);
}

int main(void)
{
    uint32_t pll_mult, pll_divmode, radtId;
#ifdef LTE_ON
    uint32_t  chan;
#endif
#ifdef DFE_CTL
#if BB_TEST == 0
    uint32_t frameNum = 0;
#endif
#endif
#ifdef RACTAC_TEST
    Iqn2Fl_Dio2ReconfigureEngineSetup   dio2EngineSetup;
#endif
    uint32_t idx, rc;
    DFE_Err dfeErr;
    DFE_CppResTbl dfeResTbl;
    DFE_Handle    dfeHandle;

    TSCL=0;

    UTILS_setCache();
    printf("\nBeginning test\n");
    memset(&iqn2LldObj, 0, sizeof(iqn2LldObj));
    memset(&pktDmaObj, 0, sizeof(pktDmaObj));

    iqn2_mapDevice();
    dfe_mapDevice();
    UTILS_fillIqn2Aid2Obj(&testObjTab, &iqn2LldObj);

	iqn2LldObj.aidConfig.numCtlChannel 	 = NUM_CTL;
	iqn2LldObj.aidConfig.firstCtlChannel = 32;
	
    if (EXTERNAL_SYNC) {
        iqn2LldObj.timerSyncSource = IQN2_PHY_SYNC;
        UTILS_resetTimer(EXTERNAL_SYNC_TIMER);
        UTILS_initTimer(EXTERNAL_SYNC_TIMER);
    } else {
        iqn2LldObj.timerSyncSource = IQN2_DIAG_SW_SYNC;
    }


#if DFE_RATE == 10
    pll_mult = 6 - 1; // to 737.28
    pll_divmode = 1;  // div by 2 = 368.64
#else
    pll_mult = 8 - 1; // to 737.28
    pll_divmode = 0;  // div by 2 = 368.64
#endif

    if(DFESS_ProgPll_csl(1-1/*prediv*/, pll_mult /*mult*/, 1-1 /*postdiv*/, pll_divmode /*divmode*/) == 0){
    printf("FATAL: DFE PLL failed!\n");
    return -1;
    }

    //DFE pll should be set correctly before turning on DFE power domains
    UTILS_iqn2DfeEnable(iqn2LldObj.aidConfig.aidEnable);

    // Open EDMA channels for Ingress IQ sample capture mechanism
#ifdef LTE_ON
    memset(mono_region, 0, sizeof(mono_region));
	atevt2_userIsr           = int4_isr;
#endif
#ifdef DFE_CTL
	memset(mono_region_ctrl, 0, sizeof(mono_region_ctrl));
#ifdef HOSTCTL
	memset(hostBufferTx, 0, sizeof(hostBufferTx));
	memset(hostBufferRx, 0, sizeof(hostBufferRx));
#endif
#endif

#ifdef WCDMA_ON
    memset(wcdma_dio_result, 0xFF, sizeof(wcdma_dio_result));//clear dest buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_result, sizeof(wcdma_dio_result)); // writeback in MSM
    memset(wcdma_dio_data, 0x00, sizeof(wcdma_dio_data));//clear src buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_data, sizeof(wcdma_dio_data)); // writeback in MSM
	atevt0_userIsr           = int4_isr;
	atevt1_userIsr           = tti_isr;
#endif

    // initialization function for the interrupt controllers
    iqn2ev0except_userIsr    = UTILS_getIqn2EV0Exception;
    dfeev1except_userIsr     = UTILS_getDfeEV1Exception;
    iqn2pktdmaexcept_userIsr = UTILS_getIqn2PktDMAException;
    UTILS_iqn2IntcSetup();

#ifdef DUAL_MODE
    iqn2LldObj.aidConfig.siDelay = 660;
#endif


    IQN2_calcParameters(&iqn2LldObj);

    /****** DFE setup for egress to ingress digital loopback (LTE use case) **************/
    // none resource reserved
    memset(&dfeResTbl, 0, sizeof(dfeResTbl));
    memset(&dfeLldObj, 0, sizeof(dfeLldObj));

    for(idx = 0; idx < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; idx++)
    {
        dfeResTbl.discreteTrig[idx] = DFE_FL_CPP_OPEN_ANY;
    }
    dfeHandle = Dfe_open(CSL_DFE, &dfeLldObj, &dfeResTbl, CSL_DFE_CFG_REGS, &dfeErr);
#if LOADDATCONFIG == 1
    loadDfe_local(dfeHandle);
#else
#ifdef LTE_ONLY
    preloadTgtcfg_local();
#if DFE_JESD_LOOPBACK == 1
#ifdef DFE_CTL
    iqn2_dfe_ctl_jesd_loopback_config();
#else
    iqn2_jesd_loopback_config();
#endif
#else
    iqn2_bb_loopback_config();
#endif
#endif
#ifdef WCDMA_ONLY
    preloadTgtcfg_local();
    iqn2_bb_loopback_wcdma_dl_config();
#endif
#ifdef DUAL_MODE
    preloadTgtcfg_local();// preload all zeros
    iqn2_bb_loopback_config(); //enable DFE BB loopback for dual mode
#endif
#endif
    /*************************************************************************/


    /**********************************************************************************************************/
    /***************************** PktDma parameters and initialization ***************************************/
    /**********************************************************************************************************/
#ifdef LTE_ON
	chan    = 0;
	// PktDma parameters
	pktDmaObj.firstAxC  = 0;
	pktDmaObj.numAxC    = iqn2LldObj.aidConfig.numLteEgressAxC;
	for (idx=0 ; idx < pktDmaObj.numAxC ; idx++)
	{
		pktDmaObj.txRegionAxC[chan]   = Qmss_MemRegion_MEMORY_REGION1; // UTILS_getMemRegionNum(mono_region);
		pktDmaObj.txNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		pktDmaObj.rxRegionAxC[chan]   = Qmss_MemRegion_MEMORY_REGION1; //UTILS_getMemRegionNum(mono_region);
		pktDmaObj.rxNumDescAxC[chan]  = NBSYMBOL*2; // double num of Pkts
		if(iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_30P72MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE20;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE20;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE20;
		} else if (iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_15P36MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE10;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE10;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE10;
		} else if (iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_7P68MHZ)
		{
			pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE5;
			pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE5;
			pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE5;
        } else if (iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_23P04MHZ)
        {
            pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE15;
            pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE15;
            pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE15;
        } else if (iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_61P44MHZ)
        {
            pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE40;
            pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE40;
            pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE40;
        } else if (iqn2LldObj.AxCconfig[iqn2LldObj.aidConfig.firstLteAxC + idx].sampleRate == IQN2_SRATE_122P88MHZ)
        {
            pktDmaObj.txDescSizeAxC[idx] = SYMBOLSIZE80;
            pktDmaObj.txDesc2SizeAxC[idx]= SYMBOL2SIZE80;
            pktDmaObj.rxDescSizeAxC[idx] = SYMBOLSIZE80;
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
	
#ifdef DFE_CTL
	chan = 0;
	pktDmaObj.numCtrl = iqn2LldObj.aidConfig.numCtlChannel;
	pktDmaObj.firstCtrl = iqn2LldObj.aidConfig.firstCtlChannel;
	pktDmaObj.psMsgEnable = 1;
	for (idx=0 ; idx<NUM_CTL ; idx++)
	{
		pktDmaObj.txNumDescCtrl[chan]  = NUMDESCCTL; // double num of Pkts
		pktDmaObj.txDescSizeCtrl[chan] = CTL_SIZE;
		pktDmaObj.rxNumDescCtrl[chan]  = NUMDESCCTL; // double num of Pkts
		pktDmaObj.rxDescSizeCtrl[chan] = CTL_SIZE;

		memset(&rxFlowCtrl[idx], 0, sizeof(Cppi_RxFlowCfg));
		rxFlowCtrl[idx].rx_dest_qnum     = MONO_RX_Q_CTRL+chan;
		rxFlowCtrl[idx].rx_fdq0_sz0_qnum = MONO_RX_FDQ_CTRL+chan;
		rxFlowCtrl[idx].rx_fdq1_qnum     = MONO_RX_FDQ_CTRL+chan;
		rxFlowCtrl[idx].rx_fdq2_qnum     = MONO_RX_FDQ_CTRL+chan;
		rxFlowCtrl[idx].rx_fdq3_qnum     = MONO_RX_FDQ_CTRL+chan;

#ifdef HOSTCTL
		pktDmaObj.txRegionCtrl[chan]   = Qmss_MemRegion_MEMORY_REGION0;
		pktDmaObj.rxRegionCtrl[chan]   = Qmss_MemRegion_MEMORY_REGION0;
		pktDmaObj.txDataBuff[chan] = hostBufferTx[chan];
		pktDmaObj.rxDataBuff[chan] = hostBufferRx[chan];
		pktDmaObj.ctrlDescType 	   =  Cppi_DescType_HOST;
		rxFlowCtrl[idx].rx_desc_type     = (uint8_t)Cppi_DescType_HOST;    // HOST
//		rxFlowCtrl[idx].rx_size_thresh0 = CTL_SIZE;
//		rxFlowCtrl[idx].rx_size_thresh0_en = 0;
#else
		pktDmaObj.txRegionCtrl[chan]   = Qmss_MemRegion_MEMORY_REGION0;
		pktDmaObj.rxRegionCtrl[chan]   = Qmss_MemRegion_MEMORY_REGION0;
		pktDmaObj.ctrlDescType 	   =  Cppi_DescType_MONOLITHIC;
		rxFlowCtrl[idx].rx_desc_type     = (uint8_t)Cppi_DescType_MONOLITHIC;    // HOST
		rxFlowCtrl[idx].rx_sop_offset    = 12;   // desc header size for Monolithic packet without PS
#endif

		pktDmaObj.hRxFlowCtrl[chan]  = &rxFlowCtrl[idx];;
		chan++;
	}
#endif

	// initialization function for qmss and cppi low-level drivers
#if LTE_RATE == 1
	UTILS_initQmss((uint32_t*)mono_region, NBDESCMAX, ((1024/64)+1) * 64);
#else
	UTILS_initQmss((uint32_t*)NULL, NBDESCMAX, ((pktDmaObj.txDescSizeAxC[0]/64)+1) * 64,NBDESCMAX + NUM_DESC_CTL);
#endif

#ifdef DFE_CTL
#ifdef HOSTCTL
	UTILS_insertQmssRegion((uint32_t*) mono_region_ctrl, Qmss_MemRegion_MEMORY_REGION0, NUM_DESC_CTL, 64, 0);
	UTILS_insertQmssRegion((uint32_t*) mono_region, Qmss_MemRegion_MEMORY_REGION1, NBDESCMAX, ((pktDmaObj.txDescSizeAxC[0]/64)+1) * 64, NUM_DESC_CTL);
#else
    UTILS_insertQmssRegion((uint32_t*) mono_region_ctrl, Qmss_MemRegion_MEMORY_REGION0, NUM_DESC_CTL, ((pktDmaObj.txDescSizeCtrl[0]/64)+1) * 64, 0);
    UTILS_insertQmssRegion((uint32_t*) mono_region, Qmss_MemRegion_MEMORY_REGION1, NBDESCMAX, ((pktDmaObj.txDescSizeAxC[0]/64)+1) * 64, NUM_DESC_CTL);
#endif
#else
    UTILS_insertQmssRegion((uint32_t*) mono_region, Qmss_MemRegion_MEMORY_REGION1, NBDESCMAX, ((pktDmaObj.txDescSizeAxC[0]/64)+1) * 64, 0);
#endif


	UTILS_initPktDma(&pktDmaObj);
	UTILS_initCppiChannel(&pktDmaObj);
#endif
	/**********************************************************************************************************/


    /**********************************************************************************************************/
    /******************************** Dio engine parameters and initialization ********************************/
    /**********************************************************************************************************/
#ifdef WCDMA_ON
	iqn2LldObj.dioConfig[0].firstAxC = iqn2LldObj.aidConfig.firstWcdmaAxC;
#ifndef RACTAC_TEST
	for (idx=0;idx<iqn2LldObj.aidConfig.numWcdmaEgressAxC;idx++)
	{
	    iqn2LldObj.dioConfig[0].in[idx] = UTILS_local2GlobalAddr(&(wcdma_dio_result[idx][0]));
	    iqn2LldObj.dioConfig[0].out[idx] = UTILS_local2GlobalAddr(&(wcdma_dio_data[idx][0]));
    }
	iqn2LldObj.dioConfig[0].inNumBlock = DIO_NUM_BLOCK;
	iqn2LldObj.dioConfig[0].outNumBlock = DIO_NUM_BLOCK;
	iqn2LldObj.dioConfig[0].rsaOn = RSA_ON;
#else
	// Initial configuration for RAC and TAC and then override using IQN2FL
	iqn2LldObj.dioConfig[0].usedWithRAC = 1;
	iqn2LldObj.dioConfig[0].rsaOn       = 0;
	iqn2LldObj.dioConfig[0].usedWithTAC = 1;
	iqn2LldObj.dioConfig[0].egRsaOn     = 0;
	iqn2LldObj.dioConfig[0].outNumBlock = 8;
#endif
	IQN2_initDio(&iqn2LldObj);
#endif
    /**********************************************************************************************************/

    //SERDES configuration
    serdesCfg0_mapDevice();
    serdesCfg1_mapDevice();
    bootCfg_mapDevice();
    
    for (idx=0 ; idx<4 ; idx++){
        if (idx<4) {
            dfeSerDesCfg.laneEnable[idx]   = 1;
#if SD_LOOPBACK == 1
            dfeSerDesCfg.lbackEnable[idx]  = 1;
#else
            dfeSerDesCfg.lbackEnable[idx]  = 0;
#endif
        } else {
            dfeSerDesCfg.laneEnable[idx]   = 0;
        }
        dfeSerDesCfg.laneCtrlRate[idx] = CSL_SERDES_LANE_HALF_RATE;
    }

#if DFE_RATE == 20
    UTILS_dfeSerdesConfig(&dfeSerDesCfg, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);
#elif DFE_RATE == 10
    UTILS_dfeSerdesConfig(&dfeSerDesCfg, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_7p3728G);
#endif

    IQN2_initHw(&iqn2LldObj,&iqn2InitCfg);

#ifdef DUAL_MODE
    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[1] = 1460;
    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[16] = 420;
    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[19] = 2660;
#endif

    // all AxC in same radio standard in this test, so get radtId from AxC0, and then use default LLD LTE values for this radio timer
#ifdef LTE_ON
	radtId = IQN2_getEgressRadioTimerId(iqn2LldObj.aidConfig.firstLteAxC);

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
	radtId = IQN2_getIngressRadioTimerId(iqn2LldObj.aidConfig.firstWcdmaAxC);
    // Setup INGRESS radio timer frame strobe to tick on 40ms TTI
	iqn2LldObj.radTimerConfig[radtId].mode = IQN2_RADT_ING_MODE;
	iqn2LldObj.radTimerConfig[radtId].userSpecified = 1;
	iqn2LldObj.radTimerConfig[radtId].frameTerminalCount = 4096;
	iqn2LldObj.radTimerConfig[radtId].initFrameLsbNum = 1;
	iqn2LldObj.radTimerConfig[radtId].initFrameMsbNum = 0;
	iqn2LldObj.radTimerConfig[radtId].wcdma.numSlotStrobesForFrameStrobe = 4*15;

	IQN2_initRadioTimer(&iqn2LldObj);

	// use At Event 2, init here and enable after IQN2_startHw()
	memset(&at2Evt2, 0, sizeof(at2Evt2));
	at2Evt2.EventSelect   = IQN2_AT2_EVENT_0;      // Evt 0
	at2Evt2.EventOffset   = 0;                     // Offset from frame boundary in ns
	at2Evt2.EvtStrobeSel  = IQN2FL_RADT0_SYMBOL;   // Radio timer 0, slot strobe
	at2Evt2.EventModulo   = -1;                    // wcdma slot
	// Convert to byte clocks prior to IQN2_initAt2Event()
	IQN2_initAt2Event(&iqn2LldObj,&at2Evt2);

	// use At Event 1, 40ms TTI tick
    memset(&at2Evt2, 0, sizeof(at2Evt2));
    at2Evt2.EventSelect   = IQN2_AT2_EVENT_1;      // Evt 1
    at2Evt2.EventOffset   = 0;                     // Offset from frame boundary in ns
    at2Evt2.EvtStrobeSel  = IQN2FL_RADT1_FRAME;   // Radio timer 1, frame strobe
    at2Evt2.EventModulo   = -1;                    // wcdma frame
    at2Evt2.EventMaskLsb  = 0xFFFFFFF8;
    at2Evt2.EventMaskMsb  = 0xFFFFFFFF;
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

#ifdef RACTAC_TEST
    memset(&dio2EngineSetup, 0x00, sizeof(Iqn2Fl_Dio2ReconfigureEngineSetup));
    dio2EngineSetup.engine_idx = 0;
    for (idx=0;idx<iqn2LldObj.aidConfig.numWcdmaEgressAxC;idx++)
    {
        dio2EngineSetup.egress_axc_buffer_start_addr[idx] = ((uint32_t)UTILS_local2GlobalAddr(&(wcdma_dio_data[idx][0])))>>4;
        memset(&(wcdma_dio_data[idx][0]), 0xAA, sizeof(uint32_t) * NUM_CHIP_PER_EVT * DIO_NUM_BLOCK);
    }
    for (idx=0;idx<iqn2LldObj.aidConfig.numWcdmaIngressAxC;idx++)
    {
        dio2EngineSetup.ingress_axc_buffer_start_addr[idx] = ((uint32_t)UTILS_local2GlobalAddr(&(wcdma_dio_result[idx][0])))>>4;
    }
    dio2EngineSetup.egress_num_block          = DIO_NUM_BLOCK - 1;
    dio2EngineSetup.ingress_num_block         = DIO_NUM_BLOCK - 1;
    dio2EngineSetup.egress_block_addr_stride  = IQN2FL_1QW + 1;
    dio2EngineSetup.ingress_block_addr_stride = IQN2FL_1QW + 1;

    Iqn2Fl_setupDio2ReconfigureEngine(iqn2LldObj.hFl, &dio2EngineSetup);
#endif

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

    dfeHandle->sync_cnter_ssel = DFE_FL_SYNC_GEN_SIG_SYSREF;
    rc = startDfe(dfeHandle);
    if(rc != DFE_ERR_NONE) testcheck++;

#ifdef DFE_CTL
#if BB_TEST == 0
#if CB_TEST == 1
   	dfeErr = doTestCbDma(&dfeLldObj, &pktDmaObj);
#else
    /* Wait for some radio frames (3) after DFE start prior to DFE CTL tests */
    frameNum = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
    while (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS <= (3+frameNum)) {
        UTILS_waitForHw(1000);
    }
    testDfeCtlSimple(&dfeLldObj ,&pktDmaObj);
#endif
#else
	dfeErr = doTestBbPm(&dfeLldObj, &pktDmaObj);
#endif
#endif

    // Enable At event 2 now that IQN2 HW is started
#ifdef LTE_ON
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_2);
#else
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_0);
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_1);
#endif

    //testDfeInterruptEnable();

    /****** wait for data transfer completion.***********************/
    while(1)
    {
        asm (" NOP 5 ");
        asm (" NOP 5 ");
        if(int4_result >= (SLOT_NUM_DATA_CHECK + 2))
		{
            UTILS_doCleanup(&iqn2LldObj);
            break;
        }
    }

    IQN2_printException(&iqn2LldObj);
    Dfe_printException(&dfeLldObj);

#ifdef LTE_ON
        lteFinalCheck(&pktDmaObj);
#endif
#ifdef WCDMA_ON
        wcdmaFinalCheck(&iqn2LldObj.dioConfig[0]);
#endif

#ifdef DFE_CTL
#if BB_TEST == 1
	if (resultTx >= 1)
		printf("Tx power meter measurement packet received = %d \n", resultTx);
	if (resultRx >= 1)
		printf("Rx power meter measurement packet received = %d \n", resultRx);
#endif
#if CB_TEST == 1
	resultTestCbDma(&dfeLldObj, &pktDmaObj);
	if (cbQueueCount != 0)
	{
		printf("CB capture Buffer received: %d \n", cbQueueCount);
	}
#endif
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
#ifdef USERM
    UTILS_resetRm();
#endif
    printf("\nEnding test\n");
    return (0);
}


