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

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

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

#include <ti/drv/iqn2/test/utils/cslUtils.h>
#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#include <ti/drv/iqn2/test/utils/lteUtilsMulti.h>
#include <ti/drv/iqn2/test/utils/wcdmaUtils.h>
#include <ti/drv/iqn2/test/utils/dfess.c>

#define RACTAC_TESTxx
#ifdef RACTAC_TEST
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>
#endif

//include for GPIO
#include <ti/csl/csl_gpio.h>
#include <ti/csl/csl_gpioAux.h>


#include "aidCfg.h"

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
#pragma DATA_SECTION(lte_dio_data,".intData_sect")//use MSMC memory for test
#pragma DATA_ALIGN (lte_dio_data, 64)
#endif
uint32_t   lte_dio_data[NUM_AXCS_LTE_AID][4096]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching
#ifdef _TMS320C6X
#pragma DATA_SECTION(lte_dio_result,".intData_sect")//use MSMC memory for test
#pragma DATA_ALIGN (lte_dio_result, 64)
#endif
uint32_t   lte_dio_result[NUM_AXCS_LTE_AID][4096]; // aligned on 64 bytes and multiple of 64 bytes for L1 caching
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

#define NUM_CTL					0
#define NUM_DESC_CTL            0

#if DFE_JESD_LOOPBACK == 1 || DFE_BB_LOOPBACK == 1
#define SD_LOOPBACK 1
#endif

extern int DFESS_ProgPll(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);

#ifdef LTE_ONLY
extern void preloadTgtcfg_local();
extern void iqn2_bb_loopback_config();
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
extern void startDfe_local();
extern void loadDfe_local();
#ifdef DFE_CTL
extern int  testDfeCtlSimple(PktDmaConfigHandle hPktDma);
#endif

/* IQN2 Global structures and variables  */
IQN2_ConfigObj	             iqn2LldObj;
DFE_Obj                      dfeLldObj;
PktDmaConfigObj              pktDmaObj;
IQN2_At2EventObj             at2Evt2;
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
extern volatile uint32_t slotcount;
#endif

volatile unsigned int int4_result = 0;


Swi_Handle slotCountSwi;
Swi_Handle eeWcdmaSwi;


#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif


void incrementSwi();
void exceptionWcdmaSwi();


// Register/Memory Read/Write Functions
void reg_write_32(Uint32 addr,uint32_t write_data)
{
  *(uint32_t *)(addr) = write_data;
}

void int4_isr(){    //interrupt
	Swi_post(slotCountSwi);
	Swi_post(eeWcdmaSwi);
    int4_result++;
}

void myIqn2Test();

extern DfeFl_BbHandle hDfeBb[DFE_FL_BB_PER_CNT];
extern DfeFl_JesdHandle hDfeJesd[DFE_FL_JESD_PER_CNT];
extern DfeFl_MiscHandle hDfeMisc[DFE_FL_MISC_PER_CNT];

void main(void)
{
	//create a task
    Task_Params  tskParams;

    Task_Params_init(&tskParams);
    tskParams.stackSize =0x10000;

    Task_create(myIqn2Test, &tskParams, NULL);

    //create a Swi that will be trigger by the slot Hwi to procces the recycling and pushing of packet at a slot time base.
    Swi_Params swiParams;

	Swi_Params_init (&swiParams);
	swiParams.arg0 = 0;
	swiParams.arg1 = 0;
	swiParams.priority = 1;
	swiParams.trigger = 0;
	eeWcdmaSwi = Swi_create(exceptionWcdmaSwi, &swiParams, NULL);

    BIOS_start();

}


void myIqn2Test()
{
    uint32_t pll_mult, pll_divmode;
#ifdef LTE_ON
    uint32_t  chan, radtId;
#endif

#ifdef RACTAC_TEST
    Iqn2Fl_Dio2ReconfigureEngineSetup   dio2EngineSetup;
#endif
    uint32_t idx;

    UTILS_setCache();
    System_printf("\nBeginning test\n");
    memset(&iqn2LldObj, 0, sizeof(iqn2LldObj));
    memset(&pktDmaObj, 0, sizeof(pktDmaObj));

    iqn2_mapDevice();
    dfe_mapDevice();
    UTILS_fillIqn2Aid2Obj(&testObjTab, &iqn2LldObj);
	iqn2LldObj.aidConfig.lteDio = 1;
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
    System_printf("FATAL: DFE PLL failed!\n");
    System_abort("End of Program due to error!!\n");
    }

    //DFE pll should be set correctly before turning on DFE power domains
    UTILS_iqn2DfeEnable(iqn2LldObj.aidConfig.aidEnable);

    memset(lte_dio_result, 0xFF, sizeof(lte_dio_result));//clear dest buffer
	UTILS_cacheWriteBack((void *)lte_dio_result, sizeof(lte_dio_result)); // writeback in MSM
	memset(lte_dio_data, 0x00, sizeof(lte_dio_data));//clear src buffer
	UTILS_cacheWriteBack((void *)lte_dio_data, sizeof(lte_dio_data)); // writeback in MSM
	atevt2_userIsr           = int4_isr;

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

#ifdef DUAL_MODE
    iqn2LldObj.aidConfig.siDelay = 660;
#endif


    IQN2_calcParameters(&iqn2LldObj);

    /****** DFE setup for egress to ingress digital loopback (LTE use case) **************/
    openDfe_local();
#if LOADDATCONFIG == 1
    loadDfe_local();
#else

#ifdef LTE_ONLY
    preloadTgtcfg_local();
    iqn2_bb_loopback_config();
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
    /***************************** Dio engine parameters and initialization for LTE traffic ***************************************/
    /**********************************************************************************************************/
#ifdef LTE_ON
	for (idx=0;idx<iqn2LldObj.aidConfig.numLteEgressAxC;idx++)
	{
	    iqn2LldObj.dioConfig[0].in[idx] = UTILS_local2GlobalAddr(&(lte_dio_result[idx][0]));
	    iqn2LldObj.dioConfig[0].out[idx] = UTILS_local2GlobalAddr(&(lte_dio_data[idx][0]));
    }
	iqn2LldObj.dioConfig[0].inNumBlock = 64; //3840;
	iqn2LldObj.dioConfig[0].outNumBlock = 64; //3840;
	iqn2LldObj.dioConfig[0].rsaOn = 0;
	iqn2LldObj.dioConfig[0].numPeDBCH = iqn2LldObj.aidConfig.numLteEgressAxC;
	iqn2LldObj.dioConfig[0].numPdDBCH = iqn2LldObj.aidConfig.numLteIngressAxC;
	iqn2LldObj.dioConfig[0].lteMode = 1;
	IQN2_initDio(&iqn2LldObj);
#endif
	/**********************************************************************************************************/


    /**********************************************************************************************************/
    /***************************** Dio engine parameters and initialization for WCDMA traffic ************************************/
    /**********************************************************************************************************/
#ifdef WCDMA_ON
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
	at2Evt2.EventOffset   = 0;                 // Offset from frame boundary in ns
	at2Evt2.EvtStrobeSel  = IQN2_getRadioTimerFrameStrobe(radtId); // Radio timer 1, frame strobe
	at2Evt2.EventModulo   = -1;               // 0.5ms in ns (lte slot)
	// Convert to byte clocks prior to IQN2_initAt2Event()
//	IQN2_initNanoSecsToByteClocks(&iqn2LldObj,&at2Evt2);
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

#ifdef WCDMA_ON
    load_dioData(&iqn2LldObj.dioConfig[0]);
#endif

#ifdef LTE_ON
    load_dioData_lte(&iqn2LldObj.dioConfig[0]);
#endif

//    iqn2LldObj.hIqn2Setup->aid2Setup->uat_evt_radt_cmp_cfg_val[0] = 200;
//    iqn2LldObj.hIqn2Setup->aid2Setup->uat_evt_radt_cmp_cfg_val[8] = 208;

//    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[0] = //700; //si egress (ingress traffic) DIO initiall delay
//    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[19] = //729;  //core ingress (ingress traffic)
//    iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[16] = 0;  //core egress (egress traffic)

    iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_egress.dma_cfg0[0].dma_brst_ln = IQN2FL_4QW;
    iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_egress.dma_cfg0[0].dma_num_qwd = IQN2FL_4QW;
    iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_egress.dma_cfg1_dma_blk_addr_stride[0] = 4;

    iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_ingress.dma_cfg0[0].dma_brst_ln = IQN2FL_4QW;
	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_ingress.dma_cfg0[0].dma_num_qwd = IQN2FL_4QW;
	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_core_ingress.dma_cfg1_dma_blk_addr_stride[0] = 4;

//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].sym_tc = 0xe;//15 WCDMA slots. Set N-1
//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].index_sc = 0;
//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_ife_radio_std_grp.ife_frm_tc_cfg[0].index_tc = 0;
//	iqn2LldObj.hIqn2Setup->dio2Setup->ife_frm_samp_tc_cfg_samp_tc[0] = 2559;//Set N-1
//
//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].sym_tc = 0xe;//15 WCDMA slots. Set N-1
//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].index_sc = 0;
//	iqn2LldObj.hIqn2Setup->dio2Setup->dio2_efe_radio_std_grp.efe_frm_tc_cfg[0].index_tc = 0;
//	iqn2LldObj.hIqn2Setup->dio2Setup->efe_frm_samp_tc_cfg[0] = 2559;//Set N-1

//	//DIO Core Egress RADT0 with event16 (Used for Soc level Egress operation)
//	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[16] = 0;//No initial delay
	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_clk_cnt_tc_cfg_val[16] = 127; //255

	//DIO SI Egress RADT0 with event0 (Used for SoC level Ingress operation)
//	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[0] -= 180;//400(pi)+ 500(ingress pipe delay) = 1000 clock delay for DIO SI
	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_clk_cnt_tc_cfg_val[0] = 31; //255;//4 sample event
//	//DIO Core Ingress RADT0 with event19 (Used for Soc level Ingress operation)
	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_radt_cmp_cfg_val[19] += 90;//DIO SI delay + 500 = 1400 clock delay for final DMA
	iqn2LldObj.hIqn2Setup->dio2Setup->uat_evt_clk_cnt_tc_cfg_val[19] = 127; //255;//4 chip event for DL

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

	IQN2_startHw(&iqn2LldObj);

    // Start BCN timer free running
    IQN2_runBcnTimer(&iqn2LldObj);
    //init basic DFE setup
#if LOADDATCONFIG == 1
//    dfe_init();
#endif
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

    startDfe_local();

#ifdef DFE_CTL
    /* Wait for some radio frames (3) after DFE start prior to DFE CTL tests */
    frameNum = iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS;
    while (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS <= (3+frameNum)) {
        UTILS_waitForHw(1000);
    }

    testDfeCtlSimple(&pktDmaObj);
#endif

    // Enable At event 2 now that IQN2 HW is started
#ifdef LTE_ON
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_2);
#else
        IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_0);
#endif

    //testDfeInterruptEnable();

    /****** wait for data transfer completion.***********************/
    while(1)
    {
    	Task_yield();
        if(int4_result == (SLOT_NUM_DATA_CHECK - 40))
        {
            // testDfeInterruptForce();
        }
        if(int4_result >= (SLOT_NUM_DATA_CHECK + 2))
		{
            UTILS_doCleanup(&iqn2LldObj);
            break;
        }
    }

    IQN2_printException(&iqn2LldObj);

	wcdmaFinalCheck(&iqn2LldObj.dioConfig[0]);

	if (testcheck == 1) {
	   testcheck = 0;
	   System_printf("All tests have passed \n");
	} else {
	   System_printf("Some tests have failed : testcheck = %d\n", testcheck);
	}


#ifdef USERM
    UTILS_resetRm();
#endif
    System_abort("End of Program!!\n");

}


void incrementSwi()
{
	slotCount(iqn2LldObj.hFl);
}
void exceptionWcdmaSwi()
{
	if (int4_result == 20) {
		IQN2_enableException(&iqn2LldObj);
	}
	if (int4_result == (SLOT_NUM_DATA_CHECK-1)) {
		UTILS_iqn2ExceptIntDisable();
	}
}
/////////////////////////////
