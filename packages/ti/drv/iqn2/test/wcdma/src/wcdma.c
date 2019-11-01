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
/* ============================================================================
*  Filename       : wcdma.c
*  Date Created   : Sep. 10, 2013
*  Last Modified  : Sep. 10, 2013
*  Description    : 16x16 WCDMA DIO AxCs AIL loopback test (CPRI 4x or 8x)
*
*  History        : v0.1
*  Project        : Lamarr
*  Owner          : Benjamin Mouchard
*                 : 
\*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_chip.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>
#include <ti/drv/iqn2/test/utils/cslUtils.h>

#define _IQN2FL_DUMP			0

#define EXTERNAL_SYNC           0
#define EXTERNAL_SYNC_TIMER     0   // Used to trigger an external sync, Frame sync with one shot pulse - could be any timer
// test control
#define NUM_AXCS_WCDMA_AIL0     16
#define NUM_AXCS_WCDMA_AIL1     0
#define NUM_AXCS_LTE_AIL0       0
#define NUM_AXCS_LTE_AIL1       0
#ifdef WCDMA_UL
#define DIO_NUM_BLOCK           32 //used large block only for test purpose
#define NUM_CHIP_PER_EVT        8
#define RSA_ON					1
#else
#define DIO_NUM_BLOCK           64 //used large block only for test purpose
#define NUM_CHIP_PER_EVT        4
#define RSA_ON					0
#endif
#define IQN2FL_AIL_MAX			2
#define SAMPLE_FREQ             IQN2_SRATE_3P84MHZ

#define     TEST_NUM            1
#define     TEST_IDX            0

//Users should use 16 bytes aligned(Quad word) data for Aif2 and PktDMA data flow
#ifdef _TMS320C6X
#pragma DATA_SECTION(wcdma_dio_data,".intData_sect")//use SL2 memory
#pragma DATA_ALIGN (wcdma_dio_data, 64)
#endif
uint32_t  wcdma_dio_data[NUM_AXCS_WCDMA_AIL0 * NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];

#ifdef _TMS320C6X
#pragma DATA_SECTION(wcdma_dio_result,".intData_sect")//use SL2 memory
#pragma DATA_ALIGN (wcdma_dio_result, 64)
#endif
uint32_t  wcdma_dio_result[NUM_AXCS_WCDMA_AIL0 * NUM_CHIP_PER_EVT * DIO_NUM_BLOCK];


Iqn2Fl_Status iqn2Status; // IQN2FL status


/* IQN2 Global structures and variables  */
IQN2_ConfigObj 	iqn2LldObj;
IQN2_At2EventObj at2Evt0;

TestObj	testObjTab[TEST_NUM] = {
	{//1st test - OBSAI LTE 20Mhz
#ifdef OBSAI
		"OBSAI_WCDMA", 				//test name
		IQN2FL_PROTOCOL_OBSAI,				//CPRI or OBSAI
#else
		"CPRI_WCDMA", 				//test name
		IQN2FL_PROTOCOL_CPRI,				//CPRI or OBSAI
#endif
        {IQN2FL_LINK_RATE_4x,   IQN2FL_LINK_RATE_4x },          //link rate.
        {NUM_AXCS_WCDMA_AIL0,   NUM_AXCS_WCDMA_AIL1 },          //number of WCDMA AxC. Here it is just LTE application so no WCDMA AxC are required.
        {NUM_AXCS_LTE_AIL0,     NUM_AXCS_LTE_AIL1   },          //number of LTE AxC.
        {240,                   240                 },          //peoffset
        {SAMPLE_FREQ,           SAMPLE_FREQ         },          //sample rate to chose between LTE 20, 10 and 5 MHz.
        {0,                     0                   },          //CPRIPACKMODE, 8b8 for 20MHz, 4b4 for 10 MHz and 2b2 for 5 MHz else you can chose 1b1 for all config
        {IQN2_COM_SD_LOOPBACK,  IQN2_COM_SD_LOOPBACK},
	},
};

volatile unsigned int int4_result = 0;

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif

void int4_isr(){ 	//interrupt

   int4_result++;

   if (int4_result == 30) {
       IQN2_enableException(&iqn2LldObj);
   }
}

void main(void)
{
    uint16_t  testpass;
    int32_t idx, idx2;
    uint32_t radtId;
    UTILS_setCache();
#ifdef OBSAI
    printf("\nBeginning WCDMA OBSAI 16AxCs loopback test\n");
#else
    printf("\nBeginning WCDMA CPRI 16AxCs loopback test\n");	
#endif
    memset(&iqn2LldObj, 0, sizeof(iqn2LldObj));

    UTILS_populateIqn2Obj(testObjTab, &iqn2LldObj);
    iqn2LldObj.ailConfig[0].comType = IQN2_COM_SD_LOOPBACK;

    if (EXTERNAL_SYNC) {
        iqn2LldObj.timerSyncSource = IQN2_PHY_SYNC;
        UTILS_resetTimer(EXTERNAL_SYNC_TIMER);
        UTILS_initTimer(EXTERNAL_SYNC_TIMER);
    } else {
        iqn2LldObj.timerSyncSource = IQN2_DIAG_SW_SYNC;
    }
#ifdef OBSAI
    for (idx=0;idx<iqn2LldObj.ailConfig[0].numWcdmaPeAxC;idx++)
    {
    	iqn2LldObj.AxCconfig[idx].egressAxCOffset = 240;
    	iqn2LldObj.AxCconfig[idx].ingressAxCOffset = 390;
    }
#endif

#if _IQN2FL_DUMP != 1
    //DFE pll should be set correctly before turning on DFE power domains
    UTILS_iqn2DfeEnable(iqn2LldObj.aidConfig.aidEnable);
#endif

    atevt0_userIsr = int4_isr;
	iqn2ev0except_userIsr    = UTILS_getIqn2EV0Exception;

    UTILS_iqn2IntcSetup();


    memset(wcdma_dio_result, 0xFF, sizeof(wcdma_dio_result));//clear dest buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_result, sizeof(wcdma_dio_result)); // writeback in MSM

    memset(wcdma_dio_data, 0x00, sizeof(wcdma_dio_data));//clear src buffer
    UTILS_cacheWriteBack((void *)wcdma_dio_data, sizeof(wcdma_dio_data)); // writeback in MSM

  	for(idx =0; idx < DIO_NUM_BLOCK; idx++){
         for (idx2 = 0; idx2 < NUM_CHIP_PER_EVT*NUM_AXCS_WCDMA_AIL0; idx2 ++) {
  		   wcdma_dio_data[(NUM_CHIP_PER_EVT*NUM_AXCS_WCDMA_AIL0*idx) + idx2] = (idx << 24) + (idx2/4 << 16) + (NUM_AXCS_WCDMA_AIL0*idx) + idx2/4;
         }
  	}
    UTILS_cacheWriteBack((void *)wcdma_dio_data, sizeof(wcdma_dio_data)); // writeback in MSM

#if _IQN2FL_DUMP != 1
//    //SERDES configuration
    serdesCfg0_mapDevice();
    bootCfg_mapDevice();
    if (testObjTab[0].protocol == IQN2FL_PROTOCOL_OBSAI) {
        UTILS_iqn2SerdesConfig(&iqn2LldObj, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_6p144G);
    } else if (testObjTab[0].protocol == IQN2FL_PROTOCOL_CPRI) {
        UTILS_iqn2SerdesConfig(&iqn2LldObj, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_4p9152G);
    }
    iqn2_mapDevice();
#endif

  	iqn2LldObj.dioConfig[0].in = wcdma_dio_data;
  	iqn2LldObj.dioConfig[0].out = wcdma_dio_result;
  	iqn2LldObj.dioConfig[0].inNumBlock = DIO_NUM_BLOCK;
  	iqn2LldObj.dioConfig[0].outNumBlock = DIO_NUM_BLOCK;
  	iqn2LldObj.dioConfig[0].rsaOn = RSA_ON;

	IQN2_calcParameters(&iqn2LldObj);
	IQN2_initDio(&iqn2LldObj);
    IQN2_initHw(&iqn2LldObj,&iqn2InitCfg);
    // all AxC in same radio standard in this test, so get radtId from AxC0, and then use default LLD LTE values for this radio timer
    radtId = IQN2_getEgressRadioTimerId(iqn2LldObj.ailConfig[0].firstWcdmaAxC);
    IQN2_initRadioTimer(&iqn2LldObj);
    // use At Event 2, init here and enable after IQN2_startHw()
    if (radtId == 0) {
        memset(&at2Evt0, 0, sizeof(at2Evt0));
        at2Evt0.EventSelect   = IQN2_AT2_EVENT_0;                       // Evt 0
        at2Evt0.EventOffset   = 0;                                      // Offset from symbol boundary in byte clocks
        at2Evt0.EvtStrobeSel  = IQN2_getRadioTimerSymbolStrobe(radtId); // Radio timer 0, symbol or wcdma slot strobe
        at2Evt0.EventModulo   = -1;                                     // Set to max modulo
        IQN2_initAt2Event(&iqn2LldObj,&at2Evt0);
    }

	iqn2LldObj.hIqn2Setup->ailSetup[0]->pAilUatSetup->ail_uat_pimax_cfg_val = 390;
	iqn2LldObj.hIqn2Setup->ailSetup[0]->pAilUatSetup->ail_uat_pimin_cfg_val = 360;
	iqn2LldObj.hIqn2Setup->ailSetup[0]->pAilUatSetup->ail_uat_tm_fb_cfg_val = 340;
	iqn2LldObj.hIqn2Setup->ailSetup[0]->pAilUatSetup->ail_uat_rt_fb_cfg_val = 320;
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
    // Enable At event 0 now that IQN2 HW is started
    IQN2_enableAt2Event(&iqn2LldObj,0);

    /****** wait for data transfer completion.***********************/
    while(1)
    {
       asm (" NOP 5 ");
       asm (" NOP 5 ");
       if(int4_result == (15*20) + 2)// Wait 17 slot time
       {
           UTILS_iqn2ExceptIntDisable();
          UTILS_doCleanup(&iqn2LldObj);
          break;
       }
    }
	IQN2_printException(&iqn2LldObj);
    testpass = 0;
    /* Compare the WCDMA DIO loopback data */
    UTILS_cacheInvalidate((void *)wcdma_dio_result, sizeof(wcdma_dio_result)); // writeback in MSM
    testpass |= memcmp(&wcdma_dio_data[0], &wcdma_dio_result[0], NUM_AXCS_WCDMA_AIL0*NUM_CHIP_PER_EVT*DIO_NUM_BLOCK);

    if (testpass == 0)
      printf(" \nDIO AxC Data Send/Recv: PASS\n");
    else
      printf(" \nDIO AxC Data Send/Recv: FAIL\n");
#ifdef OBSAI
    printf("\nEnding OBSAI WCDMA 16 AxCs  loopback test\n");
#else
	printf("\nEnding CPRI WCDMA 16 AxCs  loopback test\n");
#endif
    
}

