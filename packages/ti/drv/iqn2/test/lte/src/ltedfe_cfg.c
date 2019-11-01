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

#ifdef CFG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/iqn2/device/iqn2_device.h>
#include <ti/drv/dfe/dfe_drv.h>

#include <ti/drv/iqn2/test/utils/dfess.c>

#include <ti/drv/iqn2/test/utils/cslUtils.h>

#include <aidCfg.h>

#define _IQN2FL_DUMP		0

// test control
#define     TEST_NUM_MAX            1

extern int DFESS_ProgPll(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);


#if DFE_BB_LOOPBACK == 1
extern void preloadTgtcfg_local();
extern void iqn2_bb_loopback_config();
#elif DFE_JESD_LOOPBACK == 1
extern void preloadTgtcfg_local();
extern void iqn2_jesd_loopback_config();
#elif DFE_NORMAL_OPERATION == 1
extern void preloadTgtcfg_local();
extern void iqn2_normal_operation_config();
#endif

extern void dfe_init();
extern int  openDfe_local();
extern void startDfe_local();
extern void loadDfe_local();

/* IQN2 Global structures and variables  */
IQN2_ConfigObj	 			iqn2LldObj;
DFE_Obj                     dfeLldObj;
IQN2_At2EventObj 			at2Evt2;
UTILS_dfeSerDesCfg          dfeSerDesCfg;

TestAidObj testObjTab = {
    "CPRI test",                                            //test name
#if DFE_RATE == 20
    IQN2FL_PROTOCOL_DFE_245_76,                             //CPRI or OBSAI
#elif DFE_RATE == 10
    IQN2FL_PROTOCOL_DFE_368_64,                             //CPRI or OBSAI
#endif
    AID_RATE,                                               //link rate.
    NUM_AXCS_WCDMA_AID,                                     //number of WCDMA AxC.
    NUM_AXCS_LTE20_AID,                                     //number of LTE20 AxC.
    NUM_AXCS_LTE10_AID,                                     //number of LTE10 AxC.
    NUM_AXCS_LTE5_AID,                                      //number of LTE5 AxC.
    NUM_AXCS_LTE15_AID,
};

#if _IQN2FL_DUMP == 1
void dump_Iqn2Fl_Setup (FILE *output, Iqn2Fl_Setup *value);
#endif

// Register/Memory Read/Write Functions
void reg_write_32(Uint32 addr,Uint32 write_data)
{
  *(Uint32 *)(addr) = write_data;
}
int32_t main(void)
{
    uint32_t radtId;
    uint32_t idx;
#ifdef _TMS320C6X
    UTILS_setCache();
    /* Make 3 * 16MB cacheable starting from 0xA1000000*/
    CACHE_setMemRegionInfo(161,1,0);// MAR161 - cacheable (always), not prefetchable
    //CACHE_enableCaching(161);       // MAR161 - enable caching
    CACHE_setMemRegionInfo(162,1,0);// MAR162 - cacheable (always), not prefetchable
    //CACHE_enableCaching(162);       // MAR162 - enable caching
    CACHE_setMemRegionInfo(163,1,0);// MAR163 - cacheable (always), not prefetchable
    //CACHE_enableCaching(163);       // MAR163 - enable caching
#endif

    printf("\nBeginning configuration of LTE20MHz 4 AxCs test\n");
    memset(&iqn2LldObj, 0, sizeof(iqn2LldObj));

    serdesCfg0_mapDevice();
    serdesCfg1_mapDevice();
    bootCfg_mapDevice();
    iqn2_mapDevice();
    dfe_mapDevice();

    UTILS_fillIqn2Aid2Obj(&testObjTab, &iqn2LldObj);

#ifdef _TMS320C6X
    if (EXTERNAL_SYNC) {
        iqn2LldObj.timerSyncSource = IQN2_PHY_SYNC;
        UTILS_resetTimer(EXTERNAL_SYNC_TIMER);
        UTILS_initTimer(EXTERNAL_SYNC_TIMER);
    } else {
        iqn2LldObj.timerSyncSource = IQN2_DIAG_SW_SYNC;
    }
#else
    iqn2LldObj.timerSyncSource = IQN2_DIAG_SW_SYNC;
#endif

//    iqn2ev0except_userIsr    = UTILS_getIqn2EV0Exception;
//    iqn2pktdmaexcept_userIsr = UTILS_getIqn2PktDMAException;

#if 1
    DFESS_ProgDfeTestCntr(3); // internal sysref  //Segmentation fault
            // Init SerDes (will be required when not using Jesd loopback)
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

#else
    //SERDES configuration
    UTILS_iqn2SerdesConfig(&iqn2LldObj, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_4p9152G);
#endif

    /****** DFE setup for egress to ingress digital loopback (LTE use case) **************/
    // Use DFE CSL
    openDfe_local();
#if DFE_BB_LOOPBACK == 1
    preloadTgtcfg_local();
    iqn2_bb_loopback_config(); //enable DFE BB loopback for simple AID LTE loopback test
#elif DFE_JESD_LOOPBACK == 1
    preloadTgtcfg_local();
    iqn2_jesd_loopback_config(); //enable DFE JESD loopback
#elif DFE_NORMAL_OPERATION == 1
    preloadTgtcfg_local();
    iqn2_normal_operation_config();
#endif
    /*************************************************************************/

    IQN2_calcParameters(&iqn2LldObj);

    IQN2_initHw(&iqn2LldObj,&iqn2InitCfg);

    // all AxC in same radio standard in this test, so get radtId from AxC0, and then use default LLD LTE values for this radio timer
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
#ifdef _TMS320C6X
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
#endif

    //init basic DFE setup
    //dfe_init();
    startDfe_local();
    // Enable At event 2 now that IQN2 HW is started
    IQN2_enableAt2Event(&iqn2LldObj,IQN2_AT2_EVENT_2);
    /* Wait for first radio frame with valid DL transmission*/
    while (iqn2LldObj.hFl->regs->At2.AT2_BCN.AT2_BCN_FRM_VALUE_LSB_STS !=2) {
        UTILS_waitForHw(1000);
    }

    /* Wait for first lte symbols to be transmitted */
    while ( CSL_FEXT(iqn2LldObj.hFl->regs->At2.AT2_RADT[0].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL) !=14) {
        UTILS_waitForHw(1000);
    }

    IQN2_enableException(&iqn2LldObj);

    /* Wait for most lte symbols to be transmitted in this radio frame */
    while ( CSL_FEXT(iqn2LldObj.hFl->regs->At2.AT2_RADT[0].AT2_RADT_0_STS,IQN_AT2_AT2_RADT_0_STS_RADT_SYMCNT_VAL) !=126) {
        UTILS_waitForHw(1000);
    }

    IQN2_getException(&iqn2LldObj);

    printf("Ending playback configuration RF test \n");
    return (0);
}

#endif

////////////////////////////////
