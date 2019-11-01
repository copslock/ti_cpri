/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#include <stdlib.h>
#include <string.h>
#ifdef _TMS320C6X
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#else
#include <stdio.h>
#endif
#if SECDEV ==1
#include <ti/sk/sk.h>
#endif

#include "dfetest.h"
#include "serdes_cfg.h"
#include "cslUtils.h"

#define SD_LOOPBACK 1

uint32_t dfe_cfg_base = CSL_DFE_CFG_REGS;
UTILS_dfeSerDesCfg           dfeSerDesCfg;

volatile uint32_t testcheck = 1;    // reports pass fail at the end of the test

#if SECDEV==1
/* Secure context for SK */
#pragma DATA_SECTION(scwp,".neardata");
static int scwp, skErrCode;
#endif

void myDfeTest();

#if SECDEV==1
int registerScwpWrap (void)
{
    SK_registerSCWP(&scwp, 1);
    scwp = SK_allocSC(0, 0xffffffff);
    return 0;
}
#endif

#ifndef _TMS320C6X
#define System_printf		printf
#define System_abort		printf
int main(void)
{
	myDfeTest();
	return(0);
}
#else

void main(void)
{
    //create a task
    Task_Params  tskParams;

    Task_Params_init(&tskParams);
    tskParams.stackSize =0x20000;

    Task_create(myDfeTest, &tskParams, NULL);

#if SECDEV==1
    registerScwpWrap();
#endif
    BIOS_start();

}
#endif

void myDfeTest(void)
{
	int idx, rc = PASS;
    DFE_Handle dfeHandle = NULL;
    DFE_Obj dfeObj;
    DFE_Err dfeErr;
    DFE_CppResTbl dfeResTbl;
    DFE_DevInfo devInfo;

#ifdef _TMS320C6X
	UTILS_setCache();
#endif
	dfe_mapDevice();
	bootCfg_mapDevice();
	// dfess setup OR use GEL file to setup PLL and POWER
	if(rc == PASS) rc = DFESS_ProgPll_csl(1-1/*prediv*/, 8-1 /*mult*/, 1-1 /*postdiv*/, 0 /*divmode*/);

	if(rc != PASS)
	{
		System_printf("DFESS is fail!\n");
		testcheck++;
	} else {

        UTILS_iqn2DfeEnable(1/*dfe enable*/, 1/*dfe dpd enable*/);

        System_printf("DFESS is up, and DFE_CLK is running at 245.76Mhz\n");

        System_printf("serdes_cfg(SERDES_HALF_RATE) ...\n");
        serdesCfg0_mapDevice();
        serdesCfg1_mapDevice();

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

        UTILS_dfeSerdesConfig(&dfeSerDesCfg, CSL_SERDES_REF_CLOCK_122p88M, CSL_SERDES_LINK_RATE_9p8304G);
        //serdes_cfg_csl(DFE_REFCLK122P88MHz_2P4576Gbps);
        //serdes_cfg(DFE_REFCLK122P88MHz_2P4576Gbps);



        // open DFE
        System_printf("open DFE ...\n");
        // none resource reserved
        memset(&dfeResTbl, 0, sizeof(dfeResTbl));
        memset(&dfeObj, 0, sizeof(dfeObj));

        for(idx = 0; idx < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; idx++)
        {
            dfeResTbl.discreteTrig[idx] = DFE_FL_CPP_OPEN_ANY;
        }
        dfeHandle = Dfe_open(CSL_DFE, &dfeObj, &dfeResTbl, dfe_cfg_base, &dfeErr);

        // load tgtcfg
        System_printf("load DFE .reg ...\n");
        if(dfeErr != DFE_ERR_NONE)
        {
            testcheck++;
        } else {

            preloadTgtcfg(dfeHandle);
            rc = Dfe_loadTgtCfg(dfeHandle, (DFE_RegPair *)&tgtData[0][0]);
            if(rc != DFE_ERR_NONE) testcheck++;

            // enable/disable BB AID loopback
            rc = Dfe_enableDisableBbaidLoopback(dfeHandle, 0/*disable*/);
            if(rc != DFE_ERR_NONE) testcheck++;

            // start dfe
            if(rc == DFE_ERR_NONE)
            {
                System_printf("start DFE ...\n");
                dfeHandle->sync_cnter_ssel = DFE_FL_SYNC_GEN_SIG_SYSREF;
                rc = startDfe(dfeHandle);
                if(rc != DFE_ERR_NONE) testcheck++;
                rc = Dfe_getDevInfo(dfeHandle, &devInfo);
                if(rc != DFE_ERR_NONE) testcheck++;
                System_printf("PID is %#x\n", devInfo.pid);

            }

            if(rc == DFE_ERR_NONE)
            {
                rc = dynamicTest(dfeHandle);
                if(rc != DFE_ERR_NONE) testcheck++;
            }
        }
	}

    if (testcheck == 1) {
       testcheck = 0;
       System_printf("All tests have passed \n");
    } else {
       System_printf("Some tests have failed : testcheck = %d\n", testcheck);
    }

	// Close DFE LLD device opened by Dfe_open().
	if (dfeHandle) rc = Dfe_close(dfeHandle);

#if SECDEV==1
    skErrCode = SK_freeSC(scwp);
#endif

	System_abort("\nEnding dfe dynamic test\n");

}


////////////

