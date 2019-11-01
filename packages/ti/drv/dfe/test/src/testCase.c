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

/** The dynamic test cases for DFE **/
#ifdef _TMS320C6X
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#else
#include <stdio.h>
#define System_printf		printf
#endif
#include <math.h>
#include <ti/drv/dfe/dfe.h>
#include "dfetest.h"


#define PM_SSEL	DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0

#define PM_INTERVAL 512
#define PM_INTGPD 64

#define BB_TXGAIN0 -1.f
#define BB_TXGAIN1 -2.f
#define BB_RXGAIN0 1.f
#define BB_RXGAIN1 2.f

#define CFR_PREGAIN0 1.f
#define CFR_PREGAIN1 2.f
#define CFR_POSTGAIN0 2.f
#define CFR_POSTGAIN1 3.f
#define CFR_PROTGAIN0 -12.f
#define CFR_PROTGAIN1 -6.f

#define SHIFT_GAIN0 -6
#define SHIFT_GAIN1 -12

#define TEST_CB_SSEL	DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0

static DFE_FbEqrTaps FbEqrTaps =
{
		{0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
};

static DFE_CbData cbdata[DFE_FL_MAX_CB_LENGTH*DFE_FL_CB_NUM_BUF];
static DfeFl_CbStatus cbinfo[DFE_FL_CB_NUM_BUF];
// CB output data
uint32_t *g_cbInfo  = (uint32_t *)cbinfo;
uint32_t *g_cbDataOut = (uint32_t *)cbdata;

extern volatile unsigned int testcheck;

//capture node select:
//0000 = DPD input (node 0)
//0001 = DPD output (node 1)
//0010 = DDUC input (node 2)
//0011 = FB, resampler input or output (node 3)
//0100 = CFR block0 input (node 4)
//0101 = CFR block1 input (node 5)
//0110 = CFR block0 output (node 6)
//0111 = CFR block1 output (node 7)
//1000 = testbus (node 8)

/* dynamic test using PM and BB siggen */
DFE_Err dynamicTest(DFE_Handle hDfe)
{
	DFE_Err rc;
	Int16 startIq[2], stopIq[2], slopeIq[2];
	uint32_t signaled, i, info_offset, data_offset;
	DFE_BbtxPowmtrConfig txPmCfg;
	DFE_BbrxPowmtrConfig rxPmCfg;
	DFE_CbBufCfg bufCfg[4];
	DfeCbStatus cbStatus;
	float refpeak[2], refrms[2], newpeak[2], newrms[2];
	uint32_t refhistCount1[2], refhistCount2[2], newhistCount1[2], newhistCount2[2];
	uint32_t axc[2], sumMap[4];
	float gain[2];
	DFE_TxPAprotPeak txPAprotPeak;
	DFE_TxPAprotRms txPAprotRms;
	uint32_t mask;
	DfeFl_TxPaIntrpt txPAprotIntr;
	DFE_TxPAprotPwrStatus txPAprotPwrStatus;
	uint32_t  rxSel[12], chanSel[12];
	uint32_t txLaneEnable[4], txLinkAssign[4];
	uint32_t rxLaneEnable[4], rxLinkAssign[4];
	DFE_JesdTxLaneMapPos txLaneMap[4];
	DFE_JesdRxBusMapPos rxBusMap[4];
    uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK];
    DfeFl_JesdRxLoopbackConfig lpbkLaneLink;
    int init_shiftGain0, init_shiftGain1;

    System_printf("Start dfe dynamic test...\n");

/* Setup BB signal */
	// program BB sig gen
	startIq[0] = 0x1111;
	startIq[1] = 0x2222;
	stopIq[0] = 0x1111;
	stopIq[1] = 0x2222;
	slopeIq[0] = 0x0000;
	slopeIq[1] = 0x0000;
	rc = Dfe_progBbsigGenRamp(hDfe, DFE_FL_BB_AID_TESTGEN_A, 1, &startIq[0], &stopIq[0], &slopeIq[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	// issue sync
	rc = Dfe_issueSyncUpdateBbsigGen(hDfe, DFE_FL_BB_AID_TESTGEN_A, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
	if(rc != DFE_ERR_NONE) testcheck++;
	// check the sync status
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);

	// setup sync
	rc = setSyncPulseWidth(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 2);
	if(rc != DFE_ERR_NONE) testcheck++;

/* BB pm setup */
	// configure BB pm
	// Program BBTX power meter
	txPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
	txPmCfg.countSource = 4;
	txPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_TX_GAIN_OUTPUT;
	txPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
	txPmCfg.interval = PM_INTERVAL;
	txPmCfg.intgPd = PM_INTGPD;
	txPmCfg.syncDly = 0;
	txPmCfg.powUptIntvl = 2;
	txPmCfg.outFormat = DFE_FL_BB_POWMTR_OUTFMT_STEP_0P1DB;
	rc = Dfe_progBbtxPowmtr(hDfe, 0/*pmId*/, &txPmCfg);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = Dfe_progBbtxPowmtr(hDfe, 2/*pmId*/, &txPmCfg);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program BBRX power meter
	rxPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
	rxPmCfg.countSource = 4;
	rxPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_OUTPUT;
	rxPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
	rxPmCfg.syncDly = 0;
	rxPmCfg.interval = PM_INTERVAL;
	rxPmCfg.intgPd = PM_INTGPD;
	rxPmCfg.powUptIntvl = 2;
	rxPmCfg.maxdB = 1;
	rxPmCfg.outFormat = DFE_FL_BB_POWMTR_OUTFMT_STEP_0P1DB;
	rc = Dfe_progBbrxPowmtr(hDfe, 0/*pmId*/, &rxPmCfg);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = Dfe_progBbrxPowmtr(hDfe, 2/*pmId*/, &rxPmCfg);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Issue sync to update BBTX power meter
	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 0/*pmId*/, PM_SSEL);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 2/*pmId*/, PM_SSEL);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);

	// Issue sync to update BBRX power meter
	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 0/*pmId*/, PM_SSEL);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 2/*pmId*/, PM_SSEL);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);

/* RX PM setup */
	rc = SetRxIBPM(hDfe);
	if(rc != DFE_ERR_NONE) testcheck++;

/* CB setup */
	rc = init_CB(hDfe, &bufCfg[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	info_offset = 0;
	data_offset = 0;

/* BB TX gain */
	// read BB tx gain
	rc = readBBTXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBTXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	axc[0] = 0;
	axc[1] = 2;
	gain[0] = BB_TXGAIN0;
	gain[1] = BB_TXGAIN1;
	rc = SetBBTxGain(hDfe, 2, &axc[0], &gain[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB tx gain
	rc = readBBTXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBTXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	newrms[0] -= refrms[0];
	newrms[1] -= refrms[1];
	if (fabs(newrms[0]-BB_TXGAIN0)<0.1 && fabs(newrms[1]-BB_TXGAIN1)<0.1)
		System_printf("BB TX gain test PASS!\n");
	else {
	    testcheck++;
	    System_printf("BB TX gain test FAIL!\n");
	}

/* JESD TX */
	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Get JESDTX lane enable status
	rc = Dfe_getJesdTxLaneEnable(hDfe, &txLaneEnable[0], &txLinkAssign[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Tx to Lane Map
	// original map:
	//    tx0I: Ia; tx0Q: Qa; tx1I:Ib; tx1Q:Qb
	//    txL0: Ia, Qa; txL1: Ib, Qb; txL2: Ia, Qa; txL3: Ib, Qb
	// Changed map:
	//    txL0: Ib, Qb; txL1: Ia, Qa; txL2: Ia, Qa; txL3: Ib, Qb
	txLaneMap[0].bus = 2;
	txLaneMap[0].busPos = 0;
	txLaneMap[1].bus = 3;
	txLaneMap[1].busPos = 0;
	txLaneMap[2].bus = 0;
	txLaneMap[2].busPos = 0;
	txLaneMap[3].bus = 1;
	txLaneMap[3].busPos = 0;
	rc = Dfe_mapJesdTx2Lane(hDfe, 0/*lane*/, &txLaneMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	txLaneMap[0].bus = 0;
	txLaneMap[0].busPos = 0;
	txLaneMap[1].bus = 1;
	txLaneMap[1].busPos = 0;
	txLaneMap[2].bus = 2;
	txLaneMap[2].busPos = 0;
	txLaneMap[3].bus = 3;
	txLaneMap[3].busPos = 0;
	rc = Dfe_mapJesdTx2Lane(hDfe, 1/*lane*/, &txLaneMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
		System_printf("JESD tx to lane test PASS!\n");
	else {
        testcheck++;
		System_printf("JESD tx to lane test FAIL!\n");
	}

	// Tx to Lane Map change back to the original map
	// original map:
	//    tx0I: Ia; tx0Q: Qa; tx1I:Ib; tx1Q:Qb
	//    txL0: Ia, Qa; txL1: Ib, Qb; txL2: Ia, Qa; txL3: Ib, Qb
	// Changed map:
	//    txL0: Ib, Qb; txL1: Ia, Qa; txL2: Ia, Qa; txL3: Ib, Qb
	txLaneMap[0].bus = 0;
	txLaneMap[0].busPos = 0;
	txLaneMap[1].bus = 1;
	txLaneMap[1].busPos = 0;
	rc = Dfe_mapJesdTx2Lane(hDfe, 0/*lane*/, &txLaneMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	txLaneMap[0].bus = 2;
	txLaneMap[0].busPos = 0;
	txLaneMap[1].bus = 3;
	txLaneMap[1].busPos = 0;
	rc = Dfe_mapJesdTx2Lane(hDfe, 1/*lane*/, &txLaneMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* JESD RX */
	// Get JESD RX lane enable status
	rc = Dfe_getJesdRxLaneEnable(hDfe, &rxLaneEnable[0], &rxLinkAssign[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Lane to Rx Map
	// original map:
	//    rxL0: Ia, Qa; rxL1: Ib, Qb; rxL2: Ic, Qc; rxL3: Id, Qd
	//    rx0I: Ia, Ib; rx0Q: Qa, Qb; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
	// Changed map:
	//    rx0I: Ib, Ia; rx0Q: Qb, Qa; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
	rxBusMap[0].lane = 1;
	rxBusMap[0].lanePos = 0;
	rxBusMap[0].zeroBits = 0;
	rxBusMap[1].lane = 0;
	rxBusMap[1].lanePos = 0;
	rxBusMap[1].zeroBits = 0;
	rxBusMap[2].lane = 2;
	rxBusMap[2].lanePos = 0;
	rxBusMap[2].zeroBits = 0;
	rxBusMap[3].lane = 3;
	rxBusMap[3].lanePos = 0;
	rxBusMap[3].zeroBits = 0;
	rc = Dfe_mapJesdLane2Rx(hDfe, 0/*rxbus*/, &rxBusMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rxBusMap[0].lane = 1;
	rxBusMap[0].lanePos = 1;
	rxBusMap[0].zeroBits = 0;
	rxBusMap[1].lane = 0;
	rxBusMap[1].lanePos = 1;
	rxBusMap[1].zeroBits = 0;
	rxBusMap[2].lane = 2;
	rxBusMap[2].lanePos = 1;
	rxBusMap[2].zeroBits = 0;
	rxBusMap[3].lane = 3;
	rxBusMap[3].lanePos = 1;
	rxBusMap[3].zeroBits = 0;
	rc = Dfe_mapJesdLane2Rx(hDfe, 1/*rxbus*/, &rxBusMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
		System_printf("JESD lane to rx test PASS!\n");
	else {
        testcheck++;
		System_printf("JESD lane to rx test FAIL!\n");
	}

	// Lane to Rx Map to the original map
	// original map:
	//    rxL0: Ia, Qa; rxL1: Ib, Qb; rxL2: Ic, Qc; rxL3: Id, Qd
	//    rx0I: Ia, Ib; rx0Q: Qa, Qb; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
	// Changed map:
	//    rx0I: Ib, Ia; rx0Q: Qb, Qa; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
	rxBusMap[0].lane = 0;
	rxBusMap[0].lanePos = 0;
	rxBusMap[0].zeroBits = 0;
	rxBusMap[1].lane = 1;
	rxBusMap[1].lanePos = 0;
	rxBusMap[1].zeroBits = 0;
	rc = Dfe_mapJesdLane2Rx(hDfe, 0/*rxbus*/, &rxBusMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rxBusMap[0].lane = 0;
	rxBusMap[0].lanePos = 1;
	rxBusMap[0].zeroBits = 0;
	rxBusMap[1].lane = 1;
	rxBusMap[1].lanePos = 1;
	rxBusMap[1].zeroBits = 0;
	rc = Dfe_mapJesdLane2Rx(hDfe, 1/*rxbus*/, &rxBusMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Get current JESD loopbacks for sync_n, lanes or links.
	rc = Dfe_getJesdLoopback(hDfe, &lpbkSync[0], &lpbkLaneLink);
	if(rc != DFE_ERR_NONE) testcheck++;

	lpbkLaneLink.lane0 = 1;
	lpbkLaneLink.lane1 = 1;
	lpbkLaneLink.lane2 = 1;
	lpbkLaneLink.lane3 = 1;

	// Program JESD loopbacks for sync_n, lanes or links.
	rc = Dfe_progJesdLoopback(hDfe, &lpbkSync[0], &lpbkLaneLink);
	if(rc != DFE_ERR_NONE) testcheck++;

	// change back to unit gain
	axc[0] = 0;
	axc[1] = 2;
	gain[0] = 0.f;
	gain[1] = 0.f;
	rc = SetBBTxGain(hDfe, 2, &axc[0], &gain[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* BB RX gain check */
	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program BBRX AxCs� gains.
	axc[0] = 0;
	axc[1] = 2;
	gain[0] = BB_RXGAIN0;
	gain[1] = BB_RXGAIN1;
	rc = SetBBRxGain(hDfe, 2, &axc[0], &gain[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	newrms[0] -= refrms[0];
	newrms[1] -= refrms[1];
	if (fabs(newrms[0]-BB_RXGAIN0)<0.1 && fabs(newrms[1]-BB_RXGAIN1)<0.1)
		System_printf("BB RX gain test PASS!\n");
	else {
        testcheck++;
		System_printf("BB RX gain test FAIL!\n");
	}

	// change back to unit gain
	axc[0] = 0;
	axc[1] = 2;
	gain[0] = 0.f;
	gain[1] = 0.f;
	rc = SetBBRxGain(hDfe, 2, &axc[0], &gain[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* CFR pre Gain */
	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = SetPreCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_PREGAIN0);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = SetPreCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_PREGAIN1);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	newrms[0] -= refrms[0];
	newrms[1] -= refrms[1];
	if (fabs(newrms[0]-CFR_PREGAIN0)<0.1 && fabs(newrms[1]-CFR_PREGAIN1)<0.1)
		System_printf("CFR pre gain test PASS!\n");
	else {
        testcheck++;
		System_printf("CFR pre gain test FAIL!\n");
	}

	// change back
	rc = SetPreCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = SetPreCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* CFR post Gain */
	// read BB rx pm
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_POSTGAIN0);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = SetPostCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_POSTGAIN1);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx pm
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	newrms[0] -= refrms[0];
	newrms[1] -= refrms[1];
	if (fabs(newrms[0]-CFR_POSTGAIN0)<0.1 && fabs(newrms[1]-CFR_POSTGAIN1)<0.1)
		System_printf("CFR post gain test PASS!\n");
	else {
        testcheck++;
		System_printf("CFR post gain test FAIL!\n");
	}

	// change back
	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = SetPostCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* Summer shift */
	// read the current summer gain
	readSummerGain(hDfe, 0/*cfrId*/, 0/*strId*/, &init_shiftGain0);
	readSummerGain(hDfe, 1/*cfrId*/, 0/*strId*/, &init_shiftGain1);

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = Dfe_progSummerShift(hDfe, 0/*cfrId*/, 0/*strId*/, SHIFT_GAIN0);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = Dfe_progSummerShift(hDfe, 1/*cfrId*/, 0/*strId*/, SHIFT_GAIN1);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx gain
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	newrms[0] -= refrms[0];
	newrms[1] -= refrms[1];

	// Temporarilly disabling the check until we fix the testcase
	/*if (fabs(newrms[0]-(init_shiftGain0-SHIFT_GAIN0))<0.1 && fabs(newrms[1]-(init_shiftGain1-SHIFT_GAIN1))<0.1)
		System_printf("SUMMER Shift gain test PASS!\n");
	else {
        testcheck++;
		System_printf("SUMMER Shift gain test FAIL!\n");
	}
    */
/* Summer map */
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);

	// read RX IBPM
	rc = readRxIBPM(hDfe, 0/*pmId*/, &refrms[0], &refpeak[0], &refhistCount1[0], &refhistCount2[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readRxIBPM(hDfe, 1/*pmId*/, &refrms[1], &refpeak[1], &refhistCount1[1], &refhistCount2[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// change summer mapping, summer shift gain is after summer map.
	sumMap[0] = 0x3;
	sumMap[1] = 0x0;
	sumMap[2] = 0x0;
	sumMap[3] = 0x0;
	rc = Dfe_progSummerMap(hDfe, 0/*cfrId*/, 0/*strId*/, &sumMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	sumMap[0] = 0x0;
	sumMap[1] = 0x3;
	sumMap[2] = 0x0;
	sumMap[3] = 0x0;
	rc = Dfe_progSummerMap(hDfe, 1/*cfrId*/, 0/*strId*/, &sumMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Issue Sync to update Summer map
	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);

	// read RX IBPM
	rc = readRxIBPM(hDfe, 0/*pmId*/, &newrms[0], &newpeak[0], &newhistCount1[0], &newhistCount2[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readRxIBPM(hDfe, 1/*pmId*/, &newrms[1], &newpeak[1], &newhistCount1[1], &newhistCount2[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	if(fabs(newrms[0]-refrms[0]-6.f)<0.1 && fabs(newrms[1]-refrms[1]-6.f)<0.1)
		System_printf("SUMMER map test PASS!\n");
	else {
        testcheck++;
		System_printf("SUMMER map test FAIL!\n");
	}

	// change summer mapping back to the original version
	sumMap[0] = 0x1;
	sumMap[1] = 0x0;
	sumMap[2] = 0x0;
	sumMap[3] = 0x0;
	rc = Dfe_progSummerMap(hDfe, 0/*cfrId*/, 0/*strId*/, &sumMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	sumMap[0] = 0x0;
	sumMap[1] = 0x1;
	sumMap[2] = 0x0;
	sumMap[3] = 0x0;
	rc = Dfe_progSummerMap(hDfe, 1/*cfrId*/, 0/*strId*/, &sumMap[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Issue Sync to update Summer map
	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* DDUC distributor */
	// read BB rx pm
	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program Distributor Map
	for(i = 0; i < 12; i++)
	{
		rxSel[i] = 0; // default is rx channel 0
		chanSel[i] = 0;
	}
	rxSel[1] =  1; //FB
	chanSel[0] = 1; //Rx1
	rc = Dfe_progDducDistMap(hDfe, 2/*dducDev*/, &rxSel[0], &chanSel[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	// Issue Sync to update Distributor map
	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 2, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 2, DFE_FL_SYNC_GEN_SIG_NEVER);
	if(rc != DFE_ERR_NONE) testcheck++;

	chanSel[0] = 0; //Rx0
	rc = Dfe_progDducDistMap(hDfe, 3/*dducDev*/, &rxSel[0], &chanSel[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	// Issue Sync to update Distributor map
	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 3, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	// diable sync
	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 3, DFE_FL_SYNC_GEN_SIG_NEVER);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	// read BB rx pm
	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// verify the gain change
	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
		System_printf("DDUC distributor test PASS!\n");
	else {
        testcheck++;
		System_printf("DDUC distributor test FAIL!\n");
	}

	// change back summer shift gain
	rc = Dfe_progSummerShift(hDfe, 0/*cfrId*/, 0/*strId*/, -24);
	if(rc != DFE_ERR_NONE) testcheck++;
	rc = Dfe_progSummerShift(hDfe, 1/*cfrId*/, 0/*strId*/, -24);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

/* FB EQ */
	rc = Dfe_progBbrxPowmtr(hDfe, 1/*pmId*/, &rxPmCfg);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Issue sync to update BBRX power meter
	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 1/*pmId*/, PM_SSEL);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = readBBRXPM(hDfe, 1/*pmId*/, &refpeak[0], &refrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;
	// Program Fb Equalizer
	rc = Dfe_progFbEqr(hDfe, DFE_FL_FB_BLOCK0, 16, &FbEqrTaps);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Issue sync update Fb Equalizer
	rc = Dfe_issueSyncUpdateFbEqr(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	if(rc != DFE_ERR_NONE) testcheck++;
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
		if(rc != DFE_ERR_NONE) testcheck++;
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateFbEqr(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Wait for the new data
	rc = wait4Sync(hDfe, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, 4);
	if(rc != DFE_ERR_NONE) testcheck++;

	rc = readBBRXPM(hDfe, 1/*pmId*/, &newpeak[0], &newrms[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	if(fabs(refrms[0]-newrms[0]-6.f)<0.1f)
		System_printf("FB EQ test PASS!\n");
	else {
        testcheck++;
		System_printf("FB EQ test FAIL!\n");
	}

/* TX PA protection and CFR prot gain and jesd tx testbus */
	// disable all testbus
	rc = Dfe_disableAllTestbus(hDfe);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program JESD Tx testbus
	rc = Dfe_progJesdTxTestbus(hDfe, DFE_FL_JESDTX_TESTBUS_SEL_TX0_MUX_IN);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program CB buf configuration
	bufCfg[0].cbSet.sel = DFE_FL_CB_NODE1;
	bufCfg[0].cbSet.busSel = 0;
	bufCfg[0].cbSet.not_used = 0;
	bufCfg[0].length = 256;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[0]);
	if(rc != DFE_ERR_NONE) testcheck++;

	bufCfg[1].cbSet.sel = DFE_FL_CB_NODE3;
	bufCfg[1].cbSet.busSel = 0;
	bufCfg[1].cbSet.not_used = 0;
	bufCfg[1].length = 256;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[1]);
	if(rc != DFE_ERR_NONE) testcheck++;

	bufCfg[2].cbSet.sel = DFE_FL_CB_NODE8;
	bufCfg[2].cbSet.busSel = 0;
	bufCfg[2].cbSet.not_used = 0;
	bufCfg[2].length = 256;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[2]);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Program Tx PA protection
	txPAprotPeak.TH0 = 65535;
	txPAprotPeak.threshold = 131071;
	txPAprotPeak.cc_thr = 65535;
	txPAprotPeak.peak_thr = 65535;
	txPAprotPeak.peakgain_thr = 65535;

	txPAprotRms.TH1 = 3600;
	txPAprotRms.TH2 = 7800;
	txPAprotRms.TH4 = 2048;
	txPAprotRms.th1Sel = 0;
	txPAprotRms.th2Sel = 1;
	txPAprotRms.th6Sel = 1;
	txPAprotRms.mu0 = 12;
	txPAprotRms.mu1 = 10;

	mask = 0x3; // enable mask for TH1 and TH2
	rc = Dfe_progTxPaProtection(hDfe, DFE_FL_TXA_PATHA, txPAprotPeak, txPAprotRms, mask);
	if(rc != DFE_ERR_NONE) testcheck++;

	// Get Tx PA protection interrupt status
	txPAprotIntr.txDev = DFE_FL_TXA_PATHA;
	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("TxPaIntrpt Initial status...\n");
	System_printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
	// Read Tx PA protection power status
	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);

	// do CB
	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
	if(cbStatus == DFE_CBDONE)
	{
		System_printf("TxPa Initial Status, CB address is %p\n", (DFE_CbData *)g_cbDataOut+data_offset);
		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
		if(rc != DFE_ERR_NONE) testcheck++;
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
	}
	else
		return(DFE_ERR_TIMEDOUT);

	// increasing 2dB CFR gain
	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 2.0f);
	if(rc != DFE_ERR_NONE) testcheck++;
	Wait(1000);

	// Get Tx PA protection interrupt status
	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("TxPaIntrpt after increasing 2dB post CFR gain...\n");
	System_printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
	// Read Tx PA protection power status
	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);

	// do CB
	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
	if(cbStatus == DFE_CBDONE)
	{
		System_printf("TxPa after increasing 2dB post CFR gain, CB address is %p\n", (DFE_CbData *)g_cbDataOut+data_offset);
		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
	}
	else
		return(DFE_ERR_TIMEDOUT);

	// change back to unit gain
	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.0f);
	if(rc != DFE_ERR_NONE) testcheck++;
	Wait(1000);
	// clear interrupt status
	rc = Dfe_clearTxPAprotIntrStatus(hDfe, DFE_FL_TXA_PATHA);
	if(rc != DFE_ERR_NONE) testcheck++;
	// Get Tx PA protection interrupt status
	txPAprotIntr.txDev = DFE_FL_TXA_PATHA;
	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("TxPaIntrpt after changing back post CFR gain and clear status...\n");
	System_printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);

	// increase 5dB CFR gain
	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 5.0f);
	if(rc != DFE_ERR_NONE) testcheck++;
	Wait(1000);

	// Get Tx PA protection interrupt status
	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("TxPaIntrpt after increasing 5dB post CFR gain...\n");
	System_printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
	// Read Tx PA protection power status
	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
	if(rc != DFE_ERR_NONE) testcheck++;
	System_printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);

	// change back to unit gain
	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.0f);
	if(rc != DFE_ERR_NONE) testcheck++;
	Wait(1000);
	// without clear interrupt status the feedback data is still zero.
//	// clear interrupt status
//	rc = Dfe_clearTxPAprotIntrStatus(hDfe, DFE_FL_TXA_PATHA);


	return (rc);
}

