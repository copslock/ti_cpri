/****************************************************************************\
 *           (C) Copyright 2013, Texas Instruments, Inc.                    *
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

#include <stdio.h>
#include <math.h>
#include <constants.h>

#include <ti/drv/dfe/test/utils/dfetest.h>
#include <ti/drv/dfe/test/src/testCmd.c>
//#include "LLDtest.h"
//#include "CFRcoeff.h"

//#ifdef USE_IQN
#define PM_SSEL	DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0 //DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0
//#else
//	#define PM_SSEL	DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0
//#endif
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

//#ifdef USE_IQN
//	#define TEST_CB_SSEL	DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0
//#else
//	#define TEST_CB_SSEL	DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0
//#endif

//extern DFE_DpdData *pDpdData;
DFE_DpdData *pDpdData = (DFE_DpdData *) 0x81510000;

DFE_FbEqrTaps FbEqrTaps =
{
		{0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
};

DFE_RxEqrTaps RxEqrTaps =
{
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
		{0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}
};

float BBTXgain0[8] =
{
		-1.0f, 0.f, -2.f, 0.f, 1.f, 0.f, 2.f, 0.f
};

float BBTXgain1[8] =
{
		1.0f, 0.f, 2.f, 0.f, -1.f, 0.f, -2.f, 0.f
};

// CB output data
Uint32 *g_cbInfo  = (Uint32 *)0x81600000u;
Uint32 *g_cbDataOut = (Uint32 *)0x81610000u;
// current Tx power meter results
float g_BBTXPM[16*2];
// current Rx power meter results
float g_BBRXPM[16*2];

volatile int resultTx = 0;
volatile int resultRx = 0;

volatile int enableCB = 0;
volatile int disableCB = 0;


DFE_CbBufCfg bufCfg[4];

/* DMA test for BB and CB using IQN */
DFE_Err doTestBbPm(DFE_Handle hDfe, PktDmaConfigHandle hPktDma)
{
	DFE_Err rc;
	DFE_BbtxPowmtrConfig txPmCfg;
	DFE_BbrxPowmtrConfig rxPmCfg;

    memset(g_BBTXPM,0x00,sizeof(g_BBTXPM));
    memset(g_BBRXPM,0x00,sizeof(g_BBRXPM));

/* BB pm setup */
	// configure BB pm
	// Program BBTX power meter   					- can do LLD API for that purpose
	txPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
	txPmCfg.countSource = 4;  //CType
	txPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_TX_GAIN_OUTPUT;
	txPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
	txPmCfg.interval = 0;//PM_INTERVAL; //512
	txPmCfg.intgPd = 30720;//PM_INTGPD; Set for LTE20 sampling rate
	txPmCfg.syncDly = 512;//512;
	txPmCfg.powUptIntvl = 0;
	rc = Dfe_progBbtxPowmtr(hDfe, 0/*pmId*/, &txPmCfg);
	if(rc != DFE_ERR_NONE)
		printf("error\n");
	rc = Dfe_progBbtxPowmtr(hDfe, 2/*pmId*/, &txPmCfg);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	// Program BBRX power meter                       - can do LLD API for that purpose
	rxPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
	rxPmCfg.countSource = 4;  //CType
	rxPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_OUTPUT;
	rxPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
	rxPmCfg.syncDly = 512;//512;//256;
	rxPmCfg.interval = PM_INTERVAL;
	rxPmCfg.intgPd =  30720;//PM_INTGPD; Set for LTE20 sampling rate
	rxPmCfg.powUptIntvl = 0;
	rxPmCfg.maxdB = 1;
	rc = Dfe_progBbrxPowmtr(hDfe, 0/*pmId*/, &rxPmCfg);
	if(rc != DFE_ERR_NONE)
		printf("error\n");
	rc = Dfe_progBbrxPowmtr(hDfe, 2/*pmId*/, &rxPmCfg);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

    updateBbPm(hDfe);

/* BB pwmtr DMA */
	// Open CPP/DMA for BBRX power meters
	rc = Dfe_openBbrxPowmtrDma(hDfe, 1/*1 cppDmaId*/, 1/*1 cppDescripId*/, BB_RX_CTL_CHL);
	if(rc != DFE_ERR_NONE)
		printf("error\n");
	// Open CPP/DMA for BBTX power meters
	rc = Dfe_openBbtxPowmtrDma(hDfe, 2/*cppDmaId*/, 2/*cppDescripId*/, BB_TX_CTL_CHL);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	// Enable CPP/DMA for BBRX power meters
	rc = Dfe_enableBbrxPowmtrDma(hDfe, 2/*2 pmId*/);
	if(rc != DFE_ERR_NONE)
		printf("error\n");
	// Enable CPP/DMA for BBTX power meters
	rc = Dfe_enableBbtxPowmtrDma(hDfe, 2/*pmId*/);
	if(rc != DFE_ERR_NONE)
		printf("error\n");
	Wait(100);

	return(rc);
}

DFE_Err updateBbPm(DFE_Handle hDfe)
{
	DFE_Err rc = DFE_ERR_NONE;

#if 1
    DfeFl_BbPowerMeterSsel BbPmSsel;

	// set ssel for RX
	BbPmSsel.pmId = 0;
	BbPmSsel.ssel = PM_SSEL;
	dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_RXPM_SSEL, &BbPmSsel);

    BbPmSsel.pmId = 2;
    BbPmSsel.ssel = PM_SSEL;
    dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_RXPM_SSEL, &BbPmSsel);

    // set ssel for TX
    BbPmSsel.pmId = 0;
    BbPmSsel.ssel = PM_SSEL;
    dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_TXPM_SSEL, &BbPmSsel);

    BbPmSsel.pmId = 2;
    BbPmSsel.ssel = PM_SSEL;
    dfeFl_BbHwControl(hDfe->hDfeBb[0], DFE_FL_BB_CMD_SET_TXPM_SSEL, &BbPmSsel);

#else

	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 0/*pmId*/, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0);//DFE_FL_SYNC_GEN_SIG_NEVER
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 2/*pmId*/, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0);//DFE_FL_SYNC_GEN_SIG_NEVER
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 0/*pmId*/, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0); //DFE_FL_SYNC_GEN_SIG_NEVER
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 2/*pmId*/, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0); //DFE_FL_SYNC_GEN_SIG_NEVER
	if(rc != DFE_ERR_NONE)
		printf("error\n");
#endif
	return(rc);
}

volatile uint32_t txPmTimestamp[400];
volatile uint32_t rxPmTimestamp[400];

volatile uint32_t txPmAxC0NonZero[400];
volatile uint32_t rxPmAxC0NonZero[400];

void resultTestBbPm(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, uint32_t slotcount)
{
	// read BB pm
	resultTx += readBBTXPM_DMA(hDfe, hPktDma, (float *)g_BBTXPM, 1);
	resultRx += readBBRXPM_DMA(hDfe, hPktDma, (float *)g_BBRXPM, 1);
    // Count the number of measurement vs slotcount
    txPmTimestamp[slotcount] = resultTx;
    rxPmTimestamp[slotcount] = resultRx;
    // Timestamp valid measurements
    if ((resultTx != 0) && (txPmTimestamp[slotcount-1] != txPmTimestamp[slotcount])) {
        if ((g_BBTXPM[0] != 0) && (g_BBTXPM[1] != 0)) {
            txPmAxC0NonZero[slotcount] = 1;
        } else txPmAxC0NonZero[slotcount] = 0;
    }
    if ((resultRx != 0) && (rxPmTimestamp[slotcount-1] != rxPmTimestamp[slotcount])) {
        if ((g_BBRXPM[0] != 0) && (g_BBRXPM[1] != 0)) {
            rxPmAxC0NonZero[slotcount] = 1;
        } else rxPmAxC0NonZero[slotcount] = 0;
    }

	return;
}

DFE_Err disableCppDma(DFE_Handle hDfe)
{
	DFE_Err rc;
	// disable CPP/DMA for BBTX power meters
	rc = Dfe_disableBbtxPowmtrDma(hDfe);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	// disable CPP/DMA for BBRX power meters
	rc = Dfe_disableBbrxPowmtrDma(hDfe);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	return (rc);
}

/* DMA test for BB and CB using IQN */
DFE_Err doTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma)
{
	DFE_Err rc;
	Uint32 i;
    Uint32 cbCppDescripId[8];

/* CB setup */
	rc = init_CB(hDfe, &bufCfg[0]);

/* CB DMA */
	// Open CB buf DMA for reading CB buf
	for(i = 0; i < 8; i++)
		cbCppDescripId[i] = i+3;

	rc = Dfe_openCbBufDma(hDfe, 1/*flag_18bit*/, 10/*cppDamId*/, &cbCppDescripId[0], CB_CTL_CHL);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	/* Test CB DMA */
	// Issue CB
	// Program CB buf configuration
	bufCfg[0].cbSet.sel = DFE_FL_CB_NODE0;
	bufCfg[0].cbSet.busSel = 0;
	bufCfg[0].cbSet.not_used = 0;
	bufCfg[0].length = 2048*4;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[0]);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	bufCfg[1].cbSet.sel = DFE_FL_CB_NODE1;
	bufCfg[1].cbSet.busSel = 0;
	bufCfg[1].cbSet.not_used = 0;
	bufCfg[1].length = 2048*4;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[1]);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	bufCfg[2].cbSet.sel = DFE_FL_CB_NODE3;
	bufCfg[2].cbSet.busSel = 0;
	bufCfg[2].cbSet.not_used = 0;
	bufCfg[2].length = 2048*4;
	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[2]);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	return(rc);
}

int readTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma)
{
	uint32_t status = 0, trigMonDecWord;
	DfeCbStatus cbStatus;
	DfeFl_MiscSyncGenSig ssel = DFE_FL_SYNC_GEN_SIG_MPU_SYNC;

	if (ssel == DFE_FL_SYNC_GEN_SIG_MPU_SYNC)
	{
        if(enableCB != 1)
        {
            // do CB of dpdin
            cbStatus = doStaticCB(hDfe, ssel, 100);

            // read CB
            if(cbStatus != DFE_CBDONE)
            {
                printf("CB not started properly\n");
            }
            enableCB_DMA(hDfe, &bufCfg[0], (DfeFl_CbStatus *)g_cbInfo);
            enableCB = 1;
        }

    //	readCB_DMA(hDfe, hPktDma, &bufCfg[0], hDfe->flag_18bit, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
        if (disableCB == 0)
            status = readCB_DMA(hDfe, hPktDma);

    //	rc = readCB(hDfe,&bufCfg[0] , 1, (DfeFl_CbStatus *)g_cbInfo, (DFE_CbData *)g_cbDataOut);
        // Disable CB buf DMA
        if ((status != 0) && (enableCB == 1))
        {
            Dfe_disableCbBufDma(hDfe);
            disableCB = 1;
        }
	}

    if (ssel == DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0)
    {
        //if(enableCB != 1)
        {
            trigMonDecWord = 0xFFFF;
            status = dfeFl_CbHwControl( hDfe->hDfeCb[0],
                                        DFE_FL_CB_CMD_SET_TRIG_DECODER,
                                        &trigMonDecWord );

            Dfe_armCbIssueSync(hDfe, ssel);
            //enableCB = 1;
        }
    }

	return (status);
}

void resultTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma)
{
	uint32_t info_offset, data_offset;

	info_offset = 0;
	data_offset = 0;

	info_offset += 4;
	data_offset += 4*DFE_FL_MAX_CB_LENGTH;
	resultCB_DMA(hDfe, hPktDma, &bufCfg[0], hDfe->flag_18bit, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);

}

DFE_Err closeTestCbDma(DFE_Handle hDfe)
{
	DFE_Err rc;

	// Close CB buf DMA
	rc = Dfe_closeCbBufDma(hDfe);
	if(rc != DFE_ERR_NONE)
		printf("error\n");

	return(rc);
}

/* dynamic test 1 using PM and BB siggen */
//DFE_Err doTestLLD1(DFE_Handle hDfe)
//{
//	DFE_Err rc;
//	Int16 startIq[2], stopIq[2], slopeIq[2];
//	Uint32 signaled, i, info_offset, data_offset;
//	DFE_BbtxPowmtrConfig txPmCfg;
//	DFE_BbrxPowmtrConfig rxPmCfg;
//	DFE_CbBufCfg bufCfg[4];
//	DfeCbStatus cbStatus;
//	float refpeak[2], refrms[2], newpeak[2], newrms[2];
//	Uint32 refhistCount1[2], refhistCount2[2], newhistCount1[2], newhistCount2[2];
//	Uint32 axc[2], sumMap[4];
//	float gain[2];
//	DFE_TxPAprotPeak txPAprotPeak;
//	DFE_TxPAprotRms txPAprotRms;
//	Uint32 mask;
//	DfeFl_TxPaIntrpt txPAprotIntr;
//	DFE_TxPAprotPwrStatus txPAprotPwrStatus;
//	Uint32  rxSel[12], chanSel[12];
//	Uint32 txLaneEnable[4], rxLaneEnable[4], txLinkAssign[4], rxLinkAssign[4];
//	DFE_JesdTxLaneMapPos txLaneMap[4];
//	DFE_JesdRxBusMapPos rxBusMap[4];
//	DFE_JesdTxLinkStatus txLinkStatus;
//	DFE_JesdTxLaneStatus txLaneStatus;
//	DFE_JesdRxLinkStatus rxLinkStatus;
//	DFE_JesdRxLaneStatus rxLaneStatus;
//    Uint32 lpbkSync[DFE_FL_JESD_NUM_LINK];
//    DfeFl_JesdRxLoopbackConfig lpbkLaneLink;
//    int init_shiftGain0, init_shiftGain1;
//    DfeFl_FbIOCtrl FbIOMux;
//
//    printf("Start test1...\n");
//
///* Setup BB signal */
//	// program BB sig gen
//	startIq[0] = 0x1111;
//	startIq[1] = 0x2222;
//	stopIq[0] = 0x1111;
//	stopIq[1] = 0x2222;
//	slopeIq[0] = 0x0000;
//	slopeIq[1] = 0x0000;
//	rc = Dfe_progBbsigGenRamp(hDfe, DFE_FL_BB_AID_TESTGEN_A, 1, &startIq[0], &stopIq[0], &slopeIq[0]);
//	// issue sync
//	rc = Dfe_issueSyncUpdateBbsigGen(hDfe, DFE_FL_BB_AID_TESTGEN_A, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/);
//	// check the sync status
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, &signaled);
//	} while(signaled == 0);
//
//	// setup sync
//	rc = setSyncPulseWidth(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 2);
//
///* BB pm setup */
//	// configure BB pm
//	// Program BBTX power meter
//	txPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
//	txPmCfg.countSource = 4;
//	txPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_TX_GAIN_OUTPUT;
//	txPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
//	txPmCfg.interval = PM_INTERVAL;
//	txPmCfg.intgPd = PM_INTGPD;
//	txPmCfg.syncDly = 0;
//	txPmCfg.powUptIntvl = 2;
//	rc = Dfe_progBbtxPowmtr(hDfe, 0/*pmId*/, &txPmCfg);
//	rc = Dfe_progBbtxPowmtr(hDfe, 2/*pmId*/, &txPmCfg);
//
//	// Program BBRX power meter
//	rxPmCfg.enable = DFE_FL_BB_POWMETR_SINGLE_POWER_MEASUREMENT;
//	rxPmCfg.countSource = 4;
//	rxPmCfg.inSource = DFE_FL_BB_POWMTR_INSRC_OUTPUT;
//	rxPmCfg.tddMode = DFE_FL_BB_POWMTR_TDDMODE_DISABLED;
//	rxPmCfg.syncDly = 0;
//	rxPmCfg.interval = PM_INTERVAL;
//	rxPmCfg.intgPd = PM_INTGPD;
//	rxPmCfg.powUptIntvl = 2;
//	rxPmCfg.maxdB = 1;
//	rc = Dfe_progBbrxPowmtr(hDfe, 0/*pmId*/, &rxPmCfg);
//	rc = Dfe_progBbrxPowmtr(hDfe, 2/*pmId*/, &rxPmCfg);
//
//	// Issue sync to update BBTX power meter
//	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 0/*pmId*/, PM_SSEL);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
//	} while(signaled == 0);
//	rc = Dfe_issueSyncUpdateBbtxPowmtr(hDfe, 2/*pmId*/, PM_SSEL);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
//	} while(signaled == 0);
//
//	// Issue sync to update BBRX power meter
//	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 0/*pmId*/, PM_SSEL);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
//	} while(signaled == 0);
//	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 2/*pmId*/, PM_SSEL);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
//	} while(signaled == 0);
//
///* RX PM setup */
//	rc = SetRxIBPM(hDfe);
//
///* CB setup */
//	rc = init_CB(hDfe, &bufCfg[0]);
//	info_offset = 0;
//	data_offset = 0;
//
//
//	// read BB tx gain
//	rc = readBBTXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBTXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	axc[0] = 0;
//	axc[1] = 2;
//	gain[0] = BB_TXGAIN0;
//	gain[1] = BB_TXGAIN1;
//	rc = SetBBTxGain(hDfe, 2, &axc[0], &gain[0]);
//
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB tx gain
//	rc = readBBTXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBTXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	newrms[0] -= refrms[0];
//	newrms[1] -= refrms[1];
//	if (fabs(newrms[0]-BB_TXGAIN0)<0.1 && fabs(newrms[1]-BB_TXGAIN1)<0.1)
//		printf("BB TX gain test PASS!\n");
//	else
//		printf("BB TX gain test FAIL!\n");
//
///* JESD TX */
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	// Get JESDTX lane enable status
////	rc = Dfe_getJesdTxLaneEnable(hDfe, &txLaneEnable[0]);
//	rc = Dfe_getJesdTxLaneEnable(hDfe, &txLaneEnable[0], &txLinkAssign[0]);
//
//	// Tx to Lane Map
//	// original map:
//	//    tx0I: Ia; tx0Q: Qa; tx1I:Ib; tx1Q:Qb
//	//    txL0: Ia, Qa; txL1: Ib, Qb; txL2: Ia, Qa; txL3: Ib, Qb
//	// Changed map:
//	//    txL0: Ib, Qb; txL1: Ia, Qa; txL2: Ia, Qa; txL3: Ib, Qb
//	txLaneMap[0].bus = 2;
//	txLaneMap[0].busPos = 0;
//	txLaneMap[1].bus = 3;
//	txLaneMap[1].busPos = 0;
//	txLaneMap[2].bus = 0;
//	txLaneMap[2].busPos = 0;
//	txLaneMap[3].bus = 1;
//	txLaneMap[3].busPos = 0;
//	rc = Dfe_mapJesdTx2Lane(hDfe, 0/*lane*/, &txLaneMap[0]);
//
//	txLaneMap[0].bus = 0;
//	txLaneMap[0].busPos = 0;
//	txLaneMap[1].bus = 1;
//	txLaneMap[1].busPos = 0;
//	txLaneMap[2].bus = 2;
//	txLaneMap[2].busPos = 0;
//	txLaneMap[3].bus = 3;
//	txLaneMap[3].busPos = 0;
//	rc = Dfe_mapJesdTx2Lane(hDfe, 1/*lane*/, &txLaneMap[0]);
//
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
//		printf("JESD tx to lane test PASS!\n");
//	else
//		printf("JESD tx to lane test FAIL!\n");
//
//	// Tx link status
//	rc = Dfe_getJesdTxLinkStatus(hDfe, &txLinkStatus);
//	printf("JESD tx link status:\n");
//	printf("     firstSyncRequest[0] = %d, firstSyncRequest[1] = %d, \n", txLinkStatus.firstSyncRequest[0], txLinkStatus.firstSyncRequest[1]);
//	printf("     syncErrCount[0] = %d, syncErrCount[1] = %d,\n", txLinkStatus.syncErrCount[0], txLinkStatus.syncErrCount[1]);
//	printf("     sysrefAlignCount = %d, \n", txLinkStatus.sysrefAlignCount);
//	printf("     sysrefErr[0] = %d, sysrefErr[1] = %d,\n", txLinkStatus.sysrefErr[0], txLinkStatus.sysrefErr[1]);
//	printf("     sysrefReqAssert = %d, sysrefReqDeassert = %d,\n", txLinkStatus.sysrefReqAssert, txLinkStatus.sysrefReqDeassert);
//
//	// clear Tx link errors
//	rc = Dfe_clearJesdTxLinkErrors(hDfe);
//
//	// Tx link status
//	rc = Dfe_getJesdTxLinkStatus(hDfe, &txLinkStatus);
//	printf("JESD tx link status after clearing errors:\n");
//	printf("     firstSyncRequest[0] = %d, firstSyncRequest[1] = %d, \n", txLinkStatus.firstSyncRequest[0], txLinkStatus.firstSyncRequest[1]);
//	printf("     syncErrCount[0] = %d, syncErrCount[1] = %d,\n", txLinkStatus.syncErrCount[0], txLinkStatus.syncErrCount[1]);
//	printf("     sysrefAlignCount = %d, \n", txLinkStatus.sysrefAlignCount);
//	printf("     sysrefErr[0] = %d, sysrefErr[1] = %d,\n", txLinkStatus.sysrefErr[0], txLinkStatus.sysrefErr[1]);
//	printf("     sysrefReqAssert = %d, sysrefReqDeassert = %d,\n", txLinkStatus.sysrefReqAssert, txLinkStatus.sysrefReqDeassert);
//
//	// Tx Lane status
//	rc = Dfe_getJesdTxLaneStatus(hDfe, &txLaneStatus);
//	printf("JESD tx lane status:\n");
//	printf("     fifoEmpty[0] = %d, fifoEmpty[1] = %d, fifoEmpty[2] = %d, fifoEmpty[3] = %d, \n", txLaneStatus.fifoEmpty[0], txLaneStatus.fifoEmpty[1],txLaneStatus.fifoEmpty[2], txLaneStatus.fifoEmpty[3]);
//	printf("     fifoFull[0] = %d, fifoFull[1] = %d, fifoFull[2] = %d, fifoFull[3] = %d, \n", txLaneStatus.fifoFull[0], txLaneStatus.fifoFull[1],txLaneStatus.fifoFull[2], txLaneStatus.fifoFull[3]);
//	printf("     fifoReadErr[0] = %d, fifoReadErr[1] = %d, fifoReadErr[2] = %d, fifoReadErr[3] = %d, \n", txLaneStatus.fifoReadErr[0], txLaneStatus.fifoReadErr[1],txLaneStatus.fifoReadErr[2], txLaneStatus.fifoReadErr[3]);
//	printf("     fifoWriteErr[0] = %d, fifoWriteErr[1] = %d, fifoWriteErr[2] = %d, fifoWriteErr[3] = %d, \n", txLaneStatus.fifoWriteErr[0], txLaneStatus.fifoWriteErr[1],txLaneStatus.fifoWriteErr[2], txLaneStatus.fifoWriteErr[3]);
//	printf("     syncState[0] = %d, syncState[1] = %d, syncState[2] = %d, syncState[3] = %d, \n", txLaneStatus.syncState[0], txLaneStatus.syncState[1],txLaneStatus.syncState[2], txLaneStatus.syncState[3]);
//
//	// clear Tx lane errors
//	rc = Dfe_clearJesdTxLaneErrors(hDfe);
//
//	// Tx Lane status
//	rc = Dfe_getJesdTxLaneStatus(hDfe, &txLaneStatus);
//	printf("JESD tx lane status after clearing errors:\n");
//	printf("     fifoEmpty[0] = %d, fifoEmpty[1] = %d, fifoEmpty[2] = %d, fifoEmpty[3] = %d, \n", txLaneStatus.fifoEmpty[0], txLaneStatus.fifoEmpty[1],txLaneStatus.fifoEmpty[2], txLaneStatus.fifoEmpty[3]);
//	printf("     fifoFull[0] = %d, fifoFull[1] = %d, fifoFull[2] = %d, fifoFull[3] = %d, \n", txLaneStatus.fifoFull[0], txLaneStatus.fifoFull[1],txLaneStatus.fifoFull[2], txLaneStatus.fifoFull[3]);
//	printf("     fifoReadErr[0] = %d, fifoReadErr[1] = %d, fifoReadErr[2] = %d, fifoReadErr[3] = %d, \n", txLaneStatus.fifoReadErr[0], txLaneStatus.fifoReadErr[1],txLaneStatus.fifoReadErr[2], txLaneStatus.fifoReadErr[3]);
//	printf("     fifoWriteErr[0] = %d, fifoWriteErr[1] = %d, fifoWriteErr[2] = %d, fifoWriteErr[3] = %d, \n", txLaneStatus.fifoWriteErr[0], txLaneStatus.fifoWriteErr[1],txLaneStatus.fifoWriteErr[2], txLaneStatus.fifoWriteErr[3]);
//	printf("     syncState[0] = %d, syncState[1] = %d, syncState[2] = %d, syncState[3] = %d, \n", txLaneStatus.syncState[0], txLaneStatus.syncState[1],txLaneStatus.syncState[2], txLaneStatus.syncState[3]);
//
//	// Tx to Lane Map change back to the original map
//	// original map:
//	//    tx0I: Ia; tx0Q: Qa; tx1I:Ib; tx1Q:Qb
//	//    txL0: Ia, Qa; txL1: Ib, Qb; txL2: Ia, Qa; txL3: Ib, Qb
//	// Changed map:
//	//    txL0: Ib, Qb; txL1: Ia, Qa; txL2: Ia, Qa; txL3: Ib, Qb
//	txLaneMap[0].bus = 0;
//	txLaneMap[0].busPos = 0;
//	txLaneMap[1].bus = 1;
//	txLaneMap[1].busPos = 0;
//	rc = Dfe_mapJesdTx2Lane(hDfe, 0/*lane*/, &txLaneMap[0]);
//
//	txLaneMap[0].bus = 2;
//	txLaneMap[0].busPos = 0;
//	txLaneMap[1].bus = 3;
//	txLaneMap[1].busPos = 0;
//	rc = Dfe_mapJesdTx2Lane(hDfe, 1/*lane*/, &txLaneMap[0]);
//
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* JESD RX */
//	// Get JESD RX lane enable status
////	rc = Dfe_getJesdRxLaneEnable(hDfe, &rxLaneEnable[0]);
//	rc = Dfe_getJesdRxLaneEnable(hDfe, &rxLaneEnable[0], &rxLinkAssign[0]);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	// Lane to Rx Map
//	// original map:
//	//    rxL0: Ia, Qa; rxL1: Ib, Qb; rxL2: Ic, Qc; rxL3: Id, Qd
//	//    rx0I: Ia, Ib; rx0Q: Qa, Qb; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
//	// Changed map:
//	//    rx0I: Ib, Ia; rx0Q: Qb, Qa; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
//	rxBusMap[0].lane = 1;
//	rxBusMap[0].lanePos = 0;
//	rxBusMap[0].zeroBits = 0;
//	rxBusMap[1].lane = 0;
//	rxBusMap[1].lanePos = 0;
//	rxBusMap[1].zeroBits = 0;
//	rxBusMap[2].lane = 2;
//	rxBusMap[2].lanePos = 0;
//	rxBusMap[2].zeroBits = 0;
//	rxBusMap[3].lane = 3;
//	rxBusMap[3].lanePos = 0;
//	rxBusMap[3].zeroBits = 0;
//	rc = Dfe_mapJesdLane2Rx(hDfe, 0/*rxbus*/, &rxBusMap[0]);
//	rxBusMap[0].lane = 1;
//	rxBusMap[0].lanePos = 1;
//	rxBusMap[0].zeroBits = 0;
//	rxBusMap[1].lane = 0;
//	rxBusMap[1].lanePos = 1;
//	rxBusMap[1].zeroBits = 0;
//	rxBusMap[2].lane = 2;
//	rxBusMap[2].lanePos = 1;
//	rxBusMap[2].zeroBits = 0;
//	rxBusMap[3].lane = 3;
//	rxBusMap[3].lanePos = 1;
//	rxBusMap[3].zeroBits = 0;
//	rc = Dfe_mapJesdLane2Rx(hDfe, 1/*rxbus*/, &rxBusMap[0]);
//
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
//		printf("JESD lane to rx test PASS!\n");
//	else
//		printf("JESD lane to rx test FAIL!\n");
//
//	// Rx link status
//	rc = Dfe_getJesdRxLinkStatus(hDfe, &rxLinkStatus);
//	printf("JESD rx link status:\n");
//	printf("     sysrefAlignCount = %d, \n", rxLinkStatus.sysrefAlignCount);
//	printf("     sysrefErr[0] = %d, sysrefErr[1] = %d, \n", rxLinkStatus.sysrefErr[0], rxLinkStatus.sysrefErr[1]);
//	printf("     sysrefReqAssert = %d, sysrefReqDeassert = %d,\n", rxLinkStatus.sysrefReqAssert, rxLinkStatus.sysrefReqDeassert);
//
//	// clear Rx line errors
//	rc = Dfe_clearJesdRxLinkErrors(hDfe);
//
//	// Rx link status
//	rc = Dfe_getJesdRxLinkStatus(hDfe, &rxLinkStatus);
//	printf("JESD rx link status after clearing errors:\n");
//	printf("     sysrefAlignCount = %d, \n", rxLinkStatus.sysrefAlignCount);
//	printf("     sysrefErr[0] = %d, sysrefErr[1] = %d, \n", rxLinkStatus.sysrefErr[0], rxLinkStatus.sysrefErr[1]);
//	printf("     sysrefReqAssert = %d, sysrefReqDeassert = %d,\n", rxLinkStatus.sysrefReqAssert, rxLinkStatus.sysrefReqDeassert);
//
//	// Rx lane status
//	rc = Dfe_getJesdRxLaneStatus(hDfe, &rxLaneStatus);
//	printf("JESD rx lane status:\n");
//	printf("     bufMatchErr[0] = %d, bufMatchErr[1] = %d, bufMatchErr[2] = %d, bufMatchErr[3] = %d, \n", rxLaneStatus.bufMatchErr[0], rxLaneStatus.bufMatchErr[1], rxLaneStatus.bufMatchErr[2], rxLaneStatus.bufMatchErr[3]);
//	printf("     bufOverflowErr[0] = %d, bufOverflowErr[1] = %d, bufOverflowErr[2] = %d, bufOverflowErr[3] = %d,\n", rxLaneStatus.bufOverflowErr[0], rxLaneStatus.bufOverflowErr[1], rxLaneStatus.bufOverflowErr[2], rxLaneStatus.bufOverflowErr[3]);
//	printf("     codeState[0] = %d, codeState[1] = %d, codeState[2] = %d, codeState[3] = %d,\n", rxLaneStatus.codeState[0], rxLaneStatus.codeState[1], rxLaneStatus.codeState[2], rxLaneStatus.codeState[3]);
//
//	// clear Rx lane errors
//	rc = Dfe_clearJesdRxLaneErrors(hDfe);
//
//	// Rx lane status
//	rc = Dfe_getJesdRxLaneStatus(hDfe, &rxLaneStatus);
//	printf("JESD rx lane status after clearing errors:\n");
//	printf("     bufMatchErr[0] = %d, bufMatchErr[1] = %d, bufMatchErr[2] = %d, bufMatchErr[3] = %d, \n", rxLaneStatus.bufMatchErr[0], rxLaneStatus.bufMatchErr[1], rxLaneStatus.bufMatchErr[2], rxLaneStatus.bufMatchErr[3]);
//	printf("     bufOverflowErr[0] = %d, bufOverflowErr[1] = %d, bufOverflowErr[2] = %d, bufOverflowErr[3] = %d,\n", rxLaneStatus.bufOverflowErr[0], rxLaneStatus.bufOverflowErr[1], rxLaneStatus.bufOverflowErr[2], rxLaneStatus.bufOverflowErr[3]);
//	printf("     codeState[0] = %d, codeState[1] = %d, codeState[2] = %d, codeState[3] = %d,\n", rxLaneStatus.codeState[0], rxLaneStatus.codeState[1], rxLaneStatus.codeState[2], rxLaneStatus.codeState[3]);
//
//	// Lane to Rx Map to the original map
//	// original map:
//	//    rxL0: Ia, Qa; rxL1: Ib, Qb; rxL2: Ic, Qc; rxL3: Id, Qd
//	//    rx0I: Ia, Ib; rx0Q: Qa, Qb; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
//	// Changed map:
//	//    rx0I: Ib, Ia; rx0Q: Qb, Qa; fb0I:Ic; fb0Q:Qc; fb1I: Id; fb1Q: Qd
//	rxBusMap[0].lane = 0;
//	rxBusMap[0].lanePos = 0;
//	rxBusMap[0].zeroBits = 0;
//	rxBusMap[1].lane = 1;
//	rxBusMap[1].lanePos = 0;
//	rxBusMap[1].zeroBits = 0;
//	rc = Dfe_mapJesdLane2Rx(hDfe, 0/*rxbus*/, &rxBusMap[0]);
//	rxBusMap[0].lane = 0;
//	rxBusMap[0].lanePos = 1;
//	rxBusMap[0].zeroBits = 0;
//	rxBusMap[1].lane = 1;
//	rxBusMap[1].lanePos = 1;
//	rxBusMap[1].zeroBits = 0;
//	rc = Dfe_mapJesdLane2Rx(hDfe, 1/*rxbus*/, &rxBusMap[0]);
//
//	// Get current JESD loopbacks for sync_n, lanes or links.
//	rc = Dfe_getJesdLoopback(hDfe, &lpbkSync[0], &lpbkLaneLink);
//
//	lpbkLaneLink.lane0 = 1;
//	lpbkLaneLink.lane1 = 1;
//	lpbkLaneLink.lane2 = 1;
//	lpbkLaneLink.lane3 = 1;
//
//	// Program JESD loopbacks for sync_n, lanes or links.
//	rc = Dfe_progJesdLoopback(hDfe, &lpbkSync[0], &lpbkLaneLink);
//
//	// change back to unit gain
//	axc[0] = 0;
//	axc[1] = 2;
//	gain[0] = 0.f;
//	gain[1] = 0.f;
//	rc = SetBBTxGain(hDfe, 2, &axc[0], &gain[0]);
//
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* BB RX gain check */
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	// Program BBRX AxCs’ gains.
//	axc[0] = 0;
//	axc[1] = 2;
//	gain[0] = BB_RXGAIN0;
//	gain[1] = BB_RXGAIN1;
//	rc = SetBBRxGain(hDfe, 2, &axc[0], &gain[0]);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	newrms[0] -= refrms[0];
//	newrms[1] -= refrms[1];
//	if (fabs(newrms[0]-BB_RXGAIN0)<0.1 && fabs(newrms[1]-BB_RXGAIN1)<0.1)
//		printf("BB RX gain test PASS!\n");
//	else
//		printf("BB RX gain test FAIL!\n");
//
//	// change back to uinit gain
//	axc[0] = 0;
//	axc[1] = 2;
//	gain[0] = 0.f;
//	gain[1] = 0.f;
//	rc = SetBBRxGain(hDfe, 2, &axc[0], &gain[0]);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* CFR pre Gain */
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	rc = SetPreCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_PREGAIN0);
//	rc = SetPreCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_PREGAIN1);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	newrms[0] -= refrms[0];
//	newrms[1] -= refrms[1];
//	if (fabs(newrms[0]-CFR_PREGAIN0)<0.1 && fabs(newrms[1]-CFR_PREGAIN1)<0.1)
//		printf("CFR pre gain test PASS!\n");
//	else
//		printf("CFR pre gain test FAIL!\n");
//
//	// change back
//	rc = SetPreCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
//	rc = SetPreCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* CFR post Gain */
//	// read BB rx pm
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_POSTGAIN0);
//	rc = SetPostCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, CFR_POSTGAIN1);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx pm
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	newrms[0] -= refrms[0];
//	newrms[1] -= refrms[1];
//	if (fabs(newrms[0]-CFR_POSTGAIN0)<0.1 && fabs(newrms[1]-CFR_POSTGAIN1)<0.1)
//		printf("CFR post gain test PASS!\n");
//	else
//		printf("CFR post gain test FAIL!\n");
//
//	// change back
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
//	rc = SetPostCFRGain(hDfe, 1/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.f);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* Summer shift */
//	// read the current summer gain
//	readSummerGain(hDfe, 0/*cfrId*/, 0/*strId*/, &init_shiftGain0);
//	readSummerGain(hDfe, 1/*cfrId*/, 0/*strId*/, &init_shiftGain1);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	rc = Dfe_progSummerShift(hDfe, 0/*cfrId*/, 0/*strId*/, SHIFT_GAIN0);
//	rc = Dfe_progSummerShift(hDfe, 1/*cfrId*/, 0/*strId*/, SHIFT_GAIN1);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx gain
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	newrms[0] -= refrms[0];
//	newrms[1] -= refrms[1];
//	if (fabs(newrms[0]-(init_shiftGain0-SHIFT_GAIN0))<0.1 && fabs(newrms[1]-(init_shiftGain1-SHIFT_GAIN1))<0.1)
//		printf("SUMMER Shift gain test PASS!\n");
//	else
//		printf("SUMMER Shift gain test FAIL!\n");
//
///* FB IO Mux */
//	// Program CB buf configuration
//	bufCfg[0].cbSet.sel = DFE_FL_CB_NODE3;
//	bufCfg[0].cbSet.busSel = 0;
//	bufCfg[0].cbSet.not_used = 0;
//	bufCfg[0].length = 512;
//	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[0]);
//
//	bufCfg[1].cbSet.sel = DFE_FL_CB_NODE8;
//	bufCfg[1].cbSet.busSel = 0;
//	bufCfg[1].cbSet.not_used = 1;
//	bufCfg[1].length = 512;
//	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[1]);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("FB IO Mux, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// get Fb IO Mux
//	dfeFl_FbGetHwStatus(hDfe->hDfeFb[0], DFE_FL_FB_QUERY_GET_IO_CONTROL, &FbIOMux);
//
//	// Program Fb IO Mux
//	rc = Dfe_progFbIOMux(hDfe, DFE_FL_FB_CFG_SEL_JESDFB1_ROUTE2);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("FB IO Mux after switching, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// Change Fb IO Mux back
//	rc = Dfe_progFbIOMux(hDfe, DFE_FL_FB_CFG_SEL_JESDFB0_ROUTE0);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* Summer map */
////	// read BB rx gain
////	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
////	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	// read RX IBPM
//	rc = readRxIBPM(hDfe, 0/*pmId*/, &refrms[0], &refpeak[0], &refhistCount1[0], &refhistCount2[0]);
//	rc = readRxIBPM(hDfe, 1/*pmId*/, &refrms[1], &refpeak[1], &refhistCount1[1], &refhistCount2[1]);
//
//	// change summer mapping, summer shift gain is after summer map.
//	sumMap[0] = 0x3;
//	sumMap[1] = 0x0;
//	sumMap[2] = 0x0;
//	sumMap[3] = 0x0;
//	rc = Dfe_progSummerMap(hDfe, 0/*cfrId*/, 0/*strId*/, &sumMap[0]);
//
//	sumMap[0] = 0x0;
//	sumMap[1] = 0x3;
//	sumMap[2] = 0x0;
//	sumMap[3] = 0x0;
//	rc = Dfe_progSummerMap(hDfe, 1/*cfrId*/, 0/*strId*/, &sumMap[0]);
//
//	// Issue Sync to update Summer map
//	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
//	} while(signaled == 0);
//	// disable sync
//	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
////	// read BB rx gain
////	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
////	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// read RX IBPM
//	rc = readRxIBPM(hDfe, 0/*pmId*/, &newrms[0], &newpeak[0], &newhistCount1[0], &newhistCount2[0]);
//	rc = readRxIBPM(hDfe, 1/*pmId*/, &newrms[1], &newpeak[1], &newhistCount1[1], &newhistCount2[1]);
//
//	if(fabs(newrms[0]-refrms[0]-6.f)<0.1 && fabs(newrms[1]-refrms[1]-6.f)<0.1)
//		printf("SUMMER map test PASS!\n");
//	else
//		printf("SUMMER map test FAIL!\n");
//
//	// change summer mapping back to the original version
//	sumMap[0] = 0x1;
//	sumMap[1] = 0x0;
//	sumMap[2] = 0x0;
//	sumMap[3] = 0x0;
//	rc = Dfe_progSummerMap(hDfe, 0/*cfrId*/, 0/*strId*/, &sumMap[0]);
//
//	sumMap[0] = 0x0;
//	sumMap[1] = 0x1;
//	sumMap[2] = 0x0;
//	sumMap[3] = 0x0;
//	rc = Dfe_progSummerMap(hDfe, 1/*cfrId*/, 0/*strId*/, &sumMap[0]);
//
//	// Issue Sync to update Summer map
//	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
//	} while(signaled == 0);
//	// disable sync
//	rc = Dfe_issueSyncUpdateSummerMap(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* DDUC distributor */
//	// read BB rx pm
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &refpeak[0], &refrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &refpeak[1], &refrms[1]);
//
//	// Program Distributor Map
//	for(i = 0; i < 12; i++)
//	{
//		rxSel[i] = 0; // default is rx channel 0
//		chanSel[i] = 0;
//	}
//	rxSel[1] =  1; //FB
//	chanSel[0] = 1; //Rx1
//	rc = Dfe_progDducDistMap(hDfe, 2/*dducDev*/, &rxSel[0], &chanSel[0]);
//	// Issue Sync to update Distributor map
//	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 2, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
//	} while(signaled == 0);
//	// disable sync
//	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 2, DFE_FL_SYNC_GEN_SIG_NEVER);
//
//	chanSel[0] = 0; //Rx0
//	rc = Dfe_progDducDistMap(hDfe, 3/*dducDev*/, &rxSel[0], &chanSel[0]);
//	// Issue Sync to update Distributor map
//	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 3, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
//	} while(signaled == 0);
//	// diable sync
//	rc = Dfe_issueSyncUpdateDducDistMap(hDfe, 3, DFE_FL_SYNC_GEN_SIG_NEVER);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// read BB rx pm
//	rc = readBBRXPM(hDfe, 0/*pmId*/, &newpeak[0], &newrms[0]);
//	rc = readBBRXPM(hDfe, 2/*pmId*/, &newpeak[1], &newrms[1]);
//
//	// verify the gain change
//	if (fabs(newrms[0]-refrms[1])<0.1 && fabs(newrms[1]-refrms[0])<0.1)
//		printf("DDUC distributor test PASS!\n");
//	else
//		printf("DDUC distributor test FAIL!\n");
//
//	// change back summer shift gain
//	rc = Dfe_progSummerShift(hDfe, 0/*cfrId*/, 0/*strId*/, -24);
//	rc = Dfe_progSummerShift(hDfe, 1/*cfrId*/, 0/*strId*/, -24);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
///* FB EQ */
//	rc = Dfe_progBbrxPowmtr(hDfe, 1/*pmId*/, &rxPmCfg);
//
//	// Issue sync to update BBRX power meter
//	rc = Dfe_issueSyncUpdateBbrxPowmtr(hDfe, 1/*pmId*/, PM_SSEL);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, PM_SSEL, &signaled);
//	} while(signaled == 0);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	rc = readBBRXPM(hDfe, 1/*pmId*/, &refpeak[0], &refrms[0]);
//	// Program Fb Equalizer
//	rc = Dfe_progFbEqr(hDfe, DFE_FL_FB_BLOCK0, 16, &FbEqrTaps);
//
//	// Issue sync update Fb Equalizer
//	rc = Dfe_issueSyncUpdateFbEqr(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
//	do // check the sync status
//	{
//		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
//	} while(signaled == 0);
//	// disable sync
//	rc = Dfe_issueSyncUpdateFbEqr(hDfe, DFE_FL_SYNC_GEN_SIG_NEVER);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	rc = readBBRXPM(hDfe, 1/*pmId*/, &newpeak[0], &newrms[0]);
//
//	if(fabs(refrms[0]-newrms[0]-6.f)<0.1f)
//		printf("FB EQ test PASS!\n");
//	else
//		printf("FB EQ test FAIL!\n");
//
///* TX PA protection and CFR prot gain and jesd tx testbus */
//	// disable all testbus
//	rc = Dfe_disableAllTestbus(hDfe);
//
//	// Program JESD Tx testbus
//	rc = Dfe_progJesdTxTestbus(hDfe, DFE_FL_JESDTX_TESTBUS_SEL_TX0_MUX_IN);
//
//	// Program CB buf configuration
//	bufCfg[0].cbSet.sel = DFE_FL_CB_NODE1;
//	bufCfg[0].cbSet.busSel = 0;
//	bufCfg[0].cbSet.not_used = 0;
//	bufCfg[0].length = 256;
//	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[0]);
//
//	bufCfg[1].cbSet.sel = DFE_FL_CB_NODE3;
//	bufCfg[1].cbSet.busSel = 0;
//	bufCfg[1].cbSet.not_used = 0;
//	bufCfg[1].length = 256;
//	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[1]);
//
//	bufCfg[2].cbSet.sel = DFE_FL_CB_NODE8;
//	bufCfg[2].cbSet.busSel = 0;
//	bufCfg[2].cbSet.not_used = 0;
//	bufCfg[2].length = 256;
//	rc = Dfe_progCbBufcfg(hDfe, &bufCfg[2]);
//
//	// Program Tx PA protection
//	txPAprotPeak.TH0 = 65535;
//	txPAprotPeak.threshold = 131071;
//	txPAprotPeak.cc_thr = 65535;
//	txPAprotPeak.peak_thr = 65535;
//	txPAprotPeak.peakgain_thr = 65535;
//
//	txPAprotRms.TH1 = 3600;
//	txPAprotRms.TH2 = 7800;
//	txPAprotRms.TH4 = 2048;
//	txPAprotRms.th1Sel = 0;
//	txPAprotRms.th2Sel = 1;
//	txPAprotRms.th6Sel = 1;
//	txPAprotRms.mu0 = 12;
//	txPAprotRms.mu1 = 10;
//
//	mask = 0x3; // enable mask for TH1 and TH2
//	rc = Dfe_progTxPaProtection(hDfe, DFE_FL_TXA_PATHA, txPAprotPeak, txPAprotRms, mask);
//
//	// Get Tx PA protection interrupt status
//	txPAprotIntr.txDev = DFE_FL_TXA_PATHA;
//	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
//	printf("TxPaIntrpt Initial status...\n");
//	printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
//	// Read Tx PA protection power status
//	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
//	printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa Initial Status, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// increasing 2dB CFR gain
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 2.0f);
//	Wait(1000);
//
//	// Get Tx PA protection interrupt status
//	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
//	printf("TxPaIntrpt after increasing 2dB post CFR gain...\n");
//	printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
//	// Read Tx PA protection power status
//	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
//	printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa after increasing 2dB post CFR gain, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// change back to unit gain
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.0f);
//	Wait(1000);
//	// clear interrupt status
//	rc = Dfe_clearTxPAprotIntrStatus(hDfe, DFE_FL_TXA_PATHA);
//	// Get Tx PA protection interrupt status
//	txPAprotIntr.txDev = DFE_FL_TXA_PATHA;
//	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
//	printf("TxPaIntrpt after changing back post CFR gain and clear status...\n");
//	printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa after changing back post CFR gain, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// increase 5dB CFR gain
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 5.0f);
//	Wait(1000);
//
//	// Get Tx PA protection interrupt status
//	rc = Dfe_getTxPAprotIntrStatus(hDfe, &txPAprotIntr);
//	printf("TxPaIntrpt after increasing 5dB post CFR gain...\n");
//	printf("     lCfrG=%d, sd=%d, sdpd=%d, 1=%d, 2=%d, 3=%d, 6=%d\n", txPAprotIntr.lowerCfrGain, txPAprotIntr.shutdown, txPAprotIntr.stopDpd, txPAprotIntr.intrpt1, txPAprotIntr.intrpt2, txPAprotIntr.intrpt3, txPAprotIntr.intrpt6);
//	// Read Tx PA protection power status
//	rc = Dfe_getTxPAprotPwrStatus(hDfe, DFE_FL_TXA_PATHA, 1/*clear after read*/, &txPAprotPwrStatus);
//	printf("     MaxMag=%d, d50=%d, d51=%d\n", txPAprotPwrStatus.mag, txPAprotPwrStatus.d50, txPAprotPwrStatus.d51);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa after increasing 5dB post CFR gain, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// change back to unit gain
//	rc = SetPostCFRGain(hDfe, 0/*cfrDev*/, DFE_FL_CFR_PATH_0, 0.0f);
//	Wait(1000);
//	// without clear interrupt status the feedback data is still zero.
////	// clear interrupt status
////	rc = Dfe_clearTxPAprotIntrStatus(hDfe, DFE_FL_TXA_PATHA);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa after changing back post CFR gain, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//
//	// jesd tx siggen
//	rc = Dfe_progJesdTxSigGenRamp(hDfe, DFE_FL_JESDTX_SIGNAL_GEN_ALL, 1/*enable*/, 0/*start*/, 0x2000/*stop*/, 0x2/*slope*/);
//	// Issue sync update JESDTX Signal Generator.
//	rc = Dfe_issueSyncUpdateJesdTxSigGen(hDfe, DFE_FL_JESDTX_SIGNAL_GEN_ALL, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/);
//
//	// Wait for the new data
//	rc = wait4Sync(hDfe, PM_SSEL /*DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0*/, 4);
//
//	// do CB
//	cbStatus = doStaticCB(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 100);
//	if(cbStatus == DFE_CBDONE)
//	{
//		printf("TxPa after enabling jesd tx siggen, CB address is %#x\n", (DFE_CbData *)g_cbDataOut+data_offset);
//		rc = readCB(hDfe, &bufCfg[0], 1, (DfeFl_CbStatus *)g_cbInfo+info_offset, (DFE_CbData *)g_cbDataOut+data_offset);
//		info_offset += 4;
//		data_offset += 4*DFE_FL_MAX_CB_LENGTH;
//	}
//	else
//		return(DFE_ERR_TIMEDOUT);
//	return (rc);
//}
//
