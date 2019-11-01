/*****************************************************************************\
*           TEXAS INSTRUMENTS INCORPORATED PROPRIETARY INFORMATION           
*                                                                            
*  Property of Texas Instruments 
*  For  Unrestricted  Internal  Use  Only 
*  Unauthorized reproduction and/or distribution is strictly prohibited.  
*  This product is protected under copyright law and trade secret law 
*  as an unpublished work.  
*  Created 2004, (C) Copyright 2003 Texas Instruments.  All rights reserved.
*------------------------------------------------------------------------------
*  Filename       : BBCmd.c
*  Date Created   : Feb 19, 2012
*  Last Modified  : Feb 19, 2012
*  Description    : DFE BB Cmd test
*
*  History        : v0.1
*  Project        : Lamarr
*  Owner          : Lamarr    : Xiaohan Chen
*                 : 
\*****************************************************************************/
#if (defined _TMS320C6X) && (defined USESYSBIOS)
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#else
#include <stdio.h>
#define System_printf		printf
#endif
#include <ti/drv/dfe/dfe.h>
#include <math.h>
#ifdef _TMS320C6X
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#endif
#ifdef DFE_CTL
#include <ti/drv/dfe/test/utils/cslUtils.h>
#include <ti/drv/dfe/test/utils/dfetest.h>
#else
#include "cslUtils.h"
#include "dfetest.h"
#endif

#define TEST_CB_SSEL	DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0
//#define TEST_CB_SSEL	DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0

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
static DfeFl_CbNodeCfg initNodeCfg[DFE_FL_CB_NUM_NODE] =
{
		//     cbNode, I0bus_sel, Q0bus_sel, I1bus_sel, Q1bus_sel, I0fsdly, Q0fsdly, I1fsdly, Q1fsdly, fsf, fsfm
		 {DFE_FL_CB_NODE0, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE1, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE2, 0,       0,         4,         4,         0,       1,       0,       1,     3,   3}
		,{DFE_FL_CB_NODE3, 0,       1,         0,         1,         0,       0,       1,       1,     3,   3}
		,{DFE_FL_CB_NODE4, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE5, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE6, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE7, 0,       1,         4,         5,         0,       0,       0,       0,     3,   3}
		,{DFE_FL_CB_NODE8, 0,       1,         4,         5,         0,       0,       0,       0,     0,   0}
};

Uint32 addrTable[18][2] =
{
		{0x00000000,  0x0006C0F4},
		{0x00100000,  0x001438BC},
		{0x00180000,  0x001C38BC},
		{0x00200000,  0x002438BC},
		{0x00280000,  0x002C38BC},
		{0x00900000,  0x00900074},
		{0x00B00000,  0x00B183FC},
		{0x00B80000,  0x00B983FC},
		{0x00F00000,  0x00F00000},
		{0x01000000,  0x01001118},
		{0x01300000,  0x01363FFC},
		{0x01500000,  0x01500000},
		{0x01600000,  0x01640BFC},
		{0x01A00000,  0x01A423FC},
		{0x01C00000,  0x01C00488},
		{0x01D00000,  0x01D42DD0},
		{0x01E80000,  0x01EC09FC},
		{0x01F00000,  0x01F00000}
};

void preloadTgtcfg(DFE_Handle hDfe)
{
    Uint32 ui, addr, i;

    for(i = 0; i < 18; i++)
		if(addrTable[i][1]>addrTable[i][0])
			for(ui = addrTable[i][0]; ui<=addrTable[i][1]; ui+=4)
			{
				addr = ui >> 2;
				hDfe->hDfe->regs[addr] = 0;
			}

    return;
}

void Wait(int count)
{
	int i, j, sum;

	for(i = 0; i < count; i++)
	{
		sum = 0;
		for(j = 0; j < 100; j++)
			sum++;
	}
}

DFE_Err wait4Sync(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, Uint32 Count)
{
	DFE_Err rc = DFE_ERR_NONE;
	int cbkCtx;
	Uint32 i;

	cbkCtx = -1;

	for(i = 0; i < Count; i++)
	{
		// clear signal status
		dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CLR_SYNC_INTR_STATUS, &syncSig);
		rc = WaitSync((DFE_CallbackContext)&cbkCtx, hDfe, syncSig);

		if(rc == DFE_ERR_TIMEDOUT)
			return DFE_ERR_SYNC_NOT_COME;
	}

	return (rc);
}

DFE_Err init_CB(DFE_Handle hDfe, DFE_CbBufCfg bufCfg[])
{
	DFE_Err rc;
	DfeFl_CbNodeCfg nodeCfg;
	Uint32 i;

	// Program CB node configuration
	for(i = 0; i < DFE_FL_CB_NUM_NODE; i++)
	{
		nodeCfg.cbNode = initNodeCfg[i].cbNode;
		nodeCfg.I0bus_sel = initNodeCfg[i].I0bus_sel;
		nodeCfg.Q0bus_sel = initNodeCfg[i].Q0bus_sel;
		nodeCfg.I1bus_sel = initNodeCfg[i].I1bus_sel;
		nodeCfg.Q1bus_sel = initNodeCfg[i].Q1bus_sel;
		nodeCfg.I0fsdly = initNodeCfg[i].I0fsdly;
		nodeCfg.Q0fsdly = initNodeCfg[i].Q0fsdly;
		nodeCfg.I1fsdly = initNodeCfg[i].I1fsdly;
		nodeCfg.Q1fsdly = initNodeCfg[i].Q1fsdly;
		nodeCfg.fsf = initNodeCfg[i].fsf;
		nodeCfg.fsfm = initNodeCfg[i].fsfm;
		rc = Dfe_progCbNodecfg(hDfe, &nodeCfg);
	}

	// Program CB buf configuration
	for(i = 0; i < DFE_FL_CB_NUM_BUF; i++)
	{
		bufCfg[i].cbSet.cbBuf = (DfeFl_CbBuf)i;
		bufCfg[i].cbSet.sel = DFE_FL_CB_NODE0;
		bufCfg[i].cbSet.busSel = 0;
		bufCfg[i].cbSet.ref_or_fb = 0;
		bufCfg[i].cbSet.not_used = 1;

		bufCfg[i].dly = 0;
		bufCfg[i].rate_mode = 0;
		bufCfg[i].frac_cnt = 0;
		bufCfg[i].frac_cnt_ssel = TEST_CB_SSEL;
		bufCfg[i].len_cnt_ssel = TEST_CB_SSEL;
		bufCfg[i].length = 1024;
		rc = Dfe_progCbBufcfg(hDfe, &bufCfg[i]);
	}

	return (rc);

}

DfeCbStatus doStaticCB(DFE_Handle hDfe, DfeFl_MiscSyncGenSig ssel, Uint32 timeOut)
{
	DFE_Err rc;
	Uint32 startTime;
	DfeFl_CbArm cbDoneStatus;
	DfeCbStatus cbStatus;

	// Arm CB and Issue Sync
	rc = Dfe_armCbIssueSync(hDfe, ssel);
	if (rc != DFE_ERR_NONE)
		return DFE_CBNOSYNC;

	// wait for cb done
	startTime = 0;
	while(1)
	{
		Wait(10);
		// Get CB done status
		rc = Dfe_getCbDoneStatus(hDfe, &cbDoneStatus);

		if(0 == cbDoneStatus.cbDone)
		{
			cbStatus = DFE_CBDONE;
			break;
		}
		if(startTime++ > timeOut)
		{
			if(0 == cbDoneStatus.sync)
				cbStatus = DFE_CBTIMEOUT;
			else
				cbStatus = DFE_CBNOSYNC;
			break;
		}
	}

	return(cbStatus);
}

DFE_Err readCB(DFE_Handle hDfe, DFE_CbBufCfg *bufCfg, Uint32 flag_18bit,
	    DfeFl_CbStatus cbStatus[],
	    DFE_CbData cbData[])
{
	DFE_Err rc = DFE_ERR_NONE;
	Uint32 i;
	static DfeFl_CbComplexInt cbTemp[DFE_FL_MAX_CB_LENGTH];
	// read cb data
	for(i = 0; i < DFE_FL_CB_NUM_BUF; i++)
	{
		// only read valid capture data
		if(bufCfg[i].cbSet.not_used == 0)
		{
			// Read Cb Buf and Status
			rc = Dfe_readCbBuf(hDfe, (DfeFl_CbBuf)i, bufCfg[i].length, flag_18bit, &cbTemp[0], &cbStatus[i], &cbData[i*DFE_FL_MAX_CB_LENGTH]);
		}
	}

	return(rc);
}

DFE_Err readBBTXPM(DFE_Handle hDfe, Uint32 pmId, float *peak, float *rms)
{
	DFE_Err rc;
	Uint32 complete;

	do
	{
		rc = Dfe_getBbtxPowmtrDoneIntrStatus(hDfe, pmId, &complete);
	}while(complete == 0);

	rc = Dfe_clearBbtxPowmtrDoneIntrStatus(hDfe, pmId);

	rc = Dfe_readBbtxPowmtr(hDfe, pmId, peak, rms);

	return(rc);
}

DFE_Err readBBRXPM(DFE_Handle hDfe, Uint32 pmId, float *peak, float *rms)
{
	DFE_Err rc;
	Uint32 complete;

	do
	{
		rc = Dfe_getBbrxPowmtrDoneIntrStatus(hDfe, pmId, &complete);
	}while(complete == 0);

	rc = Dfe_clearBbrxPowmtrDoneIntrStatus(hDfe, pmId);

	rc = Dfe_readBbrxPowmtr(hDfe, pmId, peak, rms);

	return(rc);
}

DFE_Err SetBBTxGain(DFE_Handle hDfe, Uint32 numAxcs, Uint32 axc[], float gain[])
{
	DFE_Err rc;
	Uint32 signaled, complete;

	// Program BBTX AxCs� gains.
	rc = Dfe_progBbtxGain(hDfe, numAxcs, &axc[0], &gain[0]);

	// Issue sync to update BBTX gain
	rc = Dfe_issueSyncUpdateBbtxGain(hDfe, 4/*ct*/, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateBbtxGain(hDfe, 4/*ct*/, DFE_FL_SYNC_GEN_SIG_NEVER);

	// Get BBTX gain update complete
	do
	{
		rc = Dfe_getBbtxGainUpdateComplete(hDfe, 4/*ct*/, &complete);
	}while(complete == 0);

	return(rc);
}

DFE_Err SetBBRxGain(DFE_Handle hDfe, Uint32 numAxcs, Uint32 axc[], float gain[])
{
	DFE_Err rc;
	Uint32 signaled, complete;

	rc = Dfe_progBbrxGain(hDfe, numAxcs, &axc[0], &gain[0]);

	// Issue sync to update BBRX gain
	rc = Dfe_issueSyncUpdateBbrxGain(hDfe, 4/*ct*/, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateBbrxGain(hDfe, 4/*ct*/, DFE_FL_SYNC_GEN_SIG_NEVER);

	// Get BBRX gain update complete
	do
	{
		rc = Dfe_getBbrxGainUpdateComplete(hDfe, 4/*ct*/, &complete);
	}while(complete == 0);

	return(rc);
}

DFE_Err SetPreCFRGain(DFE_Handle hDfe, Uint32 cfrDev, DfeFl_CfrPath cfrPath, float gain)
{
	DFE_Err rc;
	Uint32 signaled;

	rc = Dfe_progCfrPreGain(hDfe, cfrDev, cfrPath, gain);
	// Issue Sync Update Cfr preGain
	rc = Dfe_issueSyncUpdatCfrPreGain(hDfe, cfrDev, cfrPath, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdatCfrPreGain(hDfe, cfrDev, cfrPath, DFE_FL_SYNC_GEN_SIG_NEVER);

	return(rc);
}

DFE_Err SetPostCFRGain(DFE_Handle hDfe, Uint32 cfrDev, DfeFl_CfrPath cfrPath, float gain)
{
	DFE_Err rc;
	Uint32 signaled;

	rc = Dfe_progCfrPostGain(hDfe, cfrDev, cfrPath, gain);
	// Issue Sync Update Cfr preGain
	rc = Dfe_issueSyncUpdatCfrPostGain(hDfe, cfrDev, cfrPath, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdatCfrPostGain(hDfe, cfrDev, cfrPath, DFE_FL_SYNC_GEN_SIG_NEVER);

	return(rc);
}

DFE_Err SetRxIBPM(DFE_Handle hDfe)
{
	DFE_Err rc;
	Uint32 signaled;
	CSL_Uint64 unityMagsq;

	unityMagsq = 17179869184; // powf(2,34)

	// Program RX IBPM global
	rc = Dfe_progRxIbpmGlobal(hDfe, DFE_FL_RX_POWMTR_READ_MODE_HANDSHAKE, -200.f, -9.1, unityMagsq);

	// Program individual RX IBPM
	rc = Dfe_progRxIbpm(hDfe, 0/*pmId*/, 1/*one shot mode*/, 1/*normal mode*/, 0, 128/*nsamp*/, 512/*interval*/);

	// Issue Sync Update Rx Ibpm
	rc = Dfe_issueSyncUpdateRxIbpm(hDfe, 0/*pmId*/, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateRxIbpm(hDfe, 0/*pmId*/, DFE_FL_SYNC_GEN_SIG_NEVER);


	rc = Dfe_progRxIbpm(hDfe, 1/*pmId*/, 1/*one shot mode*/, 1/*normal mode*/, 0, 128/*nsamp*/, 512/*interval*/);
	// Issue Sync Update Rx Ibpm
	rc = Dfe_issueSyncUpdateRxIbpm(hDfe, 1/*pmId*/, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);
	// disable sync
	rc = Dfe_issueSyncUpdateRxIbpm(hDfe, 1/*pmId*/, DFE_FL_SYNC_GEN_SIG_NEVER);

	return (rc);
}

DFE_Err readRxIBPM(DFE_Handle hDfe, Uint32 pmId, float *power, float *peak, Uint32 *histCount1, Uint32 *histCount2)
{
	DFE_Err rc;
	Uint32 ackRead;

	// Issue Read Request to Rx Ibpm
	rc = Dfe_issueRxIbpmReadRequest(hDfe, pmId);

	// Get Rx Ibpm read ack
	do
	{
		rc = Dfe_getRxIbpmReadAck(hDfe, pmId, &ackRead);
	} while(ackRead==0);

	// Read Rx IBPM result
	rc = Dfe_readRxIbpmResult(hDfe, pmId, power, peak, histCount1, histCount2);

	// clear Read Request to Rx Ibpm
	rc = Dfe_clearRxIbpmReadRequest(hDfe, pmId);
	return (rc);

}

void readSummerGain(DFE_Handle hDfe, Uint32 cfrId, Uint32 strId, int *summerGain)
{
	DfeFl_SummerShiftCfg SumShift;

	SumShift.idx = (DfeFl_SummerCfrStr) (cfrId*2+strId);
	SumShift.data = 0;
	dfeFl_SummerGetHwStatus(hDfe->hDfeSummer[0], DFE_FL_SUMMER_QUERY_GET_SHIFT, &SumShift);
	*summerGain = (SumShift.data-6)*6;

	return;
}
DFE_Err countBBTXPM(DFE_Handle hDfe, Uint32 pmId, Uint32 *pCt)
{
	DFE_Err rc;
	Uint32 complete;

	do
	{
		rc = Dfe_getBbtxPowmtrDoneIntrStatus(hDfe, pmId, &complete);
	}while(complete == 0);

	rc = Dfe_clearBbtxPowmtrDoneIntrStatus(hDfe, pmId);
	*pCt += 1;

	return (rc);

}
#ifdef _TMS320C6X
volatile float peak_AxC0,rms_AxC0;
volatile float peak_AxC2,rms_AxC2;

volatile uint32_t ctlRxPacketSize[400];
uint32_t ctlindex=0;

#ifdef DFE_CTL
int readBBTXPM_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, float *result, int NumQue)
{
    Uint32 pkt, idx, pmId;
	Uint32  queCount;//, queCountFdq;
    Cppi_MonolithicDesc *ptrMonoDesc;
    Cppi_HostDesc *ptrHostDesc;
    int integer, fraction, exponent;
    Uint32          integrationPeriod;
    Uint32          *payloadPtr;
    Uint32			payloadLen;
    int 	status=0;
    DfeFl_BbPowerMeterConfig bbpmConfig;

    queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[BB_TX_CTL_CHL]);
    //queCountFdq = Qmss_getQueueEntryCount(hPktDma->rxFqCtrl[BB_TX_CTL_CHL]);

    if (queCount >= NumQue)
    {
		for (pkt = 0; pkt < queCount; pkt ++)
		{
			if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
			{
				ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[BB_TX_CTL_CHL]));
				if (ptrHostDesc == NULL)
					status++;
				UTILS_cacheInvalidate((void *)ptrHostDesc, 64);
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t**)&payloadPtr, &payloadLen);
				if (payloadLen != 256) {
				    ctlRxPacketSize[ctlindex] = payloadLen;
//				    if (*payloadPtr!=0) {
//				        exit(0);
//				    }
				    //System_printf("FATAL: incorrect CTL packet size\n");
				}
				ctlindex++;
				UTILS_cacheInvalidate((void *)payloadPtr, 256);
			} else {
				ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[BB_TX_CTL_CHL]));
				if (ptrMonoDesc == NULL)
					status++;
				UTILS_cacheInvalidate((void *)ptrMonoDesc, 12+256);
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
				//UTILS_cacheInvalidate((void *)payloadPtr, payloadLen);
                if (payloadLen != (256)) {
                    System_printf("FATAL: incorrect CTL packet size\n");
                }
			}

			// Read power meters from DFE registers to check
			Dfe_readBbtxPowmtr(hDfe,0,(float*)&peak_AxC0,(float*)&rms_AxC0);
			Dfe_readBbtxPowmtr(hDfe,2,(float*)&peak_AxC2,(float*)&rms_AxC2);
			pmId = 0;
			for (idx = 0; idx < 256/4; idx += 4, pmId++)
			{
				integer = CSL_FEXTR(payloadPtr[idx], 15, 6);
				exponent = CSL_FEXTR(payloadPtr[idx], 5, 0);
				fraction = payloadPtr[idx+1];
				// PEAK
				if ( (integer !=0) || (exponent !=0) || (fraction !=0))
				{
				    *result = 10*log10(((float)integer+((float)fraction/65536.0))*pow(2,exponent));
				} else *result = 0;
				result++;

				integer = CSL_FEXTR(payloadPtr[idx+2], 15, 6);
				exponent = CSL_FEXTR(payloadPtr[idx+2], 5, 0);
				fraction = payloadPtr[idx+3];
				// RMS
				// Query power meter configuration and recover the integration period
			    bbpmConfig.pmId =  pmId;
                dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_TXPM_CFG, &bbpmConfig);
				integrationPeriod = bbpmConfig.intgPd;//30720;
                if ( (integer !=0) || (exponent !=0) || (fraction !=0))
                {
                    *result = 10*log10(((float)integer+((float)fraction/65536.0))*pow(2,exponent));
                    *result -= 10*log10(integrationPeriod);
                } else *result = 0;
                result++;
			}

			if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
			{
				Qmss_queuePushDescSize(hPktDma->rxFqCtrl[BB_TX_CTL_CHL], (uint32_t*)ptrHostDesc, 64);
			} else {
				Qmss_queuePushDesc(hPktDma->rxFqCtrl[BB_TX_CTL_CHL], (uint32_t*)ptrMonoDesc);
			}
			status = 1;
		}
    } else {
    	status = 0;
    }
	return (queCount);
}

int readBBRXPM_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, float *result, int NumQue)
{
    Uint32 pkt, idx, pmId;
	Uint32  queCount;
//    MNAV_MonolithicPacketDescriptor *mono_pkt;
	Cppi_MonolithicDesc *ptrMonoDesc;
	Cppi_HostDesc *ptrHostDesc;
//    MNAV_HostPacketDescriptor *host_pkt;
    int integer, fraction, exponent;
    Uint32          integrationPeriod;
    Uint32          *payloadPtr;
    Uint32			payloadLen;
    int status=0;
    DfeFl_BbPowerMeterConfig bbpmConfig;

    queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[BB_RX_CTL_CHL]);

    queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[BB_RX_CTL_CHL]);

    if (queCount >= NumQue)
    {
		for (pkt = 0; pkt < queCount; pkt ++)
		{
			if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
			{
				ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[BB_RX_CTL_CHL]));
				if (ptrHostDesc == NULL)
					status++;
				UTILS_cacheInvalidate((void *)ptrHostDesc, 64);
				Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t**)&payloadPtr, &payloadLen);
                if (payloadLen != 256) {
                    //System_printf("FATAL: incorrect CTL packet size\n");
                    ctlRxPacketSize[ctlindex] = payloadLen;
                }
                ctlindex++;
				UTILS_cacheInvalidate((void *)payloadPtr, 256);
			} else {
				ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[BB_RX_CTL_CHL]));
				if (ptrMonoDesc == NULL)
					status++;
				UTILS_cacheInvalidate((void *)ptrMonoDesc, 12+256);
				Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
                if (payloadLen != (256)) {
                    System_printf("FATAL: incorrect CTL packet size\n");
                }
			}
            pmId = 0;
            for (idx = 0; idx < 256/4; idx += 4, pmId++)
			{
				integer = CSL_FEXTR(payloadPtr[idx], 15, 6);
				exponent = CSL_FEXTR(payloadPtr[idx], 5, 0);
				fraction = payloadPtr[idx+1];
				// PEAK
                if ( (integer !=0) || (exponent !=0) || (fraction !=0))
                {
                    *result = 10*log10(((float)integer+((float)fraction/65536.0))*pow(2,exponent));
                    // Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block, so shifting the measurement result by 8 bits
                    *result -= 10*log10((float)(256*256));
                } else *result = 0;
                result++;

				integer = CSL_FEXTR(payloadPtr[idx+2], 15, 6);
				exponent = CSL_FEXTR(payloadPtr[idx+2], 5, 0);
				fraction = payloadPtr[idx+3];
				// RMS
                // Query power meter configuration and recover the integration period
                bbpmConfig.pmId =  pmId;
                dfeFl_BbGetHwStatus(hDfe->hDfeBb[0], DFE_FL_BB_QUERY_RXPM_CFG, &bbpmConfig);
                integrationPeriod = bbpmConfig.intgPd;//30720;
                if ( (integer !=0) || (exponent !=0) || (fraction !=0))
                {
                    *result = 10*log10(((float)integer+((float)fraction/65536.0))*pow(2,exponent));
                    // Rx samples are on 24 bits and scaled down to 16 bits after the power meter/beAGC block, so shifting the measurement result by 8 bits
                    *result -= 10*log10((float)(integrationPeriod*256*256));
                } else *result = 0;
                result++;
			}

			if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
			{
				Qmss_queuePushDesc(hPktDma->rxFqCtrl[BB_RX_CTL_CHL], (uint32_t*)ptrHostDesc);
//				Qmss_queuePushDescSize(hPktDma->rxFqCtrl[BB_RX_CTL_CHL], (uint32_t*)ptrHostDesc, 64);
			} else {
				Qmss_queuePushDesc(hPktDma->rxFqCtrl[BB_RX_CTL_CHL], (uint32_t*)ptrMonoDesc);
			}
			status = 1;
		}
    } else {
    	status = 0;
    }
	return (queCount);
}

DFE_Err enableCB_DMA(DFE_Handle hDfe, DFE_CbBufCfg *bufCfg, DfeFl_CbStatus cbStatus[])
{
	DFE_Err rc;
	Uint32 i, signaled;
	DfeFl_CbBufMode cbBufMode;

	for(i = 0; i < DFE_FL_CB_NUM_BUF; i++)
	{
		// only read valid capture status
		if(bufCfg[i].cbSet.not_used == 0)
		{
			cbBufMode.cbBuf = (DfeFl_CbBuf)i;
			cbBufMode.data = (Uint32)DFE_FL_CB_MPU;
			dfeFl_CbHwControl(hDfe->hDfeCb[0], DFE_FL_CB_CMD_SET_CB_BUF_MODE, &cbBufMode);

			cbStatus[i].cbBuf = (DfeFl_CbBuf)i;
			dfeFl_CbGetHwStatus(hDfe->hDfeCb[0], DFE_FL_CB_QUERY_GET_CB_STATUS, &cbStatus[i]);
		}
	}

	// Enable CB buf DMA
	rc = Dfe_enableCbBufDma(hDfe);

	do // check the sync status
	{
		rc = Dfe_getSyncStatus(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, &signaled);
	} while(signaled == 0);

	return (rc);
}

int readCB_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma)
{
	uint32_t queCount;
	int status = 0;
	uint32_t LinkLen;

	if(hDfe->flag_18bit == 1)
		LinkLen = 8;
	else
		LinkLen = 4;

//    // wait till data arrives Rx queue
    queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[CB_CTL_CHL]);

    if (queCount >= LinkLen)
    {
		if (queCount != LinkLen)
		{
			System_printf("Wrong queCont!\n");
			return(-1);
		} else {
			status = queCount;
		}
    } else {
    	status = 0;
    }
    return (status);
}

void resultCB_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, DFE_CbBufCfg *bufCfg, Uint32 flag_18bit,
	    DfeFl_CbStatus cbStatus[],
	    DFE_CbData cbData[])
{
	Cppi_MonolithicDesc *ptrMonoDesc;
	Cppi_HostDesc *ptrHostDesc;
	Uint32 pkt, idx, idxd;
    Uint32          *payloadPtr;
    Uint32			payloadLen;

    Uint32 j, k, dstIdx, srcIdx, queCount, tempReal, tempImag;

    queCount = Qmss_getQueueEntryCount(hPktDma->rxQCtrl[CB_CTL_CHL]);
	// read cb data
    for (pkt = 0; pkt < queCount; pkt ++)
    {
		if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
		{
			ptrHostDesc = (Cppi_HostDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[CB_CTL_CHL]));
			Cppi_getData (Cppi_DescType_HOST, (Cppi_Desc*)ptrHostDesc, (uint8_t**)&payloadPtr, &payloadLen);
		} else {
			ptrMonoDesc = (Cppi_MonolithicDesc *)QMSS_DESC_PTR(Qmss_queuePop(hPktDma->rxQCtrl[CB_CTL_CHL]));
			Cppi_getData (Cppi_DescType_MONOLITHIC, (Cppi_Desc*)ptrMonoDesc, (uint8_t**)&payloadPtr, &payloadLen);
		}

        // get the idx of cb
        if(flag_18bit == 1)
		{
        	idx = pkt>>1;
        	idxd = (pkt>>1)*DFE_FL_MAX_CB_LENGTH;
        }
        else
        {
          idx = pkt;
          idxd = pkt*DFE_FL_MAX_CB_LENGTH;
        }

        // start reading position
        srcIdx = (DFE_FL_MAX_CB_LENGTH + cbStatus[idx].doneAddr - bufCfg[idx].length) % DFE_FL_MAX_CB_LENGTH;
        dstIdx = min(DFE_FL_MAX_CB_LENGTH-srcIdx, bufCfg[idx].length);
        if (bufCfg[idx].cbSet.not_used == 0) // read the cb
        {
        	if(flag_18bit == 1)
        	{
        		if (pkt%2 == 0) // MSB
				{
					k = 0;
					for(j = srcIdx; j < srcIdx+dstIdx; j++)
					{
						cbData[idxd+k].Idata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
						cbData[idxd+k].Qdata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
						k++;
					}
					for(j = 0; j < bufCfg[idx].length - dstIdx; j++)
					{
						cbData[idxd+k].Idata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
						cbData[idxd+k].Qdata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
						k++;
					}
				}
				else // LSB
				{
					k = 0;
					for(j = srcIdx; j < srcIdx+dstIdx; j++)
					{
						tempReal = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
						tempImag = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
						cbData[idxd+k].Idata = (cbData[idxd+k].Idata<<2)|(tempReal&0x3u);
						cbData[idxd+k].Qdata = (cbData[idxd+k].Qdata<<2)|(tempImag&0x3u);
						k++;
					}
					for(j = 0; j < bufCfg[idx].length - dstIdx; j++)
					{
						tempReal = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
						tempImag = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
						cbData[idxd+k].Idata = (cbData[idxd+k].Idata<<2)|(tempReal&0x3u);
						cbData[idxd+k].Qdata = (cbData[idxd+k].Qdata<<2)|(tempImag&0x3u);
						k++;
					}
				}
        	}
        	else
        	{
				k = 0;
				for(j = srcIdx; j < srcIdx+dstIdx; j++)
				{
					cbData[idxd+k].Idata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
					cbData[idxd+k].Qdata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
					k++;
				}
				for(j = 0; j < bufCfg[idx].length - dstIdx; j++)
				{
					cbData[idxd+k].Idata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
					cbData[idxd+k].Qdata = CSL_FEXT(payloadPtr[j], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
					k++;
				}
        	}
        }

		if(hPktDma->ctrlDescType == Cppi_DescType_HOST)
		{
			Qmss_queuePushDesc(hPktDma->rxFqCtrl[CB_CTL_CHL], (uint32_t*)ptrHostDesc);
		} else {
			Qmss_queuePushDesc(hPktDma->rxFqCtrl[CB_CTL_CHL], (uint32_t*)ptrMonoDesc);
		}
    }

}
#endif
#endif

