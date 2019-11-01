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

#ifdef _TMS320C6X
#include <c6x.h>
#endif
#include <ti/csl/csl.h>
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_tsc.h>
#include <ti/drv/dfe/dfe.h>
#include <ti/drv/dfe/dfe_osal.h>
#include <ti/drv/dfe/test/utils/dfetest.h>

//#undef CHK_MAX_TIME
//
//#define MK_INITS(pInits, _ssel, _initClkGate, _initState, _clearData) \
//do { \
//    (pInits)->ssel = (_ssel); \
//    (pInits)->initClkGate = (_initClkGate); \
//    (pInits)->initState = (_initState); \
//    (pInits)->clearData = (_clearData); \
//} while(0);
//
//#define MK_INITS_JESD(pInits, _ssel, _initClkGate, _initState, _clearData) \
//do { \
//    (pInits)->cmn.ssel = (_ssel); \
//    (pInits)->cmn.initClkGate = (_initClkGate); \
//    (pInits)->cmn.initState = (_initState); \
//    (pInits)->cmn.clearData = (_clearData); \
//    (pInits)->clearDataLane[0] = (_clearData); \
//    (pInits)->clearDataLane[1] = (_clearData); \
//    (pInits)->clearDataLane[2] = (_clearData); \
//    (pInits)->clearDataLane[3] = (_clearData); \
//} while(0);

//#ifdef CHK_MAX_TIME
//// we don't care sync
//#define issueSync(a, b)
//// we don't care sync_cntr
//#define activeSyncCntr(a, b, c)
//// update max time used
//#define UPD_MAX_TIME(t_start, t_now, t_max) \
//do { \
//    uint32_t t_delta; \
//    t_now = TSCL; \
//    t_delta = t_now - t_start; \
//    t_start = t_now; \
//    if(t_delta > t_max) t_max = t_delta; \
//} while(0);
//#else
//#define UPD_MAX_TIME(t_start, t_now, t_max)
//#endif

//uint32_t vbusp_read32  (uint32_t address)
//{
//	return( *(volatile uint32_t *)address );
//}
//
//void vbusp_write32 (uint32_t address, uint32_t data)
//{
//	*(volatile uint32_t *)address = data;
//}
//
//void vbusp_rmw32   (uint32_t address, uint32_t data, uint32_t mask)
//{
//	uint32_t temp = *(volatile uint32_t *)address;
//
//	temp = (temp & ~mask) | (data & mask);
//
//	*(volatile uint32_t *)address = temp;
//}

static inline uint32_t CSL_DFESerdesPllTxlockSts
(
 uint32_t base_addr
)
{
	uint32_t data;
	data = CSL_FEXTR(*(volatile uint32_t *)(base_addr + 0x1FC0 + 0x34), 28, 28);
	return (data);
}

static inline void CSL_DFESerdesReleaseTxIdl
(
 uint32_t base_addr
)
{
	CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1FC0 + 0x20), 25, 24, (uint32_t)0x2);
	CSL_FINSR(*(volatile uint32_t *)(base_addr + 0x1FC0 + 0x24), 25, 24, (uint32_t)0x2);
}

static inline uint32_t CSL_DFESerdesRxSts
(
 uint32_t base_addr,
 uint32_t bias
)
{
	uint32_t data;
	data = CSL_FEXTR(*(volatile uint32_t *)(base_addr + 0x1FC0 + bias), 1, 0);
	return (data);
}

DFE_Err WaitSync(DFE_CallbackContext cbkCtx, DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig)
{
	DFE_Err dfeErr;
	uint32_t i, signaled;
	int ct, *p;

	p = (int *)cbkCtx;
	ct = *p;

	if(ct < 0) // wait forever
	{
		do // check the sync status
		{
			dfeErr = Dfe_getSyncStatus(hDfe, syncSig, &signaled);
		} while(signaled == 0);
	}
	else
	{
		i = 0;
		signaled = 0;
		do // check the sync status
		{
			dfeErr = Dfe_getSyncStatus(hDfe, syncSig, &signaled);
			i++;
		} while(signaled == 0 && i < ct);

		if(signaled == 0)
		{
			dfeErr = DFE_ERR_TIMEDOUT;
		}
	}

	return (dfeErr);
}

DFE_Err setSyncPulseWidth_lld(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, uint32_t pulseWidth)
{
    CSL_Status status;
    DfeFl_MiscSyncGenGeneric syncGeneric;

	syncGeneric.syncSig = syncSig;
	syncGeneric.data    = pulseWidth;
	status = dfeFl_MiscHwControl(hDfe->hDfeMisc[0], DFE_FL_MISC_CMD_CFG_SYNC_ONE_SHOT, &syncGeneric);
	if(status != CSL_SOK)
	{
		Dfe_osalLog("dfeFl_MiscHwControl() error = %d", status);
		return DFE_ERR_HW_CTRL;
	}

	return DFE_ERR_NONE;
}

extern uint32_t serdes_cfg0_base;
extern uint32_t serdes_cfg1_base;
DFE_Err startDfe_lld(DFE_Handle hDfe)
{
	DFE_Err dfeErr;
	int waitctx;
	uint32_t lpbkSync[DFE_FL_JESD_NUM_LINK];
	DfeFl_JesdRxLoopbackConfig rxLoopbackCfg;
	uint32_t txLaneEnb[4], txLinkAssign[4];
	uint32_t rxLaneEnb[4], rxLinkAssign[4];
	uint32_t data, lane_lpbk;

	// soft reset dfe
	dfeErr = Dfe_softReset(hDfe);

	if(dfeErr == DFE_ERR_NONE)
		dfeErr = Dfe_getJesdLoopback(hDfe, &lpbkSync[0], &rxLoopbackCfg);
	else
		return (dfeErr);

	if(dfeErr == DFE_ERR_NONE)
		dfeErr = Dfe_getJesdTxLaneEnable(hDfe, &txLaneEnb[0], &txLinkAssign[0]);
	else
		return (dfeErr);

	if(dfeErr == DFE_ERR_NONE)
		dfeErr = Dfe_getJesdRxLaneEnable(hDfe, &rxLaneEnb[0], &rxLinkAssign[0]);
	else
		return (dfeErr);

	if(dfeErr == DFE_ERR_NONE)
	{
		// poll for tx lock (ststx[4]) - serdes0 - lane0 & lane1 ==> pll_ok = 28th bit
		// pll_ok is equivalent to ststx[4].
		if (txLaneEnb[0] && !rxLoopbackCfg.lane0 && txLaneEnb[1] && !rxLoopbackCfg.lane1)
		{
			data = 0x0;
//			while ( (data & 0x10000000) != 0x10000000 )
//			{
//				data=vbusp_read32(CSL_CSISC2_0_SERDES_CFG_REGS +0x00001FC0+0x00000034);
//			}
			while ( data != 1)
			{
				data = CSL_DFESerdesPllTxlockSts(serdes_cfg0_base);
			}
		}

		// poll for tx lock (ststx[4]) - serdes1 - lane0 & lane1 ==> pll_ok = 28th bit
		if (txLaneEnb[2] && !rxLoopbackCfg.lane2 && txLaneEnb[3] && !rxLoopbackCfg.lane3)
		{
			data = 0x0;
//			while ( (data & 0x10000000) != 0x10000000 )
//			{
//				data=vbusp_read32(CSL_CSISC2_1_SERDES_CFG_REGS +0x00001FC0+0x00000034);
//			}
			while ( data != 1)
			{
				data = CSL_DFESerdesPllTxlockSts(serdes_cfg1_base);
			}
		}
	}
	else
		return (dfeErr);

	waitctx = -1;

	dfeErr = Dfe_initTgtTx(hDfe, (DFE_CallbackContext)&waitctx, WaitSync);

	if(dfeErr == DFE_ERR_NONE)
	{
		// Disabling serdes TX IDLE
//	    vbusp_rmw32(CSL_CSISC2_0_SERDES_CFG_REGS + 0x00001FC0 + 0x00000020, 0x02000000, 0x03000000);
//	    vbusp_rmw32(CSL_CSISC2_0_SERDES_CFG_REGS + 0x00001FC0 + 0x00000024, 0x02000000, 0x03000000);
//	    vbusp_rmw32(CSL_CSISC2_1_SERDES_CFG_REGS + 0x00001FC0 + 0x00000020, 0x02000000, 0x03000000);
//	    vbusp_rmw32(CSL_CSISC2_1_SERDES_CFG_REGS + 0x00001FC0 + 0x00000024, 0x02000000, 0x03000000);
	    CSL_DFESerdesReleaseTxIdl(serdes_cfg0_base);
	    CSL_DFESerdesReleaseTxIdl(serdes_cfg1_base);
	    //Released the serdes TX IDLE !!

	    //This is the gc_dfe_iqn_lamarr testbench, which has the serdes. Poll the
	    //stsrx[2] (the OK bit) and stsrx[1] (the LOSS bit) from the appropriate serdes
	    //macros.

	    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes0 - lane0
	    if (rxLaneEnb[0] && !rxLoopbackCfg.lane0)
	    {
	        data = 0x0;
//	        while ( (data & 0x00000003) != 0x00000002 )
//	            data=vbusp_read32( CSL_CSISC2_0_SERDES_CFG_REGS +0x00001FC0+0x00000020);
	        while ( data != 0x2 )
	            data=CSL_DFESerdesRxSts( serdes_cfg0_base, 0x20 );
	    }

	    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes0 - lane1
	    if (rxLaneEnb[1] && !rxLoopbackCfg.lane1)
	    {
	        data = 0x0;
//	        while ( (data & 0x00000003) != 0x00000002 )
//	            data=vbusp_read32( CSL_CSISC2_0_SERDES_CFG_REGS +0x00001FC0+0x00000024);
	        while ( data != 0x2 )
	            data=CSL_DFESerdesRxSts( serdes_cfg0_base, 0x24 );
	    }

	    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes1 - lane0
	    if (rxLaneEnb[2] && !rxLoopbackCfg.lane2)
	    {
	        data = 0x0;
//	        while ( (data & 0x00000003) != 0x00000002 )
//	            data=vbusp_read32( CSL_CSISC2_1_SERDES_CFG_REGS+0x00001FC0+0x00000020);
	        while ( data != 0x2 )
	            data=CSL_DFESerdesRxSts( serdes_cfg1_base, 0x20 );
	    }

	    // poll for rx OK (stsrx[2]) & rx LOSS (stsrx[1]) - serdes1 - lane1
	    if (rxLaneEnb[3] && !rxLoopbackCfg.lane3)
	    {
	        data = 0x0;
//	        while ( (data & 0x00000003) != 0x00000002 )
//	            data=vbusp_read32( CSL_CSISC2_1_SERDES_CFG_REGS +0x00001FC0+0x00000024);
	        while ( data != 0x2 )
	            data=CSL_DFESerdesRxSts( serdes_cfg1_base, 0x24 );
	    }
	}
	else
		return (dfeErr);

    lane_lpbk = rxLoopbackCfg.lane0 || rxLoopbackCfg.lane1 || rxLoopbackCfg.lane2 || rxLoopbackCfg.lane3;
    if (lane_lpbk && hDfe->bbrx_chksum_ssel && (hDfe->bbtx_siggen_ssel == hDfe->bbrx_chksum_ssel)) //checksum
    	hDfe->ulStrobe_Sync = DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0;
    else
    	hDfe->ulStrobe_Sync = DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0;
	dfeErr = Dfe_initTgtRx(hDfe, (DFE_CallbackContext)&waitctx, WaitSync);

	if(dfeErr == DFE_ERR_NONE)
		dfeErr = setSyncPulseWidth_lld(hDfe, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, 1);
	else
		return(dfeErr);

	return (dfeErr);
}
