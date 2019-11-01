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
#ifndef __DFE_INC_H__
#define __DFE_INC_H__

#include <ti/drv/dfe/dfe_fl.h>
#include <ti/drv/dfe/dfe_fl_bb.h>
#include <ti/drv/dfe/dfe_fl_dduc.h>
#include <ti/drv/dfe/dfe_fl_summer.h>
#include <ti/drv/dfe/dfe_fl_autocp.h>
#include <ti/drv/dfe/dfe_fl_cfr.h>
#include <ti/drv/dfe/dfe_fl_cdfr.h>
#include <ti/drv/dfe/dfe_fl_dpd.h>
#include <ti/drv/dfe/dfe_fl_dpda.h>
#include <ti/drv/dfe/dfe_fl_tx.h>
#include <ti/drv/dfe/dfe_fl_jesd.h>
#include <ti/drv/dfe/dfe_fl_rx.h>
#include <ti/drv/dfe/dfe_fl_fb.h>
#include <ti/drv/dfe/dfe_fl_cb.h>
#include <ti/drv/dfe/dfe_fl_misc.h>
#include <ti/drv/dfe/dfe_fl_cbParams.h>

// test result code
#define TEST_RESULT_CODE_OK					(0u)
#define TEST_RESULT_CODE_FAIL_OPEN_DFE		(1u)
#define TEST_RESULT_CODE_FAIL_OPEN_BB		(2u)
#define TEST_RESULT_CODE_FAIL_OPEN_DDUC0	(3u)
#define TEST_RESULT_CODE_FAIL_OPEN_DDUC1	(4u)
#define TEST_RESULT_CODE_FAIL_OPEN_DDUC2	(5u)
#define TEST_RESULT_CODE_FAIL_OPEN_DDUC3	(6u)
#define TEST_RESULT_CODE_FAIL_OPEN_SUMMER	(7u)
#define TEST_RESULT_CODE_FAIL_OPEN_AUTOCP	(8u)
#define TEST_RESULT_CODE_FAIL_OPEN_CFR0		(9u)
#define TEST_RESULT_CODE_FAIL_OPEN_CFR1		(10u)
#define TEST_RESULT_CODE_FAIL_OPEN_CDFR		(11u)
#define TEST_RESULT_CODE_FAIL_OPEN_DPD		(12u)
#define TEST_RESULT_CODE_FAIL_OPEN_DPDA		(13u)
#define TEST_RESULT_CODE_FAIL_OPEN_TX		(14u)
#define TEST_RESULT_CODE_FAIL_OPEN_JESD		(15u)
#define TEST_RESULT_CODE_FAIL_OPEN_RX		(16u)
#define TEST_RESULT_CODE_FAIL_OPEN_FB		(17u)
#define TEST_RESULT_CODE_FAIL_OPEN_CB		(18u)
#define TEST_RESULT_CODE_FAIL_OPEN_MISC		(19u)
#define TEST_RESULT_CODE_FAIL_OPEN_CPP_DMA  (20u)
#define TEST_RESULT_CODE_FAIL_OPEN_CPP_DECSRIP  (21u)
#define TEST_RESULT_CODE_FAIL_ARM_CPP_DMA   (22u)
#define TEST_RESULT_CODE_FAIL_WAIT_SYNC	    (23u)

#define TEST_RESULT_CODE_FAIL_WRITE         (127u)
#define TEST_RESULT_CODE_CHECK_RECORD		(255u)

typedef struct
{
	Uint32 rwb; // 1 = read, write = 0
	Uint32 addr; // addr in dfe scope
	Uint32 value; // value 
	Uint32 mask; // valid bit mask
} AVV_RwTestRecord;

typedef struct
{
    Uint32 addr;        // dfe mpu address
    Uint32 value;       // value read back
    Uint32 expected;    // value expected
    Uint32 mask;        // valid bitmask
} AVV_TestResult;

#define min(a, b)  (((a) < (b)) ? (a) : (b))
//#define DFE_FL_CB_NUM_BUF 4
//#define DFE_FL_MAX_CB_LENGTH 8192
typedef enum
{
	DFE_CBDONE = 0,
	DFE_CBTIMEOUT,
	DFE_CBNOSYNC
} DfeCbStatus;

//typedef enum
//{
//	DFE_CBMPU = 0,
//	DFE_CBCAPC,
//	DFE_CBSRC,
//	DFE_CBC_TRIGGER,
//	DFE_CBC_SHARING,
//	DFE_CBC_MONITOR,
//	DFE_CBF
//} DfeCbMode;

typedef struct
{
	DfeFl_CbModeSet  cbSet;
	Uint32 dly;
	Uint32 rate_mode;   // 0 = 1s/1c mode; 1 = 2s/1c mode
	Uint32 frac_cnt;    // capture buffer A fractional counter length minus 1; range 0-15; value depends on the relative sampling rates for different buffers
	Uint32 frac_cnt_ssel;
	Uint32 len_cnt_ssel;
	Uint32 length;
} AVV_DfeCbBufCfg;

typedef struct
{
	DfeFl_CbNodeCfg CbNodeCfg[DFE_FL_CB_NUM_NODE];
	AVV_DfeCbBufCfg CbBufCfg[DFE_FL_CB_NUM_BUF];
	Uint32 timeOut;
	Uint32 cbSsel;
} AVV_DfeCbSetup;

typedef struct
{
	DfeFl_CbBuf cbBuf;
	Uint32	not_used;
	Uint32  done_addr;
	Uint32	done_frac_cnt;
	Uint32  done_length_cnt;
	Uint32	full_flag;
} AVV_DfeCbInfo;

typedef struct
{
	Uint32 Idata[DFE_FL_MAX_CB_LENGTH];
	Uint32 Qdata[DFE_FL_MAX_CB_LENGTH];
} AVV_DfeCbData;

// used to on-the-fly change CB buf config
typedef struct
{
	DfeFl_CbBuf cbBuf;
	Uint32 sel;
	Uint32 busSel;
	Uint32 not_used;
	Uint32 dly;
	Uint32 length;
} AVV_DfeCbBufTestCfg;

typedef struct
{
    Uint32 bus;
    Uint32 busPos;
} AVV_JesdTxLaneMapPos;

typedef struct
{
    Uint32 lane;
    Uint32 lanePos;
    Uint32 zeroBits;
} AVV_JesdRxBusMapPos;

typedef struct
{
    Uint32 numAxCs;
    Uint32 numSmpls;
    Uint32 resv0;
    Uint32 resv1;
} AVV_AxcInData;

// power up dfess
int DFESS_Powerup();
// program dfess PLL
int DFESS_ProgPll(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);
// program DFE_TEST_CNTR
void DFESS_ProgDfeTestCntr(Uint32 testCntr);

// open DFE and its subchips
int openDfe();
// load DFE tgtcfg
int loadDfe();
// put DFE in running condition
void startDfe();
// issue a sync
int issueSync(Uint32 syncSig, Uint32 wait);
// program sync counter
void resetSyncCntr(Uint32 cntrInst);
void activeSyncCntr(Uint32 cntrInst, Uint32 startSsel, Uint32 progSsel);
void progSyncCntr(Uint32 cntrInst, Uint32 dly, Uint32 period, Uint32 pulseWidth);
// set sync pulse width (0 - raw, n - width)
void setSyncPulseWidth(Uint32 syncSig, Uint32 pulseWidth);
// program DPD TestGen to ramp
void progDpdTestGenRamp(Uint32 tgDev, Uint32 start, Uint32 stop, Uint32 slope, Uint32 ssel);
// program BB TestGen to ramp
void progBbTestGenRamp(Uint32 tgDev, Uint32 start, Uint32 stop, Uint32 slope, Uint32 ssel);
// program BB test bus
void progBbTestbus(Uint32 testCbCtrl, Uint32 testCbAxc);
// disable BB sig-gen
void disableBbTestGen(Uint32 tgDev);

// set pre cfr gain
void SetPreCFRGain(Uint32 cfrDevice, Uint32 cfrPath, Uint32 gain);
// set post cfr gain
void SetPostCFRGain(Uint32 cfrDevice, Uint32 cfrPath, Uint32 gain);
// set cfr coeff
void setCfrCoeff(Uint32 cfrDevice, Uint32 CoeffType, Uint32 *cfrCoeff_i, Uint32 *cfrCoeff_q, Uint32 NumCoeff);

// Arm CB and read CB data
DfeCbStatus doCBread(AVV_DfeCbSetup *CbSetup, AVV_DfeCbData *cBufData, AVV_DfeCbInfo *CbInfo, Uint32 ssel, DfeFl_CbComplexInt *CbTemp);
DfeCbStatus doCBPreread(AVV_DfeCbSetup *CbSetup, AVV_DfeCbInfo *CbInfo, Uint32 ssel);
void CbWait(int count);

void mapJesdTx2Lane(Uint32 lane, AVV_JesdTxLaneMapPos laneMap[4]);
void mapJesdLane2Rx(Uint32 rxBus, AVV_JesdRxBusMapPos busMap[4]);
void setJesdTxSysrefMode(Uint32 link, Uint32 mode);
void setJesdRxSysrefMode(Uint32 link, Uint32 mode);
void progJesdTxTestbus(int tp);
void progJesdRxTestbus(int tp);
void progFbIoMux(DfeFl_CbMux cbMux, DfeFl_FbIoMux ioMux);
void getFbIoMux(DfeFl_CbMux *cbMux, DfeFl_FbIoMux *ioMux);
void progFbGain(DfeFl_FbBlk blk, Uint32 gain_real, Uint32 gain_image, Uint32 ssel);
void setSyncnLoopback(Uint32 link, Uint32 lpbkOn);

Uint32 vbusp_read32  (Uint32 address);
void vbusp_write32 (Uint32 address, Uint32 data);
void vbusp_rmw32   (Uint32 address, Uint32 data, Uint32 mask);

extern Uint32 *g_dfeTgtCfgSize;
extern AVV_RwTestRecord *g_dfeTgtCfg;
extern AVV_TestResult *g_testResult;
extern Uint32 *g_testResultLen;
extern Uint32 *g_testResultCode;
extern DfeFl_Handle hDfe;
extern DfeFl_BbHandle hDfeBb[DFE_FL_BB_PER_CNT];
extern DfeFl_DducHandle hDfeDduc[DFE_FL_DDUC_PER_CNT];
extern DfeFl_SummerHandle hDfeSummer[DFE_FL_SUMMER_PER_CNT];
extern DfeFl_AutocpHandle hDfeAutocp[DFE_FL_AUTOCP_PER_CNT];
extern DfeFl_CfrHandle hDfeCfr[DFE_FL_CFR_PER_CNT];
extern DfeFl_CdfrHandle hDfeCdfr[DFE_FL_CDFR_PER_CNT];
extern DfeFl_DpdHandle hDfeDpd[DFE_FL_DPD_PER_CNT];
extern DfeFl_DpdaHandle hDfeDpda[DFE_FL_DPDA_PER_CNT];
extern DfeFl_TxHandle hDfeTx[DFE_FL_TX_PER_CNT];
extern DfeFl_RxHandle hDfeRx[DFE_FL_RX_PER_CNT];
extern DfeFl_CbHandle hDfeCb[DFE_FL_CB_PER_CNT];
extern DfeFl_JesdHandle hDfeJesd[DFE_FL_JESD_PER_CNT];
extern DfeFl_FbHandle hDfeFb[DFE_FL_FB_PER_CNT];
extern DfeFl_MiscHandle hDfeMisc[DFE_FL_MISC_PER_CNT];

#define SET_TEST_RESULT_CODE(rc)	do { *g_testResultCode = (rc); } while(0);

#endif // __DFE_INC_H__

