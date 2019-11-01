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
#ifndef __DFETEST_H__
#define __DFETEST_H__

#include <ti/drv/dfe/dfe.h>
#ifdef DFE_CTL
#include <ti/drv/iqn2/test/utils/mnavUtils.h>
#endif

#define BB_RX_CTL_CHL 0
#define BB_TX_CTL_CHL 1
#define CB_CTL_CHL 2

#ifndef PASS
#define PASS		(1)
#endif

#ifndef FAIL
#define FAIL		(0)
#endif
#define min(a, b)  (((a) < (b)) ? (a) : (b))

extern unsigned int tgtData[][2];

typedef enum
{
	DFE_CBDONE = 0,
	DFE_CBTIMEOUT,
	DFE_CBNOSYNC
} DfeCbStatus;

// program dfess PLL
int DFESS_ProgPll_csl(unsigned int clkr, unsigned int clkf, unsigned int clkod,unsigned int divmode);
//int verify_ul_circlebuff();

uint32_t vbusp_read32  (uint32_t address);
void vbusp_write32 (uint32_t address, uint32_t data);
void vbusp_rmw32   (uint32_t address, uint32_t data, uint32_t mask);
// set sync pulse width
DFE_Err setSyncPulseWidth(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, Uint32 pulseWidth);
// put DFE in running condition
DFE_Err startDfe(DFE_Handle hDfe);
DFE_Err WaitSync(DFE_CallbackContext cbkCtx, DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig);

void Wait(int count);
DFE_Err wait4Sync(DFE_Handle hDfe, DfeFl_MiscSyncGenSig syncSig, Uint32 Count);
void preloadTgtcfg(DFE_Handle hDfe);
DFE_Err init_CB(DFE_Handle hDfe, DFE_CbBufCfg bufCfg[]);
DfeCbStatus doStaticCB(DFE_Handle hDfe, DfeFl_MiscSyncGenSig ssel, Uint32 timeOut);
DFE_Err readCB(DFE_Handle hDfe, DFE_CbBufCfg *bufCfg, Uint32 flag_18bit,
	    DfeFl_CbStatus cbStatus[],
	    DFE_CbData cbData[]);
DFE_Err readBBTXPM(DFE_Handle hDfe, Uint32 pmId, float *peak, float *rms);
DFE_Err readBBRXPM(DFE_Handle hDfe, Uint32 pmId, float *peak, float *rms);
DFE_Err SetBBTxGain(DFE_Handle hDfe, Uint32 numAxcs, Uint32 axc[], float gain[]);
DFE_Err SetBBRxGain(DFE_Handle hDfe, Uint32 numAxcs, Uint32 axc[], float gain[]);
DFE_Err SetPreCFRGain(DFE_Handle hDfe, Uint32 cfrDev, DfeFl_CfrPath cfrPath, float gain);
DFE_Err SetPostCFRGain(DFE_Handle hDfe, Uint32 cfrDev, DfeFl_CfrPath cfrPath, float gain);
DFE_Err SetRxIBPM(DFE_Handle hDfe);
DFE_Err readRxIBPM(DFE_Handle hDfe, Uint32 pmId, float *power, float *peak, Uint32 *histCount1, Uint32 *histCount2);
void readSummerGain(DFE_Handle hDfe, Uint32 cfrId, Uint32 strId, int *summerGain);
DFE_Err countBBTXPM(DFE_Handle hDfe, Uint32 pmId, Uint32 *pCt);
#ifdef DFE_CTL
int readBBTXPM_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, float *result, int NumQue);
int readBBRXPM_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, float *result, int NumQue);
DFE_Err enableCB_DMA(DFE_Handle hDfe, DFE_CbBufCfg *bufCfg, DfeFl_CbStatus cbStatus[]);
int readCB_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
void resultCB_DMA(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, DFE_CbBufCfg *bufCfg, Uint32 flag_18bit,
	    DfeFl_CbStatus cbStatus[],
	    DFE_CbData cbData[]);
#endif

// test DFE using sig gen data and check the gain change
DFE_Err dynamicTest(DFE_Handle hDfe);
#ifdef DFE_CTL
// test IQN and CPP
DFE_Err doTestBbPm(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
DFE_Err updateBbPm(DFE_Handle hDfe);
void resultTestBbPm(DFE_Handle hDfe, PktDmaConfigHandle hPktDma, uint32_t slotcount);
DFE_Err disableCppDma(DFE_Handle hDfe);
DFE_Err doTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
int readTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
void resultTestCbDma(DFE_Handle hDfe, PktDmaConfigHandle hPktDma);
DFE_Err closeTestCbDma(DFE_Handle hDfe);
#endif



#endif // __DFETEST_H__

