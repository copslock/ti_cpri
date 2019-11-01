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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>

#include <ti/drv/iqn2/iqn2.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include "cslUtils.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif


#define 	MAX_SLOT				400
#define 	SLOT_NUM_FIRST_PUSH		60
#define 	SLOT_NUM_DATA_CHECK		7502//80
#define 	SYMBOL_NUM_FIRST_PUSH	140
#define 	SYMBOL_NUM_DATA_CHECK	280


#if LTE_RATE == 20
#define     NBCHANNELPERLINK	4
#elif LTE_RATE == 10
#define     NBCHANNELPERLINK	4
#elif LTE_RATE == 5
#define     NBCHANNELPERLINK	8
#elif LTE_RATE == 3
#define     NBCHANNELPERLINK	16
#elif LTE_RATE == 1
#define     NBCHANNELPERLINK	4
#endif


#define     NBSYMBOL			IQN2_LTE_SYMBOL_NUM											// Number of symbols per slot
#define 	NUMDESCCTL			8
#define		CTLDESCSIZE			720

#if LTE_RATE == 20
#define		LTESYMBOLSIZE		((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)    // 8848: FFT size + CP first
#define 	LTESYMBOL2SIZE		((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL_SIZE)*4 + 16)     // 8784: FFT size + CP 2nd-6th
#define 	SAMPLE_FREQ 		IQN2_SRATE_30P72MHZ
#define 	CPRI_PACK_MODE		IQN2_LTE_CPRI_8b8
#elif LTE_RATE == 10
#define		LTESYMBOLSIZE		((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL_SIZE)*4 + 16)
#define 	SAMPLE_FREQ 		IQN2_SRATE_15P36MHZ
#define 	CPRI_PACK_MODE		IQN2_LTE_CPRI_4b4
#elif LTE_RATE == 5
#define		LTESYMBOLSIZE		((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL_SIZE)*4 + 16)
#define 	SAMPLE_FREQ 		IQN2_SRATE_7P68MHZ
#define 	CPRI_PACK_MODE		IQN2_LTE_CPRI_2b2
#elif LTE_RATE == 3
#define		LTESYMBOLSIZE		((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL_SIZE)*4 + 16)
#define 	SAMPLE_FREQ 		IQN2_SRATE_3P84MHZ
#define 	CPRI_PACK_MODE		IQN2_LTE_CPRI_1b1
#elif LTE_RATE == 1
#define		LTESYMBOLSIZE		((IQN2_LTE1P4_FFT_SIZE+IQN2_LTE1P4_CYPRENORMAL1_SIZE)*4 + 16)
#define 	LTESYMBOL2SIZE		((IQN2_LTE1P4_FFT_SIZE+IQN2_LTE1P4_CYPRENORMAL_SIZE)*4 + 16)
#define 	SAMPLE_FREQ 		IQN2_SRATE_1P92MHZ
#define 	CPRI_PACK_MODE		IQN2_LTE_CPRI_0b1
#endif

//Value for Sin calculation
#define     Q_FACTOR           	 	8191
#if LTE_RATE == 20
#define     SAMP_FREQ_FLOAT     	30.72
#elif LTE_RATE == 10
#define     SAMP_FREQ_FLOAT     	15.36
#elif LTE_RATE == 5
#define     SAMP_FREQ_FLOAT     	7.68
#endif
#define     PI                  	3.14159265358979323846

#define		NBDESCMAX			128                                                         // Descriptor count rounded to the next power of 2
																							// 7 sym * 2 AxCs * 2 pingpong * 2 directions + 16 control packets

#define MONO_RX_FDQ            3000
#define MONO_RX_Q              2000
#define MONO_RX_FDQ_CTRL         2048
#define MONO_RX_Q_CTRL           1048


extern volatile unsigned int testcheck;

extern volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
extern volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
extern volatile uint32_t getCpuTimestamp[MAX_SLOT];
extern volatile uint32_t getRadt0Frames[MAX_SLOT];
extern volatile uint32_t getRadt1Frames[MAX_SLOT];
extern volatile uint32_t getBCNFrame[MAX_SLOT];
extern volatile uint32_t getRxPktCnt[MAX_SLOT];
extern volatile uint32_t symbolNum[MAX_SLOT][7];
extern unsigned char   saveDesc[2][14][32];
extern volatile uint32_t rxPktCnt[NBCHANNELPERLINK * 2];
extern volatile uint32_t symbolcount;
extern volatile uint32_t symbolNb;
extern volatile uint32_t slotcount;

extern void slotRecycling(PktDmaConfigHandle hPktDma);
extern void symbolRecycling(PktDmaConfigHandle hPktDma);
extern void slotPushing(PktDmaConfigHandle hPktDma);
extern void symbolPushing(PktDmaConfigHandle hPktDma);
extern void load_data(PktDmaConfigHandle hPktDma);
extern void lteFinalCheck(PktDmaConfigHandle hPktDma);
extern void pktDmaCtrlFinalCheck(PktDmaConfigHandle hPktDma);
extern void slotCount(Iqn2Fl_Handle   hIqn2Fl);
extern void symbolCount(Iqn2Fl_Handle   hIqn2Fl);
extern void packetCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable);
extern void symbolPacketCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable);



