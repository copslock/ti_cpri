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
#include "aidCfg.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif

#define SYMBOLSIZE20        ((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL1_SIZE)*4 + 16)
#define SYMBOL2SIZE20       ((IQN2_LTE20_FFT_SIZE+IQN2_LTE20_CYPRENORMAL_SIZE)*4 + 16)
#define SYMBOLSIZE10        ((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL1_SIZE)*4 + 16)
#define SYMBOL2SIZE10       ((IQN2_LTE10_FFT_SIZE+IQN2_LTE10_CYPRENORMAL_SIZE)*4 + 16)
#define SYMBOLSIZE5         ((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL1_SIZE)*4 + 16)
#define SYMBOL2SIZE5        ((IQN2_LTE5_FFT_SIZE+IQN2_LTE5_CYPRENORMAL_SIZE)*4 + 16)

#define 	MAX_SLOT				400
#define		MAX_DEBUG_SLOT		    400			             // For debug, size of monitoring arrays

#define 	SLOT_NUM_FIRST_PUSH	    100  			// The first push needs to happen on the last slot number before frame boundary
#if DFE_BB_LOOPBACK == 1
#define 	SLOT_NUM_DATA_CHECK	    7502 			// If testInfo.never_end == 0, test stops after this amount of Lte slots
#else
#define 	SLOT_NUM_DATA_CHECK	    7502			// If testInfo.never_end == 0, test stops after this amount of Lte slots
#endif

#define     NBCHANNEL               NUM_AXCS_LTE20_AID + NUM_AXCS_LTE10_AID + NUM_AXCS_LTE5_AID

#define     NBSYMBOL			    IQN2_LTE_SYMBOL_NUM											// Number of symbols per slot
#define 	NUMDESCCTL			    7
#define		CTLDESCSIZE			    720

#define		NBDESCMAX			(128)                   // 140 sym * 16 AxCs * 2 directions + 16 control packets
#if LTE_RATE == 20
#define     NBDESCMAXRX         (64)                    // 14 sym * 16 AxCs + 16 control packets
#else
#define     NBDESCMAXRX         (64)                    // 14 sym * 16 AxCs + 16 control packets
#endif
#define MONO_RX_FDQ            3000
#define MONO_RX_Q              2000
#define MONO_RX_FDQ_CTRL       2048
#define MONO_RX_Q_CTRL         1048

#define 	RX_SUCCESS_CNT		32000                    // Define a number of consecutive runtime RF check success prior to monitoring runtime failures


#define     FREQ                1.0
#define     N   (2048)                                            // Used for twiddle factor generation
#define     PAD (16)                                              // Used for twiddle factor generation

//////////////////////////////////////////////////////////////////////////////
// Typedefs
typedef struct {
	    		int32_t	magic;
	    		int32_t revision;
	const 		int32_t type;
	const		int32_t	loopback;
	volatile 	int32_t	test_case;
	volatile	int32_t	capture_requested;
	volatile	int32_t	never_end;
	volatile 	int32_t	use_cio;
				int32_t	error_condition;
	volatile	int32_t	force_stop;
	volatile    int32_t alive_test;
				} test_info_t;

typedef struct {
				uint8_t	firstsymbol[SYMBOLSIZE20  - 16];
				uint8_t	othersymbol[6][SYMBOL2SIZE20 - 16];
	} lte_slot_t;
	
	
extern volatile unsigned int testcheck;

extern volatile uint32_t aif2SymbolEgressCount[MAX_SLOT];
extern volatile uint32_t aif2SymbolIngressCount[MAX_SLOT];
extern volatile uint32_t getCpuTimestamp[MAX_SLOT];
extern volatile uint32_t getRadt0Frames[MAX_SLOT];
extern volatile uint32_t getRadt1Frames[MAX_SLOT];
extern volatile uint32_t getBCNFrame[MAX_SLOT];
extern volatile uint32_t getRxPktCnt[MAX_SLOT];
extern volatile uint32_t symbolNum[MAX_SLOT][7];
extern unsigned char   saveDesc[NBCHANNEL][14][32];
extern volatile uint32_t rxPktCnt[8 * 2];
extern volatile uint32_t symbolcount;
extern volatile uint32_t symbolNb;
extern volatile uint32_t slotcount;
extern test_info_t  testInfo;

extern void slotRecycling(PktDmaConfigHandle hPktDma);
extern void slotPushing(PktDmaConfigHandle hPktDma);
extern void load_data(PktDmaConfigHandle hPktDma);
extern void lteFinalCheck(PktDmaConfigHandle hPktDma);
extern void pktDmaCtrlFinalCheck(PktDmaConfigHandle hPktDma);
extern void slotCount(Iqn2Fl_Handle   hIqn2Fl);
extern void packetCount(Iqn2Fl_Handle   hIqn2Fl, uint32_t aidEnable);
extern void UTILS_genSingleDualTone(PktDmaConfigHandle hPktDma);
extern void UTILS_genDummyInitData(PktDmaConfigHandle hPktDma);

