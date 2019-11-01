/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
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


#ifndef __CSLUTILS_H
#define __CSLUTILS_H

#include <ti/csl/csl.h>
#include <ti/csl/csl_pllc.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/csl/csl_edma3.h>
#include <ti/csl/csl_edma3Aux.h>
#ifdef _TMS320C6X
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>
#endif
#include <ti/csl/src/intc/csl_intc.h>
#include <ti/csl/csl_cpIntc.h>
#include <ti/csl/csl_gpio.h>
#include <ti/csl/cslr_device.h>
#include <ti/drv/aif2/aif2.h>

#define DATA_TYPE_DL    AIF2FL_LINK_DATA_TYPE_NORMAL
#define DATA_TYPE_UL    AIF2FL_LINK_DATA_TYPE_RSA
#define DATA_WIDTH_16   AIF2FL_DATA_WIDTH_16_BIT
#define DATA_WIDTH_15   AIF2FL_DATA_WIDTH_15_BIT
#define DATA_WIDTH_8    AIF2FL_DATA_WIDTH_8_BIT
#define DATA_WIDTH_7    AIF2FL_DATA_WIDTH_7_BIT

#define DSP_1           1
#define DSP_2           2

////////////////////////////////////////////
// define for local pktdma transfer ////////
////////////////////////////////////////////
#define SIZE_HOST_DESC               48
#define NUM_DESC		             7
#define SIZE_PS_DATA                 4
#define DATA_SIZE                    5120
#define CPPI_COMPLETION_QUE_NUM      1000
                                                      

typedef void (*UTILS_FxnPtr)();

extern uint8_t      	       DSP_procId;
extern CSL_CPINTCRegs     *CicRegs[3];
extern CSL_IntcRegs       *IntcRegs;
extern UTILS_FxnPtr        fsevt1_userIsr;
extern UTILS_FxnPtr        aif2evt6_userIsr;
extern UTILS_FxnPtr        aif2evt5_userIsr;
extern UTILS_FxnPtr        aif2evt4_userIsr;

/** Get core number */
#define UTILS_getCoreNum() ( DNUM & ((uint32_t)0x000000FF) )
// DEFINES WHETHER GLOBAL ADDRESSING IS REQUIRED FOR MASTER (EDMA, Peripherals, OTHER CorePacs) ACCESSES TO L2/L1
#define UTILS_GLOBAL_MEM_MASK    1
/** Get global address */
#define UTILS_local2GlobalAddr(dspAddr)     ( ((((uint32_t)dspAddr)&((uint32_t)0xFF000000))==(uint32_t)0x00000000) ?  \
                                                   (uint32_t*)((uint32_t)dspAddr|((UTILS_GLOBAL_MEM_MASK<<28)|((UTILS_getCoreNum()<<24) \
                                                   & 0x0F000000))) : (uint32_t*)dspAddr )
/** Get memory region number, LL2 or MSM to region0, DDR3 to region16 */
#define UTILS_getMemRegionNum(dspAddr)      ( ((((uint32_t)dspAddr)&((uint32_t)0xE0000000))==(uint32_t)0x00000000) ?  \
                                                   (Qmss_MemRegion)(Qmss_MemRegion_MEMORY_REGION0) : (Qmss_MemRegion)Qmss_MemRegion_MEMORY_REGION16 )


#ifndef __CSLUTILS_C
extern
#endif
void
AIF_enable(
);


#ifndef __CSLUTILS_C
extern
#endif
void
AIF_disable(
);

#ifndef __CSLUTILS_C
extern
#endif
void
AIF_pscDisableResetIso(
);


#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_GPIO8_setup(

);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_doCleanup(
    AIF_ConfigHandle    hAif,
    uint32_t timer
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_resetTimer(
		uint32_t timer
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_startTimer(
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_initTimer(
		uint32_t timer
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_triggerFsync(
	AIF_ConfigHandle hAif
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_triggerBoardsSync(
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_boardsSync(
		uint32_t timer,
		uint32_t timerpll
#ifdef EXT_CLOCK
		,uint32_t timer10Mhz
#endif
);

#ifndef __CSLUTILS_C
extern
#endif
int32_t
UTILS_configRtwp(
);

#ifndef __CSLUTILS_C
extern
#endif
int32_t
UTILS_readRtwpAnt01(
);

#ifndef __CSLUTILS_C
extern
#endif
void 
UTILS_configMasterSlave(
);


#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_printLinkConfig(
    AIF_ConfigHandle    hAif
);

#ifndef __CSLUTILS_C
extern
#endif 
void 
UTILS_aifIntcSetup(
);



#ifndef __CSLUTILS_C
extern
#endif 
void
UTILS_waitForHw(
   uint32_t delay
);

#ifndef __CSLUTILS_C
extern
#endif 
int32_t
UTILS_bytecmp(
    void *x1,
    void *x2,
    uint32_t byteCount
);

#ifndef __CSLUTILS_C
extern
#endif
uint32_t 
UTILS_isL2Asymmetric(
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_resetQmss(
);


#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_deinterleaveLteSuperPacket(
		uint32_t* superPktAxCPtr,
		uint32_t* dstPktPtr,
		uint32_t  numPackedSamples,
		uint32_t  numAxCs,
		uint32_t  payloadSize
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_openEdmaSampleCapt(
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_triggerEdmaSampleCapt(
		uint32_t  chan,
		uint32_t* dstAddr,
		uint32_t* srcAddr,
		uint32_t  numBytes
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_aif2ExceptIntDisable(
);

#ifndef __CSLUTILS_C
extern
#endif
void
UTILS_aif2IsOnLoss(
		AIF_ConfigHandle    hAif,
		uint32_t activeLink
);

#ifndef __CSLUTILS_C
extern
#endif
uint32_t aif2_mapDevice();

#ifndef __CSLUTILS_C
extern
#endif
uint32_t aif2_mmap(uint32_t addr, uint32_t size);

#ifndef __CSLUTILS_C
extern
#endif
uint32_t aif2_unmapDevice();

#endif//__CSLUTILS_H

/** @} */ // end of module additions

