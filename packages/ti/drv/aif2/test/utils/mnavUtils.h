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


#ifndef __MNAVUTILS_H
#define __MNAVUTILS_H

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl.h>
#include <ti/csl/cslr_device.h>
#include <ti/drv/aif2/aif2.h>


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



#ifndef __MNAVUTILS_C
extern
#endif
int32_t
UTILS_initQmss(
		uint32_t *region,
		uint32_t num_desc,
		uint32_t desc_size,
		uint32_t psmessage,
		uint32_t *regioncw
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_resetQmss(
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_resetRm(
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_hostLocalTransferInit(
		uint32_t *host_region,
		Qmss_MemRegion memRegion,
		uint32_t* txBuffer,
		uint32_t* rxBuffer,
		Qmss_QueueHnd* txFq,
		Qmss_QueueHnd* txQ,
		Qmss_QueueHnd* rxFq,
		Qmss_QueueHnd* rxQ,
		Qmss_QueueHnd* txCmplQueHnd,
		uint32_t nbLteCh
);

#ifndef __MNAVUTILS_C
extern
#endif
int32_t
UTILS_initPktDma(
		AIF_PktDmaConfigHandle hPktDma
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_initCppiChannel(
	AIF_CfgHandle hAifCfgObj,
	AIF_PktDmaConfigHandle hPktDma
);


#ifndef __MNAVUTILS_C
extern
#endif
void
Utils_populatePktDmaCfgObj(
		AIF_CoreCfgHandle      	ptrAifCoreCfg,
		AIF_PktDmaConfigHandle 	hPktDma,
		uint8_t*					region,
		uint32_t 					lteSymbolSize,
		uint32_t 					nbSymbol,
		uint32_t 					rxQ,
		uint32_t					rxFq,
		Cppi_RxFlowCfg			*rxFlow
);

#endif//__MNAVUTILS_H

/** @} */ // end of module additions

