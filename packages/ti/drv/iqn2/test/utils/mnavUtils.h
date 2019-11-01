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


#ifndef __MNAVUTILS_H
#define __MNAVUTILS_H

#include <ti/csl/csl.h>
#include <ti/csl/cslr_device.h>

#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2.h>

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

/**
 * @brief This structure contains the parameters for the
 * PKTDMA and QMSS setup.
 */
 typedef struct PktDmaConfigObj {
                    /** Holds AIF2 CPPI handle returned by the CPPI library   (set by the library) */
                    Cppi_Handle              hCppi;
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           txRegionAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 txNumDescAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 txDescSizeAxC[124];
	 	 	 	 	/** */
                    uint32_t                 txDesc2SizeAxC[124];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           rxRegionAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 rxNumDescAxC[124];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 rxDescSizeAxC[124];
                    /** Points to a table describing the Rx flow for each AxC in use  */
                    Cppi_RxFlowCfg*          hRxFlowAxC[124];
                    /** */
                    Qmss_QueueHnd            txFqAxC[124];
                    /** */
                    Qmss_QueueHnd            txQAxC[124];
                    /** */
                    Qmss_QueueHnd            rxFqAxC[124];
                    /** */
                    Qmss_QueueHnd            rxQAxC[124];
                    /** */
                    Cppi_ChHnd               txChAxC[124];
                    /** */
                    Cppi_ChHnd               rxChAxC[124];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           txRegionCtrl[16];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 txNumDescCtrl[16];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 txDescSizeCtrl[16];
	 	 	 	 	/** */
	 	 	 	 	Qmss_MemRegion           rxRegionCtrl[16];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 rxNumDescCtrl[16];
	 	 	 	 	/** */
	 	 	 	 	uint32_t                 rxDescSizeCtrl[16];
                    /** Points to a table describing the Rx flow for control channels - only one for now */
                    Cppi_RxFlowCfg*          hRxFlowCtrl[16];
                    /** */
                    Qmss_QueueHnd            txFqCtrl[16];
                    /** */
                    Qmss_QueueHnd            txQCtrl[16];
                    /** */
                    Qmss_QueueHnd            rxFqCtrl[16];
                    /** */
                    Qmss_QueueHnd            rxQCtrl[16];
                    /** */
                    Cppi_ChHnd               txChCtrl[16];
                    /** */
                    Cppi_ChHnd               rxChCtrl[16];
                    /** */
                    uint32_t*                rxDataBuff[16];
                    /** */
                    uint32_t*                txDataBuff[16];
					/** */
					Cppi_ChHnd               dioTxChAxC;
					/** */
					Cppi_ChHnd               dioRxChAxC;
					/** */
					uint32_t 				 firstAxC;
					/** */
					uint32_t 				 numAxC;
					/** */
					uint32_t 				 firstCtrl;
					/** */
					uint32_t 				 numCtrl;
					/** */
					uint32_t				 psMsgEnable;
					/** */
					Cppi_DescType			 ctrlDescType;
                    } PktDmaConfigObj, *PktDmaConfigHandle;

#ifndef __MNAVUTILS_C
extern
#endif
int32_t
UTILS_initQmss(
		uint32_t *region,
		uint32_t num_desc,
		uint32_t desc_size,
		uint32_t num_desc_tot
);

#ifndef __MNAVUTILS_C
extern
#endif
int32_t
UTILS_insertQmssRegion(
		uint32_t *region,
		Qmss_MemRegion mem_region,
		uint32_t num_desc,
		uint32_t desc_size,
		uint32_t start_index
);

#ifndef __MNAVUTILS_C
extern
#endif
int32_t
UTILS_initPktDma(
		PktDmaConfigHandle hPktDma
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_initCppiChannel(
	PktDmaConfigHandle hPktDma
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_resetCppi(
        PktDmaConfigHandle hPktDma
);

#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_resetQmss(
        PktDmaConfigHandle hPktDma
);


#ifndef __MNAVUTILS_C
extern
#endif
void
UTILS_resetRm(
);

#endif//__MNAVUTILS_H

/** @} */ // end of module additions

