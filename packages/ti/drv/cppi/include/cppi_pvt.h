/*
 *   file  cppi_pvt.h
 *
 *   Private data structure of CPPI Low Level Driver.
 *
 *  ============================================================================
 *      (C) Copyright 2009-2015, Texas Instruments, Inc.
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
 *  \par
*/


#ifndef CPPI_PVT_H_
#define CPPI_PVT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* CPPI LLD includes */
#include <ti/drv/cppi/cppi_drv.h>        
#include <ti/drv/cppi/cppi_desc.h> 
#include <ti/drv/cppi/include/cppi_listlib.h>       
#include <ti/drv/cppi/include/cppi_heap.h>
#include <ti/csl/csl_cppi.h>

/* This macro generates compilier error if postulate is false, so 
 * allows 0 overhead compile time size check.  This "works" when
 * the expression contains sizeof() which otherwise doesn't work
 * with preprocessor */
#define CPPI_COMPILE_TIME_SIZE_CHECK(postulate)                         \
   do {                                                                 \
       typedef struct {                                                 \
         uint8_t NegativeSizeIfPostulateFalse[((int)(postulate))*2 - 1];\
       } PostulateCheck_t;                                              \
   }                                                                    \
   while (0)

#define CPPI_BLOCK_ALIGN_POW2    7 /* Align heap requests to 128 bytes */
#define CPPI_BLOCK_SIZE          1024 /* Request 1024 bytes from system in each malloc */

#define CPPI_MAX_CACHE_ALIGN     128 /* Maximum alignment for cache line size */

/* Channel Object */
typedef struct 
{
    /* List of channel objects */
    Cppi_ListNode       links;
    /* Channel number */
    uint8_t             channelNum;
    /* Number of times Channel is opened */
    uint8_t             refCnt;
    /* CPDMA opening the channel */
    Cppi_CpDma          dmaNum;
    /* Channel Type - Rx or Tx */
    Cppi_ChType         chType;
    /* Pointer back to the CPDMA Object that opened this channel */
    struct Cppi_DMAObj  *dmaObjHnd; 
}Cppi_ChObj;

/* Flow Object */
typedef struct 
{
    /* List of flow objects */
    Cppi_ListNode       links;
    /* Channel number */
    uint8_t             flowId;
    /* Number of times Channel is opened */
    uint8_t             refCnt;
    /* CPDMA opening the channel */
    Cppi_CpDma          dmaNum;
    /* Pointer back to the CPDMA Object that opened this channel */
    struct Cppi_DMAObj  *dmaObjHnd; 
}Cppi_FlowObj;

/* CPDMA Object */
typedef struct Cppi_DMAObj
{
    /* CPDMA this object belongs to */
    Cppi_CpDma              dmaNum;
    /* Reference count, the number of times CPDMA called cppi_init */
    uint8_t                 refCnt;
    /* Tx channel reference count, the number of times Tx channel was opened */
    uint8_t                 txChCnt;
    /* Rx channel reference count, the number of times Rx channel was opened */
    uint8_t                 rxChCnt;
    /* Rx flow reference count, the number of times Rx flow was configured */
    uint8_t                 rxFlowCnt;

    /* Depth of write arbitration FIFO */
    uint8_t                 writeFifoDepth;
    /* Minimum amount of time in clock cycles that an Rx channel will be required to wait when it 
     * encounters a buffer starvation */
    uint16_t                timeoutCount;
    /* Queue Manager 0 base address register */
    volatile uint32_t       qm0BaseAddress;
    /* Queue Manager 1 base address register */
    volatile uint32_t       qm1BaseAddress;
    /* Queue Manager 2 base address register */
    volatile uint32_t       qm2BaseAddress;
    /* Queue Manager 3 base address register */
    volatile uint32_t       qm3BaseAddress;

    /* Base address for the CPDMA overlay registers */

    /* Global Config registers */
    CSL_Cppidma_global_configRegs       *gblCfgRegs;
    /* Rx Channel Config registers */
    CSL_Cppidma_rx_channel_configRegs   *rxChRegs;
    /* Tx Channel Config registers */
    CSL_Cppidma_tx_channel_configRegs   *txChRegs;
    /* Rx Flow Config registers */
    CSL_Cppidma_rx_flow_configRegs      *rxFlowRegs;
    /* Tx Channel Scheduler registers */
    CSL_Cppidma_tx_scheduler_configRegs *txSchedRegs;

    /* Maximum supported Rx Channels */
    uint8_t                 maxRxCh;
    /* Maximum supported Tx Channels */
    uint8_t                 maxTxCh;
    /* Maximum supported Rx Flows */
    uint8_t                 maxRxFlow;
    /* Priority for all Rx transactions of this CPDMA */
    uint8_t                 rxPriority;
    /* Priority for all Tx transactions of this CPDMA */
    uint8_t                 txPriority;
    /* Allocated Rx channels */
    Uint32                  rxChMask[5];
    /* Allocated Tx channels */
    Uint32                  txChMask[5];
    /* Allocated Rx flows */
    Uint32                  rxFlowMask[5];
    /* Rx Channel Handles */
    Cppi_ChObj              *rxChHnd;
    /* Tx Channel Handles */
    Cppi_ChObj              *txChHnd;
    /* Rx Flow Handles */
    Cppi_FlowObj            *rxFlowHnd;
    /** RM DTS resource name for CPDMA rx channels */
    char                     rmCpdmaRxCh[CPPI_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for CPDMA tx channels */
    char                     rmCpdmaTxCh[CPPI_RM_RESOURCE_NAME_MAX_CHARS];
    /** RM DTS resource name for CPDMA rx flows */
    char                     rmCpdmaRxFlow[CPPI_RM_RESOURCE_NAME_MAX_CHARS];    
    /** RM DTS resource name for register writes in @ref Cppi_open */
    char                     rmCpdmaHwOpen[CPPI_RM_RESOURCE_NAME_MAX_CHARS];
}Cppi_DMAObj;


/* CPPI Object (unpadded) */
typedef struct 
{
    /* CPDMA handle */
    Cppi_DMAObj             dmaCfg[CPPI_MAX_CPDMA];
    Cppi_HeapDesc           heapDesc;
    /* Default QM base addresses */
    uint32_t                qm0BaseAddress;
    uint32_t                qm1BaseAddress;
    uint32_t                qm2BaseAddress;
    uint32_t                qm3BaseAddress;
}Cppi_Obj_Unpadded;

/* CPPI Object (padded) */
typedef struct
{
    /** Data structure without padding, so sizeof() can compute padding */
    Cppi_Obj_Unpadded       obj;
    /** Pad out to end of CPPI_MAX_CACHE_ALIGN bytes to prevent something else
     * from being placed on same cache line as Cppi_Obj.  Note that 
     * pad[0] is illegal, so must add full CPPI_MAX_CACHE_ALIGN if structure
     * is already padded by chance. */
    uint8_t                 pad[CPPI_MAX_CACHE_ALIGN - 
                            (sizeof(Cppi_Obj_Unpadded) % CPPI_MAX_CACHE_ALIGN)];
} Cppi_Obj;

/* CPPI Local Object */
typedef struct
{
    /* RM service handle */
    Cppi_RmServiceHnd cppiRmServiceHandle;
    /* Tracks whether CPDMA HW open was denied for each CPDMA */
    int8_t          hwOpenDenied[CPPI_MAX_CPDMA];
}Cppi_LocalObj;

/* wrapper to strncpy to avoid covery overrun warning */
static inline void cppi_strncpy (char *dst, const char *src, size_t len)
{
    strncpy (dst, src, len);
    dst[len - 1] = 0;
}

#ifdef __cplusplus
}
#endif

#endif /* CPPI_PVT_H_ */

