/*
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
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



#ifndef _TCP3D_SAMPLE_H_
#define _TCP3D_SAMPLE_H_

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/IHeap.h>

#include <ti/sdo/edma3/drv/edma3_drv.h>

#include <ti/drv/tcp3d/tcp3d_drv.h>

/* CSL includes */
#include <ti/csl/soc.h>
#include <ti/csl/cslr_tcp3d_cfg.h>
#include <ti/csl/cslr_tcp3d_dma.h>
#include <ti/csl/cslr_tcp3d_dma_offsets.h>

#include "sample.h"

#if (CSL_TCP3D_PER_CNT > 1)
#include "tcp3d_multi_inst.h"
#else
#include "tcp3d_single_inst.h"
#endif

#if EDMA_LOCAL_COMP_ISR
extern tccCallbackParams edma3IntrParamsLoc[];
extern unsigned int allocatedTCCsLoc[];
#endif

#define EDMA_RESULT_CHECK(res)      ((res == EDMA3_DRV_SOK) ? "Passed": "Failed")

/*
 * EDMA Resource structure
 */
typedef struct EDMA_RES
{
    UInt32                  chNo;
    UInt32                  tccNo;
    EDMA3_RM_TccCallback    cbFunc;
    Void                    *cbData;
} EDMA_RES;

/*
 * EDMA configuration structure
 */
typedef struct EDMA_CONFIG
{
    EDMA_RES            pingChRes[TCP3D_DRV_MAX_CH_PER_PATH];
    EDMA_RES            pongChRes[TCP3D_DRV_MAX_CH_PER_PATH];
    EDMA_RES            linkChRes[TCP3D_DRV_MAX_LINK_CH];
} EDMA_CONFIG;

/**
 * \brief   TCP3D Initialization
 *
 * This function initializes the TCP3D Driver for the given TCP3D instance ID.
 * It internally calls Tcp3d_getNumBuf(), Tcp3d_getBufDesc() and Tcp3d_init(),
 * in that order and memory is allocated from the heap pointer given.
 */
Tcp3d_Instance* tcp3dSampleInit(
                    IHeap_Handle        dataHeap,
                    UInt8               instNum,
                    UInt32              testMaxBlocks,
                    UInt32              testMode,
                    UInt32              testDoubleBuffer,
                    UInt32              testLteCrcSel,
                    UInt32              dspCoreID,
                    EDMA3_DRV_Handle    hEdma,
                    UInt32              tpccRegionUsed,
                    EDMA_CONFIG         *edmaConfig,
                    Tcp3d_Result        *errCode);

/**
 * \brief   TCP3D De-initialization
 *
 * This function de-initializes the TCP3D Driver for the given TCP3D instance ID.
 * Frees any memory allocated from the heap during the initialization.
 * 
 * Currently, there are no driver function calls.
 */
Tcp3d_Result tcp3dSampleDeinit(
                    IHeap_Handle    dataHeap,
                    UInt8           instNum,
                    Tcp3d_Instance  *tcp3dInst);

/**
 * \brief   Open EDMA channels for TCP3D driver
 *
 * Function for opening EDMA3 channels using the EDMA3 LLD APIs based on the
 * TCP3D instance ID given and the details are updated in the edmaConfig structure.
 */
Void openEdmaChannels ( EDMA3_DRV_Handle    hEdma,
                        UInt8               instNum,
                        EDMA_CONFIG         *edmaConfig);

/**
 * \brief   Close EDMA channels for TCP3D driver
 *
 * This function initializes the TCP3D Driver for the given TCP3D instance ID.
 * It internally calls Tcp3d_getNumBuf(), Tcp3d_getBufDesc() and Tcp3d_init(),
 * in that order and memory is allocated from the heap pointer given.
 */
Void closeEdmaChannels (EDMA3_DRV_Handle    hEdma,
                        UInt8               instNum,
                        EDMA_CONFIG         *edmaConfig);

#if EDMA_LOCAL_COMP_ISR // flag defined in sample.h file
/**
 * Fill the tables for allocated TCC & tccCB params, used with local
 * EDMA3 call back ISR routine (see in sample_int_reg.c file).
 */
Void updateAllocatedTccsLoc(  EDMA_CONFIG         *edmaConfig);
#endif

#endif  /* _TCP3D_SAMPLE_H_ */

