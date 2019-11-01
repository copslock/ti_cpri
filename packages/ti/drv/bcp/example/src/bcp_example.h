/** 
 *   @file  bcp_example.h
 *
 *   @brief  
 *      Header file with data structures and definitions required for the BCP 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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

#ifndef _BCP_EXAMPLE_H_
#define _BCP_EXAMPLE_H_

/* XDC types include */
#include <xdc/std.h>

/* BCP driver include */
#include <ti/drv/bcp/bcp.h>
#include <ti/drv/bcp/bcp_osal.h>

/* Chip Level definitions include */
#include <ti/csl/csl_chip.h>
#include <ti/csl/csl_semAux.h>

/* PSC CSL definitions include */
#include <ti/csl/csl_psc.h>
#include <ti/csl/csl_pscAux.h>

/* QMSS CSL definitions include */
#include <ti/csl/csl_qm_queue.h>

/* BIOS include */
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>

/* IPC includes */ 
#include <ti/ipc/GateMP.h>
#include <ti/ipc/Ipc.h>
#include <ti/ipc/ListMP.h>
#include <ti/ipc/SharedRegion.h>
#include <ti/ipc/MultiProc.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h> 
#include <ti/sysbios/knl/Semaphore.h>

#ifndef DEVICE_K2L
/* SRIO driver Include */
#include <ti/drv/srio/srio_drv.h>
#endif /* DEVICE_K2L */

/* Standard C includes */
#include <stdio.h>
#include <string.h>
#include <math.h>

/** Number of host descriptors used by the BCP application */
#define     BCP_EXAMPLE_NUM_HOST_DESC              256

/** Host descriptor size. 
 *
 *  Big enough to hold the mandatory fields of the 
 *  host descriptor. PS data is always sent in SOP
 *  of Buffer for BCP.
 * 
 *  = 32 bytes of Host desc + PS in desc
 */
#define     BCP_EXAMPLE_SIZE_HOST_DESC             64 

#define     BCP_TEST_NUM_PACKETS                10


extern Void read_data_from_file 
(
    FILE*               fp, 
    UInt8*              pDataBuffer, 
    UInt32*             pDataBufferLen
);
extern Void write_data_to_file 
(
    UInt8*              pDataBuffer, 
    UInt32              dataBufferLen, 
    FILE*               fpOut
);
extern UInt8 gind_row_index (UInt32 input);
extern Int32 validate_rxdata 
(
    UInt8*              pRefDataBuffer,
    UInt32              refDataBufferLen,
    UInt8*              pRxDataBuffer, 
    UInt32              rxDataBufferLen, 
    UInt32              dataOffset
);
extern Int32 allocate_fdq 
(
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize,
    UInt8               bAllocHistQ,
    Qmss_QueueHnd*      phFDQ,
    Qmss_QueueHnd*      phHistQ
);
extern Void deallocate_fdq 
(
    Qmss_QueueHnd       hFDQ, 
    Qmss_QueueHnd       hGlblFDQ, 
    UInt32              numDesc, 
    UInt32              buffSize,
    Qmss_QueueHnd       hHistQ
);
extern Void dump_memory_usage (Void);
extern Void prepare_crchdr_cfg 
(
    Bcp_CrcHdrCfg*      pCrcHdrCfg, 
    Bcp_RadioStd        radioStd,
    UInt32              tbSize,
    UInt32              numFillerBits,
    UInt32              method2Id,
    UInt32              dtxFormat,
    UInt8               scrambFlag,
    UInt32              crcSize, 
    UInt32              numTb, 
    UInt32              dataFormatIn, 
    UInt32              numTrCh, 
    UInt32*             trChLen
);
extern UInt32 convert_coreLocalToGlobalAddr (UInt32 addr);
extern Void test_lte_dl (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_lte_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wcdma_dl (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wcdma_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wimax_dl (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wimax_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wcdma_rel99_dl (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);
extern Void test_wcdma_rel99_ul (Bcp_DrvHandle hBcp, Qmss_QueueHnd hGlblQ);

extern Void read_harq_data_from_file (FILE* fp, UInt8* pDataBuffer, UInt32* pDataBufferLen);
extern Int32 validate_harqoutput 
(
    UInt32*             pRefDataBuffer,
    UInt32              refDataBufferLen,
    UInt32*             pRxDataBuffer, 
    UInt32              rxDataBufferLen, 
    UInt32              dataOffset
);

#endif  /* _BCP_EXAMPLE_H_ */
