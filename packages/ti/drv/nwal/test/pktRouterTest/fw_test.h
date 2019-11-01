/******************************************************************************
 * FILE PURPOSE:  Header file for unit test package
 ******************************************************************************
 * FILE NAME:   fw_test.h
 *
 * DESCRIPTION: Header file for unit test package
 *
 * REVISION HISTORY:
 *
 *  Copyright (c) Texas Instruments Incorporated 2010-2011
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

#ifndef _NWAL_TEST_H
#define _NWAL_TEST_H


#include <c6x.h>
#include <stdlib.h>
#include <stdio.h>

/* TBD: Need to be moved to shannon specific file. 
 * CSL Semaphore module includes 
 */
#include <ti/drv/qmss/qmss_qm.h>


#include <stdio.h>
#include <ti/drv/nwal/nwal.h>
#include <ti/drv/nwal/nwal_tune.h>
#include <ti/drv/nwal/nwal_util.h>

#ifndef __LINUX_USER_SPACE
#include <xdc/runtime/System.h>
#endif
#include "fw_rm.h"

#define CPU_NCORES CPU_NUM_REM_FAST_PATH_CORES
//#define NWAL_TEST_DESC_GLOB_MEM
//#define NWAL_TEST_DESC_BUF_GLOB_MEM


#define TEST_MAX_NUM_MAC                1
#define TEST_MAX_NUM_IP                 14
#define TEST_MAX_NUM_PORTS_PER_CORE     10
#define TEST_MAX_NUM_PORTS              ((TEST_MAX_NUM_PORTS_PER_CORE * CPU_NCORES)+ CPU_NUM_REM_FAST_PATH_CORES)

#ifdef NWAL_ENABLE_SA
#define TEST_MAX_NUM_IPSEC_CHANNELS     14
#define TEST_MAX_NUM_DM_SA_CHANNELS     2
#else
#define TEST_MAX_NUM_IPSEC_CHANNELS     0
#define TEST_MAX_NUM_DM_SA_CHANNELS     0
#endif
#define TEST_MAX_NUM_L2_L3_HDRS         10
#define TEST_MAX_NUM_TRANS              (TEST_MAX_NUM_MAC + TEST_MAX_NUM_IP + TEST_MAX_NUM_PORTS + TEST_MAX_NUM_IPSEC_CHANNELS)
#define TEST_CONFIG_MAX_DESC_NUM            16384   /* Maximum Number of descriptors configured */
#define TEST_CONFIG_MAX_PA_TO_SA_DESC       32
#define TEST_CONFIG_MAX_SA_TO_PA_DESC       32
#define TEST_CONFIG_MAX_GLOB_DESC           (TEST_CONFIG_MAX_PA_TO_SA_DESC + TEST_CONFIG_MAX_SA_TO_PA_DESC)
#define TEST_CONFIG_GLOB_DESC_SIZE          128   /* Cache Line Size not needed as this is not accessed at CGEM */
#define TEST_CONFIG_GLOB_BUF_SIZE           1536 /* MTU Size */

#define TEST_CONFIG_BASE_QM_GLOBAL_MSMC_MEMORY_REGION   Qmss_MemRegion_MEMORY_REGION0
#define TEST_CONFIG_BASE_QM_MEMORY_REGION               Qmss_MemRegion_MEMORY_REGION1
#define TEST_CONFIG_BASE_QM_GLOBAL_DDR_MEMORY_REGION    Qmss_MemRegion_MEMORY_REGION17
/*****************************************************************************
 * Local Reources per Core
 *****************************************************************************/
#define TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE        10
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
#define TEST_CONFIG_MAX_CTL_RX_NBUFS                (TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_CTL_RX_NBUFS                TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE
#endif

#define TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE        10
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
#define TEST_CONFIG_MAX_CTL_TX_NBUFS                 (TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_CTL_TX_NBUFS                 TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE
#endif
#define TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE        34
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
#define TEST_CONFIG_MAX_PKT_RX_NBUFS                 (TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_PKT_RX_NBUFS                 TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE
#endif

#define TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE        10
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
#define TEST_CONFIG_MAX_PKT_TX_NBUFS                 (TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_PKT_TX_NBUFS                 TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE
#endif

#define TEST_CONFIG_MAX_LOC_DESC_PER_CORE (TEST_CONFIG_MAX_CTL_RX_NBUFS_PER_CORE + \
                                           TEST_CONFIG_MAX_CTL_TX_NBUFS_PER_CORE + \
                                           TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE + \
                                           TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE) /* Would need to be  2^^ */
#ifdef NWAL_TEST_DESC_GLOB_MEM
#define TEST_CONFIG_MAX_LOC_DESC    (TEST_CONFIG_MAX_LOC_DESC_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_LOC_DESC    TEST_CONFIG_MAX_LOC_DESC_PER_CORE
#endif

#ifdef NWAL_TEST_DESC_GLOB_MEM
#define TEST_CONFIG_LOC_DESC_SIZE           128
#else
#define TEST_CONFIG_LOC_DESC_SIZE           128
#endif

#define TEST_CONFIG_MAX_CTL_RX_BUF_SIZE     384  /* Need to replace with a define from PA Aligned to next
                                                  * multiple of Cache line size
                                                  */
#define TEST_CONFIG_MAX_CTL_TX_BUF_SIZE     512

#define TEST_CONFIG_MAX_PKT_RX_BUF_SIZE     1536  /* MTU Size*/
#define TEST_CONFIG_MAX_PKT_TX_BUF_SIZE     TEST_CONFIG_MAX_PKT_RX_BUF_SIZE


#define TEST_CONFIG_DESC_USER_OFFSET            96
#define TEST_CONFIG_BUF_USER_RX_RES_BYTES       16
#define TEST_CONFIG_BUF_USER_RX_BUF_RES_BYTES   0xAB


#define TEST_MAX_BURST  16

/* For Fragmentation Test */
#define TEST_MAX_MTU_SIZE   82
/******************************************************************************
 * DATA DEFINITION: testNwLocContext structure. Resource used per core
 *****************************************************************************/
typedef struct {
    uint16_t               state;
#define TEST_NW_CXT_LOC_INACTIVE     0x0
#define TEST_NW_CXT_LOC_ACTIVE       0x2
    Qmss_QueueHnd          descQ;    /* Queue holding unassigned descriptors */

    Qmss_QueueHnd          pktQHandle;    /* Queue holding unassigned descriptors */
    uint16_t               txErrDrop;
    uint16_t               numPktSent;
    uint16_t               numUnhandledEthType;
    uint16_t               numUnhandledL4Type;
    uint16_t               invL3Offset;
    uint16_t               numL2PktsRecvd;
    uint16_t               numL3PktsRecvd;
    uint16_t               numL4PktsRecvd;
    nwalLocCfg_t           nwalLocCfg;
    nwalTxPSCmdInfo_t      txPsCmdInfo;
}testNwLocContext_t;
extern testNwLocContext_t   testNwLocContext;



/******************************************************************************
 * DATA DEFINITION: testNwGlobContext structure
 *****************************************************************************/
typedef struct {
    uint16_t               state;
#define TEST_NW_CXT_GLOB_INACTIVE               0x0
#define TEST_NW_CXT_GLOB_ACTIVE                 0x1
#define TEST_NW_CXT_GLOB_RES_ALLOC_COMPLETE     0x3
    nwal_Handle             nwalInstHandle;
    Qmss_QueueHnd           descQ;    /* Queue holding unassigned descriptors */
    Qmss_QueueHnd           descExtMemQ;
    paSysStats_t            paStats;
    uint16_t                numCoresStarted;
}testNwGlobContext_t;

#ifdef __LINUX_USER_SPACE
#define System_printf   printf
#endif

/*****************************************************************************
 * FUNCTION PURPOSE: NWAL Print Flush
 *****************************************************************************
 * DESCRIPTION: Wrapper for System_Flush()
 *****************************************************************************/
static inline void nwal_SystemFlush ()
{
#ifndef __LINUX_USER_SPACE
    System_flush();
#endif
}
extern cregister volatile unsigned int TSCL;
static inline unsigned long fw_read_clock(void)
{
        return TSCL;
}

extern cregister volatile unsigned int TSCL;

extern testNwGlobContext_t   testNwGlobContext;
nwal_Bool_t testNwInit (void);
nwal_Bool_t testNwSwUpdateMacAddr(uint8_t macAddress[6]);

void Osal_invalidateCache (void *blockPtr, uint32_t size);
void Osal_writeBackCache (void *blockPtr, uint32_t size);
void testNWALRxPktCallback     (uint32_t            appCookie,
                                uint16_t            numPkts,
                                nwalRxPktInfo_t*    pPktInfo,
                                uint64_t            timestamp,
                                nwal_Bool_t*        pFreePkt);
void testNWALCmdPaStatsReply (nwal_AppId        appHandle,
                              nwal_TransID_t    trans_id,
                              paSysStats_t      *stats);
void testNwCPSWInit(nwal_Bool_t enableALE);
void Osal_invalidateCache (void *blockPtr, uint32_t size);
void Osal_writeBackCache (void *blockPtr, uint32_t size);
extern void  Osal_nwalCsEnter (uint32_t *key);
extern void  Osal_nwalCsExit (uint32_t key);
#endif



