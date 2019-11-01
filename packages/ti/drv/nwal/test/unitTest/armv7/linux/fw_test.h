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

#include <stdio.h>
#include <ti/drv/nwal/nwal.h>
#include <ti/drv/nwal/nwal_tune.h>
#include <ti/drv/nwal/nwal_util.h>
#include <ti/runtime/hplib/hplib.h>

#ifndef __LINUX_USER_SPACE
#include <xdc/runtime/System.h>
#endif
#include "fw_rm.h"


#define CPU_NCORES      4

/* Enable User Mode CACHE Support. Support would require using
 * additional Kernel Module
 * #define NWAL_ENABLE_USER_MODE_CACHE 1
 */
#define NWAL_ENABLE_USER_MODE_CACHE 1

#ifdef NWAL_ENABLE_USER_MODE_CACHE
#define NWAL_ENABLE_PROFILE 1
#endif

/* Define to enable descriptors and buffer memory being located in global MSMC/DDR memory.Linker command file need
 * to reflect the memory placement
 */
#define NWAL_TEST_DESC_GLOB_MEM
#define NWAL_TEST_DESC_BUF_GLOB_MEM

#ifndef NWAL_ENABLE_USER_MODE_CACHE
#define TUNE_NWAL_QM_START_BASE_INDEX  0  //ok to use 0 since kernel is somewhere higher
#define TUNE_NWAL_QM_GLOBAL_REGION 0  //ok to to use 0 since msmc addr < ddr addr
#else
/* DDR */
#define TUNE_NWAL_QM_START_BASE_INDEX  0x2000  //WARNING:  must reflect what kernel is using for their region, see device tree blob
#define TUNE_NWAL_QM_GLOBAL_REGION 18  //WARNING: must reflect what kernel is using for their region, see device tree blob
#endif


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


/*****************************************************************************
 * Global Common Reources per Chip
 *****************************************************************************/
#define TEST_CONFIG_MAX_DESC_NUM            16384   /* Maximum Number of descriptors configured */
#define TEST_CONFIG_MAX_PA_TO_SA_DESC       32
#define TEST_CONFIG_MAX_SA_TO_PA_DESC       32
#define TEST_CONFIG_MAX_GLOB_DESC           (TEST_CONFIG_MAX_PA_TO_SA_DESC + TEST_CONFIG_MAX_SA_TO_PA_DESC)
#define TEST_CONFIG_GLOB_DESC_SIZE          128   /* Cache Line Size not needed as this is not accessed at CGEM */
#define TEST_CONFIG_GLOB_BUF_SIZE           1600 /* MTU Size */

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
#define TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE        64
#ifdef NWAL_TEST_DESC_BUF_GLOB_MEM
#define TEST_CONFIG_MAX_PKT_RX_NBUFS                 (TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE * CPU_NCORES)
#else
#define TEST_CONFIG_MAX_PKT_RX_NBUFS                 TEST_CONFIG_MAX_PKT_RX_NBUFS_PER_CORE
#endif

#define TEST_CONFIG_MAX_PKT_TX_NBUFS_PER_CORE        28
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

#define TEST_CONFIG_MAX_CTL_RX_BUF_SIZE     520  /* Need to replace with a define from PA Aligned to next
                                                  * multiple of Cache line size
                                                  */
#define TEST_CONFIG_MAX_CTL_TX_BUF_SIZE     TEST_CONFIG_MAX_CTL_RX_BUF_SIZE

#define TEST_CONFIG_MAX_PKT_RX_BUF_SIZE     1536  /* MTU Size*/
#define TEST_CONFIG_MAX_PKT_TX_BUF_SIZE     TEST_CONFIG_MAX_PKT_RX_BUF_SIZE

#define TEST_MAX_BURST  32

/* For Fragmentation Test */
#define TEST_MAX_MTU_SIZE   82

/******************************************************************************
 * DATA DEFINITION: Remote connection configuration at Core 0
 *****************************************************************************/
typedef struct {
    uint16_t                state;  /* Same Transation ID state being used for remote connection */
    nwal_Handle             handle;
    uint16_t                udpPort;
}testNwalRemoteConn_t;

/******************************************************************************
 * DATA DEFINITION: NWAL Transaction Info structure being maintained in unit test App
 *****************************************************************************/
#define TEST_NWAL_HANDLE_TRANS_NONE             0
#define TEST_NWAL_HANDLE_TRANS_MAC              1
#define TEST_NWAL_HANDLE_TRANS_IP               2
#define TEST_NWAL_HANDLE_TRANS_PORT             3
#define TEST_NWAL_HANDLE_TRANS_SEC_ASSOC        4
#define TEST_NWAL_HANDLE_TRANS_SEC_POLICY       5


#define TEST_NWAL_HANDLE_STATE_IDLE             0
#define TEST_NWAL_HANDLE_STATE_OPEN_PENDING     1
#define TEST_NWAL_HANDLE_STATE_OPEN             2
#define TEST_NWAL_HANDLE_STATE_CLOSE_PENDING    3
typedef struct {
    nwal_Bool_t             inUse;
    uint16_t                transType;
    uint16_t                state;
    nwal_Handle             handle;
    uint64_t                transId;
    testNwalRemoteConn_t*   pRemoteConn;
}testNwalTransInfo_t;

/******************************************************************************
 * DATA DEFINITION: testNwLocContext structure. Resource used per core
 *****************************************************************************/
typedef struct {
    uint16_t               state;
#define TEST_NW_CXT_LOC_INACTIVE     0x0
#define TEST_NW_CXT_LOC_ACTIVE       0x2

    Qmss_QueueHnd          descQ;    /* Queue holding unassigned descriptors */
    uint16_t               numPendingCfg;
    uint16_t               numCmdPass;
    uint16_t               numCmdFail;
    uint16_t               numL2PktsSent;
    uint16_t               numL2PktsRecvd;
    uint16_t               numL3PktsSent;
    uint16_t               numL3PktsRecvd;
    uint16_t               numL4PktsRecvd;
    uint16_t               numL4PktsSent;
    uint16_t               numFragPktsSent;
    uint16_t               numFragPktsRecvd;
    uint16_t               TxErrDrop;
    nwalLocCfg_t           nwalLocCfg;
    testNwalTransInfo_t    transInfos[TEST_MAX_NUM_TRANS];
}testNwLocContext_t;
extern __thread  testNwLocContext_t   testNwLocContext;



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
    testNwalRemoteConn_t    remUDPConn[CPU_NCORES-1];  /* Remote connection configuration
                                                        * details done at core -0
                                                        */
#ifdef NWAL_ENABLE_SA
    testNwalRemoteConn_t    remIPSecUDPConn[CPU_NCORES-1];  /* Remote connection configuration
                                                     * details done at core -0
                                                     */
#endif
    uint16_t                numDMEncReqSent;
    uint16_t                numDMEncRespRecvd;
    uint16_t                numDMDecReqSent;
    uint16_t                numDMDecRespRecvd;
    uint16_t                numCoresStarted;
    uint16_t                numCoresTested;
    Qmss_QueueHnd       qosQ[5];

}testNwGlobContext_t;


#define ALIGN(x)    __attribute__((aligned (x)))

#define CACHE_LINESZ    NWAL_CACHE_LINE_SIZE
#define TEST_CONFIG_BUFSIZE_NWAL_HANDLE     3400
#define TEST_CONFIG_BUFSIZE_NWAL_PER_MAC                    384
#define TEST_CONFIG_BUFSIZE_NWAL_IPSEC_HANDLE_PER_CHAN      384
#define TEST_CONFIG_BUFSIZE_NWAL_DM_HANDLE_PER_CHAN         256
#define TEST_CONFIG_BUFSIZE_NWAL_PER_IP                     384
#define TEST_CONFIG_BUFSIZE_NWAL_PER_PORT                   256
#define TEST_CONFIG_BUFSIZE_NWAL_PER_L2L3_HDR               128
#define TEST_CONFIG_BUFSIZE_NWAL_PER_LOC_CONTEXT            384
#define NWAL_CHAN_HANDLE_SIZE    ((TEST_CONFIG_BUFSIZE_NWAL_PER_MAC * TEST_MAX_NUM_MAC) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_IPSEC_HANDLE_PER_CHAN * TEST_MAX_NUM_IPSEC_CHANNELS) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_DM_HANDLE_PER_CHAN * TEST_MAX_NUM_DM_SA_CHANNELS) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_IP * TEST_MAX_NUM_IP) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_PORT * TEST_MAX_NUM_PORTS)+ \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_LOC_CONTEXT * CPU_NCORES) + \
                                  (TEST_CONFIG_BUFSIZE_NWAL_PER_L2L3_HDR * TEST_MAX_NUM_L2_L3_HDRS))

/******************************************************************************
 * DATA DEFINITION: testNwGlobContextShmMem structure
 *****************************************************************************/
typedef struct
{
    uint8_t nwalInstMem[TEST_CONFIG_BUFSIZE_NWAL_HANDLE]ALIGN(CACHE_LINESZ);
    uint8_t nwalHandleMem[NWAL_CHAN_HANDLE_SIZE]ALIGN(CACHE_LINESZ);
} testNwGlobContextShmMem_t;


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

static inline unsigned long fw_read_clock(void)
{
#ifdef NWAL_ENABLE_PROFILE
    return (hplib_mUtilGetPmuCCNT());
#endif
        return 0;
}


extern testNwGlobContext_t   testNwGlobContext;
void testNWALRxPktCallback     (uint32_t            appCookie,
                                uint16_t            numPkts,
                                nwalRxPktInfo_t*    pPktInfo,
                                uint64_t            timestamp,
                                nwal_Bool_t*        pFreePkt);

void testNWALCmdCallBack (nwal_AppId        appHandle,
                          uint16_t            trans_id,
                          nwal_RetValue     ret);

void testNWALCmdPaStatsReply (nwal_AppId        appHandle,
                              nwal_TransID_t    trans_id,
                              paSysStats_t      *stats);
void testNWALRxDmBack( uint32_t                appCookie,
                       uint16_t                numPkts,
                       nwalDmRxPayloadInfo_t*  pDmRxPayloadInfo,
                       nwal_Bool_t*            pFreePkt);
nwal_Bool_t testNwInit (void);
nwal_Bool_t testNwSwUpdateMacAddr(uint8_t macAddress[6]);

void Osal_invalidateCache (void *blockPtr, uint32_t size);
void Osal_writeBackCache (void *blockPtr, uint32_t size);
unsigned int Osal_cache_op_measure(unsigned long long * p_n);
void Osal_cache_op_measure_reset(void);
extern void  Osal_nwalCsEnter (uint32_t *key);
extern void  Osal_nwalCsExit (uint32_t key);
#endif



