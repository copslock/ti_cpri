/**  
 * @file multicore_example.h
 *
 * @brief 
 *  Holds all the constants and API definitions required by the example
 *  application to run.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
#ifndef _MULTICORE_EXAMPLE_H_
#define _MULTICORE_EXAMPLE_H_

/* C Standard library Include */
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef __LINUX_USER_SPACE
/* XDC types include */
#include <xdc/std.h>

/* IPC include */
#include <ti/ipc/Ipc.h>
#include <ti/ipc/SharedRegion.h>

/* XDC/BIOS includes */
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <xdc/cfg/global.h>

/* Cpsw Manangement */
#include <cpsw_mgmt.h>

/* Chip Level definitions include */
#include <ti/csl/csl_chip.h>

/* semaphor module */
#include <ti/csl/csl_semAux.h>

/* CSL XMC module includes */
#include <ti/csl/csl_xmcAux.h>

/* CSL Cache module includes */
#include <ti/csl/csl_cacheAux.h>
#endif /* __LINUX_USER_SPACE */

/* CSL queue definitions */
#include <ti/csl/csl_qm_queue.h>


/* PA LLD include */
#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pasahost.h>
#include <ti/drv/pa/nss_if.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

/* 
 * Shut off: remark #880-D: parameter "descType" was never referenced
*
* This is better than removing the argument since removal would break
* backwards compatibility
*/
#ifdef _TMS320C6X
#elif defined(__GNUC__)
/* Same for GCC:
* warning: unused parameter descType [-Wunused-parameter]
*/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif


/** Number of cores in the test */
#define         pa_MC_EXAMPLE_NUM_CORES      4

/** Define RM use or not */
#ifdef SIMULATOR_SUPPORT
#define         RM             0 /* 1: Use, 0: No RM */
#else
#define         RM             1 /* 1: Use, 0: No RM */
#endif
/* Define the System init core/process ID */
#define         SYSINIT        0

/** Number of host descriptors used by the example program 
 * The number of descriptors needs to be in 2^X.
 * Therefore, this will be constant and not a function
 * of the number of cores in the test */  
#define         NUM_HOST_DESC               256

/** Host descriptor size. 
 *
 *  Big enough to hold the mandatory fields of the 
 *  host descriptor and PA Control Data
 * 
 *  = 32 bytes for Host desc + PA Control data
 */
#define         SIZE_HOST_DESC              128


#define CACHE_LINESZ    128
#define SYS_ROUND_UP(x,y)   (((x) + ((y) -1))/(y)*(y))

/** Number of PA Tx queues available */
#define         NUM_PA_TX_QUEUES            NSS_NUM_TX_QUEUES

/** Number of PA Tx channels available */
#define         NUM_PA_TX_CHANNELS          NSS_NUM_TX_PKTDMA_CHANNELS

/** Number of PA Rx channels available */
#define         NUM_PA_RX_CHANNELS          NSS_NUM_RX_PKTDMA_CHANNELS

#define TF_PA_Q_INPUT                       NSS_PA_QUEUE_INPUT_INDEX
#define TF_PA_Q_OUTER_IP                    NSS_PA_QUEUE_OUTER_IP_INDEX
#define TF_PA_Q_INNER_IP                    NSS_PA_QUEUE_INNER_IP_INDEX
#define TF_PA_Q_LUT2                        NSS_PA_QUEUE_LUT2_INDEX
#define TF_PA_Q_POST                        NSS_PA_QUEUE_POST_INDEX
#define TF_PA_Q_TXCMD                       NSS_PA_QUEUE_TXCMD_INDEX
#define TF_PA_Q_EMAC                        NSS_CPSW_QUEUE_ETH_INDEX

/**********************************************************************
 ****************************** Defines *******************************
 **********************************************************************/

/* Hardware Semaphore to synchronize access from
 * multiple applications (PA applications and non-PASS applications)
 * across different cores to the QMSS library.
 */
#define     QMSS_HW_SEM         3 

/* Hardware Semaphore to synchronize access from
 * multiple applications (PASS applications and non-PASS applications)
 * across different cores to the CPPI library.
 */
#define     CPPI_HW_SEM         4 

/* Hardware Semaphore to synchronize access from
 * multiple applications (PASS applications and non-PASS applications)
 * across different cores to the PA library.
 */
#define     PA_HW_SEM           5 

#ifndef __LINUX_USER_SPACE
/* Hardware Semaphore to synchronize access from
 * multiple applications (PASS applications and non-PASS applications)
 * across different cores in this application.
 */
#define PA_APP_HW_SEM_SYS       6

#undef L2_CACHE
#ifdef L2_CACHE
    /* Invalidate L2 cache. This should invalidate L1D as well. 
     * Wait until operation is complete. */    
#define SYS_CACHE_INV(addr, size, code)    CACHE_invL2 (addr, size, code)

    /* Writeback L2 cache. This should Writeback L1D as well. 
     * Wait until operation is complete. */ 
#define SYS_CACHE_WB(addr, size, code)     CACHE_wbL2 (addr, size, code)

#else       
    /* Invalidate L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
#define SYS_CACHE_INV(addr, size, code)    CACHE_invL1d (addr, size, code)
    /* Writeback L1D cache and wait until operation is complete. 
     * Use this approach if L2 cache is not enabled */    
#define SYS_CACHE_WB(addr, size, code)     CACHE_wbL1d (addr, size, code)

#endif

#endif

/* Multicore results */
#define TEST_NOT_COMPLETED 0
#define TEST_PASSED        1
#define TEST_FAILED        2
extern  char    test_stat[50];
extern int no_bootMode;

/* Number of PA internal buffers to allocate */
#define     PA_NUM_BUFFERS              3

/* PA definitions */
#define     MAX_NUM_L2_HANDLES          10
#define     MAX_NUM_L3_HANDLES          20
#define     MAX_NUM_L4_HANDLES          40

#define     BUFSIZE_PA_INST             512
#define     BUFSIZE_L2_TABLE            1000
#define     BUFSIZE_L3_TABLE            4000
#define     BUFSIZE_USR_STATS_TABLE     1024

typedef struct testRes {
	uint32_t    rxStatus;
	uint32_t    pad[CACHE_LINESZ - sizeof (uint32_t)];
}testRes_t;

/* Example Multicore Shared Object Structure */
typedef struct pa_Example_MC_ShObj {
	/* TX queue with free decriptors attached to data buffers for transmission.*/
	Qmss_QueueHnd                           gTxFreeQHnd;
	uint8_t                                rsvd1[CACHE_LINESZ - sizeof (Qmss_QueueHnd)];

	/* RX queue with free decriptors attached to data buffers to be used
	   by the PASS CPDMA to hold the received data.*/
	Qmss_QueueHnd                           gRxFreeQHnd;
	uint8_t                                rsvd2[CACHE_LINESZ - sizeof (Qmss_QueueHnd)];

	/* CPPI Handles used by the application */
	Cppi_Handle                             gCpdmaHnd;
	uint8_t                                rsvd3[CACHE_LINESZ - sizeof (Cppi_Handle)];

	/* PA Driver Handle */
	Pa_Handle                               gPAInstHnd;
	uint8_t                                rsvd4[CACHE_LINESZ - sizeof (Pa_Handle)];

	/* PA L2 Handle */
	paHandleL2L3_t                          gPaL2Handles;
	uint8_t                                rsvd5[CACHE_LINESZ - sizeof (paHandleL2L3_t)];

	/* PA L3 Handle */
	paHandleL2L3_t                          gPaL3Handles;
	uint8_t                                rsvd6[CACHE_LINESZ - sizeof (paHandleL2L3_t)];

	/* multicore sync up variables */
	uint32_t                                globalCfgDone;
	uint8_t                                 rsvd7[CACHE_LINESZ - sizeof (uint32_t)];

	uint32_t                                localCfgDone;
	uint8_t                                 rsvd8[CACHE_LINESZ - sizeof (uint32_t)];
    
	uint32_t                                localTestDone;	
	uint8_t                                 rsvd9[CACHE_LINESZ - sizeof (uint32_t)];

	/* multicore results, note that testRes_t is already cache line sized */
	uint32_t                                allPktRxCnt;	
	uint8_t                                 rsvd10[CACHE_LINESZ - sizeof (uint32_t)];

	/* PA Buffers needed for the Multicore Example, only Master Core needs this, others access via Handles */
	uint8_t                                 gPAInst[SYS_ROUND_UP(BUFSIZE_PA_INST, CACHE_LINESZ)];
	uint8_t                                 gMemL2Ram[SYS_ROUND_UP(BUFSIZE_L2_TABLE, CACHE_LINESZ)];
	uint8_t                                 gMemL3Ram[SYS_ROUND_UP(BUFSIZE_L3_TABLE, CACHE_LINESZ)];
	uint8_t                                 gMemUsrStats[SYS_ROUND_UP(BUFSIZE_USR_STATS_TABLE, CACHE_LINESZ)];

}pa_Example_MC_ShObj_t;

/* Pointer to the shared memory to get to all the handles needed for
 * multicore/multiprocess */
extern pa_Example_MC_ShObj_t*      shObj;

#define pa_Example_MC_SHM_SIZE     20000

typedef enum pa_Example_shmStr {
	gTxFreeQHndAddr = 0,
	gRxFreeQHndAddr,
	gCpdmaHndAddr,
	gPAInstHndAddr,
	gPaL2HandlesAddr,
	gPaL3HandlesAddr,
	globalCfgDoneAddr,
	localCfgDoneAddr,
    localTestDoneAddr,
    allPktRxCntAddr,
	gPaInstBufAddr,
	gMemL2RamBufAddr,
	gMemL3RamBufAddr,
	gMemUsrStatsBufAddr,
	gPaInstBufSize,
	gMemL2RamBufSize,
	gMemL3RamBufSize,
    gMemUsrStatsBufSize
}pa_Example_shmStr_e;

/* RM instance handle */
extern Rm_Handle                   rmHandle;
extern Rm_ServiceHandle           *rmClientServiceHandle;

#ifndef __LINUX_USER_SPACE
#if RM
/* RM test Global Resource List (GRL) */
extern const char rmGlobalResourceList[];
/* RM test Global Policy provided to RM Server */
extern const char rmDspOnlyPolicy[];
extern const char rmDspPlusArmPolicy[];
/* RM instance transport code */
extern int setupRmTransConfig(uint32_t numTestCores, uint32_t systemInitCore, Task_FuncPtr testTask);
extern volatile uint32_t           isRmInitialized;
#endif /* RM */
/* Handle to CPPI heap */
extern IHeap_Handle            cppiHeap;
#endif /* __LINUX_USER_SPACE */

/* Define LoopBack modes */
//#define CPSW_LOOPBACK_NONE           0    /* No Loopback */
//#define CPSW_LOOPBACK_INTERNAL       1    /* SGMII internal Loopback */
//#define CPSW_LOOPBACK_EXTERNAL       2    /* Loopback outside SoC */

/* Shared Memory Object for the Example */
extern pa_Example_MC_ShObj_t*      shObj;

extern int32_t cpswLpbkMode;
extern int32_t cpswSimTest;
extern uint32_t                procId;
extern uint32_t                errorCount;
/* Tx/Rx packet counters */
extern volatile uint32_t	   gTxCounter, gRxCounter;

extern Qmss_QueueHnd                           gTxFreeQHnd, gRxFreeQHnd, gRxQHnd;

/* Queue with free descriptors */
extern Qmss_QueueHnd                           gGlobalFreeQHnd;

/* TX queues used to send data to PA PDSP/CPSW.*/
extern Qmss_QueueHnd                           gPaTxQHnd [NUM_PA_TX_QUEUES];

/* RX queue used by the application to receive packets from PASS/CPSW.
   Each core has an independent RX queue. */
extern Qmss_QueueHnd                           gRxQHnd;

extern Cppi_ChHnd                              gCpdmaTxChanHnd [NUM_PA_TX_QUEUES];

extern Cppi_ChHnd                              gCpdmaRxChanHnd [NUM_PA_RX_CHANNELS];

extern Cppi_FlowHnd                            gRxFlowHnd;

/* PA command response queue handle */
extern Qmss_QueueHnd                           gPaCfgCmdRespQHnd;

/* TX queue with free decriptors attached to data buffers for transmission.*/
extern Qmss_QueueHnd                           gTxFreeQHnd;

/* RX queue with free decriptors attached to data buffers to be used
   by the PASS CPDMA to hold the received data.*/
extern Qmss_QueueHnd                           gRxFreeQHnd;

/* CPPI Handles used by the application */
extern Cppi_Handle                             gCpdmaHnd;

/* PA Driver Handle */
extern Pa_Handle                               gPAInstHnd;

/* PA L2 Handle */
extern paHandleL2L3_t                          gPaL2Handles;

/* PA L3 Handle */
extern paHandleL2L3_t                          gPaL3Handles;

/* PA L4 Handle */
extern paHandleL4_t                            gPaL4Handles;

/* Packet */
#define PACKET_UDP_DEST_PORT_SHIFT  36
#define PACKET_PAYLOAD_SHIFT        42
#define PACKET_SIZE                 122
extern uint8_t pktMatchBuf[PACKET_SIZE];
extern uint8_t*   pktMatch;

/* Core num/Process number */
extern uint32_t coreNum;

/* External functions */
extern int32_t Cpsw_SwitchOpen (void);
extern int32_t Mdio_Open (void);
extern int32_t Sgmii_Open (void);
extern int32_t Init_Qmss (void);
extern int32_t Init_Qmss_Local (void);
extern int32_t Init_Cppi_Local (void);
extern int32_t Init_Pa_Local (void);
extern int32_t Init_Cppi (void);
extern int32_t Init_Cpsw (void);
extern int32_t Init_PASS (void);
extern int32_t Setup_Tx (void);
extern int32_t Setup_Rx (void);
extern int32_t Setup_PASS (void);
extern uint32_t Convert_CoreLocal2GlobalAddr (uint32_t  addr);
extern int32_t SendPacket (void);
extern int32_t ReceivePacket (void);
extern void  CycleDelay (int32_t count);
extern int32_t VerifyPacket (Cppi_Desc* pCppiDesc);
extern void  ModifyPacket (uint32_t procId);
extern void passPowerUp (void);
extern void APP_exit (int32_t code);
extern int clearFramework(uint32_t procId);
extern void get_qmssGblCfgParamsRegsPhy2Virt(Qmss_GlobalConfigParams     *fw_qmssGblCfgParams);
extern int32_t Del_Port (void);
extern int32_t Del_MACAddress (void);
extern int32_t Del_IPAddress (void);
extern void APP_updateTestStatus(uint32_t id, int32_t rxStatus);
extern int32_t APP_getTestStatus(uint32_t id);
extern void APP_waitAllLocalCfgDone(void);
extern void APP_waitAllLocalTestDone(void);
extern void APP_waitGlobalCfgDone(void);
extern void APP_publishLocalCfgDone(void);
extern void APP_publishLocalTestDone(void);
extern void APP_publishGlobalCfgDone(void);
extern void APP_waitShVar(uint32_t* shVar, uint32_t maxCount);
extern void APP_readAllHndles(void);
extern int32_t initRm(void);
extern void get_cppiGblCfgParamsRegsPhy2Virt(Cppi_GlobalConfigParams     *fw_cppiGblCfgParams);
extern uint32_t fw_shmGetEntry(pa_Example_shmStr_e shmstr);
extern void fw_shmSetEntry(void);
extern void fw_shmCreate(void);
extern int32_t fw_shmDelete(void);
extern void fw_shmOpen(void);
extern void fw_shmClose(void);
extern int32_t fw_shmDelete(void);
extern void fw_SemDestroy(void);
extern void PrintPkt (uint8_t* pkt, uint32_t len, const char* func_name, uint32_t id);
extern void yield (void);
extern void Setup_Tx_local(void);
extern void APP_publishTestStatus(void);
extern int32_t APP_checkTestStatus(void);

#ifdef __LINUX_USER_SPACE
extern void MultiCoreApp(void *args);
#else
extern void MultiCoreApp(UArg arg0, UArg arg1);
#endif

#endif
/* nothing past this point */

