/*
 *
 * Copyright (C) 2010-2014 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef _PAUTEST_H
#define _PAUTEST_H

#ifndef __LINUX_USER_SPACE
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/gates/GateHwi.h>
#else
#include <stdint.h>
#include <stdio.h>
/* Socket Includes */
#include "./armv7/linux/sockutils.h"
#include <ti/drv/rm/rm_server_if.h>
/* Semaphore Includes */
#include <semaphore.h>
#include <errno.h>
#endif

#include "pa_log.h"

#include <ti/drv/pa/pa.h>
#include <ti/drv/pa/pa_fc.h>
#include <ti/drv/pa/pasahost.h>
#include <ti/drv/pa/nss_if.h>
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_osal.h>
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>
#include <ti/drv/cppi/cppi_osal.h>
#ifdef _TMS320C6X
#include <ti/csl/csl_cacheAux.h>
#include <ti/csl/csl_xmcAux.h>
#endif
#include <ti/csl/csl_qm_queue.h>

#include <string.h>

#ifdef __LINUX_USER_SPACE
#define System_printf printf
#endif


/** Define RM use or not */
#if defined(SIMULATOR_SUPPORT) || defined(__LINUX_USER_SPACE)
#define         RM             0 /* 1: Use, 0: No RM */
#else
#define         RM             1 /* 1: Use, 0: No RM */
#endif

#if RM
extern int setupRm (void);
#ifdef __LINUX_USER_SPACE
/* Linux Specific global variables per process */
extern sock_h              rmClientSocket;
extern sem_t               mutex;
extern Rm_ServiceHandle   *rmClientServiceHandle;
#else
extern Rm_Handle          rmHandle;
extern Rm_ServiceHandle  *rmServiceHandle;
#endif

#endif


/* Due to a bug in the simulator all packet lengths must be rounded up to 4 byte multiples */
#define PA_SIM_BUG_4BYTES

typedef enum  {
	PA_TEST_FAILED  = -1,
	PA_TEST_NOT_RUN,
	PA_TEST_PASSED
} paTestStatus_t;


/* Define the test interface */
typedef struct paTest_s
{
#ifdef __LINUX_USER_SPACE
	void* (*testFunction)(void *arg);   /* The code that runs the test */
#else
	void (*testFunction)(UArg, UArg);   /* The code that runs the test */
#endif
	char *name;							/* The test name */
	paTestStatus_t testStatus;			/* Test status */

} paTest_t;

extern paTest_t  paTestList[];

#define TF_NUM_PDSPS                       NSS_PA_NUM_PDSPS

#define TF_PA_NUM_RX_CPDMA_CHANNELS        NSS_NUM_RX_PKTDMA_CHANNELS
#define TF_PA_NUM_TX_CPDMA_CHANNELS        NSS_NUM_TX_PKTDMA_CHANNELS

/* Device-specific general purpose queue layout */
#ifdef NSS_GEN2

#define TF_Q_FREE_DESC					   1024   /* Unassociated descriptor queue number */
#define TF_TEARDOWN_QUE_MGR  			   1025   /* Teardown queue number */
#define TF_TEARDOWN_QUE_NUM  				  0

#define TF_LINKED_BUF_Q1           		   1026   /* First queue with attached buffers */
#define TF_LINKED_BUF_Q1_BUF_SIZE  			128
#define TF_LINKED_BUF_Q1_NBUFS       		 32
extern unsigned char memQ1[TF_LINKED_BUF_Q1_NBUFS][TF_LINKED_BUF_Q1_BUF_SIZE];

#define TF_LINKED_BUF_Q2		   		   1027   /* Second queue with attached buffers */
#define TF_LINKED_BUF_Q2_BUF_SIZE  			512
#define TF_LINKED_BUF_Q2_NBUFS				 32
extern unsigned char memQ2[TF_LINKED_BUF_Q2_NBUFS][TF_LINKED_BUF_Q2_BUF_SIZE];

#define TF_LINKED_BUF_Q3		   		   1028
#define TF_LINKED_BUF_Q3_BUF_SIZE  		   3500
#define TF_LINKED_BUF_Q3_NBUFS      		  8
extern unsigned char memQ3[TF_LINKED_BUF_Q3_NBUFS][TF_LINKED_BUF_Q3_BUF_SIZE];

#define TF_DEF_RET_Q                       1029    /* Default return of queues with linked descriptors */

#define TF_COMMON_CMD_REPL_Q			   1030	   /* Common code PA command reply queue */


#define TF_FIRST_GEN_QUEUE		   		   1031	   /* Queues available for general use */

#define TF_NUM_GEN_QUEUES		   			100

/* Temporary test cases before local DMA is available */
//#define TF_GLB_LINKED_BUF_Q1           	   1200
//#define TF_GLB_LINKED_BUF_Q2		   	   1201
//#define TF_GLB_LINKED_BUF_Q3		   	   1202   /* Second queue with attached buffers */

/* PASS Local Queue definitions */
#define TF_Q_LOC_FREE_DESC 				     32   /* Unassociated descriptor queue number */

#define TF_LOC_LINKED_BUF_Q1           	     33   /* First queue with attached buffers */
extern unsigned char memLocQ1[TF_LINKED_BUF_Q1_NBUFS][TF_LINKED_BUF_Q1_BUF_SIZE];

#define TF_LOC_LINKED_BUF_Q2		   		 34   /* Second queue with attached buffers */
extern unsigned char memLocQ2[TF_LINKED_BUF_Q2_NBUFS][TF_LINKED_BUF_Q2_BUF_SIZE];

#define TF_LOC_LINKED_BUF_Q3		   		 35
extern unsigned char memLocQ3[TF_LINKED_BUF_Q3_NBUFS][TF_LINKED_BUF_Q3_BUF_SIZE];


#else

#define TF_Q_FREE_DESC						900   /* Unassociated descriptor queue number */
#define TF_TEARDOWN_QUE_MGR  				901   /* Teardown queue number */
#define TF_TEARDOWN_QUE_NUM  				  0

#define TF_LINKED_BUF_Q1           			902     	/* First queue with attached buffers */
#define TF_LINKED_BUF_Q1_BUF_SIZE  			128
#define TF_LINKED_BUF_Q1_NBUFS       		 30
extern unsigned char memQ1[TF_LINKED_BUF_Q1_NBUFS][TF_LINKED_BUF_Q1_BUF_SIZE];

#define TF_LINKED_BUF_Q2		   			903		/* Second queue with attached buffers */
#define TF_LINKED_BUF_Q2_BUF_SIZE  			512
#define TF_LINKED_BUF_Q2_NBUFS				 32
extern unsigned char memQ2[TF_LINKED_BUF_Q2_NBUFS][TF_LINKED_BUF_Q2_BUF_SIZE];

#define TF_LINKED_BUF_Q3		   			904
#define TF_LINKED_BUF_Q3_BUF_SIZE  		   3500
#define TF_LINKED_BUF_Q3_NBUFS      		  8
extern unsigned char memQ3[TF_LINKED_BUF_Q3_NBUFS][TF_LINKED_BUF_Q3_BUF_SIZE];

#define TF_DEF_RET_Q                        905     /* Default return of queues with linked descriptors */

#define TF_COMMON_CMD_REPL_Q				906		/* Common code PA command reply queue */


#define TF_FIRST_GEN_QUEUE		   			907		/* Queues available for general use */
#define TF_NUM_GEN_QUEUES		   			100

#endif

#define TF_PA_TX_QUEUE_BASE               QMSS_PASS_QUEUE_BASE

#define TF_PA_QUEUE_INPUT                 (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_INPUT_INDEX)
#define TF_PA_QUEUE_MAC                   (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_MAC_INDEX)
#define TF_PA_QUEUE_OUTER_IP              (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_OUTER_IP_INDEX)
#define TF_PA_QUEUE_INNER_IP              (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_INNER_IP_INDEX)
#define TF_PA_QUEUE_IPSEC                 (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_IPSEC_INDEX)
#define TF_PA_QUEUE_IPSEC2                (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_IPSEC2_INDEX)
#define TF_PA_QUEUE_POST                  (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_POST_INDEX)
#define TF_PA_QUEUE_TXCMD                 (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_TXCMD_INDEX)

#define TF_PA_QUEUE_SASS                  (TF_PA_TX_QUEUE_BASE + NSS_SA_QUEUE_SASS_INDEX)
#define TF_PA_QUEUE_SASS2                 (TF_PA_TX_QUEUE_BASE + NSS_SA_QUEUE_SASS2_INDEX)
#define TF_PA_QUEUE_ETH                   (TF_PA_TX_QUEUE_BASE + NSS_CPSW_QUEUE_ETH_INDEX)


#define TF_PA_Q_INPUT                     (NSS_PA_QUEUE_INPUT_INDEX)
#define TF_PA_Q_OUTER_IP                  (NSS_PA_QUEUE_OUTER_IP_INDEX)
#define TF_PA_Q_INNER_IP                  (NSS_PA_QUEUE_INNER_IP_INDEX)
#define TF_PA_Q_LUT2                      (NSS_PA_QUEUE_LUT2_INDEX)
#define TF_PA_Q_POST                      (NSS_PA_QUEUE_POST_INDEX)
#define TF_PA_Q_TXCMD                     (NSS_PA_QUEUE_TXCMD_INDEX)

#define TF_PA_Q_CONFIG_BASE               NSS_PA_QUEUE_INPUT_INDEX      /* To be deleted */

#ifdef NSS_GEN2

#define TF_PA_QUEUE_FIREWALL              (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_FIREWALL_INDEX)
#define TF_PA_QUEUE_FIREWALL2             (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_FIREWALL2_INDEX)
#define TF_PA_QUEUE_EGRESS0               (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_EGRESS0_INDEX)
#define TF_PA_QUEUE_EGRESS1               (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_EGRESS1_INDEX)
#define TF_PA_QUEUE_EGRESS2               (TF_PA_TX_QUEUE_BASE + NSS_PA_QUEUE_EGRESS2_INDEX)

#define TF_PA_Q_FIREWALL                  (NSS_PA_QUEUE_FIREWALL_INDEX)
#define TF_PA_Q_FIREWALL2                 (NSS_PA_QUEUE_FIREWALL2_INDEX)

#define TF_PA_LOC_QUEUE_BASE              QMSS_NETSS_PASS_QUEUE_BASE

#define TF_PA_LOC_QUEUE_INPUT             (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_INPUT_INDEX)
#define TF_PA_LOC_QUEUE_MAC               (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_MAC_INDEX)
#define TF_PA_LOC_QUEUE_OUTER_IP          (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_OUTER_IP_INDEX)
#define TF_PA_LOC_QUEUE_INNER_IP          (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_INNER_IP_INDEX)
#define TF_PA_LOC_QUEUE_IPSEC             (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_IPSEC_INDEX)
#define TF_PA_LOC_QUEUE_IPSEC2            (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_IPSEC2_INDEX)
#define TF_PA_LOC_QUEUE_POST              (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_POST_INDEX)
#define TF_PA_LOC_QUEUE_TXCMD             (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_TXCMD_INDEX)
#define TF_PA_LOC_QUEUE_SASS              (TF_PA_LOC_QUEUE_BASE + NSS_SA_QUEUE_SASS_INDEX)
#define TF_PA_LOC_QUEUE_SASS2             (TF_PA_LOC_QUEUE_BASE + NSS_SA_QUEUE_SASS2_INDEX)
#define TF_PA_LOC_QUEUE_ETH               (TF_PA_LOC_QUEUE_BASE + NSS_CPSW_QUEUE_ETH_INDEX)
#define TF_PA_LOC_QUEUE_FIREWALL          (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_FIREWALL_INDEX)
#define TF_PA_LOC_QUEUE_FIREWALL2         (TF_PA_LOC_QUEUE_BASE + NSS_PA_QUEUE_FIREWALL2_INDEX)

#endif

/* Number of command set and exception entries for the test framework */
#define TF_PA_NUM_CMDSETS                    pa_MAX_CMD_SETS
#define TF_PA_NUM_EXCEPTION                  pa_EROUTE_MAX
#define TF_PA_NUM_EFLOW_EXCEPTION            pa_EFLOW_EROUTE_MAX
#define TF_PA_NUM_MRENTRIES                  pa_MAX_MULTI_ROUTE_SETS

/* The following software info 0 command IDs are reserved by the common setup */
#define TF_COMMON_CMD_ID_ADD_MAC	0xff000000
#define TF_COMMON_CMD_ID_ADD_IP		0xff010000
#define TF_COMMON_CMD_ID_ADD_PORT   0xff020000
#define TF_COMMON_CMD_ID_CFG_CL4  	0xff030000
#define TF_COMMON_CMD_ID_ADD_CL4	0xff040000
#define TF_COMMON_CMD_ID_DEL_CL4	0xff050000
#define TF_COMMON_CMD_ID_DEL_PORT   0xff060000
#define TF_COMMON_CMD_ID_DEL_IP		0xff070000
#define TF_COMMON_CMD_ID_DEL_MAC    0xff080000
#define TF_COMMON_CMD_ID_CFG_CL3  	0xff090000
#define TF_COMMON_CMD_ID_ADD_CL3	0xff0A0000
#define TF_COMMON_CMD_ID_DEL_CL3	0xff0B0000
#define TF_COMMON_CMD_ID_ADD_CMDSET 0xff0C0000
#define TF_COMMON_CMD_ID_DEFGBLCFG  0xff0D0000
#define TF_COMMON_CMD_ID_DEF_802P1AG_CFG  0xff0E0000
#define TF_COMMON_CMD_ID_DEF_NAT_T_CFG    0xff0F0000
#define TF_COMMON_CMD_ID_MULTIROUTE_CFG   0xff100000
#define TF_COMMON_CMD_ID_EXCEPTION_CFG    0xff110000

/* Define the command set log structure */
typedef enum {
  TF_PA_RIDX_NOT_USED = 0, /* PA Resource ID not used */
  TF_PA_RIDX_USED          /* PA Resource ID used */
} TfPaUse_e;

typedef struct PaCmdSet_Log_s {
	TfPaUse_e indication; /* Indicate if cmd set is used or not */
}PaCmdSet_Log_t;

/* Define the command set log structure */
typedef struct PaExc_Log_s {
	TfPaUse_e indication; /* Indicate if exception is used or not  */
}PaExc_Log_t;

/* Define the command set log structure */
typedef struct PaMR_Log_s {
	TfPaUse_e indication; /* Indicate if exception is used or not  */
}PaMR_Log_t;

/* Define the test framework */
typedef struct tFramework_s  {

	Pa_Handle   passHandle;	/* PA instance handle */

#ifndef __LINUX_USER_SPACE
	GateHwi_Handle gateHwi;		/* HW interrupt disable handle */
#endif

	Cppi_Handle tfPaCppiHandle;    /* PA CDMA handle */
	Cppi_Handle tfPaLocCppiHandle; /* PA Local CDMA handle */

	Cppi_ChHnd  tfPaTxChHnd[TF_PA_NUM_TX_CPDMA_CHANNELS];
	Cppi_ChHnd  tfPaRxChHnd[TF_PA_NUM_RX_CPDMA_CHANNELS];
	Cppi_ChHnd  tfPaLocTxChHnd[TF_PA_NUM_TX_CPDMA_CHANNELS];
	Cppi_ChHnd  tfPaLocRxChHnd[TF_PA_NUM_RX_CPDMA_CHANNELS];

	/* Queues */
    Qmss_SubSysHnd  tfPaQmssHandle;             /* PA sub-system QMSS handle */
	int32_t QPaTx[TF_PA_NUM_TX_CPDMA_CHANNELS];
	int32_t QfreeDesc;		      				/* Unassociated descriptor queue handle */
    int32_t QDefRet;                            /* Default return queue for descriptors with linked buffers */
	int32_t QLinkedBuf1;						/* First Queue with descriptors and attached linked buffers */
	int32_t QLinkedBuf2;						/* Second Queue with descriptors and attached linked buffers */
	int32_t QLinkedBuf3;						/* Third Queue with descriptors and attached linked buffers */
	int32_t QCommonCmdRep;						/* Queue for command reply used by common code				*/
	int32_t QGen[TF_NUM_GEN_QUEUES];			/* General purpose queues */

	int32_t QLocfreeDesc;		      			/* Unassociated descriptor queue handle */
	int32_t QLocLinkedBuf1;						/* First Queue with descriptors and attached linked buffers */
	int32_t QLocLinkedBuf2;						/* Second Queue with descriptors and attached linked buffers */
	int32_t QLocLinkedBuf3;						/* Third Queue with descriptors and attached linked buffers */


	Cppi_FlowHnd tfPaFlowHnd[4]; 				/* Flow handle for flow 0-3 */
	int32_t		 tfFlowNum[4];					/* Physical flow number */

	Cppi_FlowHnd tfPaFlowHnd1;					/* Flow handle for flow 4  */
	int32_t		 tfFlowNum1;					/* Physical flow number */

	Cppi_FlowHnd tfPaFlowHnd2;					/* Flow handle for flow 5  */
	int32_t		 tfFlowNum2;					/* Physical flow number */

	Cppi_FlowHnd tfPaLocFlowHnd0; 				/* Flow handle for local flow 0  */
	int32_t		 tfLocFlowNum; 					/* Physical local flow number */

	PaCmdSet_Log_t cmdSetUseList[TF_PA_NUM_CMDSETS];  /* Cmd Set usage logs from PA tests  */
	PaExc_Log_t    excUseList[TF_PA_NUM_EXCEPTION];   /* Exception usage logs from PA tests */
	PaExc_Log_t    efExcUseList[TF_PA_NUM_EFLOW_EXCEPTION]; /* Egree Flow Exception usage logs from PA tests */
	PaMR_Log_t     mrUseList[TF_PA_NUM_MRENTRIES];    /* Multi route usage logs from PA tests */

#ifndef __LINUX_USER_SPACE
	Semaphore_Handle tfPaTableL2Sem;			/* Semaphore for PA internal table for L2 */
	Semaphore_Handle tfPaTableL3Sem;			/* Semaphore for PA internal table for L3 */
	Semaphore_Handle tfQmSem;					/* Semaphore for queue manager */
#endif

} tFramework_t;

extern tFramework_t tFramework;

#ifdef __LINUX_USER_SPACE
/* Define the arguments to unit test functions */
typedef struct paTestArgs_s
{
    tFramework_t *tf;
    paTest_t     *pat;
} paTestArgs_t;
#endif

#define TF_CACHE_LINESZ    128
#define TF_ROUND_UP(x, y)   (((x) + ((y)-1))/(y)*(y))

/* PA memory */
#define TF_PA_INST_SIZE	  400					/* Required size = 320 */
extern uint8_t memPaInst[TF_ROUND_UP(TF_PA_INST_SIZE, TF_CACHE_LINESZ)];

#define TF_MAX_NUM_L2_HANDLES	256
#define TF_L2_TABLE_SIZE	(TF_MAX_NUM_L2_HANDLES * 36)	/* Requires 36 bytes per entry */
extern uint8_t memL2Ram[TF_ROUND_UP(TF_L2_TABLE_SIZE, TF_CACHE_LINESZ)];

#define TF_MAX_NUM_L3_HANDLES	512
#define TF_L3_TABLE_SIZE	(TF_MAX_NUM_L3_HANDLES * 76)	/* Requires 76 bytes per entry */
extern uint8_t memL3Ram[TF_ROUND_UP(TF_L3_TABLE_SIZE, TF_CACHE_LINESZ)];

#define TF_MAX_NUM_VLINK_HANDLES	256
#define TF_VLINK_TABLE_SIZE	(TF_MAX_NUM_VLINK_HANDLES * 12)	/* Requires 12 bytes per entry */
extern uint8_t memVLinkRam[TF_ROUND_UP(TF_VLINK_TABLE_SIZE, TF_CACHE_LINESZ)];

#define TF_MAX_NUM_ACL_HANDLES	512
#define TF_ACL_TABLE_SIZE	(TF_MAX_NUM_ACL_HANDLES * 140)	/* Requires 140 bytes per entry */
extern uint8_t memAclRam[TF_ROUND_UP(TF_ACL_TABLE_SIZE, TF_CACHE_LINESZ)];

#define TF_MAX_NUM_FC_HANDLES	256
#define TF_FC_TABLE_SIZE	(TF_MAX_NUM_FC_HANDLES * 100)	/* Requires 104 bytes per entry */
extern uint8_t memFcRam[TF_ROUND_UP(TF_FC_TABLE_SIZE, TF_CACHE_LINESZ)];

#define TF_USR_STATS_LNK_TABLE_SIZE   (pa_USR_STATS_MAX_COUNTERS * 4)  /* Require 4 bytes per satistics */
extern uint8_t memUsrStatsLnkTbl[TF_USR_STATS_LNK_TABLE_SIZE];

#define TF_MAX_NUM_EOAM_HANDLES	512
#define TF_EOAM_TABLE_SIZE	(TF_MAX_NUM_EOAM_HANDLES * 36)	/* Requires 36 bytes per entry */
extern uint8_t memEoamRam[TF_ROUND_UP(TF_EOAM_TABLE_SIZE, TF_CACHE_LINESZ)];

/* QM memory */
#define TF_NUM_DESC		 128		/* 128 host descriptors managed by the QM */
#define TF_SIZE_DESC     128		/* Must be multiple of 16 */

/* Memory used for the linking RAM and descriptor RAM */
extern uint64_t memLinkRam[TF_NUM_DESC];
extern uint8_t  memDescRam[TF_NUM_DESC * TF_SIZE_DESC];
extern uint8_t  *passDescRam;

typedef uint32_t  paStatsBmap_t;

/* Stats numbers. Util functions are used to add these to the bit maps.
 * The values match the actual stats numbers */
#define TF_STATS_BM_C1_NUM_PACKETS		0
#define TF_STATS_BM_C1_NUM_IPV4			1
#define TF_STATS_BM_C1_NUM_IPV6			2
#define TF_STATS_BM_C1_NUM_CUSTOM		3
#define TF_STATS_BM_C1_NUM_SRIO		    4
#define TF_STATS_BM_C1_TABLE_MATCH   	6
#define TF_STATS_BM_C1_NO_TABLE_MATCH   7
#define TF_STATS_BM_C1_IP_FRAG          8
#define TF_STATS_BM_C1_VLAN_OVERFLOW    10
#define TF_STATS_BM_C1_NUM_MPLS			12
#define TF_STATS_BM_C1_PARSE_FAIL		13
#define TF_STATS_BM_C1_TX_IP_FRAG       15
#define TF_STATS_BM_C1_SILENT_DISCARD   17

#define TF_STATS_BM_C2_NUM_PACKETS		21
#define TF_STATS_BM_C2_NUM_UDP			23
#define TF_STATS_BM_C2_NUM_TCP			24
#define TF_STATS_BM_C2_NUM_CUSTOM		25

#define TF_STATS_BM_M_CMD_FAIL			30

/* Define additional PA Error Codes */
#define pa_NO_BUF                       1    /* Descriptor for PA command packet is not available */

/* Packet and associated structures */
typedef struct pktTestInfo_s  {
	uint8_t 		 *pkt;
	pasahoLongInfo_t *info;
	int			      pktLen;
	paStatsBmap_t     statsMap[3];  /* Bit map of which stats to increment. Some stats must be incremented 6 times */
	int    		      idx;		    /* Used to increment the test tracking - tells the test to look for
						             * a packet with this index */

} pktTestInfo_t;

/* Interface-based Packet and associated structures */
typedef struct ifPktTestInfo_s  {
    pktTestInfo_t    pktInfo;       /* packet Info */
    int              emacPort;      /* Ingress/Egress port number (1-based) */
    int              param[3];      /* test specific parameters */
} ifPktTestInfo_t;


/* Macros to form the packet info structure */
#ifdef NSS_GEN2
#define TF_FORM_PKT_INFO_WORD0(cmdId,recordLen,pmatch,frag,startOffset)  ((cmdId << 29) | (recordLen << 24) | (pmatch << 23) | (frag << 19) | (startOffset))
#define TF_FORM_PKT_INFO_WORD1(endOffset,errIdx,portNum, nextHdr)  ((endOffset << 16) | (errIdx << 10) | (portNum << 6) | (nextHdr))
#define TF_FORM_PKT_INFO_WORD2(l3Offset,l4Offset,l5Offset,ahEspOffset)		((l3Offset << 24) | (l4Offset << 16) | (l5Offset << 8) | ahEspOffset)
#define TF_FORM_EOAM_PKT_INFO_WORD2(megLevel,opCode)		((megLevel << 24) | (opCode << 16) )
#define TF_FORM_PKT_INFO_WORD3(hdrBitmask, pdspNum, l1Index)  \
							   ((hdrBitmask ) | (l1Index << 16) | (pdspNum << 26))
#define TF_FORM_PKT_INFO_WORD4(vc, pri, vlanCnt, greCnt, ipCnt) ((vc << 16) | (pri << 8) | (vlanCnt << 6)  | (greCnt << 3) | (ipCnt))
#define TF_FORM_PKT_INFO_WORD5(pseudo)  (pseudo << 16)
#define TF_FORM_EOAM_PKT_INFO_WORD5(count) (count)

#else
#define TF_FORM_PKT_INFO_WORD0(cmdId,recordLen,startOffset)  ((cmdId << 29) | (recordLen << 24) | startOffset)
#define TF_FORM_PKT_INFO_WORD1(endOffset,errIdx,pmatch,c2c,l1PdspId,l1Idx)  ((endOffset << 16) | (errIdx << 11) | (pmatch << 10) | \
																			 (c2c << 9) | (l1PdspId << 6) | (l1Idx))
#define TF_FORM_PKT_INFO_WORD2(l3Offset,l4Offset,l5Offset,ahEspOffset)		((l3Offset << 24) | (l4Offset << 16) | (l5Offset << 8) | ahEspOffset)
#define TF_FORM_PKT_INFO_WORD3(hdrBitmask,nextHdr,vCount,ipCount,greCount,frag,hdrBitmask2,ipmrt)  \
								((hdrBitmask << 21) | (nextHdr << 16) | (vCount << 14) | (ipCount << 8) | (greCount << 11) | \
								 (hdrBitmask2 << 4) | (frag << 3) | (ipmrt << 1))
#define TF_FORM_PKT_INFO_WORD4(pseudo)  (pseudo << 16)
#endif

#define TF_GET_UDP_OFFSET(x) ((((x)->word2) >> 16) & 0x00ff)
#define TF_GET_UDP_PSEUDO(x) (((x)->word4) >> 16)
#define TF_GET_IP_OFFSET(x) ((((x)->word2) >> 24) & 0x00ff)
#define TF_GET_START_OFFSET(x) ((((x)->word0) >> 0) & 0x00ff)


/* Macros to form the PS Info words for the SRIO packet */
#define TF_FORM_SRIO_PSINFO_WORD0(srcId, destId)        (srcId << 16) | (destId)
#define TF_FORM_SRIO_PSINFO_WORD1_TYPE11(pri, cc, tt, letter, mbox)  ((cc) << 15) | ((pri) << 11) | ((tt) << 9) | ((letter) << 6) | (mbox)
#define TF_FORM_SRIO_PSINFO_WORD1_TYPE9(pri, cc, tt, streamId, cos)  ((cc) <<  8) | ((pri) << 11) | ((tt) << 10) | ( (streamId) << 16) | (cos)

#define PA_PKT_TYPE_SRIO_TYPE_9     30
#define PA_PKT_TYPE_SRIO_TYPE_11    31

/* A simple fifo */
typedef struct pauFifo_s  {
	int		 inIdx;
	int		 outIdx;
	int		 size;
	unsigned int    *data;
} pauFifo_t;

extern paLog_t *paLogLevel;


/* Generic test setup
 * Test setup involves configuring PA to receive MAC, IP, TCP/UDP, custom.
 */

typedef enum   {
	PAU_TEST_SETUP_STATUS_CMD_NOT_SENT,		/* Command not yet sent to the PA */
	PAU_TEST_SETUP_STATUS_CMD_SENT,			/* Command sent, no reply received */
	PAU_TEST_SETUP_STATUS_CMD_REPLIED,		/* Received reply from PA, no errors */
	PAU_TEST_SETUP_STATUS_CMD_REP_ERR,		/* Received reply from PA with errors */
	PAU_TEST_SETUP_STATUS_CMD_NO_CMD		/* Command was intentionally invalid to test PA */
} pauTestSetupStatus_t;



/* MAC test setup */
typedef struct pauTestMacSetup_s  {

	paEthInfo_t  	ethInfo;		/* PA LLD Ethernet configuration */
	paRouteInfo_t   matchRoute;		/* Routing information for a match */
	paRouteInfo_t   nFail;			/* Routing information on subsequent failure */
	paReturn_t   	paret;			/* The expected PA return value  */
	paHandleL2L3_t  handle;			/* The returned PA handle */
	Bool			waitDone;		/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestMacSetup_t;

/* IP test setup. It is an error if both l2Link and L3Link are non-null */
typedef struct pauTestIpSetup_s  {

	paIpInfo_t  			 ipInfo;			/* PA LLD IP configuration */
	paRouteInfo_t   		 matchRoute;		/* Routing information for a match */
	paRouteInfo_t   		 nFail;				/* Routing information on subsequent failure */
	pauTestMacSetup_t  	    *l2Link;			/* Link to L2 handle, NULL if no L2 link */
	struct pauTestIpSetup_s *l3Link;			/* Link to L3 handle, NULL if no L3 Link. */
	paReturn_t      		 paret;				/* The expected PA return value */
	paHandleL2L3_t  		 handle;			/* The returned PA handle */
	Bool					 waitDone;			/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestIpSetup_t;

/* TCP/UDP test setup */
typedef struct pauTestL4Setup_s  {

	uint16_t			destPort;		/* PA LLD destination port */
	paRouteInfo_t   	matchRoute;		/* Routing information for a match */
	pauTestIpSetup_t    *l3Link;		/* Link to L3 handle, NULL if no L3 Link */
	paReturn_t          paret;			/* The expected PA return value */
	paHandleL4_t        handle;			/* The returned PA handle */
	Bool			    waitDone;		/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestL4Setup_t;


/* Custom L3 (LUT1) test setup */
typedef struct pauTestCl3Config_s  {

	uint16_t	custIndex;  					     	/* The custom Index */
    uint16_t    offset;                                 /* offset to where the PA begins custom match */
    uint16_t    nextHdr;                                /* The next header to be parsed */
    uint16_t    nextHdrOffset;                          /* offset to where the PA begins after the custom match */
	uint8_t		byteMasks[pa_NUM_BYTES_CUSTOM_LUT1];	/* The byte bit field masks */
	paReturn_t  paret;									/* The expected PA return value */
	Bool		waitDone;								/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestCl3Config_t;

/* Custom L3 table entries */
typedef struct pauTestCl3Setup_s  {

    uint16_t          custIndex;                        /* Custom Index */
	uint8_t			  match[pa_NUM_BYTES_CUSTOM_LUT1];	/* The table match values */
	paRouteInfo_t     matchRoute;						/* Routing information for a match */
	paRouteInfo_t     nFail;				            /* Routing information on subsequent failure */
	pauTestMacSetup_t  	    *l2Link;			        /* Link to L2 handle, NULL if no L2 link */
	struct pauTestIpSetup_s *l3Link;			        /* Link to L3 handle, NULL if no L3 Link. */
	paReturn_t        paret;							/* The expected PA return value */
	paHandleL2L3_t    handle;			                /* The returned PA handle */
	Bool			  waitDone;							/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestCl3Setup_t;

/* Custom L4 test setup */
typedef struct pauTestCl4Config_s  {

	Bool		handleLink;								/* Set to True if the previous link is included in the look up pattern */
	uint16_t	custIndex;  					     	/* The custom Index */
	uint16_t	custHdrSize;  					     	/* The custom Header size */
	uint16_t	byteOffsets[pa_NUM_BYTES_CUSTOM_LUT2];	/* The byte offsets */
	uint8_t		byteMasks[pa_NUM_BYTES_CUSTOM_LUT2];	/* The byte bit field masks */
	paReturn_t  paret;									/* The expected PA return value */
	Bool		waitDone;								/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestCl4Config_t;

/* Custom L4 table entries */
typedef struct pauTestCl4Setup_s  {

    uint16_t          custIndex;                        /* Custom Index */
	uint8_t			  match[pa_NUM_BYTES_CUSTOM_LUT2];	/* The table match values */
	paRouteInfo_t     matchRoute;						/* Routing information for a match */
	pauTestIpSetup_t *l3Link;							/* Link to IP handle */
	paReturn_t        paret;							/* The expected PA return value */
	paHandleL4_t      handle;							/* The returned PA handle */
	Bool			  waitDone;							/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestCl4Setup_t;

/* Command set entries */
typedef struct pauTestCmdSetSetup_s  {

    uint16_t          index;                            /* command set index */
    int               nCmds;                            /* number of commands */
    paCmdInfo_t      *cmdInfo;                          /* pointer to the command array */
	paReturn_t        paret;							/* The expected PA return value */
	Bool			  waitDone;							/* If True wait for the reply before proceeding */

	int				bufferQ;		/* The queue to use to get the buffer for the command */
	pauTestSetupStatus_t status;    /* The command status. Used internally by the setup program */

} pauTestCmdSetSetup_t;


/* A complete test setup */
typedef struct pauTestSetup_s  {

	int 				nMac;			/* Number of MAC setups */
	pauTestMacSetup_t  *macs;			/* Array of MAC setups */

	int					nIps;			/* Number of IP setups */
	pauTestIpSetup_t   *ips;			/* Array of IP setups */

	int					nL4s;			/* Number of L4 setups */
	pauTestL4Setup_t   *l4s;			/* Array of L4 setups */

	int					nCl3Cfgs;		/* Number of custom L3 configurations */
	pauTestCl3Config_t *cl3cfgs;		/* Array of L3 configs */

	int					nCl3s;			/* Number of custom L3 setups */
	pauTestCl3Setup_t  *cl3s;			/* Array of custom L3 setups */

	int					nCl4Cfgs;		/* Number of custom L4 configurations */
	pauTestCl4Config_t *cl4cfgs;		/* Array of L4 configs */

	int					nCl4s;			/* Number of custom L4 setups */
	pauTestCl4Setup_t  *cl4s;			/* Array of custom L4 setups */

	int					nCmdSets;   	/* Number of command sets */
	pauTestCmdSetSetup_t  *cmdSet; 		/* Array of command sets */

} pauTestSetup_t;

typedef struct pauUsrStatsSetup_s  {
	int	nStats;                                   /* number of groups of user-defined statistics to be configured */
	paUsrStatsCounterEntryConfig_t* cntEntryTbl;  /* PA User-defined statistics configuration structure */
	paReturn_t  paret;							  /* The expected PA return value */

} pauUsrStatsSetup_t;

/* User-defined statistics related definitions and data structures */
#define PAU_USR_STATS_NO_LINK        -1
typedef struct pauUsrStatsEntry_s
{
    uint16_t    cntIndex;    /* counter index */
    int         lnkId;       /* link counter offset within the User-status entry table */
    int         fAlloc;      /* Allocate this counter */
    int         f64b;        /* TRUE: 64-byte counter */
    int         fByteCnt;    /* TRUE: Byte counter */
} pauUsrStatsEntry_t;

/******************************************************************************
 * DATA DEFINITION:  PA Unit Test Payload Type
 ******************************************************************************
 * DESCRIPTION:  Defines the payload types
 ******************************************************************************/
typedef enum pauPayloadType_s
{
    PAU_PAYLOAD_CONST,
    PAU_PAYLOAD_INC8,
    PAU_PAYLOAD_DEC8,
    PAU_PAYLOAD_RANDOM
}   pauPayloadType_e;

/* Prototypes */
void clearPaInfo(void);
int clearTestFramework (void);
int setupTestFramework (void);
int verifyTestFramework (void);
void utilCycleDelay (int count);
int  setupPktTestInfo(pktTestInfo_t* testInfo, int count, char* tfName);
int  setupIfPktTestInfo(ifPktTestInfo_t* testInfo, int count, char* tfname);
uint32_t utilgAddr(uint32_t x);
uint16_t utilCompUdpChksums (pktTestInfo_t *pktinfo, Bool insert);
uint16_t utilCompIpChksums (pktTestInfo_t *pktinfo,  Bool insert);
uint16_t utilGetIpPsudoChkSum (uint8_t *ip, uint16_t payloadLen, uint16_t proto);
uint16_t utilUpdateIpChksums (uint8_t  *ip);
int32_t  utilInitCpsw (void);

void testGenPayload(pauPayloadType_e type, uint8_t initValue, uint16_t len, uint8_t* buf);

int testCommonRequestPaStats (char *fname, tFramework_t *tf, Bool reset, int32_t QSource, int32_t QRecycle,  paCmdReply_t *reply);
int testCommonQueryPaStats (char *fname, tFramework_t *tf, Bool reset, paSysStats_t *paStats);
int testCommonRequestUsrStats (char *fname, tFramework_t *tf, Bool reset, int32_t QSource, int32_t QRecycle,  paCmdReply_t *reply, paUsrStats_t  *usrStats);
int testCommonCompareStats (char *fname, paSysStats_t *expected, paSysStats_t *actual);
int testCommonCompareUsrStats (char *fname, paUsrStats_t *expected, paUsrStats_t *actual);
int testCommonCompareRaStats (char *fname, paRaStats_t *expected, paRaStats_t *actual);
void testCommonDispRaStats (char *fname, paRaStats_t *pStats);
int testCommonRecycleLBDesc (tFramework_t *tf, Cppi_HostDesc *hd);
int testCommonComparePktInfo (char *tfName, pasahoLongInfo_t *expected, pasahoLongInfo_t *actual);
int testCommonComparePktInfoEoamMode (char *tfName, pasahoLongInfo_t *expected, pasahoLongInfo_t *actual, int countCheck);
void testCommonIncStats (paStatsBmap_t *map,  paSysStats_t *stats);
void testCommonCheckComplete (tFramework_t *tf, pauTestSetup_t *setup, char *tfName, char *fname, int fline);
int testCommonSendPackets (tFramework_t *tf, char *tfName, paSysStats_t *stats, pktTestInfo_t *pktInfo, int nPackets, int dest);
Cppi_HostDesc *testCommonAddSrio (tFramework_t *tf, int index, paSrioInfo_t *srioInfo, uint16_t nextHdr, uint16_t nextHdrOffset,
                                 paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute, paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddMac (tFramework_t *tf, int index, paEthInfo_t *ethInfo, paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute,
 	                       		 paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddMac2 (tFramework_t *tf, int index, paEthInfo_t *ethInfo, paRouteInfo2_t *matchRoute, paRouteInfo2_t *nfailRoute,
                                  paHandleL2L3_t nextLink,
 	                       		  paHandleL2L3_t *l2handle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	  paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, uint32_t ctrlBitMap, paReturn_t *paret);
Cppi_HostDesc *testCommonAddAcl (tFramework_t *tf, int aclInst,int action, paAclInfo_t *aclInfo,
 	                       		 paHandleAcl_t *aclhandle, paHandleL2L3_t linkedHandle, paHandleAcl_t nextHandle,
                                 int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddEoam(tFramework_t     *tf,
                                 paEthInfo2_t     *ethInfo,
                                 paEoamFlowInfo_t *eoamInfo,
 	                       		     paHandleL2L3_t   *eoamHandle,
 	                       		     int32_t           Qrecycle,
 	                       		     int32_t           QCmdMem,
 	                       		     paCmdReply_t     *repInfo,
 	                       		     int              *cmdDest,
 	                       		     uint16_t         *cmdSize,
 	                       		     paReturn_t       *paret);
Cppi_HostDesc *testCommonAddFc  (tFramework_t *tf, int index, paEfOpInfo_t *efInfo, paFcInfo_t *fcInfo,
 	                       		 paHandleFc_t *fchandle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonDelHandle (tFramework_t *tf, paHandleL2L3_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply,
									int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonDelL4Handles (tFramework_t *tf, paHandleL4_t l4Handle, int Qrecycle, int QCmdMem, paCmdReply_t *reply,
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonDelAclHandle (tFramework_t *tf, paHandleAcl_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply,
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonDelEoamHandle (tFramework_t *tf, paHandleEoam_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply,
									   int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonDelFcHandle (tFramework_t *tf, paHandleFc_t *paHdl, int Qrecycle, int QCmdMem, paCmdReply_t *cmdReply,
									  int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddIp (tFramework_t *tf, int index, paIpInfo_t *ipInfo, paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute,
 	                       		 paHandleL2L3_t *l3handle, paHandleL2L3_t linkedL2Handle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddIp2 (tFramework_t *tf, int lutInst,int index, paIpInfo_t *ipInfo, paRouteInfo_t *matchRoute, paRouteInfo_t *nfailRoute,
 	                       		 paHandleL2L3_t *l3handle, paHandleL2L3_t linkedL2Handle, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddIp3 (tFramework_t *tf, int lutInst,int index, paIpInfo_t *ipInfo, paRouteInfo2_t *matchRoute, paRouteInfo2_t *nfailRoute,
 	                       		 paHandleL2L3_t *l3handle, paHandleL2L3_t prevLink, paHandleL2L3_t nextLink, int32_t Qrecycle, int32_t QCmdMem,
 	                        	 paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, uint32_t ctrlBitMap, paReturn_t *paret);
Cppi_HostDesc *testCommonAddPort (tFramework_t *tf, int portSize, uint32_t destPort, paRouteInfo_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddPort2 (tFramework_t *tf, int portSize, uint32_t destPort, uint16_t fReplace, uint16_t divertQ, paRouteInfo_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                   int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonAddPort3 (tFramework_t *tf, int portSize, uint32_t destPort, uint16_t fReplace, uint16_t divertQ, paRouteInfo2_t *matchRoute, paHandleL4_t *l4handle, paHandleL2L3_t *l3handle,
                                   int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigMultiRoute (tFramework_t *tf, paMultiRouteModes_e mode, uint16_t index, uint16_t nRoute, paMultiRouteEntry_t *routeEntry,
 	                       		           int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigExceptionRoute (tFramework_t *tf, int nRoute, int *routeTypes, paRouteInfo_t *eRoutes,
 	                       		               int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret, int fEflow);
Cppi_HostDesc *testCommonConfigExceptionRoute2 (tFramework_t *tf, int nRoute, int *routeTypes, paRouteInfo2_t *eRoutes,
 	                       		                int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigDefaultRoute  (tFramework_t *tf, int numPorts, paDefRouteConfig_t *defRouteCfg,
 	                       		              int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigEQoSMode  (tFramework_t *tf, int numPorts, paEQosModeConfig_t *eQoSModeCfg,
 	                       		         int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigCmdSet (tFramework_t *tf, uint16_t index, int nCmd, paCmdInfo_t *cmdInfo,
 	                       		       int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigCrcEngine (tFramework_t *tf, uint16_t index, paCrcConfig_t *cfgInfo,
 	                       		          int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonGlobalConfig (tFramework_t *tf, paCtrlInfo_t *cfgInfo,
 	                       		       int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
Cppi_HostDesc *testCommonConfigUsrStats (tFramework_t *tf, paUsrStatsConfigInfo_t *cfgInfo,
 	                       		         int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
int testCommonAllocUsrStats (tFramework_t *tf, char *tName, int numEntries, pauUsrStatsEntry_t *usrStatsTbl, paUsrStatsCounterEntryConfig_t *usrStatsCfgTbl);
int testCommonFreeUsrStats (tFramework_t *tf, char *tName, int numEntries, pauUsrStatsEntry_t *usrStatsTbl);
paTestStatus_t testCommonUsrStatsSetup (tFramework_t *tf, paTest_t *pat, char *tName, int numStatsGroup, pauUsrStatsSetup_t *usrStatsSetup, uint32_t cmdId, int32_t qSource, int32_t qCmdRecycle, int32_t qCmdReply, Bool clear);
paTestStatus_t testCommonUsrStatsConfigReset (tFramework_t *tf, paTest_t *pat, char *tName, int numStatsGroup, pauUsrStatsSetup_t *usrStatsSetup, uint32_t cmdId, int32_t qSource, int32_t qCmdRecycle, int32_t qCmdReply);


uint16_t testCommonOnesCompAdd (uint16_t v1, uint16_t v2);
int testCommonWaitCmdReply (tFramework_t *tf, paTest_t *pat, char *tname, int Qcmd, uint32_t swinfo0, int line);
int testCommonSetupTest (tFramework_t *tf, paSysStats_t *stats, pauTestSetup_t *setup, char *tfName, char *fname, int fline);
int testCommonTeardownTest (tFramework_t *tf, paSysStats_t *stats, pauTestSetup_t *setup, char *tfName, char *fname, int fline);
paTestStatus_t testCommonCheckStats (tFramework_t *tf, paTest_t *pat, char *tName, paSysStats_t *expected, int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear);
paTestStatus_t testCommonCheckUsrStats (tFramework_t *tf, paTest_t *pat, char *tName, paUsrStats_t *expected, int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear);
paTestStatus_t testCommonCheckUsrStatsList (tFramework_t *tf, paTest_t *pat, char *tName, paUsrStats_t *expected,
                                            int num64bStats, int numCnts, pauUsrStatsEntry_t *usrStatsTbl,
                                            int32_t qSource, int32_t qCmdRecycle, int32_t qReply, Bool clear);
int testCommonCmdSetSetup (tFramework_t *tf, paSysStats_t *stats, pauTestCmdSetSetup_t *cmdSet, paCmdReply_t *rep, char *tfName, char *fname, int fline, int idx);
int testCommonCmdSetRecover (void);
int testCommonSetDefaultGlobalConfig (void);
int testExceptionSetRecover (void);
int testCommonMultiRouteRecover(void);
int testCommonClearPaStats (void);
void testCommonCmdRep (tFramework_t *tf, int *cmdReply, int32_t QCmdResp, int32_t QRet, int expReplyId);
int commonFifoPushElement (pauFifo_t *fifo, unsigned int elem);
int commonFifoGetCount (pauFifo_t *fifo);
unsigned int commonFifoPopElement (pauFifo_t *fifo, int *numInQBeforePop);

void mdebugHaltPdsp (int pdspNum);
void mdebugRunPdsp (int pdspNum);
paTestStatus_t testCommonPrintDbgInfo(tFramework_t *tf, paTest_t *pat, char *tName, paSnapShotDebugInfo_t* paDbgInfo, uint32_t debugType);
Cppi_HostDesc *testCommonConfigDefaultRoute2 (tFramework_t *tf, int nPorts, int nRoute, int *routeTypes, paRouteInfo2_t *dRoutes,
                                              int32_t Qrecycle, int32_t QCmdMem, paCmdReply_t *repInfo, int *cmdDest, uint16_t *cmdSize, paReturn_t *paret);
void testCommonRelayQueueBouncePkts (tFramework_t *tf, char *tfName, int ddrQIdx, int msmcQIdx, int* ddrCnt, int* msmcCnt);
/* Tests */
#ifdef __LINUX_USER_SPACE
void* paTestUnconfigured (void *args);
void* paTestL2Routing (void *args);
void* paTestSrioRouting (void *args);
void* saSimple (void *args);
void* paTestTxFmtRt (void *args);
void* paTestL3Routing (void *args);
void* paTestL4Routing (void *args);
void* paTestPatchRoute (void *args);
void* paTestCustom (void *args);
void* paTestMultiRouting (void *args);
void* paTestIPv4FragReassem (void *args);
void* paTestIPv6FragReassem (void *args);
void* paTestACL (void *args);
void* paTestACLRescore(void *args);
void* paTestEflow (void *args);
#else
void paTestUnconfigured (UArg a0, UArg a1);
void paTestL2Routing (UArg a0, UArg a1);
void paTestSrioRouting (UArg a0, UArg a1);
void saSimple (UArg a0, UArg a1);
void paTestTxFmtRt (UArg a0, UArg a1);
void paTestL3Routing (UArg a0, UArg a1);
void paTestL4Routing (UArg a0, UArg a1);
void paTestPatchRoute (UArg a0, UArg a1);
void paTestCustom (UArg a0, UArg a1);
void paTestMultiRouting (UArg a0, UArg a1);
void paTestIPv4FragReassem (UArg a0, UArg a1);
void paTestIPv6FragReassem (UArg a0, UArg a1);
void paTestACL (UArg a0, UArg a1);
void paTestACLRescore  (UArg a0, UArg a1);
void paTestEOAMFlow(UArg a0, UArg a1);
void paTestEflow (UArg a0, UArg a1);
void paTestLUT2Full (UArg a0, UArg a1);
#endif
#endif /* _PAUTEST_H */
