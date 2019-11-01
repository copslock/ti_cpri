/**
 * @file pcap_singlecore.h
 *
 * @brief
 *  Holds all the constants and API definitions required by the example
 *  application to run.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2014, Texas Instruments, Inc.
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
#ifndef _PCAP_SINGLECORE_H_
#define _PCAP_SINGLECORE_H_

/* C Standard library Include */
#include <string.h>

 /* c99 types */
#include <stdint.h>

/* Chip Level definitions include */
#include <ti/csl/csl_chip.h>

/* CSL EMAC include */
#include <ti/csl/csl_cpsw.h>

#include <ti/csl/csl_device_interrupt.h>
#include <ti/csl/cslr_msmc.h>
#include <ti/csl/csl_qm_queue.h>

/* CPPI LLD include */
#include <ti/drv/cppi/cppi_drv.h>
#include <ti/drv/cppi/cppi_desc.h>

/* NSS layout */
#include <ti/drv/pa/nss_if.h>
#include <ti/drv/pa/nss_cfg.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>

#ifndef __LINUX_USER_SPACE
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Error.h>
#ifdef _TMS320C6X
#include <ti/sysbios/family/c64p/Hwi.h>
#include <ti/sysbios/family/c64p/EventCombiner.h>
#else
#include <ti/sysbios/family/arm/gic/Hwi.h>
#include <ti/sysbios/family/arm/a15/Mmu.h>
#endif
#else
#include "armv7/linux/fw_test.h"
/* Socket Includes */
#include "./armv7/linux/sockutils.h"
/* Semaphore Includes */
#include <semaphore.h>
#include <errno.h>
#endif

#ifdef __LINUX_USER_SPACE
#define  FLOW_WORKAROUND                    1u
#else
#define  FLOW_WORKAROUND                    0u
#endif

/** Number of host descriptors used by the CPSW example program */
#define         NUM_HOST_DESC               40

/* Number of packets to be used for testing the example. */
#define                     MAX_NUM_PACKETS                          10u

#define                     MAX_RETRIES                              5u

/* Packet Capture configuration unit test structure */
#define    PA_PKT_CAP_INGRESS_CAP_BASE_QUEUE   950
#define    PA_PKT_CAP_EGRESS_CAP_BASE_QUEUE    960
#define    PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_DDR   970
#define    PA_PKT_CAP_QUEUE_BOUNCE_QUEUE_MSMC  971
#define    PA_PKT_CAP_FLOW                       0
#define    PA_PKT_CAP_SWINFO_HIGH_WORD         0xFFFF0000
#define    INGRESS_MIRROR_PORT                 pa_EMAC_PORT_1
#define    EGRESS_MIRROR_PORT                  pa_EMAC_PORT_2

typedef enum {
   NOT_AVAILABLE_FOR_PCAP_TEST = 0,
   AVAILABLE_FOR_PCAP_TEST
}eMacPortState_e;

typedef struct uTestEmacport_s {
  uint8_t          capturePort;
  eMacPortState_e  pState;
} uTestEmacport_t;

typedef struct pktStats_s {
  uint32_t      txCount;
  uint32_t      emacRxCount;
  uint32_t      cloneCaptureCount;
} pktStats_t;

extern pktStats_t expected, actual;

/** Host descriptor size.
 *
 *  Big enough to hold the mandatory fields of the
 *  host descriptor and PA Control Data
 *
 *  = 32 bytes for Host desc + PA Control data
 */
#define         SIZE_HOST_DESC              128

#define         MAX_PA_TX_QUEUES            NSS_MAX_TX_QUEUES
#define         MAX_PA_TX_CHANNELS          NSS_MAX_TX_PKTDMA_CHANNELS
#define         MAX_PA_RX_CHANNELS          NSS_MAX_RX_PKTDMA_CHANNELS

#ifdef SIMULATOR_SUPPORT
/** Number of ports in the ethernet subsystem */
#undef          NUM_PORTS
#define         NUM_PORTS                   CSL_CPSW_NUM_PORTS

#else

#ifndef NUM_PORTS
/** Number of ports in the ethernet subsystem (default for EVM) */
#define         NUM_PORTS                   3u

#endif

#endif /* SIMULATOR_SUPPOR */

/** Number of MAC/GMII ports in the ethernet switch */
#define         NUM_MAC_PORTS               (NUM_PORTS - 1)


#ifndef NSS_GEN2

/* Initial RX queue number */
#define         RX_QUEUE_NUM_INIT           900

#else

/* Initial RX queue number */
#define         RX_QUEUE_NUM_INIT           1024

#endif


/* High Priority QM Rx Interrupt Threshold */
#define		RX_INT_THRESHOLD			1u

/* Accumulator channel to use */
#define		PA_ACC_CHANNEL_NUM			0u

#define CACHE_LINESZ    128
#define SYS_ROUND_UP(x,y)   (((x) + ((y) -1))/(y)*(y))

/* Define LoopBack modes */
#define CPSW_LOOPBACK_NONE           0   /* No Loopback */
#define CPSW_LOOPBACK_INTERNAL       1   /* SGMII internal Loopback */
#define CPSW_LOOPBACK_EXTERNAL       2   /* Loopback outside SoC */
#define CPSW_LOOPBACK_SERDES         3   /* SGMII Serdes Loopback */

/* Define the Receive Data Buffer size */
#define PA_EMAC_EX_RXBUF_SIZE        1518

/** Define RM use or not */
#ifdef SIMULATOR_SUPPORT
#define         RM             0 /* 1: Use, 0: No RM */
#else
#define         RM             1 /* 1: Use, 0: No RM */
#endif

#if RM
extern int setupRm (void);
#ifdef __LINUX_USER_SPACE
/* Linux Specific global variables per process */
extern sock_h                      rmClientSocket;
extern sem_t                       mutex;
extern Rm_ServiceHandle   *rmClientServiceHandle;
#else
extern Rm_Handle          rmHandle;
extern Rm_ServiceHandle  *rmServiceHandle;
#endif

#endif
extern uint8_t                                 gPaCmdBuf1[];
extern uint8_t                                 gPaCmdBuf2[];
extern Qmss_QueueHnd                           gPaCfgCmdRespQHnd;
extern Pa_Handle                               gPAInstHnd;
extern   volatile int  pdsp_halt;
extern uTestEmacport_t ports[8];

extern nssGlobalConfigParams_t nssGblCfgParams;
extern int cpswLpbkMode;
extern int cpswSimTest;
extern Cppi_FlowHnd gRxFlowHnd;
extern Qmss_QueueHnd    gPaTxQHnd [MAX_PA_TX_QUEUES], gTxFreeQHnd, gRxFreeQHnd;
extern volatile uint32_t	  	    gTxCounter, gRxCounter;
extern uint8_t pktMatch[];
extern uint32_t no_bootMode;
extern int dest_emac_port_id;


extern int32_t Cpsw_SwitchOpen (void);
extern int32_t Mdio_Open (void);
extern int32_t Sgmii_Open (void);
extern int32_t Init_Qmss (void);
extern int32_t Init_Cppi (void);
extern int32_t Init_PASS (void);
extern int32_t Setup_Tx (void);
extern int32_t Setup_Rx (void);
extern int32_t Setup_PASS (void);
extern uint32_t Convert_CoreLocal2GlobalAddr (uint32_t  addr);

extern void CycleDelay (int32_t count);
extern void mdebugHaltPdsp (Int pdspNum);
extern int  setupFramework(void);
extern void  get_qmssGblCfgParamsRegsPhy2Virt(Qmss_GlobalConfigParams*);
extern void  get_cppiGblCfgParamsRegsPhy2Virt(Cppi_GlobalConfigParams*);
extern int32_t setup_rx_queue(Qmss_Queue *rxQInfo);
extern void passPowerUp (void);
extern void APP_exit (int32_t code);
extern int clearQm(void);
extern int clearFramework(void);
extern void closeAllOpenedQueues(void);
extern uint8_t * DataBufAlloc(void);
extern void      DataBufFree(void *, uint32_t);
extern int32_t getPaStats (void);
extern int32_t Del_MACAddress(void);
extern int32_t Del_IPAddress(void);
extern int32_t Del_Port(void);
extern uint32_t Convert_CoreGlobal2L2Addr (uint32_t  addr);
extern int32_t Init_Cpsw (void);
extern void cpsw_getStats(CSL_CPSW_STATS*   stats, int clear);
extern void pCap_SingleCoreApp (void);
extern int32_t Download_PAFirmware (void);
extern void CycleDelay (int32_t count);
#ifdef _TMS320C6X
extern volatile unsigned int cregister TSCL;
#endif

/* packet capture functions */
extern int32_t TrigPacketToPdsp0(int emac_dest_port);
extern int32_t VerifyPacket_queue (int direction, int emac_dest_port, uint32_t swInfoMatch);
extern int32_t VerifyPacket_port (Cppi_Desc* pCppiDesc, int emac_dest_port, uint32_t swInfoMatch);

extern int32_t pa_global_config (paCtrlInfo_t* cfgInfo, uint32_t swInfoId );
extern int32_t pcap_enable_global_ingress ( void );
extern int32_t pcap_enable_global_egress ( void );
extern int32_t pcap_disable_global_ingress ( void );
extern int32_t pcap_disable_global_egress ( void );
extern int32_t pkt_capture_test(int direction, int dest_emac_port_id);
extern int32_t port_mirror_test(int direction, int dest_emac_port_id);
#endif /* _PCAP_SINGLECORE_H_ */
/* Nothing past this point */

