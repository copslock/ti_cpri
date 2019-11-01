/**
 *   @file  qmss_srioRtr.h
 *
 *   @brief   
 *      This is the QMSS SRIORTR header file.
 *
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
 *  \par
*/


#ifndef QMSS_SRIORTR_H_
#define QMSS_SRIORTR_H_

#ifdef __cplusplus
extern "C" {
#endif

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_qm.h>
#include <ti/drv/qmss/include/qmss_srioRtr_regs.h>

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** QMSS SrioRtr max monitored queues */
#define QMSS_SRIORTR_FW_QUEUES                         16

/** QMSS SrioRtr return and Error Codes */
/** QMSS SrioRtr successful return code */
#define QMSS_SRIORTR_RETCODE_SUCCESS                   QMSS_SOK
/** QMSS SrioRtr invalid/unknown magic number */
#define QMSS_SRIORTR_RETCODE_INVALID_MAGIC             100
/** QMSS SrioRtr port bits set for ports > 4 */
#define QMSS_SRIORTR_RETCODE_INVALID_PORTS             101
/** QMSS SrioRtr port > 4 */
#define QMSS_SRIORTR_RETCODE_INVALID_PORT              102
/** QMSS SrioRtr invalid configuration */
#define QMSS_SRIORTR_RETCODE_INVALID_CFG               103
/**
@}
*/

/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/**
@}
*/

/** @addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/

/** 
 * @brief SrioRtr configuration structure for global (non per port) parameters
 */
typedef struct
{
    /** Base queue for all 12 monitored input queues.  Must be such that
     * base to base + 12 doesn't cross a 32 queue boundary, ie
     * (queueBase / 32) == ((queueBase + 11) / 32)  */
    Qmss_QueueHnd queueBase;
    /** Number of credits associated with each credits granted message */
    uint32_t      creditSize;
} Qmss_SrioRtrGblCfg;

/** 
 * @brief SrioRtr configuration structure for one srio output port
 */
typedef struct
{
    /** SRIO TX queue to use to send packets initiated by the host */
    Qmss_QueueHnd hostSrioTxQueue;
    /** SRIO TX queue to use to send credit packets */
    Qmss_QueueHnd creditSrioTxQueue;
    /** Return queue for received credit packets */
    Qmss_QueueHnd creditReturnQueue;
    /** Source queue for transmit credit packets */
    Qmss_QueueHnd creditSourceQueue;
    /** SRIO TX queue to use to send packets received from SRIO port N */
    Qmss_QueueHnd sourcePortSrioTxQueue[4];
    /** forwarding TX final completion queue (actual forwarding rx free pool) */
    Qmss_QueueHnd fwdTxFinCmpQueue;
    /** Replacement destID (for type9/type11 *router only*).  Upper 16 bits is source, 
     * lower 16 is dest, only lower 8 are used for routing.  If the associated isFinalDest
     * flag is set, then FW patches the LSB of sourceID to 1, else forces it to 0.
     * This gives srio rxu something to use to route the packets to different queues/pools */
    uint32_t      destID;
} Qmss_SrioRtrPortCfg;


typedef struct
{
    /** port number to send each packet for each destination specified in PSINFO */
    uint8_t forwardToPort[256];
    /** set to 1 iff next port is final destination for this packet.  This means not to require credit to transmit, since the final receive queue may have large capacity */
    uint8_t isFinalDest[256];
} Qmss_SrioRtrRouteTbl;

typedef struct
{
    /** SRIO packets received from the port */
    uint32_t packetsRx;
    /** SRIO packets sent to the port */
    uint32_t packetsTx;
    /** credit grants sent to the port */
    uint32_t creditsTx;
    /** credit grants received from the port */
    uint32_t creditsRx;
    /** Host packets sent to the port */
    uint32_t hostPkts;
    /** Forwarded packets returned by srio */
    uint32_t fwdRets;
} Qmss_SrioRtrStats;

/** 
@} 
*/

/* Exported APIs */
extern uint32_t Qmss_getSrioRtrFwVersionSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId);
extern uint32_t Qmss_getSrioRtrFwVersion            (Qmss_PdspId pdspId);
extern uint32_t Qmss_getSrioRtrFwMagicSubSys        (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId);
extern uint32_t Qmss_getSrioRtrFwMagic              (Qmss_PdspId pdspId);

extern Qmss_Result Qmss_disableSrioRtrPortsSubSys   (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t ports);
extern Qmss_Result Qmss_disableSrioRtrPorts         (Qmss_PdspId pdspId, uint32_t ports);
extern Qmss_Result Qmss_enableSrioRtrPortsSubSys    (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t ports);
extern Qmss_Result Qmss_enableSrioRtrPorts          (Qmss_PdspId pdspId, uint32_t ports);

extern Qmss_Result Qmss_getSrioRtrGblCfgSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg);
extern Qmss_Result Qmss_getSrioRtrGblCfg            (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg);
extern Qmss_Result Qmss_setSrioRtrGblCfgSubSys      (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg);
extern Qmss_Result Qmss_setSrioRtrGblCfg            (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrGblCfg *cfg);

extern Qmss_Result Qmss_getSrioRtrPortCfgSubSys     (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg);
extern Qmss_Result Qmss_getSrioRtrPortCfg           (Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg);
extern Qmss_Result Qmss_setSrioRtrPortCfgSubSys     (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg);
extern Qmss_Result Qmss_setSrioRtrPortCfg           (Qmss_PdspId pdspId, int portNum, int stop, Qmss_SrioRtrPortCfg *cfg);

extern Qmss_Result Qmss_getSrioRtrStatsSubSys       (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int portNum, int reset, int stop, Qmss_SrioRtrStats *stats);
extern Qmss_Result Qmss_getSrioRtrStats             (Qmss_PdspId pdspId, int portNum, int reset, int stop, Qmss_SrioRtrStats *stats);

extern Qmss_Result Qmss_getSrioRtrGblRouteTblSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl);
extern Qmss_Result Qmss_getSrioRtrGblRouteTbl       (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl);
extern Qmss_Result Qmss_setSrioRtrGblRouteTblSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl);
extern Qmss_Result Qmss_setSrioRtrGblRouteTbl       (Qmss_PdspId pdspId, int stop, Qmss_SrioRtrRouteTbl *tbl);

#ifdef __cplusplus
}
#endif

#endif /* QMSS_SRIORTR_H_ */

