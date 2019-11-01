/**
 *   @file  qmss_qos.h
 *
 *   @brief   
 *      This is the QMSS QOS header file.
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
 *  \par
*/


#ifndef QMSS_QOS_H_
#define QMSS_QOS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_qm.h>

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** QMSS QoS PDSP number. QoS PDSP is downloaded to PDSP 1 */
#define QMSS_QOS_DEFAULT_PDSP                       Qmss_PdspId_PDSP2
#define QMSS_QOS_MAX_CLUSTERS                       8
#define QMSS_QOS_MAX_QUEUES                         64
#define QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT          9

/** Specifications for Round Robin cluster */
#define QMSS_QOS_MAX_QUE_RR_HIGH_PRI                4
#define QMSS_QOS_MAX_QUE_RR_LOW_PRI                 4
#define QMSS_QOS_MAX_QUE_RR_EGRESS                  1

/** Specifications for the SRIO monitor cluster */
#define QMSS_QOS_SRIO_MIN_TX_Q                      2
#define QMSS_QOS_SRIO_MAX_TX_Q                      5
#define QMSS_QOS_SRIO_MAX_GARBAGE_Q                 6

/** Offset relative to @ref Qmss_QosSrioCfg.queBase for first
 *  Shadow Garbage Collection Queue. 
 */
#define QMSS_QOS_SRIO_SHADOW_GARBAGE_Q_OFFSET       0

/** Offset relative to @ref Qmss_QosSrioCfg.queBase for first
 *  Shadow Transmit Queue. 
 *
 *  SW places descriptors for SRIO transactions here
 */
#define QMSS_QOS_SRIO_SHADOW_TX_Q_OFFSET            6

/** Offset relative to @ref Qmss_QosSrioCfg.queBase for first
 *  Shadow Transmit Completion Queue. 
 *
 *  SRIO's return queues should be pointed here.
 */
#define QMSS_QOS_SRIO_SHADOW_TX_COMPLETION_Q_OFFSET 11

/** Offset relative to @ref Qmss_QosSrioCfg.queBase for first
 *  Transmit Completion Queue. 
 *
 *  The firmware will move descriptors from the Shadow Transmit 
 *  Completion Queue to the Transmit Completion queue after 
 *  subtracting the pending descriptor the number of pending
 *  descriptors on the hardware tx queue.
 */
#define QMSS_QOS_SRIO_TX_COMPLETION_Q_OFFSET       16

/** Total possible queues firmware can use, starting from
 *  @ref Qmss_QosSrioCfg.queBase.
 */
#define QMSS_QOS_SRIO_TX_MAX_FW_Q                  21


/** QMSS QoS commands */
#define QMSS_QCMD_GET_QUEUE_BASE                    0x80
#define QMSS_QCMD_SET_QUEUE_BASE                    0x81
#define QMSS_QCMD_TIMER_CONFIG                      0x82
#define QMSS_QCMD_ENABLE_CLUSTER                    0x83
#define QMSS_QCMD_SRIO_ENABLE                       0x84

/** QMSS QoS return and Error Codes */
/** QMSS QoS successful return code */
#define QCMD_RETCODE_SUCCESS                        1
/** QMSS QoS invalid command return code */
#define QMSS_QCMD_INVALID_COMMAND                   2
/** QMSS QoS invalid index return code */
#define QMSS_QCMD_INVALID_INDEX                     3
/** QMSS QoS invalid option return code */
#define QMSS_QCMD_INVALID_OPTION                    4
/** QMSS QoS invalid cluster mode */
#define QMSS_QCMD_INVALID_MODE                      100
/** QMSS QoS invalid round robin high priority q num */
#define QMSS_QCMD_INVALID_RR_HIGH_Q                 101
/** QMSS QoS invalid round robin low priority q num */
#define QMSS_QCMD_INVALID_RR_LOW_Q                  102
/** QMSS QoS invalid round robin low priority q num */
#define QMSS_QCMD_INVALID_RR_EGRESS_Q               103
/** QMSS QoS SRIO invalid number of queues */
#define QMSS_QCMD_INVALID_SRIO_NUM_Q                104

/**
@}
*/

/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/** 
 * @brief QoS cluster mode
 */
typedef enum
{
    /** Modified Token Bucket Mode */
    Qmss_QosMode_TokenBucket,
    /** Round Robin Mode */
    Qmss_QosMode_RoundRobin
} Qmss_QosMode;

/**
@}
*/

/** @addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/

/** 
 * @brief QoS queue configuration structure
 */
typedef struct
{
    /** Queue manger and Queue index of the forwarding queue */
    uint16_t              egressQueNum;
    /** The amount of forwarding byte credit that the queue receives every 25us */
    uint16_t              iterationCredit;
    /** The maximum amount of forwarding byte credit that the queue is allowed to 
     * hold at the end of the timer iteration. Any credit over the maximum limit 
     * is added to a global pool */
    uint32_t              maxCredit;
    /** The size in bytes at which point the QOS queue is considered to be congested */
    uint32_t              congestionThreshold;
} Qmss_QosQueueCfg;


/** 
 * @brief QoS cluster configuration structure for Modified Token Bucket
 */
typedef struct
{
    /** The maximum amount of global credit allowed to carry over to the next queue. 
     * Excess global credit is discarded */       
    uint32_t              maxGlobalCredit;
    /** The number of QOS queues in this cluster. Valid range is 1 to QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT */
    uint8_t               qosQueCnt;
    /** The queue index (0 to 63) of each QOS queue in the cluster listed 
     * in priority order. These queue indices are relative to the configured QOS 
     * queue base index 
     * Ensure that the queue base passed into @ref Qmss_setQosQueueBase supports 
     * the size of the queue index provided (eg some devices allocate fewer than 64
     * queues).
     */
    uint8_t               qosQueNum[QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT];
    /** This 9-bit mask contains 1 bit for each QOS queue in the cluster. 
     * When this bit is set for its corresponding QOS queue, iteration credit is treated 
     * as "real time" scheduling and does not scale when the egress queue become congested */
    uint16_t              qosQueRTFlags;
    /** The total number of egress queues sampled to obtain the egress queue congestion estimation. 
     * Valid range is 1 to QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT */
    uint8_t               egressQueCnt;
    /** The Queue manger and Queue index of every egress queue enumerated in Egress Queue Count. 
     * These queue indices are absolute index values */
    Qmss_Queue          egressQueNum[QMSS_QOS_MAX_QUE_PER_CLUSTER_COUNT];
    /** Each QOS cluster is configured with four egress congestion threshold values. 
     * Iteration credit is assigned to each queue in the cluster depending on the egress 
     * congestion, and the value of these four congestion thresholds. 
     *
     * It is implemented as shown below:
     *
     * Egress Queue Congestion (Backlog) Level  |   QOS Queue Credit Assigned
     * From no backlog to Threshold 1           |   Double credit
     * Between Threshold 1 and Threshold 2      |   Normal credit
     * Between Threshold 2 and Threshold 3      |   Half credit
     * Between Threshold 3 and Threshold 4      |   Quarter credit
     * Over Threshold 4                         |   No credit
     * 
     */ 
    /** Egress Congestion Threshold point 1 */ 
    uint32_t              egressCongestionThreshold1;
    /** Egress Congestion Threshold point 2 */ 
    uint32_t              egressCongestionThreshold2;
    /** Egress Congestion Threshold point 3 */ 
    uint32_t              egressCongestionThreshold3;
    /** Egress Congestion Threshold point 4 */ 
    uint32_t              egressCongestionThreshold4;
} Qmss_QosClusterCfgTB;

/** 
@} 
*/

/** 
 * @brief QoS cluster configuration structure for Round Robin
 */
typedef struct
{
    /** The maximum amount of global credit allowed to carry over to the next queue. 
     * Excess global credit is discarded */       
    uint32_t              maxGlobalCredit;

    /** The number of high priority QOS queues in this cluster. Valid value is 4. */
    uint8_t               qosQueHighCnt;

    /** The queue index (0 to 63) of each QOS queue in the high priority 
     * round robin group. These queue indices are relative to the configured 
     * QOS queue base index. These fields must be set to 56, 57, 58, and 59 
     * respectively.  Ensure that the base address passed into 
     * @ref Qmss_setQosQueueBase supports 64 queues.
     */
    uint8_t               qosQueNumHigh[QMSS_QOS_MAX_QUE_RR_HIGH_PRI];

    /** The number of high priority QOS queues in this cluster. Valid value is 4. */
    uint8_t               qosQueLowCnt;

    /** The queue index (0 to 63) of each QOS queue in the low priority 
     * round robin group. These queue indices are relative to the configured 
     * QOS queue base index. These fields must be set to 60, 61, 62, and 63 
     * respectively.  Ensure that the base address passed into 
     * @ref Qmss_setQosQueueBase supports 64 queues.
     */
    uint8_t               qosQueNumLow[QMSS_QOS_MAX_QUE_RR_LOW_PRI];

    /** This field holds the value of a packet size adjustment that can be 
     * applied to each packet. For example, setting this value to 24 
     * can adjust for the preamble, inter-packet gap, and CRC for packets 
     * without CRC being sent over Ethernet. This adjustment value is 
     * applied across all queues. */
    uint16_t              sizeAdjust;

    /** The total number of egress queues sampled to obtain the egress 
     * queue congestion estimation.  Valid value is 1.
     */
    uint8_t               egressQueCnt;

    /** The Queue manger and Queue index of every (1) egress queue enumerated 
     * in Egress Queue Count. 
     * These queue indices are absolute index values */
    Qmss_Queue            egressQueNum[QMSS_QOS_MAX_QUE_RR_EGRESS];

    /** This is the per timer tick real time iteration credit for the cluster. 
     * (The iteration credit specified in each of the round robin queues is 
     * ignored.) */
    uint32_t              iterationCredit;

    /** This is the max number of bytes allowed to reside in the egress 
     * queue(s). Note that packets will be written until this threshold is 
     * crossed, so the actual number of bytes queued can be larger. */
    uint32_t              maxEgressBacklog;

    /** This 8-bit mask contains 1 bit for each QOS queue in the cluster. 
     * When this bit is set for its corresponding QOS queue, the queue 
     * is disabled for forwarding. */
    uint32_t              queueDisableMask;
} Qmss_QosClusterCfgRR;

/** 
@} 
*/

/** 
 * @brief QoS cluster configuration structure
 */
typedef struct
{
    /** Select Modified Token Bucket or Round Robin mode */
    Qmss_QosMode mode;
    union { 
        /** configuration for Modified Token Bucket mode */
        Qmss_QosClusterCfgTB cfgTB;
        /** configuration for Round Robin mode */
        Qmss_QosClusterCfgRR cfgRR;
    } u;
} Qmss_QosClusterCfg;
/** 
@} 
*/

/** 
 * @brief QoS SRIO Tracking configuration structure
 */
typedef struct
{
    /** Hardware TX queue # N */
    Qmss_Queue txQ;
    /** High water mark for SRIO queue # at which additional
     *  transmit packets will be held. */
    uint8_t    threshold;
} Qmss_QosSrioTXQCfg;
/** 
@} 
*/

/** 
 * @brief QoS SRIO Tracking configuration structure
 */
typedef struct
{
    /** The number of queues to monitor. This controls both the number of 
     *  valid TXQ entries in this structure, plus the number of queues 
     *  considered valid from the SRIO base queue index. 
     *
     * The valid range is QMSS_QOS_SRIO_MIN_TX_Q to QMSS_QOS_SRIO_MAX_TX_Q.
     */
    uint8_t  queCount;

    /** The Queue index of the base queue of the SRIO queue cluster. 
     *  This value must be a multiple of 32.
     */
    uint16_t queBase;

    /** Configurations for each queue requested via @ref queCount */
    Qmss_QosSrioTXQCfg TXQs[QMSS_QOS_SRIO_MAX_TX_Q];

    /** Configurations for each SRIO garbage return queue (output queue numbers) */
    Qmss_Queue garbageRetQs[QMSS_QOS_SRIO_MAX_GARBAGE_Q];
} Qmss_QosSrioCfg;

/** 
@} 
*/

/* Exported APIs */
extern Qmss_Result Qmss_setQosQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum);
extern Qmss_Result Qmss_setQosQueueBase (uint32_t queueNum);
extern Qmss_Result Qmss_getQosQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum);
extern Qmss_Result Qmss_getQosQueueBase (uint32_t *queueNum);
extern Qmss_Result Qmss_configureQosTimerSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t timerConstant);
extern Qmss_Result Qmss_configureQosTimer (uint32_t timerConstant);
extern Qmss_Result Qmss_enableQosClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex);
extern Qmss_Result Qmss_enableQosCluster (uint32_t clusterIndex);
extern Qmss_Result Qmss_disableQosClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex);
extern Qmss_Result Qmss_disableQosCluster (uint32_t clusterIndex);
extern Qmss_Result Qmss_configureQosQueueSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queIndex, Qmss_QosQueueCfg *cfg);
extern Qmss_Result Qmss_configureQosQueue (uint32_t queIndex, Qmss_QosQueueCfg *cfg);
extern Qmss_Result Qmss_configureQosClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex, Qmss_QosClusterCfg *cfg);
extern Qmss_Result Qmss_configureQosCluster (uint32_t clusterIndex, Qmss_QosClusterCfg *cfg);
extern Qmss_Result Qmss_getQosQueueForwardPktStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueIndex);
extern Qmss_Result Qmss_getQosQueueForwardPktStats (uint32_t queueIndex);
extern Qmss_Result Qmss_getQosQueueDroppedPktStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueIndex);
extern Qmss_Result Qmss_getQosQueueDroppedPktStats (uint32_t queueIndex);
extern Qmss_Result Qmss_resetQosQueueStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueIndex);
extern Qmss_Result Qmss_resetQosQueueStats (uint32_t queueIndex);
extern Qmss_Result Qmss_configureQosSrioClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex, Qmss_QosSrioCfg *cfg);
extern Qmss_Result Qmss_configureQosSrioCluster (uint32_t clusterIndex, Qmss_QosSrioCfg *cfg);
extern Qmss_Result Qmss_enableQosSrioClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex);
extern Qmss_Result Qmss_enableQosSrioCluster (uint32_t clusterIndex);
extern Qmss_Result Qmss_disableQosSrioClusterSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t clusterIndex);
extern Qmss_Result Qmss_disableQosSrioCluster (uint32_t clusterIndex);
uint32_t Qmss_getQosFwVersionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId);
uint32_t Qmss_getQosFwVersion (void);

#ifdef __cplusplus
}
#endif

#endif /* QMSS_QOS_H_ */

