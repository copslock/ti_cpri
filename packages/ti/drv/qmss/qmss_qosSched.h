/**
 *   @file  qmss_qosSched.h
 *
 *   @brief   
 *      This is the QMSS QOS packet scheduler header file.
 *      This corresponds to the firmware qos_sched_[be|le]_bin.h
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2012-2015, Texas Instruments, Inc.
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


#ifndef QMSS_QOSSCHED_H_
#define QMSS_QOSSCHED_H_

#ifdef __cplusplus
extern "C" {
#endif

/* QMSS LLD includes */
#include <ti/drv/qmss/qmss_drv.h>

/**
@addtogroup QMSS_LLD_SYMBOL
@{
*/

/** QMSS QoS PDSP number. QoS PDSP is downloaded to PDSP 1 */
#define QMSS_QOS_SCHED_PDSP_NUM                     1

/** Number of full featured physical ports */
#define QMSS_QOS_SCHED_FULL_MAX_PHYS_PORTS          2

/** Number of reduced feature physical ports */
#define QMSS_QOS_SCHED_LITE_MAX_PHYS_PORTS          10

/** Number of full featured physical ports */
#define QMSS_QOS_SCHED_WIDE_FULL_MAX_PHYS_PORTS      1

/** Number of reduced feature physical ports */
#define QMSS_QOS_SCHED_WIDE_LITE_MAX_PHYS_PORTS      0

/** Number of full feature physical ports, when drop scheduler present */
#define QMSS_QOS_SCHED_DROP_SCHED_FULL_MAX_PHYS_PORTS 0
/** Number of reduced feature physical ports, when drop scheduler present */
#define QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS 20

/** Total number of physical ports */
#define QMSS_QOS_SCHED_MAX_PHYS_PORTS (QMSS_QOS_SCHED_FULL_MAX_PHYS_PORTS +\
                                       QMSS_QOS_SCHED_LITE_MAX_PHYS_PORTS)

/** Total number of physical ports for wide build */
#define QMSS_QOS_SCHED_WIDE_MAX_PHYS_PORTS             \
            (QMSS_QOS_SCHED_WIDE_FULL_MAX_PHYS_PORTS + \
             QMSS_QOS_SCHED_WIDE_LITE_MAX_PHYS_PORTS)

/** Total number of physical ports when drop scheduler present */
#define QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS \
                              (QMSS_QOS_SCHED_DROP_SCHED_FULL_MAX_PHYS_PORTS +\
                               QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS)

/** Number of logical groups in full featured port */
#define QMSS_QOS_SCHED_FULL_MAX_LOG_GROUPS          5

/** Number of logical groups in wide full featured port */
#define QMSS_QOS_SCHED_WIDE_MAX_LOG_GROUPS          17

/** Number of logical groups in reduced feature port */
#define QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS          1

/** Number of queues per group in full featured port */
#define QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP    8

/** Number of queues per group in wide full featured port */
#define QMSS_QOS_SCHED_WIDE_MAX_QUEUES_PER_GROUP    8

/** Number of queues per group in reduced feature port */
#define QMSS_QOS_SCHED_LITE_MAX_QUEUES_PER_GROUP    4

/** Bits representing statistics types (forwarded bytes) */
#define QMSS_QOS_SCHED_STATS_FORWARDED_BYTES        1

/** Bits representing statistics types (forwarded packets) */
#define QMSS_QOS_SCHED_STATS_FORWARDED_PACKETS      2

/** Bits representing statistics types (dropped bytes) */
#define QMSS_QOS_SCHED_STATS_DISCARDED_BYTES        4

/** Bits representing statistics types (dropped packets) */
#define QMSS_QOS_SCHED_STATS_DISCARDED_PACKETS      8

/** Number of drop scheduler config profiles supported */
#define QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES         8

/** Number of drop scheduler output profiles supported */
#define QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES        36

/** Number of drop scheduler input queue configs supported */
#define QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS         80

/** Number of drop scheduler stats blocks supported */
#define QMSS_QOS_SCHED_DROP_SCHED_STATS_BLOCKS        48

/** Maximum drop scheduler time constant */
#define QMSS_QOS_SCHED_DROP_SCHED_MAX_TC              16

/** Number of stats queue pairs for push stats */
#define QMSS_QOS_SCHED_DROP_SCHED_STATS_QUEUES        4

/** QMSS QoS scheduler return and Error Codes */
/** QMSS QoS scheduler successful return code */
#define QMSS_QOS_SCHED_RETCODE_SUCCESS              1
/** QMSS QoS scheduler invalid command return code */
#define QMSS_QOS_SCHED_INVALID_COMMAND              2
/** QMSS QoS scheduler invalid index return code */
#define QMSS_QOS_SCHED_INVALID_INDEX                3
/** QMSS QoS scheduler invalid option return code */
#define QMSS_QOS_SCHED_INVALID_OPTION               4
/** QMSS QoS scheduler invalid physical port */
#define QMSS_QOS_SCHED_INVALID_PORT                 100
/** QMSS QoS scheduler invalid group in port */
#define QMSS_QOS_SCHED_INVALID_GROUP                101
/** QMSS QoS scheduler invalid queue in group */
#define QMSS_QOS_SCHED_INVALID_QUEUE                102
/** QMSS QoS scheduler invalid port config wrrType */
#define QMSS_QOS_SCHED_INVALID_WRRTYPE              103
/** QMSS QoS scheduler invalid port config cirType */
#define QMSS_QOS_SCHED_INVALID_CIRTYPE              104
/** QMSS QoS scheduler invalid port config congestionType */
#define QMSS_QOS_SCHED_INVALID_CONGTYPE             105
/** QMSS QoS scheduler attempted to configure a running port */
#define QMSS_QOS_SCHED_INVALID_PORT_STATE           106
/** QMSS QoS scheduler attempted to configure a running group */
#define QMSS_QOS_SCHED_INVALID_GROUP_STATE          107
/** Internal error code for invalid CIR/PIR max */
#define QMSS_QOS_SCHED_INVALID_RATEMAX              0
/** Internal error code for invalid CIR/PIR iteration */
#define QMSS_QOS_SCHED_INVALID_RATEITER             1

/** Internal error for main port CIR */
#define QMSS_QOS_SCHED_INVALID_PORTCIR              108
/** QMSS QoS scheduler cirMax too big (adding credit would overflow) or negative */
#define QMSS_QOS_SCHED_INVALID_CIR_MAX              (QMSS_QOS_SCHED_INVALID_PORTCIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler cirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_CIR_ITERATION        (QMSS_QOS_SCHED_INVALID_PORTCIR + QMSS_QOS_SCHED_INVALID_RATEITER)

/** Internal error for main port PKTCIR */
#define QMSS_QOS_SCHED_INVALID_PORTPKTCIR           110
/** QMSS Qos Scheduler - pktCirMax invalid value */
#define QMSS_QOS_SCHED_INVALID_PORTPKTCIR_MAX       (QMSS_QOS_SCHED_INVALID_PORTPKTCIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler pktCirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_PORTPKTCIR_ITERATION (QMSS_QOS_SCHED_INVALID_PORTPKTCIR + QMSS_QOS_SCHED_INVALID_RATEITER)

/** QMSS QoS scheduler wrrInitialCredit invalid (too big or negative) */
#define QMSS_QOS_SCHED_INVALID_WRR_CREDIT           112
/** QMSS QoS scheduler result underflowed */
#define QMSS_QOS_SCHED_UNDERFLOW                    113
/** QMSS QoS scheduler result overflowed */
#define QMSS_QOS_SCHED_OVERFLOW                     114
/** QMSS QoS scheduler invalid argument */
#define QMSS_QOS_SCHED_INVALID_PARAM                115
/** QMSS QoS scheduler invalid/unknown magic number */
#define QMSS_QOS_SCHED_INVALID_MAGIC                116
/** QMSS QoS scheduler - unused argument is defined */
#define QMSS_QOS_SCHED_UNUSED_GROUP_ARGS            117
/** QMSS QoS scheduler - invalid port config outThrotType */
#define QMSS_QOS_SCHED_INVALID_THROTTYPE            118
/** QMSS QoS Scheduler - API requires build with drop scheduler */
#define QMSS_QOS_SCHED_REQ_DROP_SCHED               119
/** QMSS QoS Scheduler - profile valid flag enum bad */
#define QMSS_QOS_SCHED_INVALID_VALID_FLAG           120
/** QMSS QoS Scheduler - drop scheduler mode bad */
#define QMSS_QOS_SCHED_INVALID_DROP_MODE            121
/** QMSS QoS Scheduler - tail drop threshold type bad */
#define QMSS_QOS_SCHED_INVALID_TAILDROP_THRESH_TYPE 122
/** QMSS QoS Scheduler - drop scheduler RED time constant invalid */
#define QMSS_QOS_SCHED_INVALID_RED_TC               123
/** QMSS QoS Scheduler - config profile index invalid */
#define QMSS_QOS_SCHED_INVALID_CFG_PROF_IDX         124
/** QMSS QoS Scheduler - stats block index invalid */
#define QMSS_QOS_SCHED_INVALID_STATS_BLOCK_IDX      125
/** QMSS QoS Scheduler - output profile index invalid */
#define QMSS_QOS_SCHED_INVALID_OUT_PROF_IDX         126
/** QMSS QoS Scheduler - drop sched REDLowThresh >= REDHighThresh */
#define QMSS_QOS_SCHED_INVALID_RED_THRESH           127
/** QMSS QoS Scheduler - drop sched computation of 
 * 1.0 / (REDHighThresh - REDLowThresh) >> timeConstantP2 failed */
#define QMSS_QOS_SCHED_INVALID_RED_THRESH_RECIP     128
/** QMSS QoS Scheduler - invalid interrupt number */
#define QMSS_QOS_SCHED_INVALID_INTNUM               131
/** QMSS QoS Scheduler - invalid tail drop threshold (overflow) */
#define QMSS_QOS_SCHED_INVALID_TAIL_DROP_THRESH     132
/** QMSS QoS scheduler wrrInitialCredit invalid (too small) */
#define QMSS_QOS_SCHED_SMALL_WRR_CREDIT             133
/** QMSS QoS scheduler push proxy can only do tail push */
#define QMSS_QOS_SCHED_TAIL_ONLY                    134
/** QMSS Qmss_putCfgQosSchedPort() with isJoint only allowed on even ports */
#define QMSS_QOS_SCHED_ISJOINT_EVEN_ONLY            135
/** QMSS Qmss_putCfgQosSchedPort() with isJoint only allowed on lite ports */
#define QMSS_QOS_SCHED_ISJOINT_LITE_ONLY            136

/** Internal error for group CIR */
#define QMSS_QOS_SCHED_INVALID_GROUPCIR             137
/** QMSS Qos Scheduler - pktCirMax invalid value */
#define QMSS_QOS_SCHED_INVALID_GROUPCIR_MAX         (QMSS_QOS_SCHED_INVALID_GROUPCIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler pktCirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_GROUPCIR_ITERATION   (QMSS_QOS_SCHED_INVALID_GROUPCIR + QMSS_QOS_SCHED_INVALID_RATEITER)

/** Internal error for group PKTCIR */
#define QMSS_QOS_SCHED_INVALID_GROUPPKTCIR          139
/** QMSS Qos Scheduler - pktCirMax invalid value */
#define QMSS_QOS_SCHED_INVALID_GROUPPKTCIR_MAX       (QMSS_QOS_SCHED_INVALID_GROUPPKTCIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler pktCirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_GROUPPKTCIR_ITERATION (QMSS_QOS_SCHED_INVALID_GROUPPKTCIR + QMSS_QOS_SCHED_INVALID_RATEITER)

/** Internal error for group PIR */
#define QMSS_QOS_SCHED_INVALID_GROUPPIR             141
/** QMSS QoS scheduler group pktPirMax too big (adding credit would overflow) or negative */
#define QMSS_QOS_SCHED_INVALID_PIR_MAX              (QMSS_QOS_SCHED_INVALID_GROUPPIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler group pktPirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_PIR_ITERATION        (QMSS_QOS_SCHED_INVALID_GROUPPIR + QMSS_QOS_SCHED_INVALID_RATEITER)

/** Internal error for group PKTPIR */
#define QMSS_QOS_SCHED_INVALID_GROUPPKTPIR          143
/** QMSS QoS scheduler group pktPirMax too big (adding credit would overflow) or negative */
#define QMSS_QOS_SCHED_INVALID_PKTPIR_MAX           (QMSS_QOS_SCHED_INVALID_GROUPPKTPIR + QMSS_QOS_SCHED_INVALID_RATEMAX)
/** QMSS QoS scheduler group pktPirIteration invalid (negative) */
#define QMSS_QOS_SCHED_INVALID_PKTPIR_ITERATION     (QMSS_QOS_SCHED_INVALID_GROUPPKTPIR + QMSS_QOS_SCHED_INVALID_RATEITER)
/** QMSS QoS scheduler invalid port config cirType */
#define QMSS_QOS_SCHED_INVALID_GROUP_CIRTYPE        145

/** Shift to convert from packets to the format used for credits.
 *  This is used when the type is Qmss_QosSchedAcctType_PACKETS.
 *  20 means that each unit represents 1/1048576 of a packet */
#define QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT          20
/** Shift to convert from bytes to the format used for credits.
 *  This is used when the type is Qmss_QosSchedAcctType_BYTES 
 *  11 means that each unit represents 1/2048 of a byte */
#define QMSS_QOS_SCHED_BYTES_SCALE_SHIFT            11
/** Shift to convert from packets to the format used for credits
 *  for WRR.
 *  This is used when the type is Qmss_QosSchedAcctType_PACKETS.
 *  17 means that each unit represents 1/131072 of a packet */
#define QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT      (QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT - 3)
/** Shift to convert from bytes to the format used for credits
 *  for WRR.
 *  This is used when the type is Qmss_QosSchedAcctType_BYTES
 *  8 means that each unit represents 1/256 of a byte */
#define QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT        (QMSS_QOS_SCHED_BYTES_SCALE_SHIFT - 3)

/**
@}
*/

/**
@addtogroup QMSS_LLD_ENUM
@{
*/

/** 
 * @brief QoS scheduler accounting types
 */
typedef enum
{
    /** For ports and any WRR, equivalent to packets, since there is 
     * nothing to inheret and prior software used 0 for packets.
     * For group CIR/PIR, inherit the type from port.  This
     * enables backwards compatibilty if structures are memset(x,0,...). */
    Qmss_QosSchedAcctType_INHERITED = 0,
    /** credits are accounted in packets with QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT.
     * This is supported by all firmwares, and works for CIR/PIR/WRR.  The
     * credit is placed in 
     * cirIteration/pirIteration/cirMax/pirMax/wrrInitialCredit */
    Qmss_QosSchedAcctType_PACKETS,
    /** credits are accounted in bytes with QMSS_QOS_SCHED_BYTES_SCALE_SHIFT.
     * This is supported by all firmwares, and works for CIR/PIR/WRR.  The
     * credit is placed in 
     * cirIteration/pirIteration/cirMax/pirMax/wrrInitialCredit */
    Qmss_QosSchedAcctType_BYTES,
    /** credits are accounted in both bytes and packets at the same time.
     * This is only supported by firmware with magic QMSS_QOS_SCHED_MAGIC_MULTIGROUP.
     * It is NOT supported by QMSS_QOS_SCHED_MAGIC_DROPSCHED nor 
     * QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP.
     * This is only supported by CIR/PIR but not WRR
     * The byte credits (with QMSS_QOS_SCHED_BYTES_SCALE_SHIFT) are put in
     * cirIteration/pirIteration/cirMax/pirMax.  The packet credits are put in
     * pktCirIteration/pktPirIteration/pktCirMax/pktPirMax. */
    Qmss_QosSchedAcctType_BOTH
} Qmss_QosSchedAcctType;

/** 
 * @brief QoS scheduler Drop Scheduler profile valid/invalid
 */
typedef enum
{
    /** Profile entry is marked invalid/not used */
    Qmss_QosSchedDropSchedProf_INVALID,

    /** Profile entry is marked valid */
    Qmss_QosSchedDropSchedProf_VALID
} Qmss_QosSchedDropSchedProfValid;

/** 
 * @brief QoS scheduler Drop Scheduler mode
 */
typedef enum
{
    /** Tail Drop Only */
    Qmss_QosSchedDropSchedMode_TAILDROP,

    /** Random Early Drop */
    Qmss_QosSchedDropSchedMode_RED,

    /** Random Early Mark (placeholder, not currently supported */
    Qmss_QosSchedDropSchedMode_REM
} Qmss_QosSchedDropSchedMode;

/** 
 * @brief QoS sched + drop sched join lite ports
 */
typedef enum
{
    /** ports operate independantly */
    Qmss_QosSchedIsJoint_SPLIT = 0,

    /** even+odd port work together to double input queues */
    Qmss_QosSchedIsJoint_JOINT
} Qmss_QosSchedIsJointType;

/**
@}
*/

/** 
@addtogroup QMSS_LLD_DATASTRUCT
@{ 
*/

/** 
 * @brief QoS scheduler queue configuration structure
 */
typedef struct
{
    /** Initial WRR credit for the queue in its group in
     *  @ref Qmss_QosSchedPortCfg.wrrType units.  The value
     *  should be shifted by @ref QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT,
     *  or @ref QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT. */
    int32_t     wrrInitialCredit;
    /** Maximum congestion before drop in
     *  @ref Qmss_QosSchedPortCfg.congestionType units.  0
     *  means no congestion checking on this queue */
    uint32_t    congestionThresh;
} Qmss_QosSchedQueueCfg;


/** 
 * @brief QoS scheduler group configuration
 */
typedef struct
{
    /* Defines bytes/packets/both for this group.  If set to 0 
     * (Qmss_QosSchedAcctType_INHERITED), inherits 
     * value from @ref Qmss_QosSchedPortCfg.cirType.  This value must be 
     * Qmss_QosSchedAcctType_INHERITED or match the ports for
     * all firmwares other than QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP.  */
    Qmss_QosSchedAcctType cirType;

    /** Committed information rate (CIR) credit per iteration in
     *  cirType units.  When using 
     *  @ref Qmss_QosSchedAcctType_BOTH, @ref cirIteration is the byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t     cirIteration;

    /** Committed information rate (CIR) credit per iteration in
     *  packet units, when cirType is @ref Qmss_QosSchedAcctType_BOTH.  
     *  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */
    int32_t     pktCirIteration;

    /** Peak information rate (PIR) credit per iteration in
     *  cirType units.  When using @ref Qmss_QosSchedAcctType_BOTH, 
     *  @ref pirIteration is the byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t     pirIteration;

    /** Peak information rate (PIR) credit per iteration in
     *  packet units, when cirType is @ref Qmss_QosSchedAcctType_BOTH.  
     *  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */
    int32_t     pktPirIteration;

    /** Configured limit to accumulated CIR credit in
     *  cirType units.  When using @ref Qmss_QosSchedAcctType_BOTH, 
     *  @ref cirMax is the max byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t     cirMax;

    /** Configured limit to accumulated pktCir credit in 
     *  packet units, when cirType is @ref Qmss_QosSchedAcctType_BOTH.  
     *  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */
    int32_t     pktCirMax;

    /** Configured limit to accumulated PIR credit in 
     *  cirType units.  When using @ref Qmss_QosSchedAcctType_BOTH, 
     *  @ref pirMax is the max byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t     pirMax;

    /** Configured limit to accumulated pktPir credit in 
     *  packet units, when @ref Qmss_QosSchedPortCfg.cirType is 
     *  @ref Qmss_QosSchedAcctType_BOTH.  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */
    int32_t     pktPirMax;

    /** Initial WRR credit for the group on its port in
     *  @ref Qmss_QosSchedPortCfg.wrrType units.  The value
     *  should be shifted by @ref QMSS_QOS_SCHED_WRR_PACKETS_SCALE_SHIFT,
     *  or @ref QMSS_QOS_SCHED_WRR_BYTES_SCALE_SHIFT. */
    int32_t     wrrInitialCredit;

    /** Total number of active QoS queues (up to 8).  Any queues beyond
     * @ref spQueueCount + @ref wrrQueueCount are best effort queues */
    uint8_t     totQueueCount;

    /** Number of strict priority (SP) queues - usually 2 - 3  */
    uint8_t     spQueueCount;

    /** Number of weighted round robin (WRR) queues - usually 
     *  @ref totQueueCount - @ref spQueueCount */
    uint8_t     wrrQueueCount;

    /** Configuration for each queue in the group */
    Qmss_QosSchedQueueCfg Queue[QMSS_QOS_SCHED_FULL_MAX_QUEUES_PER_GROUP];
} Qmss_QosSchedGroupCfg;

/** 
 * @brief QoS scheduler port configuration
 */
typedef struct
{
    /** Determines if weighted round robin parameters are specified in bytes 
     *  or packets */
    Qmss_QosSchedAcctType wrrType;

    /** Determines if committed information rate and peak information rate are
     * specified in bytes or packets */
    Qmss_QosSchedAcctType cirType;

    /** Determines if congestion thresholds are specified in bytes or packets */
    Qmss_QosSchedAcctType congestionType;

    /** Determines if output throttle threshold is specified in bytes or packets */
    Qmss_QosSchedAcctType outThrotType;

    /** determines whether even lite port is joined with odd lite port 
     * in order to get twice the input queues
     * only used for Qos Scheduler + Drop Sechduler */
    Qmss_QosSchedIsJointType isJoint;

    /** Committed information rate per iteration in 
     *  @ref cirType units.  When using @ref Qmss_QosSchedAcctType_BOTH, 
     *  @ref cirIteration is the byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t cirIteration;

    /** Max committed information rate credit in 
     *  @ref cirType units.  When using @ref Qmss_QosSchedAcctType_BOTH, 
     *  @ref cirMax is the max byte credit.
     *  In all cases, the value should be pre-shifted by 
     *     @ref QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT
     *  or @ref QMSS_QOS_SCHED_BYTES_SCALE_SHIFT directly or by using 
     *  @ref Qmss_convertQosSchedPacketRate or 
     *  @ref Qmss_convertQosSchedBitRate.  */
    int32_t cirMax;

    /** Committed information rate (CIR) credit per iteration in
     *  packet units, when @ref cirType is @ref Qmss_QosSchedAcctType_BOTH.  
     *  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */
    int32_t pktCirIteration;

    /** Configured limit to accumulated pktCir credit in 
     *  packet units, when @ref cirType is @ref Qmss_QosSchedAcctType_BOTH.  
     *  Otherwise, not used.
     *  Value should be pre-shifted by QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT 
     *  directly or by using @ref Qmss_convertQosSchedPacketRate */

    int32_t     pktCirMax;
    /** Number of bytes of wire overhead to account, beyond packet size
     * in QM.  This is often set to 24.  This only affects credits deducted,
     * not statistics.  It also only has effect on credits configured as
     * bytes, not packets.  0 is valid.  This could be used to charge credits
     * for preamble, start of frame, interframe gap, and CRC on ethernet. 
     * Note that both @ref overheadBytes and @ref removeBytes are applied
     * to each packet if they are nonzero.  */
    uint8_t overheadBytes;

    /** Number of bytes of accounting to remove from each packet
     * in QM.  This is often set to 24.  This only affects credits deducted,
     * not statistics.  It also only has effect on credits configured as
     * bytes, not packets.  0 is valid.  This can be used to remove
     * accounting for L1 or L2 headers in order to do L3 or L4 based
     * shaping.  Warning: if any packet enters QoS with a size less than
     * this value, it will cause QoS to break.  This is typically cuased
     * by passing, unintentionally, a 0 value.  Make sure the "C" register
     * gets set for all packets before using this parameter.
     * Note that both @ref overheadBytes and @ref removeBytes are applied
     * to each packet if they are nonzero.  */
    uint8_t removeBytes;

    /** Number of bytes or packets allowed in the output queue at any
     * given time.  Once this limit is reached, no further packets will
     * be forwarded for this port on this tick.  However, unused credits
     * are stored the cir to be used on the next tick (up to @ref cirMax)
     * A value of 0 indicagtes no limit. */
    uint16_t outThrotThresh;

    /** Number of logical groups in @ref group */
    int8_t  groupCount;

    /** Configuration for up to 17 groups */
    Qmss_QosSchedGroupCfg group[QMSS_QOS_SCHED_WIDE_MAX_LOG_GROUPS];

    /** Output/egress queue associated with this port */
    Qmss_Queue  outputQueue;
} Qmss_QosSchedPortCfg;

/** 
 * @brief QoS scheduler statistics
 */
typedef struct
{
    /** # of bytes forwarded */
    uint64_t bytesForwarded;

    /** # of bytes discarded */
    uint64_t bytesDiscarded;

    /** # of packets forwarded */
    uint32_t packetsForwarded;

    /** # of packets discarded */
    uint32_t packetsDiscarded;
} Qmss_QosSchedStats;

/** 
 * @brief Drop scheduler statistics
 */
typedef struct
{
    /** # of bytes forwarded */
    uint32_t bytesForwarded;

    /** # of bytes discarded */
    uint32_t bytesDiscarded;

    /** # of packets forwarded */
    uint32_t packetsForwarded;

    /** # of packets discarded */
    uint32_t packetsDiscarded;
} Qmss_QosSchedDropSchedStats;

/** 
 * @brief Drop scheduler push statistics
 */
typedef struct
{
    /** 8 bit sequence number which is incremented each time a new
     * push stats block is written to the shadow area */
    uint8_t seqn;

    /** Block index associated with following stats */
    uint8_t statBlockIdx;

    /** Forward/Discard stats for this stats block.  Note that the
     * FW reset these stats atomically with pushing them into
     * the stats block shadow area.  */
    Qmss_QosSchedDropSchedStats block;
} Qmss_QosSchedDropSchedPushStats;

/** 
 * @brief Drop Scheduler Output Configuration Profile
 */
typedef struct
{
    /** is this profile entry valid? */
    Qmss_QosSchedDropSchedProfValid valid;

    /** RED drop probability in Q16.  For example, 0x8000 sets to 0.5, and 
     * 0x51f sets to 0.02.  This is the maximum probability to drop packets
     * which applies when the average queue depth is just below the
     * REDHighThresh. */
    uint16_t REDDropProb;

    /** Output queue */
    Qmss_Queue outputQueue;

    /** Index to which configuration profile to use for this queue */
    uint8_t cfgProfIdx;

    /** Readonly result indicating measured queue depth. */
    /* this must be last in the structure for compare to work in testcase */
    uint32_t avgQueDepth;
} Qmss_QosSchedDropSchedOutProf;

/** 
 * @brief Drop Scheduler Configuration Profile
 */
typedef struct
{
    /** Tail drop, RED drop, RED mark */
    Qmss_QosSchedDropSchedMode mode;

    /** Determines if tail drop threshold is in bytes or packets */
    Qmss_QosSchedAcctType tailDropType;

    /** Tail Drop threshold in @ref tailDropType units.  A value of 0
     * disables tail drop */
    uint32_t tailDropThresh;
    
    /** Random Early Drop/Mark low threshold.  If the average queue
     * depth is below this thrshold, then no packets are marked/dropped.
     * Units is bytes shifted by timeConstantP2. */
    uint32_t REDLowThresh;

    /** Random Early Drop/Mark high threshold.  If the average queue
     * depth is above this thrshold, then all packets are marked/dropped.
     * Units is bytes shifted by timeConstantP2. */
    uint32_t REDHighThresh;

    /** Time constant for average as a power of 2.  For example a 
     * time constant of 1/512 is 9. */
    uint8_t  timeConstantP2;
} Qmss_QosSchedDropSchedCfgProf;

/** 
 * @brief Drop Scheduler Queue Configuration Profile
 */
typedef struct
{
    /** Is queue profile valid? */
    Qmss_QosSchedDropSchedProfValid valid;

    /** Queue pair index (+1) to use for push stats in 
     * @ref Qmss_QosSchedDropSchedCfg.statsQueues
     *
     * For example, a value of 0 disables push stats; a value of 3
     * uses statsQueues[2].
     *
     * If the MSB of a stat becomes set, a descriptor is popped from
     * statsQueues[pushStatsIdx].src, the stats are placed
     * in the descriptor directly (its not CPPI) in format 
     * Qmss_QosSchedDropSchedPushStatsFmt.  The filled descriptor
     * is then placed in statsQueuse[pushStatsIdx - 1].dst.
     *
     * The size of the descriptor is not preserved or checked (using
     * C register).  The hint bits (low 4 bits of the descriptor pointer)
     * are preserved.
     *
     * The stats can be decoded into a usable structure using
     * @ref Qmss_convertQosSchedDropSchedPushStats.
     */
    uint8_t pushStatsIdx;

    /** Index to which stats block will be used for this input queue. */ 
    uint8_t statBlockIdx;

    /** Index to which output profile will be used for this input queue. */
    uint8_t outProfIdx;
} Qmss_QosSchedDropSchedQueCfg;

/**
 * @brief Drop Scheduler Push Stats Queue Pair
 *
 * When MSB of a stat becomes set, the firmware will pop a
 * descriptor from the source queue, put the stats in it, then
 * push the descriptor to the destination queue.  These are NOT
 * CPPI descriptors.  The stats are directly placed in the 
 * descriptor.
 */
typedef struct
{
    /** Source queue where firmware takes free descriptors */
    Qmss_Queue src;

    /** Destination queue where firmware puts descriptors with stats */
    Qmss_Queue dst;
} Qmss_QosSchedDropSchedStatsQueues;

/** 
 * @brief Drop Scheduler Top Level configuration
 */
typedef struct
{
    /* seeds must be last for unit test case to compare correctly */
    /** Tausworth seed #1 (0 means leave alone) */
    uint32_t seed1;

    /** Tausworth seed #2 (0 means leave alone) */
    uint32_t seed2;

    /** Tausworth seed #3 (0 means leave alone) */
    uint32_t seed3;

    /** Queue Pairs for push stats */
    Qmss_QosSchedDropSchedStatsQueues 
        statsQueues[QMSS_QOS_SCHED_DROP_SCHED_STATS_QUEUES];
} Qmss_QosSchedDropSchedCfg;


/** 
@} 
*/

/* Exported APIs */
extern Qmss_Result Qmss_setQosSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum);
extern Qmss_Result Qmss_setQosSchedQueueBase (uint32_t queueNum);
extern Qmss_Result Qmss_getQosSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum);
extern Qmss_Result Qmss_getQosSchedQueueBase (uint32_t *queueNum);
extern Qmss_Result Qmss_configureQosSchedTimerSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t timerConstant);
extern Qmss_Result Qmss_configureQosSchedTimer (uint32_t timerConstant);
extern Qmss_Result Qmss_enableQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port);
extern Qmss_Result Qmss_enableQosSchedPort (uint32_t port);
extern Qmss_Result Qmss_disableQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port);
extern Qmss_Result Qmss_disableQosSchedPort (uint32_t port);
extern Qmss_Result Qmss_putCfgQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedPortCfg *cfg);
extern Qmss_Result Qmss_putCfgQosSchedPort (uint32_t port, Qmss_QosSchedPortCfg *cfg);
extern Qmss_Result Qmss_getCfgQosSchedPortSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedPortCfg *cfg);
extern Qmss_Result Qmss_getCfgQosSchedPort (uint32_t port, Qmss_QosSchedPortCfg *cfg);
extern Qmss_Result Qmss_getQosSchedStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedStats *stats, uint32_t port, uint32_t group, uint32_t queue, uint32_t which_reset);
extern Qmss_Result Qmss_getQosSchedStats (Qmss_QosSchedStats *stats, uint32_t port, uint32_t group, uint32_t queue, uint32_t which_reset);
extern Qmss_Result Qmss_getQosSchedGroupStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedStats *stats, int *nStats, uint32_t port, uint32_t group, uint32_t reset);
extern Qmss_Result Qmss_getQosSchedGroupStats (Qmss_QosSchedStats *stats, int *nStats, uint32_t port, uint32_t group, uint32_t reset);
extern uint32_t    Qmss_getQosSchedFwVersionSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId);
extern uint32_t    Qmss_getQosSchedFwVersion (void);
extern uint32_t    Qmss_getQosSchedFwMagicSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId);
extern uint32_t    Qmss_getQosSchedFwMagic (void);
extern Qmss_Result Qmss_convertQosSchedBitRate (int32_t *result, uint32_t ticksPerSecond, uint32_t bitRatePerSecond);
extern Qmss_Result Qmss_convertQosSchedPacketRate (int32_t *result, uint32_t ticksPerSecond, uint32_t packetRatePerSecond);
extern Qmss_Result Qmss_convertQosSchedWrrBits (int32_t *result, uint32_t wrrCredit);
extern Qmss_Result Qmss_convertQosSchedWrrPackets (int32_t *result, uint32_t wrrCredit);

/* APIs that only apply to the Drop Scheduler */
extern Qmss_Result Qmss_setQosSchedDropSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t queueNum);
extern Qmss_Result Qmss_setQosSchedDropSchedQueueBase (uint32_t queueNum);
extern Qmss_Result Qmss_getQosSchedDropSchedQueueBaseSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t *queueNum);
extern Qmss_Result Qmss_getQosSchedDropSchedQueueBase (uint32_t *queueNum);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedOutProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedOutProfs (uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedOutProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedOutProfs (uint32_t port, Qmss_QosSchedDropSchedOutProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedCfgProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedCfgProfs (uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedCfgProfsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedCfgProfs (uint32_t port, Qmss_QosSchedDropSchedCfgProf *profs, int32_t nProfs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedQueCfgsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedQueCfgs (uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedQueCfgsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedQueCfgs (uint32_t port, Qmss_QosSchedDropSchedQueCfg *cfgs, int32_t nCfgs);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfg *cfg);
extern Qmss_Result Qmss_getCfgQosSchedDropSchedCfg (uint32_t port, Qmss_QosSchedDropSchedCfg *cfg);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedCfgSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port, Qmss_QosSchedDropSchedCfg *cfg);
extern Qmss_Result Qmss_putCfgQosSchedDropSchedCfg (uint32_t port, Qmss_QosSchedDropSchedCfg *cfg);
extern Qmss_Result Qmss_enableQosSchedDropSchedSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port);
extern Qmss_Result Qmss_enableQosSchedDropSched (uint32_t port);
extern Qmss_Result Qmss_disableQosSchedDropSchedSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, uint32_t port);
extern Qmss_Result Qmss_disableQosSchedDropSched (uint32_t port);
extern Qmss_Result Qmss_getQosSchedDropSchedStatsSubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QosSchedDropSchedStats *stats, uint32_t port, uint32_t block, uint32_t which_reset);
extern Qmss_Result Qmss_getQosSchedDropSchedStats (Qmss_QosSchedDropSchedStats *stats, uint32_t port, uint32_t block, uint32_t which_reset);
extern Qmss_Result Qmss_convertQosSchedDropSchedPushStats (Qmss_QosSchedDropSchedPushStats *stats, void *desc);
extern Qmss_Result Qmss_qosSchedDropSchedPushProxySubSys (Qmss_SubSysHnd subSysHnd, Qmss_PdspId pdspId, Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location);
extern Qmss_Result Qmss_qosSchedDropSchedPushProxy (Qmss_QueueHnd hnd, void *descAddr, uint32_t packetSize, uint32_t descSize, Qmss_Location location);

#ifdef __cplusplus
}
#endif

#endif /* QMSS_QOSSCHED_H_ */

