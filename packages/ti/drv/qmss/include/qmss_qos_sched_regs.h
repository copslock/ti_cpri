/********************************************************************
* Copyright (C) 2012-2014 Texas Instruments Incorporated.
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
#ifndef _QMSS_QOS_SCHED_REGS_H_
#define _QMSS_QOS_SCHED_REGS_H_

#include <stdint.h>
#include <stdlib.h>

/* Minimum unit = 1 byte */

/**************************************************************************\
* Register Overlay Structure for COMMAND_BUFFER
\**************************************************************************/
typedef struct  
{
    volatile uint32_t COMMAND_BUFFER_WORD0;
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND_MASK         (0x000000FFu)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_COMMAND_RESETVAL     (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION_MASK          (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION_SHIFT         (0x00000008u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_OPTION_RESETVAL      (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX_MASK           (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX_SHIFT          (0x00000010u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_INDEX_RESETVAL       (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD0_RESETVAL             (0x00000000u)

    volatile uint32_t COMMAND_BUFFER_WORD1;
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE_MASK     (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RETURN_CODE_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_BUFFER_WORD1_RESETVAL             (0x00000000u)

    volatile uint8_t  RSVD0[8];
} Qmss_QosSchedCommandRegs;

/** QMSS QoS commands */
#define QMSS_QOSSCHED_COMMAND_GET_QUEUE_BASE                        (0x80)
#define QMSS_QOSSCHED_COMMAND_SET_QUEUE_BASE                        (0x81)
#define QMSS_QOSSCHED_COMMAND_TIMER_CONFIG                          (0x82)
#define QMSS_QOSSCHED_COMMAND_ENABLE_PORT                           (0x90)
#define QMSS_QOSSCHED_COMMAND_SHADOW_PORT                           (0x91)
#define QMSS_QOSSCHED_COMMAND_STATS_REQUEST                         (0x92)

/* INDEX field encoding for STATS_REQUEST */
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_PORT_MASK                   (0x001F0000u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_PORT_SHIFT                  (0x00000010u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_PORT_RESETVAL               (0x00000000u)

#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_PORT_MASK              (0x00070000u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_PORT_SHIFT             (0x00000010u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_PORT_RESETVAL          (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_GROUP_MASK                  (0x00E00000u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_GROUP_SHIFT                 (0x00000015u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_GROUP_RESETVAL              (0x00000000u)

#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_GROUP_MASK             (0x00F80000u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_GROUP_SHIFT            (0x00000013u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_GROUP_RESETVAL         (0x00000000u)

#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_QUEUE_MASK                  (0xFF000000u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_QUEUE_SHIFT                 (0x00000018u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_QUEUE_RESETVAL              (0x00000000u)

#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_QUEUE_MASK             (0xFF000000u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_QUEUE_SHIFT            (0x00000018u)
#define CSL_QMSS_QOSSCHED_WIDE_COMMAND_INDEX_QUEUE_RESETVAL         (0x00000000u)

/* INDEX field encodign for PORT_SHADOW */
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_ID_MASK                 (0xFF000000u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_ID_SHIFT                (0x00000018u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_ID_RESETVAL             (0x00000000u)

#define QMSS_QOSCHED_PORT_SHADOW_CFG_ID_QOS_SCHED_PORT      0
#define QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG_PROF 1
#define QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_QUE_CFG  2
#define QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_OUT_PROF 3
#define QMSS_QOSSHED_PORT_SHADOW_CFG_ID_DROP_SCHED_CFG      4

#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_IDX_MASK                (0x00FF0000u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_IDX_SHIFT               (0x00000010u)
#define CSL_QMSS_QOSSCHED_COMMAND_INDEX_CFG_IDX_RESETVAL            (0x00000000u)


/**************************************************************************\
* Register Overlay Structure for per queue configuration
\**************************************************************************/
typedef struct 
{
    volatile uint32_t WRR_INITIAL_CREDIT;  /* Initial Queue WRR credit on a "new" schedule */
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_WRR_INITIAL_CREDIT_MASK        (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_WRR_INITIAL_CREDIT_SHIFT       (0x00000000u)
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_WRR_INITIAL_CREDIT_RESETVAL    (0x00000000u)
    volatile uint32_t CONGESTION_THRESH;  /* The max amount of congestion before drop */
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_CONGESTION_THRESH_MASK         (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_CONGESTION_THRESH_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_QUEUE_REGS_CONGESTION_THRESH_RESETVAL     (0x00000000u)
} Qmss_QosSchedQueueRegs;

typedef struct
{
    volatile uint32_t CIR_ITERATION;  /* CIR credit per iteration */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_ITERATION_MASK             (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_ITERATION_SHIFT            (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_ITERATION_RESETVAL         (0x00000000u)

    volatile uint32_t PIR_ITERATION;  /* PIR credit per iteration */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_ITERATION_MASK             (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_ITERATION_SHIFT            (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_ITERATION_RESETVAL         (0x00000000u)

    volatile uint32_t CIR_MAX;        /* CIR max */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_MAX_MASK                   (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_MAX_SHIFT                  (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_CIR_MAX_RESETVAL               (0x00000000u)

    volatile uint32_t PIR_MAX;        /* PIR max */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_MAX_MASK                   (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_MAX_SHIFT                  (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_PIR_MAX_RESETVAL               (0x00000000u)
} Qmss_QosGroupRateCfg;

typedef struct
{
    volatile uint32_t WRR_INITIAL_CREDIT; /* Initial WRR credit */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_WRR_INITIAL_CREDIT_MASK        (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_WRR_INITIAL_CREDIT_SHIFT       (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_WRR_INITIAL_CREDIT_RESETVAL    (0x00000000u)
} Qmss_QosWrrCfg;

/**************************************************************************\
* Register Overlay Structure for per group configuration with bytes or
* packets
\**************************************************************************/
typedef struct
{
    Qmss_QosGroupRateCfg  rate;

    Qmss_QosWrrCfg        wrrCfg;

    volatile uint32_t FLAGS_QUEUE_COUNTS;  /* Total, SP, and WRR counts */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT_MASK          (0x000000FFu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT_SHIFT         (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_TOT_RESETVAL      (0x00000000u)

#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP_MASK           (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP_SHIFT          (0x00000008u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_SP_RESETVAL       (0x00000000u)

#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR_MASK          (0x00FF0000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR_SHIFT         (0x00000010u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_WRR_RESETVAL      (0x00000000u)

/* The following 3 bits are only used for simu bytes & packets */
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING_MASK     (0x20000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING_SHIFT    (0x0000001du)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_BYTE_SHAPING_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING_MASK      (0x40000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING_SHIFT     (0x0000001eu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_PKT_SHAPING_RESETVAL  (0x00000000u)

#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED_MASK        (0x80000000u)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED_SHIFT       (0x0000001fu)
#define CSL_QMSS_QOSSCHED_GROUP_REGS_FLAGS_QUEUE_COUNTS_INHERITED_RESETVAL    (0x00000000u)

    Qmss_QosSchedQueueRegs QUEUE[8];
} Qmss_QosSchedGroupBOPRegs; /* Bytes Or Packets */

/**************************************************************************\
* Register Overlay Structure for per group configuration with simutaneous
* bytes and packets
\**************************************************************************/
typedef struct
{
    Qmss_QosGroupRateCfg  byteRate;

    Qmss_QosWrrCfg        wrrCfg;

    volatile uint32_t FLAGS_QUEUE_COUNTS;  /* Total, SP, and WRR counts */

    Qmss_QosGroupRateCfg  packetRate;

    Qmss_QosSchedQueueRegs QUEUE[8];
} Qmss_QosSchedGroupBAPRegs; /* simutaneous bytes and packets */

typedef struct {
    volatile uint32_t CIR_ITERATION_CREDIT;  /* CIR iteration credit */
    volatile uint32_t CIR_MAX;               /* Max CIR */
} Qmss_QosSchedPortRate_t;

/**************************************************************************\
* Register Overlay Structure for per port configuration excluding groups
\**************************************************************************/
typedef struct {
    volatile uint32_t PORT_FLAGS;   /* Control flags */
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES_MASK             (0x00000001u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES_SHIFT            (0x00000000u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_WRR_IS_BYTES_RESETVAL         (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_IS_BYTES_MASK             (0x00000002u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_IS_BYTES_SHIFT            (0x00000001u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_IS_BYTES_RESETVAL         (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES_MASK     (0x00000004u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES_SHIFT    (0x00000002u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CONG_THRESH_IS_BYTES_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES_MASK       (0x00000008u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES_SHIFT      (0x00000003u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_THROT_IS_BYTES_RESETVAL   (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT_MASK                 (0x00000010u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT_SHIFT                (0x00000004u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_IS_JOINT_RESETVAL             (0x00000000u)

/* The following two flags only work with qos sched w/o drop sched */
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES_MASK             (0x00000002u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES_SHIFT            (0x00000001u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_BYTES_RESETVAL         (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS_MASK              (0x00000020u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS_SHIFT             (0x00000005u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_CIR_BY_PKTS_RESETVAL          (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS_MASK               (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS_SHIFT              (0x00000008u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_NUM_GROUPS_RESETVAL           (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR_MASK            (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR_SHIFT           (0x00000010u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_OUT_QNUM_QMGR_RESETVAL        (0x00000000u)

    volatile uint32_t PORT_FLAGS_2;  /* Control Flags 2 */
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH_MASK       (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH_SHIFT      (0x00000010u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OUT_THROT_THRESH_RESETVAL   (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES_MASK           (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES_SHIFT          (0x00000008u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_REMOVE_BYTES_RESETVAL       (0x00000000u)

#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES_MASK         (0x000000FFu)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_PORT_REGS_PORT_FLAGS_2_OVERHEAD_BYTES_RESETVAL     (0x00000000u)
} Qmss_QosSchedPortHeaderRegs;

/**************************************************************************\
* Register Overlay Structure for per port configuration including groups
\**************************************************************************/
typedef struct {
    Qmss_QosSchedPortHeaderRegs header;
    Qmss_QosSchedPortRate_t     rate;
    /** Configuration for up to 5 groups */
    Qmss_QosSchedGroupBOPRegs   GROUPS[5];
} Qmss_QosSchedPortRegs;

typedef struct {
    Qmss_QosSchedPortHeaderRegs header;
    Qmss_QosSchedPortRate_t     byteRate;
    Qmss_QosSchedPortRate_t     packetRate;
    /** Configuration for up to 5 groups */
    Qmss_QosSchedGroupBAPRegs   GROUPS[5];
} Qmss_QosSchedNarrowPortRegs;

/**************************************************************************\
* Register Overlay Structure for wide port configuration including groups
\**************************************************************************/
typedef struct {
    Qmss_QosSchedPortHeaderRegs header;
    Qmss_QosSchedPortRate_t     rate;
    /** Configuration for up to 17 groups */
    Qmss_QosSchedGroupBOPRegs   GROUPS[17];
} Qmss_QosSchedWidePortRegs;

/**************************************************************************\
* Register Overlay Structure for statistics
\**************************************************************************/
typedef struct {
    volatile uint32_t BYTES_FORWARDED_LSW;
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_LSW_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_LSW_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_LSW_RESETVAL   (0x00000000u)

    volatile uint32_t BYTES_FORWARDED_MSW;
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_MSW_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_MSW_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_FORWARDED_MSW_RESETVAL   (0x00000000u)

    volatile uint32_t BYTES_DISCARDED_LSW;
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_LSW_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_LSW_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_LSW_RESETVAL   (0x00000000u)

    volatile uint32_t BYTES_DISCARDED_MSW;
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_MSW_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_MSW_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_BYTES_DISCARDED_MSW_RESETVAL   (0x00000000u)

    volatile uint32_t PACKETS_FORWARDED;
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_FORWARDED_MASK         (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_FORWARDED_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_FORWARDED_RESETVAL     (0x00000000u)

    volatile uint32_t PACKETS_DISCARDED;
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_DISCARDED_MASK         (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_DISCARDED_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_STATS_REGS_PACKETS_DISCARDED_RESETVAL     (0x00000000u)
} Qmss_QosSchedStatsRegs;

/**************************************************************************\
* Register Overlay Structure for Drop Scheduler Top Level Config
\**************************************************************************/
typedef struct 
{
    volatile uint32_t RES;

    volatile uint32_t SEED1;
    volatile uint32_t SEED2;
    volatile uint32_t SEED3;

#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_SRC_MASK     (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_SRC_SHIFT    (0x00000010u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_SRC_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_DST_MASK     (0x0000FFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_DST_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CONFIG_REGS_STATSQUEUES_DST_RESETVAL (0x00000000u)
    volatile uint32_t STATS_QUEUES[4];
} Qmss_QosSchedDropSchedCfgRegs;

/**************************************************************************\
* Register Overlay Structure for Drop Scheduler Config Profile
\**************************************************************************/
typedef struct
{
    volatile uint32_t CONFIG;
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TCSHIFT_MASK          (0x00FF0000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TCSHIFT_SHIFT         (0x00000010u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TCSHIFT_RESETVAL      (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_MODE_MASK             (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_MODE_SHIFT            (0x00000008u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_MODE_RESETVAL         (0x00000000u)

#define QMSS_QOSSCHED_DROPSCHED_MODE_TAILDROP 0
#define QMSS_QOSSCHED_DROPSCHED_MODE_REDDROP  1
#define QMSS_QOSSCHED_DROPSCHED_MODE_REDMARK  2

#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TD_IS_BYTES_MASK      (0x00000001u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TD_IS_BYTES_SHIFT     (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_CONFIG_TD_IS_BYTES_RESETVAL  (0x00000000u)

    volatile uint32_t TAIL_THRESH;
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_TAIL_THRESH_MASK             (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_TAIL_THRESH_SHIFT            (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_TAIL_THRESH_RESETVAL         (0x00000000u)

    volatile uint32_t RED_LOW;
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_LOW_MASK                 (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_LOW_SHIFT                (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_LOW_RESETVAL             (0x00000000u)

    volatile uint32_t RED_HIGH;
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_HIGH_MASK                (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_HIGH_SHIFT               (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_RED_HIGH_RESETVAL            (0x00000000u)

    volatile uint32_t THRESH_RECIP;
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_THRESH_RECIP_MASK            (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_THRESH_RECIP_SHIFT           (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_CFGPROF_REGS_THRESH_RECIP_RESETVAL        (0x00000000u)
} Qmss_QosSchedDropSchedCfgProfRegs;

/**************************************************************************\
* Register Overlay Structure for Drop Scheduler Output Profile
\**************************************************************************/
typedef struct
{
    volatile uint32_t CONFIG1;
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_DROPPROB_MASK     (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_DROPPROB_SHIFT    (0x00000010u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_DROPPROB_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_OUTQUEUE_MASK     (0x0000FFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_OUTQUEUE_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG1_OUTQUEUE_RESETVAL (0x00000000u)

    volatile uint32_t CONFIG2;
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_ENABLED_MASK      (0x00000100u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_ENABLED_SHIFT     (0x00000008u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_ENABLED_RESETVAL  (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_CFGIDX_MASK       (0x000000FFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_CFGIDX_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_CONFIG2_CFGIDX_RESETVAL   (0x00000000u)

    volatile uint32_t AVG_Q_DEPTH;
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_AVG_Q_DEPTH_MASK          (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_AVG_Q_DEPTH_SHIFT         (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_OUTPROF_REGS_AVG_Q_DEPTH_RESETVAL      (0x00000000u)
} Qmss_QosSchedDropSchedOutProfRegs;

/**************************************************************************\
* Register Overlay Structure for Drop Scheduler Queue Configuration
\**************************************************************************/
typedef struct
{
    volatile uint32_t CONFIG;
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_VALID_MASK        (0x01000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_VALID_SHIFT       (0x00000018u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_VALID_RESETVAL    (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_PUSHSTATSIDX_MASK     (0x00FF0000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_PUSHSTATSIDX_SHIFT    (0x00000010u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_PUSHSTATSIDX_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_STATSIDX_MASK     (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_STATSIDX_SHIFT    (0x00000008u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_STATSIDX_RESETVAL (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_OUTIDX_MASK       (0x000000FFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_OUTIDX_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_QUECFG_REGS_CONFIG_OUTIDX_RESETVAL   (0x00000000u)
} Qmss_QosSchedDropSchedQueCfgRegs;

/**************************************************************************\
* Register Overlay Structure for Push Proxy
\**************************************************************************/
typedef struct
{
    volatile uint32_t QUEUENUM_SIZE;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_QNUM_MASK         (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_QNUM_SHIFT        (0x00000010u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_QNUM_RESETVAL     (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_SIZE_MASK         (0x0000FFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_SIZE_SHIFT        (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_QNUMSIZE_SIZE_RESETVAL     (0x00000000u)

    volatile uint32_t DESCPTR;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_DESCPTR_MASK               (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_DESCPTR_SHIFT              (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHPROXY_REGS_DESCPTR_RESETVAL           (0x00000000u)
} Qmss_QosSchedDropSchedPushProxyRegs;

/**************************************************************************\
* Register Overlay Structure for Drop Scheduler Push Stats
\**************************************************************************/
typedef struct
{
    volatile uint32_t STATSINFO;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_SIZE_MASK        (0x0000FF00u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_SIZE_SHIFT       (0x00000008u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_SIZE_RESETVAL    (0x00000000u)

#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_BLOCK_MASK       (0x000000FFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_BLOCK_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_STATSINFO_BLOCK_RESETVAL   (0x00000000u)

    volatile uint32_t BYTES_FORWARDED;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_FORWARDED_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_FORWARDED_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_FORWARDED_RESETVAL   (0x00000000u)

    volatile uint32_t BYTES_DISCARDED;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_DISCARDED_MASK       (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_DISCARDED_SHIFT      (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_BYTES_DISCARDED_RESETVAL   (0x00000000u)

    volatile uint32_t PACKETS_FORWARDED;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_FORWARDED_MASK     (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_FORWARDED_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_FORWARDED_RESETVAL (0x00000000u)

    volatile uint32_t PACKETS_DISCARDED;
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_DISCARDED_MASK     (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_DISCARDED_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_DROPSCHED_PUSHSTATS_REGS_PACKETS_DISCARDED_RESETVAL (0x00000000u)
} Qmss_QosSchedDropSchedPushStatsFmt;

typedef union
{
    Qmss_QosSchedPortRegs             PORT;          /* size = 0x01c8 */
    Qmss_QosSchedDropSchedCfgRegs     DROP_CFG;      /* size = 0x0004 */
                                                 /* size = 0x00A0 */
    Qmss_QosSchedDropSchedCfgProfRegs CFG_PROFILES[QMSS_QOS_SCHED_DROP_SCHED_CFG_PROFILES];
                                                 /* size = 0x01B0 */
    Qmss_QosSchedDropSchedOutProfRegs OUT_PROFILES[QMSS_QOS_SCHED_DROP_SCHED_OUT_PROFILES];
                                                 /* size = 0x0140 */
    Qmss_QosSchedDropSchedQueCfgRegs  QUE_CONFIGS[QMSS_QOS_SCHED_DROP_SCHED_QUE_CONFIGS];
    /* Note: drop sched + lite ports do NOT support shadow stats for queue stats */
} Qmss_QosSchedShadowURegs;

typedef struct {
    volatile uint32_t                 flags;
#define CSL_QMSS_QOSSCHED_GROUP_SHADOW_STATS_FLAGS_QUEUES_MASK     (0x0000000Fu)
#define CSL_QMSS_QOSSCHED_GROUP_SHADOW_STATS_FLAGS_QUEUES_SHIFT    (0x00000000u)
#define CSL_QMSS_QOSSCHED_GROUP_SHADOW_STATS_FLAGS_QUEUES_RESETVAL (0x00000000u)
    Qmss_QosSchedStatsRegs            stats[8];
} Qmss_QosSchedGroupShadowStats;

typedef union
{
    Qmss_QosSchedWidePortRegs         PORT;
    Qmss_QosSchedGroupShadowStats     STATS;
} Qmss_QosSchedWideURegs;

typedef union
{
    Qmss_QosSchedNarrowPortRegs       PORT;
    Qmss_QosSchedGroupShadowStats     STATS;
} Qmss_QosSchedNarrowURegs;

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
    Qmss_QosSchedShadowURegs            SHADOW_U;        /* offset 0x0040, size = 0x01C8 */
    volatile uint8_t                    RSVD2[0x00D8];   /* offset 0x0208, size = 0x00D8 */
    Qmss_QosSchedDropSchedPushProxyRegs PROXY;           /* offset 0x02E0, size = 0x0008 */
    volatile uint8_t                    RSVD3[0x0018];   /* offset 0x02E8, size = 0x0018 */
    Qmss_QosSchedStatsRegs              STATS;           /* offset 0x0300, size = 0x0018 */
    volatile uint8_t                    RSVD4[0x1ce0];   /* offset 0x0318, size = 0x1ce0 */
} Qmss_QosSchedDropPlusNarrowRegs;

typedef struct  {
    Qmss_QosSchedWideURegs              SHADOW_U;        /* offset 0x0040, size = 0x05e8 */
    volatile uint8_t                    RSVD2[0x0018];   /* offset 0x0628, size = 0x0018 */
    Qmss_QosSchedDropSchedPushProxyRegs PROXY;           /* offset 0x0640, size = 0x0008 */
    volatile uint8_t                    RSVD3[0x0018];   /* offset 0x0648, size = 0x0018 */
    Qmss_QosSchedStatsRegs              STATS;           /* offset 0x0660, size = 0x0018 */
    volatile uint8_t                    RSVD4[0x1980];   /* offset 0x0678, size = 0x1980 */
} Qmss_QosSchedWideRegs;

typedef struct  {
    Qmss_QosSchedNarrowURegs            SHADOW_U;        /* offset 0x0040, size = 0x0220 */
    volatile uint8_t                    RSVD2[0x0080];   /* offset 0x0260, size = 0x0080 */
    Qmss_QosSchedDropSchedPushProxyRegs PROXY;           /* offset 0x02E0, size = 0x0008 */
    volatile uint8_t                    RSVD3[0x0018];   /* offset 0x02E8, size = 0x0018 */
    Qmss_QosSchedStatsRegs              STATS;           /* offset 0x0300, size = 0x0018 */
    volatile uint8_t                    RSVD4[0x1ce0];   /* offset 0x0318, size = 0x1ce0 */
} Qmss_QosSchedNarrowRegs;

typedef struct {
    Qmss_QosSchedCommandRegs            COMMAND_BUFFER;  /* offset 0x0000, size = 0x0010 */
    volatile uint8_t                    RSVD1[0x0030];   /* offset 0x0010, size = 0x0030 */
    union {
	Qmss_QosSchedDropPlusNarrowRegs dropPlusNarrow;  /* qos scheduler (lite) w/ drop scheduler config */
        Qmss_QosSchedNarrowRegs         narrow;          /* qos scheduler narrow multigroup config */
        Qmss_QosSchedWideRegs           wide;            /* wide multigroup config has different address map */
    } u;
    volatile uint32_t                   MAGIC;           /* offset 0x1FF8, size = 0x0004 */
#define CSL_QMSS_QOSSCHED_MAGIC_ENDIAN_MASK                         (0x00000001u)
#define CSL_QMSS_QOSSCHED_MAGIC_ENDIAN_SHIFT                        (0x00000000u)
#define CSL_QMSS_QOSSCHED_MAGIC_ENDIAN_RESETVAL                     (0x00000000u)

#define QMSS_QOS_SCHED_MAGIC_ENDIAN_BIG    1
#define QMSS_QOS_SCHED_MAGIC_ENDIAN_LITTLE 0

#define CSL_QMSS_QOSSCHED_MAGIC_MAGIC_MASK                          (0xFFFF0000u)
#define CSL_QMSS_QOSSCHED_MAGIC_MAGIC_SHIFT                         (0x00000010u)
#define CSL_QMSS_QOSSCHED_MAGIC_MAGIC_RESETVAL                      (0x00000000u)

#define QMSS_QOS_SCHED_MAGIC_MULTIGROUP      0x8010
#define QMSS_QOS_SCHED_MAGIC_DROPSCHED       0x8020
#define QMSS_QOS_SCHED_MAGIC_WIDE_MULTIGROUP 0x8040

    volatile uint32_t                   VERSION;         /* offset 0x1ffC, size = 0x0004 */
#define CSL_QMSS_QOSSCHED_VERSION_MASK                              (0xFFFFFFFFu)
#define CSL_QMSS_QOSSCHED_VERSION_SHIFT                             (0x00000000u)
#define CSL_QMSS_QOSSCHED_VERSION_RESETVAL                          (0x00000000u)
} Qmss_QosSchedRegs;

/* Check size of each overlay is right */
#define QOS_SCHED_OVERLAY_SIZE 0x1FB8

#endif /* _QMSS_QOS_SCHED_REGS_H_ */
