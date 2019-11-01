/**
 *   @file  test_qosSched.h
 *
 *   @brief   
 *       Common definitons for both qos scheduler and drop scheduler
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

#ifndef __TEST_QOSSCHED_H__
#define __TEST_QOSSCHED_H__

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <string.h>

/* QMSS LLD include */
#include <ti/drv/qmss/qmss_drv.h>
#include <ti/drv/qmss/qmss_qosSched.h>

/************************ USER DEFINES ********************/
#define QOS_SCHED_FAST_SCENARIO  // Define this to run scenarios for 1/100th of the configured time
#define NUM_MONOLITHIC_DESC         8192
#define SIZE_MONOLITHIC_DESC        32
#ifdef TEST_DROP_SCHED
#define NUM_STATS_DESC              32
#define SIZE_STATS_DESC             32

#define TEST_LITE_MAX_PHYS_PORTS QMSS_QOS_SCHED_DROP_SCHED_LITE_MAX_PHYS_PORTS
#define TEST_FULL_MAX_PHYS_PORTS QMSS_QOS_SCHED_DROP_SCHED_FULL_MAX_PHYS_PORTS
#define TEST_MAX_PHYS_PORTS      QMSS_QOS_SCHED_DROP_SCHED_MAX_PHYS_PORTS
#define TEST_MAX_FULL_LOG_GROUPS QMSS_QOS_SCHED_LITE_MAX_LOG_GROUPS
#define QOS_SCHED_FAST_SCENARIO  // Define this to run scenarios for 1/100th of the configured time
#define PROFILE_DESCS               60
#define QOS_TX_QUEUES               (80) // This is the number of queues that are allocated for qos scheduler
#define QOS_TX_DROP_QUEUES          (80) // allocated for drop scheduler
#define TEST_PORTS                  20
#define TEST_QUEUES                 (4 * TEST_PORTS) // This is the number of queues actually used
#else
#ifdef WIDE
#define TEST_LITE_MAX_PHYS_PORTS QMSS_QOS_SCHED_WIDE_LITE_MAX_PHYS_PORTS
#define TEST_FULL_MAX_PHYS_PORTS QMSS_QOS_SCHED_WIDE_FULL_MAX_PHYS_PORTS
#define TEST_MAX_PHYS_PORTS      QMSS_QOS_SCHED_WIDE_MAX_PHYS_PORTS
#define TEST_MAX_FULL_LOG_GROUPS QMSS_QOS_SCHED_WIDE_MAX_LOG_GROUPS
#define QOS_SCHED_FAST_SCENARIO  // Define this to run scenarios for 1/100th of the configured time
#define PROFILE_DESCS               16
#define QOS_TX_QUEUES               (17*8) // This is the number of queues that are allocated
#define TEST_PORTS                  1
#define TEST_QUEUES                 (136) // This is the number of queues actually used
#else
#define TEST_LITE_MAX_PHYS_PORTS QMSS_QOS_SCHED_LITE_MAX_PHYS_PORTS
#define TEST_FULL_MAX_PHYS_PORTS QMSS_QOS_SCHED_FULL_MAX_PHYS_PORTS
#define TEST_MAX_PHYS_PORTS      QMSS_QOS_SCHED_MAX_PHYS_PORTS
#define TEST_MAX_FULL_LOG_GROUPS QMSS_QOS_SCHED_FULL_MAX_LOG_GROUPS
#define QOS_SCHED_FAST_SCENARIO  // Define this to run scenarios for 1/100th of the configured time
#define PROFILE_DESCS               39
#define QOS_TX_QUEUES               (80+40) // This is the number of queues that are allocated
#define TEST_PORTS                  12
#define TEST_QUEUES                 (40*2 + 4*(TEST_PORTS - 2)) // This is the number of queues actually used
#endif

#define CORE_SPEED                  983000000
#endif

/*****************************************************************************
 * Disable some test cases on simulator to speed up 
 *****************************************************************************/
#ifdef SIMULATOR_SUPPORT
#ifdef SIMULATOR_SUPPORT_FULL
#define FULL_TEST_SUITE
#endif
#else
#define FULL_TEST_SUITE
#endif

#if (TEST_QUEUES*PROFILE_DESCS > NUM_MONOLITHIC_DESC)
#error Not enough descriptors 
#endif

/* 100us when clock is 983 MHz (or QM's clock is 983/3 = 327.67 MHz) */
/* The test case will still operate if the clocks are different */
#define QOS_TIMER_CONFIG            16383

/* No data is actually transmitted, just used to allow QoS to calculate bandwidth */
#define QOS_DATA_PACKET_SIZE        60
/* Ethernet ovehead -- accounts for preamble, gap, and crc */
#define QOS_DATA_PACKET_OVHD        24


/* Stops the model on descID for debug purposes */
//#define QOS_MODEL_DEBUG_TRIGGER_ENABLE
#define QOS_MODEL_DEBUG_TRIGGER_VAL 0x04600000

#define DESC_ID_OFFSET 1 /* uses tagInfo, which is not used by firmware, this is in 32 bit words */
/* All the queue handles used by the test */
typedef struct
{
    /* Queue handle containing free/unused descriptors */
    Qmss_QueueHnd freeQ;
    /* Queue handle for scratch divert */
    Qmss_QueueHnd divQ;
    /* Set of queues that feed the firmware (input) */
    Qmss_QueueHnd fwQHnds[QOS_TX_QUEUES];
    /* Output queue from firmware */
    Qmss_QueueHnd qosOutQHnd;
    /* Queue to recycle to when firmware drops descriptors */
    Qmss_QueueHnd dropQHnd;
#ifdef TEST_DROP_SCHED
    /* Queues for drop scheduler */
    Qmss_QueueHnd dropFwQHnds[QOS_TX_DROP_QUEUES];
    /* Queues for stats */
    Qmss_QueueHnd statsQs[2];
#endif
} testState_t;

/* Packet generate state */
typedef struct
{
    uint64_t      nextTime;     /* Next time this generator should output packet */
    uint32_t      deltaTime;    /* Delta time between packets */
    Qmss_QueueHnd outputQ;      /* Queue to place packets */
    uint32_t      sizeIdx;      /* Index of next size to use */
    uint32_t      packetsRx;    /* Number of packets received from this stream */
    uint64_t      bytesRx;      /* Number of bytes received from this stream */
    uint32_t      packetsTx;    /* Number of packets transmitted to this stream */
    uint32_t      packetYanked; /* Number of packets yanked from this stream at EOT */
    uint64_t      bytesTx;      /* Number of bytes transmitted to this stream */
    uint64_t      lastRxTime;   /* Timestamp of last received packet */
} pktGenEngState_t;

typedef struct
{
    uint32_t      numPresent;               /* Number of idx used */
    uint32_t      idx[MAX_PKT_GEN_QUEUES];  /* Ordered (by nextTime) list of pending engines */
} pktGenEngList_t;
/************************ EXTERN VARIABLES ********************/
/* Error counter */
extern UInt32                   errorCount;
/* QMSS device specific configuration */
extern Qmss_GlobalConfigParams  qmssGblCfgParams;


#endif /* __TEST_QOSSCHED_H__ */


