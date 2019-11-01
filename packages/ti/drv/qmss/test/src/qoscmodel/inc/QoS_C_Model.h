/**
 *   @file QoS_C_Model.h
 *
 *   @brief   
 *      This is the QMSS unit test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2015, Texas Instruments, Inc.
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
#ifndef QOS_C_MODEL_H
#define QOS_C_MODEL_H

#define QMSS_QOS_WRR_BYTES_SCALE_SHIFT   (QMSS_QOS_SCHED_BYTES_SCALE_SHIFT - 3)
#define QMSS_QOS_WRR_PACKETS_SCALE_SHIFT (QMSS_QOS_SCHED_PACKETS_SCALE_SHIFT - 3)


typedef struct _QosQueueProc {
    int32_t       WrrCurrentCredit;          // Current Queue WRR credit
    uint32_t      PacketsForwarded;          // Number of packets forwarded
    uint32_t      PacketsDropped;            // Number of packets dropped
    uint64_t      BytesForwarded;            // Number of bytes forwarded
    uint64_t      BytesDropped;              // Number of bytes dropped
    Qmss_Queue    QueueNumber;               // Input queue
} QosQueueProc;

typedef struct _QosQueueCfg {
    int32_t       WrrInitialCredit;          // Initial Queue WRR credit on a "new" schedule
    uint32_t      CongestionThresh;          // The max amount of congestion before drop
} QosQueueCfg;

typedef struct _LogicalGroupProc {
    int32_t       CirCurrentByByte;           // Current CIR credit
    int32_t       CirCurrentByPkt;            // Current CIR credit
    int32_t       PirCurrentByByte;           // Current PIR credit
    int32_t       PirCurrentByPkt;            // Current PIR credit
    uint8_t       NextQueue;                  // The next RR queue to examine in the group
    uint8_t       WrrCreditMask;              // Flag mask of WRR queues that have WRR credit remaining
    int32_t       WrrCurrentCredit;           // Current Group WRR credit
    QosQueueProc  Queue[8];                   // Up to eight queues per logical group
} LogicalGroupProc;

typedef struct _LogicalGroupCfg {
    uint8_t       fIsSupportByteShaping;      // scheduling using *ByByte is enabled
    uint8_t       fIsSupportPacketShaping;    // scheduling using *ByPacket is enabled
    uint8_t       fIsInherited;               // only used for API readback not by c model or real fw
    int32_t       CirIterationByByte;         // CIR credit per iteration
    int32_t       CirIterationByPkt;          // CIR credit per iteration
    int32_t       PirIterationByByte;         // PIR credit per iteration
    int32_t       PirIterationByPkt;          // PIR credit per iteration
    int32_t       CirMaxByByte;               // Max total CIR credit
    int32_t       PirMaxByByte;               // Max total PIR credit
    int32_t       CirMaxByPkt;                // Max total CIR credit
    int32_t       PirMaxByPkt;                // Max total PIR credit
    int32_t       WrrInitialCredit;           // Initial Group WRR credit on a "new" schedule
    uint8_t       QueueCount;                 // Total number of active QOS queues (up to 8)
    uint8_t       SPCount;                    // The number of SP queues (usually 2 or 3)
    uint8_t       RRCount;                    // The number of RR queues (usually QueueCount-SPCount)
    QosQueueCfg   Queue[8];                   // Up to eight queues per logical group
} LogicalGroupCfg;


typedef struct _PhysicalPortProc {
    uint8_t          fEnabled;                // port enable flag
    int32_t          CirCurrentByByte;        // Current CIR credit
    int32_t          CirCurrentByPkt;         // Current CIR credit
    uint8_t          WrrCreditMask;           // Flag mask of WRR groups that have WRR credit remaining
    uint8_t          NextGroup;               // The next RR group to examine
    uint8_t          LastTimerTicks;          // Used to schedule missed interrupts.  Initialized
                                              // to TimerTicks when port is turned on.
    LogicalGroupProc Group[5];                // Up to 5 logical groups
} PhysicalPortProc;

typedef struct _PhysicalPortCfg {
    uint8_t         fByteWrrCredits;          // When set, WRR credits are always in bytes
    uint8_t         fByteCongest;             // When set, congestion is in bytes, else packets
    uint8_t         fByteDestThrottle;        // When set, destination throttle is in bytes, else packets
    uint8_t         fIsJoint;                 // When set, even/odd pair of ports behaves as one
    uint8_t         fIsSupportByteShaping;    // scheduling using *ByByte is enabled
    uint8_t         fIsSupportPacketShaping;  // scheduling using *ByPacket is enabled
    int32_t         CirIterationByByte;       // CIR credit per iteration
    int32_t         CirIterationByPkt;        // CIR credit per iteration
    int32_t         CirMaxByByte;             // Max total CIR credit
    int32_t         CirMaxByPkt;              // Max total CIR credit (always in bytes)
    uint8_t         GroupCount;               // The number of logical groups
    uint8_t         OverheadBytes;            // Number of bytes of wire overhead to account, beyond packet size in QM.
                                              // This is often set to 24.  This only affects credits deducted,
                                              // not statistics.  It also only has effect on credits configured
                                              // as bytes, not packets.
    uint8_t         RemoveBytes;              // Number of bytes
    uint16_t        DestThrottleThresh;       // If output queue has more than this many bytes/packets nothing will be forwarded
    Qmss_Queue      DestQueueNumber;          // Output queue
    LogicalGroupCfg Group[5];                 // Up to 5 logical groups
} PhysicalPortCfg;


void    ForegroundTask(void);
void    BackgroundTask(void);
void PhysPortScheduler(PhysicalPortCfg *pPortCfg, PhysicalPortProc *pPortProc);
int32_t LogicalGroupScheduler(PhysicalPortCfg *pPortCfg, LogicalGroupCfg *pGroupCfg, LogicalGroupProc *pGroupProc);
int32_t QosQueueScheduler(PhysicalPortCfg *pPortCfg, QosQueueProc *pQueueProc);

#endif // QOS_C_MODEL_H
