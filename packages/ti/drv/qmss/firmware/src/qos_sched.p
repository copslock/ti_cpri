//
//  TEXAS INSTRUMENTS TEXT FILE LICENSE
// 
//   Copyright (c) 2016 Texas Instruments Incorporated
// 
//  All rights reserved not granted herein.
//  
//  Limited License.  
// 
//  Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive 
//  license under copyrights and patents it now or hereafter owns or controls to 
//  make, have made, use, import, offer to sell and sell ("Utilize") this software 
//  subject to the terms herein.  With respect to the foregoing patent license, 
//  such license is granted  solely to the extent that any such patent is necessary 
//  to Utilize the software alone.  The patent license shall not apply to any 
//  combinations which include this software, other than combinations with devices 
//  manufactured by or for TI (“TI Devices”).  No hardware patent is licensed hereunder.
// 
//  Redistributions must preserve existing copyright notices and reproduce this license 
//  (including the above copyright notice and the disclaimer and (if applicable) source 
//  code license limitations below) in the documentation and/or other materials provided 
//  with the distribution.
//  
//  Redistribution and use in binary form, without modification, are permitted provided 
//  that the following conditions are met:
// 	No reverse engineering, decompilation, or disassembly of this software is 
//   permitted with respect to any software provided in binary form.
// 	Any redistribution and use are licensed by TI for use only with TI Devices.
// 	Nothing shall obligate TI to provide you with source code for the software 
//   licensed and provided to you in object code.
//  
//  If software source code is provided to you, modification and redistribution of the 
//  source code are permitted provided that the following conditions are met:
// 	Any redistribution and use of the source code, including any resulting derivative 
//   works, are licensed by TI for use only with TI Devices.
// 	Any redistribution and use of any object code compiled from the source code
//   and any resulting derivative works, are licensed by TI for use only with TI Devices.
// 
//  Neither the name of Texas Instruments Incorporated nor the names of its suppliers 
//  may be used to endorse or promote products derived from this software without 
//  specific prior written permission.
// 
//  DISCLAIMER.
// 
//  THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
//  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
//  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S 
//  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
//  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
//  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 
//
//==================================================================================
// QoS Scheduler
//----------------------------------------------------------------------------------
//
// qos_sched.p - Quality of Service Firmware using weighted round robin
//
// Target: Nysh Queue Manager PDSP 2
//
// Description:
//
// This code implements the PDSP Quality of Service (QOS) firmware
//
// Copyright 2009 - 2015 by Texas Instruments Inc.
//----------------------------------------------------------------------------------
// Revision History:
//
// 29-Feb-12 : John Dowdal : Version 1.0.0.0 
// 06-Mar-12 : John Dowdal : Version 1.0.0.1 - fixed signed arith bugs & context save
//                                             in packet drop code.
// 08-Mar-12 : John Dowdal : Version 1.0.0.2 - qbgt vs qblt in pir code
// 14-Apr-12 : John Dowdal : Version 1.0.0.3 - increase lite ports to 10, and adjust
//                                             resolution of bytes and packet counts.
// 19-Apr-12 : John Dowdal : Version 1.0.0.4 - increase resolution of bytes to shift of 11
// 02-May-12 : John Dowdal : Version 1.0.0.5 - fix memory corruption when popping null from queue
//                                             which lead to "crash"; add "watchpoint"
//                                             code in #if DEBUG_WP for debugging this kind
//                                             of problem.
// 11-Jun-12 : John Dowdal : Version 1.0.0.6 - change to match updated spec/c code from
//                                             customer predominently wrr behavior
// 18-Jun-12 : John Dowdal : Version 1.0.0.7 - fix infinite loop caused by state.pirCreditMask
//                                             initialization in wrong palce
// 06-Aug-12 : John Dowdal : Version 1.0.0.8 - Keystone 2 compatibility
// 05-Oct-12 : John Dowdal : Version 1.0.1.0 - Drop Scheduler
// 15-Oct-12 : John Dowdal : Version 1.0.1.1 - Drop Scheduler Code Complete milestone
// 24-Oct-12 : John Dowdal : Version 1.0.1.2 - fix bugs
// 14-Nov-12 : John Dowdal : Version 1.0.1.3 - fix BE, active to shadow copy, new push stats
// 30-Nov-12 : John Dowdal : Version 1.0.1.4 - add push proxy due to broken hw, and remove
//                                             support for different tick rates
// 22-Feb-13 : John Dowdal : version 2.0.1.5 - keystone 2 build, fix drop empty queue bug
// 14-Oct-13 : John Dowdal : version 2.0.1.6 - add removeBytes to allow "negative" offsetBytes
// 01-Apr-14 : John Dowdal : version 2.0.1.7 - add joint lite ports
// 05-Sep-14 : John Dowdal : version 2.0.1.8 - add WRR_*_SCALE to enable 10000:1 ratios
// 14-Oct-14 : John Dowdal : version 2.0.1.9 - add wide (17 group) qos scheduler build
// 15-Jun-15 : John Dowdal : version 2.0.1.10 - add simu byte/pkt cir/pir for wide and regular build
// 17-Jul-15 : John Dowdal : version 2.0.1.11 - make pps work even with packet size = 0
// 20-Oct-15 : John Dowdal : version 2.0.1.12 - re-enable K1
// 22-Nov-16 : John Dowdal : version 2.0.1.13 - fix discard logic for split-mode return
//
// Reminder: set version number in HEADER_VERSION below.
//==================================================================================

#include "pm_config.h"


//
// Verify one and only one Endian mode flag is set _LE_ or _BE_
//
#ifndef _LE_
#ifndef _BE_
#error Must set either _LE_ or _BE_!
#endif
#endif
#ifdef _LE_
#ifdef _BE_
#error Must set either _LE_ or _BE_!
#endif
#endif

#define pushRet mov r30.w2, r30.w0
#define popRet  mov r30.w0, r30.w2

#define RAM_SIZE            0x2000      
#define cLocalRAM           cLRAM3
#define TIMER_IPS           100000      // 10us=100000
#define TIMER_SETTING       ((350000000/TIMER_IPS)/2)
#define PROFILEID           12

// Debug trigger.  This will stop in an infinite loop when a
// packet is transferred whose descriptor has DEBUG_TRIGGER_VAL
// at DEBUG_TRIGGER_OFFSET.  This should NOT be included in
// release code!!
// The idea is to configure the test program test_qosSched.c to
// reproduce a problem, then set DEBUG_TRIGGER_VAL to a descID
// that happens before the problem.  The pview can be started
// when the firmware spins in loop to allow debug of the problem.
//#define DEBUG_TRIGGER_ENABLE
#define DEBUG_TRIGGER_VAL    0x04600000
#define DEBUG_TRIGGER_OFFSET 4

// Wait two timer ticks on boot to store timer calibration to
// QOSSCHED_OFF_TCAL
//#define TIMER_CHECK_CAL
//

// Narrow/Wide Stats bitpacking in index
#ifdef WIDE
#define STATS_PORT_MASK    0x7
#define STATS_GROUP_SHIFT  3
#define STATS_GROUP_MASK   0x1f
#define STATS_QUEUE_SHIFT  8
#else
#define STATS_PORT_MASK    0x1f
#define STATS_GROUP_SHIFT  5
#define STATS_GROUP_MASK   0x7
#define STATS_QUEUE_SHIFT  8
#endif

//
// ENDIAN DEPENDENT STRUCTURES
//
// Structures that are shared with the host must be defined to
// represent the same byte ordering in the host, regardless of
// endian mode. Thus there is one set for Little Endian and
// one set for Big Endian.
//

// Qos Command
#ifdef _LE_
.struct struct_qoscmd
    .u8     command         // Command Number
    .u8     option          // Command Option
    .u16    index           // Index Parameter
    .u32    returnCode      // Return code
.ends
.struct struct_qoscmd_split_index
    .u8     command         // Command Number
    .u8     option          // Command Option
    .u8     index_lsb       // Index Parameter
    .u8     index_msb       // Index Parameter
    .u32    returnCode      // Return code
.ends
#else
.struct struct_qoscmd
    .u16    index           // Index Parameter
    .u8     option          // Command Option
    .u8     command         // Command Number
    .u32    returnCode      // Return code
.ends
.struct struct_qoscmd_split_index
    .u8     index_msb       // Index Parameter
    .u8     index_lsb       // Index Parameter
    .u8     option          // Command Option
    .u8     command         // Command Number
    .u32    returnCode      // Return code
.ends
#endif

// Stats results register
.struct struct_Stats
    .u32    bytesForwarded_lsw
    .u32    bytesForwarded_msw
    .u32    bytesDropped_lsw
    .u32    bytesDropped_msw
    .u32    packetsForwarded
    .u32    packetsDropped
.ends

// flags for group stats response in shadow
#ifndef DROP_SCHED
.struct struct_StatsRespFlags
#ifdef _LE_
    .u8    queueCount
    .u8    res1
    .u16   res2
#else
    .u16   res2
    .u8    res1
    .u8    queueCount
#endif
.ends
#endif

// Qos Queue Config
.struct struct_QueueCfgSize
    .u32    wrrInitialCredit
    .u32    congestionThresh
.ends
.struct struct_QueueCfgRegMap
    .u32    wrrInitialCredit
.ends

// Qos Group Config
#ifdef _LE_
// This structure used to step over each group config in memory (hence size) 
.struct struct_GroupCfgSize
#ifdef MULTIGROUP
    .u32    cirIteration
    .u32    pirIteration
    .u32    cirMax
    .u32    pirMax
    .u32    wrrInitialCredit
#else
#define GROUP_JUMP (5*4)
#endif
    .u8     queueCount
    .u8     SPCount
    .u8     RRCount
    .u8     flags
#ifdef SIMUBYTEPKT
    .u32    pktCirIteration
    .u32    pktPirIteration
    .u32    pktCirMax
    .u32    pktPirMax
#endif
.ends

// This structure used to map registers -- its subset of GroupCfgSize b/c of 
// runtime overlay of packet credits 
.struct struct_GroupCfgRegMap
#ifdef MULTIGROUP
    .u32    cirIteration
    .u32    pirIteration
    .u32    cirMax
    .u32    pirMax
    .u32    wrrInitialCredit
#else
#endif
    .u8     queueCount
    .u8     SPCount
    .u8     RRCount
    .u8     flags
.ends
#ifdef SIMUBYTEPKT
// This will be loaded on top of the byte cirIteration block above when needed 
.struct struct_GroupCfgRegPktOverlay
    .u32    pktCirIteration
    .u32    pktPirIteration
    .u32    pktCirMax
    .u32    pktPirMax
.ends
#endif

#else

.struct struct_GroupCfgSize
// This structure used to step over each group config in memory (hence size) 
#ifdef MULTIGROUP
    .u32    cirIteration
    .u32    pirIteration
    .u32    cirMax
    .u32    pirMax
    .u32    wrrInitialCredit
#else
#define GROUP_JUMP (5*4)
#endif
    .u8     flags
    .u8     RRCount
    .u8     SPCount
    .u8     queueCount
#ifdef SIMUBYTEPKT
    .u32    pktCirIteration
    .u32    pktPirIteration
    .u32    pktCirMax
    .u32    pktPirMax
#endif
.ends

.struct struct_GroupCfgRegMap
// This structure used to map registers -- its subset of GroupCfgSize b/c of 
// runtime overlay of packet credits 
#ifdef MULTIGROUP
    .u32    cirIteration
    .u32    pirIteration
    .u32    cirMax
    .u32    pirMax
    .u32    wrrInitialCredit
#else
#endif
    .u8     flags
    .u8     RRCount
    .u8     SPCount
    .u8     queueCount
.ends
#ifdef SIMUBYTEPKT
// This will be loaded on top of the byte cirIteration block above when needed 
.struct struct_GroupCfgRegPktOverlay
    .u32    pktCirIteration
    .u32    pktPirIteration
    .u32    pktCirMax
    .u32    pktPirMax
.ends
#endif

#endif

// Qos Port Config
// Offset of overlay inside PortCfgRegMap
#define PORTCFGPKTOV_OFFSET OFFSET(struct_PortCfgRegMap.cirIterationCredit) 
#ifdef _LE_
.struct struct_PortCfgSize
// This structure used to step over each group config in memory (hence size) 
    .u8     flags
    .u8     groupCount
    .u16    destQueueNumber
    .u8     overheadBytes
    .u8     removeBytes
    .u16    destThrottleThresh
    .u32    cirIterationCredit
    .u32    cirMax
#ifdef SIMUBYTEPKT
    .u32    pktCirIterationCredit
    .u32    pktCirMax
#endif
.ends

.struct struct_PortCfgRegMap
// This structure used to map registers -- its subset of PortCfgSize b/c of 
// runtime overlay of packet credits 
    .u8     flags
    .u8     groupCount
    .u16    destQueueNumber
    .u8     overheadBytes
    .u8     removeBytes
    .u16    destThrottleThresh
    .u32    cirIterationCredit
    .u32    cirMax
.ends
#ifdef SIMUBYTEPKT
.struct struct_PortCfgRegPktOverlay
// This will be loaded on top of the byte cirIteration block above when needed 
    .u32    pktCirIterationCredit
    .u32    pktCirMax
.ends
#endif

#else
.struct struct_PortCfgSize
// This structure used to step over each group config in memory (hence size) 
    .u16    destQueueNumber
    .u8     groupCount
    .u8     flags
    .u16    destThrottleThresh
    .u8     removeBytes
    .u8     overheadBytes
    .u32    cirIterationCredit
    .u32    cirMax
#ifdef SIMUBYTEPKT
    .u32    pktCirIterationCredit
    .u32    pktCirMax
#endif
.ends
.struct struct_PortCfgRegMap
// This structure used to map registers -- its subset of PortCfgSize b/c of 
// runtime overlay of packet credits 
    .u16    destQueueNumber
    .u8     groupCount
    .u8     flags
    .u16    destThrottleThresh
    .u8     removeBytes
    .u8     overheadBytes
    .u32    cirIterationCredit
    .u32    cirMax
.ends
#ifdef SIMUBYTEPKT
.struct struct_PortCfgRegPktOverlay
// This will be loaded on top of the byte cirIteration block above when needed 
    .u32    pktCirIterationCredit
    .u32    pktCirMax
.ends
#endif

#endif

#ifdef DROP_SCHED
// Drop Scheduler Top Level Config
#ifdef _LE_
.struct struct_DropCfg
    .u32    res
    .u32    rng_s1
    .u32    rng_s2
    .u32    rng_s3
.ends
.struct struct_DropCfgStatsQs
    .u16    dst1
    .u16    src1
    .u16    dst2
    .u16    src2
    .u16    dst3
    .u16    src3
    .u16    dst4
    .u16    src4
.ends
.struct struct_DropCfgStatsQPair
    .u16    dst
    .u16    src
.ends
#else
.struct struct_DropCfg
    .u32    res
    .u32    rng_s1
    .u32    rng_s2
    .u32    rng_s3
.ends
.struct struct_DropCfgStatsQs
    .u16    src1
    .u16    dst1
    .u16    src2
    .u16    dst2
    .u16    src3
    .u16    dst3
    .u16    src4
    .u16    dst4
.ends
.struct struct_DropCfgStatsQPair
    .u16    src
    .u16    dst
.ends
#endif

// Drop Scheduler Config Profile
#ifdef _LE_
.struct struct_DropCfgProf
    .u8     unitFlags
    .u8     mode
    .u8     tcShift
    .u8     res1
    .u32    TailThresh
    .u32    REDlow
    .u32    REDhigh
    .u32    threshRecip
.ends
#else
.struct struct_DropCfgProf
    .u8     res1
    .u8     tcShift
    .u8     mode
    .u8     unitFlags
    .u32    TailThresh
    .u32    REDlow
    .u32    REDhigh
    .u32    threshRecip
.ends
#endif
.macro calcDropCfgProfAddr
.mparam outReg, inReg, scratchReg
     ldi    scratchReg.w0, QOSSCHED_DROPSCHED_OFF_CFG_PROFS
     lsl    scratchReg.w2, inReg, 4 // *16
     lsl    outReg, inReg, 2 // *4
     add    outReg, scratchReg.w2, outReg // *20
     add    outReg, scratchReg.w0, outReg // + base
.endm

// Drop Scheduler Output Profile
#ifdef _LE_
.struct struct_DropOutProf
    .u16    destQueueNumber
    .u16    REDProb
    .u8     cfgProfIdx
    .u8     fEnabled
    .u16    res1
    .u32    QAvg
.ends
#else
.struct struct_DropOutProf
    .u16    REDProb
    .u16    destQueueNumber
    .u16    res1
    .u8     fEnabled
    .u8     cfgProfIdx
    .u32    QAvg
.ends
#endif
.macro calcDropOutProfAddr
.mparam outReg, inReg, scratchReg
     ldi    scratchReg.w0, QOSSCHED_DROPSCHED_OFF_OUT_PROFS
     lsl    scratchReg.w2, inReg, 3 // *8
     lsl    outReg, inReg, 2 // *4
     add    outReg, scratchReg.w2, outReg // *12
     add    outReg, scratchReg.w0, outReg // + base
.endm

// Drop Scheduler Input Queue Profile
#ifdef _LE_
.struct struct_DropQueProf
    .u8     outProfIdx
    .u8     statsIdx
    .u8     statsQProf
    .u8     fEnabled
.ends
#else
// Drop Scheduler Input Queue Profile
.struct struct_DropQueProf
    .u8     fEnabled
    .u8     statsQProf
    .u8     statsIdx
    .u8     outProfIdx
.ends
#endif
.macro calcDropQueueProfAddr
.mparam outReg, inReg, scratchReg
     ldi    scratchReg, QOSSCHED_DROPSCHED_OFF_QUE_PROFS
     lsl    outReg, inReg, 2
     add    outReg, scratchReg, outReg
.endm

// Drop Scheduler Push Stats
#ifdef _LE_
.struct struct_PushStats
     .u8    statsIdx
     .u8    size
     .u16   res
     .u32   bytesForwarded
     .u32   bytesDropped
     .u32   packetsForwarded
     .u32   packetsDropped
.ends
#else
// Drop Scheduler Push Stats
.struct struct_PushStats
     .u16   res
     .u8    size
     .u8    statsIdx
     .u32   bytesForwarded
     .u32   bytesDropped
     .u32   packetsForwarded
     .u32   packetsDropped
.ends
#endif

// Drop Scheduler Push Proxy (to workaround Si bugs)
#ifdef _LE_
.struct struct_PushProxy
     .u16   descSize
     .u16   qNum
     .u32   descPtr
.ends
#else
// Drop Scheduler Push Proxy (to workaround Si bugs)
.struct struct_PushProxy
     .u16   qNum
     .u16   descSize
     .u32   descPtr
.ends
#endif

#endif // DROP_SCHED 

// Bit values for struct_PortCfg.flags
#define PORT_FLAGS_BIT_WRR_BYTES       0
#define PORT_FLAGS_BIT_CIR_BYTES       1
#define PORT_FLAGS_BIT_CONG_BYTES      2
#define PORT_FLAGS_BIT_OUT_THROT_BYTES 3
#define PORT_FLAGS_BIT_IS_JOINT        4
#ifdef SIMUBYTEPKT
#define PORT_FLAGS_BIT_CIR_BY_BYTES    1
#define PORT_FLAGS_BIT_CIR_BY_PKTS     5
#define GROUP_FLAGS_BIT_CIR_BY_BYTES   5
#define GROUP_FLAGS_BIT_CIR_BY_PKTS    6
#endif

#define DROP_UNIT_FLAGS_BIT_TAILDROP_BYTES 0

// config profile modes
#define DROP_MODE_TAIL_ONLY 0
#define DROP_MODE_RED       1
#define DROP_MODE_REM       2
//
// ENDIAN AGNOSTIC STRUCTURES
//
// Structures that are private to the PDSPs can be allowed to
// alter their byte ordering according to endian mode. This is
// done automatically by the assembler.
//

// Overlay with struct_static2, to define que base, stack ptr, and timer tick
.struct struct_static1
    .u16    queueBase                 // Base QoS Queue
    .u16    stackPtr                  // Stack pointer

// The following overlay stateAndPortEnable in struct_static2
// This allows 24 bits of state bits (instead of 16 or 32 as pasm struct would otherwise allow)
#ifdef _LE_
    .u16    res2
    .u8     res1
    .u8     timerTicks                // Software timer incremented each time HW timer expires
#else
    .u8     timerTicks                // Software timer incremented each time HW timer expires
    .u8     res1
    .u16    res2
#endif
.ends

// Overlay struct_static1, to define 24 bits of port state
.struct struct_static2
    .u32    res1
#define STATIC_STATE_BIT_CFG_CHANGED        23
#ifdef DROP_SCHED
#define STATIC_STATE_BIT_DROP_SCHED_ENABLE  22
#endif
    .u32    stateAndPortEnable        // Enable bits for 22 ports plus 2 state bit
                                      // upper 8 bits overlay timerTicks above
.ends

.struct struct_QueueProc
    .u32    wrrCurrent            // WRR credit accumulator (signed) - MUST be first
    .u32    bytesForwardedMSW
    .u32    bytesForwardedLSW
    .u32    bytesDroppedMSW 
    .u32    bytesDroppedLSW
    .u32    packetsForwarded
    .u32    packetsDropped
.ends

.struct struct_QueueProcLite
    .u32    wrrCurrent            // WRR credit accumulator (signed)
.ends

.struct struct_GroupProcSize
#ifdef MULTIGROUP
    .u32    cirCurrent            // CIR credit accumulator (signed)         (r17)
    .u32    pirCurrent            // WRR credit accumulator (signed)         (r18)
    .u32    wrrCurrent            // WRR credit accumulator (signed)         (r19)
#endif
    .u8     nextQueue             // Next queue to be looked at              (r20.b0)
    .u8     wrrCreditMask         // Which queues have WRR credit (bitfield) (r20.b1)
    .u16    res
#ifdef MULTIGROUP
#ifdef SIMUBYTEPKT
    .u32    pktCirCurrent
    .u32    pktPirCurrent
#endif
#endif
.ends

.struct struct_GroupProcRegMap
#ifdef MULTIGROUP
    .u32    cirCurrent            // CIR credit accumulator (signed)         (r17)
    .u32    pirCurrent            // WRR credit accumulator (signed)         (r18)
    .u32    wrrCurrent            // WRR credit accumulator (signed)         (r19)
#endif
    .u8     nextQueue             // Next queue to be looked at              (r20.b0)
    .u8     wrrCreditMask         // Which queues have WRR credit (bitfield) (r20.b1)
    .u16    res
.ends

#ifdef MULTIGROUP
#ifdef SIMUBYTEPKT
// Overlays cirCurrent and pirCurrent for simutaneous packets & bytes (swapped)
.struct struct_GroupProcRegPktOverlay
    .u32    pktCirCurrent
    .u32    pktPirCurrent
.ends
#endif
#endif


.struct struct_PortProc
#ifdef WIDE
    .u32    cirCurrent             // CIR credit accumulator (signed)                         (r13)
    .u32    wrrCreditMask          // Bitfield representing which groups have wrr credit      (r14)
    .u16    portProcSize           // Size of processing/dynamic portion of this port         (r15.w0)
    .u8     lastTimerTicks         // last value of static.timerTicks when port ran           (r15.b2)
    .u8     groupCfgSize           // Size of config/static portion of one group on port      (r15.b3)
    .u8     groupProcSize          // Size of processing/dynamic portion of one group on port (r16.b0)
    .u8     nextGroup              // Round robin group index                                 (r16.b1)
    .u8     groupMaxQueues         // queues supported in this group                          (r16.b2)
    .u8     portMaxQueues          // queues supported in this port                           (r16.b3)
#else
    .u32    cirCurrent             // CIR credit accumulator (signed)                         (r13)
#ifdef SIMUBYTEPKT
    .u32    pktCirCurrent
#endif
    .u16    portCfgSize            // Size of config/static portion of this port              (r14.w0)
    .u16    portProcSize           // Size of processing/dynamic portion of this port         (r14.w2)
    .u8     lastTimerTicks         // last value of static.timerTicks when port ran           (r15.b0)
    .u8     res                    //                                                         (r15.b1)
    .u8     groupCfgSize           // Size of config/static portion of one group on port      (r15.b2)
    .u8     groupProcSize          // Size of processing/dynamic portion of one group on port (r15.b3)
    .u8     wrrCreditMask          // Bitfield representing which groups have wrr credit      (r16.b0)
    .u8     nextGroup              // Round robin group index                                 (r16.b1)
    .u8     groupMaxQueues         // queues supported in this group                          (r16.b2)
    .u8     portMaxQueues          // queues supported in this port                           (r16.b3)
#endif
.ends

.struct struct_Desc
    .u32    descInfo    // 31:30 DescType: HOST(0), MONO(2)
    .u32    srcDstTag
    .u32    pktInfo     //  15   Ret Policy: Linked(0), Unlinked(1) (HOST)
                        //  14   Ret Push Policy: Tail(0), Head(1)  (HOST)
                        // 13:12 Ret QM 
                        // 11: 0 Ret Queue
    .u32    dataLength  // HOST mode only
    .u32    dataPtr     // HOST mode only
    .u32    nextPtr     // HOST mode only
.ends    

.struct struct_Enable
    .u16    size        // Size to zero
    .u16    dstProc     // Address to zero
    .u16    dstCfg      // corresponding config address
    .u16    stepProc    // proc step size
    .u16    stepCfg     // cfg step size
    .u16    offProc     // proc offset
    .u16    offCfg      // cfg offset
    .u8     portNum     // port counter
    .u8     secPort     // second port cleared
    .u32    scratch     // Scratch containing zeros
.ends

.struct struct_Disable
    .u16    i           // loop counter
    .u16    res
.ends

.struct struct_Copy
    .u16    size        // Size to copy 
    .u16    jumpOffset  // offset where to jump over group
    .u16    srcAddr     // Souce address
    .u16    dstAddr     // Destination address
    .u32    scratch     // Scratch to perform copy
.ends

.struct struct_LoopCounters
#ifndef WIDE
    .u16    portCfgOffset    // offset into scratch for ports static config (r4.w0)
    .u16    portProcOffset   // offset into scratch for ports dynamic state (r4.w2)
#endif
    .u16    groupCfgOffset   // Offset counter for groups static config     (r5.w0)
    .u16    groupProcOffset  // Offset counter for groups dynamic state     (r5.w2)
    .u16    queueCfgOffset   // Offset counter for queues static config     (r6.w0)
    .u16    queueProcOffset  // Offset counter for queues dynamic state     (r6.w2)
    .u16    queueNumber      // Computed queue number                       (r7.w0)
    .u8     portIndex        // loop counter to go through ports            (r7.b2)
    .u8     groupIndex       // loop counter to go through groups in port   (r7.b3)
    .u8     queueIndex       // loop counter to go through queues in group  (r8.b0)
    .u8     portQBaseIndex   // total queues consumed by previous ports     (r8.b1)
    .u8     groupQBaseIndex  // total queues consumed by previous groups    (r8.b2)
    .u8     res1             //                                             (r8.b3)
.ends

#ifdef WIDE
.macro setFPacketsSent
    set     state.groupPktPendMask, state.groupPktPendMask, 31
.endm
.macro clrFPacketsSent
    clr     state.groupPktPendMask, state.groupPktPendMask, 31
.endm
.macro testFPacketsSentTrue
.mparam label
    qbbs    label, state.groupPktPendMask, 31
.endm
#else
.macro setFPacketsSent
    set     state.flags, state.flags, 0
.endm
.macro clrFPacketsSent
    clr     state.flags, state.flags, 0
.endm
.macro testFPacketsSentTrue
.mparam label
    qbbs    label, state.flags, 0
.endm
#ifdef SIMUBYTEPKT
.macro setFPortCreditAvail
    set     state.flags, state.flags, 1
.endm
.macro clrFPortCreditAvail
    clr     state.flags, state.flags, 1
.endm
.macro testFPortCreditNotAvail
.mparam label
    qbbc    label, state.flags, 1
.endm
.macro setFGroupPirCreditAvail
    set     state.flags, state.flags, 2
.endm
.macro clrFGroupPirCreditAvail
    clr     state.flags, state.flags, 2
.endm
.macro testFGroupPirCreditAvail
.mparam label
    qbbs    label, state.flags, 2
.endm
.macro setFPirCreditOnly
    set     state.flags, state.flags, 3
.endm
.macro clrFPirCreditOnly
    clr     state.flags, state.flags, 3
.endm
.macro testFPirCreditOnly
.mparam label
    qbbs    label, state.flags, 3
.endm

#endif
#endif

.struct struct_PortState
#ifdef WIDE
    .u32    pirCreditMask    // Group has PIR credit
    .u32    groupPktPendMask // group has packet pending and fPacketsSent
#else
    .u8     pirCreditMask    // Group has PIR credit
    .u8     groupPktPendMask // group has packet pending
    .u8     flags            // bit 0: fPacketsSent; bit 1: fPortCreditAvail
                             // bit 2: fChargeWRR
    .u8     res
#endif
.ends

.struct struct_DiscardState
    .u32    packetSize
    .u32    packetDescPtr
    .u32    savedSize
.ends

.struct struct_GroupSched
    .u8    spPlusRR
    .u8    i
    .u8    j
    .u8    packetPendingMask
.ends

.struct struct_StatsReqR0
    .u8    port
    .u8    group
    .u8    queue
    .u8    option
.ends

.struct struct_QueueDebug
    .u32   descAddr
    .u32   descReadVal
    .u32   descCheckVal
.ends

#ifdef DROP_SCHED
.struct struct_DropStats
     .u32   bytesForwarded
     .u32   packetsForwarded
     .u32   bytesDropped
     .u32   packetsDropped
.ends
// Calculate offset by multiply inReg by size (16)
.macro calcDropStatsAddr
.mparam outReg, inReg, scratchReg
     ldi    scratchReg.w0, QOSSCHED_DROPSCHED_OFF_STATS_BLOCKS
     lsl    outReg, inReg, 4 // *16
     add    outReg, scratchReg.w0, outReg // + base
.endm

.struct struct_DropProc
     .u16   QueueBase
     .u8    randPos
     .u8    res1
     .u32   lastRand
.ends
#endif

#ifdef MULTIGROUP
#define GROUP_CFG_SIZE_FULL  (SIZE(struct_GroupCfgSize) + (SIZE(struct_QueueCfgSize) * QOS_SCHED_FULL_QUEUES))
#define GROUP_PROC_SIZE_FULL (SIZE(struct_GroupProcSize) + (SIZE(struct_QueueProc) * QOS_SCHED_FULL_QUEUES))
#define PORT_CFG_SIZE_FULL SIZE(struct_PortCfgSize) + (GROUP_CFG_SIZE_FULL * QOS_SCHED_FULL_GROUPS)
#define PORT_PROC_SIZE_FULL SIZE(struct_PortProc) + (GROUP_PROC_SIZE_FULL * QOS_SCHED_FULL_GROUPS)
#else
#define GROUP_CFG_SIZE_FULL 0
#define GROUP_PROC_SIZE_FULL 0
#define PORT_CFG_SIZE_FULL 0
#define PORT_PROC_SIZE_FULL 0
#endif

#define GROUP_CFG_SIZE_LITE  (SIZE(struct_GroupCfgSize) + (SIZE(struct_QueueCfgSize) * QOS_SCHED_LITE_QUEUES))
#define GROUP_PROC_SIZE_LITE (SIZE(struct_GroupProcSize) + (SIZE(struct_QueueProc) * QOS_SCHED_LITE_QUEUES))
#define PORT_CFG_SIZE_LITE SIZE(struct_PortCfgSize) + (GROUP_CFG_SIZE_LITE * QOS_SCHED_LITE_GROUPS)
#define PORT_PROC_SIZE_LITE SIZE(struct_PortProc) + (GROUP_PROC_SIZE_LITE * QOS_SCHED_LITE_GROUPS)

#define PORT_CFG_TOT_SIZE (PORT_CFG_SIZE_FULL * QOS_SCHED_FULL_PORTS) + (PORT_CFG_SIZE_LITE * QOS_SCHED_LITE_PORTS)
#define PORT_PROC_TOT_SIZE (PORT_PROC_SIZE_FULL * QOS_SCHED_FULL_PORTS) + (PORT_PROC_SIZE_LITE * QOS_SCHED_LITE_PORTS)

#ifdef DROP_SCHED
#define DROPSCHED_SIZE_OUT_PROFS    (SIZE(struct_DropOutProf) * QOSSCHED_DROPSCHED_NUM_OUT_PROFS)
#define DROPSCHED_SIZE_CFG_PROFS    (SIZE(struct_DropCfgProf) * QOSSCHED_DROPSCHED_NUM_CFG_PROFS)
#define DROPSCHED_SIZE_QUE_PROFS    (SIZE(struct_DropQueProf) * QOSSCHED_DROPSCHED_NUM_QUE_PROFS)
#define DROPSCHED_SIZE_STATS_BLOCKS (SIZE(struct_DropStats) * QOSSCHED_DROPSCHED_NUM_STATS_BLOCKS)
#define DROPSCHED_SIZE_CFG          (SIZE(struct_DropCfg) + SIZE(struct_DropCfgStatsQs))
#define DROPSCHED_SIZE_PUSH_STATS   (SIZE(struct_PushStats))
#endif

//-------------------------------------------------------------------
// REGISTER ALLOCATION 
//
// Regs R0 and R1 are temporary scratch
//
// R30 is reserved for subroutine calls
// 
// Regs R2 through R29 are allocated below
//
// Note that register allocations are overlapped according to how
// and when they are used. Stuctures that are unions or partial 
// unions must maintain the same relative addressing range as their
// peers. These restrictions are noted in the comments below. Register
// allocations in the same "use zone" are not allowed to overlap, nor
// can the static allocations overlap any other allocation.
//

// Static allocations are always valid
.assign struct_static1,                     R2,  R3, static
.assign struct_static2,                     R2,  R3, static2

// Register map for command processing
.enter qosCmd
.assign struct_qoscmd,                      R4,  R5, qoscmd        
.assign struct_qoscmd_split_index,          R4,  R5, qoscmd_split
#ifdef DROP_SCHED
.assign struct_DropProc,                    R6,  R7, dropProc
#endif
.leave qosCmd

#ifdef DROP_SCHED
// Register map for stats request in drop scheduler
.enter dropSchedStatsReqScope
.assign struct_Stats,                       R9,  R14, statsResp
.assign struct_DropStats,                   R15,  R18, dropStats
.leave dropSchedStatsReqScope
#endif

// Register map for memory (shadow) copy
.enter shadCopy
.assign struct_Copy,                        R6,  R8, shadcopy
.leave shadCopy

// Register map for port enable
.enter portEnableScope
.assign struct_Enable,                      R6,  R10, portSet
#ifdef SIMUBYTEPKT
.assign struct_PortProc,                    R11, R15, portProc
.assign struct_PortCfgSize,                 R16, R21, portCfg
#else
.assign struct_PortProc,                    R11, R14, portProc
.assign struct_PortCfgSize,                 R15, R18, portCfg
#endif
.leave portEnableScope

.enter portDisableScope
.assign struct_Disable,                     R29, R29, disable  // Overlay struct_PortState
.leave portDisableScope

// Register map for both foreground and background processing
.enter portProcessing
#ifdef MULTIGROUP
#ifdef WIDE
   .assign struct_LoopCounters,             R4, R7, counters
   .assign struct_PortCfgSize,              R8, R11, portCfg
   .assign struct_PortProc,                 R12, R15, portProc
   .assign struct_GroupProcSize,            R16, R19, groupProc
   .assign struct_QueueProc,                R12, R18, queueProc // queueProc intentionaly overlays portProc and groupProc
   .assign struct_GroupCfgSize,             R20, R25, groupCfg
   .assign struct_QueueCfgRegMap,           R26, R26, queueCfg
   .assign struct_QueueCfgSize,             R26, R27, queueCfgBgDrop
   .assign struct_PortState,                R27, R28, state
#else
   .assign struct_LoopCounters,             R4, R8, counters
   .assign struct_PortCfgRegMap,            R9, R12, portCfg
   .assign struct_PortCfgRegPktOverlay,     R11, R12, portCfgPktOv // overlays last 2 words of portCfg
   .assign struct_PortProc,                 R13, R17, portProc
   .assign struct_GroupProcRegMap,          R18, R21, groupProc
   .assign struct_GroupProcRegPktOverlay,   R18, R19, groupProcPktOv // overlays first 2 words of groupProc
   .assign struct_QueueProc,                R13, R19, queueProc // queueProc intentionaly overlays portProc and groupProc
   .assign struct_GroupCfgRegMap,           R22, R27, groupCfg
   .assign struct_GroupCfgRegPktOverlay,    R22, R25, groupCfgPktOv // overlays first 4 words of groupCfg
   .assign struct_QueueCfgRegMap,           R28, R28, queueCfg
// since bg drop doesn't use state, overlay it in reg 29
   .assign struct_QueueCfgSize,             R28, R29, queueCfgBgDrop // overlays state
   .assign struct_PortState,                R29, R29, state
#endif
#else
   .assign struct_LoopCounters,             R4, R8, counters
   .assign struct_PortCfgSize,              R9, R12, portCfg
   .assign struct_PortProc,                 R13, R16, portProc
   .assign struct_GroupProcSize,            R17, R17, groupProc
   .assign struct_QueueProc,                R13, R19, queueProc // queueProc intentionaly overlays portProc and groupProc
   .assign struct_GroupCfgSize,             R20, R20, groupCfg
   .assign struct_QueueCfgRegMap,           R27, R27, queueCfg
   .assign struct_QueueCfgSize,             R27, R28, queueCfgBgDrop
   .assign struct_PortState,                R28, R28, state
#endif
.leave portProcessing

.enter queueDebugScope
   .assign struct_QueueDebug,               R20, R22, queueDebug // Must immediately follow queueProc above
.leave queueDebugScope

// Register map for stats request uses portProcessing
.enter statsReqScope
   .assign struct_Stats,                    R20, R25, statsResp   // Intentional overlay groupProc and groupCfg
#ifndef DROP_SCHED
   .assign struct_StatsRespFlags,           R20, R20, statsRespFlags
#endif
   .assign struct_StatsReqR0,               R0, R0, r0s // name r0 fields 
.leave statsReqScope

// Register map for packet discard
.enter discardPacketScope
   .assign struct_DiscardState,             R4, R6, state
   .assign struct_Desc,                     R7, R12, desc
.leave discardPacketScope

.enter groupSchedScope
   // uses portProcessing plus the following.  This overlays portProc
   .assign struct_GroupSched,               R13, R13, groupSched
   .assign struct_QueueProcLite,            R14, R14, queueProcLite
.leave groupSchedScope

#ifdef DROP_SCHED
// Register map for drop scheduler
.enter dropSchedProcessing
   .assign struct_DropCfg,                  r4, r7, dropCfg
   .assign struct_DropProc,                 r8, r9, dropProc
.leave dropSchedProcessing
#endif

//-------------------------------------------------------------------
//
// Code Starts
//
.macro push
.mparam reg, size
        sbco    reg, cLocalRAM, static.stackPtr, size
        add     static.stackPtr, static.stackPtr, size
.endm

.macro pop
.mparam reg, size
        sub     static.stackPtr, static.stackPtr, size
        lbco    reg, cLocalRAM, static.stackPtr, size
.endm

.macro popDiscard
.mparam size
        sub     static.stackPtr, static.stackPtr, size
.endm

.macro ldi32
.mparam reg, val
        ldi     reg.w0, val & 0xffff
        ldi     reg.w2, val >> 16
.endm
        

        .entrypoint entry
        .origin     0
entry:
        jmp     start
header:
#ifdef MULTIGROUP
#ifdef WIDE
#define MAGIC_BUILD 0x80400000
#else
#define MAGIC_BUILD 0x80100000
#endif
#else
#define MAGIC_BUILD 0x80200000
#endif
#ifdef _LE_
#define HEADER_MAGIC   MAGIC_BUILD                   // "QOS" scheduler-0
#else
#define HEADER_MAGIC   (MAGIC_BUILD + 1)             // "QOS" scheduler-1
#endif
#define HEADER_VERSION 0x0200010D                    // 0x02.0x00.0x01.0x0D
        .codeword  HEADER_MAGIC
        .codeword  HEADER_VERSION
start:
        //--------------------------------------------------------------
        //
        // Initialization
        //

        // Timer Init
        ldi     r0, 0
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        ldi     r0, TIMER_SETTING                   
        sbco    r0, cTimer, TIMER_OFF_LOAD, 4
        ldi     r0, 1<<15 | 0b0000<<2 | 0b11        // Enable | /2 | Mode+Go
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        // Zero out our memory
        mov     r0, RAM_SIZE
        mov     r1, 0
zeroLoop:        
        sub     r0, r0, 4
        sbco    r1, cLocalRAM, r0, 4
        qbne    zeroLoop, r0, 0

        // Save the version key to scratch
        // This allows host cores to check version of preloaded firmware
        // matches the LLD, since the host cores cant read program when running
        ldi32   r1, HEADER_MAGIC
        ldi32   r2, HEADER_VERSION
        ldi     r0, QOSSCHED_OFF_VERSION
        sbco    r1, cLocalRAM, r0, 8
      

#ifdef TIMER_CHECK_CAL
        // This is used to check the speed of timer, for calibrating sim
        ldi     r0, 0
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
timerCheckWait1:
        qbbc    timerCheckWait1, r31.tStatus_Timer
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
timerCheckWait2:
        add     r0, r0, 1
        qbbc    timerCheckWait2, r31.tStatus_Timer
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
        ldi     r1, QOSSCHED_OFF_TCAL
        sbco    r0, cLocalRAM, r1, 4
#endif

#ifdef DROP_SCHED
        // Seed the RNG
.using dropSchedProcessing
        ldi     r0, 0xfee1
        ldi     r1, 0xdead
        ldi     r2, 0xbeef
        ldi     r3, QOSSCHED_DROPSCHED_OFF_CFG + OFFSET(dropCfg.rng_s1)
        sbco    r0, cLocalRAM, r3, 12
.leave dropSchedProcessing
#endif

        // Init the static variables        
        zero    &static, SIZE(static)

        // Set up the stack
        ldi     static.stackPtr, QOSSCHED_OFF_STACK_BASE

#ifdef TIMER_CHECK_CAL
        // This is used to check the speed of timer, for calibrating sim
        ldi     r0, 0
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
timerCheckWait1:
        qbbc    timerCheckWait1, r31.tStatus_Timer
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
timerCheckWait2:
        add     r0, r0, 1
        qbbc    timerCheckWait2, r31.tStatus_Timer
        set     r31.tStatus_Timer                       // Clear the timer trigger                   
        ldi     r1, QOSSCHED_OFF_TCAL
        sbco    r0, cLocalRAM, r1, 4
#endif
        // Configure the port size parameters for each port
sizeConfig:
.using portProcessing
        ldi     counters.portIndex, 0
#ifndef WIDE
        ldi     counters.portProcOffset, QOSSCHED_OFF_PORT_PROCS
#endif
        zero    &portProc, SIZE(portProc)
#ifdef MULTIGROUP
#ifndef WIDE
        ldi     portProc.portCfgSize, PORT_CFG_SIZE_FULL
#endif
        ldi     portProc.portProcSize, PORT_PROC_SIZE_FULL
        ldi     portProc.groupCfgSize, GROUP_CFG_SIZE_FULL
        ldi     portProc.groupProcSize, GROUP_PROC_SIZE_FULL
        ldi     portProc.groupMaxQueues, QOS_SCHED_FULL_QUEUES
        ldi     portProc.portMaxQueues, QOS_SCHED_FULL_QUEUES * QOS_SCHED_FULL_GROUPS
#else
        ldi     portProc.portCfgSize, PORT_CFG_SIZE_LITE
        ldi     portProc.portProcSize, PORT_PROC_SIZE_LITE
        ldi     portProc.groupCfgSize, GROUP_CFG_SIZE_LITE
        ldi     portProc.groupProcSize, GROUP_PROC_SIZE_LITE
        ldi     portProc.groupMaxQueues, QOS_SCHED_LITE_QUEUES
        ldi     portProc.portMaxQueues, QOS_SCHED_LITE_QUEUES * QOS_SCHED_LITE_GROUPS
#endif
sizeConfigLoop:
#ifdef WIDE
        ldi     r0, QOSSCHED_OFF_PORT_PROCS
        sbco    portProc, cLocalRAM, r0, SIZE(portProc)
#else
        sbco    portProc, cLocalRAM, counters.portProcOffset, SIZE(portProc)
        add     counters.portProcOffset, counters.portProcOffset, portProc.portProcSize
        add     counters.portIndex, counters.portIndex, 1
        qbeq    sizeConfigLoopDone, counters.portIndex, QOS_SCHED_PORTS
#ifdef MULTIGROUP
        qbne    sizeConfigLoop, counters.portIndex, QOS_SCHED_FULL_PORTS
        ldi     portProc.portCfgSize, PORT_CFG_SIZE_LITE
        ldi     portProc.portProcSize, PORT_PROC_SIZE_LITE
        ldi     portProc.groupCfgSize, GROUP_CFG_SIZE_LITE
        ldi     portProc.groupProcSize, GROUP_PROC_SIZE_LITE
        ldi     portProc.groupMaxQueues, QOS_SCHED_LITE_QUEUES
        ldi     portProc.portMaxQueues, QOS_SCHED_LITE_QUEUES * QOS_SCHED_LITE_GROUPS
#endif
        jmp     sizeConfigLoop
#endif 
sizeConfigLoopDone:
.leave portProcessing

#ifdef _MODEL_
        //-------------------------------
        // Test Code for Profiling
        jmp     fakeMainLoop
        // Test Code for Profiling
        //-------------------------------
#endif        

        // fresh start the bg process
        set     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_CFG_CHANGED
        //--------------------------------------------------------------
        //
        // Background Processing Loop
        //
.using portProcessing
bgTaskRestore:
        // save timer tick and increment seqn for external load measurement
        ldi     r0, QOSSCHED_OFF_TIMER_TICK
        lbco    r1, cTimer, TIMER_OFF_COUNT, 4
        lbco    r4, cLocalRAM, r0.w0, 4
        add     r1.w2, r4.w2, 1
        sbco    r1, cLocalRAM, r0.w0, 4
        // Restore the background context from scratch (or load 0s on startup)
        ldi     r0, QOSSCHED_OFF_BACKGROUND_CONTEXT
        lbco    counters, cLocalRAM, r0, SIZE(counters)

        qbbc    bgNoConfigChange, static2.stateAndPortEnable, STATIC_STATE_BIT_CFG_CHANGED
        clr     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_CFG_CHANGED
        jmp     bgPortReset
        
bgNoConfigChange:
        // Reload the port config which wasnt saved
#ifdef WIDE
        ldi     r0, QOSSCHED_OFF_PORT_CFGS
        lbco    portCfg, cLocalRAM, r0, SIZE(portCfg)
        ldi     r0, QOSSCHED_OFF_PORT_PROCS
        lbco    portProc, cLocalRAM, r0, SIZE(portProc)
#else
        lbco    portCfg, cLocalRAM, counters.portCfgOffset, SIZE(portCfg)
        lbco    portProc, cLocalRAM, counters.portProcOffset, SIZE(portProc)
#endif
        lbco    groupCfg, cLocalRAM, counters.groupCfgOffset, SIZE(groupCfg)
        lbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
        lbco    queueCfg, cLocalRAM, counters.queueCfgOffset, SIZE(queueCfg)
bgTaskLoop:
#ifdef DROP_SCHED
        call    dropSchedCheckPushProxy
#endif
        // If the current port is disabled, simply move to next port
        qbbc    bgNeedNextPort, static2.stateAndPortEnable, counters.portIndex
        // Load threshold, not normally loaded by moveNextQueue since it overlays state
        add     r0, counters.queueCfgOffset, SIZE(queueCfg)
        lbco    queueCfgBgDrop.congestionThresh, cLocalRAM, r0, SIZE(queueCfgBgDrop.congestionThresh)
        // Is threshold configured for this queue?
        qbeq    bgMoveNextQueue, queueCfgBgDrop.congestionThresh, 0
        
        // Compute absolute queue number
        add     counters.queueNumber, static.queueBase, counters.portQBaseIndex
        add     counters.queueNumber, counters.queueNumber, counters.groupQBaseIndex
        add     counters.queueNumber, counters.queueNumber, counters.queueIndex
       
bgDrainQLoop:
        // Peek the packet and byte count for the queue
        lsl     r0, counters.queueNumber, 4   // *16 to get offset
        lbco    r0, cQPeekBase, r0, 8

        qbbs    bgUseBytes, portCfg.flags, PORT_FLAGS_BIT_CONG_BYTES
        qbge    bgMoveNextQueue, r0, queueCfgBgDrop.congestionThresh   // no discard needed (count in packets)
        jmp     bgDoDiscard
bgUseBytes:
        qbge    bgMoveNextQueue, r1, queueCfgBgDrop.congestionThresh   // no discard needed (count in bytes)
bgDoDiscard:
        mov     r0, counters.queueNumber
        call    discardPacket
        // queueProc is overlaid in register file.  Save registers, then load queueProc
        // therefore cant use portProc and groupProc at same time
        push    queueProc, SIZE(queueProc)
        lbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)
        // Update stats in queueProc
        add     queueProc.bytesDroppedLSW, queueProc.bytesDroppedLSW, r0
        adc     queueProc.bytesDroppedMSW, queueProc.bytesDroppedMSW, 0
        add     queueProc.packetsDropped, queueProc.packetsDropped, 1
        // Store queueProc
        sbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)
        // Restore overlay
        pop     queueProc, SIZE(queueProc)
        jmp     bgDrainQLoop

bgMoveNextQueue:
        // Try next queue in same group
        call    moveQueueState
        qbgt    bgCheckTimer, counters.queueIndex, groupCfg.queueCount

#ifdef MULTIGROUP
        // Need to move to next group
        call    moveGroupState
        // Is there next group?
        qble    bgNeedNextPort, counters.groupIndex, portCfg.groupCount
        call    resetQueueState
        jmp     bgCheckTimer
#endif

        // Need to move to next port
bgNeedNextPort:
        call    movePortState
        qbgt    bgNoPortRollover, counters.portIndex, QOS_SCHED_PORTS
bgPortReset:
        call    resetPortState   // go back to port 0
bgNoPortRollover:
        qbbc    bgMoveNextPortDisabled, static2.stateAndPortEnable, counters.portIndex
        call    resetGroupState
        call    resetQueueState
bgMoveNextPortDisabled:

bgCheckTimer:

#ifdef _MODEL_
        //-------------------------------
        // Test Code for Profiling
        mov     r0.w0, PRCMD_BASE & 0xFFFF
        mov     r0.w2, PRCMD_BASE >> 16
        mov     r1, PROFILEID+1
        sbbo    r1, r0, 0, 4
fakeMainLoop:
        // Test Code for Profiling
        //-------------------------------
#endif

        //
        // Wait for the next clock tick
        //
        qbbc    bgTaskLoop, r31.tStatus_Timer            // Look for a timer tick
        set     r31.tStatus_Timer                        // Clear the timer trigger                   
        add     static.timerTicks, static.timerTicks, 1  // Account for the tick

        // Save background context to scratch
        ldi     r0, QOSSCHED_OFF_BACKGROUND_CONTEXT
        sbco    counters, cLocalRAM, r0, SIZE(counters)
        jmp     foregroundTask                      // Process the scheduler

        //--------------------------------------------------------------
        //
        // The background task inspects all the queues for congestion
        //
        // It inspects one queue per trip through the loop, then returns
        // to poll the timer to go back to the foreground task
        //

        

.leave portProcessing

foregroundTask:
#ifdef _MODEL_
        //-------------------------------
        // Test Code for Profiling
        mov     r0.w0, PRCMD_BASE & 0xFFFF
        mov     r0.w2, PRCMD_BASE >> 16
        mov     r1, PROFILEID
        sbbo    r1, r0, 0, 4
        // Test Code for Profiling
        //-------------------------------
#endif

.using qosCmd
        //
        // Get a user command
        //
        lbco    qoscmd, cLocalRAM, QOSSCHED_OFF_COMMAND, SIZE(qoscmd)
        qbbc    noCommand, qoscmd.Command.t7
        mov     qoscmd.returnCode, QOSSCHED_CMD_RETCODE_SUCCESS
        qbne    notCmdGetBase, qoscmd.Command, QOSSCHED_CMD_GET_QUEUE_BASE
        
        //-------------------------
        //
        // Command: Get Queue Base
        //
#ifdef DROP_SCHED
        qbne    getQueBaseRegular, qoscmd.option, 1
        ldi     r0, QOSSCHED_DROPSCHED_OFF_PROC
        lbco    dropProc, cLocalRAM, r0, SIZE(dropProc)
        mov     qoscmd.Index, dropProc.QueueBase
        jmp     cmdDone
getQueBaseRegular:
#endif
        qbne    ERROR_InvalidOption, qoscmd.option, 0
        mov     qoscmd.Index, static.QueueBase
        jmp     cmdDone

notCmdGetBase:
        qbne    notCmdSetBase, qoscmd.Command, QOSSCHED_CMD_SET_QUEUE_BASE
        
        //-------------------------
        //
        // Command: Set Queue Base
        //
        and     r0, qoscmd.Index, 0x1F                  // Must be a multiple of 32
        qbeq    baseIsLegal, r0, 0
ERROR_InvalidParam:
        mov     qoscmd.returnCode, QOSSCHED_CMD_RETCODE_INVALID_INDEX
        jmp     cmdDone
baseIsLegal:        
#ifdef DROP_SCHED
        qbne    setQueBaseRegular, qoscmd.option, 1
        ldi     r0, QOSSCHED_DROPSCHED_OFF_PROC
        lbco    dropProc, cLocalRAM, r0, SIZE(dropProc)
        mov     dropProc.QueueBase, qoscmd.Index
        sbco    dropProc, cLocalRAM, r0, SIZE(dropProc)
        jmp     cmdDone
setQueBaseRegular:
#endif
        qbne    ERROR_InvalidOption, qoscmd.option, 0
        mov     static.QueueBase, qoscmd.Index
        jmp     cmdDone
        
notCmdSetBase:
        qbne    notCmdTimer, qoscmd.Command, QOSSCHED_CMD_TIMER_CONFIG

        //-------------------------------------------------------------
        //
        // Command: Timer Config
        //
        mov     r0, 0
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        mov     r0, qoscmd.Index                   
        sbco    r0, cTimer, TIMER_OFF_LOAD, 4
        ldi     r0, 1<<15 | 0b0000<<2 | 0b11        // Enable | /2 | Mode+Go
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        jmp     cmdDone

notCmdTimer:
        qbne    notCmdPort, qoscmd.Command, QOSSCHED_CMD_PORT_ENABLE

        //-------------------------------------------------------------
        //
        // Command: Enable/Disable Port
        //

        // fresh start the bg process
        set     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_CFG_CHANGED
#ifdef DROP_SCHED
        qbne    portEnDisRegularPort, qoscmd_split.Index_msb, 1
        qbne    ERROR_InvalidParam, qoscmd_split.Index_lsb, 0
        jmp     portEnDisCheck
portEnDisRegularPort:
#endif
        qble    ERROR_InvalidParam, qoscmd_split.Index_lsb, QOS_SCHED_PORTS
        qbne    ERROR_InvalidParam, qoscmd_split.Index_msb, 0  
portEnDisCheck:
        qbeq    portEnable, qoscmd.Option, QOPT_PORT_ENABLE
        qbeq    portDisable, qoscmd.Option, QOPT_PORT_DISABLE
ERROR_InvalidOption:
        mov     qoscmd.returnCode, QOSSCHED_CMD_RETCODE_INVALID_OPTION
        jmp     cmdDone
portEnable:
#ifdef DROP_SCHED
        qbeq    portEnableRegularPort, qoscmd_split.Index_msb, 0
        set     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_DROP_SCHED_ENABLE
        jmp     cmdDoneOK
portEnableRegularPort:
#endif
        set     static2.stateAndPortEnable, qoscmd_split.Index_lsb
        //--------------------------------------------------------------------
        // Initialize portProc, groupProc, and queueProc for this port
        //--------------------------------------------------------------------
        // PDSP cant multiply so need to count to find offset for ports structure
.using portEnableScope
        ldi     portSet.dstProc, QOSSCHED_OFF_PORT_PROCS
        ldi     portSet.dstCfg, QOSSCHED_OFF_PORT_CFGS
        ldi     portSet.stepProc, PORT_PROC_SIZE_LITE
        ldi     portSet.stepCfg, PORT_CFG_SIZE_LITE
        ldi     portSet.secPort, 0
        mov     portSet.portNum, qoscmd_split.Index_lsb
        qbeq    portEnableOffsetZero, portSet.portNum, 0
portEnableOffsetLoop:
        // really strict less than:  portSet.portNum < QOS_SCHED_FULL_PORTS
        // assuming QOS_SCHED_FULL_PORTS is 2
        // if configuring port 2, we want to add 2 full sized ports
        // if configuring port 3, we add to add 2 full sized port and 1 lite ports
#ifdef MULTIGROUP
        qblt    portEnableOffsetLoopLite, portSet.portNum, QOS_SCHED_FULL_PORTS
portEnableOffsetLoopFull:
        ldi     portSet.stepProc, PORT_PROC_SIZE_FULL
        ldi     portSet.stepCfg, PORT_CFG_SIZE_FULL
#endif
portEnableOffsetLoopLite:
        add     portSet.dstProc, portSet.dstProc, portSet.stepProc
        add     portSet.dstCfg, portSet.dstCfg, portSet.stepCfg
        sub     portSet.portNum, portSet.portNum, 1
        qblt    portEnableOffsetLoop, portSet.portNum, 0
portEnableOffsetZero:
        // Preserve size information
        lbco    portProc, cLocalRAM, portSet.dstProc, SIZE(portProc)
        // zero all but the sizes
        ldi     portProc.cirCurrent, 0
        ldi     portProc.wrrCreditMask, 0
        ldi     portProc.nextGroup, 0
#ifdef SIMUBYTEPKT
        ldi     portProc.pktCirCurrent, 0
#endif
        // Zero the whole ports process/dynamic area
        mov     portSet.size, portProc.portProcSize
        mov     r0, portSet.dstProc
        ldi     portSet.scratch, 0
portEnableZeroLoop:
        sbco    portSet.scratch, cLocalRAM, r0, 4
        sub     portSet.size, portSet.size, 4
        add     r0, r0, 4
        qbne    portEnableZeroLoop, portSet.size, 0

        // Store back the portProc structure which includes sizes
        sbco    portProc, cLocalRAM, portSet.dstProc, SIZE(portProc)

        // If its lite joint even port, zero next port too
        qbbs    portEnableZeroNonJoint, portSet.secPort, 1 // already did second port
        lbco    portCfg, cLocalRAM, portSet.dstCfg, SIZE(portCfg)
        qbbc    portEnableZeroNonJoint, portCfg.flags, PORT_FLAGS_BIT_IS_JOINT
        ldi     portSet.secPort, 1
        ldi     portSet.portNum, 1  // execute one more port
#ifdef MULTIGROUP
        ldi     portSet.stepProc, PORT_PROC_SIZE_LITE
        ldi     portSet.stepCfg, PORT_CFG_SIZE_LITE
#endif
        jmp     portEnableOffsetLoopLite
        
portEnableZeroNonJoint:
        // Put the port number in r0
        mov     r0, qoscmd_split.Index_lsb
.leave portEnableScope
.using portProcessing
        // Now go through each group in the port and set portProc to 
        // match config
        call    resetPortState
portEnablePortWalk:
        qbeq    portEnablePortWalkDone, counters.portIndex, r0
        call    movePortState
        jmp     portEnablePortWalk
portEnablePortWalkDone:
        // Mark port that it has just been scheduled
        mov     portProc.lastTimerTicks, static.timerTicks
        call    resetGroupState
portEnableGroupWalk:
#ifdef MULTIGROUP
        qbeq    portEnableGroupWalkDone, counters.groupIndex, portCfg.groupCount
        mov     groupProc.nextQueue, groupCfg.SPCount
        call    moveGroupState
        jmp     portEnableGroupWalk
#endif
portEnableGroupWalkDone:
        call    movePortState    // Save lastTimerTick

.leave portProcessing
        jmp     cmdDoneOK
portDisable:
#ifdef DROP_SCHED
        qbeq    portDisableRegularPort, qoscmd_split.Index_msb, 0
        clr     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_DROP_SCHED_ENABLE
        jmp     cmdDoneOK
portDisableRegularPort:
#endif
        // Discard all packets on queues on the port
        clr     static2.stateAndPortEnable, qoscmd.Index
        mov     r0, qoscmd.Index
        call    resetPortState
.using portProcessing
.using portDisableScope
portDisablePortWalk:
        qbeq    portDisablePortWalkDone, counters.portIndex, r0
        call    movePortState
        jmp     portDisablePortWalk
portDisablePortWalkDone:
        // Discard all packets in all queues starting from 
        // counters.portQBaseIndex for portProc.portMaxQueues
        add     counters.queueNumber, counters.portQBaseIndex, static.queueBase
        mov     disable.i, 0
portDisableQueueWalk:
        qbeq    portDisableQueueWalkDone, disable.i, portProc.portMaxQueues
portDisableDescWalk:
        // Check if there is a packet
        lsl     r0, counters.queueNumber, 4   // *16 to get offset
        lbco    r0, cQPeekBase, r0, 4
        qbeq    portDisableDescWalkDone, r0, 0
        mov     r0, counters.queueNumber
        call    discardPacket
        jmp     portDisableDescWalk
portDisableDescWalkDone:
        add     disable.i, disable.i, 1
        add     counters.queueNumber, counters.queueNumber, 1
        jmp     portDisableQueueWalk
portDisableQueueWalkDone:

.leave portDisableScope
.leave portProcessing
        jmp     cmdDoneOK

notCmdPort:
        qbne    notCmdShadow, qoscmd.Command, QOSSCHED_CMD_PORT_SHADOW
        //-------------------------------------------------------------
        //
        // Command: Move in/out of shadow config for port
        //
.using shadCopy
        ldi     shadcopy.srcAddr, QOSSCHED_OFF_PORT_SHADOW
        ldi     shadcopy.jumpOffset, 0
#ifdef DROP_SCHED
        qbne    shadNotDropCfgProf, qoscmd_split.Index_msb, DROP_SCHED_SHADOW_CFG_PROF
        // Copy config profiles
        ldi     shadcopy.dstAddr, QOSSCHED_DROPSCHED_OFF_CFG_PROFS
        ldi     shadcopy.size, DROPSCHED_SIZE_CFG_PROFS
        jmp     shadFixDirection
shadNotDropCfgProf:        
        qbne    shadNotDropQue, qoscmd_split.Index_msb, DROP_SCHED_SHADOW_QUE_CFG
        // Copy Queue Configs
        ldi     shadcopy.dstAddr, QOSSCHED_DROPSCHED_OFF_QUE_PROFS
        ldi     shadcopy.size, DROPSCHED_SIZE_QUE_PROFS
        qbeq    shadQueCfgShadIn, qoscmd.Option, QOPT_PORT_SHADOW_IN
        call    dropSchedSetEnBits  // separate out enable bits
shadQueCfgShadIn:
        jmp     shadFixDirection
shadNotDropQue:        
        qbne    shadNotDropOutProf, qoscmd_split.Index_msb, DROP_SCHED_SHADOW_OUT_PROF
        // Copy Out Profiles
        ldi     shadcopy.dstAddr, QOSSCHED_DROPSCHED_OFF_OUT_PROFS
        ldi     shadcopy.size, DROPSCHED_SIZE_OUT_PROFS
        jmp     shadFixDirection
shadNotDropOutProf:        
        qbne    shadRegular, qoscmd_split.Index_msb, DROP_SCHED_SHADOW_CFG
        // Copy top level config
        ldi     shadcopy.dstAddr, QOSSCHED_DROPSCHED_OFF_CFG
        ldi     shadcopy.size, DROPSCHED_SIZE_CFG
        qbeq    shadFixDirection, qoscmd.Option, QOPT_PORT_SHADOW_IN
        // When copying shadow to active, need to special case the seeds
        // since they aren't copied if input is 0
        sub     shadcopy.size, shadcopy.size, 12
        add     r0.w0, shadcopy.size, shadcopy.srcAddr
        add     r0.w2, shadcopy.size, shadcopy.dstAddr
        ldi     r1, 3
shadSeedLoop:
        lbco    shadcopy.scratch, cLocalRAM, r0.w0, 4
        qbeq    shadSeedNoWrite, shadcopy.scratch, 0
        sbco    shadcopy.scratch, cLocalRAM, r0.w2, 4
shadSeedNoWrite:
        add     r0.w0, r0.w0, 4
        add     r0.w2, r0.w2, 4
        sub     r1, r1, 1
        qbne    shadSeedLoop, r1, 0
        jmp     shadFixDirection
shadRegular:
#endif
        qble    ERROR_InvalidParam, qoscmd_split.Index_lsb, QOS_SCHED_PORTS
        qbne    ERROR_InvalidParam, qoscmd_split.Index_msb, DROP_SCHED_SHADOW_QOS_PORT

        // PDSP can't multiply so need to count to find offset for port's structure
        ldi     shadcopy.dstAddr, QOSSCHED_OFF_PORT_CFGS
        ldi     r1, PORT_CFG_SIZE_LITE
        mov     r0, qoscmd.Index
        qbeq    shadOffsetZero, r0, 0
shadOffsetLoop:
        // really strict less than:  r0 < QOS_SCHED_FULL_PORTS
        // assuming QOS_SCHED_FULL_PORTS is 2
        // if configuring port 2, we want to add 2 full sized ports
        // if configuring port 3, we add to add 2 full sized port and 1 lite ports
        qblt    shadOffsetLoopLite, r0, QOS_SCHED_FULL_PORTS
shadOffsetLoopFull:
        ldi     r1, PORT_CFG_SIZE_FULL
shadOffsetLoopLite:
        add     shadcopy.dstAddr, shadcopy.dstAddr, r1
        sub     r0, r0, 1
        qblt    shadOffsetLoop, r0, 0

shadOffsetZero:
        // Determine copy size
        ldi     shadcopy.size, PORT_CFG_SIZE_LITE
        ldi     shadcopy.jumpOffset, PORT_CFG_SIZE_LITE - SIZE(struct_PortCfgSize)
        // really less than equal to: port <= QOS_SCHED_FULL_PORTS
        // assuming QOS_SCHED_FULL_PORTS is 2
        // then the size of this port is LITE.
        qble    shadOffsetThisLite, qoscmd.Index, QOS_SCHED_FULL_PORTS
        ldi     shadcopy.size, PORT_CFG_SIZE_FULL
        ldi     shadcopy.jumpOffset, 0
shadOffsetThisLite:
shadFixDirection:
        // Determine copy direction
        qbeq    shadowIsIn, qoscmd.Option, QOPT_PORT_SHADOW_IN
        qbne    ERROR_InvalidOption, qoscmd.Option, QOPT_PORT_SHADOW_OUT
        // fresh start the bg process if copying shadow out
#ifdef DROP_SCHED
        // but only on regular ports
        qbne    shadowIsOut, qoscmd_split.Index_msb, 0
#endif
        set     static2.stateAndPortEnable, static2.stateAndPortEnable, STATIC_STATE_BIT_CFG_CHANGED
        jmp     shadowIsOut

shadowIsIn:
        // copy is from active to shadow - assumption above is backwards
        mov     shadcopy.srcAddr, shadcopy.dstAddr
        ldi     shadcopy.dstAddr, QOSSCHED_OFF_PORT_SHADOW
shadowIsOut:
shadowLoop:
        lbco    &shadcopy.scratch, cLocalRAM, shadcopy.srcAddr, 4
        sbco    &shadcopy.scratch, cLocalRAM, shadcopy.dstAddr, 4
        add     shadcopy.srcAddr, shadcopy.srcAddr, 4
        add     shadcopy.dstAddr, shadcopy.dstAddr, 4
        sub     shadcopy.size, shadcopy.size, 4
#ifndef MULTIGROUP
        // Skip unused group parameters
        qbne    shadowNoSkip, shadcopy.jumpOffset, shadcopy.size
        qbeq    shadowSkipIn, qoscmd.Option, QOPT_PORT_SHADOW_IN
        add     shadcopy.srcAddr, shadcopy.srcAddr, GROUP_JUMP
        jmp     shadowSkipDone
shadowSkipIn:
        add     shadcopy.dstAddr, shadcopy.dstAddr, GROUP_JUMP
shadowSkipDone:
shadowNoSkip:
#endif
        qblt    shadowLoop, shadcopy.size, 0
        
        jmp     cmdDone
.leave shadCopy

notCmdShadow:
        qbne    notCmdStats, qoscmd.Command, QOSSCHED_CMD_REQ_STATS
        //-------------------------------------------------------------
        //
        // Command: Request/reset stats
        //
        qbbc    statsReqRegular, qoscmd.option, QOPT_STATS_REQUEST_DROPSCHED
#ifdef DROP_SCHED
.using dropSchedStatsReqScope
        qble    ERROR_InvalidParam, qoscmd.Index, QOSSCHED_DROPSCHED_NUM_STATS_BLOCKS
        // Calculate stats address in r0
        calcDropStatsAddr r0, qoscmd.Index, r1
        lbco    dropStats, cLocalRAM, r0, SIZE(dropStats)
        mov     statsResp.bytesForwarded_lsw, dropStats.bytesForwarded
        ldi     statsResp.bytesForwarded_msw, 0
        mov     statsResp.bytesDropped_lsw, dropStats.bytesDropped
        ldi     statsResp.bytesDropped_msw, 0
        mov     statsResp.packetsForwarded, dropStats.packetsForwarded
        mov     statsResp.packetsDropped, dropStats.packetsDropped

        // Perform reset
        qbbc    statsDropNoFwdBytesRst, qoscmd.option, QOPT_RESET_STATS_FORWARDED_BYTES
        ldi     dropStats.bytesForwarded, 0
statsDropNoFwdBytesRst:
        qbbc    statsDropNoFwdPktRst, qoscmd.option, QOPT_RESET_STATS_FORWARDED_PACKETS
        ldi     dropStats.packetsForwarded, 0
statsDropNoFwdPktRst:
        qbbc    statsDropNoDscBytesRst, qoscmd.option, QOPT_RESET_STATS_DISCARDED_BYTES
        ldi     dropStats.bytesDropped, 0
statsDropNoDscBytesRst:
        qbbc    statsDropNoDscPktRst, qoscmd.option, QOPT_RESET_STATS_DISCARDED_PACKETS
        ldi     dropStats.packetsDropped, 0
statsDropNoDscPktRst:
        sbco    dropStats, cLocalRAM, r0, SIZE(dropStats)
        ldi     r0, QOSSCHED_OFF_STATS
        sbco    statsResp, cLocalRAM, r0, SIZE(statsResp)
        jmp     cmdDoneOK
.leave dropSchedStatsReqScope
#else
        jmp     ERROR_InvalidOption
#endif
statsReqRegular:
.using portProcessing
.using statsReqScope
        // Extract the command and options into r0.  qoscmd will be destroyed
        // by the rest of this code.
        // Extract the port
        and     r0s.port, qoscmd.Index, STATS_PORT_MASK
        // Extract the group
        lsr     r0s.group, qoscmd.Index, STATS_GROUP_SHIFT 
        and     r0s.group, r0s.group, STATS_GROUP_MASK
        // Extract the queue
        lsr     r0s.queue, qoscmd.Index, STATS_QUEUE_SHIFT
        // Extract the options
        mov     r0s.option, qoscmd.option

        // check and load the port
        qble    ERROR_InvalidParam, r0s.port, QOS_SCHED_PORTS
        call    resetPortState
        qbeq    statsReqPortDone, counters.portIndex, r0s.port
statsReqPortLoop:
        call    movePortState
        qbgt    statsReqPortLoop, counters.portIndex, r0s.port
statsReqPortDone:
        // check and load the group
        qble    ERROR_InvalidParam, r0s.group, portCfg.groupCount
        call    resetGroupState
#ifdef MULTIGROUP
        qbeq    statsReqGroupDone, counters.groupIndex, r0s.group
statsReqGroupLoop:
        call    moveGroupState
        qbgt    statsReqGroupLoop, counters.groupIndex, r0s.group
#endif
statsReqGroupDone:
        // check and load the queue
        ldi     r1, QOSSCHED_OFF_STATS
#ifndef DROP_SCHED
        // See if multiple stats dump needed
        qbne    statsReqOneQueue, r0s.queue, 0xff
        ldi     r0s.queue, 0 // start at queue 0
        ldi     r1, QOSSCHED_OFF_PORT_SHADOW
        mov     statsRespFlags.queueCount, groupCfg.queueCount
        sbco    statsRespFlags, cLocalRAM, r1, SIZE(statsRespFlags)
        add     r1, r1, SIZE(statsRespFlags)
        // whack port to control loop
        mov     r0s.port, groupCfg.queueCount
        jmp     statsReqMultiQ
statsReqOneQueue:
        // whack port to control loop
        add     r0s.port, r0s.queue, 1
statsReqMultiQ:
#endif
        qble    ERROR_InvalidParam, r0s.queue, groupCfg.queueCount
        call    resetQueueState
        qbeq    statsReqQueueDone, counters.queueIndex, r0s.queue
statsReqQueueLoop:
        call    moveQueueState
        qbgt    statsReqQueueLoop, counters.queueIndex, r0s.queue
statsReqQueueDone:
        // Can directly load queueProc and use statsResp which are overlaid,
        // since only counters is needed from this point on
        lbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)
        mov     statsResp.bytesForwarded_lsw, queueProc.bytesForwardedLSW
        mov     statsResp.bytesForwarded_msw, queueProc.bytesForwardedMSW
        mov     statsResp.bytesDropped_lsw, queueProc.bytesDroppedLSW
        mov     statsResp.bytesDropped_msw, queueProc.bytesDroppedMSW
        mov     statsResp.packetsForwarded, queueProc.packetsForwarded
        mov     statsResp.packetsDropped, queueProc.packetsDropped
        // Store statsResp
        sbco    statsResp, cLocalRAM, r1, SIZE(statsResp)
        // Perform reset
        qbbc    statsNoFwdBytesRst, r0s.option, QOPT_RESET_STATS_FORWARDED_BYTES
        ldi     queueProc.bytesForwardedLSW, 0
        ldi     queueProc.bytesForwardedMSW, 0
statsNoFwdBytesRst:
        qbbc    statsNoFwdPktRst, r0s.option, QOPT_RESET_STATS_FORWARDED_PACKETS
        ldi     queueProc.packetsForwarded, 0
statsNoFwdPktRst:
        qbbc    statsNoDscBytesRst, r0s.option, QOPT_RESET_STATS_DISCARDED_BYTES
        ldi     queueProc.bytesDroppedLSW, 0
        ldi     queueProc.bytesDroppedMSW, 0
statsNoDscBytesRst:
        qbbc    statsNoDscPktRst, r0s.option, QOPT_RESET_STATS_DISCARDED_PACKETS
        ldi     queueProc.packetsDropped, 0
statsNoDscPktRst:
        sbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)
#ifndef DROP_SCHED
        // loop to dump multiple queues in group
        add     r0s.queue, r0s.queue, 1
        add     r1, r1, SIZE(statsResp)
        // use port as loop counter (# of queues)
        qbgt    statsReqQueueLoop, r0s.queue, r0s.port
#endif
.leave statsReqScope
.leave portProcessing
        jmp     cmdDoneOK

notCmdStats:
        //-------------------------------------------------------------
        //
        // Invalid Command
        //
        mov     qoscmd.returnCode, QOSSCHED_CMD_RETCODE_INVALID_COMMAND
        jmp     cmdDone
cmdDoneOK:
        mov     qoscmd.returnCode, QOSSCHED_CMD_RETCODE_SUCCESS
cmdDone:
        // Wrap up command
        mov     qoscmd.Command, 0
        sbco    qoscmd, cLocalRAM, QOSSCHED_OFF_COMMAND, SIZE(qoscmd)
.leave qosCmd

noCommand:
#ifdef DROP_SCHED
.using dropSchedProcessing
        ldi     r0, QOSSCHED_DROPSCHED_OFF_CFG
        // load dropCfg and dropProCFG + OFFSET(dropCfg.rng_s1)
        lbco    dropCfg, cLocalRAM, r0, SIZE(dropCfg)
        ldi     r0, QOSSCHED_DROPSCHED_OFF_PROC
        lbco    dropProc, cLocalRAM, r0, SIZE(dropProc)

        qbbc    dropSchedRunQosPorts, static2.stateAndPortEnable, STATIC_STATE_BIT_DROP_SCHED_ENABLE

#ifdef RNG_TEST
// The following is the C code to test the RNG -- it should put put into
// the unit test just after enabling drop scheduler.  Note that the
// seed1..3 need to be manually set, or get..DropSchedCfg needs to be
// called BEFORE enabling the drop scheduler!
// #ifdef RNG_TEST
//     /* If the firmware is compiled as RNG testing build, all it will do is generate RNG once
//      * per tick
//      */
//     model_drop_sched.rng_s1 = dropCfgRB.seed1;
//     model_drop_sched.rng_s2 = dropCfgRB.seed2;
//     model_drop_sched.rng_s3 = dropCfgRB.seed3;
//     {
//         int n;
//         for (n = 0; n < 100000; n++)
//         {
//             uint32_t rng, crng;
//             while ( ! ((rng = *(volatile uint32_t *)0x2abdd80)));
//             *(volatile uint32_t *)0x2abdd80 = 0;
//             rng &= 0xffff;
//             crng = model_Rand16 (&model_drop_sched);
//             if (crng != rng)
//             {
//                 System_printf("rn #%d, c got %d, asm got %d\n", n, crng, rng);
//                 errorCount++;
//             }
//         }
//     }
// #endif
        // RNG test stores 1 random number per tick to debug context
        lbco    r0, cLocalRAM, r1, 4
        qbbs    norng, r0, 31
        call    dropSchedRand16
        set     r0, r0, 31
        ldi     r1, QOSSCHED_OFF_DEBUG_CONTEXT
        sbco    r0, cLocalRAM, r1, 4
norng:
#else
// Drop Scheduler Main
    // void DropScheduler (DROP_SCHED *dSched)
    // {
    //     uint8_t in_depth_p[NUM_DROP_INPUT_QUEUES]; // input packets
    //     uint32_t in_packets_present[(NUM_DROP_INPUT_QUEUES + 31)/32];
    //     uint8_t out_depth_p[NUM_DROP_OUTPUT_PROFILES];
    //     uint16_t out_depth_b32[NUM_DROP_OUTPUT_PROFILES];

    //         // Snapshot instantaneous queue depth of input queues.
    //         // If there are more than 255 packets, then only 255 are processed this tick
    //         DropSchedSnapInput (dSched, in_depth_p, in_packets_present);
        call    dropSchedSnapInput
    //         DropSchedSnapOutput (dSched, out_depth_p, out_depth_b32);
    // 
        call    dropSchedSnapOutput
    //         DropSchedDropDisabled (dSched, in_packets_present);
        call    dropSchedDropDisabled
    // 
    //         DropSchedSched (dSched, in_depth_p, out_depth_p, out_depth_b32, in_packets_present);
        call    dropSchedSched
#endif

        // Save state before running qos sched
        ldi     r0, QOSSCHED_DROPSCHED_OFF_CFG + OFFSET(dropCfg.rng_s1)
        sbco    dropCfg.rng_s1, cLocalRAM, r0, SIZE(dropCfg) - OFFSET(dropCfg.rng_s1)
        ldi     r0, QOSSCHED_DROPSCHED_OFF_PROC
        sbco    dropProc, cLocalRAM, r0, SIZE(dropProc)

dropSchedRunQosPorts:
.leave dropSchedProcessing
#endif

.using portProcessing
        call    resetPortState

portLoop:
        qbbc    portDoneNoFrame, static2.stateAndPortEnable, counters.portIndex
        // Port is enabled
//-----------------------------------------------------------------------------
// PhysPortScheduler()
//-----------------------------------------------------------------------------
        ldi     state.pirCreditMask, 0

portSchedCreditLoop:
    // /* Add credits for all TimerTicks that occurred since last time this port ran */
    // while ((uint8)(TimerTicks - pPort->LastTimerTicks) > 0)
    // {
        qbeq    portSchedCreditLoopEnd, static.timerTicks, portProc.lastTimerTicks
    //    pPort->lastTimerTicks++;
        add     portProc.lastTimerTicks, portProc.lastTimerTicks, 1

    // Credit for the main port (byte credits for simu build)
    // 
    //#ifdef SIMUBYTEPKT
    //  if (pPortCfg->fIsSupportByteShaping)
    //#endif
    //  {
    //    pPort->CirCurrent +=  pPort->CirIteration;
    //    if( pPort->CirCurrent > pPort->CirMax )
    //        pPort->CirCurrent = pPort->CirMax;
    //  }
#ifdef SIMUBYTEPKT
        qbbc    portPktCir, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_BYTES
#endif
        add     portProc.cirCurrent, portProc.cirCurrent, portCfg.cirIterationCredit
        qbbs    portCirCapped, portProc.cirCurrent, 31 // < 0
        qbge    portCirCapped, portProc.cirCurrent, portCfg.cirMax
        mov     portProc.cirCurrent, portCfg.cirMax
portCirCapped:
#ifdef SIMUBYTEPKT
portPktCir:
    // Credit for the main port packet credits
    //#ifdef SIMUBYTEPKT
    //  if (pPortCfg->fIsSupportPacketShaping)
    //  {
    //    pPort->pktCirCurrent +=  pPort->pktCirIteration;
    //    if( pPort->pktCirCurrent > pPort->pktCirMax )
    //        pPort->pktCirCurrent = pPort->pktCirMax;
    //  }
    //#endif
        qbbc    portPktCirDone, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_PKTS
        // Load the packet overlay for cfg only
        add     r0, counters.portCfgOffset, SIZE(struct_PortCfgRegMap)
        lbco    portCfgPktOv, cLocalRAM, r0, SIZE(portCfgPktOv)
        add     portProc.pktCirCurrent, portProc.pktCirCurrent, portCfgPktOv.pktCirIterationCredit
        qbbs    portPktCirCapped, portProc.pktCirCurrent, 31 // < 0
        qbge    portPktCirCapped, portProc.pktCirCurrent, portCfgPktOv.pktCirMax
        mov     portProc.pktCirCurrent, portCfgPktOv.pktCirMax
portPktCirCapped:
        // Load back the byte overlay in case the while() loop iterates
        add     r0, counters.portCfgOffset, PORTCFGPKTOV_OFFSET
        lbco    portCfgPktOv, cLocalRAM, r0, SIZE(portCfgPktOv)
portPktCirDone:
#endif
#ifdef MULTIGROUP
    // Credit for the ports logical groups
    //    for( i=0; i<pPort->GroupCount; i++ )
    //    {
        call    resetGroupState
groupCreditLoop:
    //#ifdef SIMUBYTEPKT
    //     if (pGroupCfg->fIsSupportByteShaping)
    //#endif
    //     {
    //        pPort->Group[i].CirCurrent +=  pPort->Group[i].CirIteration;
    //        // Cap CIR credit at its max level
    //        if( pPort->Group[i].CirCurrent > pPort->Group[i].CirMax )
    //            pPort->Group[i].CirCurrent = pPort->Group[i].CirMax;
#ifdef SIMUBYTEPKT
        qbbc    groupPktCir, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_BYTES
#endif
        add     groupProc.cirCurrent, groupProc.cirCurrent, groupCfg.cirIteration
        qbbs    groupCirCapped, groupProc.cirCurrent, 31 // < 0
        qbge    groupCirCapped, groupProc.cirCurrent, groupCfg.cirMax
        mov     groupProc.cirCurrent, groupCfg.cirMax
groupCirCapped:
    //        pPort->Group[i].PirCurrent +=  pPort->Group[i].PirIteration;
        add     groupProc.pirCurrent, groupProc.pirCurrent, groupCfg.pirIteration
    //        if( pPort->Group[i].PirCurrent > 0 )
    //        {
    //            // Track every group with PIR credit for later
    //            PirCreditMask |= (1<<i);
    //            // Cap PIR credit at its max level
    //            if( pPort->Group[i].PirCurrent > pPort->Group[i].PirMax )
    //                pPort->Group[i].PirCurrent = pPort->Group[i].PirMax;
    //        }
    //     }
        qbeq    groupCreditLoopEnd, groupProc.pirCurrent, 0  // == 0
        qbbs    groupCreditLoopEnd, groupProc.pirCurrent, 31 // < 0
        set     state.pirCreditMask, state.pirCreditMask, counters.groupIndex
    //  qbbs    groupPirCapped, groupProc.pirCurrent, 31 // < 0  (not needed see prev line)
        qbge    groupPirCapped, groupProc.pirCurrent, groupCfg.pirMax
        mov     groupProc.pirCurrent, groupCfg.pirMax
groupPirCapped:
groupCreditLoopEnd:
#ifdef SIMUBYTEPKT
groupPktCir:
    //#ifdef SIMUBYTEPKT
    //     if (pGroupCfg->fIsSupportPacketShaping)
    //     {
    //        pPort->Group[i].PktCirCurrent +=  pPort->Group[i].PktCirIteration;
    //        // Cap CIR credit at its max level
    //        if( pPort->Group[i].PktCirCurrent > pPort->Group[i].PktCirMax )
    //            pPort->Group[i].PktCirCurrent = pPort->Group[i].PktCirMax;
        qbbc    groupCreditPktDone, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_PKTS
        // Load overlays
        add     r0.w0, counters.groupProcOffset, SIZE(struct_GroupProcRegMap)
        add     r0.w2, counters.groupCfgOffset, SIZE(struct_GroupCfgRegMap)
        sbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
        lbco    groupProcPktOv, cLocalRAM, r0.w0, SIZE(groupProcPktOv)
        lbco    groupCfgPktOv, cLocalRAM, r0.w2, SIZE(groupCfgPktOv)
        add     groupProcPktOv.pktCirCurrent, groupProcPktOv.pktCirCurrent, groupCfgPktOv.pktCirIteration
        qbbs    groupPktCirCapped, groupProcPktOv.pktCirCurrent, 31 // < 0
        qbge    groupPktCirCapped, groupProcPktOv.pktCirCurrent, groupCfgPktOv.pktCirMax
        mov     groupProcPktOv.pktCirCurrent, groupCfgPktOv.pktCirMax
groupPktCirCapped:
    //        pPort->Group[i].PktPirCurrent +=  pPort->Group[i].PktPirIteration;
        add     groupProcPktOv.pktPirCurrent, groupProcPktOv.pktPirCurrent, groupCfgPktOv.pktPirIteration
    //        if( pPort->Group[i].PktPirCurrent > 0 )
    //        {
    //            // Track every group with PIR credit for later
    //            PirCreditMask |= (1<<i);
    //            // Cap PIR credit at its max level
    //            if( pPort->Group[i].PktPirCurrent > pPort->Group[i].PktPirMax )
    //                pPort->Group[i].PktPirCurrent = pPort->Group[i].PktPirMax;
    //        }
    //     }
        clr     state.pirCreditMask, state.pirCreditMask, counters.groupIndex // if pkt credit required assume none avail
        qbeq    groupPktPirNoCred, groupProcPktOv.pktPirCurrent, 0  // == 0
        qbbs    groupPktPirNoCred, groupProcPktOv.pktPirCurrent, 31 // < 0
        set     state.pirCreditMask, state.pirCreditMask, counters.groupIndex
   //   qbbs    groupPktPirCapped, groupProcPktOv.pktPirCurrent, 31 // < 0  (not needed see prev line)
        qbge    groupPktPirCapped, groupProcPktOv.pktPirCurrent, groupCfgPktOv.pktPirMax
        mov     groupProcPktOv.pktPirCurrent, groupCfgPktOv.pktPirMax
groupPktPirCapped:
groupPktPirNoCred:
        // Store overlays
        sbco    groupProcPktOv, cLocalRAM, r0.w0, SIZE(groupProcPktOv)
        lbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
groupCreditPktDone:
        // dont need to load the config since moveGroupState replaces it
#endif
        call    moveGroupState
        qbgt    groupCreditLoop, counters.groupIndex, portCfg.groupCount
    //    }
#endif
        jmp     portSchedCreditLoop
portSchedCreditLoopEnd:

    //    /* Find out how much room is left in output queue */
    //    if (pPort->DestThrottleThresh)
    //    {
        ldi     r0, 0
        qbeq    portSchedNoOutThrottle, portCfg.destThrottleThresh, 0

    //        OutputSpaceAvail = pPort->DestThrottleThresh;
    //        if (pPort->fByteDestThrottle)
    //        {
    //            OutputSpaceAvail -= QueueByteLength (pPort->DestQueueNumber);
    //        }
    //        else
    //        {
    //            OutputSpaceAvail -= QueuePacketCount (pPort->DestQueueNumber);
    //        }
        lsl     r0, portCfg.destQueueNumber, 4
        qbbc    portSchedOutThrotRegSel, portCfg.flags, PORT_FLAGS_BIT_OUT_THROT_BYTES
        add     r0, r0, 4   // Read the bytes register (else read packet register)
portSchedOutThrotRegSel:
        lbco    r0, cQPeekBase, r0, 4
        sub     r0, portCfg.destThrottleThresh, r0

    //        // No room in output queue */
    //        if (OutputSpaceAvail <= 0)
    //        {
    //            return;
    //        }
        qbeq   portDoneNoFrame, r0, 0  // == 0
        qbbs   portDoneNoFrame, r0, 31 // < 0
    //    }
portSchedNoOutThrottle:
        // at this point r0 has the remaining bytes/packets that can be sent
        push   r0, 4 // OutputSpaceAvail

    // Assume all groups have packets pending until we find out otherwise
    //    PacketPendingMask = 0x1F;
#ifdef WIDE
        ldi     state.groupPktPendMask.w0, 0xffff
        ldi     state.groupPktPendmask.b2, 0xff
#else
        ldi     state.groupPktPendMask, 0xff
#endif

    //
    // Schedule each logic group's CIR, while also ensuring that the
    // physical port's CIR is not violated.
    // If the physical port has no credit quit out of the scheduler entirely
    //    if( pPort->CirCurrent <= 0 )
    //        return;
#ifdef SIMUBYTEPKT
        qbbc    portByteCirNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_BYTES
#endif
        qbeq    portDone, portProc.cirCurrent, 0 // == 0
        qbbs    portDone, portProc.cirCurrent, 31 // < 0
#ifdef SIMUBYTEPKT
portByteCirNotNeeded:
        qbbc    portPktCirNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_PKTS
        qbeq    portDone, portProc.pktCirCurrent, 0 // == 0
        qbbs    portDone, portProc.pktCirCurrent, 31 // < 0

portPktCirNotNeeded:
    // this would get here if neither PKTS or BYTES configured, but thats
    // invalid config. LLD prevents it.
#endif

    // Foreground task can exit once all packets are sent either because

    // Foreground task can exit once all packets are sent either because
    // the input queues are empty, or we ran out of group CIR, or we run 
    // out of port CIR.

    //    do
    //    {
schedCirPortCirLoop:
    //        fPacketsSent = 0;
        clrFPacketsSent
    //
    // Schedule each logic groups CIR, while also ensuring that the
    // physical ports CIR is not violated.
    //
    //        for( i=0; i<pPort->GroupCount; i++ )
    //        {
        call    resetGroupState
#ifdef MULTIGROUP
schedCirEachGroupLoop:
    //            // We will schedule each group for its full CIR
    //            if(pPort->Group[i].CirCurrent > 0)
    //            {
#ifdef SIMUBYTEPKT
        qbbc    schedCirByteNotNeeded, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_BYTES
#endif
        qbeq    schedCirOneGroupEnd, groupProc.cirCurrent, 0   // == 0
        qbbs    schedCirOneGroupEnd, groupProc.cirCurrent, 31  // < 0
#ifdef SIMUBYTEPKT
schedCirByteNotNeeded:
        qbbc    schedCirPktNotNeeded, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_PKTS

        // Swap overlay area -- no need to write since cir is now clean
        add     r0, counters.groupProcOffset, SIZE(struct_GroupProcRegMap)
        lbco    groupProcPktOv, cLocalRAM, r0, SIZE(groupProcPktOv)
        qbeq    schedCirNoPktCredit, groupProcPktOv.pktCirCurrent, 0   // == 0
        qbbs    schedCirNoPktCredit, groupProcPktOv.pktCirCurrent, 31  // < 0
        // Put back byte cir
        lbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
schedCirPktNotNeeded:
#endif
#endif
    //                // Attempt to schedule a packet
    //                BytesUsed = LogicalGroupScheduler( &pPort->Group[i] );
        call    logicalGroupScheduler
    //                // If no packet scheduled, clear the pending mask bit
    //                if( !BytesUsed )
    //                {
    //                    // Clear the pending mask bit
    //                    PacketPendingMask &= ~(1<<i);
    //                }
    //                else
    //                {
        qbne    schedCirOneGroupScheduled, r0, 0
        clr     state.groupPktPendMask, state.groupPktPendMask, counters.groupIndex
        jmp     schedCirOneGroupEnd
#ifdef SIMUBYTEPKT
schedCirNoPktCredit:
        // No credit, restore overlay and move next group
        lbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
        jmp     schedCirOneGroupEnd
#endif
schedCirOneGroupScheduled:
        clr     r0, r0, 31 // get rid of flag indicating packet forwarded to keep just bytes
#ifdef SIMUBYTEPKT
        clrFPirCreditOnly
        call    chargeCredits

        // fPacketsSent = 1;
        setFPacketsSent 
        // If no credit left return
        testFPortCreditNotAvail writebackGroupAndReturn
#else
    //
    //                    // Use packet or byte count, depending on configuration
    //                    if( pPort->fByteCirCredits )
    //                        CirCreditUsed = (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << BYTES_SCALE;
    //                    else
    //                        CirCreditUsed = 1 << PACKETS_SCALE;
        add      r1, r0, portCfg.overheadBytes
        sub      r1, r1, portCfg.removeBytes
        lsl      r1, r1, QOS_SCHED_BYTES_SCALE
        qbbs     schedCirOneGroupUseBytes, portCfg.flags, PORT_FLAGS_BIT_CIR_BYTES
        ldi      r1, 1 
        lsl      r1, r1, QOS_SCHED_PACKETS_SCALE // because ldi limited to 16 bits
schedCirOneGroupUseBytes:
    //
    //                    // Here we have a packet, so deduct the credit
    //                    pPort->CirCurrent          -= CirCreditUsed;
    //                    pPort->Group[i].CirCurrent -= CirCreditUsed;
    //                    pPort->Group[i].PirCurrent -= CirCreditUsed;
        sub     portProc.cirCurrent, portProc.cirCurrent, r1
#ifdef MULTIGROUP
        sub     groupProc.cirCurrent, groupProc.cirCurrent, r1
        sub     groupProc.pirCurrent, groupProc.pirCurrent, r1
#endif
    //                    fPacketsSent = 1;
        setFPacketsSent 

    //                    // If the physical port has no credit quit out of the scheduler entirely
    //                    if( pPort->CirCurrent <= 0 )
    //                        return;
        qbeq    writebackGroupAndReturn, portProc.cirCurrent, 0 // == 0
        qbbs    writebackGroupAndReturn, portProc.cirCurrent, 31 // < 0
#endif

    //                    // See if we used up output space
    //                    if (PhysPortUpdateOutputSpace (pPort, &OutputSpaceAvail, BytesUsed) == 0)
    //                    {
    //                        return;
    //                    }
 
    // r0 has bytes scheduled
        call    physPortUpdateOutputSpace
        qbeq    writebackGroupAndReturn, r0, 0
    //                }
    //            }
schedCirOneGroupEnd:
schedCirEachGroupLoopEnd:

#ifdef MULTIGROUP
        call    moveGroupState
        qbgt    schedCirEachGroupLoop, counters.groupIndex, portCfg.groupCount 
#else
        sbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
#endif
    //        }
    //    } while (fPacketsSent);
        testFPacketsSentTrue schedCirPortCirLoop
#ifdef MULTIGROUP

        // Move counters.groupIndex to 0
        call    resetGroupState
schedPirLoop:
    //
    // Schedule each logic groups PIR in a WRR fashion while the
    // physical ports CIR is not violated.
    //
    //    while(pPort->CirCurrent > 0)
    //    {
#ifdef SIMUBYTEPKT
        qbbc    schedPirByteCirNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_BYTES
#endif
        qbeq    schedPirLoopEnd, portProc.cirCurrent, 0   // == 0
        qbbs    schedPirLoopEnd, portProc.cirCurrent, 31   // < 0
#ifdef SIMUBYTEPKT
schedPirByteCirNotNeeded:
        qbbc    schedPirPktCirNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_PKTS
        qbeq    schedPirLoopEnd, portProc.pktCirCurrent, 0 // == 0
        qbbs    schedPirLoopEnd, portProc.pktCirCurrent, 31 // < 0

schedPirPktCirNotNeeded:
    // this would get here if neither PKTS or BYTES configured, but thats
    // invalid config. LLD prevents it.
#endif


    //        // If there are no queues left with PIR credit and packets, then were done
    //        if( !(PirCreditMask & PacketPendingMask) )
    //            return;
        and     r0, state.pirCreditMask, state.groupPktPendMask
        qbeq    portProcEnd, r0, 0

    //        // If all groups with WRR credit remaining are empty, add WRR credit
    //        while( !(PirCreditMask & pPort->WrrCreditMask & PacketPendingMask) )
    //        {
schedPirLoopAddWrrOuterLoop:
        and     r0, state.pirCreditMask, state.groupPktPendMask
        and     r0, r0, portProc.wrrCreditMask
        qbne    schedPirSetNextGroupLoop, r0, 0
    //            // Reset credits
    //            for(i=0; i<pPort->GroupCount; i++(
    //            {
        call    moveGroupState // write back current groupProc
        call    resetGroupState
schedPirLoopAddWrrInnerLoop:
    //                pPort->Group[i].WrrCurrentCredit += pPort->Group[i].WrrInitialCredit;
        add     groupProc.wrrCurrent, groupProc.wrrCurrent, groupCfg.wrrInitialCredit
    //                if (pPort->Group[i].WrrCurrentCredit > (pPort->Group[i].WrrInitialCredit << 1))
    //                    pPort->Group[i].WrrCurrentCredit = (pPort->Group[i].WrrInitialCredit << 1);
        lsl     r0, groupCfg.wrrInitialCredit, 1
        qbbs    schedPirLoopAddWrrInnerLoopNoOverflow, groupProc.wrrCurrent, 31  // < 0
        qbge    schedPirLoopAddWrrInnerLoopNoOverflow, groupProc.wrrCurrent, r0  // r0 >= wrrCurrent
        mov     groupProc.wrrCurrent, r0
schedPirLoopAddWrrInnerLoopNoOverflow:
    //                if ((pPort->Group[i].WrrCurrentCredit > 0) || (! pPort->Group[i].WrrInitialCredit))
        qbeq    schedPirLoopAddWrrInnerLoopHasCredit, groupCfg.wrrInitialCredit, 0
        qbbs    schedPirLoopAddWrrInnerLoopNoCredit, groupProc.wrrCurrent, 31 // < 0
        qbeq    schedPirLoopAddWrrInnerLoopNoCredit, groupProc.wrrCurrent, 0  // == 0
schedPirLoopAddWrrInnerLoopHasCredit:
    //                pPort->WrrCreditMask |= (1<<i);
        set     portProc.wrrCreditMask, portProc.wrrCreditMask, counters.groupIndex
schedPirLoopAddWrrInnerLoopNoCredit:
        call    moveGroupState
        qbgt    schedPirLoopAddWrrInnerLoop, counters.groupIndex, portCfg.groupCount 
    //            }
    //
        call    resetGroupState  // no writeback needed since schedPirLoopResetWrrLoop went to end
        jmp     schedPirLoopAddWrrOuterLoop
    //        }
        
schedPirSetNextGroupLoop:
    // pre-condition: resetGroupState has already been called
    // point counters.groupIndex back to portProc.nextGroup
        qbeq    schedPirLoopNoResetWrr, counters.groupIndex, portProc.nextGroup
        call    moveGroupState
        jmp     schedPirSetNextGroupLoop


schedPirLoopNoResetWrr:
    //        // If this group has PIR credit, WRR credit, and packets pending, then schedule a packet
    //        if( (PirCreditMask & pPort->WrrCreditMask & PacketPendingMask) & (1<<pPort->NextGroup) )
    //        {
// At this point r0 contains PirCreditMask & portProc.wrrCreditMask & PacketPendingMask
        qbbc    schedPirLoopNoSched, r0, portProc.nextGroup
    //            // Attempt to schedule a packet
    //            BytesUsed = LogicalGroupScheduler( &pPort->Group[pPort->NextGroup] );
        call    logicalGroupScheduler
    //
    //            // If no packet scheduled, clear the pending mask
    //            if( !BytesUsed )
        qbne    schedPirLoopPacketScheduled, r0, 0
    //                PacketPendingMask &= ~(1<<pPort->NextGroup);
        clr     state.groupPktPendMask, state.groupPktPendMask, portProc.nextGroup
        jmp     schedPirLoopNoSched
    //            else
    //            {
schedPirLoopPacketScheduled:
        clr     r0, r0, 31 // get rid of flag indicating packet forwarded to keep just bytes
#ifdef SIMUBYTEPKT
        setFPirCreditOnly
        call    chargeCredits

        lsl     r1, r0, QOS_SCHED_WRR_BYTES_SCALE
        qbbs    schedPirLoopWrrUseBytes, portCfg.flags, PORT_FLAGS_BIT_WRR_BYTES
        ldi     r1, 1 
        lsl     r1, r1, QOS_SCHED_WRR_PACKETS_SCALE // because ldi limited to 16 bits
schedPirLoopWrrUseBytes:
    //                // We also deduct the WRR credit
    //                pPort->Group[pPort->NextGroup].WrrCurrent -= WrrCreditUsed;
        sub     groupProc.wrrCurrent, groupProc.wrrCurrent, r1
    //
    //                // Clear the groups PIR credit mask if we depleted the PIR credit
    //                if( pPort->Group[pPort->NextGroup].PirCurrent <= 0 )
    //                    PirCreditMask &= ~(1<<pPort->NextGroup);
        testFGroupPirCreditAvail schedPirLoopPirLeft
schedPirLoopNoPirLeft:        
        clr     state.pirCreditMask, state.pirCreditMask, portProc.nextGroup
schedPirLoopPirLeft:
    //
    //                // Clear the groups WWR credit mask if we depleted the WRR credit
    //                if( pPort->Group[pPort->NextGroup].WrrCurrent <= 0 )
    //                    pPort->WrrCreditMask &= ~(1<<pPort->NextGroup);
        qbeq    schedPirLoopNoWrrLeft, groupProc.wrrCurrent, 0   // == 0
        qbbs    schedPirLoopNoWrrLeft, groupProc.wrrCurrent, 31  // < 0

        jmp     schedPirLoopWrrLeft
schedPirLoopNoWrrLeft:        
        clr     portProc.wrrCreditMask, portProc.wrrCreditMask, portProc.nextGroup
schedPirLoopWrrLeft:
    //               // See if we used up output space
    //                if (PhysPortUpdateOutputSpace (pPort, &OutputSpaceAvail, BytesUsed) == 0)
    //                {
    //                    return;
    //                }
#else
        push    r0, 4 // Save bytes used
    //                // Use packet or byte count, depending on configuration
    //                if( pPort->fByteCirCredits )
    //                    CirCreditUsed = (BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes) << BYTES_SCALE;
    //                else
    //                    CirCreditUsed = 1 << PACKETS_SCALE;
        add     r0, r0, portCfg.overheadBytes
        sub     r0, r0, portCfg.removeBytes
        lsl     r1, r0, QOS_SCHED_BYTES_SCALE
        qbbs    schedPirLoopCirUseBytes, portCfg.flags, PORT_FLAGS_BIT_CIR_BYTES
        ldi     r1, 1 
        lsl     r1, r1, QOS_SCHED_PACKETS_SCALE // because ldi limited to 16 bits
schedPirLoopCirUseBytes:
    //
    //                // Use packet or byte count, depending on configuration
    //                if( pPort->fByteWrrCredits )
    //                    WrrCreditUsed = (BytesUsed + pPort->OverheadBytes) << BYTES_SCALE;
    //                else
    //                    WrrCreditUsed = 1 << PACKETS_SCALE;
                                // r0 == WrrCreditUsed
        lsl     r0, r0, QOS_SCHED_WRR_BYTES_SCALE
        qbbs    schedPirLoopWrrUseBytes, portCfg.flags, PORT_FLAGS_BIT_WRR_BYTES
        ldi     r0, 1 
        lsl     r0, r0, QOS_SCHED_WRR_PACKETS_SCALE // because ldi limited to 16 bits
schedPirLoopWrrUseBytes:
    //
    //                // Deduct the PIR/CIR credit
    //                pPort->CirCurrent -= CirCreditUsed;
        sub     portProc.cirCurrent, portProc.cirCurrent, r1
    //                pPort->Group[pPort->NextGroup].PirCurrent -= CirCreditUsed;
        sub     groupProc.pirCurrent, groupProc.pirCurrent, r1
    //
    //                // We also deduct the WRR credit
    //                pPort->Group[pPort->NextGroup].WrrCurrent -= WrrCreditUsed;
        sub     groupProc.wrrCurrent, groupProc.wrrCurrent, r0
    //
    //                // Clear the groups PIR credit mask if we depleted the PIR credit
    //                if( pPort->Group[pPort->NextGroup].PirCurrent <= 0 )
    //                    PirCreditMask &= ~(1<<pPort->NextGroup);
        qbeq    schedPirLoopNoPirLeft, groupProc.pirCurrent, 0   // == 0
        qbbs    schedPirLoopNoPirLeft, groupProc.pirCurrent, 31  // < 0
        jmp     schedPirLoopPirLeft
schedPirLoopNoPirLeft:        
        clr     state.pirCreditMask, state.pirCreditMask, portProc.nextGroup
schedPirLoopPirLeft:
    //
    //                // Clear the groups WWR credit mask if we depleted the WRR credit
    //                if( pPort->Group[pPort->NextGroup].WrrCurrent <= 0 )
    //                    pPort->WrrCreditMask &= ~(1<<pPort->NextGroup);
        qbeq    schedPirLoopNoWrrLeft, groupProc.wrrCurrent, 0   // == 0
        qbbs    schedPirLoopNoWrrLeft, groupProc.wrrCurrent, 31  // < 0

        jmp     schedPirLoopWrrLeft
schedPirLoopNoWrrLeft:        
        clr     portProc.wrrCreditMask, portProc.wrrCreditMask, portProc.nextGroup
schedPirLoopWrrLeft:
    //               // See if we used up output space
    //                if (PhysPortUpdateOutputSpace (pPort, &OutputSpaceAvail, BytesUsed) == 0)
    //                {
    //                    return;
    //                }
        pop     r0, 4 // restore bytes used
#endif
        call    physPortUpdateOutputSpace
        qbeq    writebackGroupAndReturn, r0, 0
    //                }
    //            }
    //        }
schedPirLoopNoSched:
    //        // Move on to the next group
    //        pPort->NextGroup++;
        call    moveGroupState
    //        if( pPort->NextGroup == pPort->GroupCount )
    //            pPort->NextGroup = 0;
        add     portProc.nextGroup, portProc.nextGroup, 1
        qbgt    schedPirLoop, portProc.nextGroup, portCfg.groupCount
        ldi     portProc.nextGroup, 0
        call    resetGroupState
        jmp     schedPirLoop
    //    }
schedPirLoopEnd:
#else
        jmp     writebackGroupAndReturn
#endif // MULTIGROUP
portProcEnd:
portDone:
        popDiscard 4  // discard OutputSpaceAvail
portDoneNoFrame:
        // Move to next port
        call    movePortState
        qbgt    portLoop, counters.portIndex, QOS_SCHED_PORTS

        jmp     bgTaskRestore
writebackGroupAndReturn:
        sbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
        jmp     portProcEnd


#ifdef SIMUBYTEPKT
chargeCreditsPortCirNoBytesLeft:
        // No port byte credit remaining, clear flag
        clrFPortCreditAvail
        jmp     chargeCreditsPortByteDone
chargeCreditsPortCirNoPktsLeft:
        // No port packet credit remaining, clear flag
        clrFPortCreditAvail
        jmp     chargeCreditsPortPktDone

chargeCreditsCheckPirBytes:
        qbeq    chargeCreditsNoPirBytes, groupProc.pirCurrent, 0   // == 0
        qbbs    chargeCreditsNoPirBytes, groupProc.pirCurrent, 31  // < 0
        jmp     chargeCreditsPirByteDone
chargeCreditsNoPirBytes:
        clrFGroupPirCreditAvail
        jmp     chargeCreditsPirByteDone

chargeCreditsCheckPirPkts:
        qbeq    chargeCreditsNoPirPkts, groupProcPktOv.pktPirCurrent, 0   // == 0
        qbbs    chargeCreditsNoPirPkts, groupProcPktOv.pktPirCurrent, 31  // < 0
        jmp     chargeCreditsPirPktsDone
chargeCreditsNoPirPkts:
        clrFGroupPirCreditAvail
        jmp     chargeCreditsPirPktsDone

// Precondition: bytes in r0, FChargeWRR bit set/cleared in state.flags
// Group/Port conf/proc loaded
// Postcondition: r0 will still contain bytes
// fPortCreditAvail will be set appropriately in state.flags
chargeCredits:
        setFPortCreditAvail  // Assume credit still available after charging
        setFGroupPirCreditAvail // assume credit still available after charging
        // Calculate packet cost in bytes in R1
        add     r1, r0, portCfg.overheadBytes
        sub     r1, r1, portCfg.removeBytes
        lsl     r1, r1, QOS_SCHED_BYTES_SCALE
        // Charge port bytes if required
        qbbc    chargeCreditsPortByteNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_BYTES
        sub     portProc.cirCurrent, portProc.cirCurrent, r1
        qbeq    chargeCreditsPortCirNoBytesLeft, portProc.cirCurrent, 0 // == 0
        qbbs    chargeCreditsPortCirNoBytesLeft, portProc.cirCurrent, 31 // < 0
chargeCreditsPortByteNotNeeded:
chargeCreditsPortByteDone:
        // Charge group bytes if required
        qbbc    chargeCreditsGrpByteNotNeeded, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_BYTES
        sub     groupProc.pirCurrent, groupProc.pirCurrent, r1
        testFPirCreditOnly chargeCreditsCheckPirBytes
        sub     groupProc.cirCurrent, groupProc.cirCurrent, r1
chargeCreditsPirByteDone:
chargeCreditsGrpByteNotNeeded:
        // Calculate cost in packets in R1
        ldi     r1, 1
        lsl     r1, r1, QOS_SCHED_PACKETS_SCALE
        // Charge port packets if required
        qbbc    chargeCreditsPortPktNotNeeded, portCfg.flags, PORT_FLAGS_BIT_CIR_BY_PKTS
        sub     portProc.pktCirCurrent, portProc.pktCirCurrent, r1
        qbeq    chargeCreditsPortCirNoPktsLeft, portProc.pktCirCurrent, 0 // == 0
        qbbs    chargeCreditsPortCirNoPktsLeft, portProc.pktCirCurrent, 31 // < 0
chargeCreditsPortPktNotNeeded:
chargeCreditsPortPktDone:
        // Charge group packets if required
        qbbc    chargeCreditsGrpPktNotNeeded, groupCfg.flags, GROUP_FLAGS_BIT_CIR_BY_PKTS
        // Swap overlay
        sbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
        // using pktCirCurrent to avoid destroying r1 since that takes 2 inst to get back
        add     groupProcPktOv.pktCirCurrent, counters.groupProcOffset, SIZE(struct_GroupProcRegMap)
        lbco    groupProcPktOv, cLocalRAM, groupProcPktOv.pktCirCurrent, SIZE(groupProcPktOv)
        sub     groupProcPktOv.pktPirCurrent, groupProcPktOv.pktPirCurrent, r1
        testFPirCreditOnly chargeCreditsCheckPirPkts
        sub     groupProcPktOv.pktCirCurrent, groupProcPktOv.pktCirCurrent, r1
chargeCreditsPirPktsDone:
        // Swap overlay
        add     r1, counters.groupProcOffset, SIZE(struct_GroupProcRegMap)
        sbco    groupProcPktOv, cLocalRAM, r1, SIZE(groupProcPktOv)
        lbco    groupProcPktOv, cLocalRAM, counters.groupProcOffset, SIZE(groupProcPktOv)
chargeCreditsGrpPktNotNeeded:

        ret
#endif

// Precondition: port and group data structures are preloaded into
// state, groupCfg, groupProc, portCfg, portProc
// Returns in r0: bytes transmitted
logicalGroupScheduler:
    .using groupSchedScope
        pushRet
        // Save out groupSched and queueProcLite because they overlay portProc
        push    groupSched, SIZE(groupSched) + SIZE(queueProcLite)

    //if (timerExpired())  // this costs 1 PDSP cycle if timer didnt expire
    //{
    //    clearTimer();
    //    TimerTicks++;
    //}
#ifndef DROP_SCHED
        qbbc    logicalGroupTimerOK, r31.tStatus_Timer   // Look for a timer tick
        set     r31.tStatus_Timer                        // Clear the timer trigger                   
        add     static.timerTicks, static.timerTicks, 1  // Account for the tick
logicalGroupTimerOK:
#else
        call    dropSchedCheckTimerPushProxy
#endif
    //    // With queues, we can directly read the pending status
    //    PacketPendingMask = ReadQosQueuePendingBits();
        add     r1, counters.portQBaseIndex, counters.groupQBaseIndex
        add     r1, r1, static.queueBase
        and     r0, r1, 0x1f        // %= 32 to find bit in status register
        lsr     r1, r1, 5           // /= 32 to find status register #
        lsl     r1, r1, 2           // get register address in bytes
        lbco    r1, cQPending, r1, 4
        lsr     groupSched.packetPendingMask, r1, r0  // right justify this group in r0

    //    if(!PacketPendingMask)
    //        return 0;
        qbeq    logicalGroupNoPkts, groupSched.packetPendingMask, 0
        
        // Initialize variable
        add     groupSched.spPlusRR, groupCfg.SPCount, groupCfg.RRCount

        call    resetQueueState
        qbeq    logicalGroupNoSP, counters.queueIndex, groupCfg.SPCount
    //
    //    //
    //    // Try to take a high priority queue first
    //    //
    //    for( i=0; i<pGroup->SPCount; i++ )
    //    {
logicalGroupSPLoop:
    //        if( PacketPendingMask & (1<<i) )
    //            return( QosQueueScheduler(&pGroup->Queue[i]) );
        qbbc    logicalGroupSPNext, groupSched.packetPendingMask, counters.queueIndex
        call    queueScheduler
        jmp     logicalGroupDone
logicalGroupSPNext:
        call    moveQueueState
        qbgt    logicalGroupSPLoop, counters.queueIndex, groupCfg.SPCount
    //    }
    //
logicalGroupNoSP:
        // If no RR queues then skip to SP queues
        qbeq    logicalGroupNoRR, groupCfg.RRCount, 0

        ldi     groupSched.i, 0
    //        //
    //        // Next try to pick a round robin queue
    //        //
    //    if (PacketPendingMask & (((1 << pGroup->RRCount) - 1) << pGroup->SPCount))
    //    {
        ldi   r1, 1
        lsl   r1, r1, groupCfg.RRCount
        sub   r1, r1, 1
        lsl   r1, r1, groupCfg.SPCount
        and   r1, r1, groupSched.packetPendingMask
        qbeq  logicalGroupRRDone, r1, 0
    //        // There are RR packets pending
    //        for( i=0; i<pGroup->RRCount; i++ )
    //        {
logicalGroupRRLoop:
    //            // If all queues with WRR credit remaining are empty, reset the credit
    //            while ( !(pGroup->WrrCreditMask & PacketPendingMask) )
logicalGroupRRLoopAddWrrOuterLoop:
        and     r1, groupProc.wrrCreditMask, groupSched.packetPendingMask
        qbne    logicalGroupRRLoopAddWrrOuterLoopDone, r1, 0
        // Reset the queue to beginning
        call    resetQueueState
        // Move to first RR queue
logicalGroupRRLoopAddWrrOuterLoopResetState:
        qbeq    logicalGroupRRLoopAddWrrOuterLoopResetStateDone, counters.queueIndex, groupCfg.SPCount
        call    moveQueueState
        jmp     logicalGroupRRLoopAddWrrOuterLoopResetState

logicalGroupRRLoopAddWrrOuterLoopResetStateDone:
        mov     groupSched.j, groupCfg.SPCount
logicalGroupRRLoopAddWrrInnerLoop:
    //            {
    //                // Reset credits
    //                for(j=pGroup->SPCount; j<(pGroup->SPCount+pGroup->RRCount); j++)
    //                {
    //                    pGroup->Queue[j].WrrCurrentCredit += pGroup->Queue[j].WrrInitialCredit;
        lbco    queueProcLite, cLocalRAM, counters.queueProcOffset, SIZE(queueProcLite)
        add     queueProcLite.wrrCurrent, queueProcLite.wrrCurrent, queueCfg.wrrInitialCredit
    //                    if (pGroup->Queue[j].WrrCurrentCredit > (pGroup->Queue[j].WrrInitialCredit << 1))
    //                        pGroup->Queue[j].WrrCurrentCredit = (pGroup->Queue[j].WrrInitialCredit << 1);
        lsl     r0, queueCfg.wrrInitialCredit, 1
        qbbs    logicalGroupRRLoopAddWrrInnerLoopNoOverflow, queueProcLite.wrrCurrent, 31  // < 0
        qbge    logicalGroupRRLoopAddWrrInnerLoopNoOverflow, queueProcLite.wrrCurrent, r0  // r0 >= wrrCurrent
        mov     queueProcLite.wrrCurrent, r0
logicalGroupRRLoopAddWrrInnerLoopNoOverflow:
    //                    if (pGroup->Queue[j].WrrCurrentCredit > 0 || (! pGroup->Queue[j].WrrInitialCredit))
        qbeq    logicalGroupRRLoopAddWrrInnerLoopHasCredit, queueCfg.wrrInitialCredit, 0
        qbbs    logicalGroupRRLoopAddWrrInnerLoopNoCredit, queueProcLite.wrrCurrent, 31 // < 0
        qbeq    logicalGroupRRLoopAddWrrInnerLoopNoCredit, queueProcLite.wrrCurrent, 0  // == 0
logicalGroupRRLoopAddWrrInnerLoopHasCredit:
    //                        pGroup->WrrCreditMask |= (1<<j);
        set     groupProc.wrrCreditMask, groupProc.wrrCreditMask, groupSched.j
logicalGroupRRLoopAddWrrInnerLoopNoCredit:
        sbco    queueProcLite, cLocalRAM, counters.queueProcOffset, SIZE(queueProcLite)
        add     groupSched.j, groupSched.j, 1
        call    moveQueueState
        qbgt    logicalGroupRRLoopAddWrrInnerLoop, groupSched.j, groupSched.spPlusRR
    //                }
    //
        jmp     logicalGroupRRLoopAddWrrOuterLoop

logicalGroupRRLoopAddWrrOuterLoopDone:
        // Move queueIndex back to groupProc.nextQueue
        qbeq    logicalGroupRRLoopResetStateDone, counters.queueIndex, groupProc.nextQueue
        call    resetQueueState
logicalGroupRRLoopResetState:
        qbeq    logicalGroupRRLoopResetStateDone, counters.queueIndex, groupProc.nextQueue
        call    moveQueueState
        jmp     logicalGroupRRLoopResetState

logicalGroupRRLoopResetStateDone:
    //            }
    //
    //            // If the next queue has WRR credit and packets, then schedule a packet
    //            if( (pGroup->WrrCreditMask & PacketPendingMask) & (1<<pGroup->NextQueue) )
        and     r1, groupProc.wrrCreditMask, groupSched.packetPendingMask
        qbbc    logicalGroupRRNoPktThisQueue, r1, groupProc.nextQueue
    //            {
    //                // Attempt to schedule a packet
    //                BytesUsed = QosQueueScheduler( &pGroup->Queue[pGroup->NextQueue] );
        call    queueScheduler   // always sends one packet since pending bit was found
                                 // Even if it doesnt, it just means this group lost a slot
    //
    //                // Deduct the WRR credit
    //                if( pPort>fByteWrrCredits )
    //                    pGroup->Queue[pGroup->NextQueue].WrrCurrent -= (BytesUsed + pPort->overheadBytes - pPort->RemoveBytes) << BYTES_SCALE;
    //                else
    //                    pGroup->Queue[pGroup->NextQueue].WrrCurrent -= 1 << PACKETS_SCALE;
        qbeq    logicalGroupOops, r0, 0  // somehow packet was consumed by something other than QoS
        add     r1, r0, portCfg.overheadBytes
        sub     r1, r1, portCfg.removeBytes
        lsl     r1, r1, QOS_SCHED_WRR_BYTES_SCALE
        qbbs    logicalGroupRRUseBytes, portCfg.flags, PORT_FLAGS_BIT_WRR_BYTES
        ldi     r1, 1 
        lsl     r1, r1, QOS_SCHED_WRR_PACKETS_SCALE // because ldi limited to 16 bits
        
logicalGroupRRUseBytes:
        lbco    queueProcLite, cLocalRAM, counters.queueProcOffset, SIZE(queueProcLite)
        sub     queueProcLite.wrrCurrent, queueProcLite.wrrCurrent, r1
        sbco    queueProcLite, cLocalRAM, counters.queueProcOffset, SIZE(queueProcLite)
    //
    //                    // Clear the queuess WWR credit mask if we depleted the WRR credit
    //                    if( pGroup->Queue[pGroup->NextQueue].WrrCurrent <= 0 )
    //                        pGroup->WrrCreditMask &= ~(1<<pGroup->NextQueue);
        qbeq    logicalGroupRRNoWrr, queueProcLite.wrrCurrent, 0   // == 0
        qbbs    logicalGroupRRNoWrr, queueProcLite.wrrCurrent, 31  // < 0
        jmp     logicalGroupRRHasWrr
logicalGroupRRNoWrr:
logicalGroupOops:
        clr     groupProc.wrrCreditMask, groupProc.wrrCreditMask, groupProc.nextQueue
logicalGroupRRHasWrr:
    //                // Quit now if we moved a packet
    //                if(BytesUsed)
    //                    return(BytesUsed);
        call    moveWrrQueue // uses r1
        jmp     logicalGroupDone 
    //            }
    //        }
    //
logicalGroupRRNoPktThisQueue:
    //            // Move on to the next group
    //            pGroup->NextQueue++;
    //            if( pGroup->NextQueue == pGroup->SPCount+pGroup->RRCount )
    //                pGroup->NextQueue = pGroup->SPCount;
    //
        call    moveWrrQueue // uses r1
        qbgt    logicalGroupRRLoop, groupSched.i, groupCfg.RRCount
    //    }
logicalGroupRRDone:
        // move counters.queueIndex back to end of RR queues
        qbeq    logicalGroupNoRR, counters.queueIndex, groupSched.spPlusRR
        call    moveQueueState
        jmp     logicalGroupRRDone
       
logicalGroupNoRR:
    //
    //
    // Finally, try to get a packet from the OPTIONAL best effort queues
    //
        qbeq    logicalGroupNoPkts, counters.queueIndex, groupCfg.queueCount
    //    for( i=pGroup->SPCount+pGroup->RRCount; i<pGroup->QueueCount; i++ )
    //        {
logicalGroupBELoop:
    //            if( PacketPendingMask & (1<<i) )
    //                return( QosQueueScheduler(&pGroup->Queue[i]) );
        qbbc    logicalGroupBENextQ, groupSched.packetPendingMask, counters.queueIndex
        call    queueScheduler
        jmp     logicalGroupDone
    //        }
    //
    //        // No packet was transferred
    //        return(0);
logicalGroupBENextQ:
        call    moveQueueState
        qbgt    logicalGroupBELoop, counters.queueIndex, groupCfg.queueCount

    //    }

logicalGroupNoPkts:
        ldi     r0, 0  // no bytes transferred
logicalGroupDone:
        // Restore groupSched and queueProcLite because they overlay portProc
        pop     groupSched, SIZE(groupSched) + SIZE(queueProcLite)
        popRet
        ret

// Precondition: port, group and queue data structures are preloaded into
// state, groupCfg, groupProc, portCfg, portProc, queueCfg
// Moves groupProc.nextQueue between 
// [groupCfg.SPCount and groupCfg.SPCount + grouCfg.RRCount) and loads the
// queue state
moveWrrQueue:
        push    r30, 4 // save return address
    //        // Move on to the next group
    //        pGroup->NextQueue++;
    //        if( pGroup->NextQueue == pGroup->SPCount+pGroup->RRCount )
    //            pGroup->NextQueue = pGroup->SPCount;
        add     groupProc.nextQueue, groupProc.nextQueue, 1
        qbgt    moveWrrQueueNoRollover, groupProc.nextQueue, groupSched.spPlusRR
    // perform the rollover
        call    resetQueueState
        mov     groupProc.nextQueue, groupCfg.SPCount
moveWrrQueueNoRollover:
    // Advance counters.queueIndex to groupProc.nextQueue
moveWrrQueueLoop:
        qbeq    moveWrrQueueDone, counters.queueIndex, groupProc.nextQueue
        call    moveQueueState
        jmp     moveWrrQueueLoop
moveWrrQueueDone:
        pop     r30, 4 // restore return address
        ret

    .leave groupSchedScope

// Precondition: port, group and queue data structures are preloaded into
// state, groupCfg, groupProc, portCfg, portProc, queueCfg
// Returns in r0: bytes transmitted
#ifdef DEBUG_TRIGGER_ENABLE
#warn ******* Debug trigger (infinite loop!) enabled ***********
    .using queueDebugScope
#define QUEUE_SCHEDULER_SAVE_SIZE SIZE(queueProc) + SIZE(queueDebug)
#else
#define QUEUE_SCHEDULER_SAVE_SIZE SIZE(queueProc)
#endif
queueScheduler:
        // Save what is under queueProc
        push    queueProc, QUEUE_SCHEDULER_SAVE_SIZE
 
        // Load the queue dynamic structure
        lbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)

        // Find the queue register
        add     r0, counters.portQBaseIndex, counters.groupQBaseIndex
        add     r0, r0, counters.queueIndex
        add     r0, r0, static.queueBase
        lsl     r0, r0, 4        // * 16
        add     r0, r0, 8                           // Read C + D
        lbco    r0, cQBase, r0, 8  // Pop packet

#ifdef DEBUG_TRIGGER_ENABLE
        add     queueDebug.descAddr, r1, DEBUG_TRIGGER_OFFSET
        and     queueDebug.descAddr.b0, queueDebug.descAddr.b0, 0xf0
        lbbo    queueDebug.descReadVal, queueDebug.descAddr, 0, 4
        ldi32   queueDebug.descCheckVal, DEBUG_TRIGGER_VAL
queueSchedDebugTriggered:
        qbeq    queueSchedDebugTriggered, queueDebug.descCheckVal, queueDebug.descReadVal
#endif
        // Dont transfer a null packet as it would trash the outbound queue
        qbeq    queueSchedulerNullPkt, r1, 0

        // Update stats
        add     queueProc.packetsForwarded, queueProc.packetsForwarded, 1
        add     queueProc.bytesForwardedLSW, queueProc.bytesForwardedLSW, r0
        adc     queueProc.bytesForwardedMSW, queueProc.bytesForwardedMSW, 0
        sbco    queueProc, cLocalRAM, counters.queueProcOffset, SIZE(queueProc)

        // Calculate output queue -- re-using packetsForwarded, since its been stored
        lsl     queueProc.packetsForwarded, portCfg.destQueueNumber, 4
        add     queueProc.packetsForwarded, queueProc.packetsForwarded, 8 // C+D
        sbco    r0, cQBase, queueProc.packetsForwarded, 8  // Push packet
        
        // indicate packet is forwarded even if its size is 0
        set     r0, r0, 31
queueSchedulerNullPkt:
        // Restore queueProc
        pop     queueProc, QUEUE_SCHEDULER_SAVE_SIZE

        // Size is still in r0 from queue pop above
        ret
#ifdef DEBUG_TRIGGER_ENABLE
    .leave queueDebugScope
#endif

//-----------------------------------------------------------------------------
// Sets static.portIndex* to the first port
// Loads the first port into portCfg and portProc if enabled
//-----------------------------------------------------------------------------
resetPortState:
        // Prime the loop
        ldi     counters.portIndex, 0
        ldi     counters.portQBaseIndex, 0
#ifdef WIDE
        ldi     r1, QOSSCHED_OFF_PORT_CFGS
        lbco    portCfg, cLocalRAM, r1, SIZE(portCfg)
        ldi     r1, QOSSCHED_OFF_PORT_PROCS
        lbco    portProc, cLocalRAM, r1, SIZE(portProc)
#else
        ldi     counters.portCfgOffset, QOSSCHED_OFF_PORT_CFGS
        ldi     counters.portProcOffset, QOSSCHED_OFF_PORT_PROCS
        lbco    portCfg, cLocalRAM, counters.portCfgOffset, SIZE(portCfg)
        lbco    portProc, cLocalRAM, counters.portProcOffset, SIZE(portProc)
#endif
        ret

//-----------------------------------------------------------------------------
// Stores portProc back to memory.
// Moves counters.portIndex to the next port
// Loads portProc and portCfg if port is enabled
//-----------------------------------------------------------------------------
movePortState:
#ifdef WIDE
        // several callers of movePortState use r0
        ldi     r1, QOSSCHED_OFF_PORT_PROCS
        sbco    portProc, cLocalRAM, r1, SIZE(portProc)
        add     counters.portIndex, counters.portIndex, 1
#else
        sbco    portProc, cLocalRAM, counters.portProcOffset, SIZE(portProc)
        add     counters.portCfgOffset, counters.portCfgOffset, portProc.portCfgSize
        add     counters.portProcOffset, counters.portProcOffset, portProc.portProcSize
        add     counters.portQBaseIndex, counters.portQBaseIndex, portProc.portMaxQueues
        add     counters.portIndex, counters.portIndex, 1
        qbeq    movePortStatePortInvalid, counters.portIndex, QOS_SCHED_PORTS
        lbco    portCfg, cLocalRAM, counters.portCfgOffset, SIZE(portCfg)
        lbco    portProc, cLocalRAM, counters.portProcOffset, SIZE(portProc)
#endif
movePortStatePortInvalid:
        ret

//-----------------------------------------------------------------------------
// Sets counters.group* to the first group in the current port, as specified
// in portProc and portCfg.  Loads the first group into groupCfg and groupProc
//-----------------------------------------------------------------------------
resetGroupState:
        ldi     counters.groupIndex, 0
        ldi     counters.groupQBaseIndex, 0
#ifdef WIDE
        ldi     counters.groupCfgOffset, QOSSCHED_OFF_PORT_CFGS + SIZE(struct_PortCfgSize)
        ldi     counters.groupProcOffset, QOSSCHED_OFF_PORT_PROCS + SIZE(portProc)
#else
        add     counters.groupCfgOffset, counters.portCfgOffset, SIZE(struct_PortCfgSize)
        add     counters.groupProcOffset, counters.portProcOffset, SIZE(portProc)
#endif
        lbco    groupCfg, cLocalRAM, counters.groupCfgOffset, SIZE(groupCfg)
        lbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
        ret
//-----------------------------------------------------------------------------
// Stores groupProc back to memory.
// Moves counters.group* to the next group in the current port, as specified
// in portProc and portCfg
// Loads groupCfg and groupProc if groupIndex is valid
//-----------------------------------------------------------------------------
#ifdef MULTIGROUP
moveGroupState:
        sbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
        add     counters.groupCfgOffset, counters.groupCfgOffset, portProc.groupCfgSize
        add     counters.groupProcOffset, counters.groupProcOffset, portProc.groupProcSize
        add     counters.groupIndex, counters.groupIndex, 1
        add     counters.groupQBaseIndex, counters.groupQBaseIndex, portProc.groupMaxQueues
        // Skip the load if past the end of the groups in the port
        qble    moveGroupNoLoad, counters.groupIndex, portCfg.groupCount
        lbco    groupCfg, cLocalRAM, counters.groupCfgOffset, SIZE(groupCfg)
        lbco    groupProc, cLocalRAM, counters.groupProcOffset, SIZE(groupProc)
moveGroupNoLoad:
        ret
#endif

//-----------------------------------------------------------------------------
// Sets counters.queue to the first group in the current port and group, as specified
// in portProc, portCfg, groupProc, groupCfg.  Loads the first queue into 
// queueCfg (queueProc is not loaded)
//-----------------------------------------------------------------------------
resetQueueState:
        ldi     counters.queueIndex, 0
        add     counters.queueCfgOffset, counters.groupCfgOffset, SIZE(struct_GroupCfgSize)
        add     counters.queueProcOffset, counters.groupProcOffset, SIZE(struct_GroupProcSize)
        lbco    queueCfg, cLocalRAM, counters.queueCfgOffset, SIZE(queueCfg)
        ret

//-----------------------------------------------------------------------------
// Moves counters.queue to the next queue in the current group, as specified
// in portProc, portCfg, groupProc, groupCfg.
// Loads queueCfg if queueIndex is valid
//-----------------------------------------------------------------------------
moveQueueState:
        add     counters.queueCfgOffset, counters.queueCfgOffset, SIZE(struct_QueueCfgSize)
        add     counters.queueProcOffset, counters.queueProcOffset, SIZE(struct_QueueProc)
        add     counters.queueIndex, counters.queueIndex, 1
        qbbc    moveQueueNoJoint, portCfg.flags, PORT_FLAGS_BIT_IS_JOINT
        qbne    moveQueueNoJoint, counters.queueIndex, QOS_SCHED_LITE_QUEUES
        add     counters.queueCfgOffset, counters.queueCfgOffset, PORT_CFG_SIZE_LITE-(QOS_SCHED_LITE_QUEUES*SIZE(struct_QueueCfgSize))
        add     counters.queueProcOffset, counters.queueProcOffset, PORT_PROC_SIZE_LITE - (QOS_SCHED_LITE_QUEUES * SIZE(struct_QueueProc))
moveQueueNoJoint:
        // Skip the load if past the end of the groups in the port
        qble    moveQueueNoLoad, counters.queueIndex, groupCfg.queueCount
        lbco    queueCfg, cLocalRAM, counters.queueCfgOffset, SIZE(queueCfg)
moveQueueNoLoad:
        ret

.leave portProcessing

//-----------------------------------------------------------------------------
// Discards packet from head of queue index in r0
// Returns size of discarded packet in r0
//-----------------------------------------------------------------------------
discardPacket:
    .using discardPacketScope
        // Save overlay registers
        push    state, SIZE(state) + SIZE(desc)

        // Calculate the address of the C and D register
        lsl     r0, r0, 4                           // 16
        add     r0, r0, 8                           // Read C + D
        lbco    state.packetSize, cQBase, r0, 8  // Pop packet
        and     state.packetDescPtr.b0, state.packetDescPtr.b0, 0xF0
        mov     state.savedSize, state.packetSize
        // oops - somebody yanked the descriptor from under me
        qbeq    discardDone, state.packetDescPtr, 0

        // Read the descriptor
        lbbo    desc, state.packetDescPtr, 0, SIZE(desc)
        
        // Make sure of its type
        and     r0, desc.descInfo.b3, 0xC0
        qbeq    discardHostDescriptor, r0, 0
        qbeq    discardMonoDescriptor, r0, 0x80

        // If we get here, it aint good, but the best we can
        // do is ignore the problem and move on
        jmp     discardDone
       
discardHostDescriptor:
        // Jump if were splitting descriptors
        qbbs    discardHostDescriptorSplit, desc.PktInfo.t15
        mov     desc.NextPtr, 0                     // Force an end to the return loop
discardHostDescriptorSplit:
        mov     state.packetSize, 0                 // Assume normal push to tail
        qbbc    discardHostDescriptorNormalPush,  desc.PktInfo.t14
        set     state.packetSize.t31                // Push to head
discardHostDescriptorNormalPush:
        // Free the descriptor

        and     desc.PktInfo.b1, desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 0x08                        // Write to C+D
        sbco    state.packetSize, cQBase, r0, 8     // Return old desc 
        // Jump if were done returning host desc
        qbeq    discardDone, desc.NextPtr, 0

        mov     state.packetDescPtr, desc.NextPtr
        and     state.packetDescPtr.b0, state.packetDescPtr.b0, 0xF0 // Mask off the hint (if any)

        // Load the descriptor info and return it
        lbbo    desc, state.packetDescPtr, 0, SIZE(desc)
        jmp     discardHostDescriptorSplit

discardMonoDescriptor:
        // Free the descriptor
        and     desc.PktInfo.b1, desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 12                          // Write to D
        sbco    state.packetDescPtr, cQBase, r0, 4  // Return old desc 

discardDone:
#ifdef DROP_SCHED
        push    r30, 4 // save return address
// check clock and push proxy
        call    dropSchedCheckPushProxy
        pop     r30, 4
#endif
        // Recover return value
        mov     r0, state.savedSize
        // Restore overlay registers
        pop     state, SIZE(state) + SIZE(desc)
    .leave discardPacketScope

        ret


#ifdef DEBUG_WP
// This implements a watchpoint in scratch memory.  It can be changed
// to implement any kind of memory with a system address by changing the
// lbco to a lbbo.  
//
// In DEBUG_CONTEXT, the first 16 bytes are used to save r0-r3.  At
// the 32 bit word starting at DEBUG_CONTEXT+16, bit 0 is used as an enable
// Thus you can use the SoC debuger to turn this code on by writing "1" to this
// location.
//
// There is no hw watch point support.  You call debugWP using following
// code fragment from within the code to check for the overwrite.  Divide
// and conquer. 
//
//      mov     r29.w2, r30.w0
//      call    debugWP
//      mov     r30.w0, r29.w2

// This is the scratch address to look for
#define DEBUG_WP_ADDR 0x958
// This is the "bad" value to look for.
#define DEBUG_WP_VAL  0
        // r29.w2 is saved return address
        // r29.w0 is avail
debugWP:
        ldi    r29.w0, QOSSCHED_OFF_DEBUG_CONTEXT
        sbco   r0, cLocalRAM, r29.w0, 16  // save r0-r3
        add    r0.w2, r29.w0, 16
        lbco   r3, cLocalRAM, r0.w2, 4
        ldi    r0.w0, DEBUG_WP_ADDR
        ldi32  r1, DEBUG_WP_VAL
        lbco   r2, cLocalRAM, r0.w0, 4
        
        qbbc   debugDONE, r3, 0
debugLOOP:
        qbeq   debugLOOP, r2, r1        
debugDONE:
        lbco   r0, cLocalRAM, r29.w0, 16  // restore r0-r3
        ret
#endif 

.using portProcessing
   // r0: bytes
   // stack top: outputSpaceAvail variable
   // On return:
   // r0 0: no space left; 1: space left
   // r1 destroyed
physPortUpdateOutputSpace:
       pop     r1, 4
    //    if (*OutputSpaceAvail)
    //    {
       qbeq    physPortUpdateOutputSpaceOK, r1, 0
    //        if (pPort->fByteDestThrottle)
    //        {
    //            *OutputSpaceAvail -= BytesUsed + pPort->OverheadBytes - pPort->RemoveBytes;
    //        }
    //        else
    //        {
    //            (*OutputSpaceAvail)--;
    //        }
       qbbs    physPortUpdateOutputSpaceBytes, portCfg.flags, PORT_FLAGS_BIT_OUT_THROT_BYTES
       sub     r1, r1, 1
       jmp     physPortUpdateOutputSpaceCheck
physPortUpdateOutputSpaceBytes:
       sub     r1, r1, r0
       sub     r1, r1, portCfg.overheadBytes
       add     r1, r1, portCfg.removeBytes
    //
    //        if (*OutputSpaceAvail <= 0)
    //        {
    //            return 0;
    //        }
physPortUpdateOutputSpaceCheck:
       ldi     r0, 0
       qbeq    physPortUpdateOutputSpaceNotOK, r1, 0   // == 0
       qbbs    physPortUpdateOutputSpaceNotOK, r1, 31  // < 0
    //    }

physPortUpdateOutputSpaceOK:
       ldi     r0, 1
physPortUpdateOutputSpaceNotOK:
       push    r1, 4
       ret
.leave portProcessing

#ifdef DROP_SCHED
.using dropSchedProcessing

dropSchedPushStats:
       ret


dropSchedRand16:
    // uint16_t Rand16 (DROP_SCHED *dSched)
    // {
    //     static uint32_t last_rand;
    //     static int pos = 0;
    //     uint16_t this_result;
    // 
    //     if (pos < 12)
    //     {
    //         last_rand = taus88 (&dSched->rng_s1);
    //         pos = 32;
       qble    dropSchedRand16NoDraw, dropProc.randPos, 12  // jump if 12 >= randPos

// calculate a random 32 bit number using tausworthe from 
// http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme.ps.
// Exptected that dropCfg.rng_s1, dropCfg.rng_s2, and dropCfg.rng_s3
// are pre-loaded and will be stored after this call. 
    // uint32_t taus88 (uint32_t *seeds)
    // {   
    //     uint32_t b;
    //     b = (((seeds[0] << 13) ^ seeds[0]) >> 19);
    //     seeds[0] = (((seeds[0] & 4294967294u) << 12) ^ b);
       lsl     dropProc.lastRand, dropCfg.rng_s1, 13
       xor     dropProc.lastRand, dropProc.lastRand, dropCfg.rng_s1
       lsr     dropProc.lastRand, dropProc.lastRand, 19
       and     dropCfg.rng_s1.b0, dropCfg.rng_s1.b0, 0xfe // & 4294967294 = 0xffff fffe
       lsl     dropCfg.rng_s1, dropCfg.rng_s1, 12
       xor     dropCfg.rng_s1, dropCfg.rng_s1, dropProc.lastRand
    //     b = (((seeds[1] << 2) ^ seeds[1]) >> 25);
    //     seeds[1] = (((seeds[1] & 4294967288u) << 4) ^ b);
       lsl     dropProc.lastRand, dropCfg.rng_s2, 2
       xor     dropProc.lastRand, dropProc.lastRand, dropCfg.rng_s2
       lsr     dropProc.lastRand, dropProc.lastRand, 25
       and     dropCfg.rng_s2.b0, dropCfg.rng_s2.b0, 0xf8 // & 4294967288 = 0xffff fff8
       lsl     dropCfg.rng_s2, dropCfg.rng_s2, 4 
       xor     dropCfg.rng_s2, dropCfg.rng_s2, dropProc.lastRand
       
    //     b = (((seeds[2] << 3) ^ seeds[2]) >> 11);
    //     seeds[2] = (((seeds[2] & 4294967280u) << 17) ^ b);
       lsl     dropProc.lastRand, dropCfg.rng_s3, 3
       xor     dropProc.lastRand, dropProc.lastRand, dropCfg.rng_s3
       lsr     dropProc.lastRand, dropProc.lastRand, 11
       and     dropCfg.rng_s3.b0, dropCfg.rng_s3.b0, 0xf0 // & 4294967280 = 0xffff fff0
       lsl     dropCfg.rng_s3, dropCfg.rng_s3, 17
       xor     dropCfg.rng_s3, dropCfg.rng_s3, dropProc.lastRand

    //     return (seeds[0] ^ seeds[1] ^ seeds[2]);
    // }
       xor     dropProc.lastRand, dropCfg.rng_s1, dropCfg.rng_s2
       xor     dropProc.lastRand, dropProc.lastRand, dropCfg.rng_s3
       ldi     dropProc.randPos, 32
    //     }

dropSchedRand16NoDraw:
    //     this_result = (uint16_t)(last_rand << 4);
    //     last_rand >>= 8;
    //     pos -= 8;
       lsl     r0, dropProc.lastRand.w0, 4
       lsr     dropProc.lastRand, dropProc.lastRand, 8
       sub     dropProc.randPos, dropProc.randPos, 8

    //     return this_result;
    // }

       ret

.struct struct_inputSnapLocal
       .u32     scratch32
       .u32     pending
       .u16     addr
       .u16     queuePendAddr
       .u16     queuePendStorAddr
       .u16     queueBlockIdx
       .u16     depthPAddr
       .u16     res1
       .u8      thisQueueIdx
       .u8      res2
       .u8      lmbdval
       .u8      bf
.ends
.struct struct_outputSnapLocal
       .u32     scratch32
       .u8      thisOutProfIdx
       .u8      res
       .u16     outProfAddr
.ends

.enter dropSchedSnapInputScop
.assign struct_inputSnapLocal, R12,  R17, local
.assign struct_DropQueProf, R18, R18, dropQueProf
.leave dropSchedSnapInputScop
.enter dropSchedSnapOutputScop
.assign struct_outputSnapLocal, R12,  R13, local
.assign struct_DropOutProf, R14, R16, dropOutProf
.leave dropSchedSnapOutputScop

.struct struct_snapQueueInputArgs
       .u16     thisQueue
       .u16     packetsAddr
       .u16     bytesAddr
       .u16     res
.ends
.enter snapQueueInputArgs
.assign struct_snapQueueInputArgs, R0, R1, snapQueue
.leave snapQueueInputArgs
.struct struct_snapQueueLocal
       .u32     packets
       .u32     bytes
       .u32     peekAddr
       .u16     scratch16
       .u16     res
.ends
.enter snapQueue
.assign struct_snapQueueLocal, R19, R22, local
.assign struct_snapQueueInputArgs, r0, r1, args
.leave snapQueue

dropSchedSnapInput:
       pushRet
.using dropSchedSnapInputScop
    //    int      bf;
    //    uint16_t queueBlockIdx = 0;
    //    uint8_t  thisOutIdx;
    //

    //    // Read qpend bits for each of the queues
    //    for (bf = 0; bf < (NUM_DROP_INPUT_QUEUES+31)/32; bf++)
        lsr     local.queuePendAddr, dropProc.queueBase, 5  // register N = queue/32
        lsl     local.queuePendAddr, local.queuePendAddr, 2 // convert to bytes
        ldi     local.queuePendStorAddr, QOSSCHED_DROPSCHED_OFF_IN_PENDING_BITS
        ldi     local.queueBlockIdx, 0
        ldi     local.depthPAddr, QOSSCHED_DROPSCHED_OFF_IN_PKTS_PER_IN_Q
        ldi     local.bf, 0
dropSchedSnapInputQPendLoop:
    //    {
    //        uint32_t pending = ReadQosQueuePendingBits(dSched->BaseQueue + queueBlockIdx);
        lbco    local.pending, cQPending, local.queuePendAddr, 4
        add     local.queuePendAddr, local.queuePendAddr, 4

    //        uint32_t lmbdval;
    //        if ( (NUM_DROP_INPUT_QUEUES - queueBlockIdx) < 32 )
    //        {
    //           // Ignore unintended queues
    //           PacketPendingMask &= (1 << (NUM_DROP_INPUT_QUEUES - queueBlockIdx)) - 1;
    //        }
        qbgt    dropSchedSnapInputNoMask, local.queueBlockIdx, (QOSSCHED_DROPSCHED_NUM_INPUT_QUE - 32)
// Note: if this overflows 16 bits due to NUM_QUE % 32 > 16 then change to ldi32
        ldi     local.scratch32, (1 << (QOSSCHED_DROPSCHED_NUM_INPUT_QUE % 32)) - 1
        and     local.pending, local.pending, local.scratch32

dropSchedSnapInputNoMask:
    //        packets_present[bf] = pending;
        sbco    local.pending, cLocalRAM, local.queuePendStorAddr, 4
        add     local.queuePendStorAddr, local.queuePendStorAddr, 4
        
    //
    //        while ((lmbdval = lmbd32(pending, 1)) < 32)
dropSnapInputQueueLoop:
        lmbd    local.lmbdval, local.pending, 1
        qbeq    dropSnapInputQueueLoopEnd, local.lmbdval, 32

    //        {
    //            uint16_t thisQueueIdx = queueBlockIdx + lmbdval;
        add     local.thisQueueIdx, local.queueBlockIdx, local.lmbdval
    //            uint16_t thisQueueNum = thisQueueIdx + dSched->BaseQueue;
        calcDropQueueProfAddr local.addr, local.thisQueueIdx, local.scratch32
        lbco    dropQueProf, cLocalRAM, local.addr, SIZE(dropQueProf)
        qbeq    dropSnapInputQueueDisabled, dropQueProf.fEnabled, 0

    //            if (dSched->Queues[thisQueueIdx].fEnabled)
    //            {
.using snapQueueInputArgs
    //                DropSchedSnapQueue (thisQueueNum, depth_p + thisQueueIdx, NULL);
        add     snapQueue.thisQueue, local.thisQueueIdx, dropProc.queueBase
        add     snapQueue.packetsAddr, local.depthPAddr, local.thisQueueIdx
        ldi     snapQueue.bytesAddr, 0
        call    dropSchedSnapQue
    //            }
dropSnapInputQueueNoCap:
.leave snapQueueInputArgs
dropSnapInputQueueDisabled:
        clr     local.pending, local.lmbdval
    //            pending &= ~(1 << lmbdval);
    //        }
    //        queueBlockIdx += 32;
        jmp     dropSnapInputQueueLoop
dropSnapInputQueueLoopEnd:
        add     local.queueBlockIdx, local.queueBlockIdx, 32
        add     local.bf, local.bf, 1
        qbne    dropSchedSnapInputQPendLoop, local.bf, (QOSSCHED_DROPSCHED_NUM_INPUT_QUE+31)/32
    //    }
    //
    //}

.leave dropSchedSnapInputScop
        popRet
        ret

dropSchedSnapQue:
.using snapQueue
    //    uint32_t bytes = QueueByteLength (thisQueueNum);
    //    uint32_t packets = QueuePacketCount (thisQueueNum);
        lsl     local.peekAddr, args.thisQueue, 4
        lbco    local, cQPeekBase, local.peekAddr, 8
    //    if (packets > 255)
        qbge    dropSchedSnapQueueNoPktOvflw, local.packets, 255
    //    {
    //        packets = 255;
        ldi     local.packets, 255
    //    }
dropSchedSnapQueueNoPktOvflw:
    //    // Scaled such that 255 8K packets doesn't overflow
    //    bytes = ((bytes + 31) >> 5);
        add     local.bytes, local.bytes, 31
        lsr     local.bytes, local.bytes, 5
    //    if (bytes > 65535)
        qbeq    dropSchedSnapQueueNoByteOvflw, local.bytes.w2, 0
        ldi     local.bytes, 0xffff
    //    {
    //        bytes = 65535;
    //    }
dropSchedSnapQueueNoByteOvflw:
    //    if (depth_b32) 
    //    {
    //        *depth_b32 = (uint16_t)bytes;
    //    }
        qbeq    dropSchedSnapQueueNoBytes, args.bytesAddr, 0
        sbco    local.bytes.w0, cLocalRAM, args.bytesAddr, 2
dropSchedSnapQueueNoBytes:
    //    *depth_p   = (uint8_t)packets;
        sbco    local.packets.b0, cLocalRAM, args.packetsAddr, 1

.leave snapQueue
        ret

dropSchedSnapOutput:
        pushRet
.using dropSchedSnapOutputScop
.using snapQueueInputArgs
    //    uint8_t thisOutProfIdx;
    //    for (thisOutProfIdx = 0; thisOutProfIdx < NUM_DROP_OUTPUT_PROFILES; thisOutProfIdx++)
        ldi     local.thisOutProfIdx, 0
        ldi     local.outProfAddr, QOSSCHED_DROPSCHED_OFF_OUT_PROFS
        ldi     snapQueue.packetsAddr, QOSSCHED_DROPSCHED_OFF_OUT_PKTS_PER_OUT_PROF
        ldi     snapQueue.bytesAddr, QOSSCHED_DROPSCHED_OFF_OUT_BYTES_PER_OUT_PROF
dropSchedSnapOutputLoop:
    //    {
        lbco    dropOutProf, cLocalRAM, local.outProfAddr, SIZE(dropOutProf)
    //        if (dSched->OutProfiles[thisOutProfIdx].fEnabled)
    //        {
        qbeq    dropSchedSnapOutputDisabled, dropOutProf.fEnabled, 0
    //            uint16_t thisQueueNum = dSched->OutProfiles[thisOutProfIdx].DestQueueNumber;
    //            DropSchedSnapQueue (thisQueueNum, depth_p + thisOutProfIdx, depth_b32 + thisOutProfIdx);

        mov     snapQueue.thisQueue, dropOutProf.destQueueNumber
        call    dropSchedSnapQue
    //        }
dropSchedSnapOutputDisabled:
        add     snapQueue.packetsAddr, snapQueue.packetsAddr, 1
        add     snapQueue.bytesAddr, snapQueue.bytesAddr, 2
        add     local.thisOutProfIdx, local.thisOutProfIdx, 1
        add     local.outProfAddr, local.outProfAddr, SIZE(dropOutProf)
        qbgt    dropSchedSnapOutputLoop, local.thisOutProfIdx, QOSSCHED_DROPSCHED_NUM_OUT_PROFS 
    //    }

.leave snapQueueInputArgs
.leave dropSchedSnapOutputScop
        popRet
        ret

.struct struct_dropDisabledLocal
       .u32     pending
       .u32     scratch32
       .u32     pendingAfterDrop
       .u16     queuePendAddr
       .u16     queueEnAddr
       .u8      bf
       .u8      lmbdval
       .u16     queueNum
       .u16     dropQueueNum
       .u16     res1
.ends

.enter dropSchedDropDisabledScop
.assign struct_dropDisabledLocal, R12,  R17, local
.leave dropSchedDropDisabledScop

dropSchedDropDisabled:
        pushRet
.using dropSchedDropDisabledScop
    //    int      bf;
    //    uint16_t queueNum = dSched->BaseQueue;
        ldi     local.queuePendAddr, QOSSCHED_DROPSCHED_OFF_IN_PENDING_BITS
        ldi     local.queueEnAddr, QOSSCHED_DROPSCHED_OFF_QUE_EN_BITS 
        ldi     local.bf, 0
        mov     local.queueNum, dropProc.queueBase
    //    for (bf = 0; bf < (NUM_DROP_INPUT_QUEUES + 31) / 32; bf++)
    //    {
    //        uint32_t lmbdval;
    //        uint32_t pending = packets_present[bf];
    //        uint32_t enabled = dSched->queEnBits[bf];
    //        uint32_t disabled = enabled;
    //        uint32_t pendingAndDisabled = pending & disabled;
    //        uint32_t updatedPending     = pending & ~pendingAndDisabled;
    //        packets_present[bf] = updatedPending;
dropSchedDropDisBFLoop:
        lbco    local.scratch32, cLocalRAM, local.queuePendAddr, 4
        lbco    local.pending, cLocalRAM, local.queueEnAddr, 4
        not     local.pending, local.pending
        and     local.pending, local.pending, local.scratch32
        // pending now contains disabled queues with packets
        not     local.pendingAfterDrop, local.pending
        and     local.pendingAfterDrop, local.pendingAfterDrop, local.scratch32
        // clear those pending bits and store back
        sbco    local.pendingAfterDrop, cLocalRAM, local.queuePendAddr, 4
        add     local.queuePendAddr, local.queuePendAddr, 4
        add     local.queueEnAddr, local.queueEnAddr, 4

    //        while ((lmbdval = lmbd32(pendingAndDisabled, 1)) < 32)
    //        {
dropSchedDropDisPendLoop:
        lmbd    local.lmbdval, local.pending, 1
        qbeq    dropSchedDropDisPendLoopDone, local.lmbdval, 32
    //            uint32_t thisQueueNum = queueNum + lmbdval;
    //
    //            /* Drop without stats */
    //            while(DropPacket (thisQueueNum));
        
        add    local.dropQueueNum, local.queueNum, local.lmbdval
        lsl    r0, local.dropQueueNum, 4
        lbco   local.scratch32, cQPeekBase, r0, 4
        qbeq   dropSchedErrPkts, local.scratch32, 0
dropSchedDropDisLoop:
        mov    r0, local.dropQueueNum
        call   discardPacket
        sub    local.scratch32, local.scratch32, 1
        qbne   dropSchedDropDisLoop, local.scratch32, 0
    //            pendingAndDisabled &= ~(1 << lmbdval);
dropSchedErrPkts:
        clr     local.pending, local.lmbdval
        jmp     dropSchedDropDisPendLoop
dropSchedDropDisPendLoopDone:
    //        }
    //        queueNum += 32;
        add     local.queueNum, local.queueNum, 32
        add     local.bf, local.bf, 1
        qbne    dropSchedDropDisBFLoop, local.bf, (QOSSCHED_DROPSCHED_NUM_INPUT_QUE+31)/32
 
    //    }
    //
.leave dropSchedDropDisabledScop
        popRet
        ret

.struct struct_dropSchedLocalAvg
        .u32    bytesPend
        .u32    scratch32
        .u32    scratch32b
        .u16    outProfAddr
        .u16    res
        .u16    outDepthBAddr
        .u16    thisProb
        .u16    addr
        .u16    dropProbAddr
        .u8     thisOutProfIdx
        .u8     queueBlockIdx
        .u8     lmbdval
        .u8     scratch8
.ends

.struct struct_dropSchedLocalRun
        .u32    pending
        .u32    scratch32
        .u32    addr
        .u16    queuePendAddr
        .u16    thisQueueNum
        .u16    dropProb
        .u16    statsAddr
        .u16    depthB
        .u8     depthP
        .u8     lmbdval
        .u8     bf
        .u8     queueBlockIdx
        .u8     fwdPkts
        .u8     thisQueueIdx
.ends
.struct struct_OneStat
        .u32    bytes
        .u32    packets
.ends

.enter dropSchedScop
    .assign struct_DropOutProf, R12, R14, outProf
    .assign struct_DropCfgProf, R15, R19, cfgProf
.leave dropSchedScop
.enter dropSchedAvgScop
    .assign struct_dropSchedLocalAvg, R20,  R26, local
.leave dropSchedAvgScop
.enter dropSchedRunScop
    .assign struct_DropQueProf, R20, R20, dropQue
    .assign struct_dropSchedLocalRun, R21, R27, local
    .assign struct_OneStat, R28, R29, stats
.leave dropSchedRunScop
.enter pushStatsScop
    .assign struct_DropCfgStatsQPair, R21, R21, statsQPair
    .assign struct_PushStats, R22, R26, pushStats
    .assign struct_DropStats, R27, R30, dropStats // R30 is OK, its pushed
.leave pushStatsScop

dropSchedSched:
        pushRet
.using dropSchedScop
.using dropSchedRunScop

    //     // Step through each pending bitfield and process all
    //     // queues with input packets
        ldi     local.bf, 0
        ldi     local.queueBlockIdx, 0
        ldi     local.queuePendAddr, QOSSCHED_DROPSCHED_OFF_IN_PENDING_BITS
dropSchedBfLoop:
    //     for (bf = 0; bf < (NUM_DROP_INPUT_QUEUES + 31) / 32; bf++)
    //     {
    //         uint32_t lmbdval;
    //         uint32_t pending = in_packets_present[bf];
    //         int needInt = 0;
        lbco    local.pending, cLocalRAM, local.queuePendAddr, 4
    //         while ((lmbdval = lmbd32(pending, 1)) < 32)
    //         {
dropSchedPendLoop:
        lmbd    local.lmbdval, local.pending, 1
        qbeq    dropSchedPendLoopDone, local.lmbdval, 32
    //             uint16_t thisQueueIdx = queueBlockIdx + lmbdval;
        add     local.thisQueueIdx, local.queueBlockIdx, local.lmbdval
        calcDropQueueProfAddr local.addr, local.thisQueueIdx, local.scratch32
        lbco    dropQue, cLocalRAM, local.addr, SIZE(dropQue)

    //             uint16_t thisQueueNum = thisQueueIdx + dSched->BaseQueue;
        add     local.thisQueueNum, local.thisQueueIdx, dropProc.QueueBase
    //             uint8_t thisStatsIdx = dSched->Queues[thisQueueIdx].StatsBlockIdx;
    //             uint8_t thisOutIdx = dSched->Queues[thisQueueIdx].OutProfIdx;
    //             uint8_t thisProfIdx = dSched->OutProfiles[thisOutIdx].CfgProfIdx;
    //             uint8_t fwdPkts;
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_IN_PKTS_PER_IN_Q
        add     local.addr, local.addr, local.thisQueueIdx
        lbco    local.fwdPkts, cLocalRAM, local.addr, 1
        // This could happen if somebody else yanks descriptors from
        // under us, avoid semi infinite loop
        qbeq    dropSchedNoDrain, local.fwdPkts, 0
        calcDropOutProfAddr local.addr, dropQue.outProfIdx, local.scratch32
        lbco    outProf, cLocalRAM, local.addr, SIZE(outProf)
        calcDropCfgProfAddr local.addr, outProf.cfgProfIdx, local.scratch32
        lbco    cfgProf, cLocalRAM, local.addr, SIZE(cfgProf)
        calcDropStatsAddr local.statsAddr, dropQue.statsIdx, local.scratch32
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_OUT_BYTES_PER_OUT_PROF
        add     local.addr, local.addr, dropQue.outProfIdx
        add     local.addr, local.addr, dropQue.outProfIdx  // 2 add for *2
        lbco    local.depthB, cLocalRAM, local.addr, 2
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_OUT_PKTS_PER_OUT_PROF
        add     local.addr, local.addr, dropQue.outProfIdx
        lbco    local.depthP, cLocalRAM, local.addr, 1
        
    //             for (fwdPkts = in_depth_p[thisQueueIdx]; fwdPkts; fwdPkts--)
    //             {
dropSchedDrainLoop:
        call    dropSchedCheckTimerPushProxy
    //                 // Don't need to check enable since in_packets_present already
    //                 // compensated for disabled queues
    //                 bool    dropPacket = 0;
    // 
    //                 // Tail drop block
    //                 if (dSched->CfgProfiles[thisProfIdx].TailThresh)
        qbeq    dropSchedNoTailDrop, cfgProf.tailThresh, 0
    //                 {
    //                     if (dSched->CfgProfiles[thisProfIdx].fByteTailThresh)
        qbbc    dropSchedTailDropPkts, cfgProf.unitFlags, DROP_UNIT_FLAGS_BIT_TAILDROP_BYTES
    //                     {
    //                         if ((out_depth_b32[thisOutIdx] << 5) >=
    //                             dSched->CfgProfiles[thisProfIdx].TailThresh)
        lsl     local.scratch32, local.depthB, 5
        qbgt    dropSchedNoTailDrop, local.scratch32, cfgProf.tailThresh
    //                         {
    //                             dropPacket = 1;
        jmp     dropSchedDropPkt
    //                         }
    //                     }
    //                     else
    //                     {
dropSchedTailDropPkts:
    //                         if (out_depth_p[thisOutIdx] >=
    //                             dSched->CfgProfiles[thisProfIdx].TailThresh)
    //                         {
    //                             dropPacket = 1;
        qble    dropSchedDropPkt, local.depthP, cfgProf.tailThresh
    //                         }
    //                     }
    //                 }
dropSchedNoTailDrop:
    // 
    //                 // RED drop block
    //                 if (!dropPacket && (dSched->CfgProfiles[thisProfIdx].Mode != DROP_MODE_TAIL_ONLY))
        qbeq    dropSchedFwdPkt, cfgProf.mode, DROP_MODE_TAIL_ONLY
    //                 {
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_PROBS
        add     local.addr, local.addr, dropQue.outProfIdx
        add     local.addr, local.addr, dropQue.outProfIdx  // 2 bytes each
        lbco    local.dropProb, cLocalRAM, local.addr, 2
    //                     if (dropProb[thisOutIdx] == 0xffff)
        ldi     local.scratch32, 0xffff
        qbeq    dropSchedDropPkt, local.dropProb, local.scratch32
    //                     {
    //                         dropPacket = 1;
    //                     }
    //                     else if (dropProb[thisOutIdx] == 0)
        qbeq    dropSchedFwdPkt, local.dropProb, 0
    //                     {
    //                         dropPacket = 0;
    //                     }
    //                     else
    //                     {
        call    dropSchedRand16
        qbge    dropSchedFwdPkt, local.dropProb, r0
    //                         if (dropProb[thisOutIdx] <= Rand16(dSched))
    //                         {
    //                             dropPacket = 1;
    //                         }
    //                         else
    //                         {
    //                             dropPacket = 0;
    //                         }
    //                     }
    //                 }
    // 
    //                 // Execute drop block
    //                 if (dropPacket)
    //                 {
dropSchedDropPkt:
    //                     // drop 1 packet and count it
        mov     r0, local.thisQueueNum
        call    discardPacket
        add     local.addr, local.statsAddr, 8
        lbco    stats, cLocalRAM, local.addr, 8
        add     stats.packets, stats.packets, 1
        add     stats.bytes, stats.bytes, r0
        sbco    stats, cLocalRAM, local.addr, 8
        qbbs    dropSchedIntNeeded, stats.packets, 31
        qbbc    dropSchedDrainLoopEnd, stats.bytes, 31
        jmp     dropSchedIntNeeded
    //                     uint32_t bytes = DropPacket (thisQueueNum);
    //                     dSched->StatsBlocks[thisStatsIdx].PacketsDropped ++;
    //                     needInt |= dSched->StatsBlocks[thisStatsIdx].PacketsDropped >> 31;
    //                     dSched->StatsBlocks[thisStatsIdx].BytesDropped += bytes;
    //                     needInt |= dSched->StatsBlocks[thisStatsIdx].BytesDropped >> 31;
    //                 }
    //                 else
    //                 {
dropSchedFwdPkt:
    //                     // Forward 1 packet and count it
    //                     uint32_t bytes = TransferPacket(dSched->OutProfiles[thisOutIdx].DestQueueNumber, thisQueueNum);
        lsl     local.addr, local.thisQueueNum, 4
        add     local.addr, local.addr, 8
        lbco    r0, cQBase, local.addr, 8  // read c&d
        // Don't zap output queue if somebody ripped the descriptors from
        // under us (push NULL zaps whole queue and leaks descs)
        qbeq    dropSchedFwdPoppedNull, r1, 0
        lsl     local.addr, outProf.destQueueNumber, 4
        add     local.addr, local.addr, 8
        sbco    r0, cQBase, local.addr, 8  // write c&d
dropSchedFwdPoppedNull:
        lbco    stats, cLocalRAM, local.statsAddr, 8
        add     stats.bytes, stats.bytes, r0
        add     stats.packets, stats.packets, 1
        sbco    stats, cLocalRAM, local.statsAddr, 8
        qbeq    dropSchedFwdPDepthNoOv, local.depthP, 0xff
        add     local.depthP, local.depthP, 1
dropSchedFwdPDepthNoOv:
        add     r0, r0, 31
        lsr     r0, r0, 5
        add     r0, local.depthB, r0
        ldi     r1, 0xffff
        qbgt    dropSchedFwdBDepthNoOv, r0, r1
        mov     r0, r1
dropSchedFwdBDepthNoOv:
        mov     local.depthB, r0
        qbbs    dropSchedIntNeeded, stats.packets, 31
        qbbc    dropSchedDrainLoopEnd, stats.bytes, 31
    //                     dSched->StatsBlocks[thisStatsIdx].PacketsForwarded ++;
    //                     needInt |= dSched->StatsBlocks[thisStatsIdx].PacketsForwarded >> 31;
    //                     dSched->StatsBlocks[thisStatsIdx].BytesForwarded += bytes;
    //                     needInt |= dSched->StatsBlocks[thisStatsIdx].BytesForwarded >> 31;
    //                     
    //                     // Update output queue depth so tail drop "sees it".
    //                     out_depth_b32[thisQueueIdx] += (bytes + 31) >> 5;
    //                     out_depth_p[thisQueueIdx] ++;
    //                 }
    // 
dropSchedIntNeeded:
    //                 if (needInt)
    //                 {
    //                     GenInt (dSched, thisProfIdx);
    //                 }
    //
        // Is push stat enabled?
        qbeq    dropSchedDrainLoopEnd, dropQue.statsQProf, 0

        // -4 because statsQProf is 1 based
        ldi     r0.w0, QOSSCHED_DROPSCHED_OFF_CFG + SIZE(dropCfg) - 4
        lsl     r0.w2, dropQue.statsQProf, 2 // * 4
        add     r0.w2, r0.w0, r0.w2
        mov     r0.w0, local.statsAddr

.using pushStatsScop
        push    statsQPair, SIZE(pushStats)+SIZE(dropStats)+SIZE(statsQPair)
        // Load the queue numbers
        lbco    statsQPair, cLocalRAM, r0.w2, SIZE(statsQPair)
        // pop a descriptor
        lsl     r1, statsQPair.src, 4
        add     r1, r1, 0xc
        lbco    r1, cQBase, r1, 4  
        // Did we get one?
        qbeq    dropSchedNoPushStats, r1, 0

        // Form the stats packet
        lbco    dropStats, cLocalRAM, r0.w0, SIZE(dropStats)
        mov     pushStats.packetsDropped, dropStats.packetsDropped
        mov     pushStats.bytesDropped, dropStats.bytesDropped,
        mov     pushStats.packetsForwarded, dropStats.packetsForwarded
        mov     pushStats.bytesForwarded, dropStats.bytesForwarded
        // write back 0s
        zero    &dropStats, SIZE(dropStats)
        sbco    dropStats, cLocalRAM, r0.w0, SIZE(dropStats)
        mov     pushStats.size, SIZE(pushStats)
        mov     pushStats.statsIdx, dropQue.statsIdx

        mov     r0, r1

        // lose hint bits
        and     r0.b0, r0.b0, 0xf0
        // put the stats in the descriptor
        sbbo    pushStats, r0, 0, SIZE(pushStats)

        // put the descriptor (with hint bits) on return queue
        lsl     r0, statsQPair.dst, 4
        add     r0, r0, 0xc
        sbco    r1, cQBase, r0, 4

dropSchedNoPushStats:
        pop     statsQPair, SIZE(pushStats)+SIZE(dropStats)+SIZE(statsQPair)
.leave pushStatsScop
dropSchedDrainLoopEnd:
        sub     local.fwdPkts, local.fwdPkts, 1
        qbne    dropSchedDrainLoop, local.fwdPkts, 0
    //             }
    // 
    //             pending &= ~(1 << lmbdval);
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_OUT_BYTES_PER_OUT_PROF
        add     local.addr, local.addr, dropQue.outProfIdx
        add     local.addr, local.addr, dropQue.outProfIdx  // 2 add for *2
        sbco    local.depthB, cLocalRAM, local.addr, 2
        ldi     local.addr, QOSSCHED_DROPSCHED_OFF_OUT_PKTS_PER_OUT_PROF
        add     local.addr, local.addr, dropQue.outProfIdx
        sbco    local.depthP, cLocalRAM, local.addr, 1
        
dropSchedNoDrain:
        clr     local.pending, local.pending, local.lmbdval
        jmp     dropSchedPendLoop
dropSchedPendLoopDone:
    //         }
    //         queueBlockIdx += 32;
        add     local.queueBlockIdx, local.queueBlockIdx, 32
        add     local.bf, local.bf, 1
        add     local.queuePendAddr, local.queuePendAddr, 4
        qbne    dropSchedBfLoop, local.bf, (QOSSCHED_DROPSCHED_NUM_INPUT_QUE+31)/32
    //     }

    // }
.leave dropSchedRunScop
.using dropSchedAvgScop
    // {
    //     int      que;
    //     int      bf;
    //     uint16_t queueBlockIdx = 0;
    //     uint16_t dropProb[NUM_DROP_OUTPUT_PROFILES];
    //     uint8_t  thisOutProfIdx;
    // 
    //     // Update averages every time, so 0's propegate and compute drop prob
    //     for (thisOutProfIdx = 0; thisOutProfIdx < NUM_DROP_OUTPUT_PROFILES; thisOutProfIdx++)
    //     {
        ldi     local.thisOutProfIdx, 0
        ldi     local.outProfAddr, QOSSCHED_DROPSCHED_OFF_OUT_PROFS
        ldi     local.outDepthBAddr, QOSSCHED_DROPSCHED_OFF_OUT_BYTES_PER_OUT_PROF
        ldi     local.dropProbAddr, QOSSCHED_DROPSCHED_OFF_PROBS
dropSchedUpdateAvgLoop:
        call    dropSchedCheckTimerPushProxy
        lbco    outProf, cLocalRAM, local.outProfAddr, SIZE(outProf)
    // 
    //         if (dSched->OutProfiles[thisOutProfIdx].fEnabled)
    //         {
        qbeq    dropSchedOutProfDis, outProf.fEnabled, 0
    //             // Find new average, presuming all the packets are forwarded
    //             uint32_t QAvg = dSched->OutProfiles[thisOutProfIdx].QAvg;
    //             // bytes_pend = out;
    //             uint32_t bytes_pend = (out_depth_b32[thisOutProfIdx] << 5);
        lbco    local.bytesPend.w0, cLocalRAM, local.outDepthBAddr, 2
        lsl     local.bytesPend, local.bytesPend.w0, 5
    //             uint16_t thisProb;
    //             uint8_t  thisCfgProfIdx = dSched->OutProfiles[thisOutProfIdx].CfgProfIdx;
        calcDropCfgProfAddr local.addr, outProf.cfgProfIdx, local.scratch32
        lbco    cfgProf, cLocalRAM, local.addr, SIZE(cfgProf)

    //             DROP_OUTPUT_PROFILE *out_p = &dSched->OutProfiles[thisOutProfIdx];
    //             DROP_CFG_PROFILE *cfg_p = &dSched->CfgProfiles[thisCfgProfIdx];
    //             if (cfg_p->Mode != DROP_MODE_TAIL_ONLY)
        qbeq    dropSchedNoAvgTailOnly, cfgProf.mode, DROP_MODE_TAIL_ONLY
    //             {
    //                 QAvg += bytes_pend - (QAvg >> cfg_p->TC);
    //                 dSched->OutProfiles[thisOutProfIdx].QAvg = QAvg;
        lsr     local.scratch32, outProf.QAvg, cfgProf.tcShift
        sub     local.scratch32, local.bytesPend, local.scratch32;
        add     outProf.QAvg, outProf.QAvg, local.scratch32
        add     local.addr, local.outProfAddr, OFFSET(outProf.QAvg)
        sbco    outProf.QAvg, cLocalRAM, local.addr, 4
    // 
    //                 // Determine drop probability 
    //                 if (QAvg <= cfg_p->RedThreshLow)
    //                 {
        qblt    dropSchedAboveLowThresh, outProf.QAvg, cfgProf.REDLow
    //                     thisProb = 0;
        ldi     local.thisProb, 0
        jmp     dropSchedProbDone
    //                 }
dropSchedAboveLowThresh:
    //                 else if (QAvg < cfg_p->RedThreshHigh)
    //                 {
        qble    dropSchedAboveHighThresh, outProf.QAvg, cfgProf.REDHigh
    //                     uint32_t threshDiff = (QAvg - cfg_p->RedThreshLow) >> cfg_p->TC;
        sub     local.scratch32, outProf.QAvg, cfgProf.REDLow
        lsr     local.scratch32, local.scratch32, cfgProf.tcShift
    //                     thisProb = mulProb (threshDiff, cfg_p->RedHighMLowRecip, out_p->RedProb);
    //    // Calculates
    //    // res32 = dif*recip
    //    // res64 = res32*prob
    //    // return res64 >> 32
    //    //
    //    if (dif < recip)
    //    {
    //        a = dif;
    //        b = recip;
        mov     r0, local.scratch32
        mov     r1, cfgProf.threshRecip
        qblt    dropSchedRecipSmaller, cfgProf.threshRecip, local.scratch32 // diff < threshRecip
    //    }
    //    else
    //        a = recip;
    //        b = dif;
    //    {
        mov     r0, cfgProf.threshRecip
        mov     r1, local.scratch32
    //    }
dropSchedRecipSmaller:
    //    res32 = 0;
        ldi     local.scratch32, 0
    //    while ( (lmbdval = lmbd32 (a, 1) ) != 32 )
    //    {
dropSchedm32x32Loop:
        lmbd    local.lmbdval, r0, 1
        qbeq    dropSchedm32x32done, local.lmbdval, 32
    //        res32 += b << lmbdval;
        lsl     local.scratch32b, r1, local.lmbdval
        add     local.scratch32, local.scratch32, local.scratch32b
    //        a &= ~ (1<<lmbdval);
        clr     r0, r0, local.lmbdval
        jmp     dropSchedm32x32Loop
    //    }
    //    res64 = 0;
dropSchedm32x32done:
        ldi     r0, 0  // LSW
        ldi     local.thisProb, 0
        mov     r1.w2, outProf.redProb
    //    while ( (lmbdval = lmbd32 (prob, 1)) != 32)
    //    {
dropSchedm32x16Loop:
        lmbd    local.lmbdval, r1.w2, 1
        qbeq    dropSchedm32x16done, local.lmbdval, 32
    //        res64 += ((uint64_t)res32) << lmbdval;
        rsb     local.scratch8, local.lmbdval, 32
        lsl     local.scratch32b, local.scratch32, local.lmbdval
        add     r0, r0, local.scratch32b
        lsr     local.scratch32b, local.scratch32, local.scratch8
        adc     local.thisProb, local.thisProb, local.scratch32b
    //        prob &= ~ (1<<lmbdval);
        clr     r1.w2, r1.w2, local.lmbdval
        jmp     dropSchedm32x16Loop
    //    }
dropSchedm32x16done:
    //    return (uint16_t)(res64 >> 32);
        jmp     dropSchedProbDone
    //                 }
    //                 else 
    //                 {
dropSchedAboveHighThresh:
    //                     thisProb = 0xffff;
        ldi     local.thisProb, 0xffff
    //                 }
dropSchedProbDone:
    //                 dropProb[que] = thisProb;
        sbco    local.thisProb, cLocalRAM, local.dropProbAddr, 2
    //             }
dropSchedNoAvgTailOnly:
    //         }

dropSchedOutProfDis:
        add     local.dropProbAddr, local.dropProbAddr, 2
        add     local.thisOutProfIdx, local.thisOutProfIdx, 1
        add     local.outProfAddr, local.outProfAddr, SIZE(outProf)
        add     local.outDepthBAddr, local.outDepthBAddr, 2
        qbgt    dropSchedUpdateAvgLoop, local.thisOutProfIdx, QOSSCHED_DROPSCHED_NUM_OUT_PROFS
    //     }
    // 
.leave dropSchedAvgScop
.leave dropSchedScop
        popRet
        ret

.struct struct_setEnableLocal
        .u32    queuecfg
        .u32    enBits
        .u8     idx
        .u8     bit
        .u16    queueCfgAddr
        .u16    enBitsAddr
        .u16    res1
.ends

.enter dropSchedSetEnableLocalScop
.assign struct_setEnableLocal, R12,  R15, local
.assign struct_DropQueProf, R12, R12, dropQueProf

.leave dropSchedSetEnableLocalScop

// New config in scratch, update enable bits
dropSchedSetEnBits:
.using dropSchedSetEnableLocalScop
        push    local, SIZE(local)
        ldi     local.idx, 0
        ldi     local.queueCfgAddr, QOSSCHED_OFF_PORT_SHADOW
        ldi     local.enBitsAddr, QOSSCHED_DROPSCHED_OFF_QUE_EN_BITS
        ldi     local.enBits, 0
dropSchedSetEnBitsLoop:
        lbco    dropQueProf, cLocalRAM, local.queueCfgAddr, SIZE(dropQueProf)
        add     local.queueCfgAddr, local.queueCfgAddr, 4
        and     local.bit, local.idx, 0x1f
        qbeq    dropSchedSetEnNotEn, dropQueProf.fEnabled, 0
        set     local.enBits, local.bit
dropSchedSetEnNotEn:
        qbne    dropSchedSetEnNoNewWord, local.bit, 0x1f
        sbco    local.enBits, cLocalRAM, local.enBitsAddr, 4
        add     local.enBitsAddr, local.enBitsAddr, 4
        ldi     local.enBits, 0
dropSchedSetEnNoNewWord:
        add     local.idx, local.idx, 1
        qbgt    dropSchedSetEnBitsLoop, local.idx, QOSSCHED_DROPSCHED_NUM_QUE_PROFS
        qbeq    dropSchedSetEnNoWrite, local.bit, 0x1f
        sbco    local.enBits, cLocalRAM, local.enBitsAddr, 4
dropSchedSetEnNoWrite:
        pop     local, SIZE(local)
.leave dropSchedSetEnableLocalScop
        ret

.leave dropSchedProcessing

.struct struct_ProxyScratch
        .u16    scratchAddr
        .u16    res
        .u32    queueOffset
.ends

.enter pushProxyScop
.assign struct_PushProxy, R0, R1, proxy
.assign struct_ProxyScratch, R2, R3, proxyScratch


.leave pushProxyScop

.using pushProxyScop
// warning: overwrites r0/r1
// checks if a push is pending (both size and ptr nonzero) as fast as possible
// if pending, save some regs and do the push
dropSchedCheckTimerPushProxy:
        // Poll the timer
        qbbc    pushProxyTimerOK, r31.tStatus_Timer   // Look for a timer tick
        set     r31.tStatus_Timer                        // Clear the timer trigger                   
        add     static.timerTicks, static.timerTicks, 1  // Account for the tick
pushProxyTimerOK:
dropSchedCheckPushProxy:
        ldi    r0, QOSSCHED_DROPSCHED_OFF_PUSHPROXY
        lbco   proxy, cLocalRAM, r0, SIZE(proxy)
        qbeq   pushProxyNotReady, r0, 0
        qbeq   pushProxyNotReady, r1, 0

        // save regs
        push   proxyScratch, SIZE(proxyScratch)
        // calculate the queue
        lsl    proxyScratch.queueOffset, proxy.qNum, 4
        add    proxyScratch.queueOffset, proxyScratch.queueOffset, 8
        // 0 fill the size
        mov    r0, proxy.descSize 
        // do the push
        sbco   proxy, cQBase, proxyScratch.queueOffset, SIZE(proxy)

        // Wipe the accepted packet
        zero   &proxy, SIZE(proxy)
        ldi    proxyScratch.scratchAddr, QOSSCHED_DROPSCHED_OFF_PUSHPROXY
        sbco   proxy, cLocalRAM, proxyScratch.scratchAddr, SIZE(proxy)

        pop    proxyScratch, SIZE(proxyScratch)
pushProxyNotReady:

        ret
.leave pushProxyScop

#endif

.codeword   PORT_CFG_SIZE_LITE
.codeword   PORT_CFG_SIZE_FULL
.codeword   PORT_PROC_SIZE_LITE
.codeword   PORT_PROC_SIZE_FULL
.codeword   SIZE(struct_PortCfgSize) 
.codeword   SIZE(struct_GroupCfgSize) * QOS_SCHED_FULL_GROUPS 
.codeword   SIZE(struct_QueueCfgSize) * QOS_SCHED_FULL_QUEUES
#ifdef DROP_SCHED
.codeword   DROPSCHED_SIZE_OUT_PROFS
.codeword   DROPSCHED_SIZE_CFG_PROFS
.codeword   DROPSCHED_SIZE_QUE_PROFS
.codeword   DROPSCHED_SIZE_STATS_BLOCKS
.codeword   DROPSCHED_SIZE_CFG
.codeword   DROPSCHED_SIZE_PUSH_STATS
#endif


