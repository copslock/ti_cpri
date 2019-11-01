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
// Session Router Firmware for Puma5
//----------------------------------------------------------------------------------
//
// qosrr2.p - Quality of Service Firmware
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
// 29-Mar-11 : MAD : Version 1.01 : Fixed bug in discard return queue masking
// 10-Aug-11 : MAD : Version 1.02 : Integrate bug fixes from QOS2 trial
// 11-Aug-11 : MAD : Version 1.03 : Add "Round Robin" mode cluster
// 20-Dec-11 : MAD : Version 1.04 : Fixed 2 bugs in RR mode, added SRIO monitoring
// 04-Jan-12 : MAD : Version 1.05 : Changed SRIO monitoring to strict SP and allow
//                                  it to move multiple packets per loop iteration
// 19-Jan-12 : John Dowdal : Version 1.06 : hint field should be 4 bits, not 5.
// 19-Jun-12 : John Dowdal : Version 1.0.0.7 : Make SRIO garbage queues configurable
//                                             and export version in scratch
// 06-Aug-12 : John Dowdal : Version 1.0.0.8 : Keystone 2 compatibility
// 20-Oct-15 : John Dowdal : Version 1.0.0.9 : Enable K1
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

#define QUEUES              64          // 32, 64
#define RAM_SIZE            0x1000      
#define cLocalRAM           cLRAM3
#define TIMER_IPS           100000      // 10us=100000
#define TIMER_SETTING       ((350000000/TIMER_IPS)/2)
#define QUEUE_BASE          320
#define PROFILEID           12

//
// ENDIAN DEPENDENT STRUCTURES
//
// Structures that are shared with the host must be defined to
// represent the same byte ordering in the host, regardless of
// endian mode. Thus there is one set for Little Endian and
// one set for Big Endian.
//
#ifdef _LE_

// Qos Command
.struct struct_qoscmd
    .u8     Command         // Command Number
    .u8     Option          // Command Option
    .u16    Index           // Index Parameter
    .u32    ReturnCode      // Return code
.ends

// Qos Queue
.struct struct_QosQueue
    .u16    EgressQueue
    .u16    IterationCredit
    .u32    TotalCredit
    .u32    MaxCredit
    .u32    Congested
    .u32    PktForward
    .u32    PktDrop
.ends

// Qos Cluster (Top Half)
.struct struct_QosClusterT
    .u32    GlobalCredit
    .u32    MaxGlobal
    .u8     QQCount
    .u8     QQ0
    .u8     QQ1
    .u8     QQ2
    .u8     QQ3
    .u8     QQ4
    .u8     QQ5
    .u8     QQ6
    .u8     QQ7
    .u8     QQ8
    .u16    RealTimeFlags
.ends

// Qos Cluster (Bottom Half)
.struct struct_QosClusterB
    .u8     EQCount
    .u8     Flags
    .u16    EQ0
    .u16    EQ1
    .u16    EQ2
    .u16    EQ3
    .u16    EQ4
    .u16    EQ5
    .u16    EQ6
    .u16    EQ7
    .u16    EQ8
    .u32    ECThresh1
    .u32    ECThresh2
    .u32    ECThresh3
    .u32    ECThresh4
.ends

// Qos RR Cluster (Top Half)
.struct struct_QosRRClusterT
    .u32    GlobalCredit
    .u32    MaxGlobal
    .u32    Remaining
    .u32    PendBits
    .u8     QQ7
    .u8     res2
    .u16    AdjSize
.ends

// Qos RR Cluster (Bottom Half)
.struct struct_QosRRClusterB
    .u8     EQCount
    .u8     Flags
    .u16    EQ0
    .u16    EQ1
    .u16    EQ2
    .u16    EQ3
    .u16    EQ4
    .u16    EQ5
    .u16    EQ6
    .u16    EQ7
    .u16    EQ8
    .u32    IterationCredit
    .u32    ECThresh
    .u32    DisableMask
    .u32    res2
.ends

// SRIO Queue Monitoring Config
.struct struct_SRIOcfg
    .u16    QueueBase
    .u8     QueueCnt
    .u8     res8
    .u8     Thresh0
    .u8     res8_0
    .u16    HwTxQ0
    .u8     Thresh1
    .u8     res8_1
    .u16    HwTxQ1
    .u8     Thresh2
    .u8     res8_2
    .u16    HwTxQ2
    .u8     Thresh3
    .u8     res8_3
    .u16    HwTxQ3
    .u8     Thresh4
    .u8     res8_4
    .u16    HwTxQ4
.ends

#else

// Qos Command
.struct struct_qoscmd
    .u16    Index           // Index Parameter
    .u8     Option          // Command Option
    .u8     Command         // Command Number
    .u32    ReturnCode      // Return code
.ends

// Qos Queue
.struct struct_QosQueue
    .u16    IterationCredit
    .u16    EgressQueue
    .u32    TotalCredit
    .u32    MaxCredit
    .u32    Congested
    .u32    PktForward
    .u32    PktDrop
.ends

// Qos Cluster (Top Half)
.struct struct_QosClusterT
    .u32    GlobalCredit
    .u32    MaxGlobal
    .u8     QQ2
    .u8     QQ1
    .u8     QQ0
    .u8     QQCount
    .u8     QQ6
    .u8     QQ5
    .u8     QQ4
    .u8     QQ3
    .u16    RealTimeFlags
    .u8     QQ8
    .u8     QQ7
.ends

// Qos Cluster (Bottom Half)
.struct struct_QosClusterB
    .u16    EQ0
    .u8     Flags
    .u8     EQCount
    .u16    EQ2
    .u16    EQ1
    .u16    EQ4
    .u16    EQ3
    .u16    EQ6
    .u16    EQ5
    .u16    EQ8
    .u16    EQ7
    .u32    ECThresh1
    .u32    ECThresh2
    .u32    ECThresh3
    .u32    ECThresh4
.ends

// Qos Cluster (Top Half)
.struct struct_QosRRClusterT
    .u32    GlobalCredit
    .u32    MaxGlobal
    .u32    Remaining
    .u32    PendBits
    .u16    AdjSize
    .u8     res2
    .u8     QQ7
.ends

// Qos Cluster (Bottom Half)
.struct struct_QosRRClusterB
    .u16    EQ0
    .u8     Flags
    .u8     EQCount
    .u16    EQ2
    .u16    EQ1
    .u16    EQ4
    .u16    EQ3
    .u16    EQ6
    .u16    EQ5
    .u16    EQ8
    .u16    EQ7
    .u32    IterationCredit
    .u32    ECThresh
    .u32    DisableMask
    .u32    res2
.ends

// SRIO Queue Monitoring Config
.struct struct_SRIOcfg
    .u8     res8
    .u8     QueueCnt
    .u16    QueueBase
    .u16    HwTxQ0
    .u8     res8_0
    .u8     Thresh0
    .u16    HwTxQ1
    .u8     res8_1
    .u8     Thresh1
    .u16    HwTxQ2
    .u8     res8_2
    .u8     Thresh2
    .u16    HwTxQ3
    .u8     res8_3
    .u8     Thresh3
    .u16    HwTxQ4
    .u8     res8_4
    .u8     Thresh4
.ends

#endif

//
// ENDIAN AGNOSTIC STRUCTURES
//
// Structures that are private to the PDSP's can be allowed to
// alter their byte ordering according to endian mode. This is
// done automatically by the assembler.
//

.struct struct_static
    // General Static
    .u16    QueueBase           // Base QOS queue
    .u8     Flags               // Firmware operating flags
#define tSRIO   t0                  // When set, SRIO mode is enabled    
    .u8     ClusterEnable       // Enable flags for 8 clusters
    // Static fields for RR
    .u8     DisableMask         // Disable mask for RR mode
    .u8     RRCQIdx             // Next congestion queue check for RR mode
    .u8     RRLQIdx             // Next Hi-Pri queue check for RR mode
    .u8     RRHQIdx             // Next Lo-Pri queue check for RR mode
    // Static fields for SRIO        
    .u8     SRIOcount           // SRIO Usage Count
    // Cached values for Queue processing
    .u8     ClusterIndex        // Current cluster processing
    .u8     LeftShift           // Credit left shift
    .u8     RightShift          // Credit right shift
.ends

.struct struct_clustproc
    .u32    TotalBytes
.ends

.struct struct_queueproc
    .u8     QueueIdx
    .u8     ClusterPos
    .u8     RRCount
    .u8     res
    .u32    QueueRecPtr
    .u32    NumPackets
    .u32    NumBytes
    .u32    PacketSize
    .u32    PacketPtr
.ends

.struct struct_Desc
    .u32    DescInfo    // 31:30 DescType: HOST(0), MONO(2)
    .u32    SrcDstTag
    .u32    PktInfo     //  15   Ret Policy: Linked(0), Unlinked(1) (HOST)
                        //  14   Ret Push Policy: Tail(0), Head(1)  (HOST)
                        // 13:12 Ret QM 
                        // 11: 0 Ret Queue
    .u32    DataLength  // HOST mode only
    .u32    DataPtr     // HOST mode only
    .u32    NextPtr     // HOST mode only
.ends    

// SRIO Scratch Variables
.struct struct_SRIOproc
    .u32    PendBits
    .u32    GarbageQNum // Which garbage queue with packet
    .u32    Qaddr
    .u32    PktSize
    .u32    DescPtr
    .u32    PktInfo     //  Return Q in bits 11:0
.ends    

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
.assign struct_static,                      R2,  R4, static      
.assign struct_qoscmd,                      R5,  R6, qoscmd        

.enter QOS_Processing
    .assign struct_QosClusterT,             R7, R11, ClusterT
    .assign struct_QosRRClusterT,           R7, R11, RRClusterT

    .enter QOS_Cluster_Processing
        .assign struct_QosClusterB,         R12, R20, ClusterB 
        .assign struct_QosRRClusterB,       R12, R20, RRClusterB 
        .assign struct_clustproc,           R21, R21, clustproc      
    .leave QOS_Cluster_Processing

    .enter QOS_Queue_Processing
        .assign struct_QosQueue,            R12, R17, Queue        
        .assign struct_queueproc,           R18, R23, queueproc

        .enter QOS_Discard_Packet
            .assign struct_Desc,            R24, R29, Desc
        .leave QOS_Discard_Packet
    .leave QOS_Queue_Processing
.leave QOS_Processing

.enter SRIO_Processing                     
    .assign struct_SRIOcfg,                  R7, R12, SRIOcfg
    .assign struct_SRIOproc,                R13, R18, SRIOproc
.leave SRIO_Processing    


//-------------------------------------------------------------------
//
// Code Starts
//

.macro ldi32
.mparam reg, val
        ldi     reg.w0, val & 0xffff
        ldi     reg.w2, val >> 16
.endm
        
        .entrypoint Header
        .origin     0
Header:
        jmp     Start
#ifdef _LE_
#define HEADER_MAGIC   0x80020000                   // "QOS"-0
#else
#define HEADER_MAGIC   0x80020001                   // "QOS"-1
#endif
#define HEADER_VERSION 0x01000009                   // 0x01.0x00.0x00.0x09
        .codeword  HEADER_MAGIC
        .codeword  HEADER_VERSION
Start:
        //--------------------------------------------------------------
        //
        // Initialization
        //

        // Timer Init
        ldi     r0, TIMER_SETTING                   
        sbco    r0, cTimer, TIMER_OFF_LOAD, 4
        ldi     r0, 1<<15 | 0b0000<<2 | 0b11        // Enable | /2 | Mode+Go
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4

        // Init the static variables        
        zero    &static, SIZE(static)
        mov     static.QueueBase, QUEUE_BASE

        // Zero out our memory
        mov     r0, RAM_SIZE
        mov     r1, 0
ZeroLoop:        
        sub     r0, r0, 4
        sbco    r1, cLocalRAM, r0, 4
        qbne    ZeroLoop, r0, 0

        // Save the version key to scratch
        // This allows host cores to check version of preloaded firmware
        // matches the LLD, since the host cores cant read program when running
        ldi32   r1, HEADER_MAGIC
        ldi32   r2, HEADER_VERSION
        ldi     r0, QOSOFF_VERSION
        sbco    r1, cLocalRAM, r0, 8
      

#ifdef _MODEL_
        //-------------------------------
        // Test Code for Profiling
        jmp     FakeMainLoop
        // Test Code for Profiling
        //-------------------------------
#endif        

        //--------------------------------------------------------------
        //
        // Main Processing Loop
        //

MainLoop:
#ifdef _MODEL_
        //-------------------------------
        // Test Code for Profiling
        mov     r0.w0, PRCMD_BASE & 0xFFFF
        mov     r0.w2, PRCMD_BASE >> 16
        mov     r1, PROFILEID+1
        sbbo    r1, r0, 0, 4
FakeMainLoop:
        // Test Code for Profiling
        //-------------------------------
#endif

        //
        // Wait for the next clock tick
        //
        wbs     r31.tStatus_Timer                   // Look for a timer tick
        set     r31.tStatus_Timer                   // Clear the timer trigger                   

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

        //
        // Get a user command
        //
        lbco    qoscmd, cLocalRAM, QOSOFF_COMMAND, SIZE(qoscmd)
        qbbc    NoCommand, qoscmd.Command.t7
        mov     qoscmd.ReturnCode, QCMD_RETCODE_SUCCESS
        qbne    NotCmdGetBase, qoscmd.Command, QCMD_GET_QUEUE_BASE
        
        //-------------------------
        //
        // Command: Get Queue Base
        //
        mov     qoscmd.Index, static.QueueBase
        jmp     CmdDone
        
NotCmdGetBase:
        qbne    NotCmdSetBase, qoscmd.Command, QCMD_SET_QUEUE_BASE
        
        //-------------------------
        //
        // Command: Set Queue Base
        //
        and     r0, qoscmd.Index, 0x1F                  // Must be a multiple of 32
        qbeq    BaseIsLegal, r0, 0
ERROR_InvalidParam:
        mov     qoscmd.ReturnCode, QCMD_RETCODE_INVALID_INDEX
        jmp     CmdDone
BaseIsLegal:        
        mov     static.QueueBase, qoscmd.Index
        jmp     CmdDone
        
NotCmdSetBase:
        qbne    NotCmdTimer, qoscmd.Command, QCMD_TIMER_CONFIG

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
        jmp     CmdDone

NotCmdTimer:
        qbne    NotCmdCluster, qoscmd.Command, QCMD_CLUSTER_ENABLE

        //-------------------------------------------------------------
        //
        // Command: Enable/Disable Cluster
        //
        qble    ERROR_InvalidParam, qoscmd.Index, 8    
        qbeq    ClusterEnable, qoscmd.Option, QOPT_CLUSTER_ENABLE
        qbeq    ClusterDisable, qoscmd.Option, QOPT_CLUSTER_DISABLE
ERROR_InvalidOption:
        mov     qoscmd.ReturnCode, QCMD_RETCODE_INVALID_OPTION
        jmp     CmdDone
ClusterEnable:
        set     static.ClusterEnable, qoscmd.Index
        jmp     CmdDone
ClusterDisable:
        // If cluster wasn't enabled, then skip the discard
        qbbc    CmdDone, static.ClusterEnable, qoscmd.Index
        clr     static.ClusterEnable, qoscmd.Index
.using QOS_Processing
        // Load the cluster record and discard all the packets
        lsl     r0, qoscmd.Index, 3                 // R0 = qoscmd.Index * 8
        sub     r0, r0, qoscmd.Index                // R0 = qoscmd.Index * 7
        lsl     r0, r0, 3                           // R0 = qoscmd.Index * 56
        add     r0, r0, QOSOFF_QOSCLUSTER
        lbco    ClusterT, cLocalRAM, r0, SIZE(ClusterT)
        call    DiscardCluster
.leave QOS_Processing
        jmp     CmdDone

NotCmdCluster:
        qbne    NotCmdSRIO, qoscmd.Command, QCMD_SRIO_ENABLE

        //-------------------------------------------------------------
        //
        // Command: Enable/Disable SRIO Monitoring
        //
        qbeq    SRIOEnable, qoscmd.Option, QOPT_SRIO_ENABLE
        qbne    ERROR_InvalidOption, qoscmd.Option, QOPT_SRIO_DISABLE
        // SRIO Disable
        clr     static.Flags.tSRIO
        jmp     CmdDone
SRIOEnable:
        // SRIO Enable
        mov     static.SRIOcount, 0
        set     static.Flags.tSRIO
        jmp     CmdDone

NotCmdSRIO:
        //-------------------------------------------------------------
        //
        // Invalid Command
        //
        mov     qoscmd.ReturnCode, QCMD_RETCODE_INVALID_COMMAND
        // Wrap up command
CmdDone:
        mov     qoscmd.Command, 0
        sbco    qoscmd, cLocalRAM, QOSOFF_COMMAND, SIZE(qoscmd)

NoCommand:


//====================================================================================        
//
// Begin QOS Processing
//
    .using QOS_Processing

//====================================================================================        
//
// Begin QOS Cluster Processing
//
    .using QOS_Cluster_Processing

        mov     static.ClusterIndex, 0
ClusterLoop:
        //
        // Get the next QOS Cluster
        //
        qbbc    ClusterDone, static.ClusterEnable, static.ClusterIndex
        lsl     r0, static.ClusterIndex, 3          // R0 = ClusterIndex * 8
        sub     r0, r0, static.ClusterIndex         // R0 = ClusterIndex * 7
        lsl     r0, r0, 3                           // R0 = ClusterIndex * 56
        add     r0, r0, QOSOFF_QOSCLUSTER
        lbco    ClusterT, cLocalRAM, r0, SIZE(ClusterT)+SIZE(ClusterB)
        // Normal mode only if not index 7
        qbne    NormalModeOnly, static.ClusterIndex, 7
        // Jump out now if RR mode
        qbbs    RRCluster, ClusterB.Flags, 0
        
NormalModeOnly:        
        //
        // Update the cluster state
        //
        mov     clustproc.TotalBytes, 0
        qbeq    EQbacklog_done, ClusterB.EQCount, 0
        lsl     r0, ClusterB.EQ0, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 1
        lsl     r0, ClusterB.EQ1, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 2
        lsl     r0, ClusterB.EQ2, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 3
        lsl     r0, ClusterB.EQ3, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 4
        lsl     r0, ClusterB.EQ4, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 5
        lsl     r0, ClusterB.EQ5, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 6
        lsl     r0, ClusterB.EQ6, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 7
        lsl     r0, ClusterB.EQ7, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
        qbeq    EQbacklog_done, ClusterB.EQCount, 8
        lsl     r0, ClusterB.EQ8, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        add     clustproc.TotalBytes, clustproc.TotalBytes, r1
 
EQbacklog_done:
        mov     static.LeftShift, 1
        mov     static.RightShift, 0
        qbgt    EQscale_done, clustproc.TotalBytes, ClusterB.ECThresh1
        mov     static.LeftShift, 0
        qbgt    EQscale_done, clustproc.TotalBytes, ClusterB.ECThresh2
        mov     static.RightShift, 1
        qbgt    EQscale_done, clustproc.TotalBytes, ClusterB.ECThresh3
        mov     static.RightShift, 2
        qbgt    EQscale_done, clustproc.TotalBytes, ClusterB.ECThresh4
        mov     static.RightShift, 31
        jmp     EQscale_done

ClusterDone:        
        add     static.ClusterIndex, static.ClusterIndex, 1
        qbgt    ClusterLoop, static.ClusterIndex, 8
        jmp     RRClusterDone
        
RRCluster:
        // Assign the iteration credit and cap
        add     RRClusterT.GlobalCredit, RRClusterT.GlobalCredit, RRClusterB.IterationCredit
        min     RRClusterT.GlobalCredit, RRClusterT.GlobalCredit, RRClusterT.MaxGlobal
        // Copy over the disable flags
        mov     static.DisableMask, RRClusterB.DisableMask.b0
        // Get the backlog of the egress queue
        lsl     r0, ClusterB.EQ0, 4                 // Get queue offset
        lbco    r0, cQPeekBase, r0, 8               // Read pending packets & bytes
        // Calculate the backlog remaining
        min     r1, r1, RRClusterB.ECThresh
        sub     RRClusterT.Remaining, RRClusterB.ECThresh, r1
        jmp     RRProcessQueues

RRClusterDone:
        qbbc    MainLoop, static.Flags.tSRIO
        jmp     SrioCheck

    .leave QOS_Cluster_Processing
//
// End QOS Cluser Processing
//
//====================================================================================        

//====================================================================================        
//
// Begin QOS Queue Processing
//
    .using QOS_Queue_Processing

EQscale_done:
        //
        // Process the queues
        //
        qbeq    QQ_done, ClusterT.QQCount, 0
        mov     queueproc.ClusterPos, 0
        mov     queueproc.QueueIdx, ClusterT.QQ0
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 1
        mov     queueproc.ClusterPos, 1
        mov     queueproc.QueueIdx, ClusterT.QQ1
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 2
        mov     queueproc.ClusterPos, 2
        mov     queueproc.QueueIdx, ClusterT.QQ2
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 3
        mov     queueproc.ClusterPos, 3
        mov     queueproc.QueueIdx, ClusterT.QQ3
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 4
        mov     queueproc.ClusterPos, 4
        mov     queueproc.QueueIdx, ClusterT.QQ4
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 5
        mov     queueproc.ClusterPos, 5
        mov     queueproc.QueueIdx, ClusterT.QQ5
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 6
        mov     queueproc.ClusterPos, 6
        mov     queueproc.QueueIdx, ClusterT.QQ6
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 7
        mov     queueproc.ClusterPos, 7
        mov     queueproc.QueueIdx, ClusterT.QQ7
        call    ProcessQueue
        qbeq    QQ_done, ClusterT.QQCount, 8
        mov     queueproc.ClusterPos, 8
        mov     queueproc.QueueIdx, ClusterT.QQ8
        call    ProcessQueue
        
 QQ_done:
        // Save the cluster global credit
        lsl     r0, static.ClusterIndex, 3          // R0 = ClusterIndex * 8
        sub     r0, r0, static.ClusterIndex         // R0 = ClusterIndex * 7
        lsl     r0, r0, 3                           // R0 = ClusterIndex * 56
        add     r0, r0, QOSOFF_QOSCLUSTER
        sbco    ClusterT, cLocalRAM, r0, 4
        jmp     ClusterDone


//------------------------------------------------------------------------
// RRProcessQueues
//
// Process a Round Robin mode QOS cluster.
//
// Assumed Valid:
//     static.DisableMask
//     "RRClusterT"
//
RRProcessQueues:
        //
        // We know we have external credit to move a packet. We will look
        // for a packet on each of the high priority queues and if none found, 
        // look on each of the low priority queues.
        //
        // Once we pick a queue, we will check to see if there is enough
        // global credit to move the packet. If not, we setup the index 
        // such that the same queue will be selected the next time through
        // this function.
        //
        add     r0, static.QueueBase, 56
        lsr     r0, r0, 5                               // r0 = Queue Number / 32
        lsl     r0, r0, 2                               // Get word address
        lbco    RRClusterT.PendBits, cQPending, r0, 4   // Load queue pending bits

RRMainLoop:
        // Jump past the RR if we can't send anyway
        qbeq    RRPacketMoveDone, RRClusterT.Remaining, 0

        // Skip high priority looping if empty
        and     r0, RRClusterT.PendBits.b3, 0xF             // Bits 27:24 
        qbeq    RRNoHighPri, r0, 0

        // Find a high priority queue that has packets
        mov     queueproc.RRCount, 0                        // Count the loops
RRHQLoop:
        add     r0, queueproc.RRCount, static.RRHQIdx       // Offset frop current start
        and     r0, r0, 0x3                                 // Make sure we are (0-3)
        qbbs    RRHQDisabled, static.DisableMask, r0        // Skip this Q if disabled
        add     r0, r0, 56%32                               // Add offset to status bit
        qbbs    RRHQueueHasPacket, RRClusterT.PendBits, r0  // Check status bit, jump if set
RRHQDisabled:
        add     queueproc.RRCount, queueproc.RRCount, 1     // Count loop
        qbne    RRHQLoop, queueproc.RRCount, 4              // Check 4 queues total
        
RRNoHighPri:
        // Find a low priority queue that has packets
        mov     queueproc.RRCount, 0                        // Count the loops
RRLQLoop:
        add     r0, queueproc.RRCount, static.RRLQIdx       // Offset frop current start
        and     r0, r0, 0x3                                 // Make sure we are (0-3)
        add     r0, r0, 4                                   // Offset by 4 for low pri
        qbbs    RRLQDisabled, static.DisableMask, r0        // Skip this Q if disabled
        add     r0, r0, 56%32                               // Add offset to status bit
        qbbs    RRLQueueHasPacket, RRClusterT.PendBits, r0  // Check status bit, jump if set
RRLQDisabled:                                               
        add     queueproc.RRCount, queueproc.RRCount, 1     // Count loop
        qbne    RRLQLoop, queueproc.RRCount, 4              // Check 4 queues total
        jmp     RRPacketMoveDone                            // No packets on any queue - quit

RRLQueueHasPacket:
        add     queueproc.QueueIdx, r0, 56 - (56%32)        // Remove offset to status and add offset to queue
        call    CheckRRQueue                                // Move a packet
        add     r1, queueproc.RRCount, static.RRLQIdx       // The check function may have bumped the RRCount
        and     static.RRLQIdx, r1, 0x3                     // Make sure the idx is 0-3
        qbne    RRMainLoop, RRClusterT.Remaining, 0         // Keep going if we can move more packets
        jmp     RRPacketMoveDone                        

RRHQueueHasPacket:
        add     queueproc.QueueIdx, r0, 56 - (56%32)        // Remove offset to status and add offset to queue
        call    CheckRRQueue                                // Move a packet
        add     r1, queueproc.RRCount, static.RRHQIdx       // The check function may have bumped the RRCount
        and     static.RRHQIdx, r1, 0x3                     // Make sure the idx is 0-3
        qbne    RRMainLoop, RRClusterT.Remaining, 0         // Keep going if we can move more packets

RRPacketMoveDone:
        // Check for congestion on one queue
        add     static.RRCQIdx, static.RRCQIdx, 1           // Bump the congestion idx to check
        and     static.RRCQIdx, static.RRCQIdx, 0x7         // Make sure it is 0-7
        add     r0, static.RRCQIdx, 56%32                   // Add the offset to the status bits
        qbbc    RRAllDone, RRClusterT.PendBits, r0          // Not congested if not packets on queue
        add     queueproc.QueueIdx, static.RRCQIdx, 56      // Get the actual queue index to check
        jmp     RRCongestionCheck                           // Check the queue

RRAllDone:
        // Save the cluster RR state
        mov     r0, (7 * 56) + QOSOFF_QOSCLUSTER
        sbco    RRClusterT, cLocalRAM, r0, 4
        jmp     RRClusterDone


//------------------------------------------------------------------------
// CheckRRQueue
//
// Process the QOS queue specified in "queueproc.QueueIdx".
//
// Assumed Valid:
//     queueproc.ClusterPos
//     queueproc.QueueIdx
//     "ClusterT"
//
CheckRRQueue:
        lsl     r0, queueproc.QueueIdx, 1           // r0 = QQ * 2
        add     r0, r0, queueproc.QueueIdx          // r0 = QQ * 3
        lsl     r0, r0, 3                           // r0 = QQ * 24
        mov     r1, QOSOFF_QOSQUEUES
        add     queueproc.QueueRecPtr, r0, r1
        lbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)
        // Look at the next packet
        add     r0, queueproc.QueueIdx, static.QueueBase    // Get real QOS queue index
        lsl     r0, r0, 4                                   // Get queue offset
        lbco    queueproc.NumPackets, cQBase, r0, 16        // Read all 4 regs
        // Quit if no packet
        qbeq    RRQueueNoPacket, queueproc.NumPackets, 0      
        // Adjust the size
        add     r1, queueproc.PacketSize, RRClusterT.AdjSize
        // See if we can forward the packet
        qbgt    RRQueueCantForward, ClusterT.GlobalCredit, r1
        // Adjust the real time credit
        sub     ClusterT.GlobalCredit, ClusterT.GlobalCredit, r1
        // Adjust the backlog credit
        min     r0, RRClusterT.Remaining, r1
        sub     RRClusterT.Remaining, RRClusterT.Remaining, r0
        lsl     r0, Queue.EgressQueue, 4            // Get egress queue offset
        add     r0, r0, 8                           // Just write C and D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Push packet
        add     Queue.PktForward, Queue.PktForward, 1
        // Save Queue 
        sbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)
RRQueueDone:
        add     queueproc.RRCount, queueproc.RRCount, 1
        ret
RRQueueCantForward:
        // Zap the backlog credit to keep the RR engine here
        mov     RRClusterT.Remaining, 0
        // Put the packet back on the queue (r0 still points to queue reg A)
        set     queueproc.PacketSize.t31            // Push to head
        add     r0, r0, 0x08                        // Write to C+D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Return old desc 
        ret
RRQueueNoPacket:
        sub     r0, queueproc.QueueIdx, 32
        clr     RRClusterT.PendBits, r0
        add     queueproc.RRCount, queueproc.RRCount, 1
        ret


//------------------------------------------------------------------------
// RRCongestionCheck:
//
// Process the QOS queue specified in "queueproc.QueueIdx".
//
// Assumed Valid:
//     queueproc.QueueIdx
//     "ClusterT"
//
RRCongestionCheck:
        lsl     r0, queueproc.QueueIdx, 1           // r0 = QQ * 2
        add     r0, r0, queueproc.QueueIdx          // r0 = QQ * 3
        lsl     r0, r0, 3                           // r0 = QQ * 24
        mov     r1, QOSOFF_QOSQUEUES
        add     queueproc.QueueRecPtr, r0, r1
        lbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)
        // If congestion is disabled, quit
        qbbs    RRAllDone, Queue.Congested.t31
RRDiscardLoop:        
        // Look at the next packet and see if we have credit
        add     r0, queueproc.QueueIdx, static.QueueBase  // Get real QOS queue index
        lsl     r0, r0, 4                               // Get queue offset
        lbco    queueproc.NumPackets, cQBase, r0, 16    // Read all 4 regs
        // Quit if no packets
        qbeq    RRCQDone, queueproc.NumPackets, 0      
        // Quit if the queue is not congested
        qble    RRReturnPacket, Queue.Congested, queueproc.NumBytes
        // Discard this packet and loop
        call    DiscardPacket
        jmp     RRDiscardLoop
RRReturnPacket:
        set     queueproc.PacketSize.t31            // Push to head
        add     r0, r0, 0x08                        // Write to C+D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Return old desc 
RRCQDone:
        // Save Queue
        sbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)
        jmp     RRAllDone


//------------------------------------------------------------------------
// ProcessQueue
//
// Process the QOS queue specified in "queueproc.QueueIdx".
//
// Assumed Valid:
//     queueproc.QueueIdx
//     queueproc.ClusterPos
//     static.LeftShift and static.RightShift
//     "ClusterT"
//
ProcessQueue:
        pushRet
        lsl     r0, queueproc.QueueIdx, 1           // r0 = QQ * 2
        add     r0, r0, queueproc.QueueIdx          // r0 = QQ * 3
        lsl     r0, r0, 3                           // r0 = QQ * 24
        mov     r1, QOSOFF_QOSQUEUES
        add     queueproc.QueueRecPtr, r0, r1
        lbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)

        // Assign credit for this round
        qbbs    RealTime, ClusterT.RealTimeFlags, queueproc.ClusterPos
        lsl     r0, Queue.IterationCredit, static.LeftShift
        lsr     r0, r0, static.RightShift
        add     Queue.TotalCredit, Queue.TotalCredit, r0
        jmp     ProcessCredit
RealTime:
        add     Queue.TotalCredit, Queue.TotalCredit, Queue.IterationCredit
ProcessCredit:
        add     Queue.TotalCredit, Queue.TotalCredit, ClusterT.GlobalCredit
        mov     ClusterT.GlobalCredit, 0
        
ForwardingLoop:        
        // Look at the next packet and see if we have credit
        add     r0, queueproc.QueueIdx, static.QueueBase  // Get real QOS queue index
        lsl     r0, r0, 4                               // Get queue offset
        lbco    queueproc.NumPackets, cQBase, r0, 16    // Read all 4 regs
        // Quit if no packets
        qbeq    QueueDone, queueproc.NumPackets, 0      
        // Forward the packet if we have enough credit
        qble    ForwardPacket, Queue.TotalCredit, queueproc.PacketSize
        // Quit if the queue is not congested
        qble    ReturnPacket, Queue.Congested, queueproc.NumBytes
        // Discard this packet and loop
        call    DiscardPacket
        jmp     ForwardingLoop
                
ForwardPacket:                
        // We will forward this packet, so take credit away
        sub     Queue.TotalCredit, Queue.TotalCredit, queueproc.PacketSize
        lsl     r0, Queue.EgressQueue, 4            // Get egress queue offset
        add     r0, r0, 8                           // Just write C and D
        sbco    queueproc.PacketSize, cQBase, r0, 8      // Push packet
        add     Queue.PktForward, Queue.PktForward, 1
        jmp     ForwardingLoop

ReturnPacket:
        set     queueproc.PacketSize.t31            // Push to head
        add     r0, r0, 0x08                        // Write to C+D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Return old desc 

QueueDone:
        // Recover any global credit
        qble    NoExtra, Queue.MaxCredit, Queue.TotalCredit
        sub     ClusterT.GlobalCredit, Queue.TotalCredit, Queue.MaxCredit
        mov     Queue.TotalCredit, Queue.MaxCredit
        // Make sure our global credit is ok
        min     ClusterT.GlobalCredit, ClusterT.GlobalCredit, ClusterT.MaxGlobal

NoExtra:        
        // Save Queue and exit
        sbco    Queue, cLocalRAM, queueproc.QueueRecPtr, SIZE(Queue)
        
        popRet
        ret


//====================================================================================        
//
// Begin QOS Discard Packet
//
    .using QOS_Discard_Packet

DiscardPacket:
        // Discard this packet
        and     queueproc.PacketPtr.b0, queueproc.PacketPtr.b0, 0xF0
        // Read the descriptor
        lbbo    Desc, queueproc.PacketPtr, 0, SIZE(Desc)
        
        // Make sure of its type
        and     r0, Desc.DescInfo.b3, 0xC0
        qbeq    HostDescriptor, r0, 0
        qbeq    MonoDescriptor, r0, 0x80

        // If we get here, it ain't good, but the best we can
        // do is ignore the problem and move on
        jmp     DiscardDone
       
HostDescriptor:
        // Jump if we're splitting descriptors
        qbbs    HostDescriptorSplit, Desc.PktInfo.t15
        mov     Desc.NextPtr, 0                     // Force an end to the return loop
HostDescriptorSplit:
        mov     queueproc.PacketSize, 0             // Assume normal push to tail
        qbbc    HostDescriptorNormalPush,  Desc.PktInfo.t14
        set     queueproc.PacketSize.t31            // Push to head
HostDescriptorNormalPush:
        // Free the descriptor
        and     Desc.PktInfo.b1, Desc.PktInfo.b1, 0x07 // We save PktInfo & 0x7FF
        lsl     r0, Desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 0x08                        // Write to C+D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Return old desc 
        // Jump if we're done returning host desc
        qbeq    DiscardDone, Desc.NextPtr, 0
        // Load the descriptor info and return it
        lbbo    Desc, Desc.NextPtr, 0, SIZE(Desc)
        jmp     HostDescriptorSplit

MonoDescriptor:
        // Free the descriptor
        and     Desc.PktInfo.b1, Desc.PktInfo.b1, 0x07 // We save PktInfo & 0x7FF
        lsl     r0, Desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 12                          // Write to D
        sbco    queueproc.PacketPtr, cQBase, r0, 4  // Return old desc 

DiscardDone:        
        add     Queue.PktDrop, Queue.PktDrop, 1
        ret

    .leave QOS_Discard_Packet
//
// End QOS Discard Packet
//
//====================================================================================        

            
    .leave QOS_Queue_Processing
//
// End QOS Queue Processing
//
//====================================================================================                   


//====================================================================================        
//
// Begin QOS Queue Processing
//
    .using QOS_Queue_Processing

//------------------------------------------------------------------------
// DiscardCluster
//
// Discard all packets held by queues in the cluster.
//
// Assumed Valid:
//     "ClusterT"
//
DiscardCluster:
        pushRet
        qbeq    discard_done, ClusterT.QQCount, 0
        mov     queueproc.QueueIdx, ClusterT.QQ0
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 1
        mov     queueproc.QueueIdx, ClusterT.QQ1
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 2
        mov     queueproc.QueueIdx, ClusterT.QQ2
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 3
        mov     queueproc.QueueIdx, ClusterT.QQ3
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 4
        mov     queueproc.QueueIdx, ClusterT.QQ4
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 5
        mov     queueproc.QueueIdx, ClusterT.QQ5
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 6
        mov     queueproc.QueueIdx, ClusterT.QQ6
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 7
        mov     queueproc.QueueIdx, ClusterT.QQ7
        call    DiscardQueue
        qbeq    discard_done, ClusterT.QQCount, 8
        mov     queueproc.QueueIdx, ClusterT.QQ8
        call    DiscardQueue
 discard_done:
        popRet
        ret

//====================================================================================        
//
// Begin QOS Discard Packet
//
    .using QOS_Discard_Packet

//------------------------------------------------------------------------
// DiscardQueue
//
// Process the QOS queue specified in "queueproc.QueueIdx".
//
// Assumed Valid:
//     queueproc.QueueIdx
//
DiscardQueue:
        lsl     r0, queueproc.QueueIdx, 1           // r0 = QQ * 2
        add     r0, r0, queueproc.QueueIdx          // r0 = QQ * 3
        lsl     r0, r0, 3                           // r0 = QQ * 24
        mov     r1, QOSOFF_QOSQUEUES
        add     r0, r0, r1
        lbco    Queue, cLocalRAM, r0, SIZE(Queue)
       
DiscardLoop2:       
        // Get the next packet
        add     r0, queueproc.QueueIdx, static.QueueBase  // Get real QOS queue index
        lsl     r0, r0, 4                           // Get queue offset

        // Discard this packet
        add     r0, r0, 12                          // Just read D
        lbco    queueproc.PacketPtr, cQBase, r0, 4  // Pop packet
        and     queueproc.PacketPtr.b0, queueproc.PacketPtr.b0, 0xF0
        qbeq    QueueDone2, queueproc.PacketPtr, 0

        // Read the descriptor
        lbbo    Desc, queueproc.PacketPtr, 0, SIZE(Desc)
        
        // Make sure of its type
        and     r0, Desc.DescInfo.b3, 0xC0
        qbeq    HostDescriptor2, r0, 0
        qbeq    MonoDescriptor2, r0, 0x80

        // If we get here, it ain't good, but the best we can
        // do is ignore the problem and move on
        jmp     DiscardDone2
       
HostDescriptor2:
        // Jump if we're splitting descriptors
        qbbs    HostDescriptorSplit2, Desc.PktInfo.t15
        mov     Desc.NextPtr, 0                     // Force an end to the return loop
HostDescriptorSplit2:
        mov     queueproc.PacketSize, 0            // Assume normal push to tail
        qbbc    HostDescriptorNormalPush2,  Desc.PktInfo.t14
        set     queueproc.PacketSize.t31           // Push to head
HostDescriptorNormalPush2:
        // Free the descriptor
        and     Desc.PktInfo.b1, Desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, Desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 0x08                        // Write to C+D
        sbco    queueproc.PacketSize, cQBase, r0, 8 // Return old desc 
        // Jump if we're done returning host desc
        qbeq    DiscardDone2, Desc.NextPtr, 0
        // Load the descriptor info and return it
        lbbo    Desc, Desc.NextPtr, 0, SIZE(Desc)
        jmp     HostDescriptorSplit2

MonoDescriptor2:
        // Free the descriptor
        and     Desc.PktInfo.b1, Desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, Desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 12                          // Write to D
        sbco    queueproc.PacketPtr, cQBase, r0, 4  // Return old desc 

DiscardDone2:        
        add     Queue.PktDrop, Queue.PktDrop, 1
        jmp     DiscardLoop2

QueueDone2:
        // Save Queue and exit
        lsl     r0, queueproc.QueueIdx, 1           // r0 = QQ * 2
        add     r0, r0, queueproc.QueueIdx          // r0 = QQ * 3
        lsl     r0, r0, 3                           // r0 = QQ * 24
        mov     r1, QOSOFF_QOSQUEUES
        add     r0, r0, r1
        sbco    Queue, cLocalRAM, r0, SIZE(Queue)
        
        ret

    .leave QOS_Discard_Packet
//
// End QOS Discard Packet
//
//====================================================================================        

            
    .leave QOS_Queue_Processing
//
// End QOS Queue Processing
//
//====================================================================================                   


    .leave QOS_Processing
//
// End QOS Processing
//
//====================================================================================        

//====================================================================================        
//
// Begin SRIO Queue Processing
//
.using SRIO_Processing                     

SrioCheck:
        // Load our state
        mov     r0, QOSOFF_SRIO
        lbco    SRIOcfg, cLocalRAM, r0, SIZE(SRIOcfg)                            
        lsl     SRIOproc.Qaddr, SRIOcfg.QueueBase, 4    // QOS queue address

        // Get the base queue pending bits
        lsr     r0, SRIOcfg.QueueBase, 5                // r0 = Queue Number / 32       
        lsl     r0, r0, 2                               // Get word address
        lbco    SRIOproc.PendBits, cQPending, r0, 4     // Load queue pending bits

        // Do STXCQ0 
        qbeq    STXCQ_Done, SRIOcfg.QueueCnt, 0         // Jump out if not this many queues
        qbbc    NoSTXCQ0, SRIOproc.PendBits.t11         // Jump around if no pkt waiting
        add     r0, SRIOproc.Qaddr, 11*16 + 8  
        add     r1, r0, 5*16  
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopSTXCQ0:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        sub     static.SRIOcount, static.SRIOcount, 1
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopSTXCQ0, SRIOproc.DescPtr, 0
NoSTXCQ0:

        // Do STXCQ1 
        qbeq    STXCQ_Done, SRIOcfg.QueueCnt, 1         // Jump out if not this many queues
        qbbc    NoSTXCQ1, SRIOproc.PendBits.t12         // Jump around if no pkt waiting
        add     r0, SRIOproc.Qaddr, 12*16 + 8  
        add     r1, r0, 5*16  
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopSTXCQ1:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        sub     static.SRIOcount, static.SRIOcount, 1
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopSTXCQ1, SRIOproc.DescPtr, 0
NoSTXCQ1:

        // Do STXCQ2 
        qbeq    STXCQ_Done, SRIOcfg.QueueCnt, 2         // Jump out if not this many queues
        qbbc    NoSTXCQ2, SRIOproc.PendBits.t13         // Jump around if no pkt waiting
        add     r0, SRIOproc.Qaddr, 13*16 + 8  
        add     r1, r0, 5*16  
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopSTXCQ2:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        sub     static.SRIOcount, static.SRIOcount, 1
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopSTXCQ2, SRIOproc.DescPtr, 0
NoSTXCQ2:

        // Do STXCQ3 
        qbeq    STXCQ_Done, SRIOcfg.QueueCnt, 3         // Jump out if not this many queues
        qbbc    NoSTXCQ3, SRIOproc.PendBits.t14         // Jump around if no pkt waiting
        add     r0, SRIOproc.Qaddr, 14*16 + 8  
        add     r1, r0, 5*16  
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopSTXCQ3:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        sub     static.SRIOcount, static.SRIOcount, 1
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopSTXCQ3, SRIOproc.DescPtr, 0
NoSTXCQ3:

        // Do STXCQ4 
        qbeq    STXCQ_Done, SRIOcfg.QueueCnt, 4         // Jump out if not this many queues
        qbbc    NoSTXCQ4, SRIOproc.PendBits.t15         // Jump around if no pkt waiting
        add     r0, SRIOproc.Qaddr, 15*16 + 8  
        add     r1, r0, 5*16  
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopSTXCQ4:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        sub     static.SRIOcount, static.SRIOcount, 1
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopSTXCQ4, SRIOproc.DescPtr, 0
NoSTXCQ4:


STXCQ_Done:
        // Do STXQ0 
        qbeq    STXQ_Done, SRIOcfg.QueueCnt, 0          // Jump out if not this many queues
        qbbc    NoTxQ0, SRIOproc.PendBits.t6            // Jump around if no pkt waiting
        qble    NoTxQ0, static.SRIOcount, SRIOcfg.Thresh0   // Jump around if theshold hit
        add     r0, SRIOproc.Qaddr, 6*16 + 8  
        lsl     r1, SRIOcfg.HwTxQ0, 4                   // QOS queue address
        add     r1, r1, 8
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopTxQ0:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        add     static.SRIOcount, static.SRIOcount, 1
        qble    NoTxQ0, static.SRIOcount, SRIOcfg.Thresh0   // Jump around if theshold hit
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopTxQ0, SRIOproc.DescPtr, 0
NoTxQ0:

        // Do STXQ1 
        qbeq    STXQ_Done, SRIOcfg.QueueCnt, 1          // Jump out if not this many queues
        qbbc    NoTxQ1, SRIOproc.PendBits.t7            // Jump around if no pkt waiting
        qble    NoTxQ1, static.SRIOcount, SRIOcfg.Thresh1   // Jump around if theshold hit
        add     r0, SRIOproc.Qaddr, 7*16 + 8  
        lsl     r1, SRIOcfg.HwTxQ1, 4                   // QOS queue address
        add     r1, r1, 8
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopTxQ1:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        add     static.SRIOcount, static.SRIOcount, 1
        qble    NoTxQ1, static.SRIOcount, SRIOcfg.Thresh1   // Jump around if theshold hit
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopTxQ1, SRIOproc.DescPtr, 0
NoTxQ1:

        // Do STXQ2 
        qbeq    STXQ_Done, SRIOcfg.QueueCnt, 2          // Jump out if not this many queues
        qbbc    NoTxQ2, SRIOproc.PendBits.t8            // Jump around if no pkt waiting
        qble    NoTxQ2, static.SRIOcount, SRIOcfg.Thresh2   // Jump around if theshold hit
        add     r0, SRIOproc.Qaddr, 8*16 + 8  
        lsl     r1, SRIOcfg.HwTxQ2, 4                   // QOS queue address
        add     r1, r1, 8
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopTxQ2:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        add     static.SRIOcount, static.SRIOcount, 1
        qble    NoTxQ2, static.SRIOcount, SRIOcfg.Thresh2   // Jump around if theshold hit
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopTxQ2, SRIOproc.DescPtr, 0
NoTxQ2:

        // Do STXQ3 
        qbeq    STXQ_Done, SRIOcfg.QueueCnt, 3          // Jump out if not this many queues
        qbbc    NoTxQ3, SRIOproc.PendBits.t9            // Jump around if no pkt waiting
        qble    NoTxQ3, static.SRIOcount, SRIOcfg.Thresh3   // Jump around if theshold hit
        add     r0, SRIOproc.Qaddr, 9*16 + 8  
        lsl     r1, SRIOcfg.HwTxQ3, 4                   // QOS queue address
        add     r1, r1, 8
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopTxQ3:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        add     static.SRIOcount, static.SRIOcount, 1
        qble    NoTxQ3, static.SRIOcount, SRIOcfg.Thresh3   // Jump around if theshold hit
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopTxQ3, SRIOproc.DescPtr, 0
NoTxQ3:

        // Do STXQ4 
        qbeq    STXQ_Done, SRIOcfg.QueueCnt, 4          // Jump out if not this many queues
        qbbc    NoTxQ4, SRIOproc.PendBits.t10           // Jump around if no pkt waiting
        qble    NoTxQ4, static.SRIOcount, SRIOcfg.Thresh4   // Jump around if theshold hit
        add     r0, SRIOproc.Qaddr, 10*16 + 8  
        lsl     r1, SRIOcfg.HwTxQ4, 4                   // QOS queue address
        add     r1, r1, 8
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
LoopTxQ4:
        sbco    SRIOproc.PktSize, cQBase, r1, 8         // Write 2 regs
        add     static.SRIOcount, static.SRIOcount, 1
        qble    NoTxQ4, static.SRIOcount, SRIOcfg.Thresh4   // Jump around if theshold hit
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        qbne    LoopTxQ4, SRIOproc.DescPtr, 0
NoTxQ4:

STXQ_Done:
        // Check the error packet queues
        and     SRIOproc.PendBits, SRIOproc.PendBits, 0x3F
SrioErrorPktLoop:
        lmbd    SRIOproc.GarbageQNum, SRIOproc.PendBits, 1  // Get the next error queue with a packet
        qbeq    SrioErrorDone, SRIOproc.GarbageQNum, 32     // Jump out if there are none         
        clr     SRIOproc.PendBits, SRIOproc.PendBits, SRIOproc.GarbageQNum    // We'll only service one per loop
        lsl     r0, SRIOproc.GarbageQNum, 4
        add     r0, r0, 8
        add     r0, r0, SRIOproc.Qaddr                  // Get the address of this error queue
        lbco    SRIOproc.PktSize, cQBase, r0, 8         // Read 2 regs
        lbbo    SRIOproc.PktInfo, SRIOproc.DescPtr, 8, 4   // Get the original return queue info
        // Use the CPPI return queue info in order to decide if this packet
        // was intended for SRIO, before accounting for it
        mov     r1, 0x3FFF
        and     r1, SRIOproc.PktInfo, r1
        sub     r1, r1, SRIOcfg.QueueBase               // r1 offset from our SRIO queue base
        sub     r1, r1, 11                              // r1 offset from out Shadow TXCQ
        qble    SrioNotInRange, r1, SRIOcfg.QueueCnt    // Jump around if not one of ours
        sub     static.SRIOcount, static.SRIOcount, 1   // Decrement the count if it is ours
SrioNotInRange:
        // Look up configured output queue
        lsl     r0, SRIOproc.GarbageQNum, 2             // 4 bytes per config to avoid endian
        ldi     r1, QOSOFF_SRIO_GARBAGE_QS              // offset to config
        add     r0, r0, r1
        lbco    r0, cLocalRAM, r0, 4                    // load the configed queue
        lsl     r0, r0, 4                               // offset to q registers
        add     r0, r0, 8                               // offset to C register
        sbco    SRIOproc.PktSize, cQBase, r0, 8         // Write 2 regs
        jmp     SrioErrorPktLoop                        // Check the other error queues

SrioErrorDone:
        // 
        // It shouldn't happen unless SRIO mode were enabled with stray packets
        // on the queues, but in the event that the count becomes out of synch
        // to the point where it goes negative, we will reset it back to zero.
        // Eventually, the count will become accurate as this error is means 
        // that there is "too much credit". It is self leveling to a degree.
        //
        qbbc    SrioNoCountWrap, static.SRIOcount.t7
        mov     static.SRIOcount, 0     
SrioNoCountWrap:
        jmp     MainLoop

.leave SRIO_Processing 
//
// End SRIO Queue Processing
//
//====================================================================================        

