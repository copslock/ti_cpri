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
// acc.p - Accumulator Firmware
//
// Target: Nysh Queue Manager PDSP
//
// Description:
//
// This code implements the PDSP accumulator firmware
//
// Copyright 2009 - 2015 by Texas Instruments Inc.
//----------------------------------------------------------------------------------
// Revision History:
//
// 05-Feb-09 : MAD : Version 0.00 : First pass test code
// 09-Feb-09 : MAD : Version 0.01 : Changed channels 32-47 to "low priority"
// 19-Mar-10 : MAD : Version 1.00 : Added version numbering
// 21-Apr-10 : MAD : Version 1.01 : Added the ability to program the timer
// 06-Dec-10 : MAD : Version 1.02 : Added reclamation queue feature
// 29-Mar-11 : MAD : Version 1.03 : Fixed bug in reclamation queue with monolithic
//                                : and masking queue to 2048 instead of 8192
// 31-Aug-11 : MAD : Version 1.04 : Added diversion queue functionality
// 26-Oct-11 : MAD : Version 1.05 : Fixed bug in queue diversion (was masking off hint)
// 19-Jan-12 : John Dowdal : Version 1.06 : hint should be 4 bits instead of 5
//                                          and allow for 16K queues
// 07-Feb-12 : John Dowdal : Version 1.07 : Reclaimation had 8K mask instead of 16K
// 25-Aug-12 : John Dowdal : Version 1.08 : Require specification of PDSP # as part
//                                          of download process (for 8 PDSP K2)
// 08-May-14 : John Dowdal : Version 1.09 : Check list/ping pong size (MaxEntries) 
//                                          for over run.
// 20-Oct-15 : John Dowdal : Version 1.10 : Re-enable K1
// 04-Oct-16 : John Dowdal : Version 1.11 : DDR/MSMC coherence order barrier
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

#ifdef _CHANNEL48_
#define ACCOFF_CHAN_LIST    0x0400
#define CHANNELS            48          // 16, 32, 48
#define RAM_SIZE            0x4000      // 0x2000, 0x3000, 0x4000
#ifdef KEYSTONE1
#define cLocalRAM           cLRAM1
#define LOCAL_BASE          LRAM1_BASE
#else
#define cLocalRAM           cRAMPRIV
#endif
#define TIMER_IPS           40000       // 25us=40000, 20us=50000, 10us=100000
#define DMAVALIDMASK        0b1111
#define INTBASE             0
#define PROFILEID           3           // For profile testing
#define _MULT320_           1
#endif

#ifdef _CHANNEL32_
#define ACCOFF_CHAN_LIST    0x02C0
#define CHANNELS            32          // 16, 32, 48
#define RAM_SIZE            0x3000      // 0x2000, 0x3000, 0x4000
#ifdef KEYSTONE1
#define cLocalRAM           cLRAM1
#define LOCAL_BASE          LRAM1_BASE
#else
#define cLocalRAM           cRAMPRIV
#endif
#define TIMER_IPS           40000       // 25us=40000, 20us=50000, 10us=100000
#define DMAVALIDMASK        0b0011
#define INTBASE             0
#define PROFILEID           3           // For profile testing
#define _MULT352_           1
#endif

#ifdef _CHANNEL16_
#define ACCOFF_CHAN_LIST    0x0180
#define CHANNELS            16          // 16, 32, 48
#define RAM_SIZE            0x2000      // 0x2000, 0x3000, 0x4000
#ifdef KEYSTONE1
#define cLocalRAM           cLRAM2
#define LOCAL_BASE          LRAM2_BASE
#else
#define cLocalRAM           cRAMPRIV
#endif
#define TIMER_IPS           40000       // 25us=40000, 20us=50000, 10us=100000
#define DMAVALIDMASK        0b1100
#define INTBASE             32
#define PROFILEID           4           // For profile testing
#define _MULT480_           1
#endif

//
// ENDIAN DEPENDENT STRUCTURES
//
// Structures that are shared with the host must be defined to
// represent the same byte ordering in the host, regardless of
// endian mode. Thus there is one set for Little Endian and
// one set for Big Endian.
//
#ifdef _LE_

//
// Accumulator Channel Command
//
.struct struct_acccmd
    .u8     Channel         // Channel (0 - Max)
    .u8     Command         // Channel Command
    .u8     res1
    .u8     ReturnCode
    .u32    QMask           // Mask of active queues
    .u32    ListBuf         // Pointer to ping/pong list buffer
    .u16    Queue           // Queue to monitor
    .u16    MaxEntries      // Max entries per page (including terminator)
    .u16    TimerLoad       // Timer load count (in 25us ticks)
    .u8     Config          // Channel Congifuration
    .u8     res2
.ends

//
// Accumulator Channel Status
//
.struct struct_acc
    .u32    QMask           // Mask of active queues
    .u32    ListBuf         // Pointer to ping/pong list buffer
    .u16    Queue           // Queue to monitor
    .u16    MaxEntries      // Max entries per page (including terminator)
    .u16    TimerLoad       // Timer load count (in 25us ticks)
    .u8     Config          // Channel Congifuration
#define fMultiMode   t5         // Multi-Queue mode when set
#define fCountMode   t4         // Entry count mode when set     
    .u8     Status          // Channel Status
#define fDMAPending  t2         // Channel has a DMA pending
#define fTimerRun    t1         // Timer running when set
#define fPageOne     t0         // When set, page one is next (not page zero)
    .u16    EntryCnt        // Entry Count
    .u16    TimerCnt        // Timer Count
.ends

#else

//
// Accumulator Channel Command
//
.struct struct_acccmd
    .u8     ReturnCode
    .u8     res1
    .u8     Command         // Channel Command
    .u8     Channel         // Channel (0-31)
    .u32    QMask           // Mask of active queues
    .u32    ListBuf         // Pointer to ping/pong list buffer
    .u16    MaxEntries      // Max entries per page (including terminator)
    .u16    Queue           // Queue to monitor
    .u8     res2
    .u8     Config          // Channel Congifuration
    .u16    TimerLoad       // Timer load count (in 25us ticks)
.ends

//
// Accumulator Channel Status
//
.struct struct_acc
    .u32    QMask           // Mask of active queues
    .u32    ListBuf         // Pointer to ping/pong list buffer
    .u16    MaxEntries      // Max entries per page (including terminator)
    .u16    Queue           // Queue to monitor
    .u8     Status          // Channel Status
#define fDMAPending  t2         // Channel has a DMA pending
#define fTimerRun    t1         // Timer running when set
#define fPageOne     t0         // When set, page one is next (not page zero)
    .u8     Config          // Channel Congifuration
#define fMultiMode   t5         // Multi-Queue mode when set
#define fCountMode   t4         // Entry count mode when set     
    .u16    TimerLoad       // Timer load count (in 25us ticks)
    .u16    TimerCnt        // Timer Count
    .u16    EntryCnt        // Entry Count
.ends

#endif

//
// ENDIAN AGNOSTIC STRUCTURES
//
// Structures that are private to the PDSP's can be allowed to
// alter their byte ordering according to endian mode. This is
// done automatically by the assembler.
//

//
// Static Variables
//
.struct struct_static
    .u32    ChanListAddr      // Base address of channel list
    .u32    ChanActiveL       // Active channel flags (0-31)
    .u16    ChanActiveH       // Active channel flags (32-47)
    .u16    ReclaimQ          // Reclamation Queue Index
    .u16    BarrierInfDDRQ    // PKTDMA barrier for coherence workaround (all DDR)
    .u16    BarrierInfMSMCQ   // PKTDMA barrier for coherence workaround (DDR or MSMC)
    .u16    BarrierNetCPDDRQ  // NETCP barrier for coherence workaround (all DDR)
    .u16    BarrierNetCPMSMCQ // NETCP barrier for coherence workaround (DDR or MSMC)
    .u16    BarrierMSMCMSW    // MSW for base of MSMC (only used with MSMCQs)
    .u16    BarrierDDRMSW     // MSW for base of DDR (only used with MSMCQs)
    .u32    DiversionQ        // Diversion and Diversion Completion Queue
    .u8     ChanCurrent       // Current Channel
    .u8     LowCurrent        // Current Low Priority Channel
    .u8     Flags             // Status flags
#define fTimerTick      t0      // Timer was active this loop
#define fAccActivity    t7      // New activity on current channel
#define FLGMASK_CLEAR_ACC 0x7F  // AND mask to clear all ACC flags
    .u8     InUse   
#define fDMA0           t0      // DMA Busy Flag
#define fDMA1           t1      // DMA Busy Flag
#define fDMA2           t2      // DMA Busy Flag
#define fDMA3           t3      // DMA Busy Flag
#define fDMA0r          t4      // DMA Read Flag
#define fDMA1r          t5      // DMA Read Flag
#define fDMA2r          t6      // DMA Read Flag
#define fDMA3r          t7      // DMA Read Flag
    .u8     DMA0Chan        // Channel blocked in DMA0
    .u8     DMA1Chan        // Channel blocked in DMA1
    .u8     DMA2Chan        // Channel blocked in DMA2
    .u8     DMA3Chan        // Channel blocked in DMA3
.ends

//
// Local Variables for ACC
//
.struct struct_al
    .u16    FillQueue
    .u16    Count
    .u32    PendingMask
    .u32    LocalPtr
    .u32    AccPtr
    .u32    qWord0          
    .u32    qWord1          
    .u32    qWord2          
    .u32    qWord3          
.ends

//
// Local Variables for ServiceDMA
//
.struct struct_sd
    .u8     DMAPendFlags
    .u8     DMAChan
    .u8     AccChan
    .u8     res
.ends    

//
// DMA Registers
//
.struct struct_dma
    .u32    SrcAddr
    .u32    DstAddr
    .u32    Control
.ends    


//
// Packet descriptor format
//
.struct struct_desc
    .u32    DescInfo    // 31:30 DescType: HOST(0), MONO(2)
    .u32    SrcDstTag
    .u32    PktInfo     //  31   EPIB is present (1)
                        //  15   Ret Policy: Linked(0), Unlinked(1) (HOST)
                        //  14   Ret Push Policy: Tail(0), Head(1)  (HOST)
                        // 13:12 Ret QM 
                        // 11: 0 Ret Queue
    .u32    DataLength  // HOST mode only
    .u32    DataPtr     // HOST mode only
    .u32    NextPtr     // HOST mode only
.ends    

//
// Packet descriptor addendum
//
.struct struct_desc_add
    .u32    OrgBufLength    // Original pointer to the physical data buffer
    .u32    OrgBufPtr       // Original pointer to the physical data buffer
    .u32    Timestamp
.ends

//
// Reclamation Handling
//
.struct struct_reclaim
    .u32    Temp1   
    .u32    Temp2  
.ends


//
// Barrier Handling
//
.struct struct_barrier
    .u32    cReg   
    .u32    dReg 
    .u32    readAddr
    .u8     tripCount
    .u8     retQOffset
    .u16    retQ
    .u32    srcQReg         
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
.assign struct_static,               R2,  R10, static      

// The descriptor mapping is only used in reclamation, diversion and barrier
.assign struct_desc,                R11, R16, desc      
.assign struct_desc_add,            R17, R19, desc_add
.assign struct_reclaim,             R20, R21, reclaim
.assign struct_barrier,             R11, R15, barrier

// The remaining structures are used in the main loop of accumulation
.assign struct_acccmd,              R11, R15, acccmd        
.assign struct_acc,                 R12, R16, acc        // Must start 1 reg after accmcd!     
.assign struct_al,                  R17, R24, al
.assign struct_dma,                 R25, R27, dma
.assign struct_sd,                  R28, R28, sd


#define DMACTRL  ((1<<31)|(4<<26)|(3<<20))

.macro ldi32
.mparam reg, val
        ldi     reg.w0, val & 0xffff
        ldi     reg.w2, val >> 16
.endm

//-------------------------------------------------------------------
//
// Code Starts
//
        .entrypoint Header
        .origin     0
Header:
        jmp     Start
#ifdef _LE_
#define HEADER_MAGIC   0x80010000 + (CHANNELS<<8)      // "ACC"-0
#else
#define HEADER_MAGIC   0x80010001 + (CHANNELS<<8)      // "ACC"-1
#endif
#define HEADER_VERSION 0x0100000B                      // 0x01.0x00.0x00.0x0B
        .codeword  HEADER_MAGIC
        .codeword  HEADER_VERSION
Start:
        //--------------------------------------------------------------
        //
        // Initialization
        //

        // Timer Init
        ldi     r0, 0
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        ldi     r0, (350000000/TIMER_IPS)/2         // Get ticks per 25us with /2 div
        sbco    r0, cTimer, TIMER_OFF_LOAD, 4
        ldi     r0, 1<<15 | 0b0000<<2 | 0b11        // Enable | /2 | Mode+Go
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4

        // Init the static variables        
        zero    &static, SIZE(static)
        mov     static.DMA0Chan, 0xFF
        mov     static.DMA1Chan, 0xFF
        mov     static.DMA2Chan, 0xFF
        mov     static.DMA3Chan, 0xFF
        mov     static.LowCurrent, 47

#ifdef KEYSTONE2
        // Calculate base of IRAM
        // LRAM1_BASE and LRAM2_BASE must be defined (systems always have at least 2 PDSP)
        lbco    r0, cLocalRAM, ACCOFF_CPUNUM, 4
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM1_BASE)
        qbeq    IramDone, r0, 1
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM2_BASE)
        qbeq    IramDone, r0, 2
#ifdef LRAM3_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM3_BASE)
        qbeq    IramDone, r0, 3
#endif
#ifdef LRAM4_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM4_BASE)
        qbeq    IramDone, r0, 4
#endif
#ifdef LRAM5_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM5_BASE)
        qbeq    IramDone, r0, 5
#endif
#ifdef LRAM6_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM6_BASE)
        qbeq    IramDone, r0, 6
#endif
#ifdef LRAM7_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM7_BASE)
        qbeq    IramDone, r0, 7
#endif
#ifdef LRAM8_BASE
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LRAM8_BASE)
        qbeq    IramDone, r0, 8
#endif
        // Invalid PDSP number
        ldi32   r0, 0xfee1de00
        ldi     r1, ACCOFF_CPUNUM
        sbco    r0, cLocalRAM, r1, 4
BadPdspNum:
        jmp     BadPdspNum
#else
        ldi32   static.ChanListAddr, (ACCOFF_CHAN_LIST + LOCAL_BASE)
#endif
IramDone:
        // Block any DMA channel we're not using
        mov     static.InUse, DMAVALIDMASK ^ 0xF

        // Zero out our memory
        mov     r0, RAM_SIZE
        mov     r1, 0
ZeroLoop:        
        sub     r0, r0, 4
        sbco    r1, cLocalRAM, r0, 4
        qbne    ZeroLoop, r0, 0

        // Mark download successful
        // Save the version key to scratch
        // This allows host cores to check version of preloaded firmware
        // matches the LLD, since the host cores cant read program when running
        ldi32   r1, HEADER_MAGIC
        ldi     r0, ACCOFF_VERSION
        sbco    r1, cLocalRAM, r0, 4
        add     r0, r0, 4
        ldi32   r1, HEADER_VERSION
        sbco    r1, cLocalRAM, r0, 4

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
        mov     r1, PROFILEID
        sbbo    r1, r0, 0, 4
        // Test Code for Profiling
        //-------------------------------
#endif        

        //
        // Check the time Status
        //
        clr     static.Flags.fTimerTick             // Assume no timer tick
        qbbc    NoTimerTick, r31.tStatus_Timer      // Look for a timer tick
        set     r31.tStatus_Timer                   // Ack the interrupt
        set     static.Flags.fTimerTick             // Set the timer flag
NoTimerTick:        

        // Cycle the low priority channel
        add     static.LowCurrent, static.LowCurrent, 1
        qbne    LPNotWrapped, static.LowCurrent, 48
        mov     static.LowCurrent, 32
LPNotWrapped:

        //
        // Get a user command
        //
        lbco    acccmd, cLocalRAM, ACCOFF_CHAN_COMMAND, SIZE(acccmd)
        qbbc    NoCommand, acccmd.Command.t7
        qbne    NotCmdDisable, acccmd.Command, ACMD_DISABLE_CHAN
        
        //-------------------------
        //
        // Command: Channel Disable
        //
        qbgt    DisChanRangeValid, acccmd.Channel, CHANNELS
        mov     acccmd.ReturnCode, ACMD_RETCODE_INVALID_CHANNEL
        jmp     CmdDone
DisChanRangeValid:        
        qbgt    DisChanLowChannel, acccmd.Channel, 32
        sub     r0, acccmd.Channel, 32
        qbbs    DisChanHighValid, static.ChanActiveH, r0
DisChanNotActive:
        mov     acccmd.ReturnCode, ACMD_RETCODE_CHANNEL_NOT_ACTIVE
        jmp     CmdDone
DisChanHighValid:        
        clr     static.ChanActiveH, r0
        jmp     DisChanCleared
DisChanLowChannel:
        qbbc    DisChanNotActive, static.ChanActiveL, acccmd.Channel
        clr     static.ChanActiveL, acccmd.Channel
DisChanCleared:

        //
        // Fire an interrupt for packets we've already read, but not
        // previously made known to the host
        //
        mov     static.ChanCurrent, acccmd.Channel
        lsl     r0, static.ChanCurrent, 4           // r0 = channel * 16
        lsl     r1, static.ChanCurrent, 2           // r1 = channel * 4
        add     r0, r0, r1                          // r0 = channel * 20
        add     r0, r0, ACCOFF_CHAN_STATUS          // r0 = ptr to channel status
        lbco    acc, cLocalRAM, r0, SIZE(acc)
        and     static.Flags, static.Flags, FLGMASK_CLEAR_ACC // Clear flags
        qbeq    DisNoInt, acc.EntryCnt, 0           // No INT needed for 0 entries
        call    ForceInterrupt                      // Fire one last interrupt
DisNoInt:

        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone
        
NotCmdDisable:
        qbne    NotCmdEnable, acccmd.Command, ACMD_ENABLE_CHAN
        
        //-------------------------------------------------------------
        //
        // Command: Channel Enable
        //
        mov     r0, QM_MAX_QUEUES
        qbgt    EnQueueValid, acccmd.Queue, r0
        mov     acccmd.ReturnCode, ACMD_RETCODE_INVALID_QUEUE
        jmp     CmdDone
EnQueueValid:        
        qbgt    EnChanRangeValid, acccmd.Channel, CHANNELS
        mov     acccmd.ReturnCode, ACMD_RETCODE_INVALID_CHANNEL
        jmp     CmdDone
EnChanRangeValid:        
        ldi     r0.w0, ((RAM_SIZE - ACCOFF_CHAN_LIST) / CHANNELS)  // r0.w0 = channel region size in bytes
        and     r0.w2, acccmd.Config, 0b1100        // Bits 3:2 are the list entry size
        lsr     r0.w2, r0.w2, 2                     // r0.w2 contains 0 for 4 byte entries, 1, for 8 byte, 2, for 16 byte
        add     r0.w2, r0.w2, 2                     // r0.w2 contains entry size in 2^x (2 for 4 byte, 3 for 8 byte, 4 for 16 byte)
        lsl     r0.w2, acccmd.MaxEntries, r0.w2     // r0.w2 contains chan region req in bytes
        qbge    EnChanMaxEntriesValid, r0.w2, r0.w0 // r0.w0 >= r0.w2 -> region size >= req size
        mov     acccmd.ReturnCode, ACMD_RETCODE_INVALID_MAXENTRIES
        jmp     CmdDone
EnChanMaxEntriesValid:
        qbgt    EnChanLowChannel, acccmd.Channel, 32
        sub     r0, acccmd.Channel, 32
        qbbc    EnChanHighValid, static.ChanActiveH, r0
EnChanActive:
        mov     acccmd.ReturnCode, ACMD_RETCODE_CHANNEL_ACTIVE
        jmp     CmdDone
EnChanHighValid:
        set     static.ChanActiveH, r0
        jmp     EnChanSet
EnChanLowChannel:
        qbbs    EnChanActive, static.ChanActiveL, acccmd.Channel
        set     static.ChanActiveL, acccmd.Channel
EnChanSet:

        // Initialize the internal fields
        mov     acc.Status, 0               
        mov     acc.EntryCnt, 0
        mov     acc.TimerCnt, 0

        // If timer mode 1, expire timer now
        qbbs    EnNotTimerMode1, acc.Config.t1
        qbbc    EnNotTimerMode1, acc.Config.t0
        set     acc.Status.fTimerRun                // Signal a countdown (count is NULL)
EnNotTimerMode1:
        
        // Write out the internal record
        lsl     r0, acccmd.Channel, 4               // r0 = channel * 16
        lsl     r1, acccmd.Channel, 2               // r0 = channel * 4
        add     r0, r0, r1                          // r0 = channel * 20
        add     r0, r0, ACCOFF_CHAN_STATUS          // r0 = ptr to channel status
        sbco    acc, cLocalRAM, r0, SIZE(acc)

        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone
        
NotCmdEnable:
        qbne    NotCmdSetTimer, acccmd.Command, ACMD_SET_TIMER
        
        //-------------------------------------------------------------
        //
        // Command: Set Timer Constant
        //
        
        // Stop the timer so we can reload
        ldi     r0, 0
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4       // Stop timer
        set     r31.tStatus_Timer                   // Ack any interrupt

        // The timer count value is supplied in QMask
        sbco    acccmd.QMask, cTimer, TIMER_OFF_LOAD, 4
        ldi     r0, 1<<15 | 0b0000<<2 | 0b11        // Enable | /2 | Mode+Go
        sbco    r0, cTimer, TIMER_OFF_CTRL, 4
        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone


NotCmdSetTimer:
        qbne    NotCmdSetReclamation, acccmd.Command, ACMD_SET_RECLAMATION
        
        //-------------------------------------------------------------
        //
        // Command: Set Reclamation Queue
        //
        mov     static.ReclaimQ, acccmd.QMask
        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone

NotCmdSetReclamation:

        qbne    NotCmdSetDiversion, acccmd.Command, ACMD_SET_DIVERSION
        
        //-------------------------------------------------------------
        //
        // Command: Set Diversion Queue
        //
        mov     static.DiversionQ, acccmd.QMask     // .w0=Divert .w2=Completion
        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone

NotCmdSetDiversion:
#ifdef KEYSTONE2
        qbne    NotCmdSetDDRBarrier, acccmd.Command, ACMD_SET_DDR_BARRIER
        
        //-------------------------------------------------------------
        //
        // Command: Set DDR Barrier Queue
        //
        mov     static.BarrierInfDDRQ, acccmd.QMask.w0  
        mov     static.BarrierNetCPDDRQ, acccmd.QMask.w2  
        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone

NotCmdSetDDRBarrier:
        qbne    NotCmdSetMSMCBarrier, acccmd.Command, ACMD_SET_MSMC_BARRIER
        
        //-------------------------------------------------------------
        //
        // Command: Set MSMC Barrier Queue
        //
        mov     static.BarrierInfMSMCQ, acccmd.QMask.w0
        mov     static.BarrierNetCPMSMCQ, acccmd.QMask.w2
        mov     static.BarrierMSMCMSW, acccmd.ListBuf.w0 
        mov     static.BarrierDDRMSW, acccmd.ListBuf.w2 
        mov     acccmd.ReturnCode, ACMD_RETCODE_SUCCESS
        jmp     CmdDone

NotCmdSetMSMCBarrier:
#endif
        //-------------------------------------------------------------
        //
        // Invalid Command
        //
        mov     acccmd.ReturnCode, ACMD_RETCODE_INVALID_COMMAND
        // Wrap up command
CmdDone:
        mov     acccmd.Command, 0
        sbco    acccmd, cLocalRAM, 0, 4

NoCommand:

        //-------------------------------------------------------------
        //
        // Service the reclamation queue when enabled
        //
        qbeq    NoReclamation, static.ReclaimQ, 0
        call    ServiceReclamation
NoReclamation:

        //-------------------------------------------------------------
        //
        // Service the diversion queue when enabled
        //
        qbeq    NoDiversion, static.DiversionQ.w0, 0
        call    ServiceDiversion
NoDiversion:

#ifdef KEYSTONE2
        //-------------------------------------------------------------
        //
        // Service the DDR and MSMC barriers
        //
        call    ServiceBarrier
#endif

        //-------------------------------------------------------------
        //
        // Service any pending DMA
        //
        call    ServiceDMA

        //-------------------------------------------------------------
        //
        // Process Accumulator Channels
        //
        mov     static.ChanCurrent, 0
ChannelStart:
        //
        // If there's not at least 1 DMA channel free, then service
        // until there is.
        //
        and     r0, static.InUse, 0xF
        qbne    DMANotBlocked, r0, 0xF
        call    ServiceDMA
        jmp     ChannelStart
DMANotBlocked:
        
        // Make sure channel is active
        qbgt    ChannelLow, static.ChanCurrent, 32
        sub     r0, static.ChanCurrent, 32
        qbbc    ChannelSkip, static.ChanActiveH, r0          
        jmp     ChannelActive
ChannelLow:
        qbbc    ChannelSkip, static.ChanActiveL, static.ChanCurrent          
ChannelActive:
        and     static.Flags, static.Flags, FLGMASK_CLEAR_ACC // Clear flags

        // Read in the internal record
        lsl     r0, static.ChanCurrent, 4           // r0 = channel * 16
        lsl     r1, static.ChanCurrent, 2           // r1 = channel * 4
        add     r0, r0, r1                          // r0 = channel * 20
        add     al.AccPtr, r0, ACCOFF_CHAN_STATUS   // al.AccPtr = ptr to channel status
        lbco    acc, cLocalRAM, al.AccPtr, SIZE(acc)

        // If we are pendinging DMA, then we are fully stalled
        qbbs    ChannelSkip, acc.Status.fDMAPending

        // Processing Steps
        //
        // 1. Add more entries to the existing list if possible
        // 2. Service timer as required
        // 3. Start interrupt process as needed

        //
        // 1. Add more entries to the existing list if possible
        //

        // Jump out to Interrupt now if list is full
        add     r0, acc.EntryCnt, 1
        qbeq    NeedInterrupt, r0, acc.MaxEntries

        // Jump out of Fill now if no entries are pending
        lsr     r0, acc.Queue, 5                    // r0 = Queue Number / 32
        lsl     r0, r0, 2                           // Get word address
        lbco    r1, cQPending, r0, 4                // r1 = pending bits
        qbbs    ProcessMulti, acc.Config.fMultiMode
        qbbc    FillComplete, r1, acc.Queue         // Jump if no packet pending        
        
        //
        // Fill from single queue
        //
        mov     al.FillQueue, acc.Queue
        call    FillQueue                           // Fill from queue in FillQueue

        // Check for full (and immediate int)
        add     r0, acc.EntryCnt, 1
        qbeq    NeedInterrupt, r0, acc.MaxEntries
        jmp     FillComplete

        //
        // Fill from multi-queue
        //
ProcessMulti:
        and     al.PendingMask, r1, acc.QMask
MultiLoop:
        lmbd    al.FillQueue, al.PendingMask, 1
        qbeq    FillComplete, al.FillQueue, 32      // Jump out if none pending
        clr     al.PendingMask, al.FillQueue        // Clear the flag we're servicing
        add     al.FillQueue, al.FillQueue, acc.Queue
        call    FillQueue                           // Fill from queue in FillQueue
        // Loop on pending bits if list is not full
        add     r0, acc.EntryCnt, 1
        qbne    MultiLoop, r0, acc.MaxEntries
        // Else jump right to Interrupt
        jmp     NeedInterrupt
FillComplete:

        //
        // 2. Service timer as required
        //
    
        // If we're here, we know we're not full
        // Jump to timer if no activity
        qbbc    ChannelTimer, static.Flags.fAccActivity

        // New activity detected.
        // Load the timer on two conditions:
        // 1. User wants timer on first activity and the timer isn't loaded yet
        // 2. Uset wants timer on last activity
        qbbc    NoTimerLoad, acc.Config.t1          // Timer modes 0 and 1 not loaded here
        qbbs    SetTimer, acc.Config.t0             // Always set timer on mode 3
        qbbs    NoTimerLoad, acc.Status.fTimerRun   // Don't load timer on mode 2 if running
SetTimer:
        mov     acc.TimerCnt, acc.TimerLoad
        set     acc.Status.fTimerRun 
NoTimerLoad:

ChannelTimer:
        // Jump out if timer is not running
        qbbc    ChannelDone, acc.Status.fTimerRun 
        // See if timer already expired
        qbeq    TimerExpired, acc.TimerCnt, 0   
        // Jump out if no timer tick this run
        qbbc    ChannelDone, static.Flags.fTimerTick
        // Decrement count
        sub     acc.TimerCnt, acc.TimerCnt, 1
        // Jump out if timer not expired
        qbne    ChannelDone, acc.TimerCnt, 0

TimerExpired:
        qbeq    ChannelDone, acc.EntryCnt, 0        // No INT if there are no entries
NeedInterrupt:

        //
        // 3. Start interrupt process as needed
        //

        // The channel will "soft-stall" if the INTD count is not zero
        add     r1, static.ChanCurrent, INTBASE
        qbgt    LowInt, r1, 32
        sub     r1, r1, 32         
        lbco    r0, cIntStatus, 0xC, 4              // Read the interrupt pending flags
        jmp     IntCheck                           
LowInt:
        lbco    r0, cIntStatus, 0x8, 4              // Read the interrupt pending flags
IntCheck:
        qbbs    ChannelDone, r0, r1
        call    CloseAndCopy        

ChannelDone:
        sbco    acc, cLocalRAM, al.AccPtr, SIZE(acc)   // Write out the status record
ChannelSkip:
        qblt    MainLoop, static.ChanCurrent, 31    // Jump out if this was low-pri
        qbeq    DoLowPri, static.ChanCurrent, 31
        add     static.ChanCurrent, static.ChanCurrent, 1
        qbgt    ChannelStart, static.ChanCurrent, CHANNELS
        jmp     MainLoop
DoLowPri:
        mov     static.ChanCurrent, static.LowCurrent 
        qbgt    ChannelStart, static.ChanCurrent, CHANNELS
        jmp     MainLoop


//---------------------------------------------------------------------
// FillQueue - Attempts to fill the list from the supplied queue
//
// Assumes:     + al.FillQueue
//              + static.ChanCurrent
//              + acc is valid
// Overwrites:  ---
//
FillQueue:
        // Setup Local Page Address
        mov     al.LocalPtr, static.ChanListAddr
#ifdef _MULT320_
        lsl     r0, static.ChanCurrent, 2           // r0 = channel * 4
        add     r0, r0, static.ChanCurrent          // r0 = channel * 5
        lsl     r0, r0, 6                           // r0 = channel * 320
#endif
#ifdef _MULT352_
        lsl     r0, static.ChanCurrent, 3           // r0 = channel * 8
        add     r0, r0, static.ChanCurrent          // r0 = channel * 9
        add     r0, r0, static.ChanCurrent          // r0 = channel * 10
        add     r0, r0, static.ChanCurrent          // r0 = channel * 11
        lsl     r0, r0, 5                           // r0 = channel * 352
#endif
#ifdef _MULT480_
        lsl     r0, static.ChanCurrent, 4           // r0 = channel * 16
        sub     r0, r0, static.ChanCurrent          // r0 = channel * 15
        lsl     r0, r0, 5                           // r0 = channel * 480
#endif
        add     al.LocalPtr, al.LocalPtr, r0

FillQueueLoop:
        // Get the max entries we can read this go
        add     al.Count, acc.EntryCnt, 1
        sub     al.Count, acc.MaxEntries, al.Count
        // Jump to page full if no room
        qbeq    FillQueueDone, al.Count, 0
        
        // Do the next processing by record size
        and     r0.b0, acc.Config, 0b1100           // Bits 3:2 are the list size
        qbeq    ProcRec8, r0.b0, 1<<2
        qbeq    ProcRec16, r0.b0, 2<<2

        //
        // 4 Byte Record Processing (up to 4 records)
        //
        min     al.Count, al.Count, 4
        lsl     r0, al.FillQueue, 4                 // r0 = Queue * 16
        add     r0, r0, 0xC
        // Read record 0
        lbco    al.qWord0, cQBase, r0, 4
        qbeq    FillQueueDone, al.qWord0, 0
        qbge    Rec4ReadDone, al.Count, 1        
        // Read record 1
        lbco    al.qWord1, cQBase, r0, 4
        qbne    Rec4HaveWord1, al.qWord1, 0
        mov     al.Count, 1
Rec4HaveWord1:
        qbge    Rec4ReadDone, al.Count, 2        
        // Read record 2
        lbco    al.qWord2, cQBase, r0, 4
        qbne    Rec4HaveWord2, al.qWord2, 0
        mov     al.Count, 2
Rec4HaveWord2:
        qbge    Rec4ReadDone, al.Count, 3        
        // Read record 3
        lbco    al.qWord3, cQBase, r0, 4
        qbne    Rec4ReadDone, al.qWord3, 0
        mov     al.Count, 3
Rec4ReadDone:        
        lsl     r1, acc.EntryCnt, 2                 // Get offset to next entry in bytes
        qbbc    Rec4EntryPtrSet, acc.Config.fCountMode
        add     r1, r1, 4                           // Add a slot for count
Rec4EntryPtrSet:        
        lsl     r0.b0, al.Count, 2                  // Byte len in r0.b0
        sbbo    al.qWord0, al.LocalPtr, r1, b0      // Write out 4 to 16 bytes record
        // Add these entries
        add     acc.EntryCnt, acc.EntryCnt, al.Count
        // Set the activity flag
        set     static.Flags.fAccActivity
        // If the count was 4, get some more
        qbeq    FillQueueLoop, al.Count, 4
        jmp     FillQueueDone      

ProcRec8:   
        //
        // 8 Byte Record Processing (up to 2 records)
        //
        min     al.Count, al.Count, 2
        lsl     r0, al.FillQueue, 4                 // r0 = Queue * 16
        add     r0, r0, 0x8
        // Read record 0
        lbco    al.qWord0, cQBase, r0, 8
        qbeq    FillQueueDone, al.qWord1, 0
        mov     al.qWord0.w2, al.FillQueue
        qbeq    Rec8ReadDone, al.Count, 1
        // Read record 1
        lbco    al.qWord2, cQBase, r0, 8
        qbne    Rec8ReadDone, al.qWord3, 0
        mov     al.Count, 1
Rec8ReadDone:        
        mov     al.qWord2.w2, al.FillQueue
        lsl     r1, acc.EntryCnt, 3                 // Get offset to next entry in bytes
        qbbc    Rec8EntryPtrSet, acc.Config.fCountMode
        add     r1, r1, 8                           // Add a slot for count
Rec8EntryPtrSet:        
        lsl     r0.b0, al.Count, 3                  // Byte len in r0.b0
        sbbo    al.qWord0, al.LocalPtr, r1, b0      // Write 8 byte record
        // Add these entries
        add     acc.EntryCnt, acc.EntryCnt, al.Count
        // Set the activity flag
        set     static.Flags.fAccActivity
        // If the count was 2, get some more
        qbeq    FillQueueLoop, al.Count, 2
        jmp     FillQueueDone

ProcRec16:
        //
        // 16 Byte Record Processing
        //
        min     al.Count, al.Count, 1
        lsl     r0, al.FillQueue, 4                 // r0 = Queue * 16
        lbco    al.qWord0, cQBase, r0, 16
        qbeq    FillQueueDone, al.qWord3, 0
        mov     al.qWord2.w2, al.FillQueue
        lsl     r1, acc.EntryCnt, 4                 // Get offset to next entry in bytes
        qbbc    Rec16EntryPtrSet, acc.Config.fCountMode
        add     r1, r1, 16                          // Add a slot for count
Rec16EntryPtrSet:        
        sbbo    al.qWord0, al.LocalPtr, r1, 16      // Write 16 byte record
        // Add this entry
        add     acc.EntryCnt, acc.EntryCnt, 1
        // Set the activity flag
        set     static.Flags.fAccActivity
        // Check for more
        jmp     FillQueueLoop

FillQueueDone:
        ret



//---------------------------------------------------------------------
// CloseAndCopy - Close list and copy to DMA
//
// Assumes:     + static.ChanCurrent
//              + One free DMA channel
//              + Non-zero list
//              + acc is valid
// Overwrites:  al, dma
//
CloseAndCopy:
        // Setup Source Address
        mov     dma.SrcAddr, static.ChanListAddr
#ifdef _MULT320_
        lsl     r0, static.ChanCurrent, 2           // r0 = channel * 4
        add     r0, r0, static.ChanCurrent          // r0 = channel * 5
        lsl     r0, r0, 6                           // r0 = channel * 320
#endif
#ifdef _MULT352_
        lsl     r0, static.ChanCurrent, 3           // r0 = channel * 8
        add     r0, r0, static.ChanCurrent          // r0 = channel * 9
        add     r0, r0, static.ChanCurrent          // r0 = channel * 10
        add     r0, r0, static.ChanCurrent          // r0 = channel * 11
        lsl     r0, r0, 5                           // r0 = channel * 352
#endif
#ifdef _MULT480_
        lsl     r0, static.ChanCurrent, 4           // r0 = channel * 16
        sub     r0, r0, static.ChanCurrent          // r0 = channel * 15
        lsl     r0, r0, 5                           // r0 = channel * 480
#endif
        add     dma.SrcAddr, dma.SrcAddr, r0
        // Set R1 to be shift amount to multiply by element size
        lsr     r0, acc.Config, 2                   // Bits 3:2 are the list size
        and     r0, r0, 0b11
        add     r1, r0, 2
        // Close out the list
        // Write either the record count or the NULL terminator
        qbbc    NullTermMode, acc.Config.fCountMode
        mov     r0, acc.EntryCnt
        sbbo    r0, dma.SrcAddr, 0, 4               // Write record count
        jmp     PageClosed
NullTermMode:
        zero    &al.qWord0, 16
        mov     r0.b0, 1
        lsl     r0.b0, r0.b0, r1                    // R0.b0 = Element Size
        lsl     r0.w2, acc.EntryCnt, r1             // R0.w2 = Offset
        sbbo    al.qWord0, dma.SrcAddr, r0.w2, b0   // Write out NULL term 
PageClosed:
        // Assign the destination address
        mov     dma.DstAddr, acc.ListBuf
        qbbc    DstAddrSet, acc.Status.fPageOne
        lsl     r0, acc.MaxEntries, r1              // Get r0 = page size in bytes
        add     dma.DstAddr, dma.DstAddr, r0
DstAddrSet:
        // Setup Control
        add     r0, acc.EntryCnt, 1
        lsl     dma.Control.w0, r0, r1              // Size to copy
        mov     dma.Control.w2, DMACTRL>>16         // Control bits
        // Get DMA channel in R0
        or      r0.b0, static.InUse, 0xF0
        lmbd    r0, r0.b0, 0
        // Get DMA offset in R1
        lsl     r1, r0, 6
        // Start DMA
        sbco    dma, cDMA, r1, SIZE(dma)
        set     acc.Status.fDMAPending
        set     static.InUse, r0
        add     r1.b0, r0, &static.DMA0Chan
        mvib    *r1.b0, static.ChanCurrent
        ret


//---------------------------------------------------------------------
// ServiceDMA - Service Pending DMA Channels
//
// Assumes:     --- 
// Overwrites:  acc, al, dma, sd
//
ServiceDMA:
        // Get a all the pending DMA channels
        and     sd.DMAPendFlags, static.InUse, DMAVALIDMASK
ServiceDMALoop:
        // Get the next channel to check
        lmbd    sd.DMAChan, sd.DMAPendFlags, 1
        // Jump out if none left
        qbeq    SerivceDMADone, sd.DMAChan, 32
        // Clear the one we're about check
        clr     sd.DMAPendFlags, sd.DMAChan
        // Get DMA offset in R1
        lsl     r0, sd.DMAChan, 6
        // Load the DMA Info
        lbco    dma, cDMA, r0, SIZE(dma)
        qbbs    ServiceDMALoop, dma.control.t30     // Not done if active bit is set
        qbne    ServiceDMALoop, dma.control.w0, 0   // Not done if size not zero

        // Here the channel has completed
        add     r1, sd.DMAChan, 4                   // Get the "read" state bit
        qbbs    DoneRead, static.InUse, r1          // Jump if done with the read stage

        // We need to do a final read on this channel for synchronization
        set     static.InUse, r1                    // Set the read flag
        sub     r1, dma.SrcAddr, 4
        sub     dma.SrcAddr, dma.DstAddr, 4
        mov     dma.DstAddr, r1
        mov     dma.Control.w0, 4                   // Size to copy
        mov     dma.Control.w2, DMACTRL>>16         // Control bits
        sbco    dma, cDMA, r0, SIZE(dma)            // Program the DMA to read
        jmp     ServiceDMALoop

DoneRead:
        // Clear the DMA Channel pending
        clr     static.InUse, r1                    // Clear the read
        clr     static.InUse, sd.DMAChan            // Clear the base flag
        add     r1.b0, sd.DMAChan, &static.DMA0Chan
        mvib    sd.AccChan, *r1.b0
        // Read in the internal record
        lsl     r0, sd.AccChan, 4                   // r0 = channel * 16
        lsl     r1, sd.AccChan, 2                   // r1 = channel * 4
        add     r0, r0, r1                          // r0 = channel * 20
        add     al.AccPtr, r0, ACCOFF_CHAN_STATUS   // al.AccPtr = ptr to channel status
        lbco    acc, cLocalRAM, al.AccPtr, SIZE(acc)
        clr     acc.Status.fDMAPending              // Clear DMA pending
        xor     acc.Status, acc.Status, 1           // Toggle host memory page
        mov     acc.EntryCnt, 0                     // Clear the entry count
        clr     acc.Status.fTimerRun                // Reset the timer
        // Load the timer if the user wants pacing based on last interrupt
        qbbs    FireInt, acc.Config.t1              // Timer modes 2 and 3 not loaded here
        qbbc    FireInt, acc.Config.t0              // Timer mode 0 not loaded here
        mov     acc.TimerCnt, acc.TimerLoad
        set     acc.Status.fTimerRun 
FireInt:
        // Save out the record
        sbco    acc, cLocalRAM, al.AccPtr, SIZE(acc)
        // Fire the interrupt
        add     r0, sd.AccChan, INTBASE            
        mov     r1, 1
        qbgt    FILowChan, r0, 32
        sub     r0, r0, 32
        lsl     r1, r1, r0
        sbco    r1, cIntStatus, 4, 4                // Fire the interrupt
        jmp     ServiceDMALoop
FILowChan:        
        lsl     r1, r1, r0
        sbco    r1, cIntStatus, 0, 4                // Fire the interrupt
        jmp     ServiceDMALoop
SerivceDMADone:
        ret


//---------------------------------------------------------------------
// ForceInterrupt - Close list, copy to DMA, and Fire Interrupt
//
// Assumes:     + static.ChanCurrent
//              + Non-zero list
//              + acc is valid
// Overwrites:  acc, al, dma
//
ForceInterrupt:
        pushRet

SpinOnInt:
        // The channel will "soft-stall" if the INTD count is not zero
        add     r1, static.ChanCurrent, INTBASE
        qbgt    LowInt2, r1, 32
        sub     r1, r1, 32         
        lbco    r0, cIntStatus, 0xC, 4              // Read the interrupt pending flags
        jmp     IntCheck2                           
LowInt2:
        lbco    r0, cIntStatus, 0x8, 4              // Read the interrupt pending flags
IntCheck2:
        qbbs    SpinOnInt, r0, r1

        // If DMA is already pending, then skip to next part
        qbbs        FI_DMAPending, acc.Status.fDMAPending
        //
        // If there's not at least 1 DMA channel free, then service
        // until there is.
        //
        and     r0, static.InUse, 0xF
        qbne    FI_DMANotBlocked, r0, 0xF
FI_DMACheckLoop:        
        call    ServiceDMA
        and     r0, static.InUse, 0xF
        qbeq    FI_DMACheckLoop, r0, 0xF
        // Here we were blocked, so we have to re-load acc
        lsl     r0, static.ChanCurrent, 4           // r0 = channel * 16
        lsl     r1, static.ChanCurrent, 2           // r1 = channel * 4
        add     r0, r0, r1                          // r0 = channel * 20
        add     r0, r0, ACCOFF_CHAN_STATUS          // r0 = ptr to channel status
        lbco    acc, cLocalRAM, r0, SIZE(acc)
FI_DMANotBlocked:
        call    CloseAndCopy        

FI_DMAPending:
        call    ServiceDMA
        // Loop until all DMA transactions are done
        and     r0, static.InUse, 0xF
        qbne    FI_DMAPending, r0, 0

        popRet
        ret


//---------------------------------------------------------------------
// ServiceReclamation - Reclaim any packet on the reclamation queue
//
// Assumes:     ---
// Overwrites:  reclaim, desc
//
ServiceReclamation:
        lsl     r0, static.ReclaimQ, 4              // r0 = reclamation queue * 16
        add     r0, r0, 12                          // Just read D
        lbco    reclaim.Temp2, cQBase, r0, 4        // Pop packet
        qbeq    ReclamationDone, reclaim.Temp2, 0   // Jump out if empty
        and     reclaim.Temp2.b0, reclaim.Temp2.b0, 0xF0 // Mask off the hint (if any)

        // Read the descriptor
        lbbo    desc, reclaim.Temp2, 0, SIZE(desc)
        
        // Make sure of its type
        and     r0, desc.DescInfo.b3, 0xC0
        qbeq    HostDescriptor, r0, 0
        qbeq    MonoDescriptor, r0, 0x80

        // If we get here, it ain't good, but the best we can
        // do is ignore the problem and move on
        jmp     ReclamationDone
       
HostDescriptor:
        // Jump if we're splitting descriptors
        qbbs    HostDescriptorSplit, desc.PktInfo.t15
        mov     desc.NextPtr, 0                     // Force an end to the return loop
HostDescriptorSplit:
        mov     reclaim.Temp1, 0                    // Assume normal push to tail
        qbbc    HostDescriptorNormalPush,  desc.PktInfo.t14
        set     reclaim.Temp1.t31                   // Push to head
HostDescriptorNormalPush:
        // Free the descriptor
        and     desc.PktInfo.b1, desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 8                           // Write to C+D
        sbco    reclaim.Temp1, cQBase, r0, 8        // Return old desc 
        // Jump if we're done returning host desc
        qbeq    ServiceReclamation, desc.NextPtr, 0

        mov     reclaim.Temp2, desc.NextPtr
        and     reclaim.Temp2.b0, reclaim.Temp2.b0, 0xF0 // Mask off the hint (if any)

        // Load the descriptor info and return it
        lbbo    desc, reclaim.Temp2, 0, SIZE(desc)
        jmp     HostDescriptorSplit

MonoDescriptor:
        // Free the descriptor
        and     desc.PktInfo.b1, desc.PktInfo.b1, (QM_MAX_QUEUES - 1) >> 8 // We save PktInfo & 0x1FFF on k1 (or 0x3fff on k2)
        lsl     r0, desc.PktInfo.w0, 4              // r0 = queue offset old ret Q
        add     r0, r0, 12                          // Write to D
        sbco    reclaim.Temp2, cQBase, r0, 4        // Return old desc 
        jmp     ServiceReclamation                  // Get the next desc in the list

ReclamationDone:
        ret


//---------------------------------------------------------------------
// ServiceDiversion - Process any packet on the diversion queue
//
// Assumes:     ---
// Overwrites:  reclaim, desc
//
ServiceDiversion:
        lsl     r0, static.DiversionQ.w0, 4         // r0 = diversion queue * 16
        add     r0, r0, 12                          // Just read D
        lbco    reclaim.Temp2, cQBase, r0, 4        // Pop packet
        qbeq    DiversionDone, reclaim.Temp2, 0     // Jump out if empty
        mov     reclaim.Temp1, reclaim.Temp2        // Copy to masked pointer
        and     reclaim.Temp1.b0, reclaim.Temp1.b0, 0xF0 // Mask off the hint (if any)

        // Read the descriptor
        lbbo    desc, reclaim.Temp1, 0, SIZE(desc)+SIZE(desc_add)
        
        // Make sure of its type
        and     r0, desc.DescInfo.b3, 0xC0
        qbne    DiversionReturn, r0, 0

        // Make sure the timestamp field is available
        qbbc    DiversionReturn, desc.PktInfo.t31

        // Do diversion
        mov     r0, QM1_CFG + 8
        sbbo    desc_add.Timestamp, r0, 0, 4

DiversionReturn:
        lsl     r0, static.DiversionQ.w2, 4         // r0 = completion queue * 16
        add     r0, r0, 12                          // Just read D
        sbco    reclaim.Temp2, cQBase, r0, 4        // Push packet

DiversionDone:
        ret

#ifdef KEYSTONE2
//---------------------------------------------------------------------
// ServiceBarrier
//
// Assumes:     ---
//
#define BARRIER_MAX_TRIP     (16)
#ifdef _LE_
#define BARRIER_INF_RET_OFFSET   (4)  // src/dest tag
#define BARRIER_NETCP_RET_OFFSET (40) // LSBs of SWINFO 1
#else
#define BARRIER_INF_RET_OFFSET   (6)  // src/dest tag
#define BARRIER_NETCP_RET_OFFSET (42) // LSBs of SWINFO 1
#endif
ServiceBarrier: 
        pushRet
        qbeq    noInfDDRQ,       static.BarrierInfDDRQ, 0
        lsl     barrier.srcQReg, static.BarrierInfDDRQ, 4    // qReg = barrier queue * 16
        ldi     barrier.retQOffset, BARRIER_INF_RET_OFFSET
        call    DdrBarrier
noInfDDRQ:
        qbeq    noNetCPDDRQ,     static.BarrierNetCPDDRQ, 0
        lsl     barrier.srcQReg, static.BarrierNetCPDDRQ, 4  // qReg = barrier queue * 16
        ldi     barrier.retQOffset, BARRIER_NETCP_RET_OFFSET
        call    DdrBarrier
noNetCPDDRQ:
        qbeq    noInfMSMCQ,      static.BarrierInfMSMCQ, 0
        lsl     barrier.srcQReg, static.BarrierInfMSMCQ, 4   // qReg = barrier queue * 16
        ldi     barrier.retQOffset, BARRIER_INF_RET_OFFSET
        call    MsmcBarrier
noInfMSMCQ:
        qbeq    noNetCPMSMCQ,    static.BarrierNetCPMSMCQ, 0
        lsl     barrier.srcQReg, static.BarrierNetCPMSMCQ, 4 // qReg = barrier queue * 16
        ldi     barrier.retQOffset, BARRIER_NETCP_RET_OFFSET
        call    MsmcBarrier
noNetCPMSMCQ:
        popRet
        ret

// DDR only barrier
// R0 = Q * 16
// R1.B0 = return queue offset
DdrBarrier:
        mov     barrier.tripCount, BARRIER_MAX_TRIP
        add     barrier.srcQReg, barrier.srcQReg, 8 // read C+D
DdrBarrierLoop:
        lbco    barrier.cReg, cQBase, barrier.srcQReg, 8 // Pop packet (C+D)
        qbeq    DdrBarrierDone, barrier.dReg, 0     // Jump out if empty
        mov     barrier.readAddr, barrier.dReg
        and     barrier.readAddr.b0, barrier.readAddr.b0, 0xF0 // Mask off the hint (if any)

        add     barrier.readAddr, barrier.readAddr, barrier.retQOffset
        // return q from srd/dst tag or swinfo 1.  This serves as the actual barrier!
        lbbo    barrier.retQ, barrier.readAddr, 0, 2
        lsl     r0, barrier.retQ, 4                 // q * 16 = push addr
        add     r0, r0, 8                           // write C+D
        sbco    barrier.cReg, cQBase, r0, 8         // Push packet (C+D)
          
        sub     barrier.tripCount, barrier.tripCount, 1
        qbne    DdrBarrierLoop, barrier.tripCount, 0
        
DdrBarrierDone:
        ret

MsmcBarrier:
// go max up to items in queue, but no more than BARRIER_MAX_TRIP
        lsl     r0, static.BarrierInfMSMCQ, 4       // r0 = barrier queue * 16
        lbco    barrier.dReg, cQPeekBase, barrier.srcQReg, 4  // peek packets in queue
        qbeq    MsmcBarrierDone, barrier.dReg, 0    // no packets

// At least one packet in queue, do barrier
        ldi     barrier.readAddr, 0
        mov     barrier.readAddr.w2, static.BarrierDDRMSW
        lbbo    r0, barrier.readAddr, 0, 4          // Barrier for DDR
        mov     barrier.readAddr.w2, static.BarrierMSMCMSW
        mov     barrier.tripCount, 8                // 8 banks
MsmcBarrierBankLoop:
        lbbo    r0, barrier.readAddr, 0, 4          // Barrier for MSMC BANK #tripCount
        add     barrier.readAddr, barrier.readAddr, 0x80 // assumes msmc banks are 128 bytes apart 
        sub     barrier.tripCount, barrier.tripCount, 1
        qbne    MsmcBarrierBankLoop, barrier.tripCount, 0
        
// Forward at most BARRIER_MAX_TRIP
        qbge    MsmcBarrierNoClamp, barrier.dReg, BARRIER_MAX_TRIP  // MAX_TRIP >= dReg
        ldi     barrier.dReg, BARRIER_MAX_TRIP
MsmcBarrierNoClamp:
        mov     barrier.tripCount, barrier.dReg

        add     barrier.srcQReg, barrier.srcQReg, 8 // read C+D

MsmcBarrierLoop:
        lbco    barrier.cReg, cQBase, barrier.srcQReg, 8 // Pop packet (C+D)
        qbeq    MsmcBarrierDone, barrier.dReg, 0    // Jump out if empty
        mov     barrier.readAddr, barrier.dReg
        and     barrier.readAddr.b0, barrier.readAddr.b0, 0xF0 // Mask off the hint (if any)

        add     barrier.readAddr, barrier.readAddr, barrier.retQOffset
        // Read the descriptor src/dest tag. 
        lbbo    barrier.retQ, barrier.readAddr, 0, 2
        lsl     r0, barrier.retQ, 4                 // q * 16 = push addr
        add     r0, r0, 8                           // write C+D
        sbco    barrier.cReg, cQBase, r0, 8         // Push packet (C+D)
          
        sub     barrier.tripCount, barrier.tripCount, 1
        qbne    MsmcBarrierLoop, barrier.tripCount, 0
        
MsmcBarrierDone:
        ret
#endif
