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
// sriortr.p - srio router/switch
//
// Target: k2x any qmss pdsp
//
// Description:
//
// This code implements the PDSP srio router/switch firmware
//
// Copyright 2014-2015 by Texas Instruments Inc.
//----------------------------------------------------------------------------------
// Revision History:
//
// 24-Oct-14 : John Dowdal : Version 1.0.0.0 
// 13-Mar-15 : John Dowdal : Version 1.0.0.1 - add fence after modifying descriptor
// 20-Oct-15 : John Dowdal : Version 1.0.0.2 - enable K1
//
// Reminder: set version number in HEADER_VERSION below.
//==================================================================================

#include "pm_config.h"

#define SRIO_ROUTER

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
#define TIMER_SETTING       (100)

// ENDIAN DEPENDENT STRUCTURES
//
// Structures that are shared with the host must be defined to
// represent the same byte ordering in the host, regardless of
// endian mode. Thus there is one set for Little Endian and
// one set for Big Endian.
//

#ifdef _LE_
.struct struct_enables
    .u8     enabled         // Current enable/diable status
    .u8     res1
    .u16    res2
    .u8     enables         // Ports to enable/disable
    .u8     res3
    .u16    res4
.ends
#else
.struct struct_enables
    .u16    res2
    .u8     res1
    .u8     enabled         // Current enable/diable status
    .u16    res4
    .u8     res3
    .u8     enables         // Ports to enable/disable
.ends
#endif

#ifdef _LE_
.struct struct_gblconfig
    .u16    queueBase       // base queue for input queues
    .u16    res1            
    .u16    credits         // credits per credit message
    .u16    res2
.ends
#else
.struct struct_gblconfig
    .u16    res1            
    .u16    queueBase       // base queue for input queues
    .u16    res2
    .u16    credits         // credits per credit message
.ends
#endif

#ifdef _LE_
.struct struct_queueconfig
    .u16    queue           // queue number
    .u16    res1
.ends
#else
.struct struct_queueconfig
    .u16    res1
    .u16    queue           // queue number
.ends
#endif


//
// ENDIAN AGNOSTIC STRUCTURES
//
// Structures that are private to the PDSPs can be allowed to
// alter their byte ordering according to endian mode. This is
// done automatically by the assembler.
//
.struct struct_static
    .u32     enabledMask // enabledMask as 20 bit mask
    .u8      rrLastHost  // last host queue received (for round robin)
    .u8      rrLastPort  // last port queue received (for round robin)
    .u8      dontSleep   // loop did something, so iterate without sleeping
    .u8      creditAvail // bit set if port has credit
    .u8      creditNeed  // bit set if port can TX credit
    .u8      res1
    .u16     creditBase  // base address of credit structure
.ends

.struct struct_enables_scr
    .u8      changed 
    .u8      res1
    .u16     res2
.ends

.struct struct_credit_TX
    .u8      creditMask
    .u8      res1
    .u16     res2
.ends

.struct struct_credit_TX_scr
    .u16     res1
    .u16     scr16
    .u32     scr32
.ends

.struct struct_packet_RX
    .u32     qPendBits
.ends

.struct struct_packet_RX_scr
    .u32     scr32
    .u32     scr32b
    .u32     scr32c
    .u8      scr8
    .u8      readyQ
    .u16     scr16
.ends

.struct struct_packet_RX_srio_fwd
    .u8      srcPort
    .u8      dstPort
    .u8      fFinalDest
    .u8      res1
    .u16     res2
    .u16     destID
    .u16     srcQ
    .u16     dstQ
    .u32     descSize
    .u32     descPtr
    .u32     descPtrClean
.ends

.struct struct_forward
    .u16     srcQ
    .u16     dstAddr
    .u16     statsBase
    .u8      result
    .u8      port
.ends

.struct struct_forward_scr
    .u32     queAddr
    .u32     descSize
    .u32     descPtr
.ends

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
.assign struct_static,                        R2,  R4, static
.assign struct_gblconfig,                     r5,  r6, gblcfg

// Overlays
.enter    pEnables
.assign   struct_enables_scr,                 r27, r27, scratch
.assign   struct_enables,                     r28, r29, enables
.leave    pEnables

.enter    pCreditTX
.assign   struct_credit_TX,                   r20, r20, credTX
.assign   struct_credit_TX_scr,               r28, r29, scratch
.leave    pCreditTX

.enter    pPacketRX 
.assign   struct_packet_RX,                   r19, r19, pktRX
.assign   struct_packet_RX_scr,               r26, r29, scratch
.leave    pPacketRX

// used only with pPacketRX, but not pGenForward
.enter    pSrioForward
.assign   struct_packet_RX_srio_fwd,          r20, r25, srioFwd
.leave    pSrioForward

// used with pPacketRX and pCreditTX
.enter    pGenForward
.assign   struct_forward_scr,                 r21, r23, fwdScratch
.assign   struct_forward,                     r24, r25, forward
.leave    pGenForward


//-------------------------------------------------------------------
//
// Code Starts
//
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
#define MAGIC_BUILD 0x81000000
#ifdef _LE_
#define HEADER_MAGIC   MAGIC_BUILD                   // sriortr-0
#else
#define HEADER_MAGIC   (MAGIC_BUILD + 1)             // sriortr-1
#endif
#define HEADER_VERSION 0x01000001                    // 0x01.0x00.0x00.0x01
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
        ldi     r0, SRIORTR_OFF_VERSION
        sbco    r1, cLocalRAM, r0, 8

        // Zero persistent static variables
        zero    &static, SIZE(static)
        ldi     static.rrLastPort, 7
        ldi     static.rrLastHost, 3
        ldi     static.creditNeed, 0xf
        ldi     static.creditBase, SRIORTR_OFF_CREDITS

mainLoop:
        ldi     static.dontSleep, 0
.using pEnables
        // Check for port enables
        ldi     r0, SRIORTR_OFF_ENABLES
        lbco    enables, cLocalRAM, r0, SIZE(enables)
        qbeq    enablesNoChange, enables.enables, enables.enabled
        // Build new enable mask
        lsl     static.enabledMask, enables.enables, 4                  // enabledMask = 0000eeee0000b
        or      static.enabledMask, static.enabledMask, enables.enables // enabledMask = 0000eeeeEEEEb
        lsl     static.enabledMask, static.enabledMask, 4               // enabledMask = eeeeEEEE0000b
        or      static.enabledMask, static.enabledMask, enables.enables // enabledMask = eeeeEEEEeeeeb
        lsl     static.enabledMask, static.enabledMask, 4               
        or      static.enabledMask, static.enabledMask, enables.enables 
        lsl     static.enabledMask, static.enabledMask, 4               
        or      static.enabledMask, static.enabledMask, enables.enables 

        // Adjust enables
        mov     enables.enabled, enables.enables
        sbco    enables, cLocalRAM, r0, 4 // Only store enabled
enablesNoChange:
.leave pEnables
        // Re-load config in case somebody changed it
        ldi     r0, SRIORTR_OFF_CFG
        lbco    gblcfg, cLocalRAM, r0, SIZE(gblcfg)
.using pCreditTX
.using pGenForward

        // See which ports could be ready to TX credit
        and     credTX.creditMask, static.creditNeed, static.enabledMask
txCredLoop:
        lmbd    forward.port, credTX.creditMask, 1
        qbeq    txCredDone, forward.port, 32   // no bits set
        call    txCredPkt
        clr     credTX.creditMask, credTX.creditMask, forward.port // mark port as processed for this loop
        qbeq    txCredLoop, forward.result, 0 // send failed -- will try again next trip through main loop
        add     scratch.scr32, static.creditBase, forward.port
        add     scratch.scr32, scratch.scr32, forward.port
        add     scratch.scr32, scratch.scr32, 8
        lbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        add     scratch.scr16, scratch.scr16, gblcfg.credits
        sbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        clr     static.creditNeed, static.creditNeed, forward.port // successfully sent credit
        jmp     txCredLoop

txCredDone:
.leave pGenForward
.leave pCreditTX

.using pPacketRX
        // find qpend bit address
        lsr     scratch.scr32, gblcfg.queueBase, 5  // 32 bits per qpend set
        lsl     scratch.scr32, scratch.scr32, 2     // 4 bytes per qpend set
        lbco    pktRX.qPendBits, cQPending, scratch.scr32, 4
        and     scratch.scr8, gblcfg.queueBase, 0x1f
        lsr     pktRX.qPendBits, pktRX.qPendBits, scratch.scr8  // right justify
        and     pktRX.qPendBits, pktRX.qPendBits, static.enabledMask // mask out disabled ports & non monitored queues
        
rxPktLoop:
        lmbd    scratch.readyQ, pktRX.qPendBits, 1
        qbeq    rxNoPkts, scratch.readyQ, 32           // no bits are set
        ldi     static.dontSleep, 1                    // did some work, so don't sleep
        qbgt    rxNoCreditPackets, scratch.readyQ, 12  // queues 12-15 are credit queues  (high priority)
        

.using pGenForward
        clr     pktRX.qPendBits, pktRX.qPendBits, scratch.readyQ // acknowledge it 
        add     forward.srcQ, gblcfg.queueBase, scratch.readyQ
        sub     forward.port, scratch.readyQ, 12
        ldi     forward.dstAddr, SRIORTR_OFF_CFG_CREDRET
        ldi     forward.statsBase, SRIORTR_OFF_STATS_CRRX
        call    fwdPkt
        qbeq    rxPktLoop, forward.result, 0           // it failed
        // add the credits
        add     scratch.scr32, static.creditBase, forward.port
        add     scratch.scr32, scratch.scr32, forward.port
        lbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        add     scratch.scr16, scratch.scr16, gblcfg.credits
        sbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        set     static.creditAvail, static.creditAvail, forward.port

        jmp     rxPktLoop
.leave pGenForward        
rxNoCreditPackets:
        qbgt    rxNoFwdCmpPkts, scratch.readyQ, 8      // queues 8-11 are fwd completion queues (mid priority)
.using pGenForward
        sub     forward.port, scratch.readyQ, 8
        ldi     forward.dstAddr, SRIORTR_OFF_CFG_FWDRXFREE
        ldi     forward.statsBase, SRIORTR_OFF_STATS_FWDRETS
        clr     pktRX.qPendBits, pktRX.qPendBits, scratch.readyQ // acknowledge it 
        add     forward.srcQ, gblcfg.queueBase, scratch.readyQ
        call    fwdPkt
        qbeq    rxPktLoop, forward.result, 0           // it failed
        //      tx credit processing
        add     scratch.scr32, static.creditBase, forward.port
        add     scratch.scr32, scratch.scr32, forward.port
        add     scratch.scr32, scratch.scr32, 8
        lbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        sub     scratch.scr16, scratch.scr16, 1
        sbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        qbeq    rxFwdCreditNeed, scratch.scr16, 0  // == 0
        qbbs    rxFwdCreditNeed, scratch.scr16, 31 // > 0 (positive signed number)
rxFwdNoCreditNeed:
        jmp     rxPktLoop
rxFwdCreditNeed:
        set     static.creditNeed, static.creditNeed, forward.port
        jmp     rxPktLoop
.leave pGenForward
rxNoFwdCmpPkts:
        qbgt    rxNoFwdPkts, scratch.readyQ, 4         // queues 4-7 are forwarding queues (mid priority)
.using pSrioForward
rxFwdPktsRRLoop:
        // apply round robin
        add     static.rrLastPort, static.rrLastPort, 1
        qbne    rxFwdPktsNoWrap, static.rrLastPort, 8
        ldi     static.rrLastPort, 4
rxFwdPktsNoWrap:
        qbbc    rxFwdPktsRRLoop, pktRX.qPendBits, static.rrLastPort
        // clear pending bit 
        clr     pktRX.qPendBits, pktRX.qPendBits, static.rrLastPort
        sub     srioFwd.srcPort, static.rrLastPort, 4
        add     srioFwd.srcQ, gblcfg.queueBase, static.rrLastPort
        // get address of source queue C reg
        lsl     srioFwd.srcQ, srioFwd.srcQ, 4 // *16
        add     srioFwd.srcQ, srioFwd.srcQ, 8
        // pop it
        lbco    srioFwd.descSize, cQBase, srioFwd.srcQ, 8
        qbeq    rxPktLoop, srioFwd.descPtr, 0 // got NULL
        // prep desc ptr by dumping hint bits
        mov     srioFwd.descPtrClean, srioFwd.descPtr
        and     srioFwd.descPtrClean.b0, srioFwd.descPtrClean.b0, 0xf0
#ifdef SRIO_ROUTER
        // get buffer pointer
        add     scratch.scr32, srioFwd.descPtrClean, SRIORTR_OFF_DESC_BUFPTR
        lbbo    scratch.scr32, scratch.scr32, 0, 4
        // look up destination from payload LSB of first word
        lbbo    scratch.scr32, scratch.scr32, 0, 4
        mov     srioFwd.destID, scratch.scr32.b0
#else // SRIO_SWITCH
        // look up destination from psinfo LSB of first word
        add     scratch.scr32, srioFwd.descPtrClean, SRIORTR_OFF_DESC_PSINFO 
        lbbo    scratch.scr32, scratch.scr32, 0, 4
        mov     srioFwd.destID, scratch.scr32.b0
#endif
        // look up destination port from destination ID
        lsr     scratch.scr8, srioFwd.destID, 3
        lsl     scratch.scr8, scratch.scr8, 2
        ldi     scratch.scr16, SRIORTR_OFF_LOOKUP // > 1 byte so need LDI
        add     scratch.scr16, scratch.scr16, scratch.scr8
        lbco    scratch.scr32, cLocalRAM, scratch.scr16, 4
        and     scratch.scr8, srioFwd.destID, 0x7
        lsl     scratch.scr8, scratch.scr8, 2
        lsr     scratch.scr32, scratch.scr32, scratch.scr8
        and     srioFwd.dstPort, scratch.scr32, 0x3
        and     srioFwd.fFinalDest, scratch.scr32, 0x4

        // If next hop is final dest, dont do credit
        qbne    rxFwdPktHaveCredit, srioFwd.fFinalDest, 0
        // Is there credit to send to this port?
        qbbs    rxFwdPktHaveCredit, static.creditAvail, srioFwd.dstPort
        // no -- put back the packet on head
        set     srioFwd.descSize, srioFwd.descSize, 31
        sbco    srioFwd.descSize, cQBase, srioFwd.srcQ, 8
        jmp     rxPktLoop

rxFwdPktHaveCredit:
#ifdef SRIO_ROUTER
        // replace PSINFO destination

        // Get new destID
        ldi     scratch.scr16, SRIORTR_OFF_DESTID
        lsl     scratch.scr8, srioFwd.dstPort, 2
        add     scratch.scr16, scratch.scr16, scratch.scr8
        lbco    scratch.scr32b, cLocalRAM, scratch.scr16, 8
        mov     srioFwd.destID, scratch.scr32b
        add     scratch.scr32, srioFwd.descPtrClean, SRIORTR_OFF_DESC_PSINFO
#ifdef SRIO_ROUTER_PATCH_DESTID_ONLY

        // Replace destID
        lbbo    scratch.scr32b, scratch.scr32, 0, 4
        mov     scratch.scr32b.w0, srioFwd.destID
#else
        // Replace entire psinfo[0]
        lbbo    scratch.scr32c, scratch.scr32, 4, 4
        // [nothing needed]
#endif
        ldi     scratch.scr32c.w2, 0x1c
        // if next hop is final destination make LSB of mailbox 1, else
        // make it 0
        clr     scratch.scr32c, scratch.scr32c, 0
        qbeq    rxFwdPktStorePSInfo, srioFwd.fFinalDest, 0
        set     scratch.scr32c, scratch.scr32c, 0
rxFwdPktStorePSInfo:
        sbbo    scratch.scr32b, scratch.scr32, 0, 8
        lbbo    scratch.scr32b, scratch.scr32, 4, 4 // fence to ensure write happens before push
#endif

        // look up output queue
        lsl     scratch.scr16, srioFwd.srcPort, 4 // * 16
        lsl     scratch.scr8, srioFwd.dstPort, 2  // * 4
        add     scratch.scr16, scratch.scr16, scratch.scr8
        add     scratch.scr16, scratch.scr16, SRIORTR_OFF_CFG_OUTPUTS
        lbco    scratch.scr32, cLocalRAM, scratch.scr16, 4
        mov     srioFwd.dstQ, scratch.scr32
        // get address of dstQ queue C reg
        lsl     scratch.scr32, srioFwd.dstQ, 4 // *16
        add     scratch.scr32, scratch.scr32, 8
        // push it
        sbco    srioFwd.descSize, cQBase, scratch.scr32, 8
        // bump the stats
        lsl     scratch.scr16, srioFwd.dstPort, 2
        add     scratch.scr16, scratch.scr16, SRIORTR_OFF_STATS_PKTTX
        lbco    scratch.scr32, cLocalRAM, scratch.scr16, 4
        add     scratch.scr32, scratch.scr32, 1
        sbco    scratch.scr32, cLocalRAM, scratch.scr16, 4
        // bump stats
        lsl     scratch.scr16, srioFwd.srcPort, 2
        add     scratch.scr16, scratch.scr16, SRIORTR_OFF_STATS_PKTRX
        lbco    scratch.scr32, cLocalRAM, scratch.scr16, 4
        add     scratch.scr32, scratch.scr32, 1
        sbco    scratch.scr32, cLocalRAM, scratch.scr16, 4

        // if next hop final destination, skip creditAvail
        qbne    rxPktLoop, srioFwd.fFinalDest, 0
        add     scratch.scr32, static.creditBase, srioFwd.dstPort
        add     scratch.scr32, scratch.scr32, srioFwd.dstPort
        lbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        sub     scratch.scr16, scratch.scr16, 1
        sbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        qbne    rxFwdCreditAvail, scratch.scr16, 0  // == 0
        // cant go negative
        clr     static.creditAvail, static.creditAvail, srioFwd.dstPort
rxFwdCreditAvail:

        jmp     rxPktLoop
.leave pSrioForward
rxNoFwdPkts:                                           // queues 0-3 are host queues (lowest priority)

rxHostPacketsRRLoop:
        // apply round robin
        add     static.rrLastHost, static.rrLastHost, 1
        qbne    rxFwdHostNoWrap, static.rrLastHost, 4
        ldi     static.rrLastHost, 0
rxFwdHostNoWrap:
        qbbc    rxHostPacketsRRLoop, pktRX.qPendBits, static.rrLastHost
.using pGenForward
        // clear pending bit 
        clr     pktRX.qPendBits, pktRX.qPendBits, static.rrLastHost
        // Is there credit?
        qbbc    rxPktLoop, static.creditAvail, static.rrLastHost

        add     forward.srcQ, gblcfg.queueBase, static.rrLastHost
        mov     forward.port, static.rrLastHost
        ldi     forward.dstAddr, SRIORTR_OFF_CFG_OUTPUTS_HOST
        ldi     forward.statsBase, SRIORTR_OFF_STATS_HOSTRX
        call    fwdPkt
        qbeq    rxPktLoop, forward.result, 0 // it didn't work

        // dock credits
        add     scratch.scr32, static.creditBase, forward.port
        add     scratch.scr32, scratch.scr32, forward.port
        lbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        sub     scratch.scr16, scratch.scr16, 1
        sbco    scratch.scr16, cLocalRAM, scratch.scr32, 2
        qbne    rxPktLoop, scratch.scr16, 0  // == 0
        // cant go negative
        clr     static.creditAvail, static.creditAvail, forward.port
.leave pGenForward
        jmp     rxPktLoop
         
rxNoPkts:
        
.leave pPacketRX
        
        qbne    mainLoop, static.dontSleep, 0
        // sleep 2 timer ticks to avoid banging the QM
sleep1: qbbc    sleep1, r31.tStatus_Timer
        set     r31.tStatus_Timer
sleep2: qbbc    sleep2, r31.tStatus_Timer
        set     r31.tStatus_Timer

        jmp     mainLoop

.using pGenForward
// forward host packets and credit packets
fwdPkt:
        // Calculate queue C register
        lsl     fwdScratch.queAddr, forward.srcQ, 4
        add     fwdScratch.queAddr, fwdScratch.queAddr, 8
        // Pop the queue
        lbco    fwdScratch.descSize, cQBase, fwdScratch.queAddr, 8
        // Did we get something?
        qbne    fwdPopOK, fwdScratch.descPtr, 0
        ldi     forward.result, 0
        ret
fwdPopOK:
        // force hint bits to 48 bytes
        and     fwdScratch.descPtr.b0, fwdScratch.descPtr.b0, 0xf0
        or      fwdScratch.descPtr.b0, fwdScratch.descPtr.b0, 2
        // find dest queue config address
        lsl     fwdScratch.queAddr, forward.port, 2  // * 4
        add     fwdScratch.queAddr, fwdScratch.queAddr, forward.dstAddr
        lbco    fwdScratch.queAddr, cLocalRAM, fwdScratch.queAddr, 4  // load queue #
        // Calculate queue C register
        lsl     fwdScratch.queAddr, fwdScratch.queAddr, 4
        add     fwdScratch.queAddr, fwdScratch.queAddr, 8
        // Push the descriptor there
        sbco    fwdScratch.descSize, cQBase, fwdScratch.queAddr, 8
        ldi     forward.result, 1
        // Bump stats
        lsl     fwdScratch.queAddr, forward.port, 2
        add     fwdScratch.queAddr, fwdScratch.queAddr, forward.statsBase
        lbco    fwdScratch.descPtr, cLocalRAM, fwdScratch.queAddr, 4
        add     fwdScratch.descPtr, fwdScratch.descPtr, 1
        sbco    fwdScratch.descPtr, cLocalRAM, fwdScratch.queAddr, 4
        ret
.leave pGenForward

txCredPkt:
.using pCreditTX
.using pGenForward
        pushRet

        // find source queue config address and source queue #
        lsl     scratch.scr32, forward.port, 2
        add     scratch.scr32, scratch.scr32, SRIORTR_OFF_CFG_CREDSRC
        lbco    scratch.scr32, cLocalRAM, scratch.scr32, 4
        mov     forward.srcQ, scratch.scr32
        ldi     forward.statsBase, SRIORTR_OFF_STATS_CRTX
        ldi     forward.dstAddr, SRIORTR_OFF_CFG_OUTPUTS_CREDIT
        call    fwdPkt

        popRet
        ret
.leave pGenForward
.leave pCreditTX
      
