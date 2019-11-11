// ************************************************************************************************
// * FILE PURPOSE: Packet processing on PDSPs without LUTs
// ************************************************************************************************
// * FILE NAME: pam.p
// *
// * DESCRIPTION: PDPS without LUTs can do the following processing:
// *              checksum insertion
// *              crc insertion
// *              blind data insertion
// *              multi route forwarding
// *              basic forwarding
// *
// ************************************************************************************************
//
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
//  manufactured by or for TI (�TI Devices�).  No hardware patent is licensed hereunder.
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
//  THIS SOFTWARE IS PROVIDED BY TI AND TI�S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED 
//  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
//  AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI�S 
//  LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE 
//  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
//  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// 
//

#define PDSP_PAM  1

#include "pdsp_pa.h"
#include "pdsp_mem.h"
#include "pdsp_subs.h"
#include "pm_config.h"
#include "pdsp_ver.h"
#include "pm_constants.h"
#include "parsescope.h"

#define PASS_PROC_CMDSET
#define PASS_PROC_MULTI_ROUTE
#define PASS_PROC_USR_STATS
#define PASS_PROC_LUT2

#define HEADER_MAGIC   0xBABE0003                    // PASS modifier1
#define PASS_M1_VER    PASS_VERSION                  // 0x01.0x00.0x00.0x03

    .origin      0
    .entrypoint  f_mInit
    
f_mInit:
  jmp f_mStart
  
header:
  .codeword  HEADER_MAGIC
  .codeword  PASS_M1_VER

#ifdef PASS_GLOBAL_INIT
    
#include "meminit.p"

#endif

    .using  globalScopeM

f_mStart:
    // f_mLocalInit is no longer needed since eventflag is removed
    //call f_mLocalInit
    
    // Clear the mailbox 
    zero &r2, 12
    sbco r2, FIRMWARE_MBOX, 4, 12 

    // Write a non-zero value to mailbox slot 0. 
    mov   r2, 1
    sbco  r2, FIRMWARE_MBOX, 0, 4
    
	// Wait for the set command to take hold
    wbs  s_flags.info.tStatus_Command0

    // The host will clear the flag when it is ready for the PDSP to run
    wbc  s_flags.info.tStatus_Command0

#ifdef PASS_GLOBAL_INIT
//only PDSP0 should do global init    
    // Do common initialization if the host has requested it
    // The host requests a global init by writing a non-zero value into mailbox slot 1
    // (Before clearing mailbox slot 0)
    qbbc  l_mStart0,  s_flags.info.tStatus_Command1

        call f_commonInit
        mov  r2, 0
        sbco r2, FIRMWARE_MBOX, 4, 4      // Clear the mailbox
#endif        
        
l_mStart0:

    // Store the PDSP ID Information in LSB
    lbco  s_modCxt.flags,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  SIZE(s_modCxt.flags)
    and   s_modCxt.flags,  s_modCxt.flags, 1
    
    mov   s_modCxt.usrStatsPdsp, 0
    
    // Store the version number
    mov   r2.w0,   PASS_M1_VER & 0xFFFF
    mov   r2.w2,   PASS_M1_VER >> 16 
    sbco  r2,   PAMEM_CONST_PDSP_INFO,  OFFSET_VER,     4  
    
    // Zero out debug 
    zero &r0, 16  
    sbco  r0,   PAMEM_CONST_PDSP_INFO,  0x10,          16
    
    zero &s_statsFlags,  SIZE(s_statsFlags)
           

// *******************************************************************************
// * FUNCTION PURPOSE: The main processing loop
// *******************************************************************************
// * DESCRIPTION: Packet commands are processed
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:          |
// *   R19:          |
// *   R20:          |
// *   R21:          |
// *   R22:     
// *   R23:     
// *   R24:     
// *   R25:     
// *   R26:  Packet ID
// *   R27:  
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:              
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************



f_mainLoop:
fci_mainLoop7:   // For compatibility with classify1.p
   // Look for command (FIRMWARE_CMD_PKT_CTRL_CFG only) 
    qbbc l_mainLoop, s_flags.info.tStatus_Command3
        // Process Packet Control Configuration Update
	    lbco  r1, FIRMWARE_MBOX, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        // Clear the valid bit fields 
        not r1.b2, r1.b1
        
        // force clear valid bits to zero and retain valid bits positions
        and s_modCxt.flags, s_modCxt.flags, r1.b2
        
        // set the run time flags
        or  s_modCxt.flags, s_modCxt.flags, r1.b0
    
        // clear the command flag
        mov   r1, 0           
	    sbco  r1, FIRMWARE_MBOX, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4

l_mainLoop:

    qbeq  l_mainLoop0,  s_statsFlags,  0
        sbco  s_statsFlags.event,  cStatistics,       OFFSET_STATS_FLAGS,  4
        zero &s_statsFlags,        SIZE(s_statsFlags)

.using modifyScope

l_mainLoop0:
    //
    // User Statistics Request Polling Engine:
    //  - Poll the User Statistic Request FIFO from the strating PDSP
    //  - If pending entry exists, process the entry, record the next PDSP as starting PDSP and exit
    //  - Otherwise, poll the request FIFO for next PDSP in round-robin
    //  - Exit if no entry found in all four PDSPs 

    // Only process user_statistics FIFO at PDSP4 
    qbbs  l_mainLoop1,  s_modCxt.flags.t_isPdsp5
    
        // Process the user statistics FIFOs 
        mov     s_rxUsrStatsFifoCxt.cnt,      4
        mov     s_rxUsrStatsFifoCxt.pdsp,     s_modCxt.usrStatsPdsp
        
l_mainLoop0_1:
        qbeq    l_mainLoop0_2, s_rxUsrStatsFifoCxt.cnt, 0    
            lsl     s_rxUsrStatsFifoCxt.cbOffset, s_rxUsrStatsFifoCxt.pdsp,     8
            add     s_rxUsrStatsFifoCxt.offset,   s_rxUsrStatsFifoCxt.cbOffset, OFFSET_PDSP_USR_STATS_FIFO
            add     s_rxUsrStatsFifoCxt.cbOffset, s_rxUsrStatsFifoCxt.cbOffset, OFFSET_PDSP_USR_STATS_FIFO_CB
        
            lbco    s_rxFifoCb,   PAMEM_USR_STATS_BASE,  s_rxUsrStatsFifoCxt.cbOffset, SIZE(s_rxFifoCb)
        
            add     s_rxUsrStatsFifoCxt.pdsp,    s_rxUsrStatsFifoCxt.pdsp,     1
            and     s_rxUsrStatsFifoCxt.pdsp,    s_rxUsrStatsFifoCxt.pdsp,     PA_USR_STATS_START_PDSP_MASK
            sub     s_rxUsrStatsFifoCxt.cnt,     s_rxUsrStatsFifoCxt.cnt,      1
            qbeq    l_mainLoop0_1,  s_rxFifoCb.in,    s_rxFifoCb.out  // FIFO is empty
        
            // FIFO is not empty
            add     r1.w0,  s_rxFifoCb.out,  s_rxUsrStatsFifoCxt.offset
            lbco    s_rxUsrStatsReq,  PAMEM_USR_STATS_BASE, r1.w0,    SIZE(s_rxUsrStatsReq)
            add     s_rxFifoCb.out, s_rxFifoCb.out, 4
            and     s_rxFifoCb.out, s_rxFifoCb.out, 0x1F
            sbco    s_rxFifoCb.out, PAMEM_USR_STATS_BASE,  s_rxUsrStatsFifoCxt.cbOffset, SIZE(s_rxFifoCb.out)  
            call    f_usrStatsUpdate
        
l_mainLoop0_2:
            // Record the start PDSP for round-robbin polling operation  
            mov     s_modCxt.usrStatsPdsp, s_rxUsrStatsFifoCxt.pdsp 
            
            // Pass through
        // Process the Tx user statistics FIFOs (from PDSP5)
        lbco    s_rxFifoCb,   PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB, SIZE(s_rxFifoCb)
        qbeq    l_mainLoop1,  s_rxFifoCb.in,    s_rxFifoCb.out  // FIFO is empty
        
            // FIFO is not empty
            add     r1.w0,  s_rxFifoCb.out,  OFFSET_PDSP_TX_USR_STATS_FIFO
            lbco    s_rxUsrStatsReq,  PAMEM_CONST_TX_USR_STATS_FIFO, r1.w0,    SIZE(s_rxUsrStatsReq)
            add     s_rxFifoCb.out, s_rxFifoCb.out, 4
            and     s_rxFifoCb.out, s_rxFifoCb.out, 0x1F
            sbco    s_rxFifoCb.out, PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB, SIZE(s_rxFifoCb.out)  
            call    f_usrStatsUpdate
            
            // pass through
    
.leave modifyScope

l_mainLoop1:
    qbbc  f_mainLoop,  s_flags.info.tStatus_CDENewPacket

    // Fall through to f_mProc


// ************************************************************************************
// * FUNCTION PURPOSE: Process a packet
// ************************************************************************************
// * DESCRIPTION: A packet is examined for command info and processed.
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:       |  Packet ID
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:    | 
// *   R23:    | 
// *   R24:    |  Packet context        
// *   R25:    |        
// *   R26:    |
// *   R27:    |
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using modifyScope
    .using currentFwdScope

f_mProc:

    // packet counter
    // this count gets updates even when the capture/mirror is set
	lbco  r1, FIRMWARE_MBOX, 0, 4
	add   r1, r1, 1
	sbco  r1, FIRMWARE_MBOX, 0, 4

    zero  &s_modState, SIZE(s_modState)
    mov   r24,        0
    mov   s_modCxt.paddingCnt,  0
    mov   s_modCxt.cmdFlushBytes, 0

    // Debug code to duplicate the packet: The user can set up circular queue to record the last N entries
    // Enable by writing configuration parameters to PDSP Info location + 0x10
    // b3 = enable/disable b2 = flow Id w0 = queue number 
    lbco  r0,   PAMEM_CONST_PDSP_INFO,  0x10,     4
    qbeq  l_mProc00,  r0.b3,   0  
    
    zero &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE
    mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_COPY
    mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
    mov  s_cdeCmdPkt.destQueue,   r0.w0
    mov  s_cdeCmdPkt.flowId,      r0.b2
    xout XID_CDECTRL,             s_cdeCmdPkt,       SIZE(s_cdeCmdPkt)
  
    // Wait for new packet to be ready
    wbs   s_flags.info.tStatus_CDENewPacket
    
l_mProc00:  
    xin   XID_CDEDATA,  s_pktDescr,  SIZE(s_pktDescr)

    // Record the ctrl size associated with the packet
    mov   s_modState3.remCmdSize, s_pktDescr.ctrlDataSize
    
    // Record packet size
    mov   s_modState.pktSize,   s_pktDescr.pktDataSize          

#ifdef TO_BE_DELETE
    // Assign a packet ID if this is the first packet seen by the PDSPs
    qbbs  l_mProc2a,  s_pktDescr.pktId.t_pktIdAllocated

l_mProc2:

    // Have a new packet ID. Set the alloc bit
    set  s_pktDescr.pktId,   r18.w0.t_pktIdAllocated
    xout XID_CDEDATA,        s_pktDescr,               SIZE(s_pktDescr)
#endif

l_mProc2:

    // There must be a command in the control section for this PDSP
    qbne  fci_mProc3, s_pktDescr.ctrlDataSize, 0

#ifdef TO_BE_DELETE
        // Load the error routing info for this error type
        mov  r30.w2,       EROUTE_SYSTEM_FAIL << 4  // multiply by 16, the size of struct_paFwdPlace
        lbco s_curFmPlace, PAMEM_CONST_EROUTE,      r30.w2,   SIZE(s_curFmPlace)

        // r3 holds the stats bits to set
        zero &r3,                 4
        set   r3.t_nCommandFail

        mov   r30.w0,  f_mainLoop  // return address
        jmp   f_curPktForward
#else
        // just drop this illegal packet
        set s_statsFlags.event.t_nCommandFail
        
        // Discard the packet
        zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
        mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
        xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)
        jmp   f_mainLoop
#endif        
        


fci_mProc3:

    // Advance to the control section
    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout XID_CDECTRL,            s_cdeCmdWd,        4

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    //mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    //mov s_cdeInsert.byteCount,  32
    ldi  r4,    CDE_CMD_INSERT_PSDATA | (32 << 8)     
    xout XID_CDECTRL,           s_cdeCmdWd,         4

fci_mProc4:

    // Read in the command header
    // The size of a complete packet context is read in, since this will
    // used for the multi route and command set case.
    xin  XID_CDEDATA,  s_msg,   SIZE(s_pktCxt)

    // Extract the command ID and process
    lsr  r1.b0,  s_msg.cmdId,  PA_MSG_CMD_ID_SHIFT

    // Search based on expected occurance of header type
    qbeq  f_insChksum,        r1.b0,  PSH_CMD_PA_TX_CHKSUM
    qbeq  f_nextRoute,        r1.b0,  PSH_CMD_PA_TX_NEXT_ROUTE
    qbeq  f_blindPatch,       r1.b0,  PSH_CMD_PA_TX_BLIND_PATCH
    qbeq  f_insCrc,           r1.b0,  PSH_CMD_PA_TX_CRC
    qbeq  l_jmp_f_paSetupRxFwd,     r1.b0,  PSH_CMD_RX_FWD
// Fix for conditional jump out of range
    jmp   l_no_jmp_f_paSetupRxFwd
l_jmp_f_paSetupRxFwd:
    jmp   f_paSetupRxFwd
l_no_jmp_f_paSetupRxFwd:

    qbeq  l_jmp_f_paSetupConfigure, r1.b0,  PSH_CMD_CONFIG
// Fix for conditional jump out of range
    jmp   l_no_jmp_f_paSetupConfigure
l_jmp_f_paSetupConfigure:
    jmp   f_paSetupConfigure
l_no_jmp_f_paSetupConfigure:
    qbeq  f_rptTimestamp,     r1.b0,  PSH_CMD_PA_TX_RPT_TS
    qbeq  f_group7Cmd,        r1.b0,  PSH_CMD_PA_GROUP_7

fci_mProc5:
//   set   s_statsFlags.event.t_nInvalidControl
     
    // Discard the packet
    zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)

        jmp   f_mainLoop

    .leave currentFwdScope
    .leave cdeScope
    .leave pktScope
    .leave modifyScope

// ********************************************************************************************
// * FUNCTION PURPOSE: Tx Cmd Error handling
// ********************************************************************************************
// * DESCRIPTION: Error handling during a tx cmd error
// *
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    
// *   R2:    
// *   R3:       |  bit mask for stat to update on silent discard
// *   R4:    
// *   R5:    
// *   R6:          |
// *   R7:          |  
// *   R8:          |
// *   R9:          |
// *   R10:     
// *   R11:     
// *   R12:     
// *   R13:
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  
// *   R18:          |                                  
// *   R19:
// *   R20:
// *   R21:          
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c2RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ****************************************************************************************************

    .using  cdeScope
f_txCmdErrHandle:
    set s_statsFlags.event.t_nCommandFail
    // Discard the packet
    zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)

    jmp f_mainLoop
    
    .leave  cdeScope

// *******************************************************************************
// * FUNCTION PURPOSE: handle a checksum calculate request
// *******************************************************************************
// * DESCRIPTION: The checksum command is stored. It will be activated when
// *              the packet is scrolled to the packet data section
// *
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:       |  Packet ID
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

    .using  cdeScope
    .using  modifyScope
   
f_insChksum:
    xin  XID_CDEDATA,  s_cmdChkCrc,  SIZE(s_cmdChkCrc)

    // Delete the command from the control info
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount,  SIZE(s_cmdChkCrc)

    // Check if we still have any commands left
    qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount  
    
    xout XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)

    // Only two checksums can be computed at one time
    qble  l_insChksum0,  s_modState.chkSumCount,  2

    // Store the checksum command. It will be executed when
    // the CDE advances to the start of the packet
    mov  r2,            OFFSET_INSERT_CHKSUM
    qbeq l_insChksumX,  s_modState.chkSumCount,  0
         add    r2,       r2,          SIZE(s_cmdChkCrc)

l_insChksumX:
    sbco s_cmdChkCrc,  PAMEM_CONST_MODIFY,     r2,                   SIZE(s_cmdChkCrc)

    add  s_modState.chkSumCount,  s_modState.chkSumCount,  1

    qbbc  fci_mProc4,  s_modState.flags.t_subsMCxtTerminate 

        // The terminate bit was set. Jump to the terminate address
        //jmp  s_modState.termAddress
        jmp fci_nextRoute0

l_insChksum0:

    // Number of checksum commands exceeds the maximum
    set s_statsFlags.event.t_nCommandFail
    // Discard the packet
    zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)

    jmp f_mainLoop


    .leave cdeScope
    .leave modifyScope


// *****************************************************************************
// * FUNCTION PURPOSE: Insert CRC
// *****************************************************************************
// * DESCRIPTION: The compute and insert CRC command is stored. It will be
// *              computed when the CDE is advanced to the packet data
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:       |  Packet ID
// *   R19:          |
// *   R20:       |   Modify State machine (s_modState)
// *   R21:       |
// *   R22:     
// *   R23:     
// *   R24:            
// *   R25:            
// *   R26:  Packet ID
// *   R27: 
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// **********************************************************************************

    .using  cdeScope
    .using  modifyScope

f_insCrc:

    xin  XID_CDEDATA,  s_cmdChkCrc,  SIZE(s_cmdChkCrc)

    // Delete the command from the control info
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount,  SIZE(s_cmdChkCrc)

    qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount   
    
    xout XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)

    // Only one CRC command can be stored
    qbbs  l_insCrc0,  s_modState.flags.t_subsMCxtCrc
    
    // Store the crc command. It will be executed when
    // the CDE advances to the start of the packet
    sbco s_cmdChkCrc,  PAMEM_CONST_MODIFY,     OFFSET_INSERT_CRC,  SIZE(s_cmdChkCrc)
    set  s_modState.flags.t_subsMCxtCrc

    qbbc  fci_mProc4,  s_modState.flags.t_subsMCxtTerminate 

        // The terminate bit was set. Jump to the terminate address
        //jmp  s_modState.termAddress
        jmp fci_nextRoute0

l_insCrc0:

    // Two CRC commands 
    set s_statsFlags.event.t_nCommandFail
    // Discard the packet
    zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)

    jmp f_mainLoop


    .leave cdeScope
    .leave modifyScope


// ******************************************************************************
// * FUNCTION PURPOSE: Perform a blind patch
// ******************************************************************************
// * DESCRIPTION: The blind patch command is stored. The patches will be done
// *              when the CDE is advanced to the desired point in the packet
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:       | Packet Id
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:          
// *   R23:          
// *   R24:          
// *   R25:          
// *   R26:  Packet ID
// *   R27: 
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// **********************************************************************************

    .using cdeScope
    .using modifyScope

f_blindPatch:
    xin  XID_CDEDATA,  s_blindPatch,  SIZE(s_blindPatch)
    
    // Advance the CDE, flushing the command
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_FLUSH
    lsr  s_cdeCmdWd.byteCount,   s_blindPatch.cmdLen_insert, 4   // dont count the insert flag
    lsl  s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount,       2   // convert 32 bit word count to bytes

    qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    
    xout XID_CDECTRL,            s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    add  s_modCxt.cmdFlushBytes, s_modCxt.cmdFlushBytes, s_cdeCmdWd.byteCount

    qble  l_blindPatch0,  s_modState.patchCount, SUBS_MOD_CXT_MAX_PATCHES

    // Copy the entire command, including pad, to memory
    // The command length is in 4 byte units, from bits 7:4. Strip 
    // bit 3:0 and convert to bytes
    lsr  r0.b0,  s_blindPatch.cmdLen_insert, 4
    lsl  r0.b0,  r0.b0,                      2

    lsl  r3.w2,  s_modState.patchCount,    5                      // Offset to memory (32 bytes each entry)
    add  r3.w2,  r3.w2,                  OFFSET_BLIND_PATCH     // Base offset

    sbco  s_blindPatch,  PAMEM_CONST_MODIFY,  r3.w2,   b0

    // keep count of the number of patch commands are stored
    add s_modState.patchCount, s_modState.patchCount, 1

    qbbc fci_mProc4, s_modState.flags.t_subsMCxtTerminate

    // The terminate bit was set. Complete all processing
    //jmp s_modState.termAddress
    jmp fci_nextRoute0
    

l_blindPatch0:

    // Too many patch commands
    set s_statsFlags.event.t_nCommandFail

l_blindPatch1:
    // Discard the packet
    zero  &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov    s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout   XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)

    jmp f_mainLoop


    .leave cdeScope
    .leave modifyScope
    
// *******************************************************************************
// * FUNCTION PURPOSE: Insert timestamp
// *******************************************************************************
// * DESCRIPTION: Insert Timestamp to the packet descriptor 
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                            
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          |  
// *   R18:          |  
// *   R19:            |  next route stub (s_nRouteStub)
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     
// *   R23:       
// *   R24:            
// *   R25:            
// *   R26:  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    .using cdeScope
    .using pktScope     // For offset info
    .using modifyScope

f_rptTimestamp:

    xin  XID_CDEDATA,  s_reportTs,  SIZE(s_reportTs)
    
    // Attempt to check if the command is valid
    and  r0.b0,  s_reportTs.cmdId, PA_RPT_TIME_STAMP_NOT_CMD_MASK
    qbne  f_txCmdErrHandle, r0.b0, 0    

    // Store the timestamp command. It will be executed when
    // the CDE forward the packets out
    sbco s_reportTs,  PAMEM_CONST_MODIFY,     OFFSET_REPORT_TIMESTAMP,  SIZE(s_reportTs)
    set  s_modState.flags.t_subsMCxtTs

    // Advance past the command
    mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount, SIZE(s_reportTs)

    qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount 
    
    xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)

    jmp  fci_mProc4
    
    .leave cdeScope
    .leave pktScope 
    .leave modifyScope
    
// *******************************************************************************
// * FUNCTION PURPOSE: Forward a packet
// *******************************************************************************
// * DESCRIPTION: The next route command is used to forward a packet. 
// *              If the next command process bit (N bit) is set then
// *              the command is stored and processed after the next
// *              command in the list is processed.
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  | can be re-used user Stats update context             | EQoS parsing header
// *   R7:        |  | at packet forwarding                                 |
// *   R8:        |  |                                                      |
// *   R9:        |  |                                                        | s_paComIfEQoS
// *   R10:       |    | Can be re-used for usrStats Global configuration     |
// *   R11:       |  (timestamp) [for packet descriptor updated] (report timestamp only) 
// *   R12:       |  (swInfo0)   [IP fragmentation only]
// *   R13:       |  (swInfo1)
// *   R14:          |                                                      | EQoS etherType
// *   R15:          | | reused as user_stats request                       |                    
// *   R16:          | |                                                    |
// *   R17:          |                                                      |
// *   R18:       | Packet ID                                                 | s_eqosQueueFlow
// *   R19:             |  next route stub (s_nRouteStub)
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     
// *   R23:       
// *   R24:            
// *   R25:            
// *   R26:  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    .using cdeScope
    .using modifyScope
    .using pktScope     // For offset info
    .using pktCaptureScope

f_nextRoute:

    xin  XID_CDEDATA,  s_nextRoute,   SIZE(s_nextRoute)
    
    // The software info words can be patched even if the next command
    // must be processed
    mov  s_cdeCmdWd.byteCount, SIZE(s_nextRoute) - 4
    // make sure that the output buffer is raedy for patch
    wbs   s_flags.info.tStatus_CDEOutPacket
    
    qbbc  f_nextRoute_dest_cdma, s_nextRoute.cmdId_N_E_Dest.t_nextRouteE
        // Increase the byteCount for extension parameter
        add  s_cdeCmdWd.byteCount, s_cdeCmdWd.byteCount,    4
        // record L2 padding flag
        qbbc l_nextRoute_1_0, s_nextRoute.ctrlFlags.t_nextRoute_ctrl_l2padding
            set s_modState.flags.t_subsMCxtPadding
            
l_nextRoute_1_0:
        qbbc l_nextRoute_1, s_nextRoute.ctrlFlags.t_nextRoute_ctrl_txUsrStats
            set s_modState.flags.t_subsMCxtUsrStats
            mov s_modState.statsIndex, s_nextRoute.statsIndex
        
l_nextRoute_1:        
        and  r2.b2, s_nextRoute.cmdId_N_E_Dest,  PA_NEXT_ROUTE_DEST_MASK
        qbne f_nextRoute_dest_eth_cdma, r2.b2,   PA_DEST_SRIO
    
        //SRIO operation
        //Restore the destination field  
        and s_nextRoute.cmdId_N_E_Dest,  s_nextRoute.cmdId_N_E_Dest, NOT_PA_NEXT_ROUTE_DEST_MASK
        or  s_nextRoute.cmdId_N_E_Dest, s_nextRoute.cmdId_N_E_Dest, PA_DEST_CDMA
        
        // pactch the packet type
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_TYPE_MASK
        lsl  r2.b1, s_nextRoute.pktType_psFlags, PA_PKT_TYPE_SHIFT
        or   r2.b0, r2.b0,  r2.b1
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        
        // store modified pktType for IP fragmentation
        mov  s_modState.pktType_psFlags,    r2.b0
        
        // Patch the PS Info
        sbco s_nextRoute.swInfo0, cCdeOutPkt,  SIZE(s_pktDescr),  8
        
        mov  s_cdeCmdWd.byteCount, SIZE(s_nextRoute)
        
        set  s_modState.flags.t_subsMCxtSrio
        
        jmp  f_nextRoute_dest_common
        
f_nextRoute_dest_eth_cdma: 
        //ETH, CDMA operations
        //patch the psFlags
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0, s_nextRoute.pktType_psFlags
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
        // store modified psFlags for IP fragmentation
        mov  s_modState.pktType_psFlags,    r2.b0
        
        set  s_modState.flags.t_subsMCxtPsFlags
        
        qbeq f_nextRoute_dest_common, r2.b2,  PA_DEST_ETH 
    
f_nextRoute_dest_cdma:    
    sbco s_nextRoute.swInfo0, cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0),  8

f_nextRoute_dest_common:
    // Store the route info stub. This allows the same structure to be
    // used regardless of the next command state
    // this is really mvid s_routeStub, *&s_nextRoute
    mvid  r19, *&s_nextRoute
    
    // store softeware Info for IP fragmentation
    mov  s_pktDescr.swinfo0,    s_nextRoute.swInfo0
    mov  s_pktDescr.swinfo1,    s_nextRoute.swInfo1

    // Advance past the command
    mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_FLUSH
    
    qbbs  l_nextRoute_txChkSkip, s_modCxt.flags.t_MCxtCopyState    
      qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
      sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount  
      
l_nextRoute_txChkSkip:    

    xout XID_CDECTRL,          s_cdeCmdWd,             4
    add  s_modCxt.cmdFlushBytes, s_modCxt.cmdFlushBytes, s_cdeCmdWd.byteCount
    qbbc  fci_nextRoute0,  s_nextRoute.cmdId_N_E_Dest.t_nextRouteN

    // The next command bit is set. Setup the mod state to handle the termination
    set  s_modState.flags.t_subsMCxtTerminate
    //mov  s_modState.termAddress,  fci_nextRoute0 
    //mov  s_modState.termAddress.w2,  fci_nextRoute0 >> 16
    jmp  fci_mProc4

fci_nextRoute0:
    // Verify whether EQoS operation is enabled and required
    // r2.w0: current packet offset
    mov r2.w0, 0
    qbbc l_nextRoute0, s_modCxt.flags.t_eqos_feature
       qbbs l_nextRoute0, s_modState.flags.t_subsMCxtTs
        and  r2.b2, s_nRouteStub.cmdId_N_E_Dest,  PA_NEXT_ROUTE_DEST_MASK
        qbne l_nextRoute0, r2.b2,  PA_DEST_ETH
        and  r2.b2, s_modState.pktType_psFlags, 0x70
        qbeq l_nextRoute0, r2.b2,  0
            call f_EQoS_PraseL2 

l_nextRoute0:
    // skip f_modComplete if ipFrag is required
    // The IP fragmenation can only be combined with the blind patch command to insert the IPSEC
    // AH authentication, all other modify commands will be ignored 
    qbbs fci_ipFrag,    s_modState.flags.t_subsMCxtIpFrag  // no return
    call f_modComplete

fci_nextRoute_skipModComplete:

    // Tx padding verification
    qbbc    l_nextRoute_copy, s_modState.flags.t_subsMCxtPadding
    qble    l_nextRoute_copy, s_modState.pktSize, 60

        // Move to the end of the packet
        mov     s_cdeCmd.v0.w0, CDE_CMD_ADVANCE_TO_END
        xout    XID_CDECTRL, s_cdeCmd, 4              // Send the command         

        // Insert packet data
        mov     s_cdeCmd.v0.w0, CDE_CMD_INSERT_PACKET
        rsb     s_cdeCmd.v0.b1, s_modState.pktSize, 60
        xout    XID_CDECTRL, s_cdeCmd, 4              // Send the command   
        
        //Update padding cnt
        add     s_modCxt.paddingCnt,    s_modCxt.paddingCnt,    1 
        mov     s_modState.pktSize,     60

l_nextRoute_copy:

        // Check whether copy is already done or not 
        qbbs  l_nextRoute_capture_done,   s_modCxt.flags.t_MCxtCopyState        

        // Check if IP fragment command is executed, if yes jump to normal operations.
        qbbs  l_nextRoute_txStats_update, s_modState.flags.t_subsMCxtIpFrag      

        // Check if mod context has PsFlags set, if not jump to normal operations.
        qbbc  l_nextRoute_txStats_update, s_modState.flags.t_subsMCxtPsFlags        

        // Check whether global packet capture for egress is set for copy, else normal-op
        qbbc  l_nextRoute_txStats_update, s_modCxt.flags.t_egress_pCapEnable
        
        // ETH destination, get the ethernet port destined
        // Shift right by 1 bit to get the correct configuration size per port
        and  r1.b0, s_modState.pktType_psFlags, PA_ETH_PS_FLAGS_PORT_MASK
        lsr  r1.w0, r1.b0, 1
        // Update r1 with offset for interface
        add   r1.w0, r1.w0, OFFSET_EGRESS_PKT_CAP_CFG_BASE
    
        // load the configurations 
        lbco  s_paPktCapScr, PAMEM_CONST_PORTCFG, r1.w0, SIZE(s_paPktCapScr) 
    
        // Check the enable bit for ethernet capture
        qbbc  l_nextRoute_txStats_update, s_paPktCapScr.ctrlBitMap.t_pkt_cap_enable

        // Copy is needed, set the bit in the modify context
        set  s_modCxt.flags.t_MCxtCopyState        

       // Issue Copy command (either to Host or Destination)
       // Check is the copy to be done to host?
       zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)    
       qbbs  l_nextRoute_eth_dest_copy_host, s_paPktCapScr.ctrlBitMap.t_pkt_cap_host    
       // Port mirror activity
       // Routing to ETH
       // Patch the psflags
         lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
         and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
         or   r2.b0, r2.b0,  s_paPktCapScr.mport_flow
         sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
         // Send the packet to ethernet mirror port 
         mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
         // Jump to send packet and normal operation 
         jmp  l_nextRoute_eth_dest_copy_send
       
l_nextRoute_eth_dest_copy_host:
l_nextRoute_eth_dest_copy_host_queue_bounce:
        // Check for Queue Bounce operation
l_nextRoute_eth_dest_copy_host_queue_bounce_ddr:
        qbbc l_nextRoute_eth_dest_copy_host_queue_bounce_msmc, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_nextRoute_eth_dest_copy_host_queue_bounce_end

l_nextRoute_eth_dest_copy_host_queue_bounce_msmc:
        qbbc l_nextRoute_eth_dest_copy_host_queue_bounce_end, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through
l_nextRoute_eth_dest_copy_host_queue_bounce_end:

         // Packet Capture activity
         mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
         mov  s_cdeCmdPkt.destQueue,    s_paPktCapScr.destQueue
         mov  s_cdeCmdPkt.flowId,       s_paPktCapScr.mport_flow 
         sbco  s_paPktCapScr.context,  cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0), SIZE(s_paPktCapScr.context)         
   
l_nextRoute_eth_dest_copy_send:
         mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY      
         mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE) 
         mov  s_cdeCmdPkt.psInfoSize,   0
         xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)

        // Wait for new packet to be ready
        // wbs   s_flags.info.tStatus_CDENewPacket
   
         jmp   f_mainLoop

l_nextRoute_capture_done:
        // clear copy done state and continue regular operations
        clr   s_modCxt.flags.t_MCxtCopyState
        
l_nextRoute_txStats_update:
        qbbc  l_nextRoute_padding_cnt_updtae,  s_modState.flags.t_subsMCxtUsrStats
            // prepare and update user-stats
            mov  s_rxUsrStatsReq.index,     s_modState.statsIndex
            mov  s_rxUsrStatsReq.pktSize,   s_modState.pktSize
              
            // Only update  user-defined statistics  at PDSP4 
            qbbc  l_nextRoute_txStats_update_1,  s_modCxt.flags.t_isPdsp5
                // Call the common user-statistic update routine
                call  f_usrStatsUpdate
                jmp   l_nextRoute_padding_cnt_updtae
        
l_nextRoute_txStats_update_1: 
            // Just push the statistics update request into PDSP5 FIFO to be processed by PDSP4
            lbco  s_rxFifoCb,   PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB, SIZE(s_rxFifoCb)
            add   r0.b0,    s_rxFifoCb.in, 4
            and   r0.b0,    r0.b0,       0x1F
       
            qbne  l_nextRoute_txStats_update_2, s_rxFifoCb.out, r0.b0
                // FIFO is full, bump the system error
                set s_statsFlags.event.t_nSystemFail
                jmp l_nextRoute_padding_cnt_updtae

l_nextRoute_txStats_update_2:
            // Insert the request into the FIFO   
            add   r0.w2,   s_rxFifoCb.in, OFFSET_PDSP_TX_USR_STATS_FIFO
            sbco  s_rxUsrStatsReq,  PAMEM_CONST_TX_USR_STATS_FIFO, r0.w2, SIZE(s_rxUsrStatsReq)
            sbco  r0.b0,   PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB + OFFSET(s_rxFifoCb.in), SIZE(s_rxFifoCb.in)     
        
            // pass through
        
l_nextRoute_padding_cnt_updtae:
        qbeq  l_nextRoute_fwdPkt,  s_modCxt.paddingCnt, 0  
                 
            // 
            // All the user-statistics update registers are free at this moment
            // Prepare and request tx padding statistics update
            //
            // store the user statistics info 
            lbco  s_rxUsrStatsReq.index,     PAMEM_CONST_CUSTOM,  OFFSET_MAC_PADDING_CFG + OFFSET(struct_paMacPaddingCfg.txPaddingCntIndex),  2
            mov   s_rxUsrStatsReq.pktSize,   s_modCxt.paddingCnt  // used padding count as packet size
        
            // Make sure that the tx padding counter is a (pseudo) byte counter
            lsl   r0.w0,    s_rxUsrStatsReq.index,  1
        
            lbco  r0.w2,    PAMEM_CONST_USR_STATS_CB, r0.w0, 2
            qbbs  l_nextRoute_padding_cnt_updtae_1,    r0.w2.t_pa_usr_stats_cb_byte_cnt
                set r0.w2.t_pa_usr_stats_cb_byte_cnt
                sbco  r0.w2,    PAMEM_CONST_USR_STATS_CB, r0.w0, 2
            
l_nextRoute_padding_cnt_updtae_1:
            // Only update  user-defined statistics  at PDSP4 
            qbbs  l_nextRoute_padding_cnt_updtae_2,  s_modCxt.flags.t_isPdsp5
                // Call the common user-statistic update routine
                call  f_usrStatsUpdate
                jmp   l_nextRoute_fwdPkt
        
l_nextRoute_padding_cnt_updtae_2: 
            // Just push the statistics update request into PDSP5 FIFO to be processed by PDSP4
            lbco  s_rxFifoCb,   PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB, SIZE(s_rxFifoCb)
       
            add   r0.b0,    s_rxFifoCb.in, 4
            and   r0.b0,    r0.b0,       0x1F
       
            qbne  l_nextRoute_padding_cnt_updtae_3, s_rxFifoCb.out, r0.b0
                // FIFO is full, bump the system error
                set s_statsFlags.event.t_nSystemFail
                jmp l_nextRoute_fwdPkt

l_nextRoute_padding_cnt_updtae_3:
            // Insert the request into the FIFO   
            add   r0.w2,   s_rxFifoCb.in, OFFSET_PDSP_TX_USR_STATS_FIFO
            sbco  s_rxUsrStatsReq,  PAMEM_CONST_TX_USR_STATS_FIFO, r0.w2, SIZE(s_rxUsrStatsReq)
            sbco  r0.b0,   PAMEM_CONST_TX_USR_STATS_FIFO,  OFFSET_PDSP_TX_USR_STATS_FIFO_CB + OFFSET(s_rxFifoCb.in), SIZE(s_rxFifoCb.in)     
            
            // pass through
        //sub     s_modCxt.paddingCnt,    s_modCxt.paddingCnt,    1 
        //jmp     l_nextRoute_paddingLoop

        // pass through
l_nextRoute_fwdPkt:

    // Forward the packet
    zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    and  s_cdeCmdPkt.threadId,     s_nRouteStub.cmdId_N_E_Dest,  PA_NEXT_ROUTE_DEST_MASK
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.destQueue,    s_nRouteStub.destQueue
    mov  s_cdeCmdPkt.flowId,       s_nRouteStub.flowId
    qbbc f_nextRoute0_1,  s_modState.flags.t_subsMCxtSrio
        // SRIO use 8 byte PSINFO
        mov  s_cdeCmdPkt.psInfoSize,   8
        
        // pass through
f_nextRoute0_1:
    
    // Should we report timestamp
    qbbc f_nextRoute0_2,    s_modState.flags.t_subsMCxtTs 
    
        // Load the timestamp command
        lbco s_reportTs,  PAMEM_CONST_MODIFY,  OFFSET_REPORT_TIMESTAMP,  SIZE(s_reportTs)
        
        // Copy and forward the packet out
        mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_COPY
        xout XID_CDECTRL,             s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
    
        // Calculate, load and insert timestamp
        ldi   r1,    PDSP0_TIMER
        lbbo  r0,    r1, 8, 4
        mov   r0.w2, 0xffff
        sub   r0.w0, r0.w2,  r0.w0 
        //lbco  r0.w2, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP+2,   2 
        lbco  r1,   PAMEM_CONST_PARSE, OFFSET_SYS_TIMESTAMP, 8
        mov   r0.w2, r1.w0
        mov   s_pktDescr.swInfo1, r1.w2
        mov   s_pktDescr.swInfo1.w2, r2.w0
        // Wait for new packet to be ready
        wbs   s_flags.info.tStatus_CDENewPacket
        
        // Update packet descriptor
        // xin   XID_CDEDATA, s_pktDescr.timestamp,   8
        mov   s_pktDescr.timestamp,  r0
        mov   s_pktDescr.swInfo0,    s_reportTs.swInfo0
        xout  XID_CDEDATA, s_pktDescr.timestamp,   12
        
        // Flush out the packet                   
        mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
        xout XID_CDECTRL,           s_cdeCmdWd,              4
  
        mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_END
        xout XID_CDECTRL,           s_cdeCmdWd,              4
        
        // Forward the packet
        zero &s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
        mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
        mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
        mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
        mov  s_cdeCmdPkt.destQueue,    s_reportTs.destQueue
        mov  s_cdeCmdPkt.flowId,       s_reportTs.flowId
    
        // Pass through
    
f_nextRoute0_2:
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
    jmp  f_mainLoop

    .leave cdeScope
    .leave modifyScope
    .leave pktScope 
    .leave pktCaptureScope
    
// *******************************************************************************
// * FUNCTION PURPOSE: Group 7 (Dummy and IP Fragments) Command
// *******************************************************************************
// * DESCRIPTION: Process the dummy command 
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      | Message Length Patch 1 
// *   R3:      | Message Length Patch 2
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  pktDesc (s_bPatchStub)
// *   R7:        |
// *   R8:        |  &
// *   R9:        |  
// *   R10:       |  ipHeader  
// *   R11:       |  (timestamp) [for packet descriptor updated}
// *   R12:       |  (swInfo0)
// *   R13:       |  (swInfo1)
// *   R14:          |  ip Fragment Control block                                          
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:            | swInfo0   
// *   R18:       | Packet ID 
// *   R19:            |  next route stub (s_nRouteStub)
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:            | s_msg or s_ipFrag 
// *   R23:              | It will be over-written by the command loaded
// *   R24:              | 
// *   R25:              |
// *   R26:  Packet ID   | 
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    .using cdeScope
    .using pktScope     // For offset info
    .using ipScope
    .using modifyScope

f_group7Cmd:

    // Advance past the command
    // Dummy, IP fragment and Patch Message Length commands: 4 bytes
    mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount, 4
    
    qbgt  f_txCmdErrHandle, s_modState3.remCmdSize, s_cdeCmdWd.byteCount
    sub   s_modState3.remCmdSize, s_modState3.remCmdSize, s_cdeCmdWd.byteCount  
    
    xout XID_CDECTRL,          s_cdeCmdWd,             4
    add  s_modCxt.cmdFlushBytes, s_modCxt.cmdFlushBytes, s_cdeCmdWd.byteCount
    
    // Extract the sub command code
    and  r1.b0,  s_ipFrag.cmdId_subCode,  PA_SUB_CMD_CODE_MASK
    qbeq fci_mProc4, r1.b0, PA_SUB_CMD_CODE_DUMMY 
    qbeq l_group7Cmd_crcVerify, r1.b0, PA_SUB_CMD_CODE_EMAC_CRC_VERIFY 
    qbne f_ipFrag, r1.b0, PA_SUB_CMD_CODE_PATCH_MSG_LEN 
    
f_patchMsgLen:    
    // Ignore this command if the number of message length commands exceed the limit
    // Note: checksum command will not be in conjunction with IP
    //       fragmentation and message length patching command 
    qble  fci_mProc4,  s_modState.chkSumCount, SUBS_MOD_CXT_MAX_MSGLEN_PATCHES

    // Convert and Copy the entire command to memory
    // The message length size is stored as a flag in the msgLen field 
    mov  s_patchMsgLen3.msgLenSize, 2       //default message length size is 16-bit
    qbbc l_patchMsgLen1,    s_patchMsgLenCmd.msgLen.t_patch_msg_len_size32
        mov  s_patchMsgLen3.msgLenSize, 4
        clr  s_patchMsgLenCmd.msgLen.t_patch_msg_len_size32  
        
        //pass through
              
l_patchMsgLen1:
    lsl  r3.w2,  s_modState.chkSumCount,    2                     // Offset to memory (4 bytes each entry)
    add  r3.w2,  r3.w2,                     OFFSET_PATCH_MSG_LEN  // Base offset

    sbco  s_patchMsgLen3,  PAMEM_CONST_MODIFY,  r3.w2,   SIZE(s_patchMsgLen3)

    // keep count of the number of message length patching commands are stored
    add s_modState.chkSumCount, s_modState.chkSumCount, 1

    jmp  fci_mProc4
    
l_group7Cmd_crcVerify:
    jmp  f_emacCrcVerify    
        
f_ipFrag:    
    qbbs l_ipFrag1, s_modState.flags.t_subsMCxtTerminate
    // Store the commad if the next route info is not received provided
    set s_modState.flags.t_subsMCxtIpFrag    
    set s_modState.flags.t_subsMCxtPadding  // padding is always on if IP fragments is required
    sbco s_ipFrag,  PAMEM_CONST_MODIFY,     OFFSET_IP_FRAG,  SIZE(s_ipFrag)
    jmp  fci_mProc4
    
fci_ipFrag:    
    //Load the IP fragmentation data 
    lbco s_ipFrag,  PAMEM_CONST_MODIFY,     OFFSET_IP_FRAG,  SIZE(s_ipFrag)
    
l_ipFrag1:    
    // Clear out and initialize the IP Fragmentation working Context
    zero    &s_ipFragCxt, SIZE(s_ipFragCxt)
    mov     s_ipFragCxt.mtuSize,    s_ipFrag.mtuSize
    mov     s_ipFragCxt.ipOffset,   s_ipFrag.ipOffset        
    
    qbne  l_ipFrag1_skipPktMove, r2.w0, 0  
    // Advance the CDE to the beginning of the packet data
    mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
    xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

l_ipFrag1_skipPktMove:

    // Advance to the IP header
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_ipFragCxt.ipOffset,    r2.w0
    xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)
    
    //Load the message length patching data (r2, r3)
    lbco s_patchMsgLen1, PAMEM_CONST_MODIFY,     OFFSET_PATCH_MSG_LEN,  2*SIZE(s_patchMsgLen1)

    //
    // Process IPv4
    //
l_ipFrag_ProcessIp:
    // Load the packet Ip header
    xin     XID_CDEDATA, s_Ip, SIZE(s_Ip)
    
    // Support IP Version 4 only
    and     r0.b0, s_Ip.VerLen, 0xF0
    qbeq    l_ipFrag_ProcessIp_1, r0.b0, 0x40
    
    //Must be IPv6: Error check may be required
    //
    qbeq    fci_ipv6Frag, r0.b0, 0x60
    jmp     l_ipFrag_ProcessIp_none
    
l_ipFrag_ProcessIp_1:
    // Save off the length reported by IP
    mov     s_ipFragCxt.ipLen, s_Ip.TotalLen

    // Set offset to L4
    and     s_ipFragCxt.ipHdrLen, s_Ip.VerLen, 0xF
    lsl     s_ipFragCxt.ipHdrLen, s_ipFragCxt.ipHdrLen, 2

    // If the packet fits in the MTU, we simply forward it
    qble    l_ipFrag_ProcessIp_none, s_ipFragCxt.mtuSize, s_ipFragCxt.ipLen

    // If the DF flag is set, we are done
    qbbs    l_ipFrag_ProcessIp_none, s_Ip.FragOff.t_ipv4_frag_df

    // Save the base offset
    mov     s_ipFragCxt.baseOffset, s_Ip.FragOff
        
    // Get the payload bytes per frag amount
    sub     s_ipFragCxt.payloadSize, s_ipFragCxt.mtuSize, s_ipFragCxt.ipHdrLen
    and     s_ipFragCxt.payloadSize.b0, s_ipFragCxt.payloadSize.b0, 0xF8

    // Clear the frag loop index
    //mov     s_ipFragCxt.loopOffset, 0

l_ipFrag_Loop:
    // Patch the frag offset
    lsr     r1, s_ipFragCxt.loopOffset, 3
    add     s_Ip.FragOff, s_ipFragCxt.baseOffset, r1

    // Get the payload size for this fragment
    add     r0, s_ipFragCxt.loopOffset, s_ipFragCxt.payloadSize
    sub     r1, s_ipFragCxt.ipLen, s_ipFragCxt.ipHdrLen
    qblt    l_ipFrag_NotLast, r1, r0
   
    // This is the last frag - adjust the final size
    sub     s_ipFragCxt.payloadSize, r1, s_ipFragCxt.loopOffset
    jmp     l_ipFrag_CommonFrag
        
l_ipFrag_NotLast:        
    // Set more fragments
    set     s_Ip.FragOff.t_ipv4_frag_m

l_ipFrag_CommonFrag:
    // Patch the IP length
    add     s_Ip.TotalLen, s_ipFragCxt.payloadSize, s_ipFragCxt.ipHdrLen
    
    mov     s_Ip.Checksum, 0

    // Write out the new IP header
    xout    XID_CDEDATA, s_Ip, SIZE(s_Ip)-8         // We never update the last 8 bytes (or options)
    
    // record IP fragment size
    add     s_modState.pktSize, s_Ip.TotalLen, s_ipFragCxt.ipOffset
    
    // update tx stats
    set  s_statsFlags.event.t_nTxIpFrag
    sbco s_statsFlags.event,  cStatistics,       OFFSET_STATS_FLAGS,  4
    zero &s_statsFlags,       SIZE(s_statsFlags)
    
    // Perform message length patching if any
    qbeq    l_ipFrag_CommonFrag_5,      s_modState.chkSumCount, 0 
        // first message patch command
        mov  s_cdeCmdPatch.operation,   CDE_CMD_PATCH_PACKET
        mov  s_cdeCmdPatch.len,         s_patchMsgLen1.msgLenSize
        mov  s_cdeCmdPatch.offset,      s_patchMsgLen1.offset
        qbeq l_ipFrag_CommonFrag_1, s_cdeCmdPatch.len, 2
            // 32-bit message length
            add s_cdeCmdPatch.data,    s_patchMsgLen1.msgLen,  s_Ip.TotalLen
            jmp l_ipFrag_CommonFrag_2
l_ipFrag_CommonFrag_1:
            // 16-bit message length
            add s_cdeCmdPatch.data.w2, s_patchMsgLen1.msgLen,  s_Ip.TotalLen
l_ipFrag_CommonFrag_2:            
        xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
        
    qbeq    l_ipFrag_CommonFrag_5,      s_modState.chkSumCount, 1 
        // second message patch command
        mov  s_cdeCmdPatch.len,         s_patchMsgLen2.msgLenSize
        mov  s_cdeCmdPatch.offset,      s_patchMsgLen2.offset
        qbeq l_ipFrag_CommonFrag_3, s_cdeCmdPatch.len, 2
            // 32-bit message length
            add s_cdeCmdPatch.data,    s_patchMsgLen2.msgLen,  s_Ip.TotalLen
            jmp l_ipFrag_CommonFrag_4
l_ipFrag_CommonFrag_3:
            // 16-bit message length
            add s_cdeCmdPatch.data.w2, s_patchMsgLen2.msgLen,  s_Ip.TotalLen
l_ipFrag_CommonFrag_4:            
        xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
    
l_ipFrag_CommonFrag_5:    
    // Start the IP checksum. 
    zero    &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
    mov     s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
    mov     s_cdeCmdChk.byteLen,  s_ipFragCxt.ipHdrLen
    mov     s_cdeCmdChk.offset,   OFFSET(s_Ip.Checksum)          // Starting sum is 0, offset is 10
    xout    XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
            
    // Slide the window past the IP header
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_ipFragCxt.ipHdrLen
    xout XID_CDECTRL,           s_cdeCmdWd,              4
    
    qbeq l_ipFrag_Patch, s_ipFragCxt.loopOffset, 0
    // Now flush any data contained in s_ipFragCxt.loopOffset
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_ipFragCxt.loopOffset
        xout XID_CDECTRL, s_cdeCmdWd, 4  
        // r1.w0 store the payload offset from the IP payload due to potential patch operation
        mov  r1.w0, 0
                
l_ipFrag_NoFlush:        
    // re-enter from l_ipFrag_Patch as well:r1.w0 store the payload offset from the IP payload due to potential patch operation 
    // Slide the window past the valid data
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_ipFragCxt.payloadSize, r1.w0
    xout XID_CDECTRL,           s_cdeCmdWd,              4

    // Setup the offset for the next packet (jump out if final frag)
    add     s_ipFragCxt.loopOffset, s_ipFragCxt.loopOffset, s_ipFragCxt.payloadSize
    add     r1, s_ipFragCxt.loopOffset, s_ipFragCxt.ipHdrLen
    qble    l_ipFrag_LastPacket, r1, s_ipFragCxt.ipLen
    
    // Flush out the remaining packet 
    // Note: It is not necessary if we can use CDE_FLG_TRUNCATE.
    //       However, CDE_FLG_TRUNCATE flag will prevent the IP header checksum to be calculated due to a silicon bug
    //       It is just a workaround for IP header checksum 
    mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,             s_cdeCmdPkt,             4
    
    // IP fragment padding check
    // Make sure the fragment is at least 60 bytes long
    qble    l_ipFrag_ForwardPkt_0, s_modState.pktSize, 60

        // Insert packet data
        mov     s_cdeCmd.v0.w0, CDE_CMD_INSERT_PACKET
        rsb     s_cdeCmd.v0.b1, s_modState.pktSize, 60
        xout    XID_CDECTRL, s_cdeCmd, 4              // Send the command 
        
        //Update padding cnt
        add     s_modCxt.paddingCnt,    s_modCxt.paddingCnt,    1 
        
        // pass through   
    
l_ipFrag_ForwardPkt_0:
    // Forward the packet
    zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
    and  s_cdeCmdPkt.threadId,     s_nRouteStub.cmdId_N_E_Dest,  PA_NEXT_ROUTE_DEST_MASK
    //mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_RETAIN_CHKSUM | CDE_FLG_TRUNCATE | CDE_FLG_SET_THREADID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_RETAIN_CHKSUM | CDE_FLG_SET_THREADID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.destQueue,    s_nRouteStub.destQueue
    mov  s_cdeCmdPkt.flowId,       s_nRouteStub.flowId
    
    qbbc l_ipFrag_ForwardPkt,  s_modState.flags.t_subsMCxtSrio
        // SRIO use 8 byte PSINFO
        mov  s_cdeCmdPkt.psInfoSize,   8
        
l_ipFrag_ForwardPkt:    
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)

    // pass through
    // jmp     l_ipFrag_WaitForCopy                         // Let the new packet bit clear

l_ipFrag_WaitForCopy:
    // Wait for a new packet
    qbbc    l_ipFrag_WaitForCopy, s_flags.info.tStatus_CDENewPacket
        
    // Read packet descriptor
    // 
    // xin  XID_CDEDATA,  s_pktDescr,  OFFSET(s_pktDescr.pktDataSize) 
   
    qbbc l_ipFrag_WaitForCopy_1,  s_modState.flags.t_subsMCxtSrio
         // Update the pktType
         mov     s_pktDescr.pktType_pvtFlags,    s_modState.pktType_psFlags
         xout    XID_CDEDATA,  s_pktDescr.pktType_pvtFlags,  SIZE(s_pktDescr.pktType_pvtFlags)
         //jmp     l_ipFrag_WaitForCopy_2          // the next check will fail and so jmp     

l_ipFrag_WaitForCopy_1: 
    qbbc l_ipFrag_WaitForCopy_2,  s_modState.flags.t_subsMCxtPsFlags
         // Update the psFlags
         mov     s_pktDescr.psFlags_errorFlags,    s_modState.pktType_psFlags
         xout    XID_CDEDATA,  s_pktDescr.psFlags_errorFlags,  SIZE(s_pktDescr.psFlags_errorFlags)
        
         // Pass through

l_ipFrag_WaitForCopy_2: 
    // Update swInfo0 and swinfo1
    xout  XID_CDEDATA,  s_pktDescr.swInfo0,  8
   
    // Advance to the control section
    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout XID_CDECTRL,            s_cdeCmdWd,                    4

    qbbc l_ipFrag_WaitForCopy_3,  s_modState.flags.t_subsMCxtSrio

    // Insert 8 bytes of PS info
    // This is legal even though the window has advanced to control
    mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    mov s_cdeInsert.byteCount,  4
    mov s_cdeInsert.bytes,      s_pktDescr.swInfo0
    xout XID_CDECTRL,           s_cdeInsert,             SIZE(s_cdeInsert)
   
    mov s_cdeInsert.bytes,      s_pktDescr.swInfo1
    xout XID_CDECTRL,           s_cdeInsert,             SIZE(s_cdeInsert)
   
l_ipFrag_WaitForCopy_3:

    // Flush out the control info
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount, s_modCxt.cmdFlushBytes
    xout XID_CDECTRL,            s_cdeCmdWd,            4

    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_PACKET
    xout XID_CDECTRL,            s_cdeCmdWd,            4
   
    
    //mov  s_cdeCmdWd.operation,   CDE_CMD_FLUSH_TO_PACKET
    //xout XID_CDECTRL,            s_cdeCmdWd,            4
   
    // Advance to the IP header
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_ipFragCxt.ipOffset
    xout XID_CDECTRL,           s_cdeCmdWd,             4
    
    // Load the packet Ip header
    xin     XID_CDEDATA, s_Ip, SIZE(s_Ip)
   
    jmp     l_ipFrag_Loop

l_ipFrag_LastPacket:
    // Make sure the packet is at least 60 bytes long
    //add     s_modState.pktSize, s_Ip.TotalLen, s_ipFragCxt.ipOffset
    jmp     fci_nextRoute_skipModComplete
    
l_ipFrag_ProcessIp_none:
    // There is no IP fragmentation
    // Perform message length patching if any
    qbeq    l_ipFrag_ProcessIp_none_5,       s_modState.chkSumCount, 0 
        // first message patch command
        mov  s_cdeCmdPatch.operation,   CDE_CMD_PATCH_PACKET
        mov  s_cdeCmdPatch.len,         s_patchMsgLen1.msgLenSize
        mov  s_cdeCmdPatch.offset,      s_patchMsgLen1.offset
        qbeq l_ipFrag_ProcessIp_none_1, s_cdeCmdPatch.len, 2
            // 32-bit message length
            add s_cdeCmdPatch.data,    s_patchMsgLen1.msgLen,  s_ipFragCxt.ipLen
            jmp l_ipFrag_ProcessIp_none_2
l_ipFrag_ProcessIp_none_1:
            // 16-bit message length
            add s_cdeCmdPatch.data.w2, s_patchMsgLen1.msgLen,  s_ipFragCxt.ipLen
l_ipFrag_ProcessIp_none_2:            
        xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
        
    qbeq    l_ipFrag_ProcessIp_none_5,        s_modState.chkSumCount, 1 
        // second message patch command
        mov  s_cdeCmdPatch.len,         s_patchMsgLen2.msgLenSize
        mov  s_cdeCmdPatch.offset,      s_patchMsgLen2.offset
        qbeq l_ipFrag_ProcessIp_none_3, s_cdeCmdPatch.len, 2
            // 32-bit message length
            add s_cdeCmdPatch.data,    s_patchMsgLen2.msgLen,  s_ipFragCxt.ipLen
            jmp l_ipFrag_ProcessIp_none_4
l_ipFrag_ProcessIp_none_3:
            // 16-bit message length
            add s_cdeCmdPatch.data.w2, s_patchMsgLen2.msgLen,  s_ipFragCxt.ipLen
l_ipFrag_ProcessIp_none_4:            
        xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
    
l_ipFrag_ProcessIp_none_5:    
    // Prepare and jump to the modify operation
    mov r2.w0,  s_ipFragCxt.ipOffset 
    mov r30.w0, fci_nextRoute_skipModComplete
    jmp fci_modComplete5    // only patch command is allowed 
    
l_ipFrag_Patch:
    // Only one potential patch command  (insert the authentication tag to the AH header) 
    // Local variable: r1.w0 record the payload offset from IP after the patch operation
    mov r1.w0,  0
    qbne  l_ipFrag_NoFlush,  s_modState.patchCount,  1
    
    // Blind patch
    // Set the buffer address
    mov  r0.w0,   (PAMEM_BASE_BLIND_PATCH + 4) & 0xffff     // patch address
    mov  r0.w2,   (PAMEM_BASE_BLIND_PATCH + 4) >> 16

    // Need to update the patch address for PDSP5
    qbbc    l_ipFrag_Patch_1, s_modCxt.flags.t_isPdsp5
    add r0.b1,  r0.b1,  1      // Adding 0x100 to r0.w0

l_ipFrag_Patch_1:
    // read in the stub
    mov  r1.b2,   OFFSET_BLIND_PATCH                        // Stub offset
    lbco  s_bPatchStub,  PAMEM_CONST_MODIFY,  r1.b2,   SIZE(s_bPatchStub)
    
    add  r1.w2,   s_ipFragCxt.ipHdrLen, s_ipFragCxt.ipOffset 

    // Verify the offset is in range, otherwise, ignore the patch command
    qbgt  l_ipFrag_NoFlush, s_bPatchStub.patchOffset,  r1.w2
    
    // Only the replacement command is supported since we can not change IP payload length 
    qbbc  l_ipFrag_NoFlush, s_bPatchStub.cmdLen_insert.t_paBlindPatchOverwrite

    // Advance to the patch point
    // mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_bPatchStub.patchOffset,  r1.w2
    mov  r1.w0,                 s_cdeCmdWd.byteCount
    xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

    // Flush out the data
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    and  s_cdeCmdWd.byteCount,  s_bPatchStub.cmdId_Len,   PA_BLIND_PATCH_LEN_MASK
    xout XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)
    add  r1.w0,                 r1.w0,                    s_cdeCmdWd.byteCount

    // Do the patch
    mov  s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
    and  s_cdeCmdInD.lenLsb,     s_bPatchStub.cmdId_Len,  PA_BLIND_PATCH_LEN_MASK
    mov  s_cdeCmdInD.lenMsbs,    0
    mov  s_cdeCmdInD.dataP,      r0
    xout XID_CDECTRL,            s_cdeCmdWd,              SIZE(s_cdeCmdWd)

    // The packet actually advanced during the insert
    // Make no change to the tracking offset since the commands are all
    // relative to the original packet, not the patched packet
    jmp  l_ipFrag_NoFlush


.struct struct_ipv6FragUtil
    .u8    nextHdr
    .u8    hdrExtLen
.ends

.struct struct_ipv6FragUtil_ctr
    .u16    offset
.ends

.struct struct_ipv6FragCtrl
    .u8     ctrlFlag
#define     finishParse     t0
#define     foundNonFrag    t1
#define     saveNextHdr     t2
    .u8     fragNextHdr
    .u16    fragHdrOffset
.ends

.assign struct_ipv6FragUtil,      r0.w0,  r0.w0, ipv6FragUtil
.assign struct_ipv6FragUtil_ctr,  r0.w2,  r0.w2, ipv6FragUtil_ctr
.assign struct_ipv6FragCtrl,      r1,     r1,    ipv6FragCtrl
.assign struct_ipv6Frag,          r22,    r23,   s_ipv6Fragv1

// Macro for parsing for unfragmentable IPv6 ext headers
.macro m_ipv6FindUnfragHdr
.mparam proto
l_ipv6FindUnfragHdr0:
    // Route header is non-fragmentable
    qbeq l_ipv6FindUnfragHdr6, proto, IP_PROTO_NEXT_IPV6_ROUTE
    // Hop-by-hop header is non-fragmentable
    qbeq l_ipv6FindUnfragHdr6, proto, IP_PROTO_NEXT_IPV6_HOP_BY_HOP

    //l_ipv6FindUnfragHdr1:
    //  qbeq l_ipv6FindUnfragHdr7, proto, IP_PROTO_NEXT_IPV6_IN_IPV4

    qbeq l_ipv6FindUnfragHdr7, proto, IP_PROTO_NEXT_IPV6_NO_NEXT
    qbeq l_ipv6FindUnfragHdr7, proto, IP_PROTO_NEXT_IPV6_DEST_OPT
    qbeq l_ipv6FindUnfragHdr7, proto, IP_PROTO_NEXT_IPV6_FRAG

    set     ipv6FragCtrl.ctrlFlag.finishParse
    jmp     l_ipv6FindUnfragHdr7

l_ipv6FindUnfragHdr6:
    set     ipv6FragCtrl.ctrlFlag.foundNonFrag
    // Save nextHdr for fragHdr 
    set     ipv6FragCtrl.ctrlFlag.saveNextHdr

l_ipv6FindUnfragHdr7:
    // normal processing
.endm

fci_ipv6Frag:

l_ipv6Frag_ProcessIp:
    // Ip header has already been loaded 
    //xin     XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
   
l_ipv6Frag_ProcessIp_1:
    // mtu should be 8-byte alignned
    and     s_ipFragCxt.mtuSize.b0,    s_ipFragCxt.mtuSize.b0,  0xf8
    
    // Save off the payload length reported by IP
    add     s_ipFragCxt.ipLen, s_Ipv6a.payloadLen, IPV6_HEADER_LEN_BYTES
    mov     s_ipFragCxt.ipHdrLen, IPV6_HEADER_LEN_BYTES

    // If the packet fits in the MTU, we simply forward it
    qble    l_ipFrag_ProcessIp_none, s_ipFragCxt.mtuSize, s_ipFragCxt.ipLen

    // Look for ipv6 non-fragmentable header
    mov     s_ipFragCxt.nextHdr,   s_Ipv6a.next 
    mov     s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov     s_cdeCmdWd.byteCount,  IPV6_HEADER_LEN_BYTES
      
l_ipv6Frag_ShiftCheck:    
    qbeq    l_ipv6Frag_ProcessIp_none, s_ipFragCxt.nextHdr, IP_PROTO_NEXT_IPV6_FRAG      // Can't frag a fragged packet
    qbeq    l_ipv6Frag_CheckNext,  s_ipFragCxt.nextHdr,   IP_PROTO_NEXT_IPV6_HOP_BY_HOP
    qbeq    l_ipv6Frag_CheckNext,  s_ipFragCxt.nextHdr,   IP_PROTO_NEXT_IPV6_ROUTE
    qbeq    l_ipv6Frag_CheckNext,  s_ipFragCxt.nextHdr,   IP_PROTO_NEXT_IPV6_DEST_OPT
    jmp     l_ipv6Frag_HdrDone   
    
l_ipv6Frag_CheckNext:        
    // Step past the last header checked
    xout    XID_CDECTRL,           s_cdeCmdWd,              4
    
    // Load the next header
    xin  XID_CDEDATA,  s_Ipv6Opt,         SIZE(s_Ipv6Opt)

    // Check for hdr size greater than 255
    qble    l_ipv6Frag_HdrDone,     s_Ipv6Opt.optlen, 63    
    
    mov     s_ipFragCxt.nextHdr,    s_Ipv6Opt.proto
    add     s_cdeCmdWd.byteCount,   s_Ipv6Opt.optlen, 1
    lsl     s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount, 3
    add     s_ipFragCxt.ipHdrLen,   s_ipFragCxt.ipHdrLen, s_cdeCmdWd.byteCount
    
    jmp     l_ipv6Frag_ShiftCheck

l_ipv6Frag_HdrDone:
    add     r0, s_ipFragCxt.ipHdrLen,   IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    sub     s_ipFragCxt.payloadSize,    s_ipFragCxt.mtuSize,    r0    

    // If the non-fragmentable portion consumes the full mtu, we can't frag
    qbge    l_ipv6Frag_ProcessIp_none, s_ipFragCxt.mtuSize, r0
   
    // FragTotalSize (length of fragmentable data) = ipLen - ipHdrLen  
    sub     s_ipFragCxt.fragTotalSize, s_ipFragCxt.ipLen, s_ipFragCxt.ipHdrLen
        
    // Clear the frag loop index
    // The entire ipFragCxt are initialized to 0
    //mov     s_ipFragCxt.loopOffset, 0
    
    // Record and update the fragmentation Id
    lbco    s_ipFragCxt.fragId, PAMEM_CONST_CUSTOM, OFFSET_IPV6_FRAG_ID, 4
    add     s_ipFragCxt.fragId, s_ipFragCxt.fragId, 1
    sbco    s_ipFragCxt.fragId, PAMEM_CONST_CUSTOM, OFFSET_IPV6_FRAG_ID, 4
    
    // If ipHdrLen==IPV6_HEADER_LEN_BYTES, then we still have the original IP header,
    // else we need to patch
    qbeq    l_ipv6Frag_only_fixed_header, s_ipFragCxt.ipHdrLen, IPV6_HEADER_LEN_BYTES
        // Found non-fragmentable extension header
        mov     s_Ipv6Opt.proto,    IP_PROTO_NEXT_IPV6_FRAG 
        xout    XID_CDEDATA, s_Ipv6Opt,         SIZE(s_Ipv6Opt)
        
        // Step past the last header we checked
        xout    XID_CDECTRL, s_cdeCmdWd, 4 
        
        // Patch the IPv6 header Length
        mov     s_cdeCmdPatch.operation,   CDE_CMD_PATCH_PACKET
        mov     s_cdeCmdPatch.len,         SIZE(s_Ipv6a.payloadLen)
        add     s_cdeCmdPatch.offset,      s_ipFragCxt.ipOffset, OFFSET(s_Ipv6a.payloadLen)
        sub     s_cdeCmdPatch.data.w2,     s_ipFragCxt.mtuSize,  IPV6_HEADER_LEN_BYTES
        xout    XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
        jmp     l_ipv6Frag_Loop
    
l_ipv6Frag_only_fixed_header:
        // Set next header in IPv6 fixed header to frag
        mov     s_Ipv6a.next, IP_PROTO_NEXT_IPV6_FRAG
        sub     s_Ipv6a.payloadLen, s_ipFragCxt.mtuSize, IPV6_HEADER_LEN_BYTES        
        xout    XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
        
        // Step past the last header we checked
        xout    XID_CDECTRL, s_cdeCmdWd, 4 

l_ipv6Frag_Loop:

    // update tx stats
    set     s_statsFlags.event.t_nTxIpFrag
    sbco    s_statsFlags.event,  cStatistics,       OFFSET_STATS_FLAGS,  4
    zero    &s_statsFlags,       SIZE(s_statsFlags)
    
    // Perform message length patching if any
    qbeq    l_ipv6Frag_CommonFrag_5,      s_modState.chkSumCount, 0 
        add r1.w0,  s_ipFragCxt.ipHdrLen,  s_ipFragCxt.payloadSize
        add r1.w0,  r1.w0, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES 
        // first message patch command
        mov     s_cdeCmdPatch.operation,   CDE_CMD_PATCH_PACKET
        mov     s_cdeCmdPatch.len,         s_patchMsgLen1.msgLenSize
        mov     s_cdeCmdPatch.offset,      s_patchMsgLen1.offset
        qbeq    l_ipv6Frag_CommonFrag_1, s_cdeCmdPatch.len, 2
            // 32-bit message length
            add     s_cdeCmdPatch.data,    s_patchMsgLen1.msgLen,  r1.w0
            jmp     l_ipv6Frag_CommonFrag_2
l_ipv6Frag_CommonFrag_1:
            // 16-bit message length
            add     s_cdeCmdPatch.data.w2, s_patchMsgLen1.msgLen,  r1.w0
l_ipv6Frag_CommonFrag_2:            
        xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
        
        qbeq    l_ipv6Frag_CommonFrag_5,      s_modState.chkSumCount, 1 
            // second message patch command
            mov     s_cdeCmdPatch.len,         s_patchMsgLen2.msgLenSize
            mov     s_cdeCmdPatch.offset,      s_patchMsgLen2.offset
            qbeq    l_ipv6Frag_CommonFrag_3, s_cdeCmdPatch.len, 2
                // 32-bit message length
                add     s_cdeCmdPatch.data,    s_patchMsgLen2.msgLen,  r1.w0
                jmp     l_ipv6Frag_CommonFrag_4
                
l_ipv6Frag_CommonFrag_3:
                // 16-bit message length
                add     s_cdeCmdPatch.data.w2, s_patchMsgLen2.msgLen,  r1.w0
l_ipv6Frag_CommonFrag_4:            
            xout    XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
    
l_ipv6Frag_CommonFrag_5:  

    // Update and insert the Fragmentation header
    mov     s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
    mov     s_cdeCmdIn2.len,           4 
    lsl     s_cdeCmdIn2.data.w2,       s_ipFragCxt.nextHdr,  8      
    mov     s_cdeCmdIn2.data.w0,       s_ipFragCxt.loopOffset

    // Get the payload size for this fragment
    add     r0, s_ipFragCxt.loopOffset, s_ipFragCxt.payloadSize
    qble    l_ipv6Frag_CommonFrag_6, r0,  s_ipFragCxt.fragTotalSize
    
    // There are more fragments
    set     s_cdeCmdIn2.data.w0.t_ipv6_frag_m
   
l_ipv6Frag_CommonFrag_6:
    // Complete the fragmentation header
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    mov     s_cdeCmdIn2.data, s_ipFragCxt.fragId
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
            
    // specify the window move because of the potential patch operation
    mov r1.w0, 0    
    qbeq l_ipv6Frag_Patch, s_ipFragCxt.loopOffset, 0
        // Now flush any data contained in s_ipFragCxt.loopOffset
        mov     s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov     s_cdeCmdWd.byteCount,  s_ipFragCxt.loopOffset
        xout    XID_CDECTRL, s_cdeCmdWd, 4  
        
        // r1.w0 store the payload offset from the IP payload due to potential patch operation
        mov     r1.w0, 0
                
l_ipv6Frag_NoFlush:        
    // re-enter from l_ipv6Frag_Patch as well:r1.w0 store the payload offset from the IP payload due to potential patch operation 
    // Slide the window past the valid data
    mov     s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub     s_cdeCmdWd.byteCount,  s_ipFragCxt.payloadSize, r1.w0
    xout    XID_CDECTRL,           s_cdeCmdWd,              4
    
l_ipv6Frag_FragReady:
    // Setup the offset for the next packet (jump out if final frag)
    add     s_ipFragCxt.loopOffset, s_ipFragCxt.loopOffset, s_ipFragCxt.payloadSize
    qble    l_ipv6Frag_LastPacket, s_ipFragCxt.loopOffset, s_ipFragCxt.fragTotalSize
    
    // Flush out the remaining packet 
    
l_ipv6Frag_ForwardPkt_0:
    // Forward the packet
    zero    &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
    mov     s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
    and     s_cdeCmdPkt.threadId,     s_nRouteStub.cmdId_N_E_Dest,  PA_NEXT_ROUTE_DEST_MASK
    mov     s_cdeCmdPkt.optionsFlag,  (CDE_FLG_TRUNCATE | CDE_FLG_SET_THREADID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    mov     s_cdeCmdPkt.destQueue,    s_nRouteStub.destQueue
    mov     s_cdeCmdPkt.flowId,       s_nRouteStub.flowId
    
    qbbc    l_ipv6Frag_ForwardPkt,  s_modState.flags.t_subsMCxtSrio
    // SRIO use 8 byte PSINFO
    mov     s_cdeCmdPkt.psInfoSize,   8
        
l_ipv6Frag_ForwardPkt:    
    xout    XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)

    // pass through
    // jmp     l_ipv6Frag_WaitForCopy                         // Let the new packet bit clear

l_ipv6Frag_WaitForCopy:
    // Wait for a new packet
    qbbc    l_ipv6Frag_WaitForCopy, s_flags.info.tStatus_CDENewPacket
        
    // Read packet descriptor
    // 
    // xin  XID_CDEDATA,  s_pktDescr,  OFFSET(s_pktDescr.pktDataSize) 
   
    qbbc    l_ipv6Frag_WaitForCopy_1,  s_modState.flags.t_subsMCxtSrio
        // Update the pktType
        mov     s_pktDescr.pktType_pvtFlags,    s_modState.pktType_psFlags
        xout    XID_CDEDATA,  s_pktDescr.pktType_pvtFlags,  SIZE(s_pktDescr.pktType_pvtFlags)
        //jmp     l_ipv6Frag_WaitForCopy_2          // the next check will fail and so jmp     

l_ipv6Frag_WaitForCopy_1: 
    qbbc    l_ipv6Frag_WaitForCopy_2,  s_modState.flags.t_subsMCxtPsFlags
        // Update the psFlags
        mov     s_pktDescr.psFlags_errorFlags,    s_modState.pktType_psFlags
        xout    XID_CDEDATA,  s_pktDescr.psFlags_errorFlags,  SIZE(s_pktDescr.psFlags_errorFlags)

        // Pass through

l_ipv6Frag_WaitForCopy_2: 
    // Update swInfo0 and swinfo1
    xout    XID_CDEDATA,  s_pktDescr.swInfo0,  8
   
    // Advance to the control section
    mov     s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout    XID_CDECTRL,            s_cdeCmdWd,                    4

    qbbc    l_ipv6Frag_WaitForCopy_3,  s_modState.flags.t_subsMCxtSrio

        // Insert 8 bytes of PS info
        // This is legal even though the window has advanced to control
        mov     s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
        mov     s_cdeInsert.byteCount,  4
        mov     s_cdeInsert.bytes,      s_pktDescr.swInfo0
        xout    XID_CDECTRL,           s_cdeInsert,             SIZE(s_cdeInsert)
   
        mov     s_cdeInsert.bytes,      s_pktDescr.swInfo1
        xout    XID_CDECTRL,           s_cdeInsert,             SIZE(s_cdeInsert)
   
l_ipv6Frag_WaitForCopy_3:
   
    // Flush out the control info
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount, s_modCxt.cmdFlushBytes
    xout XID_CDECTRL,            s_cdeCmdWd,            4

    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_PACKET
    xout XID_CDECTRL,            s_cdeCmdWd,            4

    //mov     s_cdeCmdWd.operation,   CDE_CMD_FLUSH_TO_PACKET
    //xout    XID_CDECTRL,            s_cdeCmdWd,            4
   
    // Advance to the IP header
    mov     s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov     s_cdeCmdWd.byteCount,  s_ipFragCxt.ipOffset
    xout    XID_CDECTRL,           s_cdeCmdWd,             4
    
    // Load the packet Ip header
    xin     XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
    
    // Calc the size for this loop
    add     r0, s_ipFragCxt.loopOffset, s_ipFragCxt.payloadSize
    qbge    l_ipv6Frag_NoAdjust, r0, s_ipFragCxt.fragTotalSize
        sub s_ipFragCxt.payloadSize, s_ipFragCxt.fragTotalSize, s_ipFragCxt.loopOffset 
        // pass through
        
l_ipv6Frag_NoAdjust:   
        add     s_Ipv6a.payloadLen, s_ipFragCxt.payloadSize,    s_ipFragCxt.ipHdrLen
        sub     s_Ipv6a.payloadLen, s_Ipv6a.payloadLen,         IPV6_HEADER_LEN_BYTES - IPV6_OPT_FRAG_EXTENSION_LEN_BYTES         
        xout    XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
        
        mov     s_cdeCmdWd.byteCount,  s_ipFragCxt.ipHdrLen
        xout    XID_CDECTRL,           s_cdeCmdWd,             4
    
   
    jmp     l_ipv6Frag_Loop

l_ipv6Frag_LastPacket:
    // Make sure the packet is at least 60 bytes long
    //add     s_modState.pktSize, s_Ip.TotalLen, s_ipFragCxt.ipOffset
    jmp     fci_nextRoute_skipModComplete
    
l_ipv6Frag_Patch:
    // Only one potential patch command  (insert the authentication tag to the AH header) 
    // Local variable: r1.w0 record the payload offset from IP after the patch operation
    mov     r1.w0,  0
    qbne    l_ipv6Frag_NoFlush,  s_modState.patchCount,  1
    
    // Blind patch
    // Set the buffer address
    mov     r0.w0,   (PAMEM_BASE_BLIND_PATCH + 4) & 0xffff     // patch address
    mov     r0.w2,   (PAMEM_BASE_BLIND_PATCH + 4) >> 16

    // Need to update the patch address for PDSP5
    qbbc    l_ipv6Frag_Patch_1, s_modCxt.flags.t_isPdsp5    
    add     r0.b1,  r0.b1,  1      // Adding 0x100 to r0.w0

l_ipv6Frag_Patch_1:
    // read in the stub
    mov     r1.b2,   OFFSET_BLIND_PATCH                        // Stub offset
    lbco    s_bPatchStub,  PAMEM_CONST_MODIFY,  r1.b2,   SIZE(s_bPatchStub)
    
    add     r1.w2,   s_ipFragCxt.ipHdrLen, s_ipFragCxt.ipOffset 

    // Verify the offset is in range, otherwise, ignore the patch command
    qbgt    l_ipv6Frag_NoFlush, s_bPatchStub.patchOffset,  r1.w2
    
    // Only the replacement command is supported since we can not change IP payload length 
    qbbc    l_ipv6Frag_NoFlush, s_bPatchStub.cmdLen_insert.t_paBlindPatchOverwrite

    // Advance to the patch point
    mov  s_cdeCmdWd.operation,     CDE_CMD_WINDOW_ADVANCE
    sub     s_cdeCmdWd.byteCount,  s_bPatchStub.patchOffset,  r1.w2
    mov     r1.w0,                 s_cdeCmdWd.byteCount
    xout    XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

    // Flush out the data
    mov     s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    and     s_cdeCmdWd.byteCount,  s_bPatchStub.cmdId_Len,   PA_BLIND_PATCH_LEN_MASK
    xout    XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)
    add     r1.w0,                 r1.w0,                    s_cdeCmdWd.byteCount

    // Do the patch
    mov     s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
    and     s_cdeCmdInD.lenLsb,     s_bPatchStub.cmdId_Len,  PA_BLIND_PATCH_LEN_MASK
    mov     s_cdeCmdInD.lenMsbs,    0
    mov     s_cdeCmdInD.dataP,      r0
    xout    XID_CDECTRL,            s_cdeCmdWd,              SIZE(s_cdeCmdWd)

    // The packet actually advanced during the insert
    // Make no change to the tracking offset since the commands are all
    // relative to the original packet, not the patched packet
    jmp     l_ipv6Frag_NoFlush
    
l_ipv6Frag_ProcessIp_none:
    // Need to adjust the packet offset to be consistent with l_ipFrag_ProcessIp_none for patching operation
    // Step past the last header we checked
    xout    XID_CDECTRL, s_cdeCmdWd, 4 
    
l_ipv6Frag_ProcessIp_none2:    
    add     s_ipFragCxt.ipOffset, s_ipFragCxt.ipOffset, s_ipFragCxt.ipHdrLen
    jmp     l_ipFrag_ProcessIp_none
    
    
    .leave cdeScope
    .leave pktScope 
    .leave ipScope
    .leave modifyScope
    
// *********************************************************************************
// * FUNCTION PURPOSE: Parse L2 layer 
// *********************************************************************************
// * DESCRIPTION: Parse L2 Layer in EQoS Mode operation
// *              - Extract VLAN P-bit
// *              - Extract IP DSCP value
// *              - Replace VLAN ID
// *              - Update flow number and Queue number per EQoS algorithm
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    r1.w2: EQoS table offset (local)
// *   R2:    r2.w0: packet parsing offset  (pass in/out)
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  L2 Header
// *   R7:        |
// *   R8:        |  
// *   R9:          | s_paComIfEQoS 
// *   R10:         |
// *   R11:       | timestamp
// *   R12:       | swInfo0
// *   R13:       | swInfo2
// *   R14:          |  etherTypes
// *   R15:          |                                            
// *   R16:          |                                            
// *   R17:          | 
// *   R18:            |  s_paEQosScratch
// *   R19:            |  next route stub (s_nRouteStub)
// *   R20:       | Modify State machine (s_modState)
// *   R21:       |
// *   R22:     
// *   R23:       
// *   R24:            
// *   R25:            
// *   R26:  
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ***********************************************************************************

    .using cdeScope
    .using modifyScope
    .using eQosCompScope
    
f_EQoS_PraseL2:

    // Advance the CDE to the beginning of the packet data
    mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
    xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

    // r2.w0 has the current packet offset
    // mov  r2.w0,  0
    //qbbc fci_modComplete1, s_modCxt.flags.t_eqos_feature  
      
      // walk through the packet to derive queue and flow
      // clear vlan tag indicator, IP PKT indicator
      zero  &s_paEQosScratch.status, SIZE(s_paEQosScratch)
      
      // clear the eqos queue and flow 
      // mov  s_eqosQueueFlow, 0
  
      // load per port configuration from the scratch memory
      // get the port number to be captured and obtain the offset                              
      mov   r1.w0, OFFSET_EQOS_CFG_BASE
      and   r1.w2, s_modState.pktType_psFlags, 0x70
      sub   r1.w2, r1.w2, 0x10  
      lsl   r1.w2, r1.w2, 8 - 4
  
      // point to per port configuration structure
      add   r1.w2, r1.w2, r1.w0
      
      // load the port control and other information
      lbco  s_paComIfEQoS, PAMEM_CONST_PORTCFG, r1.w2, SIZE(s_paComIfEQoS)  
      
      // Move to the VLAN offset table
      add   r1.w2, r1.w2, 8

      // load default priority
      mov   r1.w0, OFFSET_EQOS_CFG_EG_DEF_PRI		  
      lbco s_paEQosScratch.priority , PAMEM_CONST_PORTCFG, r1.w0, 1  
  
      // Pass off macAddr bytes 
      mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
      mov  s_cdeCmdWd.byteCount,   SIZE(struct_MacAddr)
      xout XID_CDECTRL,            s_cdeCmdWd,            SIZE(s_cdeCmdWd)
      mov  r2.w0, s_cdeCmdWd.byteCount  
  
      // Load the ethertype table from memory
      lbco  s_ethertypes,  PAMEM_CONST_PARSE,  OFFSET_ETYPE_TABLE,  SIZE(s_ethertypes)  

l_EQoS_ParseL2_Loop:  
      mov  s_cdeCmdWd.byteCount,   2   // minimum 2 bytes for ethertype

      // Read in enough data to cover the llc/snap header if present
      xin  XID_CDEDATA,   s_tagOrLen,  SIZE(s_tagOrLen)  

      mov    r3,    1500
      qblt   l_EQoS_ParseL2_0,  s_tagOrLen.len,   r3

        // tag/len < 1500, so verify DSP, SSAP and ctrl
        //qbne  l_EQoS_ParseL2_8,  s_tagOrLen.dsap,  0xaa
        //qbne  l_EQoS_ParseL2_8,  s_tagOrLen.asap,  0xaa
        //qbne  l_EQoS_ParseL2_8,  s_tagOrLen.ctrl,  0x03
        //  802.3
        //  Copy the new ethertype value to the common location
        mov   s_tagOrLen.len,    s_tagOrLen.etype2

        // scroll past the 8 bytes of llc/snap header
        add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,  8

l_EQoS_ParseL2_0:
        // scroll past the ethertype/llc snap
        // cdeCmdWd.operation already has the value CDE_CMD_WINDOW_ADVANCE
        xout  XID_CDECTRL,   s_cdeCmdWd,          SIZE(s_cdeCmdWd)
        // Track the active parse
        add  r2.w0,  r2.w0,    s_cdeCmdWd.byteCount		  
        qbne  l_EQoS_ParseL2_1,   s_tagOrLen.len,  s_ethertypes.vlan
      
l_EQoS_ParseL2_Vlan:
        // Hdr is  PA_HDR_VLAN 
        xin  XID_CDEDATA,         s_vtag.tag,           SIZE(s_vtag.tag)
        
        // Should we replace vlan id
        qbbc  l_EQoS_ParseL2_Vlan_1, s_paComIfEQoS.ctrlBitMap.t_eqos_vlan_override
          // Clear the vlan id (lower 12 bits)
          // set the new vlan id (lower 12 bits)
          mov                     r2.w2,      0xF000
          and                     s_vtag.tag, s_vtag.tag, r2.w2
          or                      s_vtag.tag, s_vtag.tag, s_paComIfEQoS.vlanId
          xout XID_CDEDATA,       s_vtag.tag, SIZE(s_vtag.tag)   
          
l_EQoS_ParseL2_Vlan_1:
          // Indicate and store the P-bit as priority
          set  s_paEQosScratch.status.t_eqos_status_vlan_tag 
          lsr  s_paEQosScratch.vlanPri, s_vtag.tag,         VLAN_PCP_SHIFT 

          // Scroll the CDE past the tag to point to the next ethertype candidate
          mov  s_cdeCmdWd.byteCount,  SIZE(s_vtag.tag)       
          xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
          add  r2.w0,  r2.w0,         s_cdeCmdWd.byteCount	  
  
          jmp    l_EQoS_ParseL2_Loop  

l_EQoS_ParseL2_1:          
       qbne  l_EQoS_ParseL2_2,   s_tagOrLen.len,  s_ethertypes.Spvlan
         // Hdr is  PA_HDR_VLAN 
         // Scroll the CDE past the tag to point to the next ethertype candidate
         mov  s_cdeCmdWd.byteCount,  SIZE(s_vtag.tag)       
         xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
         add  r2.w0,  r2.w0,         s_cdeCmdWd.byteCount		  
         jmp  l_EQoS_ParseL2_Loop

l_EQoS_ParseL2_2:  
       qbne  l_EQoS_ParseL2_3,   s_tagOrLen.len,  s_ethertypes.ip
       
l_EQoS_ParseL2_Ipv4:          
         // Hdr is  PA_HDR_IPv4, get IPV4 DSCP priority		  	      
         xin  XID_CDEDATA,         s_Ipv4qos,           SIZE(s_Ipv4qos)	
         	  
         // Store DSCP in case we need DSCP priority routing
         lsr  s_paEQosScratch.dscp, s_Ipv4qos.Tos, 2	
         set  s_paEQosScratch.status.t_eqos_status_is_ip
         
         jmp l_EQoS_ParseL2_8
         
l_EQoS_ParseL2_3:          
        qbne  l_EQoS_ParseL2_4,   s_tagOrLen.len,  s_ethertypes.ipv6
        
l_EQoS_ParseL2_Ipv6:
          //next Hdr is  PA_HDR_IPv6, get IpV6 DSCP priority		  
          xin  XID_CDEDATA,         s_Ipv6qos,           SIZE(s_Ipv6qos)		  
          // Store DSCP in case we need DSCP priority routing
          lsr s_paEQosScratch.dscp, s_Ipv6qos.ver_tclass_flow.w2,  4
          lsr s_paEQosScratch.dscp, s_paEQosScratch.dscp, 2
          set s_paEQosScratch.status.t_eqos_status_is_ip
          
          jmp l_EQoS_ParseL2_8
          
l_EQoS_ParseL2_4:          
          qbne  l_EQoS_ParseL2_5,   s_tagOrLen.len,  s_ethertypes.mpls
l_EQoS_ParseL2_Mpls:		  
          // Hdr is  PA_HDR_MPLS 
          // get the next ethertype to proceed and continue parsing
          // Read in a single tag  	  
          xin  XID_CDEDATA,  s_mpls,  SIZE(s_mpls)     
          // Advance the mpls header 
          mov   s_cdeCmdWd.byteCount,  SIZE(s_mpls.tagTtl)
          xout  XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)
          add   r2.w0,  r2.w0,         s_cdeCmdWd.byteCount		  

          // Extract the S bit. If set then get a potential IP version number
          // For possible IPv4 and IPv6 continue parse
          qbbc  l_EQoS_ParseL2_7,  s_mpls.tagTtl.t_s	
            lsr  r3.b0,  s_mpls.ipVer,  4
            qbeq l_EQoS_ParseL2_Ipv4,  r3.b0, 4
            qbeq l_EQoS_ParseL2_Ipv6,  r3.b0, 6
            jmp l_EQoS_ParseL2_8 	

l_EQoS_ParseL2_5:
        qbeq  l_EQoS_ParseL2_Mpls,   s_tagOrLen.len,  s_ethertypes.mplsMulti
        // pass through
  
l_EQoS_ParseL2_6:
        qbne  l_EQoS_ParseL2_7,   s_tagOrLen.len,  s_ethertypes.PPPoE
        
l_EQoS_ParseL2_PPPoE:          
       // Hdr is  PA_HDR_PPPoE 
       // get the next ethertype to proceed and continue parsing
           xin  XID_CDEDATA,  s_pppoe,  SIZE(s_pppoe)   
           // Read in the PPPoE header		  
           //qbne  l_EQoS_ParseL2_8, s_pppoe.verType, PPPoE_VER_TYPE
           //qbne  l_EQoS_ParseL2_8, s_pppoe.code,    PPPoE_CODE_SESSION		  
           // Find out the next prot
           
          // Advance the PPPoE payload header 
          mov   s_cdeCmdWd.byteCount,  SIZE(s_pppoe)
          add   r2.w0,  r2.w0,         s_cdeCmdWd.byteCount		  
          xout  XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)		  
           
          mov   r2.w2, PPPoE_PROT_IPv4           
          qbeq  l_EQoS_ParseL2_Ipv4,  s_pppoe.prot, r2.w2
          mov   r2.w2, PPPoE_PROT_IPv6           
          qbeq  l_EQoS_ParseL2_Ipv6,  s_pppoe.prot, r2.w2
          
          jmp   l_EQoS_ParseL2_8

l_EQoS_ParseL2_7:
         qbeq   l_EQoS_ParseL2_PPPoE,   s_tagOrLen.len,  s_ethertypes.PPPoE_discov
            // pass through 
  
l_EQoS_ParseL2_8:
         // Execute EQoS algorithm
         // check mode, whether DSCP mode or DP-BIT mode
         qbbc l_EQoS_ParseL2_DscpEqosMode, s_paComIfEQoS.ctrlBitMap.t_eqos_dp_bit_mode
           qbbc l_EQoS_ParseL2_NotVlanTag, s_paEQosScratch.status.t_eqos_status_vlan_tag
             mov  s_paEQosScratch.priority, s_paEQosScratch.vlanPri
             jmp  l_EQoS_ParseL2_comp
l_EQoS_ParseL2_NotVlanTag:
         qbbs l_EQoS_ParseL2_comp, s_paComIfEQoS.ctrlBitMap.t_eqos_pri_override
l_EQoS_ParseL2_DscpEqosMode: 
         qbbc l_EQoS_ParseL2_comp, s_paEQosScratch.status.t_eqos_status_is_ip
           mov  s_paEQosScratch.priority, s_paEQosScratch.dscp
           add  r1.w2, r1.w2, 16
           // pass through
l_EQoS_ParseL2_comp:         
         // multiply priority by 2
         lsl  s_paEQosScratch.priority, s_paEQosScratch.priority, 1
         // point to the dscp priority containing queue and flow offsets
         add  r1.w2, r1.w2, s_paEQosScratch.priority 
     
         lbco r1.w0, PAMEM_CONST_PORTCFG, r1.w2, 2
         add  s_nRouteStub.flowId, s_paComIfEQoS.flowBase,  r1.b1
         add  s_nRouteStub.destQueue,  s_paComIfEQoS.queueBase, r1.b0
         mov  s_nRouteStub.cmdId_N_E_Dest,  PA_DEST_CDMA
         ret
    
    .leave cdeScope
    .leave modifyScope
    .leave eQosCompScope    

// *********************************************************************************
// * FUNCTION PURPOSE: Packet modifications are completed
// *********************************************************************************
// * DESCRIPTION: The checksums, crcs, and blind patch commands are 
// *              performed.
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:    r2.w0: current packet offset  
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |
// *   R8:        |  
// *   R9:        |  
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |  scratch
// *   R15:       |                                            
// *   R16:       |                                            
// *   R17:       |  
// *   R18:          | packet ID
// *   R19:            |  next route stub (s_nRouteStub)
// *   R20:       | Modify State machine (s_modState)
// *   R21:       |
// *   R22:     
// *   R23:       
// *   R24:            
// *   R25:            
// *   R26:  
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ***********************************************************************************

    .using cdeScope
    .using modifyScope
    .using  eQosCompScope
    
f_modComplete:

    qbne  fci_modComplete1, r2.w0,  0
      // Advance the CDE to the beginning of the packet data
      mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
      xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

      // r2.w0 has the current packet offset
      // mov  r2.w0,  0

fci_modComplete1:    

    // Setup the checksums
    qbeq  l_modComplete3,  s_modState.chkSumCount, 0

        // The first checksum
        lbco  s_cmdChkCrc,  PAMEM_CONST_MODIFY,  OFFSET_INSERT_CHKSUM,  SIZE(s_cmdChkCrc)
        
        // Verify the range
        qbgt  l_modComplete1,  s_cmdChkCrc.startOffset,  r2.w0

        // Scroll to the start of the first checksum
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_cdeCmdWd.byteCount,  s_cmdChkCrc.startOffset, r2.w0
        xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)
        sub  s_cdeCmdWd.byteCount,  s_cmdChkCrc.startOffset,  r2.w0
        add  r2.w0,                 r2.w0,                    s_cdeCmdWd.byteCount        

        mov  s_cdeCmdChk.operation, CDE_CMD_CHECKSUM1_COMPUTE
        mov  s_cdeCmdChk.byteLen,   s_cmdChkCrc.length
        mov  s_cdeCmdChk.initSum,   s_cmdChkCrc.initVal
        mov  s_cdeCmdChk.offset,    s_cmdChkCrc.resultOffset
        lsr  s_cdeCmdChk.options,   s_cmdChkCrc.flags,        7
        xout XID_CDECTRL,           s_cdeCmdChk,              SIZE(s_cdeCmdChk)


l_modComplete1:

    qbge  l_modComplete3,  s_modState.chkSumCount,  1

        // The second checksum
        lbco  s_cmdChkCrc,  PAMEM_CONST_MODIFY,  OFFSET_INSERT_CHKSUM+SIZE(s_cmdChkCrc),  SIZE(s_cmdChkCrc)

        // Make sure the range is valid
        qble  l_modComplete2,  s_cmdChkCrc.startOffset,  r2.w0
        
            // Invalid checksum specified. Discard the packet
            set s_statsFlags.event.t_nCommandFail

            mov  s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
            xout XID_CDECTRL,            s_cdeCmdPkt,          SIZE(s_cdeCmdPkt)

            jmp   f_mainLoop

l_modComplete2:

        // Scroll to the start of the second checksum
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_cdeCmdWd.byteCount,  s_cmdChkCrc.startOffset,  r2.w0
        add  r2.w0,                 r2.w0,                    s_cdeCmdWd.byteCount
        xout XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)

        mov  s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_COMPUTE
        mov  s_cdeCmdChk.byteLen,   s_cmdChkCrc.length
        mov  s_cdeCmdChk.initSum,   s_cmdChkCrc.initVal
        mov  s_cdeCmdChk.offset,    s_cmdChkCrc.resultOffset
        lsr  s_cdeCmdChk.options,   s_cmdChkCrc.flags,        7
        xout XID_CDECTRL,           s_cdeCmdChk,              SIZE(s_cdeCmdChk)


l_modComplete3:

    // CRC calculation
    qbbc  fci_modComplete5,  s_modState.flags.t_subsMCxtCrc

        lbco  s_cmdChkCrc,  PAMEM_CONST_MODIFY,  OFFSET_INSERT_CRC,  SIZE(s_cmdChkCrc)

        // Verify the range
        qble  l_modComplete4,  s_cmdChkCrc.startOffset,  r2.w0

            // Invalid CRC specified Discard the packet
            set s_statsFlags.event.t_nCommandFail

            mov  s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
            xout XID_CDECTRL,            s_cdeCmdPkt,          SIZE(s_cdeCmdPkt)

 // fix for conditional jump out of range
 l_mod_jmp_mainloop:
                 jmp   f_mainLoop


l_modComplete4:

        // Scroll to the start of the CRC
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_cdeCmdWd.byteCount,  s_cmdChkCrc.startOffset,  r2.w0
        add  r2.w0,                 r2.w0,                    s_cdeCmdWd.byteCount
        xout XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)

        mov  s_cdeCmdChk.operation,  CDE_CMD_CRC_COMPUTE
        mov  s_cdeCmdChk.byteLen,    s_cmdChkCrc.length
        mov  s_cdeCmdChk.initSum,    s_cmdChkCrc.initVal
        mov  s_cdeCmdChk.offset,     s_cmdChkCrc.resultOffset
        and  s_cdeCmdChk.options,    s_cmdChkCrc.flags,        PA_CRC_FLAGS_MASK
        xout XID_CDECTRL,            s_cdeCmdChk,              SIZE(s_cdeCmdChk)


fci_modComplete5:
    qbeq  l_modComplete8,  s_modState.patchCount,  0
    
    // Blind patch
    mov  r3.b0,   0                                         // Loop count
    mov  r3.b1,   OFFSET_BLIND_PATCH                        // Stub offset

    mov  r0.w0,   (PAMEM_BASE_BLIND_PATCH + 4) & 0xffff     // patch address
    mov  r0.w2,   (PAMEM_BASE_BLIND_PATCH + 4) >> 16

    // Need to update the patch address for PDSP5
    qbbc    l_modComplete6, s_modCxt.flags.t_isPdsp5        
    add r0.b1,  r0.b1,  1      // Adding 0x100 to r0.w0

l_modComplete6:

    qbge  l_modComplete8,  s_modState.patchCount,  r3.b0

        // read in the stub
        lbco  s_bPatchStub,  PAMEM_CONST_MODIFY,  r3.b1,   SIZE(s_bPatchStub)

        // Verify the offset is in range
        qble  l_modComplete7,  s_bPatchStub.patchOffset,   r2.w0

            // Invalid Patch. Discard the packet
            set s_statsFlags.event.t_nCommandFail

            mov  s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
            xout XID_CDECTRL,            s_cdeCmdPkt,          SIZE(s_cdeCmdPkt)
            jmp   f_mainLoop
    

l_modComplete7:

        // Advance to the patch point
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_cdeCmdWd.byteCount,  s_bPatchStub.patchOffset,  r2.w0
        add  r2.w0,                 r2.w0,                     s_cdeCmdWd.byteCount
        xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

        // Use r14.b0 as an advance flag. In the case of data overwrite the CDE is
        // not advanced after patch, but for data addition the CDE must be advanced
        // to keep all of the stored patch/checksum/crc offsets correct
        //mov r14.b0, 0

        // If the patch is a replace then the bytes must be deleted first
        qbbc l_modComplete7b, s_bPatchStub.cmdLen_insert.t_paBlindPatchOverwrite

            mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_FLUSH
            and  s_cdeCmdWd.byteCount, s_bPatchStub.cmdId_Len,   PA_BLIND_PATCH_LEN_MASK
            xout XID_CDECTRL,          s_cdeCmdWd,               SIZE(s_cdeCmdWd)
            //mov  r14.b0,               1
            add  r2.w0,                r2.w0,                    s_cdeCmdWd.byteCount
            //Adjust packet size
            sub  s_modState.pktSize,    s_modState.pktSize,      s_cdeCmdWd.byteCount


l_modComplete7b:
        qbbs l_modComplete7c, s_bPatchStub.cmdLen_insert.t_paBlindPatchDelete
        // Do the patch
        mov  s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
        and  s_cdeCmdInD.lenLsb,     s_bPatchStub.cmdId_Len,       PA_BLIND_PATCH_LEN_MASK
        mov  s_cdeCmdInD.lenMsbs,    0
        mov  s_cdeCmdInD.dataP,      r0
        xout XID_CDECTRL,            s_cdeCmdWd,                   SIZE(s_cdeCmdWd)
        //Adjust packet size
        add  s_modState.pktSize,    s_modState.pktSize,            s_cdeCmdInD.lenLsb
        

        // The packet actually advanced during the insert
        // Make no change to the tracking offset since the commands are all
        // relative to the original packet, not the patched packet

l_modComplete7c:

        // context for the next loop
        add  r3.b0,   r3.b0,   1        // Increment loop count
        add  r3.b1,   r3.b1,   32       // Increment offset
        add  r0,      r0,      32       // Increment absolute address
        jmp  l_modComplete6

l_modComplete8:

    ret


    .leave cdeScope
    .leave modifyScope
    .leave eQosCompScope    

    
// *******************************************************************************
// * FUNCTION PURPOSE: EMAC CRC Verify Command
// *******************************************************************************
// * DESCRIPTION: Process the EMAC CRC Verify 
// *
// *   Register Usage:  
// * 
// *   R0:    | CRC recalculated
// *   R1:    
// *   R2:      
// *   R3:      
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   
// *   R6:       | CRC stored    
// *   R7:        
// *   R8:        
// *   R9:          
// *   R10:         
// *   R11:         
// *   R12:         
// *   R13:         
// *   R14:       
// *   R15:                          
// *   R16:                          
// *   R17:       
// *   R18:       
// *   R19:       
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       
// *   R22:            | s_msg or s_emacCrcVerify 
// *   R23:              
// *   R24:               
// *   R25:              
// *   R26:               
// *   R27:              
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    .using cdeScope
    .using pktScope     // For offset info
    .using modifyScope
    
f_emacCrcVerify:
    
  // Flush out the control info (ignore all other commands)
  mov  s_cdeCmdWd.operation,   CDE_CMD_FLUSH_TO_PACKET
  xout XID_CDECTRL,            s_cdeCmdWd,            4

  // Check if the packet is a valid packet (at least has 4 bytes)
  qbgt l_emacCrcVerify_badCRC, s_modState.pktSize,    4
    //
    // Generate CRC and place it at packet description swinfo0 for comparison
    //
    mov  s_cdeCmd.v0,  CDE_CMD_CRC_COMPUTE | ((PA_CRC_FLAG_CRC_OFFSET_VALID | PA_CRC_FLAG_CRC_OFFSET_FROM_DESC)<< 8)  
    //mov  s_cdeCmdChk.operation,  CDE_CMD_CRC_COMPUTE
    //mov  s_cdeCmdChk.options,    PA_CRC_FLAG_CRC_OFFSET_VALID
    sub  s_cdeCmdChk.byteLen,    s_modState.pktSize,    4
    //  mov  s_cdeCmdChk.initSum,    0
    mov  s_cdeCmdChk.offset,     OFFSET(s_pktDescr.swinfo0)
    xout XID_CDECTRL,            s_cdeCmdChk,              SIZE(s_cdeCmdChk)
    
    //Advance the packet to the end of packet (keep the same length)
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    xout XID_CDECTRL,            s_cdeCmdWd,            4
    
    // Record the original CRC
    xin  XID_CDEDATA, r6, 4
    
    // Trim CRC (to the current window) from the packet and hold
    mov  s_cdeCmd.v0, CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_HOLD_PACKET | CDE_FLG_TRUNCATE)<<8)
    mov  s_cdeCmd.v1, (4<<8)
    xout XID_CDECTRL, s_cdeCmd, 8                  // Send the command  
    
    // Wait for the held packet to appear in the CDE
    wbs s_flags.info.tStatus_CDEHeldPacket
    
    // Wait for the CDE to be idle
    wbc s_flags.info.tStatus_CDEBusy
    
    // Get the CRC
    lbco    r0, cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0), 4
    
    // Verify CRC
    qbne    l_emacCrcVerify_badCRC, r0, r6
        // Forward the packet out
        // Patch the psflags
        // Note: There will be no errorFlags for host-generated packets, 
        //       therefor, the following code can be commented out
        //     
        //lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        //and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        //or   r2.b0, r2.b0,  s_emacCrcVerify.psflags
        //sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        sbco s_emacCrcVerify.psflags, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
    
        // Send the packet on its way
        mov  s_cdeCmd.v0,  CDE_CMD_HPKT_RELEASE | (CDE_FLG_SET_THREADID << 8) 
        mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
        xout  XID_CDECTRL,            s_cdeCmdPkt,            SIZE( s_cdeCmdPkt)
    
        jmp   f_mainLoop
    
l_emacCrcVerify_badCRC:
        // CRC is bad - Discard Packet
        mov     s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
        xout    XID_CDECTRL, s_cdeCmdPkt, 4              // Send the command 
        
        // Increment Tx CRC drop stats
        set   s_statsFlags.event.t_nTxEthCrcErr  
              
        jmp   f_mainLoop
    
    .leave cdeScope
    .leave pktScope
    .leave modifyScope
    
    
// ***********************************************************************************
// * FUNCTION PURPOSE: Setup the system to process a PA Configure message
// ***********************************************************************************
// * DESCRIPTION:  On entry the packet descriptor is in place. Where the packet
// *               context would go for a normal packet instead is the start
// *               of the paConfigure command. 
// *               
// *               The f_paConfigure function requires that the packet scratch
// *               field contain the packet ID, so this is put into place
// *               before calling that function.
// *
// **********************************************************************************

    .using pktScope
    .using modifyScope

f_paSetupConfigure:
#ifdef PASS_USE_PKTID
    mov  s_pktCxt.scratch, s_pktDescr.pktId
#endif    
    jmp  f_paConfigure




    .leave pktScope
    .leave modifyScope
    
// ***********************************************************************************
// * FUNCTION PURPOSE: Setup the system to process a PA Multi-route message
// ***********************************************************************************
// * DESCRIPTION:  On entry the packet descriptor is in place. Where the packet
// *               context would go for a normal packet instead is the start
// *               of the paConfigure command. 
// *               
// *               The f_paMultiFwd function requires the packet scratch
// *               field with the packet ID, and the return address. So put it into place
// *               before calling that function.
// *
// **********************************************************************************
    .using pktScope
    .using modifyScope
    
f_paSetupRxFwd:    
    mov  r30.w0,   f_mainLoop
    
    //Enhance to handle other commands 
    qbbc l_paSetupRxFwd_1, s_pktCxt.flag.t_flag_multi_route
    jmp  f_paMultiFwd
l_paSetupRxFwd_1:    
    qbbs l_paSetupRxFwd_2,  s_pktCxt.flag.t_flag_crc_verify    // CRC verification at DSP4 or Payload split at PDSP 5
    qbbs f_paRxCmdSet, s_pktCxt.flag.t_flag_cmdset
    
    // Illeagal flag, pass through to drop the packet
f_paSetupRxFwd_end:
    // drop the packet
    jmp fci_mProc5    
    
l_paSetupRxFwd_2:
    qbbs    f_paRxPayloadSplit, s_modCxt.flags.t_isPdsp5      
    jmp   f_paRxVerifyCRC
     

     .leave pktScope
     .leave modifyScope
// *********************************************************************************
// * FUNCTION PURPOSE: Rx Command Set Processing
// *********************************************************************************
// * DESCRIPTION: Process all the commands in the rxcommand set
// *
// *   On entry:
// *            - CDE points to the Control Info section
// *            - r30.w0 has the return address
// *            - s_pktCxt has valid packet context
// *
// *   On exit:
// *            - CDE points to the end of packets
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  Rx Command Header
// *   R7:          |
// *   R8:          | Rx Commands 
// *   R9:          |  
// *   R10:         |
// *   R11:         |
// *   R12:           |   RxCommand Conext
// *   R13:           |   
// *   R14:             |                                            
// *   R15:             | Packet Data                                           
// *   R16:             |                                            
// *   R17:             |  
// *   R18:       |  Packet ID
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

    .using cdeScope
    .using modifyScope
    .using pktScope
    
 f_paRxCmdSet:
    // Scroll past and flush the control info
    // mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    // xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    // Delete only the pktCxt from the control info
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    mov  s_cdeCmdWd.byteCount,  (SIZE(s_pktCxt) + 7) & 0xf8     // Round up to multiple of 8 bytes
    xout XID_CDECTRL,           s_cdeCmdWd,                 4
     
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
    xout  XID_CDECTRL,           s_cdeCmdWd,                4
 
    //  Clear the cmdset flag From the packet context, extract the command set index
    clr  s_pktCxt.flag.t_flag_cmdset
    mov  r0.b0,      s_pktCxt5.cmdSetIdx    
    
    // zero out the processing context
    zero &s_rxCmdCxt,    SIZE(s_rxCmdCxt)
    mov  s_rxCmdCxt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8      // 8-byte alignment
    mov  s_modState2.usrDataOffset, (SIZE(s_pktCxt) + 7) & 0xf8   // 8-byte alignment

    // Read the command set global comfiguration
    lbco s_rxCmdSetCfg,  PAMEM_CONST_CUSTOM,  OFFSET_CMDSET_CFG,      SIZE(s_rxCmdSetCfg)
    
    lsl  s_rxCmdCxt.cmdOffset,  r0.b0,             6            // Multiply by 64 to get the mem offset
    
    qbne l_paRxCmdSet_0, s_rxCmdSetCfg.cmdSetSize, 128
       lsl  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset,  1     // Multiply by 128 to get the mem offset
    
l_paRxCmdSet_0:
   
    // Load the command set header information 
    lbco s_rxCmdCxt, PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, 2
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(struct_paCommandSet)
   
    // discard the packet if command set does not match
    qbne l_paRxcmdSet_jmp_mProc5, s_rxCmdCxt.cmdSetIndex, r0.b0
    qblt l_paRxcmdSet_jmp_mProc5, s_rxCmdCxt.nCmds, 10
    jmp l_paRxcmdSet_no_jmp_mProc5
    l_paRxcmdSet_jmp_mProc5:
    jmp fci_mProc5
    l_paRxcmdSet_no_jmp_mProc5:
fci_paRxCmdSet_1:
    // number of command should never reach 0 without next route or multi-route
    qbeq l_paRxCmdSet_3, s_rxCmdCxt.nCmds, 0

    // Read in the command header
    lbco s_rxCmdHdr, PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdHdr)
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdHdr)

    // Search based on expected occurance of header type
    qbeq  f_rxNextRoute,        s_rxCmdHdr.cmd,  PA_RX_CMD_NEXT_ROUTE
    qbeq  f_rxCrcOp,            s_rxCmdHdr.cmd,  PA_RX_CMD_CRC_OP
    qbeq  f_rxCopyData,         s_rxCmdHdr.cmd,  PA_RX_CMD_COPY_DATA
    qbeq  f_rxPatchData,        s_rxCmdHdr.cmd,  PA_RX_CMD_PATCH_DATA
    qbeq  f_rxRmHhr,            s_rxCmdHdr.cmd,  PA_RX_CMD_REMOVE_HDR
    qbeq  f_rxRmTail,           s_rxCmdHdr.cmd,  PA_RX_CMD_REMOVE_TAIL
    qbeq  f_rxMultiRoute,       s_rxCmdHdr.cmd,  PA_RX_CMD_MULTI_ROUTE
    qbeq  f_rxUsrStats,         s_rxCmdHdr.cmd,  PA_RX_CMD_USR_STATS
    qbeq  f_rxVerifyPktErr,     s_rxCmdHdr.cmd,  PA_RX_CMD_VERIFY_PKT_ERROR 
    qbeq  f_rxSplitOp,          s_rxCmdHdr.cmd,  PA_RX_CMD_SPLIT

l_paRxCmdSet_2:
    // illegal command included, discard the packet
    jmp  fci_mProc5 
    
l_paRxCmdSet_3:
    // number of command should never reach 0 without next route or multi-route
    // forward the packet as if the next route command without multi-route or other special routes present
    mov s_rxCmdNextRoute.ctrlFlags, 0
    jmp fci_rxNextRoute0   
    
f_rxNextRoute:
    // Read in the nextRoute header
    lbco s_rxCmdNextRoute,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdNextRoute)
    //It is the last command to be processed. It is not necessary to update the command offset 
    //add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdNextRoute)
    
fci_rxNextRoute0:
    // TBD: Use usrDataOffset to handle more general operation
    //qbbs l_rxNextRoute2,    s_modState2.flags.t_rx_cmd_ctrl_no_ctx
    
        qbbc l_rxNextRoute1,    s_modState2.flags.t_rx_cmd_ctrl_no_hdr
            // The header has been removed, adjust offsets
            sub s_pktCxt.endOffset, s_pktCxt.endOffset, s_pktCxt.startOffset 
            mov s_pktCxt.startOffset, 0
    
l_rxNextRoute1:
            // update and copy pkt context
            // adjust the end offset with the number of insertion bytes
            mov   r1.w0,    0
            qbbc  l_rxNextRoute1_1, s_rxCmdCxt.updateLen.t7
                // negative number, 1 extension 
                mov r1.b1, 0xff
            
l_rxNextRoute1_1:            
            mov   r1.b0,    s_rxCmdCxt.updateLen  
            add   s_pktCxt.endOffset,  s_pktCxt.endOffset,    r1.w0
            and   s_pktCxt.paCmdId_Length, s_pktCxt.paCmdId_Length, 0xe0
            add   s_pktCxt.paCmdId_Length, s_pktCxt.paCmdId_Length, s_modState2.usrDataOffset
            sub   r0.b0,   s_modState2.usrDataOffset, 4
            wbs   s_flags.info.tStatus_CDEOutPacket
            sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  b0
        
l_rxNextRoute2:
    qbbs l_rxNextRoute4,    s_rxCmdNextRoute.ctrlFlags.t_rx_cmd_next_route_ctrl_emac_route

    // Prepare the packet command
    mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.psInfoSize,   s_rxCmdCxt.psInfoSize
    
    qbbs l_rxNextRoute3,    s_rxCmdNextRoute.ctrlFlags.t_rx_cmd_next_route_ctrl_multi_route
    
    // Note: We do not support payload split and multi-route within the same command set
    qbbc l_rxNextRoute2_1,  s_pktCxt.flag.t_flag_split
        // Store payload split context
        add  r0.b0,   s_modState2.usrDataOffset, SIZE(s_pktDescr) - 4 
        wbs  s_flags.info.tStatus_CDEOutPacket
        sbco s_modState2.hdrSize,  cCdeOutPkt, r0.b0, SIZE(s_rxCmdSplitCxt) 
        
        // forward packet to PDSP5
        mov  s_cdeCmdPkt.threadId,   PA_DEST_PA_M_1
        
        // pass through
    
l_rxNextRoute2_1:
    // Forward the packet
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)

    // Next route must be the end of rx command
    ret
    
l_rxNextRoute3:
    // Copy the packet and then perform multi-route
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
        
    // Debug code and patch  (new packet is ready)
    // Read the descriptor
    // xin  XID_CDEDATA,   s_pktDescr,   SIZE(s_pktDescr)
  
    // The context is read from the control section. Advance
    // to the control section
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_CONTROL
    xout  XID_CDECTRL,           s_cdeCmdWd,                  SIZE(s_cdeCmdWd)

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    mov s_cdeInsert.byteCount,  32
    xout XID_CDECTRL,           s_cdeCmdWd,             SIZE(s_cdeCmdWd)
   
    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    
    // store the multi-route index
    lsl   s_pktCxt2.eP1C2Id,   s_rxCmdNextRoute.multiRouteIndex, PKT2_EIDX_SHIFT    // set the multi route index
    jmp   fci_paMultiFwd1  // It does not return here
    
l_rxNextRoute4:

    qbbc l_rxNextRoute4_1,    s_rxCmdNextRoute.ctrlFlags.t_rx_cmd_next_route_ctrl_psflags_valid
        //patch the psFlags
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0, s_rxCmdNextRoute.psFlags
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
l_rxNextRoute4_1: 
    ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8)
    mov  s_cdeCmdPkt.psInfoSize,  0
    mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
    xout XID_CDECTRL,             s_cdeCmdPkt,              SIZE(s_cdeCmdPkt)
    ret
    
f_rxCrcOp: 
    // Scratch variables: r1.w0 CRC length
    //                    r1.b2 payload offset to the byte CRC calculation starts
    //                    r1.b3 frame type 
    
    // Read in the crcOP header
    lbco s_rxCmdCrcOp,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdCrcOp)
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdCrcOp)
    sub  s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    
    // Initialize CRC length to len, payload offset to 0 
    mov  r1.w0,     s_rxCmdCrcOp.len                           
    mov  r1.b2,     0
    
    // Adjust offset fields related to the parsed header
    add  s_rxCmdCrcOp.startOffset,   s_rxCmdCrcOp.startOffset, s_pktCxt.startOffset
    add  s_rxCmdCrcOp.crcOffset,     s_rxCmdCrcOp.crcOffset,   s_pktCxt.startOffset
    
    qbbc l_rxCrcOp1,        s_rxCmdCrcOp.ctrlFlags.t_rx_cmd_crc_op_ctrl_len_in_header 
    qbbc l_rxCrcOp_lenOffset_1,  s_rxCmdCrcOp.ctrlFlags.t_rx_cmd_crc_op_ctrl_len_offset_negative 
    qblt fci_paRxCmdSet_1, s_rxCmdCrcOp.lenOffset,   s_pktCxt.startOffset
        sub s_rxCmdCrcOp.lenOffset, s_pktCxt.startOffset, s_rxCmdCrcOp.lenOffset
        jmp l_rxCrcOp_lenOffset_2  
    
l_rxCrcOp_lenOffset_1:    
    add  s_rxCmdCrcOp.lenOffset,    s_rxCmdCrcOp.lenOffset,   s_pktCxt.startOffset
l_rxCrcOp_lenOffset_2:    

    // Extract and adjust the payload length from pktIn sideband data
    add    r0.w2,   s_rxCmdCrcOp.lenOffset,  (32+32)  
    // wait for the sideband data to be ready 
    wbc    s_flags.info.tStatus_CDEBusy
    lbco   r14.w2,  cCdeInPkt,  r0.w2,  2
    and    r1.w0,   r14.w2,     s_rxCmdCrcOp.lenMask
    sub    r1.w0,   r1.w0,      s_rxCmdCrcOp.lenAdjust
        
        
l_rxCrcOp1:       
    qbbc l_rxCrcOp2,        s_rxCmdCrcOp.ctrlFlags.t_rx_cmd_crc_op_ctrl_frame_type_included 
    
    //calculate the payload offset based on the frame type
    // FP HS-DSCH Type 2:
    // n = number of PDUs at b15:b11
    // payload offset = 6 + 2.5n (n: even)
    //                = 6 + 2.5n + 0.5 (n:odd)
    // FP HS-DSCH Type 3:
    // n = number of PDUs at b7:b3
    // payload offset = 4 + 2.5n (n: even)
    //                = 4 + 2.5n + 0.5 (n:odd)
    
    // Extract pkt Info from pktIn sideband data
    add   r0.w2,   s_rxCmdCrcOp.startOffset, (32+32)  
    // wait for the sideband data to be ready 
    wbc   s_flags.info.tStatus_CDEBusy
    lbco  r14,     cCdeInPkt,  r0.w2,  4
    and   r1.b3,   s_rxCmdCrcOp.ctrlFlags, PA_RX_CMD_CRC_OP_FRAME_TYPE_MASK
    qbne  l_rxCrcOp_type_hs_dsch_type3,  r1.b3,   PA_RX_CMD_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2
    
l_rxCrcOp_type_hs_dsch_type2: 
    lsr   r14.b1, r14.b1, 3
    mov   r1.b2,  6
    jmp   l_rxCrcOp_payload_offset

l_rxCrcOp_type_hs_dsch_type3: 
    lsr   r14.b1, r14.b0, 3
    mov   r1.b2,  4
    
l_rxCrcOp_payload_offset:
    lsl   r14.b2, r14.b1, 1         // 2n
    lsr   r14.b3, r14.b1, 1         // 0.5n
    add   r14.b2, r14.b2, r14.b3
    add   r1.b2,  r1.b2, r14.b2
    qbbc  l_rxCrcOp_payload_offset_2, r14.b1, 0
    add   r1.b2,  r1.b2, 1
    
l_rxCrcOp_payload_offset_2:    
    // adjust start offset
    add  s_rxCmdCrcOp.startOffset, s_rxCmdCrcOp.startOffset, r1.b2
    
    // pass through 
    
l_rxCrcOp2: 
    // Ignore the command if the current location passes over the CRC or message message offset
    qblt fci_paRxCmdSet_1,   s_rxCmdCxt.pktOffset, s_rxCmdCrcOp.startOffset
    
    // Scroll to the CRC or payload start offset
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_rxCmdCrcOp.startOffset,   s_rxCmdCxt.pktOffset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    mov  s_rxCmdCxt.pktOffset,  s_rxCmdCrcOp.startOffset
   
    zero &s_cdeCmdCrcChk,  SIZE(s_cdeCmdCrcChk) 
    qbbs l_rxCrcOp3 ,      s_rxCmdCrcOp.ctrlFlags.t_rx_cmd_crc_op_ctrl_crc_follow_payload
        // crc offset should be specified
        set s_cdeCmdCrcChk.options.t_cde_crc_cmd_option_crcOffset 
        sub  s_cdeCmdCrcChk.offset,     s_rxCmdCrcOp.crcOffset, s_rxCmdCrcOp.startOffset           
    
l_rxCrcOp3:    
    mov  s_cdeCmdCrcChk.operation,  CDE_CMD_CRC_VALIDATE
    sub  s_cdeCmdCrcChk.byteLen,    r1.w0,                  r1.b2
    xout XID_CDECTRL,               s_cdeCmdCrcChk,         SIZE(s_cdeCmdCrcChk)
    
    jmp  fci_paRxCmdSet_1
    
f_rxSplitOp: 
    // Scratch variables: r1.b2 payload offset to the byte CRC calculation starts
    
    // Read in the splitOp configuration
    lbco s_rxCmdSplitOp,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdSplitOp)
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdSplitOp)
    sub  s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    
    // Adjust offset fields related to the parsed header
    add  s_modState2.hdrSize,   s_rxCmdSplitOp.startOffset, s_pktCxt.startOffset
    
l_rxSplitOp1:       
    qbbc l_rxSplitOp2,        s_rxCmdSplitOp.ctrlFlags.t_rx_cmd_split_op_ctrl_frame_type_included 
    
    //calculate the payload offset based on the frame type
    // FP HS-DSCH Type 2:
    // n = number of PDUs at b15:b11
    // payload offset = 6 + 2.5n (n: even)
    //                = 6 + 2.5n + 0.5 (n:odd)
    // FP HS-DSCH Type 3:
    // n = number of PDUs at b7:b3
    // payload offset = 4 + 2.5n (n: even)
    //                = 4 + 2.5n + 0.5 (n:odd)
    
    // Extract pkt Info from pktIn sideband data
    add   r0.w2,   s_modState2.hdrSize, (32+32)  
    // wait for the sideband data to be ready 
    wbc   s_flags.info.tStatus_CDEBusy
    lbco  r14,     cCdeInPkt,  r0.w2,  4
    qbne  l_rxSplitOp_type_hs_dsch_type3,  s_rxCmdSplitOp.frameType,   PA_RX_CMD_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2
    
l_rxSplitOp_type_hs_dsch_type2: 
    lsr   r14.b1, r14.b1, 3
    mov   r1.b2,  6
    jmp   l_rxSplitOp_payload_offset

l_rxSplitOp_type_hs_dsch_type3: 
    lsr   r14.b1, r14.b0, 3
    mov   r1.b2,  4
    
l_rxSplitOp_payload_offset:
    lsl   r14.b2, r14.b1, 1         // 2n
    lsr   r14.b3, r14.b1, 1         // 0.5n
    add   r14.b2, r14.b2, r14.b3
    add   r1.b2,  r1.b2, r14.b2
    qbbc  l_rxSplitOp_payload_offset_2, r14.b1, 0
    add   r1.b2,  r1.b2, 1
    
l_rxSplitOp_payload_offset_2:    
    // adjust start offset
    add  s_modState2.hdrSize, s_modState2.hdrSize, r1.b2
    
    // pass through 
    
l_rxSplitOp2: 
    
    // Store the split context at the end of pktInfo and set the indication flag
    set  s_pktCxt.flag.t_flag_split
    qble l_rxSplitOp3,   s_modState2.usrDataOffset,  8
       // Need to reserve 8-byte for payload splitting operation
       mov  s_modState2.usrDataOffset, 8
         
l_rxSplitOp3:    
    // Record payload splitting parameters
    mov  s_modState2.destQueue, s_rxCmdSplitCxt.destQueue
    mov  s_modState2.flowId,    s_rxCmdSplitCxt.flowId
    jmp  fci_paRxCmdSet_1
             
f_rxCopyData:       
    // Read in the copy header
    lbco s_rxCmdCopy,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdCopy)
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdCopy)
    sub  s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1

    qbbs l_rxCopyData1, s_rxCmdCopy.ctrlFlags.t_rx_cmd_copy_ctrl_from_end
     
        // Adjust length fields related to the parsed header
        add  s_rxCmdCopy.srcOffset,   s_rxCmdCopy.srcOffset, s_pktCxt.startOffset
        jmp  l_rxCopyData2
        
l_rxCopyData1:
        sub  s_rxCmdCopy.srcOffset,   s_pktCxt.endOffset,    s_rxCmdCopy.numBytes
        
l_rxCopyData2:
    // Read copy data from pktIn sideband data
    add   r0.w2,   s_rxCmdCopy.srcOffset, (32+32)  
    // wait for the sideband data to be ready 
    wbc   s_flags.info.tStatus_CDEBusy
    lbco  r14,     cCdeInPkt,  r0.w2,  16
        
        // Ignore the command if the expected copy exceeds the limit
        add  r0.b0, s_rxCmdCopy.destOffset, s_rxCmdCopy.numBytes
        qblt fci_paRxCmdSet_1,  r0.b0, 32
        qble l_rxCopyData2_1, s_rxCmdCxt.psInfoSize, r0.b0  
            add s_rxCmdCxt.psInfoSize, r0.b0, 3
            and s_rxCmdCxt.psInfoSize, s_rxCmdCxt.psInfoSize, 0xfc 
        
l_rxCopyData2_1:
        // Special check if payload splitting is enabled
        qbbc  l_rxCopyData2_2, s_pktCxt.flag.t_flag_split
            // The first 8-byte is reserved for pktInfo and payload splitting context
            qbgt fci_paRxCmdSet_1, s_rxCmdCopy.destOffset, 8
            // pass through
                
l_rxCopyData2_2: 
            qble l_rxCopyData3,   s_rxCmdCopy.destOffset,  s_modState2.usrDataOffset
                and s_modState2.usrDataOffset, s_rxCmdCopy.destOffset, 0xfc
                //jmp l_rxCopyData3 
                // pass through
        
l_rxCopyData2_3:
        // TBD: Determine whether the packet context should be overwritten
        // qble l_rxCopyData3,         s_rxCmdCopy.destOffset,     SIZE(s_pktCxt)
        //    set s_modState2.flags.t_rx_cmd_ctrl_no_ctx 
        
l_rxCopyData3:
    // copy the data
    mov   r0.b0, s_rxCmdCopy.numBytes
    add   r0.b1, s_rxCmdCopy.destOffset, SIZE(s_pktDescr)      
    
    wbs   s_flags.info.tStatus_CDEOutPacket
    sbco  r14, cCdeOutPkt,  r0.b1,  b0
    jmp   fci_paRxCmdSet_1

f_rxPatchData:  
    // Scratch: r0.w2: patch data offset
    // Read in the patch header
    sub  r0.b0, s_rxCmdHdr.len, SIZE(s_rxCmdHdr)
    lbco s_rxCmdPatch,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, b0
    add  r0.w2,                 s_rxCmdCxt.cmdOffset, 4
    add  s_rxCmdCxt.cmdOffset,  s_rxCmdCxt.cmdOffset, r0.b0
    sub  s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    
    qbbc l_rxPatchData1,     s_rxCmdPatch.ctrlFlags.t_rx_cmd_patch_ctrl_mac_hdr  
    // MAC patch: replace the entire MAC header
    // Ignore the command if the packet offset is non-zero since MAC patch has to be the first command
    qbne fci_paRxCmdSet_1,   s_rxCmdCxt.pktOffset, 0
    
    //
    // Note: Make sure that the l3Offset points to the outer IP 
    // 
    qbne l_rxPatchData0, s_pktCxt.l3Offset, 0
        mov s_pktCxt.l3Offset, s_pktCxt.startOffset  
    
l_rxPatchData0:    
    // flush out the original MAC header
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    mov  s_cdeCmdWd.byteCount,  s_pktCxt.l3Offset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    mov  s_rxCmdCxt.pktOffset,  s_pktCxt.l3Offset 
    
    jmp  l_rxPatchData2
    
l_rxPatchData1:
    // Normal Patch opertaion    
    add  s_rxCmdPatch.offset,   s_rxCmdPatch.offset,  s_pktCxt.startOffset   
    // Ignore the command if the current location passes over the Patch start offset
    qblt fci_paRxCmdSet_1,   s_rxCmdCxt.pktOffset, s_rxCmdPatch.offset
    
    // Scroll to the Patch start offset
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_rxCmdPatch.offset,        s_rxCmdCxt.pktOffset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    mov  s_rxCmdCxt.pktOffset,  s_rxCmdPatch.offset
    
    qbbc l_rxPatchData1_1,   s_rxCmdPatch.ctrlFlags.t_rx_cmd_patch_ctrl_delete
        // Delete Data 
        mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_rxCmdPatch.numBytes
        xout XID_CDECTRL,           s_cdeCmdWd,             SIZE(s_cdeCmdWd)
        
        //
        // Adjust the hdrSize if some of the header portion is deleted
        //
        qbge  l_rxPatchData1_0, s_modState2.hdrSize, s_rxCmdCxt.pktOffset
            sub r0.b0,  s_modState2.hdrSize, s_rxCmdCxt.pktOffset
            qbge  l_rxPatchData1_00, r0.b0,  s_rxCmdPatch.numBytes
                sub  s_modState2.hdrSize, s_modState2.hdrSize,  s_rxCmdPatch.numBytes
                jmp  l_rxPatchData1_0
                
l_rxPatchData1_00:                
                sub  s_modState2.hdrSize, s_modState2.hdrSize,  r0.b0
                // pass through
        
l_rxPatchData1_0:        
        
        //adjust the pktOffset and length 
        add s_rxCmdCxt.pktOffset, s_rxCmdCxt.pktOffset,  s_rxCmdPatch.numBytes
        //sub s_pktCxt.endOffset,   s_pktCxt.endOffset,    s_rxCmdPatch.numBytes
        sub  s_rxCmdCxt.updateLen, s_rxCmdCxt.updateLen,  s_rxCmdPatch.numBytes  
        
        jmp  fci_paRxCmdSet_1     
    
l_rxPatchData1_1:    
    
    qbbs l_rxPatchData2,     s_rxCmdPatch.ctrlFlags.t_rx_cmd_patch_ctrl_insert 
        // Patch data
        mov  r0.b0,     s_rxCmdPatch.numBytes 
        lbco r14,       PAMEM_CONST_CMDSET_TABLE, r0.w2,  b0
        xout XID_CDEDATA,  r14,    b0
        jmp  fci_paRxCmdSet_1       
    
l_rxPatchData2:    
        // Insert command
        mov  s_cdeCmdInD.dataP.w2,   PAMEM_BASE_CMDSET_TABLE >> 16
        mov  s_cdeCmdInD.dataP.w0,   PAMEM_BASE_CMDSET_TABLE & 0xffff
        mov  s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
        mov  s_cdeCmdInD.lenLsb,     s_rxCmdPatch.numBytes
        mov  s_cdeCmdInD.lenMsbs,    0
        add  s_cdeCmdInD.dataP.w0,   s_cdeCmdInD.dataP.w0,   r0.w2
        xout XID_CDECTRL,            s_cdeCmdWd,             SIZE(s_cdeCmdInD)
        
    qbbc l_rxPatchData3,    s_rxCmdPatch.ctrlFlags.t_rx_cmd_patch_ctrl_mac_hdr  
        // Assume that there is no other command which will adjust the packet length if
        // MAC routing is required
        // Adjust the startOffset and the endOffset
        //add  s_pktCxt.endOffset,    s_pktCxt.endOffset,  s_rxCmdPatch.numBytes
        //sub  s_pktCxt.endOffset,    s_pktCxt.endOffset,  s_pktCxt.l3Offset
        sub s_rxCmdCxt.updateLen,   s_rxCmdPatch.numBytes, s_pktCxt.l3Offset     
        
        qbeq l_rxPatchData2_1,      s_pktCxt.l4Offset,   0
            add  s_pktCxt.l4Offset,     s_pktCxt.l4Offset,   s_rxCmdPatch.numBytes
            sub  s_pktCxt.l4Offset,     s_pktCxt.l4Offset,   s_pktCxt.l3Offset
        
l_rxPatchData2_1:
        qbeq l_rxPatchData2_2,      s_pktCxt.l5Offset,   0
            add  s_pktCxt.l5Offset,     s_pktCxt.l5Offset,   s_rxCmdPatch.numBytes
            sub  s_pktCxt.l5Offset,     s_pktCxt.l5Offset,   s_pktCxt.l3Offset
        
l_rxPatchData2_2:
        qbeq l_rxPatchData2_3,      s_pktCxt.espAhOffset,   0
            add  s_pktCxt.espAhOffset,  s_pktCxt.espAhOffset,   s_rxCmdPatch.numBytes
            sub  s_pktCxt.espAhOffset,  s_pktCxt.espAhOffset,   s_pktCxt.l3Offset
        
l_rxPatchData2_3:
        qbeq l_rxPatchData2_4,      s_pktCxt6.l3Offset2,   0
            add  s_pktCxt6.l3Offset2,   s_pktCxt6.l3Offset2, s_rxCmdPatch.numBytes
            sub  s_pktCxt6.l3Offset2,   s_pktCxt6.l3Offset2, s_pktCxt.l3Offset
        
l_rxPatchData2_4:
        add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_rxCmdPatch.numBytes
        sub  s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_pktCxt.l3Offset
            
        mov  s_pktCxt.l3Offset,     s_rxCmdPatch.numBytes
        jmp  fci_paRxCmdSet_1
        
l_rxPatchData3:        
        // 
        // We can not adjust the end packet offset until the packet is ready to be forwarded
        // Record the size of the data insertion here (insert size <= 16)
        // 
        //  
        //and  r1.b0,  s_rxCmdCxt.ctrl, PA_RX_CMD_INS_SIZE_MASK
        //add  r1.b0,  r1.b0, s_rxCmdPatch.numBytes
        //and  s_rxCmdCxt.ctrl, s_rxCmdCxt.ctrl,  NOT_PA_RX_CMD_INS_SIZE_MASK    
        //add  s_rxCmdCxt.ctrl, s_rxCmdCxt.ctrl,  r1.b0
        
        //
        // Adjust the hdrSize if data is inserted into the header 
        //
        qbgt  l_rxPatchData3_1, s_modState2.hdrSize, s_rxCmdCxt.pktOffset
            add  s_modState2.hdrSize, s_modState2.hdrSize,  s_rxCmdPatch.numBytes
            // pass through
        
l_rxPatchData3_1:        
        add  s_rxCmdCxt.updateLen, s_rxCmdCxt.updateLen,  s_rxCmdPatch.numBytes  
        jmp  fci_paRxCmdSet_1       
    
f_rxRmHhr:   
    sub     s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    // Ignore the command if pkt offset is not zero 
    qbne    fci_paRxCmdSet_1,      s_rxCmdCxt.pktOffset, 0
    mov     s_rxCmdCxt.pktOffset,  s_pktCxt.startOffset
    set     s_modState2.flags.t_rx_cmd_ctrl_no_hdr 
    
    // Scroll to the start of the first checksum
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    mov  s_cdeCmdWd.byteCount,  s_pktCxt.startOffset
    xout  XID_CDECTRL,          s_cdeCmdWd,         SIZE(s_cdeCmdWd)
    jmp  fci_paRxCmdSet_1       
       
f_rxRmTail:
    sub     s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    // Ignore the command if pkt offset exceeds the end of packet
    qblt    fci_paRxCmdSet_1,    s_rxCmdCxt.pktOffset,  s_pktCxt.endOffset
    
    // Scroll to the end of parsed packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_pktCxt.endOffset, s_rxCmdCxt.pktOffset
    xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
    mov  s_rxCmdCxt.pktOffset,  s_pktCxt.endOffset
    
    // Flush out the tail
    //mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_END
    // TBD: It is a temporary patch due to a simulator bug
    //      the original code should e be restored when the simulator fix is available
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
    sub  s_cdeCmdWd.byteCount,  s_pktDescr.pktDataSize, s_pktCxt.endOffset
    xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
    jmp  fci_paRxCmdSet_1
    
f_rxUsrStats:       
    sub  s_rxCmdCxt.nCmds,      s_rxCmdCxt.nCmds,   1
    
    // store the user statistics info 
    mov   s_rxUsrStatsReq.index,    s_rxCmdUsrStats.index
    mov   s_rxUsrStatsReq.pktSize,  s_pktCxt.endOffset
    
    // Call the common user-statistic update routine
    call  f_usrStatsUpdate

    jmp   fci_paRxCmdSet_1
       
f_rxMultiRoute:
    // store the multi-route index
    lsl   s_pktCxt2.eP1C2Id,   s_rxCmdMultiRoute.index, PKT2_EIDX_SHIFT    // set the multi route index
    jmp   fci_paMultiFwd1  // It does not return here
    
f_rxVerifyPktErr:
    // Read in the verifyPktErrCmd
    lbco s_rxCmdVerifyPktErr,  PAMEM_CONST_CMDSET_TABLE, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdVerifyPktErr)
    add  s_rxCmdCxt.cmdOffset, s_rxCmdCxt.cmdOffset, SIZE(s_rxCmdVerifyPktErr)
    sub  s_rxCmdCxt.nCmds,     s_rxCmdCxt.nCmds,   1
    
    // load the error flags
    lbco   r0.b0,  cCdeInPkt,  OFFSET(struct_pktDscFine.psFlags_errorFlags),  1
    and    r0.b0,  r0.b0,      s_rxCmdVerifyPktErr.errMask
    
    // Process the next command if no err
    qbeq   fci_paRxCmdSet_1,   r0.b0,   0
    
        // Forward the error packet accordingly  
        // Forwarding packets to the host
        qbne  l_rxVerifyPktErr_2,  s_rxCmdVerifyPktErr.forwardType,  PA_FORWARD_TYPE_HOST

        // Patch swinfo0
        wbs   s_flags.info.tStatus_CDEOutPacket
        sbco s_rxCmdVerifyPktErr.swInfo0,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        //qbbs l_rxVerifyPktErr_1_3, s_modState2.flags.t_rx_cmd_ctrl_no_ctx
    
        qbbc l_rxVerifyPktErr_1_1, s_modState2.flags.t_rx_cmd_ctrl_no_hdr
            // The header has been removed, adjust offsets
            sub s_pktCxt.endOffset, s_pktCxt.endOffset, s_pktCxt.startOffset 
            mov s_pktCxt.startOffset, 0
    
l_rxVerifyPktErr_1_1:
            // update and copy pkt context
            // adjust the end offset with the number of insertion bytes
            //and   r1.b0,  s_rxCmdCxt.ctrl, PA_RX_CMD_INS_SIZE_MASK
            //add   s_pktCxt.endOffset,  s_pktCxt.endOffset,    r1.b0
            mov   r1.w0,    0
            qbbc  l_rxVerifyPktErr_1_2, s_rxCmdCxt.updateLen.t7
                // negative number, 1 extension 
                mov r1.b1, 0xff
            
l_rxVerifyPktErr_1_2:            
            mov   r1.b0,    s_rxCmdCxt.updateLen  
            add   s_pktCxt.endOffset,  s_pktCxt.endOffset,    r1.w0
            
            //wbs   s_flags.info.tStatus_CDEOutPacket (wait above)
            sub   r0.b0, s_modState2.usrDataOffset, 4
            sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  b0
        
l_rxVerifyPktErr_1_3:
l_rxVerifyPktErr_1_3_queue_bounce:
        // Check for Queue Bounce operation
l_rxVerifyPktErr_1_3_queue_bounce_ddr:
        qbbc l_rxVerifyPktErr_1_3_queue_bounce_msmc, s_rxCmdVerifyPktErr.queue.t_pa_forward_queue_bounce_ddr
            clr s_rxCmdVerifyPktErr.queue.t_pa_forward_queue_bounce_ddr
            sbco s_rxCmdVerifyPktErr.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_rxCmdVerifyPktErr.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_rxVerifyPktErr_1_3_queue_bounce_end

l_rxVerifyPktErr_1_3_queue_bounce_msmc:
        qbbc l_rxVerifyPktErr_1_3_queue_bounce_end, s_rxCmdVerifyPktErr.queue.t_pa_forward_queue_bounce_msmc
            clr s_rxCmdVerifyPktErr.queue.t_pa_forward_queue_bounce_msmc
            sbco s_rxCmdVerifyPktErr.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_rxCmdVerifyPktErr.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through
l_rxVerifyPktErr_1_3_queue_bounce_end:

            // Send the packet on its way
            ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,   s_rxCmdCxt.psInfoSize
            mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            mov  s_cdeCmdPkt.destQueue,   s_rxCmdVerifyPktErr.queue
            mov  s_cdeCmdPkt.flowId,      s_rxCmdVerifyPktErr.flowId
            xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
            ret
    
l_rxVerifyPktErr_2:
        // Only supported forward types: Host, Discard 
        // Discard the packet 
        mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_FLUSH
        xout  XID_CDECTRL,           s_cdeCmdPkt,                 SIZE(s_cdeCmdPkt)
        ret

.leave cdeScope  
.leave modifyScope
.leave pktScope

// *********************************************************************************
// * FUNCTION PURPOSE: Rx CRC Verification Processing
// *********************************************************************************
// * DESCRIPTION: Verify the CRC calculation result with pre-stored data
// *
// *   On entry:
// *            - CDE points to the Control Info section
// *            - r30.w0 has the return address
// *            - s_pktCxt has valid packet context
// *
// *   On exit:
// *            - packet is forwarded
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:          |   
// *   R7:          |
// *   R8:          | Not used 
// *   R9:          |  
// *   R10:         |
// *   R11:           |  CRC context
// *   R12:           |   
// *   R13:              
// *   R14:             |                                            
// *   R15:             | Packet Data                                           
// *   R16:             |                                            
// *   R17:             |  
// *   R18:       |  Packet ID (not used)
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

.using cdeScope
.using modifyScope
.using pktScope

 f_paRxVerifyCRC:
    // Read in the CRC context 
    xin  XID_CDEDATA,  s_rxCrcCxt, SIZE(s_rxCrcCxt)
 
    // Scroll past and flush the control info
    //mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    //xout  XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)
    
    // Scroll past and flush the packet info (ling info only)
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    mov  s_cdeCmdWd.byteCount,  32     
    xout XID_CDECTRL,           s_cdeCmdWd,                 4
    
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
    xout  XID_CDECTRL,           s_cdeCmdWd,                4
 
    // Scroll to the CRC location in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_rxCrcCxt.offset
    xout XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    
    // Read and compare the CRC
    xin  XID_CDEDATA,  r14,     4
    
    qbeq l_paRxVerifyCRC1,      s_rxCrcCxt.crc,     r14
        // CRC is incorrect
        mov r14, s_rxCrcCxt.crc
        xout  XID_CDEDATA,  r14,     4
        
        // Set the CRC error flag
        lbco  r1.b0,  cCdeOutPkt,  OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
        set   r1.b0.t_pkt_desc_err_flag_chksum2
        sbco  r1.b0,  cCdeOutPkt,  OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
    
l_paRxVerifyCRC1:
    // Update and copy the pkt context
    clr   s_pktCxt.flag.t_flag_crc_verify  
    sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

    // forward the packet out
    mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
    
    ret
    
.leave cdeScope  
.leave modifyScope
.leave pktScope

// *********************************************************************************
// * FUNCTION PURPOSE: Rx Payload Splitting Processing
// *********************************************************************************
// * DESCRIPTION: Split packet into header and payload portion and deliver them to
// *              different queue with different flow
// *
// *   On entry:
// *            - CDE points to the Control Info section
// *            - r30.w0 has the return address
// *            - s_pktCxt has valid packet context
// *
// *   On exit:
// *            - packet is forwarded
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:          |   
// *   R7:          |
// *   R8:          | Not used 
// *   R9:          |  
// *   R10:         |
// *   R11:           |  Payload split context
// *   R12:              
// *   R13:              
// *   R14:             |                                            
// *   R15:             | Packet Data                                           
// *   R16:             |                                            
// *   R17:             |  
// *   R18:       |  Packet ID (not used)
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  Packet ID
// *   R27:  statistics  (s_statsFlags)                             -  Global scope
// *   R28:  event (INTD) enabled mask register  (s_eventFlags)     -
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

.using cdeScope
.using modifyScope
.using pktScope

 f_paRxPayloadSplit:
    // Read in the packet split context 
    and r0.b0, s_pktCxt.paCmdId_Length, 0x1f
    add r0.b0, r0.b0, SIZE(s_pktDescr) - 4
    wbc   s_flags.info.tStatus_CDEBusy
    lbco  s_rxSplitCxt,  cCdeInPkt,  r0.b0,  SIZE(s_rxSplitCxt)
 
    // Pass through the control info (note: The new psInfo section has been inserted already and should be ignored) 
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)
 
    // Scroll to the payload location in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_rxSplitCxt.hdrSize
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)
    
    // Flush out the payload portion
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,           s_cdeCmdWd,         4
    
l_paRxPayloadSplit1:
    // Copy the pkt context
    //wbs   s_flags.info.tStatus_CDEOutPacket
    //sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

    // forward the packet out
    // TBD: instruction can be simplified
    mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.psInfoSize,   0                    
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
    xout XID_CDECTRL,              s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
    
l_paRxPayloadSplit2:
    wbs  s_flags.info.tStatus_CDENewPacket
    
    // Move to the packet                   
    mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
    xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)
    
    // Flush the packet header
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    mov  s_cdeCmdWd.byteCount,  s_rxSplitCxt.hdrSize
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)
    
l_paRxPayloadSplit2_queue_bounce:
    // Move to the end of the packet
    mov     s_cdeCmd.v0.w0, CDE_CMD_ADVANCE_TO_END
    xout    XID_CDECTRL, s_cdeCmd, 4              // Send the command

    // Check for Queue Bounce operation
    wbs s_flags.info.tStatus_CDEOutPacket
l_paRxPayloadSplit2_queue_bounce_ddr:
    qbbc l_paRxPayloadSplit2_queue_bounce_msmc, s_rxSplitCxt.destQueue.t_pa_forward_queue_bounce_ddr
        clr s_rxSplitCxt.destQueue.t_pa_forward_queue_bounce_ddr
        sbco s_rxSplitCxt.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
        lbco s_rxSplitCxt.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
        jmp  l_paRxPayloadSplit2_queue_bounce_end

l_paRxPayloadSplit2_queue_bounce_msmc:
    qbbc l_paRxPayloadSplit2_queue_bounce_end, s_rxSplitCxt.destQueue.t_pa_forward_queue_bounce_msmc
        clr s_rxSplitCxt.destQueue.t_pa_forward_queue_bounce_msmc
        sbco s_rxSplitCxt.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
        lbco s_rxSplitCxt.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
        // pass through
l_paRxPayloadSplit2_queue_bounce_end:

    // Forward the packet out
    zero &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE
    mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
    mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
    mov  s_cdeCmdPkt.destQueue,   s_rxSplitCxt.destQueue
    mov  s_cdeCmdPkt.flowId,      s_rxSplitCxt.flowId
    xout XID_CDECTRL,             s_cdeCmdPkt,       SIZE(s_cdeCmdPkt)
    
    ret
    
.leave cdeScope  
.leave modifyScope
.leave pktScope


// *********************************************************************************
// * FUNCTION PURPOSE: Update User Statistics
// *********************************************************************************
// * DESCRIPTION: Process the user-statistics update from command set and user-statistics
// *              FIFO
// *
// *   On entry:
// *            - s_pktCxt has valid packet context
// *
// *   On exit:
// *            - CDE does not move
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    
// *   R2:      
// *   R3:  
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  Rx Command Header  (User-Statistics command ==> can be re-used
// *   R7:        |  user Stats update context
// *   R8:        |  
// *   R9:        |  
// *   R10:         |  Usr Stats Global Configuration
// *   R11:           
// *   R12:           |   RxCommand Conext  (reserved, do not touch)
// *   R13:           |   
// *   R14:             |  Rx user Stats FIFO context (reserved)                                           
// *   R15:             |  Rx user stats FIFO control block (reserved)                                          
// *   R16:             |  Rx user Stats FIFO request (used)                                          
// *   R17:             |  
// *   R18:       |  Packet ID  (reserved, do not touch)
// *   R19:          |
// *   R20:       |  Modify State machine (s_modState)  (reserved, do not touch)
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -  Global scope
// *   R29:  Modify context (s_modCxt)                              -
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

.using cdeScope
.using modifyScope
.using pktScope

f_usrStatsUpdate:
    lbco    s_paUsrStatsCfg, PAMEM_CONST_CUSTOM, OFFSET_USR_STATS_CFG, SIZE(s_paUsrStatsCfg)
    lsl     s_rxUsrStatsUpdateCxt.cnt32Offset, s_paUsrStatsCfg.num64bCounters,  3

l_usrStatsUpdates_0:

    lsl     s_rxUsrStatsUpdateCxt.lnkOffset, s_rxUsrStatsReq.index,   1
    lbco    s_rxUsrStatsUpdateCxt.lnkIndex,  PAMEM_CONST_USR_STATS_CB, s_rxUsrStatsUpdateCxt.lnkOffset, SIZE(s_rxUsrStatsUpdateCxt.lnkIndex)
    
    qbbs    l_usrStatsUpdates_4,  s_rxUsrStatsUpdateCxt.lnkIndex.t_pa_usr_stats_cb_disable  
    qble    l_usrStatsUpdates_2,  s_rxUsrStatsReq.index, s_paUsrStatsCfg.num64bCounters
    
l_usrStatsUpdates_1:
        // 64-bit counter    
        lsl     s_rxUsrStatsUpdateCxt.cntOffset, s_rxUsrStatsReq.index,  3  
        lbco    s_rxUsrStatsUpdateCxt.data1,  PAMEM_CONST_USR_STATS_COUNTERS,  s_rxUsrStatsUpdateCxt.cntOffset, 8
        
        qbbs    l_usrStatsUpdates_1_1,    s_rxUsrStatsUpdateCxt.lnkIndex.t_pa_usr_stats_cb_byte_cnt
            // packet Counter
            add     s_rxUsrStatsUpdateCxt.data2,  s_rxUsrStatsUpdateCxt.data2, 1
            jmp     l_usrStatsUpdates_1_2
             
   l_usrStatsUpdates_1_1:
            // byte Counter
            add     s_rxUsrStatsUpdateCxt.data2,  s_rxUsrStatsUpdateCxt.data2, s_rxUsrStatsReq.pktSize
           
           // Common operation
   l_usrStatsUpdates_1_2:  
            adc     s_rxUsrStatsUpdateCxt.data1,  s_rxUsrStatsUpdateCxt.data1, 0
            sbco    s_rxUsrStatsUpdateCxt.data1,  PAMEM_CONST_USR_STATS_COUNTERS,  s_rxUsrStatsUpdateCxt.cntOffset, 8
            jmp     l_usrStatsUpdates_3
            
l_usrStatsUpdates_2:
        // 32-bit counter  
        sub     r1.w0,  s_rxUsrStatsReq.index,  s_paUsrStatsCfg.num64bCounters  
        lsl     s_rxUsrStatsUpdateCxt.cntOffset, r1.w0,  2
        add     s_rxUsrStatsUpdateCxt.cntOffset, s_rxUsrStatsUpdateCxt.cntOffset, s_rxUsrStatsUpdateCxt.cnt32Offset  
        lbco    s_rxUsrStatsUpdateCxt.data1,  PAMEM_CONST_USR_STATS_COUNTERS,  s_rxUsrStatsUpdateCxt.cntOffset, 4
        
        qbbs    l_usrStatsUpdates_2_1,    s_rxUsrStatsUpdateCxt.lnkIndex.t_pa_usr_stats_cb_byte_cnt
            // packet Counter
            add     s_rxUsrStatsUpdateCxt.data1,  s_rxUsrStatsUpdateCxt.data1, 1
            jmp     l_usrStatsUpdates_2_2
             
   l_usrStatsUpdates_2_1:
            // byte Counter
            add     s_rxUsrStatsUpdateCxt.data1,  s_rxUsrStatsUpdateCxt.data1, s_rxUsrStatsReq.pktSize
           
           // Common operation
   l_usrStatsUpdates_2_2:  
            sbco    s_rxUsrStatsUpdateCxt.data1,  PAMEM_CONST_USR_STATS_COUNTERS,  s_rxUsrStatsUpdateCxt.cntOffset, 4
            
l_usrStatsUpdates_3:
        qbbs    l_usrStatsUpdates_4, s_rxUsrStatsUpdateCxt.lnkIndex.t_pa_usr_stats_cb_no_lnk  
            // prepare for next layer
            mov   s_rxUsrStatsReq.index,    s_rxUsrStatsUpdateCxt.lnkIndex
            and   s_rxUsrStatsReq.index.b1, s_rxUsrStatsReq.index.b1, PA_USR_STATS_LNK_MASK_MSB
            jmp   l_usrStatsUpdates_0           

l_usrStatsUpdates_4:
        ret

.leave cdeScope  
.leave modifyScope
.leave pktScope


#include "pacfgcmn.p"


// ****************************************************************************
// * FUNCTION PURPOSE: Handle commands that are invalid for the modify pdsp
// ****************************************************************************
// * DESCRIPTION: Commands which configure LUTs are invalid
// *
// ****************************************************************************

  .using configScope
  .using cdeScope
  
f_paComAddRepLut2:
    // Process Queue Divert response
    // Read in the command info to check the control flag. 
    xin  XID_CDEDATA,  s_paAddL2Std,  SIZE(s_paAddL2Std)
      
    qbbc  l_paComAddRepLut2_1,  s_paAddL2Std.ctrlBitMap.t_pa_lut2_ctrl_queue_divert 
    
        // Clear the queue divert flag at PDSP3 mailbox register
        mov     r1, 0
        sbbo    r1, r1, 0x34, 4 
        jmp     f_cfgReply
  
l_paComAddRepLut2_1:
f_paComAddRepLut1:
f_paComDelLut1:
f_paComDelLut2:

    mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_DESTINATION
    xout XID_CDEDATA,            s_paCmd1.commandResult,                   SIZE(s_paCmd1.commandResult)
    jmp  f_cfgReply 


    .leave configScope
    .leave cdeScope
    
#ifdef TO_BE_DELETE    

// *********************************************************************************
// * FUNCTION PURPOSE: Initialize the Modify PDSPs
// *********************************************************************************
// * DESCRIPTION: Global setup is done
// *
// *********************************************************************************

f_mLocalInit:

    // clear event flag by default

    ret

#endif    
    
// *****************************************************************************************
// * FUNCTION PURPOSE: Hold the command reply
// *****************************************************************************************
// * DESCRIPTION: Forwards the reply only if the destination is the host
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        |  LUT1 View1   - lut1Scope
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                          
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View2  - lut1Scope                 
// *   R18:          |
// *   R19:          |
// *   R20:          |
// *   R21:          |
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |  b3: ctrlFlag, b2:command result offset < 256: w0: pktId
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *****************************************************************************************/
// dumy function
f_cfgReplyHoldPkt:
        ret

    .leave globalScopeM





