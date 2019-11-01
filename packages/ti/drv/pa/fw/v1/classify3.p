// **************************************************************************************************************
// * FILE PURPOSE: Peform packet classification on PDSPS with a Flow Cache LUT1
// **************************************************************************************************************
// * FILE NAME: classify3.p
// *
// * DESCRIPTION: The PDSP code for L3/L4 classification using a LUT1 for Flow Cache classification
// *
// **************************************************************************************************************
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
#define PASS_CLASSIFY3  
#include "pdsp_pa.h"
#include "pdsp_mem.h"
#include "pdsp_mem2.h"
#include "pdsp_subs.h"
#include "pm_config.h"
#include "pdsp_ver.h"
#include "pm_constants.h"
#include "parsescope.h"

// TBD: move to top layer file per PDSP image
#define PASS_C1_VER    PASS_VERSION                  // 0x01.0x00.0x00.0x03

    .origin      0
    .entrypoint  f_c1Init

f_c1Init:
    jmp f_c1Start
    
header:
    .codeword  HEADER_MAGIC
    .codeword  PASS_C1_VER

    .using   globalScopeC1

f_c1Start:
    call   f_c1LocalInit
    
    // Store the version number
    mov   r2.w0,   PASS_C1_VER & 0xFFFF
    mov   r2.w2,   PASS_C1_VER >> 16 
    sbco  r2,   PAMEM_CONST_PDSP_INFO,  OFFSET_VER,     4 
    
    // Clear the mailbox 
    zero &r2, 12
    sbco r2, cMailbox, 4, 12 

    // Write a non-zero value to mailbox slot 0. 
    mov   r2, 1
    sbco  r2, cMailbox, 0, 4

	// Wait for the set command to take hold
    wbs  s_flags.info.tStatus_Command0

    // The host will clear mailbox slot 0 when it is ready for this PDSP to run
    wbc  s_flags.info.tStatus_Command0

    // Do common initialization if the host has requested it
    // The host requests a global init by writing a non-zero value into mailbox slot 1
    // (Before clearing mailbox slot 0)
    //qbbc  l_c1Start0,  s_flags.info.tStatus_Command1

// Note: The first PDSP at each stage should perform global init
#ifdef PASS_GLOBAL_INIT    
      call f_commonInit
#endif      

l_c1Start0:

    zero  &s_runCxt,      SIZE(s_runCxt)
    // Store the PDSP ID
    mov   r2.b0, PASS_PDSP_ID
    sbco  r2.b0, PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1

// ****************************************************************************************************
// * FUNCTION PURPOSE: The main processing loop
// ****************************************************************************************************
// * DESCRIPTION: The packet processing state machine
// *
// *        Register usage:
// *
// *   R0:
// *   R1:    scratch
// *   R2:
// *   R3:
// *   R4:    LUT1 Status (s_l1Status) / LUT1 Command (s_l1Cmd)  - lut1Scope
// *   R5:    
// *   R6:        |  
// *   R7:        |  
// *   R8:        |  
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     
// *   R11:       |                                     
// *   R12:       |                                     
// *   R13:       |                                     
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  
// *   R19:          |                                  
// *   R20:          |                                  
// *   R21:          |                                  
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************************


   .using  lut1Scope
   .using  pktScope
   .using  cdeScope    

f_mainLoop:

#ifdef PASS_PROC_PKT_CONTROL    

   // Look for command (FIRMWARE_CMD_PKT_CTRL_CFG only) 
    qbbc l_mainLoop001, s_flags.info.tStatus_Command3
        // Process Packet Verification Configuration Update
	    lbco  r1, cMailbox, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        // set the packet control flags
        and s_runCxt.flag2, s_runCxt.flag2, NOT_SUBS_PKT_CTRL_MASK
        or  s_runCxt.flag2, s_runCxt.flag2, r1.b0          
            
        // clear the command flag
        mov   r1, 0           
	    sbco  r1, cMailbox, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        // pass through       
#endif

l_mainLoop001:

   // Look for held packets and completed lookup
   qbbc  l_mainLoop5, s_runCxt.flags.t_previous_held

       qbbc  l_mainLoop00, s_runCxt.flags.t_held_packet
       
       // There is already a packet pending to submit, 
       // We need to wait for the lookup to be complete 
            wbc  s_flags.info.tStatus_Lut1Busy

l_mainLoop00:
           qbbs l_mainLoop5,  s_flags.info.tStatus_Lut1Busy

               // A lookup is complete for the held packet
               xin   XID_LUT1CMD,       s_l1Status,     SIZE(s_l1Status)
               
               // Common operation regredless of the lookup result
               // Pop the next extended info and write it out
               xin   XID_PINFO_B_POP,   s_pktExtDescr,  SIZE(s_pktExtDescr)
               
               // Wait for the held packet to appear in the CDE
               // (Its almost certainly already there)
               wbs   s_flags.info.tStatus_CDEHeldPacket
               
               // Load the first 16-byte to pktInfo
               lbco  s_txPktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    OFFSET(s_txPktCxt.fragSize)
               
               qbbc  l_mainLoop0,   s_l1Status.status.L1_STATUS_ENTRY_MATCH
                    call  f_c1ForwardHeldFCPktMatch
                    jmp   l_mainLoop1

l_mainLoop0:   
               call  f_c1ForwardHeldFCPktNoMatch
l_mainLoop1:
               clr   s_runCxt.flags.t_previous_held
               qbbc  l_mainLoop5, s_runCxt.flags.t_held_packet

                   // An already parsed packet is waiting to be submitted to the LUT
                   clr   s_runCxt.flags.t_held_packet
                   qbbs  l_mainLoop1_1, s_runCxt.flags.t_previous_skip
                        // Initiate Lookup for pending packet
                        // TBD: Need to move packet to B and set previous held
                        set   s_runCxt.flags.t_previous_held
                        mov   s_l1Cmd.operation,             LUT1_CMD_SEARCH
                        xout  XID_LUT1CMD,                   s_l1Cmd,              4
                        // Wait for LUT1 Busy
                        wbs  s_flags.info.tStatus_Lut1Busy
                        jmp   l_mainLoop5 
                        
l_mainLoop1_1:
                    // No lookup is required
                    clr  s_runCxt.flags.t_previous_skip
                    // Pop the next extended info and write it out
                    xin     XID_PINFO_B_POP, s_pktExtDescr, SIZE(s_pktExtDescr)
                        
                    // Forward packet now    
                    //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_RELEASE 
                    ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_RELEASE 
                    
#ifdef PASS_LAST_PDSP                        
                    // TBD: clear Bypass Flag
                    lbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
                    clr   r1.w0.t_pktBypass
                    sbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
                    qbbc    l_mainLoop1_2,  s_pktExtDescr.flags.fDroppedInd 
                        set   s_pktExtDescr.flags.fDropped
                        mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
l_mainLoop1_2:                    
#endif               
                    xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
                    xout XID_CDECTRL,    s_cdeCmdPkt,    4
                    jmp  l_mainLoop2
                        
l_mainLoop2:
                   jmp  l_mainLoop5

        // if ( (pendingConfig == FALSE) && (newPacket == TRUE) ) then parse
l_mainLoop5:
    qbbs  fci_mainLoop6, s_runCxt.flags.t_pendingConfig
        qbbs f_c1Parse,  s_flags.info.tStatus_CDENewPacket
            // jmp fci_mainLoop7

        // if (  (pendingConfig == TRUE) && (Lut1Busy == FALSE) ) then complete modify
fci_mainLoop6:
    qbbc  f_mainLoop,  s_runCxt.flags.t_pendingConfig
        qbbc  l_mainLoop8,  s_flags.info.tStatus_Lut1Busy


fci_mainLoop7:
        jmp    f_mainLoop

     
l_mainLoop8:
    // Relative jump was out of range
    jmp f_paComLut1CompleteModify      // no return

    //jmp f_mainLoop
   
   .leave  lut1Scope
   .leave  pktScope
   .leave  cdeScope  
   
// **************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that had a match in the Flow Cache LUT1 search
// **************************************************************************************************
// * DESCRIPTION:  On entry r5 contains the LUT1 status, which indicated a match for the
// *               held packet, and the match index value. The packet is forwarded as required.
// *    
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    scratch
// *   R2:
// *   R3:
// *   R4:
// *   R5:    LUT1 Status (s_l1Status) / LUT1 Command (s_l1Cmd)  - lut1Scope
// *   R6:
// *   R7:
// *   R8:   LUT1 info (s_l1fFc)                                     -
// *   R9:      |                                                    -
// *   R10:     |  Forward match Info (valid at function exit)       -  lut1MatchSCope
// *   R11:     |                                                    -
// *   R12:     |                                                    -
// *   R13:
// *   R14:          |                                          
// *   R15:          |  Extended Header Info                                        
// *   R16:          |                                          
// *   R17:          |  
// *   R18:          |
// *   R19:            | usrStats FIFO CB
// *   R20:            | usrStats Request
// *   R21:
// *   R22:   |
// *   R23:   |
// *   R24:   |  Packet context  
// *   R25:   |       
// *   R26:   |  
// *   R27:   |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// **************************************************************************************************/
    .using  lut1Scope
    .using  lut1MatchScope
    .using  pktScope        // Used only for sizing
    .using  cdeScope

f_c1ForwardHeldFCPktMatch:

    //Record the match (new system statistics) 
    //mov    s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_MATCH 

    // Load the routing info associated with LUT1 
    // This is the l1Info and match routing information
    lsl    r1.w0,       s_l1Status.index,             5                                  // l1 index * 16
    lbco   &s_l1fFc,    PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,     SIZE(s_l1fFc)+SIZE(s_fmPlace)  // l1Info + routing info
    
    //  Update Flow Cache related statistics
    mov  s_stats.value,  s_l1fFc.statsCmd  // Update the packet counter
    qble fci_stdHeldFCPktForward, s_l1fFc.uCnt, PAFRM_FC_USAGE_CNT_MAX
        add  s_l1fFc.uCnt, s_l1fFc.uCnt, 1
        sbco s_l1fFc, PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,     SIZE(s_l1fFc)   
        // pass through
    
fci_stdHeldFCPktForward:    

    // Only Egress flow is supported
    qbne  l_stdHeldFCPktForward1,  s_matchForward.forwardType,  PA_FORWARD_TYPE_EFLOW 
        // TBD: may want to add more error check
        // TBD: other cleanup?
        lsl   s_txPktCxt.flags.b0, s_matchForward_ef.validBitMap,  4
        mvid  *&s_txPktCxt.lvl1RecIndex, *&s_matchForward_ef.lvl1RecIndex 
        sbco  s_txPktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    OFFSET(s_txPktCxt.fragSize)
        
#ifndef PASS_LAST_PDSP
        // clear the bypass bit since the packet should be processed by the next PDSP
        lbco  r1.w0,  cCdeHeldPkt,  OFFSET(s_pktDescr.pktflags), 2
        clr   r1.w0.t_pktBypass
        sbco  r1.w0,  cCdeHeldPkt,  OFFSET(s_pktDescr.pktflags), 2
#endif
        //  Send the packet on its way
        //  Free the packet Ext Info
        // mov  s_pktExtDescr.threadId, s_matchForward_pa.dest

        //   Load flags and operation in one instruction
        ldi  r4, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_txPktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
        
        xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
        xout  XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
        ret

fci_stdHeldFCPktForwardErr:   // A jump in point for other functions to use
                              // the system error info
        // Record Error and then pass through                  

fci_stdHeldFCPktDrop:
l_stdHeldFCPktForward1: 
        //   Free the packet Ext Info
#ifdef PASS_LAST_PDSP
        //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
        ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_DISCARD 
        set   s_pktExtDescr.flags.fDropped
#else
        ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_RELEASE 
        //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_RELEASE
        //mov   s_cdeCmdPkt.optionsFlag, 0
        set  s_pktExtDescr.flags.fDroppedInd
#endif        
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        xout  XID_CDECTRL,  s_cdeCmdPkt,   4
        ret

    .leave pktScope        
    .leave lut1MatchScope
    .leave lut1Scope
    .leave  cdeScope
    
// ******************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that failed a Flow Cache LUT1 search
// ******************************************************************************************************
// * DESCRIPTION: The packet is forwarded based on firewall default setting 
// *
// *              This function exits through a call to stdHeldFirewallPktForward, so
// *              the forwarding information must be loaded as required in
// *              structures s_matchForward, s_matchForward_host, etc
// *
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    scratch
// *   R2:    scratch
// *   R3:
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:                                                                       -
// *   R7:                                                                     -
// *   R8:                                                                     -
// *   R9:      |                      |paAclCfg   -                           -  pktScope used
// *   R10:     |  Error forward Info              -  lut1MatchSCope           -  for size and offset info
// *   R11:     |                                  -                           -  only
// *   R12:     |                                  -                           -
// *   R13:                                                                    -
// *   R14:
// *   R15:
// *   R16:
// *   R17:
// *   R18:
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
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ******************************************************************************************************/
    .using  cdeScope
    .using  pktScope
    .using  lut1MatchScope

f_c1ForwardHeldFCPktNoMatch:

    // Inc the no match stat (new system statistics)
    //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_NO_MATCH
    
    // Load the error routing information
    lbco  s_fmPlace,   PAMEM_CONST_EROUTE,  EF_EROUTE_FC_LUT1_FAIL << 4,   SIZE(s_fmPlace)
    
    qbeq  l_c1ForwardHeldFCPktNoMatch_1,   s_matchForward.forwardType,   PA_FORWARD_TYPE_HOST
        // Discard
        jmp fci_stdHeldFCPktDrop
        
l_c1ForwardHeldFCPktNoMatch_1:
        // Host Routing
        
        // Patch swinfo0
        sbco s_matchForward_host.context,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_c1ForwardHeldFCPktNoMatch_2,    s_matchForward_host.psflags,  0 
            lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
            or   r2.b0, r2.b0,  s_matchForward_host.psflags
            sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
l_c1ForwardHeldFCPktNoMatch_2:
        // Send the packet on its way
        // Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   
            
l_c1ForwardHeldFCPktNoMatch_2_queue_bounce:
        // Check for Queue Bounce operation
l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_ddr:
        qbbc l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_msmc, s_matchForward.queue.t_pa_forward_queue_bounce_ddr
            clr s_matchForward.queue.t_pa_forward_queue_bounce_ddr
            sbco s_matchForward.queue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_matchForward.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_end

l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_msmc:
        qbbc l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_end, s_matchForward.queue.t_pa_forward_queue_bounce_msmc
            clr s_matchForward.queue.t_pa_forward_queue_bounce_msmc
            sbco s_matchForward.queue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_matchForward.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_c1ForwardHeldFCPktNoMatch_2_queue_bounce_end:
        // CDE workaround: Do not use CDE_FLG_SET_DESTQUEUE
        //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
        sbco s_matchForward.queue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ), SIZE(s_pktDescr.destQ)  
        
        mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        ret
        
    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    
// ******************************************************************************************************
// * FUNCTION PURPOSE: Standard routing of the current packet
// ******************************************************************************************************
// * DESCRIPTION: The current packet is routed
// *
// *              The only time a current packet is routed is if there is an error.
// *              the exception index is supplied in r30.w2, and the packet is forwarded based on
// *              that index.
// *
// *              s_txPktCxt.scratch contains the packet ID
// *
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    
// *   R2:    
// *   R3:          | discard stat mask for classify1
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:       |
// *   R7:       | Forwarding info
// *   R8:       | 
// *   R9:       |
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
// *   R20:
// *   R21:
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |     
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Exception Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *            
// ******************************************************************************************************

    .using currentFwdScope
    .using pktScope
    
f_errCurrentPktForward:
    // Load the error routing information
    lsl   r0.w2,         s_txPktCxt2.eId,              4
    lbco  s_curFmPlace,   PAMEM_CONST_EROUTE,  r0.w2,   SIZE(s_curFmPlace)

    // Need a return address
    mov r30.w0, fci_c1Parse14_1
    jmp f_c1CurPktForward

    .leave pktScope
    .leave currentFwdScope
   
// *********************************************************************************************
// * FUNCTION PURPOSE: C1 Current packet forwarding
// *********************************************************************************************
// * DESCRIPTION: The current packet is forwarded based on the forward information
// *              input for exception only
// *    On Entry:
// *        R6-R9  has the forward match structure
// *        R14-R18 extPktInfo
// *
// *   Register Usage:  
// * 
// *   R0:   scratch
// *   R1:   scratch 
// *   R2:   scratch 
// *   R3:       |  bit mask for stat to update on silent discard
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:          |
// *   R7:          |  Current Packet Forwarding info
// *   R8:          |
// *   R9:          |
// *   R10:     
// *   R11:     
// *   R12:     
// *   R13:                                                                    -
// *   R14:         |extPacketInfo
// *   R15:         |  
// *   R16:         |  
// *   R17:         |
// *   R18:         |
// *   R19:           | Usr statistics
// *   R20:           |
// *   R21:
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:  
// *   R28:  
// *   R29:  c1RunContext (s_runCxt) rsvd for C1, C2 and P          -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************

    .using currentFwdScope
    .using pktScope
    .using cdeScope
    .using usrStatsFifoScope

f_c1CurPktForward:

    // Record the effective packet size for user statistics update
    mov   s_usrStatsReq.pktSize,  s_txPktCxt.endOffset  
    
    // Update the current packet Info
    sbco s_txPktCxt,  cCdeOutPkt,     SIZE(s_pktDescr), SIZE(s_txPktCxt)

    // Forwarding packets to the host
    qbne  l_c1CurPktForward2,  s_curFwd.forwardType,  PA_FORWARD_TYPE_HOST

        // Patch swinfo0
        sbco s_curFwd_host.context,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_c1CurPktForward1_0,    s_curFwd_host.psflags,  0 
            lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
            or   r2.b0, r2.b0,  s_curFwd_host.psflags
            sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            
l_c1CurPktForward1_0:
l_c1CurPktForward1_0_queue_bounce:
        // Check for Queue Bounce operation
l_c1CurPktForward1_0_queue_bounce_ddr:
        qbbc l_c1CurPktForward1_0_queue_bounce_msmc, s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            clr s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1CurPktForward1_0_queue_bounce_end

l_c1CurPktForward1_0_queue_bounce_msmc:
        qbbc l_c1CurPktForward1_0_queue_bounce_end, s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            clr s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_c1CurPktForward1_0_queue_bounce_end:
        // Send the packet on its way
        // Free the packet Ext Info
        mov     s_pktExtDescr.threadId, THREADID_CDMA0
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE    
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8)
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,    (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.destQueue,     s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,        s_curFwd.flowId
        sbco s_curFwd.queue,            cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        //xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_c1CurPktForward4,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_c1CurPktForward2:
    qbeq  l_c1CurPktForward3,  s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD

        // Invalid match type in table - this should never happen

l_c1CurPktForward3: 
        // Do a silent discard, release the packet (new system statistics)
        // mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_DISCARD
        
        //   Free the packet Ext Info
        set  s_pktExtDescr.flags.fDroppedInd
        
        mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_ADVANCE
        //xout  XID_CDECTRL,         s_cdeCmdPkt,                 SIZE(s_cdeCmdPkt)
        qbeq  l_c1CurPktForward4,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret
        
l_c1CurPktForward4: 
#ifdef TO_BE_TBD  
        // User Statistics operation:
        // Record and insert the statistics update into the FIFO
        // Note all user commands are in the same location (r12)
        mov   s_usrStatsReq.index,  s_curFwd_host.cmd.w0
        
        lbco  s_fifoCb, PAMEM_CONST_USR_STATS_FIFO_BASE,  OFFSET_PDSP_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
        add   r1.b0,    s_fifoCb.in, 4
        and   r1.b0,    r1.b0,       0x1F
        
        qbne  l_c1CurPktForward9_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL
            ret

l_c1CurPktForward9_1:
            // Insert the request into the FIFO   
            add   r1.w2,   s_fifoCb.in, OFFSET_PDSP_USR_STATS_FIFO
            sbco  s_usrStatsReq,  PAMEM_CONST_USR_STATS_FIFO_BASE, r1.w2, SIZE(s_usrStatsReq)
            sbco  r1.b0,   PAMEM_CONST_USR_STATS_FIFO_BASE,    OFFSET_PDSP_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
            ret
#else
            ret

#endif            

    .leave currentFwdScope
    .leave pktScope
    .leave cdeScope
    .leave usrStatsFifoScope

   
// ************************************************************************************************
// * FUNCTION PURPOSE: Begin parsing a packet
// ************************************************************************************************
// * DESCRIPTION: A new packet ID is assigned and a packet parse is begun
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:      |  checkScope  
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |  
// *   R8:        |  
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     
// *   R11:       |                                     
// *   R12:       |                                     
// *   R13:       |                                     
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  
// *   R19:          |                                  
// *   R20:          |                                  
// *   R21:          |                                  
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2 - param.action w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  checkScope

f_c1Parse:

	lbco  r1, cMailbox, 0, 4
	add   r1, r1, 1
	sbco  r1, cMailbox, 0, 4

    // Read the descriptor
    xin  XID_CDEDATA,  s_pktDescr,   SIZE(s_pktDescr)
    
    // Read packet extended info
    xin  XID_PINFO_SRC, s_pktExtDescr, SIZE(s_pktExtDescr)

l_c1Parse1:
l_c1Parse2:     
l_c1Parse3:

    // The context is read from the control section. Advance
    // to the control section
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_CONTROL
    xout  XID_CDECTRL,           s_cdeCmdWd,            4

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    //mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    //mov s_cdeInsert.byteCount,  32
    ldi  r4,        CDE_CMD_INSERT_PSDATA | (32 << 8)
    xout XID_CDECTRL,           s_cdeCmdWd,             4
    
    qbne  l_c1Parse3a,   s_pktDescr.ctrlDataSize,    0   // ctrlSize should be non-zero
    
        // Update Statistics
        // TBD: system statistics
        mov   s_stats.value, PA_STATS_UPDATE_REQ | PA_STATS_COMMAND_FAIL
    
        // Discard the packet
        set  s_pktExtDescr.flags.fDroppedInd
    
        mov   s_cdeCmdPkt.operation,  CDE_CMD_PACKET_ADVANCE
        mov   s_cdeCmdPkt.optionsFlag, 0
        set   s_runCxt.flags.t_previous_skip
        //xout  XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)

        // Packet should be dropped at the last DSP at this stage
        jmp   l_c1Parse14_1

l_c1Parse3a:
    // TBD: Update system statistics
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_N_PKTS
    
    // l_c1Parse3a - l_c1Parse5: command and forwarding packet
    // Read in the whole packet context. This is the largest command size
    // There is no cost if this is not a data packet. 
    xin  XID_CDEDATA,        s_txPktCxt,          SIZE(s_txPktCxt)
    
    // extract the command ID 
    lsr  r1.b0,  s_txPktCxt.paCmdId_Length, SUBS_CMD_WORD_ID_SHIFT

    qbeq  l_c1Parse9,         r1.b0,   PSH_CMD_PA_TX_FLOW
    qbeq  f_paConfigure,      r1.b0,   PSH_CMD_CONFIG        // no return

l_c1Parse4:
    // The packet contains other Tx commands and it should be processed
    // by the next PDSP
        
    // pass through
l_c1Parse5:
    // Forward the packet
    // TBD: Make sure that the psInfoSize is 0
    mov   s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
    mov   s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_PSINFO
    mov   s_cdeCmdPkt.psInfoSize,  0
    
    set   s_runCxt.flags.t_previous_skip
    //xout  XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)

    // Packet should be processed at the next PDSP 
    jmp   l_c1Parse14_2

l_c1Parse9:
    // Check whether CmdSet needs to be processed. CmdSet operations can land in 
    // this PDSP from QoS output
#ifdef    PASS_PROC_CMDSET_IN_EG
    qbbc  l_c1Parse9a_0,   s_txPktCxt.flags.t_flag_eg_cmdSet
      // Set the thread ID to Post Processing PDSPs for CmdSet processing
      mov  s_pktExtDescr.threadId,  THREADID_POST
      // Forward the packet
      // TBD: Make sure that the psInfoSize is 0
      mov   s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
      mov   s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_PSINFO
      mov   s_cdeCmdPkt.psInfoSize,  0
    
      set   s_runCxt.flags.t_previous_skip
      //xout  XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)      
      jmp  l_c1Parse14_1
l_c1Parse9a_0:    
#endif
    // Verify whether it is a lookup packet or forwarding packet
    qbbc  l_c1Parse4,   s_txPktCxt.flags.t_flag_fc_lookup
        // Delete the control information. It will be replace with the packet context during parse
        mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
        xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

l_c1Parse9a:
    // advance the CDE to the first byte of innerIp to examine in the packet
    mov  s_txPktCxt.startOffset, s_txPktCxt.l3Offset2 
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.startOffset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    
    // save  Extended Header to be examined later
    xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    // next header is stored in 8 bit format during the parse 
    // Flow cacke lookup always strats with IPv4 which may lead to IPv6 directly
    mov s_next.Hdr,         PA_HDR_IPv4
    mov s_param.action,     SUBS_ACTION_PARSE
    
l_c1Parse10:
    // s_l1View1(16), s_l1View2 (16) and s_l1View3 (16) share the same area
    // Clear the LUT1 view areas
    // Note: packet descriptor will still reside at r6-r13
    zero &s_l1ViewA,     SIZE(s_l1ViewA)
    xout  XID_LUT1V1,     s_l1ViewA,         SIZE(s_l1ViewA)
    xout  XID_LUT1V2,     s_l1ViewA,         SIZE(s_l1ViewA)
    xout  XID_LUT1V3,     s_l1ViewA,         SIZE(s_l1ViewA)
    jmp   l_c1Parse11_1

l_c1Parse11:
    //  The main parse loop
    //
    //  General Error check to avoid infinite loop
    //  r28.b3: previous startOffset
    //
    qbge l_c1Parse11_1,  r28.b3, s_txPktCxt.startOffset
        mov s_param.action,          SUBS_ACTION_DISCARD
        jmp l_c1Parse14
     
l_c1Parse11_1:
    //  Util SUBS_ACTION_LOOKUP or SUBS_ACTION_EXIT occur
    mov   r28.b3,         s_txPktCxt.startOffset
    lsl   r0.b0,          s_next.Hdr,         1
    lbco  r0.w2,          PAMEM_CONST_PARSE,  r0.b0,              2
    call  r0.w2
    qbeq  l_c1Parse11,    s_param.action,     SUBS_ACTION_PARSE

l_c1Parse12:
    // TBD: Post classification operation
    clr   s_txPktCxt.flags.t_flag_fc_lookup
            
l_c1Parse14:
    // Restore the latest extended Packet Info
    xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
    wbs   s_flags.info.tStatus_CDEOutPacket
    //mov   s_txPktCxt.paCmdId_Length, ((SIZE(s_txPktCxt) + 7) & 0xf8) + (PSH_CMD_PA_TX_FLOW << 5)     // round up to a multiple of 8 bytes
    sbco  s_txPktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_txPktCxt)

    // Packet is ready for lookup
    qbne  l_c1Parse16,  s_param.action,  SUBS_ACTION_LOOKUP

        // Hold the packet
        mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
        mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
        //xout XID_CDECTRL,             s_cdeCmdPkt,              SIZE(s_cdeCmdPkt)

l_c1Parse14_1:
fci_c1Parse14_1:
#ifndef PASS_LAST_PDSP
        // TBD: packet could be too small to trigger the output ready flag. 
        // wbs   s_flags.info.tStatus_CDEOutPacket
        wbc    s_flags.info.tStatus_CDEBusy
        lbco  r1.w0,  cCdeOutPkt,  OFFSET(s_pktDescr.pktflags), 2
        set   r1.w0.t_pktBypass
        sbco  r1.w0,  cCdeOutPkt,  OFFSET(s_pktDescr.pktflags), 2
#endif
fci_c1Parse14_2:
l_c1Parse14_2:
        set  s_runCxt.flags.t_held_packet
        qbbs l_c1Parse15, s_runCxt.flags.t_previous_held

            // No previous lookup is active
            qbbs l_c1Parse15_1, s_runCxt.flags.t_previous_skip
                // Issue new lookup
                set s_runCxt.flags.t_previous_held
                
                // Move ext Packet Header to PFIFO_B 
                xout XID_PINFO_B,    s_pktExtDescr, SIZE(s_pktExtDescr)
                
                // Issue packet held command
                xout XID_CDECTRL,    s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
                  
                // Clear packet held
                clr s_runCxt.flags.t_held_packet

                //mov s_l1Cmd.careFlags, s_runCxt.keyFlags
                mov s_l1Cmd.operation,       LUT1_CMD_SEARCH
            
                //Wait for LUT1 add/delete to be completed in case there is one pending
                wbc  s_flags.info.tStatus_Lut1Busy

                xout  XID_LUT1CMD,  s_l1Cmd,  SIZE(s_l1Cmd)

                //Wait for LUT1 Busy
                wbs  s_flags.info.tStatus_Lut1Busy

                jmp   fci_mainLoop6

l_c1Parse15:
            // There is an active lookup pending
            // The extended Header is at FIFO_A
            xout XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
            or   s_cdeCmdPkt.optionsFlag, s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
            xout XID_CDECTRL,             s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
            jmp fci_mainLoop6
            
l_c1Parse15_1:
           // Skip packet: Forward or drop the packet 
           clr s_runCxt.flags.t_previous_skip
           clr s_runCxt.flags.t_held_packet
           
           xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
           xout XID_CDECTRL,             s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
           jmp  fci_mainLoop6          


l_c1Parse16:
        // Handle all cases where lookup is not required
        set  s_runCxt.flags.t_previous_skip
        
        // No lookup is required, see whether we should discard the packet
        qbeq  l_c1Parse18,  s_param.action,  SUBS_ACTION_DISCARD
        
l_c1Parse17:
        // TBD: The followin few mode should not be supported
        qbeq  l_c1Parse17_1,  s_param.action,  SUBS_ACTION_FWPKT
            // It is an error packet. 
            // Forward the packet based on the exception index, which must be loaded at the forwarding function
            // Error Packet Handling 
            jmp  f_errCurrentPktForward

l_c1Parse17_1:
            // It is a normal packet (IP forwarding) 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_pktExtDescr.threadId,  THREADID_CDMA0
            jmp  l_c1Parse14_1
            
l_c1Parse18:
            // Discard this packet
            set  s_pktExtDescr.flags.fDroppedInd
            mov  s_cdeCmdPkt.operation, CDE_CMD_PACKET_ADVANCE
            mov  s_cdeCmdPkt.optionsFlag, 0
            jmp  l_c1Parse14_1

    .leave  lut1Scope
    .leave  pktScope
    .leave  cdeScope
    .leave  checkScope



    .using  startScope
    .using  initScope
f_c1LocalInit:

     // Clear the LUT table of all 256 entries
	.using lut1Scope
     
     mov  r3, 0
l_c1LocalInitD:
     loop l_c1LocalInitC, 256
     // Wait for LUT free 
     wbc s_flags.info.tStatus_Lut1Busy

     mov  s_l1Cmd.index,     r3
     mov  s_l1Cmd.operation, LUT1_CMD_REMOVEM
     xout XID_LUT1CMD,       s_l1Cmd,            SIZE(s_l1Cmd)
     // Wait for LUT1 Busy
     wbs  s_flags.info.tStatus_Lut1Busy
     add r3, r3, 1

     .leave lut1Scope
l_c1LocalInitC:

    // Initialize memory used only by this pdsp

    // The lut1 associated memory. There are 256 entries, each of which is
    // 20 bytes, organized as follows: 
    //
    //   Byte offset    byte size    description
    //      0              4          l1Info
    //      4             16          paForward for matches

    zero  &r3,                    20       // zeros r3 - r6
    mov s_paForward.forwardType,  PA_FORWARD_TYPE_DISCARD   // default flow cache: disabled

    // r2.w0 = 0x2000, r2.w2 = 0
    mov r2,  256*32

c1LocalInit_0:
    sbco  r3,     PAMEM_CONST_PDSP_LUT1_INFO, r2.w2, 20
    add   r2.w2,          r2.w2,  32
    qbne  c1LocalInit_0,  r2.w2,  r2.w0

    // Clear the bitmask the indicates which LUT1 entries are valid
    zero  &s_l1Map,  SIZE(s_l1Map) + SIZE(s_l1Map2)        // also redundant, but this is init code so be safe
    sbco   s_l1Map,  PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map) + SIZE(s_l1Map2)

    // Clear the pending configuration enable 
    zero  &s_l1PendCmd,  SIZE(s_l1PendCmd)
    sbco   s_l1PendCmd, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING, SIZE(s_l1PendCmd)


    // Initialize the call table. Each PDSP with a LUT1 will do this, but they will
    // each write the same values
    // TBD: Add compiler switch to initialize this at first PDSP only
    // May need different table for different PDSP
    // For example, MAC related entries is not supported by L3/L4 PDSP
    mov s_headerParse.c1ParseMac,            f_c1ParseUnkn
    mov s_headerParse.c1ParseVlan,           f_c1ParseUnkn
    mov s_headerParse.c1ParseMpls,           f_c1ParseUnkn
    mov s_headerParse.c1ParseIpv4,           f_c1ParseIpv4
    mov s_headerParse.c1ParseIpv6,           f_c1ParseIpv6
    mov s_headerParse.c1ParseIpv6ExtHop,     f_c1ParseIpv6ExtHop
    mov s_headerParse.c1ParseIpv6ExtRoute,   f_c1ParseIpv6ExtRoute
    mov s_headerParse.c1ParseIpv6ExtFrag,    f_c1ParseIpv6ExtFrag
    mov s_headerParse.c1ParseIpv6ExtDestOpt, f_c1ParseIpv6ExtDestOpt
    mov s_headerParse.c1ParseGre,            f_c1ParseGre
    mov s_headerParse.c1ParseEsp,            f_c1ParseUnkn
    mov s_headerParse.c1parseEspDec,         f_c1ParseUnkn
    mov s_headerParse.c1ParseAuth,           f_c1ParseUnkn
    mov s_headerParse.c1ParseCustom,         f_c1ParseUnkn
    mov s_headerParse.c1ParsePPPoE,          f_c1ParseUnkn
    mov s_headerParse.c1ParseSctp,           f_c1ParseUnkn
    mov s_headerParse.c1ParseUnkn,           f_c1ParseUnkn
    mov s_headerParse.c2ParseUdp,            f_c1ParseUdp
    mov s_headerParse.c2ParseUdpLite,        f_c1ParseUdpLite
    mov s_headerParse.c2ParseTcp,            f_c1ParseTcp

    sbco s_headerParse.c1ParseMac,  PAMEM_CONST_PARSE, 0, SIZE(s_headerParse)
    
  // IP protocol table (TBD)
  // Each entry in the table is of the form:
  //   /--------------------------------------------\
  //   |  7     6  |  5                           0 |
  //   +-----------+--------------------------------+
  //   |   action  |   internal header id           |
  //   \--------------------------------------------/
  //   
  //   The default header type is <unknown> 
  //   The default action is lookup

  mov   r4.w0,  (((SUBS_ACTION_LOOKUP << 6)  | PA_HDR_UNKNOWN) << 8)  |  ((SUBS_ACTION_LOOKUP << 6) | PA_HDR_UNKNOWN)
  mov   r4.w2,  r4.w0
  mov   r5,     r4
  mov   r6,     r4
  mov   r7,     r4
  mov   r8,     r4
  mov   r9,     r4
  mov   r10,    r4
  mov   r11,    r4
  sbco  r4,     PAMEM_CONST_IP_PROTO,  0,    32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  32,   32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  64,   32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  96,   32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  128,  32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  160,  32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  192,  32
  sbco  r4,     PAMEM_CONST_IP_PROTO,  224,  32

  // Protocol type 4: IP over IP 
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_IPv4
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IP_IN_IP,         1

  // Protocol type 41: IPv6 over IP
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_IPv6
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IPV6_IN_IPV4,     1

  // Protocol type 50: ESP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_ESP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_ESP,              1

  // Protocol type 51: AH
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_AUTH
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_AUTH,             1

  // Protocol type 47: GRE
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_GRE
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_GRE,              1

  // Protocol type 6: TCP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_TCP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_TCP,              1

  // Protocol tpye 0: IPv6 hop by hop
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_IPv6_EXT_HOP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IPV6_HOP_BY_HOP,  1

  // Protocol type 43: IPv6 routing header
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_IPv6_EXT_ROUTE
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IPV6_ROUTE,       1

  // Protocol type 44: IPv6 fragmentation header
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_IPv6_EXT_FRAG
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IPV6_FRAG,        1

  // Protocol type 59: IPv6 no next header - uses default action

  // Protocol type 60: IPv6 destination options
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_IPv6_EXT_DEST
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_IPV6_DEST_OPT,    1

  // Protocol type 17: UDP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_UDP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP,              1 

  // Protocol type 136: UDP lite
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_UDP_LITE
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP_LITE,         1

  // Protocol type 132: SCTP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_SCTP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_SCTP,             1
  
  // Configurable egress flow exception routing
  // All exceptions are initially configured for silent discard
  mov    r5,          0
  mov    r1,          EF_EROUTE_N_MAX * 16
  zero  &s_paF,       SIZE(s_paF)+SIZE(s_paFh)

  mov    s_paF.forwardType,  PA_FORWARD_TYPE_DISCARD

commonInit_0:
  sbco  &s_paF, PAMEM_CONST_EF_EROUTE, r5, SIZE(s_paF)+SIZE(s_paFh)
  add   r5,     r5,                 16
  qbne  commonInit_0, r5,           r1
    
c1LocalInit_1:
    ret

    .leave  initScope
    .leave  startScope


// *******************************************************************************************************
// * FUNCTION PURPOSE: An entry is deleted from LUT1
// *******************************************************************************************************
// * DESCRIPTION: An entry is disabled from the LUT
// *    On Entry:
// *            - r0.w0 contains the length of the data packet
// *            - paCmd1 has the command header
// *            - The CDE points to the start of the command header
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - length of the packet
// *   R1:    
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  
// *   R7:        |  
// *   R8:        |  
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     
// *   R11:       |                                     
// *   R12:       |                                     
// *   R13:       |                                     
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  
// *   R19:          |                                  
// *   R20:          |                                  
// *   R21:          |                                  
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************************************

    .using  lut1Scope
    .using  cdeScope
    .using  configScope
    .using  pktScope

f_paComDelLut1:
    xin  XID_CDEDATA,  s_paDelL1,  SIZE(s_paDelL1)

    mov   r0.w2, PA_LUT1_INDEX_LAST_FREE   
    qbge  l_paComDelLut1_0,  s_paDelL1.index,  r0.w2
        mov  s_pktCxt2.commandResult,  PA_COMMAND_RESULT_LUT1_INDEX_INVALID
        set  s_pktCxt2.ctrlFlag.t_cmdResultPatch
        clr  s_pktCxt2.ctrlFlag.t_cmdContinue
        jmp  f_cfgReply

l_paComDelLut1_0:

    // derive the LUT1 utilization map
    lsr   r1.b1,    s_paDelL1.index, 5
    lsl   r1.w2,    r1.b1,  2
    add   r1.w2,    r1.w2,  PDSP_CXT_OFFSET_L1_MAP
    lbco  r2,       PAMEM_CONST_PDSP_CXT,  r1.w2,   4  
    and   r3.b0,    s_paDelL1.index, 0x1F
    
    // Read in the allocated index bitmap snd clear the corresponding bits
    // Its not an error to disable an already disabled entry
    lbco  r1.b0,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP2,  1
    
    clr   r2,   r3.b0
    sbco  r2,   PAMEM_CONST_PDSP_CXT,  r1.w2,   4  
    clr   r1.b0,  r1.b1
    sbco r1.b0,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP2,  1

l_paComDelLut1_1:
    // Disable all the routing info
    zero  &s_paFwdA,              SIZE(s_paFwdMatchPlace)
    mov    s_paFwdA.forwardType,  PA_FORWARD_TYPE_DISCARD
    mov    s_paFwdA.flowId,       0xde                     // Indicate that this entry has been deleted
    
    // Store the deleted forward table
    lsl  r0.w0,       s_paDelL1.index,             4       // The offset to this table entry
    sbco s_paFwdA,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,    SIZE(s_paFwdMatchPlace)
    
    mov  s_l1PendCmdS.operation,  LUT1_CMD_REMOVEM
    mov  s_l1PendCmdS.index, s_paDelL1.index
    mov  s_l1PendCmdS.flags, 0

    // see whether an active search is in progress, need to wait for it to be completed
    qbbs  l_paComDelLut1_2,  s_runCxt.flags.t_previous_held

        // wait for previous add/delete to be complete
        wbc   s_flags.info.tStatus_Lut1Busy

        // If the LUT is free do the deletion
        xout XID_LUT1CMD,       s_l1PendCmdS,            SIZE(s_l1PendCmdS)
        // Wait for LUT1 Busy
        wbs  s_flags.info.tStatus_Lut1Busy
        jmp  f_cfgReply

l_paComDelLut1_2:
    // If the LUT is busy save the disable info

    sbco  s_l1PendCmdS, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,    SIZE(s_l1PendCmdS)
    set   s_runCxt.flags.t_pendingConfig
    jmp   f_cfgReply
    
    .leave lut1Scope
    .leave cdeScope
    .leave configScope
    .leave pktScope


#include "pacfgcmn.p"

// ********************************************************************************************************
// * FUNCTION PURPOSE: An add/del to LUT2 command was sent to a PDSP without a LUT2. An error is returned
// ********************************************************************************************************
// * DESCRIPTION:
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
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     
// *   R11:       |                                     
// *   R12:       |                                     
// *   R13:       |                                     
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  
// *   R19:          |                                  
// *   R20:          |                                  
// *   R21:          |                                  
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************************
    .using configScope
    .using cdeScope

f_paComAddRepLut2:
f_paComDelLut2:
    mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_DESTINATION
    xout XID_CDEDATA,            s_paCmd1.commandResult,                   SIZE(s_paCmd1.commandResult)
    jmp  f_cfgReply 

    .leave configScope
    .leave cdeScope

// *********************************************************************************************
// * FUNCTION PURPOSE: Add an entry to LUT1
// *********************************************************************************************
// * DESCRIPTION: The new entry is loaded. It will not be activated if the LUT is busy
// *
// *    On Entry:
// *            - r0.w0 contains the length of the data packet
// *            - paCmd1 has the command header
// *            - The CDE points to the start of the command header
// *
// *   Register Usage:  
// * 
// *   R0:    w2 - PA_LUT1_INDEX_LAST_FREE,  w0 - the packet length (entry)
// *   R1:        |  
// *   R2:        | Scratch
// *   R3:        |      
// *   R4:    |  CDE commands     -  cdeScope                                   |  s_l1PendCmd
// *   R5:    |                   -                                             |
// *   R6:        |                                     |                       |                     
// *   R7:        |                                     |  s_paCmd1 (entry)     |      
// *   R8:        |                                     |                       |      
// *   R9:        |  LUT1 View1   - lut1Scope           |
// *   R10:       |                                         | s_paAddL1Hdr
// *   R11:       |
// *   R12:       |
// *   R13:       |                                         | s_paAddL1f (s_paAddL1fAcl)
// *   R14:          |                                          |
// *   R15:          |                                          |  Forward match info
// *   R16:          |                                          |
// *   R17:          |  LUT1 View2  - lut1Scope                 |
// *   R18:          |                                           |
// *   R19:          |                                           |  Forward prev match info
// *   R20:          |                                           |
// *   R21:          |                                           |
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *********************************************************************************************

    .using lut1Scope
    .using cdeScope
    .using configScope
    .using pktScope

f_paComAddRepLut1:
    // Read the common header
    xin  XID_CDEDATA,  s_paAddL1Hdr,  SIZE(s_paAddL1Hdr)
    
    // s_paAddL1f may be populated during the LUT1 configuration
    zero  &s_paAddL1fFc,             SIZE(s_paAddL1fFc)    
    mov   s_paAddL1fFc.statsCmd,     s_paAddL1Hdr.statsIndex
    set   s_paAddL1fFc.statsCmd.t_stats_update_req 
        
    // Make sure the packet has all the data
    mov  r0.w2,  SIZE(s_paAddL1Hdr)+(2*SIZE(s_paFwdMatchPlace)) + (SIZE(s_paAddL1Std1)*3) + SIZE(s_paAddL1Cmdhdr)
    qble l_paComAddRepLut1_0,  r0.w0,  r0.w2
        mov  r0.b0, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        qbbc l_paComAddRepLut1_err, s_pktCxt2.ctrlFlag.t_cmdHdrMoved
            jmp l_paComAddRepLut1_errPatch
            
l_paComAddRepLut1_0:
    // Read in the allocated index bitmap
    lbco  r1.b0,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP2,  1

    // Get a free index if requested
    mov   r0.w2,    PA_LUT1_INDEX_LAST_FREE                                          
    qbne  l_paComAddRepLut1_3,  s_paAddL1Hdr.index,  r0.w2
    
        lmbd  r1.b1, r1.b0,  0  // Find free MAP entry index
        qbne l_paComAddRepLut1_1,  r1.b1,           32
            // No free entries
            mov  r0.b0, PA_COMMAND_RESULT_LUT1_INDEX_INVALID
            qbbc l_paComAddRepLut1_err, s_pktCxt2.ctrlFlag.t_cmdHdrMoved
                jmp l_paComAddRepLut1_errPatch
            
l_paComAddRepLut1_1:   
        lsl   r1.w2,    r1.b1,  2
        add   r1.w2,    r1.w2,  PDSP_CXT_OFFSET_L1_MAP
        lbco  r2,       PAMEM_CONST_PDSP_CXT,  r1.w2,   4  

        lmbd  r3.b0,  r2,  0   // Find free bit index at the MAP entry
        qbne  l_paComAddRepLut1_2, r3.b0,                  32
                // TBD: Error condition should never occur
                mov  r0.b0, PA_COMMAND_RESULT_LUT1_INDEX_INVALID
                qbbc l_paComAddRepLut1_err, s_pktCxt2.ctrlFlag.t_cmdHdrMoved
                    jmp l_paComAddRepLut1_errPatch
                
l_paComAddRepLut1_2:
    // Free entry found 
    lsl s_paAddL1Hdr.index, r1.b1, 5
    add s_paAddL1Hdr.index, s_paAddL1Hdr.index, r3.b0 

    // Write back the index
    xout  XID_CDEDATA,      s_paAddL1Hdr.index,  SIZE(s_paAddL1Hdr.index)
    jmp   l_paComAddRepLut1_5

l_paComAddRepLut1_3:
    // Verify the index is in range
    qbgt  l_paComAddRepLut1_4,  s_paAddL1Hdr.index,  r0.w2
          mov  r0.b0, PA_COMMAND_RESULT_LUT1_INDEX_INVALID
          qbbc l_paComAddRepLut1_err, s_pktCxt2.ctrlFlag.t_cmdHdrMoved
              jmp l_paComAddRepLut1_errPatch

l_paComAddRepLut1_4:
    // derive the LUT1 utilization map
    lsr   r1.b1,    s_paAddL1Hdr.index, 5
    lsl   r1.w2,    r1.b1,  2
    add   r1.w2,    r1.w2,  PDSP_CXT_OFFSET_L1_MAP
    lbco  r2,       PAMEM_CONST_PDSP_CXT,  r1.w2,   4  
    and   r3.b0,    s_paAddL1Hdr.index, 0x1F

    
l_paComAddRepLut1_5:
    // Scroll past the command headers
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  SIZE(s_paCmd1)+SIZE(s_paAddL1Hdr)
    xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)

    // TBD: can we use call here
    jmp  f_paComAddLut1Standard

fci_paComAddRepLut1_6:
    // Update the LUT1 utilization maps
    set   r2,   r3.b0
    sbco  r2,   PAMEM_CONST_PDSP_CXT,  r1.w2,   4  
    lmbd  r3.b1, r2, 0
    qbne  l_paComAddRepLut1_7,  r3.b1, 32
        // this entry is full
        set  r1.b0,  r1.b1
        sbco r1.b0,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP2,  1
        
l_paComAddRepLut1_7:
    // The CDE is now pointing to the Packet forward information.
    xin  XID_CDEDATA,  s_paFwdMatchPlace,  SIZE(s_paFwdMatchPlace)

    // store all the info (l1Info + match info)
    lsl  r0.w0,       s_paAddL1Hdr.index,          5       // The offset to this table entry
    sbco s_paAddL1fFc,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1fFc)+SIZE(s_paFwdMatchPlace)
    
    mov  s_l1PendCmd.operation,  LUT1_CMD_INSERTM
    mov  s_l1PendCmd.index, s_paAddL1Hdr.index
    mov  s_l1PendCmd.flags, 0
    mvid *&s_l1PendCmd.bitMask, *&s_paAddL1Cmdhdr.bitMask
    
    // Complete the entry if there is no active search in progress, need to wait for it to be completed
    qbbs  l_paComAddRepLut1_8,  s_runCxt.flags.t_previous_held
    
        // wait for previous entry update to be completed
        wbc  s_flags.info.tStatus_Lut1Busy
        xout XID_LUT1CMD,       s_l1PendCmd,            SIZE(s_l1PendCmd)
        // Wait for LUT1 Busy
        wbs  s_flags.info.tStatus_Lut1Busy
        jmp f_cfgReply

l_paComAddRepLut1_8:

    // The LUT was busy. Store the information in scratch until the LUT is free
    // Note: PDSP will not process either packets or configuration util this one is completed
    sbco  s_l1PendCmd,   PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,    SIZE(s_l1PendCmd)
    set   s_runCxt.flags.t_pendingConfig
    jmp   f_cfgReply
    
l_paComAddRepLut1_errPatch:
    mov  s_pktCxt2.commandResult,  r0.b0
    set  s_pktCxt2.ctrlFlag.t_cmdResultPatch
    clr  s_pktCxt2.ctrlFlag.t_cmdContinue
    jmp  f_cfgReply

l_paComAddRepLut1_err:    
    mov s_paCmd1.commandResult, r0.b0
    xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
    clr  s_pktCxt2.ctrlFlag.t_cmdContinue
    jmp f_cfgReply
    
    .leave lut1Scope
    .leave cdeScope
    .leave configScope
    .leave pktScope


// ************************************************************************************
// * FUNCTION PURPOSE: Add a standard entry to LUT1
// ************************************************************************************
// * DESCRIPTION: The LUT1 fields are entered
// *
// *              On Entry:
// *                - r1,r2 has the bit map of used entry indicies
// *                - r0.b2 has the udp custom port flag
// *                - r0.w0 has the offset value to the command result flag
// *                - The CDE points to the start of the standard entry data
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - offset to the command result value (for patching)
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
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************/

    .using configScope
    .using cdeScope
    .using lut1Scope

f_paComAddLut1Standard:

   // LUT1V1 and LUT1V2
   xin  XID_CDEDATA, s_paAddL1Std1,  SIZE(s_paAddL1Std1) + SIZE(s_paAddL1Std2)
   xout XID_LUT1V1,  s_paAddL1Std1,  SIZE(s_paAddL1Std1)
   xout XID_LUT1V2,  s_paAddL1Std2,  SIZE(s_paAddL1Std2)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1Std1) + SIZE(s_paAddL1Std2)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)
   
   // LUT1V3
   xin  XID_CDEDATA, s_paAddL1Std3,  SIZE(s_paAddL1Std3)
   xout XID_LUT1V3,  s_paAddL1Std3,  SIZE(s_paAddL1Std3)
   
   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1Std3)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   // Command Header
   xin  XID_CDEDATA, s_paAddL1Cmdhdr, SIZE(s_paAddL1Cmdhdr)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1Cmdhdr)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   jmp  fci_paComAddRepLut1_6       

    .leave configScope
    .leave cdeScope
    .leave lut1Scope
  
    
// *********************************************************************************************************
// * FUNCTION PURPOSE: Complete a waiting LUT1 modify
// *********************************************************************************************************
// * DESCRIPTION: A pending LUT1 entry or deletion is completed
// *              The LUT is free (not checked in this function)    
// *
// *   Register Usage:  
// * 
// *   R0:    
// *   R1:    |  Valid map
// *   R2:    |
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope      |  s_l1PendCmd
// *   R5:    |                   -                |
// *   R6:        |                                |      
// *   R7:        |                                |      
// *   R8:        |                                |      
// *   R9:        |  LUT1 View1   - lut1Scope
// *   R10:       |                                 
// *   R11:       |
// *   R12:       |
// *   R13:       |                                 | s_paAddL1f
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
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *********************************************************************************************************

    .using  lut1Scope
    .using  cdeScope
    .using  configScope

f_paComLut1CompleteModify:

    clr s_runCxt.flags.t_pendingConfig

    // Get the pending information from scratch 
    lbco s_l1PendCmd,  PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,  SIZE(s_l1PendCmd)
    xout XID_LUT1CMD,   s_l1PendCmd,         SIZE(s_l1PendCmd)
    // Wait for LUT1 Busy
    wbs  s_flags.info.tStatus_Lut1Busy
    jmp  fci_mainLoop7

    .leave lut1Scope
    .leave configScope
    .leave cdeScope
    
#include "parse_tx.p"

    .leave globalScopeC1

