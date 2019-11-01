// *********************************************************************************************************
// * FILE PURPOSE: Perform packet classification on PDSPS with a LUT2
// *********************************************************************************************************
// * FILE NAME: classify2.p
// *
// * DESCRIPTION: The PDSP code for L4 classification using a LUT2
// *
// *********************************************************************************************************/
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
#define PASS_CLASSIFY2  1
#include "pdsp_pa.h"
#include "pdsp_mem.h"
#include "pdsp_subs.h"
#include "pm_config.h"
#include "pdsp_ver.h"
#include "pm_constants.h"
#include "parsescope.h"

#define PASS_C2_VER    PASS_VERSION                  // 0x01.0x00.0x00.0x03

    .origin      0
    .entrypoint  f_c2Init

f_c2Init:
    jmp f_c2Start
    
header:
    .codeword  HEADER_MAGIC
    .codeword  PASS_C2_VER

    .using  globalScopeC2

#include "meminit.p"

f_c2Start:

    call  f_c2LocalInit

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
    // qbbc  l_c2Start0,  s_flags.info.tStatus_Command1
    
// Note: The first PDSP at each stage should perform global init
#ifdef PASS_GLOBAL_INIT    
      call f_commonInit
#endif      

      //mov  r2, 0
      //sbco r2, cMailbox, 4, 4      // Clear the mailbox

l_c2Start0:

    zero  &s_runCxt,       SIZE(s_runCxt)

    // Store the PDSP ID
    mov   r2.b0,  PASS_PDSP_ID
    sbco  r2.b0,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID, 1


    // Store the version number
    mov   r2.w0,   PASS_C2_VER & 0xFFFF
    mov   r2.w2,   PASS_C2_VER >> 16 
    sbco  r2,   PAMEM_CONST_PDSP_INFO,  OFFSET_VER,     4    

// *****************************************************************************************
// * FUNCTION PURPOSE: The main processing loop
// *****************************************************************************************
// * DESCRIPTION: The packet processing state machine
// *
// *        Register usage:
// *
// *   R0:
// *   R1:
// *   R2:
// *   R3:
// *   R4:
// *   R5:
// *   R6:
// *   R7:
// *   R8:
// *   R9:
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
// *   R29:  c2RunContext (s_runCxt)                              -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// ***********************************************************************************


f_mainLoop:
fci_mainLoop7:  // For compatibility with classify1.p

#ifdef PASS_PROC_CMD1
   // Look for command 
    qbbc l_mainLoop0_1, s_flags.info.tStatus_Command1
#ifdef PASS_PROC_EOAM_CMD
    // Process EOAM config command
     lbco  r1, cMailbox, FIRMWARE_CMD_EOAM_CFG_OFFSET, 4
     qbne  l_mainLoop_not_eoamcfg, r1.b3, FIRMWARE_CMD_EOAM_CFG
        // Clear the valid bit fields 
        not r1.b2, r1.b1
        // force clear valid bits to zero and retain valid bits positions
        and s_runCxt.flags, s_runCxt.flags, r1.b2
        
        // set the run time flags
        or  s_runCxt.flags, s_runCxt.flags, r1.b0
    
        // clear the command flag
        mov   r1, 0           
        sbco  r1, cMailbox, FIRMWARE_CMD_EOAM_CFG_OFFSET, 4
l_mainLoop_not_eoamcfg:        
#endif
l_mainLoop0_1:       
#endif
f_mainLoop0:
  qbbc  f_mainLoop,  s_flags.info.tStatus_CDENewPacket

//      Fall through to f_c2Parse
  
// **************************************************************************************
// * FUNCTION PURPOSE: Begin parsing a packet
// **************************************************************************************
// * DESCRIPTION: A new packet ID is assigned and a packet parse is begun
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
// *   R30:  w2 - param.action w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ****************************************************************************************

    .using  pktScope
    .using  cdeScope
    .using  currentFwdScope

f_c2Parse:

    // packet counter
    lbco  r1, cMailbox, 0, 4
    add   r1, r1, 1
    sbco  r1, cMailbox, 0, 4

    // Read the descriptor
    xin  XID_CDEDATA,  s_pktDescr,  SIZE(s_pktDescr)
    
    // Read packet extended info
    xin  XID_PINFO_SRC, s_pktExtDescr, SIZE(s_pktExtDescr)
    
    // TBD: check drop/bypass flag
    //      jmp to skip operation
    qbbc l_c2Parse1,  s_pktDescr.pktFlags.t_pktBypass
        // Advance to the control section to read the packet context
        mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_END
        xout XID_CDECTRL,            s_cdeCmdWd,            SIZE(s_cdeCmdWd)
        jmp f_c2ForwardPkt      
        
l_c2Parse1:
l_c2Parse2:
l_c2Parse3:

    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C2_N_PKTS


    // Advance to the control section to read the packet context
    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout XID_CDECTRL,            s_cdeCmdWd,            4

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    //mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    //mov s_cdeInsert.byteCount,  32
    ldi r4,     CDE_CMD_INSERT_PSDATA | (32 << 8)
    xout XID_CDECTRL,           s_cdeCmdWd,             4
    
    // Read in the whole packet context.
    // There is no cost if this is not a data packet
    xin  XID_CDEDATA,      s_pktCxt,           SIZE(s_pktCxt)

    // There must be a command in the control section for packets
    qbne  l_c2Parse4,  s_pktDescr.ctrlDataSize, 0
      //    Load the error routing info for this error type
      //    There was invalid data in the packet context. Record a system error
      //    and route the packet as a system error packet.
      and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
      or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_SYSTEM_FAIL << PKT_EIDX_SHIFT
      mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C2_INVALID_CONTROL
      
      jmp  f_c2ForwardErr

l_c2Parse4:
    
#ifdef PASS_VERIFY_SCTP_CRC
    
    qbbc l_c2Parse4_1, s_pktCxt.flags.t_flag_crc_verify
        // SCTP CRC Verification
        jmp  f_paSctpVerifyCRC       // No return

l_c2Parse4_1:

#endif    

    // extract the command ID
    lsr  r1.b0,  s_pktCxt.paCmdId_Length, SUBS_CMD_WORD_ID_SHIFT

    qbeq l_c2Parse6,        r1.b0,  PSH_CMD_PA_RX_PARSE
    qbeq f_paConfigure,     r1.b0,  PSH_CMD_CONFIG

l_c2Parse5:
    // This pdsp requires that the packet have a command word. No valid
    // command was found, the packet must be discarded.
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C2_DISCARD
    // TBD: drop bad packet (bypass)
    set  s_pktExtDescr.flags.fDroppedInd
    //mov  s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    //xout XID_CDECTRL,            s_cdeCmdPkt,           SIZE(s_cdeCmdPkt)
    jmp  f_c2ForwardPkt

l_c2Parse6:
    // The next header type is stored in 8 bit format during the parse
    and s_next.Hdr,  s_pktCxt.eId_portNum_nextHdr.b0,  0x3f

    // Clear any values in the error index if not custom
    qbeq l_c2Parse7,    s_next.Hdr,   PA_HDR_CUSTOM_C2  
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  NOT_PKT_EIDX_MASK

l_c2Parse7:
    // Delete the control info. It will be replaced with new data
    // in protocol specific when the packet is forwarded
    //mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    //xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)
    // Delete only the pktCxt from the control info
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    mov  s_cdeCmdWd.byteCount,  (SIZE(s_pktCxt) + 7) & 0xf8     // Round up to multiple of 8 bytes
    xout XID_CDECTRL,           s_cdeCmdWd,                 4
     
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
    xout  XID_CDECTRL,           s_cdeCmdWd,                4

    // Advance the CDE to the first byte to examine in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_pktCxt.startOffset
    xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)

    mov  s_param.action,   SUBS_ACTION_PARSE
    
    // save  Extended Header to be examined later
    xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    //  Convert an PA_HDR_ESP_DECODED header type to C2 version
    qbne  l_c2Parse8,  s_next.hdr,  PA_HDR_ESP_DECODED
        mov s_next.hdr,  PA_HDR_ESP_DECODED_C2

l_c2Parse8:
    qbge  l_c2Parse9,    s_next.hdr,  PA_HDR_UNKNOWN
    qbge  l_c2Parse10_1, s_next.hdr,  PA_HDR_CUSTOM_C2

l_c2Parse9:
        // The header type was not valid for classify2
        // The header type can not be processed by classify2, use next fail route
        // Note: It is not a parsing error exception
        // Need a return address
        mov r30.w0, f_mainLoop
        // store innerIP offset
        mov  s_pktCxt5.l3offset2,  s_pktCxt5.l3l5Offset
        // Patch the context back to the packet
        wbs   s_flags.info.tStatus_CDEOutPacket
        sbco  s_pktCxt, cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)
        jmp  f_c2ForwardNoMatch


    // The main loop
    // Multiple parses are only possible because a ESP_DECODED and UDP parse can
    // trigger a change into other type

l_c2Parse10:

    //
    //  General Error check to avoid infinite loop
    //  r28.b3: previous startOffset
    //
    qbge l_c2Parse10_1,  r28.b3, s_pktCxt.startOffset
        and s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or  s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT
        mov s_param.action,          SUBS_ACTION_EXIT
        jmp l_c2Parse11_1
     
l_c2Parse10_1:
    mov   r28.b3,      s_pktCxt.startOffset
    lsl   r0.b0,       s_next.Hdr,         1
    lbco  r0.w2,       PAMEM_CONST_PARSE,  r0.b0,             2
    call  r0.w2
    qbeq  l_c2Parse10,  s_param.action,     SUBS_ACTION_PARSE

l_c2Parse11:
  // Put the next header type back into the context
    and  s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  0xc0
    or   s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  s_next.Hdr

l_c2Parse11_1:
  // Patch the context back to the packet
  // Restore the latest extended Packet Info
  xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
  wbs   s_flags.info.tStatus_CDEOutPacket
  sbco  s_pktCxt, cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

  qbne  l_c2Parse12,  s_param.action,  SUBS_ACTION_LOOKUP
      // The lookup was initiated in the parse. Wait for the lookup to complete
      // Should there be a timeout here?
      wbc  s_flags.info.tStatus_Lut2LookupBusy

      // Forward the results
      // Need a return address
      mov r30.w0, f_mainLoop
      qbbs f_c2ForwardMatch,  s_flags.info.tStatus_Lut2MatchData
      jmp  f_c2ForwardNoMatch
  
  l_c2Parse12:
    // Exception handling 
    // No Lookup required. Forward the packet based on the exception index
    jmp  f_c2ForwardErr
  
    .leave currentFwdScope
    .leave pktScope
    .leave cdeScope

// ***************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that matched LUT2 lookup
// ***************************************************************************************
// * DESCRIPTION: The forwarding information from the table is used to forward 
// *              the packet
// *
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    
// *   R2:    
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
// **************************************************************************************

    .using  cdeScope
    .using  currentFwdScope
    .using  pktScope

f_c2ForwardMatch:

    // Get the match info
    xin  XID_LUT2DATA,   s_curFmPlace,  SIZE(s_curFmPlace)

    // Set the bitmask for silent discard (credit to classify2)
    mov   r3.w0, PA_STATS_UPDATE_REQ | PA_STATS_C2_DISCARD

    mov r30.w0,  f_mainLoop  // return address
    jmp f_curPktForward
    
    .leave cdeScope
    .leave currentFwdScope
    .leave pktScope

// **************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that did not match
// **************************************************************************************
// * DESCRIPTION: On match fail the packet is forwarded if previous match information
// *              is available, otherwise it is discarded.
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
// *   R7:          |  Current Packet Forwarding info
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
// **************************************************************************************

    .using pktScope
    .using currentFwdScope
    .using  lut1MatchScope

f_c2ForwardNoMatch:
    // Must read the match data even if there was no match - to re-enable the LUT2 search
    xin  XID_LUT2DATA,   s_curFmPlace,  SIZE(s_curFmPlace)
    
    qbbc l_c2ForwardNoMatch_NotGTPU, s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_GTPU
       and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
       or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MATCH_FAIL << PKT_EIDX_SHIFT
    
    // prepare r1.w0, load the exception route
    // check to see if discard
    // true then jump to notgtpu
    // false then 
    
    mov  r1.w0,     EROUTE_GTPU_MATCH_FAIL * 16              

    lbco  s_curFmPlace,  PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_curFmPlace)

    qbeq l_c2ForwardNoMatch_NotGTPU, s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD
    
        //store the exception ID only if it is used
        sbco  s_pktCxt.eId_portNum_nextHdr,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2
        jmp  fci_c2ForwardErr_2

l_c2ForwardNoMatch_NotGTPU:
    
    qbbs  l_c2ForwardNoMatch0,  s_pktCxt.flags.t_flag_pl1Match

        // No previous match indicated. Set the error field and route based on the error
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_LUT2_FAIL << PKT_EIDX_SHIFT
        sbco s_pktCxt.eId_portNum_nextHdr,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2
        jmp f_c2ForwardErr

l_c2ForwardNoMatch0:

        // Use the previous lut1 match info to forward the packet
        // Extract the LUT1 index number, multiply by 32 to get memory offset
        // In the case of a previous match, the previous match routing information
        // must be loaded into the matchForward context
        lsr  r1.b2,  s_pktCxt.phyLink,     PKT_PDSP_ID_SHIFT   // Get PDSP ID 
        mov  r1.w0,  s_pktCxt.phyLink
        and  r1.b1,  r1.b1,     PKT_L1IDX_MASK
        lsl  r1.w0,  r1.w0,     6                              // get the associated table offset

        lsl  r1.b2,  r1.b2,     2  // PDSP ID * 4 = offset to the nextFail table base address
        lbco r2,     PAMEM_CONST_NFAIL_BASE, r1.b2,     4
        
        //Load the nextfail routing information
        lbbo  s_curFmPlace,     r2,  r1.w0,  SIZE(s_curFmPlace)
        // Store the stat for silent discard
        mov   r3.w0, PA_STATS_UPDATE_REQ | PA_STATS_C2_DISCARD
        
        jmp   f_curPktForward

    .leave lut1MatchScope
    .leave pktScope
    .leave currentFwdScope


// ********************************************************************************************
// * FUNCTION PURPOSE: Forward a packet based on the error index information
// ********************************************************************************************
// * DESCRIPTION: The error index is used to extract the routing information and forward
// *              the packet
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
// *   R7:          |  Current Packet Forwarding info
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

    .using pktScope
    .using currentFwdScope
          
f_c2ForwardErr:

    // Get the error index, multiply by 16 to get the memory offset
    // Load the error routing information
    lsr   r30.w2,  s_pktCxt.eId_portNum_nextHdr.b1,  PKT_EIDX_SHIFT
    lsl   r30.w2,  r30.w2,     4

    lbco  s_curFmPlace,  PAMEM_CONST_EROUTE,  r30.w2,  SIZE(s_curFmPlace)

fci_c2ForwardErr_2:

l_c2ForwardErr_2:
        // Get the bitfield mask for silent discard for classify2
        mov   r3.w0, PA_STATS_UPDATE_REQ | PA_STATS_C2_DISCARD

        // Need a return address
        mov    r30.w0, f_mainLoop
        jmp    f_curPktForward


    .leave pktScope
    .leave currentFwdScope
    
    
// ********************************************************************************************
// * FUNCTION PURPOSE: Forward a packet without routing information
// ********************************************************************************************
// * DESCRIPTION: Forward a byapps or dropped packet
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

    .using pktScope
    .using currentFwdScope
    .using cdeScope
          
f_c2ForwardPkt:
    // TBD: packet could be too small to trigger the output ready flag. 
    // wbs   s_flags.info.tStatus_CDEOutPacket
    wbc   s_flags.info.tStatus_CDEBusy
    lbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
    mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_ADVANCE
    mov   s_cdeCmdPkt.optionsFlag, 0

#ifdef PASS_LAST_PDSP
    // Clear Bypass Flag
    clr   s_pktDescr.pktFlags.t_pktBypass
    qbbc  l_c2ForwardPkt_1,  s_pktExtDescr.flags.fDroppedInd 
        set  s_pktExtDescr.flags.fDropped
        //clr  s_pktExtDescr.flags.fDroppedInd
        mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_FLUSH 
#else
    set   s_pktDescr.pktFlags.t_pktBypass
#endif   

l_c2ForwardPkt_1:                        
    sbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
    xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
    
    xout  XID_CDECTRL,       s_cdeCmdPkt,      SIZE(s_cdeCmdPkt)
    jmp   f_mainLoop

    .leave pktScope
    .leave currentFwdScope
    .leave cdeScope

// *****************************************************************************************************
// * FUNCTION PURPOSE: Process an add/modify or delete LUT1 command
// *****************************************************************************************************
// * DESCRIPTION: These commands are invalid for PDSPs with a classify2
// *
// *   Register Usage:  
// * 
// *   R0:   w0 - the packet length
// *   R1:    
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands
// *   R5:   |
// *   R6:          
// *   R7:
// *   R8:
// *   R9:
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
// ************************************************************************************************
    .using configScope
    .using cdeScope

f_paComAddRepLut1:
f_paComDelLut1:

    mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_INVALID_DESTINATION
    xout XID_CDEDATA,             s_paCmd1.commandResult,                SIZE(s_paCmd1.commandResult)
    jmp  f_cfgReply
    
    .leave configScope
    .leave cdeScope

// ************************************************************************************************
// * FUNCTION PURPOSE: Add or modify an entry in LUT2
// ************************************************************************************************
// * DESCRIPTION: A new entry is added to the lut
// *
// *   Register Usage:  
// * 
// *   R0:   w0 - the packet length
// *   R1:    
// *   R2:    
// *   R3:   queueDivert Info
// *   R4:   |  CDE commands
// *   R5:   |                  
// *   R6:          |  Command header (entry)
// *   R7:          |
// *   R8:          |
// *   R9:          |
// *   R10:             |  Add LUT2 command
// *   R11:             |
// *   R12:             |
// *   R13:                
// *   R14:     | LUT2  routing data0
// *   R15:     | LUT2          data1
// *   R16:     | LUT2          data2
// *   R17:     | LUT2          data3
// *   R18:     | LUT2  queueDivert 
// *   R19:     | LUT2 control
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
// *******************************************************************************************
    .using cdeScope
    .using configScope
    .using lut2Scope
    .using pktScope

f_paComAddRepLut2:

    // If there is an active add or delete the command must be rejected
    qbbc  l_paComAddRepLut2_check_full,  s_flags.info.tStatus_Lut2AddDelBusy

        mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT2_ADD_BUSY
        xout XID_CDEDATA,             s_paCmd1.commandResult,                SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply

l_paComAddRepLut2_check_full:
        // If there is an active add or delete the command must be rejected
        qbbc  l_paComAddRepLut2_a,  s_flags.info.tStatus_Lut2Full
          mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT2_FULL
          xout XID_CDEDATA,             s_paCmd1.commandResult,                SIZE(s_paCmd1.commandResult)
          jmp  f_cfgReply
        
l_paComAddRepLut2_a:

    // Verify that the packet has all the data
    qble l_paComAddRepLut2_0,  r0.w0,  SIZE(s_paAddL2Std)
        mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        xout XID_CDEDATA,            s_paCmd1.commandResult,              SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply

l_paComAddRepLut2_0:

    // Read in the command info. The command could be either a custom or standard,
    // but it really makes no difference to the add function since the sizes are the same
    xin  XID_CDEDATA,  s_paAddL2Std,  SIZE(s_paAddL2Std)

    // It is not required to update the number of entries for entry replacement
    qbbs  l_paComAddRepLut2_1,  s_paAddL2Std.ctrlBitMap.t_pa_lut2_ctrl_replace  
    
    // Keep a count of the number of entries. Return a warning if
    // it exceeds the max
    add   s_runCxt.c2NumInL2Table,  s_runCxt.c2NumInL2Table,  1
    mov   r1.w0,                    SUBS_MAX_LUT2_ENTRIES
    qbge  l_paComAddRepLut2_1,      s_runCxt.c2NumInL2Table,  r1.w0
    
    mov   s_paCmd1.commandResult,   PA_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES
    jmp   l_paComAddRepLut2_1_0

l_paComAddRepLut2_1:

    // Return success
    mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_SUCCESS
    
l_paComAddRepLut2_1_0:
    xout XID_CDEDATA,             s_paCmd1.commandResult,    SIZE(s_paCmd1.commandResult)
    
    // Advance the CDE then read in the forwarding info 
    // The advance must include the common command header as well as the add lut2 command
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  SIZE(s_paCmd1)+SIZE(s_paAddL2Std)
    xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)

    xin  XID_CDEDATA,           s_l2Entry.data0,         SIZE(s_paFwdMatchPlace) + SIZE(s_l2QueueDivert)
    // record the potential Queue diversion parameters 
    mvid  r3,                   *&s_l2QueueDivert   

    // Put the previously read key into the correct location
    // Switched to the custom version of the structure since it moves all 32 bits at once
    mov   s_l2Entry.key1.b0,    0 
    add   s_l2Entry.key1.b1,    s_paAddL2Custom.l4Type, s_paAddL2Custom.index     
    mvid  s_l2Entry.key2,       *&s_paAddL2Custom.match0
    mov   s_l2Entry.key1.w2,    s_paAddL2Custom.srcVC
    
    // We may need to update b3 if custom 
    qbeq l_paComAddRepLut2_3,  s_paAddL2Std.type,  PA_COM_ADD_LUT2_STANDARD 
    qbgt l_paComAddRepLut2_2,  s_paAddL2Std.index, PA_MAX_C2_CUSTOM_TYPES
       mov  s_pktCxt2.commandResult,  PA_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX
       set  s_pktCxt2.ctrlFlag.t_cmdResultPatch
       clr  s_pktCxt2.ctrlFlag.t_cmdContinue
       jmp  f_cfgReply
    
l_paComAddRepLut2_2:    
       // Calcualte the offset: 16-byte entries 
       lsl  r1.w0,    s_paAddL2Std.index,  4
       add  r1.w0,    r1.w0,  OFFSET_CUSTOM_C2 + OFFSET(struct_paC2Custom.bitSet) 
       lbco r1.b3,    PAMEM_CONST_CUSTOM2,  r1.w0,  SIZE(struct_paC2Custom.bitSet)
       //  Update the MS byte
       or   s_l2Entry.key2.b3,  s_l2Entry.key2.b3, r1.b3 

l_paComAddRepLut2_3:

    // Disable GTPU parsing only if GTPU flag is set
    qbbc  l_paComAddRepLut2_4,  s_paAddL2Std.ctrlBitMap.t_pa_lut2_ctrl_gtpu  
       set   s_runCxt.ctrlFlag.t_c2_disable_GTPU

l_paComAddRepLut2_4:
    // Put the command word in place
    zero  &s_l2Entry.ctrl,   SIZE(s_l2Entry.ctrl)
    mov   s_l2Entry.ctrl.b3, SUBS_L2_ENTRY_CTRL_ADD

    sbco  s_l2Entry,  cLut2Regs,  LUT2_REG_DATA0,  SIZE(s_l2Entry)
    
    // Should we perform queue diversion assistance
    qbbc  l_paComAddRepLut2_5,  s_paAddL2Std.ctrlBitMap.t_pa_lut2_ctrl_queue_divert 
    
        // forward the response packet to the Queue-diverion processing queue
        lbco    s_paAddLut2QueueDivert, PAMEM_CONST_CUSTOM, OFFSET_QUEUE_DIVERT_CFG, SIZE(s_paAddLut2QueueDivert)
        
        // Patch timestamp field with queue diversion information 
        sbco  r3,  cCdeOutPkt,  OFFSET(s_pktDescr.timestamp), SIZE(s_pktDescr.timestamp)
        
        
        xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr) 
        
        lbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#ifdef PASS_LAST_PDSP
        // Clear Bypass Flag
        clr   s_pktDescr.pktFlags.t_pktBypass
#else
        set   s_pktDescr.pktFlags.t_pktBypass
#endif   
        sbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
        
        // Add PA Config command
        mov   r1.b0,   PSH_CMD_CONFIG << 5
        sbco  r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr),  1 
        
        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        mov  s_cdeCmdPkt.psInfoSize,  4
        //mov  s_cdeCmdPkt.threadId,    THREADID_CDMA
        //mov  s_cdeCmdPkt.destQueue,   s_paAddLut2QueueDivert.destQueue
        mov  s_cdeCmdPkt.flowId,        s_paAddLut2QueueDivert.destFlowId
        //mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
        //mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE
        mov    r4.w0,   CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8)   
        sbco   s_paAddLut2QueueDivert.destQueue,    cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        xout XID_CDECTRL,             s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
        
        // Free the extended header
        mov  s_pktExtDescr.threadId,    THREADID_CDMA0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
   
        // command response counter
	    lbco  r1, cMailbox, 8, 4
	    add   r1, r1, 1
	    sbco  r1, cMailbox, 8, 4
        
        // set queue divert flag (command1)
        mov   r1, 1
	    sbco  r1, cMailbox, 4, 4

        // Wait for the set command to take hold
        wbs  s_flags.info.tStatus_Command1
        
        // wait for entry update
        wbc     s_flags.info.tStatus_Lut2AddDelBusy
        
        // wait for qeue diversion to be completed
        wbc     s_flags.info.tStatus_Command1

        jmp   f_mainLoop


l_paComAddRepLut2_5:
    jmp  f_cfgReply

    .leave cdeScope
    .leave configScope
    .leave lut2Scope
    .leave pktScope

// ********************************************************************************************
// * FUNCTION PURPOSE: Delete an entry from LUT2
// ********************************************************************************************
// * DESCRIPTION: An existing entry is deleted
// *
// *   Register Usage:  
// * 
// *   R0:   w0 - the packet length
// *   R1:    
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands
// *   R5:   |                  
// *   R6:          |  Command header (entry)
// *   R7:          |
// *   R8:          |
// *   R9:          |
// *   R10:             |  del LUT2 command
// *   R11:             |
// *   R12:             |
// *   R13:   
// *   R14:     
// *   R15:     
// *   R16:     
// *   R17:     
// *   R18:   | LUT2 key
// *   R19:       | LUT2 control
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
// ******************************************************************************************

    .using cdeScope
    .using configScope
    .using lut2Scope
    .using pktScope

f_paComDelLut2:

    // If there is an active add or delete the command must be rejected
    qbbc  l_paComDelLut2_0,  s_flags.info.tStatus_Lut2AddDelBusy

        mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT2_ADD_BUSY
        xout XID_CDEDATA,             s_paCmd1.commandResult,                SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply

l_paComDelLut2_0:

    // Keep a count of the number of entries. Return a warning if
    // it seems to drop below zero
    qbne  l_paComDelLut2_1,   s_runCxt.c2NumInL2Table,   0
        mov s_paCmd1.commandResult,  PA_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT
        xout XID_CDEDATA,             s_paCmd1.commandResult,                SIZE(s_paCmd1.commandResult)
        jmp l_paComDelLut2_2

l_paComDelLut2_1:
    // Return success
    mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_SUCCESS
    xout XID_CDEDATA,             s_paCmd1.commandResult,    SIZE(s_paCmd1.commandResult)

    sub  s_runCxt.c2NumInL2Table, s_runCxt.c2NumInL2table,   1

l_paComDelLut2_2:

    // read in delete command parameters
    xin  XID_CDEDATA,  s_paDelL2Custom,  SIZE(s_paDelL2Custom)

    mov   s_l2Entry.key1.b0,    0 
    add   s_l2Entry.key1.b1,    s_paDelL2Custom.l4Type, s_paDelL2Custom.index     
    mvid  s_l2Entry.key2,       *&s_paDelL2Custom.match0
    mov   s_l2Entry.key1.w2,    s_paDelL2Custom.srcVC

    zero &s_l2Entry.ctrl,     SIZE(s_l2Entry.ctrl)
    mov   s_l2Entry.ctrl.b3,  SUBS_L2_ENTRY_CTRL_DEL
    
    // We may need to update b3 if custom 
    qbeq l_paComDelLut2_4,  s_paDelL2Custom.type,  PA_COM_ADD_LUT2_STANDARD 
    qbge l_paComDelLut2_3,  s_paDelL2Custom.index, PA_MAX_C2_CUSTOM_TYPES
       mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX
       xout XID_CDEDATA,    s_paCmd1.commandResult,   SIZE(s_paCmd1.commandResult)
       jmp  f_cfgReply
    
l_paComDelLut2_3:    
       // Calcualte the offset: 16-byte entries 
       lsl  r1.w0,    s_paDelL2Custom.index,  4
       add  r1.w0,    r1.w0,  OFFSET_CUSTOM_C2 + OFFSET(struct_paC2Custom.bitSet) 
       lbco r1.b3,    PAMEM_CONST_CUSTOM2,  r1.w0,  SIZE(struct_paC2Custom.bitSet)
       //  Update the MS byte
       or   s_l2Entry.key2.b3,  s_l2Entry.key2.b3, r1.b3 
       jmp  l_paComDelLut2_5

l_paComDelLut2_4:
    
    // Re-enable GTPU parsing if GTPU flag is set
    qbbc  l_paComDelLut2_5,  s_paDelL2Std.ctrlBitMap.t_pa_lut2_ctrl_gtpu  
    clr   s_runCxt.ctrlFlag.t_c2_disable_GTPU

l_paComDelLut2_5:
    sbco  s_l2Entry.key1,  cLut2Regs, LUT2_REG_ADDDEL_KEY0, SIZE(s_l2Entry.ctrl)+ 2*SIZE(s_l2Entry.key1)

    jmp  f_cfgReply

    .leave cdeScope
    .leave configScope
    .leave lut2Scope
    .leave pktScope

// *********************************************************************************************
// * FUNCTION PURPOSE: Initialize the classify2 PDSP
// *********************************************************************************************
// * DESCRIPTION: One time setup for the PDSP
// *
// *********************************************************************************************

    .using initScope

f_c2LocalInit:

	// Reset the LUT2 table to clear all entries. It will take 8192 cycles to
	// clear the table. Any value written will cause the reset, but just
	// to be safe a non-zero value will be written
	mov r3, 1
	sbco r3, cLut2Regs, LUT2_REG_SOFTRESET, 4

    // The run context
    // TBD: duplicate code
    //zero  &s_runCxt,  SIZE(s_runCxt)
    //lbco   s_runCxt.pdspId,  PAMEM_CONST_PDSP_INFO, OFFSET_ID,  1   // PDSP ID

    // The call table (per PDSP initialization)
    // TBD: may need more entries
    mov s_headerParse.c2ParseUdp,     f_c2ParseUdp
    mov s_headerParse.c2ParseUdpLite, f_c2ParseUdpLite
    mov s_headerParse.c2ParseTcp,     f_c2ParseTcp
    mov s_headerParse.c2ParseGtpu,    f_c2ParseGtpu
    mov s_headerParse.c2ParseEspDec,  f_c2ParseEspDec
    mov s_headerParse.c2ParseCustom,  f_c2ParseCustom

    sbco s_headerParse.c1ParseMac, PAMEM_CONST_PARSE, 0, SIZE(s_headerParse)
    
    // Custom Classify 2  (16*16)
    zero  &r0, 64
    sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2, 4*16
    sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64, 4*16
    sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64*2, 4*16
    sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64*3, 4*16
   
    
#ifdef PASS_FIRST_PDSP    
    // Initialize the next Fail Route base table (4 * 8 = 32 byte)
    // Dead code: not used by classify2
    mov32 s_nextFailRouteBase.pdsp0,    PAMEM_NFAIL_BASE_ADDR_PDSP0          
    mov32 s_nextFailRouteBase.pdsp1,    PAMEM_NFAIL_BASE_ADDR_PDSP1          
    mov32 s_nextFailRouteBase.pdsp2,    PAMEM_NFAIL_BASE_ADDR_PDSP2          
    mov32 s_nextFailRouteBase.pdsp3,    PAMEM_NFAIL_BASE_ADDR_PDSP3          
    mov32 s_nextFailRouteBase.pdsp4,    PAMEM_NFAIL_BASE_ADDR_PDSP4          
    mov32 s_nextFailRouteBase.pdsp5,    PAMEM_NFAIL_BASE_ADDR_PDSP5          
    mov32 s_nextFailRouteBase.pdsp6,    PAMEM_NFAIL_BASE_ADDR_PDSP6          
    mov32 s_nextFailRouteBase.pdsp7,    PAMEM_NFAIL_BASE_ADDR_PDSP7  
    
    sbco  s_nextFailRouteBase.pdsp0, PAMEM_CONST_NFAIL_BASE,   0, SIZE(s_nextFailRouteBase)         
    
#endif
    
    ret

    .leave initScope
    
// *********************************************************************************
// * FUNCTION PURPOSE: SCTP CRC Verification Processing
// *********************************************************************************
// * DESCRIPTION: Verify the SCTP CRC calculation result with pre-stored data
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
// *   R6:        | SCTP CRC value  
// *   R7:           |
// *   R8:           | Not used 
// *   R9:           |  
// *   R10:          |
// *   R11:          |  
// *   R12:          |   
// *   R13:          |    
// *   R14:             |                                            
// *   R15:             | Extended packet descriptor                                           
// *   R16:             |                                            
// *   R17:             |  
// *   R18:             |  
// *   R19:          
// *   R20:       | CRC context
// *   R21:       |
// *   R22:     |
// *   R23:     |
// *   R24:     |  Packet context     
// *   R25:     |       
// *   R26:     |  
// *   R27:     |
// *   R28:  
// *   R29:  
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************

.using cdeScope
.using sctpScope
.using pktScope

#ifdef PASS_VERIFY_SCTP_CRC

 f_paSctpVerifyCRC:
    // Read in the CRC context 
    xin  XID_CDEDATA,  s_sctpCrcCxt, SIZE(s_sctpCrcCxt)
 
    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,               SIZE(s_cdeCmdWd)
 
    // Scroll to the CRC location in the packet must be within the CDE range
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    sub  s_cdeCmdWd.byteCount,  s_sctpCrcCxt.offset,  s_pktExtDescr.mopLength  
    xout XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)
    
    // Write back the original CRC
    mov   r6,   s_sctpCrcCxt.crc
    mov   r0.b0, s_sctpCrcCxt.crcSize       
    xout  XID_CDEDATA,  r6, b0
    
    qbeq l_paSctpVerifyCRC1,      s_sctpCrcCxt.crc,     s_pktExtDescr.crcValue
        // CRC is incorrect
       wbs  s_flags.info.tStatus_CDEOutPacket
        
       // Set the CRC error flag
       lbco  r1.b0,  cCdeOutPkt,  OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
       set   r1.b0.t_pkt_desc_err_flag_chksum2
       sbco  r1.b0,  cCdeOutPkt,  OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
    
l_paSctpVerifyCRC1:
    // Update and copy the pkt context
    clr   s_pktCxt.flags.t_flag_crc_verify  
    mov  s_pktCxt5.l3offset2,   s_pktCxt5.l3l5Offset
    sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)
    clr   s_pktExtDescr.flags.fDoCRC
    
    // Ready to forward the packet
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    
#ifndef PASS_LAST_PDSP
    lbco  r1.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
    set   r1.w0.t_pktBypass
    sbco  r1.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#endif        

    xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
    xout  XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
    jmp   f_mainLoop
    
#endif    
    
.leave cdeScope  
.leave sctpScope
.leave pktScope
    
    

// **********************************************************************************************
// * FUNCTION PURPOSE: Parse a UDP header
// **********************************************************************************************
// * DESCRIPTION: The UDP dest port is added to the LUT and the search is initiated
// *
// *    On entry:
// *            - the CDE is at the start of the UDP header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP or SUBS_ACTION_PARSE (GTPU only)
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - the CDE is at the first byte after the UDP header
// *            - startOffset is adjusted by the UDP header size 
// *            - s_next.Hdr is set as PA_HDR_UNKNOWN if non-GTPU, otherwise, it is set to PA_HDR_GTPU
// *            
// *   Register Usage:  
// * 
// *   R0:   
// *   R1:    
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands     |
// *   R5:   |                   | LUT2 search key
// *   R6:          |  UDP header
// *   R7:          |
// *   R8:          
// *   R9:          
// *   R10:             
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:     | Extender Header
// *   R15:     |
// *   R16:     |
// *   R17:     |
// *   R18:     |
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
// ***********************************************************************************************

    .using cdeScope
    .using pktScope
    .using udpScope
    .using lut2Scope
    .using custC2Scope

f_c2ParseUdp:

    // Record parsing UDP
    set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_UDP
    mov s_pktCxt.l4Offset,      s_pktCxt.startOffset

    // Read in the UDP header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udp,  SIZE(s_udp) + SIZE(s_ipsecNatT)

#ifdef PASS_PROC_L4_CHECKSUM

    // If the checksum value is non-zero, configure the CDE to compute the checksum
    qbeq  l_c2ParseUdp0,  s_udp.chksum,  0

        add s_pktCxt.pseudo,       s_pktCxt.pseudo, s_udp.len
        adc s_pktCxt.pseudo,       s_pktCxt.pseudo, IP_PROTO_NEXT_UDP
        adc s_cdeCmdChk.initSum,   s_pktCxt.pseudo, s_pktExtDescr.mopCsum
        adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum, 0
        sub s_cdeCmdChk.byteLen,   s_udp.len,  s_pktExtDescr.mopLength 
        mov s_cdeCmdChk.options,   0
        mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_VALIDATE

        xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)

#endif

l_c2ParseUdp0:
    // UDP length includes the udp header, to the end length is computed 
    // before updating the start offset.
    add  r0.w0,    s_pktCxt.startOffset,  s_udp.len
    qbge l_c2ParseUdp0_1, r0.w0,    s_pktCxt.endOffset   
        // The end of UDP payload goes beyond the end of packet, it must be an illegal packet 
       mov s_param.action, SUBS_ACTION_EXIT
       and s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
       or  s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT
       ret
    
l_c2ParseUdp0_1:    
    mov  s_pktCxt.endOffset,    r0.w0 
    add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  UDP_HEADER_LEN_BYTES
    mov  s_pktCxt5.l3offset2,   s_pktCxt5.l3l5Offset
    mov  s_pktCxt.l5Offset,     s_pktCxt.startOffset

    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_UDP_PKTS
    
    // Verify whether GTPU parsing is enabled
    qbbs  l_c2ParseUdp1, s_runCxt.ctrlFlag.t_c2_disable_GTPU

    // A switch to GTPU mode is made based on destination UDP port
    mov   r0.w0,       UDP_TCP_DEST_PORT_GTP
    qbne  l_c2ParseUdp1,  s_udp.dst,  r0.w0 
 
       // On match change the next parse type to GTPU and return
       set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_GTPU
       mov s_next.Hdr,  PA_HDR_GTPU

       // Advance past the end of the header 
       mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
       mov s_cdeCmdWd.byteCount,   SIZE(s_udp)
       xout XID_CDECTRL,           s_cdeCmdWd,             SIZE(s_cdeCmdWd)
       ret

l_c2ParseUdp1:

#ifdef PASS_PROC_IPSEC_NAT_T

    // Verify whether IPSEC NAT-T  parsing is enabled
    qbbc  l_c2ParseUdp2, s_runCxt.ctrlFlag.t_c2_enable_IPSEC_NAT_T

    // A switch to IPSEC_NAT_T mode is made based on destination or source UDP port
    lbco  r0.w0,  PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG + OFFSET(struct_ipsec_nat_t_cfg.udpPort),  SIZE(struct_ipsec_nat_t_cfg.udpPort)
    qbeq  l_c2ParseUdpNatT_1,  s_udp.dst,  r0.w0 
    qbne  l_c2ParseUdp2,       s_udp.src,  r0.w0

l_c2ParseUdpNatT_1:
       // IPSEC NAT_T parsing
       // Keepalive packet: UDP length == 9, payload = 0xFF
       // Control packet: UDP length > 12 SPI = 0
       // IPSEC Data Packet: UDP length > 12 SPI != 0
       // Error Packet UDP length <= 12
       
       // Common Operations for all NAT_T packet
       set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_IPSEC_NAT_T
       mov s_next.Hdr,  PA_HDR_UNKNOWN
       and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
       mov s_param.action, SUBS_ACTION_EXIT

       qbge l_c2ParseUdpNatT_3, s_udp.len, SIZE(s_udp) + SIZE(s_ipsecNatT) 
            qbeq l_c2ParseUdpNatT_2, s_ipsecNatT.spi, 0    
                // IPSEC Data packet
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_DATA << PKT_EIDX_SHIFT
                mov s_next.Hdr,  PA_HDR_ESP
                ret   
       
l_c2ParseUdpNatT_2: 
                // NAT_T Control packet      
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_CTRL << PKT_EIDX_SHIFT
                ret
                
l_c2ParseUdpNatT_3:
            qbne l_c2ParseUdpNatT_4,    s_udp.len,          SIZE(s_udp) + SIZE(s_ipsecNatT2)
            qbne l_c2ParseUdpNatT_4,    s_ipsecNatT2.data,  0xFF
                // NAT-T Keepalive Packet
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_KEEPALIVE << PKT_EIDX_SHIFT
                ret
       
l_c2ParseUdpNatT_4:       
                // NAT-T Error Packet
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_FAIL << PKT_EIDX_SHIFT
                ret
                
#endif                

l_c2ParseUdp2:

    // Do the lookup
    zero &s_l2Search,   SIZE(s_l2Search)
    mov  s_l2Search.dstPort,  s_udp.dst
    mov  s_l2Search.type,     PA_L4_PKT_TYPE_PORT16
  // Check to see whether virtual link is used 
  qbbs l_c2ParseUdp_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l2Search.srcVC,  s_pktCxt.phyLink
    jmp  l_c2ParseUdp_link_end
l_c2ParseUdp_vLink:    
    mov  s_l2Search.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l2Search.srcVC.b1, s_l2Search.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c2ParseUdp_link_end:
    
    xout XID_LUT2CMD,  s_l2Search,  SIZE(s_l2Search)

    mov s_param.action, SUBS_ACTION_LOOKUP

    ret

    .leave cdeScope
    .leave pktScope
    .leave udpScope
    .leave lut2Scope
    .leave custC2Scope


// **********************************************************************************************
// * FUNCTION PURPOSE: Parse a UDP-Lite header
// **********************************************************************************************
// * DESCRIPTION: The UDP-Lite header is parsed
// *
// *    On entry:
// *            - the CDE is at the start of the UDP header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP or SUBS_ACTION_PARSE (GTPU only)
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - the CDE is at the first byte after the UDP-lite header
// *            - startOffset is adjusted by the UDP header size 
// *            - s_next.Hdr is set as PA_HDR_UNKNOWN if non-GTPU, otherwise, it is set to PA_HDR_GTPU
// *   Register Usage:  
// * 
// *   R0:   
// *   R1:    
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands     |
// *   R5:   |                   | LUT2 search key
// *   R6:          |  UDP header
// *   R7:          |
// *   R8:          
// *   R9:          
// *   R10:             
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:     | Extender Header
// *   R15:     |
// *   R16:     |
// *   R17:     |
// *   R18:     |
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
// ************************************************************************************************


    .using cdeScope
    .using pktScope
    .using udpScope
    .using lut2Scope
    .using custC2Scope  

f_c2ParseUdpLite:
        
    // Record parsing UDP
    set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_UDP
    mov s_pktCxt.l4Offset,      s_pktCxt.startOffset

    // Read in the UDP lite header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udpLite,  SIZE(s_udpLite)
    
#ifdef PASS_PROC_L4_CHECKSUM

    // Setup for the UDP checksum
    // Get the IP length
    sub  r2.w0,  s_pktCxt.startOffset, s_pktCxt.endOffset

    mov  s_cdeCmdChk.bytelen,  s_udpLite.chkCov

    qbne  l_c2ParseUdpLite0,  s_udpLite.chkCov,  0
        mov  s_cdeCmdChk.bytelen,   r2.w0          // 0 means the whole packet

l_c2ParseUdpLite0:

    qblt l_c2ParseUdpLite2,  s_cdeCmdChk.bytelen,  r2.w0
    qbgt l_c2ParseUdpLite2,  s_cdeCmdChk.bytelen,  UDP_LITE_HEADER_LEN_BYTES

    sub s_cdeCmdChk.bytelen,   s_cdeCmdChk.bytelen, s_pktExtDescr.mopLength  
    add s_pktCxt.pseudo,       s_pktCxt.pseudo, r2.w0
    adc s_pktCxt.pseudo,       s_pktCxt.pseudo, IP_PROTO_NEXT_UDP_LITE
    adc s_cdeCmdChk.initSum,   s_pktCxt.pseudo, s_pktExtDescr.mopCsum
    adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum, 0
    mov s_cdeCmdChk.options,   0
    mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_VALIDATE

    xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)

#endif

    // UDP-lite length includes the udp header
    add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  UDP_LITE_HEADER_LEN_BYTES
    mov  s_pktCxt5.l3offset2,   s_pktCxt5.l3l5Offset
    mov  s_pktCxt.l5Offset,     s_pktCxt.startOffset

    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_UDP_PKTS
    
    // Verify whether GTPU parsing is enabled
    qbbs  l_c2ParseUdpLite1, s_runCxt.ctrlFlag.t_c2_disable_GTPU
    
    // A switch to GTPU mode is made based on destination UDP port
    mov   r0.w0,       UDP_TCP_DEST_PORT_GTP
    qbne  l_c2ParseUdpLite1,  s_udpLite.dst,  r0.w0 
 
       // On match change the next parse type to custom and return
       mov s_next.Hdr,  PA_HDR_GTPU

       // Advance past the end of the header 
       mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
       mov s_cdeCmdWd.byteCount,   SIZE(s_udpLite)
       xout XID_CDECTRL,           s_cdeCmdWd,             SIZE(s_cdeCmdWd)

       ret

l_c2ParseUdpLite1:

    // Do the lookup
    zero &s_l2Search,   SIZE(s_l2Search)
    mov  s_l2Search.dstPort,  s_udpLite.dst
    mov  s_l2Search.type,     PA_L4_PKT_TYPE_PORT16
  // Check to see whether virtual link is used 
  qbbs l_c2ParseUdpLite_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l2Search.srcVC,  s_pktCxt.phyLink
    jmp  l_c2ParseUdpLite_link_end
l_c2ParseUdpLite_vLink:    
    mov  s_l2Search.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l2Search.srcVC.b1, s_l2Search.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c2ParseUdpLite_link_end:
    
    
    xout XID_LUT2CMD,  s_l2Search,  SIZE(s_l2Search)

    mov s_param.action, SUBS_ACTION_LOOKUP
    ret
    
#ifdef PASS_PROC_L4_CHECKSUM

l_c2ParseUdpLite2:

    // UDP lite checksum coverage failed
    and s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  NOT_PKT_EIDX_MASK
    or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_UDP_LITE_FAIL << PKT_EIDX_SHIFT

    mov s_param.action, SUBS_ACTION_EXIT
    ret
    
#endif    

    .leave cdeScope
    .leave pktScope
    .leave udpScope
    .leave lut2Scope
    .leave custC2Scope


// ***********************************************************************************
// * FUNCTION PURPOSE: Process a TCP header
// ***********************************************************************************
// * DESCRIPTION: The TCP header is parsed and a LUT2 initiated
// *
// *   Register Usage:  
// * 
// *   R0:   
// *   R1:   | scratch
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands     |
// *   R5:   |                   | LUT2 search key
// *   R6:          |  TCP header
// *   R7:          |
// *   R8:          |
// *   R9:          |
// *   R10:         |    
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:     | Extender Header
// *   R15:     |
// *   R16:     |
// *   R17:     |
// *   R18:     |
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
// *************************************************************************************

    .using cdeScope
    .using pktScope
    .using tcpScope
    .using lut2Scope

f_c2ParseTcp:

    // Record parsing TCP
    set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_TCP
    mov s_pktCxt.l4Offset,  s_pktCxt.startOffset

    // Read in the TCP header
    xin  XID_CDEDATA,  s_tcp,  SIZE(s_tcp)

    // Configure and initiate the lookup
    zero &s_l2Search,   SIZE(s_l2Search)
    mov  s_l2Search.dstPort,  s_tcp.dst
    mov  s_l2Search.type,     PA_L4_PKT_TYPE_PORT16
  // Check to see whether virtual link is used 
  qbbs l_c2ParseTcp_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l2Search.srcVC,  s_pktCxt.phyLink
    jmp  l_c2ParseTcp_link_end
l_c2ParseTcp_vLink:    
    mov  s_l2Search.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l2Search.srcVC.b1, s_l2Search.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c2ParseTcp_link_end:
    
    xout XID_LUT2CMD,  s_l2Search,  SIZE(s_l2Search)

#ifdef PASS_PROC_L4_CHECKSUM

    // Setup the CDE to do the L4 checkusm
    sub s_cdeCmdChk.byteLen,   s_pktCxt.endOffset,  s_pktCxt.startOffset
    sub s_cdeCmdChk.byteLen,   s_cdeCmdChk.byteLen, s_pktExtDescr.mopLength
    add s_pktCxt.pseudo,       s_pktCxt.pseudo,     s_cdeCmdChk.byteLen
    adc s_pktCxt.pseudo,       s_pktCxt.pseudo,     IP_PROTO_NEXT_TCP
    adc s_cdeCmdChk.initSum,   s_pktCxt.pseudo, s_pktExtDescr.mopCsum
    adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum, 0
    mov s_cdeCmdChk.options,   0
    mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_VALIDATE

    xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
#endif

    // Adjust the start offset past the tcp header
    lsr  r1.w0,                s_tcp.offset_ecn,       4     // extract the length (32 bit length)
    lsl  r1.w0,                r1.w0,                  2      // convert to byte length
    add  s_pktCxt.startOffset, s_pktCxt.startOffset,   r1.w0
    mov  s_pktCxt5.l3offset2,  s_pktCxt5.l3l5Offset
    mov  s_pktCxt.l5Offset,    s_pktCxt.startOffset

    mov  s_param.action,  SUBS_ACTION_LOOKUP
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_TCP_PKTS
    ret

    .leave cdeScope
    .leave pktScope
    .leave tcpScope
    .leave lut2Scope
    
// ******************************************************************************************
// * FUNCTION PURPOSE: Parse a GTPU header
// ******************************************************************************************
// * DESCRIPTION: The custom header fields are extracted, masked, and entered into the LUT
// *
// *    GTPU processing algorithm:
// *        - Version <> 1 or PT <> ==> Error Queue
// *        - Message Type 1,2, 26, 31 254 ==> User specified exception routes
// *        - Message Type 255: TEID lookup
// *           - If ext header present and it is not 0, or 0xc0 ==> Error Queue
// *           - If ext header = 0xc0, but the next one is not 0 ==> Error Queue
// *           - If ext header = 0xc0, Extract the PDU number to the pktCtx and set the corresponding flag
// *           - TEID lookup
// *
// *    On entry:
// *            - the CDE is at the start of the GTPU header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP  or SUBS_ACTION_EXIT
// *            - the CDE is at the first byte after the GTP header and extention header only if loopkup is required
// *            - startOffset is adjusted to the GTPU payload only if lookup is required
// *            - s_next.Hdr is set as PA_HDR_UNKNOWN
// *
// *   Register Usage:  
// * 
// *   R0:   
// *   R1:   | scratch r1.b1 GTPU header size
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands     |
// *   R5:   |                   | LUT2 search key
// *   R6:     | GTPU header
// *   R7:     |     
// *   R8:     |     
// *   R9:     |     
// *   R10:         
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:     | Extender Header
// *   R15:     |
// *   R16:     |
// *   R17:     |
// *   R18:     |
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
// *****************************************************************************************

    .using cdeScope
    .using pktScope
    .using gtpScope
    .using lut2Scope

f_c2ParseGtpu:

    // Record parsing GTP
    set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_GTPU
    
    // Assume Error case
    and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
    mov s_param.action,      SUBS_ACTION_EXIT
    mov s_next.Hdr,          PA_HDR_UNKNOWN
    
    // Read in the GTPU header
    xin  XID_CDEDATA,  s_gtp,  SIZE(s_gtp)
    
    // Extract version number
    and  r1.b0, s_gtp.ctrl,  GTP_VER_MASK
    
    // Only version 1 should be handled
    qbeq    l_c2ParseGtpu1, r1.b0,  GTP_VER_GTP_V1
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_FAIL << PKT_EIDX_SHIFT  
        ret  
      
l_c2ParseGtpu1:  
    // Protocol Type must be 1
    qbbs    l_c2ParseGtpu2, s_gtp.ctrl.t_gtp_ctrl_pt_1  
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_FAIL << PKT_EIDX_SHIFT  
        ret  
        
l_c2ParseGtpu2:
    qbne    l_c2ParseGtpu3, s_gtp.msgType,  255
    
l_c2ParseGtpu2_0:    
        mov r1.b1, GTPV1_HEADER_LEN_BYTES 
        and r1.b0, s_gtp.ctrl, GTP_CTRL_MASK 
        
        qbeq l_c2ParseGtpu2_1,  r1.b0, 0
            // basic extension header exists 
            add r1.b1, r1.b1, GTPV1_EXT_HDR_LEN_BYTES
            
            qbbc l_c2ParseGtpu2_1, s_gtp.ctrl.t_gtp_ctrl_extHdr
            // extension header exist
                qbeq    l_c2ParseGtpu2_1, s_gtp.nextHdr, GTP_NEXTHDR_NONE 
                
                    qbne  l_c2ParseGtpu_Err, s_gtp.nextHdr, GTP_NEXTHDR_PDU_NUM  
                        // PDU header exists
                        qbne  l_c2ParseGtpu_Err, s_gtp.nextHdr2, GTP_NEXTHDR_NONE
                            // Process the PDU number
                            add r1.b1, r1.b1, GTPV1_EXT_HDR_LEN_BYTES
                            set s_pktCxt.flags.t_flag_gtpu_seqnum_present
                            sbco    s_gtp.pduNum,  cCdeOutPkt,  SIZE(s_pktDescr) + SIZE(s_pktCxt),  SIZE(s_gtp.pduNum)   
                            
                            // Pass Through for TEID lookup
                
l_c2ParseGtpu2_1:
        // Adjust the start offset to the GTPU paylaod    
        add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  r1.b1
        
        // Advance past the end of the header 
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  r1.b1
        xout XID_CDECTRL,           s_cdeCmdWd,             SIZE(s_cdeCmdWd)
        
        // TEID lookup
        zero &s_l2Search2,   SIZE(s_l2Search2)
        mov  s_l2Search2.srcVC,   s_pktCxt.phyLink
        mov  s_l2Search2.data0,   s_gtp.teid.w2
        mov  s_l2Search2.data1,   s_gtp.teid.w0
        mov  s_l2Search2.type,    PA_L4_PKT_TYPE_PORT32
  // Check to see whether virtual link is used 
  qbbs l_c2ParseGtpu_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l2Search2.srcVC,  s_pktCxt.phyLink
    jmp  l_c2ParseGtpu_link_end
l_c2ParseGtpu_vLink:    
    mov  s_l2Search2.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l2Search2.srcVC.b1, s_l2Search2.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c2ParseGtpu_link_end:
        
        xout XID_LUT2CMD,  s_l2Search2,  SIZE(s_l2Search2)
        mov  s_param.action,    SUBS_ACTION_LOOKUP
        ret
    
        
l_c2ParseGtpu3:
    qbne    l_c2ParseGtpu4, s_gtp.msgType,  1
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MESSAGE_TYPE_1 << PKT_EIDX_SHIFT  
        ret  
         
l_c2ParseGtpu4:
    qbne    l_c2ParseGtpu5, s_gtp.msgType,  2
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MESSAGE_TYPE_2 << PKT_EIDX_SHIFT  
        ret  

l_c2ParseGtpu5:
    qbne    l_c2ParseGtpu6, s_gtp.msgType,  26
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MESSAGE_TYPE_26 << PKT_EIDX_SHIFT  
        ret  

l_c2ParseGtpu6:
    qbne    l_c2ParseGtpu7, s_gtp.msgType, 31 
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MESSAGE_TYPE_31 << PKT_EIDX_SHIFT  
        ret  
         
l_c2ParseGtpu7:
    qbne    l_c2ParseGtpu_Err, s_gtp.msgType, 254
      // Check if the routing feature of treating End marker same as G-PDU is enabled
      qbbc  l_c2ParseGtpu7_0, s_runCxt.ctrlFlag.t_c2_GTPU_route_msg254_as_msg255
        // Route same as message 255 if the message type is 254
        jmp l_c2ParseGtpu2_0
      
l_c2ParseGtpu7_0:    
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_MESSAGE_TYPE_254 << PKT_EIDX_SHIFT  
        ret  
    
l_c2ParseGtpu_Err:
        or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GTPU_FAIL << PKT_EIDX_SHIFT  
        ret  

    .leave cdeScope
    .leave pktScope
    .leave gtpScope
    .leave lut2Scope

// ******************************************************************************************
// * FUNCTION PURPOSE: Parse a custom header
// ******************************************************************************************
// * DESCRIPTION: The custom header fields are extracted, masked, and entered into the LUT
// *
// *   Register Usage:  
// * 
// *   R0:   
// *   R1:   | scratch
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands     |
// *   R5:   |                   | LUT2 search key
// *   R6:   | packet data ==> one byte at a time       
// *   R7:          
// *   R8:          
// *   R9:          
// *   R10:         
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:  | custom2 Control data   
// *   R15:  |   
// *   R16:  |   
// *   R17:  |   
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
// *   R29:  c2RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// *****************************************************************************************
    .using cdeScope
    .using pktScope
    .using custC2Scope
    .using lut2Scope

f_c2ParseCustom:

    set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_CUSTOM
    
    // eIndex should stote the custom Index
    // offset = index * 16 + OFFSET_CUSTOM_C2
    // TBD: Should we do error check here?
    lsr   r1.b0,  s_pktCxt.eId_portNum_nextHdr.b1,  PKT_EIDX_SHIFT 
    lsl   r1.w0,  r1.b0,  4
    add   r1.w0,  r1.w0,  OFFSET_CUSTOM_C2 

    // Read the custom parse information from scratch
    lbco  s_c2Cust,  PAMEM_CONST_CUSTOM2,  r1.w0,  SIZE(s_c2Cust)
    
    add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_c2Cust.hdrSize
    mov  s_pktCxt5.l3offset2,   s_pktCxt5.l3l5Offset
    
    add   s_l2SCust.type, s_c2Cust.idx, PA_L4_PKT_TYPE_CUSTOM 

    // store the current offset, update while scrolling
    mov  r1.w0,                 0
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE

    zero &s_l2SCust,  SIZE(s_l2SCust)  
    // Scroll to the first byte
    sub  s_cdeCmdWd.byteCount,  s_c2Cust.offset3,  r1.w0
    add  r1.w0,                 r1.w0,             s_cdeCmdWd.byteCount
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

    // Get the byte, mask it and put it in place
    xin  XID_CDEDATA,   r6.b3,   1
    and  s_l2SCust.cb0, r6.b3,   s_c2Cust.bitMask3
    or   s_l2SCust.cb0, s_l2SCust.cb0,  s_c2Cust.bitSet 

    // Next byte
    sub  s_cdeCmdWd.byteCount,  s_c2Cust.offset2,  r1.w0
    add  r1.w0,                 r1.w0,             s_cdeCmdWd.byteCount
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

    // Get the byte, mask it and put it in place
    xin  XID_CDEDATA,   r6.b3,   1
    and  s_l2SCust.cb1, r6.b3,   s_c2Cust.bitMask2

    // Next byte
    sub  s_cdeCmdWd.byteCount,  s_c2Cust.offset1,  r1.w0
    add  r1.w0,                 r1.w0,             s_cdeCmdWd.byteCount
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

    // Get the byte, mask it and put it in place
    xin  XID_CDEDATA,   r6.b3,   1
    and  s_l2SCust.cb2, r6.b3,   s_c2Cust.bitMask1

    // Last byte
    sub  s_cdeCmdWd.byteCount,  s_c2Cust.offset0,  r1.w0
    add  r1.w0,                 r1.w0,             s_cdeCmdWd.byteCount
    xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

    // Get the byte, mask it and put it in place
    xin  XID_CDEDATA,   r6.b3,   1
    and  s_l2SCust.cb3, r6.b3,   s_c2Cust.bitMask0
        
    // The last byte can be from the packet or it can be the PDSP ID and index
    // of the previous lookup. Assume it will use the idx and then 
    // overwrite if the packet values are used
    mov s_l2SCust.srcVC, s_pktCxt.phyLink
    add s_l2SCust.type,  s_c2Cust.idx,  PA_L4_PKT_TYPE_CUSTOM
        

l_c2ParseCustom1:

    xout XID_LUT2CMD,                  s_l2SCust,             SIZE(s_l2SCust)
    mov  s_param.action,               SUBS_ACTION_LOOKUP
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C2_CUSTOM_PKTS
    ret

    .leave cdeScope
    .leave pktScope
    .leave custC2Scope
    .leave lut2Scope

// ***************************************************************************************
// * FUNCTION PURPOSE: Parse a decoded ESP header
// ***************************************************************************************
// * DESCRIPTION: After (optional) authentication and decryption, the ESP header is
// *              reparsed. the end offset must now point to the end of the ESP 
// *              trailer.
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:
// *   R4:   |  CDE commands
// *   R5:   |                   | LUT2 search key
// *   R6:          |  UDP header
// *   R7:          |
// *   R8:          
// *   R9:          
// *   R10:             
// *   R11:             
// *   R12:     
// *   R13:   
// *   R14:          |                                          
// *   R15:          |  extended packet dewscriptor                                        
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
// *
// **************************************************************************************/

    .using cdeScope
    .using pktScope

f_c2ParseEspDec:

   // Read in the padlen (r0.b1) and protocol (r0.b0) fields at the end of the packet
   // Note: It should be the absolute offset including Packet descriptor, PS Info.
   // sub    r0.w2,   s_pktCxt.endOffset,  s_pktCxt.startOffset
   // Note: Both the size of packet descriptor and PS Info are constants 
   //       We may need to enhance it for more general cases
   xin    XID_PINFO_A,  s_pktExtDescr,  SIZE(s_pktExtDescr)  
   add    r0.w2,   s_pktCxt.endOffset,  (32+32-2)  
   // Extra 8 bytes are inserted during EOAM enabled case
   qbbc l_c2ParseEspDec_1, s_runCxt.flags.t_c2_flags_enable_EOAM
     add  r0.w2,   r0.w2, 8
l_c2ParseEspDec_1:   

   sub    r0.w2,   r0.w2, s_pktExtDescr.mopLength
   
   // wait for the sideband data to be ready 
   wbc    s_flags.info.tStatus_CDEBusy
   lbco   r0.w0,   cCdeInPkt,           r0.w2,                  2

   // Note: the s_pktCtx.startOffset should already point to the next header 
   sub  s_pktCxt.endOffset,    s_pktCxt.endOffset,   r0.b1
   sub  s_pktCxt.endOffset,    s_pktCxt.endOffset,   2
   
   // TBD: Perform the standard pending check
   // sub   r0.w2,  r0.w2, r0.b1

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  r0.b0,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   
   // Always contiune to parse the next header 
   mov      s_param.action,    SUBS_ACTION_PARSE 

   ret

    .leave cdeScope
    .leave pktScope

#include "pacfgcmn.p"
