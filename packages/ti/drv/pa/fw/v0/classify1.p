// **************************************************************************************************************
// * FILE PURPOSE: Peform packet classification on PDSPS with a LUT1
// **************************************************************************************************************
// * FILE NAME: classify1.p
// *
// * DESCRIPTION: The PDSP code for L2 and L3 classification using a LUT1
// *
// **************************************************************************************************************
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

#define PDSP_CLASSIFY1  1
#define PASS_PROC_PKT_FORWARD

#include "pdsp_pa.h"
#include "pdsp_subs.h"
#include "pm_constants.h"
#include "parsescope.h"

    .origin      0
    .entrypoint  f_c1Init
    

f_c1Init:
    jmp f_c1Start
    
header:
    .codeword  HEADER_MAGIC
    .codeword  PASS_C1_VER

    .using   globalScopeC1

#ifdef PASS_GLOBAL_INIT

#include "meminit.p"

#endif


f_c1Start:

    call   f_c1LocalInit
    
    // Clear the mailbox 
    zero &r2, 12
    sbco r2, FIRMWARE_MBOX, 4, 12 

    // Write a non-zero value to mailbox slot 0. 
    mov   r2, 1
    sbco  r2, FIRMWARE_MBOX, 0, 4

    // Wait for the set command to take hold
    wbs  s_flags.info.tStatus_Command0

    // The host will clear mailbox slot 0 when it is ready for this PDSP to run
    wbc  s_flags.info.tStatus_Command0

#ifdef PASS_GLOBAL_INIT

    // Do common initialization if the host has requested it
    // The host requests a global init by writing a non-zero value into mailbox slot 1
    // (Before clearing mailbox slot 0)
    qbbc  l_c1Start0,  s_flags.info.tStatus_Command1

      call f_commonInit
      mov  r2, 0
      sbco r2, FIRMWARE_MBOX, 4, 4      // Clear the mailbox
      
#endif      

l_c1Start0:

    zero  &s_runCxt,      SIZE(s_runCxt)
    zero  &s_statsFlags,  SIZE(s_statsFlags)

    // Store the PDSP ID
    lbco  s_runCxt.flags,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1
    
    // Store the version number
    mov   r2.w0,   PASS_C1_VER & 0xFFFF
    mov   r2.w2,   PASS_C1_VER >> 16 
    sbco  r2,   PAMEM_CONST_PDSP_INFO,  OFFSET_VER,     4    


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
// *   R4:
// *   R5:    LUT1 Status (s_l1Status) / LUT1 Command (s_l1Cmd)  - lut1Scope
// *   R6:
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
// *   R20:
// *   R21:
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:  
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************************


   .using  lut1Scope
   .using  pktScope
   .using  cdeScope    

f_mainLoop:

#ifdef PASS_TIMESTAMP_OP
    // look for timer roll over
    qbbc l_mainLoop000, s_flags.info.tStatus_Timer
        set  s_flags.info.tStatus_Timer // set to clear timer event 
        lbco r1, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP,   8
        add r1, r1, 1
        adc r2, r2, 0
        sbco r1, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP,   8
#endif    


l_mainLoop000:

#ifdef PASS_PROC_IP_REASSEM    

   // Look for command (FIRMWARE_CMD_IP_REASSEM_CFG only) 
    qbbc l_mainLoop001, s_flags.info.tStatus_Command1
        // Process IP Reassembly Configuration Update
#ifdef TO_BE_DELETE        
        and  r1.b0,  s_runCxt.flags, SUBS_RUN_CXT_ID_MASK // get PDSP ID
        qbeq l_mainLoop00_ipReassem_1, r1.b0,   2         // Command for PDSP1 and PDSP2 only
            mov r1.w0, OFFSET_OUT_IP_REASSM_CFG
            jmp l_mainLoop00_ipReassem_2
          
l_mainLoop00_ipReassem_1:        
            mov r1.w0, OFFSET_IN_IP_REASSM_CFG
#else
        mov r1.w0, OFFSET_IP_REASSM_CFG
#endif
            
            
            
l_mainLoop00_ipReassem_2: 
        lbco s_paIpReassmCfg,   PAMEM_CONST_CUSTOM,  r1.w0,      SIZE(s_paIpReassmCfg)
        lbco s_ipReassmCxtInit, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxtInit.tfMapTemp)
        
        qbne l_mainLoop00_ipReassem_3,  s_paIpReassmCfg.numTrafficFlow, 0
            clr  s_runCxt.flags.t_ipReassmEn
            zero &s_ipReassmCxtInit,    OFFSET(s_ipReassmCxtInit.tfMapTemp)
            jmp  l_mainLoop00_ipReassem_4
               
l_mainLoop00_ipReassem_3:
            set  s_runCxt.flags.t_ipReassmEn   
            mov  s_ipReassmCxtInit.numTF,   s_paIpReassmCfg.numTrafficFlow
            mov  s_ipReassmCxtInit.queue,   s_paIpReassmCfg.destQueue
            mov  s_ipReassmCxtInit.flowId,  s_paIpReassmCfg.destFlowId
           
l_mainLoop00_ipReassem_4:
        sbco s_ipReassmCxtInit, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxtInit.tfMapTemp)
        
        // clear command flag
        mov  r1, 0
        sbco  r1, FIRMWARE_MBOX, FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET, 4
        
#endif
        // pass through
        
l_mainLoop001:

   // Look for command (FIRMWARE_CMD_PKT_CTRL_CFG only) 
    qbbc l_mainLoop002, s_flags.info.tStatus_Command3
        // Process Packet Verification Configuration Update
        lbco  r1, FIRMWARE_MBOX, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        // Clear the valid bit fields 
        not r1.b2, r1.b1
        // force clear valid bits to zero and retain valid bits positions
        and s_runCxt.flag2, s_runCxt.flag2, r1.b2

        // Clear garbage ctrl bit fields (since the sender already cleard it no need */
        // and r1.b0, r1.b0, r1.b1
        
        // set the run time flags
        or  s_runCxt.flag2, s_runCxt.flag2, r1.b0

        // clear the command flag
        mov   r1, 0           
        sbco  r1, FIRMWARE_MBOX, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        // pass through       

l_mainLoop002:
   // Look for held packets and completed lookup
   qbbc  l_mainLoop5, s_flags.info.tStatus_CDEHeldPacket

       qbbc  l_mainLoop3, s_runCxt.flags.t_activeLookup
       
       // There is already a packet pending to submit, 
       // We need to wait for the lookup to be complete 
       // Note: the t_pendingSubmit will be set when t_pendingConfig is set
       qbbc  l_mainLoop00, s_runCxt.flags.t_pendingSubmit
            wbc  s_flags.info.tStatus_Lut1Busy

l_mainLoop00:
           qbbs l_mainLoop5,  s_flags.info.tStatus_Lut1Busy

               // A lookup is complete for the held packet
               xin   XID_LUT1CMD,               s_l1Status,                              SIZE(s_l1Status)
               qbbc  l_mainLoop0,               s_l1Status.status.L1_STATUS_ENTRY_MATCH
               call  f_c1ForwardHeldPacketMatch
               jmp   l_mainLoop1

l_mainLoop0:                 
               call  f_c1ForwardHeldPacketNoMatch
l_mainLoop1:

               qbbc  l_mainLoop2, s_runCxt.flags.t_pendingSubmit

                   // An already parsed packet is waiting to be submitted to the LUT
                   clr   s_runCxt.flags.t_pendingSubmit
                   qbbs  l_mainLoop1_1, s_runCxt.flags.t_fakeLookup
                        // Initiate Lookup for pending packet
                        mov   s_l1Cmd.cmd,                   LUT1_CMD_SEARCHNS
                        xout  XID_LUT1CMD,                   s_l1Cmd,              4
                        jmp   l_mainLoop5 
                        
l_mainLoop1_1:
                   // fake lookup 
                    clr  s_runCxt.flags.t_fakeLookup
                    qbbs  l_mainLoop1_4,  s_runCxt.flag2.t_failLookup 
                    qbeq  l_mainLoop1_2,  s_param.action,  SUBS_ACTION_FWPKT
#ifdef PASS_PROC_DEF_ROUTE   
                    qbeq  l_mainLoop1_3,  s_param.action,  SUBS_ACTION_FWPKT2
#endif                    
                        // Forward Error Packet 
                        call  f_c1ForwardHeldPacketErrPkt
                        jmp  l_mainLoop2

l_mainLoop1_2:
                        // Forward packet now
                        ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_RELEASE 
                        xout XID_CDECTRL,    s_cdeCmdPkt,    4
                        jmp  l_mainLoop2
                       
#ifdef PASS_PROC_DEF_ROUTE   
l_mainLoop1_3:
                        // Forward BC/MC Packet 
                        call  f_c1ForwardHeldPacketBcMcPkt
                        jmp  l_mainLoop2
#endif                        
                        
                        
l_mainLoop1_4:          // Next Fail Route opertaion
                        clr  s_runCxt.flag2.t_failLookup
                        call  f_c1ForwardHeldPacketNoMatch  
                        
                        // pass through

l_mainLoop2:
                   clr  s_runCxt.flags.t_activeLookup   
                   jmp  l_mainLoop5
                


l_mainLoop3:
        // There is a held packet, but no active lookup. This should
        // never happen except for nextFail fakelookup
        qbbs  l_mainLoop1_1, s_runCxt.flag2.t_failLookup
l_mainLoop4:
            // write to the status register
            set s_statsFlags.event.t_nInvalidState
            // Discard the packet held that caused invalid state
            mov s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
            xout XID_CDECTRL, s_cdeCmdPkt, SIZE(s_cdeCmdPkt)
            wbc  s_flags.info.tStatus_CDEHeldPacket

        // if ( (pendingConfig == FALSE) && (newPacket == TRUE) ) then parse
l_mainLoop5:
    qbbs  fci_mainLoop6, s_runCxt.flags.t_pendingConfig
      qbbs  fci_mainLoop6, s_runCxt.flags.t_fakeLookup       
        qbbs f_c1Parse,  s_flags.info.tStatus_CDENewPacket

        // if (  (pendingConfig == TRUE) && (Lut1Busy == FALSE) ) then complete modify
fci_mainLoop6:
    qbbc  fci_mainLoop7,  s_runCxt.flags.t_pendingConfig
        qbbc  l_mainLoop8,  s_flags.info.tStatus_Lut1Busy


fci_mainLoop7:

    qbeq f_mainLoop, s_statsFlags.event, 0
        sbco   s_statsFlags.event, cStatistics,       OFFSET_STATS_FLAGS, 4
        zero  &s_statsFlags,       SIZE(s_statsFlags)

        jmp    f_mainLoop

     
l_mainLoop8:
    // Relative jump was out of range
    jmp f_paComLut1CompleteModify
    jmp fci_mainLoop7


    //jmp f_mainLoop
   
   .leave  lut1Scope
   .leave  pktScope
   .leave  cdeScope    
   

// **************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that had a match in the LUT1 search
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
// *   R8:   LUT1 info (s_l1f)                                       -
// *   R9:      |                                                    -
// *   R10:     |  Forward match Info (valid at function exit)       -  lut1MatchSCope
// *   R11:     |                                                    -
// *   R12:     |                                                    -
// *   R13:
// *   R14:
// *   R15:
// *   R16:
// *   R17:
// *   R18:
// *   R19:
// *   R20:
// *   R21:
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// **************************************************************************************************/

    .using  lut1Scope
    .using  lut1MatchScope
    .using  pktScope        // Used only for sizing


f_c1ForwardHeldPacketMatch:

    //     Record the match
    set    s_statsFlags.event.t_nTableMatch
    
    // Wait for the held packet to appear in the CDE
    // (Its almost certainly already there)
    wbs   s_flags.info.tStatus_CDEHeldPacket
    wbc   s_flags.info.tStatus_CDEBusy
    
    // Load the held pktInfo
    // The pkt context may be examized and patched
    lbco   s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    SIZE(s_pktCxt)

    // Load the routing info associated with LUT1 
    // This is the l1Info and match routing information
    lsl    r1.w0,    s_l1Status.index,             6                                     // l1 index * 64
    lbco   &s_l1f,   PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,   SIZE(s_l1f)+SIZE(s_fmPlace)  // l1Info + routing info

    // The following fields must be patched into the packet context with
    // Results from the lookup
    //   - previous LUT1 match bit
    //   - custom C2 bit
    //   - LUT1 PDSP ID
    //   - LUT1 match index
    // The 16 bit word s_pktCxt.eP1C2IdIdx will be created in r1.w0
    // and then patched in
  
    // The run context has the PDSP ID 
    and    r1.w0,     s_runCxt.flags,  SUBS_RUN_CXT_ID_MASK
    lsl    r1.w0,     r1.w0,           SUBS_PKT_CXT_L1ID_BIT_OFFSET
   
    //  Or in the LUT1 matching index
    or     r1.w0,  r1.w0,           s_l1Status.index

    // Set the bit to indicate a LUT1 match has occurred
    set    r1.w0.t_pl1Match                

    // Set the virtual link enable bit if necessary
    qbbc   l_c1ForwardHeldPacketMatch0, s_l1f.ctrlFlag.vlink_enable
        set    r1.w0.t_vlinkEn
        // Store virtual link num in s_pktCxt.vLinkNum for further stages
        sbco   s_l1fv.vLinkNum, cCdeHeldPkt, SIZE(s_pktDescr)+OFFSET(s_pktCxt.vLinkNum), 1

l_c1ForwardHeldPacketMatch0:

    // The pmatch, c2c, pdsp ID and LUT1 index are patched into psinfo
    mov    s_pktCxt.eP1C2IdIdx, r1.w0
    sbco   r1.w0,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eP1C2IdIdx), 2

//  jmp  stdPktForward   - save the cycle and just fall through

    .leave pktScope        
    .leave lut1MatchScope
    .leave lut1Scope

// ******************************************************************************************************
// * FUNCTION PURPOSE: Standard routing of a held packet
// ******************************************************************************************************
// * DESCRIPTION: A held packet is routed
// *              On Entry r9-r12 contains the forwarding information
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch  (r2.b0: psInfoSize, r2.b2: Thread ID of packet with command set, r2.b1: ehternet interface port)
// *   R3:    scratch
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:
// *   R7:
// *   R8:   LUT1 info (s_l1f)                                       -
// *   R9:      |                                                    -
// *   R10:     |  Forward match Info (valid at function exit)       -  lut1MatchSCope
// *   R11:     |                                                    -  Must be valid on Entry
// *   R12:     |                                                    -
// *   R13:
// *   R14:     
// *   R15:           | usrStats FIFO CB
// *   R16:           | usrStats Request
// *   R17:
// *   R18:
// *   R19:
// *   R20:
// *   R21:
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-unused     w0-function return address               -
// *   R31:  System Flags (s_flags)                                 -
// *
// *            
// ******************************************************************************************************

    .using  lut1MatchScope
    .using  cdeScope
    .using  pktScope
    .using  usrStatsFifoScope

f_stdHeldPktForward:
    // Record the effective packet size for user statistics update
    // It will casue 3 cycles for all forwarding packets regardless of statistics update or not
    //lbco    s_usrStatsReq.pktSize,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset) 
    mov s_usrStatsReq.pktSize,  s_pktCxt.endOffset
    
    // Load the hdrBitMasks_frag_portNum
    // Note that it should be loaded only once to prevent infinite re-entry for fragments routing
    //lbco s_pktCxt.hdrBitmask3_frag_portNum,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.hdrBitmask3_frag_portNum), SIZE(s_pktCxt.hdrBitmask3_frag_portNum)  
    
fci_stdHeldPktForward_retry:    
    // Default  thread Id for command set is PA_DEST_PA_M_0
    // The value will be changed to PA_DEST_CDMA only if QoS routing is required
    // The packet will be routed to QoS queues and then maybe forwarded to Q645 where command set can be executed
    mov     r2.b2,  PA_DEST_PA_M_0    

    // Do not release the packet ID if the packet remains in PA
    qbne  l_stdHeldPktForward1,  s_matchForward.forwardType,  PA_FORWARD_TYPE_PA
        //
        // IP Fragemnt packet can not be forwarded within PA
        //
        qbbs l_stdHeldPktForward11,  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
    
        // L1 Info handling
        qbbc l_stdHeldPktForward0_1,  s_l1f.ctrlFlag.custom_enable
        
        // Patch the startOffset and nextHdr
        sbco   s_l1f.nextHdrOffset,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.startOffset), SIZE(s_pktCxt.startOffset)
        
        and    s_pktCxt2.hdrBitmask2_nextHdr,   s_pktCxt2.hdrBitmask2_nextHdr, NOT_PKT_NEXTHDR_MASK
        or     s_pktCxt2.hdrBitmask2_nextHdr,   s_pktCxt2.hdrBitmask2_nextHdr, s_l1f.nextHdr
        sbco   s_pktCxt2.hdrBitmask2_nextHdr,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt2.hdrBitmask2_nextHdr), 1
    
l_stdHeldPktForward0_1:
        // Custom Routing handling 
        qbeq l_stdHeldPktForward0_4,  s_matchForward_pa.custType, PA_CUSTOM_TYPE_NONE
        
        // Patch the pktContext
        // eIndex should stote the custom Index
        and    s_pktCxt2.eP1C2Id,   s_pktCxt2.eP1C2Id, NOT_PKT2_EIDX_MASK
        lsl    r1.b1,   s_matchForward_pa.custIdx, PKT2_EIDX_SHIFT
        or     s_pktCxt2.eP1C2Id,   s_pktCxt2.eP1C2Id,  r1.b1
        sbco   s_pktCxt2.eP1C2Id,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt2.eP1C2Id), 1
        
        // Patch the next header Type
        and    s_pktCxt2.hdrBitmask2_nextHdr,   s_pktCxt2.hdrBitmask2_nextHdr, NOT_PKT_NEXTHDR_MASK
        qbeq   l_stdHeldPktForward0_2,  s_matchForward_pa.custType, PA_CUSTOM_TYPE_LUT1
        or     s_pktCxt2.hdrBitmask2_nextHdr,   s_pktCxt2.hdrBitmask2_nextHdr,  PA_HDR_CUSTOM_C2
        jmp    l_stdHeldPktForward0_3
l_stdHeldPktForward0_2:        
        or     s_pktCxt2.hdrBitmask2_nextHdr,   s_pktCxt2.hdrBitmask2_nextHdr,  PA_HDR_CUSTOM_C1
l_stdHeldPktForward0_3:        
        sbco   s_pktCxt2.hdrBitmask2_nextHdr,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt2.hdrBitmask2_nextHdr), 1

l_stdHeldPktForward0_4:
        //  Verify various control flags
        qbbc l_stdHeldPktForward0_5, s_matchForward_pa.ctrlBitMap.t_pa_cascaded_forwarding
            // update the pktCtx.flag2
            lbco s_pktCxt.flag2,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flag2), SIZE(s_pktCxt.flag2)
            set  s_pktCxt.flag2.t_flag2_cascaded_forwarding    
            sbco s_pktCxt.flag2,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flag2), SIZE(s_pktCxt.flag2)
            // pass through

l_stdHeldPktForward0_5:
        //   Load flags and operation in one instruction
        ldi  r4, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.threadId,     s_matchForward_pa.dest
        mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
        xout XID_CDECTRL,              s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
        
        //Is there user-stats update command 
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd,    PA_RX_CMD_USR_STATS
        ret

l_stdHeldPktForward1:
    // Forwarding packets to the host
    qbne  l_stdHeldPktForward3,  s_matchForward.forwardType,  PA_FORWARD_TYPE_HOST
        // store inner IP offset
        //lbco   r1.b0,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt6.l3l5offset), 1
        sbco   s_pktCxt6.l3l5offset,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt6.l3offset2), 1
    
        //
        // Perform IP Fragemnt packet check if configured and it is not cascaded forwarding packet
        //
        //lbco s_pktCxt.flag2,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flag2), SIZE(s_pktCxt.flag2)
        qbbs l_stdHeldPktForward1_0, s_pktCxt.flag2.t_flag2_cascaded_forwarding    
        qbbc l_stdHeldPktForward1_0, s_runCxt.flag2.t_ipFragToEroute
            qbbs l_stdHeldPktForward11,  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
            // pass through
    
l_stdHeldPktForward1_0:
    
        // Patch swinfo0
        sbco s_matchForward_host.context,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_stdHeldPktForward1_1, s_matchForward_host.psflags,   0
        lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0,  s_matchForward_host.psflags
        sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
l_stdHeldPktForward1_1:        
        // The pkt context may be examized and patched
        // For multi routing: Set flag and the multi route index, update Command ID
        // For command set command: Set flag and command set index, update Command ID
        // For CRC verification: Check the flag and update Command ID 
        // Both words are read in, changed, and sent out at once
        // For interface-based route, the emac port number and VLAN and P-bit priority is required
        //lbco   s_pktCxt,                cCdeHeldPkt,              SIZE(s_pktDescr),    8
        //lbco   s_pktCxt,                cCdeHeldPkt,              SIZE(s_pktDescr),    SIZE(s_pktCxt)
        
        // Reload the priority fields
        //lbco s_pktCxt.dscpPriority,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.dscpPriority), 2*SIZE(s_pktCxt.dscpPriority)

        // Check whether interface based routing is enabled or not 
        qbbc l_stdHeldPktForward1_no_if_changes, s_matchForward_host.ctrlBitmap.t_pa_routing_if_selection
            and  r2.b1, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK
            // There is no need to update destion queue and flow if interfcae is unknown
            qbeq  l_stdHeldPktForward1_no_priority, r2.b1, 0
                sub r2.b1, r2.b1, 1
                qbbc l_stdHeldPktForward1_no_eqos_changes, s_matchForward_host.ctrlBitmap.t_pa_routing_if_eqos
                 // load per port configuration from the scratch memory
                 // get the port number to be captured and obtain the offset  
                 // point to per port configuration structure   
                 // Extract output port number 
                 // r0.w2: Ingress port base to extract default priority only
                 // r1.b0: port ctrlBitMap (queue offset)
                 // r1.b1: priority (flow offset)
                 // r1.w2: Egress port base (control and Offset table)
                 and   r2.b3, s_matchForward_host.psflags, PA_PKT_PS_EMAC_PORTS_MASK
                 lsr   r2.b3, r2.b3, PA_PKT_PS_FLAGS_SHIFT
                 sub   r2.b3, r2.b3, 1  
                 mov   r1.w0, OFFSET_EQOS_CFG_BASE
                 lsl   r1.w2, r2.b3, 8
                 add   r1.w2, r1.w2, r1.w0
                 lsl   r0.w2, r2.b1, 8
                 add   r0.w2, r0.w2, r1.w0
                    
                 // load the port control and other information
                 lbco  r1.b0, PAMEM_CONST_PORTCFG, r1.w2, 1
                 // point to vlan priority table offset
                 add  r1.w2, r1.w2, 8
                 
                 // Ingress port  Byte4: default priority port
                 add   r0.w2, r0.w2, 4 
                 lbco  r1.b1, PAMEM_CONST_PORTCFG, r0.w2, 1

                 // check mode, whether DSCP mode or DP-BIT mode
                 qbbc  l_stdHeldPktForward1DscpEqosMode, r1.b0.t_eqos_dp_bit_mode
                   // dp bit mode
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_VLAN_MASK << PROT_COUNT_VLAN_SHIFT
                   qbeq l_stdHeldPktForward1NotVlanTag, r0.b0, 0
                     mov  r1.b1, s_pktCxt.vlanPriority
                     jmp  l_stdHeldPktForward1_comp
l_stdHeldPktForward1NotVlanTag:
                   qbbs l_stdHeldPktForward1_comp, r1.b0.t_eqos_pri_override
l_stdHeldPktForward1DscpEqosMode: 
#ifndef PASS_PROC_L2
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_IP_MASK << PROT_COUNT_IP_SHIFT
                   qbeq l_stdHeldPktForward1_comp, r0.b0, 0
#else
                   and  r0.b0, s_pktCxt2.hdrBitmask2_nextHdr, PKT_NEXTHDR_MASK 
                   qbeq l_stdHeldPktForward1ProcDscp, r0.b0, PA_HDR_IPv4
                   qbne l_stdHeldPktForward1_comp, r0.b0, PA_HDR_IPv6
l_stdHeldPktForward1ProcDscp:                   
#endif                   
                     // point to dscp table
                     add  r1.w2, r1.w2, 16
                     mov  r1.b1, s_pktCxt.dscpPriority
                     // pass through
l_stdHeldPktForward1_comp:
                   //point to correct priority table offset
                   lsl  r1.b1, r1.b1, 1
                   add  r1.w2, r1.w2, r1.b1
                   lbco r1.w0, PAMEM_CONST_PORTCFG, r1.w2, 2
                   // add to base queue and flow
                   add  s_matchForward.flowId, s_matchForward.flowId, r1.b1
                   add  s_matchForward.queue,  s_matchForward.queue,  r1.b0
                   mov  r2.b2,  PA_DEST_CDMA                   
                   jmp  l_stdHeldPktForward1_no_priority                   

l_stdHeldPktForward1_no_eqos_changes:                
                add  s_matchForward.queue, s_matchForward.queue, r2.b1

                qbbc l_stdHeldPktForward1_no_priority, s_matchForward_host.ctrlBitmap.t_pa_routing_if_dest_flow
                    // Now update the destination flow based on the ethernet interface
                    add  s_matchForward.flowId, s_matchForward.flowId, r2.b1
                    jmp  l_stdHeldPktForward1_no_priority

l_stdHeldPktForward1_no_if_changes:  
        // Verify whether priority-based routing is enabled       
        qbbc l_stdHeldPktForward1_dscp_priority, s_matchForward_host.ctrlBitMap.t_pa_routing_priority_vlan
        
l_stdHeldPktForward1_vlan_priority: 
            add  s_matchForward.queue, s_matchForward.queue, s_pktCxt.vlanPriority
            mov  r2.b2,  PA_DEST_CDMA 
               
l_stdHeldPktForward1_dscp_priority:
        // No priority routing enabled 
        qbbc l_stdHeldPktForward1_no_priority, s_matchForward_host.ctrlBitMap.t_pa_routing_priority_dscp
            add  s_matchForward.queue, s_matchForward.queue, s_pktCxt.dscpPriority
            mov  r2.b2,  PA_DEST_CDMA
            
l_stdHeldPktForward1_no_priority:

l_stdHeldPktForward1_queue_bounce:
        // Check for Queue Bounce operation
l_stdHeldPktForward1_queue_bounce_ddr:
        qbbc l_stdHeldPktForward1_queue_bounce_msmc, s_matchForward.queue.t_pa_forward_queue_bounce_ddr
            clr s_matchForward.queue.t_pa_forward_queue_bounce_ddr
            sbco s_matchForward.queue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_matchForward.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_stdHeldPktForward1_queue_bounce_end

l_stdHeldPktForward1_queue_bounce_msmc:
        qbbc l_stdHeldPktForward1_queue_bounce_end, s_matchForward.queue.t_pa_forward_queue_bounce_msmc
            clr s_matchForward.queue.t_pa_forward_queue_bounce_msmc
            sbco s_matchForward.queue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_matchForward.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_stdHeldPktForward1_queue_bounce_end:
        // Note CRC verification is only required by the SCTP header
        // The destination must be host and there should be neither multi-route nor command set
        mov    r2.b0,   (SIZE(s_pktCxt) + 7) & 0xf8 
        
        // check whether the CRC verification is enabled, which precedes all other special case
        qbbs   l_stdHeldPktForward10, s_pktCxt.flag.t_flag_crc_verify

        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        qbbs l_stdHeldPktForward2,  s_matchForward_host.ctrlBitMap.t_pa_multiroute
l_stdHeldPktForward_normal_host_route:
            // Normal host route: no special operation is required
            // Send the packet on its way
            ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
            mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
            xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
            qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret

l_stdHeldPktForward2:     
            // For multi routing the command ID and the multi route index
            // must be patched into the packet. These are two bytes which span 2 32 bits words.
            // Both words are read in, changed, and sent out at once

            // lbco s_pktCxt,                 cCdeHeldPkt,              SIZE(s_pktDescr),    8
            // Note: It can be skipped since the original commad is 0 
            //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
            or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
            set  s_pktCxt.flag.t_flag_multi_route
    
            and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,              NOT_PKT2_EIDX_MASK
            lsl  r3.b0,              s_matchForward_host.multiIdx,   PKT2_EIDX_SHIFT
            or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,              r3.b0
            sbco s_pktCxt,           cCdeHeldPkt,                    SIZE(s_pktDescr),    8

            // Send the packet on its way
            ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    s_matchForward_host.paPdspRouter
            xout XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
            qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret


l_stdHeldPktForward3:
    qbne  l_stdHeldPktForward4, s_matchForward.forwardType, PA_FORWARD_TYPE_SA
        //
        // IP Fragemnt packet can not be forwarded to SA
        //
        qbbs l_stdHeldPktForward11,  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
    
        // Routing to SA
        // Patch swinfo0 and swinfo2
        sbco s_matchForward_sa.swInfo0,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  8
        
        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        
        // Send the packet on its way
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8       // Round up to multiple of 8 bytes
        mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
        mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
        xout  XID_CDECTRL,            s_cdeCmdPkt,                           SIZE( s_cdeCmdPkt)
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret
        
l_stdHeldPktForward4:
    qbne  l_stdHeldPktForward5, s_matchForward.forwardType, PA_FORWARD_TYPE_SRIO
        // Routing to SRIO
        // Overwrite Packet Info with psinfo0 and psinfo2
        sbco s_matchForward_srio.psInfo0,  cCdeHeldPkt, SIZE(s_pktDescr),  8
        
        // Patch the packet type
        lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_TYPE_MASK
        lsl  r2.b1, s_matchForward_srio.pktType, PA_PKT_TYPE_SHIFT
        or   r2.b0, r2.b0,  r2.b1
        sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        
        // Send the packet on its way
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  8                       // SRIO take 8 bytes Ps Info
        mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
        mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
        xout  XID_CDECTRL,            s_cdeCmdPkt,                           SIZE( s_cdeCmdPkt)
        ret
        

l_stdHeldPktForward5:
    qbne  l_stdHeldPktForward6, s_matchForward.forwardType, PA_FORWARD_TYPE_ETH
        // Routing to ETH
        // Patch the psflags
        lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0,  s_matchForward_eth.psflags
        sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1

        // Send the packet on its way
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  0                       // remove pktCtx
        mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
        xout  XID_CDECTRL,            s_cdeCmdPkt,            SIZE( s_cdeCmdPkt)
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_stdHeldPktForward6:
    qbeq  l_stdHeldPktForward8,  s_matchForward.forwardType, PA_FORWARD_TYPE_DISCARD

fci_stdHeldPktForward7:   // A jump in point for other functions to use
                          // the system error info

        // Invalid match type in table - this should never happen
        // Inc the stat and generate an event if enabled
        set s_statsFlags.event.t_nSystemFail

l_stdHeldPktForward8: 
        // Do a silent discard, release the packet ID
        set   s_statsFlags.event.t_nSilentDiscardC1
        mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
        xout  XID_CDECTRL,           s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
        //Is there user-stats update command 
        qbeq l_stdHeldPktForward12,  s_matchRxCmdHdr.cmd,    PA_RX_CMD_USR_STATS
        ret
        
l_stdHeldPktForward9:     
        // For command set command, the command ID and the command set index
        // must be patched into the packet. These are two bytes which span 2 32 bits words.
        // Both words are read in, changed, and sent out at once
        set  s_pktCxt.flag.t_flag_cmdset
    
        //and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,              NOT_PKT2_EIDX_MASK
        //lsl  r3.b0,              s_matchRxCmdSet.index,          PKT2_EIDX_SHIFT
        //or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,              r3.b0
        sbco   s_matchRxCmdSet.index,   cCdeHeldPkt,               SIZE(s_pktDescr) + OFFSET(s_pktCxt5.cmdSetIdx),    SIZE(s_pktCxt5.cmdSetIdx) 
        
        mov  r2.b0,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        
l_stdHeldPktForward10:    
        // Share operation by both command set and CRC verification
        // Note: It can be skipped since the original commad is 0 
        // and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
        or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
        sbco s_pktCxt,           cCdeHeldPkt,                    SIZE(s_pktDescr),    8
        
        // patch the flow Id and queue Id
        sbco s_matchForward.flowId, cCdeHeldPkt,                 OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
        sbco s_matchForward.queue,  cCdeHeldPkt,                 OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)
        
        // Send the packet on its way
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.threadId,    r2.b2
        mov  s_cdeCmdPkt.psInfoSize,  32         
        xout XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        ret
        
l_stdHeldPktForward11:
        // IP fragmented packet
        // Follow the exception route
        // Note: It is up to the user to set the exception route correctly
        // Note: clear local copy to prevent infinite re-entry
        clr   s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
        and   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  NOT_PKT2_EIDX_MASK
        or    s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  EROUTE_IP_FRAG << PKT2_EIDX_SHIFT
        sbco  s_pktCxt2.eP1C2Id,  cCdeHeldPkt,        SIZE(s_pktDescr)+OFFSET(s_pktCxt2.eP1C2Id),  SIZE(s_pktCxt2.eP1C2Id)
        
        mov   r1.w0, EROUTE_IP_FRAG*SIZE(s_fmPlace)
        lbco  s_fmPlace,         PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_fmPlace)
        jmp   fci_stdHeldPktForward_retry
        
l_stdHeldPktForward12:
        // User Statistics operation:
        // Record and insert the statistics update into the FIFO
        // Note all user commands are in the same location (r12)
        mov   s_usrStatsReq.index,  s_matchForward_host.cmd.w0
        
        lbco  s_fifoCb, PAMEM_CONST_PDSP_CXT,  OFFSET_PDSP0_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
        add   r1.b0,    s_fifoCb.in, 4
        and   r1.b0,    r1.b0,       0x1F
        
        qbne  l_stdHeldPktForward12_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            set s_statsFlags.event.t_nSystemFail
            ret

l_stdHeldPktForward12_1:
        // Insert the request into the FIFO   
        add   r1.w2,   s_fifoCb.in, OFFSET_PDSP0_USR_STATS_FIFO
        sbco  s_usrStatsReq,  PAMEM_CONST_PDSP_CXT, r1.w2, SIZE(s_usrStatsReq)
        sbco  r1.b0,   PAMEM_CONST_PDSP_CXT,    OFFSET_PDSP0_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
        ret

    .leave  pktScope
    .leave  cdeScope
    .leave  lut1MatchScope
    .leave  usrStatsFifoScope

// ******************************************************************************************************
// * FUNCTION PURPOSE: Standard routing of the current packet
// ******************************************************************************************************
// * DESCRIPTION: The current packet is routed
// *
// *              The only time a current packet is routed is if there is an error.
// *              the exception index is supplied in r30.w2, and the packet is forwarded based on
// *              that index.
// *
// *              s_pktCxt.scratch contains the packet ID
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
// *   R26:     |     w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Exception Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *            
// ******************************************************************************************************

    .using currentFwdScope
    .using pktScope

f_errCurrentPktForward:
    // store inner IP offsets
    //lbco   r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt6.l3l5offset), 1
    sbco   s_pktCxt6.l3l5offset,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt6.l3offset2), 1

    // Load the error routing information
    lsl   r30.w2,         r30.w2,              4
    lbco  s_curFmPlace,   PAMEM_CONST_EROUTE,  r30.w2,   SIZE(s_curFmPlace)

    // f_curPktForward needs which discard stat to set (classify1 or classify2)
    zero &r3,  4
    set   r3.t_nSilentDiscardC1

    // Need a return address
    mov r30.w0, fci_mainLoop6
    jmp f_curPktForward

    .leave currentFwdScope
    .leave pktScope
    

// ******************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that failed a LUT1 search
// ******************************************************************************************************
// * DESCRIPTION: The packet is forwarded based on a previous match, or else 
// *              through the no match route
// *
// *              This function exits through a call to stdHeldPktForward, so
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
// *   R6:                                                                     -
// *   R7:                                                                     -
// *   R8:                                                                     -
// *   R9:      |                                  -                           -  pktScope used
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
// *   R26:     |    w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ******************************************************************************************************/
    .using  cdeScope
    .using  pktScope
    .using  lut1MatchScope

f_c1ForwardHeldPacketNoMatch:

    // Inc the no match stat
    set s_statsFlags.event.t_nNoTableMatch
    wbc   s_flags.info.tStatus_CDEBusy
    
    // Wait for the held packet to appear in the CDE
    // (Its almost certainly already there)
    wbs   s_flags.info.tStatus_CDEHeldPacket
    
    // Load the held pktInfo
    // The pkt context may be examized and patched
    lbco   s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    SIZE(s_pktCxt)
    
#ifdef PASS_PROC_DEF_ROUTE   

    qbbc   l_c1ForwardHeldPacketNoMatch_no_def_r, s_runCxt.flag2.t_def_route    

        // Reload the packet context
        lbco   s_pktCxt.paCmdId_Length,   cCdeHeldPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

        // default route is enabled globally
        // Update r1 with offset for interface (get to zero base)
        and   r1.w0, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK 
        qbeq l_c1ForwardHeldPacketNoMatch_no_def_r, r1.w0, 0
        mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE      
      
        sub   r1.w0, r1.w0, 1
        lsl   r1.w0, r1.w0, 6
        add   r1.w0, r1.w0, r1.w2
      
        // load the default control bit map for post-classification no match packets
        lbco  r2.b0, PAMEM_CONST_PORTCFG, r1.w0, 1   
        
l_c1ForwardHeldPacketNoMatch_DFR_MC:        
        //Is it a multicast packet
        qbbc   l_c1ForwardHeldPacketNoMatch_DFR_BC, s_pktCxt.flag.t_flag_mac_multicast 
            //multicast packet
            qbbc    l_c1FwdHeldPktERoute_MC, r2.b0.t_default_route_mc_enable
                add r1.w0, r1.w0, 4
                jmp l_c1ForwardHeldPacketNoMatch_DFR_end
        
l_c1ForwardHeldPacketNoMatch_DFR_BC:
        //Is it a broadcast packet
        qbbc   l_c1ForwardHeldPacketNoMatch_DFR_UC, s_pktCxt.flag.t_flag_mac_broadcast 
            //broadcast packet
            qbbc    l_c1FwdHeldPktERoute_BC, r2.b0.t_default_route_bc_enable
                add r1.w0, r1.w0, 4 + SIZE(s_fmPlace)
                jmp l_c1ForwardHeldPacketNoMatch_DFR_end
        
l_c1ForwardHeldPacketNoMatch_DFR_UC:
            //Unicast packet
            qbbc    l_c1ForwardHeldPacketNoMatch_BM_6, r2.b0.t_default_route_uc_enable
                add r1.w0, r1.w0, 4 + SIZE(s_fmPlace)*2
        
             // pass through
l_c1ForwardHeldPacketNoMatch_DFR_end:
        lbco   s_fmPlace,         PAMEM_CONST_PORTCFG,  r1.w0,  SIZE(s_fmPlace)        
        jmp    f_stdHeldPktForward 
      
l_c1ForwardHeldPacketNoMatch_no_def_r:

#endif   

    // Read from the packet the previous match information
    //lbco   s_pktCxt.paCmdId_Length,   cCdeHeldPkt,  SIZE(s_pktDescr),  OFFSET(s_pktCxt.l3Offset)
    
    // Broadcast and multicast check if no match found 
    // Assume that this rule precedes the pre-match nextFail rule
l_c1ForwardHeldPacketNoMatch_BM_1:
    and    r1.b0,   s_pktCxt.flag,  PA_BM_FLAG_MASK
    qbeq   l_c1ForwardHeldPacketNoMatch_BM_6, r1.b0, 0    
    
    qbbc   l_c1ForwardHeldPacketNoMatch_BM_2, s_pktCxt.flag.t_flag_ip_multicast 
        mov    r1.w0, EROUTE_IP_MULTICAST*SIZE(s_fmPlace)
        jmp    l_c1ForwardHeldPacketNoMatch_BM_5  
        
l_c1ForwardHeldPacketNoMatch_BM_2:    
    qbbc   l_c1ForwardHeldPacketNoMatch_BM_3, s_pktCxt.flag.t_flag_ip_broadcast 
        mov    r1.w0, EROUTE_IP_BROADCAST*SIZE(s_fmPlace)
        jmp    l_c1ForwardHeldPacketNoMatch_BM_5  
        
l_c1ForwardHeldPacketNoMatch_BM_3:    
    qbbc   l_c1ForwardHeldPacketNoMatch_BM_4, s_pktCxt.flag.t_flag_mac_broadcast 
l_c1FwdHeldPktERoute_BC:    
        mov    r1.w0, EROUTE_MAC_BROADCAST*SIZE(s_fmPlace)
        jmp    l_c1ForwardHeldPacketNoMatch_BM_5  
    
l_c1ForwardHeldPacketNoMatch_BM_4:    
l_c1FwdHeldPktERoute_MC:        
        mov    r1.w0, EROUTE_MAC_MULTICAST*SIZE(s_fmPlace)
        // jmp    l_c1ForwardHeldPacketNoMatch_BM_5          
        // pass through
    
l_c1ForwardHeldPacketNoMatch_BM_5:
        // Load the routing information for broadcast/multicast type into the 
        // matchforward context
        //mov   s_pktCxt.flag,     0
        //sbco  s_pktCxt.flag,     cCdeHeldPkt,         SIZE(s_pktDescr)+OFFSET(s_pktCxt.flag),  SIZE(s_pktCxt.flag)
        
        lbco  s_fmPlace,         PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_fmPlace)
        qbeq  l_c1ForwardHeldPacketNoMatch_BM_6, s_matchForward.forwardType, PA_FORWARD_TYPE_DISCARD
        jmp   f_stdHeldPktForward
    
l_c1ForwardHeldPacketNoMatch_BM_6:

    qbbs  l_c1ForwardHeldPacketNoMatch0, s_pktCxt.eP1C2IdIdx.t_pl1Match

        // For no previous match set the error type in the packet, load the 
        // error routing information and route
        // Here the overlayed pkt2 structure is used even though the load was to
        // pktContext
        and  s_pktCxt2.eP1C2Id,       s_pktCxt2.eP1C2Id,   PKT2_EIDX_MASK
        or   s_pktCxt2.eP1C2Id,       s_pktCxt2.eP1C2Id,   EROUTE_LUT1_FAIL << PKT2_EIDX_SHIFT
        sbco s_pktCxt.eP1C2IdIdx,     cCdeHeldPkt,         SIZE(s_pktDescr)+OFFSET(s_pktCxt.eP1C2IdIdx),  2


        // Load the routing information for LUT1 fail error type into the 
        // matchforward context
        lbco  s_fmPlace,            PAMEM_CONST_EROUTE,  EROUTE_LUT1_FAIL << 4,  SIZE(s_fmPlace)
        jmp   f_stdHeldPktForward

l_c1ForwardHeldPacketNoMatch0:

        // In the case of a previous match, the previous match routing information
        // must be loaded into the matchForward context
        and  r1.w0,  s_pktCxt.eP1C2IdIdx,  L1IDX_MASK
        lsl  r1.w0,  r1.w0,                6                              // get the associated table offset
        set  r1.w0,  12                                                   // add offset 0x1000
        add  r1.w0,  r1.w0,                SIZE(s_fmPlace)+SIZE(s_l1f)    // Load the next fail information

        lsr  r1.w2,  s_pktCxt.eP1C2IdIdx,  SUBS_PKT_CXT_L1ID_BIT_OFFSET
        and  r1.w2,  r1.w2,                SUBS_PKT_CXT_L1ID_SHIFTED_MASK  // The PDSP ID

        qbne  l_c1ForwardHeldPacketNoMatch1,  r1.w2,   0
            lbco  s_fmPlace,           PAMEM_CONST_PDSP0_LUT1_BASE,  r1.w0,  SIZE(s_fmPlace)
            jmp   f_stdHeldPktForward

l_c1ForwardHeldPacketNoMatch1:
        qbne  l_c1ForwardHeldPacketNoMatch2,  r1.w2,        1
            lbco  s_fmPlace,           PAMEM_CONST_PDSP1_LUT1_BASE,  r1.w0,  SIZE(s_fmPlace)
            jmp   f_stdHeldPktForward

l_c1ForwardHeldPacketNoMatch2:
        qbne  l_c1ForwardHeldPacketNoMatch3,  r1.w2,   2
            lbco  s_fmPlace,           PAMEM_CONST_PDSP2_LUT1_BASE,  r1.w0,  SIZE(s_fmPlace)
            jmp   f_stdHeldPktForward


l_c1ForwardHeldPacketNoMatch3:
        // There was an invalid previous PDSP ID in the packet
        // jump into the error handling section of the held packet forward function
        jmp fci_stdHeldPktForward7

    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    
// ******************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that failed during L2/3 parsing
// ******************************************************************************************************
// * DESCRIPTION: The packet is forwarded based on exception route 
// *
// *              This function exits through a call to stdHeldPktForward, so
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
// *   R6:                                                                     -
// *   R7:                                                                     -
// *   R8:                                                                     -
// *   R9:      |                                  -                           -  pktScope used
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
// *   R26:     |    w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ******************************************************************************************************/
    .using cdeScope
    .using pktScope
    .using lut1MatchScope
    .using currentFwdScope
    

f_c1ForwardHeldPacketErrPkt:
    // Wait for the held packet to appear in the CDE
    // (Its almost certainly already there)
    wbs   s_flags.info.tStatus_CDEHeldPacket
    wbc   s_flags.info.tStatus_CDEBusy
    
    // Load the held pktInfo
    // The pkt context may be examized and patched
    lbco   s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    SIZE(s_pktCxt)
    
    // Read from the packet the previous match information
    lbco   s_pktCxt.paCmdId_Length,   cCdeHeldPkt,  SIZE(s_pktDescr),  OFFSET(s_pktCxt.l3Offset)
    
    lsr   r30.w2,         s_pktCxt2.eP1C2Id,  PKT2_EIDX_SHIFT
    lsl   r30.w2,         r30.w2,              4
    lbco  s_fmPlace,      PAMEM_CONST_EROUTE,  r30.w2,   SIZE(s_fmPlace)
    
    jmp   f_stdHeldPktForward
    
    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    .leave currentFwdScope
    
    
// ******************************************************************************************************
// * FUNCTION PURPOSE: Forward a held BC/MC packet per pre-classification default route
// ******************************************************************************************************
// * DESCRIPTION: The packet is forwarded based on pre-classification default route
// *
// *              This function exits through a call to stdHeldPktForward, so
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
// *   R6:                                                                     -
// *   R7:                                                                     -
// *   R8:                                                                     -
// *   R9:      |                                  -                           -  pktScope used
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
// *   R26:     |    w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ******************************************************************************************************/
#ifdef PASS_PROC_DEF_ROUTE   

    .using cdeScope
    .using pktScope
    .using lut1MatchScope
    .using currentFwdScope
    

f_c1ForwardHeldPacketBcMcPkt:

    // Wait for the held packet to appear in the CDE
    // (Its almost certainly already there)
    wbs   s_flags.info.tStatus_CDEHeldPacket
    wbc   s_flags.info.tStatus_CDEBusy
    
    // Load the held pktInfo
    // The pkt context may be examized and patched
    lbco   s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    SIZE(s_pktCxt)

    // We will not enter this function unless default route and pre-classification is enabled
    // Update r1 with offset for interface (get to zero base)
    and   r1.w0, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK  
    mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE     
    
    sub   r1.w0, r1.w0, 1
    lsl   r1.w0, r1.w0, 6
    add   r1.w0, r1.w0, r1.w2
    
l_c1ForwardHeldPacketBcMcPkt_MC:        
    //Is it a multicast packet
    qbbc   l_c1ForwardHeldPacketBcMcPkt_BC, s_pktCxt.flag.t_flag_mac_multicast 
        add r1.w0, r1.w0, 4
        jmp l_c1ForwardHeldPacketBcMcPkt_end
        
l_c1ForwardHeldPacketBcMcPkt_BC:
    // If it is not multicast, then it must be broadcast
        add r1.w0, r1.w0, 4 + SIZE(s_fmPlace)
        // jmp l_c1ForwardHeldPacketBcMcPkt_end
        // pass throiugh
        
l_c1ForwardHeldPacketBcMcPkt_end:
     lbco   s_fmPlace,         PAMEM_CONST_PORTCFG,  r1.w0,  SIZE(s_fmPlace)        
     jmp    f_stdHeldPktForward 
    
    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    .leave currentFwdScope
    
#endif

// ******************************************************************************************************
// * FUNCTION PURPOSE: Standard routing of the pre-classified BC/MC packet
// ******************************************************************************************************
// * DESCRIPTION: The current BC/MC packet is routed baed on default route
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
// *   R26:     |     w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Exception Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *            
// ******************************************************************************************************

#ifdef PASS_PROC_DEF_ROUTE   

    .using currentFwdScope
    .using pktScope

f_BcMcCurrentPktForward:

    // We will not enter this function unless default route and pre-classification is enabled
    // Update r1 with offset for interface (get to zero base)
    and   r1.w0, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK      
    mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE 
    
    sub   r1.w0, r1.w0, 1
    lsl   r1.w0, r1.w0, 6
    add   r1.w0, r1.w0, r1.w2
    
l_BcMcCurrentPktForward_MC:        
    //Is it a multicast packet
    qbbc   l_BcMcCurrentPktForward_BC, s_pktCxt.flag.t_flag_mac_multicast 
        add r1.w0, r1.w0, 4
        jmp l_BcMcCurrentPktForward_end
        
l_BcMcCurrentPktForward_BC:
    // If it is not multicast, then it must be broadcast
        add r1.w0, r1.w0, 4 + SIZE(struct_paFwdPlace)
        // jmp l_BcMcCurrentPktForward_end
        // pass throiugh
        
l_BcMcCurrentPktForward_end:
    // Load the packet routing information
    lbco   s_curFmPlace,  PAMEM_CONST_PORTCFG,  r1.w0,  SIZE(struct_paFwdPlace)        

    // Need a return address
    mov r30.w0, fci_mainLoop6
    jmp f_curPktForward

    .leave currentFwdScope
    .leave pktScope

#endif    


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
// *   R26:     |   w0(scratch) - Packet ID
// *   R27:     |
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2 - param.action w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  checkScope
    .using  pktCaptureScope

f_c1Parse:

    set s_statsFlags.event.t_nPackets

    lbco  r1, FIRMWARE_MBOX, 0, 4
    add   r1, r1, 1
    sbco  r1, FIRMWARE_MBOX, 0, 4

    // Read the descriptor
    xin  XID_CDEDATA,   s_pktDescr,   SIZE(s_pktDescr)

 // qbbs  l_c1Parse3,  s_pktDescr.pktId.t_pktIdAllocated

l_c1Parse2:                        
    // Have a new packet ID. Set the alloc bit
    // s_pktDescr.pktId = r1.w0 | ( 1<<14)
    // set   s_pktDescr.pktId,    r1.w0.t_pktIdAllocated
    // xout  XID_CDEDATA,         s_pktDescr.pktId,                    SIZE(s_pktDescr.pktId)

#ifdef PASS_TIMESTAMP_OP
    //and  r1.b0,  s_runCxt.flags, SUBS_RUN_CXT_ID_MASK // get PDSP ID
    // Timestamp will be only be added to packets in PDSP 0
    //qbne l_c1Parse3, r1.b0,   0
    // Add timestamp to all the packets with new packet ID including the command packet
    ldi   r1,    PDSP0_TIMER
    lbbo  r0,    r1, 8, 4
    mov   r0.w2, 0xffff
    sub   s_pktDescr.timestamp.w0,  r0.w2,  r0.w0 
    lbco    r0, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP,   8
    qblt    l_c1Parse2_timestamp_loaded, s_pktDescr.timestamp.w0, 100
    qbbc    l_c1Parse2_timestamp_loaded, s_flags.info.tStatus_Timer
    set     s_flags.info.tStatus_Timer // set to clear timer event 
    add     r0, r0, 1
    adc     r1, r1, 0
    sbco    r0, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP,   8

l_c1Parse2_timestamp_loaded:
    mov   s_pktDescr.timestamp.w2,  r0.w0
    // upper 16 bits
    //mov   s_pktCxt.scratch,         r0.w2
    xout  XID_CDEDATA,         s_pktDescr.timestamp,                SIZE(s_pktDescr.timestamp)
    // upper 32 bits: store at r5 to be inserted later
    // upper upper 16 bits
    mov     s_cdeCmdIn2.data.w0,   r0.w2
    mov     s_cdeCmdIn2.data.w2,   r1.w0

#endif


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

#ifdef PASS_PROC_L2
    qbeq  l_c1Parse6,   s_pktDescr.ctrlDataSize,    0   // Packet is from ethernet
    qbne  l_c1Parse3a,  s_pktDescr.ctrlDataSize,    8 
    // Extract pkt type
    lsr   r1.b0,        s_pktDescr.pktType_pvtFlags,   PA_PKT_TYPE_SHIFT 
    qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_9    
    qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_11    

#else
    qbeq  l_c1Parse4,   s_pktDescr.ctrlDataSize,    0   // Packet is from ethernet
#endif


l_c1Parse3a:
    // l_c1Parse3a - l_c1Parse5: command and re-entry packet
    // Read in the whole packet context. This is the largest command size
    // There is no cost if this is not a data packet. 
    xin  XID_CDEDATA,        s_pktCxt,          SIZE(s_pktCxt)

    // extract the command ID 
    lsr  r1.b0,  s_pktCxt.paCmdId_Length, SUBS_CMD_WORD_ID_SHIFT

    qbeq  l_c1Parse9,         r1.b0,   PSH_CMD_PA_RX_PARSE
    qbeq  f_paConfigure,      r1.b0,   PSH_CMD_CONFIG

l_c1Parse4:
        // The packet has an invalid command
        // Inc the stat
        set s_statsFlags.event.t_nInvalidControlC1

l_c1Parse5:
            // Discard the packet
            zero &s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
            mov   s_cdeCmdPkt.operation,   CDE_CMD_PACKET_FLUSH
            xout  XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)

            jmp   f_mainLoop

#ifdef PASS_PROC_L2

l_c1Parse6:
    // Packet is from ethernet and has not been seen by the PA before
    // Check if global packet capture feature is enabled
    // When disabled go through regular code flow
    qbbc  l_c1Parse6a, s_runCxt.flag2.t_ingress_pCapEnable
    // Global packet capture feature is enabled, check whether
    // per interface capture is enabled?    
    // load the ingress port number to get the offset    

    // Update r1 with offset for interface
    lsl  r1.w0, s_pktDescr.srcId, 3
    // There is no need to update destination queue and flow when interface is unknown
    qbeq  l_c1Parse6a, r1.w0, 0 

    add  r1.w0, r1.w0, OFFSET_INGRESS_PKT_CAP_CFG_BASE
    // load the configurations control bit map
    lbco  s_paPktCapScr, PAMEM_CONST_PORTCFG, r1.w0, SIZE(s_paPktCapScr)
    // Check the enable bit for ethernet capture
    qbbc  l_c1Parse6a, s_paPktCapScr.ctrlBitMap.t_pkt_cap_enable
    
    //store the upper 32-bit of timestamp
    sbco    &s_cdeCmdIn2.data, PAMEM_CONST_PARSE,  OFFSET_TIMESTAMP_TMP,   4

    // Jump to copy initiate, when there is no active look up
    qbbc  l_c1Parse6_copy, s_runCxt.flags.t_activeLookup
    // There is an active lookup, check whether we have room to copy packet
    // The formula to compute the room is 
    // (Previous packet Size + current Pkt Size * 2 ) < 21K
    add    r1.w2,  s_pktCxt.endOffset, s_pktDescr.pktDataSize
    add    r1.w2,  r1.w2, s_pktDescr.pktDataSize
    mov    r1.w0, FIRMWARE_CONSTANT_21K
    // Jump to copy operation if there is a space in output buffer
    qbgt   l_c1Parse6_copy, r1.w2,  r1.w0
    // LUT1 is very busy holding up the space, so wait and free output space before copy
     wbc  s_flags.info.tStatus_Lut1Busy
     // A lookup is complete for the held packet
     xin   XID_LUT1CMD,              s_l1Status,     SIZE(s_l1Status)
     qbbc  l_c1Parse6_NoMatchRelease, s_l1Status.status.L1_STATUS_ENTRY_MATCH
     call  f_c1ForwardHeldPacketMatch
     jmp   l_c1Parse6_clr_activeLookup
l_c1Parse6_NoMatchRelease:   
     call  f_c1ForwardHeldPacketNoMatch
l_c1Parse6_clr_activeLookup:     
     clr  s_runCxt.flags.t_activeLookup
l_c1Parse6_copy:    
      wbs   s_flags.info.tStatus_CDEOutPacket  
    // Issue Copy command (either to Host or Destination)
    // Check is the copy to be done to host?
    // Made sure if the packet capture scratch is intact
    zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)   
    qbbs  l_c1Parse6_host, s_paPktCapScr.ctrlBitMap.t_pkt_cap_host    
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
      jmp  l_c1Parse6_send
    
l_c1Parse6_host:
l_c1Parse6_host_queue_bounce:
        // Check for Queue Bounce operation
l_c1Parse6_host_queue_bounce_ddr:
        qbbc l_c1Parse6_host_queue_bounce_msmc, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1Parse6_host_queue_bounce_end

l_c1Parse6_host_queue_bounce_msmc:
        qbbc l_c1Parse6_host_queue_bounce_end, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through
l_c1Parse6_host_queue_bounce_end:

      // Packet Capture activity
      mov  s_cdeCmdPkt.threadId,     PA_DEST_CDMA
      mov  s_cdeCmdPkt.destQueue,    s_paPktCapScr.destQueue
      mov  s_cdeCmdPkt.flowId,       s_paPktCapScr.mport_flow  
      sbco  s_paPktCapScr.context,  cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0), SIZE(s_paPktCapScr.context) 
      
l_c1Parse6_send:
      mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY      
      mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE)     
      mov  s_cdeCmdPkt.psInfoSize,  0
      xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)

     // Wait for new packet to be ready
     wbs   s_flags.info.tStatus_CDENewPacket
     
     // Read the packet descriptor
     xin  XID_CDEDATA,   s_pktDescr,   SIZE(s_pktDescr)
    
     // Move up to control and insert 32 bytes of PS data      
     // The context is read from the control section. Advance
     // to the control section
     mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_CONTROL
     xout  XID_CDECTRL,           s_cdeCmdWd,           4

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    //mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    //mov s_cdeInsert.byteCount,  32
    ldi  r4,        CDE_CMD_INSERT_PSDATA | (32 << 8)
    xout XID_CDECTRL,           s_cdeCmdWd,             4     
    
    //restore the upper 32-bit of timestamp
    lbco    &s_cdeCmdIn2.data, PAMEM_CONST_PARSE,  OFFSET_TIMESTAMP_TMP,   4

l_c1Parse6a:    
    // Note: nextHdr = 0 (MAC) startOffset = 0
    // Last 2 bytes already contain the upper 16 - bits of the timestamp
    
    //Insert the upper 32-bit timestamp as control Info
    //mov s_cdeInsert2.operation,  CDE_CMD_INSERT_CONTROL
    //mov s_cdeInsert2.byteCount,  8
    ldi   r4,   CDE_CMD_INSERT_CONTROL | (8 << 8)   
    xout  XID_CDECTRL,          s_cdeCmdIn2,            SIZE(s_cdeCmdIn2)
    
    //zero  &s_pktCxt,            SIZE(s_pktCxt) -2
    zero  &s_pktCxt,            SIZE(s_pktCxt)
    mov    s_pktCxt.endOffset,  s_pktDescr.pktDataSize

    jmp    l_c1Parse9a


l_c1Parse8:
        // Packet is from SRIO with a control data size of 8 bytes
        // which means this is the first time the PA is seeing it.
        zero  &s_pktCxt,          SIZE(s_pktCxt) - 2
        mov   s_pktCxt.endOffset, s_pktDescr.pktDataSize 

l_c1Parse8a:
        
        mov s_pktCxt2.hdrBitmask2_nextHdr,    PA_HDR_UNKNOWN  

        // Continue to parse the SRIO packet normally  (fall through)
        mov  r30.w0,         l_c1Parse12             // return to l_c1Parse9 after call to f_paSrio 
        jmp  f_c1ParseSrio   
#endif        
        

l_c1Parse9:

        // Delete the control information. It will be replace with the packet context during parse
        //mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
        //xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
        
        // Delete only the pktCxt from the control info
        mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
        mov  s_cdeCmdWd.byteCount,  (SIZE(s_pktCxt) + 7) & 0xf8     // Round up to multiple of 8 bytes
        xout XID_CDECTRL,           s_cdeCmdWd,                 4
         
        mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
        xout  XID_CDECTRL,           s_cdeCmdWd,                4

l_c1Parse9a:

    // advance the CDE to the first byte to examine in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_pktCxt.startOffset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)


    // next header is stored in 8 bit format during the parse
    and  s_next.Hdr,  s_pktCxt2.hdrBitmask2_nextHdr,  0x1f

    mov s_param.action,     SUBS_ACTION_PARSE
    mov s_runCxt.keyFlags,  0

    // Clear any values in the error index only if not custom
    qbeq l_c1Parse10, s_next.Hdr, PA_HDR_CUSTOM_C1 
    and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK

    // Make sure the parse entry point is valid for classify1
    // PA_HDR_UNKNOWN and others should use nextFail route
    // Note: This is not a parsing error exception
    qbgt  l_c1Parse10,  s_next.Hdr,  PA_HDR_UNKNOWN
        set s_runCxt.flag2.t_failLookup
        mov s_param.action,          SUBS_ACTION_EXIT

        jmp l_c1Parse14      


l_c1Parse10:
    // s_l1View1ab(16), s_l1View2b (32) and s_l1View3b (16) share the same area
    // Clear the LUT1 view areas
    // Note: packet descriptor will still reside at r6-r13
    zero &s_l1View2b,     SIZE(s_l1View2b)
    xout  XID_LUT1V1,     s_l1View1b,         SIZE(s_l1View1b)
    xout  XID_LUT1V2,     s_l1View2b,         SIZE(s_l1View2b)
    xout  XID_LUT1V3,     s_l1View3b,         SIZE(s_l1View3b)

l_c1Parse11:

    //  The main parse loop
    //
    //  General Error check to avoid infinite loop
    //
    qblt l_c1Parse11_1,  s_pktCxt.endOffset, s_pktCxt.startOffset
        mov s_pktCxt2.eP1C2Id,  EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT 
        mov s_param.action,          SUBS_ACTION_EXIT
        jmp l_c1Parse14
     
l_c1Parse11_1:
    //  Util SUBS_ACTION_LOOKUP or SUBS_ACTION_EXIT occur
    lsl   r0.b0,          s_next.Hdr,         1
    add   r0.b0,          r0.b0,              OFFSET_PARSE_TABLE
    lbco  r0.w2,          PAMEM_CONST_PARSE,  r0.b0,              2
    call  r0.w2
    qbeq  l_c1Parse11,    s_param.action,     SUBS_ACTION_PARSE


l_c1Parse12:

    // Put the next header type back into the context
    and  s_pktCxt2.hdrBitmask2_nextHdr, s_pktCxt2.hdrBitmask2_nexthdr,  0xe0
    or   s_pktCxt2.hdrBitmask2_nextHdr, s_pktCxt2.hdrBitmask2_nextHdr,  s_next.Hdr
    
#ifdef PASS_PROC_DEF_ROUTE 

    qbbc   l_c1Parse14, s_runCxt.flag2.t_def_route    

        // default route is enabled globally
        // Update r1 with offset for interface (get to zero base)
        and   r1.w0, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK      
        
        qbeq  l_c1Parse14, r1.w0, 0
        mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE 
      
        sub   r1.w0, r1.w0, 1
        lsl   r1.w0, r1.w0, 6
        add   r1.w0, r1.w0, r1.w2
      
        // load the default control bit map 
        lbco  r2.b0, PAMEM_CONST_PORTCFG, r1.w0, 1   
        
l_c1Parse14_DFR_MC:        
        //Is it a multicast packet
        qbbc   l_c1Parse14_DFR_BC, s_pktCxt.flag.t_flag_mac_multicast 
            //multicast packet
            qbbc  l_c1Parse14, r2.b0.t_default_route_mc_enable
            qbbc  l_c1Parse14, r2.b0.t_default_route_mc_pre_classify_enable
                jmp l_c1Parse14_DFR_end
        
l_c1Parse14_DFR_BC:
        //Is it a broadcast packet
        qbbc   l_c1Parse14, s_pktCxt.flag.t_flag_mac_broadcast 
            //broadcast packet
            qbbc  l_c1Parse14, r2.b0.t_default_route_bc_enable
            qbbc  l_c1Parse14, r2.b0.t_default_route_bc_pre_classify_enable
            
l_c1Parse14_DFR_end:            
            mov s_param.action,          SUBS_ACTION_FWPKT2
                
            // pass through

#endif
    
            
l_c1Parse14:
    wbs   s_flags.info.tStatus_CDEOutPacket
    mov   s_pktCxt.paCmdId_Length, (SIZE(s_pktCxt) + 7) & 0xf8      // round up to a multiple of 8 bytes
    sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

    // Packet is ready for lookup
    qbne  l_c1Parse16,  s_param.action,  SUBS_ACTION_LOOKUP

        // Hold the packet
        mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
        mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
        xout XID_CDECTRL,             s_cdeCmdPkt,              SIZE(s_cdeCmdPkt)
        
        // update View3a key byte
        mov  s_l1View3a.KeyByte,   s_runCxt.keyFlags
        xout XID_LUT1V3,           s_l1View3a.KeyByte,  SIZE(s_l1View3a.KeyByte)

        qbbs  l_c1Parse15, s_runCxt.flags.t_activeLookup

            // No previous lookup is active
#ifdef PASS_USE_PKTID
            mov s_runCxt.lookupPktId,           s_pktCxt.scratch
#endif            
            set s_runCxt.flags.t_activeLookup

            //mov s_l1Cmd.careFlags, s_runCxt.keyFlags
            mov s_l1Cmd.cmd,       LUT1_CMD_SEARCHNS
            
            //Wait for LUT1 add/delete to be completed in case there is one pending
            wbc  s_flags.info.tStatus_Lut1Busy

            xout  XID_LUT1CMD,  s_l1Cmd,  SIZE(s_l1Cmd)
            jmp   fci_mainLoop6

l_c1Parse15:

            // There is an active lookup
            set s_runCxt.flags.t_pendingSubmit
            jmp fci_mainLoop6


l_c1Parse16:
        // No lookup is required, see whether we should discard the packet
        qbeq  l_c1Parse18,  s_param.action,  SUBS_ACTION_DISCARD
        // No lookup is required, see whether it is fail route
        qbbs  l_c1Parse19,  s_runCxt.flag2.t_failLookup
        
        // No lookup is required, see whether we can forward the packet now
        qbbc  l_c1Parse17, s_runCxt.flags.t_activeLookup
            // There is active lookup pending and therefore, packet held; 
            // hold the packet,set fakeLookup flag
            set  s_runCxt.flags.t_fakeLookup
            mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
            mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET | CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            xout XID_CDECTRL,             s_cdeCmdPkt,              SIZE(s_cdeCmdPkt)
            jmp  l_c1Parse15
        
l_c1Parse17:
        // No active lookup pending, it is OK to forward the packet
        qbeq  l_c1Parse17_1,  s_param.action,  SUBS_ACTION_FWPKT
#ifdef PASS_PROC_DEF_ROUTE   
        qbeq  f_BcMcCurrentPktForward,  s_param.action,  SUBS_ACTION_FWPKT2
#endif        
            // It is an error packet. 
            // Forward the packet based on the error index, which must be loaded into r30.w2
            // Error Packet Handling 
            lsr  r30.w2,  s_pktCxt2.eP1C2Id,  PKT2_EIDX_SHIFT
            jmp  f_errCurrentPktForward

l_c1Parse17_1:
            // It is a normal packet (IP forwarding) 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            
l_c1Parse17_2: // From l_c1Parse18 as well           
            xout XID_CDECTRL,             s_cdeCmdPkt,                SIZE(s_cdeCmdPkt)
            jmp  fci_mainLoop6
            
l_c1Parse18:
            // Discard this packet
            // Do a silent discard, release the packet ID
            mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_FLUSH
            jmp   l_c1Parse17_2
            
l_c1Parse19:
           // Hold the packet even if activeLookup is not set
           set  s_runCxt.flags.t_fakeLookup
           // Hold the packet
           mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
           mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
           xout XID_CDECTRL,             s_cdeCmdPkt,              SIZE(s_cdeCmdPkt)
           // Wait for the held packet to appear in the CDE
           wbs   s_flags.info.tStatus_CDEHeldPacket
           qbbs  l_c1Parse15, s_runCxt.flags.t_activeLookup
           jmp  fci_mainLoop6
            

    .leave  lut1Scope
    .leave  pktScope
    .leave  cdeScope
    .leave  checkScope

    .using  startScope
    .using  initScope
f_c1LocalInit:

     // Clear the LUT table of all 64 entries
     .using lut1Scope
     
     mov  r3, 0
l_c1LocalInitD:
     qble l_c1LocalInitC, r3, 64
     // Wait for LUT free 
     wbc s_flags.info.tStatus_Lut1Busy

     mov  s_l1Cmd.index,     r3
     mov  s_l1Cmd.cmd,       LUT1_CMD_REMOVE
     xout XID_LUT1CMD,       s_l1Cmd,            SIZE(s_l1Cmd)

     add r3, r3, 1

     jmp  l_c1LocalInitD

     .leave lut1Scope
l_c1LocalInitC:

    // Initialize memory used only by this pdsp

    // The lut1 associated memory. There are 64 entries, each of which is
    // 36 bytes, organized as follows: 
    //
    //   Byte offset    byte size    description
    //      0              4          l1Info
    //      4             16          paForward for matches
    //      20            16          paForward for previous matches

    zero  &r3,                    36       // zeros r3 - r11
    clr s_l1Info.ctrlFlag.custom_enable    // redundant, but safe if someone changes FALSE.
    mov s_paForward.forwardType,  PA_FORWARD_TYPE_DISCARD   // default successful route
    mov s_paForward2.forwardType, PA_FORWARD_TYPE_DISCARD   // default failure route

    mov r2.w0,  0
    mov r2.w2,  64*64

c1LocalInit_0:
    sbco  s_l1Info,       PAMEM_CONST_PDSP_LUT1_INFO, r2.w0, 36
    add   r2.w0,          r2.w0,  64
    qbne  c1LocalInit_0,  r2.w0,  r2.w2


    // Clear the bitmask the indicates which LUT1 entries are valid
    zero  &s_l1Map,  SIZE(s_l1Map)         // also redundant, but this is init code so be safe
    sbco   s_l1Map,  PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)


    // Clear the pending configuration enable 
    zero  &s_l1Pend,  SIZE(s_l1Pend)
    sbco   s_l1Pend, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING, SIZE(s_l1Pend)


    // Initialize the call table. Each PDSP with a LUT1 will do this, but they will
    // each write the same values
    // zero    &s_headerParse.c1ParseMac,  OFFSET(s_headerParse.c2ParseUdp)
	// set the call table for all 24 entries to unknown parse
	mov  r0.w0,               f_c1ParseUnkn
	mov  r1.b0,               &s_headerParse.c1ParseMac
	mov  r0.w2,               SIZE(struct_headerParse)/2
c1LocalClearCallTbl:	
	mviw *r1.b0++,             r0.w0
	sub   r0.w2,               r0.w2,  1
	qbne  c1LocalClearCallTbl, r0.w2, 0

#ifdef PASS_PROC_L2    
    mov s_headerParse.c1ParseMac,            f_c1ParseMac
    mov s_headerParse.c1ParseVlan,           f_c1ParseVlan
    mov s_headerParse.c1ParseMpls,           f_c1ParseMpls
    mov s_headerParse.c1ParsePPPoE,          f_c1ParsePPPoE
#endif
    
#ifdef PASS_PROC_L3    
    mov s_headerParse.c1ParseIpv4,           f_c1ParseIpv4
    mov s_headerParse.c1ParseIpv6,           f_c1ParseIpv6
    mov s_headerParse.c1ParseIpv6ExtHop,     f_c1ParseIpv6ExtHop
    mov s_headerParse.c1ParseIpv6ExtRoute,   f_c1ParseIpv6ExtRoute
    mov s_headerParse.c1ParseIpv6ExtFrag,    f_c1ParseIpv6ExtFrag
    mov s_headerParse.c1ParseIpv6ExtDestOpt, f_c1ParseIpv6ExtDestOpt
    mov s_headerParse.c1ParseGre,            f_c1ParseGre
    mov s_headerParse.c1ParseEsp,            f_c1ParseEsp
    mov s_headerParse.c1parseEspDec,         f_c1ParseEspDec
    mov s_headerParse.c1ParseAuth,           f_c1ParseAuth
    mov s_headerParse.c1ParseCustom,         f_c1ParseCustom
    mov s_headerParse.c1ParseSctp,           f_c1ParseSctp
#endif    
    // No need to re-initialze the unknown parse as it is done already   
    // mov s_headerParse.c1ParseUnkn,           f_c1ParseUnkn

    // arg4 is too long if I use a direct set. Since this is init code the extra two instructions are OK
    // mov  r0.b0,                     OFFSET(s_headerParse.c2ParseUdp)
    //add  r0.b0,                     r0.b0,                            SIZE(s_headerParse.c1ParseUnkn)
    //sbco s_headerParse.c1parseMac,  PAMEM_CONST_PARSE,                OFFSET(s_headerParse.c1ParseMac), b0
    sbco s_headerParse.c1parseMac,    PAMEM_CONST_PARSE,                OFFSET_PARSE_TABLE, SIZE(struct_headerParse)
    // Verify the PDSP ID
    lbco  r0.b0,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1
    qbeq  c1LocalInit_1, r0.b0, 0 
    
    .using  ipScope
    // Clear IP reassembly context
    zero &s_ipReassmCxt, OFFSET(s_ipReassmCxt.tfMapTemp)  
    sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    .leave  ipScope
    
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************************************

    .using  lut1Scope
    .using  cdeScope
    .using  configScope

f_paComDelLut1:
    xin  XID_CDEDATA,  s_paDelL1,  SIZE(s_paDelL1)

    qbge  l_paComDelLut1_0,  s_paDelL1.index,  PA_LUT1_INDEX_LAST_FREE

        mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT1_INDEX_INVALID
        xout XID_CDEDATA,             s_paCmd1.commandResult,                 SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply

l_paComDelLut1_0:

    // Read in the allocated index bitmap
    lbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)
    
    // clear the bit. Its not an error to disable an already disabled entry
    qble l_paComDelLut1_1,  s_paDelL1.index,   32
       clr  s_l1Map.validEntries0,  s_l1Map.validEntries0,  s_paDelL1.index
       jmp  l_paComDelLut1_2 

l_paComDelLut1_1:
    sub  s_paDelL1.rsvd1,        s_paDelL1.index,        32
    clr  s_l1Map.validEntries1,  s_l1Map.validEntries1,  s_paDelL1.rsvd1


l_paComDelLut1_2:

    // Save the map
    //sbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)

    // Disable all the routing info
    zero  &s_paAddL1f,            SIZE(s_paAddL1f)+SIZE(s_paFwdMatchPlace)+SIZE(s_paFwdNextFailPlace)
    mov    s_paFwdA.forwardType,  PA_FORWARD_TYPE_DISCARD
    mov    s_paFwdB.forwardType,  PA_FORWARD_TYPE_DISCARD
    mov    s_paFwdA.flowId,       0xde                     // Indicate that this entry has been deleted

    // see whether an active search is in progress, need to wait for it to be completed
    qbbs  l_paComDelLut1_3,  s_runCxt.flags.t_activeLookup
    
        // Save the map
        sbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)

        // wait for previous add/delete to be complete
        wbc   s_flags.info.tStatus_Lut1Busy

        // If the LUT is free do the deletion
        mov  s_l1Cmd.index,     s_paDelL1.index
        mov  s_l1Cmd.cmd,       LUT1_CMD_REMOVE
        xout XID_LUT1CMD,       s_l1Cmd,            SIZE(s_l1Cmd)

        // Store the deleted forward table
        lsl  r0.w0,       s_paDelL1.index,               6        // The offset to this table entry
        sbco s_paAddL1f,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))
        jmp  f_cfgReply

l_paComDelLut1_3:
    // If the LUT is busy save the disable info
    mov   s_l1Pend.mode,  SUBS_LUT1_PENDING_MODE_DEL_ENTRY
    mov   s_l1Pend.index, s_paDelL1.index

    sbco  s_l1Pend,   PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,    SIZE(s_l1Pend)
    sbco  s_paAddL1f, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_PENDING_TABLE, SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))
    set   s_runCxt.flags.t_pendingConfig
    jmp   f_cfgReply
    
    .leave lut1Scope
    .leave cdeScope
    .leave configScope


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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
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
// *   R0:    w2 - scratch,  w0 - the packet length (entry), offset to command result (exit)
// *   R1:        |  L1 valid entry bit map
// *   R2:        |
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                     |                                      
// *   R7:        |                                     |  s_paCmd1 (entry)
// *   R8:        |                                     |
// *   R9:        |  LUT1 View1   - lut1Scope           |
// *   R10:       |                                         | s_paAddL1Hdr
// *   R11:       |
// *   R12:       |
// *   R13:       |                                         | s_paAddL1f
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
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
    zero  &s_paAddL1f,             SIZE(s_paAddL1f)
    
    // Make sure the packet has all the data
    mov  r0.w2,  SIZE(s_paCmd1)+SIZE(s_paAddL1Hdr)+(2*SIZE(s_paFwdMatchPlace)) 
    add  r0.w2,  r0.w2,  SIZE(s_paAddL1StdA)+SIZE(s_paAddL1StdB)+SIZE(s_paAddL1StdC)
    add  r0.w2,  r0.w2,  SIZE(s_paAddL1StdD)
    qble l_paComAddRepLut1_0,  r0.w0,  r0.w2
        mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_INVALID_PKT_SIZE
        xout XID_CDEDATA,             s_paCmd1.commandResult,               SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply

l_paComAddRepLut1_0:

    // Read in the allocated index bitmap
    lbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)

    // Get a free index if requested
    qbne  l_paComAddRepLut1_3,  s_paAddL1Hdr.index,  PA_LUT1_INDEX_LAST_FREE

        lmbd  r0.b3,  s_l1Map.validEntries1,  0
        qbne  l_paComAddRepLut1_1, r0.b3,                  32

            lmbd  r0.b3,  s_l1Map.validEntries0,  0
            qbne l_paComAddRepLut1_2,  r0.b3,                  32

                // No free entries
                mov s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT1_INDEX_INVALID
                xout XID_CDEDATA,            s_paCmd1.commandResult,               SIZE(s_paCmd1.commandResult)
                jmp f_cfgReply

l_paComAddRepLut1_1:

    // Free entry found in MSW
    add r0.b3,   r0.b3,  32

l_paComAddRepLut1_2:

    // Write back the index
    // If an error is detected after this the return value will have an index, but
    // the index will not be valid
    mov   s_paAddL1Hdr.index,  r0.b3
    xout  XID_CDEDATA,         s_paAddL1Hdr.index,  SIZE(s_paAddL1Hdr.index)


l_paComAddRepLut1_3:

    // Verify the index is in range
    qbgt  l_paComAddRepLut1_4,  s_paAddL1Hdr.index,  PA_LUT1_INDEX_LAST_FREE
        mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT1_INDEX_INVALID
        xout XID_CDEDATA,             s_paCmd1.commandResult,                 SIZE(s_paCmd1.commandResult)
        jmp  f_cfgReply


l_paComAddRepLut1_4:

    mov s_l1Pend.mode,  SUBS_LUT1_PENDING_MODE_ADD_ENTRY
    mov s_l1Pend.index, s_paAddL1Hdr.index

    // Scroll past the command headers
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  SIZE(s_paCmd1)+SIZE(s_paAddL1Hdr)
    xout XID_CDECTRL,           s_cdeCmdWd,                          SIZE(s_cdeCmdWd)

    qbeq  f_paComAddLut1Standard,  s_paAddL1Hdr.type, PA_COM_ADD_LUT1_SRIO
    qbeq  f_paComAddLut1Custom,    s_paAddL1Hdr.type, PA_COM_ADD_LUT1_CUSTOM
    qbeq  f_paComAddLut1Standard,  s_paAddL1Hdr.type, PA_COM_ADD_LUT1_VLINK
    qbeq  f_paComAddLut1Standard,  s_paAddL1Hdr.type, PA_COM_ADD_LUT1_STANDARD

    // Otherwise it was an invalid type field 
    // Note; It will not return here for AddLUT1 routes
    mov  s_paCmd1.commandResult,  PA_COMMAND_RESULT_LUT1_TYPE_INVALID
    set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
    jmp  f_cfgReply


fci_paComAddRepLut1_5:

    // Update the LUT1 utilization maps
    qbgt  l_paComAddRepLut1_51, s_paAddL1Hdr.index, 32
      sub  r0.b3,                  s_paAddL1Hdr.index, 32
      set  s_l1Map.validEntries1,  r0.b3
      jmp  l_paComAddRepLut1_52


l_paComAddRepLut1_51:
      set  s_l1Map.validEntries0,  s_paAddL1Hdr.index

l_paComAddRepLut1_52:

    // Store the allocated index bitmap
    //sbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)

    // The CDE is now pointing to the Packet forward information.
    xin  XID_CDEDATA,  s_paFwdMatchPlace,  2*SIZE(s_paFwdMatchPlace)

    // store all the info (l1Info + match info)

    // Complete the entry if there is no active search in progress, need to wait for it to be completed
    qbbs  l_paComAddRepLut1_6,  s_runCxt.flags.t_activeLookup
    
        // Store the allocated index bitmap
        sbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)
        
        // wait for previous entry update to be completed
        wbc  s_flags.info.tStatus_Lut1Busy
        
        lsl  r0.w0,       s_l1Pend.index,                6        // The offset to this table entry
        sbco s_paAddL1f,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))

        mov  s_l1Cmd.careFlags, s_l1Pend.matchFlags
        mov  s_l1Cmd.index,     s_l1Pend.index
        mov  s_l1Cmd.cmd,       LUT1_CMD_INSERT
        xout XID_LUT1CMD,       s_l1Cmd,            SIZE(s_l1Cmd)

        jmp f_cfgReply


l_paComAddRepLut1_6:

    // The LUT was busy. Store the information in scratch until the LUT is free
    sbco  s_l1Pend,   PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,    SIZE(s_l1Pend)
    sbco  s_paAddL1f, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_PENDING_TABLE, SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))
    set   s_runCxt.flags.t_pendingConfig
    jmp   f_cfgReply

    
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
// *   R26:  w0 - packet Id
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// ************************************************************************************/

    .using configScope
    .using cdeScope
    .using lut1Scope

f_paComAddLut1Standard:

   // Mac  Info
   xin  XID_CDEDATA, s_paAddL1StdA,  SIZE(s_paAddL1StdA)
   xout XID_LUT1V1,  s_paAddL1StdA,  SIZE(s_paAddL1StdA)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1StdA)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   // IP Info
   xin  XID_CDEDATA, s_paAddL1StdB,  SIZE(s_paAddL1StdB)
   xout XID_LUT1V2,  s_paAddL1StdB,  SIZE(s_paAddL1StdB)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1StdB)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   // Misc
   xin  XID_CDEDATA, s_paAddL1StdC,  SIZE(s_paAddL1StdC)
   xout XID_LUT1V3,  s_paAddL1StdC,  SIZE(s_paAddL1StdC)
   
   qbne l_paComAddLut1Standard_vlink, s_paAddL1Hdr.type, PA_COM_ADD_LUT1_SRIO
        // Populate the s_paAddL1f
        set     s_paAddL1f.ctrlFlag.custom_enable
        mov     s_paAddL1f.nextHdr,       s_paAddL1SrioC.nextHdr
        mov     s_paAddL1f.nextHdrOffset, s_paAddL1SrioC.nextHdrOffset
l_paComAddLut1Standard_vlink:
   qbne l_paComAddLut1Standard0, s_paAddL1Hdr.type, PA_COM_ADD_LUT1_VLINK
        // Populate the virtual link number
        set     s_paAddL1vlnk.ctrlFlag.vlink_enable
        mov     s_paAddL1vlnk.vLinkNum,      s_paAddL1Hdr.vLinkNum
        
l_paComAddLut1Standard0:
   mov  s_cdeCmdWd.byteCount,  SIZE(s_paAddL1StdC)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   xin  XID_CDEDATA,          s_paAddL1StdD,            SIZE(s_paAddL1StdD)
   mov  s_l1Pend.matchFlags,  s_paAddL1StdD.matchFlags

   mov  s_cdeCmdWd.byteCount, SIZE(s_paAddL1StdD)
   xout XID_CDECTRL,          s_cdeCmdWd,           SIZE(s_cdeCmdWd)

l_paComAddLut1Standard1:

    jmp  fci_paComAddRepLut1_5       

    .leave configScope
    .leave cdeScope
    .leave lut1Scope
  
// ********************************************************************************************************
// * FUNCTION PURPOSE: Enter a custom configuration into the LUT
// ********************************************************************************************************
// * DESCRIPTION: The custom match data is added
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - offset to the reply result (input)
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************************/

    .using cdeScope
    .using configScope
    .using lut1Scope

f_paComAddLut1Custom:

   // Mac  Info : only the etype and vlan is used for link 
   xin  XID_CDEDATA, s_paComL1CustA,  SIZE(s_paComL1CustA)
   xout XID_LUT1V1,  s_paComL1CustA,  SIZE(s_paComL1CustA)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paComL1CustA)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   // Custom Match 
   xin  XID_CDEDATA, s_paComL1CustB,  SIZE(s_paComL1CustB)
   xout XID_LUT1V2,  s_paComL1CustB,  SIZE(s_paComL1CustB)

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paComL1CustB)
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   // View3: only key is used
   xin  XID_CDEDATA, s_paComL1CustC,  SIZE(s_paComL1CustC)
   xout XID_LUT1V3,  s_paComL1CustC,  SIZE(s_paComL1CustC)

   // Scroll past this and the unused next window
   mov  s_cdeCmdWd.byteCount,    SIZE(s_paComL1CustC)
   xout XID_CDECTRL,             s_cdeCmdWd,            SIZE(s_cdeCmdWd)

   xin  XID_CDEDATA,          s_paComL1CustD,           SIZE(s_paComL1CustD)
   mov  s_l1Pend.matchFlags,  s_paComL1CustD.matchFlags

   mov  s_cdeCmdWd.byteCount, SIZE(s_paAddL1StdD)
   xout XID_CDECTRL,          s_cdeCmdWd,           SIZE(s_cdeCmdWd)

   jmp  fci_paComAddRepLut1_5

    .leave cdeScope
    .leave configScope
    .leave lut1Scope
    
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
    .using cdeScope
    .using configScope
    .using pktScope
    .using  lut1Scope
    
f_cfgReplyHoldPkt:

   qbbc  l_cfgReplyHoldPkt_1, s_runCxt.flags.t_activeLookup
        // Active lookup pending, need to hold the packet
        or  s_cdeCmdPkt.optionsFlag, s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
        set s_runCxt.flags.t_fakeLookup
        set s_runCxt.flags.t_pendingSubmit
#ifdef PASS_USE_PKTID        
        mov s_runCxt.heldPktId, s_pktCxt3.pktId
#endif        
        mov s_param.action,  SUBS_ACTION_FWPKT
        xout XID_CDECTRL,             s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
        jmp   fci_mainLoop7
        
l_cfgReplyHoldPkt_1:
        ret
        
    .leave  cdeScope
    .leave  configScope
    .leave  pktScope
    .leave  lut1Scope
    


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
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        |  LUT1 View1   - lut1Scope
// *   R10:       |                                 | s_l1Pend
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
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
    lbco  s_l1Pend,   PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING,  SIZE(s_l1Pend)
    lbco  s_paAddL1f, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_PENDING_TABLE, SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))

    lsl  r0.w0,       s_l1Pend.index,                6        // The offset to this table entry
    sbco s_paAddL1f,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))

    mov  s_l1Cmd.careFlags, s_l1Pend.matchFlags
    mov  s_l1Cmd.index,     s_l1Pend.index
    mov  s_l1Cmd.cmd,       LUT1_CMD_INSERT
    qbeq l_paComLut1CompleteModify0,           s_l1Pend.mode,      SUBS_LUT1_PENDING_MODE_ADD_ENTRY
        mov s_l1Cmd.cmd,    LUT1_CMD_REMOVE

l_paComLut1CompleteModify0:

    xout XID_LUT1CMD,     s_l1Cmd,            SIZE(s_l1Cmd)
    
    // Update the flags
    // Read in the allocated index bitmap
    lbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)
    
    // clear the bit. Its not an error to update the entry twice
    qble l_paComLut1CompleteModify2, s_l1Pend.index,   32
    
l_paComLut1CompleteModify1:    
        qbeq l_paComLut1CompleteModify1_1,  s_l1Pend.mode,  SUBS_LUT1_PENDING_MODE_ADD_ENTRY
            clr  s_l1Map.validEntries0,  s_l1Map.validEntries0,  s_l1Pend.index
            jmp  l_paComLut1CompleteModify3
            
l_paComLut1CompleteModify1_1:            
            set  s_l1Map.validEntries0,  s_l1Map.validEntries0,  s_l1Pend.index
            jmp  l_paComLut1CompleteModify3

l_paComLut1CompleteModify2:
        sub  s_l1Pend.index,        s_l1Pend.index,        32
        qbeq l_paComLut1CompleteModify2_1,  s_l1Pend.mode,  SUBS_LUT1_PENDING_MODE_ADD_ENTRY
            clr  s_l1Map.validEntries1,  s_l1Map.validEntries1,  s_l1Pend.index
            jmp  l_paComLut1CompleteModify3
    
l_paComLut1CompleteModify2_1:
            set  s_l1Map.validEntries1,  s_l1Map.validEntries1,  s_l1Pend.index
            // pass through
            
l_paComLut1CompleteModify3:
    // Save the map
    sbco  s_l1Map,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map)

    jmp  fci_mainLoop7

    .leave lut1Scope
    .leave configScope
    .leave cdeScope
    
#include "parse1.p"

    .leave globalScopeC1

