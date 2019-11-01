// **************************************************************************************************************
// * FILE PURPOSE: Peform packet classification on PDSPS with a LUT1
// **************************************************************************************************************
// * FILE NAME: classify1.p
// *
// * DESCRIPTION: The PDSP code for L2 and L3 classification using a LUT1
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
#define PASS_CLASSIFY1  
#include "pdsp_pa.h"
#include "pdsp_mem.h"
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

#include "meminit.p"

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
     // mov  r2, 0
     // sbco r2, cMailbox, 4, 4      // Clear the mailbox

l_c1Start0:

    zero  &s_runCxt,      SIZE(s_runCxt)

    // Need not store PDSP ID as it is replaced with #define
    // Store the PDSP ID
    // TBD: To be replaced with #define (from image build)
    mov   r2.b0, PASS_PDSP_ID
    sbco  r2.b0, PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1

#ifdef PASS_PROC_RA
    // Temporary debug code
    //set s_runCxt.flag2.t_raEn, 
    //mov   s_runCxt.raFlow, PASS_RA_FLOW_ID
#endif    
    
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

.assign struct_timeAccConstants,  r3,     r5,    s_ConstLoc
.assign struct_timeAcc,           r0,     r2,    s_AccLoc

f_mainLoop:
#ifdef PASS_TIMESTAMP_OP 
 qbbc l_mainLoop000, s_runCxt.flag3.t_eoamEn 
    // Only when the timer feature is enabled
    // look for timer roll over
    qbbc l_mainLoop000, s_flags.info.tStatus_Timer
        set  s_flags.info.tStatus_Timer // set to clear timer event 
        lbco r0, PAMEM_CONST_CUSTOM,  OFFSET_SYS_TIMESTAMP,   8
        add r0, r0, 1
        adc r1, r1, 0
        sbco r0, PAMEM_CONST_CUSTOM,  OFFSET_SYS_TIMESTAMP,   8
        
        // Call EOAM time convertion routine
        call f_eoamTimeConvert
l_mainLoop000:
#endif


#ifdef PASS_PROC_CMD1
   // Look for command 
    qbbc l_mainLoop001, s_flags.info.tStatus_Command1
     // If there is a command, differentiate between EOAM and IP Reassem
     // in the PDSP that has both features
     lbco  r1, cMailbox, 4, 4     
#ifdef PASS_PROC_IPSEC_NAT_T_EOAM 
     qbne  l_mainLoop_not_ipsec_nat_t_cfg, r1.b3, FIRMWARE_CMD_IPSEC_NAT_T_EOAM_CFG
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
        sbco  r1, cMailbox, FIRMWARE_CMD_IPSEC_NAT_T_EOAM_CFG_OFFSET, 4     
        
        jmp   l_mainLoop001
l_mainLoop_not_ipsec_nat_t_cfg:     
#endif
#ifdef PASS_PROC_EOAM_CMD
     qbne  l_mainLoop_not_eoam_cfg, r1.b3, FIRMWARE_CMD_EOAM_CFG
        // Clear the valid bit fields 
        not r1.b2, r1.b1
        // force clear valid bits to zero and retain valid bits positions
        and s_runCxt.flag3, s_runCxt.flag3, r1.b2

        // Clear garbage ctrl bit fields (since the sender already cleard it no need */
        // and r1.b0, r1.b0, r1.b1
        
        // set the run time flags
        or  s_runCxt.flag3, s_runCxt.flag3, r1.b0

        // clear the command flag
        mov   r1, 0           
        sbco  r1, cMailbox, FIRMWARE_CMD_EOAM_CFG_OFFSET, 4
        jmp   l_mainLoop001
        
l_mainLoop_not_eoam_cfg: 
#endif

#ifdef PASS_PROC_IP_REASSEM          
     qbne  l_mainLoop_not_reasm_cfg, r1.b3, FIRMWARE_CMD_IP_REASSEM_CFG
        // Process IP Reassembly Configuration Update
        mov  r0.w0, FIRMWARE_INNER_IP_PDSP 
        qbeq l_mainLoop00_ipReassem_1, r0.w0,   PASS_PDSP_ID            
            mov r1.w0, OFFSET_OUT_IP_REASSM_CFG
            jmp l_mainLoop00_ipReassem_2
          
l_mainLoop00_ipReassem_1:        
            mov r1.w0, OFFSET_IN_IP_REASSM_CFG
            
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
        sbco  r1, cMailbox, FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET, 4
        
        // pass through        
l_mainLoop_not_reasm_cfg:         
#endif  

#endif
l_mainLoop001:


#ifdef PASS_PROC_CMD3

   // Look for command (FIRMWARE_CMD_PKT_CTRL_CFG only) 
    qbbc l_mainLoop003, s_flags.info.tStatus_Command3

       // Process Packet Verification Configuration Update
       lbco  r1, cMailbox, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
       qbne  l_mainLoop002, r1.b3, FIRMWARE_CMD_PKT_CTRL_CFG   
        // set the packet control flags    
#ifdef PASS_PROC_PKT_CONTROL 
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
        sbco  r1, cMailbox, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4
        
        jmp   l_mainLoop003
#endif

l_mainLoop002:

   // Look for command (FIRMWARE_CMD_RA_CFG only) 
//    qbbc l_mainLoop003, s_flags.info.tStatus_Command3
        // Process RA Configuration Update
//        lbco  r1, cMailbox, FIRMWARE_CMD_RA_CFG_OFFSET, 4
      qbne  l_mainLoop003, r1.b3, FIRMWARE_CMD_RA_CFG        
        and s_runCxt.flag2, s_runCxt.flag2, NOT_SUBS_RA_CTRL_MASK      
#ifndef PASS_PROC_RA        
        // Retain only RA Enable ctrl bit for other IP Processing PDSPs (clear other bits)
        and r1.b0,   r1.b0, SUBS_PKT_CTRL_RA_En
#endif
        or  s_runCxt.flag2, s_runCxt.flag2, r1.b0
        
#ifdef PASS_PROC_RA
        mov s_runCxt.raFlow, r1.b1 
#endif            
        // clear the command flag
        mov   r1, 0           
        sbco  r1, cMailbox, FIRMWARE_CMD_RA_CFG_OFFSET, 4
        
        // pass through       
#endif

l_mainLoop003:

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
               wbc   s_flags.info.tStatus_CDEBusy
               
               // Load the first 20-byte to pktInfo
               // The first 5 32-bit word in the pkt context may be examized and patched
               lbco   s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    20
               
               qbbc  l_mainLoop0,               s_l1Status.status.L1_STATUS_ENTRY_MATCH
                    #ifndef PASS_PROC_FIREWALL
                    call  f_c1ForwardHeldPacketMatch
                    #else
                        #ifdef PASS_PROC_EOAM
                            qbbc  l_c1FwdFirewallMatch, s_runCxt.flag3.t_eoamEn                     
                            call  f_c1ForwardHeldEoamPktMatch
                            jmp   l_mainLoop1
                        #endif  
l_c1FwdFirewallMatch:                      
                    call  f_c1ForwardHeldFirewallPktMatch
                    #endif
                    jmp   l_mainLoop1

l_mainLoop0:   
               #ifndef PASS_PROC_FIREWALL
               call  f_c1ForwardHeldPacketNoMatch
               #else
                        #ifdef PASS_PROC_EOAM
                            qbbc  l_c1FwdFirewallNoMatch, s_runCxt.flag3.t_eoamEn                     
                            call  f_c1ForwardHeldEoamPktNoMatch
                            jmp   l_mainLoop1
                        #endif  
l_c1FwdFirewallNoMatch:                  
               call  f_c1ForwardHeldFirewallPktNoMatch
               #endif
l_mainLoop1:
               clr   s_runCxt.flags.t_previous_held

#ifdef PASS_PROC_INGRESS_PKT_CLONE_OR_PA_ROUTE_L2_CAPTURE
               // check whether there is extended header ready to be sent out due to packet copy
               qbbc l_mainLoop1_0, s_runCxt.flags.t_extHdr_held
                    //restore the extended header
                    lbco  s_pktExtDescr,  PAMEM_CONST_PDSP_CXT,  OFFSET_EXT_HDR_PENDING, SIZE(s_pktExtDescr)
                    clr s_runCxt.flags.t_extHdr_held 
                    // send the packet out
                    xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr) 
                    // pass through
l_mainLoop1_0:
#endif               
               qbbc  l_mainLoop5, s_runCxt.flags.t_held_packet

                   // An already parsed packet is waiting to be submitted to the LUT
                   clr   s_runCxt.flags.t_held_packet
                   qbbs  l_mainLoop1_1, s_runCxt.flags.t_previous_skip
                        // Initiate Lookup for pending packet
                        set   s_runCxt.flags.t_previous_held
                        //  zero    &s_l1Cmd, SIZE(s_l1Cmd)
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
                    // mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_RELEASE  
                    ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_RELEASE 
                      
#ifdef PASS_LAST_PDSP                        
                    // TBD: clear Bypass Flag
                    lbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
                    clr   r1.w0.t_pktBypass
                    sbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
                    qbbc    l_mainLoop1_2,  s_pktExtDescr.flags.fDroppedInd 
                        set   s_pktExtDescr.flags.fDropped
                        //clr   s_pktExtDescr.flags.fDroppedInd
                        mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
l_mainLoop1_2:                    
#endif               
                    xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
                    xout XID_CDECTRL,    s_cdeCmdPkt,    4
                    jmp  l_mainLoop2
                        
l_mainLoop2:
                   //clr  s_runCxt.flags.t_activeLookup   
                   jmp  l_mainLoop5

        // if ( (pendingConfig == FALSE) && (newPacket == TRUE) ) then parse
l_mainLoop5:
    qbbs  fci_mainLoop6, s_runCxt.flags.t_pendingConfig
        #ifdef PASS_PROC_FIREWALL
        // No special check for EOAM case is needed as rescore is never triggered
        qbbc l_mainLoop5_0,s_runCxt.flag3.t_trigRescoreProc
          // Skip newpacket parse, during rescore state and during held packets
          qbbs fci_mainLoop6, s_runCxt.flags.t_previous_held
          qbbs fci_mainLoop6, s_runCxt.flags.t_held_packet
            // Rescore can be scheduled now as the LUT1 is clean (nothing held)
            call f_c1FirewallRescore
l_mainLoop5_0:        
        #endif
        qbbs l_mainLoop9,  s_flags.info.tStatus_CDENewPacket
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

l_mainLoop9:
    // Relative jump was out of range
    jmp f_c1Parse
   
   .leave  lut1Scope
   .leave  pktScope
   .leave  cdeScope 


#ifdef PASS_TIMESTAMP_OP  
// ****************************************************************************************************
// * FUNCTION PURPOSE: The EOAM time convertion processing function
// ****************************************************************************************************
// * DESCRIPTION: The EOAM Time Convert processing function
// *
// *        Register usage:
// *
// *   R0:    scratch (r0  = roll over cnt lsw)
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch
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

// Local structure defintion and assignment    
.struct strct_thrChk
  .u16    numRo
  .u16    rollOverCnt
.ends

.assign strct_thrChk,             r3,     r3,    s_thrChk
.assign struct_timeAcc,           r0,     r2,    s_accLoc

f_eoamTimeConvert:   
//
// uint32_t nsAcc;
// 
// Below code is to covert the ticks to seconds/nano seconds
// if (rollover)
// {
//   /* increment the roll over count, when detected */
//   rollOverCnt++;
//  
//   /* accumulate the nano seconds time associated with a roll over */
//   nsAccWithErr += nsRoAcc;
//   
//   /* Check if we reached the threshold to reset for the error accumulation */
//   if (rollOverCnt == numRo)
//   {
//     /* Clear the rollOver count */ 
//     rollOverCnt = 0;  
//     /* clear the current accumulation till threshold to zero */
//     nsAccWithErr = 0;
//     /* Increment big Roll Over accumulation, correcting for the errors */
//     nsAccNoErr += nsNumRoAcc;
//     /* Check if we crossed a second in accumulation */
//     if (nsAccNoErr > PA_ONE_SEC_EXP_IN_NS)
//     {
//       nsAccNoErr -= PA_ONE_SEC_EXP_IN_NS;
//       secAcc++;
//     }
//   }
// }  

     // Load the accumulation constants 
     lbco    s_thrChk,              PAMEM_CONST_RO_TIME_ACC_TABLE,  OFFSET_NUMRO,        SIZE(s_thrChk)
     add     s_thrChk.rollOverCnt,  s_thrChk.rollOverCnt,           1
     
     // load accumulation values (global)
     lbco    s_accLoc,              PAMEM_CONST_CUSTOM,             OFFSET_RAW_TIME_ACC, SIZE(s_accLoc) 

     lbco    r5,               PAMEM_CONST_RO_TIME_ACC_TABLE,       OFFSET_NS_ROACC,     4
     add     s_accLoc.nsAccWithErr, s_accLoc.nsAccWithErr,          r5

  
     // Check if we reached the threshold to reset for the error accumulation
     qblt      l_eoamTimeConvert_1, s_thrChk.numRo, s_thrChk.rollOverCnt
       mov     s_thrChk.rollOverCnt, 0

       lbco    r5,              PAMEM_CONST_RO_TIME_ACC_TABLE, OFFSET_NS_NUM_ROACC, 4        
       add     s_accLoc.nsAccNoErr,   s_accLoc.nsAccNoErr,           r5
       
       // We can overwrite this as it is not used at this time and it is not stored in the scratch memory
       mov     r5, PA_ONE_SEC_EXP_IN_NS
       
       // clear it locally before writing to scratch memory
       mov     s_accLoc.nsAccWithErr, 0
       
       qblt  l_eoamTimeConvert_1,    r5,         s_accLoc.nsAccNoErr
         sub   s_accLoc.nsAccNoErr,  s_accLoc.nsAccNoErr,           r5
         add   s_accLoc.secAcc,      s_accLoc.secAcc,               1        

l_eoamTimeConvert_1:       
       // store the roll over count
       sbco    s_thrChk.rollOverCnt,  PAMEM_CONST_RO_TIME_ACC_TABLE,  OFFSET_RO_COUNT,     SIZE(s_thrChk.rollOverCnt)
       
       // store the accumulation values (global)
       sbco    s_accLoc,     PAMEM_CONST_CUSTOM,   OFFSET_RAW_TIME_ACC, SIZE(s_accLoc)
       ret   
  
#endif   
   
#ifdef PASS_PROC_FIREWALL   
   
// **************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that had a match in the Firewall LUT1 search
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
// *   R8:   LUT1 info (s_l1fAcl)                                       -
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

f_c1ForwardHeldFirewallPktMatch:

    //Record the match
    //mov    s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_MATCH 

    // Load the routing info associated with LUT1 
    // This is the l1Info and match routing information
    lsl    r1.w0,    s_l1Status.index,             6                                     // l1 index * 256
    lbco   &s_l1fAcl,   PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,   SIZE(s_l1fAcl)+SIZE(s_fmPlace)  // l1Info + routing info
    
    // TBD: Update Firewall-related statistics
    mov s_stats.value,  s_l1fAcl.statsCmd  // Update the packet counter
    sub r0, s_pktCxt.endOffset, s_pktCxt.startOffset
    mov s_stats.value,  s_l1fAcl.statsCmd2 // Update the byte counter
     
    
    
    // Record the effective packet size for user statistics update
    // mov s_usrStatsReq.pktSize,  s_pktCxt.endOffset 
    
    // Load control flags
    // Note that it should be loaded only once to prevent infinite re-entry for fragments routing (already loaded)
    // lbco s_pktCxt.flags,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.flags), SIZE(s_pktCxt.flags)  
    
fci_stdHeldFirewallPktForward:    

    // Do not release the packet ID if the packet remains in PA
    qbne  l_stdHeldFirewallPktForward1,  s_matchForward.forwardType,  PA_FORWARD_TYPE_PA 
    
        qbbc  l_stdHeldFirewallPktForward0_0,   s_matchForward_pa.ctrlflags.t_pa_fwd_ctrlflags_pkt_mark
            // Mark this package for further processing
            // The first 16-byte of packet Info has already been loaded 
            set    s_pktCxt.flags.t_flag_acl_mark
            sbco   s_pktCxt.flags, cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flags),    SIZE(s_pktCxt.flags)
    
l_stdHeldFirewallPktForward0_0:

        qbbs  l_stdHeldFirewallPktForward0_1,   s_runCxt.flag2.t_raEn
            //  Send the packet on its way
            //  Free the packet Ext Info
            mov  s_pktExtDescr.threadId, s_matchForward_pa.dest

            //   Load flags and operation in one instruction
            ldi  r4, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
            jmp  l_stdHeldFirewallPktForward0_2
        
l_stdHeldFirewallPktForward0_1:        
            // Send the packet on its way to RA
            qbbs l_stdHeldFirewallPktForward0_set_ra_dest_1,    s_runCxt.flag2.t_raUseLocDMA
                // use global DMA     
                mov  s_pktExtDescr.threadId, THREADID_CDMA0
                mov  s_cdeCmdPkt.destQueue,  PASS_RA_QUEUE
                jmp  l_stdHeldFirewallPktForward0_set_ra_dest_end
                
l_stdHeldFirewallPktForward0_set_ra_dest_1:  
                // use local DMA
                mov  s_pktExtDescr.threadId, THREADID_CDMA1
                mov  s_cdeCmdPkt.destQueue,  PASS_RA_LOC_QUEUE
                // pass through              
                
l_stdHeldFirewallPktForward0_set_ra_dest_end:                
            //   Free the packet Ext Info
            //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
            //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
            ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
            sbco s_cdeCmdPkt.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
            mov  s_cdeCmdPkt.psInfoSize,  32
            mov  s_cdeCmdPkt.flowId,      s_runCxt.raFlow
            
            qbbc  l_stdHeldFirewallPktForward0_2,   s_matchForward_pa.ctrlflags.t_pa_fwd_ctrlflags_pkt_drop
                // Set drop flag
                lbco  r1.b0,  cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
                set   r1.b0.t_psFlags_pktDropFromRA
                sbco  r1.b0,  cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
            //pass through
        
l_stdHeldFirewallPktForward0_2:        
        xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
        xout  XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
        ret
        
l_stdHeldFirewallPktForward1:
        
    qbeq  l_stdHeldFirewallPktForward3,  s_matchForward.forwardType, PA_FORWARD_TYPE_DISCARD

fci_stdHeldFirewallPktForward2:   // A jump in point for other functions to use
                                  // the system error info
        // Record Error and then pass through                  

l_stdHeldFirewallPktForward3: 
        //   Free the packet Ext Info
#ifdef PASS_LAST_PDSP
        ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_DISCARD 
        //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
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
// * FUNCTION PURPOSE: Forward a packet that failed a Firewall LUT1 search
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

f_c1ForwardHeldFirewallPktNoMatch:

    // Inc the no match stat
    //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_NO_MATCH
    
#ifdef PASS_OUTER_ACL
    lbco  s_paAclCfg, PAMEM_CONST_CUSTOM,  OFFSET_OUT_IP_ACL_CFG, SIZE(s_paAclCfg)
#else
    lbco  s_paAclCfg, PAMEM_CONST_CUSTOM,  OFFSET_IN_IP_ACL_CFG, SIZE(s_paAclCfg)
#endif    
    
    qbeq  l_c1ForwardHeldFirewallPktNoMatch_1,   s_paAclCfg.action,   PA_FORWARD_TYPE_HOST
        // Forwarding or discard
        mov s_matchForward_pa.dest,      PASS_FW_DEST_THREAD_ID
        mov s_matchForward_pa.ctrlflags, 0
        jmp fci_stdHeldFirewallPktForward

l_c1ForwardHeldFirewallPktNoMatch_1:
        // Host Routing
        // Send the packet on its way
        // Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
            
l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce:
        // Check for Queue Bounce operation
l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_ddr:
        qbbc l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_msmc, s_paAclCfg.destQueue.t_pa_forward_queue_bounce_ddr
            clr s_paAclCfg.destQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paAclCfg.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paAclCfg.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_end

l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_msmc:
        qbbc l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_end, s_paAclCfg.destQueue.t_pa_forward_queue_bounce_msmc
            clr s_paAclCfg.destQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paAclCfg.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paAclCfg.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_c1ForwardHeldFirewallPktNoMatch_1_queue_bounce_end:
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE 
        //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8)
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.destQueue,   s_paAclCfg.destQueue
        sbco s_paAclCfg.destQueue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        mov  s_cdeCmdPkt.flowId,      s_paAclCfg.destFlowId
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        ret
        
f_c1ForwardFirewallPktNoMatch:

    // Inc the no match stat
    //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_NO_MATCH
    
#ifdef PASS_OUTER_ACL
    lbco  s_paAclCfg, PAMEM_CONST_CUSTOM,  OFFSET_OUT_IP_ACL_CFG, SIZE(s_paAclCfg)
#else
    lbco  s_paAclCfg, PAMEM_CONST_CUSTOM,  OFFSET_IN_IP_ACL_CFG, SIZE(s_paAclCfg)
#endif    
    
    qbeq  l_c1ForwardFirewallPktNoMatch_2,   s_paAclCfg.action,   PA_FORWARD_TYPE_HOST
    qbeq  l_c1ForwardFirewallPktNoMatch_1,   s_paAclCfg.action,   PA_FORWARD_TYPE_PA
    
        // Discard the packet
        set  s_pktExtDescr.flags.fDroppedInd
        ldi  r4, CDE_CMD_PACKET_ADVANCE
        jmp  fci_c1Parse14_1    
    
l_c1ForwardFirewallPktNoMatch_1:

        #ifdef PASS_RA_ERR_PKT
        qbbs  l_c1ForwardFirewallPktNoMatch_1_1,   s_runCxt.flag2.t_raEn
        #endif
            //  Send the packet on its way
            //  Free the packet Ext Info
            mov  s_pktExtDescr.threadId,  PASS_FW_DEST_THREAD_ID

            //   Load flags and operation in one instruction
            ldi  r4, CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
            jmp  fci_c1Parse14_1    
        
#ifdef PASS_RA_ERR_PKT
        
l_c1ForwardFirewallPktNoMatch_1_1:        
            // Send the packet on its way to RA
            qbbs l_c1ForwardFirewallPktNoMatch_1_set_ra_dest_1,    s_runCxt.flag2.t_raUseLocDMA
                // use global DMA     
                mov  s_pktExtDescr.threadId, THREADID_CDMA0
                mov  s_cdeCmdPkt.destQueue,  PASS_RA_QUEUE
                jmp  l_c1ForwardFirewallPktNoMatch_1_set_ra_dest_end
                
l_c1ForwardFirewallPktNoMatch_1_set_ra_dest_1:  
                // use local DMA
                mov  s_pktExtDescr.threadId, THREADID_CDMA1
                mov  s_cdeCmdPkt.destQueue,  PASS_RA_LOC_QUEUE
                // pass through              
                
l_c1ForwardFirewallPktNoMatch_1_set_ra_dest_end:        
            //   Free the packet Ext Info
            //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
            //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8)
            ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  32
            mov  s_cdeCmdPkt.flowId,      s_runCxt.raFlow
            sbco s_cdeCmdPkt.destQueue,   cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
            jmp  fci_c1Parse14_1    

#endif
    
l_c1ForwardFirewallPktNoMatch_2:
        // Host Routing
        // Send the packet on its way
        // Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0

l_c1ForwardFirewallPktNoMatch_2_queue_bounce:
        // Check for Queue Bounce operation
l_c1ForwardFirewallPktNoMatch_2_queue_bounce_ddr:
        qbbc l_c1ForwardFirewallPktNoMatch_2_queue_bounce_msmc, s_paAclCfg.destQueue.t_pa_forward_queue_bounce_ddr
            clr s_paAclCfg.destQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paAclCfg.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paAclCfg.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1ForwardFirewallPktNoMatch_2_queue_bounce_end

l_c1ForwardFirewallPktNoMatch_2_queue_bounce_msmc:
        qbbc l_c1ForwardFirewallPktNoMatch_2_queue_bounce_end, s_paAclCfg.destQueue.t_pa_forward_queue_bounce_msmc
            clr s_paAclCfg.destQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paAclCfg.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paAclCfg.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_c1ForwardFirewallPktNoMatch_2_queue_bounce_end:
            
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        mov  r4.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.destQueue,   s_paAclCfg.destQueue
        mov  s_cdeCmdPkt.flowId,      s_paAclCfg.destFlowId
        sbco s_paAclCfg.destQueue,    cCdeOutPkt, OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)  
        //xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        jmp  fci_c1Parse14_1
        
    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    
#endif    

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
// *   R6:         | packet capture info (L2 PDSP only)
// *   R7:         |
// *   R8:   LUT1 info (s_l1f)                                       -
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
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:  
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

#ifndef PASS_PROC_FIREWALL

f_c1ForwardHeldPacketMatch:

    //     Record the match
    mov    s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_MATCH 

    // Load the routing info associated with LUT1 
    // This is the l1Info and match routing information
    lsl    r1.w0,    s_l1Status.index,             6                                     // l1 index * 256
    lbco   &s_l1f,   PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,   SIZE(s_l1f)+SIZE(s_fmPlace)  // l1Info + routing info

    // The following fields must be patched into the packet context with
    // Results from the lookup
    //   // previous LUT1 match bit
    //   // custom C2 bit
    //   - LUT1 PDSP ID
    //   - LUT1 match index
    // The 16 bit word s_pktCxt.phyLink will be created in r1.w0
    // and then patched in
  
    // The run context has the PDSP ID 
    //lsl    s_pktCxt.phyLink,  s_runCxt.pdspId,    PKT_PDSP_ID_SHIFT
    mov      s_pktCxt.phyLink,  PASS_PDSP_ID << PKT_PDSP_ID_SHIFT
   
    //  Or in the LUT1 matching index
    or     s_pktCxt.phyLink,  s_pktCxt.phyLink,   s_l1Status.index

    // Set the bit to indicate a LUT1 match has occurred
    set    s_pktCxt.flags.t_flag_pl1Match  
    
    // Set the virtual link enable bit if necessary
    qbbc   l_c1ForwardHeldPacketMatch0, s_l1f.ctrlFlag.vlink_enable
        set s_pktCxt.flags.t_flag_use_vlink                 
        // Store virtual link num in s_pktCxt.vlanPri_vLink for further stages
        and    s_pktCxt.vlanPri_vLink.b1, s_pktCxt.vlanPri_vLink.b1,  PKT_VALN_PRI_MASK 
        or     s_pktCxt.vlanPri_vLink.b1, s_pktCxt.vlanPri_vLink.b1,  s_l1fv.vLinkNum.b1
        mov    s_pktCxt.vlanPri_vLink.b0, s_l1fv.vLinkNum.b0
        sbco   s_pktCxt.vlanPri_vLink, cCdeHeldPkt, SIZE(s_pktDescr)+OFFSET(s_pktCxt.vlanPri_vLink), SIZE(s_pktCxt.vlanPri_vLink)

l_c1ForwardHeldPacketMatch0:
    // The pmatch, c2c, pdsp ID and LUT1 index are patched into psinfo
    sbco   s_pktCxt.flags,     cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.flags), SIZE(s_pktCxt.flags)
    sbco   s_pktCxt.phyLink,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.phyLink), SIZE(s_pktCxt.phyLink)

#endif
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
// *   R0:
// *   R1:    scratch
// *   R2:    scratch  (r2.b0: psInfoSize)
// *   R3:
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        | packet capture info (L2 PDSP only)
// *   R7:        |
// *   R8:   LUT1 info (s_l1f)                                       -
// *   R9:      |                                                    -
// *   R10:     |  Forward match Info (valid at function exit)       -  lut1MatchSCope
// *   R11:     |                                                    -  Must be valid on Entry
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
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:  
// *   R28:  event (INTD) enabled mask register  (s_eventFlags)     -
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
    // lbco    s_usrStatsReq.pktSize,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset) 
    mov s_usrStatsReq.pktSize,  s_pktCxt.endOffset 
    
    // Load control flags
    // Note that it should be loaded only once to prevent infinite re-entry for fragments routing (already loaded)
    // lbco s_pktCxt.flags,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.flags), SIZE(s_pktCxt.flags)  
    
fci_stdHeldPktForward_retry:    
    // Default  thread Id for command set is THREADID_POST
    // The value will be changed to PA_DEST_CDMA only if QoS routing is required
    // The packet will be routed to QoS queues and then maybe forwarded to Q645 where command set can be executed
    mov     r2.b2,  THREADID_POST    

    // Do not release the packet ID if the packet remains in PA
    qbne  l_stdHeldPktForward1,  s_matchForward.forwardType,  PA_FORWARD_TYPE_PA
        //
        // IP Fragemnt packet can not be forwarded within PA
        //
        qbbs l_stdHeldPktForward11,  s_pktCxt.flags.t_flag_frag
    
        // L1 Info handling
        qbbc l_stdHeldPktForward0_1,  s_l1f.ctrlFlag.custom_enable
        
        // Patch the startOffset and nextHdr
        sbco   s_l1f.nextHdrOffset,  cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.startOffset), SIZE(s_pktCxt.startOffset)
        //lbco   r1.w0,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2
        and    s_pktCxt.eId_portNum_nextHdr.b0,   s_pktCxt.eId_portNum_nextHdr.b0, NOT_PKT_NEXTHDR_MASK
        or     s_pktCxt.eId_portNum_nextHdr.b0,   s_pktCxt.eId_portNum_nextHdr.b0, s_l1f.nextHdr
        sbco   s_pktCxt.eId_portNum_nextHdr,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2
    
l_stdHeldPktForward0_1:
        // Custom Routing handling 
        qbeq l_stdHeldPktForward0_4,  s_matchForward_pa.custType, PA_CUSTOM_TYPE_NONE
        
        // Patch the pktContext
        // eIndex should stote the custom Index
        // Patch the next header Type
        //lbco   r1.w0,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2
        and    s_pktCxt.eId_portNum_nextHdr.b1,   s_pktCxt.eId_portNum_nextHdr.b1, NOT_PKT_EIDX_MASK
        lsl    r1.b2,   s_matchForward_pa.custIdx, PKT_EIDX_SHIFT
        or     s_pktCxt.eId_portNum_nextHdr.b1,   s_pktCxt.eId_portNum_nextHdr.b1,  r1.b2
        
        and    s_pktCxt.eId_portNum_nextHdr.b0,   s_pktCxt.eId_portNum_nextHdr.b0, NOT_PKT_NEXTHDR_MASK
        qbeq   l_stdHeldPktForward0_2,  s_matchForward_pa.custType, PA_CUSTOM_TYPE_LUT1
        or     s_pktCxt.eId_portNum_nextHdr.b0,   s_pktCxt.eId_portNum_nextHdr.b0,  PA_HDR_CUSTOM_C2
        jmp    l_stdHeldPktForward0_3
l_stdHeldPktForward0_2:        
        or     s_pktCxt.eId_portNum_nextHdr.b0,   s_pktCxt.eId_portNum_nextHdr.b0,  PA_HDR_CUSTOM_C1
l_stdHeldPktForward0_3:        
        sbco   s_pktCxt.eId_portNum_nextHdr,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr), 2

l_stdHeldPktForward0_4:
        //   Free the packet Ext Info
#ifndef PASS_LAST_PDSP
    #ifndef PASS_FIREWALL_FORWARDING
        qbne l_stdHeldPktForward0_4_1, s_matchForward_pa.dest, PASS_THREAD_ID
    #else
        // Custom packet does not go through Firewall or RA engine
        qbne l_stdHeldPktForward0_4_1, s_matchForward_pa.custType, PA_CUSTOM_TYPE_NONE
    #endif   
            // Need to process this packet at the next PDSP 
            lbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
            clr   r1.w0.t_pktBypass
            sbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#endif    

l_stdHeldPktForward0_4_1:
        //  Verify various control flags
        qbbc l_stdHeldPktForward0_5, s_matchForward_pa.ctrlflags.t_pa_cascaded_forwarding
            // update the pktCtx.flag2
            set  s_pktCxt.flags.t_flag_cascaded_forwarding
            sbco s_pktCxt.flags,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flags), SIZE(s_pktCxt.flags)
            // pass through
    

l_stdHeldPktForward0_5:

#ifdef PASS_PROC_PA_ROUTE_L2_CAPTURE
        qbbc l_stdHeldPktForward0_5_l2_cap_end, s_matchForward_pa.ctrlflags.t_pa_l2_capture
            // prepare packet for L2 capture at the next stage
            // Patch swinfo0, queue and flow Id
            sbco s_matchForward_pa.context,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  4
            sbco s_matchForward.queue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)
            sbco s_matchForward.flowId,   cCdeHeldPkt, OFFSET(s_pktDescr.flowIdx), SIZE(s_pktDescr.flowIdx)  
            // Need to capture this packet at the next PDSP 
            lbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
            set   r1.w0.t_pktCapture
            sbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)

l_stdHeldPktForward0_5_l2_cap_end:
#endif

        mov     s_pktExtDescr.threadId, s_matchForward_pa.dest
        xout    XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   

        //   Load flags and operation in one instruction
        ldi  r4, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
        //mov  s_cdeCmdPkt.threadId,     s_matchForward_pa.dest
        mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
        xout XID_CDECTRL,              s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
        
        //Is there user-stats update command 
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd,    PA_RX_CMD_USR_STATS
        ret

l_stdHeldPktForward1:
    // Forwarding packets to the host
    qbne  l_stdHeldPktForward3,  s_matchForward.forwardType,  PA_FORWARD_TYPE_HOST
    
        //lbco   r1.b0,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt5.l3l5offset), 1
        qbeq   l_stdHeldPktForward1_00, s_pktCxt5.l3l5offset, 0
            sbco   s_pktCxt5.l3l5offset,   cCdeHeldPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt5.l3offset2), 1
    
l_stdHeldPktForward1_00:    
        //
        // Perform IP Fragemnt packet check if configured and it is not cascaded forwarding packet
        //
        qbbs l_stdHeldPktForward1_0, s_pktCxt.flags.t_flag_cascaded_forwarding
        qbbc l_stdHeldPktForward1_0, s_runCxt.flag2.t_ipFragToEroute
            qbbs l_stdHeldPktForward11,  s_pktCxt.flags.t_flag_frag
            // pass through
    
l_stdHeldPktForward1_0:
    
        // Patch swinfo0
        sbco s_matchForward_host.context,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_stdHeldPktForward1_1, s_matchForward_host.psflags,   0
            lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            and  r2.b0, r2.b0,  NOT_PA_PKT_PS_FLAGS_MASK
            and  r0.b0, s_matchForward_host.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
            or   r2.b0, r2.b0, r0.b0
            sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
            // Set EMAC output port
            and  r0.b0, s_matchForward_host.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
            sbco r0.b0, cCdeHeldPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
            // TBD: Clear TimeSync word (Should we clear time stamp 
                                      
l_stdHeldPktForward1_1:        
        // The first 2 32-bit word in the pkt context may be examized and patched
        // For multi routing: Set flag and the multi route index, update Command ID
        // For command set command: Set flag and command set index, update Command ID
        // For CRC verification: Check the flag and update Command ID 
        // Both words are read in, changed, and sent out at once
        //lbco   s_pktCxt,                cCdeHeldPkt,              SIZE(s_pktDescr),    8
        
        // Check whether interface based routing is enabled or not 
        qbbc l_stdHeldPktForward1_no_if_changes, s_matchForward_host.ctrlBitmap.t_pa_routing_if_selection
            lsr  r2.b1, s_pktCxt.eId_portNum_nextHdr, PKT_EMACPORT_SHIFT
		    and  r2.b1, r2.b1, PKT_EMACPORT_MASK
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
                 and   r2.b3, s_matchForward_host.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
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
                     lsr  r1.b1, s_pktCxt.vlanPri_vLink.b1,   PKT_VLAN_PRI_SHIFT
                     jmp  l_stdHeldPktForward1_comp
l_stdHeldPktForward1NotVlanTag:
                   qbbs l_stdHeldPktForward1_comp, r1.b0.t_eqos_pri_override
l_stdHeldPktForward1DscpEqosMode: 
#ifndef PASS_PROC_L2
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_IP_MASK << PROT_COUNT_IP_SHIFT
                   qbeq l_stdHeldPktForward1_comp, r0.b0, 0
#else
                   and  r0.b0, s_pktCxt.eId_portNum_nextHdr.b0, PKT_NEXTHDR_MASK
                   qbeq l_stdHeldPktForward1ProcDscp, r0.b0, PA_HDR_IPv4
                   qbne l_stdHeldPktForward1_comp, r0.b0, PA_HDR_IPv6
l_stdHeldPktForward1ProcDscp:                   
#endif                   
                     // point to dscp table
                     add  r1.w2, r1.w2, 16
                     mov  r1.b1, s_pktCxt.priority
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
            //lbco r2.b0,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.vlanPri_vLink), 1
            lsr  r2.b0,    s_pktCxt.vlanPri_vLink,   PKT_VLAN_PRI_SHIFT
            add  s_matchForward.queue, s_matchForward.queue, r2.b0
            mov  r2.b2,  THREADID_CDMA0
l_stdHeldPktForward1_dscp_priority:
        // No priority routing enabled 
        qbbc l_stdHeldPktForward1_no_priority, s_matchForward_host.ctrlBitMap.t_pa_routing_priority_dscp
            //lbco r2.b0,    cCdeHeldPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.priority), 1
            add  s_matchForward.queue, s_matchForward.queue, s_pktCxt.priority
            mov  r2.b2,  THREADID_CDMA0
            
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
        mov    r2.b0,   SIZE(s_pktCxt) + 8 
        mov    s_pktExtDescr.threadId, THREADID_CDMA0
        
        // check whether the CRC verification is enabled, which precedes all other special case
        qbbc   l_stdHeldPktForward1_2, s_pktCxt.flags.t_flag_crc_verify
        
#ifndef PASS_LAST_PDSP
            // Need to process this packet at the next PDSP 
            lbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
            clr   r1.w0.t_pktBypass
            sbco  r1.w0,  cCdeHeldPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#endif        
            jmp l_stdHeldPktForward10 

l_stdHeldPktForward1_2:
        
l_stdHeldPktForward1_3:        
        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        qbbs l_stdHeldPktForward2,  s_matchForward_host.ctrlBitMap.t_pa_multiroute
            // Normal host route: no special operation is required
            // Send the packet on its way
            //   Free the packet Ext Info
            //mov     s_pktExtDescr.threadId, THREADID_CDMA0
            xout    XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   
            
            //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE 
            //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
            mov  r4.w0,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
            //mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
            //mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
            sbco s_matchForward.queue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
            mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
            xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
            qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret

l_stdHeldPktForward2:     
            // For multi routing the command ID and the multi route index
            // must be patched into the packet. These are two bytes which span 2 32 bits words.
            // Both words are read in, changed, and sent out at once

            // Note: It can be skipped since the original commad is 0 
            //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
            or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
            set  s_pktCxt.flags.t_flag_multi_route
    
            and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
            lsl  r3.b0,              s_matchForward_host.multiIdx,   PKT_EIDX_SHIFT
            or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   r3.b0
            sbco s_pktCxt,           cCdeHeldPkt,                    SIZE(s_pktDescr),    8

            // Send the packet on its way
            //   Free the packet Ext Info
            mov     s_pktExtDescr.threadId, THREADID_POST
            xout    XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   
            
            ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            //mov  s_cdeCmdPkt.threadId,    s_matchForward_host.paPdspRouter
            xout XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
            qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret


l_stdHeldPktForward3:
    and   r2.b0, s_matchForward.forwardType, PA_FORWARD_TYPE_MASK
    qbne  l_stdHeldPktForward3_1, r2.b0, PA_FORWARD_TYPE_SA
        //   Record ThreadId
        qbbs l_stdHeldPktForward3_set_threadId_1, s_matchForward.forwardType.t_pa_fwd_type_ctrl_use_loc_dma
            mov  s_pktExtDescr.threadId, THREADID_CDMA0
            jmp  l_stdHeldPktForward3_2
l_stdHeldPktForward3_set_threadId_1:
            mov  s_pktExtDescr.threadId, THREADID_CDMA1
            jmp  l_stdHeldPktForward3_2   
        
l_stdHeldPktForward3_1:
    qbne  l_stdHeldPktForward4, r2.b0, PA_FORWARD_TYPE_SA_DIRECT
            mov  s_pktExtDescr.threadId,  s_matchForward.flowId

l_stdHeldPktForward3_2:
    
        //
        // IP Fragemnt packet can not be forwarded to SA
        //
        qbbs l_stdHeldPktForward11,  s_pktCxt.flags.t_flag_frag
    
        // Routing to SA
        // Patch swinfo0 and swinfo2
        sbco s_matchForward_sa.swInfo0,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  8
        
        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_stdHeldPktForward9,  s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        
        // Send the packet on its way
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE 
        //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8       // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        //mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
        sbco s_matchForward.queue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ), SIZE(s_pktDescr.destQ)  
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
        //   Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE 
        //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  8                       // SRIO take 8 bytes Ps Info
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        //mov  s_cdeCmdPkt.destQueue,   s_matchForward.queue
        sbco s_matchForward.queue,    cCdeHeldPkt, OFFSET(s_pktDescr.destQ), SIZE(s_pktDescr.destQ)  
        mov  s_cdeCmdPkt.flowId,      s_matchForward.flowId
        xout  XID_CDECTRL,            s_cdeCmdPkt,                           SIZE( s_cdeCmdPkt)
        ret
        

l_stdHeldPktForward5:
    qbne  l_stdHeldPktForward6, s_matchForward.forwardType, PA_FORWARD_TYPE_ETH
        // Routing to ETH
        // Clear psflags
        lbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        and  r0.b0, s_matchForward_eth.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
        or   r2.b0, r2.b0, r0.b0
        sbco r2.b0, cCdeHeldPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
        // Set EMAC output port
        and  r0.b0, s_matchForward_eth.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
        sbco r0.b0, cCdeHeldPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
        // Clear TimeSync word
        mov  r0, 0
        sbco r0, cCdeHeldPkt, OFFSET(s_pktDescrCpsw.tsWord), 4
        
        // Send the packet on its way
        //   Free the packet Ext Info
        add  s_pktExtDescr.threadId,  s_matchForward_eth.priority, THREADID_ETHERNET1
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  0                       // remove pktCtx
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
        xout  XID_CDECTRL,            s_cdeCmdPkt,            SIZE( s_cdeCmdPkt)
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_stdHeldPktForward6:

#ifdef PASS_PROC_EFLOW_ROUTE

    // Only Egress flow is supported
    qbne  l_stdHeldPktForward7,  s_matchForward.forwardType,  PA_FORWARD_TYPE_EFLOW 
        // TBD: may want to add more error check
        // TBD: other cleanup?
        //mviw  *&s_txPktCxt.paCmdId_Length,  PSH_CMD_PA_TX_FLOW << (PKT_CONTEXT_CMD_SHIFT + 8)
        mov   s_txPktCxt.paCmdId_Length,  PSH_CMD_PA_TX_FLOW << PKT_CONTEXT_CMD_SHIFT
        mov   s_txPktCxt.flags.b1, 0
        qbbc  l_stdHeldPktForward6_1, s_matchForward_ef.ctrlflags.t_pa_ef_ctrlflags_fc_lookup
            set s_txPktCxt.flags.t_flag_fc_lookup
            // pass through

l_stdHeldPktForward6_1:
        and   r0.b0, s_pktCxt.eId_portNum_nextHdr.b0,   PKT_NEXTHDR_MASK    
        lsl   s_txPktCxt.flags.b0, s_matchForward_ef.validBitMap,  4
        mvid  *&s_txPktCxt.lvl1RecIndex, *&s_matchForward_ef.lvl1RecIndex 
        mov   s_txPktCxt.l2Offset, 0

#ifdef PASS_PROC_L2
    // end of L2 processing
    // MAC-Routing only
    mov  s_pktExtDescr.threadId,  THREADID_EGRESS2
    mov s_txPktCxt.l3Offset, s_pktCxt.startOffset
    mov s_txPktCxt.ipHdrLen, 0
    sub s_txPktCxt.fragSize, s_pktCxt.endOffset, s_pktCxt.startOffset
    mov s_txPktCxt.lastFragSize,    s_txPktCxt.fragSize
    sbco  s_txPktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    SIZE(s_txPktCxt)
#endif


#ifdef PASS_PROC_OUTER_IP
    // end of Outer IP processing
    mov  s_pktExtDescr.threadId,  THREADID_EGRESS0
    #ifdef PASS_DETECT_INNER_IP
    qbeq  l_stdHeldPktForward6_outer_ip_1, r0.b0, PA_HDR_IPv4
    qbeq  l_stdHeldPktForward6_outer_ip_1, r0.b0, PA_HDR_IPv6
        // Single IP 
        mov s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset
        jmp l_stdHeldPktForward6_outer_ip_2
    
l_stdHeldPktForward6_outer_ip_1: 
        // with inner IP   
        mov s_txPktCxt.l3Offset2,   s_txPktCxt.startOffset
    #else
    // Assume there is no inner IP at the outer IP processing stage
    // In other words, the potential inner IP is ignored if IP forwarding occurs at this stage
    mov s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset
    #endif

l_stdHeldPktForward6_outer_ip_2:    
        sbco  s_txPktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    OFFSET(s_txPktCxt.fragSize)
#endif

#ifdef PASS_PROC_INNER_IP
    // end of Inner IP processing
    mov  s_pktExtDescr.threadId,  THREADID_EGRESS0
    mov s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset
    qbeq  l_stdHeldPktForward6_inner_ip_end,    s_pktCxt5.l3l5Offset,   0
    mov s_txPktCxt.l3Offset2,   s_pktCxt5.l3l5Offset
    mov s_txPktCxt.l4Offset,    s_pktCxt.startOffset
l_stdHeldPktForward6_inner_ip_end:
    sbco  s_txPktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    OFFSET(s_txPktCxt.fragSize)
    
#endif
    // Send the packet on its way
    //   Free the packet Ext Info
    xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
    
    ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
    mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8       // Round up to multiple of 8 bytes
    xout  XID_CDECTRL,            s_cdeCmdPkt,            SIZE( s_cdeCmdPkt)
    qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
    ret
    
#endif

l_stdHeldPktForward7:

    qbeq  l_stdHeldPktForward8,  s_matchForward.forwardType, PA_FORWARD_TYPE_DISCARD

fci_stdHeldPktForward7:   // A jump in point for other functions to use
                          // the system error info
        // Record Error and then pass through                  

l_stdHeldPktForward8: 
        // Do a silent discard, release the packet
        mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_DISCARD
        
        //   Free the packet Ext Info
        set  s_pktExtDescr.flags.fDroppedInd
        
#ifdef PASS_LAST_PDSP
        //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_DISCARD
        ldi   s_cdeCmd.v0,    CDE_CMD_HPKT_DISCARD 
        
        set   s_pktExtDescr.flags.fDropped
#else
        ldi  s_cdeCmd.v0,    CDE_CMD_HPKT_RELEASE 
        //mov   s_cdeCmdPkt.operation, CDE_CMD_HPKT_RELEASE
        //mov   s_cdeCmdPkt.optionsFlag, 0
#endif        
        xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        xout  XID_CDECTRL,           s_cdeCmdPkt,            4
        //Is there user-stats update command 
        qbeq l_stdHeldPktForward12,  s_matchRxCmdHdr.cmd,    PA_RX_CMD_USR_STATS
        ret
        
l_stdHeldPktForward9:     
        // For command set command, the command ID and the command set index
        // must be patched into the packet. These are two bytes which span 2 32 bits words.
        // Both words are read in, changed, and sent out at once
        set  s_pktCxt.flags.t_flag_cmdset
        qbne l_stdHeldPktForward9_0, r2.b2, THREADID_CDMA0
            // during QoS the destination would have been modified to CDMA0, indicate it in reserved bit 14, 
            // for egress PDSPs to handle when it gets from QoS.
            set  s_pktCxt.flags.t_flag_eg_cmdSet        
l_stdHeldPktForward9_0:
        sbco s_matchRxCmdSet.index,  cCdeHeldPkt, SIZE(s_pktDescr) + OFFSET(s_pktCxt4.cmdSetIdx),     SIZE(s_pktCxt4.cmdSetIdx) 
        
        mov  r2.b0,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        
        // record the original threadId 
        sbco s_pktExtDescr.threadId, cCdeHeldPkt, SIZE(s_pktDescr) + OFFSET(s_pktCxt4.threadIdOrig),  SIZE(s_pktCxt4.threadIdOrig)
        
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
        //  Free the packet Ext Info
        mov  s_pktExtDescr.threadId, r2.b2
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
               
        ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_PA_M_0
        mov  s_cdeCmdPkt.psInfoSize,  r2.b0         
        xout XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
        qbeq l_stdHeldPktForward12,   s_matchRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        ret
        
l_stdHeldPktForward11:
        // IP fragmented packet
        // Follow the exception route
        // Note: It is up to the user to set the exception route correctly
        // Note: clear local copy to prevent infinite re-entry
        clr  s_pktCxt.flags.t_flag_frag
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_IP_FRAG << PKT_EIDX_SHIFT
        sbco s_pktCxt.eId_portNum_nextHdr,  cCdeHeldPkt,        SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr),  SIZE(s_pktCxt.eId_portNum_nextHdr)
        
        mov   r1.w0, EROUTE_IP_FRAG*SIZE(s_fmPlace)
        lbco  s_fmPlace,         PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_fmPlace)
        jmp   fci_stdHeldPktForward_retry
        
l_stdHeldPktForward12:
        // User Statistics operation:
        // Record and insert the statistics update into the FIFO
        // Note all user commands are in the same location (r12)
        mov   s_usrStatsReq.index,  s_matchForward_host.cmd.w0
        
        lbco  s_fifoCb, PAMEM_CONST_USR_STATS_FIFO_BASE,  OFFSET_PDSP_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
        add   r1.b0,    s_fifoCb.in, 4
        and   r1.b0,    r1.b0,       0x1F
        
        qbne  l_stdHeldPktForward12_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL
            ret

l_stdHeldPktForward12_1:
        // Insert the request into the FIFO   
        add   r1.w2,   s_fifoCb.in, OFFSET_PDSP_USR_STATS_FIFO
        sbco  s_usrStatsReq,  PAMEM_CONST_USR_STATS_FIFO_BASE, r1.w2, SIZE(s_usrStatsReq)
        sbco  r1.b0,   PAMEM_CONST_USR_STATS_FIFO_BASE,    OFFSET_PDSP_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
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
// *   R26:     |     
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Exception Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *            
// ******************************************************************************************************

    .using currentFwdScope

f_errCurrentPktForward:
    // Load the error routing information
    lsl   r30.w2,         r30.w2,              4
    lbco  s_curFmPlace,   PAMEM_CONST_EROUTE,  r30.w2,   SIZE(s_curFmPlace)

    // Need a return address
    mov r30.w0, fci_c1Parse14_1
    jmp f_c1CurPktForward

    .leave currentFwdScope

#ifndef PASS_PROC_FIREWALL    
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
// *   R14:      | Extended Packet Info
// *   R15:      |
// *   R16:      |
// *   R17:      |
// *   R18:      |
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
    .using  cdeScope
    .using  pktScope
    .using  lut1MatchScope
    .using currentFwdScope

f_nextFailPktForward:

    // Inc the no match stat
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_NO_MATCH 
    mov r30.w0, fci_c1Parse14_1

    // Broadcast and multicast check if no match found 
    // Assume that this rule precedes the pre-match rule
l_nextFailPktForward_BM_1:
    qbbc   l_nextFailPktForward_BM_2, s_pktCxt.flags.t_flag_multicast 
        mov    r1.w0, EROUTE_IP_MULTICAST*SIZE(s_curFmPlace)
        jmp    l_nextFailPktForward_BM_3  
        
l_nextFailPktForward_BM_2:    
    qbbc   l_nextFailPktForward_BM_4, s_pktCxt.flags.t_flag_broadcast 
        mov    r1.w0, EROUTE_IP_BROADCAST*SIZE(s_curFmPlace)
        //pass through
    
l_nextFailPktForward_BM_3:
        // Load the routing information for broadcast/multicast type into the 
        // matchforward context
        lbco  s_curFmPlace,         PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_curFmPlace)
        qbeq  l_nextFailPktForward_BM_4, s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD
        jmp   f_c1CurPktForward
    
l_nextFailPktForward_BM_4:

    qbbs  l_nextFailPktForward0, s_pktCxt.flags.t_flag_pl1Match

        // For no previous match set the error type in the packet, load the 
        // error routing information and route
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_LUT1_FAIL << PKT_EIDX_SHIFT
        //sbco s_pktCxt.eId_portNum_nextHdr,     cCdeOutPkt,         SIZE(s_pktDescr)+OFFSET(eId_portNum_nextHdr),  2

        // Load the routing information for LUT1 fail error type into the 
        // matchforward context
        lbco  s_curFmPlace,            PAMEM_CONST_EROUTE,  EROUTE_LUT1_FAIL << 4,  SIZE(s_curFmPlace)
        jmp   f_c1CurPktForward

l_nextFailPktForward0:

        // In the case of a previous match, the previous match routing information
        // must be loaded into the matchForward context
        lsr  r1.b2,  s_pktCxt.phyLink,     PKT_PDSP_ID_SHIFT   // Get PDSP ID 
        mov  r1.w0,  s_pktCxt.phyLink
        and  r1.b1,  r1.b1,     PKT_L1IDX_MASK
        lsl  r1.w0,  r1.w0,     6                              // get the associated table offset

        lsl  r1.b2,  r1.b2,     2  // PDSP ID * 4 = offset to the nextFail table base address
        lbco r2,     PAMEM_CONST_NFAIL_BASE, r1.b2,     4
        
        //Load the nextfail routing information
        lbbo  s_curFmPlace,        r2,  r1.w0,  SIZE(s_curFmPlace)
        jmp   f_c1CurPktForward


    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope

    .leave currentFwdScope
#endif

// *********************************************************************************************
// * FUNCTION PURPOSE: C1 Current packet forwarding
// *********************************************************************************************
// * DESCRIPTION: The current packet is forwarded based on the forward information
// *              input for exception and nextFail route only
// *    On Entry:
// *        R6-R9  has the forward match structure
// *        R14-R18 extPktInfo
// *          
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
    mov   s_usrStatsReq.pktSize,  s_pktCxt.endOffset 
    
    // Default  thread Id for command set is THREADID_POST
    // The value will be changed to PA_DEST_CDMA only if QoS routing is required
    // The packet will be routed to QoS queues and then maybe forwarded to Q645 where command set can be executed
    mov     r2.b2,  THREADID_POST
    
    // Update the current packet Info
    sbco s_pktCxt,  cCdeOutPkt,     SIZE(s_pktDescr), SIZE(s_pktCxt)

    // Forwarding packets to the host
    qbne  l_c1CurPktForward3,  s_curFwd.forwardType,  PA_FORWARD_TYPE_HOST

        // Patch swinfo0
        sbco s_curFwd_host.context,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_c1CurPktForward1_0,    s_curFwd_host.psflags,  0 
            lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
            and  r0.b0, s_curFwd_host.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
            or   r2.b0, r2.b0, r0.b0
            sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
            // Set EMAC output port
            and  r0.b0, s_curFwd_host.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
            sbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
            // TBD: Clear TimeSync word
            
l_c1CurPktForward1_0:
        // Check whether interface based routing is enabled or not 
        qbbc l_c1CurPktForward1_no_if_changes, s_curFwd_host.ctrlBitmap.t_pa_routing_if_selection
            lsr  r2.b1, s_pktCxt.eId_portNum_nextHdr, PKT_EMACPORT_SHIFT
		    and  r2.b1, r2.b1, PKT_EMACPORT_MASK
            // There is no need to update destion queue and flow if interfcae is unknown
            qbeq  l_c1CurPktForward1_no_priority, r2.b1, 0
                sub r2.b1, r2.b1, 1
                qbbc l_c1CurPktForward1_no_eqos_changes, s_curFwd_host.ctrlBitmap.t_pa_routing_if_eqos
                 // load per port configuration from the scratch memory
                 // get the port number to be captured and obtain the offset  
                 // point to per port configuration structure                 
                 // Extract output port number 
                 // r0.w2: Ingress port base to extract default priority only
                 // r1.b0: port ctrlBitMap (queue offset)
                 // r1.b1: priority (flow offset)
                 // r1.w2: Egress port base (control and Offset table)
                 and   r2.b3, s_curFwd_host.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
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
                 qbbc  l_c1CurPktForward1DscpEqosMode, r1.b0.t_eqos_dp_bit_mode
                   // dp bit mode
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_VLAN_MASK << PROT_COUNT_VLAN_SHIFT
                   qbeq l_c1CurPktForward1NotVlanTag, r0.b0, 0
                     lsr  r1.b1, s_pktCxt.vlanPri_vLink.b1,   PKT_VLAN_PRI_SHIFT
                     jmp  l_c1CurPktForward1_comp
l_c1CurPktForward1NotVlanTag:
                   qbbs l_c1CurPktForward1_comp, r1.b0.t_eqos_pri_override
l_c1CurPktForward1DscpEqosMode: 
#ifndef PASS_PROC_L2
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_IP_MASK << PROT_COUNT_IP_SHIFT
                   qbeq l_c1CurPktForward1_comp, r0.b0, 0
#else
                   and  r0.b0, s_pktCxt.eId_portNum_nextHdr.b0, PKT_NEXTHDR_MASK 
                   qbeq l_c1CurPktForward1ProcDscp, r0.b0, PA_HDR_IPv4
                   qbne l_c1CurPktForward1_comp, r0.b0, PA_HDR_IPv6
l_c1CurPktForward1ProcDscp:                   
#endif                   
                     // point to dscp table
                     add  r1.w2, r1.w2, 16
                     mov  r1.b1, s_pktCxt.priority
                     // pass through
l_c1CurPktForward1_comp:
                   //point to correct priority table offset
                   lsl  r1.b1, r1.b1, 1
                   add  r1.w2, r1.w2, r1.b1
                   lbco r1.w0, PAMEM_CONST_PORTCFG, r1.w2, 2
                   // add to base queue and flow
                   add  s_curFwd.flowId, s_curFwd.flowId, r1.b1
                   add  s_curFwd.queue,  s_curFwd.queue,  r1.b0
                   mov  r2.b2,  PA_DEST_CDMA                   
                   jmp  l_c1CurPktForward1_no_priority                   

l_c1CurPktForward1_no_eqos_changes:

                add  s_curFwd.queue, s_curFwd.queue, r2.b1
		
                qbbc l_c1CurPktForward1_no_priority, s_curFwd_host.ctrlBitmap.t_pa_routing_if_dest_flow
		            // Now update the destination flow based on the ethernet interface
                    add  s_curFwd.flowId, s_curFwd.flowId, r2.b1
                    jmp  l_c1CurPktForward1_no_priority

l_c1CurPktForward1_no_if_changes:

        // adjust destination queue if QoS queue 
        qbbc l_c1CurPktForward1_dscp_priority, s_curFwd_host.ctrlBitMap.t_pa_routing_priority_vlan
l_c1CurPktForward1_vlan_priority: 
            lsr  r2.b0,    s_pktCxt.vlanPri_vLink.b1,   PKT_VLAN_PRI_SHIFT
            add  s_curFwd.queue, s_curFwd.queue, r2.b0
            mov  r2.b2,  THREADID_CDMA0    
l_c1CurPktForward1_dscp_priority:
            // No priority routing enabled 
        qbbc l_c1CurPktForward1_no_priority, s_curFwd_host.ctrlBitMap.t_pa_routing_priority_dscp
            add  s_curFwd.queue, s_curFwd.queue, s_pktCxt.priority
            mov  r2.b2,  THREADID_CDMA0    
            
l_c1CurPktForward1_no_priority:
        // store L3offset2 only if host route
        lbco   r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt5.l3l5offset), 1
        qbeq   l_c1CurPktForward1_1, r1.b0, 0
            sbco   r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt5.l3offset2), 1
            
l_c1CurPktForward1_1:
l_c1CurPktForward1_1_queue_bounce:
        // Check for Queue Bounce operation
l_c1CurPktForward1_1_queue_bounce_ddr:
        qbbc l_c1CurPktForward1_1_queue_bounce_msmc, s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            clr s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1CurPktForward1_1_queue_bounce_end

l_c1CurPktForward1_1_queue_bounce_msmc:
        qbbc l_c1CurPktForward1_1_queue_bounce_end, s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            clr s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_c1CurPktForward1_1_queue_bounce_end:

        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_c1CurPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_c1CurPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        
        qbbs l_c1CurPktForward2,  s_curFwd_host.ctrlBitMap.t_pa_multiroute
        
        // Send the packet on its way
        // Free the packet Ext Info
        mov     s_pktExtDescr.threadId, THREADID_CDMA0
            
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE 
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8)
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        //mov  s_cdeCmdPkt.destQueue,     s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,        s_curFwd.flowId
        sbco s_curFwd.queue,            cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        //xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_c1CurPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_c1CurPktForward2:     
         // For multi routing the command ID and the multi route index
         // must be patched into the packet. These are two bytes which span 2 32 bits words.
         // Both words are read in, changed, and sent out at once
         // lbco s_pktCxt,                 cCdeOutPkt,                 SIZE(s_pktDescr), 8
         // Note: It can be skipped since the original commad is 0 
         //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
         or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,    PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
         set  s_pktCxt.flags.t_flag_multi_route
         
         and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
         lsl  r2.b0,  s_curFwd_host.multiIdx,   PKT_EIDX_SHIFT
         or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   r2.b0
         
         sbco s_pktCxt,                 cCdeOutPkt,                 SIZE(s_pktDescr), 8

         // Send the packet on its way
         //   Free the packet Ext Info
         mov     s_pktExtDescr.threadId, THREADID_POST
         //xout    XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   
    
         // Send the packet on its way
         ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
         mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
         //mov  s_cdeCmdPkt.threadId,    s_curFwd_host.paPdspRouter
         //xout XID_CDECTRL,             s_cdeCmdPkt,                  SIZE(s_cdeCmdPkt)
         qbeq l_c1CurPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
         ret


l_c1CurPktForward3:
l_c1CurPktForward4:
    qbne  l_c1CurPktForward5, s_curFwd.forwardType, PA_FORWARD_TYPE_SRIO
        // Routing to SRIO
        // Overwrite Packet Info with psinfo0 and psinfo2
        sbco s_curFwd_srio.psInfo0,  cCdeOutPkt, SIZE(s_pktDescr),  8
        
        // pactch the packet type
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_TYPE_MASK
        lsl  r2.b1, s_curFwd_srio.pktType, PA_PKT_TYPE_SHIFT
        or   r2.b0, r2.b0,  r2.b1
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        
        
        // Send the packet on its way
        // Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0
        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  8                       // SRIO take 8 bytes Ps Info
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        //mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        sbco s_curFwd.queue,            cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        //xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        ret

l_c1CurPktForward5:
    qbne  l_c1CurPktForward6, s_curFwd.forwardType, PA_FORWARD_TYPE_ETH
        // Routing to ETH
        // Clear psflags
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        and  r0.b0, s_curFwd_eth.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
        or   r2.b0, r2.b0, r0.b0
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
        // Set EMAC output port
        and  r0.b0, s_curFwd_eth.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
        sbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
        // Clear TimeSync word
        mov  r0, 0
        sbco r0, cCdeHeldPkt, OFFSET(s_pktDescrCpsw.tsWord), 4
        
        // Send the packet on its way
        add  s_pktExtDescr.threadId,  s_curFwd_eth.priority, THREADID_ETHERNET1
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8)
        mov  s_cdeCmdPkt.psInfoSize,  0
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
        //xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_c1CurPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret


l_c1CurPktForward6:
    qbeq  l_c1CurPktForward7,  s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD

        // Invalid match type in table - this should never happen

l_c1CurPktForward7: 
        // Do a silent discard, release the packet
        mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_DISCARD
        
        //   Free the packet Ext Info
        set  s_pktExtDescr.flags.fDroppedInd
        
        //mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_ADVANCE  (clear all other bytes)
        ldi     r4, CDE_CMD_PACKET_ADVANCE
        //xout  XID_CDECTRL,         s_cdeCmdPkt,                 SIZE(s_cdeCmdPkt)
        qbeq  l_c1CurPktForward9,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret
        
l_c1CurPktForward8:     
        // For command set, the command ID and the command set index
        // must be patched into the packet. 

        lbco s_pktCxt,                 cCdeOutPkt,                 SIZE(s_pktDescr), 8
        // Note: It can be skipped since the original commad is 0 
        //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
        or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,    PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
        set  s_pktCxt.flags.t_flag_cmdset
        qbne l_c1CurPktForward8_0, r2.b2, THREADID_CDMA0
            // during QoS the destination would have been modified to CDMA0, indicate it in reserved bit 14, 
            // for egress PDSPs to handle when it gets from QoS.
            set  s_pktCxt.flags.t_flag_eg_cmdSet        
l_c1CurPktForward8_0:        
        sbco s_pktCxt,                 cCdeOutPkt,          SIZE(s_pktDescr),  OFFSET(s_pktCxt.startOffset)
        sbco s_curFwdRxCmdSet.index,   cCdeOutPkt,          SIZE(s_pktDescr) + OFFSET(s_pktCxt4.cmdSetIdx),    SIZE(s_pktCxt4.cmdSetIdx) 
        sbco s_pktExtDescr.threadId,   cCdeOutPkt,          SIZE(s_pktDescr) + OFFSET(s_pktCxt4.threadIdOrig), SIZE(s_pktCxt4.threadIdOrig) 
        
        // patch the flow Id and queue Id
        sbco s_curFwd.flowId, cCdeOutPkt,            OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
        sbco s_curFwd.queue,  cCdeOutPkt,            OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)
        
        // Send the packet on its way
        // Free the packet Ext Info
        mov  s_pktExtDescr.threadId, r2.b2
        //xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
        
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        //mov  s_cdeCmdPkt.threadId,    PA_DEST_PA_M_0
        //xout XID_CDECTRL,             s_cdeCmdPkt,                  SIZE(s_cdeCmdPkt)
        qbeq l_c1CurPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        ret
        
        
l_c1CurPktForward9:   
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

    .leave currentFwdScope
    .leave pktScope
    .leave cdeScope
    .leave usrStatsFifoScope

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
// *   R6:         | packet capture info (L2 PDSP only)                                                            -
// *   R7:         |                                                            -
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
    
#ifndef PASS_PROC_FIREWALL
    

f_c1ForwardHeldPacketNoMatch:

    // Inc the no match stat
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LUT1_NO_MATCH
    
#ifdef PASS_PROC_DEF_ROUTE   

    qbbc   l_c1ForwardHeldPacketNoMatch_no_def_r, s_runCxt.flag2.t_def_route    

        // Reload the packet context  (already loaded)
        // lbco   s_pktCxt.paCmdId_Length,   cCdeHeldPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

        // default route is enabled globally
        // Update r1 with offset for interface (get to zero base)
        lsr   r1.w0, s_pktCxt.eId_portNum_nextHdr, PKT_EMACPORT_SHIFT
	    and   r1.w0, r1.w0, PKT_EMACPORT_MASK
         
        qbeq l_c1ForwardHeldPacketNoMatch_no_def_r, r1.w0, 0
        mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE      
      
        sub   r1.w0, r1.w0, 1
        lsl   r1.w0, r1.w0, 6
        add   r1.w0, r1.w0, r1.w2
      
        // load the default control bit map for post-classification no match packets
        lbco  r2.b0, PAMEM_CONST_PORTCFG, r1.w0, 1   
        
l_c1ForwardHeldPacketNoMatch_DFR_MC:        
        //Is it a multicast packet
        qbbc   l_c1ForwardHeldPacketNoMatch_DFR_BC, s_pktCxt.flags.t_flag_multicast 
            //multicast packet
            qbbc    l_c1ForwardHeldPacketNoMatch_BM_MC, r2.b0.t_default_route_mc_enable
                add r1.w0, r1.w0, 4
                jmp l_c1ForwardHeldPacketNoMatch_DFR_end
        
l_c1ForwardHeldPacketNoMatch_DFR_BC:
        //Is it a broadcast packet
        qbbc   l_c1ForwardHeldPacketNoMatch_DFR_UC, s_pktCxt.flags.t_flag_broadcast 
            //broadcast packet
            qbbc    l_c1ForwardHeldPacketNoMatch_BM_BC, r2.b0.t_default_route_bc_enable
                add r1.w0, r1.w0, 4 + SIZE(s_fmPlace)
                jmp l_c1ForwardHeldPacketNoMatch_DFR_end
        
l_c1ForwardHeldPacketNoMatch_DFR_UC:
            //Unicast packet
            qbbc    l_c1ForwardHeldPacketNoMatch_BM_4, r2.b0.t_default_route_uc_enable
                add r1.w0, r1.w0, 4 + SIZE(s_fmPlace)*2
        
             // pass through
l_c1ForwardHeldPacketNoMatch_DFR_end:
        lbco   s_fmPlace,         PAMEM_CONST_PORTCFG,  r1.w0,  SIZE(s_fmPlace)        
        jmp    f_stdHeldPktForward 
      
l_c1ForwardHeldPacketNoMatch_no_def_r:

#endif   

    // Read from the packet the previous match information (already loaded)
    // lbco   s_pktCxt.paCmdId_Length,   cCdeHeldPkt,  SIZE(s_pktDescr),  OFFSET(s_pktCxt.hdrBitmask)
    
    // Broadcast and multicast check if no match found 
    // Assume that this rule precedes the pre-match rule
l_c1ForwardHeldPacketNoMatch_BM_1:
    qbbc   l_c1ForwardHeldPacketNoMatch_BM_2, s_pktCxt.flags.t_flag_multicast 
l_c1ForwardHeldPacketNoMatch_BM_MC:
        mov    r1.w0, EROUTE_IP_MULTICAST*SIZE(s_fmPlace)
        jmp    l_c1ForwardHeldPacketNoMatch_BM_3  
        
l_c1ForwardHeldPacketNoMatch_BM_2:    
    qbbc   l_c1ForwardHeldPacketNoMatch_BM_4, s_pktCxt.flags.t_flag_broadcast 
l_c1ForwardHeldPacketNoMatch_BM_BC:
        mov    r1.w0, EROUTE_IP_BROADCAST*SIZE(s_fmPlace)
        //pass through
    
l_c1ForwardHeldPacketNoMatch_BM_3:
        // Load the routing information for broadcast/multicast type into the 
        // matchforward context
        lbco  s_fmPlace,         PAMEM_CONST_EROUTE,  r1.w0,  SIZE(s_fmPlace)
        qbeq  l_c1ForwardHeldPacketNoMatch_BM_4, s_matchForward.forwardType, PA_FORWARD_TYPE_DISCARD
        jmp   f_stdHeldPktForward
    
l_c1ForwardHeldPacketNoMatch_BM_4:

    qbbs  l_c1ForwardHeldPacketNoMatch0, s_pktCxt.flags.t_flag_pl1Match

        // For no previous match set the error type in the packet, load the 
        // error routing information and route
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_LUT1_FAIL << PKT_EIDX_SHIFT
        sbco s_pktCxt.eId_portNum_nextHdr,     cCdeHeldPkt,         SIZE(s_pktDescr)+OFFSET(s_pktCxt.eId_portNum_nextHdr),  2

        // Load the routing information for LUT1 fail error type into the 
        // matchforward context
        lbco  s_fmPlace,            PAMEM_CONST_EROUTE,  EROUTE_LUT1_FAIL << 4,  SIZE(s_fmPlace)
        jmp   f_stdHeldPktForward

l_c1ForwardHeldPacketNoMatch0:

        // In the case of a previous match, the previous match routing information
        // must be loaded into the matchForward context
        lsr  r1.b2,  s_pktCxt.phyLink,   PKT_PDSP_ID_SHIFT     // Get PDSP ID 
        mov  r1.w0,  s_pktCxt.phyLink
        and  r1.b1,  r1.b1,     PKT_L1IDX_MASK
        lsl  r1.w0,  r1.w0,     6                              // get the associated table offset

        lsl  r1.b2,  r1.b2,     2  // PDSP ID * 4 = offset to the nextFail table base address
        lbco r2,     PAMEM_CONST_NFAIL_BASE, r1.b2,     4
        
        //Load the nextfail routing information
        lbbo  s_fmPlace,        r2,  r1.w0,  SIZE(s_fmPlace)
        jmp   f_stdHeldPktForward

#endif

    .leave cdeScope
    .leave pktScope
    .leave lut1MatchScope
    
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
// *   R6:        |                                | packet capture info (L2 PDSP only)
// *   R7:        |                                |
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
    .using  pktCaptureScope

f_c1Parse:

    lbco  r1, cMailbox, 0, 4
    add   r1, r1, 1
    sbco  r1, cMailbox, 0, 4

    // Read the descriptor
    xin  XID_CDEDATA,  s_pktDescr,   SIZE(s_pktDescr)
    
    // Read packet extended info
    xin  XID_PINFO_SRC, s_pktExtDescr, SIZE(s_pktExtDescr)

    // Check drop/bypass flag
    qbbc l_c1Parse1,  s_pktDescr.pktFlags.t_pktBypass
    
#ifdef PASS_PROC_EOAM
    // Check pkt capture flag
    qbbs l_c1Parse1_0,  s_pktDescr.pktFlags.t_pktCapture
    // check if pkt from ethernet
    // Honor the bypass flag if packet is not from ethernet
    qbbc  l_c1Parse1_0,   s_pktDescr.pktFlags.t_pktEthernet
    // ignore the bypass flag when EOAM feature is enabled
    qbbc l_c1Parse1_0, s_runCxt.flag3.t_eoamEn
    
l_c1Parse1_eoam:    
        clr   s_pktDescr.pktFlags.t_pktBypass

        // mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_N_PKTS
        
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

        // below check is not needed since the packet is not from Ethernet OR SRIO
        // qbeq  l_c1Parse6,   s_pktDescr.ctrlDataSize,    0   // Packet is from ethernet
        // qbne  l_c1Parse3a,  s_pktDescr.ctrlDataSize,    8   // Only SRIO packet contains 8-byte PS Info
            // Extract pkt type
            // lsr   r1.b0,        s_pktDescr.pktType_pvtFlags,  PA_PKT_TYPE_SHIFT 
            // qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_9    
            // qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_11    

    // l_c1Parse3a:
        // l_c1Parse3a - l_c1Parse5: command and re-entry packet
        // Read in the whole packet context. This is the largest command size
        // There is no cost if this is not a data packet. 
        xin  XID_CDEDATA,        s_pktCxt,          SIZE(s_pktCxt)

        // extract the command ID 
        lsr  r1.b0,  s_pktCxt.paCmdId_Length, SUBS_CMD_WORD_ID_SHIFT

        // qbeq  l_c1Parse9,         r1.b0,   PSH_CMD_PA_RX_PARSE
        qbeq  f_paConfigure,      r1.b0,   PSH_CMD_CONFIG
        
    // l_c1Parse9:
        // Delete the control information. It will be replace with the packet context during parse
        //mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
        //xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
        
        // Delete only the pktCxt from the control info
        mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
        //mov  s_cdeCmdWd.byteCount,  (SIZE(s_pktCxt) + 7) & 0xf8     // Round up to multiple of 8 bytes
        sub  s_cdeCmdWd.byteCount,  s_pktDescr.ctrlDataSize,    16
        xout XID_CDECTRL,           s_cdeCmdWd,                 4
         
        mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
        xout  XID_CDECTRL,           s_cdeCmdWd,                4

        // save  Extended Header to be examined later
        xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    // l_c1Parse10:
       // Default RA setting 
       // TBD: priority settings
      zero &s_raInfo,   SIZE(s_raInfo)
      qbbc l_c1Parse0_set_ra_threadId_1, s_runCxt.flag2.t_raToQueue
        // RA output is host queue
        mov  s_raInfo.flags, THREADID_CDMA0
        jmp  l_c1Parse0_set_ra_threadId_end
l_c1Parse0_set_ra_threadId_1:  
        mov  s_raInfo.flags, PASS_RA_DEST_THREAD_ID 
        // pass through  
l_c1Parse0_set_ra_threadId_end:
      wbs   s_flags.info.tStatus_CDEOutPacket
      sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)

      // s_l1View1(16), s_l1View2 (16) and s_l1View3 (16) share the same area
      // Clear the LUT1 view areas
      // Note: packet descriptor will still reside at r6-r13
        zero &s_l1ViewD,     SIZE(s_l1ViewD)
        xout  XID_LUT1V1,     s_l1ViewD,         SIZE(s_l1ViewD)
        xout  XID_LUT1V2,     s_l1ViewD,         SIZE(s_l1ViewD)
        xout  XID_LUT1V3,     s_l1ViewD,         SIZE(s_l1ViewD)

        jmp  f_c1ParseMac    
#endif 

l_c1Parse1_0:    
        // Clear Bypass at the last PDSP
        #ifdef PASS_LAST_PDSP
        clr s_pktDescr.pktFlags.t_pktBypass
        xout  XID_CDEDATA,         s_pktDescr.pktFlags,                    SIZE(s_pktDescr.pktFlags)
        #endif
        
        // TBD: Move to the end of packet
        mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_END
        xout  XID_CDECTRL,           s_cdeCmdWd,                  SIZE(s_cdeCmdWd)
        
        mov   s_cdeCmdWd.operation,   CDE_CMD_PACKET_ADVANCE
        mov   s_cdeCmdPkt.optionsFlag, 0
        
        set   s_runCxt.flags.t_previous_skip
        jmp   l_c1Parse14_1

l_c1Parse1:

#ifdef PASS_PROC_L2_CAPTURE
    // Note: pktCapture and pktPypass are exclsuive to each other
    qbbc l_c1Parse2,  s_pktDescr.pktFlags.t_pktCapture
        // Clear falg Capture
        clr s_pktDescr.pktFlags.t_pktCapture
        xout  XID_CDEDATA,         s_pktDescr.pktFlags,                    SIZE(s_pktDescr.pktFlags)

    // Jump to copy initiate, when there is no active look up
    qbbc  l_c1Parse1_copy, s_runCxt.flags.t_previous_held
        // The extended header must be in order. To handle this case, we need to store the extended header at
        // scratch memory and set a flag       
        // There is an active lookup, check whether we have room to copy packet
        // The formula to compute the room is 
        // (Previous CDE packet Size + current CDE Pkt Size * 2 ) < 2300
        set    s_runCxt.flags.t_extHdr_held
        add    r1.w2,  s_pktCxt.pseudo, s_pktDescr.pktDataSize
        add    r1.w2,  r1.w2, s_pktDescr.pktDataSize
        mov    r1.w0,  2300
    // Jump to copy operation if there is a space in output buffer
    qbgt   l_c1Parse1_copy, r1.w2,  r1.w0
        clr    s_runCxt.flags.t_extHdr_held
        // LUT1 is very busy holding up the space, so wait and free output space before copy
        // save  Extended Header to be examined later
        xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
      
        wbc  s_flags.info.tStatus_Lut1Busy
      
        // A lookup is complete for the held packet
        xin   XID_LUT1CMD,              s_l1Status,                              SIZE(s_l1Status)
      
        // Common operation regredless of the lookup result
        // Pop the next extended info and write it out
        xin   XID_PINFO_B,   s_pktExtDescr,  SIZE(s_pktExtDescr)
     
        // Wait for the held packet to appear in the CDE
        // (Its almost certainly already there)
        wbs   s_flags.info.tStatus_CDEHeldPacket
        wbc   s_flags.info.tStatus_CDEBusy
     
        // Load the first 20-byte to pktInfo
        // The first 5 32-bit word in the pkt context may be examized and patched
        lbco  s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    20
      
        qbbc  l_c1Parse1_NoMatchRelease,          s_l1Status.status.L1_STATUS_ENTRY_MATCH

          qbbc  l_c1Parse1_FWallMatchRelease, s_runCxt.flag3.t_eoamEn                     
            call  f_c1ForwardHeldEoamPktMatch
            jmp   l_c1Parse1_clr_activeLookup
l_c1Parse1_FWallMatchRelease:                            
          call  f_c1ForwardHeldFirewallPktMatch
          jmp   l_c1Parse1_clr_activeLookup
l_c1Parse1_NoMatchRelease:   

          qbbc  l_c1Parse1_FWallNoMatchRelease, s_runCxt.flag3.t_eoamEn                     
            call  f_c1ForwardHeldEoamPktNoMatch
            jmp   l_c1Parse1_clr_activeLookup
l_c1Parse1_FWallNoMatchRelease:            
          call  f_c1ForwardHeldFirewallPktNoMatch
     
l_c1Parse1_clr_activeLookup:     
        clr  s_runCxt.flags.t_previous_held
      
        // restore extended packet header
        xin  XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
      
        // pass through
     
l_c1Parse1_copy: 
        // save the current threadId for EOAM (needed if EOAM mode is enabled)
        mov    r0.b0, s_pktExtDescr.threadId
        // Packet Capture activity
        mov  s_pktExtDescr.threadId,   PA_DEST_CDMA
        //mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY      
        //mov  s_cdeCmdPkt.optionsFlag,  0   
        ldi  r4, CDE_CMD_PACKET_COPY     
        xout XID_CDECTRL,              s_cdeCmdPkt,             4
      
        clr  s_pktExtDescr.flags.fFinal 
      
        // The extended header must be in order. To handle this case, we need to store the extended header at
        // scratch memory and set a flag       
        qbbs l_c1Parse1_copy_held_extHdr, s_runCxt.flags.t_extHdr_held
            // use the fFinal bit as an indication
            xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr) 
            jmp  l_c1Parse1_copy_wait_new_packet
      
l_c1Parse1_copy_held_extHdr:
        sbco  s_pktExtDescr,  PAMEM_CONST_PDSP_CXT,  OFFSET_EXT_HDR_PENDING, SIZE(s_pktExtDescr)
        // pass through

l_c1Parse1_copy_wait_new_packet:
        // Restore the threadID before EOAM
        mov    s_pktExtDescr.threadId, r0.b0
        
        set  s_pktExtDescr.flags.fFinal 

        // Wait for new packet to be ready
        wbs   s_flags.info.tStatus_CDENewPacket
     
        // Read the packet descriptor
        xin  XID_CDEDATA,   s_pktDescr,      SIZE(s_pktDescr)       
#endif

l_c1Parse2:     

#ifdef PASS_PROC_EOAM
        qbbc  l_c1Parse2_0, s_pktDescr.pktFlags.t_pktEthernet
        qbbc  l_c1Parse2_0, s_runCxt.flag3.t_eoamEn
          jmp l_c1Parse1_eoam
#endif

l_c1Parse2_0:

    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_N_PKTS

#ifdef PASS_TIMESTAMP_OP
  qbbc l_c1Parse2_no_eoam, s_runCxt.flag3.t_eoamEn    
    // Add timestamp to all the packets with new packet ID including the command packet
    mov32 r1,    PDSP0_TIMER    
    lbbo  r0,    r1, 8, 4
    mov   r0.w2, 0xffff
    sub   s_pktCxt7.timeStamp_lo.w0,  r0.w2,  r0.w0 
    
    lbco  r0, PAMEM_CONST_CUSTOM,  OFFSET_SYS_TIMESTAMP,       8
    clr   r5.b0.t0
    qblt  l_c1Parse2_timestamp_loaded, s_pktCxt7.timeStamp_lo.w0, 100
    qbbc  l_c1Parse2_timestamp_loaded, s_flags.info.tStatus_Timer
        set   s_flags.info.tStatus_Timer // set to clear timer event 
        add   r0, r0, 1
        adc   r1, r1, 0
        sbco  r0, PAMEM_CONST_CUSTOM,  OFFSET_SYS_TIMESTAMP,   8
        set   r5.b0.t0

l_c1Parse2_timestamp_loaded:
    mov   s_pktCxt7.timeStamp_lo.w2,  r0.w0
    // prepare high 4 bytes
    mov   s_pktCxt7.timeStamp_hi.w0,  r0.w2
    mov   s_pktCxt7.timeStamp_hi.w2,  r1.w0
    
    qbbc  l_c1Parse2_no_eoam, r5.b0.t0
        call  f_eoamTimeConvert

l_c1Parse2_no_eoam:
    // EOAM Time Convert uses r4, r5, it is okay as long as
    // the code following it updates r4 and r5 accordingly
    mov   s_cdeCmdIn2.data, s_pktDescr.swinfo0
#endif

l_c1Parse3:

    // The context is read from the control section. Advance
    // to the control section
    //mov s_cdeInsert.operation,  CDE_CMD_ADVANCE_TO_CONTROL    
    ldi   r4,                    CDE_CMD_ADVANCE_TO_CONTROL
    xout  XID_CDECTRL,           s_cdeCmdWd,            4

    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    //mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    //mov s_cdeInsert.byteCount,  32
    ldi  r4,        CDE_CMD_INSERT_PSDATA | (32 << 8)  
    xout XID_CDECTRL,           s_cdeCmdWd,             4

    qbeq  l_c1Parse6,   s_pktDescr.ctrlDataSize,    0   // Packet is from ethernet
    qbne  l_c1Parse3a,  s_pktDescr.ctrlDataSize,    8   // Only SRIO packet contains 8-byte PS Info
        // Extract pkt type
        lsr   r1.b0,        s_pktDescr.pktType_pvtFlags,  PA_PKT_TYPE_SHIFT 
        qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_9    
        qbeq  l_c1Parse8,   r1.b0,  PA_PKT_TYPE_SRIO_TYPE_11    

l_c1Parse3a:

    // l_c1Parse3a - l_c1Parse5: command and re-entry packet
    // Read in the whole packet context. This is the largest command size
    // There is no cost if this is not a data packet. 
    xin  XID_CDEDATA,        s_pktCxt,          SIZE(s_pktCxt)

#ifdef PASS_DETECT_IP_PROC
    qbbc l_c1Parse3a_in3, s_runCxt.flag3.t_eoamEn
      // Clear the IP Processing information
      clr  s_pktCxt.flags.t_flag_ipProc        
l_c1Parse3a_in3:        
#endif

    
#ifdef PASS_VERIFY_SCTP_CRC
    qbbc l_c1Parse3b, s_pktCxt.flags.t_flag_crc_verify
        // SCTP CRC Verification
        call  f_paSctpVerifyCRC
        set   s_runCxt.flags.t_previous_skip
        jmp   l_c1Parse14_1

l_c1Parse3b:

#endif

    // extract the command ID 
    lsr  r1.b0,  s_pktCxt.paCmdId_Length, SUBS_CMD_WORD_ID_SHIFT

    qbeq  l_c1Parse9,         r1.b0,   PSH_CMD_PA_RX_PARSE
    qbeq  f_paConfigure,      r1.b0,   PSH_CMD_CONFIG

l_c1Parse4:
    // The packet has an invalid command
    // Inc the stat
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_INVALID_CONTROL
        
        // pass through
l_c1Parse5:
    // Discard the packet
    set  s_pktExtDescr.flags.fDroppedInd
    
    mov   s_cdeCmdPkt.operation,  CDE_CMD_PACKET_ADVANCE
    mov   s_cdeCmdPkt.optionsFlag, 0
    set   s_runCxt.flags.t_previous_skip
    //xout  XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)

    // Packet should be dropped at the last DSP at this stage
    jmp   l_c1Parse14_1

l_c1Parse6:
    // Packet is from ethernet and has not been seen by the PA before
#ifdef PASS_PROC_L2
    qbbc l_c1Parse5_0, s_runCxt.flag3.t_eoamEn
      set  s_pktDescr.pktFlags.t_pktEthernet
       wbs   s_flags.info.tStatus_CDEOutPacket      
       sbco s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)      
l_c1Parse5_0:      
#endif

#ifdef PASS_PROC_FIREWALL
   // Firewall does not expect MAC packet, drop the packet
   jmp l_c1Parse5 
#endif    

#ifdef PASS_PROC_INGRESS_PKT_CLONE
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
    qbbc  l_c1Parse6_copy, s_runCxt.flags.t_previous_held
    // The extended header must be in order. To handle this case, we need to store the extended header at
    // scratch memory and set a flag       
    // There is an active lookup, check whether we have room to copy packet
    // The formula to compute the room is 
    // (Previous CDE packet Size + current CDE Pkt Size * 2 ) < 2300
    set    s_runCxt.flags.t_extHdr_held
    add    r1.w2,  s_pktCxt.pseudo, s_pktDescr.pktDataSize
    add    r1.w2,  r1.w2, s_pktDescr.pktDataSize
    mov    r1.w0,  2300
    // Jump to copy operation if there is a space in output buffer
    qbgt   l_c1Parse6_copy, r1.w2,  r1.w0
      clr    s_runCxt.flags.t_extHdr_held
      // LUT1 is very busy holding up the space, so wait and free output space before copy
      // save  Extended Header to be examined later
      xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
      
      wbc  s_flags.info.tStatus_Lut1Busy
      
      // A lookup is complete for the held packet
      xin   XID_LUT1CMD,              s_l1Status,                              SIZE(s_l1Status)
      
      // Common operation regredless of the lookup result
      // Pop the next extended info and write it out
      xin   XID_PINFO_B,   s_pktExtDescr,  SIZE(s_pktExtDescr)
     
      // Wait for the held packet to appear in the CDE
      // (Its almost certainly already there)
      wbs   s_flags.info.tStatus_CDEHeldPacket
      wbc   s_flags.info.tStatus_CDEBusy
     
      // Load the first 20-byte to pktInfo
      // The first 5 32-bit word in the pkt context may be examized and patched
      lbco  s_pktCxt, cCdeHeldPkt,    SIZE(s_pktDescr),    20
      
      qbbc  l_c1Parse6_NoMatchRelease,          s_l1Status.status.L1_STATUS_ENTRY_MATCH
        call  f_c1ForwardHeldPacketMatch
        jmp   l_c1Parse6_clr_activeLookup
l_c1Parse6_NoMatchRelease:   
        call  f_c1ForwardHeldPacketNoMatch
     
l_c1Parse6_clr_activeLookup:     
      clr  s_runCxt.flags.t_previous_held
      
      // restore extended packet header
      xin  XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
      
      // pass through
     
l_c1Parse6_copy:    
      wbs   s_flags.info.tStatus_CDEOutPacket  
      // Issue Copy command (either to Host or Destination)
      // Check is the copy to be done to host?
      // Made sure if the packet capture scratch is intact
      zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)   
      qbbs  l_c1Parse6_copy_host, s_paPktCapScr.ctrlBitMap.t_pkt_cap_host  
        
      // Port mirror activity
      // Routing to ETH
      // Patch the output port
      sbco s_paPktCapScr.mport_flow, cCdeOutPkt, OFFSET(s_pktDescrCpsw.outPort), 1
      
      // Send the packet to ethernet mirror port 
      mov  s_pktExtDescr.threadId,    PA_DEST_ETH
      // Jump to send packet and normal operation 
      
      jmp  l_c1Parse6_copy_send
    
l_c1Parse6_copy_host:
l_c1Parse6_copy_host_queue_bounce:
        // Check for Queue Bounce operation
l_c1Parse6_copy_host_queue_bounce_ddr:
        qbbc l_c1Parse6_copy_host_queue_bounce_msmc, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1Parse6_copy_host_queue_bounce_end

l_c1Parse6_copy_host_queue_bounce_msmc:
        qbbc l_c1Parse6_copy_host_queue_bounce_end, s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            clr s_paPktCapScr.destQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paPktCapScr.destQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paPktCapScr.destQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through
l_c1Parse6_copy_host_queue_bounce_end:

      // Packet Capture activity
      mov  s_pktExtDescr.threadId,   PA_DEST_CDMA
      mov  s_cdeCmdPkt.destQueue,    s_paPktCapScr.destQueue
      mov  s_cdeCmdPkt.flowId,       s_paPktCapScr.mport_flow  
      sbco  s_paPktCapScr.context,   cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0), SIZE(s_paPktCapScr.context) 
      
l_c1Parse6_copy_send:
      mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY      
      mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE)     
      //mov  s_cdeCmdPkt.psInfoSize,  0
      xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
      
      
#ifndef PASS_LAST_PDSP
      lbco  r0.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
      set   r0.w0.t_pktBypass 
      set   r0.w0.t_pktCapture
      sbco  r0.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#endif
   
      clr  s_pktExtDescr.flags.fFinal 
      
     // The extended header must be in order. To handle this case, we need to store the extended header at
     // scratch memory and set a flag       
      qbbs l_c1Parse6_copy_held_extHdr, s_runCxt.flags.t_extHdr_held
        // use the fFinal bit as an indication
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr) 
        jmp  l_c1Parse6_copy_wait_new_packet
      
l_c1Parse6_copy_held_extHdr:
        sbco  s_pktExtDescr,  PAMEM_CONST_PDSP_CXT,  OFFSET_EXT_HDR_PENDING, SIZE(s_pktExtDescr)
        // pass through

l_c1Parse6_copy_wait_new_packet:
      set  s_pktExtDescr.flags.fFinal 

      // Wait for new packet to be ready
      wbs   s_flags.info.tStatus_CDENewPacket
     
      // Read the packet descriptor
      xin  XID_CDEDATA,   s_pktDescr,      SIZE(s_pktDescr)
      
      // Move up to control and insert 32 bytes (40 bytes during EOAM) of PS data      
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
      //restore the upper 32-bit of timestamp
      lbco    &s_cdeCmdIn2.data, PAMEM_CONST_PARSE,  OFFSET_TIMESTAMP_TMP,   4
        
#endif

l_c1Parse6a:    
    // Note: nextHdr = 0 (MAC) startOffset = 0
    // Last 2 bytes already contain the upper 16 - bits of the timestamp

    //Insert the upper 32-bit timestamp as control Info
    //mov s_cdeInsert2.operation,  CDE_CMD_INSERT_CONTROL
    //mov s_cdeInsert2.byteCount,  8
    ldi   r4,   CDE_CMD_INSERT_CONTROL | (8 << 8)   
    xout  XID_CDECTRL,          s_cdeCmdIn2,            SIZE(s_cdeCmdIn2)
    // wait for the sideband data to be ready 
    wbc    s_flags.info.tStatus_CDEBusy            
    
#ifdef PASS_TIMESTAMP_OP
  qbbc l_c1Parse8_0, s_runCxt.flag3.t_eoamEn  
    //Insert the upper 32-bit PA timestamp as control Info
    //mov s_cdeInsert2.operation,  CDE_CMD_INSERT_CONTROL
    //mov s_cdeInsert2.byteCount,  8
    ldi   r4,   CDE_CMD_INSERT_CONTROL | (4 << 8)  

    mov   r5,   s_pktCxt7.timeStamp_hi
    xout  XID_CDECTRL,          s_cdeCmdIn2,            SIZE(s_cdeCmdIn2)
    // wait for the sideband data to be ready 
    wbc    s_flags.info.tStatus_CDEBusy       

    mov   r5,   s_pktCxt7.timeStamp_lo
    xout  XID_CDECTRL,          s_cdeCmdIn2,            SIZE(s_cdeCmdIn2)
    // wait for the sideband data to be ready 
    wbc    s_flags.info.tStatus_CDEBusy   
     
l_c1Parse8_0:  
#endif

    //zero  &s_pktCxt,            OFFSET(s_pktCxt.timestamp)
    zero  &s_pktCxt,            SIZE(s_pktCxt)
    add    s_pktCxt.endOffset,  s_pktDescr.pktDataSize, s_pktExtDescr.mopLength
#ifdef PASS_PROC_L2
    mov    s_pktCxt.pseudo,     s_pktDescr.pktDataSize
#endif     
#ifdef PASS_PROC_L2_CAPTURE
    mov    s_pktCxt.pseudo,     s_pktDescr.pktDataSize
#endif     
    jmp    l_c1Parse9a

l_c1Parse8:
    // Packet is from SRIO with a control data size of 8 bytes
    // which means this is the first time the PA is seeing it.
    zero  &s_pktCxt,          SIZE(s_pktCxt)
    add    s_pktCxt.endOffset,  s_pktDescr.pktDataSize, s_pktExtDescr.mopLength
    // save  Extended Header to be examined later
    xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
    mov s_pktCxt.eId_portNum_nextHdr,    PA_HDR_UNKNOWN  

    // Continue to parse the SRIO packet normally  
    mov  r30.w0,         l_c1Parse12             // return to l_c1Parse12 after call to f_paSrio 
    jmp  f_c1ParseSrio   

l_c1Parse9:
    // Delete the control information. It will be replace with the packet context during parse
    //mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    //xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    
    // Delete only the pktCxt from the control info
    mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH
    //mov  s_cdeCmdWd.byteCount,  (SIZE(s_pktCxt) + 7) & 0xf8     // Round up to multiple of 8 bytes
    sub  s_cdeCmdWd.byteCount,  s_pktDescr.ctrlDataSize,    8
    qbbc l_c1Parse9a_0, s_runCxt.flag3.t_eoamEn     
      // reserve another 8 bytes for PA timestamp
      sub  s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,    8       
l_c1Parse9a_0:    
    xout XID_CDECTRL,           s_cdeCmdWd,                 4
     
    mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET 
    xout  XID_CDECTRL,           s_cdeCmdWd,                4

l_c1Parse9a:
    // advance the CDE to the first byte to examine in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_pktCxt.startOffset
    xout XID_CDECTRL,           s_cdeCmdWd,                 SIZE(s_cdeCmdWd)
    
    // save  Extended Header to be examined later
    xout   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    // next header is stored in 8 bit format during the parse
    and s_next.Hdr,  s_pktCxt.eId_portNum_nextHdr.b0,  0x3f

    mov s_param.action,     SUBS_ACTION_PARSE

#ifndef PASS_PROC_FIREWALL    
    // non-Firewall operation
    
    // Clear any values in the error index only if not custom
    qbeq l_c1Parse10, s_next.Hdr, PA_HDR_CUSTOM_C1 
    qbeq l_c1Parse9a_1, s_next.Hdr, PA_HDR_CUSTOM_C2
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  NOT_PKT_EIDX_MASK

    // Make sure the parse entry point is valid for classify1
    // PA_HDR_UNKNOWN and others should use nextFail route
    // Note: This is not a parsing error exception
    // TBD: check may not be good enough now
    qbgt l_c1Parse10,  s_next.Hdr,  PA_HDR_UNKNOWN
 l_c1Parse9a_1:    
        #ifndef PASS_L4_BYPASS
            #ifdef PASS_PROC_IPSEC_NAT_T
            // Ingress1, PDSP1: Perform IPSEC ESP NAT-T detection
            // Non-UDP and normal UDP packets should be forwarded to Ingress4 for L4 classification  
            qbeq l_c1Parse10,  s_next.Hdr,  PA_HDR_UDP
                // Forward Non-UDP packets to Ingess4 for further classification
                mov s_param.action,  SUBS_ACTION_FWPKT3
                jmp l_c1Parse14
            #else
            // The next header is not L3 header, use nextFail route 
            set s_runCxt.flags.t_failLookup
            mov s_param.action,         SUBS_ACTION_EXIT
            jmp l_c1Parse16 
            #endif     
        #else
        // L4 BYPASS: Ingess4, PDSP0: Forward L4 packets to PDSP1 for further classification
        qbne l_c1Parse9b,   s_next.Hdr, PA_HDR_UNKNOWN
            set s_runCxt.flags.t_failLookup
            mov s_param.action,         SUBS_ACTION_EXIT
            jmp l_c1Parse16      
l_c1Parse9b:
            set  s_runCxt.flags.t_previous_skip
            wbs  s_flags.info.tStatus_CDEOutPacket
            mov  s_pktCxt.paCmdId_Length, (SIZE(s_pktCxt) + 7) & 0xf8      // round up to a multiple of 8 bytes
            sbco s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            // The bypass flag should not be set since this packet should be processed by the next PDSP for L4 lookup
            jmp  l_c1Parse14_2
        #endif
        
#else
    // Firewall operation
    // r28.b0: store the original startoffset
    // Firewall does not handle the following protocols
    qbeq l_c1Parse9b, s_next.Hdr, PA_HDR_CUSTOM_C1
    qbeq l_c1Parse9b, s_next.Hdr, PA_HDR_CUSTOM_C2
    qble l_c1Parse9b, s_next.Hdr, PA_HDR_TCP
        // Firewall parsing preparation
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  NOT_PKT_EIDX_MASK
        mov  r28.b0, s_pktCxt.startOffset
        set  s_runCxt.flags.t_l4Avil
        jmp  l_c1Parse10
        
l_c1Parse9b:        
        set s_runCxt.flags.t_failLookup
        mov s_param.action,         SUBS_ACTION_EXIT
        wbs  s_flags.info.tStatus_CDEOutPacket
        mov  s_pktCxt.paCmdId_Length, (SIZE(s_pktCxt) + 7) & 0xf8      // round up to a multiple of 8 bytes
        sbco s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)
        jmp l_c1Parse16      
#endif        

l_c1Parse10:
#ifdef PASS_PROC_FIREWALL   
   // Default RA setting 
   // TBD: priority settings
  zero &s_raInfo,   SIZE(s_raInfo)
  qbbc l_c1Parse10_set_ra_threadId_1, s_runCxt.flag2.t_raToQueue
    // RA output is host queue
    mov  s_raInfo.flags, THREADID_CDMA0
    jmp  l_c1Parse10_set_ra_threadId_end
l_c1Parse10_set_ra_threadId_1:  
    mov  s_raInfo.flags, PASS_RA_DEST_THREAD_ID 
    // pass through  
l_c1Parse10_set_ra_threadId_end:
#ifdef PASS_INNER_RA
      set s_raInfo.flags.t_ra_flag_2nd_inst   
#endif         
  wbs   s_flags.info.tStatus_CDEOutPacket
  sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
#endif        

    // s_l1View1(16), s_l1View2 (16) and s_l1View3 (16) share the same area
    // Clear the LUT1 view areas
    // Note: packet descriptor will still reside at r6-r13
    zero &s_l1ViewD,     SIZE(s_l1ViewD)
    xout  XID_LUT1V1,     s_l1ViewD,         SIZE(s_l1ViewD)
    xout  XID_LUT1V2,     s_l1ViewD,         SIZE(s_l1ViewD)
    xout  XID_LUT1V3,     s_l1ViewD,         SIZE(s_l1ViewD)
    set   s_runCxt.flags.t_firstLookup
    jmp   l_c1Parse11_1
l_c1Parse11:
    //  The main parse loop
    //
    //  General Error check to avoid infinite loop
    //  r28.b3: previous startOffset
    //
    qbge l_c1Parse11_1,  r28.b3, s_pktCxt.startOffset
        and s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or  s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT
        mov s_param.action,          SUBS_ACTION_EXIT
        jmp l_c1Parse14
     
l_c1Parse11_1:
    //  Util SUBS_ACTION_LOOKUP or SUBS_ACTION_EXIT occur
    mov   r28.b3,         s_pktCxt.startOffset
    lsl   r0.b0,          s_next.Hdr,         1
    lbco  r0.w2,          PAMEM_CONST_PARSE,  r0.b0,              2
    call  r0.w2
    clr   s_runCxt.flags.t_firstLookup
    qbeq  l_c1Parse11,    s_param.action,     SUBS_ACTION_PARSE

l_c1Parse12:
#ifndef PASS_PROC_FIREWALL    
    // Put the next header type back into the context
    and  s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  0xc0
    or   s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  s_next.Hdr
    
#ifdef PASS_PROC_DEF_ROUTE 

    qbbc   l_c1Parse14, s_runCxt.flag2.t_def_route    

        // default route is enabled globally
        // Update r1 with offset for interface (get to zero base)
        lsr   r1.w0, s_pktCxt.eId_portNum_nextHdr, PKT_EMACPORT_SHIFT
        and   r1.w0, r1.w0, PKT_EMACPORT_MASK
        
        qbeq  l_c1Parse14, r1.w0, 0
        mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE 
      
        sub   r1.w0, r1.w0, 1
        lsl   r1.w0, r1.w0, 6
        add   r1.w0, r1.w0, r1.w2
      
        // load the default control bit map 
        lbco  r2.b0, PAMEM_CONST_PORTCFG, r1.w0, 1   
        
l_c1Parse14_DFR_MC:        
        //Is it a multicast packet
        qbbc   l_c1Parse14_DFR_BC, s_pktCxt.flags.t_flag_multicast 
            //multicast packet
            qbbc  l_c1Parse14, r2.b0.t_default_route_mc_enable
            qbbc  l_c1Parse14, r2.b0.t_default_route_mc_pre_classify_enable
                jmp l_c1Parse14_DFR_end
        
l_c1Parse14_DFR_BC:
        //Is it a broadcast packet
        qbbc   l_c1Parse14, s_pktCxt.flags.t_flag_broadcast 
            //broadcast packet
            qbbc  l_c1Parse14, r2.b0.t_default_route_bc_enable
            qbbc  l_c1Parse14, r2.b0.t_default_route_bc_pre_classify_enable
            
l_c1Parse14_DFR_end:            
            mov s_param.action,          SUBS_ACTION_FWPKT5
                
            // pass through

#endif
    
#else
    // Restore the stratOffset
    mov  s_pktCxt.startOffset,   r28.b0 
#endif    
fci_c1Parse14: 
l_c1Parse14:
    // Restore the latest extended Packet Info
    xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
    wbs   s_flags.info.tStatus_CDEOutPacket
    mov   s_pktCxt.paCmdId_Length, (SIZE(s_pktCxt) + 7) & 0xf8      // round up to a multiple of 8 bytes
    sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)

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
        wbc   s_flags.info.tStatus_CDEBusy
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
            // The extended Header of the active lookup packet is at FIFO_B
            xout XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
            or   s_cdeCmdPkt.optionsFlag, s_cdeCmdPkt.optionsFlag, CDE_FLG_HOLD_PACKET
            xout XID_CDECTRL,             s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
            jmp fci_mainLoop6
            
l_c1Parse15_1:
           // Skip packet: Forward or drop the packet 
           clr s_runCxt.flags.t_previous_skip
           clr s_runCxt.flags.t_held_packet
           
#ifdef PASS_LAST_PDSP
           // TBD: clear Bypass Flag
           lbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
           clr   s_pktDescr.pktFlags.t_pktBypass
           sbco  s_pktDescr.pktFlags,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)

           qbbc l_c1Parse15_2,  s_pktExtDescr.flags.fDroppedInd   
                mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_FLUSH
                set  s_pktExtDescr.flags.fDropped
                //clr  s_pktExtDescr.flags.fDroppedInd 
#endif  
           
l_c1Parse15_2:           
           xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
           xout XID_CDECTRL,             s_cdeCmdPkt,             SIZE(s_cdeCmdPkt)
           jmp  fci_mainLoop6          


l_c1Parse16:
        // Handle all cases where lookup is not required
        set  s_runCxt.flags.t_previous_skip
        
        // No lookup is required, see whether we should discard the packet
        qbeq  l_c1Parse18,  s_param.action,  SUBS_ACTION_DISCARD
        
        // No lookup is required, see whether it is next fail route
        qbbc  l_c1Parse17,  s_runCxt.flags.t_failLookup
            clr  s_runCxt.flags.t_failLookup
#ifndef PASS_PROC_FIREWALL        
            jmp  f_nextFailPktForward
#else
            // Should never reach here during EOAM, fire EOAM exceptions
            jmp  f_c1ForwardFirewallPktNoMatch
#endif            
        
l_c1Parse17:
        qbeq  l_c1Parse17_1,  s_param.action,  SUBS_ACTION_FWPKT
        qbeq  l_c1Parse17_2,  s_param.action,  SUBS_ACTION_FWPKT2
        qbeq  l_c1Parse17_3,  s_param.action,  SUBS_ACTION_FWPKT3
        qbeq  l_c1Parse17_4,  s_param.action,  SUBS_ACTION_FWPKT4
#ifdef PASS_PROC_DEF_ROUTE        
        qbeq  f_BcMcCurrentPktForward, s_param.action,  SUBS_ACTION_FWPKT5
#endif        
            // It is an error packet. 
            // Forward the packet based on the exception index, which must be loaded into r30.w2
            // Error Packet Handling 
            lsr  r30.w2,  s_pktCxt.eId_portNum_nextHdr.b1,  PKT_EIDX_SHIFT
            jmp  f_errCurrentPktForward

l_c1Parse17_1:
            // It is a normal packet (IP forwarding) 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_pktExtDescr.threadId,  THREADID_CDMA0
            jmp  l_c1Parse14_1                                   
            
l_c1Parse17_2:
            // It is a normal packet to be processed at the next stage 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_pktExtDescr.threadId,  THREADID_CDMA0
            jmp  l_c1Parse14_2
            
l_c1Parse17_3:
            // It is a L4 packet to be processed at Ingress4 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_pktExtDescr.threadId,  THREADID_INGRESS4
            jmp  l_c1Parse14_1
            
l_c1Parse17_4:
            // It is a L3 packet to be processed at Ingress3 
            // Forward packet now
            ldi  s_cdeCmd.v0.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_pktExtDescr.threadId,  THREADID_INGRESS3
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
    .leave  pktCaptureScope

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
    lsr   r1.w0, s_pktCxt.eId_portNum_nextHdr, PKT_EMACPORT_SHIFT
	and   r1.w0, r1.w0, PKT_EMACPORT_MASK
    mov   r1.w2, OFFSET_DEFAULT_ROUTE_CFG_BASE 
    
    sub   r1.w0, r1.w0, 1
    lsl   r1.w0, r1.w0, 6
    add   r1.w0, r1.w0, r1.w2
    
l_BcMcCurrentPktForward_MC:        
    //Is it a multicast packet
    qbbc   l_BcMcCurrentPktForward_BC, s_pktCxt.flags.t_flag_multicast 
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
    mov r30.w0, fci_c1Parse14_1
    jmp f_c1CurPktForward


    .leave currentFwdScope
    .leave pktScope

#endif    

#ifdef  PASS_PROC_EOAM
// **************************************************************************************************
// * FUNCTION PURPOSE: Forward a packet that had a match/nomatch in the EOAM LUT1 search
// **************************************************************************************************
// * DESCRIPTION:  On entry r5 contains the LUT1 status, which indicated a match for the
// *               held packet, and the match index value. The packet is forwarded as required.
// *    
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    scratch
// *   R2:    scratch
// *   R3:
// *   R4:
// *   R5:    LUT1 Status (s_l1Status) / LUT1 Command (s_l1Cmd)  - lut1Scope
// *   R6:         | packet capture info (L2 PDSP only)
// *   R7:         |
// *   R8:   LUT1 info (s_l1f)                                       -
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
// *   R22:
// *   R23:
// *   R24:
// *   R25:
// *   R26:  
// *   R27:  
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
    .using  usrStatsFifoScope

f_c1ForwardHeldEoamPktMatch:
    lsl   r1.w0,    s_l1Status.index,             6                                     // l1 index * 256 
    // Load the routing info associated with LUT1 
    // This is the l1Info and match routing information
    lbco   &s_l1f,   PAMEM_CONST_PDSP_LUT1_INFO,   r1.w0,   SIZE(s_l1f)+SIZE(s_fmPlace)  // l1Info + routing info    
    // Check whether the packet has Ethertype 0x8902 packet
    qbbc l_stdHeldPktEoamForward2,s_pktCxt6.eoamFlags.t_flag_eType8902    
      // target flow matched and it is 802.1ag packet
      // check if we need to force disable statistics counts
      // Statistics Updates here as per MEG level and Packet Opcodes
      // Do not check if we need to disable the statistics (do it when the version is corrupted)
        qbbs  l_stdHeldPktEoamForward2, s_pktCxt6.eoamFlags.t_flag_invalidVer
          qbgt  l_stdHeldPktMatchEoamForward1_1, s_matchForward_eoam.megLevel, s_pktCxt6.megLevel
          qblt  l_stdHeldPktMatchEoamForward1_0, s_matchForward_eoam.megLevel, s_pktCxt6.megLevel
          qbeq  l_stdHeldPktMatchEoamForward1_1, s_pktCxt6.opCode, PA_EOAM_APS_LINEAR_OPCODE
          qbeq  l_stdHeldPktMatchEoamForward1_1, s_pktCxt6.opCode, PA_EOAM_APS_RING_OPCODE
          qbeq  l_stdHeldPktMatchEoamForward1_1, s_pktCxt6.opCode, PA_EOAM_CCM_OPCODE

l_stdHeldPktMatchEoamForward1_0:
          // force disable the statistics
          set  s_pktCxt6.eoamFlags.t_flag_fDisableCnt 
l_stdHeldPktMatchEoamForward1_1:          
          qbbc l_stdHeldPktEoamForward2, s_pktCxt6.eoamFlags.t_flag_knownOpcodes
            // Known EOAM control packet, forward to queue            
            // Normal host route: no special operation is required
            // Send the packet on its way            
            //   Free the packet Ext Info
            mov     s_pktExtDescr.threadId, THREADID_CDMA0
            ldi     r1.w0,  CDE_FLG_SET_FLOWID << 8

            // Patch swinfo0 with the context
            sbco s_matchForward_eoam.context,  cCdeHeldPkt, OFFSET(s_pktDescr.swinfo0),  SIZE(s_pktDescr.swinfo0)
            
            // patch the flow Id and queue Id
            sbco s_matchForward.flowId,        cCdeHeldPkt, OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
            sbco s_matchForward.queue,         cCdeHeldPkt, OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)

            // Clear packet drop indicator if any (Override previous stage)
            clr  s_pktExtDescr.flags.fDroppedInd
            
            // Unknown EOAM type found OR non EOAM packet      
l_stdHeldPktEoamForward2:
          // Honor the previous stage (MAC) decision if it is not EOAM control packet meant to send to host
          qbbs l_stdHeldEoamPktDiscard,   s_pktExtDescr.flags.fDroppedInd

          ldi r4.w0, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8)
          or  r4.w0, r4.w0, r1.w0 
          mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes            
          
         // Check if we need to record the statistics
         qbbs  l_stdHeldPktEoamForward3_0, s_pktCxt6.eoamFlags.t_flag_fDisableCnt
           // we need record
           // User Statistics operation:
           // Record and insert the statistics update into the FIFO
           // Note all user commands are in the same location (r12)
           // Read the target flow match count and update in the packet PS_INFO
           qbbc l_stdHeldEoamPktForward2_0, s_pktCxt6.eoamFlags.t_flag_reportCount 
             // Load the eoam count patch command
             lbco    s_paUsrStatsCfg, PAMEM_CONST_CUSTOM, OFFSET_USR_STATS_CFG, SIZE(s_paUsrStatsCfg)
             qble    l_stdHeldPktMatchStatsRead_1,  s_matchForward_eoam.statsIndex, s_paUsrStatsCfg.num64bCounters
               // Error condition
               // TBD: EOAM count is 4 bytes and hence is not 64bit count (8 bytes)
l_stdHeldPktMatchStatsRead_1:    
             lsl     s_usrStatsReadCxt.cnt32Offset, s_paUsrStatsCfg.num64bCounters,  3
             sub     r1.w0,  s_matchForward_eoam.statsIndex,  s_paUsrStatsCfg.num64bCounters  
             lsl     s_usrStatsReadCxt.cntOffset, r1.w0,  2
             add     s_usrStatsReadCxt.cntOffset, s_usrStatsReadCxt.cnt32Offset, s_usrStatsReadCxt.cntOffset
             mov     r1, PAMEM_USR_STATS_COUNTERS_LOC_GLOBAL
             lbbo    s_pktCxt6.count,  r1,  s_usrStatsReadCxt.cntOffset, 4 
             // Compensate for this packet
             add     s_pktCxt6.count, s_pktCxt6.count, 1
             sbco    s_pktCxt6.count, cCdeHeldPkt, SIZE(s_pktDescr)+OFFSET(s_pktCxt6.count),    4
             
l_stdHeldEoamPktForward2_0:             
           mov   s_usrStatsReq.index,  s_matchForward_eoam.statsIndex
           mov   s_usrStatsReq.pktSize,  s_pktCxt.endOffset 

           lbco  s_fifoCb, PAMEM_CONST_USR_STATS_FIFO_BASE,  OFFSET_PDSP_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
           add   r1.b0,    s_fifoCb.in, 4
           and   r1.b0,    r1.b0,       0x1F
        
           qbne  l_stdHeldEoamPktForward2_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL
            // Skip insert the request
            jmp fci_stdHeldPktEoamForward3

l_stdHeldEoamPktForward2_1:
           // Insert the request into the FIFO   
           add   r1.w2,   s_fifoCb.in, OFFSET_PDSP_USR_STATS_FIFO
           sbco  s_usrStatsReq,  PAMEM_CONST_USR_STATS_FIFO_BASE, r1.w2, SIZE(s_usrStatsReq)
           sbco  r1.b0,   PAMEM_CONST_USR_STATS_FIFO_BASE,    OFFSET_PDSP_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)

l_stdHeldPktEoamForward3_0:
        // No need to check further as the match happened for a known EOAM packet, that needs forwarding to a queue, after stats update
        qbbs  l_stdHeldEoamPktForward4, s_pktCxt6.eoamFlags.t_flag_knownOpcodes

        // OR No Match         
fci_stdHeldPktEoamForward3:
        // Check whether it is a cipher packet
        qbbc  l_stdHeldPktEoamForward5, s_pktCxt6.eoamFlags.t_flag_cipherPkt
          // Cipher packet found
          // Route to RA
          qbbs  l_stdHeldEoamPktForward3_1,   s_runCxt.flag2.t_raEn
            //  Send the packet on its way
            //  Free the packet Ext Info
            //  mov  s_pktExtDescr.threadId, s_matchForward_pa.dest

            //   Load flags and operation in one instruction
            ldi  r4, CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes            
            jmp  l_stdHeldEoamPktForward4
        
l_stdHeldEoamPktForward3_1:    
          // Increment the System stats for RA fragmentations, if found (Only during Ingress1 traffic)
          qbbc l_stdHeldEoamPktNoFrag, s_pktCxt6.eoamFlags.t_flag_fragFound 
             mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG           
l_stdHeldEoamPktNoFrag:
          // Send the packet on its way to RA
          qbbs l_stdHeldEoamPktForward3_set_ra_dest_1,    s_runCxt.flag2.t_raUseLocDMA
            // use global DMA     
            mov  s_pktExtDescr.threadId, THREADID_CDMA0
            mov  s_cdeCmdPkt.destQueue,  PASS_RA_QUEUE
            jmp  l_stdHeldEoamPktForward3_set_ra_dest_end
                
l_stdHeldEoamPktForward3_set_ra_dest_1:  
          // use local DMA
          mov  s_pktExtDescr.threadId, THREADID_CDMA1
          mov  s_cdeCmdPkt.destQueue,  PASS_RA_LOC_QUEUE
          // pass through              
                
l_stdHeldEoamPktForward3_set_ra_dest_end:                
          //   Free the packet Ext Info
          //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
          //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
          ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
          sbco s_cdeCmdPkt.destQueue,  cCdeHeldPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
          // mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes            
          mov  s_cdeCmdPkt.psInfoSize,  32          
          mov  s_cdeCmdPkt.flowId,      s_runCxt.raFlow          

l_stdHeldEoamPktForward4:
       
          // Clear Word2 flags in pktCxt, if there is no target match OR unknown opcode list
          zero   &s_pktCxt6.eoamFlags, 2        
          sbco   s_pktCxt6.megLevel, cCdeHeldPkt, SIZE(s_pktDescr)+OFFSET(s_pktCxt6.megLevel), 4              
          xout  XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)          
          xout  XID_CDECTRL,   s_cdeCmdPkt,   SIZE( s_cdeCmdPkt)          
          ret          

l_stdHeldPktEoamForward5:
        //   Free the packet Ext Info
        //   CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        //ldi  r4,  CDE_CMD_HPKT_RELEASE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO) << 8) 
        ldi  r4,  CDE_CMD_HPKT_RELEASE | (CDE_FLG_SET_PSINFO << 8)  
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes 
        // Check if the next stage is Ingress1 Othrewise no action
        qbne  l_stdHeldEoamPktForward4,   s_pktExtDescr.threadId,  PA_DEST_INGRESS1
        qbbc  l_stdHeldEoamPktForward4, s_pktCxt6.eoamFlags.t_flag_ip
          // Ip packet found
          // Update next route to Ingress 3 for Outer IP/UDP Parse
          mov  s_pktExtDescr.threadId,       PA_DEST_INGRESS3
          jmp  l_stdHeldEoamPktForward4

f_c1ForwardHeldEoamPktNoMatch:
        // Do a silent discard, release the packet, if previous stage set it as discard
        qbbc fci_stdHeldPktEoamForward3, s_pktExtDescr.flags.fDroppedInd
        
l_stdHeldEoamPktDiscard:        
          // mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_DISCARD
          ldi   s_cdeCmd.v0,    CDE_CMD_HPKT_DISCARD 
          set   s_pktExtDescr.flags.fDropped
          jmp   l_stdHeldEoamPktForward4

    .leave  usrStatsFifoScope        
    .leave  pktScope        // Used only for sizing
    .leave  lut1MatchScope    
    .leave  lut1Scope   
    .leave  cdeScope    
#endif


    .using  startScope
    .using  initScope

// .assign struct_timeAccConstants, r3, r5, s_accConst

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
    // 36 bytes, organized as follows: 
    //
    //   Byte offset    byte size    description
    //      0              4          l1Info
    //      4             16          paForward for matches
    //      20            16          paForward for previous matches

    zero  &r3,                    36       // zeros r3 - r11
    mov s_paForward.forwardType,  PA_FORWARD_TYPE_DISCARD   // default successful route
    mov s_paForward2.forwardType, PA_FORWARD_TYPE_DISCARD   // default failure route

    // r2.w0 = 0x4000, r2.w2 = 0
    mov r2,  256*64

#ifdef PASS_TIMESTAMP_OP
    sbco    r3,          PAMEM_CONST_RO_TIME_ACC_TABLE, OFFSET_RO_TIME_ACC_CONSTANTS, 12
#endif

#ifdef PASS_PROC_FIREWALL
    sbco    r3,          PAMEM_CONST_RESCORE_HDR,     OFFSET_RESCORE_HDR, 4
    // No need to clear the big 1k block of rescore data as it is not valid when header info is zero
#endif

c1LocalInit_0:
    sbco  s_l1Info,       PAMEM_CONST_PDSP_LUT1_INFO, r2.w2, 36
    add   r2.w2,          r2.w2,  64
    qbne  c1LocalInit_0,  r2.w2,  r2.w0

    // Clear the bitmask the indicates which LUT1 entries are valid
    zero  &s_l1Map,  SIZE(s_l1Map) + SIZE(s_l1Map2)        // also redundant, but this is init code so be safe
#ifdef PASS_FIREWALL_BUG_WORKAROUND
   // Disable all even entries to workaround the LUT1 automatic score bug
   mov  s_l1Map.validEntries0, 0x55555555
   mov  s_l1Map.validEntries1, 0x55555555
   mov  s_l1Map.validEntries2, 0x55555555
   mov  s_l1Map.validEntries3, 0x55555555
   mov  s_l1Map.validEntries4, 0x55555555
   mov  s_l1Map.validEntries5, 0x55555555
   mov  s_l1Map.validEntries6, 0x55555555
   mov  s_l1Map.validEntries7, 0x55555555
#endif

    sbco   s_l1Map,  PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_MAP,  SIZE(s_l1Map) + SIZE(s_l1Map2)

    // Clear the pending configuration enable 
    zero  &s_l1PendCmd,  SIZE(s_l1PendCmd)
    sbco   s_l1PendCmd, PAMEM_CONST_PDSP_CXT, PDSP_CXT_OFFSET_L1_PENDING, SIZE(s_l1PendCmd)


    // Initialize the call table. Each PDSP with a LUT1 will do this, but they will
    // each write the same values
    // TBD: Add compiler switch to initialize this at first PDSP only
    // May need different table for different PDSP
    // For example, MAC related entries is not supported by L3/L4 PDSP
    mov s_headerParse.c1ParseMac,            f_c1ParseMac
    mov s_headerParse.c1ParseVlan,           f_c1ParseVlan
    mov s_headerParse.c1ParseMpls,           f_c1ParseMpls
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
    mov s_headerParse.c1ParsePPPoE,          f_c1ParsePPPoE
    mov s_headerParse.c1ParseSctp,           f_c1ParseSctp
    mov s_headerParse.c1ParseUnkn,           f_c1ParseUnkn
    mov s_headerParse.c2ParseUdp,            f_c1ParseUdp
    mov s_headerParse.c2ParseUdpLite,        f_c1ParseUdpLite
    mov s_headerParse.c2ParseTcp,            f_c1ParseTcp

    sbco s_headerParse.c1ParseMac,  PAMEM_CONST_PARSE, 0, SIZE(s_headerParse)
    
#ifdef PASS_PROC_L2_PARSE
    // Ethertypes table
    mov s_ethertypes.vlan,      ETH_TAG_VLAN
    mov s_ethertypes.spVlan,    ETH_TAG_SP_OUTER_VLAN
    mov s_ethertypes.ip,        ETH_TYPE_IP
    mov s_ethertypes.ipv6,      ETH_TYPE_IPV6
    mov s_ethertypes.mpls,      ETH_TYPE_MPLS
    mov s_ethertypes.mplsMulti, ETH_TYPE_MPLS_MULTI
    mov s_ethertypes.PPPoE,     ETH_TYPE_PPPoE_SESSION
    mov s_ethertypes.PPPoE_discov, ETH_TYPE_PPPoE_DISCOVER
  
    sbco  &s_ethertypes,  PAMEM_CONST_PARSE,  OFFSET_ETYPE_TABLE, SIZE(s_ethertypes)
#endif

  // IP protocol table
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
#ifndef PASS_PROC_FIREWALL  
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_TCP
#else
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_TCP
#endif  
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
#ifndef PASS_PROC_FIREWALL  
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_UDP
#else
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_UDP
#endif  
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP,              1 

  // Protocol type 136: UDP lite
#ifndef PASS_PROC_FIREWALL  
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_UDP_LITE
#else
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6)  | PA_HDR_UDP_LITE
#endif  
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP_LITE,         1

  // Protocol type 132: SCTP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_SCTP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_SCTP,             1

#ifdef PASS_FIRST_PDSP    
  
    // Initialize the next Fail Route base table (4 * 8 = 32 byte)
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
    
    // clear event flag by deafult
    //zero &s_eventFlags, SIZE(s_eventFlags)
    
    // Verify the PDSP ID
    // TBD: Add compiler switch
    //lbco  r0.b0,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1
    //qbeq  c1LocalInit_1, r0.b0, 0 
    
#ifdef PASS_PROC_IP_REASSEM   
    
    .using  ipScope
    // Clear IP reassembly context
    // TBD: Add compiler switch (only two PDSPs need to do this)
    zero &s_ipReassmCxt, OFFSET(s_ipReassmCxt.tfMapTemp)  
    sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    .leave  ipScope
    
#endif  

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
    zero  &s_paAddL1f,            SIZE(s_paAddL1f)+SIZE(s_paFwdMatchPlace)+SIZE(s_paFwdNextFailPlace)
    mov    s_paFwdA.forwardType,  PA_FORWARD_TYPE_DISCARD
    mov    s_paFwdB.forwardType,  PA_FORWARD_TYPE_DISCARD
    mov    s_paFwdA.flowId,       0xde                     // Indicate that this entry has been deleted
    
    // Store the deleted forward table
    lsl  r0.w0,       s_paDelL1.index,             6        // The offset to this table entry
    sbco s_paAddL1f,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))
    
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
// *   R11:       |                                         |
// *   R12:       |
// *   R13:       |                                         | s_paAddL1f (s_paAddL1fAcl, s_paAddL1fVlink)
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
    zero  &s_paAddL1f,             SIZE(s_paAddL1f)
    
    // Make sure the packet has all the data
    mov  r0.w2,  SIZE(s_paAddL1Hdr)+(2*SIZE(s_paFwdMatchPlace)) + (SIZE(s_paAddL1Std1)*3) + SIZE(s_paAddL1Cmdhdr)
    qble l_paComAddRepLut1_0,  r0.w0,  r0.w2
        mov  r0.b0, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        qbbc l_paComAddRepLut1_err, s_pktCxt2.ctrlFlag.t_cmdHdrMoved
            jmp l_paComAddRepLut1_errPatch
            
l_paComAddRepLut1_0:
    // Read in the allocated index bitmap
    lbco  r1.b0,  PAMEM_CONST_PDSP_CXT,  PDSP_CXT_OFFSET_L1_MAP2,  1

#ifndef PASS_PROC_FIREWALL
    // Get a free index if requested
    mov   r0.w2,    PA_LUT1_INDEX_LAST_FREE                                          
    qbne  l_paComAddRepLut1_3,  s_paAddL1Hdr.index,  r0.w2
#else

#ifdef PASS_PROC_EOAM
   qbbc   l_paComAddRepLut1Eoam, s_runCxt.flag3.t_eoamEn 
    // Get a free index if requested
    mov   r0.w2,    PA_LUT1_INDEX_LAST_FREE                                          
    qbne  l_paComAddRepLut1_3,  s_paAddL1Hdr.index,  r0.w2     
    jmp   l_paComAddRepLut1_00    
l_paComAddRepLut1Eoam:   
#endif // PASS_PROC_EOAM
    mov   s_paAddL1fAcl.statsCmd,   s_paAddL1Hdr.index
    set   s_paAddL1fAcl.statsCmd.t_stats_update_req 
    add   s_paAddL1fAcl.statsCmd2, s_paAddL1fAcl.statsCmd,  1 
    set   s_paAddL1fAcl.statsCmd2.t_stats_type_byte
#endif    
l_paComAddRepLut1_00:     
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
    xin  XID_CDEDATA,  s_paFwdMatchPlace,  2*SIZE(s_paFwdMatchPlace)

    // store all the info (l1Info + match info)
    lsl  r0.w0,       s_paAddL1Hdr.index,          6       // The offset to this table entry
    sbco s_paAddL1f,  PAMEM_CONST_PDSP_LUT1_INFO,  r0.w0,  SIZE(s_paAddL1f)+(2*SIZE(s_paFwdMatchPlace))
    
    qbbc l_paComAddRepLut1_7a, s_pktCxt2.ctrlFlag.t_cmdContinue
        //  Need to patch the next LUT1 configuration command of the physical link (srcVC)
        //  Note: support multiple LUT1 command only, enhancement is required for more flexible multiple command opeartion
       mov  s_cdeCmdWd.byteCount,  2*SIZE(s_paFwdMatchPlace)+SIZE(s_paCmdHdr)+SIZE(s_paAddL1Hdr)+SIZE(s_paAddL1Std1)*2
       xout XID_CDECTRL,    s_cdeCmdWd,                  SIZE(s_cdeCmdWd)
       xin  XID_CDEDATA,    s_l1View3cIpsec.srcVC,       SIZE(s_l1View3cIpsec.srcVC)
       add  s_l1View3cIpsec.srcVC, s_l1View3cIpsec.srcVC, s_paAddL1Hdr.index
       xout XID_CDEDATA,    s_l1View3cIpsec.srcVC,       SIZE(s_l1View3cIpsec.srcVC)
    
l_paComAddRepLut1_7a:    
#ifdef TO_BE_DELETE
    #ifndef PASS_PROC_FIREWALL
    mov  s_l1PendCmd.operation,  LUT1_CMD_INSERTM
    #else
    mov  s_l1PendCmd.operation,  LUT1_CMD_INSERT
#ifdef PASS_PROC_EOAM
   // Overwrite the information for IN0PDSP1, when EOAM is enabled
   qbbc  l_paComAddRepLut1_7b, s_runCxt.flag3.t_eoamEn 
     mov  s_l1PendCmd.operation,  LUT1_CMD_INSERTM
l_paComAddRepLut1_7b:    
#endif    
    #endif
#endif  
    mov  s_l1PendCmd.operation,  LUT1_CMD_INSERTM
    mov  s_l1PendCmd.index, s_paAddL1Hdr.index
    mov  s_l1PendCmd.flags, 0
    mvid *&s_l1PendCmd.bitMask, *&s_paAddL1Cmdhdr.bitMask

#ifdef PASS_PROC_FIREWALL
    .using aclLut1RescoreScope
     
    qbbs  l_paComAddRepLut1_7b, s_runCxt.flag3.t_eoamEn 
      qbbc  l_paComAddRepLut1_7b, s_paAddL1Hdr.vLinkNum.t_pa_acl_rescore_list_gen
        // Set the bit in runtime context to indicate the rescore 
        set s_runCxt.flag3.t_trigRescoreProc
        clr s_paAddL1Hdr.vLinkNum.t_pa_acl_rescore_list_gen
        
        // Record the index found to the pending Lut1 index associated score, 
        // during rescore operations, if needed
        qbbc  l_paComAddRepLut1_7b, s_paAddL1Hdr.vLinkNum.t_pa_acl_rescore_pend_idx
          clr  s_paAddL1Hdr.vLinkNum.t_pa_acl_rescore_pend_idx
          add  r0, s_paAddL1Hdr.vLinkNum, OFFSET_RESCORE_DATA
          sbco s_paAddL1Hdr.index, PAMEM_CONST_RESCORE_DATA, r0, SIZE(s_paAclRescoreData.index)
        
l_paComAddRepLut1_7b:  

     .leave aclLut1RescoreScope      

#endif
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

   qbne l_paComAddLut1Standard_vlink, s_paAddL1Hdr.type, PA_COM_ADD_LUT1_SRIO
        // Populate the s_paAddL1f
        set     s_paAddL1f.ctrlFlag.custom_enable
        mov     s_paAddL1f.nextHdr,       s_paAddL1Srio2.nextHdr
        mov     s_paAddL1f.nextHdrOffset, s_paAddL1Srio2.nextHdrOffset
        
l_paComAddLut1Standard_vlink:
   qbne l_paComAddLut1Standard0, s_paAddL1Hdr.type, PA_COM_ADD_LUT1_VLINK
        // Populate the virtual link number
        set     s_paAddL1fv.ctrlFlag.vlink_enable
        mov     s_paAddL1fv.vLinkNum,      s_paAddL1Hdr.vLinkNum

l_paComAddLut1Standard0:
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
#ifdef PASS_PROC_FIREWALL
    // No special check for EOAM case is needed as rescore is never triggered
    qbbc l_paComLut1CompleteModify_1,s_runCxt.flag3.t_trigRescoreProc
      // Rescore can be scheduled now as the LUT1 is clean (nothing held)
      call f_c1FirewallRescore
l_paComLut1CompleteModify_1:        
#endif    

    jmp  fci_mainLoop7

    .leave lut1Scope
    .leave configScope
    .leave cdeScope

#ifdef PASS_PROC_FIREWALL
// *********************************************************************************************************
// * FUNCTION PURPOSE: ACL rescoring of an index
// *********************************************************************************************************
// * DESCRIPTION: Rescoring on a configured index is done, eventually all the indexs are rescored
// *              The rescore completion is indicated by clearing t_trigRescoreProc bit (which is set during add)    
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
    .using  aclLut1RescoreScope
    
f_c1FirewallRescore:
     // Wait for LUT1 engine to be free 
     wbc  s_flags.info.tStatus_Lut1Busy     


     lbco  s_paAclRescoreHdr, PAMEM_CONST_RESCORE_HDR, OFFSET_RESCORE_HDR,  SIZE(s_paAclRescoreHdr)     
     qbge  l_c1FirewallRescore_done, s_paAclRescoreHdr.finalOffset, s_paAclRescoreHdr.offset
       // Active entry, rescore it
       add  r0, s_paAclRescoreHdr.offset, OFFSET_RESCORE_DATA       

       // load data
       lbco  s_paAclRescoreData, PAMEM_CONST_RESCORE_DATA, r0,  SIZE(s_paAclRescoreData)     
       mov   s_l1DbgEntryCntlReg.entry, s_paAclRescoreData.index

       // read the entry values
       set   s_l1DbgEntryCntlReg.dbgRead.l1DbgCntlRegReadBit
       sbco  s_l1DbgEntryCntlReg, cLut1Regs, LUT1_DBG_CNTL_REG_OFFSET, 4
         
       // Wait until read is set in the debug entry data registers
l_c1FirewallRescore_1:       
       lbco  s_l1DbgEntryCntlReg, cLut1Regs, LUT1_DBG_CNTL_REG_OFFSET, 4       
       qbbs  l_c1FirewallRescore_1, s_l1DbgEntryCntlReg.dbgRead.l1DbgCntlRegReadBit

       // load bitmask, score, care fields and range values
       lbco  s_l1DbgEntryDataRegs12_15, cLut1Regs,    LUT1_DBG_ENTRY_DATA_REG12_OFFSET,      16  

       // if the entry is already deleted, do not re-enable it 
       qbeq l_c1FirewallRescore_skip, s_l1DbgEntryDataRegs12_15.careField0Entry, 0
         
           // Update View1 (16 bytes)
           lbco  s_l1ViewE,      cLut1Regs,         LUT1_DBG_ENTRY_DATA_REG_VIEW1_OFFSET, 16
           xout  XID_LUT1V1,     s_l1ViewE,         SIZE(s_l1ViewE)
    
           // Update View2
           lbco  s_l1ViewE,      cLut1Regs,         LUT1_DBG_ENTRY_DATA_REG_VIEW2_OFFSET, 16       
           xout  XID_LUT1V2,     s_l1ViewE,         SIZE(s_l1ViewE)
    
           // Update View3       
           lbco  s_l1ViewE.data0,      cLut1Regs,         LUT1_DBG_ENTRY_DATA_REG_VIEW3_OFFSET1, 4
           lbco &s_l1ViewE.data1,      cLut1Regs,         LUT1_DBG_ENTRY_DATA_REG_VIEW3_OFFSET2, 12       
           xout  XID_LUT1V3,           s_l1ViewE,         SIZE(s_l1ViewE)
    
           // Create the command, Update the new score in the command from rescore data information       
           mov   s_l1RescoreCmd.score, s_paAclRescoreData.score
           mov   s_l1RescoreCmd.index, s_paAclRescoreData.index
           
           // Rest of the values are same as whatever read from debug entry data regs
           mov   s_l1RescoreCmd.flags,   0
           mov   s_l1RescoreCmd.bitMask, s_l1DbgEntryDataRegs12_15.bitmaskEntry
           mov   s_l1RescoreCmd.range1,  s_l1DbgEntryDataRegs12_15.range1HighEntry
           mov   s_l1RescoreCmd.range0,  s_l1DbgEntryDataRegs12_15.range0HighEntry
           mov   s_l1RescoreCmd.care0,   s_l1DbgEntryDataRegs12_15.careField0Entry
           mov   s_l1RescoreCmd.care1,   s_l1DbgEntryDataRegs12_15.careField1Entry
     
           // Issue LUT1_INSERT command
           mov   s_l1RescoreCmd.operation, LUT1_CMD_INSERTM
           xout XID_LUT1CMD,       s_l1RescoreCmd,            SIZE(s_l1RescoreCmd)

           //Wait for LUT1 Busy
           wbs  s_flags.info.tStatus_Lut1Busy

           // Wait for LUT1 engine to be free 
           wbc  s_flags.info.tStatus_Lut1Busy 
         
l_c1FirewallRescore_skip:
           // Indicate one entry is completed
           add s_paAclRescoreHdr.offset, s_paAclRescoreHdr.offset, 4
           sbco  s_paAclRescoreHdr, PAMEM_CONST_RESCORE_HDR, OFFSET_RESCORE_HDR,  SIZE(s_paAclRescoreHdr)
           ret

l_c1FirewallRescore_done:
         clr  s_runCxt.flag3.t_trigRescoreProc
         ret
     
    .leave lut1Scope
    .leave aclLut1RescoreScope
    .leave cdeScope    
#endif
    
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
    //xin XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr) 
    
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
    sbco  s_pktCxt,  cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_pktCxt)
    clr   s_pktExtDescr.flags.fDoCRC
    
    // Ready to forward the packet
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_pktCxt) + 7) & 0xf8      // Round up to multiple of 8 bytes
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    
    ret
    
#endif    
    
.leave cdeScope  
.leave sctpScope
.leave pktScope
    
    
#include "parse1.p"

    .leave globalScopeC1

