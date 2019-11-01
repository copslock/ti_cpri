// *******************************************************************************************
// * FILE PURPOSE: Perform common PA configuration
// *******************************************************************************************
// * FILE NAME: pacfgcmn.p
// *
// * DESCRIPTION: Contains the functions that are common among PDSPs for PA configuration
// *
// *******************************************************************************************
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


// ********************************************************************************************
// * FUNCTION PURPOSE: Configuration main
// ********************************************************************************************
// * DESCRIPTION:  The top configuration function.
// *
// *        On entry:
// *            -    r26.w0 contains the packet ID (r26.b0) 
// *            -    the CDE is in the control section
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (exit)
// *   R1:    
// *   R2:    
// *   R3:    b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                        |
// *   R7:        |                                        |  Pa Command Header (exit)
// *   R8:        |                                        |
// *   R9:        |  LUT1 View1   - lut1Scope              |
// *   R10:       |                                        |  First command
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                     |      
// *   R15:          |                                     |     
// *   R16:          |                                     |  Command data 
// *   R17:          |  LUT1 View2  - lut1Scope            |     
// *   R18:          |                                     |
// *   R19:          |                                     |
// *   R20:          |                                     |
// *   R21:          |                                     |
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     | b3: ctrlFlag, b2:command result offset < 256: w0: pktId
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *********************************************************************************************


    .using  configScope
    .using  cdeScope
    .using  pktScope

f_paConfigure:

  // Store the packet ID into r26  (note: r26 = s_pktCxt.pseudo + scratch)
  // This operation also clears the s_pktCxt3.ctrlFlag and cmdResultOffset 
#ifdef PASS_USE_PKTID
  mov  r1.w0,   PA_PKT_ID_MASK
  and  r26,     s_pktCxt.scratch,  r1.w0
#else
  mov  r26.w2,  0  
#endif  

  // Store the packet size information from the descriptor
  // It will be overwritten 
  mov r0.w0,   s_pktDescr.pktDataSize

  // Scroll past the control info
  mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
  xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

  // Read in the command header
  xin   XID_CDEDATA,         s_paCmd1,      SIZE(s_paCmd1)

  // Patch swinfo 0 with the command header return ID value
  // There is no guarantee at this point that there was any valid data in this field,
  // but if that is the case then the command will fail later
  wbs   s_flags.info.tStatus_CDEOutPacket
  sbco  s_paCmd1.retContext,  cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0), SIZE(s_paCmd1.retContext)

  qbeq  l_paConfigure0,   s_paCmd1.magic,   PA_CONFIG_COMMAND_SEC_BYTE

      // Invalid command byte. Return an error
      // If the return destination is not host, no reply can be sent
      mov s_paCmd1.commandResult,    PA_COMMAND_RESULT_NO_COMMAND_MAGIC
      xout XID_CDEDATA,            s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
      jmp f_cfgReply

l_paConfigure0:

    // Store the offset to the command return value  (i.e. s_paCmd1.commandResult)
    // relativ to the beginning of packet data
    // The value may have to be patched if there is an error after the CDE window is advanced
    mov s_pktCxt3.cmdResultOffset,  OFFSET(s_paCmd1.commandResult)

   // Do not advance the window. The return code is still in view

#ifdef PASS_PROC_LUT1
   qbeq  l_paComAddRepLut1,  s_paCmd1.command, PA_CONFIG_COMMAND_ADDREP_LUT1
   qbeq  l_paComDelLut1,     s_paCmd1.command, PA_CONFIG_COMMAND_DEL_LUT1
#endif
#ifdef PASS_PROC_LUT2   
   qbeq  l_paComAddRepLut2,  s_paCmd1.command, PA_CONFIG_COMMAND_ADDREP_LUT2
   qbeq  l_paComDelLut2,     s_paCmd1.command, PA_CONFIG_COMMAND_DEL_LUT2
#endif   
   qbeq  f_paComReqStats,    s_paCmd1.command, PA_CONFIG_COMMAND_REQ_STATS
   qbeq  f_paSysConfigPa,    s_paCmd1.command, PA_CONFIG_COMMAND_SYS_CONFIG
#ifdef PASS_PROC_CMDSET   
   qbeq  f_paComCmdSet,      s_paCmd1.command, PA_CONFIG_COMMAND_CMD_SET
#endif   
#ifdef PASS_PROC_MULTI_ROUTE
   qbeq  f_paComMultiRoute,  s_paCmd1.command, PA_CONFIG_COMMAND_MULTI_ROUTE
#endif  
#ifdef PASS_PROC_USR_STATS 
   qbeq  f_paComUsrStats,    s_paCmd1.command, PA_CONFIG_COMMAND_USR_STATS
#endif   
   qbeq  f_paComCrcEngine,   s_paCmd1.command, PA_CONFIG_COMMAND_CRC_ENGINE
   qbeq  f_paComConfigPa,    s_paCmd1.command, PA_CONFIG_COMMAND_CONFIG_PA
   qbeq  f_paComReqVersion,  s_paCmd1.command, PA_CONFIG_COMMAND_REQ_VERSION

   mov   s_paCmd1.commandResult,  PA_COMMAND_RESULT_INVALID_CMD
   xout  XID_CDEDATA,             s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)

   jmp   f_cfgReply
      
#ifdef PASS_PROC_LUT1   
l_paComAddRepLut1:
    jmp  f_paComAddRepLut1  // no return
    
l_paComDelLut1:
    jmp  f_paComDelLut1     // no return
#endif
    
#ifdef PASS_PROC_LUT2   
l_paComAddRepLut2:
    jmp  f_paComAddRepLut2  // no return
    
l_paComDelLut2:
    jmp  f_paComDelLut2     // no return
#endif    
    .leave  cdeScope
    .leave  configScope
    .leave pktScope


// *****************************************************************************************
// * FUNCTION PURPOSE: Return the command reply
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
    
f_cfgReply:

   qbne  l_cfgReply1,      s_paCmd1.replyDest,   THREADID_CDMA
   
   // command response counter
   lbco  r1, FIRMWARE_MBOX, 8, 4
   add   r1, r1, 1
   sbco  r1, FIRMWARE_MBOX, 8, 4
   
   qbbc  l_cfgReply0,      s_pktCxt3.ctrlFlag.t_cmdResultPatch
   
   // Patch the command result
   mov  s_cdeCmdPatch.operation,    CDE_CMD_PATCH_PACKET
   mov  s_cdeCmdPatch.offset,       s_pktCxt3.cmdResultOffset
   mov  s_cdeCmdPatch.len,          SIZE(s_paCmd1.commandResult)
   mov  s_cdeCmdPatch.data,         s_paCmd1.commandResult
   xout XID_CDECTRL,                s_cdeCmdPatch,                          SIZE(s_cdeCmdPatch)

l_cfgReply0:

l_cfgReply0_queue_bounce:
        // Check for Queue Bounce operation
l_cfgReply0_queue_bounce_ddr:
        qbbc l_cfgReply0_queue_bounce_msmc, s_paCmd1.replyQueue.t_pa_forward_queue_bounce_ddr
            clr s_paCmd1.replyQueue.t_pa_forward_queue_bounce_ddr
            sbco s_paCmd1.replyQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paCmd1.replyQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_cfgReply0_queue_bounce_end

l_cfgReply0_queue_bounce_msmc:
        qbbc l_cfgReply0_queue_bounce_end, s_paCmd1.replyQueue.t_pa_forward_queue_bounce_msmc
            clr s_paCmd1.replyQueue.t_pa_forward_queue_bounce_msmc
            sbco s_paCmd1.replyQueue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paCmd1.replyQueue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_cfgReply0_queue_bounce_end:

   mov  s_cdeCmdPkt.psInfoSize,  0
   mov  s_cdeCmdPkt.threadId,    s_paCmd1.replyDest
   mov  s_cdeCmdPkt.destQueue,   s_paCmd1.replyQueue
   mov  s_cdeCmdPkt.flowId,      s_paCmd1.flowId
   mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_ADVANCE
   mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE
   
   //
   // The command response packet should be held if there is held packet at classify1 (PDSP0, PDSP1 and PDSP2)
   //
   lbco  r1.b0,  PAMEM_CONST_PDSP_INFO,  OFFSET_ID,  1
   
   qble  l_cfgReply0_1, r0.b0, 3
   call  f_cfgReplyHoldPkt
   
l_cfgReply0_1:
   xout XID_CDECTRL,             s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
   jmp   fci_mainLoop7


l_cfgReply1:
   //  Discard the packet
   mov  s_cdeCmd.v0,    CDE_CMD_PACKET_FLUSH
   xout XID_CDECTRL,    s_cdeCmd.v0,     SIZE(s_cdeCmd.v0)
   
   // command response discard counter
   // Disable this since it is used by command3
#ifdef DISABLE_CMD_RESP_DISCARD_COUNTER   
   lbco  r1, FIRMWARE_MBOX, 12, 4
   add   r1, r1, 1
   sbco  r1, FIRMWARE_MBOX, 12, 4
#endif   

   jmp  fci_mainLoop7

 
    .leave  cdeScope
    .leave  configScope
    .leave  pktScope

// *****************************************************************************
// * FUNCTION PURPOSE: Process the pa config command
// *****************************************************************************
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
// *   R7:        |  command header (entry)
// *   R8:        |
// *   R9:        |
// *   R10:          | pa config
// *   R11:          | max counts             
// *   R12:          | outer IP Reassm        
// *   R13:          | inner IP Reassm        
// *   R14:            | cmdset                                      
// *   R15:            | usrStats
// *   R16:            | queueDiverst
// *   R17:            | pktCtrl
// *   R18:            | pktCtrl (MacPaddingCfg)
// *   R19:            | Events BitMap
// *   R20:            | queueBounce
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
// *****************************************************************************

    .using cdeScope
    .using configScope
    .using pktScope 

f_paComConfigPa:
   qble l_paComConfigPa0,  r0.w0,  PA_CONFIG_COMMAND_SIZE_CONFIG_PA
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply

l_paComConfigPa0:
   // Assume success. Will be overwritten on failure
   mov s_paCmd1.commandResult,  PA_COMMAND_RESULT_SUCCESS
   xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)

   // Input paConfig, max Counts,  outer IP Reassm and inner IP Reassm
   xin  XID_CDEDATA,  s_paCmdCfgA,  SIZE(s_paCmdCfgA) + SIZE(s_paComMaxCount) + 2*SIZE(struct_paIpReassmCfg)
   
   // Scroll the CDE past the command, max counts, valid events, and eroute enables
   mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,  PA_CONFIG_COMMAND_SIZE_THROUGH_IN_IP_REASM
   xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)
   
   // Input configurations for command set, usr stats, queue divert, and packet control
   xin  XID_CDEDATA,  s_paComCmdSetCfg,  SIZE(s_paComCmdSetCfg) + SIZE(s_paComUsrStats) + SIZE(s_paComQueueDivert) + SIZE(s_paComPktCtrl)+ SIZE(s_paComQueueBounce)


   qbbc  l_paComConfigPa1,   s_paCmdCfgA.validFlag.t_paCmdConfigValidMaxCount
   // Configure max counts
       sbco s_paComMaxCount,  PAMEM_CONST_CUSTOM,  OFFSET_MAX_HDR,      SIZE(s_paComMaxCount)

l_paComConfigPa1:
   qbbc  l_paComConfigPa2, s_paCmdCfgA.validFlag.t_paCmdConfigValidOutIpReassem
        // Configure outer IP
        sbco s_paComOutIpReassm,  PAMEM_CONST_CUSTOM,  OFFSET_OUT_IP_REASSM_CFG,      SIZE(s_paComOutIpReassm)
       
        // Inform PDSP1 through the mailbox command
        mov  r1.b3, FIRMWARE_CMD_IP_REASSEM_CFG
        mov  r2, FIRMWARE_P1_MBOX
        sbbo r1, r2, FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET, 4    
   
l_paComConfigPa2:
   qbbc  l_paComConfigPa3, s_paCmdCfgA.validFlag.t_paCmdConfigValidInIpReassem
        // Configure inner IP
        sbco s_paComInIpReassm,  PAMEM_CONST_CUSTOM,  OFFSET_IN_IP_REASSM_CFG,      SIZE(s_paComInIpReassm)
       
        // Inform PDSP1 through the mailbox command
        mov  r1.b3, FIRMWARE_CMD_IP_REASSEM_CFG
        mov  r2, FIRMWARE_P2_MBOX
        sbbo r1, r2, FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET, 4    
   
l_paComConfigPa3:
   qbbc  l_paComConfigPa4, s_paCmdCfgA.validFlag.t_paCmdConfigValidCmdSet
   // Configure command set
       sbco s_paComCmdSetCfg,  PAMEM_CONST_CUSTOM,  OFFSET_CMDSET_CFG,      SIZE(s_paComCmdSetCfg)
   
l_paComConfigPa4:
   qbbc  l_paComConfigPa5, s_paCmdCfgA.validFlag.t_paCmdConfigValidUsrStats
   // Configure user stats
       sbco s_paComUsrStats,  PAMEM_CONST_CUSTOM,  OFFSET_USR_STATS_CFG,      SIZE(s_paComUsrStats)
   
l_paComConfigPa5:
   qbbc  l_paComConfigPa6, s_paCmdCfgA.validFlag.t_paCmdConfigValidQueueDivert
   // Configure queue diversion
       sbco s_paComQueueDivert,  PAMEM_CONST_CUSTOM,  OFFSET_QUEUE_DIVERT_CFG,     SIZE(s_paComQueueDivert)

l_paComConfigPa6:
   qbbc  l_paComConfigPa7, s_paCmdCfgA.validFlag.t_paCmdConfigValidPktCtrl
   
    qbbc l_paComConfigPa6_1, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_valid_mac_padding_cnt
        // Store padding configurations
        sbco s_paComMacPadding,   PAMEM_CONST_CUSTOM,  OFFSET_MAC_PADDING_CFG,      SIZE(s_paComMacPadding)
        // pass through
   
l_paComConfigPa6_1:
    // skip mailbox 0 post if not needed
    mov     r1.w2, PA_SUB_VALID_POST_MAILBOX0_MASK
    and     r1.w0, s_paComPktCtrl.validBitMap, r1.w2
    qbeq    l_paComConfigPa6_chk_mbox12_post,  r1.w0, 0
   
        // Configure Packet Control
        mov r1.w2,  FIRMWARE_CMD_PKT_CTRL_CFG << 8
        
            // Inform PDSP0 for PPPoE, Packet Control and Padding check configuration
            mov r1.w0,  0
l_paComConfigPa6_ingress_pkt_clone_valid:
            qbbc    l_paComConfigPa6_pppoe_valid, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_ingress_pkt_capture
                set r1.b1.t_ingress_pCapEnable  
               
            qbbc    l_paComConfigPa6_pppoe_valid, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_ingress_pkt_capture
                set r1.b0.t_ingress_pCapEnable  
                
l_paComConfigPa6_pppoe_valid:
            qbbc    l_paComConfigPa6_macpad_valid, s_paComPktCtrl.validBitMap.t_pa_pkt_verify_proto_pppoe
                set r1.b1.t_PPPoEHdrCheck 

            qbbc    l_paComConfigPa6_macpad_valid, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_verify_proto_pppoe
                set r1.b0.t_PPPoEHdrCheck

l_paComConfigPa6_macpad_valid:
            qbbc    l_paComConfigPa6_ingress_route_valid, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_mac_padding_chk
                set r1.b1.t_macPaddingChk
                
            qbbc    l_paComConfigPa6_ingress_route_valid, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_mac_padding_chk
                set r1.b0.t_macPaddingChk               

l_paComConfigPa6_ingress_route_valid:
            qbbc    l_paComConfigPa6_mbox0_post, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_ingress_def_route
                set r1.b1.t_def_route
                
            qbbc    l_paComConfigPa6_mbox0_post, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_ingress_def_route
                set r1.b0.t_def_route

l_paComConfigPa6_mbox0_post: 
            mov  r2, FIRMWARE_P0_MBOX
            sbbo r1, r2, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4    

l_paComConfigPa6_chk_mbox12_post:
            // Check if we need to skip mailbox 1,2 post
            mov     r1.w2, PA_SUB_VALID_POST_MAILBOX12_MASK
            and     r1.w0, s_paComPktCtrl.validBitMap, r1.w2            
            qbeq    l_paComConfigPa6_chk_mbox45_post,  r1.w0, 0
            // Configure Packet Control
            mov r1.w2,  FIRMWARE_CMD_PKT_CTRL_CFG << 8 
            
            // Inform PDSP1/2 for IP configuration and Fragment configuration
            mov     r1.w0,  0

            qbbc    l_paComConfigPa6_ipfragsEroute_valid, s_paComPktCtrl.validBitMap.t_pa_pkt_verify_proto_ip
                set r1.b1.t_ipHdrCheck
                
            qbbc    l_paComConfigPa6_ipfragsEroute_valid, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_verify_proto_ip
                set r1.b0.t_ipHdrCheck
                
l_paComConfigPa6_ipfragsEroute_valid:                
            qbbc    l_paComConfigPa6_l3offsetInner_valid, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_ip_frags_to_eroute
                set r1.b1.t_ipFragToEroute 
                
            qbbc    l_paComConfigPa6_l3offsetInner_valid, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_ip_frags_to_eroute
                set r1.b0.t_ipFragToEroute

l_paComConfigPa6_l3offsetInner_valid:
            qbbc    l_paComConfigPa6_mbox12_post, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_l3offset_use_inner_ip
                set r1.b1.t_l3toInnerIp

            qbbc    l_paComConfigPa6_mbox12_post, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_l3offset_use_inner_ip
                set r1.b0.t_l3toInnerIp
                
l_paComConfigPa6_mbox12_post: 
            mov  r2, FIRMWARE_P1_MBOX
            sbbo r1, r2, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4 
            
            mov  r2, FIRMWARE_P2_MBOX
            sbbo r1, r2, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4 

l_paComConfigPa6_chk_mbox45_post:
            // check if we need to post anything in mailbox 4/5
            mov     r1.w2, PA_SUB_VALID_POST_MAILBOX45_MASK
            and     r1.w0, s_paComPktCtrl.validBitMap, r1.w2
            qbeq    l_paComConfigPa7,  r1.w0, 0
            
            // Configure Packet Control
            mov r1.w2,  FIRMWARE_CMD_PKT_CTRL_CFG << 8 
            
            // Inform PDSP4/5 for egress packet capture configuration
            mov     r1.w0,  0

            // Check valid bit check for egress pkt capture 
            qbbc    l_paComConfigPa6_eqos_valid_0, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_egress_pkt_capture
                set r1.b1.t_egress_pCapEnable 
               
            qbbc    l_paComConfigPa6_eqos_valid_0, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_egress_pkt_capture
                set r1.b0.t_egress_pCapEnable

l_paComConfigPa6_eqos_valid_0:
            qbbc    l_paComConfigPa6_mbox45_post, s_paComPktCtrl.validBitMap.t_pa_pkt_ctrl_eqos_mode
                set r1.b1.t_eqos_feature
                
            qbbc    l_paComConfigPa6_mbox45_post, s_paComPktCtrl.ctrlBitMap.t_pa_pkt_ctrl_eqos_mode
                set r1.b0.t_eqos_feature   
                //Store the global egress default priority
                mov  r2.w0, OFFSET_EQOS_CFG_EG_DEF_PRI
                sbco s_paComPktCtrl.egressDefPri, PAMEM_CONST_PORTCFG, r2.w0, 1

l_paComConfigPa6_mbox45_post:
            mov  r2, FIRMWARE_P4_MBOX
            sbbo r1, r2, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4             

            mov  r2, FIRMWARE_P5_MBOX
            sbbo r1, r2, FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET, 4             
         
l_paComConfigPa7:
   qbbc  l_paComConfigPa8, s_paCmdCfgA.validFlag.t_paCmdConfigValidQueueBounce
   // Configure queue bounce
       sbco s_paComQueueBounce,  PAMEM_CONST_CUSTOM,  OFFSET_QUEUE_BOUNCE_CFG,     SIZE(s_paComQueueBounce)

l_paComConfigPa8:
   jmp f_cfgReply

    .leave cdeScope
    .leave configScope
    .leave pktScope

// *****************************************************************************
// * FUNCTION PURPOSE: Process the pa system config command
// *****************************************************************************
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
// *   R7:        |  command header (entry)
// *   R8:        |
// *   R9:        |
// *   R10:          | pa system config
// *   R11:          | exception route bitmap 
// *   R12:          | 
// *   R13:          | 
// *   R14:            |                                       
// *   R15:            | Custom C1/C2 config or Exception Route
// *   R16:            |
// *   R17:            |
// *   R18:            | 
// *   R19:            |
// *   R20:            |
// *   R21:            |
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
// *****************************************************************************

    .using cdeScope
    .using configScope
    .using pktScope 

f_paSysConfigPa:
   // Assume success. Will be overwritten on failure
   mov s_paCmd1.commandResult,  PA_COMMAND_RESULT_SUCCESS
   xout  XID_CDEDATA,           s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)

   xin  XID_CDEDATA,  s_paCmdSysCfg,  SIZE(s_paCmdSysCfg) + SIZE(s_paComEroute)
   
   // Set Scroll command
   mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE

   qbne  l_paSysConfigPa1,   s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_EROUTE
   
   // Process EROUTE configuration
   mov  r0.w2,             PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EROUTE 
   qble l_paSysConfigPa0,  r0.w0,  r0.w2
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       
       jmp f_cfgReply
   
l_paSysConfigPa0:

   // Read the Eroute header now. The window must be advanced since the window
   // is not big enough to read all the exception route entries

   // Scroll the CDE past the command, and eroute enables
   mov  s_cdeCmdWd.byteCount,  PA_CONFIG_COMMAND_SIZE_THROUGH_EROUTE
   xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)

   jmp  f_paSysConfigEroute    // no return

l_paSysConfigPa1:
   // Scroll past the system configuration
   mov  s_cdeCmdWd.byteCount,   PA_CONFIG_COMMAND_SIZE_THROUGH_SYSCFG
   xout XID_CDECTRL,            s_cdeCmdWd,                           SIZE(s_cdeCmdWd)

   qbne  l_paSysConfigPa2, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_CUSTOM_LUT1
        // Process Custom LUT1 configuration
       qble l_paSysConfigPa1a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT1
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
              
       jmp f_cfgReply
   
l_paSysConfigPa1a:
       // configure custom LUT1 classify
       // r1.w0  offset to custom table
   
       xin  XID_CDEDATA,        s_paC1CustomHdr,      SIZE(s_paC1CustomHdr)
       
       qbge l_paSysConfigPa1b,  s_paC1CustomHdr.idx,  PA_MAX_C1_CUSTOM_TYPES
       mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX
       set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
       
       jmp  f_cfgReply
       
l_paSysConfigPa1b: 
       // Calcualte the offset: 40 bytes entries 
       lsl  r1.w0,    s_paC1CustomHdr.idx,  5
       lsl  r1.w2,    s_paC1CustomHdr.idx,  3
       add  r1.w0,    r1.w0,    r1.w2
       add  r1.w0,    r1.w0,    OFFSET_CUSTOM_C1
       
       sbco s_paC1CustomHdr,    PAMEM_CONST_CUSTOM,       r1.w0,  SIZE(s_paC1CustomHdr)
        
       mov  s_cdeCmdWd.byteCount, SIZE(s_paC1CustomHdr)
       xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)

       xin  XID_CDEDATA,     s_paC1Custom,          SIZE(s_paC1Custom)
       add  r1.w0,  r1.w0,   SIZE(s_paC1CustomHdr)
       sbco s_paC1Custom,    PAMEM_CONST_CUSTOM,    r1.w0,  SIZE(s_paC1Custom)

       mov  s_cdeCmdWd.byteCount, SIZE(s_paC1Custom)
       xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)
       
       jmp  f_cfgReply

l_paSysConfigPa2:

   qbne  l_paSysConfigPa3, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_CUSTOM_LUT2
   
   // Process Custom LUT2
   qble l_paSysConfigPa2a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT2
   mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
   set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
              
   jmp f_cfgReply
   
l_paSysConfigPa2a:
   
       // configure custom2 classify
       // r1.w0  offset to custom table
       xin  XID_CDEDATA,     s_paC2Custom,          SIZE(s_paC2Custom)
       qbge l_paSysConfigPa2b,  s_paC2Custom.idx,   PA_MAX_C2_CUSTOM_TYPES
       mov  s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX
       set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
       jmp  f_cfgReply
       
l_paSysConfigPa2b: 
       // Calcualte the offset: 16-byte entries 
       lsl    r1.w0,    s_paC2Custom.idx,  4
       add    r1.w0,    r1.w0,  OFFSET_CUSTOM_C2  
       sbco s_paC2Custom,  PAMEM_CONST_CUSTOM2,  r1.w0,  SIZE(s_paC2Custom)

       jmp f_cfgReply
       
l_paSysConfigPa3:

#ifdef PDSP_CLASSIFY1

   qbne  l_paSysConfigPa4, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_802_1AG
   
   .using lut1Scope
   
   
   // Process 802.1ag detector configuration
   // Note: This command will be processed at PDSP0 only
   qble l_paSysConfigPa3a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_802_1AG
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
              
        jmp f_cfgReply
   
l_paSysConfigPa3a:
       // 802.1ag configuration
       xin  XID_CDEDATA,     s_pa802p1agDet,        SIZE(s_pa802p1agDet)
       clr  s_runCxt.flag2.t_802_1agDet
       qbbc l_paSysConfigPa3b, s_pa802p1agDet.ctrlBitMap.t_802_1ag_cfg_enable
            // Detector is enabled
            set s_runCxt.flag2.t_802_1agDet
            clr s_runCxt.flag2.t_802_1agStd
                qbbc l_paSysConfigPa3b, s_pa802p1agDet.ctrlBitMap.t_802_1ag_cfg_standard
                    set s_runCxt.flag2.t_802_1agStd
       
l_paSysConfigPa3b:
   jmp f_cfgReply
   
   .leave lut1Scope
   
#endif   
   
   
l_paSysConfigPa4:

#ifdef PDSP_CLASSIFY2
   .using lut2Scope

   qbne  l_paSysConfigPa5, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_IPSEC_NAT_T

   // Process IPSEC NAT-T packet detector configuration
   // Note: This command will be processed at PDSP3 only
   qble l_paSysConfigPa4a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_IPSEC_NAT_T
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply
   
l_paSysConfigPa4a:
       // IPSEC NAT-T detector configuration
       xin  XID_CDEDATA,     s_paIpsecNatTDet,        SIZE(s_paIpsecNatTDet)
       clr  s_runCxt.ctrlFlag.t_c2_enable_IPSEC_NAT_T
       qbbc l_paSysConfigPa4b, s_paIpsecNatTDet.ctrlBitMap.t_ipsec_nat_t_cfg_enable
            set  s_runCxt.ctrlFlag.t_c2_enable_IPSEC_NAT_T
            
               
l_paSysConfigPa4b:
       // Store the new configuration 
       sbco s_paIpsecNatTDet,        PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG,  SIZE(s_paIpsecNatTDet)
       jmp f_cfgReply
       
l_paSysConfigPa5:       
       
   qbne  l_paSysConfigPa6, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_GTPU

   // Process GTPU configuration
   // Note: This command will be processed at PDSP3 only
   qble l_paSysConfigPa5a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_GTPU
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply
   
l_paSysConfigPa5a:
       // GTPU configuration
       xin  XID_CDEDATA,     s_paGtpuCfg,        SIZE(s_paGtpuCfg)
       clr  s_runCxt.ctrlFlag.t_c2_GTPU_use_link
       qbbc l_paSysConfigPa5b, s_paGtpuCfg.ctrlBitMap.t_pa_gtpu_control_use_link
            set  s_runCxt.ctrlFlag.t_c2_GTPU_use_link
               
l_paSysConfigPa5b:
       clr  s_runCxt.ctrlFlag.t_c2_GTPU_route_msg254_as_msg255
       qbbc l_paSysConfigPa5c, s_paGtpuCfg.ctrlBitMap.t_pa_gtpu_msg254_as_msg255
            set  s_runCxt.ctrlFlag.t_c2_GTPU_route_msg254_as_msg255       

l_paSysConfigPa5c:
       jmp f_cfgReply
       
    .leave lut2Scope
       
#endif       
       
l_paSysConfigPa6:
#ifdef PDSP_CLASSIFY1
   .using lut1Scope

   qbne  l_paSysConfigPa7, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_IGRESS_PCAP

   // Process Ingress packet capture command  
   qble l_paSysConfigPa6a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_PCAP
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply
        
l_paSysConfigPa6a:   
   // Load Ingress packet capture configuration
      xin   XID_CDEDATA, s_paPcapHdr, SIZE(s_paPcapHdr) + SIZE(s_paPktCapCfg)
     
l_paSysConfigPa6_readCfg:      
      qbeq  l_paSysConfigPa6a_complete, s_paPcapHdr.numPorts, 0

       // get the port number to be captured and obtain the offset
       lsl   r2.w0, s_paPktCapCfg.capPort, 3
       add   r2.w0, r2.w0, OFFSET_INGRESS_PKT_CAP_CFG_BASE 
       // Store the new configuration for interface ports
       sbco s_paPktCapInfo,        PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paPktCapInfo)

       // Advance the window
       mov  s_cdeCmdWd.byteCount, SIZE(s_paPktCapCfg)
       xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)
       
       // Load the next configuration
       // load the configuration
       xin   XID_CDEDATA,  s_paPktCapCfg,        SIZE(s_paPktCapCfg)
       
       // decrement loop counter to indicate a configuration is stored
       sub      s_paPcapHdr.numPorts, s_paPcapHdr.numPorts, 1
       jmp      l_paSysConfigPa6_readCfg

l_paSysConfigPa6a_complete:       
       jmp f_cfgReply   

l_paSysConfigPa7:
   qbne  l_paSysConfigPa8, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_EGRESS_PCAP

   // Process Engress packet capture command  
   qble l_paSysConfigPa7a,  r0.w0,  PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_PCAP
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply
        
l_paSysConfigPa7a:   
   // Load egress packet capture configuration
      xin   XID_CDEDATA, s_paPcapHdr, SIZE(s_paPcapHdr) + SIZE(s_paPktCapCfg)
   
l_paSysConfigPa7_readCfg:      
      qbeq  l_paSysConfigPa7a_complete, s_paPcapHdr.numPorts, 0

       // get the port number to be captured and obtain the offset
       lsl   r2.w0, s_paPktCapCfg.capPort, 3
       add   r2.w0, r2.w0, OFFSET_EGRESS_PKT_CAP_CFG_BASE
       // Store the new configuration for interface ports
       sbco s_paPktCapInfo,        PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paPktCapInfo)

       // Advance the window
       mov  s_cdeCmdWd.byteCount, SIZE(s_paPktCapCfg)
       xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)
       
       // Load the next configuration
       // load the configuration
       xin   XID_CDEDATA,  s_paPktCapCfg,        SIZE(s_paPktCapCfg)
       
       // decrement loop counter to indicate a configuration is stored
       sub      s_paPcapHdr.numPorts, s_paPcapHdr.numPorts, 1
       jmp      l_paSysConfigPa7_readCfg

l_paSysConfigPa7a_complete:       
       jmp f_cfgReply
   
   .leave lut1Scope   
#endif

l_paSysConfigPa8:
   qbne  l_paSysConfigPa9, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_DEFAULT_ROUTE
   // Process default route command 
   mov  r0.w2,             PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_DEF_ROUTE   
   qble l_paSysConfigPa8a,  r0.w0,  r0.w2
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply        

l_paSysConfigPa8a:   
   // Load default route configuration
      xin   XID_CDEDATA, s_paComDroute, SIZE(s_paComDroute) + SIZE(s_paComDrouteCfg) +  SIZE(s_paComDrouteFwd)      
      mov  r2.w2,        OFFSET_DEFAULT_ROUTE_CFG_BASE

l_paSysConfigPa8_readCfg:      
      qbeq  l_paSysConfigPa8a_complete, s_paComDroute.numPorts, 0

       // get the port number to be captured and obtain the offset
       lsl   r2.w0, s_paComDrouteCfg.capPort, 6
       add   r2.w0, r2.w0, r2.w2
       
       // Store the new configuration for multicast default route for this port
       sbco s_paComDrouteCfg,        PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paComDrouteCfg) + SIZE(s_paComDrouteFwd)
       add  r2.w0, r2.w0,            SIZE(s_paComDrouteCfg) + SIZE(s_paComDrouteFwd)

      // Advance the window to get broad cast
       mov  s_cdeCmdWd.byteCount, SIZE(s_paComDrouteFwd)
       xout XID_CDECTRL,          s_cdeCmdWd,             4 

       // Read the broadcast configuration
       xin   XID_CDEDATA, s_paComDrouteFwd, SIZE(s_paComDrouteFwd)
       sbco s_paComDrouteFwd,     PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paComDrouteFwd)
       add  r2.w0, r2.w0,         SIZE(s_paComDrouteFwd)

       // Advance the window for unicast no match cfg
       //mov  s_cdeCmdWd.byteCount,  SIZE(s_paComDrouteFwd)
       xout XID_CDECTRL,          s_cdeCmdWd,             4
       
       // Read the no match unicast default route configuration
       xin   XID_CDEDATA, s_paComDrouteFwd, SIZE(s_paComDrouteFwd)
       sbco s_paComDrouteFwd,     PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paComDrouteFwd)
       //add  r2.w0, r2.w0,         SIZE(s_paComDrouteFwd)       
       
       // Advance the window
       mov  s_cdeCmdWd.byteCount, SIZE(s_paComDrouteCfg) +  SIZE(s_paComDrouteFwd)
       xout XID_CDECTRL,          s_cdeCmdWd,             4
      
       // Load the next configuration
       // load the configuration
       xin   XID_CDEDATA,  s_paComDrouteCfg,        SIZE(s_paComDrouteCfg) + SIZE(s_paComDrouteFwd)        
       
       // decrement loop counter to indicate a configuration is stored
       sub      s_paComDroute.numPorts, s_paComDroute.numPorts, 1
       jmp      l_paSysConfigPa8_readCfg

l_paSysConfigPa8a_complete:       
       jmp f_cfgReply

l_paSysConfigPa9:
   qbne  l_paSysConfigPa10, s_paCmdSysCfg.sysCode,  PA_SYSTEM_CONFIG_CODE_EQOS
   // Process default route command 
   mov  r0.w2,             PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EQOS   
   qble l_paSysConfigPa9a,  r0.w0,  r0.w2
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply        

l_paSysConfigPa9a:   
   // Load default route configuration
      xin   XID_CDEDATA, s_paComEQoS, SIZE(s_paComEQoS) + SIZE(s_paComIfEQoS) +  SIZE(s_paComIfEQoSRouteOffset)      
      mov  r2.w2,                OFFSET_EQOS_CFG_BASE

l_paSysConfigPa9_readCfg:      
      qbeq  l_paSysConfigPa9a_complete, s_paComEQoS.numPorts, 0

       // get the port number to be captured and obtain the offset
       lsl   r2.w0, s_paComIfEQoS.port, 8
       add   r2.w0, r2.w0, r2.w2
       
       // Store the per port configuration for pbit eqos mode
       sbco s_paComIfEQoS,           PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paComIfEQoS) + SIZE(s_paComIfEQoSRouteOffset)
       add  r2.w0, r2.w0,            SIZE(s_paComIfEQoS) + SIZE(s_paComIfEQoSRouteOffset)

        // now store the per port configuration for the dscp eqos route table information
         mov  s_paComEQoS.scratch,  8
l_paSysConfigPa9a_dscp_map_store:
         qbeq  l_paSysConfig9_continue_next0, s_paComEQoS.scratch, 0
         // Advance the window to get next set
         mov  s_cdeCmdWd.byteCount, SIZE(s_paComIfEQoSRouteOffset)
         xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd) 

         // Read the configurations
         xin   XID_CDEDATA, s_paComIfEQoSRouteOffset, SIZE(s_paComIfEQoSRouteOffset)        
         // Store the rest of the configurations
         sbco s_paComIfEQoSRouteOffset,        PAMEM_CONST_PORTCFG, r2.w0,  SIZE(s_paComIfEQoSRouteOffset)       
         add  r2.w0, r2.w0,                    SIZE(s_paComIfEQoSRouteOffset) 
         
         sub    s_paComEQoS.scratch, s_paComEQoS.scratch, 1
         jmp    l_paSysConfigPa9a_dscp_map_store         

l_paSysConfig9_continue_next0:
         // Advance the window to get next port configuration
         mov  s_cdeCmdWd.byteCount, SIZE(s_paComIfEQoS) + SIZE(s_paComIfEQoSRouteOffset)
         xout XID_CDECTRL,          s_cdeCmdWd,           4 

         // Read the configurations
         xin   XID_CDEDATA, s_paComIfEQoS, SIZE(s_paComIfEQoS) + SIZE(s_paComIfEQoSRouteOffset)
         
         // decrement loop counter to indicate a configuration is stored
         sub      s_paComEQoS.numPorts, s_paComEQoS.numPorts, 1
         jmp      l_paSysConfigPa9_readCfg

l_paSysConfigPa9a_complete:       
         jmp f_cfgReply

l_paSysConfigPa10:
        // Error Handling: invalid command 
        mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_CMD
        set  s_pktCxt3.ctrlFlag.t_cmdResultPatch
        jmp f_cfgReply
       

    .leave cdeScope
    .leave configScope
    .leave pktScope

// ***********************************************************************************
// * FUNCTION PURPOSE: Load the exception routing table
// ***********************************************************************************
// * DESCRIPTION: Error routing is setup for each error type, which each type
// *              of event individually maskable
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:     scratch
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
// *   R15:          | Exception Routing
// *   R16:          |
// *   R17:          |    
// *   R18:            |
// *   R19:            |
// *   R20:            |
// *   R21:            |
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
// *
// ***********************************************************************************

    .using cdeScope
    .using configScope

f_paSysConfigEroute:
   
   mov  r1,   0   // r1.b0 counts the table entry number,  r1.w2 counts the offset in memory for the table

   mov  s_cdeCmdWd.byteCount,  SIZE(s_paComErouteFwd)

l_paSysConfigEroute0:
                   
   xin  XID_CDEDATA, s_paComErouteFwd,  SIZE(s_paComErouteFwd)
   xout XID_CDECTRL, s_cdeCmdWd,        SIZE(s_cdeCmdWd)

   // Store the routing table only if the route enable was set 
   qbbc  l_paSysConfigEroute1,  s_paComEroute.routeBitmap,  r1.b0  
      sbco  s_paComErouteFwd, PAMEM_CONST_EROUTE, r1.w2, SIZE(s_paComErouteFwd)

l_paSysConfigEroute1:

   add r1.b0,  r1.b0,  1
   add r1.w2,  r1.w2,  SIZE(s_paComErouteFwd)

   qbgt  l_paSysConfigEroute0,  r1.b0,  EROUTE_N_MAX

   jmp f_cfgReply

    .leave cdeScope
    .leave configScope
    
   
// *********************************************************************************************
// * FUNCTION PURPOSE: Return the statistics
// *********************************************************************************************
// * DESCRIPTION: The statistics are captured and optionally cleared
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:           |                                        
// *   R7:           | reply context
// *   R8:           |                                      
// *   R9:           |  
// *   R10:       |  paReqStats
// *   R11:       |  paUsrStatsGlobCfg
// *   R12:       |
// *   R13:       |
// *   R14:          | paUsrStatsClrCxt (clrBitMap)  or | system stats                                      
// *   R15:          |                                  |        
// *   R16:          |                                  |        
// *   R17:          |                                  |
// *   R18:            | 0  for clear                   |
// *   R19:            | 0                              |
// *   R20:            | 0                              |
// *   R21:            | 0                              |
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

    .using cdeScope
    .using configScope

f_paComReqStats:

    // Get the bits to clear
    xin  XID_CDEDATA,  s_paReqStats,  SIZE(s_paReqStats)

    // Move the CDE past the entire command
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  SIZE(s_paCmd1)+SIZE(s_paReqStats)
    xout XID_CDECTRL,           s_cdeCmdWd,                       SIZE(s_cdeCmdWd)
   
    // Is it system or user-defined statistics
    qbeq  l_paComReqUsrStats1, s_paReqStats.type,    PA_STATS_TYPE_USR

        // Latch the stats and clear as requested
        ldi   r1,    0
        qbbc  l_paComReqStats1,  s_paReqStats.ctrlBitMap.t_pa_usr_stats_clr
            fill &r1, 4 
      
l_paComReqStats1:
        sbco  r1,  cStatistics,  OFFSET_STATS_CAPTURE,  4
        
        // Copy the reply context to scratch temporarily  0x40000 + OFFSET_TEMP_BUFFER1
        mov   r3.w0,     OFFSET_TEMP_BUFFER1
        sbco  s_paCmd1,  PAMEM_CONST_SCRATCH2_BASE,  r3.w0,  SIZE(s_paCmd1)

        // Copy the system stats 16 register at a time into the local regs, then to scratch, then patch them
        // into the packet

        // Temporarily store stats to 0x40000 + OFFSET_TEMP_BUFFER2
        mov   r2.w2, 4					
        mov   r2.w0, OFFSET_TEMP_BUFFER2

        mov   r1, STATS_DATA_ADDR		// Copy stats from stats memory to local registers
        lbbo  r6, r1, 0, 16*4			// and then to scratch memory
        sbbo  r6, r2, 0, 16*4

        //zero  &s_cdeCmdInD,            SIZE(s_cdeCmdInD)
        //mov    s_cdeCmdInD.lenLsb,     16*4
        //mov    s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
        mov    s_cdeCmd.v0,            ((16*4) << 8) + CDE_CMD_INSERT_PACKET_BUFFER 
        mov    s_cdeCmdInD.dataP,      r2
        xout XID_CDECTRL,    s_cdeCmdInD,  SIZE(s_cdeCmdInD)

        // prepare and copy the second half of system statistics
        add   r1,  r1,  16*4
        add   r2,  r2,  16*4
        lbbo  r6,  r1,  0,    16*4
        sbbo  r6,  r2,  0,    16*4

        mov    s_cdeCmdInD.dataP,      r2
        xout XID_CDECTRL,    s_cdeCmdInD,  SIZE(s_cdeCmdInD)

        // Restore the reply context
        lbco s_paCmd1,  PAMEM_CONST_SCRATCH2_BASE,  r3.w0,  SIZE(s_paCmd1)
        jmp  f_cfgReply
   
l_paComReqUsrStats1:

#ifdef PASS_PROC_USR_STATS

   qbbc  l_paComReqUsrStats2,  s_paReqStats.ctrlBitMap.t_pa_usr_stats_req
        // Insert all user-defined statistics into the packet    
        //zero  &s_cdeCmdInD,            SIZE(s_cdeCmdInD)
        mov    s_cdeCmd.v0,            CDE_CMD_INSERT_PACKET_BUFFER
        mov    s_cdeCmdInD.lenMsbs,    (512*4) >> 8
        mov    s_cdeCmdInD.dataP,      PAMEM_USR_STATS_COUNTERS
        xout XID_CDECTRL,    s_cdeCmdInD,  SIZE(s_cdeCmdInD)
   
l_paComReqUsrStats2:
   qbbc  f_cfgReply,  s_paReqStats.ctrlBitMap.t_pa_usr_stats_clr
   
        qbne  l_paComReqUsrStats3,  s_paReqStats.numCnt,  PA_USR_STATS_CLEAR_ALL
            // Copy the reply context to scratch temporarily  0x40000 + OFFSET_TEMP_BUFFER1
            mov   r3.w0,     OFFSET_TEMP_BUFFER1
            sbco  s_paCmd1,  PAMEM_CONST_SCRATCH2_BASE,  r3.w0,  SIZE(s_paCmd1)
            zero   &r6,  16*4
        
            // Clear all statistics 
            // zero out 16 registers at a time 
            mov    r2,    0
            mov    r1,    PA_USR_STATS_NUM_ENTRIES * 4   
            
            // wait for CDE insertion operation to be completed.
            wbc   s_flags.info.tStatus_CDEBusy

l_paComReqUsrStats2_loop1:
            sbco  r6,    PAMEM_CONST_USR_STATS_COUNTERS, r2, 16*4
            add   r2,    r2,     16*4                
            qbne  l_paComReqUsrStats2_loop1, r2,           r1
            
            // Restore the reply context
            lbco s_paCmd1,  PAMEM_CONST_SCRATCH2_BASE,  r3.w0,  SIZE(s_paCmd1)
            jmp   f_cfgReply

l_paComReqUsrStats3:
        // Reset stats according to the usr-stats clear bitmaps
        // r0.b0: size of user-defined stats
        // r0.b1: remaining counters when the clrBitMap reach zero
        // r0.w2: offset changes based on 64-bit counters
        // r1.w0: 
        lbco    s_paUsrStatsGlobCfg, PAMEM_CONST_CUSTOM, OFFSET_USR_STATS_CFG, SIZE(s_paUsrStatsGlobCfg)
        // number of couners to be cleared should be a subset of counters set
        qbgt    l_paComReqUsrStats_err, s_paUsrStatsGlobCfg.numCounters, s_paReqStats.numCnt
        
            // Prepare CDE advance command
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            mov  s_cdeCmdWd.byteCount,  4
            // zero-out s_paUsrStatsClrCxt and r18-21
            zero &r14,  32
            
            // wait for CDE insertion operation to be completed.
            wbc   s_flags.info.tStatus_CDEBusy
            
            qbeq l_paComReqUsrStats3_init1, s_paUsrStatsGlobCfg.num64bCounters, 0
                // 64-bit counters
                mov r0.b0, 8
                lsl s_paUsrStatsClrCxt.cnt32Offset, s_paUsrStatsGlobCfg.num64bCounters, 3
                jmp l_paComReqUsrStats3_Loop1
            
l_paComReqUsrStats3_init1:
                // There is only 32-bit counter
                mov r0.b0, 4
                // mov s_paUsrStatsClrCxt.cnt32Offset, 0
                // pass through  
                
        
l_paComReqUsrStats3_Loop1:        
        
            // read in the next clrBitMap to be processed
            xin  XID_CDEDATA,  s_paUsrStatsClrCxt.clrBitMap,  SIZE(s_paUsrStatsClrCxt.clrBitMap)  
            xout XID_CDECTRL,  s_cdeCmdWd,                    4
            
            mov  s_paUsrStatsClrCxt.bitLoc, 0
            
l_paComReqUsrStats3_Loop2:            
            qbne l_paComReqUsrStats3_2, s_paUsrStatsClrCxt.clrBitMap, 0
                rsb r0.b1, s_paUsrStatsClrCxt.bitLoc, 32
                sub s_paUsrStatsGlobCfg.numCounters, s_paUsrStatsGlobCfg.numCounters, r0.b1
                qbeq l_paComReqUsrStats_end, s_paUsrStatsGlobCfg.numCounters, 0
                qbbs l_paComReqUsrStats_end, s_paUsrStatsGlobCfg.numCounters.t15
                
                qbeq l_paComReqUsrStats3_1_32b, r0.b0, 4
                
                // 64-bit counter range 
                lsl r0.w2, r0.b1, 3
                add r1.w0, s_paUsrStatsClrCxt.cntOffset, r0.w2
                qblt l_paComReqUsrStats3_1_64b, s_paUsrStatsClrCxt.cnt32Offset, r1.w0
                    // switch to 32-bit counter
                    sub r1.w0, r1.w0,  s_paUsrStatsClrCxt.cnt32Offset
                    lsr r1.w0, r1.w0,  1
                    add s_paUsrStatsClrCxt.cntOffset, s_paUsrStatsClrCxt.cnt32Offset, r1.w0
                    mov r0.b0, 4
                    jmp l_paComReqUsrStats3_Loop1
                
l_paComReqUsrStats3_1_64b:                
                    // remaining at 64-bit counter range
                    mov  s_paUsrStatsClrCxt.cntOffset, r1.w0
                    jmp  l_paComReqUsrStats3_Loop1
                    
l_paComReqUsrStats3_1_32b:                
                    // 32-bit counter range
                    lsl  r0.w2, r0.b1, 2
                    add  s_paUsrStatsClrCxt.cntOffset, s_paUsrStatsClrCxt.cntOffset, r0.w2
                    jmp  l_paComReqUsrStats3_Loop1
            
l_paComReqUsrStats3_2:
                // Verify whetehr this counter should be cleared 
                qbbc    l_paComReqUsrStats3_3,  s_paUsrStatsClrCxt.clrBitMap, s_paUsrStatsClrCxt.bitLoc
                    // Reset the counters
                    sbco r18,  PAMEM_CONST_USR_STATS_COUNTERS,  s_paUsrStatsClrCxt.cntOffset, b0
                    clr  s_paUsrStatsClrCxt.clrBitMap, s_paUsrStatsClrCxt.bitLoc 
                    sub  s_paReqStats.numCnt, s_paReqStats.numCnt, 1           
            
l_paComReqUsrStats3_3:
                // update for next counter
                sub s_paUsrStatsGlobCfg.numCounters, s_paUsrStatsGlobCfg.numCounters, 1
                qbeq l_paComReqUsrStats_end, s_paReqStats.numCnt, 0
                qbeq l_paComReqUsrStats_end, s_paUsrStatsGlobCfg.numCounters, 0
                
                add s_paUsrStatsClrCxt.bitLoc, s_paUsrStatsClrCxt.bitLoc, 1
                add s_paUsrStatsClrCxt.cntOffset, s_paUsrStatsClrCxt.cntOffset, r0.b0
                
                qbne l_paComReqUsrStats3_Loop2, s_paUsrStatsClrCxt.cntOffset, s_paUsrStatsClrCxt.cnt32Offset
                    // strat to process 32-bit counter
                    mov r0.b0, 4
                    jmp l_paComReqUsrStats3_Loop2  
        
   
l_paComReqUsrStats_err:   
l_paComReqUsrStats_end:

#endif

   jmp  f_cfgReply

    .leave cdeScope
    .leave configScope
   
// *********************************************************************************************
// * FUNCTION PURPOSE: Return version information
// *********************************************************************************************
// * DESCRIPTION: The version number is read from memory
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
    
    .using cdeScope
    .using configScope

f_paComReqVersion:

   // Move the CDE past the command
   mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,  SIZE(s_paCmd1)
   xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)


   // Insert 12 bytes to place the version info. A direct patch is not used
   // because it would take two steps to find out which PDSP this is
   mov  s_cdeCmdIn.len,        SIZE(s_paComReqVer)
   mov  s_cdeCmdIn.operation,  CDE_CMD_INSERT_PACKET
   xout XID_CDECTRL,           s_cdeCmdIn,              SIZE(s_cdeCmdIn)

   // Get the version info from memory
   lbco  s_paComReqVer,  PAMEM_CONST_PDSP_INFO,  0,                    SIZE(s_paComReqVer)
   xout  XID_CDEDATA,    s_paComReqVer,             SIZE(s_paComReqVer)


   jmp f_cfgReply

    .leave cdeScope
    .leave configScope

#ifdef PASS_PROC_MULTI_ROUTE
  
// ********************************************************************************************
// * FUNCTION PURPOSE: Configure multiple routing
// ********************************************************************************************
// * DESCRIPTION: A multiple destination route is added
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    scratch
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        | Commands 
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          | 
// *   R15:          |
// *   R16:          |  4 single route info  
// *   R17:          |
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
// ********************************************************************************************/

    .using cdeScope
    .using configScope
    .using multiFwdScope
    
f_paComMultiRoute:

   mov  r0.w2,  SIZE(s_paCmd1)+SIZE(s_paComMulti)+ (8 * SIZE(s_paSr0))
   qble l_paComMultiRoute0,  r0.w0,  r0.w2
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply
       
l_paComMultiRoute0:       
   // Read in the multi route command mode
   xin  XID_CDEDATA,  s_paComMulti,   SIZE(s_paComMulti)

   // Verify that the index is valid
   qbge  l_paComMultiRoute1,  s_paComMulti.idx,   PA_MULTI_ROUTE_NUM_ROUTES
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX
       xout XID_CDEDATA,    s_paCmd1.commandResult, SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply
       
l_paComMultiRoute1:
       // Initialize all 4 entries in the table to be discarded
       zero &s_paSr0, 4 * SIZE(s_paSr0)
       lsl  r2.w2,        s_paComMulti.idx,         6          // Offset in memory of where to put the data
       

   // Handle a new multi route request
   qbne  l_paComMultiRoute2,  s_paComMulti.mode,  PA_COMMAND_MULTI_ROUTE_MODE_ADD

       // Scroll the CDE
       mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
       mov  s_cdeCmdWd.byteCount, SIZE(s_paCmd1)+SIZE(s_paComMulti)
       xout XID_CDECTRL,          s_cdeCmdWd,                          SIZE(s_cdeCmdWd)

       // Read in the number of entries specified.
       // They are of type struct_paSingleRoute
       // Write the first 4  entries to memory, regardless of how many came in
       mov   r0.b3, 4 * SIZE(s_paSr0) 
       qble  l_paComMultiRoute1_1, s_paComMulti.nRoutes, 4  
            lsl   r0.b3,         s_paComMulti.nRoutes,   3
l_paComMultiRoute1_1:         
            xin   XID_CDEDATA,   s_paSr0,          b3

        sbco s_paSr0,      PAMEM_CONST_MULTI_ROUTE,  r2.w2,  4 * SIZE(s_paSr0)
        
        // Initialize all 4 entries in the table to be discarded
        zero &s_paSr0, 4 * SIZE(s_paSr0)
        add  r2.w2,    r2.w2,  4 * SIZE(s_paSr0)
        
       qbge  l_paComMultiRoute1_2, s_paComMulti.nRoutes, 4
          
         // Scroll the CDE
         mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
         mov  s_cdeCmdWd.byteCount, 4 * SIZE(s_paSr0)
         xout XID_CDECTRL,          s_cdeCmdWd,                          SIZE(s_cdeCmdWd)
         
         sub  r0.b3,    s_paComMulti.nRoutes,   4
         lsl  r0.b3,    r0.b3,                  3 
         xin  XID_CDEDATA,   s_paSr0,          b3
         
l_paComMultiRoute1_2:       
       sbco s_paSr0,      PAMEM_CONST_MULTI_ROUTE,  r2.w2,  4 * SIZE(s_paSr0)
         
       jmp  f_cfgReply

l_paComMultiRoute2:
   // Handle a delete
   qbne  l_paComMultiRoute3,  s_paComMulti.mode,  PA_COMMAND_MULTI_ROUTE_MODE_DEL

       sbco s_paSr0,    PAMEM_CONST_MULTI_ROUTE,  r2.w2,  4 * SIZE(s_paSr0)
       add  r2.w2,      r2.w2,  4 * SIZE(s_paSr0)
       sbco s_paSr0,    PAMEM_CONST_MULTI_ROUTE,  r2.w2,  4 * SIZE(s_paSr0)
       jmp  f_cfgReply

l_paComMultiRoute3:
   // Handle get multi route request
   qbne  l_paComMultiRoute4,  s_paComMulti.mode,  PA_COMMAND_MULTI_ROUTE_MODE_GET

       // Scroll the CDE
       mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
       mov  s_cdeCmdWd.byteCount, SIZE(s_paCmd1)+SIZE(s_paComMulti)
       xout XID_CDECTRL,          s_cdeCmdWd,                       SIZE(s_cdeCmdWd)

       // Patch in the data
       zero  &s_cdeCmdInD,            SIZE(s_cdeCmdInD)
       mov    s_cdeCmdInD.lenLsb,     8 * SIZE(s_paSr0)
       mov    s_cdeCmdInD.operation,  CDE_CMD_PATCH_PACKET_BUFFER
       mov    s_cdeCmdInD.dataP.w2,   PAMEM_BASE_MULTI_ROUTE >> 16
       mov    s_cdeCmdInD.dataP.w0,   PAMEM_BASE_MULTI_ROUTE & 0xffff
       add    s_cdeCmdInD.dataP,      s_cdeCmdInD.dataP,    r2.w2
       xout   XID_CDECTRL,            s_cdeCmdInD,          SIZE(s_cdeCmdInD)
       jmp    f_cfgReply


l_paComMultiRoute4:
   //  The command mode is invalid
   mov s_paCmd1.commandResult, PA_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE
   xout XID_CDEDATA,    s_paCmd1.commandResult, SIZE(s_paCmd1.commandResult)
   
   jmp f_cfgReply


    .leave configScope
    .leave cdeScope
    .leave multiFwdScope
    
#endif    

#ifdef PASS_PROC_USR_STATS
    
// ********************************************************************************************
// * FUNCTION PURPOSE: Configure Usr Statistics Link Table
// ********************************************************************************************
// * DESCRIPTION: Add multiple entries into the User statistics Link Table
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    scratch: r2.w0 = OFFSET_USR_STATS_CB; r2.w2 offset
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:           |                                        
// *   R7:           | reply context
// *   R8:           |                                      
// *   R9:           |  
// *   R10:       |  paComUsrStatsCfg
// *   R11:       |  paUsrStatsGlobCfg
// *   R12:       |
// *   R13:       |
// *   R13:       |
// *   R14:          | paComUsrStatsEntry (Link Entry)
// *   R15:          |
// *   R16:          |   
// *   R17:          |
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
// ********************************************************************************************/

    .using cdeScope
    .using configScope

f_paComUsrStats:


   // Variable length commad: Verify the minimum size
   mov  r0.w2,  SIZE(s_paCmd1)+SIZE(s_paComUsrStatsCfg)
   qble l_paComUsrStats0,  r0.w0,  r0.w2
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply
       
l_paComUsrStats0:       
   // Read in the usr stats configuration
   xin  XID_CDEDATA,  s_paComUsrStatsCfg,   SIZE(s_paComUsrStatsCfg)

   // Verify that the configuration is valid
   lbco  s_paUsrStatsGlobCfg,  PAMEM_CONST_CUSTOM,  OFFSET_USR_STATS_CFG,      SIZE(s_paUsrStatsGlobCfg)
   qbge  l_paComUsrStats1,  s_paComUsrStatsCfg.nEntries,   s_paUsrStatsGlobCfg.numCounters
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_USR_STATS_INVALID_CONFIG
       xout XID_CDEDATA,    s_paCmd1.commandResult, SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply
       
l_paComUsrStats1:
       // Verify whether to re-initialize the link table
       qbbc l_paComUsrStats2, s_paComUsrStatsCfg.ctrlBitMap.t_pa_usr_stats_cfg_clr_all
       
       // Initialize the User Statistics Link buffer  (512 * 2 bytes)
       mov    r2.w0, OFFSET_USR_STATS_CB
       mov    r2.w2, OFFSET_USR_STATS_CB + (PA_USR_STATS_NUM_ENTRIES * 2) 
  
       // Set all link Index to no link and disable 
       mov    r1.w0,       0xa000
       mov    r1.w2,       0xa000

l_paComUsrStats1_1:
       sbco  r1,    PAMEM_USR_STATS_BASE, r2.w0, 4
       add   r2.w0, r2.w0,     4                
       qbne  l_paComUsrStats1_1, r2.w0,  r2.w2

l_paComUsrStats2:
       // Scroll the CDE to the first link entry
       mov  s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
       mov  s_cdeCmdWd.byteCount, SIZE(s_paCmd1)+SIZE(s_paComUsrStatsCfg)
       xout XID_CDECTRL,          s_cdeCmdWd,         SIZE(s_cdeCmdWd)
       
       mov  s_cdeCmdWd.byteCount, SIZE(s_paComUsrStatsEntry)
       mov  r2.w0,  OFFSET_USR_STATS_CB

l_paComUsrStats3:
       qbeq f_cfgReply, s_paComUsrStatsCfg.nEntries, 0
        
       // Read in the link Entry and copy it the the specified Link Table location
       xin   XID_CDEDATA,   s_paComUsrStatsEntry,   SIZE(s_paComUsrStatsEntry)   
       
       lsl  r2.w2,        s_paComUsrStatsEntry.index,       1          // Offset in memory of where to put the data
       add  r2.w2,  r2.w2, r2.w0
       sbco s_paComUsrStatsEntry.lnkIndex,  PAMEM_USR_STATS_BASE, r2.w2, 2
       
       //prepare for the next entry
       sub  s_paComUsrStatsCfg.nEntries, s_paComUsrStatsCfg.nEntries, 1
       xout XID_CDECTRL,          s_cdeCmdWd,         SIZE(s_cdeCmdWd)
           
       jmp  l_paComUsrStats3
       

    .leave configScope
    .leave cdeScope

#endif    

// ********************************************************************************************
// * FUNCTION PURPOSE: Configure CRC engine
// ********************************************************************************************
// * DESCRIPTION: Configure CRC engine
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    scratch: b0 = data register index
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        | Common Command Header                                     
// *   R8:        | s_paCmd1                                     
// *   R9:        | 
// *   R10:         |  CRC Configuration Parameters
// *   R11:         |  s_paComCfgCrc
// *   R12:       |
// *   R13:       |
// *   R14:          | 
// *   R15:          |
// *   R16:          |  16 4-byte CRC table entries (Two groups) 
// *   R17:          |
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
// ********************************************************************************************/

    .using cdeScope
    .using configScope

f_paComCrcEngine:

   mov  r0.w2,  SIZE(s_paCmd1)+SIZE(s_paComCfgCrc)+ (16 * SIZE(s_cdeCrcTbl.value))
   qble l_paComCrcEngine0,  r0.w0,  r0.w2
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply

l_paComCrcEngine0:
#ifdef PDSP_CLASSIFY1
   // enable SCTP checksum verification since the CRC engine is configured
   set  s_runCxt.flags.t_sctpChksum
#endif
   // Read in the CRC configuration command
   xin  XID_CDEDATA,  s_paComCfgCrc,   SIZE(s_paComCfgCrc)

   // Issue CDE CRC configuration command
   mov  s_cdeCrcCfg.flags,      s_paComCfgCrc.ctrlBitMap
   mov  s_cdeCrcCfg.initVal,    s_paComCfgCrc.initVal
   mov  s_cdeCrcCfg.operation,  CDE_CMD_CRC_CONFIG
   xout XID_CDECTRL,            s_cdeCrcCfg,    SIZE(s_cdeCrcCfg)
   
   // Scroll the CDE to the first half of CRC table
   mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,   SIZE(s_paCmd1)+SIZE(s_paComCfgCrc)
   xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
   
   // Progrom the first 8 entries of the CRC table
   xin  XID_CDEDATA,    r14,    32
   mov  s_cdeCrcTbl.index,      0
   mov  s_cdeCrcTbl.operation,  CDE_CMD_CRC_TABLE_ENTRY
   
   mov  r1.b0,  &r14
   
l_paComCrcEngine1:   
  
   mvid  s_cdeCrcTbl.value,  *r1.b0
   xout  XID_CDECTRL,         s_cdeCrcTbl,    SIZE(s_cdeCrcTbl) 
   
   // Increement counters for the next entry 
   add   r1.b0, r1.b0, 4
   add   s_cdeCrcTbl.index, s_cdeCrcTbl.index, 1
   qbgt  l_paComCrcEngine1, s_cdeCrcTbl.index, 8 
   
   
   // Scroll the CDE to the second half of CRC table
   mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,   32
   xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
   
   // Progrom the last 8 entries of the CRC table
   xin  XID_CDEDATA,    r14,    32
   mov  s_cdeCrcTbl.index,      8
   mov  s_cdeCrcTbl.operation,  CDE_CMD_CRC_TABLE_ENTRY

   mov  r1.b0,  &r14

l_paComCrcEngine2:   
  
   mvid  s_cdeCrcTbl.value,  *r1.b0
   xout  XID_CDECTRL,         s_cdeCrcTbl,    SIZE(s_cdeCrcTbl) 
   
   // Increement counters for the next entry 
   add   r1.b0, r1.b0, 4
   add   s_cdeCrcTbl.index, s_cdeCrcTbl.index, 1
   qbgt  l_paComCrcEngine2, s_cdeCrcTbl.index, 16 

   jmp f_cfgReply

    .leave configScope
    .leave cdeScope
    
#ifdef PASS_PROC_CMDSET    
    
// ********************************************************************************************
// * FUNCTION PURPOSE: Configure Command Set
// ********************************************************************************************
// * DESCRIPTION: Configure Command Set
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    scratch: b0 = data register index
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        | Common Command Header                                     
// *   R8:        | s_paCmd1                                     
// *   R9:        | 
// *   R10:         |  Command Set Parameters (s_paComCmdSet)
// *   R11:       |    
// *   R12:       |
// *   R13:       |
// *   R14:          | 
// *   R15:          |
// *   R16:          |  Command Set data (Two groups) 
// *   R17:          |
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
// ********************************************************************************************/

    .using cdeScope
    .using configScope

f_paComCmdSet:

   mov  r0.w2,  SIZE(s_paCmd1)+SIZE(s_paComCmdSet)+ PA_CMD_SET_SIZE
   qble l_paComCmdSet0,  r0.w0,  r0.w2
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_PKT_SIZE
       xout  XID_CDEDATA,          s_paCmd1.commandResult,           SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply


l_paComCmdSet0:

   // Read in the Command set command
   xin  XID_CDEDATA,  s_paComCmdSet,   SIZE(s_paComCmdSet)
   
   // Read the command set global comfiguration
   lbco s_paCmdSetCfg,  PAMEM_CONST_CUSTOM,  OFFSET_CMDSET_CFG,      SIZE(s_paCmdSetCfg)

   // Verify that the index is valid
   qbgt l_paComCmdSet1,  s_paComCmdSet.idx,   s_paCmdSetCfg.numCmdSets
       mov s_paCmd1.commandResult, PA_COMMAND_RESULT_INVALID_CMDSET_IDX
       xout XID_CDEDATA,    s_paCmd1.commandResult, SIZE(s_paCmd1.commandResult)
       jmp f_cfgReply
       
l_paComCmdSet1:
   // calculate the command set offset
   lsl  r1.w0,  s_paComCmdSet.idx, 6   
   
   qbne l_paComCmdSet2, s_paCmdSetCfg.cmdSetSize, 128
       lsl  r1.w0,  r1.w0,  1   
   
l_paComCmdSet2:   
   // Store the command header
   sbco s_paComCmdSet,  PAMEM_CONST_CMDSET_TABLE, r1.w0, SIZE(s_paComCmdSet)
   add  r1.w0,  r1.w0,  SIZE(s_paComCmdSet)

   // Scroll the CDE to the first section of command set
   mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,   SIZE(s_paCmd1)+SIZE(s_paComCmdSet)
   xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
   mov  s_cdeCmdWd.byteCount,   32
   sub  r0.b2,  s_paCmdSetCfg.cmdSetSize, 4
   
l_paComCmdSet3:   
   // Stote the command set
   xin  XID_CDEDATA,    r14,    32
   sbco r14,    PAMEM_CONST_CMDSET_TABLE, r1.w0, 32
   
   // Scroll the CDE to next section of command set
   xout XID_CDECTRL,   s_cdeCmdWd,     SIZE(s_cdeCmdWd)
   
   sub  r0.b2,  r0.b2,  32
   add  r1.w0,  r1.w0,  32
   qblt l_paComCmdSet3, r0.b2, 32
   
   // Stote the last section of the command set
   xin  XID_CDEDATA,    r14,    b2
   sbco r14,    PAMEM_CONST_CMDSET_TABLE, r1.w0, b2

   jmp f_cfgReply

    .leave configScope
    .leave cdeScope
    
#endif

#ifdef PASS_PROC_MULTI_ROUTE

// *******************************************************************************************
// * FUNCTION PURPOSE: Perform multi-forwarding of a packet
// *******************************************************************************************
// * DESCRIPTION: The multi-forward index is stored in the error index of the context.
// *              The packet is forwarded to multiple locations until the first
// *              discard destination is found
// *
// *        Note: The packet copy command will forward the packet at cdeOut, but keep the original
// *              packet at cedIn, All the opertion done to the output packet will not affect the
// *              original packet. 
// *              1. There is no need to flush out the control info section
// *              2. There is no need to set the PS Info section, just make it 0
// *              3. We need to flush out the packet data for every descriptor only destination  
// *
// *        On entry:
// *            - r26 has the packet ID
// *            - the CDE is in the control section
// *            - packet context has been loaded
// *
// *   Register Usage:  
// * 
// *   R0:    w0 - the packet length (input)
// *   R1:    scratch
// *   R2:    s_paSrX
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
// *   R15:          |  1 Single routei nfo                                        
// *   R16:          |                                          
// *   R17:          |  
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
// *******************************************************************************************

    .using cdeScope
    .using pktScope
    .using multiFwdScope


f_paMultiFwd:
  // Scroll past and flush the control info
  //mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
  //xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)

fci_paMultiFwd1:
  //  From the packet context, extract the multi route index, erase the error index
  //  Reset the command back to PSH_CMD_PA_RX_PARSE (0), reset control flag  and write back the context

  lsr  r0.b0,              s_pktCxt2.eP1C2Id,   PKT2_EIDX_SHIFT    // Extract the multi route index
  and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK
  and  s_pktCxt2.paCmdId_Length,    s_pktCxt2.paCmdId_Length,   NOT_PKT_CONTEXT_CMD_MASK
  and  s_pktCxt2.flag,  s_pktCxt2.flag, PA_BM_FLAG_MASK  
  xout XID_CDEDATA,        s_pktCxt2.paCmdId_Length,   OFFSET(s_pktCxt2.IdIdx)

  lsl  r0.w2,   r0.b0,             6    // Multiply by 64 to get the mem offset
  mov  r0.b0,   0
  
l_paMultiFwd2:
  
  lbco s_paSr0,  PAMEM_CONST_MULTI_ROUTE,  r0.w2,  SIZE(s_paSr0)
  
l_paMultiFwd3:

  qbbs l_paMultiFwd4,  s_paSr0.ctrlFlags.t_pa_multi_route_ctrl_active  

l_paMultiFwd3_1:
    // Discard the current packet and return
    mov  s_cdeCmdPkt.operation,  CDE_CMD_PACKET_FLUSH
    xout XID_CDECTRL,            s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
    ret

l_paMultiFwd4:

  qbbc l_paMultiFwd5,  s_paSr0.ctrlFlags.t_pa_multi_route_ctrl_desc_only
  
  // Flush out the packet                   
  mov  s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_PACKET
  xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)
  
  mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_END
  xout XID_CDECTRL,           s_cdeCmdWd,              SIZE(s_cdeCmdWd)

  // pass through
l_paMultiFwd5:  
  qbbc l_paMultiFwd6,  s_paSr0.ctrlFlags.t_pa_multi_route_ctrl_replace_swinfo0
    //replace swInfo0
    wbc   s_flags.info.tStatus_CDEBusy
    sbco  s_paSr0.swInfo0,  cCdeOutPkt,  OFFSET(s_pktDescr.swinfo0),  SIZE(s_pktDescr.swinfo0)

l_paMultiFwd6:
l_paMultiFwd6_queue_bounce:
        // Check for Queue Bounce operation
l_paMultiFwd6_queue_bounce_ddr:
        qbbc l_paMultiFwd6_queue_bounce_msmc, s_paSr0.queue.t_pa_forward_queue_bounce_ddr
            clr s_paSr0.queue.t_pa_forward_queue_bounce_ddr
            sbco s_paSr0.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paSr0.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_paMultiFwd6_queue_bounce_end

l_paMultiFwd6_queue_bounce_msmc:
        qbbc l_paMultiFwd6_queue_bounce_end, s_paSr0.queue.t_pa_forward_queue_bounce_msmc
            clr s_paSr0.queue.t_pa_forward_queue_bounce_msmc
            sbco s_paSr0.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_paSr0.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_paMultiFwd6_queue_bounce_end:

  zero &s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
  mov  s_cdeCmdPkt.optionsFlag, CDE_FLG_SET_THREADID | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_DESTQUEUE
  mov  s_cdeCmdPkt.operation,   CDE_CMD_PACKET_COPY
  mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
  mov  s_cdeCmdPkt.destQueue,   s_paSr0.queue
  mov  s_cdeCmdPkt.flowId,      s_paSr0.flowId
  xout XID_CDECTRL,             s_cdeCmdPkt,       SIZE(s_cdeCmdPkt)
  
  add  r0.w2,   r0.w2,  SIZE(s_paSr0)
  add  r0.b0,   r0.b0,  1 
  // Wait for new packet to be ready
  wbs   s_flags.info.tStatus_CDENewPacket
  
  // The context is read from the control section. Advance
  // to the control section 
  // Note:Extra step for swInfo0 patching 
  mov   s_cdeCmdWd.operation,  CDE_CMD_ADVANCE_TO_CONTROL
  xout  XID_CDECTRL,           s_cdeCmdWd,                  SIZE(s_cdeCmdWd)
  
  qbeq  l_paMultiFwd3_1, r0.b0,   8
    jmp  l_paMultiFwd2   

    .leave cdeScope
    .leave pktScope
    .leave multiFwdScope
    
#endif    

// *********************************************************************************************
// * FUNCTION PURPOSE: Current packet forwarding
// *********************************************************************************************
// * DESCRIPTION: The current packet is forwarded based on the forward information
// *              input.
// *    On Entry:
// *        R6-R9  has the forward match structure
// *        r26 has the packet ID
// *        r3 has the statistic to update on silent discard (can be used for
// *           classify1 or classify2)
// *
// *   Register Usage:  
// * 
// *   R0:   scratch
// *   R1:   scratch 
// *   R2:   scratch (r2.b0: psInfoSize, r2.b2: Thread ID of packet with command set)
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
// *   R14:
// *   R15:           | usrStats FIFO CB
// *   R16:           | usrStats Request
// *   R17:
// *   R18:
// *   R19:
// *   R20:
// *   R21:
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:  Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt) rsvd for C1, C2 and P          -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************

#ifdef PASS_PROC_PKT_FORWARD

    .using currentFwdScope
    .using pktScope
    .using cdeScope
    .using usrStatsFifoScope

f_curPktForward:

    // Record the effective packet size for user statistics update
    mov   s_usrStatsReq.pktSize,  s_pktCxt.endOffset  
    
    // Default  thread Id for command set is PA_DEST_PA_M_0
    // The value will be changed to PA_DEST_CDMA only if QoS routing is required
    // The packet will be routed to QoS queues and then maybe forwarded to Q645 where command set can be executed
    mov     r2.b2,  PA_DEST_PA_M_0    

    // Do not release the packet ID if the packet remains in PA
    qbne  l_curPktForward1,  s_curFwd.forwardType,  PA_FORWARD_TYPE_PA
    
        // Custom Routing handling 
        //      There should be no custom lookup after LUT2
        //      It does not happen for LUT1 or command error, either
        qbeq l_curPktForward0_3,  s_curFwd_pa.custType, PA_CUSTOM_TYPE_NONE
        
        // Patch the pktContext
        // eIndex should stote the custom Index
        lbco   r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt2.eP1C2Id), 1
        and    r1.b0,   r1.b0, NOT_PKT2_EIDX_MASK
        lsl    r1.b1,   s_curFwd_pa.custIdx, PKT2_EIDX_SHIFT
        or     r1.b0,   r1.b0,  r1.b1
        sbco   r1.b0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt2.eP1C2Id), 1
        
        // Patch the next header Type
        lbco   r1.w0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.hdrBitmask_nextHdr), 2
        and    r1.w0,   r1.w0, NOT_PKT_NEXTHDR_MASK
        qbeq   l_curPktForward0_1,   s_curFwd_pa.custType, PA_CUSTOM_TYPE_LUT1
        mov    r1.w2,   PA_HDR_CUSTOM_C2
        jmp    l_curPktForward0_2
l_curPktForward0_1:        
        mov    r1.w2,   PA_HDR_CUSTOM_C1
l_curPktForward0_2:        
        or     r1.w0,   r1.w0,  r1.w2
        sbco   r1.w0,   cCdeOutPkt,  SIZE(s_pktDescr)+OFFSET(s_pktCxt.hdrBitmask_nextHdr), 2

l_curPktForward0_3:
        //  Verify various control flags
        qbbc l_curPktForward0_4, s_curFwd_pa.ctrlBitMap.t_pa_cascaded_forwarding
            // update the pktCtx.flag2
            lbco s_pktCxt.flag2,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flag2), SIZE(s_pktCxt.flag2)
            set  s_pktCxt.flag2.t_flag2_cascaded_forwarding    
            sbco s_pktCxt.flag2,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.flag2), SIZE(s_pktCxt.flag2)
            // pass through
        
l_curPktForward0_4:
        //   Load flags and operation in one instruction
        ldi  r4, CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        mov  s_cdeCmdPkt.threadId,    s_curFwd_pa.dest
        xout XID_CDECTRL,             s_cdeCmdPkt,               SIZE(s_cdeCmdPkt)
        qbeq l_curPktForward9,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_curPktForward1:

    // Forwarding packets to the host
    qbne  l_curPktForward3,  s_curFwd.forwardType,  PA_FORWARD_TYPE_HOST

        // Patch swinfo0
        sbco s_curFwd_host.context,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_curPktForward1_0,    s_curFwd_host.psflags,  0 
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0,  s_curFwd_host.psflags
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
l_curPktForward1_0:
        // Check whether interface based routing is enabled or not 
        qbbc l_curPktForward1_no_if_changes, s_curFwd_host.ctrlBitmap.t_pa_routing_if_selection
            and  r2.b1, s_pktCxt.hdrBitmask3_frag_portNum, PKT_EMACPORT_MASK
            // There is no need to update destion queue and flow if interfcae is unknown
            qbeq  l_curPktForward1_no_priority, r2.b1, 0
                sub r2.b1, r2.b1, 1
                qbbc l_curPktForward1_no_eqos_changes, s_curFwd_host.ctrlBitmap.t_pa_routing_if_eqos
                 // load per port configuration from the scratch memory
                 // get the port number to be captured and obtain the offset  
                 // point to per port configuration structure                 
                 // Extract output port number 
                 // r0.w2: Ingress port base to extract default priority only
                 // r1.b0: port ctrlBitMap (queue offset)
                 // r1.b1: priority (flow offset)
                 // r1.w2: Egress port base (control and Offset table)
                 and   r2.b3, s_curFwd_host.psflags, PA_PKT_PS_EMAC_PORTS_MASK
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
                 qbbc  l_curPktForward1DscpEqosMode, r1.b0.t_eqos_dp_bit_mode
                   // dp bit mode
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_VLAN_MASK << PROT_COUNT_VLAN_SHIFT
                   qbeq l_curPktForward1NotVlanTag, r0.b0, 0
                     mov  r1.b1, s_pktCxt.vlanPriority
                     jmp  l_curPktForward1_comp
l_curPktForward1NotVlanTag:
                   qbbs l_curPktForward1_comp, r1.b0.t_eqos_pri_override
l_curPktForward1DscpEqosMode: 
#ifndef PASS_PROC_L2
                   and  r0.b0, s_pktCxt.protCount, PROT_COUNT_IP_MASK << PROT_COUNT_IP_SHIFT
                   qbeq l_curPktForward1_comp, r0.b0, 0
#else
                   and  r0.b0, s_pktCxt2.hdrBitmask2_nextHdr, PKT_NEXTHDR_MASK 
                   qbeq l_curPktForward1ProcDscp, r0.b0, PA_HDR_IPv4
                   qbne l_curPktForward1_comp, r0.b0, PA_HDR_IPv6
l_curPktForward1ProcDscp:                   
#endif                   
                     // point to dscp table
                     add  r1.w2, r1.w2, 16
                     mov  r1.b1, s_pktCxt.dscpPriority
                     // pass through
l_curPktForward1_comp:
                   //point to correct priority table offset
                   lsl  r1.b1, r1.b1, 1
                   add  r1.w2, r1.w2, r1.b1
                   lbco r1.w0, PAMEM_CONST_PORTCFG, r1.w2, 2
                   // add to base queue and flow
                   add  s_curFwd.flowId, s_curFwd.flowId, r1.b1
                   add  s_curFwd.queue,  s_curFwd.queue,  r1.b0
                   mov  r2.b2,  PA_DEST_CDMA                   
                   jmp  l_curPktForward1_no_priority                   

l_curPktForward1_no_eqos_changes:

                add  s_curFwd.queue, s_curFwd.queue, r2.b1

                qbbc l_curPktForward1_no_priority, s_curFwd_host.ctrlBitmap.t_pa_routing_if_dest_flow
                    // Now update the destination flow based on the ethernet interface
                    add  s_curFwd.flowId, s_curFwd.flowId, r2.b1
                    jmp  l_curPktForward1_no_priority

l_curPktForward1_no_if_changes:

        // adjust destination queue if QoS queue 
        qbbc l_curPktForward1_dscp_priority, s_curFwd_host.ctrlBitMap.t_pa_routing_priority_vlan
l_curPktForward1_vlan_priority: 
            add  s_curFwd.queue, s_curFwd.queue, s_pktCxt.vlanPriority
            mov  r2.b2,  PA_DEST_CDMA    
l_curPktForward1_dscp_priority:
            // No priority routing enabled 
        qbbc l_curPktForward1_no_priority, s_curFwd_host.ctrlBitMap.t_pa_routing_priority_dscp
            add  s_curFwd.queue, s_curFwd.queue, s_pktCxt.dscpPriority
            mov  r2.b2,  PA_DEST_CDMA    
l_curPktForward1_no_priority:

l_curPktForward1_queue_bounce:
        // Check for Queue Bounce operation
l_curPktForward1_queue_bounce_ddr:
        qbbc l_curPktForward1_queue_bounce_msmc, s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            clr s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_curPktForward1_queue_bounce_end

l_curPktForward1_queue_bounce_msmc:
        qbbc l_curPktForward1_queue_bounce_end, s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            clr s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_curPktForward1_queue_bounce_end:

        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_curPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_curPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        
        qbne l_curPktForward1_1, s_curFwdRxCmdHdr.cmd, PA_RX_CMD_INSERT
            // Insert bytes at the current location */
            mov  s_cdeCmdIn.len,        s_curFwdRxCmdInsert.numBytes
            mov  s_cdeCmdIn.operation,  CDE_CMD_INSERT
            mviw *&s_cdeCmdIn.data3,    *&s_curFwdRxCmdInsert.data1
            xout XID_CDECTRL,           s_cdeCmdIn,                 SIZE(s_cdeCmdIn)
            
            // update the pktCtx.endOffset
            lbco s_pktCxt.endOffset,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset)
            add  s_pktCxt.endOffset,    s_pktCxt.endOffset,  s_curFwdRxCmdInsert.numBytes
            sbco s_pktCxt.endOffset,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset)

            // pass through
        
l_curPktForward1_1:        
        
        qbbs l_curPktForward2,  s_curFwd_host.ctrlBitMap.t_pa_multiroute
            // Send the packet on its way
            ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
            mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
            mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
            xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
            qbeq l_curPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret

l_curPktForward2:     
            // For multi routing the command ID and the multi route index
            // must be patched into the packet. These are two bytes which span 2 32 bits words.
            // Both words are read in, changed, and sent out at once

            lbco s_pktCxt,                 cCdeOutPkt,                 SIZE(s_pktDescr), 8
            // Note: It can be skipped since the original commad is 0 
            //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
            or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,    PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
            set  s_pktCxt.flag.t_flag_multi_route
    
            and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  NOT_PKT2_EIDX_MASK
            lsl  r2.b0,              s_curFwd_host.multiIdx,   PKT2_EIDX_SHIFT
            or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  r2.b0
            sbco s_pktCxt,           cCdeOutPkt,         SIZE(s_pktDescr), 8
            
            // Send the packet on its way
            ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    s_curFwd_host.paPdspRouter
            xout XID_CDECTRL,             s_cdeCmdPkt,                  SIZE(s_cdeCmdPkt)
            qbeq l_curPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
            ret


l_curPktForward3:
    qbne  l_curPktForward4, s_curFwd.forwardType, PA_FORWARD_TYPE_SA
        // Routing to SA
        // Patch swinfo0 and swinfo2
        sbco s_curFwd_sa.swInfo0,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  8
        
        // Check whether command set is enabled, which precedes the multi-route option
        qbeq l_curPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET
        qbeq l_curPktForward8,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
        
        qbne l_curPktForward3_1, s_curFwdRxCmdHdr.cmd, PA_RX_CMD_INSERT
            // Insert bytes at the current location */
            mov  s_cdeCmdIn.len,        s_curFwdRxCmdInsert.numBytes
            mov  s_cdeCmdIn.operation,  CDE_CMD_INSERT
            mviw *&s_cdeCmdIn.data3,    *&s_curFwdRxCmdInsert.data1
            xout XID_CDECTRL,           s_cdeCmdIn,                 SIZE(s_cdeCmdIn)
            
            // update the pktCtx.endOffset
            lbco s_pktCxt.endOffset,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset)
            add  s_pktCxt.endOffset,    s_pktCxt.endOffset,  s_curFwdRxCmdInsert.numBytes
            sbco s_pktCxt.endOffset,    cCdeOutPkt,    SIZE(s_pktDescr) + OFFSET(s_pktCxt.endOffset), SIZE(s_pktCxt.endOffset)

            // pass through
            
l_curPktForward3_1:        

        // Send the packet on its way
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_curPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_curPktForward4:
    qbne  l_curPktForward5, s_curFwd.forwardType, PA_FORWARD_TYPE_SRIO
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
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  8                       // SRIO take 8 bytes Ps Info
        mov  s_cdeCmdPkt.threadId,    PA_DEST_CDMA
        mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        ret

l_curPktForward5:
    qbne  l_curPktForward6, s_curFwd.forwardType, PA_FORWARD_TYPE_ETH
        // Routing to ETH
        // Patch the psflags
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        or   r2.b0, r2.b0,  s_curFwd_eth.psflags
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1

        // Send the packet on its way
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8)
        mov  s_cdeCmdPkt.psInfoSize,  0
        mov  s_cdeCmdPkt.threadId,    PA_DEST_ETH
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_curPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret


l_curPktForward6:
    qbeq  l_curPktForward7,  s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD

        // Invalid match type in table - this should never happen
        // Inc the stat and generate an event if enabled
        set s_statsFlags.event.t_nSystemFail

l_curPktForward7: 
        // Do a silent discard, release the packet ID
        or   s_statsFlags.event,  s_statsFlags.event,  r3   // r3 has silent discard stat for C1 or C2
        mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_FLUSH
        xout  XID_CDECTRL,           s_cdeCmdPkt,                 SIZE(s_cdeCmdPkt)
        qbeq  l_curPktForward9,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret
        //jmp   fci_mainLoop7
        
l_curPktForward8:     
            // For command set, the command ID and the command set index
            // must be patched into the packet. 

            lbco s_pktCxt,                 cCdeOutPkt,                 SIZE(s_pktDescr), 8
            // Note: It can be skipped since the original commad is 0 
            //and  s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,  NOT_PKT_CONTEXT_CMD_MASK
            or   s_pktCxt.paCmdId_Length,  s_pktCxt.paCmdId_Length,    PSH_CMD_RX_FWD << PKT_CONTEXT_CMD_SHIFT
            set  s_pktCxt.flag.t_flag_cmdset
    
            //and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,        NOT_PKT2_EIDX_MASK
            //lsl  r2.b0,              s_curFwdRxCmdSet.index,   PKT2_EIDX_SHIFT
            //or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  r2.b0
            sbco   s_curFwdRxCmdSet.index,  cCdeOutPkt,          SIZE(s_pktDescr) + OFFSET(s_pktCxt5.cmdSetIdx),    SIZE(s_pktCxt5.cmdSetIdx) 
            sbco   s_pktCxt,                cCdeOutPkt,          SIZE(s_pktDescr),  OFFSET(s_pktCxt.startOffset)
            
            // patch the flow Id and queue Id
            sbco s_curFwd.flowId, cCdeOutPkt,            OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
            sbco s_curFwd.queue,  cCdeOutPkt,            OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)
            
            // Send the packet on its way
            ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_THREADID | CDE_FLG_SET_PSINFO) << 8) 
            mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_pktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
            mov  s_cdeCmdPkt.threadId,    r2.b2
            xout XID_CDECTRL,             s_cdeCmdPkt,                  SIZE(s_cdeCmdPkt)
            qbeq l_curPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_CMDSET_USR_STATS
            ret
            
l_curPktForward9:   
        // User Statistics operation:
        // Record and insert the statistics update into the FIFO
        // Note all user commands are in the same location (r12)
        mov   s_usrStatsReq.index,  s_curFwd_host.cmd.w0
        
        lbco  s_fifoCb, PAMEM_CONST_PDSP_CXT,  OFFSET_PDSP0_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
        add   r1.b0,    s_fifoCb.in, 4
        and   r1.b0,    r1.b0,       0x1F
        
        qbne  l_curPktForward9_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            set s_statsFlags.event.t_nSystemFail
            ret

l_curPktForward9_1:
            // Insert the request into the FIFO   
            add   r1.w2,   s_fifoCb.in, OFFSET_PDSP0_USR_STATS_FIFO
            sbco  s_usrStatsReq,  PAMEM_CONST_PDSP_CXT, r1.w2, SIZE(s_usrStatsReq)
            sbco  r1.b0,   PAMEM_CONST_PDSP_CXT,    OFFSET_PDSP0_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
            ret

    .leave currentFwdScope
    .leave pktScope
    .leave cdeScope
    .leave usrStatsFifoScope
#endif

  
