// ********************************************************************************
// * FILE PURPOSE: Packet parsing functions
// ********************************************************************************
// * FILE NAME: parse1.p
// *
// * DESCRIPTION: Contains the functions that actually parse packets
// *
// ********************************************************************************
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
// *********************************************************************************
// * FUNCTION PURPOSE: Parse a MAC header
// *********************************************************************************
// * DESCRIPTION: the MAC destination and source addresses are added to the LUT
// *
// *   On entry:
// *            - CDE points to the MAC address
// *            - r30.w0 has the return address
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - param.action has SUBS_ACTION_PARSE
// *            - s_runCxt has valid packet context
// *
// *   On exit:
// *            - CDE points to the first byte after the MAC address
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - param.action has SUBS_ACTION_PARSE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  (packet desc) before read           |
// *   R7:        |                in MAC                |  mac addres (macVlanScope)
// *   R8:        |                                      |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// **********************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  macVlanScope


f_c1ParseMac:

#ifdef PASS_PROC_FIREWALL
  qbbs  l_c1ParseMacEoam, s_runCxt.flag3.t_eoamEn  
    jmp fci_c1FirewallException

l_c1ParseMacEoam:
  // May need to rewind to beginning of the packet
  // clear all the EOAM flags, they would be set per packet after EOAM parse
  #ifdef PASS_PROC_EOAM
      //  clr  s_pktCxt6.eoamFlags.t_flag_ip
      //  clr  s_pktCxt6.eoamFlags.t_flag_cipherPkt    
      //  clr  s_pktCxt6.eoamFlags.t_flag_fDisableCnt    
      //  clr  s_pktCxt6.eoamFlags.t_flag_knownOpcodes
      //  clr  s_pktCxt6.eoamFlags.t_flag_eType8902

      // The flags are cleared already from previous stage
      //  mov  s_pktCxt6.eoamFlags, 0
      //  mov  s_pktCxt6.startOffsetEoam, 0
      // As the previous stage used the count value, clear it here 
        mov  s_pktCxt6.count, 0
   #endif  // PASS_PROC_EOAM
#endif  // PASS_PROC_FIREWALL

#ifdef PASS_PROC_L2_PARSE

#ifndef PASS_PROC_EOAM 
  set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_MAC
  
  // Verify and clear psFlags_errorFlags
  // The CPSW passes CRC and Error Information: To be checked for error handling
  // This field needs to be cleared since psFlags and errorFlags will be used by PASS
  // abd SASS differently
  wbs  s_flags.info.tStatus_CDEOutPacket
  mov  r1.b0, 0
  sbco  r1.b0,  cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)

  // record the input EMA port number (1-4)
  // Note: It is one-based port number where 0 indicates that the packet is not from CPSW
  //       We can simply add the EMAC port without mask since this is only place that this field may be updated from initial value (0)
  and s_pktDescr.srcId, s_pktDescr.srcId, PKT_EMACPORT_MASK 
  lsl s_pktCxt.eId_portNum_nextHdr,  s_pktDescr.srcId, PKT_EMACPORT_SHIFT
  
  //Remove CRC from the endOffset if the packet is from CPSW
  //qbeq  l_c1ParseMac_1, s_pktDescr.srcId, 0
  qbbc  l_c1ParseMac_1, s_pktDescr.psFlags_errorFlags.t_psFlags_crcPresent  
      sub s_pktCxt.endOffset, s_pktCxt.endOffset, 4
#endif // PASS_PROC_EOAM

l_c1ParseMac_1:  
  // update inport
  zero &s_l1View3bMac, SIZE(s_l1View3bMac)
  mov  s_l1View3bMac.inPort,  s_pktDescr.srcId
  
  // Set pktType to indicate MAC operation
  set  s_l1View3bMac.pktType.fL2PktTypeMac

  // Load the destination and source mac into the LUT1 search
  xin  XID_CDEDATA,   s_macAddr,  SIZE(s_macAddr)
  mov  s_l1View3bMac.dstMac5,  s_macAddr.DstAddr_45.b0 
  // To be removed
  // Set the vlan tag to 0x8000 - the four msbs will be zero for a true tag.
  // This is done in case there is no vlan present. The ethertype will
  // definitely be placed later
  //mov  s_l1View1aMac.etherType,  0x8000

  // Write the entire view to the lut
  xout XID_LUT1V1,   s_l1View1aMac,    SIZE(s_l1View1aMac)
  
#ifndef PASS_PROC_EOAM 
  // cdeCmdWd.operation already has CDE_CMD_WINDOW_ADVANCE
  add  s_pktCxt.startOffset,   s_pktCxt.startOffset,  SIZE(s_macAddr)
#else
  add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, SIZE(s_macAddr)
  mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE  
#endif // PASS_PROC_EOAM

  mov  s_cdeCmdWd.byteCount,   SIZE(s_macAddr)
  xout XID_CDECTRL,            s_cdeCmdWd,            SIZE(s_cdeCmdWd)
  
l_c1ParseMac_BM:  
  // Detect Broadcast & Multicast
  qbbc  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_01.t_eth_multicast_ind
#ifndef PASS_PROC_EOAM
       // assume it is muticast 
       set   s_pktCxt.flags.t_flag_multicast
#endif     // PASS_PROC_EOAM  
       set   s_l1View3bMac.pktFlags.fL2Mcast
  
l_c1ParseMac_BM_0:
  
  mov   r0, 0xFFFF
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_01, r0
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_23, r0
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_45, r0 
#ifndef PASS_PROC_EOAM  
        // It is broadcast
        clr   s_pktCxt.flags.t_flag_multicast
        set   s_pktCxt.flags.t_flag_broadcast
#endif    // PASS_PROC_EOAM    
        clr   s_l1View3bMac.pktFlags.fL2Mcast         
        set   s_l1View3bMac.pktFlags.fL2Bcast

l_c1ParseMac_BM_end:

    // Load the ethertype table from memory  (move here to avoid multile load)
    // Note: The register space will not be overwritten during mac header parsing
    lbco  s_ethertypes,  PAMEM_CONST_PARSE,  OFFSET_ETYPE_TABLE,  SIZE(s_ethertypes)


  // Fall through to c1ProcessTagOrLen

    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave macVlanScope


// ************************************************************************************
// * FUNCTION PURPSE: Process an ethertype/802.3 ethertype/length field
// ************************************************************************************
// * DESCRIPTION: Examine the packet starting at an ethertype or length field
// *              and find the ethertype
// *
// *    On entry:
// *                - CDE points at the tag or length field
// *                - r30.w0 contains the return address
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *                - param.action has SUBS_ACTION_PARSE
// *
// *    On exit:
// *                - CDE points to the 1st byte after the ethertype (and llc/snap if present)
// *                - param.action has the next action to take
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:      |  checkScope  
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  (packet desc) before read           |
// *   R7:        |                in MAC                |  mac addres (macVlanScope)
// *   R8:        |                                      |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***********************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  macVlanScope

f_c1ProcessTagOrLen:

    mov  s_cdeCmdWd.byteCount,   2   // minimum 2 bytes for ethertype

    // Read in enough data to cover the llc/snap header if present
    xin  XID_CDEDATA,   s_tagOrLen,  SIZE(s_tagOrLen)

    mov    r3,    1500
    qblt   l_c1ProcessTagOrLen0,  s_tagOrLen.len,   r3

        // tag/len < 1500, so verify DSP, SSAP and ctrl
        qbne  l_c1ProcessTagOrLen9,  s_tagOrLen.dsap,  0xaa
        qbne  l_c1ProcessTagOrLen9,  s_tagOrLen.asap,  0xaa
        qbne  l_c1ProcessTagOrLen9,  s_tagOrLen.ctrl,  0x03
        
#ifndef PASS_PROC_EOAM        
        //Padding check ?
        qbbc l_c1ProcessTagOrLen00, s_runCxt.flag2.t_macPaddingChk
            // Is payload length >= 46?
            qble l_c1ProcessTagOrLen00, s_tagOrLen.len, 46 
            // Is packet size == 64?
            qbeq l_c1ProcessTagOrLen00, s_pktCxt.endOffset, 64
                //Padding error, update the counter and drop the packet
                // User Statistics operation:
                // Record and insert the statistics update into the FIFO
                
                mov   s_usrStatsReq.pktSize, s_pktCxt.endOffset
                lbco  s_usrStatsReq.index,  PAMEM_CONST_CUSTOM,  OFFSET_MAC_PADDING_CFG,  SIZE(s_usrStatsReq.index)
                lbco  s_fifoCb, PAMEM_CONST_USR_STATS_FIFO_BASE,  OFFSET_PDSP_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
                add   r1.b0,    s_fifoCb.in, 4
                and   r1.b0,    r1.b0,       0x1F
        
                qbne  l_c1ProcessTagOrLen000, s_fifoCb.out, r1.b0
                    // FIFO is full, bump the system error
                    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL

                    jmp l_c1ProcessTagOrLen9_1

l_c1ProcessTagOrLen000:
                // Insert the request into the FIFO   
                add   r1.w2,   s_fifoCb.in, OFFSET_PDSP_USR_STATS_FIFO
                sbco  s_usrStatsReq,  PAMEM_CONST_USR_STATS_FIFO_BASE, r1.w2, SIZE(s_usrStatsReq)
                sbco  r1.b0,   PAMEM_CONST_USR_STATS_FIFO_BASE,    OFFSET_PDSP_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
                 
                jmp l_c1ProcessTagOrLen9_1        
        
        
l_c1ProcessTagOrLen00:  
        // Mark 802.3
        set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_802_3
#endif       // PASS_PROC_EOAM  
        set   s_l1View3bMac.pktFlags.f802p3

        //  Copy the new ethertype value to the common location
        mov   s_tagOrLen.len,    s_tagOrLen.etype2

        // scroll past the 8 bytes of llc/snap header
        add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,  8

l_c1ProcessTagOrLen0:

    // Copy the ethertype value to LUT1 view 1
    mov  s_l1View1aMac.etherType,  s_tagOrLen.len

    xout XID_LUT1V1, s_l1View1aMac.etherType,  SIZE(s_l1View1aMac.etherType)
    xout XID_LUT1V3, s_l1View3bMac, SIZE(s_l1View3bMac)             

    // scroll past the ethertype/llc snap
    // cdeCmdWd.operation already has the value CDE_CMD_WINDOW_ADVANCE
    xout  XID_CDECTRL,   s_cdeCmdWd,          SIZE(s_cdeCmdWd)

#ifndef PASS_PROC_EOAM 
    // Track the active parse
    add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_cdeCmdWd.byteCount 
#else
    add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, s_cdeCmdWd.byteCount 
#endif // PASS_PROC_EOAM

    // Determine the next header type based on the tag value
    // These are arranged in order of expected appearance to reduce cycles in
    // the common cases. Assume the action is to do a lookup, and change
    // as required

    mov s_param.action,  SUBS_ACTION_LOOKUP

    qbne  l_c1ProcessTagOrLen1,   s_tagOrLen.len,  s_ethertypes.ip
        jmp  f_l2Ipv4Proc

l_c1ProcessTagOrLen1:

    qbne l_c1ProcessTagOrLen2,    s_tagOrLen.len,  s_ethertypes.ipv6
        jmp f_l2Ipv6Proc


l_c1ProcessTagOrLen2:
    qbne l_c1ProcessTagOrLen3,    s_tagOrLen.len,  s_ethertypes.vlan
        set s_l1View3bMac.pktFlags.fL2Vlan1
        // mov s_param.action,  SUBS_ACTION_PARSE
        // mov s_next.Hdr,      PA_HDR_VLAN
        // ret
        jmp f_c1ParseVlan

l_c1ProcessTagOrLen3:

    qbne l_c1ProcessTagOrLen4,    s_tagOrLen.len,   s_ethertypes.SpVlan
        set s_l1View3bMac.pktFlags.fL2Vlan2
        // mov s_param.action,  SUBS_ACTION_PARSE
        // mov s_next.Hdr,      PA_HDR_VLAN
        // ret
        jmp f_c1ParseVlan

l_c1ProcessTagOrLen4:

    qbne l_c1ProcessTagOrLen5,    s_tagOrLen.len,   s_ethertypes.mpls
        // mov s_param.action,  SUBS_ACTION_PARSE
        // mov s_next.Hdr,  PA_HDR_MPLS
        // ret
        jmp f_c1ParseMpls        

l_c1ProcessTagOrLen5:

    qbne l_c1ProcessTagOrLen6,    s_tagOrLen.len,   s_ethertypes.mplsMulti
        // mov s_param.action,  SUBS_ACTION_PARSE
        // mov s_next.Hdr,      PA_HDR_MPLS
        // ret
        jmp f_c1ParseMpls

l_c1ProcessTagOrLen6:

    qbne l_c1ProcessTagOrLen6_1,    s_tagOrLen.len,   s_ethertypes.PPPoE
        // mov s_param.action,  SUBS_ACTION_PARSE
        // mov s_next.Hdr,      PA_HDR_PPPoE
        // ret
        jmp f_c1ParsePPPoE
        
l_c1ProcessTagOrLen6_1:
    qbne l_c1ProcessTagOrLen7,      s_tagOrLen.len,   s_ethertypes.PPPoE_discov
#ifndef PASS_PROC_EOAM
    // Mark PPPoE packet and then pass through
   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_PPPoE
#endif    // PASS_PROC_EOAM
   set  s_l1View3bMac.pktFlags.fPPPoE //no match required

l_c1ProcessTagOrLen7:
#ifndef PASS_PROC_EOAM
    // Unknown ethertype  proceed to lookup with next header type unknown.
    mov s_next.Hdr,         PA_HDR_UNKNOWN
    // 802.1ag detection
    qbbc    l_c1ProcessTagOrLen8, s_runCxt.flag2.t_802_1agDet
#endif     // PASS_PROC_EOAM
        // Common rule (draft and standard) 01 80 C2 XX XX XX or 01 80 C2 00 00 3X
        mov     r0.w0,  ETH_TYPE_802_1AG
        qbne    l_c1ProcessTagOrLen8,   s_tagOrLen.len, r0.w0   
#ifndef PASS_PROC_EOAM        
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_01.b1,    0x01  
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_01.b0,    0x80
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_23.b1,    0xc2
//#ifndef PASS_PROC_EOAM
        // draft or standard
        qbbc    l_c1ProcessTagOrLen7_1, s_runCxt.flag2.t_802_1agStd 
//#endif  // PASS_PROC_EOAM      
            qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_23.b0,    0x00
            qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_45.b1,    0x00
            and     r0.b0,  s_macAddr.DstAddr_45.b0,    0xF0
            qbne    l_c1ProcessTagOrLen8,   r0.b0,   0x30
        
l_c1ProcessTagOrLen7_1:
#endif
        // 802.1ag packet detected      
#ifndef PASS_PROC_EOAM 
        //   Note: save one instruction since pl1Match is not important for exception route
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1, EROUTE_802_1ag << PKT_EIDX_SHIFT
        jmp  l_c1ProcessTagOrLen9_2
#else
        // Record the opcde and meg level in packet context
        set  s_pktCxt6.eoamFlags.t_flag_eType8902
.using eoamScope        
        // Store MEG Level and Opcode in case we need EOAM routing
        xin  XID_CDEDATA,  s_Eoam,   SIZE(s_Eoam)
        and  r1.b0, s_Eoam.megVer,  EOAM_VER_MASK
        // If version is not zero, it is a invalid EOAM packet and no need
      // to work on deciding if it is a known type or not
      qbeq l_c1ProcessTagOrLen7_2_0, r1.b0, 0
        set  s_pktCxt6.eoamFlags.t_flag_invalidVer
        jmp  l_c1ProcessTagOrLen7_6

l_c1ProcessTagOrLen7_2_0:
        lsr  s_pktCxt6.megLevel, s_Eoam.megVer, EOAM_MEL_SHIFT
        mov  s_pktCxt6.opcode,   s_Eoam.opcode

        qbne l_c1ProcessTagOrLen7_2,  s_Eoam.opcode, PA_EOAM_OPCODE_LMR
          set s_pktCxt6.eoamFlags.t_flag_knownOpcodes
          set s_pktCxt6.eoamFlags.t_flag_reportCount
          jmp l_c1ProcessTagOrLen7_6
          
l_c1ProcessTagOrLen7_2:
        qbne l_c1ProcessTagOrLen7_3,  s_Eoam.opcode, PA_EOAM_OPCODE_LMM
          set s_pktCxt6.eoamFlags.t_flag_knownOpcodes
          set s_pktCxt6.eoamFlags.t_flag_reportCount          
          jmp l_c1ProcessTagOrLen7_6

l_c1ProcessTagOrLen7_3:
        qbne l_c1ProcessTagOrLen7_4,  s_Eoam.opcode, PA_EOAM_OPCODE_IDM
          set s_pktCxt6.eoamFlags.t_flag_knownOpcodes
          jmp l_c1ProcessTagOrLen7_6          

l_c1ProcessTagOrLen7_4:
        qbne l_c1ProcessTagOrLen7_5,  s_Eoam.opcode, PA_EOAM_OPCODE_DMR
          set s_pktCxt6.eoamFlags.t_flag_knownOpcodes
          jmp l_c1ProcessTagOrLen7_6  

l_c1ProcessTagOrLen7_5:
        qbne l_c1ProcessTagOrLen7_6,  s_Eoam.opcode, PA_EOAM_OPCODE_DMM
          set s_pktCxt6.eoamFlags.t_flag_knownOpcodes
          // jmp l_c1ProcessTagOrLen7_6          

l_c1ProcessTagOrLen7_6:        
.leave eoamScope        
#endif // PASS_PROC_EOAM

l_c1ProcessTagOrLen8:
#ifndef PASS_PROC_EOAM
    xout XID_LUT1V3, s_l1View3bMac, SIZE(s_l1View3bMac) 
    ret
#else
.using  eoamScope
    // Parse through the Exception protocols to disable count for match
    // Load the protocol list from the exception table
    // Check with each exception tabled protocol to force disable the counts

    // mov  r1, OFFSET_EOAM_EXCEPTION_TBL
    // Load first two protocols to check
    lbco r1,  PAMEM_CONST_EOAM_EXC_TABLE, OFFSET_EOAM_EXCEPTION_TBL, 8
    qbeq l_c1ProcessEoamExcProcComplete, r1.w2, 0
      qbne l_c1ProcessEoamCheckProto1, r1.w2, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt       
        jmp l_c1ProcessEoamExcProcComplete

l_c1ProcessEoamCheckProto1:    
    qbeq l_c1ProcessEoamExcProcComplete, r1.w0, 0
      qbne l_c1ProcessEoamCheckProto2, r1.w0, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt    
        jmp l_c1ProcessEoamExcProcComplete    

l_c1ProcessEoamCheckProto2:
    // point to next two exception protocols
    //add  r1,  r1, 4
    //lbco r3,  PAMEM_CONST_EOAM_EXC_TABLE, r1, 4        
    qbeq l_c1ProcessEoamExcProcComplete, r2.w2, 0
      qbne l_c1ProcessEoamCheckProto3, r2.w2, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt   
        jmp l_c1ProcessEoamExcProcComplete

l_c1ProcessEoamCheckProto3:      
    qbeq  l_c1ProcessEoamExcProcComplete, r2.w0,0 
      qbne l_c1ProcessEoamCheckProto4, r2.w0, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt   
        jmp l_c1ProcessEoamExcProcComplete    

l_c1ProcessEoamCheckProto4:
    // point to next two exception protocols
    // add  r1,  r1, 8
    lbco r1,  PAMEM_CONST_EOAM_EXC_TABLE, OFFSET_EOAM_EXCEPTION_TBL + 8, 8
    qbeq l_c1ProcessEoamExcProcComplete, r1.w2, 0
      qbne l_c1ProcessEoamCheckProto5, r1.w2, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt    
        jmp l_c1ProcessEoamExcProcComplete

l_c1ProcessEoamCheckProto5:      
    qbeq  l_c1ProcessEoamExcProcComplete, r1.w0, 0 
      qbne l_c1ProcessEoamCheckProto6, r1.w0, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt   
        jmp l_c1ProcessEoamExcProcComplete    

l_c1ProcessEoamCheckProto6:
    // point to next two exception protocols
    // add  r1,  r1, 4
    // lbco r3,  PAMEM_CONST_EOAM_EXC_TABLE, r1, 4        
    qbeq  l_c1ProcessEoamExcProcComplete, r2.w2,0
      qbne l_c1ProcessEoamCheckProto1, r2.w2, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt   
        jmp l_c1ProcessEoamExcProcComplete

l_c1ProcessEoamCheckProto7:      
    qbeq  l_c1ProcessEoamExcProcComplete, r2.w0, 0
      qbne l_c1ProcessEoamExcProcComplete, r2.w0, s_tagOrLen.len
        set s_pktCxt6.eoamFlags.t_flag_fDisableCnt    
        //jmp l_c1ProcessEoamExcProcComplete    
    
l_c1ProcessEoamExcProcComplete:    
     jmp  fci_c1Parse14
.leave eoamScope    
#endif // PASS_PROC_EOAM

l_c1ProcessTagOrLen9:
    //  LLC/SNAP failure
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_LLC_SNAP_FAIL 

l_c1ProcessTagOrLen9_1:    
    //   Note: save one instruction since pl1Match is not important for exception route
    or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1, EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT

l_c1ProcessTagOrLen9_2:    
    mov  s_param.action,     SUBS_ACTION_EXIT
    ret 

#else
    // Error Handling
    mov  s_param.action,     SUBS_ACTION_EXIT
    ret 

#endif    // PASS_PROC_L2_PARSE

    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave macVlanScope

#ifdef PASS_PROC_L2_PARSE
// ************************************************************************************
// * FUNCTION PURPSE: Process Ipv4 packet in L2 Parsing
// ************************************************************************************
// * DESCRIPTION: Process the IP packet, (EOAM: Also record the RA specific information)
// *
// *    On entry:
// *                - CDE points at the IPV4 header
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *                - param.action has SUBS_ACTION_PARSE
// *
// *    On exit:
// *                - CDE points to the 1st byte after the ethertype (and llc/snap if present)
// *                - param.action has the next action to take
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:      |  checkScope  
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                     | 
// *   R7:        |                                     |
// *   R8:        |                                     |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***********************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  ipScope

f_l2Ipv4Proc: 

#ifndef PASS_PROC_EOAM
        // Store DSCP in case we need DSCP priority routing
        xin  XID_CDEDATA,  s_Ip,   OFFSET(s_Ip.checksum) 
        lsr s_pktCxt.priority, s_Ip.tos, 2
        mov s_next.Hdr,  PA_HDR_IPv4
        ret
#else
        xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 
        set  s_runCxt.flags.t_l4Avil  
        set  s_pktCxt6.eoamFlags.t_flag_ip 
        // Setup RA Info
        lbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
        mov  s_raInfo.l3Offset, s_pktCxt6.startOffsetEoam   

        // Fragmentation check
        mov   r3.w0,    IPV4_FRAG_MASK
        and   r3.w0,    s_Ip.fragOff,     r3.w0
        qbeq  l_l2Ipv4Proc_2,  r3.w0, 0
          clr  r3.w0.t_ipv4_frag_m
          qbeq l_l2Ipv4Proc_1, r3.w0, 0
          clr  s_runCxt.flags.t_l4Avil
         
l_l2Ipv4Proc_1: 
        set s_raInfo.flags.t_ra_flag_reasm  
        set s_pktCxt6.eoamFlags.t_flag_fragFound        
        
l_l2Ipv4Proc_2:
        wbs   s_flags.info.tStatus_CDEOutPacket
        sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
        
        // Check if it is ESP header as next Header, if so set Cipher Pkt Detected
        qbne l_l2Ipv4Proc_3, s_Ip.protocol, IP_PROTO_NEXT_ESP
          set s_pktCxt6.eoamFlags.t_flag_cipherPkt
          
l_l2Ipv4Proc_3:
        qbne l_l2Ipv4Proc_4, s_Ip.protocol, IP_PROTO_NEXT_AUTH
          set s_pktCxt6.eoamFlags.t_flag_cipherPkt 
          
l_l2Ipv4Proc_4:

        // Check if NAT-T detection is enabled, if yes treat the Pkt as Cipher if matching port found
        qbbc  l_l2Ipv4Proc_5,  s_runCxt.flag2.t_IpsecNatTDetEnEoam 

          add  r0.w0, s_pktCxt6.startOffsetEoam, SIZE(s_Ip)
         
          // Read in the UDP header
          add     r0.w0, r0.w0, 32+32+8
          lbco    r1,  cCdeInPkt, r0.w0, 4
        
          // A switch to IPSEC_NAT_T mode is made based on destination or source UDP port
          lbco  r0.w0,  PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG + OFFSET(struct_ipsec_nat_t_cfg.udpPort),  SIZE(struct_ipsec_nat_t_cfg.udpPort)
          qbeq  l_l2Ipv4Proc_5_0, r1.w0,  r0.w0 
          qbne  l_l2Ipv4Proc_5,   r1.w2,  r0.w0
l_l2Ipv4Proc_5_0:          
            set s_pktCxt6.eoamFlags.t_flag_cipherPkt 
            
l_l2Ipv4Proc_5:        
        jmp fci_c1Parse14
#endif
    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave ipScope

// ************************************************************************************
// * FUNCTION PURPSE: Process Ipv6 packet in L2 Parsing
// ************************************************************************************
// * DESCRIPTION: Process the IP packet, (EOAM: Also record the RA specific information)
// *
// *    On entry:
// *                - CDE points at the IPV4 header
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *                - param.action has SUBS_ACTION_PARSE
// *
// *    On exit:
// *                - CDE points to the 1st byte after the ethertype (and llc/snap if present)
// *                - param.action has the next action to take
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:      |  checkScope  
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                     | 
// *   R7:        |                                     |
// *   R8:        |                                     |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***********************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  ipScope

#ifdef PASS_PROC_EOAM
// Local structure defintion and assignment    
.struct struct_ipv6Frag_ctrl
  .u8    nextHdr
  .u8    hdrLen
  .u16   offset
  .u16   maxOffset
  .u16   rsvd
.ends

.assign struct_ipv6Frag_ctrl,  r0,     r1,    s_ipv6Frag_ctrl
#endif

#define PA_EOAM_MAX_HDR_LEN                   256

f_l2Ipv6Proc:
#ifndef PASS_PROC_EOAM
        // Store DSCP in case we need DSCP priority routing
        xin  XID_CDEDATA,  s_Ipv6a,   OFFSET(s_Ipv6a.payloadLen)
        lsr  s_pktCxt.priority,   s_Ipv6a.ver_tclass_flow.w2,  4
        lsr  s_pktCxt.priority,   s_pktCxt.priority,           2
        mov s_next.Hdr,  PA_HDR_IPv6
        ret
#else
        // Read in the IPv6 header.
        xin  XID_CDEDATA,  s_Ipv6a,   SIZE(s_Ipv6a)
        
        set  s_pktCxt6.eoamFlags.t_flag_ip
        set  s_runCxt.flags.t_l4Avil 

        // Setup RA Info
        lbco s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
        mov  s_raInfo.l3Offset, s_pktCxt6.startOffsetEoam  

        add  s_raInfo.ipv6NextOffset, s_pktCxt6.startOffsetEoam, OFFSET(s_Ipv6a.next)
        add  s_pktCxt6.startOffsetEoam,   s_pktCxt6.startOffsetEoam,  IPV6_HEADER_LEN_BYTES
   
        qbbs  l_l2Ipv6Proc_1_0, s_pktCxt.flags.t_flag_cascaded_forwarding
        qbbc  l_l2Ipv6Proc_1_0, s_runCxt.flag2.t_raEn
          wbs   s_flags.info.tStatus_CDEOutPacket
          sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
//           jmp  l_l2Ipv6Proc_1
l_l2Ipv6Proc_1_0: 
   
        // Pass 1 operation
        // Fragmentation detection: Is there fragment header?
        // Get to nextHdr field in IPv6 header
        // Load nextHdr and hdrLen field
        // Check to see if nextHdr is an extension header
        zero    &s_ipv6Frag_ctrl,     SIZE(s_ipv6Frag_ctrl)
        mov     s_ipv6Frag_ctrl.nextHdr,  s_Ipv6a.next 
        add     s_ipv6Frag_ctrl.offset,   s_pktCxt6.startOffsetEoam, (32 + 40) 
        mov     s_ipv6Frag_ctrl.maxOffset, (32 + PA_EOAM_MAX_HDR_LEN)
            
        //mov     s_ipv6Frag_ctrl.hdrLen,   IPV6_HEADER_LEN_BYTES

        //  Advance past the ipv6 main header
        //  Advance past the fisr 8 byte (Ipv6a).
        zero &s_cdeCmdWd,            SIZE(s_cdeCmdWd)
        mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,   SIZE(s_Ipv6a)                     // Bytes always present in the header
        xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)

l_l2Ipv6Proc_exthdrCheck:
        qble    l_l2Ipv6Proc_exthdrDone,  s_ipv6Frag_ctrl.offset,  s_ipv6Frag_ctrl.maxOffset    
        qbeq    l_l2Ipv6Proc_fragfound,   s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_FRAG      
        qbeq    l_l2Ipv6Proc_exthdrNext,  s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_HOP_BY_HOP
        qbeq    l_l2Ipv6Proc_exthdrNext,  s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_ROUTE
        qbeq    l_l2Ipv6Proc_exthdrNext,  s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_DEST_OPT
//   below would save one cycle since jmp is to header done.
//      qbeq    l_l2Ipv6Proc_exthdrDone,  s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_NO_NEXT
        jmp     l_l2Ipv6Proc_exthdrDone   

l_l2Ipv6Proc_exthdrNext:    
        // load the next header
        lbco    s_ipv6Frag_ctrl.nextHdr,  cCdeInPkt, s_ipv6Frag_ctrl.offset, 2

        // Check for hdr size greater than 255
        qble    l_l2Ipv6Proc_exthdrDone, s_ipv6Frag_ctrl.hdrLen, 63

        // adjust the offset for next one
        add     s_ipv6Frag_ctrl.hdrLen,    s_ipv6Frag_ctrl.hdrLen, 1
        lsl     s_ipv6Frag_ctrl.hdrLen,    s_ipv6Frag_ctrl.hdrLen, 3
        add     s_ipv6Frag_ctrl.offset,    s_ipv6Frag_ctrl.offset,  s_ipv6Frag_ctrl.hdrLen
        mov     s_raInfo.ipv6NextOffset,   s_pktCxt6.startOffsetEoam
        add     s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, s_ipv6Frag_ctrl.hdrLen        
        jmp     l_l2Ipv6Proc_exthdrCheck
    
l_l2Ipv6Proc_fragfound:
        qbbs  l_c1L2ParseIpv6ExtFrag00, s_pktCxt.flags.t_flag_cascaded_forwarding
        qbbc  l_c1L2ParseIpv6ExtFrag00, s_runCxt.flag2.t_raEn
          mov  s_raInfo.ipv6FragOffset, s_pktCxt6.startOffsetEoam
          set  s_raInfo.flags.t_ra_flag_reasm
          set  s_pktCxt6.eoamFlags.t_flag_fragFound            
          //wbs  s_flags.info.tStatus_CDEOutPacket
          //sbco s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
l_c1L2ParseIpv6ExtFrag00: 
          add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
        // The fragmentation header is always 8 bytes. The whole 
        // header is read here, but only the 1st byte is used
        lbco    s_ipv6Frag_ctrl.nextHdr,  cCdeInPkt, s_ipv6Frag_ctrl.offset, 1
   
        lsr   r3.w0,   s_Ipv6Frag.fragnFlag,     IPV6_FRAG_OFF_SHIFT
        qbeq  l_l2Ipv6Proc_ExtFrag1, r3.w0, 0
        //    clr  s_l1View3bIpv6.pktFlags.fL3IpContainL4
            clr  s_runCxt.flags.t_l4Avil
l_l2Ipv6Proc_ExtFrag1:
         // mov   s_raInfo.ipv6FragOffset,   s_ipv6Frag_ctrl.offset          
         // set   s_raInfo.flags.t_ra_flag_reasm
         // set   s_pktCxt6.eoamFlags.t_flag_fragFound  

l_l2Ipv6Proc_exthdrDone: 
         qbbc  l_l2Ipv6Proc_1, s_runCxt.flag2.t_raEn
           wbs   s_flags.info.tStatus_CDEOutPacket
           sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
  
          //pass through
        
l_l2Ipv6Proc_1: 
        // Check if it is ESP header as next Header, if so set Cipher Pkt Detected
        qbne l_l2Ipv6Proc_2, s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_ESP
          set s_pktCxt6.eoamFlags.t_flag_cipherPkt
          
l_l2Ipv6Proc_2:   
        // Check if it is AH header as next Header, if so set Cipher Pkt Detected
        qbne l_l2Ipv6Proc_3, s_ipv6Frag_ctrl.nextHdr, IP_PROTO_NEXT_AUTH
          set s_pktCxt6.eoamFlags.t_flag_cipherPkt 
          
l_l2Ipv6Proc_3:  

        // Check if NAT-T detection is enabled, if yes treat the Pkt as Cipher if matching port found
        qbbc  l_l2Ipv6Proc_4,  s_runCxt.flag2.t_IpsecNatTDetEnEoam        
        
          // Read in the UDP PORTs
          lbco    r1,  cCdeInPkt, s_ipv6Frag_ctrl.offset, 4       
  
          // A switch to IPSEC_NAT_T mode is made based on destination or source UDP port
          lbco  r0.w0,  PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG + OFFSET(struct_ipsec_nat_t_cfg.udpPort),  SIZE(struct_ipsec_nat_t_cfg.udpPort)
          qbeq  l_l2Ipv6Proc_4_0,  r1.w0,  r0.w0 
          qbne  l_l2Ipv6Proc_4, r1.w2,  r0.w0
l_l2Ipv6Proc_4_0:
            set s_pktCxt6.eoamFlags.t_flag_cipherPkt 
            
l_l2Ipv6Proc_4:
        jmp fci_c1Parse14

#endif
        
    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave ipScope

#endif    
// ***********************************************************************************
// * FUNCTION PURPOSE: Parse VLAN tags
// ***********************************************************************************
// * DESCRIPTION: Vlan tags are a special case for maximum depth check. Unlike
// *              GRE and IP, a lookup is not performed after a vlan tag is found,
// *              so the max depth must be checked each time a tag is found.
// *
// *    On entry:
// *                - CDE points to the VLAN tag
// *                - param.action has SUBS_ACTION_PARSE
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *                - r30.w0 has the return address
// *
// *    On exit:
// *                - CDE points to the 1st byte after the VLAN tag
// *                - param.action has the next action to take  
// *                  jmp back to f_c1ProcessTagOrLen until a ethertype is recognized 
// *                - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:      |  checkScope  
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  (packet desc) before read           |
// *   R7:        |                in MAC                |  mac addres (macVlanScope)
// *   R8:        |                                      |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// *
// *
// ***********************************************************************************
    .using pktScope
    .using checkScope
    .using cdeScope
    .using lut1Scope
    .using macVlanScope

f_c1ParseVlan:
#ifdef PASS_PROC_L2_PARSE
#ifndef PASS_PROC_EOAM
  set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_VLAN

  // Check agains the max
  lsr  r0.w0,   s_pktCxt.protCount, PROT_COUNT_VLAN_SHIFT
  lbco  s_paMaxHdrCount.vlanMaxCount,  PAMEM_CONST_CUSTOM,  OFFSET_MAX_HDR + OFFSET(s_paMaxHdrCount.vlanMaxCount), 2

  qbgt l_c1ParseVlan1,  r0.w0,  s_paMaxHdrCount.vlanMaxCount
      or     s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_VLAN_MAX_DEPTH << PKT_EIDX_SHIFT
      mov s_param.action, SUBS_ACTION_EXIT
      ret

l_c1ParseVlan1:
  // Inc the vlan count
  add  s_pktCxt.protCount,   s_pktCxt.protCount,    PROT_COUNT_VLAN_STEP
#endif

  // Read in the pri/cfi/vlanId fields. Mask out pri/cfi
  xin  XID_CDEDATA,         s_vtag.tag,           SIZE(s_vtag.tag)

#ifndef PASS_PROC_EOAM  
  // record vlan priority 
  and  s_pktCxt.vlanPri_vLink.b1,    s_vtag.tag.b1, PKT_VALN_PRI_MASK
#endif

  qbbc l_c1ParseVlan2, s_l1View3bMac.pktFlags.fL2Vlan1
    // VLAN2 always appear in front of VLAN1
    mov  s_l1View3bMac.vlanId1,     s_vtag.tag
    and  s_l1View3bMac.vlanId1.b1,  s_l1View3bMac.vlanId1.b1,   0xf
    lsr  s_l1View3bMac.vlanPri1,    s_vtag.tag, 13       
    //xout XID_LUT1V1,          s_l1View1a.VLAN,      SIZE(s_l1View1a.VLAN)
    jmp  l_c1ParseVlan3
  
l_c1ParseVlan2: 
    // VLAN1 operation
    mov  s_l1View3bMac.vlanId2,     s_vtag.tag
    and  s_l1View3bMac.vlanId2.b1,  s_l1View3bMac.vlanId2.b1,   0xf
    lsr  s_l1View3bMac.vlanPri2,    s_vtag.tag, 13
    
    // pass through       
l_c1ParseVlan3:
#ifndef PASS_PROC_EOAM
  add s_pktCxt.startOffset, s_pktCxt.startOffset, SIZE(s_vtag.tag)
#else
  add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, SIZE(s_vtag.tag)
#endif
  // Scroll the CDE past the tag to point to the next ethertype candidate
  mov  s_cdeCmdWd.byteCount,  SIZE(s_vtag.tag)
  xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)

  jmp f_c1ProcessTagOrLen

#endif      
    .leave pktScope
    .leave checkScope
    .leave cdeScope
    .leave lut1Scope
    .leave macVlanScope
    
// ***************************************************************************************
// * FUNCTION PURPOSE: Parse MPLS headers
// ***************************************************************************************
// * DESCRIPTION: An MPLS label stack is processed. If there is more then one label in
// *              the stack then the packet is sent immediately to LUT1 with the top
// *              label in the LUT. If there is only one lable then parsing continues
// *              after checking that the next header protocol COULD be Ipv4 or IPv6
// *
// *        On Entry
// *            -  CDE points to start of MPLS header
// *            -  param.action set to SUBS_ACTION_PARSE
// *            -  next.hdr set to PA_HDR_MPLS
// *        
// *        On Exit
// *            -  CDE points to 1st byte after 1st MPLS tag
// *            -  param.action is (either SUBS_ACTION_PARSE ) or SUBS_ACTION_LOOKUP
// *            -  next.hdr set to either PA_HDR_IPv4, PA_HDR_IPv6 or PA_HDR_UNKNOWN
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  (packet desc) before read           |
// *   R7:        |                in MAC                |  mac addres (macVlanScope)
// *   R8:        |                                      |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using mplsScope

f_c1ParseMpls:
#ifdef PASS_PROC_L2_PARSE

#ifndef PASS_PROC_EOAM
   set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_MPLS
   mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_MPLS_PKTS
#endif
   xin  XID_CDEDATA,  s_mpls,  SIZE(s_mpls)   // Read in a single tag

   // The tag is the 20 MSBs of the 32 bit word
   lsr  s_l1View2cMac.mpls,  s_mpls.tagTtl,  12
   xout XID_LUT1V2,          s_l1View2cMac.mpls,  SIZE(s_l1View2cMac.mpls)

   set  s_l1View3bMac.pktFlags.fMpls
   
#ifndef PASS_PROC_EOAM   
   add  s_pktCxt.startOffset,   s_pktCxt.startOffset,  SIZE(s_mpls.tagTtl)
#else
   add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, SIZE(s_mpls.tagTtl)
#endif

   mov   s_cdeCmdWd.byteCount,  SIZE(s_mpls.tagTtl)
   xout  XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)

   // Always perform a lookup after finding an MPLS headeer
   mov  s_param.action,  SUBS_ACTION_LOOKUP

   // Extract the S bit. If set then get a potential IP version number
   // For possible IPv4 and IPv6 continue parse

   qbbc  l_c1ParseMpls1,  s_mpls.tagTtl.t_s
       lsr  r0.b0,  s_mpls.ipVer,   4
       qbne l_c1ParseMpls0,  r0.b0,   4
           //mov   s_next.hdr,          PA_HDR_IPv4
           //ret
           jmp  f_l2Ipv4Proc

l_c1ParseMpls0:
       qbne l_c1ParseMpls1,  r0.b0,   6
           // mov   s_next.hdr,          PA_HDR_IPv6
           // ret
           jmp  f_l2Ipv6Proc
    
     
    // If there is more then one label or there is one label but the
    // next header doesnt look like IP, run the current packet through the LUT
l_c1ParseMpls1:
#ifndef PASS_PROC_EOAM
    mov  s_next.hdr,      PA_HDR_UNKNOWN
    ret
#else
    jmp fci_c1Parse14
#endif
    
#endif    //PASS_PROC_L2_PARSE
      
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave mplsScope

// ***************************************************************************************
// * FUNCTION PURPOSE: Parse PPPoE headers
// ***************************************************************************************
// * DESCRIPTION: An PPPoE stack is processed. 
// *              Check that the PPP protocol number, Set next hdr to unknown if it is
// *              not IPv4 or IPv6 
// *
// *        On Entry
// *            -  CDE points to start of PPPoE header
// *            -  param.action set to SUBS_ACTION_PARSE
// *            -  next.hdr set to PA_HDR_PPPoE
// *        
// *        On Exit
// *            -  CDE points to start of PPPoE header
// *            -  startOffset is set to the start of IP header if IPv4 or IPv6
// *               Otherwise, it is set to the 1st byte after PPPoE header 
// *            -  param.action is either SUBS_ACTION_EXIT  or SUBS_ACTION_LOOKUP
// *            -  next.hdr set to either PA_HDR_IPv4, PA_HDR_IPv6 or PA_HDR_UNKNOWN
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  (packet desc) before read           |
// *   R7:        |                in MAC                |  mac addres (macVlanScope)
// *   R8:        |                                      |  Note: Should not be overwritten by other prorocol header
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  | 
// *   R19:          |                                  |  ethertypes table
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using pppoeScope

f_c1ParsePPPoE:

#ifdef PASS_PROC_L2_PARSE
   xin  XID_CDEDATA,  s_pppoe,  SIZE(s_pppoe)   // Read in the PPPoE header
   
   // Protocol header error check
   qbbc  l_c1ParsePPPoE_0, s_runCxt.flag2.t_PPPoEHdrCheck
     qbne  l_c1ParsePPPoE_3, s_pppoe.verType, PPPoE_VER_TYPE
     qbne  l_c1ParsePPPoE_3, s_pppoe.code,    PPPoE_CODE_SESSION
   
l_c1ParsePPPoE_0:
#ifndef PASS_PROC_EOAM
   // Mark PPPoE
   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_PPPoE
#endif   
   set  s_l1View3bMac.pktFlags.fPPPoE
   
    mov  s_l1View1aMac.PPPoE_SessionId,  s_pppoe.sessionId
    xout XID_LUT1V1, s_l1View1aMac.PPPoE_SessionId,  SIZE(s_l1View1aMac.PPPoE_SessionId)
   
#ifndef PASS_PROC_EOAM  
   add s_pktCxt.startOffset,    s_pktCxt.startOffset,  OFFSET(s_pppoe.prot)
   add s_pktCxt.endOffset,      s_pktCxt.startOffset,  s_pppoe.len
#else
   add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, OFFSET(s_pppoe.prot)
#endif

   // Always perform a lookup after finding an PPPoE headeer
   mov  s_param.action,  SUBS_ACTION_LOOKUP

   // Find out the next prot
   mov   r0.w0, PPPoE_PROT_IPv4           

   qbne  l_c1ParsePPPoE_1,  s_pppoe.prot, r0.w0
#ifndef PASS_PROC_EOAM    
       add   s_pktCxt.startOffset,    s_pktCxt.startOffset, SIZE(s_pppoe.prot)
#else
       add   s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, SIZE(s_pppoe.prot)
#endif       
       //mov   s_next.hdr,          PA_HDR_IPv4
       //ret
       jmp  f_l2Ipv4Proc

l_c1ParsePPPoE_1:
   mov   r0.w0, PPPoE_PROT_IPv6
   qbne  l_c1ParsePPPoE_2,  s_pppoe.prot, r0.w0
#ifndef PASS_PROC_EOAM    
       add   s_pktCxt.startOffset,    s_pktCxt.startOffset, SIZE(s_pppoe.prot)
#else
       add  s_pktCxt6.startOffsetEoam, s_pktCxt6.startOffsetEoam, SIZE(s_pppoe.prot)
#endif       
       // mov   s_next.hdr,          PA_HDR_IPv6
       // ret
       jmp  f_l2Ipv6Proc       
     
    // Unsupported protocol
    // next header does not look like IP;  Exception route
l_c1ParsePPPoE_2:
#ifndef PASS_PROC_EOAM
    or     s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1, EROUTE_PPPoE_CTRL << PKT_EIDX_SHIFT      
#endif    
    jmp l_c1ParsePPPoE_4
    
    // PPPoE header Error: Exception route
l_c1ParsePPPoE_3:
#ifndef PASS_PROC_EOAM
    or     s_pktCxt.eId_portNum_nextHdr.b1, s_pktCxt.eId_portNum_nextHdr.b1, EROUTE_PPPoE_FAIL << PKT_EIDX_SHIFT      
    // pass through
l_c1ParsePPPoE_4:    
    // Common Error handling 
    mov s_param.action, SUBS_ACTION_EXIT
    mov  s_next.hdr,    PA_HDR_UNKNOWN
    ret
#else
l_c1ParsePPPoE_4:  
    jmp fci_c1Parse14
#endif
#endif    
      
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave pppoeScope
    
// *********************************************************************************
// *********************************************************************************
// * FUNCTION PURPOSE: Parsing of SRIO header information
// *********************************************************************************
// * DESCRIPTION: The 1st two 32 bit words in the control data contain L0-L2 info. 
// *
// *   On entry:
// *            - CDE points to the SRIO header (Control Info section)
// *            - r30.w0 has the return address
// *            - cdeCmdWd.operation has is not set
// *            - param.action is not set
// *            - s_runCxt has valid packet context
// *            - r0.b0 packet Type (SRIO type 9 or 11 only)
// *
// *   On exit:
// *            - CDE points to the first byte of packet
// *            - cdeCmdWd.operation has CDE_CMD_FLUSH_TO_PACKET
// *            - param.action has SUBS_ACTION_LOOKUP
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    r1.b0: packet type
// *   R2:    scratch
// *   R3:               
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:                                              
// *   R7:                                              
// *   R8:                                              
// *   R9:          
// *   R10:       |
// *   R11:       |  LUT1 View3a   - lut1Scope
// *   R12:       |
// *   R13:       |
// *   R14:          |  SRIO header L0-L2                                        
// *   R15:          |  SRIO header L0-L2                                        
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
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// **********************************************************************************/

    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  srioScope

f_c1ParseSrio:

#ifdef PASS_PROC_FIREWALL
    jmp fci_c1FirewallException
#endif

#ifdef PASS_PROC_L2

  mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_SRIO_PKTS

  // Load the SRIO L0-L2 info: assuming type 11 
  xin  XID_CDEDATA,   s_srio11,  SIZE(s_srio11)
  
  // Delete the control information. It will be replace with the packet context during parse
  mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
  xout  XID_CDECTRL,           s_cdeCmdWd,       SIZE(s_cdeCmdWd)
  
  mov  s_param.action, SUBS_ACTION_LOOKUP

  // Clear the LUT1 view areas; Only Lut1V3 is used  
  zero &s_l1View3bSrio,     SIZE(s_l1View3bSrio)
  set   s_l1View3bSrio.pktType.fL2PktTypeSrio
  
  qbeq  l_c1ParseSrioType9, r1.b0, PA_PKT_TYPE_SRIO_TYPE_9

l_c1ParseSrioType11:
    // Type 11 Message Lookup
    set  s_l1View3bSrio.pktFlags.fSrioType11
    
    qbbc    l_c1ParseSrioType11_1,  s_srio11.ctrl.b1.t_srio_type11_t_port16
        // 16-bit device 
        //clr s_l1View3bSrio.pktFlags.fSrioPort8
        mov s_l1View3bSrio.srcId,   s_srio11.srcId
        mov s_l1View3bSrio.dstId,   s_srio11.dstId
        jmp l_c1ParseSrioType11_2
        
 l_c1ParseSrioType11_1:        
        // 8-bit device 
        set s_l1View3bSrio.pktFlags.fSrioPort8
        and s_l1View3bSrio.srcId,   s_srio11.srcId,     0xFF
        and s_l1View3bSrio.dstId,   s_srio11.dstId,     0xFF
        
 l_c1ParseSrioType11_2:    
   
   and  s_l1View3bSrio.typeParam1, s_srio11.ctrl.b0,  SRIO_TYPE11_MBOX_MASK
   and  r1.b0,              s_srio11.ctrl.b1,   SRIO_TYPE11_PRI_MASK
   lsr  s_l1View3bSrio.pri, r1.b0,              SRIO_TYPE11_PRI_SHIFT   
   lsr  s_l1View3bSrio.typeParam2,   s_srio11.ctrl.w0,     SRIO_TYPE11_LTR_SHIFT
   and  s_l1View3bSrio.typeParam2,   s_l1View3bSrio.typeParam2, SRIO_TYPE11_LTR_MASK >> SRIO_TYPE11_LTR_SHIFT
   lsr  s_l1View3bSrio.cc,   s_srio11.ctrl.w1,  SRIO_TYPE11_CC_SHIFT
   and  s_l1View3bSrio.cc,   s_l1View3bSrio.cc, SRIO_TYPE11_CC_MASK >> SRIO_TYPE11_CC_SHIFT              
   xout XID_LUT1V3,         s_l1View3bSrio,     SIZE(s_l1View3bSrio)
   
   ret
   
l_c1ParseSrioType9:
    // Type 9 Message Lookup
    set  s_l1View3bSrio.pktFlags.fSrioType9
    
    qbbc l_c1ParseSrioType9_1,   s_srio9.ctrl.t_srio_type9_t_port16
        // 16-bit device 
        //clr s_l1View3bSrio.pktFlags.fSrioPort8
        mov s_l1View3bSrio.srcId,   s_srio9.srcId
        mov s_l1View3bSrio.dstId,   s_srio9.dstId
        jmp l_c1ParseSrioType9_2
        
 l_c1ParseSrioType9_1:        
        // 8-bit device 
        set s_l1View3bSrio.pktFlags.fSrioPort8
        and s_l1View3bSrio.srcId,   s_srio9.srcId,     0xFF
        and s_l1View3bSrio.dstId,   s_srio9.dstId,     0xFF
        
 l_c1ParseSrioType9_2:    
   mov  s_l1View3bSrio.typeParam1, s_srio9.streamId
   and  r1.b0,              s_srio9.ctrl,       SRIO_TYPE9_PRI_MASK
   lsr  s_l1View3bSrio.pri, r1.b0,              SRIO_TYPE9_PRI_SHIFT   
   mov  s_l1View3bSrio.typeParam2,   s_srio9.cos
   and  s_l1View3bSrio.cc,  s_srio9.ctrl,       SRIO_TYPE9_CC_MASK              
   xout XID_LUT1V3,         s_l1View3bSrio,     SIZE(s_l1View3bSrio)
   
   ret
   
#else
    // Error Handling
    mov  s_param.action,     SUBS_ACTION_EXIT
    ret 

#endif  
   
    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave srioScope
    
// ************************************************************************************
// * FUNCTION PURPOSE: Parse a custom LUT1 header
// ************************************************************************************
// * DESCRIPTION:
// *
// *    On entry:
// *            - the CDE is at the start of the Custom header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - the CDE is at the first byte after Custom offset
// *            - startOffset is adjusted based on nextHdr Offset
// *            - s_next.Hdr is set as custHdr.nextHdr
// *            
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        |  LUT1 View1&2   - lut1Scope
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |  CustomHdr & Custom                                      
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |                   
// *   R18:          |                    |  LUT1 View3
// *   R19:          |                    |
// *   R20:          |                    |
// *   R21:          |                    |
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
// *
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using lut1Scope
    .using customC1Scope

f_c1ParseCustom:

#ifdef PASS_PROC_LUT1_CUSTOM

  set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_CUSTOM
  mov  s_pktCxt.l3Offset,  s_pktCxt.startOffset
  mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_CUSTOM_PKTS

  // The previous lookup PDSP ID and index is stored in the sourceVC (Virtual Link) field
  zero &s_l1View3dCustom,     SIZE(s_l1View3dCustom)  
  mov  s_l1View3dCustom.srcVC,  s_pktCxt.phyLink

  // eIndex should stote the custom Index
  // offset = index * 40 + OFFSET_CUSTOM_C1
  // TBD: Should we do error check here?
  lsr   r1.b0,  s_pktCxt.eId_portNum_nextHdr.b1,  PKT_EIDX_SHIFT 
  mov   s_l1View3dCustom.pktType,   r1.b0  // record the custom index as part of matching crierta 
  set   s_l1View3dCustom.pktType.fL2PktTypeCustom
  lsl   r1.w2,  r1.b0,  5
  lsl   r1.w0,  r1.b0,  3
  add   r1.w0,  r1.w0,  r1.w2
  add   r1.w0,  r1.w0,  OFFSET_CUSTOM_C1 
  
  xout  XID_LUT1V3,  s_l1View3dCustom,  SIZE(s_l1View3dCustom)
  
  // find customer offset
  lbco  s_c1CustomHdr,  PAMEM_CONST_CUSTOM,  r1.w0,  SIZE(s_c1CustomHdr)

  mov  s_cdeCmdWd.byteCount,  s_c1CustomHdr.offset
  mov  s_next.Hdr,            s_c1CustomHdr.nextHdr
  xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
  
  add   s_pktCxt.startOffset, s_pktCxt.startOffset, s_c1CustomHdr.nextHdrOffset  
  
  // read in the customer masks
  add   r1.w0,  r1.w0,  SIZE(s_c1CustomHdr) 
  lbco  s_c1Custom,  PAMEM_CONST_CUSTOM,  r1.w0,  SIZE(s_c1Custom)

  // Read in the 32 bytes of the custom info (s_l1View1aCustom & s_l1View2bCustom)
  xin  XID_CDEDATA,   s_l1View1aCustom,   SIZE(s_l1View1aCustom) + SIZE(s_l1View2bCustom)   
  
  //  Relying on big endian data
  and   s_l1View1aCustom.match0,  s_l1View1aCustom.match0,  s_c1Custom.bitmask0
  and   s_l1View1aCustom.match1,  s_l1View1aCustom.match1,  s_c1Custom.bitmask1
  and   s_l1View1aCustom.match2,  s_l1View1aCustom.match2,  s_c1Custom.bitmask2
  and   s_l1View1aCustom.match3,  s_l1View1aCustom.match3,  s_c1Custom.bitmask3
  and   s_l1View2bCustom.match4,  s_l1View2bCustom.match4,  s_c1Custom.bitmask4
  and   s_l1View2bCustom.match5,  s_l1View2bCustom.match5,  s_c1Custom.bitmask5
  and   s_l1View2bCustom.match6,  s_l1View2bCustom.match6,  s_c1Custom.bitmask6
  and   s_l1View2bCustom.match7,  s_l1View2bCustom.match7,  s_c1Custom.bitmask7

  xout  XID_LUT1V1,  s_l1View1aCustom,  SIZE(s_l1View1aCustom)
  xout  XID_LUT1V2,  s_l1View2bCustom,  SIZE(s_l1View2bCustom)

  mov  s_param.action, SUBS_ACTION_LOOKUP
  ret
  
#else 

    // Error Handling
    mov  s_param.action,     SUBS_ACTION_EXIT
    ret 
 
#endif

    .leave cdeScope
    .leave pktScope
    .leave lut1Scope
    .leave customC1Scope

#ifdef PASS_PROC_LUT1_L4
// *************************************************************************************
// * FUNCTION PURPOSE: Parse an IPv4/IPv6 header when EOAM feature is enabled for the 2nd ACL
// *************************************************************************************
// * DESCRIPTION: The IPv4 source and destination address are placed into the LTU.
// *              The pseudo header checksum is computed, and any options are parsed
// *
// *    On entry:
// *            - the CDE is at the start of the IPv4 header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - the next action to take is in param.action
// *            - the IP addresses, next protocol and TOS are in the lut
// *            - the CDE points to the first byte after the IP header
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      | IPv4 header 
// *   R7:        |                                      |  
// *   R8:        |                                      |  
// *   R9:        |  LUT1 View   - lut1Scope             |
// *   R10:       |                                     || dst address
// *   R11:       |                                     |  LUT1 View1
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)      
// *   R15:          |                                    
// *   R16:          |                                    | raInfo     
// *   R17:          |  LUT1 View  - lut1Scope            |     
// *   R18:          |                                  |
// *   R19:          |                                  |  LUT1 View3
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// *
// *************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using ipScope
    
f_c1ParseIpForEoamAcl:

  // Read the minimum IP header
  add   r0, s_pktCxt.l3Offset, 32+32+8
  lbco   s_Ip,   cCdeInPkt, r0, SIZE(s_Ip)
  and   r0,               s_Ip.verLen,  0xF0
  qbeq  l_c1ParseIpvForEoamAcl_v4, r0,         0x40
  qbeq  l_c1ParseIpvForEoamAcl_v6, r0,         0x60
  jmp   fci_c1FirewallException      

l_c1ParseIpvForEoamAcl_v4:
  // Save src port and destination port information before clearing
  mov  r0.w0, s_l1View3dIpv4.srcPort
  mov  r0.w2, s_l1View3dIpv4.dstPort
  zero  &s_l1View3dIpv4,    SIZE(s_l1View3dIpv4)
  // Restore the information
  mov  s_l1View3dIpv4.srcPort, r0.w0
  mov  s_l1View3dIpv4.dstPort, r0.w2
  
  mov  s_l1View3dIpv4.srcVC,  s_pktCxt.phyLink  
  // Fragmentation check
  set  s_l1View3dIpv4.pktFlags.fL3IpContainL4
  mov   r3.w0,    IPV4_FRAG_MASK
  and   r3.w0,    s_Ip.fragOff,     r3.w0
  qbeq  l_c1ParseIpv4ForEoamAcl_1,  r3.w0,            0
      set  s_l1View3dIpv4.pktFlags.fL3IpFrag
      clr  r3.w0.t_ipv4_frag_m
      qbeq l_c1ParseIpv4ForEoamAcl_1, r3.w0, 0
        clr  s_l1View3dIpv4.pktFlags.fL3IpContainL4
         
l_c1ParseIpv4ForEoamAcl_1:   
  // dstIp already in the correct location
  mov   s_l1View1bIpv4.srcIp,   s_Ip.srcIp
  xout  XID_LUT1V1,             s_l1View1bIpv4,                  OFFSET(s_l1View1bIpv4.res32a)
  set  s_l1View3dIpv4.pktType.fL3PktTypeFirewall  
  set  s_l1View3dIpv4.pktFlags.fL3Ipv4
  mov  s_l1View3dIpv4.protocol,    s_Ip.protocol
  mov  s_l1View3dIpv4.pktFlags.b0, s_pktCxt.priority
  xout XID_LUT1V3,               s_l1View3dIpv4,  SIZE(s_l1View3dIpv4)     
  ret
     
l_c1ParseIpvForEoamAcl_v6:
   // Save src port and destination port information before clearing
   mov  r0.w0, s_l1View3bIpv6.srcPort
   mov  r0.w2, s_l1View3bIpv6.dstPort
   zero  &s_l1View3bIpv6,    SIZE(s_l1View3bIpv6)   
   // Restore the information
   mov  s_l1View3bIpv6.srcPort, r0.w0
   mov  s_l1View3bIpv6.dstPort, r0.w2
   
   mov  s_l1View3bIpv6.srcVC,  s_pktCxt.phyLink 
   mov  r0.w0, s_pktCxt.l3Offset   
   lbco   s_Ipv6a,   cCdeInPkt, r0.w0, SIZE(s_Ipv6a)	
   add  r0.w0, r0.w0, SIZE(s_Ipv6a) 
   lbco  s_Ipv6b,  cCdeInPkt, r0.w0, SIZE(s_Ipv6b)
   
   // Source and dest IP go out unchanged
   xout XID_LUT1V1,   s_l1View1cIpv6,   SIZE(s_l1View1cIpv6)
   xout XID_LUT1V2,   s_l1View2dIpv6,   SIZE(s_l1View2dIpv6)   
   // The protocol, tclass, and flow labels are moved as 
   // required
   mov   s_l1View3bIpv6.flowLabel, s_Ipv6a.ver_tclass_flow
   and   s_l1View3bIpv6.flowLabel.w2,  s_l1View3bIpv6.flowLabel.w2,   0x0f   
   set   s_l1View3bIpv6.pktType.fL3PktTypeFirewall

   mov   s_l1View3bIpv6.protocol,      s_Ipv6a.next
   lsr   s_l1View3bIpv6.pktFlags.b0,   s_Ipv6a.ver_tclass_flow.w2,  4
   and   s_l1View3bIpv6.pktFlags.b0,   s_l1View3bIpv6.pktFlags.b0, 0x3F
   
   xout  XID_LUT1V3,               s_l1View3bIpv6,         SIZE(s_l1View3bIpv6)  

   // Get the next header and action
   set    s_l1View3bIpv6.pktFlags.fL3IpContainL4     
   ret
     
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave ipScope    
#endif 

// *************************************************************************************
// * FUNCTION PURPOSE: Parse an IPv4 header
// *************************************************************************************
// * DESCRIPTION: The IPv4 source and destination address are placed into the LTU.
// *              The pseudo header checksum is computed, and any options are parsed
// *
// *    On entry:
// *            - the CDE is at the start of the IPv4 header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - the next action to take is in param.action
// *            - the IP addresses, next protocol and TOS are in the lut
// *            - the CDE points to the first byte after the IP header
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      | IPv4 header 
// *   R7:        |                                      |  
// *   R8:        |                                      |  
// *   R9:        |  LUT1 View   - lut1Scope             |
// *   R10:       |                                     || dst address
// *   R11:       |                                     |  LUT1 View1
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)      
// *   R15:          |                                    
// *   R16:          |                                    | raInfo     
// *   R17:          |  LUT1 View  - lut1Scope            |     
// *   R18:          |                                  |
// *   R19:          |                                  |  LUT1 View3
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// *
// *************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using ipScope

f_c1ParseIpv4:

#ifdef PASS_PROC_L3
 
#ifndef PASS_PROC_FIREWALL

  // Ipv4 reassembly operation
  qbbs fci_c1ParseIpv4_0,    s_pktCxt.flags.t_flag_cascaded_forwarding    
  qbbs f_c1IpReassm,         s_runCxt.flags.t_ipReassmEn   

fci_c1ParseIpv4_0:

#ifdef PASS_VERIFY_POST_RA_DROP
    wbs  s_flags.info.tStatus_CDEOutPacket
    lbco  r1.b0,  cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
    qbbc  l_c1ParseIpv4_ra_drop_end, r1.b0.t_psFlags_pktDropFromRA
        // drop the packet
        mov s_param.action, SUBS_ACTION_DISCARD
        ret
l_c1ParseIpv4_ra_drop_end:    
#endif    

  // New IPv4 packet or Reassembled packet: check maximum count
  lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.ipMaxCount), 2

  // IP depth count
  and  r0.w0,         s_pktCxt.protCount,  PROT_COUNT_IP_MASK
  qblt l_c1ParseIpv4_00,   r0.w2,  r0.w0
      // Setup for IP overflow exception route
      and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
      or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_IP_MAX_DEPTH << PKT_EIDX_SHIFT  

      mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_DEPTH_OVERFLOW
      mov s_param.action,        SUBS_ACTION_EXIT
      ret

l_c1ParseIpv4_00:  

  add  s_pktCxt.protCount,  s_pktCxt.protCount, PROT_COUNT_IP_STEP
  set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_IPv4
  mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IPV4_PKTS
  
#ifdef PASS_PROC_INNER_IP
  mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_INNER_IPV4_PKTS
  // store the inner IP offset
  mov s_pktCxt5.l3l5Offset, s_pktCxt.startOffset
#else
  // store the outer IP offset
  mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
#endif  

#ifdef TOBE_DELETED
  
  //
  // t_l3toInnerIp = FALSE: Set l3Offset for outer IP 
  // t_l3toInnerIp = TRUE:  Set l3Offset for inner IP
  //
  qbbs l_c1ParseIpv4_01_setInnerIp, s_runCxt.flag2.t_l3toInnerIp
    qbne l_c1ParseIpv4_01,    s_pktCxt.l3Offset, 0
l_c1ParseIpv4_01_setInnerIp:      
        mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
#endif        
        
#else

#ifdef PASS_DETECT_IP_PROC
 qbbc l_c1ParseIpv4_in3, s_runCxt.flag3.t_eoamEn
  set  s_pktCxt.flags.t_flag_ipProc
l_c1ParseIpv4_in3:
#endif

// Firewall (RA) specific processing
// TBD: priority settings
  lbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
  //zero &s_raInfo,   SIZE(s_raInfo)
  mov  s_raInfo.l3Offset, s_pktCxt.startOffset 
  //qbbc l_c1ParseIpv4_set_ra_threadId_1, s_runCxt.flag2.t_raToQueue
    // RA output is host queue
    //mov  s_raInfo.flags, THREADID_CDMA0
    //jmp  l_c1ParseIpv4_set_ra_threadId_end
//l_c1ParseIpv4_set_ra_threadId_1:  
  //  mov  s_raInfo.flags, PASS_RA_DEST_THREAD_ID 
    // pass through  
//l_c1ParseIpv4_set_ra_threadId_end:

#endif    

l_c1ParseIpv4_01:
  zero  &s_l1View3dIpv4,    SIZE(s_l1View3dIpv4)
   
#ifndef PASS_PROC_FIREWALL
  // Check to see whether virtual link is used 
  qbbs l_c1ParseIpv4_01_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l1View3dIpv4.srcVC,  s_pktCxt.phyLink
    jmp  l_c1ParseIpv4_01_link_end
l_c1ParseIpv4_01_vLink:    
    mov  s_l1View3dIpv4.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l1View3dIpv4.srcVC.b1, s_l1View3dIpv4.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c1ParseIpv4_01_link_end:

#else
    mov  s_l1View3dIpv4.srcVC,  s_pktCxt.phyLink
#endif


  // Read the minimum IP header
  xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 
  
  // Basic Error Check
  // - version = 4
  // - ipHdrLen >= 20 
  // - (ipOffset + ipLength) <= pktLen
  // version check
  and   r0,               s_Ip.verLen,  0xF0
  qbne  fci_c1ParseIpv4_10, r0,         0x40
  
  // Is ipHdrLength vaild?
  and  r0.b0,  s_Ip.verLen,   0x0F
  qbgt fci_c1ParseIpv4_10,    r0.b0,       IPV4_MIN_HDR_SIZE >> 2  // IP Hdr length check
  
  // Is ipLength valid?
  // TBD: The outer IP length is incorrect at our IP fragmentation test
#ifdef TO_BE_ADDED  
  add  r0.w0,  s_Ip.totalLen, s_pktCxt.startOffset
  qblt fci_c1ParseIpv4_10,    r0.w0,       s_pktCxt.endOffset      // IP length check
#endif  
  
#ifndef PASS_PROC_FIREWALL  
  // Strong error check required by some customers: (Disabled by default)
  // ipLength <= 20
  // src addr == 0xffffffff (broadcast)
  // dest addr == 0
  // ttl == 0
  
  qbbc  l_c1ParseIpv4_000,   s_runCxt.flag2.t_ipHdrCheck
    qbge fci_c1ParseIpv4_10, s_Ip.totalLen,  IPV4_MIN_HDR_SIZE     // Total length check
    qbeq fci_c1ParseIpv4_10,    s_Ip.dstIp,     0                  // Dest IP == 0
    qbeq fci_c1ParseIpv4_10,    s_Ip.ttl,       0                  // TTL == 0
    // IP Source IP check == 0xFFFFFFFF 
    // Method 1: Assignment plus two 16-bit comparison (3 instruction, 3 cycles)
    // Method 2: Search for zero and then check result (2 instruction, 2 cycles)
    lmbd r0,    s_Ip.srcIp,     0    
    qbeq fci_c1ParseIpv4_10,    r0,        32     
    
#endif                    
        
    // IPv4 Hdr check complete: pass through
l_c1ParseIpv4_000:
  // Fragmentation check
  set  s_l1View3dIpv4.pktFlags.fL3IpContainL4
  mov   r3.w0,    IPV4_FRAG_MASK
  and   r3.w0,    s_Ip.fragOff,     r3.w0
  qbeq  l_c1ParseIpv4_0,  r3.w0,            0
      set  s_l1View3dIpv4.pktFlags.fL3IpFrag
      clr  r3.w0.t_ipv4_frag_m
      qbeq l_c1ParseIpv4_000_1, r3.w0, 0
        clr  s_l1View3dIpv4.pktFlags.fL3IpContainL4
        clr  s_runCxt.flags.t_l4Avil
         
l_c1ParseIpv4_000_1: 
#ifndef PASS_PROC_RA
      set  s_pktCxt.flags.t_flag_frag
#else // PASS_PROC_RA
      mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG 
      set s_raInfo.flags.t_ra_flag_reasm   
#ifdef PASS_INNER_RA
      //set s_raInfo.flags.t_ra_flag_2nd_inst   
#endif         
#endif  
    
      // pass through

l_c1ParseIpv4_0:
  // dstIp already in the correct location
  mov   s_l1View1bIpv4.srcIp,   s_Ip.srcIp
  xout  XID_LUT1V1,             s_l1View1bIpv4,                  OFFSET(s_l1View1bIpv4.res32a)

#ifdef PASS_PROC_FIREWALL
  // TBD???: Read back in the IP header info (It should not be required anymore)
  // xin  XID_CDEDATA,    s_Ip,   SIZE(s_Ip)
  
  // Setup the CDE for the IP header checksum, set the global pointer
  // past the IP header
  zero &s_cdeCmdChk,            SIZE(s_cdeCmdChk)
  and   r2.w0,                  s_Ip.verLen,                    0x0f
  lsl   s_cdeCmdChk.byteLen,    r2.w0,                          2  // 32 bit size to 8 bit size conversion
  mov   s_cdeCmdChk.operation,  CDE_CMD_CHECKSUM1_VALIDATE
  xout  XID_CDECTRL,            s_cdeCmdChk,                    SIZE(s_cdeCmdChk)
  
#else
  // Recalculate IP checksum after IP 
  // Skip this operation if checksum error flag is set
  // wait for the sideband data to be ready 
  //wbc    s_flags.info.tStatus_CDEBusy
  //lbco   r0.b0,   cCdeInPkt,           OFFSET(struct_pktDscFine.psFlags_errorFlags),     1   
  //qbbs   l_c1ParseIpv4_cksum_skip, r0.b0.t_errorFlags_cksumIp          
    zero  &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
    mov   s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
    and   r2.w0,                s_Ip.verLen,              0x0f
    lsl   s_cdeCmdChk.byteLen,  r2.w0,                    2  // 32 bit size to 8 bit size conversion
    mov   s_cdeCmdChk.offset, OFFSET(s_Ip.Checksum)          // Starting sum is 0, offset is 10
    xout  XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
  
    mov   s_Ip.Checksum, 0
    xout  XID_CDEDATA, s_Ip.Checksum, 2
  
l_c1ParseIpv4_cksum_skip:  

#endif

#ifndef PASS_PROC_FIREWALL  

  // Update the start and end offsets while the IP header length is in bytes
  add   s_pktCxt.endOffset,     s_pktCxt.startOffset,           s_Ip.totalLen
  add   s_pktCxt.startOffset,   s_pktCxt.startOffset,           s_cdeCmdChk.byteLen

  // Compute the pseudo header checkusm except protocol and payload length
  add    r1,               s_Ip.srcIp,       s_Ip.dstIp
  adc    r1,               r1.w0,            r1.w2
  add    s_pktCxt.pseudo,  r1.w0,            r1.w2
  
  // Broadcast and Multicast IP detection
l_c1ParseIpv4_BM:  
   // IP Destination IP broadcast == 0xFFFFFFFF 
   // Method 1: Assignment plus two 16-bit comparison (3 instruction, 3 cycles)
   // Method 2: Search for zero and then check result (2 instruction, 2 cycles)
  //mov    r1.w0,               0xffff
  //qbne   l_c1ParseIpv4_BM_1,  s_Ip.dstIp.w2,   r1.w0
  //qbne   l_c1ParseIpv4_BM_1,  s_Ip.dstIp.w0,   r1.w0
    lmbd r1.b0,    s_Ip.DstIp,     0    
    qbne l_c1ParseIpv4_BM_1,       r1.b0,        32    // 0 found?                 
        set  s_pktCxt.flags.t_flag_broadcast
        jmp  l_c1ParseIpv4_BM_end
    
l_c1ParseIpv4_BM_1:
  qbgt   l_c1ParseIpv4_BM_end, s_Ip.dstIp.b3,   IPV4_MULTICAST_START     
  qblt   l_c1ParseIpv4_BM_end, s_Ip.dstIp.b3,   IPV4_MULTICAST_END
    set  s_pktCxt.flags.t_flag_multicast  
  
l_c1ParseIpv4_BM_end:

#else
  // Firewall operation:adjust startOffset only
  and   r2.w0,                  s_Ip.verLen,                    0x0f
  lsl   s_cdeCmdWd.byteCount,   r2.w0,                          2  // 32 bit size to 8 bit size conversion
  add   s_pktCxt.startOffset,   s_pktCxt.startOffset,           s_cdeCmdWd.byteCount
  
  qbbs  l_c1ParseIpv4_8_0, s_pktCxt.flags.t_flag_cascaded_forwarding
  qbbc  l_c1ParseIpv4_8_0, s_runCxt.flag2.t_raEn
    //wbs   s_flags.info.tStatus_CDEOutPacket
    sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
  
    //pass through
l_c1ParseIpv4_8_0:  
#endif

  // Get the next action and header type from the IP protocol field
  lbco   r0.b0,             PAMEM_CONST_IP_PROTO,  s_Ip.protocol,   1
  and    s_next.Hdr,        r0.b0,                 0x3f
  lsr    s_param.action,    r0.b0,                 6
  
#ifdef PASS_PROC_LUT1_L4
    // TBD???:To be delted
    //qbeq l_c1ParseIpv4_9_0, s_next.Hdr,   PA_HDR_UNKNOWN
    //    mov s_param.action, SUBS_ACTION_PARSE 
    
l_c1ParseIpv4_9_0:
#endif 

#ifdef PASS_PROC_FIREWALL
    qbbs l_c1ParseIpv4_9_1, s_runCxt.flags.t_l4Avil
        mov s_param.action, SUBS_ACTION_LOOKUP 
    
l_c1ParseIpv4_9_1:
#endif 
  
  //  Advance past the IP header.
  mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
  xout   XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
  
   // The protocol type and TOS must be added
   // to the LUT1 search

#ifdef PASS_PROC_FIREWALL
   set  s_l1View3dIpv4.pktType.fL3PktTypeFirewall
#else
   set  s_l1View3dIpv4.pktType.fL3PktTypeIp
#endif   
   set  s_l1View3dIpv4.pktFlags.fL3Ipv4
   mov  s_l1View3dIpv4.protocol,    s_Ip.protocol
   // Store DSCP in case we need DSCP priority routing
   lsr  s_pktCxt.priority, s_Ip.tos, 2
   mov  s_l1View3dIpv4.dscp, s_pktCxt.priority
   xout XID_LUT1V3,               s_l1View3dIpv4,  SIZE(s_l1View3dIpv4)
   ret

l_c1ParseIpv4_9:
   // IPv4 Parse error
   and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
   or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT  
   jmp  l_c1ParseIpv4_11
   
fci_c1ParseIpv4_10:
   // IPv4/v6 Hdr Error
#ifdef PASS_PROC_FIREWALL
  //add  s_pktCxt.protCount,  s_pktCxt.protCount, PROT_COUNT_IP_STEP
  //set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_IPv4
  //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IPV4_PKTS
  
  //
  // Set l3Offset only for outer IP
  //
  //qbne l_c1ParseIpv4_10_1,    s_pktCxt.l3Offset, 0    
    //mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
  //TBD: forward to next stage since it should be dropped there
  jmp   fci_c1FirewallException  
    
l_c1ParseIpv4_10_1:    
#endif    
   
   and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
   or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_IP_FAIL << PKT_EIDX_SHIFT  
   
   // Common Error
l_c1ParseIpv4_11:
   mov  s_param.action,     SUBS_ACTION_EXIT
   ret
       
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave ipScope
    
    
// *************************************************************************************
// * FUNCTION PURPOSE: IPv4 Reassembly Assistance
// *************************************************************************************
// * DESCRIPTION: Perform the IP Reassembly Assistance.
// * First Pass: (traffics from network)
// * 	Search to find the matching traffic flow (cycle :12 * number of active traffic flows)
// * 	If traffic flow is identified, increment its packet counter and forward the packet with its traffic flow id to the host queue
// * 	If traffic flow is not identified
// *        -	Non-fragmented: normal lookup operation
// *        -	Fragments: allocate a new traffic flow, set its packet counter to 1 and forward the fragment with its traffic flow id 
// *            to the host queue. If traffic flow is not available, just forward the fragment with "NA" traffic flow id to the host queue
// *
// * Second Pass: (traffics from the host)
// *	Decrement the packet counter by the specific count if the traffic flow id is specified. 
// *    Free the traffic flow if its counter reaches 0
// * 	Normal lookup operation
// * 
// *
// *    On entry:
// *            - the CDE is at the start of the IPv4 header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - the next action to take is in param.action
// *            - the IP addresses, next protocol and TOS are in the lut
// *            - the CDE points to the first byte after the IP header
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *
// *   Note: It is an extension of the f_c1ParseIp. Therefore, it is OK to jump back to f_c1ParseIp 
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch r1.w0: fragment offset field: (non-zero ==> fragmented)
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |  IPv4 header                                    
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        |  
// *   R10:       |
// *   R11:         | Traffic Flow
// *   R12:         |
// *   R13:         |
// *   R14:           | IP Reassembly Control Block                                         
// *   R15:           |                                          
// *   R16:           |                                          
// *   R17:           |  
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
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// *************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using ipScope
    
#ifndef PASS_PROC_FIREWALL    

f_c1IpReassm:

  qbbs l_c1IpReassm_pass2,  s_pktCxt.flags.t_flag_2nd_pass  

// First Pass operations
l_c1IpReassm_pass1:

  // Read the minimum IP header
  xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 

  // Verify version
  and   r0,   s_Ip.verLen,  0xF0
  qbne  fci_c1ParseIpv4_10,  r0,         0x40
  
  mov   r1.w0,    IPV4_FRAG_MASK
  and   r1.w0,    s_Ip.FragOff,          r1.w0
  // 
  // IP Frag stats has already been incremented at the RA stage
  //
  //qbeq  l_c1IpReassm_pass1_0,  r1.w0,    0
  //    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG
  
l_c1IpReassm_pass1_0:
  
  // 
  // Search through the active Traffic flows
  //
  lbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  mov   s_ipReassmCxt.tfMapTemp, s_ipReassmCxt.tfMap  
  
l_c1IpReassm_pass1_1:
  
  qbeq  l_c1IpReassm_pass1_2, s_ipReassmCxt.tfMapTemp, 0
  lmbd  s_pktCxt3.tfIndex, s_ipReassmCxt.tfMapTemp, 1 
  clr   s_ipReassmCxt.tfMapTemp, s_pktCxt3.tfIndex  
  lsl   s_ipReassmCxt.offset,   s_pktCxt3.tfIndex, 4
  lbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  qbne  l_c1IpReassm_pass1_1, s_ipTf.srcIp, s_Ip.srcIp
  qbne  l_c1IpReassm_pass1_1, s_ipTf.destIp, s_Ip.dstIp
  qbne  l_c1IpReassm_pass1_1, s_ipTf.proto, s_Ip.protocol
  
  //match found: update and store the counter
  add   s_ipTf.cnt, s_ipTf.cnt, 1
  sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  OFFSET(s_ipTf.proto)
  
  jmp   l_c1IpReassm_pass1_4
  
l_c1IpReassm_pass1_2:
  //match not found
  
  // normal operation for non-fragments packet
  qbeq  fci_c1ParseIpv4_0,  r1.w0,  0
  
  // Find available traffic flow
  qble  l_c1IpReassm_pass1_3, s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numTF  
  lmbd  s_pktCxt3.tfIndex, s_ipReassmCxt.tfMap, 0
  qbeq  l_c1IpReassm_pass1_3, s_pktCxt3.tfIndex, 32
  
  // Traffic Flow found
  set   s_ipReassmCxt.tfMap, s_pktCxt3.tfIndex
  add   s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF,  1
  sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  
  // store the traffic flow
  mov   s_ipTf.srcIp, s_Ip.srcIp
  mov   s_ipTf.destIp, s_Ip.dstIp
  mov   s_ipTf.proto, s_Ip.protocol
  mov   s_ipTf.cnt, 1
  lsl   s_ipReassmCxt.offset,   s_pktCxt3.tfIndex, 4
  sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  
  jmp    l_c1IpReassm_pass1_4
  
l_c1IpReassm_pass1_3:  
  //No Traffic flow available
  mov  s_pktCxt3.tfIndex, PA_INV_TF_INDEX 
  
  // pass through 
   
l_c1IpReassm_pass1_4:
  mov   s_pktCxt3.fragCnt,  1
  set   s_pktCxt.flags.t_flag_2nd_pass  
  wbs   s_flags.info.tStatus_CDEOutPacket

l_c1IpReassm_pass1_4_queue_bounce:
        // Check for Queue Bounce operation
l_c1IpReassm_pass1_4_queue_bounce_ddr:
        qbbc l_c1IpReassm_pass1_4_queue_bounce_msmc, s_ipReassmCxt.queue.t_pa_forward_queue_bounce_ddr
            clr s_ipReassmCxt.queue.t_pa_forward_queue_bounce_ddr
            sbco s_ipReassmCxt.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_ipReassmCxt.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_c1IpReassm_pass1_4_queue_bounce_end

l_c1IpReassm_pass1_4_queue_bounce_msmc:
        qbbc l_c1IpReassm_pass1_4_queue_bounce_end, s_ipReassmCxt.queue.t_pa_forward_queue_bounce_msmc
            clr s_ipReassmCxt.queue.t_pa_forward_queue_bounce_msmc
            sbco s_ipReassmCxt.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_ipReassmCxt.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through
l_c1IpReassm_pass1_4_queue_bounce_end:

  sbco  s_ipReassmCxt.queue,  cCdeOutPkt,  OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)
  sbco  s_ipReassmCxt.flowId, cCdeOutPkt,  OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
  mov   s_param.action,     SUBS_ACTION_FWPKT
  mov   s_next.hdr,         PA_HDR_IPv4
  ret
  
  // Second Pass operations
l_c1IpReassm_pass2:
  clr   s_pktCxt.flags.t_flag_2nd_pass
  // TF index range check
  qbeq  l_c1IpReassm_pass2_3, s_pktCxt3.tfIndex, PA_INV_TF_INDEX
  qble  l_c1ParseIpv4_9,   s_pktCxt3.tfIndex,   32
  
  lsl   r1.w0,  s_pktCxt3.tfIndex, 4  //tfIndex * 16
  lbco  s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
  
  qbge  l_c1IpReassm_pass2_1,   s_ipTf.cnt,     s_pktCxt3.fragCnt
  
    sub  s_ipTf.cnt,    s_ipTf.cnt, s_pktCxt3.fragCnt
    sbco s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
    jmp  l_c1IpReassm_pass2_3
    
l_c1IpReassm_pass2_1:
    // Counter has reached zero, clear the corresponding bit
    lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    clr  s_ipReassmCxt.tfMap,   s_pktCxt3.tfIndex
    qbeq l_c1IpReassm_pass2_2,  s_ipReassmCxt.numActiveTF, 0
        sub  s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF, 1
                       
l_c1IpReassm_pass2_2:
    sbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    // pass through  
    
l_c1IpReassm_pass2_3:
    // verify whether it is an null packet  
    qbbc fci_c1ParseIpv4_0,  s_pktCxt.flags.t_flag_null_pkt  
        // drop the null packet
        mov   s_param.action,     SUBS_ACTION_DISCARD
        ret
#endif        
        
#else
        // Ingress1, PDSP1 only
        // Forward the packets to Ingess3 for continue L3 processing
        mov   s_param.action,     SUBS_ACTION_FWPKT4
        ret

#endif        
       
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave ipScope
    

// ************************************************************************************
// * FUNCTION PURPOSE: Parse an IPv6 header
// ************************************************************************************
// * DESCRIPTION: Ipv6 entries are added to the LUT
// *
// *        On entry:
// *            -  CDE points to the start of the IPv6 packet
// *            -  param.action is SUBS_ACTION_PARSE
// *            -  next.Hdr is PA_HDR_IPv6
// *            -  cdeCmdWd.operation is CDE_CMD_WINDOW_ADVANCE
// *
// *        On exit
// *            -  CDE points to first byte after the IPv6 header
// *            -  param.action is SUBS_ACTION_LOOKUP
// *            -  next.Hdr is as found in the IP protocol table
// *            -  cdeCmdWd.operationis CDE_CMD_WINDOW_ADVANCE
// *
// *   Register Usage:  
// * 
// *   R0:    scratch   | Local Reassembly control instance
// *   R1:    scratch   |
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)     |                        | IP reassembly control block                     
// *   R15:          |                                   |                        |
// *   R16:          |                                   | LUT1 View1   | raInfo  |
// *   R17:          |  LUT1 View  - lut1Scope           |              |         |
// *   R18:          |  (IP address)                       |                      
// *   R19:          |                                     | LUT1 View2           
// *   R20:          |                                     |                      
// *   R21:          |                                     |                      
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
// *************************************************************************************

    .using lut1Scope
    .using cdeScope
    .using pktScope
    .using ipScope
    
// Local structure defintion and assignment    
.struct struct_ipv6Reassm_ctrl
  .u8    nextHdr
  .u8    hdrLen
  .u16   offset
  .u8    flag
  .u8    rsvd1
  .u16   maxOffset
  .u16   tHdrLen
  .u16   rsvd2
.ends

#define  t_reassm_ctrl_flag_frag         t0

.assign struct_ipv6Reassm_ctrl,  r0,     r2,    s_ipv6Reassm_ctrl

#define PA_MAX_HDR_LEN                   256

f_c1ParseIpv6:

#ifdef PASS_PROC_L3
// IPv6 Reassembly-assisted code does not run at PDSP of Firewall and Reassembly
#ifndef PASS_PROC_FIREWALL
  
    // Reassembly-assisted operation
    qbbs l_c1Ipv6ExtReasm_pass2, s_pktCxt.flags.t_flag_null_pkt
    
        // Read in the IPv6 header.
        xin  XID_CDEDATA,  s_Ipv6a,   SIZE(s_Ipv6a)

        // Verify version
        and   r0.b0,                s_Ip.VerLen,   0xF0
        qbne  fci_c1ParseIpv4_10,   r0.b0,         0x60

        //  Advance past the fisr 8 byte (Ipv6a).
        mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,   SIZE(s_Ipv6a)                      // Bytes always present in the header
        xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)

        xin  XID_CDEDATA,  s_Ipv6b,   SIZE(s_Ipv6b)

        // Source and dest IP go out unchanged
        xout XID_LUT1V1,   s_l1View1cIpv6,   SIZE(s_l1View1cIpv6)
        xout XID_LUT1V2,   s_l1View2dIpv6,   SIZE(s_l1View2dIpv6)

        // If no reassm check
        qbbs l_c1ParseIpv6_main, s_pktCxt.flags.t_flag_cascaded_forwarding    
        qbbc l_c1ParseIpv6_main, s_runCxt.flags.t_ipReassmEn
        qbbs l_c1Ipv6ExtReasm_pass2, s_pktCxt.flags.t_flag_2nd_pass

        // Pass 1 operation
        // Fragmentation detection: Is there fragment header?
        // Get to nextHdr field in IPv6 header
        // add s_ipv6Reassm_ctrl.offset, s_pktCxt.startOffset, (32+32+6)
        // Load nextHdr and hdrLen field
        // Check to see if nextHdr is an extension header
        zero    &s_ipv6Reassm_ctrl,     SIZE(s_ipv6Reassm_ctrl)
        mov     s_ipv6Reassm_ctrl.nextHdr,  s_Ipv6a.next 
        add     s_ipv6Reassm_ctrl.offset,   s_pktCxt.startOffset, (32+32+40) 
        mov     s_ipv6Reassm_ctrl.maxOffset, (32 + 32 + PA_MAX_HDR_LEN)
        qbbc    l_c1Ipv6ExtReasm_eoamCheck, s_runCxt.flag2.t_raEn
          add  s_ipv6Reassm_ctrl.offset,   s_ipv6Reassm_ctrl.offset, 8 
l_c1Ipv6ExtReasm_eoamCheck:        
        qbbc l_c1Ipv6ExtReasm_exthdrCheck, s_runCxt.flag3.t_eoamEn   
          add  s_ipv6Reassm_ctrl.offset,   s_ipv6Reassm_ctrl.offset, 8 
          add  s_ipv6Reassm_ctrl.maxOffset, s_ipv6Reassm_ctrl.maxOffset, 8          
l_c1Ipv6ExtReasm_exthdrCheck:    
    qble    l_c1Ipv6ExtReasm_exthdrDone,  s_ipv6Reassm_ctrl.offset,  s_ipv6Reassm_ctrl.maxOffset
    qbeq    l_c1Ipv6ExtReasm_fragfound,   s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_FRAG      
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_HOP_BY_HOP
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_ROUTE
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_DEST_OPT
//   below would save one cycle since jmp is to header done.
//    qbeq    l_c1Ipv6ExtReasm_exthdrDone,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_NO_NEXT
    jmp     l_c1Ipv6ExtReasm_exthdrDone   
    
l_c1Ipv6ExtReasm_exthdrNext:        
    // load the next header
    lbco    s_ipv6Reassm_ctrl.nextHdr,  cCdeInPkt, s_ipv6Reassm_ctrl.offset, 2
    
    // adjust the offset for next one
    add     s_ipv6Reassm_ctrl.thdrLen,  s_ipv6Reassm_ctrl.hdrLen, 1
    lsl     s_ipv6Reassm_ctrl.tHdrLen,  s_ipv6Reassm_ctrl.thdrLen, 3
    add     s_ipv6Reassm_ctrl.offset,  s_ipv6Reassm_ctrl.offset, s_ipv6Reassm_ctrl.tHdrLen
    
    jmp     l_c1Ipv6ExtReasm_exthdrCheck
    
l_c1Ipv6ExtReasm_fragfound:
    lbco    s_ipv6Reassm_ctrl.nextHdr,  cCdeInPkt, s_ipv6Reassm_ctrl.offset, 1
    set     s_ipv6Reassm_ctrl.flag.t_reassm_ctrl_flag_frag 
    // 
    // IP Frag stats has already been incremented at the RA stage
    //
    //mov     s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG

l_c1Ipv6ExtReasm_exthdrDone:        

l_c1Ipv6ExtReasm_pass1_1:

    // s_ipReassmCxt is r14 - r17, which overwrites s_Ipv6b.srcIp_3
    mov  r2, s_Ipv6b.srcIp_3
    lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    mov s_ipReassmCxt.tfMapTemp, s_ipReassmCxt.tfMap

l_c1Ipv6ExtReasm_pass1_2:
    qbeq l_c1Ipv6ExtReasm_pass1_3, s_ipReassmCxt.tfMapTemp, 0
        lmbd s_pktCxt3.tfIndex, s_ipReassmCxt.tfMapTemp, 1
        clr s_ipReassmCxt.tfMapTemp, s_pktCxt3.tfIndex
        lsl s_ipReassmCxt.offset, s_pktCxt3.tfIndex, 4
        lbco s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset, SIZE(s_ipTf)

        // Only compare the last 4 bytes of Ipv6 header
        // r1 now contains s_Ipv6b.srcIp_3
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.srcIp, r2
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.destIp, s_Ipv6b.dstIp_3
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.proto,  s_ipv6Reassm_ctrl.nextHdr
  
        // match found: update and store the counter
        add   s_ipTf.cnt, s_ipTf.cnt, 1
        sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  OFFSET(s_ipTf.proto)
  
        jmp   l_c1Ipv6ExtReasm_pass1_5
  
l_c1Ipv6ExtReasm_pass1_3:
        //match not found
        // Fragmentation check  
        // normal operation for non-fragments packet
        qbbc  l_c1ParseIpv6_main,  s_ipv6Reassm_ctrl.flag.t_reassm_ctrl_flag_frag
  
        // Find available traffic flow
        qble  l_c1Ipv6ExtReasm_pass1_4, s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numTF  
            lmbd  s_pktCxt3.tfIndex, s_ipReassmCxt.tfMap, 0
            qbeq  l_c1Ipv6ExtReasm_pass1_4, s_pktCxt3.tfIndex, 32
  
        // Traffic Flow found
        set   s_ipReassmCxt.tfMap, s_pktCxt3.tfIndex
        add   s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF,  1
        sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  
        // store the traffic flow
        // r2 now contains s_Ipv6b.srcIp_3
        mov   s_ipTf.srcIp, r2
        mov   s_ipTf.destIp, s_Ipv6b.dstIp_3
        mov   s_ipTf.proto,  s_ipv6Reassm_ctrl.nextHdr
        mov   s_ipTf.cnt, 1
        lsl   s_ipReassmCxt.offset,   s_pktCxt3.tfIndex, 4
        sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  
        jmp    l_c1Ipv6ExtReasm_pass1_5
  
l_c1Ipv6ExtReasm_pass1_4:  
    // No Traffic flow available
    mov  s_pktCxt3.tfIndex, PA_INV_TF_INDEX 
  
  // pass through 
l_c1Ipv6ExtReasm_pass1_5:
    mov   s_pktCxt3.fragCnt,  1
    set   s_pktCxt.flags.t_flag_2nd_pass  
    wbs   s_flags.info.tStatus_CDEOutPacket

l_c1Ipv6ExtReasm_pass1_5_queue_bounce:
    // Check for Queue Bounce operation
l_c1Ipv6ExtReasm_pass1_5_queue_bounce_ddr:
    qbbc l_c1Ipv6ExtReasm_pass1_5_queue_bounce_msmc, s_ipReassmCxt.queue.t_pa_forward_queue_bounce_ddr
        clr s_ipReassmCxt.queue.t_pa_forward_queue_bounce_ddr
        sbco s_ipReassmCxt.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
        lbco s_ipReassmCxt.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
        jmp  l_c1Ipv6ExtReasm_pass1_5_queue_bounce_end

l_c1Ipv6ExtReasm_pass1_5_queue_bounce_msmc:
    qbbc l_c1Ipv6ExtReasm_pass1_5_queue_bounce_end, s_ipReassmCxt.queue.t_pa_forward_queue_bounce_msmc
        clr s_ipReassmCxt.queue.t_pa_forward_queue_bounce_msmc
        sbco s_ipReassmCxt.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
        lbco s_ipReassmCxt.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
         // pass through
l_c1Ipv6ExtReasm_pass1_5_queue_bounce_end:

    sbco  s_ipReassmCxt.queue,  cCdeOutPkt,  OFFSET(s_pktDescr.destQ),    SIZE(s_pktDescr.destQ)
    sbco  s_ipReassmCxt.flowId, cCdeOutPkt,  OFFSET(s_pktDescr.flowIdx),  SIZE(s_pktDescr.flowIdx)
    mov   s_param.action,     SUBS_ACTION_FWPKT
    mov   s_next.hdr,         PA_HDR_IPv6
    ret
  
  // Second Pass operations
l_c1Ipv6ExtReasm_pass2:
    clr   s_pktCxt.flags.t_flag_2nd_pass
    // TF index range check
    qbeq  l_c1Ipv6ExtReasm_pass2_3, s_pktCxt3.tfIndex, PA_INV_TF_INDEX
    qble  l_c1ParseIpv4_9,   s_pktCxt3.tfIndex,   32
  
    lsl   r1.w0,  s_pktCxt3.tfIndex, 4  //tfIndex * 16
    lbco  s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
  
    qbge  l_c1Ipv6ExtReasm_pass2_1,   s_ipTf.cnt,     s_pktCxt3.fragCnt
  
    sub  s_ipTf.cnt,    s_ipTf.cnt, s_pktCxt3.fragCnt
    sbco s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
    jmp  l_c1Ipv6ExtReasm_pass2_3
    
l_c1Ipv6ExtReasm_pass2_1:
    // Counter has reached zero, clear the corresponding bit
    lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    clr  s_ipReassmCxt.tfMap,   s_pktCxt3.tfIndex
    qbeq l_c1Ipv6ExtReasm_pass2_2,  s_ipReassmCxt.numActiveTF, 0
    sub  s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF, 1
                       
l_c1Ipv6ExtReasm_pass2_2:
    sbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    // pass through
    
l_c1Ipv6ExtReasm_pass2_3:
    // verify whether it is an null packet  
    qbbc l_c1ParseIpv6_main,  s_pktCxt.flags.t_flag_null_pkt
        // drop the null packet
        mov   s_param.action,     SUBS_ACTION_DISCARD
        ret
        
#endif  // PASS_PROC_FIREWALL        

l_c1ParseIpv6_main:

#ifdef PASS_VERIFY_POST_RA_DROP
        wbs  s_flags.info.tStatus_CDEOutPacket
        lbco  r1.b0,  cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags),  SIZE(s_pktDescr.psFlags_errorFlags)
        qbbc  l_c1ParseIpv6_main_0, r1.b0.t_psFlags_pktDropFromRA
            // drop the packet
            mov s_param.action, SUBS_ACTION_DISCARD
            ret
    l_c1ParseIpv6_main_0:    
#endif    

#ifndef PASS_PROC_FIREWALL

    // New IPv6 packet: check maximum count
    lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.ipMaxCount), 2

    // IP depth count
    and  r0.w0,         s_pktCxt.protCount,  PROT_COUNT_IP_MASK
    qblt l_c1ParseIpv6_00,   r0.w2,  r0.w0
        and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
        or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_IP_MAX_DEPTH << PKT_EIDX_SHIFT  

        mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_DEPTH_OVERFLOW
        mov s_param.action,        SUBS_ACTION_EXIT
        ret

l_c1ParseIpv6_00:
    add  s_pktCxt.protCount, s_pktCxt.protCount, PROT_COUNT_IP_STEP
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IPV6_PKTS
    set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_IPv6
    
#ifdef PASS_PROC_INNER_IP
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_INNER_IPV6_PKTS
  // store the inner IP offset
  mov s_pktCxt5.l3l5Offset, s_pktCxt.startOffset
#else
  // store the outer IP offset
  mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
#endif  

#ifdef TOBE_DELETED
    
  //
  // t_l3toInnerIp = FALSE: Set l3Offset for outer IP 
  // t_l3toInnerIp = TRUE:  Set l3Offset for inner IP
  //
  qbbs l_c1ParseIpv6_01_setInnerIp, s_runCxt.flag2.t_l3toInnerIp
    qbne l_c1ParseIpv6_0,    s_pktCxt.l3Offset, 0
l_c1ParseIpv6_01_setInnerIp:      
        mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
    
#endif
        
#endif  // End of Firewall condition    
   
l_c1ParseIpv6_0:
    zero  &s_l1View3bIpv6,    SIZE(s_l1View3bIpv6)
   
#ifndef PASS_PROC_FIREWALL
  // Check to see whether virtual link is used 
  qbbs l_c1ParseIpv6_01_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l1View3bIpv6.srcVC,  s_pktCxt.phyLink
    jmp  l_c1ParseIpv6_01_link_end
l_c1ParseIpv6_01_vLink:    
    mov  s_l1View3bIpv6.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l1View3bIpv6.srcVC.b1, s_l1View3bIpv6.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c1ParseIpv6_01_link_end:
#else
    mov  s_l1View3bIpv6.srcVC,  s_pktCxt.phyLink
#endif

#ifdef   PASS_PROC_FIREWALL

#ifdef PASS_DETECT_IP_PROC
 qbbc l_c1ParseIpv6_in3, s_runCxt.flag3.t_eoamEn
  set  s_pktCxt.flags.t_flag_ipProc
l_c1ParseIpv6_in3:
#endif 
   // Read in the IPv6 header.
   xin  XID_CDEDATA,  s_Ipv6a,   SIZE(s_Ipv6a)
   
   // Verify version
   and   r0.b0,                s_Ip.verLen,   0xF0
   qbne  fci_c1ParseIpv4_10,   r0.b0,         0x60
  
   //  Advance past the fisr 8 byte (Ipv6a).
   zero &s_cdeCmdWd,            SIZE(s_cdeCmdWd)
   mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
   mov  s_cdeCmdWd.byteCount,   SIZE(s_Ipv6a)                      // Bytes always present in the header
   xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   
   xin  XID_CDEDATA,  s_Ipv6b,   SIZE(s_Ipv6b)

   // Source and dest IP go out unchanged
   xout XID_LUT1V1,   s_l1View1cIpv6,   SIZE(s_l1View1cIpv6)
   xout XID_LUT1V2,   s_l1View2dIpv6,   SIZE(s_l1View2dIpv6)
   
#endif
   
#ifndef PASS_PROC_FIREWALL   

   // Begin the pseudo header checksum to free up LUT window 1 & 2
   // note: the pseudo header checksum does not include protocol and length
   // reload the IPv6 addresses
   xin   XID_CDEDATA,  s_Ipv6b,   SIZE(s_Ipv6b)
   add   r0,   s_Ipv6b.srcIp_0,   s_Ipv6b.srcIp_1
   adc   r0,   r0,                s_Ipv6b.srcIp_2
   adc   r0,   r0,                s_Ipv6b.srcIp_3
   adc   r0,   r0,                s_Ipv6b.dstIp_0
   adc   r0,   r0,                s_Ipv6b.dstIp_1
   adc   r0,   r0,                s_Ipv6b.dstIp_2
   adc   r0,   r0,                s_Ipv6b.dstIp_3
   adc   r0,   r0.w0,             r0.w2     // Fold carry once
   add   s_pktCxt.pseudo, r0.w0,  r0.w2     // Fold carry twice
   
   //Multicast detection
   qbne l_c1ParseIpv6_1,    s_Ipv6b.dstIp_0.b3, IPV6_MULTICAST_ADDR_BYTE0
   qbne l_c1ParseIpv6_1,    s_Ipv6b.dstIp_0.b2, IPV6_MULTICAST_ADDR_BYTE1
        set s_pktCxt.flags.t_flag_multicast  
        
#else
   // Firewall (RA) specific operation 
   // TBD: priority settings
   lbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
   //zero &s_raInfo,   SIZE(s_raInfo)
   mov  s_raInfo.l3Offset, s_pktCxt.startOffset 
  //qbbc l_c1ParseIpv6_set_ra_threadId_1, s_runCxt.flag2.t_raToQueue
    // RA output is host queue
    //mov  s_raInfo.flags, THREADID_CDMA0
    //jmp  l_c1ParseIpv6_set_ra_threadId_end
l_c1ParseIpv6_set_ra_threadId_1:  
    //mov  s_raInfo.flags, PASS_RA_DEST_THREAD_ID 
    // pass through  
l_c1ParseIpv6_set_ra_threadId_end:
#endif        
   
l_c1ParseIpv6_1:
   
   // The protocol, tclass, and flow labels are moved as 
   // required
   mov   s_l1View3bIpv6.flowLabel, s_Ipv6a.ver_tclass_flow
   and   s_l1View3bIpv6.flowLabel.w2,  s_l1View3bIpv6.flowLabel.w2,   0x0f

#ifdef PASS_PROC_FIREWALL   
   set   s_l1View3bIpv6.pktType.fL3PktTypeFirewall
#else
   set   s_l1View3bIpv6.pktType.fL3PktTypeIp
#endif   
   mov   s_l1View3bIpv6.protocol,      s_Ipv6a.next
   lsr   s_l1View3bIpv6.dscp,   s_Ipv6a.ver_tclass_flow.w2,  4
   and   s_l1View3bIpv6.dscp,   s_l1View3bIpv6.dscp, 0x3F
   // Store the DSCP priority bits in case we need DSCP priority routing 
   mov   s_pktCxt.priority,    s_l1View3bIpv6.dscp
   
   xout  XID_LUT1V3,               s_l1View3bIpv6,         SIZE(s_l1View3bIpv6)

   // Update packet offsets
#ifndef PASS_PROC_FIREWALL  
   add   s_pktCxt.startOffset,   s_pktCxt.startOffset,  IPV6_HEADER_LEN_BYTES
   add   s_pktCxt.endOffset,     s_pktCxt.startOffset,  s_Ipv6a.payloadLen
#else 
   add   s_raInfo.ipv6NextOffset, s_pktCxt.startOffset, OFFSET(s_Ipv6a.next)    
   add   s_pktCxt.startOffset,   s_pktCxt.startOffset,  IPV6_HEADER_LEN_BYTES
   qbbs  l_c1ParseIpv6_1_0, s_pktCxt.flags.t_flag_cascaded_forwarding
   qbbc  l_c1ParseIpv6_1_0, s_runCxt.flag2.t_raEn
      //wbs   s_flags.info.tStatus_CDEOutPacket
      sbco  s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
  
    //pass through
l_c1ParseIpv6_1_0:   
#endif

   // Get the next header and action
   set    s_l1View3bIpv6.pktFlags.fL3IpContainL4
   lbco   r0.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6a.next,   1
   and    s_next.Hdr,        r0.b0,                 0x3f
   lsr    s_param.action,    r0.b0,                 6
   
#ifdef PASS_PROC_LUT1_L4
    //qbeq l_c1ParseIpv6_1_1, s_next.Hdr,   PA_HDR_UNKNOWN
    //    mov s_param.action, SUBS_ACTION_PARSE 
    
l_c1ParseIpv6_1_1:

#endif  
   
   
   //  Advance past the always present IP header.
   //  Note: The other parameters are set previously
   mov   s_cdeCmdWd.byteCount,   IPV6_HEADER_LEN_BYTES - SIZE(s_Ipv6a) // Bytes always present in the header
   xout   XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   
   ret
   
#else
   // Ingress1, PDSP1 only
   // Forward the packets to Ingess3 for continue L3 processing
   mov   s_param.action,     SUBS_ACTION_FWPKT4
   ret

#endif
    .leave lut1Scope
    .leave cdeScope
    .leave pktScope
    .leave ipScope

// ***********************************************************************************
// * FUNCTION PURPOSE: Parse the IPv6 optional hop by hop header
// ***********************************************************************************
// * DESCRIPTION: The hop by hop option header is processed. The only recognized option
// *              is the jumbo frame option. When detected the entire packet is sent
// *              to the jumbo frame queue. In reality it should have never made it 
// *              to classify1 since the smallest jumbo frame is larger then 64k bytes
// *
// *              Unknown hop by hop options are bypassed, independent of the action bits
// *              in the option type
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)     |             
// *   R15:          |                                   |                   
// *   R16:          |                                   | LUT1 View1  | raInfo   
// *   R17:          |  LUT1 View  - lut1Scope           |             |
// *   R18:          |  (IP address)                       | 
// *   R19:          |                                     | LUT1 View2 
// *   R20:          |                                     |
// *   R21:          |                                     |
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
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtHop:

#ifdef PASS_PROC_L3

#ifdef PASS_PROC_RA
   mov  s_raInfo.ipv6NextOffset, s_pktCxt.startOffset
#endif

   // The length of the options field in the header is in 8 byte units, not
   // including the 1st 8 bytes.
   xin  XID_CDEDATA,  s_Ipv6Opt,         SIZE(s_Ipv6Opt)
   lsl  r0.w0,        s_Ipv6Opt.optlen,  3
   qblt fci_c1ParseIpv4_10,      r0.w0,       (PA_MAX_HDR_LEN-1)      // Hdr length check     
   add  s_cdeCmdWd.byteCount,    r0.w0,             8
   add  s_pktCxt.startOffset, s_pktCxt.startOffset, s_cdeCmdWd.byteCount
   
   // Scroll past this header
   xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Opt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6
   ret
   
#endif   

    .leave cdeScope
    .leave pktScope
    .leave ipScope
    .leave lut1Scope


// ************************************************************************************
// * FUNCTION PURPOSE: Parse the IPv6 route extension header
// ************************************************************************************
// * DESCRIPTION: The header is checked to see if any routes remain. If so a flag is set
// *              which will cause the packet to route to the destination specified in the
// *              error routing table for IP routing (assuming LUT1 lookup is successful).
// *              If LUT1 lookup fails then the packet is routed as a LUT1 match fail.
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     |              
// *   R11:       |                                     |  LUT1 View3  
// *   R12:       |                                     |              | raInfo
// *   R13:       |                                     |              |      
// *   R14:          |  (packet extended descriptor)     |                                    
// *   R15:          |                                   |        
// *   R16:          |                                   | LUT1 View1      
// *   R17:          |  LUT1 View  - lut1Scope           |      
// *   R18:          |  (IP address)                       | 
// *   R19:          |                                     | LUT1 View2 
// *   R20:          |                                     |
// *   R21:          |                                     |
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
// *************************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtRoute:

#ifdef PASS_PROC_L3

#ifdef PASS_PROC_RA
   mov  s_raInfo.ipv6NextOffset, s_pktCxt.startOffset
#endif

   xin  XID_CDEDATA,  s_Ipv6ExtRt,   SIZE(s_Ipv6ExtRt)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6ExtRt.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)
   
   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6ExtRt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6

l_c1ParseIpv6ExtRoute0:
   // The options header length is in 8 byte units, not including the 1st 8 bytes
   lsl  r0.w0,                 s_Ipv6ExtRt.hdrlen,    3
   qblt fci_c1ParseIpv4_10,    r0.w0,       (PA_MAX_HDR_LEN-1)      // Hdr length check     
   add  s_cdeCmdWd.byteCount,  r0.w0,                 8
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)
   add  s_pktCxt.startOffset,  s_pktCxt.startoffset,  s_cdeCmdWd.byteCount

   ret
   
#endif   

    .leave cdeScope
    .leave pktScope
    .leave ipScope
    .leave lut1Scope

// *************************************************************************************
// * FUNCTION PURPOSE: Parse the IPv6 fragment extension header
// *************************************************************************************
// * DESCRIPTION: The packet is marked as fragmented
// *
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     |              
// *   R11:       |                                     |  LUT1 View3  
// *   R12:       |                                     |              | raInfo
// *   R13:       |                                     |              |      
// *   R14:          |  (packet extended descriptor)     |                                    
// *   R15:          |                                   |        
// *   R16:          |                                   | LUT1 View1      
// *   R17:          |  LUT1 View  - lut1Scope           |      
// *   R18:          |  (IP address)                       | 
// *   R19:          |                                     | LUT1 View2 
// *   R20:          |                                     |
// *   R21:          |                                     |
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
// *************************************************************************************/
    .using cdeScope
    .using ipScope
    .using pktScope
    .using lut1Scope

f_c1ParseIpv6ExtFrag:

#ifdef PASS_PROC_L3

#ifdef PASS_PROC_RA
  qbbs  l_c1ParseIpv6ExtFrag00, s_pktCxt.flags.t_flag_cascaded_forwarding
  qbbc  l_c1ParseIpv6ExtFrag00, s_runCxt.flag2.t_raEn
    mov  s_raInfo.ipv6FragOffset, s_pktCxt.startOffset
    set  s_raInfo.flags.t_ra_flag_reasm
#ifdef PASS_INNER_RA
      //set s_raInfo.flags.t_ra_flag_2nd_inst   
#endif         
    //wbs  s_flags.info.tStatus_CDEOutPacket
    sbco s_raInfo,  cCdeOutPkt,  SIZE(s_pktDescr) + 24,  SIZE(s_raInfo)
l_c1ParseIpv6ExtFrag00:   
#endif

   // The fragmentation header is always 8 bytes. The whole 
   // header is read here, but only the 1st byte is used
   xin  XID_CDEDATA,           s_Ipv6Frag,      SIZE(s_Ipv6Frag)      
   mov  s_cdeCmdWd.byteCount,  SIZE(s_Ipv6Frag)
   xout XID_CDECTRL,           s_cdeCmdWd,      SIZE(s_cdeCmdWd)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Frag.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)
   
l_c1ParseIpv6ExtFrag0:
   add  s_pktCxt.startOffset,           s_pktCxt.startOffset,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
   set  s_l1View3bIpv6.pktFlags.fL3IpFrag
  // Fragmentation check
  lsr   r3.w0,   s_Ipv6Frag.fragnFlag,     IPV6_FRAG_OFF_SHIFT
  qbeq  l_c1ParseIpv6ExtFrag1, r3.w0, 0
        clr  s_l1View3bIpv6.pktFlags.fL3IpContainL4
        clr  s_runCxt.flags.t_l4Avil
l_c1ParseIpv6ExtFrag1:  
   xout XID_LUT1V3,           s_l1View3bIpv6.pktFlags,  SIZE(s_l1View3bIpv6.pktFlags)
   
#ifndef PASS_PROC_RA    
   set  s_pktCxt.flags.t_flag_frag
#else  // PASS_PROC_RA
   mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG
  //qbbs  l_c1ParseIpv6ExtFrag2_1, s_pktCxt.flags.t_flag_cascaded_forwarding
  //qbbc  l_c1ParseIpv6ExtFrag2_1, s_runCxt.flag2.t_raEn
    
l_c1ParseIpv6ExtFrag2_1:
#endif


   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Frag.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6
   
#ifdef PASS_PROC_FIREWALL
    qbbs l_c1ParseIpv6ExtFrag3, s_runCxt.flags.t_l4Avil
        mov s_param.action, SUBS_ACTION_LOOKUP 
    
l_c1ParseIpv6ExtFrag3:
#endif 
   
   ret

#endif

    .leave cdeScope
    .leave ipScope
    .leave pktScope
    .leave lut1Scope

// ***********************************************************************************
// * FUNCTION PURPOSE: Parse the IPv6 destination options extension header
// ***********************************************************************************
// * DESCRIPTION: The destination options header is ignored
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     |              
// *   R11:       |                                     |  LUT1 View3  
// *   R12:       |                                     |              | raInfo
// *   R13:       |                                     |              |      
// *   R14:          |  (packet extended descriptor)     |                                    
// *   R15:          |                                   |        
// *   R16:          |                                   | LUT1 View1      
// *   R17:          |  LUT1 View  - lut1Scope           |      
// *   R18:          |  (IP address)                       | 
// *   R19:          |                                     | LUT1 View2 
// *   R20:          |                                     |
// *   R21:          |                                     |
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
// *
// ***********************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtDestOpt:

#ifdef PASS_PROC_L3

#ifdef PASS_PROC_RA
   mov  s_raInfo.ipv6NextOffset, s_pktCxt.startOffset
#endif

   xin  XID_CDEDATA,  s_Ipv6Opt,  SIZE(s_Ipv6Opt)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Opt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6

   lsl    r0.w0,                s_Ipv6Opt.optlen,     3
   qblt   fci_c1ParseIpv4_10,   r0.w0,       (PA_MAX_HDR_LEN-1)      // Hdr length check     
   add    s_cdeCmdWd.byteCount, r0.w0,                8
   xout   XID_CDECTRL,          s_cdeCmdWd,           SIZE(s_cdeCmdWd)
   add    s_pktCxt.startOffset, s_pktCxt.startOffset, s_cdeCmdWd.byteCount

   ret

#endif
   
    .leave cdeScope
    .leave pktScope
    .leave ipScope
    .leave lut1Scope

// **********************************************************************************
// * FUNCTION PURPOSE: Parse a GRE header
// **********************************************************************************
// * DESCRIPTION:
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  GRE header
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)     |                                    
// *   R15:          |                                   |        
// *   R16:          |                                   | LUT1 View1      
// *   R17:          |  LUT1 View  - lut1Scope           |      
// *   R18:          |  (IP address)                       | 
// *   R19:          |                                     | LUT1 View2 
// *   R20:          |                                     |
// *   R21:          |                                     |
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
// *
// **********************************************************************************
    .using cdeScope
    .using pktScope
    .using greScope
    .using lut1Scope

f_c1ParseGre:

#ifdef PASS_PROC_FIREWALL
    // Terminate parsing
    mov s_param.action,        SUBS_ACTION_LOOKUP
    mov s_next.Hdr,     PA_HDR_UNKNOWN
    ret
#endif

#ifdef PASS_PROC_L3

  // New GRE packet: check maximum count
  lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.greMaxCount), 2

  // IP depth count
  lsr  r0.w0,   s_pktCxt.protCount,  PROT_COUNT_GRE_SHIFT
  and  r0.w0,   r0.w0,               PROT_COUNT_GRE_MASK
  qblt l_c1ParseGre00,   r0.w2,  r0.w0
      and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
      or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_GRE_MAX_DEPTH << PKT_EIDX_SHIFT  
      mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_GRE_DEPTH_OVERFLOW
      mov s_param.action,        SUBS_ACTION_EXIT
      ret

l_c1ParseGre00:
  mov  s_param.action, SUBS_ACTION_LOOKUP
  add  s_pktCxt.protCount,  s_pktCxt.protCount,  PROT_COUNT_GRE_STEP

  xin  XID_CDEDATA,           s_greHdr,      SIZE(s_greHdr)
  mov  s_cdeCmdWd.byteCount,  SIZE(s_greHdr)
  xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)
  
  add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  SIZE(s_greHdr)
  mov  s_cdeCmdWd.byteCount,  0

  // GRE checksum calculation
  qbbc  l_c1ParseGre0, s_greHdr.flags.t_GreCBit
#ifdef GRE_CHECKSUM_VALIDATE
      // Not validated  
      // Restore the latest extended Packet Info
      xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)
      zero &s_cdeCmdChk,           SIZE(s_cdeCmdChk)
      sub   s_cdeCmdChk.byteLen,   s_pktCxt.endOffset,  s_pktCxt.startOffset
      sub   s_cdeCmdChk.byteLen,   s_cdeCmdChk.byteLen, s_pktExtDescr.mopLength
      mov   s_cdeCmdChk.initSum,   s_pktExtDescr.mopCsum
      mov   s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_VALIDATE
      xout  XID_CDECTRL,           s_cdeCmdChk,                   SIZE(s_cdeCmdChk)
#endif
      mov   s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
      mov   s_cdeCmdWd.byteCount,  GRE_SIZE_CHKSUM_BYTES

l_c1ParseGre0:
   // Load the GRE ethertype value into the View3 dstPort
   xin XID_LUT1V3,        s_l1View3bIpv6.pktFlags,   SIZE(s_l1View3bIpv6.pktFlags)
   set  s_l1View3bIpv6.pktFlags.fGre
   xout XID_LUT1V3,       s_l1View3bIpv6.pktFlags,   SIZE(s_l1View3bIpv6.pktFlags)

   mov  s_l1View3bIpv6.dstPort, s_greHdr.proto
   xout XID_LUT1V3,       s_l1View3bIpv6.dstPort,   SIZE(s_l1View3bIpv6.dstPort)

   //mov  s_cdeCmdWd.byteCount,  0
   qbbc  l_c1ParseGre1,  s_greHdr.flags.t_GreKBit
       add  s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount,  GRE_SIZE_KEY_BYTES

l_c1ParseGre1:
   qbbc  l_c1ParseGre2,  s_greHdr.flags.t_GreSBit
       add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,   GRE_SIZE_SEQNUM_BYTES

l_c1ParseGre2:
   //  scroll past the optional key and sequence number fields
   //  a scroll of 0 should be ok
   xout  XID_CDECTRL,  s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   add   s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_cdeCmdWd.byteCount

   mov   r0.w0,  ETH_TYPE_IP
   qbne  l_c1ParseGre3, s_greHdr.proto, r0.w0 
       mov s_next.Hdr,  PA_HDR_IPv4
       ret

l_c1ParseGre3:
   mov   r0.w0,  ETH_TYPE_IPV6
   qbne  l_c1ParseGre4, s_greHdr.proto, r0.w0
       mov s_next.Hdr,  PA_HDR_IPv6
       ret

l_c1ParseGre4:
       mov s_next.Hdr,  PA_HDR_UNKNOWN
       ret
#endif

    .leave cdeScope
    .leave pktScope
    .leave greScope
    .leave lut1Scope
    

// **************************************************************************************
// * FUNCTION PURPOSE: Parse an ESP header
// **************************************************************************************
// * DESCRIPTION: The SPI field is added to the LUT. At this stage there is no information
// *              available about the next header
// *              
// *              The offset is not updated
// *
// *    On entry:
// *            - the CDE is at the start of the ESP header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - CDE View window does not move
// *            - startOffset is added by ESP_HEADER_LEN_BYTES
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                  |  ESP Header
// *   R7:        |                                    
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
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
// *
// **************************************************************************************

    .using cdeScope
    .using lut1Scope
    .using pktScope
    .using secScope


f_c1ParseEsp:

#ifdef PASS_PROC_FIREWALL
    // Terminate parsing
    mov s_param.action,        SUBS_ACTION_LOOKUP
    ret
#endif

#ifdef PASS_PROC_IPSEC   


#ifdef PASS_SKIP_IPSEC
   qbbs l_c1ParseEsp_fwpkt, s_runCxt.flags.t_firstLookup
        xin  XID_LUT1V3,   s_l1View3bIpsec.pktFlags, SIZE(s_l1View3bIpsec.pktFlags)
        set  s_l1View3bIpsec.pktFlags.fIpsec
        xout XID_LUT1V3,   s_l1View3bIpsec.pktFlags, SIZE(s_l1View3bIpsec.pktFlags)
        set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_ESP
        mov  s_param.action, SUBS_ACTION_LOOKUP
        ret
l_c1ParseEsp_fwpkt:
   // The IPSEC lookup occurs at the next stage (Ingress1, CED1)
   // This is IPSEC ESP NAT-T traffic back from Ingress4
        mov  s_param.action, SUBS_ACTION_FWPKT2
        ret
#else
   // Read in the ESP spi and add it to the LUT
   xin  XID_CDEDATA,  s_esp,     SIZE(s_esp)
   
   // Populate the lookup views
   zero &s_l1View3bIpsec,   SIZE(s_l1View3bIpsec)
   mov  s_l1View3bIpsec.spi,    s_esp.spi
   set  s_l1View3bIpsec.pktFlags.fIpsecEsp
   set  s_l1View3bIpsec.pktType.fL3PktTypeIpsec
   
#ifndef PASS_PROC_FIREWALL
  // Check to see whether virtual link is used 
  qbbs l_c1ParseEsp_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.phyLink
    jmp  l_c1ParseEsp_link_end
l_c1ParseEsp_vLink:    
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l1View3bIpsec.srcVC.b1, s_l1View3bIpsec.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c1ParseEsp_link_end:
#else
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.phyLink
#endif   
   xout XID_LUT1V3,   s_l1View3bIpsec, SIZE(s_l1View3bIpsec)
   
   mov  s_next.hdr,     PA_HDR_UNKNOWN
   mov  s_param.action, SUBS_ACTION_LOOKUP

   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_ESP

   mov  s_pktCxt.espAhOffset,  s_pktCxt.startOffset
   // Adjust the start offset to end of ESP header
   // It is up to the Host or SA to further adjust it for the IV size
   add  s_pktCxt.startOffset,  s_pktCxt.startOffset, ESP_HEADER_LEN_BYTES
   ret
#endif   
#endif   

    .leave cdeScope
    .leave lut1Scope
    .leave pktScope
    .leave secScope

// ***************************************************************************************
// * FUNCTION PURPOSE: Parse a decoded ESP header
// ***************************************************************************************
// * DESCRIPTION: After (optional) authentication and decryption, the ESP header is
// *              reparsed. the end offset must now point to the end of the ESP 
// *              trailer.
// *
// *    On entry:
// *            - the CDE points to the first byte after the ESP header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - the next action to take is in param.action
// *            - the CDE points to the first byte after the ESP header
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - endOffset is adjusted by the size of ESP Trail
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
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
// *   R15:          |  extended packet dewscriptor                                        
// *   R16:          |                                          
// *   R17:          |  
// *   R18:          |   
// *   R19:             |
// *   R20:             |
// *   R21:             |
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
// *
// **************************************************************************************/

    .using cdeScope
    .using pktScope

f_c1ParseEspDec:
   
#ifdef PASS_PROC_POST_IPSEC  
   
   // Read in the padlen (r0.b1) and protocol (r0.b0) fields at the end of the packet
   // Note: It should be the absolute offset including Packet descriptor, PS Info.
   // sub    r0.w2,   s_pktCxt.endOffset,  s_pktCxt.startOffset
   // Note: Both the size of packet descriptor and PS Info are constants 
   //       We may need to enhance it for more general cases
   xin    XID_PINFO_A,  s_pktExtDescr,  SIZE(s_pktExtDescr)  
   add    r0.w2,   s_pktCxt.endOffset,  (32+32-2 )  
   qbbc l_c1ParseEspDec_1, s_runCxt.flag3.t_eoamEn   
     add  r0.w2,   r0.w2, 8      
l_c1ParseEspDec_1:    
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
   
#ifdef PASS_PROC_FIREWALL    
    // Adjust and  the next header type and startOffset for next stage lookup
    and  s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  0xc0
    or   s_pktCxt.eId_portNum_nextHdr.b0, s_pktCxt.eId_portNum_nextHdr.b0,  s_next.Hdr
    // The stratOffset does not change
    //mov  s_pktCxt5.temp.b1, s_pktCxt.startOffset  
#endif    
   

   ret
   
#endif   

    .leave cdeScope
    .leave pktScope


// ************************************************************************************
// * FUNCTION PURPOSE: Process an AH header
// ************************************************************************************
// * DESCRIPTION: The authentication header is parsed. The length field could have 0,
// *              which means the authentication length is not known, so the length
// *              field is not updated at all.
// *    On entry:
// *            - the CDE is at the start of the AH header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - CDE View window does not move
// *            - startOffset is adjusted by the size of AH Header
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                  |  AH header
// *   R7:        |                                  |  
// *   R8:        |                                  |  
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
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
// *
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using lut1Scope
    .using secScope

f_c1ParseAuth:

#ifdef PASS_PROC_FIREWALL
    // Terminate parsing
    mov s_param.action,        SUBS_ACTION_LOOKUP
    ret
#endif


#ifdef PASS_PROC_IPSEC  

#ifdef PASS_SKIP_IPSEC
   xin  XID_LUT1V3,   s_l1View3bIpsec, SIZE(s_l1View3bIpsec)
   set  s_l1View3bIpsec.pktFlags.fIpsec
   xout XID_LUT1V3,   s_l1View3bIpsec, SIZE(s_l1View3bIpsec)
   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_AUTH
   mov  s_param.action, SUBS_ACTION_LOOKUP
   ret
#endif

   xin  XID_CDEDATA, s_ah,  SIZE(s_ah)

   // Populate the lookup views
   zero &s_l1View3bIpsec,   SIZE(s_l1View3bIpsec)
   mov  s_l1View3bIpsec.spi,    s_ah.spi
   set  s_l1View3bIpsec.pktFlags.fIpsecAh
   set  s_l1View3bIpsec.pktType.fL3PktTypeIpsec
   
#ifndef PASS_PROC_FIREWALL
  // Check to see whether virtual link is used 
  qbbs l_c1ParseAuth_vLink, s_pktCxt.flags.t_flag_use_vlink
    // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.phyLink
    jmp  l_c1ParseAuth_link_end
l_c1ParseAuth_vLink:    
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.vlanPri_vLink
    and  s_l1View3bIpsec.srcVC.b1, s_l1View3bIpsec.srcVC.b1, NOT_PKT_VLAN_PRI_MASK
    // Clear virtual link enable for further look ups
    clr s_pktCxt.flags.t_flag_use_vlink

l_c1ParseAuth_link_end:
#else
    mov  s_l1View3bIpsec.srcVC,  s_pktCxt.phyLink
#endif   
   
   xout XID_LUT1V3,   s_l1View3bIpsec, SIZE(s_l1View3bIpsec)

   mov  s_pktCxt.espAhOffset,  s_pktCxt.startOffset
   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_AUTH

   // Handle length. the length field in AH is the authentication length, in 32
   // bit values, -2. Since the AH header length is 3 32 bit words the 
   // addition of 1 to the length field is used.
   add  r0.w0,               s_ah.len,              2
   lsl  r0.w0,               r0.w0,                 2
   add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  r0.w0

l_c1ParseAuth0:

   // Get the next header type and action from the protocol field,
   // but the action must be overridden to force a lookup
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_ah.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   mov    s_param.action,    SUBS_ACTION_LOOKUP

   ret
   
#endif   

    .leave cdeScope
    .leave pktScope
    .leave lut1Scope
    .leave secScope
    
// **************************************************************************************
// * FUNCTION PURPOSE: Parse an SCTP header
// **************************************************************************************
// * DESCRIPTION: The SCTP port number is added to the LUT. There is no information
// *              available about the next header
// *              
// *              The offset is not updated
// *
// *    On entry:
// *            - the CDE is at the start of the SCTP header
// *            - r30.w0 has the function return address
// *            - param.action has SUBS_ACTION_PARSE
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVNACE
// *
// *    On exit:
// *            - param.action has SUBS_ACTION_LOOKUP
// *            - cdeCmdWd.operation has CDE_CMD_WINDOW_ADVANCE
// *            - CDE View window does not move
// *            - startOffset is adjusted by the size of SCTP header
// *            - s_next.Hdr is set as PA_HDR_UNKNOWN
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
// *   R3:              b0 - next header type  - pktScope
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R5:    |                   -
// *   R6:        |                                  |  IPv6 header & option headers
// *   R7:        |                                  |  
// *   R8:        |                                    
// *   R9:        |  LUT1 View   - lut1Scope
// *   R10:       |                                     | 
// *   R11:       |                                     |  LUT1 View3
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  SCTP Header    |  Extended Packet Header                                    
// *   R15:          |                 |                         
// *   R16:          |                 |                         
// *   R17:             | not used     |             
// *   R18:             |              |
// *   R19:             |
// *   R20:           | Checksum contrl block
// *   R21:           |
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
// *
// **************************************************************************************

    .using cdeScope
    .using lut1Scope
    .using pktScope
    .using sctpScope

f_c1ParseSctp:

#ifdef PASS_PROC_FIREWALL
    // Read in the SCTP header 
    xin  XID_CDEDATA,  s_sctp,    SIZE(s_sctp)
   
    mov  s_l1View3bIpv6.dstPort,    s_sctp.dst    
    mov  s_l1View3bIpv6.srcPort,    s_sctp.src   

    xout XID_LUT1V3,  s_l1View3bIpv6.srcPort,  SIZE(s_l1View3bIpv6.srcPort) + SIZE(s_l1View3bIpv6.dstPort)

    mov s_param.action, SUBS_ACTION_LOOKUP
    
    ret
    
#else    

#ifdef PASS_PROC_L3

   // Read in the SCTP header 
   xin  XID_CDEDATA,  s_sctp,    SIZE(s_sctp)
   
   // Save and update View3
   xin XID_LUT1V3,        s_l1View3bIpv6.pktFlags,   SIZE(s_l1View3bIpv6.pktFlags)
   set  s_l1View3bIpv6.pktFlags.fSctp
   xout XID_LUT1V3,       s_l1View3bIpv6.pktFlags,   SIZE(s_l1View3bIpv6.pktFlags)
   
   mov  s_l1View3bIpv6.dstPort, s_sctp.dst
   xout XID_LUT1V3,     s_l1View3bIpv6.dstPort, SIZE(s_l1View3bIpv6.dstPort)
   
   mov  s_next.hdr,     PA_HDR_UNKNOWN
   mov  s_param.action, SUBS_ACTION_LOOKUP
   
   // Mark SCTP
   set  s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_SCTP
   
   // Store, clear checksum and Issue CRC checksum command only if enabled
   qbbc  l_c1ParseSctp_end, s_runCxt.flags.t_sctpChksum
   
   mov  s_crcCxt.crc, s_sctp.chksum
   add  s_crcCxt.offset, s_pktCxt.startOffset, OFFSET(s_sctp.chksum)
   mov  s_crcCxt.crcSize,  4
     
   zero &s_sctp.chksum, SIZE(s_sctp.chksum)
   xout  XID_CDEDATA,  s_sctp.chksum,  SIZE(s_sctp.chksum)
   
   // Pop the extended info and issue CRC operation
   xin   XID_PINFO_A,   s_pktExtDescr,  SIZE(s_pktExtDescr)
   
   set   s_pktExtDescr.flags.fDoCRC
   sub   s_pktExtDescr.crcLength,   s_pktCxt.endOffset,         s_pktCxt.startOffset 
   mov   s_pktExtDescr.crcOffset,   s_pktCxt.startOffset
   mov32 s_pktExtDescr.crcValue,    0xFFFFFFFF   
   
   xout  XID_PINFO_A,   s_pktExtDescr,  SIZE(s_pktExtDescr)
   
   // Set flag to indicate that CRC checksum verification is required
   // Append the CRC information to the end of pkt context in the PS Info area
   set  s_pktCxt.flags.t_flag_crc_verify
   
   sbco s_crcCxt,  cCdeOutPkt, SIZE(s_pktDescr) + SIZE(s_pktCxt), SIZE(s_crcCxt) 

l_c1ParseSctp_end:
   // Adjust the start offset to end of SCTP header
   mov  s_pktCxt.l4Offset,     s_pktCxt.startOffset
   add  s_pktCxt.startOffset,  s_pktCxt.startOffset, SCTP_HEADER_LEN_BYTES
   ret
   
#endif   
#endif

    .leave cdeScope
    .leave lut1Scope
    .leave pktScope
    .leave sctpScope
    
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
// *            - TBD: the CDE is at the first byte after the UDP header
// *            - TBD: startOffset is adjusted by the UDP header size 
// *            - TBD: s_next.Hdr is set as PA_HDR_UNKNOWN if non-GTPU, otherwise, it is set to PA_HDR_GTPU
// *            
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      | UDP header 
// *   R7:        |                                      |  
// *   R8:        |                                      |  
// *   R9:        |  LUT1 View   - lut1Scope             |
// *   R10:       |                                     |
// *   R11:       |                                     |  LUT1 View1
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  |
// *   R19:          |                                  |  LUT1 View3
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ***********************************************************************************************

    .using cdeScope
    .using pktScope
    .using udpScope
    .using lut1Scope

f_c1ParseUdp:

#ifdef PASS_PROC_LUT1_L4
    // Read in the UDP header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udp,  SIZE(s_udp)
    
    mov  s_l1View3dIpv4.dstPort,    s_udp.dst    
    mov  s_l1View3dIpv4.srcPort,    s_udp.src   

    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)

    mov s_param.action, SUBS_ACTION_LOOKUP

  // When EOAM feature is enabled, check if IP processing is already done
  // If done, there is no action
  // otherwise initiate a call to have Outer IP for ACL
  qbbc  l_c1ParseUdp_1, s_runCxt.flag3.t_eoamEn
  
#ifndef PASS_PROC_IP_REASSEM  
    // is IP processing done in this stage?
    qbbs l_c1ParseUdp_1, s_pktCxt.flags.t_flag_ipProc
      // Jmp to submit outer IP information
      jmp  f_c1ParseIpForEoamAcl

l_c1ParseUdp_1: 
    // Clear the information
    clr  s_pktCxt.flags.t_flag_ipProc
#endif
    ret
    
#else 

#ifdef PASS_PROC_IPSEC_NAT_T
    
    qbbc  l_c1ParseUdpNatT_end,  s_runCxt.flag2.t_IpsecNatTDetEn

    // Read in the UDP header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udp,  SIZE(s_udp) + SIZE(s_ipsecNatT)
    
    // A switch to IPSEC_NAT_T mode is made based on destination or source UDP port
    lbco  r0.w0,  PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG + OFFSET(struct_ipsec_nat_t_cfg.udpPort),  SIZE(struct_ipsec_nat_t_cfg.udpPort)
    qbeq  l_c1ParseUdpNatT_1,   s_udp.dst,  r0.w0 
    qbne  l_c1ParseUdpNatT_end, s_udp.src,  r0.w0

l_c1ParseUdpNatT_1:
       // IPSEC NAT_T parsing
       // Keepalive packet: UDP length == 9, payload = 0xFF
       // Control packet: UDP length > 12 SPI = 0
       // IPSEC Data Packet: UDP length > 12 SPI != 0
       // Error Packet UDP length <= 12
       
       // Common Operations for all NAT_T packet
       set s_pktCxt.hdrBitmask.SUBS_PA_BIT_HEADER_IPSEC_NAT_T
       mov s_next.Hdr,  PA_HDR_UNKNOWN
       and s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
       mov s_param.action, SUBS_ACTION_EXIT

       qbge l_c1ParseUdpNatT_3, s_udp.len, SIZE(s_udp) + SIZE(s_ipsecNatT) 
            qbeq l_c1ParseUdpNatT_2, s_ipsecNatT.spi, 0    
                // IPSEC Data packet
                mov s_next.Hdr,  PA_HDR_ESP
                mov s_param.action, SUBS_ACTION_PARSE
                
                //  Advance past the UDP header.
                mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
                mov   s_cdeCmdWd.byteCount,   SIZE(s_udp)
                xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
                
                add   s_pktCxt.startOffset, s_pktCxt.startOffset, SIZE(s_udp)
                
                ret   
       
l_c1ParseUdpNatT_2: 
                // NAT_T Control packet      
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_CTRL << PKT_EIDX_SHIFT
                ret
                
l_c1ParseUdpNatT_3:
            qbne l_c1ParseUdpNatT_4,    s_udp.len,          SIZE(s_udp) + SIZE(s_ipsecNatT2)
            qbne l_c1ParseUdpNatT_4,    s_ipsecNatT2.data,  0xFF
                // NAT-T Keepalive Packet
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_KEEPALIVE << PKT_EIDX_SHIFT
                ret
       
l_c1ParseUdpNatT_4:       
                // NAT-T Error Packet
                or  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,  EROUTE_NAT_T_FAIL << PKT_EIDX_SHIFT
                ret
                
l_c1ParseUdpNatT_end:
    // normal UDP packets are sent to Ingress4 for L4 classification
    mov s_param.action,  SUBS_ACTION_FWPKT3
    ret
    
#endif    
   
#endif
    .leave cdeScope
    .leave pktScope
    .leave udpScope
    .leave lut1Scope


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
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      | UDP-Lite Header 
// *   R7:        |                                      |  
// *   R8:        |                                      |  
// *   R9:        |  LUT1 View   - lut1Scope             |
// *   R10:       |                                     |
// *   R11:       |                                     |  LUT1 View1
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  |
// *   R19:          |                                  |  LUT1 View3
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// ************************************************************************************************
    .using cdeScope
    .using pktScope
    .using udpScope
    .using lut1Scope

f_c1ParseUdpLite:

#ifdef PASS_PROC_LUT1_L4

    // Read in the UDP lite header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udpLite,  SIZE(s_udpLite)

l_c1ParseUdpLite1:

    // Do the lookup
    mov  s_l1View3dIpv4.dstPort,    s_udpLite.dst    
    mov  s_l1View3dIpv4.srcPort,    s_udpLite.src   

    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)
    
    mov s_param.action, SUBS_ACTION_LOOKUP
#ifndef PASS_PROC_IP_REASSEM    
  // When EOAM feature is enabled, check if IP processing is already done
  // If done, there is no action
  // otherwise initiate a call to have Outer IP for ACL
  qbbc  l_c1ParseUdpLite_1, s_runCxt.flag3.t_eoamEn
    // is IP processing done?
    qbbs l_c1ParseUdpLite_1, s_pktCxt.flags.t_flag_ipProc
      // Jmp to submit outer IP information
      jmp  f_c1ParseIpForEoamAcl
l_c1ParseUdpLite_1:     
    // Clear the information
    clr  s_pktCxt.flags.t_flag_ipProc
#endif
    ret

#endif

    .leave cdeScope
    .leave pktScope
    .leave udpScope
    .leave lut1Scope


// ***********************************************************************************
// * FUNCTION PURPOSE: Process a TCP header
// ***********************************************************************************
// * DESCRIPTION: The TCP header is parsed and a LUT2 initiated
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    scratch
// *   R2:    scratch
// *   R3:    scratch          
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |                                      | TCP header 
// *   R7:        |                                      |  
// *   R8:        |                                      |  
// *   R9:        |  LUT1 View   - lut1Scope             |
// *   R10:       |                                     || 
// *   R11:       |                                     |  LUT1 View1
// *   R12:       |                                     |
// *   R13:       |                                     |
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  LUT1 View  - lut1Scope                 
// *   R18:          |                                  |
// *   R19:          |                                  |  LUT1 View3
// *   R20:          |                                  |
// *   R21:          |                                  |
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
// *************************************************************************************

    .using cdeScope
    .using pktScope
    .using tcpScope
    .using lut1Scope

f_c1ParseTcp:

#ifdef PASS_PROC_LUT1_L4

    // Read in the TCP header
    xin  XID_CDEDATA,  s_tcp,  SIZE(s_tcp)
    
    // Firewall-specific matching of type

    // Configure and initiate the lookup
    mov  s_l1View3dIpv4.dstPort,    s_tcp.dst    
    mov  s_l1View3dIpv4.srcPort,    s_tcp.src   
    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)

    mov  s_param.action,  SUBS_ACTION_LOOKUP

#ifndef PASS_PROC_IP_REASSEM    
  // When EOAM feature is enabled, check if IP processing is already done
  // If done, there is no action
  // otherwise initiate a call to have Outer IP for ACL
  qbbc  l_c1ParseTcp_1, s_runCxt.flag3.t_eoamEn
    // is IP processing done?
    qbbs l_c1ParseTcp_1, s_pktCxt.flags.t_flag_ipProc
      // Jmp to submit outer IP information
      jmp  f_c1ParseIpForEoamAcl

l_c1ParseTcp_1:   
    // Clear the information
    clr  s_pktCxt.flags.t_flag_ipProc
#endif
    ret
    
#endif    

    .leave cdeScope
    .leave pktScope
    .leave tcpScope
    .leave lut1Scope

// ************************************************************************************
// * FUNCTION PURPOSE: Unkown header
// ************************************************************************************
// * DESCRIPTION:
// *
// *   Register Usage:  
// * 
// *   R0:    scratch
// *   R1:    
// *   R2:    
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
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// ************************************************************************************

    .using pktScope

f_c1ParseUnkn:

#ifdef PASS_PROC_FIREWALL
    // Terminate parsing
    mov s_param.action,        SUBS_ACTION_LOOKUP
    ret
#endif

   mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_PARSE_FAIL

   and  s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   NOT_PKT_EIDX_MASK
   or   s_pktCxt.eId_portNum_nextHdr.b1,  s_pktCxt.eId_portNum_nextHdr.b1,   EROUTE_PARSE_FAIL << PKT_EIDX_SHIFT  

c1pun1:
   mov s_param.action, SUBS_ACTION_EXIT
   ret
   
   
#ifdef PASS_PROC_FIREWALL   
fci_c1FirewallException:
    // Common processing for unsupported protocols 
    set s_runCxt.flags.t_failLookup
    mov s_param.action,         SUBS_ACTION_EXIT

    ret  
#endif    

    .leave pktScope    
    
    




