// ********************************************************************************
// * FILE PURPOSE: Packet parsing functions
// ********************************************************************************
// * FILE NAME: parse1.p
// *
// * DESCRIPTION: Contains the functions that actually parse packets
// *
// ********************************************************************************
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
// **********************************************************************************/

#ifdef PASS_PROC_L2
    .using  lut1Scope
    .using  pktScope
    .using  cdeScope
    .using  macVlanScope



f_c1ParseMac:

  set  s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_MAC
  
  // record the input EMA port number (1-4)
  // Note: It is one-based port number where 0 indicates that the packet is not from CPSW
  //       We can simply add the EMAC port without mask since this is only place that this field may be updated from initial value (0)
  and s_pktDescr.srcId, s_pktDescr.srcId, PKT_EMACPORT_MASK 
  add s_pktCxt.hdrBitmask3_frag_portNum,  s_pktCxt.hdrBitmask3_frag_portNum, s_pktDescr.srcId
  
  //Remove CRC from the endOffset if the packet is from CPSW
  qbeq  l_c1ParseMac_1, s_pktDescr.srcId, 0
    sub s_pktCxt.endOffset, s_pktCxt.endOffset, 4
  
l_c1ParseMac_1:  
  // Set key to indicate MAC operation
  set s_runCxt.keyFlags.t_pa_lut1_key_mac
  
  // update inport
  mov  s_l1View3a.IngressPort,  s_pktDescr.srcId
  xout XID_LUT1V3,           s_l1View3a.IngressPort,  SIZE(s_l1View3a.IngressPort)

  // Load the destination and source mac into the LUT1 search
  xin  XID_CDEDATA,   s_macAddr,  SIZE(s_macAddr)
  
  // Write the srcMAC and destMAC to the lut
  xout XID_LUT1V1,   s_l1View1a,    SIZE(s_l1View1a) - 4

  // cdeCmdWd.operation already has CDE_CMD_WINDOW_ADVANCE
  add  s_pktCxt.startOffset,   s_pktCxt.startOffset,  SIZE(s_macAddr)
  mov  s_cdeCmdWd.byteCount,   SIZE(s_macAddr)
  xout XID_CDECTRL,            s_cdeCmdWd,            SIZE(s_cdeCmdWd)
  
l_c1ParseMac_BM:  
  // Detect Broadcast & Multicast
  qbbc  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_01.t_eth_multicast_ind
       // assume it is muticast 
       set   s_pktCxt.flag.t_flag_mac_multicast   
  
l_c1ParseMac_BM_0:
  
  mov   r0, 0xFFFF
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_01, r0
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_23, r0
  qbne  l_c1ParseMac_BM_end,  s_macAddr.DstAddr_45, r0 
        // It is broadcast
        clr   s_pktCxt.flag.t_flag_mac_multicast 
        set   s_pktCxt.flag.t_flag_mac_broadcast

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
// *   R6:        |                                        |  Keep the MAC address 
// *   R7:        |                                        | 
// *   R8:        |                                        |
// *   R9:        |  LUT1 View1   - lut1Scope
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |                                          | 
// *   R15:          |                                          | MAC sub-layer header
// *   R16:          |                                          |
// *   R17:          |  LUT1 View2  - lut1Scope                 | | Stats Request
// *   R18:          |                                             | 
// *   R19:          |                                             |  ethertypes table
// *   R20:          |                                             |
// *   R21:          |                                             |
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
                lbco  s_fifoCb, PAMEM_CONST_PDSP_CXT,  OFFSET_PDSP0_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
                add   r1.b0,    s_fifoCb.in, 4
                and   r1.b0,    r1.b0,       0x1F
        
                qbne  l_c1ProcessTagOrLen000, s_fifoCb.out, r1.b0
                    // FIFO is full, bump the system error
                    set s_statsFlags.event.t_nSystemFail
                    jmp l_c1ProcessTagOrLen9_1

l_c1ProcessTagOrLen000:
                // Insert the request into the FIFO   
                add   r1.w2,   s_fifoCb.in, OFFSET_PDSP0_USR_STATS_FIFO
                sbco  s_usrStatsReq,  PAMEM_CONST_PDSP_CXT, r1.w2, SIZE(s_usrStatsReq)
                sbco  r1.b0,   PAMEM_CONST_PDSP_CXT,    OFFSET_PDSP0_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)     
                 
                jmp l_c1ProcessTagOrLen9_1        
        
        
l_c1ProcessTagOrLen00:        
        // Mark 802.3
        set  s_pktCxt.hdrBitmask3_frag_portNum.SUBS_PA_BIT_HEADER_802_3

        //  Copy the new ethertype value to the common location
        mov   s_tagOrLen.len,    s_tagOrLen.etype2

        // scroll past the 8 bytes of llc/snap header
        add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,  8
        


l_c1ProcessTagOrLen0:

    // Copy the ethertype value to LUT1 view 1
    mov  s_l1View1a.EtherType,  s_tagOrLen.len
    xout XID_LUT1V1,            s_l1View1a.EtherType,  SIZE(s_l1View1a.EtherType)


    // scroll past the ethertype/llc snap
    // cdeCmdWd.operation already has the value CDE_CMD_WINDOW_ADVANCE
    xout  XID_CDECTRL,   s_cdeCmdWd,          SIZE(s_cdeCmdWd)

    // Track the active parse
    add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_cdeCmdWd.byteCount

    // Load the ethertype table from memory  (move to avoid multile load)
    //lbco  s_ethertypes,  PAMEM_CONST_PARSE,  OFFSET_ETYPE_TABLE,  SIZE(s_ethertypes)


    // Determine the next header type based on the tag value
    // These are arranged in order of expected appearance to reduce cycles in
    // the common cases. Assume the action is to do a lookup, and change
    // as required

    mov s_param.action,  SUBS_ACTION_LOOKUP

.using  ipScope

    qbne  l_c1ProcessTagOrLen1,   s_tagOrLen.len,  s_ethertypes.ip
        // Store DSCP in case we need DSCP priority routing
        xin  XID_CDEDATA,  s_Ip,   OFFSET(s_Ip.TotalLen) 
        lsr s_pktCxt.dscpPriority, s_Ip.Tos, 2
        mov s_next.Hdr,  PA_HDR_IPv4
        ret
        
l_c1ProcessTagOrLen1:

    qbne l_c1ProcessTagOrLen2,    s_tagOrLen.len,  s_ethertypes.ipv6
        // Store DSCP in case we need DSCP priority routing
        xin  XID_CDEDATA,  s_Ipv6a,   OFFSET(s_Ipv6a.payloadLen)
        lsr  s_pktCxt.dscpPriority,   s_Ipv6a.ver_tclass_flow.w2,  4
        lsr  s_pktCxt.dscpPriority,   s_pktCxt.dscpPriority,    2
        mov s_next.Hdr,  PA_HDR_IPv6
        ret
.leave ipScope


l_c1ProcessTagOrLen2:
 
    qbne l_c1ProcessTagOrLen3,    s_tagOrLen.len,  s_ethertypes.vlan
        mov s_param.action,  SUBS_ACTION_PARSE
        mov s_next.Hdr,      PA_HDR_VLAN
        ret


l_c1ProcessTagOrLen3:

    qbne l_c1ProcessTagOrLen4,    s_tagOrLen.len,   s_ethertypes.SpVlan
        mov s_param.action,  SUBS_ACTION_PARSE
        mov s_next.Hdr,      PA_HDR_VLAN
        ret

l_c1ProcessTagOrLen4:

    qbne l_c1ProcessTagOrLen5,    s_tagOrLen.len,   s_ethertypes.mpls
        mov s_param.action,  SUBS_ACTION_PARSE
        mov s_next.Hdr,  PA_HDR_MPLS
        ret

l_c1ProcessTagOrLen5:

    qbne l_c1ProcessTagOrLen6,    s_tagOrLen.len,   s_ethertypes.mplsMulti
        mov s_param.action,  SUBS_ACTION_PARSE
        mov s_next.Hdr,      PA_HDR_MPLS
        ret

l_c1ProcessTagOrLen6:

    qbne l_c1ProcessTagOrLen6_1,    s_tagOrLen.len,   s_ethertypes.PPPoE
        mov s_param.action,  SUBS_ACTION_PARSE
        mov s_next.Hdr,      PA_HDR_PPPoE
        ret
        
l_c1ProcessTagOrLen6_1:
    qbne l_c1ProcessTagOrLen7,      s_tagOrLen.len,   s_ethertypes.PPPoE_discov
    // Mark PPPoE packet and then pass through
   set  s_pktCxt.hdrBitmask3_frag_portNum.SUBS_PA_BIT_HEADER_PPPoE

l_c1ProcessTagOrLen7:  
    // Unknown ethertype  proceed to lookup with next header type unknown.
    mov s_next.Hdr,         PA_HDR_UNKNOWN
    
    // 802.1ag detection
    qbbc    l_c1ProcessTagOrLen8, s_runCxt.flag2.t_802_1agDet
        // Common rule (draft and standard) 01 80 C2 XX XX XX or 01 80 C2 00 00 3X
        mov     r0.w0,  ETH_TYPE_802_1AG
        qbne    l_c1ProcessTagOrLen8,   s_tagOrLen.len, r0.w0   
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_01.b1,    0x01  
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_01.b0,    0x80
        qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_23.b1,    0xc2
        // draft or standard
        qbbc    l_c1ProcessTagOrLen7_1, s_runCxt.flag2.t_802_1agStd 
            qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_23.b0,    0x00
            qbne    l_c1ProcessTagOrLen8,   s_macAddr.DstAddr_45.b1,    0x00
            and     r0.b0,  s_macAddr.DstAddr_45.b0,    0xF0
            qbne    l_c1ProcessTagOrLen8,   r0.b0,   0x30
        
l_c1ProcessTagOrLen7_1:  
        // 802.1ag packet detected      
        //   Note: save one instruction since pl1Match is not important for exception route
        mov  s_pktCxt2.eP1C2Id,  EROUTE_802_1ag << PKT2_EIDX_SHIFT
        mov  s_param.action,     SUBS_ACTION_EXIT

l_c1ProcessTagOrLen8:
    ret


l_c1ProcessTagOrLen9:

    //  LLC/SNAP failure
    set  s_statsFlags.event.t_nLlcSnapFail
    
l_c1ProcessTagOrLen9_1:    
    //   Note: save one instruction since pl1Match is not important for exception route
    mov  s_pktCxt2.eP1C2Id,  EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT
    mov  s_param.action,     SUBS_ACTION_EXIT
    ret 

    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave macVlanScope


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
// *
// *
// ***********************************************************************************
    .using pktScope
    .using checkScope
    .using cdeScope
    .using lut1Scope
    .using macVlanScope

f_c1ParseVlan:

  set  s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_VLAN

  // Check agains the max
  lsr  r0.w0,   s_pktCxt.protCount, PROT_COUNT_VLAN_SHIFT
  lbco  s_paMaxHdrCount.vlanMaxCount,  PAMEM_CONST_CUSTOM,  OFFSET_MAX_HDR + OFFSET(s_paMaxHdrCount.vlanMaxCount), 2

  qbgt l_c1ParseVlan1,  r0.w0,  s_paMaxHdrCount.vlanMaxCount
      or     s_pktCxt2.eP1C2Id, s_pktCxt2.eP1C2Id,        EROUTE_VLAN_MAX_DEPTH << PKT2_EIDX_SHIFT

l_c1ParseVlan0:
      mov s_param.action, SUBS_ACTION_EXIT
      ret

l_c1ParseVlan1:
  // Inc the vlan count
  add  s_pktCxt.protCount,   s_pktCxt.protCount,    PROT_COUNT_VLAN_STEP
   
  // Read in the pri/cfi/vlanId fields. Mask out pri/cfi
  xin  XID_CDEDATA,         s_vtag.tag,           SIZE(s_vtag.tag)
  mov  s_l1View1a.VLAN,     s_vtag.tag
  and  s_l1View1a.VLAN.b1,  s_l1View1a.VLAN.b1,   VLAN_VID_B1_MASK

  // store VLAN incase we need to do VLAN priority routing 
  lsr  s_pktCxt.vlanPriority, s_vtag.tag,         VLAN_PCP_SHIFT
  xout XID_LUT1V1,          s_l1View1a.VLAN,      SIZE(s_l1View1a.VLAN)

  add s_pktCxt.startOffset, s_pktCxt.startOffset, SIZE(s_vtag.tag)


  // Scroll the CDE past the tag to point to the next ethertype candidate
  mov  s_cdeCmdWd.byteCount,  SIZE(s_vtag.tag)
  xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)

  jmp f_c1ProcessTagOrLen

      
    .leave pktScope
    .leave checkScope
    .leave cdeScope
    .leave lut1Scope
    .leave macVlanScope
    
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// *************************************************************************************

#ifdef PASS_PROC_L3
    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using ipScope

f_c1ParseIpv4:

  // Ipv4 reassembly operation
  qbbs fci_c1ParseIpv4_0,    s_pktCxt.flag2.t_flag2_cascaded_forwarding    
  qbbs f_c1IpReassm,         s_runCxt.flags.t_ipReassmEn   

fci_c1ParseIpv4_0:
  // New IPv4 packet or Reassembled packet: check maximum count
  lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.ipMaxCount), 2

  // IP depth count
  and  r0.w0,         s_pktCxt.protCount,  PROT_COUNT_IP_MASK
  qblt l_c1ParseIpv4_00,   r0.w2,  r0.w0
      //  and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK
      //  or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   EROUTE_IP_MAX_DEPTH << PKT2_EIDX_SHIFT
      //  Note: save one instruction since pl1Match and customC2 is not important for exception route
      mov s_pktCxt2.eP1C2Id, EROUTE_IP_MAX_DEPTH << PKT2_EIDX_SHIFT      

      set s_statsFlags.event.t_nIpDepthOverflow
      mov s_param.action,        SUBS_ACTION_EXIT
      ret

l_c1ParseIpv4_00:  

  add  s_pktCxt.protCount,  s_pktCxt.protCount, PROT_COUNT_IP_STEP
  set  s_pktCxt.hdrBitmask_nexthdr.SUBS_PA_BIT_HEADER_IP
  set  s_statsFlags.event.t_nIpv4Packets

  // PDSP ID stored in s_runCxt.flags in this routine would either be 1 or 2
  qbbs l_c1ParseIpv4_00_no_extra_stats, s_runCxt.flags, 0
  set  s_statsFlags.event.t_nIpv4PacketsInner

l_c1ParseIpv4_00_no_extra_stats:
  //
  // t_l3toInnerIp = FALSE: Set l3Offset for outer IP 
  // t_l3toInnerIp = TRUE:  Set l3Offset for inner IP
  //
  qbbs l_c1ParseIpv4_01_setInnerIp, s_runCxt.flag2.t_l3toInnerIp
    qbne l_c1ParseIpv4_01,    s_pktCxt.l3Offset, 0
l_c1ParseIpv4_01_setInnerIp:      
        mov  s_pktCxt.l3Offset, s_pktCxt.startOffset

l_c1ParseIpv4_01:
  // Save the inner most IP offset all the time
  mov  s_pktCxt6.l3l5Offset, s_pktCxt.startOffset

  // Check to see whether virtual link is used 
  qbbs l_c1ParseIpv4_01_vLink, s_pktCxt.eP1C2IdIdx.t_vlinkEn
    // The previous lookup PDSP ID is stored in the ethertype field, previous LUT1 index in vlan
    lsr  s_l1View1a.EtherType,  s_pktCxt.eP1C2IdIdx,    SUBS_PKT_CXT_L1ID_BIT_OFFSET
    and  s_l1View1a.EtherType,  s_l1View1a.EtherType,   SUBS_PKT_CXT_L1ID_SHIFTED_MASK

    and  s_l1View1a.VLAN,       s_pktCxt2.IdIdx,        L1IDX_MASK

    jmp l_c1ParseIpv4_01_issue_lookup

  // Populate lookup using virtual link
  l_c1ParseIpv4_01_vLink: 
    mov s_l1View1a.EtherType, 0x3
    mov s_l1View1a.VLAN,    s_pktCxt.vLinkNum
    
    // Clear virtual link enable for further look ups
    clr s_pktCxt.eP1C2IdIdx.t_vlinkEn

  l_c1ParseIpv4_01_issue_lookup:
  xout XID_LUT1V1,            s_l1View1a.EtherType,   SIZE(s_l1View1a.EtherType)+SIZE(s_l1View1a.VLAN)


  // Read the minimum IP header
  xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 
  
  // Verify version
  and   r0,               s_Ip.VerLen,  0xF0
  qbne  fci_c1ParseIpv4_10, r0,           0x40
  
  // Strong error check required by some customers: (Disabled by default)
  // ipLength <= 20
  // ipHdrLen < 20
  // src addr == 0xffffffff (broadcast)
  // dest addr == 0
  // ttl == 0
  qbbc  l_c1ParseIpv4_000,   s_runCxt.flag2.t_ipHdrCheck
    qbge fci_c1ParseIpv4_10, s_Ip.TotalLen,  IPV4_MIN_HDR_SIZE       // Total length check
    and  r0.b0,  s_Ip.VerLen,   0x0F
    qbgt fci_c1ParseIpv4_10,    r0.b0,       IPV4_MIN_HDR_SIZE >> 2  // IP Hdr length check
    qbeq fci_c1ParseIpv4_10,    s_Ip.DstIp,     0                  // Dest IP == 0
    qbeq fci_c1ParseIpv4_10,    s_Ip.Ttl,       0                  // TTL == 0
    // IP Source IP check == 0xFFFFFFFF 
    // Method 1: Assignment plus two 16-bit comparison (3 instruction, 3 cycles)
    // Method 2: Search for zero and then check result (2 instruction, 2 cycles)
    lmbd r0,    s_Ip.SrcIp,     0    
    qbeq fci_c1ParseIpv4_10,      r0,        32                     
        
    // IPv4 Hdr check complete: pass through

l_c1ParseIpv4_000:
  // Fragmentation check
  mov   r3.w0,    IPV4_FRAG_MASK
  and   r3.w0,    s_Ip.FragOff,     r3.w0
  qbeq  l_c1ParseIpv4_0,  r3.w0,            0
      set  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
      set  s_statsFlags.event.t_nIpFrag

l_c1ParseIpv4_0:

  // SrcIp3 already in the correct location
  mov  s_l1View2a.DstIp_3,   s_Ip.DstIp
  zero &s_l1View2a.SrcIp_0,  OFFSET(s_l1View2a.SrcIp_3) - OFFSET(s_l1View2a.SrcIp_0)
  zero &s_l1View2a.DstIp_0,  OFFSET(s_l1View2a.DstIp_3) - OFFSET(s_l1View2a.DstIp_0)
  xout  XID_LUT1V2,          s_l1View2a,                  SIZE(s_l1View2a)

  // Read back in the IP header info
  xin  XID_CDEDATA,    s_Ip,   SIZE(s_Ip)
  
  // Setup the CDE for the IP header checksum, set the global pointer
  // past the IP header
  zero &s_cdeCmdChk,            SIZE(s_cdeCmdChk)
  and   r2.w0,                  s_Ip.VerLen,                    0x0f
  lsl   s_cdeCmdChk.byteLen,    r2.w0,                          2  // 32 bit size to 8 bit size conversion
  mov   s_cdeCmdChk.operation,  CDE_CMD_CHECKSUM1_VALIDATE
  xout  XID_CDECTRL,            s_cdeCmdChk,                    SIZE(s_cdeCmdChk)

  // Update the start and end offsets while the IP header length is in bytes
  add   s_pktCxt.endOffset,     s_pktCxt.startOffset,           s_Ip.TotalLen
  add   s_pktCxt.startOffset,   s_pktCxt.startOffset,           s_cdeCmdChk.byteLen

  // Compute the pseudo header checkusm except protocol and payload length
  add    r1,               s_Ip.SrcIp,       s_Ip.DstIp
  adc    r1,               r1.w0,            r1.w2
  add    s_pktCxt.pseudo,  r1.w0,            r1.w2
  
  // Broadcast and Multicast IP detection
l_c1ParseIpv4_BM:  
   // IP Destination IP broadcast == 0xFFFFFFFF 
   // Method 1: Assignment plus two 16-bit comparison (3 instruction, 3 cycles)
   // Method 2: Search for zero and then check result (2 instruction, 2 cycles)
  //mov    r1.w0,               0xffff
  //qbne   l_c1ParseIpv4_BM_1,  s_Ip.DstIp.w2,   r1.w0
  //qbne   l_c1ParseIpv4_BM_1,  s_Ip.DstIp.w0,   r1.w0
    lmbd r1.b0,    s_Ip.DstIp,     0    
    qbne l_c1ParseIpv4_BM_1,       r1.b0,        32    // 0 found?                 
        set  s_pktCxt.flag.t_flag_ip_broadcast
        jmp  l_c1ParseIpv4_BM_end
    
l_c1ParseIpv4_BM_1:
  qbgt   l_c1ParseIpv4_BM_end, s_Ip.DstIp.b3,   IPV4_MULTICAST_START     
  qblt   l_c1ParseIpv4_BM_end, s_Ip.DstIp.b3,   IPV4_MULTICAST_END
    set  s_pktCxt.flag.t_flag_ip_multicast     
  
l_c1ParseIpv4_BM_end:

  // Get the next action and header type from the IP protocol field
  lbco   r0.b0,             PAMEM_CONST_IP_PROTO,  s_Ip.Protocol,   1
  and    s_next.Hdr,        r0.b0,                 0x3f
  lsr    s_param.action,    r0.b0,                 6
  
  //  Advance past the IP header.
  mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
  xout   XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
  
   // The protocol type and TOS must be added
   // to the LUT1 search

   mov  s_l1View3a.Protocol,  s_Ip.Protocol
   mov  s_l1View3a.Tos,       s_Ip.Tos

   // Store DSCP in case we need DSCP priority routing
   lsr s_pktCxt.dscpPriority, s_Ip.Tos, 2

   xout XID_LUT1V3,           s_l1View3a.Protocol,  SIZE(s_Ip.Protocol)+SIZE(s_Ip.Tos)

   set s_runCxt.keyFlags.t_pa_lut1_key_ipv4
   ret

  // IPv4 Parse error
l_c1ParseIpv4_9:
   //   and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK
   //   or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT
   //   Note: save one instruction since pl1Match is not important for exception route
   mov  s_pktCxt2.eP1C2Id,  EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT
   jmp  l_c1ParseIpv4_11
   
  // IPv4/v6 Hdr error
fci_c1ParseIpv4_10:
   //   Note: save one instruction since pl1Match is not important for exception route
   mov  s_pktCxt2.eP1C2Id,  EROUTE_IP_FAIL << PKT2_EIDX_SHIFT
   
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
// *   R1:    scratch
// *   R2:    scratch
// *   R3:              b0 - next header type  - pktScope   (at output only)
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
// *   R18:           |
// *   R19:           |
// *   R20:           |
// *   R21:           |
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
// *************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using ipScope

f_c1IpReassm:

  qbbs l_c1IpReassm_pass2,  s_pktCxt.flag.t_flag_2nd_pass  

// First Pass operations
l_c1IpReassm_pass1:

  // Read the minimum IP header
  xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 

  // Verify version
  and   r0,                s_Ip.VerLen,  0xF0
  qbne  fci_c1ParseIpv4_10,  r0,         0x40
  
  mov   r1.w0,    IPV4_FRAG_MASK
  and   r1.w0,    s_Ip.FragOff,         r1.w0
  qbeq  l_c1IpReassm_pass1_0,  r1.w0,            0
      //set  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
      set  s_statsFlags.event.t_nIpFrag
  
l_c1IpReassm_pass1_0:
  
  // 
  // Search through the active Traffic flows
  //
  lbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  mov   s_ipReassmCxt.tfMapTemp, s_ipReassmCxt.tfMap  
  
l_c1IpReassm_pass1_1:
  
  qbeq  l_c1IpReassm_pass1_2, s_ipReassmCxt.tfMapTemp, 0
  lmbd  s_pktCxt4.tfIndex, s_ipReassmCxt.tfMapTemp, 1 
  clr   s_ipReassmCxt.tfMapTemp, s_pktCxt4.tfIndex  
  lsl   s_ipReassmCxt.offset,   s_pktCxt4.tfIndex, 4
  lbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  qbne  l_c1IpReassm_pass1_1, s_ipTf.srcIp, s_Ip.SrcIp
  qbne  l_c1IpReassm_pass1_1, s_ipTf.destIp, s_Ip.DstIp
  qbne  l_c1IpReassm_pass1_1, s_ipTf.proto, s_Ip.Protocol
  
  //match found: update and store the counter
  add   s_ipTf.cnt, s_ipTf.cnt, 1
  sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  OFFSET(s_ipTf.proto)
  
  jmp   l_c1IpReassm_pass1_4
  
l_c1IpReassm_pass1_2:
  //match not found
  // Fragmentation check (move up)
  // mov   r1.w0,    IPV4_FRAG_MASK
  // and   r1.w0,    s_Ip.FragOff,     r1.w0
  
  // normal operation for non-fragments packet
  qbbc  fci_c1ParseIpv4_0,  s_statsFlags.event.t_nIpFrag
  
  // Find available traffic flow
  qble  l_c1IpReassm_pass1_3, s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numTF  
  lmbd  s_pktCxt4.tfIndex, s_ipReassmCxt.tfMap, 0
  qbeq  l_c1IpReassm_pass1_3, s_pktCxt4.tfIndex, 32
  
  // Traffic Flow found
  set   s_ipReassmCxt.tfMap, s_pktCxt4.tfIndex
  add   s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF,  1
  sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  
  // store the traffic flow
  mov   s_ipTf.srcIp, s_Ip.SrcIp
  mov   s_ipTf.destIp, s_Ip.DstIp
  mov   s_ipTf.proto, s_Ip.Protocol
  mov   s_ipTf.cnt, 1
  lsl   s_ipReassmCxt.offset,   s_pktCxt4.tfIndex, 4
  sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  
  jmp    l_c1IpReassm_pass1_4
  
l_c1IpReassm_pass1_3:  
  //No Traffic flow available
  mov  s_pktCxt4.tfIndex, PA_INV_TF_INDEX 
  
  // pass through 
   
l_c1IpReassm_pass1_4:
  mov   s_pktCxt4.fragCnt,  1
  set   s_pktCxt.flag.t_flag_2nd_pass  
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
  clr   s_pktCxt.flag.t_flag_2nd_pass
  // TF index range check
  qbeq  l_c1IpReassm_pass2_3, s_pktCxt4.tfIndex, PA_INV_TF_INDEX
  qble  l_c1ParseIpv4_9,   s_pktCxt4.tfIndex,   32
  
  lsl   r1.w0,  s_pktCxt4.tfIndex, 4  //tfIndex * 16
  lbco  s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
  
  qbge  l_c1IpReassm_pass2_1,   s_ipTf.cnt,     s_pktCxt4.fragCnt
  
    sub  s_ipTf.cnt,    s_ipTf.cnt, s_pktCxt4.fragCnt
    sbco s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
    jmp  l_c1IpReassm_pass2_3
    
l_c1IpReassm_pass2_1:
    // Counter has reached zero, clear the corresponding bit
    lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    clr  s_ipReassmCxt.tfMap,   s_pktCxt4.tfIndex
    qbeq l_c1IpReassm_pass2_2,  s_ipReassmCxt.numActiveTF, 0
        sub  s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF, 1
                       
l_c1IpReassm_pass2_2:
    sbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    // pass through  
    
l_c1IpReassm_pass2_3:
    // verify whether it is an null packet  
    qbbc fci_c1ParseIpv4_0,  s_pktCxt.flag.t_flag_null_pkt  
        // drop the null packet
        mov   s_param.action,     SUBS_ACTION_DISCARD
        ret
       
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave ipScope
    
#endif

#ifdef PASS_PROC_L2

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
// ***************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using mplsScope

f_c1ParseMpls:

   set s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_MPLS
   set  s_statsFlags.event.t_nMplsPackets

   xin  XID_CDEDATA,  s_mpls,  SIZE(s_mpls)   // Read in a single tag

   // The tag is the 20 MSBs of the 32 bit word
   lsr  s_l1View3bMpls.mpls,  s_mpls.tagTtl,  12
   xout XID_LUT1V3,           s_l1View3bMpls.mpls,  SIZE(s_mpls.tagTtl)

   //set s_runCxt.keyFlags.t_pa_lut1_key_mpls
   add s_pktCxt.startOffset,                s_pktCxt.startOffset,  SIZE(s_mpls.tagTtl)

   mov   s_cdeCmdWd.byteCount,  SIZE(s_mpls.tagTtl)
   xout  XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)

   // Always perform a lookup after finding an MPLS headeer
   mov  s_param.action,  SUBS_ACTION_LOOKUP

   // Extract the S bit. If set then get a potential IP version number
   // For possible IPv4 and IPv6 continue parse

   qbbc  l_c1ParseMpls1,  s_mpls.tagTtl.t_s
       lsr  r0.b0,  s_mpls.ipVer,   4
       qbne l_c1ParseMpls0,  r0.b0,   4
           mov   s_next.hdr,          PA_HDR_IPv4
           ret

l_c1ParseMpls0:
       qbne l_c1ParseMpls1,  r0.b0,   6
           mov   s_next.hdr,          PA_HDR_IPv6
           ret
    
     
    // If there is more then one label or there is one label but the
    // next header doesnt look like IP, run the current packet through the LUT
l_c1ParseMpls1:
    mov  s_next.hdr,      PA_HDR_UNKNOWN
    ret
      
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
// ***************************************************************************************

    .using pktScope
    .using cdeScope
    .using lut1Scope
    .using pppoeScope

f_c1ParsePPPoE:
   xin  XID_CDEDATA,  s_pppoe,  SIZE(s_pppoe)   // Read in the PPPoE header
   
   // move over PPPoE header
   //mov   s_cdeCmdWd.byteCount,  OFFSET(s_pppoe.prot)
   //xout  XID_CDECTRL,           s_cdeCmdWd,           SIZE(s_cdeCmdWd)
   
   // Protocol header error check
   qbbc  l_c1ParsePPPoE_0, s_runCxt.flag2.t_PPPoEHdrCheck
     qbne  l_c1ParsePPPoE_3, s_pppoe.verType, PPPoE_VER_TYPE
     qbne  l_c1ParsePPPoE_3, s_pppoe.code,    PPPoE_CODE_SESSION
   
l_c1ParsePPPoE_0:
   // Mark PPPoE
   set  s_pktCxt.hdrBitmask3_frag_portNum.SUBS_PA_BIT_HEADER_PPPoE
   
   add s_pktCxt.startOffset,    s_pktCxt.startOffset,  OFFSET(s_pppoe.prot)
   add r0.w0,      s_pktCxt.startOffset,  s_pppoe.len
   // r0.w0 should be within the endOffset since end of PPPoE payload can not go beyond the payload end of
   // the previous protocol 
   qblt  l_c1ParsePPPoE_Error,   r0.w0,                s_pktCxt.endOffset
    mov   s_pktCxt.endOffset,    r0.w0

   // Always perform a lookup after finding an PPPoE headeer
   mov  s_param.action,  SUBS_ACTION_LOOKUP

   // Find out the next prot
   mov   r0.w0, PPPoE_PROT_IPv4           

   qbne  l_c1ParsePPPoE_1,  s_pppoe.prot, r0.w0
       add   s_pktCxt.startOffset,    s_pktCxt.startOffset, SIZE(s_pppoe.prot)
       mov   s_next.hdr,          PA_HDR_IPv4
       ret

l_c1ParsePPPoE_1:
   mov   r0.w0, PPPoE_PROT_IPv6
   qbne  l_c1ParsePPPoE_2,  s_pppoe.prot, r0.w0
       add   s_pktCxt.startOffset,    s_pktCxt.startOffset, SIZE(s_pppoe.prot)
       mov   s_next.hdr,          PA_HDR_IPv6
       ret
     
    // Unsupported protocol
    // next header doesn't look like IP;  Exception route
l_c1ParsePPPoE_2:
    mov s_pktCxt2.eP1C2Id, EROUTE_PPPoE_CTRL << PKT2_EIDX_SHIFT      
    jmp l_c1ParsePPPoE_4
    
    // PPPoE header Error: Exception route
l_c1ParsePPPoE_3:
    mov s_pktCxt2.eP1C2Id, EROUTE_PPPoE_FAIL << PKT2_EIDX_SHIFT      
    
    // pass through
l_c1ParsePPPoE_4:    
    // Common Error handling 
    mov s_param.action, SUBS_ACTION_EXIT
    mov  s_next.hdr,    PA_HDR_UNKNOWN
    ret
    
l_c1ParsePPPoE_Error:    
    mov s_pktCxt2.eP1C2Id, EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT 
    jmp l_c1ParsePPPoE_4     
      
    .leave pktScope
    .leave cdeScope
    .leave lut1Scope
    .leave pppoeScope
    
#endif 

#ifdef PASS_PROC_L3
   

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
// *   R0:    scratch
// *   R1:    scratch
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
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
  .u16   maxOffset
  .u16   tHdrLen
.ends

.assign struct_ipv6Reassm_ctrl,  r0,     r1,    s_ipv6Reassm_ctrl

#define PA_MAX_HDR_LEN                   256

f_c1ParseIpv6:

    // Reassembly-assisted operation
    qbbs l_c1Ipv6ExtReasm_pass2, s_pktCxt.flag.t_flag_null_pkt
        // Read in the IPv6 header.
        xin  XID_CDEDATA,  s_Ipv6a,   SIZE(s_Ipv6a)

        // Verify version
        and   r0.b0,                s_Ip.VerLen,   0xF0
        qbne  fci_c1ParseIpv4_10,   r0.b0,         0x60

        //  Advance past the fisr 8 byte (Ipv6a).
        zero &s_cdeCmdWd,            SIZE(s_cdeCmdWd)
        mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,   SIZE(s_Ipv6a)                      // Bytes always present in the header
        xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)

        xin  XID_CDEDATA,  s_Ipv6b,   SIZE(s_Ipv6b)

        // Source and dest IP go out unchanged
        xout XID_LUT1V2,   s_Ipv6b,   SIZE(s_Ipv6b)

        // If no reassm check
        qbbs l_c1ParseIpv6_main, s_pktCxt.flag2.t_flag2_cascaded_forwarding    
        qbbc l_c1ParseIpv6_main, s_runCxt.flags.t_ipReassmEn
        qbbs l_c1Ipv6ExtReasm_pass2, s_pktCxt.flag.t_flag_2nd_pass
        
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
        
l_c1Ipv6ExtReasm_exthdrCheck:    
    qble    l_c1Ipv6ExtReasm_exthdrDone,  s_ipv6Reassm_ctrl.offset,  s_ipv6Reassm_ctrl.maxOffset
    qbeq    l_c1Ipv6ExtReasm_fragfound,   s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_FRAG      
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_HOP_BY_HOP
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_ROUTE
    qbeq    l_c1Ipv6ExtReasm_exthdrNext,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_DEST_OPT
//   below would save one cycle since jmp is to header done.	
//  qbeq    l_c1Ipv6ExtReasm_exthdrDone,  s_ipv6Reassm_ctrl.nextHdr, IP_PROTO_NEXT_IPV6_NO_NEXT
    jmp     l_c1Ipv6ExtReasm_exthdrDone   
    
l_c1Ipv6ExtReasm_exthdrNext:        
    // load the next header
    lbco    s_ipv6Reassm_ctrl.nextHdr,  cCdeInPkt, s_ipv6Reassm_ctrl.offset, 2
    
    // adjust the offset for next one
    add     s_ipv6Reassm_ctrl.thdrLen,  s_ipv6Reassm_ctrl.hdrLen,  1
    lsl     s_ipv6Reassm_ctrl.tHdrLen,  s_ipv6Reassm_ctrl.thdrLen, 3
    add     s_ipv6Reassm_ctrl.offset,  s_ipv6Reassm_ctrl.offset, s_ipv6Reassm_ctrl.tHdrLen
    
    jmp     l_c1Ipv6ExtReasm_exthdrCheck
    
l_c1Ipv6ExtReasm_fragfound:
    lbco    s_ipv6Reassm_ctrl.nextHdr,  cCdeInPkt, s_ipv6Reassm_ctrl.offset, 1
    set     s_statsFlags.event.t_nIpFrag 

l_c1Ipv6ExtReasm_exthdrDone:        

l_c1Ipv6ExtReasm_pass1_1:

  // s_ipReassmCxt is r14 - r17, which overwrites s_Ipv6b.srcIp_3
  mov  r1, s_Ipv6b.srcIp_3
  lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  mov s_ipReassmCxt.tfMapTemp, s_ipReassmCxt.tfMap
  
l_c1Ipv6ExtReasm_pass1_2:
  qbeq l_c1Ipv6ExtReasm_pass1_3, s_ipReassmCxt.tfMapTemp, 0
        lmbd s_pktCxt4.tfIndex, s_ipReassmCxt.tfMapTemp, 1
        clr s_ipReassmCxt.tfMapTemp, s_pktCxt4.tfIndex
        lsl s_ipReassmCxt.offset, s_pktCxt4.tfIndex, 4
        lbco s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset, SIZE(s_ipTf)

        // Only compare the last 4 bytes of Ipv6 header
        // r1 now contains s_Ipv6b.srcIp_3
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.srcIp, r1
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.destIp, s_Ipv6b.dstIp_3
        qbne  l_c1Ipv6ExtReasm_pass1_2, s_ipTf.proto, s_ipv6Reassm_ctrl.nextHdr
  
        // match found: update and store the counter
        add   s_ipTf.cnt, s_ipTf.cnt, 1
        sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  OFFSET(s_ipTf.proto)
  
        jmp   l_c1Ipv6ExtReasm_pass1_5
  
l_c1Ipv6ExtReasm_pass1_3:
        //match not found
        // Fragmentation check  
        // normal operation for non-fragments packet
        qbbc  l_c1ParseIpv6_main,  s_statsFlags.event.t_nIpFrag
  
        // Find available traffic flow
        qble  l_c1Ipv6ExtReasm_pass1_4, s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numTF  
        lmbd  s_pktCxt4.tfIndex, s_ipReassmCxt.tfMap, 0
        qbeq  l_c1Ipv6ExtReasm_pass1_4, s_pktCxt4.tfIndex, 32
  
        // Traffic Flow found
        set   s_ipReassmCxt.tfMap, s_pktCxt4.tfIndex
        add   s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF,  1
        sbco  s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
  
        // store the traffic flow
        // r1 now contains s_Ipv6b.srcIp_3
        mov   s_ipTf.srcIp, r1
        mov   s_ipTf.destIp, s_Ipv6b.dstIp_3
        mov   s_ipTf.proto, s_ipv6Reassm_ctrl.nextHdr
        mov   s_ipTf.cnt, 1
        lsl   s_ipReassmCxt.offset,   s_pktCxt4.tfIndex, 4
        sbco  s_ipTf, PAMEM_CONST_TF_TABLE, s_ipReassmCxt.offset,  SIZE(s_ipTf)
  
        jmp   l_c1Ipv6ExtReasm_pass1_5
  
l_c1Ipv6ExtReasm_pass1_4:  
    // No Traffic flow available
    mov  s_pktCxt4.tfIndex, PA_INV_TF_INDEX 
  
  // pass through 
l_c1Ipv6ExtReasm_pass1_5:
    mov   s_pktCxt4.fragCnt,  1
    set   s_pktCxt.flag.t_flag_2nd_pass  
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
    clr   s_pktCxt.flag.t_flag_2nd_pass
    // TF index range check
    qbeq  l_c1Ipv6ExtReasm_pass2_3, s_pktCxt4.tfIndex, PA_INV_TF_INDEX
    qble  l_c1ParseIpv4_9,   s_pktCxt4.tfIndex,   32
  
    lsl   r1.w0,  s_pktCxt4.tfIndex, 4  //tfIndex * 16
    lbco  s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
  
    qbge  l_c1Ipv6ExtReasm_pass2_1,   s_ipTf.cnt,     s_pktCxt4.fragCnt
  
    sub  s_ipTf.cnt,    s_ipTf.cnt, s_pktCxt4.fragCnt
    sbco s_ipTf, PAMEM_CONST_TF_TABLE, r1.w0,  OFFSET(s_ipTf.srcIp)
    jmp  l_c1Ipv6ExtReasm_pass2_3
    
l_c1Ipv6ExtReasm_pass2_1:
    // Counter has reached zero, clear the corresponding bit
    lbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    clr  s_ipReassmCxt.tfMap,   s_pktCxt4.tfIndex
    qbeq l_c1Ipv6ExtReasm_pass2_2,  s_ipReassmCxt.numActiveTF, 0
    sub  s_ipReassmCxt.numActiveTF, s_ipReassmCxt.numActiveTF, 1
                       
l_c1Ipv6ExtReasm_pass2_2:
    sbco s_ipReassmCxt, PAMEM_CONST_IP_REASSEM_CONTEXT, 0, OFFSET(s_ipReassmCxt.tfMapTemp)
    // pass through
    
l_c1Ipv6ExtReasm_pass2_3:
    // verify whether it is an null packet  
    qbbc l_c1ParseIpv6_main,  s_pktCxt.flag.t_flag_null_pkt
    // drop the null packet
    mov   s_param.action,     SUBS_ACTION_DISCARD
    ret

l_c1ParseIpv6_main:

  // Restore packet context data
  xin  XID_CDEDATA,  s_Ipv6b,   SIZE(s_Ipv6b)
  
  // New IPv6 packet: check maximum count
  lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.ipMaxCount), 2

  // IP depth count
  and  r0.w0,         s_pktCxt.protCount,  PROT_COUNT_IP_MASK
  qblt l_c1ParseIpv6_00,   r0.w2,  r0.w0
      //  and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK
      //  or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   EROUTE_IP_MAX_DEPTH << PKT2_EIDX_SHIFT
      //  Note: save one instruction since pl1Match is not important for exception route
      mov s_pktCxt2.eP1C2Id, EROUTE_IP_MAX_DEPTH << PKT2_EIDX_SHIFT      

      set s_statsFlags.event.t_nIpDepthOverflow
      mov s_param.action,        SUBS_ACTION_EXIT
      ret

l_c1ParseIpv6_00:

   add  s_pktCxt.protCount, s_pktCxt.protCount, PROT_COUNT_IP_STEP
   set  s_statsFlags.event.t_nIpv6Packets

   // PDSP ID stored in s_runCxt.flags in this routine would either be 1 or 2
   qbbs l_c1ParseIpv6_00_no_extra_stats, s_runCxt.flags, 0
   set s_statsFlags.event.t_nIpv6PacketsInner

l_c1ParseIpv6_00_no_extra_stats:
   set  s_pktCxt.hdrBitmask_nexthdr.SUBS_PA_BIT_HEADER_IP
   set  s_runCxt.keyFlags.t_pa_lut1_key_ipv6
   //
   // t_l3toInnerIp = FALSE: Set l3Offset for outer IP 
   // t_l3toInnerIp = TRUE:  Set l3Offset for inner IP
   //
   qbbs l_c1ParseIpv6_01_setInnerIp, s_runCxt.flag2.t_l3toInnerIp
   qbne l_c1ParseIpv6_0,    s_pktCxt.l3Offset, 0 
l_c1ParseIpv6_01_setInnerIp:      
     mov  s_pktCxt.l3Offset, s_pktCxt.startOffset
   
l_c1ParseIpv6_0:
   // Save the inner most IP offset all the time
   mov  s_pktCxt6.l3l5Offset, s_pktCxt.startOffset

   // Check to see whether virtual link is used 
   qbbs l_c1ParseIpv6_0_vlink, s_pktCxt.eP1C2IdIdx.t_vlinkEn
   // The previous lookup PDSP ID is stored in the ethertype field, previous LUT1 index in vlan
   lsr  s_l1View1a.EtherType,  s_pktCxt.eP1C2IdIdx,    SUBS_PKT_CXT_L1ID_BIT_OFFSET
   and  s_l1View1a.EtherType,  s_l1View1a.EtherType,   SUBS_PKT_CXT_L1ID_SHIFTED_MASK

   and  s_l1View1a.VLAN,       s_pktCxt2.IdIdx,        L1IDX_MASK

   jmp l_c1ParseIpv6_0_issue_lookup

   // Populate lookup using virtual link
   l_c1ParseIpv6_0_vlink: 
      // virtual link pdsp num is always 0x3, virtuallink number is stored in s_pktCxt.vLinkNum
      mov  s_l1View1a.EtherType, 0x3
      mov  s_l1View1a.VLAN,     s_pktCxt.vLinkNum
      
      // Clear virtual link enable for further look ups
      clr s_pktCxt.eP1C2IdIdx.t_vlinkEn
   l_c1ParseIpv6_0_issue_lookup:

   xout XID_LUT1V1,            s_l1View1a.EtherType,   SIZE(s_l1View1a.EtherType)+SIZE(s_l1View1a.VLAN)

   // Begin the pseudo header checksum to free up LUT window 2
   // note: the pseudo header checksum does not include protocol and length
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
        set s_pktCxt.flag.t_flag_ip_multicast  
   
l_c1ParseIpv6_1:
   
   // The protocol, tclass, and flow labels are moved as 
   // required
   mov   s_l1View3b.FlowLabel,     s_Ipv6a.ver_tclass_flow
   and   s_l1View3b.FlowLabel.w2,  s_l1View3b.FlowLabel.w2,   0x0f
   xout  XID_LUT1V3,               s_l1View3b.FlowLabel,      SIZE(s_l1View3b.FlowLabel)

   mov   s_l1View3b.Protocol,      s_Ipv6a.next
   lsr   s_l1View3b.Tos,           s_Ipv6a.ver_tclass_flow.w2,  4

   // Store the DSCP priority bits in case we need DSCP priority routing 
   lsr   s_pktCxt.dscpPriority,    s_l1View3b.Tos, 2 
   xout  XID_LUT1V3,               s_l1View3b.Protocol,         SIZE(s_l1View3b.Protocol)+SIZE(s_l1View3b.Tos)

   // Update packet offsets
   add   s_pktCxt.startOffset,   s_pktCxt.startOffset,  IPV6_HEADER_LEN_BYTES
   add   s_pktCxt.endOffset,     s_pktCxt.startOffset,  s_Ipv6a.payloadLen

   // Get the next header and action
   lbco   r0.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6a.next,   1
   and    s_next.Hdr,        r0.b0,                 0x3f
   lsr    s_param.action,    r0.b0,                 6
   
   //  Advance past the always present IP header.
   //  Note: The other parameters are set previously
   mov   s_cdeCmdWd.byteCount,   IPV6_HEADER_LEN_BYTES - SIZE(s_Ipv6a) // Bytes always present in the header
   xout   XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   
   ret

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
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtHop:

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
   mov  s_l1View3b.Protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_Ipv6Opt.proto,  SIZE(s_Ipv6Opt.proto)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Opt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6
   ret

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
// *************************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtRoute:

   xin  XID_CDEDATA,  s_Ipv6ExtRt,   SIZE(s_Ipv6ExtRt)

   // Put the next proto type into the lookup
   mov  s_l1View3b.Protocol,  s_Ipv6ExtRt.proto
   xout XID_LUT1V3,           s_Ipv6ExtRt.proto,  SIZE(s_Ipv6ExtRt.proto)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6ExtRt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6

   //The following flag is not used any more.It is reserved for future enhancement
   //qbne  l_c1ParseIpv6ExtRoute0,   s_Ipv6ExtRt.segsleft,  0
   //    set s_pktCxt.hdrBitmask3_frag_portNum.t_ipMroute

l_c1ParseIpv6ExtRoute0:
   // The options header length is in 8 byte units, not including the 1st 8 bytes
   lsl  r0.w0,                 s_Ipv6ExtRt.hdrlen,    3
   qblt fci_c1ParseIpv4_10,    r0.w0,       (PA_MAX_HDR_LEN-1)      // Hdr length check   
   add  s_cdeCmdWd.byteCount,  r0.w0,                 8
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)
   add  s_pktCxt.startOffset,  s_pktCxt.startoffset,  s_cdeCmdWd.byteCount

   ret

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
// *************************************************************************************/
    .using cdeScope
    .using ipScope
    .using pktScope
    .using lut1Scope

f_c1ParseIpv6ExtFrag:

   // The fragmentation header is always 8 bytes. The whole 
   // header is read here, but only the 1st byte is used
   xin  XID_CDEDATA,           s_Ipv6Frag,      SIZE(s_Ipv6Frag)      
   mov  s_cdeCmdWd.byteCount,  SIZE(s_Ipv6Frag)
   xout XID_CDECTRL,           s_cdeCmdWd,      SIZE(s_cdeCmdWd)

   // Put the next proto type into the lookup
   mov  s_l1View3b.Protocol,  s_Ipv6Frag.proto
   xout XID_LUT1V3,           s_Ipv6Frag.proto,  SIZE(s_Ipv6Frag.proto)

l_c1ParseIpv6ExtFrag0:
   set  s_pktCxt.hdrBitmask3_frag_portNum.t_ipFrag
   add  s_pktCxt.startOffset,           s_pktCxt.startOffset,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
   set  s_statsFlags.event.t_nIpFrag

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Frag.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6
   ret

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
// *
// ***********************************************************************************

    .using cdeScope
    .using pktScope
    .using ipScope
    .using lut1Scope

f_c1ParseIpv6ExtDestOpt:

   xin  XID_CDEDATA,  s_Ipv6Opt,  SIZE(s_Ipv6Opt)

   // Put the next proto type into the lookup
   mov  s_l1View3b.Protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_Ipv6Opt.proto,  SIZE(s_Ipv6Opt.proto)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Opt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6

   lsl    r0.w0,                s_Ipv6Opt.optlen,     3
   qblt fci_c1ParseIpv4_10,     r0.w0,       (PA_MAX_HDR_LEN-1)      // Hdr length check   
   add    s_cdeCmdWd.byteCount, r0.w0,                8
   xout   XID_CDECTRL,          s_cdeCmdWd,           SIZE(s_cdeCmdWd)
   add    s_pktCxt.startOffset, s_pktCxt.startOffset, s_cdeCmdWd.byteCount

   ret
   
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
// *
// **********************************************************************************
    .using cdeScope
    .using pktScope
    .using greScope
    .using lut1Scope

f_c1ParseGre:

  // New GRE packet: check maximum count
  lbco  r0.w2,  PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR + OFFSET(struct_paComMaxCount.greMaxCount), 2

  // IP depth count
  lsr  r0.w0,   s_pktCxt.protCount,  PROT_COUNT_GRE_SHIFT
  and  r0.w0,   r0.w0,               PROT_COUNT_GRE_MASK
  qblt l_c1ParseGre00,   r0.w2,  r0.w0
      //  and  s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   NOT_PKT2_EIDX_MASK
      //  or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,   EROUTE_GRE_MAX_DEPTH << PKT2_EIDX_SHIFT
      //  Note: save one instruction since pl1Match and customC2 is not important for exception route
      mov s_pktCxt2.eP1C2Id, EROUTE_GRE_MAX_DEPTH << PKT2_EIDX_SHIFT      

      set s_statsFlags.event.t_nGreDepthOverflow
      mov s_param.action,        SUBS_ACTION_EXIT
      ret

l_c1ParseGre00:
  mov  s_param.action, SUBS_ACTION_LOOKUP
  add  s_pktCxt.protCount,  s_pktCxt.protCount,  PROT_COUNT_GRE_STEP

  //set  s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_GRE  (GRE can be recognized by the GRE count)
  //set  s_runCxt.keyflags.t_pa_lut1_key_gre

  //mov  s_pktCxt.l5Offset,  s_pktCxt.startOffset

  xin  XID_CDEDATA,           s_greHdr,      SIZE(s_greHdr)
  mov  s_cdeCmdWd.byteCount,  SIZE(s_greHdr)
  xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)

  add  s_pktCxt.startOffset,  s_pktCxt.startOffset,  SIZE(s_greHdr)  
  mov  s_cdeCmdWd.byteCount,  0
  
  // GRE checksum calculation
  qbbc  l_c1ParseGre0, s_greHdr.flags.t_GreCBit

#ifdef GRE_CHECKSUM_VALIDATE
      // Below code is not validated 
      zero &s_cdeCmdChk,           SIZE(s_cdeCmdChk)
      sub   s_cdeCmdChk.byteLen,   s_pktCxt.endOffset,            s_pktCxt.startOffset
      mov   s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_VALIDATE
      xout  XID_CDECTRL,           s_cdeCmdChk,                   SIZE(s_cdeCmdChk)
      mov   s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE      
 #endif
     mov   s_cdeCmdWd.byteCount,  GRE_SIZE_CHKSUM_BYTES

   // Load the GRE ethertype value into the LUT SPI field
l_c1ParseGre0:
   mov  s_l1View3b.SPI,   s_greHdr.proto
   xout XID_LUT1V3,       s_l1View3b.SPI,   SIZE(s_l1View3b.SPI)
   
   mov  s_l1View3b.IngressPort, PA_LUT1_GRE    
   xout XID_LUT1V3,       s_l1View3b.IngressPort, SIZE(s_l1View3b.IngressPort)

   // mov  s_cdeCmdWd.byteCount,  0
   qbbc  l_c1ParseGre1,  s_greHdr.flags.t_GreKBit
       add  s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount,  GRE_SIZE_KEY_BYTES

l_c1ParseGre1:
   qbbc  l_c1ParseGre2,  s_greHdr.flags.t_GreSBit
       add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,  GRE_SIZE_SEQNUM_BYTES

l_c1ParseGre2:
   //  scroll past the optional key and sequence number fields
   //  a scroll of 0 should be ok
   xout  XID_CDECTRL,  s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   add   s_pktCxt.startOffset,  s_pktCxt.startOffset,  s_cdeCmdWd.byteCount   

   mov   r0.w0,  ETH_TYPE_IP
   qbne  l_c1ParseGre3,  s_greHdr.proto, r0.w0 
       mov s_next.Hdr,  PA_HDR_IPv4
       ret

l_c1ParseGre3:
   mov   r0.w0,  ETH_TYPE_IPV6
   qbne  l_c1ParseGre4,  s_greHdr.proto,  r0.w0
       mov s_next.Hdr,  PA_HDR_IPv6
       ret

l_c1ParseGre4:
       mov s_next.Hdr,  PA_HDR_UNKNOWN
       ret

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
// *
// **************************************************************************************

    .using cdeScope
    .using lut1Scope
    .using pktScope
    .using secScope


f_c1ParseEsp:

   // Read in the ESP spi and add it to the LUT
   xin  XID_CDEDATA,  s_esp,     SIZE(s_esp)
   xout XID_LUT1V3,   s_esp.spi, SIZE(s_esp.spi)
   
   mov  s_l1View3b.IngressPort, PA_LUT1_SPI    
   xout XID_LUT1V3,        s_l1View3b.IngressPort, SIZE(s_l1View3b.IngressPort)


  // Check to see whether virtual link is used (Virtual Link check is redundant during SPI link)
//  qbbs l_c1ParseEsp_vLink, s_pktCxt.eP1C2IdIdx.t_vlinkEn
    // The previous lookup PDSP ID is stored in the ethertype field, previous LUT1 index in vlan
    lsr  s_l1View1a.EtherType,  s_pktCxt.eP1C2IdIdx,    SUBS_PKT_CXT_L1ID_BIT_OFFSET
    and  s_l1View1a.EtherType,  s_l1View1a.EtherType,   SUBS_PKT_CXT_L1ID_SHIFTED_MASK

    and  s_l1View1a.VLAN,       s_pktCxt2.IdIdx,        L1IDX_MASK

 //   jmp l_c1ParseEsp_issue_lookup

  // Populate lookup using virtual link
//l_c1ParseEsp_vLink: 
//    mov s_l1View1a.EtherType, 0x3
//    mov s_l1View1a.VLAN,    s_pktCxt.vLinkNum
    
    // Clear virtual link enable for further look ups
//    clr s_pktCxt.eP1C2IdIdx.t_vlinkEn

l_c1ParseEsp_issue_lookup:
  xout XID_LUT1V1,            s_l1View1a.EtherType,   SIZE(s_l1View1a.EtherType)+SIZE(s_l1View1a.VLAN)  
 

   mov  s_next.hdr,     PA_HDR_UNKNOWN
   mov  s_param.action, SUBS_ACTION_LOOKUP

   set  s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_ESP
   //set  s_runCxt.keyflags.t_pa_lut1_key_spi

   mov  s_pktCxt.espAhOffset,  s_pktCxt.startOffset
   // Adjust the start offset to end of ESP header
   // It is up to the Host or SA to further adjust it for the IV size
   add  s_pktCxt.startOffset,  s_pktCxt.startOffset, ESP_HEADER_LEN_BYTES
   ret

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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// **************************************************************************************/

    .using cdeScope
    .using pktScope

f_c1ParseEspDec:

   
   // Read in the padlen (r0.b1) and protocol (r0.b0) fields at the end of the packet
   // Note: It should be the absolute offset including Packet descriptor, PS Info.
   // sub    r0.w2,   s_pktCxt.endOffset,  s_pktCxt.startOffset
   // Note: Both the size of packet descriptor and PS Info are constants 
   //       We may need to enhance it for more general cases
   //add    r0.w2,   s_pktCxt.endOffset,  (32+24)  
   //sub    r0.w2,   r0.w2,               2
   add    r0.w2,   s_pktCxt.endOffset,  (32+32)-2
   // wait for the sideband data to be ready 
   wbc    s_flags.info.tStatus_CDEBusy
   lbco   r0.w0,   cCdeInPkt,           r0.w2,                  2

   // Note: the s_pktCtx.startOffset should already point to the next header 
   //add  s_pktCxt.startOffset,  s_pktCxt.startOffset, ESP_HEADER_LEN_BYTES
   sub  s_pktCxt.endOffset,    s_pktCxt.endOffset,   r0.b1
   sub  s_pktCxt.endOffset,    s_pktCxt.endOffset,   2

   //mov  s_cdeCmdWd.byteCount, ESP_HEADER_LEN_BYTES
   //xout XID_CDECTRL,          s_cdeCmdWd,             SIZE(s_cdeCmdWd)
   
   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  r0.b0,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   // Always contiune to parse the next header 
   //lsr    s_param.action,    r1.b0,                 6
   mov      s_param.action,    SUBS_ACTION_PARSE 

   ret

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
// *
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using lut1Scope
    .using secScope

f_c1ParseAuth:

   mov  s_pktCxt.espAhOffset,  s_pktCxt.startOffset
   set  s_pktCxt.hdrBitmask_nextHdr.SUBS_PA_BIT_HEADER_AUTH
   //set  s_runCxt.keyflags.t_pa_lut1_key_spi

   xin  XID_CDEDATA, s_ah,  SIZE(s_ah)
   
   // Add the SPI to the LUT
   mov  s_l1View3b.SPI,    s_ah.spi
   xout XID_LUT1V3,    s_l1View3b.SPI,  SIZE(s_l1View3b.SPI)
   
   mov  s_l1View3b.IngressPort, PA_LUT1_SPI    
   xout XID_LUT1V3,    s_l1View3b.IngressPort, SIZE(s_l1View3b.IngressPort)


  // Check to see whether virtual link is used (Virtual Link check is redundant during SPI link)
//  qbbs l_c1ParseAuth_vLink, s_pktCxt.eP1C2IdIdx.t_vlinkEn
    // The previous lookup PDSP ID is stored in the ethertype field, previous LUT1 index in vlan
    lsr  s_l1View1a.EtherType,  s_pktCxt.eP1C2IdIdx,    SUBS_PKT_CXT_L1ID_BIT_OFFSET
    and  s_l1View1a.EtherType,  s_l1View1a.EtherType,   SUBS_PKT_CXT_L1ID_SHIFTED_MASK

    and  s_l1View1a.VLAN,       s_pktCxt2.IdIdx,        L1IDX_MASK

//    jmp l_c1ParseAuth_issue_lookup

  // Populate lookup using virtual link
//l_c1ParseAuth_vLink: 
//    mov s_l1View1a.EtherType, 0x3
//    mov s_l1View1a.VLAN,    s_pktCxt.vLinkNum
    
    // Clear virtual link enable for further look ups
//    clr s_pktCxt.eP1C2IdIdx.t_vlinkEn

l_c1ParseAuth_issue_lookup:
  xout XID_LUT1V1,            s_l1View1a.EtherType,   SIZE(s_l1View1a.EtherType)+SIZE(s_l1View1a.VLAN) 


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
// *   R6:        |                                      
// *   R7:        |                                      
// *   R8:        |                                      
// *   R9:        |  LUT1 View1   - lut1Scope
// *   R10:       |
// *   R11:       |
// *   R12:       |
// *   R13:       |
// *   R14:          |  SCTP Header                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:           | not used                  
// *   R18:             | Checksum contrl block
// *   R19:             |
// *   R20:           |
// *   R21:           |
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
// **************************************************************************************

    .using cdeScope
    .using lut1Scope
    .using pktScope
    .using sctpScope

f_c1ParseSctp:

   // Read in the SCTP header 
   xin  XID_CDEDATA,  s_sctp,    SIZE(s_sctp)
   
   // Save and update SPI
   mov  s_l1View3a.SPI, s_sctp.dst
   xout XID_LUT1V3,     s_l1View3a.SPI, SIZE(s_l1View3a.SPI)
   
   mov  s_l1View3a.IngressPort, PA_LUT1_SCTP    
   xout XID_LUT1V3,     s_l1View3a.IngressPort, SIZE(s_l1View3a.IngressPort)

   mov  s_next.hdr,     PA_HDR_UNKNOWN
   mov  s_param.action, SUBS_ACTION_LOOKUP
   
   // Mark SCTP
   set  s_pktCxt.hdrBitmask3_frag_portNum.SUBS_PA_BIT_HEADER_SCTP
   //set  s_runCxt.keyflags.t_pa_lut1_key_sctp
   
   // Store, clear checksum and Issue CRC checksum command only if enabled
   qbbc  l_c1ParseSctp_end, s_runCxt.flags.t_sctpChksum
   
   mov  s_crcCxt.crc, s_sctp.chksum
   add  s_crcCxt.offset, s_pktCxt.startOffset, OFFSET(s_sctp.chksum)
   zero &s_sctp.chksum, SIZE(s_sctp.chksum)
   xout  XID_CDEDATA,  s_sctp.chksum,  SIZE(s_sctp.chksum)
   
   zero &s_cdeCmdCrcChk,           SIZE(s_cdeCmdCrcChk)
   sub   s_cdeCmdCrcChk.byteLen,   s_pktCxt.endOffset,         s_pktCxt.startOffset
   mov   s_cdeCmdCrcChk.offset,    OFFSET(s_sctp.chksum)
   set   s_cdeCmdCrcChk.options.t_cde_crc_cmd_option_crcOffset
   mov   s_cdeCmdCrcChk.operation, CDE_CMD_CRC_COMPUTE
   xout  XID_CDECTRL,           s_cdeCmdCrcChk,                SIZE(s_cdeCmdCrcChk)

   mov   s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
   
   // Set flag to indicate that CRC checksum verification is required
   // Append the CRC information to the end of pkt context in the PS Info area
   set  s_pktCxt.flag.t_flag_crc_verify
   
   sbco s_crcCxt,  cCdeOutPkt, SIZE(s_pktDescr) + SIZE(s_pktCxt), SIZE(s_crcCxt) 

l_c1ParseSctp_end:
   // Adjust the start offset to end of SCTP header
   mov  s_pktCxt.l4Offset,     s_pktCxt.startOffset
   add  s_pktCxt.startOffset,  s_pktCxt.startOffset, SCTP_HEADER_LEN_BYTES
   ret

    .leave cdeScope
    .leave lut1Scope
    .leave pktScope
    .leave sctpScope
    
#endif    

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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
// *   R29:  c1RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-param.action  w0-function return address            -
// *   R31:  System Flags (s_flags)                                 -
// *
// *
// ************************************************************************************

    .using pktScope

f_c1ParseUnkn:

   set  s_statsFlags.event.t_nParseFailC1

   or   s_pktCxt2.eP1C2Id,  s_pktCxt2.eP1C2Id,  EROUTE_PARSE_FAIL << PKT2_EIDX_SHIFT

c1pun1:
   mov s_param.action, SUBS_ACTION_EXIT
   ret

    .leave pktScope


#ifdef PASS_PROC_L3


// ************************************************************************************
// * FUNCTION PURPOSE: Parse a custom header
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
// *            - startOffset is not adjusted 
// *            - s_next.Hdr is set as custHdr.nextHdr
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
// *   R14:          |  CustomHdr & Custom                                      
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
// *
// ************************************************************************************

    .using cdeScope
    .using pktScope
    .using lut1Scope
    .using customC1Scope

f_c1ParseCustom:

  set  s_pktCxt.hdrBitmask_nexthdr.SUBS_PA_BIT_HEADER_CUSTOM
  mov  s_pktCxt.l3Offset,            s_pktCxt.startOffset
  set  s_statsFlags.event.t_nCustomPacketsC1

  qbbs l_c1ParseCustom_vlink, s_pktCxt.eP1C2IdIdx.t_vlinkEn
  // The previous lookup PDSP ID is stored in the ethertype field, previous LUT1 index in vlan
  lsr  s_l1View1a.EtherType,  s_pktCxt.eP1C2IdIdx,    SUBS_PKT_CXT_L1ID_BIT_OFFSET
  and  s_l1View1a.EtherType,  s_l1View1a.EtherType,   SUBS_PKT_CXT_L1ID_SHIFTED_MASK

  and  s_l1View1a.VLAN,       s_pktCxt2.IdIdx,        L1IDX_MASK

  jmp l_c1ParseCustom_issue_lookup

  // Populate lookup using virtual link
  l_c1ParseCustom_vlink: 
    mov s_l1View1a.EtherType, 0x3
    mov s_l1View1a.VLAN,  s_pktCxt.vLinkNum
    // Clear virtual link enable for further look ups
    clr s_pktCxt.eP1C2IdIdx.t_vlinkEn

  l_c1ParseCustom_issue_lookup:
  xout XID_LUT1V1,            s_l1View1a.EtherType,   SIZE(s_l1View1a.EtherType)+SIZE(s_l1View1a.VLAN)
  
  // eIndex should stote the custom Index
  // offset = index * 40 + OFFSET_CUSTOM_C1
  // TBD: Should we do error check here?
  and   r1.b0,  s_pktCxt2.eP1C2Id,  PKT2_EIDX_MASK
  lsr   r1.b0,  r1.b0,  PKT2_EIDX_SHIFT 
  mov   s_runCxt.keyFlags, r1.b0   // record the custom index as part of matching crierta
  lsl   r1.w2,  r1.b0,  5
  lsl   r1.w0,  r1.b0,  3
  add   r1.w0,  r1.w0,  r1.w2
  add   r1.w0,  r1.w0,  OFFSET_CUSTOM_C1 
  
  // find customer offset
  lbco  s_c1CustomHdr,  PAMEM_CONST_CUSTOM,  r1.w0,  SIZE(s_c1CustomHdr)

  mov  s_cdeCmdWd.byteCount,  s_c1CustomHdr.offset
  mov  s_next.Hdr,            s_c1CustomHdr.nextHdr
  xout XID_CDECTRL,           s_cdeCmdWd,         SIZE(s_cdeCmdWd)
  
  add   s_pktCxt.startOffset, s_pktCxt.startOffset, s_c1CustomHdr.nextHdrOffset  
  
  // read in the customer masks
  add   r1.w0,  r1.w0,  SIZE(s_c1CustomHdr) 
  lbco  s_c1Custom,  PAMEM_CONST_CUSTOM,  r1.w0,  SIZE(s_c1Custom)

  // Read in the 32 bytes of the custom info
  xin  XID_CDEDATA,   s_l1View2a,      SIZE(s_l1View2a)
  
  //  Relying on big endian data
  and   s_l1View2a.SrcIp_0,  s_l1View2a.SrcIp_0,  s_c1Custom.bitmask0
  and   s_l1View2a.SrcIp_1,  s_l1View2a.SrcIp_1,  s_c1Custom.bitmask1
  and   s_l1View2a.SrcIp_2,  s_l1View2a.SrcIp_2,  s_c1Custom.bitmask2
  and   s_l1View2a.SrcIp_3,  s_l1View2a.SrcIp_3,  s_c1Custom.bitmask3
  and   s_l1View2a.DstIp_0,  s_l1View2a.DstIp_0,  s_c1Custom.bitmask4
  and   s_l1View2a.DstIp_1,  s_l1View2a.DstIp_1,  s_c1Custom.bitmask5
  and   s_l1View2a.DstIp_2,  s_l1View2a.DstIp_2,  s_c1Custom.bitmask6
  and   s_l1View2a.DstIp_3,  s_l1View2a.DstIp_3,  s_c1Custom.bitmask7

  xout  XID_LUT1V2,  s_l1View2a,  SIZE(s_l1View2a)

  mov  s_param.action, SUBS_ACTION_LOOKUP

  set  s_runCxt.keyFlags.t_pa_lut1_key_custom
  
  ret

    .leave cdeScope
    .leave pktScope
    .leave lut1Scope
    .leave customC1Scope
    
#endif

#ifdef PASS_PROC_L2
    

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
// *   R3:               next header type  - pktScope
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
// *   R26:  w0 - Packet ID
// *   R27:
// *   R28:  statistics  (s_statsFlags)                             -
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

  set  s_statsFlags.event.t_nSrioPackets

  // Load the SRIO L0-L2 info: assuming type 11 
  xin  XID_CDEDATA,   s_srio11,  SIZE(s_srio11)
  
  // Delete the control information. It will be replace with the packet context during parse
  mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
  xout  XID_CDECTRL,           s_cdeCmdWd,       SIZE(s_cdeCmdWd)
  
  mov  s_param.action, SUBS_ACTION_LOOKUP

  mov  s_runCxt.keyFlags,  0
  
  set  s_runCxt.keyFlags.t_pa_lut1_srio_key_srio
    
  // Clear the LUT1 view areas (View1 and View 3 will be updated completely 
  zero &s_l1View2a,     SIZE(s_l1View2a)
  xout  XID_LUT1V2,     s_l1View2a,         SIZE(s_l1View2a)
  
  qbeq  l_c1ParseSrioType9, r1.b0, PA_PKT_TYPE_SRIO_TYPE_9
   

l_c1ParseSrioType11:
    // Type 11 Message Lookup
    //set  s_runCxt.keyFlags.t_pa_lut1_srio_key_type_11
    
    qbbc    l_c1ParseSrioType11_1,  s_srio11.ctrl.b1.t_srio_type11_t_port16
        // 16-bit device 
        set  s_runCxt.keyFlags.t_pa_lut1_srio_key_transport_16
        mov s_l1View1aSrio.srcId,   s_srio11.srcId
        mov s_l1View1aSrio.dstId,   s_srio11.dstId
        jmp l_c1ParseSrioType11_2
        
 l_c1ParseSrioType11_1:        
        // 8-bit device 
        set  s_runCxt.keyFlags.t_pa_lut1_srio_key_transport_8
        and s_l1View1aSrio.srcId,   s_srio11.srcId,     0xFF
        and s_l1View1aSrio.dstId,   s_srio11.dstId,     0xFF
        
 l_c1ParseSrioType11_2:    
  
   // Write the entire view to the lut
   xout XID_LUT1V1,   s_l1View1aSrio,    SIZE(s_l1View1aSrio)
   
   // LUT1 view2
   and s_l1View2aSrio.typeParam1, s_srio11.ctrl.b0,  SRIO_TYPE11_MBOX_MASK
   xout XID_LUT1V2,   s_l1View2aSrio.typeParam1,    SIZE(s_l1View2aSrio.typeParam1) 
   
   // LUT1 view3
   zero &s_l1View3aSrio,     SIZE(s_l1View3aSrio)
   and  r1.b0,              s_srio11.ctrl.b1,   SRIO_TYPE11_PRI_MASK
   lsr  s_l1View3aSrio.pri, r1.b0,              SRIO_TYPE11_PRI_SHIFT   
   mov  r1.w2,              SRIO_TYPE11_LTR_MASK
   and  r1.w0,              s_srio11.ctrl.w0,   r1.w2
   lsr  s_l1View3aSrio.typeParam2,   r1.w0,     SRIO_TYPE11_LTR_SHIFT
   set  s_l1View3aSrio.msgType.t_pa_lut1_srio_msg_type_11
   xout XID_LUT1V3,         s_l1View3aSrio,     SIZE(s_l1View3aSrio)
   
   ret
   
l_c1ParseSrioType9:
    // Type 9 Message Lookup
    //set  s_runCxt.keyFlags.t_pa_lut1_srio_key_type_9
    
    qbbc    l_c1ParseSrioType9_1,   s_srio9.ctrl.t_srio_type9_t_port16
        // 16-bit device 
        set  s_runCxt.keyFlags.t_pa_lut1_srio_key_transport_16
        mov s_l1View1aSrio.srcId,   s_srio9.srcId
        mov s_l1View1aSrio.dstId,   s_srio9.dstId
        jmp l_c1ParseSrioType9_2
        
 l_c1ParseSrioType9_1:        
        // 8-bit device 
        set  s_runCxt.keyFlags.t_pa_lut1_srio_key_transport_8
        and s_l1View1aSrio.srcId,   s_srio9.srcId,     0xFF
        and s_l1View1aSrio.dstId,   s_srio9.dstId,     0xFF
        
 l_c1ParseSrioType9_2:    
  
   // Write the entire view to the lut
   xout XID_LUT1V1,   s_l1View1aSrio,    SIZE(s_l1View1aSrio)
   
   // LUT1 view2
   mov  s_l1View2aSrio.typeParam1, s_srio9.streamId
   xout XID_LUT1V2,                s_l1View2aSrio.typeParam1,    SIZE(s_l1View2aSrio.typeParam1) 
   
   // LUT1 view3
   zero &s_l1View3aSrio,     SIZE(s_l1View3aSrio)
   and  r1.b0,              s_srio9.ctrl,       SRIO_TYPE9_PRI_MASK
   lsr  s_l1View3aSrio.pri, r1.b0,              SRIO_TYPE9_PRI_SHIFT   
   mov  s_l1View3aSrio.typeParam2,   s_srio9.cos
   set  s_l1View3aSrio.msgType.t_pa_lut1_srio_msg_type_9
   xout XID_LUT1V3,        s_l1View3aSrio,     SIZE(s_l1View3aSrio)
   
   ret
   
    .leave lut1Scope
    .leave pktScope
    .leave cdeScope
    .leave srioScope

#endif


