// ********************************************************************************
// * FILE PURPOSE: Tx Packet parsing functions
// ********************************************************************************
// * FILE NAME: parse_tx.p
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
// *************************************************************************************
// * FUNCTION PURPOSE: Parse an IPv4 header
// *************************************************************************************
// * DESCRIPTION: The IPv4 source and destination address are placed into the LTU.
// *              The pseudo header checksum is computed, and any options are parsed
// *
// *    On entry:
// *            - the CDE is at the start of the IPv4 (IPv6) header
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
  zero  &s_l1View3dIpv4,   SIZE(s_l1View3dIpv4)
   
  // Read the minimum IP header
  xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 
  
  // Basic Error Check
  // - version = 4
  // - ipHdrLen >= 20 
  // - (ipOffset + ipLength) <= pktLen
  // version check
  and   r0,               s_Ip.verLen,  0xF0
  qbeq  f_c1ParseIpv6,      r0,         0x60
  qbne  fci_c1ParseIpv4_10, r0,         0x40
  
  // Is ipHdrLength vaild?
  and  r0.b0,  s_Ip.verLen,   0x0F
  qbgt fci_c1ParseIpv4_10,    r0.b0,       IPV4_MIN_HDR_SIZE >> 2  // IP Hdr length check
  
  // Is ipLength valid?
  // TBD: The outer IP length is incorrect at our IP fragmentation test
#ifdef TO_BE_ADDED  
  add  r0.w0,  s_Ip.totalLen, s_txPktCxt.startOffset
  qblt fci_c1ParseIpv4_10,    r0.w0,       s_txPktCxt.endOffset      // IP length check
#endif  
  
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
    lmbd r0,    s_Ip.SrcIp,     0    
    qbeq fci_c1ParseIpv4_10,    r0,        32     
        
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
        //clr  s_runCxt.flags.t_l4Avil
         
l_c1ParseIpv4_000_1:
      // TBD: should we count forwarding fragments ???
      //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG 
    
      // pass through
l_c1ParseIpv4_0:
  // dstIp already in the correct location
  mov   s_l1View1bIpv4.srcIp,   s_Ip.srcIp
  xout  XID_LUT1V1,             s_l1View1bIpv4,                  OFFSET(s_l1View1bIpv4.res32a)

  // Skip checksum 
  zero &s_cdeCmdWd,             SIZE(s_cdeCmdWd)
  and   r2.w0,                  s_Ip.verLen,                    0x0f
  lsl   s_cdeCmdWd.byteCount,   r2.w0,                          2  // 32 bit size to 8 bit size conversion
  
  // Update the start and end offsets while the IP header length is in bytes
  add   s_txPktCxt.endOffset,     s_txPktCxt.startOffset,           s_Ip.totalLen
  add   s_txPktCxt.startOffset,   s_txPktCxt.startOffset,           s_cdeCmdWd.byteCount
  
  // skip pseudo header checksum and broadcast detection

  // Get the next action and header type from the IP protocol field
  lbco   r0.b0,             PAMEM_CONST_IP_PROTO,  s_Ip.protocol,   1
  and    s_next.Hdr,        r0.b0,                 0x3f
  lsr    s_param.action,    r0.b0,                 6
  
  //qbbs l_c1ParseIpv4_9_1, s_runCxt.flags.t_l4Avil
  //     mov s_param.action, SUBS_ACTION_LOOKUP 
    
l_c1ParseIpv4_9_1:
  
  //  Advance past the IP header.
  mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
  xout   XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
  
  // The protocol type and TOS must be added
  // to the LUT1 search

  set  s_l1View3dIpv4.pktType.fL3PktTypeFc
  set  s_l1View3dIpv4.pktFlags.fL3Ipv4
  mov  s_l1View3dIpv4.protocol,    s_Ip.protocol
  lsr  s_l1View3dIpv4.dscp, s_Ip.tos, 2
  xout XID_LUT1V3,               s_l1View3dIpv4,  SIZE(s_l1View3dIpv4)
  ret

fci_c1ParseIpv4_10:
   // IPv4/v6 Hdr Error
   mov   s_txPktCxt2.eId,     EROUTE_IP_FAIL   
   
   // Common Error
l_c1ParseIpv4_11:
   mov  s_param.action,     SUBS_ACTION_EXIT
   ret
       
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
    
f_c1ParseIpv6:
   
l_c1ParseIpv6_0:
   zero  &s_l1View3bIpv6,    SIZE(s_l1View3bIpv6)
   
   // The previous lookup PDSP ID and LUT1 index is stored in the srcVC field
   // mov  s_l1View3bIpv6.srcVC,  s_pktCxt.phyLink

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
   
   // skip pesudo checksum and multi-cast detection 
   
l_c1ParseIpv6_1:
   
   // The protocol, tclass, and flow labels are moved as 
   // required
   mov   s_l1View3bIpv6.flowLabel, s_Ipv6a.ver_tclass_flow
   and   s_l1View3bIpv6.flowLabel.w2,  s_l1View3bIpv6.flowLabel.w2,   0x0f

   set   s_l1View3bIpv6.pktType.fL3PktTypeFc
   mov   s_l1View3bIpv6.protocol,      s_Ipv6a.next
   lsr   s_l1View3bIpv6.dscp,   s_Ipv6a.ver_tclass_flow.w2,  4
   and   s_l1View3bIpv6.dscp,   s_l1View3bIpv6.dscp, 0x3F
   xout  XID_LUT1V3,               s_l1View3bIpv6,         SIZE(s_l1View3bIpv6)

   // Update packet offsets
   add   s_txPktCxt.startOffset,   s_txPktCxt.startOffset,  IPV6_HEADER_LEN_BYTES
   add   s_txPktCxt.endOffset,     s_txPktCxt.startOffset,  s_Ipv6a.payloadLen

   // Get the next header and action
   set    s_l1View3bIpv6.pktFlags.fL3IpContainL4
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

   // The length of the options field in the header is in 8 byte units, not
   // including the 1st 8 bytes.
   xin  XID_CDEDATA,  s_Ipv6Opt,         SIZE(s_Ipv6Opt)
   lsl  r0.w0,        s_Ipv6Opt.optlen,  3
   add  r0.w0,        r0.w0,             8
   add  s_txPktCxt.startOffset, s_txPktCxt.startOffset, r0.w0

#ifdef TO_BE_DELETE
   // Ignore HOP_BY_OPT option for now
   // Scroll past the options header
   mov  s_cdeCmdWd.byteCount,  SIZE(s_Ipv6Opt)
   xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)

   mov  r0.w2,        SIZE(s_Ipv6Opt)

l_c1ParseIpv6ExtHop0:

   qble  l_c1ParseIpv6ExtHop2,   r0.w2,  r0.w0

     // read in the sub-option byte
     xin  XID_CDEDATA,           r14.b3,      1
     mov  s_cdeCmdWd.byteCount,  1
     xout XID_CDECTRL,           s_cdeCmdWd,  SIZE(s_cdeCmdWd)
     add  r0.w2,                 r0.w2,       1

     // PAD0 just advances one byte
     qbeq  l_c1ParseIpv6ExtHop0,  r14.b3,      IPV6_OPT_HOP_BY_HOP_OPT_PAD0

l_c1ParseIpv6ExtHop1:
      // PADN and unsupported options
      xin  XID_CDEDATA,            r14.b3,     1   // the length (not counting opt and len)
      add  s_cdeCmdWd.byteCount,   r14.b3,     1
      xout XID_CDECTRL,            s_cdeCmdWd,   SIZE(s_cdeCmdWd)
      add  r0.w2,                  r0.w2,        s_cdeCmdWd.byteCount
      jmp  l_c1ParseIpv6ExtHop0
      
#else
   mov  s_cdeCmdWd.byteCount,  r0.w0
   xout XID_CDECTRL,           s_cdeCmdWd,        SIZE(s_cdeCmdWd)
#endif      

l_c1ParseIpv6ExtHop2:
   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)

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
   add  s_cdeCmdWd.byteCount,  r0.w0,                 8
   xout XID_CDECTRL,           s_cdeCmdWd,            SIZE(s_cdeCmdWd)
   add  s_txPktCxt.startOffset,  s_txPktCxt.startoffset,  s_cdeCmdWd.byteCount

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

   // The fragmentation header is always 8 bytes. The whole 
   // header is read here, but only the 1st byte is used
   xin  XID_CDEDATA,           s_Ipv6Frag,      SIZE(s_Ipv6Frag)      
   mov  s_cdeCmdWd.byteCount,  SIZE(s_Ipv6Frag)
   xout XID_CDECTRL,           s_cdeCmdWd,      SIZE(s_cdeCmdWd)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Frag.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)
   
l_c1ParseIpv6ExtFrag0:
   add  s_txPktCxt.startOffset,           s_txPktCxt.startOffset,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
   set  s_l1View3bIpv6.pktFlags.fL3IpFrag
  // Fragmentation check
  lsr   r3.w0,   s_Ipv6Frag.fragnFlag,     IPV6_FRAG_OFF_SHIFT
  qbeq  l_c1ParseIpv6ExtFrag1, r3.w0, 0
        clr  s_l1View3bIpv6.pktFlags.fL3IpContainL4
        clr  s_runCxt.flags.t_l4Avil
l_c1ParseIpv6ExtFrag1:  
   xout XID_LUT1V3,           s_l1View3bIpv6.pktFlags,  SIZE(s_l1View3bIpv6.pktFlags)
   
   // TBD: TX fragment statistics
   //set  s_pktCxt.flags.t_flag_frag
   //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_IP_FRAG

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Frag.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6
   
   // TBD:
   qbbs l_c1ParseIpv6ExtFrag3, s_runCxt.flags.t_l4Avil
        mov s_param.action, SUBS_ACTION_LOOKUP 
    
l_c1ParseIpv6ExtFrag3:
   
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

   xin  XID_CDEDATA,  s_Ipv6Opt,  SIZE(s_Ipv6Opt)

   // Put the next proto type into the lookup
   mov  s_l1View3bIpv6.protocol,  s_Ipv6Opt.proto
   xout XID_LUT1V3,           s_l1View3bIpv6.protocol,  SIZE(s_l1View3bIpv6.protocol)

   // Get the next header type and action from the protocol field
   lbco   r1.b0,             PAMEM_CONST_IP_PROTO,  s_Ipv6Opt.proto,   1
   and    s_next.Hdr,        r1.b0,                 0x3f
   lsr    s_param.action,    r1.b0,                 6

   lsl    r0.w0,                s_Ipv6Opt.optlen,     3
   add    s_cdeCmdWd.byteCount, r0.w0,                8
   xout   XID_CDECTRL,          s_cdeCmdWd,           SIZE(s_cdeCmdWd)
   add    s_txPktCxt.startOffset, s_txPktCxt.startOffset, s_cdeCmdWd.byteCount

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

  mov  s_txPktCxt.l4Offset,   s_txPktCxt.startOffset   // TBD: Why?

  xin  XID_CDEDATA,           s_greHdr,      SIZE(s_greHdr)
  mov  s_cdeCmdWd.byteCount,  SIZE(s_greHdr)
  xout XID_CDECTRL,           s_cdeCmdWd,    SIZE(s_cdeCmdWd)
  
  add  s_txPktCxt.startOffset,  s_txPktCxt.startOffset,  SIZE(s_greHdr)
  mov  s_cdeCmdWd.byteCount,  0

  // Skip GRE checksum calculation
  
l_c1ParseGre0:
   mov  s_l1View3bIpv6.protocol,   s_greHdr.proto
   xout XID_LUT1V3,       s_l1View3bIpv6.protocol,   SIZE(s_l1View3bIpv6.protocol)
   
   mov  s_cdeCmdWd.byteCount,  0
   qbbc  l_c1ParseGre1,  s_greHdr.flags.t_GreKBit
       add  s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount,  GRE_SIZE_KEY_BYTES

l_c1ParseGre1:
   qbbc  l_c1ParseGre2,  s_greHdr.flags.t_GreSBit
       add   s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,   GRE_SIZE_SEQNUM_BYTES

l_c1ParseGre2:
   //  scroll past the optional key and sequence number fields
   //  a scroll of 0 should be ok
   xout  XID_CDECTRL,  s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   add   s_txPktCxt.startOffset,  s_txPktCxt.startOffset,  s_cdeCmdWd.byteCount

   mov   r0.w0,  ETH_TYPE_IP
   qbne  l_c1ParseGre3, s_greHdr.proto, r0.w0 
       mov s_next.Hdr,  PA_HDR_IPv4
       ret

l_c1ParseGre3:
   mov   r0.w0,  ETH_TYPE_IPV6
   qbne  l_c1ParseGre4, s_greHdr.proto,  r0.w0
       mov s_next.Hdr,  PA_HDR_IPv6
       ret

l_c1ParseGre4:
       mov s_next.Hdr,  PA_HDR_UNKNOWN
       ret

    .leave cdeScope
    .leave pktScope
    .leave greScope
    .leave lut1Scope

   
    
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

    mov s_txPktCxt.l4Offset,  s_txPktCxt.startOffset

    // Read in the UDP header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udp,  SIZE(s_udp)
    
    // Skip checksum 

    // UDP length includes the udp header, to the end length is computed 
    // before updating the start offset.
    // TBD: We do not want to adjust length at firewall stage
    // Restore the offset information per after firewall operation
    add  s_txPktCxt.endOffset,    s_txPktCxt.startOffset,  s_udp.len
    add  s_txPktCxt.startOffset,  s_txPktCxt.startOffset,  UDP_HEADER_LEN_BYTES
    //mov  s_txPktCxt.l5Offset,     s_txPktCxt.startOffset
    //mov  s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_UDP_PKTS
    
    
    // Do the lookup
    mov  s_l1View3dIpv4.dstPort,    s_udp.dst    
    mov  s_l1View3dIpv4.srcPort,    s_udp.src   

    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)

    mov s_param.action, SUBS_ACTION_LOOKUP

    ret
    
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

    // Record parsing UDP
    mov s_txPktCxt.l4Offset,      s_txPktCxt.startOffset
    
    // Read in the UDP lite header. Do not advance since there could be a switch to
    // custom mode
    xin  XID_CDEDATA, s_udpLite,  SIZE(s_udpLite)

    // skip checksum calculation

    // UDP-lite length includes the udp header
    // TBD: adjust the length 
    add  s_txPktCxt.startOffset,  s_txPktCxt.startOffset,  UDP_LITE_HEADER_LEN_BYTES
    //mov  s_txPktCxt.l5Offset,     s_txPktCxt.startOffset
    //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_UDP_PKTS
    
l_c1ParseUdpLite1:

    // Do the lookup
    mov  s_l1View3dIpv4.dstPort,    s_udpLite.dst    
    mov  s_l1View3dIpv4.srcPort,    s_udpLite.src   

    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)
    
    mov s_param.action, SUBS_ACTION_LOOKUP
    ret

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

    mov s_txPktCxt.l4Offset,      s_txPktCxt.startOffset

    // Read in the TCP header
    xin  XID_CDEDATA,  s_tcp,  SIZE(s_tcp)
    
    // Firewall-specific matching of type

    // Configure and initiate the lookup
    mov  s_l1View3dIpv4.dstPort,    s_tcp.dst    
    mov  s_l1View3dIpv4.srcPort,    s_tcp.src   
    xout XID_LUT1V3,  s_l1View3dIpv4.srcPort,  SIZE(s_l1View3dIpv4.srcPort) + SIZE(s_l1View3dIpv4.dstPort)
    
    // skip chesum calculation

    // Adjust the start offset past the tcp header
    lsr  r1.w0,                s_tcp.offset_ecn,       4     // extract the length (32 bit length)
    lsl  r1.w0,                r1.w0,                  2      // convert to byte length
    add  s_txPktCxt.startOffset, s_txPktCxt.startOffset,   r1.w0
    //mov  s_txPktCxt.l5Offset,    s_txPktCxt.startOffset
    //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_TCP_PKTS

    mov  s_param.action,  SUBS_ACTION_LOOKUP
    ret
    
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

    // Terminate parsing
    mov s_param.action,        SUBS_ACTION_LOOKUP
    ret

#ifdef TO_BE_TBD    

   // TBD: system statistics
   // mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_PARSE_FAIL
   mov s_txPktCxt2.eId,   EROUTE_PARSE_FAIL   
   mov s_param.action, SUBS_ACTION_EXIT
   ret
   
#endif   

    .leave pktScope
    
    




