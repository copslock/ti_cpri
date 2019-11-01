// ***********************************************************************************************************
// * FILE PURPOSE: Perform PDSP memory intialization
// ***********************************************************************************************************
// * FILE NAME: meminit.p
// *
// * DESCRIPTION: Every PDSP has this function, but on startup the host will choose only a single
// *			  PDSP to initialize the common memory for all PDSPs.
// *
// ***********************************************************************************************************/
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

//  Register usage
// 
//   R0 - R29 General purpose
//   
//   R30  Return addres
//   R31  Flags

	.using initScope
	.using checkScope

f_commonInit:

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
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_TCP
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
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_UDP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP,              1 


  // Protocol type 136: UDP lite
  mov   r4.b0,   (SUBS_ACTION_LOOKUP << 6) | PA_HDR_UDP_LITE
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_UDP_LITE,         1

  // Protocol type 132: SCTP
  mov   r4.b0,   (SUBS_ACTION_PARSE << 6) | PA_HDR_SCTP
  sbco  r4.b0,   PAMEM_CONST_IP_PROTO,  IP_PROTO_NEXT_SCTP,             1


  // Custom Classify 1  (40*4)
  zero  &r0,  80
  sbco  &r0,  PAMEM_CONST_CUSTOM, OFFSET_CUSTOM_C1, 80
  sbco  &r0,  PAMEM_CONST_CUSTOM, OFFSET_CUSTOM_C1 + 2*40, 80

  // Custom Classify 2  (16*16)
  //zero  &r0, 64
  sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2, 4*16
  sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64, 4*16
  sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64*2, 4*16
  sbco  &r0, PAMEM_CONST_CUSTOM2, OFFSET_CUSTOM_C2+64*3, 4*16

  // Packet Capture configuration
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_EGRESS_PKT_CAP_CFG_BASE,  5*8
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_INGRESS_PKT_CAP_CFG_BASE, 5*8
  
  // Clear system timestamp
  sbco  &r0, PAMEM_CONST_PARSE,  OFFSET_SYS_TIMESTAMP,   8 

  // Packet ingress default route configuration (64 *4)
  mov   r20, OFFSET_DEFAULT_ROUTE_CFG_BASE
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  
  add   r20, r20, 64
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  
  add   r20, r20, 64
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  
  add   r20, r20, 64
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64

  // EQOS configuration table (1024 bytes)
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_EQOS_CFG_EG_DEF_PRI,    4
  mov   r20, OFFSET_EQOS_CFG_BASE
  mov   r21,  256*4
  add   r21,  r21, r20
eqosInit_0:  
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  add   r20, r20, 64
  qbne  eqosInit_0, r20,          r21
  
  // Configurable exception routing
  // All exceptions are initially configured for silent discard
  mov    r5,          0
  mov    r1,          EROUTE_N_MAX * 16
  zero  &s_paF,  SIZE(s_paF)+SIZE(s_paFh)

  mov    s_paF.forwardType,  PA_FORWARD_TYPE_DISCARD

commonInit_0:
  sbco  &s_paF, PAMEM_CONST_EROUTE, r5, SIZE(s_paF)+SIZE(s_paFh)
  add   r5,     r5,                 16
  qbne  commonInit_0, r5,           r1

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
  
  // Multiple Routing tables
  // Initialize all the routing tables to discard.
  // There are 32 multiple routes, each of which has 8 single routes, each single route is 8 bytes
  zero  &s_pa1,       8*SIZE(s_pa1)
  mov    r5.w0,       0
  mov    r5.w2,       32*8*SIZE(s_pa1)

commonInit_1:
  sbco  &s_pa1,  PAMEM_CONST_MULTI_ROUTE,  r5.w0,  8*SIZE(s_pa1)
  add  r5.w0,  r5.w0,  8*SIZE(s_pa1)
  qbne commonInit_1,   r5.w0, r5.w2


  // PDSP version info
  mov  r4.b0,  0
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP0, 1

  mov  r4.b0,  1
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP1, 1
  
  mov  r4.b0,  2
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP2, 1

  mov  r4.b0,  3
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP3, 1

  mov  r4.b0,  4
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP4, 1

  mov  r4.b0,  5
  sbco  &r4.b0,  PAMEM_CONST_PDSP_ALL_INFO, OFFSET_ID_PDSP5, 1

  // Initialize global configuration parameters
  zero &s_paMaxHdrCountInit, 36
  
  // Initialize the total block to zero (replace individual initialization 
  sbco s_paMaxHdrCountInit,               PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR, 36
  
  // Initialize the max counts
  mov  s_paMaxHdrCountInit.vlanMaxCount,  2
  mov  s_paMaxHdrCountInit.ipMaxCount,    2
  mov  s_paMaxHdrCountInit.greMaxCount,   2
  sbco s_paMaxHdrCountInit,               PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR, SIZE(s_paMaxHdrCountInit)
  
  // Initialize the outer IP Reassembly configuration
  //mov  s_paOutIpReassmInit.numTrafficFlow, 0
  //sbco s_paOutIpReassmInit,                PAMEM_CONST_CUSTOM, OFFSET_OUT_IP_REASSM_CFG, SIZE(s_paOutIpReassmInit)
  
  // Initialize the inner IP Reassembly configuration
  //mov  s_paInIpReassmInit.numTrafficFlow, 0
  //sbco s_paInIpReassmInit,                PAMEM_CONST_CUSTOM, OFFSET_IN_IP_REASSM_CFG, SIZE(s_paInIpReassmInit)

  // Initialize the Command Set configuration
  mov  s_paCmdSetInit.numCmdSets,         64
  mov  s_paCmdSetInit.cmdSetSize,         64
  sbco s_paCmdSetInit,                PAMEM_CONST_CUSTOM, OFFSET_CMDSET_CFG, SIZE(s_paCmdSetInit)
  
  // Initialize the User-defined Statistics configuration
  //mov  s_paUsrStatsInit.numCounters,      0
  //mov  s_paUsrStatsInit.num64bCounters,   0
  //sbco s_paUsrStatsInit,                  PAMEM_CONST_CUSTOM, OFFSET_USR_STATS_CFG, SIZE(s_paUsrStatsInit)
  
  // Initialize the Queue Diversion configuration
  //sbco s_paQueueDivertInit,               PAMEM_CONST_CUSTOM, OFFSET_QUEUE_DIVERT_CFG, SIZE(s_paQueueDivertInit)
  
  // Initialize the IPSEC NAT-T configuration
  //sbco s_paIpsecNatTDetInit,              PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG,  SIZE(s_paIpsecNatTDetInit)
  
  // Initialize the Queue Bounce configuration

  // Initialize the User-defined statistics related memory blocks
  // Initialize the User Statistics Link buffer  (512 * 2 bytes)
  mov    r20,    OFFSET_USR_STATS_CB
  mov    r21,    OFFSET_USR_STATS_CB + PA_USR_STATS_NUM_ENTRIES * 2
  
  // Set all link Index to no link and disable
  mov    r0.w0,       0xa000
  mov    r0.w2,       0xa000

commonInit_2:
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 4
  add   r20,     r20,     4                
  qbne  commonInit_2, r20,           r21
  
  // Clear all User Statistics (512 * 4 bytes)
  mov    r20,    OFFSET_USR_STATS_COUNTERS
  mov    r21,    OFFSET_USR_STATS_COUNTERS + PA_USR_STATS_NUM_ENTRIES * 4
  
  // zero out 16 registers 
  zero   &r0,   16*4

commonInit_3:
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16*4
  add   r20,    r20,     16*4                
  qbne  commonInit_3, r20,           r21
  
  // Clear all 32-bit User Statistics (192 * 4 bytes)
  //mov    r20,    OFFSET_USR_STATS_32B_COUNTERS
  //mov    r21,    OFFSET_USR_STATS_32B_COUNTERS + 192 * 4
  
  // zero out 16 registers 
  //zero   &r0,   16*4

commonInit_4:
  //sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16*4
  //add   r20,    r20,     16*4                
  //qbne  commonInit_4, r20,           r21
  
  // Initialize the User Statistics Request FIFOs
  mov   r20,    OFFSET_PDSP0_USR_STATS_FIFO_CB
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16
  mov   r20,    OFFSET_PDSP1_USR_STATS_FIFO_CB
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16
  mov   r20,    OFFSET_PDSP2_USR_STATS_FIFO_CB
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16
  mov   r20,    OFFSET_PDSP3_USR_STATS_FIFO_CB
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16
  mov   r20,    OFFSET_PDSP5_USR_STATS_FIFO_CB
  sbco  r0,     PAMEM_USR_STATS_BASE, r20, 16

  ret

	.leave checkScope
	.leave initScope
