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

#ifndef _PDSP_PA_H
#define _PDSP_PA_H   1

// ***********************************************************************************************
// * FILE PURPOSE: Define the API to the PA subsystem from the host and the SA
// ***********************************************************************************************
// * FILE NAME: pdsp_pa.h
// *
// * DESCRIPTION: PA Subsystem interface description
// *              This file is the PDSP version equivalent to pafrm.h at pa/src
// *              These two files must be synchronous to each other.   
// *
// ***********************************************************************************************

// ***********************************************************************************************
// * Exit Packet Routing
// *
// *  The following structures are intended to form a C like union using the .assign
// *  operator. 
// *  Example:
//      .assign  struct_paForward,            R1,  R1, paForward
//      .assign  struct_paForwardHost,        R2,  R4, paForward_host
//      .assign  struct_paForwardSa,          R2,  R4, paForward_sa
//      .assign  struct_paForwardSrio,        R2,  R4, paForward_srio
//      .assign  struct_paForwardEth,         R2,  R4, paForward_eth
//      .assign  struct_paForwardPa,          R2,  R4, paForward_pa

// Packet routing info for packets sent to the host
.struct struct_paForwardHost
    .u32  context
    .u8   ctrlBitmap
    .u8   multiIdx
    .u8   paPdspRouter
    .u8   psFlags
    .u32  cmd
.ends

#define t_pa_multiroute               t0
#define t_pa_routing_priority_dscp    t1
#define t_pa_routing_priority_vlan    t2
#define t_pa_routing_if_dest_flow     t1
#define t_pa_routing_if_eqos          t6
#define t_pa_routing_if_selection     t7

// Packet routing info for packets sent to SA
.struct struct_paForwardSa
    .u32  swInfo0
    .u32  swInfo1
    .u32  cmd
.ends

// Packet routing info for packets sent to SRIO
.struct struct_paForwardSrio
    .u32  psInfo0
    .u32  psInfo1
    .u8   pktType
    .u8   cmd0
    .u16  cmd12
.ends

// Packet routing info for packets sent to ETH
.struct struct_paForwardEth
    .u8   psFlags
    .u8   rsvd1
    .u16  rsvd2
    .u32  rsvd3
    .u32  cmd
.ends

#define PA_ETH_PS_FLAGS_DISABLE_CRC          0x80
#define PA_ETH_PS_FLAGS_PORT_MASK            0x70
#define PA_ETH_PS_FLAGS_PORT_SHIFT              4
#define t_pa_eth_ps_flags_disable_crc        t7 

// Packet routing info for packets sent to PA
.struct struct_paForwardPa
    .u8   dest
    .u8   custType
    .u8   custIdx
    .u8   ctrlBitmap
    .u32  rsvd2
    .u32  cmd
.ends

#define t_pa_cascaded_forwarding             t0


// Packet routing info for packets dropped
.struct struct_paDiscard
    .u32  rsvd1
    .u32  rsvd2
    .u32  cmd
.ends

#define PA_CUSTOM_TYPE_NONE     0    
#define PA_CUSTOM_TYPE_LUT1     1
#define PA_CUSTOM_TYPE_LUT2     2

// Destination (route) values 
#define PA_DEST_PDSP0       0
#define PA_DEST_PDSP1       1
#define PA_DEST_PDSP2       2
#define PA_DEST_PDSP3       3
#define PA_DEST_PDSP4       4
#define PA_DEST_PDSP5       5
#define PA_DEST_CDMA        6   
#define PA_DEST_ETH         7

#define PA_DEST_SRIO        0    // virtual destination used by next route command only

// Thread ID no longer used 
// #define PA_DEST_ACE0        8
// #define PA_DEST_ACE1        9

#define PA_DEST_DISCARD     10

// Assigning names based on PDSP functions 
#define PA_DEST_PA_C1_0         PA_DEST_PDSP0
#define PA_DEST_PA_C1_1         PA_DEST_PDSP1
#define PA_DEST_PA_C1_2         PA_DEST_PDSP2 
#define PA_DEST_PA_C2           PA_DEST_PDSP3
#define PA_DEST_PA_M_0          PA_DEST_PDSP4
#define PA_DEST_PA_M_1          PA_DEST_PDSP5


// Packet Routing info
.struct struct_paForward
    .u8   forwardType
    .u8   flowId
    .u16  queue
.ends

#define PA_FORWARD_TYPE_HOST     0
#define PA_FORWARD_TYPE_SA       1
#define PA_FORWARD_TYPE_PA       2
#define PA_FORWARD_TYPE_ETH      3
#define PA_FORWARD_TYPE_SRIO     4
#define PA_FORWARD_TYPE_DISCARD  5

#define PA_FOWARD_QUEUE_MASK     0x3FFF
#define t_pa_forward_queue_bounce_ddr       t14
#define t_pa_forward_queue_bounce_msmc      t15

// Placeholder structure for PA forward
.struct struct_paFwdPlace
    .u32   p0
    .u32   p1
    .u32   p2
    .u32   p3
.ends



// **********************************************************************************************
// * PA configuration commands
// * The following structures are intended to make a C like union using the .assign
// * operator.
// *

// Both standard and custom add have this header
.struct struct_paComAddL1Hdr
    .u8   index
    .u8   type
    .u8   vLinkNum
    .u8   custIndex       // Custom Only: 
.ends

// Add/Modify a standard (MAC/IP) entry in a LUT1 table
.struct struct_paComAddLut1StdA
    .u32  dmac5_2       
    
    .u32  dmac1_0_smac5_4
    .u32  dmac3_0       
    
    .u16  etype
    .u16  vlan
.ends

.struct struct_paComAddLut1StdB
    .u32  srcIp3        
    .u32  srcIp2
    .u32  srcIp1
    .u32  srcIp0        
    
    .u32  dstIp3        
    .u32  dstIp2
    .u32  dstIp1
    .u32  dstIp0        

.ends

.struct struct_paComAddLut1StdC
    .u32  spi_gre_etype
    
    .u32  flow
    
    .u32  ports_mpls
    
    .u8   protoNext
    .u8   tosTclass
    .u8   inport
    .u8   key
    
.ends

.struct struct_paComAddLut1StdD
    
    .u16  matchFlags
    .u16  rsvd
.ends

// Key values. The PDSP will set these bits as it parses the headers.
// The host must supply matching values if the key field is marked as care 
// key used by L3 (LUT1_1 and LUT1_2)
#define t_pa_lut1_key_ipv4   t3
#define t_pa_lut1_key_ipv6   t4
#define t_pa_lut1_key_custom t7 

// SPI, GRE, SCTP flags (inport)
#define t_pa_lut1_spi        t0
#define t_pa_lut1_gre        t1
#define t_pa_lut1_sctp       t2 

#define PA_LUT1_SPI          1
#define PA_LUT1_GRE          2
#define PA_LUT1_SCTP         4


#define PA_LUT1_KEY_BIT_CUSTOM   7

// key used by L2 MAC and SRIO (L0-l2) (LUT1_0)
#define t_pa_lut1_key_srio   t7
#define t_pa_lut1_key_mac    t0

// Add/Modify a SRIO entry in a LUT1 table
.struct struct_paComAddLut1SrioA
    .u32  rsvd1       
    .u16  srcId
    
    .u16  destId
    .u32  rsvd2       
    
    .u16  etype
    .u16  vlan
.ends

.struct struct_paComAddLut1SrioB
    .u32  rsvd1        
    .u32  rsvd2
    .u32  rsvd3
    .u32  rsvd4        
    
    .u32  rsvd5        
    .u32  rsvd6
    .u32  rsvd7
    .u16  rsvd8
    .u16  typeParam1     // stream ID or mailbox */        
.ends

.struct struct_paComAddLut1SrioC
    .u32  spi_gre_etype
    
    .u32  flow
    
    .u16  nextHdrOffset  // They are nott used for matching : Source port (0), destination port (1) 
    .u8   nextHdr        // place holder for nextHdr and nextOffset         
    .u8   rsvd5 
    
    .u8   pri
    .u8   typeParam2     // cos or letter 
    .u8   msgType
    .u8   key
    
.ends

.struct struct_paComAddLut1SrioD
    
    .u16  matchFlags
    .u16  rsvd
.ends

// Key values. The PDSP will set these bits as it parses the headers.
// The host must supply matching values if the key field is marked as care 
#define t_pa_lut1_srio_key_transport_8  t2
#define t_pa_lut1_srio_key_transport_16 t3
#define t_pa_lut1_srio_key_srio         t7

// msgType at message type field
#define t_pa_lut1_srio_msg_type_9       t0
#define t_pa_lut1_srio_msg_type_11      t1


// SRIO match flag bits 
#define PA_LUT1_SRIO_MATCH_SRCID         (1 << 0)
#define PA_LUT1_SRIO_MATCH_DESTID        (1 << 1)
#define PA_LUT1_SRIO_MATCH_LINK          (3 << 2)
#define PA_LUT1_SRIO_MATCH_TYPEPARAM1    (1 << 5)
#define PA_LUT1_SRIO_MATCH_PRI           (1 << 10)
#define PA_LUT1_SRIO_MATCH_TYPEPARAM2    (1 << 11)
#define PA_LUT1_SRIO_MATCH_MSGTYPE       (1 << 12)
#define PA_LUT1_SRIO_MATCH_KEY           (1 << 13)
#define PA_LUT1_SRIO_MATCH_VALID         (1 << 15)

// Add/Modify a Custom entry in a LUT1 table
.struct struct_paComL1CustomA
    .u32  dmac5_2       
    
    .u32  dmac1_0_smac5_4
    .u32  dmac3_0       
    
    .u16  etype
    .u16  vlan
.ends

.struct struct_paComL1CustomB
    .u32   match0
    .u32   match1
    .u32   match2
    .u32   match3
    
    .u32   match4
    .u32   match5
    .u32   match6
    .u32   match7
.ends
    
.struct struct_paComL1CustomC    
    .u32  rsvd0
    .u32  rsvd1
    .u32  rsvd2
    .u16  rsvd3
    .u8   rsvd4
    .u8   key
.ends

.struct struct_paComL1CustomD
    .u16  matchFlags
    .u16  rsvd
.ends

// Standard SRIO and custom are followed by two paForwardMatch values for 
// match and nextFail

// Custom match flag bits
#define PA_LUT1_CUSTOM_MATCH_DMAC   (1 << 0)
#define PA_LUT1_CUSTOM_MATCH_SMAC   (1 << 1)
#define PA_LUT1_CUSTOM_MATCH_ETYPE  (1 << 2)
#define PA_LUT1_CUSTOM_MATCH_VLAN   (1 << 3)
#define PA_LUT1_CUSTOM_MATCH_MATCH  (3 << 4)  
#define PA_LUT1_CUSTOM_MATCH_KEY    (1 << 13)  
#define PA_LUT1_CUSTOM_MATCH_VALID  (1 << 15)  


#define PA_LUT1_INDEX_LAST_FREE       64
#define PA_COM_ADD_LUT1_STANDARD      0
#define PA_COM_ADD_LUT1_SRIO          1
#define PA_COM_ADD_LUT1_CUSTOM        2
#define PA_COM_ADD_LUT1_VLINK         3

// Delete LUT1 entry
.struct struct_paCommandDelLut1
    .u8   index
    .u8   rsvd1
    .u16  rsvd2
.ends

// Add/replace a standard entry in LUT2
// This is followed by match routing info
.struct  struct_paComAddLut2Standard
    .u8   type
    .u8   index         // custom Index
    .u8   ctrlBitMap   
    .u8   rsvd
    
    .u16  match1        // MSW  
    .u16  match0        // LSW
.ends

// Add/replace custom entry in LUT2
// This is followed by match routing info
.struct struct_paComAddLut2Custom
    .u8   type
    .u8   index         // custom Index
    .u8   ctrlBitMap   
    .u8   rsvd
    
    .u32  match
.ends

// control flags
#define t_pa_lut2_ctrl_replace      t0
#define t_pa_lut2_ctrl_gtpu         t1   // Disable GTPU if GTPU port (2152) is added into LUT2 table */
                                         // Re-enable GTPU if GTPU port (2152) is deleted from LUT2 table */
#define t_pa_lut2_ctrl_queue_divert t2   // Enable Queue divert, Note: It is for replaced entry only                                          

#define PA_COM_ADD_LUT2_STANDARD    0
#define PA_COM_ADD_LUT2_CUSTOM      1

// Queue divert information is used by QMSS only
// Define here for reference purpose only
.struct struct_paComLut2QueueDivert
    .u16    destQ
    .u16    srcQ
.ends 

// delete a standard entry in LUT2
.struct  struct_paComDelLut2Standard
    .u8   type
    .u8   index
    .u8   ctrlBitMap   
    .u8   rsvd
    
    .u16  match1        // MSW
    .u16  match0        // LSW
.ends

// Delete custom entry in LUT2
.struct struct_paComDelLut2Custom
    .u8   type
    .u8   index
    .u8   ctrlBitMap   
    .u8   rsvd
    
    .u32  match
.ends

#define PA_COM_DEL_LUT2_STANDARD   0
#define PA_COM_DEL_LUT2_CUSTOM     1


// Request Firmware version info
.struct struct_paCommandReqVer
    .u8   pdspNum
    .u8   rsvd1
    .u16  rsvd2
    
    .u16  version3       // Version ID - major, minor, tiny, nitpick 

    .u16  version2
    .u16  version1
    .u16  version0
.ends


// Statistics
// Classify1 statistics
#define t_nPackets                t0
#define t_nIpv4Packets            t1
#define t_nIpv4PacketsInner       t2
#define t_nIpv6Packets            t3
#define t_nIpv6PacketsInner       t4
#define t_nCustomPacketsC1        t5
#define t_nSrioPackets            t6
#define t_nLlcSnapFail            t7
#define t_nTableMatch             t8
#define t_nNoTableMatch           t9
#define t_nIpFrag                 t10
#define t_nIpDepthOverflow        t11
#define t_nVlanDepthOverflow      t12
#define t_nGreDepthOverflow       t13
#define t_nMplsPackets            t14
#define t_nParseFailC1            t15
#define t_nInvalidIPv6Opt         t16
#define t_nTxIpFrag               t17
//#define t_nInvalidComReplyDestC1  t16       // not used
#define t_nSilentDiscardC1        t18
#define t_nInvalidControlC1       t19
#define t_nInvalidState           t20
#define t_nSystemFail             t21

// Classify2 statistics
#define t_nPacketsC2              t22
//#define t_nInvldHdr               t22       // not used
#define t_nUdp                    t23
#define t_nTcp                    t24
#define t_nCustom                 t25
#define t_nCommandFailC2          t26       // not used
#define t_nInvalidComReplyDestC2  t27       // not used
#define t_nSilentDiscardC2        t28
#define t_nInvalidControlC2       t29

// Modify statistics
#define t_nCommandFail            t30

// Common statistics
#define t_nTxEthCrcErr            t31       

// Event reporting
.struct struct_paComEvents
    .u32  validEvents
.ends

// event definitions
#define  PA_EVENT_C1_SOP                  t0
#define  PA_EVENT_C2_SOP                  t1
#define  PA_EVENT_M_SOP                   t2
#define  PA_EVENT_L1_MATCH                t3
#define  PA_EVENT_L2_MATCH                t4
#define  PA_EVENT_L1_NO_MATCH             t5
#define  PA_EVENT_L2_NO_MATCH             t6
#define  PA_EVENT_C1_PARSE_FAIL           t7
#define  PA_EVENT_C2_PARSE_FAIL           t8
#define  PA_EVENT_IP_FRAG                 t9
#define  PA_EVENT_C1_SILENT_DISCARD       t10
#define  PA_EVENT_C2_SILENT_DISCARD       t11
#define  PA_EVENT_VLAN_MAX_DEPTH          t12
#define  PA_EVENT_IP_MAX_DEPTH            t13
#define  PA_EVENT_GRE_MAX_DEPTH           t14
#define  PA_EVENT_IP_INCOMPLETE_ROUTE     t15
#define  PA_EVENT_LUT1_TIMEOUT            t16
#define  PA_EVENT_DISCARD_HELD_PACKET     t17
#define  PA_EVENT_NO_FREE_PKT_ID          t18
#define  PA_EVENT_INVALID_CTRL            t19
#define  PA_EVENT_SYSTEM_ERR              t20


// Next header type values
#define PA_HDR_MAC               0
#define PA_HDR_VLAN              1
#define PA_HDR_MPLS              2
#define PA_HDR_IPv4              3
#define PA_HDR_IPv6              4
#define PA_HDR_IPv6_EXT_HOP      5
#define PA_HDR_IPv6_EXT_ROUTE    6
#define PA_HDR_IPv6_EXT_FRAG     7
#define PA_HDR_IPv6_EXT_DEST     8
#define PA_HDR_GRE               9
#define PA_HDR_ESP               10
#define PA_HDR_ESP_DECODED       11
#define PA_HDR_AUTH              12
#define PA_HDR_CUSTOM_C1         13
#define PA_HDR_PPPoE             14   
                                      
#define PA_HDR_SCTP              15
                                      
#define PA_HDR_UNKNOWN           16
#define PA_HDR_UDP               17
#define PA_HDR_UDP_LITE          18
#define PA_HDR_TCP               19
#define PA_HDR_GTPU              20
#define PA_HDR_ESP_DECODED_C2    21
#define PA_HDR_CUSTOM_C2         22


// Request stats
#define PA_STATS_TYPE_SYS         0    // System Statistics
#define PA_STATS_TYPE_USR         1    // User Defined Statistics

.struct struct_paComReqStats
    .u8  ctrlBitMap// a 1 bit clears the corresponding stat
    .u8  type      // Statistics Type
    .u16 numCnt    // number of stats to be cleared
                   // list of 16-bit stats index follows
.ends

#define t_pa_usr_stats_clr      t0  // clear stats entries 
                                    // system stats: all
                                    // user-defined stats: stats clear bitmaps follow
#define t_pa_usr_stats_req      t1  // Request statistics, applicable to user-defined stats only
                                    // since PASS needs t return system statistics all the time
                                    
#define PA_USR_STATS_CLEAR_ALL  0                                    


.struct struct_commonStats
    .u32  rsvd5
.ends

.struct struct_classify1Stats
    .u32   nPackets
    .u32   nIpv4Packets
    .u32   nIpv4PacketsInner
    .u32   nIpv6Packets
    .u32   nIpv6PacketsInner
    .u32   nCustomPackets
    .u32   nSrioPackets
    .u32   nLlcSnapFail
    .u32   nTableMatch
    .u32   nNoTableMatch
    .u32   nIpFrag
    .u32   nIpDepthOverflow
    .u32   nVlanDepthOverflow
    .u32   nGreDepthOverflow
    .u32   nMplsPackets
    .u32   nParseFail
    .u32   nInvalidIpv6Opt
    .u32   ntxIpFrag
    .u32   nSilentDiscard
    .u32   nInvalidControl
    .u32   nInvalidState
    .u32   nSystemFail
.ends

.struct  struct_classify2Stats
    .u32   nPackets
    .u32   rsvd2
    .u32   nUdp
    .u32   nTcp
    .u32   nCustom
    .u32   rsvd3
    .u32   rsvd4
    .u32   nSilentDiscard
    .u32   nInvalidControl
.ends

.struct  struct_modifyStats
    .u32   nCommandFail
.ends

// Exception routing
.struct struct_paComEroute
    .u32  routeBitmap
    // Followed by EROUTE_N_MAX struct_paForward defining exception routes
.ends

#define EROUTE_LUT1_FAIL                0   // Packet failed to match in L2/L3 (LUT1) table
#define EROUTE_VLAN_MAX_DEPTH           1   // Packet exceeded maximum number of VLAN tags
#define EROUTE_IP_MAX_DEPTH             2   // Packet exceeded maximum number of IP headers
#define EROUTE_MPLS_MAX_DEPTH           3   // Packet exceeded maximum number of MPLS headers
#define EROUTE_GRE_MAX_DEPTH            4   // Packet exceeded maximum number of GRE headers
#define EROUTE_PARSE_FAIL               5   // Packet failed to parse
#define EROUTE_LUT2_FAIL                6   // Packet failed to match in L4 (LUT2) table
#define EROUTE_IP_FRAG                  7   // IP fragmented packet
#define EROUTE_IPV6_OPT_FAIL            8   // Packet failed due to unsupported IPV6 option header
#define EROUTE_UDP_LITE_FAIL            9   // Udp lite checksum coverage invalid
#define EROUTE_ROUTE_OPTION             10  // IPv4 strict source route or IPv6 routing extension header
#define EROUTE_SYSTEM_FAIL              11  // Unknown system failure - should never happen
#define EROUTE_MAC_BROADCAST            12  // MAC broadcast packet 
#define EROUTE_MAC_MULTICAST            13  // MAC multicast packet 
#define EROUTE_IP_BROADCAST             14  // IP broadcast packet 
#define EROUTE_IP_MULTICAST             15  // IP multicast packet 
#define EROUTE_GTPU_MESSAGE_TYPE_1      16  // GTP-U PING Request packet
#define EROUTE_GTPU_MESSAGE_TYPE_2      17  // GTP-U PING Response packet
#define EROUTE_GTPU_MESSAGE_TYPE_26     18  // GTP-U Error Indication packet
#define EROUTE_GTPU_MESSAGE_TYPE_31     19  // GTP-U Supported Header Notification packet
#define EROUTE_GTPU_MESSAGE_TYPE_254    20  // GTP-U End Markr packet
#define EROUTE_GTPU_FAIL                21  // Packet failed due to GTPU parsing error or unsupporte dmessage types
#define EROUTE_PPPoE_FAIL               22  // Packet failed due to PPPoE session packet parsing error
#define EROUTE_PPPoE_CTRL               23  // PPPoE session stage non-IP packets
#define EROUTE_802_1ag                  24  // 802.1ag Packet
#define EROUTE_IP_FAIL                  25  // Packet failed due to invalid IP header
#define EROUTE_NAT_T_KEEPALIVE          26  // NAT-T Keep Alive packet where UDP Length = 9, data = 0xFF
#define EROUTE_NAT_T_CTRL               27  // NAT-T control packet where UDP Length > 12 and the first 4 payload bytes are equal to 0
#define EROUTE_NAT_T_DATA               28  // NAT-T IPSEC ESP data packet where UDP Length > 12 and the first 4 payload bytes are not equal to 0
#define EROUTE_NAT_T_FAIL               29  // Invalid NAT-T packet
#define EROUTE_GTPU_MATCH_FAIL          30  // Packet failed to match GTPU
#define EROUTE_N_MAX                    31  // Number of exceptaion routes

// Custom Classify for LUT1
.struct  struct_paC1CustomHdr
    
    .u8    idx
    .u8    nextHdr          // Next Header to be parsed 
    .u16   offset           // Offset to the match area
    .u16   nextHdrOffset    // Offset to the next header
    .u16   rsvd
.ends    
    
.struct  struct_paC1Custom
    .u32   bitmask0
    .u32   bitmask1
    .u32   bitmask2
    .u32   bitmask3
    
    .u32   bitmask4
    .u32   bitmask5
    .u32   bitmask6
    .u32   bitmask7
.ends

#define PA_MAX_C1_CUSTOM_TYPES           4


// Custom Classify for LUT2
.struct   struct_paC2Custom
    .u8   idx           
    .u8   bitSet
    .u8   ctrlBitMap
    .u8   hdrSize         
    
    .u16  offset3
    .u16  offset2
    .u16  offset1
    .u16  offset0
    
    .u8   bitMask3
    .u8   bitMask2
    .u8   bitMask1
    .u8   bitMask0
.ends

// control flag definition
#define t_lut2_custom_ctrl_use_link      t0

#define PA_MAX_C2_CUSTOM_TYPES           16


.struct struct_802_1ag_cfg
    .u8  ctrlBitMap
    .u8  rsvd1
    .u16 rsvd2
.ends

#define t_802_1ag_cfg_enable             t0
#define t_802_1ag_cfg_standard           t1

.struct struct_ipsec_nat_t_cfg
    .u8  ctrlBitMap
    .u8  rsvd
    .u16 udpPort
.ends

#define t_ipsec_nat_t_cfg_enable         t0    

.struct struct_gtpu_cfg
    .u8  ctrlBitMap   //   Control Bit Map 
    .u8  rsvd1
    .u16 rsvd2
.ends

#define t_pa_gtpu_control_use_link          t0
#define t_pa_gtpu_msg254_as_msg255          t1

// Packet Capture Header 
.struct struct_pcap_hdr
    .u8  numPorts       // Number of ports
	.u8  rsvd1
	.u16 rsvd2
.ends
	
// Packet Capture control configuration 
.struct struct_pcap_cfg
    .u8  capPort       // Capture port ID
	.u8  rsvd1
	.u16 rsvd2
	
	.u8  ctrlBitMap    // Control bit map - feature enable, indicate destination (mirror or capture)
	.u8  mport_flow    // mirror port number for dest = EMAC or flowID for dest = HOST
    .u16 destQueue     // Destination host queue where PASS will deliver the captured packets when dest = HOST	
	
	.u32 context       // Context returned as swInfo0 for matched packet when dest = HOST
.ends
// Packet Capture control Info 
.struct struct_pcap_info
	.u8  ctrlBitMap    // Control bit map - feature enable, indicate destination (mirror or capture)
	.u8  mport_flow    // mirror port number for dest = EMAC or flowID for dest = HOST
    .u16 destQueue     // Destination host queue where PASS will deliver the captured packets when dest = HOST
	
	.u32 context       // Context returned as swInfo0 for matched packet when dest = HOST
.ends

 // this bit defines whether the feature is enabled or disabled for packet capture per interface
#define t_pkt_cap_enable                    t0 
// this bit defines whether the destinatino for the captured packet is host (t1 = 1) or ethernet (t1 = 0)
#define t_pkt_cap_host                      t1


// default Route Header 
.struct struct_paComDroute
    .u8  numPorts       // Number of ports
	.u8  rsvd1
	.u16 rsvd2
.ends
	
// default packet route control configuration 
.struct struct_paComDrouteCfg
	.u8  ctrlBitMap       // Control bit map - feature enable
	.u8  capPort          // Capture port ID
	.u16 rsvd
    // followed by multicast, broadcast and unicast firmware forward routing information

.ends

// this bit defines whether the feature is enabled or disabled for packet routing per interface
#define t_default_route_mc_enable                    t0 
#define t_default_route_bc_enable                    t1 
#define t_default_route_uc_enable                    t2 
#define t_default_route_mc_pre_classify_enable       t3
#define t_default_route_bc_pre_classify_enable       t4


#define DROUTE_MULTICAST                0   // Default Multicast Route Info
#define DROUTE_BROADCAST                1   // Default Broadcast Route Info
#define DROUTE_UNICAST                  2   // Default Unicast Route Info

.struct struct_paComEQoS
  .u8  numPorts       // Number of ports
  .u8  scratch        // scratch/temp use
  .u16 rsvd
.ends

.struct struct_paComIfEQoS  
  .u8  ctrlBitMap     // Control bit map 
  .u8  flowBase       // base QoS Flow 
  .u16 queueBase      // base QoS Queue
  
  .u8  ingressDefPri  // Default ingress default priority
  .u8  port  
  .u16 vlanId
  
  // followed by route table for PBIT flow/queue route offset map
  // followed by route table for DSCP flow/queue route offset map
.ends
// Control bit map
#define t_eqos_dp_bit_mode                                  t0
#define t_eqos_pri_override                                 t1                 
#define t_eqos_vlan_override                                t2

.struct struct_paComIfEQoSRouteOffset
   // read 8 priorities at a time
   .u32 word0
   .u32 word1
   .u32 word2
   .u32 word3
.ends

.struct struct_paEQosScratch   
   .u8  status      // t0: vlan Tagged, 
                    // t1: IP Pkt
   .u8  priority    // 3 bit PCP value, if vlan tagged packet OR 6 bit DSCP pri value, if IP Packet
   .u8  vlanPri     // 3 bit PCP value
   .u8  dscp        // 6-bit DSCP
.ends

//  status indicator
#define  t_eqos_status_vlan_tag                           t0
#define  t_eqos_status_is_ip                              t1

#define  PA_SUB_EQOS_STATUS_UNTAGGED_NON_IP_PKT         0x00
#define  PA_SUB_EQOS_STATUS_VLAN_TAG_MASK               0x01
#define  PA_SUB_EQOS_STATUS_IP_MASK                     0x02
#define  PA_SUB_EQOS_STATUS_VLAN_TAG_IP_PKT             0x03

.struct struct_Ipv4qos
    .u8     VerLen
    .u8     Tos
    .u16    TotalLen
.ends

.struct struct_Ipv6qos
    .u32    ver_tclass_flow
.ends

// EQOS queue, flow information
.struct struct_eqosQueueFlow
    .u16  queue        // The destination queue where PASS will deliver the packets which require reassembly assistance 
    .u8   flowId       // Specify the CPPI flow which instructs how free queues are used for receiving packets
	.u8   rsvd
.ends

// Global Configuration
.struct struct_paComMaxCount
    .u8  vlanMaxCount
    .u8  ipMaxCount
    .u8  greMaxCount
    .u8  rsvd
.ends

.struct struct_paIpReassmCfg 
    .u8  numTrafficFlow // Maximum number of IP reassembly traffic flows supported, default = 0, maxmium = 32 
    .u8  destFlowId     // CPPI flow which instructs how the link-buffer queues are used for forwarding packets 
    .u16 destQueue      // Destination host queue where PASS will deliver the packets which require IP reassembly assistance 
.ends

.struct struct_paCmdSetCfg
    .u8  numCmdSets      // Number of command sets supported (32, 64)
    .u8  cmdSetSize      // maxmium size of each command set
    .u16 rsvd
.ends 

#define PA_USR_STATS_NUM_ENTRIES    512

.struct struct_paUsrStatsGlobCfg
    .u16 numCounters    //  Number of user-defined counters, default = 0, maxmium = 512
    .u16 num64bCounters //  Number of user-defined 64-bit counters, default = 0, maxmium = 256
.ends 

.struct struct_paQueueDivertCfg
    .u16 destQueue     //   Destination queue where PASS will deliver the LUT2 response packet which contains the
                       //   queue diversion information
    .u8  destFlowId    //   CPPI flow which instructs how the link-buffer queues are used for forwarding
                       //   the LUT2 response packets
    .u8  rsvd                                            
.ends       

.struct struct_paPktCtrlCfg
    .u16 ctrlBitMap     //   Control Bit Map 
    .u16 validBitMap    //   Valid Bit Map
    .u16 rxPaddingErrCntIndex  // Specify the user statistics index of Rx padding error counter
    .u16 txPaddingCntIndex     // Specify the user statistics index of Tx MAC padding counter
    .u8  egressDefPri          // global default priority for untagged non IP egress traffic for enhanced QoS mode
    .u8  rsvd1
    .u16 rsvd2
.ends

.struct struct_paMacPaddingCfg
    .u16 rxPaddingErrCntIndex  // Specify the user statistics index of Rx MAC padding error counter
    .u16 txPaddingCntIndex     // Specify the user statistics index of Tx MAC padding counter
.ends

.struct struct_paQueueBounceCfg
    .u16 ddrQueue      //   Bounce queue where PASS will deliver the host-routed packet with DDR bit set
    .u16 msmcQueue     //   Bounce queue where PASS will deliver the host-routed packet with MSMC bit set
.ends

#define t_pa_pkt_verify_proto_pppoe         t0
#define t_pa_pkt_verify_proto_ip            t1
#define t_pa_pkt_ctrl_mac_padding_chk       t2
#define t_pa_pkt_ctrl_ip_frags_to_eroute    t3
#define t_pa_pkt_ctrl_l3offset_use_inner_ip t4
#define t_pa_pkt_ctrl_ingress_pkt_capture   t5
#define t_pa_pkt_ctrl_egress_pkt_capture    t6
#define t_pa_pkt_ctrl_ingress_def_route     t7
#define t_pa_pkt_ctrl_eqos_mode             t8
#define t_pa_pkt_ctrl_valid_mac_padding_cnt t15

// Ingress packet capture, pppoe header check, mac padding check, ingress default route
#define PA_SUB_VALID_POST_MAILBOX0_MASK    0x00a5
// ipheader chk, ipfrag to eroute, l3toInner
#define PA_SUB_VALID_POST_MAILBOX12_MASK   0x001a
// egress packet capture, eqos mode
#define PA_SUB_VALID_POST_MAILBOX45_MASK   0x0140

//  Configure PA. These configurations apply globally (to all PDSPs). Any
//  PDSP can do the configuration
.struct  struct_paCommandConfig
    .u8   validFlag
    .u8   rsvd1
    .u16  rsvd2
    
    // Followed by:
    //  Configure max counts
    //  Configure outer IP Reassembly
    //  Configure inner IP Reassembly
    //  Configure command set
    //  Configure User-defined Statistics
    //  Configure Queue Diversion
    //  Configure Packet Verification 
    //  Configure Queue Bounce
.ends

#define t_paCmdConfigValidMaxCount      t0
#define t_paCmdConfigValidOutIpReassem  t1
#define t_paCmdConfigValidInIpReassem   t2
#define t_paCmdConfigValidCmdSet        t3
#define t_paCmdConfigValidUsrStats      t4
#define t_paCmdConfigValidQueueDivert   t5
#define t_paCmdConfigValidPktCtrl       t6
#define t_paCmdConfigValidQueueBounce   t7


// Command sizes. The assembler can't do all the SIZE(x)+SIZE(y)+... because of line size limitations
// Command size in 32 bit words is:  struct_paCommand:         4
//                                   struct_paCommandConfig:   1
//                                   struct_paComMaxCount:     1
//                                   struct_paIpReassmCfg:     1
//                                   struct_paIpReassmCfg:     1
//                                   struct_paCmdSetCfg:       1
//                                   struct_paUsrStatsCfg:     1
//                                   struct_paQueueDivertCfg:  1
//                                   struct_paPktCtrlCfg:      3
//                                   struct_paQueueBounceCfg:  1
//
// 
//                                               Total:       15   words = 60 bytes
#define PA_CONFIG_COMMAND_SIZE_CONFIG_PA              60
#define PA_CONFIG_COMMAND_SIZE_THROUGH_IN_IP_REASM    32  // Byte up to and including struct_paIpReassmCfg for inner IP


//  Sysmtem Configure These configurations apply globally (to all PDSPs). Any
//  PDSP can do the configuration
.struct  struct_paSystemConfig
    .u8   sysCode
    .u8   rsvd1
    .u16  rsvd2
    
    // Followed by one of the following strucrure specified by system code:
    //  configure exception routing
    //  configure custom1 classify
    //  configure custom2 classify
.ends

#define PA_SYSTEM_CONFIG_CODE_EROUTE         0
#define PA_SYSTEM_CONFIG_CODE_CUSTOM_LUT1    1
#define PA_SYSTEM_CONFIG_CODE_CUSTOM_LUT2    2
#define PA_SYSTEM_CONFIG_CODE_802_1AG        3
#define PA_SYSTEM_CONFIG_CODE_IPSEC_NAT_T    4
#define PA_SYSTEM_CONFIG_CODE_GTPU           5
#define PA_SYSTEM_CONFIG_CODE_IGRESS_PCAP    6
#define PA_SYSTEM_CONFIG_CODE_EGRESS_PCAP    7
#define PA_SYSTEM_CONFIG_CODE_DEFAULT_ROUTE  8
#define PA_SYSTEM_CONFIG_CODE_EQOS           9

// Command sizes. The assembler can't do all the SIZE(x)+SIZE(y)+... because of line size limitations
// Command size in 32 bit words is:  struct_paCommand:         4
//                                   struct_paSystemConfig:    1
//                                   struct_paComEroute:       1
//                EROUTE_N_MAX(30) * struct_paFwdPlace(4):   120
//                                   struct_paC1Custom:       10
//                                   struct_paC2Custom:        4
//                                   struct_pa802p1agDet:      1
//                                   struct_paIpsecNatTDet:    1  
//                                   struct_pa802p1agDet:      1
//                                   struct_paIpsecNatTDet:    1  
//                                   struct_pcap_info:         1 
//                MAX_PORTS (5)    * struct_pcap_cfg:          3(15)
//                                   struct_paComDroute:       1
//                MAX_PORTS (4)    * struct_paComDrouteCfg:    1(4)
// MAX_PORTS (4)* DROUTE_N_MAX (3) * struct_paFwdPlace(4):    12(48)
//                                   struct_paComEQoS:         1
//                                   struct_paComIfEQoS:       2(8)
//                        struct_paComIfEQoSRouteOffset:       4(16)  (vlanTbl)
//                      4*struct_paComIfEQoSRouteOffset:      32(128)  (dscpTbl)
//
//                                               EROUTE:       126 words = 504 bytes
//                                               CUSTOM LUT1:   15 words = 60 bytes
//                                               CUSTOM LUT2:    9 words = 36 bytes
//                                               802.1ag Detect: 6 words = 24 bytes 
//                                               NAT-T Detect:   6 words = 24 bytes
//                                               GTPU Config:    6 words = 24 bytes
//                                               PCap Config:    9 words = 36 bytes  (minimum)
//                                               PCap Config:   21 words = 84 bytes  (maximum)
//                                               DefRoute Cfg:  19 words = 76 bytes  (minimum)
//                                               DefRoute Cfg:  58 words = 232 bytes (maximum)
//                                               EQOS Cfg:      44 words = 176 bytes (minimum)
//                                               EQOS Cfg:     158 words = 632 bytes (maximum)
//
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EROUTE         504  
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT1     60  
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT2     36  
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_802_1AG         24 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_IPSEC_NAT_T     24 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_GTPU            24 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_PCAP            36
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_DEF_ROUTE       76
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EQOS           176

#define PA_CONFIG_COMMAND_SIZE_THROUGH_EROUTE     24         // Byte up to and including struct_paComEroute
#define PA_CONFIG_COMMAND_SIZE_THROUGH_SYSCFG     20  

// User-defined Statistics configuration
.struct   struct_paUsrStatsCfg
    .u8  ctrlBitMap
    .u8  rsvd
    .u16 nEntries
    // Followed by up to 256 copies of struct_paSingleRoute
.ends

#define t_pa_usr_stats_cfg_clr_all      t0  // clear all entries 

// User-defined Statistics entry
.struct   struct_paUsrStatsEntry
    .u16  index                   //Index to the counter  
    .u16  lnkIndex                //Index to the next layer counter
                                  //b15: No Link
                                  //b14: 0: pkt counter; 1: byte counter
                                  //b13: 1: Disable
.ends

// User-defined Statistics control block
.struct   struct_paUsrStatsCB
    .u16  lnkIndex                //b0-b11 linking index
                                  //b13   disable ctrl 0:enable; 1: disable
                                  //b14   counter type 0:pkt counter; 1: byte counter
                                  //b15
.ends

#define t_pa_usr_stats_cb_disable       t13     // Indicate that the counter is disabled
#define t_pa_usr_stats_cb_byte_cnt      t14     // Indicate that it is a byte counter
#define t_pa_usr_stats_cb_no_lnk        t15     // Indicate there is no link
#define PA_USR_STATS_LNK_MASK_MSB       0x0F    // Link Mask for higher byte 

// Multi Route configuration
.struct   struct_paCommandMultiRoute
    .u8  idx
    .u8  mode
    .u8  nRoutes
    .u8  rsvd
    
    // Followed by up to PA_MAX_HOST_PKT_DUP copies of struct_paSingleRoute
.ends

// Multi-route entry
.struct   struct_paMultiRouteEntry
    .u8  ctrlFlags
    .u8  flowId
    .u16 queue
    .u32 swInfo0
.ends

// Multi-route control flags 
#define t_pa_multi_route_ctrl_desc_only        t0
#define t_pa_multi_route_ctrl_replace_swinfo0  t1
#define t_pa_multi_route_ctrl_active           t7

#define PA_MAX_HOST_PKT_DUP              8
#define PA_MULTI_ROUTE_NUM_ROUTES        32
#define PA_MULTI_ROUTE_NEXT_FREE_IDX     0xff

#define PA_COMMAND_MULTI_ROUTE_MODE_ADD  0
#define PA_COMMAND_MULTI_ROUTE_MODE_DEL  1
#define PA_COMMAND_MULTI_ROUTE_MODE_GET  2

// Command Set configuration
.struct   struct_paCommandSet
    .u8  idx
    .u8  nCmd
    .u16 rsvd
    
    // follow by up to 60 bytes command
    
.ends

#define PA_CMD_SET_NUM_ENTRIES   64
#define PA_CMD_SET_SIZE         124 

// CRC engine configuration */
.struct   struct_paCommandCfgCrc
    .u8   ctrlBitMap         // It is formatted by the PA LLD, just pass it as part of CDE command
    .u8   rsvd1
    .u16  rsvd2 
    
    .u32  initVal
    
    // follow by 16 32-bit table value
.ends

// Command header
.struct struct_paCommand
    .u32  commandResult
    
    .u8   command
    .u8   magic
    .u16  comId
    
    .u32  retContext
    
    .u16  replyQueue
    .u8   replyDest
    .u8   flowId
.ends

// Define Command values
// They are corresponding to the command values defined at pa/src/pafrm.h 
#define PA_CONFIG_COMMAND_RSVD              0
#define PA_CONFIG_COMMAND_ADDREP_LUT1       1
#define PA_CONFIG_COMMAND_DEL_LUT1          2
#define PA_CONFIG_COMMAND_ADDREP_LUT2       3
#define PA_CONFIG_COMMAND_DEL_LUT2          4
#define PA_CONFIG_COMMAND_CONFIG_PA         5
#define PA_CONFIG_COMMAND_REQ_STATS         6
#define PA_CONFIG_COMMAND_REQ_VERSION       7
#define PA_CONFIG_COMMAND_MULTI_ROUTE       8
#define PA_CONFIG_COMMAND_CRC_ENGINE        9
#define PA_CONFIG_COMMAND_CMD_SET          10
#define PA_CONFIG_COMMAND_USR_STATS        11
#define PA_CONFIG_COMMAND_SYS_CONFIG       12

// Command magic value
#define PA_CONFIG_COMMAND_SEC_BYTE  0xce

// Command return values
#define PA_COMMAND_RESULT_SUCCESS                      0   // Must be 0
#define PA_COMMAND_RESULT_NO_COMMAND_MAGIC             1   // Command magic value not found
  
#define PA_COMMAND_RESULT_INVALID_CMD                  2   // Invalid command identifier
  
// Add entry to LUT1 fails
#define PA_COMMAND_RESULT_LUT1_TYPE_INVALID            3   // Invalid type, custom or standard IP/ethernet
#define PA_COMMAND_RESULT_LUT1_INDEX_INVALID           4   // Invalid LUT1 index (0-63) or no free indices available
#define PA_COMMAND_RESULT_LUT1_MATCH_DEST_INVALID      5   // Sent a match packet to q0 on c1 or c2 - this is illegal.
#define PA_COMMAND_RESULT_LUT1_NMATCH_INVALID          6   // Previous match forward info was somewhere in chunk domain
#define PA_COMMAND_RESULT_LUT1_INVALID_KEYS            7   // Invalid combination found in the key value
  
// Lut 2 warnings since the lut can be configured without pdsp
#define PA_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES        8
#define PA_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT    9

// Lut2 errors
#define PA_COMMAND_RESULT_LUT2_ADD_BUSY                10
  
// Not enough room in stats request packet for the reply
#define PA_COMMAND_RESULT_WARN_STATS_REPLY_SIZE        11
  
// Command sent to PDSP which couldn't handle it
#define PA_COMMAND_RESULT_INVALID_DESTINATION          12
  
// Add/Delete/Read entries to multi route table
#define PA_COMMAND_RESULT_MULTI_ROUTE_NO_FREE_ENTRIES  13   // Asked to use a free entry, but none found
#define PA_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX      14   // Illegal index value used
#define PA_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE     15   // Illegal multi route mode used

// Packet size didn't match command
#define PA_COMMAND_RESULT_INVALID_PKT_SIZE             16

#define PA_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX        17
#define PA_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX        18
#define PA_COMMAND_RESULT_INVALID_CMDSET_IDX           19
#define PA_COMMAND_RESULT_USR_STATS_INVALID_CONFIG     20
#define PA_COMMAND_RESULT_LUT2_FULL                    21

// Packet descriptor - course view
.struct struct_pktDscCourse
    .u32    pif0
    .u32    pif1
    .u32    pif2
    .u32    pif3
    .u32    pif4
    .u32    timestamp
    .u32    swinfo0
    .u32    swinfo1
.ends


// Packet descriptor, fine view
.struct struct_pktDscFine
    .u32   pif0
    
    .u8    pvtData1
    .u8    pvtData2
    .u8    validPsSize
    .u8    physPsSize
    
    .u8    pktType_pvtFlags
    .u8    psFlags_errorFlags
    .u8    srcId
    .u8    flowIdx
    
    .u16   ctrlDataSize
    .u16   pktDataSize
    
    .u16   pktId
    .u16   destQ
    
    .u32   timestamp
    
    .u32   swinfo0
    
    .u32   swinfo1

.ends

#define t_pktIdAllocated        t14
#define PA_PKT_ID_MASK          0x3fff
#define PA_PKT_TYPE_MASK        0xf8
#define NOT_PA_PKT_TYPE_MASK    0x07
#define PA_PKT_TYPE_SHIFT       3
#define PA_PKT_PS_FLAGS_MASK        0xf0
#define PA_PKT_PS_EMAC_PORTS_MASK   0x70

#define NOT_PA_PKT_PS_FLAGS_MASK    0x0f
#define PA_PKT_PS_FLAGS_SHIFT   4

#define PA_PKT_TYPE_SRIO_TYPE_9     30
#define PA_PKT_TYPE_SRIO_TYPE_11    31

#define t_pkt_desc_err_flag_crc         t1
#define t_pkt_desc_err_flag_chksum2     t2
#define t_pkt_desc_err_flag_chksum1     t3

// Compute Checksum/CRC command
.struct  struct_cmdChkCrc
    .u8  cmdId
    .u8  flags         // bits 0-3 are the cde crc flags. Bit 7 is the checksum negative 0 bit
    .u16 startOffset
    
    .u16 length
    .u16 resultOffset
    
    .u16 initVal
    .u16 rsvd
.ends

#define t_paFlagsChksumNeg0    t7
#define PA_CRC_FLAGS_MASK      0x0f
#define NOT_PA_CRC_FLAGS_MASK  0xf0
#define PA_CRC_FLAG_CRC_OFFSET_VALID        0x01
#define PA_CRC_FLAG_CRC_OFFSET_FROM_DESC    0x02
#define PA_CHKSUM_FALG_NEGATIVE             0x01


// Blind Patch (Tx)
.struct  struct_blindPatch
    .u8  cmdId_Len
    .u8  cmdLen_insert   // bits 7:4 are the command length, in 4 byte units, bit 3 is the insert flag (if clear overwrite), bit 2 is the delete flag
    .u16 patchOffset
    
    .u8  byte0
    .u8  byte1
    .u8  byte2
    .u8  byte3
    
    .u8  byte4
    .u8  byte5
    .u8  byte6
    .u8  byte7
    
    .u8  byte8
    .u8  byte9
    .u8  byte10
    .u8  byte11
    
    .u8  byte12
    .u8  byte13
    .u8  byte14
    .u8  byte15
.ends

#define PA_BLIND_PATCH_LEN_MASK  0x1f

.struct struct_bPatchStub
    .u8  cmdId_Len
    .u8  cmdLen_insert   // bits 7:4 are the command length, in 4 byte units, bit 3 is the insert flag (if clear overwrite)
    .u16 patchOffset
.ends

#define t_paBlindPatchOverwrite t3
#define t_paBlindPatchDelete t2    // Overwrite flag will be set when this flag is set 

// Next Route command
.struct  struct_nextRoute
    .u8  cmdId_N_E_Dest
    .u8  flowId
    .u16 destQueue
    
    .u32 swInfo0
    .u32 swInfo1
    .u8  pktType_psFlags
    .u16 statsIndex
    .u8  ctrlFlags
.ends

// Next Route stub
.struct  struct_nextRouteStub
    .u8  cmdId_N_E_Dest
    .u8  flowId
    .u16 destQueue
.ends

#define t_nextRouteN    t4
#define t_nextRouteE    t3
#define PA_NEXT_ROUTE_DEST_MASK      0x07
#define NOT_PA_NEXT_ROUTE_DEST_MASK  0xf8
#define t_nextRoute_ctrl_l2padding   t0
#define t_nextRoute_ctrl_txUsrStats  t1

// Report Timestamp  command
.struct  struct_reportTs
    .u8  cmdId
    .u8  flowId
    .u16 destQueue
    
    .u32 swInfo0
.ends

#define PA_RPT_TIME_STAMP_NOT_CMD_MASK  0x1F

// Ip Fragmentation command
.struct  struct_ipFragCmd
    .u8  cmdId_subCode
    .u8  ipOffset
    .u16 mtuSize
.ends

// Patch Message Length command
.struct  struct_patchMsgLenCmd
    .u8  cmdId_subCode
    .u8  offset               //offset to message length
    .u16 msgLen               //b15 (message length field size flag): 0: 2-byte;1:4-byte
.ends    

#define t_patch_msg_len_size32          t15
// EMAC CRC verify command
.struct  struct_emacCrcVerifyCmd
    .u8  cmdId_subCode
    .u8  psFlags              // specify destination port number in psFlags field of packet descriptor
    .u16 rsvd
.ends


#define PA_SUB_CMD_CODE_MASK            0x1F
#define PA_SUB_CMD_CODE_DUMMY           0
#define PA_SUB_CMD_CODE_IP_FRAG         1
#define PA_SUB_CMD_CODE_PATCH_MSG_LEN   2
#define PA_SUB_CMD_CODE_EMAC_CRC_VERIFY 3

// General message
.struct  struct_msg
    .u8   cmdId
    .u8   rsvd1
    .u16  rsvd2
.ends

#define PA_MSG_CMD_ID_SHIFT      5

// PAFRM receive commands related definitions 

// 
// There are the following two groups of PAFRM receive commands:
// PAFRM short commands which can be used as part of the routing info 
// PAFRM commands which can be used within a command set
//
 
#define PA_RX_CMD_NONE           0

// short commands 
#define PA_RX_CMD_CMDSET            1      // Execute Command set
#define PA_RX_CMD_INSERT            2      // Insert up to two types at the current location 
#define PA_RX_CMD_USR_STATS         3      // Increment the specific user-statistics chain 
#define PA_RX_CMD_CMDSET_USR_STATS  4      // Increment the specific user-statistics chain and execute command set 

// command set commands 
#define PA_RX_CMD_NEXT_ROUTE        11     // Specify the next route
#define PA_RX_CMD_CRC_OP            12     // CRC generation or verification 
#define PA_RX_CMD_COPY_DATA         13     // Copy data to the PS Info section
#define PA_RX_CMD_PATCH_DATA        14     // Insert or pacth packet data at the specific location
#define PA_RX_CMD_REMOVE_HDR        15     // Remove the parsed packet header
#define PA_RX_CMD_REMOVE_TAIL       16     // Remove the parsed packet tail 
#define PA_RX_CMD_MULTI_ROUTE       17     // Duplicate packet to multiple destinations 
#define PA_RX_CMD_VERIFY_PKT_ERROR  18     // Verify packet error based on error flags
#define PA_RX_CMD_SPLIT             19     // Payload splitting 


// Rx command Header 
// It can be shared with any command which caontains 0 to 2-byte parameters
.struct struct_rxCmdHdr
    .u8     cmd
    .u8     len      // command total length 
    .u16    rsvd
.ends

// Rx command set (short)
.struct struct_rxCmdSet
    .u8     cmd
    .u8     index    // command set index 
    .u16    rsvd
.ends

// Rx insert command  (short)
.struct struct_rxCmdInsert
    .u8     cmd
    .u8     numBytes // number of bytes to be inserted 
    .u8     data1
    .u8     data0
.ends

// Rx user-defined statistics update  (short and command set)
.struct struct_rxCmdUsrStats
    .u8     cmd
    .u8     len
    .u16    index    // user stats index  
.ends

// Rx command set and user-defined statistics update (short)
.struct struct_rxCmdSetUsrStats
    .u8     cmd
    .u8     setIndex
    .u16    statsIndex    // user commands update index 
.ends

// 
//  Rx Next Route command (cmdset command)
//  
//  The SW Info should be present in the descriptor
//  The destination is always CDMA (Host, SA and ETH)
// 
//  Note: The routing to SRIO from command set is not required at this moment.
//        This structure needs to be enhanced to support SRIO routing
//       
.struct struct_rxCmdNextRoute
    .u8    ctrlFlags
    .u8    multiRouteIndex
    .u8    psFlags
    .u8    rsvd
.ends

#define t_rx_cmd_next_route_ctrl_multi_route    t0
#define t_rx_cmd_next_route_ctrl_emac_route     t1
#define t_rx_cmd_next_route_ctrl_psflags_valid  t2


// Rx CRC verification command (cmdset command) 
.struct struct_rxCmdCrcOp
    .u8   ctrlFlags
    .u8   lenAdjust
    .u8   rsvd
    .u8   startOffset
    .u16  len
    .u16  lenOffset
    .u16  lenMask
    .u16  crcOffset
.ends

#define t_rx_cmd_crc_op_ctrl_len_in_header         t7
#define t_rx_cmd_crc_op_ctrl_crc_follow_payload    t6
#define t_rx_cmd_crc_op_ctrl_frame_type_included   t5 
#define t_rx_cmd_crc_op_ctrl_len_offset_negative   t4


#define PA_RX_CMD_CRC_OP_FRAME_TYPE_MASK                    0x0f
#define PA_RX_CMD_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE2    0
#define PA_RX_CMD_CRC_OP_FRAME_TYPE_IUB_FP_HS_DSCH_TYPE3    1

// Rx Split command  (Cammand set command
.struct struct_rxCmdSplitOp
    .u8   ctrlFlags    // Split operation control information as defined below 
    .u8   startOffset  // Byte location, from the protocol header, where the payload or frame begins 
    .u8   frameType    // WCDMA Frame type
    .u8   rsvd1        // alignment
    .u8   hdrSize      // place holder for header offset
    .u8   flowId       // CPPI flow which instructs how link-buffer queues are used for sending payload packets
    .u16  destQueue    // Host queue for the payload packet 
.ends 

#define t_rx_cmd_split_op_ctrl_frame_type_included   t7 

#define PA_RX_CMD_SPLIT_OP_FRAME_TYPE_MASK                  0x0f

// Rx Copy Command (cmdset command)
.struct  struct_rxCmdCopy   
    .u8  ctrlFlags
    .u8  srcOffset
    .u8  destOffset
    .u8  numBytes
.ends

#define t_rx_cmd_copy_ctrl_from_end                t0

// Rx Patch Command (cmdset command)
.struct  struct_rxCmdPatch   
    .u8  ctrlFlags
    .u8  offset
    .u8  numBytes
    .u8  rsvd
    .u32 data1
    .u32 data2
    .u32 data3
    .u32 data4
    // another 16 bytes may follow
.ends

#define t_rx_cmd_patch_ctrl_insert                 t0
#define t_rx_cmd_patch_ctrl_mac_hdr                t1
#define t_rx_cmd_patch_ctrl_delete                 t2

// Rx MultiRoute Command (cmdset command)
.struct  struct_rxCmdMultiRoute
    .u8  cmd
    .u8  len      // command total length */
    .u8  index
    .u8  rsvd
.ends

// Rx Packet Error verification command 
.struct  struct_rxCmdVerifyPktErr
  .u8   errMask
  .u8   forwardType
  .u8   flowId
  .u8   rsvd1
  .u16  queue
  .u16  rsvd2
  .u32  swInfo0
.ends  

#define PA_RX_PKT_ERR_IP_CHECKSUM       0x08
#define PA_RX_PKT_ERR_L4_CHECKSUM       0x04
#define PA_RX_PKT_ERR_CRC               0x02


#endif // _PDSP_PA_H
