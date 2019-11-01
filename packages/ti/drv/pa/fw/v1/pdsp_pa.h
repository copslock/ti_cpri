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
    .u8   paPdspRouter    //reserved
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
    .u8   priority
    .u16  rsvd2
    .u32  rsvd3
    .u32  cmd
.ends

#define t_pa_eth_ps_flags_crc_present      t7
#define t_pa_eth_ps_flags_crc_cantagnoli   t6
#define PAFRM_ETH_PS_FLAGS_CTRL_MASK       0xc0
#define PAFRM_ETH_PS_FLAGS_PORT_MASK       0x1f

// Packet routing info for packets sent to PA
.struct struct_paForwardPa
    .u8   dest
    .u8   custType
    .u8   custIdx
    .u8   ctrlflags
    .u32  context
    .u32  cmd
.ends

#define t_pa_cascaded_forwarding                t0
#define t_pa_fwd_ctrlflags_pkt_mark             t1      // Mark the entry per ACL rule
#define t_pa_fwd_ctrlflags_pkt_drop             t2      // Indicate that the packet should be dropped after reassembly per ACL rule
#define t_pa_l2_capture                         t3      // Indicate that the (L2) packet should be captured at the next stage


// Packet routing info for packets sent to egress path
.struct struct_paForwardEf
    .u8   ctrlflags
    .u8   validBitMap   // Egress record valid bit map, if flow cache lookup is not enabled */ 
    .u16  rsvd1
    .u8  lvl1RecIndex   // Egress Flow level one record index
    .u8  lvl2RecIndex   // Egress Flow level two record index
    .u8  lvl3RecIndex   // Egress Flow level three record index
    .u8  lvl4RecIndex   // Egress Flow level four record index
    .u32  rsvd2
.ends

#define t_pa_ef_ctrlflags_fc_lookup              t0      // flow cache lookup 

// egress flow record valid bits
#define t_pa_ef_valid_rec_lvl1  t0
#define t_pa_ef_valid_rec_lvl2  t1
#define t_pa_ef_valid_rec_lvl3  t2
#define t_pa_ef_valid_rec_lvl4  t3

// Packet routing info for EOAM packets sent to HOST
.struct struct_paForwardEoam
    .u32  context
    .u16  statsIndex 
    .u8   megLevel    
    .u8   rsvd1
    .u32  rsvd2
.ends

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
#define PA_DEST_CDMA0              0           // Packets to Global CDMA
#define PA_DEST_CDMA1              1           // Packets to Local CDMA
#define PA_DEST_ETHERNET1          2           // Packets to Ethernet TX
#define PA_DEST_ETHERNET2          3           // Packets to Ethernet TX
#define PA_DEST_ETHERNET3          4           // Packets to Ethernet TX
#define PA_DEST_ETHERNET4          5           // Packets to Ethernet TX
#define PA_DEST_ETHERNET5          6           // Packets to Ethernet TX
#define PA_DEST_ETHERNET6          7           // Packets to Ethernet TX
#define PA_DEST_ETHERNET7          8           // Packets to Ethernet TX
#define PA_DEST_ETHERNET8          9           // Packets to Ethernet TX
#define PA_DEST_INGRESS0           10          // Packets to Cluster Ingress 0
#define PA_DEST_INGRESS1           11          // Packets to Cluster Ingress 1
#define PA_DEST_INGRESS2           12          // Packets to Cluster Ingress 2
#define PA_DEST_INGRESS3           13          // Packets to Cluster Ingress 3
#define PA_DEST_INGRESS4           14          // Packets to Cluster Ingress 4
#define PA_DEST_POST               15          // Packets to Cluster Post Processing
#define PA_DEST_EGRESS0            16          // Packets to Cluster Egress 0
#define PA_DEST_EGRESS1            17          // Packets to Cluster Egress 1
#define PA_DEST_EGRESS2            18          // Packets to Cluster Egress 2
#define PA_DEST_REASM              19          // Packets to Reasm Accelerator
#define PA_DEST_ACE0               20          // Placeholder for model
#define PA_DEST_ACE1               21          // Placeholder for model
#define PA_DEST_STATSBLOC          22          // Packets to Statsbloc

#define PA_DEST_CDMA               PA_DEST_CDMA0
#define PA_DEST_CDMA_LOC           PA_DEST_CDMA1
#define PA_DEST_ETH                PA_DEST_ETHERNET1

// Virtual number used by next route command only
#define PA_DEST_NR_ACE0        3
#define PA_DEST_NR_ACE1        4 
#define PA_DEST_NR_SRIO        7                  


#define PA_DEST_DISCARD     0xFF

// Assigning names based on PDSP functions */
#define PA_DEST_PA_C1_0         PA_DEST_INGRESS0
#define PA_DEST_PA_C1_1         PA_DEST_INGRESS1
#define PA_DEST_PA_C1_2         PA_DEST_INGRESS4 
#define PA_DEST_PA_C2           PA_DEST_INGRESS4
#define PA_DEST_PA_M_0          PA_DEST_POST
#define PA_DEST_PA_M_1          PA_DEST_EGRESS2

// Packet Routing info
.struct struct_paForward
    .u8   forwardType
    .u8   flowId
    .u16  queue
.ends

#define PA_FORWARD_TYPE_HOST        0
#define PA_FORWARD_TYPE_SA          1
#define PA_FORWARD_TYPE_PA          2
#define PA_FORWARD_TYPE_ETH         3
#define PA_FORWARD_TYPE_SRIO        4
#define PA_FORWARD_TYPE_SA_DIRECT   5
#define PA_FORWARD_TYPE_DISCARD     6
#define PA_FORWARD_TYPE_EFLOW       7
#define PA_FORWARD_TYPE_EOAM        8

#define t_pa_fwd_type_ctrl_use_loc_dma      t7
#define PA_FORWARD_TYPE_MASK                0x0F

#define PA_FOWARD_QUEUE_MASK                0x3FFF
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

#define PA_LUT1_INDEX_LAST_FREE       256
#define PA_COM_ADD_LUT1_STANDARD      0
#define PA_COM_ADD_LUT1_SRIO          1
#define PA_COM_ADD_LUT1_CUSTOM        2
#define PA_COM_ADD_LUT1_VLINK         3    // MAC/IP/ESP entry with link 

// Both standard and custom add have this header
.struct struct_paComAddL1Hdr
    .u16  index           // LUT1 index
    .u8   type            // Custom or Standard
    .u8   custIndex       // Vaild only if type is custom 
    .u16  vLinkNum        // Virtual Link number if used 
    .u16  statsIndex      // entry statistic index (Flow Cache only)
    
.ends

// May be merged with the previous structure
.struct struct_paCmdAddL1CmdHdr
    .u16 range1Hi  // Range High for bytes 44-45
    .u16 range0Hi  // Range High for bytes 42-43
    .u32 CBWords1  // Care Bits Word1
    .u32 CBWords2  // Care Bits Word2
    .u16 bitMask   // BitMask for Bytes 1-2
    .u16 priority  // Record priority "score", relative index
.ends 

// Standard LUT1 view for View1, View2 and Vire3
.struct struct_Lut1View
    .u32 word0
    .u32 word1
    .u32 word2
    .u32 word3
.ends

.struct struct_Lut1V1
    .u32 word0
    .u32 word1
    .u32 word2
    .u32 word3
.ends
    
.struct struct_Lut1V2
    .u32 word0
    .u32 word1
    .u32 word2
    .u32 word3
.ends

.struct struct_Lut1V3
    .u32 word0
    .u32 word1
    .u32 word2
    .u32 word3
.ends


// EF Command and Command Response
// Host will write the command and wait for it be executed by PDSP
// PDSP acknowledge that the command has been processed by 
// clearing the command field to 0
//
// Command Word0: Command
// /--------------------------------------------------------\
// | 31    24   | 23      16  | 15                        0 |
// |  Command   |  Param1     |       Param2                |
// \--------------------------------------------------------/
// Command Word1: Command Respond
// /--------------------------------------------------------\
// | 31                                                  0 |
// |           Command respond code                        |
// \--------------------------------------------------------/
// Command Word 2/3 Reserved
// Command Word 4-19: Command Specific Data

//
//  Command Definitions
//        

//
//  Egress Flow record configuration command
//  param1: not used
//  param2: record index
// 
#define  PA_CMD_CFG_EF_RECORD           1    

//
//  Command Response Code
//
//
#define  PA_CMD_RESP_OK                 0
#define  PA_CMD_RESP_ERROR              1
#define  PA_CMD_RESP_UNSUPP             2
#define  PA_CMD_RESP_BAD_PARAMS         3

.struct  struct_paCfgCommand
    .u8  code
    .u8  param1
    .u16 param2
    .u32 response
.ends

#define  FA_CMD_DATA_BUF_OFFSET         16
#define  FA_CMD_DATA_BUF_SIZE           64


// Add LUT1 command
// L1 Header, Command Hdr,  View1, 2 and 3, Route and NextFail Route (Reference from Global Address)  4 + 16*6= 100 bytes

// LUT1 structure at lut1_lut2.h

// Delete LUT1 entry
.struct struct_paCommandDelLut1
    .u16 index
    .u16 rsvd2
.ends

// Add/replace a standard entry in LUT2
// This is followed by match routing info
.struct  struct_paComAddLut2Standard
    .u8   type
    .u8   index         // custom Index
    .u8   ctrlBitMap   
    .u8   l4Type
    
    .u16  inkTableIdx
    .u16  srcVC        
    .u16  matchHi        
    .u16  matchLo          
.ends

// Add/replace custom entry in LUT2
// This is followed by match routing info
.struct struct_paComAddLut2Custom
    .u8   type
    .u8   index         // custom Index
    .u8   ctrlBitMap   
    .u8   l4Type
    
    .u16  inkTableIdx
    .u16  srcVC        
    .u8   match0
    .u8   match1
    .u8   match2
    .u8   match3
.ends

// control flags
#define t_pa_lut2_ctrl_replace      t0
#define t_pa_lut2_ctrl_gtpu         t1   // Disable GTPU if GTPU port (2152) is added into LUT2 table */
                                         // Re-enable GTPU if GTPU port (2152) is deleted from LUT2 table */
#define t_pa_lut2_ctrl_queue_divert t2   // Enable Queue divert, Note: It is for replaced entry only                                          

#define PA_COM_ADD_LUT2_STANDARD    0
#define PA_COM_ADD_LUT2_CUSTOM      1

// Layer 2 packet Type 
#define PA_L4_PKT_TYPE_PORT16            0x80
#define PA_L4_PKT_TYPE_PORT32            0x40
#define PA_L4_PKT_TYPE_CUSTOM            0x20


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
    .u8   l4Type
    
    .u16  inkTableIdx
    .u16  srcVC        
    .u16  matchHi        
    .u16  matchLo          
.ends

// Delete custom entry in LUT2
.struct struct_paComDelLut2Custom
    .u8   type
    .u8   index
    .u8   ctrlBitMap   
    .u8   l4Type
    
    .u16  inkTableIdx
    .u16  srcVC        
    .u8   match0
    .u8   match1
    .u8   match2
    .u8   match3
.ends

#define PA_COM_DEL_LUT2_STANDARD   0
#define PA_COM_DEL_LUT2_CUSTOM     1

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
#define t_nIdAllocationFail       t31       // not used

#define PA_STATS_UPDATE        

#define PA_STATS_C1_N_PKTS              0
#define PA_STATS_IPV4_PKTS              1
#define PA_STATS_INNER_IPV4_PKTS        2
#define PA_STATS_IPV6_PKTS              3
#define PA_STATS_INNER_IPV6_PKTS        4
#define PA_STATS_C1_CUSTOM_PKTS         5
#define PA_STATS_SRIO_PKTS              6
#define PA_STATS_LLC_SNAP_FAIL          7
#define PA_STATS_LUT1_MATCH             8
#define PA_STATS_LUT1_NO_MATCH          9
#define PA_STATS_IP_FRAG               10
#define PA_STATS_IP_DEPTH_OVERFLOW     11
#define PA_STATS_VLAN_DEPTH_OVERFLOW   12
#define PA_STATS_GRE_DEPTH_OVERFLOW    13
#define PA_STATS_MPLS_PKTS             14
#define PA_STATS_PARSE_FAIL            15
#define PA_STATS_INVALID_IPV6_OPTION   16
#define PA_STATS_TX_IP_FRAG            17
#define PA_STATS_C1_DISCARD            18
#define PA_STATS_C1_INVALID_CONTROL    19
#define PA_STATS_C1_INVALID_STATE      20
#define PA_STATS_C1_SYSTEM_FAIL        21

#define PA_STATS_C2_N_PKTS             22
#define PA_STATS_UDP_PKTS              23
#define PA_STATS_TCP_PKTS              24
#define PA_STATS_C2_CUSTOM_PKTS        25
#define PA_STATS_C2_DISCARD            28
#define PA_STATS_C2_INVALID_CONTROL    29

#define PA_STATS_COMMAND_FAIL          30


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
#define PA_HDR_MAX               PA_HDR_CUSTOM_C2 

// Request stats
// To be removed: Stats Module
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


#define EF_EROUTE_FC_LUT1_FAIL           0   // Packet failed to match in Flow Cache (LUT1) table
#define EF_EROUTE_PARSE_FAIL             1   // Packet failed to parse
#define EF_EROUTE_IP_FRAG                2   // Unexpected fragmented packets
#define EF_EROUTE_IPV6_OPT_FAIL          3   // Packet failed due to unsupported IPV6 option header
#define EF_EROUTE_IP_OPTIONS             4   // Unexpected IP packet with IPv4 options and IPv6 extension headers
#define EF_EROUTE_IP_EXPIRE              5   // IP packet with TTL expired or Hop Limitis reached
#define EF_EROUTE_TCP_CTRL               6   // TCP Control Packet
#define EF_EROUTE_INVALID_REC            7   // Invalid Egress Flow record
#define EF_EROUTE_SYSTEM_FAIL           10   // Unknown system failure - should never happen
#define EF_EROUTE_N_MAX                 11   // Number of exception routes


// Custom Classify for LUT1
// Ingress 1 only (From MAC/SRIO to custom)
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
// To Ingress 4 only
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

// Ingress 0 MAC: Disabled by default
// Support for backward-compatible only   
.struct struct_802_1ag_cfg
    .u8  ctrlBitMap
    .u8  rsvd1
    .u16 rsvd2
.ends

#define t_802_1ag_cfg_enable             t0
#define t_802_1ag_cfg_standard           t1

// Ingress 1: 
// Outer IP with UDP/TCP parsing and IPSEC NAT-T detection
// if NAT-T: Update the Stard Offset for next stage
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
// Global memory: copy to local one
.struct struct_paComMaxCount
    .u8  vlanMaxCount
    .u8  ipMaxCount
    .u8  greMaxCount
    .u8  rsvd
.ends

// Ingress 0/3: Part of firewall routine
.struct struct_paIpReassmCfg 
    .u8  numTrafficFlow // Maximum number of IP reassembly traffic flows supported, default = 0, maxmium = 32 
    .u8  destFlowId     // CPPI flow which instructs how the link-buffer queues are used for forwarding packets 
    .u16 destQueue      // Destination host queue where PASS will deliver the packets which require IP reassembly assistance 
.ends

// Only for post-processing
.struct struct_paCmdSetCfg
    .u8  numCmdSets      // Number of command sets supported (32, 64)
    .u8  cmdSetSize      // maxmium size of each command set
    .u16 rsvd
.ends 

#define PA_USR_STATS_NUM_ENTRIES    512

// Global 
.struct struct_paUsrStatsGlobCfg
    .u16 numCounters    //  Number of user-defined counters, default = 0, maxmium = 512
    .u16 num64bCounters //  Number of user-defined 64-bit counters, default = 0, maxmium = 256
.ends 

// Ingress 4 only
.struct struct_paQueueDivertCfg
    .u16 destQueue     //   Destination queue where PASS will deliver the LUT2 response packet which contains the
                       //   queue diversion information
    .u8  destFlowId    //   CPPI flow which instructs how the link-buffer queues are used for forwarding
                       //   the LUT2 response packets
    .u8  rsvd                                            
.ends       

// Global
.struct struct_paPktCtrlCfg
    .u16 ctrlBitMap     //   Control Bit Map 
    .u16 validBitMap    //   Valid Bit Map
    .u16 rxPaddingErrCntIndex  // Specify the user statistics index of Rx padding error counter
    .u16 txPaddingCntIndex     // Specify the user statistics index of Tx MAC padding counter
    .u8  egressDefPri          // global default priority for untagged non IP egress traffic for enhanced QoS mode
    .u8  rsvd1
    .u16 rsvd2
.ends

// Global
.struct struct_paMacPaddingCfg
    .u16 rxPaddingErrCntIndex  // Specify the user statistics index of Rx MAC padding error counter
    .u16 txPaddingCntIndex     // Specify the user statistics index of Tx MAC padding counter
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
#define PA_SUB_VALID_MAILBOX_L2_MASK    0x00a5
// ipheader chk, ipfrag to eroute, l3toInner
#define PA_SUB_VALID_MAILBOX_L3_MASK    0x001a
// egress packet capture, eqos mode
#define PA_SUB_VALID_MAILBOX_TX_MASK    0x0140

// The same as Packet Routing info
//.struct struct_paForward
//    .u8   forwardType
//    .u8   flowId
//    .u16  queue
//.ends

// ACL Configuration 
.struct struct_paAclCfg
    .u8  action;        // Drop/Forward/Host   
    .u8  destFlowId;    // CPPI flow which instructs how the link-buffer queues are used for forwarding 
    .u16 destQueue;     // Destination host queue  
.ends

#define PA_ACL_ACTION_FORWARD   PA_FORWARD_TYPE_PA
#define PA_ACL_ACTION_DROP      PA_FORWARD_TYPE_DISCARD
#define PA_ACL_ACTION_HOST      PA_FORWARD_TYPE_HOST
#define t_pa_acl_rescore_list_gen    t15
#define t_pa_acl_rescore_pend_idx    t14
#define PA_ACL_RESCORE_OFFSET_MASK   0x03FF


// ACL rescore operation header
.struct struct_paAclRescoreHdr
     .u16 finalOffset
     .u16 offset
.ends

// ACL rescore operation data
.struct struct_paAclRescoreData
       .u16 index
       .u16 score
.ends

// RA Control Configuration 
.struct struct_paRaCfg
    .u8  ctrlBitMap;   // RA control bits 
    .u8  flowId;       // Input flow Id  
    .u16 rsvd2;        // alignment     
.ends

#define PA_RA_CTRL_EN             (1 << 0)     // RA is enabled 
#define PA_RA_CTRL_USE_LOC_DMA    (1 << 1)     // Input traffic uses local DMA 
#define PA_RA_CTRL_TO_QUEUE       (1 << 2)     // Output packets are delivered to host queue              

#define t_pa_ra_ctrl_en           t0
#define t_pa_ra_ctrl_use_loc_dma  t1
#define t_pa_ra_ctrl_to_queue     t2

.struct struct_paQueueBounceCfg
    .u16 ddrQueue      //   Bounce queue where PASS will deliver the host-routed packet with DDR bit set
    .u16 msmcQueue     //   Bounce queue where PASS will deliver the host-routed packet with MSMC bit set
.ends

//  Configure PA. These configurations apply globally (to all PDSPs). Any
//  PDSP can do the configuration
.struct  struct_paCommandConfig
    .u16  validFlag
    .u16  rsvd2
    
    // Followed by:
    //  Configure max counts
    //  Configure outer IP Reassembly
    //  Configure inner IP Reassembly
    //  Configure command set
    //  Configure User-defined Statistics
    //  Configure Queue Diversion
    //  Configure Packet Verification 
    //  Configure outer IP ACL
    //  Configure inner IP ACL
    //  Configure outer IP RA 
    //  Configure inner IP RA
    //  Configure Queue Bounce
    //  Configure EOAM
.ends

#define t_paCmdConfigValidMaxCount      t0
#define t_paCmdConfigValidOutIpReassem  t1
#define t_paCmdConfigValidInIpReassem   t2
#define t_paCmdConfigValidCmdSet        t3
#define t_paCmdConfigValidUsrStats      t4
#define t_paCmdConfigValidQueueDivert   t5
#define t_paCmdConfigValidPktCtrl       t6
#define t_paCmdConfigValidOutAcl        t7
#define t_paCmdConfigValidInAcl         t8
#define t_paCmdConfigValidOutRa         t9
#define t_paCmdConfigValidInRa          t10
#define t_paCmdConfigValidQueueBounce   t11
#define t_paCmdConfigValidEoam          t12

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
//                                   struct_paAclCfg:          1
//                                   struct_paAclCfg:          1
//                                   struct_paRaCfg:           1
//                                   struct_paRaCfg:           1
//                                   struct_paQueueBounceCfg:  1
//                                   struct_eoamCfg            8
// 
//                                               Total:       27   words = 108 bytes
#define PA_CONFIG_COMMAND_SIZE_CONFIG_PA              (108 - 16)
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
#define PA_SYSTEM_CONFIG_CODE_TIMESTAMP     10

// Command sizes. The assembler can't do all the SIZE(x)+SIZE(y)+... because of line size limitations
// Command size in 32 bit words is:  struct_paCommand:         4
//                                   struct_paSystemConfig:    1
//                                   struct_paComEroute:       1
//                EROUTE_N_MAX(30) * struct_paFwdPlace(4):   120
//             EF_EROUTE_N_MAX(11) * struct_paFwdPlace(4):    44
//                                   struct_paC1Custom:       10
//                                   struct_paC2Custom:        4
//                                   struct_pa802p1agDet:      1
//                                   struct_paIpsecNatTDet:    1  
//                                   struct_pcap_info:         1 
//                                   struct_paEoamTSOffset     2
//                                   struct_paEoamProtoExcl    5
//                MAX_PORTS (9)    * struct_pcap_cfg:          3(27)
//                                   struct_paComDroute:       1
//                MAX_PORTS (8)    * struct_paComDrouteCfg:    1(8)
// MAX_PORTS (8)* DROUTE_N_MAX (3) * struct_paFwdPlace(4):    12(96)
//                                   struct_paComEQoS:         1
//                                   struct_paComIfEQoS:       2(16)
//                        struct_paComIfEQoSRouteOffset:       4(32)  (vlanTbl)
//                      8*struct_paComIfEQoSRouteOffset:      32(256)  (dscpTbl)
// 
//                                               EROUTE:       126 words = 504 bytes
//                                               EROUTE:        50 words = 200 bytes
//                                               CUSTOM LUT1:   15 words = 60 bytes
//                                               CUSTOM LUT2:    9 words = 36 bytes
//                                               802.1ag Detect: 6 words = 24 bytes 
//                                               NAT-T Detect:   6 words = 24 bytes
//                                               GTPU Config:    6 words = 24 bytes
//                                               PCap Config:    9 words = 36 bytes  (minimum)
//                                               PCap Config:   33 words = 132 bytes (maximum)
//                                               DefRoute Cfg:  19 words =  76 bytes (minimum)
//                                               DefRoute Cfg: 110 words = 440 bytes (maximum)
//                                               EQOS Cfg:      44 words = 176 bytes (minimum)
//                                               EQOS Cfg:     310 words =1240 bytes (maximum)
//                                               EOAM Time Offset: 7 words = 28 bytes
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EROUTE         504 - 16 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EF_EROUTE      200 - 16 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT1     60 - 16 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_CUSTOM_LUT2     36 - 16 
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_802_1AG         24 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_IPSEC_NAT_T     24 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_GTPU            24 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_PCAP            36 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_DEF_ROUTE       76 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EQOS           176 - 16
#define PA_CONFIG_COMMAND_SIZE_SYS_CONFIG_EOAM_TOFFSET    28 - 16

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

.struct  struct_paCmdHdr
    .u8  command                
    .u8  offset             // Offset to the next command
    .u16 cmdId              // used by LLD only
.ends

#define PA_CFG_CMD_STATUS_PROC      0
#define PA_CFG_CMD_STATUS_DONE      1

// Command header
.struct struct_paCommand
    .u8   status                // Command Status
    .u8   pdspIndex             // index of the first targeted PDSP in a clsuter
    .u16  commandResult
    .u16  offset                // Offset to the next command to be processed  or comdId for none multi-command
    .u8   command
    .u8   magic
    
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
#define PA_CONFIG_COMMAND_MULTIPLE_CMD    100 

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

// LUT2 is full
#define PA_COMMAND_RESULT_LUT2_FULL                    21

// Compute Checksum/CRC command
.struct  struct_cmdChkCrc
    .u8  cmdId
    .u8  flags         // bits 0-3 are the cde crc flags. Bit 7 is the checksum negative 0 bit
    .u8  crcSize
    .u8  startOffset
    .u16 length
    .u16 resultOffset
    .u16 initVal   // MSB of 32-bit CRC init value or 16-bit checksum init value
    .u16 initVal2  // LSB of 32-bit CRC init value
.ends

#define t_paFlagsChksumNeg0    t7
#define t_paFlagCrcOffsetValid t0
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
#define t_nextRoute_ctrl_txTimestamp t2


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

// Patch EOAM time command
.struct struct_cmdInsEoamTime
    .u8  cmdId_subCode    
    .u8  rsvd
    .u16 offset
.ends

// Patch EOAM count command
.struct struct_cmdInsEoamCount
    .u8  cmdId_subCode    
    .u8  rsvd1
    .u16 offset
    .u16 index
    .u16 rsvd2
.ends

// Patch Message Length command
.struct  struct_patchMsgLenCmd
    .u8  cmdId_subCode
    .u8  offset               //offset to message length
    .u16 msgLen               //b15 (message length field size flag): 0: 2-byte;1:4-byte
.ends    

// Insert CRC command note: This command is generated by PASS internally
// We need to use the command to pass CRC information since the CRC will be gnerated by checker between two PDSPs
// All the routing information at both packet descriptor and extended packet descriptor should be ready and should not be altered
.struct  struct_patchCrcCmd
    .u8  cmdId_subCode
    .u8  crcSize              // CRC size in bytes
    .u16 offset               // Offset to CRC location
.ends    


#define t_patch_msg_len_size32          t15

#define PA_SUB_CMD_CODE_MASK            0x1F
#define PA_SUB_CMD_CODE_DUMMY           0
#define PA_SUB_CMD_CODE_IP_FRAG         1
#define PA_SUB_CMD_CODE_PATCH_MSG_LEN   2
#define PA_SUB_CMD_CODE_PATCH_MSG_TIME  3
#define PA_SUB_CMD_CODE_PATCH_MSG_CNT   4
#define PA_SUB_CMD_CODE_PATCH_CRC       0x1f

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
    .u16  startOffset
    .u16  len
    .u8   crcSize
    .u8   lenOffset
    .u16  lenMask
    .u16  crcOffset
    .u32  initVal
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



// Egress Flow related definitions
#define PA_EF_REC_TYPE_LVL1          1
#define PA_EF_REC_TYPE_LVL2          2
#define PA_EF_REC_TYPE_LVL3          3
#define PA_EF_REC_TYPE_LVL4          4

#define PA_EF_MAX_REC_INDEX          255

// Egress Flow record 1
.struct  struct_efRecord1
    .u16 ctrlFlags
    .u16 mtu                // MTU size of inner IP
    .u8  tos                // IPv4: TOS; Ipv6: Class
    .u8  flowLablelHi       // upper 4-bit of IPV6 Flow Label
    .u16 flowLableLo        // lower 16-bit of IPV6 Flow Label
    .u16 srcPort            // TCP/UDP source port
    .u16 dstPort            // TCP/UDP destination port
.ends

#define PA_EF_REC1_SRC_IP_OFFSET    16
#define PA_EF_REC1_DST_IP_OFFSET    32
#define PA_EF_REC1_SIZE             48

#define t_ef_rec1_ip_src_addr          t0
#define t_ef_rec1_ip_dst_addr          t1
#define t_ef_rec1_ip_flow_label        t2
#define t_ef_rec1_ip_tos_class         t3
#define t_ef_rec1_ip_mtu               t4    // Inner IP fragmentation
#define t_ef_rec1_l4_src_port          t5
#define t_ef_rec1_l4_dst_port          t6
#define t_ef_rec1_ipv4_cksum           t7
#define t_ef_rec1_l4_cksum             t8
#define t_ef_rec1_ip_ttl_update        t9
#define t_ef_rec1_strip_outer_ip       t10
#define t_ef_rec1_exp_tcp_ctrl         t11
#define t_ef_rec1_exp_ip_options       t12
#define t_ef_rec1_exp_ip_frag          t13
#define t_ef_rec1_exp_ip_expire        t14
#define t_ef_rec1_valid                t15

// Egress Flow record 2
.struct  struct_efRecord2
    .u16 ctrlFlags
    .u16 mtu                // MTU size of inner IP
    .u8  l3HdrSize          // L3 Header (Outer IP) size in bytes
    .u8  encBlkSize         // Encryption block size: 1, 4 and 16
    .u8  ivSize             // Initialization vector size
    .u8  icvSize            // Authentication tag size in bytes   
    .u8  rsvd 
    .u8  flowId             // CPPI flow Id or destination thread Id
    .u16 queueId            // Destination queue Id
.ends

#define PA_EF_REC2_SPI_OFFSET       12
#define PA_EF_REC2_SWINFO0_OFFSET   16
#define PA_EF_REC2_SWINFO1_OFFSET   20
#define PA_EF_REC2_L3_HDR_OFFSET    24
#define PA_EF_REC2_SIZE             64

#define t_ef_rec2_strip_outer_ip       t0     // not used
#define t_ef_rec2_single_ip            t1
#define t_ef_rec2_ipsec_proc           t2
#define t_ef_rec2_ipsec_ah             t3
#define t_ef_rec2_ins_ipsec            t4    // Insert IPSEC ESP header and IPSEC AH header
#define t_ef_rec2_ins_ipsec_trail      t5
#define t_ef_rec2_ip_ttl_update        t6
#define t_ef_rec2_ip_mtu               t7    // Outer IP fragmentation
#define t_ef_rec2_loc_dma              t8    // Use local DMA
#define t_ef_rec2_exp_ip_options       t12
#define t_ef_rec2_exp_ip_frag          t13
#define t_ef_rec2_exp_ip_expire        t14
#define t_ef_rec2_valid                t15

// Egress Flow record 3
.struct  struct_efRecord3Nat
    .u16 ctrlFlags
    .u16 mtu                // MTU size of inner IP
    .u16 srcPort            // UDP source port
    .u16 dstPort            // UDP destination port
.ends

.struct  struct_efRecord3Ah
    .u16 ctrlFlags
    .u16 mtu                // MTU size of inner IP
    .u16 rsvd1 
    .u8  ivSize             // Initialization vector size
    .u8  icvSize            // Authentication tag size in bytes   
    .u8  rsvd2
    .u8  flowId             // CPPI flow Id or destination thread Id
    .u16 queueId            // Destination queue Id
.ends

#define PA_EF_REC3_SPI_OFFSET       12
#define PA_EF_REC3_SWINFO0_OFFSET   16
#define PA_EF_REC3_SWINFO1_OFFSET   20
#define PA_EF_REC3_SIZE             32 

#define t_ef_rec3_ipsec_ah             t0    // 1: IPSEC AH, 0: IPSEC ESP NAT-T
#define t_ef_rec3_replace_hdr          t1    // 1: replace AH or NAT-T header
                                             // 0: Insert AH or NAT-T header 
#define t_ef_rec3_ip_mtu               t2    // Outer IP fragmentation
#define t_ef_rec3_loc_dma              t3    // Use local DMA
#define t_ef_rec3_valid                t15   // This entry is valid
                            
// Egress Flow record 4
.struct  struct_efRecord4
    .u16 ctrlFlags
    .u8  l2HdrSize          // L2 Header size in bytes
    .u8  l2LenOffset        // Offset to L2 (802.3) length field
    .u8  pppoEOffset        // Offset to PPPoE header
    .u8  flowId             // CPPI flow Id or destination thread Id
    .u16 queueId            // Destination queue Id
    .u8  minPktSize         // Minimum L2 packe size
    .u8  pktType_psFlags    // SRIO:packet Type; EMAC:psFlags
    .u8  destType           // Destination type: Host, EMAC or SRIO
    .u8  rsvd 
.ends

#define PA_EF_REC4_VLAN1_OFFSET     12
#define PA_EF_REC4_VLAN2_OFFSET     14

#define PA_EF_REC4_SRIO_HDR_OFFSET  16
#define PA_EF_REC4_SWINFO0_OFFSET   16
#define PA_EF_REC4_SWINFO1_OFFSET   20
#define PA_EF_REC4_L2_HDR_OFFSET    24
#define PA_EF_REC4_SIZE             64

#define t_ef_rec4_strip_l2_hdr         t0
#define t_ef_rec4_valid_802_3_len      t1
#define t_ef_rec4_valid_pppoe          t2
#define t_ef_rec4_valid_vlan1          t3
#define t_ef_rec4_valid_vlan2          t4    
#define t_ef_rec4_tx_padding           t5  
#define t_ef_rec4_pri_dscp             t6
#define t_ef_rec4_pri_vlan             t7
#define t_ef_rec4_valid                t15    


// Ethernet OAM structures (common config between Ingress0 and Egress0 )
.struct struct_eoamCfg
      .u8   ctrlBitmap
      .u8   rsvd1
      .u16  rsvd2

      .u32  nsRoAcc           // nano second accumulation value over one roll over (accumulates error also)
      .u32  nsNumRoAcc        // very precise nano second accumulation value over N roll overs, to reset the accumulation error
      .u16  numRo             // number of roll overs to consider for resetting error accumulation
      .u16  mul               // multiplication factor, keeping the shift right factor of 13
      
      .u16  proto0            // first proto ID that need no statistics      
      .u16  proto1

      .u16  proto2      
      .u16  proto3

      .u16  proto4            
      .u16  proto5

      .u16  proto6      
      .u16  proto7
.ends

#define t_eoam_ctrl_enable t0

// Local structure in Ingress 0
.struct struct_timeAccConstants
  .u32  nsRoAcc           // nano second accumulation value over one roll over (accumulates error also)
  .u32  nsNumRoAcc        // very precise nano second accumulation value over N roll overs, to reset the accumulation error
  .u16  numRo             // number of roll overs to consider for resetting error accumulation
  .u16  rollOverCnt       // roll over Count (not a constant, running count value)
.ends

// Local structure in Ingress 0
.struct struct_eoamExceptionTbl
      .u16  proto0            // first proto ID that need no statistics      
      .u16  proto1
      .u16  proto2      
      .u16  proto3
      .u16  proto4
      .u16  proto5
      .u16  proto6
      .u16  proto7 
.ends

// Configuration structure for Egress 0 on time offsets
.struct struct_timeStampOffsetCfg
      .u32  offset_sec        // time stamp offset in seconds
      .u32  offset_ns         // time stamp offset in nano seconds 
.ends



// Below are constants useful in converting the ticks to nano seconds (local to Egress 0)
.struct struct_timeConvConst
      .u32  offset_sec        // time stamp offset in seconds
      .u32  offset_ns         // time stamp offset in nano seconds  
      .u16  mul               // multiplication factor, keeping the shift right factor of 13    
      .u16  rsvd      
.ends 

// Below are counters to accumulate time per roll over in seconds and nano seconds (common between Ingress 0 and Egress 0)
.struct struct_timeAcc
      .u32  nsAccWithErr      // Counter to accumulate nano seconds per N roll overs
      .u32  nsAccNoErr        // Counter to accumulate nano seconds per K * N * roll over
      .u32  secAcc            // Counter to accumulate seconds
.ends

#endif // _PDSP_PA_H
