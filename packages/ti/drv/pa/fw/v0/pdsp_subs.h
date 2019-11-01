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

#ifndef _PDSP_SUBS_H
#define _PDSP_SUBS_H    1

// ************************************************************************************************
// * FILE PURPOSE: Define structures local to the PA subsystem
// ************************************************************************************************
// * FILE NAME: pdsp_subs.h
// *
// * DESCRIPTION:  Non API related definitions
// *
// ************************************************************************************************  

//
// Mailbox Command and Command Response
// The four mailbox registers (0, 4, 8 and 12) correspond to the four command status bits 
// (tStatus_Commandx) respectively
// /--------------------------------------------------------\
// | 31    24   | 23                                     0 |
// |  Command   |     command specific data                |
// \--------------------------------------------------------/

// PDSP A can issue a simple command to PDSP B by placing an non-zero command code to its
// mailbox register; 
// PDSP B should process the command and then clear to status bit by replacing the command code
// with zero when the corresponding command status bit is set
//
// Refer to pdsp_mem.h
// #define FIRMWARE_P0_MBOX                0x0000      // c4
// #define FIRMWARE_P1_MBOX                0x0010      // c4
// #define FIRMWARE_P2_MBOX                0x0020      // c4
// #define FIRMWARE_P3_MBOX                0x0030      // c4
// #define FIRMWARE_P4_MBOX                0x0040      // c4
// #define FIRMWARE_P5_MBOX                0x0050      // c4

//
//  PDSPx informs the corresponding PDSP that the host has sent an new IP Reassembly configuration
//        where PDSP1 for outer IP reassembly and PDSP2 for inner IP Reassembly
// 
#define  FIRMWARE_CMD_IP_REASSEM_CFG            1         
#define  FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET     4       // command offset  (1st command)

//
//  PDSPx informs the corresponding PDSP that the host has send another packet control 
//        configuration:
//        PDSP0 for PPPoE header verfication, 802.1ag detector, ingress packet capture, ingress default route
//        PDSP1/2 for IP header verfication, IP fragments routing
//        PDSP5   for egress packet capture 
//
//  b3: command
//  b0: control flags 
#define  FIRMWARE_CMD_PKT_CTRL_CFG            1         
#define  FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET     12       // command offset  (3rd command)

// Packet source
//#define PAS_PKT_SRC_RIO         4  // Need to get the real value! (not yet in spec)
//#define PAS_PKT_SRC_MAC         5  // Also need the real value

// Action values
#define SUBS_ACTION_PARSE       0
#define SUBS_ACTION_LOOKUP      1
#define SUBS_ACTION_EXIT        2

// The following value is not used at the dispatch table
#define SUBS_ACTION_FWPKT       3    // Forward the packet out
#define SUBS_ACTION_DISCARD     4    // Discrad the packet
#define SUBS_ACTION_FWPKT2      5    // Forward the BC/MC packet out


// PSInfo Command Header
.struct  struct_cmdHeader
    .u16  cmdOffset             // Offset to a command overview
    .u16  linkedCmdOffset         
.ends



// The control data 1st byte identifies the packet format
//  /-----------------------------------\
//  | 7       5  |   4               0  |
//  |  Cmd Id    |                      |
//  \-----------------------------------/
#define  SUBS_CMD_WORD_ID_SHIFT            5
#define  SUBS_CMD_WORD_ID_MASK_SHIFTED     0x7


// Command values 
#define PSH_CMD_PA_RX_PARSE        0   // Classify 1 and 2
#define PSH_CMD_PA_TX_CHKSUM       0   // Modifier
#define PSH_CMD_SA_LONG_INFO       0   // SA only

#define PSH_CMD_PA_TX_CRC          1   // Modifier
#define PSH_CMD_SA_SHORT_INFO      1   // SA only

#define PSH_CMD_PA_TX_BLIND_PATCH  2   // Modifier

#define PSH_CMD_PA_TX_NEXT_ROUTE   3   // Modifier

#define PSH_CMD_CONFIG             4   // All

#define PSH_CMD_RX_FWD             5   // Modifier

#define PSH_CMD_PA_TX_RPT_TS       6   // Modifier: Report Timestamp   

#define PSH_CMD_PA_GROUP_7         7   // Modifier: Group 7 commands: specify the sub-command code
#define PSH_CMD_PA_DUMMY           7   // Modifier: Dummy commad which is used for alignment puepose only
#define PSH_CMD_PA_IP_FRAG         7   // Modifier: Ip Fragmenation 


#define PSH_CMD_DEST_PA     1
#define PSH_CMD_DEST_SA     2
#define PSH_CMD_DEST_HOST   3
#define PSH_CMD_DEST_ALL    4


// Packet Context
.struct  struct_pasPktContext

     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  | vLnkEnable | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l5Offset
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u16  hdrBitmask_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    .u16 pseudo
    .u8  vLinkNum
    .u8  dscpPriority
    .u8  vlanPriority
    .u8  flag2
    .u16 scratch
.ends

//flag 
//Broadcast, Multicast indication flags
#define t_flag_mac_broadcast    t0
#define t_flag_mac_multicast    t1
#define t_flag_ip_broadcast     t2
#define t_flag_ip_multicast     t3

#define PA_BM_FLAG_MASK         0x0F


// IP Reassembly flags
#define t_flag_2nd_pass         t4  // To be clear at second pass
#define t_flag_null_pkt         t5  // This flag will be set by host to indicate that the packet should be dropped
                                    // It is used to clear the traffic flow when the reassembly timeout occur
                                    // It can be shared with other flags since the packet will be dropped when 
                                    // this flag is set  

// PASS-SASS intercommunication flags  
#define t_flag_gtpu_seqnum_present  t4  

// PAM command control flags
#define t_flag_split            t5     // set by Payload split command, execute at PDSP 5; 
#define t_flag_crc_verify       t5     // set by SCTP of PDSP1/2, execute at PDSP 4
#define t_flag_cmdset           t6
#define t_flag_multi_route      t7

//flag2
// more control flag definitions
#define t_flag2_cascaded_forwarding     t0   // Indicate that packets should be forwarded to next processor without any changes
                                             // Disable some global actions such as IP assembly-assisted operation and/or
                                             // fragments exception route

//eP1C2IdIdx
#define t_pl1Match  t10
#define t_vlinkEn   t9
#define PKT_EIDX_MASK       0xF800
#define PKT_EIDX_SHIFT      11
#define SUBS_PKT_CXT_L1ID_BIT_OFFSET         6
#define SUBS_PKT_CXT_L1ID_SHIFTED_MASK       0x7
#define L1IDX_MASK                           0x3F

// hdrBitmask: A bit index of headers found
#define  SUBS_PA_BIT_HEADER_MAC        t5
#define  SUBS_PA_BIT_HEADER_VLAN       t6
#define  SUBS_PA_BIT_HEADER_MPLS       t7
#define  SUBS_PA_BIT_HEADER_IP         t8
#define  SUBS_PA_BIT_HEADER_ESP        t9
#define  SUBS_PA_BIT_HEADER_AUTH       t10
#define  SUBS_PA_BIT_HEADER_UDP        t11
#define  SUBS_PA_BIT_HEADER_UDP_LITE   t12
#define  SUBS_PA_BIT_HEADER_TCP        t13
#define  SUBS_PA_BIT_HEADER_GTPU       t14
#define  SUBS_PA_BIT_HEADER_CUSTOM     t15

// Protocol Counters
#define PROT_COUNT_IP_MASK                   0x07
#define PROT_COUNT_IP_SHIFT                  0
#define PROT_COUNT_IP_STEP                   (1 << PROT_COUNT_IP_SHIFT)
#define PROT_COUNT_GRE_MASK                  0x07
#define PROT_COUNT_GRE_SHIFT                 3
#define PROT_COUNT_GRE_STEP                  (1 << PROT_COUNT_GRE_SHIFT)
#define PROT_COUNT_VLAN_MASK                 0x03
#define PROT_COUNT_VLAN_SHIFT                6
#define PROT_COUNT_VLAN_STEP                 (1 << PROT_COUNT_VLAN_SHIFT)

//hdrBitmask3_frag_portNum
#define t_ipFrag    t3
#define PKT_EMACPORT_MASK   0x07
// hdrBitmask3: A bit index of headers found
#define  SUBS_PA_BIT_HEADER_SCTP        t4
#define  SUBS_PA_BIT_HEADER_IPSEC_NAT_T t5
#define  SUBS_PA_BIT_HEADER_PPPoE       t6
#define  SUBS_PA_BIT_HEADER_802_3       t7

//hdrBitmask_nextHdr
#define PKT_NEXTHDR_MASK        0x1F
#define NOT_PKT_NEXTHDR_MASK    0xE0

//paCmdId_Length
#define PKT_CONTEXT_CMD_MASK      0xe0
#define NOT_PKT_CONTEXT_CMD_MASK  0x1F
#define PKT_CONTEXT_CMD_SHIFT 5 


// This is a repeat of pasPktContext, but with the error index, previous match, custom c2, id and
// index fields broken into two bytes. This makes access to the error index possible through a 
// constant in commands. But the 16 bit version is required to for the PDSP ID field
// Packet Context
.struct  struct_pasPktContext2
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  |  reserved  | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u8  eP1C2Id
    .u8  IdIdx
//    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l5Offset
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u8   hdrBitmask1
    .u8   hdrBitmask2_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    .u16 pseudo
    .u8  vLinkNum
    .u8  dscpPriority
    .u8  vlanPriority
    .u8  flag2
    .u16 scratch
.ends

#define PKT2_EIDX_MASK      0xF8
#define NOT_PKT2_EIDX_MASK  0x07
#define PKT2_EIDX_SHIFT     3

// This is a repeat of pasPktContext for configuration command  
// only the latest 32-bit word is used (R26)
.struct  struct_pasPktContext3
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  |  reserved  | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u8  eP1C2Id
    .u8  IdIdx
//    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l5Offset
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u8   hdrBitmask1
    .u8   hdrBitmask2_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    .u8   ctrlFlag
    .u8   cmdResultOffset
    .u8   vLinkNum
    .u8   dscpPriority
    .u8   vlanPriority
    .u8   flag2
    .u16  scratch
.ends

#define t_cmdResultPatch        t0  

// This is a repeat of pasPktContext for IP Reassembly support  
// only the latest 32-bit word is used (R26)
.struct  struct_pasPktContext4
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  |  reserved  | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u8  eP1C2Id
    .u8  IdIdx
//    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l5Offset
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u8   hdrBitmask1
    .u8   hdrBitmask2_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    // re-use the pseudo field since the field is not used at the first pass
    .u8  tfIndex    // Traffic Flow Index
    .u8  fragCnt    // Number of fragments in the packet
    .u8  vLinkNum
    .u8  dscpPriority
    .u8  vlanPriority
    .u8  flag2
    .u16 scratch
.ends

// This is a repeat of pasPktContext for command set command  
// only the last 32-bit word is changed
//  re-use the pseudo field since the field is no longer used when the packet is forwarded out
.struct  struct_pasPktContext5
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  |  reserved  | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u8  eP1C2Id
    .u8  IdIdx
//    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l5Offset
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u8   hdrBitmask1
    .u8   hdrBitmask2_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    .u8   cmdSetIdx                     // command set index                 
    .u8   rsvd1
    .u8   vLinkNum
    .u8   dscpPriority
    .u8   vlanPriority
    .u8   flag2
    .u16  scratch
.ends

// This is a repeat of pasPktContext for l3offset2 (inner most IP offset  
// only the last 32-bit word is changed
//  re-use the pseudo field since the field is no longer used after UDP/TCP checksum or when packet is
//  forwarded to the host
.struct  struct_pasPktContext6
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u8   flag
    
    .u16  startOffset
    .u16  endOffset
    
    
    // /--------------------------------------------------------\
    // | 15    11 |   10      |     9      | 8   6 |  5       0 |
    // |   eIdx   | pl1Match  |  reserved  | l1Id  |  l1Index   |
    // \--------------------------------------------------------/
    .u8  eP1C2Id
    .u8  IdIdx
//    .u16  eP1C2IdIdx
    
    .u8  l3Offset
    .u8  l4Offset
    .u8  l3l5Offset     // Offset to the inner IP loaction (overlap with l5offset)
                        // it is valid until the TCP/UDP processing and thyen the offset information will
                        // be passed to l3offset2
    .u8  espAhOffset
    
    //  /-----------------------------\
    //  | 15           5 | 4        0 |
    //  |  hdrBitmask    |  nextHdr   |
    //  \-----------------------------/
    .u8   hdrBitmask1
    .u8   hdrBitmask2_nextHdr
    
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP
    
    //  /------------------------------------------\
    //  | 7        4  |   3   |    2          0  |
    //  | hdrBitmask3 |  frag |  EMAC port number|
    //  \------------------------------------------/
    .u8  hdrBitmask3_frag_portNum
    
    .u8   cmdSetIdx                     // command set index                 
    .u8   l3offset2                     // inner most ip offset  
    .u8   vLinkNum
    .u8   dscpPriority
    .u8   vlanPriority
    .u8   flag2
    .u16  scratch
.ends


// next header is stored temporarily during parse as an 8 bit value
.struct  struct_next
  .u8    Hdr 
.ends

// the action to take is always stored in the function return space
.struct  struct_param
  .u16   action
.ends

// Classify1 run time context
.struct  struct_c1RunContext
    // /--------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5        |       4      |       3      |       2        |   1   0     |   
    // |  active lookup | pending submit | pending config | fake Lookup  | IP Reassem   | SCTP Checksum  |  PDSP ID    |
    // \--------------------------------------------------------------------------------------------------------------/
    .u8    flags
    
    // /-------------------------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5           |       4          |       3         |   2       |     1     |  --    0    |
    // |  Hdr check     | ipFragToEroute | l3offset:inner IP | 802.1ag standard |  802.1ag Detect |  ingress  |           |             |
    // |                |                |                   |                  |                 | def route |   pcap En | fail lookup |
    // \-------------------------------------------------------------------------------------------------------------------------------/
    .u8    flag2 
    .u8    rsvd
    .u8    keyFlags
.ends

#define   t_activeLookup          t7
#define   t_pendingSubmit         t6
#define   t_pendingConfig         t5
#define   t_fakeLookup            t4
#define   t_ipReassmEn            t3    // 1: IP reassembly assistance is enabled
#define   t_sctpChksum            t2    // 1: SCTP checksum is enabled 
#define   SUBS_RUN_CXT_ID_MASK    0x03

#define   t_hdrCheck              t7    // 1: Enhanced header check is enabled    (PDSP0-2)
#define   t_ipHdrCheck            t7    // 1: Enhanced IP header check is enabled (PDSP1/2)
#define   t_PPPoEHdrCheck         t7    // 1: PPPoE haeder check is enabled  (PDSP0)
#define   t_macPaddingChk         t6    // 1: MAC (802.3 only) padding check (PDSP0)
#define   t_ipFragToEroute        t6    // 1: All IPv4 fragments should be delivered through exception route (PDSP1/2)
#define   t_l3toInnerIp           t5    // 1: L3offset points to inner IP
#define   t_802_1agStd            t4    // 1: 802.1ag standard mode (Destination MAC=01-80-C2-00-00-3X) 
#define   t_802_1agDet            t3    // 1: 802.1ag Detector is enabled
#define   t_def_route             t2    // 1: Ingress default route (PDSP0)
#define   t_ingress_pCapEnable    t1    // 1: packet capture feature enable (ingress) (PDSP0)
#define   t_failLookup            t0    // 1: LUT1 lookup is determined to fail due to unknown (unsupported) protocol

// #define   SUBS_PKT_CTRL_HDR_CHK            0x80
// #define   SUBS_PKT_CTRL_IP_FRAG_TO_EROUTE  0x40
// #define   SUBS_PKT_CTRL_802_1agStd         0x20
// #define   SUBS_PKT_CTRL_802_1agDet         0x10
// #define   SUBS_PKT_CTRL_MASK               0xe2
// #define   NOT_SUBS_PKT_CTRL_MASK           0x1d

// LUT1 table entries
.struct  struct_l1Info
    .u8  ctrlFlag         // Boolean - enables next classification info (SRIO) only
    .u8  nextHdr
    .u16 nextHdrOffset
    
    // Followed by 2 struct_paForward (each 12 bytes):
    //  forwardMatch
    //  prevMatch
.ends

.struct struct_l1InfoVlink
    .u8  ctrlFlag
    .u8  vLinkNum
    .u16 rsvd
.ends

#define   custom_enable     t0
#define   vlink_enable      t1

// Classify2 run context
.struct  struct_c2RunContext
    .u16  c2NumInL2Table
    .u8   pdspId
    .u8   ctrlFlag
.ends

// 
// GTPU is enbaled by default
// GTPU parsing should be disabled when GTPU port number (2152) is added into LUT2 table
// GTPU parsing will be re-enabled when the GTPU port entry is deleted from the LUT2 table
//
#define t_c2_disable_GTPU       t0


// 
// GTPU use Link is disabled by default
// GTPU use link can be enabled/(disabled) through global configuration
//
#define t_c2_GTPU_use_link      t1


// 
// IPSEC NAT-T is disabled by default
// IPSEC NAT-T parsing can be enabled/disabled through global configuration with configurable UDP port number
//
#define t_c2_enable_IPSEC_NAT_T t2

// 
// GTPU End Marker message route as G-PDU
//
#define t_c2_GTPU_route_msg254_as_msg255    t3


#define SUBS_MAX_LUT2_ENTRIES  8192

// Multi routing tables
.struct  struct_subsMultiRoute

    .u8  nRoutes
    .u8  rsvd
    .u16 rsvd2

    // Followed by PA_MAX_HOST_PKT_DUP structures of type struct_paSingleRoute (4 bytes each)
.ends

// Supported ethertypes
.struct struct_ethertypes
    .u16  vlan
    .u16  spVlan
    .u16  ip
    .u16  ipv6
    .u16  mpls
    .u16  mplsMulti
    .u16  PPPoE
    .u16  PPPoE_discov
.ends

// The function call table. The ordering of the table must
// precisely match the header number ordering in pdsp_pa.h (PA_HDR_xxx)
.struct struct_headerParse
    .u16    c1ParseMac
    .u16    c1ParseVlan
    .u16    c1ParseMpls
    .u16    c1ParseIpv4
    .u16    c1ParseIpv6
    .u16    c1ParseIpv6ExtHop
    .u16    c1ParseIpv6ExtRoute
    .u16    c1ParseIpv6ExtFrag
    .u16    c1ParseIpv6ExtDestOpt
    .u16    c1ParseGre
    .u16    c1ParseEsp
    .u16    c1ParseEspDec
    .u16    c1ParseAuth
    .u16    c1ParseCustom
    .u16    c1ParsePPPoE
    .u16    c1ParseSctp
    .u16    c1ParseUnkn
    .u16    c2ParseUdp
    .u16    c2ParseUdpLite
    .u16    c2ParseTcp
    .u16    c2ParseGtpu
    .u16    c2ParseEspDec
    .u16    c2ParseCustom
    .u16    c2RarseRsvd23
.ends

// LUT1 entry mapping
.struct struct_subsLut1Map
    .u32  validEntries1   // Most significant word (LUT entries 32-63)
    .u32  validEntries0   // Least significant word (LUT entries 0-31)
.ends


// Pending entry LUT1 info
.struct struct_subsLut1AddPendingInfo
    .u8  mode
    .u8  index
    .u16 matchFlags
.ends

#define SUBS_LUT1_PENDING_MODE_DEL_ENTRY  0
#define SUBS_LUT1_PENDING_MODE_ADD_ENTRY  1

// Mailbox assignments
#define  initRun           t0
#define  globalMemInit     t1
#define  tempHalt          t2

.struct struct_regFlags
    .u32  info
.ends

#define FALSE  0
#define TRUE   1

// statistics tracked during packet operation
.struct struct_statsFlags
    .u32  event 
.ends


// Offsets for hardware regs
#define OFFSET_STATS_FLAGS   8  

#define OFFSET_PACKET_ID_SOFT_RESET  0x4
#define OFFSET_PACKET_ID_RANGE       0x8
#define OFFSET_PACKET_ID_IDVAL       0xC

#define PACKET_ID_RANGE_LIMIT        100
#define PACKET_ID_ALLOC_FAIL         0x400

// Event flags that trigger an interrupt
.struct struct_eventFlags
    .u32  enabledEvent
.ends


// LUT2 add/delete control msb values
#define SUBS_L2_ENTRY_CTRL_ADD      0xc0
#define SUBS_L2_ENTRY_CTRL_DEL      0x80


// Modify PDSP checksum context 
.struct struct_modifyContext
    .u8  flags
    .u8  usrStatsPdsp     // User statistics polling start pdsp
    .u8  paddingCnt       // number of fragment or packet padding occurs sofar
    .u8  cmdFlushBytes      // number of bytes to flush during IP Fragment cmd processing
.ends

// flags used by Modify opertaion
#define t_isPdsp5           t0  // 0: PDSP 4 and 1: PDSP 5
#define t_egress_pCapEnable t1  // 1: packet capture feature enable (egress)
#define t_eqos_feature      t2  // 0: EQOS feature disabled, 1: EQOS feature enabled

#define t_MCxtCopyState     t7  // 0: Copy Packet Pending, 1: Copy packet Done

//#define t_eqos_mode         t5  // 1: EQOS Mode is enabled

//#define SUBS_MOD_GLOBAL_PKT_CAP_ENABLE            2
//#define NOT_SUBS_MOD_GLOBAL_PKT_CAP_ENABLE     0xFD


#define SUBS_MOD_CXT_MAX_PATCHES           4
#define SUBS_MOD_CXT_MAX_MSGLEN_PATCHES    2


#define PA_USR_STATS_START_PDSP_MASK        0x3                                        
#define NOT_PA_USR_STATS_START_PDSP_MASK    0xFC                                        

// Additional tracking context for modify (Tx command)
.struct  struct_modifyState
    .u16  statsIndex        // User stats index
    .u16  pktSize           
    .u8   chkSumCount       // Note: cksumCount can be used for msgLen Patch counters since checksum command will not be in conjuction with IP
                            //       fragmentation and message length patching command 
    .u8   patchCount
    .u8   flags             
    .u8   pktType_psFlags   // store packet type (SRIO) or ps flags (ETH) for IP fragment 
.ends

// flags used by Modify opertaion
#define t_subsMCxtCrc        t7
#define t_subsMCxtTerminate  t6
#define t_subsMCxtSrio       t5     // Next Route destination is SRIO
#define t_subsMCxtPsFlags    t4     // Need to update PsFlags
#define t_subsMCxtTs         t3     // Report timestamp
#define t_subsMCxtIpFrag     t2     // IP Fragmentation pending
#define t_subsMCxtPadding    t1     // Layer 2 padding 
#define t_subsMCxtUsrStats   t0     // Update Tx user-defined statistics

// Additional tracking context for modify (Rx command set)
.struct  struct_modifyState2
    .u8   usrDataOffset     // Offset to the user copied data: valid only if t_rx_cmd_ctrl_split is set 
    .u8   flags             
    .u16  pktSize           
    .u8   hdrSize           // store the packet header size 
    .u8   flowId            // valid only if t_rx_cmd_ctrl_split is set
    .u16  destQueue         // valid only if t_rx_cmd_ctrl_split is set
.ends

// flags used by Command set operation
#define  t_rx_cmd_ctrl_no_ctx     t7
#define  t_rx_cmd_ctrl_no_hdr     t6

// Rx Cmd processing Context
.struct struct_rxCmdContext
    .u8  cmdSetIndex   
    .u8  nCmds             // number of commands left to be processed
    .u16 cmdOffset         // offset in the memory table
    .u16 pktOffset         // current processing location of the packet    
    .u8  psInfoSize
    .u8  updateLen         // Record total szie of packet lenhth update may be negative */
.ends     

//
// This structure is used to pass the CRC information from classify1 PDSP to the modify PDSP
// It should be appended to the packet context at the PS Info section (size 32)
// It is only used internally and therefore is not required to be passed to the Host
//

// Context for CRC verification
.struct struct_crcVerifyContext
    .u32    crc
    .u16    offset
    .u16    rsvd
.ends

//
// This structure is used to pass the Payload splitting information from PDSP4 to PDSP5
// It should be appended to the packet context at the PS Info section (size 32)
// It is only used internally and therefore is not required to be passed to the Host
//

// Context for payload Splitting
.struct struct_payloadSplitContext
    .u8     hdrSize  
    .u8     flowId
    .u16    destQueue
.ends

//
// These structures are used for the User-defined statistics FIFO operation
// PDSP0-PDSP3: Insert the statistics update request into its FIFO
// PDSP4 Process the FIFOs for all PDSPs
//

// FIFO control block
.struct struct_fifoCb
    .u8     out          // Out as the first entry to save one cycle and instruction 
    .u8     in  
.ends

// FIFO content
.struct struct_usrStatsReq
    .u16    index       // index to the counter to be updated
    .u16    pktSize     // record the size of packet which initiates the statistics update
.ends

// FIFO processing control block
.struct struct_usrStatsFifoContext
    .u8     cnt         // number of PDSP entries left
    .u8     pdsp        // current processing PDSP
    .u16    cbOffset    // offset to the PDSP FIFO control block
    .u16    offset      // offset to the PDSP FIFO
.ends

// User Statistics update  Context
.struct struct_usrStatsUpdateContext
    .u16   lnkIndex    // Link index for the current counter
    .u16   lnkOffset   // Offset from the top of the link table
    .u16   cntOffset   // Offset from the top of the statistics table
    .u16   cnt32Offset // base offset of 32-bit counters 
    .u32   data1
    .u32   data2
.ends

// User Statistics clear  Context
.struct struct_usrStatsClearContext
    .u32   clrBitMap   // Clear bitmap of 32 counters
    .u16   cntIndex    // Index of the current counter
    .u16   cntOffset   // Offset from the top of the statistics table
    .u16   cnt32Offset // base offset of 32-bit counters 
    .u8    bitLoc      // the current botLoc at the clrBitMap
    .u8    rsvd1
    .u32   rsvd2
.ends


// IP Fragmentation Control Context
.struct struct_ipFragContext
    .u16  mtuSize       // desired MTU size
    .u8   ipOffset      // offset to the IP header from the top of the packet  
    .u8   ipHdrLen      // IP header size
    .u16  ipLen         // total IP length including IP header
    .u16  baseOffset    // original payload offset. It will be non-zero only if the original packet is fragmented.
    .u16  payloadSize   // payload size of the current fragment
    .u16  loopOffset    // offset to the payload of the next fragment
    .u8   nextHdr       // nextHdr in the last non-fragmentable Header
    .u8   rsvd          // alignment 
    .u16  fragTotalSize   // Total fragmentable Ipv6 payload size
    .u32  fragId       // 32-bit IPv6 fragmentation Id
.ends

// Patch Message Length 
.struct  struct_patchMsgLen
    .u8  msgLenSize           //msgLenSize in bytes
    .u8  offset               //offset to message length
    .u16 msgLen               //message Length
.ends    

// IP Reassembly Traffic Flow
.struct struct_ipTrafficFlow
    .u8   cnt            // number of pending fragments and non-fragmented packets
    .u8   proto          
    .u16  rsvd
    .u32  srcIp
    .u32  destIp
.ends  

#define PA_INV_TF_INDEX     0xFF
    

// IP Reassembly Control Context
.struct struct_ipReassmContext
    .u8   numTF        // Maximum number of Traffic Flow entries
    .u8   numActiveTF  // Number of active Traffic Flows  
    .u16  offset       // Local: offset to the current TF to be loaded
    .u16  queue        // The destination queue where PASS will deliver the packets which require reassembly assistance 
    .u8   flowId       // Specify the CPPI flow which instructs how free queues are used for receiving packets
    .u8   rsvd2
    .u32  tfMap        // 32-bit TF bitmap. 0: inactive; 1: active
    .u32  tfMapTemp    // Local: Local copy of the tfMap which represents the TF flows which has not been compared yet 
.ends

// Additional tracking context for modify (Tx command)
.struct struct_modifyState3
    .u8  rsvd
    .u8  remCmdSize   //  Remaining command size log for error protection     
.ends

#endif  // _PDSP_SUBS_H
