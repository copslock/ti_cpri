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

//===========================================================
//
// Macros
//

// mov32 : Move a 32bit value to a register
.macro  mov32
.mparam arg1, arg2
        mov     arg1.w0, arg2 & 0xFFFF
        mov     arg1.w2, arg2 >> 16
.endm


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
//        where Ingress1 PDSP0 for outer IP reassembly and Ingress4 PDSP0 for inner IP Reassembly
// 
#define  FIRMWARE_CMD_IP_REASSEM_CFG            1         
#define  FIRMWARE_CMD_IP_REASSEM_CFG_OFFSET     4       // command offset  (1st command)
#define  FIRMWARE_OUTER_IP_PDSP                 2       // Ingress0, PDSP0
#define  FIRMWARE_INNER_IP_PDSP                 6       // Ingress4, PDSP0
#define  FIRMWARE_OUTER_RA_PDSP                 1       // Ingress0, PDSP1
#define  FIRMWARE_INNER_RA_PDSP                 5       // Ingress3, PDSP0
#define  FIRMWARE_TX_CMD_PDSP                  11       // Egress0,  PDSP1

//
//  PDSPx informs the corresponding PDSP that the host has send another packet control 
//  This is overlaid with IP Reassem configuration (PA assisted reassembly which is more likely not used as
//  hardware support for reassembly is supported for this version of hardware
//  however, care should be taken that no two
//        configuration:
//        Ingress 0 PDSP0 for PA time stamp accumulation
//        Ingress 0 PDSP1 for EOAM feature, 
//        Ingress 1 ESP Header Proc (time stamp )
//        Ingress 3 PDSP5 for Firewall operations during EOAM
//
//  b3: command
//  b1: valid flags
//  b0: control flags 
#define  FIRMWARE_CMD_EOAM_CFG                 2         
#define  FIRMWARE_CMD_EOAM_CFG_OFFSET          4        // command offset  (1st command)


// Inform IPSec NAT-T Detector enabled at Ingress1, PDSP1 to Ingress 0 PDSP1
//
//  PDSPx informs the Ingress 0 PDSP1 that the host has send another packet control 
//  This is overlaid with IP Reassem configuration and EOAM configuration
//  with the assumption that the system configuration and the command configurations
//  does not occur at the same time as there is only one master DSP core doing the
//  system initializations, hence the problem is not triggered. Revisit this area
//  if this is not the case.
//  b3: command
//  b1: valid flags
//  b0: control flags 
#define  FIRMWARE_CMD_IPSEC_NAT_T_EOAM_CFG          3         
#define  FIRMWARE_CMD_IPSEC_NAT_T_EOAM_CFG_OFFSET   4        // command offset  (1st command)


//
//  PDSPx informs the corresponding PDSP that the host has send another packet control 
//        configuration:
//        Ingress 0 PDSP0(0) for PPPoE header verfication, 802.1ag detector
//        Ingress 1 PDSP0(2) and Ingress 4 PDSP0 (6) for IP header verfication, IP fragments routing
//
//  b3: command
//  b1: valid flags
//  b0: control flags 
#define  FIRMWARE_CMD_PKT_CTRL_CFG            1         
#define  FIRMWARE_CMD_PKT_CTRL_CFG_OFFSET     12       // command offset  (3rd command)

//
//  PDSPx informs the corresponding PDSP that the host has send another RA control 
//        configuration:
//        Ingress 0 PDSP0(1) for RA of outer IP
//        Ingress 3 PDSP0(5) for RA of inner IP
//
//  b3: command
//  b0: control flags 
//  b1: flow Id
//note: No need to check command code since each PDSP only handle one type of command. 
//      Need to enhance command process route if required
#define  FIRMWARE_CMD_RA_CFG                  2         
#define  FIRMWARE_CMD_RA_CFG_OFFSET          12       // command offset  (3rd command)


// Packet source
//#define PAS_PKT_SRC_RIO         4  // Need to get the real value! (not yet in spec)
//#define PAS_PKT_SRC_MAC         5  // Also need the real value

// Action values
#define SUBS_ACTION_PARSE       0
#define SUBS_ACTION_LOOKUP      1
#define SUBS_ACTION_EXIT        2

// The following value is not used at the dispatch table
#define SUBS_ACTION_FWPKT       3    // Forward the packet out
#define SUBS_ACTION_FWPKT2      4    // Forward the packet to the next stage
#define SUBS_ACTION_FWPKT3      5    // Forward the packet to Ingress4 for L4 classification
#define SUBS_ACTION_FWPKT4      6    // Forward the packet to Ingress3 for L3/L4 ACL operation
#define SUBS_ACTION_FWPKT5      7    // Forward the BC/MC packet to host
#define SUBS_ACTION_DISCARD    10    // Discrad the packet

// PSInfo Command Header
.struct  struct_cmdHeader
    .u16  cmdOffset             // Offset to a command overview
    .u16  linkedCmdOffset         
.ends

// Packet descriptor - course view
.struct struct_pktDscCourse
    .u32   pif0
    .u32   pif1
    .u32   pif2
    .u32   pif3
    .u32   pif4
    .u32   timestamp
    .u32   swinfo0
    .u32   swinfo1
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
    
    .u16   pktFlags          //b4:b0:  One-based Tx EMAC port number
    .u16   destQ
    
    .u32   timestamp
    .u32   swinfo0
    .u32   swinfo1

.ends


.struct struct_pktDscCpsw
    .u32   pif0
    
    .u8    pvtData1
    .u8    pvtData2
    .u8    validPsSize
    .u8    physPsSize
    
    .u8    pktType_pvtFlags
    .u8    psFlags_errorFlags  // b3: CRC present, b2: CRC Type
    .u8    srcId
    .u8    flowIdx
    
    .u16   ctrlDataSize
    .u16   pktDataSize
    
    .u8    pktFlags          
    .u8    outPort           //b4:b0:  One-based Tx EMAC port number
    .u16   destQ
    
    .u32   tsWord            //Time Sync Word: b31: tsEn, b27:20: domain, b19:16: message type, b15:0 seq ID
    .u32   swinfo0           
    .u32   swinfo1

.ends

#define t_psFlags_crcPresent      t7       // Ingress packet contains CRC
#define t_psFlags_crc_Castagnoli  t6       // Ingress packet contains Castagnoli CRC

#define t_psFlags_pktDropFromRA   t7       // Internal: Indicate that packet should be dropped after IP reassembly
                                           // Drop check is only perform at the PDSP next to RA                      

#define t_pktBypass               t15
#define t_pktReportTs             t14
#define t_pktCapture              t13
#define t_pktPatchTimeEoam        t12      // Internal: Packet Modifier PDSPs use this flag
#define t_pktEthernet             t12      // Internal: EOAM PDSP uses this flag to differentiate packets from Ethernet

#define t_errorFlags_cksumIp      t3
#define t_errorFlags_cksumL4      t2
#define t_errorFlags_crc          t1


// Store timestamp related parameters at the swInfo1 and timestamp (flowId and Queue) since timestamp should be recorded at the last PDSP

#define PA_PKT_ID_MASK          0x3fff
#define PA_EMAC_PORT_MASK       0x1f
#define PA_PKT_TYPE_MASK        0xf8
#define NOT_PA_PKT_TYPE_MASK    0x07
#define PA_PKT_TYPE_SHIFT       3
#define PA_PKT_PS_FLAGS_MASK        0xf0
#define NOT_PA_PKT_PS_FLAGS_MASK    0x0f
#define PA_PKT_PS_FLAGS_SHIFT   4

#define PA_PKT_TYPE_SRIO_TYPE_9     30
#define PA_PKT_TYPE_SRIO_TYPE_11    31
#define PA_PKT_TYPE_CPSW             7

#define t_pkt_desc_err_flag_crc         t1
#define t_pkt_desc_err_flag_chksum2     t2
#define t_pkt_desc_err_flag_chksum1     t3

//===========================================================
//
// Packet Descriptor
//
.struct struct_PktDesc
    .u8     ThreadId        // Thread ID
    .u8     SA_EgressStatus // Egress status length in SA
    .u16    SA_FullSizeL    // Low word of Full Size in SA
    .u8     SA_NextEngId    // Next Engine ID in SA
    .u8     SA_CmdLblInfo   // Command Label Info in SA
    .u8     PsValidSize     // Byte size of valid data in PsInfo
    .u8     PsPhysSize      // Physical byte size of PsInfo
    .u16    PktInfo         // Packet Flags and Type Fields
#define fDROP   t10
#define fSOP    t9
#define fEOP    t8
    .u8     SrcID           // Source ID
    .u8     FlowIndex       // Flow Index
    .u16    ControlSize     // Control Data Size
    .u16    PacketSize      // Packet Data Size
    .u16    PacketId        // Unique ID for this packet
    .u16    DestQueue       // Destination queue
    .u32    Timestamp       // Packet Timestamp
    .u32    SwInfo0         // Software Info Word 0
    .u32    SwInfo1         // Software Info Word 1
.ends

// RA control structure
.struct struct_raInfo
    .u16    l3Offset        // Offset to L3
    .u8     priority        // DSCP / Priority Code
    //  /--------------------------\
    //  | 7        5 | 4        0  |
    //  |  flags     |  Thread ID  |
    //  \--------------------------/
    .u8     flags           // Thread ID and falgs
#define t_ra_flag_reasm         t7  // Do IP reassembly
#define t_ra_flag_2nd_inst      t6  // Use the second RA instance   
#define t_ra_flag_destQ         t5  // Destination queue overwrite
    .u16    ipv6NextOffset  // Offset to IPv6 "Next Header" Field
    .u16    ipv6FragOffset  // Offset to IPv6 Frag Header
.ends


//===========================================================
//
// Packet Extended Info
//
.struct struct_pktExtInfo
    .u16    sopLength       // SOP Byte Length
    .u8     threadId        // Final Egress Thread Id
    .u8     flags           // Packet State Flags
#define fFinal      t0
#define fCSUM       t1
#define fDoCRC      t2
#define fCRCType0   t3
#define fCRCType1   t4
#define fDroppedInd t6      // softwarefalg to indicate that this packet should be dropped at the last stage 
#define fDropped    t7
    .u16    mopCsum         // MOP 1's compliment SUM
    .u16    mopLength       // MOP Byte Length
    .u32    mopPtr          // Pointer for MOP data
    .u16    crcLength       // CRC Range Length
    .u16    crcOffset       // Data Offset to Start of CRC
    .u32    crcValue        // CRC Start/End Value
.ends


//===========================================================
//
// Packet Extended Info for IP fragmentation opertaion
//
// Use CRC field to store the original MOP information
//
.struct struct_pktExtInfo2
    .u16    sopLength       // SOP Byte Length
    .u8     threadId        // Final Egress Thread Id
    .u8     flags           // Packet State Flags
    .u16    mopCsum         // MOP 1's compliment SUM
    .u16    mopLength       // MOP Byte Length
    .u32    mopPtr          // Pointer for MOP data
    .u16    sopL4Length     // Length of SOP excluding L2/L3 header in bytes
    .u16    mopLengthOrig   // Original MOP Length
    .u32    mopPtrOrig      // Original MOP Ptr
.ends


// The control data 1st byte identifies the packet format
//  /-----------------------------------\
//  | 7       5  |   4               0  |
//  |  Cmd Id    |                      |
//  \-----------------------------------/
#define  SUBS_CMD_WORD_ID_SHIFT            5
#define  SUBS_CMD_WORD_ID_MASK_SHIFTED     0x7


// Command values 
// TBD modified
// Ingress Command 
#define PSH_CMD_PA_RX_PARSE        0   // Ingress
#define PSH_CMD_PA_TX_CHKSUM       0   // Egress
#define PSH_CMD_SA_LONG_INFO       0   // SA only

#define PSH_CMD_PA_TX_CRC          1   // Egress
#define PSH_CMD_SA_SHORT_INFO      1   // SA only

#define PSH_CMD_PA_TX_BLIND_PATCH  2   // Egress

#define PSH_CMD_PA_TX_NEXT_ROUTE   3   // Egress

#define PSH_CMD_CONFIG             4   // All

#define PSH_CMD_RX_FWD             5   // Post-Processing

#define PSH_CMD_PA_TX_FLOW         5   // Egress Flow processing

#define PSH_CMD_PA_TX_RPT_TS       6   // Egress: Report Timestamp   

#define PSH_CMD_PA_GROUP_7         7   // Egress: Group 7 commands: specify the sub-command code
#define PSH_CMD_PA_DUMMY           7   // Egress: Dummy commad which is used for alignment puepose only
#define PSH_CMD_PA_IP_FRAG         7   // Egress: Ip Fragmenation 


//TBD 
#define PSH_CMD_DEST_PA     1
#define PSH_CMD_DEST_SA     2
#define PSH_CMD_DEST_HOST   3
#define PSH_CMD_DEST_ALL    4


.struct  struct_pasPktContext
     // WORD0
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

//flag 
// Note: Both control and indication flags may be overlayed among PDSP clusters, i.e. each bit can have different meaning
//       at different stage
//  
// 
// Control flag
//#define t_flag_ctrl_reasm       t0      // Packet is fragmented, perform reassem ??

// IP Reassembly flags  (only if we still support PASS-assistent reassembly for backward compatibility)
#define t_flag_2nd_pass         t4  // To be clear at second pass
#define t_flag_null_pkt         t5  // This flag will be set by host to indicate that the packet should be dropped
                                    // It is used to clear the traffic flow when the reassembly timeout occur
                                    // It can be shared with other flags since the packet will be dropped when 
                                    // this flag is set  
// Status or indication flag  
#define t_flag_acl_mark                t0    // packet which is marked per ACL rule
#define t_flag_cascaded_forwarding     t1    // Indicate that packets should be forwarded to next processor without any changes
                                             // Disable some global actions such as IP assembly-assisted operation and/or
                                             // fragments exception route
#define t_flag_use_vlink               t2    // Use virtual link in stead of physical link for classification 
                                             
#define t_flag_broadcast        t8
#define t_flag_multicast        t9
#define t_flag_vlan2            t10
#define t_flag_frag             t11
#define t_flag_frag1st          t12
#define t_flag_ttl_exp          t13     // IP TTL is 1 or 0
#define t_flag_rsvd             t14     // This flag is set in Ingress to indicate to Egress PDSP that CmdSet is included in the packet
                                        // Pkt with CmdSet ==> QoS Queue ==> Egress PDSP ==> Post Processing PDSP
#define t_flag_eg_cmdSet        t14
#define t_flag_pl1Match         t15

// PASS-SASS intercommunication flags  
#define t_flag_gtpu_seqnum_present  t3  

// PAM command control flags
#define t_flag_split            t4      // Used in POST PDSP0 and PDSP1     
#define t_flag_crc_verify       t5     
#define t_flag_cmdset           t6
#define t_flag_multi_route      t7

// IP Processing detector flag (Required for Ingress 3)
#define t_flag_ipProc           t4

    // Word 1
    .u16  endOffset
    
    // eIdx: Exception Index, Multi-set Index or Custom Index
    //       original threadId while command set is used
    // portNum: Input EMAC port number
    // nextHdr: next Protocol header to be processed
    // /------------------------------------\
    // | 15    10 |   9      6 |   5       0 |
    // |   eIdx   |  portNum   |   nextHdr   |
    // \------------------------------------/
    .u16  eId_portNum_nextHdr
    
     // Word 2
    .u8  l3Offset       // Offset to the first IP
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  l5Offset       // Offset to layer 5 header
    .u8  espAhOffset    // Offset to the latest ESP/AH header
    
    // Word 3 
    //  /-----------------------------\
    //  | 15     10    | 9         0  |
    //  | CDE number   | Lut Index    |
    //  \-----------------------------/
    .u16  phyLink

    //  /-----------------------------\
    //  | 15                       0 |
    //  |       hdrBitmask           |
    //  \-----------------------------/
    .u16  hdrBitmask
    
// hdrBitmask: A bit index of headers found
#define  SUBS_PA_BIT_HEADER_MAC         t0
#define  SUBS_PA_BIT_HEADER_VLAN        t1
#define  SUBS_PA_BIT_HEADER_MPLS        t2
#define  SUBS_PA_BIT_HEADER_802_3       t3
#define  SUBS_PA_BIT_HEADER_PPPoE       t4
#define  SUBS_PA_BIT_HEADER_IPv4        t5
#define  SUBS_PA_BIT_HEADER_IPv6        t6
#define  SUBS_PA_BIT_HEADER_IP_OPTIONS  t7
#define  SUBS_PA_BIT_HEADER_ESP         t8
#define  SUBS_PA_BIT_HEADER_AUTH        t9
#define  SUBS_PA_BIT_HEADER_SCTP        t10
#define  SUBS_PA_BIT_HEADER_UDP         t11
#define  SUBS_PA_BIT_HEADER_TCP         t12
#define  SUBS_PA_BIT_HEADER_GTPU        t13
#define  SUBS_PA_BIT_HEADER_CUSTOM      t14
#define  SUBS_PA_BIT_HEADER_IPSEC_NAT_T t15
    
    // Word 4             
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    //  /------------------------------------\
    //  | 7         6| 5        3 | 2     0  |
    //  |  vlanCount |  greCount  |  ipCount |
    //  \------------------------------------/
    .u8  protCount        // vlan|GRE|IP (may be removed)
    
     // Word 5
    .u16 pseudo           // Ipv4 pseudo checksum 
                          // (CDE packet size only for the Ingess Copy PDSP)  
    .u16 timestamp        // Upper 16-bit of timestamp
.ends

//eId_portNum_nextHdr
#define PKT_NEXTHDR_MASK        0x3F
#define NOT_PKT_NEXTHDR_MASK    0xC0

#define PKT_EIDX_MASK           0xFC
#define NOT_PKT_EIDX_MASK       0x03
#define PKT_EIDX_SHIFT          2

#define PKT_EMACPORT_MASK       0x0F
#define PKT_EMACPORT_SHIFT      6

//paCmdId_Length
#define PKT_CONTEXT_CMD_MASK      0xe0
#define NOT_PKT_CONTEXT_CMD_MASK  0x1F
#define PKT_CONTEXT_CMD_SHIFT 5 

//phyLink
#define PKT_L1IDX_MASK            0x03  // MSB (higher byte) only
#define PKT_PDSP_ID_SHIFT         10  

#define PA_BM_FLAG_MASK           0x03

//Vlan priority and virtual Link
#define PKT_VALN_PRI_MASK         0xE0
#define PKT_VLAN_PRI_SHIFT           5
#define NOT_PKT_VLAN_PRI_MASK     0x1F  

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

// This is a repeat of pasPktContext for configuration command  
// only the latest 3 32-bit worda are used (R25-R27)
.struct  struct_pasPktContext2
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    .u8  l3Offset       // Offset to the first IP
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  l5Offset       // Offset to layer 5 header
    .u8  espAhOffset    // Offset to the latest ESP/AH header
     
    // Word 3
    .u16  commandResult
    .u16  commandOffset  // Offset to the next command to be processed
     
    // Word 4
    .u16  replyQueue
    .u8   replyDest
    .u8   flowId
    
     // Word 5
    .u8  ctrlFlag
    .u8  psInfoSize
    .u16 rsvd
.ends

#define t_cmdDiscard            t7
#define t_cmdExtPktInfo         t6      // reload extended packet Info
#define t_cmdContinue           t5      // Need to continue to process the command at the next stage: Patch psInfo 0x80000000
#define t_cmdResultPatch        t4      // Update command result
#define t_cmdHdrMoved           t3      // The CDE window is moved passed the command header, therefore the command result patch is required if changes.

// This is a repeat of pasPktContext for IP reassembly-assistance  
// only the latest 32-bit word is used (R27)
.struct  struct_pasPktContext3
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    .u8  l3Offset       // Offset to the first IP
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  l5Offset       // Offset to layer 5 header
    .u8  espAhOffset    // Offset to the latest ESP/AH header
     
    // Word 3
    .u16 phyLink
    .u16 hdrBitmask
     
    // Word 4
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    .u8  protCount      // vlan|GRE|IP (may be removed)
    
     // Word 5
    // re-use the pseudo field since the field is not used at the first pass
    .u8  tfIndex        // Traffic Flow Index
    .u8  fragCnt        // Number of fragments in the packet
    .u16 timestamp      // Upper 16-bit of timestamp
.ends

// This is a repeat of pasPktContext for configuration command  
// only the latest 32-bit word is used (R27)
.struct  struct_pasPktContext4
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    .u8  l3Offset       // Offset to the first IP
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  l5Offset       // Offset to layer 5 header
    .u8  espAhOffset    // Offset to the latest ESP/AH header
     
    // Word 3
    .u16  phyLink
    .u16  hdrBitmask
     
    // Word 4
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    .u8  protCount      // vlan|GRE|IP (may be removed)
    
     // Word 5
    .u8  cmdSetIdx      // command set index                 
    .u8  threadIdOrig   // threadId if command set required
    
    .u16 timestamp      // Upper 16-bit of timestamp
.ends

// This is a repeat of pasPktContext for L3 offset enhancements
// Inner IP offset will be stored at l5offset location until
// L4 operation or packet is delivered to host where the inner L3 offset 
// will be moved to psuedo checksum location  
.struct  struct_pasPktContext5
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    .u8  l3Offset       // Offset to the first IP
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  l3l5Offset     // Offset to the inner IP loaction (overlap with l5offset)
                        // it is valid until the TCP/UDP processing and thyen the offset information will
                        // be passed to l3offset2
    .u8  espAhOffset    // Offset to the latest ESP/AH header
     
    // Word 3
    .u16 phyLink
    .u16 hdrBitmask
     
    // Word 4
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    .u8  protCount      // vlan|GRE|IP (may be removed)
    
     // Word 5
    // re-use the pseudo field since the field is no longer needed */
    .u8  l3offset2      // Offset to the inner most IP
    .u8  rsvd           // alignment
    .u16 timestamp      // Upper 16-bit of timestamp
.ends

#define PA_ONE_SEC_EXP_IN_NS        1000000000
#define PA_DELTA_TIME_TICK_ADJUST   54 
#define PA_EOAM_OPCODE_LMR          42
#define PA_EOAM_OPCODE_LMM          43
#define PA_EOAM_OPCODE_IDM          45
#define PA_EOAM_OPCODE_DMR          46
#define PA_EOAM_OPCODE_DMM          47
#define PA_EOAM_APS_LINEAR_OPCODE   39
#define PA_EOAM_APS_RING_OPCODE     40
#define PA_EOAM_CCM_OPCODE          1

#define PA_TIME_SHIFT_VAL           13

// Below are arbitrary selected, would need to compute the actual cycles when coded in fw 
#define PA_TIME_CORRECTION_SEC      10
#define PA_TIME_CORRECTION_NS       10
#define PA_TIME_AMBIGUIOUS_TIMER_TICK   200


// This is a repeat of pasPktContext for Ethernet OAM enhancements
// As Ethernet OAM packets are non IP packets, all IP related fields can 
// over lay with the EOAM packet context
// When the packet is delivered to host,
// the MEG level from the packet is reported in Word 2 (b0)
// the Opcode of the EOAM control packet is reported in Word2 (b1)
//
.struct  struct_pasPktContext6
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset
    
    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    // reuse IP Offset location field as it is no longer needed for EOAM
    .u8  megLevel       // MEG level 
    .u8  opCode         // Opcode recorded from the EOAM packet
    .u8  eoamFlags      // 8 bit flags that control statistics, indicate IPSec Transport, IPSec Tunnel or non IP pkts

    // EOAM flags  
#define t_flag_ip               t0  // Set Indicates IP Pkt otherwise non IP pkt
#define t_flag_cipherPkt        t1  // Set Indicates Crypto packet otherwise non crypto pkt
#define t_flag_fDisableCnt      t2  // Set Indicates force Ethernet OAM does not need record
#define t_flag_knownOpcodes     t3  // Set Indicates that the opcode of the packet is a known EOAM control packet (1DM/LMM/LMR/DMM/DMR)
#define t_flag_eType8902        t4  // Set indicates that ethertype 0x8902 packet is detected
#define t_flag_reportCount      t5  // Set Indicates that count value needs to be reported
#define t_flag_invalidVer       t6  // Set indicates that the EOAM packet version is not zero and hence invalid
#define t_flag_fragFound        t7  // Set indicates that the EOAM parsing found IP Frag packet

    .u8  startOffsetEoam
     
    // Word 3
    .u16 phyLink
    .u16 hdrBitmask
     
    // Word 4
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    .u8  protCount      // vlan|GRE|IP (may be removed)
    
     // Word 5
    // re-use the pseudo field since the field is no longer needed */
    .u32 count                // 32 bit counter 
.ends


// This is a repeat of pasPktContext for Ethernet OAM enhancements
// As Ethernet OAM packets are non IP packets, all IP related fields can 
// over lay with the EOAM packet context
// When the packet is delivered to host,
// the MEG level from the packet is reported in Word 2 (b0)
// the Opcode of the EOAM control packet is reported in Word2 (b1)
//
.struct  struct_pasPktContext7
    // WORD0
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset
    
    // Word 1
    .u16  endOffset
    .u16  eId_portNum_nextHdr
    
    // Word 2 
    // reuse IP Offset location field as it is no longer needed during timestamp recording in In0
    .u32  timeStamp_hi
     
    // Word 3
    .u16 phyLink
    .u16 hdrBitmask
     
    // Word 4
    //  /------------------------------------\
    //  | 15         13| 12             0  |
    //  |  vlanPri     |  virtual link     |
    //  \------------------------------------/
    
    .u16 vlanPri_vLink
    .u8  priority
    .u8  protCount      // vlan|GRE|IP (may be removed)
    
     // Word 5
    // re-use the pseudo field since the field is no longer needed */
    .u32  timeStamp_lo
.ends

// Tx (Egress Flow) Packet Info
.struct  struct_pasTxPktContext
     // WORD0
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

// flag 
// Note: Both control and indication flags may be overlayed among PDSP clusters, i.e. each bit can have different meaning
//       at different stage
//  
// 

// Status or indication flag  
#define t_flag_fc_lookup          t15      // Flow cache lookup is required
#define t_flag_ah_patch           t14      // this flag is set only during egress flow operations and any other time if this bit is
                                           // set, it would be overloaded with CmdSet operations pending to be executed from QoS
#define t_flag_ip_frag            t13      // IP fragment required at the next stage  

// egress flow record valid bits
#define t_flag_ef_rec_valid_lvl1  t4
#define t_flag_ef_rec_valid_lvl2  t5
#define t_flag_ef_rec_valid_lvl3  t6
#define t_flag_ef_rec_valid_lvl4  t7

#define PA_EF_ICV_SIZE_MASK       0x3    // ICV size: 8, 12, 16 == 8 + (b1b0 << 2)
#define NOT_PA_EF_ICV_SIZE_MASK   0xFC

    //  Word 1
    .u16 endOffset
    
    .u8  l2Offset       // Offset to L2 header
    .u8  l3Offset2      // Offset to inner IP header
    
     // Word 2: the same as pktInfo          
    .u8  l3Offset       // Offset to the first IP 
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  rsvd1
    .u8  espAhOffset    // Offset to the latest ESP/AH header (?)
    
    //  Word 3             
    .u8  lvl1RecIndex   // Egress Flow level one record index
    .u8  lvl2RecIndex   // Egress Flow level two record index
    .u8  lvl3RecIndex   // Egress Flow level three record index
    .u8  lvl4RecIndex   // Egress Flow level four record index
    
    // Word 4
    .u16 fragSize       // IP Fragment size
    .u16 lastFragSize   // Size of last fragment
    
     // Word 5
    .u8  nextHdr
    .u8  ipHdrLen       // IPv4 header header length: IPv6 header plus non-fragmentable extension headers
    .u8  nextHdrOffset  // Offset to IPv6 last non-fragmentable extension header   
    .u8  rsvd2 
.ends


// Tx (Egress Flow) Packet Info
.struct  struct_pasTxPktContext2
     // WORD0
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    //  Word 1
    .u16 endOffset
    
    .u8  l2Offset       // Offset to L2 header
    .u8  l3Offset       // Offset to first IP header
    
     // Word 2: the same as pktInfo          
    .u8  l3Offset2      // Offset to the inner IP (to be compatible with the ingress record 
                        // and maintain backward compatibility 
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  rsvd1
    .u8  espAhOffset    // Offset to the latest ESP/AH header (?)
    
    //  Word 3             
    .u8  lvl1RecIndex   // Egress Flow level one record index
    .u8  lvl2RecIndex   // Egress Flow level two record index
    .u8  lvl3RecIndex   // Egress Flow level three record index
    .u8  lvl4RecIndex   // Egress Flow level four record index
    
    // Word 4
    .u8  eId            // Exception Id
    .u8  rsvd2
    .u16 rsvd3          
    
     // Word 5
    .u32 rsvd4 
.ends

// Tx (Egress Flow) Packet Info
// Re-use the record index fileds 
.struct  struct_pasTxPktContext3
     // WORD0
     // /-----------------------\
     // | 7     5  |  4       0 |
     // |  cmd Id  |   length32 |
     // \-----------------------/
    .u8   paCmdId_Length
    .u16  flags // 16-bit control flag
    .u8   startOffset

    //  Word 1
    .u16 endOffset
    
    .u8  l2Offset       // Offset to L2 header
    .u8  l3Offset2      // Offset to inner IP header
    
     // Word 2: the same as pktInfo          
    .u8  l3Offset       // Offset to the first IP 
    .u8  l4Offset       // Offset to Layer 4 header
    .u8  rsvd1
    .u8  espAhOffset    // Offset to the latest ESP/AH header (?)
    
    //  Word 3             
    .u8  dscp           // 6-bit DSCP value 
    .u8  vlanPri        // 3-bit VLAN priority
    .u8  lvl3RecIndex   // Egress Flow level three record index
    .u8  lvl4RecIndex   // Egress Flow level four record index
    
    // Word 4
    .u16 fragSize       // IP Fragment size
    .u16 lastFragSize   // Size of last fragment
    
     // Word 5
    .u8  nextHdr
    .u8  ipHdrLen       // IPv4 header header length: IPv6 header plus non-fragmentable extension headers
    .u8  nextHdrOffset  // Offset to IPv6 last non-fragmentable extension header   
    .u8  rsvd2 
.ends


// Tx (Egress Flow) Fragment Info
.struct  struct_pasEfFragInfo
    .u16 fragSize       // IP Fragment size
    .u16 lastFragSize   // Size of last fragment
    
    .u8  nextHdr
    .u8  ipHdrLen       // IPv4 header header length: IPv6 header plus non-fragmentable extension headers
    .u8  nextHdrOffset  // Offset to IPv6 last non-fragmentable extension header   
    .u8  rsvd 
.ends




//===========================================================
//
// NetCP Firmware Error Codes (TBD)
//

#define NETCP_ERROR_BAD_CRC             1
#define NETCP_ERROR_BAD_QINQ            2
#define NETCP_ERROR_BAD_SNAP            3
#define NETCP_ERROR_BAD_PPPOE           4
#define NETCP_ERROR_BAD_IPV4VERSION     5
#define NETCP_ERROR_BAD_IPV4LENGTH      6

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
    // /-------------------------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5        |       4       |   3       |   2              |       1        |   0         |
    // |  held packet   | previous held  | pending config | previous skip |  L4 Avil  |  first lookup    | extHdr Restore | fail lookup |
    // \-------------------------------------------------------------------------------------------------------------------------------/
    .u8    flags
    
    // /-------------------------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5          |       4        |       3      |       2        |     1     |      0       |
    // |  Hdr check     | ipFragToEroute | 802.1ag standard | 802.1ag Detect | IP Reassem   |  SCTP Checksum |  pkt Cap  |  IP Reassm   |
    // \--------------------------------------------------------------------------------------------------------------------------------/
    .u8    flag2
    .u8    raFlow      // Flow Id for RA: Note the queue number is fixed:PASS_RA_QUEUE
    // /---------------------------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5          |       4        |       3      |   2   |           1           |     0       |
    // |                |                |                  |                |              |       |  ACL Rescore triggered| EOAM Enable |
    // \---------------------------------------------------------------------------------------------------------------------------------/    
    .u8    flag3
.ends

// flag bit map
#define   t_held_packet           t7
#define   t_previous_held         t6
#define   t_pendingConfig         t5
#define   t_previous_skip         t4
#define   t_l4Avil                t3    // 0: L4 entry is not available 
#define   t_firstLookup           t2
#define   t_extHdr_held           t1    // 1: extHdr is restored and need to be pushed out
#define   t_failLookup            t0    // 1: LUT1 lookup is determined to fail due to unknown (unsupported) protocol
  
// flag2 bit map
#define   t_hdrCheck              t7    // 1: Enhanced header check is enabled
#define   t_ipHdrCheck            t7    // 1: Enhanced IP header check is enabled (Ingress1 and Ingress4 PDSP0)
#define   t_PPPoEHdrCheck         t7    // 1: PPPoE haeder check is enabled (Ingress0, PDSP0)
#define   t_macPaddingChk         t6    // 1: MAC (802.3 only) padding check (Ingress0, PDSP0)
#define   t_ipFragToEroute        t6    // 1: All IPv4 fragments should be delivered through exception route (Ingress1 and Ingress4 PDSP0) 
#define   t_l3toInnerIp           t5    // 1: L3offset points to inner IP (NA)
#define   t_IpsecNatTDetEnEoam    t5    // 1: IPSEC ESP NAT-T detector is enabled in Ingress1 - PDSP1 (Info needed in Ingress 0 PDSP1 during EOAM)
#define   t_802_1agStd            t5    // 1: 802.1ag standard mode (Destination MAC=01-80-C2-00-00-3X)(Ingress0, PDSP0) 
#define   t_802_1agDet            t4    // 1: 802.1ag Detector is enabled (Ingress0, PDSP0)
#define   t_ipReassmEn            t3    // 1: IP reassembly assistance is enabled (Ingress1, PDSP0 and Ingress4, PDSP0)
#define   t_sctpChksum            t2    // 1: SCTP checksum is enabled (Ingress1, PDSP0, Ingress4, PDSP0)
#define   t_def_route             t2    // 1: Ingress default route (Ingress0, PDSP0)
#define   t_IpsecNatTDetEn        t2    // 1: IPSEC ESP NAT-T detector is enabled (Ingress1 - PDSP1)
#define   t_ingress_pCapEnable    t1    // 1: packet capture feature enable (Ingress0, PDSP0)

// RA processing stage: Ingress0, PDSP1 and Ingress3, PDSP0 (Also enabled in Ingress1 PDSP0, Ingress4 PDSP0)
//
#define   t_raEn                  t0    // 1: IP Reassembly is enabled  
#define   t_raUseLocDMA           t1    // 1: IP Reassembly using local DMA
#define   t_raToQueue             t2    // 1: IP reassembly output packets to host queue

// flag3 bit map
#define   t_eoamEn                t0    // 1: Ethernet OAM feature is enabled (Ingress0-PDSP1, Ingress3-PDSP5)
#define   t_trigRescoreProc       t1    // 1: Firewall Rescore triggered and in progress: Ingress1-PDSP1, Ingress3-PDSP0


#define   SUBS_PKT_CTRL_RA_En              0x01
#define   SUBS_RA_CTRL_MASK                0x07
#define   NOT_SUBS_RA_CTRL_MASK            0xf8   

#define   SUBS_PKT_CTRL_HDR_CHK            0x80
#define   SUBS_PKT_CTRL_IP_FRAG_TO_EROUTE  0x40
#define   SUBS_PKT_CTRL_802_1agStd         0x20
#define   SUBS_PKT_CTRL_802_1agDet         0x10
#define   SUBS_PKT_CTRL_MASK               0xe0
#define   NOT_SUBS_PKT_CTRL_MASK           0x1f

#define   SUBS_PKT_CTRL_FW_DEF_DROP        0x20


// LUT1 table entries
.struct  struct_l1Info
    .u8  ctrlFlag         // Boolean - enables next classification info (SRIO) only
    .u8  nextHdr
    .u16 nextHdrOffset
    
    // Followed by 2 struct_paForward (each 12 bytes):
    //  forwardMatch
    //  prevMatch
.ends

// LUT1 table entries
.struct  struct_l1AclInfo
    .u16 statsCmd         // store packet count command
    .u16 statsCmd2        // store byte count command
    
    // Followed by 2 struct_paForward (each 12 bytes):
    //  forwardMatch
    //  prevMatch
.ends

// LUT1 table entries
.struct  struct_l1FcInfo
    .u16 statsCmd         // store packet count command
    .u16 uCnt             // Usage Counter:
                          // The usage counter is used to measure how frequency this flow is used
                          // It will be incremented by one per each match until it reaches the maxmium value
                          // It will be decremented by one per PASS internal timer loop until it reaches 0 
    
    // Followed by 2 struct_paForward (each 12 bytes):
    //  forwardMatch
    //  prevMatch
.ends

#define PAFRM_FC_USAGE_CNT_MAX      100


.struct struct_l1InfoVlink
    .u8  ctrlFlag
    .u8  rsvd
    .u16 vLinkNum
.ends

#define   custom_enable     t0
#define   vlink_enable      t1

// Classify2 run context
.struct  struct_c2RunContext
    .u16  c2NumInL2Table
    // /--------------------------------------------------------------------------------------------------------------------------------\
    // |      7         |       6        |       5          |       4        |       3      |       2        |     1       |     0       |
    // |                |                |                  |                |              |                |             | EOAM Enable |
    // \--------------------------------------------------------------------------------------------------------------------------------/      
    .u8   flags
    .u8   ctrlFlag
.ends

// 
// EOAM is disabled by default
//
#define t_c2_flags_enable_EOAM  t0

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
// TBD:This table should be different per PDSP with complier switch
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

// LUT1 entry mapping (for initialization only)
.struct struct_subsLut1Map
    .u32  validEntries0   // (LUT entries  0-31)
    .u32  validEntries1   // (LUT entries 32-63)
    .u32  validEntries2   // (LUT entries 64-95)
    .u32  validEntries3   // (LUT entries 96-127)
    .u32  validEntries4   // (LUT entries 128-159)
    .u32  validEntries5   // (LUT entries 160-191)
    .u32  validEntries6   // (LUT entries 192-223)
    .u32  validEntries7   // (LUT entries 224-255)
.ends

// NextFail Route Base address
// This table will be initialized by the first DSP at each Ingress stage.
// The base address will be access through LBCO operation
.struct struct_nextFailRouteBase
    .u32  pdsp0
    .u32  pdsp1
    .u32  pdsp2
    .u32  pdsp3
    .u32  pdsp4
    .u32  pdsp5
    .u32  pdsp6
    .u32  pdsp7
.ends    

// Second level map to indicate which group entry is entry 
.struct struct_subsLut1Map2
    .u32  busyMask        // (1: Indicate all entries are valid)
.ends



// Pending entry LUT1 info
//.struct struct_subsLut1AddPendingInfo
//    .u8  mode
//    .u8  index
//    .u16 matchFlags
//.ends

//#define SUBS_LUT1_PENDING_MODE_DEL_ENTRY  0
//#define SUBS_LUT1_PENDING_MODE_ADD_ENTRY  1

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
.struct struct_statsReg
    .u16  value 
.ends

#define t_stats_update_req          t15
#define t_stats_type_byte           t14    // Increment the stats by  r0 [16:0]
#define PA_STATS_UPDATE_REQ         0x8000
#define PA_STATS_TYPE_BYTE          0x4000   // byte counter
#define PA_STATS_TYPE_PKT           0x0000   // packet counter 

// LUT2 add/delete control msb values
#define SUBS_L2_ENTRY_CTRL_ADD      0xc0
#define SUBS_L2_ENTRY_CTRL_DEL      0x80


// Modify PDSP context  
.struct struct_modifyContext
    .u8  flags
    .u8  usrStatsPdsp     // User statistics polling start pdsp
    .u8  paddingCnt       // number of fragment or packet padding occurs sofar
    .u8  sopAdjust        // sopAdjustment
.ends

// flags used by Modify opertaion
//#define t_isPdsp5           t0  // 0: PDSP 4 and 1: PDSP 5
#define t_mod_eoamEnable      t0
#define t_egress_pCapEnable   t1  // 1: Egress packet capture feature enable
#define t_eqos_feature        t2  // 0: EQoS feature disabled, 1: EQoS feature enabled
// We run out of flag bits in struct_modifyState - using bits from Context, Make sure these bits are cleared for each packet
#define t_subsMCxtPatchTime   t3  // 0: EOAM patch time disabled, 1: EOAM patch time enabled
#define t_subsMCxtPatchCount  t4  // 0: EOAM patch count disabled, 1: EOAM patch count enabled

#define t_MCxtCopyState       t7  // 0: Copy Packet Pending, 1: Copy packet Done

#define SUBS_MOD_CXT_EOAM_FLAGS_SHIFT      3
#define SUBS_MOD_CXT_EOAM_FLAGS            3
#define SUBS_MOD_CXT_MAX_PATCHES           4
#define SUBS_MOD_CXT_MAX_MSGLEN_PATCHES    2


#define PA_USR_STATS_START_PDSP_MASK        0x0F                                        
#define NOT_PA_USR_STATS_START_PDSP_MASK    0xF0                                        

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
#define  t_rx_cmd_ctrl_no_tail    t5

// Additional tracking context for Egress Flow (Egress Flow Record1 Processing)
.struct  struct_efState1
    .u8   proto             // next level protocol 
    .u8   rsvd1             
    .u16  pseudo            // Ip pseudo checksum  
    .u16  payloadLen
    .u16  rsvd2
    .u32  rsvd3 
.ends

// Additional tracking context for Egress Flow (Egress Flow Record2 Processing)
.struct  struct_efState2
    .u8  extHdrFlags
    .u8  nextHdr
    .u8  flags
    .u8  paddingLen
    .u16 ipLen
    .u16 loopOffset
    .u16 baseOffset
    .u16 origSopLen
.ends

#define t_ef_firstLoop      t0             // Indicate that it is the first loop that the outer IP has not been altered
#define t_ef_lastFrag       t1             // Indicate that it is the last fragment
#define t_ef_noFrag         t2             // Indicate that there will be no fragments
#define t_ef_withinSop      t3             // Indicate the end of packet is still within the SOP range
#define t_ef_msgLen_update  t4             // Indicate that the message length update is required for this fragments
#define t_ef2_outer_ipv6    t5             // Indicate outer IP is ipv6 
#define t_ef2_retain_chksum t6             // Indicate checksum has been re-calculated

// Additional tracking context for Egress Flow Modify (Egress Flow Record3 Processing)
.struct  struct_efState3
    .u8   flags   
    .u8   hdrSize       // UDP or AH Header size 
    .u8   proto         // Updated IP protocol 
    .u8   nextHdr       // next Header at the AH header    
    .u16  payloadLen                           
    .u16  rsvd3
    .u32  rsvd4        
.ends

#define t_ef3_ipv6       t0

// Additional tracking context for Egress Flow Modify (Egress Flow Record4 Processing)
.struct  struct_efState4
    .u8  extHdrFlags
    .u8  flags
    .u16 l3Len              // L3 length of each fragment or packet
    .u16 ipLen
    .u16 loopOffset
    .u16 baseOffset
    .u16 origSopLen
.ends

.struct  struct_ipPktId
    .u32 id                 // IPv4: 16-bit packet id in fragments
                            // IPv6: 32-bit packet id in the fragment header
.ends    

// Rx Cmd processing Context
.struct struct_rxCmdContext
    .u8  cmdSetIndex   
    .u8  nCmds             // number of commands left to be processed
    .u16 cmdOffset         // offset in the memory table
    .u16 pktOffset         // current processing location of the packet    
    .u8  psInfoSize
    .u8  updateLen         // Record total szie of packet lenhth update (may be negative) */
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
    .u8     crcSize
    .u8     rsvd
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

// User Statistics read  Context
.struct struct_usrStatsReadContext
    .u16   cntOffset   // Offset from the top of the statistics table
    .u16   cnt32Offset // base offset of 32-bit counters 
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
    .u16  mtuSize      // desired MTU size
    .u8   ipOffset     // offset to the IP header from the top of the packet  
    .u8   ipHdrLen     // IP header size including non-fragmentable headers
    .u16  ipLen        // total IP length including IP header
    .u16  baseOffset   // original payload offset. It will be non-zero only if the original packet is fragmented.
    .u16  payloadSize  // payload size of the current fragment
    .u16  loopOffset   // offset to the payload of the next fragment
    .u16  dataRemain   // Data remaining in this frag
    .u16  offsetRemain // Offset left to be used in this frag
    .u8   nextHdr      // nextHdr in the last non-fragmentable Header
    .u8   exhdrFlags   // record the original flags 
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
    .u8  cmdFlushBytes   // number of bytes to flush during IP Fragment cmd processing     
    .u8  remCmdSize      // Remaining command size log for error protection
.ends

#endif  // _PDSP_SUBS_H
