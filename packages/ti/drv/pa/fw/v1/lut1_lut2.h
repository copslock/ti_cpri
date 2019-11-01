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
#ifndef _LUT1_LUT2_H
#define _LUT1_LUT2_H   1
// *********************************************************************************************************
// * FILE PURPOSE: LUT1 and LUT2 definitions
// *********************************************************************************************************
// * FILE NAME: lut1_lut2.h
// *
// * DESCRIPTION: Defines the LUT1 and LUT2 structures
// *              The command values are defined in pm_config.h
// *
// *********************************************************************************************************
//===========================================================
//
// LUT1 Structures
//
.struct struct_lut1FullCmd
    .u16    index
    .u8     flags
    .u8     operation
    .u16    bitMask
    .u16    score
    .u16    range1
    .u16    range0
    .u32    care0
    .u32    care1
.ends

.struct struct_lut1Cmd
    .u16    index
    .u8     flags
    .u8     operation
.ends

.struct struct_lut1Results
    .u16    index
    .u8     res8
    .u8     status
    .u32    rssHash
.ends

#define L1_STATUS_ENTRY_MATCH  t0   

.struct struct_lut1View
    .u32    data0
    .u32    data1
    .u32    data2
    .u32    data3
.ends

//
// Layer 2 classification (MAC, SRIO and customLUT1)
//   - Ingress 0: CDE0
//

//-------------------------------------------------------------------
//
// PDSP0 LUT1 Example Format (MAC)
//
//
// View 1
// ------
//  4: 9    - Destination Mac Addr
// 10:15    - Source Mac Addr
// 16:17    - Ethertype
// 18:19    - PPPoE Session ID
//
// View 2
// ------
// 20:23    - MPLS
// 24:35    - *reserved*
//
// View 3
// ------
//  0       - Flags                                     Data0.b3
//            bit 0     Layer2 has VLAN1 (inner)
//            bit 1     Layer2 has VLAN2 (outer)
//            bit 2     Layer2 Multicast
//            bit 3     Layer2 Broadcast
//            bit 4     Packet Includes PPPoE
//            bit 5     802.3
//            bit 6     Packet includes MPLS
//
//  1       - Destination Mac byte5                     Data0.b2
//  2: 3    - *reserved*
// 36:37    - VLAN1 ID (inner)                          Data1.w2
// 38:39    - VLAN2 ID (outer)                          Data1.w0
//  40      - packet Type                               Data2.b3
//  41      - input port                                Data2.b2
// 42:43    - VLAN1 Priority (inner)                    Data2.w0
// 44:45    - VLAN1 Priority (inner)                    Data3.w2
// 42:47    - SourceVC (Ingress Id)                     Data3.w0
//

// L2 packet types
#define fL2PktTypeMac    t7          // MAC:  0x80
#define fL2PktTypeSrio   t6          // SRIO: 0x40
#define fL2PktTypeCustom t5          // Custom: 0x20 - 0x2F (or 0x00 - 0x0F) 

// MAC 
// macFlags
#define fL2Vlan1        t0          // Layer2 has VLAN1
#define fL2Vlan2        t1          // Layer2 has VLAN2
#define fL2Mcast        t2          // Layer2 Multicast
#define fL2Bcast        t3          // Layer2 Broadcast
#define fPPPoE          t4          // When set packet contains PPPoE
#define f802p3          t5          // 802.3 packet
#define fMpls           t6          // When set packet contains MPLS

.struct struct_lut1V1_mac
    .u16    dstMac0
    .u16    dstMac1
    .u16    dstMac2
    .u16    srcMac0
    .u16    srcMac1
    .u16    srcMac2
    .u16    etherType
    .u16    PPPoE_SessionId
.ends

.struct struct_lut1V2_mac
    .u32    mpls
    .u32    res1
    .u32    res2
    .u32    res3
.ends    

.struct struct_lut1V3_mac
    .u8     pktFlags         // with bitMask
    .u8     dstMac5          // LSB of destination mac is bit maskable 
    .u16    res1
    .u16    vlanId1          // 12-bit ID of inner VLAN (0x8100) 
    .u16    vlanId2          // 12-bit ID of outer VLAN 
    .u8     pktType          // Common filed to indicate packet type (MAC, SRIO or custom 
    .u8     inPort           // One-base input EMAC port number 
    .u16    vlanPri1         // 3-bit priority of inner VLAN (0x8100) with range capability
    .u16    vlanPri2         // 3-bit priority of outer VLAN with range capability
    .u16    srcVC            // Virtual Link
.ends

// SRIO 

.struct struct_paComAddLut1SrioV2
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

// Only used for add LUT1 command, not used for classification
.struct struct_addLut1V2_srio
    .u16  nextHdrOffset  // They are nott used for matching 
    .u8   nextHdr        // place holder for nextHdr and nextOffset  
    .u8   res1
    .u32  res2
    .u32  res3
    .u32  res4  
.ends

// srioFlags
#define fSrioType9      t0          // SRIO: Message Type 9
#define fSrioType11     t1          // SRIO: Message Type 11
#define fSrioPort8      t2          // TRUE: Port 8: FALSE Port 11

.struct struct_lut1V3_srio
    .u8     pktFlags         // with valid bit mask
    .u8     typeParam2       // cos or letter 
    .u16    typeParam1       // stream ID or mailbox
    .u16    srcId
    .u16    dstId
    .u8     pktType          // SRIO 
    .u8     cc               // Completion code (not used)
    .u16    pri              // 3-bit priority with range capability
    .u16    rsv1            
    .u16    srcVC            // Virtual Link  (not used)
.ends


// Custom LUT1 
.struct struct_lut1V1_custom
    .u32    match0
    .u32    match1
    .u32    match2
    .u32    match3
.ends

.struct struct_lut1V2_custom
    .u32    match4
    .u32    match5
    .u32    match6
    .u32    match7
.ends    

.struct struct_lut1V3_custom
    .u32    res1
    .u32    res2
    .u8     pktType         // Custom type (index)
    .u8     res3            
    .u16    res4            
    .u16    res5            
    .u16    srcVC           // Virtual (Physical) Link
.ends

//
// L3/L4 Firewall, Classification and Flow Cache 
//   - Ingress 0: CDE1
//   - Ingress 1: CED2
//   - Ingress 3: CDE5 
//   - Ingress 4: CDE6 
//   - Egress  0: CDE8                                
//

// L3L4 packet types
// TBD: Use enum type
#define fL3PktTypeFirewall      t7      // Firewall Entry 0x80
#define fL3PktTypeIp            t6      // IP 0x40
#define fL3PktTypeCustom        t5      // Custom: 0x20 - 0x2F (or 0x00 - 0x0F) 
#define fL3PktTypeIpsec         t4      // IPsec entry 0x10
#define fL3PktTypeFc            t3      // Flow cache entry 0x08


// ipFlags
#define fL3Ipv4         t15         // Layer3 is IPv4
#define fGre            t14          // Contain GRE
#define fSctp           t13          // Contain SCTP
#define fL4TcpData      t12         // TCP type: 0:control; 1:Data
#define fL3IpOptions    t11         // IP options
#define fL3IpFrag       t10         // IP Fragments
#define fL3IpContainL4  t9          // First IP Fragment
#define fHopLimit       t8          // HopLimit
#define fIpsec          t7          // Contain IPSEC 
                                    // b5-b0: reserved for DSCP
 

//-------------------------------------------------------------------
//
// PDSP1 LUT1 Example Format
// (IPv4) (updated)
//
// View 1
// ------
//  4: 7    - Destination IP Addr
//  8:11    - Source IP Addr
// 12:19    - *reserved*
//
// View 2
// ------
// 20:35    - *reserved*
//
// View 3
// ------
//  0: 1   - Flags                                      Data0.w2
//            bit 15    When set, L3 = IPv4, when clear L3 = IPv6
//            bit 14:13 L4 type: 00 (UDP) 01 (TCP) 10 (SCTP) 11 (others)
//            bit 12    TCP type: 0:control; 1:data
//            bit 11    Packet contains IP options
//            bit 10    IP and fragmented
//            bit  9    First IP Fragment(??)
//            bit  8    IP TTL is 1 or 0 (??)
//            bit  7 
//            bit  6  
//            bit  5:0   DSCP 
//  2         dscp                                      Data0.b1    
//  3       - *reserved*                                Data0.b0
// 36:39    - *reserved*
//  40      - pktType                                   Data2.b3
//  41      - Protocol                                  Data2.b2
// 42:43    - L4 Source Port                            Data2.w0
// 44:45    - L4 Destination Port                       Data3.w2
// 46:47    - SourceVC (Ingress Id)
.struct struct_lut1V1_L3L4_ipv4
    .u32    dstIp
    .u32    srcIp
    .u32    res32a
    .u32    res32b
.ends

.struct struct_lut1V3_L3L4_ipv4
    .u16    pktFlags
    .u8     dscp
    .u8     res8a
    .u32    res32
    .u8     pktType          // IP or custom LUT1
    .u8     protocol
    .u16    srcPort
    .u16    dstPort
    .u16    srcVC            // Virtual Link
.ends

//-------------------------------------------------------------------
//
// PDSP1 LUT1 Example Format
// (IPv6) 
//
// View 1
// ------
//  4: 19   - Source IP Addr
//
// View 2
// ------
// 20:35    - Destination IP Addr
//
// View 3
// ------
//  0: 1   - Flags                                      Data0.w2
//            bit 15    When set, L3 = IPv4, when clear L3 = IPv6
//            bit 14:13 L4 type: 00 (UDP) 01 (TCP) 10 (SCTP) 11 (others)
//            bit 12    TCP type: 0:control; 1:data
//            bit 11    Packet contains IP options
//            bit 10    IP and fragmented
//            bit  9    First IP Fragment(??)
//            bit  8    IP HOP Limit is 1 or 0 (??)
//            bit  7 
//            bit  6  
//            bit  3:0  Priority (Class) 
//  2         dscp                                      Data0.b1    
//  3       - *reserved*                                Data0.b0
// 36:39    - Flow Label                                Data1
//  40      - pktType                                   Data2.b3
//  41      - Protocol                                  Data2.b2
// 42:43    - L4 Source Port                            Data2.w0
// 44:45    - L4 Destination Port                       Data3.w2
// 46:47    - SourceVC (Ingress Id)
.struct struct_lut1V1_L3L4_ipv6
    .u32    srcIp0
    .u32    srcIp1
    .u32    srcIp2
    .u32    srcIp3
.ends

.struct struct_lut1V2_L3L4_ipv6
    .u32    dstIp0
    .u32    dstIp1
    .u32    dstIp2
    .u32    dstIp3
.ends


.struct struct_lut1V3_L3L4_ipv6
    .u16    pktFlags
    .u8     dscp
    .u8     res8a
    .u32    flowLabel
    .u8     pktType             // IP or custom LUT1
    .u8     protocol
    .u16    srcPort
    .u16    dstPort
    .u16    srcVC               // Virtual Link
.ends

//
//  IPSEC Classification
//  - Ingress 1: CDE3
//  - Ingress 2: CDE4
//

//-------------------------------------------------------------------
//
// PDSP1 LUT1 Example Format
// (IPSEC) 
//
// View 1
// ------
//  4:19    - *reserved*
//
// View 2
// ------
// 20:35    - *reserved*
//
// View 3
// ------
//  0: 1   - Flags                                      Data0.w2
//            bit 15    When set, IPSEC ESP; clear IPSEC AH
//  2: 3    - *reserved*                                Data0.w0
// 36:39    - SPI                                       Data1
//  40      - pktType                                   Data2.b3
//  41      - *reserved*                                Data2.b2
// 42:43    - *reserved*                                Data2.w0
// 44:45    - *reserved*                                Data3.w2
// 46:47    - SourceVC (Ingress Id)                     Data3.w0

// IPSEC 
// ipsecFlags
#define fIpsecEsp       t15          // IPSEC: ESP
#define fIpsecAh        t14          // IPSEC: AH

.struct struct_lut1V3_L3L4_ipsec
    .u16    pktFlags
    .u16    res16a
    .u32    spi
    .u8     pktType             // IP or custom LUT1
    .u8     res8a
    .u16    res16b
    .u16    res16c
    .u16    srcVC               // Virtual Link
.ends

.struct struct_l1DbgEntryCntlReg
    .u16    dbgRead
    .u16    entry
.ends
#define     l1DbgCntlRegReadBit   t15

.struct struct_l1DbgEntryDataRegs12_15
    .u16    scoreEntry
    .u16    bitmaskEntry
    .u32    careField0Entry
    .u32    careField1Entry    
    .u16    range0HighEntry
    .u16    range1HighEntry    
.ends

// Define a structure that is used to write a complete entry to the LUT2
.struct struct_lut2Cmd
    .u32    cmd0
    .u32    cmd1
.ends

.struct struct_lut2Data
    .u32    data0
    .u32    data1
    .u32    data2
    .u32    data3
.ends

// TBD
.struct  struct_l2Entry
    .u32   data0
    .u32   data1
    .u32   data2
    .u32   data3
    .u32   key1
    .u32   key2
    .u32   ctrl
.ends

//TBD
.struct struct_l2Search
    .u16  srcVC        
    .u8   type
    .u8   rsvd1
    .u16  ravd2
    .u16  dstPort
    
.ends

//TBD
.struct struct_l2Search2
    .u16  srcVC        
    .u8   type
    .u8   rsvd
    .u16  data0
    .u16  data1
.ends

.struct struct_l2SCustom
    .u16  srcVC        
    .u8   type
    .u8   rsvd
    .u8   cb0
    .u8   cb1
    .u8   cb2
    .u8   cb3
.ends

#endif  // _LUT1__LUT2_H





