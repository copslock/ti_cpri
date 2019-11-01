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

#ifndef _PDSP_ETHPROTO_H
#define _PDSP_ETHPROTO_H   1
// **************************************************************************************
// * FILE PURPOSE: Provide ethernet header definitions
// **************************************************************************************
// * FILE NAME: pdsp_ethproto.h
// *
// * DESCRIPTION: Ethernet header definitions
// *
// **************************************************************************************
// Ethertypes 
#define ETH_TAG_VLAN            0x8100
#define ETH_TAG_SP_OUTER_VLAN   0x88a8      // 802.1ad - service provider outer vlan tag
#define ETH_TYPE_IP             0x0800
#define ETH_TYPE_IPV6           0x86dd
#define ETH_TYPE_MPLS           0x8847
#define ETH_TYPE_MPLS_MULTI     0x8848
#define ETH_TYPE_PPPoE_DISCOVER 0x8863
#define ETH_TYPE_PPPoE_SESSION  0x8864
#define ETH_TYPE_802_1AG        0x8902

// Dest/Source MAC address used during parse
.struct struct_MacAddr
    .u16    dstAddr_01
    .u16    dstAddr_23
    .u16    dstAddr_45
    .u16    srcAddr_01
    .u16    srcAddr_23
    .u16    srcAddr_45
.ends

// MAC Multicast indication bit: b0 of MSB 
#define t_eth_multicast_ind    t8

// Ethertype tag/length llc/snap used during parse
.struct struct_tagOrLen
    .u16   len
    .u8    dsap
    .u8    asap
    .u8    ctrl
    .u8    oui_0    // Organizationally unique identifier
    .u8    oui_1
    .u8    oui_2
    .u16   etype2
    .u16   rsvd
.ends

.struct  struct_vtag
    .u16   tag
    .u16   rsvd
.ends

.struct  struct_vtag2
    .u16   etherType
    .u16   tag
.ends


// *********************************************************************
// * FILE PURPOSE: Define MPLS headers
// *********************************************************************
// * FILE NAME: pdsp_mpls.h
// * 
// * DESCRIPTION: The MPLS fields are defined
// *
// *********************************************************************

.struct  struct_mpls
    .u32  tagTtl
    .u8   ipVer
    .u8   rsvd1
    .u16  rsvd2
.ends


// *********************************************************************
// * FILE PURPOSE: Define PPPoE headers
// *********************************************************************
// * FILE NAME: pdsp_pppoe.h
// * 
// * DESCRIPTION: The PPPoE fields are defined
// *
// *********************************************************************

.struct  struct_pppoe
    .u8   verType
    .u8   code
    .u16  sessionId
    .u16  len
    .u16  prot
.ends

#define PPPoE_VER_TYPE       0x11
#define PPPoE_CODE_SESSION   0
#define PPPoE_PROT_IPv4      0x0021
#define PPPoE_PROT_IPv6      0x0057  

// PPPoE header plus etherType
.struct  struct_pppoe2
    .u16  etherType
    .u8   verType
    .u8   code
    .u16  sessionId
    .u16  len
    .u16  prot
.ends


#define VLAN_PCP_SHIFT          13
#define VLAN_CFI_SHIFT          12
#define VLAN_PCP_MASK           0x07
#define VLAN_CFI_MASK           0x01
#define VLAN_VID_SHIFT          0
#define VLAN_VID_MASK           0x0FFF
#define VLAN_VID_B1_MASK        0x0F 

// Ethernet OAM 
.struct struct_eoam
     .u8 megVer
     .u8 opcode
     .u16 rsvd
.ends
#define EOAM_MEL_SHIFT          5
#define EOAM_VER_MASK           0x1F
#endif // _PDSP_ETHPROTO_H


