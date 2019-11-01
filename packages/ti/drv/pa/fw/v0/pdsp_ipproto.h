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

#ifndef _PDSP_IPPROTO_H
#define _PDSP_IPPROTO_H   1
// ******************************************************************************
// * FILE PURPOSE: IP header definitions
// ******************************************************************************
// * FILE NAME: pdsp_ipproto.h
// *
// * DESCRIPTION: Defines values used for ipv4 and ipv6 parsing
// *
// ******************************************************************************


// Ipv4
.struct struct_Ip
    .u8     VerLen
    .u8     Tos
    .u16    TotalLen
    .u16    Id
    .u16    FragOff
    .u8     Ttl
    .u8     Protocol
    .u16    Checksum
    .u32    SrcIp
    .u32    DstIp
.ends

#define IPV4_MIN_HDR_SIZE   20
#define IPV4_FRAG_MASK 0x3fff
#define t_ipv4_frag_df      t14     // Don't not flag 
#define t_ipv4_frag_m       t13     // more fragments


.struct struct_IpOpt
    .u8     opt
    .u8     len
    .u8     pointer
    .u8     rsvd
.ends

.struct struct_Ipv6a
    .u32    ver_tclass_flow
    .u16    payloadLen
    .u8     next
    .u8     hopLimit
.ends

.struct struct_Ipv6b
    .u32    srcIp_0
    .u32    srcIp_1
    .u32    srcIp_2
    .u32    srcIp_3
    .u32    dstIp_0
    .u32    dstIp_1
    .u32    dstIp_2
    .u32    dstIp_3
.ends

// IPV4 multicast
// 224.0.0.0 - 239.255.255.255
#define IPV4_MULTICAST_START    224
#define IPV4_MULTICAST_END      239

//IPV6 multicast
// FF00::/8
#define IPV6_MULTICAST_ADDR_BYTE0     0xFF
#define IPV6_MULTICAST_ADDR_BYTE1     0x00




// IPv4/IPV6 type identifiers 
#define IP_TYPE_IPV4        4
#define IP_TYPE_IPV6        6

// IPv4 options 
#define IPV4_OPT_MASK                   0x1f
#define IPV4_OPT_END_OF_LIST            0
#define IPV4_OPT_NOP                    1
#define IPV4_OPT_LOOSE_SOURCE_ROUTE     3
#define IPV4_OPT_STRICT_SOURCE_ROUTE    9


// Protocol field values (IPV4) / next header (IPV6) 
#define IP_PROTO_NEXT_IPV6_HOP_BY_HOP    0   // IPv6 extension header - hop by hop 
#define IP_PROTO_NEXT_IP_IN_IP           4   // IP tunneling 
#define IP_PROTO_NEXT_TCP                6 
#define IP_PROTO_NEXT_UDP               17
#define IP_PROTO_NEXT_IPV6_IN_IPV4      41   // IP tunneling 
#define IP_PROTO_NEXT_IPV6_ROUTE        43   // IPv6 extension header - route 
#define IP_PROTO_NEXT_IPV6_FRAG         44   // IPv6 extension header - fragmentation 
#define IP_PROTO_NEXT_GRE               47
#define IP_PROTO_NEXT_ESP               50   // Encapsulating security payload 
#define IP_PROTO_NEXT_AUTH              51   // Authentication header (ipv4) 
#define IP_PROTO_NEXT_IPV6_NO_NEXT      59   // IPv6 extention header - no next header      
#define IP_PROTO_NEXT_IPV6_DEST_OPT     60   // IPv6 extension header - destination options
#define IP_PROTO_NEXT_SCTP             132
#define IP_PROTO_NEXT_UDP_LITE         136


#define IPV6_HEADER_LEN_BYTES           40

// IPv6 hop by hop options
#define IPV6_OPT_HOP_BY_HOP_OPT_PAD0    0
#define IPV6_OPT_HOP_BY_HOP_OPT_PADN    1   
#define IPV6_OPT_HOP_BY_HOP_OPT_JUMBO   0xc2

// Fixed length IPv6 extension header options
#define IPV6_OPT_FRAG_EXTENSION_LEN_BYTES  8

.struct struct_ipv6Opt
    .u8  proto
    .u8  optlen
.ends


.struct struct_ipv6ExtRt
    .u8  proto
    .u8  hdrlen
    .u8  segsleft
    .u8  rsvd
.ends

// Least significant byte mask
#define IPV6_FRAG_MASK 0xf8
#define t_ipv6_frag_m       t0     // more fragments

.struct  struct_ipv6Frag
    .u8  proto
    .u8  rsvd1
    .u16 fragnFlag
    
    .u32 id
.ends


.struct struct_esp
    .u32  spi
    
    // seq number (.u32) not read
.ends

#define ESP_HEADER_LEN_BYTES   8


.struct struct_ah
    .u8  proto
    .u8  len
    .u16 rsvd
    
    .u32 spi
    
    .u32 seq
.ends

#define AH_HEADER_LEN32    3

#endif  // _PDSP_IPPROTO_H
