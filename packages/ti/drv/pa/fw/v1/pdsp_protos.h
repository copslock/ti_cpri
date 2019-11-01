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

#ifndef _PDSP_PROTOS_H
#define _PDSP_PROTOS_H  1

// **************************************************************************************
// * FILE PURPOSE: Provide srio header definitions
// **************************************************************************************
// * FILE NAME: pdsp_srio.h
// *
// * DESCRIPTION: SRIO header definitions
// *
// **************************************************************************************

// SRIO type 11
.struct struct_SrioType11
    .u16    srcId
    .u16    dstId
    .u32    ctrl
.ends

#define SRIO_TYPE11_MBOX_MASK       0x3F     // b0
#define SRIO_TYPE11_LTR_MASK        0x01C0   // w0
#define SRIO_TYPE11_LTR_SHIFT       6        // w0
#define SRIO_TYPE11_TT_MASK         0x06     // b1
#define SRIO_TYPE11_TT_SHIFT        1        // b1
#define SRIO_TYPE11_PRI_MASK        0x78     // b1
#define SRIO_TYPE11_PRI_SHIFT       3        // b1
#define SRIO_TYPE11_CC_MASK         0x0180   // w1
#define SRIO_TYPE11_CC_SHIFT        7        // w1

#define t_srio_type11_t_port16      t1      


// SRIO type 9
.struct struct_SrioType9
    .u16    srcId
    .u16    dstId
    .u16    streamId
    .u8     ctrl        // PRI, TT and CC 
    .u8     cos                 
.ends

#define SRIO_TYPE9_PRI_MASK        0x78     
#define SRIO_TYPE9_PRI_SHIFT       3        
#define SRIO_TYPE9_CC_MASK         0x03     
#define SRIO_TYPE9_CC_SHIFT        0 

#define t_srio_type9_t_port16      t2      

// ****************************************************************************************
// * FILE PURPOSE: Defines the TCP and UDP headers
// ****************************************************************************************
// * FILE NAMEP pdsp_tcp_udp.h
// *
// * DESCRIPTION:
// *
// ****************************************************************************************

.struct  struct_udp
    .u16  src
    .u16  dst
    .u16  len
    .u16  chksum
.ends

#define UDP_HEADER_LEN_BYTES        8

.struct struct_udpLite
    .u16  src
    .u16  dst
    .u16  chkCov
    .u16  chksum
.ends

#define UDP_TCP_DEST_PORT_GTP       2152

#define UDP_LITE_HEADER_LEN_BYTES   8


.struct struct_tcp
    .u16  src
    .u16  dst
    .u32  seq
    .u32  ackn
    .u8   offset_ecn
    .u8   ctrl
    .u16  window
    .u16  chksum
    .u16  urgent
.ends

#define TCP_CTRL_IND_MASK           0x07       // RST|SYN|FIN: indicate control packet
                                               // note ACK = 0 only when SYN=1

// ****************************************************************************************
// * FILE PURPOSE: Defines the ISEC NAT-T headers
// ****************************************************************************************
// * FILE NAME: ipsec_nat_t.h
// *
// * DESCRIPTION:
// *
// ****************************************************************************************
.struct  struct_ipsec_nat_t
    .u32  spi
.ends

.struct  struct_ipsec_nat_t2
    .u8   data
.ends


// ******************************************************************************
// * FILE PURPOSE: GTP header definitions
// ******************************************************************************
// * FILE NAME: pdsp_gtp.h
// *
// * DESCRIPTION: Defines values used for GTP v1 and GTP v2 parsing
// *
// ******************************************************************************


// GTP v1
.struct struct_gtp
    .u8     ctrl
    .u8     msgType
    .u16    totalLen
    .u32    teid
    .u16    seqNum
    .u8     numPdu
    .u8     nextHdr
    .u8     extHdrByte1
    .u16    pduNum
    .u8     nextHdr2
.ends

#define GTP_VER_MASK  0xE0
#define GTP_VER_SHIFT 5

// GTP Version identifiers 
#define GTP_VER_GTP_V1      (1 << GTP_VER_SHIFT)  
#define GTP_VER_GTP_V2      (2 << GTP_VER_SHIFT)

#define t_gtp_ctrl_pt_1     t4    //set: GTP1 ; clear GTP0 
#define t_gtp_ctrl_extHdr   t2
#define t_gtp_ctrl_segNum   t1
#define t_gtp_ctrl_numPdu   t0
#define GTP_CTRL_MASK 0x07        // if any of the control bits is set, the seqence number, N-PDU and next hdr is present 

#define GTPV1_HEADER_LEN_BYTES     8
#define GTPV1_EXT_HDR_LEN_BYTES    4


#define GTP_NEXTHDR_NONE           0
#define GTP_NEXTHDR_PDU_NUM        0xC0

.struct struct_gtpv2
    .u8     ctrl
    .u8     msgType
    .u16    totalLen
    .u32    teid
    .u16    seqNum
    .u16    rsvd
.ends

#define t_gtpv2_ctrl_piggyBack    t4
#define t_gtpv2_ctrl_teid         t3




#endif // _PDSP_PROTOS_H

