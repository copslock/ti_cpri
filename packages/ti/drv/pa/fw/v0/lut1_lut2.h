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

.struct  struct_lut1Status
    .u16   rsvd
    .u8    index
    .u8    status
.ends

#define L1_STATUS_ENTRY_MATCH  t3

.struct struct_lut1Cmd
    .u16  careFlags
    .u8   index
    .u8   cmd
.ends

.struct struct_View1
    .u16    DstAddr_01
    .u16    DstAddr_23
    .u16    DstAddr_45
    .u16    SrcAddr_01
    .u16    SrcAddr_23
    .u16    SrcAddr_45
    .u16    EtherType
    .u16    VLAN
.ends

.struct struct_View1Srio
    .u32    rsvd1       
    .u16    srcId
    
    .u16    dstId
    .u32    rsvd2       
    
    .u16    rsvd3
    .u16    rsvd4
.ends

.struct struct_View1Rio
    .u32    rio0
    .u32    rsvd1
    .u32    rio1
    .u32    rsvd2
.ends

.struct struct_View2
    .u32    SrcIp_0
    .u32    SrcIp_1
    .u32    SrcIp_2
    .u32    SrcIp_3
    .u32    DstIp_0
    .u32    DstIp_1
    .u32    DstIp_2
    .u32    DstIp_3
.ends

.struct struct_View2Srio
    .u32    rsvd1        
    .u32    rsvd2
    .u32    rsvd3
    .u32    rsvd4        
    
    .u32    rsvd5        
    .u32    rsvd6
    .u32    rsvd7
    .u16    rsvd8
    .u16    typeParam1     // stream ID or mailbox */        
.ends


.struct struct_View3
    .u32    SPI
    .u32    FlowLabel
    .u16    SrcPort
    .u16    DstPort
    .u8     Protocol
    .u8     Tos
    .u8     IngressPort
    .u8     KeyByte
.ends

.struct struct_View3Srio
    .u32    rsvd1
    .u32    rsvd2
    .u32    rsvd3
    .u8     pri
    .u8     typeParam2     // cos or letter 
    .u8     msgType
    .u8     keyByte
.ends


.struct struct_View3mpls
    .u32    SPI
    .u32    FlowLabel
    .u32    mpls
    .u8     Protocol
    .u8     Tos
    .u8     IngressPort
    .u8     KeyByte
.ends


// Define a structure that is used to write a complete entry to the LUT
.struct  struct_l2Entry
    .u32   data0
    .u32   data1
    .u32   data2
    .u32   data3
    .u32   key
    .u32   ctrl
.ends

.struct struct_l2Search
    .u8    rsvd
    .u16   dstPort
    .u8    IdIdx
.ends

.struct struct_l2Search2
    .u32   data
.ends



.struct struct_l2SCustom
    .u8    cb0
    .u8    cb1
    .u8    cb2
    .u8    cb3
.ends



#endif  // _LUT1__LUT2_H





