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
#ifndef _CDE_H
#define _CDE_H   1

//// ********************************************************************************************************
////  FILE PURPOSE: Define CDE data structures
//// ********************************************************************************************************
////  FILE NAME: cde.h
//// 
////  DESCRIPTION: Defines the data structures used to drive the CDE
//// 
//// ********************************************************************************************************

// The command structure

.struct struct_CdeCmdPkt
	.u8		psInfoSize
	.u8		threadId
	.u8     optionsFlag
	.u8		operation

	.u16	destQueue
	.u8		rsvd
	.u8	    flowId
.ends

.struct struct_CdeCmdWd
    .u16    byteCount
    .u8     rsvd
    .u8     operation
    
    .u32    dataPointer
.ends

.struct struct_CdeCmdChk
    .u16    byteLen
    .u8     options
    .u8     operation
    
    .u16    initSum
    .u16    offset
.ends

.struct struct_CdeCmdCrcChk
    .u16    byteLen
    .u8     options
    .u8     operation
    
    .u16    rsvd
    .u16    offset
.ends

#define t_cde_crc_cmd_option_crcOffset      t0
#define t_cde_crc_cmd_option_FromDesc       t1

.struct struct_CdeCmd
    .u32    v0
    .u32    v1
.ends


.struct struct_CdeCmdInD
    .u16    lenMsbs
    .u8     lenLsb
    .u8     operation
    
    .u32    dataP
.ends

.struct struct_CdeCmdIn
    .u16    rsvd
    .u8     len
    .u8     operation
    
    .u8     data3
    .u8     data2
    .u8     data1
    .u8     data0
.ends

.struct struct_CdeCmdIn2
    .u16    rsvd
    .u8     len
    .u8     operation
    
    .u32    data
.ends


.struct struct_CdeCmdPatch
    .u16    offset
    .u8     len
    .u8     operation
    
    .u32    data
.ends

.struct struct_CdeStatus
    .u16    offset
    .u8     rsvd
    .u8     type
.ends
    
.struct struct_CdeInsert
    .u16    rsvd
    .u8     byteCount
    .u8     operation
    
    .u32    bytes
.ends

.struct struct_CdeCrcTbl
    .u16    rsvd
    .u8     index
    .u8     operation
    
    .u32    value
.ends

.struct struct_CdeCrcCfg
    .u16    rsvd
    .u8     flags
    .u8     operation
    
    .u32    initVal
.ends




#endif // _CDE_H
