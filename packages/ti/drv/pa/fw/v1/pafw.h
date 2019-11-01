// *****************************************************************************
// * FILE PURPOSE: PASS Firmware Image Definitions for the PA LLD 
// ******************************************************************************
// * FILE NAME:   pafw.h
// *
// * DESCRIPTION: Define PASS firmware image related constants and data structures.  
// *  The PASS contains 15 PDSPs wihich perform the command and packet processing. 
// *  There are 15 PDSP images provided by the module under the ti/drv/pa/fw/v1 directory:
// *  -Ingress0 PDSP0 image: pa15_in0_pdsp0_bin.c
// *  -Ingress0 PDSP1 image: pa15_in0_pdsp1_bin.c
// *  -Ingress1 PDSP0 image: pa15_in1_pdsp0_bin.c
// *  -Ingress1 PDSP1 image: pa15_in1_pdsp1_bin.c
// *  -Ingress2 PDSP0 image: pa15_in2_pdsp0_bin.c
// *  -Ingress3 PDSP1 image: pa15_in3_pdsp0_bin.c
// *  -Ingress4 PDSP0 image: pa15_in4_pdsp0_bin.c
// *  -Ingress4 PDSP1 image: pa15_in4_pdsp1_bin.c
// *  -Post PDSP0 image    : pa15_post_pdsp0_bin.c
// *  -Post PDSP1 image    : pa15_post_pdsp1_bin.c
// *  -Egress0 PDSP0 image : pa15_eg0_pdsp0_bin.c
// *  -Egress0 PDSP1 image : pa15_eg0_pdsp1_bin.c
// *  -Egress0 PDSP2 image : pa15_eg0_pdsp2_bin.c
// *  -Egress1 PDSP0 image : pa15_eg1_pdsp0_bin.c
// *  -Egress2 PDSP0 image : pa15_eg2_pdsp0_bin.c
// *
// * REVISION HISTORY:
//
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

#ifndef _PAFW_V1_H
#define _PAFW_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
/* PDSP Packet Ingress0 PDSP0 image */
extern const uint32_t in0_pdsp0[];
extern const int in0_pdsp0Size;
/* PDSP Packet Ingress0 PDSP1 image */
extern const uint32_t in0_pdsp1[];
extern const int in0_pdsp1Size;
/* PDSP Packet Ingress1 PDSP0 image */
extern const uint32_t in1_pdsp0[];
extern const int in1_pdsp0Size;
/* PDSP Packet Ingress1 PDSP1 image */
extern const uint32_t in1_pdsp1[];
extern const int in1_pdsp1Size;
/* PDSP Packet Ingress2 PDSP0 image */
extern const uint32_t in2_pdsp0[];
extern const int in2_pdsp0Size;
/* PDSP Packet Ingress3 PDSP0 image */
extern const uint32_t in3_pdsp0[];
extern const int in3_pdsp0Size;
/* PDSP Packet Ingress4 PDSP0 image */
extern const uint32_t in4_pdsp0[];
extern const int in4_pdsp0Size;
/* PDSP Packet Ingress4 PDSP1 image */
extern const uint32_t in4_pdsp1[];
extern const int in4_pdsp1Size;
/* PDSP Packet Post PDSP0 image */
extern const uint32_t post_pdsp0[];
extern const int post_pdsp0Size;
/* PDSP Packet Post PDSP1 image */
extern const uint32_t post_pdsp1[];
extern const int post_pdsp1Size;
/* PDSP Packet Egress0 PDSP0 image */
extern const uint32_t eg0_pdsp0[];
extern const int eg0_pdsp0Size;
/* PDSP Packet Egress0 PDSP1 image */
extern const uint32_t eg0_pdsp1[];
extern const int eg0_pdsp1Size;
/* PDSP Packet Egress0 PDSP2 image */
extern const uint32_t eg0_pdsp2[];
extern const int eg0_pdsp2Size;
/* PDSP Packet Egress1 PDSP0 image */
extern const uint32_t eg1_pdsp0[];
extern const int eg1_pdsp0Size;
/* PDSP Packet Egress2 PDSP0 image */
extern const uint32_t eg2_pdsp0[];
extern const int eg2_pdsp0Size;

#ifdef __cplusplus
}
#endif
  

#endif  /* _PAFW_V1_H */
