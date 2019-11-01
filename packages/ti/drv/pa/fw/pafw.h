// ****************************************************************************
// * FILE PURPOSE: PASS Firmware Image Definitions for the PA LLD 
// ****************************************************************************
//
// FILE NAME:   pafw.h
//
// DESCRIPTION: Define PASS firmware image related constants and data structures.  
//  The PASS contains six PDSPs wihich perform the command and packet processing. 
//  There are three PDSP images provided by the module under the ti/drv/pa/fw/v0 
//  directory:
//  -Packet Classifier 1 image: classify1_bin.c for PDSP0, PDSP1 and PDSP2
//  -Packet Classifier 2 image: classify2_bin.c for PDSP3
//  -Packet Modifier image: pam_bin.c for PDSP4 and PDSP5
//
// REVISION HISTORY:
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
#ifndef _PAFW_H
#define _PAFW_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(DEVICE_K2K) || defined(SOC_K2K)

#include <ti/drv/pa/fw/v0/pafw.h>

#elif defined(SOC_C6678)

#include <ti/drv/pa/fw/v0/pafw.h>

#elif defined(DEVICE_K2H) || defined(SOC_K2H)

#include <ti/drv/pa/fw/v0/pafw.h>

#elif defined(DEVICE_K2E) || defined(SOC_K2E)

#include <ti/drv/pa/fw/v1/pafw.h>

#elif defined(DEVICE_K2L) || defined(SOC_K2L)

#include <ti/drv/pa/fw/v1/pafw.h>

#endif /* DEVICE_XXXXX */

#ifdef __cplusplus
}
#endif


#endif  /* _PAFW_H */
