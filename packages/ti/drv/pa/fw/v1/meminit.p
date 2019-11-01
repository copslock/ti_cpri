// ***********************************************************************************************************
// * FILE PURPOSE: Perform PDSP memory intialization
// ***********************************************************************************************************
// * FILE NAME: meminit.p
// *
// * DESCRIPTION: Every PDSP has this function, but on startup the host will choose only a single
// *			  PDSP to initialize the common memory for all PDSPs.
// *
// ***********************************************************************************************************/
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
//  Register usage
// 
//   R0 - R29 General purpose
//   
//   R30  Return addres
//   R31  Flags

	.using initScope
	.using checkScope
    
#ifdef PASS_GLOBAL_INIT

f_commonInit:
  // Custom Classify 1  (40*4)
  zero  &r0,  80
  sbco  &r0,  PAMEM_CONST_CUSTOM, OFFSET_CUSTOM_C1, 80
  sbco  &r0,  PAMEM_CONST_CUSTOM, OFFSET_CUSTOM_C1 + 2*40, 80

  // Packet Capture configuration Init
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_EGRESS_PKT_CAP_CFG_BASE,  10*8
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_INGRESS_PKT_CAP_CFG_BASE, 10*8
  
  // Clear system timestamp
  sbco  &r0, PAMEM_CONST_CUSTOM,  OFFSET_SYS_TIMESTAMP,   8 

  // Clear global raw time accumulation counters to accumulate time in sec, ns
  sbco &r0,   PAMEM_CONST_CUSTOM, OFFSET_RAW_TIME_ACC,           16
  
  // Packet ingress default route configuration (64 * 8)
  mov   r20, OFFSET_DEFAULT_ROUTE_CFG_BASE
  mov   r21,  64*8
  add   r21,  r21, r20
drInit_0:  
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  add   r20, r20, 64
  qbne  drInit_0, r20,          r21

  // EQOS configuration table (1024 bytes)
  sbco  &r0, PAMEM_CONST_PORTCFG, OFFSET_EQOS_CFG_EG_DEF_PRI,    4
  mov   r20, OFFSET_EQOS_CFG_BASE
  mov   r21,  256*4
  add   r21,  r21, r20
eqosInit_0:  
  sbco  &r0, PAMEM_CONST_PORTCFG, r20,                           64
  add   r20, r20, 64
  qbne  eqosInit_0, r20,          r21
  
  // Configurable exception routing (0xFF020200)
  // All exceptions are initially configured for silent discard
  mov    r5,          0
  mov    r1,          EROUTE_N_MAX * 16
  zero  &s_paF,       SIZE(s_paF)+SIZE(s_paFh)

  mov    s_paF.forwardType,  PA_FORWARD_TYPE_DISCARD

commonInit_0:
  sbco  &s_paF, PAMEM_CONST_EROUTE, r5, SIZE(s_paF)+SIZE(s_paFh)
  add   r5,     r5,                 16
  qbne  commonInit_0, r5,           r1
  
  // Initialize global configuration parameters
  zero &s_paMaxHdrCountInit, 44
  
  // Initialize the total block to zero (replace individual initialization 
  sbco s_paMaxHdrCountInit,               PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR, 44
  
  // Initialize the max counts
  mov  s_paMaxHdrCountInit.vlanMaxCount,  2
  mov  s_paMaxHdrCountInit.ipMaxCount,    2
  mov  s_paMaxHdrCountInit.greMaxCount,   2
  sbco s_paMaxHdrCountInit,               PAMEM_CONST_CUSTOM, OFFSET_MAX_HDR, SIZE(s_paMaxHdrCountInit)
  
  // Initialize the outer IP Reassembly configuration
  //mov  s_paOutIpReassmInit.numTrafficFlow, 0
  //sbco s_paOutIpReassmInit,                PAMEM_CONST_CUSTOM, OFFSET_OUT_IP_REASSM_CFG, SIZE(s_paOutIpReassmInit)
  
  // Initialize the inner IP Reassembly configuration
  //mov  s_paInIpReassmInit.numTrafficFlow, 0
  //sbco s_paInIpReassmInit,                PAMEM_CONST_CUSTOM, OFFSET_IN_IP_REASSM_CFG, SIZE(s_paInIpReassmInit)

  // Initialize the Command Set configuration
  mov  s_paCmdSetInit.numCmdSets,         64
  mov  s_paCmdSetInit.cmdSetSize,         64
  sbco s_paCmdSetInit,                PAMEM_CONST_CUSTOM, OFFSET_CMDSET_CFG, SIZE(s_paCmdSetInit)
  
  // Initialize the User-defined Statistics configuration
  //mov  s_paUsrStatsInit.numCounters,      0
  //mov  s_paUsrStatsInit.num64bCounters,   0
  //sbco s_paUsrStatsInit,                  PAMEM_CONST_CUSTOM, OFFSET_USR_STATS_CFG, SIZE(s_paUsrStatsInit)
  
  // Initialize the Queue Diversion configuration
  //sbco s_paQueueDivertInit,               PAMEM_CONST_CUSTOM, OFFSET_QUEUE_DIVERT_CFG, SIZE(s_paQueueDivertInit)
  
  // Initialize the IPSEC NAT-T configuration
  //sbco s_paIpsecNatTDetInit,              PAMEM_CONST_CUSTOM, OFFSET_IPSEC_NAT_T_CFG,  SIZE(s_paIpsecNatTDetInit)
  
  // Initialize the outer ACL configuration
  mov  s_paOutAclInit.action,    PA_FORWARD_TYPE_PA
  sbco s_paOutAclInit,           PAMEM_CONST_CUSTOM, OFFSET_OUT_IP_ACL_CFG, SIZE(s_paOutAclInit)
  
  // Initialize the inner ACL configuration
  mov  s_paInAclInit.action,     PA_FORWARD_TYPE_PA
  sbco s_paInAclInit,            PAMEM_CONST_CUSTOM, OFFSET_IN_IP_ACL_CFG, SIZE(s_paInAclInit)

  // Initialize the Queue Bounce configuration
  // No default value is required, clear to zero

  ret
  
#endif

	.leave checkScope
	.leave initScope
