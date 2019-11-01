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

#ifndef _PARSESCOPE_H
#define _PARSESCOPE_H  1

#include "pdsp_pa.h"
//#include "pdsp_mem.h"
#include "pdsp_subs.h"
#include "pdsp_ipproto.h"
#include "pdsp_ethproto.h"
#include "pdsp_protos.h"
#include "lut1_lut2.h"
#include "pm_config.h"
#include "cde.h"

// ************************************
// * FILE PURPOSE: Defines the scopes used by the PA firmware
// **************************************************************************************************************
// * FILE NAME: parsescope.h
// *
// * DESCRIPTION: every scope used in the firmware is defined
// *
// **************************************************************************************************************/


//  Global scope C1 is active for every procedure that is used by a classify1 pdsp
    .enter   globalScopeC1
        .assign  struct_statsReg,     r31.w0, r31.w0, s_stats
        .assign  struct_c1RunContext, r29,    r29,    s_runCxt 
        .assign  struct_regFlags,     r31,    r31,    s_flags
        .assign  struct_param,        r30.w2, r30.w2, s_param
    .leave   globalScopeC1
    
// Global scope C2 is active for every procesure that is used by a classify2 pdsp    
    .enter   globalScopeC2
        .assign  struct_statsReg,     r31.w0, r31.w0, s_stats
        .assign  struct_c2RunContext, r29,    r29,    s_runCxt 
        .assign  struct_regFlags,     r31,    r31,    s_flags
        .assign  struct_param,        r30.w2, r30.w2, s_param
    .leave   globalScopeC2
    
    .enter   globalScopeM
        .assign  struct_statsReg,      r31.w0, r31.w0, s_stats
        .assign  struct_modifyContext, r29,    r29,    s_modCxt
        .assign  struct_regFlags,      r31,    r31,    s_flags
     .leave  globalScopeM

// All allocations in lut1Scope are hardware defined
    .enter  lut1Scope
        .assign   struct_lut1Results, r4,    r5,     s_l1Status    
        .assign   struct_lut1Cmd,     r4,    r4,     s_l1Cmd       
        .assign   struct_lut1FullCmd, r4,    r8,     s_l1FullCmd       
        .assign   struct_lut1View,    r6,    r9,     s_l1ViewA
        .assign   struct_lut1View,    r10,   r13,    s_l1ViewB
        .assign   struct_lut1View,    r14,   r17,    s_l1ViewC
        .assign   struct_lut1View,    r18,   r21,    s_l1ViewD
        .assign   struct_lut1V1_mac,       r6,    r9,     s_l1View1aMac
        .assign   struct_lut1V3_mac,       r10,   r13,    s_l1View3bMac
        .assign   struct_lut1V2_mac,       r14,   r17,    s_l1View2cMac
        .assign   struct_lut1V3_srio,      r10,   r13,    s_l1View3bSrio
        .assign   struct_lut1V1_custom,    r6,    r9,     s_l1View1aCustom
        .assign   struct_lut1V2_custom,    r10,   r13,    s_l1View2bCustom
        .assign   struct_lut1V3_custom,    r18,   r21,    s_l1View3dCustom
        .assign   struct_lut1V1_L3L4_ipv4, r10,   r13,    s_l1View1bIpv4
        .assign   struct_lut1V3_L3L4_ipv4, r18,   r21,    s_l1View3dIpv4
        .assign   struct_lut1V1_L3L4_ipv6, r14,   r17,    s_l1View1cIpv6
        .assign   struct_lut1V2_L3L4_ipv6, r18,   r21,    s_l1View2dIpv6
        .assign   struct_lut1V3_L3L4_ipv6, r10,   r13,    s_l1View3bIpv6
        .assign   struct_lut1V3_L3L4_ipsec,r10,   r13,    s_l1View3bIpsec
        .assign   struct_lut1V3_L3L4_ipsec,r14,   r17,    s_l1View3cIpsec
        .assign   struct_raInfo,           r16,   r17,    s_raInfo 
    .leave lut1Scope

    .enter aclLut1RescoreScope
        .assign     struct_lut1FullCmd,          r4,     r8,    s_l1RescoreCmd
        .assign     struct_lut1View,             r6,     r9,    s_l1ViewE
        .assign     struct_l1DbgEntryDataRegs12_15,  r10,    r13,    s_l1DbgEntryDataRegs12_15        
        .assign     struct_paAclRescoreHdr,     r26,    r26,    s_paAclRescoreHdr         // Classify1 ACL rescore hdr during Main Loop    
        .assign     struct_paAclRescoreData,    r27,    r27,    s_paAclRescoreData        // Classify1 ACL rescore hdr during Main Loop    
        .assign     struct_l1DbgEntryCntlReg,    r6,     r6,    s_l1DbgEntryCntlReg        
    .leave aclLut1RescoreScope
    
    .enter lut2Scope
        .assign   struct_l2Entry,     r14,   r20,    s_l2Entry
        .assign   struct_paComLut2QueueDivert, r18, r18, s_l2QueueDivert // must align with s_l2Entry.key
        .assign   struct_l2Search,    r4,    r5,     s_l2Search   // Hardware defined
        .assign   struct_l2Search2,   r4,    r5,     s_l2Search2  // Hardware defined
        .assign   struct_l2SCustom,   r4,    r5,     s_l2SCust    // Hardware defined
    .leave lut2Scope
    
// Forward match scope stores the information associated by LUT1 index
    .enter  lut1MatchScope
        .assign     struct_l1Info,              r8,   r8,   s_l1f                       // 
        .assign     struct_l1InfoVlink,         r8,   r8,   s_l1fv                      // l1f for Virtual Link
        .assign     struct_l1AclInfo,           r8,   r8,   s_l1fAcl                    // l1f for ACL entry
        .assign     struct_l1FcInfo,            r8,   r8,   s_l1fFc                     // l1f for Flow Cache entry
        .assign     struct_paForward,           r9,   r9,   s_matchForward
        .assign     struct_paForwardHost,       r10,  r12,  s_matchForward_host
        .assign     struct_paForwardEoam,       r10,  r12,  s_matchForward_eoam        
        .assign     struct_paForwardSa,         r10,  r12,  s_matchForward_sa
        .assign     struct_paForwardSrio,       r10,  r12,  s_matchForward_srio
        .assign     struct_paForwardEth,        r10,  r12,  s_matchForward_eth
        .assign     struct_paForwardPa,         r10,  r12,  s_matchForward_pa
        .assign     struct_paForwardEf,         r10,  r12,  s_matchForward_ef
        .assign     struct_rxCmdSet,            r12,  r12,  s_matchRxCmdSet             // It is part of the s_fmPlace
        .assign     struct_rxCmdHdr,            r12,  r12,  s_matchRxCmdHdr             // It is part of the s_fmPlace
        .assign     struct_rxCmdUsrStats,       r12,  r12,  s_matchRxCmdUsrStats        // It is part of the s_fmPlace 
        .assign     struct_paFwdPlace,          r9,   r12,  s_fmPlace                   // covers s_matchForward through s_matchForward_sa_sroute
        .assign     struct_paAclCfg,            r9,   r9,   s_paAclCfg                  // Align with s_matchForward
    .leave  lut1MatchScope
    
// Current packet (not held) forwarding     
    .enter currentFwdScope
        .assign     struct_paForward,           r6,   r6,   s_curFwd                    // aligns with LUT2 results
        .assign     struct_paForwardHost,       r7,   r9,   s_curFwd_host
        .assign     struct_paForwardSa,         r7,   r9,   s_curFwd_sa
        .assign     struct_paForwardSrio,       r7,   r9,   s_curFwd_srio
        .assign     struct_paForwardEth,        r7,   r9,   s_curFwd_eth
        .assign     struct_paForwardPa,         r7,   r9,   s_curFwd_pa
        .assign     struct_paForwardEf,         r7,   r9,   s_curFwd_ef
        .assign     struct_paFwdPlace,          r6,   r9,   s_curFmPlace                // covers s_matchForward through s_matchForward_sa_sroute
        .assign     struct_rxCmdSet,            r9,   r9,   s_curFwdRxCmdSet            // It is part of the s_curFmPlace
        .assign     struct_rxCmdInsert,         r9,   r9,   s_curFwdRxCmdInsert         // It is part of the s_curFmPlace
        .assign     struct_rxCmdHdr,            r9,   r9,   s_curFwdRxCmdHdr            // It is part of the s_curFmPlace
        .assign     struct_rxCmdUsrStats,       r9,   r9,   s_curFwdRxCmdUsrStats       // It is part of the s_curFmPlace 
    .leave currentFwdScope       
   
// Share by Match and Forwarding scope    
    .enter usrStatsFifoScope
        .assign     struct_fifoCb,              r19.w2, r19.w2, s_fifoCb
        .assign     struct_usrStatsReq,         r20,    r20,    s_usrStatsReq                   
    .leave usrStatsFifoScope
    
// Packet Descriptor and Packet context
    .enter  pktScope
        .assign     struct_next,                r3.b0,  r3.b0,  s_next
        .assign     struct_pktDscFine,          r6,     r13,    s_pktDescr
        .assign     struct_pktDscCpsw,          r6,     r13,    s_pktDescrCpsw
        .assign     struct_pktExtInfo,          r14,    r18,    s_pktExtDescr
        .assign     struct_pktExtInfo2,         r14,    r18,    s_pktExtDescr2
        .assign     struct_pasPktContext,       r22,    r27,    s_pktCxt           
        .assign     struct_pasPktContext2,      r22,    r27,    s_pktCxt2
        .assign     struct_pasPktContext3,      r22,    r27,    s_pktCxt3
        .assign     struct_pasPktContext4,      r22,    r27,    s_pktCxt4
        .assign     struct_pasPktContext5,      r22,    r27,    s_pktCxt5
        .assign     struct_pasPktContext6,      r22,    r27,    s_pktCxt6 
        .assign     struct_pasPktContext7,      r22,    r27,    s_pktCxt7        
        .assign     struct_pasTxPktContext,     r22,    r27,    s_txPktCxt
        .assign     struct_pasTxPktContext2,    r22,    r27,    s_txPktCxt2
        .assign     struct_pasTxPktContext3,    r22,    r27,    s_txPktCxt3
        .assign     struct_cmdHeader,           r14,    r14,    s_cmdHeader   
        .assign     struct_paIpReassmCfg,       r14,    r14,    s_paIpReassmCfg           // Classify1 Packet Main Loop
        .assign     struct_ipReassmContext,     r15,    r18,    s_ipReassmCxtInit         // Classify1 Packet Main Loop
        .assign     struct_paUsrStatsGlobCfg,   r17,    r17,    s_paUsrStatsCfg           // Classify1 EOAM route count report   
        .assign     struct_usrStatsReadContext, r18,    r18,    s_usrStatsReadCxt         // Classify1 EOAM route count report   
    .leave  pktScope
    
// CDE
    .enter cdeScope
        .assign struct_CdeCmdChk,               r4,     r5,     s_cdeCmdChk
        .assign struct_CdeCmdCrcChk,            r4,     r5,     s_cdeCmdCrcChk
        .assign struct_CdeCmdPkt,               r4,     r5,     s_cdeCmdPkt                  // Hardware defined location
        .assign struct_CdeCmdWd,                r4,     r5,     s_cdeCmdWd
        .assign struct_CdeCmd,                  r4,     r5,     s_cdeCmd
        .assign struct_CdeCmdInD,               r4,     r5,     s_cdeCmdInD
        .assign struct_CdeCmdIn,                r4,     r5,     s_cdeCmdIn
        .assign struct_CdeCmdIn2,               r4,     r5,     s_cdeCmdIn2
        .assign struct_CdeCmdPatch,             r4,     r5,     s_cdeCmdPatch
        .assign struct_CdeCmdPatchD,            r4,     r5,     s_cdeCmdPatchD
        .assign struct_CdeStatus,               r4,     r4,     s_cdeStatus
        .assign struct_CdeInsert,               r4,     r5,     s_cdeInsert
        .assign struct_CdeCrcTbl,               r4,     r5,     s_cdeCrcTbl
        .assign struct_CdeCrcCfg,               r4,     r5,     s_cdeCrcCfg
    .leave  cdeScope    
    
    // Error checking
    .enter checkScope
        .assign struct_paComMaxCount,           r2,     r2,     s_paMaxHdrCount
        .assign struct_paIpReassmCfg,           r2,     r2,     s_paOutIpReassmCfg           
        .assign struct_paIpReassmCfg,           r2,     r2,     s_paInIpReassmCfg            
        .assign struct_paCmdSetCfg,             r2,     r2,     s_paCmdSetCfg                
        .assign struct_paQueueDivertCfg,        r2,     r2,     s_paQueueDivertCfg           
    .leave checkScope
    
    // Mac/Vlan parsing
    .enter  macVlanScope
        .assign struct_MacAddr,        r6,     r8,      s_macAddr         // Must line up with LUT1 view1a
        .assign struct_tagOrLen,       r14,    r16,     s_tagOrLen
        .assign struct_tagOrLen,       r6,     r8,      s_tagOrLen2
        .assign struct_fifoCb,         r16.w0, r16.w0,  s_fifoCb          // Align the usused field within s_tagOrLen
        .assign struct_usrStatsReq,    r17,    r17,     s_usrStatsReq                   
        .assign struct_ethertypes,     r18,    r21,     s_ethertypes
        .assign struct_vtag,           r14,    r14,     s_vtag
        .assign struct_vtag2,          r6,     r6,      s_vtag2
        .assign struct_pppoe2,         r6,     r8.w2,   s_pppoe2
    .leave  macVlanScope
    
    .enter  srioScope
        .assign struct_SrioType11,     r14,    r15,     s_srio11          // Must not line up with LUT1 view1a
        .assign struct_SrioType9,      r14,    r15,     s_srio9           // Must not line up with LUT1 view1a
    .leave  srioScope
    
    .enter ipScope
        .assign struct_Ip,             r6,     r10,    s_Ip
        .assign struct_IpOpt,          r6,     r6,     s_IpOpt
        .assign struct_Ipv6a,          r6,     r7,     s_Ipv6a
        .assign struct_Ipv6b,          r14,    r21,    s_Ipv6b
        .assign struct_ipv6Opt,        r6.b3,  r6.b2,  s_Ipv6Opt
        .assign struct_ipv6ExtRt,      r6,     r6,     s_Ipv6ExtRt
        .assign struct_ipv6Frag,       r6,     r7,     s_Ipv6Frag
        .assign struct_ipTrafficFlow,  r11,    r13,    s_ipTf
        .assign struct_ipReassmContext,r14,    r17,    s_ipReassmCxt 
    .leave ipScope 

    .enter eoamScope
        .assign struct_eoam,           r6,     r6,     s_Eoam
    .leave eoamScope
    
    .enter mplsScope
        .assign struct_mpls,           r14,    r15,    s_mpls
    .leave mplsScope
    
    .enter pppoeScope
        .assign struct_pppoe,          r14,    r15,     s_pppoe
    .leave pppoeScope
    
    .enter greScope
        .assign struct_greHdr,         r6,     r6,     s_greHdr
        .assign struct_greWord,        r14,    r14,    s_greWord
    .leave greScope
    
    .enter secScope
        .assign struct_esp,            r6,     r6,     s_esp
        .assign struct_ah,             r6,     r8,     s_ah
    .leave secScope
    
    .enter sctpScope
        .assign struct_sctp,              r14,   r16,   s_sctp
        .assign struct_crcVerifyContext,  r20,   r21,   s_crcCxt
        .assign struct_crcVerifyContext,  r20,   r21,   s_sctpCrcCxt
    .leave sctpScope
    
    .enter customC1Scope
        .assign struct_paC1CustomHdr,     r14,    r15,    s_c1CustomHdr
        .assign struct_paC1Custom,        r14,    r21,    s_c1Custom
    .leave customC1Scope
    
    
    .enter  startScope
        .assign  struct_l1Info,                  r3,  r3,  s_l1Info
        .assign  struct_paForward,               r4,  r4,  s_paForward
        .assign  struct_paForward,               r8,  r8,  s_paForward2
        .assign  struct_subsLut1Map,             r3,  r10, s_l1Map
        .assign  struct_subsLut1Map2,            r11, r11, s_l1Map2   // Must follow s_l1Map
        .assign  struct_lut1FullCmd,             r3,  r7,  s_l1PendCmd   
    .leave  startScope
    
    
    .enter multiFwdScope
        .assign struct_paMultiRouteEntry,           r14,    r15,    s_paSr0            // Must align with a CDE window
        .assign struct_paMultiRouteEntry,           r16,    r17,    s_paSr1            // Must align with a CDE window
        .assign struct_paMultiRouteEntry,           r18,    r19,    s_paSr2            // Must align with a CDE window
        .assign struct_paMultiRouteEntry,           r20,    r21,    s_paSr3            // Must align with a CDE window
    .leave multiFwdScope      

//    .enter eQosModeScope
//		.assign struct_paComEQoS,               r14,    r14,    s_paComEQoS
//		.assign struct_paComIfEQoS,             r15,    r16,    s_paComIfEQoS
//		.assign struct_paComIfEQoSRouteOffset,  r17,    r20,    s_paComIfEQoSRouteOffset 	
//    .leave eQosModeScope    

// Forward Scope is used for packet forwarding. 
// Note that there is required matching with parse scope
    .enter forwardScope
        .assign     struct_CdeCmdPkt,           r4,   r5,   cdeCmdPkt                  // Hardware defined location
        .assign     struct_cmdHeader,           r14,  *,    cmdHeader                  // Used only for size info
        .assign     struct_pktDscCourse,        r14,  *,    pktDscCourse               // Used only for offset info
        //.assign     struct_pasPktContext,       r22,  r27,  pktContext                 // !!!! Must match parse scope !!!
        //.assign     struct_pasPktContext2,      r22,  r27,  pkt2                       // !!!! Must match parse scope !!!
    .leave  forwardScope


    .enter  configScope
        .assign struct_paCommand,               r6,     r9,     s_paCmd1
        .assign struct_paCmdHdr,                r10,    r10,    s_paCmdHdr                   // Must follow paCmd1
        .assign struct_paCommandConfig,         r10,    r10,    s_paCmdCfgA                  // Must follow paCmd1
        .assign struct_paComMaxCount,           r11,    r11,    s_paComMaxCount              // Must follow paCmdCfgA
        .assign struct_paIpReassmCfg,           r12,    r12,    s_paComOutIpReassm           // Must follow paComMaxCount
        .assign struct_paIpReassmCfg,           r13,    r13,    s_paComInIpReassm            // Must follow paComOutIpReassm
        .assign struct_paCmdSetCfg,             r14,    r14,    s_paComCmdSetCfg             // Must follow paComInIpReassm
        .assign struct_paUsrStatsGlobCfg,       r15,    r15,    s_paComUsrStats              // Must follow paComCmdSetCfg
        .assign struct_paQueueDivertCfg,        r16,    r16,    s_paComQueueDivert           // Must follow psComUsrStats
        .assign struct_paPktCtrlCfg,            r17,    r19,    s_paComPktCtrl               // Must follow paQueueDivert 
        .assign struct_paMacPaddingCfg,         r18,    r18,    s_paComMacPadding            // Must align  paComPktCtrl
        .assign struct_paAclCfg,                r14,    r14,    s_paComOutAcl                // First entry at the next configuration block
        .assign struct_paAclCfg,                r15,    r15,    s_paComInAcl                 // Must follow paComOutAcl
        .assign struct_paRaCfg,                 r16,    r16,    s_paComRa                    // Must follow paComInAcl
        .assign struct_paRaCfg,                 r17,    r17,    s_paComRa2                   // Must follow paComRa
        .assign struct_paQueueDivertCfg,        r18,    r18,    s_paComQueueBounce           // Must follow paComRa2
        .assign struct_eoamCfg,                 r14,    r21,    s_eoamCfg                    // First entry at the next configuration block
        .assign struct_timeAccConstants,        r15,    r17,    s_timeAccConstants           // Split up of s_eoamCfg structure
        .assign struct_eoamExceptionTbl,        r18,    r21,    s_eoamExceptionTbl           // Split up of s_eoamCfg structure
        .assign struct_timeConvConst,           r14,    r16,    s_timeConvConst              // Alias to map multiplication          
        .assign struct_paSystemConfig,          r10,    r10,    s_paCmdSysCfg                // Must follow paCmd1
        .assign struct_paComEroute,             r11,    r11,    s_paComEroute                // Must follow paCmdSysCfg
        .assign struct_paFwdPlace,              r14,    r17,    s_paComErouteFwd
        .assign struct_paC1CustomHdr,           r14,    r15,    s_paC1CustomHdr
        .assign struct_paC1Custom,              r14,    r21,    s_paC1Custom
        .assign struct_paC2Custom,              r14,    r17,    s_paC2Custom
        .assign struct_802_1ag_cfg,             r14,    r14,    s_pa802p1agDet               
        .assign struct_ipsec_nat_t_cfg,         r14,    r14,    s_paIpsecNatTDet               
        .assign struct_gtpu_cfg,                r14,    r14,    s_paGtpuCfg               
        .assign struct_pcap_hdr,                r14,    r14,    s_paPcapHdr	
        .assign struct_pcap_cfg,                r15,    r17,    s_paPktCapCfg 
        .assign struct_pcap_info,               r16,    r17,    s_paPktCapInfo
        .assign struct_paComDroute,             r14,    r14,    s_paComDroute				
		.assign struct_paComDrouteCfg,          r15,    r15,    s_paComDrouteCfg
		.assign struct_paComEQoS,               r14,    r14,    s_paComEQoS
		.assign struct_paComIfEQoS,             r15,    r16,    s_paComIfEQoS
		.assign struct_paComIfEQoSRouteOffset,  r17,    r20,    s_paComIfEQoSRouteOffset 
        .assign struct_timeStampOffsetCfg,      r14,    r15,    s_timeStampOffsetCfg
        .assign struct_paFwdPlace,              r16,    r19,    s_paComDrouteFwd	    
        .assign struct_pktDscFine,              r6,     r13,    s_pktDescr                   // Must match same value in parsescope
        .assign struct_pktDscCpsw,              r6,     r13,    s_pktDescrCpsw               // Must match same value in parsescope
        .assign struct_paComReqStats,           r10,    r10,    s_paReqStats                 // Must follow paCmd1
        .assign struct_paCommandMultiRoute,     r10,    r10,    s_paComMulti                 // Must follow paCmd1
        .assign struct_paUsrStatsCfg,           r10,    r10,    s_paComUsrStatsCfg           // Must follow paCmd1
        .assign struct_paUsrStatsGlobCfg,       r11,    r11,    s_paUsrStatsGlobCfg          // Error check at the     
        .assign struct_usrStatsClearContext,    r14,    r17,    s_paUsrStatsClrCxt           // Must start at r14 for clearBitMap        
        .assign struct_paCommandCfgCrc,         r10,    r11,    s_paComCfgCrc                // Must follow paCmd1
        .assign struct_paCommandSet,            r10,    r10,    s_paComCmdSet                // Must follow paCmd1
        .assign struct_paCmdSetCfg,             r11,    r11,    s_paCmdSetCfg                
        .assign struct_paComAddL1Hdr,           r10,    r11,    s_paAddL1Hdr                 // Must follow paCmd1
        .assign struct_paCommandDelLut1,        r10,    r10,    s_paDelL1                    // Must follow paCmd1
        .assign struct_lut1FullCmd,             r4,     r8,     s_l1PendCmd
        .assign struct_lut1Cmd,                 r4,     r4,     s_l1PendCmdS
        .assign struct_paUsrStatsEntry,         r22,    r22,    s_paComUsrStatsEntry
        .assign struct_Lut1View,                r14,    r17,    s_paAddL1Std1
        .assign struct_Lut1View,                r18,    r21,    s_paAddL1Std2
        .assign struct_Lut1View,                r14,    r17,    s_paAddL1Std3
        .assign struct_addLut1V2_srio,          r18,    r21,    s_paAddL1Srio2
        .assign struct_paCmdAddL1CmdHdr,        r6,     r9,     s_paAddL1Cmdhdr
        .assign struct_l1Info,                  r13,    r13,    s_paAddL1f                    // Must precede paFwMatchPlace
        .assign struct_l1InfoVlink,             r13,    r13,    s_paAddL1fv                   // Must precede paFwMatchPlace
        .assign struct_l1AclInfo,               r13,    r13,    s_paAddL1fAcl                 // Must precede paFwMatchPlace (L1f for ACL entry)
        .assign struct_l1FcInfo,                r13,    r13,    s_paAddL1fFc                  // Must precede paFwMatchPlace (L1f for ACL entry)
        .assign struct_paFwdPlace,              r14,    r17,    s_paFwdMatchPlace
        .assign struct_paFwdPlace,              r18,    r21,    s_paFwdNextFailPlace
        .assign struct_paForward,               r14,    r14,    s_paFwdA
        .assign struct_paForward,               r18,    r18,    s_paFwdB
        .assign struct_subsLut1Map2,            r2,     r2,     s_l1Map2
        .assign struct_paComAddLut2Standard,    r10,    r12,    s_paAddL2Std                // Must follow paCmd1
        .assign struct_paQueueDivertCfg,        r13,    r13,    s_paAddLut2QueueDivert           
        .assign struct_paComAddLut2Custom,      r10,    r12,    s_paAddL2Custom             // Must follow paCmd1
        .assign struct_paComDelLut2Standard,    r10,    r12,    s_paDelL2Std                // Must follow paCmd1
        .assign struct_paComDelLut2Custom,      r10,    r12,    s_paDelL2Custom             // Must follow paCmd1
        
    .leave  configScope

    .enter pktCaptureScope
        .assign struct_pcap_info,            r6,   r7,   s_paPktCapScr
    .leave pktCaptureScope
    
    .enter  initScope
        .assign struct_paComMaxCount,        r2,   r2,   s_paMaxHdrCountInit
        .assign struct_paIpReassmCfg,        r3,   r3,   s_paOutIpReassmInit           
        .assign struct_paIpReassmCfg,        r4,   r4,   s_paInIpReassmInit            
        .assign struct_paCmdSetCfg,          r5,   r5,   s_paCmdSetInit                
        .assign struct_paUsrStatsGlobCfg,    r6,   r6,   s_paUsrStatsInit              
        .assign struct_paQueueDivertCfg,     r7,   r7,   s_paQueueDivertInit 
        .assign struct_ipsec_nat_t_cfg,      r8,   r8,   s_paIpsecNatTDetInit   
        .assign struct_paAclCfg,             r9,   r9,   s_paOutAclInit           
        .assign struct_paAclCfg,             r10,  r10,  s_paInAclInit            
        .assign struct_paC1CustomHdr,        r8,   r9,   s_paC1CustomHdr
        .assign struct_paC1Custom,           r10,  r17,  s_paC1Custom
        .assign struct_paC2Custom,           r8,   r11,  s_paC2Custom
        .assign struct_paForward,            r8,   r8,   s_paF
        .assign struct_paForwardHost,        r9,   r11,  s_paFh
        .assign struct_headerParse,          r8,   r19,  s_headerParse
        .assign struct_nextFailRouteBase,    r8,   r15,  s_nextFailRouteBase
        .assign struct_ethertypes,           r8,   r11,  s_ethertypes
        .assign struct_paMultiRouteEntry,    r9,   r10,  s_pa1
        .assign struct_paMultiRouteEntry,    r11,  r12,  s_pa2
        .assign struct_paMultiRouteEntry,    r13,  r14,  s_pa3
        .assign struct_paMultiRouteEntry,    r15,  r16,  s_pa4
        .assign struct_paMultiRouteEntry,    r17,  r18,  s_pa5
        .assign struct_paMultiRouteEntry,    r19,  r20,  s_pa6
        .assign struct_paMultiRouteEntry,    r21,  r22,  s_pa7
        .assign struct_paMultiRouteEntry,    r23,  r24,  s_pa8
    .leave  initScope

    .enter udpScope
        .assign  struct_udp,         r6,  r7,  s_udp
        .assign  struct_udpLite,     r6,  r7,  s_udpLite
        .assign  struct_ipsec_nat_t, r8,  r8,  s_ipsecNatT
        .assign  struct_ipsec_nat_t2,r8.b3,  r8.b3,  s_ipsecNatT2
    .leave udpScope
    
    .enter tcpScope
        .assign  struct_tcp,         r6,  r10,  s_tcp
    .leave tcpScope
    
    .enter gtpScope
        .assign  struct_gtp,         r6,  r9,  s_gtp
        .assign  struct_gtpv2,       r6,  r8,  s_gtpv2
    .leave gtpScope
    
    
    .enter custC2Scope
        .assign  struct_paC2Custom,  r14,  r17,  s_c2Cust
    .leave custC2Scope
    
    .enter modifyScope
        .assign  struct_patchMsgLen,    r2,     r2,     s_patchMsgLen1  
        .assign  struct_patchMsgLen,    r3,     r3,     s_patchMsgLen2  
        .assign  struct_cmdChkCrc,      r6,     r8,     s_cmdChkCrc
        .assign  struct_blindPatch,     r6,     r10,    s_blindPatch
        .assign  struct_nextRoute,      r6,     r9,     s_nextRoute
        .assign  struct_pasPktContext,  r6,     r11,    s_mRoute
        .assign  struct_rxCmdHdr,       r6,     r6,     s_rxCmdHdr
        .assign  struct_rxCmdMultiRoute,r6,     r6,     s_rxCmdMultiRoute
        .assign  struct_rxCmdUsrStats,  r6,     r6,     s_rxCmdUsrStats
        .assign  struct_rxCmdNextRoute, r7,     r7,     s_rxCmdNextRoute
        .assign  struct_rxCmdCrcOp,     r7,     r10,    s_rxCmdCrcOp
        .assign  struct_rxCmdSplitOp,   r7,     r8,     s_rxCmdSplitOp
        .assign  struct_payloadSplitContext,   r8,     r8, s_rxCmdSplitCxt  // align with s_rxCmdSplitOp
        .assign  struct_rxCmdCopy,      r7,     r7,     s_rxCmdCopy
        .assign  struct_rxCmdPatch,     r7,     r11,    s_rxCmdPatch
        .assign  struct_rxCmdVerifyPktErr,     r7,     r9,    s_rxCmdVerifyPktErr
        .assign  struct_rxCmdContext,   r12,    r13,    s_rxCmdCxt         // Command Set only
        .assign  struct_paCmdSetCfg,    r11,    r11,    s_rxCmdSetCfg                
        .assign  struct_crcVerifyContext,       r12, r13, s_rxCrcCxt       // CRC Verification operation
        .assign  struct_crcVerifyContext,       r14, r15, s_rxCrcCxt2      // Temp buffer to record CRC Verify context 
        .assign  struct_payloadSplitContext,    r11, r11, s_rxSplitCxt  
        .assign  struct_usrStatsUpdateContext,  r6, r9,      s_rxUsrStatsUpdateCxt
        .assign  struct_usrStatsReadContext,    r6, r6,      s_usrStatsReadCxt        
        .assign  struct_paUsrStatsGlobCfg,      r10,   r10,  s_paUsrStatsCfg              
        .assign  struct_usrStatsFifoContext,    r14, r15.w2, s_rxUsrStatsFifoCxt 
        .assign  struct_fifoCb,         r15.w0, r15.w0, s_rxFifoCb
        .assign  struct_fifoCb,         r3.w0,  r3.w0,  s_crcFifoCb
        .assign  struct_fifoCb,         r3.w0,  r3.w0,  s_splitFifoCb
        .assign  struct_usrStatsReq,    r16,    r16,    s_rxUsrStatsReq
        .assign  struct_ipFragContext,  r23,    r28,    s_ipFragCxt
        .assign  struct_msg,            r22,    r22,    s_msg        // Must match s_pktCxt in pktScope
        .assign  struct_ipFragCmd,      r22,    r22,    s_ipFrag     // Must match with s_msg
        .assign  struct_patchMsgLenCmd, r22,    r22,    s_patchMsgLenCmd// Must match with s_msg
        .assign  struct_patchMsgLen,    r22,    r22,    s_patchMsgLen3  // Must match with s_msg
        .assign  struct_patchCrcCmd,    r22,    r22,    s_patchCrc  
        .assign  struct_reportTs,       r22,    r23,    s_reportTs
        .assign  struct_cmdInsEoamTime,  r22,    r22,    s_insEoamTime
        .assign  struct_cmdInsEoamCount, r22,    r23,    s_insEoamCount
        .assign  struct_modifyState,    r20,    r21,    s_modState
        .assign  struct_modifyState2,   r20,    r21,    s_modState2
        .assign  struct_nextRouteStub,  r19,    r19,    s_nRouteStub
        .assign  struct_modifyState3,   r30.w2, r30.w2, s_modState3
        .assign  struct_bPatchStub,     r6,     r6,     s_bPatchStub 
        .assign  struct_timeAcc,        r14,    r16,    s_timeCounters            // Classify1 Packet Main Loop for Time accumulation  
        .assign  struct_timeConvConst,  r14,    r16,    s_timeConvConst           // Alias to map multiplication          
    .leave modifyScope

    .enter eQosCompScope
        .assign struct_tagOrLen,       r6,    r8,     s_tagOrLen
        .assign struct_vtag,           r6,    r6,     s_vtag	
        .assign struct_Ipv4qos,        r6,    r6,     s_Ipv4qos
        .assign struct_Ipv6qos,        r6,    r6,     s_Ipv6qos
        .assign struct_pppoe,          r6,    r7,     s_pppoe
		.assign struct_mpls,           r6,    r7,     s_mpls
        .assign struct_paComIfEQoS,	   r9,    r10,    s_paComIfEQoS	
        .assign struct_ethertypes,     r14,   r17,    s_ethertypes
	    .assign struct_paEQosScratch,  r18,   r18,    s_paEQosScratch        
        //.assign struct_eqosQueueFlow,  r18,   r18,    s_eqosQueueFlow
	.leave eQosCompScope
    
    .enter efScope
        .assign  struct_efRecord1,      r11,    r13,    s_efRec1
        .assign  struct_efRecord2,      r11,    r13,    s_efRec2
        .assign  struct_efRecord3Ah,    r11,    r13,    s_efRec3Ah
        .assign  struct_efRecord3Nat,   r11,    r12,    s_efRec3Nat
        .assign  struct_efRecord4,      r11,    r13,    s_efRec4
        .assign  struct_efState1,       r19,    r21,    s_efState1
        .assign  struct_efState2,       r19,    r21,    s_efState2
        .assign  struct_efState3,       r19,    r21,    s_efState3
        .assign  struct_efState4,       r19,    r21,    s_efState4
        //.assign  struct_ipPktId,        r29,    r29,    s_ipPktId
        .assign  struct_pasEfFragInfo,  r4,     r5,     s_efFragInfo
        .assign  struct_paCfgCommand,   r4,     r5,     s_paCfgCmd
    .leave efScope

#endif // _PARSESCOPE_H
