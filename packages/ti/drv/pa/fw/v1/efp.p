// ************************************************************************************************
// * FILE PURPOSE: Egress Flow Processing on PDSPs without LUTs
// ************************************************************************************************
// * FILE NAME: efp.p
// *
// * DESCRIPTION: Egress Flow modification record processing:
// *              Record 1: Inner IP/UDP/TCP Modification
// *              Record 2: Outer IP insertion, IPSEC ESP/AH Pre-processing and Inner IP fragmentation
// *              Record 3: IPSEC AH Pre-processing or NAT-T header insertion
// *              Record 4: Layer 2 Header insertion/replacement and Outer IP fragmentation
// *
// ************************************************************************************************
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
#ifdef PASS_PROC_EF_REC

#define PDSP_EFP  1

#include "pdsp_pa.h"
#include "pdsp_mem.h"
#include "pdsp_mem2.h"
#include "pdsp_subs.h"
#include "pm_config.h"
#include "pdsp_ver.h"
#include "pm_constants.h"
#include "parsescope.h"
    
.using  globalScopeM
.using  efScope

// *********************************************************************************************
// * FUNCTION PURPOSE: Egress flow packet forwarding
// *********************************************************************************************
// * DESCRIPTION: The current packet is forwarded based on the forward information
// *              input.
// *    On Entry:
// *        R6-R9  has the forward match structure
// *        r26 has the packet ID
// *        r3 has the statistic to update on silent discard (can be used for
// *           classify1 or classify2)
// *
// *   Register Usage:  
// * 
// *   R0:   scratch
// *   R1:   scratch 
// *   R2:   scratch 
// *   R3:       |  bit mask for stat to update on silent discard
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:          |
// *   R7:          |  Current Packet Forwarding info
// *   R8:          |
// *   R9:          |
// *   R10:            
// *   R11:     
// *   R12:     
// *   R13:                                                                    -
// *   R14:          |                                          
// *   R15:          |  Extended Header Info                                        
// *   R16:          |                                          
// *   R17:          |  
// *   R18:          |
// *   R19:            | usrStats FIFO CB
// *   R20:            | usrStats Request
// *   R21:
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |  
// *   R27:     |  
// *   R28:  
// *   R29:  
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ********************************************************************************************
    .using currentFwdScope
    .using pktScope
    .using cdeScope
    .using usrStatsFifoScope

f_efPktForward:
    wbs   s_flags.info.tStatus_CDEOutPacket
    lbco  r0.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)
#ifdef PASS_LAST_PDSP
    // Clear Bypass Flag
    clr   r0.w0.t_pktBypass
#else
    set   r0.w0.t_pktBypass
#endif
    sbco  r0.w0,  cCdeOutPkt, OFFSET(s_pktDescr.pktFlags),  SIZE(s_pktDescr.pktFlags)

    // Record the effective packet size for user statistics update
    mov   s_usrStatsReq.pktSize,  s_txPktCxt.endOffset  

l_efPktForward1:

    // Forwarding packets to the host
    qbne  l_efPktForward3,  s_curFwd.forwardType,  PA_FORWARD_TYPE_HOST

        // Patch swinfo0
        sbco s_curFwd_host.context,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  4
        
        // Patch the psflags only if non-zero
        qbeq l_efPktForward1_0,    s_curFwd_host.psflags,  0 
            lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
            and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
            and  r0.b0, s_curFwd_host.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
            or   r2.b0, r2.b0, r0.b0
            sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
            // Set EMAC output port
            and  r0.b0, s_curFwd_host.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
            sbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
            // TBD: Clear TimeSync word

l_efPktForward1_0:
        mov     s_pktExtDescr.threadId, THREADID_CDMA0
        
l_efPktForward1_1:        
        
l_efPktForward1_1_queue_bounce:
        // Check for Queue Bounce operation
l_efPktForward1_1_queue_bounce_ddr:
        qbbc l_efPktForward1_1_queue_bounce_msmc, s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            clr s_curFwd.queue.t_pa_forward_queue_bounce_ddr
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG, 2
            jmp  l_efPktForward1_1_queue_bounce_end

l_efPktForward1_1_queue_bounce_msmc:
        qbbc l_efPktForward1_1_queue_bounce_end, s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            clr s_curFwd.queue.t_pa_forward_queue_bounce_msmc
            sbco s_curFwd.queue,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo1) + 2,  2
            lbco s_curFwd.queue,  PAMEM_CONST_CUSTOM, OFFSET_QUEUE_BOUNCE_CFG+2, 2
            // pass through

l_efPktForward1_1_queue_bounce_end:

        // Send the packet on its way
        //   Free the packet Ext Info
        xout    XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)   
       
        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8)
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        //sbco s_curFwd.queue,          cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        xout XID_CDECTRL,             s_cdeCmdPkt,          SIZE(s_cdeCmdPkt)
        qbeq l_efPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_efPktForward3:
    and   r2.b0, s_curFwd.forwardType, PA_FORWARD_TYPE_MASK
    qbne  l_efPktForward3_1, r2.b0, PA_FORWARD_TYPE_SA
    
        //   Record ThreadId  (SA through queue)
        qbbs l_efPktForward3_set_threadId_1, s_curFwd.forwardType.t_pa_fwd_type_ctrl_use_loc_dma
            mov  s_pktExtDescr.threadId, THREADID_CDMA0
            jmp  l_efPktForward3_2
l_efPktForward3_set_threadId_1:
            mov  s_pktExtDescr.threadId, THREADID_CDMA1
            jmp  l_efPktForward3_2   
        
l_efPktForward3_1:
    qbne  l_efPktForward4, r2.b0, PA_FORWARD_TYPE_SA_DIRECT
            // To SA through PASS DMA 
            mov  s_pktExtDescr.threadId,  s_curFwd.flowId

l_efPktForward3_2:
    
        // Routing to SA
        // Patch swinfo0 and swinfo2
        sbco s_curFwd_sa.swInfo0,  cCdeOutPkt, OFFSET(s_pktDescr.swinfo0),  8
        
l_efPktForward3_3:        
        // Send the packet on its way
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        
        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8)
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8   // Round up to multiple of 8 bytes
        mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        //sbco s_curFwd.queue,          cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        qbeq l_efPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret

l_efPktForward4:
    qbne  l_efPktForward5, s_curFwd.forwardType, PA_FORWARD_TYPE_SRIO
        // Routing to SRIO
        // Overwrite Packet Info with psinfo0 and psinfo2
        sbco s_curFwd_srio.psInfo0,  cCdeOutPkt, SIZE(s_pktDescr),  8
        
        // pactch the packet type
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_TYPE_MASK
        lsl  r2.b1, s_curFwd_srio.pktType, PA_PKT_TYPE_SHIFT
        or   r2.b0, r2.b0,  r2.b1
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), 1
        
        // Send the packet on its way
        //   Free the packet Ext Info
        mov  s_pktExtDescr.threadId, THREADID_CDMA0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        
        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID | CDE_FLG_SET_DESTQUEUE) << 8) 
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  8                       // SRIO take 8 bytes Ps Info
        //mov  s_cdeCmdPkt.threadId,  PA_DEST_CDMA
        mov  s_cdeCmdPkt.destQueue,   s_curFwd.queue
        mov  s_cdeCmdPkt.flowId,      s_curFwd.flowId
        //sbco s_curFwd.queue,          cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        xout XID_CDECTRL,             s_cdeCmdPkt,                           SIZE(s_cdeCmdPkt)
        ret

l_efPktForward5:
    qbne  l_efPktForward6, s_curFwd.forwardType, PA_FORWARD_TYPE_ETH
        // Routing to ETH
        // Clear psflags
        lbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r2.b0, r2.b0, NOT_PA_PKT_PS_FLAGS_MASK
        and  r0.b0, s_curFwd_eth.psflags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
        or   r2.b0, r2.b0, r0.b0
        sbco r2.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
        // Set EMAC output port
        and  r0.b0, s_curFwd_eth.psflags, PAFRM_ETH_PS_FLAGS_PORT_MASK
        sbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
        // Clear TimeSync word
        mov  r0, 0
        sbco r0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.tsWord), 4
        
        // Send the packet on its way
        add  s_pktExtDescr.threadId,  s_curFwd_eth.priority, THREADID_ETHERNET1
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)  
        
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8)
        mov  s_cdeCmdPkt.psInfoSize,  0
        xout XID_CDECTRL,             s_cdeCmdPkt,            SIZE(s_cdeCmdPkt)
        qbeq l_efPktForward9,        s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret


l_efPktForward6:
    qbeq  l_efPktForward7,  s_curFwd.forwardType, PA_FORWARD_TYPE_DISCARD

        // Invalid match type in table - this should never happen
        // Inc the stat and generate an event if enabled
        //mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL
        
        // Record Error and then pass through                  

l_efPktForward7: 
        // Do a silent discard, release the packet ID
        // mov s_stats.value,  r3.w0 
        
#ifdef PASS_LAST_PDSP
        mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_FLUSH
        set  s_pktExtDescr.flags.fDropped
#else
        mov   s_cdeCmdPkt.operation, CDE_CMD_PACKET_ADVANCE
        mov   s_cdeCmdPkt.optionsFlag, 0
        set  s_pktExtDescr.flags.fDroppedInd
#endif        
        //   Free the packet Ext Info
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr) 
         
        xout  XID_CDECTRL,       s_cdeCmdPkt,          SIZE(s_cdeCmdPkt)
        qbeq  l_efPktForward9,  s_curFwdRxCmdHdr.cmd, PA_RX_CMD_USR_STATS
        ret
        
l_efPktForward9:   
        // User Statistics operation:
        // Record and insert the statistics update into the FIFO
        // Note all user commands are in the same location (r12)
#ifdef EFP_TBD        
        mov   s_usrStatsReq.index,  s_curFwd_host.cmd.w0
        
        lbco  s_fifoCb, PAMEM_CONST_USR_STATS_FIFO_BASE,  OFFSET_PDSP_USR_STATS_FIFO_CB, SIZE(s_fifoCb)
        add   r1.b0,    s_fifoCb.in, 4
        and   r1.b0,    r1.b0,       0x1F
        
        qbne  l_efPktForward9_1, s_fifoCb.out, r1.b0
            // FIFO is full, bump the system error
            mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_C1_SYSTEM_FAIL
            ret

l_efPktForward9_1:
            // Insert the request into the FIFO   
            add   r1.w2,   s_fifoCb.in, OFFSET_PDSP_USR_STATS_FIFO
            sbco  s_usrStatsReq,  PAMEM_CONST_USR_STATS_FIFO_BASE, r1.w2, SIZE(s_usrStatsReq)
            sbco  r1.b0,   PAMEM_CONST_USR_STATS_FIFO_BASE,    OFFSET_PDSP_USR_STATS_FIFO_CB + OFFSET(s_fifoCb.in), SIZE(s_fifoCb.in)  
#endif               
            ret

    .leave currentFwdScope
    .leave pktScope
    .leave cdeScope
    .leave usrStatsFifoScope
    
// ********************************************************************************************
// * FUNCTION PURPOSE: Forward a packet based on the exception index information
// ********************************************************************************************
// * DESCRIPTION: The error index is used to extract the routing information and forward
// *              the packet
// *
// *   Register Usage:  
// * 
// *   R0:
// *   R1:    
// *   R2:    
// *   R3:       |  bit mask for stat to update on silent discard
// *   R4:    
// *   R5:    
// *   R6:          |
// *   R7:          |  Current Packet Forwarding info
// *   R8:          |
// *   R9:          |
// *   R10:     
// *   R11:     
// *   R12:     
// *   R13:
// *   R14:          |  (packet extended descriptor)                                        
// *   R15:          |                                          
// *   R16:          |                                          
// *   R17:          |  
// *   R18:          |                                  
// *   R19:
// *   R20:
// *   R21:          
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     |
// *   R28:  
// *   R29:  c2RunContext (s_runCxt)                                -  Global Scope
// *   R30:  w2-Error Index w0-function return address              -
// *   R31:  System Flags (s_flags)                                 -
// *
// ****************************************************************************************************

    .using pktScope
    .using currentFwdScope
          
f_efForwardExp:

    // Get the error index, multiply by 16 to get the memory offset
    // Load the error routing information
    lsl   r0.w2,  s_txPktCxt2.eId,     4

    lbco  s_curFmPlace,  PAMEM_CONST_EF_EROUTE,  r0.w2,  SIZE(s_curFmPlace)
    
    // Restore the latest extended Packet Info
    xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    // Need a return address
    mov    r30.w0, f_mainLoop
    jmp    f_efPktForward

    .leave pktScope
    .leave currentFwdScope

    
    .using pktScope     // For offset info
    
// Exception IP expire      
f_paEfProc_ip_expire:
    mov  s_txPktCxt2.eId,   EF_EROUTE_IP_EXPIRE
    jmp  f_efForwardExp 
    
    .leave pktScope
    
f_paEfRecProc:    

#ifdef PASS_PROC_EF_REC1

// *******************************************************************************
// * FUNCTION PURPOSE: Record 1 Processing
// *******************************************************************************
// * DESCRIPTION: Process the egress flow record 0 
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:      | Temp and local variables
// *   R2:      | 
// *   R3:      | 
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |        
// *   R7:        |  L3/L4 Header                            
// *   R8:        |                             
// *   R9:        |  
// *   R10:       |  
// *   R11:         |  
// *   R12:         |  Record Info
// *   R13:         |   
// *   R14:            |                                            
// *   R15:            | Extended Descriptor                                           
// *   R16:            |                                            
// *   R17:            |                         
// *   R18:            | 
// *   R19:       |     
// *   R20:       |  Record State machine (s_recState)  | s_modState will be overwritten 
// *   R21:       |                                     | for EF Record processing
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:           |w0: record1 offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    .using cdeScope
    .using pktScope     // For offset info
    .using ipScope
        
f_paEfRec1Proc:

    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,                4
    
    qbbs  l_paEfRec1Proc_1, s_txPktCxt.flags.t_flag_ef_rec_valid_lvl1
        // use default record: no modification
        zero    &s_efRec1,  SIZE(s_efRec1)
        jmp     l_paEfRec1Proc_2
    
l_paEfRec1Proc_1:
    // Calculate and record the record1 offset
    lsl  r0.w0,     s_txPktCxt.lvl1RecIndex,   5
    lsl  r0.w2,     s_txPktCxt.lvl1RecIndex,   4  
    
    add  r28.w0,     r0.w0, r0.w2       
    lbco s_efRec1,   PAMEM_CONST_EF_RECORD1,    r28.w0,     SIZE(s_efRec1)
    
    // Check whether we need to remove outer IP header
    qbbc    l_paEfRec1Proc_2,   s_efRec1.ctrlFlags.t_ef_rec1_strip_outer_ip
    qbeq    l_paEfRec1Proc_2,   s_txPktCxt.l3Offset, s_txPktCxt.l3Offset2   
        // Move to outer IP header  */
        // Advance the CDE to the IP header to examine in the packet
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset
        xout XID_CDECTRL,           s_cdeCmdWd,     4
        
        // Flushout outer IP header
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        sub  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset2, s_txPktCxt.l3Offset
        xout XID_CDECTRL,           s_cdeCmdWd,     4
        
        mov  s_txPktCxt.l3Offset2,  s_txPktCxt.l3Offset
        sub  s_pktExtDescr.sopLength,  s_pktExtDescr.sopLength, s_cdeCmdWd.byteCount
        jmp  l_paEfRec1Proc_3
    
l_paEfRec1Proc_2:
    // Move to inner IP header  */
    // Advance the CDE to the IP header to examine in the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset2
    xout XID_CDECTRL,           s_cdeCmdWd,     SIZE(s_cdeCmdWd)
    
l_paEfRec1Proc_3:

    // Read the minimum IP header
    xin  XID_CDEDATA,  s_Ip,   SIZE(s_Ip) 
    and  r0.b0,  s_Ip.verLen,  0xF0
    
    qbeq l_paEf1ParseIpv4,  r0.b0,       0x40
    qbeq l_paEf1ParseIpv6,  r0.b0,       0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
l_paEf1ParseIpv4:
    // TBD: Error check
    zero  &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
    and   r2.w0,                s_Ip.verLen,              0x0f
    lsl   s_cdeCmdChk.byteLen,  r2.w0,                    2  // 32 bit size to 8 bit size conversion
    
    // Exception Check
l_paEf1ParseIpv4_exp_check:    
    qbbc l_paEf1ParseIpv4_exp_check_1, s_efRec1.ctrlFlags.t_ef_rec1_exp_ip_expire
        qbeq f_paEfProc_ip_expire,  s_Ip.ttl,  0  
        // pass through
        
l_paEf1ParseIpv4_exp_check_1:        
    qbbc l_paEf1ParseIpv4_exp_check_2, s_efRec1.ctrlFlags.t_ef_rec1_exp_ip_frag
        mov   r0.w0,    IPV4_FRAG_MASK
        and   r0.w0,    s_Ip.fragOff,     r0.w0
        qbeq  l_paEf1ParseIpv4_exp_check_2,  r0.w0,            0
            // IPv4 fragmentation
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_FRAG
            jmp  f_efForwardExp                                // no return
        
l_paEf1ParseIpv4_exp_check_2: 
    qbbc l_paEf1ParseIpv4_exp_check_end, s_efRec1.ctrlFlags.t_ef_rec2_exp_ip_options
        and  r0.b0,  s_Ip.verLen,  0x0F
        qbeq l_paEf1ParseIpv4_exp_check_end, r0.b0, 5
            // IPv4 options exists
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_OPTIONS
            jmp  f_efForwardExp                                // no return
    
l_paEf1ParseIpv4_exp_check_end:    
    
    // Update parameters
    qbbc l_paEf1ParseIpv4_update_1, s_efRec1.ctrlFlags.t_ef_rec1_ip_ttl_update
        // TTL check 
        qbeq l_paEf1ParseIpv4_update_1,  s_Ip.ttl,  0  
            sub s_Ip.ttl, s_Ip.ttl, 1
            // pass through    
        
l_paEf1ParseIpv4_update_1:
    qbbc l_paEf1ParseIpv4_update_2, s_efRec1.ctrlFlags.t_ef_rec1_ip_src_addr
        add r0.w0,  r28.w0, PA_EF_REC1_SRC_IP_OFFSET
        lbco    s_Ip.srcIp, PAMEM_CONST_EF_RECORD1,    r0.w0,     SIZE(s_Ip.srcIp)
        // pass through
        
l_paEf1ParseIpv4_update_2:
    qbbc l_paEf1ParseIpv4_update_3, s_efRec1.ctrlFlags.t_ef_rec1_ip_dst_addr
        add r0.w0,  r28.w0, PA_EF_REC1_DST_IP_OFFSET
        lbco    s_Ip.dstIp, PAMEM_CONST_EF_RECORD1,    r0.w0,     SIZE(s_Ip.dstIp)
        // pass through
        
l_paEf1ParseIpv4_update_3:
    qbbc l_paEf1ParseIpv4_update_end, s_efRec1.ctrlFlags.t_ef_rec1_ip_tos_class
        mov    s_Ip.tos, s_efRec1.tos
        // pass through
        
l_paEf1ParseIpv4_update_end: 
    // Record DSCP value 
    lsr s_txPktCxt3.dscp,   s_Ip.tos,   2   
 
    qbbc l_paEf1ParseIpv4_1, s_efRec1.ctrlFlags.t_ef_rec1_ipv4_cksum
        mov   s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
        mov   s_cdeCmdChk.offset, OFFSET(s_Ip.checksum)          // Starting sum is 0, offset is 10
        xout  XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
  
        mov   s_Ip.checksum, 0
        //xout  XID_CDEDATA, s_Ip.Checksum, 2
        // pass through
         
l_paEf1ParseIpv4_1:
    // update IP header
    xout  XID_CDEDATA, s_Ip, SIZE(s_Ip)

    // record the next protocol
    mov    s_efState1.proto,   s_Ip.protocol

    // Compute the pseudo header checkusm except protocol and payload length
    add    r1,                 s_Ip.srcIp,       s_Ip.dstIp
    adc    r1,                 r1.w0,            r1.w2
    add    s_efState1.pseudo,  r1.w0,            r1.w2
    
    // Pre-calculate the IPv4 fragmentation
    mov    s_txPktCxt.nextHdr,  s_efState1.proto 
    mov    s_txPktCxt.ipHdrLen, s_cdeCmdChk.byteLen
    mov    s_txPktCxt.fragSize, s_Ip.totalLen  
    mov    s_txPktCxt.lastFragSize, s_txPktCxt.fragSize  
    qbbc   l_paEf1ParseIpv4_end, s_efRec1.ctrlFlags.t_ef_rec1_ip_mtu
    qbge   l_paEf1ParseIpv4_end, s_Ip.totalLen,   s_efRec1.mtu 
    // TBD: non-fragmentable packet?  
        // Inner IP fragmentation is required 
        set s_txPktCxt.flags.t_flag_ip_frag
        // local variable: payloadSize = r0.w0
        //                 ipLen = r0.w2
        sub r0.w0, s_efRec1.mtu,   s_txPktCxt.ipHdrLen
        and r0.b0, r0.b0, 0xF8
        sub r0.w2, s_Ip.totalLen,  s_txPktCxt.ipHdrLen
        add s_txPktCxt.fragSize,   r0.w0, s_txPktCxt.ipHdrLen  
l_paEf1ParseIpv4_fragment_loop:
        //add  s_txPktCxt.numFrags, s_txPktCxt.numFrags, 1
        sub  r0.w2, r0.w2, r0.w0
        qblt l_paEf1ParseIpv4_fragment_loop, r0.w2, r0.w0  
            // last fragment
            add s_txPktCxt.lastFragSize, r0.w2, s_txPktCxt.ipHdrLen
            
            // end of the loop
                    
l_paEf1ParseIpv4_end:
    // TBD: Adjust Offsets
    //add   s_txPktCxt.startOffset,   s_txPktCxt.l3Offset2,   s_txPktCxt.ipHdrLen
    add  s_txPktCxt.l4Offset,    s_txPktCxt.l3Offset2,   s_txPktCxt.ipHdrLen
    add  s_txPktCxt.endOffset,   s_txPktCxt.l3Offset2,   s_Ip.totalLen
          
    //  Advance past the IP header.
    mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov   s_cdeCmdWd.byteCount,   s_txPktCxt.ipHdrLen
    xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
    jmp   l_paEf1ParsePostIp
    
l_paEf1ParseIpv6:  
    // IPv6 Error check
    
    // Record IPv6 related information
    mov    s_efState1.proto,   s_Ipv6a.next
    mov    s_efState1.payloadLen, s_Ipv6a.payloadLen
    
    // Adjust the endOffset
    add  s_txPktCxt.endOffset,   s_txPktCxt.l3Offset2,   s_Ipv6a.payloadLen
    add  s_txPktCxt.endOffset,   s_txPktCxt.endOffset,   40
    
    // Exception Check
l_paEf1ParseIpv6_exp_check:    
    // Exception check group 1
    qbbc l_paEf1ParseIpv6_update, s_efRec1.ctrlFlags.t_ef_rec1_exp_ip_expire
        qbeq f_paEfProc_ip_expire,  s_Ipv6a.hopLimit,  0  
        // pass through
        
l_paEf1ParseIpv6_update:
    // Update parameters
    qbbc l_paEf1ParseIpv6_update_1, s_efRec1.ctrlFlags.t_ef_rec1_ip_ttl_update
        // TTL check 
        qbeq l_paEf1ParseIpv6_update_1,  s_Ipv6a.hopLimit,  0  
            sub s_Ipv6a.hopLimit, s_Ipv6a.hopLimit, 1
            // pass through 
        
l_paEf1ParseIpv6_update_1:
    qbbc l_paEf1ParseIpv6_update_2, s_efRec1.ctrlFlags.t_ef_rec1_ip_tos_class
        and    s_Ipv6a.ver_tclass_flow.b3, s_Ipv6a.ver_tclass_flow.b3, 0xF0
        lsr    r0.b3,   s_efRec1.tos, 4
        or     s_Ipv6a.ver_tclass_flow.b3, s_Ipv6a.ver_tclass_flow.b3, r0.b3
        lsl    r0.b2,   s_efRec1.tos, 4
        and    s_Ipv6a.ver_tclass_flow.b2, s_Ipv6a.ver_tclass_flow.b2, 0x0F 
        or     s_Ipv6a.ver_tclass_flow.b2, s_Ipv6a.ver_tclass_flow.b2, r0.b2
        // pass through
        
l_paEf1ParseIpv6_update_2:
    qbbc l_paEf1ParseIpv6_update_3, s_efRec1.ctrlFlags.t_ef_rec1_ip_flow_label
        mov    s_Ipv6a.ver_tclass_flow.w0,  s_efRec1.flowLableLo
        and    s_Ipv6a.ver_tclass_flow.b2,  s_Ipv6a.ver_tclass_flow.b2, 0xF0 
        or     s_Ipv6a.ver_tclass_flow.b2,  s_Ipv6a.ver_tclass_flow.b2, s_efRec1.flowLablelHi
        // pass through
        
l_paEf1ParseIpv6_update_3:
    xout   XID_CDEDATA,  s_Ipv6a,   SIZE(s_Ipv6a)
    
    // record the updated class value as DSCP
   lsr   s_txPktCxt3.dscp,   s_Ipv6a.ver_tclass_flow.w2,  4
   and   s_txPktCxt3.dscp,   s_txPktCxt3.dscp,            0x3F
    
    //  Advance past the fisr 8 byte (Ipv6a)
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,   SIZE(s_Ipv6a)                      // Bytes always present in the header
    xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
   
    // Load IPv6 source address
    xin  XID_CDEDATA,  r6,   16

    qbbc l_paEf1ParseIpv6_update_4, s_efRec1.ctrlFlags.t_ef_rec1_ip_src_addr
        add r0.w0,  r28.w0, PA_EF_REC1_SRC_IP_OFFSET
        lbco   r6, PAMEM_CONST_EF_RECORD1,    r0.w0,     16
        xout   XID_CDEDATA,  r6,   16
        // pass through
        
l_paEf1ParseIpv6_update_4:
    // update pseudo checksum
    add  r0, r6, r7
    adc  r1, r8, r9
    adc  r0, r0, r1
    adc  s_efState1.pseudo, r0.w0, r0.w2
    adc  s_efState1.pseudo, s_efState1.pseudo, 0  
    
    //  Advance past source IP.
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,   16    
    xout XID_CDECTRL,    s_cdeCmdWd,   4
              
    // Load IPv6 destination address
    xin  XID_CDEDATA,  r6,   16
        
    // pass through
    qbbc l_paEf1ParseIpv6_update_5, s_efRec1.ctrlFlags.t_ef_rec1_ip_dst_addr
        add r0.w0,  r28.w0, PA_EF_REC1_DST_IP_OFFSET
        lbco   r6, PAMEM_CONST_EF_RECORD1,    r0.w0,     16
        xout   XID_CDEDATA,  r6,   16
        
        // pass through
        
l_paEf1ParseIpv6_update_5:      
    // update pseudo checksum
    add  r0, r6, r7
    adc  r1, r8, r9
    adc  r0, r0, r1
    adc  s_efState1.pseudo, s_efState1.pseudo, r0.w0
    adc  s_efState1.pseudo, s_efState1.pseudo, r0.w2
    adc  s_efState1.pseudo, s_efState1.pseudo, 0  
        
    //  Advance past destination IP.
    mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,   16    
    xout XID_CDECTRL,    s_cdeCmdWd,   4
        
l_paEf1ParseIpv6_1:
    // Pre-calculate the IPv6 fragmentation information
    // r0.b0: current offset
    // mov    s_txPktCxt.numFrags, 1 
    add    s_txPktCxt.fragSize, s_efState1.payloadLen, 40
    mov    s_txPktCxt.lastFragSize, s_txPktCxt.fragSize
    //mov    s_txPktCxt.ipHdrLen, 40 
    mov    s_txPktCxt.nextHdrOffset, OFFSET(s_Ipv6a.next)
    mov    r0.b0, 40
    
    // Look for ipv6 non-fragmentable header
l_paEf1ParseIpv6Frag_ShiftCheck:   
    qbeq    l_paEf1ParseIpv6Frag_HdrDone,    s_efState1.proto,   IP_PROTO_NEXT_IPV6_FRAG      // Can not frag a fragged packet
    qbeq    l_paEf1ParseIpv6Frag_CheckNext,  s_efState1.proto,   IP_PROTO_NEXT_IPV6_HOP_BY_HOP
    qbeq    l_paEf1ParseIpv6Frag_CheckNext,  s_efState1.proto,   IP_PROTO_NEXT_IPV6_ROUTE
    qbeq    l_paEf1ParseIpv6Frag_CheckNext,  s_efState1.proto,   IP_PROTO_NEXT_IPV6_DEST_OPT
    jmp     l_paEf1ParseIpv6Frag_HdrDone   
    
l_paEf1ParseIpv6Frag_CheckNext:        
    // Load the next header
    xin  XID_CDEDATA,  s_Ipv6Opt,  SIZE(s_Ipv6Opt)

    // Check for hdr size greater than 255
    qble    l_paEf1ParseIpv6Frag_HdrDone, s_Ipv6Opt.optlen, 63    
    mov     s_txPktCxt.nextHdrOffset, r0.b0 
    mov     s_efState1.proto,       s_Ipv6Opt.proto
    add     s_cdeCmdWd.byteCount,   s_Ipv6Opt.optlen, 1
    lsl     s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount, 3
    add     s_txPktCxt.ipHdrLen,    s_txPktCxt.ipHdrLen, s_cdeCmdWd.byteCount
    add     r0.b0,  r0.b0, s_cdeCmdWd.byteCount
    // Step past this header
    xout    XID_CDECTRL,            s_cdeCmdWd,              4
    
    jmp     l_paEf1ParseIpv6Frag_ShiftCheck

l_paEf1ParseIpv6Frag_HdrDone:
    // Exception Check Group 2
    qbbc l_paEf1ParseIpv6_exp_check_1, s_efRec1.ctrlFlags.t_ef_rec1_exp_ip_frag
        qbne  l_paEf1ParseIpv6_exp_check_1,  s_efState1.proto,            IP_PROTO_NEXT_IPV6_FRAG
            // Ipv6 fragmentation
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_FRAG
            jmp  f_efForwardExp                                // no return
        
l_paEf1ParseIpv6_exp_check_1: 
    qbbc l_paEf1ParseIpv6_exp_check_end, s_efRec1.ctrlFlags.t_ef_rec2_exp_ip_options
        qbeq l_paEf1ParseIpv6_exp_check_end, r0.b0, 40
            // Ipv6 options exists
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_OPTIONS
            jmp  f_efForwardExp                                // no return
    
l_paEf1ParseIpv6_exp_check_end: 
    qbne  l_paEf1ParseIpv6Frag_HdrDone_1,  s_efState1.proto,            IP_PROTO_NEXT_IPV6_FRAG
        // Fragmented packet can not be fragmented
        // TBD: do we need to adjust nextHdr????
       mov s_txPktCxt.ipHdrLen, 40
       jmp l_paEf1ParseIpv6_end
   
l_paEf1ParseIpv6Frag_HdrDone_1:
    mov    s_txPktCxt.nextHdr,  s_efState1.proto 
    mov    s_txPktCxt.ipHdrLen, r0.b0
    qbbc   l_paEf1ParseIpv6_end, s_efRec1.ctrlFlags.t_ef_rec1_ip_mtu
    qbge   l_paEf1ParseIpv6_end, s_txPktCxt.fragSize, s_efRec1.mtu
    add    r0, s_txPktCxt.ipHdrLen,    IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    // TBD: If the non-fragmentable portion consumes the full mtu, we can not frag
    qbge    l_paEf1ParseIpv6_end, s_efRec1.mtu, r0
        // IPv6 Fragmentation is required
        set  s_txPktCxt.flags.t_flag_ip_frag
    
        // local variable: payloadSize r0.w0: 
        //                 ipLen (FragTotalSize) r0.w2: ipLen - ipHdrLen
        and s_efRec1.mtu.b0,   s_efRec1.mtu.b0,  0xF8
        sub r0.w0, s_efRec1.mtu,   s_txPktCxt.ipHdrLen
        sub r0.w0, r0.w0, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
        sub r0.w2, s_txPktCxt.fragSize,  s_txPktCxt.ipHdrLen
        mov s_txPktCxt.fragSize,   s_efRec1.mtu 
        
l_paEf1ParseIpv6_fragment_loop:
        //add  s_txPktCxt.numFrags, s_txPktCxt.numFrags, 1
        sub  r0.w2, r0.w2, r0.w0
        qblt l_paEf1ParseIpv6_fragment_loop, r0.w2, r0.w0  
            // last fragment including the fragment header
            add s_txPktCxt.lastFragSize, r0.w2, s_txPktCxt.ipHdrLen
            add s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
            
            // end of the loop
                    
l_paEf1ParseIpv6_end:
    // Adjust Offsets (TBD:)
    add   s_txPktCxt.l4Offset,      s_txPktCxt.l3Offset2,   s_txPktCxt.ipHdrLen
          
    //jmp   l_paEf1ParsePostIp
    

l_paEf1ParsePostIp: 

    // Layer 4 protocol processing
    qbeq    f_paEf1ParseUdp,        s_efState1.proto,   IP_PROTO_NEXT_UDP  
    qbeq    f_paEf1ParseTcp,        s_efState1.proto,   IP_PROTO_NEXT_TCP  
    qbeq    f_paEf1ParseUdpLite,    s_efState1.proto,   IP_PROTO_NEXT_UDP_LITE  
    jmp     fci_paEfRec1_pktfwd

    .using udpScope

f_paEf1ParseUdp:

    // Read in the UDP header. 
    xin  XID_CDEDATA, s_udp,  SIZE(s_udp)
    
    // Update parameters
    qbbc l_paEf1ParseUdp_update_1, s_efRec1.ctrlFlags.t_ef_rec1_l4_src_port
        // Replace Source Port
        mov  s_udp.src, s_efRec1.srcPort 
        
        // pass through    
l_paEf1ParseUdp_update_1:

    qbbc l_paEf1ParseUdp_update_2, s_efRec1.ctrlFlags.t_ef_rec1_l4_dst_port
        // Replace Destination Port
        mov  s_udp.dst, s_efRec1.dstPort 
        
        // pass through    
l_paEf1ParseUdp_update_2:

    qbbc l_paEf1ParseUdp_update_end, s_efRec1.ctrlFlags.t_ef_rec1_l4_cksum
        mov s_udp.chksum,   0 
        // Configure the CDE to compute the checksum
        add s_efState1.pseudo,     s_efState1.pseudo, s_udp.len
        adc s_efState1.pseudo,     s_efState1.pseudo, IP_PROTO_NEXT_UDP
        adc s_cdeCmdChk.initSum,   s_efState1.pseudo, s_pktExtDescr.mopCsum
        adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum,     0
        sub s_cdeCmdChk.byteLen,   s_udp.len,   s_pktExtDescr.mopLength   
        mov s_cdeCmdChk.options,   1                          //TBD: Inverse
        mov s_cdeCmdChk.offset,    OFFSET(s_udp.chksum)
        mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_COMPUTE

        xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
        
l_paEf1ParseUdp_update_end:
    xout XID_CDEDATA, s_udp,  SIZE(s_udp)
    jmp  fci_paEfRec1_pktfwd    
    
f_paEf1ParseUdpLite:
    // Read in the UdpLite header. 
    xin  XID_CDEDATA, s_udpLite,  SIZE(s_udpLite)
    
    // Update parameters
    qbbc l_paEf1ParseUdpLite_update_1, s_efRec1.ctrlFlags.t_ef_rec1_l4_src_port
        // Replace Source Port
        mov  s_udpLite.src, s_efRec1.srcPort 
        
        // pass through    
l_paEf1ParseUdpLite_update_1:
    qbbc l_paEf1ParseUdpLite_update_2, s_efRec1.ctrlFlags.t_ef_rec1_l4_dst_port
        // Replace Destination Port
        mov  s_udpLite.dst, s_efRec1.dstPort 
        
        // pass through    
l_paEf1ParseUdpLite_update_2:
    qbbc l_paEf1ParseUdpLite_update_end, s_efRec1.ctrlFlags.t_ef_rec1_l4_cksum
        mov s_udpLite.chksum,   0 
        // Configure the CDE to compute the checksum
        //xin XID_PINFO_A,  s_pktExtDescr,  OFFSET(s_pktExtDescr.mopPtr)
        sub s_cdeCmdChk.byteLen,   s_txPktCxt.endOffset, s_txPktCxt.l4Offset  
        sub s_cdeCmdChk.byteLen,   s_cdeCmdChk.byteLen,  s_pktExtDescr.mopLength   
        add s_efState1.pseudo,     s_efState1.pseudo, s_cdeCmdChk.byteLen
        adc s_efState1.pseudo,     s_efState1.pseudo, IP_PROTO_NEXT_UDP_LITE
        adc s_cdeCmdChk.initSum,   s_efState1.pseudo, s_pktExtDescr.mopCsum
        adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum,     0
        mov s_cdeCmdChk.options,   1                  
        mov s_cdeCmdChk.offset,    OFFSET(s_udpLite.chksum)
        mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_COMPUTE

        xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
        
l_paEf1ParseUdpLite_update_end:
    xout XID_CDEDATA, s_udpLite,  SIZE(s_udpLite)
    jmp  fci_paEfRec1_pktfwd    

    .leave udpScope
    
    .using tcpScope
    
f_paEf1ParseTcp:

    // Read in the TCP header. 
    xin  XID_CDEDATA, s_tcp,  SIZE(s_tcp)
    
    // Exception check
l_paEf1ParseTcp_exp_check:
    qbbc l_paEf1ParseTcp_exp_check_end, s_efRec1.ctrlFlags.t_ef_rec1_exp_tcp_ctrl
        and   r0.b0,    s_tcp.ctrl,     TCP_CTRL_IND_MASK
        qbeq  l_paEf1ParseTcp_exp_check_end,  r0.b0,            0
            mov  s_txPktCxt2.eId,   EF_EROUTE_TCP_CTRL
            jmp  f_efForwardExp                                // no return
    
l_paEf1ParseTcp_exp_check_end:    
    // Update parameters
    qbbc l_paEf1ParseTcp_update_1, s_efRec1.ctrlFlags.t_ef_rec1_l4_src_port
        // Replace Source Port
        mov  s_tcp.src, s_efRec1.srcPort 
        
        // pass through    
l_paEf1ParseTcp_update_1:
    qbbc l_paEf1ParseTcp_update_2, s_efRec1.ctrlFlags.t_ef_rec1_l4_dst_port
        // Replace Destination Port
        mov  s_tcp.dst, s_efRec1.dstPort 
        
        // pass through    
l_paEf1ParseTcp_update_2:
    qbbc l_paEf1ParseTcp_update_end, s_efRec1.ctrlFlags.t_ef_rec1_l4_cksum
        mov s_tcp.chksum,   0 
        // Configure the CDE to compute the checksum
        //xin XID_PINFO_A,  s_pktExtDescr,  OFFSET(s_pktExtDescr.mopPtr)
        sub s_cdeCmdChk.byteLen,   s_txPktCxt.endOffset, s_txPktCxt.l4Offset  
        sub s_cdeCmdChk.byteLen,   s_cdeCmdChk.byteLen,  s_pktExtDescr.mopLength   
        add s_efState1.pseudo,     s_efState1.pseudo, s_cdeCmdChk.byteLen
        adc s_efState1.pseudo,     s_efState1.pseudo, IP_PROTO_NEXT_TCP
        adc s_cdeCmdChk.initSum,   s_efState1.pseudo, s_pktExtDescr.mopCsum
        adc s_cdeCmdChk.initSum,   s_cdeCmdChk.initSum,     0
        mov s_cdeCmdChk.options,   0                  
        mov s_cdeCmdChk.offset,    OFFSET(s_tcp.chksum)
        mov s_cdeCmdChk.operation, CDE_CMD_CHECKSUM2_COMPUTE

        xout XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
        
l_paEf1ParseTcp_update_end:
    xout XID_CDEDATA, s_tcp,  SIZE(s_tcp)
    //jmp  fci_paEfRec1_pktfwd 
    // pass through   
    
fci_paEfRec1_pktfwd:  
        // The packet location is at l4Offset
        // Check whether we need to remove the outer IP trail
        qbbc    l_paEfRec1_pktfwd_1,   s_efRec1.ctrlFlags.t_ef_rec1_strip_outer_ip
        
            // Advance the CDE to the end of inner IP packet
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            sub  s_cdeCmdWd.byteCount,  s_txPktCxt.endOffset,  s_txPktCxt.l4Offset
            sub  s_cdeCmdWd.byteCount,  s_cdeCmdWd.byteCount,  s_pktExtDescr.mopLength 
            xout XID_CDECTRL,           s_cdeCmdWd,     4
        
            mov  s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_END
            xout XID_CDECTRL,           s_cdeCmdWd,     4
            
            // Adjust sopLength if necessary
            qble l_paEfRec1_pktfwd_1,   s_txPktCxt.endOffset,  s_pktExtDescr.sopLength
                mov  s_pktExtDescr.sopLength,   s_txPktCxt.endOffset 
    
l_paEfRec1_pktfwd_1:    
        // Forward the packet for next stage operation
        // Restore the latest extended Packet Info
        //xin   XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
        wbs   s_flags.info.tStatus_CDEOutPacket
        sbco  s_txPktCxt, cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_txPktCxt)
        
        // Forward the packet
        // Pass to the next stage at Egres0
        //mov  s_pktExtDescr.threadId, THREADID_EGRESS0
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
            
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        xout XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
    
        jmp  f_mainLoop
    

    .leave tcpScope
    
    .leave cdeScope
    .leave pktScope 
    .leave ipScope
    
#endif    


// *******************************************************************************
// * FUNCTION PURPOSE: Record 2 Processing
// *******************************************************************************
// * DESCRIPTION: Process the egress flow record 2 
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:      | Temp and local variables
// *   R2:      | 
// *   R3:      | 
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |        
// *   R7:        |  L3 Header                            
// *   R8:        |  (Temporary data)                           
// *   R9:        |  
// *   R10:       |  
// *   R11:         |  
// *   R12:         |  Record Info
// *   R13:         |   
// *   R14:            |                                            
// *   R15:            | Extended Descriptor                                           
// *   R16:            |                                            
// *   R17:            |                         
// *   R18:            | 
// *   R19:       |     
// *   R20:       |  Record State machine (s_efState)   | s_modState will be overwritten
// *   R21:       |                                     | for EF Record processing
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:  r28.w0: record offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    
#ifdef PASS_PROC_EF_REC2

    .using cdeScope
    .using pktScope     // For offset info
    .using ipScope

//
// Define several groups of Local variables R2-R3 
//

.struct struct_ef2_loc1
    .u16    payLoadLen
    .u8     ipHdrLen
    .u8     ipsecHdrLen
    .u16    payLoadSize          // align with loc2 variable payLoadSize
    .u16    totalFragSize        // align with loc2 variable totalFragSize 
.ends

.struct struct_ef2_loc2
    .u16    dataRemain
    .u16    offsetRemain
    .u16    payloadSize          // global for multiple lopp
    .u16    totalFragSize        // global for multiple loop
.ends

.enter   ef2ScopeLoc1
    .assign struct_ef2_loc1,    r2,     r3,        s_ef2_loc1
.leave   ef2ScopeLoc1    

.enter   ef2ScopeLoc2
    .assign struct_ef2_loc2,    r2,     r3,        s_ef2_loc2
.leave   ef2ScopeLoc2 

// Egress Flow record2 processing route main entry

f_paEfRec2Proc:

    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    
    // Restore the latest extended Packet Info (Extended Packet Info is already loaded by main loop)
    // xin  XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
    
    qbbc  f_paEfRec2_pktfwd, s_txPktCxt.flags.t_flag_ef_rec_valid_lvl2
        
l_paEfRec2Proc_1:
    // Calculate and record the record2 offset
    lsl  r28.w0,    s_txPktCxt.lvl2RecIndex,   6
    lbco s_efRec2,  PAMEM_CONST_EF_RECORD2,    r28.w0,     SIZE(s_efRec2)
    
    // TBD: Verify whetehr the record is valid
    
    // Global initialization prior to the record processing loop
    zero  &s_efState2,  SIZE(s_efState2)
    mov   s_efState2.extHdrFlags, s_pktExtDescr2.flags 
    mov   s_pktExtDescr2.flags, 0
    
    // Setup MOP tracking
    mov   s_pktExtDescr2.mopLengthOrig,   s_pktExtDescr2.mopLength
    mov   s_pktExtDescr2.mopPtrOrig,      s_pktExtDescr2.mopPtr
    
    mov   s_efState2.origSopLen, s_pktExtDescr2.sopLength  
    
    add   r0, s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset2 
    sub   s_pktExtDescr2.sopL4Length, s_pktExtDescr2.sopLength, r0
    
    set  s_efState2.flags.t_ef_firstLoop  
    qbbs l_paEfRec2Proc_2,  s_txPktCxt.flags.t_flag_ip_frag  
        set s_efState2.flags.t_ef_noFrag
        set s_efState2.flags.t_ef_lastFrag
        
l_paEfRec2Proc_2:  
      
    // Set Threadd ID for all fragments
    qbbc l_paEf2_setThreadID_nonIpsec,    s_efRec2.ctrlFlags.t_ef_rec2_ipsec_proc
        qbbs l_paEf2_setThreadID_Ipsec_1,    s_efRec2.ctrlFlags.t_ef_rec2_loc_dma
            // Global DMA
            mov  s_pktExtDescr.threadId, THREADID_CDMA0
            jmp  l_paEf2_setThreadID_end 
l_paEf2_setThreadID_Ipsec_1:
            mov  s_pktExtDescr.threadId, THREADID_CDMA1
            jmp  l_paEf2_setThreadID_end 

l_paEf2_setThreadID_nonIpsec:
        qbbs l_paEf2_setThreadID_nonIpsec_1,    s_txPktCxt.flags.t_flag_ef_rec_valid_lvl3
            // Egress Stage 2
            mov  s_pktExtDescr.threadId, THREADID_EGRESS2
            jmp  l_paEf2_setThreadID_end 
l_paEf2_setThreadID_nonIpsec_1:
            // Egress Stage 1
            mov  s_pktExtDescr.threadId, THREADID_EGRESS1
            // jmp  l_paEf2_setThreadID_end
    
l_paEf2_setThreadID_end:

l_paEfRec2Proc_loop: 

.using  ef2ScopeLoc1


    // restore original packet information
    mov s_pktExtDescr2.sopLength, s_efState2.origSopLen
    clr s_txPktCxt.flags.t_flag_ip_frag  
    
    // Calculate the IP length
    zero    &s_ef2_loc1,    SIZE(s_ef2_loc1)
    qbbs    l_paEfRec2Proc_ip_payload_1, s_efState2.flags.t_ef_lastFrag
            // Non-last fragment
            mov s_ef2_loc1.payLoadLen,  s_txPktCxt.fragSize
            sub s_ef2_loc1.payLoadSize, s_txPktCxt.fragSize,    s_txPktCxt.ipHdrLen    
            jmp l_paEfRec2Proc_ip_payload_2  

l_paEfRec2Proc_ip_payload_1:
            mov s_ef2_loc1.payLoadLen,  s_txPktCxt.lastFragSize
            sub s_ef2_loc1.payLoadSize, s_txPktCxt.lastFragSize,    s_txPktCxt.ipHdrLen    
            
            // pass through
            
l_paEfRec2Proc_ip_payload_2:            
    // Move to outer IP header  */
    // Advance the CDE to the IP header to examine the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset
    xout XID_CDECTRL,           s_cdeCmdWd,     SIZE(s_cdeCmdWd)
    
    qbeq l_paEfRec2Proc_update_ip,  s_efRec2.l3HdrSize,    0 
    qbeq l_paEfRec2Proc_ip_ins, s_txPktCxt.l3Offset2, s_txPktCxt.l3Offset 
        // Flush out outer IP
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        sub  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset
        xout XID_CDECTRL,           s_cdeCmdWd,             4    

        // Adjust the offsets
        mov s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset 
        sub s_txPktCxt.l4Offset,    s_txPktCxt.l4Offset,    s_cdeCmdWd.byteCount
        
        // Note: ESP/AH header will be set only when it is required
        //       l5 offset is not used
        
        // Adjust the sopLength
        sub s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_cdeCmdWd.byteCount 
        
        // pass through
        
//
// Insert and update IP header from EF2 record
//        

l_paEfRec2Proc_ip_ins:
    // Read the minimum IP header
    add  r0.w0,  r28.w0,   PA_EF_REC2_L3_HDR_OFFSET  
    lbco s_Ip,   PAMEM_CONST_EF_RECORD2, r0.w0, OFFSET(s_Ip.srcIp) 
    and  r0.b0,  s_Ip.verLen,  0xF0
    
    qbeq l_paEf2InsertIpv4,  r0.b0,       0x40
    qbeq l_paEf2InsertIpv6,  r0.b0,       0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
l_paEf2InsertIpv4:
    // record outer IP protocol
    mov  s_efState2.nextHdr,  s_Ip.protocol  
        
    // Calculate the IP length
    and     r0.b0,          s_Ip.verLen,              0x0f
    lsl     s_ef2_loc1.ipHdrLen,  r0.b0,              2  // 32 bit size to 8 bit size conversion
    
    // record the dscp value
    lsr     s_txPktCxt3.dscp,   s_Ip.tos,             2  
    
    call f_paEf2CalIPLengthAndUpdate
    
    // Issue checksum command
    qbbc l_paEf2InsertIpv4_chksum_end,   s_efState2.flags.t_ef_firstLoop
        zero  &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
        mov   s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
        mov   s_cdeCmdChk.byteLen,  s_ef2_loc1.ipHdrLen    
        mov   s_cdeCmdChk.offset,   OFFSET(s_Ip.checksum)          // Starting sum is 0, offset is 10
        xout  XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)

l_paEf2InsertIpv4_chksum_end:
    add  s_Ip.totalLen, s_ef2_loc1.payLoadLen,  s_ef2_loc1.ipHdrLen
    mov  s_Ip.checksum, 0
    
    // provide unique id for each fragment or packet
    //mov  s_Ip.id, s_ipPktId.id.w0
    //add  s_ipPktId.id, s_ipPktId.id, 1
	lbco  s_Ip.id, cMailbox, 2, 2
    
    // Insert the first 12-bytes
    mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
    mov  s_cdeCmdIn2.len,           4 
    mvid s_cdeCmdIn2.data,          *&s_Ip.verLen
    xout XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    mvid s_cdeCmdIn2.data,          *&s_Ip.id
    xout XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    mvid s_cdeCmdIn2.data,          *&s_Ip.ttl
    xout XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)    
    
    // Insert the remaining data
    mov  s_cdeCmdInD.operation,     CDE_CMD_INSERT_PACKET_BUFFER
    sub  s_cdeCmdInD.lenLsb,        s_ef2_loc1.ipHdrLen,  12
    mov  s_cdeCmdInD.lenMsbs,       0
    mov  s_cdeCmdInD.dataP,         PAMEM_CONST_EF_RECORD2_BASE
    add  s_cdeCmdInD.dataP,         s_cdeCmdInD.dataP,  r28.w0
    add  s_cdeCmdInD.dataP,         s_cdeCmdInD.dataP,  PA_EF_REC2_L3_HDR_OFFSET + 12
    xout XID_CDECTRL,               s_cdeCmdInD,        SIZE(s_cdeCmdInD)
    
    jmp   l_paEf2InsertIp_common
            
l_paEf2InsertIpv6:
    // record outer IP protocol
    mov  s_efState2.nextHdr,  s_Ipv6a.next 
    
    // record the updated class value as DSCP
   lsr   s_txPktCxt3.dscp,   s_Ipv6a.ver_tclass_flow.w2,  4
   and   s_txPktCxt3.dscp,   s_txPktCxt3.dscp,            0x3F
    
    // Calculate the IP length
    // TBD: Extension header processing
    set s_efState2.flags.t_ef2_outer_ipv6
    mov s_ef2_loc1.ipHdrLen,    40
    
    call f_paEf2CalIPLengthAndUpdate
    
    mov  s_Ipv6a.payloadLen, s_ef2_loc1.payLoadLen
    
    // Insert the first 8-bytes
    mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
    mov  s_cdeCmdIn2.len,           4 
    mov s_cdeCmdIn2.data,           s_Ipv6a.ver_tclass_flow
    xout XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    mvid s_cdeCmdIn2.data,          *&s_Ipv6a.payloadLen
    xout XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    
    // Insert the remaining data
    mov  s_cdeCmdInD.operation,     CDE_CMD_INSERT_PACKET_BUFFER
    sub  s_cdeCmdInD.lenLsb,        s_ef2_loc1.ipHdrLen,  8
    mov  s_cdeCmdInD.lenMsbs,       0
    mov  s_cdeCmdInD.dataP,         PAMEM_CONST_EF_RECORD2_BASE
    add  s_cdeCmdInD.dataP,         s_cdeCmdInD.dataP,  r28.w0
    add  s_cdeCmdInD.dataP,         s_cdeCmdInD.dataP,  PA_EF_REC2_L3_HDR_OFFSET + 8
    xout XID_CDECTRL,               s_cdeCmdWd,         SIZE(s_cdeCmdWd)
            
l_paEf2InsertIp_common:            
    // Adjust all offset length
    add s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset2,   s_ef2_loc1.ipHdrLen 
    add s_txPktCxt.l4Offset,    s_txPktCxt.l4Offset,    s_ef2_loc1.ipHdrLen
    
    // Adjust the sopLength
    add s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_ef2_loc1.ipHdrLen 
    
    jmp l_paEf2PostIpProc
    
l_paEfRec2Proc_update_ip:    
    //
    // Update outer or Inner IP header
    //
    
    // Update outer IP
    // Read the minimum IP header
    xin  XID_CDEDATA,  s_Ip,   OFFSET(s_Ip.srcIp) 
    and  r0.b0,  s_Ip.verLen,  0xF0
    
    qbeq l_paEf2UpdateIpv4,  r0.b0,       0x40
    qbeq l_paEf2UpdateIpv6,  r0.b0,       0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
l_paEf2UpdateIpv4:
    qbbc    l_paEf2UpdateIpv4_update_2, s_efState2.flags.t_ef_firstLoop
        // TBD: Error check
        
    // Exception Check
l_paEf2UpdateIpv4_exp_check:    
        qbbc l_paEf2UpdateIpv4_exp_check_1, s_efRec2.ctrlFlags.t_ef_rec2_exp_ip_expire
            qbeq f_paEfProc_ip_expire,  s_Ip.ttl,  0  
        // pass through
        
l_paEf2UpdateIpv4_exp_check_1:        
    qbbc l_paEf2UpdateIpv4_exp_check_2, s_efRec2.ctrlFlags.t_ef_rec2_exp_ip_frag
        mov   r0.w0,    IPV4_FRAG_MASK
        and   r0.w0,    s_Ip.fragOff,     r0.w0
        qbeq  l_paEf2UpdateIpv4_exp_check_2,  r0.w0,            0
            // IPv4 fragmentation
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_FRAG
            jmp  f_efForwardExp                                // no return
        
l_paEf2UpdateIpv4_exp_check_2: 
    qbbc l_paEf2UpdateIpv4_exp_check_end, s_efRec2.ctrlFlags.t_ef_rec2_exp_ip_options
        and  r0.b0,  s_Ip.verLen,  0x0F
        qbeq l_paEf2UpdateIpv4_exp_check_end, r0.b0, 5
            // IPv4 options exists
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_OPTIONS
            jmp  f_efForwardExp                                // no return
    
l_paEf2UpdateIpv4_exp_check_end:    
    
        // Update parameters
        qbbc l_paEf2UpdateIpv4_update_1, s_efRec2.ctrlFlags.t_ef_rec2_ip_ttl_update
            // TTL check 
            qbeq l_paEf2UpdateIpv4_update_1,  s_Ip.ttl,  0  
                sub s_Ip.ttl, s_Ip.ttl, 1
            // pass through 
        
l_paEf2UpdateIpv4_update_1:
            // record outer IP protocol
            mov  s_efState2.nextHdr,  s_Ip.protocol  
        
l_paEf2UpdateIpv4_update_2:  
    // Calculate and record outer IP header length
    and     r0.b0,          s_Ip.verLen,              0x0f
    lsl     s_ef2_loc1.ipHdrLen,  r0.b0,              2  // 32 bit size to 8 bit size conversion
    
    // record the dscp value
    lsr     s_txPktCxt3.dscp,   s_Ip.tos,             2  

    call f_paEf2CalIPLengthAndUpdate
    
    add  s_Ip.totalLen, s_ef2_loc1.payLoadLen,  s_ef2_loc1.ipHdrLen
    
    qbbc l_paEf2UpdateIpv4_update_chksum_end,   s_efState2.flags.t_ef_firstLoop
        // Issue checksum command
        zero  &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
        mov   s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
        mov   s_cdeCmdChk.byteLen,  s_ef2_loc1.ipHdrLen
        mov   s_cdeCmdChk.offset, OFFSET(s_Ip.checksum)          // Starting sum is 0, offset is 10
        xout  XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
  
l_paEf2UpdateIpv4_update_chksum_end:
    mov   s_Ip.checksum, 0
    xout  XID_CDEDATA, s_Ip,    OFFSET(s_Ip.srcIp)
    
    //  Advance past the IP header.
    mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov   s_cdeCmdWd.byteCount,   s_txPktCxt.ipHdrLen
    xout  XID_CDECTRL,    s_cdeCmdWd,   4
    jmp   l_paEf2PostIpProc
            
l_paEf2UpdateIpv6:
    qbbc    l_paEf2UpdateIpv6_update_2, s_efState2.flags.t_ef_firstLoop
    // TBD: Error check
    
    // Exception check group 1
    qbbc l_paEf2UpdateIpv6_update,  s_efRec2.ctrlFlags.t_ef_rec2_exp_ip_expire
        qbeq f_paEfProc_ip_expire,  s_Ipv6a.hopLimit,  0  
        // pass through
    
l_paEf2UpdateIpv6_update:
    
    // Update parameters
    qbbc l_paEf2UpdateIpv6_update_1, s_efRec2.ctrlFlags.t_ef_rec2_ip_ttl_update
        // Hop limit  check 
        qbge f_paEfProc_ip_expire,  s_Ipv6a.hopLimit,  1  
            sub s_Ipv6a.hopLimit, s_Ipv6a.hopLimit, 1
        // pass through 
        
l_paEf2UpdateIpv6_update_1:
        // record the updated class value as DSCP
        lsr s_txPktCxt3.dscp,   s_Ipv6a.ver_tclass_flow.w2,  4
        and s_txPktCxt3.dscp,   s_txPktCxt3.dscp,            0x3F
        
l_paEf2UpdateIpv6_update_2:  
        // Calculate the IP length
        // TBD: Extension header processing
        mov s_efState2.nextHdr,  s_Ipv6a.next  
        set s_efState2.flags.t_ef2_outer_ipv6
        mov s_ef2_loc1.ipHdrLen,  40
        mov s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
        mov s_cdeCmdWd.byteCount, 40  
        
    //mov    s_txPktCxt.ipHdrLen, 40 
    //mov    s_txPktCxt.nextHdrOffset, OFFSET(s_Ipv6a.next)
    //mov    r0.b0, 40
    
    // Look for ipv6 non-fragmentable header
l_paEf2UpdateIpv6_update_ShiftCheck:   
    // TBD: Error processing 
    qbeq    l_paEf2UpdateIpv6_update_HdrDone,    s_efState2.nextHdr,   IP_PROTO_NEXT_IPV6_FRAG      // Can not frag a fragged packet
    qbeq    l_paEf2UpdateIpv6_update_CheckNext,  s_efState2.nextHdr,   IP_PROTO_NEXT_IPV6_HOP_BY_HOP
    qbeq    l_paEf2UpdateIpv6_update_CheckNext,  s_efState2.nextHdr,   IP_PROTO_NEXT_IPV6_ROUTE
    qbeq    l_paEf2UpdateIpv6_update_CheckNext,  s_efState2.nextHdr,   IP_PROTO_NEXT_IPV6_DEST_OPT
    jmp     l_paEf2UpdateIpv6_update_HdrDone   
    
l_paEf2UpdateIpv6_update_CheckNext:
    // Step past the previous header
    xout    XID_CDECTRL,            s_cdeCmdWd,              4
            
    // Load the next header
    xin  XID_CDEDATA,  s_Ipv6Opt,  SIZE(s_Ipv6Opt)

    // Check for hdr size greater than 255
    qble    l_paEf2UpdateIpv6_update_HdrDone, s_Ipv6Opt.optlen, 63  
    
    //mov     s_txPktCxt.nextHdrOffset, r0.b0 
    mov     s_efState2.nextHdr,     s_Ipv6Opt.proto
    add     s_cdeCmdWd.byteCount,   s_Ipv6Opt.optlen, 1
    lsl     s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount, 3
    add     s_ef2_loc1.ipHdrLen,    s_ef2_loc1.ipHdrLen, s_cdeCmdWd.byteCount
    // add     r0.b0,  r0.b0, s_cdeCmdWd.byteCount
    
    jmp     l_paEf2UpdateIpv6_update_ShiftCheck

l_paEf2UpdateIpv6_update_HdrDone:
    // Exception Check Group 2
    qbbc l_paEf2UpdateIpv6_exp_check_1, s_efRec2.ctrlFlags.t_ef_rec2_exp_ip_frag
        qbne  l_paEf2UpdateIpv6_exp_check_1,  s_efState2.nextHdr,            IP_PROTO_NEXT_IPV6_FRAG
            // Ipv6 fragmentation
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_FRAG
            jmp  f_efForwardExp                                // no return
        
l_paEf2UpdateIpv6_exp_check_1: 
    qbbc l_paEf2UpdateIpv6_exp_check_end, s_efRec1.ctrlFlags.t_ef_rec2_exp_ip_options
        qbeq l_paEf2UpdateIpv6_exp_check_end, s_ef2_loc1.ipHdrLen, 40
            // Ipv6 options exists
            mov  s_txPktCxt2.eId,   EF_EROUTE_IP_OPTIONS
            jmp  f_efForwardExp                                // no return
    
l_paEf2UpdateIpv6_exp_check_end: 
        //
        // TBD: move to local variable
        // Note: r1 can not be used at f_paEf2CalIPLengthAndUpdate
        //       r1.b0: length of the last non-fragmentable header
        //       r1.b1: nextHdr
        //
        mov  r1.b0,  s_cdeCmdWd.byteCount

        call f_paEf2CalIPLengthAndUpdate
        
        mov   s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
        mov   s_cdeCmdWd.byteCount, r1.b0  
        
        qbne l_paEf2UpdateIpv6_update_3, s_ef2_loc1.ipHdrLen, 40
            mov  s_Ipv6a.payloadLen, s_ef2_loc1.payLoadLen
            xout XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
            
            // Advance past the IPv6 header
            xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
            
            jmp  l_paEf2UpdateIpv6_update_4
        
l_paEf2UpdateIpv6_update_3: 
            // Update protocol Header
            xout  XID_CDEDATA,  s_Ipv6Opt.proto,  SIZE(s_Ipv6Opt.proto)
            
            // Advance past the last non-fragmentable header
            xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
            
            // patch IPv6 payload length
            mov  s_cdeCmdPatch.operation,   CDE_CMD_PATCH_PACKET
            mov  s_cdeCmdPatch.len,         2
            add  s_cdeCmdPatch.offset,      s_txPktCxt.l3Offset,    OFFSET(s_Ipv6a.payloadLen)
            add  s_cdeCmdPatch.data.w2,     s_ef2_loc1.payLoadLen,  s_ef2_loc1.ipHdrLen
            sub  s_cdeCmdPatch.data.w2,     s_cdeCmdPatch.data.w2,  40
            xout XID_CDECTRL,    s_cdeCmdPatch,     SIZE(s_cdeCmdPatch)
               
l_paEf2UpdateIpv6_update_4:        
        //  Advance past the IPv6 header.
        //mov   s_cdeCmdWd.operation, CDE_CMD_WINDOW_ADVANCE
        //mov   s_cdeCmdWd.byteCount, 40  
        //xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
        //jmp   l_paEf2PostIpProc
        
l_paEf2PostIpProc: 
    qbbc l_paEf2IpFrag, s_efRec2.ctrlFlags.t_ef_rec2_ipsec_proc
        // IPSEC Processing is required
        add s_txPktCxt.espAhOffset, s_txPktCxt.l3Offset, s_ef2_loc1.ipHdrLen 
        qbbs l_paEf2PostIpProc_ipsec_ins, s_efRec2.ctrlFlags.t_ef_rec2_ins_ipsec
            // No need to insert IPSEC Header, just move past it
            mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
            mov   s_cdeCmdWd.byteCount,   s_ef2_loc1.ipsecHdrLen
            xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
            jmp   l_paEf2IpFrag
            
l_paEf2PostIpProc_ipsec_ins:
        qbbs    l_paEf2PostIpProc_ipsec_ins_ah,   s_efRec2.ctrlFlags.t_ef_rec2_ipsec_ah
        
l_paEf2PostIpProc_ipsec_ins_esp: 
            // Insert ESP Header 
            // SPI, 0 ...
            mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
            mov  s_cdeCmdIn2.len,           4 
            add  r0.w0, r28.w0, PA_EF_REC2_SPI_OFFSET
            lbco s_cdeCmdIn2.data,  PAMEM_CONST_EF_RECORD2, r0.w0,  4
            xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
            
            // Insert 0s
            sub  s_cdeCmdIn2.len,           s_ef2_loc1.ipsecHdrLen, 4 
            xout XID_CDECTRL, s_cdeCmdIn2,  4 
            jmp  l_paEf2PostIpProc_ipsec_ins_common  
              
l_paEf2PostIpProc_ipsec_ins_ah:
            // Insert AH Header 
            // Next Hdr, length, 0, SPI, 0 ...
            mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
            mov  s_cdeCmdIn2.len,           4 
            mov  s_cdeCmdIn2.data.w0,       0
            mov  s_cdeCmdIn2.data.b3,       s_efState2.nextHdr
            lsr  s_cdeCmdIn2.data.b2,       s_ef2_loc1.ipsecHdrLen, 2
            sub  s_cdeCmdIn2.data.b2,       s_cdeCmdIn2.data.b2,    2
            xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
            
            add  r0.w0, r28.w0, PA_EF_REC2_SPI_OFFSET
            lbco s_cdeCmdIn2.data,  PAMEM_CONST_EF_RECORD2, r0.w0,  4
            xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
            
            // Insert 0s
            sub  s_cdeCmdIn2.len,           s_ef2_loc1.ipsecHdrLen, 8 
            xout XID_CDECTRL, s_cdeCmdIn2,  4 
            
            // Update control info for data patch
            set s_txPktCxt.flags.t_flag_ah_patch
        
            // icvSize = 8 + r1r0 * 4 (8, 12, 16)
            and s_txPktCxt.flags.b0, s_txPktCxt.flags.b0, NOT_PA_EF_ICV_SIZE_MASK
            sub r0.b0,  s_efRec2.icvSize, 8
            lsr r0.b0,  r0.b0,  2
            or  s_txPktCxt.flags.b0,    s_txPktCxt.flags.b0, r0.b0
            

l_paEf2PostIpProc_ipsec_ins_common:
            // adjust length        
            // Adjust all offset length
            add s_txPktCxt.l4Offset,    s_txPktCxt.l4Offset,    s_ef2_loc1.ipsecHdrLen
        
            // Adjust the sopLength
            add s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_ef2_loc1.ipsecHdrLen 
            
            qbbs   l_paEf2IpFrag, s_efRec2.ctrlFlags.t_ef_rec2_single_ip
                add s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset2,   s_ef2_loc1.ipsecHdrLen 
            
            // Pass through for Inner IP fragmentation
            
l_paEf2IpFrag: 
     qbbc  l_paEf2IpFrag_1, s_efState2.flags.t_ef_noFrag
         // No IP fragmentation is required
         // Advance over inner IP
         mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
         sub   s_cdeCmdWd.byteCount,   s_txPktCxt.fragSize, s_pktExtDescr2.mopLength 
         // Adjust the move length to exclude IP header if single_ip 
         qbbc  l_paEf2IpFrag_00, s_efRec2.ctrlFlags.t_ef_rec2_single_ip
            sub s_cdeCmdWd.byteCount,   s_cdeCmdWd.byteCount,   s_ef2_loc1.ipHdrLen 
l_paEf2IpFrag_00:                
         xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
         
         // Flush out the remaining packet 
         mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
         xout XID_CDECTRL,             s_cdeCmdPkt,             4
         
         qblt l_paEf2IpFrag_0, s_ef2_loc1.payloadSize, s_pktExtDescr2.sopL4Length 
            set s_efState2.flags.t_ef_withinSop 

l_paEf2IpFrag_0:
         jmp   f_paEf2InsEspTrail
         
    .leave  ef2ScopeLoc1
         
    .using  ef2ScopeLoc2
     
l_paEf2IpFrag_1: 
    // Update the statistics
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_TX_IP_FRAG
    
    // Load the packet Ip header
    xin     XID_CDEDATA, s_Ip, SIZE(s_Ip)
    
    // Find whetehr it is IPv4 or IPv6
    and     r0.b0, s_Ip.VerLen, 0xF0
    qbeq    l_paEf2IpFrag_ProcessIp_1, r0.b0, 0x40
    
    //Must be IPv6: otherwise ignore
    qbeq    fci_paEf2Ipv6Frag, r0.b0, 0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
//
// IPv4 fragmentation loop
//     
    
l_paEf2IpFrag_ProcessIp_1:
    // Save the base offset
    qbbc     l_paEf2IpFrag_ProcessIp_2,  s_efState2.flags.t_ef_firstLoop         
        mov s_efState2.baseOffset, s_Ip.FragOff
        mov s_efState2.ipLen, s_Ip.TotalLen 
        //sub s_ef2_loc2.payloadSize, s_txPktCxt.fragSize,   s_txPktCxt.ipHdrLen   
       
l_paEf2IpFrag_ProcessIp_2:
    // SOP is at least L2+L3
    add     s_pktExtDescr2.sopLength, s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset2   

    // Patch the frag offset
    lsr     r1, s_efState2.loopOffset, 3
    add     s_Ip.FragOff, s_efState2.baseOffset, r1
    
    // check for last fragment
    qbbs    l_paEf2IpFrag_CommonFrag,   s_efState2.flags.t_ef_lastFrag   
        
l_paEf2IpFrag_NotLast:        
    // Set more fragments
    set     s_Ip.FragOff.t_ipv4_frag_m

l_paEf2IpFrag_CommonFrag:
    add     s_Ip.TotalLen, s_ef2_loc2.payloadSize,  s_txPktCxt.ipHdrLen  
    mov     s_Ip.Checksum, 0

    // Write out the new IP header
    xout    XID_CDEDATA, s_Ip, SIZE(s_Ip)-8         // We never update the last 8 bytes (or options)
    
    // Start the IP Header checksum. 
    qbbc l_paEf2IpFrag_chksum_end,   s_efState2.flags.t_ef_firstLoop
        zero    &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
        mov     s_cdeCmdChk.operation,CDE_CMD_CHECKSUM2_COMPUTE
        mov     s_cdeCmdChk.byteLen,  s_txPktCxt.ipHdrLen
        mov     s_cdeCmdChk.offset,   OFFSET(s_Ip.Checksum)          // Starting sum is 0, offset is 10
        xout    XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
            
l_paEf2IpFrag_chksum_end:
    // Slide the window past the IP header
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.ipHdrLen
    xout XID_CDECTRL,           s_cdeCmdWd,              4
    
    // Move the amount we need to copy  and skip into "Remain"
    sub s_ef2_loc2.dataRemain,   s_Ip.TotalLen,    s_txPktCxt.ipHdrLen 
    mov s_ef2_loc2.offsetRemain, s_efState2.loopOffset
        
    // See if we still have data remaining in SOP
    qbgt    l_paEf2IpFrag_HaveSOP,   s_ef2_loc2.offsetRemain,   s_pktExtDescr2.sopL4Length
        // Flush past all SOP
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length
        xout XID_CDECTRL,           s_cdeCmdWd,              4    
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_ef2_loc2.offsetRemain,    s_ef2_loc2.offsetRemain, s_pktExtDescr2.sopL4Length
        jmp  l_paEf2IpFrag_PastSOP
            
l_paEf2IpFrag_HaveSOP: 
        qbeq l_paEf2IpFrag_NoFlush, s_efState2.loopOffset, 0       // redundancy check
            // Now flush any data contained in s_paEf2IpFragCxt.loopOffset
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
            mov  s_cdeCmdWd.byteCount,  s_ef2_loc2.offsetRemain
            xout XID_CDECTRL, s_cdeCmdWd, 4 
                
l_paEf2IpFrag_NoFlush:        
            // Slide the window past the valid data
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            sub  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length, s_ef2_loc2.offsetRemain
            // Trim the SOP to the data size
            qbgt    l_paEf2IpFrag_NoSOPTrim, s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain 
                mov s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain
                set s_efState2.flags.t_ef_withinSop
                
l_paEf2IpFrag_NoSOPTrim:                 
                xout XID_CDECTRL,  s_cdeCmdWd,   4
                mov  s_ef2_loc2.offsetRemain,    0
                sub  s_ef2_loc2.dataRemain,     s_ef2_loc2.dataRemain, s_cdeCmdWd.byteCount
                add  s_pktExtDescr2.sopLength,  s_pktExtDescr2.sopLength,   s_cdeCmdWd.byteCount 
                
l_paEf2IpFrag_PastSOP:
            // Here we're past SOP or we don't have any data left
            // We may need to copy some MOP
            mov     s_pktExtDescr2.mopLength,   s_ef2_loc2.dataRemain
            qbeq    l_paEf2IpFrag_FragReady,    s_ef2_loc2.dataRemain, 0
            
            // see if we have any MOP
            qbgt    l_paEf2IpFrag_HaveMOP,   s_ef2_loc2.offsetRemain, s_pktExtDescr2.mopLengthOrig
                sub s_ef2_loc2.offsetRemain, s_ef2_loc2.offsetRemain, s_pktExtDescr2.mopLengthOrig
                mov s_pktExtDescr2.mopLength, 0
                jmp l_paEf2IpFrag_PastMOP
                
l_paEf2IpFrag_HaveMOP:
            add     s_pktExtDescr2.mopPtr,  s_pktExtDescr2.mopPtrOrig, s_ef2_loc2.offsetRemain 
            // Trim the MOP to the MOP remaining size
            add     r0,  s_ef2_loc2.offsetRemain,  s_ef2_loc2.dataRemain
            qbgt    l_paEf2IpFrag_NoMOPTrim, r0, s_pktExtDescr2.mopLengthOrig
                sub  s_pktExtDescr2.mopLength,  s_pktExtDescr2.mopLengthOrig,   s_ef2_loc2.offsetRemain
                
l_paEf2IpFrag_NoMOPTrim:
                sub  s_ef2_loc2.dataRemain,    s_ef2_loc2.dataRemain, s_pktExtDescr2.mopLength
                mov  s_ef2_loc2.offsetRemain,  0
                           
l_paEf2IpFrag_PastMOP:
            qbeq    l_paEf2IpFrag_NoEOPFlush,    s_ef2_loc2.offsetRemain,   0 
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
                mov  s_cdeCmdWd.byteCount,  s_ef2_loc2.offsetRemain
                xout XID_CDECTRL, s_cdeCmdWd, 4 
                mov  s_ef2_loc2.offsetRemain, 0
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
                // pass through
             
l_paEf2IpFrag_NoEOPFlush:             
            // If there is any EOP data to move, move it now
            qbeq    l_paEf2IpFrag_FragReady, s_ef2_loc2.dataRemain, 0
                // Slide the window past the valid data
                mov s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain
                xout XID_CDECTRL,  s_cdeCmdWd,    4
            
l_paEf2IpFrag_FragReady:
    // Flush out the remaining packet 
    mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,             s_cdeCmdPkt,             4

    // Setup the offset for the next packet (jump out if final frag)
    add  s_efState2.loopOffset, s_efState2.loopOffset, s_ef2_loc2.payloadSize
    
    jmp  f_paEf2InsEspTrail
    
fci_paEf2Ipv6Frag:

l_paEf2Ipv6Frag_ProcessIp:
    // Ip header has already been loaded 
    //xin     XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
   
l_paEf2Ipv6Frag_ProcessIp_1:
    // Save the base offset
    qbbc     l_paEf2Ipv6Frag_ProcessIp_2,  s_efState2.flags.t_ef_firstLoop         
        mov s_efState2.baseOffset, 0
        add s_efState2.ipLen, s_Ipv6a.payloadLen, IPV6_HEADER_LEN_BYTES
        //sub s_ef2_loc2.totalFragSize, s_efState2.ipLen, s_txPktCxt.ipHdrLen
        //sub s_ef2_loc2.payloadSize,  s_txPktCxt.fragSize, s_txPktCxt.ipHdrLen
        //add s_ipPktId.id, s_ipPktId.id, 1 
    
l_paEf2Ipv6Frag_ProcessIp_2:    
    // Update payloadLen
    sub s_ef2_loc2.payloadSize,  s_ef2_loc2.payloadSize,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES 
    add     r0.w0,  s_ef2_loc2.payloadSize, s_txPktCxt.ipHdrLen
    add     r0.w0,  r0.w0, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    sub     s_Ipv6a.payloadLen, r0.w0, IPV6_HEADER_LEN_BYTES
    xout    XID_CDEDATA, s_Ipv6a.payloadLen, SIZE(s_Ipv6a.payloadLen) 
    
    // assume this is the last fragment
    //set     s_efState2.flags.t_ef_lastFrag

    // If ipHdrLen==IPV6_HEADER_LEN_BYTES, then we still have the original IP header,
    // else we need to patch
    qbeq    l_paEf2Ipv6Frag_only_fixed_header, s_txPktCxt.ipHdrLen, IPV6_HEADER_LEN_BYTES
        // There is non-fragmentable extension header
        // Advance to the extended Header
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  s_txPktCxt.nextHdrOffset
        xout XID_CDECTRL,  s_cdeCmdWd,    4
        
        mov  s_Ipv6Opt.proto,    IP_PROTO_NEXT_IPV6_FRAG 
        xout XID_CDEDATA, s_Ipv6Opt.proto,         SIZE(s_Ipv6Opt.proto)
        
        // Step past the last header we checked
        sub     s_cdeCmdWd.byteCount,    s_txPktCxt.ipHdrLen, s_txPktCxt.nextHdrOffset
        xout    XID_CDECTRL, s_cdeCmdWd, 4 
        
        jmp     l_paEf2Ipv6Frag_Loop

l_paEf2Ipv6Frag_only_fixed_header:
        // Set next header in IPv6 fixed header to frag
        mov     s_Ipv6a.next, IP_PROTO_NEXT_IPV6_FRAG
        xout    XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
        
        // Step past the last header we checked
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  IPV6_HEADER_LEN_BYTES
        xout    XID_CDECTRL, s_cdeCmdWd, 4 

l_paEf2Ipv6Frag_Loop:
    // SOP is at least L2+L3
    add     s_pktExtDescr2.sopLength,   s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset2 
    add     s_pktExtDescr2.sopLength,   s_pktExtDescr2.sopLength, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
      
    // Update and insert the Fragmentation header
    mov     s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
    mov     s_cdeCmdIn2.len,           4 
    lsl     s_cdeCmdIn2.data.w2,       s_txPktCxt.nextHdr,  8      
    mov     s_cdeCmdIn2.data.w0,       s_efState2.loopOffset
    
    // Check wether it is the last fragment
    qbbs    l_paEf2Ipv6Frag_CommonFrag, s_efState2.flags.t_ef_lastFrag
    
        // There are more fragments
        set     s_cdeCmdIn2.data.w0.t_ipv6_frag_m
        //clr     s_efState2.flags.t_ef_lastFrag
   
l_paEf2Ipv6Frag_CommonFrag:
    // Complete the fragmentation header
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    //mov     s_cdeCmdIn2.data, s_ipPktId.id
	lbco  s_cdeCmdIn2.data, cMailbox, 0, 4
    
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      

    // Move the amount we need to copy  and skip into "Remain"
    mov s_ef2_loc2.dataRemain, s_ef2_loc2.payloadSize
    mov s_ef2_loc2.offsetRemain, s_efState2.loopOffset
    
    // See if we still have data remaining in SOP
    qbgt    l_paEf2Ipv6Frag_HaveSOP,   s_ef2_loc2.offsetRemain,   s_pktExtDescr2.sopL4Length
        // Flush past all SOP
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length
        xout XID_CDECTRL,           s_cdeCmdWd,              4    
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_ef2_loc2.offsetRemain,    s_ef2_loc2.offsetRemain, s_pktExtDescr2.sopL4Length
        jmp  l_paEf2Ipv6Frag_PastSOP
            
l_paEf2Ipv6Frag_HaveSOP: 
        qbeq l_paEf2Ipv6Frag_NoFlush, s_efState2.loopOffset, 0       
            // Now flush any data contained in s_efState2.loopOffset
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
            mov  s_cdeCmdWd.byteCount,  s_ef2_loc2.offsetRemain
            xout XID_CDECTRL, s_cdeCmdWd, 4 
            
l_paEf2Ipv6Frag_NoFlush:        
            // Slide the window past the valid data
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            sub  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length, s_ef2_loc2.offsetRemain
            // Trim the SOP to the data size
            qbgt    l_paEf2Ipv6Frag_NoSOPTrim, s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain 
                mov s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain
                set s_efState2.flags.t_ef_withinSop
                
l_paEf2Ipv6Frag_NoSOPTrim:                 
                xout XID_CDECTRL,  s_cdeCmdWd,    4
                mov  s_ef2_loc2.offsetRemain,     0
                sub  s_ef2_loc2.dataRemain,    s_ef2_loc2.dataRemain, s_cdeCmdWd.byteCount
                add  s_pktExtDescr2.sopLength,  s_pktExtDescr2.sopLength,   s_cdeCmdWd.byteCount 
                
l_paEf2Ipv6Frag_PastSOP:
            // Here we're past SOP or we don't have any data left
            // We may need to copy some MOP
            mov s_pktExtDescr2.mopLength, s_ef2_loc2.dataRemain
            qbeq    l_paEf2Ipv6Frag_FragReady, s_ef2_loc2.dataRemain, 0
            
            // see if we have any MOP
            qbgt    l_paEf2Ipv6Frag_HaveMOP,   s_ef2_loc2.offsetRemain, s_pktExtDescr2.mopLengthOrig
                sub s_ef2_loc2.offsetRemain, s_ef2_loc2.offsetRemain, s_pktExtDescr2.mopLengthOrig
                mov s_pktExtDescr2.mopLength, 0
                jmp l_paEf2Ipv6Frag_PastMOP
                
l_paEf2Ipv6Frag_HaveMOP:
            add     s_pktExtDescr2.mopPtr,  s_pktExtDescr2.mopPtrOrig, s_ef2_loc2.offsetRemain 
            // Trim the MOP to the MOP remaining size
            add     r0,  s_ef2_loc2.offsetRemain,  s_ef2_loc2.dataRemain
            qbgt    l_paEf2Ipv6Frag_NoMOPTrim, r0, s_pktExtDescr2.mopLengthOrig
                sub  s_pktExtDescr2.mopLength,  s_pktExtDescr2.mopLengthOrig,   s_ef2_loc2.offsetRemain
                
l_paEf2Ipv6Frag_NoMOPTrim:
                sub  s_ef2_loc2.dataRemain,    s_ef2_loc2.dataRemain, s_pktExtDescr2.mopLength
                mov  s_ef2_loc2.offsetRemain,  0
                           
l_paEf2Ipv6Frag_PastMOP:
            qbeq    l_paEf2Ipv6Frag_NoEOPFlush,    s_ef2_loc2.offsetRemain,   0 
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
                mov  s_cdeCmdWd.byteCount,  s_ef2_loc2.offsetRemain
                xout XID_CDECTRL, s_cdeCmdWd, 4 
                mov  s_ef2_loc2.offsetRemain, 0
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
                // pass through
             
l_paEf2Ipv6Frag_NoEOPFlush:             
            // If there is any EOP data to move, move it now
            qbeq    l_paEf2Ipv6Frag_FragReady, s_ef2_loc2.dataRemain, 0
                // Slide the window past the valid data
                mov s_cdeCmdWd.byteCount, s_ef2_loc2.dataRemain
                xout XID_CDECTRL,  s_cdeCmdWd,    4
            
l_paEf2Ipv6Frag_FragReady:
    // Setup the offset for the next packet 
    add     s_efState2.loopOffset, s_efState2.loopOffset, s_ef2_loc2.payloadSize
    
    // Calc the size for this next loop
    //add     r0, s_efState2.loopOffset, s_ef2_loc2.payloadSize
    //qbge    l_paEf2Ipv6Frag_NoAdjust, r0, s_ef2_loc2.totalFragSize
    //    sub s_ef2_loc2.payloadSize, s_ef2_loc2.totalFragSize, s_efState2.loopOffset 
    
l_paEf2Ipv6Frag_NoAdjust:
    
    // Flush out the remaining packet 
    mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,             s_cdeCmdPkt,             4
    
f_paEf2InsEspTrail:
    // Insert Insert ESP trailer and reserved ICV location if required
    qbbc   f_paEf2_ForwardPkt, s_efRec2.ctrlFlags.t_ef_rec2_ins_ipsec_trail
        // Insert the fixed padding pattern 0x01 0x02 0x03 ...
        mov  s_cdeCmdInD.operation,     CDE_CMD_INSERT_PACKET_BUFFER
        mov  s_cdeCmdInD.lenLsb,        s_efState2.paddingLen
        mov  s_cdeCmdInD.lenMsbs,       0
        mov  s_cdeCmdInD.dataP,         PAMEM_CONST_ESP_PADDING_BUF_BASE
        xout XID_CDECTRL,               s_cdeCmdInD,         SIZE(s_cdeCmdInD)
        
        // Insert the padding length, Protocol and ICV 
        zero &s_cdeCmdIn,   SIZE(s_cdeCmdIn)
        mov  s_cdeCmdIn.operation,     CDE_CMD_INSERT_PACKET 
        add  s_cdeCmdIn.len,           s_efRec2.icvSize, 2
        mov  s_cdeCmdIn.data3,         s_efState2.paddingLen
        mov  s_cdeCmdIn.data2,         s_efState2.nextHdr
        xout XID_CDECTRL, s_cdeCmdIn,  SIZE(s_cdeCmdIn)   
        
        qbbc f_paEf2_ForwardPkt,    s_efState2.flags.t_ef_withinSop
            clr s_efState2.flags.t_ef_withinSop
            add r0.b0, s_cdeCmdIn.len,  s_efState2.paddingLen
            add s_pktExtDescr2.sopLength,  s_pktExtDescr2.sopLength, r0.b0        
    
f_paEf2_ForwardPkt:
    // reset control flags
    clr s_efState2.flags.t_ef_firstLoop    
    
    //wbs   s_flags.info.tStatus_CDEOutPacket
    // update swInfo
    add     r0.w0,  r28.w0, PA_EF_REC2_SWINFO0_OFFSET
    lbco    r6, PAMEM_CONST_EF_RECORD2, r0.w0, 8
    sbco    r6, cCdeOutPkt, OFFSET(s_pktDescr.swinfo0), 8
    
    // update pktCxt
    sbco s_txPktCxt, cCdeOutPkt, SIZE(s_pktDescr), OFFSET(s_txPktCxt.fragSize)   

    zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.destQueue,    s_efRec2.queueId
    //sbco s_efRec2.queueId,            cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
    mov  s_cdeCmdPkt.flowId,       s_efRec2.flowId
    mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_txPktCxt) + 4) & 0xF8
    qbbc l_paEf2_ForwardPkt_1,      s_txPktCxt.flags.t_flag_ah_patch
        // AH tag patch required: Adjust psInfoSize to be multiple of 16 due to SA output alignment problem
        mov  s_cdeCmdPkt.psInfoSize,   32
    
        // pass through
    
        
l_paEf2_ForwardPkt_1:    
    qbbs l_paEf2IpFrag_LastPacket, s_efState2.flags.t_ef_lastFrag
        mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
        mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_RETAIN_CHKSUM | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
        //mov  r4.w0,  CDE_CMD_PACKET_COPY | ((CDE_FLG_RETAIN_CHKSUM | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8)
        xout XID_CDECTRL,   s_cdeCmdPkt,    SIZE(s_cdeCmdPkt)
        xout XID_PINFO_DST, s_pktExtDescr2, SIZE(s_pktExtDescr2)   // Send the extended info

    // pass through

l_paEf2_ForwardPkt_WaitForCopy:
    // Wait for a new packet
    wbs s_flags.info.tStatus_CDENewPacket
        
    // Read packet descriptor
    // 
    // xin  XID_CDEDATA,  s_pktDescr,  OFFSET(s_pktDescr.pktDataSize) 
   
    // Advance to the control section
    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout XID_CDECTRL,            s_cdeCmdWd,            4
    
    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    mov s_cdeInsert.byteCount,  32
    xout XID_CDECTRL,           s_cdeCmdWd,             4
    
    // Re-load the packet Info
    xin  XID_CDEDATA,  s_txPktCxt,   SIZE(s_txPktCxt)

    // Flush out the control info
    mov  s_cdeCmdWd.operation,   CDE_CMD_FLUSH_TO_PACKET
    xout XID_CDECTRL,            s_cdeCmdWd,            4
    
    // Is this the last fragment
    add     r0, s_efState2.loopOffset, s_ef2_loc2.payloadSize
    sub     r1, s_efState2.ipLen, s_txPktCxt.ipHdrLen
    
    qblt    l_paEfRec2Proc_loop, r1, r0
   
        // This is the last frag - adjust the final size
        set     s_efState2.flags.t_ef_lastFrag
        //sub     s_ef2_loc2.payloadSize, r1, s_efState2.loopOffset
        //pass through
   
    jmp  l_paEfRec2Proc_loop

    .leave ef2ScopeLoc2

l_paEf2IpFrag_LastPacket:
    mov  s_pktExtDescr2.flags, s_efState2.extHdrFlags 
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    //mov  r4.w0,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8)
    xout XID_CDECTRL,   s_cdeCmdPkt,    SIZE(s_cdeCmdPkt)
    xout XID_PINFO_DST, s_pktExtDescr2, SIZE(s_pktExtDescr2)   // Send the extended info
    
    jmp  f_mainLoop
    
    
//
// Packet forwarding utility function
//   

f_paEfRec2_pktfwd:        
    
        // Forward the packet for next stage operation
        // TBD: may be enhnaced to be a more general utility function
        wbs   s_flags.info.tStatus_CDEOutPacket
        sbco  s_txPktCxt, cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_txPktCxt)
        
        // Forward the packet
        // The next stage will be either Egress1 or Egress2
        qbbs  l_paEfRec2_pktfwd_1, s_txPktCxt.flags.t_flag_ef_rec_valid_lvl3
            mov  s_pktExtDescr.threadId, THREADID_EGRESS2
            jmp  l_paEfRec2_pktfwd_2
        
l_paEfRec2_pktfwd_1: 
            mov  s_pktExtDescr.threadId, THREADID_EGRESS1
            // pass through       
        
l_paEfRec2_pktfwd_2:
        
        xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
            
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
        mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
        xout XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
    
        jmp  f_mainLoop

    
// *******************************************************************************
// * FUNCTION PURPOSE: Calculate IP length and update the protocol header
// *******************************************************************************
// * DESCRIPTION: Calculate IP length and update the protocol header
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:          | r1.b3: next protocol Reserved
// *   R2:        | EF2 loc1
// *   R3:        | 
// *   R4:   | s_efFragInfo 
// *   R5:   |                   -
// *   R6:        |        
// *   R7:        |  L3 Header                            
// *   R8:        |  (Temporary data)                           
// *   R9:        |  
// *   R10:       |  
// *   R11:         |  
// *   R12:         |  Record Info
// *   R13:         |   
// *   R14:            |                                            
// *   R15:            | Extended Descriptor                                           
// *   R16:            |                                            
// *   R17:            |                         
// *   R18:            | 
// *   R19:       |     
// *   R20:       |  Record State machine (s_efState)   | s_modState will be overwritten
// *   R21:       |                                     | for EF Record processing
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:  r28.w0: record offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    
    
f_paEf2CalIPLengthAndUpdate:
    .using   ef2ScopeLoc1
    mov     r1.b1, s_efState2.nextHdr
    qbbs    l_paEf2CalIPLengthAndUpdate_1,   s_efRec2.ctrlFlags.t_ef_rec2_ipsec_proc
        // No extra processing is required
        jmp     l_paEf2CalIPLengthAndUpdate_ipFrag    
    
l_paEf2CalIPLengthAndUpdate_1:    
    // IPSEC can not be applied to fragmented packet
    qbne    l_paEf2CalIPLengthAndUpdate_1_1, s_efState2.nextHdr,  IP_PROTO_NEXT_IPV6_FRAG
        // Ipv6 fragmentation
        mov  s_txPktCxt2.eId,   EF_EROUTE_IP_FRAG
        jmp  f_efForwardExp                                // no return
    
l_paEf2CalIPLengthAndUpdate_1_1:    
    qbbs    l_paEf2CalIPLengthAndUpdate_ah,  s_efRec2.ctrlFlags.t_ef_rec2_ipsec_ah
    
l_paEf2CalIPLengthAndUpdate_esp:
    mov r1.b1, IP_PROTO_NEXT_ESP
    qbbs    l_paEf2CalIPLengthAndUpdate_esp_1, s_efState2.flags.t_ef2_outer_ipv6  
        mov s_Ip.protocol,  IP_PROTO_NEXT_ESP
        jmp l_paEf2CalIPLengthAndUpdate_esp_2   
l_paEf2CalIPLengthAndUpdate_esp_1:
        qbeq l_paEf2CalIPLengthAndUpdate_esp_1_1,   s_ef2_loc1.ipHdrLen, 40
            mov s_Ipv6Opt.proto, IP_PROTO_NEXT_ESP
            jmp l_paEf2CalIPLengthAndUpdate_esp_2
l_paEf2CalIPLengthAndUpdate_esp_1_1:        
            mov s_Ipv6a.next,   IP_PROTO_NEXT_ESP
            // pass through

l_paEf2CalIPLengthAndUpdate_esp_2:    
    add     s_ef2_loc1.ipsecHdrLen, s_efRec2.ivSize,    8
    
    qblt    l_paEf2CalIPLengthAndUpdate_esp_3,   s_efRec2.encBlkSize,    1 
        mov s_efState2.paddingLen,  1
        jmp l_paEf2CalIPLengthAndUpdate_esp_4
        
l_paEf2CalIPLengthAndUpdate_esp_3:
        // Calculate padding length
        add s_efState2.paddingLen,  s_ef2_loc1.payLoadLen.b0,   2
        sub r0.b0,  s_efRec2.encBlkSize, 1
        and s_efState2.paddingLen,  s_efState2.paddingLen, r0.b0
        sub s_efState2.paddingLen,  s_efRec2.encBlkSize,   s_efState2.paddingLen
        
l_paEf2CalIPLengthAndUpdate_esp_4:
        // Update IP payload length        
        add s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen, s_efState2.paddingLen
        add s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen, 2
        add s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen, s_ef2_loc1.ipsecHdrLen
        add s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen, s_efRec2.icvSize
        jmp l_paEf2CalIPLengthAndUpdate_ipFrag
    
l_paEf2CalIPLengthAndUpdate_ah:
    mov r1.b1, IP_PROTO_NEXT_AUTH
    qbbs    l_paEf2CalIPLengthAndUpdate_ah_1, s_efState2.flags.t_ef2_outer_ipv6  
        mov s_Ip.protocol,  IP_PROTO_NEXT_AUTH
        jmp l_paEf2CalIPLengthAndUpdate_ah_2   
l_paEf2CalIPLengthAndUpdate_ah_1:
        qbeq l_paEf2CalIPLengthAndUpdate_ah_1_1,   s_ef2_loc1.ipHdrLen, 40
            mov s_Ipv6Opt.proto, IP_PROTO_NEXT_AUTH
            jmp l_paEf2CalIPLengthAndUpdate_ah_2
l_paEf2CalIPLengthAndUpdate_ah_1_1:        
            mov s_Ipv6a.next,   IP_PROTO_NEXT_AUTH
            // pass through

l_paEf2CalIPLengthAndUpdate_ah_2:    
    add     s_ef2_loc1.ipsecHdrLen, s_efRec2.ivSize,           12
    add     s_ef2_loc1.ipsecHdrLen, s_ef2_loc1.ipsecHdrLen,    s_efRec2.icvSize
    qbbc    l_paEf2CalIPLengthAndUpdate_ah_3, s_efState2.flags.t_ef2_outer_ipv6
        // adjust ipsecHdrLen
        add s_ef2_loc1.ipsecHdrLen, s_ef2_loc1.ipsecHdrLen,    4
        and s_ef2_loc1.ipsecHdrLen, s_ef2_loc1.ipsecHdrLen,    0xF8
    
l_paEf2CalIPLengthAndUpdate_ah_3:    
    add     s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen,     s_ef2_loc1.ipsecHdrLen
    //jmp     l_paEf2CalIPLengthAndUpdate_ipFrag  
    
l_paEf2CalIPLengthAndUpdate_ipFrag:
    // Initialize fragment info
    qbbc   l_paEf2CalIPLengthAndUpdate_ipFrag_1, s_efRec2.ctrlFlags.t_ef_rec2_single_ip
          sub  s_ef2_loc1.payLoadLen,  s_ef2_loc1.payLoadLen,   s_ef2_loc1.ipHdrLen
            
l_paEf2CalIPLengthAndUpdate_ipFrag_1:            
    zero   &s_efFragInfo, SIZE(s_efFragInfo)    
    //mov    s_efFragInfo.nextHdr,  s_efState2.nextHdr
    mov    s_efFragInfo.nextHdr,  r1.b1 
    mov    s_efFragInfo.ipHdrLen, s_ef2_loc1.ipHdrLen
    add    s_efFragInfo.fragSize, s_ef2_loc1.ipHdrLen,  s_ef2_loc1.payLoadLen
    mov    s_efFragInfo.lastFragSize, s_efFragInfo.fragSize
    qbeq   l_paEf2CalIPLengthAndUpdate_ipFrag_2, s_ef2_loc1.ipHdrLen, 40
        sub s_efFragInfo.nextHdrOffset, s_ef2_loc1.ipHdrLen, r1.b0 
        jmp l_paEf2CalIPLengthAndUpdate_ipFrag_3
l_paEf2CalIPLengthAndUpdate_ipFrag_2:    
        mov s_efFragInfo.nextHdrOffset, OFFSET(s_Ipv6a.next)
        //pass through
l_paEf2CalIPLengthAndUpdate_ipFrag_3:        
    clr    s_txPktCxt.flags.t_flag_ip_frag 
    // TBD: Fragmented packet can not be fragmented
    qbeq   l_paEf2CalIPLengthAndUpdate_ipFrag_end,  s_efState2.nextHdr,  IP_PROTO_NEXT_IPV6_FRAG
    qbbc   l_paEf2CalIPLengthAndUpdate_ipFrag_end,  s_efRec2.ctrlFlags.t_ef_rec2_ip_mtu
    qbge   l_paEf2CalIPLengthAndUpdate_ipFrag_end,  s_efFragInfo.fragSize,  s_efRec2.mtu 
        
        qbbs l_paEf2CalIPLengthAndUpdate_ipv6Frag, s_efState2.flags.t_ef2_outer_ipv6
        
l_paEf2CalIPLengthAndUpdate_ipv4Frag:  
            // Outer IP fragmentation is required 
            set s_txPktCxt.flags.t_flag_ip_frag
      
            // local variable: payloadSize = r0.w0
            //                 ipLen = r0.w2
            sub r0.w0, s_efRec2.mtu,   s_efFragInfo.ipHdrLen
            and r0.b0, r0.b0, 0xF8
            mov r0.w2, s_ef2_loc1.payLoadLen
            add s_efFragInfo.fragSize,   r0.w0, s_efFragInfo.ipHdrLen  
l_paEf2CalIPLengthAndUpdate_ipv4Frag_loop:
            sub  r0.w2, r0.w2, r0.w0
            qblt l_paEf2CalIPLengthAndUpdate_ipv4Frag_loop, r0.w2, r0.w0  
                // last fragment
                add s_efFragInfo.lastFragSize, r0.w2, s_efFragInfo.ipHdrLen
                jmp l_paEf2CalIPLengthAndUpdate_ipFrag_end
            // end of the loop
            
l_paEf2CalIPLengthAndUpdate_ipv6Frag:      
            add    r0, s_efFragInfo.ipHdrLen,    IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
            // TBD: If the non-fragmentable portion consumes the full mtu, we can not frag
            qbge   l_paEf2CalIPLengthAndUpdate_ipFrag_end, s_efRec2.mtu, r0
            
            // IPv6 Fragmentation is required
            set  s_txPktCxt.flags.t_flag_ip_frag
    
            // local variable: payloadSize r0.w0: 
            //                 ipLen (totalFragSize) r0.w2: ipLen - ipHdrLen
            and s_efRec2.mtu.b0,   s_efRec2.mtu.b0,  0xF8
            sub r0.w0, s_efRec2.mtu,   s_efFragInfo.ipHdrLen
            sub r0.w0, r0.w0, 8
            mov r0.w2, s_ef2_loc1.payLoadLen
            mov s_efFragInfo.fragSize,   s_efRec1.mtu 
l_paEf2CalIPLengthAndUpdate_ipv6Frag_loop:
            sub  r0.w2, r0.w2, r0.w0
            qblt l_paEf2CalIPLengthAndUpdate_ipv6Frag_loop, r0.w2, r0.w0  
                // last fragment
                add s_efFragInfo.lastFragSize, r0.w2, s_efFragInfo.ipHdrLen
                add s_efFragInfo.lastFragSize, s_efFragInfo.lastFragSize, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
                    
l_paEf2CalIPLengthAndUpdate_ipFrag_end:
    //update the fragInfo of packetInfo at the output packet
    wbs  s_flags.info.tStatus_CDEOutPacket
    sbco s_efFragInfo, cCdeOutPkt, SIZE(s_pktDescr) + OFFSET(s_txPktCxt.fragSize), SIZE(s_efFragInfo)   
    
    // TBD: Adjust endOffset
    add  s_txPktCxt.endOffset,  s_ef2_loc1.ipHdrLen,  s_ef2_loc1.payLoadLen
    add  s_txPktCxt.endOffset,  s_txPktCxt.endOffset, s_txPktCxt.l3Offset
    
    ret  
    
    .leave     ef2ScopeLoc1
        
    .leave cdeScope
    .leave pktScope 
    .leave ipScope
    
#endif   


// *******************************************************************************
// * FUNCTION PURPOSE: Record 2 Processing
// *******************************************************************************
// * DESCRIPTION: Process the egress flow record 3 
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:      | Temp and local variables
// *   R2:      | 
// *   R3:      | 
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |        
// *   R7:        |  L3 Header                            
// *   R8:        |  (Temporary data)                           
// *   R9:        |  
// *   R10:       |  
// *   R11:         |  
// *   R12:         |  Record Info
// *   R13:         |   
// *   R14:            |                                            
// *   R15:            | Extended Descriptor                                           
// *   R16:            |                                            
// *   R17:            |                         
// *   R18:            | 
// *   R19:       |     
// *   R20:       |  Record State machine (s_efState)   | s_modState will be overwritten
// *   R21:       |                                     | for EF Record processing
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:  r28.w0: record offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
#ifdef PASS_PROC_EF_REC3

    .using cdeScope
    .using pktScope     // For offset info
    .using ipScope
 
// Egress Flow record3 processing route main entry

f_paEfRec3Proc:

    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,                SIZE(s_cdeCmdWd)
    
    // Restore the latest extended Packet Info (Extended Packet Info is already loaded by main loop)
    // xin  XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  
    
    qbbc  f_paEfRec3_pktfwd, s_txPktCxt.flags.t_flag_ef_rec_valid_lvl3
        
l_paEfRec3Proc_1:
    // Calculate and record the record3 offset
    lsl  r28.w0,      s_txPktCxt.lvl3RecIndex,   5
    lbco s_efRec3Ah,  PAMEM_CONST_EF_RECORD3,    r28.w0,     SIZE(s_efRec3Ah)
    
    // TBD: Verify whetehr the record is valid
    
    // Global initialization prior to the record processing loop
    zero  &s_efState3,  SIZE(s_efState3)
    
    // Move to outer IP header  */
    // Advance the CDE to the IP header to examine the packet
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset
    xout XID_CDECTRL,           s_cdeCmdWd,     SIZE(s_cdeCmdWd)
    
l_paEfRec3Proc_calcHdrLen:      
    qbbs l_paEfRec3Proc_calcHdrLen_ah,  s_efRec3Ah.ctrlFlags.t_ef_rec3_ipsec_ah
l_paEfRec3Proc_calcHdrLen_nat:
        mov     s_efState3.proto,   IP_PROTO_NEXT_UDP
        mov     s_efState3.hdrSize,  8
        jmp     l_paEfRec3Proc_calcHdrLen_end
    
l_paEfRec3Proc_calcHdrLen_ah:  
        mov     s_efState3.proto,   IP_PROTO_NEXT_AUTH
        add     s_efState3.hdrSize,  s_efRec3Ah.ivSize,           12
        add     s_efState3.hdrSize,  s_efState3.hdrSize,          s_efRec3Ah.icvSize
        // pass through    
    
l_paEfRec3Proc_calcHdrLen_end:    
    
l_paEfRec3Proc_update_ip:    
    //
    // Update outer IP header
    //  
    
    // Update outer IP
    // Read the minimum IP header
    xin  XID_CDEDATA,  s_Ip,   OFFSET(s_Ip.srcIp) 
    and  r0.b0,  s_Ip.verLen,  0xF0
    
    qbeq l_paEf3UpdateIpv4,  r0.b0,       0x40
    qbeq l_paEf3UpdateIpv6,  r0.b0,       0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
l_paEf3UpdateIpv4:
    //mov  s_efState3.nextHdr,    s_Ip.protocol
    //mov  s_efState3.ipHdrLen,   s_txPktCxt.ipHdrLen
    sub  s_efState3.payloadLen,   s_Ip.totalLen,    s_txPktCxt.ipHdrLen    
    add  s_efState3.payloadLen,   s_efState3.payloadLen, s_efState3.hdrSize 
    
    qbbs l_paEf3UpdateIp_end, s_efRec3Ah.ctrlFlags.t_ef_rec3_replace_hdr
        // Update IP header and re-calculate checksum
        mov   s_Ip.protocol,  s_efState3.proto
        add   s_Ip.totalLen,  s_Ip.totalLen,  s_efState3.hdrSize 
        
        // Issue checksum command
        zero  &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
        mov   s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
        mov   s_cdeCmdChk.byteLen,  s_txPktCxt.ipHdrLen
        mov   s_cdeCmdChk.offset, OFFSET(s_Ip.checksum)          // Starting sum is 0, offset is 10
        xout  XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
        
        mov   s_Ip.checksum, 0
        xout  XID_CDEDATA, s_Ip, OFFSET(s_Ip.srcIp)
        
        jmp   l_paEf3UpdateIp_end
        
            
l_paEf3UpdateIpv6:
    //mov  s_efState3.nextHdr,    s_Ipv6a.next
    //mov  s_efState3.ipHdrLen,   IPV6_HEADER_LEN_BYTES
    // IPv6 header length should be 8-byte aligned
    // Note: UDP length = 8 
    add s_efState3.hdrSize, s_efState3.hdrSize, 4
    and s_efState3.hdrSize, s_efState3.hdrSize, 0xF8 
    
    set s_efState3.flags.t_ef3_ipv6
    mov s_efState3.payloadLen,  s_Ipv6a.payloadLen
    // Update parameters
    qbbs l_paEf3UpdateIp_end, s_efRec3Ah.ctrlFlags.t_ef_rec3_replace_hdr
        // Update IP header
        add s_Ipv6a.payloadLen, s_Ipv6a.payloadLen, s_efState3.hdrSize
        add s_efState3.payloadLen,  s_efState3.payloadLen, s_efState3.hdrSize 
        
        qble l_paEf3UpdateIpv6_update_1, s_txPktCxt.nextHdrOffset,   IPV6_HEADER_LEN_BYTES
            mov s_Ipv6a.next,   s_efState3.proto
            xout  XID_CDEDATA,  s_Ipv6a, SIZE(s_Ipv6a)
            jmp   l_paEf3UpdateIp_end
            
l_paEf3UpdateIpv6_update_1:
            // advance to the next header prior to extension header
            xout XID_CDEDATA,  s_Ipv6a, SIZE(s_Ipv6a)
            mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
            mov  s_cdeCmdWd.byteCount,   s_txPktCxt.nextHdrOffset
            xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
            
            // replace protocol of next header
            mov  s_Ipv6Opt.proto,  s_efState3.proto
            xout XID_CDEDATA,  s_Ipv6Opt.proto, SIZE(s_Ipv6Opt.proto)
            
            // advance to where new header should be inserted
            sub  s_cdeCmdWd.byteCount,   s_txPktCxt.ipHdrLen, s_txPktCxt.nextHdrOffset
            xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
            jmp  l_paEf3PostIpProc
            
l_paEf3UpdateIp_end:
        mov  s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,   s_txPktCxt.ipHdrLen
        xout XID_CDECTRL,            s_cdeCmdWd,     SIZE(s_cdeCmdWd)
            
        //pass through    
            
l_paEf3PostIpProc:
        mov  s_efState3.nextHdr,    s_txPktCxt.nextHdr 
        mov  s_txPktCxt.nextHdr,    s_efState3.proto
    
        qbbs l_paEf3PostIpProc_1, s_efRec3Ah.ctrlFlags.t_ef_rec3_replace_hdr  
            // New Header should be inserted, update SOP length and re-calculate the last fragment size  
            add s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_efState3.hdrSize
            add s_txPktCxt.l3Offset2, s_txPktCxt.l3Offset2,  s_efState3.hdrSize
            add s_txPktCxt.l4Offset,  s_txPktCxt.l4Offset,   s_efState3.hdrSize
            add s_txPktCxt.endOffset, s_txPktCxt.endOffset,  s_efState3.hdrSize
            
            //adjust espAhOffset only if NAT-T
            qbbs l_paEf3PostIpProc_0, s_efRec3Ah.ctrlFlags.t_ef_rec3_ipsec_ah
                add s_txPktCxt.espAhOffset, s_txPktCxt.espAhOffset, 8
                //pass through  
l_paEf3PostIpProc_0:
            qbbc    l_paEf3PostIpProc_ipFrag_1,  s_efRec3Ah.ctrlFlags.t_ef_rec3_ip_mtu
                qbbs l_paEf3PostIpProc_ipFrag_2,    s_txPktCxt.flags.t_flag_ip_frag
                    // whether we need to enable fragmentation               
                    add  r0.w0, s_txPktCxt.fragSize, s_efState3.hdrSize
                    qbge l_paEf3PostIpProc_ipFrag_1, r0.w0, s_efRec3Ah.mtu 
                        // enable fragmentation
                        set s_txPktCxt.flags.t_flag_ip_frag
                        and s_efRec3Ah.mtu.b0, s_efRec3Ah.mtu.b0, 0xF8
                        mov s_txPktCxt.fragSize,  s_efRec3Ah.mtu
                        sub s_txPktCxt.lastFragSize, r0.w0, s_efRec3Ah.mtu
                        add s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, s_txPktCxt.ipHdrLen
                        qbbc l_paEf3PostIpProc_2,   s_efState3.flags.t_ef3_ipv6
                            add  s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, 2*IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
                            jmp  l_paEf3PostIpProc_2
                    
l_paEf3PostIpProc_ipFrag_1:
                        // just update the packet length
                        add  s_txPktCxt.fragSize, s_txPktCxt.fragSize, s_efState3.hdrSize
                        mov  s_txPktCxt.lastFragSize, s_txPktCxt.fragSize
                        jmp  l_paEf3PostIpProc_2  
                        
l_paEf3PostIpProc_ipFrag_2:
                    // Outer IP fragmentation is already required, adjust lastFragSize
                    and s_efRec3Ah.mtu.b0, s_efRec3Ah.mtu.b0, 0xF8
                    add s_txPktCxt.lastFragSize,    s_txPktCxt.lastFragSize, s_efState3.hdrSize
                    qbge l_paEf3PostIpProc_2, s_txPktCxt.lastFragSize, s_efRec3Ah.mtu      
                        sub  s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, s_efRec3Ah.mtu
                        add  s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, s_txPktCxt.ipHdrLen
                        qbbc l_paEf3PostIpProc_2,   s_efState3.flags.t_ef3_ipv6
                            add  s_txPktCxt.lastFragSize, s_txPktCxt.lastFragSize, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
                            jmp  l_paEf3PostIpProc_2

                            
l_paEf3PostIpProc_1:
        // Flush out the original header
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_efState3.hdrSize
        xout XID_CDECTRL,           s_cdeCmdWd,             4    

        //pass through            
                            
l_paEf3PostIpProc_2:  
        // AH/NAT-T header processing
        qbbc l_paEfRec3Proc_nat,  s_efRec3Ah.ctrlFlags.t_ef_rec3_ipsec_ah
        
l_paEfRec3Proc_ah: 
        // AH processing
        // Insert AH header
        // Next Hdr, length, 0, SPI, 0 ...
        mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
        mov  s_cdeCmdIn2.len,           4 
        mov  s_cdeCmdIn2.data.w0,       0
        mov  s_cdeCmdIn2.data.b3,       s_efState3.nextHdr
        lsr  s_cdeCmdIn2.data.b2,       s_efState3.hdrSize,     2
        sub  s_cdeCmdIn2.data.b2,       s_cdeCmdIn2.data.b2,    2
        xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
        
        add  r0.w0, r28.w0, PA_EF_REC3_SPI_OFFSET
        lbco s_cdeCmdIn2.data,  PAMEM_CONST_EF_RECORD3, r0.w0,  4
        xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
        
        // Insert 0s
        sub  s_cdeCmdIn2.len,           s_efState3.hdrSize, 8 
        xout XID_CDECTRL, s_cdeCmdIn2,  4 
        
        // Update the Ah header
        add s_txPktCxt.espAhOffset, s_txPktCxt.l3Offset, s_txPktCxt.ipHdrLen
        set s_txPktCxt.flags.t_flag_ah_patch
        
        // icvSize = 8 + r1r0 * 4 (8, 12, 16)
        and s_txPktCxt.flags.b0, s_txPktCxt.flags.b0, NOT_PA_EF_ICV_SIZE_MASK
        sub r0.b0,  s_efRec3Ah.icvSize, 8
        lsr r0.b0,  r0.b0,  2
        or  s_txPktCxt.flags.b0,    s_txPktCxt.flags.b0, r0.b0
            
        // Forward the packet to SASS    
            
        wbs   s_flags.info.tStatus_CDEOutPacket
        // update swInfo
        add     r0.w0,  r28.w0, PA_EF_REC3_SWINFO0_OFFSET
        lbco    r6, PAMEM_CONST_EF_RECORD3, r0.w0, 8
        sbco    r6, cCdeOutPkt, OFFSET(s_pktDescr.swinfo0), 8
    
        // update pktCxt
        sbco s_txPktCxt, cCdeOutPkt, SIZE(s_pktDescr), SIZE(s_txPktCxt) 

        // CDE workaround: do not use CDE_FLG_SET_DESTQUEUE
        //zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
        //mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
        //mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
        mov  s_cdeCmdPkt.destQueue,    s_efRec3Ah.queueId
        ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        //ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO | CDE_FLG_SET_FLOWID) << 8) 
        //sbco s_efRec3Ah.queueId,         cCdeOutPkt, OFFSET(s_pktDescr.destQ),  SIZE(s_pktDescr.destQ)  
        mov  s_cdeCmdPkt.flowId,       s_efRec3Ah.flowId
        // AH tag patch required: Adjust psInfoSize to be multiple of 16 due to SA output alignment problem
        //mov  s_cdeCmdPkt.psInfoSize,   (SIZE(s_txPktCxt) + 4) & 0xF8
        mov  s_cdeCmdPkt.psInfoSize,   32
        
        qbbs l_paEfRec3Proc_ah_1,      s_efRec3Ah.ctrlFlags.t_ef_rec3_loc_dma
            // Global DMA
            mov  s_pktExtDescr.threadId, THREADID_CDMA0
            jmp  l_paEfRec3Proc_ah_2 
l_paEfRec3Proc_ah_1:
            mov  s_pktExtDescr.threadId, THREADID_CDMA1
            // pass through
l_paEfRec3Proc_ah_2:        
        
        xout XID_CDECTRL,   s_cdeCmdPkt,    SIZE(s_cdeCmdPkt)
        xout XID_PINFO_DST, s_pktExtDescr , SIZE(s_pktExtDescr)   // Send the extended info
        jmp  f_mainLoop
                                                            
l_paEfRec3Proc_nat:
        // NAT-T processing
        // Insert NAT-T (UDP) header
        mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
        mov  s_cdeCmdIn2.len,           4 
        mov  s_cdeCmdIn2.data.w2,       s_efRec3Nat.srcPort
        mov  s_cdeCmdIn2.data.w0,       s_efRec3Nat.dstPort
        xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
        
        
        mov  s_cdeCmdIn2.data.w2,       s_efState3.payloadLen
        mov  s_cdeCmdIn2.data.w0,       0
        xout XID_CDECTRL, s_cdeCmdIn2,  SIZE(s_cdeCmdIn2)  
        
        // Forward the packet to Egress2
        //jmp f_paEfRec3_pktfwd  // No return    
        
//
// Packet forwarding utility function
//   

f_paEfRec3_pktfwd:        
    
    // Forward the packet for next stage operation
    // TBD: may be enhnaced to be a more general utility function
    wbs   s_flags.info.tStatus_CDEOutPacket
    sbco  s_txPktCxt, cCdeOutPkt,  SIZE(s_pktDescr),  SIZE(s_txPktCxt)
    
    // Forward the packet
    // The next stage will be Egress2
    mov  s_pktExtDescr.threadId, THREADID_EGRESS2
    
    xout XID_PINFO_DST, s_pktExtDescr, SIZE(s_pktExtDescr)
        
    ldi  r4,  CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_PSINFO) << 8) 
    mov  s_cdeCmdPkt.psInfoSize,  (SIZE(s_txPktCxt) + 7) & 0xf8  // Round up to multiple of 8 bytes
    
        // pass through
l_paEfRec3_pktfwd_1:    
    xout XID_CDECTRL,   s_cdeCmdPkt,   SIZE(s_cdeCmdPkt)
    
    jmp  f_mainLoop
        
    .leave cdeScope
    .leave pktScope 
    .leave ipScope
        
#endif   

// *******************************************************************************
// * FUNCTION PURPOSE: Record 4 Processing
// *******************************************************************************
// * DESCRIPTION: Process the egress flow record 4 
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:      | Temp and local variables
// *   R2:      | 
// *   R3:      | 
// *   R4:    |  CDE commands     -  cdeScope
// *   R5:    |                   -
// *   R6:        |        
// *   R7:        |  L3 Header                            
// *   R8:        |  (Temporary data)                           
// *   R9:        |  
// *   R10:       |  
// *   R11:         |  
// *   R12:         |  Record Info
// *   R13:         |   
// *   R14:            |                                            
// *   R15:            | Extended Descriptor                                           
// *   R16:            |                                            
// *   R17:            |                         
// *   R18:            | 
// *   R19:       |     
// *   R20:       |  Record State machine (s_efState)
// *   R21:       |
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:  r28.w0: record offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************
    
#ifdef PASS_PROC_EF_REC4

    .using cdeScope
    .using pktScope     // For offset info
    .using ipScope

//
// Define several groups of Local variables R2-R3 
//

.struct struct_ef4_loc1
    .u8     l2pktOffset
    .u8     insertCnt
    .u16    dataRemain
    .u16    offsetRemain
    .u16    payloadSize        // valid for multiple loops
    .u16    totalFragSize      // valid for multiple lopps
.ends

.enter   ef4ScopeLoc1
    .assign struct_ef4_loc1,    r1.w0,     r3,        s_ef4_loc1
.leave   ef4ScopeLoc1 

// Egress Flow record2 processing route main entry

f_paEfRec4Proc:
    qbbs  l_paEfRec4Proc_1, s_txPktCxt.flags.t_flag_ef_rec_valid_lvl4
        // There is no valid record4
        mov  s_txPktCxt2.eId,   EF_EROUTE_INVALID_REC
        jmp  f_efForwardExp     

l_paEfRec4Proc_1:
    // Restore the latest extended Packet Info (Extended Packet Info is already loaded by main loop)
    // xin  XID_PINFO_A, s_pktExtDescr, SIZE(s_pktExtDescr)  

    qbbc  l_paEfRec4Proc_2,      s_txPktCxt.flags.t_flag_ah_patch
        // Scroll past and flush out packet Info
        mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH
        //mov   s_cdeCmdWd.byteCount,  (SIZE(s_txPktCxt) + 7) & 0xF8
        sub  s_cdeCmdWd.byteCount,  s_pktDescr.ctrlDataSize,    16 
        xout  XID_CDECTRL,           s_cdeCmdWd,                4
    
        // Record AH patch data
        xin  XID_CDEDATA,  r6,   16
        sbco r6,    PAMEM_CONST_TEMP_BUF, OFFSET_TEMP_BUF_A, 16       

        // pass through
        
l_paEfRec4Proc_2:
    // Scroll past and flush the control info
    mov   s_cdeCmdWd.operation,  CDE_CMD_FLUSH_TO_PACKET
    xout  XID_CDECTRL,           s_cdeCmdWd,            4
    
    // Calculate and record the record4 offset
    lsl  r28.w0,    s_txPktCxt.lvl4RecIndex,   6
    lbco s_efRec4,  PAMEM_CONST_EF_RECORD4,    r28.w0,     SIZE(s_efRec4)
    
    // TBD: Verify whetehr the record is valid
    
    // Global initialization prior to the record processing loop
    zero  &s_efState4,  SIZE(s_efState4)
    mov   s_efState4.extHdrFlags, s_pktExtDescr2.flags 
    mov   s_pktExtDescr2.flags, 0
    mov   s_txPktCxt3.vlanPri,  0
    
    // Setup MOP tracking
    mov   s_pktExtDescr2.mopLengthOrig,   s_pktExtDescr2.mopLength
    mov   s_pktExtDescr2.mopPtrOrig,      s_pktExtDescr2.mopPtr
    
    mov   s_efState4.origSopLen, s_pktExtDescr2.sopLength  
    
    add   r0, s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset 
    sub   s_pktExtDescr2.sopL4Length, s_pktExtDescr2.sopLength, r0
    
    set  s_efState4.flags.t_ef_firstLoop 
    set  s_efState4.flags.t_ef_msgLen_update
    
    // Initialize to CDMA as default   
    mov  s_pktExtDescr.threadId, THREADID_CDMA0
     
    qbbs l_paEfRec4Proc_loop,  s_txPktCxt.flags.t_flag_ip_frag  
        set s_efState4.flags.t_ef_noFrag
        set s_efState4.flags.t_ef_lastFrag
    
l_paEfRec4Proc_loop: 

.using  ef4ScopeLoc1

    // restore original packet information
    clr s_txPktCxt.flags.t_flag_ip_frag
    mov s_pktExtDescr2.sopLength, s_efState4.origSopLen  
    
    // Calculate the IP length
    qbbs    l_paEfRec4Proc_ip_length_1, s_efState4.flags.t_ef_lastFrag
            // Non-last fragment
            mov s_efState4.l3Len,  s_txPktCxt.fragSize
            jmp l_paEfRec4Proc_ip_length_end  

l_paEfRec4Proc_ip_length_1:
            mov s_efState4.l3Len,  s_txPktCxt.lastFragSize
            set s_efState4.flags.t_ef_msgLen_update
            
            // pass through
    
l_paEfRec4Proc_ip_length_end:
    // Calcualte IP payload size here
    sub s_ef4_loc1.payloadSize, s_efState4.l3Len, s_txPktCxt.ipHdrLen  
    
    // Layer 2 header processing
    //  Advance to the L2 header.
    qbeq  l_paEfRec4Proc_l2_0,    s_txPktCxt.l2Offset, 0
        mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov   s_cdeCmdWd.byteCount,   s_txPktCxt.l2Offset
        xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
    
l_paEfRec4Proc_l2_0:
    
    //qbbc l_paEfRec4Proc_l2_1, s_efRec4.ctrlFlags.t_ef_rec4_strip_l2_hdr 
    qbeq l_paEfRec4Proc_update_l2,  s_efRec4.l2HdrSize, 0
    qbeq l_paEfRec4Proc_l2_1, s_txPktCxt.l3Offset, s_txPktCxt.l2Offset 
        // Flush out L2 header
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        sub  s_cdeCmdWd.byteCount,  s_txPktCxt.l3Offset,    s_txPktCxt.l2Offset
        xout XID_CDECTRL,           s_cdeCmdWd,             4    

        // Adjust the offsets
        //mov s_txPktCxt.l3Offset2,   s_txPktCxt.l3Offset 
        //sub s_txPktCxt.l4Offset,    s_txPktCxt.l4Offset,    s_cdeCmdWd.byteCount
          mov s_txPktCxt.l3Offset,    s_txPktCxt.l2Offset  
          sub s_txPktCxt.espAhOffset, s_txPktCxt.espAhOffset, s_cdeCmdWd.byteCount
        
        // Note: ESP/AH header will be set only when it is required
        //       l3offset2, l4offset are not used
        
        // Adjust the sopLength
        sub s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_cdeCmdWd.byteCount 
        
        // pass through
        
l_paEfRec4Proc_l2_1:
//
// Insert and update L2 header from EF4 record
//        

l_paEfRec4Proc_ins_l2:
    qbbc l_paEfRec4Proc_ins_l2_2,   s_efState4.flags.t_ef_msgLen_update 
    qbbc l_paEfRec4Proc_ins_l2_1,   s_efRec4.ctrlFlags.t_ef_rec4_valid_802_3_len
        // Calculate and update 802.3 length
        sub  r0.w0,  s_efRec4.l2HdrSize, s_efRec4.l2LenOffset
        sub  r0.w0,  r0.w0,  2
        add  r0.w0,  r0.w0,  s_efState4.l3Len
        add  r0.w2,  r28.w0, s_efRec4.l2LenOffset
        add  r0.w2,  r0.w2,  PA_EF_REC4_L2_HDR_OFFSET
        sbco r0.w0,  PAMEM_CONST_EF_RECORD4, r0.w2, 2
        // pass through
              
l_paEfRec4Proc_ins_l2_1:
    qbbc l_paEfRec4Proc_ins_l2_2,   s_efRec4.ctrlFlags.t_ef_rec4_valid_pppoe
        // Calculate and update PPPoE paylaod length
        sub  r0.w0,  s_efRec4.l2HdrSize, s_efRec4.pppoeOffset
        sub  r0.w0,  r0.w0,  OFFSET(struct_pppoe.prot)
        add  r0.w0,  r0.w0,  s_efState4.l3Len
        add  r0.w2,  r28.w0, s_efRec4.pppoeOffset
        add  r0.w2,  r0.w2,  OFFSET(struct_pppoe.len) + PA_EF_REC4_L2_HDR_OFFSET
        sbco r0.w0,  PAMEM_CONST_EF_RECORD4, r0.w2, 2
        // pass through
                       
l_paEfRec4Proc_ins_l2_2:
    // Insert the L2 header
    mov  s_cdeCmdInD.operation,  CDE_CMD_INSERT_PACKET_BUFFER
    mov  s_cdeCmdInD.lenLsb,     s_efRec4.l2HdrSize
    mov  s_cdeCmdInD.lenMsbs,    0
    mov  s_cdeCmdInD.dataP,      PAMEM_CONST_EF_RECORD4_BASE + PA_EF_REC4_L2_HDR_OFFSET
    add  s_cdeCmdInD.dataP,      s_cdeCmdInD.dataP,       r28.w0,    
    xout XID_CDECTRL,            s_cdeCmdWd,              SIZE(s_cdeCmdWd)
                       
    // Adjust all offset length
    add s_txPktCxt.l3Offset,     s_txPktCxt.l3Offset,    s_efRec4.l2HdrSize 
    
    // TBD: The following offsets are no longer required 
    //add s_txPktCxt.l3Offset2,    s_txPktCxt.l3Offset2,   s_efRec4.l2HdrSize  
    //add s_txPktCxt.l4Offset,     s_txPktCxt.l4Offset,    s_efRec4.l2HdrSize  
    add s_txPktCxt.espAhOffset,  s_txPktCxt.espAhOffset, s_efRec4.l2HdrSize
    add s_txPktCxt.endOffset,    s_txPktCxt.endOffset,   s_efRec4.l2HdrSize  
    
    // Adjust the sopLength
    add s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_efRec4.l2HdrSize 
    
    // Clear message update flag
    clr s_efState4.flags.t_ef_msgLen_update
    
    jmp l_paEf4PostL2Proc
    
l_paEfRec4Proc_update_l2: 
    //
    // Update L2 header
    //  
.using  macVlanScope
    
    add   s_ef4_loc1.l2PktOffset, s_txPktCxt.l2Offset, SIZE(s_macAddr)
    mov   s_ef4_loc1.insertCnt, 0
        
    // advance over MAC addresses
    mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
    mov   s_cdeCmdWd.byteCount,   SIZE(s_macAddr)
    xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
    
    // Read in enough data to cover the llc/snap header if present
    xin  XID_CDEDATA,   s_tagOrLen2,  SIZE(s_tagOrLen2)
    
l_paEfRec4Proc_update_l2_vlan2:    
    // Outer VLAN processing
    mov  s_cdeCmdIn2.data.w2, ETH_TAG_SP_OUTER_VLAN 
    qbne l_paEfRec4Proc_update_l2_vlan2_2,  s_vtag2.etherType, s_cdeCmdIn2.data.w2
        // upadte the VLAN Id inside the CDE input window in first loop only if required
        qbbc l_paEfRec4Proc_update_l2_vlan2_1,  s_efState4.flags.t_ef_firstLoop
        qbbc l_paEfRec4Proc_update_l2_vlan2_1_pri,  s_efRec4.ctrlFlags.t_ef_rec4_valid_vlan2
            //update VLAN ID only
            add     r0.w2,  r28.w0, PA_EF_REC4_VLAN2_OFFSET
            lbco    r0.w0, PAMEM_CONST_EF_RECORD4, r0.w2, 2
            
            mov s_vtag2.tag.b0, r0.b0
            and s_vtag2.tag.b1, s_vtag2.tag.b1, 0xF0
            and r0.b1, r0.b1, 0x0F
            or  s_vtag2.tag.b1, s_vtag2.tag.b1, r0.b1
            xout  XID_CDEDATA,   s_vtag2.tag,  SIZE(s_vtag2.tag)
            
l_paEfRec4Proc_update_l2_vlan2_1_pri:  
            // record the VLAN priority
            lsr   s_txPktCxt3.vlanPri,  s_vtag2.tag.b1, 5 
            
            // pass through
l_paEfRec4Proc_update_l2_vlan2_1:
            // adavnce over VLAN2 tag
            // mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
            mov   s_cdeCmdWd.byteCount,   SIZE(s_vtag2)
            xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)

            add   s_ef4_loc1.l2PktOffset, s_ef4_loc1.l2PktOffset, SIZE(s_vtag2)
            // Read in enough data to cover the llc/snap header if present
            xin  XID_CDEDATA,   s_tagOrLen2,  SIZE(s_tagOrLen2)
            jmp  l_paEfRec4Proc_update_l2_vlan1
    
l_paEfRec4Proc_update_l2_vlan2_2:
    qbbc l_paEfRec4Proc_update_l2_vlan1, s_efRec4.ctrlFlags.t_ef_rec4_valid_vlan2
        // Insert VLAN2 tag
        mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
        mov  s_cdeCmdIn2.len,           4 
        add  r0.w2,  r28.w0, PA_EF_REC4_VLAN2_OFFSET
        lbco s_cdeCmdIn2.data.w0, PAMEM_CONST_EF_RECORD4, r0.w2, 2
        xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdIn2)
        // record the VLAN priority
        lsr   s_txPktCxt3.vlanPri,  s_cdeCmdIn2.data.b1,  5 
        
        add  s_ef4_loc1.insertCnt,  s_ef4_loc1.insertCnt, 4
        // pass through
        
l_paEfRec4Proc_update_l2_vlan1:
    // Outer VLAN processing
    mov  s_cdeCmdIn2.data.w2, ETH_TAG_VLAN 
    qbne l_paEfRec4Proc_update_l2_vlan1_2,  s_vtag2.etherType, s_cdeCmdIn2.data.w2
        // upadte the VLAN Id in the CDE input window in first loop only if required
        qbbc l_paEfRec4Proc_update_l2_vlan1_1,  s_efState4.flags.t_ef_firstLoop
        qbbc l_paEfRec4Proc_update_l2_vlan1_1_pri,  s_efRec4.ctrlFlags.t_ef_rec4_valid_vlan1
            //update VLAN ID only
            add     r0.w2,  r28.w0, PA_EF_REC4_VLAN1_OFFSET
            lbco    r0.w0, PAMEM_CONST_EF_RECORD4, r0.w2, 2
            
            mov s_vtag2.tag.b0, r0.b0
            and s_vtag2.tag.b1, s_vtag2.tag.b1, 0xF0
            and r0.b1, r0.b1, 0x0F
            or  s_vtag2.tag.b1, s_vtag2.tag.b1, r0.b1
            xout  XID_CDEDATA,   s_vtag2.tag,  SIZE(s_vtag2.tag)
            
l_paEfRec4Proc_update_l2_vlan1_1_pri:
            
            // record the VLAN priority
            lsr   s_txPktCxt3.vlanPri,  s_vtag2.tag.b1, 5 
            
            // pass through
l_paEfRec4Proc_update_l2_vlan1_1:
            // adavnce over VLAN1 tag
            mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
            mov   s_cdeCmdWd.byteCount,   SIZE(s_vtag2)
            xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)

            add   s_ef4_loc1.l2PktOffset, s_ef4_loc1.l2PktOffset, SIZE(s_vtag2)
            // Read in enough data to cover the llc/snap header if present
            xin  XID_CDEDATA,   s_tagOrLen2,  SIZE(s_tagOrLen2)
            jmp  l_paEfRec4Proc_update_l2_len
    
l_paEfRec4Proc_update_l2_vlan1_2:
    qbbc l_paEfRec4Proc_update_l2_len, s_efRec4.ctrlFlags.t_ef_rec4_valid_vlan1
        // Insert VLAN2 tag
        mov  s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
        mov  s_cdeCmdIn2.len,           4 
        add  r0.w2,  r28.w0, PA_EF_REC4_VLAN1_OFFSET
        lbco s_cdeCmdIn2.data.w0, PAMEM_CONST_EF_RECORD4, r0.w2, 2
        xout XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdIn2)
        
        // record the VLAN priority
        lsr   s_txPktCxt3.vlanPri,  s_cdeCmdIn2.data.b1,  5 
        
        add  s_ef4_loc1.insertCnt,  s_ef4_loc1.insertCnt, 4 
        // pass through
        
l_paEfRec4Proc_update_l2_len:
    // Verify whether it is an 802.3 packet, update the 802.3 length accordingly      
    mov  r0,    1500
    qblt   l_paEfRec4Proc_update_l2_pppoe,  s_tagOrLen2.len,   r0
        // TBD: Error check?
        qbbc l_paEfRec4Proc_update_l2_len_1,  s_efState4.flags.t_ef_msgLen_update 
            // Calculate and update 802.3 length
            sub  s_tagOrLen2.len,  s_txPktCxt.l3Offset, s_ef4_loc1.l2PktOffset
            sub  s_tagOrLen2.len,  s_tagOrLen2.len,  2
            add  s_tagOrLen2.len,  s_tagOrLen2.len,  s_efState4.l3Len
            xout XID_CDEDATA,   s_tagOrLen2.len,  SIZE(s_tagOrLen2.len)
            
l_paEfRec4Proc_update_l2_len_1:
        // adavnce over 803.3 header
        mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
        mov   s_cdeCmdWd.byteCount,   8
        xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
                
        add   s_ef4_loc1.l2PktOffset, s_ef4_loc1.l2PktOffset, 8
        // Read in enough data to cover the llc/snap header if present
        xin  XID_CDEDATA,   s_tagOrLen2,  SIZE(s_tagOrLen2)
        
        // pass through
        
l_paEfRec4Proc_update_l2_pppoe: 
    // Verify whether it is a PPPoE data packet, update the PPPoE payload length accordingly 
    mov r0,  ETH_TYPE_PPPoE_SESSION     
    qbne   l_paEfRec4Proc_update_l2_end,  s_pppoe2.etherType,   r0
        // TBD: Error check?
        qbbc l_paEfRec4Proc_update_l2_end,  s_efState4.flags.t_ef_msgLen_update 
            // Calculate and update PPPoE length
            sub  s_pppoe2.len,  s_txPktCxt.l3Offset, s_ef4_loc1.l2PktOffset
            sub  s_pppoe2.len,  s_pppoe2.len,  OFFSET(s_pppoe2.prot)
            add  s_pppoe2.len,  s_pppoe2.len,  s_efState4.l3Len
            xout XID_CDEDATA,   s_pppoe2.len,  SIZE(s_pppoe2.len)
            
.leave  macVlanScope
      
l_paEfRec4Proc_update_l2_end:
     // adavnce over L2 header
     mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
     sub   s_cdeCmdWd.byteCount,   s_txPktCxt.l3Offset, s_ef4_loc1.l2PktOffset
     xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
     
     //adjust offsets and sopLength
    // Adjust all offset length
    add s_txPktCxt.l3Offset,     s_txPktCxt.l3Offset,    s_ef4_loc1.insertCnt 
    
    // TBD: The following offsets are no longer required 
    //add s_txPktCxt.l3Offset2,    s_txPktCxt.l3Offset2,   s_efRec4.l2HdrSize  
    //add s_txPktCxt.l4Offset,     s_txPktCxt.l4Offset,    s_efRec4.l2HdrSize  
    add s_txPktCxt.espAhOffset,  s_txPktCxt.espAhOffset, s_ef4_loc1.insertCnt
    add s_txPktCxt.endOffset,    s_txPktCxt.endOffset,   s_ef4_loc1.insertCnt  
    
    // Adjust the sopLength
    add s_pktExtDescr2.sopLength, s_pktExtDescr2.sopLength, s_ef4_loc1.insertCnt 
    
    // Clear message update flag
    clr s_efState4.flags.t_ef_msgLen_update
    
    //jmp l_paEf4PostL2Proc

l_paEf4PostL2Proc: 

f_paEf4IpFrag: 
     qbbc  l_paEf4IpFrag_1, s_efState4.flags.t_ef_noFrag
         // No IP fragmentation is required
         // Advance over outer IP
         mov   s_cdeCmdWd.operation,   CDE_CMD_WINDOW_ADVANCE
         sub   s_cdeCmdWd.byteCount,   s_efState4.l3Len, s_pktExtDescr2.mopLength  
         xout  XID_CDECTRL,    s_cdeCmdWd,   SIZE(s_cdeCmdWd)
         
         // Flush out the remaining packet 
         mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
         xout XID_CDECTRL,             s_cdeCmdPkt,             4
         
         qblt l_paEf4IpFrag_0, s_ef4_loc1.payloadSize, s_pktExtDescr2.sopL4Length 
            set s_efState4.flags.t_ef_withinSop 

l_paEf4IpFrag_0:
         jmp   f_paEf4_ahPatch
         
l_paEf4IpFrag_1: 
    // Update the statistics
    mov s_stats.value,  PA_STATS_UPDATE_REQ | PA_STATS_TX_IP_FRAG

    // Load the packet Ip header
    xin     XID_CDEDATA, s_Ip, SIZE(s_Ip)
    
    // Find whetehr it is IPv4 or IPv6
    and     r0.b0, s_Ip.VerLen, 0xF0
    qbeq    l_paEf4IpFrag_ProcessIp_1, r0.b0, 0x40
    
    //Must be IPv6: otherwise ignore
    qbeq    fci_paEf4Ipv6Frag, r0.b0, 0x60
    
    // exception handle: bad tx packet (no return)
    mov  s_txPktCxt2.eId,   EF_EROUTE_PARSE_FAIL
    jmp  f_efForwardExp  
    
//
// IPv4 fragmentation loop
//     
    
l_paEf4IpFrag_ProcessIp_1:
    // Save the base offset
    qbbc     l_paEf4IpFrag_ProcessIp_2,  s_efState4.flags.t_ef_firstLoop         
        mov s_efState4.baseOffset, s_Ip.FragOff
        mov s_efState4.ipLen, s_Ip.TotalLen 
        //add s_ipPktId.id, s_ipPktId.id, 1 
        //mov s_ef4_loc1.payloadSize, s_txPktCxt.fragSize  
       
l_paEf4IpFrag_ProcessIp_2:
    // SOP is at least L2+L3
    add     s_pktExtDescr2.sopLength, s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset   

    // Patch the frag offset
    lsr     r1, s_efState4.loopOffset, 3
    add     s_Ip.fragOff, s_efState4.baseOffset, r1
    //mov     s_Ip.id,    s_ipPktId.id.w0
	lbco    s_Ip.id, cMailbox, 2, 2
    
    qbbs    l_paEf4IpFrag_CommonFrag, s_efState4.flags.t_ef_lastFrag  
        
l_paEf4IpFrag_NotLast:        
    // Set more fragments
    set     s_Ip.FragOff.t_ipv4_frag_m

l_paEf4IpFrag_CommonFrag:
    add     s_Ip.TotalLen, s_ef4_loc1.payloadSize, s_txPktCxt.ipHdrLen  
    mov     s_Ip.Checksum, 0

    // Write out the new IP header
    xout    XID_CDEDATA, s_Ip, SIZE(s_Ip)-8         // We never update the last 8 bytes (or options)
    
    // Start the IP Header checksum. 
    qbbc    l_paEf4IpFrag_chksum_end, s_efState4.flags.t_ef_firstLoop
        zero    &s_cdeCmdChk,         SIZE(s_cdeCmdChk)
        mov     s_cdeCmdChk.operation,CDE_CMD_CHECKSUM1_COMPUTE
        mov     s_cdeCmdChk.byteLen,  s_txPktCxt.ipHdrLen
        mov     s_cdeCmdChk.offset,   OFFSET(s_Ip.Checksum)          // Starting sum is 0, offset is 10
        xout    XID_CDECTRL, s_cdeCmdChk,  SIZE(s_cdeCmdChk)
            
l_paEf4IpFrag_chksum_end:            
    // Slide the window past the IP header
    mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
    mov  s_cdeCmdWd.byteCount,  s_txPktCxt.ipHdrLen
    xout XID_CDECTRL,           s_cdeCmdWd,              4
    
    // Move the amount we need to copy  and skip into "Remain"
    sub s_ef4_loc1.dataRemain,   s_Ip.TotalLen,    s_txPktCxt.ipHdrLen 
    mov s_ef4_loc1.offsetRemain, s_efState4.loopOffset
        
    // See if we still have data remaining in SOP
    qbgt    l_paEf4IpFrag_HaveSOP,   s_ef4_loc1.offsetRemain,   s_pktExtDescr2.sopL4Length
        // Flush past all SOP
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length
        xout XID_CDECTRL,           s_cdeCmdWd,              4    
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_ef4_loc1.offsetRemain,    s_ef4_loc1.offsetRemain, s_pktExtDescr2.sopL4Length
        jmp  l_paEf4IpFrag_PastSOP
            
l_paEf4IpFrag_HaveSOP: 
        qbeq l_paEf4IpFrag_NoFlush, s_efState4.loopOffset, 0       
            // Now flush any data contained in s_paEf4IpFragCxt.loopOffset
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
            mov  s_cdeCmdWd.byteCount,  s_ef4_loc1.offsetRemain
            xout XID_CDECTRL, s_cdeCmdWd, 4 
                
l_paEf4IpFrag_NoFlush:        
            // Slide the window past the valid data
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            sub  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length, s_ef4_loc1.offsetRemain
            // Trim the SOP to the data size
            qbgt    l_paEf4IpFrag_NoSOPTrim, s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain 
                mov s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain
                set s_efState4.flags.t_ef_withinSop
                
l_paEf4IpFrag_NoSOPTrim:                 
                xout XID_CDECTRL,  s_cdeCmdWd,   4
                mov  s_ef4_loc1.offsetRemain,    0
                sub  s_ef4_loc1.dataRemain,     s_ef4_loc1.dataRemain, s_cdeCmdWd.byteCount
                add  s_pktExtDescr2.sopLength,  s_pktExtDescr2.sopLength,   s_cdeCmdWd.byteCount 
                
l_paEf4IpFrag_PastSOP:
            // Here we're past SOP or we don't have any data left
            // We may need to copy some MOP
            mov     s_pktExtDescr2.mopLength,   s_ef4_loc1.dataRemain
            qbeq    l_paEf4IpFrag_FragReady,    s_ef4_loc1.dataRemain, 0
            
            // see if we have any MOP
            qbgt    l_paEf4IpFrag_HaveMOP,   s_ef4_loc1.offsetRemain, s_pktExtDescr2.mopLengthOrig
                sub s_ef4_loc1.offsetRemain, s_ef4_loc1.offsetRemain, s_pktExtDescr2.mopLengthOrig
                mov s_pktExtDescr2.mopLength, 0
                jmp l_paEf4IpFrag_PastMOP
                
l_paEf4IpFrag_HaveMOP:
            add     s_pktExtDescr2.mopPtr,  s_pktExtDescr2.mopPtrOrig, s_ef4_loc1.offsetRemain 
            // Trim the MOP to the MOP remaining size
            add     r0,  s_ef4_loc1.offsetRemain,  s_ef4_loc1.dataRemain
            qbgt    l_paEf4IpFrag_NoMOPTrim, r0, s_pktExtDescr2.mopLengthOrig
                sub  s_pktExtDescr2.mopLength,  s_pktExtDescr2.mopLengthOrig,   s_ef4_loc1.offsetRemain
                
l_paEf4IpFrag_NoMOPTrim:
                sub  s_ef4_loc1.dataRemain,    s_ef4_loc1.dataRemain, s_pktExtDescr2.mopLength
                mov  s_ef4_loc1.offsetRemain,  0
                           
l_paEf4IpFrag_PastMOP:
            qbeq    l_paEf4IpFrag_NoEOPFlush,    s_ef4_loc1.offsetRemain,   0 
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
                mov  s_cdeCmdWd.byteCount,  s_ef4_loc1.offsetRemain
                xout XID_CDECTRL, s_cdeCmdWd, 4 
                mov  s_ef4_loc1.offsetRemain, 0
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
                // pass through
             
l_paEf4IpFrag_NoEOPFlush:             
            // If there is any EOP data to move, move it now
            qbeq    l_paEf4IpFrag_FragReady, s_ef4_loc1.dataRemain, 0
                // Slide the window past the valid data
                mov s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain
                xout XID_CDECTRL,  s_cdeCmdWd,    4
            
l_paEf4IpFrag_FragReady:
    // Flush out the remaining packet 
    mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,             s_cdeCmdPkt,             4

    // Setup the offset for the next packet (jump out if final frag)
    add  s_efState4.loopOffset, s_efState4.loopOffset, s_ef4_loc1.payloadSize
    
    jmp  f_paEf4_ahPatch
    
fci_paEf4Ipv6Frag:

l_paEf4Ipv6Frag_ProcessIp:
    // Ip header has already been loaded 
    //xin     XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
   
l_paEf4Ipv6Frag_ProcessIp_1:
    // Save the base offset
    qbbc     l_paEf4Ipv6Frag_ProcessIp_2,  s_efState4.flags.t_ef_firstLoop         
        mov s_efState4.baseOffset, 0
        add s_efState4.ipLen, s_Ipv6a.payloadLen, IPV6_HEADER_LEN_BYTES
        //sub s_ef4_loc1.totalFragSize, s_efState4.ipLen, s_txPktCxt.ipHdrLen
        //sub s_ef4_loc1.payloadSize,  s_txPktCxt.fragSize, s_txPktCxt.ipHdrLen
        //sub s_ef4_loc1.payloadSize,  s_ef4_loc1.payloadSize,  8 
        //add s_ipPktId.id, s_ipPktId.id, 1 
        add s_txPktCxt.espAhOffset, s_txPktCxt.espAhOffset, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    
l_paEf4Ipv6Frag_ProcessIp_2:    
    // Update payloadLen
    sub     s_ef4_loc1.payloadSize,  s_ef4_loc1.payloadSize,  IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    add     r0.w0,  s_ef4_loc1.payloadSize, s_txPktCxt.ipHdrLen
    add     r0.w0,  r0.w0, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
    sub     s_Ipv6a.payloadLen, r0.w0, IPV6_HEADER_LEN_BYTES
    xout    XID_CDEDATA, s_Ipv6a.payloadLen, SIZE(s_Ipv6a.payloadLen) 
    
    // assume this is the last fragment
    //set     s_efState4.flags.t_ef_lastFrag

    // If ipHdrLen==IPV6_HEADER_LEN_BYTES, then we still have the original IP header,
    // else we need to patch
    qbeq    l_paEf4Ipv6Frag_only_fixed_header, s_txPktCxt.ipHdrLen, IPV6_HEADER_LEN_BYTES
        // Found non-fragmentable extension header
        // Advance to the extended Header
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  s_txPktCxt.nextHdrOffset
        xout XID_CDECTRL,  s_cdeCmdWd,    4
        
        mov  s_Ipv6Opt.proto,    IP_PROTO_NEXT_IPV6_FRAG 
        xout XID_CDEDATA, s_Ipv6Opt.proto,         SIZE(s_Ipv6Opt.proto)
        
        // Step past the last header we checked
        sub     s_cdeCmdWd.byteCount,    s_txPktCxt.ipHdrLen, s_txPktCxt.nextHdrOffset
        xout    XID_CDECTRL, s_cdeCmdWd, 4 
        
        jmp     l_paEf4Ipv6Frag_Loop

l_paEf4Ipv6Frag_only_fixed_header:
        // Set next header in IPv6 fixed header to frag
        mov     s_Ipv6a.next, IP_PROTO_NEXT_IPV6_FRAG
        xout    XID_CDEDATA, s_Ipv6a, SIZE(s_Ipv6a)
        
        // Step past the last header we checked
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        mov  s_cdeCmdWd.byteCount,  IPV6_HEADER_LEN_BYTES
        xout    XID_CDECTRL, s_cdeCmdWd, 4 

l_paEf4Ipv6Frag_Loop:
    // SOP is at least L2+L3
    add     s_pktExtDescr2.sopLength,   s_txPktCxt.ipHdrLen, s_txPktCxt.l3Offset 
    add     s_pktExtDescr2.sopLength,   s_pktExtDescr2.sopLength, IPV6_OPT_FRAG_EXTENSION_LEN_BYTES
      
    // Update and insert the Fragmentation header
    mov     s_cdeCmdIn2.operation,     CDE_CMD_INSERT_PACKET 
    mov     s_cdeCmdIn2.len,           4 
    lsl     s_cdeCmdIn2.data.w2,       s_txPktCxt.nextHdr,  8      
    mov     s_cdeCmdIn2.data.w0,       s_efState4.loopOffset
    
    // Check whether it is the last fragment
    qbbs    l_paEf4Ipv6Frag_CommonFrag, s_efState4.flags.t_ef_lastFrag
    
        // There are more fragments
        set     s_cdeCmdIn2.data.w0.t_ipv6_frag_m
        //clr     s_efState4.flags.t_ef_lastFrag
   
l_paEf4Ipv6Frag_CommonFrag:
    // Complete the fragmentation header
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      
    //mov     s_cdeCmdIn2.data, s_ipPktId.id
	lbco  s_cdeCmdIn2.data, cMailbox, 0, 4
    xout    XID_CDECTRL, s_cdeCmdIn2,   SIZE(s_cdeCmdIn2)                      

    // Move the amount we need to copy  and skip into "Remain"
    mov s_ef4_loc1.dataRemain, s_ef4_loc1.payloadSize
    mov s_ef4_loc1.offsetRemain, s_efState4.loopOffset
    
    // See if we still have data remaining in SOP
    qbgt    l_paEf4Ipv6Frag_HaveSOP,   s_ef4_loc1.offsetRemain,   s_pktExtDescr2.sopL4Length
        // Flush past all SOP
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
        mov  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length
        xout XID_CDECTRL,           s_cdeCmdWd,              4    
        mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
        sub  s_ef4_loc1.offsetRemain,    s_ef4_loc1.offsetRemain, s_pktExtDescr2.sopL4Length
        jmp  l_paEf4Ipv6Frag_PastSOP
            
l_paEf4Ipv6Frag_HaveSOP: 
        //qbeq l_paEf4Ipv6Frag_NoFlush, s_efState4.loopOffset, 0       // redundancy check
            // Now flush any data contained in s_efState4.loopOffset
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
            mov  s_cdeCmdWd.byteCount,  s_ef4_loc1.offsetRemain
            xout XID_CDECTRL, s_cdeCmdWd, 4 
            
l_paEf4Ipv6Frag_NoFlush:        
            // Slide the window past the valid data
            mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
            sub  s_cdeCmdWd.byteCount,  s_pktExtDescr2.sopL4Length, s_ef4_loc1.offsetRemain
            // Trim the SOP to the data size
            qbgt    l_paEf4Ipv6Frag_NoSOPTrim, s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain 
                mov s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain
                set s_efState4.flags.t_ef_withinSop
                
l_paEf4Ipv6Frag_NoSOPTrim:                 
                xout XID_CDECTRL,  s_cdeCmdWd,    4
                mov  s_ef4_loc1.offsetRemain,     0
                sub  s_ef4_loc1.dataRemain,    s_ef4_loc1.dataRemain, s_cdeCmdWd.byteCount
                add  s_pktExtDescr2.sopLength,  s_pktExtDescr2.sopLength,   s_cdeCmdWd.byteCount 
                
l_paEf4Ipv6Frag_PastSOP:
            // Here we're past SOP or we don't have any data left
            // We may need to copy some MOP
            mov s_pktExtDescr2.mopLength, s_ef4_loc1.dataRemain
            qbeq    l_paEf4Ipv6Frag_FragReady, s_ef4_loc1.dataRemain, 0
            
            // see if we have any MOP
            qbgt    l_paEf4Ipv6Frag_HaveMOP,   s_ef4_loc1.offsetRemain, s_pktExtDescr2.mopLengthOrig
                sub s_ef4_loc1.offsetRemain, s_ef4_loc1.offsetRemain, s_pktExtDescr2.mopLengthOrig
                mov s_pktExtDescr2.mopLength, 0
                jmp l_paEf4Ipv6Frag_PastMOP
                
l_paEf4Ipv6Frag_HaveMOP:
            add     s_pktExtDescr2.mopPtr,  s_pktExtDescr2.mopPtrOrig, s_ef4_loc1.offsetRemain 
            // Trim the MOP to the MOP remaining size
            add     r0,  s_ef4_loc1.offsetRemain,  s_ef4_loc1.dataRemain
            qbgt    l_paEf4Ipv6Frag_NoMOPTrim, r0, s_pktExtDescr2.mopLengthOrig
                sub  s_pktExtDescr2.mopLength,  s_pktExtDescr2.mopLengthOrig,   s_ef4_loc1.offsetRemain
                
l_paEf4Ipv6Frag_NoMOPTrim:
                sub  s_ef4_loc1.dataRemain,    s_ef4_loc1.dataRemain, s_pktExtDescr2.mopLength
                mov  s_ef4_loc1.offsetRemain,  0
                           
l_paEf4Ipv6Frag_PastMOP:
            qbeq    l_paEf4Ipv6Frag_NoEOPFlush,    s_ef4_loc1.offsetRemain,   0 
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_FLUSH
                mov  s_cdeCmdWd.byteCount,  s_ef4_loc1.offsetRemain
                xout XID_CDECTRL, s_cdeCmdWd, 4 
                mov  s_ef4_loc1.offsetRemain, 0
                mov  s_cdeCmdWd.operation,  CDE_CMD_WINDOW_ADVANCE
                // pass through
             
l_paEf4Ipv6Frag_NoEOPFlush:             
            // If there is any EOP data to move, move it now
            qbeq    l_paEf4Ipv6Frag_FragReady, s_ef4_loc1.dataRemain, 0
                // Slide the window past the valid data
                mov s_cdeCmdWd.byteCount, s_ef4_loc1.dataRemain
                xout XID_CDECTRL,  s_cdeCmdWd,    4
            
l_paEf4Ipv6Frag_FragReady:
    // Setup the offset for the next packet 
    add     s_efState4.loopOffset, s_efState4.loopOffset, s_ef4_loc1.payloadSize
    
    // Calc the size for this next loop
    //add     r0, s_efState4.loopOffset, s_ef4_loc1.payloadSize
    //qbge    l_paEf4Ipv6Frag_NoAdjust, r0, s_ef4_loc1.totalFragSize
    //    sub s_ef4_loc1.payloadsize, s_ef4_loc1.totalFragSize, s_efState4.loopOffset 
    
l_paEf4Ipv6Frag_NoAdjust:
    
    // Flush out the remaining packet 
    mov  s_cdeCmdPkt.operation,   CDE_CMD_FLUSH_TO_END
    xout XID_CDECTRL,             s_cdeCmdPkt,             4
    
f_paEf4_ahPatch:
    // AH header patch with ICV if required
    qbbc   f_paEf4_txPadding, s_efState4.flags.t_ef_firstLoop
    qbbc   f_paEf4_txPadding, s_txPktCxt.flags.t_flag_ah_patch
        // AH header patch
        and    r0.b0,   s_txPktCxt.flags.b0, PA_EF_ICV_SIZE_MASK
        lsl    r0.b0,   r0.b0, 2
        add    s_cdeCmdPatchD.len,        r0.b0,    8
        add    s_cdeCmdPatchD.offset,     s_txPktCxt.espAhOffset, 12
        mov    s_cdeCmdPatchD.operation,  CDE_CMD_PATCH_PACKET_BUFFER
        mov    s_cdeCmdPatchD.dataP.w2,   PAMEM_CONST_TEMP_BUF_A_BASE >> 16
        mov    s_cdeCmdPatchD.dataP.w0,   PAMEM_CONST_TEMP_BUF_A_BASE & 0xffff
        xout   XID_CDECTRL,               s_cdeCmdPatchD,          SIZE(s_cdeCmdPatchD)
        
f_paEf4_txPadding:
    // Verify whether padding is required
    add  r0.w0,  s_efState4.l3Len, s_txPktCxt.l3Offset
    qbge f_paEf4_ForwardPkt, s_efRec4.minPktSize, r0.w0 
        // Insert packet data
        mov     s_cdeCmdIn.operation, CDE_CMD_INSERT_PACKET
        sub     s_cdeCmdIn.len,  s_efRec4.minPktSize, r0.b0
        xout    XID_CDECTRL,    s_cdeCmdIn, 4              // Send the command 
        
#ifdef TO_BE_DELETE
        // During padding requirement, we should not adjust the length incorrectly.
        // This would create invalid CPPI descriptors
        mov     s_pktExtDescr2.sopLength,  s_efRec4.minPktSize
#endif        
    
f_paEf4_ForwardPkt:

    qbbc l_paEf4_ForwardPkt_pri_udpate_end, s_efState4.flags.t_ef_firstLoop
l_paEf4_ForwardPkt_pri_udpate:
        // adjust destination queue if QoS queue 
        qbbc l_paEf4_ForwardPkt_pri_udpate_dscp, s_efRec4.ctrlFlags.t_ef_rec4_pri_vlan
l_paEf4_ForwardPkt_pri_udpate_vlan: 
            add  s_efRec4.queueId, s_efRec4.queueId, s_txPktCxt3.vlanPri
l_paEf4_ForwardPkt_pri_udpate_dscp:
        qbbc l_paEf4_ForwardPkt_pri_udpate_end, s_efRec4.ctrlFlags.t_ef_rec4_pri_dscp
            add  s_efRec4.queueId, s_efRec4.queueId, s_txPktCxt3.dscp

l_paEf4_ForwardPkt_pri_udpate_end:

    // reset control flags
    clr s_efState4.flags.t_ef_firstLoop  
    
    zero &s_cdeCmdPkt,  SIZE(s_cdeCmdPkt)
    mov  s_cdeCmdPkt.destQueue,    s_efRec4.queueId
    mov  s_cdeCmdPkt.flowId,       s_efRec4.flowId
    
    //wbs   s_flags.info.tStatus_CDEOutPacket
    // update swInfo
    add     r0.w0,  r28.w0, PA_EF_REC4_SWINFO0_OFFSET
    lbco    r6, PAMEM_CONST_EF_RECORD4, r0.w0, 8
    
l_paEf4_ForwardPkt_srio:    
    qbne    l_paEf4_ForwardPkt_eth_host, s_efRec4.destType,  PA_FORWARD_TYPE_SRIO 
        //update psInfo
        sbco r6, cCdeOutPkt, SIZE(s_pktDescr),          8  
        mov  s_cdeCmdPkt.psInfoSize,   8
        
        //update pktType
        sbco s_efRec4.pktType_psFlags, cCdeOutPkt, OFFSET(s_pktDescr.pktType_pvtFlags), SIZE(s_pktDescr.pktType_pvtFlags)  
        
        jmp  l_paEf4_ForwardPkt_2
    
l_paEf4_ForwardPkt_eth_host:    
        //update psFlags
        lbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        and  r0.b0, r0.b0, NOT_PA_PKT_PS_FLAGS_MASK
        and  r0.b1, s_efRec4.pktType_psFlags, PAFRM_ETH_PS_FLAGS_CTRL_MASK
        or   r0.b0, r0.b0, r0.b1
        sbco r0.b0, cCdeOutPkt, OFFSET(s_pktDescr.psFlags_errorFlags), 1
        
        // Set EMAC output port
        and  r0.b1, s_efRec4.pktType_psFlags, PAFRM_ETH_PS_FLAGS_PORT_MASK
        sbco r0.b1, cCdeHeldPkt, OFFSET(s_pktDescrCpsw.outPort), 1
        
    qbne    l_paEf4_ForwardPkt_host, s_efRec4.destType,  PA_FORWARD_TYPE_ETH 
        // Clear TimeSync word
        mov  r0, 0
        sbco r0, cCdeOutPkt, OFFSET(s_pktDescrCpsw.tsWord), 4
        
        // Update Thread ID
        mov  s_pktExtDescr.threadId, THREADID_ETHERNET1
    
l_paEf4_ForwardPkt_host: 
        //update swInfo
        sbco r6, cCdeOutPkt, OFFSET(s_pktDescr.swinfo0), 8  

        
l_paEf4_ForwardPkt_2:    
    qbbs l_paEf4IpFrag_LastPacket, s_efState4.flags.t_ef_lastFrag
        mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_COPY
        mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_RETAIN_CHKSUM | CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
        //mov  r4.w0, CDE_CMD_PACKET_COPY | ((CDE_FLG_RETAIN_CHKSUM | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
        xout XID_CDECTRL,   s_cdeCmdPkt,    SIZE(s_cdeCmdPkt)
        xout XID_PINFO_DST, s_pktExtDescr2, SIZE(s_pktExtDescr2)   // Send the extended info

    // pass through

l_paEf4_ForwardPkt_WaitForCopy:
    // Wait for a new packet
    wbs s_flags.info.tStatus_CDENewPacket
        
    // Read packet descriptor
    // 
    // xin  XID_CDEDATA,  s_pktDescr,  OFFSET(s_pktDescr.pktDataSize) 
   
    // Advance to the control section
    mov  s_cdeCmdWd.operation,   CDE_CMD_ADVANCE_TO_CONTROL
    xout XID_CDECTRL,            s_cdeCmdWd,            4
    
    // Insert 32 bytes of PS info
    // This is legal even though the window has advanced to control
    mov s_cdeInsert.operation,  CDE_CMD_INSERT_PSDATA
    mov s_cdeInsert.byteCount,  32
    xout XID_CDECTRL,           s_cdeCmdWd,             4
    
    // Re-load the packet Info
    xin  XID_CDEDATA,  s_txPktCxt,   SIZE(s_txPktCxt)

    // Flush out the control info
    mov  s_cdeCmdWd.operation,   CDE_CMD_FLUSH_TO_PACKET
    xout XID_CDECTRL,            s_cdeCmdWd,            4
    
    // Check for the last fragment
    add     r0, s_efState4.loopOffset, s_ef4_loc1.payloadSize
    sub     r1, s_efState4.ipLen, s_txPktCxt.ipHdrLen
    qblt    l_paEfRec4Proc_loop, r1, r0
   
        // The next one will be the last frag - adjust the final size
        set     s_efState4.flags.t_ef_lastFrag
   
    jmp  l_paEfRec4Proc_loop

l_paEf4IpFrag_LastPacket:
    mov  s_pktExtDescr2.flags, s_efState4.extHdrFlags 
    mov  s_cdeCmdPkt.optionsFlag,  (CDE_FLG_SET_DESTQUEUE | CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO)
    mov  s_cdeCmdPkt.operation,    CDE_CMD_PACKET_ADVANCE
    //mov  r4.w0, CDE_CMD_PACKET_ADVANCE | ((CDE_FLG_SET_FLOWID | CDE_FLG_SET_PSINFO) << 8) 
    xout XID_CDECTRL,   s_cdeCmdPkt,    SIZE(s_cdeCmdPkt)
    xout XID_PINFO_DST, s_pktExtDescr2, SIZE(s_pktExtDescr2)   // Send the extended info
    
    jmp  f_mainLoop
    
    .leave ef4ScopeLoc1
    

    .leave cdeScope
    .leave pktScope 
    .leave ipScope
    
#endif   

// *******************************************************************************
// * FUNCTION PURPOSE: Record Configuration
// *******************************************************************************
// * DESCRIPTION: Configure the egress flow record  
// *
// *   Register Usage:  
// * 
// *   R0:      |
// *   R1:      | Temp and local variables
// *   R2:      | 
// *   R3:      | 
// *   R4:    |  Pa configuration command
// *   R5:    |                   -
// *   R6:        |        
// *   R7:        |                              
// *   R8:        |                             
// *   R9:        |  
// *   R10:       |  
// *   R11:       |  
// *   R12:       |  Record Buffer
// *   R13:       |   
// *   R14:       |                                            
// *   R15:       | 
// *   R16:       |                                            
// *   R17:       |                         
// *   R18:       | 
// *   R19:       |     
// *   R20:       |  
// *   R21:       |
// *   R22:     |     
// *   R23:     |  Packet context - pktScope   
// *   R24:     |
// *   R25:     |
// *   R26:     |
// *   R27:     | 
// *   R28:           |w0: record1 offset
// *   R29:  | 32-bit packet Id (system)
// *   R30:  w0-function return address                             -
// *   R31:  System Flags (s_flags)                                 -
// *
// *******************************************************************************

f_paEfRecConfig:  
    qblt l_paEfRecConfig_err,  s_paCfgCmd.param2, PA_EF_MAX_REC_INDEX               

#ifdef PASS_PROC_EF_REC1
l_paEfRec1Config:
    qbne l_paEfRecConfig_err,  s_paCfgCmd.param1, PA_EF_REC_TYPE_LVL1
        lsl  r0.w0,     s_paCfgCmd.param2,   5
        lsl  r0.w2,     s_paCfgCmd.param2,   4  
    
        add  r28.w0,     r0.w0, r0.w2       
        lbco r6,   PAMEM_CONST_EF_CMD,        FA_CMD_DATA_BUF_OFFSET, PA_EF_REC1_SIZE
        sbco r6,   PAMEM_CONST_EF_RECORD1,    r28.w0,     PA_EF_REC1_SIZE
#endif

#ifdef PASS_PROC_EF_REC2
l_paEfRec2Config:
    qbne l_paEfRecConfig_err,  s_paCfgCmd.param1, PA_EF_REC_TYPE_LVL2
        lsl  r28.w0,    s_paCfgCmd.param2,   6
    
        lbco r6,   PAMEM_CONST_EF_CMD,        FA_CMD_DATA_BUF_OFFSET, PA_EF_REC2_SIZE
        sbco r6,   PAMEM_CONST_EF_RECORD2,    r28.w0,     PA_EF_REC2_SIZE
#endif

#ifdef PASS_PROC_EF_REC3
l_paEfRec3Config:
    qbne l_paEfRecConfig_err,  s_paCfgCmd.param1, PA_EF_REC_TYPE_LVL3
        lsl  r28.w0,    s_paCfgCmd.param2,   5
    
        lbco r6,   PAMEM_CONST_EF_CMD,        FA_CMD_DATA_BUF_OFFSET, PA_EF_REC3_SIZE
        sbco r6,   PAMEM_CONST_EF_RECORD3,    r28.w0,     PA_EF_REC3_SIZE
#endif

#ifdef PASS_PROC_EF_REC4
l_paEfRec4Config:
    qbne l_paEfRecConfig_err,  s_paCfgCmd.param1, PA_EF_REC_TYPE_LVL4
        lsl  r28.w0,    s_paCfgCmd.param2,   6
    
        lbco r6,   PAMEM_CONST_EF_CMD,        FA_CMD_DATA_BUF_OFFSET, PA_EF_REC4_SIZE
        sbco r6,   PAMEM_CONST_EF_RECORD4,    r28.w0,     PA_EF_REC4_SIZE
#endif

    mov s_paCfgCmd.response,  PA_CMD_RESP_OK
    jmp l_paEfRecConfig_end

l_paEfRecConfig_err:
    mov s_paCfgCmd.response,  PA_CMD_RESP_BAD_PARAMS
    // pass through

l_paEfRecConfig_end:
    sbco s_paCfgCmd.response,  PAMEM_CONST_EF_CMD, OFFSET(s_paCfgCmd.response), SIZE(s_paCfgCmd.response)
    //mvid *&s_paCfgCmd.code,  0
    zero &s_paCfgCmd,          4 
    sbco s_paCfgCmd.code,      PAMEM_CONST_EF_CMD, 0, 4        
    jmp  fci_mainLoop1

.leave efScope
.leave globalScopeM

#endif



