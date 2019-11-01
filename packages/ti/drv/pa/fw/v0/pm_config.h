//============================================================================
// pm_config.h
//
// This is the top level include file for both PA and SA.
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

#ifndef _NYSH_CONFIG_H
#define _NYSH_CONFIG_H  1

// Big Endian Environment
#define PA_BIGENDIAN    1

// PDSP IRAM 2K instructions = 8K bytes
// PDSP IRAM 4K instructions = 16K bytes
#define PDSP_IRAM_SIZE_PA           2048
#define PDSP_IRAM_SIZE_SA           4096

//=============================================================================
//
// Platform Memory Map (PA)
//

// Mailbox Module
#define MBOXRAM_BASE                0x00000000
#define MBOXRAM_SIZE                0x00000080

// Platform Report Commands
#define PRCMD_BASE                  0x00000100
#define PRCMD_SIZE                  0x00000004

// Packet ID Manager
#define PACKETID_BASE               0x00000400
#define PACKETID_SIZE               0x00000010

// PDSP
#define PDSP0_BASE                  0x00001000
#define PDSP1_BASE                  0x00001100
#define PDSP2_BASE                  0x00001200
#define PDSP3_BASE                  0x00001300
#define PDSP4_BASE                  0x00001400
#define PDSP5_BASE                  0x00001500
#define PDSP0_DEBUG                 0x00002000
#define PDSP1_DEBUG                 0x00002100
#define PDSP2_DEBUG                 0x00002200
#define PDSP3_DEBUG                 0x00002300
#define PDSP4_DEBUG                 0x00002400
#define PDSP5_DEBUG                 0x00002500
#define PDSP0_TIMER                 0x00003000
#define PDSP1_TIMER                 0x00003100
#define PDSP2_TIMER                 0x00003200
#define PDSP3_TIMER                 0x00003300
#define PDSP4_TIMER                 0x00003400
#define PDSP5_TIMER                 0x00003500
#define PDSP0_IRAM                  0x00010000
#define PDSP1_IRAM                  0x00018000
#define PDSP2_IRAM                  0x00020000
#define PDSP3_IRAM                  0x00028000
#define PDSP4_IRAM                  0x00030000
#define PDSP5_IRAM                  0x00038000

// Stage2 Lookup Engine
#define LUT2_BASE                   0x00005000
#define LUT2_SIZE                   0x00000040

// Statistics
#define STATISTICS_BASE             0x00006000
#define STATISTICS_SIZE             0x00000100

// INTD
#define INTD_BASE                   0x00007000
#define INTD_SIZE                   0x00000400

// Scratch RAM
#define LRAM1_BASE                  0x00040000
#define LRAM1_SIZE                  0x00002000
#define LRAM1_RD                    0
#define LRAM1_WD                    0

#define LRAM2_BASE                  0x00042000
#define LRAM2_SIZE                  0x00002000
#define LRAM2_RD                    0
#define LRAM2_WD                    0

#define LRAM3_BASE                  0x00044000
#define LRAM3_SIZE                  0x00002000
#define LRAM3_RD                    0
#define LRAM3_WD                    0

#define LRAM4_BASE                  0x00046000
#define LRAM4_SIZE                  0x00002000
#define LRAM4_RD                    0
#define LRAM4_WD                    0

// CDE
#define CDE0_BASE                   0x00050000
#define CDE0_SIZE                   0x00008000

#define CDE1_BASE                   0x00058000
#define CDE1_SIZE                   0x00008000

#define CDE2_BASE                   0x00060000
#define CDE2_SIZE                   0x00008000

#define CDE3_BASE                   0x00068000
#define CDE3_SIZE                   0x00008000

#define CDE4_BASE                   0x00070000
#define CDE4_SIZE                   0x00008000

#define CDE5_BASE                   0x00078000
#define CDE5_SIZE                   0x00008000


//=============================================================================
//
// Platform Memory Map (SA)
//

// PDSP
#define SAPDSP0_BASE                0x00001000
#define SAPDSP1_BASE                0x00001100
#define SAPDSP0_DEBUG               0x00001400
#define SAPDSP1_DEBUG               0x00001500
#define SAPDSP0_IRAM                0x00004000
#define SAPDSP1_IRAM                0x00008000

// PDSP Scratch RAM
#define SALRAM1_BASE                0x0000C000
#define SALRAM1_SIZE                0x00002000
#define SALRAM1_RD                  0
#define SALRAM1_WD                  0

#define SALRAM2_BASE                0x0000E000
#define SALRAM2_SIZE                0x00002000
#define SALRAM2_RD                  0
#define SALRAM2_WD                  0

// CDE
#define PHPCDE0_BASE                0x00010000
#define PHPCDE0_SIZE                0x00008000

#define PHPCDE1_BASE                0x00018000
#define PHPCDE1_SIZE                0x00008000

// 64-bit Memory Map
#define VBUS64_BASE                 0x00030000
#define VBUS64_SIZE                 0x00020000

// Scheduler Ports
#define ENGINE_BASE                 0x00030000
#define ENCRYPT_BASE                (ENGINE_BASE+0x0010)
#define ENCRYPT_SIZE                0x0010
#define AUTH_BASE                   (ENGINE_BASE+0x0020)
#define AUTH_SIZE                   0x0010
#define PHP0_BASE                   (ENGINE_BASE+0x0040)
#define PHP0_SIZE                   0x0020
#define EGRESS0_BASE                (ENGINE_BASE+0x0060)
#define EGRESS0_SIZE                0x0010
#define AIRC_BASE                   (ENGINE_BASE+0x0070)
#define AIRC_SIZE                   0x0010
#define PHP1_BASE                   (ENGINE_BASE+0x0080)
#define PHP1_SIZE                   0x0020
#define EGRESS1_BASE                (ENGINE_BASE+0x00A0)
#define EGRESS1_SIZE                0x0010
#define CCMLUP0_BASE                (ENGINE_BASE+0x00B0)
#define CCMLUP0_SIZE                0x0010
#define CCMEOP0_BASE                (ENGINE_BASE+0x00C0)
#define CCMEOP0_SIZE                0x0010
#define CCMLUP1_BASE                (ENGINE_BASE+0x00E0)
#define CCMLUP1_SIZE                0x0010
#define CCMEOP1_BASE                (ENGINE_BASE+0x00F0)
#define CCMEOP1_SIZE                0x0010

// Block Manager
#define BLOCKMGR_BASE               0x00030100
#define BLOCKMGR_SIZE               0x00000010

// Packet RAM
#define PKTRAM0_BASE                0x00031000
#define PKTRAM0_SIZE                0x00000800
#define PKTRAM0_RD                  0
#define PKTRAM0_WD                  0

#define PKTRAM1_BASE                0x00031800
#define PKTRAM1_SIZE                0x00000800
#define PKTRAM1_RD                  0
#define PKTRAM1_WD                  0

#define PKTRAM2_BASE                0x00032000
#define PKTRAM2_SIZE                0x00000800
#define PKTRAM2_RD                  0
#define PKTRAM2_WD                  0

#define PKTRAM3_BASE                0x00032800
#define PKTRAM3_SIZE                0x00000800
#define PKTRAM3_RD                  0
#define PKTRAM3_WD                  0

#define PKTRAM4_BASE                0x00033000
#define PKTRAM4_SIZE                0x00000800
#define PKTRAM4_RD                  0
#define PKTRAM4_WD                  0

#define PKTRAM5_BASE                0x00033800
#define PKTRAM5_SIZE                0x00000800
#define PKTRAM5_RD                  0
#define PKTRAM5_WD                  0

// 256-bit Memory Map
#define VBUS256_BASE                0x00040000
#define VBUS256_SIZE                0x00010000

// Context RAM
#define CTXRAM0_BASE                0x00040000
#define CTXRAM0_SIZE                0x00002800
#define CTXRAM0_RD                  0
#define CTXRAM0_WD                  0

#define CTXRAM1_BASE                0x00042800
#define CTXRAM1_SIZE                0x00002800
#define CTXRAM1_RD                  0
#define CTXRAM1_WD                  0

#define CTXRAM2_BASE                0x00045000
#define CTXRAM2_SIZE                0x00002800
#define CTXRAM2_RD                  0
#define CTXRAM2_WD                  0


//=============================================================================
//
// Streaming Interface Switch
//

// Destination Thread Ids (from the PA PDSP perspective)
#define THREADID_CDE0               0           // Packets to PDSP0
#define THREADID_CDE1               1           // Packets to PDSP1
#define THREADID_CDE2               2           // Packets to PDSP2
#define THREADID_CDE3               3           // Packets to PDSP3
#define THREADID_CDE4               4           // Packets to PDSP4
#define THREADID_CDE5               5           // Packets to PDSP5
#define THREADID_CDMA               6           // Packets to CDMA
#define THREADID_ETHERNET           7           // Packets to Ethernet TX
#define THREADID_ACE0               8           // Placeholder for model
#define THREADID_ACE1               9           // Placeholder for model


//=============================================================================
//
// PDSP
//


//-----------------------------------------------------
//
// Event/Satus Bits
//
#define tStatus_Command0            t0
#define tStatus_Command1            t1
#define tStatus_Command2            t2
#define tStatus_Command3            t3
#define tStatus_CDENewPacket        t4
#define tStatus_CDEHeldPacket       t5
#define tStatus_CDEBusy             t6
#define tStatus_CDEOutPacket        t7
#define tStatus_Timer               t9

// PDSP0, PDSP1, PDSP2
#define tStatus_Lut1Busy            t10

// PDSP3
#define tStatus_Lut2AddDelBusy      t10
#define tStatus_Lut2LookupBusy      t11
#define tStatus_Lut2MatchData       t12
#define tStatus_Lut2Full            t13


//-----------------------------------------------------
//
// Constants
//
#define cLRAM1                      c0      // Pointer to local RAM block 1
#define cLRAM2                      c1      // Pointer to local RAM block 2
#define cLRAM3                      c2      // Pointer to local RAM block 3
#define cLRAM4                      c3      // Pointer to local RAM block 4
#define cCdeInPkt                   c6      // Pointer to CDE new packet input region
#define cCdeOutPkt                  c7      // Pointer to CDE new packet output region
#define cCdeHeldPkt                 c8      // Pointer to CDE held packet region
#define cTimer                      c9      // Pointer to PDSP Timer (PDSP specific)
#define cPacketId                   c10     // Packet ID Allocation
#define cLut2Regs                   c11     // Pointer to LUT2 Registers
#define cStatistics                 c12     // Pointer to statistics block
#define cIntStatus                  c13     // Pointer to INTD Status Reg
#define cIntCount                   c14     // Pointer to INTD Count Registers


//-----------------------------------------------------
//
// XFR Ids
//
#define XID_CDECTRL                 0
#define XID_CDEDATA                 1
#define XID_SCVCTRL                 2
#define XID_SCVDATA                 3
#define XID_LUT1CMD                 2
#define XID_LUT1V1                  3
#define XID_LUT1V2                  4
#define XID_LUT1V3                  5
#define XID_LUT2CMD                 2
#define XID_LUT2DATA                3


#endif



