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

// PDSP IRAM 3K instructions = 12K bytes
// PDSP IRAM 4K instructions = 16K bytes
#define PDSP_IRAM_SIZE_PA           3072
#define PDSP_IRAM_SIZE_SA           4096

// PA Global Address Mapping
#define PA_GLOBAL_BASE             0x03000000
#define PA_GLOBAL_SIZE             0x01000000
#define PA_LOCAL_BASE              0xFF000000
// #define PA_GLOBAL_ADDR(x) ((x & ~0xFF000000)+PA_GLOBAL_BASE)
//#define PA_LOCAL_ADDR(x) (((x) & ~0xFF000000)+PA_LOCAL_BASE)
// RA Model RAM Config
#define RA_MEMBLOCK_BASE            0x90000000      // Software configurable base
#define RA_MEMBLOCK_SIZE            0x41000         // Enough for 4 contexts

//=============================================================================
//
// PA1.5 Top Level Memory Map
//

// Mailbox Module
#define MBOXRAM_BASE                0xFF000000
#define MBOXRAM_SIZE                0x00000200

// Platform Report Commands
#define PRCMD_BASE                  0xFF000500
#define PRCMD_SIZE                  0x00000100
// RA Module
#define RA15_BASE                   0xFF000400
#define RA15_SIZE                   0x00000100
#define RA20_BASE                   0xFF000800
#define RA20_SIZE                   0x00000400

// Statsbloc Module
#define STATSBLOC_REG_BASE          0xFF006000
#define STATSBLOC_REG_SIZE          0x00000100
#define STATSBLOC_QUERY_BASE        0xFF008000
#define STATSBLOC_QUERY_SIZE        0x00004000
#define STATSBLOC_COLLECT_BASE      0xFF00C000
#define STATSBLOC_COLLECT_SIZE      0x00004000

// Local PA RAM (32K)
#define PA_LRAM_BASE                0xFF020000
#define PA_LRAM_SIZE                0x8000
#define PA_LRAM_RD                  0
#define PA_LRAM_WD                  0

//
// Clusters
//

// Cluster 0 : "Ingress 0", (PDSP0, PDSP1)
#define CLUSTER0_BASE               0xFF400000  // PA Local Base Address
#define CLUSTER0_LRAM               0xC000      // Local RAM size
#define CLUSTER0_PRAM               0x4000      // Packet RAM size
#define CLUSTER0_PDSPCNT            2           // Number of PDSP cores
#define CLUSTER0_LUT1FLG            3           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER0_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 1 : "Ingress 1", (PDSP2, PDSP3)
#define CLUSTER1_BASE               0xFF500000  // PA Local Base Address
#define CLUSTER1_LRAM               0xC000      // Local RAM size
#define CLUSTER1_PRAM               0x10000     // Packet RAM size
#define CLUSTER1_PDSPCNT            2           // Number of PDSP cores
#define CLUSTER1_LUT1FLG            3           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER1_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 2 : "Ingress 2", (PDSP4)
#define CLUSTER2_BASE               0xFF600000  // PA Local Base Address
#define CLUSTER2_LRAM               0x6000      // Local RAM size
#define CLUSTER2_PRAM               0x10000     // Packet RAM size
#define CLUSTER2_PDSPCNT            1           // Number of PDSP cores
#define CLUSTER2_LUT1FLG            1           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER2_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 3 : "Ingress 3", (PDSP5)
#define CLUSTER3_BASE               0xFF700000  // PA Local Base Address
#define CLUSTER3_LRAM               0x6000      // Local RAM size
#define CLUSTER3_PRAM               0x10000     // Packet RAM size
#define CLUSTER3_PDSPCNT            1           // Number of PDSP cores
#define CLUSTER3_LUT1FLG            1           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER3_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 4 : "Ingress 4", (PDSP6, PDSP7)
#define CLUSTER4_BASE               0xFF800000  // PA Local Base Address
#define CLUSTER4_LRAM               0x14000     // Local RAM size
#define CLUSTER4_PRAM               0x10000     // Packet RAM size
#define CLUSTER4_PDSPCNT            2           // Number of PDSP cores
#define CLUSTER4_LUT1FLG            1           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER4_LUT2FLG            2           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 5 : "Post Processing", (PDSP8, PDSP9)
#define CLUSTER5_BASE               0xFF900000  // PA Local Base Address
#define CLUSTER5_LRAM               0x4000      // Local RAM size
#define CLUSTER5_PRAM               0x10000     // Packet RAM size
#define CLUSTER5_PDSPCNT            2           // Number of PDSP cores
#define CLUSTER5_LUT1FLG            0           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER5_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 6 : "Egress 0", (PDSP10, PDSP11, PDSP12)
#define CLUSTER6_BASE               0xFFA00000  // PA Local Base Address
#define CLUSTER6_LRAM               0xc000      // Local RAM size
#define CLUSTER6_PRAM               0x10000     // Packet RAM size
#define CLUSTER6_PDSPCNT            3           // Number of PDSP cores
#define CLUSTER6_LUT1FLG            1           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER6_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 7 : "Egress 1", (PDSP13)
#define CLUSTER7_BASE               0xFFB00000  // PA Local Base Address
#define CLUSTER7_LRAM               0x4000      // Local RAM size
#define CLUSTER7_PRAM               0x10000     // Packet RAM size
#define CLUSTER7_PDSPCNT            1           // Number of PDSP cores
#define CLUSTER7_LUT1FLG            0           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER7_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)

// Cluster 8 : "Egress 2", (PDSP14)
#define CLUSTER8_BASE               0xFFC00000  // PA Local Base Address
#define CLUSTER8_LRAM               0x6000      // Local RAM size
#define CLUSTER8_PRAM               0x10000     // Packet RAM size
#define CLUSTER8_PDSPCNT            1           // Number of PDSP cores
#define CLUSTER8_LUT1FLG            0           // LUT1 Flags (0:PDSP0, 1:PDSP1, ...)
#define CLUSTER8_LUT2FLG            0           // LUT2 Flags (0:PDSP0, 1:PDSP1, ...)


//=============================================================================
//
// PA1.5 Cluster Local Memory Map
//
#define CLUSTER_LOCAL_BASE          0xFFF00000
#define CLUSTER_LOCAL_SIZE          0x00100000


// Splitter
#define SPLIT0_BASE                 0xFFF09800
#define SPLIT0_SIZE                 0x00000100


// Checker
#define CHECK1_BASE                 0xFFF19C00
#define CHECK1_SIZE                 0x00000200

#define CHECK2_BASE                 0xFFF29C00
#define CHECK2_SIZE                 0x00000200

#define CHECK3_BASE                 0xFFF39C00
#define CHECK3_SIZE                 0x00000200


// PDSP
#define PDSP0_BASE                  0xFFF08000
#define PDSP0_DEBUG                 0xFFF08400
#define PDSP0_IRAM                  0xFFF0C000
#define PDSP0_TIMER                 0xFFF08800

#define PDSP1_BASE                  0xFFF18000
#define PDSP1_DEBUG                 0xFFF18400
#define PDSP1_IRAM                  0xFFF1C000
#define PDSP1_TIMER                 0xFFF18800

#define PDSP2_BASE                  0xFFF28000
#define PDSP2_DEBUG                 0xFFF28400
#define PDSP2_IRAM                  0xFFF2C000
#define PDSP2_TIMER                 0xFFF28800

#define PDSP3_BASE                  0xFFF38000
#define PDSP3_DEBUG                 0xFFF38400
#define PDSP3_IRAM                  0xFFF3C000
#define PDSP3_TIMER                 0xFFF38800


// CDE
#define CDE0_BASE                   0xFFF00000
#define CDE0_SIZE                   0x00008000

#define CDE1_BASE                   0xFFF10000
#define CDE1_SIZE                   0x00008000

#define CDE2_BASE                   0xFFF20000
#define CDE2_SIZE                   0x00008000

#define CDE3_BASE                   0xFFF30000
#define CDE3_SIZE                   0x00008000


// LUT1
#define LUT1_0_BASE                 0xFFF09000
#define LUT1_0_SIZE                 0x00000100

#define LUT1_1_BASE                 0xFFF19000
#define LUT1_1_SIZE                 0x00000100

#define LUT1_2_BASE                 0xFFF29000
#define LUT1_2_SIZE                 0x00000100

#define LUT1_3_BASE                 0xFFF39000
#define LUT1_3_SIZE                 0x00000100


// LUT2
#define LUT2_0_BASE                 0xFFF09400
#define LUT2_0_SIZE                 0x00000100

#define LUT2_1_BASE                 0xFFF19400
#define LUT2_1_SIZE                 0x00000100

#define LUT2_2_BASE                 0xFFF29400
#define LUT2_2_SIZE                 0x00000100

#define LUT2_3_BASE                 0xFFF39400
#define LUT2_3_SIZE                 0x00000100


// Scratch RAM
#define LRAM0_BASE                  0xFFF80000
#define LRAM0_RD                    0
#define LRAM0_WD                    0

// Packet RAM
#define PRAM0_BASE                  0xFFFC0000
#define PRAM0_RD                    0
#define PRAM0_WD                    0


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
#define SALRAM1_SIZE                0x00004000
#define SALRAM1_RD                  0
#define SALRAM1_WD                  0

#define SALRAM1_2_BASE              0x0000E000     // for Const table only

#define SALRAM2_BASE                0x0002C000
#define SALRAM2_SIZE                0x00004000
#define SALRAM2_RD                  0
#define SALRAM2_WD                  0

#define SALRAM2_2_BASE              0x0002E000     // for Const table only

// CDE
#define PHPCDE0_BASE                0x00010000
#define PHPCDE0_SIZE                0x00008000

#define PHPCDE1_BASE                0x00018000
#define PHPCDE1_SIZE                0x00008000

// RNG Module
#define RNG_BASE                    0x00024000
#define RNG_SIZE                    0x00000080

// 64-bit Memory Map
#define VBUS64_BASE                 0x00030000
#define VBUS64_SIZE                 0x00020000

// Scheduler Ports
#define ENGINE_BASE                 0x00030000
#define ENCRYPT_BASE                (ENGINE_BASE+0x0010)
#define ENCRYPT_SIZE                0x0010
#define AUTH_BASE                   (ENGINE_BASE+0x0020)
#define AUTH_SIZE                   0x0010
#define ENCRYPT2_BASE               (ENGINE_BASE+0x0030)
#define ENCRYPT2_SIZE               0x0010
#define PHP0_BASE                   (ENGINE_BASE+0x0040)
#define PHP0_SIZE                   0x0010
#define AUTH2_BASE                  (ENGINE_BASE+0x0050)
#define AUTH2_SIZE                  0x0010
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
#define BLOCKMGR_SIZE               0x00000020

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

// Increase the total size to 0x8c00 to include 560 (from 480) * 64  context bytes
#define CTXRAM2_BASE                0x00045000
#define CTXRAM2_SIZE                0x00003C00
#define CTXRAM2_RD                  0
#define CTXRAM2_WD                  0


//=============================================================================
//
// Streaming Interface Switch
//

// Destination Thread Ids (from the PA PDSP perspective)
#define THREADID_CDMA0              0           // Packets to Global CDMA
#define THREADID_CDMA1              1           // Packets to Local CDMA
#define THREADID_ETHERNET1          2           // Packets to Ethernet TX
#define THREADID_ETHERNET2          3           // Packets to Ethernet TX
#define THREADID_ETHERNET3          4           // Packets to Ethernet TX
#define THREADID_ETHERNET4          5           // Packets to Ethernet TX
#define THREADID_ETHERNET5          6           // Packets to Ethernet TX
#define THREADID_ETHERNET6          7           // Packets to Ethernet TX
#define THREADID_ETHERNET7          8           // Packets to Ethernet TX
#define THREADID_ETHERNET8          9           // Packets to Ethernet TX
#define THREADID_INGRESS0           10          // Packets to Cluster Ingress 0
#define THREADID_INGRESS1           11          // Packets to Cluster Ingress 1
#define THREADID_INGRESS2           12          // Packets to Cluster Ingress 2
#define THREADID_INGRESS3           13          // Packets to Cluster Ingress 3
#define THREADID_INGRESS4           14          // Packets to Cluster Ingress 4
#define THREADID_POST               15          // Packets to Cluster Post Processing
#define THREADID_EGRESS0            16          // Packets to Cluster Egress 0
#define THREADID_EGRESS1            17          // Packets to Cluster Egress 1
#define THREADID_EGRESS2            18          // Packets to Cluster Egress 2
#define THREADID_REASM              19          // Packets to Reasm Accelerator
#define THREADID_ACE0               20          // Placeholder for model
#define THREADID_ACE1               21          // Placeholder for model
#define THREADID_STATSBLOC          22          // Packets to Statsbloc (no direct access)

#define PASS_RA_QUEUE               (896 + 17)
#define PASS_RA_LOC_QUEUE           (17)
//#define PASS_RA_LOC_QUEUE           (896 + 17)

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
#define tStatus_PInfo_In            t20
#define tStatus_PInfo_Out           t21
#define tStatus_PInfo_BufA          t22
#define tStatus_PInfo_BufB          t23
#define tStatus_Stat_MysteryBit0    t24
#define tStatus_Stat_MysteryBit1    t25
#define tStatus_Stat_MysteryBit2    t26
#define tStatus_Timer               t31

// PDSP with LUT1
#define tStatus_Lut1Busy            t16

// PDSP with LUT2
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
#define cMailbox                    c4      // Pointer to Mailbox Registers
#define cCdeInPkt                   c6      // Pointer to CDE new packet input region
#define cCdeOutPkt                  c7      // Pointer to CDE new packet output region
#define cCdeHeldPkt                 c8      // Pointer to CDE held packet region
#define cTimer                      c9      // Pointer to PDSP Timer (PDSP specific)
#define cLut1Regs                   c10     // Pointer to LUT1 Registers
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
#define XID_LUT2CMD                 6
#define XID_LUT2DATA                7
#define XID_PINFO_SRC               8
#define XID_PINFO_DST               9
#define XID_PINFO_A                 10
#define XID_PINFO_B                 11
#define XID_PINFO_B_POP             12

//-----------------------------------------------------
//
// LUT1 Debug Control
//
#define LUT1_DBG_CNTL_REG_OFFSET          0x10
// #define LUT1_DBG_ENTRY_DATA_REG0_OFFSET   0x40
// #define LUT1_DBG_ENTRY_DATA_REG1_OFFSET   0x44
// #define LUT1_DBG_ENTRY_DATA_REG2_OFFSET   0x48
// #define LUT1_DBG_ENTRY_DATA_REG3_OFFSET   0x4C
// #define LUT1_DBG_ENTRY_DATA_REG4_OFFSET   0x50
// #define LUT1_DBG_ENTRY_DATA_REG5_OFFSET   0x54
// #define LUT1_DBG_ENTRY_DATA_REG6_OFFSET   0x58
// #define LUT1_DBG_ENTRY_DATA_REG7_OFFSET   0x5C
// #define LUT1_DBG_ENTRY_DATA_REG8_OFFSET   0x60
// #define LUT1_DBG_ENTRY_DATA_REG9_OFFSET   0x64
// #define LUT1_DBG_ENTRY_DATA_REG10_OFFSET  0x68
// #define LUT1_DBG_ENTRY_DATA_REG11_OFFSET  0x6C
// #define LUT1_DBG_ENTRY_DATA_REG12_OFFSET  0x70
// #define LUT1_DBG_ENTRY_DATA_REG13_OFFSET  0x74
// #define LUT1_DBG_ENTRY_DATA_REG14_OFFSET  0x78
// #define LUT1_DBG_ENTRY_DATA_REG15_OFFSET  0x7C

#define LUT1_DBG_ENTRY_DATA_REG_VIEW1_OFFSET  0x44
#define LUT1_DBG_ENTRY_DATA_REG_VIEW2_OFFSET  0x54
#define LUT1_DBG_ENTRY_DATA_REG_VIEW3_OFFSET1 0x40
#define LUT1_DBG_ENTRY_DATA_REG_VIEW3_OFFSET2 0x64
#define LUT1_DBG_ENTRY_DATA_REG12_OFFSET      0x70

#endif



