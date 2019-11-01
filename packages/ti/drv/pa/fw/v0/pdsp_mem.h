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

#ifndef _PA_MEM_H
#define _PA_MEM_H  1

// *********************************************************************************
// * FILE PURPOSE: Define the shared memory layout used by the PDSPs in PA
// *********************************************************************************
// * FILE NAME: pa_mem.h
// *
// * DESCRIPTION: The memory usage and associated defined values are provided
// *
// *********************************************************************************

//    address              Description                                Constant value  define value (PAMEM_CONST_...)
//   
//                  --------------------------------------------
//     0x4:0000     |  PASS_SCRATCH0_BASE                      |      c0 - all pdsps   PASS_SCRATCH0_BASE
//     0x4:0000     |  PDSP0 LUT1 enable MAP - 8 bytes         |      c25 - pdsp0      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 pending enable info 4 bytes  |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        LUT1 pending table data 32 bytes  |                                 PDSP_CXT_OFFSET_PENDING_TABLE
//                  |        (access by PDSP0 only)            |
//                  |                                          |      
//     0x4:0080     |  PDSP0 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP0 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//                  |                                          |      
//     0x4:0100     |  PDSP1 LUT1 enable MAP - 4 bytes         |      c25 - pdsp1      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 pending enable info 4 bytes  |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        LUT1 pending table data 32 bytes  |                                 PDSP_CXT_OFFSET_PENDING_TABLE
//                  |        (access by PDSP1 only)            |
//                  |                                          |      
//     0x4:0180     |  PDSP1 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP1 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//                  |                                          |      
//     0x4:0200     |  PDSP2 LUT1 enable MAP - 4 bytes         |      c25 - pdsp2      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 pending enable info 4 bytes  |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        LUT1 pending table data 32 bytes  |                                 PDSP_CXT_OFFSET_PENDING_TABLE
//                  |        (access by PDSP2 only)            |
//                  |                                          |      
//     0x4:0280     |  PDSP2 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP2 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//                  |                                          |      
//     0x4:0300     |  PDSP3                                   |      c25 - pdsp3    
//                                                                    c26 - pdsp4/5
//     0x4:0300     |  PDSP5 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_TX_USR_STATS_FIFO_CB
//                  |  PDSP5 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_TX_USR_STATS_FIFO
//                  |                                          |                       
//                  |        (access by PDSP3 only)            |
//                  |                                          |      
//     0x4:0380     |  PDSP3 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP3 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//                  |                                          |      
//                  |                                          |
//     0x4:0400     |  User-defined Statistics Control blocks  |      OFFSET_USR_STATS_CB    
//                  |        512 entries of size 2 bytes       |      
//                  |                                          |
//     0x4:0800     |  User-defined Statistics Counter blocks  |      OFFSET_USR_STATS_64B_COUNTERS
//                  |        256 entries of size 8 bytes       |      
//                  |                                          |
//                  |  User-defined Statistics Counter blocks  |      OFFSET_USR_STATS_32B_COUNTERS
//                  |        512 entries of size 4 bytes       |      
//                  |                                          |
//                  |  Combination of 64B and 32B counters =   |
//                  |    2048 bytes                            |
//                  |                                          |
//     0x4:1000     |  PDSP0 LUT1 associated table memory      |      c17 - pdsp0      PDSP_LUT1_INFO
//                  |     64 entries of size 64 bytes          |      
//                  |                                          |
//     0x4:2000     |  PASS_SCRATCH1_BASE                      |      c1 - all pdsps   PASS_SCRATCH1_BASE
//     0x4:2000     |  Command Set table                       |      c1 - all pdsps   CMDSET_TABLE
//                  |     64 Entries of  64 bytes  or          |                                   
//                  |     32 Entries of 128 bytes              |                                   
//                  |                                          |
//     0x4:3000     |  PDSP1 LUT1 associated table memory      |      c17 - pdsp1      PDSP_LUT1_INFO
//                  |     64 entries of size 64 bytes          |      
//                  |                                          |
//     0x4:4000     |  PASS_SCRATCH2_BASE                      |      c2 - all pdsps   PASS_SCRATCH2_BASE
//                  |                                          |
//     0x4:4000     |  Outer IP traffic Flows                  |      c26 - pdsp1
//                  |        32 entries of size 16 bytes       |      
//                  |                                          |
//     0x4:4400     |  Inner IP traffic Flows                  |      c26 - pdsp2
//                  |        32 entries of size 16 bytes       |      
//                  |                                          |
//     0x4:4800     |  Outer IP Reassembly Control Block       |      c27 - pdsp1
//                  |        16 bytes                          |      
//                  |                                          |
//     0x4:4900     |  Inner IP Reassembly Control Block       |      c27 - pdsp2
//                  |        16 bytes                          |      
//                  |                                          |
//     0x4:4F00     |  Scrtach Memory for temporary storage    |      OFFSET_TEMP_BUFFER1
//     0x4:4F80     |                                          |      OFFSET_TEMP_BUFFER2
//
//     0x4:5000     |  PDSP2 LUT1 associated table memory      |      c17 - pdsp2      PDSP_LUT1_INFO
//                  |     64 entries of size 64 bytes          |      
//                  |                                          |
//     0x4:6000     |  PASS_SCRATCH3_BASE                      |      c3 -  all pdsps  PASS_SCRATCH3_BASE
//     0x4:6000     |  IP protocol table - 256 bytes           |      c3  - all pdsps  IP_PROTO
//                  |                                          |
//     0x4:6100     |  Custom Classify1 info 40 byte per entry |      c19 - all pdsps  CUSTOM     OFFSET_CUSTOM_C1
//                  |      4 entries of 40 byte                |
//                  |  Max header counts        4 bytes        |                                  OFFSET_MAX_HDR
//                  |  Outer IP Reassem Config  4 bytes        |                                  OFFSET_OUT_IP_REASSM_CFG
//                  |  Inner IP Reassem Config  4 bytes        |                                  OFFSET_IN_IP_REASSM_CFG
//                  |  Command Set Config       4 bytes        |                                  OFFSET_CMDSET_CFG
//                  |  User Statistics Config   4 bytes        |                                  OFFSET_USR_STATS_CFG
//                  |  Queue Diversion Config   4 bytes        |                                  OFFSET_QUEUE_DIVERT_CFG
//                  |  IPSEC NAT-T Config       4 bytes        |                                  OFFSET_IPSEC_NAT_T_CFG
//                  |  MAC Padding Config       4 bytes        |                                  OFFSET_MAC_PADDING_CFG
//                  |  Queue Bounce Config      4 bytes        |                                  OFFSET_QUEUE_BOUNCE_CFG
//                  |                                          |
//     0x4:6200     |  Configurable Exception Routing          |      c20 - all pdsps  EROUTE     
//                  |     32 entries of size 16 bytes          |
//                  |                                          |
//     0x4:6400     |  Parse Call Table  24 entries of 2 bytes |      c21 - all pdsps  PARSE      
//                  |  Reserve room for 32 entries             |                                  
//                  |  Ethertypes table 8 entries of 2 bytes   |                                  OFFSET_ETYPE_TABLE
//                  |  Reserve room for 16 entries             |
//                  |  System Timestamp 8 bytes                |                                  OFFSET_SYS_TIMESTAMP
//                  |  IPv6 Fragmentation ID 4 bytes           |                                  OFFSET_IPV6_FRAG_ID
//                  |  System Timestamp Temp Storage           |                                  OFFSET_TIMESTAMP_TMP
//                  |                                          |
//     0x4:6480     |  Parse Call Table  24 entries of 2 bytes |      For PDSP1/2 
//                  |  Reserve room for 32 entries             |      Note: Create seperate call table to save program space for
//                  |                                          |            new features
//                  |                                          |
//                  |                                          |
//     0x4:6600     |  Mutliple Routing table                  |      c22 - all pdsps  MULTI_ROUTE
//                  |     32 Entries of 64 bytes               |
//                  |                                          |
//     0x4:6E00     |  Custom Classify2 info 16 bytes          |      c26 - pdsp0, pdsp3           OFFSET_CUSTOM_C2 
//                  |      16 entries of 16 byte               |
//
//     0x4:7000     |  Modify PDSP 0 checksum cmd 2 entries    |      c25  - pdsp 4    MODIFY      OFFSET_INSERT_CHKSUM    
//                  |      of 16 bytes                         |
//     0x4:7040     |  Modify PDSP 0 blind insert 4 entries    |                                   OFFSET_BLIND_PATCH
//                  |      of 32 bytes                         |
//     0x4:70c0     |  Modify PDSP 0 crc cmd. 1 Entry          |                                   OFFSET_INSERT_CRC
//                  |      of 8 bytes                          |
//     0x4:70e0     |  Modify PDSP 0 timestamp cmd             |                                   OFFSET_REPORT_TIMESTAMP
//                  |      8 bytes                             |
//     0x4:70f0     |  Modify PDSP 0 Ip Frag cmd               |                                   OFFSET_IP_FRAG
//                  |      4 bytes                             |
//     0x4:70f8     |  Modify PDSP 0 Patch Message Length cmd  |                                   OFFSET_PATCH_MSG_LEN
//                  |      two entries of 4 bytes              |
//     0x4:7100     |  Modify PDSP 1 checksum cmd 2 entries    |      c25  - pdsp 5                OFFSET_INSERT_CHKSUM
//                  |      of 8 bytes                          |    
//     0x4:7140     |  Modify PDSP 1 blink insert 4 entries    |                                   OFFSET_BLIND_PATCH
//                  |      of 16 bytes                         |
//     0x4:71c0     |  Modify PDSP 1 crc cmd. 1 Entry          |                                   OFFSET_INSERT_CRC
//                  |      of 8 bytes                          |                                   
//     0x4:71e0     |  Modify PDSP 0 timestamp cmd             |                                   OFFSET_REPORT_TIMESTAMP
//                  |      8 bytes                             |
//     0x4:71f0     |  Modify PDSP 0 Ip Frag cmd               |                                   OFFSET_IP_FRAG
//                  |      4 bytes                             |
//     0x4:71f8     |  Modify PDSP 0 Patch Message Length cmd  |                                   OFFSET_PATCH_MSG_LEN
//                  |      two entries of 4 bytes              |
//                  |                                          | 
//     0x4:7300     |  Egress Packet Capture Config   40 bytes |      c30 - all pdsps              OFFSET_EGRESS_PKT_CAP_CFG
//                  |    5 entries of size 8 bytes             |
//                  |  Ingress Packet Capture Config  40 bytes |                                   OFFSET_INGRESS_PKT_CAP_CFG
//                  |    5 entries of size 8 bytes             |
//                  |                                          | 
//     0x4:73FC     |  EQoS Default Priority           4 bytes |                                   OFFSET_EQOS_CFG_EG_DEF_PRI
//                  |                                          | 
//     0x4:7400     |  Port Default Route Config     256 bytes |                                   OFFSET_DEFAULT_ROUTE_CFG_PORTS
//                  |    4 entries of size 64 bytes            |
//                  |                                          | 
//     0x4:7500     |  Port EQoS Config             1024 bytes |                                   OFFSET_EQOS_CFG_BASE
//                  |    4 entries of size 256 bytes           |
//                  |                                          | 
//     0x4:7f00     |  PDSP ID, VER core 0 - 8 bytes           |      c24 all cores -  PDSP_ALL_INFO 
//                  |                                          |      c23 - pdsp 0     PDSP_INFO  OFFSET_ID, OFFSET_VER
//     0x4:7f20     |  PDSP ID, VER core 1 - 12 bytes          |      c23 - pdsp 1     PDSP_INFO
//     0x4:7f40     |  PDSP ID, VER core 2 - 12 bytes          |      c23 - pdsp 2     PDSP_INFO
//     0x4:7f60     |  PDSP ID, VER core 3 - 12 bytes          |      c23 - pdsp 3     PDSP_INFO
//     0x4:7f80     |  PDSP ID, VER core 4 - 12 bytes          |      c23 - pdsp 4     PDSP_INFO
//     0x4:7fa0     |  PDSP ID, VER core 5 - 12 bytes          |      c23 - pdsp 5     PDSP_INFO
//                  |                                          |
//                  --------------------------------------------
//     
//

// PASS Scratch Memory Base
#define PAMEM_CONST_SCRATCH0_BASE       c0
#define PAMEM_CONST_SCRATCH1_BASE       c1
#define PAMEM_CONST_SCRATCH2_BASE       c2
#define PAMEM_CONST_SCRATCH3_BASE       c3

// Scratch Memory 0: 0x40000
// PASS User-defined Statistics 
#define PAMEM_USR_STATS_BASE            c0
#define OFFSET_PDSP0_USR_STATS_FIFO_CB  0x80
#define OFFSET_PDSP0_USR_STATS_FIFO     0x90
#define OFFSET_PDSP1_USR_STATS_FIFO_CB  0x180
#define OFFSET_PDSP1_USR_STATS_FIFO     0x190
#define OFFSET_PDSP2_USR_STATS_FIFO_CB  0x280
#define OFFSET_PDSP2_USR_STATS_FIFO     0x290
#define OFFSET_PDSP3_USR_STATS_FIFO_CB  0x380
#define OFFSET_PDSP3_USR_STATS_FIFO     0x390

#define OFFSET_USR_STATS_CB             0x400     
#define OFFSET_USR_STATS_COUNTERS       0x800     

#define PAMEM_CONST_USR_STATS_CB        c28  // PDSP 4/5 only
#define PAMEM_CONST_USR_STATS_COUNTERS  c29  // PDSP 4/5 only

// PASS User-defined Statistics FIFO in Egress direction (0x40300)
#define OFFSET_PDSP5_USR_STATS_FIFO_CB  0x300
#define OFFSET_PDSP5_USR_STATS_FIFO     0x310

#define PAMEM_CONST_TX_USR_STATS_FIFO      c26  // PDSP 4/5 only  
#define OFFSET_PDSP_TX_USR_STATS_FIFO_CB   0x00
#define OFFSET_PDSP_TX_USR_STATS_FIFO      0x10

// PASS User-defined Statistics localtion (0x040800)
#define PAMEM_USR_STATS_COUNTERS           0x040800

//  PDSP specific LUT1 info
//  Note: Need to add 0x1000 to access 0x41000, 0x43000 and 0x43000
#define PAMEM_CONST_PDSP0_LUT1_BASE     c0
#define PAMEM_CONST_PDSP1_LUT1_BASE     c1
#define PAMEM_CONST_PDSP2_LUT1_BASE     c2

// Local LUT1 info (0x41000, 0x43000, 0x45000)
#define PAMEM_CONST_PDSP_LUT1_INFO      c17    

// PDSP Local context
#define PAMEM_CONST_PDSP_CXT            c25    
#define PDSP_CXT_OFFSET_L1_MAP          0
#define PDSP_CXT_OFFSET_L1_PENDING      8
#define PDSP_CXT_OFFSET_PENDING_TABLE   12
#define OFFSET_PDSP_USR_STATS_FIFO_CB   0x80
#define OFFSET_PDSP_USR_STATS_FIFO      0x90

// Scratch Memory 1 (0x42000)
// Command sets

// Scratch memory 2 (0x44000)
// IP traffic flow and reassembly control blocks
#define OFFSET_TEMP_BUFFER1             0xF00        // 0x44000 based (temporary storage
#define OFFSET_TEMP_BUFFER2             0x4F80       // 0x40000 based (buffer copy)

// Scratch Memory 3 (0x46000)
// IP Protocol table
#define PAMEM_CONST_IP_PROTO            c3

// Custom Configuration
#define PAMEM_CONST_CUSTOM              c19
#define CUSTOM_C1_SIZE                  40
#define CUSTOM_C2_SIZE                  16
#define OFFSET_CUSTOM_C1                0
#define OFFSET_MAX_HDR                  OFFSET_CUSTOM_C1 + (4*CUSTOM_C1_SIZE)
#define OFFSET_OUT_IP_REASSM_CFG        OFFSET_MAX_HDR + 4
#define OFFSET_IN_IP_REASSM_CFG         OFFSET_OUT_IP_REASSM_CFG + 4
#define OFFSET_CMDSET_CFG               OFFSET_IN_IP_REASSM_CFG + 4
#define OFFSET_USR_STATS_CFG            OFFSET_CMDSET_CFG + 4
#define OFFSET_QUEUE_DIVERT_CFG         OFFSET_USR_STATS_CFG + 4
#define OFFSET_IPSEC_NAT_T_CFG          OFFSET_QUEUE_DIVERT_CFG + 4
#define OFFSET_MAC_PADDING_CFG          OFFSET_IPSEC_NAT_T_CFG + 4
#define OFFSET_QUEUE_BOUNCE_CFG         OFFSET_MAC_PADDING_CFG + 4


// Custom Configuration
#define PAMEM_CONST_CUSTOM2             c26
#define OFFSET_CUSTOM_C2                0

// Packet Capture configuration
#define PAMEM_CONST_PORTCFG             c30 // all pdsps
#define PACKET_CAP_CFG_SIZE             8
#define DEFAULT_ROUTE_CFG_SIZE          64
#define DEFAULT_ROUTE_CTRL_SIZE         4
#define EQOS_CFG_SIZE                   256

#define OFFSET_EGRESS_PKT_CAP_CFG_BASE  0
#define OFFSET_INGRESS_PKT_CAP_CFG_BASE OFFSET_EGRESS_PKT_CAP_CFG_BASE + 5*PACKET_CAP_CFG_SIZE

#define OFFSET_DEFAULT_ROUTE_CFG_BASE   OFFSET_EGRESS_PKT_CAP_CFG_BASE + 0x100

//  egress global default priority stored after all port configuration is done
#define OFFSET_EQOS_CFG_BASE            OFFSET_DEFAULT_ROUTE_CFG_BASE + 4*DEFAULT_ROUTE_CFG_SIZE
#define OFFSET_EQOS_CFG_EG_DEF_PRI      OFFSET_DEFAULT_ROUTE_CFG_BASE - 4 


#ifdef TO_BE_DELETE

#define OFFSET_EGRESS_PKT_CAP_CFG_P0    0

#define OFFSET_EGRESS_PKT_CAP_CFG_P1    OFFSET_EGRESS_PKT_CAP_CFG_P0  + PACKET_CAP_CFG_SIZE
#define OFFSET_EGRESS_PKT_CAP_CFG_P2    OFFSET_EGRESS_PKT_CAP_CFG_P1  + PACKET_CAP_CFG_SIZE
#define OFFSET_EGRESS_PKT_CAP_CFG_P3    OFFSET_EGRESS_PKT_CAP_CFG_P2  + PACKET_CAP_CFG_SIZE
#define OFFSET_EGRESS_PKT_CAP_CFG_P4    OFFSET_EGRESS_PKT_CAP_CFG_P3  + PACKET_CAP_CFG_SIZE


#define OFFSET_INGRESS_PKT_CAP_CFG_P0   5*PACKET_CAP_CFG_SIZE

#define OFFSET_INGRESS_PKT_CAP_CFG_P1   OFFSET_INGRESS_PKT_CAP_CFG_P0 + PACKET_CAP_CFG_SIZE
#define OFFSET_INGRESS_PKT_CAP_CFG_P2   OFFSET_INGRESS_PKT_CAP_CFG_P1 + PACKET_CAP_CFG_SIZE
#define OFFSET_INGRESS_PKT_CAP_CFG_P3   OFFSET_INGRESS_PKT_CAP_CFG_P2 + PACKET_CAP_CFG_SIZE
#define OFFSET_INGRESS_PKT_CAP_CFG_P4   OFFSET_INGRESS_PKT_CAP_CFG_P3 + PACKET_CAP_CFG_SIZE


#define OFFSET_DEFAULT_ROUTE_CFG_PORTS  OFFSET_EGRESS_PKT_CAP_CFG_P0 +  0x100


#define OFFSET_DEFAULT_ROUTE_CTRL_P1    OFFSET_DEFAULT_ROUTE_CFG_PORTS
#define OFFSET_DEFAULT_MC_ROUTE_CFG_P1  OFFSET_DEFAULT_ROUTE_CTRL_P1   + PACKET_ROUTE_CTRL_SIZE
#define OFFSET_DEFAULT_BC_ROUTE_CFG_P1  OFFSET_DEFAULT_MC_ROUTE_CFG_P1 + PACKET_ROUTE_CFG_SIZE
#define OFFSET_DEFAULT_UC_ROUTE_CFG_P1  OFFSET_DEFAULT_BC_ROUTE_CFG_P1 + PACKET_ROUTE_CFG_SIZE

#define OFFSET_DEFAULT_ROUTE_CTRL_P2    OFFSET_DEFAULT_UC_ROUTE_CFG_P1 + PACKET_ROUTE_CFG_SIZE + PACKET_ROUTE_HOLE
#define OFFSET_DEFAULT_MC_ROUTE_CFG_P2  OFFSET_DEFAULT_ROUTE_CTRL_P2   + PACKET_ROUTE_CTRL_SIZE
#define OFFSET_DEFAULT_BC_ROUTE_CFG_P2  OFFSET_DEFAULT_MC_ROUTE_CFG_P2 + PACKET_ROUTE_CFG_SIZE
#define OFFSET_DEFAULT_UC_ROUTE_CFG_P2  OFFSET_DEFAULT_BC_ROUTE_CFG_P2 + PACKET_ROUTE_CFG_SIZE

#define OFFSET_DEFAULT_ROUTE_CTRL_P3    OFFSET_DEFAULT_UC_ROUTE_CFG_P2 + PACKET_ROUTE_CFG_SIZE + PACKET_ROUTE_HOLE
#define OFFSET_DEFAULT_MC_ROUTE_CFG_P3  OFFSET_DEFAULT_ROUTE_CTRL_P3   + PACKET_ROUTE_CTRL_SIZE
#define OFFSET_DEFAULT_BC_ROUTE_CFG_P3  OFFSET_DEFAULT_MC_ROUTE_CFG_P3 + PACKET_ROUTE_CFG_SIZE
#define OFFSET_DEFAULT_UC_ROUTE_CFG_P3  OFFSET_DEFAULT_BC_ROUTE_CFG_P3 + PACKET_ROUTE_CFG_SIZE

#define OFFSET_DEFAULT_ROUTE_CTRL_P4    OFFSET_DEFAULT_UC_ROUTE_CFG_P3 + PACKET_ROUTE_CFG_SIZE + PACKET_ROUTE_HOLE
#define OFFSET_DEFAULT_MC_ROUTE_CFG_P4  OFFSET_DEFAULT_ROUTE_CTRL_P4   + PACKET_ROUTE_CTRL_SIZE
#define OFFSET_DEFAULT_BC_ROUTE_CFG_P4  OFFSET_DEFAULT_MC_ROUTE_CFG_P4 + PACKET_ROUTE_CFG_SIZE
#define OFFSET_DEFAULT_UC_ROUTE_CFG_P4  OFFSET_DEFAULT_BC_ROUTE_CFG_P4 + PACKET_ROUTE_CFG_SIZE

//  egress global default priority stored after all port configuration is done
#define OFFSET_EQOS_CFG_BASE            OFFSET_DEFAULT_ROUTE_CFG_PORTS + 0x100
#define OFFSET_EQOS_CFG_PORT1           OFFSET_EQOS_CFG_BASE
#define OFFSET_EQOS_CFG_EG_DEF_PRI      OFFSET_DEFAULT_ROUTE_CFG_PORTS - 4 
#endif

// Packet Error routing
#define PAMEM_CONST_EROUTE              c20

// Packet Parse
#define PAMEM_CONST_PARSE               c21
#define OFFSET_ETYPE_TABLE              64
#define OFFSET_SYS_TIMESTAMP            96
#define OFFSET_IPV6_FRAG_ID             104
#define OFFSET_TIMESTAMP_TMP            108

// Multiple Destination Packet Routing
#define PAMEM_CONST_MULTI_ROUTE         c22
#define PAMEM_BASE_MULTI_ROUTE          0x46600

// Command Set Table
#define PAMEM_CONST_CMDSET_TABLE        c1
#define PAMEM_BASE_CMDSET_TABLE         0x42000

// PDSP Info
#define PAMEM_CONST_PDSP_INFO           c23
#define PAMEM_CONST_PDSP_ALL_INFO       c24
#define OFFSET_ID                       0
#define OFFSET_VER                      4

#define OFFSET_ID_PDSP0                 0x00
#define OFFSET_ID_PDSP1                 0x20
#define OFFSET_ID_PDSP2                 0x40
#define OFFSET_ID_PDSP3                 0x60
#define OFFSET_ID_PDSP4                 0x80
#define OFFSET_ID_PDSP5                 0xa0


// Stats location is used directly for patch
#define STATS_DATA_ADDR                 0x6020

// Offsets from stats base address
#define OFFSET_STATS_REVID              0x0
#define OFFSET_STATS_SOFT_RESET         0x4
#define OFFSET_STATS_INC_FLAGS          0x8
#define OFFSET_STATS_CAPTURE            0xc

// Modify PDSP
#define PAMEM_CONST_MODIFY              c25         // PDSP4/5
#define OFFSET_INSERT_CHKSUM            0
#define OFFSET_BLIND_PATCH              0x40
#define OFFSET_INSERT_CRC               0xc0
#define OFFSET_REPORT_TIMESTAMP         0xe0
#define OFFSET_IP_FRAG                  0xf0
#define OFFSET_PATCH_MSG_LEN            0xf8
#define PAMEM_BASE_BLIND_PATCH          0x47040

// IP Reassembly-assistance PDSP1 and PDSP2
#define PAMEM_CONST_TF_TABLE            c26
#define PAMEM_CONST_IP_REASSEM_CONTEXT  c27

// Constants as seen by the system
#define FIRMWARE_P0_LUT1_INFO_BASE      0x40000     // c16
#define FIRMWARE_P0_LUT1_CONTEXT        0x41000     // c17
#define FIRMWARE_P0_ID_VER              0x47f00     // c23


#define FIRMWARE_P1_LUT1_INFO_BASE      0x42000     // c16
#define FIRMWARE_P1_LUT1_CONTEXT        0x43000     // c17
#define FIRMWARE_P1_ID_VER              0x47f20     // c23

#define FIRMWARE_P2_LUT1_INFO_BASE      0x44000     // c16
#define FIRMWARE_P2_LUT1_CONTEXT        0x45000     // c17
#define FIRMWARE_P2_ID_VER              0x47f40     // c23

#define FIRMWARE_P3_ID_VER              0x47f60     // c23

#define FIRMWARE_P4_MODIFY              0x47000     // c25
#define FIRMWARE_P4_ID_VER              0x47f80     // c23

#define FIRMWARE_P5_MODIFY              0x47100     // c25
#define FIRMWARE_P5_ID_VER              0x47fa0     // c23

#define FIRMWARE_CUSTOM_C1_INFO         0x46100     // c19
#define FIRMWARE_ERROR_ROUTE            0x46200     // c20
#define FIRMWARE_PARSE_CALL_TABLE       0x46400     // c21

#define FIRMWARE_MULTI_ROUTE_TABLE      0x46600     // c22
#define FIRMWARE_PDSPID_VER_ALL         0x47f00     // c24

// Mailbox bases
#define FIRMWARE_P0_MBOX                0x0000      // c4
#define FIRMWARE_P1_MBOX                0x0010      // c4
#define FIRMWARE_P2_MBOX                0x0020      // c4
#define FIRMWARE_P3_MBOX                0x0030      // c4
#define FIRMWARE_P4_MBOX                0x0040      // c4
#define FIRMWARE_P5_MBOX                0x0050      // c4

#define FIRMWARE_MBOX                   c4
#define PDSP_TIMER                      c9

// Constant values             
#define FIRMWARE_CONSTANT_21K           0x5400    
#endif // _PA_MEM_H

