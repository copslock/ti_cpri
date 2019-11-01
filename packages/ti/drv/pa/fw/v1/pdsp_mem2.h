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

#ifndef _PA_MEM2_H
#define _PA_MEM2_H  1

// ****************************************************************************************
// * FILE PURPOSE: Define the shared memory layout used by the PDSPs in egress pass of PA
// ****************************************************************************************
// * FILE NAME: pa_mem2.h
// *
// * DESCRIPTION: The memory usage and associated defined values are provided for all PDSPs
// *              in the egress path  
// *
// *********************************************************************************

//    address              Description                                Constant value  define value (PAMEM_CONST_...)
//   
//                  --------------------------------------------
//     0xFFF8:0000  |  PDSP0 LUT1 enable MAP  - 32 bytes       |      c14 - pdsp0      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 enable MAP2 -  1 bytes       |                                 PDSP_CXT_OFFSET_L1_MAP2
//                  |        LUT1 pending enable info 20 bytes |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        (access by PDSP0 only)            |
//                  |                                          |      
//     0xFFF8:0100  |  Reserved for more PDSP                  |
//                  |                                          |
//     0xFFF8:0200  |  IP protocol table - 256 bytes           |      c18   psdp0   IP_PROTO   // Classifyer 1 only
//     0xFFF8:0300  |  Reserved for more PDSP                  |      c18   psdp1   IP_PROTO
//     0xFFF8:0400  |  Reserved for more PDSP                  |      c18   psdp2   IP_PROTO
//                  |                                          |
//     0xFFF8:0500  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp0      
//                  |  Reserve room for 32 entries             |                                  
//                  |                                          |
//     0xFFF8:0600  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp1      
//                  |  Reserved for more pDSP                  |
//                  |                                          |
//     0xFFF8:0700  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp2      
//                  |  Reserved for more pDSP                  |
//                  |                                          |
//     0xFFF8:0800  |  Command Buffer: 16 + 64                 |      c27 - EF1, EF3, EF4
//     0xFFF8:0900  |  Command Buffer: 16 + 64                 |      c27 - EF2
//                  |                                          |
//                  |  Reserved for more pDSP                  |
//     0xFFF8:0a00  |  Scrtach Memory for temporary storage    |      OFFSET_TEMP_BUFFER1   // To be determined
//     0xFFF8:0b00  |                                          |      OFFSET_TEMP_BUFFER2
//     0xFFF8:0c00  |                                          |      OFFSET_ESP_PADDING_BUF
//                  |                                          |
//                  |                                          |
//     0xFF02:0000  |  Custom Classify1 info 40 byte per entry |      c19   all pdsps  CUSTOM     OFFSET_CUSTOM_C1  // Global configuration
//                  |      4 entries of 40 byte                |                                                    // Can be accessed by all PDSPs
//                  |  Max header counts  4 bytes              |                                  OFFSET_MAX_HDR
//                  |  Outer IP Reassem Config  4 bytes        |                                  OFFSET_OUT_IP_REASSM_CFG
//                  |  Inner IP Reassem Config  4 bytes        |                                  OFFSET_IN_IP_REASSM_CFG
//                  |  Command Set Config       4 bytes        |                                  OFFSET_CMDSET_CFG
//                  |  User Statistics Config   4 bytes        |                                  OFFSET_USR_STATS_CFG
//                  |  Queue Diversion Config   4 bytes        |                                  OFFSET_QUEUE_DIVERT_CFG
//                  |  IPSEC NAT-T Config       4 bytes        |                                  OFFSET_IPSEC_NAT_T_CFG
//                  |  MAC Padding Config       4 bytes        |                                  OFFSET_MAC_PADDING_CFG
//                  |  Outer IP ACL Config      4 bytes        |                                  OFFSET_OUT_IP_ACL_CFG
//                  |  Inner IP ACL Config      4 bytes        |                                  OFFSET_IN_IP_ACL_CFG
//                  |  Queue Bounce Config      4 bytes        |                                  OFFSET_QUEUE_BOUNCE_CFG
//                  |  System Timestamp         4 bytes        |                                  OFFSET_SYSTEM_TIMESTAMP  = 0xF0
//                  |  IPv6 Fragmentation ID    4 bytes        |                                  OFFSET_IPV6_FRAG_ID      = 0xF8
//     0xFF02:0400  |  Configurable Exception  Routing         |      c20 - all pdsps  EROUTE     // Global Memory 
//                  |  Routing                                 |
//                  |     32 entries of size 16 bytes          |
//                  |                                          |
//     0xFF02:0400  |  Configurable Egress Flow Exception      |      c26 - all pdsps  EROUTE     // Global Memory 
//                  |  Routing                                 |
//                  |     32 entries of size 16 bytes          |
//                  |                                          |
//     0xFFF8:1000  |  Modify PDSP 0 checksum cmd 2 entries    |      c17  - pdsp1     MODIFY      OFFSET_INSERT_CHKSUM  // Egress only  
//                  |      of 16 bytes                         |
//     0xFFF8:1040  |  Modify PDSP 0 blind insert 4 entries    |                                   OFFSET_BLIND_PATCH
//                  |      of 32 bytes                         |
//     0xFFF8:10c0  |  Modify PDSP 0 crc cmd. 1 Entry          |                                   OFFSET_INSERT_CRC
//                  |      of 8 bytes                          |
//     0xFFF8:10d0  |  Time Convertion scratch. 1 Entry        |                                   OFFSET_CONVERT_TIMESTAMP
//                  |      of 12 bytes                         |   
//     0xFFF8:10e0  |  Modify PDSP 0 timestamp cmd             |                                   OFFSET_REPORT_TIMESTAMP
//                  |      8 bytes                             |
//     0xFFF8:10f0  |  Modify PDSP 0 Ip Frag cmd               |                                   OFFSET_IP_FRAG
//                  |      4 bytes                             |
//     0xFFF8:10f8  |  Modify PDSP 0 Patch Message Length cmd  |                                   OFFSET_PATCH_MSG_LEN
//                  |      two entries of 4 bytes              |
//                  |                                          |
//     0xFFF8:1100  |  Ethertypes table 8 entries of 2 bytes   |                                   OFFSET_TX_ETYPE_TABLE
//                  |  Reserve room for 16 entries             |
//                  |                                          |
//                  |  (Egress 0  cluster)                     |                   
//     0xFFF8:1f00  |  PDSP ID, VER core 0 - 8 bytes           |      c23 - pdsp 0     PDSP_INFO  
//     0xFFF8:1f20  |  PDSP ID, VER core 1 - 8 bytes           |      c23 - pdsp 1     PDSP_INFO
//     0xFFF8:1f40  |  PDSP ID, VER core 2 - 8 bytes           |      c23 - pdsp 2     PDSP_INFO
//     0xFFF8:1f60  |  PDSP ID, VER core 3 - 8 bytes           |      c23 - pdsp 3     PDSP_INFO
//                  |                                          |
//                  |  (Egress 1/2 cluster)                    |                    
//     0xFFF8:0f00  |  PDSP ID, VER core 0 - 8 bytes           |      c23 - pdsp 0     PDSP_INFO  
//     0xFFF8:0f20  |  PDSP ID, VER core 1 - 8 bytes           |      c23 - pdsp 1     PDSP_INFO
//     0xFFF8:0f40  |  PDSP ID, VER core 2 - 8 bytes           |      c23 - pdsp 2     PDSP_INFO
//     0xFFF8:0f60  |  PDSP ID, VER core 3 - 8 bytes           |      c23 - pdsp 3     PDSP_INFO
//                                                             |
//     0xFFF8:2000  |  PDSP0 LUT1 associated table memory      |      c0 - pdsp0      PDSP_LUT1_INFO
//                  |     256 entries of size 32 bytes         |      
//                  |     (Single PDSP cluster)                |
//     0xFFF8:4000  |  Egress0 Egress Flow Record1    table    |      c12 - pdsp1/2    PDSP_EF_RECORD0
//                  |     256 entries of size 64 bytes         |      
//     0xFFF8:8000  |  Egress0 Egress Flow Record2    table    |      c13 - pdsp1/2    PDSP_EF_RECORD1
//                  |     256 entries of size 64 bytes         |      
//     0xFFF8:1000  |  Egress1 Egress Flow Record2    table    |      c12 - pdsp0      PDSP_EF_RECORD2
//                  |     256 entries of size 32 bytes         |      
//     0xFFF8:1000  |  Egress2 Egress Flow Record3    table    |      c13 - pdsp0      PDSP_EF_RECORD3
//                  |     256 entries of size 64 bytes         |      
//                  |                                          |
//                  --------------------------------------------

//-----------------------------------------------------
//
// Constants
//
//#define cLRAM1                      c0      // Pointer to LUT1 Info
//#define cLRAM2                      c1      // Pointer to Local SRAM
//#define cLRAM3                      c2      // Pointer to Global SRAM
//#define cLRAM4                      c3      // Pointer to System Timer (PDSP0 Timer)
//#define cMailbox                    c4      // Pointer to Mailbox Registers
//#define cCdeInPkt                   c6      // Pointer to CDE new packet input region
//#define cCdeOutPkt                  c7      // Pointer to CDE new packet output region
//#define cCdeHeldPkt                 c8      // Pointer to CDE held packet region
//#define cTimer                      c9      // Pointer to PDSP Timer (PDSP specific)
//#define cLut1Regs                   c10     // Pointer to LUT1 Registers (not used)
//#define cLut2Regs                   c11     // Pointer to LUT2 Registers
//#define cStatistics                 c12     // Pointer to statistics block (check stats operation)
//#define cIntStatus                  c13     // Pointer to INTD Status Reg   (not used)
//#define cIntCount                   c14     // Pointer to INTD Count Registers (not used)


// Egress 0, PDSP 0
//#define PAMEM_CONST_PDSP_LUT1_INFO      c0    
//#define PAMEM_CONST_EF_RECORD1          c12
//#define PAMEM_CONST_EF_RECORD2          c13
//#define PAMEM_CONST_PDSP_CXT            c14    
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_MODIFY              c17         
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_TEMP_BUF            c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25
//#define PAMEM_CONST_EF_EROUTE           c26

// Egress 0, PDSP 1/2
//#define PAMEM_CONST_EF_RECORD1          c12
//#define PAMEM_CONST_EF_RECORD2          c13
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_MODIFY              c17         
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_TEMP_BUF            c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25
//#define PAMEM_CONST_EF_EROUTE           c26
//#define PAMEM_CONST_EF_CMD              c27


// Egress 1, PDSP 0
//#define PAMEM_CONST_EF_RECORD3          c12
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_MODIFY              c17         
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_TEMP_BUF            c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25
//#define PAMEM_CONST_EF_EROUTE           c26
//#define PAMEM_CONST_EF_CMD              c27

// Egress 2, PDSP 0
//#define PAMEM_CONST_EF_RECORD4          c12
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_MODIFY              c17         
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_TEMP_BUF            c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25
//#define PAMEM_CONST_EF_EROUTE           c26
//#define PAMEM_CONST_EF_CMD              c27

// PASS Scratch Memory Base 
// Local SRAM:  0xFFF80000
// Global SRAM: 0xFF020000
//#define PAMEM_CONST_LOCAL_SRAM_BASE     c1
//#define PAMEM_CONST_GLOBAL_SRAM_BASE    c2

// PASS System Timer (PDSP0 Timer) 0xFF408800
//#define PAMEM_CONST_SYS_TIMER           c3

// Local LUT1 info (Egress 0 PDSP0 only: 0xFFF82000)
//#define PAMEM_CONST_PDSP_LUT1_INFO      c0    

// PDSP Local context (Egress 0 PDSP0 only: 0xFFF80000)
//#define PAMEM_CONST_PDSP_CXT            c14    
//#define PDSP_CXT_OFFSET_L1_MAP          0
//#define PDSP_CXT_OFFSET_L1_MAP2         0x20   // 1 byte 
//#define PDSP_CXT_OFFSET_L1_PENDING      0x24   // 20 bytes 
//#define PDSP_CXT_OFFSET_PENDING_TABLE   0x40   // 32 bytes (not used) 

// Scratch memory 2 (0xFFF80A00)
//#define OFFSET_TEMP_BUFFER1             0xF00        // 0xFFF84000 based (temporary storage
//#define OFFSET_TEMP_BUFFER2             0x4F80       // 0xFFF80000 based (buffer copy)
                                    
// IP Protocol table Ingress Stage  (0xFFF80200): Global configuration only
//#define PAMEM_CONST_IP_PROTO            c18

// Egress Flow Packet Excepotion routing (global scratch: 0xFF020400)
#define PAMEM_CONST_EF_EROUTE             c26

// Egress Flow Command Buffer (0xFF020800, 0xFF020900)
#define PAMEM_CONST_EF_CMD                c27


// Packet Parse (LUT1-PDSP 0xFFF80400,  ..)
//#define PAMEM_CONST_PARSE               c21

// PDSP Info: 0xFFF81F00, 0xFFF80F00
//#define PAMEM_CONST_PDSP_ALL_INFO       c22
//#define PAMEM_CONST_PDSP_INFO           c23
//#define OFFSET_ID                       0
//#define OFFSET_VER                      4

//#define OFFSET_ID_PDSP0                 0x00
//#define OFFSET_ID_PDSP1                 0x20
//#define OFFSET_ID_PDSP2                 0x40

// Temporary storage location (0xFFF80A00)
#define PAMEM_CONST_TEMP_BUF                c24
#define PAMEM_CONST_TEMP_BUF_A_BASE         0xFFF80A00
#define PAMEM_CONST_TEMP_BUF_B_BASE         0xFFF80B00
#define PAMEM_CONST_ESP_PADDING_BUF_BASE    0xFFF80C00

#define OFFSET_TEMP_BUF_A                   0
#define OFFSET_TEMP_BUF_B                   0x100
#define OFFSET_ESP_PADDING_BUF              0x200

// Modify PDSP Egress0, PDSP1 (0xFFF81000)
//#define PAMEM_CONST_MODIFY              c17         
//#define OFFSET_INSERT_CHKSUM            0
//#define OFFSET_BLIND_PATCH              0x40
//#define OFFSET_INSERT_CRC               0xc0
//#define OFFSET_CONVERT_TIMESTAMP        0xd0
//#define OFFSET_REPORT_TIMESTAMP         0xe0
//#define OFFSET_IP_FRAG                  0xf0
//#define OFFSET_PATCH_MSG_LEN            0xf8
#define OFFSET_TX_ETYPE_TABLE             0x100
// Location of the first blind patch command
// It will be the only one supported with IP fragmentation patch
//#define PAMEM_BASE_BLIND_PATCH          0xFFF81040

// Constants as seen by the system
//#define FIRMWARE_P0_LUT1_INFO_BASE      0xFFF82000     // c0
//#define FIRMWARE_P0_LUT1_CONTEXT        0xFFF80000     // c15
//#define FIRMWARE_P0_ID_VER              0xFFF81F00     // c23


//#define FIRMWARE_P1_LUT1_INFO_BASE      0xFFF82000     // c0
//#define FIRMWARE_P1_LUT1_CONTEXT        0xFFF80000     // c15
//#define FIRMWARE_P1_ID_VER              0xFFF81f20     // c23

//#define FIRMWARE_P2_LUT1_INFO_BASE      0xFFF82000     // c0
//#define FIRMWARE_P2_LUT1_CONTEXT        0xFFF80000     // c15
//#define FIRMWARE_P2_ID_VER              0xFFF81f40     // c23

// Egress0: EF Record0: 0xFFF84000 
#define PAMEM_CONST_EF_RECORD1          c12
#define PAMEM_CONST_EF_RECORD1_BASE     0xFFF84000

// Egress0: EF Record1: 0xFFF88000 
#define PAMEM_CONST_EF_RECORD2          c13
#define PAMEM_CONST_EF_RECORD2_BASE     0xFFF88000

// Egress1: EF Record2: 0xFFF81000 
#define PAMEM_CONST_EF_RECORD3          c12
#define PAMEM_CONST_EF_RECORD3_BASE     0xFFF81000

// Egress2: EF Record3 0xFFF81000 
#define PAMEM_CONST_EF_RECORD4          c12
#define PAMEM_CONST_EF_RECORD4_BASE     0xFFF81000

#endif // _PA_MEM2_H

