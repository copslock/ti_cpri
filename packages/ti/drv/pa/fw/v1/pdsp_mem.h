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
//     0xFFF8:0000  |  PDSP0 LUT1 enable MAP  - 32 bytes       |      c15 - pdsp0      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 enable MAP2 -  1 bytes       |                                 PDSP_CXT_OFFSET_L1_MAP2
//                  |        LUT1 pending enable info 20 bytes |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        EXTENDED Header Storage 20 bytes  |                                 OFFSET_EXT_HDR_PENDING
//                  |        (access by PDSP0 only)            |
//                  |                                          |      
//     0xFFF8:0100  |  PDSP1 LUT1 enable MAP  - 32 bytes       |      c15 - pdsp1      PDSP_CXT  PDSP_CXT_OFFSET_L1_MAP
//                  |        LUT1 enable MAP2 -  1 bytes       |                                 PDSP_CXT_OFFSET_L1_MAP2
//                  |        LUT1 pending enable info 20 bytes |                                 PDSP_CXT_OFFSET_L1_PENDING
//                  |        (access by PDSP1 only)            |
//                  |                                          |      
//     0xFFF8:0200  |  Reserved for more PDSP                  |
//                  |                                          |      c0 
//     0xFFF8:0000  |  USR STATS CB and FIFOs                  |      // Post Processing (Used by PDSP0 only)  
//     0xFFF8:0000  |  PDSP0 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP0 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//     0xFFF8:0040  |  PDSP1 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP1 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//     0xFFF8:0080  |  PDSP2 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP2 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//     0xFFF8:00C0  |  PDSP3 USR STATS FIFO CB - 8 bytes       |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP3 USR STATS FIFO 8 * 4  bytes       |      OFFSET_PDSP_USR_STATS_FIFO
//                  |       ....                               |      
//     0xFFF8:03C0  |  PDSP15 USR STATS FIFO CB - 8 bytes      |      OFFSET_PDSP_USR_STATS_FIFO_CB
//                  |  PDSP15 USR STATS FIFO 8 * 4  bytes      |      OFFSET_PDSP_USR_STATS_FIFO
//                  |                                          |
//                  |                                          |      c15 
//     0xFFF8:0400  |  User-defined Statistics Control blocks  |      OFFSET_USR_STATS_CB   // Post Processing (Used by PDSP0 only)  
//                  |        512 entries of size 2 bytes       |      
//                  |                                          |      c16
//     0xFFF8:0400  |  Roll over Time Accumulation Constants   |      OFFSET_RO_TIME_ACC_CONSTANTS    // Ingress0 PDSP0
//                  |        12 bytes                          |        
//                  |  Ethernet OAM Exception Table            |      OFFSET_EOAM_EXCEPTION_TBL       // Ingress0 PDSP1
//                  |        16 bytes                          |     
//                  |                                          |      c16
//     0xFFF8:0480  |  ACL rescore Header                      |      OFFSET_RESCORE_HDR              // Ingress0 PDSP1, Ingress 3, PDSP0
//                  |        4 bytes                           |
//                  |                                          |      c17
//     0xFFF8:0800  |  ACL rescore data                        |      OFFSET_RESCORE_DATA             // Ingress0 PDSP1, Ingress 3, PDSP0
//                  |      1024 bytes                          |         
//                  |                                          |
//                  |                                          |      c16
//     0xFFF8:0800  |  User-defined Statistics Counter blocks  |      OFFSET_USR_STATS_64B_COUNTERS // replaced by the statistics engine
//                  |        256 entries of size 8 bytes       |      
//                  |                                          |
//                  |  User-defined Statistics Counter blocks  |      OFFSET_USR_STATS_32B_COUNTERS // replaced by the statistics engine
//                  |        512 entries of size 4 bytes       |      
//                  |                                          |
//                  |  Combination of 64B and 32B counters =   |
//                  |    2048 bytes                            |
//                  |                                          |
//     0xFFF8:1000  |  Command Set table (4K)                  |      c17-  pdsp0   CMDSET_TABLE    // Psot Processing (Used by PDSP0 only)
//                  |     64 Entries of  64 bytes  or          |                                   
//                  |     32 Entries of 128 bytes              |                                   
//                  |                                          |
//     0xFFF8:0400  |  Outer IP traffic Flows                  |      c16 - pdsp1  // Ingress1, PDSP0
//                  |        32 entries of size 16 bytes       |                                          // Only one
//                  |                                          |
//     0xFFF8:0400  |  Inner IP traffic Flows                  |      c16 - pdsp2  // Ingress4, PDSP0
//                  |        32 entries of size 16 bytes       |      
//                  |                                          |
//     0xFFF8:0800  |  Outer IP Reassembly Control Block       |      c17 - pdsp1  // Ingress1, PDSP0
//                  |        16 bytes                          |      
//                  |                                          |
//     0xFFF8:0800  |  Inner IP Reassembly Control Block       |      c17 - pdsp2  // Ingress4, PDSP0
//                  |        16 bytes                          |      
//                  |                                          |
//     0xFFF8:0a00  |  Scrtach Memory for temporary storage    |      OFFSET_TEMP_BUFFER1   // To be determined
//     0xFFF8:0b00  |                                          |      OFFSET_TEMP_BUFFER2
//                  |                                          |
//     0xFFF8:0c00  |  IP protocol table - 256 bytes           |      c18   psdp0   IP_PROTO   // Classifyer 1 only
//     0xFFF8:0d00  |  IP protocol table - 256 bytes           |      c18   psdp1   IP_PROTO
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
//                  |  Raw Timestamp Acc       12 bytes        |                                  OFFSET_RAW_TIME_ACC        = 0xD0
//                  |  System Timestamp Tmp     8 bytes        |                                  OFFSET_TIMESTAMP_TMP       = 0xE0
//                  |  System Timestamp         8 bytes        |                                  OFFSET_SYSTEM_TIMESTAMP    = 0xF0
//                  |  IPv6 Fragmentation ID    4 bytes        |                                  OFFSET_IPV6_FRAG_ID        = 0xF8
//                  |                                          |
//     0xFF02:0200  |  Configurable Exception Routing          |      c20 - all pdsps  EROUTE     // Global Memory 
//                  |     32 entries of size 16 bytes          |
//                  |                                          |
//     0xFF02:0500  |  Egress Packet Capture Config   80 bytes |      c28 - all pdsps              OFFSET_EGRESS_PKT_CAP_CFG
//                  |    10 entries of size 8 bytes            |
//                  |  Ingress Packet Capture Config  80 bytes |                                   OFFSET_INGRESS_PKT_CAP_CFG
//                  |    10 entries of size 8 bytes            |
//                  |                                          | 
//     0xFF02:05FC  |  EQoS Default Priority           4 bytes |                                   OFFSET_EQOS_CFG_EG_DEF_PRI
//                  |                                          | 
//     0xFF02:0600  |  Port Default Route Config     512 bytes |                                   OFFSET_DEFAULT_ROUTE_CFG_PORTS
//                  |    8 entries of size 64 bytes            |
//                  |                                          | 
//     0xFF02:0800  |  Port EQoS Config             1024 bytes |                                   OFFSET_EQOS_CFG_BASE
//                  |    8 entries of size 256 bytes           |
//                  |                                          |
//     0xFFF8:1000  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp0      
//                  |  Reserve room for 32 entries             |                                  
//                  |  Ethertypes table 8 entries of 2 bytes   |                                  OFFSET_ETYPE_TABLE
//                  |  Reserve room for 16 entries             |
//                  |                                          |
//     0xFFF8:1100  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp1      
//                  |  Reserve room for 32 entries             |                                  
//                  |  Ethertypes table 8 entries of 2 bytes   |                                  OFFSET_ETYPE_TABLE
//                  |  Reserve room for 16 entries             |
//                  |                                          |
//     0xFFF8:1200  |  Parse Call Table  24 entries of 2 bytes |      c21 - pdsp2      
//                  |  Reserved for PDSP2/PDSP3                |
//                  |                                          |
//     0xFFF8:1400  |  NextFail Route Global Address Table     |
//                  |  8 entries of 4 bytes                    |      c24 - all pdsps  NextFail Route // Global Memory & Local Copy      
//                  |  Reserve room for 32 entries             |
//                  |                                          |
//     0xFFF8:2000  |  Mutliple Routing table                  |      c18 - all pdsps  MULTI_ROUTE  // Post Processing, PDSP1
//                  |     32 Entries of 64 bytes               |
//                  |                                          |
//     0xFFF8:2800  |  CRC VERIFY FIFO CB - 8 bytes            |      c21 - Post Processing
//                  |  CRC VERIFY FIFO 4 * 8  bytes            |      OFFSET_PDSP_CRC_VERIFY_FIFO
//                  |                                          |
//     0xFFF8:2900  |  SPLIT CXT FIFO CB - 8 bytes             |      c22 - Post Processing
//                  |  SPLIT CXT FIFO 4 * 8  bytes             |      OFFSET_PDSP_SPLIT_CXT_FIFO
//                  |                                          |
//     0xFFF8:1800  |  Custom Classify2 info 16 bytes          |      c16 -  pdsp1                 OFFSET_CUSTOM_C2   // Ingress 4 only
//                  |      16 entries of 16 byte               |      
//     0xFFF8:1000  |  Modify PDSP 0 checksum cmd 2 entries    |      c17  - pdsp 4    MODIFY      OFFSET_INSERT_CHKSUM  // Egress only  
//                  |      of 16 bytes                         |
//     0xFFF8:1030  |  Modify PDSP 0 Insert Time Scratch for   |                                   OFFSET_INS_EOAMTIME
//                  |      EOAM, one entry of 8 bytes          |
//     0xFFF8:1038  |  Modify PDSP 0 Insert Count for EOAM cmd |                                   OFFSET_INS_COUNT
//                  |      one entry of 4 bytes                |
//     0xFFF8:103c  |  Modify PDSP 0 Insert Count Scratch for  |                                   OFFSET_INS_EOAMCOUNT
//                  |      EOAM, one entry of 4 bytes          |
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
//                  |  (Single-core cluster)                   |
//     0xFFF8:1f00  |  PDSP ID, VER core 0 - 8 bytes           |      c23 - pdsp 0     PDSP_INFO  
//     0xFFF8:1f20  |  PDSP ID, VER core 1 - 8 bytes           |      c23 - pdsp 1     PDSP_INFO
//     0xFFF8:1f40  |  PDSP ID, VER core 2 - 8 bytes           |      c23 - pdsp 2     PDSP_INFO
//     0xFFF8:1f60  |  PDSP ID, VER core 3 - 8 bytes           |      c23 - pdsp 3     PDSP_INFO
//                  |                                          |
//                  |  (Double-core cluster)                   |
//     0xFFF8:3f00  |  PDSP ID, VER core 0 - 8 bytes           |      c23 - pdsp 0     PDSP_INFO  
//     0xFFF8:3f20  |  PDSP ID, VER core 1 - 8 bytes           |      c23 - pdsp 1     PDSP_INFO
//     0xFFF8:3f40  |  PDSP ID, VER core 2 - 8 bytes           |      c23 - pdsp 2     PDSP_INFO
//     0xFFF8:3f60  |  PDSP ID, VER core 3 - 8 bytes           |      c23 - pdsp 3     PDSP_INFO
//                  |                                          |
//     0xFFF8:2000  |  PDSP0 LUT1 associated table memory      |      c0 - pdsp0      PDSP_LUT1_INFO
//                  |     256 entries of size 64 bytes         |      
//                  |     (Single PDSP cluster)                |
//     0xFFF8:4000  |  PDSP0 LUT1 associated table memory      |      c0 - pdsp0      PDSP_LUT1_INFO
//                  |     256 entries of size 64 bytes         |      
//                  |     (Double PDSP cluster)                |
//     0xFFF8:8000  |  PDSP1 LUT1 associated table memory      |      c0 - pdsp1      PDSP_LUT1_INFO
//                  |     256 entries of size 64 bytes         |      
//                  |                                          |
//                  --------------------------------------------

//-----------------------------------------------------
//
// Constants
//
// Classifier  LUT1
//#define PAMEM_CONST_PDSP_LUT1_INFO      c0 
//#define PAMEM_CONST_PDSP_EOAM_TS_CTX    c12
//#define PAMEM_CONST_PDSP_EOAM_STATS_CFG c13
//#define PAMEM_CONST_PDSP_CXT            c14 
//#define PAMEM_CONST_RESCORE_HDR         c16
//#define PAMEM_CONST_RESCORE_DATA        c17
//#define PAMEM_CONST_EOAM                c16
//#define PAMEM_CONST_RO_TIME_ACC_TABLE   c16
//#define PAMEM_CONST_IP_REASSEM_CONTEXT  c17
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_NFAIL_BASE          c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25
//
// Classifier  LUT2
//#define PAMEM_CONST_CUSTOM2             c16
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PARSE               c21
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_NFAIL_BASE          c24
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25

// Post Processing PDSP0
//#define PAMEM_CONST_USR_STATS_FIFO_CB   c0
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_CMDSET_TABLE        c17
//#define PAMEM_CONST_MULTI_ROUTE         c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_CRC_VDERIFY_FIFO_CB c21
//#define PAMEM_CONST_SPLIT_CXT_CB        c22
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25

// Post Processing PDSP1
//#define PAMEM_CONST_USR_STATS_FIFO_CB   c0
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_CMDSET_TABLE        c17
//#define PAMEM_CONST_MULTI_ROUTE         c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_CRC_VDERIFY_FIFO_CB c21
//#define PAMEM_CONST_SPLIT_CXT_CB        c22
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25

// Egress 0, PDSP 0
//#define PAMEM_CONST_PDSP_LUT1_INFO      c0    
//#define PAMEM_CONST_PDSP_CXT            c14    
//#define PAMEM_CONST_USR_STATS_CB        c15
//#define PAMEM_CONST_USR_STATS_COUNTERS  c16
//#define PAMEM_CONST_MODIFY              c17         
//#define PAMEM_CONST_IP_PROTO            c18
//#define PAMEM_CONST_CUSTOM              c19
//#define PAMEM_CONST_EROUTE              c20
//#define PAMEM_CONST_PDSP_INFO           c23
//#define PAMEM_CONST_USR_STATS_FIFO_BASE c25

// Egress PDSP1 and others

// PASS Scratch Memory Base 
#define PAMEM_CONST_LOCAL_SRAM_BASE     c1
#define PAMEM_CONST_GLOBAL_SRAM_BASE    c2

// PASS System Timer (PDSP0 Timer) 0xff208800
#define PAMEM_CONST_SYS_TIMER           c3

// Scratch Memory 0: 0xFFF80000  (Global local for Post-Processing)
// PASS User-defined Statistics  
#define PAMEM_CONST_USR_STATS_FIFO_CB       c0
#define OFFSET_PDSP_USR_STATS_FIFO_CB       0x00
#define OFFSET_PDSP_USR_STATS_FIFO          0x10
#define OFFSET_PDSP_USR_STATS_FIFO_CB_SIZE  0x40
#define OFFSET_PDSP0_USR_STATS_FIFO_CB      0x00
#define OFFSET_PDSP0_USR_STATS_FIFO         0x10
#define OFFSET_PDSP1_USR_STATS_FIFO_CB      0x40
#define OFFSET_PDSP1_USR_STATS_FIFO         0x50

// PASS User-defined Statistics   Post-Processing PDSP0 (0xFFF80400, 0xFFF0800)
#define PAMEM_CONST_USR_STATS_CB        c15
#define OFFSET_USR_STATS_CB             0x000     

#define PAMEM_CONST_USR_STATS_COUNTERS  c16

// PASS User-defined Statistics localtion (0xFFF80800)
#define PAMEM_USR_STATS_COUNTERS           0xFFF80800
#define PAMEM_USR_STATS_COUNTERS_LOC_GLOBAL           (0xFF980800)


// Local LUT1 info (0xFFF82000, 0xFFF84000, 0xFFF88000, 0xFFF8C000)
#define PAMEM_CONST_PDSP_LUT1_INFO      c0    

// PDSP Local context (0xFFF80000, 0xFFF80100, 0xFFF80200)
#define PAMEM_CONST_PDSP_CXT            c14    
#define PDSP_CXT_OFFSET_L1_MAP          0
#define PDSP_CXT_OFFSET_L1_MAP2         0x20   // 1 byte 
#define PDSP_CXT_OFFSET_L1_PENDING      0x24   // 20 bytes 
#define OFFSET_EXT_HDR_PENDING          0x40   // 20 bytes (store extended header to be sent out
                                               //           after the pending LUT1 entry)    
// IP Reassembly-assistance  Ingress 1/4: 0xFFF80400, 0xFFF80800
#define PAMEM_CONST_TF_TABLE            c16
#define PAMEM_CONST_IP_REASSEM_CONTEXT  c17

// Roll Over Time accumulation scratch pad for Ingress 0: 0xFFF80400
#define PAMEM_CONST_RO_TIME_ACC_INGRESS0   0xFF480400
#define PAMEM_CONST_RO_TIME_ACC_TABLE      c16
#define OFFSET_RO_TIME_ACC_CONSTANTS         0
#define OFFSET_NS_ROACC                      0
#define OFFSET_NS_NUM_ROACC                  4
#define OFFSET_NUMRO                         8
#define OFFSET_RO_COUNT                     10

// Ethernet OAM statistics count exception table Ingress 0: 0xFFF80400
#define PAMEM_CONST_EOAM_EXC_TABLE         c16
#define OFFSET_EOAM_EXCEPTION_TBL           12

// ACL Rescore Table Header (In0-PDSP1, In3-PDSP0): 0xFFF80480
#define PAMEM_CONST_RESCORE_HDR            c16
#define OFFSET_RESCORE_HDR                 128

// ACL Rescore Table Data (In0-PDSP1, In3-PDSP0): 0xFFF80800
#define PAMEM_CONST_RESCORE_DATA           c17
#define OFFSET_RESCORE_DATA                0

// Scratch memory 2 (0xFFF80A00)
// IP traffic flow and reassembly control blocks
//#define OFFSET_TEMP_BUFFER1             0xF00        // 0xFFF84000 based (temporary storage
//#define OFFSET_TEMP_BUFFER2             0x4F80       // 0xFFF80000 based (buffer copy)
                                    
// IP Protocol table Ingress Stage  (0xFFF80C00): Global configuration only
#define PAMEM_CONST_IP_PROTO            c18

// Custom LUT1 Configuration (0xFF020000): global scratch
#define PAMEM_CONST_CUSTOM              c19
#define CUSTOM_C1_SIZE                  40
#define OFFSET_CUSTOM_C1                0
#define OFFSET_MAX_HDR                  OFFSET_CUSTOM_C1 + (4*CUSTOM_C1_SIZE)
#define OFFSET_OUT_IP_REASSM_CFG        OFFSET_MAX_HDR + 4
#define OFFSET_IN_IP_REASSM_CFG         OFFSET_OUT_IP_REASSM_CFG + 4
#define OFFSET_CMDSET_CFG               OFFSET_IN_IP_REASSM_CFG + 4
#define OFFSET_USR_STATS_CFG            OFFSET_CMDSET_CFG + 4
#define OFFSET_QUEUE_DIVERT_CFG         OFFSET_USR_STATS_CFG + 4
#define OFFSET_IPSEC_NAT_T_CFG          OFFSET_QUEUE_DIVERT_CFG + 4
#define OFFSET_MAC_PADDING_CFG          OFFSET_IPSEC_NAT_T_CFG + 4
#define OFFSET_OUT_IP_ACL_CFG           OFFSET_MAC_PADDING_CFG + 4
#define OFFSET_IN_IP_ACL_CFG            OFFSET_OUT_IP_ACL_CFG + 4
#define OFFSET_QUEUE_BOUNCE_CFG         OFFSET_IN_IP_ACL_CFG + 4
#define OFFSET_RAW_TIME_ACC             0xD0
#define OFFSET_TIMESTAMP_TMP            0xE0
#define OFFSET_SYS_TIMESTAMP            0xF0
#define OFFSET_IPV6_FRAG_ID             0xF8

// Custom LUT2 Configuration (LUT2-PDSP 0xFFF81800)
#define PAMEM_CONST_CUSTOM2             c16
#define OFFSET_CUSTOM_C2                0

// Packet Excepotion routing (global scratch: 0xFF020200)
#define PAMEM_CONST_EROUTE              c20

// Packet Parse (LUT1-PDSP 0xFFF81000, 0xFFF81100, 0xFFF81200, ..)
#define PAMEM_CONST_PARSE               c21
#define OFFSET_ETYPE_TABLE              64

// Scratch Memory 0: 0xFFF80000  (Global local for Post-Processing)
// CRC Verify FIFO  0xFFF82800
#define PAMEM_CONST_CRC_VERIFY_FIFO_CB     c21
#define OFFSET_PDSP_CRC_VERIFY_FIFO        0x10

// Split Control FIFO 0xFFF82900 
#define PAMEM_CONST_SPLIT_CXT_FIFO_CB      c22
#define OFFSET_PDSP_SPLIT_CXT_FIFO         0x10

// NextFail Table global address (local: 0xFFF81400)
#define PAMEM_CONST_NFAIL_BASE          c24

#define PAMEM_NFAIL_BASE_ADDR_PDSP0     0xFF484014
#define PAMEM_NFAIL_BASE_ADDR_PDSP1     0xFF488014
#define PAMEM_NFAIL_BASE_ADDR_PDSP2     0xFF584014
#define PAMEM_NFAIL_BASE_ADDR_PDSP3     0xFF588014
#define PAMEM_NFAIL_BASE_ADDR_PDSP4     0xFF682014
#define PAMEM_NFAIL_BASE_ADDR_PDSP5     0xFF782014
#define PAMEM_NFAIL_BASE_ADDR_PDSP6     0xFF884014
#define PAMEM_NFAIL_BASE_ADDR_PDSP7     0xFF888014


// Command Set Table (Post-processing stage PDSP0: 0xFFF81000)
#define PAMEM_CONST_CMDSET_TABLE        c17
#define PAMEM_BASE_CMDSET_TABLE         0xFFF81000

// Multiple Destination Packet Routing (Post-processing stage PDSP1: 0xFFF82000)
#define PAMEM_CONST_MULTI_ROUTE         c18
#define PAMEM_BASE_MULTI_ROUTE          0xFFF82000

// PDSP Info: 0xFFF81f00
#define PAMEM_CONST_PDSP_ALL_INFO       c22
#define PAMEM_CONST_PDSP_INFO           c23
#define OFFSET_ID                       0
#define OFFSET_VER                      4

#define OFFSET_ID_PDSP0                 0x00
#define OFFSET_ID_PDSP1                 0x20
#define OFFSET_ID_PDSP2                 0x40

// PDSP USR STATS FIFO & CB per PDSP (local global: 0xFF980000, 0xFF980040, 0xFF980080, ....)
#define PAMEM_CONST_USR_STATS_FIFO_BASE c25

// Interface-based EMAC port configurations (0xFF020500)
#define PAMEM_CONST_PORTCFG             c28 // all pdsps
#define PACKET_CAP_CFG_SIZE             8
#define DEFAULT_ROUTE_CFG_SIZE          64
#define DEFAULT_ROUTE_CTRL_SIZE         4
#define EQOS_CFG_SIZE                   256

#define OFFSET_EGRESS_PKT_CAP_CFG_BASE  0
#define OFFSET_INGRESS_PKT_CAP_CFG_BASE OFFSET_EGRESS_PKT_CAP_CFG_BASE + 10*PACKET_CAP_CFG_SIZE

#define OFFSET_DEFAULT_ROUTE_CFG_BASE   OFFSET_EGRESS_PKT_CAP_CFG_BASE + 0x100

//  egress global default priority stored after all port configuration is done
#define OFFSET_EQOS_CFG_BASE            OFFSET_DEFAULT_ROUTE_CFG_BASE + 8*DEFAULT_ROUTE_CFG_SIZE
#define OFFSET_EQOS_CFG_EG_DEF_PRI      OFFSET_DEFAULT_ROUTE_CFG_BASE - 4 

// Modify PDSP Egress0, PDSP0 (0xFFF81000)
#define PAMEM_CONST_MODIFY_EGRESS0      0xFFA81000
#define PAMEM_CONST_MODIFY              c17         
#define OFFSET_INSERT_CHKSUM            0
#define OFFSET_INS_EOAMTIME             0x30
#define OFFSET_INS_COUNT                0x38
#define OFFSET_INS_EOAMCOUNT            0x3c
#define OFFSET_BLIND_PATCH              0x40
#define OFFSET_INSERT_CRC               0xc0
#define OFFSET_CONVERT_TIMESTAMP        0xd0
#define OFFSET_REPORT_TIMESTAMP         0xe0
#define OFFSET_IP_FRAG                  0xf0
#define OFFSET_PATCH_MSG_LEN            0xf8
// Location of the first blind patch command
// It will be the only one supported with IP fragmentation patch
#define PAMEM_BASE_BLIND_PATCH          0xFFF81040

// Constants as seen by the system
#define FIRMWARE_P0_LUT1_INFO_BASE      0xFFF84000     // c0
#define FIRMWARE_P0_LUT1_CONTEXT        0xFFF80000     // c15
#define FIRMWARE_P0_ID_VER              0xFFF81f00     // c23


#define FIRMWARE_P1_LUT1_INFO_BASE      0xFFF88000     // c0
#define FIRMWARE_P1_LUT1_CONTEXT        0xFFF80100     // c15
#define FIRMWARE_P1_ID_VER              0xFFF81f20     // c23

#define FIRMWARE_P2_LUT1_INFO_BASE      0xFFF8C000     // c0
#define FIRMWARE_P2_LUT1_CONTEXT        0xFFF80200     // c15
#define FIRMWARE_P2_ID_VER              0xFFF81f40     // c23

//#define FIRMWARE_P4_MODIFY              0xFFF87000     // c25
//#define FIRMWARE_P4_ID_VER              0xFFF87f80     // c23

//#define FIRMWARE_P5_MODIFY              0xFFF87100     // c25
//#define FIRMWARE_P5_ID_VER              0xFFF87fa0     // c23

// Mailbox bases
#define FIRMWARE_P0_MBOX                0x0000      // c4
#define FIRMWARE_P1_MBOX                0x0010      // c4
#define FIRMWARE_P2_MBOX                0x0020      // c4
#define FIRMWARE_P3_MBOX                0x0030      // c4
#define FIRMWARE_P4_MBOX                0x0040      // c4
#define FIRMWARE_P5_MBOX                0x0050      // c4
#define FIRMWARE_P6_MBOX                0x0060      // c4
#define FIRMWARE_P7_MBOX                0x0070      // c4
#define FIRMWARE_P8_MBOX                0x0080      // c4
#define FIRMWARE_P9_MBOX                0x0090      // c4
#define FIRMWARE_P10_MBOX               0x00A0      // c4
#define FIRMWARE_P11_MBOX               0x00B0      // c4
#define FIRMWARE_P12_MBOX               0x00C0      // c4
#define FIRMWARE_P13_MBOX               0x00D0      // c4
#define FIRMWARE_P14_MBOX               0x00E0      // c4
#define FIRMWARE_P15_MBOX               0x00F0      // c4

#define FIRMWARE_OUTER_IP_PDSP_MBOX     FIRMWARE_P2_MBOX
#define FIRMWARE_INNER_IP_PDSP_MBOX     FIRMWARE_P6_MBOX
#define FIRMWARE_L2_PDSP_MBOX           FIRMWARE_P0_MBOX
#define FIRMWARE_OUTER_RA_PDSP_MBOX     FIRMWARE_P1_MBOX
#define FIRMWARE_INNER_RA_PDSP_MBOX     FIRMWARE_P5_MBOX
#define FIRMWARE_TXCMD_PDSP_MBOX        FIRMWARE_P11_MBOX


//#define cMailbox                        c4    (Use official name: search and replace)
//#define PDSP_TIMER                      c9


#endif // _PA_MEM_H

