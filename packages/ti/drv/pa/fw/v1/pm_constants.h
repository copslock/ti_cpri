//============================================================================
// pm_constants.h
//
// This file constains modeling constants that are independent of
// platform configuration.
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

#ifndef _PM_CONSTANTS_H
#define _PM_CONSTANTS_H  1

#define XFR_DELAY_SCALE     1
#define SA_CONTEXT_SIZE     560  //from 480

//=============================================================================
//
// CDE Equates
//

// Commands and Flags
#define CDE_CMD_ADVANCE                     0x00
#define CDE_CMD_FLUSH                       0x01
#define CDE_CMD_ADVANCE_SCOPY               0x02
#define CDE_CMD_FLUSH_SCOPY                 0x03
#define CDE_CMD_ADVANCE_TO_PSDATA           0x04
#define CDE_CMD_FLUSH_TO_PSDATA             0x05
#define CDE_CMD_ADVANCE_TO_PSDATA_SCOPY     0x06
#define CDE_CMD_FLUSH_TO_PSDATA_SCOPY       0x07
#define CDE_CMD_ADVANCE_TO_CONTROL          0x08
#define CDE_CMD_FLUSH_TO_CONTROL            0x09
#define CDE_CMD_ADVANCE_TO_CONTROL_SCOPY    0x0A
#define CDE_CMD_FLUSH_TO_CONTROL_SCOPY      0x0B
#define CDE_CMD_ADVANCE_TO_PACKET           0x0C
#define CDE_CMD_FLUSH_TO_PACKET             0x0D
#define CDE_CMD_ADVANCE_TO_PACKET_SCOPY     0x0E
#define CDE_CMD_FLUSH_TO_PACKET_SCOPY       0x0F
#define CDE_CMD_ADVANCE_TO_END              0x10
#define CDE_CMD_FLUSH_TO_END                0x11
#define CDE_CMD_ADVANCE_TO_END_SCOPY        0x12
#define CDE_CMD_FLUSH_TO_END_SCOPY          0x13
#define CDE_CMD_WINDOW_ADVANCE              (CDE_CMD_ADVANCE)
#define CDE_CMD_WINDOW_FLUSH                (CDE_CMD_FLUSH)
#define CDE_CMD_PACKET_ADVANCE              0x20
#define CDE_CMD_PACKET_FLUSH                0x21
#define CDE_CMD_PACKET_COPY                 0x22
#define CDE_FLG_IGNORE_CHKSUM               (1<<0)
#define CDE_FLG_RETAIN_CHKSUM               (1<<1)
#define CDE_FLG_TRUNCATE                    (1<<2)
#define CDE_FLG_HOLD_PACKET                 (1<<3)
#define CDE_FLG_SET_THREADID                (1<<4)
#define CDE_FLG_SET_FLOWID                  (1<<5)
#define CDE_FLG_SET_PSINFO                  (1<<6)
#define CDE_FLG_SET_DESTQUEUE               (1<<7)
#define CDE_CMD_HPKT_RELEASE                0x23
#define CDE_CMD_HPKT_DISCARD                0x24
#define CDE_CMD_INSERT                      0x30
#define CDE_CMD_INSERT_PSDATA               0x31
#define CDE_CMD_INSERT_CONTROL              0x32
#define CDE_CMD_INSERT_PACKET               0x33
#define CDE_CMD_INSERT_BUFFER               0x34
#define CDE_CMD_INSERT_PSDATA_BUFFER        0x35
#define CDE_CMD_INSERT_CONTROL_BUFFER       0x36
#define CDE_CMD_INSERT_PACKET_BUFFER        0x37
#define CDE_CMD_PATCH                       0x40
#define CDE_CMD_PATCH_PSDATA                0x41
#define CDE_CMD_PATCH_CONTROL               0x42
#define CDE_CMD_PATCH_PACKET                0x43
#define CDE_CMD_PATCH_BUFFER                0x44
#define CDE_CMD_PATCH_PSDATA_BUFFER         0x45
#define CDE_CMD_PATCH_CONTROL_BUFFER        0x46
#define CDE_CMD_PATCH_PACKET_BUFFER         0x47
#define CDE_CMD_CHECKSUM1_VALIDATE          0x50
#define CDE_CMD_CHECKSUM1_COMPUTE           0x51
#define CDE_CMD_CHECKSUM2_VALIDATE          0x52
#define CDE_CMD_CHECKSUM2_COMPUTE           0x53
#define CDE_FLG_AVOIDZERO                   (1<<0)
#define CDE_CMD_CRC_VALIDATE                0x60
#define CDE_CMD_CRC_COMPUTE                 0x61
#define CDE_FLG_CRCOFFSETVALID              (1<<0)
#define CDE_CMD_CRC_TABLE_ENTRY             0x62
#define CDE_CMD_CRC_CONFIG                  0x63
#define CDE_FLG_CRCCFG_8BIT                 (0<<0)
#define CDE_FLG_CRCCFG_16BIT                (1<<0)
#define CDE_FLG_CRCCFG_24BIT                (2<<0)
#define CDE_FLG_CRCCFG_32BIT                (3<<0)
#define CDE_FLG_CRCCFG_LSBCRC               (1<<2)
#define CDE_FLG_CRCCFG_NOT                  (1<<3)

//=============================================================================
//
// SCV Equates
//
#define SCV_CMD_POSITION_WINDOW             0x01
#define SCV_CMD_DONE                        0x02


//=============================================================================
//
// BlockMgr Equates
//

// Register Offsets
#define BLOCKMGR_REG_BLOCKID1               0x00
#define BLOCKMGR_REG_BLOCKID2               0x08
#define BLOCKMGR_REG_THREADID1              0x10
#define BLOCKMGR_REG_THREADID2              0x18


//=============================================================================
//
// LUT1 Equates
//

// LUT1 Commands
#define LUT1_CMD_INSERT                     0x01
#define LUT1_CMD_INSERTM                    0x02
#define LUT1_CMD_REMOVE                     0x03
#define LUT1_CMD_REMOVEM                    0x04
#define LUT1_CMD_SEARCH                     0x05

// LUT1 Option Flags
#define LUT1_OPTFLG_RSS_ON                  (1<<5)
#define LUT1_OPTFLG_RSS_USEPORTS            (1<<6)
#define LUT1_OPTFLG_RSS_IPV6                (1<<7)

// Register Offsets
#define LUT1_REG_REVISION                   0x00
#define LUT1_REG_CONTROL                    0x04
#define LUT1_REG_CONFIG                     0x08
#define LUT1_REG_DEBUG                      0x10
#define LUT1_REG_DEBUG_ENTRY0               0x40
#define LUT1_REG_DEBUG_ENTRY1               0x44
#define LUT1_REG_DEBUG_ENTRY2               0x48
#define LUT1_REG_DEBUG_ENTRY3               0x4C
#define LUT1_REG_DEBUG_ENTRY4               0x50
#define LUT1_REG_DEBUG_ENTRY5               0x54
#define LUT1_REG_DEBUG_ENTRY6               0x58
#define LUT1_REG_DEBUG_ENTRY7               0x5C
#define LUT1_REG_DEBUG_ENTRY8               0x60
#define LUT1_REG_DEBUG_ENTRY9               0x64
#define LUT1_REG_DEBUG_ENTRYA               0x68
#define LUT1_REG_DEBUG_ENTRYB               0x6C
#define LUT1_REG_DEBUG_ENTRYC               0x70
#define LUT1_REG_DEBUG_ENTRYD               0x74
#define LUT1_REG_DEBUG_ENTRYE               0x78
#define LUT1_REG_DEBUG_ENTRYF               0x7C
#define LUT1_REG_RSSHASH0                   0xC4
#define LUT1_REG_RSSHASH1                   0xC8
#define LUT1_REG_RSSHASH2                   0xCC
#define LUT1_REG_RSSHASH3                   0xD0
#define LUT1_REG_RSSHASH4                   0xD4
#define LUT1_REG_RSSHASH5                   0xD8
#define LUT1_REG_RSSHASH6                   0xDC
#define LUT1_REG_RSSHASH7                   0xE0
#define LUT1_REG_RSSHASH8                   0xE4
#define LUT1_REG_RSSHASH9                   0xE8

//=============================================================================
//
// LUT2 Equates
//

// Register Offsets
#define LUT2_REG_REVISION                   0x00
#define LUT2_REG_SOFTRESET                  0x04
#define LUT2_REG_DATA0                      0x20
#define LUT2_REG_DATA1                      0x24
#define LUT2_REG_DATA2                      0x28
#define LUT2_REG_DATA3                      0x2C
#define LUT2_REG_ADDDEL_KEY0                0x30
#define LUT2_REG_ADDDEL_KEY1                0x34
#define LUT2_REG_ADDDEL_CTRL                0x38
#define LUT2_REG_DEBUG_MODE                 0x60
#define LUT2_REG_DEBUG_CTRL                 0x64
#define LUT2_REG_DEBUG_KEY0                 0x68
#define LUT2_REG_DEBUG_KEY1                 0x6C
#define LUT2_REG_DEBUG_DATA0                0x70
#define LUT2_REG_DEBUG_DATA1                0x74
#define LUT2_REG_DEBUG_DATA2                0x78
#define LUT2_REG_DEBUG_DATA3                0x7C


//=============================================================================
//
// Splitter Equates
//

// Register Offsets
#define SPLIT_REG_REVISION                  0x00
#define SPLIT_REG_SOPCTRL                   0x10
#define SPLIT_REG_EOPCTRL                   0x14
#define SPLIT_REG_MOPBUFSIZE                0x20
#define SPLIT_REG_MOPBUFPTR                 0x24


//=============================================================================
//
// Stats Bloc Equates
//

// Register Offsets
#define STATSBLOC_REG_REVISION              0x00
#define STATSBLOC_REG_RESET                 0x04
#define STATSBLOC_REG_CONTROL               0x08
#define STATSBLOC_REG_COUNTER_UPDATE        0x0C
#define STATSBLOC_REG_TIMER_CONTROL         0x10
#define STATSBLOC_REG_TIMER_LOAD            0x14
#define STATSBLOC_REG_TIMER_VALUE           0x18
#define STATSBLOC_REG_PACKET_ROUTING        0x1C


//=============================================================================
//
// Timer Equates
//

// Register Offsets
#define TIMER_REG_CTRL                      0x00
#define TIMER_REG_LOAD                      0x04
#define TIMER_REG_COUNT                     0x08
#define TIMER_REG_EVENT                     0x0c


//=============================================================================
//
// PacketID Equates
//

// Register Offsets
#define PACKETID_REG_REVISION               0x00
#define PACKETID_REG_SOFTRESET              0x04
#define PACKETID_REG_RANGELIMIT             0x08
#define PACKETID_REG_IDVALUE                0x0C


//=============================================================================
//
// RA Equates
//

// Register Offsets
#define RA15_REG_ID                         0x00
#define RA15_REG_CONFIG                     0x04
#define RA_REG_ID                           0x00
#define RA_REG_CONFIG                       0x04
#define RA_REG_CONTEXTS                     0x08
#define RA_REG_DISCARD_THRESH               0x0C
#define RA_REG_CONTEXT_TIMEOUT              0x10
#define RA_REG_TICK_VALUE                   0x14
#define RA_REG_VBUSM_CONFIG                 0x18
#define RA_REG_REGION_THRESH                0x1C
#define RA_REG_REGION0_BASE_LOW             0x20
#define RA_REG_REGION0_BASE_HIGH            0x24
#define RA_REG_REGION1_BASE_LOW             0x28
#define RA_REG_REGION1_BASE_HIGH            0x2C
#define RA_REG_GRP0_TIMEOUT_FLOW            0x40
#define RA_REG_GRP0_CRITERROR_FLOW          0x44
#define RA_REG_GRP0_ERROR_FLOW              0x48
#define RA_REG_GRP1_TIMEOUT_FLOW            0x50
#define RA_REG_GRP1_CRITERROR_FLOW          0x54
#define RA_REG_GRP1_ERROR_FLOW              0x58

#define RA_REG_STAT_FORCED_TIMEOUT          0x90

#define RA_REG_STAT_BASE                    0xA0

// Stat Reg Address = Base + (StatIndex * 4) + (GroupIndex * 0x60)
#define RA_STAT_REASM_PKT                   0
#define RA_STAT_FRAGS                       1
#define RA_STAT_IN_PKT                      2
#define RA_STAT_TIMEOUT_SOP_PKT             3
#define RA_STAT_TIMEOUT_SOP_BYTES           4
#define RA_STAT_TIMEOUT_PKT                 5
#define RA_STAT_TIMEOUT_BYTES               6
#define RA_STAT_CTX_REJECT_PKT              7
#define RA_STAT_CTX_REJECT_BYTES            8
#define RA_STAT_OVERLAP_PKT                 9
#define RA_STAT_OVERLAP_BYTES               10
#define RA_STAT_LARGE_PKT                   11
#define RA_STAT_TCP_ERROR_PKT               12
#define RA_STAT_FRAG_ERROR_PKT              13
#define RA_STAT_IPV4IHL_ERROR_PKT           14
#define RA_STAT_SMALL_PKT                   15
#define RA_STAT_FRAGLEN_PKT                 16
#define RA_STAT_ARLEADY_PKT                 17
#define RA_STAT_ARLEADY_BYTES               18


//=============================================================================
//
// RNG Equates
//

// Register Offsets
#define RNG_REG_OUT_0                       0x00
#define RNG_REG_OUT_1                       0x04
#define RNG_REG_STAT                        0x08
#define RNG_REG_INTMASK                     0x0C
#define RNG_REG_INTACK                      0x10
#define RNG_REG_CTRL                        0x14
#define RNG_REG_CONFIG                      0x18
#define RNG_REG_ALARMCNT                    0x1C
#define RNG_REG_FROEN                       0x20
#define RNG_REG_FRODETUNE                   0x24
#define RNG_REG_ALARMMASK                   0x28
#define RNG_REG_ALARMSTOP                   0x2C
#define RNG_REG_LFSR_L                      0x30
#define RNG_REG_LFSR_M                      0x34
#define RNG_REG_LFSR_H                      0x38
#define RNG_REG_COUNT                       0x3C
#define RNG_REG_TEST                        0x40
#define RNG_REG_SYSSTATUS                   0x44


//-----------------------------------------------------
//
// SA Engine Ids
//
#define SAENG_ENCRYPT_1                     2
#define SAENG_ENCRYPT_2                     3
#define SAENG_AUTH_1                        4
#define SAENG_AUTH_2                        5
#define SAENG_PHP1_1                        8
#define SAENG_PHP1_2                        9
#define SAENG_PHP1_3                        10
#define SAENG_OUT1_1                        12
#define SAENG_AIRC_1                        14
#define SAENG_AIRC_2                        15
#define SAENG_PHP2_1                        16
#define SAENG_PHP2_2                        17
#define SAENG_PHP2_3                        18
#define SAENG_OUT2_1                        20

//-----------------------------------------------------
//
// System trace level identifiers
//
#define SYSTRACE_LUT1_TRACE      1   // Trace of LUT1 events
#define SYSTRACE_LUT2_TRACE      2   // Trace of LUT2 events
#define SYSTRACE_CDE_TRACE       3   // Trace of CDE events
#define SYSTRACE_PHP_TRACE       4   // Trace of PHP events


//===========================================================
//
// Packet Statistics
//
#define NETCP_IN_PKT                0x8000
#define NETCP_IN_BYTES              0xC001

#define L2_IN_ERROR_PKT             0x8016
#define L2_IN_ERROR_BYTES           0xc017
#define L2_IN_BCAST_PKT             0x8018
#define L2_IN_BCAST_BYTES           0xc019
#define L2_IN_MCAST_PKT             0x8020
#define L2_IN_MCAST_BYTES           0xc021
#define L2_IN_VLAN_PKT              0x8022
#define L2_IN_QINQ_PKT              0x8023
#define L2_IN_SNAP_PKT              0x8024
#define L2_IN_PPPOE_NONIP_PKT       0x8025
#define L2_IN_PPPOE_IP_PKT          0x8026

#define LUT_0_0_SUBMIT_PKT          0x8032
#define LUT_0_0_MATCH_PKT           0x8033
#define LUT_0_1_SUBMIT_PKT          0x8034
#define LUT_0_1_MATCH_PKT           0x8035

#define IP4_IN_PKT                  0x8048      // IP packets
#define IP4_IN_BYTES                0xc049      // IP data bytes (including header)
#define IP4_IN_MCAST_PKT            0x8050      // IP multicast packets
#define IP4_IN_MCAST_BYTES          0xc051      // IP multicast data bytes (including header)
#define IP4_IN_BCAST_PKT            0x8052      // IP broadcast packets
#define IP4_IN_BCAST_BYTES          0xc053      // IP broadcast data bytes (including header)
#define IP4_IN_HDR_ERR_PKT          0x8054      // Bad IP header
#define IP4_IN_ADDR_ERR_PKT         0x8055      // Bad IP address
#define IP4_IN_TRUNC_ERR_PKT        0x8056      // Packet truncation (size error)
#define IP4_IN_FRAG_PKT             0x8057

#define TCP_IN_PKT                  0x8064      // TCP packets
#define TCP_ERROR_PKT               0x8065
#define TCP_CTRL_PKT                0x8066      // Control Packets (SYN,FIN,RST)
#define TCP_ACK_PKT                 0x8067      // Ack Packet (with no data)

#define UDP_IN_PKT                  0x8080      // UDP packets
#define UDP_ERROR_PKT               0x8081

#endif



