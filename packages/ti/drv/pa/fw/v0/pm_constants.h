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


//=============================================================================
//
// LUT1 Equates
//

// LUT1 Commands
#define LUT1_CMD_INSERT                     0x01
#define LUT1_CMD_REMOVE                     0x02
#define LUT1_CMD_SEARCH                     0x03
#define LUT1_CMD_SEARCHNS                   0x04

// LUT1 Flags
#define LUT1_FLG_DSTMAC                     (1<<0)
#define LUT1_FLG_SRCMAC                     (1<<1)
#define LUT1_FLG_ETHERTYPE                  (1<<2)
#define LUT1_FLG_VLAN                       (1<<3)
#define LUT1_FLG_SRCIP                      (1<<4)
#define LUT1_FLG_DSTIP                      (1<<5)
#define LUT1_FLG_SPI                        (1<<6)
#define LUT1_FLG_FLOWLABEL                  (1<<7)
#define LUT1_FLG_SRCPORT                    (1<<8)
#define LUT1_FLG_DSTPORT                    (1<<9)
#define LUT1_FLG_PROTOCOL                   (1<<10)
#define LUT1_FLG_TOS                        (1<<11)
#define LUT1_FLG_INGRESSPORT                (1<<12)
#define LUT1_FLG_KEYBYTE                    (1<<13)
#define LUT1_FLG_VALID                      (1<<15)


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
#define LUT2_REG_ADDDEL_KEY                 0x30
#define LUT2_REG_ADDDEL_CTRL                0x34


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


#endif



