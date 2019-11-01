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
#ifndef _PM_CONFIG_H
#define _PM_CONFIG_H  1

#ifdef KEYSTONE2
// PDSP IRAM 1K instructions = 4K bytes
#define PDSP_IRAM_SIZE              1024

// Platform Report Commands
#define PRCMD_BASE                  0x00100000
#define PRCMD_SIZE                  0x00000004

// PDSP
#define PDSP1_BASE                  0x0000F000
#define PDSP1_DEBUG                 0x0000F800
#define PDSP1_TIMER                 0x0000E000
#define PDSP1_IRAM                  0x00010000

#define PDSP2_BASE                  0x0000F100
#define PDSP2_DEBUG                 0x0000F900
#define PDSP2_TIMER                 0x0000E100
#define PDSP2_IRAM                  0x00011000

#define PDSP3_BASE                  0x0000F200
#define PDSP3_DEBUG                 0x0000FA00
#define PDSP3_TIMER                 0x0000E200
#define PDSP3_IRAM                  0x00012000

#define PDSP4_BASE                  0x0000F300
#define PDSP4_DEBUG                 0x0000FB00
#define PDSP4_TIMER                 0x0000E300
#define PDSP4_IRAM                  0x00013000

#define PDSP5_BASE                  0x0000F400
#define PDSP5_DEBUG                 0x0000FC00
#define PDSP5_TIMER                 0x0000E400
#define PDSP5_IRAM                  0x00014000

#define PDSP6_BASE                  0x0000F500
#define PDSP6_DEBUG                 0x0000FD00
#define PDSP6_TIMER                 0x0000E500
#define PDSP6_IRAM                  0x00015000

#define PDSP7_BASE                  0x0000F600
#define PDSP7_DEBUG                 0x0000FE00
#define PDSP7_TIMER                 0x0000E600
#define PDSP7_IRAM                  0x00016000

#define PDSP8_BASE                  0x0000F700
#define PDSP8_DEBUG                 0x0000FF00
#define PDSP8_TIMER                 0x0000E700
#define PDSP8_IRAM                  0x00017000

// MCDMA
#define MCDMA1_BASE                 0x00007000
#define MCDMA1_SIZE                 0x00000100

#define MCDMA2_BASE                 0x00007400
#define MCDMA2_SIZE                 0x00000100

#define MCDMA3_BASE                 0x00007800
#define MCDMA3_SIZE                 0x00000100

#define MCDMA4_BASE                 0x00007C00
#define MCDMA4_SIZE                 0x00000100

// INTD
#define INTD1_BASE                  0x0000C000
#define INTD1_SIZE                  0x00000400

#define INTD2_BASE                  0x0000D000
#define INTD2_SIZE                  0x00000400

// Queue Managers
#define QM1_CFG                     0x00002000
#define QM1_QPEEKBASE               0x00040000
#define QM1_QPEEKSIZE               0x00020000
#define QM1_QBASE                   0x00080000
#define QM1_QSIZE                   0x00020000
#define QM1_QSTSBASE                0x00006000
#define QM1_QSTSSIZE                0x00000400

#define QM2_CFG                     0x00004000
#define QM2_QPEEKBASE               0x00060000
#define QM2_QPEEKSIZE               0x00020000
#define QM2_QBASE                   0x000A0000
#define QM2_QSIZE                   0x00020000
#define QM2_QSTSBASE                0x00006400
#define QM2_QSTSSIZE                0x00000400

// Total queues
#define QM_MAX_QUEUES               0x00004000

// Scratch RAM
#define LRAM0_BASE                  0x0001C000
#define LRAM0_SIZE                  0x00004000
#define LRAM0_RD                    0
#define LRAM0_WD                    0

#define LRAM1_BASE                  0x00020000
#define LRAM1_SIZE                  0x00004000
#define LRAM1_RD                    0
#define LRAM1_WD                    0

#define LRAM2_BASE                  0x00024000
#define LRAM2_SIZE                  0x00002000
#define LRAM2_RD                    0
#define LRAM2_WD                    0

#define LRAM3_BASE                  0x00028000
#define LRAM3_SIZE                  0x00004000
#define LRAM3_RD                    0
#define LRAM3_WD                    0

#define LRAM4_BASE                  0x0002C000
#define LRAM4_SIZE                  0x00002000
#define LRAM4_RD                    0
#define LRAM4_WD                    0

#define LRAM5_BASE                  0x00030000
#define LRAM5_SIZE                  0x00004000
#define LRAM5_RD                    0
#define LRAM5_WD                    0

#define LRAM6_BASE                  0x00034000
#define LRAM6_SIZE                  0x00002000
#define LRAM6_RD                    0
#define LRAM6_WD                    0

#define LRAM7_BASE                  0x00038000
#define LRAM7_SIZE                  0x00004000
#define LRAM7_RD                    0
#define LRAM7_WD                    0

#define LRAM8_BASE                  0x0003C000
#define LRAM8_SIZE                  0x00002000
#define LRAM8_RD                    0
#define LRAM8_WD                    0

#define EXTRAM_BASE                 0x80000000
#define EXTRAM_SIZE                 0x00010000
#define EXTRAM_RD                   32
#define EXTRAM_WD                   1
#else
#ifdef KEYSTONE1
// PDSP IRAM 1K instructions = 4K bytes
#define PDSP_IRAM_SIZE              1024


// Platform Report Commands
#define PRCMD_BASE                  0x00100000
#define PRCMD_SIZE                  0x00000004

// PDSP1
#define PDSP1_BASE                  0x0006E000
#define PDSP1_DEBUG                 0x000B0000
#define PDSP1_TIMER                 0x000A8000
#define PDSP1_IRAM                  0x00060000

// PDSP2
#define PDSP2_BASE                  0x0006F000
#define PDSP2_DEBUG                 0x000B1000
#define PDSP2_TIMER                 0x000A8800
#define PDSP2_IRAM                  0x00061000

// TPDSP
#define TPDSP_BASE                  0x00101000
#define TPDSP_DEBUG                 0x00101100
#define TPDSP_IRAM                  0x00101200

// MCDMA
#define MCDMA_BASE                  0x00090000
#define MCDMA_SIZE                  0x00000100

// INTD
#define INTD_BASE                   0x000A0000
#define INTD_SIZE                   0x00000400

// Queue Manager
#define QM1_CFG                     0x00068000
#define QM_QPEEKBASE                0x00000000
#define QM_QPEEKSIZE                0x00020000
#define QM_QBASE                    0x00020000
#define QM_QSIZE                    0x00020000
#define QM_QSTSBASE                 0x00068100
#define QM_QSTSSIZE                 0x00000800

// Total queues
#define QM_MAX_QUEUES               0x00002000

// Scratch RAM
#define LRAM1_BASE                  0x000B8000
#define LRAM1_SIZE                  0x00004000
#define LRAM1_RD                    0
#define LRAM1_WD                    0

#define LRAM2_BASE                  0x000BC000
#define LRAM2_SIZE                  0x00002000
#define LRAM2_RD                    0
#define LRAM2_WD                    0

#define EXTRAM_BASE                 0x000C0000
#define EXTRAM_SIZE                 0x00008000
#define EXTRAM_RD                   32
#define EXTRAM_WD                   1
#else
#error Unknown device
#endif
#endif


//=============================================================================
//
// PDSP
//

//-----------------------------------------------------
//
// Event/Satus Bits
//
#define tProfiler               t30
#define tStatus_Timer           t31


//-----------------------------------------------------
//
// Constants
//
#define cLRAM1                  c0      // Pointer to local RAM block 1
#define cLRAM2                  c1      // Pointer to local RAM block 2
#define cTimer                  c2      // Pointer to PDSP Timer (PDSP specific)
#define cIntStatus              c3      // Pointer to INTD Status Reg
#define cIntCount               c4      // Pointer to INTD Count Registers
#define cQPending               c5      // Pointer to QM Pending Bits
#define cQBase                  c6      // Pointer to QM Queue Base
#define cQPeekBase              c7      // Pointer to QM Queue "Peek" Base
#define cDMA                    c8      // Pointer to DMA Base
#define cLRAM3                  c9      // Pointer to local RAM block 2
#ifdef KEYSTONE2
#define cRAMPRIV                c10     // Local RAM for this PDSP
#define cRAMGLBL                c11     // Global RAM for all PDSP cores
#define cDMACHAN                c12     // First DMA channel for this PDSP
#endif


//-----------------------------------------------------
//
// PDSP Timer Equates
//
#define TIMER_OFF_CTRL      0x0
#define TIMER_OFF_LOAD      0x4
#define TIMER_OFF_COUNT     0x8
#define TIMER_OFF_EVENT     0xc


//-----------------------------------------------------
//
// PDSP Accumulator Equates
//

// Memory Map
#define ACCOFF_CHAN_COMMAND             0x0000
#define ACCOFF_CHAN_STATUS              0x0040
#define ACCOFF_CPUNUM                   0x0018
#define ACCOFF_VERSION                  0x0018

// Commands
#define ACMD_DISABLE_CHAN               0x80
#define ACMD_ENABLE_CHAN                0x81
#define ACMD_SET_TIMER                  0x82
#define ACMD_SET_RECLAMATION            0x83
#define ACMD_SET_DIVERSION              0x84
#define ACMD_SET_DDR_BARRIER            0x85
#define ACMD_SET_MSMC_BARRIER           0x86

// Return Codes
#define ACMD_RETCODE_SUCCESS            0x01
#define ACMD_RETCODE_INVALID_COMMAND    0x02
#define ACMD_RETCODE_INVALID_CHANNEL    0x03
#define ACMD_RETCODE_CHANNEL_NOT_ACTIVE 0x04
#define ACMD_RETCODE_CHANNEL_ACTIVE     0x05
#define ACMD_RETCODE_INVALID_QUEUE      0x06
#define ACMD_RETCODE_INVALID_MAXENTRIES 0x07

//-----------------------------------------------------
//
// PDSP QOS Equates
//

// Memory Map
#define QOSOFF_COMMAND                  0x0000
#define QOSOFF_QOSCLUSTER               0x40
#define QOSOFF_QOSQUEUES                0x200
#define QOSOFF_SRIO                     0xC00
#define QOSOFF_SRIO_GARBAGE_QS          0xC18
#define QOSOFF_VERSION                  0xFF8

// Commands
#define QCMD_GET_QUEUE_BASE             0x80
#define QCMD_SET_QUEUE_BASE             0x81
#define QCMD_TIMER_CONFIG               0x82
#define QCMD_CLUSTER_ENABLE             0x83
#define QCMD_SRIO_ENABLE                0x84

// Options
#define QOPT_CLUSTER_DISABLE            0
#define QOPT_CLUSTER_ENABLE             1
#define QOPT_CLUSTER_ENABLE_TIMED       1
#define QOPT_CLUSTER_ENABLE_UNTIMED     2

// Options
#define QOPT_SRIO_DISABLE               0
#define QOPT_SRIO_ENABLE                1

// Return Codes
#define QCMD_RETCODE_SUCCESS            0x01
#define QCMD_RETCODE_INVALID_COMMAND    0x02
#define QCMD_RETCODE_INVALID_INDEX      0x03
#define QCMD_RETCODE_INVALID_OPTION     0x04

//-----------------------------------------------------
//
// PDSP QOS Scheduler Equates
//

// Memory map
#define QOSSCHED_OFF_COMMAND                 0x0000
#define QOSSCHED_OFF_PORT_SHADOW             0x0040
#ifndef MULTIGROUP
#define QOSSCHED_DROPSCHED_OFF_PUSHPROXY     0x02E0
#endif
#ifdef WIDE
#define QOSSCHED_OFF_STATS                   0x0660
#else
#define QOSSCHED_OFF_STATS                   0x0300
#endif

#ifdef MULTIGROUP
// There are .codeword at the bottom of qos_sched.p that
// help decide the sizes for each area.
#ifdef WIDE
#define QOSSCHED_OFF_PORT_CFGS               0x0680
// PORT_CFG_SIZE_FULL = 0x5E8
// 1 FULL ports + 0 lite ports = 0x5E8; 0x680+0x5E8=0xC68
#define QOSSCHED_OFF_PORT_PROCS              0x0C68
// PORT_PROC_SIZE_FULL 0x1000
// 1*0x1000 = 0x1000 => end of PORT_PROCS = 1C68
//
// Free space : 0x1e00 - 0x1C68 = 0x198
#else
#define QOSSCHED_OFF_PORT_CFGS               0x0320
// PORT_CFG_SIZE_LITE = 0x60
// PORT_CFG_SIZE_FULL = 0x220
// 2 FULL ports + 0xa lite ports = 0x800; (2*0x220+10*0x60)
#define QOSSCHED_OFF_PORT_PROCS              0x0B20
// PORT_PROC_SIZE_LITE 0x9c
// PORT_PROC_SIZE_FULL 0x4ec
// 2*0x4ec + 0xa * 0x9c = 0xff0  
// end : 0xff0 + 0xb20 = 0x1b10
// Free space : 0x1e00 - 0x1b10 = 0x2f0
#endif
#else
#define QOSSCHED_OFF_PORT_CFGS               0x0320
// There are .codeword at the bottom of qos_sched.p that
// help decide the sizes for each area.
// PORT_CFG_SIZE_LITE = 0x34
// 0x34 * 20 = 0x410
#define QOSSCHED_OFF_PORT_PROCS              0x0730
// PORT_PROC_SIZE_LITE 0x84
// 20 * 0x84 = 0xA50 => end of PORT_PROCS = 0x1180
#define QOSSCHED_DROPSCHED_NUM_OUT_PROFS    36 // must be even
#define QOSSCHED_DROPSCHED_OFF_OUT_PROFS    0x1180
// 36 * 12 = 0x1b0; 0x1180 + 0x1b0 = 0x1330
#define QOSSCHED_DROPSCHED_NUM_CFG_PROFS    8
#define QOSSCHED_DROPSCHED_OFF_CFG_PROFS    0x1330
// 8 * 20 = 0x0a0; 0x1330 + 0xa0 = 0x13d0
#define QOSSCHED_DROPSCHED_NUM_STATS_BLOCKS 48
#define QOSSCHED_DROPSCHED_OFF_STATS_BLOCKS 0x13d0
// 48 * 16 = 0x300; 0x13d0 + 0x300 = 0x16d0
#define QOSSCHED_DROPSCHED_NUM_INPUT_QUE    80
#define QOSSCHED_DROPSCHED_NUM_QUE_PROFS    QOSSCHED_DROPSCHED_NUM_INPUT_QUE
#define QOSSCHED_DROPSCHED_OFF_QUE_PROFS    0x16D0
// 80 * 4 = 0x140; 0x16D0 + 0x140 = 0x1810
#define QOSSCHED_DROPSCHED_OFF_CFG          0x1810
// Config ends 0x1810 + 0x20 = 0x1830
#define QOSSCHED_DROPSCHED_OFF_PROC         0x1830
// Proc ends 0x1830 + 0xc = 0x183C
#define QOSSCHED_DROPSCHED_OFF_QUE_EN_BITS  0x183C // L=80/32 = 0x0C
//
// The following are true scratch, they don't persist across
// calls
#define QOSSCHED_DROPSCHED_OFF_IN_PKTS_PER_IN_Q       0x1848 // L=80   = 0x50
#define QOSSCHED_DROPSCHED_OFF_OUT_PKTS_PER_OUT_PROF  0x1898 // L=36   = 0x24
#define QOSSCHED_DROPSCHED_OFF_OUT_BYTES_PER_OUT_PROF 0x18BC // L=36*2 = 0x48
#define QOSSCHED_DROPSCHED_OFF_IN_PENDING_BITS        0x1904 // L=80/32 = 0x0C
#define QOSSCHED_DROPSCHED_OFF_PROBS                  0x1910 // L=36*2 = 0x48

// Free Space : 0x1D80 - 0x1958 = 0x428
#endif

// Reserving space for 32 registers for debug purposes
#define QOSSCHED_OFF_DEBUG_CONTEXT           0x1D80
// Define a stack to save overlaid registers
#define QOSSCHED_OFF_STACK_BASE              0x1E00
#define QOSSCHED_OFF_STACK_TOP               0x1F80
#define QOSSCHED_OFF_BACKGROUND_CONTEXT      0x1F80
#define QOSSCHED_OFF_TIMER_TICK              0x1FF0
#define QOSSCHED_OFF_TCAL                    0x1FF4
#define QOSSCHED_OFF_VERSION                 0x1FF8

// Commands specific to QoS scheduler
#define QOSSCHED_CMD_GET_QUEUE_BASE          0x80
#define QOSSCHED_CMD_SET_QUEUE_BASE          0x81
#define QOSSCHED_CMD_TIMER_CONFIG            0x82
#define QOSSCHED_CMD_PORT_ENABLE             0x90
#define QOSSCHED_CMD_PORT_SHADOW             0x91
#define QOSSCHED_CMD_REQ_STATS               0x92

// Options for QoS scheduler port enable
#define QOPT_PORT_DISABLE                    0
#define QOPT_PORT_ENABLE                     1

// Options for QoS scheduler port shadow
#define QOPT_PORT_SHADOW_IN                  0 // copy active to shadow
#define QOPT_PORT_SHADOW_OUT                 1 // copy shadow to active

// Options for QoS scheduler stats request
#define QOPT_RESET_STATS_FORWARDED_BYTES     0
#define QOPT_RESET_STATS_FORWARDED_PACKETS   1
#define QOPT_RESET_STATS_DISCARDED_BYTES     2
#define QOPT_RESET_STATS_DISCARDED_PACKETS   3
#define QOPT_STATS_REQUEST_DROPSCHED         7

// MSB for port shadow
#define DROP_SCHED_SHADOW_QOS_PORT 0
#define DROP_SCHED_SHADOW_CFG_PROF 1
#define DROP_SCHED_SHADOW_QUE_CFG  2
#define DROP_SCHED_SHADOW_OUT_PROF 3
#define DROP_SCHED_SHADOW_CFG      4

// Scaling factors for units
#define QOS_SCHED_WRR_PACKETS_SCALE      17
#define QOS_SCHED_PACKETS_SCALE          20
#define QOS_SCHED_WRR_BYTES_SCALE        8
#define QOS_SCHED_BYTES_SCALE            11

// QoS scheduler dimensioning
#ifdef MULTIGROUP
#ifdef WIDE
#define QOS_SCHED_FULL_PORTS             1
#define QOS_SCHED_FULL_GROUPS            17
#define QOS_SCHED_FULL_QUEUES            8
#define QOS_SCHED_LITE_PORTS             0
#define QOS_SCHED_LITE_GROUPS            1
#define QOS_SCHED_LITE_QUEUES            4
#define QOS_SCHED_PORTS                  (QOS_SCHED_FULL_PORTS + QOS_SCHED_LITE_PORTS)
#else
#define QOS_SCHED_FULL_PORTS             2
#define QOS_SCHED_FULL_GROUPS            5
#define QOS_SCHED_FULL_QUEUES            8
#define QOS_SCHED_LITE_PORTS             10
#define QOS_SCHED_LITE_GROUPS            1
#define QOS_SCHED_LITE_QUEUES            4
#define QOS_SCHED_PORTS                  (QOS_SCHED_FULL_PORTS + QOS_SCHED_LITE_PORTS)
#endif
#else
#ifdef DROP_SCHED
#define QOS_SCHED_FULL_PORTS             0
#define QOS_SCHED_FULL_GROUPS            5
#define QOS_SCHED_FULL_QUEUES            8
#define QOS_SCHED_LITE_PORTS             20
#define QOS_SCHED_LITE_GROUPS            1
#define QOS_SCHED_LITE_QUEUES            4
#define QOS_SCHED_PORTS                  (QOS_SCHED_FULL_PORTS + QOS_SCHED_LITE_PORTS)
#endif
#endif


// Qos scheduler return codes
#define QOSSCHED_CMD_RETCODE_SUCCESS         0x01
#define QOSSCHED_CMD_RETCODE_INVALID_COMMAND 0x02
#define QOSSCHED_CMD_RETCODE_INVALID_INDEX   0x03
#define QOSSCHED_CMD_RETCODE_INVALID_OPTION  0x04



//-----------------------------------------------------
//
// PDSP SRIO RTR Equates
//
#define SRIORTR_OFF_VERSION                  0x0004
#define SRIORTR_OFF_ENABLES                  0x001c
#define SRIORTR_OFF_CFG                      0x002c
#define SRIORTR_OFF_CFG_OUTPUTS              (SRIORTR_OFF_CFG + 0x08)
#define SRIORTR_OFF_CFG_OUTPUTS_HOST         (SRIORTR_OFF_CFG_OUTPUTS + 0x40)
#define SRIORTR_OFF_CFG_OUTPUTS_CREDIT       (SRIORTR_OFF_CFG_OUTPUTS + 0x50)
#define SRIORTR_OFF_CFG_CREDRET              (SRIORTR_OFF_CFG + 0x68)
#define SRIORTR_OFF_CFG_CREDSRC              (SRIORTR_OFF_CFG + 0x78)
#define SRIORTR_OFF_CFG_FWDRXFREE            (SRIORTR_OFF_CFG + 0x88)
#define SRIORTR_OFF_STATS                    0x00d0
#define SRIORTR_OFF_STATS_PKTRX              (SRIORTR_OFF_STATS + 0x00)
#define SRIORTR_OFF_STATS_PKTTX              (SRIORTR_OFF_STATS + 0x10)
#define SRIORTR_OFF_STATS_CRTX               (SRIORTR_OFF_STATS + 0x20)
#define SRIORTR_OFF_STATS_CRRX               (SRIORTR_OFF_STATS + 0x30)
#define SRIORTR_OFF_STATS_HOSTRX             (SRIORTR_OFF_STATS + 0x40)
#define SRIORTR_OFF_STATS_FWDRETS            (SRIORTR_OFF_STATS + 0x50)
#define SRIORTR_OFF_LOOKUP                   0x0130
#define SRIORTR_OFF_DESTID                   0x01B0
#define SRIORTR_OFF_CREDITS                  0x01C0


#define SRIORTR_OFF_DESC_BUFPTR              0x10
#define SRIORTR_OFF_DESC_PSINFO              0x20
#endif

