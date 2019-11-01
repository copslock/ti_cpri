/*
 *
 * Copyright (C) 2010-2013 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/



/* Static memory allocation for test framework */

#include "../../pautest.h"

tFramework_t tFramework;

/* HW interrupt disable handle */
GateHwi_Handle gateHwi;

#pragma DATA_ALIGN(memPaInst, TF_CACHE_LINESZ)
uint8_t memPaInst[TF_ROUND_UP(TF_PA_INST_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memL2Ram, TF_CACHE_LINESZ)
uint8_t memL2Ram[TF_ROUND_UP(TF_L2_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memL3Ram, TF_CACHE_LINESZ);
uint8_t memL3Ram[TF_ROUND_UP(TF_L3_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memVLinkRam, TF_CACHE_LINESZ);
uint8_t memVLinkRam[TF_ROUND_UP(TF_VLINK_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memAclRam, TF_CACHE_LINESZ);
uint8_t memAclRam[TF_ROUND_UP(TF_ACL_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memEoamRam, TF_CACHE_LINESZ);
uint8_t memEoamRam[TF_ROUND_UP(TF_EOAM_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memUsrStatsLnkTbl, TF_CACHE_LINESZ);
uint8_t memUsrStatsLnkTbl[TF_ROUND_UP(TF_USR_STATS_LNK_TABLE_SIZE, TF_CACHE_LINESZ)];

#pragma DATA_ALIGN(memFcRam, TF_CACHE_LINESZ);
uint8_t memFcRam[TF_ROUND_UP(TF_FC_TABLE_SIZE, TF_CACHE_LINESZ)];

/* Memory used for the linking RAM and descriptor RAM */
#pragma DATA_ALIGN(memLinkRam, 16)
uint64_t memLinkRam[TF_NUM_DESC];

#pragma DATA_ALIGN(memDescRam, 128)
uint8_t memDescRam[TF_NUM_DESC * TF_SIZE_DESC];

#ifdef NETSS_INTERNAL_PKTDMA
uint8_t* passDescRam = (uint8_t*)(CSL_NETCP_CFG_REGS + 0x001c0000);
#endif

/* Packet buffers attached to descriptors */
#pragma DATA_ALIGN(memQ1, 16)
#pragma DATA_ALIGN(memQ2, 16)
#pragma DATA_ALIGN(memQ3, 16)
unsigned char memQ1[TF_LINKED_BUF_Q1_NBUFS][TF_LINKED_BUF_Q1_BUF_SIZE];
unsigned char memQ2[TF_LINKED_BUF_Q2_NBUFS][TF_LINKED_BUF_Q2_BUF_SIZE];
unsigned char memQ3[TF_LINKED_BUF_Q3_NBUFS][TF_LINKED_BUF_Q3_BUF_SIZE];

#pragma DATA_SECTION (memLocQ1, ".osrBufs")
#pragma DATA_SECTION (memLocQ2, ".osrBufs")
#pragma DATA_SECTION (memLocQ3, ".osrBufs")
#pragma DATA_ALIGN(memLocQ1, 16)
#pragma DATA_ALIGN(memLocQ2, 16)
#pragma DATA_ALIGN(memLocQ3, 16)
unsigned char memLocQ1[TF_LINKED_BUF_Q1_NBUFS][TF_LINKED_BUF_Q1_BUF_SIZE];
unsigned char memLocQ2[TF_LINKED_BUF_Q2_NBUFS][TF_LINKED_BUF_Q2_BUF_SIZE];
unsigned char memLocQ3[TF_LINKED_BUF_Q3_NBUFS][TF_LINKED_BUF_Q3_BUF_SIZE];

