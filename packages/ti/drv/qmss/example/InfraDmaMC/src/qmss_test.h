/**
 *   @file  qmss_test.h
 *
 *   @brief   
 *      This files contains the definitions and data structure used in the test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2013, Texas Instruments, Inc.
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
 *  \par
*/

#ifndef __QMSS_TEST_H__
#define __QMSS_TEST_H__

/************************ USER DEFINES ********************/

#define RM 1

#define NUMBER_OF_CORES             qmss_EXAMPLE_NUM_CORES

#define SYSINIT                     0
#define NUM_ITERATION               1 * NUMBER_OF_CORES

#if !RM

/* Share queues among cores using hardcoded numbers */
#define CPPI_FREE_TX_QID            736
#define CPPI_FREE_RX_QID            737
#define CPPI_COMPLETION_QID         1500
#define QMSS_SYNC_CFG_QID           2000
#define QMSS_SYNC_QID               2001
#define QMSS_FREE_SYNC_QID          2002
#else
/* Queues are shared via RM, not hardcoded nubmers */
#endif

#define NUM_MONOLITHIC_DESC         64
#define SIZE_MONOLITHIC_DESC        160
#define MONOLITHIC_DESC_DATA_OFFSET 16
#define NUM_SYNC_DESC               32
#define SIZE_SYNC_DESC              32
#define SIZE_DATA_BUFFER            16
#define NUM_PACKETS                 8

#define NUM_RX_DATA_BUFFER          NUM_PACKETS
#define NUM_TX_DATA_BUFFER          1

/************************ EXTERN VARIABLES ********************/

#ifdef __LINUX_USER_SPACE
/* linking RAM */
extern uint64_t  *linkingRAM0;

/* Descriptor pool [Size of descriptor * Number of descriptors] */
extern uint8_t *dataBuff;
extern uint8_t *monolithicDesc;
extern uint8_t *syncDesc;
extern uint8_t *dataBuff;
extern uint32_t *hiPrioList;
#if RM
extern Rm_ServiceHandle *rmClientServiceHandle;
#endif


/* virtual core/task number */
extern uint32_t coreNum;

/* error counter */
extern uint32_t errorCount;

/* Function prototypes */
void example_main(Cppi_Handle cppiHnd);
#endif

#endif /* __QMSS_TEST_H__ */

