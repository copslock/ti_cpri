/**
 *   @file  srio_test.h
 *
 *   @brief   
 *      This files contains the definitions and data structure used in the test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2014, Texas Instruments, Inc.
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

#ifndef __SRIO_TEST_H__
#define __SRIO_TEST_H__

#include "srioPlatCfg.h"

/************************ USER DEFINES ********************/

#define RM 1

#define NUMBER_OF_CORES             srio_EXAMPLE_NUM_CORES

#define SYSINIT                     0

#define NUM_HOST_DESC               32
#define SIZE_HOST_DESC              48
#define NUM_PACKETS                 30
#define SIZE_DATA_BUFFER            (NUM_PACKETS*256)

extern Rm_ServiceHandle *rmClientServiceHandle;
extern uint8_t *host_region;
extern uint8_t *dataBuff[NUMBER_OF_CORES];


/* virtual core/task number */
extern uint32_t coreNum;

/* error counter */
extern uint32_t errorCount;

/* Function prototypes */
void example_main(Cppi_Handle cppiHnd);

#endif /* __SRIO_TEST_H__ */

