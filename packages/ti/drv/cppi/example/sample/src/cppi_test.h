/**
 *   @file  cppi_test.h
 *
 *   @brief   
 *      This files contains the definitions and data structure used in the test code.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009-2012, Texas Instruments, Inc.
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

#ifndef __CPPI_TEST_H__
#define __CPPI_TEST_H__

/************************ USER DEFINES ********************/

#define NUM_HOST_DESC               32
#define SIZE_HOST_DESC              48
#define NUM_MONOLITHIC_DESC         32
#define SIZE_MONOLITHIC_DESC        160

#define NUM_DATA_BUFFER             32
#define SIZE_DATA_BUFFER            64

/************************ EXTERN VARIABLES ********************/

/* External CPPI global configuration file */
extern Cppi_GlobalConfigParams  sample_cppiGblCfgParams;
/* External QMSS global configuration file */
extern Qmss_GlobalConfigParams  sample_qmssGblCfgParams;

#ifdef __LINUX_USER_SPACE
/* linking RAM */
extern uint64_t  *linkingRAM0;

/* Descriptor pool [Size of descriptor * Number of descriptors] */
extern uint8_t *hostDesc;
extern uint8_t *monolithicDesc;

/* Data buffer */
extern uint8_t *dataBuff;
#endif /* __LINUX_USER_SPACE */

#endif /* __CPPI_TEST_H__ */

