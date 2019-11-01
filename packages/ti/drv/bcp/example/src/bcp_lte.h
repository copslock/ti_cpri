/** 
 *   @file  bcp_lte.h
 *
 *   @brief  
 *      Header file with data structures and definitions used by LTE tests.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2010, Texas Instruments, Inc.
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
#ifndef _BCP_LTE_H_
#define _BCP_LTE_H_

/* LTE Test configuration definitions */

#define MAX_CODE_BLOCK_SIZE     6144
#define MAX_BLOCK_INDEX         189

/* LTE Channel types */
#define LTE_PDSCH               0
#define LTE_PDCCH               1
#define LTE_PUSCH_SIC           2
#define LTE_PUSCH               3
#define LTE_PUCCH               4
#define LTE_PDCCHM              5
#define LTE_PUSCH_SIC_HARD      6

typedef struct _BcpTest_LteCBParams
{
    UInt32       numCodeBks;
    UInt32       outputbitsNoFiller;
    UInt32       numCodeBksKp;
	UInt32       numCodeBksKm;
    UInt32       codeBkSizeKp;
	UInt32       codeBkSizeKm;
	UInt16       numFillerBits;
	UInt16       f1Km;
	UInt16       f2Km;
	UInt16       f1Kp;
	UInt16       f2Kp;
} BcpTest_LteCBParams;

typedef struct _BcpTest_RateMatchParams
{
    UInt32      E0;
    UInt32      E1;
    UInt32      Gamma;
	UInt32      NcbKm;
	UInt32      NcbKp;
	UInt32      rvKm;
	UInt32      rvKp;
	UInt32      rvKmCol;
	UInt32      rvKpCol;
	UInt32      NcbKmCol;
	UInt32      NcbKmRow;
	UInt32      NcbKpCol;
	UInt32      NcbKpRow;
} BcpTest_RateMatchParams;

#endif /* _BCP_LTE_H_ */
