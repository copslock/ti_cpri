/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h>
#include <math.h>

#include <ti/csl/csl.h>
#include <ti/csl/csl_cache.h>
#include <ti/csl/csl_cacheAux.h>


#if DFE_BB_LOOPBACK == 1
#define     Q_FACTOR            511     // When the signal stays digital, reducing Q factor to avoid overflow in fft software
#else
#define     Q_FACTOR            511 //8191    // 8191
#endif

#define     SAMP_FREQ_FLOAT     30.72
#define     FREQ                1.0

#define     PI                  3.14159265358979323846

// These modes are only used for extensive RF tests on CPRI relay setups
#define     DUAL_TONE_FIVE_NINE 0
#define     QUAD_TONE_ONE_TWO   0
#define     QUAD_TONE_FIVE_NINE 0
#define     EIGHTEEN_TONES      0
#define     TWENTY_FOUR_TONES   0
#define     FORTY_EIGHT_TONES   0
#define     NINETY_SIX_TONES    0

//////////////////////////////////////////////////////////////////////////////
// Typedefs
typedef struct {
	int16_t re;
	int16_t im;
	} Complex16;
	
extern	Complex16 *txPtr[16];
extern  Complex16 *rxPtr[16];

extern void resetAxCPtr();
extern void resetRxCaptBuf();
extern void getcomplex(Complex16* payload,uint32_t index,uint32_t tx, float sampFreq);

extern void getTMdata(Complex16* payload,uint32_t tx);
extern void getRMdata(Complex16* payload,uint32_t rx);

