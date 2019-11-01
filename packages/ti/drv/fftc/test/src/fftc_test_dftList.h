/** 
 *   @file  fftc_test.h
 *
 *   @brief  
 *      Header file with data structures and definitions required for the FFTC 
 *      driver testing.
 * 
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2009, Texas Instruments, Inc.
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

#ifndef _FFTC_TEST_H_
#define _FFTC_TEST_H_

/** Number of monolithic descriptors used by the FFTC test */
#define     FFTC_TEST_NUM_MONOLITHIC_DESC       32

/** Monolithic descriptor size. 
 *
 *  Big enough to hold all the mandatory fields of the 
 *  monolithic descriptor + data.
 */
#define     FFTC_TEST_SIZE_MONOLITHIC_DESC      10240 

/** Number of host descriptors used by the FFTC test */
#define     FFTC_TEST_NUM_HOST_DESC             32

/** Host descriptor size. 
 *
 *  Big enough to hold the mandatory fields of the 
 *  host descriptor and 16 bytes of PS info if needed.
 * 
 *  = 32 Host desc + 16 PS Info = 48 bytes
 *  Round it off to next multiple of 16 = 64 bytes
 * 
 */
#define     FFTC_TEST_SIZE_HOST_DESC            64 

/** Size of each test sample in bytes 
 *
 *  = sizeof (Cplx16) = 4 bytes
 */
#define     FFTC_TEST_SAMPLE_SIZE               4

/** 
 *  @brief  Cplx16
 *
 *          Structure to represent the FFT data input/
 *          output format.
 */     
#ifndef xdc_target__bigEndian
typedef struct _Cplx16
{
    /** Imaginary part of the FFT data */        
    Int16       imag;

    /** Real part of the FFT data */        
    Int16       real;
} Cplx16;
#else
typedef struct _Cplx16
{
    /** Real part of the FFT data */        
    Int16       real;

    /** Imaginary part of the FFT data */        
    Int16       imag;
} Cplx16;
#endif

/** 
 *  @brief  FFT_TestCfg
 *
 *          Structure to hold the FFT test input vector,
 *          output vector.
 */     
typedef struct _FFT_TestCfg
{
    /** Number of FFT blocks in the test vector */        
    UInt8               numBlocks;

    /** FFT configuration for the test */        
    Fftc_QLocalCfg      fftcQCfg;

    /** Number of input test samples */        
    UInt32              numInputSamples [FFTC_MAX_NUM_BLOCKS];

    /** Input FFT test vector data */        
    Cplx16*             pFftInputData [FFTC_MAX_NUM_BLOCKS];

    /** Number of output test samples */        
    UInt32              numOutputSamples [FFTC_MAX_NUM_BLOCKS];

    /** Expected Output FFT result data */        
    Cplx16*             pFftOutputData [FFTC_MAX_NUM_BLOCKS];

    /** Expected Clipping detect value */
    UInt32              bClippingDetected [FFTC_MAX_NUM_BLOCKS];

    /** Expected Block exponent value */
    UInt32              blockExpVal [FFTC_MAX_NUM_BLOCKS];

    /** Expected cycle count value */
    UInt32              numClockCycles [FFTC_MAX_NUM_BLOCKS];
} FFT_TestCfg;

#endif  /* _FFTC_TEST_H_ */

