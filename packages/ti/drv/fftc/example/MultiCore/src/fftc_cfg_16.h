/** 
 *   @file  fftc_cfg_16.h
 *
 *   @brief  
 *      Header file containing FFTC Test vector details.
 *
 *      The format of the file contents are as follows:
 *          *   Line 1:                         Number of FFT data blocks in this
 *                                              packet configuration.
 *          *   Line 2 onwards (32 entries):    FFT configuration for the data, details
 *                                              below:
 *                  (01)    DFT sample size - each sample is again 4 bytes wide.
 *                          (Fftc_QLocalCfg.controlRegConfig.dftSize)
 *                  (02)    Not used
 *                  (03)    Not used
 *                  (04)    Not used
 *                  (05)    Dynamic Scaling Enable Boolean flag
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.bDynamicScaleEnable)
 *                  (06)    Last Radix Stage Output Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingValLast)
 *                  (07)    Radix Stage 0 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[0])
 *                  (08)    Radix Stage 1 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[1])
 *                  (09)    Radix Stage 2 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[2])
 *                  (10)    Radix Stage 3 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[3])
 *                  (11)    Radix Stage 4 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[4])
 *                  (12)    Radix Stage 5 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[5])
 *                  (13)    Radix Stage 6 Scaling value (only for static scaling)
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.radixScalingVal[6])
 *                  (14)    Not used
 *                  (15)    Not used
 *                  (16)    Not used
 *                  (17)    Inverse FFT enable boolean flag
 *                          (Fftc_QLocalCfg.controlRegConfig.bInverseFFTEnable)
 *                  (18)    Emulate DSP 16x16 boolean flag - Not used currently
 *                          (Fftc_QLocalCfg.controlRegConfig.bEmulateDSP16x16)
 *                  (19)    Output Scaling Value
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.outputScaleVal)
 *                  (20)    Input Shifting Boolean flag
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.bInputFFTShift)
 *                  (21)    Output Shifting Boolean flag
 *                          (Fftc_QLocalCfg.scalingShiftingRegConfig.bOutputFFTShift)
 *                  (22)    Zero Pad Mode Enable boolean flag
 *                          (Fftc_QLocalCfg.controlRegConfig.bZeroPadEnable)
 *                  (23)    Zero Pad Mode (used only if zero padding enabled) 
 *                          (Fftc_QLocalCfg.controlRegConfig.zeroPadMode)
 *                  (24)    Zero Pad Factor (used only if zero padding enabled) 
 *                          (Fftc_QLocalCfg.controlRegConfig.zeroPadFactor)
 *                  (25)    LTE Frequency Shifting Enable boolean flag
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.bFreqShiftEnable)
 *                  (26)    LTE Frequency Shift value (used only if LTE freq shifting enabled)
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.freqShiftVal)
 *                  (27)    LTE Frequency Shift Table Index (used only if LTE freq shifting enabled)
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.freqShiftIndex)
 *                  (28)    LTE Frequency Shift Multiplication factor (used only if LTE freq shifting enabled)
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.freqShiftMultFactor)
 *                  (29)    LTE Frequency Shift Phase (used only if LTE freq shifting enabled)
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.freqShiftInitPhase)
 *                  (30)    LTE Frequency Shift Direction (used only if LTE freq shifting enabled)
 *                          (Fftc_QLocalCfg.freqShiftRegConfig.freqShiftDirection)
 *                  (31)    Cyclic Prefix Addition Enable Boolean flag 
 *                          (Fftc_QLocalCfg.cyclicPrefixRegConfig.bCyclicPrefixAddEnable)
 *                  (32)    Cyclic Prefix Addition Value (Used only if cyclic prefix addition enabled) 
 *                          (Fftc_QLocalCfg.cyclicPrefixRegConfig.cyclicPrefixAddNum)
 *          *   Block Number
 *          *   FFT input data (sample size * 2 entries)
 *          *   Expected Block Exponent Value
 *          *   Expected Clipping Detect Value
 *          *   Expected Number of clock cycles to be taken for operation          
 *          *   Expected FFT output data - reference data to validate result from FFT engine.
 *              (sample size * 2 entries)
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

#ifndef _FFTC_TESTCFG_16_H_
#define _FFTC_TESTCFG_16_H_

Int16 inputConfig_16[] = {
5,
16,
16,
19,
15,
1,
0,
0,
0,
0,
0,
0,
0,
0,
1,
1,
1,
0,
0,
128,
0,
0,
0,
0,
1,
0,
0,
0,
0,
0,
0,
0,
0,
1,
8617,
-6897,
9566,
-1307,
-2223,
-625,
1331,
1946,
-5060,
10525,
1071,
5138,
3774,
1234,
7729,
-6788,
-1929,
7391,
-10010,
-1800,
348,
8066,
-7841,
1202,
-3739,
6377,
-3498,
4674,
-2551,
-1685,
4247,
-1383,
1,
0,
11,
14016,
-3780,
-2261,
-8303,
1434,
13521,
11651,
32393,
10790,
17735,
-5647,
-4602,
-16023,
5677,
269,
2068,
-1066,
6802,
-5123,
4813,
1992,
-2097,
-6609,
9948,
-23908,
3465,
81,
-14425,
7240,
7444,
4721,
-8710,
2,
5804,
-5035,
-615,
6218,
294,
-7772,
3476,
591,
4597,
-7139,
-1844,
-1540,
5377,
1782,
5547,
-2393,
-513,
1378,
-5061,
-2405,
-4111,
1014,
-1200,
-10576,
-2917,
-216,
4441,
-1008,
-7439,
6065,
-8120,
-1155,
1,
0,
11,
3674,
951,
3087,
-13688,
16447,
7797,
3341,
-6363,
18962,
4092,
10344,
7407,
-6939,
-3770,
-7311,
8401,
-15912,
3879,
4664,
2490,
8000,
6138,
3247,
-13375,
-9009,
-7408,
14938,
7039,
-13040,
12366,
-6609,
-1512,
3,
7365,
1422,
-5105,
3825,
-722,
-1406,
-6138,
-1310,
2163,
2195,
-2989,
1660,
3172,
281,
-2716,
3590,
4972,
-2435,
-242,
-5518,
-2896,
1660,
5918,
-1845,
1756,
-6079,
1121,
-3880,
-4792,
-75,
-4504,
4818,
0,
0,
11,
5287,
18648,
-8259,
3982,
28422,
-1078,
-9362,
-8529,
-15227,
10206,
21345,
13594,
29430,
616,
24588,
4177,
-12021,
12654,
9643,
24056,
2020,
9891,
7950,
-19429,
14687,
-7268,
17669,
-914,
-8526,
13164,
22402,
-6425,
4,
-1857,
-1981,
1831,
6820,
-27,
-5492,
521,
-4767,
4167,
-1831,
3242,
-402,
-1754,
-1107,
-6167,
4520,
9642,
9439,
1585,
-4561,
-294,
2365,
-5359,
-838,
-2502,
-10585,
-1408,
3774,
-2599,
-6248,
-1631,
-4029,
1,
0,
11,
-2142,
6030,
6668,
5497,
864,
15548,
2243,
-8639,
2098,
-15995,
4590,
-8619,
239,
-13021,
-5319,
-4897,
-6625,
10747,
-4160,
18646,
13821,
-1450,
7509,
7140,
4059,
691,
3474,
-13259,
-2761,
10846,
13243,
15215,
5,
-3868,
-5828,
1885,
-3059,
5378,
14349,
3471,
1074,
-2134,
-1144,
-3348,
319,
-2040,
-2488,
1856,
3124,
-3899,
-5051,
-7521,
-6727,
3625,
-2271,
3151,
-5394,
-945,
4423,
636,
-1105,
844,
4223,
12404,
9993,
1,
0,
11,
3774,
-19792,
-25403,
-10024,
-986,
-12601,
5448,
-92,
-2574,
11086,
-6378,
-9360,
-5543,
23400,
7652,
10446,
3193,
-9250,
11333,
5838,
-6238,
1805,
-2423,
-2787,
5102,
-5189,
-13856,
-6991,
-2807,
-8110,
-13680,
12868
};

#endif /* _FFTC_TESTCFG_16_H_ */
