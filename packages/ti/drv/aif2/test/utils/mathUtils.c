/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <c6x.h>

#include <ti/csl/csl.h>

#define __MATHUTILS_C
#include "mathUtils.h"

int PEAKDEBUG=0;

short debugPeakAlgo[4][14][8];
short debugPeakCntAlgo[4][14];
char  debugPeakIdx[4] = {0,0,0,0};
unsigned int thresholdDividerSingleTone = 1;
unsigned int thresholdDividerDualTone = 10;

#pragma FUNCTION_OPTIONS (find_eight_peaks_over_threshold, "-o3" );
int find_eight_peaks_over_threshold(short * restrict array, int size, int threshold, short * restrict result) {
       int i, j;
       int test;
       int count = 0;
       int final_count;

       for (i=2; i<2*size-2; i+=2) {
              /* Find local max values that are over the threshold. */
              test  = (_abs(array[i-2]) + _abs(array[i-1])) < (_abs(array[i]) + _abs(array[i+1]));
              test += (_abs(array[i]) + _abs(array[i+1])) > (_abs(array[i+2]) + _abs(array[i+3]));
              test += (_abs(array[i]) + _abs(array[i+1])) > threshold;
              if (test == 3) {
                     if (count < 8) {result[count] = i/2;}
                     count++;
              }
       }
       final_count = count;
       if (count > 8) { count = 8; }
       for (i=1, j=1; i<count; i++) {
    	   /* Merge peaks that are really next to each other. */
    	   if ((result[i]-result[i-1]) <= 2) {
    		   final_count--;
    	   } else {
    		   result[j++] = result[i];
    	   }
       }
       return final_count;
}
#pragma FUNCTION_OPTIONS (checkSingleTone, "-o3" );
int avgPeak;
int checkSingleTone(
		short * restrict array, int size, int expectedPeakLoc, unsigned int axc
)
{
	short peakArray[8];
	int count;
	short * restrict arrayRe = array;
	short * restrict arrayIm = array + 1;

	avgPeak =  (_abs(arrayRe[expectedPeakLoc*2]) + _abs(arrayIm[expectedPeakLoc*2]));
	avgPeak += (_abs(arrayRe[(expectedPeakLoc-1)*2]) + _abs(arrayIm[(expectedPeakLoc-1)*2]));
	avgPeak += (_abs(arrayRe[(expectedPeakLoc+1)*2]) + _abs(arrayIm[(expectedPeakLoc+1)*2]));
	avgPeak /= 3;
	count = find_eight_peaks_over_threshold(arrayRe, size, avgPeak/thresholdDividerSingleTone, peakArray);

	if (PEAKDEBUG == 1) {
		debugPeakCntAlgo[axc][debugPeakIdx[axc]] = count;
		memcpy(&debugPeakAlgo[axc][debugPeakIdx[axc]][0], peakArray, sizeof(peakArray));
		debugPeakIdx[axc]++;
		if (debugPeakIdx[axc] == 14) debugPeakIdx[axc] = 0;
	}

	if(count == 1 && _abs(peakArray[0] - expectedPeakLoc) < 2 ) {
		return (1);
	} else {
		return (0);
	}
}
#pragma FUNCTION_OPTIONS (checkDualTone, "-o3" );
int checkDualTone(
		short * restrict array, int size, int expectedPeakLoc1, int expectedPeakLoc2, unsigned int axc
)
{
	short peakArray[8];
	int count;
	short * restrict arrayRe = array;
	short * restrict arrayIm = array + 1;

	avgPeak =  (_abs(arrayRe[expectedPeakLoc1*2]) + _abs(arrayIm[expectedPeakLoc1*2]));
	avgPeak += (_abs(arrayRe[(expectedPeakLoc1-1)*2]) + _abs(arrayIm[(expectedPeakLoc1-1)*2]));
	avgPeak += (_abs(arrayRe[(expectedPeakLoc1+1)*2]) + _abs(arrayIm[(expectedPeakLoc1+1)*2]));
	avgPeak /= 3;
	count = find_eight_peaks_over_threshold(arrayRe, size, avgPeak/thresholdDividerDualTone, peakArray);

	if (PEAKDEBUG == 1) {
		debugPeakCntAlgo[axc][debugPeakIdx[axc]] = count;
		memcpy(&debugPeakAlgo[axc][debugPeakIdx[axc]][0], peakArray, sizeof(peakArray));
		debugPeakIdx[axc]++;
		if (debugPeakIdx[axc] == 14) debugPeakIdx[axc] = 0;
	}

	if(count == 2 && _abs(peakArray[0] - expectedPeakLoc1) < 2 && _abs(peakArray[1] - expectedPeakLoc2) < 2 ) {
		return (1);
	} else {
		return (0);
	}
}

#pragma FUNCTION_OPTIONS (BufferConvertion, "-o3" );
void BufferConvertion (
		uint32_t* origin, uint32_t* result, uint32_t buf_Size
)
{
	int8_t* restrict dioBufPtr;
	int16_t* restrict rxBufPtr;
	uint32_t i;
	uint32_t j = 0;
	uint32_t k = 0;

	dioBufPtr = (int8_t*) origin;
	rxBufPtr = (int16_t*) result;

	for (i=0;i<buf_Size*4;i++)
	{
		if (j==32)
		{
			j = 0;
			k++;
		}
		if (j<16)
		*(rxBufPtr+(i-(k*16))) = (int16_t) *(dioBufPtr+i);
		j++;
	}
}

////////////////////
