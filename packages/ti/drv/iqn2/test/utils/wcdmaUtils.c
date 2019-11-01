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

#include <ti/drv/iqn2/iqn2.h>
#include "cslUtils.h"

#ifdef USESYSBIOS
#include <ti/sysbios/family/c64p/Hwi.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#endif

#include "wcdmaUtils.h"

extern volatile unsigned int testcheck;


void load_dioData(IQN2_DioConfigHandle hDioConfig)
{
	uint32_t idx, idx2, numChip;
	uint32_t *ptr;

	if(hDioConfig->rsaOn)
	    numChip = 8;
	else
	    numChip = 4;
	if (hDioConfig->out[0]) {
	    UTILS_cacheInvalidate((void *)hDioConfig->out[0], (hDioConfig->numPdDBCH*numChip*hDioConfig->inNumBlock)); // writeback in MSM
	}
	for (idx2 = 0; idx2 < hDioConfig->numPdDBCH; idx2 ++)
	{
	    ptr = hDioConfig->out[idx2];
	    if (ptr)
	    {
            for(idx =0; idx < numChip*hDioConfig->inNumBlock; idx++)
            {
                *ptr = (idx2 << 24) + idx;///numChip;
                ptr++;
            }
	    }
	}
	if (hDioConfig->out[0]) {
	    UTILS_cacheWriteBack((void *)hDioConfig->out[0], (hDioConfig->numPdDBCH*numChip*hDioConfig->inNumBlock)); // writeback in MSM
	}
}

void load_dioData_lte(IQN2_DioConfigHandle hDioConfig)
{
	uint32_t idx, idx2, numChip;
	uint32_t *ptr;

	numChip = 4;
	if (hDioConfig->out[0]) {
	    UTILS_cacheInvalidate((void *)hDioConfig->out[0], (hDioConfig->numPdDBCH*numChip*hDioConfig->inNumBlock)); // writeback in MSM
	}
	for (idx2 = 0; idx2 < hDioConfig->numPdDBCH; idx2 ++)
	{
	    ptr = hDioConfig->out[idx2];
	    if (ptr)
	    {
            for(idx =0; idx < numChip*hDioConfig->inNumBlock; idx++)
            {
                *ptr = (idx2 << 24) + idx;///numChip;
                ptr++;
            }
	    }
	}
	if (hDioConfig->out[0]) {
	    UTILS_cacheWriteBack((void *)hDioConfig->out[0], (hDioConfig->numPdDBCH*numChip*hDioConfig->inNumBlock)); // writeback in MSM
	}
}

void wcdmaFinalCheck(IQN2_DioConfigHandle hDioConfig)
{
	uint32_t testpass = 0;
	uint32_t numChip, i;

    if(hDioConfig->rsaOn)
        numChip = 8;
    else
        numChip = 4;

	/* Compare the WCDMA DIO loopback data */
    for (i = 0; i < hDioConfig->numPdDBCH; i++)
    {
        if ((hDioConfig->out[i])&&(hDioConfig->in[i]))
        {
            UTILS_cacheInvalidate((void *)hDioConfig->out[i], (numChip*hDioConfig->outNumBlock)); // writeback in MSM
            UTILS_cacheInvalidate((void *)hDioConfig->in[i], (numChip*hDioConfig->inNumBlock)); // writeback in MSM
            testpass |= memcmp(hDioConfig->in[i], hDioConfig->out[i], (numChip*hDioConfig->inNumBlock));
        } else {
            testpass = 1;
        }
    }

	if (testpass == 0)
	{
	    printf(" \nDIO AxC Data Send/Recv: PASS\n");
    } else {
	    printf(" \nDIO AxC Data Send/Recv: FAIL\n");
	    testcheck++;
    }
}
