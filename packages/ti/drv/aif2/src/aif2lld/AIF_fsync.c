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

#include <ti/csl/csl.h>

#include <ti/drv/aif2/AIF_defs.h>
#include <ti/drv/aif2/AIF_cfg.h>
#include <ti/drv/aif2/AIF_init.h>
#include <ti/drv/aif2/AIF_init_dat.h>

#define __AIF_FSYNC_C
#include <ti/drv/aif2/AIF_fsync.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_initFsync, ".text:aifHwi"); 
#pragma CODE_SECTION(AIF_fsync1Event7Count, ".text:aifHwi");
#pragma CODE_SECTION(AIF_fsyncEvent2_7Count, ".text:aifHwi");  
#pragma CODE_SECTION(AIF_dioEngineIsr, ".text:aifHwi");
#endif

// AIF2  need to link aifFsyncEventCount with AT timer and event 
void AIF_initFsync(
)
{
	int32_t 	i;
	//Clear FSync event counts
	for(i=0; i<30; i++)
		aifFsyncEventCount[i]=(int32_t)(-1);
		
	// Fsync init is completed so update global variable
	aifFsyncInitDone = 1;
}


//volatile uint32_t mytime1=0 , mytime2=0, mytime3=0 , mytime4=0;

void 
AIF_fsync1Event7Count(
)
{
	//event 109 (bit13) -> Fsync1
/*	if (!mytime1) {
		TSCL=0;
		mytime1 = TSCL;
	}
	else {
		mytime2 = mytime1;
		mytime1 = TSCL;
	}*/
	aifFsyncEventCount[1]++;

}
// for Debug AIF2
void 
AIF_fsyncEvent2_7Count(
    uint32_t  interruptFlag
)
{
/*	if (!mytime3) {
		mytime3=0;
		mytime3 = TSCL;
	}
	else {
		mytime4 = mytime3;
		mytime3 = TSCL;
	}*/
	aifFsyncEventCount[2]+= (interruptFlag>>6)&1;	
	aifFsyncEventCount[3]+= (interruptFlag>>7)&1;
	aifFsyncEventCount[4]+= (interruptFlag>>8)&1;
	aifFsyncEventCount[5]+= (interruptFlag>>9)&1;
	aifFsyncEventCount[6]+= (interruptFlag>>10)&1;
	aifFsyncEventCount[7]+= (interruptFlag>>11)&1;
}

void
AIF_dioEngineIsr(
    AIF_ConfigHandle    hAif
)
{ 
    int32_t i;        
    
        //Indicate ping-pong switch if AT Event 6 is enabled and hooked up to this ISR
        for(i=0; i<AIF_MAX_NUM_LINKS; i++)
        {
            if(1==hAif->linkConfig[i].linkEnable) 
            {
            	aif2DioIntCount[i]++;
            }
        }           
}


////////////////////
