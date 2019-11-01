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
#include <ti/csl/csl.h>
#include <ti/csl/csl_chip.h>

#include "dfe_inc.h"
#ifdef _TMS320C6X
#pragma DATA_SECTION(dfetgtcfgsize,".dfetgtcfgsize");
#pragma DATA_SECTION(dfetgtcfg,".dfetgtcfg");
#endif
Uint32 dfetgtcfgsize;
Uint32 *g_dfeTgtCfgSize = (Uint32 *)(&dfetgtcfgsize);
Uint32 dfetgtcfg;
AVV_RwTestRecord *g_dfeTgtCfg = (AVV_RwTestRecord *)(&dfetgtcfg);

//extern DfeFl_Handle hDfe;

// load DFE tgtcfg
int loadDfe_local(DfeFl_Handle hDfe)
{
    Uint32 ui, addr;
    
    for(ui = 0; ui < *g_dfeTgtCfgSize; ui++)
    {
        addr = g_dfeTgtCfg[ui].addr >> 2;
        
        hDfe->regs[addr] = g_dfeTgtCfg[ui].value;
    }
    
    return (1);
}
