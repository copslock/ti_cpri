/********************************************************************
* Copyright (C) 2012-2013 Texas Instruments Incorporated.
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
/** @file dfe_fl_cppDmaClose.c
 *
 *  @path  $(CSLPATH)\src\ip\dfe
 *
 *  @brief 
 *
 *  Description
 *  - 
 *
 */

#include <ti/drv/dfe/dfe_fl_cpp.h>
#include <ti/drv/dfe/dfe_fl_miscAux.h>

/** close prior opened dma */
/** ============================================================================
 *   @n@b dfeFl_CppDmaClose
 *
 *   @b Description
 *   @n Close prior opened Cpp/DMA channel, and free resources.
 *
 *   @b Arguments
 *   @verbatim
         hDma    CPP/DMA channel handle
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA channel handle
 *
 *   <b> Post Condition </b>
 *   @n  on-going DMA is aborted and the channel/trigger get freed.
 *
 *   @b Reads
 *   @n  CPP/DMA resource manager table
 *
 *   @b Writes
 *   @n  CPP/DMA resource manager table
 *
 *   @b Example
 *   @verbatim
        DfeFl_Status status;
        DfeFl_CppDmaHandle hDma;
        static DfeFl_CppResMgr myResMgr;
        
        memset(&myResMgr, 0, sizeof(myResMgr));
        for(i = 0; i < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; i++)
           myResMgr.discreteTrig[i] = DFE_FL_CPP_OPEN_ANY;
           
        hMisc = dfeFl_MiscOpen(...);
        
        // open channel#2 using EMBED mode, no triger destination.
        hDma = dfeFl_CppDmaOpen(hMisc, 
                   2,
                   DFE_FL_CPP_DMA_MODE_EMBED,
                   DFE_FL_CPP_OPEN_NONE,
                   &myResMgr,
                   &status);
        if(status != DFE_FL_SOK)
        {
            printf("dfeFl_CppDmaOpen error %d\n", status);
        }

        // do DMA transfer here
        
        
        // close
        dfeFl_CppDmaClose(hDma);
        
     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDmaClose
(
    DfeFl_CppDmaHandle hDma
)
{
    if(hDma == NULL)
        return;
        
    if(hDma->id < DFE_FL_CPP_NUM_DMA)
    {
        // make sure no more sync
        dfeFl_CppDmaDismissSync(hDma);
        
        // make sure abort
		dfeFl_CppDmaAbort(hDma);
            
		// wait till idle
		while(dfeFl_CppDmaGetBusy(hDma) > 0)
		{
			// spin
		}
        
        // disable DMA
        dfeFl_MiscCppSetDmaMode(hDma->hMisc, hDma->id, DFE_FL_CPP_DMA_MODE_DISABLE);
        // disable trigger 
        if(hDma->iTrig < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS)
        {
            dfeFl_MiscCppEnableDiscreteTrig(hDma->hMisc, hDma->iTrig, FALSE);
           
            // return trigger resource 
            hDma->resMgr->discreteTrig[hDma->iTrig] = DFE_FL_CPP_OPEN_ANY;
        }        
        // return dma resource
        CSL_FINSR(hDma->resMgr->dmaOpened, hDma->id, hDma->id, 0u);
    }
    
    hDma->id = DFE_FL_CPP_OPEN_NONE;
    hDma->mode = DFE_FL_CPP_DMA_MODE_DISABLE;
    hDma->iTrig = DFE_FL_CPP_OPEN_NONE;
    hDma->hMisc = NULL;
    hDma->resMgr = NULL;    
}
