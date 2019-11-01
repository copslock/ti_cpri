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
/** @file dfe_fl_cppDescripClose.c
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

/** close prior opened descriptor */
/** ============================================================================
 *   @n@b dfeFl_CppDescripClose
 *
 *   @b Description
 *   @n Close prior opened Cpp/DMA descriptor, and free resources.
 *
 *   @b Arguments
 *   @verbatim
         hDescrip    CPP/DMA descriptor handle
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA descriptor handle
 *
 *   <b> Post Condition </b>
 *   @n  CPP/DMA descriptor gets freed.
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
        DfeFl_CppDescriptorHandle hDescrip;
        static DfeFl_CppResMgr myResMgr;
        
        memset(&myResMgr, 0, sizeof(myResMgr));
        for(i = 0; i < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; i++)
           myResMgr.discreteTrig[i] = DFE_FL_CPP_OPEN_ANY;
           
        hMisc = dfeFl_MiscOpen(...);
        
        // open any channel using PROG mode, not triger destination.
        hDma = dfeFl_CppDmaOpen(hMisc, 
                   DFE_FL_CPP_OPEN_ANY,
                   DFE_FL_CPP_DMA_MODE_PROG,
                   DFE_FL_CPP_OPEN_NONE,
                   &myResMgr,
                   &status);
        if(status != DFE_FL_SOK)
        {
           printf("dfeFl_CppDmaOpen error %d\n", status);
           exit(-1);
        }

        // open any descriptor
        hDescrip = dfeFl_CppDecripOpen(hMisc,
                   DFE_FL_CPP_OPEN_ANY,
                   &myResMgr,
                   &status);
        if(status != DFE_FL_SOK)
        {
           printf("dfeFl_CppDecripOpen error %d\n", status);
           exit(-1);
        }
         
        // close
        dfeFl_CppDecripClose(hDescrip);
                    
     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDescripClose
(
    DfeFl_CppDescriptorHandle hDescrip
)
{
    uint32_t w, b;
    
    if(hDescrip == NULL)
    {
        return;  
    }
    
    // four 32-bits words, each bit corresponding to one descriptor
    w = hDescrip->id >> 5;
    b = hDescrip->id & 0x1Fu;
    
    // free descriptor
    CSL_FINSR(hDescrip->resMgr->descripOpened[w], b, b, 0u);

    hDescrip->id = DFE_FL_CPP_OPEN_NONE;
    hDescrip->hMisc = NULL;
    hDescrip->resMgr = NULL;
}


