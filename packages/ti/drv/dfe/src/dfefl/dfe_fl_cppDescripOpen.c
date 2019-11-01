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
/** @file dfe_fl_cppDescripOpen.c
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

/** open a CPP dma decriptor */
/** ============================================================================
 *   @n@b dfeFl_CppDecripOpen
 *
 *   @b Description
 *   @n open or allocate a CPP/DMA descriptor.
 *   @n When 0 <= descripId <= 127, the API try to open the specified descriptor. 
 *      If the descriptor already taken, DFE_FL_INUSE is returned.
 *      When descripId == DFE_FL_CPP_OPEN_ANY, the API looks for the first available
 *      descriptor and then opens it. If not found, DFE_FL_INUSE is returned.
 *   @n CPP/DMA resource manager, resMgr, is to keep tracking reserved, opened,
 *      free channels and/or descriptors. The caller should allocate and define
 *      this table per application.
 *
 *   @b Arguments
 *   @verbatim
         hMisc      Handle to DFE_MISC block
         descripId  Id of CPP/DMA descriptor to be opened
         resMgr   CPP/DMA resource manager
         status   DfeFl_Status buffer
     @endverbatim
 *
 *   <b> Return Value </b>  Handle to CPP/DMA descriptor, NULL if failed.
 *
 *   <b> Pre Condition </b>
 *   @n  DFE_MSIC has been opened perperly.
 *
 *   <b> Post Condition </b>
 *   @n  A CPP/DMA descriptor alloacted for the mode.
 *
 *   @b Reads
 *   @n  CPP/DMA resource manager, resMgr
 *
 *   @b Writes
 *   @n  CPP/DMA resource manager, resMgr
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
                    
                        
     @endverbatim
 * ===========================================================================
 */
DfeFl_CppDescriptorHandle dfeFl_CppDecripOpen
(
    DfeFl_MiscHandle   hMisc,
    uint32_t              descripId,
    DfeFl_CppResMgr    *resMgr,    
    DfeFl_Status          *status
)
{
    uint32_t data, w, b, id;
    DfeFl_CppDescriptorObj *descripObj = NULL;
    
    if(status == NULL)
        return NULL;
    
    *status = DFE_FL_SOK;
    
    // check parameters
    if(hMisc == NULL || resMgr == NULL)
        *status = DFE_FL_INVPARAMS;
        
    // check descripId
    if(*status == DFE_FL_SOK)        
    {
        
        if(descripId == DFE_FL_CPP_OPEN_ANY)
        {
            // find an available descriptor (not-reserved and not-opened)
            
            // four 32-bits words, each bit corresponding to one descriptor
            for(w = 0; w < 4; w++)
            {
                // a bit will be set (1) for opened or reserved descriptor
                data = resMgr->descripOpened[w] | resMgr->descripRsvd[w];
                
                for(b = 0; b < 32; b++)
                {
                    // a cleared bit (0) means available
                    if(CSL_FEXTR(data, b, b) == 0)
                    {
                        id = (w << 5) + b;
                        descripObj = &resMgr->descripTbl[id];
                        // descriptor to be opened
                        descripObj->id = id;
                        descripObj->hMisc = hMisc;
                        break;
                    }                    
                }
                
                // found?
                if(b < 32)
                    break;
            }
            
            // not found?
            if(w >= 4)
            {
                *status = DFE_FL_INUSE;
            }
        }
        else if(descripId >= DFE_FL_CPP_NUM_DESCRIPTORS)
        {
            *status = DFE_FL_INVPARAMS;
        }
        else
        {
            // four 32-bits words, each bit corresponding to one descriptor
            w = descripId >> 5;
            b = descripId & 0x1Fu;
            
            // a set bit (1) means not available
            if(CSL_FEXTR(resMgr->descripOpened[w], b, b) > 0 )
            {
                *status = DFE_FL_INUSE;
            }
            else
            {
                descripObj = &resMgr->descripTbl[descripId];
                descripObj->id = descripId;
                descripObj->hMisc = hMisc;
            }
        }                                    
    }

    if(*status != DFE_FL_SOK)
    {
        if(descripObj != NULL)
        {
            descripObj->id = DFE_FL_CPP_OPEN_NONE;
            descripObj->hMisc = NULL;
            descripObj->resMgr = NULL;
        }
        return NULL;
    }
    else
    {
        // flag resource opened
        w = descripObj->id >> 5;
        b = descripObj->id & 0x1Fu;
        
        CSL_FINSR(resMgr->descripOpened[w], b, b, 1u);
        
        // save resMgr
        descripObj->resMgr = resMgr;
        
        return (DfeFl_CppDescriptorHandle)descripObj;
    }
}


