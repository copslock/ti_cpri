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
/** @file dfe_fl_cppDmaOpen.c
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

/** open a CPP dma device */
/** ============================================================================
 *   @n@b dfeFl_CppDmaOpen
 *
 *   @b Description
 *   @n Open a CPP/DMA channel for later use. 
 *   @n When 0 <= dmaId <= 31, the API try to open the specified channel. 
 *      If the channel already taken, DFE_FL_INUSE is returned. 
 *      When dmaId == DFE_FL_CPP_OPEN_ANY, the API looks for the first available
 *      channel and then opens it. If not found, DFE_FL_INUSE is returned.
 *   @n The CPP/DMA supports three modes,
 *          DFE_FL_CPP_DMA_MODE_DISABLE, the channel is disabled
 *          DFE_FL_CPP_DMA_MODE_PROG,  the channel moves data according to
 *                                      programmed descriptors in regsiter space
 *          DFE_FL_CPP_DMA_MODE_EMBED, the channel moves data according to 
 *                                      embeded descriptor at beginning of the 
 *                                      downlink data packet
 *   @n When CPP/DMA completes, it can send a trigger to drive other blocks
 *      of DFE, iTrig is to select one of eigth desitnations.
 *   @n CPP/DMA resource manager, resMgr, is to keep tracking reserved, opened,
 *      free channels and/or descriptors. The caller should allocate and define
 *      this table per application.
 *
 *   @b Arguments
 *   @verbatim
         hMisc    Handle to DFE_MISC block
         dmaId    Id of CPP/DMA channel to be opened
         mode     CPP/DMA operation mode
         iTrig    CPP/DMA completion trigger 
         resMgr   CPP/DMA resource manager
         status   DfeFl_Status buffer
     @endverbatim
 *
 *   <b> Return Value </b>  Handle to CPP/DMA channel, NULL if failed.
 *
 *   <b> Pre Condition </b>
 *   @n  DFE_MSIC has been opened perperly.
 *
 *   <b> Post Condition </b>
 *   @n  A CPP/DMA channel alloacted for the mode.
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
        }
         
     @endverbatim
 * ===========================================================================
 */
DfeFl_CppDmaHandle dfeFl_CppDmaOpen
(
    DfeFl_MiscHandle   hMisc,
    uint32_t              dmaId, 
    DfeFl_CppDmaMode   mode, 
    uint32_t              iTrig,
    DfeFl_CppResMgr    *resMgr,
    DfeFl_Status *status
)
{
    uint32_t data, i;
    DfeFl_CppDmaObj    *dmaObj = NULL;

    if(status == NULL)
        return NULL;
            
    *status = DFE_FL_SOK;
    
    // check arguments
    if(hMisc == NULL || mode >= DFE_FL_CPP_DMA_MODE_MAX || resMgr == NULL)
        *status = DFE_FL_INVPARAMS;
    
    // check dmaId
    if(*status == DFE_FL_SOK)        
    {
        if(dmaId == DFE_FL_CPP_OPEN_ANY)
        {
            // find an available dma (not-reserved and not-opened)
            
            // a bit will be set (1) for opened or reserved dma
            data = resMgr->dmaRsvd | resMgr->dmaOpened;
            
            // a 32-bits word, each bit corresponding to one dma
            for(i = 0; i < DFE_FL_CPP_NUM_DMA; i++)
            {
                // a cleared bit (0) means available
                if(CSL_FEXTR(data, i, i) == 0)
                {
                    dmaObj = &resMgr->dmaTbl[i];
                    dmaObj->id = i;
                    dmaObj->mode = mode;
                    dmaObj->hMisc = hMisc;
                    break;
                }
            }
            // not found?
            if(i >= 32)
            {
                *status = DFE_FL_INUSE;
            }
        }
        else if(dmaId >= DFE_FL_CPP_NUM_DMA)
        {
            *status = DFE_FL_INVPARAMS;
        }
        else
        {   
            data = resMgr->dmaOpened;
            // a set bit (1) means not available         
            if(CSL_FEXTR(data, dmaId, dmaId) > 0)
            {
                *status = DFE_FL_INUSE;
            }
            else
            {
                dmaObj = &resMgr->dmaTbl[dmaId];
                dmaObj->id = dmaId;
                dmaObj->mode = mode;
                dmaObj->hMisc = hMisc;
            }                
        }                
    }
    
    // check trigger
    if(*status == DFE_FL_SOK)
    {
        if(iTrig == DFE_FL_CPP_OPEN_ANY)
        {
            // find an trigger not occupied
            for(i = 0; i < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS; i++)
            {
                if(resMgr->discreteTrig[i] == DFE_FL_CPP_OPEN_ANY)
                {
                    dmaObj->iTrig = i;
                    break;
                }
            }
            // not found?
            if(i >= DFE_FL_CPP_NUM_DISCRETE_TRIGGERS)
            {
                *status = DFE_FL_INUSE;
            }
        }
        else if(iTrig == DFE_FL_CPP_OPEN_NONE)
        {
            dmaObj->iTrig = DFE_FL_CPP_OPEN_NONE;
        }
        else if(iTrig >= DFE_FL_CPP_NUM_DISCRETE_TRIGGERS)
        {
            *status = DFE_FL_INVPARAMS;
        }
        else 
        {
            // the requesting not occupied or it reserved for me
            if( (resMgr->discreteTrig[iTrig] == DFE_FL_CPP_OPEN_ANY)
              ||(resMgr->discreteTrig[iTrig] == dmaObj->id)
              )
            {
                dmaObj->iTrig = iTrig;
            }
            else
            {
                *status = DFE_FL_INUSE;    
            }
        }
    }
        
    if(*status != DFE_FL_SOK)
    {
        if(dmaObj != NULL)
        {
            dmaObj->hMisc = NULL;
            dmaObj->id = DFE_FL_CPP_OPEN_NONE;
            dmaObj->iTrig = DFE_FL_CPP_OPEN_NONE;
            dmaObj->mode = DFE_FL_CPP_DMA_MODE_DISABLE;
            dmaObj->resMgr = NULL;
        }
                
        return NULL;
    }
    else
    {
        // flag opened
        CSL_FINSR(resMgr->dmaOpened, dmaObj->id, dmaObj->id, 1u);

        // dma mode 
        dfeFl_MiscCppSetDmaMode(hMisc, dmaObj->id, dmaObj->mode);
                
        // discrete trigger
        if(dmaObj->iTrig < DFE_FL_CPP_NUM_DISCRETE_TRIGGERS)
        {
            resMgr->discreteTrig[dmaObj->iTrig] = dmaObj->id;
            
            dfeFl_MiscCppEnableDiscreteTrig(hMisc, dmaObj->iTrig, TRUE);
            dfeFl_MiscCppSetDiscreteTrigSel(hMisc, dmaObj->iTrig, dmaObj->id);
        }
        
        // save resMgr
        dmaObj->resMgr = resMgr;
        
        return dmaObj;
    }
}
