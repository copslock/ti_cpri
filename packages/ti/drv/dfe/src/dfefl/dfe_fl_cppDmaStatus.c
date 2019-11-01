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
/** @file dfe_fl_cppDmaStatus.c
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

/** query if dma busy */
/** ============================================================================
 *   @n@b dfeFl_CppDmaGetBusy
 *
 *   @b Description
 *   @n Check if CPP/DMA channel is busy transferring.
 *
 *   @b Arguments
 *   @verbatim
         hDma    CPP/DMA channel handle
     @endverbatim
 *
 *   <b> Return Value </b>  
 *   @li 0, there's no on-going transfer
 *   @li 1, channel is busy, there's on-going transfer
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA channel
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
        
        hMisc = dfeFl_MiscOpen(...);
        
        // open any channel using PROG mode, not triger destination.
        hDma = dfeFl_CppDmaOpen(hMisc, 
                   DFE_FL_CPP_OPEN_ANY,
                   DFE_FL_CPP_DMA_MODE_PROG,
                   DFE_FL_CPP_OPEN_NONE,
                   &myResMgr,
                   &status);

        // open any descriptor
        hDescrip = dfeFl_CppDecripOpen(hMisc,
                   DFE_FL_CPP_OPEN_ANY,
                   &myResMgr,
                   &status);

        // prepare the descriptor, read CB Buf#0 MSB
        

        // Arm CPP/DMA channel with MPU Sync
        status = dfeFl_CppDmaArm(hDma, dfeFl_CppDescripGetId(hDescrip), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC));
        
        // issue sync
        issueSync(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
        
        // wait till complete
        while(dfeFl_CppDmaGetBusy(hDma) != 0) {  }
                   
     @endverbatim
 * ===========================================================================
 */
uint32_t dfeFl_CppDmaGetBusy
(
    DfeFl_CppDmaHandle hDma
)
{
    return dfeFl_MiscCppGetActiveStatus(hDma->hMisc, hDma->id);
}

/** return current dma Id */
/** ============================================================================
 *   @n@b dfeFl_CppDmaGetId
 *
 *   @b Description
 *   @n Get CPP/DMA channel Id from Handle.
 *
 *   @b Arguments
 *   @verbatim
         hDma    CPP/DMA channel handle
     @endverbatim
 *
 *   <b> Return Value </b>  CPP/DMA channel Id
 *
 *   <b> Pre Condition </b>
 *   @n  Valid CPP/DMA channel handle
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
 
         uint32_t dmaId = dfeFl_CppDmaGetId(hDma);
         
     @endverbatim
 * ===========================================================================
 */
uint32_t dfeFl_CppDmaGetId
(
    DfeFl_CppDmaHandle hDma
)
{
    return hDma->id;
}


