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
/** @file dfe_fl_cppDmaControl.c
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


/** start a dma via sync,
 */
/** ============================================================================
 *   @n@b dfeFl_CppDmaArm
 *
 *   @b Description
 *   @n Arm or enable a CPP/DMA channel, then the coming selected sync will 
 *      start the channel to move data.
 *      For PROG mode, dmaStartAddr is the Id of head descriptor in the list.
 *      For EMBED mode, dmaStartAddr is IQN2 CTL channel# to be listened.
 *
 *   @b Arguments
 *   @verbatim
         hDma           CPP/DMA channel handle
         dmaStartAddr   DMA start address
         dmaSsel        DMA sync select
     @endverbatim
 *
 *   <b> Return Value </b>  DFE_FL_SOK
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA channel handle
 *
 *   <b> Post Condition </b>
 *   @n  CPP/DMA channel is armed and ready to run
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  CPP/DMA registers
 *
 *   @b Example
 *   @verbatim
        DfeFl_Status status;
        DfeFl_CppDmaHandle hDma;
        DfeFl_CppDescriptorHandle hDescrip;
        static DfeFl_CppResMgr myResMgr;
        DfeFl_CppDescripConfig descripCfg;
        
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

        // open any descriptor
        hDescrip = dfeFl_CppDecripOpen(hMisc,
                   DFE_FL_CPP_OPEN_ANY,
                   &myResMgr,
                   &status);

        // prepare the descriptor, read CB Buf#0 MSB
        memset(&descripCfg, 0, sizeof(descripCfg));
        descripCfg.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfeCb[0]->regs->capture_buffer_a_16msb);
        descripCfg.chanNum = IQN2_CTL_UL_CH0
        descripCfg.rw = DFE_FL_CPP_DMA_UL;
        descripCfg.numBytes = 16384;
        descripCfg.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
        descripCfg.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
        descripCfg.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
        descripCfg.midImm = 0;
        descripCfg.linkNext = 0; // change later
        dfeFl_CppDescripWrite(hDescrip, &descripCfg);
        dfeFl_CppDescripLink(hDescrip, hDescrip);
        
        // Arm CPP/DMA channel with MPU Sync
        status = dfeFl_CppDmaArm(hDma, dfeFl_CppDescripGetId(hDescrip), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC));
        
        // issue sync
        issueSync(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
        
     @endverbatim
 * ===========================================================================
 */
DfeFl_Status dfeFl_CppDmaArm
(
    DfeFl_CppDmaHandle hDma,
    uint32_t dmaStartAddr,
    uint32_t dmaSsel
)
{
    DfeFl_Status status = DFE_FL_SOK;
    
    dfeFl_MiscCppSetStartAddr(hDma->hMisc, hDma->id, dmaStartAddr);
    
    if(dmaSsel == DFE_FL_CPP_DMA_SSEL_MANUAL)
    {
        dfeFl_MiscCppSetSsel(hDma->hMisc, hDma->id, 0 /* DFE_FL_SSEL_NEVER */);
        dfeFl_MiscCppManualStart(hDma->hMisc, hDma->id);
    }
    else
    {
        dfeFl_MiscCppSetSsel(hDma->hMisc, hDma->id, dmaSsel);
    }
        
    return status;
}

/** Dismiss DMA sync */
/** ============================================================================
 *   @n@b dfeFl_CppDmaDismissSync
 *
 *   @b Description
 *   @n stop sensing sync signal for CPP/DMA channel.
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
 *   @n  The CPP/DMA channel handle not sense any sync anymore
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
        ...
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

        // prepare the descriptor
        
        // Arm CPP/DMA channel with MPU Sync
        status = dfeFl_CppDmaArm(hDma, dfeFl_CppDescripGetId(hDescrip), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC));
        
        // issue sync
        issueSync(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
        
        // dismiss
        dfeFl_CppDmaDismissSync(hDma);

     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDmaDismissSync
(
    DfeFl_CppDmaHandle hDma
)
{
    dfeFl_MiscCppSetSsel(hDma->hMisc, hDma->id, 0 /* DFE_FL_SSEL_NEVER */);
}

/** manual abort a dma */
/** ============================================================================
 *   @n@b dfeFl_CppDmaAbort
 *
 *   @b Description
 *   @n force to abort on-going CPP/DMA channel
 *
 *   @b Arguments
 *   @verbatim
         hDma    CPP/DMA channel handle
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA channel
 *
 *   <b> Post Condition </b>
 *   @n  CPP/DMA channel transferring aborted
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
        ... ...
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

        // prepare the descriptor
        
        // Arm CPP/DMA channel with MPU Sync
        status = dfeFl_CppDmaArm(hDma, dfeFl_CppDescripGetId(hDescrip), DFE_FL_CPP_DMA_SSEL_GSYNC(DFE_FL_SYNC_GEN_SIG_MPU_SYNC));
        
        // issue sync
        issueSync(DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_MISC_SYNC_WAITFOREVER);
        
        // abort on-going xfer
        dfeFl_CppDmaAbort(hDma);

     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDmaAbort
(
    DfeFl_CppDmaHandle hDma
)
{
    dfeFl_MiscCppManualAbort(hDma->hMisc, hDma->id);    
}
