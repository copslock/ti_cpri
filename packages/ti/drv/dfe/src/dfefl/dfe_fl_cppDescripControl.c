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
/** @file dfe_fl_cppDescripControl.c
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

/** ============================================================================
 *   @n@b dfeFl_CppDescripWrite
 *
 *   @b Description
 *   @n Write configuration to CPP/DMA descriptor.
 *
 *   @b Arguments
 *   @verbatim
         hDescrip       CPP/DMA descriptor handle
         descripConfig  pointer to configuration
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  valid CPP/DMA descriptor handle
 *
 *   <b> Post Condition </b>
 *   @n  new configuration written to the descriptor
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  CPP/DMA descriptor registers
 *
 *   @b Example
 *   @verbatim
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
        
     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDescripWrite
(
    DfeFl_CppDescriptorHandle hDescrip,
    DfeFl_CppDescripConfig *descripConfig
)
{
    dfeFl_MiscCppWriteDescrip(hDescrip->hMisc, hDescrip->id, descripConfig);
}

/** ============================================================================
 *   @n@b dfeFl_CppDescripLink
 *
 *   @b Description
 *   @n Append descriptor#2 to descriptor#1, to construct a scatter-gather list.
 *      To end the list, the last descriptor links to itself.
 *
 *   @b Arguments
 *   @verbatim
         hDescrip1    CPP/DMA descriptor handle #1
         hDescrip2    CPP/DMA descriptor handle #2
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Both hDescrip1 and hDescrip2 are valid
 *
 *   <b> Post Condition </b>
 *   @n  descriptor#2 is appended to descriptor#1
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
        // open any descriptor
        hDescrip1 = dfeFl_CppDecripOpen(hMisc,
                   DFE_FL_CPP_OPEN_ANY,
                   &myResMgr,
                   &status);
        hDescrip2 = dfeFl_CppDecripOpen(hMisc,
                   DFE_FL_CPP_OPEN_ANY,
                   &myResMgr,
                   &status);

        // prepare the descriptor #1, read CB Buf#0 MSB
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
        
        // prepare the descriptor #2, read CB Buf#1 MSB
        memset(&descripCfg, 0, sizeof(descripCfg));
        descripCfg.mpuAddr = DFE_FL_CPP_ADDR_DSP2DFE((uint32_t)&hDfeCb[0]->regs->capture_buffer_b_16msb);
        descripCfg.chanNum = IQN2_CTL_UL_CH0
        descripCfg.rw = DFE_FL_CPP_DMA_UL;
        descripCfg.numBytes = 16384;
        descripCfg.ctlIncr = DFE_FL_CPP_CTL_INC_SIZE_4;
        descripCfg.mpuIncr = DFE_FL_CPP_MPU_INC_SIZE_4;
        descripCfg.pktSize = DFE_FL_CPP_DMA_PKT_SIZE_32K;
        descripCfg.midImm = 0;
        descripCfg.linkNext = 0; // change later
        dfeFl_CppDescripWrite(hDescrip, &descripCfg);
        
        // append hDescrip2 to hDescrip1
        dfeFl_CppDescripLink(hDescrip1, hDescrip2);
        // end the list
        dfeFl_CppDescripLink(hDescrip2, hDescrip2);
        
     @endverbatim
 * ===========================================================================
 */
void dfeFl_CppDescripLink
(
    DfeFl_CppDescriptorHandle hDescrip1,
    DfeFl_CppDescriptorHandle hDescrip2
)
{
    dfeFl_MiscCppSetDescripLink(hDescrip1->hMisc, hDescrip1->id, hDescrip2->id);
}
