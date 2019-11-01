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

/** @file dfe_fl_miscAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_MISC CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_MISCAUX_H_
#define _DFE_FL_MISCAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_misc.h>
#include <ti/drv/dfe/dfe_fl_cpp.h>

/** ============================================================================
 *   @n@b dfeFl_MiscConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         arg      [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CFG1_REG_MISC_INITS_STATE
 *       DFE_MISC_CFG1_REG_MISC_CLEAR_DATA
 *       DFE_MISC_CFG1_REG_MISC_INITS_CLK_GATE
 *       DFE_MISC_CFG1_REG_MISC_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_MiscConfigInits(DfeFl_MiscHandle hMisc, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hMisc->regs->cfg1;
    
    CSL_FINS(data, DFE_MISC_CFG1_REG_MISC_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_MISC_CFG1_REG_MISC_INITS_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_MISC_CFG1_REG_MISC_INITS_STATE, arg->initState);
    CSL_FINS(data, DFE_MISC_CFG1_REG_MISC_CLEAR_DATA, arg->clearData);
    
    hMisc->regs->cfg1 = data;
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscSetMemMpuAccess
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         setMask    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CFG2_REG_MEM_MPU_ACCESS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetMemMpuAccess(DfeFl_MiscHandle hMisc, uint32_t setMask)
{
    uint32_t data = hMisc->regs->cfg2;
    
     hMisc->regs->cfg2 = CSL_FMK(DFE_MISC_CFG2_REG_MEM_MPU_ACCESS, (data | setMask));
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscClearMemMpuAccess
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         clrMask    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CFG2_REG_MEM_MPU_ACCESS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMemMpuAccess(DfeFl_MiscHandle hMisc, uint32_t clrMask)
{
    uint32_t data = hMisc->regs->cfg2;
    
     hMisc->regs->cfg2 = CSL_FMK(DFE_MISC_CFG2_REG_MEM_MPU_ACCESS, (data & ~clrMask));
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppSetDmaMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
         dmaMode    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CPP_DMA_REG_DMA_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppSetDmaMode(DfeFl_MiscHandle hMisc, uint32_t dmaId, uint32_t dmaMode)
{
    CSL_FINS(hMisc->regs->cpp_dma[dmaId], DFE_MISC_CPP_DMA_REG_DMA_MODE, dmaMode);
}

/** dismiss sync to a dma, set ssel to NEVER */
CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppSetStartAddr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
         dmaStartAddr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CPP_DMA_REG_DMA_START_ADDRESS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppSetStartAddr(DfeFl_MiscHandle hMisc, uint32_t dmaId, uint32_t dmaStartAddr)
{
    CSL_FINS(hMisc->regs->cpp_dma[dmaId], DFE_MISC_CPP_DMA_REG_DMA_START_ADDRESS, dmaStartAddr);
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppSetSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
         dmaSsel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_CPP_DMA_REG_DMA_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppSetSsel(DfeFl_MiscHandle hMisc, uint32_t dmaId, uint32_t dmaSsel)
{
    CSL_FINS(hMisc->regs->cpp_dma[dmaId], DFE_MISC_CPP_DMA_REG_DMA_SSEL, dmaSsel);
}

/** manual start a dma */
CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppManualStart
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppManualStart(DfeFl_MiscHandle hMisc, uint32_t dmaId)
{
    uint32_t data = 0;
    CSL_DFE_MISC_CPP_CFG11_REG *cfg = (CSL_DFE_MISC_CPP_CFG11_REG *)&data;
    
    // write '0' to ctl_dma_start    
    cfg->ctl_dma_start = 0;
    cfg->ctl_dma_abort = 0;
    cfg->ctl_dma_process = dmaId;

    hMisc->regs->cpp_cfg11 = data;
    
    // write '1' to ctl_dma_start    
    cfg->ctl_dma_start = 1;
    hMisc->regs->cpp_cfg11 = data;

    // write '0' to ctl_dma_start    
    cfg->ctl_dma_start = 0;
    hMisc->regs->cpp_cfg11 = data;
}

/** manual abort a dma */
CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppManualAbort
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppManualAbort(DfeFl_MiscHandle hMisc, uint32_t dmaId)
{
    uint32_t data = 0;
    CSL_DFE_MISC_CPP_CFG11_REG *cfg = (CSL_DFE_MISC_CPP_CFG11_REG *)&data;
    
    // write '0' to ctl_dma_abort
    cfg->ctl_dma_abort = 0;
    cfg->ctl_dma_start = 0;
    cfg->ctl_dma_process = dmaId;
    
    hMisc->regs->cpp_cfg11 = data;

    // write '1' to ctl_dma_abort
    cfg->ctl_dma_abort = 1;
    hMisc->regs->cpp_cfg11 = data;

    // write '0' to ctl_dma_abort
    cfg->ctl_dma_abort = 0;
    hMisc->regs->cpp_cfg11 = data;
}

/** query if dma busy */
CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppGetActiveStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaId    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  [add content]
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
uint32_t dfeFl_MiscCppGetActiveStatus(DfeFl_MiscHandle hMisc, uint32_t dmaId)
{
    return CSL_FEXTR(hMisc->regs->cpp_cfg12, dmaId, dmaId);
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppEnableDiscreteTrig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaTrig    [add content]
         enable    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppEnableDiscreteTrig(DfeFl_MiscHandle hMisc, uint32_t dmaTrig, uint32_t enable)
{
    CSL_FINSR(hMisc->regs->cpp_cfg19, dmaTrig, dmaTrig, enable);
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_MiscCppSetDiscreteTrigSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dmaTrig    [add content]
         dmaId    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppSetDiscreteTrigSel(DfeFl_MiscHandle hMisc, uint32_t dmaTrig, uint32_t dmaId)
{
    uint32_t r = dmaTrig / 4;
    uint32_t b = (dmaTrig & 0x3u) * 8;
    volatile uint32_t *regs = &hMisc->regs->cpp_cfg20;
    CSL_FINSR(regs[r], b+4, b, dmaId);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscCppReadDescrip
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         descripId    [add content]
         descripConfig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppReadDescrip(DfeFl_MiscHandle hMisc, uint32_t descripId, DfeFl_CppDescripConfig *descripConfig)
{
    uint32_t data;
    CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM0_REG *mem0 = (CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM0_REG *)&data;
    CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *mem1 = (CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *)&data;
    
    // mem0
    data = hMisc->regs->cpp_dma_descriptor[descripId].mem0;
    descripConfig->mpuAddr = mem0->dma_descriptor_mpu_address;    
    descripConfig->chanNum = mem0->dma_descriptor_channel_num;
    descripConfig->rw = mem0->dma_descriptor_rwb;
    
    // mem1
    data = hMisc->regs->cpp_dma_descriptor[descripId].mem1;
    descripConfig->numBytes = mem1->dma_descriptor_num_bytes;
    descripConfig->ctlIncr = mem1->dma_descriptor_bytestoxfer;
    descripConfig->mpuIncr = mem1->dma_descriptor_incperxfer;
    descripConfig->pktSize = mem1->dma_descriptor_pktsize;
    descripConfig->midImm = mem1->dma_descriptor_midpkt;
    descripConfig->linkNext = mem1->dma_descriptor_linklistnext;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscCppWriteDescrip
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         descripId    [add content]
         descripConfig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppWriteDescrip(DfeFl_MiscHandle hMisc, uint32_t descripId, DfeFl_CppDescripConfig *descripConfig)
{
    uint32_t data = 0;
    CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM0_REG *mem0 = (CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM0_REG *)&data;
    CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *mem1 = (CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *)&data;
    
    // mem0
    mem0->dma_descriptor_mpu_address = descripConfig->mpuAddr;    
    mem0->dma_descriptor_channel_num = descripConfig->chanNum;
    mem0->dma_descriptor_rwb = descripConfig->rw;
    hMisc->regs->cpp_dma_descriptor[descripId].mem0 = data;
    
    // mem1
    data = 0;
    mem1->dma_descriptor_num_bytes = descripConfig->numBytes;
    mem1->dma_descriptor_bytestoxfer = descripConfig->ctlIncr;
    mem1->dma_descriptor_incperxfer = descripConfig->mpuIncr;
    mem1->dma_descriptor_pktsize = descripConfig->pktSize;
    mem1->dma_descriptor_midpkt = descripConfig->midImm;
    mem1->dma_descriptor_linklistnext = descripConfig->linkNext;
    hMisc->regs->cpp_dma_descriptor[descripId].mem1 = data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscCppSetDescripLink
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         descripId1    [add content]
         descripId2    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscCppSetDescripLink(DfeFl_MiscHandle hMisc, uint32_t descripId1, uint32_t descripId2)
{
    uint32_t data = 0;
    CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *mem1 = (CSL_DFE_MISC_CPP_DMA_DESCRIPTOR_MEM1_REG *)&data;
    
    data = hMisc->regs->cpp_dma_descriptor[descripId1].mem1;
    mem1->dma_descriptor_linklistnext = descripId2;
    hMisc->regs->cpp_dma_descriptor[descripId1].mem1 = data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenSetMpuSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         mpu    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_MPU_SYNC_REG_MPU_SYNC
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenSetMpuSync(DfeFl_MiscHandle hMisc, uint32_t mpu)
{
    CSL_FINS(hMisc->regs->sync_gen_mpu_sync, DFE_MISC_SYNC_GEN_MPU_SYNC_REG_MPU_SYNC, mpu);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenSetOneShot
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
         cycles    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_GPIO_SYNC_IN0
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_GPIO_SYNC_IN1
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_MASTER_INTR0
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_MASTER_INTR1
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_DL_IQ0_FRAME_START_SYNC0
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_DL_IQ0_FRAME_START_SYNC1
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR0
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR1
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR2
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_SYSREF
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_UL_IQ0_FRAME_STROBE_SYNC1
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_UL_IQ0_FRAME_STROBE_SYNC0
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_JESD_SYNC_IN
 *       DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_MPU_SYNC
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenSetOneShot(DfeFl_MiscHandle hMisc, uint32_t syncSig, uint32_t cycles)
{
    switch(syncSig)
    {
    case DFE_FL_SYNC_GEN_SIG_MPU_SYNC:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_MPU_SYNC, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_UL_IQ0_FRAME_STROBE_SYNC0, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_UL_IQ0_FRAME_STROBE_SYNC1, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_DL_IQ0_FRAME_START_SYNC0, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_DL_IQ0_FRAME_START_SYNC1, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_GPIO_SYNC_IN0, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_GPIO_SYNC_IN1, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_0_7, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_0_7_REG_ONE_SHOT_CTRL_JESD_SYNC_IN, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_SYSREF:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_SYSREF, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_MASTER_INTR0:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_MASTER_INTR0, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_MASTER_INTR1:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_MASTER_INTR1, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR0, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR1, cycles);
        break;
    case DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2:
        CSL_FINS(hMisc->regs->sync_gen_one_shot_ctrl_8_13, DFE_MISC_SYNC_GEN_ONE_SHOT_CTRL_8_13_REG_ONE_SHOT_CTRL_CNTR2, cycles);
        break;
    default:
    	return;
    }    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenSetUlSigFstrobe
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         ulSig    [add content]
         strobe    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_UL_IQ0_FRAME_STROBE_SYNC0_SEL
 *       DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_UL_IQ0_FRAME_STROBE_SYNC1_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenSetUlSigFstrobe(DfeFl_MiscHandle hMisc, uint32_t ulSig, uint32_t strobe)
{
    if(ulSig == DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0)
    {
        CSL_FINS(hMisc->regs->sync_gen_iq0_sync_ch_sel, DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_UL_IQ0_FRAME_STROBE_SYNC0_SEL, strobe);
    }
    else
    {
        CSL_FINS(hMisc->regs->sync_gen_iq0_sync_ch_sel, DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_UL_IQ0_FRAME_STROBE_SYNC1_SEL, strobe);
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenSetDlSigFstart
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         dlSig    [add content]
         dlCh    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_DL_IQ0_FRAME_START_SYNC1_CH
 *       DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_DL_IQ0_FRAME_START_SYNC0_CH
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenSetDlSigFstart(DfeFl_MiscHandle hMisc, uint32_t dlSig, uint32_t dlCh)
{
    if(dlSig == DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0)
    {
        CSL_FINS(hMisc->regs->sync_gen_iq0_sync_ch_sel, DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_DL_IQ0_FRAME_START_SYNC0_CH, dlCh);
    }
    else
    {
        CSL_FINS(hMisc->regs->sync_gen_iq0_sync_ch_sel, DFE_MISC_SYNC_GEN_IQ0_SYNC_CH_SEL_REG_DL_IQ0_FRAME_START_SYNC1_CH, dlCh);
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableSyncIntr(DfeFl_MiscHandle hMisc, uint32_t syncSig)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r0, syncSig, syncSig, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableSyncIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableSyncIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER, intrGrp->never);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, intrGrp->mpuSync);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, intrGrp->ulIq0FStrobeSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, intrGrp->ulIq0FStrobeSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, intrGrp->dlIq0FStartSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, intrGrp->dlIq0FStartSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, intrGrp->gpioSyncIn0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, intrGrp->gpioSyncIn1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, intrGrp->jesdSyncIn);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF, intrGrp->sysref);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, intrGrp->masterIntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, intrGrp->masterIntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, intrGrp->syncGenCntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, intrGrp->syncGenCntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, intrGrp->syncGenCntr2);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS, intrGrp->always);
    
    hMisc->regs->misc_intr_mask_r0 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableSyncIntr(DfeFl_MiscHandle hMisc, uint32_t syncSig)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r0, syncSig, syncSig, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableSyncIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableSyncIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER, intrGrp->never);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, intrGrp->mpuSync);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, intrGrp->ulIq0FStrobeSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, intrGrp->ulIq0FStrobeSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, intrGrp->dlIq0FStartSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, intrGrp->dlIq0FStartSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, intrGrp->gpioSyncIn0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, intrGrp->gpioSyncIn1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, intrGrp->jesdSyncIn);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF, intrGrp->sysref);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, intrGrp->masterIntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, intrGrp->masterIntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, intrGrp->syncGenCntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, intrGrp->syncGenCntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, intrGrp->syncGenCntr2);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS, intrGrp->always);

    hMisc->regs->misc_intr_mask_r0 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceSyncIntr(DfeFl_MiscHandle hMisc, uint32_t syncSig)
{
    CSL_FINSR(hMisc->regs->misc_intr_mask_r6, syncSig, syncSig, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceSyncIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceSyncIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER, intrGrp->never);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, intrGrp->mpuSync);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, intrGrp->ulIq0FStrobeSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, intrGrp->ulIq0FStrobeSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, intrGrp->dlIq0FStartSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, intrGrp->dlIq0FStartSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, intrGrp->gpioSyncIn0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, intrGrp->gpioSyncIn1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, intrGrp->jesdSyncIn);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF, intrGrp->sysref);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, intrGrp->masterIntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, intrGrp->masterIntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, intrGrp->syncGenCntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, intrGrp->syncGenCntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, intrGrp->syncGenCntr2);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS, intrGrp->always);
    
    hMisc->regs->misc_intr_mask_r6 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceSyncIntr(DfeFl_MiscHandle hMisc, uint32_t syncSig)
{
    CSL_FINSR(hMisc->regs->misc_intr_mask_r6, syncSig, syncSig, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceSyncIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceSyncIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER, intrGrp->never);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, intrGrp->mpuSync);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, intrGrp->ulIq0FStrobeSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, intrGrp->ulIq0FStrobeSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, intrGrp->dlIq0FStartSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, intrGrp->dlIq0FStartSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, intrGrp->gpioSyncIn0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, intrGrp->gpioSyncIn1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, intrGrp->jesdSyncIn);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF, intrGrp->sysref);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, intrGrp->masterIntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, intrGrp->masterIntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, intrGrp->syncGenCntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, intrGrp->syncGenCntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, intrGrp->syncGenCntr2);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS, intrGrp->always);

    hMisc->regs->misc_intr_mask_r6 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearSyncIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearSyncIntrStatus(DfeFl_MiscHandle hMisc, uint32_t syncSig)
{
    uint32_t b = (1u << syncSig);
    
    // write '0' to clear
    hMisc->regs->misc_intr_mask_r3 = ~b;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearSyncIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearSyncIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER, intrGrp->never);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, intrGrp->mpuSync);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, intrGrp->ulIq0FStrobeSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, intrGrp->ulIq0FStrobeSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, intrGrp->dlIq0FStartSync0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, intrGrp->dlIq0FStartSync1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, intrGrp->gpioSyncIn0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, intrGrp->gpioSyncIn1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, intrGrp->jesdSyncIn);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF, intrGrp->sysref);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, intrGrp->masterIntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, intrGrp->masterIntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, intrGrp->syncGenCntr0);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, intrGrp->syncGenCntr1);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, intrGrp->syncGenCntr2);
    CSL_FINSR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS, intrGrp->always);

    hMisc->regs->misc_intr_mask_r3 = ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetSyncIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncSig    [add content]
         status    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetSyncIntrStatus(DfeFl_MiscHandle hMisc, uint32_t syncSig, uint32_t *status)
{
    uint32_t data = hMisc->regs->misc_intr_mask_r3;
        
    *status = CSL_FEXTR(data, syncSig, syncSig);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetSyncIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetSyncIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncIntrGroup *intrGrp)
{
    uint32_t data = hMisc->regs->misc_intr_mask_r3;
    
    intrGrp->never = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_NEVER, DFE_FL_SYNC_GEN_SIG_NEVER);
    intrGrp->mpuSync = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_MPU_SYNC, DFE_FL_SYNC_GEN_SIG_MPU_SYNC);
    intrGrp->ulIq0FStrobeSync0 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC0);
    intrGrp->ulIq0FStrobeSync1 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1, DFE_FL_SYNC_GEN_SIG_UL_IQ0_FSTROBE_SYNC1);
    intrGrp->dlIq0FStartSync0 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC0);
    intrGrp->dlIq0FStartSync1 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1, DFE_FL_SYNC_GEN_SIG_DL_IQ0_FSTART_SYNC1);
    intrGrp->gpioSyncIn0 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN0);
    intrGrp->gpioSyncIn1 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1, DFE_FL_SYNC_GEN_SIG_GPIO_SYNC_IN1);
    intrGrp->jesdSyncIn = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN, DFE_FL_SYNC_GEN_SIG_JESD_SYNC_IN);
    intrGrp->sysref = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_SYSREF, DFE_FL_SYNC_GEN_SIG_SYSREF);
    intrGrp->masterIntr0 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0, DFE_FL_SYNC_GEN_SIG_MASTER_INTR0);
    intrGrp->masterIntr1 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1, DFE_FL_SYNC_GEN_SIG_MASTER_INTR1);
    intrGrp->syncGenCntr0 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR0);
    intrGrp->syncGenCntr1 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR1);
    intrGrp->syncGenCntr2 = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2, DFE_FL_SYNC_GEN_SIG_SYNC_GEN_CNTR2);
    intrGrp->always = CSL_FEXTR(data, DFE_FL_SYNC_GEN_SIG_ALWAYS, DFE_FL_SYNC_GEN_SIG_ALWAYS);
    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenResetCounter
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         cntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PERIOD_REG_SYNC_CNTR0_PERIOD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenResetCounter(DfeFl_MiscHandle hMisc, uint32_t cntr)
{
    volatile uint32_t *regs;
    
    switch(cntr)
    {
    case DFE_FL_SYNC_GEN_CNTR0:
        regs = &hMisc->regs->sync_gen_cntr0_sync_gen_cntr0_period;
        break;
    case DFE_FL_SYNC_GEN_CNTR1:
        regs = &hMisc->regs->sync_gen_cntr1_sync_gen_cntr1_period;
        break;
    case DFE_FL_SYNC_GEN_CNTR2:
        regs = &hMisc->regs->sync_gen_cntr2_sync_gen_cntr2_period;
        break;
    default:
    	return;
    }
    
    // period
    regs[0] = CSL_FMK(DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PERIOD_REG_SYNC_CNTR0_PERIOD, 0);    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenConfigCounter
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         cfg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_INVERT
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PERIOD_REG_SYNC_CNTR0_PERIOD
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_RPT
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_DELAY_REG_SYNC_CNTR0_DELAY
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PULSE_REG_SYNC_CNTR0_PULSE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenConfigCounter(DfeFl_MiscHandle hMisc, DfeFl_MiscSyncCntrConfig *cfg)
{
    volatile uint32_t *regs;
    uint32_t data;
    
    switch(cfg->cntr)
    {
    case DFE_FL_SYNC_GEN_CNTR0:
        regs = &hMisc->regs->sync_gen_cntr0_sync_gen_cntr0_ctrl;
        break;
    case DFE_FL_SYNC_GEN_CNTR1:
        regs = &hMisc->regs->sync_gen_cntr1_sync_gen_cntr1_ctrl;
        break;
    case DFE_FL_SYNC_GEN_CNTR2:
        regs = &hMisc->regs->sync_gen_cntr2_sync_gen_cntr2_ctrl;
        break;
    default:
    	return;
    }
    
    // rpt, invert
    data = regs[0];
    CSL_FINS(data, DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_RPT, cfg->repeat);
    CSL_FINS(data, DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_INVERT, cfg->invert);
    regs[0] = data;
    
    // period
    CSL_FINS(regs[1], DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PERIOD_REG_SYNC_CNTR0_PERIOD, cfg->period);
    
    // delay
    CSL_FINS(regs[2], DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_DELAY_REG_SYNC_CNTR0_DELAY, cfg->delay);

    // pulse
    CSL_FINS(regs[3], DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_PULSE_REG_SYNC_CNTR0_PULSE, cfg->pulse);    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSyncGenSetCounterSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         cntr    [add content]
         startSsel    [add content]
         progSsel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_SYNC_SEL
 *       DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_SSEL_UPDATE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSyncGenSetCounterSsel(DfeFl_MiscHandle hMisc, uint32_t cntr, uint32_t startSsel, uint32_t progSsel)
{
    volatile uint32_t *regs;
    uint32_t data;
        
    switch(cntr)
    {
    case DFE_FL_SYNC_GEN_CNTR0:
        regs = &hMisc->regs->sync_gen_cntr0_sync_gen_cntr0_ctrl;
        break;
    case DFE_FL_SYNC_GEN_CNTR1:
        regs = &hMisc->regs->sync_gen_cntr1_sync_gen_cntr1_ctrl;
        break;
    case DFE_FL_SYNC_GEN_CNTR2:
        regs = &hMisc->regs->sync_gen_cntr2_sync_gen_cntr2_ctrl;
        break;
    default:
    	return;
    }
    
    data = regs[0];
    CSL_FINS(data, DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_SYNC_SEL, startSsel);
    CSL_FINS(data, DFE_MISC_SYNC_GEN_CNTR0_SYNC_GEN_CNTR0_CTRL_REG_SYNC_CNTR0_SSEL_UPDATE_SSEL, progSsel);    
    
    regs[0] = data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioSetPinMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         pin    [add content]
         mux    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL12
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL13
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL16
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL17
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL14
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL15
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL4
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL5
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL2
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL3
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL0
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL1
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL7
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL6
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL9
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL8
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL11
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL10
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioSetPinMux(DfeFl_MiscHandle hMisc, uint32_t pin, uint32_t mux)
{
    switch(pin)
    {
    case 0:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL0, mux);
        break;        
    case 1:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL1, mux);
        break;        
    case 2:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL2, mux);
        break;        
    case 3:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL3, mux);
        break;        
    case 4:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL4, mux);
        break;        
    case 5:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map3, DFE_MISC_GPIO_CNTRL_GPIO_MAP3_REG_GPIO_MUX_SEL5, mux);
        break;        
    case 6:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL6, mux);
        break;        
    case 7:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL7, mux);
        break;        
    case 8:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL8, mux);
        break;        
    case 9:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL9, mux);
        break;        
    case 10:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL10, mux);
        break;        
    case 11:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map4, DFE_MISC_GPIO_CNTRL_GPIO_MAP4_REG_GPIO_MUX_SEL11, mux);
        break;        
    case 12:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL12, mux);
        break;        
    case 13:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL13, mux);
        break;        
    case 14:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL14, mux);
        break;        
    case 15:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL15, mux);
        break;        
    case 16:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL16, mux);
        break;        
    case 17:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map5, DFE_MISC_GPIO_CNTRL_GPIO_MAP5_REG_GPIO_MUX_SEL17, mux);
        break;
    default:
    	return;
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioSetSyncOutSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         syncout    [add content]
         ssel    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP2_REG_GPIO_SYNC_OUT_SSEL0
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP2_REG_GPIO_SYNC_OUT_SSEL1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioSetSyncOutSsel(DfeFl_MiscHandle hMisc, uint32_t syncout, uint32_t ssel)
{
    switch(syncout)
    {
    case 0:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map2, DFE_MISC_GPIO_CNTRL_GPIO_MAP2_REG_GPIO_SYNC_OUT_SSEL0, ssel);
        break;
    case 1:
        CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map2, DFE_MISC_GPIO_CNTRL_GPIO_MAP2_REG_GPIO_SYNC_OUT_SSEL1, ssel);
        break;
    default:
    	return;
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioSetGpioBank
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         gpio    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP1_REG_MPU_GPIO_DRIVE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioSetGpioBank(DfeFl_MiscHandle hMisc, uint32_t gpio)
{
    CSL_FINS(hMisc->regs->gpio_cntrl_gpio_map1, DFE_MISC_GPIO_CNTRL_GPIO_MAP1_REG_MPU_GPIO_DRIVE, gpio);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioGetGpioBank
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         gpio    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  see below,
 *       DFE_MISC_GPIO_CNTRL_GPIO_MAP0_REG_MPU_GPIO_READ
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioGetGpioBank(DfeFl_MiscHandle hMisc, uint32_t *gpio)
{
    *gpio = CSL_FEXT(hMisc->regs->gpio_cntrl_gpio_map0, DFE_MISC_GPIO_CNTRL_GPIO_MAP0_REG_MPU_GPIO_READ);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioSetGpioPin
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         pin    [add content]
         value    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioSetGpioPin(DfeFl_MiscHandle hMisc, uint32_t pin, uint32_t value)
{
    CSL_FINSR(hMisc->regs->gpio_cntrl_gpio_map1, pin, pin, value);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGpioGetGpioPin
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         pin    [add content]
         value    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGpioGetGpioPin(DfeFl_MiscHandle hMisc, uint32_t pin, uint32_t *value)
{
    *value = CSL_FEXTR(hMisc->regs->gpio_cntrl_gpio_map0, pin, pin);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscArbiterSetCmd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         data    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_ARBITER_CFG_ARB2_REG_ARBITER_CMD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscArbiterSetCmd(DfeFl_MiscHandle hMisc, uint32_t data)
{
    CSL_FINS(hMisc->regs->arbiter_cfg_arb2, DFE_MISC_ARBITER_CFG_ARB2_REG_ARBITER_CMD, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscArbiterSetCmdPar
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         data    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_ARBITER_CFG_ARB3_REG_ARBITER_CMD_PARAMS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscArbiterSetCmdPar(DfeFl_MiscHandle hMisc, uint32_t data)
{
    CSL_FINS(hMisc->regs->arbiter_cfg_arb3, DFE_MISC_ARBITER_CFG_ARB3_REG_ARBITER_CMD_PARAMS, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDvgaSetEnable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         enable    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_DVGA_DVGA0_REG_DVGA_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDvgaSetEnable(DfeFl_MiscHandle hMisc, uint32_t enable)
{
    CSL_FINS(hMisc->regs->dvga_dvga0, DFE_MISC_DVGA_DVGA0_REG_DVGA_EN, enable);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDvgaConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         cfg    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  see below,
 *       DFE_MISC_DVGA_DVGA7_REG_DVGA_LE0_WIDTH_M1
 *       DFE_MISC_DVGA_DVGA6_REG_DVGA_LE0_POLARITY
 *       DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL5
 *       DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL4
 *       DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL7
 *       DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL6
 *       DFE_MISC_DVGA_DVGA6_REG_DVGA_LE3_POLARITY
 *       DFE_MISC_DVGA_DVGA7_REG_DVGA_LE1_WIDTH_M1
 *       DFE_MISC_DVGA_DVGA0_REG_DVGA_MODE
 *       DFE_MISC_DVGA_DVGA5_REG_DVGA_NUM_STRS_M1
 *       DFE_MISC_DVGA_DVGA6_REG_DVGA_LE2_POLARITY
 *       DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL2
 *       DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL3
 *       DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL0
 *       DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL1
 *       DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL7
 *       DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL6
 *       DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL5
 *       DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL4
 *       DFE_MISC_DVGA_DVGA8_REG_DVGA_LE2_WIDTH_M1
 *       DFE_MISC_DVGA_DVGA8_REG_DVGA_LE3_WIDTH_M1
 *       DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL0
 *       DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL1
 *       DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL2
 *       DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL3
 *       DFE_MISC_DVGA_DVGA6_REG_DVGA_LE1_POLARITY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDvgaConfig(DfeFl_MiscHandle hMisc, DfeFl_MiscDvgaConfig *cfg)
{
    CSL_FINS(hMisc->regs->dvga_dvga0, DFE_MISC_DVGA_DVGA0_REG_DVGA_MODE, cfg->dvgaMode);
    
    if(cfg->dvgaMode == DFE_FL_MISC_DVGA_MODE_TRANSPARENT)
    {
        hMisc->regs->dvga_dvga1 = CSL_FMK(DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL0, cfg->un.trModeCfg.streamSel[0])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL1, cfg->un.trModeCfg.streamSel[1])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL2, cfg->un.trModeCfg.streamSel[2])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA1_REG_DVGA_TR_MODE_STR_SEL3, cfg->un.trModeCfg.streamSel[3]);
                                
        hMisc->regs->dvga_dvga2 = CSL_FMK(DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL4, cfg->un.trModeCfg.streamSel[4])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL5, cfg->un.trModeCfg.streamSel[5])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL6, cfg->un.trModeCfg.streamSel[6])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA2_REG_DVGA_TR_MODE_STR_SEL7, cfg->un.trModeCfg.streamSel[7]);
                                
        hMisc->regs->dvga_dvga3 = CSL_FMK(DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL0, cfg->un.trModeCfg.bitSel[0])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL1, cfg->un.trModeCfg.bitSel[1])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL2, cfg->un.trModeCfg.bitSel[2])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA3_REG_DVGA_TR_MODE_BIT_SEL3, cfg->un.trModeCfg.bitSel[3]);
                                
        hMisc->regs->dvga_dvga4 = CSL_FMK(DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL4, cfg->un.trModeCfg.bitSel[4])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL5, cfg->un.trModeCfg.bitSel[5])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL6, cfg->un.trModeCfg.bitSel[6])
                                | CSL_FMK(DFE_MISC_DVGA_DVGA4_REG_DVGA_TR_MODE_BIT_SEL7, cfg->un.trModeCfg.bitSel[7]);                                
    }
    else if(cfg->dvgaMode == DFE_FL_MISC_DVGA_MODE_CLOCKED)
    {
        hMisc->regs->dvga_dvga5 = CSL_FMK(DFE_MISC_DVGA_DVGA5_REG_DVGA_NUM_STRS_M1, cfg->un.clockModeCfg.numStrsM1);
        hMisc->regs->dvga_dvga6 = CSL_FMK(DFE_MISC_DVGA_DVGA6_REG_DVGA_LE0_POLARITY, cfg->un.clockModeCfg.lePolarity[0])
                            | CSL_FMK(DFE_MISC_DVGA_DVGA6_REG_DVGA_LE1_POLARITY, cfg->un.clockModeCfg.lePolarity[1])
                            | CSL_FMK(DFE_MISC_DVGA_DVGA6_REG_DVGA_LE2_POLARITY, cfg->un.clockModeCfg.lePolarity[2])
                            | CSL_FMK(DFE_MISC_DVGA_DVGA6_REG_DVGA_LE3_POLARITY, cfg->un.clockModeCfg.lePolarity[3]);
         
        hMisc->regs->dvga_dvga7 = CSL_FMK(DFE_MISC_DVGA_DVGA7_REG_DVGA_LE0_WIDTH_M1, cfg->un.clockModeCfg.leWidthM1[0])
                            | CSL_FMK(DFE_MISC_DVGA_DVGA7_REG_DVGA_LE1_WIDTH_M1, cfg->un.clockModeCfg.leWidthM1[1]);
                            
        hMisc->regs->dvga_dvga8 = CSL_FMK(DFE_MISC_DVGA_DVGA8_REG_DVGA_LE2_WIDTH_M1, cfg->un.clockModeCfg.leWidthM1[2])
                            | CSL_FMK(DFE_MISC_DVGA_DVGA8_REG_DVGA_LE3_WIDTH_M1, cfg->un.clockModeCfg.leWidthM1[3]);
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableCppDmaDoneIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableCppDmaDoneIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r1, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableCppDmaDoneIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableCppDmaDoneIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = 0;
    
    for(ui = 0; ui < 32; ui++)
    {
        CSL_FINSR(data, ui, ui, intrGrp->dmaDone[ui]);
    }
    
    hMisc->regs->misc_intr_mask_r1 |= data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableCppDmaDoneIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableCppDmaDoneIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r1, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableCppDmaDoneIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableCppDmaDoneIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = 0;
    
    for(ui = 0; ui < 32; ui++)
    {
        CSL_FINSR(data, ui, ui, intrGrp->dmaDone[ui]);
    }
    
    hMisc->regs->misc_intr_mask_r1 &= ~data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceCppDmaDoneIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceCppDmaDoneIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r7, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceCppDmaDoneIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceCppDmaDoneIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = 0;
    
    for(ui = 0; ui < 32; ui++)
    {
        CSL_FINSR(data, ui, ui, intrGrp->dmaDone[ui]);
    }
    
    hMisc->regs->misc_intr_mask_r7 |= data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceCppDmaDoneIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceCppDmaDoneIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r7, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceCppDmaDoneIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceCppDmaDoneIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = 0;
    
    for(ui = 0; ui < 32; ui++)
    {
        CSL_FINSR(data, ui, ui, intrGrp->dmaDone[ui]);
    }
    
    hMisc->regs->misc_intr_mask_r7 &= ~data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearCppDmaDoneIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearCppDmaDoneIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr)
{
    uint32_t b = (1u << intr);
    
    // write '0' to clear
    hMisc->regs->misc_intr_mask_r4 = ~b;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearCppDmaDoneIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearCppDmaDoneIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = 0;
    
    for(ui = 0; ui < 32; ui++)
    {
        CSL_FINSR(data, ui, ui, intrGrp->dmaDone[ui]);
    }
    
    hMisc->regs->misc_intr_mask_r4 = ~data;    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetCppDmaDoneIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
         status    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetCppDmaDoneIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr, uint32_t *status)
{
    uint32_t data = hMisc->regs->misc_intr_mask_r4;
        
    *status = CSL_FEXTR(data, intr, intr);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetCppDmaDoneIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetCppDmaDoneIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscCppIntrGroup *intrGrp)
{
    uint32_t ui, data = hMisc->regs->misc_intr_mask_r4;
    
    for(ui = 0; ui < 32; ui++)
    {
        intrGrp->dmaDone[ui] = CSL_FEXTR(data, ui, ui);
    }    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMiscIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMiscIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r2, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMiscIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMiscIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, intrGrp->cppRdNack);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, intrGrp->arbFifoErr);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, intrGrp->arbP2lDone);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, intrGrp->arbFbSwitch);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0, intrGrp->gpio[0]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1, intrGrp->gpio[1]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2, intrGrp->gpio[2]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3, intrGrp->gpio[3]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4, intrGrp->gpio[4]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5, intrGrp->gpio[5]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6, intrGrp->gpio[6]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7, intrGrp->gpio[7]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8, intrGrp->gpio[8]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9, intrGrp->gpio[9]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10, intrGrp->gpio[10]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11, intrGrp->gpio[11]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12, intrGrp->gpio[12]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13, intrGrp->gpio[13]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14, intrGrp->gpio[14]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15, intrGrp->gpio[15]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16, intrGrp->gpio[16]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17, intrGrp->gpio[17]);
    
    hMisc->regs->misc_intr_mask_r2 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMiscIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMiscIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r2, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMiscIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMiscIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, intrGrp->cppRdNack);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, intrGrp->arbFifoErr);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, intrGrp->arbP2lDone);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, intrGrp->arbFbSwitch);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0, intrGrp->gpio[0]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1, intrGrp->gpio[1]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2, intrGrp->gpio[2]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3, intrGrp->gpio[3]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4, intrGrp->gpio[4]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5, intrGrp->gpio[5]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6, intrGrp->gpio[6]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7, intrGrp->gpio[7]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8, intrGrp->gpio[8]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9, intrGrp->gpio[9]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10, intrGrp->gpio[10]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11, intrGrp->gpio[11]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12, intrGrp->gpio[12]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13, intrGrp->gpio[13]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14, intrGrp->gpio[14]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15, intrGrp->gpio[15]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16, intrGrp->gpio[16]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17, intrGrp->gpio[17]);
    
    hMisc->regs->misc_intr_mask_r2 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMiscIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMiscIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r8, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMiscIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMiscIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, intrGrp->cppRdNack);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, intrGrp->arbFifoErr);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, intrGrp->arbP2lDone);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, intrGrp->arbFbSwitch);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0, intrGrp->gpio[0]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1, intrGrp->gpio[1]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2, intrGrp->gpio[2]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3, intrGrp->gpio[3]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4, intrGrp->gpio[4]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5, intrGrp->gpio[5]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6, intrGrp->gpio[6]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7, intrGrp->gpio[7]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8, intrGrp->gpio[8]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9, intrGrp->gpio[9]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10, intrGrp->gpio[10]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11, intrGrp->gpio[11]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12, intrGrp->gpio[12]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13, intrGrp->gpio[13]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14, intrGrp->gpio[14]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15, intrGrp->gpio[15]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16, intrGrp->gpio[16]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17, intrGrp->gpio[17]);
    
    hMisc->regs->misc_intr_mask_r8 |= data;
}


CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMiscIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMiscIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->misc_intr_mask_r8, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMiscIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMiscIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, intrGrp->cppRdNack);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, intrGrp->arbFifoErr);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, intrGrp->arbP2lDone);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, intrGrp->arbFbSwitch);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0, intrGrp->gpio[0]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1, intrGrp->gpio[1]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2, intrGrp->gpio[2]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3, intrGrp->gpio[3]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4, intrGrp->gpio[4]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5, intrGrp->gpio[5]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6, intrGrp->gpio[6]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7, intrGrp->gpio[7]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8, intrGrp->gpio[8]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9, intrGrp->gpio[9]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10, intrGrp->gpio[10]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11, intrGrp->gpio[11]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12, intrGrp->gpio[12]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13, intrGrp->gpio[13]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14, intrGrp->gpio[14]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15, intrGrp->gpio[15]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16, intrGrp->gpio[16]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17, intrGrp->gpio[17]);
    
    hMisc->regs->misc_intr_mask_r8 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMiscIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMiscIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr)
{
    uint32_t b = (1u << intr);
    
    // write '0' to clear
    hMisc->regs->misc_intr_mask_r5 = ~b;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMiscIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMiscIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, intrGrp->cppRdNack);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, intrGrp->arbFifoErr);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, intrGrp->arbP2lDone);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, intrGrp->arbFbSwitch);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0, intrGrp->gpio[0]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1, intrGrp->gpio[1]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2, intrGrp->gpio[2]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3, intrGrp->gpio[3]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4, intrGrp->gpio[4]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5, intrGrp->gpio[5]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6, intrGrp->gpio[6]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7, intrGrp->gpio[7]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8, intrGrp->gpio[8]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9, intrGrp->gpio[9]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10, intrGrp->gpio[10]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11, intrGrp->gpio[11]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12, intrGrp->gpio[12]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13, intrGrp->gpio[13]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14, intrGrp->gpio[14]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15, intrGrp->gpio[15]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16, intrGrp->gpio[16]);
    CSL_FINSR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17, intrGrp->gpio[17]);
    
    hMisc->regs->misc_intr_mask_r5 = ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMiscIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
         status    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMiscIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr, uint32_t *status)
{
    uint32_t data = hMisc->regs->misc_intr_mask_r5;
        
    *status = CSL_FEXTR(data, intr, intr);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMiscIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMiscIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMiscIntrGroup *intrGrp)
{    
    uint32_t data = hMisc->regs->misc_intr_mask_r5;
    
    intrGrp->cppRdNack = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK, DFE_FL_MISC_MISC_INTR_CPP_RD_NACK);
    intrGrp->arbFifoErr = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR, DFE_FL_MISC_MISC_INTR_ARB_FIFO_ERR);
    intrGrp->arbP2lDone = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE, DFE_FL_MISC_MISC_INTR_ARB_P2L_DONE);
    intrGrp->arbFbSwitch = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH, DFE_FL_MISC_MISC_INTR_ARB_FB_SWITCH);
    intrGrp->gpio[0] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_0, DFE_FL_MISC_MISC_INTR_GPIO_0);
    intrGrp->gpio[1] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_1, DFE_FL_MISC_MISC_INTR_GPIO_1);
    intrGrp->gpio[2] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_2, DFE_FL_MISC_MISC_INTR_GPIO_2);
    intrGrp->gpio[3] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_3, DFE_FL_MISC_MISC_INTR_GPIO_3);
    intrGrp->gpio[4] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_4, DFE_FL_MISC_MISC_INTR_GPIO_4);
    intrGrp->gpio[5] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_5, DFE_FL_MISC_MISC_INTR_GPIO_5);
    intrGrp->gpio[6] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_6, DFE_FL_MISC_MISC_INTR_GPIO_6);
    intrGrp->gpio[7] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_7, DFE_FL_MISC_MISC_INTR_GPIO_7);
    intrGrp->gpio[8] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_8, DFE_FL_MISC_MISC_INTR_GPIO_8);
    intrGrp->gpio[9] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_9, DFE_FL_MISC_MISC_INTR_GPIO_9);
    intrGrp->gpio[10] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_10, DFE_FL_MISC_MISC_INTR_GPIO_10);
    intrGrp->gpio[11] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_11, DFE_FL_MISC_MISC_INTR_GPIO_11);
    intrGrp->gpio[12] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_12, DFE_FL_MISC_MISC_INTR_GPIO_12);
    intrGrp->gpio[13] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_13, DFE_FL_MISC_MISC_INTR_GPIO_13);
    intrGrp->gpio[14] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_14, DFE_FL_MISC_MISC_INTR_GPIO_14);
    intrGrp->gpio[15] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_15, DFE_FL_MISC_MISC_INTR_GPIO_15);
    intrGrp->gpio[16] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_16, DFE_FL_MISC_MISC_INTR_GPIO_16);
    intrGrp->gpio[17] = CSL_FEXTR(data, DFE_FL_MISC_MISC_INTR_GPIO_17, DFE_FL_MISC_MISC_INTR_GPIO_17);    
}


CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMasterLowPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMasterLowPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->master_int_master_lp_intr_mask_r0, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMasterLowPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMasterLowPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB, intrGrp->bb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, intrGrp->bbtxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, intrGrp->bbrxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, intrGrp->bbtxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, intrGrp->bbrxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, intrGrp->dduc[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, intrGrp->dduc[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, intrGrp->dduc[2]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, intrGrp->dduc[3]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0, intrGrp->cfr[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1, intrGrp->cfr[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD, intrGrp->dpd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA, intrGrp->dpda);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX, intrGrp->tx);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD, intrGrp->jesd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, intrGrp->rxIbpm);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB, intrGrp->cb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC, intrGrp->misc);
    
    hMisc->regs->master_int_master_lp_intr_mask_r0 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMasterLowPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMasterLowPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->master_int_master_lp_intr_mask_r0, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMasterLowPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMasterLowPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB, intrGrp->bb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, intrGrp->bbtxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, intrGrp->bbrxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, intrGrp->bbtxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, intrGrp->bbrxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, intrGrp->dduc[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, intrGrp->dduc[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, intrGrp->dduc[2]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, intrGrp->dduc[3]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0, intrGrp->cfr[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1, intrGrp->cfr[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD, intrGrp->dpd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA, intrGrp->dpda);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX, intrGrp->tx);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD, intrGrp->jesd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, intrGrp->rxIbpm);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB, intrGrp->cb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC, intrGrp->misc);
    
    hMisc->regs->master_int_master_lp_intr_mask_r0 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMasterLowPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMasterLowPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->master_int_master_lp_intr_mask_r2, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMasterLowPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMasterLowPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB, intrGrp->bb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, intrGrp->bbtxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, intrGrp->bbrxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, intrGrp->bbtxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, intrGrp->bbrxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, intrGrp->dduc[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, intrGrp->dduc[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, intrGrp->dduc[2]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, intrGrp->dduc[3]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0, intrGrp->cfr[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1, intrGrp->cfr[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD, intrGrp->dpd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA, intrGrp->dpda);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX, intrGrp->tx);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD, intrGrp->jesd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, intrGrp->rxIbpm);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB, intrGrp->cb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC, intrGrp->misc);
    
    hMisc->regs->master_int_master_lp_intr_mask_r2 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMasterLowPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMasterLowPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->master_int_master_lp_intr_mask_r2, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMasterLowPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMasterLowPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB, intrGrp->bb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, intrGrp->bbtxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, intrGrp->bbrxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, intrGrp->bbtxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, intrGrp->bbrxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, intrGrp->dduc[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, intrGrp->dduc[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, intrGrp->dduc[2]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, intrGrp->dduc[3]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0, intrGrp->cfr[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1, intrGrp->cfr[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD, intrGrp->dpd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA, intrGrp->dpda);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX, intrGrp->tx);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD, intrGrp->jesd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, intrGrp->rxIbpm);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB, intrGrp->cb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC, intrGrp->misc);
    
    hMisc->regs->master_int_master_lp_intr_mask_r2 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMasterLowPriIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMasterLowPriIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr)
{
    uint32_t b = (1u << intr);
    
    // write '0' to clear
    hMisc->regs->master_int_master_lp_intr_mask_r1 = ~b;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMasterLowPriIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMasterLowPriIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB, intrGrp->bb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, intrGrp->bbtxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, intrGrp->bbrxPowmtr);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, intrGrp->bbtxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, intrGrp->bbrxGainUpt);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, intrGrp->dduc[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, intrGrp->dduc[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, intrGrp->dduc[2]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, intrGrp->dduc[3]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0, intrGrp->cfr[0]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1, intrGrp->cfr[1]);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD, intrGrp->dpd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA, intrGrp->dpda);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX, intrGrp->tx);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD, intrGrp->jesd);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, intrGrp->rxIbpm);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB, intrGrp->cb);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC, intrGrp->misc);
    
    hMisc->regs->master_int_master_lp_intr_mask_r1 = ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMasterLowPriIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
         status    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMasterLowPriIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr, uint32_t *status)
{
    uint32_t data = hMisc->regs->master_int_master_lp_intr_mask_r1;
        
    *status = CSL_FEXTR(data, intr, intr);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMasterLowPriIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMasterLowPriIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterLowPriIntrGroup *intrGrp)
{    
    uint32_t data = hMisc->regs->master_int_master_lp_intr_mask_r1;
    
    intrGrp->bb = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_BB, DFE_FL_MISC_MASTER_LOWPRI_BB);
    intrGrp->bbtxPowmtr = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBTX_POWMTR);
    intrGrp->bbrxPowmtr = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR, DFE_FL_MISC_MASTER_LOWPRI_BBRX_POWMTR);
    intrGrp->bbtxGainUpt = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBTX_GAINUPT);
    intrGrp->bbrxGainUpt = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT, DFE_FL_MISC_MASTER_LOWPRI_BBRX_GAINUPT);
    intrGrp->dduc[0] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC0, DFE_FL_MISC_MASTER_LOWPRI_DDUC0);
    intrGrp->dduc[1] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC1, DFE_FL_MISC_MASTER_LOWPRI_DDUC1);
    intrGrp->dduc[2] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC2, DFE_FL_MISC_MASTER_LOWPRI_DDUC2);
    intrGrp->dduc[3] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DDUC3, DFE_FL_MISC_MASTER_LOWPRI_DDUC3);
    intrGrp->cfr[0] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR0, DFE_FL_MISC_MASTER_LOWPRI_CFR0);
    intrGrp->cfr[1] = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_CFR1, DFE_FL_MISC_MASTER_LOWPRI_CFR1);
    intrGrp->dpd = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DPD, DFE_FL_MISC_MASTER_LOWPRI_DPD);
    intrGrp->dpda = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_DPDA, DFE_FL_MISC_MASTER_LOWPRI_DPDA);
    intrGrp->tx = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_TX, DFE_FL_MISC_MASTER_LOWPRI_TX);
    intrGrp->jesd = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_JESD, DFE_FL_MISC_MASTER_LOWPRI_JESD);
    intrGrp->rxIbpm = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM, DFE_FL_MISC_MASTER_LOWPRI_RX_IBPM);
    intrGrp->cb = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_CB, DFE_FL_MISC_MASTER_LOWPRI_CB);
    intrGrp->misc = CSL_FEXTR(data, DFE_FL_MISC_MASTER_LOWPRI_MISC, DFE_FL_MISC_MASTER_LOWPRI_MISC);    
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMasterHiPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMasterHiPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->master_int_master_hp_intr_mask_r0, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscEnableMasterHiPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscEnableMasterHiPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, intrGrp->txbAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, intrGrp->txbAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, intrGrp->txbAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, intrGrp->txbAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, intrGrp->txbAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, intrGrp->txbAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, intrGrp->txbAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, intrGrp->txbAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, intrGrp->txaAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, intrGrp->txaAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, intrGrp->txaAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, intrGrp->txaAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, intrGrp->txaAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, intrGrp->txaAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, intrGrp->txaAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, intrGrp->txaAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, intrGrp->cppDmaErr);
    
    hMisc->regs->master_int_master_hp_intr_mask_r0 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMasterHiPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMasterHiPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->master_int_master_hp_intr_mask_r0, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscDisableMasterHiPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscDisableMasterHiPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, intrGrp->txbAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, intrGrp->txbAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, intrGrp->txbAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, intrGrp->txbAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, intrGrp->txbAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, intrGrp->txbAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, intrGrp->txbAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, intrGrp->txbAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, intrGrp->txaAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, intrGrp->txaAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, intrGrp->txaAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, intrGrp->txaAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, intrGrp->txaAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, intrGrp->txaAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, intrGrp->txaAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, intrGrp->txaAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, intrGrp->cppDmaErr);
    
    hMisc->regs->master_int_master_hp_intr_mask_r0 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMasterHiPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMasterHiPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '1' to enable
    CSL_FINSR(hMisc->regs->master_int_master_hp_intr_mask_r2, intr, intr, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscSetForceMasterHiPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscSetForceMasterHiPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, intrGrp->txbAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, intrGrp->txbAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, intrGrp->txbAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, intrGrp->txbAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, intrGrp->txbAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, intrGrp->txbAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, intrGrp->txbAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, intrGrp->txbAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, intrGrp->txaAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, intrGrp->txaAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, intrGrp->txaAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, intrGrp->txaAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, intrGrp->txaAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, intrGrp->txaAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, intrGrp->txaAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, intrGrp->txaAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, intrGrp->cppDmaErr);
    
    hMisc->regs->master_int_master_hp_intr_mask_r2 |= data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMasterHiPriIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMasterHiPriIntr(DfeFl_MiscHandle hMisc, uint32_t intr)
{    
    // write '0' to disable
    CSL_FINSR(hMisc->regs->master_int_master_hp_intr_mask_r2, intr, intr, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearForceMasterHiPriIntrGroup
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearForceMasterHiPriIntrGroup(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, intrGrp->txbAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, intrGrp->txbAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, intrGrp->txbAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, intrGrp->txbAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, intrGrp->txbAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, intrGrp->txbAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, intrGrp->txbAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, intrGrp->txbAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, intrGrp->txaAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, intrGrp->txaAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, intrGrp->txaAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, intrGrp->txaAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, intrGrp->txaAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, intrGrp->txaAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, intrGrp->txaAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, intrGrp->txaAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, intrGrp->cppDmaErr);
    
    hMisc->regs->master_int_master_hp_intr_mask_r2 &= ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMasterHiPriIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMasterHiPriIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr)
{
    uint32_t b = (1u << intr);
    
    // write '0' to clear
    hMisc->regs->master_int_master_hp_intr_mask_r1 = ~b;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscClearMasterHiPriIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscClearMasterHiPriIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = 0;
    
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, intrGrp->txbAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, intrGrp->txbAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, intrGrp->txbAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, intrGrp->txbAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, intrGrp->txbAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, intrGrp->txbAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, intrGrp->txbAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, intrGrp->txbAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, intrGrp->txaAnt1PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, intrGrp->txaAnt1PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, intrGrp->txaAnt1TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, intrGrp->txaAnt1CfrGain);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, intrGrp->txaAnt0PeakClip);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, intrGrp->txaAnt0PwrSat);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, intrGrp->txaAnt0TxZero);
    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, intrGrp->txaAnt0CfrGain);

    CSL_FINSR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, intrGrp->cppDmaErr);
    
    hMisc->regs->master_int_master_hp_intr_mask_r1 = ~data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMasterHiPriIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    DFE_MISC block handle
         intr    [add content]
         status    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMasterHiPriIntrStatus(DfeFl_MiscHandle hMisc, uint32_t intr, uint32_t *status)
{
    uint32_t data = hMisc->regs->master_int_master_hp_intr_mask_r1;
        
    *status = CSL_FEXTR(data, intr, intr);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_MiscGetMasterHiPriIntrGroupStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hMisc    [add content]
         intrGrp    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  [add content]
 *
 *   <b> Post Condition </b>
 *   @n  [add content]
 *
 *   @b Reads
 *   @n  None
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_MiscGetMasterHiPriIntrGroupStatus(DfeFl_MiscHandle hMisc, DfeFl_MiscMasterHiPriIntrGroup *intrGrp)
{   
    uint32_t data = hMisc->regs->master_int_master_hp_intr_mask_r1;
    
    intrGrp->txbAnt1PeakClip = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PEAK_CLIP);
    intrGrp->txbAnt1PwrSat = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_PWR_SAT);
    intrGrp->txbAnt1TxZero = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_TX_ZERO);
    intrGrp->txbAnt1CfrGain = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT1_CFR_GAIN);
    intrGrp->txbAnt0PeakClip = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PEAK_CLIP);
    intrGrp->txbAnt0PwrSat = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_PWR_SAT);
    intrGrp->txbAnt0TxZero = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_TX_ZERO);
    intrGrp->txbAnt0CfrGain = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXB_ANT0_CFR_GAIN);
    intrGrp->txaAnt1PeakClip = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PEAK_CLIP);
    intrGrp->txaAnt1PwrSat = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_PWR_SAT);
    intrGrp->txaAnt1TxZero = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_TX_ZERO);
    intrGrp->txaAnt1CfrGain = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT1_CFR_GAIN);
    intrGrp->txaAnt0PeakClip = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PEAK_CLIP);
    intrGrp->txaAnt0PwrSat = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_PWR_SAT);
    intrGrp->txaAnt0TxZero = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_TX_ZERO);
    intrGrp->txaAnt0CfrGain = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN, DFE_FL_MISC_MASTER_HIPRI_TXA_ANT0_CFR_GAIN);
    intrGrp->cppDmaErr = CSL_FEXTR(data, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR, DFE_FL_MISC_MASTER_HIPRI_CPP_DMA_ERR);    
}


#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_MISCAUX_H_ */
