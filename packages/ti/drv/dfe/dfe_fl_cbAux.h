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

/** @file dfe_fl_cbAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CB CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_CBAUX_H_
#define _DFE_FL_CBAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_cb.h>

/** ============================================================================
 *   @n@b dfeFl_CbConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_INITS_SSEL
 *       DFE_CB_INITS_REG_CLEAR_DATA
 *       DFE_CB_INITS_REG_INIT_STATE
 *       DFE_CB_INITS_REG_INIT_CLK_GATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_CbConfigInits(DfeFl_CbHandle hDfeCb, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeCb->regs->inits;
    
    CSL_FINS(data, DFE_CB_INITS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_CB_INITS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_CB_INITS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_CB_INITS_REG_CLEAR_DATA, arg->clearData);
    
    hDfeCb->regs->inits = data;
}

/** ============================================================================
 *   @n@b dfeFl_CbGetInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_INITS_SSEL
 *       DFE_CB_INITS_REG_CLEAR_DATA
 *       DFE_CB_INITS_REG_INIT_STATE
 *       DFE_CB_INITS_REG_INIT_CLK_GATE
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
CSL_IDEF_INLINE void
dfeFl_CbGetInits(DfeFl_CbHandle hDfeCb, DfeFl_SublkInitsConfig * arg)
{
	volatile uint32_t *reg = &hDfeCb->regs->inits;

    arg->ssel = CSL_FEXT(reg[0], DFE_CB_INITS_REG_INITS_SSEL);
    arg->initClkGate = CSL_FEXT(reg[0], DFE_CB_INITS_REG_INIT_CLK_GATE);
    arg->initState = CSL_FEXT(reg[0], DFE_CB_INITS_REG_INIT_STATE);
    arg->clearData = CSL_FEXT(reg[0], DFE_CB_INITS_REG_CLEAR_DATA);

}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbcArm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_ARM_REG_CB_C_CAPTURE_DONE
 *       DFE_CB_CB_ARM_REG_CB_C_SYNC_ARM
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbcArm(DfeFl_CbHandle hDfeCb, DfeFl_CbArm *arg)
{
	CSL_FINS(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_C_SYNC_ARM, arg->sync);
	CSL_FINS(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_C_CAPTURE_DONE, arg->cbDone);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbcForceReset
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_ARM_RESET
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_DONE_RESET
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbcForceReset(DfeFl_CbHandle hDfeCb, DfeFl_CbForceReset *arg)
{
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_ARM_RESET, arg->force_arm_reset);
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_DONE_RESET, arg->force_done_reset);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbcArm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_ARM_REG_CB_C_CAPTURE_DONE
 *       DFE_CB_CB_ARM_REG_CB_C_SYNC_ARM
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbcArm(DfeFl_CbHandle hDfeCb, DfeFl_CbArm *arg)
{
	arg->sync = CSL_FEXT(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_C_SYNC_ARM);
	arg->cbDone = CSL_FEXT(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_C_CAPTURE_DONE);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbcForceReset
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_ARM_RESET
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_DONE_RESET
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbcForceReset(DfeFl_CbHandle hDfeCb, DfeFl_CbForceReset *arg)
{
	arg->force_arm_reset = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_ARM_RESET);
	arg->force_done_reset = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_C_FORCE_DONE_RESET);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfArm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_ARM_REG_CB_F_SYNC_ARM
 *       DFE_CB_CB_ARM_REG_CB_F_CAPTURE_DONE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfArm(DfeFl_CbHandle hDfeCb, DfeFl_CbArm *arg)
{
	CSL_FINS(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_F_SYNC_ARM, arg->sync);
	CSL_FINS(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_F_CAPTURE_DONE, arg->cbDone);
}

//CSL_IDEF_INLINE void
//dfeFl_CbSetCbfForceReset(DfeFl_CbHandle hDfeCb, DfeFl_CbForceReset *arg)
//{
////	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_FORCE_ARM_RESET, arg->force_arm_reset);
//	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_FORCE_DONE_RESET, arg->force_done_reset);
//}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfSubsample
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_F_SUBSAMPLE_FB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfSubsample(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_SUBSAMPLE_FB, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbfArm
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_ARM_REG_CB_F_SYNC_ARM
 *       DFE_CB_CB_ARM_REG_CB_F_CAPTURE_DONE
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbfArm(DfeFl_CbHandle hDfeCb, DfeFl_CbArm *arg)
{
	arg->sync = CSL_FEXT(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_F_SYNC_ARM);
	arg->cbDone = CSL_FEXT(hDfeCb->regs->cb_arm, DFE_CB_CB_ARM_REG_CB_F_CAPTURE_DONE);
}

//CSL_IDEF_INLINE void
//dfeFl_CbQueryCbfForceReset(DfeFl_CbHandle hDfeCb, DfeFl_CbForceReset *arg)
//{
////	arg->force_arm_reset = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_FORCE_ARM_RESET);
//	arg->force_done_reset = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_FORCE_DONE_RESET);
//}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbfSubsample
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_CB_F_SUBSAMPLE_FB
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbfSubsample(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_CB_F_SUBSAMPLE_FB);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetBusCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_TBUS_SEL
 *       DFE_CB_BUS_CTRL_RESET_REG_IQ_SWAP
 *       DFE_CB_BUS_CTRL_RESET_REG_NOGATING
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetBusCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbBusCtrl *arg)
{
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_NOGATING, arg->nogating);
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_TBUS_SEL, arg->tbus_sel);
	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_IQ_SWAP, arg->IQ_swap);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetBusCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_TBUS_SEL
 *       DFE_CB_BUS_CTRL_RESET_REG_IQ_SWAP
 *       DFE_CB_BUS_CTRL_RESET_REG_NOGATING
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
CSL_IDEF_INLINE void
dfeFl_CbGetBusCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbBusCtrl *arg)
{
	arg->nogating = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_NOGATING);
	arg->tbus_sel = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_TBUS_SEL);
	arg->IQ_swap = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_IQ_SWAP);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetDspCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_DSP_CTRL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetDspCtrl(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_DSP_CTRL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetDspCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_DSP_CTRL
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
CSL_IDEF_INLINE void
dfeFl_CbGetDspCtrl(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_DSP_CTRL);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetDpdMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_DPD_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetDpdMode(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_DPD_MODE, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetDpdMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUS_CTRL_RESET_REG_DPD_MODE
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
CSL_IDEF_INLINE void
dfeFl_CbGetDpdMode(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->bus_ctrl_reset, DFE_CB_BUS_CTRL_RESET_REG_DPD_MODE);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetRefFbLatency
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_REF_FB_LATENCY_ANT0_REG_REF_FB_LATENCY_ANT0
 *       DFE_CB_CB_REF_FB_LATENCY_ANT3_REG_REF_FB_LATENCY_ANT3
 *       DFE_CB_CB_REF_FB_LATENCY_ANT2_REG_REF_FB_LATENCY_ANT2
 *       DFE_CB_CB_REF_FB_LATENCY_ANT1_REG_REF_FB_LATENCY_ANT1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetRefFbLatency(DfeFl_CbHandle hDfeCb, DfeFl_CbFbLatency *arg)
{
   	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->cb_ref_fb_latency_ant0, DFE_CB_CB_REF_FB_LATENCY_ANT0_REG_REF_FB_LATENCY_ANT0, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->cb_ref_fb_latency_ant1, DFE_CB_CB_REF_FB_LATENCY_ANT1_REG_REF_FB_LATENCY_ANT1, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->cb_ref_fb_latency_ant2, DFE_CB_CB_REF_FB_LATENCY_ANT2_REG_REF_FB_LATENCY_ANT2, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->cb_ref_fb_latency_ant3, DFE_CB_CB_REF_FB_LATENCY_ANT3_REG_REF_FB_LATENCY_ANT3, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetRefFbLatency
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_REF_FB_LATENCY_ANT0_REG_REF_FB_LATENCY_ANT0
 *       DFE_CB_CB_REF_FB_LATENCY_ANT3_REG_REF_FB_LATENCY_ANT3
 *       DFE_CB_CB_REF_FB_LATENCY_ANT2_REG_REF_FB_LATENCY_ANT2
 *       DFE_CB_CB_REF_FB_LATENCY_ANT1_REG_REF_FB_LATENCY_ANT1
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
CSL_IDEF_INLINE void
dfeFl_CbGetRefFbLatency(DfeFl_CbHandle hDfeCb, DfeFl_CbFbLatency *arg)
{
   	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_ref_fb_latency_ant0, DFE_CB_CB_REF_FB_LATENCY_ANT0_REG_REF_FB_LATENCY_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_ref_fb_latency_ant1, DFE_CB_CB_REF_FB_LATENCY_ANT1_REG_REF_FB_LATENCY_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_ref_fb_latency_ant2, DFE_CB_CB_REF_FB_LATENCY_ANT2_REG_REF_FB_LATENCY_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_ref_fb_latency_ant3, DFE_CB_CB_REF_FB_LATENCY_ANT3_REG_REF_FB_LATENCY_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetSkipChunk
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_DPDA_READ_SKIPCHUNK_REG_READREF_SKIPCHUNK
 *       DFE_CB_DPDA_READ_SKIPCHUNK_REG_READFB_SKIPCHUNK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetSkipChunk(DfeFl_CbHandle hDfeCb, DfeFl_CbSkipChunk *arg)
{
	uint32_t data = hDfeCb->regs->dpda_read_skipchunk;

	CSL_FINS(data, DFE_CB_DPDA_READ_SKIPCHUNK_REG_READREF_SKIPCHUNK, arg->readref_skipchunk);
	CSL_FINS(data, DFE_CB_DPDA_READ_SKIPCHUNK_REG_READFB_SKIPCHUNK, arg->readfb_skipchunk);

	hDfeCb->regs->dpda_read_skipchunk = data;
}

/** ============================================================================
 *   @n@b dfeFl_CbGetSkipChunk
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_DPDA_READ_SKIPCHUNK_REG_READREF_SKIPCHUNK
 *       DFE_CB_DPDA_READ_SKIPCHUNK_REG_READFB_SKIPCHUNK
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
CSL_IDEF_INLINE void
dfeFl_CbGetSkipChunk(DfeFl_CbHandle hDfeCb, DfeFl_CbSkipChunk *arg)
{
	arg->readref_skipchunk = CSL_FEXT(hDfeCb->regs->dpda_read_skipchunk, DFE_CB_DPDA_READ_SKIPCHUNK_REG_READREF_SKIPCHUNK);
	arg->readfb_skipchunk = CSL_FEXT(hDfeCb->regs->dpda_read_skipchunk, DFE_CB_DPDA_READ_SKIPCHUNK_REG_READFB_SKIPCHUNK);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetBufMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_BUFFER_MODE_REG_CBA_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBC_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBB_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBD_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetBufMode(DfeFl_CbHandle hDfeCb, DfeFl_CbBufMode *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		CSL_FINS(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBA_MODE, arg->data);
		break;
	case DFE_FL_CBB:
		CSL_FINS(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBB_MODE, arg->data);
		break;
	case DFE_FL_CBC:
		CSL_FINS(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBC_MODE, arg->data);
		break;
	case DFE_FL_CBD:
		CSL_FINS(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBD_MODE, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetBufMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_BUFFER_MODE_REG_CBA_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBC_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBB_MODE
 *       DFE_CB_CB_BUFFER_MODE_REG_CBD_MODE
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
CSL_IDEF_INLINE void
dfeFl_CbGetBufMode(DfeFl_CbHandle hDfeCb, DfeFl_CbBufMode *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBA_MODE);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBB_MODE);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBC_MODE);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_buffer_mode, DFE_CB_CB_BUFFER_MODE_REG_CBD_MODE);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetModeSet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBA_SETTING_REG_CBA_SEL
 *       DFE_CB_CBA_SETTING_REG_CBA_REF_OR_FB
 *       DFE_CB_CBA_SETTING_REG_CBA_NOT_USED
 *       DFE_CB_CBA_SETTING_REG_CBA_BUS_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetModeSet(DfeFl_CbHandle hDfeCb, DfeFl_CbModeSet *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->cba_setting;
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->cbb_setting;
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->cbc_setting;
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->cbd_setting;
		break;
    default:
    	return;
	}
    regs[0] = CSL_FMK(DFE_CB_CBA_SETTING_REG_CBA_SEL, arg->sel)
            | CSL_FMK(DFE_CB_CBA_SETTING_REG_CBA_BUS_SEL, arg->busSel)
            | CSL_FMK(DFE_CB_CBA_SETTING_REG_CBA_REF_OR_FB, arg->ref_or_fb)
            | CSL_FMK(DFE_CB_CBA_SETTING_REG_CBA_NOT_USED, arg->not_used);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetModeSet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBA_SETTING_REG_CBA_SEL
 *       DFE_CB_CBA_SETTING_REG_CBA_REF_OR_FB
 *       DFE_CB_CBA_SETTING_REG_CBA_NOT_USED
 *       DFE_CB_CBA_SETTING_REG_CBA_BUS_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetModeSet(DfeFl_CbHandle hDfeCb, DfeFl_CbModeSet *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->cba_setting;
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->cbb_setting;
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->cbc_setting;
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->cbd_setting;
		break;
    default:
    	return;
	}
	arg->sel = CSL_FEXT(regs[0], DFE_CB_CBA_SETTING_REG_CBA_SEL);
	arg->busSel = CSL_FEXT(regs[0], DFE_CB_CBA_SETTING_REG_CBA_BUS_SEL);
	arg->ref_or_fb = CSL_FEXT(regs[0], DFE_CB_CBA_SETTING_REG_CBA_REF_OR_FB);
	arg->not_used = CSL_FEXT(regs[0], DFE_CB_CBA_SETTING_REG_CBA_NOT_USED);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBA_DLY_REG_CBA_DLY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbDly(DfeFl_CbHandle hDfeCb, DfeFl_CbDly *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->cba_dly;
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->cbb_dly;
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->cbc_dly;
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->cbd_dly;
		break;
    default:
    	return;
	}
	regs[0] = CSL_FMK(DFE_CB_CBA_DLY_REG_CBA_DLY, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBA_DLY_REG_CBA_DLY
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbDly(DfeFl_CbHandle hDfeCb, DfeFl_CbDly *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->cba_dly;
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->cbb_dly;
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->cbc_dly;
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->cbd_dly;
		break;
    default:
    	return;
	}
	arg->data = CSL_FEXT(regs[0], DFE_CB_CBA_DLY_REG_CBA_DLY);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbRateMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_RATE_MODE_REG_CBD_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBA_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBC_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBB_RATE_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbRateMode(DfeFl_CbHandle hDfeCb, DfeFl_CbRateMode *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		CSL_FINS(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBA_RATE_MODE, arg->data);
		break;
	case DFE_FL_CBB:
		CSL_FINS(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBB_RATE_MODE, arg->data);
		break;
	case DFE_FL_CBC:
		CSL_FINS(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBC_RATE_MODE, arg->data);
		break;
	case DFE_FL_CBD:
		CSL_FINS(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBD_RATE_MODE, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbRateMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_RATE_MODE_REG_CBD_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBA_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBC_RATE_MODE
 *       DFE_CB_RATE_MODE_REG_CBB_RATE_MODE
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbRateMode(DfeFl_CbHandle hDfeCb, DfeFl_CbRateMode *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBA_RATE_MODE);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBB_RATE_MODE);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBC_RATE_MODE);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->rate_mode, DFE_CB_RATE_MODE_REG_CBD_RATE_MODE);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbFracCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_FRAC_CNT_REG_CBD_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBC_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBB_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBA_FRAC_CNT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbFracCnt(DfeFl_CbHandle hDfeCb, DfeFl_CbFracCnt *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		CSL_FINS(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBA_FRAC_CNT, arg->data);
		break;
	case DFE_FL_CBB:
		CSL_FINS(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBB_FRAC_CNT, arg->data);
		break;
	case DFE_FL_CBC:
		CSL_FINS(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBC_FRAC_CNT, arg->data);
		break;
	case DFE_FL_CBD:
		CSL_FINS(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBD_FRAC_CNT, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbFracCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_FRAC_CNT_REG_CBD_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBC_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBB_FRAC_CNT
 *       DFE_CB_FRAC_CNT_REG_CBA_FRAC_CNT
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbFracCnt(DfeFl_CbHandle hDfeCb, DfeFl_CbFracCnt *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBA_FRAC_CNT);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBB_FRAC_CNT);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBC_FRAC_CNT);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->frac_cnt, DFE_CB_FRAC_CNT_REG_CBD_FRAC_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBD_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBC_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBB_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBD_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBA_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBB_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBA_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBC_LEN_CNT_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbSsel *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
	{
		CSL_FINS(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBA_FRAC_CNT_SSEL, arg->frac_cnt_ssel);
		CSL_FINS(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBA_LEN_CNT_SSEL, arg->len_cnt_ssel);
		break;
	}
	case DFE_FL_CBB:
	{
		CSL_FINS(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBB_FRAC_CNT_SSEL, arg->frac_cnt_ssel);
		CSL_FINS(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBB_LEN_CNT_SSEL, arg->len_cnt_ssel);
		break;
	}
	case DFE_FL_CBC:
	{
		CSL_FINS(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBC_FRAC_CNT_SSEL, arg->frac_cnt_ssel);
		CSL_FINS(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBC_LEN_CNT_SSEL, arg->len_cnt_ssel);
		break;
	}
	case DFE_FL_CBD:
	{
		CSL_FINS(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBD_FRAC_CNT_SSEL, arg->frac_cnt_ssel);
		CSL_FINS(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBD_LEN_CNT_SSEL, arg->len_cnt_ssel);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBD_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBC_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBB_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBD_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART2_REG_CBA_FRAC_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBB_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBA_LEN_CNT_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART3_REG_CBC_LEN_CNT_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbSsel *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
	{
		arg->frac_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBA_FRAC_CNT_SSEL);
		arg->len_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBA_LEN_CNT_SSEL);
		break;
	}
	case DFE_FL_CBB:
	{
		arg->frac_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBB_FRAC_CNT_SSEL);
		arg->len_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBB_LEN_CNT_SSEL);
		break;
	}
	case DFE_FL_CBC:
	{
		arg->frac_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBC_FRAC_CNT_SSEL);
		arg->len_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBC_LEN_CNT_SSEL);
		break;
	}
	case DFE_FL_CBD:
	{
		arg->frac_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part2, DFE_CB_CB_SYNC_SELECT_PART2_REG_CBD_FRAC_CNT_SSEL);
		arg->len_cnt_ssel = CSL_FEXT(hDfeCb->regs->cb_sync_select_part3, DFE_CB_CB_SYNC_SELECT_PART3_REG_CBD_LEN_CNT_SSEL);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetNodeConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I1FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I0FSDLY
 *       DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSF
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q1BUS_SEL
 *       DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSFM
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q0BUS_SEL
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I1BUS_SEL
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q1FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q0FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I0BUS_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetNodeConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbNodeCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbNode)
	{
	case DFE_FL_CB_NODE0:
		regs = &hDfeCb->regs->node0_config;
		break;
	case DFE_FL_CB_NODE1:
		regs = &hDfeCb->regs->node1_config;
		break;
	case DFE_FL_CB_NODE2:
		regs = &hDfeCb->regs->node2_config;
		break;
	case DFE_FL_CB_NODE3:
		regs = &hDfeCb->regs->node3_config;
		break;
	case DFE_FL_CB_NODE4:
		regs = &hDfeCb->regs->node4_config;
		break;
	case DFE_FL_CB_NODE5:
		regs = &hDfeCb->regs->node5_config;
		break;
	case DFE_FL_CB_NODE6:
		regs = &hDfeCb->regs->node6_config;
		break;
	case DFE_FL_CB_NODE7:
		regs = &hDfeCb->regs->node7_config;
		break;
	case DFE_FL_CB_NODE8:
		regs = &hDfeCb->regs->node8_config;
		break;
    default:
    	return;
	}
    regs[0] = CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_I0BUS_SEL, arg->I0bus_sel)
    		| CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_Q0BUS_SEL, arg->Q0bus_sel)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_I1BUS_SEL, arg->I1bus_sel)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_Q1BUS_SEL, arg->Q1bus_sel)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_I0FSDLY, arg->I0fsdly)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_Q0FSDLY, arg->Q0fsdly)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_I1FSDLY, arg->I1fsdly)
            | CSL_FMK(DFE_CB_NODE0_CONFIG_REG_NODE0_Q1FSDLY, arg->Q1fsdly);
    regs[1] = CSL_FMK(DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSF, arg->fsf)
    		| CSL_FMK(DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSFM, arg->fsfm);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetNodeConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I1FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I0FSDLY
 *       DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSF
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q1BUS_SEL
 *       DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSFM
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q0BUS_SEL
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I1BUS_SEL
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q1FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_Q0FSDLY
 *       DFE_CB_NODE0_CONFIG_REG_NODE0_I0BUS_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetNodeConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbNodeCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbNode)
	{
	case DFE_FL_CB_NODE0:
		regs = &hDfeCb->regs->node0_config;
		break;
	case DFE_FL_CB_NODE1:
		regs = &hDfeCb->regs->node1_config;
		break;
	case DFE_FL_CB_NODE2:
		regs = &hDfeCb->regs->node2_config;
		break;
	case DFE_FL_CB_NODE3:
		regs = &hDfeCb->regs->node3_config;
		break;
	case DFE_FL_CB_NODE4:
		regs = &hDfeCb->regs->node4_config;
		break;
	case DFE_FL_CB_NODE5:
		regs = &hDfeCb->regs->node5_config;
		break;
	case DFE_FL_CB_NODE6:
		regs = &hDfeCb->regs->node6_config;
		break;
	case DFE_FL_CB_NODE7:
		regs = &hDfeCb->regs->node7_config;
		break;
	case DFE_FL_CB_NODE8:
		regs = &hDfeCb->regs->node8_config;
		break;
    default:
    	return;
	}

	arg->I0bus_sel = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_I0BUS_SEL);
    arg->Q0bus_sel = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_Q0BUS_SEL);
    arg->I1bus_sel = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_I1BUS_SEL);
    arg->Q1bus_sel = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_Q1BUS_SEL);
    arg->I0fsdly = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_I0FSDLY);
    arg->Q0fsdly = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_Q0FSDLY);
    arg->I1fsdly = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_I1FSDLY);
    arg->Q1fsdly = CSL_FEXT(regs[0], DFE_CB_NODE0_CONFIG_REG_NODE0_Q1FSDLY);
    arg->fsf = CSL_FEXT(regs[1], DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSF);
    arg->fsfm = CSL_FEXT(regs[1], DFE_CB_NODE0_FSF_FSFM_REG_NODE0_FSFM);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetMultiCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_NUM_CAPTURES
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_CHUNK_SIZE
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_MULTI_CAPTURE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetMultiCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbMultiCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_c_multi_capture_ctrl;

    regs[0] = CSL_FMK(DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_MULTI_CAPTURE, arg->enable)
    		| CSL_FMK(DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_NUM_CAPTURES, arg->numCaptures)
            | CSL_FMK(DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_CHUNK_SIZE, arg->chunkSize);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetMultiCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_NUM_CAPTURES
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_CHUNK_SIZE
 *       DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_MULTI_CAPTURE
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
CSL_IDEF_INLINE void
dfeFl_CbGetMultiCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbMultiCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_c_multi_capture_ctrl;

    arg->enable = CSL_FEXT(regs[0], DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_MULTI_CAPTURE);
    arg->numCaptures = CSL_FEXT(regs[0], DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_NUM_CAPTURES);
    arg->chunkSize = CSL_FEXT(regs[0], DFE_CB_CB_C_MULTI_CAPTURE_CTRL_REG_CB_C_CHUNK_SIZE);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetMultiTimer
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_C_MULTICAP_TIMER3_REG_CB_C_MULTICAP_TIMER3
 *       DFE_CB_CB_C_MULTICAP_TIMER2_REG_CB_C_MULTICAP_TIMER2
 *       DFE_CB_CB_C_MULTICAP_TIMER1_REG_CB_C_MULTICAP_TIMER1
 *       DFE_CB_CB_C_MULTICAP_TIMER8_REG_CB_C_MULTICAP_TIMER8
 *       DFE_CB_CB_C_MULTICAP_TIMER6_REG_CB_C_MULTICAP_TIMER6
 *       DFE_CB_CB_C_MULTICAP_TIMER5_REG_CB_C_MULTICAP_TIMER5
 *       DFE_CB_CB_C_MULTICAP_TIMER4_REG_CB_C_MULTICAP_TIMER4
 *       DFE_CB_CB_C_MULTICAP_TIMER7_REG_CB_C_MULTICAP_TIMER7
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetMultiTimer(DfeFl_CbHandle hDfeCb, DfeFl_CbMultiTimer *arg)
{
	switch(arg->cbMultiTimer)
	{
	case DFE_FL_CB_MULTI_TIMER1:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer1, DFE_CB_CB_C_MULTICAP_TIMER1_REG_CB_C_MULTICAP_TIMER1, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER2:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer2, DFE_CB_CB_C_MULTICAP_TIMER2_REG_CB_C_MULTICAP_TIMER2, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER3:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer3, DFE_CB_CB_C_MULTICAP_TIMER3_REG_CB_C_MULTICAP_TIMER3, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER4:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer4, DFE_CB_CB_C_MULTICAP_TIMER4_REG_CB_C_MULTICAP_TIMER4, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER5:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer5, DFE_CB_CB_C_MULTICAP_TIMER5_REG_CB_C_MULTICAP_TIMER5, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER6:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer6, DFE_CB_CB_C_MULTICAP_TIMER6_REG_CB_C_MULTICAP_TIMER6, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER7:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer7, DFE_CB_CB_C_MULTICAP_TIMER7_REG_CB_C_MULTICAP_TIMER7, arg->data);
		break;
	case DFE_FL_CB_MULTI_TIMER8:
		CSL_FINS(hDfeCb->regs->cb_c_multicap_timer8, DFE_CB_CB_C_MULTICAP_TIMER8_REG_CB_C_MULTICAP_TIMER8, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetMultiTimer
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_C_MULTICAP_TIMER3_REG_CB_C_MULTICAP_TIMER3
 *       DFE_CB_CB_C_MULTICAP_TIMER2_REG_CB_C_MULTICAP_TIMER2
 *       DFE_CB_CB_C_MULTICAP_TIMER1_REG_CB_C_MULTICAP_TIMER1
 *       DFE_CB_CB_C_MULTICAP_TIMER8_REG_CB_C_MULTICAP_TIMER8
 *       DFE_CB_CB_C_MULTICAP_TIMER6_REG_CB_C_MULTICAP_TIMER6
 *       DFE_CB_CB_C_MULTICAP_TIMER5_REG_CB_C_MULTICAP_TIMER5
 *       DFE_CB_CB_C_MULTICAP_TIMER4_REG_CB_C_MULTICAP_TIMER4
 *       DFE_CB_CB_C_MULTICAP_TIMER7_REG_CB_C_MULTICAP_TIMER7
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
CSL_IDEF_INLINE void
dfeFl_CbGetMultiTimer(DfeFl_CbHandle hDfeCb, DfeFl_CbMultiTimer *arg)
{
	switch(arg->cbMultiTimer)
	{
	case DFE_FL_CB_MULTI_TIMER1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer1, DFE_CB_CB_C_MULTICAP_TIMER1_REG_CB_C_MULTICAP_TIMER1);
		break;
	case DFE_FL_CB_MULTI_TIMER2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer2, DFE_CB_CB_C_MULTICAP_TIMER2_REG_CB_C_MULTICAP_TIMER2);
		break;
	case DFE_FL_CB_MULTI_TIMER3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer3, DFE_CB_CB_C_MULTICAP_TIMER3_REG_CB_C_MULTICAP_TIMER3);
		break;
	case DFE_FL_CB_MULTI_TIMER4:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer4, DFE_CB_CB_C_MULTICAP_TIMER4_REG_CB_C_MULTICAP_TIMER4);
		break;
	case DFE_FL_CB_MULTI_TIMER5:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer5, DFE_CB_CB_C_MULTICAP_TIMER5_REG_CB_C_MULTICAP_TIMER5);
		break;
	case DFE_FL_CB_MULTI_TIMER6:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer6, DFE_CB_CB_C_MULTICAP_TIMER6_REG_CB_C_MULTICAP_TIMER6);
		break;
	case DFE_FL_CB_MULTI_TIMER7:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer7, DFE_CB_CB_C_MULTICAP_TIMER7_REG_CB_C_MULTICAP_TIMER7);
		break;
	case DFE_FL_CB_MULTI_TIMER8:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_c_multicap_timer8, DFE_CB_CB_C_MULTICAP_TIMER8_REG_CB_C_MULTICAP_TIMER8);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTrigSet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_MULTIBAND
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_MULTIBAND
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_MAGSQD_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTrigSet(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigSet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->trigger_monitor_setting;
	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		regs[0] = CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_SEL, arg->sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_MULTIBAND, arg->multiband)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_MAGSQD_SEL, arg->blk0_MagSqd_sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_MAGSQD_SEL, arg->blk1_MagSqd_sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_IOC, arg->blk0_IOC)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_IOC, arg->blk1_IOC);
		break;
	case DFE_FL_CB_TRIG_B:
		regs[0] = CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_SEL, arg->sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_MULTIBAND, arg->multiband)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_MAGSQD_SEL, arg->blk0_MagSqd_sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_MAGSQD_SEL, arg->blk1_MagSqd_sel)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_IOC, arg->blk0_IOC)
				| CSL_FMK(DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_IOC, arg->blk1_IOC);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTrigSet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_MAGSQD_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_IOC
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_MULTIBAND
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_SEL
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_MULTIBAND
 *       DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_MAGSQD_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetTrigSet(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigSet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->trigger_monitor_setting;
	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		arg->sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_SEL);
		arg->multiband = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_MULTIBAND);
		arg->blk0_MagSqd_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_MAGSQD_SEL);
		arg->blk1_MagSqd_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_MAGSQD_SEL);
		arg->blk0_IOC = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK0_IOC);
		arg->blk1_IOC = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGA_BLK1_IOC);
		break;
	case DFE_FL_CB_TRIG_B:
		arg->sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_SEL);
		arg->multiband = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_MULTIBAND);
		arg->blk0_MagSqd_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_MAGSQD_SEL);
		arg->blk1_MagSqd_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_MAGSQD_SEL);
		arg->blk0_IOC = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK0_IOC);
		arg->blk1_IOC = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_SETTING_REG_TRIGB_BLK1_IOC);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTrigConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSF
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSFM
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTrigConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		regs = &hDfeCb->regs->trigger_monitor_a_config;
		break;
	case DFE_FL_CB_TRIG_B:
		regs = &hDfeCb->regs->trigger_monitor_b_config;
		break;
    default:
    	return;
	}
    regs[0] = CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0BUS_SEL, arg->I0bus_sel)
    		| CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0BUS_SEL, arg->Q0bus_sel)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1BUS_SEL, arg->I1bus_sel)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1BUS_SEL, arg->Q1bus_sel)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0FSDLY, arg->I0fsdly)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0FSDLY, arg->Q0fsdly)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1FSDLY, arg->I1fsdly)
            | CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1FSDLY, arg->Q1fsdly);
    regs[1] = CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSF, arg->fsf)
    		| CSL_FMK(DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSFM, arg->fsfm);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTrigConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1FSDLY
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSF
 *       DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1BUS_SEL
 *       DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSFM
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
CSL_IDEF_INLINE void
dfeFl_CbGetTrigConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		regs = &hDfeCb->regs->trigger_monitor_a_config;
		break;
	case DFE_FL_CB_TRIG_B:
		regs = &hDfeCb->regs->trigger_monitor_b_config;
		break;
    default:
    	return;
	}
	arg->I0bus_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0BUS_SEL);
	arg->Q0bus_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0BUS_SEL);
	arg->I1bus_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1BUS_SEL);
	arg->Q1bus_sel = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1BUS_SEL);
	arg->I0fsdly = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I0FSDLY);
	arg->Q0fsdly = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q0FSDLY);
	arg->I1fsdly = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_I1FSDLY);
	arg->Q1fsdly = CSL_FEXT(regs[0], DFE_CB_TRIGGER_MONITOR_A_CONFIG_REG_TRIGA_Q1FSDLY);
    arg->fsf = CSL_FEXT(regs[1], DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSF);
    arg->fsfm = CSL_FEXT(regs[1], DFE_CB_TRIGGER_MONITOR_A_FSF_FSFM_REG_TRIGA_FSFM);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTrigTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGB_BLK1_T2_REG_TRIGB_BLK1_T2
 *       DFE_CB_TRIGA_BLK0_T2_REG_TRIGA_BLK0_T2
 *       DFE_CB_TRIGB_BLK0_LENGTH_REG_TRIGB_BLK0_LENGTH
 *       DFE_CB_TRIGA_BLK1_T2_REG_TRIGA_BLK1_T2
 *       DFE_CB_TRIGB_BLK0_T2_REG_TRIGB_BLK0_T2
 *       DFE_CB_TRIGB_BLK0_T1_REG_TRIGB_BLK0_T1
 *       DFE_CB_TRIGA_BLK1_T1_REG_TRIGA_BLK1_T1
 *       DFE_CB_TRIGA_BLK0_LENGTH_REG_TRIGA_BLK0_LENGTH
 *       DFE_CB_TRIGB_BLK1_T1_REG_TRIGB_BLK1_T1
 *       DFE_CB_TRIGA_BLK1_LENGTH_REG_TRIGA_BLK1_LENGTH
 *       DFE_CB_TRIGA_BLK0_T1_REG_TRIGA_BLK0_T1
 *       DFE_CB_TRIGB_BLK1_LENGTH_REG_TRIGB_BLK1_LENGTH
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTrigTh(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigTh *arg)
{
	switch(arg->cbTrigBlk)
	{
	case DFE_FL_CB_TRIG_A_BLK0:
	{
		CSL_FINS(hDfeCb->regs->triga_blk0_length, DFE_CB_TRIGA_BLK0_LENGTH_REG_TRIGA_BLK0_LENGTH, arg->length);
		CSL_FINS(hDfeCb->regs->triga_blk0_t1, DFE_CB_TRIGA_BLK0_T1_REG_TRIGA_BLK0_T1, arg->T1);
		CSL_FINS(hDfeCb->regs->triga_blk0_t2, DFE_CB_TRIGA_BLK0_T2_REG_TRIGA_BLK0_T2, arg->T2);
		break;
	}
	case DFE_FL_CB_TRIG_A_BLK1:
	{
		CSL_FINS(hDfeCb->regs->triga_blk1_length, DFE_CB_TRIGA_BLK1_LENGTH_REG_TRIGA_BLK1_LENGTH, arg->length);
		CSL_FINS(hDfeCb->regs->triga_blk1_t1, DFE_CB_TRIGA_BLK1_T1_REG_TRIGA_BLK1_T1, arg->T1);
		CSL_FINS(hDfeCb->regs->triga_blk1_t2, DFE_CB_TRIGA_BLK1_T2_REG_TRIGA_BLK1_T2, arg->T2);
		break;
	}
	case DFE_FL_CB_TRIG_B_BLK0:
	{
		CSL_FINS(hDfeCb->regs->trigb_blk0_length, DFE_CB_TRIGB_BLK0_LENGTH_REG_TRIGB_BLK0_LENGTH, arg->length);
		CSL_FINS(hDfeCb->regs->trigb_blk0_t1, DFE_CB_TRIGB_BLK0_T1_REG_TRIGB_BLK0_T1, arg->T1);
		CSL_FINS(hDfeCb->regs->trigb_blk0_t2, DFE_CB_TRIGB_BLK0_T2_REG_TRIGB_BLK0_T2, arg->T2);
		break;
	}
	case DFE_FL_CB_TRIG_B_BLK1:
	{
		CSL_FINS(hDfeCb->regs->trigb_blk1_length, DFE_CB_TRIGB_BLK1_LENGTH_REG_TRIGB_BLK1_LENGTH, arg->length);
		CSL_FINS(hDfeCb->regs->trigb_blk1_t1, DFE_CB_TRIGB_BLK1_T1_REG_TRIGB_BLK1_T1, arg->T1);
		CSL_FINS(hDfeCb->regs->trigb_blk1_t2, DFE_CB_TRIGB_BLK1_T2_REG_TRIGB_BLK1_T2, arg->T2);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTrigTh
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGB_BLK1_T2_REG_TRIGB_BLK1_T2
 *       DFE_CB_TRIGA_BLK0_T2_REG_TRIGA_BLK0_T2
 *       DFE_CB_TRIGB_BLK0_LENGTH_REG_TRIGB_BLK0_LENGTH
 *       DFE_CB_TRIGA_BLK1_T2_REG_TRIGA_BLK1_T2
 *       DFE_CB_TRIGB_BLK0_T2_REG_TRIGB_BLK0_T2
 *       DFE_CB_TRIGB_BLK0_T1_REG_TRIGB_BLK0_T1
 *       DFE_CB_TRIGA_BLK1_T1_REG_TRIGA_BLK1_T1
 *       DFE_CB_TRIGA_BLK0_LENGTH_REG_TRIGA_BLK0_LENGTH
 *       DFE_CB_TRIGB_BLK1_T1_REG_TRIGB_BLK1_T1
 *       DFE_CB_TRIGA_BLK1_LENGTH_REG_TRIGA_BLK1_LENGTH
 *       DFE_CB_TRIGA_BLK0_T1_REG_TRIGA_BLK0_T1
 *       DFE_CB_TRIGB_BLK1_LENGTH_REG_TRIGB_BLK1_LENGTH
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
CSL_IDEF_INLINE void
dfeFl_CbGetTrigTh(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigTh *arg)
{
	switch(arg->cbTrigBlk)
	{
	case DFE_FL_CB_TRIG_A_BLK0:
	{
		arg->length = CSL_FEXT(hDfeCb->regs->triga_blk0_length, DFE_CB_TRIGA_BLK0_LENGTH_REG_TRIGA_BLK0_LENGTH);
		arg->T1 = CSL_FEXT(hDfeCb->regs->triga_blk0_t1, DFE_CB_TRIGA_BLK0_T1_REG_TRIGA_BLK0_T1);
		arg->T2 = CSL_FEXT(hDfeCb->regs->triga_blk0_t2, DFE_CB_TRIGA_BLK0_T2_REG_TRIGA_BLK0_T2);
		break;
	}
	case DFE_FL_CB_TRIG_A_BLK1:
	{
		arg->length = CSL_FEXT(hDfeCb->regs->triga_blk1_length, DFE_CB_TRIGA_BLK1_LENGTH_REG_TRIGA_BLK1_LENGTH);
		arg->T1 = CSL_FEXT(hDfeCb->regs->triga_blk1_t1, DFE_CB_TRIGA_BLK1_T1_REG_TRIGA_BLK1_T1);
		arg->T2 = CSL_FEXT(hDfeCb->regs->triga_blk1_t2, DFE_CB_TRIGA_BLK1_T2_REG_TRIGA_BLK1_T2);
		break;
	}
	case DFE_FL_CB_TRIG_B_BLK0:
	{
		arg->length = CSL_FEXT(hDfeCb->regs->trigb_blk0_length, DFE_CB_TRIGB_BLK0_LENGTH_REG_TRIGB_BLK0_LENGTH);
		arg->T1 = CSL_FEXT(hDfeCb->regs->trigb_blk0_t1, DFE_CB_TRIGB_BLK0_T1_REG_TRIGB_BLK0_T1);
		arg->T2 = CSL_FEXT(hDfeCb->regs->trigb_blk0_t2, DFE_CB_TRIGB_BLK0_T2_REG_TRIGB_BLK0_T2);
		break;
	}
	case DFE_FL_CB_TRIG_B_BLK1:
	{
		arg->length = CSL_FEXT(hDfeCb->regs->trigb_blk1_length, DFE_CB_TRIGB_BLK1_LENGTH_REG_TRIGB_BLK1_LENGTH);
		arg->T1 = CSL_FEXT(hDfeCb->regs->trigb_blk1_t1, DFE_CB_TRIGB_BLK1_T1_REG_TRIGB_BLK1_T1);
		arg->T2 = CSL_FEXT(hDfeCb->regs->trigb_blk1_t2, DFE_CB_TRIGB_BLK1_T2_REG_TRIGB_BLK1_T2);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTrigDecoder
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_DECODER_REG_TRIGGER_MONITOR_DECODER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTrigDecoder(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->trigger_monitor_decoder, DFE_CB_TRIGGER_MONITOR_DECODER_REG_TRIGGER_MONITOR_DECODER, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTrigDecoder
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGGER_MONITOR_DECODER_REG_TRIGGER_MONITOR_DECODER
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
CSL_IDEF_INLINE void
dfeFl_CbGetTrigDecoder(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->trigger_monitor_decoder, DFE_CB_TRIGGER_MONITOR_DECODER_REG_TRIGGER_MONITOR_DECODER);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_MODE_REG_GSG4_MODE
 *       DFE_CB_GSG_MODE_REG_GSG1_MODE
 *       DFE_CB_GSG_MODE_REG_GSG3_MODE
 *       DFE_CB_GSG_MODE_REG_GSG0_MODE
 *       DFE_CB_GSG_MODE_REG_GSG5_MODE
 *       DFE_CB_GSG_MODE_REG_GSG2_MODE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgMode(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgMode *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG0_MODE, arg->data);
		break;
	case DFE_FL_CB_GSG1:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG1_MODE, arg->data);
		break;
	case DFE_FL_CB_GSG2:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG2_MODE, arg->data);
		break;
	case DFE_FL_CB_GSG3:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG3_MODE, arg->data);
		break;
	case DFE_FL_CB_GSG4:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG4_MODE, arg->data);
		break;
	case DFE_FL_CB_GSG5:
		CSL_FINS(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG5_MODE, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_MODE_REG_GSG4_MODE
 *       DFE_CB_GSG_MODE_REG_GSG1_MODE
 *       DFE_CB_GSG_MODE_REG_GSG3_MODE
 *       DFE_CB_GSG_MODE_REG_GSG0_MODE
 *       DFE_CB_GSG_MODE_REG_GSG5_MODE
 *       DFE_CB_GSG_MODE_REG_GSG2_MODE
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgMode(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgMode *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG0_MODE);
		break;
	case DFE_FL_CB_GSG1:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG1_MODE);
		break;
	case DFE_FL_CB_GSG2:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG2_MODE);
		break;
	case DFE_FL_CB_GSG3:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG3_MODE);
		break;
	case DFE_FL_CB_GSG4:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG4_MODE);
		break;
	case DFE_FL_CB_GSG5:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_mode, DFE_CB_GSG_MODE_REG_GSG5_MODE);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG5_DELAYFROMSYNC_REG_GSG5_DELAYFROMSYNC
 *       DFE_CB_GSG0_DELAYFROMSYNC_REG_GSG0_DELAYFROMSYNC
 *       DFE_CB_GSG1_DELAYFROMSYNC_REG_GSG1_DELAYFROMSYNC
 *       DFE_CB_GSG3_DELAYFROMSYNC_REG_GSG3_DELAYFROMSYNC
 *       DFE_CB_GSG2_DELAYFROMSYNC_REG_GSG2_DELAYFROMSYNC
 *       DFE_CB_GSG4_DELAYFROMSYNC_REG_GSG4_DELAYFROMSYNC
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgDelay(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgDelay *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		CSL_FINS(hDfeCb->regs->gsg0_delayfromsync, DFE_CB_GSG0_DELAYFROMSYNC_REG_GSG0_DELAYFROMSYNC, arg->data);
		break;
	case DFE_FL_CB_GSG1:
		CSL_FINS(hDfeCb->regs->gsg1_delayfromsync, DFE_CB_GSG1_DELAYFROMSYNC_REG_GSG1_DELAYFROMSYNC, arg->data);
		break;
	case DFE_FL_CB_GSG2:
		CSL_FINS(hDfeCb->regs->gsg2_delayfromsync, DFE_CB_GSG2_DELAYFROMSYNC_REG_GSG2_DELAYFROMSYNC, arg->data);
		break;
	case DFE_FL_CB_GSG3:
		CSL_FINS(hDfeCb->regs->gsg3_delayfromsync, DFE_CB_GSG3_DELAYFROMSYNC_REG_GSG3_DELAYFROMSYNC, arg->data);
		break;
	case DFE_FL_CB_GSG4:
		CSL_FINS(hDfeCb->regs->gsg4_delayfromsync, DFE_CB_GSG4_DELAYFROMSYNC_REG_GSG4_DELAYFROMSYNC, arg->data);
		break;
	case DFE_FL_CB_GSG5:
		CSL_FINS(hDfeCb->regs->gsg5_delayfromsync, DFE_CB_GSG5_DELAYFROMSYNC_REG_GSG5_DELAYFROMSYNC, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG5_DELAYFROMSYNC_REG_GSG5_DELAYFROMSYNC
 *       DFE_CB_GSG0_DELAYFROMSYNC_REG_GSG0_DELAYFROMSYNC
 *       DFE_CB_GSG1_DELAYFROMSYNC_REG_GSG1_DELAYFROMSYNC
 *       DFE_CB_GSG3_DELAYFROMSYNC_REG_GSG3_DELAYFROMSYNC
 *       DFE_CB_GSG2_DELAYFROMSYNC_REG_GSG2_DELAYFROMSYNC
 *       DFE_CB_GSG4_DELAYFROMSYNC_REG_GSG4_DELAYFROMSYNC
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgDelay(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgDelay *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg0_delayfromsync, DFE_CB_GSG0_DELAYFROMSYNC_REG_GSG0_DELAYFROMSYNC);
		break;
	case DFE_FL_CB_GSG1:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg1_delayfromsync, DFE_CB_GSG1_DELAYFROMSYNC_REG_GSG1_DELAYFROMSYNC);
		break;
	case DFE_FL_CB_GSG2:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg2_delayfromsync, DFE_CB_GSG2_DELAYFROMSYNC_REG_GSG2_DELAYFROMSYNC);
		break;
	case DFE_FL_CB_GSG3:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg3_delayfromsync, DFE_CB_GSG3_DELAYFROMSYNC_REG_GSG3_DELAYFROMSYNC);
		break;
	case DFE_FL_CB_GSG4:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg4_delayfromsync, DFE_CB_GSG4_DELAYFROMSYNC_REG_GSG4_DELAYFROMSYNC);
		break;
	case DFE_FL_CB_GSG5:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg5_delayfromsync, DFE_CB_GSG5_DELAYFROMSYNC_REG_GSG5_DELAYFROMSYNC);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgTimer
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG0_TIMER3_REG_GSG0_TIMER3
 *       DFE_CB_GSG0_TIMER2_REG_GSG0_TIMER2
 *       DFE_CB_GSG0_TIMER5_REG_GSG0_TIMER5
 *       DFE_CB_GSG0_TIMER1_REG_GSG0_TIMER1
 *       DFE_CB_GSG0_TIMER4_REG_GSG0_TIMER4
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgTimer(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgTimer *arg)
{
	volatile uint32_t *regs;

	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		regs = &hDfeCb->regs->gsg0_timer1;
		break;
	case DFE_FL_CB_GSG1:
		regs = &hDfeCb->regs->gsg1_timer1;
		break;
	case DFE_FL_CB_GSG2:
		regs = &hDfeCb->regs->gsg2_timer1;
		break;
	case DFE_FL_CB_GSG3:
		regs = &hDfeCb->regs->gsg3_timer1;
		break;
	case DFE_FL_CB_GSG4:
		regs = &hDfeCb->regs->gsg4_timer1;
		break;
	case DFE_FL_CB_GSG5:
		regs = &hDfeCb->regs->gsg5_timer1;
		break;
    default:
    	return;
	}

	regs[0] = CSL_FMK(DFE_CB_GSG0_TIMER1_REG_GSG0_TIMER1, arg->timer1);
	regs[1] = CSL_FMK(DFE_CB_GSG0_TIMER2_REG_GSG0_TIMER2, arg->timer2);
	regs[2] = CSL_FMK(DFE_CB_GSG0_TIMER3_REG_GSG0_TIMER3, arg->timer3);
	regs[3] = CSL_FMK(DFE_CB_GSG0_TIMER4_REG_GSG0_TIMER4, arg->timer4);
	regs[4] = CSL_FMK(DFE_CB_GSG0_TIMER5_REG_GSG0_TIMER5, arg->timer5);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgTimer
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG0_TIMER3_REG_GSG0_TIMER3
 *       DFE_CB_GSG0_TIMER2_REG_GSG0_TIMER2
 *       DFE_CB_GSG0_TIMER5_REG_GSG0_TIMER5
 *       DFE_CB_GSG0_TIMER1_REG_GSG0_TIMER1
 *       DFE_CB_GSG0_TIMER4_REG_GSG0_TIMER4
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgTimer(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgTimer *arg)
{
	volatile uint32_t *regs;

	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		regs = &hDfeCb->regs->gsg0_timer1;
		break;
	case DFE_FL_CB_GSG1:
		regs = &hDfeCb->regs->gsg1_timer1;
		break;
	case DFE_FL_CB_GSG2:
		regs = &hDfeCb->regs->gsg2_timer1;
		break;
	case DFE_FL_CB_GSG3:
		regs = &hDfeCb->regs->gsg3_timer1;
		break;
	case DFE_FL_CB_GSG4:
		regs = &hDfeCb->regs->gsg4_timer1;
		break;
	case DFE_FL_CB_GSG5:
		regs = &hDfeCb->regs->gsg5_timer1;
		break;
    default:
    	return;
	}

	arg->timer1 = CSL_FEXT(regs[0], DFE_CB_GSG0_TIMER1_REG_GSG0_TIMER1);
	arg->timer2 = CSL_FEXT(regs[1], DFE_CB_GSG0_TIMER2_REG_GSG0_TIMER2);
	arg->timer3 = CSL_FEXT(regs[2], DFE_CB_GSG0_TIMER3_REG_GSG0_TIMER3);
	arg->timer4 = CSL_FEXT(regs[3], DFE_CB_GSG0_TIMER4_REG_GSG0_TIMER4);
	arg->timer5 = CSL_FEXT(regs[4], DFE_CB_GSG0_TIMER5_REG_GSG0_TIMER5);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SSEL_REG_GSG1_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG5_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG0_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG4_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG3_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG2_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgSsel *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG0_SSEL, arg->data);
		break;
	case DFE_FL_CB_GSG1:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG1_SSEL, arg->data);
		break;
	case DFE_FL_CB_GSG2:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG2_SSEL, arg->data);
		break;
	case DFE_FL_CB_GSG3:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG3_SSEL, arg->data);
		break;
	case DFE_FL_CB_GSG4:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG4_SSEL, arg->data);
		break;
	case DFE_FL_CB_GSG5:
		CSL_FINS(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG5_SSEL, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SSEL_REG_GSG1_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG5_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG0_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG4_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG3_SSEL
 *       DFE_CB_GSG_SSEL_REG_GSG2_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgSsel *arg)
{
	switch(arg->cbGsg)
	{
	case DFE_FL_CB_GSG0:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG0_SSEL);
		break;
	case DFE_FL_CB_GSG1:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG1_SSEL);
		break;
	case DFE_FL_CB_GSG2:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG2_SSEL);
		break;
	case DFE_FL_CB_GSG3:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG3_SSEL);
		break;
	case DFE_FL_CB_GSG4:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG4_SSEL);
		break;
	case DFE_FL_CB_GSG5:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_ssel, DFE_CB_GSG_SSEL_REG_GSG5_SSEL);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgCsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_REF_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_FB_GSG_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgCsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgCsel *arg)
{
	CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_REF_GSG_SEL, arg->ref_sel);
	CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_FB_GSG_SEL, arg->fb_sel);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgCsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_REF_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_FB_GSG_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgCsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgCsel *arg)
{
	arg->ref_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_REF_GSG_SEL);
	arg->fb_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_C_FB_GSG_SEL);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgFsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_FB_ANT3_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT2_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT0_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT1_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT2_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_REF_ANT3_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT1_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT0_GSG_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgFsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgFsel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
	{
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT0_GSG_SEL, arg->ref_sel);
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT0_GSG_SEL, arg->fb_sel);
		break;
	}
	case DFE_FL_CB_ANT1:
	{
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT1_GSG_SEL, arg->ref_sel);
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT1_GSG_SEL, arg->fb_sel);
		break;
	}
	case DFE_FL_CB_ANT2:
	{
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT2_GSG_SEL, arg->ref_sel);
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT2_GSG_SEL, arg->fb_sel);
		break;
	}
	case DFE_FL_CB_ANT3:
	{
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_REF_ANT3_GSG_SEL, arg->ref_sel);
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_FB_ANT3_GSG_SEL, arg->fb_sel);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgFsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_FB_ANT3_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT2_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT0_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT1_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT2_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_REF_ANT3_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT1_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT0_GSG_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgFsel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgFsel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
	{
		arg->ref_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT0_GSG_SEL);
		arg->fb_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT0_GSG_SEL);
		break;
	}
	case DFE_FL_CB_ANT1:
	{
		arg->ref_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT1_GSG_SEL);
		arg->fb_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT1_GSG_SEL);
		break;
	}
	case DFE_FL_CB_ANT2:
	{
		arg->ref_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_REF_ANT2_GSG_SEL);
		arg->fb_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part1, DFE_CB_GSG_SEQ_SEL_PART1_REG_CB_F_FB_ANT2_GSG_SEL);
		break;
	}
	case DFE_FL_CB_ANT3:
	{
		arg->ref_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_REF_ANT3_GSG_SEL);
		arg->fb_sel = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_CB_F_FB_ANT3_GSG_SEL);
		break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetGsgTrigSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGB_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGA_GSG_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetGsgTrigSel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgTrigSel *arg)
{
	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGA_GSG_SEL, arg->data);
		break;
	case DFE_FL_CB_TRIG_B:
		CSL_FINS(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGB_GSG_SEL, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetGsgTrigSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGB_GSG_SEL
 *       DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGA_GSG_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetGsgTrigSel(DfeFl_CbHandle hDfeCb, DfeFl_CbGsgTrigSel *arg)
{
	switch(arg->cbTrig)
	{
	case DFE_FL_CB_TRIG_A:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGA_GSG_SEL);
		break;
	case DFE_FL_CB_TRIG_B:
		arg->data = CSL_FEXT(hDfeCb->regs->gsg_seq_sel_part2, DFE_CB_GSG_SEQ_SEL_PART2_REG_TRIGB_GSG_SEL);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetSilentDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_SAMPLES
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_EN
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_THRESH
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetSilentDet(DfeFl_CbHandle hDfeCb, DfeFl_CbSilentDet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->silent_detect_setting;

    regs[0] = CSL_FMK(DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_EN, arg->enable)
    		| CSL_FMK(DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_SAMPLES, arg->samples)
            | CSL_FMK(DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_THRESH, arg->thresh);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetSilentDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_SAMPLES
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_EN
 *       DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_THRESH
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
CSL_IDEF_INLINE void
dfeFl_CbGetSilentDet(DfeFl_CbHandle hDfeCb, DfeFl_CbSilentDet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->silent_detect_setting;

    arg->enable = CSL_FEXT(regs[0], DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_EN);
    arg->samples = CSL_FEXT(regs[0], DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_SAMPLES);
    arg->thresh = CSL_FEXT(regs[0], DFE_CB_SILENT_DETECT_SETTING_REG_SILENT_DETECT_THRESH);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfChunkSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT3_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT0_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT2_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT1_CRITERIA_DISABLE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfChunkSel(DfeFl_CbHandle hDfeCb, DfeFl_CbfChunkSel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT0_CRITERIA_DISABLE, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT1_CRITERIA_DISABLE, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT2_CRITERIA_DISABLE, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT3_CRITERIA_DISABLE, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbfChunkSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT3_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT0_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT2_CRITERIA_DISABLE
 *       DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT1_CRITERIA_DISABLE
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbfChunkSel(DfeFl_CbHandle hDfeCb, DfeFl_CbfChunkSel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT0_CRITERIA_DISABLE);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT1_CRITERIA_DISABLE);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT2_CRITERIA_DISABLE);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_chunk_selection, DFE_CB_CB_F_CHUNK_SELECTION_REG_ANT3_CRITERIA_DISABLE);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfBrokenChainDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_REF_FB_POWERRATIO
 *       DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_POWERTH
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfBrokenChainDet(DfeFl_CbHandle hDfeCb, DfeFl_CbfBrokenChainDet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_f_broken_chain_detection;

    regs[0] = CSL_FMK(DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_POWERTH, arg->powerTH)
    		| CSL_FMK(DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_REF_FB_POWERRATIO, arg->ref_fb_powerRatio);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbfBrokenChainDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_REF_FB_POWERRATIO
 *       DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_POWERTH
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbfBrokenChainDet(DfeFl_CbHandle hDfeCb, DfeFl_CbfBrokenChainDet *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_f_broken_chain_detection;

    arg->powerTH = CSL_FEXT(regs[0], DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_POWERTH);
    arg->ref_fb_powerRatio = CSL_FEXT(regs[0], DFE_CB_CB_F_BROKEN_CHAIN_DETECTION_REG_CB_F_REF_FB_POWERRATIO);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfDeltaPow
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT1
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT0
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT3
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfDeltaPow(DfeFl_CbHandle hDfeCb, DfeFl_CbfDeltaPow *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT0, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT1, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT2, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT3, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbfDeltaPow
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT1
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT0
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT3
 *       DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT2
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbfDeltaPow(DfeFl_CbHandle hDfeCb, DfeFl_CbfDeltaPow *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_deltapowerinlinear, DFE_CB_CB_F_DELTAPOWERINLINEAR_REG_CB_F_DELTAPOWER_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfBadbuffDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT3
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT2
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT1
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfBadbuffDet(DfeFl_CbHandle hDfeCb, DfeFl_CbfBadbuffDet *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT0, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT1, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT2, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT3, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbfBadbuffDet
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT3
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT2
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT1
 *       DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT0
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbfBadbuffDet(DfeFl_CbHandle hDfeCb, DfeFl_CbfBadbuffDet *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_badbuffer_detection_en, DFE_CB_CB_F_BADBUFFER_DETECTION_EN_REG_BAD_BUFFER_DETECTION_EN_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetPmSyncDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT3_REG_POWER_MONITOR_SYNC_DLY_ANT3
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT1_REG_POWER_MONITOR_SYNC_DLY_ANT1
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT0_REG_POWER_MONITOR_SYNC_DLY_ANT0
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT2_REG_POWER_MONITOR_SYNC_DLY_ANT2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetPmSyncDly(DfeFl_CbHandle hDfeCb, DfeFl_CbPmSyncDly *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->power_monitor_sync_dly_ant0, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT0_REG_POWER_MONITOR_SYNC_DLY_ANT0, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->power_monitor_sync_dly_ant1, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT1_REG_POWER_MONITOR_SYNC_DLY_ANT1, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->power_monitor_sync_dly_ant2, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT2_REG_POWER_MONITOR_SYNC_DLY_ANT2, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->power_monitor_sync_dly_ant3, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT3_REG_POWER_MONITOR_SYNC_DLY_ANT3, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetPmSyncDly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT3_REG_POWER_MONITOR_SYNC_DLY_ANT3
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT1_REG_POWER_MONITOR_SYNC_DLY_ANT1
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT0_REG_POWER_MONITOR_SYNC_DLY_ANT0
 *       DFE_CB_POWER_MONITOR_SYNC_DLY_ANT2_REG_POWER_MONITOR_SYNC_DLY_ANT2
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
CSL_IDEF_INLINE void
dfeFl_CbGetPmSyncDly(DfeFl_CbHandle hDfeCb, DfeFl_CbPmSyncDly *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_sync_dly_ant0, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT0_REG_POWER_MONITOR_SYNC_DLY_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_sync_dly_ant1, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT1_REG_POWER_MONITOR_SYNC_DLY_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_sync_dly_ant2, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT2_REG_POWER_MONITOR_SYNC_DLY_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_sync_dly_ant3, DFE_CB_POWER_MONITOR_SYNC_DLY_ANT3_REG_POWER_MONITOR_SYNC_DLY_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetPmIntgPd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT3_REG_POWER_MONITOR_INTG_PD_ANT3
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT2_REG_POWER_MONITOR_INTG_PD_ANT2
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT1_REG_POWER_MONITOR_INTG_PD_ANT1
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT0_REG_POWER_MONITOR_INTG_PD_ANT0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetPmIntgPd(DfeFl_CbHandle hDfeCb, DfeFl_CbPmIntgPd *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->power_monitor_intg_pd_ant0, DFE_CB_POWER_MONITOR_INTG_PD_ANT0_REG_POWER_MONITOR_INTG_PD_ANT0, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->power_monitor_intg_pd_ant1, DFE_CB_POWER_MONITOR_INTG_PD_ANT1_REG_POWER_MONITOR_INTG_PD_ANT1, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->power_monitor_intg_pd_ant2, DFE_CB_POWER_MONITOR_INTG_PD_ANT2_REG_POWER_MONITOR_INTG_PD_ANT2, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->power_monitor_intg_pd_ant3, DFE_CB_POWER_MONITOR_INTG_PD_ANT3_REG_POWER_MONITOR_INTG_PD_ANT3, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetPmIntgPd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT3_REG_POWER_MONITOR_INTG_PD_ANT3
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT2_REG_POWER_MONITOR_INTG_PD_ANT2
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT1_REG_POWER_MONITOR_INTG_PD_ANT1
 *       DFE_CB_POWER_MONITOR_INTG_PD_ANT0_REG_POWER_MONITOR_INTG_PD_ANT0
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
CSL_IDEF_INLINE void
dfeFl_CbGetPmIntgPd(DfeFl_CbHandle hDfeCb, DfeFl_CbPmIntgPd *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_intg_pd_ant0, DFE_CB_POWER_MONITOR_INTG_PD_ANT0_REG_POWER_MONITOR_INTG_PD_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_intg_pd_ant1, DFE_CB_POWER_MONITOR_INTG_PD_ANT1_REG_POWER_MONITOR_INTG_PD_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_intg_pd_ant2, DFE_CB_POWER_MONITOR_INTG_PD_ANT2_REG_POWER_MONITOR_INTG_PD_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->power_monitor_intg_pd_ant3, DFE_CB_POWER_MONITOR_INTG_PD_ANT3_REG_POWER_MONITOR_INTG_PD_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetPmConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0FSDLY
 *       DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSFM
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0BUS_SEL
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0BUS_SEL
 *       DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSF
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0FSDLY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetPmConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbPmCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		regs = &hDfeCb->regs->power_monitor_config_ant0;
		break;
	case DFE_FL_CB_ANT1:
		regs = &hDfeCb->regs->power_monitor_config_ant1;
		break;
	case DFE_FL_CB_ANT2:
		regs = &hDfeCb->regs->power_monitor_config_ant2;
		break;
	case DFE_FL_CB_ANT3:
		regs = &hDfeCb->regs->power_monitor_config_ant3;
		break;
    default:
    	return;
	}
    regs[0] = CSL_FMK(DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0BUS_SEL, arg->I0bus_sel)
    		| CSL_FMK(DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0BUS_SEL, arg->Q0bus_sel)
            | CSL_FMK(DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0FSDLY, arg->I0fsdly)
            | CSL_FMK(DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0FSDLY, arg->Q0fsdly);
    regs[1] = CSL_FMK(DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSF, arg->fsf)
    		| CSL_FMK(DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSFM, arg->fsfm);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetPmConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0FSDLY
 *       DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSFM
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0BUS_SEL
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0BUS_SEL
 *       DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSF
 *       DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0FSDLY
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
CSL_IDEF_INLINE void
dfeFl_CbGetPmConfig(DfeFl_CbHandle hDfeCb, DfeFl_CbPmCfg *arg)
{
    volatile uint32_t *regs;

	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		regs = &hDfeCb->regs->power_monitor_config_ant0;
		break;
	case DFE_FL_CB_ANT1:
		regs = &hDfeCb->regs->power_monitor_config_ant1;
		break;
	case DFE_FL_CB_ANT2:
		regs = &hDfeCb->regs->power_monitor_config_ant2;
		break;
	case DFE_FL_CB_ANT3:
		regs = &hDfeCb->regs->power_monitor_config_ant3;
		break;
    default:
    	return;
	}
	arg->I0bus_sel = CSL_FEXT(regs[0], DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0BUS_SEL);
	arg->Q0bus_sel = CSL_FEXT(regs[0], DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0BUS_SEL);
	arg->I0fsdly = CSL_FEXT(regs[0], DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_I0FSDLY);
	arg->Q0fsdly = CSL_FEXT(regs[0], DFE_CB_POWER_MONITOR_CONFIG_ANT0_REG_POWER_MONITOR_ANT0_Q0FSDLY);
	arg->fsf = CSL_FEXT(regs[1], DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSF);
	arg->fsfm = CSL_FEXT(regs[1], DFE_CB_POWER_MONITOR_ANT0_FSF_FSFM_REG_POWER_MONITOR_ANT0_FSFM);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetPmNodeSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_NODE_SEL_REG_POWER_MONITOR_SEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetPmNodeSel(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->power_monitor_node_sel, DFE_CB_POWER_MONITOR_NODE_SEL_REG_POWER_MONITOR_SEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetPmNodeSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_POWER_MONITOR_NODE_SEL_REG_POWER_MONITOR_SEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetPmNodeSel(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->power_monitor_node_sel, DFE_CB_POWER_MONITOR_NODE_SEL_REG_POWER_MONITOR_SEL);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetPmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT1_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT3_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT0_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT2_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetPmSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbPmSsel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		CSL_FINS(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT0_SSEL, arg->data);
		break;
	case DFE_FL_CB_ANT1:
		CSL_FINS(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT1_SSEL, arg->data);
		break;
	case DFE_FL_CB_ANT2:
		CSL_FINS(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT2_SSEL, arg->data);
		break;
	case DFE_FL_CB_ANT3:
		CSL_FINS(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT3_SSEL, arg->data);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbGetPmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT1_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT3_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT0_SSEL
 *       DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT2_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetPmSsel(DfeFl_CbHandle hDfeCb, DfeFl_CbPmSsel *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT0_SSEL);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT1_SSEL);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT2_SSEL);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_sync_select_part4, DFE_CB_CB_SYNC_SELECT_PART4_REG_CB_F_POWERMONITOR_ANT3_SSEL);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetSourcingCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_SIZE
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_FSL
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_REPEAT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetSourcingCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbSrcCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_sourcing_control;

    regs[0] = CSL_FMK(DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_FSL, arg->fsl)
    		| CSL_FMK(DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_SIZE, arg->size)
    		| CSL_FMK(DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_REPEAT, arg->repeat);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetSourcingCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_SIZE
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_FSL
 *       DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_REPEAT
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
CSL_IDEF_INLINE void
dfeFl_CbGetSourcingCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbSrcCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_sourcing_control;

    arg->fsl = CSL_FEXT(regs[0], DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_FSL);
    arg->size = CSL_FEXT(regs[0], DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_SIZE);
    arg->repeat = CSL_FEXT(regs[0], DFE_CB_CB_SOURCING_CONTROL_REG_CB_SC_REPEAT);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetSrcNodeCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_CDFR_TO_DPD
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_FB
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DPD_TO_TX
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_SUM_TO_CFR
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_BB_TO_DDUC
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_TX_TO_JESD
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DDUC_TO_BB
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_RX
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_RX_TO_DDUC
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_FB_TO_DDUC
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetSrcNodeCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbSrcNodeCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_src_node_control;

    regs[0] = CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_BB_TO_DDUC, arg->bb_to_dduc)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_SUM_TO_CFR, arg->sum_to_cfr)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_CDFR_TO_DPD, arg->cdfr_to_dpd)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DPD_TO_TX, arg->dpd_to_tx)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_TX_TO_JESD, arg->tx_to_jesd)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_RX, arg->jesd_to_rx)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_FB, arg->jesd_to_fb)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_RX_TO_DDUC, arg->rx_to_dduc)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_FB_TO_DDUC, arg->fb_to_dduc)
    		| CSL_FMK(DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DDUC_TO_BB, arg->dduc_to_bb);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetSrcNodeCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_CDFR_TO_DPD
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_FB
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DPD_TO_TX
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_SUM_TO_CFR
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_BB_TO_DDUC
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_TX_TO_JESD
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DDUC_TO_BB
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_RX
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_RX_TO_DDUC
 *       DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_FB_TO_DDUC
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
CSL_IDEF_INLINE void
dfeFl_CbGetSrcNodeCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbSrcNodeCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->cb_src_node_control;

    arg->bb_to_dduc = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_BB_TO_DDUC);
    arg->sum_to_cfr = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_SUM_TO_CFR);
    arg->cdfr_to_dpd = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_CDFR_TO_DPD);
    arg->dpd_to_tx = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DPD_TO_TX);
    arg->tx_to_jesd = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_TX_TO_JESD);
    arg->jesd_to_rx = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_RX);
    arg->jesd_to_fb = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_JESD_TO_FB);
    arg->rx_to_dduc = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_RX_TO_DDUC);
    arg->fb_to_dduc = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_FB_TO_DDUC);
    arg->dduc_to_bb = CSL_FEXT(regs[0], DFE_CB_CB_SRC_NODE_CONTROL_REG_CB_SRC_DDUC_TO_BB);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTimeStep
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TIME_STEP_REG_TIME_STEP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTimeStep(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
	CSL_FINS(hDfeCb->regs->cb_time_step, DFE_CB_CB_TIME_STEP_REG_TIME_STEP, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTimeStep
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TIME_STEP_REG_TIME_STEP
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
CSL_IDEF_INLINE void
dfeFl_CbGetTimeStep(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeCb->regs->cb_time_step, DFE_CB_CB_TIME_STEP_REG_TIME_STEP);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetResetInt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_RESET_INT_REG_RESET_INT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetResetInt(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->cb_reset_int, DFE_CB_CB_RESET_INT_REG_RESET_INT, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetResetInt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_RESET_INT_REG_RESET_INT
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
CSL_IDEF_INLINE void
dfeFl_CbGetResetInt(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->cb_reset_int, DFE_CB_CB_RESET_INT_REG_RESET_INT);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTddPeriod
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TDD_PERIOD_REG_TDD_PERIOD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTddPeriod(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->cb_tdd_period, DFE_CB_CB_TDD_PERIOD_REG_TDD_PERIOD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTddPeriod
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TDD_PERIOD_REG_TDD_PERIOD
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
CSL_IDEF_INLINE void
dfeFl_CbGetTddPeriod(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->cb_tdd_period, DFE_CB_CB_TDD_PERIOD_REG_TDD_PERIOD);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetTddOnOff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TDD_ON_0_REG_TDD_ON_0
 *       DFE_CB_CB_TDD_ON_1_REG_TDD_ON_1
 *       DFE_CB_CB_TDD_OFF_1_REG_TDD_OFF_1
 *       DFE_CB_CB_TDD_OFF_0_REG_TDD_OFF_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetTddOnOff(DfeFl_CbHandle hDfeCb, DfeFl_CbTddOnOff *arg)
{
	CSL_FINS(hDfeCb->regs->cb_tdd_on_0, DFE_CB_CB_TDD_ON_0_REG_TDD_ON_0, arg->tdd_on_0);
	CSL_FINS(hDfeCb->regs->cb_tdd_off_0, DFE_CB_CB_TDD_OFF_0_REG_TDD_OFF_0, arg->tdd_off_0);
	CSL_FINS(hDfeCb->regs->cb_tdd_on_1, DFE_CB_CB_TDD_ON_1_REG_TDD_ON_1, arg->tdd_on_1);
	CSL_FINS(hDfeCb->regs->cb_tdd_off_1, DFE_CB_CB_TDD_OFF_1_REG_TDD_OFF_1, arg->tdd_off_1);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetTddOnOff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_TDD_ON_0_REG_TDD_ON_0
 *       DFE_CB_CB_TDD_ON_1_REG_TDD_ON_1
 *       DFE_CB_CB_TDD_OFF_1_REG_TDD_OFF_1
 *       DFE_CB_CB_TDD_OFF_0_REG_TDD_OFF_0
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
CSL_IDEF_INLINE void
dfeFl_CbGetTddOnOff(DfeFl_CbHandle hDfeCb, DfeFl_CbTddOnOff *arg)
{
	arg->tdd_on_0 = CSL_FEXT(hDfeCb->regs->cb_tdd_on_0, DFE_CB_CB_TDD_ON_0_REG_TDD_ON_0);
	arg->tdd_off_0 = CSL_FEXT(hDfeCb->regs->cb_tdd_off_0, DFE_CB_CB_TDD_OFF_0_REG_TDD_OFF_0);
	arg->tdd_on_1 = CSL_FEXT(hDfeCb->regs->cb_tdd_on_1, DFE_CB_CB_TDD_ON_1_REG_TDD_ON_1);
	arg->tdd_off_1 = CSL_FEXT(hDfeCb->regs->cb_tdd_off_1, DFE_CB_CB_TDD_OFF_1_REG_TDD_OFF_1);
}


/** ============================================================================
 *   @n@b dfeFl_CbSetCbcStartSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_C_START_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbcStartSsel(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_C_START_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbcStartSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_C_START_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbcStartSsel(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_C_START_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbfStartSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_F_START_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbfStartSsel(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_F_START_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetCbfStartSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_F_START_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetCbfStartSsel(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
   	*arg = CSL_FEXT(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_F_START_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetSourceSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_SOURCE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetSourceSsel(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_SOURCE_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetSourceSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_SOURCE_SSEL
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
CSL_IDEF_INLINE void
dfeFl_CbGetSourceSsel(DfeFl_CbHandle hDfeCb, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_SOURCE_SSEL);
}

//CSL_IDEF_INLINE void
//dfeFl_CbSetPmSsel(DfeFl_CbHandle hDfeCb, uint32_t arg)
//{
//   	CSL_FINS(hDfeCb->regs->cb_sync_select_part1, DFE_CB_CB_SYNC_SELECT_PART1_REG_CB_F_POWERMONITOR_SSEL, arg);
//}

/** ============================================================================
 *   @n@b dfeFl_CbSetInitFracPhaseCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE
 *       DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetInitFracPhaseCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbInitFracPhCtrl *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeCb->regs->initial_fractional_phase_ctrl;

    regs[0] = CSL_FMK(DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE_EN, arg->enable)
    		| CSL_FMK(DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE, arg->frac_phase);
}

/** ============================================================================
 *   @n@b dfeFl_CbGetInitFracPhaseCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE
 *       DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE_EN
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
CSL_IDEF_INLINE void
dfeFl_CbGetInitFracPhaseCtrl(DfeFl_CbHandle hDfeCb, DfeFl_CbInitFracPhCtrl *arg)
{
    arg->enable = CSL_FEXT(hDfeCb->regs->initial_fractional_phase_ctrl, DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE_EN);
    arg->frac_phase = CSL_FEXT(hDfeCb->regs->initial_fractional_phase_ctrl, DFE_CB_INITIAL_FRACTIONAL_PHASE_CTRL_REG_INIT_FRAC_PHASE);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetInitSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetInitSsel(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->inits, DFE_CB_INITS_REG_INITS_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetInitClkGate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_INIT_CLK_GATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetInitClkGate(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->inits, DFE_CB_INITS_REG_INIT_CLK_GATE, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetInitState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_INIT_STATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetInitState(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->inits, DFE_CB_INITS_REG_INIT_STATE, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetInitClrData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_INITS_REG_CLEAR_DATA
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetInitClrData(DfeFl_CbHandle hDfeCb, uint32_t arg)
{
   	CSL_FINS(hDfeCb->regs->inits, DFE_CB_INITS_REG_CLEAR_DATA, arg);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbDoneFracCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_DONE_FRAC_CNT_REG_CBA_DONE_FRAC_CNT
 *       DFE_CB_DONE_FRAC_CNT_REG_CBD_DONE_FRAC_CNT
 *       DFE_CB_DONE_FRAC_CNT_REG_CBB_DONE_FRAC_CNT
 *       DFE_CB_DONE_FRAC_CNT_REG_CBC_DONE_FRAC_CNT
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbDoneFracCnt(DfeFl_CbHandle hDfeCb, DfeFl_CbDoneInfo *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->done_frac_cnt, DFE_CB_DONE_FRAC_CNT_REG_CBA_DONE_FRAC_CNT);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->done_frac_cnt, DFE_CB_DONE_FRAC_CNT_REG_CBB_DONE_FRAC_CNT);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->done_frac_cnt, DFE_CB_DONE_FRAC_CNT_REG_CBC_DONE_FRAC_CNT);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->done_frac_cnt, DFE_CB_DONE_FRAC_CNT_REG_CBD_DONE_FRAC_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbDoneAddr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUF_CD_DONE_ADDR_REG_CBC_DONE_ADDR
 *       DFE_CB_BUF_AB_DONE_ADDR_REG_CBA_DONE_ADDR
 *       DFE_CB_BUF_CD_DONE_ADDR_REG_CBD_DONE_ADDR
 *       DFE_CB_BUF_AB_DONE_ADDR_REG_CBB_DONE_ADDR
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbDoneAddr(DfeFl_CbHandle hDfeCb, DfeFl_CbDoneInfo *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->buf_ab_done_addr, DFE_CB_BUF_AB_DONE_ADDR_REG_CBA_DONE_ADDR);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->buf_ab_done_addr, DFE_CB_BUF_AB_DONE_ADDR_REG_CBB_DONE_ADDR);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->buf_cd_done_addr, DFE_CB_BUF_CD_DONE_ADDR_REG_CBC_DONE_ADDR);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->buf_cd_done_addr, DFE_CB_BUF_CD_DONE_ADDR_REG_CBD_DONE_ADDR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbDoneLenCnt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBC_DONE_LENGTH_CNT_REG_CBC_DONE_LENGTH_CNT
 *       DFE_CB_CBB_DONE_LENGTH_CNT_REG_CBB_DONE_LENGTH_CNT
 *       DFE_CB_CBD_DONE_LENGTH_CNT_REG_CBD_DONE_LENGTH_CNT
 *       DFE_CB_CBA_DONE_LENGTH_CNT_REG_CBA_DONE_LENGTH_CNT
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbDoneLenCnt(DfeFl_CbHandle hDfeCb, DfeFl_CbDoneInfo *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->cba_done_length_cnt, DFE_CB_CBA_DONE_LENGTH_CNT_REG_CBA_DONE_LENGTH_CNT);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->cbb_done_length_cnt, DFE_CB_CBB_DONE_LENGTH_CNT_REG_CBB_DONE_LENGTH_CNT);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->cbc_done_length_cnt, DFE_CB_CBC_DONE_LENGTH_CNT_REG_CBC_DONE_LENGTH_CNT);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->cbd_done_length_cnt, DFE_CB_CBD_DONE_LENGTH_CNT_REG_CBD_DONE_LENGTH_CNT);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbBufFull
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_BUFFER_FULL_FLAG_REG_CBD_FULL
 *       DFE_CB_BUFFER_FULL_FLAG_REG_CBC_FULL
 *       DFE_CB_BUFFER_FULL_FLAG_REG_CBB_FULL
 *       DFE_CB_BUFFER_FULL_FLAG_REG_CBA_FULL
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbBufFull(DfeFl_CbHandle hDfeCb, DfeFl_CbDoneInfo *arg)
{
	switch(arg->cbBuf)
	{
	case DFE_FL_CBA:
		arg->data = CSL_FEXT(hDfeCb->regs->buffer_full_flag, DFE_CB_BUFFER_FULL_FLAG_REG_CBA_FULL);
		break;
	case DFE_FL_CBB:
		arg->data = CSL_FEXT(hDfeCb->regs->buffer_full_flag, DFE_CB_BUFFER_FULL_FLAG_REG_CBB_FULL);
		break;
	case DFE_FL_CBC:
		arg->data = CSL_FEXT(hDfeCb->regs->buffer_full_flag, DFE_CB_BUFFER_FULL_FLAG_REG_CBC_FULL);
		break;
	case DFE_FL_CBD:
		arg->data = CSL_FEXT(hDfeCb->regs->buffer_full_flag, DFE_CB_BUFFER_FULL_FLAG_REG_CBD_FULL);
		break;
    default:
    	return;
	}
}

//CSL_IDEF_INLINE void
//dfeFl_CbQueryRefChunkDoneAddr(DfeFl_CbHandle hDfeCb, DfeFl_CbChunkDoneAddr *arg)
//{
//	switch(arg->cbChunk)
//	{
//	case DFE_FL_CB_CHUNK1:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk1_done_addr, DFE_CB_CHUNK1_DONE_ADDR_REG_REF_CHUNK1_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK2:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk2_done_addr, DFE_CB_CHUNK2_DONE_ADDR_REG_REF_CHUNK2_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK3:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk3_done_addr, DFE_CB_CHUNK3_DONE_ADDR_REG_REF_CHUNK3_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK4:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk4_done_addr, DFE_CB_CHUNK4_DONE_ADDR_REG_REF_CHUNK4_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK5:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk5_done_addr, DFE_CB_CHUNK5_DONE_ADDR_REG_REF_CHUNK5_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK6:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk6_done_addr, DFE_CB_CHUNK6_DONE_ADDR_REG_REF_CHUNK6_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK7:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk7_done_addr, DFE_CB_CHUNK7_DONE_ADDR_REG_REF_CHUNK7_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK8:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk8_done_addr, DFE_CB_CHUNK8_DONE_ADDR_REG_REF_CHUNK8_DONE_ADDR);
//		break;
//	}
//}
//
//CSL_IDEF_INLINE void
//dfeFl_CbQueryFbChunkDoneAddr(DfeFl_CbHandle hDfeCb, DfeFl_CbChunkDoneAddr *arg)
//{
//	switch(arg->cbChunk)
//	{
//	case DFE_FL_CB_CHUNK1:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk1_done_addr, DFE_CB_CHUNK1_DONE_ADDR_REG_FB_CHUNK1_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK2:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk2_done_addr, DFE_CB_CHUNK2_DONE_ADDR_REG_FB_CHUNK2_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK3:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk3_done_addr, DFE_CB_CHUNK3_DONE_ADDR_REG_FB_CHUNK3_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK4:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk4_done_addr, DFE_CB_CHUNK4_DONE_ADDR_REG_FB_CHUNK4_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK5:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk5_done_addr, DFE_CB_CHUNK5_DONE_ADDR_REG_FB_CHUNK5_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK6:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk6_done_addr, DFE_CB_CHUNK6_DONE_ADDR_REG_FB_CHUNK6_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK7:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk7_done_addr, DFE_CB_CHUNK7_DONE_ADDR_REG_FB_CHUNK7_DONE_ADDR);
//		break;
//	case DFE_FL_CB_CHUNK8:
//		arg->doneAddr = CSL_FEXT(hDfeCb->regs->chunk8_done_addr, DFE_CB_CHUNK8_DONE_ADDR_REG_FB_CHUNK8_DONE_ADDR);
//		break;
//	}
//}

/** ============================================================================
 *   @n@b dfeFl_CbQueryChunkDoneAddr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CBA_CHUNK7_8_DONE_ADDR_REG_CBA_CHUNK5_DONE_ADDR
 *       DFE_CB_CBA_CHUNK3_4_DONE_ADDR_REG_CBA_CHUNK3_DONE_ADDR
 *       DFE_CB_CBA_CHUNK5_6_DONE_ADDR_REG_CBA_CHUNK6_DONE_ADDR
 *       DFE_CB_CBA_CHUNK1_2_DONE_ADDR_REG_CBA_CHUNK1_DONE_ADDR
 *       DFE_CB_CBA_CHUNK1_2_DONE_ADDR_REG_CBA_CHUNK2_DONE_ADDR
 *       DFE_CB_CBA_CHUNK7_8_DONE_ADDR_REG_CBA_CHUNK6_DONE_ADDR
 *       DFE_CB_CBA_CHUNK3_4_DONE_ADDR_REG_CBA_CHUNK4_DONE_ADDR
 *       DFE_CB_CBA_CHUNK5_6_DONE_ADDR_REG_CBA_CHUNK5_DONE_ADDR
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
CSL_IDEF_INLINE void
dfeFl_CbQueryChunkDoneAddr(DfeFl_CbHandle hDfeCb, DfeFl_CbChunkDoneAddr *arg)
{
	volatile uint32_t *regs;
	switch(arg->cbChunk)
	{
	case DFE_FL_CB_CHUNK1:
		regs = &hDfeCb->regs->cba_chunk1_2_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK1_2_DONE_ADDR_REG_CBA_CHUNK1_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK2:
		regs = &hDfeCb->regs->cba_chunk1_2_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK1_2_DONE_ADDR_REG_CBA_CHUNK2_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK3:
		regs = &hDfeCb->regs->cba_chunk3_4_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK3_4_DONE_ADDR_REG_CBA_CHUNK3_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK4:
		regs = &hDfeCb->regs->cba_chunk3_4_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK3_4_DONE_ADDR_REG_CBA_CHUNK4_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK5:
		regs = &hDfeCb->regs->cba_chunk5_6_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK5_6_DONE_ADDR_REG_CBA_CHUNK5_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK6:
		regs = &hDfeCb->regs->cba_chunk5_6_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK5_6_DONE_ADDR_REG_CBA_CHUNK6_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK7:
		regs = &hDfeCb->regs->cba_chunk7_8_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK7_8_DONE_ADDR_REG_CBA_CHUNK5_DONE_ADDR);
		break;
	case DFE_FL_CB_CHUNK8:
		regs = &hDfeCb->regs->cba_chunk7_8_done_addr;
		arg->doneAddr = CSL_FEXT(regs[arg->cbBuf*4], DFE_CB_CBA_CHUNK7_8_DONE_ADDR_REG_CBA_CHUNK6_DONE_ADDR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryTrigOutpwr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_TRIGA_BLK0_OUTPWR_REG_TRIGA_BLK0_OUTPWR
 *       DFE_CB_TRIGA_BLK1_OUTPWR_REG_TRIGA_BLK1_OUTPWR
 *       DFE_CB_TRIGB_BLK0_OUTPWR_REG_TRIGB_BLK0_OUTPWR
 *       DFE_CB_TRIGB_BLK1_OUTPWR_REG_TRIGB_BLK1_OUTPWR
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
CSL_IDEF_INLINE void
dfeFl_CbQueryTrigOutpwr(DfeFl_CbHandle hDfeCb, DfeFl_CbTrigOutpwr *arg)
{
	switch(arg->cbTrigBlk)
	{
	case DFE_FL_CB_TRIG_A_BLK0:
		arg->data = CSL_FEXT(hDfeCb->regs->triga_blk0_outpwr, DFE_CB_TRIGA_BLK0_OUTPWR_REG_TRIGA_BLK0_OUTPWR);
		break;
	case DFE_FL_CB_TRIG_A_BLK1:
		arg->data = CSL_FEXT(hDfeCb->regs->triga_blk1_outpwr, DFE_CB_TRIGA_BLK1_OUTPWR_REG_TRIGA_BLK1_OUTPWR);
		break;
	case DFE_FL_CB_TRIG_B_BLK0:
		arg->data = CSL_FEXT(hDfeCb->regs->trigb_blk0_outpwr, DFE_CB_TRIGB_BLK0_OUTPWR_REG_TRIGB_BLK0_OUTPWR);
		break;
	case DFE_FL_CB_TRIG_B_BLK1:
		arg->data = CSL_FEXT(hDfeCb->regs->trigb_blk1_outpwr, DFE_CB_TRIGB_BLK1_OUTPWR_REG_TRIGB_BLK1_OUTPWR);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbfMaxrefPow
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         arg    [add content]
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
 *       DFE_CB_CB_F_MAXREFPOWER_ANT0_1_REG_CB_F_MAXREFPOWER_ANT0
 *       DFE_CB_CB_F_MAXREFPOWER_ANT0_1_REG_CB_F_MAXREFPOWER_ANT1
 *       DFE_CB_CB_F_MAXREFPOWER_ANT2_3_REG_CB_F_MAXREFPOWER_ANT2
 *       DFE_CB_CB_F_MAXREFPOWER_ANT2_3_REG_CB_F_MAXREFPOWER_ANT3
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbfMaxrefPow(DfeFl_CbHandle hDfeCb, DfeFl_CbfMaxrefPow *arg)
{
	switch(arg->cbAnt)
	{
	case DFE_FL_CB_ANT0:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_maxrefpower_ant0_1, DFE_CB_CB_F_MAXREFPOWER_ANT0_1_REG_CB_F_MAXREFPOWER_ANT0);
		break;
	case DFE_FL_CB_ANT1:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_maxrefpower_ant0_1, DFE_CB_CB_F_MAXREFPOWER_ANT0_1_REG_CB_F_MAXREFPOWER_ANT1);
		break;
	case DFE_FL_CB_ANT2:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_maxrefpower_ant2_3, DFE_CB_CB_F_MAXREFPOWER_ANT2_3_REG_CB_F_MAXREFPOWER_ANT2);
		break;
	case DFE_FL_CB_ANT3:
		arg->data = CSL_FEXT(hDfeCb->regs->cb_f_maxrefpower_ant2_3, DFE_CB_CB_F_MAXREFPOWER_ANT2_3_REG_CB_F_MAXREFPOWER_ANT3);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbMSB
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         cbBuf    [add content]
         idx    [add content]
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
 *       DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB
 *       DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbMSB(DfeFl_CbHandle hDfeCb, uint32_t cbBuf, uint32_t idx, DfeFl_CbComplexInt *data)
{
	volatile uint32_t *regs;

	switch(cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->capture_buffer_a_16msb[0];
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->capture_buffer_b_16msb[0];
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->capture_buffer_c_16msb[0];
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->capture_buffer_d_16msb[0];
		break;
    default:
    	return;
	}
	CSL_FINS(regs[idx], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB, data->real);
	CSL_FINS(regs[idx], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB, data->imag);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbMSB
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         cbBuf    [add content]
         idx    [add content]
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
 *   @n  see below,
 *       DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB
 *       DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbMSB(DfeFl_CbHandle hDfeCb, uint32_t cbBuf, uint32_t idx, DfeFl_CbComplexInt *data)
{
	volatile uint32_t *regs;

	switch(cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->capture_buffer_a_16msb[0];
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->capture_buffer_b_16msb[0];
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->capture_buffer_c_16msb[0];
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->capture_buffer_d_16msb[0];
		break;
    default:
    	return;
	}
	data->real = CSL_FEXT(regs[idx], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_I_16MSB);
	data->imag = CSL_FEXT(regs[idx], DFE_CB_CAPTURE_BUFFER_A_16MSB_REG_CAPTURE_BUFFER_A_Q_16MSB);
}

/** ============================================================================
 *   @n@b dfeFl_CbSetCbLSB
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         cbBuf    [add content]
         idx    [add content]
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
 *       DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_I_2LSB
 *       DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_Q_2LSB
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CbSetCbLSB(DfeFl_CbHandle hDfeCb, uint32_t cbBuf, uint32_t idx, DfeFl_CbComplexInt *data)
{
	volatile uint32_t *regs;

	switch(cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->capture_buffer_a_2lsb[0];
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->capture_buffer_b_2lsb[0];
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->capture_buffer_c_2lsb[0];
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->capture_buffer_d_2lsb[0];
		break;
    default:
    	return;
	}
	CSL_FINS(regs[idx], DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_I_2LSB, data->real);
	CSL_FINS(regs[idx], DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_Q_2LSB, data->imag);
}

/** ============================================================================
 *   @n@b dfeFl_CbQueryCbLSB
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCb    [add content]
         cbBuf    [add content]
         idx    [add content]
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
 *   @n  see below,
 *       DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_I_2LSB
 *       DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_Q_2LSB
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
CSL_IDEF_INLINE void
dfeFl_CbQueryCbLSB(DfeFl_CbHandle hDfeCb, uint32_t cbBuf, uint32_t idx, DfeFl_CbComplexInt *data)
{
	volatile uint32_t *regs;

	switch(cbBuf)
	{
	case DFE_FL_CBA:
		regs = &hDfeCb->regs->capture_buffer_a_2lsb[0];
		break;
	case DFE_FL_CBB:
		regs = &hDfeCb->regs->capture_buffer_b_2lsb[0];
		break;
	case DFE_FL_CBC:
		regs = &hDfeCb->regs->capture_buffer_c_2lsb[0];
		break;
	case DFE_FL_CBD:
		regs = &hDfeCb->regs->capture_buffer_d_2lsb[0];
		break;
    default:
    	return;
	}
	data->real = CSL_FEXT(regs[idx], DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_I_2LSB);
	data->imag = CSL_FEXT(regs[idx], DFE_CB_CAPTURE_BUFFER_A_2LSB_REG_CAPTURE_BUFFER_A_Q_2LSB);
}
#endif /* _DFE_FL_CBAUX_H_ */
