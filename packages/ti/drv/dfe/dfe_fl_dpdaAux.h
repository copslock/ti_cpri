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

/** @file dfe_fl_dpdaAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DPDA CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_DPDAAUX_H_
#define _DFE_FL_DPDAAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_dpda.h>

/** ============================================================================
 *   @n@b dfeFl_DpdaConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INITS_REG_INIT_CLK_GATE
 *       DFE_DPDA_INITS_REG_CLEAR_DATA
 *       DFE_DPDA_INITS_REG_INIT_STATE
 *       DFE_DPDA_INITS_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_DpdaConfigInits(DfeFl_DpdaHandle hDpda, DfeFl_SublkInitsConfig * arg)
{
    
    uint32_t data = hDpda->regs->inits;
    
    CSL_FINS(data, DFE_DPDA_INITS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_DPDA_INITS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_DPDA_INITS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_DPDA_INITS_REG_CLEAR_DATA, arg->clearData);
    
    hDpda->regs->inits = data;
    
}

/** ============================================================================
 *   @n@b dfeFl_DpdaEnableIntreadcompleteIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MASK_REG_INT_READ_COMPLETE_INTR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaEnableIntreadcompleteIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->mask, DFE_DPDA_MASK_REG_INT_READ_COMPLETE_INTR, 1);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaDisableIntreadcompleteIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MASK_REG_INT_READ_COMPLETE_INTR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaDisableIntreadcompleteIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->mask, DFE_DPDA_MASK_REG_INT_READ_COMPLETE_INTR, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetForceIntreadcompleteIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_FORCE_REG_INT_READ_COMPLETE_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetForceIntreadcompleteIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->force, DFE_DPDA_FORCE_REG_INT_READ_COMPLETE_FORCE, 1);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaClearForceIntreadcompleteIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_FORCE_REG_INT_READ_COMPLETE_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaClearForceIntreadcompleteIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->force, DFE_DPDA_FORCE_REG_INT_READ_COMPLETE_FORCE, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaClearIntreadcompleteIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_INT_READ_COMPLETE_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaClearIntreadcompleteIntrStatus(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->status, DFE_DPDA_STATUS_REG_INT_READ_COMPLETE_STATUS, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetIntreadcompleteIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_INT_READ_COMPLETE_STATUS
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
dfeFl_DpdaGetIntreadcompleteIntrStatus(DfeFl_DpdaHandle hDpda, uint32_t *arg)
{
  *arg = CSL_FEXT(hDpda->regs->status, DFE_DPDA_STATUS_REG_INT_READ_COMPLETE_STATUS);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaEnableIntprocessedIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MASK_REG_INT_PROCESSED_INTR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaEnableIntprocessedIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->mask, DFE_DPDA_MASK_REG_INT_PROCESSED_INTR, 1);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaDisableIntprocessedIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MASK_REG_INT_PROCESSED_INTR
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaDisableIntprocessedIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->mask, DFE_DPDA_MASK_REG_INT_PROCESSED_INTR, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetForceIntprocessedIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_FORCE_REG_INT_PROCESSED_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetForceIntprocessedIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->force, DFE_DPDA_FORCE_REG_INT_PROCESSED_FORCE, 1);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaClearForceIntprocessedIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_FORCE_REG_INT_PROCESSED_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaClearForceIntprocessedIntr(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->force, DFE_DPDA_FORCE_REG_INT_PROCESSED_FORCE, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaClearIntprocessedIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_INT_PROCESSED_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaClearIntprocessedIntrStatus(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->status, DFE_DPDA_STATUS_REG_INT_PROCESSED_STATUS, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetIntprocessedIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_INT_PROCESSED_STATUS
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
dfeFl_DpdaGetIntprocessedIntrStatus(DfeFl_DpdaHandle hDpda, uint32_t *arg)
{
  *arg = CSL_FEXT(hDpda->regs->status, DFE_DPDA_STATUS_REG_INT_PROCESSED_STATUS);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaClearIdleIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_IDLE_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaClearIdleIntrStatus(DfeFl_DpdaHandle hDpda)
{
  CSL_FINS(hDpda->regs->status, DFE_DPDA_STATUS_REG_IDLE_STATUS, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetIdleIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_STATUS_REG_IDLE_STATUS
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
dfeFl_DpdaGetIdleIntrStatus(DfeFl_DpdaHandle hDpda, uint32_t *arg)
{
  *arg = CSL_FEXT(hDpda->regs->status, DFE_DPDA_STATUS_REG_IDLE_STATUS);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetDspIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MAIN_CONTROL_REG_DSP_INTERRRUPT_MASTER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetDspIntr(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->main_control, DFE_DPDA_MAIN_CONTROL_REG_DSP_INTERRRUPT_MASTER, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetDspAntEn
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_MAIN_CONTROL_REG_DSP_ANTENNA_ENABLED_MASTER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetDspAntEn(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->main_control, DFE_DPDA_MAIN_CONTROL_REG_DSP_ANTENNA_ENABLED_MASTER, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetJacobp2lscale
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_JACOB_STATIC_REG_LUTFILL_FP2I
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetJacobp2lscale(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->jacob_static, DFE_DPDA_JACOB_STATIC_REG_LUTFILL_FP2I, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetJacobInputscale
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_JACOB_STATIC_REG_INPUT_SCALE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetJacobInputscale(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->jacob_static, DFE_DPDA_JACOB_STATIC_REG_INPUT_SCALE, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetExpFp2i
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_EXPONENTS_REG_EXP_FP2I
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetExpFp2i(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->exponents, DFE_DPDA_EXPONENTS_REG_EXP_FP2I, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetExpI2fp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_EXPONENTS_REG_EXP_I2FP
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetExpI2fp(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->exponents, DFE_DPDA_EXPONENTS_REG_EXP_I2FP, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetIntrAddressDpd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_INTERRUPT_ADDRESS_DPD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetIntrAddressDpd(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->interrupt_main_and_req, DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_INTERRUPT_ADDRESS_DPD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetAntEnabledDpd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_ANTENNA_ENABLED_DPD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetAntEnabledDpd(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->interrupt_main_and_req, DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_ANTENNA_ENABLED_DPD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetNewIntDpd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_NEW_INT_DPD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetNewIntDpd(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->interrupt_main_and_req, DFE_DPDA_INTERRUPT_MAIN_AND_REQ_REG_NEW_INT_DPD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetParam1Dpd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INTERRUPT_PARAMS_REG_PARAM1_DPD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetParam1Dpd(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->interrupt_params, DFE_DPDA_INTERRUPT_PARAMS_REG_PARAM1_DPD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetParam2Dpd
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
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
 *       DFE_DPDA_INTERRUPT_PARAMS_REG_PARAM2_DPD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetParam2Dpd(DfeFl_DpdaHandle hDpda, uint32_t arg)
{
  CSL_FINS(hDpda->regs->interrupt_params, DFE_DPDA_INTERRUPT_PARAMS_REG_PARAM2_DPD, arg);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetIgreg
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_IG_REGFILE_REG_DPDA_IG_REGFILE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetIgreg(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_ig_regfile[0];
	CSL_FINS(regs[iEntry], DFE_DPDA_DPDA_IG_REGFILE_REG_DPDA_IG_REGFILE, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetScalar
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
         iedata    [add content]
         qdata    [add content]
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
 *       DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER
 *       DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetScalar(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t iedata, uint32_t qdata)
{
	volatile CSL_DFE_DPDA_DPDA_SCALAR_REGS *regs;

	regs = &hDpda->regs->dpda_scalar[0];
	CSL_FINS(regs[iEntry].ie_register, DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER, iedata);
	CSL_FINS(regs[iEntry].q_register, DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER, qdata);
}



/** ============================================================================
 *   @n@b dfeFl_DpdaSetLutMaster
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_LUT_MASTER_REG_DPDA_LUT_MASTER
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetLutMaster(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_lut_master[0];
	CSL_FINS(regs[iEntry], DFE_DPDA_DPDA_LUT_MASTER_REG_DPDA_LUT_MASTER, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetLutMaster
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_LUT_MASTER_REG_DPDA_LUT_MASTER
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
dfeFl_DpdaGetLutMaster(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t *data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_lut_master[0];
	*data = CSL_FEXT(regs[iEntry], DFE_DPDA_DPDA_LUT_MASTER_REG_DPDA_LUT_MASTER);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetLut
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         idx    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_LUT_0_REG_DPDA_LUT_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetLut(DfeFl_DpdaHandle hDpda, uint32_t idx, uint32_t iEntry, uint32_t data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_lut_0[0];
	CSL_FINS(regs[idx*1024+iEntry], DFE_DPDA_DPDA_LUT_0_REG_DPDA_LUT_0, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetLut
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         idx    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_LUT_0_REG_DPDA_LUT_0
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
dfeFl_DpdaGetLut(DfeFl_DpdaHandle hDpda, uint32_t idx, uint32_t iEntry, uint32_t *data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_lut_0[0];
	*data = CSL_FEXT(regs[idx*1024+iEntry], DFE_DPDA_DPDA_LUT_0_REG_DPDA_LUT_0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetIram
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_IRAM_REG_DPDA_IRAM
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetIram(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_iram[0];
	CSL_FINS(regs[iEntry], DFE_DPDA_DPDA_IRAM_REG_DPDA_IRAM, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetIram
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
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
 *       DFE_DPDA_DPDA_IRAM_REG_DPDA_IRAM
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
dfeFl_DpdaGetIram(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t *data)
{
	volatile uint32_t *regs;

	regs = &hDpda->regs->dpda_iram[0];
	*data = CSL_FEXT(regs[iEntry], DFE_DPDA_DPDA_IRAM_REG_DPDA_IRAM);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaSetPreg
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         idx    [add content]
         iEntry    [add content]
         iedata    [add content]
         qdata    [add content]
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
 *       DFE_DPDA_DPDA_PREG_000_Q_REG_DPDA_PREG_000_Q
 *       DFE_DPDA_DPDA_PREG_000_IE_REG_DPDA_PREG_000_IE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdaSetPreg(DfeFl_DpdaHandle hDpda, uint32_t idx, uint32_t iEntry, uint32_t iedata, uint32_t qdata)
{
	volatile CSL_DFE_DPDA_DPDA_PREG_000_REGS *regs;

	regs = &hDpda->regs->dpda_preg_000[0] + idx*32;
	CSL_FINS(regs[iEntry].ie, DFE_DPDA_DPDA_PREG_000_IE_REG_DPDA_PREG_000_IE, iedata);
	CSL_FINS(regs[iEntry].q, DFE_DPDA_DPDA_PREG_000_Q_REG_DPDA_PREG_000_Q, qdata);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetPreg
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         idx    [add content]
         iEntry    [add content]
         iedata    [add content]
         qdata    [add content]
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
 *       DFE_DPDA_DPDA_PREG_000_Q_REG_DPDA_PREG_000_Q
 *       DFE_DPDA_DPDA_PREG_000_IE_REG_DPDA_PREG_000_IE
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
dfeFl_DpdaGetPreg(DfeFl_DpdaHandle hDpda, uint32_t idx, uint32_t iEntry, uint32_t *iedata, uint32_t *qdata)
{
	volatile CSL_DFE_DPDA_DPDA_PREG_000_REGS *regs;

	regs = &hDpda->regs->dpda_preg_000[0] + idx*32;
	*iedata = CSL_FEXT(regs[iEntry].ie, DFE_DPDA_DPDA_PREG_000_IE_REG_DPDA_PREG_000_IE);
	*qdata = CSL_FEXT(regs[iEntry].q, DFE_DPDA_DPDA_PREG_000_Q_REG_DPDA_PREG_000_Q);
}

/** ============================================================================
 *   @n@b dfeFl_DpdaGetScalar
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpda    [add content]
         iEntry    [add content]
         iedata    [add content]
         qdata    [add content]
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
 *       DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER
 *       DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER
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
dfeFl_DpdaGetScalar(DfeFl_DpdaHandle hDpda, uint32_t iEntry, uint32_t *iedata, uint32_t *qdata)
{
    volatile CSL_DFE_DPDA_DPDA_SCALAR_REGS *regs;

    regs = &hDpda->regs->dpda_scalar[0];
    *iedata = CSL_FEXT(regs[iEntry].ie_register, DFE_DPDA_DPDA_SCALAR_IE_REGISTER_REG_DPDA_SCALAR_IE_REGISTER);
    *qdata = CSL_FEXT(regs[iEntry].q_register, DFE_DPDA_DPDA_SCALAR_Q_REGISTER_REG_DPDA_SCALAR_Q_REGISTER);
}


#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_DPDAAUX_H_ */
