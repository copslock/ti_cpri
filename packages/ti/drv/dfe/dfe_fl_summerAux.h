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

/** @file dfe_fl_summerAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_SUMMER CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_SUMMERAUX_H_
#define _DFE_FL_SUMMERAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_summer.h>

/** ============================================================================
 *   @n@b dfeFl_SummerConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG4_REG_CLEAR_DATA
 *       DFE_SUMMER_CFG4_REG_INIT_STATE
 *       DFE_SUMMER_CFG4_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_SummerConfigInits(DfeFl_SummerHandle hDfeSummer, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeSummer->regs->cfg4;
    
    CSL_FINS(data, DFE_SUMMER_CFG4_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_SUMMER_CFG4_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_SUMMER_CFG4_REG_CLEAR_DATA, arg->clearData);
    
    hDfeSummer->regs->cfg4 = data;
}

/** ============================================================================
 *   @n@b dfeFl_SummerGetInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG4_REG_CLEAR_DATA
 *       DFE_SUMMER_CFG4_REG_INIT_STATE
 *       DFE_SUMMER_CFG4_REG_INITS_SSEL
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
dfeFl_SummerGetInits(DfeFl_SummerHandle hDfeSummer, DfeFl_SublkInitsConfig * arg)
{
    arg->ssel = CSL_FEXT(hDfeSummer->regs->cfg4, DFE_SUMMER_CFG4_REG_INITS_SSEL);
    arg->initState = CSL_FEXT(hDfeSummer->regs->cfg4, DFE_SUMMER_CFG4_REG_INIT_STATE);
    arg->clearData = CSL_FEXT(hDfeSummer->regs->cfg4, DFE_SUMMER_CFG4_REG_CLEAR_DATA);

}

/** ============================================================================
 *   @n@b dfeFl_SummerSetShift
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR1
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR0
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR0
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR1
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR1
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR0
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR0
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_SummerSetShift(DfeFl_SummerHandle hDfeSummer, DfeFl_SummerShiftCfg * arg)
{
    switch(arg->idx)
    {
    case DFE_FL_SUMMER_CFR0_STR0:
    	CSL_FINS(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR0, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR0_STR1:
    	CSL_FINS(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR1, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR1_STR0:
    	CSL_FINS(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR0, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR1_STR1:
    	CSL_FINS(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR1, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR2_STR0:
    	CSL_FINS(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR0, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR2_STR1:
    	CSL_FINS(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR1, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR3_STR0:
    	CSL_FINS(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR0, arg->data);
    	break;
    case DFE_FL_SUMMER_CFR3_STR1:
    	CSL_FINS(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR1, arg->data);
    	break;
    }
}

/** ============================================================================
 *   @n@b dfeFl_SummerGetShift
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR1
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR0
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR0
 *       DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR1
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR1
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR0
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR0
 *       DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR1
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
dfeFl_SummerGetShift(DfeFl_SummerHandle hDfeSummer, DfeFl_SummerShiftCfg * arg)
{
    switch(arg->idx)
    {
    case DFE_FL_SUMMER_CFR0_STR0:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR0);
    	break;
    case DFE_FL_SUMMER_CFR0_STR1:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR0_STR1);
    	break;
    case DFE_FL_SUMMER_CFR1_STR0:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR0);
    	break;
    case DFE_FL_SUMMER_CFR1_STR1:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg0, DFE_SUMMER_CFG0_REG_SHIFT_CFR1_STR1);
    	break;
    case DFE_FL_SUMMER_CFR2_STR0:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR0);
    	break;
    case DFE_FL_SUMMER_CFR2_STR1:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR2_STR1);
    	break;
    case DFE_FL_SUMMER_CFR3_STR0:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR0);
    	break;
    case DFE_FL_SUMMER_CFR3_STR1:
    	arg->data = CSL_FEXT(hDfeSummer->regs->cfg1, DFE_SUMMER_CFG1_REG_SHIFT_CFR3_STR1);
    	break;
    }
}

/** ============================================================================
 *   @n@b dfeFl_SummerSetSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_SummerSetSel(DfeFl_SummerHandle hDfeSummer, DfeFl_SummerSelCfg * arg)
{
    volatile uint32_t *regs;
    uint32_t r, b, val;

	val = arg->iDduc*3+arg->iPort;
	r = val / 4;
	b = (val % 4) * 4;

	switch(arg->idx)
    {
    case DFE_FL_SUMMER_CFR0_STR0:
    {
    	regs = &hDfeSummer->regs->cfg5;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR0_STR1:
    {
    	regs = &hDfeSummer->regs->cfg8;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR1_STR0:
    {
    	regs = &hDfeSummer->regs->cfg11;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR1_STR1:
    {
    	regs = &hDfeSummer->regs->cfg14;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR2_STR0:
    {
    	regs = &hDfeSummer->regs->cfg17;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR2_STR1:
    {
    	regs = &hDfeSummer->regs->cfg20;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR3_STR0:
    {
    	regs = &hDfeSummer->regs->cfg23;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    case DFE_FL_SUMMER_CFR3_STR1:
    {
    	regs = &hDfeSummer->regs->cfg26;
    	CSL_FINSR(regs[r], b+3, b, arg->data);
    	break;
    }
    }
}

/** ============================================================================
 *   @n@b dfeFl_SummerGetSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_SummerGetSel(DfeFl_SummerHandle hDfeSummer, DfeFl_SummerSelCfg * arg)
{
    volatile uint32_t *regs;
    uint32_t r, b, val;

	val = arg->iDduc*3+arg->iPort;
	r = val / 4;
	b = (val % 4) * 4;

	switch(arg->idx)
    {
    case DFE_FL_SUMMER_CFR0_STR0:
    {
    	regs = &hDfeSummer->regs->cfg5;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR0_STR1:
    {
    	regs = &hDfeSummer->regs->cfg8;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR1_STR0:
    {
    	regs = &hDfeSummer->regs->cfg11;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR1_STR1:
    {
    	regs = &hDfeSummer->regs->cfg14;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR2_STR0:
    {
    	regs = &hDfeSummer->regs->cfg17;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR2_STR1:
    {
    	regs = &hDfeSummer->regs->cfg20;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR3_STR0:
    {
    	regs = &hDfeSummer->regs->cfg23;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    case DFE_FL_SUMMER_CFR3_STR1:
    {
    	regs = &hDfeSummer->regs->cfg26;
    	arg->data = CSL_FEXTR(regs[r], b+3, b);
    	break;
    }
    }
}

/** ============================================================================
 *   @n@b dfeFl_SummerSetNumant
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG2_REG_NUMANT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_SummerSetNumant(DfeFl_SummerHandle hDfeSummer, uint32_t arg)
{
	CSL_FINS(hDfeSummer->regs->cfg2, DFE_SUMMER_CFG2_REG_NUMANT, arg);
}

/** ============================================================================
 *   @n@b dfeFl_SummerGetNumant
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG2_REG_NUMANT
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
dfeFl_SummerGetNumant(DfeFl_SummerHandle hDfeSummer, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeSummer->regs->cfg2, DFE_SUMMER_CFG2_REG_NUMANT);
}

/** ============================================================================
 *   @n@b dfeFl_SummerSetSummerSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG3_REG_SUMMER_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_SummerSetSummerSsel(DfeFl_SummerHandle hDfeSummer, uint32_t arg)
{
	CSL_FINS(hDfeSummer->regs->cfg3, DFE_SUMMER_CFG3_REG_SUMMER_SSEL, arg);
}

/** ============================================================================
 *   @n@b dfeFl_SummerGetSummerSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeSummer    [add content]
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
 *       DFE_SUMMER_CFG3_REG_SUMMER_SSEL
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
dfeFl_SummerGetSummerSsel(DfeFl_SummerHandle hDfeSummer, uint32_t *arg)
{
	*arg = CSL_FEXT(hDfeSummer->regs->cfg3, DFE_SUMMER_CFG3_REG_SUMMER_SSEL);
}

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_SUMMERAUX_H_ */
