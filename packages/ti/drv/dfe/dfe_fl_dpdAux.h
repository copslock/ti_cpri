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

/** @file dfe_fl_dpdAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DPD CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_DPDAUX_H_
#define _DFE_FL_DPDAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_dpd.h>

/** ============================================================================
 *   @n@b dfeFl_DpdConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDpd    [add content]
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
 *       DFE_DPD_TOP_INITS_REG_INIT_STATE
 *       DFE_DPD_TOP_INITS_REG_INITS_SSEL
 *       DFE_DPD_TOP_INITS_REG_CLEAR_DATA
 *       DFE_DPD_TOP_INITS_REG_INIT_CLK_GATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_DpdConfigInits(DfeFl_DpdHandle hDpd, DfeFl_SublkInitsConfig * arg)
{
    
    uint32_t data = hDpd->regs->top_inits;
    
    CSL_FINS(data, DFE_DPD_TOP_INITS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_DPD_TOP_INITS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_DPD_TOP_INITS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_DPD_TOP_INITS_REG_CLEAR_DATA, arg->clearData);
    
    hDpd->regs->top_inits = data;    
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateSubchipMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateSubchipMode(DfeFl_DpdHandle hDfeDpd, uint32_t data)
{
	CSL_FINS(hDfeDpd->regs->top_subchip_mode_subsample, \
	DFE_DPD_TOP_SUBCHIP_MODE_SUBSAMPLE_REG_SUBCHIP_MODE, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQuerySubchipMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQuerySubchipMode(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_subchip_mode_subsample, \
			DFE_DPD_TOP_SUBCHIP_MODE_SUBSAMPLE_REG_SUBCHIP_MODE);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateSubsample
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateSubsample(DfeFl_DpdHandle hDfeDpd, uint32_t data)
{
	CSL_FINS(hDfeDpd->regs->top_subchip_mode_subsample, \
	DFE_DPD_TOP_SUBCHIP_MODE_SUBSAMPLE_REG_SUBSAMPLE, data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQuerySubsample
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQuerySubsample(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_subchip_mode_subsample, \
			DFE_DPD_TOP_SUBCHIP_MODE_SUBSAMPLE_REG_SUBSAMPLE);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxSqrt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R01_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R32_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R2_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R00_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX3_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX2_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R31_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R33_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R1_SQRT
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxSqrt(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSqrt *arg)
{
	switch(arg->Mux)
	{
	case DFE_FL_DPD_MUX_SQRT_R00:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R00_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R01:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R01_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R1:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R1_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R2:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R2_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R31:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R31_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R32:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R32_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_R33:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R33_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_MAGX2:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX2_SQRT, arg->data);
		break;
	case DFE_FL_DPD_MUX_SQRT_MAGX3:
		CSL_FINS(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX3_SQRT, arg->data);
		break;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxSqrt
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R01_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R32_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R2_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R00_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX3_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX2_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R31_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R33_SQRT
 *       DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R1_SQRT
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
dfeFl_DpdQueryMuxSqrt(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSqrt *arg)
{
	switch(arg->Mux)
	{
	case DFE_FL_DPD_MUX_SQRT_R00:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R00_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R01:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R01_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R1:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R1_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R2:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R2_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R31:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R31_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R32:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R32_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_R33:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_R33_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_MAGX2:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX2_SQRT);
		break;
	case DFE_FL_DPD_MUX_SQRT_MAGX3:
		arg->data = CSL_FEXT(hDfeDpd->regs->input_mux_sqrt, DFE_DPD_INPUT_MUX_SQRT_REG_MUX_MAGX3_SQRT);
		break;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxComplx
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
dfeFl_DpdUpdateMuxComplx(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSignal *arg)
{
	uint32_t b = arg->idxBlk*4+arg->evenOrOdd*2;

	CSL_FINSR(hDfeDpd->regs->input_mux_complx_signal, b+1, b, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxComplx
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
dfeFl_DpdQueryMuxComplx(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSignal *arg)
{
	uint32_t b = arg->idxBlk*4+arg->evenOrOdd*2;

	arg->data = CSL_FEXTR(hDfeDpd->regs->input_mux_complx_signal, b+1, b);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxRealMag
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
dfeFl_DpdUpdateMuxRealMag(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSignal *arg)
{
	uint32_t b = arg->idxBlk*4+arg->evenOrOdd*2;

	CSL_FINSR(hDfeDpd->regs->input_mux_real_magnitude, b+1, b, arg->data);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxRealMag
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
dfeFl_DpdQueryMuxRealMag(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxSignal *arg)
{
	uint32_t b = arg->idxBlk*4+arg->evenOrOdd*2;

	arg->data = CSL_FEXTR(hDfeDpd->regs->input_mux_real_magnitude, b+1, b);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxOutput
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT0
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT1
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT2
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxOutput(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT0, arg->data);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT1, arg->data);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT2, arg->data);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT3, arg->data);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxOutput
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT0
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT1
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT2
 *       DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT3
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
dfeFl_DpdQueryMuxOutput(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		arg->data = CSL_FEXT(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT0);
		break;
	case DFE_FL_DPD_B1:
		arg->data = CSL_FEXT(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT1);
		break;
	case DFE_FL_DPD_B2:
		arg->data = CSL_FEXT(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT2);
		break;
	case DFE_FL_DPD_B3:
		arg->data = CSL_FEXT(hDfeDpd->regs->output_mux_dpd_output, DFE_DPD_OUTPUT_MUX_DPD_OUTPUT_REG_MUX_DPDOUT3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateDpdadaptMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B0
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateDpdadaptMode(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B0, arg->data);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B1, arg->data);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B2, arg->data);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B3, arg->data);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryDpdadaptMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B0
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B1
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
dfeFl_DpdQueryDpdadaptMode(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B0);
		break;
	case DFE_FL_DPD_B1:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B1);
		break;
	case DFE_FL_DPD_B2:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B2);
		break;
	case DFE_FL_DPD_B3:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_DPDADAPT_UPDATE_MODE_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateDpdadaptFsync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B1
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateDpdadaptFsync(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B0, arg->data);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B1, arg->data);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B2, arg->data);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B3, arg->data);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryDpdadaptFsync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B1
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B0
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
dfeFl_DpdQueryDpdadaptFsync(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B0);
		break;
	case DFE_FL_DPD_B1:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B1);
		break;
	case DFE_FL_DPD_B2:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B2);
		break;
	case DFE_FL_DPD_B3:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_FSYNC_DPDADAPT_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateDpdadaptCsync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B0
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateDpdadaptCsync(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B0, arg->data);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B1, arg->data);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B2, arg->data);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B3, arg->data);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryDpdadaptCsync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B2
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B3
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B0
 *       DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B1
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
dfeFl_DpdQueryDpdadaptCsync(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B0);
		break;
	case DFE_FL_DPD_B1:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B1);
		break;
	case DFE_FL_DPD_B2:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B2);
		break;
	case DFE_FL_DPD_B3:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpdadapt_update_mode, DFE_DPD_TOP_DPDADAPT_UPDATE_MODE_REG_MUX_CSYNC_DPDADAPT_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlkfSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxBlk    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B2
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B3
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B0
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlkfSsel(DfeFl_DpdHandle hDfeDpd, uint32_t idxBlk, uint32_t Ssel)
{
	switch(idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B0, Ssel);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B1, Ssel);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B2, Ssel);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B3, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlkfSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxBlk    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B2
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B3
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B0
 *       DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B1
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
dfeFl_DpdQueryBlkfSsel(DfeFl_DpdHandle hDfeDpd, uint32_t idxBlk, uint32_t *Ssel)
{
	switch(idxBlk)
	{
	case DFE_FL_DPD_B0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B0);
		break;
	case DFE_FL_DPD_B1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B1);
		break;
	case DFE_FL_DPD_B2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B2);
		break;
	case DFE_FL_DPD_B3:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_f_ssel, DFE_DPD_TOP_F_SSEL_REG_C_F_SSEL_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlkcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxBlk    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B0
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B1
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B2
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlkcSsel(DfeFl_DpdHandle hDfeDpd, uint32_t idxBlk, uint32_t Ssel)
{
	switch(idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B0, Ssel);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B1, Ssel);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B2, Ssel);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B3, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlkcSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxBlk    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B0
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B1
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B2
 *       DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B3
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
dfeFl_DpdQueryBlkcSsel(DfeFl_DpdHandle hDfeDpd, uint32_t idxBlk, uint32_t *Ssel)
{
	switch(idxBlk)
	{
	case DFE_FL_DPD_B0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B0);
		break;
	case DFE_FL_DPD_B1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B1);
		break;
	case DFE_FL_DPD_B2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B2);
		break;
	case DFE_FL_DPD_B3:
		*Ssel = CSL_FEXT(hDfeDpd->regs->top_c_ssel, DFE_DPD_TOP_C_SSEL_REG_C_C_SSEL_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdSetDpdDisable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B0
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B1
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B2
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdSetDpdDisable(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		CSL_FINS(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B0, arg->data);
		break;
	case DFE_FL_DPD_B1:
		CSL_FINS(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B1, arg->data);
		break;
	case DFE_FL_DPD_B2:
		CSL_FINS(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B2, arg->data);
		break;
	case DFE_FL_DPD_B3:
		CSL_FINS(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B3, arg->data);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryDpdDisable
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B0
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B1
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B2
 *       DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B3
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
dfeFl_DpdQueryDpdDisable(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdBlkCtrl *arg)
{
	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B0);
		break;
	case DFE_FL_DPD_B1:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B1);
		break;
	case DFE_FL_DPD_B2:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B2);
		break;
	case DFE_FL_DPD_B3:
		arg->data = CSL_FEXT(hDfeDpd->regs->top_dpd_disable, DFE_DPD_TOP_DPD_DISABLE_REG_DPD_DISABLE_B3);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateSyncBSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         SyncBSsel    [add content]
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
dfeFl_DpdUpdateSyncBSsel(DfeFl_DpdHandle hDfeDpd, uint32_t SyncBSsel)
{
	CSL_FINS(hDfeDpd->regs->top_sync_b_ssel, \
			DFE_DPD_TOP_SYNC_B_SSEL_REG_SYNC_B_SSEL, SyncBSsel);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQuerySyncBSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQuerySyncBSsel(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_sync_b_ssel, \
			DFE_DPD_TOP_SYNC_B_SSEL_REG_SYNC_B_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryInitsSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryInitsSsel(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_inits, \
			DFE_DPD_TOP_INITS_REG_INITS_SSEL);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryInitClkGate
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryInitClkGate(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_inits, \
			DFE_DPD_TOP_INITS_REG_INIT_CLK_GATE);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryInitState
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryInitState(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_inits, \
			DFE_DPD_TOP_INITS_REG_INIT_STATE);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryClearData
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryClearData(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_inits, \
			DFE_DPD_TOP_INITS_REG_CLEAR_DATA);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateDpdInputScale
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         Scale    [add content]
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
dfeFl_DpdUpdateDpdInputScale(DfeFl_DpdHandle hDfeDpd, uint32_t Scale)
{
	CSL_FINS(hDfeDpd->regs->input_dpdinput_scale, \
			DFE_DPD_INPUT_DPDINPUT_SCALE_REG_DPDINPUT_SCALE, Scale);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryDpdInputScale
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryDpdInputScale(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->input_dpdinput_scale, \
			DFE_DPD_INPUT_DPDINPUT_SCALE_REG_DPDINPUT_SCALE);
}

/** ============================================================================
 *   @n@b dfeFl_DpdConfigTestGen
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_SLOPE_HI_REG_SIGNAL_GEN0_RAMP_SLOPE_31_16
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_DATA
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_STOP_LO_REG_SIGNAL_GEN0_RAMP_STOP_15_0
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_START_HI_REG_SIGNAL_GEN0_RAMP_START_31_16
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_FRAME_LEN_M1
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_FRAME
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_SLOPE_LO_REG_SIGNAL_GEN0_RAMP_SLOPE_15_0
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_STOP_HI_REG_SIGNAL_GEN0_RAMP_STOP_31_16
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_SEED
 *       DFE_DPD_TOP_SIGNAL_GEN0_GEN_TIMER_REG_SIGNAL_GEN0_GEN_TIMER
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_RAMP_MODE
 *       DFE_DPD_TOP_SIGNAL_GEN0_RAMP_START_LO_REG_SIGNAL_GEN0_RAMP_START_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdConfigTestGen(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdTestGenConfig * arg)
{
    volatile uint32_t *regs;

    switch(arg->tgDev)
    {
    // TESTGEN0 for DPD
    case DFE_FL_DPD_TESTGEN_0:
        regs = &hDfeDpd->regs->top_signal_gen0_general;
        break;
    // TESTGEN1 for DPD
    case DFE_FL_DPD_TESTGEN_1:
        regs = &hDfeDpd->regs->top_signal_gen1_general;
        break;
    // TESTGEN2 for DPD
    case DFE_FL_DPD_TESTGEN_2:
        regs = &hDfeDpd->regs->top_signal_gen2_general;
        break;
    // TESTGEN3 for DPD
    case DFE_FL_DPD_TESTGEN_3:
        regs = &hDfeDpd->regs->top_signal_gen3_general;
        break;
    // TESTGEN4 for DPD
    case DFE_FL_DPD_TESTGEN_4:
        regs = &hDfeDpd->regs->top_signal_gen4_general;
        break;
    // TESTGEN5 for DPD
    case DFE_FL_DPD_TESTGEN_5:
        regs = &hDfeDpd->regs->top_signal_gen5_general;
        break;
    // TESTGEN6 for DPD
    case DFE_FL_DPD_TESTGEN_6:
        regs = &hDfeDpd->regs->top_signal_gen6_general;
        break;
    // TESTGEN7 for DPD
    case DFE_FL_DPD_TESTGEN_7:
        regs = &hDfeDpd->regs->top_signal_gen7_general;
        break;

    default:
        return;
    }

    // general
    regs[0] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_DATA, arg->genData)
         | CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_FRAME, arg->genFrame)
         | CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_RAMP_MODE, arg->rampMode)
         | CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_SEED, arg->seed)
         | CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_FRAME_LEN_M1, arg->frameLenM1);
    // ramp start
    regs[1] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_START_LO_REG_SIGNAL_GEN0_RAMP_START_15_0, arg->rampStart);
    regs[2] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_START_HI_REG_SIGNAL_GEN0_RAMP_START_31_16, arg->rampStart >> 16);
    // ramp stop
    regs[3] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_STOP_LO_REG_SIGNAL_GEN0_RAMP_STOP_15_0, arg->rampStop);
    regs[4] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_STOP_HI_REG_SIGNAL_GEN0_RAMP_STOP_31_16, arg->rampStop >> 16);
    // ramp slope
    regs[5] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_SLOPE_LO_REG_SIGNAL_GEN0_RAMP_SLOPE_15_0, arg->slope);
    regs[6] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_RAMP_SLOPE_HI_REG_SIGNAL_GEN0_RAMP_SLOPE_31_16, arg->slope >> 16);
    // gen timer
    regs[7] = CSL_FMK(DFE_DPD_TOP_SIGNAL_GEN0_GEN_TIMER_REG_SIGNAL_GEN0_GEN_TIMER, arg->genTimer);

}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryTestGenConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_SEED
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_DATA
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_FRAME_LEN_M1
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_FRAME
 *       DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_RAMP_MODE
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
dfeFl_DpdQueryTestGenConfig(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdTestGenConfig * arg)
{
    uint32_t data;
    volatile uint32_t *regs;

    switch(arg->tgDev)
    {
    // TESTGEN0 for DPD
    case DFE_FL_DPD_TESTGEN_0:
        regs = &hDfeDpd->regs->top_signal_gen0_general;
        break;
    // TESTGEN1 for DPD
    case DFE_FL_DPD_TESTGEN_1:
        regs = &hDfeDpd->regs->top_signal_gen1_general;
        break;
    // TESTGEN2 for DPD
    case DFE_FL_DPD_TESTGEN_2:
        regs = &hDfeDpd->regs->top_signal_gen2_general;
        break;
    // TESTGEN3 for DPD
    case DFE_FL_DPD_TESTGEN_3:
        regs = &hDfeDpd->regs->top_signal_gen3_general;
        break;
    // TESTGEN4 for DPD
    case DFE_FL_DPD_TESTGEN_4:
        regs = &hDfeDpd->regs->top_signal_gen4_general;
        break;
    // TESTGEN5 for DPD
    case DFE_FL_DPD_TESTGEN_5:
        regs = &hDfeDpd->regs->top_signal_gen5_general;
        break;
    // TESTGEN6 for DPD
    case DFE_FL_DPD_TESTGEN_6:
        regs = &hDfeDpd->regs->top_signal_gen6_general;
        break;
    // TESTGEN7 for DPD
    case DFE_FL_DPD_TESTGEN_7:
        regs = &hDfeDpd->regs->top_signal_gen7_general;
        break;

    default:
        return;
    }

    // general
    data = regs[0];
    arg->genData = CSL_FEXT(data, DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_DATA);
    arg->genFrame = CSL_FEXT(data, DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_GEN_FRAME);
    arg->rampMode = (DfeFl_DpdTestGenRampMode)CSL_FEXT(data, DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_RAMP_MODE);
    arg->seed = CSL_FEXT(data, DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_SEED);
    arg->frameLenM1 = CSL_FEXT(data, DFE_DPD_TOP_SIGNAL_GEN0_GENERAL_REG_SIGNAL_GEN0_FRAME_LEN_M1);
    // ramp start
    arg->rampStart = regs[1] | (regs[2] << 16);
    // ramp stop
    arg->rampStop = regs[3] | (regs[4] << 16);
    // ramp slope
    arg->slope = regs[5] | (regs[6] << 16);
    // gen timer
    arg->genTimer = regs[7];
    // number of data bits inverted (read-only)
    arg->numDataBits = regs[8];
}

/** ============================================================================
 *   @n@b dfeFl_DpdSetTestGenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         tgDev    [add content]
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
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN6_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN2_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN3_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN5_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN1_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN7_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN4_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN0_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdSetTestGenSsel(DfeFl_DpdHandle hDfeDpd, uint32_t tgDev, uint32_t ssel)
{
    switch(tgDev)
    {
    // TESTGEN0 for DPD
    case DFE_FL_DPD_TESTGEN_0:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN0_SSEL, ssel);
        break;
    // TESTGEN1 for DPD
    case DFE_FL_DPD_TESTGEN_1:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN1_SSEL, ssel);
        break;
    // TESTGEN2 for DPD
    case DFE_FL_DPD_TESTGEN_2:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN2_SSEL, ssel);
        break;
    // TESTGEN3 for DPD
    case DFE_FL_DPD_TESTGEN_3:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN3_SSEL, ssel);
        break;
    // TESTGEN4 for DPD
    case DFE_FL_DPD_TESTGEN_4:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN4_SSEL, ssel);
        break;
    // TESTGEN5 for DPD
    case DFE_FL_DPD_TESTGEN_5:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN5_SSEL, ssel);
        break;
    // TESTGEN6 for DPD
    case DFE_FL_DPD_TESTGEN_6:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN6_SSEL, ssel);
        break;
    // TESTGEN7 for DPD
    case DFE_FL_DPD_TESTGEN_7:
        CSL_FINS(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN7_SSEL, ssel);
        break;

    default:
        return;
    }

}

/** ============================================================================
 *   @n@b dfeFl_DpdGetTestGenSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         tgDev    [add content]
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
 *   @n  see below,
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN6_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN2_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN3_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN5_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN1_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN7_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN4_SSEL
 *       DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN0_SSEL
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
dfeFl_DpdGetTestGenSsel(DfeFl_DpdHandle hDfeDpd, uint32_t tgDev, uint32_t *ssel)
{
    switch(tgDev)
    {
    // TESTGEN0 for DPD
    case DFE_FL_DPD_TESTGEN_0:
        *ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN0_SSEL);
        break;
    // TESTGEN1 for DPD
    case DFE_FL_DPD_TESTGEN_1:
        *ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN1_SSEL);
        break;
    // TESTGEN2 for DPD
    case DFE_FL_DPD_TESTGEN_2:
        *ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN2_SSEL);
        break;
    // TESTGEN3 for DPD
    case DFE_FL_DPD_TESTGEN_3:
        *ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part0, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART0_REG_SIGNAL_GEN3_SSEL);
        break;
    // TESTGEN4 for DPD
    case DFE_FL_DPD_TESTGEN_4:
        *ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN4_SSEL);
        break;
    // TESTGEN5 for DPD
    case DFE_FL_DPD_TESTGEN_5:
    	*ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN5_SSEL);
        break;
    // TESTGEN6 for DPD
    case DFE_FL_DPD_TESTGEN_6:
    	*ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN6_SSEL);
        break;
    // TESTGEN7 for DPD
    case DFE_FL_DPD_TESTGEN_7:
    	*ssel = CSL_FEXT(hDfeDpd->regs->top_signal_gen_ssel_part1, DFE_DPD_TOP_SIGNAL_GEN_SSEL_PART1_REG_SIGNAL_GEN7_SSEL);
        break;

    default:
        return;
    }

}

/** ============================================================================
 *   @n@b dfeFl_DpdConfigChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_TOP_CHECK_SUM_CTRL_REG_CHECK_SUM_STABLE_LEN
 *       DFE_DPD_TOP_CHECK_SUM_CHAN_SEL_REG_CHECK_SUM_CHAN_SEL
 *       DFE_DPD_TOP_CHECK_SUM_CTRL_REG_CHECK_SUM_MODE
 *       DFE_DPD_TOP_CHECK_SUM_SIGNAL_LEN_REG_CHECK_SUM_SIGNAL_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdConfigChksum(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdChksumConfig *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDpd->regs->top_check_sum_ctrl;

    // ctrl, stable_len
    regs[0] = CSL_FMK(DFE_DPD_TOP_CHECK_SUM_CTRL_REG_CHECK_SUM_MODE, arg->chksumMode)
            | CSL_FMK(DFE_DPD_TOP_CHECK_SUM_CTRL_REG_CHECK_SUM_STABLE_LEN, arg->latencyMode.stableLen);

    // signal_len
    regs[1] = CSL_FMK(DFE_DPD_TOP_CHECK_SUM_SIGNAL_LEN_REG_CHECK_SUM_SIGNAL_LEN, arg->latencyMode.signalLen);

    // chan_sel
    regs[2] = CSL_FMK(DFE_DPD_TOP_CHECK_SUM_CHAN_SEL_REG_CHECK_SUM_CHAN_SEL, arg->latencyMode.chanSel);
}

/** ============================================================================
 *   @n@b dfeFl_DpdSetChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_TOP_CHECK_SUM_SSEL_REG_CHECK_SUM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdSetChksumSsel(DfeFl_DpdHandle hDfeDpd, uint32_t Ssel)
{
    CSL_FINS(hDfeDpd->regs->top_check_sum_ssel, DFE_DPD_TOP_CHECK_SUM_SSEL_REG_CHECK_SUM_SSEL, Ssel);
}

/** ============================================================================
 *   @n@b dfeFl_DpdGetChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         result    [add content]
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
 *       DFE_DPD_TOP_CHECK_SUM_RESULT_HI_REG_CHECK_SUM_RESULT_31_16
 *       DFE_DPD_TOP_CHECK_SUM_RESULT_LO_REG_CHECK_SUM_RESULT_15_0
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
dfeFl_DpdGetChksumResult(DfeFl_DpdHandle hDfeDpd, uint32_t *result)
{
    uint32_t data;

	data = CSL_FEXT(hDfeDpd->regs->top_check_sum_result_lo, DFE_DPD_TOP_CHECK_SUM_RESULT_LO_REG_CHECK_SUM_RESULT_15_0);
    data |= CSL_FEXT(hDfeDpd->regs->top_check_sum_result_hi, DFE_DPD_TOP_CHECK_SUM_RESULT_HI_REG_CHECK_SUM_RESULT_31_16) << 16;

    *result = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdSetClkgateDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         ClkgateDly    [add content]
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
dfeFl_DpdSetClkgateDelay(DfeFl_DpdHandle hDfeDpd, uint32_t ClkgateDly)
{
	CSL_FINS(hDfeDpd->regs->top_gc_clk_gate_delay, \
			DFE_DPD_TOP_GC_CLK_GATE_DELAY_REG_GC_CLK_GATE_DELAY, ClkgateDly);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryClkgateDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryClkgateDelay(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_gc_clk_gate_delay, \
			DFE_DPD_TOP_GC_CLK_GATE_DELAY_REG_GC_CLK_GATE_DELAY);
}

/** ============================================================================
 *   @n@b dfeFl_DpdSetTestbusCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         TestbusCtrl    [add content]
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
dfeFl_DpdSetTestbusCtrl(DfeFl_DpdHandle hDfeDpd, uint32_t TestbusCtrl)
{
	CSL_FINS(hDfeDpd->regs->top_testbus_control, \
			DFE_DPD_TOP_TESTBUS_CONTROL_REG_TESTBUS_CONTROL, TestbusCtrl);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryTestbusCtrl
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdQueryTestbusCtrl(DfeFl_DpdHandle hDfeDpd, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDpd->regs->top_testbus_control, \
			DFE_DPD_TOP_TESTBUS_CONTROL_REG_TESTBUS_CONTROL);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxBlk
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_2X_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_2X_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_O_B0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxBlk(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlk *arg)
{
    volatile uint32_t *regs;

	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		regs = &hDfeDpd->regs->dpd0_mux_blk0;
		break;
	case DFE_FL_DPD_B1:
		regs = &hDfeDpd->regs->dpd1_mux_blk1;
		break;
	case DFE_FL_DPD_B2:
		regs = &hDfeDpd->regs->dpd2_mux_blk2;
		break;
	case DFE_FL_DPD_B3:
		regs = &hDfeDpd->regs->dpd3_mux_blk3;
		break;
	}
	regs[0] = CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_2X_B0, arg->mux_2x)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_2X_B0, arg->mux_dg_2x)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_E_B0, arg->mux_dga_e)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_O_B0, arg->mux_dga_o)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_E_B0, arg->mux_dg_e)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_O_B0, arg->mux_dg_o)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_E_B0, arg->mux_dgaxo_e)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_O_B0, arg->mux_dgaxo_o)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_E_B0, arg->mux_dgxo_e)
			| CSL_FMK(DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_O_B0, arg->mux_dgxo_o);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxBlk
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_2X_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_2X_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_E_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_O_B0
 *       DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_O_B0
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
dfeFl_DpdQueryMuxBlk(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlk *arg)
{
    uint32_t data;

	switch(arg->idxBlk)
	{
	case DFE_FL_DPD_B0:
		data = hDfeDpd->regs->dpd0_mux_blk0;
		break;
	case DFE_FL_DPD_B1:
		data = hDfeDpd->regs->dpd1_mux_blk1;
		break;
	case DFE_FL_DPD_B2:
		data = hDfeDpd->regs->dpd2_mux_blk2;
		break;
	case DFE_FL_DPD_B3:
		data = hDfeDpd->regs->dpd3_mux_blk3;
		break;
	}

	arg->mux_2x = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_2X_B0);
	arg->mux_dg_2x = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_2X_B0);
	arg->mux_dga_e = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_E_B0);
	arg->mux_dga_o = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGA_O_B0);
	arg->mux_dg_e = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_E_B0);
	arg->mux_dg_o = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DG_O_B0);
	arg->mux_dgaxo_e = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_E_B0);
	arg->mux_dgaxo_o = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGAXO_O_B0);
	arg->mux_dgxo_e = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_E_B0);
	arg->mux_dgxo_o = CSL_FEXT(data, DFE_DPD_DPD0_MUX_BLK0_REG_MUX_DGXO_O_B0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxBlk0Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_COMPLEX_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DAXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_REAL_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGAXI_B0_R0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxBlk0Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row5;
		break;
	}
	data = regs[0];
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGAXI_B0_R0, arg->mux_dgaxi);
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGXI_B0_R0, arg->mux_dgxi);
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_REAL_B0_R0, arg->mux_real);
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_COMPLEX_B0_R0, arg->mux_complex);
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DAXI_B0_R0, arg->mux_daxi);
	CSL_FINS(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DXI_B0_R0, arg->mux_dxi);
	regs[0] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxBlk1Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_REAL_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_COMPLEX_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DAXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGAXI_B1_R0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxBlk1Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row5;
		break;
	}
	data = regs[0];
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGAXI_B1_R0, arg->mux_dgaxi);
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGXI_B1_R0, arg->mux_dgxi);
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_REAL_B1_R0, arg->mux_real);
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_COMPLEX_B1_R0, arg->mux_complex);
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DAXI_B1_R0, arg->mux_daxi);
	CSL_FINS(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DXI_B1_R0, arg->mux_dxi);
	regs[0] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxBlk2Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_REAL_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_COMPLEX_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGAXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DAXI_B2_R0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxBlk2Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row5;
		break;
	}
	data = regs[0];
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGAXI_B2_R0, arg->mux_dgaxi);
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGXI_B2_R0, arg->mux_dgxi);
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_REAL_B2_R0, arg->mux_real);
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_COMPLEX_B2_R0, arg->mux_complex);
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DAXI_B2_R0, arg->mux_daxi);
	CSL_FINS(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DXI_B2_R0, arg->mux_dxi);
	regs[0] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateMuxBlk3Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_REAL_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGAXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DAXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_COMPLEX_B3_R0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateMuxBlk3Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row5;
		break;
	}
	data = regs[0];
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGAXI_B3_R0, arg->mux_dgaxi);
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGXI_B3_R0, arg->mux_dgxi);
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_REAL_B3_R0, arg->mux_real);
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_COMPLEX_B3_R0, arg->mux_complex);
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DAXI_B3_R0, arg->mux_daxi);
	CSL_FINS(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DXI_B3_R0, arg->mux_dxi);
	regs[0] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxBlk0Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_COMPLEX_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DAXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_REAL_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DXI_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGAXI_B0_R0
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
dfeFl_DpdQueryMuxBlk0Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd0_row_cell_config_blk0_row5;
		break;
	}
	data = regs[0];
	arg->mux_dgaxi =	CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGAXI_B0_R0);
	arg->mux_dgxi = CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DGXI_B0_R0);
	arg->mux_real = CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_REAL_B0_R0);
	arg->mux_complex = CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_COMPLEX_B0_R0);
	arg->mux_daxi = CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DAXI_B0_R0);
	arg->mux_dxi = CSL_FEXT(data, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_MUX_DXI_B0_R0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxBlk1Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_REAL_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_COMPLEX_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DAXI_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGAXI_B1_R0
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
dfeFl_DpdQueryMuxBlk1Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd1_row_cell_config_blk1_row5;
		break;
	}
	data = regs[0];
	arg->mux_dgaxi =	CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGAXI_B1_R0);
	arg->mux_dgxi = CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DGXI_B1_R0);
	arg->mux_real = CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_REAL_B1_R0);
	arg->mux_complex = CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_COMPLEX_B1_R0);
	arg->mux_daxi = CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DAXI_B1_R0);
	arg->mux_dxi = CSL_FEXT(data, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_MUX_DXI_B1_R0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxBlk2Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_REAL_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_COMPLEX_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGAXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGXI_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DAXI_B2_R0
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
dfeFl_DpdQueryMuxBlk2Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd2_row_cell_config_blk2_row5;
		break;
	}
	data = regs[0];
	arg->mux_dgaxi =	CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGAXI_B2_R0);
	arg->mux_dgxi = CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DGXI_B2_R0);
	arg->mux_real = CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_REAL_B2_R0);
	arg->mux_complex = CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_COMPLEX_B2_R0);
	arg->mux_daxi = CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DAXI_B2_R0);
	arg->mux_dxi = CSL_FEXT(data, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_MUX_DXI_B2_R0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryMuxBlk3Row
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_REAL_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGAXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DAXI_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_COMPLEX_B3_R0
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
dfeFl_DpdQueryMuxBlk3Row(DfeFl_DpdHandle hDfeDpd, DfeFl_DpdMuxBlkRow *arg)
{
	volatile uint32_t *regs;
	uint32_t data;

	switch(arg->idxRow)
	{
	case DFE_FL_DPD_R0:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row0;
		break;
	case DFE_FL_DPD_R1:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row1;
		break;
	case DFE_FL_DPD_R2:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row2;
		break;
	case DFE_FL_DPD_R3:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row3;
		break;
	case DFE_FL_DPD_R4:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row4;
		break;
	case DFE_FL_DPD_R5:
		regs = &hDfeDpd->regs->dpd3_row_cell_config_blk3_row5;
		break;
	}
	data = regs[0];
	arg->mux_dgaxi =	CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGAXI_B3_R0);
	arg->mux_dgxi = CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DGXI_B3_R0);
	arg->mux_real = CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_REAL_B3_R0);
	arg->mux_complex = CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_COMPLEX_B3_R0);
	arg->mux_daxi = CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DAXI_B3_R0);
	arg->mux_dxi = CSL_FEXT(data, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_MUX_DXI_B3_R0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_INIT_B0_R2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_INIT_B0_R4
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_INIT_B0_R3
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_INIT_B0_R1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_INIT_B0_R5
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_INIT_B0_R0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_INIT_B0_R0, LutInit);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_INIT_B0_R1, LutInit);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_INIT_B0_R2, LutInit);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_INIT_B0_R3, LutInit);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_INIT_B0_R4, LutInit);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_INIT_B0_R5, LutInit);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_INIT_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_INIT_B1_R3
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_INIT_B1_R2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_INIT_B1_R4
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_INIT_B1_R1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_INIT_B1_R5
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_INIT_B1_R0, LutInit);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_INIT_B1_R1, LutInit);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_INIT_B1_R2, LutInit);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_INIT_B1_R3, LutInit);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_INIT_B1_R4, LutInit);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_INIT_B1_R5, LutInit);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_INIT_B2_R5
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_INIT_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_INIT_B2_R4
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_INIT_B2_R2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_INIT_B2_R3
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_INIT_B2_R1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_INIT_B2_R0, LutInit);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_INIT_B2_R1, LutInit);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_INIT_B2_R2, LutInit);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_INIT_B2_R3, LutInit);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_INIT_B2_R4, LutInit);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_INIT_B2_R5, LutInit);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_INIT_B3_R3
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_INIT_B3_R4
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_INIT_B3_R5
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_INIT_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_INIT_B3_R1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_INIT_B3_R2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_INIT_B3_R0, LutInit);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_INIT_B3_R1, LutInit);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_INIT_B3_R2, LutInit);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_INIT_B3_R3, LutInit);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_INIT_B3_R4, LutInit);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_INIT_B3_R5, LutInit);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_INIT_B0_R2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_INIT_B0_R4
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_INIT_B0_R3
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_INIT_B0_R1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_INIT_B0_R5
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_INIT_B0_R0
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
dfeFl_DpdQueryBlk0LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_INIT_B0_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_INIT_B0_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_INIT_B0_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_INIT_B0_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_INIT_B0_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_INIT_B0_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_INIT_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_INIT_B1_R3
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_INIT_B1_R2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_INIT_B1_R4
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_INIT_B1_R1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_INIT_B1_R5
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
dfeFl_DpdQueryBlk1LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_INIT_B1_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_INIT_B1_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_INIT_B1_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_INIT_B1_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_INIT_B1_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_INIT_B1_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_INIT_B2_R5
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_INIT_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_INIT_B2_R4
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_INIT_B2_R2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_INIT_B2_R3
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_INIT_B2_R1
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
dfeFl_DpdQueryBlk2LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_INIT_B2_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_INIT_B2_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_INIT_B2_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_INIT_B2_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_INIT_B2_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_INIT_B2_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3LutInit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutInit    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_INIT_B3_R3
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_INIT_B3_R4
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_INIT_B3_R5
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_INIT_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_INIT_B3_R1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_INIT_B3_R2
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
dfeFl_DpdQueryBlk3LutInit(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutInit)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_INIT_B3_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_INIT_B3_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_INIT_B3_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_INIT_B3_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_INIT_B3_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutInit = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_INIT_B3_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_TOGGLE_B0_R1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_TOGGLE_B0_R3
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_TOGGLE_B0_R4
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_TOGGLE_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_TOGGLE_B0_R5
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_TOGGLE_B0_R2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_TOGGLE_B0_R0, LutToggle);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_TOGGLE_B0_R1, LutToggle);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_TOGGLE_B0_R2, LutToggle);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_TOGGLE_B0_R3, LutToggle);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_TOGGLE_B0_R4, LutToggle);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_TOGGLE_B0_R5, LutToggle);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_TOGGLE_B1_R5
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_TOGGLE_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_TOGGLE_B1_R4
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_TOGGLE_B1_R2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_TOGGLE_B1_R1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_TOGGLE_B1_R3
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_TOGGLE_B1_R0, LutToggle);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_TOGGLE_B1_R1, LutToggle);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_TOGGLE_B1_R2, LutToggle);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_TOGGLE_B1_R3, LutToggle);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_TOGGLE_B1_R4, LutToggle);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_TOGGLE_B1_R5, LutToggle);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_TOGGLE_B2_R4
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_TOGGLE_B2_R5
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_TOGGLE_B2_R3
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_TOGGLE_B2_R2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_TOGGLE_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_TOGGLE_B2_R1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_TOGGLE_B2_R0, LutToggle);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_TOGGLE_B2_R1, LutToggle);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_TOGGLE_B2_R2, LutToggle);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_TOGGLE_B2_R3, LutToggle);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_TOGGLE_B2_R4, LutToggle);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_TOGGLE_B2_R5, LutToggle);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_TOGGLE_B3_R1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_TOGGLE_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_TOGGLE_B3_R3
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_TOGGLE_B3_R2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_TOGGLE_B3_R4
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_TOGGLE_B3_R5
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_TOGGLE_B3_R0, LutToggle);
		break;
	case DFE_FL_DPD_R1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_TOGGLE_B3_R1, LutToggle);
		break;
	case DFE_FL_DPD_R2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_TOGGLE_B3_R2, LutToggle);
		break;
	case DFE_FL_DPD_R3:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_TOGGLE_B3_R3, LutToggle);
		break;
	case DFE_FL_DPD_R4:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_TOGGLE_B3_R4, LutToggle);
		break;
	case DFE_FL_DPD_R5:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_TOGGLE_B3_R5, LutToggle);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_TOGGLE_B0_R1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_TOGGLE_B0_R3
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_TOGGLE_B0_R4
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_TOGGLE_B0_R0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_TOGGLE_B0_R5
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_TOGGLE_B0_R2
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
dfeFl_DpdQueryBlk0LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_LUT_TOGGLE_B0_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_LUT_TOGGLE_B0_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_LUT_TOGGLE_B0_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_LUT_TOGGLE_B0_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_LUT_TOGGLE_B0_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_LUT_TOGGLE_B0_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_TOGGLE_B1_R5
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_TOGGLE_B1_R0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_TOGGLE_B1_R4
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_TOGGLE_B1_R2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_TOGGLE_B1_R1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_TOGGLE_B1_R3
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
dfeFl_DpdQueryBlk1LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_LUT_TOGGLE_B1_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_LUT_TOGGLE_B1_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_LUT_TOGGLE_B1_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_LUT_TOGGLE_B1_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_LUT_TOGGLE_B1_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_LUT_TOGGLE_B1_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_TOGGLE_B2_R4
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_TOGGLE_B2_R5
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_TOGGLE_B2_R3
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_TOGGLE_B2_R2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_TOGGLE_B2_R0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_TOGGLE_B2_R1
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
dfeFl_DpdQueryBlk2LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_LUT_TOGGLE_B2_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_LUT_TOGGLE_B2_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_LUT_TOGGLE_B2_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_LUT_TOGGLE_B2_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_LUT_TOGGLE_B2_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_LUT_TOGGLE_B2_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3LutToggle
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         LutToggle    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_TOGGLE_B3_R1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_TOGGLE_B3_R0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_TOGGLE_B3_R3
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_TOGGLE_B3_R2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_TOGGLE_B3_R4
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_TOGGLE_B3_R5
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
dfeFl_DpdQueryBlk3LutToggle(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t *LutToggle)
{
	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_LUT_TOGGLE_B3_R0);
		break;
	case DFE_FL_DPD_R1:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_LUT_TOGGLE_B3_R1);
		break;
	case DFE_FL_DPD_R2:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_LUT_TOGGLE_B3_R2);
		break;
	case DFE_FL_DPD_R3:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_LUT_TOGGLE_B3_R3);
		break;
	case DFE_FL_DPD_R4:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_LUT_TOGGLE_B3_R4);
		break;
	case DFE_FL_DPD_R5:
		*LutToggle = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_LUT_TOGGLE_B3_R5);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk0Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk1Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk2Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C2
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C1
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_DpdUpdateBlk3Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C0, Ssel);
		break;
	case DFE_FL_DPD_C1:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C1, Ssel);
		break;
	case DFE_FL_DPD_C2:
		CSL_FINS(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C2, Ssel);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C2
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
dfeFl_DpdQueryBlk0Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row0, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW0_REG_SYNCH_CELL_B0_R0_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C0
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
dfeFl_DpdQueryBlk0Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row1, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW1_REG_SYNCH_CELL_B0_R1_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C2
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
dfeFl_DpdQueryBlk0Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row2, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW2_REG_SYNCH_CELL_B0_R2_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C0
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
dfeFl_DpdQueryBlk0Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row3, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW3_REG_SYNCH_CELL_B0_R3_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C0
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C2
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
dfeFl_DpdQueryBlk0Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row4, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW4_REG_SYNCH_CELL_B0_R4_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C2
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C1
 *       DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C0
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
dfeFl_DpdQueryBlk0Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd0_row_cell_config_blk0_row5, DFE_DPD_DPD0_ROW_CELL_CONFIG_BLK0_ROW5_REG_SYNCH_CELL_B0_R5_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C1
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
dfeFl_DpdQueryBlk1Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row0, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW0_REG_SYNCH_CELL_B1_R0_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C2
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
dfeFl_DpdQueryBlk1Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row1, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW1_REG_SYNCH_CELL_B1_R1_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C1
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
dfeFl_DpdQueryBlk1Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row2, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW2_REG_SYNCH_CELL_B1_R2_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C2
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
dfeFl_DpdQueryBlk1Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row3, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW3_REG_SYNCH_CELL_B1_R3_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C2
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C1
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
dfeFl_DpdQueryBlk1Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row4, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW4_REG_SYNCH_CELL_B1_R4_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C0
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C1
 *       DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C2
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
dfeFl_DpdQueryBlk1Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd1_row_cell_config_blk1_row5, DFE_DPD_DPD1_ROW_CELL_CONFIG_BLK1_ROW5_REG_SYNCH_CELL_B1_R5_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C0
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
dfeFl_DpdQueryBlk2Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row0, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW0_REG_SYNCH_CELL_B2_R0_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C2
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
dfeFl_DpdQueryBlk2Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row1, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW1_REG_SYNCH_CELL_B2_R1_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C0
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
dfeFl_DpdQueryBlk2Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row2, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW2_REG_SYNCH_CELL_B2_R2_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C2
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
dfeFl_DpdQueryBlk2Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row3, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW3_REG_SYNCH_CELL_B2_R3_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C2
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C0
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
dfeFl_DpdQueryBlk2Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row4, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW4_REG_SYNCH_CELL_B2_R4_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C1
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C0
 *       DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C2
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
dfeFl_DpdQueryBlk2Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd2_row_cell_config_blk2_row5, DFE_DPD_DPD2_ROW_CELL_CONFIG_BLK2_ROW5_REG_SYNCH_CELL_B2_R5_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row0CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C2
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
dfeFl_DpdQueryBlk3Row0CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row0, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW0_REG_SYNCH_CELL_B3_R0_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row1CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C1
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
dfeFl_DpdQueryBlk3Row1CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row1, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW1_REG_SYNCH_CELL_B3_R1_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row2CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C2
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
dfeFl_DpdQueryBlk3Row2CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row2, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW2_REG_SYNCH_CELL_B3_R2_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row3CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C1
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
dfeFl_DpdQueryBlk3Row3CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row3, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW3_REG_SYNCH_CELL_B3_R3_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row4CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C1
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C2
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
dfeFl_DpdQueryBlk3Row4CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row4, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW4_REG_SYNCH_CELL_B3_R4_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row5CellSync
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         Ssel    [add content]
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
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C2
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C0
 *       DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C1
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
dfeFl_DpdQueryBlk3Row5CellSync(DfeFl_DpdHandle hDfeDpd, uint32_t idxCell, uint32_t *Ssel)
{
	switch(idxCell)
	{
	case DFE_FL_DPD_C0:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C0);
		break;
	case DFE_FL_DPD_C1:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C1);
		break;
	case DFE_FL_DPD_C2:
		*Ssel = CSL_FEXT(hDfeDpd->regs->dpd3_row_cell_config_blk3_row5, DFE_DPD_DPD3_ROW_CELL_CONFIG_BLK3_ROW5_REG_SYNCH_CELL_B3_R5_C2);
		break;
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0CurLutMpu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         idxCell    [add content]
         CurLutMpu    [add content]
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
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R3
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R2
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R1
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R0
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R5
 *       DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R4
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
dfeFl_DpdQueryBlk0CurLutMpu(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *CurLutMpu)
{
	uint32_t data;

	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R0);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R1:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R1);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R2:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R2);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R3:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R3);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R4:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R4);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R5:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd0_current_lut_mpu_blk0, DFE_DPD_DPD0_CURRENT_LUT_MPU_BLK0_REG_CURRENT_LUT_MPU_B0_R5);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1CurLutMpu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         idxCell    [add content]
         CurLutMpu    [add content]
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
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R4
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R5
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R2
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R3
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R0
 *       DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R1
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
dfeFl_DpdQueryBlk1CurLutMpu(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *CurLutMpu)
{
	uint32_t data;

	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R0);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R1:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R1);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R2:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R2);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R3:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R3);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R4:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R4);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R5:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd1_current_lut_mpu_blk1, DFE_DPD_DPD1_CURRENT_LUT_MPU_BLK1_REG_CURRENT_LUT_MPU_B1_R5);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2CurLutMpu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         idxCell    [add content]
         CurLutMpu    [add content]
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
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R5
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R4
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R1
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R0
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R3
 *       DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R2
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
dfeFl_DpdQueryBlk2CurLutMpu(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *CurLutMpu)
{
	uint32_t data;

	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R0);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R1:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R1);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R2:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R2);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R3:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R3);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R4:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R4);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R5:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd2_current_lut_mpu_blk2, DFE_DPD_DPD2_CURRENT_LUT_MPU_BLK2_REG_CURRENT_LUT_MPU_B2_R5);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3CurLutMpu
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxRow    [add content]
         idxCell    [add content]
         CurLutMpu    [add content]
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
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R0
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R1
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R2
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R3
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R4
 *       DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R5
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
dfeFl_DpdQueryBlk3CurLutMpu(DfeFl_DpdHandle hDfeDpd, uint32_t idxRow, uint32_t idxCell, uint32_t *CurLutMpu)
{
	uint32_t data;

	switch(idxRow)
	{
	case DFE_FL_DPD_R0:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R0);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R1:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R1);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R2:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R2);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R3:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R3);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R4:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R4);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	case DFE_FL_DPD_R5:
	{
		data = CSL_FEXT(hDfeDpd->regs->dpd3_current_lut_mpu_blk3, DFE_DPD_DPD3_CURRENT_LUT_MPU_BLK3_REG_CURRENT_LUT_MPU_B3_R5);
		*CurLutMpu = CSL_FEXTR(data, idxCell, idxCell);
		break;
	}
	default:
		return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk0Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk0Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk1Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk1Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk2Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk2Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdUpdateBlk3Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdUpdateBlk3Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt lutGain,
		DfeFl_DpdComplexInt lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	uint32_t data;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.real);
	CSL_FINSR(data, 15, 0, lutGain.real);
	regs_lut[r] = data;
	data = 0;
	CSL_FINSR(data, 25, 16, lutSlope.imag);
	CSL_FINSR(data, 15, 0, lutGain.imag);
	regs_lut[r+1] = data;
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk0Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk0Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd0_dpdlut_b0_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk1Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk1Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd1_dpdlut_b1_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk2Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk2Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd2_dpdlut_b2_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row0LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row0LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r0_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row1LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row1LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r1_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row2LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row2LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r2_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row3LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row3LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r3_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row4LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row4LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r4_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}

/** ============================================================================
 *   @n@b dfeFl_DpdQueryBlk3Row5LUT
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDpd    [add content]
         idxCell    [add content]
         idxEntry    [add content]
         lutGain    [add content]
         lutSlope    [add content]
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
dfeFl_DpdQueryBlk3Row5LUT
(
		DfeFl_DpdHandle hDfeDpd,
		uint32_t idxCell,
		uint32_t idxEntry,
		DfeFl_DpdComplexInt *lutGain,
		DfeFl_DpdComplexInt *lutSlope
)
{
	uint32_t r;
	volatile uint32_t *regs_lut;
	r = idxEntry * 2;

	switch (idxCell)
	{
	case DFE_FL_DPD_C0:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c0[0];
		break;
	case DFE_FL_DPD_C1:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c1[0];
		break;
	case DFE_FL_DPD_C2:
		regs_lut = &hDfeDpd->regs->dpd3_dpdlut_b3_r5_c2[0];
		break;
	default:
		return;
	}

	// data[25:0] = {slope_real[9:0], gain_real[15:0]} for even addresses
	// data[25:0] = {slope_imag[9:0], gain_imag[15:0]} for odd addresses
	lutSlope->real = CSL_FEXTR(regs_lut[r], 25, 16);
	lutGain->real = CSL_FEXTR(regs_lut[r], 15, 0);
	lutSlope->imag = CSL_FEXTR(regs_lut[r+1], 25, 16);
	lutGain->imag = CSL_FEXTR(regs_lut[r+1], 15, 0);
}
#endif /* _DFE_FL_DPDAUX_H_ */
