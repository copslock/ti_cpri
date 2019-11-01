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

/** @file dfe_fl_cfrAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_CFR CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_CFRAUX_H_
#define _DFE_FL_CFRAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_cfr.h>

/** ============================================================================
 *   @n@b dfeFl_CfrConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
 *       DFE_CFR_INITS_CLKS_REG_INIT_CLK_GATE
 *       DFE_CFR_INITS_CLKS_REG_CLEAR_DATA
 *       DFE_CFR_INITS_CLKS_REG_INIT_STATE
 *       DFE_CFR_INITS_CLKS_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_CfrConfigInits(DfeFl_CfrHandle hDfeCfr, DfeFl_SublkInitsConfig * arg)
{
    
    uint32_t data = hDfeCfr->regs->inits_clks;
    
    CSL_FINS(data, DFE_CFR_INITS_CLKS_REG_INITS_SSEL_3_0, arg->ssel);
    CSL_FINS(data, DFE_CFR_INITS_CLKS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_CFR_INITS_CLKS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_CFR_INITS_CLKS_REG_CLEAR_DATA, arg->clearData);
    
    
    hDfeCfr->regs->inits_clks = data;
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetPremSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *       DFE_CFR_CFR1_CFR_UPDT_GEN_1_REG_PREM_SSEL
 *       DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_PREM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_CfrSetPremSsel(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t ssel)
{
    volatile uint32_t *regs[2];
    
    regs[0] =  &hDfeCfr->regs->cfr0_cfr_updt_gen_0;
    regs[1] =  &hDfeCfr->regs->cfr1_cfr_updt_gen_1;
    
    if(path == DFE_FL_CFR_PATH_ALL)
    {
        CSL_FINS(*regs[0], DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_CFR_PREM_0_SSEL, ssel);
        CSL_FINS(*regs[1], DFE_CFR_CFR1_CFR_UPDT_GEN_1_REG_CFR_PREM_0_SSEL, ssel);
    }
    else if(path <= DFE_FL_CFR_PATH_1)
    {
        CSL_FINS(*regs[path], DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_CFR_PREM_0_SSEL, ssel);
    }
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetPostmSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *       DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_POSTM_SSEL
 *       DFE_CFR_CFR1_CFR_UPDT_GEN_1_REG_POSTM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_CfrSetPostmSsel(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t ssel)
{
    volatile uint32_t *regs[2];
    
    regs[0] =  &hDfeCfr->regs->cfr0_cfr_updt_gen_0;
    regs[1] =  &hDfeCfr->regs->cfr1_cfr_updt_gen_1;
    
    if(path == DFE_FL_CFR_PATH_ALL)
    {
        CSL_FINS(*regs[0], DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_CFR_POSTM_0_SSEL, ssel);
        CSL_FINS(*regs[1], DFE_CFR_CFR1_CFR_UPDT_GEN_1_REG_CFR_POSTM_0_SSEL, ssel);
    }
    else if(path <= DFE_FL_CFR_PATH_1)
    {
        CSL_FINS(*regs[path], DFE_CFR_CFR0_CFR_UPDT_GEN_0_REG_CFR_POSTM_0_SSEL, ssel);
    }
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetPreGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
         gain    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR0_CFR_PREM_A0_REG_PRECFRGAIN
 *       DFE_CFR_CFR1_CFR_PREM_A1_REG_PRECFRGAIN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetPreGain(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t gain)
{
    volatile uint32_t *regs[2];

    regs[0] =  &hDfeCfr->regs->cfr0_cfr_prem_a0;
    regs[1] =  &hDfeCfr->regs->cfr1_cfr_prem_a1;

    if(path == DFE_FL_CFR_PATH_ALL)
    {
        CSL_FINS(*regs[0], DFE_CFR_CFR0_CFR_PREM_A0_REG_CFR0_PRECFRGAIN_0, gain);
        CSL_FINS(*regs[1], DFE_CFR_CFR1_CFR_PREM_A1_REG_CFR1_PRECFRGAIN_1, gain);
    }
    else if(path <= DFE_FL_CFR_PATH_1)
    {
        CSL_FINS(*regs[path], DFE_CFR_CFR0_CFR_PREM_A0_REG_CFR0_PRECFRGAIN_0, gain);
    }
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetPostGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
         gain    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR1_CFR_POSTM_A1_REG_POSTCFRGAIN
 *       DFE_CFR_CFR0_CFR_POSTM_A0_REG_POSTCFRGAIN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetPostGain(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t gain)
{
    volatile uint32_t *regs[2];

    regs[0] =  &hDfeCfr->regs->cfr0_cfr_postm_a0;
    regs[1] =  &hDfeCfr->regs->cfr1_cfr_postm_a1;

    if(path == DFE_FL_CFR_PATH_ALL)
    {
        CSL_FINS(*regs[0], DFE_CFR_CFR0_CFR_POSTM_A0_REG_CFR0_POSTCFRGAIN_0, gain);
        CSL_FINS(*regs[1], DFE_CFR_CFR1_CFR_POSTM_A1_REG_CFR1_POSTCFRGAIN_1, gain);
    }
    else if(path <= DFE_FL_CFR_PATH_1)
    {
        CSL_FINS(*regs[path], DFE_CFR_CFR0_CFR_POSTM_A0_REG_CFR0_POSTCFRGAIN_0, gain);
    }
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetProtPAGain
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
         gain    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR1_CFR_PA_PROTECT_A1_REG_PROTPAGAIN
 *       DFE_CFR_CFR0_CFR_PA_PROTECT_A0_REG_PROTPAGAIN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetProtPAGain(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t gain)
{
    volatile uint32_t *regs[2];

    regs[0] =  &hDfeCfr->regs->cfr0_cfr_pa_protect_a0;
    regs[1] =  &hDfeCfr->regs->cfr1_cfr_pa_protect_a1;

    if(path == DFE_FL_CFR_PATH_ALL)
    {
        CSL_FINS(*regs[0], DFE_CFR_CFR0_CFR_PA_PROTECT_A0_REG_CFR0_PROTPAGAIN_0, gain);
        CSL_FINS(*regs[1], DFE_CFR_CFR1_CFR_PA_PROTECT_A1_REG_CFR1_PROTPAGAIN_1, gain);
    }
    else if(path <= DFE_FL_CFR_PATH_1)
    {
        CSL_FINS(*regs[path], DFE_CFR_CFR0_CFR_PA_PROTECT_A0_REG_CFR0_PROTPAGAIN_0, gain);
    }
}

/** ============================================================================
 *   @n@b dfeFl_CfrEnablePdc0Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrEnablePdc0Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_MASK, 1);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrEnablePdc1Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrEnablePdc1Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_MASK, 1);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_MASK, 1);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrDisablePdc0Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrDisablePdc0Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_MASK, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrDisablePdc1Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_MASK
 *       DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrDisablePdc1Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_MASK, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_mask, DFE_CFR_CFR_PDC_INTR_MASK_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_MASK, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetForcePdc0Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetForcePdc0Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_FORCE, 1);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetForcePdc1Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetForcePdc1Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_FORCE, 1);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_FORCE, 1);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearForcePdc0Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrClearForcePdc0Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_FORCE, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearForcePdc1Intr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_FORCE
 *       DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrClearForcePdc1Intr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_FORCE, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_force, DFE_CFR_CFR_PDC_INTR_FORCE_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_FORCE, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearPdc0IntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrClearPdc0IntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_STATUS, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearPdc1IntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrClearPdc1IntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_STATUS, 0);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		CSL_FINS(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_STATUS, 0);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrGetPdc0IntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
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
 *   @n  see below,
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_STATUS
 *
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
dfeFl_CfrGetPdc0IntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr, uint32_t *status)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S0_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A0S1_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S0_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_NO_CPGE_A1S1_INTR_STATUS);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC0_LUTS_UPDT_DONE_INTR_STATUS);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrGetPdc1IntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         pdcIntr    [add content]
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
 *   @n  see below,
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_STATUS
 *       DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_STATUS
 *
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
dfeFl_CfrGetPdc1IntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrPdcIntr pdcIntr, uint32_t *status)
{
	switch(pdcIntr)
	{
	case DFE_FL_CFR_CPAGE_A0S0:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S0_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A0S1:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A0S1_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A1S0:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S0_INTR_STATUS);
		break;
	case DFE_FL_CFR_CPAGE_A1S1:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_NO_CPGE_A1S1_INTR_STATUS);
		break;
	case DFE_FL_CFR_LUTS_UPDT_DONE:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_pdc_intr_status, DFE_CFR_CFR_PDC_INTR_STATUS_REG_CFR0_PDC1_LUTS_UPDT_DONE_INTR_STATUS);
		break;
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrEnableAgcInOutIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrEnableAgcInOutIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrEnableAgcSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrEnableAgcSyncIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrDisableAgcInOutIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrDisableAgcInOutIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrDisableAgcSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrDisableAgcSyncIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetForceAgcInOutIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrSetForceAgcInOutIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetForceAgcSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrSetForceAgcSyncIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearForceAgcInOutIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearForceAgcInOutIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearForceAgcSyncIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearForceAgcSyncIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearAgcInOutIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearAgcInOutIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearAgcSyncIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearAgcSyncIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrGetAgcInOutIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         arg    [add content]
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
CSL_IDEF_INLINE void
dfeFl_CfrGetAgcInOutIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcInOutIntrCfg *arg, uint32_t *status)
{
	uint32_t b;
	b = arg->path*4+arg->intr;

	*status = CSL_FEXTR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b);
}

/** ============================================================================
 *   @n@b dfeFl_CfrGetAgcSyncIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         arg    [add content]
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
CSL_IDEF_INLINE void
dfeFl_CfrGetAgcSyncIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrAgcSyncIntrCfg *arg, uint32_t *status)
{
	uint32_t b;
	b = arg->path*2+arg->intr+8;

	*status = CSL_FEXTR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b);
}

/** ============================================================================
 *   @n@b dfeFl_CfrEnableDthIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrEnableDthIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrDisableDthIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrDisableDthIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_mask, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetForceDthIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrSetForceDthIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 1);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearForceDthIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearForceDthIntr(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_force, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrClearDthIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
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
dfeFl_CfrClearDthIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	CSL_FINSR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b, 0);
}

/** ============================================================================
 *   @n@b dfeFl_CfrGetDthIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         arg    [add content]
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
CSL_IDEF_INLINE void
dfeFl_CfrGetDthIntrStatus(DfeFl_CfrHandle hDfeCfr, DfeFl_CfrDthIntrCfg *arg, uint32_t *status)
{
	uint32_t b;
	b = arg->intr*2+arg->path+12;

	*status = CSL_FEXTR(hDfeCfr->regs->cfr_agc_dth_intr_status, b, b);
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetLutSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *       DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_LUT_P0_SSEL
 *       DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_LUT_P1_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetLutSsel(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t ssel)
{
	switch (path)
	{
	case DFE_FL_CFR_PATH_0:
		CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P0_F0_SC0_SSEL, ssel);
		break;
	case DFE_FL_CFR_PATH_1:
		CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P1_F0_SC0_SSEL, ssel);
		break;
	case DFE_FL_CFR_PATH_ALL:
	{
        CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P0_F0_SC0_SSEL, ssel);
        CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P1_F0_SC0_SSEL, ssel);
        break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrSetHdlySsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *       DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_LUT_P0HDLY_SSEL
 *       DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_LUT_P1HDLY_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrSetHdlySsel(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t ssel)
{
	switch (path)
	{
	case DFE_FL_CFR_PATH_0:
		CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P0_F1_SC0_SSEL, ssel);
		break;
	case DFE_FL_CFR_PATH_1:
		CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P1_F1_SC0_SSEL, ssel);
		break;
	case DFE_FL_CFR_PATH_ALL:
	{
        CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P0_F1_SC0_SSEL, ssel);
        CSL_FINS(hDfeCfr->regs->cfr_cfr_updt_gen_2, DFE_CFR_CFR_CFR_UPDT_GEN_2_REG_CFR_LUT_P1_F1_SC0_SSEL, ssel);
        break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b DfeFl_CfrGetBusyStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *   @n  see below,
 *       DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P1
 *       DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P0
 *
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
DfeFl_CfrGetBusyStatus(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t *status)
{
	uint32_t data;
	switch (path)
	{
	case DFE_FL_CFR_PATH_0:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_cfr_status, DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P0);
		break;
	case DFE_FL_CFR_PATH_1:
		*status = CSL_FEXT(hDfeCfr->regs->cfr_cfr_status, DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P1);
		break;
	case DFE_FL_CFR_PATH_ALL:
	{
        data = CSL_FEXT(hDfeCfr->regs->cfr_cfr_status, DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P0);
        data |= CSL_FEXT(hDfeCfr->regs->cfr_cfr_status, DFE_CFR_CFR_CFR_STATUS_REG_CFR_STATUS_P1) << 1;
        *status = data;
        break;
	}
    default:
    	return;
	}
}

/** ============================================================================
 *   @n@b dfeFl_CfrUpdtLutCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeCfr    [add content]
         path    [add content]
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
 *       DFE_CFR_PDC0_PDC_LUTS_SW_SHDW_I_REG_CALC_CPCOEFFS_FRACLOC_RELOAD
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void
dfeFl_CfrUpdtLutCoeff(DfeFl_CfrHandle hDfeCfr, uint32_t path, uint32_t idx, uint32_t data)
{
	volatile uint32_t *regs;

	switch(path)
	{
	case DFE_FL_CFR_PDC0_I:
		regs = &hDfeCfr->regs->pdc0_pdc_luts_sw_shdw_i[0];
		break;
	case DFE_FL_CFR_PDC0_Q:
		regs = &hDfeCfr->regs->pdc0_pdc_luts_sw_shdw_q[0];
		break;
	case DFE_FL_CFR_PDC0_IQ:
		regs = &hDfeCfr->regs->pdc0_pdc_luts_sw_shdw_iq[0];
		break;
	case DFE_FL_CFR_PDC1_I:
		regs = &hDfeCfr->regs->pdc1_pdc_luts_sw_shdw_i[0];
		break;
	case DFE_FL_CFR_PDC1_Q:
		regs = &hDfeCfr->regs->pdc1_pdc_luts_sw_shdw_q[0];
		break;
	case DFE_FL_CFR_PDC1_IQ:
		regs = &hDfeCfr->regs->pdc1_pdc_luts_sw_shdw_iq[0];
		break;
	case DFE_FL_CFR_PDC01_I:
		regs = &hDfeCfr->regs->pdc_01_pdc_luts_sw_shdw_i[0];
		break;
	case DFE_FL_CFR_PDC01_Q:
		regs = &hDfeCfr->regs->pdc_01_pdc_luts_sw_shdw_q[0];
		break;
	case DFE_FL_CFR_PDC01_IQ:
		regs = &hDfeCfr->regs->pdc_01_pdc_luts_sw_shdw_iq[0];
		break;
    default:
    	return;
	}

	CSL_FINS(regs[idx], DFE_CFR_PDC0_PDC_LUTS_SW_SHDW_I_REG_PDC0_PDC_LUTS_SW_SHDW_I, data);
}
#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_CFRAUX_H_ */
