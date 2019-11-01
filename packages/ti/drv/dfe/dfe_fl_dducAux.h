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

/** @file dfe_fl_dducAux.h
 *
 *  @path  $(CSLPATH)\inc
 *
 *  @brief Header file for functional layer of DFE_DDUC CSL
 *
 *  Description
 *  - Function level symbolic constants, enumerations, structure definitions
 *    and function prototype declarations
 *
 */
#ifndef _DFE_FL_DDUCAUX_H_
#define _DFE_FL_DDUCAUX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/csl/csl.h>
#include <ti/drv/dfe/dfe_fl_dduc.h>

/** ============================================================================
 *   @n@b dfeFl_DducConfigInits
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_INITS_CLKS_REG_CLEAR_DATA
 *       DFE_DDUC_INITS_CLKS_REG_INIT_STATE
 *       DFE_DDUC_INITS_CLKS_REG_INIT_CLK_GATE
 *       DFE_DDUC_INITS_CLKS_REG_INITS_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
CSL_IDEF_INLINE void 
dfeFl_DducConfigInits(DfeFl_DducHandle hDfeDduc, DfeFl_SublkInitsConfig * arg)
{
    uint32_t data = hDfeDduc->regs->inits_clks;
    
    CSL_FINS(data, DFE_DDUC_INITS_CLKS_REG_INITS_SSEL, arg->ssel);
    CSL_FINS(data, DFE_DDUC_INITS_CLKS_REG_INIT_CLK_GATE, arg->initClkGate);
    CSL_FINS(data, DFE_DDUC_INITS_CLKS_REG_INIT_STATE, arg->initState);
    CSL_FINS(data, DFE_DDUC_INITS_CLKS_REG_CLEAR_DATA, arg->clearData);
    
    hDfeDduc->regs->inits_clks = data;
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetFrwPhaseSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL1_REG_FRW_PHASE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetFrwPhaseSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel1, DFE_DDUC_SSEL1_REG_FRW_PHASE_SSEL, ssel);
}
   
CSL_IDEF_INLINE 
void dfeFl_DducConfigFrwPhase(DfeFl_DducHandle hDfeDduc, uint32_t phase[12])
{
    uint32_t i;
    
    for(i = 0; i < 12; i++)
    {
        hDfeDduc->regs->frw_phase[i].lobank0 = CSL_FMK(DFE_DDUC_FRW_PHASE_LOBANK0_REG_PHASE_SWAP_BANK_0_15_0, phase[i]);
        hDfeDduc->regs->frw_phase[i].hibank0 = CSL_FMK(DFE_DDUC_FRW_PHASE_HIBANK0_REG_PHASE_SWAP_BANK_0_25_16, phase[i] >> 16);
    }        
}   

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetFrwPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
         phase    [add content]
     @endverbatim
 *
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
 *       DFE_DDUC_FRW_PHASE_LOBANK0_REG_PHASE_SWAP_BANK_0_15_0
 *       DFE_DDUC_FRW_PHASE_HIBANK0_REG_PHASE_SWAP_BANK_0_25_16
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetFrwPhase(DfeFl_DducHandle hDfeDduc, uint32_t *phase)
{
    uint32_t i;

    for(i = 0; i < 12; i++)
    {
    	phase[i] = CSL_FEXT(hDfeDduc->regs->frw_phase[i].hibank0, DFE_DDUC_FRW_PHASE_HIBANK0_REG_PHASE_SWAP_BANK_0_25_16);
    	phase[i] = phase[i]<<16 + CSL_FEXT(hDfeDduc->regs->frw_phase[i].lobank0, DFE_DDUC_FRW_PHASE_LOBANK0_REG_PHASE_SWAP_BANK_0_15_0);
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetFrwChksumSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL1_REG_FRW_CHKSUM_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetFrwChksumSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel1, DFE_DDUC_SSEL1_REG_FRW_CHKSUM_SSEL, ssel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetFrwSigSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL2_REG_FRW_SIG_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetFrwSigSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel1, DFE_DDUC_SSEL2_REG_FRW_SIG_SSEL, ssel);
}
      
CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetHopOffsetSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL0_REG_HOP_HO_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetHopOffsetSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel0, DFE_DDUC_SSEL0_REG_HOP_HO_SSEL, ssel);
}

CSL_IDEF_INLINE 
void dfeFl_DducConfigHopOffset(DfeFl_DducHandle hDfeDduc, uint32_t offset[12])
{
    uint32_t i;
    
    for(i = 0; i < 12; i++)
    {
        hDfeDduc->regs->hop_offset[i].bank0 = CSL_FMK(DFE_DDUC_HOP_OFFSET_BANK0_REG_HOP_OFFSET_SWAP_BANK_0, offset[i]);
    }        
}   

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetHopSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL0_REG_HOP_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetHopSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel0, DFE_DDUC_SSEL0_REG_HOP_SSEL, ssel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetSyncDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_HOP_SYNC_DELAY_REG_SYNC_DELAY
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetSyncDelay(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
	CSL_FINS(hDfeDduc->regs->hop_sync_delay, DFE_DDUC_HOP_SYNC_DELAY_REG_SYNC_DELAY, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetSyncDelay
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_HOP_SYNC_DELAY_REG_SYNC_DELAY
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetSyncDelay(DfeFl_DducHandle hDfeDduc, uint32_t *data)
{
	*data = CSL_FEXT(hDfeDduc->regs->hop_sync_delay, DFE_DDUC_HOP_SYNC_DELAY_REG_SYNC_DELAY);
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetMixNcoSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
         nco    [add content]
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
 *       DFE_DDUC_SSEL3_REG_MIX0_NCO_SSEL
 *       DFE_DDUC_SSEL3_REG_MIX2_NCO_SSEL
 *       DFE_DDUC_SSEL3_REG_MIX1_NCO_SSEL
 *       DFE_DDUC_SSEL3_REG_MIX3_NCO_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetMixNcoSsel(DfeFl_DducHandle hDfeDduc, uint32_t nco, uint32_t ssel)
{
    volatile uint32_t *regs = &hDfeDduc->regs->ssel3;
    uint32_t r, b;
    
    if(nco == DFE_FL_DDUC_MIX_NCO_ALL)
    {
        r = CSL_FMK(DFE_DDUC_SSEL3_REG_MIX0_NCO_SSEL, ssel)
          | CSL_FMK(DFE_DDUC_SSEL3_REG_MIX1_NCO_SSEL, ssel)
          | CSL_FMK(DFE_DDUC_SSEL3_REG_MIX2_NCO_SSEL, ssel)
          | CSL_FMK(DFE_DDUC_SSEL3_REG_MIX3_NCO_SSEL, ssel);
          
        regs[0] = r;
        regs[1] = r;
        regs[2] = r;          
    }
    else if(nco <= DFE_FL_DDUC_MIX_NCO_11)
    {
        r = nco / 4;
        b = (nco % 4) * 4;
        
        CSL_FINSR(regs[r], b+3, b, ssel);
    }        
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetMixPhaseSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL1_REG_MIX_PHASE_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetMixPhaseSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel1, DFE_DDUC_SSEL1_REG_MIX_PHASE_SSEL, ssel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgMixDit
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_MIX_CONFIG_REG_MIX_DIT_EN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgMixDit(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINS(hDfeDduc->regs->mix_config, DFE_DDUC_MIX_CONFIG_REG_MIX_DIT_EN, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgMixMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
         MixMode    [add content]
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
void dfeFl_DducCfgMixMode(DfeFl_DducHandle hDfeDduc, uint32_t MixMode, uint32_t data)
{
	CSL_FINSR(hDfeDduc->regs->mix_config, (MixMode+1), (MixMode+1), data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetMixPhase
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
         iMix    [add content]
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
 *       DFE_DDUC_MIX_PHASE_REG_PHASE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetMixPhase(DfeFl_DducHandle hDfeDduc, uint32_t iMix, uint32_t data)
{
    volatile uint32_t *regs = &hDfeDduc->regs->mix_phase[0];
    uint32_t i;

    if(iMix == DFE_FL_DDUC_MIX_NCO_ALL)
    {
        for (i=0; i<12; i++)
        {
        	regs[i] = CSL_FMK(DFE_DDUC_MIX_PHASE_REG_PHASE, data);
        }
    }
    else if(iMix <= DFE_FL_DDUC_MIX_NCO_11)
    {
    	regs[iMix] = CSL_FMK(DFE_DDUC_MIX_PHASE_REG_PHASE, data);
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducConfigHopFrqword
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducConfigHopFrqword(DfeFl_DducHandle hDfeDduc, DfeFl_DducHopFrqwordConfig *cfg)
{
    uint32_t i;
    
    for(i = 0; i < 4; i++)
    {
        hDfeDduc->regs->hop_mix0_freq_word_lo[i].bank0 = cfg->frqWord[i].low;
        hDfeDduc->regs->hop_mix0_freq_word_mid[i].bank0 = cfg->frqWord[i].mid;
        hDfeDduc->regs->hop_mix0_freq_word_hi[i].bank0 = cfg->frqWord[i].high;
        
        hDfeDduc->regs->hop_mix1_freq_word_lo[i].bank0 = cfg->frqWord[i+4].low;
        hDfeDduc->regs->hop_mix1_freq_word_mid[i].bank0 = cfg->frqWord[i+4].mid;
        hDfeDduc->regs->hop_mix1_freq_word_hi[i].bank0 = cfg->frqWord[i+4].high;

        hDfeDduc->regs->hop_mix2_freq_word_lo[i].bank0 = cfg->frqWord[i+8].low;
        hDfeDduc->regs->hop_mix2_freq_word_mid[i].bank0 = cfg->frqWord[i+8].mid;
        hDfeDduc->regs->hop_mix2_freq_word_hi[i].bank0 = cfg->frqWord[i+8].high;
    }
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetCicFlushSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL1_REG_CIC_FLUSH_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetCicFlushSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel1, DFE_DDUC_SSEL1_REG_CIC_FLUSH_SSEL, ssel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgCic
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CIC_CONFIG_REG_CIC_NDATA
 *       DFE_DDUC_CIC_CONFIG_REG_CIC_RX_OUTPUT_SCALE
 *       DFE_DDUC_CIC_CONFIG_REG_FL_EN
 *       DFE_DDUC_CIC_CONFIG_REG_RATE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgCic(DfeFl_DducHandle hDfeDduc, DfeFl_DducCicCfg * arg)
{
    uint32_t data = hDfeDduc->regs->cic_config;

    CSL_FINS(data, DFE_DDUC_CIC_CONFIG_REG_RATE, arg->rate);
    CSL_FINS(data, DFE_DDUC_CIC_CONFIG_REG_CIC_NDATA, arg->cic_ndata);
    CSL_FINS(data, DFE_DDUC_CIC_CONFIG_REG_FL_EN, arg->fl_en);
    CSL_FINS(data, DFE_DDUC_CIC_CONFIG_REG_CIC_RX_OUTPUT_SCALE, arg->cic_rx_output_scale);

    hDfeDduc->regs->cic_config = data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetCicConfig
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CIC_CONFIG_REG_CIC_NDATA
 *       DFE_DDUC_CIC_CONFIG_REG_CIC_RX_OUTPUT_SCALE
 *       DFE_DDUC_CIC_CONFIG_REG_FL_EN
 *       DFE_DDUC_CIC_CONFIG_REG_RATE
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetCicConfig(DfeFl_DducHandle hDfeDduc, DfeFl_DducCicCfg * arg)
{
    arg->rate = CSL_FEXT(hDfeDduc->regs->cic_config, DFE_DDUC_CIC_CONFIG_REG_RATE);
    arg->cic_ndata = CSL_FEXT(hDfeDduc->regs->cic_config, DFE_DDUC_CIC_CONFIG_REG_CIC_NDATA);
    arg->fl_en = CSL_FEXT(hDfeDduc->regs->cic_config, DFE_DDUC_CIC_CONFIG_REG_FL_EN);
    arg->cic_rx_output_scale = CSL_FEXT(hDfeDduc->regs->cic_config, DFE_DDUC_CIC_CONFIG_REG_CIC_RX_OUTPUT_SCALE);

}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetSelectorSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL2_REG_SELECTOR_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetSelectorSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel2, DFE_DDUC_SSEL2_REG_SELECTOR_SSEL, ssel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetMixSel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducSetMixSel(DfeFl_DducHandle hDfeDduc, DfeFl_DducMixSel * arg)
{
	uint32_t b;

	b = arg->iChan*4+arg->chan_or_rx*2;

	switch(arg->iMix)
	{
	case 0:
		CSL_FINSR(hDfeDduc->regs->selector_mix0_sel, b+1, b, arg->data);
		break;
	case 1:
		CSL_FINSR(hDfeDduc->regs->selector_mix1_sel, b+1, b, arg->data);
		break;
	case 2:
		CSL_FINSR(hDfeDduc->regs->selector_mix2_sel, b+1, b, arg->data);
		break;
	}
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetPfirCoefSsel
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_SSEL0_REG_PFIR_COEF_SSEL
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetPfirCoefSsel(DfeFl_DducHandle hDfeDduc, uint32_t ssel)
{
    CSL_FINS(hDfeDduc->regs->ssel0, DFE_DDUC_SSEL0_REG_PFIR_COEF_SSEL, ssel);
}

CSL_IDEF_INLINE 
/** ============================================================================
 *   @n@b dfeFl_DducSetPfirCoefOffset
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducSetPfirCoefOffset(DfeFl_DducHandle hDfeDduc, DfeFl_DducPfirCoefOffset *arg)
{
    volatile uint32_t *regs = &hDfeDduc->regs->pfir_coef_offset0;
    uint32_t r, b;

    r = arg->idx / 4;
    b = (arg->idx % 4) * 4;

    CSL_FINSR(regs[r], b+3, b, arg->data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetPfirFcMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducSetPfirFcMux(DfeFl_DducHandle hDfeDduc, DfeFl_DducPfirFcMux *arg)
{
    volatile uint32_t *regs = &hDfeDduc->regs->pfir_fcmux0;
    uint32_t r, b;

    r = arg->idx / 4;
    b = (arg->idx % 4) * 4;

    CSL_FINSR(regs[r], b+3, b, arg->data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetPfirPcSym
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducSetPfirPcSym(DfeFl_DducHandle hDfeDduc, DfeFl_DducPfirPcSym *arg)
{
    volatile uint32_t *regs = &hDfeDduc->regs->pfir_pcsym;
    uint32_t b;

    b = arg->idx;

    CSL_FINSR(regs[0], b, b, arg->data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetPfirCoeff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_PFIR_COEF_HI_REG_COEF_17_16
 *       DFE_DDUC_PFIR_COEF_LO_REG_COEF_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetPfirCoeff(DfeFl_DducHandle hDfeDduc, uint32_t idx, uint32_t data)
{
    hDfeDduc->regs->pfir_coef[idx].lo = CSL_FMK(DFE_DDUC_PFIR_COEF_LO_REG_COEF_15_0, data);
    hDfeDduc->regs->pfir_coef[idx].hi = CSL_FMK(DFE_DDUC_PFIR_COEF_HI_REG_COEF_17_16, data >> 16);

}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTestbusMux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TEST_BUS_MUX_ICG_DLY_REG_TEST_BUS_MUX
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTestbusMux(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINS(hDfeDduc->regs->test_bus_mux_icg_dly, DFE_DDUC_TEST_BUS_MUX_ICG_DLY_REG_TEST_BUS_MUX, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTestbusIcgdly
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TEST_BUS_MUX_ICG_DLY_REG_INIT_CLK_GATE_DELAY_4_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTestbusIcgdly(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINS(hDfeDduc->regs->test_bus_mux_icg_dly, DFE_DDUC_TEST_BUS_MUX_ICG_DLY_REG_INIT_CLK_GATE_DELAY_4_0, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTestbusBbmux
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TEST_BUS_BB_MUX_REG_TEST_BUS_BB_MUX
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTestbusBbmux(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINS(hDfeDduc->regs->test_bus_bb_mux, DFE_DDUC_TEST_BUS_BB_MUX_REG_TEST_BUS_BB_MUX, data);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTxSiggenMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_FRAME_LEN_M1
 *       DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_GEN_DATA
 *       DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_RAMP_MODE
 *       DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_GEN_FRAME
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTxSiggenMode(DfeFl_DducHandle hDfeDduc, DfeFl_DducSiggenMode *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->tx_signal_gen_config;

    regs[0] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_FRAME_LEN_M1, arg->frameLenM1)
            | CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_RAMP_MODE, arg->mode)
            | CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_GEN_FRAME, arg->frame)
            | CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_CONFIG_REG_GEN_DATA, arg->enable);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTxSiggenRamp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_START_LO_REG_RAMP_START_15_0
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_START_HI_REG_RAMP_START_31_16
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_STOP_LO_REG_RAMP_STOP_15_0
 *       DFE_DDUC_TX_SIGNAL_GEN_PULSE_WIDTH_REG_PULSE_WIDTH
 *       DFE_DDUC_TX_SIGNAL_GEN_RAMP_STOP_HI_REG_RAMP_STOP_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTxSiggenRamp(DfeFl_DducHandle hDfeDduc, DfeFl_DducSiggenRampConfig *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->tx_signal_gen_ramp_start_lo;

    // ramp start
    regs[0] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_START_LO_REG_RAMP_START_15_0, arg->rampStart);
    regs[1] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_START_HI_REG_RAMP_START_31_16, arg->rampStart >> 16);

    // ramp stop
    regs[2] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_STOP_LO_REG_RAMP_STOP_15_0, arg->rampStop);
    regs[3] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_STOP_HI_REG_RAMP_STOP_31_16, arg->rampStop >> 16);

    // ramp slope
    regs[4] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0, arg->rampSlope);
    regs[5] = CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16, arg->rampSlope >> 16);

    // pulse width
    regs[6]= CSL_FMK(DFE_DDUC_TX_SIGNAL_GEN_PULSE_WIDTH_REG_PULSE_WIDTH, arg->pulse_width);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgTxChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TX_CHECK_SUM_SIGNAL_LEN_REG_SIGNAL_LEN
 *       DFE_DDUC_TX_CHECK_SUM_CONFIG_REG_MODE
 *       DFE_DDUC_TX_CHECK_SUM_CHAN_SEL_REG_CHAN_SEL
 *       DFE_DDUC_TX_CHECK_SUM_CONFIG_REG_STABLE_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgTxChksum(DfeFl_DducHandle hDfeDduc, DfeFl_DducChksumConfig *arg)
{
	hDfeDduc->regs->tx_check_sum_config = CSL_FMK(DFE_DDUC_TX_CHECK_SUM_CONFIG_REG_MODE, arg->chksumMode)
                                        | CSL_FMK(DFE_DDUC_TX_CHECK_SUM_CONFIG_REG_STABLE_LEN, arg->latencyMode.stableLen);
	hDfeDduc->regs->tx_check_sum_signal_len = CSL_FMK(DFE_DDUC_TX_CHECK_SUM_SIGNAL_LEN_REG_SIGNAL_LEN, arg->latencyMode.signalLen);
	hDfeDduc->regs->tx_check_sum_chan_sel = CSL_FMK(DFE_DDUC_TX_CHECK_SUM_CHAN_SEL_REG_CHAN_SEL, arg->latencyMode.chanSel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducQueryTxChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_TX_CHECK_SUM_RESULT_HI_REG_RESULT_31_16
 *       DFE_DDUC_TX_CHECK_SUM_RESULT_LO_REG_RESULT_15_0
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducQueryTxChksumResult(DfeFl_DducHandle hDfeDduc, uint32_t *arg)
{
    uint32_t data;

    data = CSL_FEXT(hDfeDduc->regs->tx_check_sum_result_lo, DFE_DDUC_TX_CHECK_SUM_RESULT_LO_REG_RESULT_15_0);
    data |= CSL_FEXT(hDfeDduc->regs->tx_check_sum_result_hi, DFE_DDUC_TX_CHECK_SUM_RESULT_HI_REG_RESULT_31_16) << 16;

	*arg = data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgRxSiggenMode
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_GEN_DATA
 *       DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_RAMP_MODE
 *       DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_FRAME_LEN_M1
 *       DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_GEN_FRAME
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgRxSiggenMode(DfeFl_DducHandle hDfeDduc, DfeFl_DducSiggenMode *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->rx_signal_gen_config;

    regs[0] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_FRAME_LEN_M1, arg->frameLenM1)
            | CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_RAMP_MODE, arg->mode)
            | CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_GEN_FRAME, arg->frame)
            | CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_CONFIG_REG_GEN_DATA, arg->enable);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgRxSiggenRamp
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_RX_SIGNAL_GEN_PULSE_WIDTH_REG_PULSE_WIDTH
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_STOP_LO_REG_RAMP_STOP_15_0
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_START_HI_REG_RAMP_START_31_16
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_START_LO_REG_RAMP_START_15_0
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_STOP_HI_REG_RAMP_STOP_31_16
 *       DFE_DDUC_RX_SIGNAL_GEN_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgRxSiggenRamp(DfeFl_DducHandle hDfeDduc, DfeFl_DducSiggenRampConfig *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->rx_signal_gen_ramp_start_lo;

    // ramp start
    regs[0] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_START_LO_REG_RAMP_START_15_0, arg->rampStart);
    regs[1] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_START_HI_REG_RAMP_START_31_16, arg->rampStart >> 16);

    // ramp stop
    regs[2] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_STOP_LO_REG_RAMP_STOP_15_0, arg->rampStop);
    regs[3] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_STOP_HI_REG_RAMP_STOP_31_16, arg->rampStop >> 16);

    // ramp slope
    regs[4] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_SLOPE_LO_REG_RAMP_SLOPE_15_0, arg->rampSlope);
    regs[5] = CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_RAMP_SLOPE_HI_REG_RAMP_SLOPE_31_16, arg->rampSlope >> 16);

    // pulse width
    regs[6]= CSL_FMK(DFE_DDUC_RX_SIGNAL_GEN_PULSE_WIDTH_REG_PULSE_WIDTH, arg->pulse_width);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducCfgRxChksum
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_RX_CHECK_SUM_CHAN_SEL_REG_CHAN_SEL
 *       DFE_DDUC_RX_CHECK_SUM_CONFIG_REG_MODE
 *       DFE_DDUC_RX_CHECK_SUM_CONFIG_REG_STABLE_LEN
 *       DFE_DDUC_RX_CHECK_SUM_SIGNAL_LEN_REG_SIGNAL_LEN
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducCfgRxChksum(DfeFl_DducHandle hDfeDduc, DfeFl_DducChksumConfig *arg)
{
	hDfeDduc->regs->rx_check_sum_config = CSL_FMK(DFE_DDUC_RX_CHECK_SUM_CONFIG_REG_MODE, arg->chksumMode)
                                        | CSL_FMK(DFE_DDUC_RX_CHECK_SUM_CONFIG_REG_STABLE_LEN, arg->latencyMode.stableLen);
	hDfeDduc->regs->rx_check_sum_signal_len = CSL_FMK(DFE_DDUC_RX_CHECK_SUM_SIGNAL_LEN_REG_SIGNAL_LEN, arg->latencyMode.signalLen);
	hDfeDduc->regs->rx_check_sum_chan_sel = CSL_FMK(DFE_DDUC_RX_CHECK_SUM_CHAN_SEL_REG_CHAN_SEL, arg->latencyMode.chanSel);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducQueryRxChksumResult
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_RX_CHECK_SUM_RESULT_LO_REG_RESULT_15_0
 *       DFE_DDUC_RX_CHECK_SUM_RESULT_HI_REG_RESULT_31_16
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducQueryRxChksumResult(DfeFl_DducHandle hDfeDduc, uint32_t *arg)
{
    uint32_t data;

    data = CSL_FEXT(hDfeDduc->regs->rx_check_sum_result_lo, DFE_DDUC_RX_CHECK_SUM_RESULT_LO_REG_RESULT_15_0);
    data |= CSL_FEXT(hDfeDduc->regs->rx_check_sum_result_hi, DFE_DDUC_RX_CHECK_SUM_RESULT_HI_REG_RESULT_31_16) << 16;

	*arg = data;
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducEnableCicovIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducEnableCicovIntr(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINSR(hDfeDduc->regs->intr_mask_mask, data, data, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducEnableHopRolloverIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_MASK_REG_HOP_ROLLOVER_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducEnableHopRolloverIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_mask, DFE_DDUC_INTR_MASK_MASK_REG_HOP_ROLLOVER_INTR_MASK, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducEnableHopHalfwayIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_MASK_REG_HOP_HALFWAY_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducEnableHopHalfwayIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_mask, DFE_DDUC_INTR_MASK_MASK_REG_HOP_HALFWAY_INTR_MASK, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducDisableCicovIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducDisableCicovIntr(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINSR(hDfeDduc->regs->intr_mask_mask, data, data, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducDisableHopRolloverIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_MASK_REG_HOP_ROLLOVER_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducDisableHopRolloverIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_mask, DFE_DDUC_INTR_MASK_MASK_REG_HOP_ROLLOVER_INTR_MASK, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducDisableHopHalfwayIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_MASK_REG_HOP_HALFWAY_INTR_MASK
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducDisableHopHalfwayIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_mask, DFE_DDUC_INTR_MASK_MASK_REG_HOP_HALFWAY_INTR_MASK, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetForceCicovIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducSetForceCicovIntr(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINSR(hDfeDduc->regs->intr_mask_force, data, data, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetForceHopRolloverIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_FORCE_REG_HOP_ROLLOVER_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetForceHopRolloverIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_force, DFE_DDUC_INTR_MASK_FORCE_REG_HOP_ROLLOVER_INTR_FORCE, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetForceHopHalfwayIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_FORCE_REG_HOP_HALFWAY_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetForceHopHalfwayIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_force, DFE_DDUC_INTR_MASK_FORCE_REG_HOP_HALFWAY_INTR_FORCE, 1);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearForceCicovIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducClearForceCicovIntr(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    CSL_FINSR(hDfeDduc->regs->intr_mask_force, data, data, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearForceHopRolloverIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_FORCE_REG_HOP_ROLLOVER_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducClearForceHopRolloverIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_force, DFE_DDUC_INTR_MASK_FORCE_REG_HOP_ROLLOVER_INTR_FORCE, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearForceHopHalfwayIntr
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_FORCE_REG_HOP_HALFWAY_INTR_FORCE
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducClearForceHopHalfwayIntr(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_force, DFE_DDUC_INTR_MASK_FORCE_REG_HOP_HALFWAY_INTR_FORCE, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearCicovIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducClearCicovIntrStatus(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    uint32_t b = (1u << data);
    hDfeDduc->regs->intr_mask_status = ~b;
    //CSL_FINSR(hDfeDduc->regs->intr_mask_status, data, data, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearHopRolloverIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_STATUS_REG_HOP_ROLLOVER_INTR_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducClearHopRolloverIntrStatus(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_status, DFE_DDUC_INTR_MASK_STATUS_REG_HOP_ROLLOVER_INTR_STATUS, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducClearHopHalfwayIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
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
 *       DFE_DDUC_INTR_MASK_STATUS_REG_HOP_HALFWAY_INTR_STATUS
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducClearHopHalfwayIntrStatus(DfeFl_DducHandle hDfeDduc)
{
    CSL_FINS(hDfeDduc->regs->intr_mask_status, DFE_DDUC_INTR_MASK_STATUS_REG_HOP_HALFWAY_INTR_STATUS, 0);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetCicovIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
void dfeFl_DducGetCicovIntrStatus(DfeFl_DducHandle hDfeDduc, uint32_t intr, uint32_t *status)
{
    uint32_t data = hDfeDduc->regs->intr_mask_status;

    *status = CSL_FEXTR(data, intr, intr);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetHopRolloverIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_INTR_MASK_STATUS_REG_HOP_ROLLOVER_INTR_STATUS
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetHopRolloverIntrStatus(DfeFl_DducHandle hDfeDduc, uint32_t *status)
{
    uint32_t data = hDfeDduc->regs->intr_mask_status;

    *status = CSL_FEXT(data, DFE_DDUC_INTR_MASK_STATUS_REG_HOP_ROLLOVER_INTR_STATUS);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetHopHalfwayIntrStatus
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_INTR_MASK_STATUS_REG_HOP_HALFWAY_INTR_STATUS
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetHopHalfwayIntrStatus(DfeFl_DducHandle hDfeDduc, uint32_t *status)
{
    uint32_t data = hDfeDduc->regs->intr_mask_status;

    *status = CSL_FEXT(data, DFE_DDUC_INTR_MASK_STATUS_REG_HOP_HALFWAY_INTR_STATUS);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetTimestep
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CLK_GATER_TIME_STEP_LSB_REG_TIME_STEP_15_0
 *       DFE_DDUC_CLK_GATER_TIME_STEP_MSB_REG_TIME_STEP_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetTimestep(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->clk_gater_time_step_lsb;

    regs[0] = CSL_FMK(DFE_DDUC_CLK_GATER_TIME_STEP_LSB_REG_TIME_STEP_15_0, data);
    regs[1] = CSL_FMK(DFE_DDUC_CLK_GATER_TIME_STEP_MSB_REG_TIME_STEP_31_16, data >> 16);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetResetint
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CLK_GATER_RESET_INT_LSB_REG_RESET_INT_15_0
 *       DFE_DDUC_CLK_GATER_RESET_INT_MSB_REG_RESET_INT_31_16
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetResetint(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->clk_gater_reset_int_lsb;

    regs[0] = CSL_FMK(DFE_DDUC_CLK_GATER_RESET_INT_LSB_REG_RESET_INT_15_0, data);
    regs[1] = CSL_FMK(DFE_DDUC_CLK_GATER_RESET_INT_MSB_REG_RESET_INT_31_16, data >> 16);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetTddPeriod
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CLK_GATER_TDD_PERIOD_MSB_REG_TDD_PERIOD_31_16
 *       DFE_DDUC_CLK_GATER_TDD_PERIOD_LSB_REG_TDD_PERIOD_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetTddPeriod(DfeFl_DducHandle hDfeDduc, uint32_t data)
{
//    volatile uint32_t *regs;
//
//    regs = &hDfeDduc->regs->clk_gater_tdd_period_lsb;

    hDfeDduc->regs->clk_gater_tdd_period_lsb = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_PERIOD_LSB_REG_TDD_PERIOD_15_0, data);

//    rets = &hDfeDduc->regs->clk_gater_tdd_period_msb;
    hDfeDduc->regs->clk_gater_tdd_period_msb = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_PERIOD_MSB_REG_TDD_PERIOD_23_16, data >> 16);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducSetTddOnoff
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_CLK_GATER_TDD_OFF_0_MSB_REG_TDD_OFF_0_31_16
 *       DFE_DDUC_CLK_GATER_TDD_OFF_0_LSB_REG_TDD_OFF_0_15_0
 *       DFE_DDUC_CLK_GATER_TDD_OFF_1_LSB_REG_TDD_OFF_1_15_0
 *       DFE_DDUC_CLK_GATER_TDD_ON_0_MSB_REG_TDD_ON_0_31_16
 *       DFE_DDUC_CLK_GATER_TDD_ON_1_MSB_REG_TDD_ON_1_31_16
 *       DFE_DDUC_CLK_GATER_TDD_ON_0_LSB_REG_TDD_ON_0_15_0
 *       DFE_DDUC_CLK_GATER_TDD_OFF_1_MSB_REG_TDD_OFF_1_31_16
 *       DFE_DDUC_CLK_GATER_TDD_ON_1_LSB_REG_TDD_ON_1_15_0
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducSetTddOnoff(DfeFl_DducHandle hDfeDduc, DfeFl_DducTddOnOff *arg)
{
    volatile uint32_t *regs;

    regs = &hDfeDduc->regs->clk_gater_tdd_on_0_lsb;

    regs[0] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_ON_0_LSB_REG_TDD_ON_0_15_0, arg->tdd_on_0);
    regs[1] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_ON_0_MSB_REG_TDD_ON_0_23_16, arg->tdd_on_0 >> 16);

    regs[2] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_OFF_0_LSB_REG_TDD_OFF_0_15_0, arg->tdd_off_0);
    regs[3] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_OFF_0_MSB_REG_TDD_OFF_0_23_16, arg->tdd_off_0 >> 16);

    regs[4] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_ON_1_LSB_REG_TDD_ON_1_15_0, arg->tdd_on_1);
    regs[5] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_ON_1_MSB_REG_TDD_ON_1_23_16, arg->tdd_on_1 >> 16);

    regs[6] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_OFF_1_LSB_REG_TDD_OFF_1_15_0, arg->tdd_off_1);
    regs[7] = CSL_FMK(DFE_DDUC_CLK_GATER_TDD_OFF_1_MSB_REG_TDD_OFF_1_23_16, arg->tdd_off_1 >> 16);
}

CSL_IDEF_INLINE
/** ============================================================================
 *   @n@b dfeFl_DducGetDirection
 *
 *   @b Description
 *   @n [add content]
 *
 *   @b Arguments
 *   @verbatim
         hDfeDduc    [add content]
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
 *       DFE_DDUC_INITS_CLKS_REG_TX_SEL
 *
 *   @b Writes
 *   @n  None
 *
 *   @b Example
 *   @verbatim
         [add content]
     @endverbatim
 * ===========================================================================
 */
void dfeFl_DducGetDirection(DfeFl_DducHandle hDfeDduc, uint32_t *arg)
{
    *arg = CSL_FEXT(hDfeDduc->regs->inits_clks, DFE_DDUC_INITS_CLKS_REG_TX_SEL);
}

#ifdef __cplusplus
}
#endif

#endif /* _DFE_FL_DDUCAUX_H_ */
