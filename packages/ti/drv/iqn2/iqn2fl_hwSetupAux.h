/*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2012-2013
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

/** ============================================================================
 *   @file  iqn2fl_hwSetupAux.h
 *
 *   @brief  API Auxilary header file for IQN2 HW setup.
 *
 */

#ifndef _IQN2FLHWSETUPAUX_H_
#define _IQN2FLHWSETUPAUX_H_
 
#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>

#ifdef __cplusplus
extern "C" {
#endif
 
/**
 *  Hardware Setup Functions of IQN2
 */

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEfeCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 AIL Egress Framing Engine (EFE) Config Group setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEfeCfgSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_SI_IQ_EFE_CONFIG_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEfeCfgGrpRegs (hIqn2, ailIndex, &ail_si_iq_efe_config_group);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEfeCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilSiIqEfeCfgGrp *pAilEfeCfgSetup
)
{
    uint32_t   i, tempReg;

    /* Setup EFE CHANNEL CONFIGURATION REGISTER */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_EN, 
                      pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].chan_en) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_OBSAI_CTL, 
                      pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].chan_obsai_ctl) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_ENET_CTL, 
                      pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].chan_enet_ctl) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_AXC_FINE_OFFSET, 
                      pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].axc_fine_offset) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                        pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].chan_tdd_frc_off) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_RADIO_SEL, 
                        pAilEfeCfgSetup->ail_si_iq_efe_chan_config[i].chan_radio_sel);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CHAN_CFG[i] = tempReg;
    }
    
    /* Setup AIL SI IQ EFE CONFIGURATION REGISTER */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CFG, 
        IQN_AIL_AIL_IQ_EFE_CFG_LOOPBACK_EN, 
        pAilEfeCfgSetup->lpbk_en);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEfeRadStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 AIL Egress Framing Engine (EFE) Radio Standard Group setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEfeRadStdSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEfeRadStdGrpRegs (hIqn2, ailIndex, &ail_si_iq_efe_radio_std_group);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEfeRadStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilSiIqEfeRadStdGrp *pAilEfeRadStdSetup
)
{
    uint32_t   tempReg, i;

    /* Setup EFE FRAME COUNT REGISTER */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_EFE_FRM_TC_CFG_SYM_TC, 
                      pAilEfeRadStdSetup->ail_iq_efe_frm_tc_cfg[i].sym_tc) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_FRM_TC_CFG_INDEX_SC, 
                      pAilEfeRadStdSetup->ail_iq_efe_frm_tc_cfg[i].index_sc) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_FRM_TC_CFG_INDEX_TC, 
                      pAilEfeRadStdSetup->ail_iq_efe_frm_tc_cfg[i].index_tc);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup EFE RADIO STANDARD CONFIGURATION REGISTER */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_EFE_RAD_STD_CFG_TDD_FIRST_SYM, 
                      pAilEfeRadStdSetup->ail_iq_efe_rad_std_cfg[i].tdd_first_sym) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_RAD_STD_CFG_TDD_LUT_EN, 
                      pAilEfeRadStdSetup->ail_iq_efe_rad_std_cfg[i].tdd_lut_en) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_RAD_STD_CFG_GSM_AXC_BBHOP_MODE, 
                      pAilEfeRadStdSetup->ail_iq_efe_rad_std_cfg[i].gsm_axc_bbhop_mode) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EFE_RAD_STD_CFG_GSM_CMP_MODE, 
                      pAilEfeRadStdSetup->ail_iq_efe_rad_std_cfg[i].gsm_cmp_mode);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_RAD_STD_CFG[i] = tempReg;
    }

    /* Setup EFE RADIO STANDARD 0-7 TDD ENABLE LUT */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG0[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG0_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg0[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG1[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG1_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg1[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG2[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG2_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg2[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG3[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG3_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg3[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG4[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG4_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg4[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG5[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG5_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg5[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG6[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG6_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg6[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_TDD_EN_CFG7[i], 
            IQN_AIL_AIL_IQ_EFE_TDD_EN_CFG7_TDD_EN, 
            pAilEfeRadStdSetup->ail_iq_efe_tdd_en_cfg7[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEgrSchCpriRegs
 *
 *   @b Description
 *   @n IQN2 AIL Egress SCH CPRI setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEgrSchCpriSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_SI_IQ_E_SCH_CPRI
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEgrSchCpriRegs (hIqn2, ailIndex, &ail_si_iq_e_sch_cpri);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEgrSchCpriRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilSiIqEgrSchCpri *pAilEgrSchCpriSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL PE CPRI CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_CFG, 
        IQN_AIL_AIL_IQ_PE_CPRI_CFG_AXC_CONT_TC, 
        pAilEgrSchCpriSetup->axc_cont_tc);

    /* Setup PE CPRI BUBBLE FSM CONFIGURATION REGISTER PART1 */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_BUB_FSM_CFG[i], 
            IQN_AIL_AIL_IQ_PE_CPRI_BUB_FSM_CFG_KNC, 
            pAilEgrSchCpriSetup->ail_iq_pe_cpri_bub_fsm_cfg.knc[i]);
    }

    /* Setup PE CPRI BUBBLE FSM CONFIGURATION REGISTER PART2 */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_BUB_FSM2_CFG_GAP_FRAC, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_bub_fsm_cfg.gap_frac[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_BUB_FSM2_CFG_GAP_INT, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_bub_fsm_cfg.gap_int[i]);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_BUB_FSM2_CFG[i] = tempReg;
    }

    /* Setup PE CPRI TDM FSM CONFIGURATION REGISTER */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_TDM_FSM_CFG_NCONT, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_tdm_fsm_cfg.ncont[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_TDM_FSM_CFG_LUTSTRT, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_tdm_fsm_cfg.lutstrt[i]);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_TDM_FSM_CFG[i] = tempReg;
    }

    /* Setup PE CPRI RADIO STANDARD CONFIGURATION REGISTER PART 0-2 */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD_CFG[i], 
            IQN_AIL_AIL_IQ_PE_CPRI_RADSTD_CFG_EN, 
            pAilEgrSchCpriSetup->ail_iq_pe_cpri_radstd_cfg.radstd_cfg_en[i]);

        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_RADSTD1_CFG_BFRM_OFFSET, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_bfrm_offset[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_RADSTD1_CFG_HFRM_OFFSET, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_radstd_cfg.radstd1_cfg_hfrm_offset[i]);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD1_CFG[i] = tempReg;

        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD2_CFG[i], 
            IQN_AIL_AIL_IQ_PE_CPRI_RADSTD2_CFG_BFRM_NUM, 
            pAilEgrSchCpriSetup->ail_iq_pe_cpri_radstd_cfg.radstd2_cfg_bfrm_num[i]);
    }

    /* Setup PE CPRI CONTAINER LOOK UP TABLE REGISTER */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_CONT_CFG_LUT_GRP, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_cont_cfg.lut_grp[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_CONT_CFG_LUT_EN, 
                      pAilEgrSchCpriSetup->ail_iq_pe_cpri_cont_cfg.lut_en[i]);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_CONT_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEctlPktIfRegs
 *
 *   @b Description
 *   @n IQN2 AIL ECTL PKT IF setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEctlPktIfSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_ECTL_CHAN_CFG, AIL_ECTL_DB_THOLD_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEctlPktIfRegs (hIqn2, ailIndex, &ail_ectl_pkt_if);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEctlPktIfRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilEctlPktIf *pAilEctlPktIfSetup
)
{
    uint32_t   i;

    for (i = 0; i < 4; i++)
    {
        /* Setup AIL ECTL CHANNEL ENABLE CONFIGURATION REGISTER */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_ECTL_PKT_IF.AIL_ECTL_CHAN_CFG[i], 
            IQN_AIL_AIL_ECTL_CHAN_CFG_CHAN_EN, 
            pAilEctlPktIfSetup[i].chan_en);

        /* Setup AIL ECTL DB THRESHOLD REGISTER */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_ECTL_PKT_IF.AIL_ECTL_DB_THOLD_CFG[i], 
            IQN_AIL_AIL_ECTL_DB_THOLD_CFG_CHANNEL, 
            pAilEctlPktIfSetup[i].channel);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEctlRegGrp
 *
 *   @b Description
 *   @n IQN2 AIL ECTL Register setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEctlRegGrpSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_ECTL_RATE_CTL_CFG, AIL_ECTL_CH_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEctlRegGrp (hIqn2, ailIndex, &ail_ectl_reg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEctlRegGrp(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilEctlRegGrp *pAilEctlRegGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL ECTL RATE CONTROL CONFIGURATION REGISTER
     * Default value is 15. Setting this value to a non-default value is possible if 0 < rate <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_ECTL_RATE_CTL_CFG_REG CMD word
     */
    if (pAilEctlRegGrpSetup->rate) {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_ECTL_REGISTER_GROUP.AIL_ECTL_RATE_CTL_CFG,
            IQN_AIL_AIL_ECTL_RATE_CTL_CFG_RATE,
            pAilEctlRegGrpSetup->rate);
    }

    /* Setup AIL ECTL CHANNEL CONFIGURATION REGISTER */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_ECTL_CH_CFG_DAT_SWAP, 
                      pAilEctlRegGrpSetup->dat_swap[i]) |
                  CSL_FMK(IQN_AIL_AIL_ECTL_CH_CFG_IQ_ORDER, 
                      pAilEctlRegGrpSetup->iq_order[i]);
        hIqn2->regs->Ail[ailIndex].AIL_ECTL_REGISTER_GROUP.AIL_ECTL_CH_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEdcRegGrp
 *
 *   @b Description
 *   @n IQN2 AIL EDC (Egress DMA Controller) Register Group setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilEdcRegGrpSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_IQ_EDC_CFG, AIL_IQ_EDC_CH_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEdcRegGrp (hIqn2, ailIndex, &ail_iq_edc_reg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEdcRegGrp(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilEdcRegGrp *pAilEdcRegGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL EDC CONFIGURATION REGISTER */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_CFG, 
        IQN_AIL_AIL_IQ_EDC_CFG_PSI_ERR_CHK_DISABLE, 
        pAilEdcRegGrpSetup->psi_err_chk_disable);

    /* Setup AIL EDC CHANNEL CONFIGURATION REGISTERS */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_EDC_CH_CFG_DAT_SWAP, 
                      pAilEdcRegGrpSetup->dat_swap[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_EDC_CH_CFG_IQ_ORDER, 
                      pAilEdcRegGrpSetup->iq_order[i]);
        hIqn2->regs->Ail[ailIndex].AIL_IQ_EDC_REGISTER_GROUP.AIL_IQ_EDC_CH_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEgressRegs
 *
 *   @b Description
 *   @n IQN2 AIL Egress setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAilEfeCfgGrpRegs, Iqn2Fl_setupAilEfeRadStdGrpRegs
 *      Iqn2Fl_setupAilEgrSchCpriRegs
 *
 *   @b Writes
 *   @n AIL_SI_IQ_EFE_CONFIG_GROUP, AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP, 
 *      AIL_IQ_EFE_CHAN_AXC_OFFSET, AIL_IQ_EFE_FRM_SAMP_TC_MMR_RAM, 
 *      AIL_SI_IQ_E_TDM_LUT_RAM, AIL_SI_IQ_E_SCH_PHY, AIL_SI_IQ_E_OBSAI_MODTXRULE, 
 *      AIL_SI_IQ_E_OBSAI_DBM_RULE_RAM, AIL_SI_IQ_E_OBSAI_DBM_BITMAP_RAM, 
 *      AIL_SI_IQ_E_SCH_CPRI
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilEgressRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilEgressRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /** pointer to AIL Egress setup */
    Iqn2Fl_AilEgrSetup   *pAilEgrConfig;
    Iqn2Fl_AilInstance   ailIndex;
    uint32_t   tempReg, i;

    ailIndex       = ailSetup->ailInstNum;
    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilEgrConfig  = ailSetup->pAilEgrSetup;

    /* Setup AIL SI IQ EFE CONFIG GROUP */
    Iqn2Fl_setupAilEfeCfgGrpRegs (hIqn2, ailIndex, &(pAilEgrConfig->ail_si_iq_efe_config_group));

    /* Setup AIL SI IQ EFE RADIO STANDARD GROUP */
    Iqn2Fl_setupAilEfeRadStdGrpRegs (hIqn2, ailIndex, &(pAilEgrConfig->ail_si_iq_efe_radio_std_group));

    /* AIL IQ EFE CHAN AXC OFFSET */
    for (i = 0; i < 64; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_EFE_CHAN_AXC_OFFSET[i].AIL_IQ_EFE_CHAN_AXC_OFFSET_CFG, 
            IQN_AIL_AIL_IQ_EFE_CHAN_AXC_OFFSET_CFG_AXC_OFFSET, 
            pAilEgrConfig->ail_iq_efe_chan_axc_offset[i]);
    }

    /* AIL IQ EFE FRM SAMP TC MMR RAM */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_EFE_FRM_SAMP_TC_MMR_RAM[i].AIL_IQ_EFE_FRM_SAMP_TC_CFG, 
            IQN_AIL_AIL_IQ_EFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pAilEgrConfig->ail_iq_efe_frm_samp_tc_mmr_ram[i]);
    }

    /* AIL SI IQ E TDM LUT RAM */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_TDM_LUT_CFG_AXC, 
                      pAilEgrConfig->ail_iq_pe_axc_tdm_lut_cfg[i].axc) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_TDM_LUT_CFG_EN, 
                      pAilEgrConfig->ail_iq_pe_axc_tdm_lut_cfg[i].en);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_TDM_LUT_RAM[i].AIL_IQ_PE_TDM_LUT_CFG = tempReg;
    }

    /* AIL SI IQ E SCH PHY - PE DMA Channel 1 Register */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_SCH_PHY.AIL_IQ_PE_PHY_CFG, 
        IQN_AIL_AIL_IQ_PE_PHY_CFG_PHY_EN, 
        pAilEgrConfig->phy_en);

    /* AIL SI IQ E OBSAI MODTXRULE */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_MODTXRULE_CFG_RULE_MOD, 
                      pAilEgrConfig->ail_iq_pe_obsai_modtxrule_cfg[i].rule_mod) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_MODTXRULE_CFG_RULE_EN, 
                      pAilEgrConfig->ail_iq_pe_obsai_modtxrule_cfg[i].rule_en) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_MODTXRULE_CFG_RULE_CTL_MSG, 
                      pAilEgrConfig->ail_iq_pe_obsai_modtxrule_cfg[i].rule_ctl_msg) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_MODTXRULE_CFG_RULE_INDEX, 
                      pAilEgrConfig->ail_iq_pe_obsai_modtxrule_cfg[i].rule_index);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_OBSAI_MODTXRULE.AIL_IQ_PE_OBSAI_MODTXRULE_CFG[i] = tempReg;
    }

    /* AIL SI IQ E OBSAI DBM RULE RAM */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_0_CFG_DBM_EN, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_en) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_0_CFG_DBM_RADSTD, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_radstd) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_0_CFG_DBM_X, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_x) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_0_CFG_DBM_1MULT, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_1mult);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_OBSAI_DBM_RULE_RAM[i].AIL_IQ_PE_OBSAI_DBM_0_CFG = tempReg;

        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_1_CFG_DBM_1SIZE, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_1size) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_1_CFG_DBM_2SIZE, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_2size) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_1_CFG_DBM_LUTSTRT, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_lutstrt) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_OBSAI_DBM_1_CFG_DBM_XBUBBLE, 
                      pAilEgrConfig->ail_si_iq_e_obsai_dbm_rule[i].dbm_xbubble);
        hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_OBSAI_DBM_RULE_RAM[i].AIL_IQ_PE_OBSAI_DBM_1_CFG = tempReg;
    }

    /* AIL SI IQ E OBSAI DBM BITMAP RAM - PE DBMF Bit Map 1 & 2 Register */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_SI_IQ_E_OBSAI_DBM_BITMAP_RAM[i].AIL_IQ_PE_OBSAI_DBM_CFG, 
            IQN_AIL_AIL_IQ_PE_OBSAI_DBM_CFG_DBM_BIT_MAP, 
            pAilEgrConfig->ail_si_iq_e_obsai_dbm_bitmap_ram[i]);
    }

    /* Setup AIL SI IQ E SCH CPRI */
    Iqn2Fl_setupAilEgrSchCpriRegs (hIqn2, ailIndex, &(pAilEgrConfig->ail_si_iq_e_sch_cpri));

    /* Setup AIL ECTL PKT IF REGISTERS */
    Iqn2Fl_setupAilEctlPktIfRegs (hIqn2, ailIndex, &(pAilEgrConfig->ail_ectl_pkt_if[0]));

    /* Setup AIL ECTL REGISTER GROUP */
    Iqn2Fl_setupAilEctlRegGrp (hIqn2, ailIndex, &(pAilEgrConfig->ail_ectl_reg_grp));

    /* Setup AIL IQ EDC REGISTER GROUP */
    Iqn2Fl_setupAilEdcRegGrp (hIqn2, ailIndex, &(pAilEgrConfig->ail_iq_edc_reg_grp));

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilIfeRadStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 AIL Ingress Framing Engine (IFE) Radio Standard Group setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilIfeRadStdSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_IQ_IFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilIfeRadStdGrpRegs (hIqn2, ailIndex, &ail_iq_ife_radio_std_group);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilIfeRadStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilIqIfeRadStdGrp *pAilIfeRadStdSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IFE FRAME COUNT REGISTER */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_IFE_FRM_TC_CFG_SYM_TC, 
                      pAilIfeRadStdSetup->ail_iq_ife_frm_tc_cfg[i].sym_tc) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_FRM_TC_CFG_INDEX_SC, 
                      pAilIfeRadStdSetup->ail_iq_ife_frm_tc_cfg[i].index_sc) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_FRM_TC_CFG_INDEX_TC, 
                      pAilIfeRadStdSetup->ail_iq_ife_frm_tc_cfg[i].index_tc);
        hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup IFE RADIO STANDARD CONFIGURATION REGISTER */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_RAD_STD_CFG[i], 
            IQN_AIL_AIL_IQ_IFE_RAD_STD_CFG_TDD_LUT_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_rad_std_cfg[i].tdd_lut_en);
    }

    /* Setup IFE RADIO STANDARD 0-7 TDD ENABLE LUT */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG0[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG0_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg0[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG1[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG1_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg1[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG2[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG2_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg2[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG3[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG3_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg3[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG4[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG4_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg4[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG5[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG5_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg5[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG6[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG6_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg6[i]);
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_RADIO_STANDARD_GROUP.AIL_IQ_IFE_TDD_EN_CFG7[i], 
            IQN_AIL_AIL_IQ_IFE_TDD_EN_CFG7_TDD_EN, 
            pAilIfeRadStdSetup->ail_iq_ife_tdd_en_cfg7[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilIctlIdcIfRegs
 *
 *   @b Description
 *   @n IQN2 AIL ICTL IDC IF setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilIctlIdcIfSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_ICTL_PKT_IF, AIL_ICTL_CH_CFG, AIL_ICTL_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilIctlIdcIfRegs (hIqn2, ailIndex, &ail_ictl_idc_if);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilIctlIdcIfRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilIctlIdcIf *pAilIctlIdcIfSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL ICTL CHANNEL CONFIGURATION REGISTERS */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_ICTL_CH_CFG_DAT_SWAP, 
                      pAilIctlIdcIfSetup->ictl_chan_cfg[i].dat_swap) |
                  CSL_FMK(IQN_AIL_AIL_ICTL_CH_CFG_IQ_ORDER, 
                      pAilIctlIdcIfSetup->ictl_chan_cfg[i].iq_order) |
                  CSL_FMK(IQN_AIL_AIL_ICTL_CH_CFG_PKT_TYPE, 
                      pAilIctlIdcIfSetup->ictl_chan_cfg[i].pkt_type) |
                  CSL_FMK(IQN_AIL_AIL_ICTL_CH_CFG_CHAN_FRC_OFF, 
                      pAilIctlIdcIfSetup->ictl_chan_cfg[i].chan_frc_off);
        hIqn2->regs->Ail[ailIndex].AIL_ICTL_IDC_IF.AIL_ICTL_CH_CFG[i] = tempReg;
    }

    /* Setup AIL IQ IDC CONFIGURATION GROUP */
    tempReg = CSL_FMK(IQN_AIL_AIL_ICTL_CFG_FAIL_MARK_ONLY, 
                  pAilIctlIdcIfSetup->ictl_cfg.fail_mark_only) |
              CSL_FMK(IQN_AIL_AIL_ICTL_CFG_FRC_OFF_ALL, 
                  pAilIctlIdcIfSetup->ictl_cfg.frc_off_all);
    hIqn2->regs->Ail[ailIndex].AIL_ICTL_IDC_IF.AIL_ICTL_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilIngressRegs
 *
 *   @b Description
 *   @n IQN2 AIL Ingress setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAilIfeRadStdGrpRegs, Iqn2Fl_setupAilIctlIdcIfRegs
 *
 *   @b Writes
 *   @n AIL_IQ_IFE_CHANNEL_CONFIGURATION_GROUP, AIL_IQ_IFE_RADIO_STANDARD_GROUP, 
 *      AIL_IQ_IFE_CONFIG_GROUP, AIL_IQ_IDC_CONFIGURATION_GROUP, 
 *      AIL_IQ_IDC_CHANNEL_CONFIG_GROUP, AIL_IFE_FRM_SAMP_TC_MMR_RAM, AIL_ICTL_IDC_IF,
 *      AIL_ICTL_PKT_IF, AIL_IQ_INGRESS_VBUS_MMR_GROUP, 
 *      AIL_CTL_INGRESS_VBUS_MMR_GROUP 
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilIngressRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilIngressRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /** pointer to AIL Ingress setup */
    Iqn2Fl_AilIgrSetup   *pAilIgrConfig;
    Iqn2Fl_AilInstance   ailIndex;
    uint32_t   tempReg, i;

    ailIndex       = ailSetup->ailInstNum;
    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilIgrConfig  = ailSetup->pAilIgrSetup;

    /* Setup AIL IQ IFE CHANNEL CONFIGURATION REGISTER */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_EN, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_en) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_AXC_OFFSET, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_axc_offset) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_RADIO_SEL, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_radio_sel) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_tdd_frc_off) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_OBSAI_CTL, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_obsai_ctl) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IFE_CHAN_CFG_CHAN_ENET_CTL, 
                      pAilIgrConfig->ail_iq_ife_chan_config_group[i].chan_enet_ctl);
        hIqn2->regs->Ail[ailIndex].AIL_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.AIL_IQ_IFE_CHAN_CFG[i] = tempReg;
    }

    /* Setup AIL IQ IFE RADIO STANDARD GROUP */
    Iqn2Fl_setupAilIfeRadStdGrpRegs (hIqn2, ailIndex, &(pAilIgrConfig->ail_iq_ife_radio_std_group));

    /* Setup AIL IQ IDC CONFIGURATION GROUP */
    tempReg = CSL_FMK(IQN_AIL_AIL_IQ_IDC_CFG_FAIL_MARK_ONLY, 
                  pAilIgrConfig->ail_iq_idc_cfg_grp.fail_mark_only) |
              CSL_FMK(IQN_AIL_AIL_IQ_IDC_CFG_FRC_OFF_ALL, 
                  pAilIgrConfig->ail_iq_idc_cfg_grp.frc_off_all) |
              CSL_FMK(IQN_AIL_AIL_IQ_IDC_CFG_RM_FAIL_FRC_OFF_EN, 
                  pAilIgrConfig->ail_iq_idc_cfg_grp.rm_fail_frc_off_en);
    hIqn2->regs->Ail[ailIndex].AIL_IQ_IDC_CONFIGURATION_GROUP.AIL_IQ_IDC_CFG = tempReg;

    /* Setup AIL IQ IDC CHANNEL CONFIGURATION GROUP */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_IDC_CH_CFG_DAT_SWAP, 
                      pAilIgrConfig->ail_iq_idc_ch_cfg_grp[i].dat_swap) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IDC_CH_CFG_IQ_ORDER, 
                      pAilIgrConfig->ail_iq_idc_ch_cfg_grp[i].iq_order) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IDC_CH_CFG_PKT_TYPE, 
                      pAilIgrConfig->ail_iq_idc_ch_cfg_grp[i].pkt_type) |
                  CSL_FMK(IQN_AIL_AIL_IQ_IDC_CH_CFG_CHAN_FRC_OFF, 
                      pAilIgrConfig->ail_iq_idc_ch_cfg_grp[i].chan_frc_off);
        hIqn2->regs->Ail[ailIndex].AIL_IQ_IDC_CHANNEL_CONFIG_GROUP.AIL_IQ_IDC_CH_CFG[i] = tempReg;
    }

    /* Setup AIL IFE FRM SAMP TC MMR RAM */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IFE_FRM_SAMP_TC_MMR_RAM[i].AIL_IQ_IFE_FRM_SAMP_TC_CFG, 
            IQN_AIL_AIL_IQ_IFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pAilIgrConfig->samp_tc[i]);
    }

    /* Setup AIL ICTL IDC IF REGISTERS */
    Iqn2Fl_setupAilIctlIdcIfRegs (hIqn2, ailIndex, &(pAilIgrConfig->ail_ictl_idc_if));

    /* Setup AIL ICTL PKT IF REGISTER */
    for (i = 0; i < 4; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_ICTL_PKT_IF.AIL_ICTL_CHAN_EN_CFG[i], 
            IQN_AIL_AIL_ICTL_CHAN_EN_CFG_CHAN_EN, 
            pAilIgrConfig->ail_ictl_pkt_if_chan_en[i]);
    }

    /* Setup AIL IQ INGRESS VBUS MMR GROUP -> IDC RATE CONTROL CONFIGURATION REGISTER
     * Default value is 15. Setting this value to a non-default value is possible if 0 < rate_idc <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_IQ_IDC_RATE_CTL_CFG_REG CMD word
     */
    if (pAilIgrConfig->rate_idc) {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_RATE_CTL_CFG,
            IQN_AIL_AIL_IQ_IDC_RATE_CTL_CFG_RATE,
            pAilIgrConfig->rate_idc);
    }

    /* Setup AIL CTL INGRESS VBUS MMR GROUP -> ICTL RATE CONTROL CONFIGURATION REGISTER
     * Default value is 15. Setting this value to a non-default value is possible if 0 < rate_ictl <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AIL_ICTL_RATE_CTL_CFG_REG CMD word
     */
    if (pAilIgrConfig->rate_ictl) {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_CTL_INGRESS_VBUS_MMR_GROUP.AIL_ICTL_RATE_CTL_CFG,
            IQN_AIL_AIL_ICTL_RATE_CTL_CFG_RATE,
            pAilIgrConfig->rate_ictl);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPeRegs
 *
 *   @b Description
 *   @n IQN2 AIL Protocol Encoder (PE) setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PE_COMMON, AIL_PE_OBSAI_HEADER_LUT, AIL_PE_CPRI_CW
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPeRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPeRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /** pointer to AIL PE setup */
    Iqn2Fl_AilPeSetup   *pAilPeConfig;
    Iqn2Fl_AilInstance   ailIndex;
    uint32_t   tempReg, i;

    ailIndex       = ailSetup->ailInstNum;
    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilPeConfig   = ailSetup->pAilPeSetup;

    /* Setup AIL PE GLOBAL CONFIGURATION REGISTER */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_COMMON.AIL_PE_GLOBAL_CFG, 
        IQN_AIL_AIL_PE_GLOBAL_CFG_ENET_HDR_SEL, 
        pAilPeConfig->ail_pe_common.enet_hdr_sel);

    /* Setup AIL PE CHANNEL CONFIGURATION REGISTER */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PE_CHAN_CFG_CRC_EN, 
                      pAilPeConfig->ail_pe_common.ail_pe_common_chan_cfg[i].crc_en) |
                  CSL_FMK(IQN_AIL_AIL_PE_CHAN_CFG_RT_CTL, 
                      pAilPeConfig->ail_pe_common.ail_pe_common_chan_cfg[i].rt_ctl) |
                  CSL_FMK(IQN_AIL_AIL_PE_CHAN_CFG_CRC_TYPE, 
                      pAilPeConfig->ail_pe_common.ail_pe_common_chan_cfg[i].crc_type) |
                  CSL_FMK(IQN_AIL_AIL_PE_CHAN_CFG_ETHERNET, 
                      pAilPeConfig->ail_pe_common.ail_pe_common_chan_cfg[i].ethernet) |
                  CSL_FMK(IQN_AIL_AIL_PE_CHAN_CFG_CRC_HDR, 
                      pAilPeConfig->ail_pe_common.ail_pe_common_chan_cfg[i].crc_hdr);
        hIqn2->regs->Ail[ailIndex].AIL_PE_COMMON.AIL_PE_CHAN_CFG[i] = tempReg;
    }

    /* Setup AIL PE OBSAI HEADER LUT */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_TS_ADR, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].ts_adr) |
                  CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_TYP, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].typ) |
                  CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_ADR, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].adr) |
                  CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_TS_MASK, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].ts_mask) |
                  CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_TS_FRMT, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].ts_frmt) |
                  CSL_FMK(IQN_AIL_AIL_PE_OBSAI_HDR_CFG_PS_INSERT, 
                      pAilPeConfig->ail_pe_obsai_hdr_lut[i].ps_insert);
        hIqn2->regs->Ail[ailIndex].AIL_PE_OBSAI_HEADER_LUT[i].AIL_PE_OBSAI_HDR_CFG = tempReg;
    }

    /* Setup AIL PE CPRI CW CHAN CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_CRC_RVRS, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].crc_rvrs) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_DELIN_SEL, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].delin_sel) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_CRC_SEL, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].crc_sel) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_CRC_INIT, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].crc_init) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_HF_LUT_EN, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].hf_lut_en) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_DLMT_IMUX, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].dlmt_imux) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_DLMT_OMUX, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].dlmt_omux) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_IMUX, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].imux) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_RT_CTL, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].rt_ctl) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_CHAN_CFG_BYTE_EN, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_chan_cfg[i].byte_en);
        hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_CW_CHAN_CFG[i] = tempReg;
    }

    /* Setup AIL PE CPRI CW HYPFRM 0-4 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_HYPFRM0_LUT_CFG, 
        IQN_AIL_AIL_PE_CPRI_HYPFRM0_LUT_CFG_HF_EN, 
        pAilPeConfig->ail_pe_cpri_cw.hf_en_part0);
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_HYPFRM1_LUT_CFG, 
        IQN_AIL_AIL_PE_CPRI_HYPFRM1_LUT_CFG_HF_EN, 
        pAilPeConfig->ail_pe_cpri_cw.hf_en_part1);
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_HYPFRM2_LUT_CFG, 
        IQN_AIL_AIL_PE_CPRI_HYPFRM2_LUT_CFG_HF_EN, 
        pAilPeConfig->ail_pe_cpri_cw.hf_en_part2);
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_HYPFRM3_LUT_CFG, 
        IQN_AIL_AIL_PE_CPRI_HYPFRM3_LUT_CFG_HF_EN, 
        pAilPeConfig->ail_pe_cpri_cw.hf_en_part3);
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_HYPFRM4_LUT_CFG, 
        IQN_AIL_AIL_PE_CPRI_HYPFRM4_LUT_CFG_HF_EN, 
        pAilPeConfig->ail_pe_cpri_cw.hf_en_part4);

    /* Setup AIL PE CPRI CW NULL CHARACTER CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_NULL_CFG, 
        IQN_AIL_AIL_PE_CPRI_NULL_CFG_NULL_CHAR, 
        pAilPeConfig->ail_pe_cpri_cw.null_char);

    /* Setup AIL PE CPRI CW CRC8 CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_CRC_CFG, 
        IQN_AIL_AIL_PE_CPRI_CRC_CFG_CRC8_POLY, 
        pAilPeConfig->ail_pe_cpri_cw.crc8_poly);

    /* Setup AIL PE CPRI CW FAST ETHERNET 4B5B CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PE_CPRI_4B5B_CFG_BIT_ORDER, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_fast_eth_4b5b[i].bit_order) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_4B5B_CFG_SSD_ORDER, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_fast_eth_4b5b[i].ssd_order) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_4B5B_CFG_HDR, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_fast_eth_4b5b[i].hdr) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_4B5B_CFG_HDR_PREAMBLE, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_fast_eth_4b5b[i].hdr_preamble) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_4B5B_CFG_HDR_SOP, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_fast_eth_4b5b[i].hdr_sop);
        hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_4B5B_CFG[i] = tempReg;
    }

    /* Setup AIL PE CPRI CW LOOK UP TABLE CFG */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_LUT_CFG_CW_CHAN, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i].cw_chan) |
                  CSL_FMK(IQN_AIL_AIL_PE_CPRI_CW_LUT_CFG_CW_EN, 
                      pAilPeConfig->ail_pe_cpri_cw.ail_pe_cpri_cw_lut[i].cw_en);
        hIqn2->regs->Ail[ailIndex].AIL_PE_CPRI_CW.AIL_PE_CPRI_CW_LUT_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPdCpriAxcCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL Protocol Decoder (PD) CPRI AxC Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilPdCpriAxcCfgSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PD_CPRI_AXC_CFG, AIL_PD_CPRI_AXC0_CFG, AIL_PD_CPRI_BUB_FSM_CFG,
 *      AIL_PD_CPRI_BUB_FSM2_CFG, AIL_PD_CPRI_TDM_FSM_CFG, AIL_PD_CPRI_RADSTD_CFG
 *      AIL_PD_CPRI_RADSTD1_CFG, AIL_PD_CPRI_RADSTD2_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPdCpriAxcCfgRegs (hIqn2, ailIndex, &ail_pd_cpri_axc_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPdCpriAxcCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilPdCpriAxcCfg *pAilPdCpriAxcCfgSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL PD CPRI AXC0 CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_AXC0_CFG_CONT_LUT_GRP, 
                      pAilPdCpriAxcCfgSetup->ail_pd_cpri_axc0_cfg[i].cont_lut_grp) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_AXC0_CFG_CONT_LUT_EN, 
                      pAilPdCpriAxcCfgSetup->ail_pd_cpri_axc0_cfg[i].cont_lut_en);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_AXC0_CFG[i] = tempReg;
    }

    for (i = 0; i < 8; i++)
    {
        /* AIL PD CPRI BUB FSM CFG */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_BUB_FSM_CFG[i], 
            IQN_AIL_AIL_PD_CPRI_BUB_FSM_CFG_KNC, 
            pAilPdCpriAxcCfgSetup->bub_fsm_cfg_knc[i]);

        /* AIL PD CPRI BUB FSM2 CFG */
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_BUB_FSM2_CFG_GAP_INT, 
                      pAilPdCpriAxcCfgSetup->bub_fsm2_cfg_gap_int[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_BUB_FSM2_CFG_GAP_FRAC, 
                      pAilPdCpriAxcCfgSetup->bub_fsm2_cfg_gap_frac[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_BUB_FSM2_CFG[i] = tempReg;

        /* AIL PD CPRI TDM FSM CFG */
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_TDM_FSM_CFG_NCONT,
                      pAilPdCpriAxcCfgSetup->ail_pd_cpri_tdm_fsm_cfg[i].ncont) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_TDM_FSM_CFG_STRT_LUT, 
                      pAilPdCpriAxcCfgSetup->ail_pd_cpri_tdm_fsm_cfg[i].start_lut);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_TDM_FSM_CFG[i] = tempReg;

        /* AIL PD CPRI RADSTD CFG */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD_CFG[i], 
            IQN_AIL_AIL_PD_CPRI_RADSTD_CFG_EN, 
            pAilPdCpriAxcCfgSetup->ail_iq_pd_cpri_axc_radstd_cfg.radstd_cfg_en[i]);

        /* AIL PD CPRI RADSTD1 CFG */
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_RADSTD1_CFG_BFRM_OFFSET, 
                      pAilPdCpriAxcCfgSetup->ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_bfrm_offset[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_RADSTD1_CFG_HFRM_OFFSET, 
                      pAilPdCpriAxcCfgSetup->ail_iq_pd_cpri_axc_radstd_cfg.radstd1_cfg_hfrm_offset[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD1_CFG[i] = tempReg;

        /* AIL PD CPRI RADSTD2 CFG */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD2_CFG[i], 
            IQN_AIL_AIL_PD_CPRI_RADSTD2_CFG_BFRM_NUM, 
            pAilPdCpriAxcCfgSetup->ail_iq_pd_cpri_axc_radstd_cfg.radstd2_cfg_bfrm_num[i]);
    }

    /* Setup AIL PD CPRI AXC TDM LUT CFG */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_AXC_TDM_LUT_CFG_AXC, 
                      pAilPdCpriAxcCfgSetup->axc_tdm_lut_cfg_axc[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_AXC_TDM_LUT_CFG_EN, 
                      pAilPdCpriAxcCfgSetup->axc_tdm_lut_cfg_en[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_AXC_TDM_LUT_CFG[i].AIL_PD_CPRI_AXC_TDM_LUT_CFG = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPdCpriCwCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL Protocol Decoder (PD) CPRI Code Word (CW) Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilPdCpriCwCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PD_CPRI_CW_CHAN_CFG, AIL_PD_CPRI_CW_LUT_CFG, AIL_PD_CPRI_HYPFRM0_LUT_CFG,
 *      AIL_PD_CPRI_HYPFRM1_LUT_CFG, AIL_PD_CPRI_HYPFRM2_LUT_CFG, AIL_PD_CPRI_HYPFRM3_LUT_CFG
 *      AIL_PD_CPRI_HYPFRM4_LUT_CFG, AIL_PD_CPRI_NULL_CFG, AIL_PD_CPRI_CRC_CFG,
 *      AIL_PD_CPRI_4B5B_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPdCpriCwCfgRegs (hIqn2, ailIndex, &ail_pd_cpri_cw_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPdCpriCwCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilPdCpriCwCfg *pAilPdCpriCwCfgSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL PD CPRI CW CHAN CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_CHAN_EN, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].chan_en) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_HF_LUT_EN, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].hf_lut_en) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_DELIN_SEL, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].delin_sel) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_CRC_SEL, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].crc_sel) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_CRC_INIT, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].crc_init) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_DLMT_IMUX, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].dlmt_imux) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_DLMT_OMUX, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].dlmt_omux) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_QWD_OMUX, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].qwd_omux) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_HDLC_RVRS_CRC, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].hdlc_rvrs_crc) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_CHAN_CFG_BYTE_EN, 
                      pAilPdCpriCwCfgSetup->chan_cfg[i].byte_en);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_CW_CHAN_CFG[i] = tempReg;
    }

    /* Setup AIL PD CPRI CW LUT CFG */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_LUT_CFG_CW_CHAN, 
                      pAilPdCpriCwCfgSetup->lut_cfg_cw_chan[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_LUT_CFG_CW_EN, 
                      pAilPdCpriCwCfgSetup->lut_cfg_cw_en[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_CW_LUT_CFG[i] = tempReg;
    }

    /* Setup AIL PD CPRI HYPFRM0 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_HYPFRM0_LUT_CFG, 
        IQN_AIL_AIL_PD_CPRI_HYPFRM0_LUT_CFG_HF_EN, 
        pAilPdCpriCwCfgSetup->hypfrm0_lut_cfg_hf_en);

    /* Setup AIL PD CPRI HYPFRM1 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_HYPFRM1_LUT_CFG, 
        IQN_AIL_AIL_PD_CPRI_HYPFRM1_LUT_CFG_HF_EN, 
        pAilPdCpriCwCfgSetup->hypfrm1_lut_cfg_hf_en);

    /* Setup AIL PD CPRI HYPFRM2 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_HYPFRM2_LUT_CFG, 
        IQN_AIL_AIL_PD_CPRI_HYPFRM2_LUT_CFG_HF_EN, 
        pAilPdCpriCwCfgSetup->hypfrm2_lut_cfg_hf_en);

    /* Setup AIL PD CPRI HYPFRM3 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_HYPFRM3_LUT_CFG, 
        IQN_AIL_AIL_PD_CPRI_HYPFRM3_LUT_CFG_HF_EN, 
        pAilPdCpriCwCfgSetup->hypfrm3_lut_cfg_hf_en);

    /* Setup AIL PD CPRI HYPFRM4 LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_HYPFRM4_LUT_CFG, 
        IQN_AIL_AIL_PD_CPRI_HYPFRM4_LUT_CFG_HF_EN, 
        pAilPdCpriCwCfgSetup->hypfrm4_lut_cfg_hf_en);

    /* Setup AIL PD CPRI NULL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_NULL_CFG, 
        IQN_AIL_AIL_PD_CPRI_NULL_CFG_NULL_CHAR, 
        pAilPdCpriCwCfgSetup->null_cfg_null_char);

    /* Setup AIL PD CPRI CRC CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_CRC_CFG_CRC8_POLY, 
                  pAilPdCpriCwCfgSetup->crc_cfg_crc8_poly) |
              CSL_FMK(IQN_AIL_AIL_PD_CPRI_CRC_CFG_CRC8_COMP, 
                  pAilPdCpriCwCfgSetup->crc_cfg_crc8_comp);
    hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_CRC_CFG = tempReg;

    /* Setup AIL PD CPRI 4B5B CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_LUT_CFG_CW_CHAN, 
                      pAilPdCpriCwCfgSetup->cpri_4b5b_cfg_hdr[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_LUT_CFG_CW_EN, 
                      pAilPdCpriCwCfgSetup->cpri_4b5b_cfg_bit_order[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_CW_LUT_CFG_CW_EN, 
                      pAilPdCpriCwCfgSetup->cpri_4b5b_cfg_ssd_order[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_CPRI_CW_CFG.AIL_PD_CPRI_4B5B_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailSetupPdObsaiCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL Protocol Decoder (PD) OBSAI Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilPdObsaiCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PD_OBSAI_CFG, AIL_PD_OBSAI_GSM_BBHOP_CFG, AIL_PD_OBSAI_RADSTD_CFG,
 *      AIL_PD_OBSAI_RADT_CFG, AIL_PD_OBSAI_FRM_TC_CFG, AIL_PD_OBSAI_CHAN_CFG,
 *      AIL_PD_OBSAI_ROUTE_CFG, AIL_PD_OBSAI_TYPE_LUT_CFG, AIL_PD_OBSAI_FRM_MSG_TC_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_ailSetupPdObsaiCfgRegs (hIqn2, ailIndex, &ail_pd_obsai_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_ailSetupPdObsaiCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilPdObsaiCfg *pAilPdObsaiCfgSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AIL PD OBSAI GSM BBHOP CFG */
    for (i = 0; i < 2; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_CFG.AIL_PD_OBSAI_GSM_BBHOP_CFG[i], 
            IQN_AIL_AIL_PD_OBSAI_GSM_BBHOP_CFG_OFF_STB, 
            pAilPdObsaiCfgSetup->gsm_bbhop_cfg_off_stb[i]);
    }

    for (i = 0; i < 8; i++)
    {
        /* AIL_PD_OBSAI_RADSTD_CFG */
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_OBSAI_RADSTD_CFG_WDOG_TC, 
                      pAilPdObsaiCfgSetup->radstd_cfg_wdog_tc[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_RADSTD_CFG_AXCOFFSET_WIN, 
                      pAilPdObsaiCfgSetup->radstd_cfg_axcoffset_win[i]);
        hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_CFG.AIL_PD_OBSAI_RADSTD_CFG[i] = tempReg;

        /* AIL_PD_OBSAI_RADT_CFG */
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_CFG.AIL_PD_OBSAI_RADT_CFG[i], 
            IQN_AIL_AIL_PD_OBSAI_RADT_CFG_TC, 
            pAilPdObsaiCfgSetup->radt_cfg_tc[i]);
    }

    /* Setup AIL PD OBSAI FRM TC CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_OBSAI_FRM_TC_CFG_SYM_TC, 
                      pAilPdObsaiCfgSetup->frm_tc_cfg[i].sym_tc) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_FRM_TC_CFG_INDEX_SC, 
                      pAilPdObsaiCfgSetup->frm_tc_cfg[i].index_sc) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_FRM_TC_CFG_INDEX_TC, 
                      pAilPdObsaiCfgSetup->frm_tc_cfg[i].index_tc);
        hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_CFG.AIL_PD_OBSAI_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup AIL PD OBSAI CHAN CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_OBSAI_CHAN_CFG_WDOG_EN, 
                      pAilPdObsaiCfgSetup->lut_cfg.chan_cfg[i].wdog_en) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_CHAN_CFG_GSM_UL, 
                      pAilPdObsaiCfgSetup->lut_cfg.chan_cfg[i].gsm_ul);
        hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_LUT_CFG.AIL_PD_OBSAI_CHAN_CFG[i] = tempReg;
    }

    /* Setup AIL PD OBSAI ROUTE CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_OBSAI_ROUTE_CFG_CHAN_TS, 
                      pAilPdObsaiCfgSetup->lut_cfg.route_cfg[i].chan_ts) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_ROUTE_CFG_CHAN_TYPE, 
                      pAilPdObsaiCfgSetup->lut_cfg.route_cfg[i].chan_type) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_ROUTE_CFG_CHAN_ADR, 
                      pAilPdObsaiCfgSetup->lut_cfg.route_cfg[i].chan_adr) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_ROUTE_CFG_CHAN_MASK, 
                      pAilPdObsaiCfgSetup->lut_cfg.route_cfg[i].chan_mask) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_ROUTE_CFG_CHAN_EN, 
                      pAilPdObsaiCfgSetup->lut_cfg.route_cfg[i].chan_en);
        hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_LUT_CFG.AIL_PD_OBSAI_ROUTE_CFG[i] = tempReg;
    }

    /* Setup AIL PD OBSAI TYPE LUT CFG */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_TS_FORMAT, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].ts_format) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_CRC_TYPE, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].crc_type) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_CRC_EN, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].crc_en) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_CRC_HDR, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].crc_hdr) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_OBSAI_PKT_EN, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].obsai_pkt_en) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_ENET_STRIP, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].enet_strip) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_RP3_01, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].rp3_01) |
                  CSL_FMK(IQN_AIL_AIL_PD_OBSAI_TYPE_LUT_CFG_RP3_01_RST, 
                      pAilPdObsaiCfgSetup->lut_cfg.type_lut_cfg[i].rp3_01_rst);
        hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_LUT_CFG.AIL_PD_OBSAI_TYPE_LUT_CFG[i] = tempReg;
    }

    /* Setup AIL PD OBSAI FRM MSG TC CFG */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_FRM_MSG_TC_CFG[i].AIL_PD_OBSAI_FRM_MSG_TC_CFG, 
            IQN_AIL_AIL_PD_OBSAI_FRM_MSG_TC_CFG_TC, 
            pAilPdObsaiCfgSetup->frm_msg_tc_cfg_tc[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPdRegs
 *
 *   @b Description
 *   @n IQN2 AIL Protocol Decoder (PD) setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAilPdCpriAxcCfgRegs, Iqn2Fl_setupAilPdCpriCwCfgRegs,
 *      Iqn2Fl_ailSetupPdObsaiCfgRegs
 *
 *   @b Writes
 *   @n AIL_PD_COMMON_CHAN_CFG, AIL_PD_CPRI_AXC_CFG, AIL_PD_CPRI_AXC_TDM_LUT_CFG
 *      AIL_PD_CPRI_CW_CFG, AIL_PD_OBSAI_CFG, AIL_PD_OBSAI_LUT_CFG, 
 *      AIL_PD_OBSAI_FRM_MSG_TC_CFG 
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPdRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPdRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /** pointer to AIL PD setup */
    Iqn2Fl_AilPdSetup   *pAilPdConfig;
    Iqn2Fl_AilInstance   ailIndex;
    uint32_t   tempReg, i;

    ailIndex       = ailSetup->ailInstNum;
    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilPdConfig = ailSetup->pAilPdSetup;

    /* Setup AIL PD COMMON CHAN CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CHAN_CFG_AXC_OFFSET, 
                      pAilPdConfig->ail_pd_common[i].axc_offset) |
                  CSL_FMK(IQN_AIL_AIL_PD_CHAN_CFG_RAD_STD, 
                      pAilPdConfig->ail_pd_common[i].rad_std);
        hIqn2->regs->Ail[ailIndex].AIL_PD_COMMON_CHAN_CFG[i].AIL_PD_CHAN_CFG = tempReg;
    }

    /* Setup AIL PD CPRI AXC CFG and AIL PD CPRI AXC TDM LUT CFG */
    Iqn2Fl_setupAilPdCpriAxcCfgRegs (hIqn2, ailIndex, &(pAilPdConfig->ail_pd_cpri_axc_cfg));

    /* Setup AIL PD CPRI CW CFG */
    Iqn2Fl_setupAilPdCpriCwCfgRegs (hIqn2, ailIndex, &(pAilPdConfig->ail_pd_cpri_cw_cfg));

    /* Setup AIL PD OBSAI CFG */
    Iqn2Fl_ailSetupPdObsaiCfgRegs  (hIqn2, ailIndex, &(pAilPdConfig->ail_pd_obsai_cfg));

    /* Setup AIL PD OBSAI FRM MSG TC CFG */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PD_OBSAI_FRM_MSG_TC_CFG[i].AIL_PD_OBSAI_FRM_MSG_TC_CFG, 
            IQN_AIL_AIL_PD_OBSAI_FRM_MSG_TC_CFG_TC, 
            pAilPdConfig->ail_pd_obsai_frm_msg_tc_cfg_tc[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilUatRegs
 *
 *   @b Description
 *   @n IQN2 AIL Micro Antenna Timer (UAT) setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_hwControl
 *
 *   @b Writes
 *   @n  AIL_UAT_GEN_CTL, AIL_UAT_AIL_REGS, AIL_UAT_EGR_RADT, 
 *       AIL_UAT_ING_RADT, AIL_UAT_RADT_EVT
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilUatRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilUatRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /* pointer to AIL UAT setup */
    Iqn2Fl_AilUatSetup   *pAilUatConfig;
    Iqn2Fl_HwControlMultiArgs   args;
    uint32_t   i;
    Iqn2Fl_Status   ret_val = IQN2FL_SOK;

    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilUatConfig = ailSetup->pAilUatSetup;

    /* Setup AIL UAT GEN CTL -> AIL UAT BCN TERMINAL COUNT REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_TERMINAL_COUNT_REG,
                                &(pAilUatConfig->ail_uat_bcn_tc_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT GEN CTL -> AIL UAT BCN OFFSET REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG,
                                &(pAilUatConfig->ail_uat_bcn_offset_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT PIMAX CFG REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_PIMAX_CFG_REG,
                                &(pAilUatConfig->ail_uat_pimax_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT PIMIN CFG REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_PIMIN_CFG_REG,
                                &(pAilUatConfig->ail_uat_pimin_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT TM FRAME COUNT (BFN) CONFIG REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_TM_BFN_CFG_REG,
                                &(pAilUatConfig->ail_uat_tm_bfn_cfg_wr_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT RT FRAME BOUNDARY (FB) COMPARE REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_RT_FB_CFG_REG,
                                &(pAilUatConfig->ail_uat_rt_fb_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT PE FRAME BOUNDARY (FB) COMPARE REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_PE_FB_CFG_REG,
                                &(pAilUatConfig->ail_uat_pe_fb_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT AIL REGS -> AIL UAT TM FRAME BOUNDARY (FB) COMPARE REGISTER */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AIL_UAT_TM_FB_CFG_REG,
                                &(pAilUatConfig->ail_uat_tm_fb_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AIL UAT EGR RADT -> AIL UAT RADT TERMINAL COUNT REGISTER */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_egr_radt_tc_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_EGR_RADT_TC_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AIL UAT EGR RADT -> AIL UAT RADT OFFSET REGISTER */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_egr_radt_offset_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AIL UAT ING RADT -> AIL UAT RADT TERMINAL COUNT REGISTER */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_ing_radt_tc_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_ING_RADT_TC_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AIL UAT ING RADT -> AIL UAT RADT OFFSET REGISTER */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_ing_radt_offset_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AIL UAT RADT EVT -> AIL UAT RADT EVENT COMPARE REGISTER */
    for (i = 0; i < 22; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_evt_radt_cmp_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_EVT_RADT_CMP_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AIL UAT RADT EVT -> AIL UAT RADT EVENT CLOCK COUNT TC REGISTER */
    for (i = 0; i < 22; i++)
    {
        args.reg_arg = (void *)&(pAilUatConfig->ail_uat_evt_clk_cnt_tc_cfg_val[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AIL_UAT_EVT_CLK_CNT_TC_CFG_REG,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyTmCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY TM (Tx Mac) Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilPhyTmCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PHY_TM
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyTmCfgRegs (hIqn2, ailIndex, &ail_phy_tm_regs);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyTmCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilPhyTmCfg *pAilPhyTmCfgSetup
)
{
    uint32_t   tempReg;

    /* Setup AIL PHY TM CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CFG, 
        IQN_AIL_AIL_PHY_TM_CFG_EN, 
        pAilPhyTmCfgSetup->phy_tm_cfg_en);

    /* Setup AIL PHY TM CTRL CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_CTRL_CFG_FLUSH, 
                  pAilPhyTmCfgSetup->phy_tm_ctrl_cfg_flush) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CTRL_CFG_IDLE, 
                  pAilPhyTmCfgSetup->phy_tm_ctrl_cfg_idle) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CTRL_CFG_RESYNC, 
                  pAilPhyTmCfgSetup->phy_tm_ctrl_cfg_resync) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CTRL_CFG_LOS_EN, 
                  pAilPhyTmCfgSetup->phy_tm_ctrl_cfg_los_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CTRL_CFG = tempReg;

    /* Setup AIL PHY TM SCR CTRL CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_SCR_CTRL_CFG_SEED_VALUE, 
                  pAilPhyTmCfgSetup->phy_tm_scr_ctrl_cfg_seed_value) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_SCR_CTRL_CFG_SCR_EN, 
                  pAilPhyTmCfgSetup->phy_tm_scr_ctrl_cfg_scr_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_SCR_CTRL_CFG = tempReg;

    /* Setup AIL PHY TM L1 INBAND CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_CFG_L1_INBAND_RST, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_cfg.l1_inband_rst) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_CFG_L1_INBAND_RAI, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_cfg.l1_inband_rai) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_CFG_L1_INBAND_SDI, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_cfg.l1_inband_sdi) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_CFG_L1_INBAND_LOS, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_cfg.l1_inband_los) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_CFG_L1_INBAND_LOF, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_cfg.l1_inband_lof);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_L1_INBAND_CFG = tempReg;

    /* Setup AIL PHY TM L1 INBAND EN CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_RAI_RAIRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.rai_rairx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_RAI_LOFRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.rai_lofrx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_RAI_LOFERR_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.rai_loferr_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_RAI_LOSRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.rai_losrx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_RAI_LOSERR_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.rai_loserr_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_LOF_LOFRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.lof_lofrx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_LOF_LOFERR_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.lof_loferr_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_LOS_LOSRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.los_losrx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_LOS_LOSERR_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.los_loserr_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_L1_INBAND_EN_CFG_SDI_RSTRX_EN, 
                  pAilPhyTmCfgSetup->ail_phy_tm_l1_inb_en_cfg.sdi_rstrx_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_L1_INBAND_EN_CFG = tempReg;

    /* Setup AIL PHY TM LOSERR SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_LOSERR_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_LOSERR_SEL_CFG_LINK_LOSERR, 
        pAilPhyTmCfgSetup->phy_tm_loserr_sel_cfg_link_loserr);

    /* Setup AIL PHY TM LOFERR SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_LOFERR_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_LOFERR_SEL_CFG_LINK_LOFERR, 
        pAilPhyTmCfgSetup->phy_tm_loferr_sel_cfg_link_loferr);

    /* Setup AIL PHY TM LOSRX SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_LOSRX_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_LOSRX_SEL_CFG_LINK_LOSRX, 
        pAilPhyTmCfgSetup->phy_tm_losrx_sel_cfg_link_losrx);

    /* Setup AIL PHY TM LOFRX SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_LOFRX_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_LOFRX_SEL_CFG_LINK_LOFRX, 
        pAilPhyTmCfgSetup->phy_tm_lofrx_sel_cfg_link_lofrx);

    /* Setup AIL PHY TM RAIRX SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_RAIRX_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_RAIRX_SEL_CFG_LINK_RAIRX, 
        pAilPhyTmCfgSetup->phy_tm_rairx_sel_cfg_link_rairx);

    /* Setup AIL PHY TM RSTRX SEL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_RSTRX_SEL_CFG, 
        IQN_AIL_AIL_PHY_TM_RSTRX_SEL_CFG_LINK_RSTRX, 
        pAilPhyTmCfgSetup->phy_tm_rstrx_sel_cfg_link_rstrx);

    /* Setup AIL PHY TM CPRI PTRP CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_PTRP_CFG, 
        IQN_AIL_AIL_PHY_TM_CPRI_PTRP_CFG_PTR_P, 
        pAilPhyTmCfgSetup->phy_tm_cpri_ptrp_cfg_ptr_p);

    /* Setup AIL PHY TM CPRI STARTUP CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_STARTUP_CFG, 
        IQN_AIL_AIL_PHY_TM_CPRI_STARTUP_CFG_STARTUP, 
        pAilPhyTmCfgSetup->phy_tm_cpri_startup_cfg_startup);

    /* Setup AIL PHY TM CPRI VERSION CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_VERSION_CFG, 
        IQN_AIL_AIL_PHY_TM_CPRI_VERSION_CFG_PROT_VERS, 
        pAilPhyTmCfgSetup->phy_tm_cpri_version_cfg_prot_vers);

    /* Setup AIL PHY TM CPRI PORTID A CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_A_CFG_Z52_0, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_a_cfg_z52_0) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_A_CFG_Z52_1, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_a_cfg_z52_1) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_A_CFG_Z116_0, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_a_cfg_z116_0) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_A_CFG_Z116_1, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_a_cfg_z116_1);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_PORTID_A_CFG = tempReg;

    /* Setup AIL PHY TM CPRI PORTID B CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_B_CFG_Z180_0, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_b_cfg_z180_0) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_B_CFG_Z180_1, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_b_cfg_z180_1) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_B_CFG_Z244_0, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_b_cfg_z244_0) |
              CSL_FMK(IQN_AIL_AIL_PHY_TM_CPRI_PORTID_B_CFG_Z244_1, 
                  pAilPhyTmCfgSetup->phy_tm_cpri_portid_b_cfg_z244_1);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_PORTID_B_CFG = tempReg;

    /* Setup AIL PHY TM CPRI SCR CTRL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_TM.AIL_PHY_TM_CPRI_SCR_CTRL_CFG, 
        IQN_AIL_AIL_PHY_TM_CPRI_SCR_CTRL_CFG_SEED_VALUE, 
        pAilPhyTmCfgSetup->phy_tm_cpri_scr_ctrl_cfg_seed_value);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyRmCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY RM (Rx Mac) Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         ailIndex   AIL instance number
         pAilPhyRmCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_PHY_RM
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyRmCfgRegs (hIqn2, ailIndex, &ail_phy_rm_regs);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyRmCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilInstance ailIndex,
    Iqn2Fl_AilPhyRmCfg *pAilPhyRmCfgSetup
)
{
    uint32_t   tempReg;

    /* Setup AIL PHY RM CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_CFG_RX_EN, 
                  pAilPhyRmCfgSetup->phy_rm_cfg_rx_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_CFG_DATA_TRC_SEL, 
                  pAilPhyRmCfgSetup->phy_rm_cfg_data_trc_sel) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_CFG_RST_EN, 
                  pAilPhyRmCfgSetup->phy_rm_cfg_rst_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_CFG = tempReg;

    /* Setup AIL PHY RM DP CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_DP_CFG_FORCE_RX_STATE, 
                  pAilPhyRmCfgSetup->phy_rm_dp_cfg.force_rx_state) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_DP_CFG_ERROR_SUPPRESS, 
                  pAilPhyRmCfgSetup->phy_rm_dp_cfg.error_suppress) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_DP_CFG_SD_AUTO_ALIGN_EN, 
                  pAilPhyRmCfgSetup->phy_rm_dp_cfg.sd_auto_align_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_DP_CFG_LCV_UNSYNC_EN, 
                  pAilPhyRmCfgSetup->phy_rm_dp_cfg.lcv_unsync_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_DP_CFG = tempReg;

    /* Setup AIL PHY RM LCV CTRL CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_LCV_CTRL_CFG_EN, 
                  pAilPhyRmCfgSetup->phy_rm_lcv_ctrl_cfg_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_LCV_CTRL_CFG_LOS_DET_THOLD, 
                  pAilPhyRmCfgSetup->phy_rm_lcv_ctrl_cfg_los_det_thold);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_LCV_CTRL_CFG = tempReg;

    /* Setup AIL PHY RM FSM SYNC CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_FSM_SYNC_CFG_SYNC_T, 
                  pAilPhyRmCfgSetup->phy_rm_fsm_sync_cfg_sync_t) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_FSM_SYNC_CFG_FRAME_SYNC_T, 
                  pAilPhyRmCfgSetup->phy_rm_fsm_sync_cfg_frame_sync_t);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_FSM_SYNC_CFG = tempReg;

    /* Setup AIL PHY RM FSM UNSYNC CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_FSM_UNSYNC_CFG_UNSYNC_T, 
                  pAilPhyRmCfgSetup->phy_rm_fsm_unsync_cfg_unsync_t) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_FSM_UNSYNC_CFG_FRAME_UNSYNC_T, 
                  pAilPhyRmCfgSetup->phy_rm_fsm_unsync_cfg_frame_unsync_t);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_FSM_UNSYNC_CFG = tempReg;

    /* Setup AIL PHY RM DSCR CTRL CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_DSCR_CTRL_CFG, 
        IQN_AIL_AIL_PHY_RM_DSCR_CTRL_CFG_SCR_EN, 
        pAilPhyRmCfgSetup->phy_rm_dscr_ctrl_cfg_scr_en);

    /* Setup AIL PHY RM CLK DET CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RM_CLK_DET_CFG_WD_EN, 
                  pAilPhyRmCfgSetup->phy_rm_clk_det_cfg.wd_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_CLK_DET_CFG_CQ_EN, 
                  pAilPhyRmCfgSetup->phy_rm_clk_det_cfg.cq_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_CLK_DET_CFG_WD_WRAP, 
                  pAilPhyRmCfgSetup->phy_rm_clk_det_cfg.wd_wrap) |
              CSL_FMK(IQN_AIL_AIL_PHY_RM_CLK_DET_CFG_MON_WRAP, 
                  pAilPhyRmCfgSetup->phy_rm_clk_det_cfg.mon_wrap);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RM.AIL_PHY_RM_CLK_DET_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY (Global, RT, CI/CO LUT, TM, RM) setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            ailSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAilPhyTmCfgRegs, Iqn2Fl_setupAilPhyRmCfgRegs
 *
 *   @b Writes
 *   @n  AIL_PHY_GLB, AIL_PHY_RT, AIL_PHY_CI_LUT, AIL_PHY_CO_LUT, 
 *       AIL_PHY_CI_LUT_A, AIL_PHY_CI_LUT_B, AIL_PHY_CO_LUT_A, AIL_PHY_CO_LUT_B,
 *       AIL_PHY_TM, AIL_PHY_RM
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyRegs (hIqn2, ailSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilSetup *ailSetup
)
{
    /* pointer to AIL PHY setup */
    Iqn2Fl_AilPhySetup   *pAilPhyConfig;
    Iqn2Fl_AilInstance   ailIndex;
    uint32_t   tempReg, i;

    ailIndex       = ailSetup->ailInstNum;
    hIqn2->arg_ail = ailSetup->ailInstNum;
    pAilPhyConfig  = ailSetup->pAilPhySetup;

    /* Setup AIL PHY GLB CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_GLB_CFG_LINK_RATE, 
                  pAilPhyConfig->ail_phy_glb_cfg.link_rate) |
              CSL_FMK(IQN_AIL_AIL_PHY_GLB_CFG_OBSAI_CPRI, 
                  pAilPhyConfig->ail_phy_glb_cfg.obsai_cpri) |
              CSL_FMK(IQN_AIL_AIL_PHY_GLB_CFG_SHORT_FRM_EN, 
                  pAilPhyConfig->ail_phy_glb_cfg.short_frm_en);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_GLB.AIL_PHY_GLB_CFG = tempReg;

    /* Setup AIL PHY RT CFG */
    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_RT_CFG_CONFIG, 
                  pAilPhyConfig->ail_phy_rt_cfg.config) |
              CSL_FMK(IQN_AIL_AIL_PHY_RT_CFG_EM_EN, 
                  pAilPhyConfig->ail_phy_rt_cfg.em_en) |
              CSL_FMK(IQN_AIL_AIL_PHY_RT_CFG_CI_LINK, 
                  pAilPhyConfig->ail_phy_rt_cfg.ci_link) |
              CSL_FMK(IQN_AIL_AIL_PHY_RT_CFG_BF_DELAY, 
                  pAilPhyConfig->ail_phy_rt_cfg.bf_delay);
    hIqn2->regs->Ail[ailIndex].AIL_PHY_RT.AIL_PHY_RT_CFG = tempReg;

    /* Setup AIL PHY CI LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_CI_LUT.AIL_PHY_CI_LUT_CFG, 
        IQN_AIL_AIL_PHY_CI_LUT_CFG_SEL, 
        pAilPhyConfig->ail_phy_ci_co_lut_cfg.phy_ci_lut_cfg_sel);

    /* Setup AIL PHY CO LUT CFG */
    CSL_FINS(hIqn2->regs->Ail[ailIndex].AIL_PHY_CO_LUT.AIL_PHY_CO_LUT_CFG, 
        IQN_AIL_AIL_PHY_CO_LUT_CFG_SEL, 
        pAilPhyConfig->ail_phy_ci_co_lut_cfg.phy_co_lut_cfg_sel);

    /* Setup AIL PHY CI LUTA CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_COUNT, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_count) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_OFFSET, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_offset) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_TYPE, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_type) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_LAST, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_luta_cfg[i].smpl_last);
        hIqn2->regs->Ail[ailIndex].AIL_PHY_CI_LUT_A[i].AIL_PHY_CI_LUTA_CFG = tempReg;
    }

    /* Setup AIL PHY CI LUTB CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_COUNT, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_lutb_cfg[i].smpl_count) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_OFFSET, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_lutb_cfg[i].smpl_offset) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_TYPE, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_lutb_cfg[i].smpl_type) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_LAST, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_ci_lutb_cfg[i].smpl_last);
        hIqn2->regs->Ail[ailIndex].AIL_PHY_CI_LUT_B[i].AIL_PHY_CI_LUTB_CFG = tempReg;
    }

    /* Setup AIL PHY CO LUTA CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_COUNT, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_count) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_OFFSET, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_offset) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_TYPE, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_type) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_LAST, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_luta_cfg[i].smpl_last);
        hIqn2->regs->Ail[ailIndex].AIL_PHY_CO_LUT_A[i].AIL_PHY_CO_LUTA_CFG = tempReg;
    }

    /* Setup AIL PHY CO LUTB CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_COUNT, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_lutb_cfg[i].smpl_count) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_OFFSET, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_lutb_cfg[i].smpl_offset) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_TYPE, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_lutb_cfg[i].smpl_type) |
                  CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_LAST, 
                      pAilPhyConfig->ail_phy_ci_co_lut_cfg.ail_phy_co_lutb_cfg[i].smpl_last);
        hIqn2->regs->Ail[ailIndex].AIL_PHY_CO_LUT_B[i].AIL_PHY_CO_LUTB_CFG = tempReg;
    }

    /* Setup AIL PHY TM (Tx Mac) Registers */
    Iqn2Fl_setupAilPhyTmCfgRegs (hIqn2, ailIndex, &(pAilPhyConfig->ail_phy_tm_regs));

    /* Setup AIL PHY RM (Rx Mac) Registers */
    Iqn2Fl_setupAilPhyRmCfgRegs (hIqn2, ailIndex, &(pAilPhyConfig->ail_phy_rm_regs));

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupTopVcSysStsCfgRegs
 *
 *   @b Description
 *   @n IQN2 Top VC Sys Status Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pTopVcSysStsCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupTopVcSysStsSwResetStbRegs
 *
 *   @b Writes
 *   @n VC_SYS_STS_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupTopVcSysStsCfgRegs (hIqn2, &top_vc_sys_sts_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupTopVcSysStsCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_TopVcSysStsSetup *pTopVcSysStsCfgSetup
)
{
    uint32_t   tempReg;

    /* Setup VC SCRATCH */
    CSL_FINS(hIqn2->regs->Top.VC_SYS_STS_CFG.VC_SCRATCH, 
        IQN2_TOP_VC_SCRATCH_SCRATCH, 
        pTopVcSysStsCfgSetup->scratch);

    /* Setup VC SW RESET STB register fields */
    Iqn2Fl_setupTopVcSysStsSwResetStbRegs (hIqn2, &(pTopVcSysStsCfgSetup->vc_sys_sts_sw_reset_stb));

    /* Setup VC EMU CFG register fields */
    tempReg = CSL_FMK(IQN2_TOP_VC_EMU_CFG_FREERUN, 
                  pTopVcSysStsCfgSetup->freerun) |
              CSL_FMK(IQN2_TOP_VC_EMU_CFG_SOFT, 
                  pTopVcSysStsCfgSetup->soft) |
              CSL_FMK(IQN2_TOP_VC_EMU_CFG_RT_SEL, 
                  pTopVcSysStsCfgSetup->rt_sel) |
              CSL_FMK(IQN2_TOP_VC_EMU_CFG_FRC_SHUTDOWN, 
                  pTopVcSysStsCfgSetup->frc_shutdown);
    hIqn2->regs->Top.VC_SYS_STS_CFG.VC_EMU_CFG = tempReg;

    /* Setup VC DTMUX CFG register fields */
    CSL_FINS(hIqn2->regs->Top.VC_SYS_STS_CFG.VC_DTMUX_CFG, 
        IQN2_TOP_VC_DTMUX_CFG_VC_DTMUX, 
        pTopVcSysStsCfgSetup->vc_dtmux);

    /* Setup VC CLKCTL SYSCLK CFG register fields */
    tempReg = CSL_FMK(IQN2_TOP_VC_CLKCTL_SYSCLK_SEL_CFG_SYSCLK_SEL, 
                  pTopVcSysStsCfgSetup->sysclk_sel) |
              CSL_FMK(IQN2_TOP_VC_CLKCTL_SYSCLK_SEL_CFG_AT_DFE_CLK_SEL, 
                  pTopVcSysStsCfgSetup->at_dfe_clk_sel);
    hIqn2->regs->Top.VC_SYS_STS_CFG.VC_CLKCTL_SYSCLK_SEL_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupTopPsrCfgRegs
 *
 *   @b Description
 *   @n IQN2 Top PSR Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pTopVcSysStsCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n PSR_CONFIG_REGS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupTopPsrCfgRegs (hIqn2, &top_psr_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupTopPsrCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_TopPsrConfigSetup *pTopPsrConfigSetup
)
{
    uint32_t   tempReg, i;

    /* Setup PSR EGR CFG */
    CSL_FINS(hIqn2->regs->Top.PSR_CONFIG_REGS.PSR_EGR_CFG, 
        IQN2_TOP_PSR_EGR_CFG_BW_LIMIT, 
        pTopPsrConfigSetup->bw_limit);

    /* Setup PSR ING CFG */
    CSL_FINS(hIqn2->regs->Top.PSR_CONFIG_REGS.PSR_ING_CFG, 
        IQN2_TOP_PSR_ING_CFG_PS_DATA_EXT, 
        pTopPsrConfigSetup->ps_data_ext);

    /* Setup PSR ING CHAN CFG */
    for (i = 0; i < 48; i++)
    {
        tempReg = CSL_FMK(IQN2_TOP_PSR_ING_CHAN_CFG_DROP_PKT, 
                      pTopPsrConfigSetup->drop_pkt[i]) |
                  CSL_FMK(IQN2_TOP_PSR_ING_CHAN_CFG_FORCE_FLUSH, 
                      pTopPsrConfigSetup->force_flush[i]);
        hIqn2->regs->Top.PSR_CONFIG_REGS.PSR_ING_CHAN_CFG[i] = tempReg;
    }

    /* Setup PSR EGR CHAN CFG */
    for (i = 0; i < 48; i++)
    {
        tempReg = CSL_FMK(IQN2_TOP_PSR_EGR_CHAN_CFG_PACK_PS_DATA, 
                      pTopPsrConfigSetup->pack_ps_data[i]) |
                  CSL_FMK(IQN2_TOP_PSR_EGR_CHAN_CFG_ARB_PRIORITY, 
                      pTopPsrConfigSetup->arb_priority[i]);
        hIqn2->regs->Top.PSR_CONFIG_REGS.PSR_EGR_CHAN_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupTopRegs
 *
 *   @b Description
 *   @n IQN2 Top registers setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            topSetup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupTopVcSysStsCfgRegs, Iqn2Fl_setupTopPsrCfgRegs
 *
 *   @b Writes
 *   @n VC_SYS_STS_CFG, VC_SD_LK, PSR_CONFIG_REGS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupTopRegs (hIqn2, topSetup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupTopRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_TopSetup *topSetup
)
{
    /** pointer to Top registers setup */
    Iqn2Fl_TopSetup   *pTopConfig;

    pTopConfig = topSetup;

    /* Setup Top VC Sys Status Config */
    Iqn2Fl_setupTopVcSysStsCfgRegs (hIqn2, &(pTopConfig->top_vc_sys_sts_cfg));

    /* Setup Top PSR Config */
    Iqn2Fl_setupTopPsrCfgRegs (hIqn2, &(pTopConfig->top_psr_cfg));

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqEfeCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 AID2 EFE CFG GRP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqEfeCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_SI_IQ_EFE_CONFIG_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqEfeCfgGrpRegs (hIqn2, &aid2_iq_efe_cfg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqEfeCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqEfeCfgGrpSetup *pAid2IqEfeCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup EFE CHAN CFG */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_EN, 
                      pAid2IqEfeCfgGrpSetup->chan_en[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                      pAid2IqEfeCfgGrpSetup->chan_tdd_frc_off[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_RADIO_SEL, 
                      pAid2IqEfeCfgGrpSetup->chan_radio_sel[i]);
        hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CHAN_CFG[i] = tempReg;
    }

    /* Setup EFE CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CFG, 
        IQN_AID2_AID2_IQ_EFE_CFG_LOOPBACK_EN, 
        pAid2IqEfeCfgGrpSetup->loopback_en);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqEfeRadioStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 AID2 EFE RADIO STANDARD GRP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqEfeRadioStdGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqEfeRadioStdGrpRegs (hIqn2, &aid2_iq_efe_radio_std_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqEfeRadioStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqEfeRadioStdGrpSetup *pAid2IqEfeRadioStdGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup EFE FRM TC CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EFE_FRM_TC_CFG_SYM_TC, 
                      pAid2IqEfeRadioStdGrpSetup->sym_tc[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_FRM_TC_CFG_INDEX_SC, 
                      pAid2IqEfeRadioStdGrpSetup->index_sc[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_FRM_TC_CFG_INDEX_TC, 
                      pAid2IqEfeRadioStdGrpSetup->index_tc[i]);
        hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup EFE RAD STD CFG*/
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EFE_RAD_STD_CFG_TDD_FIRST_SYM, 
                      pAid2IqEfeRadioStdGrpSetup->tdd_first_sym[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_RAD_STD_CFG_TDD_LUT_EN, 
                      pAid2IqEfeRadioStdGrpSetup->tdd_lut_en[i]);
        hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_RAD_STD_CFG[i] = tempReg;
    }

    /* Setup EFE TDD EN CFG0/7 - TDD_EN */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG0[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG0_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[0][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG1[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG1_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[1][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG2[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG2_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[2][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG3[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG3_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[3][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG4[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG4_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[4][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG5[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG5_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[5][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG6[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG6_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[6][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_TDD_EN_CFG7[i], 
            IQN_AID2_AID2_IQ_EFE_TDD_EN_CFG7_TDD_EN, 
            pAid2IqEfeRadioStdGrpSetup->tdd_en_cfg[7][i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqIfeChanCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 AID2 IFE CHANNEL CONFIGURATION GRP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqIfeChanCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqIfeChanCfgGrpRegs (hIqn2, &aid2_iq_ife_chan_cfg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqIfeChanCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqIfeChanCfgGrpSetup *pAid2IqIfeChanCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IFE CHAN CFG */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_EN, 
                      pAid2IqIfeChanCfgGrpSetup[i].chan_en) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_AXC_OFFSET, 
                      pAid2IqIfeChanCfgGrpSetup[i].chan_axc_offset) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_RADIO_SEL, 
                      pAid2IqIfeChanCfgGrpSetup[i].chan_radio_sel) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                      pAid2IqIfeChanCfgGrpSetup[i].chan_tdd_frc_off);
        hIqn2->regs->Aid2.AID2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.AID2_IQ_IFE_CHAN_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqIfeRadioStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 AID2 IFE RADIO STANDARD GRP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqIfeRadioStdGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqIfeRadioStdGrpRegs (hIqn2, &aid2_iq_ife_radio_std_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqIfeRadioStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqIfeRadioStdGrpSetup *pAid2IqIfeRadioStdGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IFE FRM TC CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_IFE_FRM_TC_CFG_SYM_TC, 
                      pAid2IqIfeRadioStdGrpSetup->sym_tc[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IFE_FRM_TC_CFG_INDEX_SC, 
                      pAid2IqIfeRadioStdGrpSetup->index_sc[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IFE_FRM_TC_CFG_INDEX_TC, 
                      pAid2IqIfeRadioStdGrpSetup->index_tc[i]);
        hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup IFE RAD STD CFG */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_RAD_STD_CFG[i], 
            IQN_AID2_AID2_IQ_IFE_RAD_STD_CFG_TDD_LUT_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_lut_en[i]);
    }

    /* Setup EFE TDD EN CFG0/7 - TDD_EN */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG0[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG0_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[0][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG1[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG1_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[1][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG2[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG2_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[2][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG3[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG3_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[3][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG4[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG4_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[4][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG5[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG5_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[5][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG6[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG6_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[6][i]);

        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_RADIO_STANDARD_GROUP.AID2_IQ_IFE_TDD_EN_CFG7[i], 
            IQN_AID2_AID2_IQ_IFE_TDD_EN_CFG7_TDD_EN, 
            pAid2IqIfeRadioStdGrpSetup->tdd_en_cfg[7][i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqIdcChanCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 AID2 IDC CHANNEL CONFIGURATION GRP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqIngChanCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_IQ_IDC_CHANNEL_CONFIG_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqIdcChanCfgGrpRegs (hIqn2, &aid2_iq_idc_chan_cfg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqIdcChanCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqIngChanCfgGrpSetup *pAid2IqIngChanCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AID2 IQ IDC CHANNEL CONFIG GROUP */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_IDC_CH_CFG_DAT_SWAP, 
                      pAid2IqIngChanCfgGrpSetup[i].dat_swap) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IDC_CH_CFG_IQ_ORDER, 
                      pAid2IqIngChanCfgGrpSetup[i].iq_order) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IDC_CH_CFG_PKT_TYPE, 
                      pAid2IqIngChanCfgGrpSetup[i].pkt_type) |
                  CSL_FMK(IQN_AID2_AID2_IQ_IDC_CH_CFG_CHAN_FRC_OFF, 
                      pAid2IqIngChanCfgGrpSetup[i].chan_frc_off);
        hIqn2->regs->Aid2.AID2_IQ_IDC_CHANNEL_CONFIG_GROUP.AID2_IQ_IDC_CH_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqIctlIdcIfRegs
 *
 *   @b Description
 *   @n IQN2 AID2 ICTL IDC IF registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqIngChanCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_ICTL_IDC_IF
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqIctlIdcIfRegs (hIqn2, &aid2_ictl_idc_if_ch_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqIctlIdcIfRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqIngChanCfgGrpSetup *pAid2IqIngChanCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup AID2 ICTL IDC IF */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_ICTL_CH_CFG_DAT_SWAP, 
                      pAid2IqIngChanCfgGrpSetup[i].dat_swap) |
                  CSL_FMK(IQN_AID2_AID2_ICTL_CH_CFG_IQ_ORDER, 
                      pAid2IqIngChanCfgGrpSetup[i].iq_order) |
                  CSL_FMK(IQN_AID2_AID2_ICTL_CH_CFG_PKT_TYPE, 
                      pAid2IqIngChanCfgGrpSetup[i].pkt_type) |
                  CSL_FMK(IQN_AID2_AID2_ICTL_CH_CFG_CHAN_FRC_OFF, 
                      pAid2IqIngChanCfgGrpSetup[i].chan_frc_off);
        hIqn2->regs->Aid2.AID2_ICTL_IDC_IF.AID2_ICTL_CH_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2IqUatGenCtlRegs
 *
 *   @b Description
 *   @n IQN2 AID2 UAT GEN CTL registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAid2IqUatGenCtlSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_UAT_GEN_CTL
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2IqUatGenCtlRegs (hIqn2, &aid2_iq_uat_gen_ctl);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2IqUatGenCtlRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2IqUatGenCtlSetup *pAid2IqUatGenCtlSetup
)
{
    /* Setup AID2 UAT BCN TC CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_GEN_CTL.AID2_UAT_BCN_TC_CFG, 
        IQN_AID2_AID2_UAT_BCN_TC_CFG_VAL, 
        pAid2IqUatGenCtlSetup->bcn_tc_cfg_val);

    /* Setup AID2 UAT BCN OFFSET CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_GEN_CTL.AID2_UAT_BCN_OFFSET_CFG, 
        IQN_AID2_AID2_UAT_BCN_OFFSET_CFG_VAL, 
        pAid2IqUatGenCtlSetup->bcn_offset_cfg_val);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2Regs
 *
 *   @b Description
 *   @n IQN2 AID2 registers setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            aid2Setup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAid2IqEfeCfgGrpRegs, Iqn2Fl_setupAid2IqEfeRadioStdGrpRegs,
 *      Iqn2Fl_setupAid2IqIfeChanCfgGrpRegs, Iqn2Fl_setupAid2IqIfeRadioStdGrpRegs,
 *      Iqn2Fl_setupAid2IqIdcChanCfgGrpRegs, Iqn2Fl_setupAid2IqIctlIdcIfRegs,
 *      Iqn2Fl_setupAid2IqUatGenCtlRegs
 *
 *   @b Writes
 *   @n AID2_SI_IQ_EFE_CONFIG_GROUP, AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP, 
 *      AID2_IQ_EFE_CHAN_AXC_OFFSET, AID2_IQ_EFE_FRM_SAMP_TC_MMR_RAM, 
 *      AID2_IQ_EFE_CHAN_TDM_LUT, AID2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP,
 *      AID2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP, AID2_IQ_IFE_RADIO_STANDARD_GROUP,
 *      AID2_IQ_IFE_CONFIG_GROUP, AID2_IQ_IDC_CONFIGURATION_GROUP, 
 *      AID2_IQ_IDC_CHANNEL_CONFIG_GROUP, AID2_IFE_FRM_SAMP_TC_MMR_RAM, 
 *      AID2_ECTL_PKT_IF, AID2_ICTL_IDC_IF, AID2_ICTL_PKT_IF, AID2_UAT_GEN_CTL, 
 *      AID2_UAT_EGR_RADT, AID2_UAT_ING_RADT, AID2_UAT_RADT_EVT, 
 *      AID2_IQ_EDC_REGISTER_GROUP, AID2_IQ_INGRESS_VBUS_MMR_GROUP, 
 *      AID2_ECTL_REGISTER_GROUP, AID2_CTL_INGRESS_VBUS_MMR_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2Regs (hIqn2, aid2Setup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Aid2Setup *aid2Setup
)
{
    /* pointer to AID2 registers setup */
    Iqn2Fl_Aid2Setup   *pAid2Config;
    uint32_t   tempReg, i;

    pAid2Config = aid2Setup;

    /* Setup AID2 SI IQ EFE CONFIG GROUP */
    Iqn2Fl_setupAid2IqEfeCfgGrpRegs (hIqn2, &(pAid2Config->aid2_iq_efe_cfg_grp));

    /* Setup AID2 SI IQ EFE RADIO STANDARD GROUP */
    Iqn2Fl_setupAid2IqEfeRadioStdGrpRegs (hIqn2, &(pAid2Config->aid2_iq_efe_radio_std_grp));

    /* Setup AID2 IQ EFE CHAN AXC OFFSET */
    for (i = 0; i < 32; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EFE_CHAN_AXC_OFFSET[i].AID2_IQ_EFE_CHAN_AXC_OFFSET_CFG, 
            IQN_AID2_AID2_IQ_EFE_CHAN_AXC_OFFSET_CFG_AXC_OFFSET, 
            pAid2Config->efe_axc_offset[i]);
    }

    /* Setup AID2 IQ EFE FRM SAMP TC MMR RAM - EFE FRM SAMP TC CFG */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EFE_FRM_SAMP_TC_MMR_RAM[i].AID2_IQ_EFE_FRM_SAMP_TC_CFG, 
            IQN_AID2_AID2_IQ_EFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pAid2Config->efe_samp_tc[i]);
    }

    /* Setup AID2 IQ EFE CHAN TDM LUT - EFE CHAN TDM LUT CFG */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EFE_CHAN_TDM_LUT_CFG_CHAN_INDEX_CFG, 
                      pAid2Config->efe_chan_index_cfg[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_CHAN_TDM_LUT_CFG_CHAN_INDEX_EN_CFG, 
                      pAid2Config->efe_chan_index_en_cfg[i]);
        hIqn2->regs->Aid2.AID2_IQ_EFE_CHAN_TDM_LUT[i].AID2_IQ_EFE_CHAN_TDM_LUT_CFG = tempReg;
    }

    /* Setup AID2 IQ EFE RADIO STANDARD SCHEDULER GROUP - EFE RAD STD SCH CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EFE_RAD_STD_SCH_CFG_TDM_START, 
                      pAid2Config->efe_tdm_start[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_RAD_STD_SCH_CFG_TDM_LEN, 
                      pAid2Config->efe_tdm_len[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EFE_RAD_STD_SCH_CFG_TDM_EN, 
                      pAid2Config->efe_tdm_en[i]);
        hIqn2->regs->Aid2.AID2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP.AID2_IQ_EFE_RAD_STD_SCH_CFG[i] = tempReg;
    }

    /* Setup AID2 IQ IFE CHANNEL CONFIGURATION GROUP */
    Iqn2Fl_setupAid2IqIfeChanCfgGrpRegs (hIqn2, &(pAid2Config->aid2_iq_ife_chan_cfg_grp[0]));

    /* Setup AID2 IQ IFE RADIO STANDARD GROUP */
    Iqn2Fl_setupAid2IqIfeRadioStdGrpRegs (hIqn2, &(pAid2Config->aid2_iq_ife_radio_std_grp));

    /* Setup AID2 IQ IDC CONFIGURATION GROUP - IDC CFG */
    tempReg = CSL_FMK(IQN_AID2_AID2_IQ_IDC_CFG_FAIL_MARK_ONLY, 
                  pAid2Config->idc_fail_mark_only) |
              CSL_FMK(IQN_AID2_AID2_IQ_IDC_CFG_FRC_OFF_ALL, 
                  pAid2Config->idc_frc_off_all);
    hIqn2->regs->Aid2.AID2_IQ_IDC_CONFIGURATION_GROUP.AID2_IQ_IDC_CFG = tempReg;

    /* Setup AID2 IQ IDC CHANNEL CONFIG GROUP */
    Iqn2Fl_setupAid2IqIdcChanCfgGrpRegs (hIqn2, &(pAid2Config->aid2_iq_idc_chan_cfg_grp[0]));

    /* Setup AID2 IFE FRM SAMP TC MMR RAM - IFE FRM SAMP TC CFG */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IFE_FRM_SAMP_TC_MMR_RAM[i].AID2_IQ_IFE_FRM_SAMP_TC_CFG, 
            IQN_AID2_AID2_IQ_IFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pAid2Config->ife_samp_tc[i]);
    }

    /* Setup AID2 ECTL PKT IF */
    for (i = 0; i < 16; i++)
    {
        /* ECTL CHAN CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_CHAN_CFG[i], 
            IQN_AID2_AID2_ECTL_CHAN_CFG_CHAN_EN, 
            pAid2Config->ectl_chan_en[i]);
        /* ECTL DB THOLD CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_DB_THOLD_CFG[i], 
            IQN_AID2_AID2_ECTL_DB_THOLD_CFG_CHANNEL, 
            pAid2Config->ectl_channel[i]);
    }

    /* Setup AID2 ICTL IDC IF */
    Iqn2Fl_setupAid2IqIctlIdcIfRegs (hIqn2, &(pAid2Config->aid2_ictl_idc_if_ch_cfg[0]));

    /* Setup AID2 ICTL IDC IF - ICTL CFG */
    tempReg = CSL_FMK(IQN_AID2_AID2_ICTL_CFG_FAIL_MARK_ONLY, 
                  pAid2Config->aid2_ictl_idc_if_ictl_cfg_fail_mark_only) |
              CSL_FMK(IQN_AID2_AID2_ICTL_CFG_FRC_OFF_ALL, 
                  pAid2Config->aid2_ictl_idc_if_ictl_cfg_force_off_all);
    hIqn2->regs->Aid2.AID2_ICTL_IDC_IF.AID2_ICTL_CFG = tempReg;

    /* Setup AID2 ICTL PKT IF - ICTL CHAN CFG */
    for (i = 0; i < 16; i++)
    {
        CSL_FINS(hIqn2->regs->Aid2.AID2_ICTL_PKT_IF.AID2_ICTL_CHAN_EN_CFG[i], 
            IQN_AID2_AID2_ICTL_CHAN_EN_CFG_CHAN_EN, 
            pAid2Config->ictl_pkt_if_chan_en[i]);
    }

    /* Setup AID2 UAT GEN CTL */
    Iqn2Fl_setupAid2IqUatGenCtlRegs (hIqn2, &(pAid2Config->aid2_iq_uat_gen_ctl));

    /* Setup AID2 UAT EGR RADT */
    for (i = 0; i < 8; i++)
    {
        /* TC CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_EGR_RADT[i].AID2_UAT_EGR_RADT_TC_CFG, 
            IQN_AID2_AID2_UAT_EGR_RADT_TC_CFG_VAL, 
            pAid2Config->uat_egr_radt_tc_cfg_val[i]);
        /* OFFSET CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_EGR_RADT[i].AID2_UAT_EGR_RADT_OFFSET_CFG, 
            IQN_AID2_AID2_UAT_EGR_RADT_OFFSET_CFG_VAL, 
            pAid2Config->uat_egr_radt_offset_cfg_val[i]);
    }

    /* Setup AID2 UAT ING RADT */
    for (i = 0; i < 8; i++)
    {
        /* TC CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_ING_RADT[i].AID2_UAT_ING_RADT_TC_CFG, 
            IQN_AID2_AID2_UAT_ING_RADT_TC_CFG_VAL, 
            pAid2Config->uat_ing_radt_tc_cfg_val[i]);
        /* OFFSET CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_ING_RADT[i].AID2_UAT_ING_RADT_OFFSET_CFG, 
            IQN_AID2_AID2_UAT_ING_RADT_OFFSET_CFG_VAL, 
            pAid2Config->uat_ing_radt_offset_cfg_val[i]);
    }

    /* Setup AID2 UAT RADT EVT */
    for (i = 0; i < 22; i++)
    {
        /* EVT CMP CFG */
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_RADT_EVT[i].AID2_UAT_EVT_RADT_CMP_CFG, 
            IQN_AID2_AID2_UAT_EVT_RADT_CMP_CFG_VAL, 
            pAid2Config->uat_evt_radt_cmp_cfg_val[i]);
        /* EVT CLK CNT TC CFG*/
        CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_RADT_EVT[i].AID2_UAT_EVT_CLK_CNT_TC_CFG, 
            IQN_AID2_AID2_UAT_EVT_CLK_CNT_TC_CFG_VAL, 
            pAid2Config->uat_evt_clk_cnt_tc_cfg_val[i]);
    }

    /* Setup AID2 IQ EDC REGISTER GROUP - EDC CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_CFG, 
        IQN_AID2_AID2_IQ_EDC_CFG_PSI_ERR_CHK_DISABLE, 
        pAid2Config->edc_cfg_psi_err_chk_disable);

    /* Setup AID2 IQ EDC REGISTER GROUP - EDC CH CFG */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_IQ_EDC_CH_CFG_DAT_SWAP, 
                      pAid2Config->edc_ch_cfg_dat_swap[i]) |
                  CSL_FMK(IQN_AID2_AID2_IQ_EDC_CH_CFG_IQ_ORDER, 
                      pAid2Config->edc_ch_cfg_iq_order[i]);
        hIqn2->regs->Aid2.AID2_IQ_EDC_REGISTER_GROUP.AID2_IQ_EDC_CH_CFG[i] = tempReg;
    }

    /* Setup AID2 IQ INGRESS VBUS MMR GROUP
     * Default value is 15. Setting this value to a non-default value is possible if 0 < idc_rate_ctl_cfg_rate <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_IQ_IDC_RATE_CTL_REG CMD word
     */
    if (pAid2Config->idc_rate_ctl_cfg_rate) {
        CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_RATE_CTL_CFG,
            IQN_AID2_AID2_IQ_IDC_RATE_CTL_CFG_RATE,
            pAid2Config->idc_rate_ctl_cfg_rate);
    }

    /* Setup AID2 ECTL REGISTER GROUP - ECTL RATE CTL CFG
     * Default value is 15. Setting this value to a non-default value is possible if 0 < ectl_rate_ctl_cfg <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_ECTL_RATE_CTL_CFG_REG CMD word
     */
    if (pAid2Config->ectl_rate_ctl_cfg) {
        CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_REGISTER_GROUP.AID2_ECTL_RATE_CTL_CFG,
        IQN_AID2_AID2_ECTL_RATE_CTL_CFG_RATE, 
        pAid2Config->ectl_rate_ctl_cfg);
    }

    /* Setup AID2 IQ EDC REGISTER GROUP - EDC CH CFG */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_AID2_AID2_ECTL_CH_CFG_DAT_SWAP, 
                      pAid2Config->ectl_ch_cfg_dat_swap[i]) |
                  CSL_FMK(IQN_AID2_AID2_ECTL_CH_CFG_IQ_ORDER, 
                      pAid2Config->ectl_ch_cfg_iq_order[i]);
        hIqn2->regs->Aid2.AID2_ECTL_REGISTER_GROUP.AID2_ECTL_CH_CFG[i] = tempReg;
    }

    /* Setup AID2 CTL INGRESS VBUS MMR GROUP - ICTL RATE CTL CFG
     * Default value is 15. Setting this value to a non-default value is possible if 0 < ictl_rate_ctl_cfg_rate <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_AID2_ICTL_RATE_CTL_CFG_REG CMD word
     */
    if (pAid2Config->ictl_rate_ctl_cfg_rate) {
        CSL_FINS(hIqn2->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_RATE_CTL_CFG,
        IQN_AID2_AID2_ICTL_RATE_CTL_CFG_RATE, 
        pAid2Config->ictl_rate_ctl_cfg_rate);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupIqsIngressCfgRegs
 *
 *   @b Description
 *   @n IQN2 IQS Ingress Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pIqsIngressCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n IQS_INGRESS_CONFIG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupIqsIngressCfgRegs (hIqn2, &iqs2_ingress_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupIqsIngressCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_IqsIngressCfgSetup *pIqsIngressCfgSetup
)
{
    /* Setup IQS ING PKTDMA CFG - PB_SEL */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_PKTDMA_CFG, 
        IQN_IQS2_IQS_ING_PKTDMA_CFG_PB_SEL, 
        pIqsIngressCfgSetup->pktdma_cfg_pb_sel);

    /* Setup IQS ING AID2 AXC CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AID2_AXC_CFG, 
        IQN_IQS2_IQS_ING_AID2_AXC_CFG_PRI, 
        pIqsIngressCfgSetup->aid2_axc_cfg_pri);

    /* Setup IQS ING AID2 AXC CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AID2_AXC_CFG, 
        IQN_IQS2_IQS_ING_AID2_AXC_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->aid2_axc_cfg_allow_pushback);

    /* Setup IQS ING AID2 CTL CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AID2_CTL_CFG, 
        IQN_IQS2_IQS_ING_AID2_CTL_CFG_PRI, 
        pIqsIngressCfgSetup->aid2_ctl_cfg_pri);

    /* Setup IQS ING AID2 CTL CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AID2_CTL_CFG, 
        IQN_IQS2_IQS_ING_AID2_CTL_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->aid2_ctl_cfg_allow_pushback);

    /* Setup IQS ING AIL0 AXC CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL0_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL0_AXC_CFG_PRI, 
        pIqsIngressCfgSetup->ail0_axc_cfg_pri);

    /* Setup IQS ING AIL0 AXC CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL0_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL0_AXC_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail0_axc_cfg_allow_pushback);

    /* Setup IQS ING AIL0 CTL CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL0_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL0_CTL_CFG_PRI, 
        pIqsIngressCfgSetup->ail0_ctl_cfg_pri);

    /* Setup IQS ING AIL0 CTL CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL0_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL0_CTL_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail0_ctl_cfg_allow_pushback);

    /* Setup IQS ING AIL1 AXC CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL1_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL1_AXC_CFG_PRI, 
        pIqsIngressCfgSetup->ail1_axc_cfg_pri);

    /* Setup IQS ING AIL1 AXC CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL1_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL1_AXC_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail1_axc_cfg_allow_pushback);

    /* Setup IQS ING AIL1 CTL CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL1_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL1_CTL_CFG_PRI, 
        pIqsIngressCfgSetup->ail1_ctl_cfg_pri);

    /* Setup IQS ING AIL1 CTL CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL1_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL1_CTL_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail1_ctl_cfg_allow_pushback);

    /* Setup IQS ING AIL2 AXC CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL2_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL2_AXC_CFG_PRI, 
        pIqsIngressCfgSetup->ail2_axc_cfg_pri);

    /* Setup IQS ING AIL2 AXC CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL2_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL2_AXC_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail2_axc_cfg_allow_pushback);

    /* Setup IQS ING AIL2 CTL CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL2_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL2_CTL_CFG_PRI, 
        pIqsIngressCfgSetup->ail2_ctl_cfg_pri);

    /* Setup IQS ING AIL2 CTL CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL2_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL2_CTL_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail2_ctl_cfg_allow_pushback);

    /* Setup IQS ING AIL3 AXC CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL3_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL3_AXC_CFG_PRI, 
        pIqsIngressCfgSetup->ail3_axc_cfg_pri);

    /* Setup IQS ING AIL3 AXC CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL3_AXC_CFG, 
        IQN_IQS2_IQS_ING_AIL3_AXC_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail3_axc_cfg_allow_pushback);

    /* Setup IQS ING AIL3 CTL CFG - PRI */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL3_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL3_CTL_CFG_PRI, 
        pIqsIngressCfgSetup->ail3_ctl_cfg_pri);

    /* Setup IQS ING AIL3 CTL CFG - ALLOW_PUSHBACK */
    CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CONFIG.IQS_ING_AIL3_CTL_CFG, 
        IQN_IQS2_IQS_ING_AIL3_CTL_CFG_ALLOW_PUSHBACK, 
        pIqsIngressCfgSetup->ail3_ctl_cfg_allow_pushback);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupIqsIngressChanCfgRegs
 *
 *   @b Description
 *   @n IQN2 IQS Ingress Channel Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pIqsIngressChanCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n IQS_INGRESS_CHAN_CONFIG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupIqsIngressChanCfgRegs (hIqn2, &iqs2_ingress_chan_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupIqsIngressChanCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_IqsIngressChanCfgSetup *pIqsIngressChanCfgSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IQS ING DIO2 PSI CFG */
    for (i = 0; i < 16; i++)
    {
        CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_DIO2_PSI_CFG[i], 
            IQN_IQS2_IQS_ING_DIO2_PSI_CFG_PRI, 
            pIqsIngressChanCfgSetup->dio2_psi_cfg_pri[i]);
    }

    /* Setup IQS ING AID2 AXC LUT CFG */
    for (i = 0; i < 32; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AID2_AXC_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->aid2_axc_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AID2_AXC_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->aid2_axc_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AID2_AXC_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AID2 CTL LUT CFG */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AID2_CTL_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->aid2_ctl_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AID2_CTL_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->aid2_ctl_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AID2_CTL_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL0 AXC LUT CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL0_AXC_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail0_axc_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL0_AXC_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail0_axc_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL0_AXC_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL0 CTL LUT CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL0_CTL_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail0_ctl_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL0_CTL_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail0_ctl_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL0_CTL_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL1 AXC LUT CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL1_AXC_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail1_axc_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL1_AXC_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail1_axc_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL1_AXC_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL1 CTL LUT CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL1_CTL_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail1_ctl_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL1_CTL_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail1_ctl_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL1_CTL_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL2 AXC LUT CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL2_AXC_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail2_axc_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL2_AXC_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail2_axc_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL2_AXC_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL2 CTL LUT CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL2_CTL_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail2_ctl_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL2_CTL_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail2_ctl_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL2_CTL_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL3 AXC LUT CFG */
    for (i = 0; i < 64; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL3_AXC_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail3_axc_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL3_AXC_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail3_axc_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL3_AXC_LUT_CFG[i] = tempReg;
    }

    /* Setup IQS ING AIL3 CTL LUT CFG */
    for (i = 0; i < 4; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_ING_AIL3_CTL_LUT_CFG_CHAN, 
                      pIqsIngressChanCfgSetup->ail3_ctl_lut_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_ING_AIL3_CTL_LUT_CFG_DEST, 
                      pIqsIngressChanCfgSetup->ail3_ctl_lut_cfg[i].dest);
        hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AIL3_CTL_LUT_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupIqsEgressChanCfgRegs
 *
 *   @b Description
 *   @n IQN2 IQS Egress Channel Config registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pIqsEgressChanCfgSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n IQS_EGRESS_CHAN_CONFIG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupIqsEgressChanCfgRegs (hIqn2, &iqs2_egress_chan_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupIqsEgressChanCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_IqsEgressChanCfgSetup *pIqsEgressChanCfgSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IQS EGR PKTDMA CFG */
    for (i = 0; i < 48; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_EGR_PKTDMA_CFG_CHAN, 
                      pIqsEgressChanCfgSetup->egr_pktdma_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_PKTDMA_CFG_DEST, 
                      pIqsEgressChanCfgSetup->egr_pktdma_cfg[i].dest) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_PKTDMA_CFG_ARB_PRI, 
                      pIqsEgressChanCfgSetup->egr_pktdma_cfg[i].arb_pri) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_PKTDMA_CFG_PSI_PRI, 
                      pIqsEgressChanCfgSetup->egr_pktdma_cfg[i].psi_pri);
        hIqn2->regs->Iqs2.IQS_EGRESS_CHAN_CONFIG.IQS_EGR_PKTDMA_CFG[i] = tempReg;
    }

    /* Setup IQS EGR DIO2 CFG */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_IQS2_IQS_EGR_DIO2_CFG_CHAN, 
                      pIqsEgressChanCfgSetup->egr_dio2_cfg[i].chan) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_DIO2_CFG_DEST, 
                      pIqsEgressChanCfgSetup->egr_dio2_cfg[i].dest) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_DIO2_CFG_ARB_PRI, 
                      pIqsEgressChanCfgSetup->egr_dio2_cfg[i].arb_pri) |
                  CSL_FMK(IQN_IQS2_IQS_EGR_DIO2_CFG_PSI_PRI, 
                      pIqsEgressChanCfgSetup->egr_dio2_cfg[i].psi_pri);
        hIqn2->regs->Iqs2.IQS_EGRESS_CHAN_CONFIG.IQS_EGR_DIO2_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupIqs2Regs
 *
 *   @b Description
 *   @n IQN2 IQS2 registers setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            iqs2Setup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupIqsIngressCfgRegs, Iqn2Fl_setupIqsIngressChanCfgRegs,
 *      Iqn2Fl_setupIqsEgressChanCfgRegs
 *
 *   @b Writes
 *   @n IQS_INGRESS_CONFIG, IQS_INGRESS_CHAN_CONFIG, IQS_EGRESS_CHAN_CONFIG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupIqs2Regs (hIqn2, iqs2Setup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupIqs2Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Iqs2Setup *iqs2Setup
)
{
    /* pointer to IQS2 registers setup */
    Iqn2Fl_Iqs2Setup   *pIqs2Config;

    pIqs2Config = iqs2Setup;

    /* Setup IQS INGRESS CONFIG */
    Iqn2Fl_setupIqsIngressCfgRegs  (hIqn2, &(pIqs2Config->iqs2_ingress_cfg));

    /* Setup IQS INGRESS CHAN CONFIG */
    Iqn2Fl_setupIqsIngressChanCfgRegs (hIqn2, &(pIqs2Config->iqs2_ingress_chan_cfg));

    /* Setup IQS EGRESS CHAN CONFIG */
    Iqn2Fl_setupIqsEgressChanCfgRegs (hIqn2, &(pIqs2Config->iqs2_egress_chan_cfg));

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2SiIqEfeCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 SI IQ EFE CONFIG GROUP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2SiIqEfeCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_SI_IQ_EFE_CONFIG_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2SiIqEfeCfgGrpRegs (hIqn2, &dio2_efe_cfg_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2SiIqEfeCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2SiIqEfeCfgGrpSetup *pDio2SiIqEfeCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup EFE CHAN CFG */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_EN, 
                      pDio2SiIqEfeCfgGrpSetup->chan_en[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                      pDio2SiIqEfeCfgGrpSetup->chan_tdd_frc_off[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_RADIO_SEL, 
                      pDio2SiIqEfeCfgGrpSetup->chan_radio_sel[i]);
        hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_CHAN_CFG[i] = tempReg;
    }

    /* Setup EFE CFG - LOOPBACK_EN */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_CFG, 
        IQN_DIO2_DIO2_IQ_EFE_CFG_LOOPBACK_EN, 
        pDio2SiIqEfeCfgGrpSetup->loopback_en);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2SiIqEfeRadioStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 SI IQ EFE RADIO STANDARD GROUP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2SiIqEfeRadioStdGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2SiIqEfeRadioStdGrpRegs (hIqn2, &dio2_efe_radio_std_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2SiIqEfeRadioStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2SiIqEfeRadioStdGrpSetup *pDio2SiIqEfeRadioStdGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup EFE FRM TC CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_FRM_TC_CFG_SYM_TC, 
                      pDio2SiIqEfeRadioStdGrpSetup->efe_frm_tc_cfg[i].sym_tc) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_FRM_TC_CFG_INDEX_SC, 
                      pDio2SiIqEfeRadioStdGrpSetup->efe_frm_tc_cfg[i].index_sc) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_FRM_TC_CFG_INDEX_TC, 
                      pDio2SiIqEfeRadioStdGrpSetup->efe_frm_tc_cfg[i].index_tc);
        hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup EFE RAD STD CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_RAD_STD_CFG_TDD_FIRST_SYM, 
                      pDio2SiIqEfeRadioStdGrpSetup->efe_rad_std_cfg_tdd_first_sym[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_RAD_STD_CFG_TDD_LUT_EN, 
                      pDio2SiIqEfeRadioStdGrpSetup->efe_rad_std_cfg_tdd_lut_en[i]);
        hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_RAD_STD_CFG[i] = tempReg;
    }

    /* Setup EFE TDD EN CFG0/7 - TDD_EN */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG0[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG0_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[0][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG1[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG1_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[1][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG2[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG2_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[2][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG3[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG3_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[3][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG4[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG4_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[4][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG5[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG5_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[5][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG6[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG6_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[6][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP.DIO2_IQ_EFE_TDD_EN_CFG7[i], 
            IQN_DIO2_DIO2_IQ_EFE_TDD_EN_CFG7_TDD_EN, 
            pDio2SiIqEfeRadioStdGrpSetup->efe_tdd_en_cfg_tdd_en[7][i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2IqIfeChanCfgGrpRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 IQ IFE CHANNEL CONFIGURATION GROUP setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2IqIfeChanCfgGrpSetup   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2IqIfeChanCfgGrpRegs (hIqn2, &dio2_ife_chan_cfg_grp[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2IqIfeChanCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2IqIfeChanCfgGrpSetup *pDio2IqIfeChanCfgGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup DIO2 IQ IFE CHANNEL CONFIGURATION GROUP */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_CHAN_CFG_CHAN_EN, 
                      pDio2IqIfeChanCfgGrpSetup[i].chan_en) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_CHAN_CFG_CHAN_AXC_OFFSET, 
                      pDio2IqIfeChanCfgGrpSetup[i].chan_axc_offset) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_CHAN_CFG_CHAN_RADIO_SEL, 
                      pDio2IqIfeChanCfgGrpSetup[i].chan_radio_sel) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_CHAN_CFG_CHAN_TDD_FRC_OFF, 
                      pDio2IqIfeChanCfgGrpSetup[i].chan_tdd_frc_off);
        hIqn2->regs->Dio2.DIO2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.DIO2_IQ_IFE_CHAN_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2IqIfeRadioStdGrpRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 IQ IFE RADIO STANDARD GROUP registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2IqIfeRadioStdGrpSetup   Pointer containing "Setup" properties for IQN2. 

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_IQ_IFE_RADIO_STANDARD_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2IqIfeRadioStdGrpRegs (hIqn2, &dio2_ife_radio_std_grp);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2IqIfeRadioStdGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2IqIfeRadioStdGrpSetup *pDio2IqIfeRadioStdGrpSetup
)
{
    uint32_t   tempReg, i;

    /* Setup IFE FRM TC CFG */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_FRM_TC_CFG_SYM_TC, 
                      pDio2IqIfeRadioStdGrpSetup->ife_frm_tc_cfg[i].sym_tc) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_FRM_TC_CFG_INDEX_SC, 
                      pDio2IqIfeRadioStdGrpSetup->ife_frm_tc_cfg[i].index_sc) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IFE_FRM_TC_CFG_INDEX_TC, 
                      pDio2IqIfeRadioStdGrpSetup->ife_frm_tc_cfg[i].index_tc);
        hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_FRM_TC_CFG[i] = tempReg;
    }

    /* Setup IFE RAD STD CFG - TDD_LUT_EN */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_RAD_STD_CFG[i], 
            IQN_DIO2_DIO2_IQ_IFE_RAD_STD_CFG_TDD_LUT_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_rad_std_cfg_tdd_lut_en[i]);
    }

    /* Setup EFE TDD EN CFG0/7 - TDD_EN */
    for (i = 0; i < 5; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG0[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG0_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[0][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG1[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG1_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[1][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG2[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG2_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[2][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG3[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG3_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[3][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG4[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG4_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[4][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG5[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG5_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[5][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG6[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG6_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[6][i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_RADIO_STANDARD_GROUP.DIO2_IQ_IFE_TDD_EN_CFG7[i], 
            IQN_DIO2_DIO2_IQ_IFE_TDD_EN_CFG7_TDD_EN, 
            pDio2IqIfeRadioStdGrpSetup->ife_tdd_en_cfg_tdd_en[7][i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2IqIdcChCfgGrpRegs
 *
 *   @b Description
 *   @n DIO2 IQ IDC CHANNEL CONFIG GROUP setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2IqIdcChCfgGrp   Pointer containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_IQ_IDC_CHANNEL_CONFIG_GROUP
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2IqIdcChCfgGrpRegs (hIqn2, &dio2_idc_ch_cfg_grp[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2IqIdcChCfgGrpRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2IqIdcChCfgGrp *pDio2IqIdcChCfgGrp
)
{
    uint32_t   tempReg, i;

    /* Setup DIO2 IQ IDC CHANNEL CONFIG GROUP */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CH_CFG_DAT_SWAP, 
                      pDio2IqIdcChCfgGrp[i].dat_swap) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CH_CFG_IQ_ORDER, 
                      pDio2IqIdcChCfgGrp[i].iq_order) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CH_CFG_PKT_TYPE, 
                      pDio2IqIdcChCfgGrp[i].pkt_type) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CH_CFG_CHAN_FRC_OFF, 
                      pDio2IqIdcChCfgGrp[i].chan_frc_off);
        hIqn2->regs->Dio2.DIO2_IQ_IDC_CHANNEL_CONFIG_GROUP.DIO2_IQ_IDC_CH_CFG[i] = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2DtRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 DT (Data Trace) registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pIqn2Fl_Dio2Dt   Pointer containing "Setup" properties for IQN2.

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n DIO2_DT
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2DtRegs (hIqn2, &dio2_dt);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2DtRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2Dt *pIqn2Fl_Dio2Dt
)
{
    uint32_t   tempReg;

    /* Setup DIO2 DT CFG0 */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_DT_CFG0_DT_EN, 
                  pIqn2Fl_Dio2Dt->dt_cfg0_dt_en) |
              CSL_FMK(IQN_DIO2_DIO2_DT_CFG0_DT_START_MODE, 
                  pIqn2Fl_Dio2Dt->dt_cfg0_dt_start_mode) |
              CSL_FMK(IQN_DIO2_DIO2_DT_CFG0_DT_STOP_MODE, 
                  pIqn2Fl_Dio2Dt->dt_cfg0_dt_stop_mode) |
              CSL_FMK(IQN_DIO2_DIO2_DT_CFG0_DT_ENDIAN_SEL, 
                  pIqn2Fl_Dio2Dt->dt_cfg0_dt_endian_sel);
    hIqn2->regs->Dio2.DIO2_DT.DIO2_DT_CFG0 = tempReg;

    /* Setup DIO2 DT CFG1 */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_DT.DIO2_DT_CFG1, 
        IQN_DIO2_DIO2_DT_CFG1_DT_DMA_BASE_ADDR, 
        pIqn2Fl_Dio2Dt->dt_cfg1_dt_dma_base_addr);

    /* Setup DIO2 DT CFG2 */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_DT.DIO2_DT_CFG2, 
        IQN_DIO2_DIO2_DT_CFG2_DT_DMA_WRAP, 
        pIqn2Fl_Dio2Dt->dt_cfg2_dt_dma_wrap);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2Regs
 *
 *   @b Description
 *   @n IQN2 DIO2 registers setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            dio2Setup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupDio2SiIqEfeCfgGrpRegs, Iqn2Fl_setupDio2SiIqEfeRadioStdGrpRegs,
 *      Iqn2Fl_setupDio2IqIfeChanCfgGrpRegs, Iqn2Fl_setupDio2IqIfeRadioStdGrpRegs,
 *      Iqn2Fl_setupDio2IqIdcChCfgGrpRegs, Iqn2Fl_setupDio2CoreIngressRegs,
 *      Iqn2Fl_setupDio2CoreIngDmaCfg0Regs, Iqn2Fl_setupDio2CoreEgrDmaCfg0Regs,
 *      Iqn2Fl_setupDio2DtRegs, Iqn2Fl_setupDio2IngDbcntxRamMmrRegs,
 *      Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs
 *
 *   @b Writes
 *   @n DIO2_SI_IQ_EFE_CONFIG_GROUP, DIO2_SI_IQ_EFE_RADIO_STANDARD_GROUP, 
 *      DIO2_IQ_EFE_CHAN_AXC_OFFSET, DIO2_IQ_EFE_FRM_SAMP_TC_MMR_RAM, 
 *      DIO2_IQ_EFE_CHAN_TDM_LUT, DIO2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP, 
 *      DIO2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP, DIO2_IQ_IFE_RADIO_STANDARD_GROUP,
 *      DIO2_IQ_IFE_CONFIG_GROUP, DIO2_IQ_IDC_CONFIGURATION_GROUP, 
 *      DIO2_IQ_IDC_CHANNEL_CONFIG_GROUP, DIO2_IFE_FRM_SAMP_TC_MMR_RAM,
 *      DIO2_UAT_GEN_CTL, DIO2_UAT_EGR_RADT, DIO2_UAT_ING_RADT, 
 *      DIO2_UAT_RADT_EVT, DIO2_UAT_DIO_EGR_RADT, DIO2_UAT_DIO_ING_RADT,
 *      DIO2_IQ_EDC_REGISTER_GROUP, DIO2_IQ_INGRESS_VBUS_MMR_GROUP, 
 *      DIO2_GLOBAL, DIO2_CORE_INGRESS, DIO2_I_AXC_OFF_MMR, 
 *      DIO2_CORE_EGRESS, DIO2_DT, DIO2_I_DBCNT0_RAM_MMR,
 *      DIO2_I_DBCNT1_RAM_MMR, DIO2_I_DBCNT2_RAM_MMR, DIO2_E_AOG_RAM_MMR,
 *      DIO2_E_DBCNT0_RAM_MMR, DIO2_E_DBCNT1_RAM_MMR, DIO2_E_DBCNT2_RAM_MMR
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2Regs (hIqn2, dio2Setup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2Setup *dio2Setup
)
{
    /* pointer to DIO2 registers setup */
    Iqn2Fl_Dio2Setup   *pDio2Config;
    uint32_t   tempReg, i;
    Iqn2Fl_Status retval = IQN2FL_SOK;

    pDio2Config = dio2Setup;

    /* Setup DIO2 SI IQ EFE CONFIG GROUP */
    Iqn2Fl_setupDio2SiIqEfeCfgGrpRegs (hIqn2, &(pDio2Config->dio2_efe_cfg_grp));

    /* Setup DIO2 SI IQ EFE RADIO STANDARD GROUP */
    Iqn2Fl_setupDio2SiIqEfeRadioStdGrpRegs (hIqn2, &(pDio2Config->dio2_efe_radio_std_grp));

    /* Setup DIO2 IQ EFE CHAN AXC OFFSET CFG - AXC_OFFSET */
    for (i = 0; i < 16; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_EFE_CHAN_AXC_OFFSET[i].DIO2_IQ_EFE_CHAN_AXC_OFFSET_CFG, 
            IQN_DIO2_DIO2_IQ_EFE_CHAN_AXC_OFFSET_CFG_AXC_OFFSET, 
            pDio2Config->efe_chan_axc_offset_cfg[i]);
    }

    /* Setup DIO2 IQ EFE FRM SAMP TC MMR RAM - SAMP_TC */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_EFE_FRM_SAMP_TC_MMR_RAM[i].DIO2_IQ_EFE_FRM_SAMP_TC_CFG, 
            IQN_DIO2_DIO2_IQ_EFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pDio2Config->efe_frm_samp_tc_cfg[i]);
    }

    /* Setup DIO2 IQ EFE CHAN TDM LUT */
    for (i = 0; i < 256; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_CHAN_TDM_LUT_CFG_CHAN_INDEX_CFG, 
                      pDio2Config->efe_chan_tdm_lut_cfg_chan_idx_cfg[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_CHAN_TDM_LUT_CFG_CHAN_INDEX_EN_CFG, 
                      pDio2Config->efe_chan_tdm_lut_cfg_chan_idx_en_cfg[i]);
        hIqn2->regs->Dio2.DIO2_IQ_EFE_CHAN_TDM_LUT[i].DIO2_IQ_EFE_CHAN_TDM_LUT_CFG = tempReg;
    }

    /* Setup DIO2 IQ EFE RADIO STANDARD SCHEDULER GROUP */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_RAD_STD_SCH_CFG_TDM_START, 
                      pDio2Config->efe_rad_std_sch_cfg_tdm_start[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_RAD_STD_SCH_CFG_TDM_LEN, 
                      pDio2Config->efe_rad_std_sch_cfg_tdm_len[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EFE_RAD_STD_SCH_CFG_TDM_EN, 
                      pDio2Config->efe_rad_std_sch_cfg_tdm_en[i]);
        hIqn2->regs->Dio2.DIO2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP.DIO2_IQ_EFE_RAD_STD_SCH_CFG[i] = tempReg;
    }

    /* Setup DIO2 IQ IFE CHANNEL CONFIGURATION GROUP */
    Iqn2Fl_setupDio2IqIfeChanCfgGrpRegs (hIqn2, &(pDio2Config->dio2_ife_chan_cfg_grp[0]));

    /* Setup DIO2 IQ IFE RADIO STANDARD GROUP */
    Iqn2Fl_setupDio2IqIfeRadioStdGrpRegs (hIqn2, &(pDio2Config->dio2_ife_radio_std_grp));

    /* Setup DIO2 IQ IDC CONFIGURATION GROUP */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CFG_FAIL_MARK_ONLY, 
                  pDio2Config->idc_cfg_grp_fail_mark_only) |
              CSL_FMK(IQN_DIO2_DIO2_IQ_IDC_CFG_FRC_OFF_ALL, 
                  pDio2Config->idc_cfg_grp_frc_off_all);
    hIqn2->regs->Dio2.DIO2_IQ_IDC_CONFIGURATION_GROUP.DIO2_IQ_IDC_CFG = tempReg;

    /* Setup DIO2 IQ IDC CHANNEL CONFIG GROUP */
    Iqn2Fl_setupDio2IqIdcChCfgGrpRegs (hIqn2, &(pDio2Config->dio2_idc_ch_cfg_grp[0]));

    /* Setup DIO2 IFE FRM SAMP TC MMR RAM - SAMP_TC */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IFE_FRM_SAMP_TC_MMR_RAM[i].DIO2_IQ_IFE_FRM_SAMP_TC_CFG, 
            IQN_DIO2_DIO2_IQ_IFE_FRM_SAMP_TC_CFG_SAMP_TC, 
            pDio2Config->ife_frm_samp_tc_cfg_samp_tc[i]);
    }

    /* Setup DIO2 UAT EGR RADT */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_EGR_RADT[i].DIO2_UAT_EGR_RADT_TC_CFG, 
            IQN_DIO2_DIO2_UAT_EGR_RADT_TC_CFG_VAL, 
            pDio2Config->uat_egr_radt_tc_cfg_val[i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_EGR_RADT[i].DIO2_UAT_EGR_RADT_OFFSET_CFG, 
            IQN_DIO2_DIO2_UAT_EGR_RADT_OFFSET_CFG_VAL, 
            pDio2Config->uat_egr_radt_offset_cfg_val[i]);
    }

    /* Setup DIO2 UAT RADT EVT */
    for (i = 0; i < 22; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_RADT_EVT[i].DIO2_UAT_EVT_RADT_CMP_CFG, 
            IQN_DIO2_DIO2_UAT_EVT_RADT_CMP_CFG_VAL, 
            pDio2Config->uat_evt_radt_cmp_cfg_val[i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_RADT_EVT[i].DIO2_UAT_EVT_CLK_CNT_TC_CFG, 
            IQN_DIO2_DIO2_UAT_EVT_CLK_CNT_TC_CFG_VAL, 
            pDio2Config->uat_evt_clk_cnt_tc_cfg_val[i]);
    }

    /* Setup DIO2 UAT DIO EGR RADT */
    for (i = 0; i < 3; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_EGR_RADT[i].DIO2_UAT_DIO_EGR_RADT_TC_CFG, 
            IQN_DIO2_DIO2_UAT_DIO_EGR_RADT_TC_CFG_VAL, 
            pDio2Config->uat_dio_egr_radt_tc_cfg_val[i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_EGR_RADT[i].DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG, 
            IQN_DIO2_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG_VAL, 
            pDio2Config->uat_dio_egr_radt_offset_cfg_val[i]);
    }

    /* Setup DIO2 UAT DIO ING RADT */
    for (i = 0; i < 3; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_ING_RADT[i].DIO2_UAT_DIO_ING_RADT_TC_CFG, 
            IQN_DIO2_DIO2_UAT_DIO_ING_RADT_TC_CFG_VAL, 
            pDio2Config->uat_dio_ing_radt_tc_cfg_val[i]);

        CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_ING_RADT[i].DIO2_UAT_DIO_ING_RADT_OFFSET_CFG, 
            IQN_DIO2_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG_VAL, 
            pDio2Config->uat_dio_ing_radt_offset_cfg_val[i]);
    }

    /* Setup DIO2 IQ EDC REGISTER GROUP - EDC CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_EDC_REGISTER_GROUP.DIO2_IQ_EDC_CFG, 
        IQN_DIO2_DIO2_IQ_EDC_CFG_PSI_ERR_CHK_DISABLE, 
        pDio2Config->dio2_iq_edc_cfg_psi_err_chk_disable);

    /* Setup DIO2 IQ EDC REGISTER GROUP - EDC CH CFG */
    for (i = 0; i < 16; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_IQ_EDC_CH_CFG_DAT_SWAP, 
                      pDio2Config->dio2_iq_edc_ch_cfg_dat_swap[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_IQ_EDC_CH_CFG_IQ_ORDER, 
                      pDio2Config->dio2_iq_edc_ch_cfg_iq_order[i]);
        hIqn2->regs->Dio2.DIO2_IQ_EDC_REGISTER_GROUP.DIO2_IQ_EDC_CH_CFG[i] = tempReg;
    }

    /* Setup DIO2 IQ INGRESS VBUS MMR GROUP
     * Default value is 15. Setting this value to a non-default value is possible if 0 < dio2_iq_idc_rate_ctl_cfg_rate <= 15
     * If user wants to force this value to 0, use Iqn2Fl_HwControl() with IQN2FL_CMD_DIO2_IQ_IDC_RATE_CTL_REG CMD word
     */
    if (pDio2Config->dio2_iq_idc_rate_ctl_cfg_rate) {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_INGRESS_VBUS_MMR_GROUP.DIO2_IQ_IDC_RATE_CTL_CFG,
            IQN_DIO2_DIO2_IQ_IDC_RATE_CTL_CFG_RATE,
            pDio2Config->dio2_iq_idc_rate_ctl_cfg_rate);
    }

    /* Setup DIO2 GLOBAL CFG */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_GLOBAL_CFG_VBUSM_PRIORITY, 
                  pDio2Config->dio2_global_cfg_vbusm_priority) |
              CSL_FMK(IQN_DIO2_DIO2_GLOBAL_CFG_RSA_BIG_ENDIAN, 
                  pDio2Config->dio2_global_cfg_rsa_big_endian);
    hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_GLOBAL_CFG = tempReg;

    /* Setup DIO2 CORE INGRESS */
    Iqn2Fl_setupDio2CoreIngressRegs (hIqn2, &pDio2Config->dio2_core_ingress);

    /* Setup DIO2 I AXC OFF MMR - OFFSET CFG */
    for (i = 0; i < 16; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_I_AXC_OFF_MMR[i].DIO2_I_AXC_OFFSET_CFG, 
            IQN_DIO2_DIO2_I_AXC_OFFSET_CFG_DIO_I_AXC_4SAMP_OFFSET, 
            pDio2Config->dio2_i_axc_off_mmr_cfg_4samp_offset[i]);
    }

    /* Setup DIO2 CORE EGRESS */
    Iqn2Fl_setupDio2CoreEgressRegs (hIqn2, &pDio2Config->dio2_core_egress);

    /* Setup DIO2 SI IQ EFE RADIO STANDARD GROUP */
    Iqn2Fl_setupDio2DtRegs (hIqn2, &(pDio2Config->dio2_dt));

    /* Setup DIO2 I DBCNT0/1/2 RAM MMR */
    for (i = 0; i < 3; i++)
    {
        retval = Iqn2Fl_setupDio2IngDbcntxRamMmrRegs (hIqn2, i, (Iqn2Fl_Dio2DbcntxRamMmr *)&(pDio2Config->dio2_i_dbcntx_ram_mmr[i]));
    }

    /* Setup DIO2 E AOG RAM MMR - E AXC OFF CFG */
    for (i = 0; i < 16; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_E_AOG_RAM_MMR[i].DIO2_E_AXC_OFF_CFG, 
            IQN_DIO2_DIO2_E_AXC_OFF_CFG_DIO_E_AXC_4SAMP_OFFSET, 
            pDio2Config->dio2_e_aog_ram_mmr_axc_off_cfg_4samp_offset[i]);
    }

    /* Setup DIO2 E DBCNT0/1/2 RAM MMR */
    for (i = 0; i < 3; i++)
    {
        retval = Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs (hIqn2, i, (Iqn2Fl_Dio2DbcntxRamMmr *)&(pDio2Config->dio2_e_dbcntx_ram_mmr[i]));
    }

    return retval;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAt2Regs
 *
 *   @b Description
 *   @n IQN2 AT2 registers setup
 *
 *   @b Arguments
 *   @verbatim

            hIqn2      Handle to the iqn2 instance
            at2Setup   Instance containing "Setup" properties for IQN2. 
            
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Calls
 *   @n Iqn2Fl_setupAt2Events24ArrayRegs
 *
 *   @b Writes
 *   @n AT2_RP1, AT2_BCN, AT2_GSM, AT2_RADT, AT2_EVENTS_24ARRAY,
        AT2_RADT_SYM_LUT_RAM
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAt2Regs (hIqn2, at2Setup);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAt2Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_At2Setup *at2Setup
)
{
    /* pointer to AT2 registers setup */
    Iqn2Fl_At2Setup   *pAt2Config;
    Iqn2Fl_HwControlMultiArgs   args;
    uint32_t   tempReg, i;
    Iqn2Fl_Status   ret_val = IQN2FL_SOK;

    pAt2Config = at2Setup;

    /* Setup AT2 START - AT2 TIMER ENABLES CFG - RADT EN */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN,
                                &(pAt2Config->at2_start_timer_enables_cfg_radt_en));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AT2 START - AT2 TIMER ENABLES CFG - RUN BCN */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN,
                                &(pAt2Config->at2_start_timer_enables_cfg_run_bcn));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AT2 RP1 - CTRL CFG  */
    tempReg = CSL_FMK(IQN_AT2_AT2_RP1_CTRL_CFG_CRC_USE, 
                  pAt2Config->at2_rp1_ctrl_cfg_crc_use) |
              CSL_FMK(IQN_AT2_AT2_RP1_CTRL_CFG_CRC_FLIP, 
                  pAt2Config->at2_rp1_ctrl_cfg_crc_flip) |
              CSL_FMK(IQN_AT2_AT2_RP1_CTRL_CFG_CRC_INIT_ONES, 
                  pAt2Config->at2_rp1_ctrl_cfg_crc_init_ones) |
              CSL_FMK(IQN_AT2_AT2_RP1_CTRL_CFG_CRC_INVERT, 
                  pAt2Config->at2_rp1_ctrl_cfg_crc_invert);
    hIqn2->regs->At2.AT2_RP1.AT2_RP1_CTRL_CFG = tempReg;

    /* Setup AT2 BCN - OFFSET CFG - VAL */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL,
                                &(pAt2Config->at2_bcn_offset_cfg_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AT2 BCN - SLVSEL CFG */
    CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_SLVSEL_CFG, 
        IQN_AT2_AT2_BCN_SLVSEL_CFG_VAL, 
        pAt2Config->at2_bcn_slvsel_cfg_val);

    /* Setup AT2 BCN - FRM INIT LSBS CFG - WR VAL */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL,
                                &(pAt2Config->at2_bcn_frm_init_lsbs_cfg_wr_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AT2 BCN - FRM INIT MSBS CFG - WR VAL */
    ret_val = Iqn2Fl_hwControl(hIqn2,
                                IQN2FL_CMD_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL,
                                &(pAt2Config->at2_bcn_frm_init_msbs_cfg_wr_val));
    if (ret_val != IQN2FL_SOK)
    {
        return ret_val;
    }

    /* Setup AT2 BCN - CLKCNT TC CFG */
    CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_CLKCNT_TC_CFG, 
        IQN_AT2_AT2_BCN_CLKCNT_TC_CFG_VAL, 
        pAt2Config->at2_bcn_clkcnt_tc_cfg_val);

    /* Setup AT2 BCN - FRAME TC LSB CFG */
    CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRAME_TC_LSB_CFG, 
        IQN_AT2_AT2_BCN_FRAME_TC_LSB_CFG_VAL, 
        pAt2Config->at2_bcn_frame_tc_lsb_cfg_val);

    /* Setup AT2 BCN - FRAME TC MSB CFG */
    CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRAME_TC_MSB_CFG, 
        IQN_AT2_AT2_BCN_FRAME_TC_MSB_CFG_VAL, 
        pAt2Config->at2_bcn_frame_tc_msb_cfg_val);

    /* Setup AT2 GSM - TCOUNT INIT CFG  */
    tempReg = CSL_FMK(IQN_AT2_AT2_GSM_TCOUNT_INIT_CFG_T1, 
                  pAt2Config->at2_gsm_tcount_init_cfg_t1) |
              CSL_FMK(IQN_AT2_AT2_GSM_TCOUNT_INIT_CFG_T2, 
                  pAt2Config->at2_gsm_tcount_init_cfg_t2) |
              CSL_FMK(IQN_AT2_AT2_GSM_TCOUNT_INIT_CFG_T3, 
                  pAt2Config->at2_gsm_tcount_init_cfg_t3);
    hIqn2->regs->At2.AT2_GSM.AT2_GSM_TCOUNT_INIT_CFG = tempReg;

    /* Setup AT2 RADT - INIT 1 CFG - FRM INIT LSB */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAt2Config->at2_radt_init_1_cfg_frm_init_lsb[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AT2 RADT - INIT 2 CFG - FRM INIT MSB */
    for (i = 0; i < 8; i++)
    {
        args.reg_arg = (void *)&(pAt2Config->at2_radt_init_2_cfg_frm_init_msb[i]);
        args.reg_index = i;
        ret_val = Iqn2Fl_hwControl(hIqn2,
                                    IQN2FL_CMD_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB,
                                    &args);
        if (ret_val != IQN2FL_SOK)
        {
            return ret_val;
        }
    }

    /* Setup AT2 RADT - 0 CFG  */
    for (i = 0; i < 8; i++)
    {
        tempReg = CSL_FMK(IQN_AT2_AT2_RADT_0_CFG_CLKCNT_TC, 
                      pAt2Config->at2_radt_0_cfg_clkcnt_tc[i]) |
                  CSL_FMK(IQN_AT2_AT2_RADT_0_CFG_LUTINDEX_TC, 
                      pAt2Config->at2_radt_0_cfg_lutindex_tc[i]) |
                  CSL_FMK(IQN_AT2_AT2_RADT_0_CFG_SYMB_TC, 
                      pAt2Config->at2_radt_0_cfg_symb_tc[i]);
        hIqn2->regs->At2.AT2_RADT[i].AT2_RADT_0_CFG = tempReg;
    }

    /* Setup AT2 RADT - 1 CFG */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_RADT[i].AT2_RADT_1_CFG, 
            IQN_AT2_AT2_RADT_1_CFG_FRM_TC_LSB, 
            pAt2Config->at2_radt_1_cfg_frm_tc_lsb[i]);
    }

    /* Setup AT2 RADT - 2 CFG */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_RADT[i].AT2_RADT_2_CFG, 
            IQN_AT2_AT2_RADT_2_CFG_FRM_TC_MSB, 
            pAt2Config->at2_radt_2_cfg_frm_tc_msb[i]);
    }

    /* Setup AT2 RADT - 3 CFG */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_RADT[i].AT2_RADT_3_CFG, 
            IQN_AT2_AT2_RADT_3_CFG_LUT_INDEX_STRT, 
            pAt2Config->at2_radt_3_cfg_lut_index_strt[i]);
    }

    /* Setup AT2 RADT - 4 CFG */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_RADT[i].AT2_RADT_4_CFG, 
            IQN_AT2_AT2_RADT_4_CFG_BCN_SYNC_CMP, 
            pAt2Config->at2_radt_4_cfg_bcn_sync_cmp[i]);
    }

    /* Setup AT2 EVENTS 24ARRAY */
    Iqn2Fl_setupAt2Events24ArrayRegs (hIqn2, &(pAt2Config->at2_events_24array));

    /* Setup AT2 EVENTS - EVT ENABLE CFG - EN */
    CSL_FINS(hIqn2->regs->At2.AT2_EVENTS.AT2_EVT_ENABLE_CFG, 
        IQN_AT2_AT2_EVT_ENABLE_CFG_EN, 
        pAt2Config->at2_evt_enable_cfg);

    /* Setup AT2 RADT SYM LUT RAM - CFG */
    for (i = 0; i < 256; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_RADT_SYM_LUT_RAM[i].AT2_RADT_SYM_LUT_RAM_CFG, 
            IQN_AT2_AT2_RADT_SYM_LUT_RAM_CFG_SYMBCNT_TC, 
            pAt2Config->at2_radt_sym_lut_ram_cfg_symbcnt_tc[i]);
    }

    return IQN2FL_SOK;
}

#ifdef __cplusplus
}
#endif
#endif /* _IQN2FLHWSETUPAUX_H_ */
