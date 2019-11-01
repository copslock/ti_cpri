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
 *   @file  iqn2fl_hwControlAux.h
 *
 *   @brief  API Auxilary header file for IQN2 to set HW control 
 *
 */

#ifndef _IQN2FLHWCONTROLAUX_H_
#define _IQN2FLHWCONTROLAUX_H_

#include <ti/drv/iqn2/iqn2fl.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  Hardware Control Functions of IQN2
 */

/** ============================================================================
 *   @n@b Iqn2Fl_ailEfeGlobalEnableSet
 *
 *   @b Description
 *   @n This function sets AIL EFE Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_IQ_EFE_GLOBAL_EN_SET_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value sets global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailEfeGlobalEnableSet (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailEfeGlobalEnableSet (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_GLOBAL_EN_SET_STB, IQN_AIL_AIL_IQ_EFE_GLOBAL_EN_SET_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailEfeGlobalEnableClear
 *
 *   @b Description
 *   @n This function clears AIL EFE Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_IQ_EFE_GLOBAL_EN_CLR_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value clears global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailEfeGlobalEnableClear (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailEfeGlobalEnableClear (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_GLOBAL_EN_CLR_STB, IQN_AIL_AIL_IQ_EFE_GLOBAL_EN_CLR_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilEgrSchCpriRadstdCfgRegs
 *
 *   @b Description
 *   @n This function configures AIL PE CPRI RADSTD CFG 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
            pAilSiIqCpriRadstdCfg   Pointer containing "Setup" properties for IQN2.

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_IQ_PE_CPRI_RADSTD_CFG, AIL_IQ_PE_CPRI_RADSTD1_CFG, AIL_IQ_PE_CPRI_RADSTD2_CFG
 *
 *   @b Example
 *   @verbatim
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_setupAilEgrSchCpriRadstdCfgRegs (hIqn2, &ail_iq_pe_cpri_radstd_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_setupAilEgrSchCpriRadstdCfgRegs (
    Iqn2Fl_Handle    hIqn2,
    Iqn2Fl_AilSiIqCpriRadstdCfg   *pAilSiIqCpriRadstdCfg
)
{
    uint32_t   tempReg, i;

    /* Setup PE CPRI RADIO STANDARD CONFIGURATION REGISTER PART 0-2 */
    for (i = 0; i < 8; i++)
    {
        CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD_CFG[i], 
            IQN_AIL_AIL_IQ_PE_CPRI_RADSTD_CFG_EN, 
            pAilSiIqCpriRadstdCfg->radstd_cfg_en[i]);

        tempReg = CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_RADSTD1_CFG_BFRM_OFFSET, 
                      pAilSiIqCpriRadstdCfg->radstd1_cfg_bfrm_offset[i]) |
                  CSL_FMK(IQN_AIL_AIL_IQ_PE_CPRI_RADSTD1_CFG_HFRM_OFFSET, 
                      pAilSiIqCpriRadstdCfg->radstd1_cfg_hfrm_offset[i]);
        hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD1_CFG[i] = tempReg;

        CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_CPRI.AIL_IQ_PE_CPRI_RADSTD2_CFG[i], 
            IQN_AIL_AIL_IQ_PE_CPRI_RADSTD2_CFG_BFRM_NUM, 
            pAilSiIqCpriRadstdCfg->radstd2_cfg_bfrm_num[i]);
    }
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailEctlGlobalEnableSet
 *
 *   @b Description
 *   @n This function sets AIL ECTL Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_ECTL_GLOBAL_EN_SET_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value sets global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailEctlGlobalEnableSet (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailEctlGlobalEnableSet (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ECTL_PKT_IF.AIL_ECTL_GLOBAL_EN_SET_STB, IQN_AIL_AIL_ECTL_GLOBAL_EN_SET_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailEctlGlobalEnableClear
 *
 *   @b Description
 *   @n This function clears AIL ECTL Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_ECTL_GLOBAL_EN_CLR_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value clears global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailEctlGlobalEnableClear (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailEctlGlobalEnableClear (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ECTL_PKT_IF.AIL_ECTL_GLOBAL_EN_CLR_STB, IQN_AIL_AIL_ECTL_GLOBAL_EN_CLR_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailIctlGlobalEnableSet
 *
 *   @b Description
 *   @n This function sets AIL ICTL Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_ICTL_GLOBAL_EN_SET_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value sets global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailIctlGlobalEnableSet (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailIctlGlobalEnableSet (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ICTL_PKT_IF.AIL_ICTL_GLOBAL_EN_SET_STB, IQN_AIL_AIL_ICTL_GLOBAL_EN_SET_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailIctlGlobalEnableClear
 *
 *   @b Description
 *   @n This function clears AIL ICTL Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_ICTL_GLOBAL_EN_CLR_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value clears global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailIctlGlobalEnableClear (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailIctlGlobalEnableClear (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ICTL_PKT_IF.AIL_ICTL_GLOBAL_EN_CLR_STB, IQN_AIL_AIL_ICTL_GLOBAL_EN_CLR_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailIfeGlobalEnableSet
 *
 *   @b Description
 *   @n This function sets AIL IFE Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_IQ_IFE_GLOBAL_EN_SET_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value sets global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailIfeGlobalEnableSet (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailIfeGlobalEnableSet (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IFE_CONFIG_GROUP.AIL_IQ_IFE_GLOBAL_EN_SET_STB, IQN_AIL_AIL_IQ_IFE_GLOBAL_EN_SET_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_ailIfeGlobalEnableClear
 *
 *   @b Description
 *   @n This function clears AIL IFE Global Enable 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_IQ_IFE_GLOBAL_EN_CLR_STB
 *
 *   @b Example
 *   @verbatim
        uint32_t arg = 0x1234; write of any value clears global enable
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_ailIfeGlobalEnableClear (hIqn2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_ailIfeGlobalEnableClear (
    Iqn2Fl_Handle    hIqn2,
    uint32_t            arg
)
{
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IFE_CONFIG_GROUP.AIL_IQ_IFE_GLOBAL_EN_CLR_STB, IQN_AIL_AIL_IQ_IFE_GLOBAL_EN_CLR_STB_DONT_CARE, arg);
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPdCpriAxcRadstdCfgRegs
 *
 *   @b Description
 *   @n This function configures AIL PD CPRI AXC RADSTD CFG 
 *
 *   @b Arguments
 *   @verbatim

            hIqn2    Handle to the iqn2 instance  (should use hIqn2->arg_ail to select AIL instance)
            pAilPdCpriAxcRadstdCfg   Pointer containing "Setup" properties for IQN2.

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIL_PD_CPRI_RADSTD_CFG, AIL_PD_CPRI_RADSTD1_CFG, AIL_PD_CPRI_RADSTD2_CFG
 *
 *   @b Example
 *   @verbatim
        hIqn2->arg_ail = IQN2FL_AIL_0; // configure AIL 0
        Iqn2Fl_setupAilPdCpriAxcRadstdCfgRegs (hIqn2, &ail_iq_pd_cpri_axc_radstd_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Iqn2Fl_setupAilPdCpriAxcRadstdCfgRegs (
    Iqn2Fl_Handle    hIqn2,
    Iqn2Fl_AilSiIqCpriRadstdCfg   *pAilPdCpriAxcRadstdCfg
)
{
    uint32_t   tempReg, i;

    /* Setup PD CPRI RADIO STANDARD CONFIGURATION REGISTER PART 0-2 */
    for (i = 0; i < 8; i++)
    {
        /* AIL PD CPRI RADSTD CFG */
        CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD_CFG[i], 
            IQN_AIL_AIL_PD_CPRI_RADSTD_CFG_EN, 
            pAilPdCpriAxcRadstdCfg->radstd_cfg_en[i]);

        /* AIL PD CPRI RADSTD1 CFG */
        tempReg = CSL_FMK(IQN_AIL_AIL_PD_CPRI_RADSTD1_CFG_BFRM_OFFSET, 
                      pAilPdCpriAxcRadstdCfg->radstd1_cfg_bfrm_offset[i]) |
                  CSL_FMK(IQN_AIL_AIL_PD_CPRI_RADSTD1_CFG_HFRM_OFFSET, 
                      pAilPdCpriAxcRadstdCfg->radstd1_cfg_hfrm_offset[i]);
        hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD1_CFG[i] = tempReg;

        /* AIL PD CPRI RADSTD2 CFG */
        CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PD_CPRI_AXC_CFG.AIL_PD_CPRI_RADSTD2_CFG[i], 
            IQN_AIL_AIL_PD_CPRI_RADSTD2_CFG_BFRM_NUM, 
            pAilPdCpriAxcRadstdCfg->radstd2_cfg_bfrm_num[i]);
    }
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilUatGenCtlUatCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL UAT GEN CTL UAT configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pUatCfg    Pointer to @a Iqn2Fl_UatCfg

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
 *   @n AIL_UAT_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilUatGenCtlUatCfgRegs (hIqn2, &ail_uat_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilUatGenCtlUatCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_UatCfg *pUatCfg
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AIL_AIL_UAT_CFG_UAT_RUN, 
                  pUatCfg->uat_run) |
              CSL_FMK(IQN_AIL_AIL_UAT_CFG_DIAG_SYNC, 
                  pUatCfg->diag_sync);
    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_GEN_CTL.AIL_UAT_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyCiLutACfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY CI LUT A configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAilPhyLutSetup    Pointer to @a Iqn2Fl_AilPhyLutSetup

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
 *   @n AIL_PHY_CI_LUT_A
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyCiLutACfgRegs (hIqn2, &ail_phy_ci_lut_a_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyCiLutACfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilPhyLutSetup *pAilPhyLutSetup
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_COUNT, 
                  pAilPhyLutSetup->smpl_count) |
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_OFFSET, 
                  pAilPhyLutSetup->smpl_offset) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_TYPE, 
                  pAilPhyLutSetup->smpl_type) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTA_CFG_SMPL_LAST, 
                  pAilPhyLutSetup->smpl_last);
    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CI_LUT_A[pAilPhyLutSetup->index].AIL_PHY_CI_LUTA_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyCiLutBCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY CI LUT B configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAilPhyLutSetup    Pointer to @a Iqn2Fl_AilPhyLutSetup

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
 *   @n AIL_PHY_CI_LUT_B
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyCiLutBCfgRegs (hIqn2, &ail_phy_ci_lut_b_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyCiLutBCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilPhyLutSetup *pAilPhyLutSetup
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_COUNT, 
                  pAilPhyLutSetup->smpl_count) |
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_OFFSET, 
                  pAilPhyLutSetup->smpl_offset) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_TYPE, 
                  pAilPhyLutSetup->smpl_type) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CI_LUTB_CFG_SMPL_LAST, 
                  pAilPhyLutSetup->smpl_last);
    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CI_LUT_B[pAilPhyLutSetup->index].AIL_PHY_CI_LUTB_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyCoLutACfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY CO LUT A configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAilPhyLutSetup    Pointer to @a Iqn2Fl_AilPhyLutSetup

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
 *   @n AIL_PHY_CO_LUT_A
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyCoLutACfgRegs (hIqn2, &ail_phy_co_lut_a_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyCoLutACfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilPhyLutSetup *pAilPhyLutSetup
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_COUNT, 
                  pAilPhyLutSetup->smpl_count) |
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_OFFSET, 
                  pAilPhyLutSetup->smpl_offset) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_TYPE, 
                  pAilPhyLutSetup->smpl_type) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTA_CFG_SMPL_LAST, 
                  pAilPhyLutSetup->smpl_last);
    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CO_LUT_A[pAilPhyLutSetup->index].AIL_PHY_CO_LUTA_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAilPhyCoLutBCfgRegs
 *
 *   @b Description
 *   @n IQN2 AIL PHY CO LUT B configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pAilPhyLutSetup    Pointer to @a Iqn2Fl_AilPhyLutSetup

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
 *   @n AIL_PHY_CO_LUT_B
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAilPhyCoLutBCfgRegs (hIqn2, &ail_phy_co_lut_b_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAilPhyCoLutBCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_AilPhyLutSetup *pAilPhyLutSetup
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_COUNT, 
                  pAilPhyLutSetup->smpl_count) |
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_OFFSET, 
                  pAilPhyLutSetup->smpl_offset) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_TYPE, 
                  pAilPhyLutSetup->smpl_type) | 
              CSL_FMK(IQN_AIL_AIL_PHY_CO_LUTB_CFG_SMPL_LAST, 
                  pAilPhyLutSetup->smpl_last);
    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CO_LUT_B[pAilPhyLutSetup->index].AIL_PHY_CO_LUTB_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_updateEgressAilRadioStandardTc
 *
 *   @b Description
 *   @n This functions updates the Radio Framing Sample Terminal Count Configuration Register for
 *   for a symbol index of a radio standard id. Used for LTE to implement runtime updates required
 *   by MBSFN implementation.
 *
 *   @b Arguments
 *   @verbatim

         hIqn2         Handle to the iqn2 instance
         radioStdId    Radio Standard ID
         symbolSize    New symbol size (sample count)
         symbolIndex   Symbol index

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open(), Iqn2Fl_hwSetup
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIL_IQ_EFE_FRM_SAMP_TC_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_updateEgressAilRadioStandardTc (hIqn2, radioStdId, symbolSize, symbolIndex);
     @endverbatim
 * ===========================================================================
 */
static inline
Iqn2Fl_Status Iqn2Fl_updateEgressAilRadioStandardTc(
        Iqn2Fl_Handle hIqn2,
        uint32_t      radioStdId,
        uint32_t      symbolSize,
        uint32_t      symbolIndex
)
{
    uint32_t startIdx;

    startIdx = CSL_FEXT(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_RADIO_STANDARD_GROUP.AIL_IQ_EFE_FRM_TC_CFG[radioStdId],IQN_AIL_AIL_IQ_EFE_FRM_TC_CFG_INDEX_SC);
    CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_EFE_FRM_SAMP_TC_MMR_RAM[startIdx+symbolIndex].AIL_IQ_EFE_FRM_SAMP_TC_CFG, IQN_AIL_AIL_IQ_EFE_FRM_SAMP_TC_CFG_SAMP_TC,symbolSize);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupTopVcSysStsSwResetStbRegs
 *
 *   @b Description
 *   @n IQN2 Top VC Sys Status SW Reset Stb registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pTopVCSwResetStbSetup   Pointer containing "Setup" properties for IQN2. 

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
 *   @n VC_SW_RESET_STB
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupTopVcSysStsSwResetStbRegs (hIqn2, &vc_sys_sts_sw_reset_stb);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupTopVcSysStsSwResetStbRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_TopVCSwResetStbSetup *pTopVCSwResetStbSetup
)
{
    uint32_t   tempReg;

    /* Setup VC SW RESET STB register fields */
    tempReg = CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET, 
                  pTopVCSwResetStbSetup->sw_reset) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_AID, 
                  pTopVCSwResetStbSetup->sw_reset_aid) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_DIO, 
                  pTopVCSwResetStbSetup->sw_reset_dio) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_PKTDMA, 
                  pTopVCSwResetStbSetup->sw_reset_pktdma) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_AIL0, 
                  pTopVCSwResetStbSetup->sw_reset_ail0) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_AIL1, 
                  pTopVCSwResetStbSetup->sw_reset_ail1) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_AIL2, 
                  pTopVCSwResetStbSetup->sw_reset_ail2) |
              CSL_FMK(IQN2_TOP_VC_SW_RESET_STB_SW_RESET_AIL3, 
                  pTopVCSwResetStbSetup->sw_reset_ail3);
    hIqn2->regs->Top.VC_SYS_STS_CFG.VC_SW_RESET_STB = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAt2Events24ArrayRegs
 *
 *   @b Description
 *   @n IQN2 AT2 Events 24Array registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pIqn2Fl_At2Events24Array   Pointer containing "Setup" properties for IQN2.

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
 *   @n AT2_EVENTS_24ARRAY
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAt2Events24ArrayRegs (hIqn2, &at2_events_24array);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAt2Events24ArrayRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_At2Events24Array *pIqn2Fl_At2Events24Array
)
{
    uint32_t   i, tempReg;

    /* Setup AT2 EVENTS 24ARRAY - EVENT OFFSET CFG  */
    for (i = 0; i < 24; i++)
    {
        tempReg = CSL_FMK(IQN_AT2_AT2_EVENT_OFFSET_CFG_VAL, 
                      pIqn2Fl_At2Events24Array->at2_events_24array_offset_cfg_val[i]) |
                  CSL_FMK(IQN_AT2_AT2_EVENT_OFFSET_CFG_EVT_STRB_SEL, 
                      pIqn2Fl_At2Events24Array->at2_events_24array_offset_cfg_evt_strb_sel[i]);
        hIqn2->regs->At2.AT2_EVENTS_24ARRAY[i].AT2_EVENT_OFFSET_CFG = tempReg;
    }

    /* Setup AT2 EVENTS 24ARRAY - EVENT MOD TC CFG */
    for (i = 0; i < 24; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_EVENTS_24ARRAY[i].AT2_EVENT_MOD_TC_CFG, 
            IQN_AT2_AT2_EVENT_MOD_TC_CFG_VAL, 
            pIqn2Fl_At2Events24Array->at2_events_24array_mod_tc_cfg_val[i]);
    }

    /* Setup AT2 EVENTS 24ARRAY - EVENT MASK LSBS CFG */
    for (i = 0; i < 24; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_EVENTS_24ARRAY[i].AT2_EVENT_MASK_LSBS_CFG, 
            IQN_AT2_AT2_EVENT_MASK_LSBS_CFG_VAL, 
            pIqn2Fl_At2Events24Array->at2_events_24array_mask_lsbs_cfg_val[i]);
    }

    /* Setup AT2 EVENTS 24ARRAY - EVENT MASK MSBS CFG */
    for (i = 0; i < 24; i++)
    {
        CSL_FINS(hIqn2->regs->At2.AT2_EVENTS_24ARRAY[i].AT2_EVENT_MASK_MSBS_CFG, 
            IQN_AT2_AT2_EVENT_MASK_MSBS_CFG_VAL, 
            pIqn2Fl_At2Events24Array->at2_events_24array_mask_msbs_cfg_val[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2UatEgrRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 AID2 UAT Egress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n AID2_UAT_EGR_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2UatEgrRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2UatEgrRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup AID2 UAT EGR RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_EGR_RADT[i].AID2_UAT_EGR_RADT_OFFSET_CFG, 
        IQN_AID2_AID2_UAT_EGR_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2UatIngRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 AID2 UAT Ingress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n AID2_UAT_ING_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2UatIngRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2UatIngRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup AID2 UAT ING RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_ING_RADT[i].AID2_UAT_ING_RADT_OFFSET_CFG, 
        IQN_AID2_AID2_UAT_ING_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2UatGenCtlUatCfgRegs
 *
 *   @b Description
 *   @n IQN2 AID2 UAT GEN CTL UAT configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pUatCfg    Pointer to @a Iqn2Fl_UatCfg

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
 *   @n AID2_UAT_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2UatGenCtlUatCfgRegs (hIqn2, &aid2_uat_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2UatGenCtlUatCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_UatCfg *pUatCfg
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_AID2_AID2_UAT_CFG_UAT_RUN, 
                  pUatCfg->uat_run) |
              CSL_FMK(IQN_AID2_AID2_UAT_CFG_DIAG_SYNC, 
                  pUatCfg->diag_sync);
    hIqn2->regs->Aid2.AID2_UAT_GEN_CTL.AID2_UAT_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupAid2UatRadtEvtRegs
 *
 *   @b Description
 *   @n IQN2 AID2 UAT RADT EVT registers setup per RADT (event num)
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pUatRadtEvtCfg   Pointer containing "Setup" properties for IQN2.

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
 *   @n 
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupAid2UatRadtEvtRegs (hIqn2, &uat_radt_evt_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupAid2UatRadtEvtRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_UatRadtEvtSetup *pUatRadtEvtCfg
)
{
    uint32_t   i = pUatRadtEvtCfg->event_num;

    /* EVT RADT CMP CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_RADT_EVT[i].AID2_UAT_EVT_RADT_CMP_CFG, 
        IQN_AID2_AID2_UAT_EVT_RADT_CMP_CFG_VAL, 
        pUatRadtEvtCfg->cmp_cfg_val);

    /* EVT CLK CNT TC CFG */
    CSL_FINS(hIqn2->regs->Aid2.AID2_UAT_RADT_EVT[i].AID2_UAT_EVT_CLK_CNT_TC_CFG, 
        IQN_AID2_AID2_UAT_EVT_CLK_CNT_TC_CFG_VAL, 
        pUatRadtEvtCfg->clk_cnt_tc);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_updateEgressAid2RadioStandardTc
 *
 *   @b Description
 *   @n This functions updates the Radio Framing Sample Terminal Count Configuration Register for
 *   for a symbol index of a radio standard id. Used for LTE to implement runtime updates required
 *   by MBSFN implementation.
 *
 *   @b Arguments
 *   @verbatim

         hIqn2         Handle to the iqn2 instance
         radioStdId    Radio Standard ID
         symbolSize    New symbol size (sample count)
         symbolIndex   Symbol index

     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open(), Iqn2Fl_hwSetup
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AID2_IQ_EFE_FRM_SAMP_TC_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_updateEgressAid2RadioStandardTc (hIqn2, radioStdId, symbolSize, symbolIndex);
     @endverbatim
 * ===========================================================================
 */
static inline
Iqn2Fl_Status Iqn2Fl_updateEgressAid2RadioStandardTc(
        Iqn2Fl_Handle hIqn2,
        uint32_t      radioStdId,
        uint32_t      symbolSize,
        uint32_t      symbolIndex
)
{
    uint32_t startIdx;

    startIdx = CSL_FEXT(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_RADIO_STANDARD_GROUP.AID2_IQ_EFE_FRM_TC_CFG[radioStdId],IQN_AID2_AID2_IQ_EFE_FRM_TC_CFG_INDEX_SC);
    CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EFE_FRM_SAMP_TC_MMR_RAM[startIdx+symbolIndex].AID2_IQ_EFE_FRM_SAMP_TC_CFG, IQN_AID2_AID2_IQ_EFE_FRM_SAMP_TC_CFG_SAMP_TC, symbolSize);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatEgrRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT Egress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n DIO2_UAT_EGR_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatEgrRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatEgrRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup DIO2 UAT EGR RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_EGR_RADT[i].DIO2_UAT_EGR_RADT_OFFSET_CFG, 
        IQN_DIO2_DIO2_UAT_EGR_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatIngRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT Ingress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n DIO2_UAT_ING_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatIngRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatIngRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup DIO2 UAT ING RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_ING_RADT[i].DIO2_UAT_ING_RADT_OFFSET_CFG, 
        IQN_DIO2_DIO2_UAT_ING_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatRadtEvtRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT RADT EVT registers setup per RADT (event num)
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pUatRadtEvtCfg   Pointer containing "Setup" properties for IQN2.

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
 *   @n 
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatRadtEvtRegs (hIqn2, &uat_radt_evt_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatRadtEvtRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_UatRadtEvtSetup *pUatRadtEvtCfg
)
{
    uint32_t   i = pUatRadtEvtCfg->event_num;

    /* EVT RADT CMP CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_RADT_EVT[i].DIO2_UAT_EVT_RADT_CMP_CFG, 
        IQN_DIO2_DIO2_UAT_EVT_RADT_CMP_CFG_VAL, 
        pUatRadtEvtCfg->cmp_cfg_val);

    /* EVT CLK CNT TC CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_RADT_EVT[i].DIO2_UAT_EVT_CLK_CNT_TC_CFG, 
        IQN_DIO2_DIO2_UAT_EVT_CLK_CNT_TC_CFG_VAL, 
        pUatRadtEvtCfg->clk_cnt_tc);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatDioEgrRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT DIO2 Egress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatDioEgrRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatDioEgrRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup DIO2 UAT DIO EGR RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_EGR_RADT[i].DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG, 
        IQN_DIO2_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatDioIngRadtOffsetCfgRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT DIO2 Ingress Radio Timers offset configuration
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pOffsetCfg   Pointer to Iqn2Fl_RadtOffsetCfg

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
 *   @n DIO2_UAT_DIO_ING_RADT_OFFSET_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatDioIngRadtOffsetCfgRegs (hIqn2, &offset_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatDioIngRadtOffsetCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_RadtOffsetCfg *pOffsetCfg
)
{
    uint32_t   i = pOffsetCfg->radio_std;

    /* Setup DIO2 UAT DIO ING RADT - OFFSET CFG */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_UAT_DIO_ING_RADT[i].DIO2_UAT_DIO_ING_RADT_OFFSET_CFG, 
        IQN_DIO2_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG_VAL, 
        pOffsetCfg->offset);

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2UatGenCtlUatCfgRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 UAT GEN CTL UAT configuration registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pUatCfg    Pointer to @a Iqn2Fl_UatCfg

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
 *   @n DIO2_UAT_CFG
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2UatGenCtlUatCfgRegs (hIqn2, &dio2_uat_cfg);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2UatGenCtlUatCfgRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_UatCfg *pUatCfg
)
{
    uint32_t   tempReg;

    tempReg = CSL_FMK(IQN_DIO2_DIO2_UAT_CFG_UAT_RUN, 
                  pUatCfg->uat_run) |
              CSL_FMK(IQN_DIO2_DIO2_UAT_CFG_DIAG_SYNC, 
                  pUatCfg->diag_sync);
    hIqn2->regs->Dio2.DIO2_UAT_GEN_CTL.DIO2_UAT_CFG = tempReg;

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2CoreIngDmaCfg0Regs
 *
 *   @b Description
 *   @n DIO2 CORE INGRESS - I DMA CFG0 setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2CoreDmaCfg0   Pointer containing "Setup" properties for IQN2. 
            
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
 *   @n DIO2_CORE_INGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2CoreIngDmaCfg0Regs (hIqn2, &dio2_core_ing_dma_cfg0[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2CoreIngDmaCfg0Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2CoreDmaCfg0 *pDio2CoreDmaCfg0
)
{
    uint32_t   tempReg, i;

    /* Setup DIO2 CORE INGRESS - I DMA CFG0 */
    for (i = 0; i < 3; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_BRST_LN, 
                      pDio2CoreDmaCfg0[i].dma_brst_ln) |
                  CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_NUM_QWD, 
                      pDio2CoreDmaCfg0[i].dma_num_qwd) |
                  CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_RSA_CNVRT_EN, 
                      pDio2CoreDmaCfg0[i].rsa_cnvrt_en) |
                  CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_ENG_EN, 
                      pDio2CoreDmaCfg0[i].dma_eng_en) |
                  CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_NUM_BLKS, 
                      pDio2CoreDmaCfg0[i].dma_num_blks);
        hIqn2->regs->Dio2.DIO2_CORE_INGRESS[i].DIO2_I_DMA_CFG0 = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2CoreIngressRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 CORE INGRESS registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2CoreIngress    Pointer containing "Setup" properties for IQN2.

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
 *   @n DIO2_CORE_INGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2CoreIngressRegs (hIqn2, &dio2_core_ing);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2CoreIngressRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2CoreIngressSetup *pDio2CoreIngress
)
{
    uint32_t   i, tempReg;

    /* Setup DIO2 CORE INGRESS - I TABLE SEL CFG */
    for (i = 0; i < 3; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE_SEL_CFG_BCN_TABLE_SEL, 
                      pDio2CoreIngress->bcn_table_sel[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_I_TABLE_SEL_CFG_DMA_NUM_AXC, 
                      pDio2CoreIngress->dma_num_axc[i]);
        hIqn2->regs->Dio2.DIO2_CORE_INGRESS[i].DIO2_I_TABLE_SEL_CFG = tempReg;
    }

    /* Setup DIO2 CORE INGRESS - I DMA CFG0 */
    Iqn2Fl_setupDio2CoreIngDmaCfg0Regs (hIqn2, &(pDio2CoreIngress->dma_cfg0[0]));

    /* Setup DIO2 CORE INGRESS - I DMA CFG1 */
    for (i = 0; i < 3; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_INGRESS[i].DIO2_I_DMA_CFG1, 
            IQN_DIO2_DIO2_I_DMA_CFG1_DMA_BLK_ADDR_STRIDE, 
            pDio2CoreIngress->dma_cfg1_dma_blk_addr_stride[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2CoreEgrDmaCfg0Regs
 *
 *   @b Description
 *   @n DIO2 CORE EGRESS - E DMA CFG0 setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2CoreDmaCfg0   Pointer containing "Setup" properties for IQN2. 
            
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
 *   @n DIO2_CORE_EGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2CoreEgrDmaCfg0Regs (hIqn2, &dio2_core_egr_dma_cfg0[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2CoreEgrDmaCfg0Regs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2CoreDmaCfg0 *pDio2CoreDmaCfg0
)
{
    uint32_t   tempReg, i;

    /* Setup DIO2 CORE EGRESS - E DMA CFG0 */
    for (i = 0; i < 3; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_BRST_LN, 
                      pDio2CoreDmaCfg0[i].dma_brst_ln) |
                  CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_NUM_QWD, 
                      pDio2CoreDmaCfg0[i].dma_num_qwd) |
                  CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_RSA_CNVRT_EN, 
                      pDio2CoreDmaCfg0[i].rsa_cnvrt_en) |
                  CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_ENG_EN, 
                      pDio2CoreDmaCfg0[i].dma_eng_en) |
                  CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_NUM_BLKS, 
                      pDio2CoreDmaCfg0[i].dma_num_blks);
        hIqn2->regs->Dio2.DIO2_CORE_EGRESS[i].DIO2_E_DMA_CFG0 = tempReg;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2CoreEgressRegs
 *
 *   @b Description
 *   @n IQN2 DIO2 CORE EGRESS registers setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         pDio2CoreEgress    Pointer containing "Setup" properties for IQN2.

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
 *   @n DIO2_CORE_EGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2CoreEgressRegs (hIqn2, &dio2_core_egr);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2CoreEgressRegs(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2CoreEgressSetup *pDio2CoreEgress
)
{
    uint32_t   i, tempReg;

    /* Setup DIO2 CORE EGRESS - E TABLE SEL CFG */
    for (i = 0; i < 3; i++)
    {
        tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE_SEL_CFG_BCN_TABLE_SEL, 
                      pDio2CoreEgress->bcn_table_sel[i]) |
                  CSL_FMK(IQN_DIO2_DIO2_E_TABLE_SEL_CFG_DMA_NUM_AXC, 
                      pDio2CoreEgress->dma_num_axc[i]);
        hIqn2->regs->Dio2.DIO2_CORE_EGRESS[i].DIO2_E_TABLE_SEL_CFG = tempReg;
    }

    /* Setup DIO2 CORE EGRESS - E DMA CFG0 */
    Iqn2Fl_setupDio2CoreEgrDmaCfg0Regs (hIqn2, &(pDio2CoreEgress->dma_cfg0[0]));

    /* Setup DIO2 CORE EGRESS - E DMA CFG1 */
    for (i = 0; i < 3; i++)
    {
        CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_EGRESS[i].DIO2_E_DMA_CFG1, 
            IQN_DIO2_DIO2_E_DMA_CFG1_DMA_BLK_ADDR_STRIDE, 
            pDio2CoreEgress->dma_cfg1_dma_blk_addr_stride[i]);
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2IngDbcntxRamMmrRegs
 *
 *   @b Description
 *   @n DIO2 I DBCNT0/1/2 RAM MMR setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         eng_idx    DIO engine number. Valid values 0 to 2.
         pDio2DbcntxRamMmr   Pointer containing "Setup" properties for IQN2. 

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
 *   @n DIO2_I_DBCNT0_RAM_MMR, DIO2_I_DBCNT1_RAM_MMR, DIO2_I_DBCNT2_RAM_MMR
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2IngDbcntxRamMmrRegs (hIqn2, &dio2_i_dbcntx_ram_mmr[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2IngDbcntxRamMmrRegs(
    Iqn2Fl_Handle hIqn2,
    uint32_t eng_idx,
    Iqn2Fl_Dio2DbcntxRamMmr *pDio2DbcntxRamMmr
)
{
    uint32_t   tempReg, i;

    if (eng_idx == 0)
    {
        /* Setup DIO2 I DBCNT0 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 I TABLE0 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT0_RAM_MMR[i].DIO2_I_TABLE0_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_I_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 I TABLE0 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE0_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE0_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT0_RAM_MMR[i].DIO2_I_TABLE0_CTRL_CFG = tempReg;
        }
    }
    else if (eng_idx == 1)
    {
        /* Setup DIO2 I DBCNT1 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 I TABLE1 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT1_RAM_MMR[i].DIO2_I_TABLE1_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_I_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 I TABLE1 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE1_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE1_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT1_RAM_MMR[i].DIO2_I_TABLE1_CTRL_CFG = tempReg;
        }
    }
    else if (eng_idx == 2)
    {
        /* Setup DIO2 I DBCNT2 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 I TABLE2 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT2_RAM_MMR[i].DIO2_I_TABLE2_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_I_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 I TABLE2 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE2_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE2_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT2_RAM_MMR[i].DIO2_I_TABLE2_CTRL_CFG = tempReg;
        }
    }
    else
    {
        return IQN2FL_INVPARAMS;
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs
 *
 *   @b Description
 *   @n DIO2 E DBCNT0/1/2 RAM MMR setup
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         eng_idx    DIO engine number. Valid values 0 to 2.
         pDio2DbcntxRamMmr   Pointer containing "Setup" properties for IQN2. 
            
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
 *   @n DIO2_E_DBCNT0_RAM_MMR, DIO2_E_DBCNT1_RAM_MMR, DIO2_E_DBCNT2_RAM_MMR
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs (hIqn2, &dio2_e_dbcntx_ram_mmr[0]);
     @endverbatim
 * ===========================================================================
 */
static inline 
Iqn2Fl_Status Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs(
    Iqn2Fl_Handle hIqn2,
    uint32_t eng_idx,
    Iqn2Fl_Dio2DbcntxRamMmr *pDio2DbcntxRamMmr
)
{
    uint32_t   tempReg, i;

    if (eng_idx == 0)
    {
        /* Setup DIO2 E DBCNT0 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 E TABLE0 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT0_RAM_MMR[i].DIO2_E_TABLE0_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_E_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 E TABLE0 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE0_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE0_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT0_RAM_MMR[i].DIO2_E_TABLE0_CTRL_CFG = tempReg;
        }
    }
    else if (eng_idx == 1)
    {
        /* Setup DIO2 E DBCNT1 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 E TABLE1 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT1_RAM_MMR[i].DIO2_E_TABLE1_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_E_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 E TABLE1 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE1_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE1_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT1_RAM_MMR[i].DIO2_E_TABLE1_CTRL_CFG = tempReg;
        }
    }
    else if (eng_idx == 2)
    {
        /* Setup DIO2 E DBCNT2 RAM MMR */
        for (i = 0; i < 32; i++)
        {
            /* Setup DIO2 E TABLE2 BASE ADDR CFG */
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT2_RAM_MMR[i].DIO2_E_TABLE2_BASE_ADDR_CFG, 
                IQN_DIO2_DIO2_E_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC, 
                pDio2DbcntxRamMmr->dma_vbus_base_addr_axc[i]);

            /* Setup DIO2 E TABLE2 CTRL CFG */
            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE2_CTRL_CFG_CH_ID, 
                          pDio2DbcntxRamMmr->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE2_CTRL_CFG_CH_EN, 
                          pDio2DbcntxRamMmr->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT2_RAM_MMR[i].DIO2_E_TABLE2_CTRL_CFG = tempReg;
        }
    }
    else
    {
        return IQN2FL_INVPARAMS;
    }

    return IQN2FL_SOK;
}


/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2ReconfigureEngine
 *
 *   @b Description
 *   @n In cases where:
 *      - DIO parameters need to be adjusted by, for instance Physical Layer software
 *      - IQN2 is configured via LLD from ARM Linux or any other software entity external to, for instance Physical layer software
 *      IQN2 functional layer has dedicated hardware control commands and auxiliary APIs that can be used prior to enabling the AT2 events.
 *      Iqn2Fl_setupDio2ReconfigureEngine() is an API that allows to reconfigure the following Egress/Ingress DIO parameters:
 *      - num_block
 *      - block_addr_stride
 *      - axc_buffer_start_addr
 *
 *   @b Arguments
 *   @verbatim

         hIqn2      Handle to the iqn2 instance
         Iqn2Fl_Dio2ReconfigureEngineSetup    Pointer containing "Iqn2Fl_Dio2ReconfigureEngineSetup" properties for DIO2.

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
 *   @n DIO2_RECONFIGURE_ENGINE
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2ReconfigureEngine (hIqn2, &dio2_reconfigure_engine);
     @endverbatim
 * ===========================================================================
 */
static inline
Iqn2Fl_Status Iqn2Fl_setupDio2ReconfigureEngine(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2ReconfigureEngineSetup *pDio2ReconfigureEngine
)
{
    uint32_t   idx, i;

    idx = pDio2ReconfigureEngine->engine_idx;

    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_EGRESS[idx].DIO2_E_DMA_CFG0,
            IQN_DIO2_DIO2_E_DMA_CFG0_DMA_NUM_BLKS,
            pDio2ReconfigureEngine->egress_num_block);

    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_INGRESS[idx].DIO2_I_DMA_CFG0,
            IQN_DIO2_DIO2_I_DMA_CFG0_DMA_NUM_BLKS,
            pDio2ReconfigureEngine->ingress_num_block);

    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_EGRESS[idx].DIO2_E_DMA_CFG1,
            IQN_DIO2_DIO2_E_DMA_CFG1_DMA_BLK_ADDR_STRIDE,
            pDio2ReconfigureEngine->egress_block_addr_stride);

    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_INGRESS[idx].DIO2_I_DMA_CFG1,
            IQN_DIO2_DIO2_I_DMA_CFG1_DMA_BLK_ADDR_STRIDE,
            pDio2ReconfigureEngine->ingress_block_addr_stride);

    if(idx == 0)
    {
        for (i=0; i< 16; i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT0_RAM_MMR[i].DIO2_E_TABLE0_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->egress_axc_buffer_start_addr[i]);

            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT0_RAM_MMR[i].DIO2_I_TABLE0_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->ingress_axc_buffer_start_addr[i]);
        }
    } else if (idx == 1) {
        for (i=0; i< 16; i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT1_RAM_MMR[i].DIO2_E_TABLE1_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->egress_axc_buffer_start_addr[i]);

            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT1_RAM_MMR[i].DIO2_I_TABLE1_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->ingress_axc_buffer_start_addr[i]);
        }
    } else if (idx == 2){
        for (i=0; i< 16; i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT2_RAM_MMR[i].DIO2_E_TABLE2_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->egress_axc_buffer_start_addr[i]);

            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT2_RAM_MMR[i].DIO2_I_TABLE2_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEngine->ingress_axc_buffer_start_addr[i]);
        }
    }

    return IQN2FL_SOK;
} // Iqn2Fl_setupDio2ReconfigureEngine

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2ReconfigureEgressEngine
 *
 *   @b Description
 *   @n In cases where:
 *      - DIO parameters need to be adjusted by, for instance Physical Layer software
 *      - IQN2 is configured via LLD from ARM Linux or any other software entity external to, for instance Physical layer software
 *      IQN2 functional layer has dedicated hardware control commands and auxiliary APIs that can be used prior to enabling the AT2 events.
 *      Iqn2Fl_setupDio2ReconfigureEgressEngine() is an API that allows to reconfigure the following Egress DIO parameters.
 *
 *   @b Arguments
 *   @verbatim

         hIqn2                        Handle to the iqn2 instance
         pDio2ReconfigureEgrEngine    Pointer containing "Iqn2Fl_Dio2ReconfigureEgrEngineSetup" properties for IQN2.

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
 *   @n DIO2_CORE_EGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2ReconfigureEgressEngine (hIqn2, &pDio2ReconfigureEgrEngine);
     @endverbatim
 * ===========================================================================
 */
static inline
Iqn2Fl_Status Iqn2Fl_setupDio2ReconfigureEgressEngine(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2ReconfigureEgrEngineSetup *pDio2ReconfigureEgrEngine
)
{
    uint32_t   tempReg;
    uint32_t   idx, i;

    idx = pDio2ReconfigureEgrEngine->engine_idx;

    /* Setup DIO2 CORE EGRESS - E TABLE SEL CFG */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE_SEL_CFG_BCN_TABLE_SEL,
                      pDio2ReconfigureEgrEngine->bcn_table_sel) |
              CSL_FMK(IQN_DIO2_DIO2_E_TABLE_SEL_CFG_DMA_NUM_AXC,
                      pDio2ReconfigureEgrEngine->dma_num_axc);
    hIqn2->regs->Dio2.DIO2_CORE_EGRESS[idx].DIO2_E_TABLE_SEL_CFG = tempReg;

    /* Setup DIO2 CORE EGRESS - E DMA CFG0 */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_BRST_LN,
                  pDio2ReconfigureEgrEngine->dma_cfg0.dma_brst_ln) |
              CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_NUM_QWD,
                  pDio2ReconfigureEgrEngine->dma_cfg0.dma_num_qwd) |
              CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_RSA_CNVRT_EN,
                  pDio2ReconfigureEgrEngine->dma_cfg0.rsa_cnvrt_en) |
              CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_ENG_EN,
                  pDio2ReconfigureEgrEngine->dma_cfg0.dma_eng_en) |
              CSL_FMK(IQN_DIO2_DIO2_E_DMA_CFG0_DMA_NUM_BLKS,
                  pDio2ReconfigureEgrEngine->dma_cfg0.dma_num_blks);
    hIqn2->regs->Dio2.DIO2_CORE_EGRESS[idx].DIO2_E_DMA_CFG0 = tempReg;

    /* Setup DIO2 CORE EGRESS - E DMA CFG1 */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_EGRESS[idx].DIO2_E_DMA_CFG1,
        IQN_DIO2_DIO2_E_DMA_CFG1_DMA_BLK_ADDR_STRIDE,
        pDio2ReconfigureEgrEngine->dma_cfg1_dma_blk_addr_stride);

    if(idx == 0)
    {
        for (i=(0 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT0_RAM_MMR[i].DIO2_E_TABLE0_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEgrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE0_CTRL_CFG_CH_ID,
                          pDio2ReconfigureEgrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE0_CTRL_CFG_CH_EN,
                              pDio2ReconfigureEgrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT0_RAM_MMR[i].DIO2_E_TABLE0_CTRL_CFG = tempReg;
        }
    } else if (idx == 1) {
        for (i=(0 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT1_RAM_MMR[i].DIO2_E_TABLE1_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEgrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE1_CTRL_CFG_CH_ID,
                          pDio2ReconfigureEgrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE1_CTRL_CFG_CH_EN,
                              pDio2ReconfigureEgrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT1_RAM_MMR[i].DIO2_E_TABLE1_CTRL_CFG = tempReg;
        }
    } else if (idx == 2){
        for (i=(0 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureEgrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_E_DBCNT2_RAM_MMR[i].DIO2_E_TABLE2_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_E_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureEgrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_E_TABLE2_CTRL_CFG_CH_ID,
                          pDio2ReconfigureEgrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_E_TABLE2_CTRL_CFG_CH_EN,
                              pDio2ReconfigureEgrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_E_DBCNT2_RAM_MMR[i].DIO2_E_TABLE2_CTRL_CFG = tempReg;
        }
    }

    return IQN2FL_SOK;
}

/** ============================================================================
 *   @n@b Iqn2Fl_setupDio2ReconfigureIngressEngine
 *
 *   @b Description
 *   @n In cases where:
 *      - DIO parameters need to be adjusted by, for instance Physical Layer software
 *      - IQN2 is configured via LLD from ARM Linux or any other software entity external to, for instance Physical layer software
 *      IQN2 functional layer has dedicated hardware control commands and auxiliary APIs that can be used prior to enabling the AT2 events.
 *      Iqn2Fl_setupDio2ReconfigureIngressEngine() is an API that allows to reconfigure the following Ingress DIO parameters.
 *
 *   @b Arguments
 *   @verbatim

         hIqn2                        Handle to the iqn2 instance
         pDio2ReconfigureEgrEngine    Pointer containing "Iqn2Fl_Dio2ReconfigureEgrEngineSetup" properties for IQN2.

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
 *   @n DIO2_CORE_INGRESS
 *
 *   @b Example
 *   @verbatim
        Iqn2Fl_setupDio2ReconfigureIngressEngine (hIqn2, &pDio2ReconfigureEgrEngine);
     @endverbatim
 * ===========================================================================
 */
static inline
Iqn2Fl_Status Iqn2Fl_setupDio2ReconfigureIngressEngine(
    Iqn2Fl_Handle hIqn2,
    Iqn2Fl_Dio2ReconfigureIngrEngineSetup *pDio2ReconfigureIngrEngine
)
{
    uint32_t   tempReg;
    uint32_t   idx, i;

    idx = pDio2ReconfigureIngrEngine->engine_idx;

    /* Setup DIO2 CORE INGRESS - E TABLE SEL CFG */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE_SEL_CFG_BCN_TABLE_SEL,
                      pDio2ReconfigureIngrEngine->bcn_table_sel) |
              CSL_FMK(IQN_DIO2_DIO2_I_TABLE_SEL_CFG_DMA_NUM_AXC,
                      pDio2ReconfigureIngrEngine->dma_num_axc);
    hIqn2->regs->Dio2.DIO2_CORE_INGRESS[idx].DIO2_I_TABLE_SEL_CFG = tempReg;

    /* Setup DIO2 CORE INGRESS - E DMA CFG0 */
    tempReg = CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_BRST_LN,
                  pDio2ReconfigureIngrEngine->dma_cfg0.dma_brst_ln) |
              CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_NUM_QWD,
                  pDio2ReconfigureIngrEngine->dma_cfg0.dma_num_qwd) |
              CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_RSA_CNVRT_EN,
                  pDio2ReconfigureIngrEngine->dma_cfg0.rsa_cnvrt_en) |
              CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_ENG_EN,
                  pDio2ReconfigureIngrEngine->dma_cfg0.dma_eng_en) |
              CSL_FMK(IQN_DIO2_DIO2_I_DMA_CFG0_DMA_NUM_BLKS,
                  pDio2ReconfigureIngrEngine->dma_cfg0.dma_num_blks);
    hIqn2->regs->Dio2.DIO2_CORE_INGRESS[idx].DIO2_I_DMA_CFG0 = tempReg;

    /* Setup DIO2 CORE INGRESS - E DMA CFG1 */
    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_INGRESS[idx].DIO2_I_DMA_CFG1,
        IQN_DIO2_DIO2_I_DMA_CFG1_DMA_BLK_ADDR_STRIDE,
        pDio2ReconfigureIngrEngine->dma_cfg1_dma_blk_addr_stride);

    if(idx == 0)
    {
        for (i=(0 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT0_RAM_MMR[i].DIO2_I_TABLE0_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE0_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureIngrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE0_CTRL_CFG_CH_ID,
                          pDio2ReconfigureIngrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE0_CTRL_CFG_CH_EN,
                              pDio2ReconfigureIngrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT0_RAM_MMR[i].DIO2_I_TABLE0_CTRL_CFG = tempReg;
        }
    } else if (idx == 1) {
        for (i=(0 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT1_RAM_MMR[i].DIO2_I_TABLE1_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE1_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureIngrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE1_CTRL_CFG_CH_ID,
                          pDio2ReconfigureIngrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE1_CTRL_CFG_CH_EN,
                              pDio2ReconfigureIngrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT1_RAM_MMR[i].DIO2_I_TABLE1_CTRL_CFG = tempReg;
        }
    } else if (idx == 2){
        for (i=(0 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i< (16 + (16*pDio2ReconfigureIngrEngine->bcn_table_sel)); i++)
        {
            CSL_FINS(hIqn2->regs->Dio2.DIO2_I_DBCNT2_RAM_MMR[i].DIO2_I_TABLE2_BASE_ADDR_CFG,
                IQN_DIO2_DIO2_I_TABLE2_BASE_ADDR_CFG_DMA_VBUS_BASE_ADDR_AXC,
                pDio2ReconfigureIngrEngine->axc_buffer_start_addr[i]);

            tempReg = CSL_FMK(IQN_DIO2_DIO2_I_TABLE2_CTRL_CFG_CH_ID,
                          pDio2ReconfigureIngrEngine->ch_id[i]) |
                      CSL_FMK(IQN_DIO2_DIO2_I_TABLE2_CTRL_CFG_CH_EN,
                              pDio2ReconfigureIngrEngine->ch_en[i]);
            hIqn2->regs->Dio2.DIO2_I_DBCNT2_RAM_MMR[i].DIO2_I_TABLE2_CTRL_CFG = tempReg;
        }
    }

    return IQN2FL_SOK;
}

#ifdef __cplusplus
}
#endif

#endif /* _IQN2FLHWCONTROLAUX_H_ */


