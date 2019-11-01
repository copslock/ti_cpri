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

/** ===========================================================================
 *   @file  iqn2fl_hwControl.c
 *
 *   @brief  IQN2 HW control functional layer function
 *
 */

#include <ti/drv/iqn2/iqn2fl.h>
#include <ti/drv/iqn2/iqn2fl_hwControlAux.h>

/** ===========================================================================
 *   @n@b Iqn2Fl_hwControl
 *
 *   @b Description
 *   @n This function performs various control operations on iqn2 system,
 *      based on the command passed.
 *
 *   @b Arguments
 *   @verbatim
            hIqn2       Handle to the iqn2 instance
 
            cmd         Operation to be performed on the iqn2
 
            arg         Argument specific to the command 
 
     @endverbatim
 *
 *   <b> Return Value </b>  Iqn2Fl_Status
 *   @li                    IQN2FL_SOK            - Command execution successful.
 *   @li                    IQN2FL_BADHANDLE - Invalid handle
 *   @li                    IQN2FL_INVCMD    - Invalid command
 *
 *   <b> Pre Condition </b>
 *   @n  Iqn2Fl_init(), Iqn2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  Registers of iqn2 instance are configured according to the command
 *       and the command arguments. The command determines which registers are
 *       modified.
 *
 *   @b Writes
 *   @n Registers determined by the command
 *
 *   @b Example
 *   @verbatim
        // handle for IQN2
        Iqn2Fl_Handle hIqn2;
        // other related declarations
        ...
        // ctrl argument for hw command
        Bool ctrlArg;

        // Open handle  - for use 
        hIqn2 = Iqn2Fl_open(&Iqn2Obj, CSL_IQN, &iqn2Param, &status);

        if ((hIqn2 == NULL) || (status != IQN2FL_SOK))
        {
           printf ("\nError opening CSL_IQN");
           exit(1);
        }

        // Do config 
        Config.globalSetup = &gblCfg;
        ...
        //Do setup 
        Iqn2Fl_hwSetup(hIqn2, &Config);

        ctrlArg = 1;
        hIqn2->arg_ail = IQN2FL_AIL_0; // Using AIL_0 instance

        // Send hw control command to global enable EFE of AIL_0 
        Iqn2Fl_hwControl(hIqn2, IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET, (void *)&ctrlArg);
     @endverbatim
 * ============================================================================
 */
#ifdef _TMS320C6X
#pragma CODE_SECTION (Iqn2Fl_hwControl, ".text:iqn2fl");
#endif
Iqn2Fl_Status  Iqn2Fl_hwControl(
    Iqn2Fl_Handle          hIqn2,
    Iqn2Fl_HwControlCmd    cmd,
    void                   *arg
)
{
    uint32_t   i;
    Iqn2Fl_Status status = IQN2FL_SOK ;
    
    if (hIqn2 == NULL)
        return IQN2FL_BADHANDLE;
    
    switch (cmd) 
    {
        /** AIL EFE Global Enable Set. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_SET:
            Iqn2Fl_ailEfeGlobalEnableSet (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL EFE Global Enable Clear. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_EFE_GLOBAL_ENABLE_CLEAR:
            Iqn2Fl_ailEfeGlobalEnableClear (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL IQ EFE CFG - LOOPBACK ENABLE
         *  EFE Configuration Register.
         *  (TI use Only) 0x1: Ingress data from ICC is looped back to 
         *  Egress data to ICC. DMA traffic is unused. (i.e. for purpose 
         *  of early DFE only testing)
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
         */
        case IQN2FL_CMD_AIL_EFE_CFG_LOOPBACK_EN_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CFG, IQN_AIL_AIL_IQ_EFE_CFG_LOOPBACK_EN, value);
            }
            break;
        /** AIL IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  EFE DMA Channel Configuration Register.
         *  Use hIqn2->arg_ail to select AIL instance
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_EFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CHAN_CFG[index], IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** AIL IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  IFE DMA Channel Configuration Register.
         *  Use hIqn2->arg_ail to select AIL instance
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_IFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.AIL_IQ_IFE_CHAN_CFG[index], IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** AIL IQ IDC CH CFG - CHANNEL FORCE OFF
         *  IDC Channel Configuration Register.
         *  Use hIqn2->arg_ail to select AIL instance
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_AIL_IDC_CH_CFG_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_IDC_CHANNEL_CONFIG_GROUP.AIL_IQ_IDC_CH_CFG[index], IQN_AIL_AIL_IQ_IDC_CH_CFG_CHAN_FRC_OFF, CSL_IQN_AIL_AIL_IQ_IDC_CH_CFG_CHAN_FRC_OFF_FRC_OFF);
            }
            break;
        /**  AIL IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
         *  EFE DMA Channel Configuration Register.
         *  Use hIqn2->arg_ail to select AIL instance
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_AIL_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_EFE_CONFIG_GROUP.AIL_IQ_EFE_CHAN_CFG[index], IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, CSL_IQN_AIL_AIL_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF_FRC_SYM_OFF);
            }
            break;
        /** AIL SI IQ E SCH PHY - PE DMA Channel 1 Register 
         *  SI Egress AIL scheduler, PHY FSM is enabled to turn ON, 
         *  will turn on next PE_FB from uAT
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
         */
        case IQN2FL_CMD_AIL_SI_IQ_E_SCH_PHY_EN_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_SI_IQ_E_SCH_PHY.AIL_IQ_PE_PHY_CFG, IQN_AIL_AIL_IQ_PE_PHY_CFG_PHY_EN, value);
            }
            break;
        /** AIL ECTL Global Enable Set. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_SET:
            Iqn2Fl_ailEctlGlobalEnableSet (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL ECTL Global Enable Clear. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_ECTL_GLOBAL_ENABLE_CLEAR:
            Iqn2Fl_ailEctlGlobalEnableClear (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL SI IQ E SCH CPRI - PE CPRI RADSTD CFG. 
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
         *  (argument type: Iqn2Fl_AilSiIqCpriRadstdCfg * )
         */
        case IQN2FL_CMD_AIL_PE_CPRI_RADSTD_CFG:
            Iqn2Fl_setupAilEgrSchCpriRadstdCfgRegs (hIqn2, (Iqn2Fl_AilSiIqCpriRadstdCfg *)arg);
            break;
        /** AIL IFE Global Enable Set. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_SET:
            Iqn2Fl_ailIfeGlobalEnableSet (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL IFE Global Enable Clear. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_IFE_GLOBAL_ENABLE_CLEAR:
            Iqn2Fl_ailIfeGlobalEnableClear (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL ICTL Global Enable Set. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_SET:
            Iqn2Fl_ailIctlGlobalEnableSet (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL ICTL Global Enable Clear. Use hIqn2->arg_ail to select 
         *  AIL instance (argument type: uint32_t * ) 
         */
        case IQN2FL_CMD_AIL_ICTL_GLOBAL_ENABLE_CLEAR:
            Iqn2Fl_ailIctlGlobalEnableClear (hIqn2, *(uint32_t *)arg);
            break;
        /** AIL ECTL RATE CTL CFG - ECTL Rate Control Configuration register
         *  Rate Controller will allow the ECTL to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AIL_ECTL_RATE_CTL_CFG_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_ECTL_REGISTER_GROUP.AIL_ECTL_RATE_CTL_CFG, IQN_AIL_AIL_ECTL_RATE_CTL_CFG_RATE, value);
            }
            break;
        /** AIL ICTL RATE CTL CFG - ICTL Rate Control Configuration register
         *  Rate Controller will allow the ICTL to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AIL_ICTL_RATE_CTL_CFG_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_CTL_INGRESS_VBUS_MMR_GROUP.AIL_ICTL_RATE_CTL_CFG, IQN_AIL_AIL_ICTL_RATE_CTL_CFG_RATE, value);
            }
            break;
        /** AIL IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
         *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AIL_IQ_IDC_RATE_CTL_CFG_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQ_INGRESS_VBUS_MMR_GROUP.AIL_IQ_IDC_RATE_CTL_CFG, IQN_AIL_AIL_IQ_IDC_RATE_CTL_CFG_RATE, value);
            }
            break;

        /** AIL UAT GEN CTL -> UAT_RUN and DIAG SYNC
         *  This register simply starts the uAT timers running. It is implied 
         *  that SW is unable to precisely time the start of timers. The 
         *  intent is for the SW to correct the timers by later writting to 
         *  the offset register of each timer.
         *   UAT run starts the BCN and RAD counters free running.
         *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
         *     and an AT sync is received. This is only used in simulation and 
         *     for diagnostics.
         *  (argument type: Iqn2Fl_UatCfg* )
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t * )
         */
        case IQN2FL_CMD_AIL_UAT_GEN_CTL_UAT_CFG_REG:
            {
                Iqn2Fl_setupAilUatGenCtlUatCfgRegs (hIqn2, (Iqn2Fl_UatCfg *)arg);
            }
            break;
        /** AIL UAT GEN CTL -> AIL UAT BCN TERMINAL COUNT REGISTER. 
         *  UAT BCN terminal count. BCN counts from zero to this limit 
         *  and wraps to zero. Program as 2,457,599 for sys_clk=245.76MHz 
         *  and 3,071,999 for sys_clk=307.2MHz.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_TERMINAL_COUNT_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_GEN_CTL.AIL_UAT_BCN_TC_CFG, IQN_AIL_AIL_UAT_BCN_TC_CFG_VAL, value);
            }
            break;
        /** AIL UAT GEN CTL -> AIL UAT BCN OFFSET REGISTER. 
         *  Offset correction to the raw uAT BCN counter. Used to correct 
         *  the alignment of the local uAT BCN to the master AT2 BCN. BCN is 
         *  initially randomly started. SW uses uat_sync_bcn_capture_sts 
         *  rd_val to calculate offset correction factor. This correction 
         *  factor will be Frame size - captured value.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_GEN_CTL_BCN_OFFSET_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_GEN_CTL.AIL_UAT_BCN_OFFSET_CFG, IQN_AIL_AIL_UAT_BCN_OFFSET_CFG_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT PIMAX CFG REGISTER. 
         *  (AIL use only) PI max window. One of two values which indicate 
         *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
         *  outside this window, and error is indicated (EE).
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_PIMAX_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_PIMAX_CFG, IQN_AIL_AIL_UAT_PIMAX_CFG_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT PIMIN CFG REGISTER. 
         *  (AIL use only) PI min window. One of two values which indicate 
         *  the legal range for OBSAI or CPRI PHY SOF. When an SOF is recieved 
         *  outside this window, and error is indicated (EE).
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_PIMIN_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_PIMIN_CFG, IQN_AIL_AIL_UAT_PIMIN_CFG_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT TM FRAME COUNT (BFN) CONFIG REGISTER. 
         *  (AIL CPRI use only) uAT CPRI BFN count value (Write only). SW 
         *  overwrite current value. uAT will increment every TM_FRM_STB.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint16_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_TM_BFN_CFG_REG:
            {
                uint16_t value = *(uint16_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_TM_BFN_CFG, IQN_AIL_AIL_UAT_TM_BFN_CFG_WR_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT RT FRAME BOUNDARY (FB) COMPARE REGISTER. 
         *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
         *  Used for Egress PHY timing. RT_STB is the latest moment that RT 
         *  will wait for PE and CI SOF contribution before progressing without 
         *  either input.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_RT_FB_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_RT_FB_CFG, IQN_AIL_AIL_UAT_RT_FB_CFG_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT PE FRAME BOUNDARY (FB) COMPARE REGISTER. 
         *  (AIL use only) uAT BCN compare value which cause PE_STB to fire. 
         *  Used for Egress PROTO & PHY timing. PE_STB is the exact time which
         *  PE will start building the PHY protocol. It also represents the 
         *  latest timing for incoming DMA data to contribute to the PHY. Late 
         *  DMA data is rejected by PE.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_PE_FB_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_PE_FB_CFG, IQN_AIL_AIL_UAT_PE_FB_CFG_VAL, value);
            }
            break;
        /** AIL UAT AIL REGS -> AIL UAT TM FRAME BOUNDARY (FB) COMPARE REGISTER. 
         *  (AIL use only) uAT BCN compare value which cause RT_STB to fire. 
         *  Used for Egress PHY timing. TM_STB (OBSAI Delta) is the precise 
         *  time at which TM exports the CPRI or OBSAI PHY SOF.
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_UAT_TM_FB_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_AIL_REGS.AIL_UAT_TM_FB_CFG, IQN_AIL_AIL_UAT_TM_FB_CFG_VAL, value);
            }
            break;
        /** AIL UAT EGR RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
         *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
         *  sys_clk=245.76MHz).
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_EGR_RADT_TC_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_EGR_RADT[index].AIL_UAT_EGR_RADT_TC_CFG, IQN_AIL_AIL_UAT_EGR_RADT_TC_CFG_VAL, value);
            }
            break;
        /** AIL UAT EGR RADT -> AIL UAT RADT OFFSET REGISTER. 
         *  UAT RADT offset. Value which is added to the raw RADT 
         *  as a timing correction. RadT is initially randomly started, 
         *  SW uses radt_capture value to calculate offset correction factor.
         *  This correction factor will be Frame size - captured value.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_EGR_RADT_OFFSET_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_EGR_RADT[index].AIL_UAT_EGR_RADT_OFFSET_CFG, IQN_AIL_AIL_UAT_EGR_RADT_OFFSET_CFG_VAL, value);
            }
            break;
        /** AIL UAT ING RADT -> AIL UAT RADT TERMINAL COUNT REGISTER. 
         *  UAT RADT terminal count. (i.e. 2,457,599 for WCDMA with 
         *  sys_clk=245.76MHz).
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_ING_RADT_TC_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_ING_RADT[index].AIL_UAT_ING_RADT_TC_CFG, IQN_AIL_AIL_UAT_ING_RADT_TC_CFG_VAL, value);
            }
            break;
        /** AIL UAT ING RADT -> AIL UAT RADT OFFSET REGISTER. 
         *  UAT RADT offset. Value which is added to the raw RADT 
         *  as a timing correction. RadT is initially randomly started, 
         *  SW uses radt_capture value to calculate offset correction factor.
         *  This correction factor will be Frame size - captured value.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_ING_RADT_OFFSET_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_ING_RADT[index].AIL_UAT_ING_RADT_OFFSET_CFG, IQN_AIL_AIL_UAT_ING_RADT_OFFSET_CFG_VAL, value);
            }
            break;
        /** AIL UAT RADT EVT -> AIL UAT RADT EVENT COMPARE REGISTER. 
         *  UAT RADT event compare per RADT. When compare value equals RADT 
         *  count, frame rate event is generated. Also periodic event (i.e. 4SAMP)
         *  is started. The 0 to 7 are for si egress, 8 to 15 for si ingress, 
         *  16 to 18 for dio egress, 19 to 21 for dio ingress.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_EVT_RADT_CMP_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_RADT_EVT[index].AIL_UAT_EVT_RADT_CMP_CFG, IQN_AIL_AIL_UAT_EVT_RADT_CMP_CFG_VAL, value);
            }
            break;
        /** AIL UAT RADT EVT -> AIL UAT RADT EVENT CLOCK COUNT TC REGISTER. 
         *  UAT RADT event clock counter terminal count controls spacing of the 
         *  periodic strobe (i.e. 4SAMP). Once the uat_evt_radt_cmp_cfg equals 
         *  the RADT, the period strobe will fire and re-fire every time a 
         *  clock counter reaches this terminal count. The The 0 to 7 are for si 
         *  egress, 8 to 15 for si ingress, 16 to 18 for dio egress, 19 to 21 
         *  for dio ingress.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AIL_UAT_EVT_CLK_CNT_TC_CFG_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_UAT_RADT_EVT[index].AIL_UAT_EVT_CLK_CNT_TC_CFG, IQN_AIL_AIL_UAT_EVT_CLK_CNT_TC_CFG_VAL, value);
            }
            break;
        /** AIL PD CPRI AXC CFG - PD CPRI RADSTD CFG. 
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint8_t *) 
         *  (argument type: Iqn2Fl_AilSiIqCpriRadstdCfg * )
         */
        case IQN2FL_CMD_AIL_PD_CPRI_AXC_RADSTD_CFG:
            Iqn2Fl_setupAilPdCpriAxcRadstdCfgRegs (hIqn2, (Iqn2Fl_AilSiIqCpriRadstdCfg *)arg);
            break;
        /** AIL PHY CI LUT -> CI LUT CFG 
         *  The AIL PHY CI LUT Select Register selects between the LUT A 
         *  table and the LUT B table for CI CPRI Conversion control. 
         *  Used for dynamic modification of CI effectively giving the 
         *  user a Ping Pong buffer. Select takes effect on next PHY 
         *  frame boundary.
         *  0 Selects Table A, 1 Selects Table B
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_PHY_CI_LUT_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CI_LUT.AIL_PHY_CI_LUT_CFG, IQN_AIL_AIL_PHY_CI_LUT_CFG_SEL, value);
            }
            break;
        /** AIL PHY CO LUT -> CO LUT CFG 
         *  The AIL PHY CO LUT Select Register selects between the LUT A 
         *  table and the LUT B table for CO CPRI Conversion control. 
         *  Used for dynamic modification of CO effectively giving the 
         *  user a Ping Pong buffer. Select takes effect on next PHY 
         *  frame boundary.
         *  0 Selects Table A, 1 Selects Table B
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_PHY_CO_LUT_CFG_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_CO_LUT.AIL_PHY_CO_LUT_CFG, IQN_AIL_AIL_PHY_CO_LUT_CFG_SEL, value);
            }
            break;
        /** AIL PHY CI LUT A -> LUT A CFG
         *  The AIL PHY CI Look-up Table A Registers control up to eight 
         *  groups of AxC containers for every given CPRI basic frame.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_AilPhyLutSetup * )
         */
        case IQN2FL_CMD_AIL_PHY_CI_LUTA_CFG_REG:
            {
                Iqn2Fl_setupAilPhyCiLutACfgRegs (hIqn2, (Iqn2Fl_AilPhyLutSetup *)arg);
            }
            break;
        /** AIL PHY CI LUT B -> LUT B CFG
         *  The AIL PHY CI Look-up Table B Registers control up to eight 
         *  groups of AxC containers for every given CPRI basic frame.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_AilPhyLutSetup * )
         */
        case IQN2FL_CMD_AIL_PHY_CI_LUTB_CFG_REG:
            {
                Iqn2Fl_setupAilPhyCiLutBCfgRegs (hIqn2, (Iqn2Fl_AilPhyLutSetup *)arg);
            }
            break;
        /** AIL PHY CO LUT A -> LUT A CFG
         *  The AIL PHY CO Look-up Table A Registers control up to eight 
         *  groups of AxC containers for every given CPRI basic frame.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_AilPhyLutSetup * )
         */
        case IQN2FL_CMD_AIL_PHY_CO_LUTA_CFG_REG:
            {
                Iqn2Fl_setupAilPhyCoLutACfgRegs (hIqn2, (Iqn2Fl_AilPhyLutSetup *)arg);
            }
            break;
        /** AIL PHY CO LUT B -> LUT B CFG
         *  The AIL PHY CO Look-up Table B Registers control up to eight 
         *  groups of AxC containers for every given CPRI basic frame.
         *  Use hIqn2->arg_ail to select AIL instance 
         *  (argument type: Iqn2Fl_AilPhyLutSetup * )
         */
        case IQN2FL_CMD_AIL_PHY_CO_LUTB_CFG_REG:
            {
                Iqn2Fl_setupAilPhyCoLutBCfgRegs (hIqn2, (Iqn2Fl_AilPhyLutSetup *)arg);
            }
            break;
        /** AIL PHY TM CFG -> ENABLE/DISABLE
         *  The TM Configuration Register is used to program basic 
         *  functionality of the Tx Mac Block. Bit 0 can be updated at any 
         *  time to turn on or off the link.
         *  The Transmit Enable Bit allows the TM block transmit state 
         *  machine to operate
         *     DISABLE (0) = TM Block Disabled
         *     ENABLE (1) = TM Block Enabled
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_PHY_TM_CFG_EN_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_TM.AIL_PHY_TM_CFG, IQN_AIL_AIL_PHY_TM_CFG_EN, value);
            }
            break;
        /** AIL PHY RM CFG -> RX ENABLE/DISABLE
         *  RM Configuration Register
         *  Enable RM Link FSM to activate on next recieved PHY frame boundary
         *     DISABLE (0) = RM link disable
         *     ENABLE (1) = RM link enable
         *  Use hIqn2->arg_ail to select AIL instance (argument type: uint32_t *) 
         */
        case IQN2FL_CMD_AIL_PHY_RM_CFG_RX_EN_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Ail[hIqn2->arg_ail].AIL_PHY_RM.AIL_PHY_RM_CFG, IQN_AIL_AIL_PHY_RM_CFG_RX_EN, value);
            }
            break;
        /** AIL IQ EFE FRM SAMP TC CFG update for a symbol index of a radio standard
         *  Command to update the Radio Framing Sample Terminal Count Configuration Register
         *  Use hIqn2->arg_ail to select AIL instance
         *  (argument type: a pointer to an array of 3 uint32_t elements, radioStdId, symbol size and symbol index )
         */
        case IQN2FL_CMD_AIL_UPDATE_EGRESS_RADSTD_TC_REG:
            {
                uint32_t *update_params = (uint32_t *)arg;
                Iqn2Fl_updateEgressAilRadioStandardTc(hIqn2, update_params[0], update_params[1], update_params[2]);
            }
            break;
        /** TOP VC SYS STS CFG - VC SW RESET STB - VC software reset register.
         *  These are software resets which reset various sections of IQN2.
         *  (argument type: Iqn2Fl_TopVCSwResetStbSetup* )
         */
        case IQN2FL_CMD_TOP_VC_SYS_STS_CFG_SW_RESET_STB:
            {
                Iqn2Fl_setupTopVcSysStsSwResetStbRegs (hIqn2, (Iqn2Fl_TopVCSwResetStbSetup *)arg);
            }
            break;
        /** AID2 IQ EFE GLOBAL EN SET STB
         *  Set Global Enable for EFE.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_GLOBAL_EN_SET_STB, IQN_AID2_AID2_IQ_EFE_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** AID2 IQ EFE GLOBAL EN CLR STB
         *  Clear Global Enable for EFE.
         *  A write of any value to this register which clears (enables) global enable.
         */
        case IQN2FL_CMD_AID2_EFE_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_GLOBAL_EN_CLR_STB, IQN_AID2_AID2_IQ_EFE_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** AID2 IQ EFE CONFIG GROUP - EFE CFG - LOOPBACK EN
         *  EFE Configuration Register.
         *  (TI use Only) 0x1: Ingress data from ICC is looped back to Egress 
         *  data to ICC. DMA traffic is unused. (i.e. for purpose of early 
         *  DFE only testing)
         */
        case IQN2FL_CMD_AID2_EFE_CONFIG_LOOPBACK_EN:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CFG, IQN_AID2_AID2_IQ_EFE_CFG_LOOPBACK_EN, value);
            }
            break;
		/** AID2 IQ EFE RADIO STANDARD SCHEDULER CONFIGURATION REGISTER LENGHT
		 *
		 */
		case IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_LEN:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP.AID2_IQ_EFE_RAD_STD_SCH_CFG[index], IQN_AID2_AID2_IQ_EFE_RAD_STD_SCH_CFG_TDM_LEN, value);
			}
			break;
		/** AID2 IQ EFE RADIO STANDARD SCHEDULER CONFIGURATION REGISTER ENABLE
		 */
		case IQN2FL_CMD_AID2_IQ_EFE_RAD_STD_SCH_CFG_EN:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_EFE_RADIO_STANDARD_SCHEDULER_GROUP.AID2_IQ_EFE_RAD_STD_SCH_CFG[index], IQN_AID2_AID2_IQ_EFE_RAD_STD_SCH_CFG_TDM_EN, value);
			}
			break;
        /** AID2 IQ IFE GLOBAL EN SET STB
         *  Set Global Enable for IFE.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_CONFIG_GROUP.AID2_IQ_IFE_GLOBAL_EN_SET_STB, IQN_AID2_AID2_IQ_IFE_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** AID2 IQ IFE GLOBAL EN CLR STB
         *  Clear Global Enable for IFE.
         *  A write of any value to this register which clears (enables) global enable.
         */
        case IQN2FL_CMD_AID2_IFE_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_CONFIG_GROUP.AID2_IQ_IFE_GLOBAL_EN_CLR_STB, IQN_AID2_AID2_IQ_IFE_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** AID2 IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  EFE DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AID2_EFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CHAN_CFG[index], IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** AID2 IQ ECTL CHAN CFG - CHANNEL ENABLE/DISABLE
         *  ECTL DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AID2_ECTL_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_CHAN_CFG[index], IQN_AID2_AID2_ECTL_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** AID2 IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  IFE DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AID2_IFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.AID2_IQ_IFE_CHAN_CFG[index], IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** AID2 IQ ICTL CHAN CFG - CHANNEL ENABLE/DISABLE
         *  ECTL DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AID2_ICTL_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Aid2.AID2_ICTL_PKT_IF.AID2_ICTL_CHAN_EN_CFG[index],  IQN_AID2_AID2_ICTL_CHAN_EN_CFG_CHAN_EN, value);
            }
            break;
        /** AID2 IQ IDC CH CFG - CHANNEL FORCE OFF
         *  IDC Channel Configuration Register.
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_AID2_IDC_CH_CFG_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IDC_CHANNEL_CONFIG_GROUP.AID2_IQ_IDC_CH_CFG[index], IQN_AID2_AID2_IQ_IDC_CH_CFG_CHAN_FRC_OFF, CSL_IQN_AID2_AID2_IQ_IDC_CH_CFG_CHAN_FRC_OFF_FRC_OFF);
            }
            break;
        /**  AID2 IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
         *  EFE DMA Channel Configuration Register.
         *  (argument type: channel index of type uint8_t *)
         */

        case IQN2FL_CMD_AID2_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_SI_IQ_EFE_CONFIG_GROUP.AID2_IQ_EFE_CHAN_CFG[index], IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, CSL_IQN_AID2_AID2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF_FRC_SYM_OFF);
            }
            break;
        /**  AID2 IQ IFE CHAN CFG - TDD CHANNEL FORCE OFF
         *  IFE DMA Channel Configuration Register.
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_AID2_IFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.AID2_IQ_IFE_CHAN_CFG[index], IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_TDD_FRC_OFF, CSL_IQN_AID2_AID2_IQ_IFE_CHAN_CFG_CHAN_TDD_FRC_OFF_FRC_SYM_OFF);
            }
            break;
        /** AID2 ECTL GLOBAL EN SET STB
         *  Set Global Enable for ECTL.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_GLOBAL_EN_SET_STB, IQN_AID2_AID2_ECTL_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** AID2 ECTL GLOBAL EN CLR STB
         *  Clear Global Enable for ECTL.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_ECTL_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_PKT_IF.AID2_ECTL_GLOBAL_EN_CLR_STB, IQN_AID2_AID2_ECTL_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** AID2 ICTL GLOBAL EN SET STB
         *  Set Global Enable for ICTL.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_ICTL_PKT_IF.AID2_ICTL_GLOBAL_EN_SET_STB, IQN_AID2_AID2_ICTL_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** AID2 ICTL GLOBAL EN CLR STB
         *  Clear Global Enable for ICTL.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_AID2_ICTL_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_ICTL_PKT_IF.AID2_ICTL_GLOBAL_EN_CLR_STB, IQN_AID2_AID2_ICTL_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** AID2 ECTL RATE CTL CFG - ECTL Rate Control Configuration register
         *  Rate Controller will allow the ECTL to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the ECTL to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AID2_ECTL_RATE_CTL_CFG_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_ECTL_REGISTER_GROUP.AID2_ECTL_RATE_CTL_CFG, IQN_AID2_AID2_ECTL_RATE_CTL_CFG_RATE, value);
            }
            break;
        /** AID2 ICTL RATE CTL CFG - ICTL Rate Control Configuration register
         *  Rate Controller will allow the ICTL to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the ICTL to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AID2_ICTL_RATE_CTL_CFG_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_CTL_INGRESS_VBUS_MMR_GROUP.AID2_ICTL_RATE_CTL_CFG, IQN_AID2_AID2_ICTL_RATE_CTL_CFG_RATE, value);
            }
            break;
         /** AID2 IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
         *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
         *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
         *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
         *  (argument type: uint8_t *)
         */
        case IQN2FL_CMD_AID2_IQ_IDC_RATE_CTL_REG:
            {
                uint8_t value = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Aid2.AID2_IQ_INGRESS_VBUS_MMR_GROUP.AID2_IQ_IDC_RATE_CTL_CFG, IQN_AID2_AID2_IQ_IDC_RATE_CTL_CFG_RATE, value);
            }
            break;
        /** AID2 UAT CFG - UAT_RUN and DIAG SYNC
         *  This register simply starts the uAT timers running. It is implied 
         *  that SW is unable to precisely time the start of timers. The 
         *  intent is for the SW to correct the timers by later writting to 
         *  the offset register of each timer.
         *   UAT run starts the BCN and RAD counters free running.
         *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
         *     and an AT sync is received. This is only used in simulation and 
         *     for diagnostics.
         *  (argument type: Iqn2Fl_UatCfg* )
         */
        case IQN2FL_CMD_AID2_UAT_GEN_CTL_UAT_CFG:
            {
                Iqn2Fl_setupAid2UatGenCtlUatCfgRegs (hIqn2, (Iqn2Fl_UatCfg *)arg);
            }
            break;
        /** AID2 UAT EGR RADT - OFFSET CFG
         *  UAT RADT offset Register.
         *  UAT RADT offset. Value which is added to the raw RADT as a timing 
         *  correction. RadT is initially randomly started, SW uses 
         *  radt_capture value to calculate offset correction factor. This 
         *  correction factor will be Frame size - captured value.
         *  (argument type: Iqn2Fl_RadtOffsetCfg* )
         */
        case IQN2FL_CMD_AID2_UAT_EGR_RADT_OFFSET_CFG:
            {
                Iqn2Fl_setupAid2UatEgrRadtOffsetCfgRegs(hIqn2, (Iqn2Fl_RadtOffsetCfg *)arg);
            }
            break;
        /** AID2 UAT ING RADT - OFFSET CFG
         *  UAT RADT offset Register.
         *  UAT RADT offset. Value which is added to the raw RADT as a timing 
         *  correction. RadT is initially randomly started, SW uses 
         *  radt_capture value to calculate offset correction factor. This 
         *  correction factor will be Frame size - captured value.
         *  (argument type: Iqn2Fl_RadtOffsetCfg* )
         */
        case IQN2FL_CMD_AID2_UAT_ING_RADT_OFFSET_CFG:
            {
                Iqn2Fl_setupAid2UatIngRadtOffsetCfgRegs(hIqn2, (Iqn2Fl_RadtOffsetCfg *)arg);
            }
            break;
        /** AID2 UAT RADT EVT
         *  Configures UAT RADT event compare Register per RADT and 
         *  UAT RADT event clock counter terminal count Register per RADT.
         *  (argument type: Iqn2Fl_UatRadtEvtSetup* )
         */
        case IQN2FL_CMD_AID2_UAT_RADT_EVT_PER_RADT:
            {
                Iqn2Fl_setupAid2UatRadtEvtRegs (hIqn2, (Iqn2Fl_UatRadtEvtSetup *)arg);
            }
            break;
        /** AID2 IQ EFE FRM SAMP TC CFG update for a symbol index of a radio standard
         *  Command to update the Radio Framing Sample Terminal Count Configuration Register
         *  (argument type: a pointer to an array of 3 uint32_t elements, radioStdId, symbol size and symbol index )
         */
        case IQN2FL_CMD_AID2_UPDATE_EGRESS_RADSTD_TC_REG:
            {
                uint32_t *update_params = (uint32_t *)arg;
                Iqn2Fl_updateEgressAid2RadioStandardTc(hIqn2, update_params[0], update_params[1], update_params[2]);
            }
            break;
		/*
		 * Sets egress PKTDMA destination port and channel. Also sets
		 */
        case IQN2FL_CMD_IQS_EGR_PKTDMA_CFG_CHAN:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Iqs2.IQS_EGRESS_CHAN_CONFIG.IQS_EGR_PKTDMA_CFG[index],  IQN_IQS2_IQS_EGR_PKTDMA_CFG_CHAN, value);
			}
			break;
		/*
		 * Sets egress DIO destination port and channel. Also sets
		 */
		case IQN2FL_CMD_IQS_EGR_DIO2_CFG_CHAN:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Iqs2.IQS_EGRESS_CHAN_CONFIG.IQS_EGR_DIO2_CFG[index],  IQN_IQS2_IQS_EGR_DIO2_CFG_CHAN, value);
			}
			break;
		/*
		 * Sets egress PKTDMA destination port and channel. Also sets
		 */
        case IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_CHAN:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AID2_AXC_LUT_CFG[index],  IQN_IQS2_IQS_ING_AID2_AXC_LUT_CFG_CHAN, value);
			}
			break;
		/*
		 * Sets egress PKTDMA destination port and channel. Also sets
		 */
		case IQN2FL_CMD_IQS_ING_AID2_AXC_LUT_CFG_DEST:
			{
				Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
				uint32_t index = pArgs->reg_index;
				uint32_t value = *((uint32_t *)(pArgs->reg_arg));
				CSL_FINS(hIqn2->regs->Iqs2.IQS_INGRESS_CHAN_CONFIG.IQS_ING_AID2_AXC_LUT_CFG[index],  IQN_IQS2_IQS_ING_AID2_AXC_LUT_CFG_DEST, value);
			}
			break;
        /** DIO2 IQ EFE GLOBAL EN SET STB
         *  Set Global Enable for EFE.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_GLOBAL_EN_SET_STB, IQN_DIO2_DIO2_IQ_EFE_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 IQ EFE GLOBAL EN CLR STB
         *  Clear Global Enable for EFE.
         *  A write of any value to this register which clears (enables) global enable.
         */
        case IQN2FL_CMD_DIO2_EFE_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_GLOBAL_EN_CLR_STB, IQN_DIO2_DIO2_IQ_EFE_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 IQ IFE GLOBAL EN SET STB
         *  Set Global Enable for IFE.
         *  A write of any value to this register which sets (enables) global enable.
         */
        case IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_CONFIG_GROUP.DIO2_IQ_IFE_GLOBAL_EN_SET_STB, IQN_DIO2_DIO2_IQ_IFE_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 IQ IFE GLOBAL EN CLR STB
         *  Clear Global Enable for IFE.
         *  A write of any value to this register which clears (enables) global enable.
         */
        case IQN2FL_CMD_DIO2_IFE_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_CONFIG_GROUP.DIO2_IQ_IFE_GLOBAL_EN_CLR_STB, IQN_DIO2_DIO2_IQ_IFE_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 IQ EFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  EFE DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_DIO2_EFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_CHAN_CFG[index], IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** DIO2 IQ IFE CHAN CFG - CHANNEL ENABLE/DISABLE
         *  IFE DMA Channel Configuration Register.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_DIO2_IFE_CHAN_CFG_CHAN_EN_REG:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IFE_CHANNEL_CONFIGURATION_GROUP.DIO2_IQ_IFE_CHAN_CFG[index], IQN_DIO2_DIO2_IQ_IFE_CHAN_CFG_CHAN_EN, value);
            }
            break;
        /** DIO2 IQ IDC CH CFG - CHANNEL FORCE OFF
         *  IDC Channel Configuration Register.
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_DIO2_IDC_CH_CFG_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_IDC_CHANNEL_CONFIG_GROUP.DIO2_IQ_IDC_CH_CFG[index], IQN_DIO2_DIO2_IQ_IDC_CH_CFG_CHAN_FRC_OFF, CSL_IQN_DIO2_DIO2_IQ_IDC_CH_CFG_CHAN_FRC_OFF_FRC_OFF);
            }
            break;
        /**  DIO2 IQ EFE CHAN CFG - TDD CHANNEL FORCE OFF
         *  EFE DMA Channel Configuration Register.
         *  (argument type: channel index of type uint8_t *)
         */
        case IQN2FL_CMD_DIO2_EFE_CHAN_CFG_TDD_CHAN_FRC_OFF_REG:
            {
                uint8_t index = *(uint8_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_SI_IQ_EFE_CONFIG_GROUP.DIO2_IQ_EFE_CHAN_CFG[index], IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF, CSL_IQN_DIO2_DIO2_IQ_EFE_CHAN_CFG_CHAN_TDD_FRC_OFF_FRC_SYM_OFF);
            }
            break;
       /** DIO2 IQ IDC RATE CTL CFG - IDC Rate Control Configuration register
        *  Rate Controller will allow the IDC to create RATE+1 active requests on the PSI bus within
        *  a 16 clock cycle window. As an example, a value of 7 will allow the IDC to create 8 active
        *  requests within a 16-clock cycle window which uses 50% of the PSI bus.
        *  (argument type: uint8_t *)
        */
       case IQN2FL_CMD_DIO2_IQ_IDC_RATE_CTL_REG:
           {
               uint8_t value = *(uint8_t *)arg;
               CSL_FINS(hIqn2->regs->Dio2.DIO2_IQ_INGRESS_VBUS_MMR_GROUP.DIO2_IQ_IDC_RATE_CTL_CFG, IQN_DIO2_DIO2_IQ_IDC_RATE_CTL_CFG_RATE, value);
           }
           break;
        /** DIO2 UAT CFG - UAT_RUN and DIAG SYNC
         *  This register simply starts the uAT timers running. It is implied 
         *  that SW is unable to precisely time the start of timers. The 
         *  intent is for the SW to correct the timers by later writting to 
         *  the offset register of each timer.
         *   UAT run starts the BCN and RAD counters free running.
         *   diag_sync = 1 starts the BCN and RAD counters if uat_run is set 
         *     and an AT sync is received. This is only used in simulation and 
         *     for diagnostics.
         *  (argument type: Iqn2Fl_UatCfg* )
         */
        case IQN2FL_CMD_DIO2_UAT_GEN_CTL_UAT_CFG:
            {
                Iqn2Fl_setupDio2UatGenCtlUatCfgRegs (hIqn2, (Iqn2Fl_UatCfg *)arg);
            }
            break;
        /** DIO2 UAT RADT EVT
         *  Configures UAT RADT event compare Register per RADT and 
         *  UAT RADT event clock counter terminal count Register per RADT.
         *  (argument type: Iqn2Fl_UatRadtEvtSetup* )
         */
        case IQN2FL_CMD_DIO2_UAT_RADT_EVT_PER_RADT:
            {
                Iqn2Fl_setupDio2UatRadtEvtRegs (hIqn2, (Iqn2Fl_UatRadtEvtSetup *)arg);
            }
            break;
        /** DIO2 UAT EGR RADT - OFFSET CFG
         *  UAT RADT offset Register.
         *  UAT RADT offset. Value which is added to the raw RADT as a timing 
         *  correction. RadT is initially randomly started, SW uses 
         *  radt_capture value to calculate offset correction factor. This 
         *  correction factor will be Frame size - captured value.
         *  (argument type: Iqn2Fl_RadtOffsetCfg* )
         */
        case IQN2FL_CMD_DIO2_UAT_EGR_RADT_OFFSET_CFG:
            {
                Iqn2Fl_setupDio2UatEgrRadtOffsetCfgRegs(hIqn2, (Iqn2Fl_RadtOffsetCfg *)arg);
            }
            break;
        /** DIO2 UAT DIO EGR RADT - OFFSET CFG
         *  UAT DIO egress RADT offset Register.
         *  (DIO use only) UAT DIO egress RADT offset. Value which is added 
         *  to the raw RADT as a timing correction. RadT is initially 
         *  randomly started, SW uses radt_capture value to calculate 
         *  offset correction factor. This correction factor will be 
         *  Frame size - captured value.
         *  (argument type: Iqn2Fl_RadtOffsetCfg* )
         */
        case IQN2FL_CMD_DIO2_UAT_DIO_EGR_RADT_OFFSET_CFG:
            {
                Iqn2Fl_setupDio2UatDioEgrRadtOffsetCfgRegs(hIqn2, (Iqn2Fl_RadtOffsetCfg *)arg);
            }
            break;
        /** DIO2 UAT DIO ING RADT - OFFSET CFG
         *  UAT DIO ingress RADT offset Register.
         *  (DIO use only) UAT DIO ingress RADT offset. Value which is added 
         *  to the raw RADT as a timing correction. RadT is initially 
         *  randomly started, SW uses radt_capture value to calculate 
         *  offset correction factor. This correction factor will be 
         *  Frame size - captured value.
         *  (argument type: Iqn2Fl_RadtOffsetCfg* )
         */
        case IQN2FL_CMD_DIO2_UAT_DIO_ING_RADT_OFFSET_CFG:
            {
                Iqn2Fl_setupDio2UatDioIngRadtOffsetCfgRegs(hIqn2, (Iqn2Fl_RadtOffsetCfg *)arg);
            }
            break;
        /** DIO2 I GLOBAL EN SET STB
         *  Set Global Enable for Ingress DIO. 
         *  A write of any value to this register sets the global enable
         *  for the Ingress DIO.
         */
        case IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_I_GLOBAL_EN_SET_STB, IQN_DIO2_DIO2_I_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 I GLOBAL EN CLR STB
         *  Clear Global Enable for Ingress DIO.
         *  A write of any value to this register which clears the global 
         *  enable for the Ingress DIO.
         */
        case IQN2FL_CMD_DIO2_ING_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_I_GLOBAL_EN_CLR_STB, IQN_DIO2_DIO2_I_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 E GLOBAL EN SET STB
         *  Set Global Enable for Egress DIO. 
         *  A write of any value to this register sets the global enable
         *  for the Egress DIO.
         */
        case IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_E_GLOBAL_EN_SET_STB, IQN_DIO2_DIO2_E_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 E GLOBAL EN CLR STB
         *  Clear Global Enable for Egress DIO.
         *  A write of any value to this register which clears the global 
         *  enable for the Egress DIO.
         */
        case IQN2FL_CMD_DIO2_EGR_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_E_GLOBAL_EN_CLR_STB, IQN_DIO2_DIO2_E_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 DT GLOBAL EN SET STB
         *  Set Global Enable for Data Trace DIO. 
         *  A write of any value to this register sets the global enable
         *  for the Data Trace DIO.
         */
        case IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_SET:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_DT_GLOBAL_EN_SET_STB, IQN_DIO2_DIO2_DT_GLOBAL_EN_SET_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 DT GLOBAL EN CLR STB
         *  Clear Global Enable for Data Trace DIO.
         *  A write of any value to this register which clears the global 
         *  enable for the Data Trace DIO.
         */
        case IQN2FL_CMD_DIO2_DT_GLOBAL_ENABLE_CLEAR:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_GLOBAL.DIO2_DT_GLOBAL_EN_CLR_STB, IQN_DIO2_DIO2_DT_GLOBAL_EN_CLR_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 DT START STB
         *  DIO2 DIO Data Trace Start Register.
         *  A write of any value to this register will start Data Trace 
         *  operation if dt_en = 0.
         */
        case IQN2FL_CMD_DIO2_DT_START:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Dio2.DIO2_DT.DIO2_DT_START_STB, IQN_DIO2_DIO2_DT_START_STB_DONT_CARE, value);
            }
            break;
        /** DIO2 CORE INGRESS
         *  Configures DIO2 CORE INGRESS registers
         *  (argument type: Iqn2Fl_Dio2CoreIngressSetup* )
         */
        case IQN2FL_CMD_DIO2_CORE_INGRESS_CFG:
            {
                Iqn2Fl_setupDio2CoreIngressRegs (hIqn2, (Iqn2Fl_Dio2CoreIngressSetup *)arg);
            }
            break;
        /** DIO2 CORE EGRESS
         *  Configures DIO2 CORE EGRESS registers
         *  (argument type: Iqn2Fl_Dio2CoreEgressSetup* )
         */
        case IQN2FL_CMD_DIO2_CORE_EGRESS_CFG:
            {
                Iqn2Fl_setupDio2CoreEgressRegs (hIqn2, (Iqn2Fl_Dio2CoreEgressSetup *)arg);
            }
            break;
        /** DIO2 I DBCNT0/1/2 RAM MMR
         *  Configures I DBCNT0/1/2 RAM MMR registers per engine basis
         *  (argument type: Iqn2Fl_Dio2DbcntxRamMmrCfg* )
         */
        case IQN2FL_CMD_DIO2_I_DBCNT_RAM_MMR_CFG:
            {
                Iqn2Fl_Dio2DbcntxRamMmrCfg *cfg = (Iqn2Fl_Dio2DbcntxRamMmrCfg *)arg;
                if (cfg->engine_idx > 2)
                    return IQN2FL_INVPARAMS;
                Iqn2Fl_setupDio2IngDbcntxRamMmrRegs (hIqn2, cfg->engine_idx, &cfg->cfg_dbcn_cnt);
            }
            break;
        /** DIO2 E DBCNT0/1/2 RAM MMR
         *  Configures E DBCNT0/1/2 RAM MMR registers per engine basis
         *  (argument type: Iqn2Fl_Dio2DbcntxRamMmrCfg* )
         */
        case IQN2FL_CMD_DIO2_E_DBCNT_RAM_MMR_CFG:
            {
                Iqn2Fl_Dio2DbcntxRamMmrCfg *cfg = (Iqn2Fl_Dio2DbcntxRamMmrCfg *)arg;
                if (cfg->engine_idx > 2)
                    return IQN2FL_INVPARAMS;
                Iqn2Fl_setupDio2EgrDbcntxRamMmrRegs (hIqn2, cfg->engine_idx, &cfg->cfg_dbcn_cnt);
            }
            break;
        /** DIO2 CORE INGRESS DMA ENGINE enable/disable
         *  Command to only enable/disable dio ingress engines
         *  (argument type: a pointer to an array of 3 uint32_t elements, 0 for disable, 1 for enable )
         */
        case IQN2FL_CMD_DIO2_CORE_INGRESS_DMA_ENGINE_EN:
            {
                uint32_t *dma_eng_en = (uint32_t *)arg;
                for (i = 0; i < 3; i++)
                {
                    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_INGRESS[i].DIO2_I_DMA_CFG0, IQN_DIO2_DIO2_I_DMA_CFG0_DMA_ENG_EN, dma_eng_en[i]);
                }
            }
            break;
            /** DIO2 CORE EGRESS DMA ENGINE enable/disable
         *  Command to only enable/disable dio egress engines
         *  (argument type: a pointer to an array of 3 uint32_t elements, 0 for disable, 1 for enable )
         */
        case IQN2FL_CMD_DIO2_CORE_EGRESS_DMA_ENGINE_EN:
            {
                uint32_t *dma_eng_en = (uint32_t *)arg;
                for (i = 0; i < 3; i++)
                {
                    CSL_FINS(hIqn2->regs->Dio2.DIO2_CORE_EGRESS[i].DIO2_E_DMA_CFG0, IQN_DIO2_DIO2_E_DMA_CFG0_DMA_ENG_EN, dma_eng_en[i]);
                }
            }
            break;
        /** DIO2 RECONFIGURE INGRESS-EGRESS ENGINEs
         *  Reconfigures DIO2 registers
         *  (argument type: Iqn2Fl_Dio2ReconfigureEngineSetup* )
         */
        case IQN2FL_CMD_DIO2_RECONFIGURE_ENGINE:
            {
                Iqn2Fl_setupDio2ReconfigureEngine (hIqn2, (Iqn2Fl_Dio2ReconfigureEngineSetup *) arg);
            }
            break;
        /** DIO2 RECONFIGURE EGRESS ENGINE
         *  Reconfigures DIO2 Egress registers
         *  (argument type: Iqn2Fl_Dio2ReconfigureEgrEngineSetup* )
         */
        case IQN2FL_CMD_DIO2_RECONFIGURE_EGRESS_ENGINE:
            {
                Iqn2Fl_setupDio2ReconfigureEgressEngine (hIqn2, (Iqn2Fl_Dio2ReconfigureEgrEngineSetup *) arg);
            }
            break;
        /** DIO2 RECONFIGURE INGRESS ENGINE
         *  Reconfigures DIO2 Ingress registers
         *  (argument type: Iqn2Fl_Dio2ReconfigureIngrEngineSetup* )
         */
        case IQN2FL_CMD_DIO2_RECONFIGURE_INGRESS_ENGINE:
            {
                Iqn2Fl_setupDio2ReconfigureIngressEngine (hIqn2, (Iqn2Fl_Dio2ReconfigureIngrEngineSetup *) arg);
            }
            break;
        /** AT2 START - AT2 TIMER ENABLES CFG - RADT EN
         *  Control for each of the RADT and the BCN timers.
         *  bit 0-7 enable RadT0-to-RadT7. RADT 7 will be used for driving 
         *  the t1-t2-t3 GSM Timer. Once enabled, Timers will start running 
         *  once the compare value equals the BCN timer value (which yeilds 
         *  an exact sync).
         */
        case IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RADT_EN:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_START.AT2_TIMER_ENABLES_CFG, IQN_AT2_AT2_TIMER_ENABLES_CFG_RADT_EN, value);
            }
            break;
        /** AT2 START - AT2 TIMER ENABLES CFG - RUN BCN
         *  Control for each of the RADT and the BCN timers.
         *  SW control which starts the BCN Timer running. SW writes are not 
         *  precise so it is expected the user will correct the timer value 
         *  with an offset.
         */
        case IQN2FL_CMD_AT2_START_TIMER_ENABLE_CFG_RUN_BCN:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_START.AT2_TIMER_ENABLES_CFG, IQN_AT2_AT2_TIMER_ENABLES_CFG_RUN_BCN, value);
            }
            break;
        /** AT2 BCN - OFFSET CFG - VAL
         *  Offset for the free running raw BCN counter. The offset version 
         *  of BCN is the value used for all measurement and sync purposes. 
         *  (The offset mechanism gives a way to minimize clock domains 
         *  crossing errors when syncing timers). SW uses the desired sync 
         *  input capture status to calculate offset correction factor. 
         *  This correction factor will be Frame size - captured value.
         */
        case IQN2FL_CMD_AT2_BCN_OFFSET_CFG_VAL:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_OFFSET_CFG, IQN_AT2_AT2_BCN_OFFSET_CFG_VAL, value);
            }
            break;
        /** AT2 BCN - FRM INIT LSBS CFG - WR VAL
         *  BCN Frame count value (BTS Frame Number). Increments every time 
         *  10ms BCN timer wraps. This Register only allows writing of the 
         *  value (Part 2) User may write to this register while BCN is 
         *  running. Advisable to write mid frame to avoid wrap uncertaninty.
         *  BCN Frame Init LSBs.
         */
        case IQN2FL_CMD_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRM_INIT_LSBS_CFG, IQN_AT2_AT2_BCN_FRM_INIT_LSBS_CFG_WR_VAL, value);
            }
            break;
        /** AT2 BCN - FRM INIT MSBS CFG - WR VAL
         *  BCN Frame count value (BTS Frame Number). Increments every time 
         *  10ms BCN timer wraps. This Register only allows writing of the 
         *  value (Part 1) User may write to this register while BCN is 
         *  running. 
         *  BCN Frame Init MSBs.
         */
        case IQN2FL_CMD_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_BCN.AT2_BCN_FRM_INIT_MSBS_CFG, IQN_AT2_AT2_BCN_FRM_INIT_MSBS_CFG_WR_VAL, value);
            }
            break;
        /** AT2 RADT - INIT 1 CFG - FRM INIT LSB
         *  RADT Frame Init LSBs.
         *  RADT Frame Init LSBs loads a frame count directly to the counter 
         *  LSB bits. This register and the MSB register should only be loaded 
         *  when the RADT is off or if it is known it will not be incrementing 
         *  during the time of the writes.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->At2.AT2_RADT[index].AT2_RADT_INIT_1_CFG, IQN_AT2_AT2_RADT_INIT_1_CFG_FRM_INIT_LSB, value);
            }
            break;
        /** AT2 RADT - INIT 2 CFG - FRM INIT MSB
         *  RADT Frame Init MSBs.
         *  RADT Frame Init MSBs loads a frame count directly to the counter 
         *  MSB bits. This register and the LSB register should only be loaded 
         *  when the RADT is off or if it is known it will not be incrementing 
         *  during the time of the writes.
         *  (argument type: Iqn2Fl_HwControlMultiArgs*, register value type: uint32_t *)
         */
        case IQN2FL_CMD_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB:
            {
                Iqn2Fl_HwControlMultiArgs *pArgs = (Iqn2Fl_HwControlMultiArgs *)arg;
                uint32_t index = pArgs->reg_index;
                uint32_t value = *((uint32_t *)(pArgs->reg_arg));
                CSL_FINS(hIqn2->regs->At2.AT2_RADT[index].AT2_RADT_INIT_2_CFG, IQN_AT2_AT2_RADT_INIT_2_CFG_FRM_INIT_MSB, value);
            }
            break;
        /** AT2 EVENTS 24ARRAY registers configuration
         *  (argument type: Iqn2Fl_At2Events24Array* )
         */
        case IQN2FL_CMD_AT2_EVENTS_24ARRAY_CFG:
            {
                Iqn2Fl_setupAt2Events24ArrayRegs (hIqn2, (Iqn2Fl_At2Events24Array *)arg);
            }
            break;
        /** AT2 EVENTS - EVT ENABLE CFG - EN
         *  AT2 system events control APP SW timing. This MMR Enables each 
         *  of 24 of these system events.
         *  EVENT Enable when a bit is set to a 1. Must be enabled after 
         *  event configuration.  Setting the bit to 0 disables the EVENT.
         */
        case IQN2FL_CMD_AT2_EVENTS_ENABLE_CFG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_EVENTS.AT2_EVT_ENABLE_CFG, IQN_AT2_AT2_EVT_ENABLE_CFG_EN, value);
            }
            break;
        /** AT2 EVT FORCE STB
         *  For diagnostic purposes, this register allows the user to 
         *  activate any of the 24 system events with a simple registers 
         *  write. Useful for activating SW thread for testing purposes.
         *  EVENT Force.
         */
        case IQN2FL_CMD_AT2_EVT_FORCE:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->At2.AT2_EVENTS.AT2_EVT_FORCE_STB, IQN_AT2_AT2_EVT_FORCE_STB_STRB, value);
            }
            break;
        /** EE_EOI_0_REG
         *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
         *  handshake interrupt so that another EV0 interrupt can be generated.
         */
        case IQN2FL_CMD_EE_EOI_0_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Top.TOP_LEVEL_EE.EE_EOI_0_REG, IQN2_TOP_EE_EOI_0_REG_EOI_0_VECTOR_REG, value);
            }
            break;
        /** EE_EOI_1_REG
         *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
         *  handshake interrupt so that another EV1 interrupt can be generated.
         */
        case IQN2FL_CMD_EE_EOI_1_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Top.TOP_LEVEL_EE.EE_EOI_1_REG, IQN2_TOP_EE_EOI_1_REG_EOI_1_VECTOR_REG, value);
            }
            break;
        /** EE_EOI_CPPI_REG
         *  For exception handling purposes, this register allows the software to acknowledge the servicing of a
         *  handshake interrupt so that another CPPI interrupt can be generated.
         */
        case IQN2FL_CMD_EE_EOI_CPPI_REG:
            {
                uint32_t value = *(uint32_t *)arg;
                CSL_FINS(hIqn2->regs->Top.TOP_LEVEL_EE.EE_EOI_CPPI_REG, IQN2_TOP_EE_EOI_CPPI_REG_EOI_CPPI_VECTOR_REG, value);
            }
            break;

        /** PSR_EE_ING_FLUSH_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears channel flush indication for ingress channels0 through 31.
         *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_PSR_EE_ING_FLUSH_A_REGSET:
            {
                Iqn2Fl_PsrEeIngFlushA *eePsr = (Iqn2Fl_PsrEeIngFlushA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_RAW_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_RAW_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_EV0_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_EV0_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_EV1_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_A_EV1_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** PSR_EE_ING_FLUSH_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears channel flush indication for ingress channels0 through 31.
         *  Argument: a pointer to Iqn2Fl_PsrEeIngFlushA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_PSR_EE_ING_FLUSH_B_REGSET:
            {
                Iqn2Fl_PsrEeIngFlushA *eePsr = (Iqn2Fl_PsrEeIngFlushA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_RAW_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_RAW_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_EV0_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_EV0_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_EV1_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_ING_FLUSH_B_EV1_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.ING_FLUSH_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** PSR_EE_EGR_PROTOCOL_ERR_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears Protocol error indication of an unsupported data type or a missing PS_DATA transfer
         *  when one was expected for egress channels 0 through 31
         *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_A_REGSET:
            {
                Iqn2Fl_PsrEeEgressProtocolErrA *eePsr = (Iqn2Fl_PsrEeEgressProtocolErrA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_RAW_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_RAW_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_EV0_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_EV0_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_EV1_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_A_EV1_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** PSR_EE_EGR_PROTOCOL_ERR_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears Protocol error indication of an unsupported data type or a missing PS_DATA transfer
         *  when one was expected for egress channels 32 through 47
         *  Argument: a pointer to Iqn2Fl_PsrEeEgressProtocolErrB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_PSR_EE_EGR_PROTOCOL_ERR_B_REGSET:
            {
                Iqn2Fl_PsrEeEgressProtocolErrB *eePsr = (Iqn2Fl_PsrEeEgressProtocolErrB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_RAW_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_RAW_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_EV0_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_EV0_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_EV1_EN_SET_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN2_TOP_EGR_PROTOCOL_ERR_B_EV1_EN_CLR_ERR, eePsr->err);
                    hIqn2->regs->Top.PSR_EE.EGR_PROTOCOL_ERR_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** PKTDMA_EE_DESC_STARVE_REGSET
         *  Sets, clears, enabled sets, or enabled clears Descriptor starvation errors
         *  Argument: a pointer to Iqn2Fl_PktdmaEeDescStarve structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_PKTDMA_EE_DESC_STARVE_REGSET:
            {
                Iqn2Fl_PktdmaEeDescStarve *eePsr = (Iqn2Fl_PktdmaEeDescStarve*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_RAW_SET_SOP_ERR, eePsr->sop_err)|
                              CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_RAW_SET_MOP_ERR, eePsr->mop_err);
                    hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_RAW_CLR_SOP_ERR, eePsr->sop_err)|
                              CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_RAW_CLR_MOP_ERR, eePsr->mop_err);
                    hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_EV0_EN_SET_SOP_ERR, eePsr->sop_err)|
                              CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_EV0_EN_SET_MOP_ERR, eePsr->mop_err);
                    hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_EV0_EN_CLR_SOP_ERR, eePsr->sop_err)|
                              CSL_FMK(IQN2_TOP_PKTDMA_DESC_STARVE_EV0_EN_CLR_MOP_ERR, eePsr->mop_err);
                    hIqn2->regs->Top.PKTDMA_EE.PKTDMA_DESC_STARVE_EV0_EN_CLR = tempReg;
                }
            }
            break;

        /** AT2_AT_EE_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears RP1 Errors and Synchronization input Info
         *  Argument: a pointer to Iqn2Fl_At2EeInfoErr structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AT2_AT_EE_0_REGSET:
            {
                Iqn2Fl_At2EeInfoErr *eeAt2 = (Iqn2Fl_At2EeInfoErr*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_SET_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_RAW_CLR_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_SET_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV0_EN_CLR_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_SET_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_RP1_CRC_ERR, eeAt2->rp1_crc_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_RP1_BIT_ERR, eeAt2->rp1_bit_err)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_RP1_SYNC_INFO, eeAt2->rp1_sync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_RADSYNC_INFO, eeAt2->radsync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_PHYSYNC_INFO, eeAt2->physync_info)|
                              CSL_FMK(IQN_AT2_AT2_AT_EE_0_EV1_EN_CLR_PA_TSCOMP_INFO, eeAt2->pa_tscomp_info);
                    hIqn2->regs->At2.AT2_EE.AT2_AT_EE_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** IQS_EE_CHAN_ERR_REGSET
         *  Sets, clears, enabled sets, or enabled clears IQS2 Channel error register
         *  Argument: a pointer to Iqn2Fl_Iqs2EeChanErr structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_IQS_EE_CHAN_ERR_REGSET:
            {
                Iqn2Fl_Iqs2EeChanErr *eeIqs2 = (Iqn2Fl_Iqs2EeChanErr*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_SET_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_SET_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_SET_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_CLR_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_CLR_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_RAW_CLR_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_SET_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_SET_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_SET_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_CLR_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_CLR_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV0_EN_CLR_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_SET_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_SET_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_SET_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_CLR_ING_PKTDMA_ERR, eeIqs2->ing_pktdma_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_CLR_ING_DIO2_ERR, eeIqs2->ing_dio2_err)|
                              CSL_FMK(IQN_IQS2_IQS_EE_CHAN_ERR_EV1_EN_CLR_EGR_MUX_ERR, eeIqs2->egr_mux_err);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_CHAN_ERR_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** IQS_EE_ING_FLUSH_ERR_REGSET
         *  Sets, clears, enabled sets, or enabled clears IQS2 Flush error register
         *  Argument: a pointer to Iqn2Fl_Iqs2EeIngFlush structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_IQS_EE_ING_FLUSH_ERR_REGSET:
            {
                Iqn2Fl_Iqs2EeIngFlush *eeIqs2 = (Iqn2Fl_Iqs2EeIngFlush*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_RAW_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_RAW_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV0_EN_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV0_EN_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV1_EN_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_ING_FLUSH_ERR_EV1_EN_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_ING_FLUSH_ERR_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** IQS_EE_EGR_FLUSH_ERR_REGSET
         *  Sets, clears, enabled sets, or enabled clears IQS2 Flush error register
         *  Argument: a pointer to Iqn2Fl_Iqs2EeEgrFlush structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_IQS_EE_EGR_FLUSH_ERR_REGSET:
            {
                Iqn2Fl_Iqs2EeEgrFlush *eeIqs2 = (Iqn2Fl_Iqs2EeEgrFlush*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_RAW_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_RAW_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV0_EN_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV0_EN_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV1_EN_SET_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_IQS2_IQS_EE_EGR_FLUSH_ERR_EV1_EN_CLR_FLUSH, eeIqs2->flush);
                    hIqn2->regs->Iqs2.IQS_EE.IQS_EE_EGR_FLUSH_ERR_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ errors and info
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_A_REGSET:
            {
                Iqn2Fl_Aid2EeSiiA *eeAid2 = (Iqn2Fl_Aid2EeSiiA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_RAW_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAid2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAid2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAid2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAid2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAid2->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAid2->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_B_REGSET:
            {
                Iqn2Fl_Aid2EeSiiB *eeAid2 = (Iqn2Fl_Aid2EeSiiB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_SET_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_SET_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_SET_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_CLR_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_RAW_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_SET_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_SET_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_SET_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_SET_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_SET_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_SET_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_PKT_ERR, eeAid2->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAid2->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAid2->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAid2->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_C_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_C_REGSET:
            {
                Iqn2Fl_Aid2EeSiiC *eeAid2 = (Iqn2Fl_Aid2EeSiiC*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_RAW_SET_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_RAW_CLR_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_EV0_EN_SET_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_EV0_EN_CLR_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_EV1_EN_SET_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_C_EV1_EN_CLR_SI_ING_IQ_SOF_ERR, eeAid2->si_ing_iq_sof_err);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_C_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_D_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL per-channel SOP received from ICC info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_D_REGSET:
            {
                Iqn2Fl_Aid2EeSiiD *eeAid2 = (Iqn2Fl_Aid2EeSiiD*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_RAW_SET_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_RAW_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_EV0_EN_SET_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_EV0_EN_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_EV1_EN_SET_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_D_EV1_EN_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAid2->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SII_D_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_A_REGSET:
            {
                Iqn2Fl_Aid2EeSieA *eeAid2 = (Iqn2Fl_Aid2EeSieA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAid2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAid2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAid2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAid2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAid2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_B_REGSET:
            {
                Iqn2Fl_Aid2EeSieB *eeAid2 = (Iqn2Fl_Aid2EeSieB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_RAW_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_RAW_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_RAW_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_RAW_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV0_EN_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV0_EN_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV0_EN_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV0_EN_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV1_EN_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV1_EN_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV1_EN_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAid2->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_B_EV1_EN_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAid2->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_C_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL per-channel SOP transmitted to ICC.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_C_REGSET:
            {
                Iqn2Fl_Aid2EeSieC *eeAid2 = (Iqn2Fl_Aid2EeSieC*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_RAW_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_RAW_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_EV0_EN_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_EV0_EN_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_EV1_EN_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_C_EV1_EN_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAid2->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_SYSCLK_EE.AID2_EE_SIE_C_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_DFE_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 DFE interrupts.
         *  Argument: a pointer to Iqn2Fl_Aid2EeDfe structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_DFE_REGSET:
            {
                Iqn2Fl_Aid2EeDfe *eeAid2 = (Iqn2Fl_Aid2EeDfe*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_RAW_SET_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_RAW_CLR_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_EV0_EN_SET_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_EV0_EN_CLR_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_EV1_EN_SET_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_DFE_EE_A_EV1_EN_CLR_DFE_IQN_AID2_ERR, eeAid2->dfe_iqn_aid2_err);
                    hIqn2->regs->Aid2.AID2_EE_DFE.AID2_DFE_EE_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_E_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_E_REGSET:
            {
                Iqn2Fl_Aid2EeSiiE *eeAid2 = (Iqn2Fl_Aid2EeSiiE*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAid2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAid2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_E_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_F_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_F_REGSET:
            {
                Iqn2Fl_Aid2EeSiiF *eeAid2 = (Iqn2Fl_Aid2EeSiiF*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_RAW_SET_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_RAW_SET_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_RAW_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_RAW_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV0_EN_SET_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV0_EN_SET_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV0_EN_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV0_EN_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV1_EN_SET_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV1_EN_SET_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV1_EN_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAid2->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SII_F_EV1_EN_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAid2->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_F_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_G_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_G_REGSET:
            {
                Iqn2Fl_Aid2EeSiiG *eeAid2 = (Iqn2Fl_Aid2EeSiiG*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_RAW_SET_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_RAW_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_EV0_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_EV0_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_EV1_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_G_EV1_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAid2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_G_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SII_H_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_i CTL per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSiiH structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SII_H_REGSET:
            {
                Iqn2Fl_Aid2EeSiiH *eeAid2 = (Iqn2Fl_Aid2EeSiiH*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_RAW_SET_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_RAW_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_EV0_EN_SET_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_EV0_EN_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_EV1_EN_SET_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SII_H_EV1_EN_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAid2->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SII_H_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_D_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ errors and info..
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_D_REGSET:
            {
                Iqn2Fl_Aid2EeSieD *eeAid2 = (Iqn2Fl_Aid2EeSieD*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAid2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAid2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAid2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_D_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_E_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CTL errors and info..
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_E_REGSET:
            {
                Iqn2Fl_Aid2EeSieE *eeAid2 = (Iqn2Fl_Aid2EeSieE*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAid2->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAid2->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AID2_AID2_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAid2->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_E_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_F_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_F_REGSET:
            {
                Iqn2Fl_Aid2EeSieF *eeAid2 = (Iqn2Fl_Aid2EeSieF*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_RAW_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_RAW_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_EV0_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_EV0_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_EV1_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_F_EV1_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAid2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_F_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AID2_EE_SIE_G_REGSET
         *  Sets, clears, enabled sets, or enabled clears AID2 SI si_e CLT per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Aid2EeSieG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_AID2_EE_SIE_G_REGSET:
            {
                Iqn2Fl_Aid2EeSieG *eeAid2 = (Iqn2Fl_Aid2EeSieG*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_RAW_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_RAW_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_EV0_EN_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_EV0_EN_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_EV1_EN_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AID2_AID2_EE_SIE_G_EV1_EN_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAid2->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Aid2.AID2_IQN_AID2_EE_VBUSCLK_EE.AID2_EE_SIE_G_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ errors and info
         *  Argument: a pointer to Iqn2Fl_AilEeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_A_REGSET:
            {
                Iqn2Fl_AilEeSiiA *eeAil = (Iqn2Fl_AilEeSiiA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_RAW_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_SET_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeAil->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeAil->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeAil->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeAil->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeAil->si_ing_iq_ife_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_FIFO_OVFL_ERR, eeAil->si_ing_iq_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_B_REGSET:
            {
                Iqn2Fl_AilEeSiiB *eeAil = (Iqn2Fl_AilEeSiiB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_SET_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_SET_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_SET_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_CLR_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_RAW_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_SET_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_SET_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_SET_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV0_EN_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_SET_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_SET_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_SET_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_SET_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_PKT_ERR, eeAil->si_ing_ctl_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_ICC_EOP_INFO, eeAil->si_ing_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_ICC_DAT_INFO, eeAil->si_ing_ctl_icc_dat_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_B_EV1_EN_CLR_SI_ING_CTL_FIFO_OVFL_ERR, eeAil->si_ing_ctl_fifo_ovfl_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_C_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiC0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_C_0_REGSET:
            {
                Iqn2Fl_AilEeSiiC0 *eeAil = (Iqn2Fl_AilEeSiiC0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_RAW_SET_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_RAW_CLR_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_EV0_EN_SET_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_EV0_EN_CLR_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_EV1_EN_SET_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_0_EV1_EN_CLR_SI_ING_IQ_SOF_ERR, eeAil->si_ing_iq_sof_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_C_1_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiC1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_C_1_REGSET:
            {
                Iqn2Fl_AilEeSiiC1 *eeAil = (Iqn2Fl_AilEeSiiC1*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_RAW_SET_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_RAW_CLR_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_EV0_EN_SET_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_EV0_EN_CLR_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_EV1_EN_SET_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_C_1_EV1_EN_CLR_SI_ING_IQ_SOF_ERR_64_32, eeAil->si_ing_iq_sof_err_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_C_1_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_D_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL per-channel SOP received from ICC info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_D_REGSET:
            {
                Iqn2Fl_AilEeSiiD *eeAil = (Iqn2Fl_AilEeSiiD*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_RAW_SET_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_RAW_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_EV0_EN_SET_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_EV0_EN_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_EV1_EN_SET_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_D_EV1_EN_CLR_SI_ING_CTL_ICC_SOP_INFO, eeAil->si_ing_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SII_D_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_E_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_E_REGSET:
            {
                Iqn2Fl_AilEeSiiE *eeAil = (Iqn2Fl_AilEeSiiE*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeAil->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeAil->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_E_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_F_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_F_REGSET:
            {
                Iqn2Fl_AilEeSiiF *eeAil = (Iqn2Fl_AilEeSiiF*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_RAW_SET_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_RAW_SET_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_RAW_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_RAW_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV0_EN_SET_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV0_EN_SET_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV0_EN_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV0_EN_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV1_EN_SET_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV1_EN_SET_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV1_EN_CLR_SI_ING_CTL_PSI_EOP_INFO, eeAil->si_ing_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SII_F_EV1_EN_CLR_SI_ING_CTL_PSI_DAT_INFO, eeAil->si_ing_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_F_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_G_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiG0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_G_0_REGSET:
            {
                Iqn2Fl_AilEeSiiG0 *eeAil = (Iqn2Fl_AilEeSiiG0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_RAW_SET_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_RAW_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_EV0_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_EV0_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_EV1_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_0_EV1_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeAil->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_G_1_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiG1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_G_1_REGSET:
            {
                Iqn2Fl_AilEeSiiG1 *eeAil = (Iqn2Fl_AilEeSiiG1*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_RAW_SET_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_RAW_CLR_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_EV0_EN_SET_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_EV0_EN_CLR_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_EV1_EN_SET_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_G_1_EV1_EN_CLR_SI_ING_IQ_PSI_SOP_INFO_64_32, eeAil->si_ing_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_G_1_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SII_H_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_i CTL per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSiiH structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SII_H_REGSET:
            {
                Iqn2Fl_AilEeSiiH *eeAil = (Iqn2Fl_AilEeSiiH*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_RAW_SET_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_RAW_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_EV0_EN_SET_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_EV0_EN_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_EV1_EN_SET_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SII_H_EV1_EN_CLR_SI_ING_CTL_PSI_SOP_INFO, eeAil->si_ing_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SII_H_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_A_REGSET:
            {
                Iqn2Fl_AilEeSieA *eeAil = (Iqn2Fl_AilEeSieA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeAil->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeAil->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeAil->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeAil->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeAil->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_B_REGSET:
            {
                Iqn2Fl_AilEeSieB *eeAil = (Iqn2Fl_AilEeSieB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_RAW_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_RAW_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_RAW_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_RAW_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV0_EN_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV0_EN_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV0_EN_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV0_EN_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV1_EN_SET_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV1_EN_SET_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV1_EN_CLR_SI_EGR_CTL_ICC_EOP_INFO, eeAil->si_egr_ctl_icc_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_B_EV1_EN_CLR_SI_EGR_CTL_ICC_DAT_INFO, eeAil->si_egr_ctl_icc_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_C_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL per-channel SOP transmitted to ICC.
         *  Argument: a pointer to Iqn2Fl_AilEeSieC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_C_REGSET:
            {
                Iqn2Fl_AilEeSieC *eeAil = (Iqn2Fl_AilEeSieC*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_RAW_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_RAW_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_EV0_EN_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_EV0_EN_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_EV1_EN_SET_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_C_EV1_EN_CLR_SI_EGR_CTL_ICC_SOP_INFO, eeAil->si_egr_ctl_icc_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_EE_SIE_C_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_D_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ errors and info..
         *  Argument: a pointer to Iqn2Fl_AilEeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_D_REGSET:
            {
                Iqn2Fl_AilEeSieD *eeAil = (Iqn2Fl_AilEeSieD*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeAil->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeAil->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeAil->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_D_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_E_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CTL errors and info..
         *  Argument: a pointer to Iqn2Fl_AilEeSieE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_E_REGSET:
            {
                Iqn2Fl_AilEeSieE *eeAil = (Iqn2Fl_AilEeSieE*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_RAW_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV0_EN_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_SET_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_DATA_TYPE_ERR, eeAil->si_egr_ctl_psi_data_type_err)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_EOP_INFO, eeAil->si_egr_ctl_psi_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_EE_SIE_E_EV1_EN_CLR_SI_EGR_CTL_PSI_DAT_INFO, eeAil->si_egr_ctl_psi_dat_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_E_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_F_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieF0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_F_0_REGSET:
            {
                Iqn2Fl_AilEeSieF0 *eeAil = (Iqn2Fl_AilEeSieF0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_RAW_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_RAW_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_EV0_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_EV0_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_EV1_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_0_EV1_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeAil->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_F_1_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieF1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_F_1_REGSET:
            {
                Iqn2Fl_AilEeSieF1 *eeAil = (Iqn2Fl_AilEeSieF1*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_RAW_SET_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_RAW_CLR_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_EV0_EN_SET_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_EV0_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_EV1_EN_SET_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_F_1_EV1_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO_64_32, eeAil->si_egr_iq_psi_sop_info_64_32);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_F_1_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SIE_G_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI si_e CLT per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_AilEeSieG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SIE_G_REGSET:
            {
                Iqn2Fl_AilEeSieG *eeAil = (Iqn2Fl_AilEeSieG*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_RAW_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_RAW_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_EV0_EN_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_EV0_EN_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_EV1_EN_SET_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_EE_SIE_G_EV1_EN_CLR_SI_EGR_CTL_PSI_SOP_INFO, eeAil->si_egr_ctl_psi_sop_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_VBUSCLK_EE.AIL_EE_SIE_G_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_RM_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL RM error info.
         *  Argument: a pointer to Iqn2Fl_AilEeRm0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_RM_0_REGSET:
            {
                Iqn2Fl_AilEeRm0 *eeAil = (Iqn2Fl_AilEeRm0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_SET_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_RAW_CLR_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_SET_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV0_EN_CLR_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_SET_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_SYNC_STATUS_CHANGE, eeAil->sync_status_change)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE0, eeAil->rm_status_state0)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE1, eeAil->rm_status_state1)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE2, eeAil->rm_status_state2)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE3, eeAil->rm_status_state3)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE4, eeAil->rm_status_state4)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_STATUS_STATE5, eeAil->rm_status_state5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_NUM_LOS_DET, eeAil->num_los_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_LCV_DET, eeAil->lcv_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_FRAME_BNDRY_DET, eeAil->frame_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_BLOCK_BNDRY_DET, eeAil->block_bndry_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_MISSING_K28P5, eeAil->missing_k28p5)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_MISSING_K28P7, eeAil->missing_k28p7)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_K30P7_DET, eeAil->k30p7_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_LOC_DET, eeAil->loc_det)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RX_FIFO_OVF, eeAil->rx_fifo_ovf)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RCVD_LOS, eeAil->rcvd_los)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RCVD_LOF, eeAil->rcvd_lof)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RCVD_RAI, eeAil->rcvd_rai)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RCVD_SDI, eeAil->rcvd_sdi)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RCVD_RST, eeAil->rcvd_rst)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_LOS_ERR, eeAil->los_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_LOF_ERR, eeAil->lof_err)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_HFNSYNC_STATE, eeAil->hfnsync_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_LOF_STATE, eeAil->lof_state)|
                              CSL_FMK(IQN_AIL_AIL_RM_0_EV1_EN_CLR_RM_RST, eeAil->rm_rst);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RM_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_RT_TM_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL RT and TM error info.
         *  Argument: a pointer to Iqn2Fl_AilEeRtTm0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_RT_TM_0_REGSET:
            {
                Iqn2Fl_AilEeRtTm0 *eeAil = (Iqn2Fl_AilEeRtTm0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_SET_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_RAW_CLR_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_SET_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV0_EN_CLR_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_SET_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_HDR_ERROR, eeAil->rt_hdr_error)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_EM_INSERT, eeAil->rt_em_insert)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_UNFL, eeAil->rt_unfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_OVFL, eeAil->rt_ovfl)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_FRM_ERR, eeAil->rt_frm_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_UNALIGN_ERR, eeAil->rt_unalign_err)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_RT_AGGR_STATE_INFO, eeAil->rt_aggr_state_info)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_SYNC_STATUS_CHANGE, eeAil->tm_frame_sync_state)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_DELTA_INACTIVE, eeAil->delta_inactive)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_DELTA_MODIFIED, eeAil->delta_modified)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_FRAME_MISALIGN, eeAil->frame_misalign)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_FIFO_UNDEFLOW, eeAil->fifo_undeflow)|
                              CSL_FMK(IQN_AIL_AIL_RT_TM_0_EV1_EN_CLR_TM_FAIL, eeAil->tm_fail);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_RT_TM_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_CI_CO_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL CI and CO error info.
         *  Argument: a pointer to Iqn2Fl_AilEeRtCiCo0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_CI_CO_0_REGSET:
            {
                Iqn2Fl_AilEeCiCo0 *eeAil = (Iqn2Fl_AilEeCiCo0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_RAW_SET_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_RAW_SET_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_RAW_CLR_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_RAW_CLR_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV0_EN_SET_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV0_EN_SET_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV0_EN_CLR_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV0_EN_CLR_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV1_EN_SET_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV1_EN_SET_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV1_EN_CLR_CI_TBLTOOLONG, eeAil->ci_tbltoolong)|
                              CSL_FMK(IQN_AIL_AIL_CI_CO_0_EV1_EN_CLR_CO_TBLTOOLONG, eeAil->co_tbltoolong);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_PHY_EE.AIL_CI_CO_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_PD_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL PD 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePd0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_PD_0_REGSET:
            {
                Iqn2Fl_AilEePd0 *eeAil = (Iqn2Fl_AilEePd0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_SET_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_RAW_CLR_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_SET_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV0_EN_CLR_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_SET_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_FRM_WIN_ERR, eeAil->pd_ee_obsai_frm_win_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_SOP_ERR, eeAil->pd_ee_sop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_TS_MISS_ERR, eeAil->pd_ee_obsai_ts_miss_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_TS_WDOG_ERR, eeAil->pd_ee_obsai_ts_wdog_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_AXC_FAIL_ERR, eeAil->pd_ee_obsai_axc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_CRC_ERR, eeAil->pd_ee_obsai_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_RP3_01_SOC_RST_INFO, eeAil->pd_ee_rp3_01_soc_rst_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_ROUTE_FAIL_INFO, eeAil->pd_ee_obsai_route_fail_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_RP3_01_CAPTURE_INFO, eeAil->pd_ee_rp3_01_capture_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_RP3_01_CRC_FAIL_ERR, eeAil->pd_ee_rp3_01_crc_fail_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_GSM_OFF_STB_INFO, eeAil->pd_ee_obsai_gsm_off_stb_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_CPRI_CW_CRC_ERR, eeAil->pd_ee_cpri_cw_crc_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_CPRI_CW_OVFL_ERR, eeAil->pd_ee_cpri_cw_ovfl_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_CPRI_CW_4B5B_EOP_ERR, eeAil->pd_ee_cpri_cw_4b5b_eop_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_CPRI_CW_4B5B_CHAR_ERR, eeAil->pd_ee_cpri_cw_4b5b_char_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_SOP_INFO, eeAil->pd_ee_obsai_sop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_EOP_INFO, eeAil->pd_ee_obsai_eop_info)|
                              CSL_FMK(IQN_AIL_AIL_PD_0_EV1_EN_CLR_PD_EE_OBSAI_SOF_INFO, eeAil->pd_ee_obsai_sof_info);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_PD_1_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL PD 1 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePd1 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_PD_1_REGSET:
            {
                Iqn2Fl_AilEePd1 *eeAil = (Iqn2Fl_AilEePd1*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_RAW_SET_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_RAW_SET_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_RAW_SET_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_RAW_CLR_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_RAW_CLR_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_RAW_CLR_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_SET_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_SET_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_SET_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_CLR_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_CLR_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV0_EN_CLR_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_SET_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_SET_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_SET_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_CLR_PD_EE_CPRI_BUB_FSM_ERR, eeAil->pd_ee_cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_CLR_PD_EE_CPRI_TDM_FSM_ERR, eeAil->pd_ee_cpri_tdm_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_PD_1_EV1_EN_CLR_PD_EE_CPRI_RADSTD_ERR, eeAil->pd_ee_cpri_radstd_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PD_1_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_PE_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL PE 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEePe0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_PE_0_REGSET:
            {
                Iqn2Fl_AilEePe0 *eeAil = (Iqn2Fl_AilEePe0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_SET_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_RAW_CLR_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_SET_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV0_EN_CLR_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_SET_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_CPRI_CW_NULL_STARVE_ERR, eeAil->pe_ee_cpri_cw_null_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_CPRI_CW_4B5B_STARVE_ERR, eeAil->pe_ee_cpri_cw_4b5b_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_CPRI_CW_HYPFM_STARVE_ERR, eeAil->pe_ee_cpri_cw_hypfm_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_CPRI_CW_HDLC_STARVE_ERR, eeAil->pe_ee_cpri_cw_hdlc_starve_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_CPRI_CW_HYPFM_OFLOW_ERR, eeAil->pe_ee_cpri_cw_hypfm_oflow_err)|
                              CSL_FMK(IQN_AIL_AIL_PE_0_EV1_EN_CLR_PE_EE_OFIFO_OFLOW_ERR, eeAil->pe_ee_ofifo_oflow_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_PE_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** AIL_EE_SI_0_REGSET
         *  Sets, clears, enabled sets, or enabled clears AIL SI 0 error info.
         *  Argument: a pointer to Iqn2Fl_AilEeSi0 structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         *  use hIqn2->arg_ail to select the AIL instance
         */
        case IQN2FL_CMD_AIL_EE_SI_0_REGSET:
            {
                Iqn2Fl_AilEeSi0 *eeAil = (Iqn2Fl_AilEeSi0*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_SET_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_RAW_CLR_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_SET_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV0_EN_CLR_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_SET_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_UAT_PI_ERR, eeAil->uat_pi_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_CPRI_TDM_LUT_ERR, eeAil->cpri_tdm_lut_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_CPRI_BUB_FSM_ERR, eeAil->cpri_bub_fsm_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_OBSAI_PHY_SYNC_ERR, eeAil->obsai_phy_sync_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_OBSAI_MULTRULEFIRE_ERR, eeAil->obsai_multrulefire_err)|
                              CSL_FMK(IQN_AIL_AIL_AIL_SI_0_EV1_EN_CLR_OBSAI_DBM_WRAP_ERR, eeAil->obsai_dbm_wrap_err);
                    hIqn2->regs->Ail[hIqn2->arg_ail].AIL_IQN_AIL_EE_SYSCLK_EE.AIL_AIL_SI_0_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_DMA_ING_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO Ingress Interrupt info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_DMA_ING_A_REGSET:
            {
                Iqn2Fl_Dio2EeDmaIngA *eeDio2 = (Iqn2Fl_Dio2EeDmaIngA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_SET_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_SET_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_SET_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_CLR_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_CLR_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_RAW_CLR_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_SET_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_SET_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_SET_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_CLR_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_CLR_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV0_EN_CLR_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_SET_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_SET_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_SET_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_CLR_DMA_ING_PEND_QUE_OVF_ERR, eeDio2->dma_ing_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_CLR_DMA_ING_PROG_ERR, eeDio2->dma_ing_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_A_EV1_EN_CLR_DMA_ING_XFER_DONE_INFO, eeDio2->dma_ing_xfer_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_DMA_ING_B_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO Ingress Interrupt info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaIngB structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_DMA_ING_B_REGSET:
            {
                Iqn2Fl_Dio2EeDmaIngB *eeDio2 = (Iqn2Fl_Dio2EeDmaIngB*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_RAW_SET_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_RAW_CLR_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_EV0_EN_SET_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_EV0_EN_CLR_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_EV1_EN_SET_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_ING_B_EV1_EN_CLR_DMA_ING_BUF_OVF_ERR, eeDio2->dma_ing_buf_ovf_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_ING_B_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_DMA_EGR_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO Egress Interrupt info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeDmaEgrA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_DMA_EGR_A_REGSET:
            {
                Iqn2Fl_Dio2EeDmaEgrA *eeDio2 = (Iqn2Fl_Dio2EeDmaEgrA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dma_egr_pend_que_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_DMA_EGR_PROG_ERR, eeDio2->dma_egr_prog_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_DMA_EGR_XFER_DONE_INFO, eeDio2->dma_egr_xfer_done_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_SI_ING_IQ_FIFO_FULL_INFO, eeDio2->si_ing_iq_fifo_full_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DMA_EGR_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_DT_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO DataTrace Interrupt info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeDtA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_DT_A_REGSET:
            {
                Iqn2Fl_Dio2EeDtA *eeDio2 = (Iqn2Fl_Dio2EeDtA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_SET_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_RAW_CLR_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_SET_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV0_EN_CLR_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_SET_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_DMA_EGR_PEND_QUE_OVF_ERR, eeDio2->dt_buff_ovf_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_DMA_EGR_A_EV1_EN_CLR_DMA_EGR_PROG_ERR, eeDio2->dt_done_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_DT_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SII_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ errors and info
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SII_A_REGSET:
            {
                Iqn2Fl_Dio2EeSiiA *eeDio2 = (Iqn2Fl_Dio2EeSiiA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_SET_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_SET_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_CLR_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_RAW_CLR_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV0_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_SET_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_SOF_INFO, eeDio2->si_ing_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_ICC_DAT_INFO, eeDio2->si_ing_iq_icc_dat_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_SOP_INFO, eeDio2->si_ing_iq_ife_sop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_EOP_INFO, eeDio2->si_ing_iq_ife_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_A_EV1_EN_CLR_SI_ING_IQ_IFE_DAT_INFO, eeDio2->si_ing_iq_ife_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SII_C_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ per-channel start of frame errors.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiC structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SII_C_REGSET:
            {
                Iqn2Fl_Dio2EeSiiC *eeDio2 = (Iqn2Fl_Dio2EeSiiC*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_RAW_SET_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_RAW_CLR_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_EV0_EN_SET_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_EV0_EN_CLR_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_EV1_EN_SET_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_C_EV1_EN_CLR_SI_ING_IQ_SOF_ERR, eeDio2->si_ing_iq_sof_err);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_C_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SII_E_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiE structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SII_E_REGSET:
            {
                Iqn2Fl_Dio2EeSiiE *eeDio2 = (Iqn2Fl_Dio2EeSiiE*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_RAW_SET_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_RAW_CLR_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV0_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV0_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV1_EN_SET_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_EOP_INFO, eeDio2->si_ing_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SII_E_EV1_EN_CLR_SI_ING_IQ_PSI_DAT_INFO, eeDio2->si_ing_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_E_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SII_G_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_i IQ per-channel SOP transmitted to PSI info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSiiG structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SII_G_REGSET:
            {
                Iqn2Fl_Dio2EeSiiG *eeDio2 = (Iqn2Fl_Dio2EeSiiG*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_RAW_SET_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_RAW_CLR_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_EV0_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_EV0_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_EV1_EN_SET_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SII_G_EV1_EN_CLR_SI_ING_IQ_PSI_SOP_INFO, eeDio2->si_ing_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SII_G_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SIE_A_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ errors and info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieA structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SIE_A_REGSET:
            {
                Iqn2Fl_Dio2EeSieA *eeDio2 = (Iqn2Fl_Dio2EeSieA*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_SET_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_SET_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_RAW_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV0_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_SET_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_STARVE_ERR, eeDio2->si_egr_iq_efe_starve_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_PKT_ERR, eeDio2->si_egr_iq_efe_pkt_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_EFE_SYM_ERR, eeDio2->si_egr_iq_efe_sym_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_SOF_INFO, eeDio2->si_egr_iq_icc_sof_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_A_EV1_EN_CLR_SI_EGR_IQ_ICC_DAT_INFO, eeDio2->si_egr_iq_icc_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_A_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SIE_D_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ errors and info..
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieD structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SIE_D_REGSET:
            {
                Iqn2Fl_Dio2EeSieD *eeDio2 = (Iqn2Fl_Dio2EeSieD*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_SET_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_RAW_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_RAW_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV0_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_SET_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DATA_TYPE_ERR, eeDio2->si_egr_iq_psi_data_type_err)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_EOP_INFO, eeDio2->si_egr_iq_psi_eop_info)|
                              CSL_FMK(IQN_DIO2_DIO2_EE_SIE_D_EV1_EN_CLR_SI_EGR_IQ_PSI_DAT_INFO, eeDio2->si_egr_iq_psi_dat_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_D_EV1_EN_CLR = tempReg;
                }
            }
            break;

        /** DIO2_EE_SIE_F_REGSET
         *  Sets, clears, enabled sets, or enabled clears DIO2 SI si_e IQ per-channel SOP received from PSI info.
         *  Argument: a pointer to Iqn2Fl_Dio2EeSieF structure
         *  use hIqn2->arg_ee to select raw, ev0, or ev1 register sets
         */
        case IQN2FL_CMD_DIO2_EE_SIE_F_REGSET:
            {
                Iqn2Fl_Dio2EeSieF *eeDio2 = (Iqn2Fl_Dio2EeSieF*)arg;
                uint32_t tempReg;
                if(hIqn2->arg_ee == IQN2FL_EE_INT_SET){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_RAW_SET_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_CLR){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_RAW_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_RAW_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_EV0_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV0_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV0){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_EV0_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV0_EN_CLR = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_SET_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_EV1_EN_SET_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV1_EN_SET = tempReg;
                }
                else if(hIqn2->arg_ee == IQN2FL_EE_INT_EN_CLR_EV1){
                    tempReg = CSL_FMK(IQN_DIO2_DIO2_EE_SIE_F_EV1_EN_CLR_SI_EGR_IQ_PSI_SOP_INFO, eeDio2->si_egr_iq_psi_sop_info);
                    hIqn2->regs->Dio2.DIO2_EE.DIO2_EE_SIE_F_EV1_EN_CLR = tempReg;
                }
            }
            break;

        default:
            status = IQN2FL_INVCMD;
            break;
    }

    return status;
}
