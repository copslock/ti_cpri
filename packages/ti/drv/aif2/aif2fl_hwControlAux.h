 /*  ============================================================================
 *   Copyright (c) Texas Instruments Incorporated 2008, 2009
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
 *   @file  aif2fl_hwControlAux.h
 *
 *   @brief  API Auxilary header file for Antenna Interface 2 to set HW control 
 *
 */

/* =============================================================================
 * Revision History
 * ===============
 *  03-Jun-2009   Albert File Created.
 *  06-June-2015  Seb      File imported in the driver
 *  
 *
 * =============================================================================
 */

#ifndef _AIF2FLHWCONTROLAUX_H_
#define _AIF2FLHWCONTROLAUX_H_
 
#include <ti/drv/aif2/aif2fl.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 *  Hardware Control Functions of Antenna Interface 2
 */

/** ============================================================================
 *   @n@b Aif2Fl_enDisRxLink 
 *
 *   @b Description
 *   @n This function Starts or Stop SERDES Rx link 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance  (should use hAif2->arg_link to select link)
   
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIF2_SD_RX_EN_CFG_RXENABLE=1;
 *         AIF2_SD_RX_EN_CFG_RXENABLE=0
 *   @b Example
 *   @verbatim
        uint16_t arg = true;
        hAif2->arg_link =0; //enable link 0
        Aif2Fl_enDisRxLink (hAif2,  arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enDisRxLink (
    Aif2Fl_Handle    hAif2,
    uint16_t             arg
)
{   
    if (arg == true)
    {	
        CSL_FINST(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_EN_CFG, AIF2_SD_RX_EN_CFG_RXENABLE, ENABLE);
    }
    else 
    {
	 CSL_FINST(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_EN_CFG, AIF2_SD_RX_EN_CFG_RXENABLE, DISABLE);
    }    
}
        
/** ============================================================================
 *   @n@b Aif2Fl_enDisTxLink
 *
 *   @b Description
 *   @n This function  Starts or stops SERDES Tx link
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance  (should use hAif2->arg_link to select link)

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_TX_EN_CFG_TXENABLE=1;
 *        AIF2_SD_TX_EN_CFG_TXENABLE=0 
 *
 *   @b Example
 *   @verbatim
        uint16_t arg = true;
        hAif2->arg_link =0;
        Aif2Fl_enDisTxLink (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enDisTxLink (
    Aif2Fl_Handle    hAif2,
    uint16_t             arg
)
{    
    if (arg == true)
    {	
        CSL_FINST(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_EN_CFG, AIF2_SD_TX_EN_CFG_TXENABLE, ENABLE);
    }
    else 
    {
	 CSL_FINST(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_EN_CFG, AIF2_SD_TX_EN_CFG_TXENABLE, DISABLE);
    }
	
}

#ifndef K2
/** ============================================================================
 *   @n@b Aif2Fl_enDisLinkLoopback
 *
 *   @b Description
 *   @n Enable or Disable link internal loopback and this function controls Rx and Tx side together
 *
 *   @b Arguments
 *   @verbatim

            hAif2                Handle to the aif2 instance. should use arg_link to select link
            uint16_t                 arg:  true or false
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIF2_SD_RX_R2_CFG_RXLOOPBACK=3,AIF2_SD_TX_R1_CFG_TXLOOPBACK=2,AIF2_SD_RX_R1_CFG_RXLOSS_OF_SIGNAL=0;
 *    AIF2_SD_RX_R2_CFG_RXLOOPBACK=0,AIF2_SD_TX_R1_CFG_TXLOOPBACK=0,AIF2_SD_RX_R1_CFG_RXLOSS_OF_SIGNAL=4
 *
 *
 *   @b Example
 *   @verbatim
        hAif2->arg_link = 1;
        Aif2Fl_enDisLinkLoopback (hAif2,  true);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enDisLinkLoopback (
    Aif2Fl_Handle    hAif2,
    uint16_t                              arg
)
{
    if(arg == true){
       CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_R2_CFG, AIF2_SD_RX_R2_CFG_RXLOOPBACK, 0x3);
       CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_R1_CFG, AIF2_SD_TX_R1_CFG_TXLOOPBACK, 0x3);
	CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_R1_CFG, AIF2_SD_RX_R1_CFG_RXLOSS_OF_SIGNAL, 0x0);
    }
    else
    {
       CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_R2_CFG, AIF2_SD_RX_R2_CFG_RXLOOPBACK, 0);
       CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_R1_CFG, AIF2_SD_TX_R1_CFG_TXLOOPBACK, 0);
	CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_R1_CFG, AIF2_SD_RX_R1_CFG_RXLOSS_OF_SIGNAL, 0x4);
    }
}
#endif

/** ============================================================================
 *   @n@b Aif2Fl_enDisSdB8Pll
 *
 *   @b Description
 *   @n This function enables or disables SD B8  PLL  
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            arg      True or False                  
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL=1;AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL=0
 *      
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_enDisSdB8Pll (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enDisSdB8Pll (
    Aif2Fl_Handle    hAif2,
    uint16_t             arg
)
{
    if(arg == true)CSL_FINST(hAif2->regs->SD_PLL_B8_EN_CFG, AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL, ENABLE );
    
    else CSL_FINST(hAif2->regs->SD_PLL_B8_EN_CFG, AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL, DISABLE );
    
}


/** ============================================================================
 *   @n@b Aif2Fl_enDisSdB4Pll
 *
 *   @b Description
 *   @n This function enables or disables SD B4 PLL
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            arg      True or False
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL=1;AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL=0
 *      
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_enDisSdB4Pll (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enDisSdB4Pll (
    Aif2Fl_Handle    hAif2,
    uint16_t            arg
)
{
    if(arg == true)CSL_FINST(hAif2->regs->SD_PLL_B4_EN_CFG, AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL, ENABLE );
    
    else CSL_FINST(hAif2->regs->SD_PLL_B4_EN_CFG, AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL, DISABLE );
}

/** ============================================================================
 *   @n@b Aif2Fl_vcEmuControl
 *
 *   @b Description
 *   @n This function enables or disables EMU control fields
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_VcEmu     Free, Soft, RtSel
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AIF2_EMU_FREERUN,AIF2_AIF2_EMU_SOFT,AIF2_AIF2_EMU_RT_SEL
 *      
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_VcEmu    Emu; 
        Emu.Freerun = false;
        Emu.Soft = true;
        Emu.RtSel = true;
        Aif2Fl_vcEmuControl (hAif2, Emu);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_vcEmuControl (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_VcEmu       Emu
)
{
    uint32_t   tempReg;
    tempReg = CSL_FMK(AIF2_AIF2_EMU_FREERUN, Emu.Freerun)|
    CSL_FMK(AIF2_AIF2_EMU_SOFT, Emu.Soft)|
    CSL_FMK(AIF2_AIF2_EMU_RT_SEL, Emu.RtSel);
    hAif2->regs->AIF2_EMU = tempReg;
}

#ifndef K2
/** ============================================================================
 *   @n@b Aif2Fl_sdLinkTxTestPattern
 *
 *   @b Description
 *   @n This function selects or disables SD Tx test pattern
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_SdTestPattern     test patten
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_TX_R1_CFG_TXTEST_PATTERN
 *
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_SdTestPattern    test = CSL_AIF2_SD_ALTERNATING_0_1;
        hAif2->arg_link = 0;
        Aif2Fl_sdLinkTxTestPattern (hAif2, test);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_sdLinkTxTestPattern (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_SdTestPattern       test_pattern
)
{
    CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_TX_R1_CFG, AIF2_SD_TX_R1_CFG_TXTEST_PATTERN, test_pattern);
}


/** ============================================================================
 *   @n@b Aif2Fl_sdLinkRxTestPattern
 *
 *   @b Description
 *   @n This function selects or disables SD Rx test pattern
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_SdTestPattern     test patten
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_RX_R2_CFG_RXTEST_PATTERN
 *
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_SdTestPattern    test = CSL_AIF2_SD_ALTERNATING_0_1;
        hAif2->arg_link = 0;
        Aif2Fl_sdLinkRxTestPattern (hAif2, test);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_sdLinkRxTestPattern (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_SdTestPattern       test_pattern
)
{
    CSL_FINS(hAif2->regs->SD_LK[hAif2->arg_link].SD_RX_R2_CFG, AIF2_SD_RX_R2_CFG_RXTEST_PATTERN, test_pattern);
}

#endif

/** ============================================================================
 *   @n@b Aif2Fl_rmLinkForceRxState
 *
 *   @b Description
 *   @n This function force set RM sync status
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            Aif2Fl_RmForceSyncState     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_RM_LK_CFG0_FORCE_RX_STATE
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t    arg = AIF2FL_RM_FORCE_ST0;
        hAif2->arg_link = LINK0;
        Aif2Fl_rmLinkForceRxState (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_rmLinkForceRxState (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_RmForceSyncState          arg
)
{
    CSL_FINS(hAif2->regs->G_RM_LKS[hAif2->arg_link].RM_LK_CFG0, AIF2_RM_LK_CFG0_FORCE_RX_STATE, arg);
}


/** ============================================================================
 *   @n@b Aif2Fl_tmLinkL1InbandSet
 *
 *   @b Description
 *   @n TM L1 Inband Control Signal Set for app testing
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            uint8_t    5 bitmap for control signal 
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_L1_CFG_L1_INBAND_CFG
 *
 *   @b Example
 *   @verbatim
        uint8_t    bitmask = 0x1; //set bit 0 RESET
        hAif2->arg_link = LINK0;
        Aif2Fl_tmLinkL1InbandSet (hAif2, bitmask);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_tmLinkL1InbandSet (
    Aif2Fl_Handle    hAif2,
    uint8_t                   bitmask
)
{
    CSL_FINS(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_L1_CFG, AIF2_TM_LK_L1_CFG_L1_INBAND_CFG, bitmask);
}


/** ============================================================================
 *   @n@b Aif2Fl_forceTmFlush
 *
 *   @b Description
 *   @n This function force flush TM FIFO
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            uint16_t     true
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_CTRL_TM_FLUSH
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t    arg = true; 
        hAif2->arg_link = LINK0;
        Aif2Fl_forceTmFlush (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_forceTmFlush (
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
    CSL_FINS(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_CTRL, AIF2_TM_LK_CTRL_TM_FLUSH, arg);
}


/** ============================================================================
 *   @n@b Aif2Fl_forceTmIdle
 *
 *   @b Description
 *   @n This function force set TM IDLE state
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            uint16_t     true
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_CTRL_TM_IDLE
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t    arg = true; 
        hAif2->arg_link = LINK0;
        Aif2Fl_forceTmIdle (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_forceTmIdle (
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
    CSL_FINS(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_CTRL, AIF2_TM_LK_CTRL_TM_IDLE, arg);
}


/** ============================================================================
 *   @n@b Aif2Fl_forceTmReSync
 *
 *   @b Description
 *   @n This function force set TM RESYNC state
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            uint16_t     true
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_CTRL_TM_RESYNC
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t    arg = true; 
        hAif2->arg_link = LINK0;
        Aif2Fl_forceTmReSync (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_forceTmReSync (
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
    CSL_FINS(hAif2->regs->G_TM_LKS[hAif2->arg_link].TM_LK_CTRL, AIF2_TM_LK_CTRL_TM_RESYNC, arg);
}


/** ============================================================================
 *   @n@b Aif2Fl_pdCpriIdLutDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PD CPRI id lut register.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            Aif2Fl_PdCpriIdLut     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PD_CPRI_ID_LUT_CPRI_DMACHAN,AIF2_PD_CPRI_ID_LUT_CPRI_X_EN,
 *      AIF2_PD_CPRI_ID_LUT_CPRI_PKT_EN,AIF2_PD_CPRI_ID_LUT_CPRI_8WD_OFSET
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_PdCpriIdLut    arg; 
        arg.ChannelNum = 8; //add channel 8 
        arg.CpriDmaCh = 3;
        ..........
        hAif2->arg_link = LINK0;
        Aif2Fl_pdCpriIdLutDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_pdCpriIdLutDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PdCpriIdLut         arg
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_DMACHAN, arg.CpriDmaCh)    |
                  CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_X_EN, arg.bEnableCpriX)   |
                  CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_PKT_EN,   arg.bEnableCpriPkt) |
	           CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_8WD_OFSET, arg.Cpri8WordOffset);
    hAif2->regs->G_PD_LKS[hAif2->arg_link].PD_CPRI_ID_LUT[arg.ChannelNum] = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_pdCpriCwLutDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PD CPRI Control word lut register.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            Aif2Fl_CpriCwLut     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PD_CW_LUT_CW_CHAN, AIF2_PD_CW_LUT_CW_EN
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_CpriCwLut    arg; 
        arg.ChannelNum = 2;   //CW channel 0 ~ 255
        arg.CpriCwChannel = 1;
        arg.bEnableCpriCw = true;
        hAif2->arg_link = LINK0;
        Aif2Fl_pdCpriCwLutDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_pdCpriCwLutDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_CpriCwLut         arg
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_PD_CW_LUT_CW_CHAN, arg.CpriCwChannel) |
                  CSL_FMK(AIF2_PD_CW_LUT_CW_EN, arg.bEnableCpriCw);
    hAif2->regs->G_PD_LKS[hAif2->arg_link].PD_CW_LUT[arg.ChannelNum] = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_pdLinkDbmDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PD Link DBMR
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            Aif2Fl_DualBitMap     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PD_DBM_DBM_X,AIF2_PD_DBM_DBM_XBUBBLE,AIF2_PD_DBM_DBM_1MULT,
 *      AIF2_PD_DBM_DBM_1SIZE,AIF2_PD_DBM_DBM_2SIZE,AIF2_PD_DBM_1MAP_DBM_1MAP,
 *      AIF2_PD_DBM_2MAP_DBM_2MAP
 *   @b Example
 *   @verbatim
        Aif2Fl_DualBitMap    dbm; 
        dbm. DbmX= 3;  
        ..........
        hAif2->arg_link = LINK0;
        Aif2Fl_pdLinkDbmDynamicSetup (hAif2, dbm);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_pdLinkDbmDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DualBitMap         dbm
)
{
    uint32_t tempReg, i;
    
    /* PD Link Dual bit map FSM register setup*/
    tempReg = CSL_FMK(AIF2_PD_DBM_DBM_X, dbm.DbmX)    |
                  CSL_FMK(AIF2_PD_DBM_DBM_XBUBBLE, dbm.DbmXBubble)   |
                  CSL_FMK(AIF2_PD_DBM_DBM_1MULT,   dbm.Dbm1Mult) |
	           CSL_FMK(AIF2_PD_DBM_DBM_1SIZE, dbm.Dbm1Size)   |
                  CSL_FMK(AIF2_PD_DBM_DBM_2SIZE,   dbm.Dbm2Size);
    hAif2->regs->G_PD_LKS[hAif2->arg_link].PD_DBM = tempReg;
	
    /* PD DBM 1 map setup*/
    for(i = 0; i < 4; i++)
    CSL_FINS(hAif2->regs->G_PD_LKS[hAif2->arg_link].PD_DBM_1MAP[i], AIF2_PD_DBM_1MAP_DBM_1MAP, dbm.Dbm1Map[i]); 

    /* PD DBM 2 map setup*/
    for(i = 0; i < 3; i++)
    CSL_FINS(hAif2->regs->G_PD_LKS[hAif2->arg_link].PD_DBM_2MAP[i], AIF2_PD_DBM_2MAP_DBM_2MAP, dbm.Dbm2Map[i]); 

}


/** ============================================================================
 *   @n@b Aif2Fl_pdChConfigDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PD channel config registers
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PdChannelConfig     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PD_ROUTE_ROUTE_TS,AIF2_PD_ROUTE_ROUTE_TYPE,AIF2_PD_ROUTE_ROUTE_ADR,
 *      AIF2_PD_ROUTE_ROUTE_LK,AIF2_PD_ROUTE_ROUTE_MASK,AIF2_PD_DMACHAN_CHAN_EN,
 *      AIF2_PD_DMACHAN_DATA_FORMAT,AIF2_PD_DMACHAN_A_AXC_OFFSET,AIF2_PD_DMACHAN_B_TS_WDOG_EN,
 *      AIF2_PD_DMACHAN_B_GSM_UL,AIF2_PD_DMACHAN_B_FRM_GRP,AIF2_PD_DMACHAN_B_DIO_OFFSET,
 *      AIF2_PD_DMACHAN_B_TDD_EN,AIF2_PD_DMACHAN_C_TDD_EN,AIF2_PD_DMACHAN_D_TDD_EN,
 *      AIF2_PD_DMACHAN_E_TDD_EN,AIF2_PD_DMACHAN_F_TDD_EN
 *   @b Example
 *   @verbatim
        Aif2Fl_PdChannelConfig    arg; 
        arg. DbmX= 3;  
        ..........
   
        Aif2Fl_pdChConfigDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_pdChConfigDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PdChannelConfig         arg
)
{
    uint32_t tempReg;
    
    /* PD route register setup */
    tempReg = CSL_FMK(AIF2_PD_ROUTE_ROUTE_TS, arg.PdRoute.RouteTs)|
  	              CSL_FMK(AIF2_PD_ROUTE_ROUTE_TYPE, arg.PdRoute.RouteType)|
                     CSL_FMK(AIF2_PD_ROUTE_ROUTE_ADR, arg.PdRoute.RouteAddr)|
	              CSL_FMK(AIF2_PD_ROUTE_ROUTE_LK, arg.PdRoute.RouteLink)|
                     CSL_FMK(AIF2_PD_ROUTE_ROUTE_MASK, arg.PdRoute.RouteMask);
    hAif2->regs->PD_ROUTE[arg.ChannelNum] = tempReg;
    
    /* PD DMA channel config 0 (AxC offset) register setup */
    CSL_FINS(hAif2->regs->PD_DMACHAN_A[arg.ChannelNum], AIF2_PD_DMACHAN_A_AXC_OFFSET, arg.AxCOffset);

    /* PD DMA channel config 1 register setup */
    tempReg = CSL_FMK(AIF2_PD_DMACHAN_B_TS_WDOG_EN, arg.PdChConfig1.bTsWatchDogEn)|
  	              CSL_FMK(AIF2_PD_DMACHAN_B_GSM_UL, arg.PdChConfig1.DataFormat)|
                     CSL_FMK(AIF2_PD_DMACHAN_B_FRM_GRP, arg.PdChConfig1.FrameCounter)|
	              CSL_FMK(AIF2_PD_DMACHAN_B_DIO_OFFSET, arg.PdChConfig1.DioOffset)|
                     CSL_FMK(AIF2_PD_DMACHAN_B_TDD_EN, arg.PdChConfig1.TddEnable);
    hAif2->regs->PD_DMACHAN_B[arg.ChannelNum] = tempReg;

    /* PD DMA channel config 2 ~ 5 (tdd enable) register setup */
    CSL_FINS(hAif2->regs->PD_DMACHAN_C[arg.ChannelNum], AIF2_PD_DMACHAN_C_TDD_EN, arg.TddEnable1);
    CSL_FINS(hAif2->regs->PD_DMACHAN_D[arg.ChannelNum], AIF2_PD_DMACHAN_D_TDD_EN, arg.TddEnable2);
    CSL_FINS(hAif2->regs->PD_DMACHAN_E[arg.ChannelNum], AIF2_PD_DMACHAN_E_TDD_EN, arg.TddEnable3);
    CSL_FINS(hAif2->regs->PD_DMACHAN_F[arg.ChannelNum], AIF2_PD_DMACHAN_F_TDD_EN, arg.TddEnable4);

    /* PD DMA channel config register setup */
    tempReg = CSL_FMK(AIF2_PD_DMACHAN_CHAN_EN, arg.PdChConfig.bChannelEn)|
                     CSL_FMK(AIF2_PD_DMACHAN_DATA_FORMAT, arg.PdChConfig.DataFormat);
    hAif2->regs->PD_DMACHAN[arg.ChannelNum] = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_peCpriCwLutDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE CPRI Control word lut register.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance (should use arg_link for link selection)
            Aif2Fl_CpriCwLut     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_CPRI_CW_LUT_CW_CHAN, AIF2_PE_CPRI_CW_LUT_CW_EN
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_CpriCwLut    arg; 
        arg.ChannelNum = 2;   //CW channel 0 ~ 255
        arg.CpriCwChannel = 1;
        arg.bEnableCpriCw = true;
        hAif2->arg_link = LINK0;
        Aif2Fl_peCpriCwLutDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peCpriCwLutDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_CpriCwLut         arg
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_PE_CPRI_CW_LUT_CW_CHAN, arg.CpriCwChannel) |
                  CSL_FMK(AIF2_PE_CPRI_CW_LUT_CW_EN, arg.bEnableCpriCw);
    hAif2->regs->G_PE_LKS[hAif2->arg_link].PE_CPRI_CW_LUT[arg.ChannelNum] = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_peObsaiHeaderSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE OBSAI header register.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PeObsaiHeader     
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_OBSAI_HDR_OBSAI_TS_ADR,AIF2_PE_OBSAI_HDR_OBSAI_TYPE,AIF2_PE_OBSAI_HDR_OBSAI_ADR,
 *      AIF2_PE_OBSAI_HDR_OBSAI_TS_MASK,AIF2_PE_OBSAI_HDR_OBSAI_TS_FRMT
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_PeObsaiHeader    arg; 
        arg.ChannelNum = 8; //add channel 8 
        ........
        Aif2Fl_peObsaiHeaderSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peObsaiHeaderSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PeObsaiHeader     arg
)
{
    uint32_t tempReg;
    
    /* PD route register setup */
    tempReg = CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_ADR, arg.PeChObsaiTs)|
  	              CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TYPE, arg.PeChObsaiType)|
                     CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_ADR, arg.PeChObsaiAddr)|
	              CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_MASK, arg.PeChObsaiTsMask)|
                     CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_FRMT, arg.PeChObsaiTsfomat);
    hAif2->regs->PE_OBSAI_HDR[arg.ChannelNum] = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_peDbmrDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE Link DBMR for CPRI and 64 DBMR for OBSAI
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PeDbmr      
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_CPRI_DBM_CPRI_DBM_X,AIF2_PE_CPRI_DBM_CPRI_DBM_XBUBBLE,AIF2_PE_CPRI_DBM_CPRI_DBM_1MULT,
 *      AIF2_PE_CPRI_DBM_CPRI_DBM_1SIZE,AIF2_PE_CPRI_DBM_CPRI_DBM_2SIZE,AIF2_PE_CPRIDBM_1MAP_CPRI_DBM_1MAP,
 *      AIF2_PE_CPRIDBM_2MAP_CPRI_DBM_2MAP;
 *    
 *      AIF2_PE_OBSAI_DBM_DBM_X,AIF2_PE_OBSAI_DBM_DBM_1MULT,AIF2_PE_OBSAI_DBM_DBM_1SIZE,
 *      AIF2_PE_OBSAI_DBM_DBM_2SIZE,AIF2_PE_DBM_MAP_DBM_BIT_MAP
 *      
 *   @b Example
 *   @verbatim
        Aif2Fl_PeDbmr     dbm; 
        
        dbm.isCpri = true or false; // true for CPRI and false for OBSAI  
        dbm.CpriLinkNum = LINK0; // CPRI only
        dbm.RuleNum = 0; //OBSAI only 
        ........................
        Aif2Fl_peDbmrDynamicSetup (hAif2, dbm);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peDbmrDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PeDbmr          dbm
)
{
    uint32_t tempReg, i;

    if(dbm.isCpri == true)
    {
    /* PE Link CPRI Dual bit map FSM register setup*/
    tempReg = CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_X, dbm.DualBitMap.DbmX)    |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_XBUBBLE, dbm.DualBitMap.DbmXBubble)   |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_1MULT,  dbm.DualBitMap.Dbm1Mult) |
	           CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_1SIZE, dbm.DualBitMap.Dbm1Size)   |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_2SIZE,  dbm.DualBitMap.Dbm2Size);
    hAif2->regs->G_PE_LKS[dbm.CpriLinkNum].PE_CPRI_DBM = tempReg;
	
    /* PE CPRI DBM 1 map setup*/
    for(i = 0; i < 4; i++)
    CSL_FINS(hAif2->regs->G_PE_LKS[dbm.CpriLinkNum].PE_CPRIDBM_1MAP[i], AIF2_PE_CPRIDBM_1MAP_CPRI_DBM_1MAP, dbm.DualBitMap.Dbm1Map[i]); 
   
    /* PE CPRI DBM 2 map setup*/
    for(i = 0; i < 3; i++)
    CSL_FINS(hAif2->regs->G_PE_LKS[dbm.CpriLinkNum].PE_CPRIDBM_2MAP[i], AIF2_PE_CPRIDBM_2MAP_CPRI_DBM_2MAP, dbm.DualBitMap.Dbm2Map[i]); 
   
    }
    else {

     /* PE OBSAI Dual bit map FSM register setup*/
    tempReg = CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_X, dbm.DualBitMap.DbmX)    |
                  CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_1MULT,   dbm.DualBitMap.Dbm1Mult) |
	           CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_1SIZE, dbm.DualBitMap.Dbm1Size)   |
                  CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_2SIZE,   dbm.DualBitMap.Dbm2Size);
    hAif2->regs->PE_OBSAI_DBM[dbm.RuleNum] = tempReg;
    
    /* PE CPRI DBM 1 map setup*/
    CSL_FINS(hAif2->regs->PE_DBM_MAP[0+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm1Map[0]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[1+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm1Map[1]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[2+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm1Map[2]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[3+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm1Map[3]); 

    /* PE CPRI DBM 2 map setup*/
    CSL_FINS(hAif2->regs->PE_DBM_MAP[4+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm2Map[0]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[5+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm2Map[1]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[6+(8*dbm.RuleNum)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, dbm.DualBitMap.Dbm2Map[2]); 
    }
	
}


/** ============================================================================
 *   @n@b Aif2Fl_peModuloTxRuleSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE Modulo rule
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PeModuloRule      
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_MODTXRULE_RULE_MOD,AIF2_PE_MODTXRULE_RULE_INDEX,
 *      AIF2_PE_MODTXRULE_RULE_LK,AIF2_PE_MODTXRULE_RULE_CTL_MSG,AIF2_PE_MODTXRULE_RULE_EN,
 *      
 *   @b Example
 *   @verbatim
        Aif2Fl_PeModuloRule     mrule; 
        mrule.RuleNum = 1; 
        ........................
        Aif2Fl_peModuloTxRuleSetup (hAif2, mrule);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peModuloTxRuleSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PeModuloRule          mrule
)
{
    uint32_t tempReg;

    tempReg = CSL_FMK(AIF2_PE_MODTXRULE_RULE_MOD, mrule.PeModuloTc.RuleModulo)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_INDEX, mrule.PeModuloTc.RuleIndex)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_LK, mrule.PeModuloTc.RuleLink)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_CTL_MSG, mrule.PeModuloTc.bRuleObsaiCtlMsg)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_EN, mrule.PeModuloTc.bEnableRule);
    hAif2->regs->PE_MODTXRULE[mrule.RuleNum] = tempReg;
	
}


/** ============================================================================
 *   @n@b Aif2Fl_peChConfigDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE channel config registers
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PeChannelConfig      
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_DMACHAN_EN_CH_EN,AIF2_PE_DMA0CHAN_CRC_EN,AIF2_PE_DMA0CHAN_FRM_TC,
 *      AIF2_PE_DMA0CHAN_RT_CTL,AIF2_PE_DMA0CHAN_CRC_TYPE,AIF2_PE_DMA0CHAN_ETHERNET,AIF2_PE_DMA0CHAN_CRC_HDR,
 *      AIF2_PE_IN_FIFO_MF_WMARK,AIF2_PE_IN_FIFO_MF_FULL_LEV,
 *      AIF2_PE_IN_FIFO_SYNC_SYM,AIF2_PE_AXC_OFFSET_AXC_OFFSET
 *   @b Example
 *   @verbatim
        Aif2Fl_PeChannelConfig     arg; 
        arg.ChannelNum = 1; 
        ........................
        Aif2Fl_peChConfigDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peChConfigDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PeChannelConfig          arg
)
{
    uint32_t tempReg;

    /* PE 128 DMA channel 0 register setup */
    tempReg = CSL_FMK(AIF2_PE_DMA0CHAN_CRC_EN, arg.PeDmaCh0.bCrcEn)|
  	              CSL_FMK(AIF2_PE_DMA0CHAN_FRM_TC, arg.PeDmaCh0.FrameTC)|
                     CSL_FMK(AIF2_PE_DMA0CHAN_RT_CTL, arg.PeDmaCh0.RtControl)|
	              CSL_FMK(AIF2_PE_DMA0CHAN_CRC_TYPE, arg.PeDmaCh0.CrcType)|
                     CSL_FMK(AIF2_PE_DMA0CHAN_ETHERNET, arg.PeDmaCh0.isEthernet)|
	              CSL_FMK(AIF2_PE_DMA0CHAN_CRC_HDR, arg.PeDmaCh0.CrcObsaiHeader);
    hAif2->regs->PE_DMA0CHAN[arg.ChannelNum] = tempReg;
   
    /* PE 128 in fifo register setup */
    tempReg = CSL_FMK(AIF2_PE_IN_FIFO_MF_WMARK, arg.PeInFifo.MFifoWmark)|
                     CSL_FMK(AIF2_PE_IN_FIFO_MF_FULL_LEV, arg.PeInFifo.MFifoFullLevel)|
	              CSL_FMK(AIF2_PE_IN_FIFO_SYNC_SYM, arg.PeInFifo.SyncSymbol);
    hAif2->regs->PE_IN_FIFO[arg.ChannelNum] = tempReg;

    /* set PE AxC offset */ 
    CSL_FINS(hAif2->regs->PE_AXC_OFFSET[arg.ChannelNum], AIF2_PE_AXC_OFFSET_AXC_OFFSET, arg.AxCOffset);

    /* enable PE dma channel  */ 
    CSL_FINS(hAif2->regs->PE_DMACHAN_EN[arg.ChannelNum], AIF2_PE_DMACHAN_EN_CH_EN, arg.bEnableAxC);
}


/** ============================================================================
 *   @n@b Aif2Fl_peChRuleLutDynamicSetup
 *
 *   @b Description
 *   @n Dynamic configuration of PE channel rule LUT config registers
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            Aif2Fl_PeChRuleLut      
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_RULE_CHANLUT0_CHANINDEX,AIF2_PE_RULE_CHANLUT0_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT0_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT1_CHANINDEX,AIF2_PE_RULE_CHANLUT1_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT1_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT2_CHANINDEX,AIF2_PE_RULE_CHANLUT2_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT2_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT3_CHANINDEX,AIF2_PE_RULE_CHANLUT3_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT3_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT4_CHANINDEX,AIF2_PE_RULE_CHANLUT4_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT4_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT5_CHANINDEX,AIF2_PE_RULE_CHANLUT5_CHANINDEX_EN,
 *      AIF2_PE_RULE_CHANLUT5_CPRI_PKT_EN;
 *      AIF2_PE_RULE_CHANLUT6_CHANINDEX,AIF2_PE_RULE_CHANLUT6_CHANINDEX_EN;
 *      AIF2_PE_RULE_CHANLUT7_CHANINDEX,AIF2_PE_RULE_CHANLUT7_CHANINDEX_EN
 *      
 *   @b Example
 *   @verbatim
        Aif2Fl_PeChRuleLut     arg; 
        arg.RuleNum = 64; //0 ~ 511
        arg.LutNum = 3: //0 ~ 7
        ........................
        Aif2Fl_peChRuleLutDynamicSetup (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_peChRuleLutDynamicSetup (
    Aif2Fl_Handle    hAif2,
    Aif2Fl_PeChRuleLut          arg
)
{
    uint32_t tempReg;
	
    if(arg.LutNum == 0) {//LUT index 0 
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT0_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT0_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT0_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT0[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 1){
     tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT1_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT1_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT1_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT1[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 2){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT2_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT2_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT2_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT2[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 3){
     tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT3_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT3_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT3_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT3[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 4){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT4_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT4_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT4_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT4[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 5){
     tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT5_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT5_CHANINDEX_EN, arg.bEnableChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT5_CPRI_PKT_EN, arg.bCpriPktEnable);
    hAif2->regs->PE_RULE_CHANLUT5[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 6){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT6_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT6_CHANINDEX_EN, arg.bEnableChIndex);
    hAif2->regs->PE_RULE_CHANLUT6[arg.RuleNum] = tempReg;
    }
    else if(arg.LutNum == 7){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT7_CHANINDEX, arg.ChIndex)|
    CSL_FMK(AIF2_PE_RULE_CHANLUT7_CHANINDEX_EN, arg.bEnableChIndex);
    hAif2->regs->PE_RULE_CHANLUT7[arg.RuleNum] = tempReg;
    }
		
}


/** ============================================================================
 *   @n@b Aif2Fl_enLinkDataCapture
 *
 *   @b Description
 *   @n Enables Trace data and framing data capture for specific link
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance and link argument (arg_link) should be used

            uint16_t       true : enable    false : disable

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_RM_CFG_RAW_DATA_SEL,AIF2_DB_IDB_CFG_DTB_EN,AIF2_DB_IDB_CFG_DTF_EN,
 *        AIF2_DB_IDB_CFG_DT_EN
 *
 *   @b Example
 *   @verbatim
        uint16_t arg = true;
        Aif2Fl_enLinkDataCapture (hAif2, (void *)&arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enRxLinkDataCapture(
    Aif2Fl_Handle    hAif2,
    uint16_t             arg
)
{
    uint32_t    tempReg;
    CSL_FINS(hAif2->regs->RM_CFG, AIF2_RM_CFG_RAW_DATA_SEL, hAif2->arg_link);

    tempReg = CSL_FMK(AIF2_DB_IDB_CFG_DTB_EN, arg)|
    CSL_FMK(AIF2_DB_IDB_CFG_DTF_EN, arg)|
    CSL_FMK(AIF2_DB_IDB_CFG_DT_EN, arg); //Enable Disable Data trace for DTB and DTF
    hAif2->regs->DB_IDB_CFG |= tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_enRxTraceDataSync
 *
 *   @b Description
 *   @n Flushes all outbound pktsw fifo's if a memory leak is detected
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            uint16_t     true : Enable data sync  false : Disable data sync

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_CFG_DT_SYNC
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t arg = true;
        Aif2Fl_enRxTraceDataSync (hAif2, (void *)&arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_enRxTraceDataSync(
    Aif2Fl_Handle    hAif2,
    uint16_t             arg
)
{
    CSL_FINS(hAif2->regs->DB_IDB_CFG, AIF2_DB_IDB_CFG_DT_SYNC, arg );
}


/** ============================================================================
 *   @n@b Aif2Fl_inDbEnDisDebug
 *
 *   @b Description
 *   @n Enables Ingress DB Debug mode
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            uint16_t     true : Debug on    false : Debug off

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_CFG_IDB_DEBUG_EN
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t  arg = true;
        Aif2Fl_inDbEnDisDebug (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbEnDisDebug(
    Aif2Fl_Handle    hAif2,
    uint16_t            arg
)
{
     CSL_FINS(hAif2->regs->DB_IDB_CFG, AIF2_DB_IDB_CFG_IDB_DEBUG_EN, arg);  
}
    
/** ============================================================================
 *   @n@b Aif2Fl_inDbDebugDataSetup
 *
 *   @b Description
 *   @n Debug data written to bits 128:0 of Ingress DB RAM
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            *uint32_t   128 bit Debug data 

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_DEBUG_D0_DATA,AIF2_DB_IDB_DEBUG_D1_DATA,
 *      AIF2_DB_IDB_DEBUG_D2_DATA,AIF2_DB_IDB_DEBUG_D3_DATA
 *
 *   @b Example
 *   @verbatim
        uint32_t    DebugData[4];
        DebugData[0] = 0x.....;
        DebugData[1] = 0x.....;
        DebugData[2] = 0x.....;
        DebugData[3] = 0x.....;
        Aif2Fl_inDbDebugDataSetup(hAif2, &DebugData[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbDebugDataSetup(
    Aif2Fl_Handle    hAif2,
    uint32_t            *debug_data
)
{
    CSL_FINS(hAif2->regs->DB_IDB_DEBUG_D0, AIF2_DB_IDB_DEBUG_D0_DATA, debug_data[0]);  
    CSL_FINS(hAif2->regs->DB_IDB_DEBUG_D1, AIF2_DB_IDB_DEBUG_D1_DATA, debug_data[1]);  
    CSL_FINS(hAif2->regs->DB_IDB_DEBUG_D2, AIF2_DB_IDB_DEBUG_D2_DATA, debug_data[2]);  
    CSL_FINS(hAif2->regs->DB_IDB_DEBUG_D3, AIF2_DB_IDB_DEBUG_D3_DATA, debug_data[3]);  
}
 


/** ============================================================================
 *   @n@b Aif2Fl_inDbDebugSideDataSetup
 *
 *   @b Description
 *   @n Ingress DB debug side band data setup
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            Aif2Fl_DbSideData     Side data structure for debug

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_DEBUG_SBND_DIO_WR_EN,AIF2_DB_IDB_DEBUG_SBND_FIFO_WR_EN,
 *        AIF2_DB_IDB_DEBUG_SBND_SOP,AIF2_DB_IDB_DEBUG_SBND_EOP,AIF2_DB_IDB_DEBUG_SBND_CH_ID,
 *        AIF2_DB_IDB_DEBUG_SBND_DIO_ADDR,AIF2_DB_IDB_DEBUG_SBND_XCNT, AIF2_DB_IDB_DEBUG_SBND_SYMBOL
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_DbSideData   SideData;
        ............
        Aif2Fl_inDbDebugSideDataSetup(hAif2, SideData);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbDebugSideDataSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DbSideData             side_data
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_DIO_WR_EN, side_data.bEnDioBufferWrite)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_FIFO_WR_EN, side_data.bEnFifoBufferWrite)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_SOP, side_data.bSop)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_EOP, side_data.bEop)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_CH_ID, side_data.ChannelId)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_DIO_ADDR, side_data.DioAddress)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_XCNT, side_data.xcnt)  |
              CSL_FMK(AIF2_DB_IDB_DEBUG_SBND_SYMBOL, side_data.Symbol);

    hAif2->regs->DB_IDB_DEBUG_SBND = tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_inDbDebugWrite
 *
 *   @b Description
 *   @n Writes the data in the following registers into the Ingress DB and sideband RAMS (for customer debug purpose only)
 *         DB_IDB_DEBUG_D0, DB_IDB_DEBUG_D1, DB_IDB_DEBUG_D2, DB_IDB_DEBUG_D3, DB_IDB_DEBUG_SBDN 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            uint16_t     True : Write data and side data and it will be transferred to Egress Debug side

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open(), Aif2Fl_inDbDebugDataSetup(), Aif2Fl_inDbDebugSideDataSetup(), Aif2Fl_inDbEnDisDebug()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_DEBUG_DB_WR_DONT_CARE,
 *        
 *
 *   @b Example
 *   @verbatim
        uint16_t  arg = true;
        Aif2Fl_inDbDebugWrite(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbDebugWrite(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       CSL_FINS(hAif2->regs->DB_IDB_DEBUG_DB_WR, AIF2_DB_IDB_DEBUG_DB_WR_DONT_CARE, arg);  
}


/** ============================================================================
 *   @n@b Aif2Fl_inDbDebugOffsetAddr
 *
 *   @b Description
 *   @n Set write and read Address(channel number) used to access write or read Offset RAM for DB Debug (getting debug ram offset could be useful function for sw developer)
 *        Real Offset value of DB debug Ram could be obtained using Aif2Fl_inDbDebugGetOffsetData() function.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            uint8_t*            arg[0] : write offset address(channel num),   arg[1] : read offset address(channel num)
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open(), Aif2Fl_inDbDebugDataSetup(), Aif2Fl_inDbDebugSideDataSetup(), Aif2Fl_inDbEnDisDebug(), Aif2Fl_inDbDebugWrite()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_DEBUG_OFS_WADDR, AIF2_DB_IDB_DEBUG_OFS_RADDR
 *
 *   @b Example
 *   @verbatim
        uint8_t   debug_addr[2];

        debug_addr[0] = write_address; //0~ 127
        debug_addr[1] = read_address; //0~ 127
        
        Aif2Fl_inDbDebugOffsetAddr(hAif2, &debug_addr[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbDebugOffsetAddr(
    Aif2Fl_Handle    hAif2,
    uint8_t                    *offset_addr
)
{
       uint32_t tempReg;
       tempReg = CSL_FMK(AIF2_DB_IDB_DEBUG_OFS_WADDR, offset_addr[0])|  
	                 CSL_FMK(AIF2_DB_IDB_DEBUG_OFS_RADDR, offset_addr[1]);  
	hAif2->regs->DB_IDB_DEBUG_OFS = tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_inDbChEnable
 *
 *   @b Description
 *   @n Enable or Disable Ingress DB channel to add or remove channel dynamically
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            uint32_t*        arg[0] ~ arg[3] : bit 0 ~ 127 for all channels
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_CH_EN_EN
 *        
 *
 *   @b Example
 *   @verbatim
        uint32_t   InDbChannel[4];

        InDbChannel[0] = 0x00000001;//channel 31 ~ 0
        InDbChannel[1] = 0x0....         //channel 63 ~ 32
        InDbChannel[2] = 0x0....         //channel 95 ~ 64
        InDbChannel[3] = 0x0....         //channel 127 ~ 96
        
        Aif2Fl_inDbChEnable(hAif2, &InDbChannel[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbChEnable(
    Aif2Fl_Handle    hAif2,
    uint32_t                    *channel
)
{
       CSL_FINS(hAif2->regs->DB_IDB_CH_EN[0], AIF2_DB_IDB_CH_EN_EN, channel[0]);  
       CSL_FINS(hAif2->regs->DB_IDB_CH_EN[1], AIF2_DB_IDB_CH_EN_EN, channel[1]);  
       CSL_FINS(hAif2->regs->DB_IDB_CH_EN[2], AIF2_DB_IDB_CH_EN_EN, channel[2]);  
       CSL_FINS(hAif2->regs->DB_IDB_CH_EN[3], AIF2_DB_IDB_CH_EN_EN, channel[3]);  
}


/** ============================================================================
 *   @n@b Aif2Fl_inDbChannelSetup
 *
 *   @b Description
 *   @n Setup Ingress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            Aif2Fl_DbChannel        aif2 Db channel setup structure
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_PTR_CH_BASE_ADDR,AIF2_DB_IDB_PTR_CH_BUF_DEPTH,
 *        AIF2_DB_IDB_CFG_CH_DAT_SWAP, AIF2_DB_IDB_CFG_CH_IQ_ORDER,
 *        AIF2_DB_IDB_CFG_CH_PS_EN, AIF2_DB_IDB_CFG_CH_PKT_TYPE
 *   @b Example
 *   @verbatim
        Aif2Fl_DbChannel   newChannel;

        newChannel.ChannelNum = 3;
        ........................................
        newChannel.PacketType = 0;
        
        Aif2Fl_inDbChannelSetup(hAif2, newChannel);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_inDbChannelSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DbChannel                channel_setup
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_DB_IDB_PTR_CH_BASE_ADDR, channel_setup.BaseAddress)  |
              CSL_FMK(AIF2_DB_IDB_PTR_CH_BUF_DEPTH, channel_setup.BufDepth);

    hAif2->regs->DB_IDB_PTR_CH[channel_setup.ChannelNum] = tempReg;
	
    tempReg = CSL_FMK(AIF2_DB_IDB_CFG_CH_DAT_SWAP, channel_setup.DataSwap)  |
              CSL_FMK(AIF2_DB_IDB_CFG_CH_IQ_ORDER, channel_setup.IQOrder)  |
              CSL_FMK(AIF2_DB_IDB_CFG_CH_PS_EN, channel_setup.bEnablePsData)  |
              CSL_FMK(AIF2_DB_IDB_CFG_CH_PKT_TYPE, channel_setup.PacketType);

    hAif2->regs->DB_IDB_CFG_CH[channel_setup.ChannelNum]  = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_eDbEnDisDebug
 *
 *   @b Description
 *   @n Enables Egress DB Debug mode for customer SW debug 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            uint16_t     true : Debug on    false : Debug off

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_CFG_EDB_DEBUG_EN
 *      
 *
 *   @b Example
 *   @verbatim
        uint16_t  arg = true;
        Aif2Fl_eDbEnDisDebug (hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbEnDisDebug(
    Aif2Fl_Handle    hAif2,
    uint16_t            arg
)
{
        CSL_FINS(hAif2->regs->DB_EDB_CFG, AIF2_DB_EDB_CFG_EDB_DEBUG_EN, arg);  
}
    

/** ============================================================================
 *   @n@b Aif2Fl_eDbDebugReadControl
 *
 *   @b Description
 *   @n Setup Side band data control info like dio read enable and channel id
 *        
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            Aif2Fl_DbSideData     only dio_rd_en, ch_id field will be used

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_DEBUG_RD_CNTL_DIO_RD_EN,
 *      AIF2_DB_EDB_DEBUG_RD_CNTL_CH_ID
 *   
 *   @b Example
 *   @verbatim
        Aif2Fl_DbSideData  side_control ;
        
        side_control.bEnFifoBufferRead = true;
        side_control.ChannelId = 0;
        
        Aif2Fl_eDbDebugReadControl(hAif2, side_control);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbDebugReadControl(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DbSideData           side_data
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_DB_EDB_DEBUG_RD_CNTL_DIO_RD_EN, side_data.bEnDioBufferRead)  |
              CSL_FMK(AIF2_DB_EDB_DEBUG_RD_CNTL_CH_ID, side_data.ChannelId);
         
    hAif2->regs->DB_EDB_DEBUG_RD_CNTL = tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_eDbDebugWrToken
 *
 *   @b Description
 *   @n when this register is set to true, the value loaded into DB_EDB_DEBUG_RD_CNTL.CH_ID 
 *       being issued to the AxC Token FIFO and 64 bytes of packet data will be transfered to Egress DB RAM from L2. 
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            uint16_t     True : Read data from L2 to Egress DB ram
 *   @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open(), Aif2Fl_eDbEnDisDebug(), Aif2Fl_eDbDebugReadControl()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_DEBUG_WR_TOK_DONT_CARE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = true;
        Aif2Fl_eDbDebugWrToken(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbDebugWrToken(
    Aif2Fl_Handle    hAif2,
    uint16_t              arg
)
{
       CSL_FINS(hAif2->regs->DB_EDB_DEBUG_WR_TOK, AIF2_DB_EDB_DEBUG_WR_TOK_DONT_CARE, arg);  
}


/** ============================================================================
 *   @n@b Aif2Fl_eDbDebugRead
 *
 *   @b Description
 *   @n  Reads the data into the following registers from the Egress DB and sideband RAMS
 *         DB_EDB_DEBUG_D0, DB_EDB_DEBUG_D1, DB_EDB_DEBUG_D2, DB_EDB_DEBUG_D3, DB_EDB_DEBUG_SBDN 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance
            
            uint16_t     True : Read data and side data in the debug data RAM

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  *   @n  Aif2Fl_init(), Aif2Fl_open(), Aif2Fl_eDbDebugReadControl()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_DEBUG_DB_RD_DONT_CARE,
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = true;
        Aif2Fl_eDbDebugRead(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbDebugRead(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       CSL_FINS(hAif2->regs->DB_EDB_DEBUG_DB_RD, AIF2_DB_EDB_DEBUG_DB_RD_DONT_CARE, arg);  
}


/** ============================================================================
 *   @n@b Aif2Fl_eDbDebugOffsetAddr
 *
 *   @b Description
 *   @n Set write and read Address(channel number) used to access write or read Offset RAM for DB Debug (getting debug ram offset could be useful function for sw developer)
 *        Real Offset value of DB debug Ram could be obtained using Aif2Fl_eDbDebugGetOffsetData() function.
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            uint8_t*            arg[0] : write offset address(channel num),   arg[1] : read offset address(channel num)
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_DEBUG_OFS_WADDR, AIF2_DB_EDB_DEBUG_OFS_RADDR
 *        
 *
 *   @b Example
 *   @verbatim
        uint8_t   debug_addr[2];

        debug_addr[0] = write_address; //0~ 127
        debug_addr[1] = read_address; //0~ 127
        
        Aif2Fl_eDbDebugOffsetAddr(hAif2, &debug_addr[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbDebugOffsetAddr(
    Aif2Fl_Handle    hAif2,
    uint8_t                    *offset_addr
)
{
       uint32_t  tempReg;
       tempReg = CSL_FMK(AIF2_DB_EDB_DEBUG_OFS_WADDR, offset_addr[0])|
	                 CSL_FMK(AIF2_DB_EDB_DEBUG_OFS_RADDR, offset_addr[1]);  
	hAif2->regs->DB_EDB_DEBUG_OFS = tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_eDbChEnable
 *
 *   @b Description
 *   @n Enable or Disable Egress DB channel to add or remove channel dynamically
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            uint32_t*        arg[0] ~ arg[3] : bit 0 ~ 127 for all channels
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_CH_EN_EN
 *        
 *
 *   @b Example
 *   @verbatim
        uint32_t   InDbChannel[4];

        InDbChannel[0] = 0x00000001;//channel 31 ~ 0
        InDbChannel[1] = 0x0....         //channel 63 ~ 32
        InDbChannel[2] = 0x0....         //channel 95 ~ 64
        InDbChannel[3] = 0x0....         //channel 127 ~ 96
        
        Aif2Fl_eDbChEnable(hAif2, &InDbChannel[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbChEnable(
    Aif2Fl_Handle    hAif2,
    uint32_t                    *channel
)
{
       CSL_FINS(hAif2->regs->DB_EDB_CH_EN[0], AIF2_DB_EDB_CH_EN_EN, channel[0]);  
       CSL_FINS(hAif2->regs->DB_EDB_CH_EN[1], AIF2_DB_EDB_CH_EN_EN, channel[1]);  
       CSL_FINS(hAif2->regs->DB_EDB_CH_EN[2], AIF2_DB_EDB_CH_EN_EN, channel[2]);  
       CSL_FINS(hAif2->regs->DB_EDB_CH_EN[3], AIF2_DB_EDB_CH_EN_EN, channel[3]);  
}


/** ============================================================================
 *   @n@b Aif2Fl_eDbChannelSetup
 *
 *   @b Description
 *   @n Setup Egress DB channel to add or remove channel dynamically (argument type: Aif2Fl_DbChannel *)
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance
            
            Aif2Fl_DbChannel        aif2 Db channel setup structure
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_PTR_CH_BASE_ADDR,AIF2_DB_EDB_PTR_CH_BUF_DEPTH,
 *        AIF2_DB_EDB_CFG_CH_DAT_SWAP, AIF2_DB_EDB_CFG_CH_IQ_ORDER,AIF2_DB_EDB_CFG_CH_DIO_OFFSET
 *   @b Example
 *   @verbatim
        Aif2Fl_DbChannel   newChannel;

        newChannel.ChannelNum = 3;
        ........................................
        newChannel.EgressDioOffset = 0;
        
        Aif2Fl_eDbChannelSetup(hAif2, newChannel);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eDbChannelSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_DbChannel                channel_setup
)
{
    uint32_t tempReg;
    
    tempReg = CSL_FMK(AIF2_DB_EDB_PTR_CH_BASE_ADDR, channel_setup.BaseAddress)  |
              CSL_FMK(AIF2_DB_EDB_PTR_CH_BUF_DEPTH, channel_setup.BufDepth);

    hAif2->regs->DB_EDB_PTR_CH[channel_setup.ChannelNum] = tempReg;
	
    tempReg = CSL_FMK(AIF2_DB_EDB_CFG_CH_DAT_SWAP, channel_setup.DataSwap)  |
              CSL_FMK(AIF2_DB_EDB_CFG_CH_IQ_ORDER, channel_setup.IQOrder)|
              CSL_FMK(AIF2_DB_EDB_CFG_CH_DIO_OFFSET, channel_setup.EgressDioOffset);
    hAif2->regs->DB_EDB_CFG_CH[channel_setup.ChannelNum]  = tempReg;
}


/** ============================================================================
 *   @n@b Aif2Fl_adInGlobalEnableDisable
 *
 *   @b Description
 *   @n Enable or Disable Global Ingress AD module dynamically to support CPPI channel tearing down
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.   
            
            uint16_t     true : Enable  false : Disable
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open() 
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_ISCH_GLOBAL_EN_SET_DONT_CARE;AIF2_AD_ISCH_GLOBAL_EN_CLR_DONT_CARE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = false;
        
        Aif2Fl_adInGlobalEnableDisable(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adInGlobalEnableDisable(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       if(arg == true)
       CSL_FINS(hAif2->regs->AD_ISCH_GLOBAL_EN_SET, AIF2_AD_ISCH_GLOBAL_EN_SET_DONT_CARE, true);  
	else 
	CSL_FINS(hAif2->regs->AD_ISCH_GLOBAL_EN_CLR, AIF2_AD_ISCH_GLOBAL_EN_CLR_DONT_CARE, true);  
}


/** ============================================================================
 *   @n@b Aif2Fl_adEGlobalEnableDisable
 *
 *   @b Description
 *   @n Enable or Disable Global Egress AD module dynamically to support CPPI channel tearing down
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.   
            
            uint16_t     true : Enable  false : Disable
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open() 
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_ESCH_GLOBAL_EN_SET_DONT_CARE;AIF2_AD_ESCH_GLOBAL_EN_CLR_DONT_CARE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = false;
        
        Aif2Fl_adEGlobalEnableDisable(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEGlobalEnableDisable(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       if(arg == true)
       CSL_FINS(hAif2->regs->AD_ESCH_GLOBAL_EN_SET, AIF2_AD_ESCH_GLOBAL_EN_SET_DONT_CARE, true);  
	else 
	CSL_FINS(hAif2->regs->AD_ESCH_GLOBAL_EN_CLR, AIF2_AD_ESCH_GLOBAL_EN_CLR_DONT_CARE, true);  
}


/** ============================================================================
 *   @n@b Aif2Fl_adInDioGlobalEnableDisable
 *
 *   @b Description
 *   @n Enable or Disable Global Ingress DIO mode dynamically to support DIO channel tearing down
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.   
            
            uint16_t     true : Enable  false : Disable
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open() 
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE;AIF2_AD_DIO_I_GLOBAL_EN_CLR_DONT_CARE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = false;
        
        Aif2Fl_adInDioGlobalEnableDisable(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adInDioGlobalEnableDisable(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       if(arg == true)
       CSL_FINS(hAif2->regs->AD_DIO_I_GLOBAL_EN_SET, AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE, true);  
	else 
	CSL_FINS(hAif2->regs->AD_DIO_I_GLOBAL_EN_CLR, AIF2_AD_DIO_I_GLOBAL_EN_CLR_DONT_CARE, true);  
}


/** ============================================================================
 *   @n@b Aif2Fl_adEDioGlobalEnableDisable
 *
 *   @b Description
 *   @n Enable or Disable Global Egress DIO mode dynamically to support DIO channel tearing down
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.   
            
            uint16_t     true : Enable  false : Disable
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open() 
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_E_GLOBAL_EN_SET_DONT_CARE;AIF2_AD_DIO_E_GLOBAL_EN_CLR_DONT_CARE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = false;
        
        Aif2Fl_adEDioGlobalEnableDisable(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEDioGlobalEnableDisable(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       if(arg == true)
       CSL_FINS(hAif2->regs->AD_DIO_E_GLOBAL_EN_SET, AIF2_AD_DIO_E_GLOBAL_EN_SET_DONT_CARE, true);  
	else 
	CSL_FINS(hAif2->regs->AD_DIO_E_GLOBAL_EN_CLR, AIF2_AD_DIO_E_GLOBAL_EN_CLR_DONT_CARE, true);  
}


/** ============================================================================
 *   @n@b Aif2Fl_adInDioTableSelect
 *
 *   @b Description
 *   @n Change Ingress DIO table selection dynamically. NumAxC and BCN table setup should be done before changing table select
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t     0 : Table0  1 : Table1

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open() , Aif2Fl_adInDioNumAxC(),  Aif2Fl_adInDioBcnTableSetup()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_I_TABLE_SEL_BCN_TABLE_SEL
 *        
 *   @b Example
 *   @verbatim
        uint8_t  table = 1;
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adInDioTableSelect(hAif2, table);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adInDioTableSelect(
    Aif2Fl_Handle    hAif2,
    uint8_t                    arg
)
{
       CSL_FINS(hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_TABLE_SEL, AIF2_AD_DIO_I_TABLE_SEL_BCN_TABLE_SEL, arg);  
}

/** ============================================================================
 *   @n@b Aif2Fl_adInDioNumAxC
 *
 *   @b Description
 *   @n Change Ingress DIO AxC number dynamically 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t     total number of Antenna carrier

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC
 *        
 *   @b Example
 *   @verbatim
        uint8_t numAxC = 8;
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adInDioNumAxC(hAif2, numAxC);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adInDioNumAxC(
    Aif2Fl_Handle    hAif2,
    uint8_t                    arg
)
{
	if(arg > 0)
    CSL_FINS(hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_TABLE_LOOP_CFG, AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC, (arg-1)); 
    else
    CSL_FINS(hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_TABLE_LOOP_CFG, AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC, arg); 
}


/** ============================================================================
 *   @n@b Aif2Fl_adInDioBcnTableSetup
 *
 *   @b Description
 *   @n Change Ingress DIO BCN table dynamically. this function will write new DBCN value to the table which is not used. 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t*     DBCN array which has 64 length

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n 
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN0,AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN1,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN2,AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN3,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN4,AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN5,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN6,AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN7,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN8,AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN9,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN10,AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN11,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN12,AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN13,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN14,AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN15,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN16,AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN17,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN18,AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN19,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN20,AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN21,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN22,AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN23,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN24,AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN25,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN26,AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN27,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN28,AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN29,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN30,AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN31,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN32,AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN33,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN34,AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN35,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN36,AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN37,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN38,AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN39,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN40,AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN41,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN42,AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN43,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN44,AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN45,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN46,AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN47,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN48,AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN49,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN50,AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN51,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN52,AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN53,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN54,AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN55,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN56,AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN57,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN58,AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN59,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN60,AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN61,
 *        AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN62,AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN63;
 *
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN0,AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN1,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN2,AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN3,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN4,AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN5,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN6,AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN7,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN8,AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN9,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN10,AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN11,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN12,AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN13,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN14,AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN15,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN16,AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN17,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN18,AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN19,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN20,AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN21,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN22,AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN23,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN24,AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN25,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN26,AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN27,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN28,AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN29,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN30,AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN31,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN32,AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN33,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN34,AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN35,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN36,AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN37,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN38,AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN39,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN40,AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN41,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN42,AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN43,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN44,AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN45,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN46,AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN47,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN48,AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN49,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN50,AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN51,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN52,AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN53,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN54,AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN55,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN56,AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN57,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN58,AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN59,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN60,AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN61,
 *        AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN62,AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN63;
 *   @b Example
 *   @verbatim
        uint8_t DBCN[64];
        DBCN[0] = .....
        DBCN[1] = .....
        ..............
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adInDioBcnTableSetup(hAif2, &DBCN[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adInDioBcnTableSetup(
    Aif2Fl_Handle    hAif2,
    uint8_t*                    DBCN
)
{
       uint32_t  tempReg;
	   
       if(hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_TABLE_SEL == AIF2FL_AD_DIO_BCN_TABLE_1){
             /* setup DBCN for table 0  */
	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN0, DBCN[0])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN1, DBCN[1])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN2, DBCN[2])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN3, DBCN[3]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW0 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN4, DBCN[4])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN5, DBCN[5])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN6, DBCN[6])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN7, DBCN[7]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW1 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN8, DBCN[8])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN9, DBCN[9])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN10, DBCN[10])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN11, DBCN[11]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW2 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN12, DBCN[12])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN13, DBCN[13])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN14, DBCN[14])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN15, DBCN[15]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW3 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN16, DBCN[16])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN17, DBCN[17])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN18, DBCN[18])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN19, DBCN[19]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW4 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN20, DBCN[20])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN21, DBCN[21])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN22, DBCN[22])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN23, DBCN[23]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW5 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN24, DBCN[24])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN25, DBCN[25])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN26, DBCN[26])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN27, DBCN[27]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW6 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN28, DBCN[28])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN29, DBCN[29])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN30, DBCN[30])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN31, DBCN[31]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW7 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN32, DBCN[32])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN33, DBCN[33])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN34, DBCN[34])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN35, DBCN[35]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW8 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN36, DBCN[36])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN37, DBCN[37])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN38, DBCN[38])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN39, DBCN[39]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW9 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN40, DBCN[40])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN41, DBCN[41])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN42, DBCN[42])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN43, DBCN[43]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW10 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN44, DBCN[44])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN45, DBCN[45])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN46, DBCN[46])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN47, DBCN[47]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW11 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN48, DBCN[48])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN49, DBCN[49])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN50, DBCN[50])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN51, DBCN[51]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW12 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN52, DBCN[52])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN53, DBCN[53])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN54, DBCN[54])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN55, DBCN[55]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW13 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN56, DBCN[56])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN57, DBCN[57])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN58, DBCN[58])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN59, DBCN[59]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW14 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN60, DBCN[60])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN61, DBCN[61])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN62, DBCN[62])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN63, DBCN[63]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE0_ROW15 = tempReg;
       }
	else
	{
              /* setup DBCN for table 1  */
	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN0, DBCN[0])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN1, DBCN[1])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN2, DBCN[2])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN3, DBCN[3]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW0 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN4, DBCN[4])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN5, DBCN[5])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN6, DBCN[6])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN7, DBCN[7]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW1 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN8, DBCN[8])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN9, DBCN[9])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN10, DBCN[10])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN11, DBCN[11]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW2 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN12, DBCN[12])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN13, DBCN[13])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN14, DBCN[14])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN15, DBCN[15]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW3 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN16, DBCN[16])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN17, DBCN[17])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN18, DBCN[18])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN19, DBCN[19]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW4 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN20, DBCN[20])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN21, DBCN[21])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN22, DBCN[22])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN23, DBCN[23]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW5 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN24, DBCN[24])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN25, DBCN[25])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN26, DBCN[26])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN27, DBCN[27]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW6 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN28, DBCN[28])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN29, DBCN[29])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN30, DBCN[30])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN31, DBCN[31]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW7 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN32, DBCN[32])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN33, DBCN[33])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN34, DBCN[34])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN35, DBCN[35]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW8 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN36, DBCN[36])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN37, DBCN[37])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN38, DBCN[38])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN39, DBCN[39]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW9 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN40, DBCN[40])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN41, DBCN[41])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN42, DBCN[42])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN43, DBCN[43]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW10 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN44, DBCN[44])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN45, DBCN[45])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN46, DBCN[46])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN47, DBCN[47]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW11 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN48, DBCN[48])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN49, DBCN[49])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN50, DBCN[50])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN51, DBCN[51]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW12 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN52, DBCN[52])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN53, DBCN[53])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN54, DBCN[54])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN55, DBCN[55]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW13 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN56, DBCN[56])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN57, DBCN[57])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN58, DBCN[58])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN59, DBCN[59]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW14 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN60, DBCN[60])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN61, DBCN[61])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN62, DBCN[62])  |
	                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN63, DBCN[63]);
	hAif2->regs->AD_DIO_INGRESS[hAif2->arg_dioEngine].AD_DIO_I_BCN_TABLE1_ROW15 = tempReg;
      }

}


/** ============================================================================
 *   @n@b Aif2Fl_adEDioTableSelect
 *
 *   @b Description
 *   @n Change Egress DIO table selection dynamically. NumAxC and BCN table setup should be done before changing table select
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t     0 : Table0  1 : Table1

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open(), Aif2Fl_adEDioNumAxC(),  Aif2Fl_adEDioBcnTableSetup()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_E_TABLE_SEL_BCN_TABLE_SEL
 *        
 *   @b Example
 *   @verbatim
        uint8_t  table = 1;
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adEDioTableSelect(hAif2, table);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEDioTableSelect(
    Aif2Fl_Handle    hAif2,
    uint8_t                    arg
)
{
       CSL_FINS(hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_TABLE_SEL, AIF2_AD_DIO_E_TABLE_SEL_BCN_TABLE_SEL, arg);  
}

/** ============================================================================
 *   @n@b Aif2Fl_adEDioNumAxC
 *
 *   @b Description
 *   @n Change Egress DIO AxC number dynamically 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t     total number of Antenna carrier

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC
 *        
 *   @b Example
 *   @verbatim
        uint8_t numAxC = 8;
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adEDioNumAxC(hAif2, numAxC);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEDioNumAxC(
    Aif2Fl_Handle    hAif2,
    uint8_t                    arg
)
{
	if(arg > 0)
    CSL_FINS(hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_TABLE_LOOP_CFG, AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC, (arg-1));  
    else
    CSL_FINS(hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_TABLE_LOOP_CFG, AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC, arg); 
}


/** ============================================================================
 *   @n@b Aif2Fl_adEDioBcnTableSetup
 *
 *   @b Description
 *   @n Change Egress DIO BCN table dynamically. this function will write new DBCN value to the table which is not used. 
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_dioEngine to select dio engine
            
            uint8_t*     DBCN array which has 64 length

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n 
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN0,AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN1,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN2,AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN3,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN4,AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN5,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN6,AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN7,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN8,AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN9,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN10,AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN11,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN12,AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN13,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN14,AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN15,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN16,AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN17,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN18,AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN19,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN20,AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN21,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN22,AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN23,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN24,AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN25,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN26,AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN27,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN28,AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN29,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN30,AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN31,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN32,AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN33,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN34,AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN35,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN36,AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN37,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN38,AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN39,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN40,AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN41,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN42,AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN43,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN44,AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN45,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN46,AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN47,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN48,AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN49,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN50,AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN51,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN52,AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN53,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN54,AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN55,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN56,AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN57,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN58,AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN59,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN60,AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN61,
 *        AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN62,AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN63;
 *
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN0,AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN1,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN2,AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN3,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN4,AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN5,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN6,AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN7,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN8,AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN9,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN10,AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN11,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN12,AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN13,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN14,AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN15,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN16,AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN17,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN18,AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN19,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN20,AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN21,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN22,AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN23,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN24,AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN25,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN26,AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN27,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN28,AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN29,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN30,AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN31,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN32,AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN33,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN34,AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN35,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN36,AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN37,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN38,AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN39,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN40,AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN41,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN42,AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN43,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN44,AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN45,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN46,AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN47,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN48,AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN49,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN50,AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN51,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN52,AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN53,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN54,AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN55,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN56,AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN57,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN58,AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN59,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN60,AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN61,
 *        AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN62,AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN63;
 *   @b Example
 *   @verbatim
        uint8_t DBCN[64];
        DBCN[0] = .....
        DBCN[1] = .....
        ..............
        hAif2->arg_dioEngine = 1;//Engine 1
        
        Aif2Fl_adEDioBcnTableSetup(hAif2, &DBCN[0]);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEDioBcnTableSetup(
    Aif2Fl_Handle    hAif2,
    uint8_t*                    DBCN
)
{
       uint32_t  tempReg;
	   
       if(hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_TABLE_SEL == AIF2FL_AD_DIO_BCN_TABLE_1){
             /* setup DBCN for table 0  */
	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN0, DBCN[0])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN1, DBCN[1])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN2, DBCN[2])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN3, DBCN[3]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW0 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN4, DBCN[4])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN5, DBCN[5])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN6, DBCN[6])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN7, DBCN[7]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW1 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN8, DBCN[8])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN9, DBCN[9])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN10, DBCN[10])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN11, DBCN[11]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW2 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN12, DBCN[12])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN13, DBCN[13])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN14, DBCN[14])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN15, DBCN[15]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW3 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN16, DBCN[16])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN17, DBCN[17])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN18, DBCN[18])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN19, DBCN[19]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW4 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN20, DBCN[20])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN21, DBCN[21])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN22, DBCN[22])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN23, DBCN[23]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW5 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN24, DBCN[24])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN25, DBCN[25])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN26, DBCN[26])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN27, DBCN[27]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW6 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN28, DBCN[28])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN29, DBCN[29])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN30, DBCN[30])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN31, DBCN[31]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW7 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN32, DBCN[32])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN33, DBCN[33])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN34, DBCN[34])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN35, DBCN[35]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW8 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN36, DBCN[36])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN37, DBCN[37])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN38, DBCN[38])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN39, DBCN[39]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW9 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN40, DBCN[40])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN41, DBCN[41])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN42, DBCN[42])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN43, DBCN[43]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW10 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN44, DBCN[44])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN45, DBCN[45])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN46, DBCN[46])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN47, DBCN[47]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW11 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN48, DBCN[48])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN49, DBCN[49])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN50, DBCN[50])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN51, DBCN[51]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW12 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN52, DBCN[52])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN53, DBCN[53])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN54, DBCN[54])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN55, DBCN[55]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW13 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN56, DBCN[56])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN57, DBCN[57])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN58, DBCN[58])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN59, DBCN[59]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW14 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN60, DBCN[60])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN61, DBCN[61])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN62, DBCN[62])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN63, DBCN[63]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE0_ROW15 = tempReg;
       }
	else
	{
              /* setup DBCN for table 1  */
	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN0, DBCN[0])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN1, DBCN[1])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN2, DBCN[2])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN3, DBCN[3]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW0 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN4, DBCN[4])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN5, DBCN[5])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN6, DBCN[6])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN7, DBCN[7]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW1 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN8, DBCN[8])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN9, DBCN[9])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN10, DBCN[10])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN11, DBCN[11]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW2 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN12, DBCN[12])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN13, DBCN[13])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN14, DBCN[14])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN15, DBCN[15]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW3 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN16, DBCN[16])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN17, DBCN[17])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN18, DBCN[18])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN19, DBCN[19]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW4 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN20, DBCN[20])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN21, DBCN[21])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN22, DBCN[22])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN23, DBCN[23]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW5 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN24, DBCN[24])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN25, DBCN[25])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN26, DBCN[26])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN27, DBCN[27]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW6 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN28, DBCN[28])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN29, DBCN[29])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN30, DBCN[30])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN31, DBCN[31]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW7 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN32, DBCN[32])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN33, DBCN[33])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN34, DBCN[34])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN35, DBCN[35]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW8 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN36, DBCN[36])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN37, DBCN[37])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN38, DBCN[38])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN39, DBCN[39]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW9 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN40, DBCN[40])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN41, DBCN[41])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN42, DBCN[42])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN43, DBCN[43]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW10 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN44, DBCN[44])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN45, DBCN[45])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN46, DBCN[46])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN47, DBCN[47]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW11 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN48, DBCN[48])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN49, DBCN[49])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN50, DBCN[50])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN51, DBCN[51]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW12 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN52, DBCN[52])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN53, DBCN[53])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN54, DBCN[54])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN55, DBCN[55]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW13 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN56, DBCN[56])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN57, DBCN[57])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN58, DBCN[58])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN59, DBCN[59]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW14 = tempReg;

	tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN60, DBCN[60])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN61, DBCN[61])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN62, DBCN[62])  |
	                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN63, DBCN[63]);
	hAif2->regs->AD_DIO_EGRESS[hAif2->arg_dioEngine].AD_DIO_E_BCN_TABLE1_ROW15 = tempReg;
      }

}


/** ============================================================================
 *   @n@b Aif2Fl_adEnDisDtDmaCh
 *
 *   @b Description
 *   @n Sets Enable or Disable Data trace DMA Channel for data and framing data
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   
            
            uint16_t     true or false

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_RD_CH_EN,
 *        
 *   @b Example
 *   @verbatim
        
        Aif2Fl_adEnDisDtDmaCh(hAif2, true);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adEnDisDtDmaCh(
    Aif2Fl_Handle    hAif2,
    uint16_t                    arg
)
{
       uint32_t tempReg;
       tempReg = CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_RD_CH_EN, arg)|
	                 CSL_FMK(AIF2_AD_DIO_DT_DMA_CFG0_DT_DMA_FM_CH_EN, arg);  
	hAif2->regs->AD_DIO_DT_DMA_CFG0 = tempReg;
}

/** ============================================================================
 *   @n@b Aif2Fl_adDataTraceBaseAddr
 *
 *   @b Description
 *   @n Sets the destination VBUS base address (upper 28 bits of 32 bit data bus) for Rx data trace receive data.
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   
            
            uint32_t     upper 28 bits of 32 bit data bus (lower 4 bits will be set to zero)

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_DT_DMA_CFG1_DT_DMA_RD_BASE_ADDR
 *        
 *   @b Example
 *   @verbatim
        uint32_t  addr = 0x00480000;//could be L2, DDR3 or other type of memory address 
        
        Aif2Fl_adDataTraceBaseAddr(hAif2, addr);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adDataTraceBaseAddr(
    Aif2Fl_Handle    hAif2,
    uint32_t                    addr
)
{
       CSL_FINS(hAif2->regs->AD_DIO_DT_DMA_CFG1, AIF2_AD_DIO_DT_DMA_CFG1_DT_DMA_RD_BASE_ADDR, (addr >> 4));  
}

/** ============================================================================
 *   @n@b Aif2Fl_adFramingDataBaseAddr
 *
 *   @b Description
 *   @n Sets the destination VBUS base address (upper 28 bits of 32 bit data bus) for Rx framing data trace receive data.
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   
            
            uint32_t     upper 28 bits of 32 bit data bus (lower 4 bits will be set to zero)

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_DT_DMA_CFG2_DT_DMA_FM_BASE_ADDR
 *        
 *   @b Example
 *   @verbatim
        uint32_t  addr = 0x00480000;//could be L2, DDR3 or other type of memory address 
        
        Aif2Fl_adFramingDataBaseAddr(hAif2, addr);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adFramingDataBaseAddr(
    Aif2Fl_Handle    hAif2,
    uint32_t                    addr
)
{
       CSL_FINS(hAif2->regs->AD_DIO_DT_DMA_CFG2, AIF2_AD_DIO_DT_DMA_CFG2_DT_DMA_FM_BASE_ADDR, (addr >> 4));  
}


/** ============================================================================
 *   @n@b Aif2Fl_adDtDmaWrap
 *
 *   @b Description
 *   @n Sets the number of burst transfers before the destination address wraps back to the base address
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   
            
            uint32_t   Dma burst wrap(terminal count) value for DT

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_DT_DMA_CFG3_DT_DMA_WRAP
 *        
 *   @b Example
 *   @verbatim
        uint32_t  wrap =  32;
        
        Aif2Fl_adDtDmaChNum(hAif2, wrap);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_adDtDmaWrap(
    Aif2Fl_Handle    hAif2,
    uint32_t                    dma_wrap
)
{
       CSL_FINS(hAif2->regs->AD_DIO_DT_DMA_CFG3, AIF2_AD_DIO_DT_DMA_CFG3_DT_DMA_WRAP, dma_wrap);  
}


/** ============================================================================
 *   @n@b Aif2Fl_atEventSetup
 *
 *   @b Description
 *   @n Sets AT External Rad timer event and dio event dynamically. EventMask should be set to zero if the Event  is not for GSM
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   
            
            Aif2Fl_AtEvent     At event structure to setup offset, modulus and strobe select   

 *    @endverbatim
 *
 *   <b> Return Value </b>  None
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_EVENT_OFFSET_EVENTINDEX,AIF2_AT_EVENT_OFFSET_STROBESELECT,AIF2_AT_EVENT_MOD_TC_EVENTMODULO,
 *        AIF2_AT_EVENT_MASK_LSBS_EVENTMASK_LSBS,AIF2_AT_EVENT_MASK_MSBS_EVENTMASK_MSBS;
 *        AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX,AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT,
 *        AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO;
 *        AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX, AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,
 *        AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO;
 *   @b Example
 *   @verbatim
        Aif2Fl_AtEvent  Event; 
        Event.EventSelect = AIF2FL_EVENT_7;
        Event.Eventoffset = ....
        Event.EventModulo = ....
        ........
        Event.EventMaskLsb = 0;//if it is not GSM event, it should be zero
        Event.EventMaskLsb = 0;
        
        Aif2Fl_atEventSetup(hAif2, Event);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atEventSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtEvent                 Event
)
{
   uint32_t tempReg;

   if(Event.EventSelect < AIF2FL_IN_DIO_EVENT_0)
   {
   //Rad timer external event strobe select and offset value setup
   tempReg =   CSL_FMK(AIF2_AT_EVENT_OFFSET_EVENTINDEX, Event.EventOffset) |
                      CSL_FMK(AIF2_AT_EVENT_OFFSET_STROBESELECT, Event.EvtStrobeSel);
   hAif2->regs->AT_EVENTS[Event.EventSelect].AT_EVENT_OFFSET = tempReg;

   //Rad timer event modulus value and GSM mask setup
   CSL_FINS(hAif2->regs->AT_EVENTS[Event.EventSelect].AT_EVENT_MOD_TC, AIF2_AT_EVENT_MOD_TC_EVENTMODULO,Event.EventModulo);
   CSL_FINS(hAif2->regs->AT_EVENTS[Event.EventSelect].AT_EVENT_MASK_LSBS, AIF2_AT_EVENT_MASK_LSBS_EVENTMASK_LSBS, Event.EventMaskLsb);
   CSL_FINS(hAif2->regs->AT_EVENTS[Event.EventSelect].AT_EVENT_MASK_MSBS, AIF2_AT_EVENT_MASK_MSBS_EVENTMASK_MSBS, Event.EventMaskMsb);
   }
   else if(Event.EventSelect < AIF2FL_E_DIO_EVENT_0)
   {
    //Ingress DIO frame event strobe select and offset value setup
   tempReg =   CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX, Event.DioFrameEventOffset) |
                      CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT,  Event.DioFrameStrobeSel);
   hAif2->regs->AT_AD_INGR_EVENTS[(Event.EventSelect - AIF2FL_IN_DIO_EVENT_0) + 3].AT_AD_INGR_EVENT_OFFSET = tempReg;

   //Ingress DIO event strobe select and offset value setup
   tempReg =   CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX, Event.EventOffset) |
                      CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT, Event.EvtStrobeSel);
   hAif2->regs->AT_AD_INGR_EVENTS[(Event.EventSelect - AIF2FL_IN_DIO_EVENT_0)].AT_AD_INGR_EVENT_OFFSET = tempReg;

   //Ingress DIO event  modulus value setup
   CSL_FINS(hAif2->regs->AT_AD_INGR_EVENTS[(Event.EventSelect - AIF2FL_IN_DIO_EVENT_0) + 3].AT_AD_INGR_EVENT_MOD_TC, AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO, 3071999);
   CSL_FINS(hAif2->regs->AT_AD_INGR_EVENTS[(Event.EventSelect - AIF2FL_IN_DIO_EVENT_0)].AT_AD_INGR_EVENT_MOD_TC, AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO, Event.EventModulo);
   }
   else
   {
    //Egress DIO frame event strobe select and offset value setup
   tempReg =   CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX, Event.DioFrameEventOffset) |
                      CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,  Event.DioFrameStrobeSel);
   hAif2->regs->AT_AD_EGR_EVENTS[(Event.EventSelect - AIF2FL_E_DIO_EVENT_0) + 3].AT_AD_EGR_EVENT_OFFSET = tempReg;

   //Egress DIO  event strobe select and offset value setup
   tempReg =   CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX, Event.EventOffset) |
                      CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,  Event.EvtStrobeSel);
   hAif2->regs->AT_AD_EGR_EVENTS[(Event.EventSelect - AIF2FL_E_DIO_EVENT_0)].AT_AD_EGR_EVENT_OFFSET = tempReg;

   //Egress DIO event  modulus value setup
   CSL_FINS(hAif2->regs->AT_AD_EGR_EVENTS[(Event.EventSelect - AIF2FL_E_DIO_EVENT_0) + 3].AT_AD_EGR_EVENT_MOD_TC, AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO,3071999);
   CSL_FINS(hAif2->regs->AT_AD_EGR_EVENTS[(Event.EventSelect - AIF2FL_E_DIO_EVENT_0)].AT_AD_EGR_EVENT_MOD_TC, AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO, Event.EventModulo);
   }
   
}


/** ============================================================================
 *   @n@b Aif2Fl_atDeltaSetup
 *
 *   @b Description
 *   @n Sets Delta offset for specified link
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.   should use hAif2->arg_link to select link
            
            uint32_t   New Delta offset value   

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO
 *        
 *   @b Example
 *   @verbatim
        uint32_t  delta = 0x80; 
        
        Aif2Fl_atDeltaSetup(hAif2, delta);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atDeltaSetup(
    Aif2Fl_Handle    hAif2,
    uint32_t                delta
)
{
  /* setup Delta event offset and modulus TC value */
  CSL_FINS(hAif2->regs->AT_TM_DELTA_EVENTS[hAif2->arg_link].AT_TM_DELTA_EVENT_OFFSET, AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET, delta);
  CSL_FINS(hAif2->regs->AT_TM_DELTA_EVENTS[hAif2->arg_link].AT_TM_DELTA_EVENT_MOD_TC, AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO, 3071999);

}


/** ============================================================================
 *   @n@b Aif2Fl_atHaltTimer
 *
 *   @b Description
 *   @n Halt AT timers
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.  
            
            uint16_t     Enable Halt timers  

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_CONTROL2_HALT_TIMER,AIF2_AT_CONTROL2_ARM_TIMER
 *        
 *   @b Example
 *   @verbatim
        uint16_t  halt = true; 
        
        Aif2Fl_atHaltTimer(hAif2, halt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atHaltTimer(
    Aif2Fl_Handle    hAif2,
    uint16_t                halt
)
{
  CSL_FINS(hAif2->regs->AT_CONTROL2, AIF2_AT_CONTROL2_HALT_TIMER, halt);
  CSL_FINS(hAif2->regs->AT_CONTROL2, AIF2_AT_CONTROL2_ARM_TIMER, 0);
}


/** ============================================================================
 *   @n@b Aif2Fl_atDisableAllEvents
 *
 *   @b Description
 *   @n Disable All AT Events
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.  
            
            uint16_t     Disable all events

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE,AIF2_AT_EVT_ENABLE_EVENTENABLE
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arg = true; 
        
        Aif2Fl_atDisableAllEvents(hAif2, arg);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atDisableAllEvents(
    Aif2Fl_Handle    hAif2,
    uint16_t                arg
)
{
   if(arg == true){
   hAif2->regs->AT_EVT_ENABLE = 0;
   hAif2->regs->AT_INTERNAL_EVT_ENABLE = 0;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_atArmTimer
 *
 *   @b Description
 *   @n Arm AT timers
 *
 *   @b Arguments
 *   @verbatim

            hAif2    Handle to the aif2 instance.  
            
            uint16_t     Enable Arm timers  

     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_CONTROL2_ARM_TIMER,AIF2_AT_CONTROL2_HALT_TIMER
 *        
 *   @b Example
 *   @verbatim
        uint16_t  arm = true; 
        
        Aif2Fl_atArmTimer(hAif2, arm);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atArmTimer(
    Aif2Fl_Handle    hAif2,
    uint16_t                arm
)
{
  CSL_FINS(hAif2->regs->AT_CONTROL2, AIF2_AT_CONTROL2_ARM_TIMER, arm);
  CSL_FINS(hAif2->regs->AT_CONTROL2, AIF2_AT_CONTROL2_HALT_TIMER, 0);
}

/** ============================================================================
 *   @n@b Aif2Fl_atSwDebugSync
 *
 *   @b Description
 *   @n trigger Phy and Rad timer SW debug sync
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            uint16_t     Trigger debug sync  
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_SW_SYNC_PHY_SYNC,AIF2_AT_SW_SYNC_RAD_SYNC
 *        
 *   @b Example
 *   @verbatim
        uint16_t  sync = true; 
        
        Aif2Fl_atSwDebugSync(hAif2, sync);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atSwDebugSync(
    Aif2Fl_Handle    hAif2,
    uint16_t                sync
)
{

   uint32_t tempReg = 0;

   tempReg =   CSL_FMK(AIF2_AT_SW_SYNC_PHY_SYNC, sync) |
                      CSL_FMK(AIF2_AT_SW_SYNC_RAD_SYNC, sync);
   hAif2->regs->AT_SW_SYNC = tempReg;

}



/** ============================================================================
 *   @n@b Aif2Fl_atRadWcdmaDiv
 *
 *   @b Description
 *   @n at radt wcdma clock divider terminal count. This counter divides dualbyte clock down to 3.84MHz chip rate for wcdma counters
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            uint8_t     Terminal count  
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_RADT_WCDMA_DIV_TERMINALCOUNT
 *        
 *   @b Example
 *   @verbatim
        uint8_t  tc = 80; 
        
        Aif2Fl_atRadWcdmaDiv(hAif2, tc); //lower down 307.2 MHz to 3.84 MHz
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atRadWcdmaDiv(
    Aif2Fl_Handle    hAif2,
    uint8_t               tc
)
{
  CSL_FINS(hAif2->regs->AT_RADT_WCDMA_DIV, AIF2_AT_RADT_WCDMA_DIV_TERMINALCOUNT, tc);
}


/** ============================================================================
 *   @n@b Aif2Fl_atRadTcSetup
 *
 *   @b Description
 *   @n trigger Rad timer SW debug sync
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_AtTcObj     Trigger Rad debug sync  
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_RADT_SYMB_LUT_INDEX_TC_LUTINDEX_TC,AIF2_AT_RADT_SYMB_LUT_INDEX_TC_SYMBOLTC,
 *        AIF2_AT_RADT_FRAME_TC_LSBS_RADTFRAME_TC_LSBS,AIF2_AT_RADT_FRAME_TC_MSBS_RADTFRAME_TC_MSBS,
 *        AIF2_AT_RADT_SYM_LUT_RAM_RADT_CLOCK_TC
 *        
 *   @b Example
 *   @verbatim
        Aif2Fl_AtTcObj  Tc;
        Aif2Fl_AtCountObj  Count;

        Count.LutIndexNum = ....;
        ...................
        Tc.pRadTimerTc = &Count;
        Tc.RadClockCountTc[0] = ....;
        Tc.RadClockCountTc[1] = ....;
        .........
        
        Aif2Fl_atRadTcSetup(hAif2, Tc);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atRadTcSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtTcObj                tc
)
{
   uint32_t tempReg;
   uint8_t  count;
   // Rad timer symbol and lut index terminal count value setup
   tempReg =   CSL_FMK(AIF2_AT_RADT_SYMB_LUT_INDEX_TC_LUTINDEX_TC, tc.pRadTimerTc->LutIndexNum) |
                      CSL_FMK(AIF2_AT_RADT_SYMB_LUT_INDEX_TC_SYMBOLTC,  tc.pRadTimerTc->SymbolNum);
   hAif2->regs->AT_RADT_SYMB_LUT_INDEX_TC = tempReg;

   // Rad timer frame terminal count setup 
   CSL_FINS(hAif2->regs->AT_RADT_FRAME_TC_LSBS, AIF2_AT_RADT_FRAME_TC_LSBS_RADTFRAME_TC_LSBS, tc.pRadTimerTc->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_RADT_FRAME_TC_MSBS, AIF2_AT_RADT_FRAME_TC_MSBS_RADTFRAME_TC_MSBS, tc.pRadTimerTc->FrameMsbNum);

    // Rad, timer symbol lut ram to setup clock terminal count 
   for (count=0; count < (tc.pRadTimerTc->LutIndexNum +1); count++)
   {
         tempReg = CSL_FMK(AIF2_AT_RADT_SYM_LUT_RAM_RADT_CLOCK_TC, tc.RadClockCountTc[count]);
         hAif2->regs->AT_RADT_SYM_LUT_RAM[count] = tempReg;
   }
}



/** ============================================================================
 *   @n@b Aif2Fl_atGsmTcountSetup
 *
 *   @b Description
 *   @n Setup AT GSM Tcount 
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_AtGsmTCount     Tcount value structure 
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_GSM_TCOUNT_INIT_T1,AIF2_AT_GSM_TCOUNT_INIT_T2,
 *        AIF2_AT_GSM_TCOUNT_INIT_T3
 *        
 *        
 *   @b Example
 *   @verbatim
        Aif2Fl_AtGsmTCount  Tcount;
        
        Tcount.t1 = 13;
        Tcount.t2= 25;
        Tcount.t3 = 2048;
        .........
        
        Aif2Fl_atGsmTcountSetup(hAif2, Tcount);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atGsmTcountSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtGsmTCount                tc
)
{
   uint32_t tempReg;
   
   tempReg =   CSL_FMK(AIF2_AT_GSM_TCOUNT_INIT_T1, tc.t1) |
                      CSL_FMK(AIF2_AT_GSM_TCOUNT_INIT_T2,  tc.t2) |
                      CSL_FMK(AIF2_AT_GSM_TCOUNT_INIT_T3,  tc.t3);
   hAif2->regs->AT_GSM_TCOUNT_INIT = tempReg;
   
}

/** ============================================================================
 *   @n@b Aif2Fl_atEnableEvent
 *
 *   @b Description
 *   @n Enable Eight Rad and Six DIO Events
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_AtEventIndex         Event index to select event 
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_EVT_ENABLE_EVENTENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *        
 *   @b Reads
 *   @n AIF2_AT_EVT_ENABLE_EVENTENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_AtEventIndex    event = 1; //Rad event 1
        
        Aif2Fl_atEnableEvent(hAif2, event);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atEnableEvent(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtEventIndex              event_index
)
{
  uint32_t  Event = 0;
  
   if(event_index < AIF2FL_IN_DIO_EVENT_0)//Rad timer event enable
   {
      Event = CSL_FEXT(hAif2->regs->AT_EVT_ENABLE,AIF2_AT_EVT_ENABLE_EVENTENABLE);//Get current event enable status
      Event |= (uint32_t)(0x01 << event_index);
      CSL_FINS(hAif2->regs->AT_EVT_ENABLE, AIF2_AT_EVT_ENABLE_EVENTENABLE, Event);
   }
   else if(event_index< AIF2FL_E_DIO_EVENT_0)//Ingress DIO event  enable
   {
      Event = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE);
      Event |= (uint32_t)(0x9 << (event_index-AIF2FL_IN_DIO_EVENT_0));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE, Event);
   }
   else//Egress DIO event enable
   {
      Event = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE);
      Event |= (uint32_t)(0x9 << (event_index-AIF2FL_E_DIO_EVENT_0));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE, Event);
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_atDisableEvent
 *
 *   @b Description
 *   @n Disable Eight Rad and Six DIO Events
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_AtEventIndex         Event index to select event 
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_EVT_ENABLE_EVENTENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *        
 *   @b Reads
 *   @n AIF2_AT_EVT_ENABLE_EVENTENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE;AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_AtEventIndex    event = 1; //Rad event 1
        
        Aif2Fl_atDisableEvent(hAif2, event);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atDisableEvent(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtEventIndex              event_index
)
{
  uint32_t  Event = 0;
  
   if(event_index < AIF2FL_IN_DIO_EVENT_0)//Rad timer event disable
   {
      Event = CSL_FEXT(hAif2->regs->AT_EVT_ENABLE,AIF2_AT_EVT_ENABLE_EVENTENABLE);//Get current event enable status
      Event &= ~(uint32_t)(0x01 << event_index);
      CSL_FINS(hAif2->regs->AT_EVT_ENABLE, AIF2_AT_EVT_ENABLE_EVENTENABLE, Event);
   }
   else if(event_index< AIF2FL_E_DIO_EVENT_0)//Ingress DIO event  disable
   {
      Event = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE);
      Event &= ~(uint32_t)(0x03 << ((event_index-AIF2FL_IN_DIO_EVENT_0)*2));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE, Event);
   }
   else//Egress DIO event disable
   {
      Event = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE);
      Event &= ~(uint32_t)(0x03 << ((event_index-AIF2FL_E_DIO_EVENT_0)*2));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE, Event);
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_atForceEvent
 *
 *   @b Description
 *   @n Force set Eight Rad and Six DIO Events
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_AtEventIndex         Event index to select event 
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_AT_EVT_FORCE_EVENTFORCE;AIF2_AT_INTERNAL_EVT_FORCE_ADINGR_EVENT_FORCE;AIF2_AT_INTERNAL_EVT_FORCE_ADEGR_EVENT_FORCE
 *        
 *   @b Example
 *   @verbatim
        Aif2Fl_AtEventIndex    event = 1; //Rad event 1
        
        Aif2Fl_atForceEvent(hAif2, event);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_atForceEvent(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_AtEventIndex              event_index
)
{
  uint32_t  Event = 0;
  
   if(event_index < AIF2FL_IN_DIO_EVENT_0)//Rad timer event set
   {
      Event = (uint32_t)(0x01 << event_index);
      CSL_FINS(hAif2->regs->AT_EVT_FORCE, AIF2_AT_EVT_FORCE_EVENTFORCE, Event);
   }
   else if(event_index< AIF2FL_E_DIO_EVENT_0)//Ingress DIO event set
   {
      Event = (uint32_t)(0x03 << ((event_index-AIF2FL_IN_DIO_EVENT_0)*2));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_FORCE, AIF2_AT_INTERNAL_EVT_FORCE_ADINGR_EVENT_FORCE, Event);
   }
   else//Egress DIO event set
   {
      Event = (uint32_t)(0x03 << ((event_index-AIF2FL_E_DIO_EVENT_0)*2));
      CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_FORCE, AIF2_AT_INTERNAL_EVT_FORCE_ADEGR_EVENT_FORCE, Event);
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eeEoiSetup
 *
 *   @b Description
 *   @n EE End of interrupt vector value setup
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            uint8_t        End of Interrupt value
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_VB_EOI_EOI_VECTOR
 *        
 *   @b Example
 *   @verbatim
        uint8_t    eoi = 0x1; 
        
        Aif2Fl_eeEoiSetup(hAif2, eoi);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeEoiSetup(
    Aif2Fl_Handle    hAif2,
    uint8_t             EOI
)
{
   CSL_FINS(hAif2->regs->EE_VB_EOI, AIF2_EE_VB_EOI_EOI_VECTOR, EOI);
}

/** ============================================================================
 *   @n@b Aif2Fl_eeAif2ErrorIntSetup
 *
 *   @b Description
 *   @n EE VB error interrupt set or clear
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select between set and clear
            
            Aif2Fl_EeAif2Int       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_VB_INTR_SET_ERR_INTR_SET,AIF2_EE_VB_INTR_SET_ALARM_INTR_SET,AIF2_EE_VB_INTR_SET_CDMA_INTR_SET;
 *        AIF2_EE_VB_INTR_CLR_ERR_INTR_CLR,AIF2_EE_VB_INTR_CLR_ALARM_INTR_CLR,AIF2_EE_VB_INTR_CLR_CDMA_INTR_CLR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAif2Int    Aif2ErrInt; 
        Aif2ErrInt.Error_intr = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeAif2ErrorIntSetup(hAif2, Aif2ErrInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeAif2ErrorIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeAif2Int            Aif2ErrInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_VB_INTR_SET_ERR_INTR_SET, Aif2ErrInt.Error_intr)|
   CSL_FMK(AIF2_EE_VB_INTR_SET_ALARM_INTR_SET, Aif2ErrInt.Alarm_intr)|
   CSL_FMK(AIF2_EE_VB_INTR_SET_CDMA_INTR_SET, Aif2ErrInt.Cdma_intr);
   hAif2->regs->EE_VB_INTR_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_VB_INTR_CLR_ERR_INTR_CLR, Aif2ErrInt.Error_intr)|
   CSL_FMK(AIF2_EE_VB_INTR_CLR_ALARM_INTR_CLR, Aif2ErrInt.Alarm_intr)|
   CSL_FMK(AIF2_EE_VB_INTR_CLR_CDMA_INTR_CLR, Aif2ErrInt.Cdma_intr);
   hAif2->regs->EE_VB_INTR_CLR = tempReg;
   }
   
}

/** ============================================================================
 *   @n@b Aif2Fl_eeDbIntSetup
 *
 *   @b Description
 *   @n EE DB interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeDbInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_DB_IRS_SET_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_IRS_SET_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_IRS_SET_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_IRS_SET_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_IRS_SET_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_IRS_SET_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_IRS_SET_DB_EE_E_CD_DATA_TYPE_ERR;
 *        AIF2_EE_DB_IRS_CLR_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_IRS_CLR_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_IRS_CLR_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_IRS_CLR_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_IRS_CLR_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_IRS_CLR_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_IRS_CLR_DB_EE_E_CD_DATA_TYPE_ERR;
 *        AIF2_EE_DB_EN_SET_EV0_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_EN_SET_EV0_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_EN_SET_EV0_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_EN_SET_EV0_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_EN_SET_EV0_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_EN_SET_EV0_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_EN_SET_EV0_DB_EE_E_CD_DATA_TYPE_ERR;
 *        AIF2_EE_DB_EN_CRL_EV0_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_EN_CRL_EV0_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_EN_CRL_EV0_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_EN_CRL_EV0_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_EN_CRL_EV0_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_EN_CRL_EV0_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_EN_CRL_EV0_DB_EE_E_CD_DATA_TYPE_ERR;
 *        AIF2_EE_DB_EN_SET_EV1_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_EN_SET_EV1_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_EN_SET_EV1_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_EN_SET_EV1_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_EN_SET_EV1_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_EN_SET_EV1_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_EN_SET_EV1_DB_EE_E_CD_DATA_TYPE_ERR;
 *        AIF2_EE_DB_EN_CRL_EV1_DB_EE_I_TRC_RAM_OVFL_ERR,AIF2_EE_DB_EN_CRL_EV1_DB_EE_I_TOKEN_OVFL_ERR,
 *        AIF2_EE_DB_EN_CRL_EV1_DB_EE_I_FIFO_OVFL_ERR,AIF2_EE_DB_EN_CRL_EV1_DB_EE_I_PD2DB_FULL_ERR,
 *        AIF2_EE_DB_EN_CRL_EV1_DB_EE_E_PS_AXC_ERR,AIF2_EE_DB_EN_CRL_EV1_DB_EE_E_CD_DATA_ERR,
 *        AIF2_EE_DB_EN_CRL_EV1_DB_EE_E_CD_DATA_TYPE_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeDbInt    DbInt; 
        DbInt.db_ee_i_trc_ram_ovfl_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeDbIntSetup(hAif2, DbInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeDbIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeDbInt            DbInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_IRS_SET_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_IRS_CLR_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV0_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV0_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_EN_SET_EV1_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_I_TRC_RAM_OVFL_ERR, DbInt.db_ee_i_trc_ram_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_I_TOKEN_OVFL_ERR, DbInt.db_ee_i_token_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_I_FIFO_OVFL_ERR, DbInt.db_ee_i_fifo_ovfl_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_I_PD2DB_FULL_ERR, DbInt.db_ee_i_pd2db_full_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_E_PS_AXC_ERR, DbInt.db_ee_e_ps_axc_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_E_CD_DATA_ERR, DbInt.db_ee_e_cd_data_err)|
   CSL_FMK(AIF2_EE_DB_EN_CLR_EV1_DB_EE_E_CD_DATA_TYPE_ERR, DbInt.db_ee_e_cd_data_type_err);
   hAif2->regs->EE_DB_EN_CLR_EV1 = tempReg;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eeAdIntSetup
 *
 *   @b Description
 *   @n EE AD interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeAdInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_AD_IRS_SET_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_IRS_SET_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_2_ERR;
 *        AIF2_EE_AD_IRS_CLR_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_IRS_CLR_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_2_ERR;
 *
 *        AIF2_EE_AD_EN_SET_EV0_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_EN_SET_EV0_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_2_ERR;
 *        AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_2_ERR;
 *
 *        AIF2_EE_AD_EN_SET_EV1_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_EN_SET_EV1_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_2_ERR;
 *        AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_CD_DATA_ERR,AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_CD_SCH_ERR,
 *        AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_0_ERR,AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_1_ERR,
 *        AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_2_ERR,AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_0_ERR,
 *        AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_1_ERR,AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_2_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAdInt    AdInt; 
        AdInt.ad_ee_i_cd_data_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeAdIntSetup(hAif2, AdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeAdIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeAdInt            AdInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_AD_IRS_SET_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_IRS_CLR_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_AD_EN_SET_EV0_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV0_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_AD_EN_SET_EV1_AD_EE_I_CD_DATA_ERR, AdInt.ad_ee_i_cd_data_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_CD_SCH_ERR, AdInt.ad_ee_e_cd_sch_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_0_ERR, AdInt.ad_ee_i_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_1_ERR, AdInt.ad_ee_i_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_I_DMA_2_ERR, AdInt.ad_ee_i_dma_2_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_0_ERR, AdInt.ad_ee_e_dma_0_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_1_ERR, AdInt.ad_ee_e_dma_1_err)|
   CSL_FMK(AIF2_EE_AD_EN_CLR_EV1_AD_EE_E_DMA_2_ERR, AdInt.ad_ee_e_dma_2_err);
   hAif2->regs->EE_AD_EN_CLR_EV1 = tempReg;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eeCdIntSetup
 *
 *   @b Description
 *   @n EE CD(PKTDMA) interrupt set, clear, enable set or clear for EV0 
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeCdInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_CD_IRS_SET_CD_EE_SOP_DESC_STARVE_ERR,AIF2_EE_CD_IRS_SET_CD_EE_MOP_DESC_STARVE_ERR;
 *        AIF2_EE_CD_IRS_CLR_CD_EE_SOP_DESC_STARVE_ERR,AIF2_EE_CD_IRS_CLR_CD_EE_MOP_DESC_STARVE_ERR;
 *
 *        AIF2_EE_CD_EN_SET_EV_CD_EE_SOP_DESC_STARVE_ERR,AIF2_EE_CD_EN_SET_EV_CD_EE_MOP_DESC_STARVE_ERR;
 *        AIF2_EE_CD_EN_CLR_EV_CD_EE_SOP_DESC_STARVE_ERR,AIF2_EE_CD_EN_CLR_EV_CD_EE_MOP_DESC_STARVE_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeCdInt    CdInt; 
        CdInt.cd_ee_sop_desc_starve_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeCdIntSetup(hAif2, CdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeCdIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeCdInt            CdInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_CD_IRS_SET_CD_EE_SOP_DESC_STARVE_ERR, CdInt.cd_ee_sop_desc_starve_err)|
   CSL_FMK(AIF2_EE_CD_IRS_SET_CD_EE_MOP_DESC_STARVE_ERR, CdInt.cd_ee_mop_desc_starve_err);
   hAif2->regs->EE_CD_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_CD_IRS_CLR_CD_EE_SOP_DESC_STARVE_ERR, CdInt.cd_ee_sop_desc_starve_err)|
   CSL_FMK(AIF2_EE_CD_IRS_CLR_CD_EE_MOP_DESC_STARVE_ERR, CdInt.cd_ee_mop_desc_starve_err);
   hAif2->regs->EE_CD_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_CD_EN_SET_EV_CD_EE_SOP_DESC_STARVE_ERR, CdInt.cd_ee_sop_desc_starve_err)|
   CSL_FMK(AIF2_EE_CD_EN_SET_EV_CD_EE_MOP_DESC_STARVE_ERR, CdInt.cd_ee_mop_desc_starve_err);
   hAif2->regs->EE_CD_EN_SET_EV = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_CD_EN_CLR_EV_CD_EE_SOP_DESC_STARVE_ERR, CdInt.cd_ee_sop_desc_starve_err)|
   CSL_FMK(AIF2_EE_CD_EN_CLR_EV_CD_EE_MOP_DESC_STARVE_ERR, CdInt.cd_ee_mop_desc_starve_err);
   hAif2->regs->EE_CD_EN_CLR_EV = tempReg;
   }
   
}


/** ============================================================================
 *   @n@b Aif2Fl_eeSdIntSetup
 *
 *   @b Description
 *   @n EE SERDES interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeSdInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_SD_IRS_SET_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_IRS_SET_SD_EE_STSPLL_B8_ERR;
 *        AIF2_EE_SD_IRS_CLR_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_IRS_CLR_SD_EE_STSPLL_B8_ERR;
 *
 *        AIF2_EE_SD_EN_SET_EV0_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_EN_SET_EV0_SD_EE_STSPLL_B8_ERR;
 *        AIF2_EE_SD_EN_CLR_EV0_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_EN_CLR_EV0_SD_EE_STSPLL_B8_ERR;
 *        AIF2_EE_SD_EN_SET_EV1_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_EN_SET_EV1_SD_EE_STSPLL_B8_ERR;
 *        AIF2_EE_SD_EN_CLR_EV1_SD_EE_STSPLL_B4_ERR,AIF2_EE_SD_EN_CLR_EV1_SD_EE_STSPLL_B8_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeSdInt    SdInt; 
        SdInt.sd_ee_stspll_b4_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeSdIntSetup(hAif2, SdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeSdIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeSdInt           SdInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_SD_IRS_SET_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_IRS_SET_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_SD_IRS_CLR_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_IRS_CLR_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_SD_EN_SET_EV0_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_EN_SET_EV0_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_SD_EN_CLR_EV0_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_EN_CLR_EV0_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_SD_EN_SET_EV1_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_EN_SET_EV1_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_SD_EN_CLR_EV1_SD_EE_STSPLL_B4_ERR, SdInt.sd_ee_stspll_b4_err)|
   CSL_FMK(AIF2_EE_SD_EN_CLR_EV1_SD_EE_STSPLL_B8_ERR, SdInt.sd_ee_stspll_b8_err);
   hAif2->regs->EE_SD_EN_CLR_EV1 = tempReg;
   }
}

/** ============================================================================
 *   @n@b Aif2Fl_eeVcIntSetup
 *
 *   @b Description
 *   @n EE VC interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeVcInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_VC_IRS_SET_VC_EE_VBUS_ERR;AIF2_EE_VC_IRS_CLR_VC_EE_VBUS_ERR;
 *
 *        AIF2_EE_VC_EN_SET_EV0_VC_EE_VBUS_ERR;AIF2_EE_VC_EN_CLR_EV0_VC_EE_VBUS_ERR;
 *
 *        AIF2_EE_VC_EN_SET_EV1_VC_EE_VBUS_ERR;AIF2_EE_VC_EN_CLR_EV1_VC_EE_VBUS_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeVcInt    VcInt; 
        VcInt.vc_ee_vbus_err = true;
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        
        Aif2Fl_eeVcIntSetup(hAif2, VcInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeVcIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeVcInt            VcInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_VC_IRS_SET_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_VC_IRS_CLR_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_VC_EN_SET_EV0_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_VC_EN_CLR_EV0_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_VC_EN_SET_EV1_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_VC_EN_CLR_EV1_VC_EE_VBUS_ERR, VcInt.vc_ee_vbus_err);
   hAif2->regs->EE_VC_EN_CLR_EV1 = tempReg;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eeAif2RunSetup
 *
 *   @b Description
 *   @n EE Aif2 run control register setup
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  
            
            Aif2Fl_EeAif2Run         phy run and global run setup       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_AIF2_RUN_CTL_AIF2_PHY_RUN,AIF2_EE_AIF2_RUN_CTL_AIF2_GLOBAL_RUN;
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_EeVcInt    Aif2Run; 
        Aif2Run.aif2_phy_run = true;
        Aif2Run.aif2_global_run = true;
        
        Aif2Fl_eeAif2RunSetup(hAif2, Aif2Run);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeAif2RunSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeAif2Run            Aif2Run
)
{
   uint32_t tempReg;
   tempReg = CSL_FMK(AIF2_EE_AIF2_RUN_CTL_AIF2_PHY_RUN, Aif2Run.aif2_phy_run)|
   CSL_FMK(AIF2_EE_AIF2_RUN_CTL_AIF2_GLOBAL_RUN, Aif2Run.aif2_global_run);
   hAif2->regs->EE_AIF2_RUN_CTL = tempReg;
   
}


/** ============================================================================
 *   @n@b Aif2Fl_eeLinkAIntSetup
 *
 *   @b Description
 *   @n EE Link A interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function and hAif2->arg_link to select link
            
            Aif2Fl_EeLinkAInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_LK_IRS_SET_A_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_IRS_SET_A_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_IRS_SET_A_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_IRS_SET_A_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_IRS_SET_A_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_IRS_SET_A_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_LOS_ERR,                               AIF2_EE_LK_IRS_SET_A_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_IRS_SET_A_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_IRS_SET_A_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_IRS_SET_A_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_IRS_SET_A_TM_EE_FIFO_STARVE_ERR;
 *
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_IRS_CLR_A_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_IRS_CLR_A_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_IRS_CLR_A_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_IRS_CLR_A_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_IRS_CLR_A_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_LOS_ERR,                               AIF2_EE_LK_IRS_CLR_A_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_IRS_CLR_A_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_IRS_CLR_A_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_IRS_CLR_A_TM_EE_FIFO_STARVE_ERR;
 *
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_EN_A_SET_EV0_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_EN_A_SET_EV0_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_EN_A_SET_EV0_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_EN_A_SET_EV0_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOS_ERR,                               AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV0_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_EN_A_SET_EV0_TM_EE_FIFO_STARVE_ERR;
 *
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOS_ERR,                               AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV0_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_EN_A_CLR_EV0_TM_EE_FIFO_STARVE_ERR;
 *
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_EN_A_SET_EV1_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_EN_A_SET_EV1_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_EN_A_SET_EV1_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_EN_A_SET_EV1_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOS_ERR,                               AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_EN_A_SET_EV1_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_EN_A_SET_EV1_TM_EE_FIFO_STARVE_ERR;
 *
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR, AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_NUM_LOS_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LCV_DET_ERR,                       AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_FRAME_BNDRY_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_BLOCK_BNDRY_DET_ERR,       AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_MISSING_K28P5_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_MISSING_K28P7_ERR,             AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_K30P7_DET_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOC_DET_ERR,                       AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RX_FIFO_OVF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_LOS_ERR,                    AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_LOF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_RAI_ERR,                     AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_SDI_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOS_ERR,                               AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOF_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_HFNSYNC_STATE_ERR,            AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOF_STATE_ERR,
 *        AIF2_EE_LK_EN_A_CLR_EV1_TM_EE_FRM_MISALIGN_ERR,               AIF2_EE_LK_EN_A_CLR_EV1_TM_EE_FIFO_STARVE_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeLinkAInt    LinkAInt; 
        LinkAInt.rm_ee_sync_status_change_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        hAif2->arg_link = 0;//link 0
        Aif2Fl_eeLinkAIntSetup(hAif2, LinkAInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeLinkAIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeLinkAInt           LinkAInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_A_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_SET_A = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_A_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_CLR_A = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
    tempReg = CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV0_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV0_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
    tempReg = CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_SET_EV1_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_SYNC_STATUS_CHANGE_ERR, LinkAInt.rm_ee_sync_status_change_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_NUM_LOS_DET_ERR, LinkAInt.rm_ee_num_los_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LCV_DET_ERR, LinkAInt.rm_ee_lcv_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_FRAME_BNDRY_DET_ERR, LinkAInt.rm_ee_frame_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_BLOCK_BNDRY_DET_ERR, LinkAInt.rm_ee_block_bndry_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_MISSING_K28P5_ERR, LinkAInt.rm_ee_missing_k28p5_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_MISSING_K28P7_ERR, LinkAInt.rm_ee_missing_k28p7_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_K30P7_DET_ERR, LinkAInt.rm_ee_k30p7_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOC_DET_ERR, LinkAInt.rm_ee_loc_det_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RX_FIFO_OVF_ERR, LinkAInt.rm_ee_rx_fifo_ovf_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_LOS_ERR, LinkAInt.rm_ee_rcvd_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_LOF_ERR, LinkAInt.rm_ee_rcvd_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_RAI_ERR, LinkAInt.rm_ee_rcvd_rai_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_RCVD_SDI_ERR, LinkAInt.rm_ee_rcvd_sdi_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOS_ERR, LinkAInt.rm_ee_los_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOF_ERR, LinkAInt.rm_ee_lof_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_HFNSYNC_STATE_ERR, LinkAInt.rm_ee_hfnsync_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_RM_EE_LOF_STATE_ERR, LinkAInt.rm_ee_lof_state_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_TM_EE_FRM_MISALIGN_ERR, LinkAInt.tm_ee_frm_misalign_err)|
   CSL_FMK(AIF2_EE_LK_EN_A_CLR_EV1_TM_EE_FIFO_STARVE_ERR, LinkAInt.tm_ee_fifo_starve_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_A_CLR_EV1 = tempReg;
   }
  
}


/** ============================================================================
 *   @n@b Aif2Fl_eeLinkBIntSetup
 *
 *   @b Description
 *   @n EE Link B interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function and hAif2->arg_link to select link
            
            Aif2Fl_EeLinkBInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_LK_IRS_SET_B_PD_EE_EOP_ERR,                          AIF2_EE_LK_IRS_SET_B_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PD_EE_CPRI_FRAME_ERR,             AIF2_EE_LK_IRS_SET_B_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PD_EE_SOP_ERR,                          AIF2_EE_LK_IRS_SET_B_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PD_EE_WR2DB_ERR,                     AIF2_EE_LK_IRS_SET_B_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PE_EE_SYM_ERR,                           AIF2_EE_LK_IRS_SET_B_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_IRS_SET_B_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_IRS_SET_B_PE_EE_RT_IF_ERR,                        AIF2_EE_LK_IRS_SET_B_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_IRS_SET_B_RT_EE_FRM_ERR,                          AIF2_EE_LK_IRS_SET_B_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_IRS_SET_B_RT_EE_UNFL_ERR,                         AIF2_EE_LK_IRS_SET_B_RT_EE_EM_ERR,
 *        AIF2_EE_LK_IRS_SET_B_RT_EE_HDR_ERR;                           
 *
 *        AIF2_EE_LK_IRS_CLR_B_PD_EE_EOP_ERR,                           AIF2_EE_LK_IRS_CLR_B_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_IRS_CLR_B_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PD_EE_SOP_ERR,                          AIF2_EE_LK_IRS_CLR_B_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_IRS_CLR_B_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PE_EE_SYM_ERR,                           AIF2_EE_LK_IRS_CLR_B_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_IRS_CLR_B_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_IRS_CLR_B_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_RT_EE_FRM_ERR,                           AIF2_EE_LK_IRS_CLR_B_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_RT_EE_UNFL_ERR,                          AIF2_EE_LK_IRS_CLR_B_RT_EE_EM_ERR,
 *        AIF2_EE_LK_IRS_CLR_B_RT_EE_HDR_ERR;                           
 *
 *        AIF2_EE_LK_EN_B_SET_EV0_PD_EE_EOP_ERR,                           AIF2_EE_LK_EN_B_SET_EV0_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_B_SET_EV0_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PD_EE_SOP_ERR,                           AIF2_EE_LK_EN_B_SET_EV0_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_SET_EV0_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_B_SET_EV0_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_B_SET_EV0_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_B_SET_EV0_RT_EE_EM_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV0_RT_EE_HDR_ERR;                           
 *
 *        AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_EOP_ERR,                          AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_SOP_ERR,                          AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_WR2DB_ERR,                      AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_EM_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_HDR_ERR;                           
 *
 *        AIF2_EE_LK_EN_B_SET_EV1_PD_EE_EOP_ERR,                          AIF2_EE_LK_EN_B_SET_EV1_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_B_SET_EV1_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PD_EE_SOP_ERR,                          AIF2_EE_LK_EN_B_SET_EV1_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PD_EE_WR2DB_ERR,                     AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_SET_EV1_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_PE_EE_RT_IF_ERR,                         AIF2_EE_LK_EN_B_SET_EV1_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_RT_EE_FRM_ERR,                           AIF2_EE_LK_EN_B_SET_EV1_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_RT_EE_UNFL_ERR,                          AIF2_EE_LK_EN_B_SET_EV1_RT_EE_EM_ERR,
 *        AIF2_EE_LK_EN_B_SET_EV1_RT_EE_HDR_ERR;                           
 *
 *        AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_EOP_ERR,                          AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_CRC_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_CPRI_FRAME_ERR,              AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_AXC_FAIL_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_SOP_ERR,                          AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_OBSAI_FRM_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_WR2DB_ERR,                     AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MODRULE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_SYM_ERR,                           AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_DB_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_RT_IF_ERR,                        AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_PKT_STARVE_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_FRM_ERR,                          AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_OVFL_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_UNFL_ERR,                         AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_EM_ERR,
 *        AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_HDR_ERR;                           
 *   @b Example
 *   @verbatim
        Aif2Fl_EeLinkBInt    LinkBInt; 
        LinkBInt.rm_ee_sync_status_change_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        hAif2->arg_link = 0; //link 0
        Aif2Fl_eeLinkBIntSetup(hAif2, LinkBInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeLinkBIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeLinkBInt           LinkBInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_IRS_SET_B_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_SET_B = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_IRS_CLR_B_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_IRS_CLR_B = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV0_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV0_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_SET_EV1_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_EOP_ERR, LinkBInt.pd_ee_eop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_CRC_ERR, LinkBInt.pd_ee_crc_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_CPRI_FRAME_ERR, LinkBInt.pd_ee_cpri_frame_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_AXC_FAIL_ERR, LinkBInt.pd_ee_axc_fail_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_SOP_ERR, LinkBInt.pd_ee_sop_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_OBSAI_FRM_ERR, LinkBInt.pd_ee_obsai_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PD_EE_WR2DB_ERR, LinkBInt.pd_ee_wr2db_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MODRULE_ERR, LinkBInt.pe_ee_modrule_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_SYM_ERR, LinkBInt.pe_ee_sym_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MF_FIFO_OVERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_overflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_MF_FIFO_UNDERFLOW_ERR, LinkBInt.pe_ee_mf_fifo_underflow_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_DB_STARVE_ERR, LinkBInt.pe_ee_db_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_RT_IF_ERR, LinkBInt.pe_ee_rt_if_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_PE_EE_PKT_STARVE_ERR, LinkBInt.pe_ee_pkt_starve_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_FRM_ERR, LinkBInt.rt_ee_frm_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_OVFL_ERR, LinkBInt.rt_ee_ovfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_UNFL_ERR, LinkBInt.rt_ee_unfl_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_EM_ERR, LinkBInt.rt_ee_em_err)|
   CSL_FMK(AIF2_EE_LK_EN_B_CLR_EV1_RT_EE_HDR_ERR, LinkBInt.rt_ee_hdr_err);
   hAif2->regs->EE_LK[hAif2->arg_link].EE_LK_EN_B_CLR_EV1  = tempReg;
   }
  
}


/** ============================================================================
 *   @n@b Aif2Fl_eeAtIntSetup
 *
 *   @b Description
 *   @n EE AT interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EeAtInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_IRS_SET_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_IRS_SET_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_PI0_ERR,AIF2_EE_AT_IRS_SET_AT_EE_PI1_ERR,AIF2_EE_AT_IRS_SET_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_PI3_ERR,AIF2_EE_AT_IRS_SET_AT_EE_PI4_ERR,AIF2_EE_AT_IRS_SET_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_IRS_SET_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_IRS_SET_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_IRS_CLR_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_IRS_CLR_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_PI0_ERR,AIF2_EE_AT_IRS_CLR_AT_EE_PI1_ERR,AIF2_EE_AT_IRS_CLR_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_PI3_ERR,AIF2_EE_AT_IRS_CLR_AT_EE_PI4_ERR,AIF2_EE_AT_IRS_CLR_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_IRS_CLR_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_IRS_CLR_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_PI0_ERR,AIF2_EE_AT_EN_SET_EV0_AT_EE_PI1_ERR,AIF2_EE_AT_EN_SET_EV0_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_PI3_ERR,AIF2_EE_AT_EN_SET_EV0_AT_EE_PI4_ERR,AIF2_EE_AT_EN_SET_EV0_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_SET_EV0_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_SET_EV0_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI0_ERR,AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI1_ERR,AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI3_ERR,AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI4_ERR,AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_CLR_EV0_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_CLR_EV0_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_PI0_ERR,AIF2_EE_AT_EN_SET_EV1_AT_EE_PI1_ERR,AIF2_EE_AT_EN_SET_EV1_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_PI3_ERR,AIF2_EE_AT_EN_SET_EV1_AT_EE_PI4_ERR,AIF2_EE_AT_EN_SET_EV1_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_SET_EV1_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_SET_EV1_AT_EE_RADT_SYNC_ERR;
 *
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR,       AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR,       AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_UNSEL_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_SPARE_ERR,              AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_RSVD_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_BIT_WIDTH_ERR,                AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_CRC_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_RP3_ERR,                           AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_SYS_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI0_ERR,AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI1_ERR,AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI2_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI3_ERR,AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI4_ERR,AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI5_ERR,
 *         AIF2_EE_AT_EN_CLR_EV1_AT_EE_PHYT_SYNC_ERR,                      AIF2_EE_AT_EN_CLR_EV1_AT_EE_RADT_SYNC_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EeAtInt    AtInt; 
        AtInt.ad_ee_i_cd_data_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eeAtIntSetup(hAif2, AtInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eeAtIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EeAtInt            AtInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_IRS_SET_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_IRS_CLR_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV0_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV0_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_EN_SET_EV1_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_SYS_RCVD_ERR, AtInt.at_ee_rp1_type_sys_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_RP3_RCVD_ERR, AtInt.at_ee_rp1_type_rp3_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_TOD_RCVD_ERR, AtInt.at_ee_rp1_type_tod_rcvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_UNSEL_ERR, AtInt.at_ee_rp1_type_unsel_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_SPARE_ERR, AtInt.at_ee_rp1_type_spare_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_TYPE_RSVD_ERR, AtInt.at_ee_rp1_type_rsvd_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_BIT_WIDTH_ERR, AtInt.at_ee_rp1_bit_width_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_CRC_ERR, AtInt.at_ee_rp1_crc_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_RP3_ERR, AtInt.at_ee_rp1_rp3_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RP1_SYS_ERR, AtInt.at_ee_rp1_sys_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI0_ERR, AtInt.at_ee_pi0_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI1_ERR, AtInt.at_ee_pi1_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI2_ERR, AtInt.at_ee_pi2_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI3_ERR, AtInt.at_ee_pi3_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI4_ERR, AtInt.at_ee_pi4_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PI5_ERR, AtInt.at_ee_pi5_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_PHYT_SYNC_ERR, AtInt.at_ee_phyt_sync_err)|
   CSL_FMK(AIF2_EE_AT_EN_CLR_EV1_AT_EE_RADT_SYNC_ERR, AtInt.at_ee_radt_sync_err);
   hAif2->regs->EE_AT_EN_CLR_EV1 = tempReg;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eePdIntSetup
 *
 *   @b Description
 *   @n EE PD interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EePdInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_PD_COMMON_IRS_SET_PD_EE_TS_WDOG_ERR;AIF2_EE_PD_COMMON_IRS_CLR_PD_EE_TS_WDOG_ERR;
 *        AIF2_EE_PD_COMMON_EN_SET_EV0_PD_EE_TS_WDOG_ERR;AIF2_EE_PD_COMMON_EN_CLR_EV0_PD_EE_TS_WDOG_ERR;
 *        AIF2_EE_PD_COMMON_EN_SET_EV1_PD_EE_TS_WDOG_ERR;AIF2_EE_PD_COMMON_EN_CLR_EV1_PD_EE_TS_WDOG_ERR
 *   @b Example
 *   @verbatim
        Aif2Fl_EePdInt    PdInt; 
        PdInt.pd_ee_ts_wdog_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eePdIntSetup(hAif2, PdInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eePdIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EePdInt            PdInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_IRS_SET_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_IRS_CLR_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_EN_SET_EV0_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_EN_CLR_EV0_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_EN_SET_EV1_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
   tempReg = CSL_FMK(AIF2_EE_PD_COMMON_EN_CLR_EV1_PD_EE_TS_WDOG_ERR, PdInt.pd_ee_ts_wdog_err);
   hAif2->regs->EE_PD_COMMON_EN_CLR_EV1 = tempReg;
   }
}


/** ============================================================================
 *   @n@b Aif2Fl_eePeIntSetup
 *
 *   @b Description
 *   @n EE PE interrupt set, clear, enable set or clear for EV0 and EV1
 *
 *   @b Arguments
 *   @verbatim
            hAif2    Handle to the aif2 instance.  use hAif2->ee_arg to select function
            
            Aif2Fl_EePeInt       
     @endverbatim
 *
 *   <b> Return Value </b>  None
 *   <b> Pre Condition </b>
*   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n  None
 *
 *   @b Writes
 *   @n AIF2_EE_PE_COMMON_IRS_SET_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_IRS_SET_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_IRS_SET_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_IRS_SET_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_DAT_REQ_OVFL_ERR;
 *        AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_RD2DB_ERR,AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_TOKEN_REQ_OVFL_ERR,
 *        AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_TOKEN_WR_ERR,AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_DAT_REQ_OVFL_ERR;
 *   @b Example
 *   @verbatim
        Aif2Fl_EePeInt    PeInt; 
        PeInt.pe_ee_rd2db_err = true;
        .........
        hAif2->ee_arg = AIF2FL_EE_INT_SET;
        Aif2Fl_eePeIntSetup(hAif2, PeInt);
     @endverbatim
 * ===========================================================================
 */
static inline
void Aif2Fl_eePeIntSetup(
    Aif2Fl_Handle    hAif2,
    Aif2Fl_EePeInt            PeInt
)
{
   uint32_t tempReg;
   if(hAif2->ee_arg == AIF2FL_EE_INT_SET){
   tempReg = CSL_FMK(AIF2_EE_PE_COMMON_IRS_SET_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_SET_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_SET_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_SET_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_IRS_SET = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_CLR){
  tempReg = CSL_FMK(AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_IRS_CLR_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_IRS_CLR = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV0){
   tempReg = CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
   	             CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV0_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_EN_SET_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV0){
    tempReg = CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV0_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_EN_CLR_EV0 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_SET_EV1){
    tempReg = CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_SET_EV1_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_EN_SET_EV1 = tempReg;
   }
   else if(hAif2->ee_arg == AIF2FL_EE_INT_EN_CLR_EV1){
    tempReg = CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_RD2DB_ERR, PeInt.pe_ee_rd2db_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_TOKEN_REQ_OVFL_ERR, PeInt.pe_ee_token_req_ovfl_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_TOKEN_WR_ERR, PeInt.pe_ee_token_wr_err)|
                    CSL_FMK(AIF2_EE_PE_COMMON_EN_CLR_EV1_PE_EE_DAT_REQ_OVFL_ERR, PeInt.pe_ee_dat_req_ovfl_err);
   hAif2->regs->EE_PE_COMMON_EN_CLR_EV1 = tempReg;
   }
}


#ifdef __cplusplus
}
#endif

#endif /* Aif2Fl_hwControlAUX_H_ */


