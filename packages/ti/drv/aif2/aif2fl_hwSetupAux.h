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
 *   @file  aif2fl_hwSetupAux.h
 *
 *   @brief  API Auxilary header file for Antenna Interface 2 HW setup.
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

#ifndef _AIF2FLHWSETUPAUX_H_
#define _AIF2FLHWSETUPAUX_H_
 
#include <ti/drv/aif2/aif2fl.h>
#include <ti/drv/aif2/aif2fl_hwControlAux.h>

#ifdef __cplusplus
extern "C" {
#endif
 
/**
 *  Hardware Setup Functions of Antenna Interface 2
 */

/** ============================================================================
 *   @n@b Aif2Fl_setupGlobalRegs
 *
 *   @b Description
 *   @n AIF2 Global setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_RM_CFG_SHORT_FRM_EN, AIF2_PD_GLOBAL_SHRT_FRM_MODE,AIF2_PE_GLOBAL_SHRT_FRM_MODE,
 *      AIF2_TM_FRM_MODE_FRM_MODE
 *   @b Example
 *   @verbatim
        Aif2Fl_setupGlobalRegs (aif2Setup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupGlobalRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_GlobalSetup *globalSetup
)
{
    int i;
    /* Select the frame mode long frame  or short frame for debug */
    CSL_FINS(hAif2->regs->RM_CFG, AIF2_RM_CFG_SHORT_FRM_EN, globalSetup->frameMode);
    for(i=0; i<6; i++)
    CSL_FINS(hAif2->regs->G_TM_LKS[i].TM_FRM_MODE, AIF2_TM_FRM_MODE_FRM_MODE, globalSetup->frameMode);
	
    CSL_FINS(hAif2->regs->PD_GLOBAL, AIF2_PD_GLOBAL_SHRT_FRM_MODE, globalSetup->frameMode);
	
    CSL_FINS(hAif2->regs->PE_GLOBAL, AIF2_PE_GLOBAL_SHRT_FRM_MODE, globalSetup->frameMode);


    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupSdLinkRegs
 *
 *   @b Description
 *   @n AIF2 SERDES Link setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2 link
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n  AIF2_SD_RX_R1_CFG_RXRATE,AIF2_SD_RX_CFG_RXDISABLE_COMMA_DETECT_AND_BYTE_ALIGNMENT,
 *         AIF2_SD_RX_CFG_RXLOSS_OF_SIGNAL_DETECT_ENABLE,AIF2_SD_RX_CFG_RXPOLARITY 
 *   @b Example
 *   @verbatim
        Aif2Fl_setupSdLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupSdLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
#ifdef K2
    /** pointer to Serdes link setup */
    Aif2Fl_SdLinkSetup          *pSdLinkConfig;

    uint32_t                      tempReg = 0;
    Aif2Fl_LinkIndex    linkIndex;
    
    pSdLinkConfig  = linkSetup->pSdLinkSetup;
    linkIndex = linkSetup->linkIndex;

    /* SD Rx Configuration setup*/
    tempReg = (uint32_t) CSL_FEXT(hAif2->regs->SD_LK[linkIndex].SD_RX_CFG, AIF2_SD_RX_CFG_RXRATE);
    /* AIF2_SD_RX_CFG_RXRATE field is in bit 5 and 6. it means this two bit values should be shifted 
     * 5bits to the left before doing additional operation
     */
    tempReg <<= CSL_AIF2_SD_RX_CFG_RXRATE_SHIFT;
    tempReg |= CSL_FMK(AIF2_SD_RX_CFG_RXDISABLE_COMMA_DETECT_AND_BYTE_ALIGNMENT, pSdLinkConfig->rxAlign)    |
                  CSL_FMK(AIF2_SD_RX_CFG_RXLOSS_OF_SIGNAL_DETECT_ENABLE, pSdLinkConfig->rxLos)   |
                  CSL_FMK(AIF2_SD_RX_CFG_RXPOLARITY,   pSdLinkConfig->rxPolarity);
    hAif2->regs->SD_LK[linkIndex].SD_RX_CFG= tempReg;

#else
    /** pointer to Serdes link setup */
    Aif2Fl_SdLinkSetup          *pSdLinkConfig;

    Uint32                      tempReg = 0;
    Aif2Fl_LinkIndex    linkIndex;

    pSdLinkConfig  = linkSetup->pSdLinkSetup;
    linkIndex = linkSetup->linkIndex;

    /* SD Rx Configuration 1 setup*/
    tempReg = (Uint32) CSL_FEXT(hAif2->regs->SD_LK[linkIndex].SD_RX_R1_CFG, AIF2_SD_RX_R1_CFG_RXRATE);
    tempReg |= CSL_FMK(AIF2_SD_RX_R1_CFG_RXBYTE_ALIGNMENT, pSdLinkConfig->rxAlign)    |
                  CSL_FMK(AIF2_SD_RX_R1_CFG_RXLOSS_OF_SIGNAL, pSdLinkConfig->rxLos)   |
                  CSL_FMK(AIF2_SD_RX_R1_CFG_RXCLOCK_AND_DATA_RECOVER,   pSdLinkConfig->rxCdrAlgorithm);
    hAif2->regs->SD_LK[linkIndex].SD_RX_R1_CFG= tempReg;

    /* SD Rx Configuration 2 setup*/
    tempReg = CSL_FMK(AIF2_SD_RX_R2_CFG_RXINVERT_POLARITY, pSdLinkConfig->rxInvertPolarity)    |
                  CSL_FMK(AIF2_SD_RX_R2_CFG_RXTERMINATION, pSdLinkConfig->rxTermination)   |
                  CSL_FMK(AIF2_SD_RX_R2_CFG_RXEQUALIZER,   pSdLinkConfig->rxEqualizerConfig) |
                  CSL_FMK(AIF2_SD_RX_R2_CFG_RXEQUALIZER_HOLD, pSdLinkConfig->bRxEqHold)   |
                  CSL_FMK(AIF2_SD_RX_R2_CFG_RXENABLE_OFFSET_COMPENSATION,   pSdLinkConfig->bRxOffsetComp);
    hAif2->regs->SD_LK[linkIndex].SD_RX_R2_CFG= tempReg;

    /* SD Tx Configuration 1 setup*/
    tempReg = (Uint32) CSL_FEXT(hAif2->regs->SD_LK[linkIndex].SD_TX_R1_CFG, AIF2_SD_TX_R1_CFG_TXRATE);
    tempReg |= CSL_FMK(AIF2_SD_TX_R1_CFG_TXSYNCHRONIZATION_MASTER, pSdLinkConfig->bEnableTxSyncMater);
    hAif2->regs->SD_LK[linkIndex].SD_TX_R1_CFG= tempReg;

    /* SD Tx Configuration 2 setup*/
    tempReg = CSL_FMK(AIF2_SD_TX_R2_CFG_TXINVERT_POLARITY, pSdLinkConfig->txInvertPolarity)    |
                  CSL_FMK(AIF2_SD_TX_R2_CFG_TXOUTPUT_SWING, pSdLinkConfig->txOutputSwing)   |
                  CSL_FMK(AIF2_SD_TX_R2_CFG_TXPRECURSOR_TAP_WEIGHT,   pSdLinkConfig->txPrecursorTapWeight) |
                  CSL_FMK(AIF2_SD_TX_R2_CFG_ADJACENTPOST_CURSOR_TAP_WEIGHT, pSdLinkConfig->txPostcursorTapWeight)   |
                  CSL_FMK(AIF2_SD_TX_R2_CFG_TXCURSOR_FIR,   pSdLinkConfig->bTxFirFilterUpdate);
    hAif2->regs->SD_LK[linkIndex].SD_TX_R2_CFG= tempReg;

#endif

    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupLinkRegs
 *
 *   @b Description
 *   @n AIF2 Link common setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_CFG_OBSAI_CPRI, AIF2_RM_LK_CFG0_MODE_SEL,AIF2_RT_LK_CFG_OBSAI_CPRI,AIF2_PD_LINK_A_OBSAI_CPRI,
 *      AIF2_PE_LINK_OBSAI_CPRI,AIF2_CI_LK_CFG_OBSAI_CPRI,AIF2_CO_LK_CFG_OBSAI_CPRI,AIF2_SD_RX_CFG_RXRATE,
 *      AIF2_TM_LK_CFG_LINKRATE,AIF2_RM_LK_CFG0_LINK_RATE,AIF2_RT_LK_CFG_LINK_RATE,
 *      AIF2_PD_LINK_B_CPRI_LK_RATE,AIF2_PE_LINK_LK_RATE,AIF2_CI_LK_CFG_LINK_RATE,AIF2_CO_LK_CFG_LINK_RATE,
 *      AIF2_RT_LK_CFG_SAMPLE_WIDTH,AIF2_PD_LINK_B_CPRI_AXC_PACK,AIF2_CI_LK_CFG_SAMPLE_WIDTH,AIF2_CO_LK_CFG_SAMPLE_WIDTH
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_setupLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
#ifdef K2

	/** pointer to common link setup */
    Aif2Fl_CommonLinkSetup      *pComLinkConfig;

    uint8_t                       linkIndex, SdLinkRate = 0x00;
    
    pComLinkConfig  = linkSetup->pComLinkSetup;
    linkIndex          = linkSetup->linkIndex;
    
    /* Select the opertaing mode OBSAI or CPRI */
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CFG, AIF2_TM_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_MODE_SEL, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_A, AIF2_PD_LINK_A_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
        
    /** Configuring the link Rate */
	
    if(pComLinkConfig->linkRate == AIF2FL_LINK_RATE_8x) SdLinkRate = 0x00; //SD Full rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_4x) SdLinkRate = 0x01; //SD Half rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_2x) SdLinkRate = 0x02; //SD Quad rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_5x) SdLinkRate = 0x01; //SD Half rate
	
    CSL_FINS(hAif2->regs->SD_LK[linkIndex].SD_RX_CFG, AIF2_SD_RX_CFG_RXRATE, SdLinkRate);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CFG, AIF2_TM_LK_CFG_LINKRATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_LK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_LK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);


    /** Configuring the sample width */
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->EgrDataWidth);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_AXC_PACK, pComLinkConfig->IngrDataWidth);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->IngrDataWidth);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->EgrDataWidth);

#else
    
    /** pointer to common link setup */
    Aif2Fl_CommonLinkSetup      *pComLinkConfig;

    uint8_t                       linkIndex, SdLinkRate;

    pComLinkConfig  = linkSetup->pComLinkSetup;
    linkIndex          = linkSetup->linkIndex;

    /* Select the opertaing mode OBSAI or CPRI */
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CFG, AIF2_TM_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_MODE_SEL, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_A, AIF2_PD_LINK_A_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_OBSAI_CPRI, pComLinkConfig->linkProtocol);

    /** Configuring the link Rate */

    if(pComLinkConfig->linkRate == AIF2FL_LINK_RATE_8x) SdLinkRate = 0x01; //SD Half rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_4x) SdLinkRate = 0x02; //SD Quarter rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_2x) SdLinkRate = 0x03; //SD Eighth rate
    else if (pComLinkConfig->linkRate == AIF2FL_LINK_RATE_5x) SdLinkRate = 0x02; //SD Quarter rate

    CSL_FINS(hAif2->regs->SD_LK[linkIndex].SD_RX_R1_CFG, AIF2_SD_RX_R1_CFG_RXRATE, SdLinkRate);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CFG, AIF2_TM_LK_CFG_LINKRATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_LK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_LK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_LINK_RATE, pComLinkConfig->linkRate);


    /** Configuring the sample width */
    CSL_FINS(hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG, AIF2_RT_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->EgrDataWidth);
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_AXC_PACK, pComLinkConfig->IngrDataWidth);
    CSL_FINS(hAif2->regs->G_CI_LKS[linkIndex].CI_LK_CFG, AIF2_CI_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->IngrDataWidth);
    CSL_FINS(hAif2->regs->G_CO_LKS[linkIndex].CO_LK_CFG, AIF2_CO_LK_CFG_SAMPLE_WIDTH, pComLinkConfig->EgrDataWidth);

#endif
    return AIF2FL_SOK;
    
}


/** ============================================================================
 *   @n@b Aif2Fl_setupRmLinkRegs
 *
 *   @b Description
 *   @n AIF2 Rx MAC setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_RM_LK_CFG0_MODE_SEL,AIF2_RM_LK_CFG0_LINK_RATE,AIF2_RM_LK_CFG0_FIFO_THOLD,AIF2_RM_LK_CFG0_ERROR_SUPPRESS,
 *      AIF2_RM_LK_CFG0_SD_AUTO_ALIGN_EN,AIF2_RM_LK_CFG0_SCR_EN,AIF2_RM_LK_CFG0_LCV_UNSYNC_EN,AIF2_RM_LK_CFG0_LCV_CNTR_EN,
 *      AIF2_RM_LK_CFG1_WD_WRAP,AIF2_RM_LK_CFG1_WD_EN,AIF2_RM_LK_CFG1_CQ_EN,AIF2_RM_LK_CFG1_MON_WRAP,
 *      AIF2_RM_LK_CFG2_LOS_DET_THOLD,AIF2_RM_LK_CFG3_SYNC_T,AIF2_RM_LK_CFG3_FRAME_SYNC_T,
 *      AIF2_RM_LK_CFG4_UNSYNC_T,AIF2_RM_LK_CFG4_FRAME_UNSYNC_T,AIF2_RM_LK_CFG0_RX_EN,
 *   @b Example
 *   @verbatim
        Aif2Fl_setupRmLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupRmLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
    /* pointer to Rx MAC setup */
    Aif2Fl_RmLinkSetup   *pRmLinkConfig;
    uint32_t              tempReg, temp = 0;
    uint8_t               linkIndex;

    pRmLinkConfig = linkSetup->pRmLinkSetup;
    linkIndex    = linkSetup->linkIndex;

     /* RM Link Configuration 0 setup*/
    tempReg = (uint32_t) CSL_FEXT(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_MODE_SEL);
    temp = (uint32_t) CSL_FEXT(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_LINK_RATE);
    tempReg |= (uint32_t)(temp << 2);
    tempReg |= CSL_FMK(AIF2_RM_LK_CFG0_FIFO_THOLD, pRmLinkConfig->RmFifoThold)    |
                  CSL_FMK(AIF2_RM_LK_CFG0_ERROR_SUPPRESS, pRmLinkConfig->RmErrorSuppress)   |
                  CSL_FMK(AIF2_RM_LK_CFG0_SD_AUTO_ALIGN_EN,   pRmLinkConfig->bEnableSdAutoAlign) |
	           CSL_FMK(AIF2_RM_LK_CFG0_SCR_EN, pRmLinkConfig->bEnableScrambler)   |
                  CSL_FMK(AIF2_RM_LK_CFG0_LCV_UNSYNC_EN,   pRmLinkConfig->bEnableLcvUnsync) |
	           CSL_FMK(AIF2_RM_LK_CFG0_LCV_CNTR_EN,   pRmLinkConfig->bEnableLcvControl);
    hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0 = tempReg;

     /* RM Link Configuration 1 setup*/
    tempReg = CSL_FMK(AIF2_RM_LK_CFG1_WD_WRAP, pRmLinkConfig->WatchDogWrap)    |
                  CSL_FMK(AIF2_RM_LK_CFG1_WD_EN, pRmLinkConfig->bEnableWatchDog)   |
	           CSL_FMK(AIF2_RM_LK_CFG1_CQ_EN,   pRmLinkConfig->bEnableClockQuality) |
	           CSL_FMK(AIF2_RM_LK_CFG1_MON_WRAP,   pRmLinkConfig->ClockMonitorWrap);
    hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG1 = tempReg;
	
     /* RM Link Configuration 2 setup*/
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG2, AIF2_RM_LK_CFG2_LOS_DET_THOLD, pRmLinkConfig->losDetThreshold);

     /* RM Link Configuration 3 setup*/
    tempReg = CSL_FMK(AIF2_RM_LK_CFG3_SYNC_T, pRmLinkConfig->SyncThreshold) |
	           CSL_FMK(AIF2_RM_LK_CFG3_FRAME_SYNC_T,   pRmLinkConfig->FrameSyncThreshold);
    hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG3 = tempReg;

    /* RM Link Configuration 4 setup*/
    tempReg = CSL_FMK(AIF2_RM_LK_CFG4_UNSYNC_T, pRmLinkConfig->UnsyncThreshold) |
	           CSL_FMK(AIF2_RM_LK_CFG4_FRAME_UNSYNC_T,   pRmLinkConfig->FrameUnsyncThreshold);
    hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG4 = tempReg;
    
    /* RM Link enable setup*/
    CSL_FINS(hAif2->regs->G_RM_LKS[linkIndex].RM_LK_CFG0, AIF2_RM_LK_CFG0_RX_EN,pRmLinkConfig->bEnableRmLink);

    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupTmLinkRegs
 *
 *   @b Description
 *   @n AIF2 Tx MAC setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_TM_LK_CTRL_LOS_EN,AIF2_TM_LK_SCR_CTRL_SEED_VALUE,
 *      AIF2_TM_LK_SCR_CTRL_SCR_EN,
 *      AIF2_TM_LK_L1_EN_L1_INBAND_EN, AIF2_TM_LK_LOSERR_RM_LINK_LOSERR,AIF2_TM_LK_LOFERR_RM_LINK_LOFERR,
 *      AIF2_TM_LK_LOSRX_RM_LINK_LOSRX,AIF2_TM_LK_LOFRX_RM_LINK_LOFRX,AIF2_TM_LK_RAIRX_RM_LINK_RAIRX,
 *      AIF2_TM_LK_PTRP_PTR_P,AIF2_TM_LK_STRT_STARTUP,AIF2_TM_LK_PROT_PROT_VERS,AIF2_TM_LK_CFG_TM_EN
 *   @b Example
 *   @verbatim
        Aif2Fl_setupTmLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupTmLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
    /* pointer to Tx MAC setup */
    Aif2Fl_TmLinkSetup   *pTmLinkConfig;
    uint32_t              tempReg;
    uint8_t               linkIndex;
    
    pTmLinkConfig = linkSetup->pTmLinkSetup;
    linkIndex    = linkSetup->linkIndex;

    /* TM Link Control setup*/
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CTRL, AIF2_TM_LK_CTRL_LOS_EN, pTmLinkConfig->bEnableRmLos); 

     /* TM Link SCR control setup*/
    tempReg = CSL_FMK(AIF2_TM_LK_SCR_CTRL_SEED_VALUE, pTmLinkConfig->SeedValue) |
	           CSL_FMK(AIF2_TM_LK_SCR_CTRL_SCR_EN,   pTmLinkConfig->bEnableScrambler);
    hAif2->regs->G_TM_LKS[linkIndex].TM_LK_SCR_CTRL = tempReg;

    /* TM Link L1 inband  enable setup*/
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_L1_EN, AIF2_TM_LK_L1_EN_L1_INBAND_EN, pTmLinkConfig->pCpriTmSetup.L1InbandEn);

    /* TM Link L1 inband  LOS,LOF,RAI info link select  */
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_LOSERR, AIF2_TM_LK_LOSERR_RM_LINK_LOSERR, pTmLinkConfig->pCpriTmSetup.RmLinkLosError);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_LOFERR, AIF2_TM_LK_LOFERR_RM_LINK_LOFERR, pTmLinkConfig->pCpriTmSetup.RmLinkLofError);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_LOSRX, AIF2_TM_LK_LOSRX_RM_LINK_LOSRX, pTmLinkConfig->pCpriTmSetup.RmLinkLosRx);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_LOFRX, AIF2_TM_LK_LOFRX_RM_LINK_LOFRX, pTmLinkConfig->pCpriTmSetup.RmLinkLofRx);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_RAIRX, AIF2_TM_LK_RAIRX_RM_LINK_RAIRX, pTmLinkConfig->pCpriTmSetup.RmLinkRaiRx);

    /* TM CPRI pointer P, startup, version setup  */
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_PTRP, AIF2_TM_LK_PTRP_PTR_P, pTmLinkConfig->pCpriTmSetup.TxPointerP);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_STRT, AIF2_TM_LK_STRT_STARTUP, pTmLinkConfig->pCpriTmSetup.TxStartup);
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_PROT, AIF2_TM_LK_PROT_PROT_VERS, pTmLinkConfig->pCpriTmSetup.TxProtocolVer);
	
    /* TM Link enable setup*/
    CSL_FINS(hAif2->regs->G_TM_LKS[linkIndex].TM_LK_CFG, AIF2_TM_LK_CFG_TM_EN,pTmLinkConfig->bEnableTmLink);
    
    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupPdLinkRegs
 *
 *   @b Description
 *   @n AIF2 Protocol decoder link setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PD_LINK_A_OBSAI_CPRI,AIF2_PD_LINK_A_ETHNET_STRIP,AIF2_PD_LINK_A_CRC8_POLY,AIF2_PD_LINK_A_CRC8_SEED,AIF2_PD_LINK_B_CPRI_LK_RATE,
 *     AIF2_PD_LINK_B_CPRI_AXC_PACK,AIF2_PD_LINK_B_CPRI_NULLDELM,AIF2_PD_LINK_B_PKT_DELIM_CH0,AIF2_PD_LINK_B_PKT_DELIM_CH1,
 *     AIF2_PD_LINK_B_PKT_DELIM_CH2,AIF2_PD_LINK_B_PKT_DELIM_CH3,
 *     AIF2_PD_LK_PACK_CPRI_PRE_ENC_BITSWAP,AIF2_PD_LK_PACK_CPRI_POST_ENC_BITSWAP,
 *     AIF2_PD_PACK_MAP_PACK0_DMA_CH,AIF2_PD_PACK_MAP_PACK0_EN,
 *     AIF2_PD_PACK_MAP_PACK1_DMA_CH,AIF2_PD_PACK_MAP_PACK1_EN,
 *     AIF2_PD_PACK_MAP_PACK2_DMA_CH,AIF2_PD_PACK_MAP_PACK2_EN,
 *     AIF2_PD_PACK_MAP_PACK3_DMA_CH,AIF2_PD_PACK_MAP_PACK3_EN,
 *     AIF2_PD_CPRI_CRC_CRC0_TYPE,AIF2_PD_CPRI_CRC_CRC0_EN,AIF2_PD_CPRI_CRC_CRC1_TYPE,AIF2_PD_CPRI_CRC_CRC1_EN,
 *     AIF2_PD_CPRI_CRC_CRC2_TYPE,AIF2_PD_CPRI_CRC_CRC2_EN,AIF2_PD_CPRI_CRC_CRC3_TYPE,
 *     AIF2_PD_CPRI_CRC_CRC3_EN,
 *     AIF2_PD_DBM_DBM_X,AIF2_PD_DBM_DBM_XBUBBLE,AIF2_PD_DBM_DBM_1MULT,AIF2_PD_DBM_DBM_1SIZE,
 *     AIF2_PD_DBM_DBM_2SIZE,AIF2_PD_DBM_1MAP_DBM_1MAP,AIF2_PD_DBM_2MAP_DBM_2MAP,
 *     AIF2_PD_TYPE_LUT_TS_FORMAT,AIF2_PD_TYPE_LUT_CRC_TYPE,AIF2_PD_TYPE_LUT_CRC_EN,AIF2_PD_TYPE_LUT_OBSAI_PKT_EN,
 *     AIF2_PD_TYPE_LUT_ENET_STRIP,AIF2_PD_TYPE_LUT_CRC_HDR,AIF2_PD_CPRI_ID_LUT_CPRI_DMACHAN,
 *     AIF2_PD_CPRI_ID_LUT_CPRI_X_EN,AIF2_PD_CPRI_ID_LUT_CPRI_PKT_EN,
 *     AIF2_PD_CPRI_ID_LUT_CPRI_8WD_OFSET,AIF2_PD_CW_LUT_CW_CHAN,
 *     AIF2_PD_CW_LUT_CW_EN,AIF2_PD_LINK_A_LINK_EN,AIF2_PD_CW_LUT_HYPFM_EOP
 *   @b Example
 *   @verbatim
        Aif2Fl_setupPdLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupPdLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
    /** pointer to Protocol decoder link setup */
    Aif2Fl_PdLinkSetup          *pPdConfig;
    uint8_t                   linkIndex;
    uint32_t                  tempReg,temp =0, i;
    
    linkIndex           = linkSetup->linkIndex;
    pPdConfig           = linkSetup->pPdLinkSetup;
   
     /* PD Link register a setup*/
    temp = (uint32_t) CSL_FEXT(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_A, AIF2_PD_LINK_A_OBSAI_CPRI);
    tempReg = (uint32_t)(temp << 4);
    tempReg |= CSL_FMK(AIF2_PD_LINK_A_ETHNET_STRIP, pPdConfig->CpriEnetStrip)    |
                  CSL_FMK(AIF2_PD_LINK_A_CRC8_POLY, pPdConfig->Crc8Poly)   |
	           CSL_FMK(AIF2_PD_LINK_A_CRC8_SEED, pPdConfig->Crc8Seed);
    hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_A = tempReg;
    
    /* PD Link register b setup*/
    tempReg = (uint32_t) CSL_FEXT(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_LK_RATE);
    temp = (uint32_t) CSL_FEXT(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B, AIF2_PD_LINK_B_CPRI_AXC_PACK);
    tempReg |= (uint32_t)(temp << 2);
    tempReg |= CSL_FMK(AIF2_PD_LINK_B_CPRI_NULLDELM,   pPdConfig->CpriCwNullDelimitor) |
	           CSL_FMK(AIF2_PD_LINK_B_PKT_DELIM_CH0, pPdConfig->CpriCwPktDelimitor[0])   |
                  CSL_FMK(AIF2_PD_LINK_B_PKT_DELIM_CH1,   pPdConfig->CpriCwPktDelimitor[1]) |
	           CSL_FMK(AIF2_PD_LINK_B_PKT_DELIM_CH2,   pPdConfig->CpriCwPktDelimitor[2]) |
	           CSL_FMK(AIF2_PD_LINK_B_PKT_DELIM_CH3,   pPdConfig->CpriCwPktDelimitor[3]);
    hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_B = tempReg;

    /* PD CW link pack cpri register setup*/
#ifdef K2
    tempReg = CSL_FMK(AIF2_PD_LK_PACK_CPRI_PRE_ENC_BITSWAP, pPdConfig->PreEncBitmap)   |
                  CSL_FMK(AIF2_PD_LK_PACK_CPRI_POST_ENC_BITSWAP,   pPdConfig->PostEncBitmap);
    hAif2->regs->G_PD_LKS[linkIndex].PD_LK_PACK_CPRI = tempReg;
#endif

    /* PD pack map register setup*/
    tempReg = CSL_FMK(AIF2_PD_PACK_MAP_PACK0_DMA_CH, pPdConfig->PdPackDmaCh[0])    |
                  CSL_FMK(AIF2_PD_PACK_MAP_PACK0_EN, pPdConfig->bEnablePack[0])   |
                  CSL_FMK(AIF2_PD_PACK_MAP_PACK1_DMA_CH,   pPdConfig->PdPackDmaCh[1]) |
	           CSL_FMK(AIF2_PD_PACK_MAP_PACK1_EN, pPdConfig->bEnablePack[1])   |
                  CSL_FMK(AIF2_PD_PACK_MAP_PACK2_DMA_CH,   pPdConfig->PdPackDmaCh[2]) |
	           CSL_FMK(AIF2_PD_PACK_MAP_PACK2_EN,   pPdConfig->bEnablePack[2]) |
	           CSL_FMK(AIF2_PD_PACK_MAP_PACK3_DMA_CH,   pPdConfig->PdPackDmaCh[3]) |
                  CSL_FMK(AIF2_PD_PACK_MAP_PACK3_EN,   pPdConfig->bEnablePack[3]);
    hAif2->regs->G_PD_LKS[linkIndex].PD_PACK_MAP = tempReg;

    /* PD CPRI crc register setup*/
    tempReg = CSL_FMK(AIF2_PD_CPRI_CRC_CRC0_TYPE, pPdConfig->PdCpriCrcType[0])    |
                  CSL_FMK(AIF2_PD_CPRI_CRC_CRC0_EN, pPdConfig->bEnableCpriCrc[0])   |
                  CSL_FMK(AIF2_PD_CPRI_CRC_CRC1_TYPE,   pPdConfig->PdCpriCrcType[1]) |
	           CSL_FMK(AIF2_PD_CPRI_CRC_CRC1_EN, pPdConfig->bEnableCpriCrc[1])   |
                  CSL_FMK(AIF2_PD_CPRI_CRC_CRC2_TYPE,   pPdConfig->PdCpriCrcType[2]) |
	           CSL_FMK(AIF2_PD_CPRI_CRC_CRC2_EN,   pPdConfig->bEnableCpriCrc[2]) |
	           CSL_FMK(AIF2_PD_CPRI_CRC_CRC3_TYPE,   pPdConfig->PdCpriCrcType[3]) |
                  CSL_FMK(AIF2_PD_CPRI_CRC_CRC3_EN,   pPdConfig->bEnableCpriCrc[3]);
    hAif2->regs->G_PD_LKS[linkIndex].PD_CPRI_CRC = tempReg;
	
    /* PD Link Dual bit map FSM register setup*/
    tempReg = CSL_FMK(AIF2_PD_DBM_DBM_X, pPdConfig->PdCpriDualBitMap.DbmX)    |
                  CSL_FMK(AIF2_PD_DBM_DBM_XBUBBLE, pPdConfig->PdCpriDualBitMap.DbmXBubble)   |
                  CSL_FMK(AIF2_PD_DBM_DBM_1MULT,   pPdConfig->PdCpriDualBitMap.Dbm1Mult) |
	           CSL_FMK(AIF2_PD_DBM_DBM_1SIZE, pPdConfig->PdCpriDualBitMap.Dbm1Size)   |
                  CSL_FMK(AIF2_PD_DBM_DBM_2SIZE,   pPdConfig->PdCpriDualBitMap.Dbm2Size);
    hAif2->regs->G_PD_LKS[linkIndex].PD_DBM = tempReg;
	
    /* PD DBM 1 map setup*/
    for(i = 0; i < 4; i++)
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_DBM_1MAP[i], AIF2_PD_DBM_1MAP_DBM_1MAP, pPdConfig->PdCpriDualBitMap.Dbm1Map[i]); 

    /* PD DBM 2 map setup*/
    for(i = 0; i < 3; i++)
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_DBM_2MAP[i], AIF2_PD_DBM_2MAP_DBM_2MAP, pPdConfig->PdCpriDualBitMap.Dbm2Map[i]); 

    /* PD 32 type lut register setup*/
    for(i = 0; i < 32; i++)
   {
    tempReg = CSL_FMK(AIF2_PD_TYPE_LUT_TS_FORMAT, pPdConfig->PdTypeLut[i].ObsaiTsFormat)    |
                  CSL_FMK(AIF2_PD_TYPE_LUT_CRC_TYPE, pPdConfig->PdTypeLut[i].PdCrcType)   |
                  CSL_FMK(AIF2_PD_TYPE_LUT_CRC_EN,   pPdConfig->PdTypeLut[i].bEnableCrc) |
	           CSL_FMK(AIF2_PD_TYPE_LUT_OBSAI_PKT_EN, pPdConfig->PdTypeLut[i].PdObsaiMode)   |
                  CSL_FMK(AIF2_PD_TYPE_LUT_ENET_STRIP,   pPdConfig->PdTypeLut[i].bEnableEnetStrip)|
	           CSL_FMK(AIF2_PD_TYPE_LUT_CRC_HDR,   pPdConfig->PdTypeLut[i].bEnableCrcHeader);
    hAif2->regs->G_PD_LKS[linkIndex].PD_TYPE_LUT[i] = tempReg;
    }
 
    /* PD 128 CPRI ID lut register setup*/
    for(i = 0; i < 128; i++)
   {
    tempReg = CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_DMACHAN, pPdConfig->CpriDmaCh[i])    |
                  CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_X_EN, pPdConfig->bEnableCpriX[i])   |
                  CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_PKT_EN,   pPdConfig->bEnableCpriPkt[i]) |
	           CSL_FMK(AIF2_PD_CPRI_ID_LUT_CPRI_8WD_OFSET, pPdConfig->Cpri8WordOffset[i]);
    hAif2->regs->G_PD_LKS[linkIndex].PD_CPRI_ID_LUT[i] = tempReg;
    }

     /* PD 256 CPRI control word lut register setup*/
    for(i = 0; i < 256; i++)
   {
    tempReg = CSL_FMK(AIF2_PD_CW_LUT_CW_CHAN, pPdConfig->CpriCwChannel[i])    |
	           CSL_FMK(AIF2_PD_CW_LUT_CW_EN, pPdConfig->bEnableCpriCw[i])  |
	           CSL_FMK(AIF2_PD_CW_LUT_HYPFM_EOP, pPdConfig->bHyperFrameEop[i]);
    hAif2->regs->G_PD_LKS[linkIndex].PD_CW_LUT[i] = tempReg;
    }
	
    /* PE Link and Global enable setup*/
    CSL_FINS(hAif2->regs->G_PD_LKS[linkIndex].PD_LINK_A, AIF2_PD_LINK_A_LINK_EN, pPdConfig->bEnablePdLink);
    
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupPeLinkRegs
 *
 *   @b Description
 *   @n AIF2 PE setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_LINK_OBSAI_CPRI,AIF2_PE_LINK_LK_RATE,AIF2_PE_LINK_DIO_CPPI,AIF2_PE_LINK_TDD_AXC,AIF2_PE_LINK_GSM_COMPRESS,
 *      AIF2_PE_LINK_OBSAIBUBBLE_BW,AIF2_PE_LINK_MEMFETCH_DELAY,AIF2_PE_CRC_CRC8_POLY,AIF2_PE_CRC_CRC8_SEED,
 *      AIF2_PE_CPRI_BITSWAP_PRE_ENC_BITSWAP,AIF2_PE_CPRI_BITSWAP_POST_ENC_BITSWAP,
 *      AIF2_PE_CPRI_DBM_CPRI_DBM_X,AIF2_PE_CPRI_DBM_CPRI_DBM_XBUBBLE,AIF2_PE_CPRI_DBM_CPRI_DBM_1MULT,
 *     AIF2_PE_CPRI_DBM_CPRI_DBM_1SIZE,AIF2_PE_CPRI_DBM_CPRI_DBM_2SIZE,
 *     AIF2_PE_CPRIDBM_1MAP_CPRI_DBM_1MAP,AIF2_PE_CPRIDBM_2MAP_CPRI_DBM_2MAP,
 *     AIF2_PE_CPRI0_CPRI_AXC_PACK,AIF2_PE_CPRI0_CPRI_NULLDELM,AIF2_PE_CPRI0_PKT_DELIM_CH0,
 *    AIF2_PE_CPRI0_PKT_DELIM_CH1,AIF2_PE_CPRI0_PKT_DELIM_CH2,AIF2_PE_CPRI0_PKT_DELIM_CH3,
 *    AIF2_PE_CPRI1_CPRI_PKT0_CH,AIF2_PE_CPRI1_CPRI_PKT1_CH,AIF2_PE_CPRI1_CPRI_PKT2_CH,
 *    AIF2_PE_CPRI1_CPRI_PKT3_CH,AIF2_PE_CPRI1_CPRI_PKT0_CH_EN,AIF2_PE_CPRI1_CPRI_PKT1_CH_EN,
 *    AIF2_PE_CPRI1_CPRI_PKT2_CH_EN,AIF2_PE_CPRI1_CPRI_PKT3_CH_EN,AIF2_PE_CPRI_CW_LUT_CW_CHAN,AIF2_PE_CPRI_CW_LUT_CW_EN,
 *    AIF2_PE_LINK_LK_EN
 *   @b Example
 *   @verbatim
        Aif2Fl_setupPeLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupPeLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup    *linkSetup
)
{
    Aif2Fl_PeLinkSetup           *pPeConfig;
    uint32_t                   tempReg,temp = 0, i;    
    uint8_t                    linkIndex;

    pPeConfig = linkSetup->pPeLinkSetup;
    linkIndex = linkSetup->linkIndex;

     /* PE Link register setup*/
    tempReg = (uint32_t) CSL_FEXT(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_OBSAI_CPRI);
    tempReg = (uint32_t) (tempReg << 4);
    temp = (uint32_t) CSL_FEXT(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_LK_RATE);
    tempReg |= (uint32_t)(temp << 16);
    tempReg |= CSL_FMK(AIF2_PE_LINK_DIO_CPPI, pPeConfig->PeCppiDioSel)    |
	           CSL_FMK(AIF2_PE_LINK_TDD_AXC, pPeConfig->TddAxc)   |
	           CSL_FMK(AIF2_PE_LINK_GSM_COMPRESS, pPeConfig->bGsmCompress)   |
	           CSL_FMK(AIF2_PE_LINK_OBSAIBUBBLE_BW, pPeConfig->bEnObsaiBubbleBW)   |
                  CSL_FMK(AIF2_PE_LINK_MEMFETCH_DELAY,   pPeConfig->PeDelay);
    hAif2->regs->G_PE_LKS[linkIndex].PE_LINK = tempReg;
   

	/* PE Crc register setup*/
     tempReg = CSL_FMK(AIF2_PE_CRC_CRC8_POLY, pPeConfig->Crc8Poly)    |
                  CSL_FMK(AIF2_PE_CRC_CRC8_SEED,   pPeConfig->Crc8Seed);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CRC = tempReg;

#ifdef K2
    /* PE CPRI encoder bitmap register setup*/
    tempReg = CSL_FMK(AIF2_PE_CPRI_BITSWAP_PRE_ENC_BITSWAP, pPeConfig->PreEncBitmap)   |
                  CSL_FMK(AIF2_PE_CPRI_BITSWAP_POST_ENC_BITSWAP,   pPeConfig->PostEncBitmap);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CPRI_BITSWAP = tempReg;
#endif
    /* PE Link CPRI Dual bit map FSM register setup*/
    tempReg = CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_X, pPeConfig->PeCpriDualBitMap.DbmX)    |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_XBUBBLE, pPeConfig->PeCpriDualBitMap.DbmXBubble)   |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_1MULT,   pPeConfig->PeCpriDualBitMap.Dbm1Mult) |
	           CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_1SIZE, pPeConfig->PeCpriDualBitMap.Dbm1Size)   |
                  CSL_FMK(AIF2_PE_CPRI_DBM_CPRI_DBM_2SIZE,   pPeConfig->PeCpriDualBitMap.Dbm2Size);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CPRI_DBM = tempReg;
	
    /* PE CPRI DBM 1 map setup*/
    for(i = 0; i < 4; i++)
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_CPRIDBM_1MAP[i], AIF2_PE_CPRIDBM_1MAP_CPRI_DBM_1MAP, pPeConfig->PeCpriDualBitMap.Dbm1Map[i]); 
   
    /* PE CPRI DBM 2 map setup*/
    for(i = 0; i < 3; i++)
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_CPRIDBM_2MAP[i], AIF2_PE_CPRIDBM_2MAP_CPRI_DBM_2MAP, pPeConfig->PeCpriDualBitMap.Dbm2Map[i]); 
   
    /* PE CPRI link register1 setup*/
    tempReg = CSL_FMK(AIF2_PE_CPRI0_CPRI_AXC_PACK, pPeConfig->CpriAxCPack)    |
                  CSL_FMK(AIF2_PE_CPRI0_CPRI_NULLDELM, pPeConfig->CpriCwNullDelimitor)   |
                  CSL_FMK(AIF2_PE_CPRI0_PKT_DELIM_CH0,   pPeConfig->CpriCwPktDelimitor[0]) |
	           CSL_FMK(AIF2_PE_CPRI0_PKT_DELIM_CH1, pPeConfig->CpriCwPktDelimitor[1])   |
                  CSL_FMK(AIF2_PE_CPRI0_PKT_DELIM_CH2,   pPeConfig->CpriCwPktDelimitor[2]) |
	           CSL_FMK(AIF2_PE_CPRI0_PKT_DELIM_CH3,   pPeConfig->CpriCwPktDelimitor[3]);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CPRI0 = tempReg;

    /* PE CPRI link register2 setup*/
    tempReg = CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT0_CH, pPeConfig->PePackDmaCh[0])    |
                  CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT1_CH,   pPeConfig->PePackDmaCh[1])   |
                  CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT2_CH,   pPeConfig->PePackDmaCh[2]) |
	           CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT3_CH,   pPeConfig->PePackDmaCh[3])   |
	           CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT0_CH_EN,   pPeConfig->bEnablePack[0]) |
	           CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT1_CH_EN,   pPeConfig->bEnablePack[1]) |
                  CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT2_CH_EN,   pPeConfig->bEnablePack[2]) |
	           CSL_FMK(AIF2_PE_CPRI1_CPRI_PKT3_CH_EN,   pPeConfig->bEnablePack[3]);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CPRI1 = tempReg;

    /* PE 256 Control word LUT register setup*/
    for(i = 0; i < 256; i++) {
    tempReg = CSL_FMK(AIF2_PE_CPRI_CW_LUT_CW_CHAN, pPeConfig->CpriCwChannel[i])    |
                  CSL_FMK(AIF2_PE_CPRI_CW_LUT_CW_EN,   pPeConfig->bEnableCpriCw[i]);
    hAif2->regs->G_PE_LKS[linkIndex].PE_CPRI_CW_LUT[i] = tempReg;
    }
   
    /* Enable PE Link */
    CSL_FINS(hAif2->regs->G_PE_LKS[linkIndex].PE_LINK, AIF2_PE_LINK_LK_EN, pPeConfig->bEnablePeLink);
    
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupRtLinkRegs
 *
 *   @b Description
 *   @n AIF2 retransmitter setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_RT_LK_CFG_CI_SELECT,AIF2_RT_LK_CFG_EM_ENABLE,AIF2_RT_LK_CFG_RT_CONFIG
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_setupRtLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupRtLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
    uint8_t                   linkIndex;
    uint32_t                 tempReg = 0;
    Aif2Fl_RtLinkSetup  *pRtConfig;

    pRtConfig = linkSetup->pRtLinkSetup;
    linkIndex   = linkSetup->linkIndex;

    tempReg = (uint32_t) hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG;
    tempReg &= 0x0000001F;
    tempReg |= CSL_FMK(AIF2_RT_LK_CFG_CI_SELECT, pRtConfig->CiSelect)|
    CSL_FMK(AIF2_RT_LK_CFG_EM_ENABLE, pRtConfig->bEnableEmptyMsg)|
    CSL_FMK(AIF2_RT_LK_CFG_RT_CONFIG, pRtConfig->RtConfig);
    hAif2->regs->G_RT_LKS[linkIndex].RT_LK_CFG = tempReg;
	
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupAtLinkRegs
 *
 *   @b Description
 *   @n AIF2 Timer Link setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK0_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *      AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK1_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *      AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK2_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *      AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK3_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *      AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK4_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *      AIF2_AT_PIMAX_LK_PIMAX,AIF2_AT_PIMMIN_LK_PIMIN,AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO = 0, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO = 0,AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET,
 *      AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO = 0,AIF2_AT_NEG_DELTA_LK5_DELTA,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *      AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE;
 *
 *   @b Reads
 *   @n AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE,AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE
 *   @b Example
 *   @verbatim
        Aif2Fl_setupAtLinkRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupAtLinkRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_LinkSetup *linkSetup
)
{
    uint32_t                  tempReg;
    uint8_t                   linkIndex;
    Aif2Fl_AtLinkSetup  *pAtLinkConfig;

    pAtLinkConfig = linkSetup->pAtLinkSetup;
    linkIndex   = linkSetup->linkIndex;

     /* setup Pi max, min value */
    CSL_FINS(hAif2->regs->PI_DATA[linkIndex].AT_PIMAX_LK, AIF2_AT_PIMAX_LK_PIMAX, pAtLinkConfig->PiMax);
    CSL_FINS(hAif2->regs->PI_DATA[linkIndex].AT_PIMIN_LK, AIF2_AT_PIMIN_LK_PIMIN, pAtLinkConfig->PiMin);

    /* setup Delta event offset and modulus TC value */
    CSL_FINS(hAif2->regs->AT_TM_DELTA_EVENTS[linkIndex].AT_TM_DELTA_EVENT_OFFSET, AIF2_AT_TM_DELTA_EVENT_OFFSET_EVENTOFFSET, 
                   pAtLinkConfig->DeltaOffset);
    CSL_FINS(hAif2->regs->AT_TM_DELTA_EVENTS[linkIndex].AT_TM_DELTA_EVENT_MOD_TC, AIF2_AT_TM_DELTA_EVENT_MOD_TC_EVENTMODULO, 
		     3071999);

    //Enable Delta event
    tempReg = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE);
    tempReg |= (uint32_t)(0x1 << linkIndex);
    CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_TMDELTA_EVENT_ENABLE, tempReg);
	   
    /* setup PE event offset and modulus TC value for RT timing adjustment (precede 70 byte clock from Delta)*/
    CSL_FINS(hAif2->regs->AT_PE_EVENTS[linkIndex].AT_PE_EVENT_OFFSET, AIF2_AT_PE_EVENT_OFFSET_EVENTOFFSET, pAtLinkConfig->PE1Offset);
    CSL_FINS(hAif2->regs->AT_PE_EVENTS[linkIndex].AT_PE_EVENT_MOD_TC, AIF2_AT_PE_EVENT_MOD_TC_EVENTMODULO, 3071999);

    /* setup PE event2 offset and modulus TC value for PE frame preparation (precede 60 byte clock from Delta) */
    CSL_FINS(hAif2->regs->AT_PE_EVENT2S[linkIndex].AT_PE_EVENT2_OFFSET, AIF2_AT_PE_EVENT2_OFFSET_EVENTOFFSET, pAtLinkConfig->PE2Offset );
    CSL_FINS(hAif2->regs->AT_PE_EVENT2S[linkIndex].AT_PE_EVENT2_MOD_TC, AIF2_AT_PE_EVENT2_MOD_TC_EVENTMODULO, 3071999);

    //Enable PE event 
    tempReg = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE);
    tempReg |= (uint32_t)(0x1 << linkIndex);
    CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT_ENABLE, tempReg);

    //Enable PE event 2
    tempReg = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE);
    tempReg |= (uint32_t)(0x1 << linkIndex);
    CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_PEEVENT2_ENABLE, tempReg);

    /* select if delta is negative or positive value */
    switch(linkIndex){

       case 0 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK0_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       case 1 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK1_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       case 2 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK2_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       case 3 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK3_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       case 4 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK4_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       case 5 :
       CSL_FINS(hAif2->regs->AT_NEG_DELTA, AIF2_AT_NEG_DELTA_LK5_DELTA, pAtLinkConfig->IsNegativeDelta);
           break;
       default :
	    break;
    }
	
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupSdCommonRegs
 *
 *   @b Description
 *   @n AIF2 SERDES pll setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL,AIF2_SD_TX_CFG_TXRATE,
 *        AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL,AIF2_SD_CLK_SEL_CFG_SERDESCLKSEL,
 *        AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_0,AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_1,
 *        AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_2,AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_3,
 *        AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_4,AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_5
 *   @b Example
 *   @verbatim
        Aif2Fl_setupSdCommonRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupSdCommonRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup *commonSetup
)
{
#ifdef K2
	/** pointer to Serdes setup */
    Aif2Fl_SdCommonSetup          *pSerdesConfig;
    uint32_t                      tempReg;
    int                       linkIndex;
    
    pSerdesConfig  = commonSetup->pSdCommonSetup;
    
    for(linkIndex =0; linkIndex < 6;linkIndex++)//set SD TX link rate for all links
	CSL_FINS(hAif2->regs->SD_LK[linkIndex].SD_TX_CFG, AIF2_SD_TX_CFG_TXRATE, 0x00);//fixed to 8x rate
    
    if(pSerdesConfig->bEnablePllB8)
    {
               
	CSL_FINS(hAif2->regs->SD_PLL_B8_EN_CFG, AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL, pSerdesConfig->bEnablePllB8);
	 
	 tempReg = 0;
	 while(tempReg == 0){
      tempReg = CSL_FEXT(hAif2->regs->SD_PLL_B8_STS, AIF2_SD_PLL_B8_STS_B8PLL_LOCK);
	 }
    }

    if(pSerdesConfig->bEnablePllB4)
    {
               
	CSL_FINS(hAif2->regs->SD_PLL_B4_EN_CFG, AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL, pSerdesConfig->bEnablePllB4);
	
	 tempReg = 0;
	 while(tempReg == 0){
      tempReg = CSL_FEXT(hAif2->regs->SD_PLL_B4_STS, AIF2_SD_PLL_B4_STS_B4PLL_LOCK);
	 }
    }    

     /* The TX byte clock from either B8 or B4 SERDES link will be selected as sys_clk once the PLL has acquired lock.*/
    CSL_FINS(hAif2->regs->SD_CLK_SEL_CFG, AIF2_SD_CLK_SEL_CFG_SERDESCLKSEL, pSerdesConfig->SysClockSelect);

     /*  Select if link clock is gated off or on  each uint16_t array is matched with each link */
    tempReg = CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_0, pSerdesConfig->DisableLinkClock[0])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_1, pSerdesConfig->DisableLinkClock[1])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_2, pSerdesConfig->DisableLinkClock[2])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_3, pSerdesConfig->DisableLinkClock[3])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_4, pSerdesConfig->DisableLinkClock[4])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_5, pSerdesConfig->DisableLinkClock[5]);
    hAif2->regs->SD_LK_CLK_DIS_CFG = tempReg;

#else

    /** pointer to Serdes setup */
    Aif2Fl_SdCommonSetup          *pSerdesConfig;
    Uint32                      tempReg;
    int                       linkIndex;

    pSerdesConfig  = commonSetup->pSdCommonSetup;

    for(linkIndex =0; linkIndex < 6;linkIndex++)//set SD TX link rate for all links
	CSL_FINS(hAif2->regs->SD_LK[linkIndex].SD_TX_R1_CFG, AIF2_SD_TX_R1_CFG_TXRATE, 0x01);//fixed to 8x rate

    if(pSerdesConfig->bEnablePllB8)
    {
        /* SERDES B8 setup, links 0,1,2,3 */
	 tempReg = CSL_FMK(AIF2_SD_PLL_B8_CFG_B8PLL_MULTIPLY_FACTOR, pSerdesConfig->pllMpyFactorB8)|
                        CSL_FMK(AIF2_SD_PLL_B8_CFG_B8VOLTAGE_RANGE, pSerdesConfig->VoltRangeB8)|
	  	          CSL_FMK(AIF2_SD_PLL_B8_CFG_B8LOOP_BANDWIDTH, pSerdesConfig->LB_B8)|
                        CSL_FMK(AIF2_SD_PLL_B8_CFG_B8PLL_BYPASS, pSerdesConfig->CLKBYP_B8);
       hAif2->regs->SD_PLL_B8_CFG = tempReg;

	CSL_FINS(hAif2->regs->SD_PLL_B8_EN_CFG, AIF2_SD_PLL_B8_EN_CFG_ENABLEB8_PLL, pSerdesConfig->bEnablePllB8);

	 tempReg = 0;
	  while(tempReg == 0){
      tempReg = CSL_FEXT(hAif2->regs->SD_PLL_B8_STS, AIF2_SD_PLL_B8_STS_B8PLL_LOCK);
	  }
    }

    if(pSerdesConfig->bEnablePllB4)
    {
         /* SERDES B4 setup, links 0,1,2,3 */
	 tempReg =CSL_FMK(AIF2_SD_PLL_B4_CFG_B4PLL_MULTIPLY_FACTOR, pSerdesConfig->pllMpyFactorB4)|
                        CSL_FMK(AIF2_SD_PLL_B4_CFG_B4VOLTAGE_RANGE, pSerdesConfig->VoltRangeB4)|
	  	          CSL_FMK(AIF2_SD_PLL_B4_CFG_B4LOOP_BANDWIDTH, pSerdesConfig->LB_B4)|
                        CSL_FMK(AIF2_SD_PLL_B4_CFG_B4PLL_BYPASS, pSerdesConfig->CLKBYP_B4);
       hAif2->regs->SD_PLL_B4_CFG = tempReg;

	CSL_FINS(hAif2->regs->SD_PLL_B4_EN_CFG, AIF2_SD_PLL_B4_EN_CFG_ENABLEB4_PLL, pSerdesConfig->bEnablePllB4);
	
	 tempReg = 0;
	  while(tempReg == 0){
      tempReg = CSL_FEXT(hAif2->regs->SD_PLL_B4_STS, AIF2_SD_PLL_B4_STS_B4PLL_LOCK);
	  }
    }

     /* The TX byte clock from either B8 or B4 SERDES link 0 will be selected as sys_clk once the PLL has acquired lock.*/
    CSL_FINS(hAif2->regs->SD_CLK_SEL_CFG, AIF2_SD_CLK_SEL_CFG_ENABLERX, pSerdesConfig->SysClockSelect);

     /*  Select if link clock is gated off or on  each uint16_t array is matched with each link */
    tempReg = CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_0, pSerdesConfig->DisableLinkClock[0])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_1, pSerdesConfig->DisableLinkClock[1])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_2, pSerdesConfig->DisableLinkClock[2])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_3, pSerdesConfig->DisableLinkClock[3])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_4, pSerdesConfig->DisableLinkClock[4])|
    CSL_FMK(AIF2_SD_LK_CLK_DIS_CFG_DISABLELINK_CLOCK_5, pSerdesConfig->DisableLinkClock[5]);
    hAif2->regs->SD_LK_CLK_DIS_CFG = tempReg;

#endif
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupPdCommonRegs
 *
 *   @b Description
 *   @n AIF2 Protocol decoder common setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_PD_GLOBAL_EN_SET_DONT_CARE,AIF2_PD_GLOBAL_SHRT_FRM_MODE,
 *        AIF2_PD_GLOBAL_DIO_CPPI,AIF2_PD_GLOBAL_AXCOFFSET_WIN,AIF2_PD_DMA_TS_WDOG_CNT,AIF2_PD_DMA_WDOG_EOP_ADD,
 *        AIF2_PD_DMA_WDOG_EE_CTRL,AIF2_PD_RADT_TC_RADT_TC,AIF2_PD_FRM_TC_FRM_INDEX_TC,AIF2_PD_FRM_TC_FRM_INDEX_SC,
 *        AIF2_PD_FRM_TC_FRM_SYM_TC,AIF2_PD_ROUTE_ROUTE_TS,AIF2_PD_ROUTE_ROUTE_TYPE,
 *        AIF2_PD_ROUTE_ROUTE_ADR,AIF2_PD_ROUTE_ROUTE_LK,AIF2_PD_ROUTE_ROUTE_MASK,
 *        AIF2_PD_DMACHAN_CHAN_EN,AIF2_PD_DMACHAN_DATA_FORMAT,AIF2_PD_DMACHAN_A_AXC_OFFSET,
 *        AIF2_PD_DMACHAN_B_TS_WDOG_EN,AIF2_PD_DMACHAN_B_GSM_UL,AIF2_PD_DMACHAN_B_FRM_GRP,
 *	 AIF2_PD_DMACHAN_B_DIO_OFFSET,AIF2_PD_DMACHAN_B_TDD_EN,
 *        AIF2_PD_DMACHAN_C_TDD_EN,AIF2_PD_DMACHAN_D_TDD_EN,AIF2_PD_DMACHAN_E_TDD_EN,
 *        AIF2_PD_DMACHAN_F_TDD_EN,AIF2_PD_FRM_MSG_TC_FRME_MSG_TC
 *   @b Example
 *   @verbatim
        Aif2Fl_setupPdCommonRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupPdCommonRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup *commonSetup
)
{
    /** pointer to Protocol decoder link setup */
    Aif2Fl_PdCommonSetup          *pPdConfig;
    uint32_t      tempReg, i;
    pPdConfig  = commonSetup->pPdCommonSetup;

    /* PD global enable set  */
    CSL_FINS(hAif2->regs->DB_PD_GLOBAL_EN_SET, AIF2_DB_PD_GLOBAL_EN_SET_DONT_CARE, true);
	
    /* PD global setup */
    tempReg = (uint32_t) CSL_FEXT(hAif2->regs->PD_GLOBAL, AIF2_PD_GLOBAL_SHRT_FRM_MODE);
    tempReg |= CSL_FMK(AIF2_PD_GLOBAL_DIO_CPPI, pPdConfig->PdCppiDioSel)|
                     CSL_FMK(AIF2_PD_GLOBAL_AXCOFFSET_WIN, pPdConfig->AxCOffsetWin);
    hAif2->regs->PD_GLOBAL = tempReg;
	
    /* PD dma register setup */
    tempReg = CSL_FMK(AIF2_PD_DMA_TS_WDOG_CNT, pPdConfig->TsWatchDogCount)|
	              CSL_FMK(AIF2_PD_DMA_WDOG_EOP_ADD, pPdConfig->WatchDogEopAdd)|
                     CSL_FMK(AIF2_PD_DMA_WDOG_EE_CTRL, pPdConfig->WatchDogReport);
    hAif2->regs->PD_DMA  = tempReg;


   /* PD radt tc register setup */
    CSL_FINS(hAif2->regs->PD_RADT_TC, AIF2_PD_RADT_TC_RADT_TC, pPdConfig->PdRadtTC);

    /* PD frame terminal count setup */
    for(i = 0; i < 6; i++){
    tempReg = CSL_FMK(AIF2_PD_FRM_TC_FRM_INDEX_TC, pPdConfig->PdFrameTC[i].FrameIndexTc)|
  	              CSL_FMK(AIF2_PD_FRM_TC_FRM_INDEX_SC, pPdConfig->PdFrameTC[i].FrameIndexSc)|
                     CSL_FMK(AIF2_PD_FRM_TC_FRM_SYM_TC, pPdConfig->PdFrameTC[i].FrameSymbolTc);
    hAif2->regs->PD_FRM_TC[i] = tempReg;
    }

    /* PD 128 route register setup */
    for(i = 0; i < 128; i++){
    tempReg = CSL_FMK(AIF2_PD_ROUTE_ROUTE_TS, pPdConfig->PdRoute[i].RouteTs)|
  	              CSL_FMK(AIF2_PD_ROUTE_ROUTE_TYPE, pPdConfig->PdRoute[i].RouteType)|
                     CSL_FMK(AIF2_PD_ROUTE_ROUTE_ADR, pPdConfig->PdRoute[i].RouteAddr)|
	              CSL_FMK(AIF2_PD_ROUTE_ROUTE_LK, pPdConfig->PdRoute[i].RouteLink)|
                     CSL_FMK(AIF2_PD_ROUTE_ROUTE_MASK, pPdConfig->PdRoute[i].RouteMask);
    hAif2->regs->PD_ROUTE[i] = tempReg;
    }

    /* PD 128 DMA channel config register setup */
    for(i = 0; i < 128; i++){
    tempReg = CSL_FMK(AIF2_PD_DMACHAN_CHAN_EN, pPdConfig->PdChConfig[i].bChannelEn)|
                     CSL_FMK(AIF2_PD_DMACHAN_DATA_FORMAT, pPdConfig->PdChConfig[i].DataFormat);
    hAif2->regs->PD_DMACHAN[i] = tempReg;
    }

    /* PD 128 DMA channel config 0 (AxC offset) register setup */
    for(i = 0; i < 128; i++)
    CSL_FINS(hAif2->regs->PD_DMACHAN_A[i], AIF2_PD_DMACHAN_A_AXC_OFFSET, pPdConfig->AxCOffset[i]);

    /* PD 128 DMA channel config 1 register setup */
    for(i = 0; i < 128; i++){
    tempReg = CSL_FMK(AIF2_PD_DMACHAN_B_TS_WDOG_EN, pPdConfig->PdChConfig1[i].bTsWatchDogEn)|
  	              CSL_FMK(AIF2_PD_DMACHAN_B_GSM_UL, pPdConfig->PdChConfig1[i].DataFormat)|
                     CSL_FMK(AIF2_PD_DMACHAN_B_FRM_GRP, pPdConfig->PdChConfig1[i].FrameCounter)|
	              CSL_FMK(AIF2_PD_DMACHAN_B_DIO_OFFSET, pPdConfig->PdChConfig1[i].DioOffset)|
                     CSL_FMK(AIF2_PD_DMACHAN_B_TDD_EN, pPdConfig->PdChConfig1[i].TddEnable);
    hAif2->regs->PD_DMACHAN_B[i] = tempReg;
    }

     /* PD 128 DMA channel config 2 ~ 5 register setup */
    for(i = 0; i < 128; i++){
    CSL_FINS(hAif2->regs->PD_DMACHAN_C[i], AIF2_PD_DMACHAN_C_TDD_EN, pPdConfig->TddEnable1[i]);
    CSL_FINS(hAif2->regs->PD_DMACHAN_D[i], AIF2_PD_DMACHAN_D_TDD_EN, pPdConfig->TddEnable2[i]);
    CSL_FINS(hAif2->regs->PD_DMACHAN_E[i], AIF2_PD_DMACHAN_E_TDD_EN, pPdConfig->TddEnable3[i]);
    CSL_FINS(hAif2->regs->PD_DMACHAN_F[i], AIF2_PD_DMACHAN_F_TDD_EN, pPdConfig->TddEnable4[i]);
    }
	
    /* PD 128 frame message terminal count register setup */
    for(i = 0; i < 128; i++)
    CSL_FINS(hAif2->regs->PD_FRM_MSG_TC[i], AIF2_PD_FRM_MSG_TC_FRME_MSG_TC, pPdConfig->PdFrameMsgTc[i]);
	
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupPeCommonRegs
 *
 *   @b Description
 *   @n AIF2 PE common setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_PE_GLOBAL_EN_SET_DONT_CARE,AIF2_PE_GLOBAL_TOKEN_PHASE,AIF2_PE_GLOBAL_ENET_HDR_SEL,AIF2_PE_GLOBAL_DIO_LEN,AIF2_PE_FRM_TC_FRM_INDEX_TC,
 *        AIF2_PE_FRM_TC_FRM_INDEX_SC,AIF2_PE_FRM_TC_FRM_SYM_TC,AIF2_PE_DMACHAN_EN_CH_EN,
 *        AIF2_PE_DMA0CHAN_CRC_EN,AIF2_PE_DMA0CHAN_FRM_TC,AIF2_PE_DMA0CHAN_RT_CTL,
 *        AIF2_PE_DMA0CHAN_CRC_TYPE,AIF2_PE_DMA0CHAN_ETHERNET,AIF2_PE_DMA0CHAN_CRC_HDR,
 *        AIF2_PE_IN_FIFO_MF_WMARK,AIF2_PE_IN_FIFO_MF_FULL_LEV,AIF2_PE_IN_FIFO_SYNC_SYM,AIF2_PE_AXC_OFFSET_AXC_OFFSET,
 *        AIF2_PE_FRM_MSG_TC_FRME_MSG_TC,AIF2_PE_MODTXRULE_RULE_MOD,AIF2_PE_MODTXRULE_RULE_INDEX,AIF2_PE_MODTXRULE_RULE_LK,
 *        AIF2_PE_MODTXRULE_RULE_CTL_MSG,AIF2_PE_MODTXRULE_RULE_EN,AIF2_PE_OBSAI_HDR_OBSAI_TS_ADR,AIF2_PE_OBSAI_HDR_OBSAI_TYPE,
 *        AIF2_PE_OBSAI_HDR_OBSAI_ADR,AIF2_PE_OBSAI_HDR_OBSAI_TS_MASK,AIF2_PE_OBSAI_HDR_OBSAI_TS_FRMT,
 *        AIF2_PE_OBSAI_HDR_OBSAI_PKT,AIF2_PE_OBSAI_HDR_BB_HOP,
 *        AIF2_PE_OBSAI_DBM_DBM_X,AIF2_PE_OBSAI_DBM_DBM_1MULT,
 *        AIF2_PE_OBSAI_DBM_DBM_1SIZE,AIF2_PE_OBSAI_DBM_DBM_2SIZE,AIF2_PE_DBM_MAP_DBM_BIT_MAP,
 *        AIF2_PE_RULE_CHANLUT0_CHANINDEX,AIF2_PE_RULE_CHANLUT0_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT0_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT1_CHANINDEX,AIF2_PE_RULE_CHANLUT1_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT1_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT2_CHANINDEX,AIF2_PE_RULE_CHANLUT2_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT2_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT3_CHANINDEX,AIF2_PE_RULE_CHANLUT3_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT3_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT4_CHANINDEX,AIF2_PE_RULE_CHANLUT4_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT4_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT5_CHANINDEX,AIF2_PE_RULE_CHANLUT5_CHANINDEX_EN,AIF2_PE_RULE_CHANLUT5_CPRI_PKT_EN,
 *       AIF2_PE_RULE_CHANLUT6_CHANINDEX,AIF2_PE_RULE_CHANLUT6_CHANINDEX_EN,
 *       AIF2_PE_RULE_CHANLUT7_CHANINDEX,AIF2_PE_RULE_CHANLUT7_CHANINDEX_EN
 *   @b Example
 *   @verbatim
        Aif2Fl_setupPeCommonRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline 
Aif2Fl_Status Aif2Fl_setupPeCommonRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup    *commonSetup
)
{
    Aif2Fl_PeCommonSetup           *pPeConfig;
    uint32_t                      tempReg, i;
    pPeConfig = commonSetup->pPeCommonSetup;

     /* PE global register setup  */ 
    CSL_FINS(hAif2->regs->PE_GLOBAL_EN_SET, AIF2_PE_GLOBAL_EN_SET_DONT_CARE, pPeConfig->PeGlobalEnable);
    CSL_FINS(hAif2->regs->PE_GLOBAL, AIF2_PE_GLOBAL_TOKEN_PHASE, pPeConfig->PeTokenPhase);
    CSL_FINS(hAif2->regs->PE_GLOBAL, AIF2_PE_GLOBAL_ENET_HDR_SEL, pPeConfig->EnetHeaderSelect);
    CSL_FINS(hAif2->regs->PE_GLOBAL, AIF2_PE_GLOBAL_DIO_LEN, pPeConfig->GlobalDioLen);
    
     /* PE frame terminal count setup */
    for(i = 0; i < 6; i++){
    tempReg = CSL_FMK(AIF2_PE_FRM_TC_FRM_INDEX_TC, pPeConfig->PeFrameTC[i].FrameIndexTc)|
  	              CSL_FMK(AIF2_PE_FRM_TC_FRM_INDEX_SC, pPeConfig->PeFrameTC[i].FrameIndexSc)|
                     CSL_FMK(AIF2_PE_FRM_TC_FRM_SYM_TC, pPeConfig->PeFrameTC[i].FrameSymbolTc);
    hAif2->regs->PE_FRM_TC[i] = tempReg;
    }
	
    /* enable PE dma channel  */ 
    for(i=0;i<128;i++)
    CSL_FINS(hAif2->regs->PE_DMACHAN_EN[i], AIF2_PE_DMACHAN_EN_CH_EN, pPeConfig->bEnableCh[i]);

    /* PE 128 DMA channel 0 register setup */
    for(i = 0; i < 128; i++){
    tempReg = CSL_FMK(AIF2_PE_DMA0CHAN_CRC_EN, pPeConfig->PeDmaCh0[i].bCrcEn)|
  	              CSL_FMK(AIF2_PE_DMA0CHAN_FRM_TC, pPeConfig->PeDmaCh0[i].FrameTC)|
                     CSL_FMK(AIF2_PE_DMA0CHAN_RT_CTL, pPeConfig->PeDmaCh0[i].RtControl)|
	              CSL_FMK(AIF2_PE_DMA0CHAN_CRC_TYPE, pPeConfig->PeDmaCh0[i].CrcType)|
	              CSL_FMK(AIF2_PE_DMA0CHAN_ETHERNET, pPeConfig->PeDmaCh0[i].isEthernet)|
                     CSL_FMK(AIF2_PE_DMA0CHAN_CRC_HDR, pPeConfig->PeDmaCh0[i].CrcObsaiHeader);
    hAif2->regs->PE_DMA0CHAN[i] = tempReg;
    }

    /* PE 128 input fifo register setup */
    for(i = 0; i < 128; i++){
    tempReg = CSL_FMK(AIF2_PE_IN_FIFO_MF_WMARK, pPeConfig->PeInFifo[i].MFifoWmark)|
                     CSL_FMK(AIF2_PE_IN_FIFO_MF_FULL_LEV, pPeConfig->PeInFifo[i].MFifoFullLevel)|
	              CSL_FMK(AIF2_PE_IN_FIFO_SYNC_SYM, pPeConfig->PeInFifo[i].SyncSymbol);
    hAif2->regs->PE_IN_FIFO[i] = tempReg;
    }

     /* PE 128 AxC offset register setup  */ 
    for(i = 0; i < 128; i++)
    CSL_FINS(hAif2->regs->PE_AXC_OFFSET[i], AIF2_PE_AXC_OFFSET_AXC_OFFSET, pPeConfig->PeAxcOffset[i]);

	 
    /* PE frame message terminal count register setup */
    for(i = 0; i < 128; i++)
    CSL_FINS(hAif2->regs->PE_FRM_MSG_TC[i], AIF2_PE_FRM_MSG_TC_FRME_MSG_TC, pPeConfig->PeFrameMsgTc[i]);

    /* PE modulo rule register setup */
    for(i=0;i<64;i++)
    {
    tempReg = CSL_FMK(AIF2_PE_MODTXRULE_RULE_MOD, pPeConfig->PeModuloTc[i].RuleModulo)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_INDEX, pPeConfig->PeModuloTc[i].RuleIndex)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_LK, pPeConfig->PeModuloTc[i].RuleLink)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_CTL_MSG, pPeConfig->PeModuloTc[i].bRuleObsaiCtlMsg)|
                     CSL_FMK(AIF2_PE_MODTXRULE_RULE_EN, pPeConfig->PeModuloTc[i].bEnableRule);
    hAif2->regs->PE_MODTXRULE[i] = tempReg;
    }

    /* PE 128 DMA channel OBSAI HDR register setup*/
    for(i = 0; i < 128; i++) {
    tempReg = CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_ADR, pPeConfig->PeChObsaiTS[i])    |
                  CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TYPE, pPeConfig->PeChObsaiType[i])   |
                  CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_ADR,   pPeConfig->PeChObsaiAddr[i]) |
	           CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_MASK, pPeConfig->PeChObsaiTsMask[i])   |
                  CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_TS_FRMT,   pPeConfig->PeChObsaiTsfomat[i])|
                   CSL_FMK(AIF2_PE_OBSAI_HDR_OBSAI_PKT, pPeConfig->PeObsaiPkt[i])   |
                  CSL_FMK(AIF2_PE_OBSAI_HDR_BB_HOP,   pPeConfig->PeBbHop[i]);
    hAif2->regs->PE_OBSAI_HDR[i] = tempReg;
    }
	
    /* PE  Dual bit map FSM register setup*/
    for(i=0;i<64;i++)
    {
    tempReg = CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_X, pPeConfig->PeObsaiDualBitMap[i].DbmX)    |
                  CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_1MULT,   pPeConfig->PeObsaiDualBitMap[i].Dbm1Mult) |
	           CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_1SIZE, pPeConfig->PeObsaiDualBitMap[i].Dbm1Size)   |
                  CSL_FMK(AIF2_PE_OBSAI_DBM_DBM_2SIZE,   pPeConfig->PeObsaiDualBitMap[i].Dbm2Size);
    hAif2->regs->PE_OBSAI_DBM[i] = tempReg;
    
    /* PE CPRI DBM 1 map setup*/
    CSL_FINS(hAif2->regs->PE_DBM_MAP[0+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm1Map[0]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[1+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm1Map[1]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[2+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm1Map[2]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[3+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm1Map[3]); 

    /* PE CPRI DBM 2 map setup*/
    CSL_FINS(hAif2->regs->PE_DBM_MAP[4+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm2Map[0]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[5+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm2Map[1]); 
    CSL_FINS(hAif2->regs->PE_DBM_MAP[6+(8*i)], AIF2_PE_DBM_MAP_DBM_BIT_MAP, pPeConfig->PeObsaiDualBitMap[i].Dbm2Map[2]); 
    }

    /* PE 512 Channel Rule LUT 0 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT0_CHANINDEX, pPeConfig->ChIndex0[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT0_CHANINDEX_EN, pPeConfig->bEnableChIndex0[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT0_CPRI_PKT_EN, pPeConfig->CpriPktEn0[i]);
    hAif2->regs->PE_RULE_CHANLUT0[i] = tempReg;
    }

     /* PE 512 Channel Rule LUT 1 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT1_CHANINDEX, pPeConfig->ChIndex1[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT1_CHANINDEX_EN, pPeConfig->bEnableChIndex1[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT1_CPRI_PKT_EN, pPeConfig->CpriPktEn1[i]);
    hAif2->regs->PE_RULE_CHANLUT1[i] = tempReg;
    }

     /* PE 512 Channel Rule LUT 2 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT2_CHANINDEX, pPeConfig->ChIndex2[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT2_CHANINDEX_EN, pPeConfig->bEnableChIndex2[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT2_CPRI_PKT_EN, pPeConfig->CpriPktEn2[i]);
    hAif2->regs->PE_RULE_CHANLUT2[i] = tempReg;
    }

    /* PE 512 Channel Rule LUT 3 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT3_CHANINDEX, pPeConfig->ChIndex3[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT3_CHANINDEX_EN, pPeConfig->bEnableChIndex3[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT3_CPRI_PKT_EN, pPeConfig->CpriPktEn3[i]);
    hAif2->regs->PE_RULE_CHANLUT3[i] = tempReg;
    }

     /* PE 512 Channel Rule LUT 4 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT4_CHANINDEX, pPeConfig->ChIndex4[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT4_CHANINDEX_EN, pPeConfig->bEnableChIndex4[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT4_CPRI_PKT_EN, pPeConfig->CpriPktEn4[i]);
    hAif2->regs->PE_RULE_CHANLUT4[i] = tempReg;
    }

     /* PE 512 Channel Rule LUT 5 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT5_CHANINDEX, pPeConfig->ChIndex5[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT5_CHANINDEX_EN, pPeConfig->bEnableChIndex5[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT5_CPRI_PKT_EN, pPeConfig->CpriPktEn5[i]);
    hAif2->regs->PE_RULE_CHANLUT5[i] = tempReg;
    }

      /* PE 512 Channel Rule LUT 6 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT6_CHANINDEX, pPeConfig->ChIndex6[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT6_CHANINDEX_EN, pPeConfig->bEnableChIndex6[i]);
    hAif2->regs->PE_RULE_CHANLUT6[i] = tempReg;
    }

     /* PE 512 Channel Rule LUT 7 register setup */
    for(i = 0; i < 512; i++){
    tempReg = CSL_FMK(AIF2_PE_RULE_CHANLUT7_CHANINDEX, pPeConfig->ChIndex7[i])|
    CSL_FMK(AIF2_PE_RULE_CHANLUT7_CHANINDEX_EN, pPeConfig->bEnableChIndex7[i]);
    hAif2->regs->PE_RULE_CHANLUT7[i] = tempReg;
    }
	
    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupIngrDbRegs
 *
 *   @b Description
 *   @n AIF2 ingress Db setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_IDB_CFG_DIO_LEN,AIF2_DB_IDB_GLOBAL_EN_SET_DONT_CARE,
 *        AIF2_DB_IDB_CH_EN,
 *        AIF2_DB_IDB_PTR_CH_BUF_DEPTH,AIF2_DB_IDB_PTR_CH_BASE_ADDR,
 *        AIF2_DB_IDB_CFG_CH_DAT_SWAP,AIF2_DB_IDB_CFG_CH_IQ_ORDER,AIF2_DB_IDB_CFG_CH_PS_EN,AIF2_DB_IDB_CFG_CH_PKT_TYPE
 *   @b Example
 *   @verbatim
        Aif2Fl_setupIngrDbRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status Aif2Fl_setupIngrDbRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup *commonSetup
)
{
    Aif2Fl_IngrDbSetup *pIngrDbConfig;
    uint32_t         tempReg =0, i;

    pIngrDbConfig = commonSetup->pIngrDbSetup;

    /* setup for inbound DIO ram size */
    CSL_FINS(hAif2->regs->DB_IDB_CFG, AIF2_DB_IDB_CFG_DIO_LEN, 
                    pIngrDbConfig->DioBufferLen);
    /* IDB Global enable disable setup */
    CSL_FINS(hAif2->regs->DB_IDB_GLOBAL_EN_SET, AIF2_DB_IDB_GLOBAL_EN_SET_DONT_CARE, 
                    pIngrDbConfig->bEnableIngrDb);

    for(i=0; i< 32; i++)
    tempReg |= ((0x1 & pIngrDbConfig->bEnableChannel[i]) << i);
    hAif2->regs->DB_IDB_CH_EN[0] = tempReg;
    tempReg = 0;

    for(i=32; i< 64; i++)
    tempReg |= ((0x1 & pIngrDbConfig->bEnableChannel[i]) << (i-32));
    hAif2->regs->DB_IDB_CH_EN[1] = tempReg;
    tempReg = 0;

    for(i=64; i< 96; i++)
    tempReg |= ((0x1 & pIngrDbConfig->bEnableChannel[i]) << (i-64));
    hAif2->regs->DB_IDB_CH_EN[2] = tempReg;
    tempReg = 0;

    for(i=96; i< 128; i++)
    tempReg |= ((0x1 & pIngrDbConfig->bEnableChannel[i]) << (i-96));
    hAif2->regs->DB_IDB_CH_EN[3] = tempReg;
    tempReg = 0;

    for(i=0; i<128;i++)
    {
       if(pIngrDbConfig->bEnableChannel[i] == true)
      	{
          tempReg = CSL_FMK(AIF2_DB_IDB_PTR_CH_BASE_ADDR, pIngrDbConfig->IngrDbChannel[i].BaseAddress)  |
              CSL_FMK(AIF2_DB_IDB_PTR_CH_BUF_DEPTH, pIngrDbConfig->IngrDbChannel[i].BufDepth);
              
          hAif2->regs->DB_IDB_PTR_CH[i] = tempReg;

          tempReg = CSL_FMK(AIF2_DB_IDB_CFG_CH_DAT_SWAP, pIngrDbConfig->IngrDbChannel[i].DataSwap)  |
              CSL_FMK(AIF2_DB_IDB_CFG_CH_IQ_ORDER, pIngrDbConfig->IngrDbChannel[i].IQOrder)  |
              CSL_FMK(AIF2_DB_IDB_CFG_CH_PS_EN, pIngrDbConfig->IngrDbChannel[i].bEnablePsData) |
	       CSL_FMK(AIF2_DB_IDB_CFG_CH_PKT_TYPE, pIngrDbConfig->IngrDbChannel[i].PacketType);

          hAif2->regs->DB_IDB_CFG_CH[i] = tempReg;
      	}
    }
	
    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupEgrDbRegs
 *
 *   @b Description
 *   @n AIF2 Egress Db setup
 *
 *   @b Arguments
 *   @verbatim

            linkSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_DB_EDB_CFG_DIO_LEN,AIF2_DB_EDB_GLOBAL_EN_SET_DONT_CARE,
 *        AIF2_DB_EDB_CH_EN,AIF2_DB_EDB_CFG_PM_CTL,
 *        AIF2_DB_EDB_PTR_CH_BUF_DEPTH,AIF2_DB_EDB_PTR_CH_BASE_ADDR,
 *        AIF2_DB_EDB_CFG_CH_DAT_SWAP,AIF2_DB_EDB_CFG_CH_IQ_ORDER,AIF2_DB_EDB_CFG_CH_DIO_OFFSET
 *   @b Example
 *   @verbatim
        Aif2Fl_setupEgrDbRegs (linkSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status Aif2Fl_setupEgrDbRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup *commonSetup
)
{
    Aif2Fl_EgrDbSetup *pEgrDbConfig;
    uint32_t         tempReg = 0,i;

    pEgrDbConfig = commonSetup->pEgrDbSetup;

    /* Setup egress DIO ram size */
    CSL_FINS(hAif2->regs->DB_EDB_CFG, AIF2_DB_EDB_CFG_DIO_LEN, pEgrDbConfig->DioBufferLen);

    /* Setup egress Packet mode control */
    CSL_FINS(hAif2->regs->DB_EDB_CFG, AIF2_DB_EDB_CFG_PM_CTL, pEgrDbConfig->PmControl);
	
    /* IDB Global enable disable setup */
    CSL_FINS(hAif2->regs->DB_EDB_GLOBAL_EN_SET, AIF2_DB_EDB_GLOBAL_EN_SET_DONT_CARE, 
                    pEgrDbConfig->bEnableEgrDb);

    for(i=0; i< 32; i++)
    tempReg |= ((0x1 & pEgrDbConfig->bEnableChannel[i]) << i);
    hAif2->regs->DB_EDB_CH_EN[0] = tempReg;
    tempReg = 0;

    for(i=32; i< 64; i++)
    tempReg |= ((0x1 & pEgrDbConfig->bEnableChannel[i]) << (i-32));
    hAif2->regs->DB_EDB_CH_EN[1] = tempReg;
    tempReg = 0;

    for(i=64; i< 96; i++)
    tempReg |= ((0x1 & pEgrDbConfig->bEnableChannel[i]) << (i-64));
    hAif2->regs->DB_EDB_CH_EN[2] = tempReg;
    tempReg = 0;

    for(i=96; i< 128; i++)
    tempReg |= ((0x1 & pEgrDbConfig->bEnableChannel[i]) << (i-96));
    hAif2->regs->DB_EDB_CH_EN[3] = tempReg;
    tempReg = 0;

    for(i=0; i<128;i++)
    {
       if(pEgrDbConfig->bEnableChannel[i] == true)
      	{
          tempReg = CSL_FMK(AIF2_DB_EDB_PTR_CH_BASE_ADDR, pEgrDbConfig->EgrDbChannel[i].BaseAddress)  |
              CSL_FMK(AIF2_DB_EDB_PTR_CH_BUF_DEPTH, pEgrDbConfig->EgrDbChannel[i].BufDepth);
              
          hAif2->regs->DB_EDB_PTR_CH[i] = tempReg;

          tempReg = CSL_FMK(AIF2_DB_EDB_CFG_CH_DAT_SWAP, pEgrDbConfig->EgrDbChannel[i].DataSwap)  |
              CSL_FMK(AIF2_DB_EDB_CFG_CH_IQ_ORDER, pEgrDbConfig->EgrDbChannel[i].IQOrder) |
		CSL_FMK(AIF2_DB_EDB_CFG_CH_DIO_OFFSET, pEgrDbConfig->EgrDbChannel[i].EgressDioOffset);
              
          hAif2->regs->DB_EDB_CFG_CH[i] = tempReg;
      	}
    }
	
    return AIF2FL_SOK;
}

/** ============================================================================
 *   @n@b Aif2Fl_setupAdCommonRegs
 *
 *   @b Description
 *   @n AIF2 AD common setup
 *
 *   @b Arguments
 *   @verbatim

            commonSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AD_ISCH_CFG_FAIL_MODE,AIF2_AD_ISCH_CFG_PRI,
 *        AIF2_AD_ESCH_CFG_TXQ_QMGR,AIF2_AD_ESCH_CFG_TXQ_QNUM,AIF2_AD_ESCH_CFG_PRI,
 *        AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE,AIF2_AD_DIO_E_GLOBAL_EN_SET_DONT_CARE,
 *        AIF2_AD_ISCH_GLOBAL_EN_SET_DONT_CARE,AIF2_AD_ESCH_GLOBAL_EN_SET_DONT_CARE
 *   @b Example
 *   @verbatim
        Aif2Fl_setupAdCommonRegs (commonSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status  Aif2Fl_setupAdCommonRegs(
    Aif2Fl_Handle hAif2,
    Aif2Fl_CommonSetup *commonSetup
)
{
    Aif2Fl_AdCommonSetup *pAdCommonConfig;
    uint32_t         tempReg;

    pAdCommonConfig = commonSetup->pAdCommonSetup;

    /* DIO Global Enable/disable */
    if(pAdCommonConfig->IngrGlobalDioEnable == true)
    CSL_FINS(hAif2->regs->AD_DIO_I_GLOBAL_EN_SET, AIF2_AD_DIO_I_GLOBAL_EN_SET_DONT_CARE,0x1);
    else
    CSL_FINS(hAif2->regs->AD_DIO_I_GLOBAL_EN_CLR, AIF2_AD_DIO_I_GLOBAL_EN_CLR_DONT_CARE,0x1);

    if(pAdCommonConfig->EgrGlobalDioEnable == true)
    CSL_FINS(hAif2->regs->AD_DIO_E_GLOBAL_EN_SET, AIF2_AD_DIO_E_GLOBAL_EN_SET_DONT_CARE,0x1);
    else
    CSL_FINS(hAif2->regs->AD_DIO_E_GLOBAL_EN_CLR, AIF2_AD_DIO_E_GLOBAL_EN_CLR_DONT_CARE,0x1);

    /* AD scheduler Global Enable/Disable */
    if(pAdCommonConfig->IngrGlobalEnable == true)
    CSL_FINS(hAif2->regs->AD_ISCH_GLOBAL_EN_SET, AIF2_AD_ISCH_GLOBAL_EN_SET_DONT_CARE,0x1);
    else
    CSL_FINS(hAif2->regs->AD_ISCH_GLOBAL_EN_CLR, AIF2_AD_ISCH_GLOBAL_EN_CLR_DONT_CARE,0x1);

    if(pAdCommonConfig->EgrGlobalEnable == true)
    CSL_FINS(hAif2->regs->AD_ESCH_GLOBAL_EN_SET, AIF2_AD_ESCH_GLOBAL_EN_SET_DONT_CARE,0x1);
    else
    CSL_FINS(hAif2->regs->AD_ESCH_GLOBAL_EN_CLR, AIF2_AD_ESCH_GLOBAL_EN_CLR_DONT_CARE,0x1);

    /*Setup ingress AD scheduler configuration */
    tempReg = CSL_FMK(AIF2_AD_ISCH_CFG_FAIL_MODE, pAdCommonConfig->FailMode)  |
              CSL_FMK(AIF2_AD_ISCH_CFG_PRI, pAdCommonConfig->IngrPriority);

    hAif2->regs->AD_ISCH_CFG = tempReg;
	
    /*Setup egress AD scheduler configuration */
    tempReg = CSL_FMK(AIF2_AD_ESCH_CFG_TXQ_QMGR, pAdCommonConfig->Tx_QueManager)  |
              CSL_FMK(AIF2_AD_ESCH_CFG_TXQ_QNUM, pAdCommonConfig->Tx_QueNum)  |
              CSL_FMK(AIF2_AD_ESCH_CFG_PRI, pAdCommonConfig->EgrPriority);

    hAif2->regs->AD_ESCH_CFG = tempReg;

    return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupAdInDioRegs
 *
 *   @b Description
 *   @n AIF2 AD Direct IO setup
 *
 *   @b Arguments
 *   @verbatim

            commonSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_I_TABLE_SEL_BCN_TABLE_SEL,
 *        AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_QW,AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC,
 *        AIF2_AD_DIO_I_DMA_CFG0_DMA_BRST_LN,AIF2_AD_DIO_I_DMA_CFG0_RSA_EN,AIF2_AD_DIO_I_DMA_CFG0_DMA_CH,AIF2_AD_DIO_I_DMA_CFG0_DMA_NUM_BLKS,
 *        AIF2_AD_DIO_I_DMA_CFG1_DMA_BASE_ADDR,
 *        AIF2_AD_DIO_I_DMA_CFG2_DMA_BRST_ADDR_STRIDE,AIF2_AD_DIO_I_DMA_CFG2_DMA_BLK_ADDR_STRIDE,
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
 *        AIF2_AD_DIO_I_TABLE_SEL_BCN_TABLE_SEL,
 *        AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_QW,AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC,
 *        AIF2_AD_DIO_I_DMA_CFG0_DMA_BRST_LN,AIF2_AD_DIO_I_DMA_CFG0_RSA_EN,AIF2_AD_DIO_I_DMA_CFG0_DMA_CH,AIF2_AD_DIO_I_DMA_CFG0_DMA_NUM_BLKS,
 *        AIF2_AD_DIO_I_DMA_CFG1_DMA_BASE_ADDR,
 *        AIF2_AD_DIO_I_DMA_CFG2_DMA_BRST_ADDR_STRIDE,AIF2_AD_DIO_I_DMA_CFG2_DMA_BLK_ADDR_STRIDE,
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
        Aif2Fl_setupAdInDioRegs (commonSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status  Aif2Fl_setupAdInDioRegs(
   Aif2Fl_Handle hAif2,
   Aif2Fl_CommonSetup *commonSetup
)
{
	Aif2Fl_AdDioSetup *pAdDioConfig;
	uint32_t         tempReg, i;

	pAdDioConfig = commonSetup->pAdDioSetup;

	for(i =0; i < 3;i++)
	{
		if(pAdDioConfig->IngrDioEngineEnable[i] == true)
		{

                    /* setup for inbound DIO table selection */
                    CSL_FINS(hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_TABLE_SEL , AIF2_AD_DIO_I_TABLE_SEL_BCN_TABLE_SEL, 
                    pAdDioConfig->IngrDioEngine[i].BcnTableSelect);
		
			/*Setup ingress AD scheduler num QW and AxC */
			tempReg = CSL_FMK(AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_QW, pAdDioConfig->IngrDioEngine[i].NumQuadWord)  |
			                 CSL_FMK(AIF2_AD_DIO_I_TABLE_LOOP_CFG_NUM_AXC, pAdDioConfig->IngrDioEngine[i].NumAxC);

			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_TABLE_LOOP_CFG = tempReg;
#ifdef K2
			tempReg = CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_BRST_LN, pAdDioConfig->IngrDioEngine[i].DmaBurstLen)  |
				          CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_RSA_EN, pAdDioConfig->IngrDioEngine[i].bEnIngressRsaFormat)  |
			                 CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_CH_EN, pAdDioConfig->IngrDioEngine[i].bEnDmaChannel)  |
			                 CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_NUM_BLKS, pAdDioConfig->IngrDioEngine[i].DmaNumBlock);
#else
			tempReg = CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_BRST_LN, pAdDioConfig->IngrDioEngine[i].DmaBurstLen)  |
			                 CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_CH_EN, pAdDioConfig->IngrDioEngine[i].bEnDmaChannel)  |
			                 CSL_FMK(AIF2_AD_DIO_I_DMA_CFG0_DMA_NUM_BLKS, pAdDioConfig->IngrDioEngine[i].DmaNumBlock);
#endif

			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_DMA_CFG0 = tempReg;

            /* setup for inbound DMA base address in memory */
            tempReg = (pAdDioConfig->IngrDioEngine[i].DmaBaseAddr >> 4);
            CSL_FINS(hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_DMA_CFG1 , AIF2_AD_DIO_I_DMA_CFG1_DMA_BASE_ADDR,tempReg);

			/*Setup ingress DMA address stride configuration */
			tempReg = CSL_FMK(AIF2_AD_DIO_I_DMA_CFG2_DMA_BRST_ADDR_STRIDE, pAdDioConfig->IngrDioEngine[i].DmaBurstAddrStride)  |
			                 CSL_FMK(AIF2_AD_DIO_I_DMA_CFG2_DMA_BLK_ADDR_STRIDE, pAdDioConfig->IngrDioEngine[i].DmaBlockAddrStride);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_DMA_CFG2 = tempReg;

                    if(pAdDioConfig->IngrDioEngine[i].BcnTableSelect == AIF2FL_AD_DIO_BCN_TABLE_0){
                     /* setup DBCN for table 0  */
			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN0, pAdDioConfig->IngrDioEngine[i].DBCN[0])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN1, pAdDioConfig->IngrDioEngine[i].DBCN[1])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN2, pAdDioConfig->IngrDioEngine[i].DBCN[2])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW0_DBCN3, pAdDioConfig->IngrDioEngine[i].DBCN[3]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW0 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN4, pAdDioConfig->IngrDioEngine[i].DBCN[4])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN5, pAdDioConfig->IngrDioEngine[i].DBCN[5])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN6, pAdDioConfig->IngrDioEngine[i].DBCN[6])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW1_DBCN7, pAdDioConfig->IngrDioEngine[i].DBCN[7]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW1 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN8, pAdDioConfig->IngrDioEngine[i].DBCN[8])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN9, pAdDioConfig->IngrDioEngine[i].DBCN[9])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN10, pAdDioConfig->IngrDioEngine[i].DBCN[10])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW2_DBCN11, pAdDioConfig->IngrDioEngine[i].DBCN[11]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW2 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN12, pAdDioConfig->IngrDioEngine[i].DBCN[12])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN13, pAdDioConfig->IngrDioEngine[i].DBCN[13])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN14, pAdDioConfig->IngrDioEngine[i].DBCN[14])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW3_DBCN15, pAdDioConfig->IngrDioEngine[i].DBCN[15]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW3 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN16, pAdDioConfig->IngrDioEngine[i].DBCN[16])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN17, pAdDioConfig->IngrDioEngine[i].DBCN[17])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN18, pAdDioConfig->IngrDioEngine[i].DBCN[18])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW4_DBCN19, pAdDioConfig->IngrDioEngine[i].DBCN[19]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW4 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN20, pAdDioConfig->IngrDioEngine[i].DBCN[20])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN21, pAdDioConfig->IngrDioEngine[i].DBCN[21])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN22, pAdDioConfig->IngrDioEngine[i].DBCN[22])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW5_DBCN23, pAdDioConfig->IngrDioEngine[i].DBCN[23]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW5 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN24, pAdDioConfig->IngrDioEngine[i].DBCN[24])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN25, pAdDioConfig->IngrDioEngine[i].DBCN[25])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN26, pAdDioConfig->IngrDioEngine[i].DBCN[26])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW6_DBCN27, pAdDioConfig->IngrDioEngine[i].DBCN[27]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW6 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN28, pAdDioConfig->IngrDioEngine[i].DBCN[28])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN29, pAdDioConfig->IngrDioEngine[i].DBCN[29])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN30, pAdDioConfig->IngrDioEngine[i].DBCN[30])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW7_DBCN31, pAdDioConfig->IngrDioEngine[i].DBCN[31]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW7 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN32, pAdDioConfig->IngrDioEngine[i].DBCN[32])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN33, pAdDioConfig->IngrDioEngine[i].DBCN[33])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN34, pAdDioConfig->IngrDioEngine[i].DBCN[34])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW8_DBCN35, pAdDioConfig->IngrDioEngine[i].DBCN[35]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW8 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN36, pAdDioConfig->IngrDioEngine[i].DBCN[36])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN37, pAdDioConfig->IngrDioEngine[i].DBCN[37])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN38, pAdDioConfig->IngrDioEngine[i].DBCN[38])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW9_DBCN39, pAdDioConfig->IngrDioEngine[i].DBCN[39]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW9 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN40, pAdDioConfig->IngrDioEngine[i].DBCN[40])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN41, pAdDioConfig->IngrDioEngine[i].DBCN[41])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN42, pAdDioConfig->IngrDioEngine[i].DBCN[42])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW10_DBCN43, pAdDioConfig->IngrDioEngine[i].DBCN[43]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW10 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN44, pAdDioConfig->IngrDioEngine[i].DBCN[44])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN45, pAdDioConfig->IngrDioEngine[i].DBCN[45])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN46, pAdDioConfig->IngrDioEngine[i].DBCN[46])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW11_DBCN47, pAdDioConfig->IngrDioEngine[i].DBCN[47]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW11 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN48, pAdDioConfig->IngrDioEngine[i].DBCN[48])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN49, pAdDioConfig->IngrDioEngine[i].DBCN[49])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN50, pAdDioConfig->IngrDioEngine[i].DBCN[50])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW12_DBCN51, pAdDioConfig->IngrDioEngine[i].DBCN[51]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW12 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN52, pAdDioConfig->IngrDioEngine[i].DBCN[52])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN53, pAdDioConfig->IngrDioEngine[i].DBCN[53])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN54, pAdDioConfig->IngrDioEngine[i].DBCN[54])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW13_DBCN55, pAdDioConfig->IngrDioEngine[i].DBCN[55]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW13 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN56, pAdDioConfig->IngrDioEngine[i].DBCN[56])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN57, pAdDioConfig->IngrDioEngine[i].DBCN[57])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN58, pAdDioConfig->IngrDioEngine[i].DBCN[58])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW14_DBCN59, pAdDioConfig->IngrDioEngine[i].DBCN[59]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW14 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN60, pAdDioConfig->IngrDioEngine[i].DBCN[60])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN61, pAdDioConfig->IngrDioEngine[i].DBCN[61])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN62, pAdDioConfig->IngrDioEngine[i].DBCN[62])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE0_ROW15_DBCN63, pAdDioConfig->IngrDioEngine[i].DBCN[63]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE0_ROW15 = tempReg;
                    	}
			else {
                      /* setup DBCN for table 1  */
			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN0, pAdDioConfig->IngrDioEngine[i].DBCN[0])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN1, pAdDioConfig->IngrDioEngine[i].DBCN[1])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN2, pAdDioConfig->IngrDioEngine[i].DBCN[2])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW0_DBCN3, pAdDioConfig->IngrDioEngine[i].DBCN[3]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW0 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN4, pAdDioConfig->IngrDioEngine[i].DBCN[4])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN5, pAdDioConfig->IngrDioEngine[i].DBCN[5])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN6, pAdDioConfig->IngrDioEngine[i].DBCN[6])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW1_DBCN7, pAdDioConfig->IngrDioEngine[i].DBCN[7]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW1 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN8, pAdDioConfig->IngrDioEngine[i].DBCN[8])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN9, pAdDioConfig->IngrDioEngine[i].DBCN[9])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN10, pAdDioConfig->IngrDioEngine[i].DBCN[10])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW2_DBCN11, pAdDioConfig->IngrDioEngine[i].DBCN[11]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW2 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN12, pAdDioConfig->IngrDioEngine[i].DBCN[12])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN13, pAdDioConfig->IngrDioEngine[i].DBCN[13])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN14, pAdDioConfig->IngrDioEngine[i].DBCN[14])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW3_DBCN15, pAdDioConfig->IngrDioEngine[i].DBCN[15]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW3 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN16, pAdDioConfig->IngrDioEngine[i].DBCN[16])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN17, pAdDioConfig->IngrDioEngine[i].DBCN[17])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN18, pAdDioConfig->IngrDioEngine[i].DBCN[18])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW4_DBCN19, pAdDioConfig->IngrDioEngine[i].DBCN[19]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW4 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN20, pAdDioConfig->IngrDioEngine[i].DBCN[20])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN21, pAdDioConfig->IngrDioEngine[i].DBCN[21])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN22, pAdDioConfig->IngrDioEngine[i].DBCN[22])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW5_DBCN23, pAdDioConfig->IngrDioEngine[i].DBCN[23]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW5 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN24, pAdDioConfig->IngrDioEngine[i].DBCN[24])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN25, pAdDioConfig->IngrDioEngine[i].DBCN[25])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN26, pAdDioConfig->IngrDioEngine[i].DBCN[26])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW6_DBCN27, pAdDioConfig->IngrDioEngine[i].DBCN[27]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW6 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN28, pAdDioConfig->IngrDioEngine[i].DBCN[28])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN29, pAdDioConfig->IngrDioEngine[i].DBCN[29])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN30, pAdDioConfig->IngrDioEngine[i].DBCN[30])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW7_DBCN31, pAdDioConfig->IngrDioEngine[i].DBCN[31]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW7 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN32, pAdDioConfig->IngrDioEngine[i].DBCN[32])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN33, pAdDioConfig->IngrDioEngine[i].DBCN[33])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN34, pAdDioConfig->IngrDioEngine[i].DBCN[34])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW8_DBCN35, pAdDioConfig->IngrDioEngine[i].DBCN[35]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW8 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN36, pAdDioConfig->IngrDioEngine[i].DBCN[36])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN37, pAdDioConfig->IngrDioEngine[i].DBCN[37])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN38, pAdDioConfig->IngrDioEngine[i].DBCN[38])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW9_DBCN39, pAdDioConfig->IngrDioEngine[i].DBCN[39]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW9 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN40, pAdDioConfig->IngrDioEngine[i].DBCN[40])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN41, pAdDioConfig->IngrDioEngine[i].DBCN[41])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN42, pAdDioConfig->IngrDioEngine[i].DBCN[42])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW10_DBCN43, pAdDioConfig->IngrDioEngine[i].DBCN[43]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW10 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN44, pAdDioConfig->IngrDioEngine[i].DBCN[44])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN45, pAdDioConfig->IngrDioEngine[i].DBCN[45])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN46, pAdDioConfig->IngrDioEngine[i].DBCN[46])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW11_DBCN47, pAdDioConfig->IngrDioEngine[i].DBCN[47]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW11 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN48, pAdDioConfig->IngrDioEngine[i].DBCN[48])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN49, pAdDioConfig->IngrDioEngine[i].DBCN[49])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN50, pAdDioConfig->IngrDioEngine[i].DBCN[50])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW12_DBCN51, pAdDioConfig->IngrDioEngine[i].DBCN[51]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW12 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN52, pAdDioConfig->IngrDioEngine[i].DBCN[52])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN53, pAdDioConfig->IngrDioEngine[i].DBCN[53])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN54, pAdDioConfig->IngrDioEngine[i].DBCN[54])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW13_DBCN55, pAdDioConfig->IngrDioEngine[i].DBCN[55]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW13 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN56, pAdDioConfig->IngrDioEngine[i].DBCN[56])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN57, pAdDioConfig->IngrDioEngine[i].DBCN[57])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN58, pAdDioConfig->IngrDioEngine[i].DBCN[58])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW14_DBCN59, pAdDioConfig->IngrDioEngine[i].DBCN[59]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW14 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN60, pAdDioConfig->IngrDioEngine[i].DBCN[60])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN61, pAdDioConfig->IngrDioEngine[i].DBCN[61])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN62, pAdDioConfig->IngrDioEngine[i].DBCN[62])  |
			                 CSL_FMK(AIF2_AD_DIO_I_BCN_TABLE1_ROW15_DBCN63, pAdDioConfig->IngrDioEngine[i].DBCN[63]);
			hAif2->regs->AD_DIO_INGRESS[i].AD_DIO_I_BCN_TABLE1_ROW15 = tempReg;
                    	}
			
		}
	}
	return AIF2FL_SOK;
}


/** ============================================================================
 *   @n@b Aif2Fl_setupAdEDioRegs
 *
 *   @b Description
 *   @n AIF2 AD Direct IO setup
 *
 *   @b Arguments
 *   @verbatim

            commonSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n  Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AD_DIO_E_TABLE_SEL_BCN_TABLE_SEL,
 *        AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_QW,AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC,
 *        AIF2_AD_DIO_E_DMA_CFG0_DMA_BRST_LN,AIF2_AD_DIO_E_DMA_CFG0_RSA_EN,AIF2_AD_DIO_E_DMA_CFG0_DMA_CH_EN,
 *        AIF2_AD_DIO_E_DMA_CFG0_DMA_NUM_BLKS, AIF2_AD_DIO_E_DMA_CFG1_DMA_BASE_ADDR,
 *        AIF2_AD_DIO_E_DMA_CFG2_DMA_BRST_ADDR_STRIDE,AIF2_AD_DIO_E_DMA_CFG2_DMA_BLK_ADDR_STRIDE,
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
 *        AIF2_AD_DIO_E_TABLE_SEL_BCN_TABLE_SEL,
 *        AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_QW,AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC,
 *        AIF2_AD_DIO_E_DMA_CFG0_DMA_BRST_LN,AIF2_AD_DIO_E_DMA_CFG0_RSA_EN,AIF2_AD_DIO_E_DMA_CFG0_DMA_CH_EN,
 *        AIF2_AD_DIO_E_DMA_CFG0_DMA_NUM_BLKS, AIF2_AD_DIO_E_DMA_CFG1_DMA_BASE_ADDR,
 *        AIF2_AD_DIO_E_DMA_CFG2_DMA_BRST_ADDR_STRIDE,AIF2_AD_DIO_E_DMA_CFG2_DMA_BLK_ADDR_STRIDE,
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
 *          
 *   @b Example
 *   @verbatim
        Aif2Fl_setupAdEDioRegs (commonSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status  Aif2Fl_setupAdEDioRegs(
   Aif2Fl_Handle hAif2,
   Aif2Fl_CommonSetup *commonSetup
)
{
	Aif2Fl_AdDioSetup *pAdDioConfig;
	uint32_t         tempReg, i;

	pAdDioConfig = commonSetup->pAdDioSetup;

	for(i =0; i < 3;i++)
	{
		if(pAdDioConfig->EgrDioEngineEnable[i] == true)
		{

                    /* setup for inbound DIO table selection */
                    CSL_FINS(hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_TABLE_SEL , AIF2_AD_DIO_E_TABLE_SEL_BCN_TABLE_SEL, 
                    pAdDioConfig->EgrDioEngine[i].BcnTableSelect);
		
			/*Setup ingress AD scheduler num QW and AxC */
			tempReg = CSL_FMK(AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_QW, pAdDioConfig->EgrDioEngine[i].NumQuadWord)  |
			                 CSL_FMK(AIF2_AD_DIO_E_TABLE_LOOP_CFG_NUM_AXC, pAdDioConfig->EgrDioEngine[i].NumAxC);

			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_TABLE_LOOP_CFG = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_DMA_CFG0_DMA_BRST_LN, pAdDioConfig->EgrDioEngine[i].DmaBurstLen)  |
				          CSL_FMK(AIF2_AD_DIO_E_DMA_CFG0_RSA_EN, pAdDioConfig->EgrDioEngine[i].bEnEgressRsaFormat)  |
			                 CSL_FMK(AIF2_AD_DIO_E_DMA_CFG0_DMA_CH_EN, pAdDioConfig->EgrDioEngine[i].bEnDmaChannel)  |
			                 CSL_FMK(AIF2_AD_DIO_E_DMA_CFG0_DMA_NUM_BLKS, pAdDioConfig->EgrDioEngine[i].DmaNumBlock);

			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_DMA_CFG0 = tempReg;

                     /* setup for inbound DMA base address in memory */
                    tempReg = (pAdDioConfig->EgrDioEngine[i].DmaBaseAddr >> 4);
                    CSL_FINS(hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_DMA_CFG1 , AIF2_AD_DIO_E_DMA_CFG1_DMA_BASE_ADDR,tempReg);

			/*Setup ingress DMA address stride configuration */
			tempReg = CSL_FMK(AIF2_AD_DIO_E_DMA_CFG2_DMA_BRST_ADDR_STRIDE, pAdDioConfig->EgrDioEngine[i].DmaBurstAddrStride)  |
			                 CSL_FMK(AIF2_AD_DIO_E_DMA_CFG2_DMA_BLK_ADDR_STRIDE, pAdDioConfig->EgrDioEngine[i].DmaBlockAddrStride);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_DMA_CFG2 = tempReg;

                    if(pAdDioConfig->EgrDioEngine[i].BcnTableSelect == AIF2FL_AD_DIO_BCN_TABLE_0){
                     /* setup DBCN for table 0  */
                     tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN0, pAdDioConfig->EgrDioEngine[i].DBCN[0])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN1, pAdDioConfig->EgrDioEngine[i].DBCN[1])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN2, pAdDioConfig->EgrDioEngine[i].DBCN[2])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW0_DBCN3, pAdDioConfig->EgrDioEngine[i].DBCN[3]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW0 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN4, pAdDioConfig->EgrDioEngine[i].DBCN[4])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN5, pAdDioConfig->EgrDioEngine[i].DBCN[5])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN6, pAdDioConfig->EgrDioEngine[i].DBCN[6])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW1_DBCN7, pAdDioConfig->EgrDioEngine[i].DBCN[7]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW1 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN8, pAdDioConfig->EgrDioEngine[i].DBCN[8])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN9, pAdDioConfig->EgrDioEngine[i].DBCN[9])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN10, pAdDioConfig->EgrDioEngine[i].DBCN[10])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW2_DBCN11, pAdDioConfig->EgrDioEngine[i].DBCN[11]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW2 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN12, pAdDioConfig->EgrDioEngine[i].DBCN[12])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN13, pAdDioConfig->EgrDioEngine[i].DBCN[13])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN14, pAdDioConfig->EgrDioEngine[i].DBCN[14])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW3_DBCN15, pAdDioConfig->EgrDioEngine[i].DBCN[15]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW3 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN16, pAdDioConfig->EgrDioEngine[i].DBCN[16])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN17, pAdDioConfig->EgrDioEngine[i].DBCN[17])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN18, pAdDioConfig->EgrDioEngine[i].DBCN[18])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW4_DBCN19, pAdDioConfig->EgrDioEngine[i].DBCN[19]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW4 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN20, pAdDioConfig->EgrDioEngine[i].DBCN[20])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN21, pAdDioConfig->EgrDioEngine[i].DBCN[21])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN22, pAdDioConfig->EgrDioEngine[i].DBCN[22])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW5_DBCN23, pAdDioConfig->EgrDioEngine[i].DBCN[23]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW5 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN24, pAdDioConfig->EgrDioEngine[i].DBCN[24])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN25, pAdDioConfig->EgrDioEngine[i].DBCN[25])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN26, pAdDioConfig->EgrDioEngine[i].DBCN[26])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW6_DBCN27, pAdDioConfig->EgrDioEngine[i].DBCN[27]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW6 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN28, pAdDioConfig->EgrDioEngine[i].DBCN[28])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN29, pAdDioConfig->EgrDioEngine[i].DBCN[29])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN30, pAdDioConfig->EgrDioEngine[i].DBCN[30])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW7_DBCN31, pAdDioConfig->EgrDioEngine[i].DBCN[31]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW7 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN32, pAdDioConfig->EgrDioEngine[i].DBCN[32])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN33, pAdDioConfig->EgrDioEngine[i].DBCN[33])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN34, pAdDioConfig->EgrDioEngine[i].DBCN[34])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW8_DBCN35, pAdDioConfig->EgrDioEngine[i].DBCN[35]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW8 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN36, pAdDioConfig->EgrDioEngine[i].DBCN[36])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN37, pAdDioConfig->EgrDioEngine[i].DBCN[37])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN38, pAdDioConfig->EgrDioEngine[i].DBCN[38])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW9_DBCN39, pAdDioConfig->EgrDioEngine[i].DBCN[39]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW9 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN40, pAdDioConfig->EgrDioEngine[i].DBCN[40])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN41, pAdDioConfig->EgrDioEngine[i].DBCN[41])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN42, pAdDioConfig->EgrDioEngine[i].DBCN[42])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW10_DBCN43, pAdDioConfig->EgrDioEngine[i].DBCN[43]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW10 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN44, pAdDioConfig->EgrDioEngine[i].DBCN[44])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN45, pAdDioConfig->EgrDioEngine[i].DBCN[45])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN46, pAdDioConfig->EgrDioEngine[i].DBCN[46])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW11_DBCN47, pAdDioConfig->EgrDioEngine[i].DBCN[47]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW11 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN48, pAdDioConfig->EgrDioEngine[i].DBCN[48])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN49, pAdDioConfig->EgrDioEngine[i].DBCN[49])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN50, pAdDioConfig->EgrDioEngine[i].DBCN[50])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW12_DBCN51, pAdDioConfig->EgrDioEngine[i].DBCN[51]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW12 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN52, pAdDioConfig->EgrDioEngine[i].DBCN[52])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN53, pAdDioConfig->EgrDioEngine[i].DBCN[53])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN54, pAdDioConfig->EgrDioEngine[i].DBCN[54])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW13_DBCN55, pAdDioConfig->EgrDioEngine[i].DBCN[55]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW13 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN56, pAdDioConfig->EgrDioEngine[i].DBCN[56])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN57, pAdDioConfig->EgrDioEngine[i].DBCN[57])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN58, pAdDioConfig->EgrDioEngine[i].DBCN[58])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW14_DBCN59, pAdDioConfig->EgrDioEngine[i].DBCN[59]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW14 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN60, pAdDioConfig->EgrDioEngine[i].DBCN[60])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN61, pAdDioConfig->EgrDioEngine[i].DBCN[61])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN62, pAdDioConfig->EgrDioEngine[i].DBCN[62])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE0_ROW15_DBCN63, pAdDioConfig->EgrDioEngine[i].DBCN[63]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE0_ROW15 = tempReg;
                    	}
			else {
                      /* setup DBCN for table 1  */
			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN0, pAdDioConfig->EgrDioEngine[i].DBCN[0])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN1, pAdDioConfig->EgrDioEngine[i].DBCN[1])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN2, pAdDioConfig->EgrDioEngine[i].DBCN[2])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW0_DBCN3, pAdDioConfig->EgrDioEngine[i].DBCN[3]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW0 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN4, pAdDioConfig->EgrDioEngine[i].DBCN[4])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN5, pAdDioConfig->EgrDioEngine[i].DBCN[5])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN6, pAdDioConfig->EgrDioEngine[i].DBCN[6])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW1_DBCN7, pAdDioConfig->EgrDioEngine[i].DBCN[7]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW1 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN8, pAdDioConfig->EgrDioEngine[i].DBCN[8])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN9, pAdDioConfig->EgrDioEngine[i].DBCN[9])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN10, pAdDioConfig->EgrDioEngine[i].DBCN[10])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW2_DBCN11, pAdDioConfig->EgrDioEngine[i].DBCN[11]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW2 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN12, pAdDioConfig->EgrDioEngine[i].DBCN[12])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN13, pAdDioConfig->EgrDioEngine[i].DBCN[13])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN14, pAdDioConfig->EgrDioEngine[i].DBCN[14])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW3_DBCN15, pAdDioConfig->EgrDioEngine[i].DBCN[15]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW3 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN16, pAdDioConfig->EgrDioEngine[i].DBCN[16])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN17, pAdDioConfig->EgrDioEngine[i].DBCN[17])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN18, pAdDioConfig->EgrDioEngine[i].DBCN[18])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW4_DBCN19, pAdDioConfig->EgrDioEngine[i].DBCN[19]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW4 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN20, pAdDioConfig->EgrDioEngine[i].DBCN[20])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN21, pAdDioConfig->EgrDioEngine[i].DBCN[21])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN22, pAdDioConfig->EgrDioEngine[i].DBCN[22])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW5_DBCN23, pAdDioConfig->EgrDioEngine[i].DBCN[23]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW5 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN24, pAdDioConfig->EgrDioEngine[i].DBCN[24])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN25, pAdDioConfig->EgrDioEngine[i].DBCN[25])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN26, pAdDioConfig->EgrDioEngine[i].DBCN[26])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW6_DBCN27, pAdDioConfig->EgrDioEngine[i].DBCN[27]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW6 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN28, pAdDioConfig->EgrDioEngine[i].DBCN[28])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN29, pAdDioConfig->EgrDioEngine[i].DBCN[29])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN30, pAdDioConfig->EgrDioEngine[i].DBCN[30])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW7_DBCN31, pAdDioConfig->EgrDioEngine[i].DBCN[31]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW7 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN32, pAdDioConfig->EgrDioEngine[i].DBCN[32])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN33, pAdDioConfig->EgrDioEngine[i].DBCN[33])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN34, pAdDioConfig->EgrDioEngine[i].DBCN[34])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW8_DBCN35, pAdDioConfig->EgrDioEngine[i].DBCN[35]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW8 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN36, pAdDioConfig->EgrDioEngine[i].DBCN[36])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN37, pAdDioConfig->EgrDioEngine[i].DBCN[37])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN38, pAdDioConfig->EgrDioEngine[i].DBCN[38])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW9_DBCN39, pAdDioConfig->EgrDioEngine[i].DBCN[39]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW9 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN40, pAdDioConfig->EgrDioEngine[i].DBCN[40])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN41, pAdDioConfig->EgrDioEngine[i].DBCN[41])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN42, pAdDioConfig->EgrDioEngine[i].DBCN[42])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW10_DBCN43, pAdDioConfig->EgrDioEngine[i].DBCN[43]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW10 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN44, pAdDioConfig->EgrDioEngine[i].DBCN[44])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN45, pAdDioConfig->EgrDioEngine[i].DBCN[45])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN46, pAdDioConfig->EgrDioEngine[i].DBCN[46])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW11_DBCN47, pAdDioConfig->EgrDioEngine[i].DBCN[47]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW11 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN48, pAdDioConfig->EgrDioEngine[i].DBCN[48])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN49, pAdDioConfig->EgrDioEngine[i].DBCN[49])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN50, pAdDioConfig->EgrDioEngine[i].DBCN[50])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW12_DBCN51, pAdDioConfig->EgrDioEngine[i].DBCN[51]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW12 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN52, pAdDioConfig->EgrDioEngine[i].DBCN[52])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN53, pAdDioConfig->EgrDioEngine[i].DBCN[53])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN54, pAdDioConfig->EgrDioEngine[i].DBCN[54])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW13_DBCN55, pAdDioConfig->EgrDioEngine[i].DBCN[55]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW13 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN56, pAdDioConfig->EgrDioEngine[i].DBCN[56])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN57, pAdDioConfig->EgrDioEngine[i].DBCN[57])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN58, pAdDioConfig->EgrDioEngine[i].DBCN[58])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW14_DBCN59, pAdDioConfig->EgrDioEngine[i].DBCN[59]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW14 = tempReg;

			tempReg = CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN60, pAdDioConfig->EgrDioEngine[i].DBCN[60])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN61, pAdDioConfig->EgrDioEngine[i].DBCN[61])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN62, pAdDioConfig->EgrDioEngine[i].DBCN[62])  |
			                 CSL_FMK(AIF2_AD_DIO_E_BCN_TABLE1_ROW15_DBCN63, pAdDioConfig->EgrDioEngine[i].DBCN[63]);
			hAif2->regs->AD_DIO_EGRESS[i].AD_DIO_E_BCN_TABLE1_ROW15 = tempReg;
                    	}
			
		}
	}
	return AIF2FL_SOK;
}

 /** ============================================================================
 *   @n@b Aif2Fl_setupAtCommonRegs
 *
 *   @b Description
 *   @n AIF2 AT common setup
 *
 *   @b Arguments
 *   @verbatim

            commonSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n   Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AT_CONTROL1_PHYSYNCSEL,AIF2_AT_CONTROL1_RADSYNCSEL,AIF2_AT_CONTROL1_RP1MODE,AIF2_AT_CONTROL1_AUTORESYNC,
 *        AIF2_AT_CONTROL1_CRCUSE,AIF2_AT_CONTROL1_CRCFLIP,AIF2_AT_CONTROL1_CRCINIT_ONES,AIF2_AT_CONTROL1_CRCINVERT,
 *        AIF2_AT_CONTROL1_SYNC_SAMPL_WINDOW,AIF2_AT_CONTROL1_RP1RADT_FRAME_LOAD,AIF2_AT_CONTROL1_RP1PHYT_FRAME_LOAD,
 *        AIF2_AT_PHYT_CMP_RADSYNC_RP1RAD_TYPE_SELECT,AIF2_AT_RP1_TYPE_RP1RAD_TYPE_SELECT,
 *        AIF2_AT_PHYT_INIT_LSBS_PHYTCLOCK_COUNT_INIT;AIF2_AT_PHYT_INIT_MID_PHYTFRAME_INIT_LSBS,
 *        AIF2_AT_PHYT_INIT_MSBS_PHYTFRAME_INIT_MSBS,
 *        AIF2_AT_PHYT_TC_LSBS_PHYTCLOCK_COUNTER_TC,AIF2_AT_PHYT_FRAME_TC_LSBS_PHYTFRAME_TC_LSBS,
 *        AIF2_AT_PHYT_FRAME_TC_MSBS_PHYTFRAME_TC_MSBS,AIF2_AT_RADT_WCDMA_DIV_TERMINALCOUNT,
 *        AIF2_AT_RADT_INIT_LSBS_RADTCLOCK_COUNT_INIT,AIF2_AT_RADT_INIT_LSBS_RADTSYMBOL_COUNT_INIT,
 *        AIF2_AT_RADT_INIT_MID_RADTFRAME_INIT_LSBS,AIF2_AT_RADT_INIT_MSBS_RADTFRAME_INIT_MSBS,
 *        AIF2_AT_ULRADT_INIT_LSBS_ULRADTCLOCK_COUNT_INIT,AIF2_AT_ULRADT_INIT_LSBS_ULRADTSYMBOL_COUNT_INIT,AIF2_AT_ULRADT_INIT_LSBS_ULFCB_MINUSONE,
 *        AIF2_AT_ULRADT_INIT_MID_RADTFRAME_INIT_LSBS,AIF2_AT_ULRADT_INIT_MSBS_RADTFRAME_INIT_MSBS,
 *        AIF2_AT_DLRADT_INIT_LSBS_DLRADTCLOCK_COUNT_INIT,AIF2_AT_DLRADT_INIT_LSBS_DLRADTSYMBOL_COUNT_INIT,AIF2_AT_DLRADT_INIT_LSBS_DLFCB_MINUSONE,
 *        AIF2_AT_DLRADT_INIT_MID_RADTFRAME_INIT_LSBS,AIF2_AT_DLRADT_INIT_MSBS_RADTFRAME_INIT_MSBS,
 *        AIF2_AT_RADT_SYMB_LUT_INDEX_TC_LUTINDEX_TC,AIF2_AT_RADT_SYMB_LUT_INDEX_TC_SYMBOLTC,
 *        AIF2_AT_RADT_FRAME_TC_LSBS_RADTFRAME_TC_LSBS,AIF2_AT_RADT_FRAME_TC_MSBS_RADTFRAME_TC_MSBS,
 *        AIF2_AT_RADT_SYM_LUT_RAM_RADT_CLOCK_TC
 *        
 *   @b Example
 *   @verbatim
        Aif2Fl_setupAtCommonRegs (commonSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status  Aif2Fl_setupAtCommonRegs(
   Aif2Fl_Handle hAif2,
   Aif2Fl_CommonSetup *commonSetup
)
{
   Aif2Fl_AtCommonSetup *pAtCommonConfig;
   uint32_t tempReg;
   uint8_t count;

   pAtCommonConfig = commonSetup->pAtCommonSetup;
      
   /* AT Control 1 reg  setup */     
   tempReg =   CSL_FMK(AIF2_AT_CONTROL1_PHYSYNCSEL, pAtCommonConfig->PhySyncSel) |
                  CSL_FMK(AIF2_AT_CONTROL1_RADSYNCSEL, pAtCommonConfig->RadSyncSel) |
                  CSL_FMK(AIF2_AT_CONTROL1_RP1MODE,  pAtCommonConfig->SyncMode) |
                  CSL_FMK(AIF2_AT_CONTROL1_AUTORESYNC,  pAtCommonConfig->AutoResyncMode) |
                  CSL_FMK(AIF2_AT_CONTROL1_CRCUSE,  pAtCommonConfig->CrcMode) |
                  CSL_FMK(AIF2_AT_CONTROL1_CRCFLIP,  pAtCommonConfig->CrcFlip) |
                  CSL_FMK(AIF2_AT_CONTROL1_CRCINIT_ONES,  pAtCommonConfig->CrcInitOnes) |
                  CSL_FMK(AIF2_AT_CONTROL1_CRCINVERT,  pAtCommonConfig->CrcInvert) |
                  CSL_FMK(AIF2_AT_CONTROL1_SYNC_SAMPL_WINDOW,  pAtCommonConfig->SyncSampleWindow) |
                  CSL_FMK(AIF2_AT_CONTROL1_RP1RADT_FRAME_LOAD,  pAtCommonConfig->Rp1RadtFrameLoad) |
                  CSL_FMK(AIF2_AT_CONTROL1_RP1PHYT_FRAME_LOAD,  pAtCommonConfig->Rp1PhytFrameLoad) ;
   
   hAif2->regs->AT_CONTROL1 = tempReg;

   // phy timer compare value setup. this field only valid when rad timer sync AIF2FL_PHYT_CMP_SYNC is selected
   CSL_FINS(hAif2->regs->AT_PHYT_CMP_RADSYNC, AIF2_AT_PHYT_CMP_RADSYNC_RP1RAD_TYPE_SELECT, pAtCommonConfig->PhytCompValue);

   //RP1 type select
   CSL_FINS(hAif2->regs->AT_RP1_TYPE, AIF2_AT_RP1_TYPE_RP1RAD_TYPE_SELECT, pAtCommonConfig->Rp1Type);
   	
   // Phy timer init value setup
   CSL_FINS(hAif2->regs->AT_PHYT_INIT_LSBS, AIF2_AT_PHYT_INIT_LSBS_PHYTCLOCK_COUNT_INIT, pAtCommonConfig->AtInit.pPhyTimerInit->ClockNum);
   CSL_FINS(hAif2->regs->AT_PHYT_INIT_MID, AIF2_AT_PHYT_INIT_MID_PHYTFRAME_INIT_LSBS, pAtCommonConfig->AtInit.pPhyTimerInit->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_PHYT_INIT_MSBS, AIF2_AT_PHYT_INIT_MSBS_PHYTFRAME_INIT_MSBS, pAtCommonConfig->AtInit.pPhyTimerInit->FrameMsbNum);

   // Phy timer terminal count value setup
   CSL_FINS(hAif2->regs->AT_PHYT_TC_LSBS, AIF2_AT_PHYT_TC_LSBS_PHYTCLOCK_COUNTER_TC, pAtCommonConfig->AtTerminalCount.pPhyTimerTc->ClockNum);
   CSL_FINS(hAif2->regs->AT_PHYT_FRAME_TC_LSBS, AIF2_AT_PHYT_FRAME_TC_LSBS_PHYTFRAME_TC_LSBS, pAtCommonConfig->AtTerminalCount.pPhyTimerTc->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_PHYT_FRAME_TC_MSBS, AIF2_AT_PHYT_FRAME_TC_MSBS_PHYTFRAME_TC_MSBS, pAtCommonConfig->AtTerminalCount.pPhyTimerTc->FrameMsbNum);

   //Divide Terminal count value for WCDMA counter
   CSL_FINS(hAif2->regs->AT_RADT_WCDMA_DIV, AIF2_AT_RADT_WCDMA_DIV_TERMINALCOUNT, pAtCommonConfig->WcdmaDivTC);

   // Rad timer init value setup
   tempReg =   CSL_FMK(AIF2_AT_RADT_INIT_LSBS_RADTCLOCK_COUNT_INIT, pAtCommonConfig->AtInit.pRadTimerInit->ClockNum) |
                      CSL_FMK(AIF2_AT_RADT_INIT_LSBS_RADTSYMBOL_COUNT_INIT,  pAtCommonConfig->AtInit.pRadTimerInit->SymbolNum);
   hAif2->regs->AT_RADT_INIT_LSBS = tempReg;
   
   CSL_FINS(hAif2->regs->AT_RADT_INIT_MID, AIF2_AT_RADT_INIT_MID_RADTFRAME_INIT_LSBS, pAtCommonConfig->AtInit.pRadTimerInit->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_RADT_INIT_MSBS, AIF2_AT_RADT_INIT_MSBS_RADTFRAME_INIT_MSBS, pAtCommonConfig->AtInit.pRadTimerInit->FrameMsbNum);

   if(pAtCommonConfig->AtInit.pUlRadTimerInit != NULL){
   // Ul Rad timer clock and symbol init value and frame count minus one valu setup
   tempReg =   CSL_FMK(AIF2_AT_ULRADT_INIT_LSBS_ULRADTCLOCK_COUNT_INIT, pAtCommonConfig->AtInit.pUlRadTimerInit->ClockNum) |
                      CSL_FMK(AIF2_AT_ULRADT_INIT_LSBS_ULRADTSYMBOL_COUNT_INIT, pAtCommonConfig->AtInit.pUlRadTimerInit->SymbolNum) |
                      CSL_FMK(AIF2_AT_ULRADT_INIT_LSBS_ULFCB_MINUSONE,  pAtCommonConfig->AtInit.pUlRadTimerInit->FcbMinusOne);
   hAif2->regs->AT_ULRADT_INIT_LSBS = tempReg;

   CSL_FINS(hAif2->regs->AT_ULRADT_INIT_MID, AIF2_AT_ULRADT_INIT_MID_RADTFRAME_INIT_LSBS, pAtCommonConfig->AtInit.pUlRadTimerInit->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_ULRADT_INIT_MSBS, AIF2_AT_ULRADT_INIT_MSBS_RADTFRAME_INIT_MSBS, pAtCommonConfig->AtInit.pUlRadTimerInit->FrameMsbNum);
   }

   if(pAtCommonConfig->AtInit.pDlRadTimerInit != NULL){
    // Dl Rad timer clock and symbol init value and frame count minus one valu setup
   tempReg =   CSL_FMK(AIF2_AT_DLRADT_INIT_LSBS_DLRADTCLOCK_COUNT_INIT, pAtCommonConfig->AtInit.pDlRadTimerInit->ClockNum) |
                      CSL_FMK(AIF2_AT_DLRADT_INIT_LSBS_DLRADTSYMBOL_COUNT_INIT, pAtCommonConfig->AtInit.pDlRadTimerInit->SymbolNum) |
                      CSL_FMK(AIF2_AT_DLRADT_INIT_LSBS_DLFCB_MINUSONE,  pAtCommonConfig->AtInit.pDlRadTimerInit->FcbMinusOne);
   hAif2->regs->AT_DLRADT_INIT_LSBS = tempReg;

   CSL_FINS(hAif2->regs->AT_DLRADT_INIT_MID, AIF2_AT_DLRADT_INIT_MID_RADTFRAME_INIT_LSBS, pAtCommonConfig->AtInit.pDlRadTimerInit->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_DLRADT_INIT_MSBS, AIF2_AT_DLRADT_INIT_MSBS_RADTFRAME_INIT_MSBS, pAtCommonConfig->AtInit.pDlRadTimerInit->FrameMsbNum);
   }
  
  // Rad timer symbol and lut index terminal count value setup
   tempReg =   CSL_FMK(AIF2_AT_RADT_SYMB_LUT_INDEX_TC_LUTINDEX_TC, pAtCommonConfig->AtTerminalCount.pRadTimerTc->LutIndexNum) |
                      CSL_FMK(AIF2_AT_RADT_SYMB_LUT_INDEX_TC_SYMBOLTC,  pAtCommonConfig->AtTerminalCount.pRadTimerTc->SymbolNum);
   hAif2->regs->AT_RADT_SYMB_LUT_INDEX_TC = tempReg;

   // Rad timer frame terminal count setup 
   CSL_FINS(hAif2->regs->AT_RADT_FRAME_TC_LSBS, AIF2_AT_RADT_FRAME_TC_LSBS_RADTFRAME_TC_LSBS, pAtCommonConfig->AtTerminalCount.pRadTimerTc->FrameLsbNum);
   CSL_FINS(hAif2->regs->AT_RADT_FRAME_TC_MSBS, AIF2_AT_RADT_FRAME_TC_MSBS_RADTFRAME_TC_MSBS, pAtCommonConfig->AtTerminalCount.pRadTimerTc->FrameMsbNum);

    // Rad, timer symbol lut ram to setup clock terminal count 
   for (count=0; count < (pAtCommonConfig->AtTerminalCount.pRadTimerTc->LutIndexNum +1); count++)
   {
         tempReg = CSL_FMK(AIF2_AT_RADT_SYM_LUT_RAM_RADT_CLOCK_TC, pAtCommonConfig->AtTerminalCount.RadClockCountTc[count]);
         hAif2->regs->AT_RADT_SYM_LUT_RAM[count] = tempReg;
   }
   
   return  AIF2FL_SOK ;
 
}

 /** ============================================================================
 *   @n@b Aif2Fl_setupAtEventRegs
 *
 *   @b Description
 *   @n AIF2 AT Internal and External Event setup
 *
 *   @b Arguments
 *   @verbatim

            commonSetup   Instance containing "Setup" properties for AIF2. 
            hAif2    Handle to the aif2 instance
            
     @endverbatim
 *
 *   <b> Return Value </b>  Aif2Fl_Status
 *
 *   <b> Pre Condition </b>
 *   @n   Aif2Fl_init(), Aif2Fl_open()
 *
 *   <b> Post Condition </b>
 *   @n None
 *
 *   @b Writes
 *   @n AIF2_AT_EVENT_OFFSET_EVENTINDEX,AIF2_AT_EVENT_OFFSET_STROBESELECT,
 *        AIF2_AT_EVENT_MOD_TC_EVENTMODULO,AIF2_AT_EVENT_MASK_LSBS_EVENTMASK_LSBS,AIF2_AT_EVENT_MASK_MSBS_EVENTMASK_MSBS,
 *        AIF2_AT_EVT_ENABLE_EVENTENABLE,
 *        AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX,AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT,
 *        AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE,
 *        AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX,AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,
 *        AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *
 *   @b Reads
 *   @n AIF2_AT_EVT_ENABLE_EVENTENABLE,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE,
 *        AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE
 *
 *   @b Example
 *   @verbatim
        Aif2Fl_setupAtEventRegs (commonSetup, hAif2);
     @endverbatim
 * ===========================================================================
 */
static inline Aif2Fl_Status  Aif2Fl_setupAtEventRegs(
   Aif2Fl_Handle hAif2,
   Aif2Fl_CommonSetup *commonSetup
)
{
   Aif2Fl_AtEventSetup *pAtEventConfig;
   uint32_t tempReg, i;

   pAtEventConfig = commonSetup->pAtEventSetup;

#ifdef K2
   for(i=0; i<24; i++){
#else
   for(i=0; i<11; i++){
#endif
   	
   	if(pAtEventConfig->bEnableRadEvent[i] == true){
		
	   //Rad timer external event strobe select and offset value setup
	   tempReg =   CSL_FMK(AIF2_AT_EVENT_OFFSET_EVENTINDEX, pAtEventConfig->AtRadEvent[i].EventOffset) |
	                      CSL_FMK(AIF2_AT_EVENT_OFFSET_STROBESELECT,  pAtEventConfig->AtRadEvent[i].EvtStrobeSel);
	   hAif2->regs->AT_EVENTS[i].AT_EVENT_OFFSET = tempReg;

	   //Rad timer event modulus value and GSM mask setup
	   CSL_FINS(hAif2->regs->AT_EVENTS[i].AT_EVENT_MOD_TC, AIF2_AT_EVENT_MOD_TC_EVENTMODULO, pAtEventConfig->AtRadEvent[i].EventModulo);
	   
	   CSL_FINS(hAif2->regs->AT_EVENTS[i].AT_EVENT_MASK_LSBS, AIF2_AT_EVENT_MASK_LSBS_EVENTMASK_LSBS, 
	   	            pAtEventConfig->AtRadEvent[i].EventMaskLsb);
	   CSL_FINS(hAif2->regs->AT_EVENTS[i].AT_EVENT_MASK_MSBS, AIF2_AT_EVENT_MASK_MSBS_EVENTMASK_MSBS, 
	   	            pAtEventConfig->AtRadEvent[i].EventMaskMsb);
	   
          //Enable event number i
	   tempReg = CSL_FEXT(hAif2->regs->AT_EVT_ENABLE, AIF2_AT_EVT_ENABLE_EVENTENABLE);
          tempReg |= (uint32_t)(0x1 << i);
	   hAif2->regs->AT_EVT_ENABLE = tempReg;
   	}
	
   }

   for(i=0; i<3; i++){
   	
   	if(pAtEventConfig->bEnableIngrDioEvent[i] == true){
		
	   //Ingress DIO frame event strobe select and offset value setup
	   tempReg =   CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX, pAtEventConfig->AtIngrDioEvent[i].DioFrameEventOffset) |
	                      CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT,  pAtEventConfig->AtIngrDioEvent[i].DioFrameStrobeSel);
	   hAif2->regs->AT_AD_INGR_EVENTS[i+3].AT_AD_INGR_EVENT_OFFSET = tempReg;

	   //Ingress DIO event strobe select and offset value setup
	   tempReg =   CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_EVENTINDEX, pAtEventConfig->AtIngrDioEvent[i].EventOffset) |
	                      CSL_FMK(AIF2_AT_AD_INGR_EVENT_OFFSET_STROBESELECT,  pAtEventConfig->AtIngrDioEvent[i].EvtStrobeSel);
	   hAif2->regs->AT_AD_INGR_EVENTS[i].AT_AD_INGR_EVENT_OFFSET = tempReg;

	   //Ingress DIO event  modulus value setup
	   CSL_FINS(hAif2->regs->AT_AD_INGR_EVENTS[i+3].AT_AD_INGR_EVENT_MOD_TC, AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO, 
	                   3071999);
	   CSL_FINS(hAif2->regs->AT_AD_INGR_EVENTS[i].AT_AD_INGR_EVENT_MOD_TC, AIF2_AT_AD_INGR_EVENT_MOD_TC_EVENTMODULO, 
	   	            pAtEventConfig->AtIngrDioEvent[i].EventModulo);
	   
          //Enable ingress DIO event and additional frame event
	   tempReg = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE);
          tempReg |= (uint32_t)(0x9 << i);
	   CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADINGR_EVENT_ENABLE, tempReg);

   	}
   }
	
    for(i=0; i<3; i++){
   	
   	if(pAtEventConfig->bEnableEgrDioEvent[i] == true){
		
	   //Egress DIO frame event strobe select and offset value setup
	   tempReg =   CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX, pAtEventConfig->AtEgrDioEvent[i].DioFrameEventOffset) |
	                      CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,  pAtEventConfig->AtEgrDioEvent[i].DioFrameStrobeSel);
	   hAif2->regs->AT_AD_EGR_EVENTS[i+3].AT_AD_EGR_EVENT_OFFSET = tempReg;

	   //Egress DIO event strobe select and offset value setup
	   tempReg =   CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_EVENTINDEX, pAtEventConfig->AtEgrDioEvent[i].EventOffset) |
	                      CSL_FMK(AIF2_AT_AD_EGR_EVENT_OFFSET_STROBESELECT,  pAtEventConfig->AtEgrDioEvent[i].EvtStrobeSel);
	   hAif2->regs->AT_AD_EGR_EVENTS[i].AT_AD_EGR_EVENT_OFFSET = tempReg;

	   //Egress DIO event  modulus value setup
	   CSL_FINS(hAif2->regs->AT_AD_EGR_EVENTS[i+3].AT_AD_EGR_EVENT_MOD_TC, AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO, 
	                   3071999);
	   CSL_FINS(hAif2->regs->AT_AD_EGR_EVENTS[i].AT_AD_EGR_EVENT_MOD_TC, AIF2_AT_AD_EGR_EVENT_MOD_TC_EVENTMODULO,
	                   pAtEventConfig->AtEgrDioEvent[i].EventModulo);
          //Enable ingress DIO event and additional frame event
	   tempReg = CSL_FEXT(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE);
          tempReg |= (uint32_t)(0x9 << i);
	   CSL_FINS(hAif2->regs->AT_INTERNAL_EVT_ENABLE, AIF2_AT_INTERNAL_EVT_ENABLE_ADEGR_EVENT_ENABLE, tempReg);

   	}
   }

   return(AIF2FL_SOK);
 }
 

#ifdef __cplusplus
}
#endif
#endif /* Aif2Fl_hwSetupAUX_H_ */


