/****************************************************************************\
 *           (C) Copyright 2009, Texas Instruments, Inc.                    *
 *                                                                          *
 *  Redistribution and use in source and binary forms, with or without      *
 *  modification, are permitted provided that the following conditions      *
 *  are met:                                                                *
 *                                                                          *
 *    Redistributions of source code must retain the above copyright        *
 *    notice, this list of conditions and the following disclaimer.         *
 *                                                                          *
 *    Redistributions in binary form must reproduce the above copyright     *
 *    notice, this list of conditions and the following disclaimer in the   *
 *    documentation and/or other materials provided with the                *
 *    distribution.                                                         *
 *                                                                          *
 *    Neither the name of Texas Instruments Incorporated nor the names of   *
 *    its contributors may be used to endorse or promote products derived   *
 *    from this software without specific prior written permission.         *
 *                                                                          *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS     *
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT       *
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   *
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT    *
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   *
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT        *
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   *
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   *
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT     *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   *
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    *
 ****************************************************************************
 *                                                                          *
 * Target processors : TMS320C66xx                                          *
 *                                                                          *
\****************************************************************************/

#include <string.h>
#include <stdio.h>

#include <ti/csl/csl.h>
#include <ti/drv/aif2/aif2fl_getHwStatusAux.h>

#include <ti/drv/aif2/AIF_defs.h>
#include <ti/drv/aif2/AIF_cfg.h>
#include <ti/drv/aif2/AIF_init.h>
#include <ti/drv/aif2/AIF_init_dat.h>
#include <ti/drv/aif2/aif2_osal.h>

#define __AIF_SHUTDOWN_C
#include <ti/drv/aif2/AIF_shutdown.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_resetFsync, ".text:aifDriver");
#pragma CODE_SECTION(AIF_resetAif, ".text:aifDriver");
#pragma CODE_SECTION(AIF_cleanMMRs, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disablePeCh, ".text:aifDriver");
#pragma CODE_SECTION(AIF_disablePdCh, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enablePeCh, ".text:aifDriver");
#pragma CODE_SECTION(AIF_enablePdCh, ".text:aifDriver");
#endif

static Aif2Fl_Obj                 Aif2Obj;

#ifndef _TMS320C6X
extern AIF_InitCfg                *pGlobalAif2Cfg;
#endif

void AIF_cleanMMRs(AIF_ConfigHandle  hAif);

void
AIF_resetFsync(
    AIF_ConfigHandle    hAif
)
{
	Aif2Fl_Status status;
	uint16_t ctrlArg = true;

	//volatile uint32_t  sampelPhyTclockCounter;

	//int32_t i;
	if (aifFsyncInitDone == 0) {
#ifndef _TMS320C6X
        pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
		/* Initialize CSL library, this step is required */
	    hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
    }
	//AT disable all events	
	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_DISABLE_ALL_EVENTS, (void *)&ctrlArg);
	//stop aif2 AT timer
   	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_HALT_TIMER, (void *)&ctrlArg);	
   	// Check AT timer is actually stopped - debug code only
   	/*sampelPhyTclockCounter = Aif2Fl_getAtPhytClkCount(hAif->hFl);
   	{
   	    volatile uint32_t i, n;
   	    n = 0;
   	    for (i = 0; i < 10000; i++)
   	    {
   	        n = n + 1;
   	    }
   	}
   	if (sampelPhyTclockCounter != Aif2Fl_getAtPhytClkCount(hAif->hFl)) {
   		Aif2_osalLog("Error: AIF2 AT timer didn't stop\n");
   	}*/

   	if (hAif->hFl != NULL) {Aif2Fl_close(hAif->hFl); hAif->hFl = NULL;}
}


// This function is used to reset AIF to its default state.
void AIF_resetAif(
    AIF_ConfigHandle    hAif
)
{
	int32_t i;
	Aif2Fl_Status status;
	uint16_t ctrlArg = false;
	//CSL_Cppidma_rx_channel_configRegs* pktDMARxCfg = (CSL_Cppidma_rx_channel_configRegs*)AIF2FL_CFG_CPPI_DMA_RX_CFG_REGS;
	//CSL_Cppidma_tx_channel_configRegs* pktDMATxCfg = (CSL_Cppidma_tx_channel_configRegs*)AIF2FL_CFG_CPPI_DMA_TX_CFG_REGS;

	if (aifInitDone == 0) {
#ifndef _TMS320C6X
        pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
		/* Initialize CSL library, this step is required */
	    hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
	}	
	

	//AT disable all events and halt timer
    ctrlArg = true;
	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_DISABLE_ALL_EVENTS, (void *)&ctrlArg);
    Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AT_HALT_TIMER, (void *)&ctrlArg);	
	// Stop  the DIO engines
	ctrlArg = false;
	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AD_E_ENABLE_DISABLE_DIO_GLOBAL, (void *)&ctrlArg);
    Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_AD_IN_ENABLE_DISABLE_DIO_GLOBAL, (void *)&ctrlArg);
	
	for(i=0; i<AIF_MAX_NUM_LINKS; i++) 	//for 6 links
	{
		hAif->hFl->arg_link = (Aif2Fl_LinkIndex)i;
		ctrlArg = false;
     	Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_TX_LINK, (void *)&ctrlArg);
        Aif2Fl_hwControl(hAif->hFl, AIF2FL_CMD_ENABLE_DISABLE_RX_LINK, (void *)&ctrlArg);
            
	}
#ifdef _TMS320C6X
	// Disable all channels
	for(i=0; i<124; i++)
	{
	    if (hAif->pktDmaConfig.txChAxC[i]) {Cppi_channelDisable (hAif->pktDmaConfig.txChAxC[i]); Cppi_channelClose (hAif->pktDmaConfig.txChAxC[i]); hAif->pktDmaConfig.txChAxC[i] = NULL;}
		if (hAif->pktDmaConfig.rxChAxC[i]) {Cppi_channelDisable (hAif->pktDmaConfig.rxChAxC[i]); Cppi_channelClose (hAif->pktDmaConfig.rxChAxC[i]); hAif->pktDmaConfig.rxChAxC[i] = NULL;}
		//CSL_FINS(pktDMATxCfg->TX_CHANNEL_GLOBAL_CONFIG[i].TX_CHANNEL_GLOBAL_CONFIG_REG_A, CPPIDMA_TX_CHANNEL_CONFIG_TX_CHANNEL_GLOBAL_CONFIG_REG_A_TX_ENABLE, (uint32_t)0);	
		//CSL_FINS(pktDMARxCfg->RX_CHANNEL_GLOBAL_CONFIG[i].RX_CHANNEL_GLOBAL_CONFIG_REG, CPPIDMA_RX_CHANNEL_CONFIG_RX_CHANNEL_GLOBAL_CONFIG_REG_RX_ENABLE, (uint32_t)0);
	}
	for(i=0; i<3; i++)
	{
	    if (hAif->pktDmaConfig.txChCtrl[i]) {Cppi_channelDisable (hAif->pktDmaConfig.txChCtrl[i]); Cppi_channelClose (hAif->pktDmaConfig.txChCtrl[i]); hAif->pktDmaConfig.txChCtrl[i] = NULL;}
		if (hAif->pktDmaConfig.rxChCtrl[i]) {Cppi_channelDisable (hAif->pktDmaConfig.rxChCtrl[i]); Cppi_channelClose (hAif->pktDmaConfig.rxChCtrl[i]); hAif->pktDmaConfig.rxChCtrl[i] = NULL;}
	}

    if (hAif->pktDmaConfig.dioTxChAxC) {Cppi_channelDisable (hAif->pktDmaConfig.dioTxChAxC); Cppi_channelClose (hAif->pktDmaConfig.dioTxChAxC); hAif->pktDmaConfig.dioTxChAxC = NULL;}
	if (hAif->pktDmaConfig.dioRxChAxC) {Cppi_channelDisable (hAif->pktDmaConfig.dioRxChAxC); Cppi_channelClose (hAif->pktDmaConfig.dioRxChAxC); hAif->pktDmaConfig.dioRxChAxC = NULL;}
	
	if (hAif->pktDmaConfig.hCppi) {Cppi_close (hAif->pktDmaConfig.hCppi); hAif->pktDmaConfig.hCppi = NULL;}
#endif
	
	Aif2Fl_reset(hAif->hFl);

	if (hAif->hAif2Setup != NULL) AIF_cleanMMRs(hAif);
		
   	if (hAif->hFl != NULL) {Aif2Fl_close(hAif->hFl); hAif->hFl = NULL; aifFsyncInitDone = 0;aifInitDone = 0;}

}


void AIF_cleanMMRs(
		AIF_ConfigHandle  hAif
)
{
  int32_t  	i;

   //////////////////Initialize Aif2 structures to avoid unwanted configuration ////////////////////////////////////////

   if (hAif->hAif2Setup->commonSetup->pSdCommonSetup != NULL) memset(hAif->hAif2Setup->commonSetup->pSdCommonSetup, 0, sizeof(Aif2Fl_SdCommonSetup));
   memset(hAif->hAif2Setup->commonSetup->pPdCommonSetup, 0, sizeof(Aif2Fl_PdCommonSetup));
   memset(hAif->hAif2Setup->commonSetup->pPeCommonSetup, 0, sizeof(Aif2Fl_PeCommonSetup));
   memset(hAif->hAif2Setup->commonSetup->pIngrDbSetup, 0, sizeof(Aif2Fl_IngrDbSetup));
   memset(hAif->hAif2Setup->commonSetup->pEgrDbSetup, 0, sizeof(Aif2Fl_EgrDbSetup));
   memset(hAif->hAif2Setup->commonSetup->pAdCommonSetup, 0, sizeof(Aif2Fl_AdCommonSetup));
   memset(hAif->hAif2Setup->commonSetup->pAdDioSetup, 0, sizeof(Aif2Fl_AdDioSetup));
   memset(hAif->hAif2Setup->commonSetup->pAtEventSetup, 0, sizeof(Aif2Fl_AtEventSetup));
   memset(hAif->hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pPhyTimerInit, 0, sizeof(Aif2Fl_AtCountObj));
   memset(hAif->hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pRadTimerInit, 0, sizeof(Aif2Fl_AtCountObj));
   memset(hAif->hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pUlRadTimerInit, 0, sizeof(Aif2Fl_AtCountObj));
   memset(hAif->hAif2Setup->commonSetup->pAtCommonSetup->AtTerminalCount.pPhyTimerTc, 0, sizeof(Aif2Fl_AtCountObj));
   memset(hAif->hAif2Setup->commonSetup->pAtCommonSetup->AtTerminalCount.pRadTimerTc, 0, sizeof(Aif2Fl_AtCountObj));

   for(i=0; i< AIF_MAX_NUM_LINKS; i++) {
	   memset(hAif->hAif2Setup->linkSetup[i]->pComLinkSetup, 0, sizeof(Aif2Fl_CommonLinkSetup));
	   if (hAif->hAif2Setup->linkSetup[i]->pSdLinkSetup != NULL) memset(hAif->hAif2Setup->linkSetup[i]->pSdLinkSetup,  0, sizeof(Aif2Fl_SdLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pRmLinkSetup,  0, sizeof(Aif2Fl_RmLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pTmLinkSetup,  0, sizeof(Aif2Fl_TmLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pPdLinkSetup,  0, sizeof(Aif2Fl_PdLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pPeLinkSetup,  0, sizeof(Aif2Fl_PeLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pRtLinkSetup,  0, sizeof(Aif2Fl_RtLinkSetup));
	   memset(hAif->hAif2Setup->linkSetup[i]->pAtLinkSetup,  0, sizeof(Aif2Fl_AtLinkSetup));
   }

   /* Clean all MMRs */
   Aif2Fl_hwSetup(hAif->hFl, hAif->hAif2Setup);
   hAif->hAif2Setup = 0;

}


// This function is used to disable a given PE channel.
void AIF_disablePeCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          peChanNum
)
{
	Aif2Fl_Status status;
      if (aifInitDone == 0) {
#ifndef _TMS320C6X
          pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
            /* Initialize CSL library, this step is required */
          hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
      }
      /* disable PE dma channel  */
      CSL_FINS(hAif->hFl->regs->PE_DMACHAN_EN[peChanNum], AIF2_PE_DMACHAN_EN_CH_EN, 0);
}

// This function is used to disable a given PD channel.
void AIF_disablePdCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          pdChanNum
)
{
	Aif2Fl_Status status;
      if (aifInitDone == 0) {
#ifndef _TMS320C6X
          pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
            /* Initialize CSL library, this step is required */
          hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
      }
      /* disable PD dma channel  */
      CSL_FINS(hAif->hFl->regs->PD_DMACHAN[pdChanNum], AIF2_PD_DMACHAN_CHAN_EN, 0);
}

// This function is used to disable a given PE channel.
void AIF_enablePeCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          peChanNum
)
{
	Aif2Fl_Status status;
      if (aifInitDone == 0) {
#ifndef _TMS320C6X
          pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
            /* Initialize CSL library, this step is required */
          hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
      }
      /* Enable PE dma channel  */
      CSL_FINS(hAif->hFl->regs->PE_DMACHAN_EN[peChanNum], AIF2_PE_DMACHAN_EN_CH_EN, 1);
}

// This function is used to disable a given PD channel.
void AIF_enablePdCh(
    AIF_ConfigHandle                hAif,
    uint32_t                          pdChanNum
)
{
	Aif2Fl_Status status;
      if (aifInitDone == 0) {
#ifndef _TMS320C6X
          pGlobalAif2Cfg = hAif->hAif2SerDesBaseAddr;
#endif
            /* Initialize CSL library, this step is required */
          hAif->hFl = Aif2Fl_open(&Aif2Obj, 0, NULL, &status);
      }
      /* Enable PD dma channel  */
      CSL_FINS(hAif->hFl->regs->PD_DMACHAN[pdChanNum], AIF2_PD_DMACHAN_CHAN_EN, 1);
}



////////////////////
