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
#include <stdint.h>

#include <ti/csl/csl.h>
#ifdef K2
#include <ti/csl/csl_serdes_aif2.h>
#endif

#define __AIF_HIBERNATE_C
#include <ti/drv/aif2/AIF_hibernation.h>

/**********************************************************************
 ************************** LOCAL Definitions *************************
 **********************************************************************/
#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_hwContextSaveInit, ".text:aifDriver");
#pragma CODE_SECTION(AIF_hwContextSave, ".text:aifDriver");
#pragma CODE_SECTION(AIF_hwContextRestore, ".text:aifDriver");
#ifdef K2
#pragma CODE_SECTION(AIF_serdesRestore, ".text:aifDriver");
#endif

#pragma DATA_ALIGN  (aif2Structs, 128)
#pragma DATA_SECTION(aif2Structs, ".far:aifDriver");
#endif

AIF_HwContextSaveData aif2Structs;

/******************************/
/***** internal functions *****/
/******************************/
// NONE

/************************/
/***** Primary API: *****/
/************************/

/* This function initializes the pointer to CSL structure required to save Aif2 configuration.
 * Must be called prior to any prior to Aif2_hibernate_save(). */
void AIF_hwContextSaveInit(AIF_HwContextSaveInfo *save_list)
{
	aif2Structs.info.save_aif2_object_pointer = save_list->save_aif2_object_pointer;
	aif2Structs.info.save_aif2_struct_pointer = save_list->save_aif2_struct_pointer;
}


/* This function causes the entire state of Aif2 to be saved prior to hibernation.
 * Assumptions made by this function:
 */
void AIF_hwContextSave(void)
{
	Aif2Fl_Setup         *hAif2Setup = aif2Structs.info.save_aif2_struct_pointer;
	Aif2Fl_Obj           *hAif2Csl   = aif2Structs.info.save_aif2_object_pointer;
	Aif2Fl_AtEventIndex   atEvtIdx;

	// AIF2 reset isolation enabled or disabled
	/*
	 * Prior to hibernation, need to shutdown DMA first. So disable DIO engines before shutdown Cores. AIF2 Bypass just means disabling DIO
	 * for WCDMA and disabling PktDMA channel for LTE to cut real incoming frame data and re-transmit it to the next DSP in the chain.
	 * For Wcdma, 2 ways to proceed:
	 * 	- use dma_ch_en field in AD to disable each DIO engine ( and re-enable it when comes back)
	 * 	- OR disable internal AT event for DIO instead of disabling DIO engine itself.
	 */
	for (atEvtIdx = AIF2FL_EVENT_0; atEvtIdx <= AIF2FL_EVENT_10; atEvtIdx++ ) {
		if (hAif2Setup->commonSetup->pAtEventSetup->bEnableRadEvent[atEvtIdx] == true) {
			Aif2Fl_atDisableEvent(hAif2Csl,hAif2Setup->commonSetup->pAtEventSetup->AtRadEvent[atEvtIdx].EventSelect);
		}
	}

	for (atEvtIdx = AIF2FL_IN_DIO_EVENT_0; atEvtIdx <= AIF2FL_IN_DIO_EVENT_2; atEvtIdx++ ) {
		if (hAif2Setup->commonSetup->pAtEventSetup->bEnableIngrDioEvent[atEvtIdx-AIF2FL_IN_DIO_EVENT_0] == true) {
			Aif2Fl_atDisableEvent(hAif2Csl,hAif2Setup->commonSetup->pAtEventSetup->AtIngrDioEvent[atEvtIdx-AIF2FL_IN_DIO_EVENT_0].EventSelect);
		}
	}

	for (atEvtIdx = AIF2FL_E_DIO_EVENT_0; atEvtIdx <= AIF2FL_E_DIO_EVENT_2; atEvtIdx++ ) {
		if (hAif2Setup->commonSetup->pAtEventSetup->bEnableEgrDioEvent[atEvtIdx-AIF2FL_E_DIO_EVENT_0] == true) {
			Aif2Fl_atDisableEvent(hAif2Csl,hAif2Setup->commonSetup->pAtEventSetup->AtEgrDioEvent[atEvtIdx-AIF2FL_E_DIO_EVENT_0].EventSelect);
		}
	}

}


#ifdef K2
static void AIF_serdesRestore(Aif2Fl_Setup *hAif2Setup, uint8_t enableB8, uint8_t enableB4);
#endif

/* This function causes the entire state of Aif2 to be restored following
 * hibernation. */
void AIF_hwContextRestore(uint32_t currentBfn, uint32_t aif2_rsiso)
{
	Aif2Fl_Setup         *hAif2Setup = aif2Structs.info.save_aif2_struct_pointer;
	Aif2Fl_Obj           *hAif2Csl   = aif2Structs.info.save_aif2_object_pointer;
	Aif2Fl_AtCommonSetup *hAtCommonSetup;
	Aif2Fl_SdCommonSetup *hSdCommonSetup;
	Aif2Fl_SdLinkSetup   *hSdLinkSetup[6];
    Aif2Fl_RmLinkSetup   *hRmLinkSetup[6];
    Aif2Fl_TmLinkSetup   *hTmLinkSetup[6];
    Aif2Fl_RtLinkSetup   *hRtLinkSetup[6];
	uint16_t   ctrlArg;
	int32_t  i;
	uint8_t  serdes_blockb8_used =0;
	uint8_t  serdes_blockb4_used =0;
#ifndef K2
	uint16_t response =0;
#endif

	// Check AIF2's reset isolation configuration
	if (aif2_rsiso)
	{
	    /* Ignoring recovery of AT common / SD */
	    hAtCommonSetup = hAif2Setup->commonSetup->pAtCommonSetup;
	    hAif2Setup->commonSetup->pAtCommonSetup = NULL;
	    hSdCommonSetup = hAif2Setup->commonSetup->pSdCommonSetup;
	    hAif2Setup->commonSetup->pSdCommonSetup = NULL;

	    for(i = 0; i < 6; i++) {
		 if(hAif2Setup->globalSetup->ActiveLink[i] == true) {
			 hSdLinkSetup[i] = hAif2Setup->linkSetup[i]->pSdLinkSetup;
			 hAif2Setup->linkSetup[i]->pSdLinkSetup = NULL;
			 hTmLinkSetup[i] = hAif2Setup->linkSetup[i]->pTmLinkSetup;
			 hAif2Setup->linkSetup[i]->pTmLinkSetup = NULL;
			 hRmLinkSetup[i] = hAif2Setup->linkSetup[i]->pRmLinkSetup;
			 hAif2Setup->linkSetup[i]->pRmLinkSetup = NULL;
			 hRtLinkSetup[i] = hAif2Setup->linkSetup[i]->pRtLinkSetup;
			 hAif2Setup->linkSetup[i]->pRtLinkSetup = NULL;
		 }
	    }
	    /* Recovering from hibernation */
	    Aif2Fl_hwSetup(hAif2Csl, hAif2Setup);
	    /* restoring pointers for modules not needing recovery */
	    hAif2Setup->commonSetup->pAtCommonSetup = hAtCommonSetup;
	    hAif2Setup->commonSetup->pSdCommonSetup = hSdCommonSetup;
	    for(i = 0; i < 6; i++) {
		 if(hAif2Setup->globalSetup->ActiveLink[i] == true) {
			 hAif2Setup->linkSetup[i]->pSdLinkSetup = hSdLinkSetup[i];
			 hAif2Setup->linkSetup[i]->pTmLinkSetup = hTmLinkSetup[i];
			 hAif2Setup->linkSetup[i]->pRmLinkSetup = hRmLinkSetup[i];
			 hAif2Setup->linkSetup[i]->pRtLinkSetup = hRtLinkSetup[i];
		 }
	    }
	} else {
		// Restore AIF2 state from CSL configuration structure and restart AIF2

		/* Assume PSC_restoreState was already called by the master core -  Power domain to ON and clocks too */
	    /*CSL_PSC_enablePowerDomain (CSL_PSC_PD_AI);
	    CSL_PSC_setModuleNextState (CSL_PSC_LPSC_AI, PSC_MODSTATE_ENABLE);
	    CSL_PSC_startStateTransition (CSL_PSC_PD_AI);
	    while (!CSL_PSC_isStateTransitionDone (CSL_PSC_PD_AI));*/

	    Aif2Fl_reset(hAif2Csl);

	    // update current BFN
	    hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pPhyTimerInit->FrameLsbNum       = currentBfn;
	    hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pRadTimerInit->FrameLsbNum       = currentBfn;
	    if (currentBfn > 0) {
	    	hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pUlRadTimerInit->FrameLsbNum = currentBfn - 1;
	    } else {
	    	hAif2Setup->commonSetup->pAtCommonSetup->AtInit.pUlRadTimerInit->FrameLsbNum = 4095;
	    }

	    // Identify which SerDes block to use
		for(i=AIF2FL_LINK_0; i<AIF2FL_LINK_4; i++)       {if(hAif2Setup->globalSetup->ActiveLink[i] == true) serdes_blockb8_used=1;}
		for(i=AIF2FL_LINK_4; i<(AIF2FL_LINK_5 + 1); i++) {if(hAif2Setup->globalSetup->ActiveLink[i] == true) serdes_blockb4_used=1;}

#ifdef K2
	    AIF_serdesRestore(hAif2Setup, serdes_blockb8_used, serdes_blockb4_used);
#endif
		Aif2Fl_hwSetup(hAif2Csl, hAif2Setup);

		ctrlArg = true;
		for(i= 0; i< 6; i++)
		{
			if(hAif2Setup->globalSetup->ActiveLink[i] == true)
			{
				hAif2Csl->arg_link = (Aif2Fl_LinkIndex)i;
#ifndef K2
				if (( serdes_blockb8_used ==1)&& (i<4)) while(response == 0)
				{ // wait until SD PLL is locked
					Aif2Fl_getHwStatus(hAif2Csl, AIF2FL_QUERY_SD_B8_PLL_LOCK, (void *)&response);
				}
				response = 0;
				if (( serdes_blockb4_used ==1)&& (i>=4)) while(response == 0)
				{
					Aif2Fl_getHwStatus(hAif2Csl, AIF2FL_QUERY_SD_B4_PLL_LOCK, (void *)&response);
				}
				ctrlArg = false;
				Aif2Fl_hwControl(hAif2Csl, AIF2FL_CMD_ENABLE_DISABLE_LINK_LOOPBACK, (void *)&ctrlArg);
#endif
				ctrlArg = true;
				Aif2Fl_hwControl(hAif2Csl, AIF2FL_CMD_ENABLE_DISABLE_TX_LINK, (void *)&ctrlArg);
				Aif2Fl_hwControl(hAif2Csl, AIF2FL_CMD_ENABLE_DISABLE_RX_LINK, (void *)&ctrlArg);
			}
		}

		ctrlArg = true;
		//AT Arm timer
		Aif2Fl_hwControl(hAif2Csl, AIF2FL_CMD_AT_ARM_TIMER, (void *)&ctrlArg);
	}
}


#ifdef K2

#define AIF2FL_SERDES_B4_CFG_REGS             (0x02324000)
#define AIF2FL_SERDES_B8_CFG_REGS             (0x02326000)
#define AIF2FL_CONTROL_REGS  					(0x01F00000)

static void AIF_serdesRestore(
		Aif2Fl_Setup *hAif2Setup,
		uint8_t enableB8,
		uint8_t enableB4
)
{
	uint32_t retval, i;
	CSL_SERDES_REF_CLOCK refClock;
	CSL_SERDES_LINK_RATE serdesRate;
	CSL_SERDES_LOOPBACK  loopback;
	Aif2Fl_LinkProtocol protocol = AIF2FL_LINK_PROTOCOL_CPRI;

	refClock =  CSL_SERDES_REF_CLOCK_122p88M;

	for(i=AIF2FL_LINK_0; i<AIF2FL_LINK_5+1; i++)
	{
		if(hAif2Setup->globalSetup->ActiveLink[i] == true) {
			protocol = hAif2Setup->linkSetup[i]->pComLinkSetup->linkProtocol;
			break;
		}
	}

	if(AIF2FL_LINK_PROTOCOL_OBSAI==protocol) //used for OBSAI 2x,4x,8x and CPRI 5x
	{
		serdesRate = CSL_SERDES_LINK_RATE_6p144G;
	} else { //used for CPRI 2x,4x,8x
		serdesRate = CSL_SERDES_LINK_RATE_4p9152G;
	}

	if (enableB8)
	{
		CSL_AIF2SerdesInitB8(AIF2FL_SERDES_B8_CFG_REGS, refClock, serdesRate);
		for(i=0; i < 4; i++)
		{
			if(1==hAif2Setup->globalSetup->ActiveLink[i]) {
				CSL_AIF2SerdesLaneConfig(AIF2FL_SERDES_B8_CFG_REGS, refClock, serdesRate, i);
			}
		}
		CSL_AIF2SerdesComEnable(AIF2FL_SERDES_B8_CFG_REGS);
	}

	if (enableB4)
	{
		CSL_AIF2SerdesInitB4(AIF2FL_SERDES_B4_CFG_REGS, refClock, serdesRate);
		for(i=0; i < 2; i++)
		{
			if(1==hAif2Setup->globalSetup->ActiveLink[AIF2FL_LINK_4 + i]) {
				CSL_AIF2SerdesLaneConfig(AIF2FL_SERDES_B4_CFG_REGS, refClock, serdesRate, i);
			}
		}
		CSL_AIF2SerdesComEnable(AIF2FL_SERDES_B4_CFG_REGS);
	}

	//AIF2 Lane Enable
	for(i=0; i < 6; i++)
	{
		if(1==hAif2Setup->globalSetup->ActiveLink[i]) {
			loopback = CSL_SERDES_LOOPBACK_DISABLED;
			if(hAif2Setup->linkSetup[i]->pComLinkSetup->linkRate == AIF2FL_LINK_RATE_8x) {
				if (i < (uint32_t)AIF2FL_LINK_4) {
					CSL_AIF2SerdesLaneEnable(AIF2FL_CONTROL_REGS, AIF2FL_SERDES_B8_CFG_REGS, i, AIF2FL_LINK_RATE_8x);
					CSL_AIF2SerdesLoopbackEnable(AIF2FL_SERDES_B8_CFG_REGS, i, loopback);
				} else {
					CSL_AIF2SerdesLaneEnable(AIF2FL_CONTROL_REGS, AIF2FL_SERDES_B4_CFG_REGS, i, AIF2FL_LINK_RATE_8x);
					CSL_AIF2SerdesLoopbackEnable(AIF2FL_SERDES_B4_CFG_REGS, i - AIF2FL_LINK_4, loopback);
				}
			} else {
				if (i < (uint32_t)AIF2FL_LINK_4) {
					CSL_AIF2SerdesLaneEnable(AIF2FL_CONTROL_REGS, AIF2FL_SERDES_B8_CFG_REGS, i, AIF2FL_LINK_RATE_4x);
					CSL_AIF2SerdesLoopbackEnable(AIF2FL_SERDES_B8_CFG_REGS, i, loopback);
				} else {
					CSL_AIF2SerdesLaneEnable(AIF2FL_CONTROL_REGS, AIF2FL_SERDES_B4_CFG_REGS, i, AIF2FL_LINK_RATE_4x);
					CSL_AIF2SerdesLoopbackEnable(AIF2FL_SERDES_B4_CFG_REGS, i - AIF2FL_LINK_4, loopback);
				}
			}

		}
	}
	if (enableB8)
	{
		//AIF2 B8 PLL Enable
        CSL_AIF2SerdesPllEnableB8(AIF2FL_CONTROL_REGS,AIF2FL_SERDES_B8_CFG_REGS);

        //AIF2 PLL Status Poll
		retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
		while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
		{
			retval = CSL_AIF2SerdesGetStatusB8(AIF2FL_CONTROL_REGS);
		}
	}
	if (enableB4)
	{
		//AIF2 B8 PLL Enable
        CSL_AIF2SerdesPllEnableB4(AIF2FL_CONTROL_REGS,AIF2FL_SERDES_B4_CFG_REGS);

		 //AIF2 PLL Status Poll
		retval = CSL_SERDES_STATUS_PLL_NOT_LOCKED;
		while(retval == CSL_SERDES_STATUS_PLL_NOT_LOCKED)
		{
			retval = CSL_AIF2SerdesGetStatusB4(AIF2FL_CONTROL_REGS);
		}

	}

	/* The TX byte clock from either B8 or B4 SERDES link will be selected as sys_clk once the PLL has acquired lock.*/
	if ((enableB8 == 0) && (enableB4))
		CSL_AIF2SerdesClkSelB4(AIF2FL_CONTROL_REGS);

    //AIF2 Links Clock Enable
     for(i=0; i < 6; i++)
     {
		if(1==hAif2Setup->globalSetup->ActiveLink[i]) {
			CSL_AIF2LinkClkEnable(AIF2FL_CONTROL_REGS, i);
		} else {
			CSL_AIF2LinkClkDisable(AIF2FL_CONTROL_REGS, i);
		}
     }
}

#endif


/////////////////////
