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

#include <ti/drv/aif2/aif2.h>
#include <ti/drv/aif2/aif2_osal.h>

#define __AIF_CFG_C
#include <ti/drv/aif2/AIF_cfg.h>

#ifdef _TMS320C6X
#pragma CODE_SECTION(AIF_getCfg, ".text:aifDriver");
#pragma CODE_SECTION(AIF_populateAifObj, ".text:aifDriver");
#pragma CODE_SECTION(AIF_startCfg, ".text:aifDriver");
#endif

AIF_CoreCfgObj*
AIF_getCfg
(
	AIF_CfgHandle 		ptrInitCfg,
    uint32_t            maxCfgBlock, 
    uint32_t            core
)
{
	AIF_CoreCfgHandle 	ptrCfg;
    uint32_t             coreIndex;

    /* Get the initial configuration. */
    ptrCfg = ptrInitCfg->coreCfg;

    /* Cycle through all the configuration blocks. */
    for (coreIndex = 0; coreIndex < maxCfgBlock; coreIndex++)
    {
        /* Check if this is the configuration which is requested? */
        if (coreIndex == core)
            return ptrCfg;

        /* Goto the next configuration block */
        ptrCfg++;
    }
    /* Error: We dont have a valid configuration */
    return NULL;
}


void
AIF_populateAifObj(
		AIF_ConfigHandle	hAifObj,
		AIF_CfgHandle 	 	hAifCfgObj,
		uint32_t 				clockspeed,
		uint32_t 				swSync,
		uint32_t 				intLoopback,
		uint32_t 				lte_bw
)
{
	uint32_t i, j, k;

	// General parameters
	hAifObj->aif2ClkSpeedKhz    = clockspeed;
	hAifObj->protocol           = hAifCfgObj->protocol; //AIF2FL_LINK_PROTOCOL_CPRI;
	hAifObj->pktdmaOrDioEngine  = hAifCfgObj->pktdmaOrDioEngine;  //AIF2FL_CPPI;
	hAifObj->mode               = hAifCfgObj->mode;  //AIF_LTE_FDD_MODE;

	Aif2_osalLog("test: %s\n", hAifCfgObj->name);
	// proceed with HW cleanup but may require GEL_AdvancedReset("System Reset") on PreFileLoaded callback for 100% robustness


	for (i=0;i<AIF_MAX_NUM_LINKS;i++)
	{
		for (j=0;j<AIF_CFG_MAX_CORE;j++)
		{
			if (hAifCfgObj->coreCfg[j].linkEnable[i] == 1)
			{
				 hAifObj->linkConfig[i].linkEnable         = 1; //testObjTab[ntest].linkEnable[i];
				 hAifObj->linkConfig[i].numPeAxC			 += hAifCfgObj->coreCfg[j].numAxC[i];
				 hAifObj->linkConfig[i].numPdAxC			 += hAifCfgObj->coreCfg[j].numAxC[i];
				 hAifObj->linkConfig[i].linkRate           = (Aif2Fl_LinkRate)hAifCfgObj->coreCfg[j].linkRate[i];  //(Aif2Fl_LinkRate)testObjTab[ntest].linkRate[i];
				 if (lte_bw == 20)
				 {
					 hAifObj->linkConfig[i].sampleRate		 = AIF_SRATE_30P72MHZ;
					 hAifObj->linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
				 } else if (lte_bw == 10) {
					 hAifObj->linkConfig[i].sampleRate		 = AIF_SRATE_15P36MHZ;
					 hAifObj->linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
				 } else if (lte_bw == 5) {
					 hAifObj->linkConfig[i].sampleRate		 = AIF_SRATE_7P68MHZ;
					 hAifObj->linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
				 } else if (lte_bw == 1) {
					 hAifObj->linkConfig[i].sampleRate		 = AIF_SRATE_1P92MHZ;
					 hAifObj->linkConfig[i].cpriPackMode       = AIF2_LTE_CPRI_1b1;
				 } else {
					 hAifObj->linkConfig[i].sampleRate		 = AIF_SRATE_3P84MHZ; //if no lte_bw is set, take WCDMA sample rate
				 }
				 hAifObj->linkConfig[i].outboundDataType   = hAifCfgObj->coreCfg[j].outboundDataType[i];  //testObjTab[ntest].outboundDataType[i];
				 hAifObj->linkConfig[i].outboundDataWidth  = hAifCfgObj->coreCfg[j].outboundDataWidth[i]; //testObjTab[ntest].outboundDataWidth[i];
				 hAifObj->linkConfig[i].inboundDataType    = hAifCfgObj->coreCfg[j].inboundDataType[i];   //testObjTab[ntest].inboundDataType[i];
				 hAifObj->linkConfig[i].inboundDataWidth   = hAifCfgObj->coreCfg[j].inboundDataWidth[i];  //testObjTab[ntest].inboundDataWidth[i];
				 hAifObj->linkConfig[i].psMsgEnable        = hAifCfgObj->coreCfg[j].CWEnable;
				 hAifObj->linkConfig[i].dioEngine          = hAifCfgObj->coreCfg[j].dioEngine[i];         //testObjTab[ntest].dioEngine[i]; //NA for pkDMA
				 if (intLoopback == 1 ) hAifObj->linkConfig[i].comType = AIF2_LOOPBACK;
				 else	 				hAifObj->linkConfig[i].comType = AIF2_2_AIF2;

				 for (k = hAifCfgObj->coreCfg[j].firstAxCIndex[i]; k < (hAifCfgObj->coreCfg[j].firstAxCIndex[i] + hAifCfgObj->coreCfg[j].numAxC[i]); k++)
				 {
					 hAifObj->AxCconfig[k].peAxCOffset     = hAifCfgObj->coreCfg[j].peAxCOffset;
					 hAifObj->AxCconfig[k].pdAxCOffset     = hAifCfgObj->coreCfg[j].pdAxCOffset;
					 hAifObj->AxCconfig[k].egressBufDepth  = hAifCfgObj->coreCfg[j].egressBufDepth;
					 hAifObj->AxCconfig[k].ingressBufDepth = hAifCfgObj->coreCfg[j].ingressBufDepth;
				 }
			}
		}
	}
	if (swSync == 0) {
		hAifObj->aif2TimerSyncSource= AIF2FL_PHYT_CMP_SYNC;
		hAifObj->autoResyncMode = AIF2FL_AUTO_RESYNC_MODE;
		hAifObj->phytCompValue = 0;
	}
	else {
		hAifObj->aif2TimerSyncSource= AIF2FL_SW_SYNC;
		hAifObj->autoResyncMode = AIF2FL_NO_AUTO_RESYNC_MODE;
	}

	return;
}

void
AIF_startCfg()
{
	Aif2_osalMulticoreSyncBarrier();
	return;
}

////////////////////

